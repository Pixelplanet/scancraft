//! Watertight solid of a thin-walled part from oriented scan points.
//!
//! Input: oriented points of every scan pass merged into one frame, normals pointing out of the
//! material (towards the scanner that saw them), each tagged with its scan id. Both faces of the
//! wall may be present (outside scan + inside scans), or only one of them in places. Output: one
//! closed, manifold triangle mesh of the wall (outer face, inner face and the lip that joins them
//! at an open rim), with every vertex tagged by where its surface comes from ([`VertexOrigin`]).
//!
//! Stages of [`build_solid`]:
//! 1. **Clean:** isolated points and points standing off their local sheet are dropped. Where
//!    scans of different trust cover the same sheet only the most trusted layer is kept
//!    ([`SolidConfig::scan_priority`]). A wall thickness map is measured from the points (depth of
//!    the opposite-facing sheet behind each point); where a sheet is doubled (ghost layer of a
//!    misregistered pass) the layer that disagrees with the thickness map is dropped.
//! 2. **Markers** ([`remove_markers`]): sites come from a list (tracker markers) and/or from the
//!    detectors (raised flat disc, round hole); copies of one sticker are merged. Each site is cut
//!    out of the points and refilled from a quadric fitted to the surrounding annulus.
//! 3. **Completion:** where one face of the wall was not seen, synthetic points one local wall
//!    thickness behind the seen face stand in for it. Enclosed holes of a sheet are then patched
//!    ([`fill_holes`]) and completed in turn.
//! 4. **Solve:** screened Poisson for an indicator χ on a sparse band of 8³ node blocks around
//!    the samples, χ = 0 outside the band, so the surface is closed by construction. Measured
//!    samples keep a narrow normal kernel (sharp edges), synthetic ones a wider one.
//! 5. **Extract and finish:** welded marching cubes at χ = ½; fragments and void shells are
//!    dropped, sliver edges collapsed, marker sites faired on the mesh, completed surface
//!    smoothed, and any remaining self-intersection or fold relaxed locally.

mod cleanup;
mod extract;
pub(crate) mod grid;
mod holes;
pub mod markers;
mod poisson;
pub(crate) mod prep;
pub mod topo;
#[cfg(test)]
mod tests;

pub use holes::HoleConfig;
pub use markers::{DetectConfig, FairOutcome, Marker, MarkerConfig, MarkerOutcome, MarkerSource};
pub use prep::CompletionConfig;
pub use topo::{ComponentStats, TopologyStats};

use glam::Vec3;
use rayon::prelude::*;
use scancraft_core::{ColorRgba, Mesh, Point3D, Triangle};
use std::time::Instant;

/// One oriented input point.
#[derive(Clone, Copy, Debug)]
pub struct SolidPoint {
    pub position: Vec3,
    /// Unit normal pointing out of the material.
    pub normal: Vec3,
    /// Scan / pass id (0 when unknown).
    pub scan: u8,
}

/// Origin of a working sample.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PointKind {
    Measured,
    MarkerFill,
    Completion,
    HoleFill,
}

/// An oriented sample with its origin (output of [`remove_markers`], input and output of
/// [`fill_holes`]).
#[derive(Clone, Copy, Debug)]
pub struct Sample {
    pub position: Vec3,
    pub normal: Vec3,
    pub scan: u8,
    pub kind: PointKind,
}

/// Where the surface at a mesh vertex comes from.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum VertexOrigin {
    /// Within reach of a measured point.
    Measured = 0,
    /// Marker site: infill patch or faired mesh inside the cut.
    MarkerFill = 1,
    /// Patch over an enclosed hole of a sheet.
    HoleFill = 2,
    /// Completed: the unseen face of the wall, the lip at an open rim, or surface the solve
    /// spanned on its own.
    Completed = 3,
}

impl VertexOrigin {
    /// Colour written to the mesh vertices.
    pub fn color(self) -> ColorRgba {
        match self {
            VertexOrigin::Measured => ColorRgba::new(205, 208, 214, 255),
            VertexOrigin::MarkerFill => ColorRgba::new(70, 130, 230, 255),
            VertexOrigin::HoleFill => ColorRgba::new(60, 190, 110, 255),
            VertexOrigin::Completed => ColorRgba::new(240, 150, 60, 255),
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct WorkPoint {
    pub p: Vec3,
    pub n: Vec3,
    pub scan: u8,
    pub kind: PointKind,
    /// Surface area represented by the sample (mm²), set before the solve.
    pub area: f32,
}

impl WorkPoint {
    fn from_sample(s: &Sample) -> Self {
        Self { p: s.position, n: s.normal.normalize_or_zero(), scan: s.scan, kind: s.kind, area: 0.0 }
    }
    fn sample(&self) -> Sample {
        Sample { position: self.p, normal: self.n, scan: self.scan, kind: self.kind }
    }
}

fn measured(input: &[SolidPoint]) -> Vec<WorkPoint> {
    input
        .iter()
        .filter_map(|p| {
            let n = p.normal.normalize_or_zero();
            (p.position.is_finite() && n != Vec3::ZERO)
                .then_some(WorkPoint { p: p.position, n, scan: p.scan, kind: PointKind::Measured, area: 0.0 })
        })
        .collect()
}

#[derive(Clone, Debug)]
pub struct SolidConfig {
    /// Grid spacing of the solve (mm). The wall should span at least 6 cells.
    pub cell_mm: f32,
    /// Band around the samples (mm); rounded up to whole 8-node blocks.
    pub band_mm: f32,
    /// Dimensionless screening weight (0 = plain Poisson), scaled per sample kind by the
    /// confidences below.
    pub screen: f32,
    pub measured_confidence: f32,
    /// Confidence of the measured samples of the `sharp_scans` (dense, clean data that may be
    /// pinned harder than noisy passes).
    pub sharp_confidence: f32,
    pub marker_fill_confidence: f32,
    pub hole_fill_confidence: f32,
    pub completion_confidence: f32,
    /// [¼ ½ ¼] passes per axis on the divergence (normal kernel width) of the measured samples
    /// of the `sharp_scans`; a fractional part is a partial pass. Fewer passes keep edges sharp but leave sparse or noisy data with
    /// gaps in the normal field, which open slits in thin plates.
    pub smooth_sharp: f32,
    /// The same for the other measured samples and the marker fill.
    pub smooth_measured: f32,
    /// The same for completion and hole-fill samples (no detail to keep). The counts must not
    /// increase from soft to sharp.
    pub smooth_soft: f32,
    /// Scans dense and clean enough for the narrow kernel (e.g. a fused outside scan); empty =
    /// none. Measured vertices near their points are also refined onto them (`refine_radius_mm`).
    pub sharp_scans: Vec<u8>,
    /// Radius of the projection of measured vertices onto the sharp scans' points (mm); 0 = off.
    pub refine_radius_mm: f32,
    /// Neighbourhood radius for the area weights (mm).
    pub area_radius_mm: f32,
    pub outlier_radius_mm: f32,
    pub outlier_min_neighbours: usize,
    /// Drop points standing off their local sheet by more than max(`off_surface_mm`,
    /// 4 × local RMS); 0 disables.
    pub off_surface_mm: f32,
    /// Scan ids from the most trusted down. Where a better scan covers a sheet, the points of
    /// worse scans on that sheet are dropped (one layer per location); scans not listed share the
    /// lowest rank and are averaged. Empty: all scans are averaged.
    pub scan_priority: Vec<u8>,
    /// Neighbourhood radius and tangent-plane tolerance of the layer test (mm).
    pub layer_radius_mm: f32,
    pub layer_tolerance_mm: f32,
    /// Wall thickness prior; `None` = measured median.
    pub wall_mm: Option<f32>,
    /// Largest thickness searched when measuring the wall.
    pub wall_search_mm: f32,
    pub ghost_filter: bool,
    /// Drop laser-line strays: points about a millimetre behind the surface their own scan sees in
    /// front of them, with an empty gap between (`prep::drop_behind_strays`).
    pub laser_strays: bool,
    /// Move each scan back along its normals by its systematic offset from the other scans where
    /// they overlap (a depth bias of a pass; see `prep::equalise_scan_offsets`).
    pub equalise_offsets: bool,
    /// Where another scan covers a sheet at most this times as rough, the rougher scan's points
    /// there are dropped (the streaked grazing fringe of an inside scan on the outer face; see
    /// `prep::drop_rough_layers`). 0 disables.
    pub rough_layer_ratio: f32,
    /// Feature-preserving smoothing of the extracted surface ([`crate::FeatureSmoother`]
    /// iterations; 0 = off): removes the scanners' fine hatching and the solve's dimple grain from
    /// plain areas, keeps edges, ribs and bores. Runs before the marker fairing.
    pub smooth_surface: usize,
    /// Crease-keeping scale of that smoothing (the app's "keep edges" 0.7).
    pub smooth_surface_sigma: f32,
    pub completion: bool,
    pub completion_cfg: CompletionConfig,
    /// Fill enclosed holes of a sheet (then complete the opposite face behind them).
    pub hole_fill: Option<HoleConfig>,
    pub markers: MarkerConfig,
    /// Run the detectors (in addition to the given marker list).
    pub detect: Option<DetectConfig>,
    /// Detections within this distance of a listed marker are dropped as duplicates.
    pub detect_dedupe_mm: f32,
    pub cg_tol: f64,
    pub cg_max_iter: usize,
    pub coarse_start: bool,
    pub iso: f32,
    pub min_component_fraction: f32,
    /// Edges shorter than this fraction of the cell are collapsed after extraction.
    pub sliver_edge_frac: f32,
    /// Rounds of local relaxation of self-intersections and folds (0 = only count them).
    pub repair_rounds: usize,
    /// Taubin iterations on completed and hole-fill vertices (measured surface stays as solved).
    pub smooth_completed: usize,
    /// A vertex counts as measured when a measured sample lies within this distance (mm).
    pub origin_radius_mm: f32,
    /// Return the grid nodes inside the material (diagnostics, e.g. local Euler characteristic).
    pub keep_inside_nodes: bool,
}

impl Default for SolidConfig {
    fn default() -> Self {
        Self {
            cell_mm: 0.4,
            band_mm: 3.0,
            screen: 0.3,
            measured_confidence: 1.0,
            sharp_confidence: 3.0,
            marker_fill_confidence: 1.0,
            hole_fill_confidence: 0.5,
            completion_confidence: 0.3,
            smooth_sharp: 1.0,
            smooth_measured: 2.0,
            smooth_soft: 3.0,
            sharp_scans: Vec::new(),
            refine_radius_mm: 0.8,
            area_radius_mm: 1.0,
            outlier_radius_mm: 1.0,
            outlier_min_neighbours: 2,
            off_surface_mm: 0.15,
            scan_priority: Vec::new(),
            layer_radius_mm: 1.0,
            layer_tolerance_mm: 0.5,
            wall_mm: None,
            wall_search_mm: 6.0,
            ghost_filter: true,
            laser_strays: true,
            equalise_offsets: true,
            rough_layer_ratio: 0.4,
            smooth_surface: 4,
            smooth_surface_sigma: 0.5144,
            completion: true,
            completion_cfg: CompletionConfig::default(),
            hole_fill: Some(HoleConfig::default()),
            markers: MarkerConfig::default(),
            detect: Some(DetectConfig::default()),
            detect_dedupe_mm: 9.0,
            cg_tol: 1e-4,
            cg_max_iter: 3000,
            coarse_start: true,
            iso: 0.5,
            min_component_fraction: 0.002,
            sliver_edge_frac: 0.1,
            repair_rounds: 4,
            smooth_completed: 4,
            origin_radius_mm: 0.6,
            keep_inside_nodes: false,
        }
    }
}

#[derive(Clone, Debug, Default)]
pub struct SolidReport {
    /// (stage, seconds)
    pub stages: Vec<(String, f64)>,
    pub input_points: usize,
    pub dropped_outliers: usize,
    pub dropped_off_surface: usize,
    pub dropped_covered_layers: usize,
    pub dropped_ghosts: usize,
    pub dropped_laser_strays: usize,
    /// Shift applied per scan along its normals (mm) by the offset equalisation.
    pub scan_shifts: Vec<(u8, f32)>,
    pub dropped_rough_layers: usize,
    pub wall_median_mm: f32,
    pub wall_measurements: usize,
    pub markers_listed: usize,
    pub markers_detected: Vec<Marker>,
    /// Sites after merging copies of the same sticker.
    pub marker_sites: Vec<Marker>,
    pub marker_outcomes: Vec<MarkerOutcome>,
    pub marker_cut_points: usize,
    pub marker_fill_points: usize,
    pub completion_points: usize,
    /// Separate hole patches (see [`HoleFilling::holes`]).
    pub holes_filled: usize,
    pub hole_fill_points: usize,
    pub completion_points_2: usize,
    pub samples: usize,
    pub band_blocks: usize,
    pub unknowns: usize,
    pub cg_iterations: usize,
    pub cg_coarse_iterations: usize,
    pub cg_rel_residual: f64,
    /// Area-weighted mean of χ at the measured samples (PSR's iso choice), for reference.
    pub mean_chi_measured: f32,
    pub iso: f32,
    pub raw_triangles: usize,
    pub dropped_fragments: usize,
    pub dropped_voids: usize,
    pub sliver_collapses: usize,
    /// Measured vertices projected onto the sharp scans' points.
    pub refined_vertices: usize,
    pub sites_faired: usize,
    /// (self-intersecting pairs, folded edges) before and after the local repair.
    pub defects_before: (usize, usize),
    pub defects_after: (usize, usize),
    /// Vertex counts per [`VertexOrigin`] (measured, marker fill, hole fill, completed).
    pub origin_counts: [usize; 4],
    pub topology: TopologyStats,
}

impl SolidReport {
    pub fn total_seconds(&self) -> f64 {
        self.stages.iter().map(|s| s.1).sum()
    }

    /// Share of vertices per [`VertexOrigin`].
    pub fn origin_fractions(&self) -> [f32; 4] {
        let n = self.origin_counts.iter().sum::<usize>().max(1) as f32;
        self.origin_counts.map(|c| c as f32 / n)
    }

    /// Plain JSON (no serde_json dependency in this crate).
    pub fn to_json(&self) -> String {
        fn num(x: f32) -> String {
            if x.is_finite() {
                format!("{x:.4}")
            } else {
                "null".into()
            }
        }
        fn esc(s: &str) -> String {
            s.replace('\\', "\\\\").replace('"', "\\\"")
        }
        let mut s = String::from("{\n");
        let kv = |s: &mut String, k: &str, v: String| s.push_str(&format!("  \"{k}\": {v},\n"));
        kv(&mut s, "stages_s", format!("{{{}}}", self.stages.iter().map(|(k, v)| format!("\"{k}\": {v:.3}")).collect::<Vec<_>>().join(", ")));
        kv(&mut s, "total_s", format!("{:.3}", self.total_seconds()));
        kv(&mut s, "input_points", self.input_points.to_string());
        kv(&mut s, "dropped_outliers", self.dropped_outliers.to_string());
        kv(&mut s, "dropped_off_surface", self.dropped_off_surface.to_string());
        kv(&mut s, "dropped_covered_layers", self.dropped_covered_layers.to_string());
        kv(&mut s, "dropped_ghosts", self.dropped_ghosts.to_string());
        kv(&mut s, "dropped_laser_strays", self.dropped_laser_strays.to_string());
        kv(&mut s, "scan_shifts_mm", format!("{{{}}}", self.scan_shifts.iter().map(|(k, v)| format!("\"{k}\": {v:.3}")).collect::<Vec<_>>().join(", ")));
        kv(&mut s, "dropped_rough_layers", self.dropped_rough_layers.to_string());
        kv(&mut s, "wall_median_mm", format!("{:.3}", self.wall_median_mm));
        kv(&mut s, "wall_measurements", self.wall_measurements.to_string());
        kv(&mut s, "markers_listed", self.markers_listed.to_string());
        let marker = |m: &Marker| {
            format!(
                "{{\"id\": \"{}\", \"scan\": {}, \"source\": \"{:?}\", \"x\": {:.2}, \"y\": {:.2}, \"z\": {:.2}, \"nx\": {:.4}, \"ny\": {:.4}, \"nz\": {:.4}, \"radius_mm\": {:.2}, \"score\": {:.2}}}",
                esc(&m.id),
                m.scan.map_or(-1, |s| s as i32),
                m.source,
                m.center.x,
                m.center.y,
                m.center.z,
                m.normal.x,
                m.normal.y,
                m.normal.z,
                m.radius_mm,
                m.score
            )
        };
        kv(&mut s, "markers_detected", format!("[{}]", self.markers_detected.iter().map(marker).collect::<Vec<_>>().join(",\n    ")));
        kv(&mut s, "marker_sites", format!("[{}]", self.marker_sites.iter().map(marker).collect::<Vec<_>>().join(",\n    ")));
        kv(
            &mut s,
            "marker_outcomes",
            format!(
                "[{}]",
                self.marker_outcomes
                    .iter()
                    .map(|o| {
                        let fair = o.fair.as_ref().map_or("null".to_string(), |f| {
                            format!(
                                "{{\"status\": \"{}\", \"ring_rms_mm\": {}, \"mean_before_mm\": {}, \"max_before_mm\": {}, \"vertices\": {}}}",
                                esc(&f.status),
                                num(f.ring_rms_mm),
                                num(f.mean_before_mm),
                                num(f.max_before_mm),
                                f.vertices
                            )
                        });
                        format!(
                            "{{\"id\": \"{}\", \"x\": {:.2}, \"y\": {:.2}, \"z\": {:.2}, \"cut\": {}, \"annulus\": {}, \"annulus_rms_mm\": {}, \"sectors\": {}, \"fill_points\": {}, \"fill\": \"{}\", \"fair\": {}}}",
                            esc(&o.id),
                            o.center.x,
                            o.center.y,
                            o.center.z,
                            o.cut_points,
                            o.annulus_points,
                            num(o.annulus_rms_mm),
                            o.sectors_covered,
                            o.fill_points,
                            esc(&o.fill),
                            fair
                        )
                    })
                    .collect::<Vec<_>>()
                    .join(",\n    ")
            ),
        );
        kv(&mut s, "marker_cut_points", self.marker_cut_points.to_string());
        kv(&mut s, "marker_fill_points", self.marker_fill_points.to_string());
        kv(&mut s, "completion_points", self.completion_points.to_string());
        kv(&mut s, "holes_filled", self.holes_filled.to_string());
        kv(&mut s, "hole_fill_points", self.hole_fill_points.to_string());
        kv(&mut s, "completion_points_2", self.completion_points_2.to_string());
        kv(&mut s, "samples", self.samples.to_string());
        kv(&mut s, "band_blocks", self.band_blocks.to_string());
        kv(&mut s, "unknowns", self.unknowns.to_string());
        kv(&mut s, "cg_iterations", self.cg_iterations.to_string());
        kv(&mut s, "cg_coarse_iterations", self.cg_coarse_iterations.to_string());
        kv(&mut s, "cg_rel_residual", format!("{:.3e}", self.cg_rel_residual));
        kv(&mut s, "mean_chi_measured", format!("{:.4}", self.mean_chi_measured));
        kv(&mut s, "iso", format!("{:.4}", self.iso));
        kv(&mut s, "raw_triangles", self.raw_triangles.to_string());
        kv(&mut s, "dropped_fragments", self.dropped_fragments.to_string());
        kv(&mut s, "dropped_voids", self.dropped_voids.to_string());
        kv(&mut s, "sliver_collapses", self.sliver_collapses.to_string());
        kv(&mut s, "refined_vertices", self.refined_vertices.to_string());
        kv(&mut s, "sites_faired", self.sites_faired.to_string());
        kv(&mut s, "self_intersections_before", self.defects_before.0.to_string());
        kv(&mut s, "folded_edges_before", self.defects_before.1.to_string());
        kv(&mut s, "self_intersections_after", self.defects_after.0.to_string());
        kv(&mut s, "folded_edges_after", self.defects_after.1.to_string());
        let f = self.origin_fractions();
        kv(
            &mut s,
            "vertex_origin",
            format!(
                "{{\"measured\": {}, \"marker_fill\": {}, \"hole_fill\": {}, \"completed\": {}, \"fractions\": [{:.4}, {:.4}, {:.4}, {:.4}]}}",
                self.origin_counts[0], self.origin_counts[1], self.origin_counts[2], self.origin_counts[3], f[0], f[1], f[2], f[3]
            ),
        );
        let t = &self.topology;
        kv(
            &mut s,
            "topology",
            format!(
                "{{\"vertices\": {}, \"triangles\": {}, \"watertight\": {}, \"manifold\": {}, \"boundary_edges\": {}, \"boundary_loops\": {}, \"non_manifold_edges\": {}, \"non_manifold_vertices\": {}, \"inconsistent_edges\": {}, \"degenerate_triangles\": {}, \"components\": {}, \"genus_total\": {}, \"volume_mm3\": {:.1}, \"area_mm2\": {:.1}}}",
                t.vertices,
                t.triangles,
                t.watertight,
                t.manifold,
                t.boundary_edges,
                t.boundary_loops,
                t.non_manifold_edges,
                t.non_manifold_vertices,
                t.inconsistent_edges,
                t.degenerate_triangles,
                t.components.len(),
                t.genus_total(),
                t.total_volume_mm3,
                t.total_area_mm2,
            ),
        );
        s.truncate(s.len() - 2);
        s.push_str("\n}\n");
        s
    }
}

/// Result of [`build_solid`].
pub struct Solid {
    /// Closed manifold mesh, vertex colours by [`VertexOrigin`].
    pub mesh: Mesh,
    /// Origin of every mesh vertex.
    pub vertex_origin: Vec<VertexOrigin>,
    pub report: SolidReport,
    /// The samples that went into the solve.
    pub samples: Vec<Sample>,
    /// With `keep_inside_nodes`: grid origin, cell size and the integer coordinates of the nodes
    /// with χ > iso.
    pub inside_nodes: Option<(Vec3, f32, Vec<[i32; 3]>)>,
}

/// Result of [`remove_markers`].
pub struct MarkerRemoval {
    /// The input points without the marker sites, plus the infill patches.
    pub samples: Vec<Sample>,
    /// The sites that were cut (listed plus detected, merged).
    pub sites: Vec<Marker>,
    pub detected: Vec<Marker>,
    pub outcomes: Vec<MarkerOutcome>,
}

/// Marker removal on its own: re-orients the listed sites from the data, optionally adds the
/// detector's finds, merges copies of one sticker, cuts every site out of the points and refills
/// it with a quadric patch fitted to the annulus around it.
pub fn remove_markers(points: &[SolidPoint], markers: &[Marker], cfg: &MarkerConfig, detect: Option<&DetectConfig>) -> MarkerRemoval {
    let mut pts = measured(points);
    let (sites, detected, outcomes) = marker_stage(&mut pts, markers, cfg, detect, 9.0);
    MarkerRemoval { samples: pts.iter().map(WorkPoint::sample).collect(), sites, detected, outcomes }
}

pub(crate) fn marker_stage(
    pts: &mut Vec<WorkPoint>,
    markers: &[Marker],
    cfg: &MarkerConfig,
    detect: Option<&DetectConfig>,
    dedupe_mm: f32,
) -> (Vec<Marker>, Vec<Marker>, Vec<MarkerOutcome>) {
    let mut sites: Vec<Marker> = markers.to_vec();
    {
        let pos: Vec<Vec3> = pts.iter().map(|p| p.p).collect();
        let grid = grid::PointGrid::new(&pos, 2.0);
        markers::orient_markers(&mut sites, pts, &pos, &grid, cfg);
    }
    let mut detected = Vec::new();
    if let Some(dp) = detect {
        detected = markers::detect(pts, dp)
            .into_iter()
            .filter(|d| !sites.iter().any(|m| m.scan.map_or(true, |s| Some(s) == d.scan) && m.center.distance(d.center) < dedupe_mm))
            .collect::<Vec<_>>();
        sites.extend(detected.iter().cloned());
    }
    let mut sites = markers::merge_sites(&sites, cfg);
    markers::widen_over_sparse_rings(pts, &mut sites, cfg);
    let outcomes = markers::cut_and_fill(pts, &sites, cfg);
    (sites, detected, outcomes)
}

/// Mesh-level marker fairing on its own (any closed or open mesh): see [`markers::MarkerConfig`]
/// `fair_*`. Returns one outcome per site.
pub fn fair_marker_sites(mesh: &mut Mesh, sites: &[Marker], cfg: &MarkerConfig) -> Vec<FairOutcome> {
    let mut pos: Vec<Vec3> = mesh.vertices.iter().map(|v| Vec3::from_array(v.position)).collect();
    let tris: Vec<[u32; 3]> = mesh.triangles.iter().map(|t| t.indices).collect();
    let vn = vertex_normals(&pos, &tris);
    let mut inside = vec![false; pos.len()];
    let out = markers::fair_sites(&mut pos, &vn, sites, cfg, &mut inside);
    for (v, p) in mesh.vertices.iter_mut().zip(&pos) {
        v.position = p.to_array();
    }
    mesh.recompute_normals();
    out
}

/// Result of [`fill_holes`].
pub struct HoleFilling {
    /// The new patch points (kind [`PointKind::HoleFill`]).
    pub fill: Vec<Sample>,
    /// Number of separate patches (connected groups of fill points; a hole seen from several
    /// seeds or windows counts once, two holes whose patches touch count as one).
    pub holes: usize,
}

/// Hole filling on its own: patches the enclosed holes of every sheet in `samples` (see
/// [`HoleConfig`]). Holes open to the edge of the data (a rim) stay open.
pub fn fill_holes(samples: &[Sample], cfg: &HoleConfig) -> HoleFilling {
    let pts: Vec<WorkPoint> = samples.iter().map(WorkPoint::from_sample).collect();
    let (fill, holes) = holes::fill_holes(&pts, cfg);
    HoleFilling { fill: fill.iter().map(WorkPoint::sample).collect(), holes }
}

/// Area-weighted vertex normals.
fn vertex_normals(pos: &[Vec3], tris: &[[u32; 3]]) -> Vec<Vec3> {
    let mut n = vec![Vec3::ZERO; pos.len()];
    for t in tris {
        let (a, b, c) = (pos[t[0] as usize], pos[t[1] as usize], pos[t[2] as usize]);
        let f = (b - a).cross(c - a);
        for &i in t {
            n[i as usize] += f;
        }
    }
    n.par_iter_mut().for_each(|v| *v = v.normalize_or_zero());
    n
}

/// Taubin λ|μ smoothing of the vertices in `mask` only (the others are fixed boundary values).
fn taubin_masked(pos: &mut [Vec3], tris: &[[u32; 3]], mask: &[bool], iterations: usize, lambda: f32) {
    if iterations == 0 || !mask.iter().any(|m| *m) {
        return;
    }
    let n = pos.len();
    let mut edges: Vec<(u32, u32)> =
        tris.par_iter().flat_map_iter(|t| [(t[0], t[1]), (t[1], t[0]), (t[1], t[2]), (t[2], t[1]), (t[2], t[0]), (t[0], t[2])]).collect();
    edges.par_sort_unstable();
    edges.dedup();
    let mut start = vec![0u32; n + 1];
    for &(a, _) in &edges {
        start[a as usize + 1] += 1;
    }
    for i in 0..n {
        start[i + 1] += start[i];
    }
    let mu = -lambda / (1.0 - 0.1 * lambda);
    for _ in 0..iterations {
        for f in [lambda, mu] {
            let cur: &[Vec3] = pos;
            let upd: Vec<Vec3> = (0..n)
                .into_par_iter()
                .map(|i| {
                    let (s, e) = (start[i] as usize, start[i + 1] as usize);
                    if !mask[i] || e == s {
                        return cur[i];
                    }
                    let c = edges[s..e].iter().map(|&(_, j)| cur[j as usize]).sum::<Vec3>() / (e - s) as f32;
                    cur[i] + (c - cur[i]) * f
                })
                .collect();
            pos.copy_from_slice(&upd);
        }
    }
}

/// Origin per vertex: the kind of the nearest sample within `radius` (measured beats a
/// synthetic sample only when it is nearer), [`VertexOrigin::Completed`] when none is in reach.
fn vertex_origins(pos: &[Vec3], samples: &[WorkPoint], radius: f32) -> Vec<VertexOrigin> {
    let sp: Vec<Vec3> = samples.iter().map(|s| s.p).collect();
    let grid = grid::PointGrid::new(&sp, radius);
    pos.par_iter()
        .map(|&v| {
            let mut best = (f32::MAX, None);
            grid.for_each_within(&sp, v, radius, |i, d2| {
                if d2 < best.0 {
                    best = (d2, Some(samples[i as usize].kind));
                }
            });
            match best.1 {
                Some(PointKind::Measured) => VertexOrigin::Measured,
                Some(PointKind::MarkerFill) => VertexOrigin::MarkerFill,
                Some(PointKind::HoleFill) => VertexOrigin::HoleFill,
                Some(PointKind::Completion) | None => VertexOrigin::Completed,
            }
        })
        .collect()
}

/// Moves every measured vertex along its normal onto the local surface of the sharp scans'
/// points: the median offset of the points within `radius` whose normal agrees with the vertex
/// normal (within ~35°), when at least 6 such points are in reach and the offset is below
/// `max_mm`. This removes the grid terracing the narrow normal kernel leaves and brings edges back
/// towards the data; where normals vary (edges, corners) too few points agree and the vertex stays.
/// Returns the number of vertices moved.
fn refine_to_data(pos: &mut [Vec3], vn: &[Vec3], origin: &[VertexOrigin], data: &[WorkPoint], radius: f32, max_mm: f32) -> usize {
    if data.is_empty() {
        return 0;
    }
    let dp: Vec<Vec3> = data.iter().map(|p| p.p).collect();
    let grid = grid::PointGrid::new(&dp, radius);
    let moves: Vec<Option<Vec3>> = (0..pos.len())
        .into_par_iter()
        .map(|i| {
            if origin[i] != VertexOrigin::Measured || vn[i] == Vec3::ZERO {
                return None;
            }
            let (v, n) = (pos[i], vn[i]);
            let mut off: Vec<f32> = Vec::with_capacity(64);
            let mut nearest = (f32::MAX, 0u32);
            grid.for_each_within(&dp, v, radius, |j, d2| {
                if data[j as usize].n.dot(n) > 0.82 {
                    off.push((dp[j as usize] - v).dot(n));
                }
                if d2 < nearest.0 {
                    nearest = (d2, j);
                }
            });
            if off.len() >= 6 {
                let m = off.len() / 2;
                off.select_nth_unstable_by(m, |a, b| a.total_cmp(b));
                let d = off[m];
                return (d.abs() <= max_mm).then(|| v + n * d);
            }
            // Edge or corner: the vertex normal is a blend of the faces. Move onto the tangent
            // plane of the nearest face (nearest point and its like-facing neighbours), which
            // pulls a rounded edge back towards the data's edge.
            if nearest.0 == f32::MAX {
                return None;
            }
            let np = data[nearest.1 as usize].n;
            let (mut sum, mut cnt) = (0f32, 0usize);
            grid.for_each_within(&dp, dp[nearest.1 as usize], 0.5 * radius, |j, _| {
                if data[j as usize].n.dot(np) > 0.9 {
                    sum += (dp[j as usize] - v).dot(np);
                    cnt += 1;
                }
            });
            if cnt < 3 || np.dot(n) < 0.3 {
                return None;
            }
            let d = sum / cnt as f32;
            (d.abs() <= max_mm).then(|| v + np * d)
        })
        .collect();
    let mut count = 0;
    for (p, m) in pos.iter_mut().zip(moves) {
        if let Some(q) = m {
            *p = q;
            count += 1;
        }
    }
    count
}

/// Builds the solid. `markers`: known marker sites (may be empty).
pub fn build_solid(input: &[SolidPoint], markers: &[Marker], cfg: &SolidConfig) -> Solid {
    let mut rep = SolidReport { input_points: input.len(), markers_listed: markers.len(), ..Default::default() };
    let mut clock = Instant::now();
    let mut lap = |rep: &mut SolidReport, name: &str| {
        rep.stages.push((name.to_string(), clock.elapsed().as_secs_f64()));
        clock = Instant::now();
    };

    let mut pts = measured(input);

    // ---- 1. clean ---------------------------------------------------------------------------
    rep.dropped_outliers = prep::drop_isolated(&mut pts, cfg.outlier_radius_mm, cfg.outlier_min_neighbours);
    if cfg.off_surface_mm > 0.0 {
        rep.dropped_off_surface = prep::drop_off_surface(&mut pts, 1.0, cfg.off_surface_mm, 4.0);
    }
    if cfg.laser_strays {
        rep.dropped_laser_strays = prep::drop_behind_strays(&mut pts, 2.0, 0.4, 1.6);
    }
    if cfg.equalise_offsets {
        rep.scan_shifts = prep::equalise_scan_offsets(&mut pts, 0.03, 0.4, 500);
    }
    rep.dropped_covered_layers = prep::drop_covered_layers(&mut pts, &cfg.scan_priority, cfg.layer_radius_mm, cfg.layer_tolerance_mm);
    if cfg.rough_layer_ratio > 0.0 {
        rep.dropped_rough_layers = prep::drop_rough_layers(&mut pts, 1.5, cfg.rough_layer_ratio, 0.06);
    }
    let map = {
        let pos: Vec<Vec3> = pts.iter().map(|p| p.p).collect();
        let grid = grid::PointGrid::new(&pos, 1.0);
        prep::ThicknessMap::build(&pts, &pos, &grid, cfg.wall_search_mm, cfg.wall_mm)
    };
    rep.wall_median_mm = map.global_median;
    rep.wall_measurements = map.measurements;
    if cfg.ghost_filter {
        rep.dropped_ghosts = prep::drop_ghost_layers(&mut pts, &map, cfg.wall_search_mm);
    }
    lap(&mut rep, "clean");

    // ---- 2. markers -------------------------------------------------------------------------
    let (sites, detected, outcomes) = marker_stage(&mut pts, markers, &cfg.markers, cfg.detect.as_ref(), cfg.detect_dedupe_mm);
    rep.markers_detected = detected;
    rep.marker_cut_points = outcomes.iter().map(|o| o.cut_points).sum();
    rep.marker_fill_points = outcomes.iter().map(|o| o.fill_points).sum();
    rep.marker_outcomes = outcomes;
    lap(&mut rep, "markers");

    // ---- 3. completion and holes ------------------------------------------------------------
    if cfg.completion {
        let extra = prep::completion_points(&pts, &map, &cfg.completion_cfg);
        rep.completion_points = extra.len();
        pts.extend(extra);
    }
    lap(&mut rep, "completion");
    if let Some(hp) = &cfg.hole_fill {
        let (fill, holes) = holes::fill_holes(&pts, hp);
        rep.holes_filled = holes;
        rep.hole_fill_points = fill.len();
        pts.extend(fill);
        if cfg.completion && rep.hole_fill_points > 0 {
            let extra = prep::completion_points(&pts, &map, &cfg.completion_cfg);
            rep.completion_points_2 = extra.len();
            pts.extend(extra);
        }
        lap(&mut rep, "hole_fill");
    }

    // ---- 4. solve ---------------------------------------------------------------------------
    prep::area_weights(&mut pts, cfg.area_radius_mm);
    let conf: Vec<f32> = pts
        .iter()
        .map(|p| match p.kind {
            PointKind::Measured if cfg.sharp_scans.contains(&p.scan) => cfg.sharp_confidence,
            PointKind::Measured => cfg.measured_confidence,
            PointKind::MarkerFill => cfg.marker_fill_confidence,
            PointKind::HoleFill => cfg.hole_fill_confidence,
            PointKind::Completion => cfg.completion_confidence,
        })
        .collect();
    let class: Vec<u8> = pts
        .iter()
        .map(|p| match p.kind {
            PointKind::Measured if cfg.sharp_scans.contains(&p.scan) => 0,
            PointKind::Measured | PointKind::MarkerFill => 1,
            PointKind::Completion | PointKind::HoleFill => 2,
        })
        .collect();
    rep.samples = pts.len();
    lap(&mut rep, "area_weights");
    let ring = (cfg.band_mm / (8.0 * cfg.cell_mm)).ceil().max(1.0) as i32;
    let pp = poisson::PoissonParams {
        cell_mm: cfg.cell_mm,
        ring,
        screen: cfg.screen,
        tol: cfg.cg_tol,
        max_iter: cfg.cg_max_iter,
        coarse_start: cfg.coarse_start,
        smooth: [cfg.smooth_sharp, cfg.smooth_measured.max(cfg.smooth_sharp), cfg.smooth_soft.max(cfg.smooth_measured).max(cfg.smooth_sharp)],
    };
    let sol = poisson::solve(&pts, &conf, &class, &pp);
    drop(class);
    rep.band_blocks = sol.grid.blocks.len();
    rep.unknowns = sol.grid.len();
    rep.cg_iterations = sol.iterations;
    rep.cg_coarse_iterations = sol.coarse_iterations;
    rep.cg_rel_residual = sol.rel_residual;
    {
        let (mut sw, mut sc) = (0f64, 0f64);
        for p in pts.iter().filter(|p| p.kind == PointKind::Measured) {
            sw += p.area as f64;
            sc += p.area as f64 * sol.grid.sample(&sol.chi, p.p) as f64;
        }
        rep.mean_chi_measured = (sc / sw.max(1e-12)) as f32;
    }
    lap(&mut rep, "solve");

    // ---- 5. extract and finish --------------------------------------------------------------
    rep.iso = cfg.iso;
    let (pos, tris) = extract::extract(&sol.grid, &sol.chi, cfg.iso);
    let inside_nodes = cfg.keep_inside_nodes.then(|| {
        let mut v = Vec::new();
        for (bi, b) in sol.grid.blocks.iter().enumerate() {
            for l in 0..poisson::BV {
                if sol.chi[bi * poisson::BV + l] > cfg.iso {
                    let (x, y, z) = ((l % 8) as i32, ((l / 8) % 8) as i32, (l / 64) as i32);
                    v.push([b.x * 8 + x, b.y * 8 + y, b.z * 8 + z]);
                }
            }
        }
        (sol.grid.origin, sol.grid.h, v)
    });
    rep.raw_triangles = tris.len();
    drop(sol);
    let (pos, tris, frag, voids) = topo::filter_components(pos, tris, cfg.min_component_fraction);
    rep.dropped_fragments = frag;
    rep.dropped_voids = voids;
    let (mut pos, tris, collapses) = cleanup::collapse_short_edges(pos, tris, cfg.sliver_edge_frac * cfg.cell_mm);
    rep.sliver_collapses = collapses;
    lap(&mut rep, "extract");

    let mut origin = vertex_origins(&pos, &pts, cfg.origin_radius_mm);
    if cfg.refine_radius_mm > 0.0 && !cfg.sharp_scans.is_empty() {
        let data: Vec<WorkPoint> =
            pts.iter().filter(|p| p.kind == PointKind::Measured && cfg.sharp_scans.contains(&p.scan)).copied().collect();
        let vn = vertex_normals(&pos, &tris);
        rep.refined_vertices = refine_to_data(&mut pos, &vn, &origin, &data, cfg.refine_radius_mm, 0.15);
    }
    if cfg.smooth_surface > 0 {
        let sm = crate::FeatureSmoother { iterations: cfg.smooth_surface, sigma_normal: cfg.smooth_surface_sigma, ..Default::default() };
        sm.smooth_positions(&mut pos, &tris);
    }
    let mut outcomes = std::mem::take(&mut rep.marker_outcomes);
    if cfg.markers.fair_mesh && !sites.is_empty() {
        let vn = vertex_normals(&pos, &tris);
        let mut inside = vec![false; pos.len()];
        let fair = markers::fair_sites(&mut pos, &vn, &sites, &cfg.markers, &mut inside);
        rep.sites_faired = fair.iter().filter(|f| f.status == "faired").count();
        for (o, f) in outcomes.iter_mut().zip(fair) {
            o.fair = Some(f);
        }
        for (o, ins) in origin.iter_mut().zip(&inside) {
            if *ins {
                *o = VertexOrigin::MarkerFill;
            }
        }
    }
    rep.marker_outcomes = outcomes;
    rep.marker_sites = sites;
    let mask: Vec<bool> = origin.iter().map(|o| matches!(o, VertexOrigin::Completed | VertexOrigin::HoleFill)).collect();
    taubin_masked(&mut pos, &tris, &mask, cfg.smooth_completed, 0.5);
    let (before, after) = cleanup::repair(&mut pos, &tris, cfg.repair_rounds);
    rep.defects_before = before;
    rep.defects_after = after;
    lap(&mut rep, "finish");

    rep.topology = topo::analyze(&pos, &tris);
    for o in &origin {
        rep.origin_counts[*o as usize] += 1;
    }
    let vertices: Vec<Point3D> = pos
        .iter()
        .zip(&origin)
        .map(|(p, o)| Point3D { position: p.to_array(), normal: [0.0, 0.0, 1.0], color: o.color(), confidence: 1.0 })
        .collect();
    let triangles = tris.iter().map(|t| Triangle::new(t[0], t[1], t[2])).collect();
    let mut mesh = Mesh::from_geometry(vertices, triangles);
    mesh.recompute_normals();
    lap(&mut rep, "topology");

    let samples = pts.iter().map(WorkPoint::sample).collect();
    Solid { mesh, vertex_origin: origin, report: rep, samples, inside_nodes }
}
