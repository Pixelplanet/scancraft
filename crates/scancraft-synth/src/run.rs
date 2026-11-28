//! Runs one scene through the app's pipeline ([`crate::app_pipeline`]: import, isolate, place at
//! the true poses, Merge, Reconstruct, Make Solid) and evaluates both meshes ([`crate::eval`]),
//! optionally with an alignment check (the app's Auto Align of each further pass against pass 1,
//! compared with the true pose).

use std::io::Write as _;
use std::path::Path;
use std::time::Instant;

use glam::{DVec3, Vec3};
use scancraft_core::{Mesh, PointCloud, Transform};
use serde::{Deserialize, Serialize};
use serde_json::Value;

use crate::app_pipeline::{self as app, PassData};
use crate::eval::{evaluate, topology, truth_samples, Context, MeshEval, StrayCounts, Topology};
use crate::memory::Sampler;
use crate::write::{Truth, LABEL_STRAY};

#[derive(Debug, Clone)]
pub struct RunOptions {
    /// Run the app's Auto Align on every pass after the first and compare it with the truth.
    pub align_check: bool,
    /// Skip Make Solid (faster tests).
    pub solid: bool,
    /// The app's "whole scene" setting: no object isolation (the default is off, so the app
    /// isolates every imported pass).
    pub whole_scene: bool,
    /// Not the app: isolate every pass by the generator's labels (drop the table and clutter,
    /// keep everything else), so the meshers are measured on a perfect isolation.
    pub truth_isolation: bool,
}

impl Default for RunOptions {
    fn default() -> Self {
        RunOptions { align_check: true, solid: true, whole_scene: false, truth_isolation: false }
    }
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct PassReport {
    pub id: usize,
    pub name: String,
    pub points: usize,
    pub markers_listed: usize,
    /// Listed markers found on the kept (isolated) surface.
    pub markers_on_surface: usize,
    pub isolation_kept: usize,
    pub isolation_fallback: bool,
    pub isolation_seconds: f64,
    /// Points isolation dropped, by label: part, sticker, table, stray.
    pub dropped_by_label: [usize; 4],
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct AlignCheck {
    pub pass: usize,
    pub rotation_error_deg: Option<f64>,
    /// RMS over the pass's points of |found pose - true pose|.
    pub translation_rms_mm: Option<f64>,
    pub rmse_mm: Option<f32>,
    /// The found and the true pose (project <- pass): rotation x, y, z, w and translation.
    pub found: Option<([f32; 4], [f32; 3])>,
    pub truth: Option<([f32; 4], [f32; 3])>,
    pub seconds: f64,
    pub error: Option<String>,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Stage {
    pub seconds: f64,
    pub peak_mb: f64,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct SceneResult {
    pub scene: String,
    /// Run with the app's "whole scene" setting (no isolation).
    pub whole_scene: bool,
    /// Isolated by the generator's labels instead of the app's isolation.
    #[serde(default)]
    pub truth_isolation: bool,
    pub tags: Vec<String>,
    pub passes: Vec<PassReport>,
    pub import: Stage,
    pub isolate: Stage,
    pub merge: Stage,
    pub merged_points: usize,
    pub merge_outweighed: usize,
    pub merge_isolated: usize,
    pub merge_outliers: usize,
    pub reconstruct: Stage,
    pub reconstruct_detail: Value,
    pub solid: Stage,
    pub solid_detail: Value,
    pub truth_topology: Option<Topology>,
    pub strays: StrayCounts,
    pub alignment: Vec<AlignCheck>,
    pub peak_mb: Option<f64>,
    pub seconds: f64,
    pub evals: Vec<MeshEval>,
    pub error: Option<String>,
}

impl SceneResult {
    pub fn eval(&self, name: &str) -> Option<&MeshEval> {
        self.evals.iter().find(|e| e.mesh == name)
    }
}

/// Meshes and clouds of a run, in the part frame.
pub struct Outputs {
    pub merged: PointCloud,
    pub reconstruct: Mesh,
    pub solid: Option<Mesh>,
}

fn inside_count(truth: &Truth, pts: impl Iterator<Item = Vec3>) -> usize {
    pts.filter(|p| {
        let d = truth.part.sdf(p.as_dvec3());
        d < -0.3 && d > -1.5
    })
    .count()
}

/// Runs a scene. `passes` as imported ([`app::load_passes`] or [`app::scan_in_memory`]);
/// `reference` is the true part as a mesh (to sample its surface).
pub fn run_scene(truth: &Truth, passes: &[PassData], reference: &Mesh, import: Stage, options: &RunOptions) -> (SceneResult, Option<Outputs>) {
    let t_all = Instant::now();
    let sampler = Sampler::start();
    let mut res = SceneResult { scene: truth.scene.clone(), tags: truth.tags.clone(), import, whole_scene: options.whole_scene, truth_isolation: options.truth_isolation, ..Default::default() };

    // Import's isolation (whole_scene is off by default).
    let t = Instant::now();
    let isolations: Vec<app::Isolation> = passes
        .iter()
        .map(|p| match (&p.labels, options.truth_isolation) {
            (Some(l), true) => {
                // The labels choose the points; the support plane is the app's (Make Solid uses it).
                let object: Vec<bool> = l.iter().map(|v| *v != crate::write::LABEL_TABLE).collect();
                let kept = object.iter().filter(|o| **o).count();
                let support = app::isolate(&p.points).support;
                app::Isolation { object, support, kept, fallback: false, seconds: 0.0 }
            }
            _ => app::isolate(&p.points),
        })
        .collect();
    res.isolate = Stage { seconds: t.elapsed().as_secs_f64(), peak_mb: sampler.mark() };

    let poses: Vec<Transform> = passes.iter().map(|p| app::to_transform(&p.pass_to_part)).collect();
    let placed = app::place(passes, &isolations, &poses, options.whole_scene);
    for ((p, iso), pl) in passes.iter().zip(&isolations).zip(&placed) {
        let mut dropped = [0usize; 4];
        if let Some(l) = &p.labels {
            for (k, keep) in iso.object.iter().enumerate() {
                if !keep {
                    dropped[(l[k] as usize).min(3)] += 1;
                }
            }
        }
        res.passes.push(PassReport {
            id: p.id,
            name: p.name.clone(),
            points: p.points.len(),
            markers_listed: p.markers.len(),
            markers_on_surface: app::placed_markers(pl).len(),
            isolation_kept: iso.kept,
            isolation_fallback: iso.fallback,
            isolation_seconds: iso.seconds,
            dropped_by_label: dropped,
        });
    }
    res.strays.input = placed.iter().filter_map(|p| p.labels.as_ref()).map(|l| l.iter().filter(|v| **v == LABEL_STRAY).count()).sum();

    if options.align_check && placed.len() > 1 {
        for (i, pl) in placed.iter().enumerate().skip(1) {
            let t = Instant::now();
            let mut moving = pl.clone();
            moving.transform = Transform::default();
            let r = app::auto_align(&moving, &placed[0]);
            let mut check = AlignCheck { pass: passes[i].id, seconds: t.elapsed().as_secs_f64(), ..Default::default() };
            match r {
                Ok((found, rmse)) => {
                    let truth_t = poses[i];
                    let dq = found.rotation * truth_t.rotation.inverse();
                    check.rotation_error_deg = Some((dq.angle_between(glam::Quat::IDENTITY).to_degrees() as f64 * 1000.0).round() / 1000.0);
                    let (mf, mt) = (found.to_mat4(), truth_t.to_mat4());
                    let step = (pl.surfels.len() / 20_000).max(1);
                    let d: Vec<f64> = pl.surfels.iter().step_by(step).map(|s| mf.transform_point3(s.0).distance(mt.transform_point3(s.0)) as f64).collect();
                    check.translation_rms_mm = Some(((d.iter().map(|x| x * x).sum::<f64>() / d.len().max(1) as f64).sqrt() * 1000.0).round() / 1000.0);
                    check.rmse_mm = Some(rmse);
                    check.found = Some((found.rotation.to_array(), found.translation.to_array()));
                    check.truth = Some((truth_t.rotation.to_array(), truth_t.translation.to_array()));
                }
                Err(e) => check.error = Some(e),
            }
            res.alignment.push(check);
        }
        sampler.mark();
    }

    // Merge.
    let t = Instant::now();
    let merged = app::merge(&placed);
    res.merge = Stage { seconds: t.elapsed().as_secs_f64(), peak_mb: sampler.mark() };
    res.merged_points = merged.points.len();
    res.merge_outweighed = merged.outweighed;
    res.merge_isolated = merged.isolated;
    res.merge_outliers = merged.outliers;
    res.strays.after_merge = inside_count(truth, merged.points.iter().map(|p| Vec3::from(p.position)));
    let merged = PointCloud::from_points(merged.points);
    if merged.is_empty() {
        res.error = Some("the merge produced no points".into());
        return (res, None);
    }

    // Reconstruct.
    let markers = app::listed_markers(&placed);
    let t = Instant::now();
    let rec = app::reconstruct(&merged, &markers);
    res.reconstruct = Stage { seconds: t.elapsed().as_secs_f64(), peak_mb: sampler.mark() };
    res.strays.after_prepare = inside_count(truth, rec.prepared.points.iter().map(|p| Vec3::from(p.position)));
    let pr = &rec.prepare;
    res.reconstruct_detail = serde_json::json!({
        "markers_listed": markers.len(),
        "prepared_points": rec.prepared.len(),
        "dropped_off_surface": pr.dropped_off_surface, "dropped_ghosts": pr.dropped_ghosts, "dropped_laser_strays": pr.dropped_laser_strays,
        "marker_sites": pr.marker_sites.len(), "marker_cut_points": pr.marker_cut_points, "marker_fill_points": pr.marker_fill_points,
        "marker_outcomes": pr.marker_outcomes.iter().map(|o| format!("{} ({:.1}, {:.1}, {:.1}): {}", o.id, o.center.x, o.center.y, o.center.z, o.fill)).collect::<Vec<_>>(),
        "point_spacing_mm": rec.point_spacing_mm, "support_radius_mm": rec.support_radius_mm,
        "seconds_prepare": rec.seconds_prepare, "seconds_mesh": rec.seconds_mesh, "seconds_smooth_fair": rec.seconds_smooth_fair,
    });

    // Make Solid.
    let solid = if options.solid {
        let t = Instant::now();
        let s = app::make_solid(&placed);
        res.solid = Stage { seconds: t.elapsed().as_secs_f64(), peak_mb: sampler.mark() };
        match s {
            Ok(s) => {
                let r = &s.report;
                res.strays.solid_dropped = r.dropped_laser_strays;
                res.solid_detail = serde_json::json!({
                    "markers_listed": r.markers_listed, "markers_detected": r.markers_detected.len(), "marker_sites": r.marker_sites.len(), "sites_faired": r.sites_faired,
                    "holes_filled": r.holes_filled, "wall_median_mm": r.wall_median_mm,
                    "dropped_outliers": r.dropped_outliers, "dropped_off_surface": r.dropped_off_surface, "dropped_covered_layers": r.dropped_covered_layers,
                    "dropped_ghosts": r.dropped_ghosts, "dropped_laser_strays": r.dropped_laser_strays, "dropped_rough_layers": r.dropped_rough_layers,
                    "scan_shifts": r.scan_shifts, "measured_fraction": r.origin_fractions()[0],
                    "marker_outcomes": r.marker_outcomes.iter().map(|o| format!("{} ({:.1}, {:.1}, {:.1}): {} / {}", o.id, o.center.x, o.center.y, o.center.z, o.fill, o.fair.as_ref().map_or("no fairing", |f| f.status.as_str()))).collect::<Vec<_>>(),
                    "stages": r.stages,
                });
                Some(s.mesh)
            }
            Err(e) => {
                res.solid_detail = serde_json::json!({ "error": e });
                None
            }
        }
    } else {
        None
    };

    // Evaluate.
    let pass_points: Vec<Vec<Vec3>> = passes
        .iter()
        .map(|p| {
            let m = app::to_transform(&p.pass_to_part).to_mat4();
            p.points.iter().enumerate().filter(|(k, _)| p.labels.as_ref().is_none_or(|l| l[*k] <= 1)).map(|(_, q)| m.transform_point3(Vec3::from(q.position))).collect()
        })
        .collect();
    let samples = truth_samples(truth, reference, 0.5, &pass_points);
    res.truth_topology = Some(topology(reference));
    let strays = res.strays.clone();
    res.evals.push(evaluate("reconstruct", &rec.mesh, &Context { truth, samples: &samples, strays: Some(&strays), solid: false }));
    if let Some(s) = &solid {
        res.evals.push(evaluate("solid", s, &Context { truth, samples: &samples, strays: Some(&strays), solid: true }));
    }
    sampler.mark();
    res.peak_mb = crate::memory::peak_working_set_mb().map(|m| m.round());
    res.seconds = t_all.elapsed().as_secs_f64();
    (res, Some(Outputs { merged, reconstruct: rec.mesh, solid }))
}

/// Imports a generated scene folder and runs it.
pub fn run_scene_dir(scene_dir: &Path, options: &RunOptions) -> Result<(SceneResult, Truth, Option<Outputs>), String> {
    let truth = Truth::load(scene_dir).map_err(|e| format!("{}: {e}", scene_dir.display()))?;
    let t = Instant::now();
    let passes = app::load_passes(scene_dir, &truth)?;
    let import = Stage { seconds: t.elapsed().as_secs_f64(), peak_mb: crate::memory::working_set_mb().unwrap_or(0.0).round() };
    let file = truth.reference_mesh.as_ref().map(|m| m.file.clone()).unwrap_or_else(|| crate::write::REFERENCE_MESH_FILE.into());
    let reference = scancraft_io::import_scan(scene_dir.join(&file)).map_err(|e| e.to_string())?.to_mesh().ok_or("the reference mesh has no faces")?;
    let (res, out) = run_scene(&truth, &passes, &reference, import, options);
    Ok((res, truth, out))
}

/// Writes a mesh as binary PLY with each vertex's signed distance to the true part (`dev`).
pub fn write_dev_mesh(path: &Path, mesh: &Mesh, truth: &Truth) -> std::io::Result<()> {
    use rayon::prelude::*;
    let dev: Vec<f32> = mesh.vertices.par_iter().map(|v| truth.part.sdf(DVec3::from_array(v.position.map(f64::from))) as f32).collect();
    let mut w = std::io::BufWriter::new(std::fs::File::create(path)?);
    write!(w, "ply\nformat binary_little_endian 1.0\nelement vertex {}\nproperty float x\nproperty float y\nproperty float z\nproperty float dev\nelement face {}\nproperty list uchar int vertex_indices\nend_header\n", mesh.vertices.len(), mesh.triangles.len())?;
    for (v, d) in mesh.vertices.iter().zip(&dev) {
        for c in v.position {
            w.write_all(&c.to_le_bytes())?;
        }
        w.write_all(&d.to_le_bytes())?;
    }
    for t in &mesh.triangles {
        w.write_all(&[3u8])?;
        for i in t.indices {
            w.write_all(&(i as i32).to_le_bytes())?;
        }
    }
    w.flush()
}

/// Writes points as binary PLY with normals and each point's signed distance to the true part.
pub fn write_dev_points(path: &Path, cloud: &PointCloud, truth: &Truth) -> std::io::Result<()> {
    use rayon::prelude::*;
    let dev: Vec<f32> = cloud.points.par_iter().map(|p| truth.part.sdf(DVec3::from_array(p.position.map(f64::from))) as f32).collect();
    let mut w = std::io::BufWriter::new(std::fs::File::create(path)?);
    write!(w, "ply\nformat binary_little_endian 1.0\nelement vertex {}\nproperty float x\nproperty float y\nproperty float z\nproperty float nx\nproperty float ny\nproperty float nz\nproperty float dev\nend_header\n", cloud.len())?;
    for (p, d) in cloud.points.iter().zip(&dev) {
        for c in p.position.iter().chain(&p.normal) {
            w.write_all(&c.to_le_bytes())?;
        }
        w.write_all(&d.to_le_bytes())?;
    }
    w.flush()
}
