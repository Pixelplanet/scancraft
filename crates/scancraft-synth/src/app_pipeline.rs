//! The app's Import Scan, Isolate, Merge, Reconstruct and Make Solid on library calls, with the
//! app's default settings, so a synthetic scene can be run the way the app runs it without
//! building the app (which is a binary crate with a GPU interface).
//!
//! Every step names the app code it copies. When the app changes, change this too:
//!
//! | Step | App code |
//! | --- | --- |
//! | import a PLY and its `marker_framework.mkf` | `apps/scancraft/src/imports.rs::read_scan`, `markers_beside`; `project.rs::PassMarker::from_revo` |
//! | object isolation (`whole_scene: false`) | `isolation.rs::segment`, `from_segmentation` (200 points / 5 %) |
//! | an imported pass's surfels (count 2) | `project.rs::PassSource::surfels`, `FIXED_OBSERVATIONS` |
//! | listed markers on the surface | `app_project.rs::markers_on_surface`, `placed_markers` (3 mm, radius >= 2 mm) |
//! | Merge | `app_project.rs::merge_passes` (pitch 0.5 mm of `AccuracyPreset::Standard`, outliers 12 / 3.0) |
//! | Reconstruct | `app.rs`, `Tool::Reconstruct` (cell 0.7, detail 3, smoother 4 / keep edges 0.7, marker fairing) |
//! | Make Solid | `app.rs::SolidJob::from_settings`, `make_solid`; `app_project.rs::solid_points` (cell 0.4, markers removed) |
//! | Auto Align (alignment check) | `app_project.rs::align_auto` (`align_robust` against the placed passes) |
//!
//! Imported passes all weigh 2.0 (`default_weight` of the default light source), so Make Solid
//! gets no scan priority and `sharp_top` does not apply.

use std::path::Path;
use std::time::Instant;

use glam::{DVec3, Quat, Vec3};
use scancraft_align::{MergeConfig, MergeInput, MergeResult, Oriented, PassCloud, Plane};
use scancraft_core::{Mesh, Point3D, PointCloud, Transform};
use scancraft_fusion::{segment_object, SegmentConfig, SegmentLabel};
use scancraft_mesh::solid::markers::{Marker, MarkerSource};
use scancraft_mesh::{FeatureSmoother, Mesher, PrepareConfig, PrepareReport, Solid, SolidConfig, SolidPoint, SurfaceConfig};

use crate::scan::{scan_pass, PointKind};
use crate::scene::Scene;
use crate::sdf::Rigid;
use crate::write::{Truth, LABEL_PART, LABEL_STICKER, LABEL_STRAY, LABEL_TABLE};

/// The app's defaults (`apps/scancraft/src/app.rs`, `impl Default for Settings`).
pub mod defaults {
    pub const MC_CELL_MM: f32 = 0.7;
    pub const MESH_SMOOTHNESS: f32 = 3.0;
    pub const MESH_AUTO_SMOOTH: usize = 4;
    pub const SMOOTH_KEEP_EDGES: f32 = 0.7;
    pub const SOLID_CELL_MM: f32 = 0.4;
    /// `AccuracyPreset::Standard.point_pitch_mm()`: imported passes have no scan settings.
    pub const MERGE_PITCH_MM: f32 = 0.5;
    pub const MERGE_SIGMA: f32 = 3.0;
    /// `project.rs::FIXED_OBSERVATIONS`.
    pub const FIXED_OBSERVATIONS: u32 = 2;
    /// `app_project.rs::MARKER_ON_SURFACE_MM`, `MIN_STICKER_RADIUS_MM`.
    pub const MARKER_ON_SURFACE_MM: f32 = 3.0;
    pub const MIN_STICKER_RADIUS_MM: f32 = 2.0;
    /// `isolation.rs::MIN_OBJECT_POINTS`, `MIN_OBJECT_SHARE`.
    pub const MIN_OBJECT_POINTS: usize = 200;
    pub const MIN_OBJECT_SHARE: f32 = 0.05;
}

/// A listed marker of a pass, pass frame (`project.rs::PassMarker`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PassMarker {
    pub centre: Vec3,
    pub normal: Option<Vec3>,
    pub radius_mm: Option<f32>,
}

impl PassMarker {
    pub fn from_revo(m: &scancraft_io::RevoMarker) -> Self {
        let normal = Vec3::from_array(m.normal.map(|v| v as f32)).normalize_or_zero();
        PassMarker { centre: Vec3::from_array(m.centre.map(|v| v as f32)), normal: (normal != Vec3::ZERO).then_some(normal), radius_mm: (m.radius_mm > 0.0).then_some(m.radius_mm) }
    }
}

/// One imported pass: its points in its own frame, its marker list and the true pose.
#[derive(Debug, Clone)]
pub struct PassData {
    pub id: usize,
    pub name: String,
    pub points: Vec<Point3D>,
    /// Per point (0 part, 1 sticker, 2 table, 3 stray), when known.
    pub labels: Option<Vec<u8>>,
    pub markers: Vec<PassMarker>,
    pub pass_to_part: Rigid,
}

/// Imports every pass of a generated scene as Import Scan does.
pub fn load_passes(scene_dir: &Path, truth: &Truth) -> Result<Vec<PassData>, String> {
    truth
        .passes
        .iter()
        .map(|p| {
            let path = scene_dir.join(&p.cloud);
            let scan = scancraft_io::import_scan(&path).map_err(|e| format!("{}: {e}", path.display()))?;
            let all = scan.to_point_cloud().points;
            let finite: Vec<bool> = all.iter().map(|q| q.position.iter().all(|v| v.is_finite())).collect();
            let points: Vec<Point3D> = all.into_iter().zip(&finite).filter(|(_, f)| **f).map(|(q, _)| q).collect();
            let labels = std::fs::read(scene_dir.join(&p.labels)).ok().filter(|l| l.len() == finite.len()).map(|l| l.into_iter().zip(&finite).filter(|(_, f)| **f).map(|(l, _)| l).collect());
            let markers = match scancraft_io::marker_framework_beside(&path) {
                Some(mkf) => scancraft_io::read_marker_framework(&mkf).map_err(|e| e.to_string())?.iter().map(PassMarker::from_revo).collect(),
                None => Vec::new(),
            };
            Ok(PassData { id: p.id, name: p.name.clone(), points, labels, markers, pass_to_part: p.pass_to_part })
        })
        .collect()
}

/// Scans `scene` in memory: the same points and marker lists [`crate::write::generate`] writes.
pub fn scan_in_memory(scene: &Scene) -> Vec<PassData> {
    scene
        .passes
        .iter()
        .enumerate()
        .map(|(i, spec)| {
            let scan = scan_pass(scene, i);
            let pose = spec.scan_pose;
            let points = scan
                .points
                .iter()
                .map(|p| Point3D {
                    position: pose.apply(p.pos).as_vec3().to_array(),
                    normal: pose.apply_dir(p.normal).normalize_or_zero().as_vec3().to_array(),
                    color: scancraft_core::ColorRgba::new(176, 176, 170, 255),
                    confidence: p.hits as f32,
                })
                .collect();
            let labels = scan
                .points
                .iter()
                .map(|p| match p.kind {
                    PointKind::Part => LABEL_PART,
                    PointKind::Sticker(_) => LABEL_STICKER,
                    PointKind::Table => LABEL_TABLE,
                    PointKind::Stray => LABEL_STRAY,
                })
                .collect();
            let markers = scan
                .listed
                .iter()
                .map(|m| {
                    let n = pose.apply_dir(m.normal).normalize().as_vec3();
                    PassMarker { centre: pose.apply(m.centre).as_vec3(), normal: Some(n), radius_mm: (m.radius_mm > 0.0).then_some(m.radius_mm) }
                })
                .collect();
            PassData { id: i + 1, name: spec.name.clone(), points, labels: Some(labels), markers, pass_to_part: spec.scan_pose.inverse() }
        })
        .collect()
}

/// `Rigid` (f64) as the app's `Transform`.
pub fn to_transform(r: &Rigid) -> Transform {
    let q = r.quat();
    Transform { translation: DVec3::from_array(r.translation).as_vec3(), rotation: Quat::from_xyzw(q.x as f32, q.y as f32, q.z as f32, q.w as f32).normalize(), scale: Vec3::ONE }
}

/// A pass's object isolation (`isolation.rs::ObjectMask`).
#[derive(Debug, Clone)]
pub struct Isolation {
    pub object: Vec<bool>,
    pub support: Option<Plane>,
    pub kept: usize,
    /// Nothing could be told apart: everything is kept.
    pub fallback: bool,
    pub seconds: f64,
}

/// Isolates a pass's object as the app does on import (`isolation::segment`).
pub fn isolate(points: &[Point3D]) -> Isolation {
    let t0 = Instant::now();
    let positions: Vec<Vec3> = points.iter().map(|p| Vec3::from(p.position)).collect();
    let normals: Vec<Vec3> = points.iter().map(|p| Vec3::from(p.normal).normalize_or_zero()).collect();
    let seg = segment_object(&positions, Some(&normals), &SegmentConfig::default(), None, None);
    let object: Vec<bool> = seg.labels.iter().map(|l| *l == SegmentLabel::Object).collect();
    let kept = object.iter().filter(|o| **o).count();
    let seconds = t0.elapsed().as_secs_f64();
    if kept < defaults::MIN_OBJECT_POINTS || (kept as f32) < defaults::MIN_OBJECT_SHARE * object.len() as f32 {
        return Isolation { object: vec![true; points.len()], support: None, kept: points.len(), fallback: true, seconds };
    }
    Isolation { object, support: seg.support.map(|s| Plane::through(s.point, s.normal)), kept, fallback: false, seconds }
}

/// A pass placed in the project: its object surfels (pass frame), pose and markers.
#[derive(Debug, Clone)]
pub struct Placed {
    /// Scan id from 1, in pass order (`solid_input`).
    pub scan: u8,
    pub transform: Transform,
    /// (position, unit normal, observations) of the object's points, pass frame.
    pub surfels: Vec<(Vec3, Vec3, u32)>,
    pub markers: Vec<PassMarker>,
    /// Labels of the kept surfels, when known.
    pub labels: Option<Vec<u8>>,
    pub support: Option<Plane>,
}

/// The passes placed with `poses` (project <- pass) and isolated unless `whole_scene`.
pub fn place(passes: &[PassData], isolations: &[Isolation], poses: &[Transform], whole_scene: bool) -> Vec<Placed> {
    passes
        .iter()
        .zip(isolations)
        .zip(poses)
        .enumerate()
        .map(|(i, ((p, iso), t))| {
            let keep = |k: usize| whole_scene || iso.object[k];
            let surfels = p.points.iter().enumerate().filter(|(k, _)| keep(*k)).map(|(_, q)| (Vec3::from(q.position), Vec3::from(q.normal).normalize_or_zero(), defaults::FIXED_OBSERVATIONS)).collect();
            let labels = p.labels.as_ref().map(|l| l.iter().enumerate().filter(|(k, _)| keep(*k)).map(|(_, v)| *v).collect());
            Placed { scan: (i + 1).min(255) as u8, transform: *t, surfels, markers: p.markers.clone(), labels, support: if whole_scene { None } else { iso.support } }
        })
        .collect()
}

/// `app_project.rs::markers_on_surface`.
pub fn markers_on_surface(map: &[PassMarker], surfels: impl Iterator<Item = (Vec3, Vec3)>) -> Vec<Option<Vec3>> {
    use std::collections::HashMap;
    let r = defaults::MARKER_ON_SURFACE_MM;
    let cell = |p: Vec3| (p / r).floor().as_ivec3();
    let mut cells: HashMap<glam::IVec3, Vec<usize>> = HashMap::new();
    let (mut lo, mut hi) = (Vec3::splat(f32::MAX), Vec3::splat(f32::MIN));
    for (k, m) in map.iter().enumerate().filter(|(_, m)| m.centre.is_finite()) {
        let c = cell(m.centre);
        for d in (-1..=1).flat_map(|x| (-1..=1).flat_map(move |y| (-1..=1).map(move |z| glam::IVec3::new(x, y, z)))) {
            cells.entry(c + d).or_default().push(k);
        }
        lo = lo.min(m.centre - r);
        hi = hi.max(m.centre + r);
    }
    let mut near = vec![false; map.len()];
    let mut normal_sum = vec![Vec3::ZERO; map.len()];
    if !cells.is_empty() {
        for (p, n) in surfels.filter(|(p, _)| p.cmpge(lo).all() && p.cmple(hi).all()) {
            for &k in cells.get(&cell(p)).into_iter().flatten() {
                if p.distance_squared(map[k].centre) < r * r {
                    near[k] = true;
                    normal_sum[k] += n;
                }
            }
        }
    }
    map.iter()
        .zip(near.into_iter().zip(normal_sum))
        .map(|(m, (near, sum))| {
            let n = m.normal.unwrap_or(sum);
            let sticker = m.radius_mm.is_none_or(|r| !(r > 0.0 && r < defaults::MIN_STICKER_RADIUS_MM));
            (near && sticker && m.centre.is_finite() && n.is_finite() && n.length() > 1e-3).then_some(n)
        })
        .collect()
}

/// `app_project.rs::placed_markers`: a pass's listed markers on its kept surface, project frame.
pub fn placed_markers(p: &Placed) -> Vec<Marker> {
    let m = p.transform.to_mat4();
    let normals = markers_on_surface(&p.markers, p.surfels.iter().map(|s| (s.0, s.1)));
    p.markers
        .iter()
        .zip(normals)
        .enumerate()
        .filter_map(|(k, (marker, n))| {
            Some(Marker {
                id: format!("S{}-M{k}", p.scan),
                center: m.transform_point3(marker.centre),
                normal: (p.transform.rotation * n?).normalize(),
                scan: Some(p.scan),
                source: MarkerSource::List,
                radius_mm: marker.radius_mm.unwrap_or(0.0),
                score: 0.0,
            })
        })
        .collect()
}

/// The listed markers Reconstruct and Make Solid remove (`listed_markers`).
pub fn listed_markers(placed: &[Placed]) -> Vec<Marker> {
    placed.iter().filter(|p| !p.markers.is_empty()).flat_map(placed_markers).collect()
}

/// Merge (`merge_passes`): the placed passes into one cloud, project frame.
pub fn merge(placed: &[Placed]) -> MergeResult {
    let config = MergeConfig { pitch_mm: defaults::MERGE_PITCH_MM, outliers: Some((12, defaults::MERGE_SIGMA)), ..MergeConfig::default() };
    let inputs: Vec<MergeInput> = placed
        .iter()
        .map(|p| {
            let m = p.transform.to_mat4();
            MergeInput { points: p.surfels.iter().map(|(q, n, c)| (m.transform_point3(*q), (p.transform.rotation * *n).normalize_or_zero(), *c)).collect(), weight: 2.0 }
        })
        .collect();
    scancraft_align::merge(&inputs, &config)
}

/// Reconstruct's result.
pub struct Reconstructed {
    pub mesh: Mesh,
    pub prepared: PointCloud,
    pub prepare: PrepareReport,
    pub point_spacing_mm: f32,
    pub support_radius_mm: f32,
    pub seconds_prepare: f64,
    pub seconds_mesh: f64,
    pub seconds_smooth_fair: f64,
}

/// `Tool::Reconstruct` on `input` (the merged model) with the listed `markers`.
pub fn reconstruct(input: &PointCloud, markers: &[Marker]) -> Reconstructed {
    let config = SurfaceConfig { cell_mm: defaults::MC_CELL_MM, detail: defaults::MESH_SMOOTHNESS, ..Default::default() };
    let keep = defaults::SMOOTH_KEEP_EDGES.clamp(0.0, 1.0);
    let smoother = FeatureSmoother { iterations: defaults::MESH_AUTO_SMOOTH, sigma_normal: 1.2 + (0.22 - 1.2) * keep, ..Default::default() };
    let prepare = PrepareConfig { suppress_artefacts: true, remove_markers: true, ..Default::default() };
    let t0 = Instant::now();
    let (cloud, prep) = scancraft_mesh::prepare_points(input, markers, &prepare);
    let seconds_prepare = t0.elapsed().as_secs_f64();
    let t1 = Instant::now();
    let (mut mesh, stats) = Mesher::reconstruct_with_stats(&cloud, &config);
    let seconds_mesh = t1.elapsed().as_secs_f64();
    let t2 = Instant::now();
    if !mesh.triangles.is_empty() {
        smoother.smooth(&mut mesh);
        if !prep.marker_sites.is_empty() {
            scancraft_mesh::solid::fair_marker_sites_on_fills(&mut mesh, &prep.marker_sites, &prep.marker_outcomes, &prepare.markers);
        }
    }
    Reconstructed {
        mesh,
        prepared: cloud,
        prepare: prep,
        point_spacing_mm: stats.point_spacing_mm,
        support_radius_mm: stats.support_radius_mm,
        seconds_prepare,
        seconds_mesh,
        seconds_smooth_fair: t2.elapsed().as_secs_f64(),
    }
}

/// Make Solid with the default settings (`SolidJob::from_settings`, `make_solid`, `solid_points`).
pub fn make_solid(placed: &[Placed]) -> Result<Solid, String> {
    let config = SolidConfig { cell_mm: defaults::SOLID_CELL_MM, detect: Some(Default::default()), ..Default::default() };
    let ok = |p: Vec3, n: Vec3| p.is_finite() && n.is_finite() && n.length() > 1e-3;
    let mut points = Vec::new();
    let mut markers = Vec::new();
    for p in placed {
        markers.extend(placed_markers(p));
        let m = p.transform.to_mat4();
        points.extend(
            p.surfels
                .iter()
                .map(|(q, n, _)| (m.transform_point3(*q), (p.transform.rotation * *n).normalize_or_zero()))
                .filter(|(q, n)| ok(*q, *n))
                .map(|(position, normal)| SolidPoint { position, normal, scan: p.scan }),
        );
    }
    if points.len() < 100 {
        return Err("Make Solid needs points with normals".into());
    }
    let solid = scancraft_mesh::build_solid(&points, &markers, &config);
    if solid.mesh.triangles.is_empty() {
        return Err("No solid could be built from these points".into());
    }
    Ok(solid)
}

/// Auto Align of pass `moving` against the reference pass `reference` placed at `reference_pose`
/// (`align_auto` with one placed pass: the project model keeps that pass's table plane).
pub fn auto_align(moving: &Placed, reference: &Placed) -> Result<(Transform, f32), String> {
    let cloud = |p: &Placed| PassCloud { points: p.surfels.iter().map(|s| (s.0, s.1)).collect::<Vec<Oriented>>(), views: Vec::new(), support: p.support };
    let target = scancraft_align::quality::moved(&cloud(reference), &reference.transform);
    let r = scancraft_align::align_robust(&cloud(moving), &target, &scancraft_align::RobustConfig::default()).map_err(|e| e.to_string())?;
    Ok((r.transform(), r.best.alignment.quality.rmse_mm))
}
