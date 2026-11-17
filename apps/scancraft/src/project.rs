//! The project (`docs/architecture/scan-sessions.md`): each pass's mode, device snapshot and
//! alignment into the project frame, and saving / opening the project as a folder.

use std::path::{Path, PathBuf};

use glam::{Quat, Vec3};
use scancraft_align::{PassCloud, Quality, View};
use scancraft_core::{Mesh, Point3D, PointCloud, Transform};
use scancraft_driver::{DeviceInfo, LightSource, ScanConfig};
use scancraft_io::project::{self as files, AlignmentRecord, KeyframeRecord, OutputRecord, PassRecord, ProjectFile};
use scancraft_slam::{Keyframe, KeyframeData, SeedView, Tracker, TrackerConfig};

use crate::model::KeyframeInfo;

/// How a pass got its place in the project frame.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AlignMethod {
    Auto,
    PointPairs,
    Markers,
    /// Started without moving the object: registered into the project model while capturing.
    SamePosition,
    /// Adjusted by global refinement.
    Refined,
}

impl AlignMethod {
    pub fn label(self) -> &'static str {
        match self {
            AlignMethod::Auto => "Automatic",
            AlignMethod::PointPairs => "Point pairs",
            AlignMethod::Markers => "Markers",
            AlignMethod::SamePosition => "Same position",
            AlignMethod::Refined => "Refined",
        }
    }

    fn key(self) -> &'static str {
        match self {
            AlignMethod::Auto => "auto",
            AlignMethod::PointPairs => "point_pairs",
            AlignMethod::Markers => "markers",
            AlignMethod::SamePosition => "same_position",
            AlignMethod::Refined => "refined",
        }
    }

    fn from_key(key: &str) -> Self {
        match key {
            "point_pairs" => AlignMethod::PointPairs,
            "markers" => AlignMethod::Markers,
            "same_position" => AlignMethod::SamePosition,
            "refined" => AlignMethod::Refined,
            _ => AlignMethod::Auto,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum AlignState {
    /// Defines the project frame.
    Reference,
    Aligned { method: AlignMethod, quality: Option<Quality> },
    /// In its own frame; needs aligning.
    Unaligned,
    /// A same-position pass still locating itself in the project model while capturing.
    Locating,
}

impl AlignState {
    /// In the project frame (reference or aligned).
    pub fn is_placed(self) -> bool {
        matches!(self, AlignState::Reference | AlignState::Aligned { .. })
    }

    pub fn label(self) -> String {
        match self {
            AlignState::Reference => "Reference".into(),
            AlignState::Aligned { method, .. } => format!("Aligned · {}", method.label().to_lowercase()),
            AlignState::Unaligned => "Not aligned".into(),
            AlignState::Locating => "Locating in the project…".into(),
        }
    }
}

/// How a new pass starts relative to the earlier ones.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum PassStart {
    /// The object was flipped or repositioned: the pass gets its own frame.
    #[default]
    Moved,
    /// The object was not touched: the pass registers into the project frame as it starts.
    SamePosition,
}

/// Everything the project knows about a pass besides its geometry.
#[derive(Clone, Debug)]
pub struct PassMeta {
    /// Scan mode and settings it was captured with (fixed for the pass).
    pub scan_config: ScanConfig,
    /// The scanner it was captured with.
    pub device: Option<DeviceInfo>,
    /// project <- pass.
    pub transform: Transform,
    pub alignment: AlignState,
    /// Merge weight by expected accuracy.
    pub weight: f32,
    /// Used by Merge.
    pub included: bool,
    pub start: PassStart,
    pub raw_recording: Option<String>,
}

impl PassMeta {
    pub fn new(scan_config: ScanConfig, device: Option<DeviceInfo>, start: PassStart) -> Self {
        PassMeta {
            weight: default_weight(scan_config.light),
            scan_config,
            device,
            transform: Transform::default(),
            alignment: AlignState::Unaligned,
            included: true,
            start,
            raw_recording: std::env::var("SCANCRAFT_RECORD_DIR").ok(),
        }
    }
}

/// Merge weight of a light source: laser lines > Full-field HD > VCSEL.
pub fn default_weight(light: LightSource) -> f32 {
    match light {
        LightSource::CrossLines | LightSource::SingleLine => 3.0,
        LightSource::FullFieldHd | LightSource::Turntable => 2.0,
        LightSource::HybridHd | LightSource::ParallelLines => 1.5,
        LightSource::VcselRapid => 1.0,
    }
}

/// The tracker settings of a pass captured with `scan_config` on `device`.
pub fn tracker_config(scan_config: &ScanConfig, device: Option<&DeviceInfo>) -> TrackerConfig {
    TrackerConfig {
        fusion_voxel_mm: scan_config.accuracy.point_pitch_mm(),
        imu_to_sensor: device.and_then(|d| d.imu_to_sensor).map(Quat::from_array),
        ..TrackerConfig::default()
    }
}

/// Name, folder and save state of the open project.
pub struct ProjectInfo {
    pub name: String,
    pub path: Option<PathBuf>,
    /// The geometry generation when it was last saved or opened.
    pub saved_generation: u64,
}

impl Default for ProjectInfo {
    fn default() -> Self {
        ProjectInfo { name: "Untitled project".into(), path: None, saved_generation: 0 }
    }
}

fn is_identity(t: &Transform) -> bool {
    t.translation.length_squared() < 1e-12 && t.rotation.angle_between(Quat::IDENTITY) < 1e-7
}

/// Points of a pass's own frame placed in the project frame.
pub fn place(points: Vec<Point3D>, t: &Transform) -> Vec<Point3D> {
    if is_identity(t) {
        return points;
    }
    let m = t.to_mat4();
    points
        .into_iter()
        .map(|mut p| {
            p.position = m.transform_point3(Vec3::from(p.position)).to_array();
            p.normal = (t.rotation * Vec3::from(p.normal)).to_array();
            p
        })
        .collect()
}

/// A pass as the alignment code sees it, in its own frame: the fused surface and a spread of
/// kept keyframes as views (for the visibility check).
pub fn pass_cloud(tracker: &Tracker) -> PassCloud {
    let points = tracker.fused_surfels().iter().map(|s| (s.position, s.normal.normalize_or_zero())).collect();
    let kept: Vec<&Keyframe> = tracker.keyframes().iter().filter(|k| !k.discarded).collect();
    let stride = (kept.len() / 32).max(1);
    let views = kept
        .iter()
        .step_by(stride)
        .map(|k| {
            let pts = k.sensor_points();
            let s = (pts.len() / 40_000).max(1);
            View { pose: k.pose, points: pts.iter().step_by(s).map(|(p, _)| *p).collect() }
        })
        .collect();
    PassCloud { points, views }
}

/// The model of placed passes as a seed for a same-position pass: surface points and keyframe
/// views, in the project frame.
pub fn seed_from(passes: &[(Transform, &Tracker)]) -> (Vec<(Vec3, Vec3)>, Vec<SeedView>) {
    let mut points = Vec::new();
    let mut views = Vec::new();
    for (t, tracker) in passes {
        let m = t.to_mat4();
        points.extend(tracker.fused_surfels().iter().map(|s| (m.transform_point3(s.position), t.rotation * s.normal)));
        views.extend(tracker.keyframes().iter().filter(|k| !k.discarded).map(|k| SeedView { pose: scancraft_align::compose(t, &k.pose), imu: k.imu(), centroid: k.centroid() }));
    }
    (points, views)
}

// ---------------------------------------------------------------------- persistence

/// One pass as saved or loaded.
pub struct StoredPass {
    pub id: u32,
    pub name: String,
    pub frames: u64,
    pub visible: bool,
    pub meta: PassMeta,
    pub keyframes: Vec<KeyframeData>,
}

/// What a project folder holds.
pub struct StoredProject {
    pub name: String,
    pub reference: Option<u32>,
    pub next_pass_id: u32,
    pub passes: Vec<StoredPass>,
    pub merged: Option<(PointCloud, Vec<u32>)>,
    pub cleaned: Option<PointCloud>,
    pub mesh: Option<Mesh>,
    pub settings: serde_json::Value,
}

fn to_record(k: &KeyframeData) -> KeyframeRecord {
    KeyframeRecord {
        id: k.id,
        rotation: k.pose.rotation.to_array(),
        translation: k.pose.translation.to_array(),
        rmse_mm: k.rmse_mm,
        visibility_conflict: k.visibility_conflict,
        new_points: k.new_points as u64,
        discarded: k.discarded,
        imu: k.imu.map(|q| q.to_array()),
        centroid: k.centroid.to_array(),
        points: k.points.clone(),
    }
}

fn from_record(r: KeyframeRecord) -> KeyframeData {
    KeyframeData {
        id: r.id,
        pose: Transform { rotation: Quat::from_array(r.rotation), translation: Vec3::from(r.translation), scale: Vec3::ONE },
        rmse_mm: r.rmse_mm,
        visibility_conflict: r.visibility_conflict,
        new_points: r.new_points as usize,
        discarded: r.discarded,
        imu: r.imu.map(Quat::from_array),
        centroid: Vec3::from(r.centroid),
        points: r.points,
    }
}

fn alignment_record(a: AlignState) -> AlignmentRecord {
    let q = |q: Option<Quality>| (q.map(|q| q.overlap), q.map(|q| q.rmse_mm).filter(|r| r.is_finite()), q.map(|q| q.visibility_conflict));
    match a {
        AlignState::Reference => AlignmentRecord { status: "reference".into(), method: Some("reference".into()), ..Default::default() },
        AlignState::Aligned { method, quality } => {
            let (overlap, rmse_mm, visibility_conflict) = q(quality);
            AlignmentRecord { status: "aligned".into(), method: Some(method.key().into()), overlap, rmse_mm, visibility_conflict }
        }
        AlignState::Unaligned | AlignState::Locating => AlignmentRecord { status: "unaligned".into(), ..Default::default() },
    }
}

fn alignment_state(r: &AlignmentRecord) -> AlignState {
    match r.status.as_str() {
        "reference" => AlignState::Reference,
        "aligned" => {
            let quality = r.overlap.map(|overlap| Quality {
                overlap,
                rmse_mm: r.rmse_mm.unwrap_or(f32::INFINITY),
                visibility_conflict: r.visibility_conflict.unwrap_or(0.0),
                inliers: 1000,
            });
            AlignState::Aligned { method: AlignMethod::from_key(r.method.as_deref().unwrap_or("auto")), quality }
        }
        _ => AlignState::Unaligned,
    }
}

const MERGED_FILE: &str = "outputs/merged.ply";
const CLEANED_FILE: &str = "outputs/cleaned.ply";
const MESH_FILE: &str = "outputs/mesh.ply";

/// Writes the project into `dir`.
pub fn save(dir: &Path, project: &StoredProject) -> Result<(), String> {
    let e = |e: scancraft_io::IoError| e.to_string();
    let passes = project
        .passes
        .iter()
        .map(|p| PassRecord {
            id: p.id,
            name: p.name.clone(),
            scan_config: serde_json::to_value(&p.meta.scan_config).unwrap_or_default(),
            device: p.meta.device.as_ref().and_then(|d| serde_json::to_value(d).ok()).unwrap_or_default(),
            frames: p.frames,
            transform: p.meta.transform,
            alignment: alignment_record(p.meta.alignment),
            weight: p.meta.weight,
            included: p.meta.included,
            visible: p.visible,
            start: match p.meta.start {
                PassStart::Moved => "moved".into(),
                PassStart::SamePosition => "same_position".into(),
            },
            keyframes: files::keyframes_path(p.id),
            raw_recording: p.meta.raw_recording.clone(),
        })
        .collect();
    let mut outputs = Vec::new();
    std::fs::create_dir_all(dir.join("outputs")).map_err(|e| e.to_string())?;
    // Outputs that no longer exist are removed so an opened project shows what was saved.
    for (file, present) in [(MERGED_FILE, project.merged.is_some()), (CLEANED_FILE, project.cleaned.is_some()), (MESH_FILE, project.mesh.is_some())] {
        if !present {
            let _ = std::fs::remove_file(dir.join(file));
        }
    }
    if let Some((cloud, from)) = &project.merged {
        scancraft_io::save_ply_binary(cloud, dir.join(MERGED_FILE)).map_err(e)?;
        outputs.push(OutputRecord { kind: "merged".into(), file: MERGED_FILE.into(), points: cloud.len(), passes: from.clone() });
    }
    if let Some(cloud) = &project.cleaned {
        scancraft_io::save_ply_binary(cloud, dir.join(CLEANED_FILE)).map_err(e)?;
        outputs.push(OutputRecord { kind: "cleaned".into(), file: CLEANED_FILE.into(), points: cloud.len(), passes: Vec::new() });
    }
    if let Some(mesh) = &project.mesh {
        scancraft_io::save_mesh_ply_binary(mesh, dir.join(MESH_FILE)).map_err(e)?;
        outputs.push(OutputRecord { kind: "mesh".into(), file: MESH_FILE.into(), points: mesh.vertices.len(), passes: Vec::new() });
    }
    let file = ProjectFile {
        format: files::FORMAT_VERSION,
        name: project.name.clone(),
        reference_pass: project.reference,
        next_pass_id: project.next_pass_id,
        passes,
        outputs,
        settings: project.settings.clone(),
    };
    let keyframes: Vec<(u32, Vec<KeyframeRecord>)> = project.passes.iter().map(|p| (p.id, p.keyframes.iter().map(to_record).collect())).collect();
    files::save_project(dir, &file, &keyframes).map_err(e)
}

/// Reads a project folder.
pub fn load(dir: &Path) -> Result<StoredProject, String> {
    if !files::is_project(dir) {
        return Err(format!("{} holds no {}", dir.display(), files::PROJECT_FILE));
    }
    let (file, mut keyframes) = files::load_project(dir).map_err(|e| e.to_string())?;
    let mut passes = Vec::new();
    for record in &file.passes {
        let kfs = keyframes.iter_mut().find(|(id, _)| *id == record.id).map(|(_, k)| std::mem::take(k)).unwrap_or_default();
        let scan_config: ScanConfig = serde_json::from_value(record.scan_config.clone()).unwrap_or_default();
        let device: Option<DeviceInfo> = serde_json::from_value(record.device.clone()).ok();
        passes.push(StoredPass {
            id: record.id,
            name: record.name.clone(),
            frames: record.frames,
            visible: record.visible,
            meta: PassMeta {
                scan_config,
                device,
                transform: record.transform,
                alignment: alignment_state(&record.alignment),
                weight: record.weight,
                included: record.included,
                start: if record.start == "same_position" { PassStart::SamePosition } else { PassStart::Moved },
                raw_recording: record.raw_recording.clone(),
            },
            keyframes: kfs.into_iter().map(from_record).collect(),
        });
    }
    let output = |kind: &str| file.outputs.iter().find(|o| o.kind == kind);
    let cloud = |o: &OutputRecord| scancraft_io::load_ply(files::resolve(dir, &o.file)).ok().map(|(c, _)| c);
    let merged = output("merged").and_then(|o| cloud(o).map(|c| (c, o.passes.clone())));
    let cleaned = output("cleaned").and_then(cloud);
    let mesh = output("mesh").and_then(|o| scancraft_io::load_ply(files::resolve(dir, &o.file)).ok()).and_then(|(_, m)| m);
    Ok(StoredProject { name: file.name, reference: file.reference_pass, next_pass_id: file.next_pass_id, passes, merged, cleaned, mesh, settings: file.settings })
}

/// A loaded pass with its tracker rebuilt and its surface placed in the project frame.
pub struct LoadedPass {
    pub stored: StoredPass,
    pub tracker: Tracker,
    pub points: Vec<Point3D>,
    pub keyframes: Vec<KeyframeInfo>,
}

/// Rebuilds a stored pass's tracker (the expensive part of opening a project).
pub fn rebuild(mut stored: StoredPass) -> LoadedPass {
    let config = tracker_config(&stored.meta.scan_config, stored.meta.device.as_ref());
    let kfs = std::mem::take(&mut stored.keyframes).into_iter().map(Keyframe::from_data).collect();
    let tracker = Tracker::from_keyframes(config, kfs);
    let points = place(tracker.fused_points(), &stored.meta.transform);
    let keyframes = tracker.keyframes().iter().map(KeyframeInfo::of).collect();
    LoadedPass { stored, tracker, points, keyframes }
}

#[cfg(test)]
mod tests {
    use super::*;
    use scancraft_align::synthetic;
    use scancraft_core::PointCloud;

    /// A tracker fed with synthetic views of the test object.
    fn tracker_for(views: &[Transform]) -> Tracker {
        let obj = synthetic::object(1.0);
        let mut rng = scancraft_align::cloud::Rng::new(1);
        let mut t = Tracker::new();
        for pose in views {
            let seen = synthetic::observe(&obj, pose, 0.1, &mut rng);
            let cloud = PointCloud::from_points(seen.iter().map(|(p, n)| Point3D::new(p.to_array()).with_normal(n.to_array())).collect());
            t.register_frame(&cloud);
        }
        t
    }

    #[test]
    fn project_round_trips_through_a_folder() {
        let dir = std::env::temp_dir().join(format!("scancraft-app-project-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        let views = synthetic::ring(Vec3::ZERO, 0.6, 0.0, 0.5, 6, 260.0);
        let mut tracker = tracker_for(&views);
        let first = tracker.keyframes()[1].id;
        tracker.set_discarded(first, true);
        let mut aligned = PassMeta::new(ScanConfig::default(), None, PassStart::Moved);
        aligned.transform = scancraft_align::rigid(Quat::from_rotation_x(2.0), Vec3::new(1.0, 2.0, 3.0));
        aligned.alignment = AlignState::Aligned { method: AlignMethod::PointPairs, quality: Some(Quality { overlap: 0.4, rmse_mm: 0.2, visibility_conflict: 0.01, inliers: 1000 }) };
        aligned.weight = 3.0;
        aligned.included = false;
        let mut reference = PassMeta::new(ScanConfig::default(), None, PassStart::Moved);
        reference.alignment = AlignState::Reference;
        let keyframes: Vec<KeyframeData> = tracker.keyframes().iter().map(Keyframe::to_data).collect();
        let merged = PointCloud::from_points(vec![Point3D::new([1.0, 2.0, 3.0])]);
        let project = StoredProject {
            name: "Test".into(),
            reference: Some(1),
            next_pass_id: 3,
            passes: vec![
                StoredPass { id: 1, name: "Pass 1".into(), frames: 10, visible: true, meta: reference, keyframes: keyframes.clone() },
                StoredPass { id: 2, name: "Flipped".into(), frames: 20, visible: false, meta: aligned.clone(), keyframes: keyframes.clone() },
            ],
            merged: Some((merged, vec![1])),
            cleaned: None,
            mesh: None,
            settings: serde_json::json!({ "min_mm": 150.0 }),
        };
        save(&dir, &project).unwrap();
        let loaded = load(&dir).unwrap();
        assert_eq!(loaded.name, "Test");
        assert_eq!(loaded.reference, Some(1));
        assert_eq!(loaded.next_pass_id, 3);
        assert_eq!(loaded.passes.len(), 2);
        let p = &loaded.passes[1];
        assert_eq!((p.name.as_str(), p.frames, p.visible), ("Flipped", 20, false));
        assert_eq!(p.meta.transform, aligned.transform);
        assert_eq!(p.meta.alignment, aligned.alignment);
        assert_eq!((p.meta.weight, p.meta.included), (3.0, false));
        assert_eq!(p.keyframes, keyframes);
        assert_eq!(loaded.merged.as_ref().map(|(c, from)| (c.len(), from.clone())), Some((1, vec![1])));
        // The rebuilt tracker gives the same model, placed by the pass's transform.
        let rebuilt = rebuild(loaded.passes.into_iter().nth(1).unwrap());
        assert_eq!(rebuilt.tracker.fused_points().len(), tracker.fused_points().len());
        assert!(rebuilt.keyframes[1].discarded);
        let expected = place(tracker.fused_points(), &aligned.transform);
        assert!((Vec3::from(rebuilt.points[0].position) - Vec3::from(expected[0].position)).length() < 1e-3);
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn seed_lets_a_same_position_pass_register_into_the_project_frame() {
        let views = synthetic::ring(Vec3::ZERO, 0.6, 0.0, 0.6, 13, 260.0);
        let earlier = tracker_for(&views);
        // The earlier pass sits in the project frame at some transform.
        let t = scancraft_align::rigid(Quat::from_rotation_z(0.4), Vec3::new(25.0, -10.0, 5.0));
        let (points, seeds) = seed_from(&[(t, &earlier)]);
        let mut next = Tracker::new();
        assert!(next.seed_model(&points, &seeds));
        let obj = synthetic::object(1.0);
        let mut rng = scancraft_align::cloud::Rng::new(9);
        let view = views[5];
        let seen = synthetic::observe(&obj, &view, 0.1, &mut rng);
        let cloud = PointCloud::from_points(seen.iter().map(|(p, n)| Point3D::new(p.to_array()).with_normal(n.to_array())).collect());
        let status = next.register_frame(&cloud);
        assert_eq!(next.seed_state(), scancraft_slam::SeedState::Registered);
        // The earlier tracker's world is its first view's sensor frame; the project frame is
        // that world moved by `t`.
        let truth = scancraft_align::compose(&t, &scancraft_align::compose(&views[0].inverse(), &view));
        let err = (status.current_pose.translation - truth.translation).length();
        // The earlier pass itself tracked to ~0.5 mm (scanner pose, 260 mm from the object).
        assert!(err < 2.0, "pose error {err:.3} mm");
    }
}
