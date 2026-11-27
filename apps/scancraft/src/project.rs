//! The project (`docs/architecture/scan-sessions.md`): each pass's mode, device snapshot and
//! alignment into the project frame, and saving / opening the project as a folder.

use std::path::{Path, PathBuf};
use std::sync::Arc;

use glam::{Quat, Vec3};
use scancraft_align::{PassCloud, Quality, View};
use scancraft_core::{Mesh, Point3D, PointCloud, Transform};
use scancraft_driver::{DeviceInfo, LightSource, ScanConfig};
use scancraft_io::project::{self as files, AlignmentRecord, KeyframeRecord, MarkerRecord, OutputRecord, PassRecord, ProjectFile};
use scancraft_slam::{Keyframe, KeyframeData, SeedView, Tracker, TrackerConfig};

use crate::model::{KeyframeInfo, SharedTracker};

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
    /// Placed by Align All (every pass against every other).
    AllPasses,
}

impl AlignMethod {
    pub fn label(self) -> &'static str {
        match self {
            AlignMethod::Auto => "Automatic",
            AlignMethod::PointPairs => "Point pairs",
            AlignMethod::Markers => "Markers",
            AlignMethod::SamePosition => "Same position",
            AlignMethod::Refined => "Refined",
            AlignMethod::AllPasses => "Align all",
        }
    }

    fn key(self) -> &'static str {
        match self {
            AlignMethod::Auto => "auto",
            AlignMethod::PointPairs => "point_pairs",
            AlignMethod::Markers => "markers",
            AlignMethod::SamePosition => "same_position",
            AlignMethod::Refined => "refined",
            AlignMethod::AllPasses => "align_all",
        }
    }

    fn from_key(key: &str) -> Self {
        match key {
            "point_pairs" => AlignMethod::PointPairs,
            "markers" => AlignMethod::Markers,
            "same_position" => AlignMethod::SamePosition,
            "refined" => AlignMethod::Refined,
            "align_all" => AlignMethod::AllPasses,
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

/// Where a pass came from.
#[derive(Clone, Debug, PartialEq, Eq, Default)]
pub enum PassOrigin {
    /// Captured in this project.
    #[default]
    Captured,
    /// Read from a scan file (PLY, OBJ, STL): a fixed surface without tracker or keyframes.
    /// `source` is the file name.
    File { source: String },
    /// Copied from another ScanCraft project (with its keyframes). `source` is that project's
    /// name.
    Project { source: String },
}

impl PassOrigin {
    /// Brought in from outside this project (a file or another project).
    pub fn is_imported(&self) -> bool {
        !matches!(self, PassOrigin::Captured)
    }

    /// One line for tooltips and the properties card.
    pub fn label(&self) -> String {
        match self {
            PassOrigin::Captured => "Captured".into(),
            PassOrigin::File { source } => format!("Imported from {source}"),
            PassOrigin::Project { source } => format!("Added from project {source}"),
        }
    }
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
    pub origin: PassOrigin,
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
            origin: PassOrigin::Captured,
        }
    }

    /// A pass read from the scan file `source` (unaligned, default merge weight).
    pub fn imported(source: String) -> Self {
        PassMeta { raw_recording: None, origin: PassOrigin::File { source }, ..PassMeta::new(ScanConfig::default(), None, PassStart::Moved) }
    }
}

/// One marker sticker of a pass's marker map, in the pass frame (mm). It comes from the tracker
/// while capturing (centre only) or from the marker file next to an imported scan (Revo's
/// `marker_framework.mkf`, with normal and radius). Saved with the project
/// (`pass-<id>/markers.json`), shown when the pass is selected and used by Make Solid.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PassMarker {
    pub centre: Vec3,
    /// Unit normal out of the surface, when the source gives one.
    pub normal: Option<Vec3>,
    pub radius_mm: Option<f32>,
    /// Matched in the newest frame while capturing (not saved).
    pub matched: bool,
}

/// Two marker centres closer than this (mm) are one sticker (stickers are about 6 mm across, so
/// two different ones are farther apart).
pub const SAME_MARKER_MM: f32 = 3.0;

impl PassMarker {
    /// A marker of the tracker's map.
    pub fn tracked(centre: [f32; 3], matched: bool) -> Self {
        PassMarker { centre: Vec3::from_array(centre), normal: None, radius_mm: None, matched }
    }

    /// A marker of Revo's marker file.
    pub fn from_revo(m: &scancraft_io::RevoMarker) -> Self {
        let normal = Vec3::from_array(m.normal.map(|v| v as f32)).normalize_or_zero();
        PassMarker {
            centre: Vec3::from_array(m.centre.map(|v| v as f32)),
            normal: (normal != Vec3::ZERO).then_some(normal),
            radius_mm: (m.radius_mm > 0.0).then_some(m.radius_mm),
            matched: false,
        }
    }

    /// The marker with the pass's coordinates multiplied by `factor` (see `imports::scaled`).
    pub fn scaled(&self, factor: f32) -> Self {
        PassMarker {
            centre: self.centre * factor,
            normal: self.normal.map(|n| if factor < 0.0 { -n } else { n }),
            radius_mm: self.radius_mm.map(|r| r * factor.abs()),
            matched: self.matched,
        }
    }

    /// A pass's marker map while capturing: the markers it had when the capture started (`base`,
    /// kept as they are) and those of the capture's tracker map (`tracked`, pass frame) that are
    /// not within [`SAME_MARKER_MM`] of one of them. A base marker counts as matched when a
    /// matched tracker marker lies that close. The capture's tracker starts with an empty map, so
    /// on Continue Pass its map alone would drop every marker the pass already had.
    pub fn merged(base: &[PassMarker], tracked: &[([f32; 3], bool)]) -> Vec<PassMarker> {
        let near = |a: Vec3, b: Vec3| a.distance_squared(b) < SAME_MARKER_MM * SAME_MARKER_MM;
        let mut out: Vec<PassMarker> = base
            .iter()
            .map(|m| PassMarker { matched: tracked.iter().any(|(c, matched)| *matched && near(m.centre, Vec3::from_array(*c))), ..*m })
            .collect();
        out.extend(tracked.iter().filter(|(c, _)| !base.iter().any(|m| near(m.centre, Vec3::from_array(*c)))).map(|(c, matched)| PassMarker::tracked(*c, *matched)));
        out
    }

    fn record(&self) -> MarkerRecord {
        MarkerRecord { centre: self.centre.to_array(), normal: self.normal.map(|n| n.to_array()), radius_mm: self.radius_mm }
    }

    fn from_record(r: &MarkerRecord) -> Self {
        let normal = r.normal.map(Vec3::from_array).map(Vec3::normalize_or_zero).filter(|n| *n != Vec3::ZERO);
        PassMarker { centre: Vec3::from_array(r.centre), normal, radius_mm: r.radius_mm.filter(|v| v.is_finite() && *v > 0.0), matched: false }
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

/// Name, place on disk and save state of the open project.
pub struct ProjectInfo {
    pub name: String,
    /// The project folder, or the bundle file of a bundle tab; `None` until first saved.
    pub path: Option<PathBuf>,
    /// The geometry generation when it was last saved or opened.
    pub saved_generation: u64,
    /// Its lock, and a bundle's working folder (`crate::storage`).
    pub store: crate::storage::Store,
}

impl Default for ProjectInfo {
    fn default() -> Self {
        ProjectInfo { name: "Untitled project".into(), path: None, saved_generation: 0, store: crate::storage::Store::None }
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
    // The table plane comes from object isolation (`crate::isolation`).
    PassCloud { points, views, support: None }
}

/// A fixed surface (an imported scan) as the alignment code sees it: its points, and no views,
/// so the visibility checks are skipped for it.
pub fn fixed_cloud(surface: &PointCloud) -> PassCloud {
    let points = surface.points.iter().map(|p| (Vec3::from(p.position), Vec3::from(p.normal).normalize_or_zero())).collect();
    PassCloud { points, views: Vec::new(), support: None }
}

/// Observations credited to each point of a fixed surface when merging: an imported scan is
/// already a fused surface, so its points are not treated as single stray measurements.
pub const FIXED_OBSERVATIONS: u32 = 2;

/// A cleaned point's observation count, kept in `Point3D::confidence`.
fn observations(p: &Point3D) -> u32 {
    (p.confidence.round() as i64).clamp(1, u32::MAX as i64) as u32
}

/// Where background jobs read a pass's surface from (in the pass's own frame).
#[derive(Clone)]
pub enum PassSource {
    /// A captured pass: its tracker's fused model and keyframes.
    Tracked(SharedTracker),
    /// An imported scan: a fixed surface.
    Fixed(Arc<PointCloud>),
    /// A cleaned pass: its edited surface (observation counts in `confidence`), and for a
    /// captured pass its tracker, whose keyframes still give the views.
    Edited { surface: Arc<PointCloud>, tracker: Option<SharedTracker> },
}

impl PassSource {
    /// The pass for alignment (see [`pass_cloud`] and [`fixed_cloud`]).
    pub fn pass_cloud(&self) -> Result<PassCloud, String> {
        match self {
            PassSource::Tracked(t) => t.lock().map(|t| pass_cloud(&t)).map_err(|_| "A pass's tracker is unavailable".to_string()),
            PassSource::Fixed(s) => Ok(fixed_cloud(s)),
            PassSource::Edited { surface, tracker } => {
                let mut cloud = fixed_cloud(surface);
                if let Some(t) = tracker {
                    cloud.views = t.lock().map(|t| pass_cloud(&t).views).map_err(|_| "A pass's tracker is unavailable".to_string())?;
                }
                Ok(cloud)
            }
        }
    }

    /// (position, unit normal, observation count) of each surface point, for merging. The order
    /// matches [`pass_cloud`](Self::pass_cloud)'s points.
    pub fn surfels(&self) -> Result<Vec<(Vec3, Vec3, u32)>, String> {
        match self {
            PassSource::Tracked(t) => t
                .lock()
                .map(|t| t.fused_surfels().iter().map(|s| (s.position, s.normal.normalize_or_zero(), s.count)).collect())
                .map_err(|_| "A pass's tracker is unavailable".to_string()),
            PassSource::Fixed(s) => Ok(fixed_cloud(s).points.into_iter().map(|(p, n)| (p, n, FIXED_OBSERVATIONS)).collect()),
            PassSource::Edited { surface, .. } => {
                Ok(surface.points.iter().map(|p| (Vec3::from(p.position), Vec3::from(p.normal).normalize_or_zero(), observations(p))).collect())
            }
        }
    }

    /// The surface as points to clean, in the pass's own frame, with each point's observation
    /// count in `confidence` (what a pass's `edit` holds).
    pub fn editable(&self) -> Result<PointCloud, String> {
        match self {
            PassSource::Tracked(t) => {
                let surfels = t.lock().map(|t| t.fused_surfels()).map_err(|_| "A pass's tracker is unavailable".to_string())?;
                Ok(PointCloud::from_points(
                    surfels
                        .iter()
                        .map(|s| {
                            let mut p = Point3D::new(s.position.to_array()).with_normal(s.normal.normalize_or_zero().to_array()).with_color(scancraft_core::ColorRgba::GRAY);
                            p.confidence = s.count as f32;
                            p
                        })
                        .collect(),
                ))
            }
            PassSource::Fixed(s) => {
                let mut cloud = (**s).clone();
                for p in &mut cloud.points {
                    p.confidence = FIXED_OBSERVATIONS as f32;
                }
                Ok(cloud)
            }
            PassSource::Edited { surface, .. } => Ok((**surface).clone()),
        }
    }
}

/// The model of placed passes as a seed for a same-position pass: surface points and keyframe
/// views, in the project frame.
pub fn seed_from(passes: &[(Transform, &Tracker)]) -> (Vec<(Vec3, Vec3)>, Vec<SeedView>) {
    let mut points = Vec::new();
    let mut views = Vec::new();
    for (t, tracker) in passes {
        let m = t.to_mat4();
        points.extend(tracker.fused_surfels().iter().map(|s| (m.transform_point3(s.position), t.rotation * s.normal)));
        views.extend(seed_views(t, tracker));
    }
    (points, views)
}

/// A pass's kept keyframes as seed views in the project frame.
pub fn seed_views(t: &Transform, tracker: &Tracker) -> Vec<SeedView> {
    tracker.keyframes().iter().filter(|k| !k.discarded).map(|k| SeedView { pose: scancraft_align::compose(t, &k.pose), imu: k.imu(), centroid: k.centroid() }).collect()
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
    /// An imported pass's fixed surface (its own frame); `None` for a captured pass.
    pub surface: Option<Arc<PointCloud>>,
    /// The cleaned surface (own frame, observation counts in `confidence`), if cleaned.
    pub edit: Option<Arc<PointCloud>>,
    /// Calibrated RGB keyframes recorded during this pass.
    pub rgb_keyframes: Vec<scancraft_color::RgbKeyframe>,
    /// Which surfels are the object (`crate::isolation`), if it was computed. Saved so an opened
    /// project does not segment every pass again; its fingerprint decides whether it still fits.
    pub mask: Option<Arc<crate::isolation::ObjectMask>>,
    /// Its marker map (empty: none).
    pub markers: Vec<PassMarker>,
}

/// What a project folder holds.
pub struct StoredProject {
    pub name: String,
    pub reference: Option<u32>,
    pub next_pass_id: u32,
    pub passes: Vec<StoredPass>,
    pub merged: Option<(PointCloud, Vec<u32>)>,
    /// Only read, from projects saved before cleaning became per pass: one cleaned cloud of
    /// all passes. Never written.
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
    match a {
        AlignState::Reference => AlignmentRecord { status: "reference".into(), method: Some("reference".into()), ..Default::default() },
        AlignState::Aligned { method, quality } => {
            let finite = |v: f32| v.is_finite().then_some(v);
            let mut r = AlignmentRecord { status: "aligned".into(), method: Some(method.key().into()), ..Default::default() };
            if let Some(q) = quality {
                r.overlap = Some(q.overlap);
                r.rmse_mm = finite(q.rmse_mm);
                r.visibility_conflict = Some(q.visibility_conflict);
                r.inliers = Some(q.inliers as u64);
                r.below_support = Some(q.below_support);
                r.slide_mm = finite(q.slide_mm);
                r.shell = q.shell.map(|s| files::ShellRecord { pairs: s.pairs as u64, fraction: s.fraction, offset_mm: s.offset_mm, iqr_mm: s.iqr_mm, piercing: s.piercing });
            }
            r
        }
        AlignState::Unaligned | AlignState::Locating => AlignmentRecord { status: "unaligned".into(), ..Default::default() },
    }
}

fn alignment_state(r: &AlignmentRecord) -> AlignState {
    match r.status.as_str() {
        "reference" => AlignState::Reference,
        "aligned" => {
            // Projects saved before 2026-11-25 store only overlap, RMS and see-through; what is
            // missing reads as not measured (no wall, no table violation, slide 0 = not judged).
            let quality = r.overlap.map(|overlap| Quality {
                overlap,
                rmse_mm: r.rmse_mm.unwrap_or(f32::INFINITY),
                visibility_conflict: r.visibility_conflict.unwrap_or(0.0),
                inliers: r.inliers.map_or(1000, |n| n as usize),
                below_support: r.below_support.unwrap_or(0.0),
                slide_mm: r.slide_mm.unwrap_or(if r.inliers.is_some() { f32::INFINITY } else { 0.0 }),
                shell: r.shell.map(|s| scancraft_align::quality::Shell { pairs: s.pairs as usize, fraction: s.fraction, offset_mm: s.offset_mm, iqr_mm: s.iqr_mm, piercing: s.piercing }),
            });
            AlignState::Aligned { method: AlignMethod::from_key(r.method.as_deref().unwrap_or("auto")), quality }
        }
        _ => AlignState::Unaligned,
    }
}

/// An imported pass's surface file, relative to the project folder.
pub fn surface_path(id: u32) -> String {
    files::surface_path(id)
}

/// A cleaned pass's surface file, and the file with its points' observation counts (one
/// little-endian u32 per point, in order), relative to the project folder.
pub fn cleaned_path(id: u32) -> String {
    files::cleaned_path(id)
}

/// A pass's saved object labels, relative to the project folder.
pub fn mask_path(id: u32) -> String {
    files::object_labels_path(id)
}

/// A pass's marker map: [`files::markers_path`] (optional, like the object labels).
fn read_markers(dir: &Path, id: u32) -> Vec<PassMarker> {
    let path = dir.join(files::markers_path(id));
    if !path.is_file() {
        return Vec::new();
    }
    match files::read_markers(&path) {
        Ok(records) => records.iter().map(PassMarker::from_record).collect(),
        Err(e) => {
            log::warn!("the marker map of pass {id} is not used: {e}");
            Vec::new()
        }
    }
}

pub fn cleaned_counts_path(id: u32) -> String {
    files::cleaned_counts_path(id)
}

/// Key in a pass's `scan_config` JSON naming its cleaned surface file. Older versions ignore it
/// (and open the pass uncleaned).
const CLEANED_KEY: &str = "cleaned";

/// Key in a pass's `scan_config` JSON naming its saved RGB keyframe manifest.
const RGB_KEYFRAMES_KEY: &str = "rgb_keyframes";

fn rgb_dir_rel(id: u32) -> String {
    files::rgb_dir(id)
}

fn rgb_manifest_rel(id: u32) -> String {
    files::rgb_manifest_path(id)
}

#[derive(serde::Serialize, serde::Deserialize)]
struct RgbKeyframeManifestEntry {
    id: u64,
    timestamp_ms: u64,
    jpeg_file: String,
    width: u32,
    height: u32,
    camera: scancraft_core::ColorCamera,
    sensor_pose: Transform,
    camera_pose: Transform,
    sharpness: f32,
    depth_frame_index: u64,
}

fn write_rgb_keyframes(dir: &Path, id: u32, kfs: &[scancraft_color::RgbKeyframe]) -> Result<(), String> {
    let rgb_dir = dir.join(rgb_dir_rel(id));
    if kfs.is_empty() {
        let _ = std::fs::remove_dir_all(&rgb_dir);
        return Ok(());
    }
    std::fs::create_dir_all(&rgb_dir).map_err(|e| e.to_string())?;
    let mut entries = Vec::with_capacity(kfs.len());
    for kf in kfs {
        let jpeg_file = format!("kf-{:06}.jpg", kf.id);
        std::fs::write(rgb_dir.join(&jpeg_file), &*kf.jpeg).map_err(|e| e.to_string())?;
        entries.push(RgbKeyframeManifestEntry {
            id: kf.id,
            timestamp_ms: kf.timestamp_ms,
            jpeg_file,
            width: kf.width,
            height: kf.height,
            camera: (*kf.camera).clone(),
            sensor_pose: kf.sensor_pose,
            camera_pose: kf.camera_pose,
            sharpness: kf.sharpness,
            depth_frame_index: kf.depth_frame_index,
        });
    }
    let json = serde_json::to_string_pretty(&entries).map_err(|e| e.to_string())?;
    std::fs::write(dir.join(rgb_manifest_rel(id)), json).map_err(|e| e.to_string())
}

fn read_rgb_keyframes(dir: &Path, manifest_rel: &str) -> Result<Vec<scancraft_color::RgbKeyframe>, String> {
    let manifest_path = files::resolve(dir, manifest_rel);
    let rgb_dir = manifest_path.parent().ok_or_else(|| "invalid RGB manifest path".to_string())?;
    let json = std::fs::read_to_string(&manifest_path).map_err(|e| format!("{}: {e}", manifest_path.display()))?;
    let entries: Vec<RgbKeyframeManifestEntry> = serde_json::from_str(&json).map_err(|e| format!("{}: {e}", manifest_path.display()))?;
    let mut out = Vec::with_capacity(entries.len());
    for e in entries {
        let jpeg_path = rgb_dir.join(&e.jpeg_file);
        let jpeg = std::fs::read(&jpeg_path).map_err(|err| format!("{}: {err}", jpeg_path.display()))?;
        out.push(scancraft_color::RgbKeyframe {
            id: e.id,
            timestamp_ms: e.timestamp_ms,
            jpeg: Arc::new(jpeg),
            width: e.width,
            height: e.height,
            camera: Arc::new(e.camera),
            sensor_pose: e.sensor_pose,
            camera_pose: e.camera_pose,
            sharpness: e.sharpness,
            depth_frame_index: e.depth_frame_index,
        });
    }
    Ok(out)
}

/// Key in a pass's free-form `scan_config` JSON that says where the pass came from. Projects
/// without it (all older ones) hold captured passes only; older versions of the app ignore it.
const ORIGIN_KEY: &str = "origin";

/// The pass's `scan_config` JSON with its origin added (nothing for a captured pass).
fn scan_config_json(p: &StoredPass) -> serde_json::Value {
    let mut value = serde_json::to_value(&p.meta.scan_config).unwrap_or_default();
    let origin = match &p.meta.origin {
        PassOrigin::Captured => None,
        PassOrigin::File { source } => Some(serde_json::json!({ "kind": "file", "source": source, "surface": surface_path(p.id) })),
        PassOrigin::Project { source } => Some(serde_json::json!({ "kind": "project", "source": source })),
    };
    if let (Some(origin), Some(map)) = (origin, value.as_object_mut()) {
        map.insert(ORIGIN_KEY.into(), origin);
    }
    if let (true, Some(map)) = (p.edit.is_some(), value.as_object_mut()) {
        map.insert(CLEANED_KEY.into(), serde_json::json!(cleaned_path(p.id)));
    }
    if let (false, Some(map)) = (p.rgb_keyframes.is_empty(), value.as_object_mut()) {
        map.insert(RGB_KEYFRAMES_KEY.into(), serde_json::json!(rgb_manifest_rel(p.id)));
    }
    value
}

/// Writes a cleaned surface and its observation counts.
fn write_cleaned(dir: &Path, id: u32, edit: &PointCloud) -> Result<(), String> {
    write_surface(&dir.join(cleaned_path(id)), edit)?;
    let bytes: Vec<u8> = edit.points.iter().flat_map(|p| observations(p).to_le_bytes()).collect();
    std::fs::write(dir.join(cleaned_counts_path(id)), bytes).map_err(|e| e.to_string())
}

/// Reads a cleaned surface; without a matching counts file every point counts as a fused one.
fn read_cleaned(dir: &Path, id: u32, file: &str) -> Result<PointCloud, String> {
    let mut cloud = read_surface(&files::resolve(dir, file))?;
    let counts = std::fs::read(dir.join(cleaned_counts_path(id))).ok().filter(|b| b.len() == cloud.len() * 4);
    for (i, p) in cloud.points.iter_mut().enumerate() {
        p.confidence = match &counts {
            Some(b) => u32::from_le_bytes([b[4 * i], b[4 * i + 1], b[4 * i + 2], b[4 * i + 3]]) as f32,
            None => FIXED_OBSERVATIONS as f32,
        };
    }
    Ok(cloud)
}

/// The origin stored in a pass's `scan_config` JSON, and for an imported file its surface file.
fn origin_of(scan_config: &serde_json::Value) -> (PassOrigin, Option<String>) {
    let Some(o) = scan_config.get(ORIGIN_KEY) else { return (PassOrigin::Captured, None) };
    let text = |key: &str| o.get(key).and_then(|v| v.as_str()).unwrap_or_default().to_string();
    match o.get("kind").and_then(|v| v.as_str()) {
        Some("file") => (PassOrigin::File { source: text("source") }, Some(text("surface")).filter(|s| !s.is_empty())),
        Some("project") => (PassOrigin::Project { source: text("source") }, None),
        _ => (PassOrigin::Captured, None),
    }
}

/// Writes a fixed surface with its normals and colours (a mesh file without faces).
fn write_surface(path: &Path, surface: &PointCloud) -> Result<(), String> {
    if let Some(parent) = path.parent() {
        std::fs::create_dir_all(parent).map_err(|e| e.to_string())?;
    }
    let mesh = Mesh::from_geometry(surface.points.clone(), Vec::new());
    scancraft_io::save_mesh_ply_binary(&mesh, path).map_err(|e| e.to_string())
}

fn read_surface(path: &Path) -> Result<PointCloud, String> {
    scancraft_io::load_ply(path).map(|(cloud, _)| cloud).map_err(|e| format!("{}: {e}", path.display()))
}

const MERGED_FILE: &str = "outputs/merged.ply";
const CLEANED_FILE: &str = "outputs/cleaned.ply";
const MESH_FILE: &str = "outputs/mesh.ply";

/// Writes the project into `dir`. Fields of the project file this version does not know are
/// kept from the file it replaces (`scancraft_io::project::save_project`).
pub fn save(dir: &Path, project: &StoredProject) -> Result<(), String> {
    save_carrying(dir, project, None)
}

/// [`save`]; when `dir` holds no project yet (the project moves there), the fields this version
/// does not know come from the project at `previous` (where it was until now).
pub fn save_carrying(dir: &Path, project: &StoredProject, previous: Option<&Path>) -> Result<(), String> {
    let e = |e: scancraft_io::IoError| e.to_string();
    let passes = project
        .passes
        .iter()
        .map(|p| PassRecord {
            id: p.id,
            name: p.name.clone(),
            scan_config: scan_config_json(p),
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
            other: Default::default(),
        })
        .collect();
    for p in &project.passes {
        if let Some(surface) = &p.surface {
            write_surface(&dir.join(surface_path(p.id)), surface)?;
        }
        match &p.edit {
            Some(edit) => write_cleaned(dir, p.id, edit)?,
            None => {
                let _ = std::fs::remove_file(dir.join(cleaned_path(p.id)));
                let _ = std::fs::remove_file(dir.join(cleaned_counts_path(p.id)));
            }
        }
        write_rgb_keyframes(dir, p.id, &p.rgb_keyframes)?;
        match &p.mask {
            Some(mask) => {
                let path = dir.join(mask_path(p.id));
                if let Some(parent) = path.parent() {
                    std::fs::create_dir_all(parent).map_err(|e| e.to_string())?;
                }
                std::fs::write(path, mask.to_bytes()).map_err(|e| e.to_string())?;
            }
            None => {
                let _ = std::fs::remove_file(dir.join(mask_path(p.id)));
            }
        }
        let markers = dir.join(files::markers_path(p.id));
        if p.markers.is_empty() {
            let _ = std::fs::remove_file(markers);
        } else {
            let records: Vec<MarkerRecord> = p.markers.iter().map(PassMarker::record).collect();
            files::write_markers(&markers, &records).map_err(|e| e.to_string())?;
        }
    }
    let mut outputs = Vec::new();
    std::fs::create_dir_all(dir.join("outputs")).map_err(|e| e.to_string())?;
    // Outputs that no longer exist are removed so an opened project shows what was saved.
    // The old project-wide cleaned cloud is never written again.
    for (file, present) in [(MERGED_FILE, project.merged.is_some()), (CLEANED_FILE, false), (MESH_FILE, project.mesh.is_some())] {
        if !present {
            let _ = std::fs::remove_file(dir.join(file));
        }
    }
    if let Some((cloud, from)) = &project.merged {
        scancraft_io::save_ply_binary(cloud, dir.join(MERGED_FILE)).map_err(e)?;
        outputs.push(OutputRecord { kind: "merged".into(), file: MERGED_FILE.into(), points: cloud.len(), passes: from.clone() });
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
        other: Default::default(),
    };
    let keyframes: Vec<(u32, Vec<KeyframeRecord>)> = project.passes.iter().map(|p| (p.id, p.keyframes.iter().map(to_record).collect())).collect();
    files::save_project_carrying(dir, &file, &keyframes, previous).map(|_| ()).map_err(e)
}

/// Reads a project: its folder, its `.scancraft` file, or an older folder with `project.json`.
pub fn load(path: &Path) -> Result<StoredProject, String> {
    let location = files::locate(path).map_err(|e| e.to_string())?;
    let dir = location.root.as_path();
    let (file, mut keyframes) = files::load_project(dir).map_err(|e| e.to_string())?;
    let mut passes = Vec::new();
    for record in &file.passes {
        let kfs = keyframes.iter_mut().find(|(id, _)| *id == record.id).map(|(_, k)| std::mem::take(k)).unwrap_or_default();
        let scan_config: ScanConfig = serde_json::from_value(record.scan_config.clone()).unwrap_or_default();
        let device: Option<DeviceInfo> = serde_json::from_value(record.device.clone()).ok();
        let (origin, surface_file) = origin_of(&record.scan_config);
        let surface = match (&origin, surface_file) {
            (PassOrigin::File { .. }, Some(file)) => Some(Arc::new(read_surface(&files::resolve(dir, &file)).map_err(|e| format!("the surface of {} cannot be read ({e})", record.name))?)),
            (PassOrigin::File { .. }, None) => return Err(format!("{} is an imported pass without a surface file", record.name)),
            _ => None,
        };
        let edit = match record.scan_config.get(CLEANED_KEY).and_then(|v| v.as_str()) {
            Some(file) => Some(Arc::new(read_cleaned(dir, record.id, file).map_err(|e| format!("the cleaned surface of {} cannot be read ({e})", record.name))?)),
            None => None,
        };
        let rgb_keyframes = match record.scan_config.get(RGB_KEYFRAMES_KEY).and_then(|v| v.as_str()) {
            Some(file) => read_rgb_keyframes(dir, file).map_err(|e| format!("the colour keyframes of {} cannot be read ({e})", record.name))?,
            None => Vec::new(),
        };
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
                origin,
            },
            keyframes: kfs.into_iter().map(from_record).collect(),
            surface,
            edit,
            rgb_keyframes,
            // Optional: a missing or unreadable file only means it is computed again.
            mask: std::fs::read(dir.join(mask_path(record.id))).ok().and_then(|b| crate::isolation::ObjectMask::from_bytes(&b)).map(Arc::new),
            markers: read_markers(dir, record.id),
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

/// Rebuilds a stored pass's tracker (the expensive part of opening a project). An imported pass
/// gets an empty tracker and its fixed surface.
pub fn rebuild(mut stored: StoredPass) -> LoadedPass {
    let config = tracker_config(&stored.meta.scan_config, stored.meta.device.as_ref());
    let kfs = std::mem::take(&mut stored.keyframes).into_iter().map(Keyframe::from_data).collect();
    let tracker = Tracker::from_keyframes(config, kfs);
    let own = match (&stored.edit, &stored.surface) {
        (Some(edit), _) => edit.points.clone(),
        (None, Some(surface)) => surface.points.clone(),
        (None, None) => tracker.fused_points(),
    };
    let points = place(own, &stored.meta.transform);
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
        aligned.alignment = AlignState::Aligned { method: AlignMethod::AllPasses, quality: Some(Quality { overlap: 0.4, rmse_mm: 0.2, visibility_conflict: 0.01, inliers: 1000, ..Quality::NONE }) };
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
                StoredPass { id: 1, name: "Pass 1".into(), frames: 10, visible: true, meta: reference, keyframes: keyframes.clone(), surface: None, edit: None, rgb_keyframes: Vec::new(), mask: None, markers: Vec::new() },
                StoredPass { id: 2, name: "Flipped".into(), frames: 20, visible: false, meta: aligned.clone(), keyframes: keyframes.clone(), surface: None, edit: None, rgb_keyframes: Vec::new(), mask: None, markers: Vec::new() },
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

    /// A small coloured patch with normals, as an imported scan.
    fn surface() -> Arc<PointCloud> {
        let normal = Vec3::new(-0.1, 0.0, 1.0).normalize().to_array();
        let points = (0..500)
            .map(|i| {
                let (u, v) = ((i % 25) as f32, (i / 25) as f32);
                Point3D::new([u, v, 0.1 * u]).with_normal(normal).with_color(scancraft_core::ColorRgba::new(10 + (i % 200) as u8, 20, 30, 255))
            })
            .collect();
        Arc::new(PointCloud::from_points(points))
    }

    #[test]
    fn imported_and_added_passes_round_trip() {
        let dir = std::env::temp_dir().join(format!("scancraft-app-imported-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        let views = synthetic::ring(Vec3::ZERO, 0.6, 0.0, 0.5, 4, 260.0);
        let keyframes: Vec<KeyframeData> = tracker_for(&views).keyframes().iter().map(Keyframe::to_data).collect();
        let mut reference = PassMeta::new(ScanConfig::default(), None, PassStart::Moved);
        reference.alignment = AlignState::Reference;
        reference.origin = PassOrigin::Project { source: "Bracket".into() };
        let mut imported = PassMeta::imported("shell.ply".into());
        imported.transform = scancraft_align::rigid(Quat::from_rotation_y(0.7), Vec3::new(-4.0, 5.0, 6.0));
        imported.alignment = AlignState::Aligned { method: AlignMethod::AllPasses, quality: None };
        let s = surface();
        let project = StoredProject {
            name: "Imports".into(),
            reference: Some(1),
            next_pass_id: 3,
            passes: vec![
                StoredPass { id: 1, name: "Pass 1".into(), frames: 4, visible: true, meta: reference, keyframes: keyframes.clone(), surface: None, edit: None, rgb_keyframes: Vec::new(), mask: None, markers: Vec::new() },
                StoredPass { id: 2, name: "shell.ply".into(), frames: 0, visible: true, meta: imported.clone(), keyframes: Vec::new(), surface: Some(Arc::clone(&s)), edit: None, rgb_keyframes: Vec::new(), mask: None, markers: Vec::new() },
            ],
            merged: None,
            cleaned: None,
            mesh: None,
            settings: serde_json::json!({}),
        };
        save(&dir, &project).unwrap();
        assert!(dir.join(surface_path(2)).is_file());
        let loaded = load(&dir).unwrap();
        assert_eq!(loaded.passes[0].meta.origin, PassOrigin::Project { source: "Bracket".into() });
        assert!(loaded.passes[0].surface.is_none());
        let p = &loaded.passes[1];
        assert_eq!(p.meta.origin, PassOrigin::File { source: "shell.ply".into() });
        assert_eq!((p.meta.transform, p.meta.alignment), (imported.transform, imported.alignment));
        assert!(p.keyframes.is_empty());
        let back = p.surface.as_ref().expect("the imported surface is stored");
        assert_eq!(back.len(), s.len());
        for (a, b) in back.points.iter().zip(&s.points) {
            assert_eq!(a.position, b.position);
            assert_eq!(a.color, b.color);
            assert!((Vec3::from(a.normal) - Vec3::from(b.normal)).length() < 1e-5);
        }
        // Rebuilt, it is its surface placed by its transform, with an empty tracker.
        let rebuilt = rebuild(loaded.passes.into_iter().nth(1).unwrap());
        assert!(rebuilt.tracker.keyframes().is_empty() && rebuilt.keyframes.is_empty());
        let expected = place(s.points.clone(), &imported.transform);
        assert!((Vec3::from(rebuilt.points[7].position) - Vec3::from(expected[7].position)).length() < 1e-4);
        // A missing surface file is reported, not opened as an empty pass.
        std::fs::remove_file(dir.join(surface_path(2))).unwrap();
        assert!(load(&dir).err().is_some_and(|e| e.contains("shell.ply")));
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn a_cleaned_pass_round_trips_with_its_observation_counts() {
        let dir = std::env::temp_dir().join(format!("scancraft-app-cleaned-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        let mut meta = PassMeta::imported("shell.ply".into());
        meta.alignment = AlignState::Reference;
        let s = surface();
        let mut edit = PointCloud::from_points(s.points[..300].to_vec());
        for (i, p) in edit.points.iter_mut().enumerate() {
            p.confidence = (1 + i % 7) as f32;
        }
        let edit = Arc::new(edit);
        let mut project = StoredProject {
            name: "Cleaned".into(),
            reference: Some(1),
            next_pass_id: 2,
            passes: vec![StoredPass { id: 1, name: "shell.ply".into(), frames: 0, visible: true, meta, keyframes: Vec::new(), surface: Some(Arc::clone(&s)), edit: Some(Arc::clone(&edit)), rgb_keyframes: Vec::new(), mask: None, markers: Vec::new() }],
            merged: None,
            cleaned: None,
            mesh: None,
            settings: serde_json::json!({}),
        };
        save(&dir, &project).unwrap();
        let loaded = load(&dir).unwrap();
        let back = loaded.passes[0].edit.as_ref().expect("the cleaning is stored");
        assert_eq!(back.len(), 300);
        assert!(back.points.iter().zip(&edit.points).all(|(a, b)| a.position == b.position && a.confidence == b.confidence));
        assert_eq!(loaded.passes[0].surface.as_ref().map(|s| s.len()), Some(500), "the imported surface is kept as well");
        let rebuilt = rebuild(loaded.passes.into_iter().next().unwrap());
        assert_eq!(rebuilt.points.len(), 300, "an opened cleaned pass shows its cleaned surface");
        // Saved again without cleaning, the files go and the pass opens as imported.
        project.passes[0].edit = None;
        save(&dir, &project).unwrap();
        assert!(!dir.join(cleaned_path(1)).exists() && !dir.join(cleaned_counts_path(1)).exists());
        assert!(load(&dir).unwrap().passes[0].edit.is_none());
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn alignment_quality_and_object_labels_round_trip() {
        let dir = std::env::temp_dir().join(format!("scancraft-app-quality-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        let mut reference = PassMeta::imported("a.ply".into());
        reference.alignment = AlignState::Reference;
        let quality = Quality {
            overlap: 0.42,
            rmse_mm: 0.13,
            visibility_conflict: 0.02,
            inliers: 5321,
            shell: Some(scancraft_align::quality::Shell { pairs: 900, fraction: 0.3, offset_mm: -3.05, iqr_mm: 0.2, piercing: 0.001 }),
            below_support: 0.004,
            slide_mm: 1.7,
        };
        let mut aligned = PassMeta::imported("b.ply".into());
        aligned.alignment = AlignState::Aligned { method: AlignMethod::AllPasses, quality: Some(quality) };
        let s = surface();
        let object: Vec<bool> = (0..s.len()).map(|i| i % 3 != 0).collect();
        let kept = object.iter().filter(|o| **o).count();
        let mask = crate::isolation::ObjectMask { key: 0xDEAD_BEEF_1234, object, support: Some(scancraft_align::Plane::through(Vec3::new(0.0, 0.0, -1.0), Vec3::Z)), kept, fallback: false };
        let pass = |id: u32, meta: PassMeta, mask: Option<Arc<crate::isolation::ObjectMask>>| StoredPass {
            id,
            name: format!("p{id}"),
            frames: 0,
            visible: true,
            meta,
            keyframes: Vec::new(),
            surface: Some(Arc::clone(&s)),
            edit: None,
            rgb_keyframes: Vec::new(),
            mask,
            markers: Vec::new(),
        };
        let project = StoredProject {
            name: "Quality".into(),
            reference: Some(1),
            next_pass_id: 3,
            passes: vec![pass(1, reference, None), pass(2, aligned, Some(Arc::new(mask.clone())))],
            merged: None,
            cleaned: None,
            mesh: None,
            settings: serde_json::json!({}),
        };
        save(&dir, &project).unwrap();
        let loaded = load(&dir).unwrap();
        match loaded.passes[1].meta.alignment {
            AlignState::Aligned { quality: Some(q), .. } => assert_eq!(q, quality, "every quality field comes back"),
            other => panic!("{other:?}"),
        }
        assert!(loaded.passes[0].mask.is_none());
        assert_eq!(loaded.passes[1].mask.as_deref(), Some(&mask), "the object labels come back");
        // A damaged labels file is ignored (computed again), not an error.
        std::fs::write(dir.join(mask_path(2)), b"junk").unwrap();
        assert!(load(&dir).unwrap().passes[1].mask.is_none());
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn marker_maps_round_trip_and_go_with_their_pass() {
        let dir = std::env::temp_dir().join(format!("scancraft-app-markers-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        let mut meta = PassMeta::imported("raw_preview.ply".into());
        meta.alignment = AlignState::Reference;
        let revo = PassMarker::from_revo(&scancraft_io::RevoMarker { centre: [108.9, 85.9, 260.6], normal: [0.0, 0.6, -0.8], radius_mm: 3.05, id: -1 });
        let tracked = PassMarker::tracked([1.0, 2.0, 3.0], true);
        let s = surface();
        let pass = |id: u32, markers: Vec<PassMarker>| StoredPass {
            id,
            name: format!("p{id}"),
            frames: 0,
            visible: true,
            meta: meta.clone(),
            keyframes: Vec::new(),
            surface: Some(Arc::clone(&s)),
            edit: None,
            rgb_keyframes: Vec::new(),
            mask: None,
            markers,
        };
        let mut project = StoredProject {
            name: "Markers".into(),
            reference: Some(1),
            next_pass_id: 3,
            passes: vec![pass(1, vec![revo, tracked]), pass(2, Vec::new())],
            merged: None,
            cleaned: None,
            mesh: None,
            settings: serde_json::json!({}),
        };
        save(&dir, &project).unwrap();
        assert!(dir.join(files::markers_path(1)).is_file() && !dir.join(files::markers_path(2)).exists());
        let loaded = load(&dir).unwrap();
        let back = &loaded.passes[0].markers;
        assert_eq!(back.len(), 2);
        assert_eq!((back[0].centre, back[0].normal, back[0].radius_mm), (revo.centre, revo.normal, Some(3.05)));
        assert_eq!((back[1].centre, back[1].normal, back[1].radius_mm), (tracked.centre, None, None));
        assert!(!back[1].matched, "the match state of a capture is not saved");
        assert!(loaded.passes[1].markers.is_empty());
        // Rebuilt (Open Project, Add Project) the pass keeps them.
        assert_eq!(rebuild(loaded.passes.into_iter().next().unwrap()).stored.markers.len(), 2);
        // A damaged file loses the map, not the project.
        std::fs::write(dir.join(files::markers_path(1)), b"{").unwrap();
        assert!(load(&dir).unwrap().passes[0].markers.is_empty());
        // Saved without markers, the file goes.
        project.passes[0].markers.clear();
        save(&dir, &project).unwrap();
        assert!(!dir.join(files::markers_path(1)).exists());
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn scaled_markers_follow_their_pass() {
        let m = PassMarker { centre: Vec3::new(0.1, 0.2, 0.3), normal: Some(Vec3::Z), radius_mm: Some(0.003), matched: false };
        let mm = m.scaled(1000.0);
        assert!((mm.centre - Vec3::new(100.0, 200.0, 300.0)).length() < 1e-3);
        assert_eq!(mm.normal, Some(Vec3::Z));
        assert!((mm.radius_mm.unwrap() - 3.0).abs() < 1e-5);
        assert_eq!(m.scaled(-1.0).normal, Some(-Vec3::Z), "a mirror flips the normal as it flips the surface's");
    }

    #[test]
    fn an_old_alignment_record_reads_as_partly_measured() {
        let r = AlignmentRecord { status: "aligned".into(), method: Some("auto".into()), overlap: Some(0.5), rmse_mm: Some(0.2), visibility_conflict: Some(0.01), ..Default::default() };
        let AlignState::Aligned { quality: Some(q), .. } = alignment_state(&r) else { panic!() };
        assert_eq!((q.overlap, q.rmse_mm, q.inliers, q.slide_mm), (0.5, 0.2, 1000, 0.0), "missing fields read as not measured");
        assert!(q.shell.is_none());
        // And the new fields are not written for a pass without quality.
        let json = serde_json::to_string(&alignment_record(AlignState::Aligned { method: AlignMethod::Auto, quality: None })).unwrap();
        assert!(!json.contains("shell") && !json.contains("inliers"), "{json}");
    }

    #[test]
    fn a_project_without_origins_loads_as_captured_passes() {
        let dir = std::env::temp_dir().join(format!("scancraft-app-old-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        let mut meta = PassMeta::new(ScanConfig::default(), None, PassStart::Moved);
        meta.alignment = AlignState::Reference;
        let project = StoredProject {
            name: "Old".into(),
            reference: Some(1),
            next_pass_id: 2,
            passes: vec![StoredPass { id: 1, name: "Pass 1".into(), frames: 1, visible: true, meta, keyframes: Vec::new(), surface: None, edit: None, rgb_keyframes: Vec::new(), mask: None, markers: Vec::new() }],
            merged: None,
            cleaned: None,
            mesh: None,
            settings: serde_json::json!({}),
        };
        save(&dir, &project).unwrap();
        // What older versions wrote: the scan settings without an origin.
        let json = std::fs::read_to_string(files::locate(&dir).unwrap().file).unwrap();
        assert!(!json.contains(&format!("\"{ORIGIN_KEY}\"")), "a captured pass stores no origin");
        let loaded = load(&dir).unwrap();
        assert_eq!(loaded.passes[0].meta.origin, PassOrigin::Captured);
        assert!(loaded.passes[0].surface.is_none());
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn rgb_keyframes_round_trip_through_a_project_folder() {
        let dir = std::env::temp_dir().join(format!("scancraft-app-rgb-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        let mut meta = PassMeta::new(ScanConfig::default(), None, PassStart::Moved);
        meta.alignment = AlignState::Reference;
        let cam = Arc::new(scancraft_core::ColorCamera {
            width: 32,
            height: 24,
            intrinsics: scancraft_core::PinholeIntrinsics { fx: 40.0, fy: 40.0, cx: 16.0, cy: 12.0 },
            distortion: [0.0; 5],
            rotation: glam::Mat3::IDENTITY,
            translation: Vec3::new(-12.0, 0.0, 0.0),
        });
        let color_frame = scancraft_core::ColorFrame {
            image: scancraft_core::RgbImage { width: 32, height: 24, channels: 3, data: vec![180u8; 32 * 24 * 3] },
            jpeg: None,
            timestamp_ms: 120,
            camera: Some(cam.clone()),
        };
        let kf = scancraft_color::RgbKeyframe::new(3, &color_frame, cam, Transform::from_translation(Vec3::new(1.0, 2.0, 3.0)), 42.5, 9).unwrap();
        let project = StoredProject {
            name: "RgbTest".into(),
            reference: Some(1),
            next_pass_id: 2,
            passes: vec![StoredPass {
                id: 1,
                name: "Pass 1".into(),
                frames: 1,
                visible: true,
                meta,
                keyframes: Vec::new(),
                surface: None,
                edit: None,
                rgb_keyframes: vec![kf],
                mask: None,
                markers: Vec::new(),
            }],
            merged: None,
            cleaned: None,
            mesh: None,
            settings: serde_json::json!({}),
        };
        save(&dir, &project).unwrap();
        assert!(dir.join(rgb_manifest_rel(1)).is_file());
        let loaded = load(&dir).unwrap();
        assert_eq!(loaded.passes[0].rgb_keyframes.len(), 1);
        let back = &loaded.passes[0].rgb_keyframes[0];
        assert_eq!((back.id, back.timestamp_ms, back.width, back.height, back.depth_frame_index), (3, 120, 32, 24, 9));
        assert_eq!(&back.jpeg[..2], &[0xff, 0xd8]);
        let _ = std::fs::remove_dir_all(&dir);
    }
}
