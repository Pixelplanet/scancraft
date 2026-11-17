//! Session data: workflow stages, the scan passes and derived geometry, the browser selection,
//! and the messages exchanged with background tasks.

use std::sync::Arc;

use eframe::egui::ColorImage;
use scancraft_core::{BoundingBox, Mesh, Point3D, PointCloud, SensorFrame};
use scancraft_driver::{DeviceInfo, DiscoveredDevice, ScanConfig, ScannerCapabilities, ScannerDevice};
use scancraft_mesh::MeshAnalysis;
use scancraft_render::{GpuMesh, GpuPoint};
use scancraft_slam::{SeedState, TrackingStatus};
use tokio::sync::mpsc;

use crate::project::{AlignMethod, PassMeta};

pub type SharedDevice = Arc<tokio::sync::Mutex<Box<dyn ScannerDevice>>>;

/// Workflow stages, shown as the ribbon's tabs.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Stage {
    Connect,
    Scan,
    Align,
    Clean,
    Mesh,
    Export,
}

impl Stage {
    pub const ALL: [Stage; 6] = [Stage::Connect, Stage::Scan, Stage::Align, Stage::Clean, Stage::Mesh, Stage::Export];

    pub fn label(self) -> &'static str {
        match self {
            Stage::Connect => "Connect",
            Stage::Scan => "Scan",
            Stage::Align => "Align",
            Stage::Clean => "Clean",
            Stage::Mesh => "Mesh",
            Stage::Export => "Export",
        }
    }

    pub fn index(self) -> usize {
        Stage::ALL.iter().position(|s| *s == self).unwrap_or(0)
    }
}

/// A command whose parameters are open in the right-hand card.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Tool {
    ConnectIp,
    RemovePlane,
    Outliers,
    Downsample,
    Crop,
    Reconstruct,
    Smooth,
    Decimate,
    Export(ExportFormat),
    /// Split view for picking corresponding points on a pass and the project model.
    PointPairs(u32),
    SaveProject,
    OpenProject,
}

impl Tool {
    /// Project-file commands: open from the app bar on any stage.
    pub fn is_file(self) -> bool {
        matches!(self, Tool::SaveProject | Tool::OpenProject)
    }

    pub fn title(self) -> &'static str {
        match self {
            Tool::PointPairs(_) => "Point Pairs",
            Tool::SaveProject => "Save Project",
            Tool::OpenProject => "Open Project",
            Tool::ConnectIp => "Connect by IP",
            Tool::RemovePlane => "Remove Plane",
            Tool::Outliers => "Remove Outliers",
            Tool::Downsample => "Downsample",
            Tool::Crop => "Crop Box",
            Tool::Reconstruct => "Reconstruct Surface",
            Tool::Smooth => "Smooth",
            Tool::Decimate => "Decimate",
            Tool::Export(_) => "Export",
        }
    }

    pub fn icon(self) -> &'static str {
        match self {
            Tool::ConnectIp => "web",
            Tool::RemovePlane => "plane",
            Tool::Outliers => "filter",
            Tool::Downsample => "snap",
            Tool::Crop => "box",
            Tool::Reconstruct => "mesh_body",
            Tool::Smooth => "surface",
            Tool::Decimate => "reach",
            Tool::Export(_) => "export",
            Tool::PointPairs(_) => "point",
            Tool::SaveProject => "save",
            Tool::OpenProject => "open",
        }
    }

    pub fn stage(self) -> Stage {
        match self {
            Tool::ConnectIp | Tool::SaveProject | Tool::OpenProject => Stage::Connect,
            Tool::PointPairs(_) => Stage::Align,
            Tool::RemovePlane | Tool::Outliers | Tool::Downsample | Tool::Crop => Stage::Clean,
            Tool::Reconstruct | Tool::Smooth | Tool::Decimate => Stage::Mesh,
            Tool::Export(_) => Stage::Export,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ExportFormat {
    PlyCloud,
    PlyMesh,
    ObjMesh,
    StlMesh,
}

impl ExportFormat {
    pub const ALL: [ExportFormat; 4] = [ExportFormat::PlyCloud, ExportFormat::PlyMesh, ExportFormat::ObjMesh, ExportFormat::StlMesh];

    pub fn label(self) -> &'static str {
        match self {
            ExportFormat::PlyCloud => "Point cloud (PLY)",
            ExportFormat::PlyMesh => "Mesh (PLY)",
            ExportFormat::ObjMesh => "Mesh (OBJ)",
            ExportFormat::StlMesh => "Mesh (STL)",
        }
    }

    pub fn extension(self) -> &'static str {
        match self {
            ExportFormat::PlyCloud | ExportFormat::PlyMesh => "ply",
            ExportFormat::ObjMesh => "obj",
            ExportFormat::StlMesh => "stl",
        }
    }

    pub fn needs_mesh(self) -> bool {
        !matches!(self, ExportFormat::PlyCloud)
    }
}

/// What the browser has selected.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Node {
    Device,
    Pass(u32),
    /// A stored keyframe of a pass: (pass id, keyframe id).
    Frame(u32, u64),
    Merged,
    Cleaned,
    Mesh,
}

/// Point geometry and its GPU copy. `generation` changes whenever the points are replaced (not
/// when they are only appended to), so the renderer knows to upload them again.
pub struct CloudLayer {
    pub cloud: PointCloud,
    pub gpu: Arc<Vec<GpuPoint>>,
    pub generation: u64,
    pub visible: bool,
}

impl CloudLayer {
    pub fn new(cloud: PointCloud, generation: u64) -> Self {
        let gpu = Arc::new(scancraft_render::gpu_points(&cloud));
        CloudLayer { cloud, gpu, generation, visible: true }
    }

    pub fn append(&mut self, points: Vec<Point3D>) {
        let gpu = Arc::make_mut(&mut self.gpu);
        gpu.reserve(points.len());
        for p in points {
            gpu.push(GpuPoint::from_point(&p));
            self.cloud.push(p);
        }
    }

    pub fn len(&self) -> usize {
        self.cloud.len()
    }
}

pub type SharedTracker = Arc<std::sync::Mutex<scancraft_slam::Tracker>>;

/// What the browser shows of a stored keyframe (a copy, so the interface never waits on the
/// tracker while the worker holds it).
#[derive(Clone, Debug, PartialEq)]
pub struct KeyframeInfo {
    pub id: u64,
    pub rmse_mm: f32,
    pub visibility_conflict: f32,
    pub new_points: usize,
    pub points: usize,
    pub discarded: bool,
}

impl KeyframeInfo {
    pub fn of(k: &scancraft_slam::Keyframe) -> Self {
        KeyframeInfo { id: k.id, rmse_mm: k.rmse_mm, visibility_conflict: k.visibility_conflict, new_points: k.new_points, points: k.point_count(), discarded: k.discarded }
    }

    /// How well the frame fitted, from its alignment error and how much known surface it saw
    /// "through". The thresholds are the tracker's acceptance limits.
    pub fn quality(&self, max_rmse_mm: f32, max_conflict: f32) -> Quality {
        let r = self.rmse_mm / max_rmse_mm.max(1e-3);
        let c = self.visibility_conflict / max_conflict.max(1e-3);
        if r <= 0.6 && c <= 0.5 {
            Quality::Good
        } else if r <= 0.9 && c <= 0.8 {
            Quality::Fair
        } else {
            Quality::Poor
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Quality {
    Good,
    Fair,
    Poor,
}

impl Quality {
    pub fn label(self) -> &'static str {
        match self {
            Quality::Good => "Good fit",
            Quality::Fair => "Fair fit",
            Quality::Poor => "Poor fit",
        }
    }
}

/// One capture run. It keeps its tracker, so its frames can be discarded after the capture and
/// the pass can be continued later.
pub struct ScanPass {
    pub id: u32,
    pub name: String,
    pub layer: CloudLayer,
    pub frames: u64,
    pub tracker: SharedTracker,
    pub keyframes: Vec<KeyframeInfo>,
    /// Mode, device, alignment into the project frame, merge weight. `layer` holds the pass's
    /// surface already placed by `meta.transform`.
    pub meta: PassMeta,
    /// The layer in the pass's colour (Align stage): (layer generation, length, points).
    pub tinted: Option<(u64, usize, Arc<Vec<GpuPoint>>)>,
}

impl ScanPass {
    pub fn kept_frames(&self) -> usize {
        self.keyframes.iter().filter(|k| !k.discarded).count()
    }
}

/// Distinct colours for passes in the Align stage (the reference is the first).
pub const PASS_COLOURS: [[u8; 3]; 6] = [[150, 162, 178], [232, 140, 60], [70, 170, 220], [120, 200, 110], [210, 100, 190], [230, 205, 80]];

/// GPU points in one colour with a fixed light baked in (the renderer's colour mode is unlit).
pub fn tinted_points(cloud: &PointCloud, colour: [u8; 3]) -> Vec<GpuPoint> {
    let light = glam::Vec3::new(-0.35, -0.75, 0.55).normalize();
    cloud
        .points
        .iter()
        .map(|p| {
            let n = glam::Vec3::from(p.normal);
            let k = 0.35 + 0.65 * n.dot(light).abs();
            let c = |v: u8| (v as f32 * k).min(255.0) as u8;
            let mut g = GpuPoint::from_point(p);
            g.color = [c(colour[0]), c(colour[1]), c(colour[2]), 255];
            g
        })
        .collect()
}

/// The fused result of Merge.
pub struct MergedLayer {
    pub layer: CloudLayer,
    /// The layer shaded for the Align stage's colour mode.
    pub tinted: Arc<Vec<GpuPoint>>,
    /// Passes it was made from.
    pub passes: Vec<u32>,
    pub summary: String,
}

impl MergedLayer {
    pub fn new(cloud: PointCloud, generation: u64, passes: Vec<u32>, summary: String) -> Self {
        let tinted = Arc::new(tinted_points(&cloud, [205, 210, 218]));
        MergedLayer { layer: CloudLayer::new(cloud, generation), tinted, passes, summary }
    }
}

/// A reconstructed surface.
pub struct MeshLayer {
    pub mesh: Mesh,
    pub analysis: MeshAnalysis,
    pub gpu: Arc<GpuMesh>,
    pub generation: u64,
    pub visible: bool,
}

impl MeshLayer {
    pub fn new(mesh: Mesh, analysis: MeshAnalysis, generation: u64) -> Self {
        let gpu = Arc::new(GpuMesh::from_mesh(&mesh));
        MeshLayer { mesh, analysis, gpu, generation, visible: true }
    }
}

/// The connected scanner.
pub struct Connection {
    pub device: SharedDevice,
    pub info: DeviceInfo,
    pub capabilities: ScannerCapabilities,
    /// The driver it was connected with ("revopoint", "mock").
    #[allow(dead_code)]
    pub driver_id: String,
}

impl Connection {

    /// How it is connected, in one word.
    pub fn link(&self) -> String {
        match &self.info.active_transport {
            scancraft_driver::DeviceTransport::WiFi { ip, .. } => format!("Wi-Fi {ip}"),
            scancraft_driver::DeviceTransport::Usb { .. } => "USB".to_string(),
            scancraft_driver::DeviceTransport::Virtual { .. } => "Simulator".to_string(),
        }
    }

    /// A short name for the workspace card ("POP 4", "Virtual Pro 3D").
    pub fn short_name(&self) -> String {
        self.info.model.trim_start_matches("Revopoint ").to_string()
    }
}

/// Messages from device tasks.
pub enum DeviceEvent {
    DiscoveryFinished(Vec<DiscoveredDevice>),
    Connected { device: SharedDevice, info: DeviceInfo, capabilities: ScannerCapabilities, driver_id: String },
    ConnectFailed(String),
    PreviewStarted(mpsc::Receiver<scancraft_core::PreviewFrame>),
    PreviewFailed(String),
    ScanStarted(mpsc::Receiver<SensorFrame>, ScanConfig),
    ScanFailed(String),
    ScanStopped,
    Paused(bool),
    ParameterFailed(String),
    Disconnected,
}

/// One processed frame from the scan worker.
pub struct ScanUpdate {
    pub ir_image: Option<ColorImage>,
    pub depth_image: Option<ColorImage>,
    pub distance_mm: Option<f32>,
    pub tracking: Option<TrackingStatus>,
    pub new_points: Vec<Point3D>,
    /// Keyframes stored since the last update the interface received.
    pub keyframes: Vec<KeyframeInfo>,
    /// For a pass started inside the project model: whether it has located itself yet.
    pub seed: Option<SeedState>,
    /// Markers (or other features) of this frame, for the camera view and the 3D view.
    pub overlay: Option<scancraft_pipeline::ImageOverlay>,
    /// How many frames arrive and how many carry new information, per second.
    pub rates: Option<FrameRates>,
}

/// Frame rates measured by the scan worker over the last two seconds.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct FrameRates {
    /// Frames arriving from the scanner.
    pub incoming: f32,
    /// Frames the worker processed (it skips to the newest when it falls behind).
    pub processed: f32,
    /// Frames with a new marker observation; `None` when the pass does not track markers. Lower
    /// than `incoming` when the scanner repeats the IR images (the POP 4 does on every other
    /// packet in marker modes).
    pub marker: Option<f32>,
}

/// Results of background geometry jobs.
pub enum JobResult {
    Cloud { cloud: PointCloud, message: String },
    NoChange(String),
    Mesh { mesh: Mesh, analysis: MeshAnalysis, message: String },
    Exported(String),
    Failed(String),
    /// A pass's model rebuilt from its kept keyframes (in the pass's own frame).
    Rebuilt { pass: u32, points: Vec<Point3D>, keyframes: Vec<KeyframeInfo>, message: String },
    /// A candidate alignment for a pass, to preview before accepting.
    Aligned { pass: u32, method: AlignMethod, alignment: scancraft_align::Alignment, message: String },
    /// Refined project <- pass transforms.
    Refined { poses: Vec<(u32, scancraft_core::Transform)>, message: String },
    Merged { cloud: PointCloud, passes: Vec<u32>, message: String },
    Saved { path: std::path::PathBuf, name: String, message: String },
    Opened(Box<OpenedProject>),
}

/// A project read from disk with its trackers rebuilt.
pub struct OpenedProject {
    pub path: std::path::PathBuf,
    pub name: String,
    pub next_pass_id: u32,
    pub passes: Vec<crate::project::LoadedPass>,
    pub merged: Option<(PointCloud, Vec<u32>)>,
    pub cleaned: Option<PointCloud>,
    pub mesh: Option<(Mesh, MeshAnalysis)>,
    pub settings: serde_json::Value,
}

/// Restorable geometry for undo.
#[derive(Clone, Default)]
pub struct Snapshot {
    pub merged: Option<(PointCloud, Vec<u32>, String)>,
    pub cleaned: Option<PointCloud>,
    pub mesh: Option<(Mesh, MeshAnalysis)>,
    pub passes: Vec<PassSnapshot>,
    pub label: String,
}

/// A pass as undo restores it: its points and which of its keyframes were discarded. The
/// tracker is shared, not copied; restoring re-applies the discard flags to it.
#[derive(Clone)]
pub struct PassSnapshot {
    pub id: u32,
    pub name: String,
    pub cloud: PointCloud,
    pub frames: u64,
    pub tracker: SharedTracker,
    pub keyframes: Vec<KeyframeInfo>,
    pub meta: PassMeta,
}

/// Bounded two-stack undo history (kept from the previous app; snapshots clone geometry, so the
/// depth stays small).
#[derive(Default)]
pub struct History {
    undo: Vec<Snapshot>,
    redo: Vec<Snapshot>,
}

pub const MAX_HISTORY: usize = 8;

impl History {
    pub fn push(&mut self, snapshot: Snapshot) {
        self.redo.clear();
        self.undo.push(snapshot);
        if self.undo.len() > MAX_HISTORY {
            self.undo.remove(0);
        }
    }

    pub fn undo(&mut self, current: Snapshot) -> Option<Snapshot> {
        let previous = self.undo.pop()?;
        self.redo.push(current);
        Some(previous)
    }

    pub fn redo(&mut self, current: Snapshot) -> Option<Snapshot> {
        let next = self.redo.pop()?;
        self.undo.push(current);
        Some(next)
    }

    /// Drop the newest undo entry (an operation that turned out to change nothing).
    pub fn discard_last(&mut self) {
        self.undo.pop();
    }

    pub fn can_undo(&self) -> bool {
        !self.undo.is_empty()
    }

    pub fn can_redo(&self) -> bool {
        !self.redo.is_empty()
    }

    pub fn undo_label(&self) -> Option<&str> {
        self.undo.last().map(|s| s.label.as_str())
    }

    pub fn redo_label(&self) -> Option<&str> {
        self.redo.last().map(|s| s.label.as_str())
    }
}

/// Merge the bounding boxes of the given clouds.
pub fn union_bbox<'a>(boxes: impl IntoIterator<Item = &'a BoundingBox>) -> Option<BoundingBox> {
    let mut out: Option<BoundingBox> = None;
    for b in boxes {
        if b.is_empty() {
            continue;
        }
        out = Some(match out {
            None => *b,
            Some(mut o) => {
                o.extend(b.min);
                o.extend(b.max);
                o
            }
        });
    }
    out
}

/// Group digits in thousands: 12345 -> "12,345".
pub fn thousands(n: usize) -> String {
    let s = n.to_string();
    let mut out = String::with_capacity(s.len() + s.len() / 3);
    for (i, c) in s.chars().enumerate() {
        if i > 0 && (s.len() - i) % 3 == 0 {
            out.push(',');
        }
        out.push(c);
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn thousands_groups_digits() {
        assert_eq!(thousands(0), "0");
        assert_eq!(thousands(999), "999");
        assert_eq!(thousands(1000), "1,000");
        assert_eq!(thousands(1234567), "1,234,567");
    }

    #[test]
    fn history_round_trips() {
        let mut h = History::default();
        assert!(!h.can_undo());
        h.push(Snapshot { label: "a".into(), ..Default::default() });
        let prev = h.undo(Snapshot { label: "now".into(), ..Default::default() }).unwrap();
        assert_eq!(prev.label, "a");
        assert!(h.can_redo());
        let next = h.redo(Snapshot { label: "a".into(), ..Default::default() }).unwrap();
        assert_eq!(next.label, "now");
    }

    #[test]
    fn appended_points_reach_the_gpu_copy() {
        let mut layer = CloudLayer::new(PointCloud::new(), 1);
        layer.append(vec![Point3D::new([1.0, 2.0, 3.0])]);
        assert_eq!(layer.len(), 1);
        assert_eq!(layer.gpu.len(), 1);
    }
}
