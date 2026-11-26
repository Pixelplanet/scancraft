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

use crate::alignment::{AlignAllOutcome, RefineOutcome};
use crate::isolation::MaskCache;
use crate::project::{AlignMethod, PassMarker, PassMeta, PassSource};

pub type SharedDevice = Arc<tokio::sync::Mutex<Box<dyn ScannerDevice>>>;

/// Workflow stages, shown as the ribbon's tabs.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Stage {
    Connect,
    Scan,
    Clean,
    Align,
    Mesh,
    Export,
}

impl Stage {
    pub const ALL: [Stage; 6] = [Stage::Connect, Stage::Scan, Stage::Clean, Stage::Align, Stage::Mesh, Stage::Export];

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
    /// Keep the object: click a cluster to keep it, shift-click one to leave it out.
    IsolateObject,
    /// Segment the object with SAM 2.1 video segmentation on colour keyframes.
    AiSegment,
    Reconstruct,
    Smooth,
    Decimate,
    /// Close holes in the mesh up to a size.
    FillHoles,
    /// A closed, watertight solid of a thin-walled part, built from the points.
    MakeSolid,
    Export(ExportFormat),
    /// Split view for picking corresponding points on a pass and the project model.
    PointPairs(u32),
    /// Click a plane, an axis or a point on the model to make them world datums.
    Datums,
    /// Give an imported pass its true size from lengths picked and typed on it.
    Scale(u32),
    SaveProject,
}

impl Tool {
    /// Project-file commands: open from the app bar on any stage.
    pub fn is_file(self) -> bool {
        matches!(self, Tool::SaveProject)
    }

    pub fn title(self) -> &'static str {
        match self {
            Tool::PointPairs(_) => "Point Pairs",
            Tool::SaveProject => "Save Project",
            Tool::ConnectIp => "Connect by IP",
            Tool::RemovePlane => "Remove Plane",
            Tool::Outliers => "Remove Outliers",
            Tool::Downsample => "Downsample",
            Tool::Crop => "Crop Box",
            Tool::IsolateObject => "Isolate Object",
            Tool::AiSegment => "AI Segment",
            Tool::Reconstruct => "Reconstruct Surface",
            Tool::Smooth => "Smooth",
            Tool::Decimate => "Decimate",
            Tool::FillHoles => "Fill Holes",
            Tool::MakeSolid => "Make Solid",
            Tool::Export(_) => "Export",
            Tool::Datums => "Orient by Datums",
            Tool::Scale(_) => "Scale from Lengths",
        }
    }

    pub fn icon(self) -> &'static str {
        match self {
            Tool::ConnectIp => "web",
            Tool::RemovePlane => "plane",
            Tool::Outliers => "filter",
            Tool::Downsample => "snap",
            Tool::Crop => "box",
            Tool::IsolateObject => "body",
            Tool::AiSegment => "filter",
            Tool::Reconstruct => "mesh_body",
            Tool::Smooth => "surface",
            Tool::Decimate => "reach",
            Tool::FillHoles => "patch",
            Tool::MakeSolid => "body",
            Tool::Export(_) => "export",
            Tool::PointPairs(_) => "point",
            Tool::Datums => "measure",
            Tool::Scale(_) => "scale",
            Tool::SaveProject => "save",
        }
    }

    pub fn stage(self) -> Stage {
        match self {
            Tool::ConnectIp | Tool::SaveProject => Stage::Connect,
            Tool::PointPairs(_) | Tool::Datums | Tool::Scale(_) => Stage::Align,
            Tool::RemovePlane | Tool::Outliers | Tool::Downsample | Tool::Crop | Tool::IsolateObject | Tool::AiSegment => Stage::Clean,
            Tool::Reconstruct | Tool::Smooth | Tool::Decimate | Tool::FillHoles | Tool::MakeSolid => Stage::Mesh,
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
    Mesh,
}

/// Something the Clean tools work on: a pass, or the merged model.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CleanTarget {
    Pass(u32),
    Merged,
}

/// Point geometry and its GPU copy. `generation` changes whenever the points are replaced (not
/// when they are only appended to), so the renderer knows to upload them again.
///
/// The points and the GPU copy are shared (`Arc`): cloning a layer is cheap, which is what lets
/// undo snapshots keep every layer that did not change without copying it.
#[derive(Clone)]
pub struct CloudLayer {
    pub cloud: Arc<PointCloud>,
    pub gpu: Arc<Vec<GpuPoint>>,
    pub generation: u64,
    pub visible: bool,
}

impl CloudLayer {
    pub fn new(cloud: PointCloud, generation: u64) -> Self {
        let gpu = Arc::new(scancraft_render::gpu_points(&cloud));
        CloudLayer { cloud: Arc::new(cloud), gpu, generation, visible: true }
    }

    pub fn append(&mut self, points: Vec<Point3D>) {
        // Copies only if an undo snapshot still shares the points.
        let gpu = Arc::make_mut(&mut self.gpu);
        let cloud = Arc::make_mut(&mut self.cloud);
        gpu.reserve(points.len());
        for p in points {
            gpu.push(GpuPoint::from_point(&p));
            cloud.push(p);
        }
    }

    /// Memory held by the points and their GPU copy (for the history's memory limit).
    pub fn bytes(&self) -> usize {
        self.cloud.len() * std::mem::size_of::<Point3D>() + self.gpu.len() * std::mem::size_of::<GpuPoint>()
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
    /// Which of its surfels are the object (computed when first needed).
    pub mask: MaskCache,
    /// An imported scan's fixed surface (pass frame). Such a pass has an empty tracker, no
    /// keyframes and cannot be continued.
    pub surface: Option<Arc<PointCloud>>,
    /// The pass after cleaning (pass frame; `confidence` holds each point's observation count).
    /// When set it replaces the tracked or imported surface everywhere: display, alignment,
    /// merge and saving. A cleaned captured pass cannot be continued or have frames discarded
    /// until its cleaning is reset.
    pub edit: Option<Arc<PointCloud>>,
    /// Calibrated RGB keyframes captured during this pass (for colour projection and AI segmentation).
    pub rgb_keyframes: Vec<scancraft_color::RgbKeyframe>,
    /// Its marker map (pass frame): the tracker's while capturing, or read with an imported scan.
    pub markers: Vec<PassMarker>,
}

impl ScanPass {
    pub fn is_cleaned(&self) -> bool {
        self.edit.is_some()
    }

    pub fn kept_frames(&self) -> usize {
        self.keyframes.iter().filter(|k| !k.discarded).count()
    }

    /// A fixed surface (an imported scan) rather than a tracked capture.
    pub fn is_fixed(&self) -> bool {
        self.surface.is_some()
    }

    /// Where background jobs read its surface from.
    pub fn source(&self) -> PassSource {
        match (&self.edit, &self.surface) {
            (Some(e), _) => PassSource::Edited { surface: Arc::clone(e), tracker: (!self.is_fixed()).then(|| self.tracker.clone()) },
            (None, Some(s)) => PassSource::Fixed(Arc::clone(s)),
            (None, None) => PassSource::Tracked(self.tracker.clone()),
        }
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
#[derive(Clone)]
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
/// The mesh's holes while Fill Holes is open: found once per mesh, outlined in the view.
pub struct HolePreview {
    /// Generation of the mesh they were found in.
    pub mesh_generation: u64,
    /// Largest first.
    pub holes: Vec<scancraft_mesh::HoleInfo>,
    /// The outline points for the settings they were coloured for: (max size, keep largest).
    pub outline: Option<((f32, bool), CloudLayer)>,
}

impl HolePreview {
    /// Whether hole `i` (in `holes` order) would be filled.
    pub fn fills(&self, i: usize, max_mm: f32, keep_largest: bool) -> bool {
        !(keep_largest && i == 0) && self.holes[i].perimeter_mm <= max_mm
    }

    /// (to fill, left open).
    pub fn split(&self, max_mm: f32, keep_largest: bool) -> (usize, usize) {
        let fill = (0..self.holes.len()).filter(|&i| self.fills(i, max_mm, keep_largest)).count();
        (fill, self.holes.len() - fill)
    }
}

/// A reconstructed surface. The mesh and its GPU copy are shared, as in [`CloudLayer`].
#[derive(Clone)]
pub struct MeshLayer {
    pub mesh: Arc<Mesh>,
    pub analysis: MeshAnalysis,
    pub gpu: Arc<GpuMesh>,
    pub generation: u64,
    pub visible: bool,
}

impl MeshLayer {
    pub fn new(mesh: Mesh, analysis: MeshAnalysis, generation: u64) -> Self {
        let gpu = Arc::new(GpuMesh::from_mesh(&mesh));
        MeshLayer { mesh: Arc::new(mesh), analysis, gpu, generation, visible: true }
    }

    /// Memory held by the mesh (GPU copy estimated from the same counts).
    pub fn bytes(&self) -> usize {
        2 * (self.mesh.vertices.len() * std::mem::size_of::<Point3D>() + self.mesh.triangles.len() * 12)
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
    pub color_image: Option<ColorImage>,
    pub distance_mm: Option<f32>,
    pub tracking: Option<TrackingStatus>,
    pub new_points: Vec<Point3D>,
    /// Keyframes stored since the last update the interface received.
    pub keyframes: Vec<KeyframeInfo>,
    /// Calibrated RGB keyframes added since the last update.
    pub rgb_keyframes: Vec<scancraft_color::RgbKeyframe>,
    /// Live AI segmentation object mask overlay for `color_image`, if active.
    pub ai_mask_overlay: Option<ColorImage>,
    /// Live AI segmentation IoU score (0..1) and active provider label.
    pub ai_status: Option<(f32, String)>,
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

/// How sure an automatic alignment is: the same fit seen from the other pass, and whether
/// aligning the other way round finds it too (`scancraft_align::align_robust`).
#[derive(Debug, Clone, Copy)]
pub struct AlignCheck {
    /// The target moved onto the pass by the inverse.
    pub reverse: scancraft_align::Quality,
    /// Best result against the closest one found the other way round.
    pub agreement: Option<scancraft_align::Agreement>,
    pub consistent: bool,
    /// Runs (feature grids x directions) that found a result, and how many agree with the best.
    pub runs: usize,
    pub agreeing: usize,
    /// The table and loose fragments were left out.
    pub object_only: bool,
}

/// Results of background geometry jobs.
pub enum JobResult {
    /// Clean results: each cleaned pass's new surface (pass frame) and the merged model's new
    /// cloud, for whichever was cleaned.
    Cleaned { passes: Vec<(u32, PointCloud)>, merged: Option<PointCloud>, message: String },
    NoChange(String),
    Mesh { mesh: Mesh, analysis: MeshAnalysis, message: String },
    Exported(String),
    Failed(String),
    /// A pass's model rebuilt from its kept keyframes (in the pass's own frame).
    Rebuilt { pass: u32, points: Vec<Point3D>, keyframes: Vec<KeyframeInfo>, message: String },
    /// A candidate alignment for a pass, to preview before accepting.
    Aligned { pass: u32, method: AlignMethod, alignment: scancraft_align::Alignment, check: Option<AlignCheck>, message: String },
    /// A candidate orientation for the whole project, and what it was found on.
    Oriented(Box<scancraft_align::AutoOrient>, &'static str),
    /// Placements for all passes (Align All), to preview before accepting.
    AlignedAll(Box<AlignAllOutcome>),
    /// Refined project <- pass transforms with before / after measures, to preview.
    Refined(Box<RefineOutcome>),
    /// The Isolate Object tool's segmentation, and the picks it was made for.
    Isolated { segmentation: Arc<scancraft_fusion::Segmentation>, picks: crate::isolation::Picks },
    /// Initial single-keyframe SAM 2.1 mask (preview + score) before propagating across all keyframes.
    AiPrompted {
        pass: u32,
        keyframe_index: usize,
        mask: scancraft_segment::FrameMask,
        overlay: egui::ColorImage,
    },
    /// SAM 2.1 video propagation + 3D depth-buffered multi-view labels for the active pass.
    AiPropagated {
        pass: u32,
        masks: Vec<scancraft_segment::FrameMask>,
        labels: Vec<scancraft_segment::PointLabel>,
        message: String,
    },
    /// AI Pack install / download / verification finished.
    AiPackUpdated {
        status: crate::ai_pack::AiPackStatus,
        message: String,
        ok: bool,
    },
    Merged { cloud: PointCloud, passes: Vec<u32>, message: String },
    Saved { path: std::path::PathBuf, name: String, message: String },
    Opened(Box<OpenedProject>),
    /// A scan file read as a new pass.
    Imported(Box<crate::imports::ImportedScan>),
    /// The passes of another project (ids still those of that project).
    AddedProject { name: String, passes: Vec<crate::project::LoadedPass> },
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

/// Restorable geometry for undo: the project as it was before an action. Layers are shared with
/// the live project and with the neighbouring snapshots (see [`CloudLayer`]), so a step only
/// costs what that action changed.
#[derive(Clone)]
pub struct Snapshot {
    pub merged: Option<MergedLayer>,
    pub mesh: Option<MeshLayer>,
    pub passes: Vec<PassSnapshot>,
    /// The action that leads from this state to the next one, and when it was done.
    pub label: String,
    pub at: std::time::SystemTime,
    /// Tests only: which state this is.
    #[cfg(test)]
    pub summary_for_test: String,
}

impl Default for Snapshot {
    fn default() -> Self {
        Snapshot {
            merged: None,
            mesh: None,
            passes: Vec::new(),
            label: String::new(),
            at: std::time::SystemTime::UNIX_EPOCH,
            #[cfg(test)]
            summary_for_test: String::new(),
        }
    }
}

impl Snapshot {
    /// Every shared block of geometry it holds: (address, bytes). Equal addresses are the same
    /// memory, held once however many snapshots share it.
    pub fn blocks(&self) -> Vec<(usize, usize)> {
        let mut out = Vec::new();
        for p in &self.passes {
            out.push((Arc::as_ptr(&p.layer.cloud) as usize, p.layer.bytes()));
            if let Some(e) = &p.edit {
                out.push((Arc::as_ptr(e) as usize, e.len() * std::mem::size_of::<Point3D>()));
            }
        }
        if let Some(m) = &self.merged {
            out.push((Arc::as_ptr(&m.layer.cloud) as usize, m.layer.bytes()));
        }
        if let Some(m) = &self.mesh {
            out.push((Arc::as_ptr(&m.mesh) as usize, m.bytes()));
        }
        out
    }
}

/// A pass as undo restores it: its points and which of its keyframes were discarded. The
/// tracker is shared, not copied; restoring re-applies the discard flags to it.
#[derive(Clone)]
pub struct PassSnapshot {
    pub id: u32,
    pub name: String,
    pub layer: CloudLayer,
    pub frames: u64,
    pub tracker: SharedTracker,
    pub keyframes: Vec<KeyframeInfo>,
    pub meta: PassMeta,
    pub mask: MaskCache,
    pub surface: Option<Arc<PointCloud>>,
    pub edit: Option<Arc<PointCloud>>,
    pub rgb_keyframes: Vec<scancraft_color::RgbKeyframe>,
    pub markers: Vec<PassMarker>,
}

/// Undo history: the states before each action (oldest first) and, after an undo, the states
/// that can be redone (next first). Bounded by memory, not by a small step count: snapshots
/// share unchanged geometry, so only what each action replaced counts (see [`Snapshot`]).
#[derive(Default)]
pub struct History {
    undo: Vec<Snapshot>,
    redo: Vec<Snapshot>,
}

/// Most steps kept, however small.
pub const MAX_HISTORY: usize = 200;
/// Most memory the history may hold on its own (geometry no longer in the live project);
/// the oldest steps go first.
pub const HISTORY_BUDGET_BYTES: usize = 2 << 30;

/// One entry of the timeline, oldest first.
pub struct TimelineStep {
    pub label: String,
    pub at: std::time::SystemTime,
    /// Done (at or before the current state) rather than undone (redo).
    pub done: bool,
}

impl History {
    /// Records the state before a new action. The redo branch is dropped. The caller trims
    /// against the live project afterwards ([`trim`](Self::trim)).
    pub fn push(&mut self, snapshot: Snapshot) {
        self.redo.clear();
        self.undo.push(snapshot);
        while self.undo.len() > MAX_HISTORY {
            self.undo.remove(0);
        }
    }

    /// Drops the oldest steps while there are too many or they hold too much memory that the
    /// live project (`live`: its blocks, as `Snapshot::blocks`) does not share.
    pub fn trim(&mut self, live: &[(usize, usize)]) {
        while self.undo.len() > MAX_HISTORY {
            self.undo.remove(0);
        }
        while self.undo.len() > 1 && self.held_bytes(live) > HISTORY_BUDGET_BYTES {
            self.undo.remove(0);
        }
    }

    /// Memory held by the history alone: every distinct block once, minus the live project's.
    pub fn held_bytes(&self, live: &[(usize, usize)]) -> usize {
        let live: std::collections::HashSet<usize> = live.iter().map(|b| b.0).collect();
        let mut seen = std::collections::HashSet::new();
        self.undo
            .iter()
            .chain(&self.redo)
            .flat_map(|s| s.blocks())
            .filter(|(addr, _)| !live.contains(addr) && seen.insert(*addr))
            .map(|(_, bytes)| bytes)
            .sum()
    }

    /// Moves `steps` actions back (negative) or forward (positive) in one go and returns the
    /// state to restore; `current` is the live state. Every state passed on the way is kept, each
    /// labelled with the action that leads out of it, so the timeline stays complete in both
    /// directions. `None` if there are not that many steps.
    pub fn jump(&mut self, mut current: Snapshot, steps: isize) -> Option<Snapshot> {
        if steps == 0 || (steps < 0 && self.undo.len() < steps.unsigned_abs()) || (steps > 0 && self.redo.len() < steps as usize) {
            return None;
        }
        for _ in 0..steps.unsigned_abs() {
            if steps < 0 {
                let previous = self.undo.pop()?;
                // The action between `previous` and `current` is the one being undone.
                current.label = previous.label.clone();
                current.at = previous.at;
                self.redo.push(current);
                current = previous;
            } else {
                let next = self.redo.pop()?;
                current.label = next.label.clone();
                current.at = next.at;
                self.undo.push(current);
                current = next;
            }
        }
        Some(current)
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

    /// The actions in order, oldest first: the done ones, then the undone ones that can be
    /// redone. The current state sits after the last done step (index `done_count()`).
    pub fn timeline(&self) -> Vec<TimelineStep> {
        let done = self.undo.iter().map(|s| TimelineStep { label: s.label.clone(), at: s.at, done: true });
        let undone = self.redo.iter().rev().map(|s| TimelineStep { label: s.label.clone(), at: s.at, done: false });
        done.chain(undone).collect()
    }

    pub fn done_count(&self) -> usize {
        self.undo.len()
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
        let prev = h.jump(Snapshot { label: "now".into(), ..Default::default() }, -1).unwrap();
        assert_eq!(prev.label, "a");
        assert!(h.can_redo());
        let next = h.jump(Snapshot { label: "a".into(), ..Default::default() }, 1).unwrap();
        // The redone state is the one that was live; its label names the action leading into it.
        assert_eq!(next.label, "a");
        assert!(h.can_undo() && !h.can_redo());
    }

    fn state(name: &str, label: &str) -> Snapshot {
        Snapshot { label: label.into(), summary_for_test: name.into(), ..Default::default() }
    }

    #[test]
    fn a_jump_over_several_steps_keeps_the_timeline_in_both_directions() {
        // S0 -a-> S1 -b-> S2 -c-> S3 (live).
        let mut h = History::default();
        h.push(state("S0", "a"));
        h.push(state("S1", "b"));
        h.push(state("S2", "c"));
        let labels = |h: &History| h.timeline().iter().map(|s| (s.label.clone(), s.done)).collect::<Vec<_>>();
        assert_eq!(h.done_count(), 3);
        let back = h.jump(state("S3", ""), -2).unwrap();
        assert_eq!(back.summary_for_test, "S1");
        assert_eq!(h.done_count(), 1);
        assert_eq!(labels(&h), vec![("a".into(), true), ("b".into(), false), ("c".into(), false)]);
        let s2 = h.jump(back, 1).unwrap();
        assert_eq!(s2.summary_for_test, "S2");
        let s3 = h.jump(s2, 1).unwrap();
        assert_eq!(s3.summary_for_test, "S3");
        assert_eq!(labels(&h), vec![("a".into(), true), ("b".into(), true), ("c".into(), true)]);
        // Too far is refused and changes nothing.
        assert!(h.jump(s3, -4).is_none());
        assert_eq!(h.done_count(), 3);
        // A new action drops the redo branch.
        let back = h.jump(state("S3", ""), -1).unwrap();
        h.push(Snapshot { label: "d".into(), ..back });
        assert_eq!(labels(&h).last().unwrap(), &("d".to_string(), true));
        assert!(!h.can_redo());
    }

    #[test]
    fn the_history_keeps_shared_geometry_once_and_trims_by_memory() {
        let big = |n: usize| CloudLayer::new(PointCloud::from_points(vec![Point3D::new([0.0, 0.0, 0.0]); n]), 1);
        let layer = big(1000);
        let pass = |layer: &CloudLayer| PassSnapshot {
            id: 1,
            name: "P".into(),
            layer: layer.clone(),
            frames: 0,
            tracker: Arc::new(std::sync::Mutex::new(scancraft_slam::Tracker::new())),
            keyframes: Vec::new(),
            meta: PassMeta::new(scancraft_driver::ScanConfig::default(), None, crate::project::PassStart::Moved),
            mask: crate::isolation::new_cache(),
            surface: None,
            edit: None,
            rgb_keyframes: Vec::new(),
            markers: Vec::new(),
        };
        let mut h = History::default();
        for i in 0..5 {
            h.push(Snapshot { passes: vec![pass(&layer)], label: format!("step {i}"), ..Default::default() });
        }
        // Five steps, one shared cloud: held once, and not at all while the live project has it.
        let live = vec![(Arc::as_ptr(&layer.cloud) as usize, layer.bytes())];
        assert_eq!(h.held_bytes(&[]), layer.bytes());
        assert_eq!(h.held_bytes(&live), 0);
        // Steps that each hold their own huge cloud are dropped oldest first past the budget.
        let huge = HISTORY_BUDGET_BYTES / (std::mem::size_of::<Point3D>() + std::mem::size_of::<GpuPoint>()) / 3 + 1;
        let mut h = History::default();
        for i in 0..5 {
            h.push(Snapshot { passes: vec![pass(&big(huge))], label: format!("huge {i}"), ..Default::default() });
            h.trim(&[]);
        }
        assert!(h.held_bytes(&[]) <= HISTORY_BUDGET_BYTES, "{} bytes held", h.held_bytes(&[]));
        assert_eq!(h.timeline().last().unwrap().label, "huge 4", "the newest step is kept");
        assert!(h.timeline().len() < 5);
    }

    #[test]
    fn appended_points_reach_the_gpu_copy() {
        let mut layer = CloudLayer::new(PointCloud::new(), 1);
        layer.append(vec![Point3D::new([1.0, 2.0, 3.0])]);
        assert_eq!(layer.len(), 1);
        assert_eq!(layer.gpu.len(), 1);
    }
}
