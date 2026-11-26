//! Application state and the actions behind every control: connecting, live capture, the
//! clean/mesh/export jobs, undo, and the viewport camera.

use std::collections::HashMap;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use eframe::egui::{self, TextureHandle, TextureOptions};
use scancraft_core::{BoundingBox, PointCloud, Transform};
use scancraft_driver::mock::MockDriver;
use scancraft_driver::{DeviceParam, DeviceTransport, DiscoveredDevice, DriverRegistry, ModeSupport, ScanConfig};
use scancraft_driver_revopoint::RevopointDriver;
use scancraft_fusion::{PlaneFilterMode, RansacPlaneFilter, RoiCropFilter, StatisticalOutlierFilter, VoxelFilter};
use scancraft_mesh::Mesher;
use scancraft_render::{Camera, CameraPreset, MeshShading, PointShading};
use scancraft_slam::TrackingStatus;
use tokio::runtime::Runtime;
use tokio::sync::mpsc;

use crate::isolation::{self, ObjectPicking};
use crate::model::*;
use crate::project::{self, AlignMethod, AlignState, PassMeta, PassSource, PassStart, ProjectInfo};
use crate::scan_modes::{self, Choice};
use crate::worker::{self, WorkerConfig};

#[path = "app_orient.rs"]
mod orient_ops;
#[path = "app_project.rs"]
mod project_ops;
#[path = "app_tabs.rs"]
mod tab_ops;
pub use orient_ops::{DatumKind, DatumPicking, OrientPreview, ScalePicking};
pub use project_ops::MarkerUse;
pub use tab_ops::{ClosePrompt, ProjectTab};

/// Live capture state.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Capture {
    Idle,
    Starting,
    Running,
    Paused,
    Stopping,
}

/// What a left drag in the viewport does.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum NavTool {
    Orbit,
    Pan,
    Zoom,
}

/// Tone of the status bar message.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Tone {
    Normal,
    Success,
    Warning,
    Error,
}

/// Which camera image the bottom-left camera view displays.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum InsetView {
    #[default]
    Ir,
    Depth,
    Color,
}

/// Frames per second over a short window.
#[derive(Default)]
pub struct FpsMeter {
    stamps: std::collections::VecDeque<Instant>,
}

impl FpsMeter {
    pub fn tick(&mut self) {
        let now = Instant::now();
        self.stamps.push_back(now);
        while self.stamps.front().is_some_and(|t| now.duration_since(*t) > Duration::from_secs(2)) {
            self.stamps.pop_front();
        }
    }

    pub fn clear(&mut self) {
        self.stamps.clear();
    }

    /// `None` until there are enough recent frames to say.
    pub fn fps(&self) -> Option<f32> {
        let (first, last) = (self.stamps.front()?, self.stamps.back()?);
        if self.stamps.len() < 3 || last.elapsed() > Duration::from_millis(1500) {
            return None;
        }
        let span = last.duration_since(*first).as_secs_f32();
        (span > 0.0).then(|| (self.stamps.len() - 1) as f32 / span)
    }
}

pub struct Settings {
    pub min_mm: f32,
    pub max_mm: f32,
    pub point_budget: usize,
    pub plane_tolerance_mm: f32,
    pub sor_neighbors: usize,
    pub sor_std_dev: f32,
    pub voxel_mm: f32,
    pub crop_min: [f32; 3],
    pub crop_max: [f32; 3],
    /// Reconstruct: triangle size, smoothness (support radius in point spacings) and the
    /// feature-preserving smoothing applied right after.
    pub mc_cell_mm: f32,
    pub mesh_smoothness: f32,
    pub mesh_auto_smooth: usize,
    /// Smooth: strength (normal-filtering iterations) and how much edges are kept (0–1).
    pub smooth_iterations: usize,
    pub smooth_keep_edges: f32,
    /// Fill Holes: largest hole filled (boundary length) and whether the largest opening (the
    /// border of an open scan) stays open.
    pub fill_max_mm: f32,
    pub fill_keep_largest: bool,
    /// Make Solid: grid spacing of the solve and whether marker stickers are found and removed.
    pub solid_cell_mm: f32,
    pub solid_remove_markers: bool,
    /// Make Solid: keep edges sharp on the most trusted pass (highest merge weight).
    pub solid_sharp_top: bool,
    pub decimate_ratio: f32,
    pub export_path: String,
    pub export_format: ExportFormat,
    pub ip: String,
    /// Folder for Save / Open Project.
    pub project_path: String,
    /// Merge: statistical outlier removal and its threshold.
    pub merge_outliers: bool,
    pub merge_sigma: f32,
    /// Align, refine and merge whole passes, table and loose fragments included (otherwise
    /// each pass's object only).
    pub whole_scene: bool,
    /// Last file chosen for Import Scan, and last folder for Add Project: where their dialogs
    /// open next time.
    pub import_path: String,
    pub add_project_path: String,
    /// Optional live SAM 2.1 segmentation of RGB frames while capturing.
    pub live_ai_segment: bool,
}

impl Default for Settings {
    fn default() -> Self {
        Settings {
            min_mm: 150.0,
            max_mm: 450.0,
            point_budget: 3_000_000,
            plane_tolerance_mm: 2.0,
            sor_neighbors: 16,
            sor_std_dev: 1.2,
            voxel_mm: 0.8,
            crop_min: [-150.0, 100.0, -150.0],
            crop_max: [150.0, 450.0, 150.0],
            mc_cell_mm: 0.7,
            mesh_smoothness: 3.0,
            mesh_auto_smooth: 4,
            smooth_iterations: 4,
            smooth_keep_edges: 0.7,
            fill_max_mm: 60.0,
            fill_keep_largest: true,
            solid_cell_mm: 0.4,
            solid_remove_markers: true,
            solid_sharp_top: false,
            decimate_ratio: 0.5,
            export_path: "scan.ply".to_string(),
            export_format: ExportFormat::PlyCloud,
            ip: String::new(),
            project_path: default_project_path(),
            merge_outliers: true,
            merge_sigma: 3.0,
            whole_scene: false,
            import_path: String::new(),
            add_project_path: String::new(),
            live_ai_segment: false,
        }
    }
}

/// A path typed or pasted into a field, without surrounding spaces and quotes (Explorer's "Copy as
/// path" adds quotes).
fn trim_quotes(path: &str) -> String {
    path.trim().trim_matches('"').trim().to_string()
}

/// The feature-preserving smoother for a strength (iterations) and how much edges are kept (0 =
/// smooth like a Laplacian, 1 = keep creases crisp).
fn smoother(strength: usize, keep_edges: f32) -> scancraft_mesh::FeatureSmoother {
    let keep = keep_edges.clamp(0.0, 1.0);
    scancraft_mesh::FeatureSmoother { iterations: strength, sigma_normal: 1.2 + (0.22 - 1.2) * keep, ..Default::default() }
}

/// Make Solid's settings, taken from the panel when the tool runs.
#[derive(Clone, Debug)]
pub(crate) struct SolidJob {
    config: scancraft_mesh::SolidConfig,
    remove_markers: bool,
    sharp_top: bool,
}

impl SolidJob {
    pub(crate) fn from_settings(s: &Settings) -> Self {
        SolidJob {
            config: scancraft_mesh::SolidConfig { cell_mm: s.solid_cell_mm, detect: s.solid_remove_markers.then(Default::default), ..Default::default() },
            remove_markers: s.solid_remove_markers,
            sharp_top: s.solid_sharp_top,
        }
    }
}

/// The Make Solid job's work (off the UI thread): the passes' points and markers into a closed
/// solid.
pub(crate) fn make_solid(input: project_ops::SolidInput, job: SolidJob) -> Result<scancraft_mesh::Solid, String> {
    let SolidJob { mut config, remove_markers, sharp_top } = job;
    let (points, markers, priority) = project_ops::solid_points(input)?;
    if points.len() < 100 {
        return Err("Make Solid needs points with normals: align the passes (or merge them) first".into());
    }
    // The most trusted pass wins where passes cover the same surface; with equal weights all
    // passes are averaged (`SolidConfig::scan_priority`).
    if let Some(order) = priority {
        if sharp_top {
            config.sharp_scans = vec![order[0]];
        }
        config.scan_priority = order;
    }
    let markers = if remove_markers { markers } else { Vec::new() };
    let solid = scancraft_mesh::build_solid(&points, &markers, &config);
    if solid.mesh.triangles.is_empty() {
        return Err("No solid could be built from these points; try a larger cell size".into());
    }
    Ok(solid)
}

/// The status line after Make Solid.
pub(crate) fn solid_message(r: &scancraft_mesh::SolidReport, analysis: &scancraft_mesh::MeshAnalysis) -> String {
    format!(
        "Solid of {} triangles, wall {:.1} mm · {} hole{} filled, {} marker{} removed · {}",
        thousands(analysis.triangle_count),
        r.wall_median_mm,
        r.holes_filled,
        if r.holes_filled == 1 { "" } else { "s" },
        r.marker_sites.len(),
        if r.marker_sites.len() == 1 { "" } else { "s" },
        if analysis.is_watertight { "watertight" } else { "not watertight: check the open edges" }
    )
}

fn default_project_path() -> String {
    default_project_path_for(&ProjectInfo::default().name)
}

/// Where a project called `name` is saved unless the user picks another folder.
fn default_project_path_for(name: &str) -> String {
    std::env::var("USERPROFILE")
        .or_else(|_| std::env::var("HOME"))
        .map(|h| std::path::Path::new(&h).join("Documents").join("ScanCraft").join(name).display().to_string())
        .unwrap_or_else(|_| name.to_string())
}

/// A candidate alignment shown before it is accepted.
pub struct AlignPreview {
    pub pass: u32,
    pub method: AlignMethod,
    pub alignment: scancraft_align::Alignment,
    /// For automatic alignment: the reverse view and whether both directions agree.
    pub check: Option<AlignCheck>,
    /// The pass placed by the candidate, in its colour.
    pub layer: CloudLayer,
}

/// New placements for several passes at once, shown before they are accepted.
pub enum Proposal {
    AlignAll(Box<crate::alignment::AlignAllOutcome>),
    Refine(Box<crate::alignment::RefineOutcome>),
}

/// A [`Proposal`] and the passes it moves, drawn where they would go.
pub struct PosesPreview {
    pub proposal: Proposal,
    pub layers: Vec<(u32, CloudLayer)>,
}

/// How long a toast stays up; one that offers a choice stays longer.
const TOAST_SECONDS: f32 = 6.0;
const CHOICE_TOAST_SECONDS: f32 = 20.0;

/// A choice a toast offers besides Undo.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ToastAction {
    /// The imported pass looks like it is in metres: scale it to mm.
    MetresToMm(u32),
}

impl ToastAction {
    pub fn label(self) -> &'static str {
        match self {
            ToastAction::MetresToMm(_) => "Convert to mm",
        }
    }
}

/// Point-pair picking in the split view: the project model (left) and the pass (right).
pub struct PairPicking {
    pub pass: u32,
    /// (on the project model, project frame; on the pass, its own frame).
    pub pairs: Vec<(Option<glam::Vec3>, Option<glam::Vec3>)>,
    /// Display samples: position (in the pane's frame), normal, colour.
    pub model: Arc<Vec<(glam::Vec3, glam::Vec3, [u8; 3])>>,
    pub moving: Arc<Vec<(glam::Vec3, glam::Vec3, [u8; 3])>>,
    pub model_camera: Camera,
    pub moving_camera: Camera,
}

/// A keyframe's points placed in the world, for the viewport overlay.
pub struct Highlight {
    pub pass: u32,
    pub frame: u64,
    pub discarded: bool,
    pub layer: CloudLayer,
}

pub struct ScanCraftApp {
    pub rt: Arc<Runtime>,
    registry: DriverRegistry,
    dev_tx: mpsc::Sender<DeviceEvent>,
    dev_rx: mpsc::Receiver<DeviceEvent>,
    job_tx: mpsc::Sender<JobResult>,
    job_rx: mpsc::Receiver<JobResult>,
    updates: Option<mpsc::Receiver<ScanUpdate>>,

    // Device.
    pub discovered: Vec<DiscoveredDevice>,
    pub selected_discovered: usize,
    pub discovering: bool,
    pub connecting: bool,
    pub connection: Option<Connection>,
    pub scan_config: ScanConfig,
    pub capture: Capture,
    pub previewing: bool,
    pub preview_starting: bool,

    // Live read-outs.
    pub ir_texture: Option<TextureHandle>,
    pub depth_texture: Option<TextureHandle>,
    pub color_texture: Option<TextureHandle>,
    pub ai_mask_texture: Option<TextureHandle>,
    pub inset_view: InsetView,
    pub show_inset: bool,
    pub distance_mm: Option<f32>,
    pub tracking: Option<TrackingStatus>,
    pub fps: FpsMeter,
    pub budget_reached: bool,

    // Geometry.
    pub passes: Vec<ScanPass>,
    pub active_pass: Option<u32>,
    next_pass_id: u32,
    next_generation: u64,
    /// Generations of preview layers (see `preview_generation`), far above any geometry one.
    next_preview: u64,
    pub merged: Option<MergedLayer>,
    pub mesh: Option<MeshLayer>,
    /// Fill Holes: the current mesh's holes.
    pub holes: Option<HolePreview>,
    /// Open projects; the active one's slot holds a placeholder (its state is in the fields
    /// above). See `app_tabs.rs`.
    pub tabs: Vec<ProjectTab>,
    pub active_tab: usize,
    /// A Save / Discard / Cancel question waiting for an answer.
    pub close_prompt: Option<ClosePrompt>,
    /// Close the active tab once the save that was started for it succeeds.
    pending_close: bool,
    /// The user chose to quit without saving: the next close request goes through.
    pub quit_confirmed: bool,
    /// Save All on quit: the tabs still to save, in order; quits when the last save worked.
    save_all_queue: Option<std::collections::VecDeque<usize>>,
    /// Set when the app should close the window (Save All finished).
    pub close_window: bool,
    /// The history timeline card is open, and whether its button was under the pointer this
    /// frame (a click on the button toggles it instead of closing it as an outside click).
    pub show_history: bool,
    pub history_button_hovered: bool,
    /// A project picked in the Open dialog while the app was busy: opened once it is free.
    pending_open: Option<std::path::PathBuf>,
    pub history: History,
    pub settings: Settings,
    pub project: ProjectInfo,
    /// How the next new pass starts (object moved, or not touched).
    pub next_start: PassStart,
    pub align_preview: Option<AlignPreview>,
    /// Align All or Refine All waiting for Accept / Reject.
    pub poses_preview: Option<PosesPreview>,
    /// Auto Orient's candidate project frame waiting for Accept / Reject.
    pub orient_preview: Option<OrientPreview>,
    pub picking: Option<PairPicking>,
    /// The Orient by Datums tool's picks and the frame they imply.
    pub datum_picking: Option<DatumPicking>,
    /// The Scale from Lengths tool's picked lengths on a pass.
    pub scale_picking: Option<ScalePicking>,
    /// The Isolate Object tool's picks and result.
    pub object_picking: Option<ObjectPicking>,
    /// The AI Segment tool's interactive SAM 2.1 picking/propagation state.
    pub ai_picking: Option<crate::ai_segment::AiPicking>,
    /// Optional AI Pack runtime, installation status, and consent modal flag.
    pub ai_engine: Arc<crate::ai_pack::SharedAiEngine>,
    pub ai_status: crate::ai_pack::AiPackStatus,
    pub ai_pack_modal: bool,
    pub live_ai: Arc<crate::worker::LiveAiState>,
    pub live_ai_status: Option<(f32, String)>,
    /// The undo snapshot of the running job, pushed when it succeeds.
    pending_undo: Option<Snapshot>,
    /// A message shown over the view for a few seconds.
    pub toast: Option<(String, Instant)>,
    /// What the toast offers besides Undo.
    pub toast_action: Option<ToastAction>,
    /// A warning the choice toast also gives, said again after its choice runs (the choice's own
    /// status message would otherwise replace it).
    toast_note: Option<String>,
    /// Dropped scan files and project folders still to add, one job each.
    pending_imports: std::collections::VecDeque<std::path::PathBuf>,
    /// The open file / folder dialog, if any.
    pub picker: crate::file_picker::FilePicker,
    /// Align stage: show each pass in its own colour.
    pub colour_passes: bool,

    // Interface.
    pub stage: Stage,
    pub tool: Option<Tool>,
    pub selection: Option<Node>,
    pub folds: HashMap<String, bool>,
    pub camera: Camera,
    pub nav_tool: NavTool,
    pub point_shading: PointShading,
    pub mesh_shading: MeshShading,
    pub show_grid: bool,
    pub show_edges: bool,
    pub point_size: f32,
    pub display_menu: bool,
    /// Radius of the geometry when the view was last framed automatically (0: not yet).
    framed_radius: f32,
    pub status: String,
    pub tone: Tone,
    pub busy: Option<String>,
    /// Exposure and gain are still the chosen preset's (the user has not changed them).
    pub camera_from_preset: bool,
    /// Development: a mode table the simulator reports instead of its own (`--mock-modes pop4`).
    pub mock_modes: Option<ModeSupport>,
    /// Shared with the scan worker: while set, frames are not registered.
    paused_flag: Arc<AtomicBool>,
    /// The pass the next capture continues, if any.
    continue_pass: Option<u32>,
    /// The marker map the running capture's pass had when the capture started (Continue Pass):
    /// the new capture's marker tracker starts empty, so its map is merged into this one.
    capture_markers: Vec<project::PassMarker>,
    /// Per pass: which of its markers Make Solid would use (see `marker_use`).
    marker_use: std::sync::Mutex<HashMap<u32, Arc<MarkerUse>>>,
    /// The selected keyframe's points, drawn over the model.
    pub highlight: Option<Highlight>,
    /// When the tracker last reported a re-location.
    pub relocated_at: Option<Instant>,
    /// The last frame's markers (or other features) while capturing.
    pub overlay: Option<scancraft_pipeline::ImageOverlay>,
    /// Frame rates reported by the scan worker while capturing.
    pub rates: Option<FrameRates>,
    /// Width of the camera view's image, points (the user resizes it; kept for the session).
    pub inset_width: f32,
    /// Where the camera view was drawn last frame (the browser makes room for it).
    pub inset_rect: Option<egui::Rect>,
}

/// Device to connect as soon as the app starts.
pub enum StartConnection {
    None,
    Mock,
    Ip(String),
    /// A recorded marker stream (`--replay <recording.bin> <camparam.yaml>`).
    Replay(std::path::PathBuf, std::path::PathBuf),
}

impl ScanCraftApp {
    pub fn new(rt: Arc<Runtime>, start: StartConnection) -> Self {
        let registry = DriverRegistry::new();
        {
            let registry = registry.clone();
            rt.block_on(async move {
                registry.register(Arc::new(RevopointDriver)).await;
                registry.register(Arc::new(MockDriver)).await;
            });
        }
        let (dev_tx, dev_rx) = mpsc::channel(32);
        let (job_tx, job_rx) = mpsc::channel(8);
        let scan_config = ScanConfig::default();
        let ai_engine = Arc::new(crate::ai_pack::SharedAiEngine::default());
        let ai_status = crate::ai_pack::inspect_pack(Some(&ai_engine));
        let live_ai = Arc::new(crate::worker::LiveAiState::new(Arc::clone(&ai_engine)));
        let mut app = ScanCraftApp {
            rt,
            registry,
            dev_tx,
            dev_rx,
            job_tx,
            job_rx,
            updates: None,
            discovered: Vec::new(),
            selected_discovered: 0,
            discovering: false,
            connecting: false,
            connection: None,
            scan_config,
            capture: Capture::Idle,
            previewing: false,
            preview_starting: false,
            ir_texture: None,
            depth_texture: None,
            color_texture: None,
            ai_mask_texture: None,
            inset_view: InsetView::Ir,
            show_inset: true,
            distance_mm: None,
            tracking: None,
            fps: FpsMeter::default(),
            budget_reached: false,
            passes: Vec::new(),
            active_pass: None,
            next_pass_id: 1,
            next_generation: 1,
            next_preview: 1 << 56,
            merged: None,
            holes: None,
            tabs: vec![ProjectTab::empty(String::new())],
            active_tab: 0,
            close_prompt: None,
            pending_close: false,
            quit_confirmed: false,
            save_all_queue: None,
            close_window: false,
            show_history: false,
            history_button_hovered: false,
            pending_open: None,
            mesh: None,
            history: History::default(),
            settings: Settings::default(),
            project: ProjectInfo::default(),
            next_start: PassStart::Moved,
            align_preview: None,
            poses_preview: None,
            orient_preview: None,
            picking: None,
            datum_picking: None,
            scale_picking: None,
            object_picking: None,
            ai_picking: None,
            ai_engine,
            ai_status,
            ai_pack_modal: false,
            live_ai,
            live_ai_status: None,
            pending_undo: None,
            toast: None,
            toast_action: None,
            toast_note: None,
            pending_imports: Default::default(),
            picker: Default::default(),
            colour_passes: true,
            stage: Stage::Connect,
            tool: None,
            selection: None,
            folds: HashMap::new(),
            camera: Camera::default(),
            nav_tool: NavTool::Orbit,
            point_shading: PointShading::Lit,
            mesh_shading: MeshShading::Lit,
            show_grid: true,
            show_edges: false,
            point_size: 2.0,
            display_menu: false,
            framed_radius: 0.0,
            status: "Ready".to_string(),
            tone: Tone::Normal,
            busy: None,
            camera_from_preset: false,
            mock_modes: None,
            paused_flag: Arc::new(AtomicBool::new(false)),
            continue_pass: None,
            capture_markers: Vec::new(),
            marker_use: Default::default(),
            highlight: None,
            relocated_at: None,
            overlay: None,
            rates: None,
            inset_width: crate::ui_viewport::INSET_DEFAULT_W,
            inset_rect: None,
        };
        app.camera.set_view_preset(CameraPreset::Isometric);
        app.camera.target = glam::Vec3::new(0.0, 250.0, 0.0);
        app.camera.set_view_preset(CameraPreset::Isometric);
        match start {
            StartConnection::None => {}
            StartConnection::Mock => app.connect_mock(),
            StartConnection::Replay(recording, camparam) => app.connect_replay(recording, camparam),
            StartConnection::Ip(ip) => {
                app.settings.ip = ip;
                app.connect_ip();
            }
        }
        app
    }

    fn generation(&mut self) -> u64 {
        self.next_generation += 1;
        self.next_generation
    }

    /// A layer generation for something only shown, not part of the project (previews,
    /// highlights, outlines): unique like [`generation`](Self::generation) but it does not mark
    /// the project as changed.
    pub(crate) fn preview_generation(&mut self) -> u64 {
        self.next_preview += 1;
        self.next_preview
    }

    pub fn say(&mut self, tone: Tone, message: impl Into<String>) {
        self.tone = tone;
        self.status = message.into();
    }

    /// A message over the view for a few seconds.
    pub fn show_toast(&mut self, message: impl Into<String>) {
        self.toast = Some((message.into(), Instant::now()));
        self.toast_action = None;
        self.toast_note = None;
    }

    /// A message over the view that also offers `action`.
    pub fn show_choice_toast(&mut self, message: impl Into<String>, action: ToastAction) {
        self.show_toast(message);
        self.toast_action = Some(action);
    }

    /// The toast still showing, if any.
    pub fn current_toast(&self) -> Option<&str> {
        let seconds = if self.toast_action.is_some() { CHOICE_TOAST_SECONDS } else { TOAST_SECONDS };
        self.toast.as_ref().filter(|(_, at)| at.elapsed().as_secs_f32() < seconds).map(|(m, _)| m.as_str())
    }

    /// Runs the toast's choice and closes it.
    pub fn run_toast_action(&mut self) {
        let Some(action) = self.toast_action.take() else { return };
        self.toast = None;
        let note = self.toast_note.take();
        match action {
            ToastAction::MetresToMm(id) => self.scale_pass(id, 1000.0),
        }
        if let Some(note) = note {
            self.say(Tone::Warning, format!("{} · {note}", self.status));
        }
    }

    /// Drops the mesh: it was made from passes that have since moved (or from an older merge),
    /// and meshing must not start from it. The caller has pushed an undo snapshot that still
    /// holds it. Returns whether anything was dropped. (Each pass's cleaning is in its own frame
    /// and moves with it.)
    pub fn invalidate_derived(&mut self, reason: &str) -> bool {
        self.invalidate_results(reason, false)
    }

    /// [`invalidate_derived`](Self::invalidate_derived), and with `merged` also the merged
    /// model (named in the same message).
    fn invalidate_results(&mut self, reason: &str, merged: bool) -> bool {
        let merged = merged && self.merged.is_some();
        let dropped: Vec<&str> = [merged.then_some("merged model"), self.mesh.is_some().then_some("mesh")].into_iter().flatten().collect();
        if merged {
            self.merged = None;
            if self.selection == Some(Node::Merged) {
                self.selection = None;
            }
        }
        self.mesh = None;
        // An open Isolate Object / AI Segment tool works on a copy of the old points, placed as they were.
        if self.object_picking.take().is_some() && self.tool == Some(Tool::IsolateObject) {
            self.tool = None;
        }
        if self.ai_picking.take().is_some() && self.tool == Some(Tool::AiSegment) {
            self.tool = None;
        }
        if matches!(self.selection, Some(Node::Mesh)) {
            self.selection = None;
        }
        if dropped.is_empty() {
            return false;
        }
        self.show_toast(format!("{reason}: deleted the {}, which no longer match. Undo restores them.", dropped.join(" and ")));
        true
    }

    /// Forgets every proposal waiting for a decision.
    pub fn clear_previews(&mut self) {
        self.align_preview = None;
        self.poses_preview = None;
        self.orient_preview = None;
    }

    pub fn folded(&self, key: &str, default_open: bool) -> bool {
        self.folds.get(key).copied().unwrap_or(default_open)
    }

    // ------------------------------------------------------------------ device

    pub fn discover(&mut self) {
        if self.discovering {
            return;
        }
        self.discovering = true;
        self.say(Tone::Normal, "Searching for scanners…");
        let registry = self.registry.clone();
        let tx = self.dev_tx.clone();
        self.rt.spawn(async move {
            let devices = registry.discover_all().await;
            let _ = tx.send(DeviceEvent::DiscoveryFinished(devices)).await;
        });
    }

    fn connect_transport(&mut self, driver_id: String, transport: DeviceTransport, label: String) {
        if self.connecting || self.connection.is_some() {
            return;
        }
        self.connecting = true;
        self.say(Tone::Normal, format!("Connecting to {label}…"));
        let registry = self.registry.clone();
        let tx = self.dev_tx.clone();
        self.rt.spawn(async move {
            match registry.connect(&driver_id, &transport).await {
                Ok(device) => {
                    let info = device.info().clone();
                    let capabilities = device.capabilities().clone();
                    let device = Arc::new(tokio::sync::Mutex::new(device));
                    let _ = tx.send(DeviceEvent::Connected { device, info, capabilities, driver_id }).await;
                }
                Err(e) => {
                    let _ = tx.send(DeviceEvent::ConnectFailed(e.to_string())).await;
                }
            }
        });
    }

    pub fn connect_discovered(&mut self) {
        let Some(dev) = self.discovered.get(self.selected_discovered).cloned() else { return };
        self.connect_transport(dev.driver_id.clone(), dev.selected_transport.clone(), dev.model_name.clone());
    }

    /// Connect a recorded marker scan as the scanner (development, `--replay`).
    pub fn connect_replay(&mut self, recording: std::path::PathBuf, camparam: std::path::PathBuf) {
        match crate::replay::ReplayDevice::open(recording, camparam) {
            Ok(device) => {
                use scancraft_driver::ScannerDevice;
                let info = device.info().clone();
                let capabilities = device.capabilities().clone();
                let device: Box<dyn ScannerDevice> = Box::new(device);
                let device = Arc::new(tokio::sync::Mutex::new(device));
                // The recording is a Full-field HD marker stream.
                self.scan_config.tracking = scancraft_driver::TrackingMode::Marker;
                self.connecting = true;
                let tx = self.dev_tx.clone();
                self.rt.spawn(async move {
                    let _ = tx.send(DeviceEvent::Connected { device, info, capabilities, driver_id: "replay".into() }).await;
                });
            }
            Err(e) => self.say(Tone::Error, format!("Cannot open the recording: {e}")),
        }
    }

    pub fn connect_mock(&mut self) {
        self.connect_transport("mock".into(), DeviceTransport::Virtual { name: "simulated_scanner_0".into() }, "the simulator".into());
    }

    pub fn ip_valid(&self) -> bool {
        self.settings.ip.trim().parse::<std::net::Ipv4Addr>().is_ok()
    }

    pub fn connect_ip(&mut self) {
        if !self.ip_valid() {
            self.say(Tone::Error, "Enter an IPv4 address, for example 192.168.1.20");
            return;
        }
        let ip = self.settings.ip.trim().to_string();
        let transport = DeviceTransport::WiFi { ip: ip.clone(), control_port: 80, data_port: 8871, imu_port: 9999 };
        self.connect_transport("revopoint".into(), transport, ip);
    }

    pub fn disconnect(&mut self) {
        self.overlay = None;
        self.rates = None;
        self.stop_preview();
        self.stop_scan();
        if let Some(conn) = self.connection.take() {
            let tx = self.dev_tx.clone();
            self.rt.spawn(async move {
                let mut dev = conn.device.lock().await;
                let _ = dev.stop_scan().await;
                let _ = dev.disconnect().await;
                let _ = tx.send(DeviceEvent::Disconnected).await;
            });
        }
        self.updates = None;
        self.distance_mm = None;
        self.tracking = None;
        self.fps.clear();
        self.say(Tone::Normal, "Disconnected");
    }

    pub fn start_preview(&mut self) {
        let Some(conn) = &self.connection else { return };
        if self.previewing || self.preview_starting || self.capture != Capture::Idle {
            return;
        }
        self.preview_starting = true;
        let device = conn.device.clone();
        let tx = self.dev_tx.clone();
        self.rt.spawn(async move {
            let mut dev = device.lock().await;
            match dev.start_preview().await {
                Ok(rx) => {
                    let _ = tx.send(DeviceEvent::PreviewStarted(rx)).await;
                }
                Err(e) => {
                    let _ = tx.send(DeviceEvent::PreviewFailed(e.to_string())).await;
                }
            }
        });
    }

    pub fn stop_preview(&mut self) {
        if !self.previewing && !self.preview_starting {
            return;
        }
        self.previewing = false;
        self.preview_starting = false;
        self.updates = None;
        self.fps.clear();
        if let Some(conn) = &self.connection {
            let device = conn.device.clone();
            self.rt.spawn(async move {
                let _ = device.lock().await.stop_preview().await;
            });
        }
    }

    /// The connected scanner's modes, or `None` when nothing is connected.
    pub fn modes(&self) -> Option<&ModeSupport> {
        self.connection.as_ref().map(|c| &c.capabilities.modes)
    }

    pub fn mode_choice(&self) -> Choice {
        Choice { light: self.scan_config.light, tracking: self.scan_config.tracking, object: self.scan_config.object }
    }

    /// Change scan mode, tracking or object; the camera settings follow the preset of the new
    /// light source and object.
    pub fn set_mode_choice(&mut self, c: Choice) {
        let preset_changed = c.light != self.scan_config.light || c.object != self.scan_config.object;
        self.scan_config.light = c.light;
        self.scan_config.tracking = c.tracking;
        self.scan_config.object = c.object;
        if preset_changed {
            self.apply_preset();
        }
    }

    /// Exposure, gain and depth window from the preset of the chosen light source and object
    /// (the device applies the same preset when the stream starts).
    pub fn apply_preset(&mut self) {
        self.camera_from_preset = false;
        self.scan_config.camera_override = false;
        if let Some((exposure_us, gain, (near, far))) = scan_modes::preset(self.scan_config.light, self.scan_config.object) {
            self.scan_config.depth_exposure_us = exposure_us;
            self.scan_config.depth_gain = gain;
            self.settings.min_mm = near;
            self.settings.max_mm = far;
            self.camera_from_preset = true;
        }
    }

    /// Why the current choice cannot be scanned, if it cannot.
    pub fn mode_blocked(&self) -> Option<String> {
        let m = self.modes()?;
        let c = self.mode_choice();
        if m.can_scan(c.light, c.tracking, c.object) {
            return None;
        }
        let o = scan_modes::options(Some(m), c);
        let reason = o
            .lights
            .iter()
            .filter(|x| x.value == c.light)
            .filter_map(|x| x.disabled.clone())
            .chain(o.tracking.iter().filter(|x| x.value == c.tracking).filter_map(|x| x.disabled.clone()))
            .chain(o.objects.iter().filter(|x| x.value == c.object).filter_map(|x| x.disabled.clone()))
            .next();
        Some(reason.unwrap_or_else(|| format!("{} with {} tracking is not available on this scanner", c.light.label(), c.tracking.label())))
    }

    pub fn start_scan(&mut self) {
        self.continue_pass = None;
        self.begin_capture();
    }

    /// Start capturing into an existing pass, with its tracker: the scan carries on from that
    /// pass's last kept frame.
    pub fn continue_scan(&mut self, pass: u32) {
        if !self.passes.iter().any(|p| p.id == pass && !p.is_fixed() && !p.is_cleaned()) {
            return;
        }
        self.continue_pass = Some(pass);
        self.begin_capture();
    }

    fn begin_capture(&mut self) {
        let Some(conn) = &self.connection else {
            self.say(Tone::Warning, "Connect a scanner first");
            return;
        };
        if self.capture != Capture::Idle {
            return;
        }
        let device = conn.device.clone();
        if let Some(reason) = self.mode_blocked() {
            self.say(Tone::Error, format!("Cannot start: {reason}"));
            return;
        }
        self.stop_preview();
        self.capture = Capture::Starting;
        self.say(Tone::Normal, "Starting capture…");
        let config = self.scan_config.clone();
        let tx = self.dev_tx.clone();
        self.rt.spawn(async move {
            let mut dev = device.lock().await;
            match dev.start_scan(&config).await {
                Ok(rx) => {
                    let _ = tx.send(DeviceEvent::ScanStarted(rx, config)).await;
                }
                Err(e) => {
                    let _ = tx.send(DeviceEvent::ScanFailed(e.to_string())).await;
                }
            }
        });
    }

    pub fn toggle_pause(&mut self) {
        let Some(conn) = &self.connection else { return };
        let pause = match self.capture {
            Capture::Running => true,
            Capture::Paused => false,
            _ => return,
        };
        if pause {
            // Stop registering at once; frames already in flight must not reach the model.
            self.paused_flag.store(true, Ordering::Relaxed);
        }
        let device = conn.device.clone();
        let tx = self.dev_tx.clone();
        self.rt.spawn(async move {
            let mut dev = device.lock().await;
            let result = if pause { dev.pause_scan().await } else { dev.resume_scan().await };
            match result {
                Ok(()) => {
                    let _ = tx.send(DeviceEvent::Paused(pause)).await;
                }
                Err(e) => {
                    let _ = tx.send(DeviceEvent::ScanFailed(e.to_string())).await;
                }
            }
        });
    }

    pub fn stop_scan(&mut self) {
        if matches!(self.capture, Capture::Idle | Capture::Stopping) {
            return;
        }
        self.capture = Capture::Stopping;
        self.updates = None;
        if let Some(conn) = &self.connection {
            let device = conn.device.clone();
            let tx = self.dev_tx.clone();
            self.rt.spawn(async move {
                let _ = device.lock().await.stop_scan().await;
                let _ = tx.send(DeviceEvent::ScanStopped).await;
            });
        } else {
            self.capture = Capture::Idle;
        }
    }

    /// Send a runtime parameter to the device (exposure, gain, laser power).
    pub fn set_parameter(&mut self, param: DeviceParam) {
        let Some(conn) = &self.connection else { return };
        let device = conn.device.clone();
        let tx = self.dev_tx.clone();
        self.rt.spawn(async move {
            if let Err(e) = device.lock().await.set_parameter(param).await {
                let _ = tx.send(DeviceEvent::ParameterFailed(e.to_string())).await;
            }
        });
    }

    fn worker_config(&self) -> WorkerConfig {
        WorkerConfig {
            min_mm: self.settings.min_mm,
            max_mm: self.settings.max_mm,
            point_pitch_mm: self.scan_config.accuracy.point_pitch_mm(),
            imu_to_sensor: self.connection.as_ref().and_then(|c| c.info.imu_to_sensor),
        }
    }

    pub fn poll_device_events(&mut self) {
        while let Ok(event) = self.dev_rx.try_recv() {
            match event {
                DeviceEvent::DiscoveryFinished(devices) => {
                    self.discovering = false;
                    self.selected_discovered = 0;
                    let n = devices.len();
                    self.discovered = devices;
                    if n == 0 {
                        self.say(Tone::Warning, "No scanners found");
                    } else {
                        self.say(Tone::Normal, format!("Found {n} scanner{}", if n == 1 { "" } else { "s" }));
                    }
                }
                DeviceEvent::Connected { device, info, capabilities, driver_id } => {
                    self.connecting = false;
                    let message = format!("Connected to {} ({})", info.model, info.active_transport.description());
                    let mut capabilities = capabilities;
                    if let (Some(modes), DeviceTransport::Virtual { .. }) = (&self.mock_modes, &info.active_transport) {
                        // Development: give the simulator another device's mode table.
                        capabilities.modes = modes.clone();
                    }
                    let conn = Connection { device, info, capabilities, driver_id };
                    self.settings.min_mm = conn.info.recommended_distance_range_mm.0.min(self.settings.min_mm);
                    self.settings.max_mm = conn.info.recommended_distance_range_mm.1.max(self.settings.max_mm);
                    let choice = scan_modes::reconcile(&conn.capabilities.modes, self.mode_choice());
                    let has_rgb = conn.capabilities.has_rgb_camera;
                    self.connection = Some(conn);
                    self.scan_config.light = choice.light;
                    self.scan_config.tracking = choice.tracking;
                    self.scan_config.object = choice.object;
                    self.scan_config.color_texture = has_rgb;
                    self.apply_preset();
                    self.selection = Some(Node::Device);
                    self.say(Tone::Success, message);
                    if self.stage == Stage::Connect {
                        self.stage = Stage::Scan;
                    }
                }
                DeviceEvent::ConnectFailed(e) => {
                    self.connecting = false;
                    self.say(Tone::Error, format!("Connection failed: {e}"));
                }
                DeviceEvent::PreviewStarted(rx) => {
                    self.preview_starting = false;
                    if self.connection.is_none() || self.capture != Capture::Idle {
                        continue;
                    }
                    self.previewing = true;
                    let (tx, urx) = mpsc::channel(4);
                    self.updates = Some(urx);
                    self.fps.clear();
                    self.rt.spawn(worker::preview_worker(rx, tx));
                    self.say(Tone::Normal, "Live preview");
                }
                DeviceEvent::PreviewFailed(e) => {
                    self.preview_starting = false;
                    self.say(Tone::Error, format!("Preview failed: {e}"));
                }
                DeviceEvent::ScanStarted(rx, scan_config) => {
                    if self.capture != Capture::Starting {
                        continue;
                    }
                    self.capture = Capture::Running;
                    self.budget_reached = false;
                    let config = self.worker_config();
                    let continued = self.continue_pass.take().and_then(|id| self.passes.iter_mut().find(|p| p.id == id));
                    let (id, tracker, verb) = match continued {
                        Some(pass) => {
                            pass.layer.visible = true;
                            self.capture_markers = pass.markers.iter().map(|m| project::PassMarker { matched: false, ..*m }).collect();
                            (pass.id, pass.tracker.clone(), "Continuing")
                        }
                        None => {
                            self.capture_markers.clear();
                            let id = self.next_pass_id;
                            self.next_pass_id += 1;
                            let generation = self.generation();
                            let tracker = worker::new_tracker(&config);
                            let device = self.connection.as_ref().map(|c| c.info.clone());
                            let first = !self.passes.iter().any(|p| p.meta.alignment.is_placed());
                            let mut meta = PassMeta::new(scan_config.clone(), device, if first { PassStart::Moved } else { self.next_start });
                            meta.alignment = if first {
                                AlignState::Reference
                            } else if meta.start == PassStart::SamePosition {
                                if self.seed_tracker(&tracker) {
                                    AlignState::Locating
                                } else {
                                    self.show_toast("Same Position: the placed passes could not be loaded as a starting model; this pass starts unaligned (Align All places it afterwards)");
                                    AlignState::Unaligned
                                }
                            } else {
                                AlignState::Unaligned
                            };
                            self.passes.push(ScanPass {
                                id,
                                name: format!("Pass {id}"),
                                layer: CloudLayer::new(PointCloud::new(), generation),
                                frames: 0,
                                tracker: tracker.clone(),
                                keyframes: Vec::new(),
                                meta,
                                tinted: None,
                                mask: isolation::new_cache(),
                                surface: None,
                                edit: None,
                                rgb_keyframes: Vec::new(),
                                markers: Vec::new(),
                            });
                            (id, tracker, "Capturing")
                        }
                    };
                    self.active_pass = Some(id);
                    self.selection = Some(Node::Pass(id));
                    self.paused_flag = Arc::new(AtomicBool::new(false));
                    self.live_ai_status = None;
                    let (tx, urx) = mpsc::channel(4);
                    self.updates = Some(urx);
                    self.fps.clear();
                    self.rt.spawn(worker::scan_worker(rx, tx, config, scan_config, tracker, self.paused_flag.clone(), Arc::clone(&self.live_ai)));
                    self.say(Tone::Normal, format!("{verb} pass {id}"));
                }
                DeviceEvent::ScanFailed(e) => {
                    if self.capture == Capture::Starting {
                        self.capture = Capture::Idle;
                        self.continue_pass = None;
                    }
                    self.say(Tone::Error, format!("Capture failed: {e}"));
                }
                DeviceEvent::ScanStopped => {
                    self.capture = Capture::Idle;
                    self.updates = None;
                    self.overlay = None;
                    self.rates = None;
                    // Nothing is matched once the capture is over.
                    for m in self.passes.iter_mut().flat_map(|p| p.markers.iter_mut()) {
                        m.matched = false;
                    }
                    self.fps.clear();
                    self.tracking = None;
                    let finished = self.active_pass.take();
                    // Updates still in flight were dropped with the channel: take the pass's
                    // model and frame list from its tracker so the two agree.
                    if let Some(pass) = finished.and_then(|id| self.passes.iter_mut().find(|p| p.id == id)) {
                        if let Ok(tracker) = pass.tracker.try_lock() {
                            let points = project::place(tracker.fused_points(), &pass.meta.transform);
                            pass.keyframes = tracker.keyframes().iter().map(KeyframeInfo::of).collect();
                            drop(tracker);
                            let visible = pass.layer.visible;
                            self.next_generation += 1;
                            pass.layer = CloudLayer::new(PointCloud::from_points(points), self.next_generation);
                            pass.layer.visible = visible;
                        }
                        // Stopped before it found itself in the project model.
                        if pass.meta.alignment == AlignState::Locating {
                            pass.meta.alignment = AlignState::Unaligned;
                        }
                    }
                    let points = finished.and_then(|id| self.passes.iter().find(|p| p.id == id)).map(|p| p.layer.len()).unwrap_or(0);
                    // A pass that caught nothing is not kept.
                    self.passes.retain(|p| p.layer.len() > 0 || Some(p.id) != finished);
                    self.fit_view();
                    self.say(Tone::Normal, format!("Capture stopped · {} points", thousands(points)));
                }
                DeviceEvent::Paused(paused) => {
                    if matches!(self.capture, Capture::Running | Capture::Paused) {
                        self.capture = if paused { Capture::Paused } else { Capture::Running };
                        self.paused_flag.store(paused, Ordering::Relaxed);
                        self.say(Tone::Normal, if paused { "Capture paused" } else { "Capture resumed" });
                    }
                }
                DeviceEvent::ParameterFailed(e) => self.say(Tone::Warning, format!("The scanner rejected a setting: {e}")),
                DeviceEvent::Disconnected => {}
            }
        }
    }

    /// The running capture's marker tracker map (pass frame), merged into the markers its pass had
    /// when the capture started (see [`project::PassMarker::merged`]).
    fn apply_capture_marker_map(&mut self, map: &[([f32; 3], bool)]) {
        if map.is_empty() {
            return;
        }
        if let Some(pass) = self.active_pass.and_then(|id| self.passes.iter_mut().find(|p| p.id == id)) {
            pass.markers = project::PassMarker::merged(&self.capture_markers, map);
        }
    }

    pub fn poll_updates(&mut self, ctx: &egui::Context) {
        let Some(rx) = self.updates.as_mut() else { return };
        let mut updates = Vec::new();
        while let Ok(u) = rx.try_recv() {
            updates.push(u);
        }
        for u in updates {
            self.fps.tick();
            self.distance_mm = u.distance_mm;
            if let Some(img) = u.ir_image {
                match &mut self.ir_texture {
                    Some(t) => t.set(img, TextureOptions::LINEAR),
                    None => self.ir_texture = Some(ctx.load_texture("ir_preview", img, TextureOptions::LINEAR)),
                }
            }
            if let Some(img) = u.depth_image {
                match &mut self.depth_texture {
                    Some(t) => t.set(img, TextureOptions::LINEAR),
                    None => self.depth_texture = Some(ctx.load_texture("depth_preview", img, TextureOptions::LINEAR)),
                }
            }
            if let Some(img) = u.color_image {
                match &mut self.color_texture {
                    Some(t) => t.set(img, TextureOptions::LINEAR),
                    None => self.color_texture = Some(ctx.load_texture("color_preview", img, TextureOptions::LINEAR)),
                }
            }
            if let Some(img) = u.ai_mask_overlay {
                match &mut self.ai_mask_texture {
                    Some(t) => t.set(img, TextureOptions::LINEAR),
                    None => self.ai_mask_texture = Some(ctx.load_texture("ai_mask_preview", img, TextureOptions::LINEAR)),
                }
            }
            if u.ai_status.is_some() {
                self.live_ai_status = u.ai_status;
            }
            if !u.rgb_keyframes.is_empty() {
                if let Some(pass) = self.active_pass.and_then(|id| self.passes.iter_mut().find(|p| p.id == id)) {
                    pass.rgb_keyframes.extend(u.rgb_keyframes);
                }
            }
            if let Some(t) = &u.tracking {
                if t.relocalized {
                    self.relocated_at = Some(Instant::now());
                }
            }
            if u.tracking.is_some() {
                self.tracking = u.tracking;
            }
            if u.rates.is_some() {
                self.rates = u.rates;
            }
            if let Some(o) = u.overlay {
                self.apply_capture_marker_map(&o.map);
                self.overlay = Some(o);
            }
            if !u.keyframes.is_empty() {
                if let Some(pass) = self.active_pass.and_then(|id| self.passes.iter_mut().find(|p| p.id == id)) {
                    pass.keyframes.extend(u.keyframes);
                }
            }
            if let Some(seed) = u.seed {
                self.seed_progress(seed);
            }
            // Whatever the tracker fused belongs in the pass, even if it arrives just after a
            // pause; the worker stops registering while paused.
            if !matches!(self.capture, Capture::Running | Capture::Paused) || u.new_points.is_empty() {
                continue;
            }
            let total: usize = self.passes.iter().map(|p| p.layer.len()).sum();
            if total + u.new_points.len() > self.settings.point_budget {
                if !self.budget_reached {
                    self.budget_reached = true;
                    self.say(Tone::Warning, format!("Point budget of {} reached; new points are not kept", thousands(self.settings.point_budget)));
                }
                continue;
            }
            if let Some(pass) = self.active_pass.and_then(|id| self.passes.iter_mut().find(|p| p.id == id)) {
                pass.frames += 1;
                pass.layer.append(project::place(u.new_points, &pass.meta.transform));
            }
            // Frame the first points, and again whenever the scan has grown well past the view.
            if let Some(b) = self.data_bbox().filter(|_| self.total_points() >= 500) {
                if self.framed_radius == 0.0 || b.radius() > self.framed_radius * 1.6 {
                    self.camera.fit_bounding_box(&b);
                    self.framed_radius = b.radius().max(1.0);
                }
            }
        }
    }

    // ------------------------------------------------------------------ geometry

    pub fn total_points(&self) -> usize {
        self.passes.iter().map(|p| p.layer.len()).sum()
    }

    /// The passes meshing and export work on: the visible ones, or all when none is visible
    /// (a result hides the passes it was made from). Cleaning uses `clean_targets` instead.
    pub fn input_passes(&self) -> impl Iterator<Item = &ScanPass> {
        let any_visible = self.passes.iter().any(|p| p.layer.visible);
        self.passes.iter().filter(move |p| p.layer.visible || !any_visible)
    }

    /// The visible passes (all when none is visible) put together: what meshing and point export
    /// take when the merged model is not shown.
    pub fn merged_passes(&self) -> PointCloud {
        let mut points = Vec::with_capacity(self.input_passes().map(|p| p.layer.len()).sum());
        for p in self.input_passes() {
            points.extend_from_slice(&p.layer.cloud.points);
        }
        PointCloud::from_points(points)
    }

    /// Whether meshing and export take the merged model: it is shown, or nothing is shown and
    /// it exists.
    fn working_is_merged(&self) -> bool {
        self.merged.as_ref().is_some_and(|m| m.layer.visible || !self.passes.iter().any(|p| p.layer.visible))
    }

    /// The cloud meshing and point export work on: the merged model when it is shown, else the
    /// visible passes (each as cleaned).
    pub fn working_cloud(&self) -> PointCloud {
        match &self.merged {
            Some(m) if self.working_is_merged() => (*m.layer.cloud).clone(),
            _ => self.merged_passes(),
        }
    }

    pub fn working_len(&self) -> usize {
        match &self.merged {
            Some(m) if self.working_is_merged() => m.layer.len(),
            _ => self.input_passes().map(|p| p.layer.len()).sum(),
        }
    }

    /// What [`working_cloud`](Self::working_cloud) is, for labels.
    pub fn working_source(&self) -> &'static str {
        if self.working_is_merged() { "Merged model" } else { "Scan passes" }
    }

    /// What the Clean tools work on, each on its own: every shown pass, and the merged model
    /// when it is shown. Hidden ones are left alone.
    pub fn clean_targets(&self) -> Vec<CleanTarget> {
        let mut targets: Vec<CleanTarget> = self.passes.iter().filter(|p| p.layer.visible && p.layer.len() > 0).map(|p| CleanTarget::Pass(p.id)).collect();
        if self.merged.as_ref().is_some_and(|m| m.layer.visible && m.layer.len() > 0) {
            targets.push(CleanTarget::Merged);
        }
        targets
    }

    /// Points the Clean tools would work on.
    pub fn clean_len(&self) -> usize {
        self.clean_targets().iter().map(|t| self.target_cloud(*t).map_or(0, |c| c.len())).sum()
    }

    /// A clean target's points as shown (project frame).
    pub fn target_cloud(&self, target: CleanTarget) -> Option<&PointCloud> {
        match target {
            CleanTarget::Pass(id) => self.passes.iter().find(|p| p.id == id).map(|p| &*p.layer.cloud),
            CleanTarget::Merged => self.merged.as_ref().map(|m| &*m.layer.cloud),
        }
    }

    pub fn target_name(&self, target: CleanTarget) -> String {
        match target {
            CleanTarget::Pass(id) => self.pass_name(id),
            CleanTarget::Merged => "Merged model".to_string(),
        }
    }

    /// The Clean tools' targets for labels: "Pass 1, Pass 2", or how many.
    pub fn clean_targets_label(&self) -> String {
        let targets = self.clean_targets();
        match targets.len() {
            0 => "Nothing shown".to_string(),
            1..=3 => targets.iter().map(|t| self.target_name(*t)).collect::<Vec<_>>().join(", "),
            n => format!("{n} passes"),
        }
    }

    pub fn data_bbox(&self) -> Option<BoundingBox> {
        let mut boxes: Vec<BoundingBox> = self.passes.iter().filter(|p| p.layer.visible && p.layer.len() > 0).map(|p| p.layer.cloud.bbox).collect();
        if let Some(m) = self.merged.as_ref().filter(|m| m.layer.visible && m.layer.len() > 0) {
            boxes.push(m.layer.cloud.bbox);
        }
        if let Some(m) = self.mesh.as_ref().filter(|m| m.visible) {
            boxes.push(m.mesh.bbox);
        }
        union_bbox(boxes.iter())
    }

    pub fn has_geometry(&self) -> bool {
        self.total_points() > 0 || self.merged.is_some() || self.mesh.is_some()
    }

    /// The project as it is now, for the undo history. Cheap: layers are shared, not copied.
    fn snapshot(&self, label: &str) -> Snapshot {
        Snapshot {
            merged: self.merged.clone(),
            mesh: self.mesh.clone(),
            passes: self
                .passes
                .iter()
                .map(|p| PassSnapshot {
                    id: p.id,
                    name: p.name.clone(),
                    layer: p.layer.clone(),
                    frames: p.frames,
                    tracker: p.tracker.clone(),
                    keyframes: p.keyframes.clone(),
                    meta: p.meta.clone(),
                    mask: p.mask.clone(),
                    surface: p.surface.clone(),
                    edit: p.edit.clone(),
                    rgb_keyframes: p.rgb_keyframes.clone(),
                    markers: p.markers.clone(),
                })
                .collect(),
            label: label.to_string(),
            at: std::time::SystemTime::now(),
            #[cfg(test)]
            summary_for_test: String::new(),
        }
    }

    /// Records the state before an action in the undo history, then drops the oldest steps if
    /// the history holds too much memory of its own.
    fn record(&mut self, snapshot: Snapshot) {
        self.history.push(snapshot);
        let live = self.snapshot("").blocks();
        self.history.trim(&live);
    }

    /// Puts a snapshot's project back. Its layers come back as they were (same points, same GPU
    /// copy); only what is shown or hidden stays as it is now.
    fn restore(&mut self, s: Snapshot) {
        self.clear_previews();
        self.picking = None;
        self.object_picking = None;
        self.ai_picking = None;
        self.datum_picking = None;
        self.scale_picking = None;
        if matches!(self.tool, Some(Tool::IsolateObject | Tool::AiSegment | Tool::Datums | Tool::Scale(_))) {
            self.tool = None;
        }
        let merged_visible = self.merged.as_ref().is_none_or(|m| m.layer.visible);
        self.merged = s.merged.map(|mut m| {
            m.layer.visible = merged_visible;
            m
        });
        let mesh_visible = self.mesh.as_ref().is_none_or(|m| m.visible);
        self.mesh = s.mesh.map(|mut m| {
            m.visible = mesh_visible;
            m
        });
        let visible: HashMap<u32, bool> = self.passes.iter().map(|p| (p.id, p.layer.visible)).collect();
        // Trackers whose discard flags differ from the restored ones are brought back in line.
        let mut resync = Vec::new();
        self.passes = s
            .passes
            .into_iter()
            .map(|p| {
                let mut layer = p.layer;
                layer.visible = visible.get(&p.id).copied().unwrap_or(true);
                let flags: Vec<(u64, bool)> = p.keyframes.iter().map(|k| (k.id, k.discarded)).collect();
                let differs = p
                    .tracker
                    .try_lock()
                    .map(|t| t.keyframes().iter().any(|k| flags.iter().any(|(id, d)| *id == k.id && *d != k.discarded)))
                    .unwrap_or(true);
                if differs {
                    resync.push((p.id, p.tracker.clone(), flags));
                }
                ScanPass { id: p.id, name: p.name, layer, frames: p.frames, tracker: p.tracker, keyframes: p.keyframes, meta: p.meta, tinted: None, mask: p.mask, surface: p.surface, edit: p.edit, rgb_keyframes: p.rgb_keyframes, markers: p.markers }
            })
            .collect();
        // The project changed (for unsaved-changes tracking), though no layer is new.
        let _ = self.generation();
        match self.selection {
            Some(Node::Pass(id)) | Some(Node::Frame(id, _)) if !self.passes.iter().any(|p| p.id == id) => self.selection = None,
            Some(Node::Merged) if self.merged.is_none() => self.selection = None,
            Some(Node::Mesh) if self.mesh.is_none() => self.selection = None,
            _ => {}
        }
        self.highlight = None;
        // Several passes may need their frames restored: one job, one pass after the other.
        if !resync.is_empty() {
            self.spawn_frames_job(resync);
        }
    }

    /// Re-applies discard flags to the trackers of restored passes and takes their rebuilt
    /// models (one job for all of them, so a jump over several steps never collides with a job
    /// still running).
    fn spawn_frames_job(&mut self, passes: Vec<(u32, SharedTracker, Vec<(u64, bool)>)>) {
        if self.busy.is_some() {
            return;
        }
        self.busy = Some("Restoring frames".to_string());
        let tx = self.job_tx.clone();
        self.rt.spawn_blocking(move || {
            // All rebuilt first, then reported: the first result ends the busy state.
            let results: Vec<JobResult> = passes
                .into_iter()
                .map(|(pass, tracker, flags)| match tracker.lock() {
                    Ok(mut t) => {
                        t.set_discarded_many(&flags);
                        JobResult::Rebuilt { pass, points: t.fused_points(), keyframes: t.keyframes().iter().map(KeyframeInfo::of).collect(), message: "Restored the frames".into() }
                    }
                    Err(_) => JobResult::Failed("The pass's tracker is unavailable".into()),
                })
                .collect();
            for result in results {
                let _ = tx.blocking_send(result);
            }
        });
    }

    pub fn can_edit(&self) -> bool {
        self.capture == Capture::Idle && self.busy.is_none()
    }

    /// Why editing the project is blocked right now (for a disabled button's tooltip).
    pub fn edit_blocked(&self) -> Option<&'static str> {
        if self.busy.is_some() {
            Some("Another operation is running")
        } else if self.capture != Capture::Idle {
            Some("Stop the capture first")
        } else {
            None
        }
    }

    /// Frames can be discarded while idle or paused (the worker is not registering then).
    pub fn can_edit_frames(&self) -> bool {
        matches!(self.capture, Capture::Idle | Capture::Paused) && self.busy.is_none()
    }

    /// Run `op` on a pass's tracker off the UI thread, then take its rebuilt model.
    fn spawn_tracker_job(&mut self, label: &str, pass: u32, tracker: SharedTracker, op: impl FnOnce(&mut scancraft_slam::Tracker) -> String + Send + 'static) {
        if self.busy.is_some() {
            return;
        }
        self.busy = Some(label.to_string());
        self.say(Tone::Normal, format!("{label}…"));
        let tx = self.job_tx.clone();
        self.rt.spawn_blocking(move || {
            let result = match tracker.lock() {
                Ok(mut t) => {
                    let message = op(&mut t);
                    JobResult::Rebuilt { pass, points: t.fused_points(), keyframes: t.keyframes().iter().map(KeyframeInfo::of).collect(), message }
                }
                Err(_) => JobResult::Failed("The pass's tracker is unavailable".into()),
            };
            let _ = tx.blocking_send(result);
        });
    }

    /// Discard (or restore) one keyframe of a pass and rebuild the pass from the kept ones.
    pub fn set_frame_discarded(&mut self, pass: u32, frame: u64, discarded: bool) {
        if !self.can_edit_frames() {
            return;
        }
        if self.passes.iter().any(|p| p.id == pass && p.is_cleaned()) {
            self.say(Tone::Warning, "This pass is cleaned: reset its cleaning before discarding or restoring frames");
            return;
        }
        let Some(tracker) = self.passes.iter().find(|p| p.id == pass).map(|p| p.tracker.clone()) else { return };
        let verb = if discarded { "Discarded" } else { "Restored" };
        self.record(self.snapshot(&format!("{} Frame {frame}", if discarded { "Discard" } else { "Restore" })));
        self.spawn_tracker_job(if discarded { "Discarding frame" } else { "Restoring frame" }, pass, tracker, move |t| {
            if t.set_discarded(frame, discarded) {
                format!("{verb} frame {frame}")
            } else {
                format!("Frame {frame} is no longer stored")
            }
        });
    }

    /// Discard a keyframe and every later one (the scan drifted from there on).
    pub fn discard_frames_from(&mut self, pass: u32, frame: u64) {
        if !self.can_edit_frames() {
            return;
        }
        let Some(p) = self.passes.iter().find(|p| p.id == pass) else { return };
        if p.is_cleaned() {
            self.say(Tone::Warning, "This pass is cleaned: reset its cleaning before discarding frames");
            return;
        }
        let tracker = p.tracker.clone();
        let n = p.keyframes.iter().filter(|k| k.id >= frame && !k.discarded).count();
        self.record(self.snapshot(&format!("Discard Frames from {frame}")));
        self.spawn_tracker_job("Discarding frames", pass, tracker, move |t| {
            t.discard_from(frame);
            format!("Discarded {n} frame{} from frame {frame} on", if n == 1 { "" } else { "s" })
        });
    }

    /// Keep the selected keyframe's overlay in step with the selection.
    pub fn update_highlight(&mut self, kept: [u8; 3], discarded: [u8; 3]) {
        let Some(Node::Frame(pass, frame)) = self.selection else {
            self.highlight = None;
            return;
        };
        let Some(p) = self.passes.iter().find(|p| p.id == pass) else { return };
        let is_discarded = p.keyframes.iter().any(|k| k.id == frame && k.discarded);
        if self.highlight.as_ref().is_some_and(|h| h.pass == pass && h.frame == frame && h.discarded == is_discarded) {
            return;
        }
        // A discarded frame is shown in its own colour: where it landed, though no longer used.
        let colour = if is_discarded { discarded } else { kept };
        // While the worker holds the tracker, try again next frame.
        let Ok(tracker) = p.tracker.try_lock() else { return };
        let Some(kf) = tracker.keyframes().iter().find(|k| k.id == frame) else {
            drop(tracker);
            self.highlight = None;
            return;
        };
        let colour = scancraft_core::ColorRgba::new(colour[0], colour[1], colour[2], 255);
        let points: Vec<_> = project::place(kf.world_points(), &p.meta.transform).into_iter().map(|p| p.with_color(colour)).collect();
        drop(tracker);
        let g = self.preview_generation();
        self.highlight = Some(Highlight { pass, frame, discarded: is_discarded, layer: CloudLayer::new(PointCloud::from_points(points), g) });
    }

    pub fn undo(&mut self) {
        self.jump_history(-1);
    }

    pub fn redo(&mut self) {
        self.jump_history(1);
    }

    /// Memory the undo history holds on its own (geometry the live project no longer uses).
    pub fn history_memory(&self) -> usize {
        self.history.held_bytes(&self.snapshot("").blocks())
    }

    /// Moves to the state after `done` actions of the timeline (0 = before the first one kept).
    pub fn jump_to_step(&mut self, done: usize) {
        let delta = done as isize - self.history.done_count() as isize;
        self.jump_history(delta);
    }

    /// Undoes (negative) or redoes (positive) `steps` actions in one go.
    fn jump_history(&mut self, steps: isize) {
        if steps == 0 || !self.can_edit() {
            return;
        }
        let steps_list = self.history.timeline();
        let done = self.history.done_count();
        let current = self.snapshot("");
        let Some(target) = self.history.jump(current, steps) else { return };
        self.restore(target);
        let message = if steps < 0 {
            let undone: Vec<&str> = steps_list[(done as isize + steps) as usize..done].iter().rev().map(|s| s.label.as_str()).collect();
            if undone.len() == 1 { format!("Undid {}", undone[0]) } else { format!("Undid {} steps (back to before {})", undone.len(), undone.last().copied().unwrap_or_default()) }
        } else {
            let redone: Vec<&str> = steps_list[done..done + steps as usize].iter().map(|s| s.label.as_str()).collect();
            if redone.len() == 1 { format!("Redid {}", redone[0]) } else { format!("Redid {} steps (up to {})", redone.len(), redone.last().copied().unwrap_or_default()) }
        };
        self.say(Tone::Normal, message);
    }

    pub fn delete_pass(&mut self, id: u32) {
        if !self.can_edit() {
            return;
        }
        let name = self.passes.iter().find(|p| p.id == id).map(|p| p.name.clone()).unwrap_or_default();
        self.record(self.snapshot(&format!("Delete {name}")));
        let was_reference = self.passes.iter().any(|p| p.id == id && p.meta.alignment == AlignState::Reference);
        self.passes.retain(|p| p.id != id);
        if self.align_preview.as_ref().is_some_and(|a| a.pass == id) {
            self.align_preview = None;
        }
        // A proposal for several passes is no longer complete.
        self.poses_preview = None;
        if was_reference {
            // The first aligned pass (else the first pass) takes over the project frame.
            let next = self.passes.iter().find(|p| p.meta.alignment.is_placed()).or(self.passes.first()).map(|p| p.id);
            if let Some(next) = next {
                if self.passes.iter().any(|p| p.id == next && p.meta.alignment.is_placed()) {
                    self.rebase_project(next);
                } else if let Some(p) = self.passes.iter_mut().find(|p| p.id == next) {
                    p.meta.alignment = AlignState::Reference;
                }
            }
        }
        if self.selection == Some(Node::Pass(id)) {
            self.selection = None;
        }
        self.say(Tone::Normal, format!("Deleted {name}"));
    }

    pub fn discard_mesh(&mut self) {
        if !self.can_edit() || self.mesh.is_none() {
            return;
        }
        self.record(self.snapshot("Delete Mesh"));
        self.mesh = None;
        if self.selection == Some(Node::Mesh) {
            self.selection = None;
        }
        self.say(Tone::Normal, "Deleted the mesh");
    }

    pub fn clear_session(&mut self) {
        if !self.can_edit() || !self.has_geometry() {
            return;
        }
        self.record(self.snapshot("Clear Session"));
        self.passes.clear();
        self.clear_previews();
        self.picking = None;
        self.object_picking = None;
        self.merged = None;
        self.mesh = None;
        self.selection = None;
        self.framed_radius = 0.0;
        self.say(Tone::Normal, "Cleared all scan data");
    }

    /// Why a tool cannot run now, or `None` when it can.
    pub fn tool_blocked(&self, tool: Tool) -> Option<&'static str> {
        if self.busy.is_some() {
            return Some("Another operation is running");
        }
        match tool {
            Tool::ConnectIp => {
                if self.connection.is_some() {
                    Some("Disconnect the current scanner first")
                } else if self.connecting {
                    Some("Connecting…")
                } else {
                    None
                }
            }
            _ if self.capture != Capture::Idle => Some("Stop the capture first"),
            Tool::RemovePlane | Tool::Outliers | Tool::Downsample | Tool::Crop => {
                if !self.has_geometry() {
                    Some("No points to work on: scan or import first")
                } else if self.clean_len() == 0 {
                    Some("Nothing shown: show the passes to clean")
                } else {
                    None
                }
            }
            Tool::Reconstruct | Tool::MakeSolid => (self.working_len() == 0).then_some("No points to work on: scan first"),
            Tool::IsolateObject => {
                if let Some(why) = self.isolate_blocked() {
                    Some(why)
                } else if self.object_picking.as_ref().is_some_and(|o| !o.current()) {
                    Some("Finding the object…")
                } else {
                    None
                }
            }
            Tool::AiSegment => self.ai_segment_blocked(),
            Tool::Smooth | Tool::Decimate => self.mesh.is_none().then_some("Reconstruct a surface first"),
            Tool::FillHoles => match &self.mesh {
                None => Some("Reconstruct a surface first"),
                Some(m) if m.analysis.open_edges == 0 => Some("The mesh has no holes"),
                Some(_) => None,
            },
            Tool::PointPairs(id) => self.pairs_blocked(id),
            Tool::Datums => self.datums_blocked(),
            Tool::Scale(id) => self.scale_blocked(Some(id)),
            Tool::SaveProject => (!self.has_geometry()).then_some("Nothing to save yet"),
            Tool::Export(f) => {
                if f.needs_mesh() {
                    self.mesh.is_none().then_some("Reconstruct a surface first")
                } else {
                    (self.working_len() == 0).then_some("No points to export")
                }
            }
        }
    }

    pub fn open_tool(&mut self, tool: Tool) {
        if let Tool::Export(format) = tool {
            self.set_export_format(format);
        }
        if tool == Tool::Crop {
            if let Some(b) = self.data_bbox() {
                self.settings.crop_min = b.min.to_array();
                self.settings.crop_max = b.max.to_array();
            }
        }
        if let Tool::PointPairs(id) = tool {
            if !self.start_picking(id) {
                return;
            }
        }
        if tool == Tool::IsolateObject && !self.start_object_picking() {
            return;
        }
        if tool == Tool::AiSegment && !self.start_ai_picking() {
            return;
        }
        if tool == Tool::Datums && !self.start_datum_picking() {
            return;
        }
        if let Tool::Scale(id) = tool {
            if !self.start_scale_picking(id) {
                return;
            }
        }
        if tool != Tool::IsolateObject && !tool.is_file() {
            self.object_picking = None;
        }
        if tool != Tool::AiSegment && !tool.is_file() {
            self.ai_picking = None;
        }
        if tool != Tool::Datums && !tool.is_file() {
            self.datum_picking = None;
        }
        if !matches!(tool, Tool::Scale(_)) && !tool.is_file() {
            self.scale_picking = None;
        }
        if tool == Tool::SaveProject {
            if let Some(path) = &self.project.path {
                self.settings.project_path = path.display().to_string();
            }
        }
        self.tool = Some(tool);
        if !tool.is_file() {
            self.stage = tool.stage();
        }
    }

    /// Closes the open tool's card and whatever it was showing.
    pub fn close_tool(&mut self) {
        // Cancelling Save Project also cancels the close it was opened for.
        if self.tool == Some(Tool::SaveProject) {
            self.pending_close = false;
        }
        self.tool = None;
        self.picking = None;
        self.object_picking = None;
        self.ai_picking = None;
        self.datum_picking = None;
        self.scale_picking = None;
    }

    pub fn set_export_format(&mut self, format: ExportFormat) {
        self.settings.export_format = format;
        let path = std::path::Path::new(&self.settings.export_path);
        let stem = path.with_extension("");
        self.settings.export_path = format!("{}.{}", stem.display(), format.extension());
    }

    fn spawn_job(&mut self, label: &str, job: impl FnOnce() -> JobResult + Send + 'static) {
        if self.busy.is_some() {
            return;
        }
        self.busy = Some(label.to_string());
        self.say(Tone::Normal, format!("{label}…"));
        let tx = self.job_tx.clone();
        self.rt.spawn_blocking(move || {
            let _ = tx.blocking_send(job());
        });
    }

    /// [`spawn_job`](Self::spawn_job) for a job that changes geometry: the state before it goes
    /// on the undo stack once the job succeeds, and is dropped if it fails or changes nothing.
    fn spawn_undoable(&mut self, label: &str, undo_label: &str, job: impl FnOnce() -> JobResult + Send + 'static) {
        if self.busy.is_some() {
            return;
        }
        self.pending_undo = Some(self.snapshot(undo_label));
        self.spawn_job(label, job);
    }

    /// Whether a job result changed geometry (its undo snapshot is kept).
    fn changes_geometry(result: &JobResult) -> bool {
        matches!(result, JobResult::Cleaned { .. } | JobResult::Mesh { .. } | JobResult::Merged { .. } | JobResult::Imported(_) | JobResult::AddedProject { .. })
    }

    /// Runs a Clean filter on each shown pass and on the shown merged model, each on its own
    /// (undoable). `filter` gets the points in the project frame (so a crop box means the same for
    /// every pass) and returns `None` when it finds nothing to do. `noun` names what is removed in
    /// the message ("plane points removed").
    fn spawn_clean(&mut self, label: &str, undo_label: &str, noun: &'static str, filter: impl Fn(&PointCloud) -> Option<PointCloud> + Send + Sync + 'static) {
        let targets = self.clean_targets();
        if targets.is_empty() {
            self.say(Tone::Warning, "Nothing shown to clean: show the passes to work on");
            return;
        }
        // (target, name, surface to clean in its own frame, project <- own frame)
        enum Input {
            Pass(u32, String, PassSource, Transform),
            Merged(PointCloud),
        }
        let inputs: Vec<Input> = targets
            .iter()
            .filter_map(|t| match *t {
                CleanTarget::Pass(id) => self.passes.iter().find(|p| p.id == id).map(|p| Input::Pass(id, p.name.clone(), p.source(), p.meta.transform)),
                CleanTarget::Merged => self.merged.as_ref().map(|m| Input::Merged((*m.layer.cloud).clone())),
            })
            .collect();
        self.spawn_undoable(label, undo_label, move || {
            use rayon::prelude::*;
            // (name, before, after or None when unchanged, result in the target's own frame)
            let results: Vec<Result<(CleanTarget, String, usize, Option<PointCloud>), String>> = inputs
                .into_par_iter()
                .map(|input| match input {
                    Input::Pass(id, name, source, t) => {
                        let own = source.editable()?;
                        let placed = PointCloud::from_points(project::place(own.points, &t));
                        let before = placed.len();
                        let out = filter(&placed).filter(|c| c.len() != before).map(|c| PointCloud::from_points(project::place(c.points, &t.inverse())));
                        Ok((CleanTarget::Pass(id), name, before, out))
                    }
                    Input::Merged(cloud) => {
                        let before = cloud.len();
                        Ok((CleanTarget::Merged, "Merged model".to_string(), before, filter(&cloud).filter(|c| c.len() != before)))
                    }
                })
                .collect();
            let mut passes = Vec::new();
            let mut merged = None;
            let (mut changed, mut unchanged) = (Vec::new(), Vec::new());
            let (mut before_total, mut after_total) = (0usize, 0usize);
            for r in results {
                let (target, name, before, out) = match r {
                    Ok(r) => r,
                    Err(e) => return JobResult::Failed(e),
                };
                match out {
                    Some(cloud) => {
                        before_total += before;
                        after_total += cloud.len();
                        changed.push(name);
                        match target {
                            CleanTarget::Pass(id) => passes.push((id, cloud)),
                            CleanTarget::Merged => merged = Some(cloud),
                        }
                    }
                    None => unchanged.push(name),
                }
            }
            if changed.is_empty() {
                return JobResult::NoChange(format!("Nothing {noun} in {}", unchanged.join(", ")));
            }
            let mut message = if after_total <= before_total {
                format!("{}: {} {noun} points removed ({} → {})", changed.join(", "), thousands(before_total - after_total), thousands(before_total), thousands(after_total))
            } else {
                format!("{}: {} → {} points", changed.join(", "), thousands(before_total), thousands(after_total))
            };
            if !unchanged.is_empty() {
                message.push_str(&format!(" · nothing {noun} in {}", unchanged.join(", ")));
            }
            JobResult::Cleaned { passes, merged, message }
        });
    }

    /// Puts Clean results in place: each pass gets its cleaned surface, the merged model its new
    /// cloud. Shown and hidden stay as they were.
    ///
    /// Results made from the old surfaces go (the caller has pushed the undo snapshot that still
    /// holds them): the merged model when a pass it was made from changed, and the mesh in any
    /// case (it was made from the passes or from the merged model before this cleaning).
    fn apply_cleaned(&mut self, passes: Vec<(u32, PointCloud)>, merged: Option<PointCloud>) {
        let changed: Vec<u32> = passes.iter().map(|(id, _)| *id).collect();
        self.invalidate_after_cleaning(&changed, "Cleaning changed the scans");
        for (id, own) in passes {
            let g = self.generation();
            let Some(p) = self.passes.iter_mut().find(|p| p.id == id) else { continue };
            let visible = p.layer.visible;
            p.layer = CloudLayer::new(PointCloud::from_points(project::place(own.points.clone(), &p.meta.transform)), g);
            p.layer.visible = visible;
            p.edit = Some(Arc::new(own));
            // Object isolation and the tinted copy were made from the old surface.
            p.mask = isolation::new_cache();
            p.tinted = None;
        }
        if let Some(cloud) = merged {
            let g = self.generation();
            if let Some(m) = self.merged.take() {
                let visible = m.layer.visible;
                let mut layer = MergedLayer::new(cloud, g, m.passes, m.summary);
                layer.layer.visible = visible;
                self.merged = Some(layer);
            }
        }
    }

    /// Finds the mesh's holes if the mesh changed, and colours their outlines for the current
    /// Fill Holes settings (accent: filled, grey: left open).
    pub fn refresh_holes(&mut self) {
        let Some(mesh) = &self.mesh else {
            self.holes = None;
            return;
        };
        if self.holes.as_ref().is_none_or(|h| h.mesh_generation != mesh.generation) {
            self.holes = Some(HolePreview { mesh_generation: mesh.generation, holes: scancraft_mesh::HoleFiller::find_holes(&mesh.mesh), outline: None });
        }
        let key = (self.settings.fill_max_mm, self.settings.fill_keep_largest);
        let stale = self.holes.as_ref().is_some_and(|h| h.outline.as_ref().is_none_or(|(k, _)| *k != key));
        if !stale {
            return;
        }
        let Some(h) = self.holes.as_ref() else { return };
        let fill = scancraft_core::ColorRgba::new(255, 140, 40, 255);
        let open = scancraft_core::ColorRgba::new(140, 145, 155, 255);
        let mut points = Vec::new();
        for (i, hole) in h.holes.iter().enumerate() {
            let colour = if h.fills(i, key.0, key.1) { fill } else { open };
            let n = hole.vertices.len();
            for k in 0..n {
                let a = glam::Vec3::from(mesh.mesh.vertices[hole.vertices[k] as usize].position);
                let b = glam::Vec3::from(mesh.mesh.vertices[hole.vertices[(k + 1) % n] as usize].position);
                // Dense enough along each edge to read as a line.
                let steps = ((a.distance(b) / 0.15).ceil() as usize).clamp(1, 64);
                for s in 0..steps {
                    let p = a.lerp(b, s as f32 / steps as f32);
                    points.push(scancraft_core::Point3D::new(p.to_array()).with_color(colour));
                }
            }
        }
        let g = self.preview_generation();
        if let Some(h) = self.holes.as_mut() {
            h.outline = Some((key, CloudLayer::new(PointCloud::from_points(points), g)));
        }
    }

    /// Drops what was made from the passes in `changed` before they changed: the merged model if
    /// it contains one of them, and the mesh.
    fn invalidate_after_cleaning(&mut self, changed: &[u32], reason: &str) {
        let merged_stale = self.merged.as_ref().is_some_and(|m| changed.iter().any(|id| m.passes.contains(id)));
        self.invalidate_results(reason, merged_stale);
    }

    /// Whether any shown pass has been cleaned (Reset Cleaning has something to do).
    pub fn can_reset_cleaning(&self) -> bool {
        self.passes.iter().any(|p| p.layer.visible && p.is_cleaned())
    }

    /// Takes the shown passes back to their surfaces as scanned or imported (undoable).
    pub fn reset_cleaning(&mut self) {
        let ids: Vec<u32> = self.passes.iter().filter(|p| p.layer.visible && p.is_cleaned()).map(|p| p.id).collect();
        self.reset_cleaning_of(&ids);
    }

    /// Takes the given passes back to their surfaces as scanned or imported (undoable).
    pub fn reset_cleaning_of(&mut self, ids: &[u32]) {
        if !self.can_edit() || !self.passes.iter().any(|p| ids.contains(&p.id) && p.is_cleaned()) {
            return;
        }
        self.record(self.snapshot("Reset Cleaning"));
        let mut names = Vec::new();
        for i in 0..self.passes.len() {
            if !(ids.contains(&self.passes[i].id) && self.passes[i].is_cleaned()) {
                continue;
            }
            self.passes[i].edit = None;
            let own = match self.passes[i].source() {
                PassSource::Fixed(s) => s.points.clone(),
                PassSource::Tracked(t) => match t.lock() {
                    Ok(t) => t.fused_points(),
                    Err(_) => continue,
                },
                PassSource::Edited { .. } => continue,
            };
            let g = self.generation();
            let p = &mut self.passes[i];
            let visible = p.layer.visible;
            p.layer = CloudLayer::new(PointCloud::from_points(project::place(own, &p.meta.transform)), g);
            p.layer.visible = visible;
            p.mask = isolation::new_cache();
            p.tinted = None;
            names.push(p.name.clone());
        }
        let reset: Vec<u32> = self.passes.iter().filter(|p| ids.contains(&p.id) && names.contains(&p.name)).map(|p| p.id).collect();
        self.invalidate_after_cleaning(&reset, "Reset Cleaning changed the scans");
        self.say(Tone::Normal, format!("Reset the cleaning of {}", names.join(", ")));
    }

    // ------------------------------------------------------------------ isolate object

    /// Opens the Isolate Object tool on the working cloud and finds the object without picks.
    fn start_object_picking(&mut self) -> bool {
        if self.object_picking.is_some() {
            return true;
        }
        if self.busy.is_some() || self.isolate_blocked().is_some() {
            return false;
        }
        let Some(&target) = self.clean_targets().first() else { return false };
        // A pass is isolated from its own surface (with observation counts), placed.
        let input = match target {
            CleanTarget::Pass(id) => {
                let Some(p) = self.passes.iter().find(|p| p.id == id) else { return false };
                match p.source().editable() {
                    Ok(own) => PointCloud::from_points(project::place(own.points, &p.meta.transform)),
                    Err(e) => {
                        self.say(Tone::Error, e);
                        return false;
                    }
                }
            }
            CleanTarget::Merged => match &self.merged {
                Some(m) => (*m.layer.cloud).clone(),
                None => return false,
            },
        };
        if input.is_empty() {
            return false;
        }
        let g = self.preview_generation();
        self.object_picking = Some(ObjectPicking::new(input, g, target));
        self.segment_picks();
        true
    }

    /// Why Isolate Object cannot start: it works on one scan (or the merged model) at a time.
    pub fn isolate_blocked(&self) -> Option<&'static str> {
        if self.object_picking.is_some() {
            return None;
        }
        match self.clean_targets().len() {
            0 => Some("Nothing shown: show the pass to isolate"),
            1 => None,
            _ => Some("Show only the pass to isolate (hide the others)"),
        }
    }

    /// Adds a pick (keep or exclude) at `point` and segments again.
    pub fn add_object_pick(&mut self, point: glam::Vec3, keep: bool) {
        let Some(o) = self.object_picking.as_mut() else { return };
        o.picks.push((point, keep));
        self.segment_picks();
    }

    /// Takes back the newest pick.
    pub fn undo_object_pick(&mut self) {
        let Some(o) = self.object_picking.as_mut() else { return };
        if o.picks.pop().is_some() {
            self.segment_picks();
        }
    }

    // ------------------------------------------------------------------ AI segment & AI Pack

    pub fn refresh_ai_status(&mut self) {
        self.ai_status = crate::ai_pack::inspect_pack(Some(&self.ai_engine));
    }

    pub fn download_ai_pack(&mut self) {
        if self.busy.is_some() {
            return;
        }
        self.ai_engine.unload();
        self.spawn_job("Downloading AI Pack", || match crate::ai_pack::download_pack() {
            Ok(status) => JobResult::AiPackUpdated {
                ok: status.ready,
                message: if status.ready {
                    format!("AI Pack verified and ready ({:.0} MB)", status.total_mb())
                } else {
                    "Downloaded files did not complete AI Pack verification".to_string()
                },
                status,
            },
            Err(e) => JobResult::Failed(e),
        });
    }

    pub fn install_ai_pack_from(&mut self, folder: std::path::PathBuf) {
        if self.busy.is_some() {
            return;
        }
        self.ai_engine.unload();
        self.spawn_job("Installing AI Pack", move || match crate::ai_pack::install_from_folder(&folder) {
            Ok(status) => JobResult::AiPackUpdated {
                ok: status.ready,
                message: if status.ready {
                    format!("Installed & verified AI Pack ({:.0} MB)", status.total_mb())
                } else {
                    "Copied matching files; still waiting for remaining AI Pack files".to_string()
                },
                status,
            },
            Err(e) => JobResult::Failed(e),
        });
    }

    pub fn remove_ai_pack(&mut self) {
        if self.busy.is_some() {
            return;
        }
        let engine = Arc::clone(&self.ai_engine);
        self.spawn_job("Removing AI Pack", move || match crate::ai_pack::remove_pack(&engine) {
            Ok(status) => JobResult::AiPackUpdated {
                ok: true,
                message: "Removed downloaded AI Pack files".to_string(),
                status,
            },
            Err(e) => JobResult::Failed(e),
        });
    }

    pub fn sync_live_ai_setting(&mut self) {
        self.live_ai.set_enabled(self.settings.live_ai_segment);
        if !self.settings.live_ai_segment {
            self.live_ai_status = None;
            self.ai_mask_texture = None;
        }
    }

    pub fn trigger_live_ai_prompt(&mut self, nx: f32, ny: f32) {
        self.live_ai.set_prompt_normalized(nx, ny);
    }

    pub fn ai_segment_blocked(&self) -> Option<&'static str> {
        if self.ai_picking.is_some() {
            return None;
        }
        if !self.ai_status.ready {
            return Some("Install the AI Pack first (AI Pack in top bar)");
        }
        let targets = self.clean_targets();
        let target = match targets.as_slice() {
            [] => return Some("Nothing shown: show a pass to segment"),
            [CleanTarget::Merged] => return Some("AI Segment needs a scan pass with RGB keyframes (hide Merged model)"),
            [CleanTarget::Pass(id)] => *id,
            _ => return Some("Show only the pass to segment (hide the others)"),
        };
        let Some(pass) = self.passes.iter().find(|p| p.id == target) else {
            return Some("Pass not found");
        };
        if pass.rgb_keyframes.is_empty() {
            return Some("Selected pass has no recorded RGB keyframes (enable colour scanner)");
        }
        None
    }

    pub fn start_ai_picking(&mut self) -> bool {
        if self.ai_picking.is_some() {
            return true;
        }
        if let Some(why) = self.ai_segment_blocked() {
            if !self.ai_status.ready {
                self.ai_pack_modal = true;
            }
            self.say(Tone::Warning, why);
            return false;
        }
        let Some(&CleanTarget::Pass(id)) = self.clean_targets().first() else { return false };
        let Some(p) = self.passes.iter().find(|p| p.id == id) else { return false };
        let input = match p.source().editable() {
            Ok(c) => Arc::new(c),
            Err(e) => {
                self.say(Tone::Error, e);
                return false;
            }
        };
        match crate::ai_segment::AiPicking::from_pass(p.id, p.name.clone(), input, p.meta.transform, &p.rgb_keyframes) {
            Ok(picking) => {
                self.ai_picking = Some(picking);
                self.say(Tone::Normal, "Click the object on the keyframe, then run Propagate all");
                true
            }
            Err(e) => {
                self.say(Tone::Error, e);
                false
            }
        }
    }

    pub fn select_ai_keyframe(&mut self, index: usize) {
        if let Some(ai) = self.ai_picking.as_mut() {
            ai.selected = index.min(ai.keyframes.len().saturating_sub(1));
            ai.preview_texture = None;
        }
    }

    pub fn add_ai_pick(&mut self, pick: crate::ai_segment::ImagePick) {
        if self.busy.is_some() {
            return;
        }
        let Some(ai) = self.ai_picking.as_mut() else { return };
        let idx = ai.selected;
        ai.prompts[idx].push(pick);
        ai.labels = None;
        ai.preview = None;
        ai.masked_displays[idx] = None;
        ai.texture_rev += 1;
        ai.preview_texture = None;
        self.evaluate_selected_ai_prompt();
    }

    pub fn undo_ai_pick(&mut self) {
        if self.busy.is_some() {
            return;
        }
        let Some(ai) = self.ai_picking.as_mut() else { return };
        let idx = ai.selected;
        if ai.prompts[idx].pop().is_some() {
            ai.labels = None;
            ai.preview = None;
            ai.masks[idx] = None;
            ai.masked_displays[idx] = None;
            ai.texture_rev += 1;
            ai.preview_texture = None;
            if !ai.prompts[idx].is_empty() {
                self.evaluate_selected_ai_prompt();
            }
        }
    }

    fn evaluate_selected_ai_prompt(&mut self) {
        let Some(ai) = self.ai_picking.as_ref() else { return };
        let idx = ai.selected;
        let Some(prompt) = ai.build_prompt_for(idx) else { return };
        let pass = ai.pass;
        let kf_id = ai.keyframes[idx].kf.id;
        let rgb = ai.keyframes[idx].rgb.clone();
        let picks = ai.prompts[idx].clone();
        let engine = Arc::clone(&self.ai_engine);
        self.spawn_job("Running SAM 2.1 prompt", move || {
            match crate::ai_segment::run_single_keyframe(&engine, &rgb, &prompt, &picks, kf_id) {
                Ok((mask, overlay)) => JobResult::AiPrompted { pass, keyframe_index: idx, mask, overlay },
                Err(e) => JobResult::Failed(e),
            }
        });
    }

    pub fn propagate_ai_segment(&mut self) {
        if self.busy.is_some() {
            return;
        }
        let Some(ai) = self.ai_picking.as_ref() else { return };
        if !ai.any_prompts() {
            self.say(Tone::Warning, "Click on at least one keyframe to mark the object first");
            return;
        }
        let pass = ai.pass;
        let keyframes: Vec<(scancraft_color::RgbKeyframe, scancraft_core::RgbImage)> =
            ai.keyframes.iter().map(|k| (k.kf.clone(), k.rgb.clone())).collect();
        let prompts: Vec<Option<scancraft_segment::FramePrompt>> =
            (0..keyframes.len()).map(|i| ai.build_prompt_for(i)).collect();
        let pass_points = ai.input.points.clone();
        let engine = Arc::clone(&self.ai_engine);
        self.spawn_job("Segmenting keyframes with SAM 2.1", move || {
            match crate::ai_segment::propagate_and_project_3d(&engine, &keyframes, &prompts, &pass_points) {
                Ok((masks, labels, provider)) => {
                    let kept = labels.iter().filter(|l| **l == scancraft_segment::PointLabel::Object).count();
                    JobResult::AiPropagated {
                        pass,
                        masks,
                        labels,
                        message: format!(
                            "SAM 2.1 ({provider}): segmented {kept} of {} points across {} keyframes",
                            thousands(pass_points.len()),
                            keyframes.len()
                        ),
                    }
                }
                Err(e) => JobResult::Failed(e),
            }
        });
    }

    fn commit_ai_segment(&mut self) {
        let Some((pass, kept, total)) = self
            .ai_picking
            .as_ref()
            .and_then(|ai| ai.filter_pass_cloud().map(|c| (ai.pass, c, ai.input.len())))
        else {
            self.say(Tone::Warning, "Click a keyframe and run Propagate all first");
            return;
        };
        if kept.is_empty() {
            self.say(Tone::Warning, "No object points remained; adjust the clicks and propagate again");
            return;
        }
        self.history.push(self.snapshot("AI segment object"));
        let msg = format!("AI Segment: kept {} of {} points", thousands(kept.len()), thousands(total));
        self.apply_cleaned(vec![(pass, kept)], None);
        self.say(Tone::Success, msg);
        self.ai_picking = None;
        self.tool = None;
    }

    pub fn remove_object_pick(&mut self, index: usize) {
        let Some(o) = self.object_picking.as_mut() else { return };
        if index < o.picks.len() {
            o.picks.remove(index);
            self.segment_picks();
        }
    }

    /// Segments the tool's cloud for its current picks, unless a job is running (the result of
    /// that job starts the next run).
    fn segment_picks(&mut self) {
        let Some(o) = self.object_picking.as_ref() else { return };
        if o.current() || self.busy.is_some() {
            return;
        }
        let (input, seeds, picks) = (Arc::clone(&o.input), o.seeds(), o.picks.clone());
        self.spawn_job("Finding the object", move || {
            let segmentation = Arc::new(isolation::segment_with_seeds(&input, &seeds));
            JobResult::Isolated { segmentation, picks }
        });
    }

    /// The Isolate Object tool's OK: its object replaces the pass's (or merged model's) points
    /// (undoable).
    fn commit_object(&mut self) {
        let Some(object) = self.object_picking.as_ref().filter(|o| o.current()).and_then(|o| o.object()) else {
            self.say(Tone::Warning, "No object found: click a cluster of points to keep");
            return;
        };
        let Some((before, target)) = self.object_picking.as_ref().map(|o| (o.input.len(), o.target)) else { return };
        self.record(self.snapshot("Isolate Object"));
        let message = format!("{}: kept the object, {} of {} points", self.target_name(target), thousands(object.len()), thousands(before));
        match target {
            CleanTarget::Pass(id) => {
                let Some(t) = self.passes.iter().find(|p| p.id == id).map(|p| p.meta.transform) else { return };
                let own = PointCloud::from_points(project::place(object.points, &t.inverse()));
                self.apply_cleaned(vec![(id, own)], None);
            }
            CleanTarget::Merged => self.apply_cleaned(Vec::new(), Some(object)),
        }
        self.say(Tone::Success, message);
        self.object_picking = None;
        self.tool = None;
    }

    /// Run the open tool (its OK button).
    pub fn run_tool(&mut self, tool: Tool) {
        if self.tool_blocked(tool).is_some() {
            return;
        }
        let s = &self.settings;
        match tool {
            Tool::ConnectIp => {
                self.connect_ip();
                return;
            }
            Tool::PointPairs(id) => {
                self.align_point_pairs(id);
                return;
            }
            Tool::Datums => {
                self.apply_datums();
                return;
            }
            Tool::Scale(id) => {
                self.apply_scale(id);
                return;
            }
            Tool::SaveProject => {
                let path = trim_quotes(&self.settings.project_path);
                self.save_project(std::path::PathBuf::from(path));
                return;
            }
            Tool::IsolateObject => {
                self.commit_object();
                return;
            }
            Tool::AiSegment => {
                self.commit_ai_segment();
                return;
            }
            Tool::RemovePlane => {
                let tolerance = s.plane_tolerance_mm;
                self.spawn_clean("Removing the dominant plane", "Remove Plane", "plane", move |c| {
                    let (cleaned, plane) = RansacPlaneFilter::new(tolerance, 150, PlaneFilterMode::ClipBelow).filter(c);
                    plane.map(|_| cleaned)
                });
            }
            Tool::Outliers => {
                let (k, sd) = (s.sor_neighbors, s.sor_std_dev);
                self.spawn_clean("Removing outliers", "Remove Outliers", "outlier", move |c| Some(StatisticalOutlierFilter::new(k, sd).filter(c)));
            }
            Tool::Downsample => {
                let voxel = s.voxel_mm;
                self.spawn_clean("Downsampling", "Downsample", "merged", move |c| Some(VoxelFilter::new(voxel).filter(c)));
            }
            Tool::Crop => {
                let bbox = BoundingBox::from_arrays(s.crop_min, s.crop_max);
                self.spawn_clean("Cropping", "Crop", "cropped", move |c| Some(RoiCropFilter::new(bbox).filter(c)));
            }
            Tool::Reconstruct => {
                let input = self.working_cloud();
                let config = scancraft_mesh::SurfaceConfig { cell_mm: s.mc_cell_mm, detail: s.mesh_smoothness, ..Default::default() };
                let smoother = (s.mesh_auto_smooth > 0).then(|| smoother(s.mesh_auto_smooth, s.smooth_keep_edges));
                self.spawn_undoable("Reconstructing the surface", "Reconstruct Surface", move || {
                    let (mut mesh, stats) = Mesher::reconstruct_with_stats(&input, &config);
                    if mesh.triangles.is_empty() {
                        return JobResult::Failed("Reconstruction produced no surface; try a larger cell size or more smoothness".into());
                    }
                    if let Some(sm) = smoother {
                        sm.smooth(&mut mesh);
                    }
                    let analysis = Mesher::analyze(&mesh);
                    let holes = scancraft_mesh::HoleFiller::find_holes(&mesh).len();
                    let message = format!(
                        "Reconstructed {} triangles (point spacing {:.2} mm, smoothing radius {:.1} mm){}",
                        thousands(analysis.triangle_count),
                        stats.point_spacing_mm,
                        stats.support_radius_mm,
                        if holes > 0 { format!(" · {holes} openings: Fill Holes closes them") } else { String::new() }
                    );
                    JobResult::Mesh { message, mesh, analysis }
                });
            }
            Tool::Smooth => {
                let Some(mut mesh) = self.mesh.as_ref().map(|m| (*m.mesh).clone()) else { return };
                let sm = smoother(s.smooth_iterations, s.smooth_keep_edges);
                let it = s.smooth_iterations;
                self.spawn_undoable("Smoothing", "Smooth", move || {
                    sm.smooth(&mut mesh);
                    let analysis = Mesher::analyze(&mesh);
                    JobResult::Mesh { mesh, analysis, message: format!("Smoothed (strength {it})") }
                });
            }
            Tool::FillHoles => {
                let Some(mut mesh) = self.mesh.as_ref().map(|m| (*m.mesh).clone()) else { return };
                let filler = scancraft_mesh::HoleFiller::new(s.fill_max_mm, s.fill_keep_largest);
                self.spawn_undoable("Filling holes", "Fill Holes", move || {
                    let report = filler.fill(&mut mesh);
                    if report.filled == 0 {
                        return JobResult::NoChange("No hole small enough to fill: raise the maximum size".into());
                    }
                    mesh.recompute_normals();
                    let analysis = Mesher::analyze(&mesh);
                    let left = scancraft_mesh::HoleFiller::find_holes(&mesh).len();
                    let message = format!(
                        "Filled {} hole{} (+{} triangles){}",
                        report.filled,
                        if report.filled == 1 { "" } else { "s" },
                        thousands(report.added_triangles),
                        match (left, analysis.is_watertight) {
                            (_, true) => " · the mesh is watertight".to_string(),
                            (0, false) => String::new(),
                            (n, false) => format!(" · {n} opening{} left", if n == 1 { "" } else { "s" }),
                        }
                    );
                    JobResult::Mesh { mesh, analysis, message }
                });
            }
            Tool::MakeSolid => {
                let input = self.solid_input();
                let job = SolidJob::from_settings(s);
                self.spawn_undoable("Making the solid", "Make Solid", move || match make_solid(input, job) {
                    Ok(solid) => {
                        let analysis = Mesher::analyze(&solid.mesh);
                        let message = solid_message(&solid.report, &analysis);
                        JobResult::Mesh { mesh: solid.mesh, analysis, message }
                    }
                    Err(e) => JobResult::Failed(e),
                });
            }
            Tool::Decimate => {
                let Some(mesh) = self.mesh.as_ref().map(|m| (*m.mesh).clone()) else { return };
                let ratio = s.decimate_ratio;
                self.spawn_undoable("Decimating", "Decimate", move || {
                    let before = mesh.triangles.len();
                    let out = Mesher::decimate(&mesh, ratio);
                    let analysis = Mesher::analyze(&out);
                    JobResult::Mesh { message: format!("Decimated {} → {} triangles", thousands(before), thousands(out.triangles.len())), mesh: out, analysis }
                });
            }
            Tool::Export(format) => {
                let path = s.export_path.trim().to_string();
                if path.is_empty() {
                    self.say(Tone::Error, "Enter a file name");
                    return;
                }
                let cloud = (!format.needs_mesh()).then(|| self.working_cloud());
                let mesh = self.mesh.as_ref().map(|m| (*m.mesh).clone());
                self.spawn_job("Exporting", move || {
                    let result = match (format, &cloud, &mesh) {
                        (ExportFormat::PlyCloud, Some(c), _) => scancraft_io::save_ply_binary(c, &path).map(|_| format!("Exported {} points to {path}", thousands(c.len()))),
                        (ExportFormat::PlyMesh, _, Some(m)) => scancraft_io::save_mesh_ply_binary(m, &path).map(|_| format!("Exported mesh to {path}")),
                        (ExportFormat::ObjMesh, _, Some(m)) => scancraft_io::save_mesh_obj(m, &path).map(|_| format!("Exported mesh to {path}")),
                        (ExportFormat::StlMesh, _, Some(m)) => scancraft_io::save_mesh_stl_binary(m, &path).map(|_| format!("Exported mesh to {path}")),
                        _ => return JobResult::Failed("Nothing to export".into()),
                    };
                    match result {
                        Ok(msg) => JobResult::Exported(msg),
                        Err(e) => JobResult::Failed(format!("Export failed: {e}")),
                    }
                });
                return;
            }
        }
        self.tool = None;
    }

    /// Adds dropped scan files (as imported passes) and project folders (their passes), one
    /// after another.
    pub fn queue_imports(&mut self, paths: Vec<std::path::PathBuf>) {
        self.pending_imports.extend(paths);
        self.next_import();
    }

    /// Opens a native file / folder dialog; its answer is handled by `poll_picker`.
    pub fn pick(&mut self, ctx: &egui::Context, purpose: crate::file_picker::PickPurpose) {
        use crate::file_picker::PickPurpose as P;
        let hint = match purpose {
            P::ImportScans => self.settings.import_path.clone(),
            P::AddProjects => self.settings.add_project_path.clone(),
            P::OpenProject | P::SaveFolder => self.settings.project_path.clone(),
            P::AiPackFolder => crate::ai_pack::effective_models_dir().parent().map(|d| d.display().to_string()).unwrap_or_default(),
            P::ExportFile => {
                let path = std::path::Path::new(self.settings.export_path.trim());
                if path.is_absolute() {
                    path.display().to_string()
                } else {
                    std::env::current_dir().map(|d| d.join(path).display().to_string()).unwrap_or_default()
                }
            }
        };
        self.picker.open(ctx, purpose, &hint);
    }

    /// Acts on a dialog that has just closed: imports, opens, or fills in the chosen path.
    pub fn poll_picker(&mut self) {
        use crate::file_picker::PickPurpose as P;
        let Some(picked) = self.picker.poll() else { return };
        let Some(first) = picked.paths.first().cloned() else { return };
        match picked.purpose {
            P::ImportScans => {
                self.settings.import_path = first.display().to_string();
                self.queue_imports(picked.paths);
            }
            P::AddProjects => {
                self.settings.add_project_path = first.display().to_string();
                let (projects, others): (Vec<_>, Vec<_>) = picked.paths.into_iter().partition(|p| scancraft_io::project::is_project(p));
                if let Some(p) = others.first() {
                    self.say(Tone::Warning, format!("{} is not a ScanCraft project (no .scancraft file)", p.display()));
                }
                self.queue_imports(projects);
            }
            P::OpenProject => {
                if scancraft_io::project::is_project(&first) {
                    self.settings.project_path = first.display().to_string();
                    if self.can_edit() {
                        self.open_project(first);
                    } else {
                        // The dialog has no parent window, so the app may have started something
                        // meanwhile: open as soon as it is free instead of dropping the choice.
                        let name = first.file_name().map(|n| n.to_string_lossy().to_string()).unwrap_or_default();
                        self.pending_open = Some(first);
                        self.say(Tone::Normal, format!("{name} opens when the running operation has finished"));
                    }
                } else {
                    self.say(Tone::Warning, format!("{} is not a ScanCraft project (no .scancraft file)", first.display()));
                }
            }
            P::SaveFolder => self.settings.project_path = first.display().to_string(),
            P::AiPackFolder => self.install_ai_pack_from(first),
            P::ExportFile => {
                // The format follows the chosen extension when it names one we write.
                let ext = first.extension().map(|e| e.to_string_lossy().to_ascii_lowercase());
                let current = self.settings.export_format;
                if ext.as_deref() != Some(current.extension()) {
                    if let Some(f) = ExportFormat::ALL.iter().find(|f| Some(f.extension()) == ext.as_deref() && f.needs_mesh() == current.needs_mesh()).or_else(|| ExportFormat::ALL.iter().find(|f| Some(f.extension()) == ext.as_deref())) {
                        self.set_export_format(*f);
                        if self.tool.is_some_and(|t| matches!(t, Tool::Export(_))) {
                            self.tool = Some(Tool::Export(*f));
                        }
                    }
                }
                self.settings.export_path = first.display().to_string();
            }
        }
    }

    /// Starts the next queued import once nothing else runs.
    fn next_import(&mut self) {
        if self.import_blocked().is_some() {
            return;
        }
        let Some(path) = self.pending_imports.pop_front() else { return };
        if path.is_dir() {
            self.add_project(path);
        } else {
            self.import_scan(path);
        }
    }

    pub fn poll_jobs(&mut self) {
        self.next_import();
        if self.can_edit() {
            if let Some(dir) = self.pending_open.take() {
                self.open_project(dir);
            }
        }
        while let Ok(result) = self.job_rx.try_recv() {
            self.busy = None;
            // The state before the job goes on the undo stack only if the job changed it: a job
            // that failed or changed nothing leaves the history as it was.
            if let Some(snapshot) = self.pending_undo.take().filter(|_| Self::changes_geometry(&result)) {
                self.record(snapshot);
            }
            match result {
                JobResult::Cleaned { passes, merged, message } => {
                    self.apply_cleaned(passes, merged);
                    self.say(Tone::Success, message);
                }
                JobResult::NoChange(message) => self.say(Tone::Warning, message),
                JobResult::Mesh { mesh, analysis, message } => {
                    let g = self.generation();
                    self.mesh = Some(MeshLayer::new(mesh, analysis, g));
                    if let Some(m) = &mut self.merged {
                        m.layer.visible = false;
                    }
                    for p in &mut self.passes {
                        p.layer.visible = false;
                    }
                    self.selection = Some(Node::Mesh);
                    self.say(Tone::Success, message);
                }
                JobResult::Rebuilt { pass, points, keyframes, message } => {
                    let g = self.generation();
                    if let Some(p) = self.passes.iter_mut().find(|p| p.id == pass) {
                        // A cleaned pass keeps showing its cleaned surface.
                        if p.edit.is_none() {
                            let visible = p.layer.visible;
                            p.layer = CloudLayer::new(PointCloud::from_points(project::place(points, &p.meta.transform)), g);
                            p.layer.visible = visible;
                        }
                        p.keyframes = keyframes;
                    }
                    self.say(Tone::Success, message);
                }
                JobResult::Aligned { pass, method, alignment, check, message } => self.show_preview(pass, method, alignment, check, message),
                JobResult::Oriented(result, source) => self.show_orient(*result, source),
                JobResult::AlignedAll(outcome) => self.show_align_all(*outcome),
                JobResult::Refined(outcome) => self.show_refinement(*outcome),
                JobResult::Isolated { segmentation, picks } => {
                    let g = self.preview_generation();
                    if let Some(o) = self.object_picking.as_mut() {
                        let kept = segmentation.count(scancraft_fusion::SegmentLabel::Object);
                        let total = o.input.len();
                        o.set_result(segmentation, picks, g);
                        if o.current() {
                            let tone = if kept == 0 { Tone::Warning } else { Tone::Normal };
                            self.say(tone, format!("Object: {} of {} points · click to keep a cluster, Shift+click to leave one out", thousands(kept), thousands(total)));
                        }
                    }
                    // Picks made while it ran.
                    self.segment_picks();
                }
                JobResult::AiPrompted { pass, keyframe_index, mask, overlay } => {
                    let score = mask.iou_score;
                    if let Some(ai) = self.ai_picking.as_mut().filter(|a| a.pass == pass) {
                        ai.set_prompt_result(keyframe_index, mask, overlay);
                    }
                    self.refresh_ai_status();
                    self.say(Tone::Normal, format!("SAM 2.1 mask updated (IoU {score:.2}) · run Propagate all when ready"));
                }
                JobResult::AiPropagated { pass, masks, labels, message } => {
                    let g = self.preview_generation();
                    let provider = self.ai_engine.provider_summary().unwrap_or_default();
                    if let Some(ai) = self.ai_picking.as_mut().filter(|a| a.pass == pass) {
                        ai.set_propagation_result(masks, labels, provider, g);
                    }
                    self.refresh_ai_status();
                    self.say(Tone::Success, message);
                }
                JobResult::AiPackUpdated { status, message, ok } => {
                    self.ai_status = status;
                    self.say(if ok { Tone::Success } else { Tone::Warning }, message);
                }
                JobResult::Merged { cloud, passes, message } => {
                    // Meshing starts again from the new model.
                    self.invalidate_derived("New merged model");
                    let g = self.generation();
                    self.merged = Some(MergedLayer::new(cloud, g, passes, message.clone()));
                    for p in &mut self.passes {
                        p.layer.visible = false;
                    }
                    self.selection = Some(Node::Merged);
                    self.say(Tone::Success, message);
                }
                JobResult::Saved { path, name, message } => {
                    self.project.path = Some(path);
                    self.project.name = name;
                    self.project.saved_generation = self.next_generation;
                    self.tool = None;
                    self.say(Tone::Success, message);
                    if std::mem::take(&mut self.pending_close) {
                        self.close_active_tab();
                    }
                    if self.save_all_queue.is_some() {
                        self.save_next_for_quit();
                    }
                }
                JobResult::Opened(opened) => self.install_project(*opened),
                JobResult::Imported(scan) => self.install_imported(*scan),
                JobResult::AddedProject { name, passes } => self.install_added(name, passes),
                JobResult::Exported(message) => self.say(Tone::Success, message),
                JobResult::Failed(message) => {
                    // A failed save must not close the tab it was for, nor quit.
                    self.pending_close = false;
                    if self.save_all_queue.take().is_some() {
                        self.quit_confirmed = false;
                    }
                    self.say(Tone::Error, message)
                }
            }
        }
    }

    // ------------------------------------------------------------------ view

    pub fn fit_view(&mut self) {
        match self.data_bbox() {
            Some(b) => self.camera.fit_bounding_box(&b),
            None => {
                self.camera.target = glam::Vec3::new(0.0, 250.0, 0.0);
                self.camera.set_view_preset(CameraPreset::Isometric);
            }
        }
    }

    pub fn home_view(&mut self) {
        self.camera.set_view_preset(CameraPreset::Isometric);
        self.fit_view();
    }

    /// Keep the frame loop running while something is live.
    pub fn needs_repaint(&self) -> bool {
        self.capture != Capture::Idle || self.previewing || self.preview_starting || self.connecting || self.discovering || self.busy.is_some()
    }

    /// Hardware safety: stop the device on exit, bounded so a wedged device cannot hang exit.
    pub fn shutdown(&mut self) {
        if let Some(conn) = self.connection.take() {
            self.rt.block_on(async move {
                let stop = async move {
                    let mut dev = conn.device.lock().await;
                    let _ = dev.stop_scan().await;
                    let _ = dev.stop_preview().await;
                    let _ = dev.disconnect().await;
                };
                if tokio::time::timeout(Duration::from_secs(3), stop).await.is_err() {
                    log::warn!("Scanner shutdown timed out; the device may still be streaming");
                }
            });
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::alignment::{EdgeChange, RefineOutcome};
    use crate::project::PassMarker;
    use glam::Vec3;
    use scancraft_align::{Alignment, Quality};
    use scancraft_core::{Mesh, Point3D};

    fn app() -> ScanCraftApp {
        let rt = Arc::new(Runtime::new().unwrap());
        ScanCraftApp::new(rt, StartConnection::None)
    }

    /// A pass with a few points and an empty tracker.
    fn add_pass(app: &mut ScanCraftApp, id: u32, alignment: AlignState) {
        let g = app.generation();
        let cloud = PointCloud::from_points((0..300).map(|i| Point3D::new([i as f32, 0.0, 0.0])).collect());
        let mut meta = PassMeta::new(ScanConfig::default(), None, crate::project::PassStart::Moved);
        meta.alignment = alignment;
        app.passes.push(ScanPass {
            id,
            name: format!("Pass {id}"),
            layer: CloudLayer::new(cloud, g),
            frames: 1,
            tracker: Arc::new(std::sync::Mutex::new(scancraft_slam::Tracker::new())),
            keyframes: Vec::new(),
            meta,
            tinted: None,
            mask: isolation::new_cache(),
            surface: None,
            edit: None,
            rgb_keyframes: Vec::new(),
            markers: Vec::new(),
        });
    }

    /// The app with two passes and a mesh.
    fn with_results() -> ScanCraftApp {
        let mut app = app();
        add_pass(&mut app, 1, AlignState::Reference);
        add_pass(&mut app, 2, AlignState::Unaligned);
        let g = app.generation();
        let mesh = Mesh::default();
        let analysis = Mesher::analyze(&mesh);
        app.mesh = Some(MeshLayer::new(mesh, analysis, g + 1));
        app
    }

    /// Waits for the running job and takes its result.
    fn finish_job(app: &mut ScanCraftApp) {
        let t0 = Instant::now();
        while app.busy.is_some() && t0.elapsed() < Duration::from_secs(20) {
            app.poll_jobs();
            std::thread::sleep(Duration::from_millis(5));
        }
        assert!(app.busy.is_none(), "the job did not finish");
    }

    #[test]
    fn make_solid_replaces_the_mesh_with_a_closed_solid_and_undo_takes_it_back() {
        // Both faces of a spherical cap with a 3 mm wall, normals out of the material.
        let mut points = Vec::new();
        for (r, sign) in [(30.0f32, 1.0f32), (27.0, -1.0)] {
            let n = (4.0 * std::f32::consts::PI * r * r / 0.25) as usize;
            for i in 0..n {
                let z = 1.0 - 2.0 * (i as f32 + 0.5) / n as f32;
                if z < 0.5 {
                    break;
                }
                let phi = i as f32 * 2.399_963;
                let d = glam::Vec3::new((1.0 - z * z).sqrt() * phi.cos(), (1.0 - z * z).sqrt() * phi.sin(), z);
                points.push(Point3D { position: (d * r).to_array(), normal: (d * sign).to_array(), ..Point3D::new([0.0; 3]) });
            }
        }
        let mut app = app();
        // An imported pass: Make Solid reads each placed pass's own surface.
        add_fixed_pass(&mut app, 1, points, Transform::default());
        app.settings.whole_scene = true;
        app.settings.solid_cell_mm = 0.5;
        assert!(app.tool_blocked(Tool::MakeSolid).is_none());
        app.run_tool(Tool::MakeSolid);
        // (Longer than `finish_job` waits: the solve shares the thread pool with the other tests.)
        let t0 = Instant::now();
        while app.busy.is_some() && t0.elapsed() < Duration::from_secs(180) {
            app.poll_jobs();
            std::thread::sleep(Duration::from_millis(5));
        }
        let mesh = app.mesh.as_ref().expect("Make Solid gives a mesh");
        assert!(mesh.analysis.is_watertight, "{:?} · {:?}", mesh.analysis, app.current_toast());
        assert!(mesh.analysis.signed_volume_mm3 > 5_000.0, "{:?}", mesh.analysis);
        app.undo();
        assert!(app.mesh.is_none());
    }

    #[test]
    fn accepting_an_alignment_drops_stale_results_and_undo_restores_them() {
        let mut app = with_results();
        let g = app.generation();
        let quality = Quality { overlap: 0.5, rmse_mm: 0.2, visibility_conflict: 0.0, inliers: 1000, ..Quality::NONE };
        let transform = scancraft_align::rigid(glam::Quat::from_rotation_z(0.3), glam::Vec3::new(5.0, 0.0, 0.0));
        app.align_preview = Some(AlignPreview { pass: 2, method: AlignMethod::Auto, alignment: Alignment { transform, quality }, check: None, layer: CloudLayer::new(PointCloud::new(), g) });
        app.accept_preview();
        assert!(app.mesh.is_none(), "the mesh was made before the pass moved");
        assert!(app.working_source() == "Scan passes");
        assert!(app.current_toast().is_some_and(|t| t.contains("Undo restores them")));
        app.undo();
        assert!(app.mesh.is_some());
        assert_eq!(app.passes[1].meta.alignment, AlignState::Unaligned);
    }

    #[test]
    fn moving_a_pass_closes_an_open_isolate_object_tool() {
        let mut app = with_results();
        let input = app.working_cloud();
        let g = app.generation();
        app.object_picking = Some(ObjectPicking::new(input, g, CleanTarget::Merged));
        app.tool = Some(Tool::IsolateObject);
        app.invalidate_derived("Pass 2 was aligned");
        assert!(app.object_picking.is_none() && app.tool.is_none(), "the tool worked on the old cloud");
    }

    #[test]
    fn align_all_needs_the_reference_pass() {
        let mut app = app();
        for (id, state) in [(1, AlignState::Reference), (2, AlignState::Unaligned), (3, AlignState::Unaligned)] {
            add_pass(&mut app, id, state);
        }
        assert_eq!(app.align_all_blocked(), None);
        let g = app.generation();
        app.passes[0].layer = CloudLayer::new(PointCloud::from_points((0..150).map(|i| Point3D::new([i as f32, 0.0, 0.0])).collect()), g);
        assert!(app.align_all_blocked().is_some_and(|why| why.contains("reference")));
    }

    #[test]
    fn a_new_merge_replaces_the_mesh() {
        let mut app = with_results();
        app.passes[1].meta.alignment = AlignState::Aligned { method: AlignMethod::Auto, quality: None };
        app.spawn_undoable("Merging passes", "merge", || JobResult::Merged {
            cloud: PointCloud::from_points(vec![Point3D::new([0.0, 0.0, 0.0])]),
            passes: vec![1, 2],
            message: "Merged".into(),
        });
        finish_job(&mut app);
        assert!(app.merged.is_some());
        assert!(app.mesh.is_none());
        assert_eq!(app.working_source(), "Merged model");
        // One undo step brings back the state before the merge.
        assert_eq!(app.history.undo_label(), Some("merge"));
        app.undo();
        assert!(app.merged.is_none() && app.mesh.is_some());
    }

    #[test]
    fn a_failed_job_leaves_no_undo_step() {
        let mut app = with_results();
        app.spawn_undoable("Merging passes", "merge", || JobResult::Failed("tracker unavailable".into()));
        finish_job(&mut app);
        assert!(!app.history.can_undo(), "the failed merge left an undo step");
        assert!(app.mesh.is_some(), "a failed job must not touch the results");
        app.spawn_undoable("Removing outliers", "Remove outliers", || JobResult::NoChange("nothing".into()));
        finish_job(&mut app);
        assert!(!app.history.can_undo());
        // A job that does not change geometry and was not undoable keeps older steps.
        app.history.push(app.snapshot("earlier"));
        app.spawn_job("Refining all alignments", || JobResult::NoChange("No passes overlap enough to refine".into()));
        finish_job(&mut app);
        assert_eq!(app.history.undo_label(), Some("earlier"));
    }

    #[test]
    fn a_refinement_is_previewed_and_a_worse_one_rejected() {
        let mut app = with_results();
        app.passes[1].meta.alignment = AlignState::Aligned { method: AlignMethod::Auto, quality: None };
        let q = |rmse: f32| Quality { overlap: 0.5, rmse_mm: rmse, visibility_conflict: 0.0, inliers: 1000, ..Quality::NONE };
        let moved = scancraft_align::rigid(glam::Quat::IDENTITY, glam::Vec3::new(0.0, 1.0, 0.0));
        let outcome = |before: f32, after: f32| RefineOutcome {
            poses: vec![(2, moved)],
            edges: vec![EdgeChange { a: 2, b: 1, before: q(before), after: q(after), thickness_mm: None }],
            qualities: vec![(2, q(after))],
            seconds: 0.1,
        };
        app.show_refinement(outcome(0.4, 0.6));
        assert!(app.poses_preview.is_none(), "a worse refinement is rejected at once");
        assert!(app.passes[1].meta.transform == Transform::default());
        app.show_refinement(outcome(0.6, 0.3));
        assert!(app.poses_preview.as_ref().is_some_and(|p| p.layers.len() == 1));
        app.accept_poses();
        assert!(app.poses_preview.is_none());
        assert_eq!(app.passes[1].meta.transform, moved);
        assert!(matches!(app.passes[1].meta.alignment, AlignState::Aligned { method: AlignMethod::Refined, quality: Some(r) } if r.rmse_mm == 0.3));
        assert!(app.mesh.is_none());
        app.undo();
        assert_eq!(app.passes[1].meta.transform, Transform::default());
    }

    /// A temporary folder for one test.
    fn temp_dir(name: &str) -> std::path::PathBuf {
        let dir = std::env::temp_dir().join(format!("scancraft-app-{name}-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        dir
    }

    /// A PLY file of a 0.2 x 0.2 grid with normals (a scan saved in metres).
    fn write_scan(dir: &std::path::Path) -> std::path::PathBuf {
        let points: Vec<Point3D> = (0..900).map(|i| Point3D::new([(i % 30) as f32 * 0.007, (i / 30) as f32 * 0.007, 0.0]).with_normal([0.0, 0.0, 1.0])).collect();
        let mesh = Mesh::from_geometry(points, Vec::new());
        let path = dir.join("part.ply");
        scancraft_io::save_mesh_ply_binary(&mesh, &path).unwrap();
        path
    }

    #[test]
    fn an_imported_scan_is_an_undoable_fixed_pass_that_round_trips() {
        let dir = temp_dir("import");
        let mut app = app();
        app.import_scan(write_scan(&dir));
        finish_job(&mut app);
        assert_eq!(app.passes.len(), 1, "{}", app.status);
        let id = app.passes[0].id;
        assert!(app.passes[0].is_fixed() && app.passes[0].keyframes.is_empty());
        assert_eq!(app.passes[0].meta.origin, crate::project::PassOrigin::File { source: "part.ply".into() });
        // The first pass of the project defines its frame.
        assert_eq!(app.passes[0].meta.alignment, AlignState::Reference);
        assert!(crate::isolation::cached(&app.passes[0].mask).is_some());
        assert_eq!(app.history.undo_label(), Some("Import part.ply"));
        // 0.2 units across: offered as metres.
        assert_eq!(app.toast_action, Some(ToastAction::MetresToMm(id)));
        app.run_toast_action();
        let extent = crate::imports::largest_extent(&app.passes[0].layer.cloud);
        assert!((extent - 203.0).abs() < 0.1, "scaled to {extent}");
        assert!(app.toast_action.is_none());
        // Continuing an imported pass is refused.
        app.continue_scan(id);
        assert_eq!(app.capture, Capture::Idle);
        // Saved and opened again, it is still a fixed pass with its surface.
        let project = dir.join("project");
        app.save_project(project.clone());
        finish_job(&mut app);
        let mut reopened = self::app();
        reopened.open_project(project);
        finish_job(&mut reopened);
        assert_eq!(reopened.passes.len(), 1, "{}", reopened.status);
        assert!(reopened.passes[0].is_fixed());
        assert_eq!(reopened.passes[0].layer.len(), 900);
        assert!((crate::imports::largest_extent(&reopened.passes[0].layer.cloud) - 203.0).abs() < 0.1);
        // Undo takes back the scaling, then the import.
        app.undo();
        assert!(crate::imports::largest_extent(&app.passes[0].layer.cloud) < 1.0);
        app.undo();
        assert!(app.passes.is_empty());
        app.redo();
        assert!(app.passes.len() == 1 && app.passes[0].is_fixed());
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn an_added_project_gets_fresh_ids_and_stays_unaligned() {
        let dir = temp_dir("add").join("Demo");
        crate::dev::write_demo_project(&dir).unwrap();
        let mut app = with_results();
        app.next_pass_id = 3;
        app.add_project(dir.clone());
        finish_job(&mut app);
        assert_eq!(app.passes.len(), 4, "{}", app.status);
        let added: Vec<&ScanPass> = app.passes[2..].iter().collect();
        assert_eq!(added.iter().map(|p| p.id).collect::<Vec<_>>(), vec![3, 4]);
        assert_eq!(added[0].name, "Pass 1 (Demo)", "the name was taken");
        assert_eq!(added[1].name, "Pass 2 (flipped)");
        for p in &added {
            assert_eq!(p.meta.alignment, AlignState::Unaligned);
            assert_eq!(p.meta.transform, Transform::default());
            assert_eq!(p.meta.origin, crate::project::PassOrigin::Project { source: "Demo".into() });
            assert!(!p.is_fixed() && !p.keyframes.is_empty() && p.layer.len() > 1000);
        }
        // The project's own reference is unchanged; the next new pass gets the next id.
        assert_eq!(app.reference_pass(), Some(1));
        assert_eq!(app.next_pass_id, 5);
        assert_eq!(app.history.undo_label(), Some("Add Project Demo"));
        app.undo();
        assert_eq!(app.passes.iter().map(|p| p.id).collect::<Vec<_>>(), vec![1, 2]);
        // Into an empty project, the first added pass becomes the reference.
        let mut empty = self::app();
        empty.add_project(dir.clone());
        finish_job(&mut empty);
        assert_eq!(empty.passes.len(), 2);
        assert_eq!(empty.passes[0].meta.alignment, AlignState::Reference);
        assert_eq!(empty.passes[1].meta.alignment, AlignState::Unaligned);
        let _ = std::fs::remove_dir_all(dir.parent().unwrap());
    }

    /// An imported pass with a fixed surface, placed by `transform`.
    fn add_fixed_pass(app: &mut ScanCraftApp, id: u32, points: Vec<Point3D>, transform: Transform) {
        let g = app.generation();
        let surface = Arc::new(PointCloud::from_points(points));
        let mut meta = PassMeta::imported(format!("scan{id}.ply"));
        meta.transform = transform;
        meta.alignment = if id == 1 { AlignState::Reference } else { AlignState::Aligned { method: AlignMethod::Auto, quality: None } };
        app.passes.push(ScanPass {
            id,
            name: format!("Scan {id}"),
            layer: CloudLayer::new(PointCloud::from_points(project::place(surface.points.clone(), &transform)), g),
            frames: 0,
            tracker: Arc::new(std::sync::Mutex::new(scancraft_slam::Tracker::new())),
            keyframes: Vec::new(),
            meta,
            tinted: None,
            mask: isolation::new_cache(),
            surface: Some(surface),
            edit: None,
            rgb_keyframes: Vec::new(),
            markers: Vec::new(),
        });
    }

    /// A table with a block on it (block: |x|, |y| <= 20, 0 <= z <= 40).
    fn table_and_block() -> Vec<Point3D> {
        let mut points = Vec::new();
        for i in 0..100 {
            for j in 0..100 {
                let (x, y) = (-100.0 + 2.0 * i as f32, -100.0 + 2.0 * j as f32);
                if x.abs() > 20.0 || y.abs() > 20.0 {
                    points.push(Point3D::new([x, y, 0.0]).with_normal([0.0, 0.0, 1.0]));
                }
            }
        }
        for a in 0..=40 {
            for b in 0..=40 {
                let (u, v) = (-20.0 + a as f32, b as f32);
                points.push(Point3D::new([u, -20.0, v]).with_normal([0.0, -1.0, 0.0]));
                points.push(Point3D::new([u, 20.0, v]).with_normal([0.0, 1.0, 0.0]));
                points.push(Point3D::new([-20.0, u, v]).with_normal([-1.0, 0.0, 0.0]));
                points.push(Point3D::new([20.0, u, v]).with_normal([1.0, 0.0, 0.0]));
                points.push(Point3D::new([u, -20.0 + b as f32, 40.0]).with_normal([0.0, 0.0, 1.0]));
            }
        }
        points
    }

    #[test]
    fn clean_tools_work_on_each_shown_pass_and_leave_hidden_ones_alone() {
        let mut app = app();
        let total = table_and_block().len();
        add_fixed_pass(&mut app, 1, table_and_block(), Transform::default());
        // The second pass stands elsewhere and tilted in the project frame.
        let moved = scancraft_align::rigid(glam::Quat::from_rotation_x(0.4), glam::Vec3::new(300.0, 0.0, 50.0));
        add_fixed_pass(&mut app, 2, table_and_block(), moved);
        app.stage = Stage::Clean;
        // Work on scan 1 only.
        app.passes[1].layer.visible = false;
        assert_eq!(app.clean_targets(), vec![CleanTarget::Pass(1)]);
        app.run_tool(Tool::RemovePlane);
        finish_job(&mut app);
        let first = app.passes[0].layer.len();
        assert!(app.passes[0].is_cleaned() && first < total, "the table of scan 1 was removed ({first} of {total})");
        assert!(!app.passes[1].is_cleaned() && app.passes[1].layer.len() == total, "the hidden scan was touched");
        assert!(!app.passes[1].layer.visible && app.passes[0].layer.visible, "cleaning changed what is shown");
        // Now scan 2 only: scan 1 keeps its cleaning, scan 2 loses its own (tilted) table.
        app.passes[0].layer.visible = false;
        app.passes[1].layer.visible = true;
        app.run_tool(Tool::RemovePlane);
        finish_job(&mut app);
        assert_eq!(app.passes[0].layer.len(), first, "scan 1 was cleaned again while hidden");
        let second = app.passes[1].layer.len();
        assert!(app.passes[1].is_cleaned() && second < total && second > 5000, "scan 2 kept {second} of {total}");
        // The cleaned surface is stored in the pass's own frame and is what merging reads.
        let own = app.passes[1].edit.as_ref().unwrap();
        assert!(own.points.iter().all(|p| p.position[2] > 0.5), "scan 2's table is gone in its own frame");
        assert_eq!(app.passes[1].source().surfels().unwrap().len(), second);
        assert!(own.points.iter().all(|p| p.confidence as u32 == crate::project::FIXED_OBSERVATIONS));
        // Undo brings scan 2 back as imported, scan 1 stays cleaned.
        app.undo();
        assert!(!app.passes[1].is_cleaned() && app.passes[1].layer.len() == total);
        assert!(app.passes[0].is_cleaned());
        // Reset Cleaning on the shown scans only.
        app.passes[0].layer.visible = true;
        app.passes[1].layer.visible = false;
        app.reset_cleaning();
        assert!(!app.passes[0].is_cleaned() && app.passes[0].layer.len() == total);
    }

    /// Two imported scans, a merged model made from both and a mesh.
    fn merged_and_meshed() -> ScanCraftApp {
        let mut app = app();
        add_fixed_pass(&mut app, 1, table_and_block(), Transform::default());
        add_fixed_pass(&mut app, 2, table_and_block(), Transform::default());
        let g = app.generation();
        app.merged = Some(MergedLayer::new(PointCloud::from_points(table_and_block()), g, vec![1, 2], "Merged".into()));
        let mesh = Mesh::default();
        let analysis = Mesher::analyze(&mesh);
        app.mesh = Some(MeshLayer::new(mesh, analysis, g + 1));
        app
    }

    #[test]
    fn cleaning_a_pass_drops_the_merged_model_and_mesh_made_from_it() {
        let mut app = merged_and_meshed();
        app.merged.as_mut().unwrap().layer.visible = false;
        app.passes[1].layer.visible = false;
        app.run_tool(Tool::RemovePlane);
        finish_job(&mut app);
        assert!(app.passes[0].is_cleaned());
        assert!(app.merged.is_none() && app.mesh.is_none(), "results made before the cleaning are still there");
        assert!(app.current_toast().is_some_and(|t| t.contains("merged model") && t.contains("Undo")));
        app.undo();
        assert!(app.merged.is_some() && app.mesh.is_some() && !app.passes[0].is_cleaned(), "Undo brings them back");
        // Reset Cleaning changes the pass again: the results made from the cleaned pass go too.
        app.redo();
        assert!(app.passes[0].is_cleaned() && app.merged.is_none());
        let g = app.generation();
        app.merged = Some(MergedLayer::new(PointCloud::from_points(table_and_block()), g, vec![1, 2], "Merged".into()));
        app.reset_cleaning_of(&[1]);
        assert!(!app.passes[0].is_cleaned() && app.merged.is_none());
        assert!(!app.passes[1].layer.visible, "Reset Cleaning showed a hidden pass");
    }

    #[test]
    fn cleaning_only_the_merged_model_keeps_it_and_drops_the_mesh() {
        let mut app = merged_and_meshed();
        for p in &mut app.passes {
            p.layer.visible = false;
        }
        assert_eq!(app.clean_targets(), vec![CleanTarget::Merged]);
        app.run_tool(Tool::RemovePlane);
        finish_job(&mut app);
        let m = app.merged.as_ref().expect("the cleaned merged model stays");
        assert!(m.layer.len() < table_and_block().len());
        assert!(app.mesh.is_none(), "the mesh was made from the merged model before it was cleaned");
    }

    #[test]
    fn a_cleaned_pass_is_not_dropped_from_a_merge_it_is_not_in() {
        let mut app = merged_and_meshed();
        app.merged.as_mut().unwrap().passes = vec![2];
        app.merged.as_mut().unwrap().layer.visible = false;
        app.passes[1].layer.visible = false;
        app.run_tool(Tool::RemovePlane);
        finish_job(&mut app);
        assert!(app.merged.is_some(), "the merged model does not contain the cleaned pass");
    }

    #[test]
    fn a_cleaned_pass_seeds_a_same_position_pass() {
        let mut app = app();
        add_fixed_pass(&mut app, 1, table_and_block(), Transform::default());
        // Cleaned: only the block is left.
        let block: Vec<Point3D> = table_and_block().into_iter().filter(|p| p.position[2] > 0.5).collect();
        app.passes[0].edit = Some(Arc::new(PointCloud::from_points(block)));
        let tracker: SharedTracker = Arc::new(std::sync::Mutex::new(scancraft_slam::Tracker::new()));
        assert!(app.seed_tracker(&tracker), "a cleaned pass gave no starting model");
    }

    #[test]
    fn new_project_opens_a_tab_and_keeps_the_open_one() {
        let mut app = app();
        add_fixed_pass(&mut app, 1, table_and_block(), Transform::default());
        app.project.name = "Bracket".into();
        let points = app.passes[0].layer.len();
        assert!(app.dirty(), "an unsaved scan is a change");
        app.new_tab();
        assert_eq!(app.tabs.len(), 2);
        assert_eq!(app.active_tab, 1);
        assert!(app.passes.is_empty() && !app.dirty() && app.history.undo_label().is_none(), "the new tab is empty");
        assert_ne!(app.project.name, "Bracket");
        let labels = app.tab_labels();
        assert!(labels[0].name == "Bracket" && labels[0].dirty && !labels[0].active);
        // Back to the first tab: its scan, history and unsaved state are all there.
        app.switch_tab(0);
        assert_eq!(app.passes.len(), 1);
        assert_eq!(app.passes[0].layer.len(), points);
        assert!(app.dirty() && app.project.name == "Bracket");
    }

    #[test]
    fn closing_a_tab_with_unsaved_changes_asks_first() {
        let mut app = app();
        add_fixed_pass(&mut app, 1, table_and_block(), Transform::default());
        app.new_tab();
        // The empty tab closes at once.
        app.request_close_tab(1);
        assert_eq!(app.tabs.len(), 1);
        assert!(app.close_prompt.is_none() && app.passes.len() == 1);
        // The one with a scan asks; Cancel keeps it, Discard closes it.
        app.request_close_tab(0);
        assert_eq!(app.close_prompt, Some(ClosePrompt::Tab));
        app.close_prompt = None;
        assert_eq!(app.passes.len(), 1);
        app.request_close_tab(0);
        app.close_active_tab();
        assert!(app.passes.is_empty() && app.tabs.len() == 1, "the last tab leaves an empty project");
        assert!(!app.any_dirty());
    }

    #[test]
    fn save_and_close_closes_only_after_saving() {
        let mut app = app();
        add_fixed_pass(&mut app, 1, table_and_block(), Transform::default());
        app.new_tab();
        app.switch_tab(0);
        let dir = temp_dir("tab-save");
        app.project.path = Some(dir.clone());
        app.request_close_tab(0);
        app.save_then_close();
        finish_job(&mut app);
        assert!(scancraft_io::project::locate(&dir).is_ok_and(|l| !l.legacy), "saved");
        assert_eq!(app.tabs.len(), 1, "closed after the save");
        assert!(app.passes.is_empty());
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn tabs_cannot_switch_while_a_job_runs() {
        let mut app = app();
        app.new_tab();
        app.busy = Some("Merging".into());
        app.switch_tab(0);
        assert_eq!(app.active_tab, 1);
        app.busy = None;
        app.switch_tab(0);
        assert_eq!(app.active_tab, 0);
    }

    #[test]
    fn previews_do_not_mark_the_project_changed() {
        let mut app = app();
        add_fixed_pass(&mut app, 1, table_and_block(), Transform::default());
        app.project.saved_generation = app.next_generation;
        assert!(!app.dirty());
        let _ = app.preview_generation();
        app.refresh_holes();
        assert!(!app.dirty(), "showing something is not a change");
    }

    #[test]
    fn snapshots_share_the_passes_that_did_not_change() {
        let mut app = app();
        add_fixed_pass(&mut app, 1, table_and_block(), Transform::default());
        add_fixed_pass(&mut app, 2, table_and_block(), Transform::default());
        app.passes[1].layer.visible = false;
        app.run_tool(Tool::RemovePlane);
        finish_job(&mut app);
        assert!(app.passes[0].is_cleaned());
        // The step holds the old cloud of pass 1 and shares pass 2 with the live project.
        let held = app.history_memory();
        let old_pass1 = table_and_block().len() * (std::mem::size_of::<Point3D>() + std::mem::size_of::<scancraft_render::GpuPoint>());
        assert_eq!(held, old_pass1, "only the replaced cloud is held");
        // Undo brings back the very same layer (no copy, same GPU data).
        let before = Arc::as_ptr(&app.passes[1].layer.gpu);
        app.undo();
        assert!(!app.passes[0].is_cleaned());
        assert_eq!(Arc::as_ptr(&app.passes[1].layer.gpu), before);
    }

    #[test]
    fn jumping_to_a_timeline_step_undoes_or_redoes_several_actions_at_once() {
        let mut app = app();
        let total = table_and_block().len();
        add_fixed_pass(&mut app, 1, table_and_block(), Transform::default());
        app.run_tool(Tool::RemovePlane);
        finish_job(&mut app);
        let after_plane = app.passes[0].layer.len();
        app.settings.voxel_mm = 3.0;
        app.run_tool(Tool::Downsample);
        finish_job(&mut app);
        let after_down = app.passes[0].layer.len();
        assert!(after_down < after_plane && after_plane < total);
        assert_eq!(app.history.timeline().len(), 2);
        app.jump_to_step(0);
        assert_eq!(app.passes[0].layer.len(), total, "both steps undone in one go");
        assert!(app.history.timeline().iter().all(|s| !s.done));
        app.jump_to_step(2);
        assert_eq!(app.passes[0].layer.len(), after_down, "both redone");
        app.jump_to_step(1);
        assert_eq!(app.passes[0].layer.len(), after_plane);
        assert!(app.status.contains("Undid"), "{}", app.status);
    }

    #[test]
    fn a_project_picked_while_busy_opens_once_the_app_is_free() {
        let mut app = app();
        let dir = temp_dir("pending-open");
        add_fixed_pass(&mut app, 1, table_and_block(), Transform::default());
        app.save_project(dir.clone());
        finish_job(&mut app);
        app.new_tab();
        app.busy = Some("Merging".into());
        app.pending_open = Some(dir.clone());
        app.poll_jobs();
        assert!(app.pending_open.is_some() && app.passes.is_empty(), "nothing opens while busy");
        app.busy = None;
        app.poll_jobs();
        finish_job(&mut app);
        assert!(app.pending_open.is_none());
        assert_eq!(app.passes.len(), 1, "opened after the operation finished");
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn passes_of_another_open_tab_can_be_added_without_changing_it() {
        let mut app = app();
        add_fixed_pass(&mut app, 1, table_and_block(), Transform::default());
        app.project.name = "Bracket".into();
        app.new_tab();
        app.add_tab_passes(0);
        finish_job(&mut app);
        assert_eq!(app.passes.len(), 1, "the pass was copied in");
        let p = &app.passes[0];
        assert_eq!(p.meta.origin, crate::project::PassOrigin::File { source: "scan1.ply".into() }, "an imported pass keeps its file origin");
        assert_eq!(p.layer.len(), table_and_block().len());
        assert_eq!(app.history.undo_label(), Some("Add Passes of Bracket"));
        // The other tab still has its pass, untouched.
        app.switch_tab(0);
        assert_eq!(app.passes.len(), 1);
        app.switch_tab(1);
        app.undo();
        assert!(app.passes.is_empty());
    }

    #[test]
    fn make_solid_gets_the_passes_as_separate_scans_with_priority_and_markers() {
        let mut app = app();
        add_fixed_pass(&mut app, 1, table_and_block(), Transform::default());
        let moved = scancraft_align::rigid(glam::Quat::IDENTITY, glam::Vec3::new(0.0, 0.0, 0.5));
        add_fixed_pass(&mut app, 2, table_and_block(), moved);
        app.passes[1].meta.weight = 3.0;
        // A marker the tracker saw on top of the block of pass 2 (pass frame).
        app.passes[1].markers = vec![project::PassMarker::tracked([0.0, 0.0, 40.0], true)];
        app.settings.whole_scene = true;
        let (points, markers, priority) = project_ops::solid_points(app.solid_input()).unwrap();
        let total = table_and_block().len();
        assert_eq!(points.len(), 2 * total, "both passes, whole scene");
        assert_eq!(points.iter().filter(|p| p.scan == 1).count(), total);
        assert_eq!(points.iter().filter(|p| p.scan == 2).count(), total);
        // Pass 2 sits 0.5 mm higher in the project frame: its points were placed.
        let top2 = points.iter().filter(|p| p.scan == 2).map(|p| p.position.z).fold(f32::MIN, f32::max);
        assert!((top2 - 40.5).abs() < 1e-3, "{top2}");
        assert_eq!(priority, Some(vec![2, 1]), "the heavier pass is trusted first");
        assert_eq!(markers.len(), 1);
        let m = &markers[0];
        assert_eq!(m.scan, Some(2));
        assert!((m.center - glam::Vec3::new(0.0, 0.0, 40.5)).length() < 1e-3);
        assert!(m.normal.dot(glam::Vec3::Z) > 0.9, "normal from the surface around it: {:?}", m.normal);
        // Equal weights: no priority (overlapping passes are averaged).
        app.passes[1].meta.weight = app.passes[0].meta.weight;
        assert_eq!(project_ops::solid_points(app.solid_input()).unwrap().2, None);
        // Unplaced passes are left out, as in Merge.
        app.passes[1].meta.alignment = AlignState::Unaligned;
        assert_eq!(project_ops::solid_points(app.solid_input()).unwrap().0.len(), total);
    }

    #[test]
    fn make_solid_uses_a_listed_marker_normal_and_only_markers_on_the_surface() {
        let mut app = app();
        add_fixed_pass(&mut app, 1, table_and_block(), Transform::default());
        let tilted = glam::Vec3::new(0.3, 0.0, 1.0).normalize();
        app.passes[0].markers = vec![
            // On the block's top, with Revo's normal (kept, not replaced by the surface's).
            PassMarker { centre: glam::Vec3::new(5.0, 0.0, 40.0), normal: Some(tilted), radius_mm: Some(3.05), matched: false },
            // On the table, no normal: from the surface around it.
            PassMarker::tracked([60.0, 60.0, 0.0], false),
            // Where the scan has no points (Revo lists markers on parts it cut from the preview).
            PassMarker { centre: glam::Vec3::new(5.0, 0.0, 90.0), normal: Some(glam::Vec3::Z), radius_mm: Some(3.0), matched: false },
        ];
        app.settings.whole_scene = true;
        let (_, markers, _) = project_ops::solid_points(app.solid_input()).unwrap();
        assert_eq!(markers.len(), 2, "the marker off the surface is left out");
        assert!(markers[0].normal.dot(tilted) > 0.9999 && (markers[0].radius_mm - 3.05).abs() < 1e-6);
        assert!(markers[1].normal.dot(glam::Vec3::Z) > 0.99 && markers[1].radius_mm == 0.0);
        // The panel counts and the view draws what Make Solid uses, not the whole list.
        assert_eq!(app.solid_marker_count(), (2, true));
        assert_eq!(app.marker_use(&app.passes[0]).unwrap().on_scan, vec![true, true, false]);
        // On the object only: an upper bound until the pass is isolated, then the solid's number.
        app.settings.whole_scene = false;
        *app.passes[0].mask.lock().unwrap() = None;
        assert_eq!(app.solid_marker_count(), (2, false));
        let (_, markers, _) = project_ops::solid_points(app.solid_input()).unwrap();
        assert_eq!(app.solid_marker_count(), (markers.len(), true));
        // The cache follows the markers.
        app.passes[0].markers.pop();
        app.passes[0].markers.push(PassMarker::tracked([5.0, 0.0, 40.0], false));
        assert_eq!(app.marker_use(&app.passes[0]).unwrap().on_scan, vec![true, true, true]);
    }

    #[test]
    fn markers_on_surface_matches_a_brute_force_search() {
        // Points on a 1 mm grid of the plane z = 0, markers at assorted heights and positions.
        let points: Vec<(glam::Vec3, glam::Vec3)> = (0..60 * 60).map(|i| (glam::Vec3::new((i % 60) as f32, (i / 60) as f32, 0.0), glam::Vec3::Z)).collect();
        let map: Vec<PassMarker> = [[10.0, 10.0, 0.5], [10.0, 10.0, 2.9], [10.0, 10.0, 3.1], [-2.5, 30.0, 0.0], [-3.5, 30.0, 0.0], [59.0, 59.0, -2.0], [f32::NAN, 0.0, 0.0], [200.0, 0.0, 0.0]]
            .iter()
            .map(|c| PassMarker::tracked(*c, false))
            .collect();
        let got = project_ops::markers_on_surface(&map, points.iter().copied());
        let expected: Vec<bool> = map.iter().map(|m| points.iter().any(|(p, _)| p.distance_squared(m.centre) < 9.0)).collect();
        assert_eq!(got.iter().map(Option::is_some).collect::<Vec<_>>(), expected);
        assert_eq!(expected, vec![true, true, false, true, false, true, false, false]);
        assert!(got[0].unwrap().normalize().dot(glam::Vec3::Z) > 0.999);
    }

    #[test]
    fn a_metres_scan_with_a_broken_marker_file_still_says_it_has_no_markers() {
        let dir = temp_dir("metres-broken-mkf");
        let scan = write_scan(&dir);
        std::fs::write(dir.join(scancraft_io::revo::MARKER_FRAMEWORK_FILE), b"not a marker file").unwrap();
        let mut app = app();
        app.import_scan(scan);
        finish_job(&mut app);
        assert_eq!(app.passes.len(), 1, "{}", app.status);
        let toast = app.current_toast().unwrap_or_default().to_string();
        assert!(toast.contains("metres") && toast.contains("without markers"), "{toast}");
        assert!(app.toast_action.is_some());
        app.run_toast_action();
        assert!(app.status.contains("by 1000") && app.status.contains("without markers"), "{}", app.status);
        assert_eq!(app.tone, Tone::Warning);
        // Another toast drops the note: it belongs to that choice only.
        app.show_choice_toast("again", ToastAction::MetresToMm(app.passes[0].id));
        app.run_toast_action();
        assert!(!app.status.contains("without markers"), "{}", app.status);
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn a_revo_scan_keeps_its_markers_through_saving_tabs_undo_and_adding() {
        let dir = temp_dir("revo-markers");
        let folder = dir.join("Shell inside");
        std::fs::create_dir_all(&folder).unwrap();
        let scan = folder.join("raw_preview.ply");
        scancraft_io::save_ply_binary(&PointCloud::from_points(table_and_block()), &scan).unwrap();
        let mkf = folder.join(scancraft_io::revo::MARKER_FRAMEWORK_FILE);
        std::fs::write(&mkf, crate::imports::mkf_bytes(&[([5.0, 0.0, 40.0], [0.0, 0.0, 1.0], 3.05), ([60.0, 60.0, 0.0], [0.0, 0.0, 1.0], 3.0)])).unwrap();
        let mut app = app();
        app.import_scan(scan.clone());
        finish_job(&mut app);
        assert_eq!(app.passes.len(), 1, "{}", app.status);
        assert_eq!(app.passes[0].markers.len(), 2);
        assert!(app.status.contains("2 markers from marker_framework.mkf"), "{}", app.status);
        assert_eq!(app.tone, Tone::Success);
        // Saved and opened again.
        let project = dir.join("project");
        app.save_project(project.clone());
        finish_job(&mut app);
        let mut reopened = self::app();
        reopened.open_project(project);
        finish_job(&mut reopened);
        assert_eq!(reopened.passes[0].markers.len(), 2, "{}", reopened.status);
        assert_eq!(reopened.passes[0].markers[0].radius_mm, Some(3.05));
        // Another tab has its own passes; switching back finds the markers.
        app.project.name = "Revo".into();
        app.new_tab();
        add_fixed_pass(&mut app, 1, table_and_block(), Transform::default());
        app.next_pass_id = 2;
        app.switch_tab(0);
        assert_eq!(app.passes[0].markers.len(), 2);
        app.switch_tab(1);
        assert!(app.passes[0].markers.is_empty());
        // Added to the other tab, the pass gets a new id and keeps its markers.
        app.add_tab_passes(0);
        finish_job(&mut app);
        assert_eq!(app.passes.iter().map(|p| (p.id, p.markers.len())).collect::<Vec<_>>(), vec![(1, 0), (2, 2)]);
        // Deleted, the markers go with the pass; Undo brings both back.
        app.delete_pass(2);
        assert!(app.passes.iter().all(|p| p.markers.is_empty()));
        app.undo();
        assert_eq!(app.passes.iter().find(|p| p.id == 2).map(|p| p.markers.len()), Some(2));
        // A broken marker file does not stop the import, and says so.
        std::fs::write(&mkf, b"not a marker file").unwrap();
        app.import_scan(scan);
        finish_job(&mut app);
        assert_eq!(app.passes.len(), 3, "{}", app.status);
        assert!(app.passes[2].markers.is_empty());
        assert_eq!(app.tone, Tone::Warning, "{}", app.status);
        assert!(app.status.contains("could not be read"), "{}", app.status);
        assert!(app.current_toast().is_some_and(|t| t.contains("without markers")));
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// The `S1`, `S2`, `S3` scans of `SCANCRAFT_ALIGN_DATA` (the list of
    /// `crates/scancraft-align/tests/real_data.rs`: `key = path` lines, or a folder with
    /// `S1.ply` ...). Revo's `marker_framework.mkf` beside each scan comes in through the import.
    fn revo_scans(root: &std::path::Path) -> Vec<std::path::PathBuf> {
        let keys = ["S1", "S2", "S3"];
        if root.is_dir() {
            return keys.iter().map(|k| root.join(format!("{k}.ply"))).collect();
        }
        let text = std::fs::read_to_string(root).expect("read SCANCRAFT_ALIGN_DATA");
        let base = root.parent().unwrap_or(std::path::Path::new("."));
        let lines: HashMap<&str, &str> = text.lines().map(str::trim).filter(|l| !l.is_empty() && !l.starts_with('#')).filter_map(|l| l.split_once('=')).map(|(k, v)| (k.trim(), v.trim())).collect();
        keys.iter().map(|k| base.join(lines.get(k).unwrap_or_else(|| panic!("no {k} in SCANCRAFT_ALIGN_DATA")))).collect()
    }

    /// Runs the app's jobs until none is busy.
    fn wait_for_jobs(app: &mut ScanCraftApp, limit: Duration) {
        let t0 = Instant::now();
        while app.busy.is_some() && t0.elapsed() < limit {
            app.poll_jobs();
            std::thread::sleep(Duration::from_millis(20));
        }
        assert!(app.busy.is_none(), "the job did not finish in {limit:?}: {}", app.status);
    }

    #[cfg(windows)]
    fn peak_working_set_mb() -> Option<f64> {
        #[repr(C)]
        struct Pmc {
            cb: u32,
            page_fault_count: u32,
            peak_working_set: usize,
            working_set: usize,
            quota_peak_paged: usize,
            quota_paged: usize,
            quota_peak_nonpaged: usize,
            quota_nonpaged: usize,
            pagefile: usize,
            peak_pagefile: usize,
        }
        extern "system" {
            fn GetCurrentProcess() -> isize;
            fn K32GetProcessMemoryInfo(process: isize, pmc: *mut Pmc, cb: u32) -> i32;
        }
        // SAFETY: a plain-data struct filled by the OS for the current process.
        let mut p: Pmc = unsafe { std::mem::zeroed() };
        p.cb = std::mem::size_of::<Pmc>() as u32;
        let ok = unsafe { K32GetProcessMemoryInfo(GetCurrentProcess(), &mut p, p.cb) };
        (ok != 0).then(|| p.peak_working_set as f64 / 1048576.0)
    }
    #[cfg(not(windows))]
    fn peak_working_set_mb() -> Option<f64> {
        None
    }

    /// Revo's markers of two placed passes, compared under their poses: mutual nearest within
    /// 3 mm facing the same way (as `real_data.rs`), the RMS of the pairs, and the pose of `a`
    /// against the best fit of the pairs (degrees, and RMS over `a`'s points in mm).
    fn marker_check(a: &ScanPass, b: &ScanPass) -> serde_json::Value {
        let world = |p: &ScanPass| -> Vec<(Vec3, Vec3)> {
            let m = p.meta.transform.to_mat4();
            p.markers.iter().filter_map(|k| k.normal.map(|n| (m.transform_point3(k.centre), (p.meta.transform.rotation * n).normalize()))).collect()
        };
        let (wa, wb) = (world(a), world(b));
        let nearest = |q: &(Vec3, Vec3), set: &[(Vec3, Vec3)]| set.iter().enumerate().filter(|(_, s)| s.1.dot(q.1) > 0.866).min_by(|x, y| x.1 .0.distance(q.0).total_cmp(&y.1 .0.distance(q.0))).map(|(i, s)| (i, s.0.distance(q.0)));
        let mut pairs = Vec::new();
        for (i, qa) in wa.iter().enumerate() {
            if let Some((j, d)) = nearest(qa, &wb) {
                if d < 3.0 && nearest(&wb[j], &wa).map(|r| r.0) == Some(i) {
                    pairs.push((i, j, d));
                }
            }
        }
        let rms = (pairs.iter().map(|p| p.2 * p.2).sum::<f32>() / pairs.len().max(1) as f32).sqrt();
        let mut out = serde_json::json!({"from": a.name, "to": b.name, "markers": [wa.len(), wb.len()], "matched": pairs.len(), "rms_mm": rms});
        if pairs.len() >= 4 {
            // project <- a as the markers place it: fit a's own (pass frame) markers onto b's placed ones.
            let own: Vec<Vec3> = a.markers.iter().filter(|k| k.normal.is_some()).map(|k| k.centre).collect();
            if let Some(fit) = scancraft_align::horn::horn(&pairs.iter().map(|p| (own[p.0], wb[p.1].0)).collect::<Vec<_>>()) {
                let pose = a.meta.transform;
                let deg = (pose.rotation * fit.rotation.inverse()).angle_between(glam::Quat::IDENTITY).to_degrees();
                let (pm, fm) = (pose.to_mat4(), fit.to_mat4());
                let pts: Vec<Vec3> = a.source().surfels().map(|s| s.iter().step_by(50).map(|s| s.0).collect()).unwrap_or_default();
                let mm = (pts.iter().map(|p| pm.transform_point3(*p).distance_squared(fm.transform_point3(*p))).sum::<f32>() / pts.len().max(1) as f32).sqrt();
                let fit_rms = (pairs.iter().map(|p| fm.transform_point3(own[p.0]).distance_squared(wb[p.1].0)).sum::<f32>() / pairs.len() as f32).sqrt();
                out["fit_rms_mm"] = serde_json::json!(fit_rms);
                out["pose_vs_fit_deg"] = serde_json::json!(deg);
                out["pose_vs_fit_mm"] = serde_json::json!(mm);
            }
        }
        out
    }

    /// Make Solid of the three Revo Scan scans of the thin shell part, end to end through the app
    /// (Import Scan with Revo's markers, Align All, Accept, Make Solid with the default settings,
    /// Save, Open). Writes `solid.ply`, `solid_report.json` (the solid's own report), `app_report.json`
    /// and the project folder `project/` to `SCANCRAFT_SOLID_OUT`. Release build, one test:
    ///
    /// ```text
    /// SCANCRAFT_ALIGN_DATA=<list> SCANCRAFT_SOLID_OUT=<folder> cargo test -p scancraft --release make_solid_of_the_revo_scans -- --ignored --nocapture
    /// ```
    #[test]
    #[ignore = "needs the Revo scans (SCANCRAFT_ALIGN_DATA) and an output folder (SCANCRAFT_SOLID_OUT)"]
    fn make_solid_of_the_revo_scans_end_to_end() {
        let (Some(data), Some(out)) = (std::env::var_os("SCANCRAFT_ALIGN_DATA"), std::env::var_os("SCANCRAFT_SOLID_OUT")) else {
            eprintln!("SCANCRAFT_ALIGN_DATA or SCANCRAFT_SOLID_OUT is not set; skipping");
            return;
        };
        let out = std::path::PathBuf::from(out);
        std::fs::create_dir_all(&out).unwrap();
        let t_all = Instant::now();
        let mut app = app();

        // Import Scan, as the button does: Revo's marker_framework.mkf beside each file comes along.
        let mut imports = Vec::new();
        for path in revo_scans(std::path::Path::new(&data)) {
            let t = Instant::now();
            app.import_scan(path);
            wait_for_jobs(&mut app, Duration::from_secs(300));
            let p = app.passes.last().expect("imported");
            eprintln!("import: {}", app.status);
            assert_ne!(app.tone, Tone::Error, "{}", app.status);
            assert!(!p.markers.is_empty(), "no markers read for {}: {}", p.name, app.status);
            imports.push(serde_json::json!({"pass": p.id, "name": p.name, "points": p.layer.len(), "markers": p.markers.len(), "status": app.status, "seconds": t.elapsed().as_secs_f32()}));
        }
        assert_eq!(app.passes.len(), 3);

        // Align All, then Accept.
        let t = Instant::now();
        assert_eq!(app.align_all_blocked(), None);
        app.align_all();
        wait_for_jobs(&mut app, Duration::from_secs(1200));
        let align_status = app.status.clone();
        eprintln!("align all: {align_status}");
        let Some(PosesPreview { proposal: Proposal::AlignAll(outcome), .. }) = &app.poses_preview else { panic!("Align All proposed nothing: {align_status}") };
        let outcome = (**outcome).clone();
        let placements: Vec<serde_json::Value> = outcome
            .placements
            .iter()
            .map(|p| {
                let q = p.quality;
                serde_json::json!({
                    "pass": p.id, "name": app.pass_name(p.id), "reference": p.reference, "placed": p.pose.is_some(), "parent": p.parent,
                    "rotation_deg": p.pose.map(|t| t.rotation.angle_between(glam::Quat::IDENTITY).to_degrees()),
                    "translation_mm": p.pose.map(|t| t.translation.to_array()),
                    "verdict": q.map(|q| format!("{:?}", q.verdict())),
                    "overlap": q.map(|q| q.overlap), "rmse_mm": q.map(|q| q.rmse_mm), "visibility_conflict": q.map(|q| q.visibility_conflict),
                    "wall_offset_mm": q.and_then(|q| q.shell).map(|s| s.offset_mm), "piercing": q.and_then(|q| q.shell).map(|s| s.piercing),
                })
            })
            .collect();
        let edges: Vec<String> = outcome.edges.iter().map(|e| format!("{} -> {}: {:?}, {:?}, {:?}", app.pass_name(e.from), app.pass_name(e.to), e.kind, e.verdict, e.quality.map(|q| (q.overlap, q.rmse_mm)))).collect();
        app.accept_poses();
        let align_s = t.elapsed().as_secs_f32();
        assert!(app.passes.iter().all(|p| p.meta.alignment.is_placed()), "not every pass was placed: {align_status}");
        let mut markers = Vec::new();
        for i in 0..app.passes.len() {
            for j in i + 1..app.passes.len() {
                markers.push(marker_check(&app.passes[j], &app.passes[i]));
            }
        }
        let peak_after_align = peak_working_set_mb();

        // Make Solid with the default settings: the tool's own job, run here so its report is kept.
        let job = SolidJob::from_settings(&app.settings);
        let t = Instant::now();
        let solid = make_solid(app.solid_input(), job.clone()).expect("Make Solid");
        let solid_s = t.elapsed().as_secs_f32();
        let peak_after_solid = peak_working_set_mb();
        let analysis = Mesher::analyze(&solid.mesh);
        let r = &solid.report;
        // The panel's count (now that the passes are isolated) is what Make Solid used.
        assert_eq!(app.solid_marker_count(), (r.markers_listed, true), "the Make Solid panel's marker count");
        scancraft_io::save_mesh_ply_binary(&solid.mesh, out.join("solid.ply")).unwrap();
        std::fs::write(out.join("solid_report.json"), r.to_json()).unwrap();
        let faired: Vec<&str> = r.marker_outcomes.iter().filter(|o| o.fair.as_ref().is_some_and(|f| f.status == "faired")).map(|o| o.id.as_str()).collect();
        let not_faired: Vec<String> = r.marker_outcomes.iter().filter(|o| !o.fair.as_ref().is_some_and(|f| f.status == "faired")).map(|o| format!("{} ({:.1}, {:.1}, {:.1}): {} / {}", o.id, o.center.x, o.center.y, o.center.z, o.fill, o.fair.as_ref().map_or("no fairing", |f| f.status.as_str()))).collect();
        let t = &r.topology;
        let summary = serde_json::json!({
            "watertight": t.watertight, "manifold": t.manifold, "components": t.components.len(), "genus": t.genus_total(),
            "triangles": t.triangles, "volume_mm3": t.total_volume_mm3, "area_mm2": t.total_area_mm2,
            "wall_median_mm": r.wall_median_mm, "measured_fraction": r.origin_fractions()[0],
            "markers_listed": r.markers_listed, "markers_detected": r.markers_detected.len(), "marker_sites": r.marker_sites.len(),
            "sites_faired": r.sites_faired, "faired": faired, "not_faired": not_faired, "holes_filled": r.holes_filled,
            "solid_seconds": solid_s, "solid_stage_seconds": r.total_seconds(),
        });
        eprintln!("solid: {summary:#}");

        // Into the app as the tool's result, then saved and opened again.
        let message = solid_message(r, &analysis);
        let triangles = solid.mesh.triangles.len();
        let mesh = solid.mesh;
        app.spawn_undoable("Making the solid", "Make Solid", move || JobResult::Mesh { mesh, analysis, message });
        wait_for_jobs(&mut app, Duration::from_secs(60));
        assert!(app.mesh.is_some(), "{}", app.status);
        let project_dir = out.join("project");
        let _ = std::fs::remove_dir_all(&project_dir);
        app.save_project(project_dir.clone());
        wait_for_jobs(&mut app, Duration::from_secs(600));
        let mut reopened = self::app();
        reopened.open_project(project_dir);
        wait_for_jobs(&mut reopened, Duration::from_secs(600));
        let reopen: Vec<serde_json::Value> = app
            .passes
            .iter()
            .map(|p| {
                let q = reopened.passes.iter().find(|q| q.id == p.id);
                // (Opening normalises the normals again: equal to a few ulp, not bit for bit.)
                let same = |a: &PassMarker, b: &PassMarker| {
                    a.centre.distance(b.centre) < 1e-4
                        && match (a.normal, b.normal) {
                            (Some(x), Some(y)) => x.dot(y) > 1.0 - 1e-6,
                            (x, y) => x == y,
                        }
                        && match (a.radius_mm, b.radius_mm) {
                            (Some(x), Some(y)) => (x - y).abs() < 1e-5,
                            (x, y) => x == y,
                        }
                };
                let same = q.is_some_and(|q| q.markers.len() == p.markers.len() && q.markers.iter().zip(&p.markers).all(|(a, b)| same(a, b)));
                serde_json::json!({"pass": p.id, "markers": p.markers.len(), "reopened": q.map(|q| q.markers.len()), "same": same, "pose_same": q.is_some_and(|q| q.meta.transform == p.meta.transform)})
            })
            .collect();
        let reopened_mesh = reopened.mesh.as_ref().map(|m| m.mesh.triangles.len());

        let report = serde_json::json!({
            "imports": imports, "align": {"status": align_status, "seconds": align_s, "refine": format!("{:?}", outcome.refine), "placements": placements, "edges": edges},
            "markers_under_poses": markers, "solid": summary, "settings": format!("{job:?}"),
            "reopened": {"passes": reopen, "mesh_triangles": reopened_mesh, "saved_triangles": triangles},
            "peak_working_set_mb": {"after_align": peak_after_align, "after_solid": peak_after_solid},
            "total_seconds": t_all.elapsed().as_secs_f32(),
        });
        std::fs::write(out.join("app_report.json"), serde_json::to_string_pretty(&report).unwrap()).unwrap();
        eprintln!("{}", serde_json::to_string_pretty(&report).unwrap());

        assert!(t.watertight, "the solid is not watertight");
        assert!(reopen.iter().all(|p| p["same"] == true && p["pose_same"] == true), "the marker maps or poses did not come back: {reopen:?}");
        assert_eq!(reopened_mesh, Some(triangles));
    }

    #[test]
    fn save_all_on_quit_saves_every_project_then_closes() {
        let mut app = app();
        let (d1, d2) = (temp_dir("saveall-1"), temp_dir("saveall-2"));
        add_fixed_pass(&mut app, 1, table_and_block(), Transform::default());
        app.project.path = Some(d1.clone());
        app.new_tab();
        add_fixed_pass(&mut app, 1, table_and_block(), Transform::default());
        // Never saved: Save All cannot know where to put it.
        assert_eq!(app.dirty_without_folder().len(), 1);
        app.save_all_then_quit();
        assert!(!app.close_window && app.busy.is_none(), "nothing is saved while one project has no folder");
        app.project.path = Some(d2.clone());
        app.save_all_then_quit();
        for _ in 0..4 {
            finish_job(&mut app);
            app.poll_jobs();
        }
        assert!(scancraft_io::project::is_project(&d1) && scancraft_io::project::is_project(&d2));
        assert!(!app.any_dirty());
        assert!(app.close_window && app.quit_confirmed, "the window closes once all are saved");
        let _ = std::fs::remove_dir_all(&d1);
        let _ = std::fs::remove_dir_all(&d2);
    }

    #[test]
    fn make_reference_and_deleting_the_reference_keep_an_orientation() {
        let mut app = app();
        add_fixed_pass(&mut app, 1, table_and_block(), Transform::default());
        let second = scancraft_align::rigid(glam::Quat::from_rotation_z(0.2), glam::Vec3::new(5.0, 0.0, 0.0));
        add_fixed_pass(&mut app, 2, table_and_block(), second);
        // Orient the project (as Auto Orient / Datums do): every placed pass moves.
        let turn = scancraft_align::rigid(glam::Quat::from_rotation_x(0.7), glam::Vec3::new(0.0, 0.0, 12.0));
        app.move_project_frame(turn, "Auto Orient");
        let oriented: Vec<Transform> = app.passes.iter().map(|p| p.meta.transform).collect();
        let points: Vec<[f32; 3]> = app.passes.iter().map(|p| p.layer.cloud.points[7].position).collect();
        // Make the other pass the reference: nothing moves, only the label changes.
        app.set_reference(2);
        assert_eq!(app.passes[1].meta.alignment, AlignState::Reference);
        assert!(matches!(app.passes[0].meta.alignment, AlignState::Aligned { .. }));
        assert_eq!(app.passes.iter().map(|p| p.meta.transform).collect::<Vec<_>>(), oriented, "Make Reference moved the passes");
        assert_eq!(app.passes.iter().map(|p| p.layer.cloud.points[7].position).collect::<Vec<_>>(), points);
        // Deleting the reference hands the role on, again without moving anything.
        app.delete_pass(2);
        assert_eq!(app.passes[0].meta.alignment, AlignState::Reference);
        assert_eq!(app.passes[0].meta.transform, oriented[0]);
    }

    #[test]
    fn nothing_shown_means_nothing_to_clean() {
        let mut app = app();
        add_fixed_pass(&mut app, 1, table_and_block(), Transform::default());
        app.passes[0].layer.visible = false;
        assert!(app.tool_blocked(Tool::RemovePlane).is_some_and(|why| why.contains("show")));
        add_fixed_pass(&mut app, 2, table_and_block(), Transform::default());
        assert!(app.tool_blocked(Tool::RemovePlane).is_none());
        app.passes[0].layer.visible = true;
        assert!(app.isolate_blocked().is_some_and(|why| why.contains("only")), "isolate works on one scan at a time");
    }

    #[test]
    fn isolate_object_keeps_the_object_of_the_shown_pass() {
        let mut app = app();
        let total = table_and_block().len();
        let moved = scancraft_align::rigid(glam::Quat::from_rotation_z(0.3), glam::Vec3::new(10.0, -5.0, 0.0));
        add_fixed_pass(&mut app, 1, table_and_block(), Transform::default());
        add_fixed_pass(&mut app, 2, table_and_block(), moved);
        app.passes[0].layer.visible = false;
        app.open_tool(Tool::IsolateObject);
        assert_eq!(app.tool, Some(Tool::IsolateObject));
        finish_job(&mut app);
        assert!(app.object_picking.as_ref().is_some_and(|o| o.current() && o.target == CleanTarget::Pass(2)));
        // A pick on the block (where it stands in the project frame) keeps it.
        let pick = moved.to_mat4().transform_point3(glam::Vec3::new(0.0, 0.0, 40.0));
        app.add_object_pick(pick, true);
        finish_job(&mut app);
        app.run_tool(Tool::IsolateObject);
        let kept = app.passes[1].layer.len();
        assert!(app.passes[1].is_cleaned() && kept > 5000 && kept < total, "kept {kept} of {total}");
        let own = app.passes[1].edit.as_ref().unwrap();
        assert!(own.points.iter().all(|p| p.position[0].abs() < 25.0 && p.position[1].abs() < 25.0), "the object, in the pass's own frame");
        assert!(!app.passes[0].is_cleaned() && app.passes[0].layer.len() == total);
        assert!(app.tool.is_none() && app.object_picking.is_none());
        app.undo();
        assert!(!app.passes[1].is_cleaned() && app.passes[1].layer.len() == total);
    }

    #[test]
    fn mock_capture_with_color_populates_rgb_keyframes_by_default() {
        let mut app = app();
        app.connect_mock();
        let t0 = Instant::now();
        while app.connection.is_none() && t0.elapsed() < Duration::from_secs(5) {
            app.poll_device_events();
            std::thread::sleep(Duration::from_millis(5));
        }
        assert!(app.scan_config.color_texture, "RGB-capable scanner should default color_texture to true");
        app.start_scan();
        let t0 = Instant::now();
        while app.passes.first().is_none_or(|p| p.rgb_keyframes.is_empty()) && t0.elapsed() < Duration::from_secs(10) {
            app.poll_device_events();
            app.poll_updates(&egui::Context::default());
            std::thread::sleep(Duration::from_millis(10));
        }
        let rgb_count = app.passes.first().map(|p| p.rgb_keyframes.len()).unwrap_or(0);
        app.stop_scan();
        let t0 = Instant::now();
        while app.capture != Capture::Idle && t0.elapsed() < Duration::from_secs(5) {
            app.poll_device_events();
            std::thread::sleep(Duration::from_millis(5));
        }
        assert!(rgb_count > 0, "expected captured RGB keyframes on MockDevice");
    }

    #[test]
    fn continue_pass_adds_to_the_pass_marker_map_instead_of_replacing_it() {
        let mut app = app();
        app.connect_mock();
        let wait = |app: &mut ScanCraftApp, done: &dyn Fn(&ScanCraftApp) -> bool, what: &str| {
            let t0 = Instant::now();
            while !done(app) {
                assert!(t0.elapsed() < Duration::from_secs(10), "timed out waiting for {what}");
                app.poll_device_events();
                app.poll_updates(&egui::Context::default());
                std::thread::sleep(Duration::from_millis(5));
            }
        };
        wait(&mut app, &|a| a.connection.is_some(), "the simulator");
        app.start_scan();
        // A pass that fused nothing is dropped when the capture stops.
        wait(&mut app, &|a| a.passes.first().is_some_and(|p| p.layer.len() > 0), "the first capture's points");
        app.stop_scan();
        wait(&mut app, &|a| a.capture == Capture::Idle, "the first capture to stop");
        let id = app.passes[0].id;
        // The pass's map as saved (or from the first capture of this session).
        let saved = [[0.0, 0.0, 0.0], [20.0, 0.0, 0.0], [0.0, 20.0, 0.0]];
        app.passes[0].markers = saved.iter().map(|c| PassMarker::tracked(*c, false)).collect();
        app.continue_scan(id);
        wait(&mut app, &|a| a.capture == Capture::Running, "Continue Pass");
        assert_eq!(app.active_pass, Some(id));
        // The new capture's tracker has triangulated two markers: one already in the map (1 mm
        // off, matched this frame) and a new one.
        app.apply_capture_marker_map(&[([20.0, 1.0, 0.0], true), ([40.0, 40.0, 0.0], false)]);
        let markers = &app.passes[0].markers;
        assert_eq!(markers.len(), 4, "{markers:?}");
        assert_eq!(markers[..3].iter().map(|m| m.centre.to_array()).collect::<Vec<_>>(), saved.to_vec(), "the saved markers are kept as they were");
        assert_eq!(markers.iter().map(|m| m.matched).collect::<Vec<_>>(), vec![false, true, false, false]);
        assert_eq!(markers[3].centre, glam::Vec3::new(40.0, 40.0, 0.0));
        // A later frame with a larger tracker map still merges into the map the pass started with.
        app.apply_capture_marker_map(&[([20.0, 1.0, 0.0], false), ([40.0, 40.0, 0.0], true), ([0.0, 40.0, 0.0], true)]);
        assert_eq!(app.passes[0].markers.len(), 5);
        app.stop_scan();
        wait(&mut app, &|a| a.capture == Capture::Idle, "Continue Pass to stop");
        assert_eq!(app.passes[0].markers.len(), 5);
        assert!(app.passes[0].markers.iter().all(|m| !m.matched));
    }

    #[test]
    fn merged_marker_maps_keep_the_base_and_add_only_new_stickers() {
        let base = vec![PassMarker { centre: glam::Vec3::new(1.0, 2.0, 3.0), normal: Some(glam::Vec3::Z), radius_mm: Some(3.05), matched: false }];
        // Within SAME_MARKER_MM: the same sticker (the listed one with its normal is kept).
        let merged = PassMarker::merged(&base, &[([1.0, 2.0, 5.5], true)]);
        assert_eq!(merged, vec![PassMarker { matched: true, ..base[0] }]);
        // Farther: another sticker.
        assert_eq!(PassMarker::merged(&base, &[([1.0, 2.0, 7.0], true)]).len(), 2);
        // A first capture (no base) takes the tracker's map as it is.
        assert_eq!(PassMarker::merged(&[], &[([0.0; 3], true), ([1.0, 0.0, 0.0], false)]).len(), 2);
    }

    #[test]
    #[ignore = "requires SAM 2.1 models + ONNX Runtime DirectML/CPU binaries in SCANCRAFT_SAM2_DIR / SCANCRAFT_ORT_DIR or %LOCALAPPDATA%/ScanCraft/ai"]
    fn ai_segment_live_workflow_on_sam21() {
        let mut app = app();
        app.refresh_ai_status();
        assert!(app.ai_status.ready, "AI Pack must be installed and verified: {:?}", app.ai_status);
        let total = table_and_block().len();
        add_fixed_pass(&mut app, 1, table_and_block(), Transform::default());
        // Synthetic RGB keyframes aligned to the block standing at z > 0 along optical axis (+Y = 0..200 mm).
        let cam = Arc::new(scancraft_core::ColorCamera {
            width: 128,
            height: 128,
            intrinsics: scancraft_core::PinholeIntrinsics { fx: 160.0, fy: 160.0, cx: 64.0, cy: 64.0 },
            distortion: [0.0; 5],
            rotation: glam::Mat3::IDENTITY,
            translation: glam::Vec3::ZERO,
        });
        let pose = scancraft_align::rigid(glam::Quat::IDENTITY, glam::Vec3::new(0.0, -200.0, 20.0));
        let mut kfs = Vec::new();
        for i in 0..2u64 {
            let mut rgb_data = vec![30u8; 128 * 128 * 3];
            for y in 44..84 {
                for x in 44..84 {
                    let off = (y * 128 + x) * 3;
                    rgb_data[off] = 220;
                    rgb_data[off + 1] = 110;
                    rgb_data[off + 2] = 70;
                }
            }
            let frame = scancraft_core::ColorFrame {
                image: scancraft_core::RgbImage { width: 128, height: 128, channels: 3, data: rgb_data },
                jpeg: None,
                timestamp_ms: i * 100,
                camera: Some(cam.clone()),
            };
            kfs.push(scancraft_color::RgbKeyframe::new(i, &frame, cam.clone(), pose, 50.0, i).unwrap());
        }
        app.passes[0].rgb_keyframes = kfs;
        app.open_tool(Tool::AiSegment);
        assert_eq!(app.tool, Some(Tool::AiSegment));
        app.add_ai_pick(crate::ai_segment::ImagePick { x: 64.0, y: 64.0, positive: true });
        finish_job(&mut app);
        assert!(app.ai_picking.as_ref().unwrap().current_mask().is_some());
        app.propagate_ai_segment();
        finish_job(&mut app);
        let kept_preview = app.ai_picking.as_ref().unwrap().kept_count();
        assert!(kept_preview > 0 && kept_preview < total, "kept_preview={kept_preview}, total={total}");
        app.run_tool(Tool::AiSegment);
        assert!(app.passes[0].is_cleaned());
        assert_eq!(app.passes[0].layer.len(), kept_preview);
        assert!(app.tool.is_none() && app.ai_picking.is_none());
        app.undo();
        assert!(!app.passes[0].is_cleaned() && app.passes[0].layer.len() == total);
    }

    #[test]
    fn opened_project_with_rgb_keyframes_rebuilds_for_ai_segment() {
        let dir = temp_dir("ai_scene_open").join("desk-scene");
        let bp = scancraft_core::BackProjection::new(150.0, 420.0);
        let mut pts = Vec::new();
        let mut kfs = Vec::new();
        for i in 0..3u64 {
            let yaw = (i as f32) * 0.4;
            let frame = scancraft_driver::mock::MockDevice::render_synthetic_frame(160, 128, yaw, i * 100);
            let cam = frame.color.camera.clone().unwrap();
            let cloud = frame.depth.to_point_cloud_with(&bp);
            let mat = frame.sensor_pose.to_mat4();
            for mut p in cloud.points {
                p.position = mat.transform_point3(Vec3::from(p.position)).to_array();
                p.normal = (frame.sensor_pose.rotation * Vec3::from(p.normal)).normalize_or_zero().to_array();
                pts.push(p);
            }
            kfs.push(scancraft_color::RgbKeyframe::new(i, &frame.color, cam, frame.sensor_pose, 80.0, i).unwrap());
        }
        let fused = scancraft_fusion::VoxelFilter::new(1.5).filter(&PointCloud::from_points(pts));
        let pass = crate::project::StoredPass {
            id: 1,
            name: "Desk Scene".into(),
            frames: 3,
            visible: true,
            meta: crate::project::PassMeta {
                alignment: AlignState::Reference,
                origin: crate::project::PassOrigin::File { source: "synthetic-desk-scene".into() },
                ..crate::project::PassMeta::new(scancraft_driver::ScanConfig::default(), None, PassStart::Moved)
            },
            keyframes: Vec::new(),
            surface: Some(Arc::new(fused)),
            edit: None,
            rgb_keyframes: kfs,
            mask: None,
            markers: Vec::new(),
        };
        let stored = crate::project::StoredProject {
            name: "desk-scene".into(),
            reference: Some(1),
            next_pass_id: 2,
            passes: vec![pass],
            merged: None,
            cleaned: None,
            mesh: None,
            settings: serde_json::json!({}),
        };
        crate::project::save(&dir, &stored).unwrap();

        let mut app = app();
        app.open_project(dir.clone());
        finish_job(&mut app);
        assert_eq!(app.passes.len(), 1);
        assert!(app.passes[0].is_fixed());
        assert_eq!(app.passes[0].rgb_keyframes.len(), 3);
        let input = Arc::new(app.passes[0].source().editable().unwrap());
        let picking = crate::ai_segment::AiPicking::from_pass(
            app.passes[0].id,
            app.passes[0].name.clone(),
            input,
            app.passes[0].meta.transform,
            &app.passes[0].rgb_keyframes,
        )
        .unwrap();
        assert_eq!(picking.keyframes.len(), 3);
        let _ = std::fs::remove_dir_all(dir.parent().unwrap());
    }

    #[test]
    #[ignore = "requires SAM 2.1 models + ONNX Runtime DirectML/CPU binaries"]
    fn sam21_isolates_vase_in_multi_object_desk_scene() {
        let dir = temp_dir("ai_scene_sam21").join("desk-scene");
        let bp = scancraft_core::BackProjection::new(150.0, 420.0);
        let mut pts = Vec::new();
        let mut kfs = Vec::new();
        let views = 18usize;
        for i in 0..views {
            let yaw = (i as f32) * std::f32::consts::TAU / (views as f32);
            let frame = scancraft_driver::mock::MockDevice::render_synthetic_frame(640, 512, yaw, (i as u64) * 100);
            let cam = frame.color.camera.clone().unwrap();
            let cloud = frame.depth.to_point_cloud_with(&bp);
            let mat = frame.sensor_pose.to_mat4();
            for mut p in cloud.points {
                p.position = mat.transform_point3(Vec3::from(p.position)).to_array();
                p.normal = (frame.sensor_pose.rotation * Vec3::from(p.normal)).normalize_or_zero().to_array();
                pts.push(p);
            }
            kfs.push(scancraft_color::RgbKeyframe::new(i as u64, &frame.color, cam, frame.sensor_pose, 80.0, i as u64).unwrap());
        }
        let fused = scancraft_fusion::VoxelFilter::new(1.0).filter(&PointCloud::from_points(pts));
        let pass = crate::project::StoredPass {
            id: 1,
            name: "Desk Scene".into(),
            frames: views as u64,
            visible: true,
            meta: crate::project::PassMeta {
                alignment: AlignState::Reference,
                origin: crate::project::PassOrigin::File { source: "synthetic-desk-scene".into() },
                ..crate::project::PassMeta::new(scancraft_driver::ScanConfig::default(), None, PassStart::Moved)
            },
            keyframes: Vec::new(),
            surface: Some(Arc::new(fused)),
            edit: None,
            rgb_keyframes: kfs,
            mask: None,
            markers: Vec::new(),
        };
        crate::project::save(
            &dir,
            &crate::project::StoredProject {
                name: "desk-scene".into(),
                reference: Some(1),
                next_pass_id: 2,
                passes: vec![pass],
                merged: None,
                cleaned: None,
                mesh: None,
                settings: serde_json::json!({}),
            },
        )
        .unwrap();

        let mut app = app();
        app.refresh_ai_status();
        assert!(app.ai_status.ready, "AI Pack must be installed: {:?}", app.ai_status);
        app.open_project(dir.clone());
        finish_job(&mut app);

        let total = app.passes[0].layer.len();
        let (px, _) = app.passes[0].rgb_keyframes[0].project_world(Vec3::new(0.0, -30.0, 55.0)).unwrap();
        app.open_tool(Tool::AiSegment);
        app.add_ai_pick(crate::ai_segment::ImagePick { x: px.x, y: px.y, positive: true });
        finish_job(&mut app);
        app.propagate_ai_segment();
        finish_job(&mut app);
        app.run_tool(Tool::AiSegment);

        let kept = &app.passes[0].layer.cloud.points;
        let canister_hits = kept.iter().filter(|p| (p.position[0] - 66.0).hypot(p.position[1] - 14.0) < 22.0 && p.position[2] > 5.0).count();
        let block_hits = kept.iter().filter(|p| (p.position[0] + 62.0).hypot(p.position[1] + 14.0) < 22.0 && p.position[2] > 5.0).count();
        let turntable_hits = kept.iter().filter(|p| p.position[2].abs() < 2.0 && p.position[0].hypot(p.position[1]) > 48.0).count();
        eprintln!(
            "SAM 2.1 desk scene: kept={} / {} (canister_hits={canister_hits}, block_hits={block_hits}, turntable_hits={turntable_hits})",
            kept.len(),
            total
        );
        assert!(kept.len() > 20_000 && kept.len() < total / 2, "kept {} of {total}", kept.len());
        assert!(canister_hits < 50, "cobalt canister leaked {canister_hits} pts");
        assert!(block_hits < 500, "sage block leaked {block_hits} pts");
        assert!(turntable_hits < 50, "turntable leaked {turntable_hits} pts");
        let _ = std::fs::remove_dir_all(dir.parent().unwrap());
    }

    // ------------------------------------------------------------------ orientation

    /// A rigid `Transform` from a rotation and a translation.
    fn rt(rotation: glam::Quat, translation: Vec3) -> Transform {
        Transform { rotation, translation, ..Transform::default() }
    }

    /// The two transforms agree up to f32 noise.
    fn same_transform(a: Transform, b: Transform, eps: f32) -> bool {
        (a.to_mat4() - b.to_mat4()).to_cols_array().iter().map(|v| v.abs()).fold(0.0, f32::max) < eps
    }

    /// Surface samples (`spacing` apart) of a box with `half` extents centred on the origin,
    /// placed by `t`.
    fn box_points(half: Vec3, spacing: f32, t: Transform) -> Vec<Point3D> {
        let mut out = Vec::new();
        for axis in 0..3 {
            let (u, v) = ((axis + 1) % 3, (axis + 2) % 3);
            let nu = ((half[u] * 2.0 / spacing) as usize).max(1) + 1;
            let nv = ((half[v] * 2.0 / spacing) as usize).max(1) + 1;
            for sign in [-1.0f32, 1.0] {
                let mut n = Vec3::ZERO;
                n[axis] = sign;
                for i in 0..nu {
                    for j in 0..nv {
                        let mut p = Vec3::ZERO;
                        p[axis] = sign * half[axis];
                        p[u] = -half[u] + i as f32 * (2.0 * half[u] / (nu - 1) as f32);
                        p[v] = -half[v] + j as f32 * (2.0 * half[v] / (nv - 1) as f32);
                        out.push(Point3D { position: t.transform_point3(p).to_array(), normal: t.transform_vector3(n).to_array(), ..Point3D::new([0.0; 3]) });
                    }
                }
            }
        }
        out
    }

    /// The lateral surface of a cylinder of radius `r` running `z0..z1` on +Z, placed by `t`.
    fn cylinder_points(r: f32, z0: f32, z1: f32, spacing: f32, t: Transform) -> Vec<Point3D> {
        let mut out = Vec::new();
        let na = ((std::f32::consts::TAU * r / spacing) as usize).max(12);
        let nz = (((z1 - z0) / spacing) as usize).max(2);
        for i in 0..na {
            let a = i as f32 * std::f32::consts::TAU / na as f32;
            let n = Vec3::new(a.cos(), a.sin(), 0.0);
            for j in 0..=nz {
                let p = Vec3::new(r * a.cos(), r * a.sin(), z0 + (z1 - z0) * j as f32 / nz as f32);
                out.push(Point3D { position: t.transform_point3(p).to_array(), normal: t.transform_vector3(n).to_array(), ..Point3D::new([0.0; 3]) });
            }
        }
        out
    }

    #[test]
    fn moving_the_project_frame_moves_everything_placed_and_undoes() {
        let mut app = app();
        add_pass(&mut app, 1, AlignState::Reference);
        add_pass(&mut app, 2, AlignState::Aligned { method: AlignMethod::Auto, quality: None });
        add_pass(&mut app, 3, AlignState::Unaligned);
        // Pass 2 has a non-identity transform; its layer is placed by it as alignments leave it.
        let t2 = rt(glam::Quat::from_rotation_y(0.4), Vec3::new(1.0, 2.0, 3.0));
        app.passes[1].meta.transform = t2;
        let placed2 = PointCloud::from_points(crate::project::place(app.passes[1].layer.cloud.points.clone(), &t2));
        let g = app.generation();
        app.passes[1].layer = CloudLayer::new(placed2, g);
        // The unaligned pass keeps its own frame even when its transform is not identity.
        let t3 = rt(glam::Quat::IDENTITY, Vec3::new(9.0, 9.0, 9.0));
        app.passes[2].meta.transform = t3;
        let g = app.generation();
        app.merged = Some(MergedLayer::new(PointCloud::from_points(vec![Point3D::new([1.0, 1.0, 1.0])]), g, vec![1, 2], "test".into()));
        let mut mesh = Mesh::default();
        mesh.vertices = vec![Point3D::new([2.0, 2.0, 2.0]), Point3D::new([4.0, 2.0, 2.0]), Point3D::new([2.0, 4.0, 2.0])];
        let analysis = Mesher::analyze(&mesh);
        app.mesh = Some(MeshLayer::new(mesh, analysis, app.generation()));

        let t = rt(glam::Quat::from_rotation_z(0.6), Vec3::new(5.0, -3.0, 2.0));
        let p0 = app.passes[0].layer.cloud.points[0].position;
        let p2 = app.passes[1].layer.cloud.points[0].position;
        let p3 = app.passes[2].layer.cloud.points[0].position;
        let old: Vec<Transform> = app.passes.iter().map(|p| p.meta.transform).collect();
        // The pass-to-pass transform before the move.
        let rel_before = scancraft_align::compose(&old[1].inverse(), &old[0]);
        app.move_project_frame(t, "orient the project");

        let want0 = scancraft_align::compose(&t, &old[0]);
        let want1 = scancraft_align::compose(&t, &old[1]);
        assert_eq!(app.passes[0].meta.transform, want0);
        assert_eq!(app.passes[1].meta.transform, want1);
        assert!(same_transform(scancraft_align::compose(&want1.inverse(), &want0), rel_before, 1e-4), "the relative transform changed");
        assert_eq!(app.passes[2].meta.transform, t3, "the unaligned pass moved");
        let moved0 = Vec3::from(app.passes[0].layer.cloud.points[0].position);
        let moved2 = Vec3::from(app.passes[1].layer.cloud.points[0].position);
        assert!((moved0 - t.transform_point3(Vec3::from(p0))).length() < 1e-4);
        assert!((moved2 - t.transform_point3(Vec3::from(p2))).length() < 1e-4);
        assert_eq!(app.passes[2].layer.cloud.points[0].position, p3, "the unaligned pass's layer moved");
        let merged = app.merged.as_ref().expect("the merged model is kept");
        assert!((Vec3::from(merged.layer.cloud.points[0].position) - t.transform_point3(Vec3::new(1.0, 1.0, 1.0))).length() < 1e-4);
        let mesh = app.mesh.as_ref().expect("the mesh is kept");
        assert!((Vec3::from(mesh.mesh.vertices[0].position) - t.transform_point3(Vec3::new(2.0, 2.0, 2.0))).length() < 1e-4);

        app.undo();
        assert_eq!(app.passes[0].meta.transform, old[0]);
        assert_eq!(app.passes[1].meta.transform, old[1]);
        assert_eq!(app.passes[0].layer.cloud.points[0].position, p0);
        assert_eq!(app.passes[2].layer.cloud.points[0].position, p3);
        assert_eq!(app.merged.as_ref().unwrap().layer.cloud.points[0].position, [1.0, 1.0, 1.0]);
        assert_eq!(app.mesh.as_ref().unwrap().mesh.vertices[0].position, [2.0, 2.0, 2.0]);
        app.redo();
        assert_eq!(app.passes[0].meta.transform, want0);
        assert_eq!(app.passes[1].meta.transform, want1);
    }

    /// The app with one reference pass showing a tilted box with a cylinder standing on it.
    fn box_and_cylinder_app() -> (ScanCraftApp, Transform) {
        let mut app = app();
        add_pass(&mut app, 1, AlignState::Reference);
        let tilt = rt(glam::Quat::from_rotation_y(0.5) * glam::Quat::from_rotation_x(0.35), Vec3::new(3.0, -2.0, 7.0));
        let mut points = box_points(Vec3::new(30.0, 30.0, 15.0), 1.0, tilt);
        points.extend(cylinder_points(8.0, 15.0, 40.0, 0.9, tilt));
        let g = app.generation();
        app.passes[0].layer = CloudLayer::new(PointCloud::from_points(points), g);
        (app, tilt)
    }

    #[test]
    fn auto_orient_is_blocked_without_points_and_with_an_open_proposal() {
        let mut app = app();
        assert_eq!(app.orient_blocked(), Some("No points to orient: scan or import first"));
        add_pass(&mut app, 1, AlignState::Reference);
        assert_eq!(app.orient_blocked(), None);
        let g = app.generation();
        app.align_preview = Some(AlignPreview {
            pass: 1,
            method: AlignMethod::Auto,
            alignment: Alignment { transform: Transform::default(), quality: Quality::NONE },
            check: None,
            layer: CloudLayer::new(PointCloud::new(), g),
        });
        assert_eq!(app.orient_blocked(), Some("Accept or reject the proposed alignment first"));
    }

    #[test]
    fn auto_orient_proposes_reject_keeps_and_accept_squares_the_box() {
        let mut app = app();
        add_pass(&mut app, 1, AlignState::Reference);
        let tilt = rt(glam::Quat::from_rotation_y(0.6) * glam::Quat::from_rotation_x(0.35), Vec3::new(3.0, -2.0, 7.0));
        let g = app.generation();
        app.passes[0].layer = CloudLayer::new(PointCloud::from_points(box_points(Vec3::new(40.0, 25.0, 15.0), 1.2, tilt)), g);

        app.auto_orient();
        finish_job(&mut app);
        let preview = app.orient_preview.as_ref().expect("Auto Orient proposes an orientation");
        assert!(preview.result.turned_deg > 1.0, "the tilted box needs a real turn");
        assert_eq!(app.passes[0].meta.transform, Transform::default(), "the proposal moves nothing");
        let p0 = app.passes[0].layer.cloud.points[0].position;

        app.reject_orient();
        assert!(app.orient_preview.is_none());
        assert_eq!(app.passes[0].layer.cloud.points[0].position, p0, "reject left the model as it was");

        app.auto_orient();
        finish_job(&mut app);
        assert!(app.orient_preview.is_some());
        app.accept_orient();
        assert!(app.orient_preview.is_none());
        let mut min_z = f32::INFINITY;
        let cos_1_5_deg = 1.5f32.to_radians().cos();
        for p in &app.passes[0].layer.cloud.points {
            min_z = min_z.min(p.position[2]);
            let n = Vec3::from(p.normal).normalize();
            assert!(n.abs().max_element() > cos_1_5_deg, "face normal {n} is not on a world axis");
        }
        assert!(min_z.abs() < 0.3, "the base sits on the grid, min z {min_z}");
        assert_eq!(app.history.undo_label(), Some("orient the project"));
        app.undo();
        assert_eq!(app.passes[0].meta.transform, Transform::default());
        assert_eq!(app.passes[0].layer.cloud.points[0].position, p0);
    }

    #[test]
    fn datums_square_the_project_to_a_picked_plane_and_axis() {
        let (mut app, tilt) = box_and_cylinder_app();
        app.open_tool(Tool::Datums);
        assert!(app.datum_picking.is_some(), "the tool built its surface");
        // Click the box's bottom face (the whole model is on its other side) and the
        // cylinder's wall (axis).
        let base = tilt.transform_point3(Vec3::new(0.0, 0.0, -15.0));
        let wall = tilt.transform_point3(Vec3::new(8.0, 0.0, 27.0));
        app.datum_pick(base);
        app.set_datum_kind(DatumKind::Axis);
        app.datum_pick(wall);
        let d = app.datum_picking.as_ref().unwrap();
        assert!(d.plane.is_some() && d.axis.is_some(), "both datums fitted");
        let frame = d.frame.as_ref().expect("two datums give a frame").as_ref().expect("the choices do not conflict");
        // The cylinder's axis goes onto Z.
        let axis_dir = match d.axis.as_ref().unwrap().1.primitive {
            scancraft_align::Primitive::Cylinder { axis, .. } => axis,
            _ => panic!("the axis datum is a cylinder"),
        };
        let moved = frame.transform.transform_vector3(axis_dir).normalize();
        assert!(moved.dot(Vec3::Z).abs() > 0.5f32.to_radians().cos(), "axis {moved} not within 0.5 deg of Z");
        assert!(app.datum_apply_blocked().is_none());

        app.apply_datums();
        // The picked face lies on the floor and the whole model sits above it.
        let mut min_z = f32::INFINITY;
        for p in &app.passes[0].layer.cloud.points {
            min_z = min_z.min(p.position[2]);
            assert!(p.position[2] > -0.5, "a point went below the floor");
        }
        assert!(min_z.abs() < 0.3, "the picked face sits on z = 0, min z {min_z}");
        assert!(app.datum_picking.is_none() && app.tool.is_none(), "Apply closed the tool");
    }

    #[test]
    fn a_conflicting_datum_choice_cannot_be_applied() {
        let (mut app, tilt) = box_and_cylinder_app();
        app.open_tool(Tool::Datums);
        app.datum_pick(tilt.transform_point3(Vec3::new(0.0, 0.0, -15.0)));
        app.set_datum_kind(DatumKind::Axis);
        app.datum_pick(tilt.transform_point3(Vec3::new(8.0, 0.0, 27.0)));
        // The axis is parallel to the plane's normal: it cannot lie in the plane.
        app.set_axis_onto(scancraft_align::WorldAxis::X);
        let d = app.datum_picking.as_ref().unwrap();
        assert!(matches!(d.frame, Some(Err(scancraft_align::DatumError::Conflict))), "{:?}", d.frame.as_ref().map(|r| r.as_ref().map(|_| ())));
        assert!(app.datum_apply_blocked().is_some());
        let before = app.passes[0].meta.transform;
        app.apply_datums();
        assert_eq!(app.passes[0].meta.transform, before, "a conflicting frame still applied");
    }

    /// An imported pass (a stored surface, unaligned) with a few landmark points.
    fn add_imported(app: &mut ScanCraftApp, id: u32, points: Vec<Point3D>) {
        add_pass(app, id, AlignState::Unaligned);
        let g = app.generation();
        let p = app.passes.iter_mut().find(|p| p.id == id).unwrap();
        p.meta = crate::project::PassMeta::imported("part.ply".into());
        p.surface = Some(Arc::new(PointCloud::from_points(points.clone())));
        p.layer = CloudLayer::new(PointCloud::from_points(points), g);
    }

    #[test]
    fn scale_from_lengths_doubles_the_imported_surface() {
        let mut app = app();
        let points = vec![Point3D::new([0.0, 0.0, 0.0]), Point3D::new([10.0, 0.0, 0.0]), Point3D::new([0.0, 12.0, 0.0]), Point3D::new([10.0, 12.0, 0.0])];
        add_imported(&mut app, 1, points);
        add_pass(&mut app, 2, AlignState::Unaligned);
        assert_eq!(app.scale_blocked(Some(2)), Some("Only an imported pass can be scaled: a captured pass has its true size"));
        assert_eq!(app.scale_blocked(Some(1)), None);
        assert_eq!(app.scale_target(), Some(1));

        app.open_tool(Tool::Scale(1));
        assert_eq!(app.scale_apply_blocked(), Some("Pick two points first"));
        app.scale_pick(Vec3::new(0.0, 0.0, 0.0));
        app.scale_pick(Vec3::new(10.0, 0.0, 0.0));
        app.set_scale_true(0, 20.0);
        app.scale_pick(Vec3::new(0.0, 0.0, 0.0));
        app.scale_pick(Vec3::new(0.0, 12.0, 0.0));
        app.set_scale_true(1, 24.0);
        let estimate = app.scale_estimate().expect("two lengths give a factor");
        assert!((estimate.factor - 2.0).abs() < 1e-3, "factor {}", estimate.factor);
        assert_eq!(app.scale_apply_blocked(), None);

        app.apply_scale(1);
        let surface = app.passes[0].surface.as_ref().expect("the imported surface is kept");
        let size = surface.bbox.max - surface.bbox.min;
        assert!((size.x - 20.0).abs() < 1e-3 && (size.y - 24.0).abs() < 1e-3, "bbox {size:?}");
        assert!(app.scale_picking.is_none() && app.tool.is_none());
    }

    #[test]
    fn point_pairs_report_nearly_collinear_and_degenerate_picks() {
        let t = rt(glam::Quat::from_rotation_z(0.5), Vec3::new(10.0, 0.0, 0.0));
        let place = |src: &[Vec3]| src.iter().map(|&p| (p, t.transform_point3(p))).collect::<Vec<_>>();
        // Nearly on a line (90 mm long, 3.3 mm off it): fits, but the card warns.
        let warn = place(&[Vec3::new(0.0, 0.0, 0.0), Vec3::new(30.0, 5.0, 0.0), Vec3::new(60.0, 0.0, 0.0), Vec3::new(90.0, 5.0, 0.0)]);
        let fit = scancraft_align::fit_pairs(&warn).expect("nearly collinear points still fit");
        assert!(!fit.spread.degenerate());
        assert!(fit.spread.nearly_collinear());
        // Exactly on a line: no fit at all.
        let line = place(&(0..4).map(|i| Vec3::new(i as f32 * 30.0, 0.0, 0.0)).collect::<Vec<_>>());
        assert_eq!(scancraft_align::fit_pairs(&line).unwrap_err(), scancraft_align::AlignError::Degenerate);
        assert_eq!(scancraft_align::fit_pairs(&warn[..2]).unwrap_err(), scancraft_align::AlignError::TooFewPairs);
    }
}
