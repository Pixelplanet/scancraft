//! Application state and the actions behind every control: connecting, live capture, the
//! clean/mesh/export jobs, undo, and the viewport camera.

use std::collections::HashMap;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use eframe::egui::{self, TextureHandle, TextureOptions};
use scancraft_core::{BoundingBox, PointCloud};
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
use crate::project::{self, AlignMethod, AlignState, PassMeta, PassStart, ProjectInfo};
use crate::scan_modes::{self, Choice};
use crate::worker::{self, WorkerConfig};

#[path = "app_project.rs"]
mod project_ops;

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
    pub mc_cell_mm: f32,
    pub smooth_iterations: usize,
    pub smooth_lambda: f32,
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
    /// Scan file for Import Scan, and project folder for Add Project.
    pub import_path: String,
    pub add_project_path: String,
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
            mc_cell_mm: 1.0,
            smooth_iterations: 2,
            smooth_lambda: 0.35,
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
        }
    }
}

/// A path typed or pasted into a field, without surrounding spaces and quotes (Explorer's "Copy as
/// path" adds quotes).
fn trim_quotes(path: &str) -> String {
    path.trim().trim_matches('"').trim().to_string()
}

fn default_project_path() -> String {
    std::env::var("USERPROFILE")
        .or_else(|_| std::env::var("HOME"))
        .map(|h| std::path::Path::new(&h).join("Documents").join("ScanCraft").join("Untitled project").display().to_string())
        .unwrap_or_else(|_| "Untitled project".to_string())
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
    pub inset_depth: bool,
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
    pub merged: Option<MergedLayer>,
    pub cleaned: Option<CloudLayer>,
    pub mesh: Option<MeshLayer>,
    pub history: History,
    pub settings: Settings,
    pub project: ProjectInfo,
    /// How the next new pass starts (object moved, or not touched).
    pub next_start: PassStart,
    pub align_preview: Option<AlignPreview>,
    /// Align All or Refine All waiting for Accept / Reject.
    pub poses_preview: Option<PosesPreview>,
    pub picking: Option<PairPicking>,
    /// The Isolate Object tool's picks and result.
    pub object_picking: Option<ObjectPicking>,
    /// The undo snapshot of the running job, pushed when it succeeds.
    pending_undo: Option<Snapshot>,
    /// A message shown over the view for a few seconds.
    pub toast: Option<(String, Instant)>,
    /// What the toast offers besides Undo.
    pub toast_action: Option<ToastAction>,
    /// Dropped scan files and project folders still to add, one job each.
    pending_imports: std::collections::VecDeque<std::path::PathBuf>,
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
    /// The selected keyframe's points, drawn over the model.
    pub highlight: Option<Highlight>,
    /// When the tracker last reported a re-location.
    pub relocated_at: Option<Instant>,
    /// The last frame's markers (or other features) while capturing.
    pub overlay: Option<scancraft_pipeline::ImageOverlay>,
    /// Frame rates reported by the scan worker while capturing.
    pub rates: Option<FrameRates>,
    /// The last marker map seen per pass (pass frame, mm), shown when the pass is selected.
    pub marker_maps: HashMap<u32, Vec<([f32; 3], bool)>>,
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
            inset_depth: false,
            show_inset: true,
            distance_mm: None,
            tracking: None,
            fps: FpsMeter::default(),
            budget_reached: false,
            passes: Vec::new(),
            active_pass: None,
            next_pass_id: 1,
            next_generation: 1,
            merged: None,
            cleaned: None,
            mesh: None,
            history: History::default(),
            settings: Settings::default(),
            project: ProjectInfo::default(),
            next_start: PassStart::Moved,
            align_preview: None,
            poses_preview: None,
            picking: None,
            object_picking: None,
            pending_undo: None,
            toast: None,
            toast_action: None,
            pending_imports: Default::default(),
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
            highlight: None,
            relocated_at: None,
            overlay: None,
            rates: None,
            marker_maps: HashMap::new(),
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

    pub fn say(&mut self, tone: Tone, message: impl Into<String>) {
        self.tone = tone;
        self.status = message.into();
    }

    /// A message over the view for a few seconds.
    pub fn show_toast(&mut self, message: impl Into<String>) {
        self.toast = Some((message.into(), Instant::now()));
        self.toast_action = None;
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
        match action {
            ToastAction::MetresToMm(id) => self.scale_pass(id, 1000.0),
        }
    }

    /// Drops the cleaned cloud and the mesh: they were made from passes that have since moved
    /// (or from an older merge), and cleaning or meshing must not start from them. The caller has
    /// pushed an undo snapshot that still holds them. Returns whether anything was dropped.
    pub fn invalidate_derived(&mut self, reason: &str) -> bool {
        let dropped: Vec<&str> = [self.cleaned.is_some().then_some("cleaned cloud"), self.mesh.is_some().then_some("mesh")].into_iter().flatten().collect();
        self.cleaned = None;
        self.mesh = None;
        // An open Isolate Object tool works on a copy of the old cloud: its object must not
        // come back as the cleaned cloud.
        if self.object_picking.take().is_some() && self.tool == Some(Tool::IsolateObject) {
            self.tool = None;
        }
        if matches!(self.selection, Some(Node::Cleaned | Node::Mesh)) {
            self.selection = None;
        }
        if dropped.is_empty() {
            return false;
        }
        self.show_toast(format!("{reason}: removed the {}, which no longer match. Undo restores them.", dropped.join(" and ")));
        true
    }

    /// Forgets every proposal waiting for a decision.
    pub fn clear_previews(&mut self) {
        self.align_preview = None;
        self.poses_preview = None;
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
        if !self.passes.iter().any(|p| p.id == pass && !p.is_fixed()) {
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
                    self.connection = Some(conn);
                    self.scan_config.light = choice.light;
                    self.scan_config.tracking = choice.tracking;
                    self.scan_config.object = choice.object;
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
                            (pass.id, pass.tracker.clone(), "Continuing")
                        }
                        None => {
                            let id = self.next_pass_id;
                            self.next_pass_id += 1;
                            let generation = self.generation();
                            let tracker = worker::new_tracker(&config);
                            let device = self.connection.as_ref().map(|c| c.info.clone());
                            let first = !self.passes.iter().any(|p| p.meta.alignment.is_placed());
                            let mut meta = PassMeta::new(scan_config.clone(), device, if first { PassStart::Moved } else { self.next_start });
                            meta.alignment = if first {
                                AlignState::Reference
                            } else if meta.start == PassStart::SamePosition && self.seed_tracker(&tracker) {
                                AlignState::Locating
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
                            });
                            (id, tracker, "Capturing")
                        }
                    };
                    self.active_pass = Some(id);
                    self.selection = Some(Node::Pass(id));
                    self.paused_flag = Arc::new(AtomicBool::new(false));
                    let (tx, urx) = mpsc::channel(4);
                    self.updates = Some(urx);
                    self.fps.clear();
                    self.rt.spawn(worker::scan_worker(rx, tx, config, scan_config, tracker, self.paused_flag.clone()));
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
                    for map in self.marker_maps.values_mut() {
                        for m in map.iter_mut() {
                            m.1 = false;
                        }
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
                if let Some(id) = self.active_pass {
                    if !o.map.is_empty() {
                        self.marker_maps.insert(id, o.map.clone());
                    }
                }
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

    /// The passes cleaning and meshing work on: the visible ones, or all when none is visible
    /// (a result hides the passes it was made from).
    pub fn input_passes(&self) -> impl Iterator<Item = &ScanPass> {
        let any_visible = self.passes.iter().any(|p| p.layer.visible);
        self.passes.iter().filter(move |p| p.layer.visible || !any_visible)
    }

    /// The visible passes put together: the input to cleaning when there is neither a merged
    /// model nor a cleaned cloud.
    pub fn merged_passes(&self) -> PointCloud {
        let mut points = Vec::with_capacity(self.input_passes().map(|p| p.layer.len()).sum());
        for p in self.input_passes() {
            points.extend_from_slice(&p.layer.cloud.points);
        }
        PointCloud::from_points(points)
    }

    /// The cloud the next clean or mesh operation works on: the cleaned cloud, else the merged
    /// model, else the visible passes.
    pub fn working_cloud(&self) -> PointCloud {
        match (&self.cleaned, &self.merged) {
            (Some(c), _) => c.cloud.clone(),
            (None, Some(m)) => m.layer.cloud.clone(),
            (None, None) => self.merged_passes(),
        }
    }

    pub fn working_len(&self) -> usize {
        match (&self.cleaned, &self.merged) {
            (Some(c), _) => c.len(),
            (None, Some(m)) => m.layer.len(),
            (None, None) => self.input_passes().map(|p| p.layer.len()).sum(),
        }
    }

    /// What [`working_cloud`](Self::working_cloud) is, for labels.
    pub fn working_source(&self) -> &'static str {
        match (&self.cleaned, &self.merged) {
            (Some(_), _) => "Cleaned cloud",
            (None, Some(_)) => "Merged model",
            (None, None) => "Scan passes",
        }
    }

    pub fn data_bbox(&self) -> Option<BoundingBox> {
        let mut boxes: Vec<BoundingBox> = self.passes.iter().filter(|p| p.layer.visible && p.layer.len() > 0).map(|p| p.layer.cloud.bbox).collect();
        if let Some(m) = self.merged.as_ref().filter(|m| m.layer.visible && m.layer.len() > 0) {
            boxes.push(m.layer.cloud.bbox);
        }
        if let Some(c) = self.cleaned.as_ref().filter(|c| c.visible && c.len() > 0) {
            boxes.push(c.cloud.bbox);
        }
        if let Some(m) = self.mesh.as_ref().filter(|m| m.visible) {
            boxes.push(m.mesh.bbox);
        }
        union_bbox(boxes.iter())
    }

    pub fn has_geometry(&self) -> bool {
        self.total_points() > 0 || self.merged.is_some() || self.cleaned.is_some() || self.mesh.is_some()
    }

    fn snapshot(&self, label: &str) -> Snapshot {
        Snapshot {
            merged: self.merged.as_ref().map(|m| (m.layer.cloud.clone(), m.passes.clone(), m.summary.clone())),
            cleaned: self.cleaned.as_ref().map(|c| c.cloud.clone()),
            mesh: self.mesh.as_ref().map(|m| (m.mesh.clone(), m.analysis.clone())),
            passes: self
                .passes
                .iter()
                .map(|p| PassSnapshot {
                    id: p.id,
                    name: p.name.clone(),
                    cloud: p.layer.cloud.clone(),
                    frames: p.frames,
                    tracker: p.tracker.clone(),
                    keyframes: p.keyframes.clone(),
                    meta: p.meta.clone(),
                    mask: p.mask.clone(),
                    surface: p.surface.clone(),
                })
                .collect(),
            label: label.to_string(),
        }
    }

    fn restore(&mut self, s: Snapshot) {
        self.clear_previews();
        self.picking = None;
        self.object_picking = None;
        if self.tool == Some(Tool::IsolateObject) {
            self.tool = None;
        }
        let merged_visible = self.merged.as_ref().is_none_or(|m| m.layer.visible);
        self.merged = s.merged.map(|(cloud, passes, summary)| {
            let g = self.generation();
            let mut m = MergedLayer::new(cloud, g, passes, summary);
            m.layer.visible = merged_visible;
            m
        });
        let cleaned_visible = self.cleaned.as_ref().is_none_or(|c| c.visible);
        let mesh_visible = self.mesh.as_ref().is_none_or(|m| m.visible);
        self.cleaned = s.cleaned.map(|c| {
            let g = self.generation();
            let mut layer = CloudLayer::new(c, g);
            layer.visible = cleaned_visible;
            layer
        });
        self.mesh = s.mesh.map(|(m, a)| {
            let g = self.generation();
            let mut layer = MeshLayer::new(m, a, g);
            layer.visible = mesh_visible;
            layer
        });
        let visible: HashMap<u32, bool> = self.passes.iter().map(|p| (p.id, p.layer.visible)).collect();
        // Trackers whose discard flags differ from the restored ones are brought back in line.
        let mut resync = Vec::new();
        self.passes = s
            .passes
            .into_iter()
            .map(|p| {
                let g = self.next_generation + 1;
                self.next_generation = g;
                let mut layer = CloudLayer::new(p.cloud, g);
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
                ScanPass { id: p.id, name: p.name, layer, frames: p.frames, tracker: p.tracker, keyframes: p.keyframes, meta: p.meta, tinted: None, mask: p.mask, surface: p.surface }
            })
            .collect();
        match self.selection {
            Some(Node::Pass(id)) | Some(Node::Frame(id, _)) if !self.passes.iter().any(|p| p.id == id) => self.selection = None,
            _ => {}
        }
        self.highlight = None;
        for (pass, tracker, flags) in resync {
            self.spawn_tracker_job("Restoring frames", pass, tracker, move |t| {
                let flags: Vec<(u64, bool)> = flags.into_iter().collect();
                t.set_discarded_many(&flags);
                "Restored the frames".to_string()
            });
        }
    }

    pub fn can_edit(&self) -> bool {
        self.capture == Capture::Idle && self.busy.is_none()
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
        let Some(tracker) = self.passes.iter().find(|p| p.id == pass).map(|p| p.tracker.clone()) else { return };
        let verb = if discarded { "Discarded" } else { "Restored" };
        self.history.push(self.snapshot(&format!("{} frame {frame}", if discarded { "discard" } else { "restore" })));
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
        let tracker = p.tracker.clone();
        let n = p.keyframes.iter().filter(|k| k.id >= frame && !k.discarded).count();
        self.history.push(self.snapshot(&format!("discard frames from {frame}")));
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
        let g = self.generation();
        self.highlight = Some(Highlight { pass, frame, discarded: is_discarded, layer: CloudLayer::new(PointCloud::from_points(points), g) });
    }

    pub fn undo(&mut self) {
        if !self.can_edit() {
            return;
        }
        let label = self.history.undo_label().unwrap_or_default().to_string();
        let current = self.snapshot(&label);
        if let Some(prev) = self.history.undo(current) {
            self.restore(prev);
            self.say(Tone::Normal, format!("Undid {label}"));
        }
    }

    pub fn redo(&mut self) {
        if !self.can_edit() {
            return;
        }
        let label = self.history.redo_label().unwrap_or_default().to_string();
        let current = self.snapshot(&label);
        if let Some(next) = self.history.redo(current) {
            self.restore(next);
            self.say(Tone::Normal, format!("Redid {label}"));
        }
    }

    pub fn delete_pass(&mut self, id: u32) {
        if !self.can_edit() {
            return;
        }
        let name = self.passes.iter().find(|p| p.id == id).map(|p| p.name.clone()).unwrap_or_default();
        self.history.push(self.snapshot(&format!("Delete {name}")));
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

    pub fn discard_cleaned(&mut self) {
        if !self.can_edit() || self.cleaned.is_none() {
            return;
        }
        self.history.push(self.snapshot("Discard cleaned cloud"));
        self.cleaned = None;
        if self.selection == Some(Node::Cleaned) {
            self.selection = None;
        }
        self.say(Tone::Normal, "Discarded the cleaned cloud; cleaning starts again from the scan passes");
    }

    pub fn discard_mesh(&mut self) {
        if !self.can_edit() || self.mesh.is_none() {
            return;
        }
        self.history.push(self.snapshot("Delete mesh"));
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
        self.history.push(self.snapshot("Clear session"));
        self.passes.clear();
        self.clear_previews();
        self.picking = None;
        self.object_picking = None;
        self.merged = None;
        self.cleaned = None;
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
            Tool::RemovePlane | Tool::Outliers | Tool::Downsample | Tool::Crop | Tool::Reconstruct => {
                (self.working_len() == 0).then_some("No points to work on: scan first")
            }
            Tool::IsolateObject => {
                if self.object_picking.is_none() && self.working_len() == 0 {
                    Some("No points to work on: scan first")
                } else if self.object_picking.as_ref().is_some_and(|o| !o.current()) {
                    Some("Finding the object…")
                } else {
                    None
                }
            }
            Tool::Smooth | Tool::Decimate => self.mesh.is_none().then_some("Reconstruct a surface first"),
            Tool::PointPairs(id) => self.pairs_blocked(id),
            Tool::SaveProject => (!self.has_geometry()).then_some("Nothing to save yet"),
            Tool::OpenProject | Tool::ImportScan | Tool::AddProject => None,
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
        if tool != Tool::IsolateObject && !tool.is_file() {
            self.object_picking = None;
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
        self.tool = None;
        self.picking = None;
        self.object_picking = None;
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
        matches!(result, JobResult::Cloud { .. } | JobResult::Mesh { .. } | JobResult::Merged { .. } | JobResult::Imported(_) | JobResult::AddedProject { .. })
    }

    /// The cleaned cloud is replaced (a filter's result, or the isolated object).
    fn set_cleaned(&mut self, cloud: PointCloud, message: String) {
        let g = self.generation();
        self.cleaned = Some(CloudLayer::new(cloud, g));
        // The cleaned cloud replaces the passes and the merged model on screen.
        for p in &mut self.passes {
            p.layer.visible = false;
        }
        if let Some(m) = &mut self.merged {
            m.layer.visible = false;
        }
        self.selection = Some(Node::Cleaned);
        self.say(Tone::Success, message);
    }

    // ------------------------------------------------------------------ isolate object

    /// Opens the Isolate Object tool on the working cloud and finds the object without picks.
    fn start_object_picking(&mut self) -> bool {
        if self.object_picking.is_some() {
            return true;
        }
        let input = self.working_cloud();
        if input.is_empty() || self.busy.is_some() {
            return false;
        }
        let g = self.generation();
        self.object_picking = Some(ObjectPicking::new(input, g));
        self.segment_picks();
        true
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

    /// The Isolate Object tool's OK: its object becomes the cleaned cloud (undoable).
    fn commit_object(&mut self) {
        let Some(object) = self.object_picking.as_ref().filter(|o| o.current()).and_then(|o| o.object()) else {
            self.say(Tone::Warning, "No object found: click a cluster of points to keep");
            return;
        };
        let before = self.object_picking.as_ref().map_or(0, |o| o.input.len());
        self.history.push(self.snapshot("Isolate object"));
        let message = format!("Kept the object: {} of {} points", thousands(object.len()), thousands(before));
        self.set_cleaned(object, message);
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
            Tool::SaveProject => {
                let path = self.settings.project_path.trim().to_string();
                self.save_project(std::path::PathBuf::from(path));
                return;
            }
            Tool::OpenProject => {
                let path = self.settings.project_path.trim().to_string();
                self.open_project(std::path::PathBuf::from(path));
                return;
            }
            Tool::ImportScan => {
                let path = trim_quotes(&self.settings.import_path);
                self.import_scan(std::path::PathBuf::from(path));
                return;
            }
            Tool::AddProject => {
                let path = trim_quotes(&self.settings.add_project_path);
                self.add_project(std::path::PathBuf::from(path));
                return;
            }
            Tool::IsolateObject => {
                self.commit_object();
                return;
            }
            Tool::RemovePlane => {
                let input = self.working_cloud();
                let tolerance = s.plane_tolerance_mm;
                self.spawn_undoable("Removing the dominant plane", "Remove plane", move || {
                    let before = input.len();
                    let (cleaned, plane) = RansacPlaneFilter::new(tolerance, 150, PlaneFilterMode::ClipBelow).filter(&input);
                    match plane {
                        Some(_) => JobResult::Cloud { message: format!("Removed {} plane points", thousands(before - cleaned.len())), cloud: cleaned },
                        None => JobResult::NoChange("No dominant plane found".into()),
                    }
                });
            }
            Tool::Outliers => {
                let input = self.working_cloud();
                let (k, sd) = (s.sor_neighbors, s.sor_std_dev);
                self.spawn_undoable("Removing outliers", "Remove outliers", move || {
                    let before = input.len();
                    let cleaned = StatisticalOutlierFilter::new(k, sd).filter(&input);
                    JobResult::Cloud { message: format!("Removed {} outliers", thousands(before - cleaned.len())), cloud: cleaned }
                });
            }
            Tool::Downsample => {
                let input = self.working_cloud();
                let voxel = s.voxel_mm;
                self.spawn_undoable("Downsampling", "Downsample", move || {
                    let before = input.len();
                    let fused = VoxelFilter::new(voxel).filter(&input);
                    JobResult::Cloud { message: format!("Downsampled {} → {} points", thousands(before), thousands(fused.len())), cloud: fused }
                });
            }
            Tool::Crop => {
                let input = self.working_cloud();
                let bbox = BoundingBox::from_arrays(s.crop_min, s.crop_max);
                self.spawn_undoable("Cropping", "Crop", move || {
                    let before = input.len();
                    let cropped = RoiCropFilter::new(bbox).filter(&input);
                    JobResult::Cloud { message: format!("Cropped away {} points", thousands(before - cropped.len())), cloud: cropped }
                });
            }
            Tool::Reconstruct => {
                let input = self.working_cloud();
                let cell = s.mc_cell_mm;
                self.spawn_undoable("Reconstructing the surface", "Reconstruct surface", move || {
                    let mesh = Mesher::reconstruct(&input, cell);
                    if mesh.triangles.is_empty() {
                        return JobResult::Failed("Reconstruction produced no surface; try a larger cell size".into());
                    }
                    let analysis = Mesher::analyze(&mesh);
                    JobResult::Mesh { message: format!("Reconstructed {} triangles", thousands(analysis.triangle_count)), mesh, analysis }
                });
            }
            Tool::Smooth => {
                let Some(mut mesh) = self.mesh.as_ref().map(|m| m.mesh.clone()) else { return };
                let (it, lambda) = (s.smooth_iterations, s.smooth_lambda);
                self.spawn_undoable("Smoothing", "Smooth", move || {
                    Mesher::smooth(&mut mesh, it, lambda);
                    let analysis = Mesher::analyze(&mesh);
                    JobResult::Mesh { mesh, analysis, message: format!("Smoothed ({it} iterations)") }
                });
            }
            Tool::Decimate => {
                let Some(mesh) = self.mesh.as_ref().map(|m| m.mesh.clone()) else { return };
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
                let mesh = self.mesh.as_ref().map(|m| m.mesh.clone());
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
        while let Ok(result) = self.job_rx.try_recv() {
            self.busy = None;
            // The state before the job goes on the undo stack only if the job changed it: a job
            // that failed or changed nothing leaves the history as it was.
            if let Some(snapshot) = self.pending_undo.take().filter(|_| Self::changes_geometry(&result)) {
                self.history.push(snapshot);
            }
            match result {
                JobResult::Cloud { cloud, message } => self.set_cleaned(cloud, message),
                JobResult::NoChange(message) => self.say(Tone::Warning, message),
                JobResult::Mesh { mesh, analysis, message } => {
                    let g = self.generation();
                    self.mesh = Some(MeshLayer::new(mesh, analysis, g));
                    if let Some(c) = &mut self.cleaned {
                        c.visible = false;
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
                        let visible = p.layer.visible;
                        p.layer = CloudLayer::new(PointCloud::from_points(project::place(points, &p.meta.transform)), g);
                        p.layer.visible = visible;
                        p.keyframes = keyframes;
                    }
                    self.say(Tone::Success, message);
                }
                JobResult::Aligned { pass, method, alignment, check, message } => self.show_preview(pass, method, alignment, check, message),
                JobResult::AlignedAll(outcome) => self.show_align_all(*outcome),
                JobResult::Refined(outcome) => self.show_refinement(*outcome),
                JobResult::Isolated { segmentation, picks } => {
                    let g = self.generation();
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
                JobResult::Merged { cloud, passes, message } => {
                    // Cleaning and meshing start again from the new model.
                    self.invalidate_derived("New merged model");
                    let g = self.generation();
                    self.merged = Some(MergedLayer::new(cloud, g, passes, message.clone()));
                    for p in &mut self.passes {
                        p.layer.visible = false;
                    }
                    if let Some(c) = &mut self.cleaned {
                        c.visible = false;
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
                }
                JobResult::Opened(opened) => self.install_project(*opened),
                JobResult::Imported(scan) => self.install_imported(*scan),
                JobResult::AddedProject { name, passes } => self.install_added(name, passes),
                JobResult::Exported(message) => self.say(Tone::Success, message),
                JobResult::Failed(message) => self.say(Tone::Error, message),
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
    use scancraft_align::{Alignment, Quality};
    use scancraft_core::{Mesh, Point3D, Transform};

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
        });
    }

    /// The app with two passes, a cleaned cloud and a mesh.
    fn with_results() -> ScanCraftApp {
        let mut app = app();
        add_pass(&mut app, 1, AlignState::Reference);
        add_pass(&mut app, 2, AlignState::Unaligned);
        let g = app.generation();
        app.cleaned = Some(CloudLayer::new(PointCloud::from_points(vec![Point3D::new([1.0, 2.0, 3.0])]), g));
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
    fn accepting_an_alignment_drops_stale_results_and_undo_restores_them() {
        let mut app = with_results();
        let g = app.generation();
        let quality = Quality { overlap: 0.5, rmse_mm: 0.2, visibility_conflict: 0.0, inliers: 1000, ..Quality::NONE };
        let transform = scancraft_align::rigid(glam::Quat::from_rotation_z(0.3), glam::Vec3::new(5.0, 0.0, 0.0));
        app.align_preview = Some(AlignPreview { pass: 2, method: AlignMethod::Auto, alignment: Alignment { transform, quality }, check: None, layer: CloudLayer::new(PointCloud::new(), g) });
        app.accept_preview();
        assert!(app.cleaned.is_none() && app.mesh.is_none(), "the cleaned cloud and mesh were made before the pass moved");
        assert!(app.working_source() == "Scan passes");
        assert!(app.current_toast().is_some_and(|t| t.contains("Undo restores them")));
        app.undo();
        assert!(app.cleaned.is_some() && app.mesh.is_some());
        assert_eq!(app.passes[1].meta.alignment, AlignState::Unaligned);
    }

    #[test]
    fn moving_a_pass_closes_an_open_isolate_object_tool() {
        let mut app = with_results();
        let input = app.working_cloud();
        let g = app.generation();
        app.object_picking = Some(ObjectPicking::new(input, g));
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
    fn a_new_merge_replaces_the_cleaned_cloud_and_mesh() {
        let mut app = with_results();
        app.passes[1].meta.alignment = AlignState::Aligned { method: AlignMethod::Auto, quality: None };
        app.spawn_undoable("Merging passes", "merge", || JobResult::Merged {
            cloud: PointCloud::from_points(vec![Point3D::new([0.0, 0.0, 0.0])]),
            passes: vec![1, 2],
            message: "Merged".into(),
        });
        finish_job(&mut app);
        assert!(app.merged.is_some());
        assert!(app.cleaned.is_none() && app.mesh.is_none());
        assert_eq!(app.working_source(), "Merged model");
        // One undo step brings back the state before the merge.
        assert_eq!(app.history.undo_label(), Some("merge"));
        app.undo();
        assert!(app.merged.is_none() && app.cleaned.is_some() && app.mesh.is_some());
    }

    #[test]
    fn a_failed_job_leaves_no_undo_step() {
        let mut app = with_results();
        app.spawn_undoable("Merging passes", "merge", || JobResult::Failed("tracker unavailable".into()));
        finish_job(&mut app);
        assert!(!app.history.can_undo(), "the failed merge left an undo step");
        assert!(app.cleaned.is_some(), "a failed job must not touch the results");
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
        assert!(app.cleaned.is_none() && app.mesh.is_none());
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
        assert_eq!(app.history.undo_label(), Some("import part.ply"));
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
        assert_eq!(app.history.undo_label(), Some("add project Demo"));
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

    #[test]
    fn isolate_object_keeps_the_object_as_the_cleaned_cloud() {
        let mut app = app();
        // A table with a block on it, as one cleaned cloud.
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
        let total = points.len();
        let g = app.generation();
        app.cleaned = Some(CloudLayer::new(PointCloud::from_points(points), g));
        app.open_tool(Tool::IsolateObject);
        assert_eq!(app.tool, Some(Tool::IsolateObject));
        finish_job(&mut app);
        assert!(app.object_picking.as_ref().is_some_and(|o| o.current()));
        // A pick on the block keeps it.
        app.add_object_pick(glam::Vec3::new(0.0, 0.0, 40.0), true);
        finish_job(&mut app);
        app.run_tool(Tool::IsolateObject);
        let kept = app.cleaned.as_ref().map(|c| c.len()).unwrap_or(0);
        assert!(kept > 5000 && kept < total, "kept {kept} of {total}");
        assert!(app.cleaned.as_ref().unwrap().cloud.points.iter().all(|p| p.position[0].abs() < 25.0 && p.position[1].abs() < 25.0));
        assert!(app.tool.is_none() && app.object_picking.is_none());
        app.undo();
        assert_eq!(app.cleaned.as_ref().map(|c| c.len()), Some(total));
    }
}
