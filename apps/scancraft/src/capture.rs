//! What a captured pass records about how it was made, so the scan can be decoded, tracked,
//! fused and compared again offline (`docs/architecture/scan-sessions.md` section 8): the raw
//! streams (`pass-<id>/raw.bin`, `raw_rgb.bin`, written by the driver while scanning), the
//! capture record (`capture.json`), the per-frame tracking log (`tracking.csv`) and the
//! calibration files (`calibration/`). File formats: `scancraft_io::capture`.
//!
//! A pass's [`CaptureLog`] is shared between the app and the scan worker (which adds a row per
//! frame) and is written with every save.

use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};
use std::time::Instant;

use scancraft_driver::{DeviceInfo, DeviceTransport, RecordStats, ScanConfig, ScannerCapabilities};
use scancraft_io::capture::{self as files, AppBuild, CaptureEvent, CaptureRecord, CaptureSegment, TrackRow};

pub type SharedCapture = Arc<Mutex<CaptureLog>>;

/// Below this much free space on the drive raw streams are not recorded (scanning goes on).
pub const MIN_FREE_TO_START: u64 = 2 << 30;
/// A running recording stops when the drive has less than this left.
pub const MIN_FREE_WHILE_RECORDING: u64 = 1 << 30;

/// The build of this app.
pub fn app_build() -> AppBuild {
    AppBuild {
        name: "ScanCraft".into(),
        version: env!("CARGO_PKG_VERSION").into(),
        git_commit: env!("SCANCRAFT_GIT_COMMIT").into(),
        git_dirty: env!("SCANCRAFT_GIT_DIRTY") == "1",
        profile: env!("SCANCRAFT_BUILD_PROFILE").into(),
        target: env!("SCANCRAFT_BUILD_TARGET").into(),
    }
}

/// `0.2.0 (abc1234)`, for the interface.
pub fn version_label() -> String {
    let b = app_build();
    let short: String = b.git_commit.chars().take(7).collect();
    format!("{} ({short}{})", b.version, if b.git_dirty { ", modified" } else { "" })
}

/// The device as stored in project files: the serial number replaced by an anonymous hash and
/// the network address left out (projects are shared as test data).
pub fn sanitized_device(info: &DeviceInfo) -> DeviceInfo {
    let mut d = info.clone();
    if !files::is_serial_hash(&d.serial_number) {
        d.serial_number = files::serial_hash(&d.serial_number, &files::install_key());
    }
    if let DeviceTransport::WiFi { ip, .. } = &mut d.active_transport {
        *ip = String::new();
    }
    d
}

/// A live recording of one capture session (not saved; its counters are copied into the record).
#[derive(Debug, Clone)]
pub struct LiveRecording {
    pub segment: usize,
    pub main: Arc<RecordStats>,
    pub rgb: Arc<RecordStats>,
    /// Whether each stream is being recorded at all.
    pub main_on: bool,
    pub rgb_on: bool,
}

/// A pass's capture record, tracking log and calibration files (see the module docs).
#[derive(Debug, Default, Clone)]
pub struct CaptureLog {
    pub record: CaptureRecord,
    pub rows: Vec<TrackRow>,
    pub calibration: Vec<(String, Vec<u8>)>,
    pub live: Vec<LiveRecording>,
    /// The folder (project, working or staging folder) the running or last session recorded its
    /// raw streams into: its capture files are written there too, at the start, while
    /// recording and at the end, so a recording describes itself even after a crash.
    pub record_root: Option<PathBuf>,
    /// When the running session started (host clock), for row and event times.
    session_start: Option<Instant>,
}

/// What a new capture session starts with.
pub struct SessionStart<'a> {
    pub scan_config: &'a ScanConfig,
    pub device: Option<&'a DeviceInfo>,
    pub capabilities: Option<&'a ScannerCapabilities>,
    pub settings: serde_json::Value,
    pub tracker: serde_json::Value,
    pub info: Option<&'a scancraft_driver::RecordingInfo>,
    /// The recording target given to the driver, when the project records raw streams.
    pub target: Option<&'a scancraft_driver::RecordingTarget>,
    /// The pass folder the raw files are in (for relative names in the record).
    pub pass_dir: Option<&'a Path>,
    pub continued: bool,
    pub seeded: bool,
    pub live_ai: bool,
    /// Why the raw streams are not recorded, when they are not.
    pub not_recorded: Option<String>,
}

fn utc_now() -> (String, u64) {
    let now = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).unwrap_or_default();
    (scancraft_io::fsutil::format_utc(now.as_secs()), now.as_millis() as u64)
}

impl CaptureLog {
    pub fn shared() -> SharedCapture {
        Arc::new(Mutex::new(CaptureLog::default()))
    }

    /// Starts a capture session; returns its index.
    pub fn begin(&mut self, s: SessionStart) -> usize {
        let r = &mut self.record;
        let first = r.captures.is_empty();
        r.format = files::CAPTURE_FORMAT;
        r.app = app_build();
        // Only the file name: the folder (local app data) names the user.
        if let Some(name) = crate::logfile::path().and_then(|p| p.file_name().map(|n| n.to_string_lossy().to_string())) {
            r.app_log = Some(name);
        }
        let config_json = serde_json::to_value(s.scan_config).unwrap_or_default();
        if first {
            r.scan_config = config_json.clone();
            r.device = s.device.map(|d| serde_json::to_value(sanitized_device(d)).unwrap_or_default()).unwrap_or_default();
            r.capabilities = s.capabilities.map(|c| serde_json::to_value(c).unwrap_or_default()).unwrap_or_default();
            r.settings = s.settings.clone();
            r.tracker = s.tracker;
            r.tracking_log = Some("tracking.csv".into());
            r.turntable = turntable_json(s.scan_config, None);
        }
        if let Some(info) = s.info {
            if !info.calibration.is_empty() {
                r.calibration = info.calibration.iter().map(|(n, _)| n.clone()).collect();
                self.calibration = info.calibration.clone();
            }
        }
        // Raw file names: relative to the pass folder when inside it, else as written.
        let name = |p: &Path| match s.pass_dir.and_then(|d| p.strip_prefix(d).ok()) {
            Some(rel) => rel.to_string_lossy().replace('\\', "/"),
            None => p.display().to_string(),
        };
        let files_written: Vec<PathBuf> = s.info.map(|i| i.files.clone()).unwrap_or_default();
        let (main_on, rgb_on) = match s.target {
            Some(t) => (files_written.contains(&t.main), files_written.contains(&t.rgb)),
            None => (false, false),
        };
        // The files this session wrote: the target's, or (SCANCRAFT_RECORD_DIR) the absolute
        // ones the driver chose, one pair per session.
        let (main_file, rgb_file) = match s.target {
            Some(t) => (main_on.then(|| name(&t.main)), rgb_on.then(|| name(&t.rgb))),
            None => (files_written.first().map(|f| f.display().to_string()), files_written.get(1).map(|f| f.display().to_string())),
        };
        let (started_utc, started_unix_ms) = utc_now();
        let mut segment = CaptureSegment {
            started_utc,
            started_unix_ms,
            continued: s.continued,
            seeded: s.seeded,
            live_ai: s.live_ai,
            scan_config: config_json,
            stream: s.info.map(|i| i.stream.clone()).unwrap_or_default(),
            recorded: Some(main_file.is_some()),
            rgb_recorded: Some(rgb_file.is_some()),
            settings: s.settings,
            ..Default::default()
        };
        // The first recorded file is the pass's; a session in another file names its own.
        if let Some(f) = main_file {
            match &r.raw_main {
                None => r.raw_main = Some(f),
                Some(m) if *m != f => segment.raw_file = Some(f),
                Some(_) => {}
            }
        }
        if let Some(f) = rgb_file {
            match &r.raw_rgb {
                None => r.raw_rgb = Some(f),
                Some(m) if *m != f => segment.raw_rgb_file = Some(f),
                Some(_) => {}
            }
        }
        if let Some(t) = s.target {
            segment.raw_offset = if main_on { t.main_stats.start_offset.load(std::sync::atomic::Ordering::Relaxed) } else { 0 };
            segment.raw_rgb_offset = if rgb_on { t.rgb_stats.start_offset.load(std::sync::atomic::Ordering::Relaxed) } else { 0 };
        }
        if let Some(why) = &s.not_recorded {
            segment.events.push(CaptureEvent { t_ms: 0, what: format!("raw streams not recorded: {why}") });
        }
        if main_on || rgb_on {
            self.record_root = s.pass_dir.and_then(|d| d.parent()).map(Path::to_path_buf);
        }
        r.captures.push(segment);
        let index = r.captures.len() - 1;
        if let Some(t) = s.target {
            self.live.push(LiveRecording { segment: index, main: t.main_stats.clone(), rgb: t.rgb_stats.clone(), main_on, rgb_on });
        }
        self.session_start = Some(Instant::now());
        self.refresh();
        index
    }

    /// Milliseconds since the running session started.
    pub fn now_ms(&self) -> u64 {
        self.session_start.map_or(0, |t| t.elapsed().as_millis() as u64)
    }

    pub fn event(&mut self, what: impl Into<String>) {
        let t_ms = self.now_ms();
        if let Some(seg) = self.record.captures.last_mut() {
            seg.events.push(CaptureEvent { t_ms, what: what.into() });
        }
    }

    /// Ends the running session (end time, recording counters).
    pub fn end(&mut self) {
        let (ended, _) = utc_now();
        let duration = self.session_start.take().map_or(0.0, |t| t.elapsed().as_secs_f64());
        if let Some(seg) = self.record.captures.last_mut() {
            seg.ended_utc = Some(ended);
            seg.duration_s = (duration * 10.0).round() / 10.0;
        }
        self.refresh();
    }

    /// Laser lines: where session `segment`'s planes came from; the planes it started with are
    /// kept as a calibration file (`laser_planes-<segment>.json`), so a replay uses the same
    /// ones whatever the plane cache of the computer holds.
    pub fn set_laser_planes(&mut self, segment: usize, origin: &scancraft_laser::PlanesOrigin, start: Option<&scancraft_laser::planes::LaserPlaneSet>) {
        use scancraft_laser::PlanesOrigin as O;
        let label = match origin {
            O::Unknown => "unknown",
            O::Given => "given",
            O::Env(_) => "env",
            O::Cache(_) => "cache",
            O::Online => "online",
        };
        let file = start.and_then(|p| serde_json::to_vec_pretty(p).ok()).map(|bytes| {
            let name = format!("laser_planes-{segment}.json");
            self.calibration.retain(|(n, _)| *n != name);
            self.calibration.push((name.clone(), bytes));
            if !self.record.calibration.contains(&name) {
                self.record.calibration.push(name.clone());
            }
            name
        });
        if let Some(seg) = self.record.captures.get_mut(segment) {
            seg.laser_planes = serde_json::json!({ "origin": label, "file": file, "planes": start.map(|p| p.planes.len()) });
        }
    }

    /// The turntable capture volume, once the pipeline found it.
    pub fn set_turntable_region(&mut self, config: &ScanConfig, region: Option<scancraft_pipeline::TurntableRegion>) {
        self.record.turntable = turntable_json(config, region);
    }

    /// Copies the live recording counters into the record.
    pub fn refresh(&mut self) {
        for live in &self.live {
            if let Some(seg) = self.record.captures.get_mut(live.segment) {
                seg.recording = if live.main_on { serde_json::to_value(live.main.snapshot()).unwrap_or_default() } else { serde_json::Value::Null };
                seg.rgb_recording = if live.rgb_on { serde_json::to_value(live.rgb.snapshot()).unwrap_or_default() } else { serde_json::Value::Null };
            }
        }
    }

    /// Bytes of raw stream recorded for the pass (all sessions, both streams).
    pub fn raw_bytes(&self) -> u64 {
        let bytes = |v: &serde_json::Value| v.get("bytes_written").and_then(|b| b.as_u64()).unwrap_or(0);
        let mut total = 0;
        for (i, seg) in self.record.captures.iter().enumerate() {
            match self.live.iter().find(|l| l.segment == i) {
                Some(l) => {
                    total += if l.main_on { l.main.bytes_written.load(std::sync::atomic::Ordering::Relaxed) } else { 0 };
                    total += if l.rgb_on { l.rgb.bytes_written.load(std::sync::atomic::Ordering::Relaxed) } else { 0 };
                }
                None => total += bytes(&seg.recording) + bytes(&seg.rgb_recording),
            }
        }
        total
    }

    /// Records dropped because the disk could not keep up (all sessions).
    pub fn dropped_records(&self) -> u64 {
        self.live.iter().map(|l| l.main.records_dropped.load(std::sync::atomic::Ordering::Relaxed) + l.rgb.records_dropped.load(std::sync::atomic::Ordering::Relaxed)).sum::<u64>()
            + self
                .record
                .captures
                .iter()
                .enumerate()
                .filter(|(i, _)| !self.live.iter().any(|l| l.segment == *i))
                .map(|(_, s)| [&s.recording, &s.rgb_recording].iter().filter_map(|v| v.get("records_dropped").and_then(|b| b.as_u64())).sum::<u64>())
                .sum::<u64>()
    }

    /// The live recordings' counters (to wait for them to finish writing).
    pub fn recording_stats(&self) -> Vec<Arc<RecordStats>> {
        self.live.iter().flat_map(|l| [l.main_on.then(|| l.main.clone()), l.rgb_on.then(|| l.rgb.clone())]).flatten().collect()
    }

    /// Whether any session recorded a raw stream (anywhere).
    pub fn has_raw(&self) -> bool {
        self.record.raw_main.is_some() || self.record.raw_rgb.is_some()
    }

    /// The capture record of a pass added from another project: everything but the raw
    /// recordings, which stay in that project (`raw_source` says where).
    pub fn without_raw(&self, project: &str, pass: u32) -> CaptureLog {
        let mut log = self.clone();
        let had = log.has_raw();
        let r = &mut log.record;
        if had && r.raw_source.is_none() {
            r.raw_source = Some(format!("{project}, pass {pass}"));
        }
        (r.raw_main, r.raw_rgb) = (None, None);
        for c in &mut r.captures {
            (c.raw_file, c.raw_rgb_file) = (None, None);
        }
        log.live.clear();
        log.record_root = None;
        log
    }
}

fn turntable_json(config: &ScanConfig, region: Option<scancraft_pipeline::TurntableRegion>) -> serde_json::Value {
    if config.turntable_mode == scancraft_driver::TurntableMode::HandheldFreeform {
        return serde_json::Value::Null;
    }
    let c = scancraft_pipeline::TurntableConfig::default();
    serde_json::json!({
        "mode": config.turntable_mode,
        "config": {
            "init_frames": c.init_frames,
            "plane_margin_mm": c.plane_margin_mm,
            "radius_mm": c.radius_mm,
            "height_mm": c.height_mm,
            "radius_factor": c.radius_factor,
            "auto_margin_mm": c.auto_margin_mm,
            "max_radius_mm": c.max_radius_mm,
            "min_up": c.min_up,
        },
        // In the (fixed) sensor frame: +X right, +Y forward, +Z up, mm.
        "region": region.map(|r| serde_json::json!({
            "normal": r.normal.to_array(),
            "offset_mm": r.offset,
            "center": r.center.to_array(),
            "radius_mm": r.radius_mm,
            "height_mm": r.height_mm,
            "plane_margin_mm": r.plane_margin_mm,
        })),
    })
}

/// Writes a pass's capture files into the project folder `dir`.
pub fn write(dir: &Path, id: u32, log: &mut CaptureLog) -> Result<(), String> {
    log.refresh();
    files::write_capture(&dir.join(files::capture_path(id)), &log.record).map_err(|e| e.to_string())?;
    files::write_tracking_log(&dir.join(files::tracking_log_path(id)), &log.rows).map_err(|e| e.to_string())?;
    if !log.calibration.is_empty() {
        files::write_calibration(&dir.join(files::calibration_dir(id)), &log.calibration).map_err(|e| e.to_string())?;
    }
    Ok(())
}

/// A pass's capture files from the project folder `dir`, if it has a capture record.
pub fn read(dir: &Path, id: u32) -> Option<CaptureLog> {
    let path = dir.join(files::capture_path(id));
    if !path.is_file() {
        return None;
    }
    let record = match files::read_capture(&path) {
        Ok(r) => r,
        Err(e) => {
            log::warn!("the capture record of pass {id} is not used: {e}");
            return None;
        }
    };
    let rows = files::read_tracking_log(&dir.join(files::tracking_log_path(id))).unwrap_or_default();
    let calibration = files::read_calibration(&dir.join(files::calibration_dir(id)), &record.calibration);
    Some(CaptureLog { record, rows, calibration, live: Vec::new(), record_root: None, session_start: None })
}

/// The state name of a tracking status in the tracking log.
pub fn state_name(state: scancraft_slam::TrackingState) -> &'static str {
    match state {
        scancraft_slam::TrackingState::Idle => "idle",
        scancraft_slam::TrackingState::Tracking => "tracking",
        scancraft_slam::TrackingState::Slow => "slow",
        scancraft_slam::TrackingState::Lost => "lost",
    }
}
