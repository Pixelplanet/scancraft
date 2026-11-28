//! What a captured pass records about how it was made (`docs/architecture/scan-sessions.md`
//! section 8), so it can be decoded, tracked, fused and compared again offline:
//!
//! ```text
//! pass-<id>/capture.json     the capture record: app build, device (no serial number), scan
//!                            settings, stream layout and setup sent, calibration file list,
//!                            turntable region, one entry per capture session (start, end, frame
//!                            counts, recording statistics, parameter changes)
//! pass-<id>/tracking.csv     one row per frame the live tracker processed (pose, state, scores)
//! pass-<id>/calibration/     the calibration files as the scanner sent them
//! pass-<id>/raw.bin          the main wire stream (depth / IR), when recorded
//! pass-<id>/raw_rgb.bin      the colour wire stream, when recorded
//! ```
//!
//! All of them are optional: a project without them opens as before, and versions that do not
//! know them ignore them. Fields of the capture record this version does not know are kept.

use std::path::Path;

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};

use crate::project::pass_dir;
use crate::{fsutil, IoError};

pub const CAPTURE_FORMAT: u32 = 1;
const TRACKING_HEADER: &str = "segment,frame,device_ms,host_ms,state,relocalized,inliers,rmse_mm,conflict,keyframe,new_points,total_points,qx,qy,qz,qw,tx,ty,tz,imu_qx,imu_qy,imu_qz,imu_qw,proc_ms";
const TRACKING_MAGIC: &str = "# scancraft tracking log v1";

/// A pass's capture record, relative to the project folder.
pub fn capture_path(id: u32) -> String {
    format!("{}/capture.json", pass_dir(id))
}

/// A pass's per-frame tracking log, relative to the project folder.
pub fn tracking_log_path(id: u32) -> String {
    format!("{}/tracking.csv", pass_dir(id))
}

/// The folder of a pass's calibration files, relative to the project folder.
pub fn calibration_dir(id: u32) -> String {
    format!("{}/calibration", pass_dir(id))
}

/// A pass's recorded colour stream, relative to the project folder.
pub fn raw_rgb_path(id: u32) -> String {
    format!("{}/raw_rgb.bin", pass_dir(id))
}

/// A stable, anonymous name for a device: the first 16 hex digits of an HMAC-SHA256 of its
/// serial number under `key`, a secret of this computer ([`install_key`], never saved in a
/// project). Tells recordings of two scanners apart without putting the serial number into
/// project files that may be shared: serial numbers follow a known pattern with few unknown
/// digits, so an unkeyed hash could be reversed by trying them all.
pub fn serial_hash(serial: &str, key: &[u8]) -> String {
    if serial.trim().is_empty() {
        return String::new();
    }
    let digest = hmac_sha256(key, serial.trim().as_bytes());
    let hex: String = digest.iter().take(8).map(|b| format!("{b:02x}")).collect();
    format!("{SERIAL_HASH_PREFIX}{hex}")
}

/// What a [`serial_hash`] starts with (`sha256:` was the unkeyed hash of the first version).
pub const SERIAL_HASH_PREFIX: &str = "hmac:";

/// Whether `s` is already an anonymous device name rather than a serial number.
pub fn is_serial_hash(s: &str) -> bool {
    s.starts_with(SERIAL_HASH_PREFIX) || s.starts_with("sha256:")
}

/// HMAC-SHA256 (RFC 2104).
fn hmac_sha256(key: &[u8], message: &[u8]) -> [u8; 32] {
    let mut block = [0u8; 64];
    if key.len() > 64 {
        block[..32].copy_from_slice(&Sha256::digest(key));
    } else {
        block[..key.len()].copy_from_slice(key);
    }
    let pad = |b: u8| block.iter().map(|k| k ^ b).collect::<Vec<u8>>();
    let inner = Sha256::new().chain_update(pad(0x36)).chain_update(message).finalize();
    Sha256::new().chain_update(pad(0x5c)).chain_update(inner).finalize().into()
}

/// The secret key of this computer for [`serial_hash`]: 32 random bytes in
/// `<local app data>/ScanCraft/device-key`, made on first use (see [`install_key_in`]).
pub fn install_key() -> Vec<u8> {
    static KEY: std::sync::OnceLock<Vec<u8>> = std::sync::OnceLock::new();
    KEY.get_or_init(|| install_key_in(&fsutil::local_data_dir().join("ScanCraft"))).clone()
}

/// [`install_key`] kept in the folder `dir`. When it cannot be stored a key for this run only is
/// used (device names then differ between runs, nothing else).
pub fn install_key_in(dir: &Path) -> Vec<u8> {
    let path = dir.join("device-key");
    let read = |p: &Path| std::fs::read_to_string(p).ok().and_then(|s| decode_hex(s.trim())).filter(|k| k.len() == 32);
    if let Some(key) = read(&path) {
        return key;
    }
    let key = random_key();
    let hex: String = key.iter().map(|b| format!("{b:02x}")).collect();
    let stored = std::fs::create_dir_all(dir).and_then(|_| std::fs::OpenOptions::new().write(true).create_new(true).open(&path)).and_then(|mut f| {
        use std::io::Write;
        f.write_all(hex.as_bytes())
    });
    match stored {
        Ok(()) => key,
        // Another start made it at the same moment: theirs counts.
        Err(e) if e.kind() == std::io::ErrorKind::AlreadyExists => read(&path).unwrap_or(key),
        Err(e) => {
            log::warn!("device key not stored in {}: {e}", path.display());
            key
        }
    }
}

fn decode_hex(s: &str) -> Option<Vec<u8>> {
    (s.len() % 2 == 0).then_some(())?;
    (0..s.len()).step_by(2).map(|i| u8::from_str_radix(s.get(i..i + 2)?, 16).ok()).collect()
}

/// 32 bytes from the operating system's random source (through the standard library's
/// randomly seeded hasher keys), mixed with the time and process id.
fn random_key() -> Vec<u8> {
    use std::hash::{BuildHasher, Hasher};
    let mut sha = Sha256::new();
    for i in 0..8u64 {
        let mut h = std::collections::hash_map::RandomState::new().build_hasher();
        h.write_u64(i);
        sha.update(h.finish().to_le_bytes());
    }
    let now = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).unwrap_or_default();
    sha.update(now.as_nanos().to_le_bytes());
    sha.update(std::process::id().to_le_bytes());
    sha.finalize().to_vec()
}

/// Whether `name` (a file in a pass folder) is a raw stream recording: `raw.bin` /
/// `raw_rgb.bin`, or the per-session files `raw-<n>.bin` / `raw_rgb-<n>.bin` a continued pass
/// records into when it cannot append to the first ones.
pub fn is_raw_stream_name(name: &str) -> bool {
    let name = name.to_ascii_lowercase();
    let Some(stem) = name.strip_suffix(".bin") else { return false };
    let numbered = |prefix: &str| stem.strip_prefix(prefix).is_some_and(|n| !n.is_empty() && n.bytes().all(|b| b.is_ascii_digit()));
    stem == "raw" || stem == "raw_rgb" || numbered("raw-") || numbered("raw_rgb-")
}

/// The build of the app that made a capture.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct AppBuild {
    pub name: String,
    pub version: String,
    /// Git commit the app was built from ("unknown" outside a checkout).
    pub git_commit: String,
    /// The checkout had uncommitted changes when it was built.
    pub git_dirty: bool,
    /// "release" or "debug".
    pub profile: String,
    pub target: String,
}

/// Frame counts of one capture session.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct FrameCounts {
    /// Frames the scanner delivered to the app.
    pub received: u64,
    /// Frames skipped because a newer one was waiting (the app always works on the newest).
    pub skipped_for_newer: u64,
    /// Frames received while paused (shown, not tracked).
    pub paused: u64,
    /// Frames given to the tracker.
    pub processed: u64,
    /// Processed frames without usable geometry (also turntable frames used to find the table).
    pub no_geometry: u64,
    pub tracking: u64,
    pub slow: u64,
    pub lost: u64,
    pub relocalized: u64,
    /// Frames stored as keyframes.
    pub keyframes: u64,
    /// Surface points added to the pass.
    pub fused_points: u64,
}

/// Something that happened during a capture session (a parameter change, a recording warning).
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct CaptureEvent {
    /// Milliseconds since the session started.
    pub t_ms: u64,
    pub what: String,
}

/// One capture session of a pass (a pass continued later has several).
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct CaptureSegment {
    /// Wall-clock start and end (UTC, `YYYY-MM-DD HH:MM:SS`), and the start in ms since 1970.
    pub started_utc: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub ended_utc: Option<String>,
    pub started_unix_ms: u64,
    pub duration_s: f64,
    /// Continued an earlier capture of the pass (same tracker).
    pub continued: bool,
    /// The tracker was seeded with the placed passes (Same Position start): a replay without
    /// that seed starts in the pass's own frame.
    pub seeded: bool,
    /// Live AI masking was on (frames were masked before tracking; a replay does not mask).
    pub live_ai: bool,
    /// Where this session's data starts in the raw files (bytes).
    pub raw_offset: u64,
    pub raw_rgb_offset: u64,
    /// Whether this session recorded the main / colour stream at all (`None` in records of the
    /// first version: then `recording` / `rgb_recording` being set says so). A session that did
    /// not record has no data in the raw files, whatever its offset says.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub recorded: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub rgb_recorded: Option<bool>,
    /// The file this session recorded into, when it is not the record's `raw_main` / `raw_rgb`
    /// (a continued pass whose first recording was not where it could append: `raw-<n>.bin`).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub raw_file: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub raw_rgb_file: Option<String>,
    /// The app settings this session ran with (depth window, point pitch, ...; a continued
    /// session can differ from the first).
    #[serde(skip_serializing_if = "serde_json::Value::is_null")]
    pub settings: serde_json::Value,
    /// Laser-line modes: where the laser planes came from (`cache`, `env`, `online`) and the
    /// calibration file holding the planes the session started with.
    #[serde(skip_serializing_if = "serde_json::Value::is_null")]
    pub laser_planes: serde_json::Value,
    /// The scan settings this session was started with (exposure / gain can change between
    /// sessions of a pass).
    pub scan_config: serde_json::Value,
    /// Stream layout and the setup sent to the scanner for this session (from the driver).
    pub stream: serde_json::Value,
    /// Recording statistics of the main and colour streams (`null`: not recorded).
    pub recording: serde_json::Value,
    pub rgb_recording: serde_json::Value,
    pub frames: FrameCounts,
    pub events: Vec<CaptureEvent>,
    #[serde(flatten)]
    pub other: serde_json::Map<String, serde_json::Value>,
}

/// `pass-<id>/capture.json`.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct CaptureRecord {
    pub format: u32,
    pub app: AppBuild,
    /// The scanner: vendor, model, firmware, transport kind, resolutions, IMU mounting and an
    /// anonymous [`serial_hash`] (never the serial number).
    pub device: serde_json::Value,
    pub capabilities: serde_json::Value,
    /// The pass's scan settings (as when it was started).
    pub scan_config: serde_json::Value,
    /// App settings that shape the capture (depth window, point pitch, point budget).
    pub settings: serde_json::Value,
    /// Tracker settings.
    pub tracker: serde_json::Value,
    /// Turntable mode, filter settings and the capture volume found (`null` when handheld).
    pub turntable: serde_json::Value,
    /// Calibration files, relative to `pass-<id>/calibration/`.
    pub calibration: Vec<String>,
    /// Raw stream files relative to the pass folder (`raw.bin`, `raw_rgb.bin`), when recorded
    /// into the project; an absolute path when recorded elsewhere (`SCANCRAFT_RECORD_DIR`).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub raw_main: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub raw_rgb: Option<String>,
    /// A pass added from another project (Add Project, Add Passes) does not bring its raw
    /// recordings: where they stay ("<project>, pass <id>").
    #[serde(skip_serializing_if = "Option::is_none")]
    pub raw_source: Option<String>,
    /// The per-frame tracking log, relative to the pass folder.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub tracking_log: Option<String>,
    /// The name of the app's log file of the capture (in `<local app data>/ScanCraft/logs`; not
    /// part of the project).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub app_log: Option<String>,
    pub captures: Vec<CaptureSegment>,
    #[serde(flatten)]
    pub other: serde_json::Map<String, serde_json::Value>,
}

impl CaptureRecord {
    /// Frame counts of all sessions together.
    pub fn total_frames(&self) -> FrameCounts {
        let mut t = FrameCounts::default();
        for c in &self.captures {
            let f = &c.frames;
            t.received += f.received;
            t.skipped_for_newer += f.skipped_for_newer;
            t.paused += f.paused;
            t.processed += f.processed;
            t.no_geometry += f.no_geometry;
            t.tracking += f.tracking;
            t.slow += f.slow;
            t.lost += f.lost;
            t.relocalized += f.relocalized;
            t.keyframes += f.keyframes;
            t.fused_points += f.fused_points;
        }
        t
    }
}

pub fn write_capture(path: &Path, record: &CaptureRecord) -> Result<(), IoError> {
    if let Some(dir) = path.parent() {
        std::fs::create_dir_all(dir)?;
    }
    let json = serde_json::to_vec_pretty(record).map_err(|e| IoError::Parse(e.to_string()))?;
    fsutil::atomic_write(path, &json)
}

pub fn read_capture(path: &Path) -> Result<CaptureRecord, IoError> {
    let bytes = std::fs::read(path)?;
    let record: CaptureRecord = serde_json::from_slice(&bytes).map_err(|e| IoError::Parse(format!("{}: {e}", path.display())))?;
    if record.format > CAPTURE_FORMAT {
        log::warn!("{}: capture record format {} is newer than this version's {CAPTURE_FORMAT}; read as far as known", path.display(), record.format);
    }
    Ok(record)
}

/// One frame the live tracker processed.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct TrackRow {
    /// Capture session (index into [`CaptureRecord::captures`]).
    pub segment: u32,
    /// Frame number given by the driver, and the frame's device time stamp (the key a replay
    /// matches frames by).
    pub frame: u64,
    pub device_ms: u64,
    /// When the app received it, ms since the session started.
    pub host_ms: u64,
    /// "tracking", "slow", "lost", "idle", or "no_geometry" (nothing usable: no pose).
    pub state: String,
    pub relocalized: bool,
    pub inliers: u64,
    pub rmse_mm: f32,
    pub conflict: f32,
    pub keyframe: Option<u64>,
    pub new_points: u64,
    pub total_points: u64,
    /// Sensor pose in the pass frame (pass <- sensor): rotation (x, y, z, w), translation mm.
    pub rotation: [f32; 4],
    pub translation: [f32; 3],
    /// IMU orientation delivered with the frame (x, y, z, w).
    pub imu: Option<[f32; 4]>,
    /// Time the pipeline took for the frame, ms.
    pub proc_ms: f32,
}

impl TrackRow {
    /// The frame has a pose (it was tracked, possibly as lost).
    pub fn has_pose(&self) -> bool {
        self.state != "no_geometry"
    }

    fn csv(&self) -> String {
        let opt = |v: Option<f32>| v.map_or(String::new(), |v| format!("{v:.7}"));
        let imu = self.imu.map_or([None; 4], |q| q.map(Some));
        format!(
            "{},{},{},{},{},{},{},{:.4},{:.4},{},{},{},{:.7},{:.7},{:.7},{:.7},{:.4},{:.4},{:.4},{},{},{},{},{:.1}",
            self.segment,
            self.frame,
            self.device_ms,
            self.host_ms,
            self.state,
            self.relocalized as u8,
            self.inliers,
            finite(self.rmse_mm),
            finite(self.conflict),
            self.keyframe.map_or(String::new(), |k| k.to_string()),
            self.new_points,
            self.total_points,
            self.rotation[0],
            self.rotation[1],
            self.rotation[2],
            self.rotation[3],
            self.translation[0],
            self.translation[1],
            self.translation[2],
            opt(imu[0]),
            opt(imu[1]),
            opt(imu[2]),
            opt(imu[3]),
            self.proc_ms
        )
    }

    fn parse(line: &str) -> Option<TrackRow> {
        let f: Vec<&str> = line.split(',').map(str::trim).collect();
        if f.len() < 24 {
            return None;
        }
        let num = |i: usize| f[i].parse::<f32>().ok();
        let int = |i: usize| f[i].parse::<u64>().ok();
        let imu = match (num(19), num(20), num(21), num(22)) {
            (Some(x), Some(y), Some(z), Some(w)) => Some([x, y, z, w]),
            _ => None,
        };
        Some(TrackRow {
            segment: f[0].parse().ok()?,
            frame: int(1)?,
            device_ms: int(2)?,
            host_ms: int(3)?,
            state: f[4].to_string(),
            relocalized: f[5] == "1",
            inliers: int(6)?,
            rmse_mm: num(7)?,
            conflict: num(8)?,
            keyframe: int(9),
            new_points: int(10)?,
            total_points: int(11)?,
            rotation: [num(12)?, num(13)?, num(14)?, num(15)?],
            translation: [num(16)?, num(17)?, num(18)?],
            imu,
            proc_ms: num(23).unwrap_or(0.0),
        })
    }
}

fn finite(v: f32) -> f32 {
    if v.is_finite() {
        v
    } else {
        -1.0
    }
}

/// Writes a tracking log (CSV, one header line; see [`TrackRow`]).
pub fn write_tracking_log(path: &Path, rows: &[TrackRow]) -> Result<(), IoError> {
    if let Some(dir) = path.parent() {
        std::fs::create_dir_all(dir)?;
    }
    fsutil::atomic_write_with(path, |w| {
        use std::io::Write;
        writeln!(w, "{TRACKING_MAGIC}")?;
        writeln!(w, "{TRACKING_HEADER}")?;
        for r in rows {
            writeln!(w, "{}", r.csv())?;
        }
        Ok(())
    })
}

/// Reads a tracking log; lines that cannot be read are skipped (and counted in the log).
pub fn read_tracking_log(path: &Path) -> Result<Vec<TrackRow>, IoError> {
    let text = std::fs::read_to_string(path)?;
    let mut rows = Vec::new();
    let mut bad = 0;
    for line in text.lines().filter(|l| !l.trim().is_empty() && !l.starts_with('#') && !l.starts_with("segment,")) {
        match TrackRow::parse(line) {
            Some(r) => rows.push(r),
            None => bad += 1,
        }
    }
    if bad > 0 {
        log::warn!("{}: {bad} lines could not be read", path.display());
    }
    Ok(rows)
}

/// The calibration files of a pass as saved: (name relative to its calibration folder, bytes).
/// A name that is not a plain relative path is skipped.
pub fn write_calibration(dir: &Path, files: &[(String, Vec<u8>)]) -> Result<(), IoError> {
    for (name, bytes) in files {
        if !fsutil::is_safe_relative(name) {
            log::warn!("calibration file name {name:?} skipped");
            continue;
        }
        let path = dir.join(name);
        if let Some(parent) = path.parent() {
            std::fs::create_dir_all(parent)?;
        }
        fsutil::atomic_write(&path, bytes)?;
    }
    Ok(())
}

/// Reads the calibration files `names` from `dir` (missing ones are skipped and logged).
pub fn read_calibration(dir: &Path, names: &[String]) -> Vec<(String, Vec<u8>)> {
    names
        .iter()
        .filter(|n| fsutil::is_safe_relative(n))
        .filter_map(|n| match std::fs::read(dir.join(n)) {
            Ok(b) => Some((n.clone(), b)),
            Err(e) => {
                log::warn!("calibration file {}: {e}", dir.join(n).display());
                None
            }
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn temp(name: &str) -> std::path::PathBuf {
        let dir = std::env::temp_dir().join(format!("scancraft-capture-{}-{name}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        dir
    }

    #[test]
    fn capture_record_and_tracking_log_round_trip() {
        let dir = temp("rt");
        let mut record = CaptureRecord { format: CAPTURE_FORMAT, raw_main: Some("raw.bin".into()), tracking_log: Some("tracking.csv".into()), calibration: vec!["camparam.yaml".into()], ..Default::default() };
        record.app = AppBuild { name: "ScanCraft".into(), version: "0.2.0".into(), git_commit: "abc".into(), ..Default::default() };
        record.captures.push(CaptureSegment { started_utc: "2026-11-28 10:00:00".into(), frames: FrameCounts { received: 10, tracking: 7, lost: 1, ..Default::default() }, ..Default::default() });
        record.other.insert("future".into(), serde_json::json!({ "x": 1 }));
        let path = dir.join(capture_path(3));
        write_capture(&path, &record).unwrap();
        assert_eq!(read_capture(&path).unwrap(), record, "unknown fields are kept");
        assert_eq!(record.total_frames().tracking, 7);

        let rows = vec![
            TrackRow { segment: 0, frame: 1, device_ms: 66, state: "no_geometry".into(), rotation: [0.0, 0.0, 0.0, 1.0], ..Default::default() },
            TrackRow {
                segment: 0,
                frame: 2,
                device_ms: 132,
                host_ms: 70,
                state: "tracking".into(),
                inliers: 1234,
                rmse_mm: 0.25,
                conflict: 0.03,
                keyframe: Some(4),
                new_points: 99,
                total_points: 5000,
                rotation: [0.1, 0.2, 0.3, 0.9273618],
                translation: [1.5, -2.25, 300.125],
                imu: Some([0.0, 0.0, 0.7071068, 0.7071068]),
                relocalized: true,
                proc_ms: 41.5,
            },
            TrackRow { segment: 1, frame: 3, device_ms: 198, state: "lost".into(), rmse_mm: f32::INFINITY, rotation: [0.0, 0.0, 0.0, 1.0], ..Default::default() },
        ];
        let log = dir.join(tracking_log_path(3));
        write_tracking_log(&log, &rows).unwrap();
        let back = read_tracking_log(&log).unwrap();
        assert_eq!(back.len(), 3);
        assert_eq!(back[1], rows[1]);
        assert!(!back[0].has_pose() && back[1].has_pose());
        assert_eq!(back[2].rmse_mm, -1.0, "a non-finite score is written as -1");

        write_calibration(&dir.join(calibration_dir(3)), &[("camparam.yaml".into(), b"%YAML".to_vec()), ("../escape".into(), vec![1])]).unwrap();
        assert!(!dir.join("pass-3/escape").exists() && !dir.join("escape").exists());
        let cal = read_calibration(&dir.join(calibration_dir(3)), &["camparam.yaml".into(), "missing.bin".into()]);
        assert_eq!(cal, vec![("camparam.yaml".to_string(), b"%YAML".to_vec())]);
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn the_serial_hash_is_stable_and_hides_the_serial() {
        let dir = temp("key");
        let key = install_key_in(&dir);
        assert_eq!(key.len(), 32);
        assert_eq!(install_key_in(&dir), key, "the key is kept");
        let a = serial_hash("D00000000B0S00E1B", &key);
        assert_eq!(a, serial_hash(" D00000000B0S00E1B ", &key));
        assert!(a.starts_with(SERIAL_HASH_PREFIX) && a.len() == SERIAL_HASH_PREFIX.len() + 16 && !a.contains("D0000"));
        assert!(is_serial_hash(&a) && !is_serial_hash("D00000000B0S00E1B"));
        assert_ne!(a, serial_hash("D00000000B0S00E2B", &key));
        assert_eq!(serial_hash("", &key), "");
        // RFC 4231 test case 2.
        let mac: String = hmac_sha256(b"Jefe", b"what do ya want for nothing?").iter().map(|b| format!("{b:02x}")).collect();
        assert_eq!(mac, "5bdcc146bf60754e6a042426089575c75a003f089d2739839dec58b964ec3843");
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn the_serial_cannot_be_found_by_trying_serials_without_the_key() {
        // Serial numbers have a known form (the placeholder D00000000B0S00E1B; the real ones end
        // in the model code). Without this computer's key, hashing candidates in any public way
        // never gives the stored name; another computer gives another name.
        let serial = "D00000042B0S00E1B";
        let stored = serial_hash(serial, &random_key());
        let public = |s: &str| -> Vec<String> {
            let take = |d: &[u8]| -> String { d.iter().take(8).map(|b| format!("{b:02x}")).collect() };
            vec![take(&Sha256::digest(format!("scancraft-device:{s}").as_bytes())), take(&Sha256::digest(s.as_bytes())), take(&hmac_sha256(b"", s.as_bytes()))]
        };
        let hex = stored.trim_start_matches(SERIAL_HASH_PREFIX);
        assert!(!public(serial).iter().any(|h| h == hex), "the stored name is not an unkeyed hash of the serial");
        assert_ne!(stored, serial_hash(serial, &random_key()), "two computers name the scanner differently");
    }

    #[test]
    fn raw_stream_names() {
        for yes in ["raw.bin", "raw_rgb.bin", "raw-1.bin", "raw_rgb-12.bin", "RAW.BIN"] {
            assert!(is_raw_stream_name(yes), "{yes}");
        }
        for no in ["raw-.bin", "raw-a.bin", "raw.bin.tmp", "keyframes.bin", "raw_rgb-1.json", "raw2.bin"] {
            assert!(!is_raw_stream_name(no), "{no}");
        }
    }
}
