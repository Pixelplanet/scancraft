//! A recorded POP 4 marker stream played back as a scanner (development and screenshots):
//! `scancraft --replay <recording.bin> <camparam.yaml>`.
//!
//! The recording is the raw wire stream written by `SCANCRAFT_RECORD_DIR` (or the driver's
//! `marker_record` example) in display type 6 (`DepthIrPairOriIr`). Frames are decoded with the
//! driver's own decoder and marker stream and sent at their recorded pace, so the app sees what
//! a live marker scan would deliver, repeated IR images included.

use std::path::PathBuf;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};

use async_trait::async_trait;
use scancraft_core::{ImuSample, PreviewFrame, SensorFrame};
use scancraft_driver::{DeviceInfo, DeviceParam, DeviceTransport, DriverError, ObjectType, ScanConfig, ScannerCapabilities, ScannerDevice};
use scancraft_driver_revopoint::media::{FrameDecoder, FrameLayout, PacketParser};
use scancraft_driver_revopoint::streams::{MarkerStream, ModeStream};
use scancraft_driver_revopoint::{DeviceCalibration, StereoCalibration};
use tokio::sync::{broadcast, mpsc};

pub struct ReplayDevice {
    info: DeviceInfo,
    capabilities: ScannerCapabilities,
    recording: PathBuf,
    calibration: Arc<DeviceCalibration>,
    streaming: Arc<AtomicBool>,
    paused: Arc<AtomicBool>,
}

impl ReplayDevice {
    pub fn open(recording: PathBuf, camparam: PathBuf) -> Result<Self, String> {
        let yaml = std::fs::read_to_string(&camparam).map_err(|e| format!("{}: {e}", camparam.display()))?;
        let calibration = Arc::new(DeviceCalibration::new(StereoCalibration::parse(&yaml).ok_or_else(|| "camparam.yaml could not be parsed".to_string())?));
        if !recording.exists() {
            return Err(format!("{} not found", recording.display()));
        }
        let name = recording.file_name().map(|n| n.to_string_lossy().to_string()).unwrap_or_default();
        Ok(ReplayDevice {
            info: DeviceInfo {
                vendor: "Revopoint (recording)".into(),
                model: "POP 4 recording".into(),
                serial_number: name.clone(),
                firmware_version: "—".into(),
                active_transport: DeviceTransport::Virtual { name: format!("replay {name}") },
                depth_resolution: (640, 512),
                rgb_resolution: None,
                recommended_distance_range_mm: (250.0, 500.0),
                imu_to_sensor: None,
            },
            capabilities: ScannerCapabilities {
                has_imu: false,
                supports_continuous_scan: true,
                supports_marker_tracking: true,
                supports_feature_tracking: true,
                supported_transports: vec!["Recording".into()],
                modes: scancraft_driver_revopoint::modes::pop4_modes(),
                ..Default::default()
            },
            recording,
            calibration,
            streaming: Arc::new(AtomicBool::new(false)),
            paused: Arc::new(AtomicBool::new(false)),
        })
    }
}

#[async_trait]
impl ScannerDevice for ReplayDevice {
    fn info(&self) -> &DeviceInfo {
        &self.info
    }

    fn capabilities(&self) -> &ScannerCapabilities {
        &self.capabilities
    }

    async fn start_preview(&mut self) -> Result<mpsc::Receiver<PreviewFrame>, DriverError> {
        Err(DriverError::Other("a recording has no preview; start a capture to play it".into()))
    }

    async fn stop_preview(&mut self) -> Result<(), DriverError> {
        Ok(())
    }

    async fn start_scan(&mut self, _config: &ScanConfig) -> Result<mpsc::Receiver<SensorFrame>, DriverError> {
        let data = std::fs::read(&self.recording)?;
        let mut stream = MarkerStream::new(ObjectType::General, self.calibration.clone())?;
        let (tx, rx) = mpsc::channel(8);
        let streaming = self.streaming.clone();
        let paused = self.paused.clone();
        streaming.store(true, Ordering::SeqCst);
        std::thread::spawn(move || {
            let mut parser = PacketParser::default();
            parser.push(&data);
            let mut decoder = FrameDecoder::new(FrameLayout::DepthIrPairOriIr, 640, 512);
            let mut index = 0u64;
            let mut last_ts: Option<u64> = None;
            while let Some(packet) = parser.next_packet() {
                if !streaming.load(Ordering::Relaxed) {
                    break;
                }
                let Ok(decoded) = decoder.decode(&packet) else { continue };
                let Some(frame) = stream.to_sensor_frame(decoded, index + 1) else { continue };
                index += 1;
                // The recorded pace (packet timestamps), within sane bounds.
                let wait = last_ts.map_or(66, |prev| frame.timestamp_ms.saturating_sub(prev).clamp(10, 500));
                last_ts = Some(frame.timestamp_ms);
                std::thread::sleep(std::time::Duration::from_millis(wait));
                while paused.load(Ordering::Relaxed) && streaming.load(Ordering::Relaxed) {
                    std::thread::sleep(std::time::Duration::from_millis(50));
                }
                if tx.blocking_send(frame).is_err() {
                    break;
                }
            }
            log::info!("replay finished after {index} frames");
        });
        Ok(rx)
    }

    async fn pause_scan(&mut self) -> Result<(), DriverError> {
        self.paused.store(true, Ordering::SeqCst);
        Ok(())
    }

    async fn resume_scan(&mut self) -> Result<(), DriverError> {
        self.paused.store(false, Ordering::SeqCst);
        Ok(())
    }

    async fn stop_scan(&mut self) -> Result<(), DriverError> {
        self.streaming.store(false, Ordering::SeqCst);
        Ok(())
    }

    fn imu_stream(&mut self) -> Option<broadcast::Receiver<ImuSample>> {
        None
    }

    async fn set_parameter(&mut self, _param: DeviceParam) -> Result<(), DriverError> {
        Ok(())
    }

    async fn disconnect(&mut self) -> Result<(), DriverError> {
        self.streaming.store(false, Ordering::SeqCst);
        Ok(())
    }
}

// ------------------------------------------------------------------ replay of a project's pass

use scancraft_io::capture::TrackRow;
use scancraft_slam::Tracker;

/// Which frames of a recorded pass a replay processes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReplayFrames {
    /// Exactly the frames the live tracker processed (its `tracking.csv`, matched by device time
    /// stamp): the live run again, to compare poses.
    Logged,
    /// Every recorded frame (what a faster computer, or offline re-processing, would see).
    All,
}

/// Poses of a replay against the live run's tracking log.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct PoseComparison {
    /// Frames with a pose in both.
    pub frames: usize,
    pub max_translation_mm: f32,
    pub median_translation_mm: f32,
    pub max_rotation_deg: f32,
    /// Frames whose state (tracking / slow / lost / no geometry) differs.
    pub state_differs: usize,
}

/// What a replay of a pass gives.
pub struct PassReplay {
    /// One row per processed frame, as the live tracking log.
    pub rows: Vec<TrackRow>,
    pub tracker: Tracker,
    /// Frames decoded from the recording, and logged frames not found in it.
    pub decoded: usize,
    pub missing: usize,
    /// Against the live tracking log (`None` without one).
    pub comparison: Option<PoseComparison>,
}

/// A recording's file, relative to the pass folder or absolute.
fn raw_file(pass_dir: &std::path::Path, name: &str) -> PathBuf {
    let p = std::path::Path::new(name);
    if p.is_absolute() {
        p.to_path_buf()
    } else {
        pass_dir.join(p)
    }
}

/// `len` bytes of `path` from `offset`.
fn segment_reader(path: &std::path::Path, offset: u64, len: u64) -> Result<Box<dyn std::io::Read>, String> {
    use std::io::{Read, Seek};
    let mut f = std::fs::File::open(path).map_err(|e| format!("{}: {e}", path.display()))?;
    f.seek(std::io::SeekFrom::Start(offset)).map_err(|e| e.to_string())?;
    Ok(Box::new(std::io::BufReader::with_capacity(4 << 20, f).take(len)))
}

fn read_all(mut r: Box<dyn std::io::Read>) -> Result<Vec<u8>, String> {
    let mut v = Vec::new();
    r.read_to_end(&mut v).map_err(|e| e.to_string())?;
    Ok(v)
}

/// Replays pass `id` of the project folder `dir` from its raw recording: the recorded frames are
/// decoded with the pass's calibration files and scan settings (`capture.json`) and run through a
/// fresh pipeline and tracker, as live. See `docs/HANDOFF.md` ("Replaying a pass").
pub fn replay_pass(dir: &std::path::Path, id: u32, which: ReplayFrames) -> Result<PassReplay, String> {
    let log = crate::capture::read(dir, id).ok_or_else(|| format!("pass {id} has no capture record (pass-{id}/capture.json)"))?;
    let record = &log.record;
    let pass_dir = dir.join(scancraft_io::project::pass_dir(id));
    let main = record.raw_main.as_deref().map(|n| raw_file(&pass_dir, n)).filter(|p| p.is_file()).ok_or_else(|| format!("pass {id} has no raw recording"))?;
    let rgb = record.raw_rgb.as_deref().map(|n| raw_file(&pass_dir, n)).filter(|p| p.is_file());
    let device: Option<DeviceInfo> = serde_json::from_value(record.device.clone()).ok();
    let min_mm = record.settings.get("min_mm").and_then(|v| v.as_f64()).unwrap_or(150.0) as f32;
    let max_mm = record.settings.get("max_mm").and_then(|v| v.as_f64()).unwrap_or(450.0) as f32;
    let first_config: ScanConfig = serde_json::from_value(record.scan_config.clone()).map_err(|e| format!("the scan settings of pass {id}: {e}"))?;
    let mut pipeline = scancraft_pipeline::pipeline_for(&first_config, min_mm, max_mm)?;
    let mut tracker = Tracker::with_config(crate::project::tracker_config(&first_config, device.as_ref()));
    let mut rows: Vec<TrackRow> = Vec::new();
    let (mut decoded, mut missing) = (0usize, 0usize);
    let main_len = std::fs::metadata(&main).map(|m| m.len()).unwrap_or(0);
    let rgb_len = rgb.as_ref().and_then(|p| std::fs::metadata(p).ok()).map(|m| m.len()).unwrap_or(0);
    let sessions = record.captures.len().max(1);
    for seg in 0..sessions {
        let s = record.captures.get(seg);
        let following = record.captures.get(seg + 1);
        let config: ScanConfig = s.and_then(|s| serde_json::from_value(s.scan_config.clone()).ok()).unwrap_or_else(|| first_config.clone());
        // This session's part of the recordings.
        let (start, end) = (s.map_or(0, |s| s.raw_offset), following.map_or(main_len, |n| n.raw_offset));
        let (rgb_start, rgb_end) = (s.map_or(0, |s| s.raw_rgb_offset), following.map_or(rgb_len, |n| n.raw_rgb_offset));
        if end <= start {
            continue;
        }
        let format = s.and_then(|s| s.stream.get("format")).and_then(|f| f.as_str()).unwrap_or(scancraft_driver_revopoint::offline::STREAM_FORMAT).to_string();
        // The live run's frames of this session, in order, by device time stamp.
        let logged: Vec<u64> = log.rows.iter().filter(|r| r.segment as usize == seg).map(|r| r.device_ms).collect();
        let mut next = 0usize;
        let t0 = std::time::Instant::now();
        let mut each = |frame: SensorFrame| -> bool {
            decoded += 1;
            if which == ReplayFrames::Logged {
                // Logged frames missing from the recording (dropped by a slow disk) are skipped.
                while next < logged.len() && logged[next] < frame.timestamp_ms {
                    next += 1;
                    missing += 1;
                }
                if next >= logged.len() {
                    return false;
                }
                if logged[next] != frame.timestamp_ms {
                    return true;
                }
                next += 1;
            }
            let started = std::time::Instant::now();
            let status = pipeline.process(&frame, &mut tracker);
            let new_points = tracker.take_new_points().len();
            let pose = status.as_ref().map_or(tracker.current_pose, |s| s.current_pose);
            rows.push(TrackRow {
                segment: seg as u32,
                frame: frame.frame_index,
                device_ms: frame.timestamp_ms,
                host_ms: t0.elapsed().as_millis() as u64,
                state: status.as_ref().map_or("no_geometry", |s| crate::capture::state_name(s.state)).to_string(),
                relocalized: status.as_ref().is_some_and(|s| s.relocalized),
                inliers: status.as_ref().map_or(0, |s| s.inliers as u64),
                rmse_mm: status.as_ref().map_or(0.0, |s| s.rmse_mm),
                conflict: status.as_ref().map_or(0.0, |s| s.visibility_conflict),
                keyframe: status.as_ref().and_then(|s| s.keyframe),
                new_points: new_points as u64,
                total_points: status.as_ref().map_or(0, |s| s.total_points_fused as u64),
                rotation: pose.rotation.to_array(),
                translation: pose.translation.to_array(),
                imu: frame.imu.map(|i| i.orientation.to_array()),
                proc_ms: started.elapsed().as_secs_f32() * 1000.0,
            });
            true
        };
        let main_reader = segment_reader(&main, start, end - start)?;
        let rgb_reader = match &rgb {
            Some(p) if rgb_end > rgb_start => Some(segment_reader(p, rgb_start, rgb_end - rgb_start)?),
            _ => None,
        };
        match format.as_str() {
            scancraft_driver::mock::MOCK_STREAM_FORMAT => {
                let cal = log.calibration.iter().find(|(n, _)| n == scancraft_driver::mock::MOCK_CALIBRATION_FILE).map(|(_, b)| b.clone()).ok_or("the simulator's calibration file is missing")?;
                let main_bytes = read_all(main_reader)?;
                let rgb_bytes = rgb_reader.map(read_all).transpose()?;
                scancraft_driver::mock::decode_recording(&cal, &main_bytes, rgb_bytes.as_deref(), &mut each)?;
            }
            scancraft_driver_revopoint::offline::STREAM_FORMAT => {
                let calibration = Arc::new(scancraft_driver_revopoint::offline::calibration_from_files(&log.calibration)?);
                scancraft_driver_revopoint::offline::decode_recording(&config, calibration, main_reader, rgb_reader, &mut each)?;
            }
            other => return Err(format!("pass {id}: recorded stream format {other} is not known to this version")),
        }
        if which == ReplayFrames::Logged {
            missing += logged.len().saturating_sub(next);
        }
    }
    let comparison = (!log.rows.is_empty()).then(|| compare(&log.rows, &rows));
    Ok(PassReplay { rows, tracker, decoded, missing, comparison })
}

/// The replayed poses against the live ones, frame by frame (by session and device time stamp).
pub fn compare(live: &[TrackRow], replayed: &[TrackRow]) -> PoseComparison {
    let by_key: std::collections::HashMap<(u32, u64), &TrackRow> = live.iter().map(|r| ((r.segment, r.device_ms), r)).collect();
    let mut c = PoseComparison::default();
    let mut dists = Vec::new();
    for r in replayed {
        let Some(l) = by_key.get(&(r.segment, r.device_ms)) else { continue };
        if l.state != r.state {
            c.state_differs += 1;
        }
        if !l.has_pose() || !r.has_pose() {
            continue;
        }
        let d = (glam::Vec3::from(l.translation) - glam::Vec3::from(r.translation)).length();
        let a = glam::Quat::from_array(l.rotation).normalize().angle_between(glam::Quat::from_array(r.rotation).normalize()).to_degrees();
        c.frames += 1;
        c.max_translation_mm = c.max_translation_mm.max(d);
        c.max_rotation_deg = c.max_rotation_deg.max(a);
        dists.push(d);
    }
    if !dists.is_empty() {
        dists.sort_by(f32::total_cmp);
        c.median_translation_mm = dists[dists.len() / 2];
    }
    c
}

/// `scancraft --replay-pass <project> <pass id> [--all-frames] [--out <folder>]`: replays the pass
/// headless and prints how its poses compare with the live run; with `--out`, writes the replay's
/// tracking log and fused points there. Returns the process exit code.
pub fn replay_pass_cli(project: &std::path::Path, id: u32, all: bool, out: Option<&std::path::Path>) -> i32 {
    // A bundle is read through an unpacked copy (removed afterwards).
    let mut _copy = None;
    let dir = if scancraft_io::project::is_bundle(project) {
        let root = std::env::temp_dir().join("scancraft-replay");
        match scancraft_io::workdir::UnpackedCopy::unpack(project, &root) {
            Ok(c) => {
                let d = c.dir().to_path_buf();
                _copy = Some(c);
                d
            }
            Err(e) => {
                eprintln!("{}: {e}", project.display());
                return 2;
            }
        }
    } else {
        match scancraft_io::project::locate(project) {
            Ok(l) => l.root,
            Err(e) => {
                eprintln!("{}: {e}", project.display());
                return 2;
            }
        }
    };
    let which = if all { ReplayFrames::All } else { ReplayFrames::Logged };
    let t0 = std::time::Instant::now();
    let r = match replay_pass(&dir, id, which) {
        Ok(r) => r,
        Err(e) => {
            eprintln!("replay of pass {id}: {e}");
            return 1;
        }
    };
    let count = |s: &str| r.rows.iter().filter(|row| row.state == s).count();
    println!(
        "pass {id}: {} frames decoded, {} processed ({} tracking, {} slow, {} lost, {} without geometry), {} keyframes, {} points, {:.1} s",
        r.decoded,
        r.rows.len(),
        count("tracking"),
        count("slow"),
        count("lost"),
        count("no_geometry"),
        r.tracker.keyframes().len(),
        r.tracker.fused_points().len(),
        t0.elapsed().as_secs_f32()
    );
    if r.missing > 0 {
        println!("  {} frames of the live run are not in the recording (dropped while recording)", r.missing);
    }
    if let Some(c) = r.comparison {
        println!(
            "  against the live run: {} frames compared, translation max {:.3} mm (median {:.3}), rotation max {:.3} deg, {} frames in another state",
            c.frames, c.max_translation_mm, c.median_translation_mm, c.max_rotation_deg, c.state_differs
        );
    }
    if let Some(out) = out {
        let csv = out.join(format!("replay-pass-{id}.csv"));
        let ply = out.join(format!("replay-pass-{id}.ply"));
        let written = std::fs::create_dir_all(out)
            .map_err(|e| e.to_string())
            .and_then(|_| scancraft_io::capture::write_tracking_log(&csv, &r.rows).map_err(|e| e.to_string()))
            .and_then(|_| scancraft_io::save_ply_binary(&scancraft_core::PointCloud::from_points(r.tracker.fused_points()), &ply).map_err(|e| e.to_string()));
        match written {
            Ok(()) => println!("  wrote {} and {}", csv.display(), ply.display()),
            Err(e) => {
                eprintln!("writing the replay: {e}");
                return 1;
            }
        }
    }
    0
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A real POP 4 recording (made with `SCANCRAFT_RECORD_DIR` before capture records existed)
    /// replayed through the pass replay: wrapped as a pass with a capture record that names the
    /// recording and the calibration, decoded with the driver's offline decoder, tracked twice
    /// with the same result. `SCANCRAFT_POP4_RECORDING=<pop4_*.bin>`,
    /// `SCANCRAFT_POP4_CAMPARAM=<camparam.yaml of that scanner>` (Full-field HD, Feature).
    #[test]
    #[ignore = "needs a POP 4 recording: SCANCRAFT_POP4_RECORDING, SCANCRAFT_POP4_CAMPARAM"]
    fn a_pop4_recording_replays_as_a_pass_and_twice_the_same() {
        let rec = PathBuf::from(std::env::var("SCANCRAFT_POP4_RECORDING").expect("SCANCRAFT_POP4_RECORDING"));
        let cam = std::fs::read(std::env::var("SCANCRAFT_POP4_CAMPARAM").expect("SCANCRAFT_POP4_CAMPARAM")).unwrap();
        let dir = std::env::temp_dir().join(format!("scancraft-pop4-replay-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        let mut log = crate::capture::CaptureLog::default();
        let config = ScanConfig::default();
        log.record.scan_config = serde_json::to_value(&config).unwrap();
        log.record.settings = serde_json::json!({ "min_mm": 250.0, "max_mm": 500.0 });
        log.record.raw_main = Some(std::path::absolute(&rec).unwrap().display().to_string());
        log.record.calibration = vec!["camparam.yaml".into()];
        log.calibration = vec![("camparam.yaml".into(), scancraft_driver_revopoint::calibration::redact_serial(&cam, ""))];
        log.record.captures.push(scancraft_io::capture::CaptureSegment {
            stream: serde_json::json!({ "format": scancraft_driver_revopoint::offline::STREAM_FORMAT }),
            scan_config: log.record.scan_config.clone(),
            ..Default::default()
        });
        crate::capture::write(&dir, 1, &mut log).unwrap();
        let a = replay_pass(&dir, 1, ReplayFrames::All).unwrap();
        let count = |s: &str| a.rows.iter().filter(|r| r.state == s).count();
        let span_s = (a.rows.last().unwrap().device_ms - a.rows[0].device_ms) as f64 / 1000.0;
        let bytes = std::fs::metadata(&rec).unwrap().len() as f64;
        println!(
            "POP 4 recording: {} frames decoded over {span_s:.1} s ({:.1} MB/s), {} tracking, {} slow, {} lost, {} points",
            a.decoded,
            bytes / 1e6 / span_s.max(1e-3),
            count("tracking"),
            count("slow"),
            count("lost"),
            a.tracker.fused_points().len()
        );
        assert!(a.decoded > 20 && count("tracking") * 2 > a.decoded, "{} of {} tracked", count("tracking"), a.decoded);
        // The first replay as the "live" log: a second one gives the same poses.
        log.rows = a.rows.clone();
        crate::capture::write(&dir, 1, &mut log).unwrap();
        let b = replay_pass(&dir, 1, ReplayFrames::Logged).unwrap();
        let c = b.comparison.unwrap();
        println!("second replay against the first: {c:?}");
        assert!(b.missing == 0 && c.frames + 2 >= a.rows.len() && c.max_translation_mm < 0.05 && c.state_differs == 0, "{c:?}");
        let _ = std::fs::remove_dir_all(&dir);
    }
}
