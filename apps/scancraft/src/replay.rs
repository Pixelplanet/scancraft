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
