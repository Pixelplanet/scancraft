use crate::traits::{ScannerDevice, ScannerDriver};
use crate::types::{
    DeviceInfo, DeviceParam, DeviceTransport, DiscoveredDevice, DriverError, ScanConfig,
    ScannerCapabilities,
};
use async_trait::async_trait;
use glam::{Quat, Vec3};
use scancraft_core::{DepthMap, ImuSample, IrImages, PinholeIntrinsics, PreviewFrame, RgbImage, SensorFrame};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use tokio::sync::{broadcast, mpsc};
use tokio::time::{interval, Duration};

/// Mock scanner driver generating synthetic data for development and CI testing
pub struct MockDriver;

#[async_trait]
impl ScannerDriver for MockDriver {
    fn id(&self) -> &'static str {
        "mock"
    }

    fn name(&self) -> &'static str {
        "ScanCraft Simulated Virtual Scanner"
    }

    async fn discover(&self) -> Result<Vec<DiscoveredDevice>, DriverError> {
        let virt_transport = DeviceTransport::Virtual {
            name: "simulated_scanner_0".to_string(),
        };
        Ok(vec![DiscoveredDevice {
            driver_id: "mock".to_string(),
            model_name: "ScanCraft Virtual Pro 3D".to_string(),
            serial_number: "SIM-SCAN-001".to_string(),
            available_transports: vec![virt_transport.clone()],
            selected_transport: virt_transport,
        }])
    }

    async fn connect(&self, transport: &DeviceTransport) -> Result<Box<dyn ScannerDevice>, DriverError> {
        Ok(Box::new(MockDevice::new(transport.clone())))
    }
}

pub struct MockDevice {
    info: DeviceInfo,
    capabilities: ScannerCapabilities,
    streaming: Arc<AtomicBool>,
    imu_tx: broadcast::Sender<ImuSample>,
}

impl MockDevice {
    pub fn new(transport: DeviceTransport) -> Self {
        let (imu_tx, _) = broadcast::channel(128);
        Self {
            info: DeviceInfo {
                imu_to_sensor: None,
                vendor: "ScanCraft Simulation".to_string(),
                model: "Virtual Pro 3D".to_string(),
                serial_number: "SIM-SCAN-001".to_string(),
                firmware_version: "v1.0.0-mock".to_string(),
                active_transport: transport,
                depth_resolution: (640, 512),
                rgb_resolution: Some((1280, 1024)),
                recommended_distance_range_mm: (150.0, 400.0),
            },
            capabilities: ScannerCapabilities {
                has_rgb_camera: true,
                has_imu: true,
                has_hardware_turntable_control: true,
                supported_laser_modes: vec!["Cross Blue".to_string(), "Parallel Infrared".to_string()],
                supports_single_shot: true,
                supports_continuous_scan: true,
                supports_marker_tracking: true,
                supports_feature_tracking: true,
                supported_transports: vec!["Virtual".to_string()],
                manual_exposure: true,
                modes: crate::modes::ModeSupport {
                    light_sources: vec![(crate::modes::LightSource::FullFieldHd, crate::modes::Availability::Available)],
                    tracking_modes: vec![(crate::modes::TrackingMode::Feature, crate::modes::Availability::Available)],
                    object_types: vec![(crate::modes::ObjectType::General, crate::modes::Availability::Available)],
                    combinations: Vec::new(),
                },
            },
            streaming: Arc::new(AtomicBool::new(false)),
            imu_tx,
        }
    }

    /// Generate a synthetic depth map and IR camera image of a mechanical CAD gear/vase object
    fn generate_synthetic_scan(width: u32, height: u32, phase: f32) -> (DepthMap, RgbImage) {
        let mut depth_data = vec![0u16; (width * height) as usize];
        let mut ir_data = vec![16u8; (width * height) as usize];
        let cx = (width as f32) * 0.5;
        let cy = (height as f32) * 0.5;
        let base_depth = 275.0f32; // mm

        for y in 0..height {
            for x in 0..width {
                let dx = (x as f32 - cx) * 0.42;
                let dy = (y as f32 - cy) * 0.42;

                // Sculpted mechanical ribbed cylinder/vase (no background plane)
                let obj_y = dy;
                if obj_y > -65.0 && obj_y < 65.0 {
                    let profile = 46.0 + (obj_y * 0.09).cos() * 14.0 - (obj_y * 0.04).powi(2) * 2.5;
                    let angle = dx.atan2(35.0) + phase;
                    let flutes = (angle * 6.0).sin() * 4.5 + (obj_y * 0.22 + phase).cos() * 2.0;
                    let r = (profile + flutes).max(12.0);

                    if dx.abs() < r {
                        let dz = (r * r - dx * dx).max(0.0).sqrt();
                        let depth_mm = base_depth - dz + (obj_y * 0.15).sin() * 4.0;
                        let raw_val = (depth_mm / 0.1).clamp(1000.0, 60000.0) as u16;
                        depth_data[(y * width + x) as usize] = raw_val;

                        let norm_x = dx / r;
                        let shade = (1.0 - norm_x * norm_x).sqrt();
                        let laser_line = if ((y as i32 + (dx * 0.4) as i32) % 14).abs() <= 1 {
                            65.0
                        } else {
                            0.0
                        };
                        let intensity = (shade * 150.0 + 35.0 + flutes * 5.0 + laser_line)
                            .clamp(35.0, 255.0) as u8;
                        ir_data[(y * width + x) as usize] = intensity;
                    }
                }
            }
        }
        // The object is modelled at 0.42 mm per pixel at 275 mm, i.e. a focal length of 275 / 0.42 px.
        let depth_map = DepthMap::new(width, height, 0.1, depth_data).with_intrinsics(PinholeIntrinsics {
            fx: base_depth / 0.42,
            fy: base_depth / 0.42,
            cx,
            cy,
        });
        let ir_image = RgbImage {
            width,
            height,
            channels: 1,
            data: ir_data,
        };
        (depth_map, ir_image)
    }
}

#[async_trait]
impl ScannerDevice for MockDevice {
    fn info(&self) -> &DeviceInfo {
        &self.info
    }

    fn capabilities(&self) -> &ScannerCapabilities {
        &self.capabilities
    }

    async fn start_preview(&mut self) -> Result<mpsc::Receiver<PreviewFrame>, DriverError> {
        let (tx, rx) = mpsc::channel(16);
        let streaming = self.streaming.clone();
        streaming.store(true, Ordering::SeqCst);

        tokio::spawn(async move {
            let mut ticker = interval(Duration::from_millis(66)); // ~15 FPS
            let mut frame_idx = 0u64;

            while streaming.load(Ordering::Relaxed) {
                ticker.tick().await;
                frame_idx += 1;
                let phase = (frame_idx as f32) * 0.05;
                let (_depth, ir) = Self::generate_synthetic_scan(320, 256, phase);

                let preview = PreviewFrame {
                    frame_index: frame_idx,
                    timestamp_ms: frame_idx * 66,
                    ir_image: Some(ir),
                    rgb_image: None,
                    average_distance_mm: Some(260.0),
                    imu: Some(ImuSample::default()),
                };

                if tx.send(preview).await.is_err() {
                    break;
                }
            }
        });

        Ok(rx)
    }

    async fn stop_preview(&mut self) -> Result<(), DriverError> {
        self.streaming.store(false, Ordering::SeqCst);
        Ok(())
    }

    async fn start_scan(&mut self, _config: &ScanConfig) -> Result<mpsc::Receiver<SensorFrame>, DriverError> {
        let (tx, rx) = mpsc::channel(16);
        let streaming = self.streaming.clone();
        streaming.store(true, Ordering::SeqCst);

        tokio::spawn(async move {
            let mut ticker = interval(Duration::from_millis(66)); // ~15 FPS
            let mut frame_idx = 0u64;

            while streaming.load(Ordering::Relaxed) {
                ticker.tick().await;
                frame_idx += 1;
                let phase = (frame_idx as f32) * 0.05;
                let (depth, ir) = Self::generate_synthetic_scan(640, 512, phase);

                let imu_sample = ImuSample {
                    timestamp_ms: frame_idx * 66,
                    orientation: Quat::from_rotation_z(phase * 0.5),
                    angular_velocity: Vec3::new(0.0, 0.0, 0.05),
                    linear_acceleration: Vec3::new(0.0, 0.0, 1.0),
                    euler_degrees: Vec3::new(0.0, 0.0, (phase * 0.5).to_degrees()),
                };

                let mut raw_frame = SensorFrame::new(frame_idx, frame_idx * 66);
                raw_frame.depth = Some(depth);
                raw_frame.ir = Some(IrImages { left: ir, right: None, rectified: true, rig: None });
                raw_frame.imu = Some(imu_sample);

                if tx.send(raw_frame).await.is_err() {
                    break;
                }
            }
        });

        Ok(rx)
    }

    async fn pause_scan(&mut self) -> Result<(), DriverError> {
        Ok(())
    }

    async fn resume_scan(&mut self) -> Result<(), DriverError> {
        Ok(())
    }

    async fn stop_scan(&mut self) -> Result<(), DriverError> {
        self.streaming.store(false, Ordering::SeqCst);
        Ok(())
    }

    fn imu_stream(&mut self) -> Option<broadcast::Receiver<ImuSample>> {
        Some(self.imu_tx.subscribe())
    }

    async fn set_parameter(&mut self, _param: DeviceParam) -> Result<(), DriverError> {
        Ok(())
    }

    async fn disconnect(&mut self) -> Result<(), DriverError> {
        self.streaming.store(false, Ordering::SeqCst);
        Ok(())
    }
}

#[cfg(test)]
mod intrinsics_tests {
    use super::*;

    #[test]
    fn synthetic_depth_back_projects_to_object_size() {
        let (depth, _) = MockDevice::generate_synthetic_scan(640, 512, 0.0);
        let cloud = depth.to_point_cloud(150.0, 400.0);
        assert!(!cloud.is_empty(), "mock depth carries intrinsics");
        // The vase is ~130 mm tall and sits ~230-275 mm from the scanner.
        let size = cloud.bbox.max - cloud.bbox.min;
        assert!((100.0..160.0).contains(&size.z), "height {size:?}");
        assert!(cloud.bbox.min.y > 200.0 && cloud.bbox.max.y < 290.0, "{:?}", cloud.bbox);
    }
}
