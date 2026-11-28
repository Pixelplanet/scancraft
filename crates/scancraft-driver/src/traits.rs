use crate::types::{
    DeviceInfo, DeviceParam, DeviceTransport, DiscoveredDevice, DriverError, ScanConfig,
    ScannerCapabilities,
};
use async_trait::async_trait;
use scancraft_core::{ImuSample, PreviewFrame, SensorFrame};
use tokio::sync::{broadcast, mpsc};

/// Top-level factory and discovery interface for a scanner family
#[async_trait]
pub trait ScannerDriver: Send + Sync {
    /// Unique driver identifier (e.g. "revopoint", "creality", "mock")
    fn id(&self) -> &'static str;

    /// Human-readable vendor / brand display name
    fn name(&self) -> &'static str;

    /// Discover available devices over USB, local network broadcast, etc.
    async fn discover(&self) -> Result<Vec<DiscoveredDevice>, DriverError>;

    /// Connect to a specific device via selected transport (Wi-Fi or USB)
    async fn connect(&self, transport: &DeviceTransport) -> Result<Box<dyn ScannerDevice>, DriverError>;
}

/// Active connection instance to a physical or virtual 3D scanner
#[async_trait]
pub trait ScannerDevice: Send + Sync {
    /// Static device information (serial, model, firmware, resolutions, active transport)
    fn info(&self) -> &DeviceInfo;

    /// Hardware capabilities supported by this scanner
    fn capabilities(&self) -> &ScannerCapabilities;

    /// Start low-latency preview stream (RGB and/or Depth IR) for visual aiming
    async fn start_preview(&mut self) -> Result<mpsc::Receiver<PreviewFrame>, DriverError>;

    /// Stop preview stream
    async fn stop_preview(&mut self) -> Result<(), DriverError>;

    /// Start full scanning capture session with projector / lasers enabled
    async fn start_scan(&mut self, config: &ScanConfig) -> Result<mpsc::Receiver<SensorFrame>, DriverError>;

    /// Pause active scanning
    async fn pause_scan(&mut self) -> Result<(), DriverError>;

    /// Resume active scanning
    async fn resume_scan(&mut self) -> Result<(), DriverError>;

    /// Stop scanning and power down lasers/projectors
    async fn stop_scan(&mut self) -> Result<(), DriverError>;

    /// Real-time IMU sample stream if supported by hardware (e.g. 138 Hz POP 4 gyro)
    fn imu_stream(&mut self) -> Option<broadcast::Receiver<ImuSample>>;

    /// Update runtime hardware parameters (exposure, laser power, line mode, etc.)
    async fn set_parameter(&mut self, param: DeviceParam) -> Result<(), DriverError>;

    /// Gracefully disconnect and release device handles
    async fn disconnect(&mut self) -> Result<(), DriverError>;

    /// Records the raw stream(s) of the next `start_scan` into `target` (taken by that call;
    /// `None` cancels). Returns `false` if this device has no stream to record. Recording must
    /// never slow the stream down: see [`crate::recording`].
    fn set_recording(&mut self, _target: Option<crate::recording::RecordingTarget>) -> bool {
        false
    }

    /// For the capture record, after `start_scan` succeeded: the stream layout, the setup the
    /// device was sent, the calibration files needed to decode a recording offline and the files
    /// being recorded. `None` when the device has nothing to say.
    fn recording_info(&self, _config: &ScanConfig) -> Option<crate::recording::RecordingInfo> {
        None
    }
}
