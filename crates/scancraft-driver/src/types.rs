use serde::{Deserialize, Serialize};
use thiserror::Error;

#[derive(Error, Debug)]
pub enum DriverError {
    #[error("Device not found: {0}")]
    DeviceNotFound(String),
    #[error("Connection failed: {0}")]
    ConnectionFailed(String),
    #[error("IO error: {0}")]
    Io(#[from] std::io::Error),
    #[error("Protocol error: {0}")]
    Protocol(String),
    #[error("Parameter unsupported or invalid: {0}")]
    InvalidParameter(String),
    #[error("Device busy or streaming")]
    DeviceBusy,
    #[error("Operation timeout")]
    Timeout,
    #[error("Custom driver error: {0}")]
    Other(String),
}

/// Transport protocol connection details
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum DeviceTransport {
    /// Network connection over Wi-Fi or Ethernet
    WiFi {
        ip: String,
        control_port: u16,
        data_port: u16,
        imu_port: u16,
    },
    /// Local USB connection (HID command / UVC or Bulk stream)
    Usb {
        vid: u16,
        pid: u16,
        serial: Option<String>,
        control_interface: u8,
        stream_endpoint: u8,
    },
    /// Simulated / mock virtual device
    Virtual {
        name: String,
    },
}

impl DeviceTransport {
    pub fn description(&self) -> String {
        match self {
            DeviceTransport::WiFi { ip, .. } => format!("Wi-Fi ({})", ip),
            DeviceTransport::Usb { vid, pid, .. } => format!("USB ({:04X}:{:04X})", vid, pid),
            DeviceTransport::Virtual { name } => format!("Virtual ({})", name),
        }
    }
}

/// A discovered physical or virtual scanner, which may support multiple transports (e.g. WiFi and USB)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DiscoveredDevice {
    pub driver_id: String,
    pub model_name: String,
    pub serial_number: String,
    pub available_transports: Vec<DeviceTransport>,
    pub selected_transport: DeviceTransport,
}

/// Static and firmware information about a connected scanner
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DeviceInfo {
    pub vendor: String,
    pub model: String,
    pub serial_number: String,
    pub firmware_version: String,
    pub active_transport: DeviceTransport,
    pub depth_resolution: (u32, u32),
    pub rgb_resolution: Option<(u32, u32)>,
    pub recommended_distance_range_mm: (f32, f32),
    /// Rotation (quaternion x, y, z, w) taking the IMU's body axes to the scanner frame used by
    /// depth point clouds (+X right, +Y forward, +Z up), when the device has an IMU whose
    /// orientation arrives in `ImuSample`. Lets tracking use the IMU as a motion prior.
    pub imu_to_sensor: Option<[f32; 4]>,
}

/// Hardware features supported by the scanner
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct ScannerCapabilities {
    pub has_rgb_camera: bool,
    pub has_imu: bool,
    pub has_hardware_turntable_control: bool,
    pub supported_laser_modes: Vec<String>,
    pub supports_single_shot: bool,
    pub supports_continuous_scan: bool,
    pub supports_marker_tracking: bool,
    pub supports_feature_tracking: bool,
    pub supported_transports: Vec<String>, // e.g. ["WiFi", "USB"]
    /// Scan/tracking modes and object presets with their availability.
    pub modes: crate::modes::ModeSupport,
    /// Whether `DeviceParam::DepthExposure` / `DepthGain` actually change the depth images. `false`
    /// for devices that expose automatically (e.g. the POP 4 in Full-field HD).
    pub manual_exposure: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum TurntableMode {
    TurntableContinuous,
    TurntableStep,
    HandheldFreeform,
}

impl TurntableMode {
    pub fn label(&self) -> &'static str {
        match self {
            TurntableMode::TurntableContinuous => "Turntable (Continuous 360°)",
            TurntableMode::TurntableStep => "Turntable (Step-by-Step)",
            TurntableMode::HandheldFreeform => "Handheld (Freeform)",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum AccuracyPreset {
    HighDetail,  // 0.3 mm
    Standard,    // 0.5 mm
    FastPreview, // 1.0 mm
}

impl AccuracyPreset {
    /// Spacing of the fused surface points. Pitches finer than the sensor's depth noise
    /// (~0.25 mm RMS on the POP 4) only add noise, so the finest preset is 0.3 mm.
    pub fn point_pitch_mm(&self) -> f32 {
        match self {
            AccuracyPreset::HighDetail => 0.3,
            AccuracyPreset::Standard => 0.5,
            AccuracyPreset::FastPreview => 1.0,
        }
    }

    pub fn label(&self) -> &'static str {
        match self {
            AccuracyPreset::HighDetail => "High Detail (0.3 mm)",
            AccuracyPreset::Standard => "Standard (0.5 mm)",
            AccuracyPreset::FastPreview => "Fast Preview (1.0 mm)",
        }
    }
}

/// Configuration submitted to start a scanning session
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ScanConfig {
    /// Light source / depth method (Revo's "scan mode").
    pub light: crate::modes::LightSource,
    /// Frame alignment method.
    pub tracking: crate::modes::TrackingMode,
    /// Material / subject preset (exposure, gain, depth range, filtering).
    pub object: crate::modes::ObjectType,
    pub turntable_mode: TurntableMode,
    pub accuracy: AccuracyPreset,
    pub color_texture: bool,
    pub fps: u32,
    pub laser_power: u8, // 1..10
    pub depth_exposure_us: u32,
    pub depth_gain: u8,
    pub rgb_exposure_us: Option<u32>,
    pub rgb_gain: Option<u8>,
    /// The user changed exposure/gain from the object preset: drivers use `depth_exposure_us`
    /// and `depth_gain` at stream start instead of the preset values.
    pub camera_override: bool,
}

impl Default for ScanConfig {
    fn default() -> Self {
        Self {
            // Handheld unless the user picks a turntable: turntable capture removes everything
            // but the object on the turntable (scancraft-pipeline `TurntableFilter`).
            turntable_mode: TurntableMode::HandheldFreeform,
            light: crate::modes::LightSource::FullFieldHd,
            tracking: crate::modes::TrackingMode::Feature,
            object: crate::modes::ObjectType::General,
            accuracy: AccuracyPreset::Standard,
            color_texture: false,
            fps: 15,
            laser_power: 5,
            depth_exposure_us: 10000,
            depth_gain: 2,
            rgb_exposure_us: Some(15000),
            rgb_gain: Some(1),
            camera_override: false,
        }
    }
}

/// Runtime parameters adjustable while connected or scanning
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum DeviceParam {
    DepthExposure(u32),
    DepthGain(u8),
    LaserPower(u8),
    LaserMode(String),
    RgbExposure(u32),
    RgbGain(u8),
    TurntableSpeed(u8),
    TurntableRotationAngle(f32),
}
