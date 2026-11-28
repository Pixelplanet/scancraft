pub mod mock;
pub mod modes;
pub mod recording;
pub mod registry;
pub mod traits;
pub mod types;

pub use mock::{MockDevice, MockDriver, SyntheticFrame};
pub use modes::{Availability, LightSource, ModeSupport, ObjectType, TrackingMode};
pub use recording::{RawRecorder, RecordSnapshot, RecordStats, RecordingInfo, RecordingTarget};
pub use registry::DriverRegistry;
pub use traits::{ScannerDevice, ScannerDriver};
pub use types::{
    AccuracyPreset, DeviceInfo, DeviceParam, DeviceTransport, DiscoveredDevice, DriverError,
    ScanConfig, ScannerCapabilities, TurntableMode,
};
