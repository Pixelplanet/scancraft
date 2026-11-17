//! One module per scan mode (light source). Each says which stream it needs, how to set the
//! device up for it, and how to turn decoded frames into `SensorFrame`s. `device.rs` does the
//! generic part (format switch, HTTP streaming, decoding thread, recording).
//!
//! Adding a mode: implement [`ModeStream`] in a new module and return it from [`for_light`].

pub mod full_field;
pub mod hybrid;
pub mod laser;
pub mod lines_dots;
pub mod marker;
pub mod vcsel;

pub use full_field::FullField;
pub use hybrid::Hybrid;
pub use laser::LaserLines;
pub use lines_dots::LinesWithDots;
pub use marker::MarkerStream;
pub use vcsel::Vcsel;

use crate::calibration::DeviceCalibration;
use crate::control::Control;
use crate::media::{DecodedFrame, FrameLayout};
use async_trait::async_trait;
use scancraft_core::SensorFrame;
use scancraft_driver::{DriverError, LightSource, ScanConfig};
use std::sync::Arc;

/// The stream a mode reads.
#[derive(Debug, Clone, PartialEq)]
pub struct StreamSpec {
    /// `set_depth_output_fmt`.
    pub output_fmt: u32,
    /// `set_display_type` (bytes per pixel / plane set).
    pub display_type: u32,
    pub width: u32,
    pub height: u32,
    /// Query of `/cgi-bin/zx_media.cgi`, e.g. `camera_id=21`.
    pub media_query: String,
    pub layout: FrameLayout,
}

#[async_trait]
pub trait ModeStream: Send + Sync {
    fn spec(&self) -> StreamSpec;

    /// The mode's DSP messages (projector, exposure, depth window, LEDs), in Revo Scan's order.
    /// Sent once the stream has delivered its first frame: the format switch before it reloads
    /// the DSP, which forgets everything written earlier.
    async fn configure(&self, ctl: &Control) -> Result<(), DriverError>;

    /// Turns a decoded frame into a `SensorFrame` (attaching intrinsics, stereo rig, ...).
    /// `None` drops the frame (e.g. warm-up frames without data). Runs on the decode thread.
    fn to_sensor_frame(&mut self, decoded: DecodedFrame, frame_index: u64) -> Option<SensorFrame>;
}

/// The stream for a scan configuration. Light sources without an implementation report
/// `NotImplemented` in `modes::pop4_modes()` and are refused before getting here.
pub fn for_config(config: &ScanConfig, calibration: &Arc<DeviceCalibration>) -> Result<Box<dyn ModeStream>, DriverError> {
    use scancraft_driver::TrackingMode;
    if config.light == LightSource::FullFieldHd && matches!(config.tracking, TrackingMode::Marker | TrackingMode::GlobalMarker) {
        return Ok(Box::new(camera(MarkerStream::new(config.object, calibration.clone())?, config, MarkerStream::with_camera)));
    }
    // Cross Lines with Color tracking runs on the combined lines + dots stream (as in Revo Scan):
    // colour features need dense depth, which the lines alone do not give.
    if config.light == LightSource::CrossLines && config.tracking == TrackingMode::Color {
        return Ok(Box::new(LinesWithDots::new(config.object, calibration.clone())?));
    }
    for_light(config.light, config, calibration)
}

pub fn for_light(light: LightSource, config: &ScanConfig, calibration: &Arc<DeviceCalibration>) -> Result<Box<dyn ModeStream>, DriverError> {
    match light {
        LightSource::FullFieldHd => Ok(Box::new(camera(FullField::new(config.object, calibration.clone())?, config, FullField::with_camera))),
        LightSource::CrossLines | LightSource::SingleLine => Ok(Box::new(LaserLines::new(light, config.object, calibration)?)),
        LightSource::VcselRapid => Ok(Box::new(Vcsel::new(config.object, calibration.clone())?)),
        LightSource::HybridHd => Ok(Box::new(Hybrid::new(config.object, calibration.clone())?)),
        other => Err(DriverError::InvalidParameter(format!("{} is not implemented for the POP 4 yet", other.label()))),
    }
}

/// Applies the user's exposure / gain when they changed it from the preset (`camera_override`).
fn camera<S>(stream: S, config: &ScanConfig, with_camera: fn(S, u32, u8) -> S) -> S {
    if config.camera_override {
        with_camera(stream, config.depth_exposure_us, config.depth_gain)
    } else {
        stream
    }
}

