//! Cross Lines with the dot projector (firmware v5.3.99; what Revo Scan runs for Cross Lines with
//! Color tracking): every frame carries both cameras lit by the blue cross lines *and* by the IR
//! dot pattern (fmt 7 / display type 4, 1280x1024, `camera_id=33&type_id=22`). The lines give the
//! surface, the dots dense depth for tracking. See `docs/protocol/laser.md`.

use super::laser::{laser_preset, LaserPreset, LinePattern};
use super::{ModeStream, StreamSpec};
use crate::calibration::DeviceCalibration;
use crate::control::Control;
use crate::media::{DecodedFrame, FrameLayout};
use async_trait::async_trait;
use scancraft_core::{IrImages, SensorFrame, StereoRig};
use scancraft_driver::{DriverError, ObjectType};
use std::sync::Arc;

pub const WIDTH: u32 = 1280;
pub const HEIGHT: u32 = 1024;

pub struct LinesWithDots {
    preset: LaserPreset,
    /// Blue (laser) calibration: raw camera models for the line planes.
    line_rig: Arc<StereoRig>,
    calibration: Arc<DeviceCalibration>,
}

impl LinesWithDots {
    pub fn new(object: ObjectType, calibration: Arc<DeviceCalibration>) -> Result<Self, DriverError> {
        let preset = laser_preset(LinePattern::Cross, object)
            .ok_or_else(|| DriverError::InvalidParameter(format!("{} is not available with Cross Lines", object.label())))?;
        let line_rig = calibration
            .blue_rig()
            .ok_or_else(|| DriverError::Protocol("the scanner's blue (laser) calibration could not be loaded".into()))?;
        if calibration.red_rectify_maps(WIDTH / 2, HEIGHT / 2).is_none() {
            return Err(DriverError::Protocol("the scanner's calibration has no raw camera models (needed to rectify the dot images)".into()));
        }
        Ok(Self { preset, line_rig, calibration })
    }
}

#[async_trait]
impl ModeStream for LinesWithDots {
    fn spec(&self) -> StreamSpec {
        StreamSpec {
            output_fmt: 7,
            display_type: 4,
            width: WIDTH,
            height: HEIGHT,
            media_query: "camera_id=33&type_id=22".into(),
            layout: FrameLayout::DotsAndLines,
        }
    }

    /// Revo Scan 6 on firmware v5.3.99, Cross Lines + Color (sdk.log 2026-11-07 15:20:03..06).
    /// The DSP sets projector type 14 (`ZX_MULTI_CROSS_LINE_ADD_SPECKLE`) itself for this format;
    /// Revo sends it again. `0xb1b` = dot-projector level, `0x48d 255 1` = line level,
    /// `0xb08` = laser exposure, `0xb1c` = minimum trigger interval.
    async fn configure(&self, ctl: &Control) -> Result<(), DriverError> {
        let p = &self.preset;
        ctl.preisp("0x707 200 600").await?;
        ctl.preisp("0x3001 14").await?;
        ctl.preisp("0x488 0").await?;
        ctl.preisp(&format!("0x903 {:#x}", p.gain as u32 * 0x10)).await?;
        ctl.preisp("0xb11 6000").await?;
        ctl.preisp("0xb01 0").await?;
        ctl.preisp("0x910 10000").await?;
        ctl.preisp("0x911 300").await?;
        ctl.preisp("0xb1b 170").await?;
        ctl.preisp("0x48d 255 1").await?;
        ctl.preisp(&format!("0xb08 {}", p.laser_exposure_us)).await?;
        ctl.preisp("0xb1c 50000").await?;
        ctl.preisp("0xb04 1").await
    }

    fn to_sensor_frame(&mut self, decoded: DecodedFrame, frame_index: u64) -> Option<SensorFrame> {
        let mut dots = decoded.extra_planes.into_iter();
        let (dots_left, dots_right) = (dots.next()?, dots.next()?);
        let maps = self.calibration.red_rectify_maps(WIDTH / 2, HEIGHT / 2)?;
        let mut frame = SensorFrame::new(frame_index, decoded.device_counter as u64);
        frame.ir = Some(IrImages { left: decoded.ir_left, right: Some(decoded.ir_right), rectified: false, rig: Some(self.line_rig.clone()) });
        frame.dots = Some(IrImages {
            left: maps.rectify(&dots_left, true),
            right: Some(maps.rectify(&dots_right, false)),
            rectified: true,
            rig: Some(self.calibration.red_rig(WIDTH / 2, HEIGHT / 2)),
        });
        frame.imu = decoded.imu;
        Some(frame)
    }
}
