//! Full-field HD: the device computes depth from its coded pattern; the stream carries
//! z16 depth + rectified left/right IR (fmt 3 / display type 4, 640x512, `camera_id=21`).

use super::{ModeStream, StreamSpec};
use crate::calibration::DeviceCalibration;
use crate::control::Control;
use crate::media::{DecodedFrame, FrameLayout};
use crate::modes::{full_field_preset, FullFieldPreset};
use async_trait::async_trait;
use scancraft_core::{IrImages, SensorFrame};
use scancraft_driver::{DriverError, ObjectType};
use std::sync::Arc;

pub const WIDTH: u32 = 640;

/// Rejects the planes the device sends while (re)configuring: all-zero frames for ~2 s, and right
/// after a stream starts (most often after a boot) a frame or two whose "depth" plane holds other
/// data — every pixel non-zero, nearly all out of range (seen 2026-11-01). Real depth always has
/// holes (background beyond range, edges), so a plane covering > 95 % of pixels is not depth.
pub fn plausible_depth_plane(depth: &scancraft_core::DepthMap) -> bool {
    let valid = depth.valid_pixel_count();
    valid > 0 && valid * 100 < depth.data.len() * 95
}
pub const HEIGHT: u32 = 512;

pub struct FullField {
    preset: FullFieldPreset,
    calibration: Arc<DeviceCalibration>,
}

impl FullField {
    /// Exposure / gain chosen by the user instead of the preset's.
    pub fn with_camera(mut self, exposure_us: u32, gain: u8) -> Self {
        self.preset.exposure_us = exposure_us.clamp(50, 42_000);
        self.preset.gain = gain.clamp(1, 10);
        self
    }

    pub fn new(object: ObjectType, calibration: Arc<DeviceCalibration>) -> Result<Self, DriverError> {
        let preset = full_field_preset(object)
            .ok_or_else(|| DriverError::InvalidParameter(format!("{} is not available with Full-field HD", object.label())))?;
        Ok(Self { preset, calibration })
    }
}

#[async_trait]
impl ModeStream for FullField {
    fn spec(&self) -> StreamSpec {
        StreamSpec {
            output_fmt: 3,
            display_type: 4,
            width: WIDTH,
            height: HEIGHT,
            media_query: "camera_id=21".into(),
            layout: FrameLayout::DepthIrPair,
        }
    }

    /// Revo Scan's Full-field HD setup for an object preset, as logged by Revo for this POP 4
    /// (`docs/protocol/modes.md` section 2.1): depth window, exposure, gain, frame time,
    /// threshold, multi-frame fusion; IR fill LED and VCSEL off. The firmware drives the coded
    /// pattern projector itself in this stream format.
    async fn configure(&self, ctl: &Control) -> Result<(), DriverError> {
        // Revo Scan 6 on firmware v5.3.99 (sdk.log 2026-11-07 15:22:35..38, same on v5.3.94).
        // The DSP's names: 0x488 projector return-scan control, 0xb11 RGB-LED exposure, 0x911/0x910
        // exposure / frame time, 0x903 gain, 0xb07 IR fill-LED level, 0x707 depth window, 0x705
        // background threshold, 0xb03 white LED, 0xb04 laser-diode (projector) enable.
        let p = &self.preset;
        ctl.preisp("0x488 0").await?;
        ctl.preisp("0xb11 11000").await?;
        ctl.preisp(&format!("0x911 {}", p.exposure_us)).await?;
        ctl.preisp(&format!("0x903 {:#x}", p.gain as u32 * 0x10)).await?;
        ctl.preisp("0xb07 0").await?;
        ctl.preisp("0x910 7000").await?;
        ctl.preisp(&format!("0x707 {} {}", p.z_range_mm.0, p.z_range_mm.1)).await?;
        ctl.preisp(&format!("0x705 {}", p.threshold)).await?;
        ctl.preisp(&format!("0xa04 {}", p.multi_frame_fusion as u8)).await?;
        ctl.preisp("0xb03 0").await?;
        ctl.preisp("0xb04 1").await
    }

    fn to_sensor_frame(&mut self, decoded: DecodedFrame, frame_index: u64) -> Option<SensorFrame> {
        let depth = decoded.depth.filter(|d| plausible_depth_plane(d))?;
        let rig = self.calibration.red_rig(WIDTH, HEIGHT);
        let mut frame = SensorFrame::new(frame_index, decoded.device_counter as u64);
        frame.depth = Some(depth.with_intrinsics(rig.intrinsics));
        frame.ir = Some(IrImages { left: decoded.ir_left, right: Some(decoded.ir_right), rectified: true, rig: Some(rig) });
        frame.imu = decoded.imu;
        Some(frame)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use scancraft_core::DepthMap;

    #[test]
    fn rejects_warm_up_and_non_depth_planes() {
        let n = 64 * 48;
        assert!(!plausible_depth_plane(&DepthMap::new(64, 48, 0.01, vec![0; n])), "all zero");
        assert!(!plausible_depth_plane(&DepthMap::new(64, 48, 0.01, vec![7; n])), "every pixel set");
        let mut real = vec![0u16; n];
        real[..n / 3].fill(25_000);
        assert!(plausible_depth_plane(&DepthMap::new(64, 48, 0.01, real)));
    }
}
