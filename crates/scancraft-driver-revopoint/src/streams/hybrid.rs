//! Hybrid HD ("speckle_stripe", Revo light source 6): both projectors, stream format S64Y8Y8
//! (fmt 5 / display type 10, 640x512, `camera_id=33&type_id=21`). Depth is computed on the host.
//! Layout and evidence: `docs/protocol/hybrid.md`.
//!
//! Each frame carries, per camera, four phase-shifted fringe images (rectified by the device)
//! and one VCSEL speckle image (raw: distorted and unrectified). The speckle pair is rectified
//! here with the device calibration so that the `SensorFrame` holds a consistent rectified set:
//! `ir` = speckle pair, `fringes` = 4-step fringe pairs, both on the red rig.

use super::{ModeStream, StreamSpec};
use crate::calibration::{DeviceCalibration, RectifyMaps};
use crate::control::Control;
use crate::media::{DecodedFrame, FrameLayout};
use async_trait::async_trait;
use scancraft_core::{FringeImages, IrImages, RgbImage, SensorFrame};
use scancraft_driver::{DriverError, ObjectType};
use std::sync::Arc;

pub const WIDTH: u32 = 640;
pub const HEIGHT: u32 = 512;
pub const OUTPUT_FMT: u32 = 5;
pub const DISPLAY_TYPE: u32 = 10;
pub const MEDIA_QUERY: &str = "camera_id=33&type_id=21";

/// Device settings Revo applies for Hybrid HD per object preset (`deviceScanParam.json`,
/// model `0E1`, `accuracymode`; see `docs/protocol/modes.md` section 5).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct HybridPreset {
    /// IR exposure, µs (`0x911`).
    pub exposure_us: u32,
    /// Revo gain index; sent as `0x903 gain*0x10`.
    pub gain: u8,
    /// Working range, mm (`0x707 zmin zmax`, also the host matcher's search window).
    pub z_range_mm: (u32, u32),
}

pub fn preset(object: ObjectType) -> Option<HybridPreset> {
    let p = |exposure_us, gain| HybridPreset { exposure_us, gain, z_range_mm: (250, 500) };
    match object {
        ObjectType::General => Some(p(3000, 1)),
        ObjectType::Dark => Some(p(5400, 4)),
        ObjectType::Face => Some(p(5100, 3)),
        _ => None,
    }
}

/// Revo Scan's Hybrid HD register sequence, as logged for this POP 4 on 2026-06-19 14:55:14..27
/// (`sdk.log_back`): trigger mode, depth window, `0x48d 80`, gain, frame time 7000 µs, exposure,
/// speckle-stripe projector luminance `0xb1b 65` (`getSpeckleStripeLaserLuminance(POP4)`),
/// `0xb1c 18000`, `0xb04 1`. Revo Scan 6 (capture 2026-11-01 19:34:13) also sends auto exposure
/// `0x488 0`, `0xb11 11000` and white LED off `0xb03 0`, which are included here.
pub async fn configure_device(ctl: &Control, p: &HybridPreset) -> Result<(), DriverError> {
    ctl.preisp(&format!("0x707 {} {}", p.z_range_mm.0, p.z_range_mm.1)).await?;
    ctl.preisp("0x48d 80").await?;
    ctl.preisp("0x488 0").await?;
    ctl.preisp(&format!("0x911 {}", p.exposure_us)).await?;
    ctl.preisp(&format!("0x903 {:#x}", p.gain as u32 * 0x10)).await?;
    ctl.preisp("0xb1b 65").await?;
    ctl.preisp("0xb11 6000").await?;
    ctl.preisp("0x910 7000").await?;
    ctl.preisp("0xb1c 18000").await?;
    ctl.preisp("0xb03 0").await?;
    ctl.preisp("0xb04 1").await
}

/// Mean fringe modulation below which the stripe projector is considered off. The stream
/// starts with ~1 s of frames whose stripe planes are dark (measured 0.2 vs ~17 when lit).
const MIN_FRINGE_MODULATION: f32 = 3.0;

pub struct Hybrid {
    preset: HybridPreset,
    calibration: Arc<DeviceCalibration>,
    maps: Arc<RectifyMaps>,
}

impl Hybrid {
    pub fn new(object: ObjectType, calibration: Arc<DeviceCalibration>) -> Result<Self, DriverError> {
        let preset = preset(object)
            .ok_or_else(|| DriverError::InvalidParameter(format!("{} is not available with Hybrid HD", object.label())))?;
        let maps = calibration
            .red_rectify_maps(WIDTH, HEIGHT)
            .ok_or_else(|| DriverError::Protocol("camparam.yaml lacks the raw camera model needed for Hybrid HD".into()))?;
        Ok(Self { preset, calibration, maps })
    }

    pub fn preset(&self) -> HybridPreset {
        self.preset
    }
}

#[async_trait]
impl ModeStream for Hybrid {
    fn spec(&self) -> StreamSpec {
        StreamSpec {
            output_fmt: OUTPUT_FMT,
            display_type: DISPLAY_TYPE,
            width: WIDTH,
            height: HEIGHT,
            media_query: MEDIA_QUERY.into(),
            layout: FrameLayout::StripeSpeckle,
        }
    }

    async fn configure(&self, ctl: &Control) -> Result<(), DriverError> {
        configure_device(ctl, &self.preset).await
    }

    fn to_sensor_frame(&mut self, decoded: DecodedFrame, frame_index: u64) -> Option<SensorFrame> {
        let mut stripes = decoded.extra_planes;
        if stripes.len() != 8 || fringe_modulation(&stripes[..4]) < MIN_FRINGE_MODULATION {
            return None; // warm-up: stripe projector not lit yet
        }
        let right = stripes.split_off(4);
        let rig = self.calibration.red_rig(WIDTH, HEIGHT);
        let mut frame = SensorFrame::new(frame_index, decoded.device_counter as u64);
        frame.ir = Some(IrImages {
            left: self.maps.rectify(&decoded.ir_left, true),
            right: Some(self.maps.rectify(&decoded.ir_right, false)),
            rectified: true,
            rig: Some(rig),
        });
        frame.fringes = Some(FringeImages { left: stripes, right });
        frame.imu = decoded.imu;
        Some(frame)
    }
}

/// Mean 4-step modulation `(|I0 - I2| + |I1 - I3|) / 2` over a sparse pixel grid.
fn fringe_modulation(images: &[RgbImage]) -> f32 {
    let [a, b, c, d] = images else { return 0.0 };
    let (mut sum, mut n) = (0.0f32, 0u32);
    for i in (0..a.data.len()).step_by(49) {
        let diff = |x: &RgbImage, y: &RgbImage| (x.data[i] as f32 - y.data[i] as f32).abs();
        sum += (diff(a, c) + diff(b, d)) * 0.5;
        n += 1;
    }
    sum / n.max(1) as f32
}
