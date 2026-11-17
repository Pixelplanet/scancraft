//! VCSEL Rapid (Revo light source 4, "speckle"): the VCSEL dot projector lights the scene and the
//! device streams the IR pair (fmt 1 / display type 2, 640x512 per eye, as one JPEG on
//! `camera_id=33&type_id=20`); depth is computed on the host by stereo matching
//! (`scancraft-stereo`). See `docs/protocol/vcsel.md`.

use super::{ModeStream, StreamSpec};
use crate::calibration::DeviceCalibration;
use crate::control::Control;
use crate::media::{DecodedFrame, FrameLayout};
use async_trait::async_trait;
use scancraft_core::{IrImages, SensorFrame};
use scancraft_driver::{DriverError, ObjectType};
use std::sync::Arc;

pub const WIDTH: u32 = 640;
pub const HEIGHT: u32 = 512;

/// Device settings Revo applies for VCSEL Rapid, per object preset (`deviceScanParam.json`,
/// model `0E1`, `feature/speckle/accuracymode`; see `docs/protocol/modes.md` section 5).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct VcselPreset {
    /// IR exposure, µs (`0x911`); equals Revo's speckle gain/exposure table entry for `gain`.
    pub exposure_us: u32,
    /// Revo gain index 1..=10 (`0x903`, see [`gain_code`]).
    pub gain: u8,
    /// VCSEL luminance (`0x48d`, Revo `specialLaserLum`).
    pub luminance: u32,
    /// Working distance, mm (`0x707 zmin zmax`, and the host stereo search range).
    pub z_range_mm: (u32, u32),
}

pub fn vcsel_preset(object: ObjectType) -> Option<VcselPreset> {
    // Depth window as Revo Scan 6 sends it for VCSEL (`0x707 200 600`, capture 2026-11-01).
    let p = |exposure_us, gain, luminance| VcselPreset { exposure_us, gain, luminance, z_range_mm: (200, 600) };
    match object {
        ObjectType::General => Some(p(666, 2, 100)),
        ObjectType::Dark => Some(p(1166, 5, 100)),
        ObjectType::Face => Some(p(1000, 4, 100)),
        ObjectType::Outdoor => Some(p(60, 1, 255)),
        _ => None,
    }
}

/// `0x903` value for a Revo gain index: `0x10` per step, as in Full-field HD. Revo Scan sent
/// `0x903 0x20` for VCSEL with the General preset (gain 2) [C, capture 2026-11-01 19:28:00].
pub fn gain_code(gain: u8) -> u32 {
    gain.clamp(1, 10) as u32 * 0x10
}

pub struct Vcsel {
    preset: VcselPreset,
    calibration: Arc<DeviceCalibration>,
}

impl Vcsel {
    pub fn new(object: ObjectType, calibration: Arc<DeviceCalibration>) -> Result<Self, DriverError> {
        let preset = vcsel_preset(object)
            .ok_or_else(|| DriverError::InvalidParameter(format!("{} is not available with VCSEL Rapid", object.label())))?;
        Ok(Self { preset, calibration })
    }

    pub fn preset(&self) -> VcselPreset {
        self.preset
    }
}

#[async_trait]
impl ModeStream for Vcsel {
    /// The IR pair at 640x1024: four raw slots (an ambient and a VCSEL-lit exposure per camera);
    /// the lit slots form the stereo pair. JPEG-coded on `camera_id=33&type_id=20`, the stream
    /// Revo opens (about 170 KB per frame; the lossless `camera_id=21` variant is 620 KB and only
    /// reached 2-3 fps on firmware v5.3.99).
    fn spec(&self) -> StreamSpec {
        StreamSpec {
            output_fmt: 1,
            display_type: 2,
            width: WIDTH,
            height: HEIGHT,
            media_query: "camera_id=33&type_id=20".into(),
            layout: FrameLayout::IrSlotPairs,
        }
    }

    /// Revo Scan 6's VCSEL start, in its order (capture 2026-11-01 19:27:57 and 19:33:12, General
    /// preset): VCSEL selected (`0x3001 12`), luminance 80 then the preset's, manual exposure,
    /// `0xb11 11000`, exposure/gain, strobe `0xb08`, frame time 50 ms, laser gate on, white LED off.
    async fn configure(&self, ctl: &Control) -> Result<(), DriverError> {
        // Revo Scan 6 on firmware v5.3.99 (sdk.log 2026-11-07 15:21:33..35, General preset).
        // 0x3001 12 = projector type ZX_SPECKLE, 0x48d laser-diode level, 0xb08 laser exposure
        // (1366 with the 666 us General exposure; exposure + 700 for the others is inferred),
        // 0x910 50000 = 50 ms frame time.
        let p = &self.preset;
        ctl.preisp(&format!("0x707 {} {}", p.z_range_mm.0, p.z_range_mm.1)).await?;
        ctl.preisp("0x3001 12").await?;
        ctl.preisp("0x48d 80").await?;
        ctl.preisp("0x488 1").await?;
        ctl.preisp("0xb11 6000").await?;
        ctl.preisp(&format!("0x911 {}", p.exposure_us)).await?;
        ctl.preisp(&format!("0x903 {:#x}", gain_code(p.gain))).await?;
        ctl.preisp(&format!("0xb08 {}", p.exposure_us + 700)).await?;
        ctl.preisp("0x910 50000").await?;
        ctl.preisp(&format!("0x48d {}", p.luminance)).await?;
        ctl.preisp("0xb03 0").await?;
        ctl.preisp("0xb04 1").await
    }

    fn to_sensor_frame(&mut self, decoded: DecodedFrame, frame_index: u64) -> Option<SensorFrame> {
        let rig = self.calibration.red_rig(WIDTH, HEIGHT);
        let mut frame = SensorFrame::new(frame_index, decoded.device_counter as u64);
        // The lit slots are raw camera images.
        let (left, right) = match self.calibration.red_rectify_maps(WIDTH, HEIGHT) {
            Some(maps) => (maps.rectify(&decoded.ir_left, true), maps.rectify(&decoded.ir_right, false)),
            None => (decoded.ir_left, decoded.ir_right),
        };
        frame.ir = Some(IrImages { left, right: Some(right), rectified: true, rig: Some(rig) });
        frame.imu = decoded.imu;
        Some(frame)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn presets_and_gain_codes_match_revo() {
        assert_eq!(vcsel_preset(ObjectType::General).unwrap().exposure_us, 666);
        assert_eq!(vcsel_preset(ObjectType::Outdoor).unwrap().luminance, 255);
        assert!(vcsel_preset(ObjectType::Reflective).is_none());
        assert_eq!(gain_code(1), 0x10);
        assert_eq!(gain_code(2), 0x20);
        assert_eq!(gain_code(5), 0x50);
    }
}
