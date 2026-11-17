//! Full-field HD with Marker / Global Marker tracking: Revo's "surface" stream in its marker
//! variant (fmt 3 / display type 6 `z16y8y8p`, 640x512, `camera_id=21`) plus the IR fill-LED
//! setup Revo sends for marker modes. See `docs/protocol/markers.md` for the evidence.
//!
//! A separate module rather than an option on [`super::FullField`]: the device setup (fill LED,
//! display type), the plane layout and the frame handling (duplicate suppression) all differ,
//! and the Feature-tracking stream stays untouched.

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
pub const HEIGHT: u32 = 512;
/// Display type 4 (z16 + IR pair), as for plain Full-field HD. Revo uses 6 in marker modes
/// (`set_display_type=6`, sdk.log_back 2026-07-05 18:42:28), which adds a 2 B/px plane that was
/// never refreshed in any of our sessions (docs/protocol/markers.md) while costing 50 % more
/// WiFi bandwidth: on 2026-11-01 type 6 delivered 4.7 packets/s at 1.15 MB each, i.e. marker
/// updates at ~2.3 Hz (every other packet repeats the IR planes).
pub const DISPLAY_TYPE: u32 = 4;
/// Fill-LED luminance Revo starts a Full-field HD marker scan with (`0xb07 34`, 18:42:36; its
/// auto exposure later moved it with the exposure: 4600 µs -> 33, 5100 -> 21, 5400 -> 15).
pub const FILL_LED_LUMA: u32 = 34;

pub struct MarkerStream {
    preset: FullFieldPreset,
    calibration: Arc<DeviceCalibration>,
    /// Fingerprint of the last delivered frame. In this format packets come in pairs ~56 ms
    /// apart whose IR planes are byte-identical; in one session the depth was identical too
    /// (then the copy is dropped here), in another the depth was new (kept; the marker tracker
    /// ignores the repeated IR).
    last_fingerprint: Option<u64>,
}

impl MarkerStream {
    /// Exposure / gain chosen by the user instead of the preset's.
    pub fn with_camera(mut self, exposure_us: u32, gain: u8) -> Self {
        self.preset.exposure_us = exposure_us.clamp(50, 42_000);
        self.preset.gain = gain.clamp(1, 10);
        self
    }

    /// Revo offers Full-field HD marker tracking for the General object preset only (S1).
    pub fn new(object: ObjectType, calibration: Arc<DeviceCalibration>) -> Result<Self, DriverError> {
        if object != ObjectType::General {
            return Err(DriverError::InvalidParameter(format!("{} is not available with Full-field HD marker tracking", object.label())));
        }
        let preset = full_field_preset(object).ok_or_else(|| DriverError::InvalidParameter("no Full-field HD preset".into()))?;
        Ok(Self { preset, calibration, last_fingerprint: None })
    }
}

/// FNV-1a over a strided sample of the depth plane and the left IR plane. The first depth row is
/// skipped: the in-band header area differs between the two copies of a frame.
fn fingerprint(frame: &DecodedFrame) -> u64 {
    let mut h = 0xcbf29ce484222325u64;
    let mut mix = |v: u64| {
        h ^= v;
        h = h.wrapping_mul(0x100000001b3);
    };
    if let Some(d) = &frame.depth {
        d.data.iter().skip(d.width as usize).step_by(7).for_each(|&v| mix(v as u64));
    }
    frame.ir_left.data.iter().step_by(5).for_each(|&v| mix(v as u64));
    h
}

#[async_trait]
impl ModeStream for MarkerStream {
    fn spec(&self) -> StreamSpec {
        StreamSpec {
            output_fmt: 3,
            display_type: DISPLAY_TYPE,
            width: WIDTH,
            height: HEIGHT,
            media_query: "camera_id=21".into(),
            layout: FrameLayout::DepthIrPair,
        }
    }

    /// Revo's Full-field HD + marker setup as logged for this POP 4 (sdk.log_back 2026-07-05
    /// 18:42:35.405 .. 18:42:36.507, `ledAndLaserCtrl` / `laserParamCollection` /
    /// `setProjectorCtrlInWP`): fill light type red/IR (`0xb13 1`), fill light on (`0xb01 1`),
    /// frame time 7000, exposure / gain of the preset, fill-LED luminance (`0xb07`), RGB LED off
    /// (`0xb03 0`), `0xb05 0`, depth window, threshold, no multi-frame fusion, laser gate off.
    /// Revo sends these after the stream started; sending them before made no observable
    /// difference in our tests (`docs/protocol/markers.md`).
    async fn configure(&self, ctl: &Control) -> Result<(), DriverError> {
        // Revo Scan 6 on firmware v5.3.99 (sdk.log 2026-11-07 15:22:35..38): as Full-field HD, plus
        // the IR fill LED (0xb07 level, 0xb13 wavelength switch, 0xb01 enable) that lights the
        // retro-reflective markers, and the projector's white pattern off (0xb05 0).
        let p = &self.preset;
        ctl.preisp("0x488 0").await?;
        ctl.preisp("0xb11 11000").await?;
        ctl.preisp(&format!("0x911 {}", p.exposure_us)).await?;
        ctl.preisp(&format!("0x903 {:#x}", p.gain as u32 * 0x10)).await?;
        ctl.preisp(&format!("0xb07 {FILL_LED_LUMA}")).await?;
        ctl.preisp("0xb13 1").await?;
        ctl.preisp("0xb01 1").await?;
        ctl.preisp("0x910 7000").await?;
        ctl.preisp("0xb03 0").await?;
        ctl.preisp("0xb05 0").await?;
        ctl.preisp(&format!("0x707 {} {}", p.z_range_mm.0, p.z_range_mm.1)).await?;
        ctl.preisp(&format!("0x705 {}", p.threshold)).await?;
        ctl.preisp(&format!("0xa04 {}", p.multi_frame_fusion as u8)).await?;
        ctl.preisp("0xb04 1").await
    }

    fn to_sensor_frame(&mut self, decoded: DecodedFrame, frame_index: u64) -> Option<SensorFrame> {
        // Warm-up and reconfiguration frames (see `full_field::plausible_depth_plane`).
        if decoded.depth.as_ref().is_none_or(|d| !super::full_field::plausible_depth_plane(d)) {
            return None;
        }
        let fp = fingerprint(&decoded);
        if self.last_fingerprint == Some(fp) {
            return None;
        }
        self.last_fingerprint = Some(fp);
        let depth = decoded.depth?;
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
    use crate::calibration::StereoCalibration;
    use crate::media::decode_planes;

    fn calibration() -> Arc<DeviceCalibration> {
        let mut p_left = [0.0; 12];
        let mut p_right = [0.0; 12];
        for p in [&mut p_left, &mut p_right] {
            p[0] = 1484.0;
            p[5] = 1484.0;
            p[2] = 341.0;
            p[6] = 485.0;
            p[10] = 1.0;
        }
        p_right[2] = 951.0;
        p_right[3] = -1484.0 * 118.6;
        Arc::new(DeviceCalibration::new(StereoCalibration { width: 1280, height: 1024, p_left, p_right, raw: None }))
    }

    #[test]
    fn decodes_z16y8y8p_and_drops_duplicate_frames() {
        let mut s = MarkerStream::new(ObjectType::General, calibration()).unwrap();
        assert!(MarkerStream::new(ObjectType::Dark, calibration()).is_err());
        let (w, h) = (WIDTH as usize, HEIGHT as usize);
        let px = w * h;
        let mut raw = vec![0u8; px * 6];
        for i in 0..px {
            // Real depth has holes: fill half the plane.
            if i % 2 == 0 {
                raw[2 * i..2 * i + 2].copy_from_slice(&30000u16.to_le_bytes());
            }
            raw[2 * px + i] = 50;
            raw[3 * px + i] = 60;
            raw[4 * px + 2 * i] = 99; // OriIR plane: ignored
        }
        let decoded = decode_planes(FrameLayout::DepthIrPairOriIr, w, h, &raw).unwrap();
        assert_eq!(decoded.ir_left.data[px - 1], 50);
        assert_eq!(decoded.ir_right.data[px - 1], 60);
        let f = s.to_sensor_frame(decoded.clone(), 1).expect("first frame");
        assert!(f.ir.as_ref().unwrap().rig.is_some() && f.depth.as_ref().unwrap().intrinsics.is_some());
        assert!(s.to_sensor_frame(decoded.clone(), 2).is_none(), "identical copy is dropped");
        let mut changed = decoded;
        changed.ir_left.data[1000] = 51;
        assert!(s.to_sensor_frame(changed, 3).is_some());
    }
}
