//! Blue laser lines (Cross Lines, Single Line): the device streams the raw (unrectified)
//! 1280x1024 IR pair as one JPEG per frame on `camera_id=33&type_id=20` (fmt 1 / type 2) at
//! ~80 fps; geometry is computed on the host (`scancraft-laser`). In Cross Lines mode
//! consecutive frames alternate between the two line families.
//!
//! Device setup and register meanings: `docs/protocol/laser.md`.

use super::{ModeStream, StreamSpec};
use crate::calibration::DeviceCalibration;
use crate::control::Control;
use crate::media::{DecodedFrame, FrameLayout};
use async_trait::async_trait;
use scancraft_core::{IrImages, SensorFrame, StereoRig};
use scancraft_driver::{DriverError, LightSource, ObjectType};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

pub const WIDTH: u32 = 1280;
pub const HEIGHT: u32 = 1024;
/// Limits of the automatic laser exposure (`0xb08`), µs.
const LASER_EXPOSURE_RANGE_US: (u32, u32) = (200, 4000);

/// Which line pattern.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LinePattern {
    /// Two alternating families of ~15 parallel lines each (`0x3001 7`).
    Cross,
    /// One line (`0x3001 9`).
    Single,
}

impl LinePattern {
    /// `rk_preisp 0x3001` argument (`3DCamera.dll` `laserScanLineModeStart` @ `0x1812c2dd0`;
    /// 7 also logged by Revo for cross lines).
    pub fn line_mode(self) -> u32 {
        match self {
            LinePattern::Cross => 7,
            LinePattern::Single => 9,
        }
    }
}

/// Revo's per-object settings for a line pattern (`deviceScanParam.json`, model `0E1`,
/// `marker/crosswire_line` and `marker/single_line`, `accuracymode`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LaserPreset {
    /// Revo gain index; sent as `0x903 gain*0x10` and selects the IR fill LED level.
    pub gain: u8,
    /// `specialLaserLum`, sent as `0x48d` (logged `0x48d 255` for cross lines).
    pub laser_luminance: u32,
    /// Initial laser exposure, µs (`0xb08`); Revo starts at 950 and adjusts it during the scan.
    pub laser_exposure_us: u32,
    /// `brightnessGainMap` table that gives the IR fill LED level (`0xb07`) for `gain`.
    pub fill_table: &'static str,
}

pub fn laser_preset(pattern: LinePattern, object: ObjectType) -> Option<LaserPreset> {
    // Gains and luminances from deviceScanParam.json; the initial 0xb08 value 950 is from Revo's
    // log (general). For the other presets it is scaled with the table's "exposure" field
    // (general 500, dark 1000, reflect 500, outdoor 100) — an inference, see laser.md.
    let (gain, table, exposure_scale) = match (pattern, object) {
        (_, ObjectType::General) => (1, "lineGeneral", 1.0),
        (LinePattern::Cross, ObjectType::Dark) => (3, "lineDark", 2.0),
        (LinePattern::Single, ObjectType::Dark) => (2, "lineDark", 2.0),
        (_, ObjectType::Reflective) => (1, "lineReflect", 1.0),
        (_, ObjectType::Outdoor) => (1, "lineOutdoor", 0.2),
        _ => return None,
    };
    let laser_luminance = match pattern {
        LinePattern::Cross => 255,
        LinePattern::Single => 155,
    };
    Some(LaserPreset { gain, laser_luminance, laser_exposure_us: (950.0 * exposure_scale) as u32, fill_table: table })
}

/// IR fill LED level per gain from the device's `/data/brightnessGainMap.json`, falling back to
/// the values of this POP 4 (identical to what Revo logged: 60/30/20/11/9 for `lineGeneral`).
fn fill_led_level(map_json: Option<&str>, table: &str, gain: u8) -> u32 {
    let from_device = map_json
        .and_then(|j| serde_json::from_str::<serde_json::Value>(j).ok())
        .and_then(|v| {
            v.get(table)?.as_array()?.iter().find(|e| e.get("gain").and_then(|g| g.as_u64()) == Some(gain as u64))?.get("brightness")?.as_u64()
        });
    if let Some(level) = from_device {
        return level as u32;
    }
    let fallback: &[u32] = match table {
        "lineDark" => &[19, 9, 6, 4, 3],
        "lineReflect" => &[38, 14, 9, 7, 5],
        "lineOutdoor" => &[149, 70, 49, 36, 30],
        _ => &[60, 30, 20, 11, 9],
    };
    fallback[(gain.clamp(1, 5) - 1) as usize]
}

/// Lowest IR fill LED level used in the line modes (see `configure`).
const MIN_FILL_LED: u32 = 20;

/// Simple automatic laser exposure: keeps the brightest laser pixels below saturation and above
/// a usable level by nudging `0xb08`, like Revo (which moved it between 923 and 994 µs).
struct AutoExposure {
    ctl: Control,
    runtime: tokio::runtime::Handle,
    exposure_us: Arc<Mutex<u32>>,
    last_change: Instant,
}

impl AutoExposure {
    fn update(&mut self, left: &[u8]) {
        if self.last_change.elapsed() < Duration::from_millis(400) {
            return;
        }
        // Brightness histogram of a subsample; laser pixels are the bright tail.
        let mut hist = [0u32; 256];
        for &p in left.iter().step_by(7) {
            hist[p as usize] += 1;
        }
        let laser: u32 = hist[64..].iter().sum();
        if laser < 50 {
            return; // no laser in view
        }
        let saturated: u32 = hist[250..].iter().sum();
        let bright: u32 = hist[160..].iter().sum();
        let current = *self.exposure_us.lock().unwrap();
        let next = if saturated * 10 > laser {
            current * 92 / 100
        } else if bright * 50 < laser {
            current * 108 / 100
        } else {
            return;
        };
        let next = next.clamp(LASER_EXPOSURE_RANGE_US.0, LASER_EXPOSURE_RANGE_US.1);
        if next == current {
            return;
        }
        *self.exposure_us.lock().unwrap() = next;
        self.last_change = Instant::now();
        let ctl = self.ctl.clone();
        self.runtime.spawn(async move {
            if let Err(e) = ctl.preisp(&format!("0xb08 {next}")).await {
                log::warn!("laser exposure {next}: {e}");
            }
        });
    }
}

pub struct LaserLines {
    pattern: LinePattern,
    preset: LaserPreset,
    rig: Arc<StereoRig>,
    auto_exposure: Mutex<Option<AutoExposure>>,
    exposure_us: Arc<Mutex<u32>>,
}

impl LaserLines {
    pub fn new(light: LightSource, object: ObjectType, calibration: &DeviceCalibration) -> Result<Self, DriverError> {
        let pattern = match light {
            LightSource::CrossLines => LinePattern::Cross,
            LightSource::SingleLine => LinePattern::Single,
            other => return Err(DriverError::InvalidParameter(format!("{} is not a laser line mode", other.label()))),
        };
        let preset = laser_preset(pattern, object)
            .ok_or_else(|| DriverError::InvalidParameter(format!("{} is not available with {}", object.label(), light.label())))?;
        let rig = calibration
            .blue_rig()
            .ok_or_else(|| DriverError::Protocol("the scanner's blue (laser) calibration could not be loaded".into()))?;
        Ok(Self { pattern, preset, rig, auto_exposure: Mutex::new(None), exposure_us: Arc::new(Mutex::new(preset.laser_exposure_us)) })
    }

    pub fn pattern(&self) -> LinePattern {
        self.pattern
    }

    /// Current laser exposure, µs.
    pub fn laser_exposure_us(&self) -> u32 {
        *self.exposure_us.lock().unwrap()
    }
}

#[async_trait]
impl ModeStream for LaserLines {
    fn spec(&self) -> StreamSpec {
        StreamSpec {
            output_fmt: 1,
            display_type: 2,
            width: WIDTH,
            height: HEIGHT,
            media_query: "camera_id=33&type_id=20".into(),
            layout: FrameLayout::JpegIrPair,
        }
    }

    /// Revo Scan's cross-line setup as logged on 2026-07-05 18:38:13..16 (`sdk.log_back`), with
    /// the per-object values of `laser_preset`. `0xb04 1` switches the laser diodes on
    /// (measured 2026-11-01, see laser.md — the opposite of what older notes say).
    async fn configure(&self, ctl: &Control) -> Result<(), DriverError> {
        let p = &self.preset;
        let gain_map = ctl.cgi("download=/data/brightnessGainMap.json").await.ok();
        let fill = fill_led_level(gain_map.as_deref(), p.fill_table, p.gain);
        // Revo's table lowers the fill LED as gain and exposure rise (Dark: 6 at gain 3), which
        // keeps the markers at one brightness but far below the then much brighter laser lines;
        // markers crossed by a line were no longer detected (2-4 of ~10 triangulated, tracking
        // lost). From level 20 on, 7-9 markers carry the pose (measured 2026-11-10).
        let fill = fill.max(MIN_FILL_LED);
        // Revo Scan 6 on firmware v5.3.99, Cross Lines + Marker (sdk.log 2026-11-10 09:23:47..50;
        // the DSP log of that session names the messages): projector type, white LED off, IR
        // fill LED for the markers, laser diodes on, frame time, camera exposure (500 us for
        // General; `exposure_scale` of the preset for the others, inferred), laser level and
        // laser exposure.
        ctl.preisp("0x707 200 600").await?;
        ctl.preisp(&format!("0x3001 {}", self.pattern.line_mode())).await?;
        ctl.preisp("0xb03 0").await?;
        ctl.preisp("0x488 0").await?;
        ctl.preisp(&format!("0x903 {:#x}", p.gain as u32 * 0x10)).await?;
        ctl.preisp(&format!("0xb07 {fill}")).await?;
        ctl.preisp("0xb13 0").await?;
        ctl.preisp("0xb01 1").await?;
        ctl.preisp("0xb04 1").await?;
        ctl.preisp("0x910 11000").await?;
        ctl.preisp(&format!("0x911 {}", p.laser_exposure_us * 500 / 950)).await?;
        ctl.preisp(&format!("0x48d {}", p.laser_luminance)).await?;
        let exposure = *self.exposure_us.lock().unwrap();
        ctl.preisp(&format!("0xb08 {exposure}")).await?;
        ctl.preisp("0xb04 1").await?;
        *self.auto_exposure.lock().unwrap() = Some(AutoExposure {
            ctl: ctl.clone(),
            runtime: tokio::runtime::Handle::current(),
            exposure_us: self.exposure_us.clone(),
            last_change: Instant::now(),
        });
        Ok(())
    }

    fn to_sensor_frame(&mut self, decoded: DecodedFrame, frame_index: u64) -> Option<SensorFrame> {
        if let Some(ae) = self.auto_exposure.get_mut().unwrap().as_mut() {
            ae.update(&decoded.ir_left.data);
        }
        let mut frame = SensorFrame::new(frame_index, decoded.device_counter as u64);
        frame.ir = Some(IrImages { left: decoded.ir_left, right: Some(decoded.ir_right), rectified: false, rig: Some(self.rig.clone()) });
        frame.imu = decoded.imu;
        Some(frame)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn presets_follow_revo_tables() {
        let g = laser_preset(LinePattern::Cross, ObjectType::General).unwrap();
        assert_eq!((g.gain, g.laser_luminance, g.laser_exposure_us), (1, 255, 950));
        assert_eq!(laser_preset(LinePattern::Cross, ObjectType::Dark).unwrap().gain, 3);
        assert_eq!(laser_preset(LinePattern::Single, ObjectType::Dark).unwrap().gain, 2);
        assert_eq!(laser_preset(LinePattern::Single, ObjectType::Reflective).unwrap().laser_luminance, 155);
        assert!(laser_preset(LinePattern::Cross, ObjectType::Face).is_none());
        assert_eq!(LinePattern::Cross.line_mode(), 7);
    }

    #[test]
    fn fill_led_level_from_device_map() {
        let json = r#"{"lineGeneral":[{"brightness":60,"gain":1},{"brightness":30,"gain":2}],"lineDark":[{"brightness":19,"gain":1}]}"#;
        assert_eq!(fill_led_level(Some(json), "lineGeneral", 2), 30);
        assert_eq!(fill_led_level(Some(json), "lineDark", 1), 19);
        assert_eq!(fill_led_level(None, "lineDark", 3), 6);
        assert_eq!(fill_led_level(Some("not json"), "lineGeneral", 1), 60);
    }
}
