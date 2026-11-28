//! A recorded POP 4 scan decoded offline, the way the live driver decodes it: the calibration
//! from the files the driver downloaded (a pass's `calibration/` folder), the mode's stream
//! (`streams::for_config`), `FrameDecoder`, and the colour frame of the same instant from the RGB
//! recording. Used by the app's replay of a pass (`apps/scancraft/src/replay.rs`).
//!
//! Recordings are read in chunks, so a multi-GB recording does not have to fit into memory.

use crate::calibration::{parse_blue_files, parse_color_calibration, BLUE_FILES};
use crate::color_stream::{RgbPacketParser, MATCH_TOLERANCE_MS};
use crate::media::{FrameDecoder, PacketParser};
use crate::{DeviceCalibration, StereoCalibration};
use scancraft_core::{ColorFrame, SensorFrame};
use scancraft_driver::ScanConfig;
use std::collections::VecDeque;
use std::io::Read;
use std::sync::Arc;

/// `stream.format` of a POP 4 capture record.
pub const STREAM_FORMAT: &str = "revopoint-media-v1";

const CHUNK: usize = 4 << 20;

/// The device calibration from its files: `camparam.yaml` (red set, required), `Prgb.bin`,
/// `Distort.bin`, `LC_RT.bin` (colour, optional) and `camparam_blue/*` (laser modes, optional).
pub fn calibration_from_files(files: &[(String, Vec<u8>)]) -> Result<DeviceCalibration, String> {
    let get = |name: &str| files.iter().find(|(n, _)| n == name).map(|(_, b)| b.as_slice());
    let yaml = get("camparam.yaml").ok_or("the calibration file camparam.yaml is missing")?;
    let yaml = std::str::from_utf8(yaml).map_err(|_| "camparam.yaml is not text".to_string())?;
    let red = StereoCalibration::parse(yaml).ok_or("camparam.yaml is missing pL/pR")?;
    let mut calibration = DeviceCalibration::new(red);
    if let (Some(p), Some(d), Some(l)) = (get("Prgb.bin"), get("Distort.bin"), get("LC_RT.bin")) {
        match parse_color_calibration(p, d, l) {
            Some(color) => calibration = calibration.with_color(color),
            None => log::warn!("the colour calibration files could not be read; colour frames have no camera"),
        }
    }
    let blue: Option<Vec<Vec<u8>>> = BLUE_FILES.iter().map(|p| get(&crate::calibration::blue_file_name(p)).map(<[u8]>::to_vec)).collect();
    if let Some(blue) = blue {
        match parse_blue_files(&blue) {
            Ok(b) => calibration = calibration.with_blue(b),
            Err(e) => log::warn!("blue calibration: {e}"),
        }
    }
    Ok(calibration)
}

/// Colour frames of the RGB recording in order, looked up by depth time stamp.
struct ColourTrack<R: Read> {
    reader: Option<R>,
    parser: RgbPacketParser,
    window: VecDeque<(u64, Vec<u8>)>,
    camera: Option<Arc<scancraft_core::ColorCamera>>,
}

impl<R: Read> ColourTrack<R> {
    /// Reads on until a packet later than `ts` + tolerance is waiting (or the recording ends).
    fn fill(&mut self, ts: u64) {
        while self.window.back().is_none_or(|(t, _)| *t <= ts + MATCH_TOLERANCE_MS) {
            match self.parser.next_packet() {
                Some(p) => {
                    if let Some((_, t)) = crate::color_stream::split_packet(&p) {
                        self.window.push_back((t as u64, p));
                    }
                }
                None => {
                    let Some(r) = self.reader.as_mut() else { return };
                    let mut buf = vec![0u8; CHUNK];
                    match r.read(&mut buf) {
                        Ok(0) | Err(_) => {
                            self.reader = None;
                        }
                        Ok(n) => self.parser.push(&buf[..n]),
                    }
                }
            }
        }
    }

    /// The colour frame within the live driver's tolerance of `ts`, as `ColorSync` pairs them.
    fn take(&mut self, ts: u64) -> Option<ColorFrame> {
        self.fill(ts);
        while self.window.front().is_some_and(|(t, _)| t + MATCH_TOLERANCE_MS < ts) {
            self.window.pop_front();
        }
        let i = self.window.iter().position(|(t, _)| t.abs_diff(ts) <= MATCH_TOLERANCE_MS)?;
        let (_, packet) = self.window.remove(i)?;
        crate::color_stream::decode_packet(&packet, self.camera.as_ref()).ok()
    }
}

/// Decodes a recorded scan: `main` is the depth / IR stream (`raw.bin`), `rgb` the colour stream
/// (`raw_rgb.bin`), `config` the pass's scan settings (they choose the stream layout and the
/// mode's decoding). Frames come in recorded order through `each`; `false` stops. Returns the
/// number of frames decoded.
pub fn decode_recording<R: Read, C: Read>(
    config: &ScanConfig,
    calibration: Arc<DeviceCalibration>,
    mut main: R,
    rgb: Option<C>,
    mut each: impl FnMut(SensorFrame) -> bool,
) -> Result<usize, String> {
    let mut stream = crate::streams::for_config(config, &calibration).map_err(|e| e.to_string())?;
    let spec = stream.spec();
    let mut decoder = FrameDecoder::new(spec.layout, spec.width, spec.height);
    let mut parser = PacketParser::default();
    let wants_colour = config.color_texture || config.tracking == scancraft_driver::TrackingMode::Color;
    let mut colour = rgb.filter(|_| wants_colour).map(|r| ColourTrack { reader: Some(r), parser: RgbPacketParser::default(), window: VecDeque::new(), camera: calibration.color.clone() });
    let mut index = 0u64;
    let mut buf = vec![0u8; CHUNK];
    let mut ended = false;
    loop {
        let Some(packet) = parser.next_packet() else {
            if ended {
                break;
            }
            match main.read(&mut buf) {
                Ok(0) => ended = true,
                Ok(n) => parser.push(&buf[..n]),
                Err(e) => return Err(format!("reading the recording: {e}")),
            }
            continue;
        };
        let decoded = match decoder.decode(&packet) {
            Ok(d) => d,
            Err(e) => {
                log::warn!("recorded frame not decoded: {e}");
                continue;
            }
        };
        let Some(mut frame) = stream.to_sensor_frame(decoded, index + 1) else { continue };
        index += 1;
        if let Some(c) = colour.as_mut() {
            frame.color = c.take(frame.timestamp_ms);
        }
        if !each(frame) {
            break;
        }
    }
    Ok(index as usize)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn calibration_needs_camparam_and_takes_what_else_is_there() {
        assert!(calibration_from_files(&[]).is_err());
        let yaml = "%YAML:1.0
---
deviceID: \"\"
width: \"1280\"
height: \"1024\"
pL: !!opencv-matrix
   rows: 3
   cols: 4
   dt: d
   data: [ 1.4845567582803628e+03, 0., 3.4120636749267578e+02, 0., 0.,
       1.4845567582803628e+03, 4.8513208770751953e+02, 0., 0., 0., 1.,
       0. ]
pR: !!opencv-matrix
   rows: 3
   cols: 4
   dt: d
   data: [ 1.4845567582803628e+03, 0., 9.5103775787353516e+02,
       -1.7603769057400405e+05, 0., 1.4845567582803628e+03,
       4.8513208770751953e+02, 0., 0., 0., 1., 0. ]
";
        let cal = calibration_from_files(&[("camparam.yaml".into(), yaml.as_bytes().to_vec())]).unwrap();
        assert!(cal.color.is_none() && cal.blue.is_none());
        // A recording decodes with it: an empty one gives no frames.
        let n = decode_recording(&ScanConfig::default(), Arc::new(cal), &[][..], None::<&[u8]>, |_| true).unwrap();
        assert_eq!(n, 0);

        // The serial number never reaches a saved copy.
        let sn = "D00000000B0S00E1B";
        let red = crate::calibration::redact_serial(format!("%YAML:1.0
deviceID: \"{sn}\"
width: \"1280\"").as_bytes(), sn);
        let red = String::from_utf8(red).unwrap();
        assert!(!red.contains(sn) && red.contains("deviceID: \"\"") && red.contains("width"), "{red}");
        let mut metro = sn.as_bytes().to_vec();
        metro.extend_from_slice(&[0, 1, 2, 3]);
        let blanked = crate::calibration::redact_serial(&metro, sn);
        assert_eq!(&blanked[..17], "00000000000000000".as_bytes());
        assert_eq!(&blanked[17..], &[0, 1, 2, 3]);
    }
}
