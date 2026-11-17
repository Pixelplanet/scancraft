//! Shared loader for the offline colour examples: a Full-field HD depth recording
//! (`camera_id=21`, wire framing) plus an RGB recording (`camera_id=50&type_id=20`) and the
//! calibration files, paired by device timestamp.
#![allow(dead_code)]

use scancraft_core::{ColorCamera, ColorFrame, DepthMap};
use scancraft_driver_revopoint::color_stream;
use scancraft_driver_revopoint::{FrameDecoder, FrameLayout, PacketParser, StereoCalibration};
use std::path::Path;
use std::sync::Arc;

pub struct Pair {
    pub depth_ts: u64,
    pub depth: DepthMap,
    pub ir_left: scancraft_core::RgbImage,
    pub color: ColorFrame,
}

pub fn load_camera(dir: &Path) -> (ColorCamera, StereoCalibration) {
    let read = |f: &str| std::fs::read(dir.join(f)).unwrap_or_else(|e| panic!("{}: {e}", dir.join(f).display()));
    let cam = scancraft_driver_revopoint::calibration::parse_color_calibration(&read("Prgb.bin"), &read("Distort.bin"), &read("LC_RT.bin"))
        .expect("colour calibration");
    let stereo = StereoCalibration::parse(&String::from_utf8_lossy(&read("camparam.yaml"))).expect("camparam.yaml");
    (cam, stereo)
}

/// Depth frames (timestamp, depth with intrinsics, left IR) of a recording.
pub fn load_depth(path: &Path, stereo: &StereoCalibration, limit: usize) -> Vec<(u64, DepthMap, scancraft_core::RgbImage)> {
    let bytes = std::fs::read(path).expect("depth recording");
    let mut parser = PacketParser::default();
    parser.push(&bytes);
    let mut decoder = FrameDecoder::new(FrameLayout::DepthIrPair, 640, 512);
    let k = stereo.depth_intrinsics(640, 512);
    let mut out = Vec::new();
    while let Some(p) = parser.next_packet() {
        if out.len() >= limit {
            break;
        }
        if let Ok(f) = decoder.decode(&p) {
            if let Some(d) = f.depth.filter(|d| d.valid_pixel_count() > 0) {
                out.push((f.device_counter as u64, d.with_intrinsics(k), f.ir_left));
            }
        }
    }
    out
}

/// Colour frames paired with the depth frame within the matching tolerance.
pub fn load_pairs(dir: &Path, depth_file: &str, rgb_file: &str, limit: usize) -> (Vec<Pair>, Arc<ColorCamera>) {
    let (cam, stereo) = load_camera(dir);
    let cam = Arc::new(cam);
    let depth = load_depth(&dir.join(depth_file), &stereo, usize::MAX);
    let rgb = color_stream::read_recording(&std::fs::read(dir.join(rgb_file)).expect("rgb recording"));
    let mut pairs = Vec::new();
    for packet in rgb {
        if pairs.len() >= limit {
            break;
        }
        let Some((_, ts)) = color_stream::split_packet(&packet) else { continue };
        let Some((dts, d, ir)) = depth.iter().find(|(t, _, _)| t.abs_diff(ts as u64) <= color_stream::MATCH_TOLERANCE_MS) else { continue };
        let Ok(color) = color_stream::decode_packet(&packet, Some(&cam)) else { continue };
        pairs.push(Pair { depth_ts: *dts, depth: d.clone(), ir_left: ir.clone(), color });
    }
    (pairs, cam)
}

