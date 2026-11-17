//! Stereo-matching diagnostics for one frame of a recording: lists left/right dots and, for
//! every left dot, the right candidates near its epipolar row with disparity, depth, size.
//!
//! `cargo run -p scancraft-markers --example marker_diag -- <recording.bin> <camparam.yaml> [frame=0]`

use glam::Vec2;
use scancraft_driver::ObjectType;
use scancraft_driver_revopoint::media::{FrameDecoder, FrameLayout, PacketParser};
use scancraft_driver_revopoint::streams::{MarkerStream, ModeStream};
use scancraft_driver_revopoint::{DeviceCalibration, StereoCalibration};
use scancraft_markers::{detect_markers, DetectorConfig, StereoGeometry};
use std::sync::Arc;

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let yaml = std::fs::read_to_string(&args[2]).unwrap();
    let want: u64 = args.get(3).and_then(|s| s.parse().ok()).unwrap_or(0);
    let cal = Arc::new(DeviceCalibration::new(StereoCalibration::parse(&yaml).unwrap()));
    let mut stream = MarkerStream::new(ObjectType::General, cal).unwrap();
    let mut parser = PacketParser::default();
    parser.push(&std::fs::read(&args[1]).unwrap());
    let mut dec = FrameDecoder::new(FrameLayout::DepthIrPairOriIr, 640, 512);
    let mut n = 0u64;
    while let Some(p) = parser.next_packet() {
        let Ok(d) = dec.decode(&p) else { continue };
        let Some(f) = stream.to_sensor_frame(d, n + 1) else { continue };
        n += 1;
        if n - 1 != want {
            continue;
        }
        let ir = f.ir.as_ref().unwrap();
        let rig = ir.rig.as_ref().unwrap();
        let geo = StereoGeometry::from_rig(rig, rig.disparity_offset_px);
        let cfg = DetectorConfig::default();
        let l = detect_markers(&ir.left, &cfg);
        let r = detect_markers(ir.right.as_ref().unwrap(), &cfg);
        println!("left {} right {}", l.len(), r.len());
        for m in &r {
            println!("  R ({:6.1},{:6.1}) area {:5.0} axes {:4.1}/{:4.1} peak {:3.0} c {:3.0}", m.x, m.y, m.area, m.major, m.minor, m.peak, m.contrast);
        }
        for m in &l {
            println!("L ({:6.1},{:6.1}) area {:5.0} axes {:4.1}/{:4.1} peak {:3.0} c {:3.0}", m.x, m.y, m.area, m.major, m.minor, m.peak, m.contrast);
            for q in r.iter().filter(|q| (q.y - m.y).abs() < 6.0) {
                let d = geo.disparity(Vec2::new(m.x, m.y), Vec2::new(q.x, q.y));
                let z = geo.fx * geo.baseline_mm / d;
                println!(
                    "     -> R ({:6.1},{:6.1}) dv {:5.2} disp {:6.1} Z {:6.1} area ratio {:4.2} diam {:4.1} mm",
                    q.x,
                    q.y,
                    q.y - m.y,
                    d,
                    z,
                    (m.area / q.area).max(q.area / m.area),
                    (m.major + q.major) * z / geo.fx
                );
            }
        }
        break;
    }
}
