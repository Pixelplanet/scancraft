//! Offline laser-line extraction on a recorded `camera_id=33&type_id=20` stream.
//!
//! `cargo run --release -p scancraft-driver-revopoint --example laser_lines -- <rec.bin> <calib_dir> <out_dir> [first] [count]`
//!
//! `calib_dir` holds the blue set as downloaded from the device (`camparam.yaml`, `mapparamL.bin`,
//! `mapparamR.bin`, `metroExtra.bin`). Writes `lines_<frame>.csv` (side, raw u/v, tangent, strength,
//! peak, rectified u/v) and the decoded images as PGM for inspection.

use scancraft_driver_revopoint::calibration::BlueCalibration;
use scancraft_driver_revopoint::media::{FrameDecoder, FrameLayout, PacketParser};
use scancraft_laser::steger::{extract, StegerParams};
use std::io::Write;
use std::path::PathBuf;

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let rec = std::fs::read(&args[1]).expect("recording");
    let calib = PathBuf::from(&args[2]);
    let out = PathBuf::from(&args[3]);
    let first: usize = args.get(4).and_then(|s| s.parse().ok()).unwrap_or(0);
    let count: usize = args.get(5).and_then(|s| s.parse().ok()).unwrap_or(4);
    std::fs::create_dir_all(&out).unwrap();
    let read = |n: &str| std::fs::read(calib.join(n)).unwrap();
    let blue = BlueCalibration::parse(
        &String::from_utf8(read("camparam.yaml")).unwrap(),
        &read("mapparamL.bin"),
        &read("mapparamR.bin"),
        read("metroExtra.bin"),
    )
    .unwrap();
    let rect = &blue.rectification;

    let mut parser = PacketParser::default();
    parser.push(&rec);
    let mut decoder = FrameDecoder::new(FrameLayout::JpegIrPair, 1280, 1024);
    let params = StegerParams::default();
    let mut index = 0usize;
    while let Some(packet) = parser.next_packet() {
        if index < first {
            index += 1;
            continue;
        }
        if index >= first + count {
            break;
        }
        let t0 = std::time::Instant::now();
        let frame = decoder.decode(&packet).expect("decode");
        let t1 = t0.elapsed();
        let left = extract(&frame.ir_left.data, 1280, 1024, &params, 0);
        let right = extract(&frame.ir_right.data, 1280, 1024, &params, 0);
        let t2 = t0.elapsed();
        println!(
            "frame {index}: counter {} decode {:.1} ms, steger {:.1} ms, points L {} R {}",
            frame.device_counter,
            t1.as_secs_f64() * 1e3,
            (t2 - t1).as_secs_f64() * 1e3,
            left.len(),
            right.len()
        );
        let mut csv = std::io::BufWriter::new(std::fs::File::create(out.join(format!("lines_{index}.csv"))).unwrap());
        writeln!(csv, "side,u,v,tu,tv,strength,peak,ru,rv").unwrap();
        for (side, pts, cam) in [("L", &left, &rect.left), ("R", &right, &rect.right)] {
            for p in pts.iter() {
                let (ru, rv) = cam.rectify(p.u as f64, p.v as f64);
                writeln!(csv, "{side},{:.3},{:.3},{:.4},{:.4},{:.2},{:.2},{:.3},{:.3}", p.u, p.v, p.tu, p.tv, p.strength, p.peak, ru, rv).unwrap();
            }
        }
        for (name, img) in [("L", &frame.ir_left), ("R", &frame.ir_right)] {
            let mut f = std::fs::File::create(out.join(format!("ir{name}_{index}.pgm"))).unwrap();
            write!(f, "P5\n{} {}\n255\n", img.width, img.height).unwrap();
            f.write_all(&img.data).unwrap();
        }
        index += 1;
    }
}
