//! Offline look at a captured IR-pair stream (VCSEL Rapid exploration):
//! `cargo run --release -p scancraft-driver-revopoint --example vcsel_probe -- <capture.bin> <out_dir> [frame]`
//!
//! Handles both payload kinds seen on the POP 4 for fmt 1 / display type 2:
//! - `camera_id=21`: QuickLZ, 640x512 left y8 then right y8;
//! - `camera_id=33&type_id=20`: baseline JPEG, 640x1024 (left above right).
//!
//! Prints per-frame size / brightness and writes the chosen frame's left and right images as PNG.

use scancraft_driver_revopoint::{FrameDecoder, FrameLayout, PacketParser};
use std::path::PathBuf;

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let capture = std::fs::read(&args[1]).expect("capture");
    let out = PathBuf::from(args.get(2).map(String::as_str).unwrap_or("."));
    let pick: usize = args.get(3).and_then(|s| s.parse().ok()).unwrap_or(5);
    std::fs::create_dir_all(&out).unwrap();

    let mut parser = PacketParser::default();
    parser.push(&capture);
    let mut decoder = FrameDecoder::new(FrameLayout::IrPair, 640, 512);
    let mut n = 0;
    let mut counters = Vec::new();
    while let Some(packet) = parser.next_packet() {
        let (left, right, counter) = if packet.starts_with(&[0xff, 0xd8]) {
            let img = image::load_from_memory_with_format(&packet, image::ImageFormat::Jpeg).expect("jpeg").to_luma8();
            let (w, h) = img.dimensions();
            let data = img.into_raw();
            let half = (w * h / 2) as usize;
            let counter = if &data[..4] == b"abcd" { u32::from_le_bytes(data[20..24].try_into().unwrap()) } else { 0 };
            (data[..half].to_vec(), data[half..].to_vec(), counter)
        } else {
            match decoder.decode(&packet) {
                Ok(f) => (f.ir_left.data, f.ir_right.data, f.device_counter),
                Err(e) => {
                    println!("frame {n}: {e}");
                    n += 1;
                    continue;
                }
            }
        };
        let mean = |d: &[u8]| d.iter().map(|&v| v as f64).sum::<f64>() / d.len() as f64;
        let sat = |d: &[u8]| d.iter().filter(|&&v| v >= 250).count();
        println!(
            "frame {n}: {} B, counter {counter}, mean L {:.1} R {:.1}, saturated L {} R {}",
            packet.len(),
            mean(&left),
            mean(&right),
            sat(&left),
            sat(&right)
        );
        counters.push(counter);
        if n == pick {
            for (name, d) in [("left", &left), ("right", &right)] {
                image::GrayImage::from_raw(640, (d.len() / 640) as u32, d.clone()).unwrap().save(out.join(format!("{name}.png"))).unwrap();
            }
        }
        n += 1;
    }
    if counters.len() > 2 {
        let span = counters.last().unwrap().wrapping_sub(counters[0]) as f64;
        println!("{n} frames, counter span {span} -> {:.2} counter units/frame", span / (counters.len() - 1) as f64);
    }
}
