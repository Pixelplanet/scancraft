//! Offline analysis of a captured `zx_media.cgi` body.
//!
//! Capture with e.g. `curl -m 4 -o s21.bin "http://<ip>/cgi-bin/zx_media.cgi?camera_id=21"`, then:
//! `cargo run -p scancraft-driver-revopoint --example dump_stream -- s21.bin out_dir`
//!
//! Splits the body into `0x11223344`-framed packets, QuickLZ-decompresses each one, prints
//! per-frame statistics and writes the first frame both as a u16 depth image and as
//! two 8-bit planes so the payload layout can be checked visually.

use scancraft_driver_revopoint::quicklz::QuickLzDecompressor;
use std::path::PathBuf;

const MAGIC: [u8; 4] = [0x44, 0x33, 0x22, 0x11];

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let input = args.get(1).expect("usage: dump_stream <capture.bin> [out_dir] [width] [height] [dump_frame]");
    let out_dir = PathBuf::from(args.get(2).map(String::as_str).unwrap_or("."));
    let width: usize = args.get(3).and_then(|s| s.parse().ok()).unwrap_or(640);
    let height: usize = args.get(4).and_then(|s| s.parse().ok()).unwrap_or(512);
    let dump_frame: usize = args.get(5).and_then(|s| s.parse().ok()).unwrap_or(0);
    std::fs::create_dir_all(&out_dir).unwrap();

    let data = std::fs::read(input).unwrap();
    let mut dec = QuickLzDecompressor::new();
    let mut buf = vec![0u8; 8 * 1024 * 1024];
    let mut pos = 0usize;
    let mut frame = 0usize;

    while let Some(off) = data[pos..].windows(4).position(|w| w == MAGIC) {
        let start = pos + off;
        if start + 8 > data.len() {
            break;
        }
        let len = u32::from_le_bytes(data[start + 4..start + 8].try_into().unwrap()) as usize;
        if start + 8 + len > data.len() {
            println!("frame {frame}: truncated (len {len})");
            break;
        }
        let payload = &data[start + 8..start + 8 + len];
        pos = start + 8 + len;

        let n = match dec.decompress(payload, &mut buf) {
            Ok(n) => n,
            Err(e) => {
                println!("frame {frame}: offset {start} len {len} decompress error: {e}");
                frame += 1;
                continue;
            }
        };
        let out = &buf[..n];
        let head: String = out[..32.min(n)].iter().map(|b| format!("{b:02x}")).collect();

        let px = width * height;
        let u16s: Vec<u16> = out.chunks_exact(2).map(|c| u16::from_le_bytes([c[0], c[1]])).collect();
        let nz: Vec<u16> = u16s.iter().copied().filter(|&v| v != 0).collect();
        let (mn, mx) = (nz.iter().min().copied().unwrap_or(0), nz.iter().max().copied().unwrap_or(0));
        println!(
            "frame {frame}: comp {len} -> {n} bytes | head {head} | u16 nonzero {} min {mn} max {mx}",
            nz.len()
        );

        if frame == dump_frame {
            std::fs::write(out_dir.join("frame0.raw"), out).unwrap();
            // Interpretation A: one u16 plane, scaled for viewing.
            if u16s.len() >= px {
                let scale = if mx > mn { 255.0 / (mx - mn) as f32 } else { 1.0 };
                let img: Vec<u8> = u16s[..px]
                    .iter()
                    .map(|&v| if v == 0 { 0 } else { ((v - mn) as f32 * scale) as u8 })
                    .collect();
                save(&out_dir.join("frame0_u16.png"), width, height, img);
            }
            // Interpretation B: consecutive 8-bit planes.
            for (i, plane) in out.chunks_exact(px).enumerate().take(4) {
                save(&out_dir.join(format!("frame0_plane{i}.png")), width, height, plane.to_vec());
            }
        }
        frame += 1;
    }
    println!("{frame} frames");
}

fn save(path: &std::path::Path, w: usize, h: usize, data: Vec<u8>) {
    image::GrayImage::from_raw(w as u32, h as u32, data).unwrap().save(path).unwrap();
}
