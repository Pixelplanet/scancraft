//! VCSEL time-slot check: configures VCSEL Rapid, selects the IR pair format (fmt 1 / display
//! type 2) at 640x`height` (the device lists 512, 1024 and 3072) on `camera_id=21`, and reports the
//! mean of every 640x512 plane per frame, saving one frame's planes as PNG. A taller pair may carry
//! more exposure slots per camera (All3DScan: a stereo pair must come from the same slot).
//! Run after a Full-field session (arms the VCSEL):
//! `cargo run --release -p scancraft-driver-revopoint --example pair_slots -- <ip> <height> <out_dir> [seconds]`
//!
//! Takes the scanner lock; ends the stream with `close_stream_all` and switches emitters off.

use futures::StreamExt;
use scancraft_driver::ObjectType;
use scancraft_driver_revopoint::control::Control;
use scancraft_driver_revopoint::quicklz::QuickLzDecompressor;
use scancraft_driver_revopoint::streams::{ModeStream, Vcsel};
use scancraft_driver_revopoint::{DeviceCalibration, PacketParser, StereoCalibration};
use std::sync::Arc;
use std::time::{Duration, Instant};

#[tokio::main]
async fn main() {
    let args: Vec<String> = std::env::args().collect();
    let ip = args.get(1).cloned().unwrap_or_else(|| "192.168.178.34".into());
    let height: usize = args.get(2).and_then(|s| s.parse().ok()).unwrap_or(1024);
    let out = std::path::PathBuf::from(args.get(3).cloned().unwrap_or_else(|| "pair_slots".into()));
    let seconds: f32 = args.get(4).and_then(|s| s.parse().ok()).unwrap_or(6.0);
    std::fs::create_dir_all(&out).unwrap();
    let _lock = scancraft_driver_revopoint::scanner_lock::ScannerLock::try_acquire("pair_slots").expect("scanner lock");
    let ctl = Control::new(&ip).unwrap();

    let yaml = ctl.cgi("download=/data/camparam/camparam.yaml").await.expect("calibration");
    let calibration = Arc::new(DeviceCalibration::new(StereoCalibration::parse(&yaml).expect("camparam")));
    let vcsel = Vcsel::new(ObjectType::General, calibration).unwrap();
    ctl.cgi("close_stream_all").await.ok();
    vcsel.configure(&ctl).await.expect("VCSEL setup");
    ctl.set_stream_format(1, 2, 640, height as u32).await.expect("format");
    println!("reso: {}", ctl.cgi("cam_type=mipi&get_display_reso").await.unwrap_or_default().trim());

    let url = format!("http://{ip}/cgi-bin/zx_media.cgi?camera_id=21");
    let resp = reqwest::Client::new().get(&url).send().await.expect("media stream");
    let mut body = resp.bytes_stream();
    let mut parser = PacketParser::default();
    let mut qlz = QuickLzDecompressor::new();
    let mut buf = vec![0u8; 640 * height * 2];
    let start = Instant::now();
    let mut frames = 0usize;
    while start.elapsed().as_secs_f32() < seconds {
        let Ok(Some(Ok(chunk))) = tokio::time::timeout(Duration::from_secs(5), body.next()).await else {
            println!("no data for 5 s");
            break;
        };
        parser.push(&chunk);
        while let Some(packet) = parser.next_packet() {
            let n = match qlz.decompress(&packet, &mut buf) {
                Ok(n) => n,
                Err(e) => {
                    println!("packet {} bytes: {e}", packet.len());
                    continue;
                }
            };
            frames += 1;
            let planes = n / (640 * 512);
            let means: Vec<String> = (0..planes)
                .map(|i| {
                    let p = &buf[i * 640 * 512..(i + 1) * 640 * 512];
                    format!("{:5.1}", p.iter().map(|&v| v as f64).sum::<f64>() / p.len() as f64)
                })
                .collect();
            if frames % 10 == 1 {
                println!("t={:4.1}s frame {frames:3}: {n} bytes, {planes} planes, means [{}]", start.elapsed().as_secs_f32(), means.join(" "));
            }
            if frames == 30 {
                for i in 0..planes {
                    let p = buf[i * 640 * 512..(i + 1) * 640 * 512].to_vec();
                    image::GrayImage::from_raw(640, 512, p).unwrap().save(out.join(format!("plane{i}.png"))).unwrap();
                }
            }
        }
    }
    ctl.cgi("close_stream_all").await.ok();
    drop(body);
    ctl.turn_off_emitters().await;
    println!("{frames} frames");
}
