//! Live check of colour scanning against a real POP 4 (takes the scanner lock):
//! `cargo run --release -p scancraft-driver-revopoint --example color_live -- <ip> [frames] [tracking: feature|color]`
//!
//! Starts a Full-field HD scan with colour, and reports for each frame whether a colour frame was
//! attached, the depth-to-colour timestamp offset, the colour image size and the frame rate.
//! Set `SCANCRAFT_RECORD_DIR` to also record both streams.

use scancraft_driver::{DeviceTransport, ScanConfig, ScannerDriver, TrackingMode};
use scancraft_driver_revopoint::RevopointDriver;
use std::time::{Duration, Instant};

#[tokio::main]
async fn main() {
    env_logger::init();
    let args: Vec<String> = std::env::args().collect();
    let ip = args.get(1).cloned().unwrap_or_else(|| "192.168.178.34".into());
    let frames: u64 = args.get(2).and_then(|s| s.parse().ok()).unwrap_or(40);
    let tracking = if args.get(3).map(String::as_str) == Some("color") { TrackingMode::Color } else { TrackingMode::Feature };

    let transport = DeviceTransport::WiFi { ip, control_port: 80, data_port: 8871, imu_port: 9999 };
    let mut dev = RevopointDriver.connect(&transport).await.expect("connect");
    let caps = dev.capabilities();
    println!("connected: {} S/N {}, RGB camera: {}, rgb resolution {:?}", dev.info().model, dev.info().serial_number, caps.has_rgb_camera, dev.info().rgb_resolution);

    let config = ScanConfig { color_texture: true, tracking, ..ScanConfig::default() };
    let mut rx = dev.start_scan(&config).await.expect("start scan");
    let start = Instant::now();
    let (mut n, mut with_color) = (0u64, 0u64);
    let mut offsets = Vec::new();
    while let Ok(Some(frame)) = tokio::time::timeout(Duration::from_secs(10), rx.recv()).await {
        n += 1;
        match &frame.color {
            Some(c) => {
                with_color += 1;
                let off = c.timestamp_ms as i64 - frame.timestamp_ms as i64;
                offsets.push(off);
                println!(
                    "frame {:3} t={} : colour {}x{} t={} ({off:+} ms), jpeg {} kB, calibrated {}",
                    frame.frame_index,
                    frame.timestamp_ms,
                    c.image.width,
                    c.image.height,
                    c.timestamp_ms,
                    c.jpeg.as_ref().map_or(0, |j| j.len() / 1024),
                    c.camera.is_some()
                );
            }
            None => println!("frame {:3} t={} : no colour", frame.frame_index, frame.timestamp_ms),
        }
        if n >= frames {
            break;
        }
    }
    let secs = start.elapsed().as_secs_f32();
    dev.stop_scan().await.expect("stop");
    dev.disconnect().await.ok();
    offsets.sort();
    println!(
        "{n} frames in {secs:.1} s ({:.1} fps), {with_color} with colour, offset min {:?} median {:?} max {:?} ms",
        n as f32 / secs,
        offsets.first(),
        offsets.get(offsets.len() / 2),
        offsets.last()
    );
}
