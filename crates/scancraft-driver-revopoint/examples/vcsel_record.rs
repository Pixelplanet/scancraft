//! Records a short VCSEL Rapid session (and optionally a Full-field HD session of the same scene)
//! through the driver, for offline stereo development:
//! `SCANCRAFT_RECORD_DIR=<dir> cargo run --release -p scancraft-driver-revopoint --example vcsel_record -- <ip> [frames] [object] [fullfield]`
//!
//! `object` is general | dark | face | outdoor. Writes `<dir>/pop4_<time>.bin` per session (raw
//! wire packets, see `device.rs`), `<dir>/camparam.yaml`, and the left/right IR of one frame as
//! PNG. The driver takes the scanner lock and switches emitters off when done.

use scancraft_driver::{DeviceTransport, LightSource, ObjectType, ScanConfig, ScannerDriver};
use scancraft_driver_revopoint::RevopointDriver;
use std::path::PathBuf;
use std::time::{Duration, Instant};

#[tokio::main]
async fn main() {
    env_logger::init();
    let args: Vec<String> = std::env::args().collect();
    let ip = args.get(1).cloned().unwrap_or_else(|| "192.168.178.34".into());
    let frames: u64 = args.get(2).and_then(|s| s.parse().ok()).unwrap_or(60);
    let object = match args.get(3).map(String::as_str).unwrap_or("general") {
        "dark" => ObjectType::Dark,
        "face" => ObjectType::Face,
        "outdoor" => ObjectType::Outdoor,
        _ => ObjectType::General,
    };
    let full_field = args.get(4).is_some_and(|s| s == "fullfield");
    let dir = PathBuf::from(std::env::var("SCANCRAFT_RECORD_DIR").expect("set SCANCRAFT_RECORD_DIR"));
    std::fs::create_dir_all(&dir).unwrap();

    let transport = DeviceTransport::WiFi { ip: ip.clone(), control_port: 80, data_port: 8871, imu_port: 9999 };
    let mut dev = RevopointDriver.connect(&transport).await.expect("connect");
    println!("connected: {} S/N {}", dev.info().model, dev.info().serial_number);
    let yaml = reqwest::get(format!("http://{ip}/cgi-bin/zx_cmd.cgi?download=/data/camparam/camparam.yaml")).await.unwrap().text().await.unwrap();
    std::fs::write(dir.join("camparam.yaml"), yaml).unwrap();

    let mut sessions = vec![(LightSource::VcselRapid, object)];
    if full_field {
        sessions.push((LightSource::FullFieldHd, ObjectType::General));
    }
    for (light, object) in sessions {
        let config = ScanConfig { light, object, ..ScanConfig::default() };
        let mut rx = dev.start_scan(&config).await.expect("start scan");
        let start = Instant::now();
        let mut n = 0u64;
        let mut first = None;
        while let Ok(Some(frame)) = tokio::time::timeout(Duration::from_secs(10), rx.recv()).await {
            n += 1;
            first.get_or_insert(Instant::now());
            if let Some(ir) = &frame.ir {
                let mean = |d: &[u8]| d.iter().map(|&v| v as f64).sum::<f64>() / d.len() as f64;
                let right = ir.right.as_ref().map(|r| mean(&r.data)).unwrap_or(0.0);
                if n % 10 == 1 {
                    println!("{} frame {n}: IR mean L {:.1} R {right:.1}, depth {}", light.label(), mean(&ir.left.data), frame.depth.is_some());
                }
                if n == 10 {
                    let tag = if light == LightSource::VcselRapid { "vcsel" } else { "ff" };
                    for (name, img) in [("left", Some(&ir.left)), ("right", ir.right.as_ref())] {
                        if let Some(img) = img {
                            image::GrayImage::from_raw(img.width, img.height, img.data.clone())
                                .unwrap()
                                .save(dir.join(format!("{tag}_{name}.png")))
                                .unwrap();
                        }
                    }
                }
            }
            if n >= frames {
                break;
            }
        }
        let fps = first.map(|f| (n.saturating_sub(1)) as f64 / f.elapsed().as_secs_f64()).unwrap_or(0.0);
        println!("{}: {n} frames, {fps:.1} fps (first frame after {:.1} s)", light.label(), first.map(|f| (f - start).as_secs_f64()).unwrap_or(0.0));
        dev.stop_scan().await.expect("stop");
    }
    dev.disconnect().await.ok();
}
