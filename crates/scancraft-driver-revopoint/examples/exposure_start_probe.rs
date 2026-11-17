//! Exposure set at stream start (camera override) vs. preset: mean IR brightness and coverage.
//! `cargo run --release -p scancraft-driver-revopoint --example exposure_start_probe -- <ip>`

use scancraft_driver::{DeviceTransport, ScanConfig, ScannerDriver};
use scancraft_driver_revopoint::RevopointDriver;
use std::time::Duration;

#[tokio::main]
async fn main() {
    let ip = std::env::args().nth(1).unwrap_or_else(|| "192.168.178.34".into());
    let transport = DeviceTransport::WiFi { ip: ip.clone(), control_port: 80, data_port: 8871, imu_port: 9999 };
    let mut dev = RevopointDriver.connect(&transport).await.expect("connect");
    for (exposure, gain) in [(800u32, 1u8), (6000, 1), (3000, 4), (3000, 1)] {
        let config = ScanConfig { camera_override: true, depth_exposure_us: exposure, depth_gain: gain, ..ScanConfig::default() };
        let mut rx = dev.start_scan(&config).await.expect("scan");
        let (mut ir, mut cov, mut n) = (0.0f64, 0.0f64, 0);
        let mut skip = 3;
        while n < 4 {
            let Ok(Some(f)) = tokio::time::timeout(Duration::from_secs(8), rx.recv()).await else { break };
            if skip > 0 {
                skip -= 1;
                continue;
            }
            if let (Some(i), Some(d)) = (f.ir.as_ref(), f.depth.as_ref()) {
                ir += i.left.data.iter().map(|&v| v as f64).sum::<f64>() / i.left.data.len() as f64;
                cov += d.valid_pixel_count() as f64 / d.data.len() as f64;
                n += 1;
            }
        }
        dev.stop_scan().await.ok();
        let n = n.max(1) as f64;
        println!("start with {exposure:5} us, gain {gain}: IR mean {:6.1}, depth coverage {:5.1}%", ir / n, 100.0 * cov / n);
    }
    dev.disconnect().await.ok();
}
