//! Are streamed frames actually changing? Prints device timestamp, IR checksum and depth checksum
//! of consecutive Full-field HD frames.
use scancraft_driver::{DeviceTransport, ScanConfig, ScannerDriver};
use scancraft_driver_revopoint::RevopointDriver;
use std::time::Duration;

#[tokio::main]
async fn main() {
    let ip = std::env::args().nth(1).unwrap_or_else(|| "192.168.178.34".into());
    let transport = DeviceTransport::WiFi { ip, control_port: 80, data_port: 8871, imu_port: 9999 };
    let mut dev = RevopointDriver.connect(&transport).await.expect("connect");
    let mut rx = dev.start_scan(&ScanConfig::default()).await.expect("scan");
    let sum = |d: &[u8]| d.iter().enumerate().fold(0u64, |h, (i, &v)| h.wrapping_mul(31).wrapping_add(v as u64 ^ i as u64));
    for _ in 0..12 {
        let Ok(Some(f)) = tokio::time::timeout(Duration::from_secs(5), rx.recv()).await else { break };
        let ir = f.ir.as_ref().map(|i| sum(&i.left.data)).unwrap_or(0);
        let depth = f.depth.as_ref().map(|d| d.data.iter().fold(0u64, |h, &v| h.wrapping_mul(31).wrapping_add(v as u64))).unwrap_or(0);
        let mean = f.ir.as_ref().map(|i| i.left.data.iter().map(|&v| v as f64).sum::<f64>() / i.left.data.len() as f64).unwrap_or(0.0);
        println!("ts {:9} | IR sum {ir:016x} mean {mean:6.2} | depth sum {depth:016x} valid {}", f.timestamp_ms, f.depth.as_ref().map_or(0, |d| d.valid_pixel_count()));
    }
    dev.stop_scan().await.ok();
}
