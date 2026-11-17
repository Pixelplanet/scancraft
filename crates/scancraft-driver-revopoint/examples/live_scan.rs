//! Live end-to-end check against a real POP 4:
//! `cargo run -p scancraft-driver-revopoint --example live_scan -- <ip> [frames] [out.ply]`
//!
//! Connects through the driver, scans in structured-light mode, back-projects each depth frame
//! with the device calibration and reports frame rate, coverage and the flatness of the largest
//! plane (e.g. the table), then writes the last frame as an ASCII PLY.

use glam::Vec3;
use scancraft_driver::{DeviceTransport, ScanConfig, ScannerDriver};
use scancraft_driver_revopoint::RevopointDriver;
use std::io::Write;
use std::time::Instant;

#[tokio::main]
async fn main() {
    env_logger::init();
    let args: Vec<String> = std::env::args().collect();
    let ip = args.get(1).cloned().unwrap_or_else(|| "192.168.179.1".into());
    let frames: u64 = args.get(2).and_then(|s| s.parse().ok()).unwrap_or(30);
    let out = args.get(3).cloned().unwrap_or_else(|| "live_scan.ply".into());

    let transport = DeviceTransport::WiFi { ip, control_port: 80, data_port: 8871, imu_port: 9999 };
    let mut dev = RevopointDriver.connect(&transport).await.expect("connect");
    println!("connected: {} {} S/N {}", dev.info().vendor, dev.info().model, dev.info().serial_number);

    let mut rx = dev.start_scan(&ScanConfig::default()).await.expect("start scan");
    let start = Instant::now();
    let mut last = None;
    while let Ok(Some(frame)) = tokio::time::timeout(std::time::Duration::from_secs(10), rx.recv()).await {
        let Some(depth) = frame.depth.as_ref() else { continue };
        let cloud = depth.to_point_cloud(150.0, 400.0);
        let coverage = depth.valid_pixel_count() as f32 / depth.data.len() as f32;
        let imu = frame.imu.map(|s| format!("{:.2} g", s.linear_acceleration.length())).unwrap_or("—".into());
        println!(
            "frame {:3}: {:6} points, coverage {:4.1}%, Y {:.0}..{:.0} mm, imu accel {imu}",
            frame.frame_index,
            cloud.len(),
            coverage * 100.0,
            cloud.bbox.min.y,
            cloud.bbox.max.y
        );
        let done = frame.frame_index >= frames;
        last = Some(cloud);
        if done {
            break;
        }
    }
    let elapsed = start.elapsed().as_secs_f32();
    dev.stop_scan().await.expect("stop");
    dev.disconnect().await.ok();

    let cloud = last.expect("no frames received");
    println!("{frames} frames in {elapsed:.1} s ({:.1} fps)", frames as f32 / elapsed);

    let pts: Vec<Vec3> = cloud.points.iter().map(|p| Vec3::from(p.position)).collect();
    if let Some((n, d, inliers, rms)) = ransac_plane(&pts, 1.0) {
        println!(
            "largest plane: normal {n:.3}, offset {d:.1} mm, {inliers} inliers ({:.0}%), rms {rms:.3} mm",
            100.0 * inliers as f32 / pts.len() as f32
        );
    }

    let mut f = std::io::BufWriter::new(std::fs::File::create(&out).unwrap());
    writeln!(f, "ply\nformat ascii 1.0\nelement vertex {}\nproperty float x\nproperty float y\nproperty float z\nproperty float nx\nproperty float ny\nproperty float nz\nend_header", pts.len()).unwrap();
    for p in &cloud.points {
        let (q, n) = (p.position, p.normal);
        writeln!(f, "{} {} {} {} {} {}", q[0], q[1], q[2], n[0], n[1], n[2]).unwrap();
    }
    println!("wrote {out}");
}

/// RANSAC plane fit; returns (normal, offset, inlier count, inlier RMS distance).
fn ransac_plane(pts: &[Vec3], tol_mm: f32) -> Option<(Vec3, f32, usize, f32)> {
    if pts.len() < 3 {
        return None;
    }
    let mut seed = 0x2545F491u32;
    let mut rand = |n: usize| {
        seed ^= seed << 13;
        seed ^= seed >> 17;
        seed ^= seed << 5;
        seed as usize % n
    };
    let mut best: Option<(Vec3, f32, usize)> = None;
    for _ in 0..300 {
        let (a, b, c) = (pts[rand(pts.len())], pts[rand(pts.len())], pts[rand(pts.len())]);
        let n = (b - a).cross(c - a).normalize_or_zero();
        if n == Vec3::ZERO {
            continue;
        }
        let d = n.dot(a);
        let count = pts.iter().filter(|p| (n.dot(**p) - d).abs() < tol_mm).count();
        if best.is_none_or(|b| count > b.2) {
            best = Some((n, d, count));
        }
    }
    let (n, d, count) = best?;
    let sq: f32 = pts.iter().map(|p| n.dot(*p) - d).filter(|e| e.abs() < tol_mm).map(|e| e * e).sum();
    Some((n, d, count, (sq / count as f32).sqrt()))
}
