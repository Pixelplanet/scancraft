//! Live marker check against a POP 4 through the real driver path (Full-field HD + Marker):
//! `cargo run -p scancraft-markers --example marker_live -- <ip> [frames=30]`
//!
//! Set `SCANCRAFT_RECORD_DIR` to keep the raw stream for `marker_replay`. The driver takes the
//! scanner lock and switches the emitters off at the end.

use scancraft_core::Transform;
use scancraft_driver::{DeviceTransport, LightSource, ScanConfig, ScannerDriver, TrackingMode};
use scancraft_driver_revopoint::RevopointDriver;
use scancraft_markers::{MarkerTracker, MarkerTrackerConfig};

#[tokio::main]
async fn main() {
    env_logger::init();
    let args: Vec<String> = std::env::args().collect();
    let ip = args.get(1).cloned().unwrap_or_else(|| "192.168.178.34".into());
    let frames: u64 = args.get(2).and_then(|s| s.parse().ok()).unwrap_or(30);

    let transport = DeviceTransport::WiFi { ip, control_port: 80, data_port: 8871, imu_port: 9999 };
    let mut dev = RevopointDriver.connect(&transport).await.expect("connect");
    let config = ScanConfig { light: LightSource::FullFieldHd, tracking: TrackingMode::Marker, ..ScanConfig::default() };
    let mut rx = dev.start_scan(&config).await.expect("start scan");
    let mut tracker = MarkerTracker::new(MarkerTrackerConfig::default());
    let mut pose = Transform::default();
    let start = std::time::Instant::now();
    let mut n = 0;
    while let Ok(Some(frame)) = tokio::time::timeout(std::time::Duration::from_secs(10), rx.recv()).await {
        n += 1;
        let hint = tracker.estimate(&frame, &pose);
        let r = tracker.report();
        println!(
            "frame {:3}: dots L {:3} R {:3} | 3D {:3} | inliers {:3} rms {:.3} | hint {:?} | stale IR {} | map {}",
            frame.frame_index, r.detected_left, r.detected_right, r.triangulated, r.inliers, r.rms_mm, r.hint, r.stale_ir, r.map_size
        );
        if let Some(h) = hint {
            pose = h.pose;
        }
        if n >= frames {
            break;
        }
    }
    println!("{n} frames in {:.1} s, right cx offset {:?}", start.elapsed().as_secs_f32(), tracker.right_cx_offset());
    dev.stop_scan().await.expect("stop");
    dev.disconnect().await.ok();
}
