//! Replays a raw recording through exactly what the app does in a scan: the driver's mode stream
//! (`SensorFrame`s with depth, IR pair and stereo rig) -> `scancraft_pipeline::pipeline_for` ->
//! tracker. Prints per-frame state, pose-hint use and processing time, then a summary.
//! `cargo run --release -p scancraft-driver-revopoint --example pipeline_replay -- <rec.bin> <camparam.yaml> [feature|marker] [display_type]`

use scancraft_driver::{ScanConfig, TrackingMode};
use scancraft_driver_revopoint::media::{FrameDecoder, FrameLayout, PacketParser};
use scancraft_driver_revopoint::streams::{FullField, MarkerStream, ModeStream};
use scancraft_driver_revopoint::{DeviceCalibration, StereoCalibration};
use scancraft_slam::{Tracker, TrackerConfig, TrackingState};
use std::sync::Arc;
use std::time::Instant;

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let capture = std::fs::read(&args[1]).expect("recording");
    let yaml = std::fs::read_to_string(&args[2]).expect("camparam.yaml");
    let tracking = match args.get(3).map(String::as_str) {
        Some("marker") => TrackingMode::Marker,
        _ => TrackingMode::Feature,
    };
    let display_type: u32 = args.get(4).and_then(|s| s.parse().ok()).unwrap_or(4);
    let calibration = Arc::new(DeviceCalibration::new(StereoCalibration::parse(&yaml).expect("calibration")));
    let config = ScanConfig { tracking, ..ScanConfig::default() };

    let (layout, mut stream): (FrameLayout, Box<dyn ModeStream>) = match (tracking, display_type) {
        (TrackingMode::Marker, 6) => (FrameLayout::DepthIrPairOriIr, Box::new(MarkerStream::new(config.object, calibration.clone()).unwrap())),
        (TrackingMode::Marker, _) => (FrameLayout::DepthIrPair, Box::new(MarkerStream::new(config.object, calibration.clone()).unwrap())),
        _ => (FrameLayout::DepthIrPair, Box::new(FullField::new(config.object, calibration.clone()).unwrap())),
    };
    let mut pipeline = scancraft_pipeline::pipeline_for(&config, 150.0, 450.0).expect("pipeline");
    let imu_mount = glam::Quat::from_mat3(&glam::Mat3::from_cols(glam::Vec3::X, glam::Vec3::Z, -glam::Vec3::Y));
    let mut tracker = Tracker::with_config(TrackerConfig { imu_to_sensor: Some(imu_mount), ..TrackerConfig::default() });

    let mut parser = PacketParser::default();
    parser.push(&capture);
    let mut decoder = FrameDecoder::new(layout, 640, 512);
    let (mut n, mut lost, mut flips, mut prev_lost) = (0usize, 0usize, 0usize, false);
    let mut times = Vec::new();
    let mut hints = 0usize;
    // REALTIME=1: frames arrive every `FRAME_MS` (default 156 ms, the POP 4's ~6.4 fps); a frame
    // that arrives while the previous one is still being processed is skipped, like the app's
    // worker that always takes the newest frame.
    let realtime = std::env::var("REALTIME").is_ok();
    let frame_ms: f32 = std::env::var("FRAME_MS").ok().and_then(|v| v.parse().ok()).unwrap_or(156.0);
    let (mut arrival, mut busy_until, mut skipped) = (0.0f32, 0.0f32, 0usize);
    while let Some(packet) = parser.next_packet() {
        let Ok(decoded) = decoder.decode(&packet) else { continue };
        let Some(frame) = stream.to_sensor_frame(decoded, n as u64 + 1) else { continue };
        arrival += frame_ms;
        if realtime && arrival < busy_until {
            skipped += 1;
            continue;
        }
        let t = Instant::now();
        let Some(status) = pipeline.process(&frame, &mut tracker) else { continue };
        let ms = t.elapsed().as_secs_f32() * 1000.0;
        times.push(ms);
        busy_until = arrival + ms;
        let overlay = pipeline.overlay();
        let tracking_dots = overlay.as_ref().map_or(0, |o| o.points.iter().filter(|p| matches!(p.kind, scancraft_pipeline::OverlayKind::Tracking)).count());
        if tracking_dots > 0 {
            hints += 1;
        }
        let is_lost = status.state == TrackingState::Lost;
        if is_lost {
            lost += 1;
        }
        if is_lost != prev_lost {
            flips += 1;
        }
        prev_lost = is_lost;
        println!(
            "frame {n:3}: {:?}{} inliers {:5} rmse {:.3} conflict {:.2} | tracking markers {:2} stale {} | {ms:5.0} ms",
            status.state,
            if status.relocalized { " RELOC" } else { "" },
            status.inliers,
            status.rmse_mm,
            status.visibility_conflict,
            tracking_dots,
            overlay.as_ref().is_some_and(|o| o.stale),
        );
        n += 1;
    }
    println!("skipped (arrived while busy): {skipped}");
    times.sort_by(f32::total_cmp);
    let pct = |p: f32| times.get(((times.len() as f32 * p) as usize).min(times.len().saturating_sub(1))).copied().unwrap_or(0.0);
    println!(
        "\n{n} frames: lost {lost}, lost<->ok transitions {flips}, frames with tracking markers {hints}; process ms median {:.0}, p90 {:.0}, max {:.0}",
        pct(0.5),
        pct(0.9),
        pct(1.0)
    );
}
