//! Offline marker evaluation on a raw POP 4 recording (wire framing, as written by
//! `SCANCRAFT_RECORD_DIR` or `scancraft-driver-revopoint/examples/marker_record.rs`).
//!
//! `cargo run -p scancraft-markers --example marker_replay -- <recording.bin> <camparam.yaml> [display_type=6] [map_out.json]`
//!
//! Prints per frame: dots found left/right, triangulated markers, the pose hint, map size; then
//! for a static scene the repeatability of every marker (std dev of its 3D position across
//! frames) and the distances between markers. `camparam.yaml` is the device's
//! `/data/camparam/camparam.yaml` (`zx_cmd.cgi?download=/data/camparam/camparam.yaml`).

use glam::Vec3;
use scancraft_core::Transform;
use scancraft_driver::ObjectType;
use scancraft_driver_revopoint::media::{FrameDecoder, FrameLayout, PacketParser};
use scancraft_driver_revopoint::streams::{FullField, MarkerStream, ModeStream};
use scancraft_driver_revopoint::{DeviceCalibration, StereoCalibration};
use scancraft_markers::{MarkerTracker, MarkerTrackerConfig};
use std::sync::Arc;

fn main() {
    env_logger::init();
    let args: Vec<String> = std::env::args().collect();
    let rec = args.get(1).expect("usage: marker_replay <recording.bin> <camparam.yaml> [display_type] [map_out.json]");
    let yaml = std::fs::read_to_string(args.get(2).expect("camparam.yaml")).unwrap();
    let display_type: u32 = args.get(3).and_then(|s| s.parse().ok()).unwrap_or(6);
    let calibration = Arc::new(DeviceCalibration::new(StereoCalibration::parse(&yaml).expect("camparam.yaml")));
    let (layout, mut stream): (FrameLayout, Box<dyn ModeStream>) = match display_type {
        4 => (FrameLayout::DepthIrPair, Box::new(FullField::new(ObjectType::General, calibration.clone()).unwrap())),
        _ => (FrameLayout::DepthIrPairOriIr, Box::new(MarkerStream::new(ObjectType::General, calibration.clone()).unwrap())),
    };

    let mut parser = PacketParser::default();
    parser.push(&std::fs::read(rec).unwrap());
    let mut decoder = FrameDecoder::new(layout, 640, 512);
    let mut tracker = MarkerTracker::new(MarkerTrackerConfig { record_observations: true, ..Default::default() });
    let mut pose = Transform::default();
    let mut per_frame: Vec<Vec<Vec3>> = Vec::new();
    let (mut packets, mut frames) = (0, 0u64);
    while let Some(packet) = parser.next_packet() {
        packets += 1;
        let Ok(decoded) = decoder.decode(&packet) else { continue };
        let Some(frame) = stream.to_sensor_frame(decoded, frames + 1) else { continue };
        frames += 1;
        let t = std::time::Instant::now();
        let hint = tracker.estimate(&frame, &pose);
        let ms = t.elapsed().as_secs_f32() * 1000.0;
        let r = tracker.report().clone();
        println!(
            "frame {frames:3}: dots L {:3} R {:3} | 3D {:3} | inliers {:3} rms {:.3} mm | hint {:?} | map {:3} | {ms:.1} ms",
            r.detected_left, r.detected_right, r.triangulated, r.inliers, r.rms_mm, r.hint, r.map_size
        );
        if let Some(h) = hint {
            pose = h.pose;
        }
        per_frame.push(tracker.last_markers().iter().map(|m| m.position).collect());
    }
    println!("{packets} packets, {frames} unique frames, right cx offset {:?}", tracker.right_cx_offset());
    {
        let map = tracker.map();
        let map = map.lock().unwrap();
        let solid = map.markers.iter().filter(|m| m.observations >= 3).count();
        println!("map: {} markers, {solid} seen in >= 3 frames", map.len());
        let mut refined = map.clone();
        let mut obs = tracker.observations().lock().unwrap().clone();
        let r = scancraft_markers::refine_map(&mut refined, &mut obs, 3, 50);
        println!("global refinement over {} frames: rms {:.3} -> {:.3} mm, {} markers kept", obs.len(), r.rms_before_mm, r.rms_after_mm, refined.len());
    }
    let angle = pose.rotation.normalize().to_axis_angle().1.to_degrees();
    println!(
        "final pose vs first frame (loop closure if the scan ends at the start): translation {:.2} mm {:?}, rotation {angle:.2} deg",
        pose.translation.length(),
        pose.translation
    );

    // Static-scene repeatability: cluster each frame's markers by proximity to the first frame
    // in which they appear.
    let mut tracks: Vec<Vec<Vec3>> = Vec::new();
    for markers in &per_frame {
        for p in markers {
            match tracks.iter_mut().find(|t| t[0].distance(*p) < 3.0) {
                Some(t) => t.push(*p),
                None => tracks.push(vec![*p]),
            }
        }
    }
    tracks.retain(|t| t.len() >= 3);
    println!("\n{} markers seen in >= 3 frames (of {}):", tracks.len(), per_frame.len());
    let mut means = Vec::new();
    for (i, t) in tracks.iter().enumerate() {
        let mean = t.iter().copied().sum::<Vec3>() / t.len() as f32;
        let var = t.iter().map(|p| (*p - mean) * (*p - mean)).sum::<Vec3>() / t.len() as f32;
        let sd = Vec3::new(var.x.sqrt(), var.y.sqrt(), var.z.sqrt());
        println!(
            "  m{i:02}: seen {:3}x at ({:7.2}, {:7.2}, {:7.2}) mm, sd x {:.3} y(depth) {:.3} z {:.3} | 3D {:.3} mm",
            t.len(),
            mean.x,
            mean.y,
            mean.z,
            sd.x,
            sd.y,
            sd.z,
            sd.length()
        );
        means.push(mean);
    }
    if means.len() >= 2 {
        println!("\ndistances between markers (mm):");
        for i in 0..means.len() {
            let row: Vec<String> = (0..means.len()).map(|j| format!("{:6.2}", means[i].distance(means[j]))).collect();
            println!("  m{i:02}: {}", row.join(" "));
        }
    }
    if let Some(out) = args.get(4) {
        tracker.map().lock().unwrap().save_json(std::path::Path::new(out)).unwrap();
        println!("map saved to {out}");
    }
}
