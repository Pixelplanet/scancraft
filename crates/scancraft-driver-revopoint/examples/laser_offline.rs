//! Runs the laser-line depth source over a recorded laser stream (`camera_id=33&type_id=20`,
//! raw wire framing as written by `SCANCRAFT_RECORD_DIR` or a curl dump).
//!
//! `cargo run --release -p scancraft-driver-revopoint --example laser_offline -- <rec.bin> <calib_dir> <out_dir> [ff.bin|-] [planes.json]`
//!
//! - `calib_dir`: blue set as downloaded from the scanner (`camparam.yaml`, `mapparamL.bin`,
//!   `mapparamR.bin`, `metroExtra.bin`) plus the red `camparam.yaml` as `red_camparam.yaml`
//!   (only needed with `ff.bin`).
//! - `ff.bin`: optional Full-field HD recording (`camera_id=21`, fmt 3 / type 4) of the same static
//!   scene. Used as ground truth: the laser planes are calibrated only from pairings that agree
//!   with it, and every output frame is compared with it (mean |dZ|, fraction within 2 mm).
//! - `planes.json`: use these planes instead of calibrating.
//!
//! Prints per-frame statistics and timing, writes `planes.json`, one PLY per family and a
//! merged PLY, and the plane-fit RMS of the largest flat region.

use glam::Vec3;
use scancraft_core::{DepthMap, IrImages, PointCloud, SensorFrame};
use scancraft_driver_revopoint::calibration::{BlueCalibration, StereoCalibration};
use scancraft_driver_revopoint::media::{FrameDecoder, FrameLayout, PacketParser};
use scancraft_laser::planes::{CalibrationParams, FrameCandidates, LaserPlaneSet};
use scancraft_laser::{LaserLineDepth, Pattern};
use std::path::PathBuf;
use std::sync::Arc;

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let rec = std::fs::read(&args[1]).expect("recording");
    let calib = PathBuf::from(&args[2]);
    let out = PathBuf::from(&args[3]);
    std::fs::create_dir_all(&out).unwrap();
    let read = |n: &str| std::fs::read(calib.join(n)).unwrap();
    let blue = BlueCalibration::parse(
        &String::from_utf8(read("camparam.yaml")).unwrap(),
        &read("mapparamL.bin"),
        &read("mapparamR.bin"),
        read("metroExtra.bin"),
    )
    .unwrap();
    let rig = Arc::new(blue.rig());

    let truth = args.get(4).filter(|a| a.as_str() != "-").map(|p| {
        let red = StereoCalibration::parse(&String::from_utf8(read("red_camparam.yaml")).unwrap()).unwrap();
        load_ff_depth(&std::fs::read(p).unwrap(), &red)
    });
    if let Some(t) = &truth {
        println!("ground truth: Full-field depth with {} valid pixels", t.valid_pixel_count());
    }

    let frames = decode_all(&rec, &rig);
    println!("{} frames decoded", frames.len());
    let pattern = if std::env::var("LASER_PATTERN").as_deref() == Ok("single") { Pattern::Single } else { Pattern::Cross };
    let mut depth = LaserLineDepth::new(pattern);

    // Planes: given, calibrated against ground truth, or calibrated online (first frames).
    let planes = match args.get(5).and_then(|p| std::fs::read_to_string(p).ok()) {
        Some(json) => Some(serde_json::from_str::<LaserPlaneSet>(&json).unwrap()),
        None => truth.as_ref().map(|t| {
            let mut fc = Vec::new();
            for f in frames.iter().filter(|f| has_laser(f)).take(depth.params.calibration_frames) {
                let Some(scancraft_laser::FrameLines { left: lc, candidates: cands, .. }) = depth.candidates(f) else { continue };
                let keep: Vec<_> = cands.into_iter().filter(|c| median_abs_dz(&c.points, t) < 2.0).collect();
                let families = keep.iter().map(|c| if lc[c.left].slope > 0.0 { 1 } else { -1 }).collect();
                fc.push(FrameCandidates { candidates: keep, families });
            }
            let params = CalibrationParams { min_spread_mm: 0.0, ..Default::default() };
            LaserPlaneSet::calibrate(&fc, &params)
        }),
    };
    if let Some(p) = planes {
        depth = LaserLineDepth::with_planes(pattern, p);
    }

    let (mut t_depth, mut n_frames, mut n_points) = (0f64, 0usize, 0usize);
    let (mut dz_sum, mut dz_n, mut dz_ok) = (0f64, 0usize, 0usize);
    let mut written = [false, false];
    let mut merged = PointCloud::new();
    let mut best_cloud: Option<PointCloud> = None;
    for f in &frames {
        let had_planes = depth.planes().is_some();
        let t0 = std::time::Instant::now();
        let Some(result) = depth.process(f) else { continue };
        let dt = t0.elapsed().as_secs_f64();
        if !had_planes {
            if depth.planes().is_some() {
                println!("planes calibrated online at frame {}", f.frame_index);
            }
            continue;
        }
        if !has_laser(f) {
            continue;
        }
        let s = &result.stats;
        t_depth += dt;
        n_frames += 1;
        n_points += s.points;
        if let Some(t) = &truth {
            for p in &result.cloud.points {
                // Scanner frame -> optical frame.
                let q = Vec3::new(p.position[0], -p.position[2], p.position[1]);
                if let Some(z) = truth_z(t, q) {
                    let dz = (q.z - z).abs();
                    dz_sum += dz as f64;
                    dz_n += 1;
                    dz_ok += (dz < 2.0) as usize;
                }
            }
        }
        if f.frame_index % 20 == 0 {
            println!(
                "frame {}: samples L {} R {}, chains L {} R {}, candidates {}, matched {} ambiguous {}, points {}",
                f.frame_index, s.left_samples, s.right_samples, s.left_chains, s.right_chains, s.candidates, s.matched_chains, s.ambiguous_chains, s.points
            );
        }
        let family = (f.frame_index % 2) as usize;
        if !written[family] && s.points > 0 {
            written[family] = true;
            scancraft_io::ply::save_ply_binary(&result.cloud, out.join(format!("laser_frame{}.ply", f.frame_index))).unwrap();
            merged.points.extend_from_slice(&result.cloud.points);
        }
        if best_cloud.as_ref().is_none_or(|c| c.len() < result.cloud.len()) {
            best_cloud = Some(result.cloud.clone());
        }
    }
    if let Some(p) = depth.planes() {
        println!("{} planes", p.planes.len());
        std::fs::write(out.join("planes.json"), serde_json::to_string_pretty(p).unwrap()).unwrap();
    }
    merged.recompute_bbox();
    if !merged.is_empty() {
        scancraft_io::ply::save_ply_binary(&merged, out.join("laser_both_families.ply")).unwrap();
    }
    if n_frames > 0 {
        let span = frames.last().unwrap().timestamp_ms.saturating_sub(frames[0].timestamp_ms) as f64 / 1e3;
        println!(
            "{n_frames} laser frames: mean {:.0} points/frame, depth source {:.1} ms/frame; stream {} frames in {:.1} s = {:.1} fps",
            n_points as f64 / n_frames as f64,
            1e3 * t_depth / n_frames as f64,
            frames.len(),
            span,
            frames.len() as f64 / span.max(1e-3)
        );
    }
    if dz_n > 0 {
        println!(
            "vs Full-field HD: {dz_n} points compared, mean |dZ| {:.3} mm, {:.1}% within 2 mm",
            dz_sum / dz_n as f64,
            100.0 * dz_ok as f64 / dz_n as f64
        );
    }
    if let Some(cloud) = best_cloud {
        report_flat_region(&cloud, truth.as_ref());
    }
}

fn has_laser(f: &SensorFrame) -> bool {
    f.ir.as_ref().is_some_and(|ir| ir.left.data.iter().step_by(13).filter(|&&p| p > 100).count() > 150)
}

fn decode_all(rec: &[u8], rig: &Arc<scancraft_core::StereoRig>) -> Vec<SensorFrame> {
    let mut parser = PacketParser::default();
    parser.push(rec);
    let mut decoder = FrameDecoder::new(FrameLayout::JpegIrPair, 1280, 1024);
    let mut frames = Vec::new();
    let mut t_decode = 0f64;
    while let Some(packet) = parser.next_packet() {
        let t0 = std::time::Instant::now();
        let Ok(decoded) = decoder.decode(&packet) else { continue };
        t_decode += t0.elapsed().as_secs_f64();
        let mut frame = SensorFrame::new(frames.len() as u64 + 1, decoded.device_counter as u64);
        frame.ir = Some(IrImages { left: decoded.ir_left, right: Some(decoded.ir_right), rectified: false, rig: Some(rig.clone()) });
        frames.push(frame);
    }
    if !frames.is_empty() {
        println!("JPEG pair decode {:.1} ms/frame", 1e3 * t_decode / frames.len() as f64);
    }
    frames
}

/// Average of the last 10 non-empty Full-field depth frames (static scene), with the red
/// rectified intrinsics at 640x512. The red and blue rectified frames differ by 0.009°.
fn load_ff_depth(rec: &[u8], red: &StereoCalibration) -> DepthMap {
    let mut parser = PacketParser::default();
    parser.push(rec);
    let mut decoder = FrameDecoder::new(FrameLayout::DepthIrPair, 640, 512);
    let mut maps = Vec::new();
    let (mut packets, mut decoded, mut best_valid) = (0, 0, 0);
    while let Some(packet) = parser.next_packet() {
        packets += 1;
        match decoder.decode(&packet) {
            Ok(f) => {
                decoded += 1;
                let valid = f.depth.as_ref().map(|d| d.valid_pixel_count()).unwrap_or(0);
                best_valid = best_valid.max(valid);
                if let Some(d) = f.depth.filter(|_| valid > 10_000) {
                    maps.push(d);
                }
            }
            Err(e) if decoded == 0 => println!("FF decode: {e}"),
            Err(_) => {}
        }
    }
    println!("FF recording: {packets} packets, {decoded} decoded, {} with depth (max {best_valid} valid px)", maps.len());
    assert!(!maps.is_empty(), "no Full-field depth frames in the recording");
    let last: Vec<&DepthMap> = maps.iter().rev().take(10).collect();
    let n = 640 * 512;
    let data = (0..n)
        .map(|i| {
            let vals: Vec<u32> = last.iter().map(|m| m.data[i] as u32).filter(|&v| v > 0).collect();
            if vals.len() * 2 < last.len() {
                0
            } else {
                (vals.iter().sum::<u32>() / vals.len() as u32) as u16
            }
        })
        .collect();
    DepthMap::new(640, 512, 0.01, data).with_intrinsics(red.depth_intrinsics(640, 512))
}

/// Full-field depth (mm) along the ray of an optical-frame point.
fn truth_z(t: &DepthMap, p: Vec3) -> Option<f32> {
    let k = t.intrinsics?;
    let u = (k.fx * p.x / p.z + k.cx).round();
    let v = (k.fy * p.y / p.z + k.cy).round();
    if u < 0.0 || v < 0.0 {
        return None;
    }
    t.get_depth_mm(u as u32, v as u32)
}

fn median_abs_dz(points: &[Vec3], t: &DepthMap) -> f32 {
    let mut d: Vec<f32> = points.iter().filter_map(|p| truth_z(t, *p).map(|z| (p.z - z).abs())).collect();
    if d.len() * 2 < points.len() {
        return f32::INFINITY;
    }
    let m = d.len() / 2;
    *d.select_nth_unstable_by(m, f32::total_cmp).1
}

/// RANSAC plane over the cloud (1 mm band); RMS of the inliers, and the same for the
/// Full-field points of that region.
fn report_flat_region(cloud: &PointCloud, truth: Option<&DepthMap>) {
    let pts: Vec<Vec3> = cloud.points.iter().map(|p| Vec3::from(p.position)).collect();
    if pts.len() < 100 {
        return;
    }
    let mut best = (0usize, Vec3::Z, 0f32);
    let mut seed = 12345u64;
    let mut rnd = |n: usize| {
        seed ^= seed << 13;
        seed ^= seed >> 7;
        seed ^= seed << 17;
        (seed % n as u64) as usize
    };
    for _ in 0..3000 {
        let (a, b, c) = (pts[rnd(pts.len())], pts[rnd(pts.len())], pts[rnd(pts.len())]);
        let n = (b - a).cross(c - a).normalize_or_zero();
        if n == Vec3::ZERO {
            continue;
        }
        let d = n.dot(a);
        let count = pts.iter().filter(|p| (n.dot(**p) - d).abs() < 1.0).count();
        if count > best.0 {
            best = (count, n, d);
        }
    }
    let inliers: Vec<Vec3> = pts.iter().copied().filter(|p| (best.1.dot(*p) - best.2).abs() < 1.0).collect();
    let Some((n, d, rms, _)) = scancraft_laser::planes::fit_plane(&inliers) else { return };
    println!("laser: largest flat region {} of {} points, plane-fit RMS {:.3} mm", inliers.len(), pts.len(), rms);
    let Some(t) = truth else { return };
    // Full-field points of the same plane region: back-project all FF pixels within 1 mm of the
    // laser plane and inside the laser inliers' bounding box.
    let (lo, hi) = inliers.iter().fold((Vec3::splat(f32::MAX), Vec3::splat(f32::MIN)), |(lo, hi), p| (lo.min(*p), hi.max(*p)));
    let ff = t.to_point_cloud(100.0, 1000.0);
    let region: Vec<Vec3> = ff
        .points
        .iter()
        .map(|p| Vec3::from(p.position))
        .filter(|p| p.cmpge(lo).all() && p.cmple(hi).all() && (n.dot(*p) - d).abs() < 2.0)
        .collect();
    if let Some((_, _, ff_rms, _)) = scancraft_laser::planes::fit_plane(&region) {
        println!("full-field HD: same region {} points, plane-fit RMS {:.3} mm", region.len(), ff_rms);
    }
}
