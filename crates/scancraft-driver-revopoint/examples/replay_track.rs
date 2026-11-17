//! Replays a captured fmt 3 / type 4 `zx_media.cgi` body through decode, back-projection and the
//! tracker, reporting per-frame pose, model growth and timing:
//! `cargo run --release -p scancraft-driver-revopoint --example replay_track -- <capture.bin> <camparam.yaml>`
//!
//! Environment: `FRAMES=n`, `TOP_VIEW=out.png`, `FUSED_PLY=out.ply`, `TURNTABLE=1` (turntable
//! pipeline), `NOISE_OFF=1` / `EDGE_OFF` / `FILTER_OFF` / `CARVE_OFF` (noise handling ablations),
//! `NO_IMU`, `NO_VISIBILITY`, `MIN_INLIER`. Prints noise figures of the fused cloud at the end.

use scancraft_driver_revopoint::{FrameDecoder, FrameLayout, PacketParser, StereoCalibration};
use scancraft_slam::Tracker;
use std::time::Instant;

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let capture = std::fs::read(&args[1]).expect("capture");
    let yaml = std::fs::read_to_string(&args[2]).expect("camparam.yaml");
    let intrinsics = StereoCalibration::parse(&yaml).expect("calibration").depth_intrinsics(640, 512);

    let mut parser = PacketParser::default();
    parser.push(&capture);
    let mut decoder = FrameDecoder::new(FrameLayout::DepthIrPair, 640, 512);
    // POP 4 IMU mounting found by `imu_extrinsic_search` (IMU x,y,z -> sensor +x,+z,-y).
    let mount = glam::Quat::from_mat3(&glam::Mat3::from_cols(glam::Vec3::X, glam::Vec3::Z, -glam::Vec3::Y));
    let config = scancraft_slam::TrackerConfig {
        imu_to_sensor: (std::env::var("NO_IMU").is_err()).then_some(mount),
        imu_body_to_world: std::env::var("IMU_WORLD_TO_BODY").is_err(),
        min_inlier_ratio: std::env::var("MIN_INLIER").ok().and_then(|v| v.parse().ok()).unwrap_or(0.4),
        // NOISE_OFF=1: no flying-pixel filter, confidence filter or carving (the old behaviour).
        noise: if std::env::var("NOISE_OFF").is_ok() { scancraft_slam::NoiseConfig::off() } else { Default::default() },
        ..Default::default()
    };
    let mut config = config;
    // Ablations: EDGE_OFF, FILTER_OFF (confidence / support), CARVE_OFF.
    if std::env::var("EDGE_OFF").is_ok() {
        config.noise.edge_filter = None;
    }
    if std::env::var("FILTER_OFF").is_ok() {
        config.noise.surface.enabled = false;
    }
    if std::env::var("CARVE_OFF").is_ok() {
        config.noise.carve = false;
    }
    // TURNTABLE=1: process through the turntable pipeline (plane + cylinder filter, no IMU prior).
    let mut turntable = std::env::var("TURNTABLE").is_ok().then(|| {
        let config = scancraft_driver::ScanConfig { turntable_mode: scancraft_driver::TurntableMode::TurntableContinuous, ..Default::default() };
        scancraft_pipeline::pipeline_for(&config, 150.0, 400.0).expect("turntable pipeline")
    });
    let mut region_pool: Vec<glam::Vec3> = Vec::new();
    let mut tracker = Tracker::with_config(config);
    let max_frames: usize = std::env::var("FRAMES").ok().and_then(|v| v.parse().ok()).unwrap_or(usize::MAX);
    let mut k = 0;
    let mut last_q: Option<glam::Quat> = None;
    let mut tracker_imu_prev: Option<glam::Quat> = None;
    while let Some(packet) = parser.next_packet() {
        if k >= max_frames {
            break;
        }
        let t0 = Instant::now();
        let Ok(frame) = decoder.decode(&packet) else { continue };
        let Some(depth) = frame.depth.filter(|d| d.valid_pixel_count() > 0) else { continue };
        let imu_step = frame.imu.map(|s| {
            let step = last_q.map(|q| q.angle_between(s.orientation).to_degrees()).unwrap_or(0.0);
            last_q = Some(s.orientation);
            step
        });
        let depth = depth.with_intrinsics(intrinsics);
        let t1 = Instant::now();
        let prev_pose = tracker.current_pose;
        let prev_imu = tracker_imu_prev;
        if turntable.is_some() && k < 3 {
            // Same estimate the pipeline makes from its first three frames, for the log.
            let cloud = depth.to_point_cloud_with(&scancraft_core::BackProjection::filtered(150.0, 400.0));
            let step = (cloud.len() / 6000).max(1);
            region_pool.extend(cloud.points.iter().step_by(step).map(|p| glam::Vec3::from(p.position)));
            if k == 2 {
                let region = scancraft_pipeline::turntable::estimate_region(&region_pool, &Default::default());
                println!("turntable region: {region:?}");
            }
        }
        let status = if let Some(pipeline) = turntable.as_mut() {
            // The pipeline path the app uses (no IMU prior in turntable mode).
            let mut sensor_frame = scancraft_core::SensorFrame::new(k as u64, 0);
            sensor_frame.depth = Some(depth.clone());
            sensor_frame.imu = frame.imu;
            match pipeline.process(&sensor_frame, &mut tracker) {
                Some(status) => status,
                None => {
                    println!("frame {k:3}: no geometry (turntable region not found yet)");
                    k += 1;
                    continue;
                }
            }
        } else if std::env::var("NO_VISIBILITY").is_ok() {
            if let Some(imu) = frame.imu {
                tracker.update_imu(imu);
            }
            tracker.register_frame(&depth.to_point_cloud(150.0, 400.0))
        } else {
            if let Some(imu) = frame.imu {
                tracker.update_imu(imu);
            }
            tracker.register_depth(&depth, 150.0, 400.0)
        };
        // Disagreement between the rotation ICP found and the IMU's rotation since the last accepted frame.
        let imu_disagree = match (prev_imu, frame.imu) {
            (Some(q0), Some(s)) if status.state != scancraft_slam::TrackingState::Lost => {
                let body = q0.inverse() * s.orientation.normalize();
                let predicted = mount * body * mount.inverse();
                let icp_step = prev_pose.rotation.inverse() * status.current_pose.rotation;
                icp_step.angle_between(predicted).to_degrees()
            }
            _ => f32::NAN,
        };
        if status.state != scancraft_slam::TrackingState::Lost {
            tracker_imu_prev = frame.imu.map(|s| s.orientation.normalize());
        }
        let new = tracker.take_new_points().len();
        let t2 = Instant::now();
        println!(
            "frame {k:3}: imu step {:5.2}° disagree {:5.1}° conflict {:4.2} {}{:?} t=[{:.2} {:.2} {:.2}] rot {:.3}° inliers {} rmse {:.3} mm | +{new} new, model {} | decode {:.0} ms, track {:.0} ms",
            imu_step.unwrap_or(f32::NAN),
            imu_disagree,
            status.visibility_conflict,
            if status.relocalized { "RELOC " } else { "" },
            status.state,
            status.current_pose.translation.x,
            status.current_pose.translation.y,
            status.current_pose.translation.z,
            status.current_pose.rotation.angle_between(glam::Quat::IDENTITY).to_degrees(),
            status.inliers,
            status.rmse_mm,
            status.total_points_fused,
            (t1 - t0).as_secs_f32() * 1000.0,
            (t2 - t1).as_secs_f32() * 1000.0,
        );
        k += 1;
    }
    if let Ok(path) = std::env::var("FUSED_PLY") {
        use std::io::Write;
        let pts = tracker.fused_points();
        let mut f = std::io::BufWriter::new(std::fs::File::create(&path).unwrap());
        writeln!(f, "ply
format ascii 1.0
element vertex {}
property float x
property float y
property float z
property float nx
property float ny
property float nz
end_header", pts.len()).unwrap();
        for p in &pts {
            let (q, n) = (p.position, p.normal);
            writeln!(f, "{} {} {} {} {} {}", q[0], q[1], q[2], n[0], n[1], n[2]).unwrap();
        }
        println!("fused cloud: {} points -> {path}", pts.len());
    }
    if let Ok(path) = std::env::var("TOP_VIEW") {
        top_view(&tracker.fused_points(), &path);
    }
    // `surface()` holds the fusion lock: read it once, then release it before other calls.
    let (len, stored, withdrawn) = {
        let surface = tracker.surface();
        (surface.len(), surface.stored(), surface.withdrawn_pending())
    };
    let revision = tracker.model_revision();
    println!(
        "surface: {len} output of {stored} stored surfels, {} keyframes, model revision {revision}, {withdrawn} handed-out points withdrawn",
        tracker.keyframes().len(),
    );
    noise_metrics(&tracker.fused_points());
}

/// Noise figures of a fused cloud: isolated points (fewer than `k` neighbours within `r`) and the
/// thickness (RMS) of the dominant plane (the table), refit by least squares on its inliers.
fn noise_metrics(points: &[scancraft_core::Point3D]) {
    use glam::Vec3;
    use std::collections::HashMap;
    let pts: Vec<Vec3> = points.iter().map(|p| Vec3::from(p.position)).collect();
    if pts.len() < 100 {
        return;
    }
    for (r, k) in [(2.0f32, 5usize), (5.0, 10)] {
        let key = |p: Vec3| [(p.x / r).floor() as i32, (p.y / r).floor() as i32, (p.z / r).floor() as i32];
        let mut grid: HashMap<[i32; 3], Vec<usize>> = HashMap::new();
        for (i, p) in pts.iter().enumerate() {
            grid.entry(key(*p)).or_default().push(i);
        }
        let isolated = pts
            .iter()
            .filter(|p| {
                let [x, y, z] = key(**p);
                let mut n = 0;
                for dx in -1..=1 {
                    for dy in -1..=1 {
                        for dz in -1..=1 {
                            if let Some(c) = grid.get(&[x + dx, y + dy, z + dz]) {
                                n += c.iter().filter(|&&j| pts[j].distance_squared(**p) <= r * r).count();
                            }
                        }
                    }
                }
                n - 1 < k
            })
            .count();
        println!("isolated (<{k} neighbours within {r} mm): {isolated} of {} ({:.2}%)", pts.len(), 100.0 * isolated as f32 / pts.len() as f32);
    }
    // Dominant plane by RANSAC, then least-squares refit on points within 2 mm.
    let sample: Vec<Vec3> = pts.iter().step_by((pts.len() / 20_000).max(1)).copied().collect();
    let mut seed = 777u32;
    let mut rand = |n: usize| {
        seed ^= seed << 13;
        seed ^= seed >> 17;
        seed ^= seed << 5;
        seed as usize % n
    };
    let (mut best, mut normal, mut offset) = (0, Vec3::Z, 0.0);
    for _ in 0..500 {
        let (a, b, c) = (sample[rand(sample.len())], sample[rand(sample.len())], sample[rand(sample.len())]);
        let n = (b - a).cross(c - a).normalize_or_zero();
        if n == Vec3::ZERO {
            continue;
        }
        let d = n.dot(a);
        let count = sample.iter().filter(|p| (n.dot(**p) - d).abs() < 1.0).count();
        if count > best {
            (best, normal, offset) = (count, n, d);
        }
    }
    for _ in 0..3 {
        let inl: Vec<Vec3> = pts.iter().filter(|p| (normal.dot(**p) - offset).abs() < 2.0).copied().collect();
        let c = inl.iter().sum::<Vec3>() / inl.len() as f32;
        let mut cov = glam::Mat3::ZERO;
        for p in &inl {
            let d = *p - c;
            cov += glam::Mat3::from_cols(d * d.x, d * d.y, d * d.z);
        }
        // Smallest eigenvector by inverse power iteration on the (regularised) covariance.
        let inv = (cov + glam::Mat3::IDENTITY * 1e-3).inverse();
        let mut n = normal;
        for _ in 0..30 {
            n = (inv * n).normalize();
        }
        normal = n;
        offset = n.dot(c);
    }
    let d: Vec<f32> = pts.iter().map(|p| normal.dot(*p) - offset).filter(|d| d.abs() < 2.0).collect();
    let rms = (d.iter().map(|x| x * x).sum::<f32>() / d.len() as f32).sqrt();
    // Points hovering just off the table (between 2 and 6 mm on either side) are mostly noise
    // (the side below the table is never real surface).
    let below = pts.iter().filter(|p| (normal.dot(**p) - offset) < -2.0).count();
    let above = pts.iter().filter(|p| (normal.dot(**p) - offset) > 2.0).count();
    let (near_lo, near_hi) = if below > above { (2.0, 6.0) } else { (-6.0, -2.0) };
    let hover = pts.iter().filter(|p| (near_lo..near_hi).contains(&(normal.dot(**p) - offset))).count();
    println!("table plane: {} points within 2 mm, RMS {rms:.3} mm; {hover} points 2-6 mm under the table", d.len());
}

/// Writes a top-down picture of the fused cloud: projected onto the dominant plane (the table),
/// 0.5 mm per pixel, brightness = height above the table. Misregistration shows as doubled edges.
fn top_view(points: &[scancraft_core::Point3D], path: &str) {
    use glam::Vec3;
    let pts: Vec<Vec3> = points.iter().map(|p| Vec3::from(p.position)).collect();
    // Dominant plane by RANSAC on a subsample.
    let sample: Vec<Vec3> = pts.iter().step_by((pts.len() / 20_000).max(1)).copied().collect();
    let mut seed = 12345u32;
    let mut rand = |n: usize| {
        seed ^= seed << 13;
        seed ^= seed >> 17;
        seed ^= seed << 5;
        seed as usize % n
    };
    let (mut best, mut normal, mut offset) = (0, Vec3::Z, 0.0);
    for _ in 0..500 {
        let (a, b, c) = (sample[rand(sample.len())], sample[rand(sample.len())], sample[rand(sample.len())]);
        let n = (b - a).cross(c - a).normalize_or_zero();
        if n == Vec3::ZERO {
            continue;
        }
        let d = n.dot(a);
        let count = sample.iter().filter(|p| (n.dot(**p) - d).abs() < 1.5).count();
        if count > best {
            (best, normal, offset) = (count, n, d);
        }
    }
    // Height positive on the side where most off-plane points are (the objects).
    let above = sample.iter().filter(|p| normal.dot(**p) - offset > 3.0).count();
    let below = sample.iter().filter(|p| normal.dot(**p) - offset < -3.0).count();
    if below > above {
        normal = -normal;
        offset = -offset;
    }
    let u = normal.any_orthonormal_vector();
    let v = normal.cross(u);
    let (mut min, mut max) = (glam::Vec2::splat(f32::MAX), glam::Vec2::splat(f32::MIN));
    for p in &pts {
        let q = glam::Vec2::new(u.dot(*p), v.dot(*p));
        min = min.min(q);
        max = max.max(q);
    }
    let scale = 2.0; // px per mm
    let (w, h) = (((max.x - min.x) * scale) as u32 + 1, ((max.y - min.y) * scale) as u32 + 1);
    let (w, h) = (w.min(4000), h.min(4000));
    let mut img = image::GrayImage::new(w, h);
    for p in &pts {
        let height = normal.dot(*p) - offset;
        let (x, y) = (((u.dot(*p) - min.x) * scale) as u32, ((v.dot(*p) - min.y) * scale) as u32);
        if x < w && y < h {
            let val = (40.0 + height.clamp(-5.0, 120.0) * 1.8).clamp(0.0, 255.0) as u8;
            let px = img.get_pixel_mut(x, y);
            px.0[0] = px.0[0].max(val);
        }
    }
    img.save(path).unwrap();
    println!("top view {w}x{h} px -> {path} (plane inliers {best}/{})", sample.len());
}
