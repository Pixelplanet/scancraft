//! Finds how the POP 4's IMU is mounted relative to the depth camera by brute force: replays a
//! recording of a moving scanner through the tracker once per axis-aligned mounting (24 proper
//! rotations x 2 quaternion conventions) and ranks them by how many frames stay tracked.
//! `cargo run --release -p scancraft-driver-revopoint --example imu_extrinsic_search -- <capture.bin> <camparam.yaml> [frames]`

use glam::{Mat3, Quat, Vec3};
use scancraft_core::{ImuSample, PointCloud};
use scancraft_driver_revopoint::{FrameDecoder, FrameLayout, PacketParser, StereoCalibration};
use scancraft_slam::{Tracker, TrackerConfig, TrackingState};

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let capture = std::fs::read(&args[1]).expect("capture");
    let yaml = std::fs::read_to_string(&args[2]).expect("camparam.yaml");
    let max_frames: usize = args.get(3).and_then(|s| s.parse().ok()).unwrap_or(60);
    let intrinsics = StereoCalibration::parse(&yaml).expect("calibration").depth_intrinsics(640, 512);

    let mut parser = PacketParser::default();
    parser.push(&capture);
    let mut decoder = FrameDecoder::new(FrameLayout::DepthIrPair, 640, 512);
    let mut frames: Vec<(PointCloud, Option<ImuSample>)> = Vec::new();
    while let Some(packet) = parser.next_packet() {
        if frames.len() >= max_frames {
            break;
        }
        let Ok(frame) = decoder.decode(&packet) else { continue };
        let Some(depth) = frame.depth.filter(|d| d.valid_pixel_count() > 0) else { continue };
        frames.push((depth.with_intrinsics(intrinsics).to_point_cloud(150.0, 400.0), frame.imu));
    }
    println!("{} frames", frames.len());

    // Gravity check: the accelerometer measures "up" in IMU axes (+1 g at rest); the table is the
    // largest plane in view and its normal (towards the scanner side) is "up" in sensor axes.
    let mut pairs = Vec::new();
    for (cloud, imu) in &frames {
        let Some(imu) = imu else { continue };
        let a = imu.linear_acceleration;
        if (a.length() - 1.0).abs() > 0.1 {
            continue; // hand acceleration would bias the direction
        }
        let pts: Vec<Vec3> = cloud.points.iter().step_by(23).map(|p| Vec3::from(p.position)).collect();
        if let Some(n) = largest_plane_normal(&pts) {
            pairs.push((a.normalize(), n));
        }
    }
    let mut g_scores: Vec<(f32, String)> = axis_rotations()
        .into_iter()
        .map(|(name, m)| {
            let mut errs: Vec<f32> = pairs.iter().map(|(a, n)| (m * *a).angle_between(*n).to_degrees()).collect();
            errs.sort_by(f32::total_cmp);
            (errs.get(errs.len() / 2).copied().unwrap_or(f32::NAN), name)
        })
        .collect();
    g_scores.sort_by(|a, b| a.0.total_cmp(&b.0));
    println!("gravity check on {} frames; median angle between mapped accel and table normal:", pairs.len());
    for (e, name) in g_scores.iter().take(5) {
        println!("  {name:12}: {e:.2}°");
    }

    if std::env::var_os("GRAVITY_ONLY").is_some() {
        return;
    }

    // Hand-eye consistency: per accepted step, the rotation ICP measured vs. the IMU rotation
    // mapped through each candidate mounting. Uses the tracker's own poses for `seed`.
    let consistency = |seed: Option<(Quat, bool)>| {
        let config = TrackerConfig {
            imu_to_sensor: seed.map(|s| s.0),
            imu_body_to_world: seed.is_none_or(|s| s.1),
            fusion_voxel_mm: 1.0,
            ..Default::default()
        };
        let mut tracker = Tracker::with_config(config);
        let mut steps = Vec::new();
        let mut prev: Option<(Quat, Quat)> = None;
        for (cloud, imu) in &frames {
            let Some(imu) = imu else { continue };
            tracker.update_imu(*imu);
            let s = tracker.register_frame(cloud);
            if s.state == TrackingState::Lost {
                prev = None;
                continue;
            }
            let (pose, q) = (s.current_pose.rotation, imu.orientation.normalize());
            if let Some((p_pose, p_q)) = prev {
                let icp_step = p_pose.inverse() * pose;
                if icp_step.angle_between(Quat::IDENTITY) > 2f32.to_radians() {
                    steps.push((icp_step, p_q, q));
                }
            }
            prev = Some((pose, q));
        }
        let mut scores: Vec<(f32, String, bool)> = Vec::new();
        for (name, m) in axis_rotations() {
            let x = Quat::from_mat3(&m);
            for btw in [true, false] {
                let mut errs: Vec<f32> = steps
                    .iter()
                    .map(|(icp, q0, q1)| {
                        let body = if btw { q0.inverse() * *q1 } else { *q0 * q1.inverse() };
                        icp.angle_between(x * body * x.inverse()).to_degrees()
                    })
                    .collect();
                errs.sort_by(f32::total_cmp);
                scores.push((errs.get(errs.len() / 2).copied().unwrap_or(f32::NAN), name.clone(), btw));
            }
        }
        scores.sort_by(|a, b| a.0.total_cmp(&b.0));
        (steps.len(), scores)
    };
    let (n, scores) = consistency(None);
    println!("hand-eye check on {n} steps (>2°) from the no-IMU tracker; median ICP-vs-IMU angle:");
    for (e, name, btw) in scores.iter().take(6) {
        println!("  {name:12} body_to_world={btw:5}: {e:.2}°");
    }

    let run = |imu_to_sensor: Option<Quat>, body_to_world: bool| {
        let config = TrackerConfig { imu_to_sensor, imu_body_to_world: body_to_world, fusion_voxel_mm: 1.0, ..Default::default() };
        let mut tracker = Tracker::with_config(config);
        let (mut ok, mut rmse, mut first_loss) = (0, 0.0, None);
        for (i, (cloud, imu)) in frames.iter().enumerate() {
            if let Some(imu) = imu {
                tracker.update_imu(*imu);
            }
            let s = tracker.register_frame(cloud);
            if s.state == TrackingState::Lost {
                first_loss.get_or_insert(i);
            } else {
                ok += 1;
                rmse += s.rmse_mm;
            }
        }
        (ok, rmse / ok.max(1) as f32, first_loss)
    };

    let mut results = Vec::new();
    let (ok, rmse, loss) = run(None, true);
    println!("no IMU prior: {ok} tracked, rmse {rmse:.3}, first loss {loss:?}");
    for (name, m) in axis_rotations() {
        for body_to_world in [true, false] {
            let (ok, rmse, loss) = run(Some(Quat::from_mat3(&m)), body_to_world);
            println!("{name:12} body_to_world={body_to_world:5}: {ok:3} tracked, rmse {rmse:.3}, first loss {loss:?}");
            results.push((ok, -rmse, name.clone(), body_to_world, m));
        }
    }
    results.sort_by(|a, b| b.0.cmp(&a.0).then(b.1.total_cmp(&a.1)));
    println!("\nbest:");
    for (ok, neg_rmse, name, btw, m) in results.iter().take(5) {
        println!("  {name:12} body_to_world={btw}: {ok} tracked, rmse {:.3}, quat {:?}", -neg_rmse, Quat::from_mat3(m));
    }
}

/// The 24 rotations that map coordinate axes onto (signed) coordinate axes, named by where the
/// IMU's x, y, z axes go in sensor coordinates.
fn axis_rotations() -> Vec<(String, Mat3)> {
    let axes = [Vec3::X, -Vec3::X, Vec3::Y, -Vec3::Y, Vec3::Z, -Vec3::Z];
    let label = |v: Vec3| match (v.x, v.y, v.z) {
        (x, _, _) if x > 0.5 => "+x",
        (x, _, _) if x < -0.5 => "-x",
        (_, y, _) if y > 0.5 => "+y",
        (_, y, _) if y < -0.5 => "-y",
        (_, _, z) if z > 0.5 => "+z",
        _ => "-z",
    };
    let mut out = Vec::new();
    for &a in &axes {
        for &b in &axes {
            if a.dot(b).abs() > 0.5 {
                continue;
            }
            let c = a.cross(b);
            out.push((format!("{}{}{}", label(a), label(b), label(c)), Mat3::from_cols(a, b, c)));
        }
    }
    out
}

/// Normal of the plane with most points within 1.5 mm (RANSAC), oriented towards the sensor.
fn largest_plane_normal(pts: &[Vec3]) -> Option<Vec3> {
    if pts.len() < 100 {
        return None;
    }
    let mut seed = 0x9E3779B9u32;
    let mut rand = |n: usize| {
        seed ^= seed << 13;
        seed ^= seed >> 17;
        seed ^= seed << 5;
        seed as usize % n
    };
    let mut best = (0usize, Vec3::ZERO, 0.0f32);
    for _ in 0..400 {
        let (a, b, c) = (pts[rand(pts.len())], pts[rand(pts.len())], pts[rand(pts.len())]);
        let n = (b - a).cross(c - a).normalize_or_zero();
        if n == Vec3::ZERO {
            continue;
        }
        let d = n.dot(a);
        let count = pts.iter().filter(|p| (n.dot(**p) - d).abs() < 1.5).count();
        if count > best.0 {
            best = (count, n, d);
        }
    }
    let (count, n, d) = best;
    // Require a dominant plane, and orient it towards the sensor at the origin.
    (count * 4 > pts.len()).then(|| if d > 0.0 { -n } else { n })
}

