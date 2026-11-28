//! Turntable capture on the simulator's turntable scene (`scancraft_driver::mock`): a fixed
//! scanner looking down at a turning turntable with three objects and marker stickers, on a static
//! table. A full turn through `pipeline_for` with Feature and with Marker tracking: the object is
//! tracked (poses follow the turn), fused without smearing, the table is removed, and the IMU
//! (which says the scanner stands still) is not used as a motion prior.
//!
//! `cargo test --release -p scancraft-pipeline --test mock_turntable -- --nocapture`

use glam::{Quat, Vec3};
use scancraft_core::Transform;
use scancraft_driver::mock::{MockDevice, MockMotion};
use scancraft_driver::{ScanConfig, TrackingMode, TurntableMode};
use scancraft_slam::{Tracker, TrackerConfig, TrackingState};

fn compose(a: &Transform, b: &Transform) -> Transform {
    Transform::from_rotation_translation((a.rotation * b.rotation).normalize(), a.rotation * b.translation + a.translation)
}

struct Run {
    tracked: usize,
    lost: usize,
    skipped: usize,
    max_rot_err_deg: f32,
    max_trans_err_mm: f32,
    final_turn_deg: f32,
    points: Vec<Vec3>,
    region_radius: f32,
}

/// One turntable pass of `frames` frames; errors are of the tracker's relative pose against the
/// simulator's ground truth (sensor in the turntable frame) from the first tracked frame on.
fn run(tracking: TrackingMode, frames: u64, motion: MockMotion) -> Run {
    let config = ScanConfig { turntable_mode: TurntableMode::TurntableContinuous, tracking, color_texture: false, ..ScanConfig::default() };
    let mut pipeline = scancraft_pipeline::pipeline_for(&config, 150.0, 450.0).unwrap();
    assert!(pipeline.depth.sensor_static(), "turntable capture keeps the IMU out of tracking");
    // The simulator's IMU reports the scanner's orientation in the world (identity mounting): a
    // prior from it would hold the pose still while the object turns.
    let mut tracker = Tracker::with_config(TrackerConfig { fusion_voxel_mm: 0.8, imu_to_sensor: Some(Quat::IDENTITY), ..TrackerConfig::default() });
    let (mut tracked, mut lost, mut skipped) = (0, 0, 0);
    let mut first: Option<(Transform, Transform)> = None;
    let (mut max_rot, mut max_trans) = (0.0f32, 0.0f32);
    let mut last = Transform::default();
    for i in 1..=frames {
        let frame = MockDevice::scan_frame(&config, &motion, i);
        assert!(frame.imu.is_some());
        let Some(status) = pipeline.process(&frame, &mut tracker) else {
            skipped += 1;
            continue;
        };
        if status.state == TrackingState::Lost {
            lost += 1;
            continue;
        }
        tracked += 1;
        let truth = MockDevice::scan_truth(&config, &motion, i);
        let (t0, p0) = *first.get_or_insert((truth, status.current_pose));
        // Both relative to the first tracked frame.
        let want = compose(&t0.inverse(), &truth);
        let got = compose(&p0.inverse(), &status.current_pose);
        max_rot = max_rot.max(want.rotation.angle_between(got.rotation).to_degrees());
        max_trans = max_trans.max((want.translation - got.translation).length());
        last = got;
    }
    // Fused points in the turntable frame.
    let (t0, p0) = first.expect("tracking started");
    let to_turntable = compose(&t0, &p0.inverse());
    let points = tracker.fused_points().iter().map(|p| to_turntable.transform_point3(Vec3::from(p.position))).collect();
    let final_turn_deg = last.rotation.angle_between(Quat::IDENTITY).to_degrees();
    let region_radius = pipeline.turntable_region().map_or(0.0, |r| r.radius_mm);
    Run { tracked, lost, skipped, max_rot_err_deg: max_rot, max_trans_err_mm: max_trans, final_turn_deg, points, region_radius }
}

/// The simulator's vase radius at height `z` and angle `angle` (turntable frame; `mock.rs`).
fn vase_radius(z: f32, angle: f32) -> f32 {
    let dz = z - 55.0;
    let contour = 32.0 + 9.0 * (dz * 0.055).cos() - (dz * 0.015).powi(2) * 8.0;
    (contour + 2.5 * (6.0 * angle).sin()).max(14.0)
}

/// On or near the canister or the block (turntable frame), which also reach within 47 mm of the
/// vase axis.
fn near_other_object(p: Vec3) -> bool {
    (p.x - 66.0).hypot(p.y - 14.0) < 25.0 || (p.x + 62.0).hypot(p.y + 14.0) < 30.0
}

/// Distance of a turntable-frame point to the simulator's objects (vase approximated by its
/// bounding radius profile is too coarse, so only the table and the outside are checked).
fn on_static_table(p: Vec3) -> bool {
    p.z < -3.0 || (p.x * p.x + p.y * p.y).sqrt() > 107.0
}

fn check(name: &str, r: &Run, frames: u64, motion: &MockMotion) {
    let turn = (frames - 1) as f32 * motion.turntable_step_rad.to_degrees();
    println!(
        "{name}: {} tracked, {} lost, {} without geometry; pose error max {:.2} deg / {:.2} mm over {turn:.0} deg; {} points, region radius {:.0} mm",
        r.tracked,
        r.lost,
        r.skipped,
        r.max_rot_err_deg,
        r.max_trans_err_mm,
        r.points.len(),
        r.region_radius
    );
    assert!(r.skipped <= 3, "{name}: only the frames that find the table carry no geometry ({})", r.skipped);
    assert!(r.lost * 20 <= frames as usize, "{name}: {} of {frames} frames lost", r.lost);
    assert!(r.max_rot_err_deg < 2.5, "{name}: rotation error {:.2} deg", r.max_rot_err_deg);
    assert!(r.max_trans_err_mm < 8.0, "{name}: translation error {:.2} mm", r.max_trans_err_mm);
    let _ = r.final_turn_deg;
    // Not smeared and the table removed: nothing of the static table, and the object's points
    // stay on the turntable.
    let table = r.points.iter().filter(|p| on_static_table(**p)).count();
    assert!(table * 200 < r.points.len(), "{name}: {table} of {} fused points lie on the static table", r.points.len());
    let disc_top = r.points.iter().filter(|p| p.z.abs() < 1.5).count();
    assert!(disc_top * 20 < r.points.len(), "{name}: the turntable top is removed ({disc_top} of {})", r.points.len());
    assert!(r.points.len() > 20_000, "{name}: {} points", r.points.len());
    // Not smeared: the vase's side lies on the simulator's surface (its radius profile).
    let mut off: Vec<f32> = r.points.iter().filter(|p| (15.0..105.0).contains(&p.z) && (p.x * p.x + p.y * p.y).sqrt() < 47.0 && !near_other_object(**p)).map(|p| ((p.x * p.x + p.y * p.y).sqrt() - vase_radius(p.z, p.y.atan2(p.x))).abs()).collect();
    off.sort_by(f32::total_cmp);
    assert!(off.len() > 2_000, "{name}: {} vase points", off.len());
    let (median, p95) = (off[off.len() / 2], off[off.len() * 95 / 100]);
    println!("{name}: vase side {} points, distance to the true surface median {median:.2} mm, 95 % {p95:.2} mm", off.len());
    assert!(median < 0.5 && p95 < 1.5, "{name}: smeared vase, median {median:.2} mm, 95 % {p95:.2} mm");
}

#[test]
fn a_full_turn_is_tracked_and_fused_with_feature_tracking() {
    let motion = MockMotion { turntable_step_rad: 2.0f32.to_radians(), ..MockMotion::default() };
    let frames = 181; // 360 degrees
    let r = run(TrackingMode::Feature, frames, motion);
    check("feature", &r, frames, &motion);
}

#[test]
fn a_full_turn_is_tracked_and_fused_with_markers_on_the_turntable() {
    let motion = MockMotion { turntable_step_rad: 2.0f32.to_radians(), ..MockMotion::default() };
    let frames = 181;
    let r = run(TrackingMode::Marker, frames, motion);
    check("marker", &r, frames, &motion);
}

#[test]
fn without_turntable_mode_the_static_table_holds_the_pose_still() {
    // The documented failure the turntable mode exists for, on the simulator's scene.
    let motion = MockMotion { turntable_step_rad: 2.0f32.to_radians(), ..MockMotion::default() };
    let config = ScanConfig { turntable_mode: TurntableMode::TurntableContinuous, ..ScanConfig::default() };
    let handheld = ScanConfig { turntable_mode: TurntableMode::HandheldFreeform, ..ScanConfig::default() };
    let mut pipeline = scancraft_pipeline::pipeline_for(&handheld, 150.0, 450.0).unwrap();
    let mut tracker = Tracker::new();
    let mut poses = Vec::new();
    for i in 1..=16 {
        if let Some(s) = pipeline.process(&MockDevice::scan_frame(&config, &motion, i), &mut tracker) {
            poses.push(s.current_pose);
        }
    }
    let turned = poses.last().unwrap().rotation.angle_between(poses[0].rotation).to_degrees();
    println!("handheld processing of a turntable pass: turned {turned:.1} deg of 30");
    assert!(turned < 15.0, "turned {turned:.1} deg");
}


/// The POP 4 Full-field preset's depth window (250-500 mm), as the app applies it, with a manual
/// turntable's larger steps. (With the scene at 280 mm instead of 330 the near limit cut the
/// objects and the measured turn came out 3.7 % short: keep the object inside the window.)
#[test]
fn the_full_field_depth_window_and_larger_steps_measure_the_turn() {
    for (deg, voxel) in [(2.5f32, 0.5f32)] {
        let motion = MockMotion { turntable_step_rad: deg.to_radians(), ..MockMotion::default() };
        let frames = (120.0 / deg) as u64 + 3;
        let config = ScanConfig { turntable_mode: TurntableMode::TurntableContinuous, color_texture: false, ..ScanConfig::default() };
        let mut pipeline = scancraft_pipeline::pipeline_for(&config, 250.0, 500.0).unwrap();
        let mut tracker = Tracker::with_config(TrackerConfig { fusion_voxel_mm: voxel, ..TrackerConfig::default() });
        let mut first = None;
        let mut last = (0u64, Transform::default());
        for i in 1..=frames {
            if let Some(s) = pipeline.process(&MockDevice::scan_frame(&config, &motion, i), &mut tracker) {
                if s.state != TrackingState::Lost {
                    first.get_or_insert((i, s.current_pose));
                    last = (i, s.current_pose);
                }
            }
        }
        let (i0, p0) = first.unwrap();
        let turned = p0.rotation.angle_between(last.1.rotation).to_degrees();
        let truth = (last.0 - i0) as f32 * deg;
        println!("step {deg} voxel {voxel}: turned {turned:.2} of {truth:.2}");
        assert!((turned - truth).abs() < 1.0, "turned {turned:.2} of {truth:.2} deg");
    }
}
