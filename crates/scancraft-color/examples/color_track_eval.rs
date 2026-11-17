//! Evaluates colour tracking and vertex colouring.
//!
//! `cargo run --release -p scancraft-color --example color_track_eval -- <dir> <depth.bin> <rgb.bin> [out_dir]`
//!
//! 1. **Static recording** (real POP 4 data): colour-tracker pose jitter against the first
//!    frame, feature / inlier counts and run time per frame.
//! 2. **Synthetic flat textured object**: a plane textured with a real colour frame, seen with the
//!    real calibration along a sliding / turning trajectory. The geometric tracker alone cannot
//!    see motion along a plane; with the colour prior it can. Reports the pose error per frame
//!    for both.
//! 3. **Vertex colours** for the first real depth frame from the recording's keyframes; writes
//!    the coloured depth image (each depth pixel in its vertex colour) to `out_dir`.

mod common;

use glam::{Quat, Vec3};
use scancraft_color::image_ops::to_gray;
use scancraft_color::synthetic::TexturedPlane;
use scancraft_color::{colorize_points, frame_sharpness, ColorTracker, ColorizeConfig, RgbKeyframe};
use scancraft_core::{ColorRgba, Point3D, Transform};
use scancraft_slam::{HintKind, PoseHint, Tracker, TrackingState};
use std::path::PathBuf;
use std::time::Instant;

#[derive(Debug, Clone, Copy, PartialEq)]
enum Mode {
    IcpOnly,
    Prior,
    FixedWhenConfident,
}

impl Mode {
    fn label(self) -> &'static str {
        match self {
            Mode::IcpOnly => "ICP only            ",
            Mode::Prior => "colour prior + ICP  ",
            Mode::FixedWhenConfident => "colour fixed/prior  ",
        }
    }
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let dir = PathBuf::from(args.get(1).expect("usage: color_track_eval <dir> <depth.bin> <rgb.bin> [out_dir]"));
    let out_dir = args.get(4).map(PathBuf::from);
    let (pairs, cam) = common::load_pairs(&dir, &args[2], &args[3], 80);
    println!("{} paired frames", pairs.len());

    // 1. Static jitter.
    let mut tracker = ColorTracker::default();
    let origin = Transform::default();
    let (mut max_t, mut max_r, mut sum_t, mut n, mut ms) = (0f32, 0f32, 0f32, 0, Vec::new());
    for (i, pair) in pairs.iter().enumerate() {
        let t0 = Instant::now();
        let est = tracker.estimate(&pair.color, Some(&pair.depth), &origin);
        ms.push(t0.elapsed().as_secs_f32() * 1000.0);
        let s = tracker.last_stats;
        if i == 0 {
            tracker.accept(&origin);
            println!("frame 0: {} features, {} with depth -> keyframe", s.features, s.features_with_depth);
            continue;
        }
        match est {
            Some(p) => {
                let (dt, dr) = (p.translation.length(), p.rotation.angle_between(Quat::IDENTITY).to_degrees());
                max_t = max_t.max(dt);
                max_r = max_r.max(dr);
                sum_t += dt;
                n += 1;
                if i % 10 == 0 {
                    println!("frame {i}: {} matches, {} inliers, rms {:.2} px, |t| {dt:.3} mm, rot {dr:.3} deg", s.matches, s.inliers, s.rms_px);
                }
                tracker.accept(&p);
            }
            None => {
                println!("frame {i}: no pose ({s:?})");
                tracker.reject();
            }
        }
    }
    ms.sort_by(f32::total_cmp);
    println!(
        "static: {n}/{} frames tracked, translation jitter mean {:.3} / max {:.3} mm, rotation max {:.3} deg, estimate {:.0} ms median ({:.0} ms max)",
        pairs.len() - 1,
        sum_t / n.max(1) as f32,
        max_t,
        max_r,
        ms[ms.len() / 2],
        ms[ms.len() - 1]
    );

    // 2. Synthetic plane textured with a real colour frame.
    let plane = TexturedPlane { texture: to_gray(&pairs[pairs.len() / 2].color.image), distance_mm: 300.0, texels_per_mm: 4.0, depth_noise_mm: 0.3 };
    let depth_k = pairs[0].depth.intrinsics.unwrap();
    let depth_size = (pairs[0].depth.width, pairs[0].depth.height);
    let truth_at = |i: usize| {
        let f = i as f32;
        // Slide right then up along the plane, turning about the plane normal and wobbling.
        let t = if i < 30 { Vec3::new(1.5 * f, 0.0, 0.0) } else { Vec3::new(45.0, 0.0, 1.2 * (f - 30.0)) };
        let r = Quat::from_rotation_y(0.004 * f) * Quat::from_rotation_z(0.01 * (f * 0.3).sin()) * Quat::from_rotation_x(0.01 * (f * 0.2).sin());
        Transform::from_rotation_translation(r, t + Vec3::new(0.0, 2.0 * (f * 0.25).sin(), 0.0))
    };
    let frames = 55;
    for mode in [Mode::IcpOnly, Mode::Prior, Mode::FixedWhenConfident] {
        let use_color = mode != Mode::IcpOnly;
        let mut geo = Tracker::new();
        let mut col = ColorTracker::default();
        let (mut worst, mut lost, mut last_err, mut t_ms) = (0f32, 0, 0f32, 0f32);
        for i in 0..frames {
            let truth = truth_at(i);
            let (color, depth) = plane.render(&truth, &cam, depth_k, depth_size, i as u64 * 85);
            let cloud = depth.to_point_cloud(150.0, 450.0);
            let t0 = Instant::now();
            let hint = if use_color {
                col.estimate(&color, Some(&depth), &geo.current_pose).map(|pose| {
                    let s = col.last_stats;
                    // Same rule as scancraft_pipeline::color::ColorPoseEstimator.
                    let strong = mode == Mode::FixedWhenConfident && s.inliers >= 100 && s.rms_px <= 1.8;
                    PoseHint { pose, kind: if strong { HintKind::Fixed } else { HintKind::Prior } }
                })
            } else {
                None
            };
            t_ms += t0.elapsed().as_secs_f32() * 1000.0;
            if use_color && std::env::var_os("EVAL_VERBOSE").is_some() {
                let e = hint.map(|h| ((h.pose.translation - truth.translation).length(), h.pose.rotation.angle_between(truth.rotation).to_degrees()));
                println!("    colour frame {i}: {:?} err {:?}", col.last_stats, e);
            }
            let status = geo.register_with(&cloud, Some(&depth), hint);
            let ok = matches!(status.state, TrackingState::Tracking | TrackingState::Slow);
            if use_color {
                if ok {
                    col.accept(&status.current_pose);
                } else {
                    col.reject();
                }
            }
            if !ok {
                lost += 1;
            }
            // The tracker's world is the first frame's pose, which is the identity here too.
            let est = status.current_pose;
            let err = (est.translation - truth.translation).length();
            worst = worst.max(err);
            last_err = err;
            if i % 9 == 0 || i == frames - 1 {
                println!(
                    "  {} frame {i:2}: truth t=({:5.1},{:5.1},{:5.1}) est t=({:5.1},{:5.1},{:5.1}) err {err:6.2} mm, rot err {:.2} deg, {:?}",
                    mode.label(),
                    truth.translation.x,
                    truth.translation.y,
                    truth.translation.z,
                    est.translation.x,
                    est.translation.y,
                    est.translation.z,
                    est.rotation.angle_between(truth.rotation).to_degrees(),
                    status.state
                );
            }
        }
        println!(
            "synthetic plane, {}: final error {last_err:.2} mm, worst {worst:.2} mm, {lost} of {frames} frames not accepted{}",
            mode.label(),
            if use_color { format!(", colour estimate {:.0} ms/frame", t_ms / frames as f32) } else { String::new() }
        );
    }

    // 3. Vertex colours for the first frame's points from keyframes of the recording.
    let pair0 = &pairs[0];
    let k = pair0.depth.intrinsics.unwrap();
    let (w, h) = (pair0.depth.width as usize, pair0.depth.height as usize);
    let cloud = pair0.depth.to_point_cloud(150.0, 600.0);
    let mut points: Vec<Point3D> = cloud.points.iter().map(|p| p.with_color(ColorRgba::WHITE)).collect();
    let keyframes: Vec<RgbKeyframe> = pairs
        .iter()
        .step_by(20)
        .enumerate()
        .filter_map(|(i, p)| RgbKeyframe::new(i as u64, &p.color, cam.clone(), Transform::default(), frame_sharpness(&p.color), i as u64))
        .collect();
    let t0 = Instant::now();
    let stats = colorize_points(&mut points, &keyframes, &ColorizeConfig::default());
    println!(
        "vertex colours: {} of {} points coloured from {} keyframes in {:.0} ms (sharpness {:?})",
        stats.colored,
        stats.points,
        stats.keyframes_used,
        t0.elapsed().as_secs_f32() * 1000.0,
        keyframes.iter().map(|k| k.sharpness as i32).collect::<Vec<_>>()
    );
    if let Some(out) = out_dir {
        std::fs::create_dir_all(&out).unwrap();
        let mut img = image::RgbImage::new(w as u32, h as u32);
        for p in &points {
            let (x, y, z) = (p.position[0], p.position[1], p.position[2]);
            let u = (k.fx * x / y + k.cx).round() as i32;
            let v = (-k.fy * z / y + k.cy).round() as i32;
            if u >= 0 && v >= 0 && (u as usize) < w && (v as usize) < h {
                img.put_pixel(u as u32, v as u32, image::Rgb([p.color.r, p.color.g, p.color.b]));
            }
        }
        img.save(out.join("vertex_colors_depth_view.png")).unwrap();
        println!("wrote {}", out.join("vertex_colors_depth_view.png").display());
    }
}
