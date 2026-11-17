//! Colour (RGB-D) tracking: the sensor pose from image features (Revo's "Color" tracking mode).
//!
//! Per frame: ORB-style features on the half-resolution grey colour image, each given a 3D point
//! from the depth map registered into the colour image. Features are matched against a few
//! recent tracking keyframes whose features carry world points. A RANSAC over 3D-3D
//! correspondences (Kabsch) gives a first pose; Gauss-Newton on the reprojection error of all
//! inlier world points (PnP, including those without current depth) refines it. The result is a
//! prior for the geometric tracker, which refines it with ICP and validates it as usual.
//!
//! Works where geometry alone is ambiguous (planes, revolving bodies) as long as the surface is
//! textured; fails on uniform colour, where geometric tracking still applies.

use crate::features::{detect, match_features, Feature, FeatureConfig};
use crate::image_ops::{downsample2, to_gray};
use crate::registration::DepthInColor;
use glam::{Quat, Vec2, Vec3};
use scancraft_core::{ColorCamera, ColorFrame, DepthMap, Transform};
use std::collections::VecDeque;

#[derive(Debug, Clone)]
pub struct ColorTrackerConfig {
    pub features: FeatureConfig,
    /// Depth range used for feature 3D points, mm.
    pub min_depth_mm: f32,
    pub max_depth_mm: f32,
    pub max_hamming: u32,
    pub ratio: f32,
    /// Tracking keyframes matched against per frame (most recent first).
    pub match_keyframes: usize,
    /// Tracking keyframes kept.
    pub max_keyframes: usize,
    pub ransac_iterations: usize,
    /// 3D-3D inlier distance, mm.
    pub inlier_mm: f32,
    /// Reprojection inlier distance after refinement, full-resolution colour pixels.
    pub inlier_px: f32,
    pub min_inliers: usize,
    pub min_inlier_ratio: f32,
    /// A frame becomes a tracking keyframe once the sensor moved this far from all keyframes ...
    pub keyframe_translation_mm: f32,
    pub keyframe_rotation_rad: f32,
    /// ... or matched fewer than this many inliers.
    pub keyframe_min_inliers: usize,
    /// After this many consecutive frames without a colour pose, the keyframes are replaced by
    /// the next accepted (geometry-tracked) frame.
    pub restart_after_failures: usize,
}

impl Default for ColorTrackerConfig {
    fn default() -> Self {
        Self {
            features: FeatureConfig::default(),
            min_depth_mm: 100.0,
            max_depth_mm: 800.0,
            max_hamming: 64,
            ratio: 0.85,
            match_keyframes: 3,
            max_keyframes: 10,
            ransac_iterations: 300,
            inlier_mm: 4.0,
            inlier_px: 4.0,
            min_inliers: 20,
            min_inlier_ratio: 0.2,
            keyframe_translation_mm: 15.0,
            keyframe_rotation_rad: 8f32.to_radians(),
            keyframe_min_inliers: 80,
            restart_after_failures: 5,
        }
    }
}

/// Diagnostics of the last [`ColorTracker::estimate`].
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct ColorTrackStats {
    pub features: usize,
    /// Features with a 3D point from depth.
    pub features_with_depth: usize,
    pub matches: usize,
    pub inliers: usize,
    /// RMS reprojection error of the inliers, full-resolution pixels.
    pub rms_px: f32,
    pub keyframes: usize,
}

/// Features of one frame with their scanner-frame points (where depth exists).
#[derive(Debug, Clone)]
struct Observation {
    features: Vec<Feature>,
    /// Feature positions at full colour resolution.
    pixels: Vec<Vec2>,
    points: Vec<Option<Vec3>>,
}

#[derive(Debug, Clone)]
struct TrackKeyframe {
    pose: Transform,
    features: Vec<Feature>,
    world: Vec<Vec3>,
}

pub struct ColorTracker {
    pub config: ColorTrackerConfig,
    keyframes: VecDeque<TrackKeyframe>,
    pending: Option<Observation>,
    /// Whether the pending frame's pose came from colour (only those may become keyframes).
    pending_tracked: bool,
    /// Consecutive frames without a colour pose.
    failures: usize,
    pub last_stats: ColorTrackStats,
}

impl Default for ColorTracker {
    fn default() -> Self {
        Self::new(ColorTrackerConfig::default())
    }
}

/// Correspondence between a current feature and a keyframe world point.
#[derive(Debug, Clone, Copy)]
struct Corr {
    /// Undistorted normalised image coordinates of the current feature.
    norm: Vec2,
    sensor: Option<Vec3>,
    world: Vec3,
}

impl ColorTracker {
    pub fn new(config: ColorTrackerConfig) -> Self {
        Self { config, keyframes: VecDeque::new(), pending: None, pending_tracked: false, failures: 0, last_stats: ColorTrackStats::default() }
    }

    pub fn keyframe_count(&self) -> usize {
        self.keyframes.len()
    }

    fn observe(&self, color: &ColorFrame, camera: &ColorCamera, depth: Option<&DepthMap>) -> Observation {
        let gray = downsample2(&to_gray(&color.image));
        let features = detect(&gray, &self.config.features);
        // Half-resolution pixel x covers full-resolution pixels 2x and 2x + 1.
        let pixels: Vec<Vec2> = features.iter().map(|f| f.pos * 2.0 + Vec2::splat(0.5)).collect();
        let points = match depth {
            Some(d) if d.intrinsics.is_some() => {
                let reg = DepthInColor::new(d, &camera.scaled(gray.width as u32, gray.height as u32), self.config.min_depth_mm, self.config.max_depth_mm, 2);
                // The point on the feature's own ray at the registered depth (the nearest depth
                // sample itself can be half a depth pixel to the side).
                features.iter().zip(&pixels).map(|(f, px)| reg.depth_at(f.pos).map(|z| camera.color_to_sensor(camera.unproject_color(*px, z)))).collect()
            }
            _ => vec![None; features.len()],
        };
        Observation { features, pixels, points }
    }

    /// Estimates the sensor pose (world <- scanner frame) of `color` + `depth` from feature
    /// matches against the tracking keyframes. `predicted` (e.g. the last accepted pose) is only
    /// used to pick the refinement's start if RANSAC has too few 3D-3D matches. The frame is kept
    /// so [`accept`](Self::accept) can turn it into a keyframe.
    pub fn estimate(&mut self, color: &ColorFrame, depth: Option<&DepthMap>, predicted: &Transform) -> Option<Transform> {
        let camera = color.camera.as_deref()?;
        let camera = if (camera.width, camera.height) == (color.image.width, color.image.height) {
            camera.clone()
        } else {
            camera.scaled(color.image.width, color.image.height)
        };
        let obs = self.observe(color, &camera, depth);
        let mut stats = ColorTrackStats {
            features: obs.features.len(),
            features_with_depth: obs.points.iter().flatten().count(),
            keyframes: self.keyframes.len(),
            ..Default::default()
        };

        let mut corrs = Vec::new();
        for kf in self.keyframes.iter().rev().take(self.config.match_keyframes) {
            for (qi, ti, _) in match_features(&obs.features, &kf.features, self.config.max_hamming, self.config.ratio, |_, _| true) {
                let k = camera.intrinsics;
                let px = obs.pixels[qi];
                let (x, y) = camera.undistort((px.x - k.cx) / k.fx, (px.y - k.cy) / k.fy);
                corrs.push(Corr { norm: Vec2::new(x, y), sensor: obs.points[qi], world: kf.world[ti] });
            }
        }
        stats.matches = corrs.len();
        self.pending = Some(obs);

        let pose = self.solve(&corrs, &camera, predicted, &mut stats);
        self.last_stats = stats;
        self.pending_tracked = pose.is_some();
        self.failures = if pose.is_some() { 0 } else { self.failures + 1 };
        pose
    }

    fn solve(&self, corrs: &[Corr], camera: &ColorCamera, predicted: &Transform, stats: &mut ColorTrackStats) -> Option<Transform> {
        if corrs.len() < self.config.min_inliers {
            return None;
        }
        let with_depth: Vec<(Vec3, Vec3)> = corrs.iter().filter_map(|c| c.sensor.map(|s| (s, c.world))).collect();
        let ransac = ransac_3d(&with_depth, self.config.ransac_iterations, self.config.inlier_mm);
        let start = ransac.map(|(p, _)| p).unwrap_or(*predicted);
        let f = camera.intrinsics.fx;
        let px_tol = self.config.inlier_px / f;
        // Refine on reprojection, re-selecting inliers with a shrinking gate.
        let mut pose = start;
        for gate in [px_tol * 4.0, px_tol * 2.0, px_tol] {
            let inliers: Vec<&Corr> = corrs.iter().filter(|c| reprojection(camera, &pose, c).is_some_and(|r| r.length() < gate)).collect();
            if inliers.len() < 6 {
                return None;
            }
            pose = refine_pnp(camera, pose, &inliers, gate * 0.5);
        }
        let errors: Vec<f32> = corrs.iter().filter_map(|c| reprojection(camera, &pose, c)).map(|r| r.length()).filter(|&e| e < px_tol).collect();
        stats.inliers = errors.len();
        stats.rms_px = if errors.is_empty() { 0.0 } else { (errors.iter().map(|e| e * e).sum::<f32>() / errors.len() as f32).sqrt() * f };
        let ok = stats.inliers >= self.config.min_inliers && stats.inliers as f32 >= self.config.min_inlier_ratio * corrs.len() as f32;
        ok.then_some(pose)
    }

    /// Tells the tracker the pose finally accepted for the last estimated frame (after ICP).
    /// The frame becomes a tracking keyframe if it sees new ground.
    pub fn accept(&mut self, pose: &Transform) {
        let Some(obs) = self.pending.take() else { return };
        // A frame placed without colour (geometry only) would anchor future colour poses to a
        // pose colour never confirmed: on a plane that is exactly the drift colour should fix.
        // Allowed only to start, or to restart after colour has failed for a while.
        let restart = self.keyframes.is_empty() || self.failures >= self.config.restart_after_failures;
        if !self.pending_tracked && !restart {
            return;
        }
        if restart && !self.pending_tracked {
            self.keyframes.clear();
        }
        let far_from_all = self.keyframes.iter().all(|k| {
            (k.pose.translation - pose.translation).length() > self.config.keyframe_translation_mm
                || k.pose.rotation.angle_between(pose.rotation) > self.config.keyframe_rotation_rad
        });
        if !(self.keyframes.is_empty() || far_from_all || self.last_stats.inliers < self.config.keyframe_min_inliers) {
            return;
        }
        let (features, world): (Vec<Feature>, Vec<Vec3>) =
            obs.features.iter().zip(&obs.points).filter_map(|(f, p)| p.map(|p| (*f, pose.transform_point3(p)))).unzip();
        if features.len() < self.config.min_inliers {
            return;
        }
        self.keyframes.push_back(TrackKeyframe { pose: *pose, features, world });
        while self.keyframes.len() > self.config.max_keyframes {
            self.keyframes.pop_front();
        }
    }

    /// Drops the frame of the last estimate (it was not accepted).
    pub fn reject(&mut self) {
        self.pending = None;
    }

    pub fn reset(&mut self) {
        self.keyframes.clear();
        self.pending = None;
    }
}

/// Normalised-coordinate reprojection residual of a correspondence under `pose`.
#[inline]
fn reprojection(camera: &ColorCamera, pose: &Transform, c: &Corr) -> Option<Vec2> {
    let sensor = pose.rotation.inverse() * (c.world - pose.translation);
    let p = camera.sensor_to_color(sensor);
    (p.z > 1.0).then(|| Vec2::new(p.x / p.z, p.y / p.z) - c.norm)
}

/// RANSAC over 3D-3D pairs (sensor point, world point): pose with the most pairs within `tol` mm.
fn ransac_3d(pairs: &[(Vec3, Vec3)], iterations: usize, tol: f32) -> Option<(Transform, usize)> {
    if pairs.len() < 3 {
        return None;
    }
    let mut state = 0x9e37_79b9_7f4a_7c15u64 ^ pairs.len() as u64;
    let mut rnd = |n: usize| {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        (state % n as u64) as usize
    };
    let count = |pose: &Transform| pairs.iter().filter(|(s, w)| (pose.transform_point3(*s) - *w).length() < tol).count();
    let mut best: Option<(Transform, usize)> = None;
    for _ in 0..iterations {
        let (a, b, c) = (rnd(pairs.len()), rnd(pairs.len()), rnd(pairs.len()));
        if a == b || b == c || a == c {
            continue;
        }
        // Degenerate (nearly collinear) samples give unstable rotations.
        let (pa, pb, pc) = (pairs[a].0, pairs[b].0, pairs[c].0);
        if (pb - pa).cross(pc - pa).length() < 25.0 {
            continue;
        }
        let Some(pose) = triad_pose([pa, pb, pc], [pairs[a].1, pairs[b].1, pairs[c].1]) else { continue };
        let n = count(&pose);
        if best.as_ref().is_none_or(|(_, bn)| n > *bn) {
            best = Some((pose, n));
        }
    }
    // The reprojection refinement that follows re-fits on all inliers.
    best
}

/// Exact rigid transform taking triangle `src` onto triangle `dst` (triad method: an orthonormal
/// frame per triangle). Unlike an SVD fit it has no trouble with the rank-deficient
/// cross-covariance of three (or coplanar) points.
fn triad_pose(src: [Vec3; 3], dst: [Vec3; 3]) -> Option<Transform> {
    let frame = |t: [Vec3; 3]| -> Option<glam::Mat3> {
        let e1 = (t[1] - t[0]).try_normalize()?;
        let e3 = e1.cross(t[2] - t[0]).try_normalize()?;
        Some(glam::Mat3::from_cols(e1, e3.cross(e1), e3))
    };
    let r = frame(dst)? * frame(src)?.transpose();
    let cs = (src[0] + src[1] + src[2]) / 3.0;
    let cd = (dst[0] + dst[1] + dst[2]) / 3.0;
    let q = Quat::from_mat3(&r).normalize();
    Some(Transform::from_rotation_translation(q, cd - q * cs))
}

/// Applies a small world-frame update `[rx, ry, rz, tx, ty, tz]` (rad, mm) to a pose.
fn apply(pose: &Transform, d: &[f32; 6]) -> Transform {
    let w = Vec3::new(d[0], d[1], d[2]);
    let dq = if w.length() > 0.0 { Quat::from_scaled_axis(w) } else { Quat::IDENTITY };
    // Rotate about the pose's own position so rotation and translation updates decouple.
    Transform::from_rotation_translation((dq * pose.rotation).normalize(), pose.translation + Vec3::new(d[3], d[4], d[5]))
}

/// Gauss-Newton on the reprojection error (Huber weights with threshold `huber`, normalised
/// units) with a numeric Jacobian.
#[allow(clippy::needless_range_loop)]
fn refine_pnp(camera: &ColorCamera, start: Transform, corrs: &[&Corr], huber: f32) -> Transform {
    let mut pose = start;
    const EPS: [f32; 6] = [1e-4, 1e-4, 1e-4, 1e-2, 1e-2, 1e-2];
    for _ in 0..6 {
        let mut h = [[0f64; 6]; 6];
        let mut g = [0f64; 6];
        for c in corrs {
            let Some(r0) = reprojection(camera, &pose, c) else { continue };
            let mut jac = [[0f32; 6]; 2];
            let mut ok = true;
            for k in 0..6 {
                let mut d = [0f32; 6];
                d[k] = EPS[k];
                let Some(r1) = reprojection(camera, &apply(&pose, &d), c) else {
                    ok = false;
                    break;
                };
                let dr = (r1 - r0) / EPS[k];
                jac[0][k] = dr.x;
                jac[1][k] = dr.y;
            }
            if !ok {
                continue;
            }
            let e = r0.length();
            let w = if e <= huber { 1.0 } else { huber / e } as f64;
            for row in 0..2 {
                let r = if row == 0 { r0.x } else { r0.y } as f64;
                for a in 0..6 {
                    g[a] += w * jac[row][a] as f64 * r;
                    for b in 0..6 {
                        h[a][b] += w * jac[row][a] as f64 * jac[row][b] as f64;
                    }
                }
            }
        }
        for (a, row) in h.iter_mut().enumerate() {
            row[a] *= 1.0 + 1e-6;
            row[a] += 1e-9;
        }
        let Some(step) = solve6(h, g.map(|v| -v)) else { break };
        pose = apply(&pose, &step.map(|v| v as f32));
        if step.iter().take(3).map(|v| v * v).sum::<f64>().sqrt() < 1e-6 && step.iter().skip(3).map(|v| v * v).sum::<f64>().sqrt() < 1e-4 {
            break;
        }
    }
    pose
}

/// Solves a 6x6 linear system by Gaussian elimination with partial pivoting.
#[allow(clippy::needless_range_loop)]
fn solve6(mut a: [[f64; 6]; 6], mut b: [f64; 6]) -> Option<[f64; 6]> {
    for col in 0..6 {
        let piv = (col..6).max_by(|&i, &j| a[i][col].abs().total_cmp(&a[j][col].abs()))?;
        if a[piv][col].abs() < 1e-18 {
            return None;
        }
        a.swap(col, piv);
        b.swap(col, piv);
        for row in col + 1..6 {
            let f = a[row][col] / a[col][col];
            for k in col..6 {
                a[row][k] -= f * a[col][k];
            }
            b[row] -= f * b[col];
        }
    }
    let mut x = [0f64; 6];
    for row in (0..6).rev() {
        let s: f64 = (row + 1..6).map(|k| a[row][k] * x[k]).sum();
        x[row] = (b[row] - s) / a[row][row];
    }
    Some(x)
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::Mat3;
    use scancraft_core::PinholeIntrinsics;
    use std::sync::Arc;

    fn camera() -> Arc<ColorCamera> {
        Arc::new(ColorCamera {
            width: 640,
            height: 512,
            intrinsics: PinholeIntrinsics { fx: 780.0, fy: 780.0, cx: 320.0, cy: 256.0 },
            distortion: [0.0; 5],
            rotation: Mat3::IDENTITY,
            translation: Vec3::new(-24.0, 0.0, 0.0),
        })
    }

    fn render_plane(pose: &Transform, texture: &crate::Gray, cam: &ColorCamera) -> (ColorFrame, DepthMap) {
        let plane = crate::synthetic::TexturedPlane { texture: texture.clone(), distance_mm: 300.0, texels_per_mm: 4.0, depth_noise_mm: 0.3 };
        plane.render(pose, cam, PinholeIntrinsics { fx: 371.0, fy: 371.0, cx: 160.0, cy: 128.0 }, (320, 256), 0)
    }

    #[test]
    fn tracks_in_plane_motion_over_a_textured_plane() {
        let cam = camera();
        let texture = crate::features::blob_texture(900, 700, 3);
        let mut tracker = ColorTracker::default();
        let start = Transform::default();
        let (c0, d0) = render_plane(&start, &texture, &cam);
        assert!(tracker.estimate(&c0, Some(&d0), &start).is_none(), "nothing to match yet");
        tracker.accept(&start);
        assert_eq!(tracker.keyframe_count(), 1);

        // Slide along the plane (what ICP on a plane cannot see) and turn a little.
        let truth = Transform::from_rotation_translation(Quat::from_rotation_z(0.03), Vec3::new(9.0, 4.0, -6.0));
        let (c1, d1) = render_plane(&truth, &texture, &cam);
        let est = tracker.estimate(&c1, Some(&d1), &start).expect("tracks");
        let s = tracker.last_stats;
        let err_t = (est.translation - truth.translation).length();
        let err_r = est.rotation.angle_between(truth.rotation).to_degrees();
        assert!(err_t < 0.5 && err_r < 0.2, "error {err_t:.3} mm / {err_r:.3} deg, {s:?}");
        assert!(s.inliers > 100, "{s:?}");
    }

    #[test]
    fn solves_small_linear_systems() {
        let mut a = [[0f64; 6]; 6];
        for (i, row) in a.iter_mut().enumerate() {
            row[i] = 2.0;
            row[(i + 1) % 6] = 0.5;
        }
        let x = [1.0, -2.0, 3.0, 0.5, 0.0, 4.0];
        let b: [f64; 6] = std::array::from_fn(|i| (0..6).map(|k| a[i][k] * x[k]).sum());
        let s = solve6(a, b).unwrap();
        assert!(s.iter().zip(x).all(|(a, b)| (a - b).abs() < 1e-9));
    }
}
