//! Synthetic multi-view scan data with known transforms, for tests and demos: an asymmetric
//! object (a block with a dome, a peg, a tab and a twisted lug), scanner views around it, and
//! passes made of what those views see (occlusion and back faces removed, depth noise added).

use glam::{Mat3, Quat, Vec3};
use scancraft_core::Transform;

use crate::cloud::{downsample, Rng};
use crate::{rigid, Oriented, PassCloud, View};

enum Shape {
    Cuboid { center: Vec3, half: Vec3, rotation: Quat },
    Sphere { center: Vec3, radius: f32 },
    /// Along +X from `x0` to `x1`.
    Cylinder { y: f32, z: f32, x0: f32, x1: f32, radius: f32 },
}

impl Shape {
    fn sdf(&self, p: Vec3) -> f32 {
        match *self {
            Shape::Cuboid { center, half, rotation } => {
                let q = (rotation.inverse() * (p - center)).abs() - half;
                q.max(Vec3::ZERO).length() + q.max_element().min(0.0)
            }
            Shape::Sphere { center, radius } => (p - center).length() - radius,
            Shape::Cylinder { y, z, x0, x1, radius } => {
                let radial = Vec3::new(0.0, p.y - y, p.z - z).length() - radius;
                let axial = (p.x - (x0 + x1) / 2.0).abs() - (x1 - x0) / 2.0;
                let q = glam::Vec2::new(radial, axial);
                q.max(glam::Vec2::ZERO).length() + q.max_element().min(0.0)
            }
        }
    }

    fn sample(&self, s: f32, out: &mut Vec<Oriented>) {
        match *self {
            Shape::Cuboid { center, half, rotation } => {
                for axis in 0..3 {
                    for sign in [-1.0f32, 1.0] {
                        let (a, b) = ((axis + 1) % 3, (axis + 2) % 3);
                        let (na, nb) = ((2.0 * half[a] / s).ceil() as i32, (2.0 * half[b] / s).ceil() as i32);
                        for i in 0..=na {
                            for j in 0..=nb {
                                let mut p = Vec3::ZERO;
                                p[axis] = sign * half[axis];
                                p[a] = -half[a] + 2.0 * half[a] * i as f32 / na as f32;
                                p[b] = -half[b] + 2.0 * half[b] * j as f32 / nb as f32;
                                let mut n = Vec3::ZERO;
                                n[axis] = sign;
                                out.push((center + rotation * p, rotation * n));
                            }
                        }
                    }
                }
            }
            Shape::Sphere { center, radius } => {
                let count = (4.0 * std::f32::consts::PI * radius * radius / (s * s)) as usize;
                let golden = std::f32::consts::PI * (3.0 - 5f32.sqrt());
                for i in 0..count {
                    let z = 1.0 - 2.0 * (i as f32 + 0.5) / count as f32;
                    let r = (1.0 - z * z).sqrt();
                    let a = golden * i as f32;
                    let n = Vec3::new(r * a.cos(), r * a.sin(), z);
                    out.push((center + n * radius, n));
                }
            }
            Shape::Cylinder { y, z, x0, x1, radius } => {
                let around = (2.0 * std::f32::consts::PI * radius / s).ceil() as usize;
                let along = ((x1 - x0) / s).ceil() as usize;
                for i in 0..around {
                    let a = 2.0 * std::f32::consts::PI * i as f32 / around as f32;
                    let n = Vec3::new(0.0, a.cos(), a.sin());
                    for j in 0..=along {
                        let x = x0 + (x1 - x0) * j as f32 / along as f32;
                        out.push((Vec3::new(x, y, z) + n * radius, n));
                    }
                }
                let rings = (radius / s).ceil() as usize;
                for (x, nx) in [(x0, -1.0f32), (x1, 1.0)] {
                    out.push((Vec3::new(x, y, z), Vec3::new(nx, 0.0, 0.0)));
                    for k in 1..=rings {
                        let r = radius * k as f32 / rings as f32;
                        let m = (2.0 * std::f32::consts::PI * r / s).ceil() as usize;
                        for i in 0..m {
                            let a = 2.0 * std::f32::consts::PI * i as f32 / m as f32;
                            out.push((Vec3::new(x, y + r * a.cos(), z + r * a.sin()), Vec3::new(nx, 0.0, 0.0)));
                        }
                    }
                }
            }
        }
    }
}

fn shapes() -> Vec<Shape> {
    vec![
        Shape::Cuboid { center: Vec3::ZERO, half: Vec3::new(45.0, 30.0, 20.0), rotation: Quat::IDENTITY },
        Shape::Sphere { center: Vec3::new(18.0, 5.0, 20.0), radius: 16.0 },
        Shape::Cylinder { y: -8.0, z: -4.0, x0: -72.0, x1: -40.0, radius: 8.0 },
        Shape::Cuboid { center: Vec3::new(-15.0, -36.0, 8.0), half: Vec3::new(12.0, 8.0, 10.0), rotation: Quat::IDENTITY },
        Shape::Cuboid { center: Vec3::new(30.0, 34.0, -6.0), half: Vec3::new(10.0, 7.0, 9.0), rotation: Quat::from_rotation_z(0.5) },
    ]
}

/// Surface samples (about `spacing` apart) of the test object, centred near the origin, about
/// 120 x 80 x 75 mm.
pub fn object(spacing: f32) -> Vec<Oriented> {
    let shapes = shapes();
    let mut out = Vec::new();
    for (i, shape) in shapes.iter().enumerate() {
        let mut samples = Vec::new();
        shape.sample(spacing, &mut samples);
        // Keep the parts of this shape's surface outside every other shape.
        out.extend(samples.into_iter().filter(|(p, _)| shapes.iter().enumerate().all(|(j, o)| j == i || o.sdf(*p) > 0.05)));
    }
    out
}

/// Distance from `p` to the object's surface (for measuring merged results).
pub fn surface_distance(p: Vec3) -> f32 {
    shapes().iter().map(|s| s.sdf(p)).fold(f32::MAX, f32::min).abs()
}

/// Scanner pose (world <- sensor) at `distance` from `target`, in direction (azimuth, elevation),
/// looking at the target. Sensor axes: +X right, +Y forward, +Z up.
pub fn look_at(target: Vec3, azimuth: f32, elevation: f32, distance: f32) -> Transform {
    let dir = Vec3::new(elevation.cos() * azimuth.cos(), elevation.cos() * azimuth.sin(), elevation.sin());
    let eye = target + dir * distance;
    let forward = -dir;
    let right = {
        let r = forward.cross(Vec3::Z);
        if r.length() < 1e-3 { Vec3::X } else { r.normalize() }
    };
    let up = right.cross(forward);
    rigid(Quat::from_mat3(&Mat3::from_cols(right, forward, up)), eye)
}

/// Views on a ring around `target` from `az0` to `az1` (radians) at one elevation.
pub fn ring(target: Vec3, elevation: f32, az0: f32, az1: f32, count: usize, distance: f32) -> Vec<Transform> {
    (0..count).map(|i| look_at(target, az0 + (az1 - az0) * i as f32 / (count.max(2) - 1) as f32, elevation, distance)).collect()
}

/// What the scanner at `pose` sees of `points` (world frame), in the sensor frame: front-facing,
/// unoccluded points inside a 40-degree half-angle field of view, with depth noise of
/// `noise_mm` (standard deviation) along the line of sight.
pub fn observe(points: &[Oriented], pose: &Transform, noise_mm: f32, rng: &mut Rng) -> Vec<Oriented> {
    const BIN: f32 = 0.004; // rad
    const HALF: i32 = 200; // +-0.8 rad
    let inv = pose.inverse();
    let w = (2 * HALF) as usize;
    let mut zbuf = vec![f32::MAX; w * w];
    let key = |q: Vec3| -> Option<usize> {
        if q.y <= 1.0 {
            return None;
        }
        let (u, v) = ((q.x.atan2(q.y) / BIN).floor() as i32 + HALF, (q.z.atan2(q.y) / BIN).floor() as i32 + HALF);
        (u >= 0 && v >= 0 && u < 2 * HALF && v < 2 * HALF && (q.x / q.y).abs() < 0.84 && (q.z / q.y).abs() < 0.84).then(|| v as usize * w + u as usize)
    };
    let local: Vec<Oriented> = points.iter().map(|&(p, n)| (inv.transform_point3(p), inv.rotation * n)).collect();
    for &(q, _) in &local {
        if let Some(k) = key(q) {
            zbuf[k] = zbuf[k].min(q.length());
        }
    }
    local
        .into_iter()
        .filter(|&(q, n)| n.dot(-q.normalize()) > 0.15 && key(q).is_some_and(|k| q.length() <= zbuf[k] + 0.8))
        .map(|(q, n)| (q + q.normalize() * rng.normal() * noise_mm, n))
        .collect()
}

/// A pass: the object placed by `object_pose` (pass <- object) and seen from `views` (pass frame),
/// with noise. The pass cloud is thinned to `pitch_mm`.
pub fn pass(object: &[Oriented], object_pose: &Transform, views: &[Transform], noise_mm: f32, pitch_mm: f32, seed: u64) -> PassCloud {
    let mut rng = Rng::new(seed);
    let placed: Vec<Oriented> = object.iter().map(|&(p, n)| (object_pose.transform_point3(p), object_pose.rotation * n)).collect();
    let mut all = Vec::new();
    let mut out_views = Vec::new();
    for pose in views {
        let seen = observe(&placed, pose, noise_mm, &mut rng);
        all.extend(seen.iter().map(|&(q, n)| (pose.transform_point3(q), pose.rotation * n)));
        out_views.push(View { pose: *pose, points: seen.iter().map(|(q, _)| *q).collect() });
    }
    PassCloud { points: downsample(&all, pitch_mm), views: out_views }
}
