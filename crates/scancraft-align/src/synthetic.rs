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

/// Semi-axes of [`shell_part`]'s outer dome, mm.
pub const SHELL_AXES: Vec3 = Vec3::new(100.0, 70.0, 45.0);
/// Width of [`shell_part`]'s flange, mm.
pub const SHELL_FLANGE: f32 = 15.0;

/// A thick-walled shell part like a moulded cover, opening down at z = 0: an elliptic dome
/// ([`SHELL_AXES`]) with walls `wall_mm` thick and a flat flange round the opening
/// ([`SHELL_FLANGE`] wide, as thick as the wall, top face at z = 0). Sampled about `spacing`
/// apart, as (outside faces, inside faces): what a scan of the part standing on its flange sees
/// (outer dome, flange top) and what a scan of it lying upside down sees (inner dome, the inner
/// wall under the dome's rim, flange underside). Both contain the flange's outer edge.
pub fn shell_part(spacing: f32, wall_mm: f32) -> (Vec<Oriented>, Vec<Oriented>) {
    dome_shell(spacing, wall_mm, SHELL_AXES, SHELL_AXES)
}

/// [`shell_part`] with a dome made of quarter ellipsoids: semi-axes `pos` towards +x / +y and
/// `neg` towards -x / -y (the height `pos.z` for all).
fn dome_shell(spacing: f32, wall_mm: f32, pos: Vec3, neg: Vec3) -> (Vec<Oriented>, Vec<Oriented>) {
    let c = pos.z;
    let axes = |theta: f32| (if theta.cos() >= 0.0 { pos.x } else { neg.x }, if theta.sin() >= 0.0 { pos.y } else { neg.y });
    let (mean_a, mean_b) = ((pos.x + neg.x) / 2.0, (pos.y + neg.y) / 2.0);
    let tau = std::f32::consts::TAU;
    // Points round an ellipse of semi-axes (ea, eb), about `spacing` apart.
    let around = |ea: f32, eb: f32| -> usize {
        let h = ((ea - eb) / (ea + eb)).powi(2);
        let perimeter = std::f32::consts::PI * (ea + eb) * (1.0 + 3.0 * h / (10.0 + (4.0 - 3.0 * h).sqrt()));
        (perimeter / spacing).ceil().max(6.0) as usize
    };
    let (mut outside, mut inside) = (Vec::new(), Vec::new());
    // Domes: the inner face is the outer one moved `wall_mm` inwards along the normal.
    let rings = (std::f32::consts::FRAC_PI_2 * pos.x.max(neg.x).max(c) / spacing).ceil() as usize;
    for r in 0..=rings {
        let phi = std::f32::consts::FRAC_PI_2 * r as f32 / rings as f32;
        let count = if r == rings { 1 } else { around(mean_a * phi.cos(), mean_b * phi.cos()) };
        for k in 0..count {
            let theta = tau * k as f32 / count as f32;
            let (a, b) = axes(theta);
            let p = Vec3::new(a * phi.cos() * theta.cos(), b * phi.cos() * theta.sin(), c * phi.sin());
            let n = Vec3::new(p.x / (a * a), p.y / (b * b), p.z / (c * c)).normalize();
            outside.push((p, n));
            inside.push((p - n * wall_mm, -n));
        }
    }
    // Flange round the outer footprint ellipse: `e(theta) + r * n(theta)`, r from the inner wall
    // (-wall) out to the edge.
    let count = around(mean_a + SHELL_FLANGE / 2.0, mean_b + SHELL_FLANGE / 2.0);
    let steps = |len: f32| (len / spacing).ceil().max(1.0) as usize;
    for k in 0..count {
        let theta = tau * k as f32 / count as f32;
        let (a, b) = axes(theta);
        let e = Vec3::new(a * theta.cos(), b * theta.sin(), 0.0);
        let n = Vec3::new(theta.cos() / a, theta.sin() / b, 0.0).normalize();
        let (nr, nz) = (steps(SHELL_FLANGE + wall_mm), steps(wall_mm));
        for i in 0..=nr {
            let r = -wall_mm + (SHELL_FLANGE + wall_mm) * i as f32 / nr as f32;
            if r > 0.0 {
                outside.push((e + n * r, Vec3::Z));
            }
            inside.push((e + n * r - Vec3::Z * wall_mm, -Vec3::Z));
        }
        for j in 1..nz {
            let z = -wall_mm * j as f32 / nz as f32;
            let edge = (e + n * SHELL_FLANGE + Vec3::Z * z, n);
            outside.push(edge);
            inside.push(edge);
            inside.push((e - n * wall_mm + Vec3::Z * z, -n));
        }
    }
    (outside, inside)
}

/// Semi-axes of [`shell_cover`]'s outer dome towards +x and +y (and its height), mm.
pub const COVER_AXES_POS: Vec3 = Vec3::new(110.0, 75.0, 45.0);
/// Semi-axes of [`shell_cover`]'s outer dome towards -x and -y, mm.
pub const COVER_AXES_NEG: Vec3 = Vec3::new(85.0, 60.0, 45.0);
/// Bosses under [`shell_cover`]'s dome: (centre x, centre y, radius, bottom z), mm.
const COVER_BOSSES: [(f32, f32, f32, f32); 3] = [(40.0, 20.0, 6.0, 10.0), (-35.0, -22.0, 5.0, 14.0), (10.0, -35.0, 4.0, 20.0)];

/// A shell part without symmetries, like a real moulded cover: [`shell_part`]'s layout (opening
/// down at z = 0, flange round it) with an egg-shaped dome ([`COVER_AXES_POS`] /
/// [`COVER_AXES_NEG`]), three bosses hanging from the inside of the dome and a lug on the outside
/// near the rim (+y side). Returns (outside faces, inside faces) like [`shell_part`].
pub fn shell_cover(spacing: f32, wall_mm: f32) -> (Vec<Oriented>, Vec<Oriented>) {
    let (mut outside, mut inside) = dome_shell(spacing, wall_mm, COVER_AXES_POS, COVER_AXES_NEG);
    let semi = |x: f32, y: f32| (if x >= 0.0 { COVER_AXES_POS.x } else { COVER_AXES_NEG.x }, if y >= 0.0 { COVER_AXES_POS.y } else { COVER_AXES_NEG.y });
    // Outer dome height over (x, y) and its normal's z component.
    let dome = |x: f32, y: f32| -> (f32, f32) {
        let (a, b) = semi(x, y);
        let c = COVER_AXES_POS.z;
        let z = c * (1.0 - (x / a).powi(2) - (y / b).powi(2)).max(0.0).sqrt();
        (z, Vec3::new(x / (a * a), y / (b * b), z / (c * c)).normalize().z)
    };
    for (cx, cy, r, z0) in COVER_BOSSES {
        // The dome's inside over the boss belongs to the boss's solid.
        inside.retain(|(p, _)| p.z < z0 - 0.5 || Vec3::new(p.x - cx, p.y - cy, 0.0).length() > r);
        let around = (std::f32::consts::TAU * r / spacing).ceil() as usize;
        for i in 0..around {
            let a = std::f32::consts::TAU * i as f32 / around as f32;
            let n = Vec3::new(a.cos(), a.sin(), 0.0);
            let (x, y) = (cx + r * n.x, cy + r * n.y);
            let (h, nz) = dome(x, y);
            let top = h - wall_mm / nz.max(0.3);
            let steps = ((top - z0) / spacing).ceil().max(1.0) as usize;
            for k in 0..=steps {
                inside.push((Vec3::new(x, y, z0 + (top - z0) * k as f32 / steps as f32), n));
            }
        }
        // Bottom face.
        let rings = (r / spacing).ceil() as usize;
        inside.push((Vec3::new(cx, cy, z0), -Vec3::Z));
        for k in 1..rings {
            let rr = r * k as f32 / rings as f32;
            let m = (std::f32::consts::TAU * rr / spacing).ceil() as usize;
            for i in 0..m {
                let a = std::f32::consts::TAU * i as f32 / m as f32;
                inside.push((Vec3::new(cx + rr * a.cos(), cy + rr * a.sin(), z0), -Vec3::Z));
            }
        }
    }
    // The lug: a block standing out of the dome near the rim.
    let lug = Shape::Cuboid { center: Vec3::new(30.0, 70.0, 12.0), half: Vec3::new(9.0, 7.0, 10.0), rotation: Quat::IDENTITY };
    outside.retain(|(p, _)| lug.sdf(*p) > 0.05);
    let mut faces = Vec::new();
    lug.sample(spacing, &mut faces);
    outside.extend(faces.into_iter().filter(|(p, _)| {
        let (a, b) = semi(p.x, p.y);
        (p.x / a).powi(2) + (p.y / b).powi(2) + (p.z / COVER_AXES_POS.z).powi(2) > 1.0
    }));
    (outside, inside)
}

/// Three scan passes of [`shell_cover`] (1 mm samples, 3 mm wall) with their object poses
/// (pass <- part), like the Revo Scan test part:
///
/// 0. the outside: standing on its flange, pass frame = part frame, seen from above all round;
/// 1. the inside, lying upside down (turned and moved), seen from above;
/// 2. the inside again, placed differently, from above and low down, so that it also contains the
///    outside of the dome near the rim (with the lug).
///
/// Each pass has its table as support plane.
pub fn cover_passes(wall_mm: f32) -> (Vec<PassCloud>, Vec<Transform>) {
    let (outer, inner) = shell_cover(1.0, wall_mm);
    let band: Vec<Oriented> = inner.iter().copied().chain(outer.iter().copied().filter(|(p, _)| p.z < 25.0)).collect();
    let upside_down = |turn: f32, at: Vec3| rigid(Quat::from_rotation_x(std::f32::consts::PI) * Quat::from_rotation_z(turn), at);
    let poses = vec![Transform::default(), upside_down(0.7, Vec3::new(40.0, -25.0, 60.0)), upside_down(-2.0, Vec3::new(-30.0, 45.0, 55.0))];
    let ring_at = |pose: &Transform, elevation: f32, az0: f32| ring(pose.transform_point3(Vec3::ZERO), elevation, az0, az0 + 5.6, 8, 340.0);
    let mut low = ring_at(&poses[2], 1.0, 0.3);
    low.extend(ring_at(&poses[2], 0.15, 0.0));
    let mut passes = vec![
        pass(&outer, &poses[0], &ring_at(&poses[0], 0.7, 0.0), 0.08, 1.0, 50),
        pass(&inner, &poses[1], &ring_at(&poses[1], 1.0, 0.2), 0.08, 1.0, 51),
        pass(&band, &poses[2], &low, 0.08, 1.0, 52),
    ];
    passes[0].support = Some(crate::Plane::through(Vec3::new(0.0, 0.0, -wall_mm), Vec3::Z));
    for (p, pose) in passes.iter_mut().zip(&poses).skip(1) {
        // Upside down it rests on the dome's apex.
        p.support = Some(crate::Plane::through(pose.transform_point3(Vec3::new(0.0, 0.0, COVER_AXES_POS.z)), Vec3::Z));
    }
    (passes, poses)
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
    PassCloud { points: downsample(&all, pitch_mm), views: out_views, support: None }
}
