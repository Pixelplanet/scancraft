//! Ground-truth shapes as signed distance functions (mm, negative inside the material).
//!
//! Every shape is plain data (serde), so `truth.json` carries the exact shape and
//! [`Shape::sdf`] can be evaluated again from it. Most primitives are exact distances; the
//! combinations (`max` / `min`), chamfers and height fields are lower bounds of the true distance
//! with the exact zero set, which is all ray marching and scoring need.

use glam::{DQuat, DVec2, DVec3};
use serde::{Deserialize, Serialize};

pub type V2 = [f64; 2];
pub type V3 = [f64; 3];

pub fn v3(a: V3) -> DVec3 {
    DVec3::from_array(a)
}

pub fn v2(a: V2) -> DVec2 {
    DVec2::from_array(a)
}

/// A rigid motion `p -> R p + t` (rotation as a unit quaternion x, y, z, w).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Rigid {
    pub rotation: [f64; 4],
    pub translation: V3,
}

impl Default for Rigid {
    fn default() -> Self {
        Self::IDENTITY
    }
}

impl Rigid {
    pub const IDENTITY: Rigid = Rigid { rotation: [0.0, 0.0, 0.0, 1.0], translation: [0.0; 3] };

    pub fn new(rotation: DQuat, translation: DVec3) -> Self {
        let q = rotation.normalize();
        Rigid { rotation: [q.x, q.y, q.z, q.w], translation: translation.to_array() }
    }

    /// Rotation about `axis` by `degrees`, then the translation.
    pub fn axis_angle(axis: V3, degrees: f64, translation: V3) -> Self {
        Self::new(DQuat::from_axis_angle(v3(axis).normalize(), degrees.to_radians()), v3(translation))
    }

    pub fn quat(&self) -> DQuat {
        DQuat::from_xyzw(self.rotation[0], self.rotation[1], self.rotation[2], self.rotation[3]).normalize()
    }

    pub fn apply(&self, p: DVec3) -> DVec3 {
        self.quat() * p + v3(self.translation)
    }

    pub fn apply_dir(&self, d: DVec3) -> DVec3 {
        self.quat() * d
    }

    pub fn inverse(&self) -> Rigid {
        let q = self.quat().inverse();
        Rigid::new(q, -(q * v3(self.translation)))
    }

    /// `self` after `first`: p -> self(first(p)).
    pub fn after(&self, first: &Rigid) -> Rigid {
        Rigid::new(self.quat() * first.quat(), self.apply(v3(first.translation)))
    }

    /// Row-major 4 x 4 matrix.
    pub fn matrix(&self) -> [[f64; 4]; 4] {
        let m = glam::DMat3::from_quat(self.quat());
        let t = self.translation;
        let r = |i: usize| [m.col(0)[i], m.col(1)[i], m.col(2)[i], t[i]];
        [r(0), r(1), r(2), [0.0, 0.0, 0.0, 1.0]]
    }
}

/// How a corner of a [`Profile2::Rect`] is finished.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum Corner {
    Sharp,
    /// A 45° chamfer taking `size` mm off both edges.
    Chamfer { size: f64 },
    Fillet { radius: f64 },
}

/// A 2D region (signed distance in the plane, negative inside).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum Profile2 {
    Circle { center: V2, radius: f64 },
    /// A slot with round ends: all points within `radius` of the segment a-b (width 2 radius).
    Stadium { a: V2, b: V2, radius: f64 },
    RoundRect { center: V2, half: V2, radius: f64 },
    /// A rectangle with its own finish per corner, in the order (+x,+y), (-x,+y), (-x,-y), (+x,-y).
    Rect { center: V2, half: V2, corners: [Corner; 4] },
    /// Pen strokes: points within `radius` of any segment.
    Strokes { segments: Vec<[V2; 2]>, radius: f64 },
    Ring { center: V2, inner: f64, outer: f64 },
    /// `count` circles on a grid starting at `first` with `pitch` (radius below half the pitch).
    CircleGrid { first: V2, pitch: V2, count: [usize; 2], radius: f64 },
    Union { parts: Vec<Profile2> },
}

fn segment_distance(p: DVec2, a: DVec2, b: DVec2) -> f64 {
    let pa = p - a;
    let ba = b - a;
    let len2 = ba.length_squared();
    let h = if len2 > 0.0 { (pa.dot(ba) / len2).clamp(0.0, 1.0) } else { 0.0 };
    (pa - ba * h).length()
}

fn box2(a: DVec2) -> f64 {
    a.max(DVec2::ZERO).length() + a.x.max(a.y).min(0.0)
}

impl Profile2 {
    pub fn sdf(&self, p: DVec2) -> f64 {
        match self {
            Profile2::Circle { center, radius } => (p - v2(*center)).length() - radius,
            Profile2::Stadium { a, b, radius } => segment_distance(p, v2(*a), v2(*b)) - radius,
            Profile2::RoundRect { center, half, radius } => {
                let q = (p - v2(*center)).abs() - v2(*half) + DVec2::splat(*radius);
                box2(q) - radius
            }
            Profile2::Rect { center, half, corners } => {
                let q = p - v2(*center);
                let corner = match (q.x >= 0.0, q.y >= 0.0) {
                    (true, true) => corners[0],
                    (false, true) => corners[1],
                    (false, false) => corners[2],
                    (true, false) => corners[3],
                };
                let a = q.abs() - v2(*half);
                match corner {
                    Corner::Sharp => box2(a),
                    Corner::Fillet { radius } => box2(a + DVec2::splat(radius)) - radius,
                    Corner::Chamfer { size } => box2(a).max((a.x + a.y + size) * std::f64::consts::FRAC_1_SQRT_2),
                }
            }
            Profile2::Strokes { segments, radius } => {
                segments.iter().map(|[a, b]| segment_distance(p, v2(*a), v2(*b))).fold(f64::INFINITY, f64::min) - radius
            }
            Profile2::Ring { center, inner, outer } => {
                ((p - v2(*center)).length() - 0.5 * (inner + outer)).abs() - 0.5 * (outer - inner)
            }
            Profile2::CircleGrid { first, pitch, count, radius } => {
                let i = ((p.x - first[0]) / pitch[0]).round().clamp(0.0, (count[0].max(1) - 1) as f64);
                let j = ((p.y - first[1]) / pitch[1]).round().clamp(0.0, (count[1].max(1) - 1) as f64);
                (p - DVec2::new(first[0] + i * pitch[0], first[1] + j * pitch[1])).length() - radius
            }
            Profile2::Union { parts } => parts.iter().map(|s| s.sdf(p)).fold(f64::INFINITY, f64::min),
        }
    }
}

/// A ridge along a line, `height * exp(-(d / width)^2)` with `d` the distance from the line.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Band {
    pub point: V2,
    pub direction: V2,
    pub height: f64,
    pub width: f64,
}

/// Height over a base level, z = base + h(x, y).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum HeightFn {
    Flat,
    /// x² / (2 rx) - y² / (2 ry) plus ridges; a radius of 0 means flat in that direction.
    Saddle { rx: f64, ry: f64, bands: Vec<Band> },
    /// Diamond knurl: pyramids `depth` high on a `pitch` grid turned by `angle_deg`.
    Knurl { center: V2, pitch: f64, depth: f64, angle_deg: f64 },
    /// Smooth bumps (1 - (r/radius)²)² · height.
    Bumps { centers: Vec<V2>, radius: f64, height: f64 },
}

fn tri(t: f64) -> f64 {
    1.0 - 2.0 * (t - t.floor() - 0.5).abs()
}

impl HeightFn {
    pub fn eval(&self, x: f64, y: f64) -> f64 {
        match self {
            HeightFn::Flat => 0.0,
            HeightFn::Saddle { rx, ry, bands } => {
                let mut z = 0.0;
                if *rx != 0.0 {
                    z += x * x / (2.0 * rx);
                }
                if *ry != 0.0 {
                    z -= y * y / (2.0 * ry);
                }
                for b in bands {
                    let d = v2(b.direction).normalize();
                    let n = DVec2::new(-d.y, d.x);
                    let dist = (DVec2::new(x, y) - v2(b.point)).dot(n);
                    z += b.height * (-(dist / b.width).powi(2)).exp();
                }
                z
            }
            HeightFn::Knurl { center, pitch, depth, angle_deg } => {
                let (s, c) = angle_deg.to_radians().sin_cos();
                let (x, y) = (x - center[0], y - center[1]);
                let u = (c * x + s * y) / pitch;
                let v = (-s * x + c * y) / pitch;
                depth * tri(u).min(tri(v))
            }
            HeightFn::Bumps { centers, radius, height } => centers
                .iter()
                .map(|c| {
                    let r2 = ((x - c[0]).powi(2) + (y - c[1]).powi(2)) / (radius * radius);
                    if r2 < 1.0 {
                        height * (1.0 - r2).powi(2)
                    } else {
                        0.0
                    }
                })
                .fold(0.0, f64::max),
        }
    }

    fn gradient(&self, x: f64, y: f64) -> DVec2 {
        const E: f64 = 1e-4;
        DVec2::new(self.eval(x + E, y) - self.eval(x - E, y), self.eval(x, y + E) - self.eval(x, y - E)) / (2.0 * E)
    }
}

/// A thin layer on a surface that is real geometry (paint, a printed line).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum Coat {
    /// A band `2 half_width` wide along the outline of `outline` (seen from above), `height`
    /// thick, on the surface where z > `min_z`; edges ramp over `edge` mm.
    Stripe { outline: Profile2, half_width: f64, height: f64, edge: f64, min_z: f64 },
}

impl Coat {
    pub fn thickness(&self, p: DVec3) -> f64 {
        match self {
            Coat::Stripe { outline, half_width, height, edge, min_z } => {
                if p.z <= *min_z {
                    return 0.0;
                }
                let d = outline.sdf(DVec2::new(p.x, p.y)).abs();
                height * ramp(d, half_width - edge, *half_width)
            }
        }
    }
}

/// 1 below `a`, 0 above `b`, smooth in between.
pub fn ramp(d: f64, a: f64, b: f64) -> f64 {
    if d <= a {
        1.0
    } else if d >= b {
        0.0
    } else {
        let t = (d - a) / (b - a);
        1.0 - t * t * (3.0 - 2.0 * t)
    }
}

/// A solid (signed distance, negative inside).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum Shape {
    /// Axis-aligned box with rounded edges (`round` 0 = sharp).
    Box { center: V3, half: V3, round: f64 },
    Sphere { center: V3, radius: f64 },
    /// Cylinder with flat caps between `a` and `b`.
    Cylinder { a: V3, b: V3, radius: f64 },
    /// Points with normal · p <= offset.
    HalfSpace { normal: V3, offset: f64 },
    /// `profile` (x, y) extruded between z0 and z1.
    Prism { profile: Profile2, z0: f64, z1: f64 },
    /// `profile` in (x, z) extruded along y between y0 and y1.
    ExtrudeY { profile: Profile2, y0: f64, y1: f64 },
    /// Within `footprint`, from z = bottom up to z = base + height(x, y).
    HeightField { footprint: Profile2, bottom: f64, base: f64, height: HeightFn },
    Union { parts: Vec<Shape> },
    Intersect { parts: Vec<Shape> },
    /// Intersection of two shapes with the crease between them rounded over about `k` mm.
    SmoothIntersect { a: std::boxed::Box<Shape>, b: std::boxed::Box<Shape>, k: f64 },
    Subtract { base: std::boxed::Box<Shape>, cuts: Vec<Shape> },
    /// The shape shrunk by `by` mm (exact for exact distances; used for walls of shells).
    Inset { shape: std::boxed::Box<Shape>, by: f64 },
    /// `shape` moved by `pose`.
    Placed { shape: std::boxed::Box<Shape>, pose: Rigid },
    /// `shape` with real thin layers on its surface.
    Coated { shape: std::boxed::Box<Shape>, coats: Vec<Coat> },
}

fn extrude(d: f64, along: f64, lo: f64, hi: f64) -> f64 {
    let w = DVec2::new(d, (lo - along).max(along - hi));
    box2(w)
}

impl Shape {
    pub fn sdf(&self, p: DVec3) -> f64 {
        match self {
            Shape::Box { center, half, round } => {
                let q = (p - v3(*center)).abs() - v3(*half) + DVec3::splat(*round);
                q.max(DVec3::ZERO).length() + q.x.max(q.y.max(q.z)).min(0.0) - round
            }
            Shape::Sphere { center, radius } => (p - v3(*center)).length() - radius,
            Shape::Cylinder { a, b, radius } => {
                let (a, b) = (v3(*a), v3(*b));
                let axis = b - a;
                let len = axis.length();
                let u = axis / len;
                let t = (p - a).dot(u);
                let radial = ((p - a) - u * t).length() - radius;
                extrude(radial, t, 0.0, len)
            }
            Shape::HalfSpace { normal, offset } => v3(*normal).normalize().dot(p) - offset,
            Shape::Prism { profile, z0, z1 } => extrude(profile.sdf(DVec2::new(p.x, p.y)), p.z, *z0, *z1),
            Shape::ExtrudeY { profile, y0, y1 } => extrude(profile.sdf(DVec2::new(p.x, p.z)), p.y, *y0, *y1),
            Shape::HeightField { footprint, bottom, base, height } => {
                let top = base + height.eval(p.x, p.y);
                let g = height.gradient(p.x, p.y);
                let dz = (p.z - top) / (1.0 + g.length_squared()).sqrt();
                let vertical = dz.max(bottom - p.z);
                let f = footprint.sdf(DVec2::new(p.x, p.y));
                box2(DVec2::new(f, vertical))
            }
            Shape::Union { parts } => parts.iter().map(|s| s.sdf(p)).fold(f64::INFINITY, f64::min),
            Shape::Intersect { parts } => parts.iter().map(|s| s.sdf(p)).fold(f64::NEG_INFINITY, f64::max),
            Shape::SmoothIntersect { a, b, k } => {
                let (da, db) = (a.sdf(p), b.sdf(p));
                let h = (0.5 - 0.5 * (db - da) / k).clamp(0.0, 1.0);
                db + (da - db) * h + k * h * (1.0 - h)
            }
            Shape::Subtract { base, cuts } => cuts.iter().map(|c| -c.sdf(p)).fold(base.sdf(p), f64::max),
            Shape::Inset { shape, by } => shape.sdf(p) + by,
            Shape::Placed { shape, pose } => shape.sdf(pose.inverse().apply(p)),
            Shape::Coated { shape, coats } => shape.sdf(p) - coats.iter().map(|c| c.thickness(p)).sum::<f64>(),
        }
    }

    /// Outward unit normal (central differences of the distance).
    pub fn normal(&self, p: DVec3) -> DVec3 {
        sdf_normal(|q| self.sdf(q), p)
    }

    pub fn union(parts: Vec<Shape>) -> Shape {
        Shape::Union { parts }
    }

    pub fn subtract(base: Shape, cuts: Vec<Shape>) -> Shape {
        Shape::Subtract { base: std::boxed::Box::new(base), cuts }
    }

    pub fn placed(self, pose: Rigid) -> Shape {
        Shape::Placed { shape: std::boxed::Box::new(self), pose }
    }
}

/// Outward unit normal of any distance function at `p`.
pub fn sdf_normal(f: impl Fn(DVec3) -> f64, p: DVec3) -> DVec3 {
    const E: f64 = 2e-4;
    let g = DVec3::new(
        f(p + DVec3::X * E) - f(p - DVec3::X * E),
        f(p + DVec3::Y * E) - f(p - DVec3::Y * E),
        f(p + DVec3::Z * E) - f(p - DVec3::Z * E),
    );
    g.normalize_or_zero()
}

/// Moves `p` onto the zero set of `f` (a few Newton steps along the gradient).
pub fn project_to_surface(f: impl Fn(DVec3) -> f64, mut p: DVec3) -> DVec3 {
    for _ in 0..6 {
        let d = f(p);
        if d.abs() < 1e-7 {
            break;
        }
        let n = sdf_normal(&f, p);
        if n == DVec3::ZERO {
            break;
        }
        p -= n * d;
    }
    p
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() <= tol
    }

    #[test]
    fn primitives_give_exact_distances() {
        let plate = Shape::Box { center: [0.0, 0.0, -2.0], half: [50.0, 40.0, 2.0], round: 0.0 };
        assert!(close(plate.sdf(DVec3::new(0.0, 0.0, 1.5)), 1.5, 1e-12));
        assert!(close(plate.sdf(DVec3::new(0.0, 0.0, -1.0)), -1.0, 1e-12));
        assert!(close(plate.sdf(DVec3::new(53.0, 44.0, 0.0)), 5.0, 1e-12));
        let s = Shape::Sphere { center: [0.0, 0.0, -45.0], radius: 60.0 };
        assert!(close(s.sdf(DVec3::new(0.0, 0.0, 15.0)), 0.0, 1e-12));
        let c = Shape::Cylinder { a: [-10.0, 0.0, 0.0], b: [10.0, 0.0, 0.0], radius: 15.0 };
        assert!(close(c.sdf(DVec3::new(0.0, 0.0, 16.0)), 1.0, 1e-12));
        assert!(close(c.sdf(DVec3::new(12.0, 0.0, 0.0)), 2.0, 1e-12));
        let n = c.normal(DVec3::new(0.0, 15.0 * 0.6, 15.0 * 0.8));
        assert!((n - DVec3::new(0.0, 0.6, 0.8)).length() < 1e-6);
    }

    #[test]
    fn slots_and_corners_have_their_nominal_sizes() {
        // A 3 mm slot: the walls are 1.5 mm from the centre line.
        let slot = Profile2::Stadium { a: [-10.0, 0.0], b: [10.0, 0.0], radius: 1.5 };
        assert!(close(slot.sdf(DVec2::new(0.0, 1.5)), 0.0, 1e-12));
        assert!(close(slot.sdf(DVec2::new(11.5, 0.0)), 0.0, 1e-12));
        let rect = Profile2::Rect {
            center: [0.0, 0.0],
            half: [10.0, 5.0],
            corners: [Corner::Sharp, Corner::Fillet { radius: 2.0 }, Corner::Chamfer { size: 1.0 }, Corner::Sharp],
        };
        // Sharp corner: the corner point itself is on the boundary.
        assert!(close(rect.sdf(DVec2::new(10.0, 5.0)), 0.0, 1e-12));
        // Fillet of 2 mm: the arc's midpoint.
        let mid = DVec2::new(-8.0, 3.0) + DVec2::new(-1.0, 1.0).normalize() * 2.0;
        assert!(close(rect.sdf(mid), 0.0, 1e-12));
        assert!(rect.sdf(DVec2::new(-10.0, 5.0)) > 0.5);
        // Chamfer of 1 mm: the corner is cut, the chamfer's ends are on the boundary.
        assert!(close(rect.sdf(DVec2::new(-9.0, -5.0)), 0.0, 1e-12));
        assert!(close(rect.sdf(DVec2::new(-10.0, -4.0)), 0.0, 1e-12));
        assert!(close(rect.sdf(DVec2::new(-10.0, -5.0)), std::f64::consts::FRAC_1_SQRT_2, 1e-12));
    }

    #[test]
    fn rigid_motions_compose_and_invert() {
        let a = Rigid::axis_angle([1.0, 0.0, 0.0], 180.0, [0.0, 0.0, 10.0]);
        let b = Rigid::axis_angle([0.0, 0.0, 1.0], 30.0, [5.0, -2.0, 0.0]);
        let p = DVec3::new(1.0, 2.0, 3.0);
        assert!((a.inverse().apply(a.apply(p)) - p).length() < 1e-12);
        assert!((b.after(&a).apply(p) - b.apply(a.apply(p))).length() < 1e-12);
        let m = b.matrix();
        let q = b.apply(p);
        for i in 0..3 {
            assert!(close(m[i][0] * p.x + m[i][1] * p.y + m[i][2] * p.z + m[i][3], q[i], 1e-12));
        }
    }

    #[test]
    fn shapes_round_trip_through_json() {
        let s = Shape::subtract(
            Shape::Box { center: [0.0; 3], half: [5.0; 3], round: 0.5 },
            vec![Shape::Prism { profile: Profile2::Circle { center: [0.0, 0.0], radius: 1.0 }, z0: -10.0, z1: 10.0 }],
        )
        .placed(Rigid::axis_angle([0.0, 1.0, 0.0], 20.0, [1.0, 2.0, 3.0]));
        let json = serde_json::to_string(&s).unwrap();
        let back: Shape = serde_json::from_str(&json).unwrap();
        assert_eq!(back, s);
        let p = DVec3::new(2.0, 1.0, 4.0);
        assert_eq!(back.sdf(p), s.sdf(p));
    }
}
