//! Primitive fits grown from a clicked point: `Surface` indexes an oriented point set, then
//! `fit_plane` / `fit_cylinder` seed a RANSAC model in the clicked neighbourhood, grow the
//! region point by point and refit by least squares. Used by the datum picker (3-2-1
//! alignment) and by measurement tools.

use std::collections::VecDeque;

use glam::Vec3;

use crate::cloud::{Index, Rng};
use crate::eigen::{eigen3, vec3_of};
use crate::{Oriented, Plane};

/// A fitted primitive.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Primitive {
    /// A flat region.
    Plane {
        /// Centroid of the inliers.
        point: Vec3,
        /// Unit normal, pointing out of the surface (agreeing with the inliers' normals).
        normal: Vec3,
    },
    /// A round region (a shaft, a hole, a boss).
    Cylinder {
        /// A point on the axis, at the middle of the inliers' extent along it.
        point: Vec3,
        /// Unit axis direction.
        axis: Vec3,
        /// Radius, mm.
        radius: f32,
        /// Extent of the inliers along the axis, mm.
        length: f32,
    },
}

/// A fitted primitive with its inlier set and residual statistics.
#[derive(Debug, Clone)]
pub struct Fit {
    /// The fitted primitive.
    pub primitive: Primitive,
    /// RMS of the inlier residuals, mm.
    pub rms_mm: f32,
    /// 95th percentile of the inlier |residuals|, mm.
    pub p95_mm: f32,
    /// Indices into the `Surface`'s points.
    pub inliers: Vec<u32>,
    /// Cylinders: the angular range the inliers cover around the axis (360 = all round).
    /// Planes: 0.
    pub coverage_deg: f32,
}

/// Growing and acceptance limits of a fit.
#[derive(Debug, Clone, Copy)]
pub struct FitConfig {
    /// Residual limit for growing, mm.
    pub tolerance_mm: f32,
    /// Max angle between a point's normal and the primitive's normal there, degrees.
    pub normal_deg: f32,
}

impl Default for FitConfig {
    fn default() -> Self {
        FitConfig { tolerance_mm: 0.3, normal_deg: 25.0 }
    }
}

/// An oriented point set with a neighbour index and the typical point spacing.
pub struct Surface {
    points: Vec<Oriented>,
    index: Index,
    spacing: f32,
}

impl Surface {
    /// Indexes `points` and estimates the spacing (median nearest-neighbour distance on a
    /// sample of up to 2048 points).
    pub fn new(points: Vec<Oriented>) -> Self {
        let index = Index::new(points.iter().map(|p| p.0).collect());
        let n = points.len();
        let step = (n / 2048).max(1);
        let mut d: Vec<f32> = (0..n).step_by(step).filter_map(|i| index.nearest_n(points[i].0, 2).get(1).map(|k| k.0.sqrt())).collect();
        let spacing = if d.is_empty() { 1.0 } else { let mid = d.len() / 2; *d.select_nth_unstable_by(mid, f32::total_cmp).1 };
        Surface { points, index, spacing }
    }

    /// The points.
    pub fn points(&self) -> &[Oriented] {
        &self.points
    }

    /// The estimated point spacing, mm.
    pub fn spacing(&self) -> f32 {
        self.spacing
    }

    /// (index, distance) of the point nearest to `p`.
    pub fn nearest(&self, p: Vec3) -> Option<(u32, f32)> {
        self.index.nearest(p).map(|(d, i)| (i as u32, d.sqrt()))
    }

    /// Points within `radius` of the surface point nearest to `seed` (the fit's seed
    /// neighbourhood). None when the neighbourhood has fewer than 12 points.
    fn neighbourhood(&self, seed: Vec3) -> Option<(u32, Vec<usize>, f32)> {
        let (seed_i, _) = self.nearest(seed)?;
        let radius = (6.0 * self.spacing).max(2.0);
        let hood = self.index.within(self.points[seed_i as usize].0, radius);
        (hood.len() >= 12).then_some((seed_i, hood, radius))
    }

    /// Least-squares plane grown from `seed`: RANSAC in the click's neighbourhood, then
    /// breadth-first growth and PCA refits until the region stops doubling, and one last
    /// growth from scratch so the inliers are consistent with the final plane. None when the
    /// final region has fewer than 30 points.
    pub fn fit_plane(&self, seed: Vec3, config: &FitConfig) -> Option<Fit> {
        let (seed_i, hood, _) = self.neighbourhood(seed)?;
        let cosn = config.normal_deg.to_radians().cos();
        let tol = config.tolerance_mm;
        // RANSAC: three points make a plane; it must explain the clicked point and is scored
        // by the neighbourhood it covers.
        let mut rng = Rng::new(0x50E_ED1);
        let accept = |pl: &Plane, i: usize| pl.distance(self.points[i].0).abs() <= tol && pl.normal.dot(self.points[i].1).abs() >= cosn;
        let mut best: Option<(Plane, usize)> = None;
        for _ in 0..160 {
            let (i, j, k) = (hood[rng.below(hood.len())], hood[rng.below(hood.len())], hood[rng.below(hood.len())]);
            if i == j || j == k || i == k {
                continue;
            }
            let (a, b, c) = (self.points[i].0, self.points[j].0, self.points[k].0);
            let n = (b - a).cross(c - a).normalize_or_zero();
            if n == Vec3::ZERO {
                continue;
            }
            let pl = Plane::through(a, n);
            if !accept(&pl, seed_i as usize) {
                continue;
            }
            let count = hood.iter().filter(|&&i| accept(&pl, i)).count();
            if best.map_or(true, |(_, c)| count > c) {
                best = Some((pl, count));
            }
        }
        let (mut pl, _) = best?;
        // Orient the plane normal the way the surface faces.
        if pl.normal.dot(self.points[seed_i as usize].1) < 0.0 {
            pl.normal = -pl.normal;
            pl.d = -pl.d;
        }
        let inliers = self.grow_plane(seed_i as usize, &mut pl, tol, cosn, true);
        pl = pca_plane(&inliers, &self.points, pl.normal);
        let inliers = self.grow_plane(seed_i as usize, &mut pl, tol, cosn, false);
        if inliers.len() < 30 {
            return None;
        }
        let mut residuals: Vec<f32> = inliers.iter().map(|&i| pl.distance(self.points[i].0).abs()).collect();
        residuals.sort_by(f32::total_cmp);
        let rms_mm = (residuals.iter().map(|r| r * r).sum::<f32>() / residuals.len() as f32).sqrt();
        let p95_mm = residuals[(residuals.len() * 95 / 100).min(residuals.len() - 1)];
        let centroid = inliers.iter().map(|&i| self.points[i].0).sum::<Vec3>() / inliers.len() as f32;
        Some(Fit {
            primitive: Primitive::Plane { point: centroid, normal: pl.normal },
            rms_mm,
            p95_mm,
            inliers: inliers.into_iter().map(|i| i as u32).collect(),
            coverage_deg: 0.0,
        })
    }

    /// Breadth-first growth from `start` over neighbours within 2.5 x spacing, accepting
    /// points on `pl` (residual <= tol) with normals within `cosn` of the plane normal. With
    /// `refit`, the plane is re-fit by PCA each time the region has doubled.
    fn grow_plane(&self, start: usize, pl: &mut Plane, tol: f32, cosn: f32, refit: bool) -> Vec<usize> {
        let near = 2.5 * self.spacing;
        let mut seen = vec![false; self.points.len()];
        let mut out = Vec::new();
        let mut queue = VecDeque::from([start]);
        let mut next_refit = 60usize;
        seen[start] = true;
        while let Some(i) = queue.pop_front() {
            let (p, n) = self.points[i];
            if pl.distance(p).abs() > tol || pl.normal.dot(n).abs() < cosn {
                continue;
            }
            out.push(i);
            if refit && out.len() >= next_refit {
                *pl = pca_plane(&out, &self.points, pl.normal);
                next_refit = out.len() * 2;
            }
            for j in self.index.within(p, near) {
                if !seen[j] {
                    seen[j] = true;
                    queue.push_back(j);
                }
            }
        }
        out
    }

    /// Least-squares cylinder grown from `seed`: RANSAC pairs whose normals cross to the
    /// axis direction (plus a candidate from the neighbourhood's normal scatter), a circle
    /// fit of the consensus projected along the axis, then breadth-first growth and
    /// Gauss-Newton refits. Works for outside scans (normals off the axis) and inside scans
    /// (a bore, normals to the axis; one consistent sign per fit). None when the final
    /// region has fewer than 30 points, wraps less than 20 degrees around the axis, or is
    /// effectively flat (radius > 50 x the region's extent).
    pub fn fit_cylinder(&self, seed: Vec3, config: &FitConfig) -> Option<Fit> {
        let (seed_i, hood, _) = self.neighbourhood(seed)?;
        let seed_p = self.points[seed_i as usize].0;
        let cosn = config.normal_deg.to_radians().cos();
        let tol = config.tolerance_mm;
        // Candidate cylinders: RANSAC pairs + one from the normals' smallest eigenvector.
        let mut rng = Rng::new(0xC11_1D3A);
        let accept = |c: &Cyl, i: usize| {
            let (p, n) = self.points[i];
            (c.distance(p) - c.r).abs() <= tol && n.dot(c.radial(p)).abs() >= cosn
        };
        let mut best: Option<(Cyl, usize)> = None;
        for _ in 0..160 {
            let (i, j) = (hood[rng.below(hood.len())], hood[rng.below(hood.len())]);
            if i == j {
                continue;
            }
            let Some(c) = Cyl::from_pair(self.points[i], self.points[j]) else { continue };
            if !accept(&c, seed_i as usize) {
                continue;
            }
            let count = hood.iter().filter(|&&i| accept(&c, i)).count();
            if best.map_or(true, |(_, n)| count > n) {
                best = Some((c, count));
            }
        }
        // Axis from the normals' scatter + algebraic circle fit on the whole neighbourhood.
        let scatter = scatter3(hood.iter().map(|&i| self.points[i].1.as_dvec3()));
        let (_, vecs) = eigen3(scatter);
        let axis = vec3_of(&vecs, 0);
        if axis != Vec3::ZERO {
            if let Some((q, r)) = circle_fit_axis(&hood, &self.points, axis) {
                let c = Cyl { q, a: axis, r };
                if accept(&c, seed_i as usize) {
                    let count = hood.iter().filter(|&&i| accept(&c, i)).count();
                    if best.map_or(true, |(_, n)| count > n) {
                        best = Some((c, count));
                    }
                }
            }
        }
        let (mut cyl, _) = best?;
        // Inside or outside scan: the seed's normal picks one consistent radial sign.
        let sign = self.points[seed_i as usize].1.dot(cyl.radial(seed_p)).signum().max(0.0).mul_add(2.0, -1.0);
        let inliers = self.grow_cyl(seed_i as usize, &mut cyl, tol, cosn, sign, true);
        cyl = refit_cylinder(&inliers, &self.points, &cyl);
        let inliers = self.grow_cyl(seed_i as usize, &mut cyl, tol, cosn, sign, false);
        if inliers.len() < 30 {
            return None;
        }
        // Reject planes misread as huge cylinders.
        let (mut lo, mut hi) = (Vec3::splat(f32::MAX), Vec3::splat(f32::MIN));
        for &i in &inliers {
            lo = lo.min(self.points[i].0);
            hi = hi.max(self.points[i].0);
        }
        if cyl.r > 50.0 * (hi - lo).length() {
            return None;
        }
        // Coverage around the axis and extent along it.
        let (u, v) = ortho(cyl.a);
        let mut angles: Vec<f32> = inliers.iter().map(|&i| { let w = self.points[i].0 - cyl.q; (w.dot(v)).atan2(w.dot(u)) }).collect();
        angles.sort_by(f32::total_cmp);
        let mut gap = 0.0f32;
        for k in 0..angles.len() {
            let next = if k + 1 == angles.len() { angles[0] + std::f32::consts::TAU } else { angles[k + 1] };
            gap = gap.max(next - angles[k]);
        }
        let coverage_deg = (std::f32::consts::TAU - gap).to_degrees();
        if coverage_deg < 20.0 {
            return None;
        }
        let (mut t0, mut t1) = (f32::MAX, f32::MIN);
        for &i in &inliers {
            let t = (self.points[i].0 - cyl.q).dot(cyl.a);
            t0 = t0.min(t);
            t1 = t1.max(t);
        }
        let mut residuals: Vec<f32> = inliers.iter().map(|&i| (cyl.distance(self.points[i].0) - cyl.r).abs()).collect();
        residuals.sort_by(f32::total_cmp);
        let rms_mm = (residuals.iter().map(|r| r * r).sum::<f32>() / residuals.len() as f32).sqrt();
        let p95_mm = residuals[(residuals.len() * 95 / 100).min(residuals.len() - 1)];
        Some(Fit {
            primitive: Primitive::Cylinder { point: cyl.q + cyl.a * ((t0 + t1) / 2.0), axis: cyl.a, radius: cyl.r, length: t1 - t0 },
            rms_mm,
            p95_mm,
            inliers: inliers.into_iter().map(|i| i as u32).collect(),
            coverage_deg,
        })
    }

    /// Breadth-first growth from `start` over neighbours within 2.5 x spacing, accepting
    /// points on the cylinder with normals within `cosn` of the (signed) radial direction.
    /// With `refit`, the cylinder is re-fit by Gauss-Newton each time the region doubles.
    fn grow_cyl(&self, start: usize, cyl: &mut Cyl, tol: f32, cosn: f32, sign: f32, refit: bool) -> Vec<usize> {
        let near = 2.5 * self.spacing;
        let mut seen = vec![false; self.points.len()];
        let mut out = Vec::new();
        let mut queue = VecDeque::from([start]);
        let mut next_refit = 60usize;
        seen[start] = true;
        while let Some(i) = queue.pop_front() {
            let (p, n) = self.points[i];
            if (cyl.distance(p) - cyl.r).abs() > tol || sign * n.dot(cyl.radial(p)) < cosn {
                continue;
            }
            out.push(i);
            if refit && out.len() >= next_refit {
                *cyl = refit_cylinder(&out, &self.points, cyl);
                next_refit = out.len() * 2;
            }
            for j in self.index.within(p, near) {
                if !seen[j] {
                    seen[j] = true;
                    queue.push_back(j);
                }
            }
        }
        out
    }
}

/// A cylinder as an axis line (point `q`, unit direction `a`) and a radius `r`.
#[derive(Debug, Clone, Copy)]
struct Cyl {
    q: Vec3,
    a: Vec3,
    r: f32,
}

impl Cyl {
    /// Distance from `p` to the axis line.
    fn distance(&self, p: Vec3) -> f32 {
        let w = p - self.q;
        (w - self.a * w.dot(self.a)).length()
    }

    /// The unit radial direction from the axis towards `p`.
    fn radial(&self, p: Vec3) -> Vec3 {
        let w = p - self.q;
        (w - self.a * w.dot(self.a)).normalize_or_zero()
    }

    /// The cylinder through two oriented points: the axis direction is `n1 x n2` (both
    /// normals are square to it), the axis line where the two normals' lines meet.
    fn from_pair((p1, n1): Oriented, (p2, n2): Oriented) -> Option<Cyl> {
        let a = n1.cross(n2);
        if a.length() < 0.15 {
            return None;
        }
        let a = a.normalize();
        // Closest points of the two normal lines give the axis point.
        let d = p2 - p1;
        let c = n1.dot(n2).clamp(-0.99, 0.99);
        let (b1, b2) = (d.dot(n1), d.dot(n2));
        let det = 1.0 - c * c;
        if det < 1e-6 {
            return None;
        }
        let (t1, t2) = ((b1 - c * b2) / det, (c * b1 - b2) / det);
        let q = (p1 + n1 * t1 + p2 + n2 * t2) / 2.0;
        let cyl = Cyl { q, a, r: 0.0 };
        let r = (cyl.distance(p1) + cyl.distance(p2)) / 2.0;
        (r > 1e-3).then_some(Cyl { r, ..cyl })
    }
}

/// Two unit vectors square to `d` (a basis of the axis's cross plane).
fn ortho(d: Vec3) -> (Vec3, Vec3) {
    let u = if d.x.abs() < d.y.abs() { d.cross(Vec3::X) } else { d.cross(Vec3::Y) }.normalize();
    (u, d.cross(u).normalize())
}

/// The 3x3 scatter matrix sum(v v^T) over `vs`, f64.
fn scatter3(vs: impl Iterator<Item = glam::DVec3>) -> [[f64; 3]; 3] {
    let mut s = [[0.0f64; 3]; 3];
    for v in vs {
        for i in 0..3 {
            for j in 0..3 {
                s[i][j] += v[i] * v[j];
            }
        }
    }
    s
}

/// The 3x3 covariance of `ids`' positions, f64, and their centroid.
fn covariance(ids: &[usize], points: &[Oriented]) -> ([[f64; 3]; 3], Vec3) {
    let c = ids.iter().map(|&i| points[i].0).sum::<Vec3>() / ids.len().max(1) as f32;
    (scatter3(ids.iter().map(|&i| (points[i].0 - c).as_dvec3())), c)
}

/// Least-squares plane through `ids`: PCA of the inliers, the normal is the smallest
/// eigenvector of the covariance, oriented to agree with `keep`'s direction.
fn pca_plane(ids: &[usize], points: &[Oriented], keep: Vec3) -> Plane {
    let (cov, c) = covariance(ids, points);
    let (_, vecs) = eigen3(cov);
    let mut n = vec3_of(&vecs, 0);
    if n == Vec3::ZERO {
        n = keep;
    }
    if n.dot(keep) < 0.0 {
        n = -n;
    }
    Plane::through(c, n)
}

/// Algebraic circle fit (Kasa) of `ids`' positions projected square to `axis`: (axis point,
/// radius). None for a degenerate set.
fn circle_fit_axis(ids: &[usize], points: &[Oriented], axis: Vec3) -> Option<(Vec3, f32)> {
    if ids.len() < 3 {
        return None;
    }
    let (u, v) = ortho(axis);
    // Least squares of x^2+y^2 + D x + E y + F = 0.
    let mut ata = [[0.0f64; 3]; 3];
    let mut atb = [0.0f64; 3];
    for &i in ids {
        let w = (points[i].0 - points[ids[0]].0).as_dvec3();
        let (x, y) = (w.dot(u.as_dvec3()), w.dot(v.as_dvec3()));
        let row = [x, y, 1.0];
        for a in 0..3 {
            atb[a] -= row[a] * (x * x + y * y);
            for b in 0..3 {
                ata[a][b] += row[a] * row[b];
            }
        }
    }
    let sol = solve3(ata, atb)?;
    let (cx, cy) = (-sol[0] / 2.0, -sol[1] / 2.0);
    let r2 = cx * cx + cy * cy - sol[2];
    if r2 <= 0.0 {
        return None;
    }
    Some((points[ids[0]].0 + u * cx as f32 + v * cy as f32, r2.sqrt() as f32))
}

/// 3x3 linear solve by Gaussian elimination with partial pivoting. None when singular.
fn solve3(mut a: [[f64; 3]; 3], mut b: [f64; 3]) -> Option<[f64; 3]> {
    for col in 0..3 {
        let piv = (col..3).max_by(|&i, &j| a[i][col].abs().total_cmp(&a[j][col].abs()))?;
        if a[piv][col].abs() < 1e-14 {
            return None;
        }
        a.swap(col, piv);
        b.swap(col, piv);
        for row in (col + 1)..3 {
            let f = a[row][col] / a[col][col];
            for k in col..3 {
                a[row][k] -= f * a[col][k];
            }
            b[row] -= f * b[col];
        }
    }
    let mut x = [0.0; 3];
    for i in (0..3).rev() {
        x[i] = (b[i] - (0..3).map(|k| if k > i { a[i][k] * x[k] } else { 0.0 }).sum::<f64>()) / a[i][i];
    }
    Some(x)
}

/// 5x5 linear solve by Gaussian elimination with partial pivoting. None when singular.
fn solve5(mut a: [[f64; 5]; 5], mut b: [f64; 5]) -> Option<[f64; 5]> {
    for col in 0..5 {
        let piv = (col..5).max_by(|&i, &j| a[i][col].abs().total_cmp(&a[j][col].abs()))?;
        if a[piv][col].abs() < 1e-14 {
            return None;
        }
        a.swap(col, piv);
        b.swap(col, piv);
        for row in (col + 1)..5 {
            let f = a[row][col] / a[col][col];
            for k in col..5 {
                a[row][k] -= f * a[col][k];
            }
            b[row] -= f * b[col];
        }
    }
    let mut x = [0.0; 5];
    for i in (0..5).rev() {
        x[i] = (b[i] - (0..5).map(|k| if k > i { a[i][k] * x[k] } else { 0.0 }).sum::<f64>()) / a[i][i];
    }
    Some(x)
}

/// Gauss-Newton least-squares refit of a cylinder to `ids`: five parameters — the axis
/// point's two coordinates square to the axis, the axis direction's two tilt coordinates,
/// and the radius — iterated on the true residual (distance to the axis minus radius), f64.
fn refit_cylinder(ids: &[usize], points: &[Oriented], start: &Cyl) -> Cyl {
    let (u, v) = ortho(start.a);
    let (u, v) = (u.as_dvec3(), v.as_dvec3());
    let a0 = start.a.as_dvec3();
    let q0 = start.q.as_dvec3();
    let mut th = [0.0f64, 0.0, 0.0, 0.0, start.r as f64];
    let eval = |th: &[f64; 5], i: usize| -> f64 {
        let q = q0 + u * th[0] + v * th[1];
        let dir = (a0 + u * th[2] + v * th[3]).normalize();
        let w = points[i].0.as_dvec3() - q;
        let wp = w - dir * w.dot(dir);
        wp.length() - th[4]
    };
    for _ in 0..8 {
        // Normal equations J^T J d = -J^T f with a numeric Jacobian.
        let mut jj = [[0.0f64; 5]; 5];
        let mut jf = [0.0f64; 5];
        for &i in ids {
            let f0 = eval(&th, i);
            let mut row = [0.0f64; 5];
            for k in 0..5 {
                let h = if k < 4 { 1e-5 } else { 1e-5 };
                let mut tp = th;
                tp[k] += h;
                row[k] = (eval(&tp, i) - f0) / h;
            }
            for a in 0..5 {
                jf[a] += row[a] * f0;
                for b in 0..5 {
                    jj[a][b] += row[a] * row[b];
                }
            }
        }
        let Some(d) = solve5(jj, jf.map(|x| -x)) else { break };
        for k in 0..5 {
            th[k] += d[k];
        }
        if d.iter().map(|x| x.abs()).fold(0.0, f64::max) < 1e-7 {
            break;
        }
    }
    let q = (q0 + u * th[0] + v * th[1]).as_vec3();
    let a = (a0 + u * th[2] + v * th[3]).normalize().as_vec3();
    Cyl { q, a, r: th[4].max(1e-3) as f32 }
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::Quat;

    /// A tilted box (half extents `half`) sampled on all faces about `spacing` apart, noise
    /// `sigma` along each face's normal.
    fn box_cloud(half: Vec3, spacing: f32, rot: Quat, sigma: f32, seed: u64) -> Vec<Oriented> {
        let mut rng = Rng::new(seed);
        let mut out = Vec::new();
        for axis in 0..3 {
            for sign in [-1.0f32, 1.0] {
                let (a, b) = ((axis + 1) % 3, (axis + 2) % 3);
                let (na, nb) = ((2.0 * half[a] / spacing).ceil() as i32, (2.0 * half[b] / spacing).ceil() as i32);
                for i in 0..=na {
                    for j in 0..=nb {
                        let mut p = Vec3::ZERO;
                        p[axis] = sign * half[axis];
                        p[a] = -half[a] + 2.0 * half[a] * i as f32 / na as f32;
                        p[b] = -half[b] + 2.0 * half[b] * j as f32 / nb as f32;
                        let mut n = Vec3::ZERO;
                        n[axis] = sign;
                        out.push((rot * (p + n * rng.normal() * sigma), rot * n));
                    }
                }
            }
        }
        out
    }

    /// An open cylinder along `rot`'s +Z, radius `r`, length `len` around the origin,
    /// normals outward (`inside = false`) or to the axis (a bore), `arc` of it scanned.
    fn cylinder_cloud(r: f32, len: f32, spacing: f32, rot: Quat, sigma: f32, inside: bool, arc_deg: f32, seed: u64) -> Vec<Oriented> {
        let mut rng = Rng::new(seed);
        let mut out = Vec::new();
        let around = (std::f32::consts::TAU * r / spacing * arc_deg / 360.0).ceil() as usize;
        let along = (len / spacing).ceil() as usize;
        for i in 0..=around {
            let a = arc_deg.to_radians() * i as f32 / around as f32;
            let mut n = Vec3::new(a.cos(), a.sin(), 0.0);
            if inside {
                n = -n;
            }
            for j in 0..=along {
                let p = Vec3::new(r * a.cos(), r * a.sin(), -len / 2.0 + len * j as f32 / along as f32);
                out.push((rot * (p - n * rng.normal() * sigma), rot * n));
            }
        }
        out
    }

    /// A flat plate in z = 0, `size` square, sampled about `spacing`.
    fn plate_cloud(size: f32, spacing: f32, rot: Quat, at: Vec3, sigma: f32, seed: u64) -> Vec<Oriented> {
        let mut rng = Rng::new(seed);
        let mut out = Vec::new();
        let n = (size / spacing).ceil() as i32;
        for i in 0..=n {
            for j in 0..=n {
                let p = Vec3::new(-size / 2.0 + size * i as f32 / n as f32, -size / 2.0 + size * j as f32 / n as f32, 0.0);
                out.push((rot * p + at + rot * Vec3::Z * rng.normal() * sigma, rot * Vec3::Z));
            }
        }
        out
    }

    /// Which face of `half`-box the point is on: (axis, sign); None if off all faces.
    fn face_of(p: Vec3, half: Vec3, rot: Quat) -> Option<(usize, f32)> {
        let q = rot.inverse() * p;
        let mut best: Option<(usize, f32, f32)> = None;
        for axis in 0..3 {
            for sign in [-1.0f32, 1.0] {
                let d = (q[axis] - sign * half[axis]).abs();
                if d < 0.35 && q[(axis + 1) % 3].abs() <= half[(axis + 1) % 3] + 0.4 && q[(axis + 2) % 3].abs() <= half[(axis + 2) % 3] + 0.4 {
                    if best.map_or(true, |b| d < b.2) {
                        best = Some((axis, sign, d));
                    }
                }
            }
        }
        best.map(|(a, s, _)| (a, s))
    }

    #[test]
    fn plane_on_a_box_face_grows_to_the_face_edges() {
        let half = Vec3::new(40.0, 25.0, 15.0);
        let rot = Quat::from_rotation_y(0.4) * Quat::from_rotation_z(0.7) * Quat::from_rotation_x(-0.25);
        let (sigma, spacing) = (0.06f32, 0.7f32);
        let points = box_cloud(half, spacing, rot, sigma, 5);
        let surface = Surface::new(points.clone());
        // Click the middle of the biggest face (+z local: 80 x 50 mm).
        let click = rot * Vec3::new(5.0, -8.0, half.z);
        let fit = surface.fit_plane(click, &FitConfig::default()).expect("a plane");
        let Primitive::Plane { normal, .. } = fit.primitive else { panic!("not a plane") };
        let expected: usize = points.iter().filter(|(p, _)| face_of(*p, half, rot) == Some((2, 1.0))).count();
        // Foreign = an inlier clearly off the clicked face's plane (beyond the tolerance
        // band the edge points of the neighbouring faces live in).
        let foreign = fit
            .inliers
            .iter()
            .filter(|&&i| {
                let q = rot.inverse() * points[i as usize].0;
                (q.z - half.z).abs() > 0.5
            })
            .count();
        let angle = normal.angle_between(rot * Vec3::Z).to_degrees();
        eprintln!(
            "box plane: {} inliers of {} on the face ({} foreign), normal {angle:.3} deg off, rms {:.3} mm, p95 {:.3} mm",
            fit.inliers.len(), expected, foreign, fit.rms_mm, fit.p95_mm
        );
        assert!((fit.inliers.len() as f32 - expected as f32).abs() < 0.05 * expected as f32, "inliers {} vs {}", fit.inliers.len(), expected);
        assert_eq!(foreign, 0, "{foreign} inliers off the clicked face");
        assert!(angle < 0.3 && fit.rms_mm < 2.0 * sigma, "angle {angle}, rms {}", fit.rms_mm);
        // A click 1.5 mm inside the edge still takes that face, not the neighbour.
        let click = rot * Vec3::new(half.x - 1.5, -3.0, half.z);
        let fit = surface.fit_plane(click, &FitConfig::default()).expect("a plane");
        let foreign = fit
            .inliers
            .iter()
            .filter(|&&i| {
                let q = rot.inverse() * points[i as usize].0;
                (q.z - half.z).abs() > 0.5
            })
            .count();
        eprintln!("edge click: {} inliers, {foreign} foreign", fit.inliers.len());
        assert_eq!(foreign, 0);
        assert!((fit.inliers.len() as f32 - expected as f32).abs() < 0.05 * expected as f32);
    }

    #[test]
    fn cylinder_on_a_plate_stops_at_the_plate() {
        let rot = Quat::from_rotation_y(0.35) * Quat::from_rotation_z(-0.5);
        let (r, len, sigma) = (12.0f32, 40.0f32, 0.05f32);
        let mut points = cylinder_cloud(r, len, 0.5, rot, sigma, false, 360.0, 7);
        let cyl_count = points.len();
        // Standing on a plate the growth must not take.
        points.extend(plate_cloud(80.0, 0.6, Quat::IDENTITY, rot * Vec3::new(0.0, 0.0, -len / 2.0), sigma, 8));
        let surface = Surface::new(points);
        let click = rot * Vec3::new(r * 0.4f32.cos(), r * 0.4f32.sin(), 5.0);
        let fit = surface.fit_cylinder(click, &FitConfig::default()).expect("a cylinder");
        let Primitive::Cylinder { axis, radius, length, .. } = fit.primitive else { panic!("not a cylinder") };
        let angle = axis.angle_between(rot * Vec3::Z).min(axis.angle_between(rot * -Vec3::Z)).to_degrees();
        let on_plate = fit.inliers.iter().filter(|&&i| i as usize >= cyl_count).count();
        eprintln!(
            "cylinder: {} inliers ({on_plate} on the plate), axis {angle:.3} deg off, radius {radius:.3} mm (r {r}), length {length:.1} mm, coverage {:.0} deg, rms {:.3} mm",
            fit.inliers.len(), fit.coverage_deg, fit.rms_mm
        );
        assert!(angle < 0.5 && (radius - r).abs() < 0.05, "axis {angle} deg, radius {radius}");
        assert!(fit.rms_mm < 2.0 * sigma && fit.coverage_deg > 300.0);
        assert_eq!(on_plate, 0, "{on_plate} plate points accepted");
        // Half of it scanned: 180 deg of arc.
        let half = Surface::new(cylinder_cloud(r, len, 0.5, rot, sigma, false, 180.0, 9));
        let fit = half.fit_cylinder(click, &FitConfig::default()).expect("a cylinder");
        let Primitive::Cylinder { radius, .. } = fit.primitive else { panic!("not a cylinder") };
        eprintln!("half cylinder: radius {radius:.3} mm, coverage {:.0} deg, rms {:.3} mm", fit.coverage_deg, fit.rms_mm);
        assert!((radius - r).abs() < 0.15, "radius {radius}");
    }

    #[test]
    fn an_inside_scanned_bore_fits_the_same() {
        let rot = Quat::from_rotation_x(0.6) * Quat::from_rotation_z(0.3);
        let (r, len, sigma) = (15.0f32, 30.0f32, 0.05f32);
        let surface = Surface::new(cylinder_cloud(r, len, 0.6, rot, sigma, true, 360.0, 11));
        let click = rot * Vec3::new(-r, 0.0, 3.0);
        let fit = surface.fit_cylinder(click, &FitConfig::default()).expect("a cylinder");
        let Primitive::Cylinder { axis, radius, .. } = fit.primitive else { panic!("not a cylinder") };
        let angle = axis.angle_between(rot * Vec3::Z).min(axis.angle_between(rot * -Vec3::Z)).to_degrees();
        eprintln!("bore: radius {radius:.3} mm (r {r}), axis {angle:.3} deg, coverage {:.0} deg, rms {:.3} mm", fit.coverage_deg, fit.rms_mm);
        assert!((radius - r).abs() < 0.1 && angle < 0.5 && fit.coverage_deg > 300.0);
    }

    #[test]
    fn wrong_primitives_are_rejected() {
        // A flat face is not a cylinder.
        let half = Vec3::new(40.0, 25.0, 15.0);
        let rot = Quat::from_rotation_y(0.3);
        let surface = Surface::new(box_cloud(half, 0.7, rot, 0.05, 13));
        let click = rot * Vec3::new(0.0, 0.0, half.z);
        assert!(surface.fit_cylinder(click, &FitConfig::default()).is_none());
        // A dome is not a plane at a tight tolerance: None or a small patch (< 10 %).
        let (outside, _) = crate::synthetic::shell_part(0.8, 3.0);
        let dome_count = outside.len();
        let surface = Surface::new(outside);
        let click = surface.points()[dome_count / 3].0;
        let config = FitConfig { tolerance_mm: 0.1, normal_deg: 25.0 };
        match surface.fit_plane(click, &config) {
            None => eprintln!("dome plane: None"),
            Some(f) => {
                eprintln!("dome plane: {} inliers of {} dome points", f.inliers.len(), dome_count);
                assert!(f.inliers.len() < dome_count / 10, "took {} of {}", f.inliers.len(), dome_count);
            }
        }
    }

    #[test]
    fn a_large_surface_indexes_and_fits_fast() {
        let t0 = std::time::Instant::now();
        // ~200k points: a plate with a cylinder boss on it; the fit takes the boss's region.
        let mut points = plate_cloud(250.0, 0.6, Quat::IDENTITY, Vec3::ZERO, 0.04, 17);
        points.extend(cylinder_cloud(15.0, 45.0, 0.4, Quat::IDENTITY, 0.04, false, 360.0, 18));
        let surface = Surface::new(points);
        let built = t0.elapsed();
        let fit = surface.fit_cylinder(Vec3::new(15.0, 0.0, 10.0), &FitConfig::default()).expect("a cylinder");
        eprintln!("{} points: Surface::new {built:?}, fit {:?} -> {} inliers, rms {:.3} mm", surface.points().len(), t0.elapsed() - built, fit.inliers.len(), fit.rms_mm);
        assert!(t0.elapsed().as_secs_f32() < 1.0, "took {:?}", t0.elapsed());
    }
}
