//! The laser light planes, in the rectified left optical frame. They tell the correct left/right
//! line pairing apart from the ~10 geometrically possible ones: only the correct pairing
//! triangulates onto one of the projector's planes.
//!
//! The planes are rigid with respect to the scanner, so they can be calibrated from ordinary scan
//! frames (moving or static): every pairing of a curved line stretch defines a plane hypothesis,
//! and a true laser plane is the one many different line stretches agree on (a laser line hits
//! several surfaces). Planes through the camera centre are rejected: every pairing of one left
//! line lies on the plane through the camera centre and that image line.

use crate::stereo::Candidate;
use glam::Vec3;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct LaserPlane {
    /// Unit normal, optical frame.
    pub normal: [f32; 3],
    /// Plane: `normal . p = offset` (mm).
    pub offset: f32,
    /// Cross-line family: sign of the rectified image slope `du/dv` (+1 or -1).
    pub family: i8,
    /// Calibration points that supported the plane.
    pub support: u32,
}

impl LaserPlane {
    pub fn distance(&self, p: Vec3) -> f32 {
        Vec3::from(self.normal).dot(p) - self.offset
    }
}

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct LaserPlaneSet {
    pub planes: Vec<LaserPlane>,
}

/// Least-squares plane through points: (unit normal, offset, rms distance, in-plane spread
/// perpendicular to the main direction).
pub fn fit_plane(points: &[Vec3]) -> Option<(Vec3, f32, f32, f32)> {
    if points.len() < 3 {
        return None;
    }
    let n = points.len() as f32;
    let c = points.iter().copied().sum::<Vec3>() / n;
    let mut m = [[0f32; 3]; 3];
    for p in points {
        let d = *p - c;
        let a = d.to_array();
        for i in 0..3 {
            for j in 0..3 {
                m[i][j] += a[i] * a[j];
            }
        }
    }
    let (vals, vecs) = eigen_sym3(m);
    // Ascending eigenvalues: smallest -> normal.
    let normal = vecs[0].normalize_or_zero();
    if normal == Vec3::ZERO {
        return None;
    }
    Some((normal, normal.dot(c), (vals[0].max(0.0) / n).sqrt(), (vals[1].max(0.0) / n).sqrt()))
}

/// Eigen-decomposition of a symmetric 3x3 matrix (cyclic Jacobi), ascending eigenvalues.
pub fn eigen_sym3(mut a: [[f32; 3]; 3]) -> ([f32; 3], [Vec3; 3]) {
    let mut v = [[1f32, 0., 0.], [0., 1., 0.], [0., 0., 1.]];
    for _ in 0..24 {
        let off = a[0][1].abs() + a[0][2].abs() + a[1][2].abs();
        if off < 1e-12 * (a[0][0].abs() + a[1][1].abs() + a[2][2].abs() + 1e-30) {
            break;
        }
        for (p, q) in [(0usize, 1usize), (0, 2), (1, 2)] {
            if a[p][q].abs() < 1e-30 {
                continue;
            }
            let theta = (a[q][q] - a[p][p]) / (2.0 * a[p][q]);
            let t = theta.signum() / (theta.abs() + (theta * theta + 1.0).sqrt());
            let t = if theta == 0.0 { 1.0 } else { t };
            let c = 1.0 / (t * t + 1.0).sqrt();
            let s = t * c;
            for row in a.iter_mut() {
                let (akp, akq) = (row[p], row[q]);
                row[p] = c * akp - s * akq;
                row[q] = s * akp + c * akq;
            }
            let (rp, rq) = (a[p], a[q]);
            for k in 0..3 {
                a[p][k] = c * rp[k] - s * rq[k];
                a[q][k] = s * rp[k] + c * rq[k];
            }
            for row in v.iter_mut() {
                let (vkp, vkq) = (row[p], row[q]);
                row[p] = c * vkp - s * vkq;
                row[q] = s * vkp + c * vkq;
            }
        }
    }
    let mut idx = [0usize, 1, 2];
    idx.sort_by(|&i, &j| a[i][i].total_cmp(&a[j][j]));
    let vals = idx.map(|i| a[i][i]);
    let vecs = idx.map(|i| Vec3::new(v[0][i], v[1][i], v[2][i]));
    (vals, vecs)
}

/// Pairing candidates of one frame, with the family of each candidate's left chain.
pub struct FrameCandidates {
    pub candidates: Vec<Candidate>,
    pub families: Vec<i8>,
}

#[derive(Debug, Clone, Copy)]
pub struct CalibrationParams {
    /// A candidate's points must lie within this distance of a plane to support it (mm, 90th pct).
    pub tolerance_mm: f32,
    /// Hypotheses only from candidates whose points spread at least this much off a straight line (mm).
    pub min_spread_mm: f32,
    /// Planes closer than this to the camera centre are the degenerate back-projection planes.
    pub min_offset_mm: f32,
    /// Distinct supporting line stretches a plane needs.
    pub min_support: usize,
}

impl Default for CalibrationParams {
    fn default() -> Self {
        Self { tolerance_mm: 0.5, min_spread_mm: 0.15, min_offset_mm: 15.0, min_support: 4 }
    }
}

fn percentile_distance(points: &[Vec3], normal: Vec3, offset: f32, pct: f32) -> f32 {
    let mut d: Vec<f32> = points.iter().map(|p| (normal.dot(*p) - offset).abs()).collect();
    let k = ((d.len() as f32 - 1.0) * pct) as usize;
    *d.select_nth_unstable_by(k, f32::total_cmp).1
}

impl LaserPlaneSet {
    /// Greedy RANSAC over all candidates of all frames (see module docs).
    pub fn calibrate(frames: &[FrameCandidates], params: &CalibrationParams) -> Self {
        // Flatten: (frame, left chain) identifies a line stretch.
        struct Item<'a> {
            stretch: (usize, usize),
            family: i8,
            points: &'a [Vec3],
        }
        let mut items = Vec::new();
        for (fi, f) in frames.iter().enumerate() {
            for (c, &fam) in f.candidates.iter().zip(&f.families) {
                items.push(Item { stretch: (fi, c.left), family: fam, points: &c.points });
            }
        }
        let mut hypotheses = Vec::new();
        for it in &items {
            if let Some((n, d, rms, spread)) = fit_plane(it.points) {
                if spread >= params.min_spread_mm && rms < params.tolerance_mm * 0.5 && d.abs() >= params.min_offset_mm {
                    hypotheses.push((n, d, it.family));
                }
            }
        }
        let mut used = std::collections::HashSet::new();
        let mut planes = Vec::new();
        loop {
            let mut best: Option<(usize, Vec3, f32, i8)> = None;
            for &(n, d, fam) in &hypotheses {
                let mut stretches = std::collections::HashSet::new();
                for it in &items {
                    if it.family != fam || used.contains(&it.stretch) || stretches.contains(&it.stretch) {
                        continue;
                    }
                    if percentile_distance(it.points, n, d, 0.9) < params.tolerance_mm {
                        stretches.insert(it.stretch);
                    }
                }
                if best.as_ref().is_none_or(|b| stretches.len() > b.0) {
                    best = Some((stretches.len(), n, d, fam));
                }
            }
            let Some((count, n, d, fam)) = best else { break };
            if count < params.min_support {
                break;
            }
            // Refit on the supporting points, then claim those stretches.
            let mut pts = Vec::new();
            let mut claimed = Vec::new();
            for it in &items {
                if it.family == fam && !used.contains(&it.stretch) && percentile_distance(it.points, n, d, 0.9) < params.tolerance_mm {
                    pts.extend_from_slice(it.points);
                    claimed.push(it.stretch);
                }
            }
            let (n2, d2, _, _) = fit_plane(&pts).unwrap_or((n, d, 0.0, 0.0));
            let (n2, d2) = if n2.dot(n) < 0.0 { (-n2, -d2) } else { (n2, d2) };
            used.extend(claimed);
            planes.push(LaserPlane { normal: n2.to_array(), offset: d2, family: fam, support: count as u32 });
            hypotheses.retain(|&(hn, hd, hf)| hf != fam || !(hn.dot(n2).abs() > 0.9995 && (hd * hn.dot(n2).signum() - d2).abs() < 1.0));
        }
        Self { planes }
    }

    /// Single-line mode: exactly one light plane. RANSAC over pairs of candidates (any shape,
    /// from any frame): the plane through both that most candidates agree with. Straight stretches
    /// at different depths constrain it jointly, so no curved surface is needed.
    pub fn calibrate_single(frames: &[FrameCandidates], params: &CalibrationParams) -> Self {
        let items: Vec<&[Vec3]> = frames.iter().flat_map(|f| f.candidates.iter().map(|c| c.points.as_slice())).filter(|p| p.len() >= 6).collect();
        if items.len() < params.min_support {
            return Self::default();
        }
        let mut seed = 0x9e3779b97f4a7c15u64;
        let mut rnd = |n: usize| {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            (seed % n as u64) as usize
        };
        let mut best: Option<(usize, Vec3, f32)> = None;
        for _ in 0..3000 {
            let (a, b) = (items[rnd(items.len())], items[rnd(items.len())]);
            let (p0, p1, p2) = (a[0], a[a.len() - 1], b[b.len() / 2]);
            let n = (p1 - p0).cross(p2 - p0).normalize_or_zero();
            if n == Vec3::ZERO {
                continue;
            }
            let d = n.dot(p0);
            if d.abs() < params.min_offset_mm {
                continue;
            }
            let count = items.iter().filter(|it| percentile_distance(it, n, d, 0.9) < params.tolerance_mm * 2.0).count();
            if best.is_none_or(|b| count > b.0) {
                best = Some((count, n, d));
            }
        }
        let Some((count, n, d)) = best.filter(|b| b.0 >= params.min_support) else { return Self::default() };
        // Refine twice on the inlier points with a tightening band.
        let (mut n, mut d) = (n, d);
        for tol in [params.tolerance_mm * 2.0, params.tolerance_mm] {
            let pts: Vec<Vec3> = items.iter().filter(|it| percentile_distance(it, n, d, 0.9) < tol).flat_map(|it| it.iter().copied()).collect();
            if let Some((n2, d2, _, _)) = fit_plane(&pts) {
                (n, d) = if n2.dot(n) < 0.0 { (-n2, -d2) } else { (n2, d2) };
            }
        }
        Self { planes: vec![LaserPlane { normal: n.to_array(), offset: d, family: 0, support: count as u32 }] }
    }

    /// Best plane for a candidate: (plane index, fraction of points within `tol_mm`, rms).
    /// Planes of family 0 (single line) match candidates of either slope.
    pub fn best_fit(&self, family: i8, points: &[Vec3], tol_mm: f32) -> Option<(usize, f32, f32)> {
        let mut best: Option<(usize, f32, f32)> = None;
        for (k, pl) in self.planes.iter().enumerate() {
            if pl.family != family && pl.family != 0 {
                continue;
            }
            let mut inl = 0usize;
            let mut sq = 0f32;
            for p in points {
                let d = pl.distance(*p);
                if d.abs() < tol_mm {
                    inl += 1;
                }
                sq += d * d;
            }
            let frac = inl as f32 / points.len().max(1) as f32;
            let rms = (sq / points.len().max(1) as f32).sqrt();
            if best.is_none_or(|b| frac > b.1 || (frac == b.1 && rms < b.2)) {
                best = Some((k, frac, rms));
            }
        }
        best
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fits_plane_and_eigenvectors() {
        let n = Vec3::new(0.86, 0.48, 0.17).normalize();
        let (a, b) = (n.any_orthonormal_pair().0, n.any_orthonormal_pair().1);
        let pts: Vec<Vec3> = (0..50).map(|i| n * 40.0 + a * (i as f32) + b * ((i * i) as f32 * 0.05)).collect();
        let (fit_n, d, rms, spread) = fit_plane(&pts).unwrap();
        assert!(fit_n.dot(n).abs() > 0.9999);
        assert!((d.abs() - 40.0).abs() < 1e-2);
        assert!(rms < 1e-2 && spread > 1.0, "rms {rms} spread {spread}");
    }
}
