//! Rectified stereo triangulation and left/right chain pairing candidates.

use crate::chains::Chain;
use glam::Vec3;
use scancraft_core::StereoRig;

/// Rectified pair geometry (full precision copy of what triangulation needs).
#[derive(Debug, Clone, Copy)]
pub struct Rectified {
    pub f: f32,
    pub cx_left: f32,
    pub cx_right: f32,
    pub cy: f32,
    pub baseline_mm: f32,
}

impl Rectified {
    pub fn from_rig(rig: &StereoRig) -> Self {
        let k = rig.intrinsics;
        let cx_right = rig.rectification.as_ref().map(|r| r.right_cx as f32).unwrap_or(k.cx);
        Self { f: k.fx, cx_left: k.cx, cx_right, cy: k.cy, baseline_mm: rig.baseline_mm }
    }

    /// Point in the rectified left optical frame (x right, y down, z forward), mm.
    pub fn triangulate(&self, u_left: f32, v: f32, u_right: f32) -> Option<Vec3> {
        let disparity = u_left - u_right - (self.cx_left - self.cx_right);
        if disparity <= 1.0 {
            return None;
        }
        let z = self.f * self.baseline_mm / disparity;
        Some(Vec3::new((u_left - self.cx_left) * z / self.f, (v - self.cy) * z / self.f, z))
    }

    /// Right-image column of a point at depth `z` seen at `u_left`.
    pub fn u_right_at_depth(&self, u_left: f32, z: f32) -> f32 {
        u_left - self.f * self.baseline_mm / z - (self.cx_left - self.cx_right)
    }
}

/// One way of pairing a left chain with a right chain.
#[derive(Debug, Clone)]
pub struct Candidate {
    pub left: usize,
    pub right: usize,
    /// Triangulated samples (optical frame), one per left sample in the overlap.
    pub points: Vec<Vec3>,
    /// Rectified left (u, v) of each point.
    pub pixels: Vec<(f32, f32)>,
}

/// All pairings of each left chain with right chains of the same family whose overlap is at
/// least `min_overlap` samples and whose median depth lies in `[z_min, z_max]` mm.
/// `same_family`: only pair chains whose image slopes have the same sign (cross lines).
pub fn candidates(left: &[Chain], right: &[Chain], geo: &Rectified, z_min: f32, z_max: f32, min_overlap: usize, same_family: bool) -> Vec<Candidate> {
    let mut out = Vec::new();
    for (i, lc) in left.iter().enumerate() {
        let (l0, l1) = lc.v_range();
        for (j, rc) in right.iter().enumerate() {
            if same_family && (lc.slope > 0.0) != (rc.slope > 0.0) {
                continue;
            }
            let (r0, r1) = rc.v_range();
            if r1 < l0 || r0 > l1 || (r1.min(l1) - r0.max(l0)) < min_overlap as f32 * 0.5 {
                continue;
            }
            // Quick depth check at the middle of the overlap before triangulating everything.
            let vm = 0.5 * (r0.max(l0) + r1.min(l1));
            let (Some(ul), Some(ur)) = (lc.u_at(vm, 6.0), rc.u_at(vm, 6.0)) else { continue };
            match geo.triangulate(ul, vm, ur) {
                Some(p) if p.z >= z_min * 0.8 && p.z <= z_max * 1.2 => {}
                _ => continue,
            }
            let mut points = Vec::new();
            let mut pixels = Vec::new();
            for &(u, v) in &lc.points {
                let Some(ur) = rc.u_at(v, 3.0) else { continue };
                if let Some(p) = geo.triangulate(u, v, ur) {
                    points.push(p);
                    pixels.push((u, v));
                }
            }
            if points.len() < min_overlap {
                continue;
            }
            let mut zs: Vec<f32> = points.iter().map(|p| p.z).collect();
            let mid = zs.len() / 2;
            let z_med = *zs.select_nth_unstable_by(mid, f32::total_cmp).1;
            if z_med < z_min || z_med > z_max {
                continue;
            }
            out.push(Candidate { left: i, right: j, points, pixels });
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn triangulation_matches_rig_geometry() {
        let geo = Rectified { f: 1482.67, cx_left: 340.44, cx_right: 950.01, cy: 485.67, baseline_mm: 118.63 };
        let p = Vec3::new(20.0, -15.0, 275.0);
        let ul = geo.f * p.x / p.z + geo.cx_left;
        let v = geo.f * p.y / p.z + geo.cy;
        let ur = geo.f * (p.x - geo.baseline_mm) / p.z + geo.cx_right;
        let q = geo.triangulate(ul, v, ur).unwrap();
        assert!((q - p).length() < 1e-3, "{q:?}");
        assert!((geo.u_right_at_depth(ul, 275.0) - ur).abs() < 1e-3);
    }
}
