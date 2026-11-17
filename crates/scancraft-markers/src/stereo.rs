//! Left/right matching of marker dots along epipolar lines and triangulation.
//!
//! Works on a *rectified* pair (row-aligned epipolar lines). The two rectified cameras may have
//! different principal points: on the POP 4 the rectified right image keeps `pR`'s cx, 304.9 px
//! (at 640x512) right of the left one, so disparity is `(uL - cxL) - (uR - cxR)`, not `uL - uR`
//! (`docs/protocol/markers.md`, measured against on-device depth). [`StereoGeometry`] carries both.

use crate::detect::Marker2D;
use glam::{Vec2, Vec3};
use scancraft_core::{DepthMap, IrImages, StereoRig};
use serde::{Deserialize, Serialize};

/// Rectified stereo pair geometry at the image resolution the markers were detected in.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct StereoGeometry {
    pub fx: f32,
    pub fy: f32,
    pub cx_left: f32,
    pub cx_right: f32,
    pub cy: f32,
    pub baseline_mm: f32,
}

impl StereoGeometry {
    /// From a [`StereoRig`] plus the right image's principal-point offset (`cx_right - cx_left`,
    /// px at the rig resolution; 0 if both rectified images share the left intrinsics).
    pub fn from_rig(rig: &StereoRig, right_cx_offset: f32) -> Self {
        let k = rig.intrinsics;
        Self { fx: k.fx, fy: k.fy, cx_left: k.cx, cx_right: k.cx + right_cx_offset, cy: k.cy, baseline_mm: rig.baseline_mm }
    }

    pub fn disparity(&self, left: Vec2, right: Vec2) -> f32 {
        (left.x - self.cx_left) - (right.x - self.cx_right)
    }

    /// Optical-frame point (x right, y down, z forward) from a left pixel and a disparity.
    fn optical(&self, left: Vec2, disparity: f32) -> Vec3 {
        let z = self.fx * self.baseline_mm / disparity;
        Vec3::new((left.x - self.cx_left) * z / self.fx, (left.y - self.cy) * z / self.fy, z)
    }

    /// Triangulates a rectified correspondence into the sensor frame (+X right, +Y forward,
    /// +Z up). Uses the mean row of both observations.
    pub fn triangulate(&self, left: Vec2, right: Vec2) -> Option<Vec3> {
        let d = self.disparity(left, right);
        if d <= 0.0 {
            return None;
        }
        let o = self.optical(Vec2::new(left.x, 0.5 * (left.y + right.y)), d);
        Some(optical_to_sensor(o))
    }

    /// Projects a sensor-frame point into the left and right rectified images.
    pub fn project(&self, p: Vec3) -> Option<(Vec2, Vec2)> {
        let o = sensor_to_optical(p);
        if o.z <= 0.0 {
            return None;
        }
        let u = self.fx * o.x / o.z + self.cx_left;
        let v = self.fy * o.y / o.z + self.cy;
        let d = self.fx * self.baseline_mm / o.z;
        Some((Vec2::new(u, v), Vec2::new(u - self.cx_left - d + self.cx_right, v)))
    }

    /// Disparity of a point at depth `z_mm`.
    pub fn disparity_at(&self, z_mm: f32) -> f32 {
        self.fx * self.baseline_mm / z_mm
    }
}

/// Optical frame (x right, y down, z forward) -> Z-up sensor frame (+X right, +Y forward, +Z up),
/// as `DepthMap::to_point_cloud`.
pub fn optical_to_sensor(o: Vec3) -> Vec3 {
    Vec3::new(o.x, o.z, -o.y)
}

pub fn sensor_to_optical(p: Vec3) -> Vec3 {
    Vec3::new(p.x, -p.z, p.y)
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct StereoConfig {
    /// Largest row difference of a correspondence, px. Revo uses 1.0 px at full resolution
    /// (`markerrecon_epipolarerror 1.0`, RevoScan6 log); 640x512 streams halve the sampling.
    pub epipolar_tol_px: f32,
    /// Depth range searched, mm.
    pub min_depth_mm: f32,
    pub max_depth_mm: f32,
    /// Largest left/right area ratio of a pair (the same dot looks alike in both views).
    pub max_area_ratio: f32,
    /// A pair is rejected if the second-best candidate's row error is within this many px of
    /// the best one (ambiguous on the epipolar line).
    pub ambiguity_px: f32,
    /// Physical dot diameter limits, mm (Revo stickers: 3 mm and 6 mm inner diameter). Dots
    /// whose triangulated size falls outside are rejected.
    pub min_diameter_mm: f32,
    pub max_diameter_mm: f32,
}

impl Default for StereoConfig {
    fn default() -> Self {
        Self {
            epipolar_tol_px: 1.5,
            min_depth_mm: 80.0,
            max_depth_mm: 900.0,
            max_area_ratio: 2.5,
            ambiguity_px: 0.6,
            min_diameter_mm: 1.5,
            max_diameter_mm: 14.0,
        }
    }
}

/// A triangulated marker in the sensor frame of one exposure.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Marker3D {
    pub position: Vec3,
    pub left: Vec2,
    pub right: Vec2,
    /// Row difference of the correspondence, px.
    pub epipolar_error: f32,
    /// Estimated dot diameter (from the ellipse major axis), mm.
    pub diameter_mm: f32,
}

/// Matches left and right detections and triangulates them. `depth` (left-image depth map, e.g.
/// the on-device z16 plane) resolves epipolar ambiguities when present: a candidate whose
/// disparity disagrees with the surrounding depth by more than 10 % is dropped (markers are
/// often depth holes, then the test is skipped).
pub fn match_stereo(left: &[Marker2D], right: &[Marker2D], geo: &StereoGeometry, cfg: &StereoConfig, depth: Option<&DepthMap>) -> Vec<Marker3D> {
    let d_min = geo.disparity_at(cfg.max_depth_mm);
    let d_max = geo.disparity_at(cfg.min_depth_mm);
    // All geometrically valid pairs with a cost: row error (in tolerances) + size mismatch.
    struct Cand {
        i: usize,
        j: usize,
        dv: f32,
        cost: f32,
        p: Vec3,
        diameter_mm: f32,
    }
    let mut cands: Vec<Cand> = Vec::new();
    for (i, l) in left.iter().enumerate() {
        let depth_d = depth.and_then(|d| depth_near(d, l.x, l.y, l.major)).map(|z| geo.disparity_at(z));
        for (j, r) in right.iter().enumerate() {
            let dv = (r.y - l.y).abs();
            if dv > cfg.epipolar_tol_px {
                continue;
            }
            let (lp, rp) = (Vec2::new(l.x, l.y), Vec2::new(r.x, r.y));
            let d = geo.disparity(lp, rp);
            let ratio = (l.area / r.area).max(r.area / l.area);
            if d < d_min || d > d_max || ratio > cfg.max_area_ratio {
                continue;
            }
            if depth_d.is_some_and(|dd| (d - dd).abs() > 0.1 * dd + 1.0) {
                continue;
            }
            let Some(p) = geo.triangulate(lp, rp) else { continue };
            let diameter_mm = (l.major + r.major) * p.y / geo.fx;
            if diameter_mm < cfg.min_diameter_mm || diameter_mm > cfg.max_diameter_mm {
                continue;
            }
            let cost = dv / cfg.epipolar_tol_px + ratio.ln();
            cands.push(Cand { i, j, dv, cost, p, diameter_mm });
        }
    }
    // Greedy one-to-one assignment, cheapest first. A left dot whose next-best still-free
    // candidate is nearly as good is ambiguous and skipped (unless that candidate is taken).
    cands.sort_by(|a, b| a.cost.total_cmp(&b.cost));
    let mut used_l = vec![false; left.len()];
    let mut used_r = vec![false; right.len()];
    let mut out = Vec::new();
    let margin = cfg.ambiguity_px / cfg.epipolar_tol_px;
    for (k, c) in cands.iter().enumerate() {
        if used_l[c.i] || used_r[c.j] {
            continue;
        }
        let rival = cands[k + 1..].iter().find(|o| (o.i == c.i && !used_r[o.j]) || (o.j == c.j && !used_l[o.i]));
        if let Some(o) = rival.filter(|o| o.cost - c.cost < margin) {
            // Ambiguous: neither this pair nor the contested dots may be used by a weaker pair
            // (otherwise the rival is accepted next, unopposed).
            used_l[c.i] = true;
            used_r[c.j] = true;
            used_l[o.i] = true;
            used_r[o.j] = true;
            continue;
        }
        used_l[c.i] = true;
        used_r[c.j] = true;
        let (l, r) = (&left[c.i], &right[c.j]);
        out.push(Marker3D { position: c.p, left: Vec2::new(l.x, l.y), right: Vec2::new(r.x, r.y), epipolar_error: c.dv, diameter_mm: c.diameter_mm });
    }
    // Size consistency: a dot whose true partner is outside the right image can pair with a
    // neighbour on the same row; the wrong depth then gives a wrong physical size. Markers come
    // in a few sizes, so keep a dot only if at least two others have a similar diameter (±20 %).
    // Measured on a turntable orbit: true pairs 6.7–7.3 mm, wrong ones 9–11 mm.
    if out.len() >= 4 {
        let sizes: Vec<f32> = out.iter().map(|m| m.diameter_mm).collect();
        out.retain(|m| sizes.iter().filter(|&&s| (s / m.diameter_mm - 1.0).abs() < 0.2).count() >= 3);
    }
    out
}

/// Median valid depth in a ring around a dot (the dot itself is usually a hole), mm.
fn depth_near(depth: &DepthMap, x: f32, y: f32, radius: f32) -> Option<f32> {
    let (w, h) = (depth.width as i32, depth.height as i32);
    let r_in = radius.ceil() as i32 + 1;
    let r_out = r_in + 4;
    let (cx, cy) = (x.round() as i32, y.round() as i32);
    let mut v = Vec::new();
    for yy in (cy - r_out).max(0)..=(cy + r_out).min(h - 1) {
        for xx in (cx - r_out).max(0)..=(cx + r_out).min(w - 1) {
            let d2 = (xx - cx).pow(2) + (yy - cy).pow(2);
            if d2 < r_in * r_in || d2 > r_out * r_out {
                continue;
            }
            let raw = depth.data[(yy * w + xx) as usize];
            if raw != 0 {
                v.push(raw as f32 * depth.scale_mm);
            }
        }
    }
    if v.len() < 6 {
        return None;
    }
    let mid = v.len() / 2;
    Some(*v.select_nth_unstable_by(mid, |a, b| a.total_cmp(b)).1)
}

/// Estimates `cx_right - cx_left` of a rectified pair from a frame that also carries a depth map
/// of the left image (Full-field HD): for textured pixels with valid depth, block-match the left
/// patch along the same row of the right image and compare with the disparity the depth
/// implies. Returns the median offset if enough samples agree (IQR < 4 px).
pub fn estimate_right_cx_offset(ir: &IrImages, depth: &DepthMap, fx: f32, baseline_mm: f32) -> Option<f32> {
    offset_from_samples(&mut right_cx_offset_samples(ir, depth, fx, baseline_mm))
}

/// Offset samples of one frame (see [`estimate_right_cx_offset`]); samples of several frames
/// can be pooled before [`offset_from_samples`].
pub fn right_cx_offset_samples(ir: &IrImages, depth: &DepthMap, fx: f32, baseline_mm: f32) -> Vec<f32> {
    let Some(right) = ir.right.as_ref() else { return Vec::new() };
    let left = &ir.left;
    let (w, h) = (left.width as usize, left.height as usize);
    if right.width as usize != w || right.height as usize != h || depth.width as usize != w || depth.height as usize != h {
        return Vec::new();
    }
    const R: usize = 8;
    let patch = |img: &[u8], x: usize, y: usize| -> Vec<f32> {
        let mut p = Vec::with_capacity((2 * R + 1).pow(2));
        for yy in y - R..=y + R {
            for xx in x - R..=x + R {
                p.push(img[yy * w + xx] as f32);
            }
        }
        p
    };
    let normalise = |p: &mut Vec<f32>| -> bool {
        let m = p.iter().sum::<f32>() / p.len() as f32;
        let s = (p.iter().map(|v| (v - m) * (v - m)).sum::<f32>()).sqrt();
        if s < 1e-3 {
            return false;
        }
        p.iter_mut().for_each(|v| *v = (*v - m) / s);
        true
    };
    let mut offsets = Vec::new();
    let step = 13;
    for y in (40..h - 40).step_by(step) {
        for x in (40..w - 40).step_by(step) {
            let raw = depth.data[y * w + x];
            if raw == 0 {
                continue;
            }
            let z = raw as f32 * depth.scale_mm;
            let mut p = patch(&left.data, x, y);
            let mean = p.iter().sum::<f32>() / p.len() as f32;
            let var = p.iter().map(|v| (v - mean).powi(2)).sum::<f32>() / p.len() as f32;
            if var < 25.0 || !normalise(&mut p) {
                continue;
            }
            let (mut best, mut second, mut best_x) = (-1.0f32, -1.0f32, 0usize);
            for xr in R..w - R {
                let mut q = patch(&right.data, xr, y);
                if !normalise(&mut q) {
                    continue;
                }
                let c: f32 = p.iter().zip(&q).map(|(a, b)| a * b).sum();
                if c > best {
                    if (xr as i64 - best_x as i64).abs() > 3 {
                        second = best;
                    }
                    best = c;
                    best_x = xr;
                } else if c > second && (xr as i64 - best_x as i64).abs() > 3 {
                    second = c;
                }
            }
            if best > 0.8 && best - second > 0.05 {
                offsets.push(best_x as f32 - x as f32 + fx * baseline_mm / z);
            }
        }
    }
    log::debug!("right cx offset samples: {offsets:?}");
    offsets
}

/// Mode of pooled offset samples, `None` while there are too few consistent ones.
pub fn offset_from_samples(offsets: &mut [f32]) -> Option<f32> {
    if offsets.len() < 8 {
        return None;
    }
    // Mode: the 6 px window holding most samples (wrong matches scatter, the true offset
    // clusters; depth noise spreads it by a few px). Needs >= 8 samples and 40 % of all.
    offsets.sort_by(|a, b| a.total_cmp(b));
    let (mut best, mut best_range) = (0usize, 0..0);
    let mut j = 0;
    for i in 0..offsets.len() {
        while offsets[i] - offsets[j] > 6.0 {
            j += 1;
        }
        if i + 1 - j > best {
            best = i + 1 - j;
            best_range = j..i + 1;
        }
    }
    if best < 8 || (best as f32) < 0.4 * offsets.len() as f32 {
        return None;
    }
    let window = &offsets[best_range];
    Some(window[window.len() / 2])
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::detect::tests::{blank, draw_ellipse};
    use crate::detect::{detect_markers, DetectorConfig};
    use scancraft_core::PinholeIntrinsics;

    pub fn pop4_geometry() -> StereoGeometry {
        StereoGeometry { fx: 742.278, fy: 742.278, cx_left: 170.603, cx_right: 475.519, cy: 242.566, baseline_mm: 118.58 }
    }

    #[test]
    fn projection_and_triangulation_round_trip() {
        let g = pop4_geometry();
        for p in [Vec3::new(10.0, 300.0, 5.0), Vec3::new(-60.0, 420.0, -40.0), Vec3::new(30.0, 220.0, 30.0)] {
            let (l, r) = g.project(p).unwrap();
            let q = g.triangulate(l, r).unwrap();
            assert!((p - q).length() < 1e-3, "{p} vs {q}");
        }
    }

    #[test]
    fn matches_rendered_markers_and_triangulates() {
        let g = pop4_geometry();
        let (mut li, mut ri) = (blank(640, 512, 25), blank(640, 512, 25));
        let truth = [Vec3::new(-20.0, 280.0, 10.0), Vec3::new(15.0, 300.0, -25.0), Vec3::new(40.0, 330.0, 20.0), Vec3::new(-5.0, 310.0, 30.5)];
        for p in truth {
            let (l, r) = g.project(p).unwrap();
            let radius = 3.0 * g.fx / p.y; // 6 mm dot
            draw_ellipse(&mut li, l.x, l.y, radius, radius, 0.0, 255.0);
            draw_ellipse(&mut ri, r.x, r.y, radius, radius, 0.0, 255.0);
        }
        let dc = DetectorConfig::default();
        let (dl, dr) = (detect_markers(&li, &dc), detect_markers(&ri, &dc));
        let m = match_stereo(&dl, &dr, &g, &StereoConfig::default(), None);
        assert_eq!(m.len(), truth.len());
        for p in truth {
            let best = m.iter().map(|q| (q.position - p).length()).fold(f32::MAX, f32::min);
            assert!(best < 0.15, "{p}: {best} mm");
        }
        assert!(m.iter().all(|q| (q.diameter_mm - 6.0).abs() < 0.8), "{m:?}");
    }

    #[test]
    fn estimates_principal_point_offset_from_depth() {
        // Random texture on a fronto-parallel plane at 300 mm; right image shifted accordingly.
        let g = pop4_geometry();
        let (w, h) = (640usize, 512usize);
        let mut rng = crate::detect::tests::Lcg(11);
        let tex: Vec<u8> = (0..w * 2 * h).map(|_| (rng.next_f32() * 200.0) as u8).collect();
        let z = 300.0;
        let d = g.disparity_at(z);
        let shift = g.cx_right - g.cx_left - d; // uR = uL + shift
        let mut left = blank(w as u32, h as u32, 0);
        let mut right = blank(w as u32, h as u32, 0);
        for y in 0..h {
            for x in 0..w {
                left.data[y * w + x] = tex[y * 2 * w + x + 200];
                let src = x as f32 - shift + 200.0;
                right.data[y * w + x] = tex[y * 2 * w + src.round().clamp(0.0, (2 * w - 1) as f32) as usize];
            }
        }
        let depth = DepthMap::new(w as u32, h as u32, 0.01, vec![(z / 0.01) as u16; w * h]).with_intrinsics(PinholeIntrinsics {
            fx: g.fx,
            fy: g.fy,
            cx: g.cx_left,
            cy: g.cy,
        });
        let ir = IrImages { left, right: Some(right), rectified: true, rig: None };
        let off = estimate_right_cx_offset(&ir, &depth, g.fx, g.baseline_mm).expect("estimate");
        assert!((off - (g.cx_right - g.cx_left)).abs() < 1.0, "offset {off}");
    }
}
