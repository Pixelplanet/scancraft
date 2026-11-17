//! Host depth for structured light that combines a random speckle pattern with phase-shifted
//! fringes (Revopoint "Hybrid HD"). See `docs/protocol/hybrid.md`.
//!
//! 1. **Coarse**: windowed ZNCC on the rectified speckle pair, joined with a fringe-phase
//!    agreement term, over the disparity range of the working distance. The speckle resolves the
//!    fringe period ambiguity (~14 px on the POP 4); the phase term sharpens the winner.
//! 2. **Fine**: sub-pixel disparity where the right image's wrapped phase equals the left
//!    pixel's phase, within ±1.5 px of the coarse winner.
//! 3. **Filter**: uniqueness, then agreement with the 5x5 median of the disparity map.
//!
//! [`match_pair`] (speckle only) is the generic "rectified pair + rig -> depth" boundary; it is
//! meant to be replaced by the shared matcher (`scancraft-stereo`) once that lands. The phase
//! refinement in [`hybrid_depth`] only needs a coarse disparity within ±1.5 px.

pub mod phase;
pub mod stereo;

use phase::{find_equal_phase, PhaseMap};
use scancraft_core::{DepthMap, FringeImages, RgbImage, StereoRig};
use stereo::{score_at, sweep, Normalized, PhaseTerms, SweepInput};

/// Raw unit of the produced depth maps (same as the POP 4's on-device z16).
pub const DEPTH_SCALE_MM: f32 = 0.01;

#[derive(Debug, Clone)]
pub struct HybridParams {
    /// Working range; sets the disparity search window.
    pub min_mm: f32,
    pub max_mm: f32,
    /// Matching window radius (window = 2r+1 square).
    pub radius: usize,
    /// Weight of the fringe-phase term in the coarse score (0 = speckle only).
    pub phase_weight: f32,
    /// Minimum fringe amplitude (grey levels) for a pixel's phase to count.
    pub min_modulation: f32,
    /// Minimum local speckle contrast (grey-level standard deviation).
    pub min_texture: f32,
    /// Minimum coarse score of the winner.
    pub min_score: f32,
    /// Minimum margin between winner and the best non-adjacent disparity.
    pub min_margin: f32,
    /// Maximum deviation from the 5x5 median disparity, px.
    pub median_tolerance_px: f32,
    /// Take the coarse disparity from the coarse-to-fine census matcher of `scancraft-stereo`
    /// (~25 ms) instead of the full disparity sweep (~180 ms); the fringe phase refines it
    /// either way.
    pub fast_coarse: bool,
}

impl Default for HybridParams {
    fn default() -> Self {
        Self {
            min_mm: 200.0,
            max_mm: 600.0,
            radius: 4,
            phase_weight: 0.5,
            min_modulation: 4.0,
            min_texture: 2.0,
            min_score: 0.4,
            min_margin: 0.04,
            median_tolerance_px: 1.0,
            fast_coarse: true,
        }
    }
}

/// Speckle + fringe depth. `left`/`right` are the rectified speckle images, `fringes` the
/// rectified phase-shifted images, all on `rig`. Falls back to speckle-only matching with
/// parabolic sub-pixel fit where the fringe phase is unusable.
pub fn hybrid_depth(left: &RgbImage, right: &RgbImage, fringes: Option<&FringeImages>, rig: &StereoRig, params: &HybridParams) -> DepthMap {
    let disparity = hybrid_disparity(left, right, fringes, rig, params);
    disparity_to_depth(&disparity, left.width, left.height, rig, params)
}

/// Speckle-only stereo: rectified pair + rig -> depth. The swap point for `scancraft-stereo`.
pub fn match_pair(left: &RgbImage, right: &RgbImage, rig: &StereoRig, params: &HybridParams) -> DepthMap {
    hybrid_depth(left, right, None, rig, params)
}

/// Sub-pixel disparity `x_left - x_right` per left pixel, NaN where invalid.
pub fn hybrid_disparity(left: &RgbImage, right: &RgbImage, fringes: Option<&FringeImages>, rig: &StereoRig, p: &HybridParams) -> Vec<f32> {
    if p.fast_coarse {
        return fast_disparity(left, right, fringes, rig, p);
    }
    let (w, h) = (left.width as usize, left.height as usize);
    let ln = Normalized::new(left, p.radius);
    let rn = Normalized::new(right, p.radius);
    let phases = fringes.and_then(|f| Some((PhaseMap::from_images(&f.left)?, PhaseMap::from_images(&f.right)?)));
    let phases = phases.filter(|(pl, _)| pl.width == w && pl.height == h);
    let terms = phases.as_ref().map(|(pl, pr)| (PhaseTerms::new(pl, p.min_modulation), PhaseTerms::new(pr, p.min_modulation)));

    let d_min = rig.disparity_from_depth(p.max_mm).floor() as i32;
    let d_max = rig.disparity_from_depth(p.min_mm).ceil() as i32;
    let input = SweepInput {
        left: &ln,
        right: &rn,
        phase: terms.as_ref().map(|(a, b)| (a, b)),
        phase_weight: p.phase_weight,
        radius: p.radius,
        d_min,
        d_max,
    };
    let coarse = sweep(&input);

    let mut disparity = vec![f32::NAN; w * h];
    for y in 0..h {
        for x in 0..w {
            let i = y * w + x;
            let d = coarse.disparity[i];
            if d == i32::MIN || ln.std[i] < p.min_texture || coarse.score[i] < p.min_score || coarse.score[i] - coarse.second[i] < p.min_margin {
                continue;
            }
            let refined = match &phases {
                Some((pl, pr)) if pl.modulation[i] >= p.min_modulation => {
                    let guess = x as f32 - d as f32;
                    find_equal_phase(pr, y, pl.phase[i], guess, (guess - 2.0).floor() as isize, (guess + 1.0).ceil() as isize, p.min_modulation)
                        .filter(|xr| (xr - guess).abs() <= 1.5)
                        .map(|xr| x as f32 - xr)
                }
                _ => None,
            };
            disparity[i] = match refined {
                Some(v) => v,
                None if phases.is_some() => continue, // fringe area without a phase match: unreliable
                None => parabola(&input, x, y, d).unwrap_or(d as f32),
            };
        }
    }
    median_filter_check(&disparity, w, h, p.median_tolerance_px)
}

thread_local! {
    /// Keeps the matcher's buffers between frames.
    static MATCHER: std::cell::RefCell<scancraft_stereo::HierarchicalMatcher> = std::cell::RefCell::new(scancraft_stereo::HierarchicalMatcher::new());
}

/// [`hybrid_disparity`] with the coarse disparity from the coarse-to-fine census matcher on the
/// speckle pair. Where fringes are present, each pixel is then placed by its fringe phase, found
/// within 1.5 px of the speckle match (the fringe period is several pixels, so the speckle match
/// selects the right fringe); pixels without a usable phase are dropped, as in the sweep path.
fn fast_disparity(left: &RgbImage, right: &RgbImage, fringes: Option<&FringeImages>, rig: &StereoRig, p: &HybridParams) -> Vec<f32> {
    let (w, h) = (left.width as usize, left.height as usize);
    let d_min = rig.disparity_from_depth(p.max_mm).floor() as i32 - 1;
    let d_max = rig.disparity_from_depth(p.min_mm).ceil() as i32 + 1;
    let params = scancraft_stereo::SgmParams {
        min_disparity: d_min,
        num_disparities: ((d_max - d_min + 1).max(16) as usize).div_ceil(16) * 16,
        min_texture: p.min_texture,
        ..Default::default()
    };
    let prior = MATCHER.with(|m| m.borrow_mut().compute(&left.data, &right.data, w, h, &params));
    let phases = fringes.and_then(|f| Some((PhaseMap::from_images(&f.left)?, PhaseMap::from_images(&f.right)?)));
    let phases = phases.filter(|(pl, _)| pl.width == w && pl.height == h);
    let Some((pl, pr)) = &phases else {
        return median_filter_check(&prior.data, w, h, p.median_tolerance_px);
    };
    let mut disparity = vec![f32::NAN; w * h];
    use rayon::prelude::*;
    disparity.par_chunks_mut(w).enumerate().for_each(|(y, row)| {
        for (x, out) in row.iter_mut().enumerate() {
            let i = y * w + x;
            let d = prior.data[i];
            if d.is_nan() || pl.modulation[i] < p.min_modulation {
                continue;
            }
            let guess = x as f32 - d;
            if let Some(xr) = find_equal_phase(pr, y, pl.phase[i], guess, (guess - 2.0).floor() as isize, (guess + 2.0).ceil() as isize, p.min_modulation)
                .filter(|xr| (xr - guess).abs() <= 1.5)
            {
                *out = x as f32 - xr;
            }
        }
    });
    median_filter_check(&disparity, w, h, p.median_tolerance_px)
}

fn parabola(input: &SweepInput, x: usize, y: usize, d: i32) -> Option<f32> {
    let (a, b, c) = (score_at(input, x, y, d - 1)?, score_at(input, x, y, d)?, score_at(input, x, y, d + 1)?);
    let denom = a - 2.0 * b + c;
    (denom < 0.0).then(|| d as f32 + (0.5 * (a - c) / denom).clamp(-0.5, 0.5))
}

/// Keeps pixels within `tol` of the median of their valid 5x5 neighbourhood (at least 8 valid
/// neighbours): removes isolated mismatches and flying pixels.
fn median_filter_check(d: &[f32], w: usize, h: usize, tol: f32) -> Vec<f32> {
    let mut out = vec![f32::NAN; d.len()];
    let mut window = Vec::with_capacity(25);
    for y in 2..h.saturating_sub(2) {
        for x in 2..w.saturating_sub(2) {
            let v = d[y * w + x];
            if v.is_nan() {
                continue;
            }
            window.clear();
            for yy in y - 2..=y + 2 {
                window.extend(d[yy * w + x - 2..=yy * w + x + 2].iter().copied().filter(|v| !v.is_nan()));
            }
            if window.len() < 9 {
                continue;
            }
            let mid = window.len() / 2;
            let median = *window.select_nth_unstable_by(mid, |a, b| a.total_cmp(b)).1;
            if (v - median).abs() <= tol {
                out[y * w + x] = v;
            }
        }
    }
    out
}

fn disparity_to_depth(disparity: &[f32], width: u32, height: u32, rig: &StereoRig, p: &HybridParams) -> DepthMap {
    let data = disparity
        .iter()
        .map(|&d| match rig.depth_from_disparity(d) {
            Some(z) if !d.is_nan() && z >= p.min_mm && z <= p.max_mm => (z / DEPTH_SCALE_MM).round() as u16,
            _ => 0,
        })
        .collect();
    DepthMap::new(width, height, DEPTH_SCALE_MM, data).with_intrinsics(rig.intrinsics)
}

#[cfg(test)]
mod tests {
    use super::*;
    use scancraft_core::PinholeIntrinsics;
    use std::f32::consts::PI;

    /// Renders a synthetic rectified scene: a plane at depth z(x) seen by a verged rig, with a
    /// random dot texture and 4-step fringes attached to the surface (projector coordinate).
    fn render(rig: &StereoRig, z_of_x: impl Fn(f32) -> f32) -> (RgbImage, RgbImage, FringeImages) {
        let (w, h) = (rig.width as usize, rig.height as usize);
        // Surface coordinate s (mm along x at the surface) -> texture.
        let dots = |s: f32, y: usize| -> f32 {
            // splitmix64 of the (surface cell, row) pair.
            let mut z = ((s.floor() as i64 as u64) << 20 ^ y as u64).wrapping_add(0x9e3779b97f4a7c15);
            z = (z ^ (z >> 30)).wrapping_mul(0xbf58476d1ce4e5b9);
            z = (z ^ (z >> 27)).wrapping_mul(0x94d049bb133111eb);
            let v = ((z ^ (z >> 31)) & 0xff) as f32;
            if v > 180.0 {
                220.0
            } else {
                40.0
            }
        };
        let fringe = |s: f32, k: usize| 100.0 + 60.0 * (2.0 * PI * s / 4.0 + PI / 2.0 * k as f32).cos();
        // For each rectified image, find the surface coordinate s seen at pixel x by fixed-point
        // iteration of x = fx * X / Z + cx with X = s (surface point (s, y, z(s))).
        let k = rig.intrinsics;
        let surface_s = |x: usize, right: bool| -> f32 {
            let cx = if right { k.cx + rig.disparity_offset_px } else { k.cx };
            let mut s = 0.0;
            for _ in 0..30 {
                let z = z_of_x(s);
                s = (x as f32 - cx) * z / k.fx + if right { rig.baseline_mm } else { 0.0 };
            }
            s
        };
        let mut left = RgbImage { width: w as u32, height: h as u32, channels: 1, data: vec![0; w * h] };
        let mut right = left.clone();
        let mut fl: Vec<RgbImage> = (0..4).map(|_| left.clone()).collect();
        let mut fr = fl.clone();
        for x in 0..w {
            let (sl, sr) = (surface_s(x, false), surface_s(x, true));
            for y in 0..h {
                left.data[y * w + x] = dots(sl, y) as u8;
                right.data[y * w + x] = dots(sr, y) as u8;
                for kk in 0..4 {
                    fl[kk].data[y * w + x] = fringe(sl, kk) as u8;
                    fr[kk].data[y * w + x] = fringe(sr, kk) as u8;
                }
            }
        }
        (left, right, FringeImages { left: fl, right: fr })
    }

    fn rig() -> StereoRig {
        StereoRig {
            width: 320,
            height: 40,
            intrinsics: PinholeIntrinsics { fx: 742.28, fy: 742.28, cx: 160.0, cy: 20.0 },
            baseline_mm: 118.58,
            disparity_offset_px: 152.0,
            calibration_set: "test".into(),
            rectification: None,
        }
    }

    #[test]
    fn hybrid_recovers_slanted_plane() {
        let rig = rig();
        let z = |s: f32| 300.0 + 0.3 * s;
        let (l, r, f) = render(&rig, z);
        let depth = hybrid_depth(&l, &r, Some(&f), &rig, &HybridParams::default());
        let (mut n, mut err2) = (0, 0.0f64);
        let k = rig.intrinsics;
        for y in 8..32 {
            for x in 160..300 {
                let Some(zm) = depth.get_depth_mm(x, y) else { continue };
                // Expected depth at this pixel: solve z = 300 + 0.3 * X, X = (x - cx) z / fx.
                let t = (x as f32 - k.cx) / k.fx;
                let ze = 300.0 / (1.0 - 0.3 * t);
                err2 += ((zm - ze) as f64).powi(2);
                n += 1;
            }
        }
        let rms = (err2 / n as f64).sqrt();
        assert!(n > 2700, "coverage {n}");
        assert!(rms < 0.3, "rms {rms} mm over {n} px");
    }

    #[test]
    fn speckle_only_still_gives_depth() {
        let rig = rig();
        let (l, r, _) = render(&rig, |_| 350.0);
        let depth = match_pair(&l, &r, &rig, &HybridParams::default());
        let valid: Vec<f32> = (8..32).flat_map(|y| (160..300).map(move |x| (x, y))).filter_map(|(x, y)| depth.get_depth_mm(x, y)).collect();
        assert!(valid.len() > 2700, "coverage {}", valid.len());
        let mean = valid.iter().sum::<f32>() / valid.len() as f32;
        assert!((mean - 350.0).abs() < 2.0, "mean {mean}");
    }
}
