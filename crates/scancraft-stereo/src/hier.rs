//! Coarse-to-fine matching: semi-global matching at half resolution, then a per-pixel search at
//! full resolution in a narrow band around the coarse disparity.
//!
//! The SGM cost grows with width x height x disparities; at half resolution all three halve, so
//! the expensive part runs on an eighth of the volume. The coarse pass resolves which surface a
//! pixel belongs to (the ambiguity SGM is there for); the fine pass only has to locate it within
//! a few pixels, which a small aggregated census cost does well.

use crate::census::{census_9x7, texture_mask, HALF_H, HALF_W};
use crate::filter::{fill_small_holes, remove_speckles};
use crate::sgm::{DisparityImage, SgmMatcher, SgmParams};
use rayon::prelude::*;

/// Full-resolution disparities searched on each side of the coarse estimate. The coarse value is
/// good to about half a coarse pixel (one full pixel); the rest covers surface slope inside the
/// 2x2 block.
const BAND: i32 = 3;
/// Offsets (px) of the census cells summed into one fine cost: centre plus a ring, so the cost
/// sees an area of about 13x11 px without the price of a dense window.
const TAPS: [(i32, i32); 5] = [(0, 0), (-2, 0), (2, 0), (0, -2), (0, 2)];
/// A fine match is rejected if its mean census distance exceeds this (of 62 bits).
const MAX_MEAN_COST: u32 = 22;

/// Reusable coarse-to-fine matcher.
pub struct HierarchicalMatcher {
    coarse: SgmMatcher,
    half_left: Vec<u8>,
    half_right: Vec<u8>,
    census_left: Vec<u64>,
    census_right: Vec<u64>,
    textured: Vec<bool>,
}

impl Default for HierarchicalMatcher {
    fn default() -> Self {
        Self::new()
    }
}

impl HierarchicalMatcher {
    pub fn new() -> Self {
        Self {
            coarse: SgmMatcher::new(SgmParams::default()),
            half_left: Vec::new(),
            half_right: Vec::new(),
            census_left: Vec::new(),
            census_right: Vec::new(),
            textured: Vec::new(),
        }
    }

    /// Matches a rectified 8-bit pair. `params` are the full-resolution settings (disparity
    /// window, filters); the coarse pass derives its own from them.
    pub fn compute(&mut self, left: &[u8], right: &[u8], width: usize, height: usize, params: &SgmParams) -> DisparityImage {
        assert_eq!(left.len(), width * height, "left image size");
        assert_eq!(right.len(), width * height, "right image size");
        let (cw, ch) = (width / 2, height / 2);
        halve(left, width, cw, ch, &mut self.half_left);
        halve(right, width, cw, ch, &mut self.half_right);

        // Coarse window: the full window in half-size pixels, one pixel of margin on each side.
        let lo = params.min_disparity.div_euclid(2) - 1;
        let hi = (params.min_disparity + params.num_disparities as i32).div_euclid(2) + 2;
        self.coarse.params = SgmParams {
            min_disparity: lo,
            num_disparities: ((hi - lo) as usize).div_ceil(16) * 16,
            // Regions and holes cover a quarter of the pixels at half size.
            speckle_window: params.speckle_window / 4,
            max_hole: params.max_hole / 4,
            ..params.clone()
        };
        let coarse = self.coarse.compute(&self.half_left, &self.half_right, cw, ch);

        census_9x7(left, width, height, &mut self.census_left);
        census_9x7(right, width, height, &mut self.census_right);
        texture_mask(left, width, height, params.min_texture, &mut self.textured);
        let (cl, cr, textured) = (&self.census_left, &self.census_right, &self.textured);
        let (d_lo, d_hi) = (params.min_disparity, params.min_disparity + params.num_disparities as i32 - 1);

        let mut data = vec![f32::NAN; width * height];
        let margin_y = HALF_H + 2;
        data.par_chunks_mut(width).enumerate().for_each(|(y, out)| {
            if y < margin_y || y + margin_y >= height || y / 2 >= ch {
                return;
            }
            let crow = &coarse.data[(y / 2) * cw..(y / 2 + 1) * cw];
            for x in (HALF_W + 2)..width.saturating_sub(HALF_W + 2) {
                if !textured[y * width + x] || x / 2 >= cw {
                    continue;
                }
                let c = crow[x / 2];
                if c.is_nan() {
                    continue;
                }
                let centre = (2.0 * c).round() as i32;
                let mut costs = [u32::MAX; (2 * BAND + 1) as usize];
                let (mut best, mut best_cost) = (usize::MAX, u32::MAX);
                for (i, cost) in costs.iter_mut().enumerate() {
                    let d = centre - BAND + i as i32;
                    if d < d_lo || d > d_hi {
                        continue;
                    }
                    let xr = x as i32 - d;
                    if xr < (HALF_W + 2) as i32 || xr + (HALF_W + 2) as i32 >= width as i32 {
                        continue;
                    }
                    let mut sum = 0;
                    for (dx, dy) in TAPS {
                        let row = (y as i32 + dy) as usize * width;
                        sum += (cl[row + (x as i32 + dx) as usize] ^ cr[row + (xr + dx) as usize]).count_ones();
                    }
                    *cost = sum;
                    if sum < best_cost {
                        (best, best_cost) = (i, sum);
                    }
                }
                if best == usize::MAX || best_cost > MAX_MEAN_COST * TAPS.len() as u32 {
                    continue;
                }
                let mut sub = 0.0;
                if best > 0 && best + 1 < costs.len() && costs[best - 1] != u32::MAX && costs[best + 1] != u32::MAX {
                    let (a, b, c) = (costs[best - 1] as f32, costs[best] as f32, costs[best + 1] as f32);
                    let denom = a - 2.0 * b + c;
                    if denom > 0.0 {
                        sub = (0.5 * (a - c) / denom).clamp(-0.5, 0.5);
                    }
                }
                out[x] = (centre - BAND + best as i32) as f32 + sub;
            }
        });
        let mut disp = DisparityImage { width, height, data };
        if params.speckle_window > 0 {
            remove_speckles(&mut disp, params.speckle_window, params.speckle_range);
        }
        if params.max_hole > 0 {
            fill_small_holes(&mut disp, params.max_hole, 2.0 * params.speckle_range.max(0.5));
        }
        disp
    }
}

/// 2x2 box average into a `cw` x `ch` image.
fn halve(src: &[u8], width: usize, cw: usize, ch: usize, out: &mut Vec<u8>) {
    out.clear();
    out.resize(cw * ch, 0);
    out.par_chunks_mut(cw).enumerate().for_each(|(y, row)| {
        let (a, b) = (&src[2 * y * width..], &src[(2 * y + 1) * width..]);
        for (x, o) in row.iter_mut().enumerate() {
            *o = ((a[2 * x] as u16 + a[2 * x + 1] as u16 + b[2 * x] as u16 + b[2 * x + 1] as u16 + 2) / 4) as u8;
        }
    });
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sgm::tests::{render_right, texture};

    #[test]
    fn matches_a_slanted_surface_like_full_resolution_sgm() {
        let (w, h) = (320usize, 128usize);
        let left = texture(w, h, 11);
        // Disparity 20..28 px across the image (a slanted plane).
        let truth = |x: usize, _y: usize| 20.0 + 8.0 * x as f32 / w as f32;
        let right = render_right(&left, w, h, truth);
        let params = SgmParams { min_disparity: 8, num_disparities: 48, ..SgmParams::default() };
        let disp = HierarchicalMatcher::new().compute(&left, &right, w, h, &params);
        let mut errors: Vec<f32> = Vec::new();
        for y in 16..h - 16 {
            for x in 48..w - 16 {
                let d = disp.data[y * w + x];
                if !d.is_nan() {
                    errors.push((d - truth(x, y)).abs());
                }
            }
        }
        let interior = (h - 32) * (w - 64);
        assert!(errors.len() > interior * 8 / 10, "coverage {} of {interior}", errors.len());
        errors.sort_by(f32::total_cmp);
        assert!(errors[errors.len() / 2] < 0.25, "median error {}", errors[errors.len() / 2]);
        assert!(errors[errors.len() * 95 / 100] < 1.0, "p95 error {}", errors[errors.len() * 95 / 100]);
    }
}
