//! Semi-global matching (Hirschmüller 2008) on census costs, CPU + Rayon.
//!
//! Disparity convention: a left pixel `x` matches the right pixel `x - disparity`. Disparities are
//! searched in `min_disparity .. min_disparity + num_disparities` and may be negative (rectified
//! pairs whose right principal point lies to the right of the left one, as on the POP 4).
//!
//! Pipeline: census 9x7 → Hamming cost (computed on the fly, never stored) → 4 or 8 path
//! aggregation into a u16 cost sum `S(x, y, d)` → winner-takes-all with uniqueness test → parabola
//! sub-pixel fit → left-right consistency (right disparities taken from the same `S`) → speckle
//! removal and small-hole filling (`filter.rs`).

use crate::census::{census_9x7, texture_mask};
use crate::filter::{fill_small_holes, remove_speckles};
use rayon::prelude::*;

/// Path state padding value: large enough to never win a `min`, small enough that `+ p1` cannot
/// overflow a u16.
const BIG: u16 = 0x3fff;
/// Cost of a candidate whose right pixel lies outside the image (census costs are 0..=62).
const OUT_OF_IMAGE_COST: u16 = 63;

#[derive(Debug, Clone, PartialEq)]
pub struct SgmParams {
    /// Smallest disparity searched (`x_left - x_right`), px.
    pub min_disparity: i32,
    /// Number of disparities searched.
    pub num_disparities: usize,
    /// Penalty for a one-pixel disparity change between neighbours (census cost units, 0..62).
    pub p1: u16,
    /// Penalty for larger disparity jumps.
    pub p2: u16,
    /// 8 aggregation paths instead of 4 (fewer streaks, ~1.6x slower).
    pub diagonal_paths: bool,
    /// Keep a pixel only if its best cost is at most `uniqueness` x the best cost at a disparity
    /// more than one pixel away. 1.0 disables the test.
    pub uniqueness: f32,
    /// Maximum left-right disagreement, px. Negative disables the check.
    pub lr_max_diff: f32,
    /// Connected regions (neighbour disparity steps <= `speckle_range`) smaller than this many
    /// pixels are removed. 0 disables.
    pub speckle_window: usize,
    pub speckle_range: f32,
    /// Pixels whose 9x7 window has a lower intensity standard deviation (grey levels) in the
    /// left image are left without a match.
    pub min_texture: f32,
    /// Invalid regions of at most this many pixels, enclosed by a consistent surface, are filled.
    /// 0 disables.
    pub max_hole: usize,
}

impl Default for SgmParams {
    fn default() -> Self {
        Self {
            min_disparity: 0,
            num_disparities: 128,
            p1: 8,
            p2: 96,
            diagonal_paths: true,
            uniqueness: 0.95,
            lr_max_diff: 1.0,
            speckle_window: 200,
            speckle_range: 1.0,
            min_texture: 1.5,
            max_hole: 24,
        }
    }
}

/// Dense disparity image; `NaN` marks pixels without a match.
#[derive(Debug, Clone, PartialEq)]
pub struct DisparityImage {
    pub width: usize,
    pub height: usize,
    pub data: Vec<f32>,
}

impl DisparityImage {
    pub fn valid_count(&self) -> usize {
        self.data.iter().filter(|d| !d.is_nan()).count()
    }
}

/// Reusable matcher: keeps its buffers between frames, so repeated calls do not allocate the
/// (width x height x disparities) cost sum again.
pub struct SgmMatcher {
    pub params: SgmParams,
    census_left: Vec<u64>,
    census_right: Vec<u64>,
    textured: Vec<bool>,
    cost: Vec<u8>,
    sum: Vec<u16>,
}

impl SgmMatcher {
    pub fn new(params: SgmParams) -> Self {
        Self { params, census_left: Vec::new(), census_right: Vec::new(), textured: Vec::new(), cost: Vec::new(), sum: Vec::new() }
    }

    /// Matches a rectified 8-bit pair (row-major, same size).
    pub fn compute(&mut self, left: &[u8], right: &[u8], width: usize, height: usize) -> DisparityImage {
        assert_eq!(left.len(), width * height, "left image size");
        assert_eq!(right.len(), width * height, "right image size");
        let d = self.params.num_disparities.max(1);
        let t0 = std::time::Instant::now();
        census_9x7(left, width, height, &mut self.census_left);
        census_9x7(right, width, height, &mut self.census_right);
        texture_mask(left, width, height, self.params.min_texture, &mut self.textured);

        self.sum.resize(width * height * d, 0);
        self.cost.resize(width * height * d, 0);
        compute_costs(&self.census_left, &self.census_right, width, d, self.params.min_disparity, &mut self.cost);
        let ctx = Ctx {
            cost: &self.cost,
            w: width,
            h: height,
            d,
            dmin: self.params.min_disparity,
            p1: self.params.p1,
            p2: self.params.p2.max(self.params.p1 + 1),
        };
        let t1 = std::time::Instant::now();
        horizontal_pass(&ctx, &mut self.sum);
        let t2 = std::time::Instant::now();
        vertical_passes(&ctx, &mut self.sum, self.params.diagonal_paths);
        let t3 = std::time::Instant::now();

        let mut disp = select_disparities(&ctx, &self.sum, &self.params);
        let t4 = std::time::Instant::now();
        for (v, &ok) in disp.data.iter_mut().zip(&self.textured) {
            if !ok {
                *v = f32::NAN;
            }
        }
        if self.params.speckle_window > 0 {
            remove_speckles(&mut disp, self.params.speckle_window, self.params.speckle_range);
        }
        if self.params.max_hole > 0 {
            fill_small_holes(&mut disp, self.params.max_hole, 2.0 * self.params.speckle_range.max(0.5));
        }
        let ms = |a: std::time::Instant, b: std::time::Instant| (b - a).as_secs_f64() * 1e3;
        log::debug!(
            "sgm {width}x{height}x{d}: census {:.1} ms, horizontal {:.1}, vertical {:.1}, select {:.1}, filters {:.1}",
            ms(t0, t1),
            ms(t1, t2),
            ms(t2, t3),
            ms(t3, t4),
            ms(t4, std::time::Instant::now())
        );
        disp
    }
}

struct Ctx<'a> {
    /// Matching cost volume `C(x, y, d)`, disparities contiguous.
    cost: &'a [u8],
    w: usize,
    h: usize,
    d: usize,
    dmin: i32,
    p1: u16,
    p2: u16,
}

impl Ctx<'_> {
    #[inline]
    fn costs(&self, x: usize, y: usize) -> &[u8] {
        let o = (y * self.w + x) * self.d;
        &self.cost[o..o + self.d]
    }
}

/// Hamming costs of every pixel and disparity: `C(x, y, k) = popcount(L(x, y) ^ R(x - dmin - k, y))`.
fn compute_costs(cl: &[u64], cr: &[u64], w: usize, d: usize, dmin: i32, out: &mut [u8]) {
    out.par_chunks_mut(w * d).enumerate().for_each(|(y, row)| {
        let (lrow, rrow) = (&cl[y * w..(y + 1) * w], &cr[y * w..(y + 1) * w]);
        for (x, cell) in row.chunks_exact_mut(d).enumerate() {
            let c = lrow[x];
            // Disparities k whose right pixel x - dmin - k lies inside the image.
            let lo = (x as i64 - dmin as i64 - w as i64 + 1).clamp(0, d as i64) as usize;
            let hi = (x as i64 - dmin as i64 + 1).clamp(lo as i64, d as i64) as usize;
            cell[..lo].fill(OUT_OF_IMAGE_COST as u8);
            cell[hi..].fill(OUT_OF_IMAGE_COST as u8);
            if hi > lo {
                // k = lo maps to the rightmost right pixel; walk the right row backwards.
                let xr_top = (x as i64 - dmin as i64 - lo as i64) as usize;
                let rs = &rrow[xr_top + 1 - (hi - lo)..=xr_top];
                for (o, r) in cell[lo..hi].iter_mut().zip(rs.iter().rev()) {
                    *o = (c ^ r).count_ones() as u8;
                }
            }
        }
    });
}

/// One SGM recurrence step: `cur = cost + min(prev[d], prev[d±1] + p1, min(prev) + p2) - min(prev)`.
/// `prev` and `cur` hold `d + 2` values with `BIG` pads at both ends. Returns `min(cur)`.
#[inline(always)]
fn step(cost: &[u8], prev: &[u16], prev_min: u16, cur: &mut [u16], p1: u16, p2: u16) -> u16 {
    let n = cost.len();
    let lim = prev_min + p2;
    let (l, c, r) = (&prev[0..n], &prev[1..n + 1], &prev[2..n + 2]);
    let out = &mut cur[1..n + 1];
    let mut mn = u16::MAX;
    for k in 0..n {
        let m = c[k].min(l[k] + p1).min(r[k] + p1).min(lim);
        let v = cost[k] as u16 + m - prev_min;
        out[k] = v;
        mn = mn.min(v);
    }
    mn
}

fn fresh_state(d: usize) -> Vec<u16> {
    let mut v = vec![0u16; d + 2];
    v[0] = BIG;
    v[d + 1] = BIG;
    v
}

/// Left→right and right→left paths; initialises `sum`.
fn horizontal_pass(ctx: &Ctx, sum: &mut [u16]) {
    let (w, d) = (ctx.w, ctx.d);
    sum.par_chunks_mut(w * d).enumerate().for_each_init(
        || (fresh_state(d), fresh_state(d)),
        |(prev, cur), (y, srow)| {
            let cost = &ctx.cost[y * w * d..(y + 1) * w * d];
            for dir in 0..2 {
                prev[1..d + 1].fill(0);
                let mut prev_min = 0;
                for i in 0..w {
                    let x = if dir == 0 { i } else { w - 1 - i };
                    prev_min = step(&cost[x * d..(x + 1) * d], prev, prev_min, cur, ctx.p1, ctx.p2);
                    let s = &mut srow[x * d..(x + 1) * d];
                    if dir == 0 {
                        s.copy_from_slice(&cur[1..d + 1]);
                    } else {
                        for (a, b) in s.iter_mut().zip(&cur[1..d + 1]) {
                            *a += b;
                        }
                    }
                    std::mem::swap(prev, cur);
                }
            }
        },
    );
}

/// Raw pointer into `S` shared by the chain tasks of [`vertical_passes`], which write disjoint
/// pixels.
#[derive(Clone, Copy)]
struct SharedPtr(*mut u16);
unsafe impl Send for SharedPtr {}
unsafe impl Sync for SharedPtr {}

impl SharedPtr {
    /// # Safety
    /// No other thread may access `start..start + len` while the returned slice lives.
    #[allow(clippy::mut_from_ref)]
    unsafe fn slice_mut<'a>(self, start: usize, len: usize) -> &'a mut [u16] {
        std::slice::from_raw_parts_mut(self.0.add(start), len)
    }
}

/// Top→bottom and bottom→top paths: vertical, plus the two diagonals if `diagonals`.
///
/// Every such path runs along a chain of pixels that no other path of the same kind touches:
/// columns (`x = c`), diagonals (`x = c + y`) and anti-diagonals (`x = c - y`). Chains are
/// grouped and the groups run in parallel without any synchronisation; each group carries its
/// chains down and back up. The three path kinds run one after another, so two threads never
/// add into the same `S(x, y)` at the same time.
fn vertical_passes(ctx: &Ctx, sum: &mut [u16], diagonals: bool) {
    let (w, h) = (ctx.w as i64, ctx.h as i64);
    let sum_ptr = SharedPtr(sum.as_mut_ptr());
    let kinds: &[i64] = if diagonals { &[0, 1, -1] } else { &[0] };
    for &dx in kinds {
        // Chain c contains the pixels x = c + dx * y. Range of c that meets the image:
        let (c_lo, c_hi) = match dx {
            0 => (0, w),
            1 => (-(h - 1), w),
            _ => (0, w + h - 1),
        };
        const GROUP: i64 = 24;
        let groups: Vec<(i64, i64)> = (c_lo..c_hi).step_by(GROUP as usize).map(|c| (c, (c + GROUP).min(c_hi))).collect();
        groups.par_iter().for_each(|&(c0, c1)| run_chains(ctx, sum_ptr, dx, c0, c1));
    }
}

/// Aggregates chains `c0..c1` of slope `dx` (see [`vertical_passes`]) top→bottom, then
/// bottom→top, adding both into `S`.
fn run_chains(ctx: &Ctx, sum: SharedPtr, dx: i64, c0: i64, c1: i64) {
    let (w, h, d) = (ctx.w as i64, ctx.h, ctx.d);
    let stride = d + 2;
    let n = (c1 - c0) as usize;
    let mut prev = vec![0u16; n * stride];
    let mut cur = vec![0u16; n * stride];
    for v in [&mut prev, &mut cur] {
        for s in v.chunks_mut(stride) {
            s[0] = BIG;
            s[d + 1] = BIG;
        }
    }
    let mut prev_min = vec![0u16; n];
    let mut started = vec![false; n];
    let fresh = fresh_state(d);
    for down in [true, false] {
        started.fill(false);
        for i in 0..h {
            let y = if down { i } else { h - 1 - i };
            for j in 0..n {
                let x = c0 + j as i64 + dx * y as i64;
                if x < 0 || x >= w {
                    continue;
                }
                let x = x as usize;
                let cost = ctx.costs(x, y);
                let state = &mut cur[j * stride..(j + 1) * stride];
                let (pstate, pmin) = if started[j] { (&prev[j * stride..(j + 1) * stride], prev_min[j]) } else { (&fresh[..], 0) };
                let m = step(cost, pstate, pmin, state, ctx.p1, ctx.p2);
                prev_min[j] = m;
                started[j] = true;
                // SAFETY: pixel (x, y) lies on chain c0 + j, which only this task processes in
                // this phase, so nobody else accesses S(x, y) concurrently.
                let s = unsafe { sum.slice_mut((y * ctx.w + x) * d, d) };
                for (a, b) in s.iter_mut().zip(&state[1..d + 1]) {
                    *a += b;
                }
            }
            std::mem::swap(&mut prev, &mut cur);
        }
    }
}

/// Winner-takes-all + uniqueness + sub-pixel + left-right check.
fn select_disparities(ctx: &Ctx, sum: &[u16], params: &SgmParams) -> DisparityImage {
    let (w, h, d) = (ctx.w, ctx.h, ctx.d);
    let dmin = ctx.dmin as i64;
    let mut data = vec![f32::NAN; w * h];
    data.par_chunks_mut(w).enumerate().for_each(|(y, out)| {
        let srow = &sum[y * w * d..(y + 1) * w * d];
        // Best disparity per *right* pixel, from the same cost sum (approximate right-image SGM).
        let mut right_cost = vec![u16::MAX; w];
        let mut right_disp = vec![usize::MAX; w];
        let mut best_disp = vec![usize::MAX; w];
        for x in 0..w {
            // Disparities whose right pixel x - dmin - k lies inside the image.
            let lo = (x as i64 - dmin - w as i64 + 1).max(0) as usize;
            let hi = ((x as i64 - dmin + 1).min(d as i64)).max(0) as usize;
            if lo >= hi {
                continue;
            }
            let s = &srow[x * d..(x + 1) * d];
            let mut best = lo;
            for k in lo..hi {
                if s[k] < s[best] {
                    best = k;
                }
                let xr = (x as i64 - dmin - k as i64) as usize;
                if s[k] < right_cost[xr] {
                    right_cost[xr] = s[k];
                    right_disp[xr] = k;
                }
            }
            let second = (lo..hi).filter(|&k| k + 1 < best || k > best + 1).map(|k| s[k]).min();
            if let Some(second) = second {
                // `>=` so that ties (e.g. no texture: all costs equal) are rejected.
                if params.uniqueness < 1.0 && s[best] as f32 >= params.uniqueness * second as f32 {
                    continue;
                }
            }
            best_disp[x] = best;
        }
        for x in 0..w {
            let best = best_disp[x];
            if best == usize::MAX {
                continue;
            }
            if params.lr_max_diff >= 0.0 {
                let xr = (x as i64 - dmin - best as i64) as usize;
                let rd = right_disp[xr];
                if rd == usize::MAX || (rd as f32 - best as f32).abs() > params.lr_max_diff {
                    continue;
                }
            }
            let s = &srow[x * d..(x + 1) * d];
            let mut sub = 0.0;
            if best > 0 && best + 1 < d {
                let (a, b, c) = (s[best - 1] as f32, s[best] as f32, s[best + 1] as f32);
                let denom = a - 2.0 * b + c;
                if denom > 0.0 {
                    sub = (0.5 * (a - c) / denom).clamp(-0.5, 0.5);
                }
            }
            out[x] = (dmin + best as i64) as f32 + sub;
        }
    });
    DisparityImage { width: w, height: h, data }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;

    /// Deterministic speckle-like texture.
    pub(crate) fn texture(w: usize, h: usize, seed: u32) -> Vec<u8> {
        let mut s = seed;
        let mut v = vec![0u8; w * h];
        for p in v.iter_mut() {
            s ^= s << 13;
            s ^= s >> 17;
            s ^= s << 5;
            *p = if s % 5 == 0 { 220 } else { 30 + (s % 40) as u8 };
        }
        // Blur a little so dots span more than one pixel, like real speckle.
        let mut out = v.clone();
        for y in 1..h - 1 {
            for x in 1..w - 1 {
                let sum: u32 = [v[y * w + x], v[y * w + x - 1], v[y * w + x + 1], v[(y - 1) * w + x], v[(y + 1) * w + x]]
                    .iter()
                    .map(|&a| a as u32)
                    .sum();
                out[y * w + x] = (sum / 5) as u8;
            }
        }
        out
    }

    /// Right image of a scene whose left image is `left` and whose disparity per left pixel is
    /// `disp(x, y)` (right x = left x - disp), rendered by forward warping with linear
    /// interpolation along the row.
    pub(crate) fn render_right(left: &[u8], w: usize, h: usize, disp: impl Fn(usize, usize) -> f32) -> Vec<u8> {
        let mut right = vec![0u8; w * h];
        for y in 0..h {
            for xr in 0..w {
                // Find left x with x - disp(x) == xr by scanning (disparity varies slowly).
                let mut val = 0.0;
                for x in 0..w - 1 {
                    let a = x as f32 - disp(x, y);
                    let b = (x + 1) as f32 - disp(x + 1, y);
                    if a <= xr as f32 && (xr as f32) < b {
                        let t = (xr as f32 - a) / (b - a);
                        val = left[y * w + x] as f32 * (1.0 - t) + left[y * w + x + 1] as f32 * t;
                        break;
                    }
                }
                right[y * w + xr] = val.round() as u8;
            }
        }
        right
    }

    #[test]
    fn recovers_constant_and_sloped_disparity_with_subpixel_accuracy() {
        let (w, h) = (160, 64);
        let left = texture(w, h, 12345);
        // A slanted plane: disparity 20.0 .. 27.9 across the image (sub-pixel everywhere).
        let disp_of = |x: usize, _y: usize| 20.0 + x as f32 * 0.05;
        let right = render_right(&left, w, h, disp_of);
        let mut m = SgmMatcher::new(SgmParams { min_disparity: 10, num_disparities: 32, ..Default::default() });
        let out = m.compute(&left, &right, w, h);

        let mut errs = Vec::new();
        for y in 8..h - 8 {
            for x in 40..w - 8 {
                let v = out.data[y * w + x];
                if !v.is_nan() {
                    errs.push((v - disp_of(x, y)).abs());
                }
            }
        }
        let n_area = (h - 16) * (w - 48);
        assert!(errs.len() as f32 > 0.9 * n_area as f32, "coverage {} of {n_area}", errs.len());
        let mean = errs.iter().sum::<f32>() / errs.len() as f32;
        let bad = errs.iter().filter(|&&e| e > 1.0).count();
        assert!(mean < 0.25, "mean abs error {mean}");
        assert!((bad as f32) < 0.01 * errs.len() as f32, "{bad} gross errors");
    }

    #[test]
    fn handles_negative_disparities() {
        let (w, h) = (120, 40);
        let left = texture(w, h, 99);
        let right = render_right(&left, w, h, |_, _| -15.0);
        let mut m = SgmMatcher::new(SgmParams { min_disparity: -30, num_disparities: 32, diagonal_paths: false, ..Default::default() });
        let out = m.compute(&left, &right, w, h);
        let centre = out.data[20 * w + 50];
        assert!((centre + 15.0).abs() < 0.3, "{centre}");
    }

    #[test]
    fn untextured_regions_are_invalid() {
        let (w, h) = (100, 40);
        let flat = vec![80u8; w * h];
        let mut m = SgmMatcher::new(SgmParams { min_disparity: 0, num_disparities: 16, ..Default::default() });
        let out = m.compute(&flat, &flat, w, h);
        assert!(out.valid_count() < w * h / 20, "{} valid on a flat image", out.valid_count());
    }
}
