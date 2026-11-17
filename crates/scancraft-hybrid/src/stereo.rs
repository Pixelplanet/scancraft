//! Block matching on a rectified pair: windowed ZNCC over a disparity sweep, optionally joined
//! with a fringe-phase agreement term. Self-contained stand-in for the shared host stereo
//! matcher (`scancraft-stereo`, VCSEL work package); [`match_pair`] is the boundary to swap.

use crate::phase::PhaseMap;
use rayon::prelude::*;
use scancraft_core::RgbImage;

/// Image normalised to zero mean / unit deviation over a `(2r+1)²` window around each pixel.
pub struct Normalized {
    pub width: usize,
    pub height: usize,
    pub data: Vec<f32>,
    /// Local standard deviation (grey levels); low = no texture.
    pub std: Vec<f32>,
}

impl Normalized {
    pub fn new(img: &RgbImage, radius: usize) -> Self {
        let (w, h) = (img.width as usize, img.height as usize);
        // Integral images of I and I² (f64: 640x512x255² overflows f32 precision).
        let mut s1 = vec![0f64; (w + 1) * (h + 1)];
        let mut s2 = vec![0f64; (w + 1) * (h + 1)];
        for y in 0..h {
            let (mut r1, mut r2) = (0f64, 0f64);
            for x in 0..w {
                let v = img.data[y * w + x] as f64;
                r1 += v;
                r2 += v * v;
                s1[(y + 1) * (w + 1) + x + 1] = s1[y * (w + 1) + x + 1] + r1;
                s2[(y + 1) * (w + 1) + x + 1] = s2[y * (w + 1) + x + 1] + r2;
            }
        }
        let mut data = vec![0f32; w * h];
        let mut std = vec![0f32; w * h];
        for y in 0..h {
            let (y0, y1) = (y.saturating_sub(radius), (y + radius + 1).min(h));
            for x in 0..w {
                let (x0, x1) = (x.saturating_sub(radius), (x + radius + 1).min(w));
                let n = ((y1 - y0) * (x1 - x0)) as f64;
                let rect = |s: &[f64]| s[y1 * (w + 1) + x1] - s[y0 * (w + 1) + x1] - s[y1 * (w + 1) + x0] + s[y0 * (w + 1) + x0];
                let mean = rect(&s1) / n;
                let var = (rect(&s2) / n - mean * mean).max(0.0);
                let sd = var.sqrt() as f32;
                std[y * w + x] = sd;
                data[y * w + x] = (img.data[y * w + x] as f32 - mean as f32) / sd.max(1.0);
            }
        }
        Self { width: w, height: h, data, std }
    }
}

/// `cos φ`, `sin φ` per pixel, zero where the fringe modulation is too low to trust.
pub struct PhaseTerms {
    pub cos: Vec<f32>,
    pub sin: Vec<f32>,
}

impl PhaseTerms {
    pub fn new(p: &PhaseMap, min_modulation: f32) -> Self {
        let (cos, sin) = p
            .phase
            .iter()
            .zip(&p.modulation)
            .map(|(&ph, &m)| if m >= min_modulation { (ph.cos(), ph.sin()) } else { (0.0, 0.0) })
            .unzip();
        Self { cos, sin }
    }
}

/// Winner-take-all result of a disparity sweep.
pub struct Sweep {
    pub width: usize,
    pub height: usize,
    /// Best integer disparity `x_left - x_right`, `i32::MIN` where no disparity was evaluated.
    pub disparity: Vec<i32>,
    pub score: Vec<f32>,
    /// Best score among disparities more than one pixel away from the winner.
    pub second: Vec<f32>,
}

/// Inputs of a sweep: normalised speckle pair and, optionally, the fringe phase of both views.
pub struct SweepInput<'a> {
    pub left: &'a Normalized,
    pub right: &'a Normalized,
    pub phase: Option<(&'a PhaseTerms, &'a PhaseTerms)>,
    /// Weight of the phase-agreement term relative to ZNCC.
    pub phase_weight: f32,
    pub radius: usize,
    pub d_min: i32,
    pub d_max: i32,
}

const BAND: usize = 16;

/// Disparity sweep: score(d) = (ZNCC + λ·mean cos(φL − φR)) / (1 + λ) over the window.
pub fn sweep(input: &SweepInput) -> Sweep {
    let (w, h) = (input.left.width, input.left.height);
    let bands: Vec<(Vec<i32>, Vec<f32>, Vec<f32>)> = (0..h.div_ceil(BAND)).into_par_iter().map(|b| sweep_band(input, b * BAND, ((b + 1) * BAND).min(h))).collect();
    let mut out = Sweep { width: w, height: h, disparity: Vec::with_capacity(w * h), score: Vec::with_capacity(w * h), second: Vec::with_capacity(w * h) };
    for (d, s, s2) in bands {
        out.disparity.extend(d);
        out.score.extend(s);
        out.second.extend(s2);
    }
    out
}

fn sweep_band(input: &SweepInput, y0: usize, y1: usize) -> (Vec<i32>, Vec<f32>, Vec<f32>) {
    let (w, h, r) = (input.left.width, input.left.height, input.radius);
    let (ya, yb) = (y0.saturating_sub(r), (y1 + r).min(h));
    let rows = yb - ya;
    let n = (y1 - y0) * w;
    let mut best_d = vec![i32::MIN; n];
    let mut best = vec![f32::NEG_INFINITY; n];
    let mut second = vec![f32::NEG_INFINITY; n];
    let mut hsum = vec![0f32; rows * w];
    let mut prod = vec![0f32; w];
    let area = ((2 * r + 1) * (2 * r + 1)) as f32;
    let norm = 1.0 / (area * (1.0 + if input.phase.is_some() { input.phase_weight } else { 0.0 }));
    let (ln, rn) = (&input.left.data, &input.right.data);

    for d in input.d_min..=input.d_max {
        // Valid left x: window inside both images.
        let x_lo = (r as i32).max(d + r as i32);
        let x_hi = (w as i32 - 1 - r as i32).min(w as i32 - 1 - r as i32 + d);
        if x_lo > x_hi {
            continue;
        }
        for (ri, yy) in (ya..yb).enumerate() {
            let row = yy * w;
            for x in 0..w {
                let xr = x as i32 - d;
                prod[x] = if xr >= 0 && (xr as usize) < w {
                    let (i, j) = (row + x, row + xr as usize);
                    let mut p = ln[i] * rn[j];
                    if let Some((pl, pr)) = input.phase {
                        p += input.phase_weight * (pl.cos[i] * pr.cos[j] + pl.sin[i] * pr.sin[j]);
                    }
                    p
                } else {
                    0.0
                };
            }
            // Horizontal running window sum.
            let out = &mut hsum[ri * w..(ri + 1) * w];
            let mut acc: f32 = prod[..(2 * r + 1).min(w)].iter().sum();
            for x in r..w.saturating_sub(r) {
                out[x] = acc;
                if x + r + 1 < w {
                    acc += prod[x + r + 1] - prod[x - r];
                }
            }
        }
        for y in y0..y1 {
            if y < r || y + r >= h {
                continue;
            }
            let (top, bottom) = (y - r - ya, y + r - ya);
            for x in x_lo as usize..=x_hi as usize {
                let mut acc = 0.0;
                for ri in top..=bottom {
                    acc += hsum[ri * w + x];
                }
                let s = acc * norm;
                let i = (y - y0) * w + x;
                let far = best_d[i] == i32::MIN || (d - best_d[i]).abs() > 1;
                if s > best[i] {
                    if far && best[i] > second[i] {
                        second[i] = best[i];
                    }
                    best[i] = s;
                    best_d[i] = d;
                } else if far && s > second[i] {
                    second[i] = s;
                }
            }
        }
    }
    (best_d, best, second)
}

/// Windowed score of one disparity at one pixel (same measure as [`sweep`]), for sub-pixel fits.
pub fn score_at(input: &SweepInput, x: usize, y: usize, d: i32) -> Option<f32> {
    let (w, h, r) = (input.left.width, input.left.height, input.radius as i32);
    let (x, y) = (x as i32, y as i32);
    if x - r < 0 || x + r >= w as i32 || y - r < 0 || y + r >= h as i32 || x - d - r < 0 || x - d + r >= w as i32 {
        return None;
    }
    let mut acc = 0.0;
    for yy in y - r..=y + r {
        for xx in x - r..=x + r {
            let (i, j) = ((yy * w as i32 + xx) as usize, (yy * w as i32 + xx - d) as usize);
            acc += input.left.data[i] * input.right.data[j];
            if let Some((pl, pr)) = input.phase {
                acc += input.phase_weight * (pl.cos[i] * pr.cos[j] + pl.sin[i] * pr.sin[j]);
            }
        }
    }
    let area = ((2 * r + 1) * (2 * r + 1)) as f32;
    Some(acc / (area * (1.0 + if input.phase.is_some() { input.phase_weight } else { 0.0 })))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Pseudo-random dot texture (deterministic LCG).
    pub(crate) fn texture(w: u32, h: u32, seed: u32) -> RgbImage {
        let mut s = seed;
        let data = (0..w * h)
            .map(|_| {
                s = s.wrapping_mul(1664525).wrapping_add(1013904223);
                if (s >> 24) > 200 {
                    230
                } else {
                    30 + (s >> 28) as u8
                }
            })
            .collect();
        RgbImage { width: w, height: h, channels: 1, data }
    }

    #[test]
    fn finds_constant_shift() {
        let (w, h) = (160u32, 48u32);
        let left = texture(w, h, 7);
        // right(x) = left(x + 23): a point at left x appears at x - 23 in the right image.
        let mut right = left.clone();
        for y in 0..h as usize {
            for x in 0..w as usize {
                right.data[y * w as usize + x] = if x + 23 < w as usize { left.data[y * w as usize + x + 23] } else { 0 };
            }
        }
        let (ln, rn) = (Normalized::new(&left, 4), Normalized::new(&right, 4));
        let input = SweepInput { left: &ln, right: &rn, phase: None, phase_weight: 0.0, radius: 4, d_min: -10, d_max: 40 };
        let s = sweep(&input);
        let i = 24 * w as usize + 60;
        assert_eq!(s.disparity[i], 23);
        assert!(s.score[i] > 0.9 && s.second[i] < 0.6, "{} {}", s.score[i], s.second[i]);
        assert!((score_at(&input, 60, 24, 23).unwrap() - s.score[i]).abs() < 1e-4);
    }
}
