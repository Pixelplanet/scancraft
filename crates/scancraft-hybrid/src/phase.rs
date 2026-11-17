//! N-step phase shifting: wrapped phase and modulation per pixel, and sub-pixel correspondence
//! along a rectified row by equal phase.

use scancraft_core::RgbImage;
use std::f32::consts::PI;

/// Wrapped phase of a set of phase-shifted fringe images.
#[derive(Debug, Clone)]
pub struct PhaseMap {
    pub width: usize,
    pub height: usize,
    /// Wrapped phase in (-π, π].
    pub phase: Vec<f32>,
    /// Fringe amplitude `B` (grey levels); low where the projector does not reach.
    pub modulation: Vec<f32>,
}

impl PhaseMap {
    /// `images[k]` lit with the pattern shifted by `k * 2π / N` (`I_k = A + B cos(φ + 2πk/N)`).
    /// For N = 4 this is `φ = atan2(I3 - I1, I0 - I2)`.
    pub fn from_images(images: &[RgbImage]) -> Option<Self> {
        let n = images.len();
        let first = images.first()?;
        if n < 3 || images.iter().any(|i| i.width != first.width || i.height != first.height || i.channels != 1) {
            return None;
        }
        let (width, height) = (first.width as usize, first.height as usize);
        let basis: Vec<(f32, f32)> = (0..n).map(|k| ((2.0 * PI * k as f32 / n as f32).cos(), (2.0 * PI * k as f32 / n as f32).sin())).collect();
        let mut phase = vec![0.0; width * height];
        let mut modulation = vec![0.0; width * height];
        for i in 0..width * height {
            let (mut re, mut im) = (0.0f32, 0.0f32);
            for (img, &(c, s)) in images.iter().zip(&basis) {
                let v = img.data[i] as f32;
                re += v * c;
                im -= v * s;
            }
            phase[i] = im.atan2(re);
            modulation[i] = 2.0 / n as f32 * (re * re + im * im).sqrt();
        }
        Some(Self { width, height, phase, modulation })
    }
}

/// Wraps an angle difference to (-π, π].
#[inline]
pub fn wrap(a: f32) -> f32 {
    let mut a = a;
    while a > PI {
        a -= 2.0 * PI;
    }
    while a <= -PI {
        a += 2.0 * PI;
    }
    a
}

/// Sub-pixel right-image x on row `y` whose phase equals `target`, searched in
/// `[x_lo, x_hi]`; the crossing nearest to `x_guess` wins. Pixels below `min_modulation`
/// are not used.
pub fn find_equal_phase(right: &PhaseMap, y: usize, target: f32, x_guess: f32, x_lo: isize, x_hi: isize, min_modulation: f32) -> Option<f32> {
    let row = y * right.width;
    let lo = x_lo.max(0) as usize;
    let hi = (x_hi.max(0) as usize).min(right.width.saturating_sub(2));
    let mut best: Option<f32> = None;
    for x in lo..=hi {
        if right.modulation[row + x] < min_modulation || right.modulation[row + x + 1] < min_modulation {
            continue;
        }
        let a = wrap(right.phase[row + x] - target);
        let b = wrap(right.phase[row + x + 1] - target);
        // A genuine crossing changes sign by less than π (a wrap jumps by ~2π).
        if (a == 0.0 || (a > 0.0) != (b > 0.0)) && (a - b).abs() < PI {
            let xf = if a == b { x as f32 } else { x as f32 + a / (a - b) };
            if best.is_none_or(|bx| (xf - x_guess).abs() < (bx - x_guess).abs()) {
                best = Some(xf);
            }
        }
    }
    best
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fringes(width: u32, period: f32, offset: f32) -> Vec<RgbImage> {
        (0..4)
            .map(|k| RgbImage {
                width,
                height: 1,
                channels: 1,
                data: (0..width)
                    .map(|x| (100.0 + 60.0 * (-2.0 * PI * (x as f32 - offset) / period + PI / 2.0 * k as f32).cos()).round() as u8)
                    .collect(),
            })
            .collect()
    }

    #[test]
    fn recovers_phase_and_subpixel_shift() {
        let left = PhaseMap::from_images(&fringes(200, 13.7, 0.0)).unwrap();
        let right = PhaseMap::from_images(&fringes(200, 13.7, -37.3)).unwrap();
        assert!((left.modulation[50] - 60.0).abs() < 1.5);
        // Right pattern is the left one shifted by 37.3 px to the left: x_r = x_l - 37.3.
        for x in [60usize, 81, 120] {
            let xr = find_equal_phase(&right, 0, left.phase[x], x as f32 - 37.0, x as isize - 40, x as isize - 34, 5.0).unwrap();
            assert!((x as f32 - xr - 37.3).abs() < 0.05, "x {x}: {xr}");
        }
    }
}
