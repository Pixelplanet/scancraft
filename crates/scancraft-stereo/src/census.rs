//! Census transform: each pixel becomes a bit string of "neighbour < centre" comparisons over a
//! window. Hamming distance between bit strings is a matching cost that is robust to the gain and
//! exposure differences between the two IR cameras.

use rayon::prelude::*;

/// Census window half sizes: 9x7 (62 comparisons, fits a u64).
pub const HALF_W: usize = 4;
pub const HALF_H: usize = 3;

/// Census transform of a `width` x `height` 8-bit image. Border pixels (where the window leaves
/// the image) get 0, which makes them match poorly against textured pixels.
pub fn census_9x7(img: &[u8], width: usize, height: usize, out: &mut Vec<u64>) {
    out.clear();
    out.resize(width * height, 0);
    if width <= 2 * HALF_W || height <= 2 * HALF_H {
        return;
    }
    out.par_chunks_mut(width).enumerate().skip(HALF_H).take(height - 2 * HALF_H).for_each(|(y, row)| {
        for (x, cell) in row.iter_mut().enumerate().skip(HALF_W).take(width - 2 * HALF_W) {
            let centre = img[y * width + x];
            let mut bits = 0u64;
            for dy in 0..=2 * HALF_H {
                let base = (y + dy - HALF_H) * width + x - HALF_W;
                for &v in &img[base..base + 2 * HALF_W + 1] {
                    bits = (bits << 1) | (v < centre) as u64;
                }
            }
            *cell = bits;
        }
    });
}

/// Marks pixels whose 9x7 window has an intensity standard deviation of at least `min_std`
/// grey levels. Untextured pixels have no reliable match (all candidates cost the same, and the
/// SGM smoothness term then decides alone).
pub fn texture_mask(img: &[u8], width: usize, height: usize, min_std: f32, out: &mut Vec<bool>) {
    out.clear();
    out.resize(width * height, false);
    if width <= 2 * HALF_W || height <= 2 * HALF_H {
        return;
    }
    let n = ((2 * HALF_W + 1) * (2 * HALF_H + 1)) as f32;
    let min_var_n2 = min_std * min_std * n * n;
    out.par_chunks_mut(width).enumerate().skip(HALF_H).take(height - 2 * HALF_H).for_each(|(y, row)| {
        for (x, cell) in row.iter_mut().enumerate().skip(HALF_W).take(width - 2 * HALF_W) {
            let (mut s, mut s2) = (0u32, 0u32);
            for dy in 0..=2 * HALF_H {
                let base = (y + dy - HALF_H) * width + x - HALF_W;
                for &v in &img[base..base + 2 * HALF_W + 1] {
                    s += v as u32;
                    s2 += v as u32 * v as u32;
                }
            }
            // n * sum(v^2) - sum(v)^2 = n^2 * variance
            let var_n2 = n * s2 as f32 - (s as f32) * (s as f32);
            *cell = var_n2 >= min_var_n2;
        }
    });
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn census_is_invariant_to_gain_and_offset() {
        let (w, h) = (20, 12);
        let img: Vec<u8> = (0..w * h).map(|i| ((i * 37) % 97) as u8).collect();
        let bright: Vec<u8> = img.iter().map(|&v| v * 2 + 10).collect();
        let (mut a, mut b) = (Vec::new(), Vec::new());
        census_9x7(&img, w, h, &mut a);
        census_9x7(&bright, w, h, &mut b);
        assert_eq!(a, b);
        assert_eq!(a[0], 0, "border is zero");
        assert_ne!(a[6 * w + 10], 0);
    }
}
