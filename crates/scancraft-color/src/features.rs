//! ORB-style image features: FAST-9 corners ranked by Harris response, spread over a grid,
//! oriented by intensity centroid, described with 256-bit steered BRIEF; brute-force Hamming
//! matching with ratio and mutual-best checks.

use crate::image_ops::{blur3, Gray};
use glam::Vec2;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Feature {
    /// Pixel position in the image it was detected in.
    pub pos: Vec2,
    pub angle: f32,
    pub score: f32,
    pub descriptor: [u64; 4],
}

#[derive(Debug, Clone)]
pub struct FeatureConfig {
    pub fast_threshold: u8,
    pub max_features: usize,
    /// Features are spread over a grid of cells this many pixels wide.
    pub cell_px: usize,
}

impl Default for FeatureConfig {
    fn default() -> Self {
        Self { fast_threshold: 12, max_features: 1200, cell_px: 32 }
    }
}

/// Bresenham circle of radius 3 (FAST-16), clockwise from 12 o'clock.
const CIRCLE: [(i32, i32); 16] =
    [(0, -3), (1, -3), (2, -2), (3, -1), (3, 0), (3, 1), (2, 2), (1, 3), (0, 3), (-1, 3), (-2, 2), (-3, 1), (-3, 0), (-3, -1), (-2, -2), (-1, -3)];

const BORDER: usize = 20;
const PATCH_RADIUS: i32 = 15;

/// FAST-9 score (sum of the excess over the threshold on the contiguous arc), 0 if not a corner.
fn fast_score(img: &Gray, x: usize, y: usize, t: i16) -> f32 {
    let w = img.width as i32;
    let c = img.data[y * img.width + x] as i16;
    let base = (y * img.width + x) as i32;
    let px = |k: usize| img.data[(base + CIRCLE[k].1 * w + CIRCLE[k].0) as usize] as i16;
    // Quick rejection: at least 2 of the 4 compass points must be beyond the threshold on one side.
    let compass = [px(0), px(4), px(8), px(12)];
    let bright = compass.iter().filter(|&&p| p > c + t).count();
    let dark = compass.iter().filter(|&&p| p < c - t).count();
    if bright < 2 && dark < 2 {
        return 0.0;
    }
    let ring: [i16; 16] = std::array::from_fn(px);
    let mut best = 0f32;
    for sign in [1i16, -1] {
        let mut run = 0;
        let mut sum = 0f32;
        for k in 0..25 {
            let d = sign * (ring[k % 16] - c) - t;
            if d > 0 {
                run += 1;
                sum += d as f32;
                if run >= 9 {
                    best = best.max(sum);
                }
            } else {
                run = 0;
                sum = 0.0;
            }
        }
    }
    best
}

/// Harris corner response over a 7x7 window.
fn harris(img: &Gray, x: usize, y: usize) -> f32 {
    let (mut a, mut b, mut c) = (0f32, 0f32, 0f32);
    let w = img.width;
    for dy in -3i32..=3 {
        for dx in -3i32..=3 {
            let i = ((y as i32 + dy) as usize) * w + (x as i32 + dx) as usize;
            let ix = img.data[i + 1] as f32 - img.data[i - 1] as f32;
            let iy = img.data[i + w] as f32 - img.data[i - w] as f32;
            a += ix * ix;
            b += iy * iy;
            c += ix * iy;
        }
    }
    a * b - c * c - 0.04 * (a + b) * (a + b)
}

/// Orientation from the intensity centroid of the circular patch.
fn orientation(img: &Gray, x: usize, y: usize) -> f32 {
    let (mut m10, mut m01) = (0f32, 0f32);
    for dy in -PATCH_RADIUS..=PATCH_RADIUS {
        let span = ((PATCH_RADIUS * PATCH_RADIUS - dy * dy) as f32).sqrt() as i32;
        for dx in -span..=span {
            let v = img.at((x as i32 + dx) as usize, (y as i32 + dy) as usize) as f32;
            m10 += dx as f32 * v;
            m01 += dy as f32 * v;
        }
    }
    m01.atan2(m10)
}

/// 256 point pairs within the patch, fixed pseudo-random (deterministic) and roughly Gaussian.
fn pattern() -> &'static [[(f32, f32); 2]; 256] {
    use std::sync::OnceLock;
    static PATTERN: OnceLock<[[(f32, f32); 2]; 256]> = OnceLock::new();
    PATTERN.get_or_init(|| {
        let mut state = 0x2545_f491_4f6c_dd1du64;
        let mut uniform = move || {
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            (state >> 11) as f32 / (1u64 << 53) as f32
        };
        let mut gauss = move || {
            // Irwin-Hall approximation, sigma ~ patch/5, clamped into the patch.
            let s: f32 = (0..4).map(|_| uniform()).sum::<f32>() - 2.0;
            (s * 1.7 * 31.0 / 5.0).clamp(-12.5, 12.5)
        };
        std::array::from_fn(|_| [(gauss(), gauss()), (gauss(), gauss())])
    })
}

fn describe(smooth: &Gray, pos: Vec2, angle: f32) -> [u64; 4] {
    let (s, c) = angle.sin_cos();
    let mut d = [0u64; 4];
    let (x0, y0) = (pos.x, pos.y);
    for (i, pair) in pattern().iter().enumerate() {
        let sample = |(px, py): (f32, f32)| {
            let x = (x0 + c * px - s * py).round() as usize;
            let y = (y0 + s * px + c * py).round() as usize;
            smooth.at(x, y)
        };
        if sample(pair[0]) < sample(pair[1]) {
            d[i / 64] |= 1 << (i % 64);
        }
    }
    d
}

/// Detects and describes features.
pub fn detect(img: &Gray, config: &FeatureConfig) -> Vec<Feature> {
    let (w, h) = (img.width, img.height);
    if w <= 2 * BORDER || h <= 2 * BORDER {
        return Vec::new();
    }
    let t = config.fast_threshold as i16;
    let mut scores = vec![0f32; w * h];
    for y in BORDER..h - BORDER {
        for x in BORDER..w - BORDER {
            scores[y * w + x] = fast_score(img, x, y, t);
        }
    }
    // 3x3 non-maximum suppression, then best Harris response per grid cell.
    let cells_x = w.div_ceil(config.cell_px);
    let cells_y = h.div_ceil(config.cell_px);
    let mut per_cell: Vec<Vec<(f32, usize, usize)>> = vec![Vec::new(); cells_x * cells_y];
    for y in BORDER..h - BORDER {
        for x in BORDER..w - BORDER {
            let s = scores[y * w + x];
            if s <= 0.0 {
                continue;
            }
            let is_max = (-1i32..=1).all(|dy| {
                (-1i32..=1).all(|dx| {
                    let o = scores[((y as i32 + dy) as usize) * w + (x as i32 + dx) as usize];
                    (dx == 0 && dy == 0) || o < s || (o == s && (dy, dx) > (0, 0))
                })
            });
            if is_max {
                per_cell[(y / config.cell_px) * cells_x + x / config.cell_px].push((harris(img, x, y), x, y));
            }
        }
    }
    let per_cell_quota = (config.max_features / (cells_x * cells_y)).max(1) + 1;
    let mut chosen: Vec<(f32, usize, usize)> = Vec::new();
    for cell in per_cell.iter_mut() {
        cell.sort_by(|a, b| b.0.total_cmp(&a.0));
        chosen.extend(cell.iter().take(per_cell_quota).filter(|c| c.0 > 0.0));
    }
    chosen.sort_by(|a, b| b.0.total_cmp(&a.0));
    chosen.truncate(config.max_features);

    let smooth = blur3(&blur3(img));
    chosen
        .into_iter()
        .map(|(score, x, y)| {
            let angle = orientation(&smooth, x, y);
            let pos = Vec2::new(x as f32, y as f32);
            Feature { pos, angle, score, descriptor: describe(&smooth, pos, angle) }
        })
        .collect()
}

#[inline]
pub fn hamming(a: &[u64; 4], b: &[u64; 4]) -> u32 {
    (a[0] ^ b[0]).count_ones() + (a[1] ^ b[1]).count_ones() + (a[2] ^ b[2]).count_ones() + (a[3] ^ b[3]).count_ones()
}

/// Index pairs `(query, train)` of mutual best matches with distance <= `max_distance` and
/// best/second-best ratio below `ratio`. `allowed(query, train)` can restrict candidates (e.g.
/// to a search window around a predicted position).
pub fn match_features(
    query: &[Feature],
    train: &[Feature],
    max_distance: u32,
    ratio: f32,
    allowed: impl Fn(usize, usize) -> bool + Sync,
) -> Vec<(usize, usize, u32)> {
    use rayon::prelude::*;
    let best_for = |a: &[Feature], b: &[Feature], swap: bool| -> Vec<Option<(usize, u32)>> {
        a.par_iter()
            .enumerate()
            .map(|(i, fa)| {
                let (mut best, mut second, mut best_j) = (u32::MAX, u32::MAX, usize::MAX);
                for (j, fb) in b.iter().enumerate() {
                    let ok = if swap { allowed(j, i) } else { allowed(i, j) };
                    if !ok {
                        continue;
                    }
                    let d = hamming(&fa.descriptor, &fb.descriptor);
                    if d < best {
                        second = best;
                        best = d;
                        best_j = j;
                    } else if d < second {
                        second = d;
                    }
                }
                (best <= max_distance && (second == u32::MAX || (best as f32) < ratio * second as f32)).then_some((best_j, best))
            })
            .collect()
    };
    let forward = best_for(query, train, false);
    let backward = best_for(train, query, true);
    forward
        .iter()
        .enumerate()
        .filter_map(|(i, m)| {
            let (j, d) = (*m)?;
            (backward[j].map(|(bi, _)| bi) == Some(i)).then_some((i, j, d))
        })
        .collect()
}

/// A deterministic textured test image (Gaussian blobs of random size and contrast), for tests
/// and evaluations.
pub fn blob_texture(w: usize, h: usize, seed: u64) -> Gray {
    let mut img = Gray::new(w, h);
    let mut state = seed | 1;
    let mut rnd = move || {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        state
    };
    let blobs: Vec<(f32, f32, f32, f32)> = (0..(w * h / 300))
        .map(|_| {
            ((rnd() % w as u64) as f32, (rnd() % h as u64) as f32, 2.0 + (rnd() % 9) as f32, (rnd() % 200) as f32 - 100.0)
        })
        .collect();
    for y in 0..h {
        for x in 0..w {
            let mut v = 128.0;
            for &(bx, by, r, a) in &blobs {
                let d2 = (x as f32 - bx).powi(2) + (y as f32 - by).powi(2);
                if d2 < r * r * 4.0 {
                    v += a * (-d2 / (r * r)).exp();
                }
            }
            img.data[y * w + x] = v.clamp(0.0, 255.0) as u8;
        }
    }
    img
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;

    #[test]
    fn matches_a_shifted_image() {
        let big = blob_texture(360, 280, 7);
        let crop = |dx: usize, dy: usize| {
            let mut g = Gray::new(320, 240);
            for y in 0..240 {
                for x in 0..320 {
                    g.data[y * 320 + x] = big.at(x + dx, y + dy);
                }
            }
            g
        };
        let (a, b) = (crop(10, 10), crop(17, 14));
        let config = FeatureConfig::default();
        let fa = detect(&a, &config);
        let fb = detect(&b, &config);
        assert!(fa.len() > 100, "{} features", fa.len());
        let matches = match_features(&fa, &fb, 64, 0.85, |_, _| true);
        let good = matches.iter().filter(|(i, j, _)| (fa[*i].pos - fb[*j].pos - Vec2::new(7.0, 4.0)).length() < 1.5).count();
        assert!(matches.len() > 50 && good as f32 > 0.9 * matches.len() as f32, "{good} of {} matches correct", matches.len());
    }
}
