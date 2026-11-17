//! Retro-reflective dot detection in one 8-bit IR image.
//!
//! A marker is a compact bright disc (an ellipse under perspective) on a darker surround. The
//! detector works on local contrast rather than an absolute level, so it handles saturated dots
//! under the IR fill LED as well as unsaturated ones in the Full-field HD texture image:
//!
//! 1. candidates: pixels brighter than a large-window local mean by `min_contrast`;
//! 2. connected components (8-neighbourhood) of the candidates;
//! 3. per component: background from a ring around it, the component re-grown at half height
//!    between background and peak, then shape tests (area, elongation, fill ratio, a dark ring);
//! 4. sub-pixel centre = background-subtracted intensity centroid.
//!
//! Projector speckle (1–4 px dots), structured-light stripes and laser lines (elongated) and
//! specular glints (irregular, low fill) fail the shape tests.

use scancraft_core::RgbImage;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct DetectorConfig {
    /// Minimum peak grey level of a marker.
    pub min_peak: u8,
    /// Minimum peak height above the local background, grey levels.
    pub min_contrast: f32,
    /// Half size of the local-mean window used to find candidates, px.
    pub background_radius: usize,
    /// Marker region = pixels above `background + level * (peak - background)`.
    pub level: f32,
    /// Area limits of the region at `level`, px.
    pub min_area: usize,
    pub max_area: usize,
    /// Largest major/minor axis ratio (perspective foreshortening; 3 = 70° from frontal).
    pub max_elongation: f32,
    /// Smallest region area / area of its moment ellipse (a disc is 1, a ragged glint is lower).
    pub min_fill: f32,
    /// Width of the ring outside the region used for the background estimate, px.
    pub ring_px: usize,
    /// Largest fraction of ring pixels allowed above the region threshold (rejects regions that
    /// continue outside, e.g. stripes and edges).
    pub max_bright_ring: f32,
}

impl Default for DetectorConfig {
    fn default() -> Self {
        Self {
            min_peak: 60,
            min_contrast: 35.0,
            background_radius: 20,
            level: 0.5,
            min_area: 6,
            max_area: 3000,
            max_elongation: 3.0,
            min_fill: 0.7,
            ring_px: 3,
            max_bright_ring: 0.04,
        }
    }
}

/// One detected dot, in pixel coordinates of the image it was found in (pixel centres at
/// integer coordinates).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Marker2D {
    pub x: f32,
    pub y: f32,
    /// Region area at half height, px.
    pub area: f32,
    /// Semi-axes of the moment ellipse, px.
    pub major: f32,
    pub minor: f32,
    /// Peak grey level and its height above the background.
    pub peak: f32,
    pub contrast: f32,
}

/// Detects marker dots in an 8-bit single-channel image.
pub fn detect_markers(image: &RgbImage, config: &DetectorConfig) -> Vec<Marker2D> {
    if image.channels != 1 || image.data.len() < (image.width * image.height) as usize {
        return Vec::new();
    }
    let (w, h) = (image.width as usize, image.height as usize);
    let img = &image.data;
    let mean = box_mean(img, w, h, config.background_radius);

    let candidate: Vec<bool> = img
        .iter()
        .zip(&mean)
        .map(|(&v, &m)| v >= config.min_peak && v as f32 - m >= config.min_contrast)
        .collect();

    let mut label = vec![u32::MAX; w * h];
    let mut stack = Vec::new();
    let mut markers = Vec::new();
    let mut next = 0u32;
    for start in 0..w * h {
        if !candidate[start] || label[start] != u32::MAX {
            continue;
        }
        // Flood fill one candidate component, keeping its bounding box.
        let id = next;
        next += 1;
        label[start] = id;
        stack.push(start);
        let (mut x0, mut y0, mut x1, mut y1) = (w, h, 0, 0);
        let mut count = 0usize;
        while let Some(i) = stack.pop() {
            let (x, y) = (i % w, i / w);
            count += 1;
            x0 = x0.min(x);
            x1 = x1.max(x);
            y0 = y0.min(y);
            y1 = y1.max(y);
            for (dx, dy) in [(-1i32, -1i32), (0, -1), (1, -1), (-1, 0), (1, 0), (-1, 1), (0, 1), (1, 1)] {
                let (nx, ny) = (x as i32 + dx, y as i32 + dy);
                if nx < 0 || ny < 0 || nx >= w as i32 || ny >= h as i32 {
                    continue;
                }
                let j = ny as usize * w + nx as usize;
                if candidate[j] && label[j] == u32::MAX {
                    label[j] = id;
                    stack.push(j);
                }
            }
        }
        if count > config.max_area * 2 {
            continue;
        }
        if let Some(m) = measure(img, w, h, (x0, y0, x1, y1), config) {
            markers.push(m);
        }
    }
    // Two candidate components can describe the same dot (e.g. a saturated core split by noise).
    markers.sort_by(|a, b| b.area.total_cmp(&a.area));
    let mut kept: Vec<Marker2D> = Vec::with_capacity(markers.len());
    for m in markers {
        if kept.iter().all(|k| (k.x - m.x).hypot(k.y - m.y) > k.major.max(1.5)) {
            kept.push(m);
        }
    }
    kept
}

/// Re-segments a candidate component at half height and applies the shape tests.
fn measure(img: &[u8], w: usize, h: usize, bbox: (usize, usize, usize, usize), c: &DetectorConfig) -> Option<Marker2D> {
    let (bx0, by0, bx1, by1) = bbox;
    // Search window: the candidate box plus a margin for the half-height region and the ring.
    let margin = c.ring_px + 3 + (bx1 - bx0).max(by1 - by0) / 2;
    let x0 = bx0.saturating_sub(margin);
    let y0 = by0.saturating_sub(margin);
    let x1 = (bx1 + margin).min(w - 1);
    let y1 = (by1 + margin).min(h - 1);

    // Peak inside the candidate box.
    let (mut peak, mut px, mut py) = (0u8, bx0, by0);
    for y in by0..=by1 {
        for x in bx0..=bx1 {
            if img[y * w + x] > peak {
                (peak, px, py) = (img[y * w + x], x, y);
            }
        }
    }

    // Background: ring just outside the candidate box (robust lower half mean).
    let ring_inner = 2;
    let mut ring = Vec::new();
    for y in y0..=y1 {
        for x in x0..=x1 {
            let dx = (x as i64 - (bx0 as i64 - ring_inner)).min((bx1 as i64 + ring_inner) - x as i64);
            let dy = (y as i64 - (by0 as i64 - ring_inner)).min((by1 as i64 + ring_inner) - y as i64);
            let inside = dx >= 0 && dy >= 0;
            let outer = (x as i64) >= bx0 as i64 - ring_inner - c.ring_px as i64
                && (x as i64) <= bx1 as i64 + ring_inner + c.ring_px as i64
                && (y as i64) >= by0 as i64 - ring_inner - c.ring_px as i64
                && (y as i64) <= by1 as i64 + ring_inner + c.ring_px as i64;
            if !inside && outer {
                ring.push(img[y * w + x]);
            }
        }
    }
    if ring.len() < 8 {
        return None;
    }
    ring.sort_unstable();
    let low = &ring[..ring.len().div_ceil(2)];
    let bg = low.iter().map(|&v| v as f32).sum::<f32>() / low.len() as f32;
    let contrast = peak as f32 - bg;
    if contrast < c.min_contrast {
        return None;
    }
    let thresh = bg + c.level * contrast;
    let bright_ring = ring.iter().filter(|&&v| v as f32 >= thresh).count() as f32 / ring.len() as f32;
    if bright_ring > c.max_bright_ring {
        return None;
    }

    // Region at half height connected to the peak, within the window.
    let ww = x1 - x0 + 1;
    let wh = y1 - y0 + 1;
    let mut inside = vec![false; ww * wh];
    let mut stack = vec![(px, py)];
    inside[(py - y0) * ww + (px - x0)] = true;
    let mut touches_border = false;
    let (mut n, mut sx, mut sy) = (0f64, 0f64, 0f64);
    let (mut sxx, mut syy, mut sxy) = (0f64, 0f64, 0f64);
    while let Some((x, y)) = stack.pop() {
        n += 1.0;
        sx += x as f64;
        sy += y as f64;
        sxx += (x * x) as f64;
        syy += (y * y) as f64;
        sxy += (x * y) as f64;
        if x == x0 || y == y0 || x == x1 || y == y1 {
            touches_border = true;
        }
        for (dx, dy) in [(-1i32, 0i32), (1, 0), (0, -1), (0, 1)] {
            let (nx, ny) = (x as i32 + dx, y as i32 + dy);
            if nx < x0 as i32 || ny < y0 as i32 || nx > x1 as i32 || ny > y1 as i32 {
                continue;
            }
            let (nx, ny) = (nx as usize, ny as usize);
            let k = (ny - y0) * ww + (nx - x0);
            if !inside[k] && img[ny * w + nx] as f32 >= thresh {
                inside[k] = true;
                stack.push((nx, ny));
            }
        }
    }
    if touches_border || (n as usize) < c.min_area || (n as usize) > c.max_area {
        return None;
    }
    let (mx, my) = (sx / n, sy / n);
    // Binary second moments (+1/12 for the pixel extent) -> ellipse semi-axes a = 2 sqrt(lambda).
    let cxx = sxx / n - mx * mx + 1.0 / 12.0;
    let cyy = syy / n - my * my + 1.0 / 12.0;
    let cxy = sxy / n - mx * my;
    let tr = cxx + cyy;
    let det = cxx * cyy - cxy * cxy;
    let disc = (tr * tr / 4.0 - det).max(0.0).sqrt();
    let (l1, l2) = (tr / 2.0 + disc, (tr / 2.0 - disc).max(1e-9));
    let (major, minor) = (2.0 * l1.sqrt(), 2.0 * l2.sqrt());
    if major / minor > c.max_elongation as f64 {
        return None;
    }
    let fill = n / (std::f64::consts::PI * major * minor);
    if fill < c.min_fill as f64 {
        return None;
    }

    // Sub-pixel centre: background-subtracted intensity centroid over the region grown by one
    // pixel (captures the anti-aliased rim).
    let (mut sw, mut swx, mut swy) = (0f64, 0f64, 0f64);
    for y in y0..=y1 {
        for x in x0..=x1 {
            let near = (-1i32..=1).any(|dy| {
                (-1i32..=1).any(|dx| {
                    let (nx, ny) = (x as i32 + dx, y as i32 + dy);
                    nx >= x0 as i32
                        && ny >= y0 as i32
                        && nx <= x1 as i32
                        && ny <= y1 as i32
                        && inside[(ny as usize - y0) * ww + (nx as usize - x0)]
                })
            });
            if !near {
                continue;
            }
            let v = (img[y * w + x] as f64 - bg as f64).max(0.0);
            sw += v;
            swx += v * x as f64;
            swy += v * y as f64;
        }
    }
    if sw <= 0.0 {
        return None;
    }
    let _ = wh;
    Some(Marker2D {
        x: (swx / sw) as f32,
        y: (swy / sw) as f32,
        area: n as f32,
        major: major as f32,
        minor: minor as f32,
        peak: peak as f32,
        contrast,
    })
}

/// Mean over a (2r+1)^2 window, clamped at the borders, via an integral image.
fn box_mean(img: &[u8], w: usize, h: usize, r: usize) -> Vec<f32> {
    let mut integral = vec![0u32; (w + 1) * (h + 1)];
    for y in 0..h {
        let mut row = 0u32;
        for x in 0..w {
            row += img[y * w + x] as u32;
            integral[(y + 1) * (w + 1) + x + 1] = integral[y * (w + 1) + x + 1] + row;
        }
    }
    let mut out = vec![0f32; w * h];
    for y in 0..h {
        let (ya, yb) = (y.saturating_sub(r), (y + r + 1).min(h));
        for x in 0..w {
            let (xa, xb) = (x.saturating_sub(r), (x + r + 1).min(w));
            let s = integral[yb * (w + 1) + xb] + integral[ya * (w + 1) + xa]
                - integral[ya * (w + 1) + xb]
                - integral[yb * (w + 1) + xa];
            out[y * w + x] = s as f32 / ((yb - ya) * (xb - xa)) as f32;
        }
    }
    out
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;

    /// Renders an anti-aliased bright ellipse (semi-axes a, b, rotated by `angle`) into `img`,
    /// with 4x4 supersampling.
    pub fn draw_ellipse(img: &mut RgbImage, cx: f32, cy: f32, a: f32, b: f32, angle: f32, level: f32) {
        let (w, h) = (img.width as i32, img.height as i32);
        let r = a.max(b).ceil() as i32 + 2;
        let (s, c) = angle.sin_cos();
        for y in (cy as i32 - r)..=(cy as i32 + r) {
            for x in (cx as i32 - r)..=(cx as i32 + r) {
                if x < 0 || y < 0 || x >= w || y >= h {
                    continue;
                }
                let mut cover = 0.0;
                for sy in 0..4 {
                    for sx in 0..4 {
                        let px = x as f32 + (sx as f32 + 0.5) / 4.0 - 0.5 - cx;
                        let py = y as f32 + (sy as f32 + 0.5) / 4.0 - 0.5 - cy;
                        let u = c * px + s * py;
                        let v = -s * px + c * py;
                        if (u / a).powi(2) + (v / b).powi(2) <= 1.0 {
                            cover += 1.0 / 16.0;
                        }
                    }
                }
                let i = (y * w + x) as usize;
                let base = img.data[i] as f32;
                img.data[i] = (base + cover * (level - base)).round().clamp(0.0, 255.0) as u8;
            }
        }
    }

    pub fn blank(w: u32, h: u32, value: u8) -> RgbImage {
        RgbImage { width: w, height: h, channels: 1, data: vec![value; (w * h) as usize] }
    }

    /// Deterministic pseudo-random generator for tests.
    pub struct Lcg(pub u64);
    impl Lcg {
        pub fn next_f32(&mut self) -> f32 {
            self.0 = self.0.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            ((self.0 >> 33) as f32) / (1u64 << 31) as f32
        }
    }

    #[test]
    fn finds_discs_with_subpixel_centres() {
        let mut img = blank(200, 120, 30);
        let truth = [(40.3f32, 30.7f32, 6.0f32, 6.0f32, 0.0f32), (120.6, 60.2, 9.0, 4.5, 0.6), (170.1, 95.9, 3.0, 3.0, 0.0)];
        for &(x, y, a, b, t) in &truth {
            draw_ellipse(&mut img, x, y, a, b, t, 255.0);
        }
        let found = detect_markers(&img, &DetectorConfig::default());
        assert_eq!(found.len(), 3, "{found:?}");
        for &(x, y, a, b, _) in &truth {
            let m = found.iter().min_by(|p, q| (p.x - x).hypot(p.y - y).total_cmp(&(q.x - x).hypot(q.y - y))).unwrap();
            assert!((m.x - x).hypot(m.y - y) < 0.05, "centre {m:?} vs ({x}, {y})");
            assert!((m.major - a.max(b)).abs() < 0.6 && (m.minor - a.min(b)).abs() < 0.6, "axes {m:?}");
        }
    }

    #[test]
    fn rejects_stripes_speckle_and_glints() {
        let mut img = blank(320, 200, 40);
        let mut rng = Lcg(7);
        // Projector speckle: many 1-2 px bright dots.
        for _ in 0..800 {
            let (x, y) = ((rng.next_f32() * 319.0) as usize, (rng.next_f32() * 199.0) as usize);
            img.data[y * 320 + x] = 200;
            if rng.next_f32() > 0.5 && x + 1 < 320 {
                img.data[y * 320 + x + 1] = 180;
            }
        }
        // Structured-light stripes / laser line: long thin bright lines.
        for y in 0..200 {
            for x in [60usize, 61, 62] {
                img.data[y * 320 + x + y / 8] = 230;
            }
        }
        // A glint: thin curved arc.
        for t in 0..40 {
            let a = t as f32 * 0.06;
            let (x, y) = (250.0 + 12.0 * a.cos(), 150.0 + 12.0 * a.sin());
            img.data[y as usize * 320 + x as usize] = 255;
        }
        // One real marker among all this.
        draw_ellipse(&mut img, 150.4, 100.6, 5.0, 4.0, 0.3, 250.0);
        let found = detect_markers(&img, &DetectorConfig::default());
        assert_eq!(found.len(), 1, "{found:?}");
        assert!((found[0].x - 150.4).abs() < 0.15 && (found[0].y - 100.6).abs() < 0.15);
    }

    #[test]
    fn handles_unsaturated_markers_on_textured_background() {
        // Full-field HD texture image: mid-grey speckled surface, marker only ~60 levels brighter.
        let mut rng = Lcg(3);
        let mut img = blank(160, 120, 0);
        for v in img.data.iter_mut() {
            *v = (90.0 + 25.0 * (rng.next_f32() - 0.5)) as u8;
        }
        draw_ellipse(&mut img, 80.25, 60.75, 7.0, 6.0, 0.0, 160.0);
        let found = detect_markers(&img, &DetectorConfig::default());
        assert_eq!(found.len(), 1, "{found:?}");
        assert!((found[0].x - 80.25).abs() < 0.2 && (found[0].y - 60.75).abs() < 0.2, "{found:?}");
    }
}
