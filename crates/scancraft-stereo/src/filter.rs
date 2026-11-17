//! Disparity post-filters: speckle removal and small-hole filling.

use crate::sgm::DisparityImage;

/// Removes connected regions with fewer than `max_size` pixels. Two 4-neighbours belong to the
/// same region if their disparities differ by at most `max_diff` (same rule as OpenCV's
/// `filterSpeckles`). Small isolated blobs are almost always mismatches.
pub fn remove_speckles(disp: &mut DisparityImage, max_size: usize, max_diff: f32) {
    let (w, h) = (disp.width, disp.height);
    let mut label = vec![0u32; w * h];
    let mut stack = Vec::new();
    let mut region = Vec::new();
    let mut next = 1u32;
    for start in 0..w * h {
        if label[start] != 0 || disp.data[start].is_nan() {
            continue;
        }
        label[start] = next;
        stack.push(start);
        region.clear();
        while let Some(p) = stack.pop() {
            region.push(p);
            let v = disp.data[p];
            let (x, y) = (p % w, p / w);
            let mut visit = |q: usize| {
                if label[q] == 0 && !disp.data[q].is_nan() && (disp.data[q] - v).abs() <= max_diff {
                    label[q] = next;
                    stack.push(q);
                }
            };
            if x > 0 {
                visit(p - 1);
            }
            if x + 1 < w {
                visit(p + 1);
            }
            if y > 0 {
                visit(p - w);
            }
            if y + 1 < h {
                visit(p + w);
            }
        }
        if region.len() < max_size {
            for &p in &region {
                disp.data[p] = f32::NAN;
            }
        }
        next += 1;
    }
}

/// Fills invalid 4-connected regions of at most `max_size` pixels that do not touch the image
/// border and whose bordering valid pixels span at most `max_range` px of disparity (i.e. a hole
/// inside one surface, not a gap between foreground and background). Each hole pixel gets the
/// mean of the bordering disparities, weighted by inverse distance.
pub fn fill_small_holes(disp: &mut DisparityImage, max_size: usize, max_range: f32) {
    let (w, h) = (disp.width, disp.height);
    let mut seen = vec![false; w * h];
    let mut stack = Vec::new();
    let mut hole = Vec::new();
    let mut border: Vec<usize> = Vec::new();
    for start in 0..w * h {
        if seen[start] || !disp.data[start].is_nan() {
            continue;
        }
        seen[start] = true;
        stack.push(start);
        hole.clear();
        border.clear();
        let mut touches_edge = false;
        while let Some(p) = stack.pop() {
            hole.push(p);
            let (x, y) = (p % w, p / w);
            if x == 0 || y == 0 || x + 1 == w || y + 1 == h {
                touches_edge = true;
            }
            let neighbours = [
                (x > 0).then(|| p - 1),
                (x + 1 < w).then(|| p + 1),
                (y > 0).then(|| p - w),
                (y + 1 < h).then(|| p + w),
            ];
            for q in neighbours.into_iter().flatten() {
                if disp.data[q].is_nan() {
                    if !seen[q] {
                        seen[q] = true;
                        stack.push(q);
                    }
                } else {
                    border.push(q);
                }
            }
        }
        if touches_edge || hole.len() > max_size || border.is_empty() {
            continue;
        }
        let (mut lo, mut hi) = (f32::MAX, f32::MIN);
        for &q in &border {
            lo = lo.min(disp.data[q]);
            hi = hi.max(disp.data[q]);
        }
        if hi - lo > max_range {
            continue;
        }
        for &p in &hole {
            let (px, py) = ((p % w) as f32, (p / w) as f32);
            let (mut acc, mut wsum) = (0.0f32, 0.0f32);
            for &q in &border {
                let (qx, qy) = ((q % w) as f32, (q / w) as f32);
                let wt = 1.0 / ((px - qx).powi(2) + (py - qy).powi(2));
                acc += wt * disp.data[q];
                wsum += wt;
            }
            disp.data[p] = acc / wsum;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn image(w: usize, h: usize, v: f32) -> DisparityImage {
        DisparityImage { width: w, height: h, data: vec![v; w * h] }
    }

    #[test]
    fn removes_small_blobs_keeps_large_regions() {
        let mut d = image(30, 30, 10.0);
        // A 3x3 blob at a very different disparity.
        for y in 5..8 {
            for x in 5..8 {
                d.data[y * 30 + x] = 40.0;
            }
        }
        remove_speckles(&mut d, 20, 1.0);
        assert!(d.data[6 * 30 + 6].is_nan());
        assert_eq!(d.data[20 * 30 + 20], 10.0);
    }

    #[test]
    fn fills_holes_inside_a_surface_only() {
        let mut d = image(30, 30, 10.0);
        d.data[15 * 30 + 15] = f32::NAN;
        d.data[15 * 30 + 16] = f32::NAN;
        // A hole on a depth edge must stay.
        for y in 0..30 {
            for x in 22..30 {
                d.data[y * 30 + x] = 30.0;
            }
        }
        d.data[10 * 30 + 21] = f32::NAN;
        d.data[0] = f32::NAN; // touches the border
        fill_small_holes(&mut d, 10, 2.0);
        assert!((d.data[15 * 30 + 15] - 10.0).abs() < 1e-4);
        assert!(d.data[10 * 30 + 21].is_nan());
        assert!(d.data[0].is_nan());
    }
}
