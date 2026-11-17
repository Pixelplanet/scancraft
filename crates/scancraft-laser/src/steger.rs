//! Sub-pixel laser line centres (Steger, "An unbiased detector of curvilinear structures",
//! 1998): Gaussian-derivative Hessian at candidate pixels, line normal from its dominant
//! eigenvector, centre from the second-order Taylor expansion along that normal.
//!
//! Only pixels brighter than the local background are evaluated, so the cost scales with the
//! amount of laser in the image rather than with its size.

/// One line-centre sample in image coordinates.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LinePoint {
    /// Sub-pixel centre (column, row).
    pub u: f32,
    pub v: f32,
    /// Unit tangent of the line (orthogonal to the normal).
    pub tu: f32,
    pub tv: f32,
    /// Ridge strength: minus the dominant Hessian eigenvalue of the smoothed image.
    pub strength: f32,
    /// Smoothed peak intensity above background.
    pub peak: f32,
}

#[derive(Debug, Clone, Copy)]
pub struct StegerParams {
    /// Gaussian scale in pixels; ~ line half-width / sqrt(3) is optimal. The POP 4 lines are 2-4 px wide.
    pub sigma: f32,
    /// Minimum raw intensity above the black level for a pixel to be evaluated.
    pub min_intensity: u8,
    /// Minimum ridge strength (−λ of the smoothed image, intensity units / px²).
    pub min_strength: f32,
    /// Black level subtracted from the image (16 for the POP 4's JPEG pair stream).
    pub black_level: u8,
}

impl Default for StegerParams {
    fn default() -> Self {
        Self { sigma: 1.5, min_intensity: 24, min_strength: 2.0, black_level: 16 }
    }
}

/// Gaussian, first and second derivative kernels of radius `r`.
struct Kernels {
    r: i32,
    g: Vec<f32>,
    g1: Vec<f32>,
    g2: Vec<f32>,
}

impl Kernels {
    fn new(sigma: f32) -> Self {
        let r = (3.0 * sigma).ceil().max(2.0) as i32;
        let s2 = sigma * sigma;
        let mut g: Vec<f32> = (-r..=r).map(|x| (-(x * x) as f32 / (2.0 * s2)).exp()).collect();
        let norm: f32 = g.iter().sum();
        g.iter_mut().for_each(|v| *v /= norm);
        // Derivatives of the normalised Gaussian, mirrored because the kernels are applied as a
        // correlation (`sum k[i] * I[x - r + i]`): odd kernels change sign.
        let g1: Vec<f32> = (-r..=r).zip(&g).map(|(x, &gv)| (x as f32) / s2 * gv).collect();
        let mut g2: Vec<f32> = (-r..=r).zip(&g).map(|(x, &gv)| ((x * x) as f32 / s2 - 1.0) / s2 * gv).collect();
        // Remove the DC response of the sampled second derivative so flat regions give 0.
        let mean = g2.iter().sum::<f32>() / g2.len() as f32;
        g2.iter_mut().for_each(|v| *v -= mean);
        Self { r, g, g1, g2 }
    }
}

/// Extracts line centre points from an 8-bit grey image (row-major, `width` x `height`).
/// Rows above `first_row` are skipped (e.g. an in-band header blanked by the driver).
pub fn extract(image: &[u8], width: usize, height: usize, params: &StegerParams, first_row: usize) -> Vec<LinePoint> {
    assert_eq!(image.len(), width * height);
    let k = Kernels::new(params.sigma);
    let r = k.r as usize;
    let thr = params.black_level.saturating_add(params.min_intensity);
    let mut out = Vec::new();
    if width <= 2 * r + 2 || height <= 2 * r + 2 {
        return out;
    }
    let bl = params.black_level as f32;
    let n = 2 * r + 1;
    // Per candidate: horizontal 1-D convolutions of each window row, then vertical combination.
    let mut rows_g = vec![0f32; n];
    let mut rows_g1 = vec![0f32; n];
    let mut rows_g2 = vec![0f32; n];
    for y in first_row.max(r + 1)..height - r - 1 {
        let row = &image[y * width..(y + 1) * width];
        for x in r + 1..width - r - 1 {
            let c = row[x];
            if c < thr {
                continue;
            }
            // Ridge candidate: not darker than both neighbours along x, or along y.
            let (l, rt) = (row[x - 1], row[x + 1]);
            let (up, dn) = (image[(y - 1) * width + x], image[(y + 1) * width + x]);
            if !((c >= l && c >= rt) || (c >= up && c >= dn)) {
                continue;
            }
            for (j, yy) in (y - r..=y + r).enumerate() {
                let w = &image[yy * width + x - r..yy * width + x + r + 1];
                let (mut a, mut b, mut cc) = (0f32, 0f32, 0f32);
                for (i, &px) in w.iter().enumerate() {
                    let p = px as f32 - bl;
                    a += k.g[i] * p;
                    b += k.g1[i] * p;
                    cc += k.g2[i] * p;
                }
                rows_g[j] = a;
                rows_g1[j] = b;
                rows_g2[j] = cc;
            }
            let (mut s, mut rx, mut ry, mut rxx, mut rxy, mut ryy) = (0f32, 0f32, 0f32, 0f32, 0f32, 0f32);
            for j in 0..n {
                s += k.g[j] * rows_g[j];
                rx += k.g[j] * rows_g1[j];
                rxx += k.g[j] * rows_g2[j];
                ry += k.g1[j] * rows_g[j];
                rxy += k.g1[j] * rows_g1[j];
                ryy += k.g2[j] * rows_g[j];
            }
            // Eigen-decomposition of [[rxx, rxy], [rxy, ryy]]; the line normal belongs to the
            // eigenvalue of largest magnitude, which is negative on a bright line.
            let tr = 0.5 * (rxx + ryy);
            let det = ((0.5 * (rxx - ryy)).powi(2) + rxy * rxy).sqrt();
            let lambda = tr - det;
            if -lambda < params.min_strength || (tr + det).abs() > -lambda {
                continue;
            }
            let (mut nx, mut ny) = if rxy.abs() > 1e-9 { (lambda - ryy, rxy) } else if rxx < ryy { (1.0, 0.0) } else { (0.0, 1.0) };
            let len = (nx * nx + ny * ny).sqrt();
            nx /= len;
            ny /= len;
            let denom = nx * nx * rxx + 2.0 * nx * ny * rxy + ny * ny * ryy;
            let t = -(nx * rx + ny * ry) / denom;
            let (ox, oy) = (t * nx, t * ny);
            if ox.abs() > 0.5 || oy.abs() > 0.5 {
                continue;
            }
            out.push(LinePoint {
                u: x as f32 + ox,
                v: y as f32 + oy,
                tu: -ny,
                tv: nx,
                strength: -lambda,
                peak: s,
            });
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Renders an anti-aliased bright line u = u0 + slope * v with a Gaussian profile.
    fn render(width: usize, height: usize, u0: f32, slope: f32, half_width: f32) -> Vec<u8> {
        let mut img = vec![16u8; width * height];
        let norm = (1.0 + slope * slope).sqrt();
        for v in 0..height {
            for u in 0..width {
                let dist = (u as f32 - (u0 + slope * v as f32)) / norm;
                let val = 16.0 + 200.0 * (-(dist * dist) / (2.0 * half_width * half_width)).exp();
                img[v * width + u] = val.round() as u8;
            }
        }
        img
    }

    #[test]
    fn finds_subpixel_centre_of_diagonal_line() {
        let (w, h) = (96, 64);
        for &(u0, slope) in &[(30.3f32, 0.78f32), (70.65, -1.3), (40.1, 0.0)] {
            let img = render(w, h, u0, slope, 1.2);
            let pts = extract(&img, w, h, &StegerParams::default(), 0);
            assert!(pts.len() > 40, "found {} points", pts.len());
            let norm = (1.0 + slope * slope).sqrt();
            let mut max_err = 0f32;
            for p in &pts {
                let err = ((p.u - (u0 + slope * p.v)) / norm).abs();
                max_err = max_err.max(err);
                // Tangent is parallel to the line direction (slope, 1).
                let along = (p.tu * slope + p.tv).abs() / norm;
                assert!(along > 0.99, "tangent {:?}", (p.tu, p.tv));
            }
            assert!(max_err < 0.08, "max centre error {max_err} px for slope {slope}");
        }
    }

    #[test]
    fn ignores_flat_and_dark_images() {
        let img = vec![60u8; 64 * 64];
        assert!(extract(&img, 64, 64, &StegerParams::default(), 0).is_empty());
        let img = vec![16u8; 64 * 64];
        assert!(extract(&img, 64, 64, &StegerParams::default(), 0).is_empty());
    }
}
