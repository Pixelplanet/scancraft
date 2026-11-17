//! Stereo rectification by lookup table (the `initUndistortRectifyMap` + `remap` pair of OpenCV),
//! for IR pairs that arrive unrectified.

use rayon::prelude::*;

/// Pinhole camera with Brown–Conrady distortion (`k1, k2, p1, p2, k3`), at calibration
/// resolution.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DistortedCamera {
    pub fx: f64,
    pub fy: f64,
    pub cx: f64,
    pub cy: f64,
    pub dist: [f64; 5],
}

impl DistortedCamera {
    /// Normalised undistorted coordinates → raw pixel coordinates.
    pub fn distort_to_pixel(&self, x: f64, y: f64) -> (f64, f64) {
        let [k1, k2, p1, p2, k3] = self.dist;
        let r2 = x * x + y * y;
        let radial = 1.0 + r2 * (k1 + r2 * (k2 + r2 * k3));
        let xd = x * radial + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x);
        let yd = y * radial + p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y;
        (self.fx * xd + self.cx, self.fy * yd + self.cy)
    }
}

/// Per output pixel: source pixel coordinates (fixed point, 1/256 px) for bilinear sampling.
#[derive(Debug, Clone)]
pub struct RectifyMap {
    pub width: usize,
    pub height: usize,
    src_width: usize,
    src_height: usize,
    /// (x * 256, y * 256) of the source sample; `i32::MIN` = outside the source image.
    map: Vec<(i32, i32)>,
}

impl RectifyMap {
    /// Map from a rectified image of `width` x `height` to a raw image of `src_width` x
    /// `src_height`. Both may be a scaled version of the calibration resolution: `scale` is
    /// stream pixels per calibration pixel (0.5 for a 640x512 stream of a 1280x1024
    /// calibration). `inv_rect` (row-major 3x3) maps homogeneous rectified *calibration* pixels to
    /// normalised camera rays: `inv(P_rect[:, :3] * R_rect)`.
    pub fn new(width: usize, height: usize, src_width: usize, src_height: usize, scale: f64, camera: &DistortedCamera, inv_rect: &[f64; 9]) -> Self {
        let mut map = vec![(i32::MIN, i32::MIN); width * height];
        map.par_chunks_mut(width).enumerate().for_each(|(v, row)| {
            for (u, cell) in row.iter_mut().enumerate() {
                let (uc, vc) = (u as f64 / scale, v as f64 / scale);
                let m = inv_rect;
                let x = m[0] * uc + m[1] * vc + m[2];
                let y = m[3] * uc + m[4] * vc + m[5];
                let z = m[6] * uc + m[7] * vc + m[8];
                if z <= 0.0 {
                    continue;
                }
                let (px, py) = camera.distort_to_pixel(x / z, y / z);
                let (sx, sy) = (px * scale, py * scale);
                if sx >= 0.0 && sy >= 0.0 && sx <= (src_width - 1) as f64 && sy <= (src_height - 1) as f64 {
                    *cell = ((sx * 256.0).round() as i32, (sy * 256.0).round() as i32);
                }
            }
        });
        Self { width, height, src_width, src_height, map }
    }

    /// Source position of an output pixel, if it falls inside the source image.
    pub fn source(&self, u: usize, v: usize) -> Option<(f32, f32)> {
        let (x, y) = self.map[v * self.width + u];
        (x != i32::MIN).then(|| (x as f32 / 256.0, y as f32 / 256.0))
    }

    /// Bilinear remap of an 8-bit image; pixels mapping outside the source become 0.
    pub fn remap(&self, src: &[u8], out: &mut Vec<u8>) {
        assert_eq!(src.len(), self.src_width * self.src_height, "source size");
        let sw = self.src_width;
        out.clear();
        out.resize(self.width * self.height, 0);
        out.par_chunks_mut(self.width).zip(self.map.par_chunks(self.width)).for_each(|(row, map)| {
            for (o, &(x, y)) in row.iter_mut().zip(map) {
                if x == i32::MIN {
                    continue;
                }
                let (x0, y0) = ((x >> 8) as usize, (y >> 8) as usize);
                let (fx, fy) = ((x & 255) as u32, (y & 255) as u32);
                let x1 = (x0 + 1).min(sw - 1);
                let y1 = (y0 + 1).min(self.src_height - 1);
                let p = |xx: usize, yy: usize| src[yy * sw + xx] as u32;
                let top = p(x0, y0) * (256 - fx) + p(x1, y0) * fx;
                let bottom = p(x0, y1) * (256 - fx) + p(x1, y1) * fx;
                *o = ((top * (256 - fy) + bottom * fy + (1 << 15)) >> 16) as u8;
            }
        });
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn identity_rectification_without_distortion_is_a_copy() {
        let cam = DistortedCamera { fx: 100.0, fy: 100.0, cx: 32.0, cy: 24.0, dist: [0.0; 5] };
        // inv(K) for the same K: rectified pixel -> ray.
        let inv = [0.01, 0.0, -0.32, 0.0, 0.01, -0.24, 0.0, 0.0, 1.0];
        let map = RectifyMap::new(64, 48, 64, 48, 1.0, &cam, &inv);
        let src: Vec<u8> = (0..64 * 48).map(|i| (i % 251) as u8).collect();
        let mut out = Vec::new();
        map.remap(&src, &mut out);
        assert_eq!(out[10 * 64 + 20], src[10 * 64 + 20]);
        assert_eq!(map.source(20, 10), Some((20.0, 10.0)));
    }

    #[test]
    fn scaled_stream_uses_scaled_coordinates() {
        let cam = DistortedCamera { fx: 200.0, fy: 200.0, cx: 64.0, cy: 48.0, dist: [0.0; 5] };
        let inv = [0.005, 0.0, -0.32, 0.0, 0.005, -0.24, 0.0, 0.0, 1.0];
        let map = RectifyMap::new(64, 48, 64, 48, 0.5, &cam, &inv);
        let (x, y) = map.source(10, 7).unwrap();
        assert!((x - 10.0).abs() < 0.01 && (y - 7.0).abs() < 0.01);
    }
}
