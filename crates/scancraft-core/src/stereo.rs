//! Lens models for streams that deliver *unrectified* stereo images (e.g. the POP 4 laser
//! modes): maps raw pixels into the rectified frame of a [`crate::StereoRig`]. Only points are
//! mapped (no image remapping), which keeps sub-pixel features exact and costs nothing per pixel.
//!
//! Conventions follow OpenCV (`undistortPoints` / `stereoRectify`).

use serde::{Deserialize, Serialize};

/// One raw camera: pinhole + Brown-Conrady distortion + rectifying homography.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RawCamera {
    pub fx: f64,
    pub fy: f64,
    pub cx: f64,
    pub cy: f64,
    /// `k1, k2, p1, p2, k3`.
    pub distortion: [f64; 5],
    /// Undistorted normalised ray `(x, y, 1)` -> homogeneous rectified pixel, row-major 3x3
    /// (`P_rect[:3,:3] * R_rect`; the inverse of OpenCV's `initUndistortRectifyMap` `iR`).
    pub raw_to_rectified: [f64; 9],
}

impl RawCamera {
    /// Removes lens distortion: raw pixel -> normalised image coordinates (iterative, like
    /// OpenCV `undistortPoints`).
    pub fn undistort(&self, u: f64, v: f64) -> (f64, f64) {
        let [k1, k2, p1, p2, k3] = self.distortion;
        let (xd, yd) = ((u - self.cx) / self.fx, (v - self.cy) / self.fy);
        let (mut x, mut y) = (xd, yd);
        for _ in 0..12 {
            let r2 = x * x + y * y;
            let radial = 1.0 + r2 * (k1 + r2 * (k2 + r2 * k3));
            let dx = 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x);
            let dy = p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y;
            x = (xd - dx) / radial;
            y = (yd - dy) / radial;
        }
        (x, y)
    }

    /// Applies lens distortion: normalised coordinates -> raw pixel.
    pub fn distort(&self, x: f64, y: f64) -> (f64, f64) {
        let [k1, k2, p1, p2, k3] = self.distortion;
        let r2 = x * x + y * y;
        let radial = 1.0 + r2 * (k1 + r2 * (k2 + r2 * k3));
        let xd = x * radial + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x);
        let yd = y * radial + p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y;
        (self.fx * xd + self.cx, self.fy * yd + self.cy)
    }

    /// Raw pixel -> rectified pixel.
    pub fn rectify(&self, u: f64, v: f64) -> (f64, f64) {
        let (x, y) = self.undistort(u, v);
        let h = &self.raw_to_rectified;
        let w = h[6] * x + h[7] * y + h[8];
        ((h[0] * x + h[1] * y + h[2]) / w, (h[3] * x + h[4] * y + h[5]) / w)
    }

    /// Rectified pixel -> raw pixel.
    pub fn unrectify(&self, u: f64, v: f64) -> (f64, f64) {
        let h = invert3(&self.raw_to_rectified);
        let w = h[6] * u + h[7] * v + h[8];
        self.distort((h[0] * u + h[1] * v + h[2]) / w, (h[3] * u + h[4] * v + h[5]) / w)
    }
}

/// Raw left/right cameras of a stereo pair whose rectified geometry is the owning
/// [`crate::StereoRig`] (`intrinsics` = rectified left camera, `baseline_mm`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct StereoRectification {
    pub left: RawCamera,
    pub right: RawCamera,
    /// Principal point x of the rectified *right* image, px. Rectification of converging
    /// cameras usually gives it a different value than the left one; disparity is then
    /// `u_left - u_right - (cx_left - right_cx)`.
    pub right_cx: f64,
}

/// Inverse of a row-major 3x3 matrix (returns zeros if singular).
pub fn invert3(m: &[f64; 9]) -> [f64; 9] {
    let [a, b, c, d, e, f, g, h, i] = *m;
    let det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
    if det.abs() < 1e-300 {
        return [0.0; 9];
    }
    let s = 1.0 / det;
    [
        (e * i - f * h) * s,
        (c * h - b * i) * s,
        (b * f - c * e) * s,
        (f * g - d * i) * s,
        (a * i - c * g) * s,
        (c * d - a * f) * s,
        (d * h - e * g) * s,
        (b * g - a * h) * s,
        (a * e - b * d) * s,
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rectify_and_unrectify_round_trip() {
        let cam = RawCamera {
            fx: 1484.0,
            fy: 1483.5,
            cx: 649.7,
            cy: 478.2,
            distortion: [0.0712, -0.2102, 0.00064, -0.00016, 0.2219],
            // About 10 degrees of rotation about y, then the rectified camera matrix.
            raw_to_rectified: {
                let (s, c) = (0.1729f64, 0.9849f64);
                let r = [c, 0.0, s, 0.0, 1.0, 0.0, -s, 0.0, c];
                let k = [1482.67, 0.0, 340.44, 0.0, 1482.67, 485.67, 0.0, 0.0, 1.0];
                let mut m = [0.0; 9];
                for i in 0..3 {
                    for j in 0..3 {
                        m[i * 3 + j] = (0..3).map(|t| k[i * 3 + t] * r[t * 3 + j]).sum();
                    }
                }
                m
            },
        };
        for &(u, v) in &[(10.0, 10.0), (640.0, 512.0), (1270.0, 1000.0), (300.0, 900.0)] {
            let (ru, rv) = cam.rectify(u, v);
            let (bu, bv) = cam.unrectify(ru, rv);
            assert!((bu - u).abs() < 1e-3 && (bv - v).abs() < 1e-3, "{u},{v} -> {ru},{rv} -> {bu},{bv}");
        }
        let id = invert3(&invert3(&cam.raw_to_rectified));
        assert!(id.iter().zip(&cam.raw_to_rectified).all(|(a, b)| (a - b).abs() < 1e-6 * b.abs().max(1.0)));
    }
}
