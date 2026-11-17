//! Depth registered to the colour image: every valid depth pixel is projected into the colour
//! camera and z-buffered, so a colour pixel can be given a 3D point (for feature tracking) and
//! occlusion can be checked (for colouring).

use glam::{Vec2, Vec3};
use scancraft_core::{ColorCamera, DepthMap};

/// Per colour pixel: the nearest depth sample that projects there.
#[derive(Debug, Clone)]
pub struct DepthInColor {
    pub width: usize,
    pub height: usize,
    /// Depth along the colour camera's axis (mm), `f32::INFINITY` where nothing projects.
    pub z: Vec<f32>,
    /// The scanner-frame point of that sample.
    pub points: Vec<Vec3>,
}

impl DepthInColor {
    /// Projects `depth` (which must carry intrinsics) into `camera`, at the camera's resolution.
    /// Each sample covers a `splat` x `splat` footprint so the map has no pinholes when the colour
    /// image is sampled more finely than the depth map.
    pub fn new(depth: &DepthMap, camera: &ColorCamera, min_mm: f32, max_mm: f32, splat: usize) -> Self {
        let (w, h) = (camera.width as usize, camera.height as usize);
        let mut out = Self { width: w, height: h, z: vec![f32::INFINITY; w * h], points: vec![Vec3::ZERO; w * h] };
        let Some(k) = depth.intrinsics else { return out };
        let half = (splat as f32 - 1.0) / 2.0;
        for v in 0..depth.height as usize {
            for u in 0..depth.width as usize {
                let raw = depth.data[v * depth.width as usize + u];
                let d = raw as f32 * depth.scale_mm;
                if raw == 0 || d < min_mm || d > max_mm {
                    continue;
                }
                let x = (u as f32 - k.cx) * d / k.fx;
                let y = (v as f32 - k.cy) * d / k.fy;
                let sensor = Vec3::new(x, d, -y);
                let Some((px, zc)) = camera.project_sensor(sensor) else { continue };
                let (x0, y0) = ((px.x - half).round() as isize, (px.y - half).round() as isize);
                for dy in 0..splat as isize {
                    for dx in 0..splat as isize {
                        let (xx, yy) = (x0 + dx, y0 + dy);
                        if xx < 0 || yy < 0 || xx as usize >= w || yy as usize >= h {
                            continue;
                        }
                        let i = yy as usize * w + xx as usize;
                        if zc < out.z[i] {
                            out.z[i] = zc;
                            out.points[i] = sensor;
                        }
                    }
                }
            }
        }
        out
    }

    /// The scanner-frame point seen at colour pixel `px` (nearest cell), if any.
    #[inline]
    pub fn point_at(&self, px: Vec2) -> Option<Vec3> {
        let (x, y) = (px.x.round() as isize, px.y.round() as isize);
        if x < 0 || y < 0 || x as usize >= self.width || y as usize >= self.height {
            return None;
        }
        let i = y as usize * self.width + x as usize;
        self.z[i].is_finite().then(|| self.points[i])
    }

    /// Colour-camera depth (mm) at colour pixel `px` (nearest cell), if any. Combined with the
    /// pixel's ray (`ColorCamera::unproject_color`) this places a point exactly on the ray.
    #[inline]
    pub fn depth_at(&self, px: Vec2) -> Option<f32> {
        let (x, y) = (px.x.round() as isize, px.y.round() as isize);
        if x < 0 || y < 0 || x as usize >= self.width || y as usize >= self.height {
            return None;
        }
        let z = self.z[y as usize * self.width + x as usize];
        z.is_finite().then_some(z)
    }

    /// Fraction of cells with depth.
    pub fn coverage(&self) -> f32 {
        self.z.iter().filter(|z| z.is_finite()).count() as f32 / self.z.len().max(1) as f32
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::Mat3;
    use scancraft_core::PinholeIntrinsics;

    #[test]
    fn identity_camera_reproduces_the_depth_map() {
        let k = PinholeIntrinsics { fx: 50.0, fy: 50.0, cx: 16.0, cy: 12.0 };
        let depth = DepthMap::new(32, 24, 0.1, vec![3000; 32 * 24]).with_intrinsics(k);
        let cam = ColorCamera {
            width: 32,
            height: 24,
            intrinsics: k,
            distortion: [0.0; 5],
            rotation: Mat3::IDENTITY,
            translation: Vec3::ZERO,
        };
        let reg = DepthInColor::new(&depth, &cam, 100.0, 500.0, 1);
        assert!(reg.coverage() > 0.99);
        let p = reg.point_at(Vec2::new(20.0, 6.0)).unwrap();
        assert!((p - Vec3::new(4.0 * 300.0 / 50.0, 300.0, 6.0 * 300.0 / 50.0)).length() < 1e-3, "{p}");
    }
}
