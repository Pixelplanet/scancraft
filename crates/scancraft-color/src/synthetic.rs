//! Synthetic RGB-D frames of a textured plane, with exact ground truth, for tests and the
//! tracking evaluation (`examples/color_track_eval.rs`). A plane is the case geometric tracking
//! cannot handle: sliding along it does not change the depth.

use crate::image_ops::Gray;
use glam::{Vec2, Vec3};
use scancraft_core::{ColorCamera, ColorFrame, DepthMap, PinholeIntrinsics, RgbImage, Transform};
use std::sync::Arc;

/// A textured plane `y = distance_mm` in the world, facing the scanner at the origin.
pub struct TexturedPlane {
    pub texture: Gray,
    pub distance_mm: f32,
    /// Texture resolution, texels per mm.
    pub texels_per_mm: f32,
    /// Uniform depth noise amplitude (deterministic per pixel and frame), mm. A perfectly flat
    /// cloud is unrealistic and degenerate for k-d trees.
    pub depth_noise_mm: f32,
}

impl TexturedPlane {
    /// Renders what the scanner at `pose` (world <- scanner frame) sees: the colour image (ray
    /// cast into the texture, bilinear) and the depth map of a depth camera with `depth_k` at
    /// `depth_size`. Both use the integer-pixel-centre convention of `ColorCamera::project_color`.
    pub fn render(&self, pose: &Transform, cam: &ColorCamera, depth_k: PinholeIntrinsics, depth_size: (u32, u32), timestamp_ms: u64) -> (ColorFrame, DepthMap) {
        let (w, h) = (cam.width as usize, cam.height as usize);
        let world_from_color = cam.world_from_color(pose);
        let center = world_from_color.translation;
        let k = cam.intrinsics;
        let mut data = vec![0u8; w * h * 3];
        for v in 0..h {
            for u in 0..w {
                let (x, y) = cam.undistort((u as f32 - k.cx) / k.fx, (v as f32 - k.cy) / k.fy);
                let dir = world_from_color.rotation * Vec3::new(x, y, 1.0);
                if dir.y <= 0.0 {
                    continue;
                }
                let hit = center + dir * ((self.distance_mm - center.y) / dir.y);
                let tp = Vec2::new(
                    hit.x * self.texels_per_mm + self.texture.width as f32 / 2.0,
                    -hit.z * self.texels_per_mm + self.texture.height as f32 / 2.0,
                );
                let g = self.texture.sample(tp).unwrap_or(128.0) as u8;
                data[(v * w + u) * 3..][..3].copy_from_slice(&[g, g, g]);
            }
        }
        let image = RgbImage { width: w as u32, height: h as u32, channels: 3, data };

        let (dw, dh) = (depth_size.0 as usize, depth_size.1 as usize);
        let mut depth = vec![0u16; dw * dh];
        let inv = pose.inverse();
        let n_s = inv.rotation * Vec3::NEG_Y;
        let p0 = inv.transform_point3(Vec3::new(0.0, self.distance_mm, 0.0));
        for v in 0..dh {
            for u in 0..dw {
                let ray = Vec3::new((u as f32 - depth_k.cx) / depth_k.fx, 1.0, -(v as f32 - depth_k.cy) / depth_k.fy);
                let t = n_s.dot(p0) / n_s.dot(ray);
                if t > 0.0 && t < 600.0 {
                    let mut x = (u as u64 * 73_856_093) ^ (v as u64 * 19_349_663) ^ (timestamp_ms.wrapping_mul(83_492_791));
                    x ^= x >> 13;
                    x = x.wrapping_mul(0x5bd1_e995);
                    x ^= x >> 15;
                    let noise = ((x % 2001) as f32 / 1000.0 - 1.0) * self.depth_noise_mm;
                    depth[v * dw + u] = ((t + noise) * 100.0).round() as u16;
                }
            }
        }
        let frame = ColorFrame { image, jpeg: None, timestamp_ms, camera: Some(Arc::new(cam.clone())) };
        (frame, DepthMap::new(dw as u32, dh as u32, 0.01, depth).with_intrinsics(depth_k))
    }
}
