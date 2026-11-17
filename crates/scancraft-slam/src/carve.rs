//! Free-space evidence from a depth image: where the sensor measured a surface, the space in
//! front of it along the viewing ray is empty. Model points found there are "seen through" and
//! were noise (or misregistered), so fusion removes them (free-space carving).

use glam::Vec3;
use scancraft_core::{DepthMap, PinholeIntrinsics};

/// A reduced-resolution, conservative copy of one depth image for free-space tests. Each cell
/// holds the *nearest* depth of its block of full-resolution pixels and its 3 x 3 neighbourhood,
/// and only where all of them have a measurement: silhouettes, holes and steep slopes therefore
/// never carve, and sub-cell projection errors cannot make a point look "in front".
#[derive(Debug, Clone)]
pub struct FreeSpaceView {
    width: usize,
    height: usize,
    k: PinholeIntrinsics,
    /// Nearest depth around each cell, mm; 0 where the cell cannot be used.
    near: Vec<f32>,
}

impl FreeSpaceView {
    /// `block` full-resolution pixels per cell along each axis (4 for the POP 4's 640 x 512).
    /// `None` without intrinsics.
    pub fn from_depth(depth: &DepthMap, block: u32) -> Option<Self> {
        let k = depth.intrinsics?;
        let b = block.max(1) as usize;
        let (fw, fh) = (depth.width as usize, depth.height as usize);
        let (w, h) = (fw / b, fh / b);
        if w < 3 || h < 3 {
            return None;
        }
        // Block minima (0 if any pixel of the block lacks depth).
        let mut blocks = vec![0f32; w * h];
        for by in 0..h {
            for bx in 0..w {
                let mut m = f32::MAX;
                'block: for y in by * b..(by + 1) * b {
                    for x in bx * b..(bx + 1) * b {
                        let raw = depth.data[y * fw + x];
                        if raw == 0 {
                            m = 0.0;
                            break 'block;
                        }
                        m = m.min(raw as f32 * depth.scale_mm);
                    }
                }
                blocks[by * w + bx] = m;
            }
        }
        let mut near = vec![0f32; w * h];
        for y in 1..h - 1 {
            for x in 1..w - 1 {
                let mut m = f32::MAX;
                for dy in 0..3 {
                    for dx in 0..3 {
                        let d = blocks[(y + dy - 1) * w + x + dx - 1];
                        if d == 0.0 {
                            m = 0.0;
                        }
                        m = m.min(d);
                    }
                }
                near[y * w + x] = m;
            }
        }
        let s = 1.0 / b as f32;
        // Pixel centres: full-res pixel u covers [u - 0.5, u + 0.5]; cell c covers b pixels.
        let k = PinholeIntrinsics { fx: k.fx * s, fy: k.fy * s, cx: (k.cx + 0.5) * s - 0.5, cy: (k.cy + 0.5) * s - 0.5 };
        Some(Self { width: w, height: h, k, near })
    }

    /// True if `p` (sensor frame: +X right, +Y forward, +Z up) lies more than `margin_mm` in front
    /// of the surface the sensor measured along its ray.
    pub fn sees_through(&self, p: Vec3, margin_mm: f32) -> bool {
        if p.y <= 1.0 {
            return false;
        }
        let u = (self.k.fx * p.x / p.y + self.k.cx).round();
        let v = (self.k.fy * -p.z / p.y + self.k.cy).round();
        if u < 0.0 || v < 0.0 || u >= self.width as f32 || v >= self.height as f32 {
            return false;
        }
        let near = self.near[v as usize * self.width + u as usize];
        near > 0.0 && p.y < near - margin_mm
    }

    /// Memory held, bytes (the view is stored with each keyframe so rebuilds carve the same way).
    pub fn size_bytes(&self) -> usize {
        self.near.len() * 4
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn only_points_clearly_in_front_of_interior_measurements_are_seen_through() {
        let k = PinholeIntrinsics { fx: 80.0, fy: 80.0, cx: 31.5, cy: 23.5 };
        let mut data = vec![30000u16; 64 * 48]; // wall at 300 mm
        for v in 0..48 {
            for u in 40..64 {
                data[v * 64 + u] = 0; // no measurement on the right
            }
        }
        let view = FreeSpaceView::from_depth(&DepthMap::new(64, 48, 0.01, data).with_intrinsics(k), 4).unwrap();
        assert!(view.sees_through(Vec3::new(0.0, 250.0, 0.0), 3.0), "50 mm in front of the wall");
        assert!(!view.sees_through(Vec3::new(0.0, 298.5, 0.0), 3.0), "within the margin");
        assert!(!view.sees_through(Vec3::new(0.0, 320.0, 0.0), 3.0), "behind the wall: occluded, not empty");
        // Ray through the unmeasured part (x = 15 px * 250 / 80 right of the hole border).
        assert!(!view.sees_through(Vec3::new(20.0 * 250.0 / 80.0, 250.0, 0.0), 3.0));
        assert!(!view.sees_through(Vec3::new(0.0, -10.0, 0.0), 3.0), "behind the sensor");
    }
}
