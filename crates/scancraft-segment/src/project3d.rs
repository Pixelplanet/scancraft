//! Mask-to-3D projection through calibrated colour cameras with depth-buffer occlusion checks.

use crate::postprocess::Mask2D;
use glam::{Vec2, Vec3};
use scancraft_core::{BackProjection, ColorCamera, ColorRgba, DepthMap, Point3D, PointCloud, Transform};

/// Classification of a 3D point relative to a segmented 2D colour mask.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum PointLabel {
    /// Visible in the colour view and projects onto the foreground mask.
    Object,
    /// Visible in the colour view and projects onto background pixels.
    Background,
    /// Hidden behind a nearer depth sample along the colour camera's viewing ray.
    Occluded,
    /// Behind the colour camera or outside its calibrated field of view.
    OutOfView,
}

/// Settings for [`ColorDepthBuffer`], [`project_depth_mask`], and [`label_points_from_mask`].
#[derive(Debug, Clone, PartialEq)]
pub struct Project3DConfig {
    pub min_depth_mm: f32,
    pub max_depth_mm: f32,
    /// Points further than `z_buf + occlusion_tolerance_mm` along the colour ray count as [`PointLabel::Occluded`].
    pub occlusion_tolerance_mm: f32,
    /// Footprint (`splat x splat` colour pixels) used when rasterising the colour-camera z-buffer.
    pub zbuffer_splat_px: usize,
    /// Optional mask border erosion (pixels) to avoid boundary depth bleed onto background.
    pub mask_erode_px: usize,
}

impl Default for Project3DConfig {
    fn default() -> Self {
        Self {
            min_depth_mm: 80.0,
            max_depth_mm: 1500.0,
            occlusion_tolerance_mm: 2.5,
            zbuffer_splat_px: 2,
            mask_erode_px: 0,
        }
    }
}

/// Colour-camera-resolution z-buffer built from a depth map or sensor-frame point set.
#[derive(Debug, Clone)]
pub struct ColorDepthBuffer {
    pub width: usize,
    pub height: usize,
    /// Minimum depth along the colour camera's +z optical axis (mm), `f32::INFINITY` if empty.
    pub z: Vec<f32>,
}

impl ColorDepthBuffer {
    /// Builds a colour-space z-buffer from a `DepthMap` (requires `depth.intrinsics`).
    pub fn from_depth_map(depth: &DepthMap, camera: &ColorCamera, cfg: &Project3DConfig) -> Self {
        let (cw, ch) = (camera.width as usize, camera.height as usize);
        let mut zbuf = vec![f32::INFINITY; cw * ch];
        let Some(k) = depth.intrinsics else {
            return Self { width: cw, height: ch, z: zbuf };
        };
        let splat = cfg.zbuffer_splat_px.max(1);
        let half = (splat as f32 - 1.0) * 0.5;
        let (dw, dh) = (depth.width as usize, depth.height as usize);
        for v in 0..dh {
            for u in 0..dw {
                let raw = depth.data[v * dw + u];
                if raw == 0 {
                    continue;
                }
                let d = raw as f32 * depth.scale_mm;
                if d < cfg.min_depth_mm || d > cfg.max_depth_mm {
                    continue;
                }
                let x = (u as f32 - k.cx) * d / k.fx;
                let y = (v as f32 - k.cy) * d / k.fy;
                let sensor = Vec3::new(x, d, -y);
                let Some((px, zc)) = camera.project_sensor(sensor) else { continue };
                splat_z(&mut zbuf, cw, ch, px, zc, splat, half);
            }
        }
        Self { width: cw, height: ch, z: zbuf }
    }

    /// Builds a colour-space z-buffer from arbitrary scanner-frame 3D points.
    pub fn from_sensor_points(points_sensor: &[Vec3], camera: &ColorCamera, splat_px: usize) -> Self {
        let (cw, ch) = (camera.width as usize, camera.height as usize);
        let mut zbuf = vec![f32::INFINITY; cw * ch];
        let splat = splat_px.max(1);
        let half = (splat as f32 - 1.0) * 0.5;
        for &p in points_sensor {
            if let Some((px, zc)) = camera.project_sensor(p) {
                splat_z(&mut zbuf, cw, ch, px, zc, splat, half);
            }
        }
        Self { width: cw, height: ch, z: zbuf }
    }

    #[inline]
    pub fn depth_at(&self, px: Vec2) -> Option<f32> {
        let x = px.x.round() as isize;
        let y = px.y.round() as isize;
        if x < 0 || y < 0 || x as usize >= self.width || y as usize >= self.height {
            return None;
        }
        let d = self.z[y as usize * self.width + x as usize];
        d.is_finite().then_some(d)
    }
}

fn splat_z(zbuf: &mut [f32], w: usize, h: usize, px: Vec2, zc: f32, splat: usize, half: f32) {
    let x0 = (px.x - half).round() as isize;
    let y0 = (px.y - half).round() as isize;
    for dy in 0..splat as isize {
        for dx in 0..splat as isize {
            let (xx, yy) = (x0 + dx, y0 + dy);
            if xx >= 0 && yy >= 0 && (xx as usize) < w && (yy as usize) < h {
                let idx = yy as usize * w + xx as usize;
                if zc < zbuf[idx] {
                    zbuf[idx] = zc;
                }
            }
        }
    }
}

/// Per-pixel 3D classification of a `DepthMap` against a colour segmentation `Mask2D`.
#[derive(Debug, Clone)]
pub struct DepthMaskProjection {
    pub width: u32,
    pub height: u32,
    /// One label per depth pixel (`None` when depth is missing or outside `[min_depth_mm, max_depth_mm]`).
    pub labels: Vec<Option<PointLabel>>,
    pub object_count: usize,
    pub occluded_count: usize,
}

impl DepthMaskProjection {
    /// Extracts all pixels labelled [`PointLabel::Object`] into a scanner-frame `PointCloud`.
    pub fn to_object_cloud(&self, depth: &DepthMap, back_proj: &BackProjection) -> PointCloud {
        let Some(k) = depth.intrinsics else {
            return PointCloud::new();
        };
        let valid = depth.valid_mask(back_proj);
        let (w, h) = (depth.width as usize, depth.height as usize);
        let mut cloud = PointCloud::new();
        for v in 0..h {
            for u in 0..w {
                let idx = v * w + u;
                if !valid[idx] || self.labels[idx] != Some(PointLabel::Object) {
                    continue;
                }
                let d = depth.data[idx] as f32 * depth.scale_mm;
                let x = (u as f32 - k.cx) * d / k.fx;
                let y = (v as f32 - k.cy) * d / k.fy;
                let p = [x, d, -y];
                cloud.push(Point3D::new(p).with_color(ColorRgba::WHITE));
            }
        }
        cloud
    }
}

/// Projects `depth` into `camera`, checks occlusion against the colour-space depth buffer, and
/// classifies every valid depth pixel against `mask`.
pub fn project_depth_mask(depth: &DepthMap, camera: &ColorCamera, mask: &Mask2D, cfg: &Project3DConfig) -> DepthMaskProjection {
    let (w, h) = (depth.width as usize, depth.height as usize);
    let mut labels = vec![None; w * h];
    let mut object_count = 0usize;
    let mut occluded_count = 0usize;

    let Some(k) = depth.intrinsics else {
        return DepthMaskProjection { width: depth.width, height: depth.height, labels, object_count, occluded_count };
    };

    let mask_aligned = align_mask_to_camera(mask, camera);
    let active_mask = if cfg.mask_erode_px > 0 { mask_aligned.eroded(cfg.mask_erode_px) } else { mask_aligned };
    let zbuf = ColorDepthBuffer::from_depth_map(depth, camera, cfg);

    for v in 0..h {
        for u in 0..w {
            let idx = v * w + u;
            let raw = depth.data[idx];
            if raw == 0 {
                continue;
            }
            let d = raw as f32 * depth.scale_mm;
            if d < cfg.min_depth_mm || d > cfg.max_depth_mm {
                continue;
            }
            let x = (u as f32 - k.cx) * d / k.fx;
            let y = (v as f32 - k.cy) * d / k.fy;
            let sensor = Vec3::new(x, d, -y);

            let label = classify_sensor_point(sensor, camera, &zbuf, &active_mask, cfg.occlusion_tolerance_mm);
            if label == PointLabel::Object {
                object_count += 1;
            } else if label == PointLabel::Occluded {
                occluded_count += 1;
            }
            labels[idx] = Some(label);
        }
    }

    DepthMaskProjection { width: depth.width, height: depth.height, labels, object_count, occluded_count }
}

/// Labels 3D points (either in scanner frame if `world_from_sensor` is `None`, or in world frame
/// if `Some`) against `mask` with occlusion checking against `depth_buf`.
pub fn label_points_from_mask(
    points: &[Vec3],
    world_from_sensor: Option<&Transform>,
    camera: &ColorCamera,
    depth_buf: &ColorDepthBuffer,
    mask: &Mask2D,
    cfg: &Project3DConfig,
) -> Vec<PointLabel> {
    let sensor_from_world = world_from_sensor.map(|t| t.inverse());
    let mask_aligned = align_mask_to_camera(mask, camera);
    let active_mask = if cfg.mask_erode_px > 0 { mask_aligned.eroded(cfg.mask_erode_px) } else { mask_aligned };

    points
        .iter()
        .map(|&p| {
            let sensor = match &sensor_from_world {
                Some(inv) => inv.transform_point3(p),
                None => p,
            };
            classify_sensor_point(sensor, camera, depth_buf, &active_mask, cfg.occlusion_tolerance_mm)
        })
        .collect()
}

fn classify_sensor_point(
    sensor: Vec3,
    camera: &ColorCamera,
    depth_buf: &ColorDepthBuffer,
    mask: &Mask2D,
    occlusion_tolerance_mm: f32,
) -> PointLabel {
    let Some((px, zc)) = camera.project_sensor(sensor) else {
        return PointLabel::OutOfView;
    };
    if let Some(nearest) = depth_buf.depth_at(px) {
        if zc > nearest + occlusion_tolerance_mm {
            return PointLabel::Occluded;
        }
    }
    let ix = px.x.round() as isize;
    let iy = px.y.round() as isize;
    if ix >= 0 && iy >= 0 && (ix as usize) < mask.width as usize && (iy as usize) < mask.height as usize {
        if mask.is_foreground(ix as u32, iy as u32) {
            PointLabel::Object
        } else {
            PointLabel::Background
        }
    } else {
        PointLabel::OutOfView
    }
}

fn align_mask_to_camera(mask: &Mask2D, camera: &ColorCamera) -> Mask2D {
    if mask.width == camera.width && mask.height == camera.height {
        return mask.clone();
    }
    let (cw, ch) = (camera.width as usize, camera.height as usize);
    let (mw, mh) = (mask.width as usize, mask.height as usize);
    let mut data = vec![0u8; cw * ch];
    for y in 0..ch {
        let sy = ((y as f32 + 0.5) * mh as f32 / ch as f32 - 0.5).round().clamp(0.0, (mh - 1) as f32) as usize;
        for x in 0..cw {
            let sx = ((x as f32 + 0.5) * mw as f32 / cw as f32 - 0.5).round().clamp(0.0, (mw - 1) as f32) as usize;
            data[y * cw + x] = mask.data[sy * mw + sx];
        }
    }
    Mask2D { width: camera.width, height: camera.height, data }
}

/// Accumulates per-point segmentation votes across multiple stored keyframes (plan item 6).
#[derive(Debug, Clone)]
pub struct MultiViewVoter {
    pub object_votes: Vec<u32>,
    pub visible_votes: Vec<u32>,
}

impl MultiViewVoter {
    pub fn new(point_count: usize) -> Self {
        Self { object_votes: vec![0; point_count], visible_votes: vec![0; point_count] }
    }

    pub fn add_view(&mut self, labels: &[PointLabel]) {
        for (i, &l) in labels.iter().enumerate().take(self.object_votes.len()) {
            match l {
                PointLabel::Object => {
                    self.object_votes[i] += 1;
                    self.visible_votes[i] += 1;
                }
                PointLabel::Background => {
                    self.visible_votes[i] += 1;
                }
                PointLabel::Occluded | PointLabel::OutOfView => {}
            }
        }
    }

    /// Resolves each point: `Object` when `object_votes / visible_votes >= min_ratio` (and visible
    /// at least once); `Background` if seen as background; `OutOfView` if never visible.
    pub fn resolve(&self, min_ratio: f32) -> Vec<PointLabel> {
        self.object_votes
            .iter()
            .zip(&self.visible_votes)
            .map(|(&obj, &vis)| {
                if vis == 0 {
                    PointLabel::OutOfView
                } else if (obj as f32 / vis as f32) >= min_ratio {
                    PointLabel::Object
                } else {
                    PointLabel::Background
                }
            })
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::Mat3;
    use scancraft_core::PinholeIntrinsics;

    fn test_camera() -> ColorCamera {
        ColorCamera {
            width: 160,
            height: 120,
            intrinsics: PinholeIntrinsics { fx: 160.0, fy: 160.0, cx: 80.0, cy: 60.0 },
            distortion: [0.05, -0.1, 0.0, 0.0, 0.0],
            rotation: Mat3::from_rotation_y(0.05),
            translation: Vec3::new(-10.0, 0.0, 0.0),
        }
    }

    #[test]
    fn occlusion_and_mask_projection_work_with_offset_camera() {
        let cam = test_camera();
        let depth_k = PinholeIntrinsics { fx: 160.0, fy: 160.0, cx: 80.0, cy: 60.0 };
        // Build a synthetic depth map: background wall at 400 mm, foreground square at 220 mm.
        let (w, h) = (160u32, 120u32);
        let mut raw = vec![4000u16; (w * h) as usize]; // scale 0.1 -> 400 mm
        for y in 40..80 {
            for x in 60..100 {
                raw[(y * w + x) as usize] = 2200; // 220 mm
            }
        }
        let depth = DepthMap::new(w, h, 0.1, raw).with_intrinsics(depth_k);

        // Create a colour mask covering the projection of the foreground square.
        let mut mask_data = vec![0u8; (w * h) as usize];
        for y in 40..80 {
            for x in 60..100 {
                let d = 220.0;
                let p = Vec3::new((x as f32 - 80.0) * d / 160.0, d, -(y as f32 - 60.0) * d / 160.0);
                if let Some((px, _)) = cam.project_sensor(p) {
                    let ix = px.x.round() as usize;
                    let iy = px.y.round() as usize;
                    if ix < w as usize && iy < h as usize {
                        mask_data[iy * w as usize + ix] = 255;
                    }
                }
            }
        }
        let mask = Mask2D { width: w, height: h, data: mask_data };
        let cfg = Project3DConfig { zbuffer_splat_px: 2, ..Default::default() };
        let proj = project_depth_mask(&depth, &cam, &mask, &cfg);
        assert!(proj.object_count > 500, "expected foreground object points, got {}", proj.object_count);

        // Also test arbitrary points directly:
        // - Foreground point on the square (y=220) -> Object
        // - Hidden point directly behind it along the colour ray (same pixel, z=400) -> Occluded
        // - Background point off to the side -> Background
        // - Point behind camera -> OutOfView
        let fg_sensor = Vec3::new(0.0, 220.0, 0.0);
        let (fg_px, fg_z) = cam.project_sensor(fg_sensor).unwrap();
        // Ensure mask pixel at fg_px is set (might have small rounding gap).
        let mut full_fg_mask = mask.clone();
        full_fg_mask.data[fg_px.y.round() as usize * w as usize + fg_px.x.round() as usize] = 255;
        let zbuf = ColorDepthBuffer::from_depth_map(&depth, &cam, &cfg);
        let behind_color = cam.unproject_color(fg_px, fg_z + 150.0);
        let behind_sensor = cam.color_to_sensor(behind_color);
        let side_sensor = Vec3::new(-70.0, 400.0, 0.0);
        let behind_cam = Vec3::new(0.0, -50.0, 0.0);

        let labels = label_points_from_mask(&[fg_sensor, behind_sensor, side_sensor, behind_cam], None, &cam, &zbuf, &full_fg_mask, &cfg);
        assert_eq!(labels[0], PointLabel::Object);
        assert_eq!(labels[1], PointLabel::Occluded);
        assert_eq!(labels[2], PointLabel::Background);
        assert_eq!(labels[3], PointLabel::OutOfView);
    }

    #[test]
    fn multi_view_voter_combines_visible_views() {
        let mut voter = MultiViewVoter::new(3);
        voter.add_view(&[PointLabel::Object, PointLabel::Background, PointLabel::Occluded]);
        voter.add_view(&[PointLabel::Object, PointLabel::Object, PointLabel::OutOfView]);
        voter.add_view(&[PointLabel::Occluded, PointLabel::Background, PointLabel::OutOfView]);
        let res = voter.resolve(0.6);
        assert_eq!(res, vec![PointLabel::Object, PointLabel::Background, PointLabel::OutOfView]);
    }
}
