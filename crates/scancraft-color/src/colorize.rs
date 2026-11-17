//! Vertex colours from RGB keyframes.
//!
//! Every point visible in a keyframe gets that keyframe's colour at its projection, weighted by
//! how squarely the camera saw the surface (`cos^2` of the angle between the normal and the
//! viewing ray), how sharp the keyframe is relative to the others, and how far the projection
//! is from the image border (lens falloff and distortion are worst there). Visibility is
//! decided with a z-buffer of the points themselves at reduced resolution, so surfaces hidden
//! behind others in a view take no colour from it.

use crate::image_ops::{decode_jpeg, sample_rgb};
use crate::keyframe::RgbKeyframe;
use glam::{Vec2, Vec3};
use rayon::prelude::*;
use scancraft_core::{ColorCamera, ColorRgba, Point3D, RgbImage, Transform};

#[derive(Debug, Clone)]
pub struct ColorizeConfig {
    /// Views more oblique than this (angle between normal and ray) are ignored.
    pub max_view_angle_rad: f32,
    /// A point counts as visible if it is at most this far behind the nearest point on its ray.
    pub occlusion_tolerance_mm: f32,
    /// Z-buffer cell size in colour pixels.
    pub zbuffer_cell_px: f32,
    /// Weight falls to zero over this many pixels at the image border.
    pub border_px: f32,
}

impl Default for ColorizeConfig {
    fn default() -> Self {
        Self { max_view_angle_rad: 75f32.to_radians(), occlusion_tolerance_mm: 2.5, zbuffer_cell_px: 4.0, border_px: 60.0 }
    }
}

/// What [`colorize_points`] did.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct ColorizeStats {
    pub points: usize,
    /// Points that received colour from at least one keyframe.
    pub colored: usize,
    pub keyframes_used: usize,
}

/// One view's contribution to a point.
#[inline]
fn view_weight(camera: &ColorCamera, px: Vec2, normal: Vec3, ray_to_camera: Vec3, sharpness_weight: f32, config: &ColorizeConfig) -> f32 {
    let cos = if normal.length_squared() > 0.5 { normal.dot(ray_to_camera) } else { 1.0 };
    if cos <= config.max_view_angle_rad.cos() {
        return 0.0;
    }
    let border = px.x.min(px.y).min(camera.width as f32 - px.x).min(camera.height as f32 - px.y);
    let edge = (border / config.border_px).clamp(0.0, 1.0);
    cos * cos * edge * sharpness_weight
}

/// Colours `points` (world frame, normals facing outward/towards the scanners) from `keyframes`
/// as a weighted average over all views that see each point. Points no keyframe sees keep their
/// colour. JPEGs are decoded here, in parallel.
pub fn colorize_points(points: &mut [Point3D], keyframes: &[RgbKeyframe], config: &ColorizeConfig) -> ColorizeStats {
    let mut stats = ColorizeStats { points: points.len(), ..Default::default() };
    if points.is_empty() || keyframes.is_empty() {
        return stats;
    }
    let max_sharpness = keyframes.iter().map(|k| k.sharpness).fold(0.0f32, f32::max).max(1e-6);
    let positions: Vec<Vec3> = points.iter().map(|p| Vec3::from(p.position)).collect();
    let normals: Vec<Vec3> = points.iter().map(|p| Vec3::from(p.normal)).collect();

    // Per keyframe: (point index, weight, colour) contributions. Keyframes are processed in
    // parallel; memory stays bounded by the visible points per view.
    let contributions: Vec<Vec<(u32, f32, [f32; 3])>> = keyframes
        .par_iter()
        .map(|kf| {
            let Some(image) = decode_jpeg(&kf.jpeg) else { return Vec::new() };
            let camera = if (kf.camera.width, kf.camera.height) == (image.width, image.height) {
                (*kf.camera).clone()
            } else {
                kf.camera.scaled(image.width, image.height)
            };
            let sharpness_weight = (kf.sharpness / max_sharpness).clamp(0.05, 1.0);
            view_contributions(&positions, &normals, &image, &camera, &kf.camera_pose, sharpness_weight, config)
        })
        .collect();

    let mut sum = vec![[0f32; 4]; points.len()];
    for view in &contributions {
        if !view.is_empty() {
            stats.keyframes_used += 1;
        }
        for &(i, w, c) in view {
            let s = &mut sum[i as usize];
            s[0] += w * c[0];
            s[1] += w * c[1];
            s[2] += w * c[2];
            s[3] += w;
        }
    }
    for (p, s) in points.iter_mut().zip(&sum) {
        if s[3] > 0.0 {
            p.color = ColorRgba::new((s[0] / s[3]).round() as u8, (s[1] / s[3]).round() as u8, (s[2] / s[3]).round() as u8, 255);
            stats.colored += 1;
        }
    }
    stats
}

/// Contributions of one view: points visible in `image` taken by `camera` at `camera_pose`
/// (world <- colour optical frame).
pub fn view_contributions(
    positions: &[Vec3],
    normals: &[Vec3],
    image: &RgbImage,
    camera: &ColorCamera,
    camera_pose: &Transform,
    sharpness_weight: f32,
    config: &ColorizeConfig,
) -> Vec<(u32, f32, [f32; 3])> {
    let to_camera = camera_pose.inverse();
    let m = to_camera.to_mat4();
    let cell = config.zbuffer_cell_px.max(1.0);
    let (zw, zh) = ((camera.width as f32 / cell).ceil() as usize, (camera.height as f32 / cell).ceil() as usize);
    let mut zbuf = vec![f32::INFINITY; zw * zh];
    let projected: Vec<Option<(Vec2, f32)>> = positions
        .iter()
        .map(|p| {
            let c = m.transform_point3(*p);
            camera.project_color(c).map(|px| (px, c.z))
        })
        .collect();
    for (px, z) in projected.iter().flatten() {
        let i = (px.y / cell) as usize * zw + (px.x / cell) as usize;
        if *z < zbuf[i] {
            zbuf[i] = *z;
        }
    }
    let center = camera_pose.translation;
    let mut out = Vec::new();
    for (i, pr) in projected.iter().enumerate() {
        let Some((px, z)) = pr else { continue };
        if *z > zbuf[(px.y / cell) as usize * zw + (px.x / cell) as usize] + config.occlusion_tolerance_mm {
            continue;
        }
        let ray = (center - positions[i]).normalize_or_zero();
        let w = view_weight(camera, *px, normals[i], ray, sharpness_weight, config);
        if w <= 0.0 {
            continue;
        }
        if let Some(c) = sample_rgb(image, *px) {
            out.push((i as u32, w, c));
        }
    }
    out
}

/// Colours freshly fused points from the frame that produced them (live preview). The points
/// were just measured by this frame, so no occlusion test is needed. Returns how many got colour.
pub fn colorize_from_view(points: &mut [Point3D], image: &RgbImage, camera: &ColorCamera, camera_pose: &Transform) -> usize {
    let m = camera_pose.inverse().to_mat4();
    let mut n = 0;
    for p in points.iter_mut() {
        let c = m.transform_point3(Vec3::from(p.position));
        if let Some(rgb) = camera.project_color(c).and_then(|px| sample_rgb(image, px)) {
            p.color = ColorRgba::new(rgb[0].round() as u8, rgb[1].round() as u8, rgb[2].round() as u8, 255);
            n += 1;
        }
    }
    n
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::Mat3;
    use scancraft_core::PinholeIntrinsics;
    use std::sync::Arc;

    fn camera() -> ColorCamera {
        ColorCamera {
            width: 160,
            height: 120,
            intrinsics: PinholeIntrinsics { fx: 150.0, fy: 150.0, cx: 80.0, cy: 60.0 },
            distortion: [0.0; 5],
            rotation: Mat3::IDENTITY,
            translation: Vec3::ZERO,
        }
    }

    /// Left half red, right half blue.
    fn jpeg() -> Arc<Vec<u8>> {
        let img = image::RgbImage::from_fn(160, 120, |x, _| if x < 80 { image::Rgb([220, 20, 20]) } else { image::Rgb([20, 20, 220]) });
        let mut out = std::io::Cursor::new(Vec::new());
        img.write_to(&mut out, image::ImageFormat::Jpeg).unwrap();
        Arc::new(out.into_inner())
    }

    #[test]
    fn colours_visible_points_and_skips_hidden_ones() {
        let cam = Arc::new(camera());
        let sensor_pose = Transform::default();
        let kf = RgbKeyframe {
            id: 0,
            timestamp_ms: 0,
            jpeg: jpeg(),
            width: 160,
            height: 120,
            camera_pose: cam.world_from_color(&sensor_pose),
            camera: cam,
            sensor_pose,
            sharpness: 1.0,
            depth_frame_index: 0,
        };
        // Scanner frame: +Y forward. A wall at y = 300 facing the camera; one point hidden behind
        // a nearer one on the same ray.
        let facing = [0.0, -1.0, 0.0];
        let mut points = vec![
            Point3D::new([-40.0, 300.0, 0.0]).with_normal(facing),
            Point3D::new([40.0, 300.0, 0.0]).with_normal(facing),
            Point3D::new([20.0, 150.0, 0.0]).with_normal(facing), // on the ray of (40, 300, 0)
            Point3D::new([0.0, -50.0, 0.0]).with_normal(facing), // behind the camera
        ];
        let stats = colorize_points(&mut points, &[kf], &ColorizeConfig::default());
        assert!(points[0].color.r > 180 && points[0].color.b < 60, "{:?}", points[0].color);
        assert!(points[2].color.b > 180, "near point is blue: {:?}", points[2].color);
        assert_eq!(points[1].color, ColorRgba::WHITE, "hidden behind the near point");
        assert_eq!(points[3].color, ColorRgba::WHITE);
        assert_eq!(stats.keyframes_used, 1);
    }
}
