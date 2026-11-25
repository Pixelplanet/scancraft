//! RGB keyframes: the colour images a pass keeps for texturing, vertex colours and Gaussian
//! splatting, with everything needed to project into them.

use crate::image_ops::{downsample2, sharpness, to_gray};
use glam::Vec3;
use scancraft_core::{ColorCamera, ColorFrame, Transform};
use serde::{Deserialize, Serialize};
use std::sync::Arc;

/// One kept colour image of a colour pass.
///
/// Conventions:
/// - `sensor_pose` maps the depth sensor's scanner frame (+X right, +Y forward, +Z up; the frame
///   of depth point clouds) to the world, in mm: the tracker's pose for this instant.
/// - `camera_pose` maps the colour camera's optical frame (OpenCV: +x right, +y down,
///   +z forward) to the world, in mm: `sensor_pose` combined with the camera's extrinsic. This
///   is the camera-to-world matrix for texture mapping / splatting (COLMAP's world-to-camera is
///   its inverse; Blender/NeRF conventions flip y and z).
/// - `camera` holds intrinsics (at `width` x `height`) and Brown-Conrady distortion; the JPEG is
///   *not* undistorted.
/// - `timestamp_ms` is on the device clock shared with depth frames.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RgbKeyframe {
    pub id: u64,
    pub timestamp_ms: u64,
    /// The image exactly as the scanner sent it.
    pub jpeg: Arc<Vec<u8>>,
    pub width: u32,
    pub height: u32,
    pub camera: Arc<ColorCamera>,
    pub sensor_pose: Transform,
    pub camera_pose: Transform,
    /// Variance of the Laplacian of the half-resolution grey image; higher is sharper.
    /// Comparable between frames of the same scene, not across scenes.
    pub sharpness: f32,
    /// Index of the depth frame whose pose was used.
    pub depth_frame_index: u64,
}

impl RgbKeyframe {
    pub fn new(id: u64, frame: &ColorFrame, camera: Arc<ColorCamera>, sensor_pose: Transform, sharpness: f32, depth_frame_index: u64) -> Option<Self> {
        let jpeg = match &frame.jpeg {
            Some(j) => j.clone(),
            None => encode_rgb_jpeg(&frame.image)?,
        };
        Some(Self {
            id,
            timestamp_ms: frame.timestamp_ms,
            jpeg,
            width: frame.image.width,
            height: frame.image.height,
            camera_pose: camera.world_from_color(&sensor_pose),
            camera,
            sensor_pose,
            sharpness,
            depth_frame_index,
        })
    }

    /// Colour camera centre in the world.
    pub fn center(&self) -> Vec3 {
        self.camera_pose.translation
    }

    /// Viewing direction (colour optical axis) in the world.
    pub fn view_dir(&self) -> Vec3 {
        self.camera_pose.rotation * Vec3::Z
    }

    /// Projects a world point into the image: pixel and depth along the optical axis.
    pub fn project_world(&self, p: Vec3) -> Option<(glam::Vec2, f32)> {
        let c = self.camera_pose.inverse().transform_point3(p);
        self.camera.project_color(c).map(|px| (px, c.z))
    }
}

fn encode_rgb_jpeg(image: &scancraft_core::RgbImage) -> Option<Arc<Vec<u8>>> {
    if image.width == 0 || image.height == 0 || image.channels < 3 {
        return None;
    }
    let expected = (image.width as usize).checked_mul(image.height as usize)?.checked_mul(image.channels as usize)?;
    if image.data.len() < expected {
        return None;
    }
    let rgb = if image.channels == 3 {
        image.data[..expected].to_vec()
    } else {
        image.data[..expected].chunks_exact(image.channels as usize).flat_map(|px| [px[0], px[1], px[2]]).collect()
    };
    let mut out = Vec::new();
    let mut enc = image::codecs::jpeg::JpegEncoder::new_with_quality(&mut out, 90);
    enc.encode(&rgb, image.width, image.height, image::ExtendedColorType::Rgb8).ok()?;
    Some(Arc::new(out))
}

/// Pose at time `t` between two timed poses (linear translation, slerp rotation). The POP 4's
/// colour frames are triggered with the depth frames (1-2 ms apart), so the pose of the paired
/// depth frame is normally used directly; this is for colour frames between depth frames.
pub fn interpolate_pose(t: u64, (t0, p0): (u64, &Transform), (t1, p1): (u64, &Transform)) -> Transform {
    if t1 == t0 {
        return *p0;
    }
    let a = ((t as f64 - t0 as f64) / (t1 as f64 - t0 as f64)) as f32;
    Transform::from_rotation_translation(p0.rotation.slerp(p1.rotation, a).normalize(), p0.translation.lerp(p1.translation, a))
}

/// Sharpness score used for keyframe selection.
pub fn frame_sharpness(frame: &ColorFrame) -> f32 {
    sharpness(&downsample2(&to_gray(&frame.image)))
}

#[derive(Debug, Clone)]
pub struct SelectorConfig {
    /// A new keyframe is taken once the colour camera moved this far from the last one ...
    pub min_translation_mm: f32,
    /// ... or turned this much.
    pub min_rotation_rad: f32,
    /// Frames considered per keyframe: the sharpest of the next `window` candidates is kept.
    pub window: usize,
    /// Candidates below this fraction of the recent sharpness level count as blurred and are
    /// skipped (unless no sharper one comes within the window).
    pub blur_ratio: f32,
}

impl Default for SelectorConfig {
    fn default() -> Self {
        Self { min_translation_mm: 12.0, min_rotation_rad: 5f32.to_radians(), window: 4, blur_ratio: 0.6 }
    }
}

/// Picks sharp, well-spread keyframes from a stream of posed colour frames.
#[derive(Debug, Default)]
pub struct KeyframeSelector {
    pub config: SelectorConfig,
    last_pose: Option<Transform>,
    /// Running sharpness level (exponential average over all offered frames).
    level: Option<f32>,
    candidates: Vec<RgbKeyframe>,
    next_id: u64,
}

impl KeyframeSelector {
    pub fn new(config: SelectorConfig) -> Self {
        Self { config, ..Self::default() }
    }

    /// Offers a colour frame with its sensor pose (only frames the tracker accepted). Returns a
    /// keyframe when one is chosen. `sharpness` comes from [`frame_sharpness`].
    pub fn offer(&mut self, frame: &ColorFrame, camera: &Arc<ColorCamera>, sensor_pose: Transform, sharpness: f32, depth_frame_index: u64) -> Option<RgbKeyframe> {
        self.level = Some(match self.level {
            Some(l) => 0.9 * l + 0.1 * sharpness,
            None => sharpness,
        });
        let camera_pose = camera.world_from_color(&sensor_pose);
        let moved = self.last_pose.is_none_or(|last| {
            (camera_pose.translation - last.translation).length() >= self.config.min_translation_mm
                || camera_pose.rotation.angle_between(last.rotation) >= self.config.min_rotation_rad
        });
        if !moved && self.candidates.is_empty() {
            return None;
        }
        let id = self.next_id;
        self.candidates.push(RgbKeyframe::new(id, frame, camera.clone(), sensor_pose, sharpness, depth_frame_index)?);
        let level = self.level.unwrap_or(sharpness);
        let sharp_enough = sharpness >= self.config.blur_ratio * level;
        if self.candidates.len() < self.config.window && !(sharp_enough && self.candidates.len() == 1 && self.last_pose.is_none()) {
            return None;
        }
        let best = self.candidates.drain(..).max_by(|a, b| a.sharpness.total_cmp(&b.sharpness))?;
        let mut best = best;
        best.id = id;
        self.next_id += 1;
        self.last_pose = Some(best.camera_pose);
        Some(best)
    }

    /// Forgets the reference pose (e.g. when a new pass starts).
    pub fn reset(&mut self) {
        self.last_pose = None;
        self.candidates.clear();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::{Mat3, Quat};
    use scancraft_core::{PinholeIntrinsics, RgbImage};

    pub(crate) fn camera() -> Arc<ColorCamera> {
        Arc::new(ColorCamera {
            width: 64,
            height: 48,
            intrinsics: PinholeIntrinsics { fx: 60.0, fy: 60.0, cx: 32.0, cy: 24.0 },
            distortion: [0.0; 5],
            rotation: Mat3::IDENTITY,
            translation: Vec3::new(-24.0, 0.0, 0.0),
        })
    }

    fn frame(ts: u64) -> ColorFrame {
        ColorFrame {
            image: RgbImage { width: 64, height: 48, channels: 3, data: vec![0; 64 * 48 * 3] },
            jpeg: Some(Arc::new(vec![0xff, 0xd8, 0xff, 0xd9])),
            timestamp_ms: ts,
            camera: None,
        }
    }

    #[test]
    fn keeps_the_sharpest_frame_per_window_and_waits_for_motion() {
        let cam = camera();
        let mut sel = KeyframeSelector::new(SelectorConfig { window: 3, ..SelectorConfig::default() });
        let at = |x: f32| Transform::from_translation(Vec3::new(x, 0.0, 0.0));
        let first = sel.offer(&frame(0), &cam, at(0.0), 100.0, 1).expect("first sharp frame is kept at once");
        assert_eq!(first.id, 0);
        assert!(sel.offer(&frame(80), &cam, at(1.0), 100.0, 2).is_none(), "not moved enough");
        assert!(sel.offer(&frame(160), &cam, at(20.0), 40.0, 3).is_none());
        assert!(sel.offer(&frame(240), &cam, at(21.0), 120.0, 4).is_none());
        let kf = sel.offer(&frame(320), &cam, at(22.0), 70.0, 5).expect("window complete");
        assert_eq!((kf.id, kf.timestamp_ms, kf.depth_frame_index), (1, 240, 4));
        // camera_pose = sensor pose + extrinsic: the colour centre is 24 mm to the right.
        assert!((kf.center() - Vec3::new(21.0 + 24.0, 0.0, 0.0)).length() < 1e-4, "{}", kf.center());
        assert!((kf.view_dir() - Vec3::Y).length() < 1e-5, "optical axis looks along scanner +Y");
    }

    #[test]
    fn interpolates_poses() {
        let p0 = Transform::from_rotation_translation(Quat::IDENTITY, Vec3::ZERO);
        let p1 = Transform::from_rotation_translation(Quat::from_rotation_z(0.2), Vec3::new(10.0, 0.0, 0.0));
        let mid = interpolate_pose(150, (100, &p0), (200, &p1));
        assert!((mid.translation.x - 5.0).abs() < 1e-5);
        assert!((mid.rotation.angle_between(Quat::IDENTITY) - 0.1).abs() < 1e-4);
    }

    #[test]
    fn encodes_jpeg_when_frame_has_raw_rgb_only() {
        let cam = camera();
        let mut data = vec![0u8; 64 * 48 * 3];
        for (i, b) in data.iter_mut().enumerate() {
            *b = ((i * 17) % 251) as u8;
        }
        let f = ColorFrame {
            image: RgbImage { width: 64, height: 48, channels: 3, data },
            jpeg: None,
            timestamp_ms: 42,
            camera: Some(cam.clone()),
        };
        let kf = RgbKeyframe::new(7, &f, cam, Transform::default(), 55.0, 3).expect("raw RGB encodes to JPEG");
        assert_eq!(&kf.jpeg[..2], &[0xff, 0xd8]);
        let decoded = image::load_from_memory(&kf.jpeg).expect("encoded JPEG decodes").to_rgb8();
        assert_eq!((decoded.width(), decoded.height()), (64, 48));
    }
}
