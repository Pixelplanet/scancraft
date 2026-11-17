//! Colour camera model: OpenCV pinhole + Brown-Conrady distortion, plus the rigid transform from
//! the depth camera to the colour camera. See `docs/protocol/color.md`.
//!
//! Frames used here:
//! - **scanner frame** (what depth point clouds use): +X right, +Y forward (optical axis), +Z up.
//! - **optical frame** (OpenCV): +x right, +y down, +z forward. A scanner point `(x, y, z)` is the
//!   optical point `(x, -z, y)`.

use crate::frame::PinholeIntrinsics;
use crate::math::Transform;
use glam::{Mat3, Quat, Vec2, Vec3};
use serde::{Deserialize, Serialize};

/// Scanner frame (+X right, +Y forward, +Z up) -> optical frame (+x right, +y down, +z forward).
#[inline]
pub fn scanner_to_optical(p: Vec3) -> Vec3 {
    Vec3::new(p.x, -p.z, p.y)
}

/// Optical frame -> scanner frame.
#[inline]
pub fn optical_to_scanner(p: Vec3) -> Vec3 {
    Vec3::new(p.x, p.z, -p.y)
}

/// Rotation taking optical-frame vectors to scanner-frame vectors.
pub fn optical_to_scanner_rotation() -> Mat3 {
    Mat3::from_cols(optical_to_scanner(Vec3::X), optical_to_scanner(Vec3::Y), optical_to_scanner(Vec3::Z))
}

/// A calibrated colour camera rigidly attached to a depth camera.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ColorCamera {
    /// Image size the intrinsics refer to.
    pub width: u32,
    pub height: u32,
    pub intrinsics: PinholeIntrinsics,
    /// OpenCV order `k1, k2, p1, p2, k3`, applied to normalised coordinates.
    pub distortion: [f32; 5],
    /// Depth optical frame -> colour optical frame: `p_color = rotation * p_depth + translation`,
    /// mm. The depth optical frame is the frame the depth map's intrinsics describe (on the POP 4
    /// the rectified left IR camera).
    pub rotation: Mat3,
    pub translation: Vec3,
}

impl ColorCamera {
    /// The same camera at another image resolution (intrinsics scaled, distortion unchanged).
    pub fn scaled(&self, width: u32, height: u32) -> Self {
        let sx = width as f32 / self.width as f32;
        let sy = height as f32 / self.height as f32;
        let k = self.intrinsics;
        Self {
            width,
            height,
            intrinsics: PinholeIntrinsics { fx: k.fx * sx, fy: k.fy * sy, cx: k.cx * sx, cy: k.cy * sy },
            ..self.clone()
        }
    }

    /// Depth optical frame -> colour optical frame.
    #[inline]
    pub fn depth_optical_to_color(&self, p: Vec3) -> Vec3 {
        self.rotation * p + self.translation
    }

    /// Scanner-frame point (as in depth point clouds) -> colour optical frame.
    #[inline]
    pub fn sensor_to_color(&self, p: Vec3) -> Vec3 {
        self.depth_optical_to_color(scanner_to_optical(p))
    }

    /// Applies the lens distortion to normalised image coordinates.
    #[inline]
    pub fn distort(&self, x: f32, y: f32) -> (f32, f32) {
        let [k1, k2, p1, p2, k3] = self.distortion;
        let r2 = x * x + y * y;
        let radial = 1.0 + r2 * (k1 + r2 * (k2 + r2 * k3));
        (
            x * radial + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x),
            y * radial + p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y,
        )
    }

    /// Removes the lens distortion from normalised image coordinates (fixed-point iteration).
    pub fn undistort(&self, xd: f32, yd: f32) -> (f32, f32) {
        let (mut x, mut y) = (xd, yd);
        for _ in 0..8 {
            let (dx, dy) = self.distort(x, y);
            x += xd - dx;
            y += yd - dy;
        }
        (x, y)
    }

    /// Projects a colour-optical-frame point to a pixel (distorted image). `None` behind the
    /// camera or outside the image. The polynomial distortion model folds back far outside the
    /// calibrated field, so points beyond ~1.5x the image half-diagonal are rejected first.
    #[inline]
    pub fn project_color(&self, p: Vec3) -> Option<Vec2> {
        if p.z <= 1e-3 {
            return None;
        }
        let (x, y) = (p.x / p.z, p.y / p.z);
        let k = self.intrinsics;
        let lim_x = 1.5 * self.width as f32 / k.fx;
        let lim_y = 1.5 * self.height as f32 / k.fy;
        if x.abs() > lim_x || y.abs() > lim_y {
            return None;
        }
        let (xd, yd) = self.distort(x, y);
        let px = Vec2::new(k.fx * xd + k.cx, k.fy * yd + k.cy);
        (px.x >= 0.0 && px.y >= 0.0 && px.x < self.width as f32 && px.y < self.height as f32).then_some(px)
    }

    /// Projects a scanner-frame point; returns the pixel and the depth along the colour camera's
    /// optical axis (mm).
    #[inline]
    pub fn project_sensor(&self, p: Vec3) -> Option<(Vec2, f32)> {
        let c = self.sensor_to_color(p);
        self.project_color(c).map(|px| (px, c.z))
    }

    /// Back-projects a (distorted) pixel at colour-camera depth `z` into the colour optical frame.
    pub fn unproject_color(&self, px: Vec2, z: f32) -> Vec3 {
        let k = self.intrinsics;
        let (x, y) = self.undistort((px.x - k.cx) / k.fx, (px.y - k.cy) / k.fy);
        Vec3::new(x * z, y * z, z)
    }

    /// Colour optical frame -> scanner frame.
    pub fn color_to_sensor(&self, p: Vec3) -> Vec3 {
        optical_to_scanner(self.rotation.transpose() * (p - self.translation))
    }

    /// Pose of the colour camera's optical frame in the world, given the depth sensor's pose
    /// (`world <- scanner frame`, as tracked). The result maps colour optical coordinates (OpenCV:
    /// x right, y down, z forward) to world coordinates, i.e. it is the camera-to-world matrix
    /// most texture-mapping and splatting tools expect (after their own axis conventions).
    pub fn world_from_color(&self, world_from_sensor: &Transform) -> Transform {
        // color -> depth optical: R^T (p - t); depth optical -> scanner: fixed axis swap.
        let rot_sensor_from_color = optical_to_scanner_rotation() * self.rotation.transpose();
        let t_sensor_from_color = optical_to_scanner(-(self.rotation.transpose() * self.translation));
        let rotation = world_from_sensor.rotation * Quat::from_mat3(&rot_sensor_from_color);
        let translation = world_from_sensor.transform_point3(t_sensor_from_color);
        Transform::from_rotation_translation(rotation.normalize(), translation)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn camera() -> ColorCamera {
        ColorCamera {
            width: 1280,
            height: 1024,
            intrinsics: PinholeIntrinsics { fx: 1564.9, fy: 1564.9, cx: 635.9, cy: 516.4 },
            distortion: [0.134, -0.715, 0.0012, 0.0001, 1.339],
            rotation: Mat3::from_rotation_y(0.1),
            translation: Vec3::new(-24.1, 0.08, -2.71),
        }
    }

    #[test]
    fn axis_swaps_round_trip() {
        let p = Vec3::new(1.0, 2.0, 3.0);
        assert_eq!(optical_to_scanner(scanner_to_optical(p)), p);
        assert_eq!(optical_to_scanner_rotation() * scanner_to_optical(p), p);
    }

    #[test]
    fn projection_round_trips_through_distortion() {
        let cam = camera();
        for p in [Vec3::new(10.0, -20.0, 300.0), Vec3::new(-60.0, 40.0, 250.0)] {
            let px = cam.project_color(p).expect("in view");
            let back = cam.unproject_color(px, p.z);
            assert!((back - p).length() < 1e-2, "{p} -> {px} -> {back}");
        }
        assert!(cam.project_color(Vec3::new(0.0, 0.0, -10.0)).is_none());
    }

    #[test]
    fn world_pose_agrees_with_point_transform() {
        let cam = camera();
        let world_from_sensor =
            Transform::from_rotation_translation(Quat::from_euler(glam::EulerRot::XYZ, 0.2, -0.3, 0.5), Vec3::new(5.0, -7.0, 11.0));
        let p_sensor = Vec3::new(12.0, 280.0, -9.0);
        let p_world = world_from_sensor.transform_point3(p_sensor);
        let p_color = cam.sensor_to_color(p_sensor);
        let via_pose = cam.world_from_color(&world_from_sensor).transform_point3(p_color);
        assert!((via_pose - p_world).length() < 1e-3, "{via_pose} vs {p_world}");
        assert!((cam.color_to_sensor(p_color) - p_sensor).length() < 1e-3);
    }
}
