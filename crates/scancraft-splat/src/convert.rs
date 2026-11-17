//! ScanCraft <-> Brush conventions.
//!
//! * World: Brush trains in metres (its defaults and the PLY export assume it); ScanCraft's world
//!   is in millimetres. Axes are kept as they are — the splat lives in ScanCraft's Z-up world,
//!   scaled by [`MM_TO_M`]. The exported PLY is scaled back to millimetres so it overlays the
//!   mesh and point cloud.
//! * Camera: Brush's `Camera` is camera-to-world with an OpenCV camera-local frame (+X right,
//!   +Y down, +Z forward), `fov_x/fov_y` derived from the focal lengths and `center_uv` the
//!   principal point divided by the image size.
//! * Brush uses glam 0.30, ScanCraft glam 0.29: values cross the boundary as arrays.

use crate::types::PosedImage;
use brush_render::camera::{Camera, focal_to_fov};
use brush_render::kernels::camera_model::CameraModel;
use brush_render::kernels::camera_model::radial_tangential_8::RadialTangential8Params;

/// ScanCraft world units (mm) to Brush training units (m).
pub const MM_TO_M: f32 = 1e-3;

pub(crate) fn bvec3(v: glam::Vec3) -> brush_glam::Vec3 {
    brush_glam::Vec3::from_array(v.to_array())
}

pub(crate) fn bquat(q: glam::Quat) -> brush_glam::Quat {
    brush_glam::Quat::from_array(q.to_array())
}

/// Brush lens model for an image's distortion.
pub fn camera_model(img: &PosedImage) -> CameraModel {
    match img.intrinsics.distortion {
        Some(d) if !d.is_zero() => CameraModel::RadialTangential8(RadialTangential8Params {
            k1: d.k1,
            k2: d.k2,
            k3: d.k3,
            k4: 0.0,
            k5: 0.0,
            k6: 0.0,
            p1: d.p1,
            p2: d.p2,
        }),
        _ => CameraModel::Pinhole,
    }
}

/// Brush camera (metres) for a posed image.
pub fn brush_camera(img: &PosedImage) -> Camera {
    let c2w = img.camera_to_world_opencv();
    let (_, rotation, translation) = c2w.to_scale_rotation_translation();
    let model = camera_model(img);
    let k = &img.intrinsics;
    let fov_x = focal_to_fov(k.fx as f64, img.width, &model);
    let fov_y = focal_to_fov(k.fy as f64, img.height, &model);
    let center_uv = brush_glam::vec2(k.cx / img.width as f32, k.cy / img.height as f32);
    Camera::new(bvec3(translation * MM_TO_M), bquat(rotation.normalize()), fov_x, fov_y, center_uv, model)
}

/// Brush camera (metres) for a look-at view in ScanCraft world units, e.g. the viewport's orbit
/// camera. `fov_y` in radians, square pixels, principal point at the image centre.
pub fn look_at_camera(eye: glam::Vec3, target: glam::Vec3, up: glam::Vec3, fov_y: f32, width: u32, height: u32) -> Camera {
    let forward = (target - eye).normalize();
    let right = forward.cross(up).normalize();
    let down = forward.cross(right);
    let rot = glam::Quat::from_mat3(&glam::Mat3::from_cols(right, down, forward)).normalize();
    let fy = 0.5 * height as f64 / (fov_y as f64 * 0.5).tan();
    let fov_x = 2.0 * (0.5 * width as f64 / fy).atan();
    Camera::new(
        bvec3(eye * MM_TO_M),
        bquat(rot),
        fov_x,
        fov_y as f64,
        brush_glam::vec2(0.5, 0.5),
        CameraModel::Pinhole,
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::{CameraAxes, Intrinsics};
    use glam::{Affine3A, Vec3};

    #[test]
    fn brush_camera_projects_like_posed_image() {
        let img = PosedImage {
            name: "x".into(),
            width: 640,
            height: 480,
            rgb: vec![0; 640 * 480 * 3],
            mask: None,
            intrinsics: Intrinsics { fx: 700.0, fy: 690.0, cx: 330.0, cy: 236.0, distortion: None },
            camera_to_world: Affine3A::from_rotation_translation(
                glam::Quat::from_rotation_z(0.4) * glam::Quat::from_rotation_x(-0.3),
                Vec3::new(30.0, -250.0, 90.0),
            ),
            axes: CameraAxes::ScanCraft,
        };
        let cam = brush_camera(&img);
        let size = brush_glam::uvec2(640, 480);
        let pin = cam.build_pinhole_params(size);
        for p in [Vec3::new(0.0, 0.0, 40.0), Vec3::new(20.0, 10.0, 60.0)] {
            let (u, v, _) = img.project(p).unwrap();
            let local = cam.world_to_local().transform_point3(bvec3(p * MM_TO_M));
            let (bu, bv) = (pin.fx * local.x / local.z + pin.cx, pin.fy * local.y / local.z + pin.cy);
            assert!((u - bu).abs() < 1e-2 && (v - bv).abs() < 1e-2, "{u},{v} vs {bu},{bv}");
        }
    }

    #[test]
    fn look_at_points_at_target() {
        let cam = look_at_camera(Vec3::new(0.0, -300.0, 100.0), Vec3::new(0.0, 0.0, 40.0), Vec3::Z, 0.8, 400, 300);
        let local = cam.world_to_local().transform_point3(bvec3(Vec3::new(0.0, 0.0, 40.0) * MM_TO_M));
        assert!(local.x.abs() < 1e-5 && local.y.abs() < 1e-5 && local.z > 0.0);
        // World up appears towards the top of the image (negative OpenCV y).
        let above = cam.world_to_local().transform_point3(bvec3(Vec3::new(0.0, 0.0, 80.0) * MM_TO_M));
        assert!(above.y < 0.0);
    }
}
