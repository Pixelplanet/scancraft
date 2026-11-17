use glam::{Mat4, Vec3};
use scancraft_core::BoundingBox;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CameraPreset {
    Front,
    Back,
    Top,
    Bottom,
    Right,
    Left,
    Isometric,
}

#[derive(Debug, Clone)]
pub struct Camera {
    pub eye: Vec3,
    pub target: Vec3,
    pub up: Vec3,
    pub aspect_ratio: f32,
    pub fov_y_rad: f32,
    pub z_near: f32,
    pub z_far: f32,
}

impl Default for Camera {
    fn default() -> Self {
        Self {
            // Default isometric CAD framing centered at origin (0, 0, 20mm)
            eye: Vec3::new(220.0, -340.0, 180.0),
            target: Vec3::new(0.0, 0.0, 20.0),
            up: Vec3::Z,
            aspect_ratio: 16.0 / 9.0,
            fov_y_rad: 45.0f32.to_radians(),
            z_near: 5.0,
            z_far: 5000.0,
        }
    }
}

impl Camera {
    pub fn view_matrix(&self) -> Mat4 {
        Mat4::look_at_rh(self.eye, self.target, self.up)
    }

    pub fn projection_matrix(&self) -> Mat4 {
        Mat4::perspective_rh(self.fov_y_rad, self.aspect_ratio, self.z_near, self.z_far)
    }

    pub fn view_projection(&self) -> Mat4 {
        self.projection_matrix() * self.view_matrix()
    }

    /// Orbit the camera around the target point
    pub fn orbit(&mut self, delta_azimuth: f32, delta_elevation: f32) {
        let offset = self.eye - self.target;
        let radius = offset.length();
        if radius < 1e-4 {
            return;
        }

        let azimuth = offset.y.atan2(offset.x) + delta_azimuth;
        let mut elevation = (offset.z / radius).asin() + delta_elevation;

        let max_elev = 89.0f32.to_radians();
        elevation = elevation.clamp(-max_elev, max_elev);

        let cos_elev = elevation.cos();
        let new_offset = Vec3::new(
            radius * cos_elev * azimuth.cos(),
            radius * cos_elev * azimuth.sin(),
            radius * elevation.sin(),
        );

        self.eye = self.target + new_offset;
    }

    /// Pan the camera along its view plane
    pub fn pan(&mut self, delta_x: f32, delta_y: f32) {
        let forward = (self.target - self.eye).normalize();
        let right = forward.cross(self.up).normalize();
        let up = right.cross(forward).normalize();

        let delta = right * delta_x + up * delta_y;
        self.eye += delta;
        self.target += delta;
    }

    /// Zoom the camera towards the target point
    pub fn zoom(&mut self, factor: f32) {
        let offset = self.eye - self.target;
        let new_offset = offset * factor;
        if new_offset.length() > 20.0 && new_offset.length() < 5000.0 {
            self.eye = self.target + new_offset;
        }
    }

    /// Set camera to a standard preset orientation while maintaining view distance
    pub fn set_view_preset(&mut self, preset: CameraPreset) {
        let dist = (self.eye - self.target).length().max(100.0);
        match preset {
            CameraPreset::Front => {
                self.eye = self.target + Vec3::new(0.0, -dist, 0.0);
                self.up = Vec3::Z;
            }
            CameraPreset::Back => {
                self.eye = self.target + Vec3::new(0.0, dist, 0.0);
                self.up = Vec3::Z;
            }
            CameraPreset::Top => {
                self.eye = self.target + Vec3::new(0.0, 0.0, dist);
                self.up = Vec3::Y;
            }
            CameraPreset::Bottom => {
                self.eye = self.target + Vec3::new(0.0, 0.0, -dist);
                self.up = Vec3::Y;
            }
            CameraPreset::Right => {
                self.eye = self.target + Vec3::new(dist, 0.0, 0.0);
                self.up = Vec3::Z;
            }
            CameraPreset::Left => {
                self.eye = self.target + Vec3::new(-dist, 0.0, 0.0);
                self.up = Vec3::Z;
            }
            CameraPreset::Isometric => {
                let norm = Vec3::new(1.0, -1.0, 1.0).normalize();
                self.eye = self.target + norm * dist;
                self.up = Vec3::Z;
            }
        }
    }

    /// Center view on bounding box and set comfortable distance
    pub fn fit_bounding_box(&mut self, bbox: &BoundingBox) {
        if bbox.is_empty() {
            return;
        }
        self.target = bbox.center();
        let radius = bbox.radius().max(20.0);
        // The bounding sphere fills about 80 % of the view height.
        let dist = radius / (self.fov_y_rad * 0.5).sin() * 1.25;
        let mut dir = self.eye - self.target;
        if dir.length_squared() < 1e-4 {
            dir = Vec3::new(0.0, -1.0, 0.4);
        }
        self.eye = self.target + dir.normalize() * dist;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_camera_projection() {
        let mut cam = Camera::default();
        let target_pt = cam.target;
        let view_pt = cam.view_matrix().transform_point3(target_pt);
        assert!(view_pt.z < -cam.z_near && view_pt.z > -cam.z_far, "view_pt.z was {}", view_pt.z);
        let ndc = cam.projection_matrix().project_point3(view_pt);
        assert!(ndc.x.abs() < 1e-4 && ndc.y.abs() < 1e-4, "NDC was {:?}", ndc);

        // Test with a bbox
        let bbox = BoundingBox::from_arrays([-50.0, -50.0, 200.0], [50.0, 50.0, 300.0]);
        cam.fit_bounding_box(&bbox);
        let view_center = cam.view_matrix().transform_point3(bbox.center());
        let ndc_center = cam.projection_matrix().project_point3(view_center);
        assert!(ndc_center.x.abs() < 1e-4 && ndc_center.y.abs() < 1e-4, "NDC center was {:?}", ndc_center);

        let corner = Vec3::new(40.0, 40.0, 290.0);
        let view_corner = cam.view_matrix().transform_point3(corner);
        assert!(view_corner.z < -cam.z_near && view_corner.z > -cam.z_far, "view_corner.z was {}", view_corner.z);
        let ndc_corner = cam.projection_matrix().project_point3(view_corner);
        assert!(ndc_corner.x.abs() <= 1.0 && ndc_corner.y.abs() <= 1.0, "NDC corner was {:?}", ndc_corner);
    }

    #[test]
    fn view_presets_place_eye_on_expected_axis() {
        let mut cam = Camera::default();
        let target = cam.target;

        cam.set_view_preset(CameraPreset::Top);
        assert!(cam.eye.z > target.z, "Top eye should be above target: {:?}", cam.eye);

        cam.set_view_preset(CameraPreset::Bottom);
        assert!(cam.eye.z < target.z, "Bottom eye should be below target: {:?}", cam.eye);

        cam.set_view_preset(CameraPreset::Front);
        assert!(cam.eye.y < target.y, "Front eye should be in front (-Y): {:?}", cam.eye);

        cam.set_view_preset(CameraPreset::Right);
        assert!(cam.eye.x > target.x, "Right eye should be +X: {:?}", cam.eye);

        cam.set_view_preset(CameraPreset::Left);
        assert!(cam.eye.x < target.x, "Left eye should be -X: {:?}", cam.eye);
    }

    #[test]
    fn top_and_bottom_presets_are_mirrored() {
        let mut cam = Camera::default();
        let target = cam.target;

        cam.set_view_preset(CameraPreset::Top);
        let top = cam.eye;
        cam.set_view_preset(CameraPreset::Bottom);
        let bottom = cam.eye;

        assert!((top.z - target.z) > 0.0);
        assert!((bottom.z - target.z) < 0.0);
        assert!(
            ((top.z - target.z) + (bottom.z - target.z)).abs() < 1e-3,
            "Top/Bottom should be equidistant: top={:?} bottom={:?}",
            top,
            bottom
        );
    }
}
