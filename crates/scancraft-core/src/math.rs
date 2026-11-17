pub use glam::{EulerRot, Mat4, Quat, Vec2, Vec3, Vec4};
use serde::{Deserialize, Serialize};

/// 3D rigid body transformation
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Transform {
    pub translation: Vec3,
    pub rotation: Quat,
    pub scale: Vec3,
}

impl Default for Transform {
    fn default() -> Self {
        Self {
            translation: Vec3::ZERO,
            rotation: Quat::IDENTITY,
            scale: Vec3::ONE,
        }
    }
}

impl Transform {
    pub fn from_translation(translation: Vec3) -> Self {
        Self {
            translation,
            rotation: Quat::IDENTITY,
            scale: Vec3::ONE,
        }
    }

    pub fn from_rotation_translation(rotation: Quat, translation: Vec3) -> Self {
        Self {
            translation,
            rotation,
            scale: Vec3::ONE,
        }
    }

    pub fn to_mat4(&self) -> Mat4 {
        Mat4::from_scale_rotation_translation(self.scale, self.rotation, self.translation)
    }

    pub fn transform_point3(&self, point: Vec3) -> Vec3 {
        self.to_mat4().transform_point3(point)
    }

    pub fn transform_vector3(&self, vector: Vec3) -> Vec3 {
        self.to_mat4().transform_vector3(vector)
    }

    pub fn inverse(&self) -> Self {
        let inv_rot = self.rotation.inverse();
        let inv_scale = 1.0 / self.scale;
        let inv_trans = inv_rot * (-self.translation * inv_scale);
        Self {
            translation: inv_trans,
            rotation: inv_rot,
            scale: inv_scale,
        }
    }
}
