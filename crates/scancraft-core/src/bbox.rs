use glam::Vec3;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct BoundingBox {
    pub min: Vec3,
    pub max: Vec3,
}

impl Default for BoundingBox {
    fn default() -> Self {
        Self {
            min: Vec3::splat(f32::INFINITY),
            max: Vec3::splat(f32::NEG_INFINITY),
        }
    }
}

impl BoundingBox {
    pub fn new(min: Vec3, max: Vec3) -> Self {
        Self { min, max }
    }

    pub fn from_arrays(min: [f32; 3], max: [f32; 3]) -> Self {
        Self {
            min: Vec3::from_slice(&min),
            max: Vec3::from_slice(&max),
        }
    }

    pub fn is_empty(&self) -> bool {
        self.min.x > self.max.x || self.min.y > self.max.y || self.min.z > self.max.z
    }

    pub fn extend(&mut self, point: Vec3) {
        self.min = self.min.min(point);
        self.max = self.max.max(point);
    }

    pub fn center(&self) -> Vec3 {
        (self.min + self.max) * 0.5
    }

    pub fn extents(&self) -> Vec3 {
        (self.max - self.min).max(Vec3::ZERO)
    }

    pub fn radius(&self) -> f32 {
        self.extents().length() * 0.5
    }

    pub fn contains(&self, p: [f32; 3]) -> bool {
        p[0] >= self.min.x && p[0] <= self.max.x
            && p[1] >= self.min.y && p[1] <= self.max.y
            && p[2] >= self.min.z && p[2] <= self.max.z
    }

    pub fn contains_vec3(&self, p: Vec3) -> bool {
        p.x >= self.min.x && p.x <= self.max.x
            && p.y >= self.min.y && p.y <= self.max.y
            && p.z >= self.min.z && p.z <= self.max.z
    }
}
