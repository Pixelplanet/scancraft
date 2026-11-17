use bytemuck::{Pod, Zeroable};
use serde::{Deserialize, Serialize};

#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Pod, Zeroable)]
pub struct ColorRgba {
    pub r: u8,
    pub g: u8,
    pub b: u8,
    pub a: u8,
}

impl ColorRgba {
    pub const WHITE: Self = Self { r: 255, g: 255, b: 255, a: 255 };
    pub const BLACK: Self = Self { r: 0, g: 0, b: 0, a: 255 };
    pub const GRAY: Self = Self { r: 128, g: 128, b: 128, a: 255 };
    pub const RED: Self = Self { r: 255, g: 0, b: 0, a: 255 };
    pub const GREEN: Self = Self { r: 0, g: 255, b: 0, a: 255 };
    pub const BLUE: Self = Self { r: 0, g: 0, b: 255, a: 255 };

    pub fn new(r: u8, g: u8, b: u8, a: u8) -> Self {
        Self { r, g, b, a }
    }

    pub fn to_array(&self) -> [u8; 4] {
        [self.r, self.g, self.b, self.a]
    }
}

/// A standard 3D point in the ScanCraft scanning system.
/// Uses a compact representation convertible to GPU vertex buffers.
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Pod, Zeroable)]
pub struct Point3D {
    pub position: [f32; 3],
    pub normal: [f32; 3],
    pub color: ColorRgba,
    pub confidence: f32,
}

impl Default for Point3D {
    fn default() -> Self {
        Self {
            position: [0.0, 0.0, 0.0],
            normal: [0.0, 0.0, 1.0],
            color: ColorRgba::WHITE,
            confidence: 1.0,
        }
    }
}

impl Point3D {
    pub fn new(position: [f32; 3]) -> Self {
        Self {
            position,
            ..Default::default()
        }
    }

    pub fn with_color(mut self, color: ColorRgba) -> Self {
        self.color = color;
        self
    }

    pub fn with_normal(mut self, normal: [f32; 3]) -> Self {
        self.normal = normal;
        self
    }
}
