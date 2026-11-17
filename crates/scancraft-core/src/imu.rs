use glam::{Quat, Vec3};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct ImuSample {
    pub timestamp_ms: u64,
    pub orientation: Quat,
    pub angular_velocity: Vec3,
    pub linear_acceleration: Vec3,
    pub euler_degrees: Vec3, // Pitch, Roll, Yaw
}

impl Default for ImuSample {
    fn default() -> Self {
        Self {
            timestamp_ms: 0,
            orientation: Quat::IDENTITY,
            angular_velocity: Vec3::ZERO,
            linear_acceleration: Vec3::new(0.0, 0.0, 1.0),
            euler_degrees: Vec3::ZERO,
        }
    }
}
