use byteorder::{LittleEndian, ReadBytesExt};
use glam::{Quat, Vec3};
use scancraft_core::ImuSample;
use std::io::Cursor;

pub const IMU_PACKET_SIZE: usize = 60;

/// Parses a 60-byte binary IMU telemetry packet from Revopoint port 9999
pub fn parse_imu_packet(bytes: &[u8]) -> Option<ImuSample> {
    if bytes.len() < IMU_PACKET_SIZE {
        return None;
    }

    let mut rdr = Cursor::new(&bytes[0..IMU_PACKET_SIZE]);
    let timestamp_ms = rdr.read_u64::<LittleEndian>().ok()?;

    let qw = rdr.read_f32::<LittleEndian>().ok()?;
    let qx = rdr.read_f32::<LittleEndian>().ok()?;
    let qy = rdr.read_f32::<LittleEndian>().ok()?;
    let qz = rdr.read_f32::<LittleEndian>().ok()?;

    let gx = rdr.read_f32::<LittleEndian>().ok()?;
    let gy = rdr.read_f32::<LittleEndian>().ok()?;
    let gz = rdr.read_f32::<LittleEndian>().ok()?;

    let ax = rdr.read_f32::<LittleEndian>().ok()?;
    let ay = rdr.read_f32::<LittleEndian>().ok()?;
    let az = rdr.read_f32::<LittleEndian>().ok()?;

    let pitch = rdr.read_f32::<LittleEndian>().ok()?;
    let roll = rdr.read_f32::<LittleEndian>().ok()?;
    let yaw = rdr.read_f32::<LittleEndian>().ok()?;

    Some(ImuSample {
        timestamp_ms,
        orientation: Quat::from_xyzw(qx, qy, qz, qw),
        angular_velocity: Vec3::new(gx, gy, gz),
        linear_acceleration: Vec3::new(ax, ay, az),
        euler_degrees: Vec3::new(pitch, roll, yaw),
    })
}
