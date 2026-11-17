//! Prints the in-band IMU header of every frame in a capture: device counter, orientation
//! relative to the first frame, gyro rate and accelerometer magnitude.
//! `cargo run --release -p scancraft-driver-revopoint --example imu_dump -- <capture.bin>`

use scancraft_driver_revopoint::{FrameDecoder, FrameLayout, PacketParser};

fn main() {
    let capture = std::fs::read(std::env::args().nth(1).expect("capture")).unwrap();
    let mut parser = PacketParser::default();
    parser.push(&capture);
    let mut decoder = FrameDecoder::new(FrameLayout::DepthIrPair, 640, 512);
    let (mut first, mut prev, mut prev_counter) = (None, None, 0u32);
    let mut k = 0;
    while let Some(packet) = parser.next_packet() {
        let Ok(frame) = decoder.decode(&packet) else { continue };
        let Some(imu) = frame.imu else { continue };
        let q = imu.orientation;
        let q0 = *first.get_or_insert(q);
        let step = prev.map(|p: glam::Quat| p.angle_between(q).to_degrees()).unwrap_or(0.0);
        println!(
            "{k:3} counter {:8} (+{:5}) total {:6.2}° step {:5.2}° gyro |w| {:6.3} accel |a| {:.3}",
            frame.device_counter,
            frame.device_counter.wrapping_sub(prev_counter),
            q0.angle_between(q).to_degrees(),
            step,
            imu.angular_velocity.length(),
            imu.linear_acceleration.length()
        );
        prev = Some(q);
        prev_counter = frame.device_counter;
        k += 1;
    }
}
