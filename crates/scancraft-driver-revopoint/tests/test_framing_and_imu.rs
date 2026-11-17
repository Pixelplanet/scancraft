use scancraft_driver_revopoint::{
    decode_header, encode_packet, parse_imu_packet, MAGIC,
};

#[test]
fn test_packet_encode_decode() {
    let packet_id = 42;
    let flags = 0x01;
    let ts = 1730000000000;
    let extra_hdr = vec![0x10, 0x20, 0x30];
    let payload = b"Hello POP 4 Scanner";

    let encoded = encode_packet(packet_id, flags, ts, &extra_hdr, payload);
    assert!(encoded.len() > 24);

    let (hdr, hdr_size) = decode_header(&encoded).expect("Failed to decode header");
    assert_eq!(hdr.magic, MAGIC);
    assert_eq!(hdr.packet_id, packet_id);
    assert_eq!(hdr.flags, flags);
    assert_eq!(hdr.timestamp_ms, ts);
    assert_eq!(hdr.extra_hdr, extra_hdr);
    assert_eq!(hdr.payload_len, payload.len() as u32);

    let decoded_payload = &encoded[hdr_size..hdr_size + (hdr.payload_len as usize)];
    assert_eq!(decoded_payload, payload);
}

#[test]
fn test_imu_parse() {
    // 60-byte synthetic packet: timestamp + 13 floats
    let mut bytes = Vec::with_capacity(60);
    // timestamp_ms = 5000 (u64 little-endian)
    bytes.extend_from_slice(&5000u64.to_le_bytes());
    // q: w=1.0, x=0.0, y=0.0, z=0.0
    bytes.extend_from_slice(&1.0f32.to_le_bytes());
    bytes.extend_from_slice(&0.0f32.to_le_bytes());
    bytes.extend_from_slice(&0.0f32.to_le_bytes());
    bytes.extend_from_slice(&0.0f32.to_le_bytes());
    // gyro: 0.1, 0.2, 0.3
    bytes.extend_from_slice(&0.1f32.to_le_bytes());
    bytes.extend_from_slice(&0.2f32.to_le_bytes());
    bytes.extend_from_slice(&0.3f32.to_le_bytes());
    // accel: 0.0, 0.0, 1.0
    bytes.extend_from_slice(&0.0f32.to_le_bytes());
    bytes.extend_from_slice(&0.0f32.to_le_bytes());
    bytes.extend_from_slice(&1.0f32.to_le_bytes());
    // attitude: 10.0, 20.0, 30.0
    bytes.extend_from_slice(&10.0f32.to_le_bytes());
    bytes.extend_from_slice(&20.0f32.to_le_bytes());
    bytes.extend_from_slice(&30.0f32.to_le_bytes());

    let sample = parse_imu_packet(&bytes).expect("Failed to parse IMU sample");
    assert_eq!(sample.timestamp_ms, 5000);
    assert_eq!(sample.linear_acceleration.z, 1.0);
    assert_eq!(sample.angular_velocity.x, 0.1);
    assert_eq!(sample.euler_degrees.x, 10.0);
}
