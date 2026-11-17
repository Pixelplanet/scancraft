use byteorder::{LittleEndian, ReadBytesExt, WriteBytesExt};
use std::io::Cursor;
use thiserror::Error;

pub const MAGIC: u32 = 0xAA55AA55;
pub const BASE_HEADER_SIZE: usize = 24;

#[derive(Error, Debug)]
pub enum FramingError {
    #[error("Invalid magic: expected 0xAA55AA55, got 0x{0:08X}")]
    InvalidMagic(u32),
    #[error("Checksum mismatch: calculated 0x{calc:02X}, received 0x{recv:02X}")]
    ChecksumMismatch { calc: u8, recv: u8 },
    #[error("Buffer incomplete: need {need} bytes, have {have}")]
    Incomplete { need: usize, have: usize },
    #[error("IO error: {0}")]
    Io(#[from] std::io::Error),
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Header {
    pub magic: u32,
    pub flags: u16,
    pub payload_len: u32,
    pub timestamp_ms: u64,
    pub packet_id: u32,
    pub extra_hdr_len: u16,
    pub extra_hdr: Vec<u8>,
}

/// Compute XOR checksum over slice
pub fn compute_checksum(bytes: &[u8]) -> u8 {
    bytes.iter().fold(0u8, |acc, &b| acc ^ b)
}

/// Decode packet header from slice
pub fn decode_header(bytes: &[u8]) -> Result<(Header, usize), FramingError> {
    if bytes.len() < BASE_HEADER_SIZE {
        return Err(FramingError::Incomplete {
            need: BASE_HEADER_SIZE,
            have: bytes.len(),
        });
    }

    let mut rdr = Cursor::new(bytes);
    let magic = rdr.read_u32::<LittleEndian>()?;
    if magic != MAGIC {
        return Err(FramingError::InvalidMagic(magic));
    }

    let flags = rdr.read_u16::<LittleEndian>()?;
    let payload_len = rdr.read_u32::<LittleEndian>()?;
    let timestamp_ms = rdr.read_u64::<LittleEndian>()?;
    let packet_id = rdr.read_u32::<LittleEndian>()?;
    let extra_hdr_len = rdr.read_u16::<LittleEndian>()?;

    let total_header_len = BASE_HEADER_SIZE + (extra_hdr_len as usize) + 1; // +1 checksum
    if bytes.len() < total_header_len {
        return Err(FramingError::Incomplete {
            need: total_header_len,
            have: bytes.len(),
        });
    }

    let extra_hdr = bytes[BASE_HEADER_SIZE..BASE_HEADER_SIZE + (extra_hdr_len as usize)].to_vec();
    let expected_chk = bytes[BASE_HEADER_SIZE + (extra_hdr_len as usize)];

    let calc_chk = compute_checksum(&bytes[0..BASE_HEADER_SIZE + (extra_hdr_len as usize)]);
    if calc_chk != expected_chk {
        return Err(FramingError::ChecksumMismatch {
            calc: calc_chk,
            recv: expected_chk,
        });
    }

    Ok((
        Header {
            magic,
            flags,
            payload_len,
            timestamp_ms,
            packet_id,
            extra_hdr_len,
            extra_hdr,
        },
        total_header_len,
    ))
}

/// Encode a complete Revopoint protocol packet
pub fn encode_packet(
    packet_id: u32,
    flags: u16,
    timestamp_ms: u64,
    extra_hdr: &[u8],
    payload: &[u8],
) -> Vec<u8> {
    let mut out = Vec::with_capacity(BASE_HEADER_SIZE + extra_hdr.len() + 1 + payload.len());
    out.write_u32::<LittleEndian>(MAGIC).unwrap();
    out.write_u16::<LittleEndian>(flags).unwrap();
    out.write_u32::<LittleEndian>(payload.len() as u32).unwrap();
    out.write_u64::<LittleEndian>(timestamp_ms).unwrap();
    out.write_u32::<LittleEndian>(packet_id).unwrap();
    out.write_u16::<LittleEndian>(extra_hdr.len() as u16).unwrap();
    out.extend_from_slice(extra_hdr);

    let chk = compute_checksum(&out);
    out.push(chk);
    out.extend_from_slice(payload);
    out
}
