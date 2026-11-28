//! Writes marker lists in the layout of Revo Scan's `marker_framework.mkf` (see
//! `docs/protocol/revo-marker-framework.md`), so the app's Import Scan reads them like a Revo
//! project's.

use std::path::Path;

/// One marker record (pass frame, mm).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MkfMarker {
    pub centre: [f64; 3],
    pub normal: [f64; 3],
    pub radius_mm: f32,
}

/// The file's bytes: version 3, the count, 104-byte records. The fields Revo fills but ScanCraft
/// does not use get Revo-like values (-1, a score, zeros, a residual, zero padding).
pub fn encode(markers: &[MkfMarker]) -> Vec<u8> {
    let mut b = Vec::with_capacity(8 + 104 * markers.len());
    b.extend_from_slice(&3u32.to_le_bytes());
    b.extend_from_slice(&(markers.len() as u32).to_le_bytes());
    for (k, m) in markers.iter().enumerate() {
        for v in m.centre.iter().chain(&m.normal) {
            b.extend_from_slice(&v.to_le_bytes());
        }
        b.extend_from_slice(&(-1i32).to_le_bytes());
        b.extend_from_slice(&(200.0f32 + 7.0 * k as f32).to_le_bytes());
        b.extend_from_slice(&[0u8; 36]);
        b.extend_from_slice(&m.radius_mm.to_le_bytes());
        b.extend_from_slice(&0.25f32.to_le_bytes());
        b.extend_from_slice(&[0u8; 4]);
    }
    b
}

pub fn write(path: &Path, markers: &[MkfMarker]) -> std::io::Result<()> {
    std::fs::write(path, encode(markers))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn scancraft_io_reads_what_is_written() {
        let markers = [
            MkfMarker { centre: [1.5, -2.25, 30.0], normal: [0.0, 0.6, 0.8], radius_mm: 3.0 },
            MkfMarker { centre: [-40.0, 12.0, 5.125], normal: [1.0, 0.0, 0.0], radius_mm: 3.05 },
        ];
        let bytes = encode(&markers);
        assert_eq!(bytes.len(), 8 + 2 * 104);
        let back = scancraft_io::revo::parse_marker_framework(&bytes).unwrap();
        assert_eq!(back.len(), 2);
        for (m, r) in markers.iter().zip(&back) {
            assert_eq!(r.centre, m.centre);
            assert_eq!(r.normal, m.normal);
            assert_eq!(r.radius_mm, m.radius_mm);
            assert_eq!(r.id, -1);
        }
        assert!(scancraft_io::revo::parse_marker_framework(&encode(&[])).unwrap().is_empty());
    }
}
