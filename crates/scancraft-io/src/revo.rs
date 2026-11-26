//! Files Revo Scan 6 writes into its project folders that are useful next to an imported scan.
//!
//! `marker_framework.mkf` lists the marker stickers Revo tracked in that scan, in the frame of the
//! project's `raw_preview.ply` (mm). Layout (little endian; verified on three projects, see
//! `docs/protocol/revo-marker-framework.md`):
//!
//! ```text
//! 0   u32  version (3)
//! 4   u32  count
//! 8   count records of 104 bytes:
//!     0   f64 x3  centre, mm
//!     24  f64 x3  unit normal, out of the surface the sticker is on
//!     48  i32     always -1 in the files seen (an id or group; not used)
//!     52  f32     unknown, 12 .. 2500 (not used)
//!     56  36 bytes zero
//!     92  f32     sticker radius, mm (about 3.05)
//!     96  f32     unknown, 0 .. 1.4 (not used)
//!     100 4 bytes padding (uninitialised)
//! ```

use std::path::{Path, PathBuf};

use crate::IoError;

/// The marker file's name in a Revo Scan project folder.
pub const MARKER_FRAMEWORK_FILE: &str = "marker_framework.mkf";

const HEADER_BYTES: usize = 8;
const RECORD_BYTES: usize = 104;
/// Versions whose layout is known.
const KNOWN_VERSIONS: &[u32] = &[3];

/// One marker of a `marker_framework.mkf` (scan frame, mm).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RevoMarker {
    pub centre: [f64; 3],
    /// Unit length, pointing out of the surface the sticker is on.
    pub normal: [f64; 3],
    pub radius_mm: f32,
    /// The record's i32 at offset 48 (-1 in every file seen).
    pub id: i32,
}

/// Reads Revo's `marker_framework.mkf`.
pub fn read_marker_framework(path: &Path) -> Result<Vec<RevoMarker>, IoError> {
    let bytes = std::fs::read(path)?;
    parse_marker_framework(&bytes).map_err(|e| match e {
        IoError::Parse(m) => IoError::Parse(format!("{}: {m}", path.display())),
        IoError::Truncated(m) => IoError::Truncated(format!("{}: {m}", path.display())),
        IoError::Unsupported(m) => IoError::Unsupported(format!("{}: {m}", path.display())),
        other => other,
    })
}

/// Parses the bytes of a `marker_framework.mkf`. The length must be exactly the header plus
/// `count` records, every centre finite and every normal of (nearly) unit length.
pub fn parse_marker_framework(bytes: &[u8]) -> Result<Vec<RevoMarker>, IoError> {
    if bytes.len() < HEADER_BYTES {
        return Err(IoError::Truncated(format!("{} bytes, shorter than the {HEADER_BYTES}-byte header", bytes.len())));
    }
    let u32_at = |o: usize| u32::from_le_bytes([bytes[o], bytes[o + 1], bytes[o + 2], bytes[o + 3]]);
    let version = u32_at(0);
    if !KNOWN_VERSIONS.contains(&version) {
        return Err(IoError::Unsupported(format!("marker file version {version}")));
    }
    let count = u32_at(4) as usize;
    let expected = count.checked_mul(RECORD_BYTES).and_then(|n| n.checked_add(HEADER_BYTES));
    match expected {
        Some(n) if n == bytes.len() => {}
        Some(n) if n > bytes.len() => return Err(IoError::Truncated(format!("{count} markers need {n} bytes, the file has {}", bytes.len()))),
        _ => return Err(IoError::Parse(format!("{count} markers do not fill the {} bytes of the file", bytes.len()))),
    }
    let mut out = Vec::with_capacity(count);
    for (k, r) in bytes[HEADER_BYTES..].chunks_exact(RECORD_BYTES).enumerate() {
        let f64_at = |o: usize| f64::from_le_bytes(r[o..o + 8].try_into().expect("8 bytes"));
        let f32_at = |o: usize| f32::from_le_bytes(r[o..o + 4].try_into().expect("4 bytes"));
        let centre = [f64_at(0), f64_at(8), f64_at(16)];
        let normal = [f64_at(24), f64_at(32), f64_at(40)];
        let radius_mm = f32_at(92);
        let length = normal.iter().map(|v| v * v).sum::<f64>().sqrt();
        if !centre.iter().all(|v| v.is_finite()) {
            return Err(IoError::Parse(format!("marker {k} has no finite centre")));
        }
        let unit = (length - 1.0).abs() < 1e-3;
        if !unit {
            return Err(IoError::Parse(format!("marker {k}'s normal is {length:.4} long, not a unit vector")));
        }
        if !radius_mm.is_finite() || radius_mm < 0.0 {
            return Err(IoError::Parse(format!("marker {k} has radius {radius_mm}")));
        }
        out.push(RevoMarker { centre, normal: normal.map(|v| v / length), radius_mm, id: i32::from_le_bytes(r[48..52].try_into().expect("4 bytes")) });
    }
    Ok(out)
}

/// The `marker_framework.mkf` in the folder of the scan file `scan`, if there is one.
pub fn marker_framework_beside(scan: &Path) -> Option<PathBuf> {
    let path = scan.parent()?.join(MARKER_FRAMEWORK_FILE);
    path.is_file().then_some(path)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A file in Revo's layout with `markers` (centre, normal, radius); unknown fields filled like
    /// Revo's (-1, a score, zeros, junk padding).
    fn mkf(markers: &[([f64; 3], [f64; 3], f32)]) -> Vec<u8> {
        let mut b = Vec::new();
        b.extend_from_slice(&3u32.to_le_bytes());
        b.extend_from_slice(&(markers.len() as u32).to_le_bytes());
        for (k, (c, n, r)) in markers.iter().enumerate() {
            let start = b.len();
            for v in c.iter().chain(n) {
                b.extend_from_slice(&v.to_le_bytes());
            }
            b.extend_from_slice(&(-1i32).to_le_bytes());
            b.extend_from_slice(&(216.1f32 + k as f32).to_le_bytes());
            b.extend_from_slice(&[0u8; 36]);
            b.extend_from_slice(&r.to_le_bytes());
            b.extend_from_slice(&0.2584f32.to_le_bytes());
            b.extend_from_slice(&[0x40, 0x7c, 0x02, 0x5d]);
            assert_eq!(b.len() - start, RECORD_BYTES);
        }
        b
    }

    #[test]
    fn reads_centres_normals_and_radii() {
        let mut bytes = mkf(&[([1.0, -2.0, 300.5], [0.0, 0.6, -0.8], 3.05), ([-12.5, 40.25, 455.0], [0.6, 0.0, 0.8], 3.0)]);
        // The radius bytes as Revo writes them (0x40400003, just over 3.0) and junk padding.
        bytes[8 + 104 + 92..8 + 104 + 96].copy_from_slice(&[0x03, 0x00, 0x40, 0x40]);
        bytes[8 + 104 + 100..8 + 2 * 104].copy_from_slice(&[0xb7, 0x80, 0xfb, 0x7f]);
        let markers = parse_marker_framework(&bytes).unwrap();
        assert_eq!(markers.len(), 2);
        assert_eq!(markers[0].centre, [1.0, -2.0, 300.5]);
        assert_eq!(markers[0].normal, [0.0, 0.6, -0.8]);
        assert_eq!((markers[0].radius_mm, markers[0].id), (3.05, -1));
        let m = markers[1];
        assert_eq!((m.centre, m.normal), ([-12.5, 40.25, 455.0], [0.6, 0.0, 0.8]));
        assert!((m.radius_mm - 3.0).abs() < 1e-5, "{}", m.radius_mm);
        // An empty list is a valid file.
        assert!(parse_marker_framework(&mkf(&[])).unwrap().is_empty());
    }

    #[test]
    fn rejects_malformed_files_without_panicking() {
        let good = mkf(&[([1.0, 2.0, 3.0], [0.0, 0.0, 1.0], 3.0), ([4.0, 5.0, 6.0], [1.0, 0.0, 0.0], 3.0)]);
        assert_eq!(parse_marker_framework(&good).unwrap().len(), 2);
        // Shorter than the header, cut inside a record, one byte too many.
        for bytes in [&good[..5], &good[..good.len() - 1], &[good.as_slice(), &[0]].concat()[..]] {
            assert!(parse_marker_framework(bytes).is_err(), "{} bytes", bytes.len());
        }
        // A count that does not fit the length (also one that overflows on 32-bit sizes).
        for count in [3u32, 1, u32::MAX] {
            let mut b = good.clone();
            b[4..8].copy_from_slice(&count.to_le_bytes());
            assert!(parse_marker_framework(&b).is_err(), "count {count}");
        }
        // An unknown version.
        let mut b = good.clone();
        b[0..4].copy_from_slice(&4u32.to_le_bytes());
        assert!(matches!(parse_marker_framework(&b), Err(IoError::Unsupported(_))));
        // A normal that is not a unit vector, a centre that is not finite, a negative radius.
        for bad in [([1.0, 2.0, 3.0], [0.0, 0.0, 2.0], 3.0), ([f64::NAN, 2.0, 3.0], [0.0, 0.0, 1.0], 3.0), ([1.0, 2.0, 3.0], [0.0, 0.0, 1.0], -1.0)] {
            assert!(parse_marker_framework(&mkf(&[bad])).is_err(), "{bad:?}");
        }
        // Random bytes of every length up to a few records never panic.
        let mut state = 0x2545_f491_4f6c_dd1du64;
        for len in 0..(HEADER_BYTES + 3 * RECORD_BYTES) {
            let mut b: Vec<u8> = (0..len)
                .map(|_| {
                    state ^= state << 13;
                    state ^= state >> 7;
                    state ^= state << 17;
                    state as u8
                })
                .collect();
            if len >= 8 {
                b[0..4].copy_from_slice(&3u32.to_le_bytes());
                b[4..8].copy_from_slice(&(((len - 8) / RECORD_BYTES) as u32).to_le_bytes());
            }
            let _ = parse_marker_framework(&b);
        }
    }

    #[test]
    fn finds_the_file_beside_a_scan() {
        let dir = std::env::temp_dir().join(format!("scancraft-revo-mkf-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let scan = dir.join("raw_preview.ply");
        assert!(marker_framework_beside(&scan).is_none());
        std::fs::write(dir.join(MARKER_FRAMEWORK_FILE), mkf(&[([1.0, 2.0, 3.0], [0.0, 0.0, 1.0], 3.0)])).unwrap();
        let path = marker_framework_beside(&scan).expect("found");
        assert_eq!(read_marker_framework(&path).unwrap().len(), 1);
        std::fs::write(&path, b"junk").unwrap();
        let err = read_marker_framework(&path).unwrap_err().to_string();
        assert!(err.contains(MARKER_FRAMEWORK_FILE), "the error names the file: {err}");
        let _ = std::fs::remove_dir_all(&dir);
    }
}
