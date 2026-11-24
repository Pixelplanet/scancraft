use crate::import::{ImportedScan, ScanFormat};
use crate::normals::FxHasher;
use crate::IoError;
use byteorder::{ByteOrder, LittleEndian, WriteBytesExt};
use glam::Vec3;
use scancraft_core::Mesh;
use std::collections::HashMap;
use std::fs::File;
use std::hash::BuildHasherDefault;
use std::io::{BufWriter, Write};
use std::path::Path;

/// Export mesh to standard binary STL file (compatible with Bambu Studio, Cura, PrusaSlicer)
pub fn save_mesh_stl_binary<P: AsRef<Path>>(mesh: &Mesh, path: P) -> Result<(), IoError> {
    let file = File::create(path)?;
    let mut w = BufWriter::new(file);

    // 1. 80-byte header
    let header_text = "ScanCraft 3D Scanner Suite - Binary STL Export";
    let mut header = [0u8; 80];
    let bytes = header_text.as_bytes();
    let copy_len = bytes.len().min(80);
    header[..copy_len].copy_from_slice(&bytes[..copy_len]);
    w.write_all(&header)?;

    // 2. Number of triangles (uint32)
    w.write_u32::<LittleEndian>(mesh.triangles.len() as u32)?;

    // 3. Triangles (50 bytes each)
    for tri in &mesh.triangles {
        let i0 = tri.indices[0] as usize;
        let i1 = tri.indices[1] as usize;
        let i2 = tri.indices[2] as usize;

        if i0 < mesh.vertices.len() && i1 < mesh.vertices.len() && i2 < mesh.vertices.len() {
            let v0 = Vec3::from_slice(&mesh.vertices[i0].position);
            let v1 = Vec3::from_slice(&mesh.vertices[i1].position);
            let v2 = Vec3::from_slice(&mesh.vertices[i2].position);

            // Compute face normal
            let face_normal = (v1 - v0).cross(v2 - v0);
            let n = if face_normal.length() > 1e-6 {
                face_normal.normalize()
            } else {
                Vec3::Z
            };

            // Normal vector
            w.write_f32::<LittleEndian>(n.x)?;
            w.write_f32::<LittleEndian>(n.y)?;
            w.write_f32::<LittleEndian>(n.z)?;

            // Vertex 1
            w.write_f32::<LittleEndian>(v0.x)?;
            w.write_f32::<LittleEndian>(v0.y)?;
            w.write_f32::<LittleEndian>(v0.z)?;

            // Vertex 2
            w.write_f32::<LittleEndian>(v1.x)?;
            w.write_f32::<LittleEndian>(v1.y)?;
            w.write_f32::<LittleEndian>(v1.z)?;

            // Vertex 3
            w.write_f32::<LittleEndian>(v2.x)?;
            w.write_f32::<LittleEndian>(v2.y)?;
            w.write_f32::<LittleEndian>(v2.z)?;

            // 2-byte attribute byte count
            w.write_u16::<LittleEndian>(0)?;
        }
    }

    w.flush()?;
    Ok(())
}

/// Export mesh to ASCII STL (larger than binary; for tools that only read text).
pub fn save_mesh_stl_ascii<P: AsRef<Path>>(mesh: &Mesh, path: P) -> Result<(), IoError> {
    let mut w = BufWriter::new(File::create(path)?);
    writeln!(w, "solid scancraft")?;
    let n = mesh.vertices.len();
    for tri in &mesh.triangles {
        if tri.indices.iter().any(|&i| i as usize >= n) {
            continue;
        }
        let [v0, v1, v2] = tri.indices.map(|i| Vec3::from_slice(&mesh.vertices[i as usize].position));
        let normal = (v1 - v0).cross(v2 - v0).normalize_or_zero();
        writeln!(w, "  facet normal {:e} {:e} {:e}", normal.x, normal.y, normal.z)?;
        writeln!(w, "    outer loop")?;
        for v in [v0, v1, v2] {
            // `{:?}` prints the shortest text that reads back to the same f32.
            writeln!(w, "      vertex {:?} {:?} {:?}", v.x, v.y, v.z)?;
        }
        writeln!(w, "    endloop")?;
        writeln!(w, "  endfacet")?;
    }
    writeln!(w, "endsolid scancraft")?;
    w.flush()?;
    Ok(())
}

/// True when the bytes are a binary STL (size matches the triangle count) or an ASCII STL.
pub(crate) fn looks_like_stl(bytes: &[u8]) -> bool {
    binary_size_matches(bytes) || is_ascii_stl(bytes)
}

fn binary_size_matches(bytes: &[u8]) -> bool {
    bytes.len() >= 84 && {
        let n = LittleEndian::read_u32(&bytes[80..84]) as u64;
        bytes.len() as u64 == 84 + 50 * n
    }
}

/// `solid` at the start, a text-only head and a `facet` or `endsolid` keyword. Binary files
/// whose header happens to start with `solid` fail the text check or the size check.
fn is_ascii_stl(bytes: &[u8]) -> bool {
    let head = &bytes[..bytes.len().min(1024)];
    let text = head.iter().all(|&b| b == b'\t' || b == b'\n' || b == b'\r' || (32..127).contains(&b));
    head.trim_ascii_start().starts_with(b"solid")
        && text
        && !binary_size_matches(bytes)
        && (contains(head, b"facet") || contains(head, b"endsolid"))
}

fn contains(hay: &[u8], needle: &[u8]) -> bool {
    hay.windows(needle.len()).any(|w| w == needle)
}

/// Parses a binary or ASCII STL. Shared corners are welded (identical coordinates become one
/// vertex) so that the result is an indexed mesh with smooth vertex normals. Facet normals in
/// the file are ignored: the vertex order defines the outside, as slicers assume.
pub(crate) fn parse_stl(bytes: &[u8]) -> Result<ImportedScan, IoError> {
    let corners = if is_ascii_stl(bytes) { ascii_corners(bytes)? } else { binary_corners(bytes)? };
    let mut scan = ImportedScan::empty(ScanFormat::Stl);
    let mut index: HashMap<[u32; 3], u32, BuildHasherDefault<FxHasher>> = HashMap::default();
    index.reserve(corners.len() / 5);
    scan.points.reserve(corners.len() / 5);
    let mut faces = Vec::with_capacity(corners.len() / 3);
    for tri in corners.chunks_exact(3) {
        let mut f = [0u32; 3];
        for (slot, p) in f.iter_mut().zip(tri) {
            // Adding +0.0 turns -0.0 into 0.0, so both weld together.
            let p = p.map(|v| v + 0.0);
            let next = scan.points.len() as u32;
            *slot = *index.entry(p.map(f32::to_bits)).or_insert_with(|| {
                scan.points.push(p);
                next
            });
        }
        faces.push(f);
    }
    scan.faces = Some(faces);
    Ok(scan)
}

fn binary_corners(bytes: &[u8]) -> Result<Vec<[f32; 3]>, IoError> {
    if bytes.len() < 84 {
        return Err(IoError::Truncated("STL: file shorter than the 84-byte header".into()));
    }
    let n = LittleEndian::read_u32(&bytes[80..84]) as usize;
    let need = 84 + 50 * n;
    if bytes.len() < need {
        return Err(IoError::Truncated(format!(
            "STL: header promises {n} triangles ({need} bytes), file has {} bytes",
            bytes.len()
        )));
    }
    if bytes.len() > need {
        log::warn!("STL: {} bytes after the last triangle ignored", bytes.len() - need);
    }
    let mut corners = Vec::with_capacity(3 * n);
    for rec in bytes[84..need].chunks_exact(50) {
        for c in 0..3 {
            let o = 12 + 12 * c;
            corners.push([0, 4, 8].map(|k| LittleEndian::read_f32(&rec[o + k..o + k + 4])));
        }
    }
    Ok(corners)
}

fn ascii_corners(bytes: &[u8]) -> Result<Vec<[f32; 3]>, IoError> {
    let text = std::str::from_utf8(bytes).map_err(|_| IoError::Parse("STL: ASCII file is not text".into()))?;
    let mut corners = Vec::new();
    let mut in_facet = 0usize; // corners seen in the current facet
    let mut facets = 0usize;
    let mut ended = false;
    let mut tokens = text.split_ascii_whitespace();
    while let Some(t) = tokens.next() {
        match t {
            "facet" => in_facet = 0,
            "vertex" => {
                let mut v = [0.0f32; 3];
                for c in &mut v {
                    let s = tokens.next().ok_or_else(|| IoError::Truncated("STL: file ends inside a vertex".into()))?;
                    *c = s.parse().map_err(|_| IoError::Parse(format!("STL: bad number '{s}'")))?;
                }
                corners.push(v);
                in_facet += 1;
            }
            "endfacet" => {
                if in_facet != 3 {
                    return Err(IoError::Parse(format!("STL: facet {facets} has {in_facet} vertices, expected 3")));
                }
                facets += 1;
            }
            "endsolid" => ended = true,
            _ => {}
        }
    }
    if corners.len() != facets * 3 {
        return Err(IoError::Truncated(format!("STL: file ends inside facet {facets}")));
    }
    if !ended {
        log::warn!("STL: no endsolid; the file may be cut short");
    }
    Ok(corners)
}

#[cfg(test)]
mod tests {
    use super::*;
    use scancraft_core::Point3D;

    fn quad() -> Mesh {
        let mut m = Mesh::new();
        for p in [[0.0, 0.0, -0.0], [1.5, 0.0, 0.0], [1.5, 2.25, 0.0], [0.0, 2.25, 0.0]] {
            m.add_vertex(Point3D::new(p));
        }
        m.add_triangle(0, 1, 2);
        m.add_triangle(0, 2, 3);
        m
    }

    fn written(save: fn(&Mesh, &std::path::Path) -> Result<(), IoError>, name: &str) -> Vec<u8> {
        let path = std::env::temp_dir().join(format!("scancraft_stl_{}_{name}", std::process::id()));
        save(&quad(), &path).unwrap();
        let bytes = std::fs::read(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        bytes
    }

    #[test]
    fn binary_and_ascii_weld_to_the_same_mesh() {
        let bin = parse_stl(&written(|m, p| save_mesh_stl_binary(m, p), "b.stl")).unwrap();
        let txt = parse_stl(&written(|m, p| save_mesh_stl_ascii(m, p), "a.stl")).unwrap();
        assert_eq!(bin.points.len(), 4);
        assert_eq!(bin.faces.as_ref().unwrap().len(), 2);
        assert_eq!(bin.points, txt.points);
        assert_eq!(bin.faces, txt.faces);
    }

    #[test]
    fn binary_header_starting_with_solid_is_still_binary() {
        let mut bytes = written(|m, p| save_mesh_stl_binary(m, p), "s.stl");
        bytes[..5].copy_from_slice(b"solid");
        assert!(!is_ascii_stl(&bytes));
        assert_eq!(parse_stl(&bytes).unwrap().points.len(), 4);
    }

    #[test]
    fn truncated_stl_is_an_error() {
        let bin = written(|m, p| save_mesh_stl_binary(m, p), "t.stl");
        assert!(matches!(parse_stl(&bin[..bin.len() - 10]), Err(IoError::Truncated(_))));
        assert!(matches!(parse_stl(&bin[..50]), Err(IoError::Truncated(_))));
        let txt = written(|m, p| save_mesh_stl_ascii(m, p), "u.stl");
        assert!(parse_stl(&txt[..txt.len() * 2 / 3]).is_err());
        let bad = b"solid x\nfacet normal 0 0 1\nouter loop\nvertex 0 0 0\nvertex 1 0 0\nendloop\nendfacet\nendsolid\n";
        assert!(matches!(parse_stl(bad), Err(IoError::Parse(_))));
    }
}
