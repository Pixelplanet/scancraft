use crate::import::{ImportedScan, NormalSource, ScanFormat};
use crate::normals::normalize_or_zero;
use crate::IoError;
use scancraft_core::Mesh;
use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::Path;

/// Export mesh to standard Wavefront OBJ file
pub fn save_mesh_obj<P: AsRef<Path>>(mesh: &Mesh, path: P) -> Result<(), IoError> {
    let file = File::create(path)?;
    let mut w = BufWriter::new(file);

    writeln!(w, "# ScanCraft Mesh Export (Wavefront OBJ)")?;
    writeln!(w, "# Vertices: {}", mesh.vertices.len())?;
    writeln!(w, "# Triangles: {}", mesh.triangles.len())?;

    // Vertices
    for v in &mesh.vertices {
        // Position + optional vertex color (RGB 0.0 - 1.0)
        let r = v.color.r as f32 / 255.0;
        let g = v.color.g as f32 / 255.0;
        let b = v.color.b as f32 / 255.0;
        writeln!(
            w,
            "v {:.6} {:.6} {:.6} {:.4} {:.4} {:.4}",
            v.position[0], v.position[1], v.position[2], r, g, b
        )?;
    }

    // Normals
    for v in &mesh.vertices {
        writeln!(
            w,
            "vn {:.6} {:.6} {:.6}",
            v.normal[0], v.normal[1], v.normal[2]
        )?;
    }

    // Faces (1-indexed in OBJ format)
    for t in &mesh.triangles {
        let i0 = t.indices[0] + 1;
        let i1 = t.indices[1] + 1;
        let i2 = t.indices[2] + 1;
        writeln!(w, "f {}//{} {}//{} {}//{}", i0, i0, i1, i1, i2, i2)?;
    }

    w.flush()?;
    Ok(())
}


/// Load a Wavefront OBJ file into a Mesh (vertices, colours, faces; normals from the file's
/// `vn` entries, else from the faces).
pub fn load_obj<P: AsRef<Path>>(path: P) -> Result<Mesh, IoError> {
    let bytes = std::fs::read(path)?;
    let scan = crate::import::finish(parse_obj(&bytes)?, &crate::ImportOptions::without_estimation());
    Ok(scan.to_mesh().unwrap_or_else(|| Mesh::from_geometry(scan.to_point_cloud().points, Vec::new())))
}

/// True when the head of the file has OBJ vertex lines and no binary bytes.
pub(crate) fn looks_like_obj(bytes: &[u8]) -> bool {
    let head = &bytes[..bytes.len().min(4096)];
    let Ok(text) = std::str::from_utf8(head).or_else(|e| std::str::from_utf8(&head[..e.valid_up_to()])) else {
        return false;
    };
    !text.contains('\0') && text.lines().any(|l| l.trim_start().starts_with("v "))
}

/// Parses an OBJ: `v x y z [r g b]`, `vn`, and `f` with any of the `v`, `v/vt`, `v//vn`,
/// `v/vt/vn` corner forms (negative indices count back). Polygons become triangle fans.
/// OBJ normals belong to face corners; a vertex gets the normalised sum of its corners' normals.
pub(crate) fn parse_obj(bytes: &[u8]) -> Result<ImportedScan, IoError> {
    let text = String::from_utf8_lossy(bytes);
    let mut scan = ImportedScan::empty(ScanFormat::Obj);
    let mut colors: Vec<[u8; 4]> = Vec::new();
    let mut any_color = false;
    let mut file_normals: Vec<[f32; 3]> = Vec::new();
    let mut corner_normals: Vec<(u32, u32)> = Vec::new(); // (vertex, normal)
    let mut faces: Vec<[u32; 3]> = Vec::new();
    let mut poly: Vec<f64> = Vec::new();

    for (n, line) in text.lines().enumerate() {
        let err = |what: &str| IoError::Parse(format!("OBJ line {}: {what}: '{}'", n + 1, line.trim()));
        let mut t = line.split_ascii_whitespace();
        match t.next() {
            Some("v") => {
                let nums = numbers(t).map_err(|_| err("bad number"))?;
                if nums.len() < 3 {
                    return Err(err("vertex needs x y z"));
                }
                scan.points.push([nums[0], nums[1], nums[2]]);
                let color = if nums.len() >= 6 {
                    any_color = true;
                    // Mostly 0..1, but some tools write 0..255.
                    let scale = if nums[3..6].iter().any(|&c| c > 1.0) { 1.0 } else { 255.0 };
                    let c = |v: f32| (v * scale).round().clamp(0.0, 255.0) as u8;
                    [c(nums[3]), c(nums[4]), c(nums[5]), 255]
                } else {
                    [200, 205, 215, 255]
                };
                colors.push(color);
            }
            Some("vn") => {
                let nums = numbers(t).map_err(|_| err("bad number"))?;
                if nums.len() < 3 {
                    return Err(err("normal needs x y z"));
                }
                file_normals.push([nums[0], nums[1], nums[2]]);
            }
            Some("f") => {
                poly.clear();
                for corner in t {
                    let mut parts = corner.split('/');
                    let v = resolve(parts.next(), scan.points.len()).ok_or_else(|| err("bad vertex index"))?;
                    let vn = match parts.nth(1) {
                        Some(s) if !s.is_empty() => Some(resolve(Some(s), file_normals.len()).ok_or_else(|| err("bad normal index"))?),
                        _ => None,
                    };
                    if let Some(vn) = vn {
                        corner_normals.push((v, vn));
                    }
                    poly.push(v as f64);
                }
                if poly.len() < 3 {
                    return Err(err("face needs three corners"));
                }
                crate::ply::triangulate_fan(&poly, &mut faces)?;
            }
            _ => {} // comments, vt, groups, materials, lines, points
        }
    }

    crate::ply::check_faces(&faces, scan.points.len())?;
    if let Some(&(_, vn)) = corner_normals.iter().find(|c| c.1 as usize >= file_normals.len()) {
        return Err(IoError::Parse(format!("OBJ: face refers to normal {}, but there are only {}", vn + 1, file_normals.len())));
    }
    if !corner_normals.is_empty() {
        let mut sum = vec![[0.0f32; 3]; scan.points.len()];
        for &(v, vn) in &corner_normals {
            let s = &mut sum[v as usize];
            let n = normalize_or_zero(file_normals[vn as usize]);
            for a in 0..3 {
                s[a] += n[a];
            }
        }
        scan.normals = Some(sum);
        scan.normal_source = NormalSource::File;
    }
    if any_color {
        scan.colors = Some(colors);
    }
    if !faces.is_empty() {
        scan.faces = Some(faces);
    }
    Ok(scan)
}

/// Resolves a 1-based (or negative, relative) OBJ index to 0-based. Validity against the
/// final count is checked later, since forward references are legal.
fn resolve(s: Option<&str>, count_so_far: usize) -> Option<u32> {
    let i: i64 = s?.parse().ok()?;
    let idx = match i {
        0 => return None,
        i if i > 0 => i - 1,
        i => count_so_far as i64 + i,
    };
    u32::try_from(idx).ok()
}

fn numbers<'a>(t: impl Iterator<Item = &'a str>) -> Result<Vec<f32>, std::num::ParseFloatError> {
    t.map(str::parse).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn corner_forms_negative_indices_and_quads() {
        let text = b"# test\nv 0 0 0 1 0 0\nv 1 0 0 0 1 0\nv 1 1 0 0 0 1\nv 0 1 0 1 1 1\n\
vt 0 0\nvn 0 0 1\nvn 0 0 -1\nf 1/1/1 2/1/1 3//1 4//2\nf -4//2 -2 -1\n";
        let scan = parse_obj(text).unwrap();
        assert_eq!(scan.points.len(), 4);
        assert_eq!(scan.faces.as_ref().unwrap(), &vec![[0, 1, 2], [0, 2, 3], [0, 2, 3]]);
        assert_eq!(scan.colors.as_ref().unwrap()[0], [255, 0, 0, 255]);
        let n = scan.normals.as_ref().unwrap();
        assert_eq!(n[1], [0.0, 0.0, 1.0]);
        assert_eq!(n[3], [0.0, 0.0, -1.0]);
        // Vertex 1 has one +z and one -z corner: unknown (zero), later filled from the faces.
        assert_eq!(n[0], [0.0, 0.0, 0.0]);
        assert_eq!(scan.normal_source, NormalSource::File);
    }

    #[test]
    fn points_only_obj_has_no_faces() {
        let scan = parse_obj(b"v 1 2 3\nv 4 5 6\n").unwrap();
        assert_eq!(scan.points.len(), 2);
        assert!(scan.faces.is_none() && scan.colors.is_none() && scan.normals.is_none());
    }

    #[test]
    fn bad_and_truncated_obj_are_errors() {
        assert!(parse_obj(b"v 1 2\n").is_err());
        assert!(parse_obj(b"v 1 2 x\n").is_err());
        assert!(parse_obj(b"v 0 0 0\nv 1 0 0\nf 1 2 3\n").is_err());
        assert!(parse_obj(b"v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2\n").is_err());
        assert!(parse_obj(b"v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1//4 2//1 3//1\nvn 0 0 1\n").is_err());
        assert!(parse_obj(b"v 0 0 0\nv 1 0 0\nv 0 1 0\nf 0 1 2\n").is_err());
    }
}
