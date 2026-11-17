use crate::IoError;
use scancraft_core::{ColorRgba, Mesh, Point3D, Triangle};
use std::fs::File;
use std::io::{BufRead, BufReader, BufWriter, Write};
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

/// Load standard Wavefront OBJ file into a Mesh
pub fn load_obj<P: AsRef<Path>>(path: P) -> Result<Mesh, IoError> {
    let file = File::open(path)?;
    let reader = BufReader::new(file);

    let mut vertices = Vec::new();
    let mut normals = Vec::new();
    let mut triangles = Vec::new();

    for line in reader.lines() {
        let l = line?;
        let trimmed = l.trim();
        if trimmed.starts_with('#') || trimmed.is_empty() {
            continue;
        }

        let parts: Vec<&str> = trimmed.split_whitespace().collect();
        if parts.is_empty() {
            continue;
        }

        match parts[0] {
            "v" => {
                if parts.len() >= 4 {
                    let x: f32 = parts[1].parse().unwrap_or(0.0);
                    let y: f32 = parts[2].parse().unwrap_or(0.0);
                    let z: f32 = parts[3].parse().unwrap_or(0.0);

                    let (r, g, b) = if parts.len() >= 7 {
                        let cr: f32 = parts[4].parse().unwrap_or(0.8);
                        let cg: f32 = parts[5].parse().unwrap_or(0.8);
                        let cb: f32 = parts[6].parse().unwrap_or(0.8);
                        (
                            (cr * 255.0).clamp(0.0, 255.0) as u8,
                            (cg * 255.0).clamp(0.0, 255.0) as u8,
                            (cb * 255.0).clamp(0.0, 255.0) as u8,
                        )
                    } else {
                        (200, 205, 215)
                    };

                    vertices.push(Point3D {
                        position: [x, y, z],
                        normal: [0.0, 0.0, 1.0],
                        color: ColorRgba::new(r, g, b, 255),
                        confidence: 1.0,
                    });
                }
            }
            "vn" => {
                if parts.len() >= 4 {
                    let nx: f32 = parts[1].parse().unwrap_or(0.0);
                    let ny: f32 = parts[2].parse().unwrap_or(0.0);
                    let nz: f32 = parts[3].parse().unwrap_or(1.0);
                    normals.push([nx, ny, nz]);
                }
            }
            "f" => {
                // Parse face indices e.g. "v", "v/vt", "v//vn", "v/vt/vn"
                if parts.len() >= 4 {
                    let mut face_verts = Vec::new();
                    for &part in &parts[1..] {
                        let v_idx_str = part.split('/').next().unwrap_or("0");
                        if let Ok(v_idx) = v_idx_str.parse::<i32>() {
                            let idx = if v_idx > 0 {
                                (v_idx - 1) as u32
                            } else {
                                (vertices.len() as i32 + v_idx) as u32
                            };
                            face_verts.push(idx);
                        }
                    }

                    // Triangulate n-gon fan
                    for i in 1..face_verts.len().saturating_sub(1) {
                        triangles.push(Triangle::new(face_verts[0], face_verts[i], face_verts[i + 1]));
                    }
                }
            }
            _ => {}
        }
    }

    // Attach normals if available
    for (i, v) in vertices.iter_mut().enumerate() {
        if i < normals.len() {
            v.normal = normals[i];
        }
    }

    let mut mesh = Mesh::from_geometry(vertices, triangles);
    if normals.is_empty() {
        mesh.recompute_normals();
    }
    Ok(mesh)
}
