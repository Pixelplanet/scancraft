use crate::IoError;
use byteorder::{LittleEndian, ReadBytesExt, WriteBytesExt};
use scancraft_core::{ColorRgba, Mesh, Point3D, PointCloud, Triangle};
use std::fs::File;
use std::io::{BufRead, BufReader, BufWriter, Write};
use std::path::Path;

/// Export point cloud to binary little-endian PLY file
pub fn save_ply_binary<P: AsRef<Path>>(cloud: &PointCloud, path: P) -> Result<(), IoError> {
    let file = File::create(path)?;
    let mut w = BufWriter::new(file);

    writeln!(w, "ply")?;
    writeln!(w, "format binary_little_endian 1.0")?;
    writeln!(w, "comment ScanCraft Point Cloud Export")?;
    writeln!(w, "element vertex {}", cloud.points.len())?;
    writeln!(w, "property float x")?;
    writeln!(w, "property float y")?;
    writeln!(w, "property float z")?;
    writeln!(w, "property uchar red")?;
    writeln!(w, "property uchar green")?;
    writeln!(w, "property uchar blue")?;
    writeln!(w, "property uchar alpha")?;
    writeln!(w, "end_header")?;

    for p in &cloud.points {
        w.write_f32::<LittleEndian>(p.position[0])?;
        w.write_f32::<LittleEndian>(p.position[1])?;
        w.write_f32::<LittleEndian>(p.position[2])?;
        w.write_u8(p.color.r)?;
        w.write_u8(p.color.g)?;
        w.write_u8(p.color.b)?;
        w.write_u8(p.color.a)?;
    }

    w.flush()?;
    Ok(())
}

/// Export triangle mesh to binary little-endian PLY file
pub fn save_mesh_ply_binary<P: AsRef<Path>>(mesh: &Mesh, path: P) -> Result<(), IoError> {
    let file = File::create(path)?;
    let mut w = BufWriter::new(file);

    writeln!(w, "ply")?;
    writeln!(w, "format binary_little_endian 1.0")?;
    writeln!(w, "comment ScanCraft Mesh Export")?;
    writeln!(w, "element vertex {}", mesh.vertices.len())?;
    writeln!(w, "property float x")?;
    writeln!(w, "property float y")?;
    writeln!(w, "property float z")?;
    writeln!(w, "property float nx")?;
    writeln!(w, "property float ny")?;
    writeln!(w, "property float nz")?;
    writeln!(w, "property uchar red")?;
    writeln!(w, "property uchar green")?;
    writeln!(w, "property uchar blue")?;
    writeln!(w, "property uchar alpha")?;
    writeln!(w, "element face {}", mesh.triangles.len())?;
    writeln!(w, "property list uchar int vertex_indices")?;
    writeln!(w, "end_header")?;

    for v in &mesh.vertices {
        w.write_f32::<LittleEndian>(v.position[0])?;
        w.write_f32::<LittleEndian>(v.position[1])?;
        w.write_f32::<LittleEndian>(v.position[2])?;
        w.write_f32::<LittleEndian>(v.normal[0])?;
        w.write_f32::<LittleEndian>(v.normal[1])?;
        w.write_f32::<LittleEndian>(v.normal[2])?;
        w.write_u8(v.color.r)?;
        w.write_u8(v.color.g)?;
        w.write_u8(v.color.b)?;
        w.write_u8(v.color.a)?;
    }

    for t in &mesh.triangles {
        w.write_u8(3)?; // 3 vertices per face
        w.write_i32::<LittleEndian>(t.indices[0] as i32)?;
        w.write_i32::<LittleEndian>(t.indices[1] as i32)?;
        w.write_i32::<LittleEndian>(t.indices[2] as i32)?;
    }

    w.flush()?;
    Ok(())
}

/// Load a PLY file (supports binary little-endian and ascii PLYs)
pub fn load_ply<P: AsRef<Path>>(path: P) -> Result<(PointCloud, Option<Mesh>), IoError> {
    let file = File::open(path)?;
    let mut reader = BufReader::new(file);

    let mut is_binary = false;
    let mut vertex_count = 0usize;
    let mut face_count = 0usize;
    let mut has_normals = false;
    let mut has_colors = false;

    // Parse header
    let mut line = String::new();
    loop {
        line.clear();
        let bytes_read = reader.read_line(&mut line)?;
        if bytes_read == 0 {
            break;
        }
        let trimmed = line.trim();
        if trimmed == "end_header" {
            break;
        } else if trimmed.starts_with("format binary_little_endian") {
            is_binary = true;
        } else if trimmed.starts_with("element vertex ") {
            let parts: Vec<&str> = trimmed.split_whitespace().collect();
            if parts.len() >= 3 {
                vertex_count = parts[2].parse().unwrap_or(0);
            }
        } else if trimmed.starts_with("element face ") {
            let parts: Vec<&str> = trimmed.split_whitespace().collect();
            if parts.len() >= 3 {
                face_count = parts[2].parse().unwrap_or(0);
            }
        } else if trimmed == "property float nx" {
            has_normals = true;
        } else if trimmed == "property uchar red" {
            has_colors = true;
        }
    }

    let mut cloud = PointCloud::new();
    let mut mesh_triangles = Vec::new();

    if is_binary {
        for _ in 0..vertex_count {
            let x = reader.read_f32::<LittleEndian>()?;
            let y = reader.read_f32::<LittleEndian>()?;
            let z = reader.read_f32::<LittleEndian>()?;

            let normal = if has_normals {
                let nx = reader.read_f32::<LittleEndian>()?;
                let ny = reader.read_f32::<LittleEndian>()?;
                let nz = reader.read_f32::<LittleEndian>()?;
                [nx, ny, nz]
            } else {
                [0.0, 0.0, 1.0]
            };

            let color = if has_colors {
                let r = reader.read_u8()?;
                let g = reader.read_u8()?;
                let b = reader.read_u8()?;
                let a = reader.read_u8()?;
                ColorRgba::new(r, g, b, a)
            } else {
                ColorRgba::new(200, 205, 215, 255)
            };

            cloud.push(Point3D {
                position: [x, y, z],
                normal,
                color,
                confidence: 1.0,
            });
        }

        if face_count > 0 {
            for _ in 0..face_count {
                let n_verts = reader.read_u8()?;
                let mut idxs = Vec::with_capacity(n_verts as usize);
                for _ in 0..n_verts {
                    idxs.push(reader.read_i32::<LittleEndian>()? as u32);
                }
                if idxs.len() >= 3 {
                    mesh_triangles.push(Triangle::new(idxs[0], idxs[1], idxs[2]));
                }
            }
        }
    } else {
        // ASCII PLY
        for _ in 0..vertex_count {
            line.clear();
            if reader.read_line(&mut line)? == 0 {
                break;
            }
            let parts: Vec<&str> = line.split_whitespace().collect();
            if parts.len() >= 3 {
                let x: f32 = parts[0].parse().unwrap_or(0.0);
                let y: f32 = parts[1].parse().unwrap_or(0.0);
                let z: f32 = parts[2].parse().unwrap_or(0.0);

                let r = parts.get(3).and_then(|s| s.parse().ok()).unwrap_or(200);
                let g = parts.get(4).and_then(|s| s.parse().ok()).unwrap_or(205);
                let b = parts.get(5).and_then(|s| s.parse().ok()).unwrap_or(215);

                cloud.push(Point3D {
                    position: [x, y, z],
                    normal: [0.0, 0.0, 1.0],
                    color: ColorRgba::new(r, g, b, 255),
                    confidence: 1.0,
                });
            }
        }

        for _ in 0..face_count {
            line.clear();
            if reader.read_line(&mut line)? == 0 {
                break;
            }
            let parts: Vec<&str> = line.split_whitespace().collect();
            if parts.len() >= 4 {
                let i0: u32 = parts[1].parse().unwrap_or(0);
                let i1: u32 = parts[2].parse().unwrap_or(0);
                let i2: u32 = parts[3].parse().unwrap_or(0);
                mesh_triangles.push(Triangle::new(i0, i1, i2));
            }
        }
    }

    let mesh = if !mesh_triangles.is_empty() {
        Some(Mesh::from_geometry(cloud.points.clone(), mesh_triangles))
    } else {
        None
    };

    Ok((cloud, mesh))
}
