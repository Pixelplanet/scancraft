use crate::IoError;
use byteorder::{LittleEndian, WriteBytesExt};
use glam::Vec3;
use scancraft_core::Mesh;
use std::fs::File;
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
