pub mod bundle;
pub mod capture;
pub mod fsutil;
pub mod import;
pub mod lock;
pub mod normals;
pub mod obj;
pub mod ply;
pub mod project;
pub mod revo;
pub mod stl;
pub mod workdir;

pub use import::{
    detect_format, import_bytes, import_scan, import_scan_with, ImportOptions, ImportedScan, NormalSource,
    ScanFormat, UnitsHint,
};
pub use normals::NormalEstimation;
pub use obj::{load_obj, save_mesh_obj};
pub use ply::{load_ply, save_mesh_ply_binary, save_ply_binary};
pub use revo::{marker_framework_beside, read_marker_framework, RevoMarker};
pub use stl::{save_mesh_stl_ascii, save_mesh_stl_binary};

use thiserror::Error;

#[derive(Error, Debug)]
pub enum IoError {
    #[error("File IO error: {0}")]
    Io(#[from] std::io::Error),
    #[error("Parse error: {0}")]
    Parse(String),
    /// The file ends before the data its header or structure promises.
    #[error("Truncated file: {0}")]
    Truncated(String),
    #[error("Unsupported file: {0}")]
    Unsupported(String),
    /// The path holds no ScanCraft project (see [`project::locate`]).
    #[error("{0}")]
    NotAProject(String),
    /// Another ScanCraft (or this one) has the project open: see [`lock`].
    #[error("{}", locked_message(path, owner.as_deref()))]
    Locked { path: std::path::PathBuf, owner: Option<Box<lock::Owner>> },
    #[error("Not enough free space on the drive of {}: {} needed, {} free", path.display(), fsutil::format_bytes(*needed), fsutil::format_bytes(*free))]
    NoSpace { path: std::path::PathBuf, needed: u64, free: u64 },
    /// The file was changed by someone else since it was opened (a bundle replaced behind the
    /// app's back); saving would lose that change.
    #[error("Conflict: {0}")]
    Conflict(String),
}

fn locked_message(path: &std::path::Path, owner: Option<&lock::Owner>) -> String {
    let what = lock::locked_target(path);
    let what = what.display();
    match owner {
        Some(o) if o.is_this_process() => format!("{what} is already open in this window"),
        Some(o) => format!("{what} is in use by {}", o.describe()),
        None => format!("{what} is being opened by another ScanCraft right now"),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use scancraft_core::{ColorRgba, Mesh, Point3D, PointCloud};
    use std::fs;

    #[test]
    fn test_ply_roundtrip() {
        let temp_dir = std::env::temp_dir();
        let ply_path = temp_dir.join("test_cloud.ply");

        let mut cloud = PointCloud::new();
        cloud.push(Point3D {
            position: [1.0, 2.0, 3.0],
            normal: [0.0, 0.0, 1.0],
            color: ColorRgba::new(255, 0, 0, 255),
            confidence: 1.0,
        });
        cloud.push(Point3D {
            position: [4.0, 5.0, 6.0],
            normal: [0.0, 1.0, 0.0],
            color: ColorRgba::new(0, 255, 0, 255),
            confidence: 7.0,
        });

        save_ply_binary(&cloud, &ply_path).expect("Failed to save PLY");
        let (loaded, _) = load_ply(&ply_path).expect("Failed to load PLY");

        assert_eq!(loaded.len(), 2);
        assert!((loaded.points[0].position[0] - 1.0).abs() < 1e-5);
        assert!((loaded.points[1].position[2] - 6.0).abs() < 1e-5);
        // Normals, colours and confidence (observation count) come back as written.
        assert_eq!(loaded.points[1].normal, [0.0, 1.0, 0.0]);
        assert_eq!(loaded.points[0].color, ColorRgba::new(255, 0, 0, 255));
        assert_eq!((loaded.points[0].confidence, loaded.points[1].confidence), (1.0, 7.0));

        let _ = fs::remove_file(ply_path);
    }

    #[test]
    fn test_mesh_obj_and_stl_export() {
        let temp_dir = std::env::temp_dir();
        let obj_path = temp_dir.join("test_mesh.obj");
        let stl_path = temp_dir.join("test_mesh.stl");

        let mut mesh = Mesh::new();
        mesh.add_vertex(Point3D::new([0.0, 0.0, 0.0]));
        mesh.add_vertex(Point3D::new([10.0, 0.0, 0.0]));
        mesh.add_vertex(Point3D::new([0.0, 10.0, 0.0]));
        mesh.add_triangle(0, 1, 2);

        // OBJ export & import
        save_mesh_obj(&mesh, &obj_path).expect("Failed to save OBJ");
        let loaded_obj = load_obj(&obj_path).expect("Failed to load OBJ");
        assert_eq!(loaded_obj.vertices.len(), 3);
        assert_eq!(loaded_obj.triangles.len(), 1);

        // STL binary export
        save_mesh_stl_binary(&mesh, &stl_path).expect("Failed to save STL");
        let metadata = fs::metadata(&stl_path).expect("Failed to get STL metadata");
        // 80 header + 4 count + 50 * 1 triangle = 134 bytes
        assert_eq!(metadata.len(), 134);

        let _ = fs::remove_file(obj_path);
        let _ = fs::remove_file(stl_path);
    }
}
