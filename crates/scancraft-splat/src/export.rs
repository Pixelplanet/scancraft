//! Export the splat inputs as a standard dataset, so users can train in external tools
//! (nerfstudio / gsplat, the INRIA 3DGS code, Postshot, Brush's own app, ...).
//!
//! Both layouts are written in **metres** with ScanCraft's Z-up world axes:
//!
//! ```text
//! <dir>/images/<name>.png          RGB keyframes
//! <dir>/masks/<name>.png           foreground masks (only if the images have masks)
//! <dir>/transforms.json            nerfstudio: OpenGL camera-to-world (+X right, +Y up, +Z back)
//! <dir>/points3D.ply               initial cloud (covered points), referenced by transforms.json
//! <dir>/sparse/0/cameras.txt       COLMAP text model: PINHOLE / OPENCV / FULL_OPENCV
//! <dir>/sparse/0/images.txt        COLMAP: world-to-camera (OpenCV axes), qw qx qy qz tx ty tz
//! <dir>/sparse/0/points3D.txt      COLMAP: initial cloud, empty tracks
//! ```
//!
//! Brush loads either layout; a tool that wants both picks the one it understands.

use crate::convert::MM_TO_M;
use crate::coverage::{self, CoverageConfig};
use crate::types::{PosedImage, SplatDataset};
use crate::SplatError;
use glam::{Mat4, Vec3};
use serde_json::json;
use std::fmt::Write as _;
use std::io::Write as _;
use std::path::Path;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ExportOptions {
    pub nerfstudio: bool,
    pub colmap: bool,
    /// Drop points of the initial cloud that no image sees (as training does).
    pub filter_coverage: bool,
}

impl Default for ExportOptions {
    fn default() -> Self {
        Self { nerfstudio: true, colmap: true, filter_coverage: true }
    }
}

fn image_file(img: &PosedImage) -> String {
    format!("{}.png", img.name)
}

/// Camera-to-world in metres with OpenCV camera axes.
fn c2w_opencv_m(img: &PosedImage) -> Mat4 {
    let mut m = Mat4::from(img.camera_to_world_opencv());
    m.w_axis = (m.w_axis.truncate() * MM_TO_M).extend(1.0);
    m
}

/// Write the dataset to `dir` (created if needed).
pub fn export_dataset(dataset: &SplatDataset, dir: &Path, opts: &ExportOptions) -> Result<(), SplatError> {
    dataset.validate()?;
    std::fs::create_dir_all(dir.join("images"))?;
    let has_masks = dataset.images.iter().any(|i| i.mask.is_some());
    if has_masks {
        std::fs::create_dir_all(dir.join("masks"))?;
    }
    for img in &dataset.images {
        image::RgbImage::from_raw(img.width, img.height, img.rgb.clone())
            .expect("validated")
            .save(dir.join("images").join(image_file(img)))?;
        if has_masks {
            let mask = img.mask.clone().unwrap_or_else(|| vec![255; (img.width * img.height) as usize]);
            image::GrayImage::from_raw(img.width, img.height, mask).expect("validated").save(dir.join("masks").join(image_file(img)))?;
        }
    }

    let cloud = if opts.filter_coverage {
        coverage::filter_covered(&dataset.init_cloud, &dataset.images, &CoverageConfig::default())
    } else {
        dataset.init_cloud.clone()
    };
    let points: Vec<([f32; 3], [u8; 3])> = cloud
        .points
        .iter()
        .map(|p| ((Vec3::from_array(p.position) * MM_TO_M).to_array(), [p.color.r, p.color.g, p.color.b]))
        .collect();

    if opts.nerfstudio {
        write_points_ply(&dir.join("points3D.ply"), &points)?;
        std::fs::write(dir.join("transforms.json"), nerfstudio_json(dataset, has_masks, !points.is_empty()))?;
    }
    if opts.colmap {
        let sparse = dir.join("sparse").join("0");
        std::fs::create_dir_all(&sparse)?;
        let (cameras, images) = colmap_text(dataset);
        std::fs::write(sparse.join("cameras.txt"), cameras)?;
        std::fs::write(sparse.join("images.txt"), images)?;
        let mut pts = String::from("# 3D point list with one line of data per point:\n#   POINT3D_ID, X, Y, Z, R, G, B, ERROR, TRACK[] as (IMAGE_ID, POINT2D_IDX)\n");
        for (i, (p, c)) in points.iter().enumerate() {
            writeln!(pts, "{} {} {} {} {} {} {} 0", i + 1, p[0], p[1], p[2], c[0], c[1], c[2]).unwrap();
        }
        std::fs::write(sparse.join("points3D.txt"), pts)?;
    }
    Ok(())
}

/// nerfstudio `transforms.json`. Per-frame intrinsics (keyframes may come from different
/// calibrations); distortion as `OPENCV` (k1 k2 p1 p2 — nerfstudio's `OPENCV` has no k3, so a
/// non-zero k3 is only preserved in the COLMAP `FULL_OPENCV` model).
pub fn nerfstudio_json(dataset: &SplatDataset, has_masks: bool, with_ply: bool) -> String {
    // OpenCV camera axes -> OpenGL camera axes: flip y and z.
    let gl = Mat4::from_diagonal(glam::Vec4::new(1.0, -1.0, -1.0, 1.0));
    let frames: Vec<_> = dataset
        .images
        .iter()
        .map(|img| {
            let m = c2w_opencv_m(img) * gl;
            let rows: Vec<Vec<f32>> = (0..4).map(|r| m.row(r).to_array().to_vec()).collect();
            let k = &img.intrinsics;
            let d = k.distortion.unwrap_or_default();
            let mut f = json!({
                "file_path": format!("images/{}", image_file(img)),
                "transform_matrix": rows,
                "fl_x": k.fx, "fl_y": k.fy, "cx": k.cx, "cy": k.cy,
                "w": img.width, "h": img.height,
            });
            if k.distortion.is_some_and(|d| !d.is_zero()) {
                f["camera_model"] = json!("OPENCV");
                f["k1"] = json!(d.k1);
                f["k2"] = json!(d.k2);
                f["p1"] = json!(d.p1);
                f["p2"] = json!(d.p2);
            }
            if has_masks {
                f["mask_path"] = json!(format!("masks/{}", image_file(img)));
            }
            f
        })
        .collect();
    let mut root = json!({
        "camera_model": "OPENCV",
        "frames": frames,
        "scancraft": { "units": "metres", "world_up": [0.0, 0.0, 1.0], "source": "ScanCraft scancraft-splat export" },
    });
    if with_ply {
        root["ply_file_path"] = json!("points3D.ply");
    }
    serde_json::to_string_pretty(&root).expect("json")
}

/// COLMAP `cameras.txt` and `images.txt` contents.
pub fn colmap_text(dataset: &SplatDataset) -> (String, String) {
    let mut cameras = String::from("# Camera list with one line of data per camera:\n#   CAMERA_ID, MODEL, WIDTH, HEIGHT, PARAMS[]\n");
    let mut images = String::from("# Image list with two lines of data per image:\n#   IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, NAME\n#   POINTS2D[] as (X, Y, POINT3D_ID)\n");
    let mut seen: Vec<(String, u32)> = Vec::new();
    for (i, img) in dataset.images.iter().enumerate() {
        let k = &img.intrinsics;
        let line = match k.distortion.filter(|d| !d.is_zero()) {
            None => format!("PINHOLE {} {} {} {} {} {}", img.width, img.height, k.fx, k.fy, k.cx, k.cy),
            Some(d) if d.k3 == 0.0 => {
                format!("OPENCV {} {} {} {} {} {} {} {} {} {}", img.width, img.height, k.fx, k.fy, k.cx, k.cy, d.k1, d.k2, d.p1, d.p2)
            }
            Some(d) => format!(
                "FULL_OPENCV {} {} {} {} {} {} {} {} {} {} {} 0 0 0",
                img.width, img.height, k.fx, k.fy, k.cx, k.cy, d.k1, d.k2, d.p1, d.p2, d.k3
            ),
        };
        let cam_id = match seen.iter().find(|(l, _)| *l == line) {
            Some((_, id)) => *id,
            None => {
                let id = seen.len() as u32 + 1;
                writeln!(cameras, "{id} {line}").unwrap();
                seen.push((line, id));
                id
            }
        };
        let w2c = c2w_opencv_m(img).inverse();
        let (_, q, t) = w2c.to_scale_rotation_translation();
        let q = q.normalize();
        writeln!(images, "{} {} {} {} {} {} {} {} {} {}", i + 1, q.w, q.x, q.y, q.z, t.x, t.y, t.z, cam_id, image_file(img)).unwrap();
        images.push('\n'); // empty POINTS2D line
    }
    (cameras, images)
}

fn write_points_ply(path: &Path, points: &[([f32; 3], [u8; 3])]) -> Result<(), SplatError> {
    let mut f = std::io::BufWriter::new(std::fs::File::create(path)?);
    write!(
        f,
        "ply\nformat binary_little_endian 1.0\ncomment ScanCraft splat init cloud, metres, Z up\nelement vertex {}\nproperty float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n",
        points.len()
    )?;
    for (p, c) in points {
        for v in p {
            f.write_all(&v.to_le_bytes())?;
        }
        f.write_all(c)?;
    }
    f.flush()?;
    Ok(())
}
