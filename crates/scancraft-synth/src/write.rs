//! Writes a scene to disk: per pass a folder with the cloud (PLY, as the app imports it), the
//! marker list (`marker_framework.mkf`, as Revo Scan writes it) and per-point labels; for the
//! scene the reference mesh and `truth.json`.

use std::io;
use std::path::{Path, PathBuf};
use std::time::Instant;

use glam::DVec3;
use scancraft_core::{ColorRgba, Point3D, PointCloud};
use serde::{Deserialize, Serialize};

use crate::mkf::{self, MkfMarker};
use crate::scan::{scan_pass, PassStats, PointKind};
use crate::scene::{Feature, PassSpec, Scene, Sticker};
use crate::sdf::{Rigid, Shape, V3};
use crate::truth::reference_mesh;

pub const FORMAT: &str = "scancraft-synth/1";
pub const TRUTH_FILE: &str = "truth.json";
pub const REFERENCE_MESH_FILE: &str = "reference_mesh.ply";

/// Per-point label values in `<pass>.labels` (one byte per point, PLY order).
pub const LABEL_PART: u8 = 0;
pub const LABEL_STICKER: u8 = 1;
pub const LABEL_TABLE: u8 = 2;
pub const LABEL_STRAY: u8 = 3;

#[derive(Debug, Clone)]
pub struct Options {
    /// Write `reference_mesh.ply`.
    pub reference_mesh: bool,
}

impl Default for Options {
    fn default() -> Self {
        Options { reference_mesh: true }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ListedTruth {
    pub sticker: String,
    /// As written to the marker file (pass frame).
    pub centre: V3,
    pub normal: V3,
    pub radius_mm: f32,
    pub dot_points: usize,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PassTruth {
    pub id: usize,
    pub name: String,
    /// Folder of the pass, relative to the scene folder.
    pub folder: String,
    /// The cloud, relative to the scene folder.
    pub cloud: String,
    pub labels: String,
    pub marker_file: Option<String>,
    /// Maps the pass's points into the part frame (the inverse of `spec.scan_pose`). The pass's
    /// faults (`spec.offset_mm`, `spec.depth_bias_mm`) are errors and are not in it.
    pub pass_to_part: Rigid,
    pub pass_to_part_matrix: [[f64; 4]; 4],
    pub spec: PassSpec,
    pub stats: PassStats,
    pub listed_markers: Vec<ListedTruth>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MeshInfo {
    pub file: String,
    pub cell_mm: f64,
    pub vertices: usize,
    pub triangles: usize,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct StickerTruth {
    #[serde(flatten)]
    pub sticker: Sticker,
    /// Passes (1-based) whose marker file lists it.
    pub listed_in: Vec<usize>,
}

/// Everything known about a scene (`truth.json`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Truth {
    pub format: String,
    pub scene: String,
    pub description: String,
    pub tags: Vec<String>,
    pub seed: u64,
    pub units: String,
    pub frames: String,
    /// The true part (exact; evaluate with `Shape::sdf`).
    pub part: Shape,
    pub bounds: [V3; 2],
    pub reference_mesh: Option<MeshInfo>,
    pub passes: Vec<PassTruth>,
    pub stickers: Vec<StickerTruth>,
    pub features: Vec<Feature>,
}

impl Truth {
    pub fn load(scene_dir: &Path) -> io::Result<Truth> {
        let text = std::fs::read_to_string(scene_dir.join(TRUTH_FILE))?;
        serde_json::from_str(&text).map_err(|e| io::Error::new(io::ErrorKind::InvalidData, e))
    }
}

#[derive(Debug, Clone)]
pub struct Generated {
    pub dir: PathBuf,
    pub truth: Truth,
    pub bytes: u64,
    pub seconds: f64,
}

fn colour(kind: PointKind, scene: &Scene, p: DVec3) -> ColorRgba {
    match kind {
        PointKind::Part | PointKind::Stray => ColorRgba::new(176, 176, 170, 255),
        PointKind::Table => ColorRgba::new(110, 100, 92, 255),
        PointKind::Sticker(i) => {
            let s = &scene.stickers[i as usize];
            if s.local(p).0 < s.dot_radius {
                ColorRgba::new(245, 245, 245, 255)
            } else {
                ColorRgba::new(28, 28, 30, 255)
            }
        }
    }
}

fn label(kind: PointKind) -> u8 {
    match kind {
        PointKind::Part => LABEL_PART,
        PointKind::Sticker(_) => LABEL_STICKER,
        PointKind::Table => LABEL_TABLE,
        PointKind::Stray => LABEL_STRAY,
    }
}

fn to_io(e: scancraft_io::IoError) -> io::Error {
    io::Error::other(e.to_string())
}

/// Scans every pass of `scene` and writes the scene folder `out/<scene.name>` (replacing an
/// older one).
pub fn generate(scene: &Scene, out: &Path, options: &Options) -> io::Result<Generated> {
    let t0 = Instant::now();
    let dir = out.join(&scene.name);
    if dir.join(TRUTH_FILE).is_file() {
        std::fs::remove_dir_all(&dir)?;
    }
    std::fs::create_dir_all(&dir)?;
    let mut passes = Vec::new();
    let mut listed_in: Vec<Vec<usize>> = vec![Vec::new(); scene.stickers.len()];
    for (i, spec) in scene.passes.iter().enumerate() {
        let id = i + 1;
        let folder = format!("pass-{id}");
        std::fs::create_dir_all(dir.join(&folder))?;
        let scan = scan_pass(scene, i);
        let pose = spec.scan_pose;
        let points: Vec<Point3D> = scan
            .points
            .iter()
            .map(|p| Point3D {
                position: pose.apply(p.pos).as_vec3().to_array(),
                normal: pose.apply_dir(p.normal).normalize_or_zero().as_vec3().to_array(),
                color: colour(p.kind, scene, p.true_pos),
                confidence: p.hits as f32,
            })
            .collect();
        let cloud_name = format!("{}-pass-{id}.ply", scene.name);
        scancraft_io::save_ply_binary(&PointCloud::from_points(points), dir.join(&folder).join(&cloud_name)).map_err(to_io)?;
        let labels_name = format!("{}-pass-{id}.labels", scene.name);
        std::fs::write(dir.join(&folder).join(&labels_name), scan.points.iter().map(|p| label(p.kind)).collect::<Vec<u8>>())?;

        let has_stickers = scene.stickers.iter().any(|s| s.on_table_of_pass.is_none_or(|p| p == i));
        let mut listed = Vec::new();
        let marker_file = if has_stickers {
            let records: Vec<MkfMarker> = scan
                .listed
                .iter()
                .map(|m| MkfMarker { centre: pose.apply(m.centre).to_array(), normal: pose.apply_dir(m.normal).normalize().to_array(), radius_mm: m.radius_mm })
                .collect();
            for (m, r) in scan.listed.iter().zip(&records) {
                listed_in[m.sticker].push(id);
                listed.push(ListedTruth { sticker: scene.stickers[m.sticker].id.clone(), centre: r.centre, normal: r.normal, radius_mm: r.radius_mm, dot_points: m.dot_points });
            }
            let name = scancraft_io::revo::MARKER_FRAMEWORK_FILE;
            mkf::write(&dir.join(&folder).join(name), &records)?;
            Some(format!("{folder}/{name}"))
        } else {
            None
        };
        let pass_to_part = pose.inverse();
        passes.push(PassTruth {
            id,
            name: spec.name.clone(),
            cloud: format!("{folder}/{cloud_name}"),
            labels: format!("{folder}/{labels_name}"),
            folder,
            marker_file,
            pass_to_part,
            pass_to_part_matrix: pass_to_part.matrix(),
            spec: spec.clone(),
            stats: scan.stats,
            listed_markers: listed,
        });
    }

    let reference = if options.reference_mesh {
        let f = |p: DVec3| scene.part.sdf(p);
        let (mesh, cell) = reference_mesh(&f, scene.bounds, scene.truth_cell_mm);
        scancraft_io::save_mesh_ply_binary(&mesh, dir.join(REFERENCE_MESH_FILE)).map_err(to_io)?;
        Some(MeshInfo { file: REFERENCE_MESH_FILE.into(), cell_mm: cell, vertices: mesh.vertices.len(), triangles: mesh.triangles.len() })
    } else {
        None
    };

    let truth = Truth {
        format: FORMAT.into(),
        scene: scene.name.clone(),
        description: scene.description.clone(),
        tags: scene.tags.clone(),
        seed: scene.seed,
        units: "mm".into(),
        frames: "Everything here is in the part frame unless it says otherwise. Pass 1 is written in the part frame (its \
                 pass_to_part is the identity), so a project with pass 1 as the reference is in the part frame. Other passes \
                 are written in their own frame (the part re-placed for that pass): part = pass_to_part · pass point. Marker \
                 files are in their pass's frame."
            .into(),
        part: scene.part.clone(),
        bounds: scene.bounds,
        reference_mesh: reference,
        passes,
        stickers: scene.stickers.iter().cloned().zip(listed_in).map(|(sticker, listed_in)| StickerTruth { sticker, listed_in }).collect(),
        features: scene.features.clone(),
    };
    let json = serde_json::to_string_pretty(&truth).map_err(io::Error::other)?;
    std::fs::write(dir.join(TRUTH_FILE), json)?;
    let bytes = dir_size(&dir);
    Ok(Generated { dir, truth, bytes, seconds: t0.elapsed().as_secs_f64() })
}

/// Total size of the files under `dir`.
pub fn dir_size(dir: &Path) -> u64 {
    let mut total = 0;
    if let Ok(entries) = std::fs::read_dir(dir) {
        for e in entries.flatten() {
            let p = e.path();
            total += if p.is_dir() { dir_size(&p) } else { e.metadata().map(|m| m.len()).unwrap_or(0) };
        }
    }
    total
}
