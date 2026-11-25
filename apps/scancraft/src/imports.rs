//! Bringing passes in from outside the project: a scan file (PLY, OBJ, STL) becomes a fixed pass,
//! and the passes of another ScanCraft project are appended with fresh ids. Both arrive
//! unaligned, so Align All can connect them to the rest.

use std::path::Path;
use std::sync::Arc;

use scancraft_core::{Point3D, PointCloud, Transform};

use crate::isolation::{self, MaskCache};
use crate::project::{self, AlignState, LoadedPass, PassMeta, PassOrigin};

/// A scan whose largest extent is below this many units is taken to be in metres (ScanCraft works
/// in mm; no real scan is under 2 mm across).
pub const METRES_MAX_EXTENT: f32 = 2.0;

/// A scan file read for a new pass, with its object isolation already worked out.
pub struct ImportedScan {
    /// The pass name: the file name.
    pub name: String,
    pub meta: PassMeta,
    /// Its points (pass frame), with normals and colours.
    pub surface: Arc<PointCloud>,
    pub mask: MaskCache,
    /// The file looks like it is in metres.
    pub likely_metres: bool,
    pub normals: scancraft_io::NormalSource,
}

/// Largest edge of the cloud's bounding box (0 for an empty cloud).
pub fn largest_extent(cloud: &PointCloud) -> f32 {
    if cloud.is_empty() {
        return 0.0;
    }
    let size = cloud.bbox.max - cloud.bbox.min;
    size.x.max(size.y).max(size.z)
}

/// The cloud with every position multiplied by `factor` (normals unchanged for `factor > 0`).
pub fn scaled(cloud: &PointCloud, factor: f32) -> PointCloud {
    PointCloud::from_points(
        cloud
            .points
            .iter()
            .map(|p| {
                let mut q = *p;
                q.position = p.position.map(|v| v * factor);
                if factor < 0.0 {
                    q.normal = p.normal.map(|v| -v);
                }
                q
            })
            .collect(),
    )
}

/// Reads a scan file as a fixed pass. Normals come from the file, else its faces, else they are
/// estimated (`scancraft_io::import_scan`). Units are assumed to be mm.
pub fn read_scan(path: &Path) -> Result<ImportedScan, String> {
    let file = path.file_name().map(|n| n.to_string_lossy().to_string()).unwrap_or_else(|| "scan".into());
    let scan = scancraft_io::import_scan(path).map_err(|e| format!("Could not import {file}: {e}"))?;
    let points: Vec<Point3D> = scan.to_point_cloud().points.into_iter().filter(|p| p.position.iter().all(|v| v.is_finite())).collect();
    if points.is_empty() {
        return Err(format!("{file} holds no points"));
    }
    let surface = PointCloud::from_points(points);
    let likely_metres = largest_extent(&surface) < METRES_MAX_EXTENT;
    let mask = isolation::new_cache();
    // The object is worked out now, as it would be for a captured pass when first needed.
    isolation::mask_for(&mask, &project::fixed_cloud(&surface).points);
    Ok(ImportedScan { name: pass_name_for(path), meta: PassMeta::imported(file), surface: Arc::new(surface), mask, likely_metres, normals: scan.normal_source })
}

/// File names scanning programs give every export, which say nothing about the scan. Revo Scan
/// writes `raw_preview.ply` into each project folder, so three imported Revo scans used to
/// become "raw_preview.ply", "raw_preview.ply 2", "raw_preview.ply 3".
const GENERIC_NAMES: &[&str] = &[
    "raw_preview", "preview", "scan", "scans", "mesh", "model", "cloud", "pointcloud", "point_cloud", "points", "fused", "merged",
    "export", "output", "result", "untitled", "data",
];

/// The pass name for an imported file: its name without the extension, or, when that is a
/// generic export name, the name of the folder it is in (the scanning program's project).
pub fn pass_name_for(path: &Path) -> String {
    let stem = path.file_stem().map(|s| s.to_string_lossy().trim().to_string()).filter(|s| !s.is_empty()).unwrap_or_else(|| "Scan".into());
    // "raw_preview1", "scan_2", "Mesh 3": a generic name with a counter is still generic.
    let generic = |s: &str| {
        let key = s.to_ascii_lowercase().replace([' ', '-'], "_");
        GENERIC_NAMES.contains(&key.trim_end_matches(|c: char| c.is_ascii_digit() || c == '_'))
    };
    if !generic(&stem) {
        return stem;
    }
    path.parent()
        .and_then(|d| d.file_name())
        .map(|n| n.to_string_lossy().trim().to_string())
        .filter(|n| !n.is_empty() && !generic(n))
        .unwrap_or(stem)
}

/// Passes of another project, prepared to be appended: each in its own frame (identity
/// transform), unaligned, marked as added from `project`. Opens and rebuilds `dir`.
pub fn read_project(dir: &Path) -> Result<(String, Vec<LoadedPass>), String> {
    let stored = project::load(dir).map_err(|e| format!("Could not add the project: {e}"))?;
    let name = stored.name.clone();
    Ok((name.clone(), prepare_added(&name, stored.passes)))
}

/// Passes of project `name` made ready to append (see [`read_project`]): own frame, unaligned,
/// shown, marked as added from `name`, trackers rebuilt from their keyframes (so the two
/// projects never share one).
pub fn prepare_added(name: &str, passes: Vec<project::StoredPass>) -> Vec<LoadedPass> {
    let passes: Vec<project::StoredPass> = passes
        .into_iter()
        .map(|mut p| {
            p.meta.transform = Transform::default();
            p.meta.alignment = AlignState::Unaligned;
            if !matches!(p.meta.origin, PassOrigin::File { .. }) {
                p.meta.origin = PassOrigin::Project { source: name.to_string() };
            }
            p.visible = true;
            p
        })
        .collect();
    use rayon::prelude::*;
    passes.into_par_iter().map(project::rebuild).collect()
}

/// `name` if no pass has it yet, else `name (2)`, `name (3)`, ...
pub fn unique_name(name: &str, taken: &[String]) -> String {
    let mut candidate = name.to_string();
    let mut k = 2;
    while taken.contains(&candidate) {
        candidate = format!("{name} ({k})");
        k += 1;
    }
    candidate
}

/// Gives added passes fresh ids from `next_id` on (returns the next free id) and renames those
/// whose name is already taken: "Pass 1" from project "Bracket" becomes "Pass 1 (Bracket)".
pub fn remap(passes: &mut [LoadedPass], next_id: u32, taken: &[String], project: &str) -> u32 {
    let mut id = next_id;
    let mut names: Vec<String> = taken.to_vec();
    for p in passes {
        p.stored.id = id;
        id += 1;
        if names.contains(&p.stored.name) {
            p.stored.name = unique_name(&format!("{} ({project})", p.stored.name), &names);
        }
        names.push(p.stored.name.clone());
    }
    id
}

#[cfg(test)]
mod tests {
    use super::*;
    use scancraft_driver::ScanConfig;

    fn loaded(id: u32, name: &str) -> LoadedPass {
        let stored = project::StoredPass { id, name: name.into(), frames: 1, visible: true, meta: PassMeta::new(ScanConfig::default(), None, project::PassStart::Moved), keyframes: Vec::new(), surface: None, edit: None, rgb_keyframes: Vec::new(), mask: None };
        project::rebuild(stored)
    }

    #[test]
    fn added_passes_get_fresh_ids_and_unique_names() {
        let mut passes = vec![loaded(1, "Pass 1"), loaded(2, "Flipped"), loaded(7, "Pass 1 (Other)")];
        let next = remap(&mut passes, 5, &["Pass 1".to_string(), "Pass 2".to_string()], "Other");
        assert_eq!(next, 8);
        let ids: Vec<u32> = passes.iter().map(|p| p.stored.id).collect();
        assert_eq!(ids, vec![5, 6, 7]);
        let names: Vec<&str> = passes.iter().map(|p| p.stored.name.as_str()).collect();
        assert_eq!(names, vec!["Pass 1 (Other)", "Flipped", "Pass 1 (Other) (Other)"]);
        let taken = vec!["Bracket".to_string(), "Bracket (2)".to_string()];
        assert_eq!(unique_name("Bracket", &taken), "Bracket (3)");
        assert_eq!(unique_name("Shell", &taken), "Shell");
    }

    #[test]
    fn imported_passes_are_named_after_the_file_or_its_folder() {
        assert_eq!(pass_name_for(Path::new("C:/scans/bracket top.ply")), "bracket top");
        // Revo Scan's generic export name: the project folder says what the scan is.
        assert_eq!(pass_name_for(Path::new("C:/Revo/Shell inside 2/raw_preview.ply")), "Shell inside 2");
        assert_eq!(pass_name_for(Path::new("C:/Revo/Shell/Mesh.STL")), "Shell");
        assert_eq!(pass_name_for(Path::new("C:/Revo/Shell outside/raw_preview1.ply")), "Shell outside");
        // A generic folder too: keep the file's name.
        assert_eq!(pass_name_for(Path::new("C:/data/raw_preview.ply")), "raw_preview");
        assert_eq!(pass_name_for(Path::new("raw_preview.ply")), "raw_preview");
    }

    #[test]
    fn a_ply_file_becomes_a_fixed_pass_and_metres_are_noticed() {
        let dir = std::env::temp_dir().join(format!("scancraft-import-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let points: Vec<Point3D> = (0..400).map(|i| Point3D::new([(i % 20) as f32 * 0.01, (i / 20) as f32 * 0.01, 0.0]).with_normal([0.0, 0.0, 1.0])).collect();
        let path = dir.join("part.ply");
        scancraft_io::save_ply_binary(&PointCloud::from_points(points), &path).unwrap();
        let scan = read_scan(&path).unwrap();
        assert_eq!(scan.name, "part");
        assert_eq!(scan.surface.len(), 400);
        assert_eq!(scan.meta.origin, PassOrigin::File { source: "part.ply".into() });
        assert_eq!(scan.meta.alignment, AlignState::Unaligned);
        assert!(scan.likely_metres, "a 0.19 x 0.19 cloud is in metres");
        assert!(isolation::cached(&scan.mask).is_some(), "the object isolation is ready");
        let mm = scaled(&scan.surface, 1000.0);
        assert!((largest_extent(&mm) - 190.0).abs() < 0.01);
        assert!(read_scan(&dir.join("missing.ply")).is_err());
        let _ = std::fs::remove_dir_all(&dir);
    }
}
