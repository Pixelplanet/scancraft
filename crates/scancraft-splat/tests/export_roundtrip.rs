//! Our nerfstudio and COLMAP exports, loaded back through Brush's own dataset loaders, must give
//! the same cameras our in-process conversion (`convert::brush_camera`) produces. This pins the
//! coordinate conventions of all three paths against each other.

use brush_dataset::config::LoadDatasetConfig;
use scancraft_splat::export::{ExportOptions, export_dataset};
use scancraft_splat::{Distortion, convert, synthetic};
use std::path::Path;
use std::sync::Arc;

fn load(dir: &Path) -> brush_dataset::DatasetLoadResult {
    let cfg = LoadDatasetConfig {
        max_frames: None,
        max_resolution: 4096,
        eval_split_every: None,
        subsample_frames: None,
        subsample_points: None,
        alpha_mode: None,
        invert_masks: false,
        units_per_meter: 1.0,
        max_scene_batch_cache_size: 1 << 30,
    };
    tokio::runtime::Builder::new_current_thread().enable_all().build().unwrap().block_on(async {
        let vfs = Arc::new(brush_vfs::BrushVfs::from_path(dir).await.expect("vfs"));
        brush_dataset::load_dataset(vfs, &cfg).await.expect("brush loads our export")
    })
}

fn check(dir: &Path, opts: ExportOptions, distortion: Option<Distortion>) {
    let scene = synthetic::SceneConfig { width: 96, height: 72, focal: 114.0, views_per_ring: 4, init_points: 2000, ..Default::default() };
    let mut ds = synthetic::dataset(&scene);
    for img in &mut ds.images {
        img.intrinsics.cx += 3.0; // off-centre principal point
        img.intrinsics.distortion = distortion;
    }
    let _ = std::fs::remove_dir_all(dir);
    export_dataset(&ds, dir, &opts).unwrap();
    let loaded = load(dir);
    let views = &loaded.dataset.train.views;
    assert_eq!(views.len(), ds.images.len());
    for img in &ds.images {
        let file = format!("{}.png", img.name);
        let v = views.iter().find(|v| v.image.path().file_name().unwrap().to_str() == Some(&file)).expect("view by name");
        let ours = convert::brush_camera(img);
        let theirs = v.camera;
        assert!((ours.position - theirs.position).length() < 1e-5, "{}: position {:?} vs {:?}", img.name, ours.position, theirs.position);
        // |dot| rather than angle_between: acos is too coarse in f32 near identical rotations.
        let dot = ours.rotation.dot(theirs.rotation).abs();
        assert!(1.0 - dot < 1e-6, "{}: rotation {:?} vs {:?}", img.name, ours.rotation, theirs.rotation);
        assert!((ours.fov_x - theirs.fov_x).abs() < 1e-5 && (ours.fov_y - theirs.fov_y).abs() < 1e-5, "{}: fov", img.name);
        assert!((ours.center_uv - theirs.center_uv).length() < 1e-5, "{}: centre", img.name);
        if distortion.is_some() {
            assert_ne!(theirs.camera_model, brush_render::kernels::camera_model::CameraModel::Pinhole);
        }
    }
}

fn tmp(name: &str) -> std::path::PathBuf {
    std::env::temp_dir().join(format!("scancraft-splat-test-{name}-{}", std::process::id()))
}

#[test]
fn nerfstudio_export_matches_brush_camera() {
    let dir = tmp("ns");
    check(&dir, ExportOptions { nerfstudio: true, colmap: false, filter_coverage: true }, None);
    // The initial cloud is exported and picked up as Brush's init.
    assert!(load(&dir).init_splat.is_some());
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn colmap_export_matches_brush_camera() {
    let dir = tmp("colmap");
    let d = Distortion { k1: 0.05, k2: -0.01, p1: 0.001, p2: -0.0005, k3: 0.0 };
    check(&dir, ExportOptions { nerfstudio: false, colmap: true, filter_coverage: true }, Some(d));
    let _ = std::fs::remove_dir_all(&dir);
}
