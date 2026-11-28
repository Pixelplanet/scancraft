//! Generates a ready-to-open `.scancraft` test project containing a multi-object 3D turntable
//! scan (terracotta vase + cobalt canister + sage-green block on a slate turntable) together with
//! 18 calibrated orbiting RGB keyframes for interactive `Clean -> AI Segment` testing.
//!
//! Default output directory (outside the repo): `%LOCALAPPDATA%\ScanCraft\ai\test-projects\ai-desk-scene`
//! Override with `--out <dir>`.

use std::path::{Path, PathBuf};

use glam::Vec3;
use serde::Serialize;
use scancraft_color::RgbKeyframe;
use scancraft_core::{BackProjection, ColorCamera, ColorRgba, Mesh, Point3D, PointCloud, Transform};
use scancraft_driver::mock::MockDevice;
use scancraft_driver::{DeviceTransport, ScanConfig, ScannerDevice};
use scancraft_fusion::VoxelFilter;
use scancraft_io::project::{
    AlignmentRecord, PassRecord, ProjectFile, FORMAT_VERSION, keyframes_path, pass_dir,
    save_project,
};

#[derive(Serialize)]
struct RgbKeyframeManifestEntry {
    id: u64,
    timestamp_ms: u64,
    jpeg_file: String,
    width: u32,
    height: u32,
    camera: ColorCamera,
    sensor_pose: Transform,
    camera_pose: Transform,
    sharpness: f32,
    depth_frame_index: u64,
}

fn default_out_dir() -> PathBuf {
    let local = std::env::var_os("LOCALAPPDATA")
        .map(PathBuf::from)
        .unwrap_or_else(std::env::temp_dir);
    local.join("ScanCraft").join("ai").join("test-projects").join("ai-desk-scene")
}

fn sample_color(color: &scancraft_core::ColorFrame, cam: &ColorCamera, p_sensor: Vec3) -> ColorRgba {
    let Some((px, _)) = cam.project_sensor(p_sensor) else {
        return ColorRgba::GRAY;
    };
    let w = color.image.width as usize;
    let h = color.image.height as usize;
    let x = (px.x.round() as isize).clamp(0, w as isize - 1) as usize;
    let y = (px.y.round() as isize).clamp(0, h as isize - 1) as usize;
    let off = (y * w + x) * 3;
    let d = &color.image.data;
    ColorRgba::new(d[off], d[off + 1], d[off + 2], 255)
}

fn write_project(out_dir: &Path) -> Result<(), String> {
    const VIEW_COUNT: usize = 18;
    const WIDTH: u32 = 640;
    const HEIGHT: u32 = 512;
    const PASS_ID: u32 = 1;

    let mut raw_points: Vec<Point3D> = Vec::new();
    let mut rgb_keyframes: Vec<RgbKeyframe> = Vec::with_capacity(VIEW_COUNT);
    let back_proj = BackProjection::new(150.0, 420.0);

    for i in 0..VIEW_COUNT {
        let yaw = (i as f32) * std::f32::consts::TAU / (VIEW_COUNT as f32);
        let ts = (i as u64) * 100;
        let frame = MockDevice::render_synthetic_frame(WIDTH, HEIGHT, yaw, ts);
        let cam = frame
            .color
            .camera
            .clone()
            .ok_or_else(|| "MockDevice frame missing ColorCamera".to_string())?;

        let cloud = frame.depth.to_point_cloud_with(&back_proj);
        let mat = frame.sensor_pose.to_mat4();
        for mut pt in cloud.points {
            let p_sensor = Vec3::from(pt.position);
            pt.color = sample_color(&frame.color, &cam, p_sensor);
            pt.position = mat.transform_point3(p_sensor).to_array();
            pt.normal = (frame.sensor_pose.rotation * Vec3::from(pt.normal))
                .normalize_or_zero()
                .to_array();
            raw_points.push(pt);
        }

        let kf = RgbKeyframe::new(i as u64, &frame.color, cam, frame.sensor_pose, 80.0, i as u64)
            .ok_or_else(|| format!("Failed to encode keyframe {i}"))?;
        rgb_keyframes.push(kf);
    }

    let fused = VoxelFilter::new(0.8).filter(&PointCloud::from_points(raw_points));

    if out_dir.exists() {
        std::fs::remove_dir_all(out_dir).map_err(|e| format!("clean {}: {e}", out_dir.display()))?;
    }
    let p_dir = out_dir.join(pass_dir(PASS_ID));
    let rgb_dir = p_dir.join("rgb");
    std::fs::create_dir_all(&rgb_dir).map_err(|e| format!("mkdir {}: {e}", rgb_dir.display()))?;

    let surface_rel = format!("{}/surface.ply", pass_dir(PASS_ID));
    let mesh = Mesh::from_geometry(fused.points.clone(), Vec::new());
    scancraft_io::save_mesh_ply_binary(&mesh, out_dir.join(&surface_rel))
        .map_err(|e| format!("save surface.ply: {e}"))?;

    let mut entries = Vec::with_capacity(rgb_keyframes.len());
    for kf in &rgb_keyframes {
        let jpeg_file = format!("kf-{:06}.jpg", kf.id);
        std::fs::write(rgb_dir.join(&jpeg_file), &*kf.jpeg)
            .map_err(|e| format!("write {jpeg_file}: {e}"))?;
        entries.push(RgbKeyframeManifestEntry {
            id: kf.id,
            timestamp_ms: kf.timestamp_ms,
            jpeg_file,
            width: kf.width,
            height: kf.height,
            camera: (*kf.camera).clone(),
            sensor_pose: kf.sensor_pose,
            camera_pose: kf.camera_pose,
            sharpness: kf.sharpness,
            depth_frame_index: kf.depth_frame_index,
        });
    }
    let rgb_manifest_rel = format!("{}/rgb/keyframes.json", pass_dir(PASS_ID));
    let manifest_json = serde_json::to_string_pretty(&entries).map_err(|e| e.to_string())?;
    std::fs::write(out_dir.join(&rgb_manifest_rel), manifest_json).map_err(|e| e.to_string())?;

    let mut scan_config = serde_json::to_value(ScanConfig::default()).map_err(|e| e.to_string())?;
    if let Some(map) = scan_config.as_object_mut() {
        map.insert(
            "origin".into(),
            serde_json::json!({
                "kind": "file",
                "source": "synthetic-desk-scene",
                "surface": surface_rel,
            }),
        );
        map.insert("rgb_keyframes".into(), serde_json::json!(rgb_manifest_rel));
    }

    let device = MockDevice::new(DeviceTransport::Virtual {
        name: "simulated_scanner_0".to_string(),
    });
    let device_json = serde_json::to_value(device.info()).unwrap_or_default();

    let project = ProjectFile {
        format: FORMAT_VERSION,
        name: out_dir
            .file_name()
            .map(|n| n.to_string_lossy().to_string())
            .unwrap_or_else(|| "ai-desk-scene".to_string()),
        reference_pass: Some(PASS_ID),
        next_pass_id: PASS_ID + 1,
        passes: vec![PassRecord {
            id: PASS_ID,
            name: "Desk Scene".to_string(),
            scan_config,
            device: device_json,
            frames: VIEW_COUNT as u64,
            transform: Transform::default(),
            alignment: AlignmentRecord {
                status: "reference".to_string(),
                method: Some("reference".to_string()),
                ..Default::default()
            },
            weight: 2.0,
            included: true,
            visible: true,
            start: "moved".to_string(),
            keyframes: keyframes_path(PASS_ID),
            raw_recording: None,
            other: Default::default(),
        }],
        outputs: Vec::new(),
        settings: serde_json::json!({}),
        other: Default::default(),
    };

    save_project(out_dir, &project, &[(PASS_ID, Vec::new())])
        .map_err(|e| format!("save_project: {e}"))?;

    println!("Generated AI segmentation test project:");
    println!("  Folder:    {}", out_dir.display());
    println!("  Points:    {} fused points (0.8 mm voxel grid)", fused.len());
    println!("  Keyframes: {} calibrated RGB views (640x512)", rgb_keyframes.len());

    if let Some(kf0) = rgb_keyframes.first() {
        let report_px = |label: &str, world: Vec3| {
            if let Some((px, _)) = kf0.project_world(world) {
                println!("  {label:<18} -> ({:.1}, {:.1}) px on kf-000000.jpg", px.x, px.y);
            }
        };
        report_px("Terracotta vase", Vec3::new(0.0, -30.0, 55.0));
        report_px("Cobalt canister", Vec3::new(66.0, -7.0, 40.0));
        report_px("Sage block", Vec3::new(-62.0, -14.0, 48.0));
        report_px("Turntable rim", Vec3::new(0.0, -80.0, 0.0));
    }

    Ok(())
}

fn main() {
    let mut args = std::env::args().skip(1);
    let mut out_dir = default_out_dir();
    while let Some(arg) = args.next() {
        if arg == "--out" {
            if let Some(p) = args.next() {
                out_dir = PathBuf::from(p);
            }
        }
    }
    if let Err(e) = write_project(&out_dir) {
        eprintln!("error: {e}");
        std::process::exit(1);
    }
}
