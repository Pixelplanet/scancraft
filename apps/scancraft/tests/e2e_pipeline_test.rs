use glam::Vec3;
use scancraft_core::{DepthMap, PinholeIntrinsics, Point3D, PointCloud};
use scancraft_driver::mock::MockDriver;
use scancraft_driver::{DriverRegistry, ScanConfig};
use scancraft_fusion::{PlaneFilterMode, RansacPlaneFilter, StatisticalOutlierFilter, VoxelFilter};
use scancraft_mesh::Mesher;
use scancraft_slam::{Tracker, TrackingState};
use std::sync::Arc;
use tokio::runtime::Runtime;

/// The simulator's depth maps carry no camera model; the app back-projects them with this nominal
/// one (about 0.42 mm per pixel at 275 mm). Real devices supply their own calibration.
fn with_nominal_intrinsics(depth: DepthMap) -> DepthMap {
    if depth.intrinsics.is_some() {
        return depth;
    }
    let f = depth.width as f32 * 1.023;
    let k = PinholeIntrinsics { fx: f, fy: f, cx: depth.width as f32 * 0.5, cy: depth.height as f32 * 0.5 };
    depth.with_intrinsics(k)
}

#[test]
fn test_end_to_end_pipeline() {
    let rt = Runtime::new().expect("Failed to initialize runtime");

    // 1. Driver Discovery and Connection
    let registry = DriverRegistry::new();
    let reg_clone = registry.clone();
    rt.block_on(async move {
        reg_clone.register(Arc::new(MockDriver)).await;
    });

    let devices = rt.block_on(async { registry.discover_all().await });
    assert!(!devices.is_empty(), "Should discover mock scanner");

    let target_dev = &devices[0];
    let mut device = rt
        .block_on(async {
            registry
                .connect(&target_dev.driver_id, &target_dev.selected_transport)
                .await
        })
        .expect("Should connect to mock device");

    assert_eq!(device.info().model, "Virtual Pro 3D");

    // 2. Start Scan and Capture Frames
    let config = ScanConfig::default();
    let mut rx = rt
        .block_on(async { device.start_scan(&config).await })
        .expect("Should start scan stream");

    let mut tracker = Tracker::new();
    let mut accumulated = PointCloud::new();

    // Collect 6 frames (~400 ms of scanning)
    let frames_to_collect = 6;
    let mut collected = 0;

    while collected < frames_to_collect {
        let frame = rx
            .blocking_recv()
            .expect("Should receive frame from mock scanner");
        collected += 1;

        // Verify camera preview image and depth map are populated
        assert!(frame.ir.is_some(), "IR camera image must be present");
        let depth = frame.depth.clone().expect("mock frames carry depth");
        assert_eq!(depth.width, 640);
        assert_eq!(depth.height, 512);

        // Back-project depth map to 3D point cloud
        let cloud = with_nominal_intrinsics(depth).to_point_cloud(150.0, 400.0);
        assert!(!cloud.is_empty(), "Depth map must unproject to points");

        if let Some(imu) = frame.imu {
            tracker.update_imu(imu);
        }

        let status = tracker.register_frame(&cloud);
        assert!(
            status.state == TrackingState::Tracking || status.state == TrackingState::Slow,
            "Tracking should be healthy"
        );

        let pose = status.current_pose;
        let stride = (cloud.len() / 1000).max(1);
        for p in cloud.points.iter().step_by(stride) {
            let local_pt = Vec3::from_slice(&p.position);
            let world_pt = pose.rotation.mul_vec3(local_pt) + pose.translation;
            let world_norm = pose.rotation.mul_vec3(Vec3::from_slice(&p.normal));

            accumulated.push(
                Point3D::new([world_pt.x, world_pt.y, world_pt.z])
                    .with_normal([world_norm.x, world_norm.y, world_norm.z])
                    .with_color(p.color),
            );
        }
    }

    // Stop scanning
    rt.block_on(async { device.stop_scan().await })
        .expect("Should stop scan");

    assert!(
        accumulated.len() >= 3000,
        "Should accumulate at least 3000 points, got {}",
        accumulated.len()
    );
    assert!(!accumulated.bbox.is_empty(), "Bounding box must be valid");

    // 3. Fusion & Noise Cleanup
    // A. Voxel Centroid Grid Fusion
    let voxel_filter = VoxelFilter::new(1.0); // 1mm voxel grid
    let voxel_fused = voxel_filter.filter(&accumulated);
    assert!(voxel_fused.len() > 0 && voxel_fused.len() <= accumulated.len());

    // B. Turntable Plane Removal (RANSAC)
    let plane_filter = RansacPlaneFilter::new(2.0, 100, PlaneFilterMode::ClipBelow);
    let (no_turntable, plane) = plane_filter.filter(&voxel_fused);
    assert!(
        plane.is_some(),
        "RANSAC should detect the turntable base plane"
    );
    assert!(
        no_turntable.len() < voxel_fused.len(),
        "Turntable points should be clipped"
    );

    // C. Statistical Outlier Removal
    let sor = StatisticalOutlierFilter::new(12, 1.5);
    let cleaned = sor.filter(&no_turntable);
    assert!(
        cleaned.len() > 0,
        "Point cloud should remain populated after SOR"
    );

    // 4. Surface Meshing
    let mesh = Mesher::reconstruct(&cleaned, 2.5); // 2.5mm Marching Cubes grid
    assert!(!mesh.vertices.is_empty(), "Mesh should have vertices");
    assert!(!mesh.triangles.is_empty(), "Mesh should have triangles");

    let initial_analysis = Mesher::analyze(&mesh);
    assert!(initial_analysis.triangle_count > 0);

    // 5. Smoothing and Decimation
    let mut smoothed_mesh = mesh.clone();
    Mesher::smooth(&mut smoothed_mesh, 2, 0.3);

    let decimated_mesh = Mesher::decimate(&smoothed_mesh, 0.6);
    let decimated_analysis = Mesher::analyze(&decimated_mesh);
    assert!(decimated_analysis.triangle_count <= initial_analysis.triangle_count);

    // 6. Multi-Format I/O Export & Round-Trip Validation
    let temp_dir = std::env::temp_dir();
    let ply_path = temp_dir.join("scancraft_test.ply");
    let obj_path = temp_dir.join("scancraft_test.obj");
    let stl_path = temp_dir.join("scancraft_test.stl");

    let ply_str = ply_path.to_str().unwrap();
    let obj_str = obj_path.to_str().unwrap();
    let stl_str = stl_path.to_str().unwrap();

    // Export PLY, OBJ, and STL
    scancraft_io::save_ply_binary(&cleaned, ply_str).expect("Failed to save PLY");
    scancraft_io::save_mesh_obj(&decimated_mesh, obj_str).expect("Failed to save OBJ");
    scancraft_io::save_mesh_stl_binary(&decimated_mesh, stl_str).expect("Failed to save STL");

    assert!(ply_path.exists(), "Exported PLY must exist on disk");
    assert!(obj_path.exists(), "Exported OBJ must exist on disk");
    assert!(stl_path.exists(), "Exported STL must exist on disk");

    // Load exported PLY and verify
    let (loaded_cloud, _) = scancraft_io::load_ply(ply_str).expect("Failed to reload PLY");
    assert_eq!(
        loaded_cloud.len(),
        cleaned.len(),
        "Loaded PLY point count must match exported count"
    );

    // Load exported OBJ and verify
    let loaded_mesh = scancraft_io::load_obj(obj_str).expect("Failed to reload OBJ");
    assert_eq!(
        loaded_mesh.triangles.len(),
        decimated_mesh.triangles.len(),
        "Loaded OBJ triangle count must match"
    );

    // Clean up temporary files
    let _ = std::fs::remove_file(ply_path);
    let _ = std::fs::remove_file(obj_path);
    let _ = std::fs::remove_file(stl_path);
}
