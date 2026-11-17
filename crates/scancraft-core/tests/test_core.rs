use glam::Vec3;
use scancraft_core::{BackProjection, ColorRgba, DepthMap, PinholeIntrinsics, Point3D, PointCloud, Transform};

#[test]
fn test_point_cloud_and_bbox() {
    let mut cloud = PointCloud::new();
    assert!(cloud.is_empty());

    cloud.push(Point3D::new([10.0, 20.0, 30.0]).with_color(ColorRgba::RED));
    cloud.push(Point3D::new([-5.0, 50.0, 100.0]).with_color(ColorRgba::BLUE));

    assert_eq!(cloud.len(), 2);
    assert_eq!(cloud.bbox.min, Vec3::new(-5.0, 20.0, 30.0));
    assert_eq!(cloud.bbox.max, Vec3::new(10.0, 50.0, 100.0));
}

#[test]
fn test_depth_map_sampling() {
    let data = vec![0u16, 20000u16, 35000u16, 0u16];
    let depth_map = DepthMap::new(2, 2, 0.01, data);

    assert_eq!(depth_map.get_depth_mm(0, 0), None);
    assert!((depth_map.get_depth_mm(1, 0).unwrap() - 200.0).abs() < 1e-4);
    assert!((depth_map.get_depth_mm(0, 1).unwrap() - 350.0).abs() < 1e-4);
    assert_eq!(depth_map.get_depth_mm(1, 1), None);
}

#[test]
fn test_transform_math() {
    let t = Transform::from_translation(Vec3::new(10.0, 20.0, 30.0));
    let pt = Vec3::new(1.0, 2.0, 3.0);
    let transformed = t.transform_point3(pt);
    assert_eq!(transformed, Vec3::new(11.0, 22.0, 33.0));

    let inv = t.inverse();
    let back = inv.transform_point3(transformed);
    assert!((back - pt).length() < 1e-5);
}

#[test]
fn test_depth_map_back_projection() {
    // A fronto-parallel wall at 250 mm seen by a 64x48 camera.
    let k = PinholeIntrinsics { fx: 80.0, fy: 80.0, cx: 32.0, cy: 24.0 };
    let mut data = vec![25000u16; 64 * 48];
    data[0] = 0; // hole
    data[1] = 50000; // outside range
    let depth = DepthMap::new(64, 48, 0.01, data).with_intrinsics(k);

    let cloud = depth.to_point_cloud(150.0, 400.0);
    assert_eq!(cloud.len(), 64 * 48 - 2);
    for p in &cloud.points {
        assert!((p.position[1] - 250.0).abs() < 1e-3, "flat wall stays at Y = 250 mm");
        assert!(p.normal[1] < -0.99, "normal faces the scanner: {:?}", p.normal);
    }
    // Principal point maps to the optical axis; image rows grow downward, Z grows upward.
    let centre = cloud.points.iter().find(|p| p.position[0].abs() < 1e-3 && p.position[2].abs() < 1e-3);
    assert!(centre.is_some());
    assert!(cloud.bbox.max.z > 0.0 && cloud.bbox.min.z < 0.0);
    // Pixel (63, 24) is 31 px right of centre: x = 31 * 250 / 80.
    assert!((cloud.bbox.max.x - 31.0 * 250.0 / 80.0).abs() < 1e-3);

    assert!(DepthMap::new(2, 2, 0.01, vec![1; 4]).to_point_cloud(0.0, 1.0e6).is_empty(), "no intrinsics, no cloud");
}

#[test]
fn edge_filter_drops_flying_pixels_at_silhouettes() {
    // A box face at 250 mm in front of a wall at 300 mm, with one column of "flying pixels"
    // interpolated half-way between them along the box's right edge, and a slanted floor strip.
    let k = PinholeIntrinsics { fx: 80.0, fy: 80.0, cx: 32.0, cy: 24.0 };
    let (w, h) = (64usize, 48usize);
    let mut data = vec![30000u16; w * h];
    for v in 10..30 {
        for u in 20..40 {
            data[v * w + u] = 25000;
        }
        data[v * w + 40] = 27500; // flying pixel
    }
    for v in 40..48 {
        for u in 0..w {
            data[v * w + u] = 30000 - (v as u16 - 40) * 150; // 1.5 mm per row: a slanted surface
        }
    }
    let depth = DepthMap::new(w as u32, h as u32, 0.01, data).with_intrinsics(k);

    let all = depth.to_point_cloud(150.0, 400.0);
    assert_eq!(all.len(), w * h, "the unfiltered back-projection keeps every pixel");
    let filtered = depth.to_point_cloud_with(&BackProjection::filtered(150.0, 400.0));
    let near = |cloud: &PointCloud, y: f32| cloud.points.iter().filter(|p| (p.position[1] - y).abs() < 1.0).count();
    assert_eq!(near(&all, 275.0), 20);
    assert_eq!(near(&filtered, 275.0), 0, "no flying pixel survives");
    // Box and wall interiors are kept; only a thin band along the discontinuities goes.
    assert!(near(&filtered, 250.0) >= 16 * 16, "box interior: {}", near(&filtered, 250.0));
    assert!(filtered.len() > all.len() * 3 / 4, "{} of {}", filtered.len(), all.len());
    // A smoothly slanted surface is not a discontinuity (rows 42..47).
    let floor = filtered.points.iter().filter(|p| p.position[2] < -(42.0 - 24.0) * 290.0 / 80.0).count();
    assert!(floor >= 6 * w, "slanted floor kept: {floor}");
}
