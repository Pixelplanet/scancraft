use crate::{MarchingCubes, Mesher};
use glam::Vec3;
use kdtree::distance::squared_euclidean;
use kdtree::KdTree;
use scancraft_core::{ColorRgba, Mesh, Point3D, PointCloud};
use std::f32::consts::PI;

fn pt(p: Vec3, n: Vec3) -> Point3D {
    Point3D { position: p.to_array(), normal: n.normalize().to_array(), color: ColorRgba::WHITE, confidence: 1.0 }
}

/// Fibonacci sphere with roughly `spacing` mm between samples and outward normals.
fn sphere(r: f32, spacing: f32, center: Vec3) -> PointCloud {
    let n = (4.0 * PI * r * r / (spacing * spacing)) as usize;
    let golden = PI * (3.0 - 5f32.sqrt());
    let mut c = PointCloud::new();
    for i in 0..n {
        let y = 1.0 - 2.0 * (i as f32 + 0.5) / n as f32;
        let rad = (1.0 - y * y).sqrt();
        let th = golden * i as f32;
        let d = Vec3::new(rad * th.cos(), y, rad * th.sin());
        c.push(pt(center + d * r, d));
    }
    c
}

fn cube(side: f32, spacing: f32) -> PointCloud {
    let mut c = PointCloud::new();
    let k = (side / spacing) as i32;
    let h = side / 2.0;
    for axis in 0..3 {
        for sign in [-1.0f32, 1.0] {
            for i in 0..=k {
                for j in 0..=k {
                    let a = -h + i as f32 * side / k as f32;
                    let b = -h + j as f32 * side / k as f32;
                    let mut p = [0.0; 3];
                    p[axis] = sign * h;
                    p[(axis + 1) % 3] = a;
                    p[(axis + 2) % 3] = b;
                    let mut n = [0.0; 3];
                    n[axis] = sign;
                    c.push(pt(Vec3::from_array(p), Vec3::from_array(n)));
                }
            }
        }
    }
    c
}

/// Closed cylinder (axis Z) with caps.
fn cylinder(r: f32, height: f32, spacing: f32) -> PointCloud {
    let mut c = PointCloud::new();
    let na = (2.0 * PI * r / spacing) as i32;
    let nz = (height / spacing) as i32;
    for i in 0..na {
        let a = i as f32 / na as f32 * 2.0 * PI;
        let d = Vec3::new(a.cos(), a.sin(), 0.0);
        for j in 0..=nz {
            c.push(pt(d * r + Vec3::Z * (j as f32 * height / nz as f32), d));
        }
    }
    let steps = (r / spacing) as i32;
    for ring in 0..steps {
        let rr = r * ring as f32 / steps as f32;
        let m = ((2.0 * PI * rr / spacing) as i32).max(1);
        for i in 0..m {
            let a = i as f32 / m as f32 * 2.0 * PI;
            let xy = Vec3::new(a.cos(), a.sin(), 0.0) * rr;
            c.push(pt(xy, -Vec3::Z));
            c.push(pt(xy + Vec3::Z * height, Vec3::Z));
        }
    }
    c
}

/// One "depth frame": a curved open patch seen from +Z (normals face the scanner at +Z).
fn patch(spacing: f32) -> PointCloud {
    let mut c = PointCloud::new();
    let k = (40.0 / spacing) as i32;
    for i in 0..=k {
        for j in 0..=k {
            let x = -20.0 + i as f32 * spacing;
            let y = -20.0 + j as f32 * spacing;
            let z = 0.01 * (x * x + y * y) + 0.5 * (x * 0.2).sin();
            // Normal of z = f(x, y), facing +Z.
            let n = Vec3::new(-(0.02 * x + 0.1 * (x * 0.2).cos()), -0.02 * y, 1.0);
            c.push(pt(Vec3::new(x, y, z), n));
        }
    }
    c
}

#[test]
fn sphere_is_watertight_with_correct_volume_and_area() {
    let r = 30.0;
    let cloud = sphere(r, 0.5, Vec3::new(5.0, -3.0, 200.0));
    let mesh = Mesher::reconstruct(&cloud, 1.0);
    let a = Mesher::analyze(&mesh);
    assert!(a.is_watertight, "{a:?}");
    assert!(a.signed_volume_mm3 > 0.0, "outward winding expected: {a:?}");
    let vol = 4.0 / 3.0 * PI * r.powi(3);
    let area = 4.0 * PI * r * r;
    assert!((a.volume_mm3 - vol).abs() / vol < 0.03, "volume {} vs {}", a.volume_mm3, vol);
    // MC facets overestimate area a little; the tolerance is the spec's 3%.
    assert!((a.surface_area_mm2 - area).abs() / area < 0.03, "area {} vs {}", a.surface_area_mm2, area);

    // Vertex normals (from faces) point outwards.
    let c = Vec3::new(5.0, -3.0, 200.0);
    let outward = mesh
        .vertices
        .iter()
        .filter(|v| Vec3::from_array(v.normal).dot(Vec3::from_array(v.position) - c) > 0.0)
        .count();
    assert!(outward as f32 > 0.99 * mesh.vertices.len() as f32);
}

#[test]
fn sphere_watertight_at_other_cell_sizes() {
    let cloud = sphere(30.0, 0.6, Vec3::ZERO);
    for h in [0.7, 1.5, 2.5] {
        let a = Mesher::analyze(&Mesher::reconstruct(&cloud, h));
        assert!(a.is_watertight, "h={h}: {a:?}");
    }
}

#[test]
fn cube_and_cylinder_are_watertight() {
    let a = Mesher::analyze(&Mesher::reconstruct(&cube(40.0, 0.5), 1.0));
    assert!(a.is_watertight, "cube: {a:?}");
    assert!((a.volume_mm3 - 64000.0).abs() / 64000.0 < 0.05, "cube volume {}", a.volume_mm3);

    let a = Mesher::analyze(&Mesher::reconstruct(&cylinder(15.0, 40.0, 0.5), 1.0));
    assert!(a.is_watertight, "cylinder: {a:?}");
    let v = PI * 225.0 * 40.0;
    assert!((a.volume_mm3 - v).abs() / v < 0.05, "cylinder volume {} vs {v}", a.volume_mm3);
}

fn max_distance_to_cloud(mesh: &Mesh, cloud: &PointCloud) -> f32 {
    let mut tree = KdTree::new(3);
    for (i, p) in cloud.points.iter().enumerate() {
        tree.add(p.position, i).unwrap();
    }
    mesh.vertices
        .iter()
        .map(|v| tree.nearest(&v.position, 1, &squared_euclidean).unwrap()[0].0.sqrt())
        .fold(0.0, f32::max)
}

#[test]
fn open_patch_gives_open_surface_near_data() {
    let cloud = patch(0.4);
    let h = 1.0;
    let mesh = Mesher::reconstruct(&cloud, h);
    let a = Mesher::analyze(&mesh);
    assert!(a.triangle_count > 1000, "{a:?}");
    assert!(!a.is_watertight && a.open_edges > 0, "{a:?}");
    assert_eq!(a.non_manifold_edges, 0, "{a:?}");
    assert_eq!(a.inconsistent_edges, 0, "{a:?}");
    let d = max_distance_to_cloud(&mesh, &cloud);
    assert!(d <= 2.0 * h, "vertex {d} mm from data");
    // One sheet: area close to the patch area (40x40 plus curvature, plus a thin rim).
    assert!(a.surface_area_mm2 > 1600.0 && a.surface_area_mm2 < 2300.0, "{a:?}");
    // Faces the scanner (+Z).
    let up = mesh.vertices.iter().filter(|v| v.normal[2] > 0.0).count();
    assert!(up as f32 > 0.95 * mesh.vertices.len() as f32);
}

#[test]
fn no_surface_without_data() {
    assert!(Mesher::reconstruct(&PointCloud::new(), 1.0).triangles.is_empty());
    assert!(MarchingCubes::new(0.0).reconstruct(&sphere(10.0, 0.5, Vec3::ZERO)).triangles.is_empty());
}

#[test]
fn normals_are_estimated_when_missing() {
    // Patch without normals, scanner at the origin looking down -Z onto z ≈ 200.
    let mut cloud = patch(0.5);
    for p in &mut cloud.points {
        p.position[2] = 200.0 - p.position[2];
        p.normal = [0.0; 3];
    }
    let mesh = Mesher::reconstruct(&cloud, 1.0);
    let a = Mesher::analyze(&mesh);
    assert!(a.triangle_count > 1000 && a.non_manifold_edges == 0, "{a:?}");
    let toward = mesh.vertices.iter().filter(|v| v.normal[2] < 0.0).count();
    assert!(toward as f32 > 0.95 * mesh.vertices.len() as f32);
}

#[test]
fn smooth_and_decimate_keep_sphere_watertight() {
    let r = 30.0;
    let mut mesh = Mesher::reconstruct(&sphere(r, 0.5, Vec3::ZERO), 1.0);
    let before = Mesher::analyze(&mesh);
    Mesher::smooth(&mut mesh, 5, 0.5);
    let s = Mesher::analyze(&mesh);
    assert!(s.is_watertight, "{s:?}");
    assert!((s.volume_mm3 - before.volume_mm3).abs() / before.volume_mm3 < 0.01, "Taubin keeps volume: {s:?}");

    for ratio in [0.5, 0.2] {
        let d = Mesher::decimate(&mesh, ratio);
        let a = Mesher::analyze(&d);
        assert!(a.is_watertight, "ratio {ratio}: {a:?}");
        let got = a.triangle_count as f32 / s.triangle_count as f32;
        assert!((got - ratio).abs() < 0.05, "ratio {ratio}: got {got}");
        let vol = 4.0 / 3.0 * PI * r.powi(3);
        assert!((a.volume_mm3 - vol).abs() / vol < 0.03, "ratio {ratio}: volume {}", a.volume_mm3);
    }
}

#[test]
fn decimate_keeps_open_patch_manifold() {
    let mesh = Mesher::reconstruct(&patch(0.4), 1.0);
    let d = Mesher::decimate(&mesh, 0.3);
    let a = Mesher::analyze(&d);
    assert_eq!(a.non_manifold_edges, 0, "{a:?}");
    assert_eq!(a.inconsistent_edges, 0, "{a:?}");
    assert!((a.triangle_count as f32 / mesh.triangles.len() as f32 - 0.3).abs() < 0.05);
}
