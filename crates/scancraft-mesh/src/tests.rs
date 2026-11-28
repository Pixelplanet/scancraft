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

// ---------------------------------------------------------------- robust implicit surface

/// Deterministic noise in [-a, a].
fn jitter(i: usize, salt: u32, a: f32) -> f32 {
    let x = (i as u32).wrapping_mul(2654435761).wrapping_add(salt.wrapping_mul(40503));
    let x = x ^ (x >> 15);
    let x = x.wrapping_mul(2246822519) ^ (x >> 13);
    ((x % 10_000) as f32 / 5000.0 - 1.0) * a
}

/// Adds position noise along the normal and normal noise (degrees) to a cloud.
fn noisy(mut c: PointCloud, pos_mm: f32, normal_deg: f32) -> PointCloud {
    for (i, p) in c.points.iter_mut().enumerate() {
        let n = Vec3::from_array(p.normal);
        let q = Vec3::from_array(p.position) + n * jitter(i, 1, pos_mm);
        let t = n.any_orthonormal_vector();
        let b = n.cross(t);
        let a = normal_deg.to_radians();
        let nn = (n + t * jitter(i, 2, a) + b * jitter(i, 3, a)).normalize();
        p.position = q.to_array();
        p.normal = nn.to_array();
    }
    c
}

#[test]
fn implicit_surface_of_a_noisy_sphere_is_closed_smooth_and_accurate() {
    let r = 30.0;
    let c = Vec3::new(5.0, -3.0, 200.0);
    let cloud = noisy(sphere(r, 0.5, c), 0.08, 15.0);
    let (mesh, stats) = crate::implicit::reconstruct(&cloud, &crate::SurfaceConfig::default());
    let a = Mesher::analyze(&mesh);
    assert!(a.is_watertight, "{a:?} {stats:?}");
    assert!(a.signed_volume_mm3 > 0.0, "outward winding expected");
    // Vertices on the true sphere within a fraction of the noise.
    let err: Vec<f32> = mesh.vertices.iter().map(|v| (Vec3::from_array(v.position).distance(c) - r).abs()).collect();
    let mean = err.iter().sum::<f32>() / err.len() as f32;
    let max = err.iter().copied().fold(0.0, f32::max);
    assert!(mean < 0.03 && max < 0.15, "mean {mean} max {max} mm off the sphere (noise ±0.08)");
    // Normals (field gradients) point outwards and close to radial.
    let off = mesh.vertices.iter().map(|v| Vec3::from_array(v.normal).dot((Vec3::from_array(v.position) - c).normalize())).fold(1.0f32, f32::min);
    assert!(off > 0.95, "a vertex normal is {:.1}° off radial", off.acos().to_degrees());
}

#[test]
fn implicit_surface_keeps_both_sides_of_a_thin_wall() {
    // Two parallel plates 2 mm apart facing away from each other (a 2 mm wall), 0.4 mm spacing.
    let mut c = PointCloud::new();
    let k = 100;
    for i in 0..=k {
        for j in 0..=k {
            let (x, y) = (i as f32 * 0.4, j as f32 * 0.4);
            c.push(pt(Vec3::new(x, y, 1.0), Vec3::Z));
            c.push(pt(Vec3::new(x, y, -1.0), -Vec3::Z));
        }
    }
    let (mesh, _) = crate::implicit::reconstruct(&c, &crate::SurfaceConfig { cell_mm: 0.5, detail: 4.0, ..Default::default() });
    // Every vertex away from the plate border lies on one of the two plates.
    let inner: Vec<f32> = mesh.vertices.iter().filter(|v| v.position[0] > 3.0 && v.position[0] < 37.0 && v.position[1] > 3.0 && v.position[1] < 37.0).map(|v| v.position[2]).collect();
    assert!(inner.len() > 1000);
    let worst = inner.iter().map(|z| (z.abs() - 1.0).abs()).fold(0.0f32, f32::max);
    assert!(worst < 0.1, "a vertex is {worst} mm off the wall faces");
    let top = inner.iter().filter(|z| **z > 0.0).count();
    assert!(top > inner.len() / 3 && top < inner.len() * 2 / 3, "both faces present: {top} of {}", inner.len());
}

#[test]
fn implicit_surface_of_an_open_patch_stays_open_and_on_the_data() {
    let cloud = noisy(patch(0.4), 0.05, 10.0);
    let (mesh, _) = crate::implicit::reconstruct(&cloud, &crate::SurfaceConfig { cell_mm: 0.7, ..Default::default() });
    let a = Mesher::analyze(&mesh);
    assert!(!a.is_watertight && a.open_edges > 0, "{a:?}");
    assert_eq!(a.non_manifold_edges, 0, "{a:?}");
    assert_eq!(a.inconsistent_edges, 0, "{a:?}");
    assert!(max_distance_to_cloud(&mesh, &cloud) < 1.0);
    assert!(a.surface_area_mm2 > 1600.0 && a.surface_area_mm2 < 2200.0, "{a:?}");
}

#[test]
fn implicit_surface_drops_stray_points_up_to_the_support_radius() {
    // A noisy plane (0.4 mm spacing, ±0.05 mm) with single stray points 0.3–1.4 mm above it, the
    // range where counting a point in its own neighbourhood used to hide the far ones.
    let mut c = PointCloud::new();
    let k = 100;
    for i in 0..=k {
        for j in 0..=k {
            let idx = (i * (k + 1) + j) as usize;
            c.push(pt(Vec3::new(i as f32 * 0.4, j as f32 * 0.4, jitter(idx, 7, 0.05)), Vec3::Z));
        }
    }
    let mut strays = 0;
    for (n, off) in [0.3f32, 0.5, 0.7, 0.9, 1.0, 1.1, 1.2, 1.3, 1.4].iter().enumerate() {
        for m in 0..6 {
            let (x, y) = (4.0 + 5.0 * m as f32 + 0.13, 4.0 + 3.5 * n as f32 + 0.17);
            c.push(pt(Vec3::new(x, y, *off), Vec3::Z));
            strays += 1;
        }
    }
    let cfg = crate::SurfaceConfig { cell_mm: 0.5, detail: 3.75, ..Default::default() };
    let (mesh, stats) = crate::implicit::reconstruct(&c, &cfg);
    assert!(stats.removed_outliers >= strays, "{} of {strays} strays removed: {stats:?}", stats.removed_outliers);
    let worst = mesh.vertices.iter().filter(|v| v.position[0] > 2.0 && v.position[0] < 38.0 && v.position[1] > 2.0 && v.position[1] < 38.0).map(|v| v.position[2].abs()).fold(0.0f32, f32::max);
    assert!(worst < 0.08, "a bump of {worst} mm is left where a stray point was");
}

/// Raised strokes on a plane, as moulded lettering: four ribs (0.3–1 mm high, 1–1.5 mm wide)
/// and a ring. Returns (distance to the stroke's centre line, top half-width, height) of the
/// nearest stroke.
fn relief_stroke(x: f32, y: f32) -> (f32, f32, f32) {
    let ribs = [(8.0f32, 0.75f32, 0.3f32), (14.0, 0.6, 0.5), (20.0, 0.5, 0.8), (26.0, 0.5, 1.0)];
    let mut best = (f32::MAX, 0.0, 0.0);
    for (cx, half, h) in ribs {
        let d = Vec3::new(x - cx, y - y.clamp(6.0, 24.0), 0.0).length();
        if d - half < best.0 - best.1 {
            best = (d, half, h);
        }
    }
    let ring = ((Vec3::new(x - 34.0, y - 15.0, 0.0).length()) - 3.0).abs();
    if ring - 0.5 < best.0 - best.1 {
        best = (ring, 0.5, 0.5);
    }
    best
}

/// Height of the relief at (x, y): flat tops, flanks 0.3 mm wide.
fn relief_height(x: f32, y: f32) -> f32 {
    let (d, half, h) = relief_stroke(x, y);
    h * ((half + 0.3 - d) / 0.3).clamp(0.0, 1.0)
}

/// The relief sampled like a scan from above: 0.5 mm spacing with lateral jitter, ±0.05 mm
/// height noise, normals 10° off.
fn relief_cloud() -> PointCloud {
    let mut c = PointCloud::new();
    for i in 0..=84usize {
        for j in 0..=60usize {
            let idx = i * 61 + j;
            let (x, y) = (i as f32 * 0.5 + jitter(idx, 11, 0.15), j as f32 * 0.5 + jitter(idx, 12, 0.15));
            let e = 0.05;
            let n = Vec3::new(-(relief_height(x + e, y) - relief_height(x - e, y)) / (2.0 * e), -(relief_height(x, y + e) - relief_height(x, y - e)) / (2.0 * e), 1.0);
            c.push(pt(Vec3::new(x, y, relief_height(x, y)), n));
        }
    }
    let mut c = noisy(c, 0.0, 10.0);
    for (i, p) in c.points.iter_mut().enumerate() {
        p.position[2] += jitter(i, 13, 0.05);
    }
    c
}

/// What a mesh of [`relief_cloud`] looks like, in numbers.
#[derive(Debug)]
struct ReliefReport {
    /// Open edges away from the border of the patch: holes.
    inner_open_edges: usize,
    /// Mean height on the flat top of each stroke as a fraction of its true height (worst one).
    worst_top_fraction: f32,
    /// Furthest a vertex lies below / above every true height within 0.5 mm of it (mm).
    deepest_pit: f32,
    highest_spike: f32,
    /// RMS height on the plain area, 2.5 mm and more from any stroke (mm).
    plain_rms: f32,
}

fn relief_report(mesh: &Mesh) -> ReliefReport {
    let inside = |p: [f32; 3]| p[0] > 2.5 && p[0] < 39.5 && p[1] > 2.5 && p[1] < 27.5;
    let inner_open_edges = open_edges(mesh).into_iter().filter(|&(a, b)| inside(mesh.vertices[a as usize].position) && inside(mesh.vertices[b as usize].position)).count();
    let mut tops: std::collections::HashMap<u32, (f32, u32)> = std::collections::HashMap::new();
    let (mut pit, mut spike, mut plain, mut plain_n) = (0.0f32, 0.0f32, 0.0f32, 0u32);
    for v in mesh.vertices.iter().filter(|v| inside(v.position)) {
        let [x, y, z] = v.position;
        let (d, half, h) = relief_stroke(x, y);
        if d < half * 0.5 {
            let e = tops.entry((h * 1000.0) as u32 + (x > 30.0) as u32).or_default();
            e.0 += z / h;
            e.1 += 1;
        }
        if d > half + 2.8 {
            plain += z * z;
            plain_n += 1;
        }
        let (mut lo, mut hi) = (f32::MAX, f32::MIN);
        for k in 0..25 {
            let hh = relief_height(x + ((k % 5) as f32 - 2.0) * 0.25, y + ((k / 5) as f32 - 2.0) * 0.25);
            lo = lo.min(hh);
            hi = hi.max(hh);
        }
        pit = pit.max(lo - z);
        spike = spike.max(z - hi);
    }
    ReliefReport {
        inner_open_edges,
        worst_top_fraction: tops.values().map(|(s, n)| s / *n as f32).fold(f32::MAX, f32::min),
        deepest_pit: pit,
        highest_spike: spike,
        plain_rms: (plain / plain_n.max(1) as f32).sqrt(),
    }
}

#[test]
fn implicit_surface_keeps_small_relief_without_pits() {
    let cloud = relief_cloud();
    // The app's defaults: 0.7 mm cells, then 4 smoothing passes at Keep edges 0.7.
    let (mut mesh, stats) = crate::implicit::reconstruct(&cloud, &crate::SurfaceConfig::default());
    let raw = relief_report(&mesh);
    crate::FeatureSmoother { iterations: 4, sigma_normal: 0.5144, ..Default::default() }.smooth(&mut mesh);
    let r = relief_report(&mesh);
    // Before the fixes: 356 open edges inside, pits of 0.59 mm and spikes of 0.38 mm (raw).
    assert_eq!(raw.inner_open_edges + r.inner_open_edges, 0, "holes in the relief: {raw:?} {stats:?}");
    assert!(raw.worst_top_fraction > 0.9 && r.worst_top_fraction > 0.7, "relief flattened: {raw:?} {r:?}");
    assert!(raw.deepest_pit < 0.12 && raw.highest_spike < 0.15, "pits or spikes: {raw:?}");
    assert!(r.deepest_pit < 0.2 && r.highest_spike < 0.15, "pits or spikes after smoothing: {r:?}");
    assert!(raw.plain_rms < 0.02 && r.plain_rms < 0.01, "plain area rough: {raw:?} {r:?}");
    // The old mesher, for reference: it keeps the relief too, on a rougher plain area.
    let old = relief_report(&MarchingCubes::new(0.7).reconstruct(&cloud));
    assert!(raw.plain_rms < old.plain_rms, "{raw:?} against the old mesher's {old:?}");
}

/// Open edges with both ends inside the rectangle (3..37, 3..27) of the xy plane: holes.
fn inner_open_edges(mesh: &Mesh) -> usize {
    let inside = |p: [f32; 3]| p[0] > 3.0 && p[0] < 37.0 && p[1] > 3.0 && p[1] < 27.0;
    open_edges(mesh).into_iter().filter(|&(a, b)| inside(mesh.vertices[a as usize].position) && inside(mesh.vertices[b as usize].position)).count()
}

fn open_edges(mesh: &Mesh) -> Vec<(u32, u32)> {
    let mut edges: std::collections::HashMap<(u32, u32), u32> = std::collections::HashMap::new();
    for t in &mesh.triangles {
        let [a, b, c] = t.indices;
        for (u, v) in [(a, b), (b, c), (c, a)] {
            *edges.entry((u.min(v), u.max(v))).or_default() += 1;
        }
    }
    edges.into_iter().filter(|(_, n)| *n == 1).map(|(k, _)| k).collect()
}

#[test]
fn implicit_surface_drops_a_sparse_layer_behind_the_surface() {
    // A noisy plane (0.5 mm pitch) and, along three strokes, a thin scatter of points 1 mm
    // behind it, facing either way: what ink marks look like in a merged scan. They sit inside
    // the support radius and used to tear the plane into flaps and holes.
    let mut c = PointCloud::new();
    for i in 0..=80usize {
        for j in 0..=60usize {
            let idx = i * 61 + j;
            c.push(pt(Vec3::new(i as f32 * 0.5 + jitter(idx, 21, 0.1), j as f32 * 0.5 + jitter(idx, 22, 0.1), jitter(idx, 23, 0.04)), Vec3::Z));
        }
    }
    let plane = c.len();
    for s in 0..3usize {
        for k in 0..90usize {
            let idx = 9000 + s * 100 + k;
            let (x, y) = (8.0 + 0.27 * k as f32, 8.0 + 7.0 * s as f32 + jitter(idx, 24, 0.9));
            let n = if k % 3 == 0 { Vec3::Z } else { Vec3::new(jitter(idx, 25, 0.5), jitter(idx, 26, 0.5), -1.0) };
            c.push(pt(Vec3::new(x, y, -1.0 + jitter(idx, 27, 0.25)), n));
        }
    }
    let strays = c.len() - plane;
    let (mesh, stats) = crate::implicit::reconstruct(&noisy(c, 0.0, 10.0), &crate::SurfaceConfig::default());
    assert!(stats.removed_outliers >= strays * 9 / 10 && stats.removed_outliers < strays + 60, "{} of {strays} stray points removed: {stats:?}", stats.removed_outliers);
    let worst = mesh.vertices.iter().filter(|v| v.position[0] > 3.0 && v.position[0] < 37.0 && v.position[1] > 3.0 && v.position[1] < 27.0).map(|v| v.position[2].abs()).fold(0.0f32, f32::max);
    assert!(worst < 0.12, "the surface is {worst} mm off the plane where the stray layer was");
    assert_eq!(inner_open_edges(&mesh), 0, "holes where the stray layer was");
}

#[test]
fn implicit_surface_has_round_open_edges() {
    // A flat patch whose sides run at 20° to the voxel grid: its rim must be a line, not stairs
    // one cell high.
    let (s, co) = 20f32.to_radians().sin_cos();
    let mut c = PointCloud::new();
    for i in 0..=80usize {
        for j in 0..=80usize {
            let (x, y) = (i as f32 * 0.5, j as f32 * 0.5);
            c.push(pt(Vec3::new(co * x - s * y, s * x + co * y, jitter(i * 81 + j, 31, 0.03)), Vec3::Z));
        }
    }
    let (mesh, _) = crate::implicit::reconstruct(&c, &crate::SurfaceConfig::default());
    // Rim vertices along the side y' = 0 (patch coordinates), away from the corners: their
    // distance from that side, less its mean, is the waviness of the rim.
    let d: Vec<f32> = open_edges(&mesh)
        .into_iter()
        .filter_map(|(a, _)| {
            let p = mesh.vertices[a as usize].position;
            let (x, y) = (co * p[0] + s * p[1], -s * p[0] + co * p[1]);
            (x > 5.0 && x < 35.0 && y < 3.0).then_some(y)
        })
        .collect();
    assert!(d.len() > 30, "{} rim vertices", d.len());
    let mean = d.iter().sum::<f32>() / d.len() as f32;
    let rms = (d.iter().map(|y| (y - mean).powi(2)).sum::<f32>() / d.len() as f32).sqrt();
    assert!(rms < 0.1, "the rim wanders {rms} mm around its line (stairs of a 0.7 mm cell: 0.2 mm)");
}


// ---------------------------------------------------------------- preparation (prepare.rs)

/// The back face of a 3 mm wall under the xy rectangle [0, w] x [0, h]: z = -3, facing -z.
fn back_face(c: &mut PointCloud, w: f32, h: f32) {
    let (ni, nj) = ((w / 0.5) as usize, (h / 0.5) as usize);
    for i in 0..=ni {
        for j in 0..=nj {
            let idx = 50_000 + i * (nj + 1) + j;
            c.push(pt(Vec3::new(i as f32 * 0.5 + jitter(idx, 41, 0.1), j as f32 * 0.5 + jitter(idx, 42, 0.1), -3.0 + jitter(idx, 43, 0.04)), -Vec3::Z));
        }
    }
}

/// A 3 mm wall (front face z = 0, 0.5 mm pitch) and, as the cross-line laser leaves them behind an
/// inside surface, two families of curved stray lines 0.8 mm behind the front face, 2 mm wide,
/// 9 and 10 mm apart, as dense as the surface along a line, five in six facing the other way.
/// Returns the cloud and the number of strays (the last points).
fn wall_with_laser_strays() -> (PointCloud, usize) {
    let mut c = PointCloud::new();
    for i in 0..=80usize {
        for j in 0..=60usize {
            let idx = i * 61 + j;
            c.push(pt(Vec3::new(i as f32 * 0.5 + jitter(idx, 44, 0.1), j as f32 * 0.5 + jitter(idx, 45, 0.1), jitter(idx, 46, 0.04)), Vec3::Z));
        }
    }
    back_face(&mut c, 40.0, 30.0);
    let wall = c.len();
    let mut idx = 90_000;
    for i in 0..=80usize {
        for j in 0..=60usize {
            let (x, y) = (i as f32 * 0.5, j as f32 * 0.5);
            // Line families along y (bent) and along 60 degrees.
            let a = (x + 0.02 * (y - 15.0).powi(2)).rem_euclid(9.0);
            let b = (0.866 * x - 0.5 * y + 40.0).rem_euclid(10.0);
            if a < 2.0 || b < 2.0 {
                idx += 1;
                let n = if idx % 6 == 0 { Vec3::Z } else { Vec3::new(jitter(idx, 47, 0.3), jitter(idx, 48, 0.3), -1.0) };
                c.push(pt(Vec3::new(x + 0.21 + jitter(idx, 49, 0.1), y + 0.17 + jitter(idx, 50, 0.1), -0.8 + jitter(idx, 51, 0.12)), n));
            }
        }
    }
    let strays = c.len() - wall;
    (noisy(c, 0.0, 8.0), strays)
}

/// Worst |z| of the front-face vertices inside (3..37, 3..27).
fn front_face_worst(mesh: &Mesh) -> f32 {
    mesh.vertices
        .iter()
        .filter(|v| v.position[0] > 3.0 && v.position[0] < 37.0 && v.position[1] > 3.0 && v.position[1] < 27.0 && v.position[2] > -1.5)
        .map(|v| v.position[2].abs())
        .fold(0.0f32, f32::max)
}

#[test]
fn prepare_drops_laser_line_strays_behind_a_thin_wall() {
    let (cloud, strays) = wall_with_laser_strays();
    let cfg = crate::SurfaceConfig::default();
    // Without the preparation the mesher's own sparse-layer test keeps most of the lines (they
    // are as dense as the surface) and the front face is pitted.
    let (before, _) = crate::implicit::reconstruct(&cloud, &cfg);
    let pitted = front_face_worst(&before);
    assert!(pitted > 0.25, "the strays used to pit the surface: {pitted}");
    let (prepared, rep) = crate::prepare_points(&cloud, &[], &crate::PrepareConfig::default());
    let wall = cloud.len() - strays;
    let left = prepared.points.iter().filter(|p| p.position[2] < -0.4 && p.position[2] > -1.4).count();
    let dropped_wall = wall - prepared.points.iter().filter(|p| p.position[2] > -0.4 || p.position[2] < -2.0).count();
    eprintln!("prepare: {rep:?}; {left} of {strays} strays left, {dropped_wall} of {wall} wall points dropped; pitted {pitted:.3}");
    assert!(left * 20 <= strays, "{left} of {strays} stray points left");
    assert!(dropped_wall * 100 <= wall, "{dropped_wall} of {wall} wall points dropped");
    let (mesh, _) = crate::implicit::reconstruct(&prepared, &cfg);
    let worst = front_face_worst(&mesh);
    assert!(worst < 0.1, "the front face is {worst} mm off the plane after the preparation");
    assert_eq!(inner_open_edges(&mesh), 0, "holes where the strays were");
}

/// The triangles of `mesh` above z = -1.5 (the front face of a [`back_face`] wall), compacted.
fn front_face(mesh: &Mesh) -> Mesh {
    let mut map = vec![u32::MAX; mesh.vertices.len()];
    let mut out = Mesh::new();
    for t in &mesh.triangles {
        if t.indices.iter().all(|&i| mesh.vertices[i as usize].position[2] > -1.5) {
            let [a, b, c] = t.indices.map(|i| {
                if map[i as usize] == u32::MAX {
                    map[i as usize] = out.add_vertex(mesh.vertices[i as usize]);
                }
                map[i as usize]
            });
            out.add_triangle(a, b, c);
        }
    }
    out
}

/// The trade-off of the preparation, measured on the relief of
/// `implicit_surface_keeps_small_relief_without_pits` as the inside of a 3 mm wall: with
/// artefact suppression on (the default for scanner data) the ribs and the ring must still come
/// out as they do without it; with it off the result is the mesher's own.
#[test]
fn prepare_keeps_small_relief() {
    let mut cloud = relief_cloud();
    back_face(&mut cloud, 42.0, 30.0);
    for suppress in [true, false] {
        let cfg = crate::PrepareConfig { suppress_artefacts: suppress, ..Default::default() };
        let (prepared, rep) = crate::prepare_points(&cloud, &[], &cfg);
        let (mesh, _) = crate::implicit::reconstruct(&prepared, &crate::SurfaceConfig::default());
        let r = relief_report(&front_face(&mesh));
        eprintln!("suppress {suppress}: {r:?}, dropped {} + {}", rep.dropped_off_surface, rep.dropped_ghosts);
        assert_eq!(r.inner_open_edges, 0, "holes in the relief: {r:?}");
        assert!(r.worst_top_fraction > 0.9, "relief flattened (suppress {suppress}): {r:?}");
        assert!(r.deepest_pit < 0.12 && r.highest_spike < 0.15, "pits or spikes (suppress {suppress}): {r:?}");
        assert!(r.plain_rms < 0.02, "plain area rough: {r:?}");
    }
}

/// A gently curved 3 mm wall with a marker sticker on its front face: a flat disc 0.4 mm proud,
/// 5.5 mm in radius, listed with Revo's 3 mm radius (the white dot only).
fn wall_with_sticker() -> (PointCloud, Vec3) {
    let dome = |x: f32, y: f32| 0.004 * ((x - 20.0).powi(2) + (y - 15.0).powi(2));
    let site = Vec3::new(18.0, 14.0, dome(18.0, 14.0));
    let mut c = PointCloud::new();
    for i in 0..=80usize {
        for j in 0..=60usize {
            let idx = i * 61 + j;
            let (x, y) = (i as f32 * 0.5 + jitter(idx, 61, 0.1), j as f32 * 0.5 + jitter(idx, 62, 0.1));
            let e = 0.05;
            let n = Vec3::new(-(dome(x + e, y) - dome(x - e, y)) / (2.0 * e), -(dome(x, y + e) - dome(x, y - e)) / (2.0 * e), 1.0).normalize();
            let r = Vec3::new(x - site.x, y - site.y, 0.0).length();
            let lift = 0.4 * ((5.5 - r) / 0.5).clamp(0.0, 1.0);
            c.push(pt(Vec3::new(x, y, dome(x, y) + lift + jitter(idx, 63, 0.03)), n));
        }
    }
    back_face(&mut c, 40.0, 30.0);
    (noisy(c, 0.0, 6.0), site)
}

#[test]
fn prepare_removes_a_listed_marker_flush_with_the_surface() {
    use crate::solid::markers::{Marker, MarkerSource};
    let (cloud, site) = wall_with_sticker();
    let dome = |x: f32, y: f32| 0.004 * ((x - 20.0).powi(2) + (y - 15.0).powi(2));
    let marker = Marker { id: "M1".into(), center: site, normal: Vec3::Z, scan: Some(1), source: MarkerSource::List, radius_mm: 3.0, score: 0.0 };
    // Worst deviation from the dome of the front-face vertices within 7 mm of the site.
    let site_error = |mesh: &Mesh| {
        mesh.vertices
            .iter()
            .map(|v| Vec3::from_array(v.position))
            .filter(|p| p.z > -1.5 && Vec3::new(p.x - site.x, p.y - site.y, 0.0).length() < 7.0)
            .map(|p| (p.z - dome(p.x, p.y)).abs())
            .fold(0.0f32, f32::max)
    };
    let mesh_of = |remove: bool| {
        let cfg = crate::PrepareConfig { remove_markers: remove, ..Default::default() };
        let (prepared, rep) = crate::prepare_points(&cloud, std::slice::from_ref(&marker), &cfg);
        let (mut mesh, _) = crate::implicit::reconstruct(&prepared, &crate::SurfaceConfig::default());
        crate::FeatureSmoother { iterations: 4, sigma_normal: 0.5144, ..Default::default() }.smooth(&mut mesh);
        if remove {
            assert_eq!(rep.markers_filled(), 1, "{:?}", rep.marker_outcomes);
            // The cut reaches past the plateau, not just Revo's 3 mm dot.
            assert!(rep.marker_sites[0].radius_mm <= 3.0 && crate::solid::markers::MarkerConfig::default().cut_radius_mm >= 6.0);
            let fair = crate::solid::fair_marker_sites(&mut mesh, &rep.marker_sites, &cfg.markers);
            assert_eq!(fair[0].status, "faired", "{fair:?}");
        }
        mesh
    };
    let kept = site_error(&mesh_of(false));
    assert!(kept > 0.3, "without removal the sticker stays: {kept}");
    let removed = site_error(&mesh_of(true));
    assert!(removed < 0.03, "the site is {removed} mm off the surface around it");
}

/// A 3 mm wall whose front face is a dome with a ridge running through a marker sticker (a flat
/// disc 0.4 mm proud, r 5.5 mm, listed with Revo's 3 mm radius), 0.03 mm noise.
fn ridged_wall_with_sticker() -> (PointCloud, Vec3, fn(f32, f32) -> f32) {
    fn surface(x: f32, y: f32) -> f32 {
        let (dx, dy) = (x - 20.0, y - 15.0);
        -0.004 * (dx * dx + dy * dy) + 0.15 * (-(dx - 1.5).powi(2) / 50.0).exp()
    }
    let site = Vec3::new(20.0, 15.0, surface(20.0, 15.0));
    let mut c = PointCloud::new();
    for i in 0..=80usize {
        for j in 0..=60usize {
            let idx = i * 61 + j;
            let (x, y) = (i as f32 * 0.5 + jitter(idx, 71, 0.1), j as f32 * 0.5 + jitter(idx, 72, 0.1));
            let e = 0.05;
            let n = Vec3::new(-(surface(x + e, y) - surface(x - e, y)) / (2.0 * e), -(surface(x, y + e) - surface(x, y - e)) / (2.0 * e), 1.0).normalize();
            let r = Vec3::new(x - site.x, y - site.y, 0.0).length();
            let lift = 0.4 * ((5.5 - r) / 0.5).clamp(0.0, 1.0);
            c.push(pt(Vec3::new(x, y, surface(x, y) + lift + jitter(idx, 73, 0.03)), n));
        }
    }
    back_face(&mut c, 40.0, 30.0);
    (noisy(c, 0.0, 6.0), site, surface)
}

#[test]
fn reconstruct_fills_a_marker_site_with_the_surface_curvature() {
    // Reconstruct's path: prepare_points (cut and fill from the scan points), the implicit surface,
    // the app's smoothing, then the fairing onto the fill surfaces. With the band slopes continued
    // (not Reconstruct's default) the site follows the ridge (a ring quadric alone is ~0.04 mm off
    // at the centre); with the default (ring fit only, see `PrepareConfig`) it is the quadric, as
    // before the band slopes existed, and the limits hold it there. Either way no ring at the cut.
    for slopes in [true, false] {
        reconstruct_site_check(slopes);
    }
}

#[test]
fn reconstruct_fills_a_marker_site_on_a_small_ball_without_a_bump() {
    // Reconstruct's path with its default settings on a 3 mm wall whose front face is a ball of
    // R 20 mm round a sticker (0.5 mm pitch, 0.03 mm noise): a ring quadric misses the ball's centre
    // by r⁴/8R³ (+0.06 mm, the size of a visible flat spot); the conic ring fit follows it.
    use crate::solid::markers::{Marker, MarkerSource};
    const R: f32 = 20.0;
    let ball = |x: f32, y: f32| -(R - (R * R - x * x - y * y).max(0.0).sqrt());
    let mut cloud = PointCloud::new();
    for (k, depth, outward) in [(0u32, 0.0f32, true), (1, 3.0, false)] {
        for i in -30i32..=30 {
            for j in -30i32..=30 {
                let idx = ((i + 30) * 61 + j + 30) as usize + 10_000 * k as usize;
                let (x, y) = (i as f32 * 0.5 + jitter(idx, 81 + k, 0.1), j as f32 * 0.5 + jitter(idx, 83 + k, 0.1));
                if x.hypot(y) > 0.75 * R {
                    continue;
                }
                let d = Vec3::new(x, y, ball(x, y) + R).normalize();
                let lift = if outward { 0.4 * ((5.5 - x.hypot(y)) / 0.5).clamp(0.0, 1.0) } else { 0.0 };
                let p = Vec3::new(0.0, 0.0, -R) + d * (R - depth) + Vec3::Z * (lift + jitter(idx, 85 + k, 0.03));
                cloud.push(pt(p, if outward { d } else { -d }));
            }
        }
    }
    let marker = Marker { id: "B".into(), center: Vec3::ZERO, normal: Vec3::Z, scan: Some(1), source: MarkerSource::List, radius_mm: 3.0, score: 0.0 };
    let cfg = crate::PrepareConfig::default();
    let (prepared, rep) = crate::prepare_points(&cloud, std::slice::from_ref(&marker), &cfg);
    assert_eq!(rep.markers_filled(), 1, "{:?}", rep.marker_outcomes);
    let (mut mesh, _) = crate::implicit::reconstruct(&prepared, &crate::SurfaceConfig::default());
    crate::FeatureSmoother { iterations: 4, sigma_normal: 0.5144, ..Default::default() }.smooth(&mut mesh);
    let fair = crate::solid::fair_marker_sites_on_fills(&mut mesh, &rep.marker_sites, &rep.marker_outcomes, &cfg.markers);
    assert_eq!(fair[0].status, "faired", "{fair:?}");
    let front: Vec<Vec3> = mesh.vertices.iter().map(|v| Vec3::from_array(v.position)).filter(|p| p.z > ball(p.x, p.y) - 1.5).collect();
    let off = |r0: f32, r1: f32| -> (f32, f32) {
        let e: Vec<f32> = front.iter().filter(|p| (r0..r1).contains(&p.x.hypot(p.y))).map(|p| p.z - ball(p.x, p.y)).collect();
        (e.iter().sum::<f32>() / e.len().max(1) as f32, e.iter().map(|v| v.abs()).fold(0f32, f32::max))
    };
    let ((centre, _), (_, inside), (around_mean, around)) = (off(0.0, 1.5), off(0.0, 6.0), off(8.0, 11.0));
    eprintln!("reconstruct on a ball R {R}: centre {centre:+.4} mm (around the cut {around_mean:+.4}), worst inside the cut {inside:.4} (around it {around:.4}); {:?}", rep.marker_outcomes[0].fill);
    assert!((centre - around_mean).abs() < 0.02, "the site stands {:+.3} mm off the ball against the mesh round it", centre - around_mean);
    assert!(inside < (around + 0.01).max(0.03), "site {inside} mm off the ball (around it {around})");
}

fn reconstruct_site_check(slopes: bool) {
    use crate::solid::markers::{Marker, MarkerSource};
    let (cloud, site, surface) = ridged_wall_with_sticker();
    let marker = Marker { id: "M1".into(), center: site, normal: Vec3::Z, scan: Some(1), source: MarkerSource::List, radius_mm: 3.0, score: 0.0 };
    let mut cfg = crate::PrepareConfig::default();
    cfg.markers.continue_band_slopes = slopes;
    let (prepared, rep) = crate::prepare_points(&cloud, std::slice::from_ref(&marker), &cfg);
    assert_eq!(rep.markers_filled(), 1, "{:?}", rep.marker_outcomes);
    let local = |p: Vec3| (p.x - site.x, p.y - site.y);
    let fill_worst = prepared
        .points
        .iter()
        .map(|p| Vec3::from_array(p.position))
        .filter(|p| p.z > -1.5 && local(*p).0.hypot(local(*p).1) < 6.0)
        .map(|p| (p.z - surface(p.x, p.y)).abs())
        .fold(0f32, f32::max);
    let (mut mesh, _) = crate::implicit::reconstruct(&prepared, &crate::SurfaceConfig::default());
    crate::FeatureSmoother { iterations: 4, sigma_normal: 0.5144, ..Default::default() }.smooth(&mut mesh);
    let fair = crate::solid::fair_marker_sites_on_fills(&mut mesh, &rep.marker_sites, &rep.marker_outcomes, &cfg.markers);
    assert_eq!(fair[0].status, "faired", "{fair:?}");
    // Front-face vertices by distance from the site: inside the cut, across it, further out.
    let front: Vec<(f32, Vec3)> = mesh
        .vertices
        .iter()
        .map(|v| Vec3::from_array(v.position))
        .filter(|p| p.z > -1.5)
        .map(|p| (local(p).0.hypot(local(p).1), p))
        .collect();
    let worst_in = |r0: f32, r1: f32| front.iter().filter(|(r, _)| (r0..r1).contains(r)).map(|(_, p)| (p.z - surface(p.x, p.y)).abs()).fold(0f32, f32::max);
    let (inside, around) = (worst_in(0.0, 6.0), worst_in(8.0, 12.0));
    // Slope continuity: the normal error of the front faces across the cut against further out.
    let normal_p99 = |r0: f32, r1: f32| {
        let mut a: Vec<f32> = mesh
            .triangles
            .iter()
            .filter_map(|t| {
                let [a, b, c] = t.indices.map(|i| Vec3::from_array(mesh.vertices[i as usize].position));
                let g = (a + b + c) / 3.0;
                let r = local(g).0.hypot(local(g).1);
                if !(r0..r1).contains(&r) || g.z < -1.5 {
                    return None;
                }
                let e = 0.01;
                let nt = Vec3::new(-(surface(g.x + e, g.y) - surface(g.x - e, g.y)) / (2.0 * e), -(surface(g.x, g.y + e) - surface(g.x, g.y - e)) / (2.0 * e), 1.0).normalize();
                Some((b - a).cross(c - a).normalize().angle_between(nt) * 1000.0)
            })
            .collect();
        a.sort_by(|x, y| x.total_cmp(y));
        a[(a.len() - 1) * 99 / 100]
    };
    let kink = normal_p99(5.0, 8.0) - normal_p99(9.0, 12.0);
    eprintln!("reconstruct (band slopes {slopes}): fill points worst {fill_worst:.4} mm; mesh inside the cut worst {inside:.4} (around it {around:.4}), normal error p99 at the cut {kink:.1} mrad over further out; {fair:?}");
    let (fill_tol, site_tol): (f32, f32) = if slopes { (0.03, 0.025) } else { (0.04, 0.032) };
    assert!(fill_worst < fill_tol, "fill points {fill_worst} mm off the surface");
    assert!(inside < site_tol.max(around + 0.005), "site {inside} mm off the surface (around it {around})");
    // (Against the true ridge: the ring fit alone does not follow it, and its faces round the cut
    // are a few mrad off the ridge's even without a step there.)
    let kink_max = if slopes { 4.0 } else { 8.5 };
    assert!(kink < kink_max, "normals {kink} mrad off at the cut");
}
