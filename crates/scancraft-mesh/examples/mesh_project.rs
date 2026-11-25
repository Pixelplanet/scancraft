//! Meshes a saved ScanCraft project the way the app does and reports how good the surface is.
//!
//! ```text
//! cargo run --release -p scancraft-mesh --example mesh_project -- <project_dir> <out_dir> [options]
//!   --cell MM        marching-cubes cell (default 0.7, the app's default)
//!   --smooth N       smoothing iterations after reconstruction (default 4, the app's default)
//!   --sigma S        rimls: smoother edge-keeping scale (default 0.5144, the app's default:
//!                    Keep edges 0.7)
//!   --pitch MM       merge pitch (default 0.5, the app's default for imported scans)
//!   --method NAME    "legacy" (the app's reconstruction) or "rimls" (default)
//!   --detail X       rimls: support radius in point spacings (default 3.0)
//! ```
//!
//! The merged model is rebuilt from each pass's cleaned (or imported) surface and its transform
//! in `project.json`, then merged with `scancraft_align::merge` (the app's merge). Outputs in
//! `<out_dir>`: `mesh_<tag>.ply` and shaded views `<tag>_view{1,2,3}.png` along the principal
//! axes plus `<tag>_zoom.png`, a close-up. Printed: point spacing, triangles, open edges and
//! boundary loops, roughness (mean angle between neighbouring face normals: lower is smoother)
//! and fidelity (distance from vertices to the nearest merged point).

use std::path::{Path, PathBuf};
use std::time::Instant;

use glam::{Mat3, Quat, Vec3};
use rayon::prelude::*;
use scancraft_align::{merge, MergeConfig, MergeInput};
use scancraft_core::{Mesh, Point3D, PointCloud, Transform};
use scancraft_mesh::{MarchingCubes, Mesher};

struct Args {
    project: PathBuf,
    out: PathBuf,
    cell: f32,
    smooth: usize,
    pitch: f32,
    method: String,
    detail: f32,
}

fn args() -> Args {
    let a: Vec<String> = std::env::args().skip(1).collect();
    let value = |flag: &str, default: &str| a.iter().position(|x| x == flag).and_then(|i| a.get(i + 1)).cloned().unwrap_or_else(|| default.to_string());
    Args {
        project: PathBuf::from(&a[0]),
        out: PathBuf::from(&a[1]),
        cell: value("--cell", "0.7").parse().unwrap(),
        smooth: value("--smooth", "4").parse().unwrap(),
        pitch: value("--pitch", "0.5").parse().unwrap(),
        method: value("--method", "rimls"),
        detail: value("--detail", "3.0").parse().unwrap(),
    }
}

/// Each pass's surface in the project frame, with observation counts.
fn load_passes(dir: &Path) -> Vec<(String, Vec<(Vec3, Vec3, u32)>, f32)> {
    let json: serde_json::Value = serde_json::from_str(&std::fs::read_to_string(dir.join("project.json")).unwrap()).unwrap();
    let mut out = Vec::new();
    for p in json["passes"].as_array().unwrap() {
        if p["included"].as_bool() == Some(false) || p["alignment"]["status"].as_str() == Some("unaligned") {
            continue;
        }
        let id = p["id"].as_u64().unwrap();
        let t: Transform = serde_json::from_value(p["transform"].clone()).unwrap();
        let cfg = &p["scan_config"];
        let file = cfg["cleaned"].as_str().or(cfg["origin"]["surface"].as_str()).expect("only cleaned or imported passes are supported");
        let (cloud, _) = scancraft_io::load_ply(dir.join(file)).unwrap();
        let counts = std::fs::read(dir.join(format!("pass-{id}/cleaned_observations.bin"))).ok().filter(|b| cfg["cleaned"].is_string() && b.len() == cloud.len() * 4);
        let m = t.to_mat4();
        let points = cloud
            .points
            .iter()
            .enumerate()
            .map(|(i, q)| {
                let c = counts.as_ref().map_or(2, |b| u32::from_le_bytes([b[4 * i], b[4 * i + 1], b[4 * i + 2], b[4 * i + 3]]));
                (m.transform_point3(Vec3::from(q.position)), (t.rotation * Vec3::from(q.normal)).normalize_or_zero(), c)
            })
            .collect();
        out.push((p["name"].as_str().unwrap_or("?").to_string(), points, p["weight"].as_f64().unwrap_or(1.0) as f32));
    }
    out
}

/// Median nearest-neighbour distance over a sample (brute force on a hash grid).
fn spacing(points: &[Vec3]) -> f32 {
    let grid = Grid::new(points, 2.0);
    let step = (points.len() / 3000).max(1);
    let mut d: Vec<f32> = (0..points.len()).step_by(step).map(|i| grid.nearest_other(points, i, 2.0).unwrap_or(2.0)).collect();
    d.sort_by(|a, b| a.total_cmp(b));
    d[d.len() / 2]
}

/// A uniform hash grid over points for radius queries.
struct Grid {
    cell: f32,
    map: std::collections::HashMap<[i32; 3], Vec<u32>>,
}

impl Grid {
    fn new(points: &[Vec3], cell: f32) -> Self {
        let mut map: std::collections::HashMap<[i32; 3], Vec<u32>> = std::collections::HashMap::new();
        for (i, p) in points.iter().enumerate() {
            map.entry(Self::key(*p, cell)).or_default().push(i as u32);
        }
        Grid { cell, map }
    }
    fn key(p: Vec3, cell: f32) -> [i32; 3] {
        (p / cell).floor().as_ivec3().to_array()
    }
    fn nearest(&self, points: &[Vec3], q: Vec3, max: f32) -> Option<f32> {
        let k = Self::key(q, self.cell);
        let r = (max / self.cell).ceil() as i32;
        let mut best = max * max;
        let mut found = false;
        for dz in -r..=r {
            for dy in -r..=r {
                for dx in -r..=r {
                    if let Some(v) = self.map.get(&[k[0] + dx, k[1] + dy, k[2] + dz]) {
                        for &j in v {
                            let d = points[j as usize].distance_squared(q);
                            if d < best {
                                best = d;
                                found = true;
                            }
                        }
                    }
                }
            }
        }
        found.then(|| best.sqrt())
    }
    fn nearest_other(&self, points: &[Vec3], i: usize, max: f32) -> Option<f32> {
        let q = points[i];
        let k = Self::key(q, self.cell);
        let mut best = max * max;
        let mut found = false;
        for dz in -1..=1 {
            for dy in -1..=1 {
                for dx in -1..=1 {
                    if let Some(v) = self.map.get(&[k[0] + dx, k[1] + dy, k[2] + dz]) {
                        for &j in v {
                            let d = points[j as usize].distance_squared(q);
                            if j as usize != i && d > 1e-12 && d < best {
                                best = d;
                                found = true;
                            }
                        }
                    }
                }
            }
        }
        found.then(|| best.sqrt())
    }
}

/// Mean and 90th percentile of the angle (degrees) between the normals of triangles sharing an
/// edge, and the number of boundary loops.
fn roughness(mesh: &Mesh) -> (f32, f32, usize) {
    let pos = |i: u32| Vec3::from(mesh.vertices[i as usize].position);
    let normals: Vec<Vec3> = mesh.triangles.par_iter().map(|t| (pos(t.indices[1]) - pos(t.indices[0])).cross(pos(t.indices[2]) - pos(t.indices[0])).normalize_or_zero()).collect();
    let mut edges: Vec<(u64, u32)> = mesh
        .triangles
        .iter()
        .enumerate()
        .flat_map(|(ti, t)| {
            let [a, b, c] = t.indices;
            [(a, b), (b, c), (c, a)].into_iter().map(move |(u, v)| ((((u.min(v)) as u64) << 32) | u.max(v) as u64, ti as u32))
        })
        .collect();
    edges.par_sort_unstable();
    let mut angles = Vec::new();
    let mut boundary: Vec<(u32, u32)> = Vec::new();
    let mut s = 0;
    while s < edges.len() {
        let mut e = s + 1;
        while e < edges.len() && edges[e].0 == edges[s].0 {
            e += 1;
        }
        if e - s == 2 {
            let (a, b) = (normals[edges[s].1 as usize], normals[edges[s + 1].1 as usize]);
            angles.push(a.dot(b).clamp(-1.0, 1.0).acos().to_degrees());
        } else if e - s == 1 {
            boundary.push(((edges[s].0 >> 32) as u32, edges[s].0 as u32));
        }
        s = e;
    }
    // Boundary loops = connected components of the boundary edge graph.
    let mut parent: std::collections::HashMap<u32, u32> = std::collections::HashMap::new();
    fn find(p: &mut std::collections::HashMap<u32, u32>, x: u32) -> u32 {
        let mut r = x;
        while let Some(&q) = p.get(&r) {
            if q == r {
                break;
            }
            r = q;
        }
        p.insert(x, r);
        r
    }
    for &(a, b) in &boundary {
        parent.entry(a).or_insert(a);
        parent.entry(b).or_insert(b);
        let (ra, rb) = (find(&mut parent, a), find(&mut parent, b));
        if ra != rb {
            parent.insert(ra, rb);
        }
    }
    let keys: Vec<u32> = parent.keys().copied().collect();
    let loops: std::collections::HashSet<u32> = keys.into_iter().map(|k| find(&mut parent, k)).collect();
    angles.sort_by(|a, b| a.total_cmp(b));
    let mean = angles.iter().sum::<f32>() / angles.len().max(1) as f32;
    (mean, angles.get(angles.len() * 9 / 10).copied().unwrap_or(0.0), loops.len())
}

/// Shaded orthographic view of the mesh looking along `-axes[2]`, `axes[0]` to the right.
fn render(mesh: &Mesh, centre: Vec3, axes: [Vec3; 3], half: f32, size: u32) -> image::RgbImage {
    let mut img = image::RgbImage::from_pixel(size, size, image::Rgb([24, 26, 30]));
    let mut depth = vec![f32::MIN; (size * size) as usize];
    let s = size as f32 / (2.0 * half);
    let project = |p: Vec3| {
        let d = p - centre;
        Vec3::new((d.dot(axes[0]) * s) + size as f32 / 2.0, size as f32 / 2.0 - d.dot(axes[1]) * s, d.dot(axes[2]))
    };
    let light = (axes[2] * 0.8 + axes[1] * 0.45 + axes[0] * 0.3).normalize();
    for t in &mesh.triangles {
        let p: Vec<Vec3> = t.indices.iter().map(|&i| Vec3::from(mesh.vertices[i as usize].position)).collect();
        let n = (p[1] - p[0]).cross(p[2] - p[0]).normalize_or_zero();
        // Both sides lit (inside surfaces of shells are seen from behind).
        let shade = n.dot(light).abs();
        let c = (40.0 + 200.0 * shade).min(255.0) as u8;
        let q: Vec<Vec3> = p.iter().map(|&x| project(x)).collect();
        let (minx, maxx) = (q.iter().map(|v| v.x).fold(f32::MAX, f32::min).floor().max(0.0) as i32, q.iter().map(|v| v.x).fold(f32::MIN, f32::max).ceil().min(size as f32 - 1.0) as i32);
        let (miny, maxy) = (q.iter().map(|v| v.y).fold(f32::MAX, f32::min).floor().max(0.0) as i32, q.iter().map(|v| v.y).fold(f32::MIN, f32::max).ceil().min(size as f32 - 1.0) as i32);
        let area = (q[1].x - q[0].x) * (q[2].y - q[0].y) - (q[2].x - q[0].x) * (q[1].y - q[0].y);
        if area.abs() < 1e-9 {
            continue;
        }
        for y in miny..=maxy {
            for x in minx..=maxx {
                let (px, py) = (x as f32 + 0.5, y as f32 + 0.5);
                let w0 = ((q[1].x - px) * (q[2].y - py) - (q[2].x - px) * (q[1].y - py)) / area;
                let w1 = ((q[2].x - px) * (q[0].y - py) - (q[0].x - px) * (q[2].y - py)) / area;
                let w2 = 1.0 - w0 - w1;
                if w0 < 0.0 || w1 < 0.0 || w2 < 0.0 {
                    continue;
                }
                let z = w0 * q[0].z + w1 * q[1].z + w2 * q[2].z;
                let i = (y as u32 * size + x as u32) as usize;
                if z > depth[i] {
                    depth[i] = z;
                    img.put_pixel(x as u32, y as u32, image::Rgb([c, c, (c as f32 * 1.04).min(255.0) as u8]));
                }
            }
        }
    }
    img
}

fn principal_axes(points: &[Vec3]) -> (Vec3, [Vec3; 3]) {
    let c = points.iter().copied().sum::<Vec3>() / points.len() as f32;
    let mut cov = Mat3::ZERO;
    for p in points.iter().step_by((points.len() / 50_000).max(1)) {
        let d = *p - c;
        cov += Mat3::from_cols(d * d.x, d * d.y, d * d.z);
    }
    // Power iteration for the two largest axes.
    let mut a = Vec3::X;
    for _ in 0..100 {
        a = (cov * a).normalize();
    }
    let mut b = Vec3::Y - a * a.y;
    for _ in 0..100 {
        b = cov * b;
        b = (b - a * b.dot(a)).normalize();
    }
    (c, [a, b, a.cross(b)])
}

fn write_ply(mesh: &Mesh, path: &Path) {
    scancraft_io::save_mesh_ply_binary(mesh, path).unwrap();
}

fn main() {
    let a = args();
    std::fs::create_dir_all(&a.out).unwrap();
    let t0 = Instant::now();
    let passes = load_passes(&a.project);
    for (name, pts, w) in &passes {
        println!("pass {name}: {} points, weight {w}", pts.len());
    }
    let inputs: Vec<MergeInput> = passes.iter().map(|(_, p, w)| MergeInput { points: p.clone(), weight: *w }).collect();
    let config = MergeConfig { pitch_mm: a.pitch, outliers: Some((12, 3.0)), ..MergeConfig::default() };
    let merged = merge(&inputs, &config);
    let cloud = PointCloud::from_points(merged.points.clone());
    let positions: Vec<Vec3> = cloud.points.iter().map(|p| Vec3::from(p.position)).collect();
    let sp = spacing(&positions);
    println!("merged: {} points, spacing {:.3} mm ({:.1} s)", cloud.len(), sp, t0.elapsed().as_secs_f32());
    if std::env::args().any(|a| a == "--dump") {
        let m = Mesh::from_geometry(cloud.points.clone(), Vec::new());
        write_ply(&m, &a.out.join("merged_normals.ply"));
        println!("wrote merged_normals.ply");
        return;
    }

    let tag = format!("{}_c{}_s{}_d{}", a.method, a.cell, a.smooth, a.detail);
    let t = Instant::now();
    let mut mesh = match a.method.as_str() {
        "legacy" => MarchingCubes::new(a.cell).reconstruct(&cloud),
        _ => {
            let flag = |f: &str| std::env::args().any(|x| x == f);
            let cfg = scancraft_mesh::SurfaceConfig {
                cell_mm: a.cell,
                detail: a.detail,
                refit_normals: !flag("--no-refit"),
                normal_passes: if flag("--no-nsmooth") { 0 } else { 1 },
                robust_iterations: if flag("--no-robust") { 0 } else { 3 },
                outlier_sigma: std::env::args().skip_while(|x| x != "--outliers").nth(1).map(|v| v.parse().unwrap()).unwrap_or(4.0),
                ..Default::default()
            };
            let (m, st) = Mesher::reconstruct_with_stats(&cloud, &cfg);
            println!("{st:?}");
            m
        }
    };
    let t_rec = t.elapsed().as_secs_f32();
    let t = Instant::now();
    if a.smooth > 0 {
        if a.method == "legacy" {
            Mesher::smooth(&mut mesh, a.smooth, 0.35);
        } else {
            let sigma = std::env::args().skip_while(|x| x != "--sigma").nth(1).map(|v| v.parse().unwrap()).unwrap_or(0.5144);
            scancraft_mesh::FeatureSmoother { iterations: a.smooth, sigma_normal: sigma, ..Default::default() }.smooth(&mut mesh);
        }
    }
    let t_smooth = t.elapsed().as_secs_f32();
    // --fill MM: fill holes up to MM around (the largest opening stays open).
    let tag = match std::env::args().skip_while(|x| x != "--fill").nth(1).map(|v| v.parse::<f32>().unwrap()) {
        Some(max) => {
            let t = Instant::now();
            let before = scancraft_mesh::HoleFiller::find_holes(&mesh).len();
            let r = scancraft_mesh::HoleFiller { max_perimeter_mm: max, keep_largest: true }.fill(&mut mesh);
            mesh.recompute_normals();
            println!("fill <= {max} mm: {before} holes, {r:?} in {:.2} s", t.elapsed().as_secs_f32());
            format!("{tag}_fill{max}")
        }
        None => tag,
    };
    let an = Mesher::analyze(&mesh);
    let (mean, p90, loops) = roughness(&mesh);
    let grid = Grid::new(&positions, 1.0);
    let mut dist: Vec<f32> = mesh.vertices.par_iter().step_by(7).map(|v| grid.nearest(&positions, Vec3::from(v.position), 3.0).unwrap_or(3.0)).collect();
    dist.sort_by(|a, b| a.total_cmp(b));
    println!(
        "{tag}: {} tris, {} open edges, {} boundary loops | roughness mean {:.2}°, p90 {:.2}° | vertex->points median {:.3} mm, p95 {:.3} mm | reconstruct {:.1} s, smooth {:.1} s",
        an.triangle_count,
        an.open_edges,
        loops,
        mean,
        p90,
        dist[dist.len() / 2],
        dist[dist.len() * 95 / 100],
        t_rec,
        t_smooth
    );
    write_ply(&mesh, &a.out.join(format!("mesh_{tag}.ply")));
    let (c, axes) = principal_axes(&positions);
    let ext = positions.iter().map(|p| (*p - c).dot(axes[0]).abs()).fold(0.0f32, f32::max) * 1.05;
    let views = [[axes[0], axes[1], axes[2]], [axes[0], axes[2], -axes[1]], [axes[1], axes[2], axes[0]]];
    for (k, v) in views.iter().enumerate() {
        render(&mesh, c, *v, ext, 1400).save(a.out.join(format!("{tag}_view{}.png", k + 1))).unwrap();
    }
    // A close-up of the area around the point nearest the centre along the view direction.
    // --zoom U,V: centre of the close-up in view 1, as fractions of the half-width (right, up).
    let zoom = std::env::args().skip_while(|x| x != "--zoom").nth(1).map(|z| z.split(',').map(|v| v.parse::<f32>().unwrap()).collect::<Vec<_>>()).unwrap_or(vec![0.3, 0.0]);
    render(&mesh, c + axes[0] * ext * zoom[0] + axes[1] * ext * zoom[1], views[0], ext * 0.15, 1000).save(a.out.join(format!("{tag}_zoom.png"))).unwrap();
    let _ = (Quat::IDENTITY, Point3D::default());
}
