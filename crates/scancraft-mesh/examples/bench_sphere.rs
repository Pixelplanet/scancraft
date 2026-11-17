//! Timing check on a large synthetic cloud: a sphere with ~2 M oriented points.
//!
//! cargo run --release -p scancraft-mesh --example bench_sphere -- [radius_mm] [spacing_mm] [cell_mm...]

use glam::Vec3;
use scancraft_core::{Point3D, PointCloud};
use scancraft_mesh::{MarchingCubes, Mesher};
use std::time::Instant;

fn main() {
    let args: Vec<f32> = std::env::args().skip(1).map(|s| s.parse().unwrap()).collect();
    let r = args.first().copied().unwrap_or(100.0);
    let spacing = args.get(1).copied().unwrap_or(0.25);
    let cells = if args.len() > 2 { args[2..].to_vec() } else { vec![1.0, 0.5, 0.2] };

    let n = (4.0 * std::f32::consts::PI * r * r / (spacing * spacing)) as usize;
    let golden = std::f32::consts::PI * (3.0 - 5f32.sqrt());
    let mut cloud = PointCloud::new();
    for i in 0..n {
        let y = 1.0 - 2.0 * (i as f32 + 0.5) / n as f32;
        let rad = (1.0 - y * y).sqrt();
        let th = golden * i as f32;
        let d = Vec3::new(rad * th.cos(), y, rad * th.sin());
        let mut p = Point3D::new((d * r).to_array());
        p.normal = d.to_array();
        cloud.push(p);
    }
    cloud.recompute_bbox();
    println!("{} points, r = {r} mm", cloud.len());
    for h in cells {
        let t = Instant::now();
        let (mesh, stats) = MarchingCubes::new(h).reconstruct_with_stats(&cloud);
        let tr = t.elapsed();
        let t = Instant::now();
        let a = Mesher::analyze(&mesh);
        let ta = t.elapsed();
        let t = Instant::now();
        let mut m2 = mesh.clone();
        Mesher::smooth(&mut m2, 2, 0.5);
        let ts = t.elapsed();
        let t = Instant::now();
        let d = Mesher::decimate(&m2, 0.5);
        let td = t.elapsed();
        let ad = Mesher::analyze(&d);
        println!(
            "cell {h} mm: reconstruct {tr:.2?}, analyze {ta:.2?}, smooth {ts:.2?}, decimate {td:.2?} | {} tris, watertight {}, vol err {:.3}% | decimated watertight {}  ({stats:?})",
            a.triangle_count,
            a.is_watertight,
            100.0 * (a.volume_mm3 / (4.0 / 3.0 * std::f32::consts::PI * r.powi(3)) - 1.0),
            ad.is_watertight
        );
    }
}
