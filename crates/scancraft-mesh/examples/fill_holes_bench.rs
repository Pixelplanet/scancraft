//! Timing check for hole filling on a scan-sized mesh: an icosphere (subdivision 8, ~1.3 M
//! triangles, ~0.4 mm edges at r = 100 mm) with ~200 random small holes and two big ones.
//!
//! cargo run --release -p scancraft-mesh --example fill_holes_bench -- [subdivisions] [radius_mm]

use glam::Vec3;
use scancraft_core::{Mesh, Point3D, Triangle};
use scancraft_mesh::{HoleFiller, MeshMetrics};
use std::collections::HashMap;
use std::time::Instant;

fn icosphere(r: f32, subdivisions: u32) -> Mesh {
    let t = (1.0 + 5f32.sqrt()) / 2.0;
    let mut v: Vec<Vec3> = [
        [-1.0, t, 0.0],
        [1.0, t, 0.0],
        [-1.0, -t, 0.0],
        [1.0, -t, 0.0],
        [0.0, -1.0, t],
        [0.0, 1.0, t],
        [0.0, -1.0, -t],
        [0.0, 1.0, -t],
        [t, 0.0, -1.0],
        [t, 0.0, 1.0],
        [-t, 0.0, -1.0],
        [-t, 0.0, 1.0],
    ]
    .iter()
    .map(|p| Vec3::from_array(*p).normalize())
    .collect();
    let mut f: Vec<[u32; 3]> = vec![
        [0, 11, 5],
        [0, 5, 1],
        [0, 1, 7],
        [0, 7, 10],
        [0, 10, 11],
        [1, 5, 9],
        [5, 11, 4],
        [11, 10, 2],
        [10, 7, 6],
        [7, 1, 8],
        [3, 9, 4],
        [3, 4, 2],
        [3, 2, 6],
        [3, 6, 8],
        [3, 8, 9],
        [4, 9, 5],
        [2, 4, 11],
        [6, 2, 10],
        [8, 6, 7],
        [9, 8, 1],
    ];
    for _ in 0..subdivisions {
        let mut mid: HashMap<u64, u32> = HashMap::with_capacity(f.len() * 2);
        let mut nf = Vec::with_capacity(f.len() * 4);
        let mut m = |a: u32, b: u32, v: &mut Vec<Vec3>| {
            let key = ((a.min(b) as u64) << 32) | a.max(b) as u64;
            *mid.entry(key).or_insert_with(|| {
                v.push(((v[a as usize] + v[b as usize]) * 0.5).normalize());
                v.len() as u32 - 1
            })
        };
        for [a, b, c] in f {
            let ab = m(a, b, &mut v);
            let bc = m(b, c, &mut v);
            let ca = m(c, a, &mut v);
            nf.extend([[a, ab, ca], [b, bc, ab], [c, ca, bc], [ab, bc, ca]]);
        }
        f = nf;
    }
    Mesh::from_geometry(
        v.iter().map(|p| Point3D::new((*p * r).to_array())).collect(),
        f.iter().map(|t| Triangle::new(t[0], t[1], t[2])).collect(),
    )
}

/// xorshift64*, so the bench needs no random-number dependency and is reproducible.
struct Rng(u64);

impl Rng {
    fn next(&mut self) -> f32 {
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        ((self.0.wrapping_mul(0x2545_f491_4f6c_dd1d) >> 40) as f32) / (1u64 << 24) as f32
    }

    fn direction(&mut self) -> Vec3 {
        loop {
            let d = Vec3::new(self.next(), self.next(), self.next()) * 2.0 - 1.0;
            let l = d.length();
            if l > 0.1 && l <= 1.0 {
                return d / l;
            }
        }
    }
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let subdivisions: u32 = args.first().map(|s| s.parse().unwrap()).unwrap_or(8);
    let r: f32 = args.get(1).map(|s| s.parse().unwrap()).unwrap_or(100.0);

    let t = Instant::now();
    let mut mesh = icosphere(r, subdivisions);
    if MeshMetrics::analyze(&mesh).signed_volume_mm3 < 0.0 {
        for t in &mut mesh.triangles {
            t.indices.swap(1, 2);
        }
    }
    mesh.recompute_normals();
    println!("sphere: {} triangles, r = {r} mm, built in {:.2?}", mesh.triangles.len(), t.elapsed());

    // 200 small holes (radius 1–4 mm) and two big ones (20 and 35 mm).
    let mut rng = Rng(0x9e37_79b9_7f4a_7c15);
    let mut cuts: Vec<(Vec3, f32)> = (0..200).map(|_| (rng.direction() * r, 1.0 + 3.0 * rng.next())).collect();
    cuts.push((Vec3::Z * r, 20.0));
    cuts.push((-Vec3::Z * r, 35.0));
    let t = Instant::now();
    let v = mesh.vertices.clone();
    mesh.triangles.retain(|tri| {
        let c = tri.indices.iter().map(|&i| Vec3::from_array(v[i as usize].position)).sum::<Vec3>() / 3.0;
        cuts.iter().all(|(p, rad)| c.distance(*p) > *rad)
    });
    println!("cut: {} triangles left in {:.2?}", mesh.triangles.len(), t.elapsed());

    let t = Instant::now();
    let holes = HoleFiller::find_holes(&mesh);
    let tf = t.elapsed();
    let big: Vec<String> =
        holes.iter().take(3).map(|h| format!("{} edges / {:.0} mm", h.edge_count, h.perimeter_mm)).collect();
    println!("find_holes: {} holes in {tf:.2?} (largest: {})", holes.len(), big.join(", "));

    let t = Instant::now();
    let a = MeshMetrics::analyze(&mesh);
    println!("analyze before: {} open edges ({:.2?})", a.open_edges, t.elapsed());

    let nverts = mesh.vertices.len();
    let t = Instant::now();
    let rep = HoleFiller::default().fill(&mut mesh);
    let tfill = t.elapsed();
    println!("fill: {rep:?} in {tfill:.2?}");

    let t = Instant::now();
    let a = MeshMetrics::analyze(&mesh);
    println!(
        "analyze after: watertight {}, open {}, non-manifold {}, inconsistent {}, volume error {:.3} % ({:.2?})",
        a.is_watertight,
        a.open_edges,
        a.non_manifold_edges,
        a.inconsistent_edges,
        100.0 * (a.volume_mm3 / (4.0 / 3.0 * std::f32::consts::PI * r.powi(3)) - 1.0),
        t.elapsed()
    );
    let worst = mesh.vertices[nverts..]
        .iter()
        .map(|p| (Vec3::from_array(p.position).length() - r).abs())
        .fold(0.0, f32::max);
    println!("new vertices: worst distance from the sphere {worst:.3} mm");

    // The same holes, filled only up to 60 mm perimeter (the two big ones stay open).
    let mut m2 = icosphere(r, subdivisions);
    if MeshMetrics::analyze(&m2).signed_volume_mm3 < 0.0 {
        for t in &mut m2.triangles {
            t.indices.swap(1, 2);
        }
    }
    let v = m2.vertices.clone();
    m2.triangles.retain(|tri| {
        let c = tri.indices.iter().map(|&i| Vec3::from_array(v[i as usize].position)).sum::<Vec3>() / 3.0;
        cuts.iter().all(|(p, rad)| c.distance(*p) > *rad)
    });
    let t = Instant::now();
    let rep = HoleFiller::new(60.0, false).fill(&mut m2);
    println!("fill small only (<= 60 mm): {rep:?} in {:.2?}", t.elapsed());
}
