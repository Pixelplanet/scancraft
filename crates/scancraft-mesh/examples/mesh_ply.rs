//! Manual check: mesh an ASCII PLY point cloud (x y z [nx ny nz], millimetres) and report.
//!
//! cargo run --release -p scancraft-mesh --example mesh_ply -- <in.ply> [cell_mm] [out.ply]

use scancraft_core::{Mesh, Point3D, PointCloud};
use scancraft_mesh::{MarchingCubes, Mesher};
use std::io::{BufRead, BufReader, BufWriter, Write};
use std::time::Instant;

fn read_ascii_ply(path: &str) -> PointCloud {
    let f = BufReader::new(std::fs::File::open(path).expect("open input"));
    let mut lines = f.lines();
    let mut props = Vec::new();
    let mut count = 0usize;
    for l in lines.by_ref() {
        let l = l.unwrap();
        let t: Vec<&str> = l.split_whitespace().collect();
        match t.as_slice() {
            ["format", fmt, ..] => assert_eq!(*fmt, "ascii", "only ASCII PLY is supported"),
            ["element", "vertex", n] => count = n.parse().unwrap(),
            ["property", _, name] => props.push(name.to_string()),
            ["end_header"] => break,
            _ => {}
        }
    }
    let col = |n: &str| props.iter().position(|p| p == n);
    let (x, y, z) = (col("x").unwrap(), col("y").unwrap(), col("z").unwrap());
    let normals = (col("nx"), col("ny"), col("nz"));
    let mut cloud = PointCloud::new();
    for l in lines.take(count) {
        let v: Vec<f32> = l.unwrap().split_whitespace().map(|s| s.parse().unwrap_or(0.0)).collect();
        let mut p = Point3D::new([v[x], v[y], v[z]]);
        p.normal = match normals {
            (Some(a), Some(b), Some(c)) => [v[a], v[b], v[c]],
            _ => [0.0; 3],
        };
        cloud.push(p);
    }
    cloud.recompute_bbox();
    cloud
}

fn write_ply(mesh: &Mesh, path: &str) {
    let mut w = BufWriter::new(std::fs::File::create(path).expect("create output"));
    write!(
        w,
        "ply\nformat binary_little_endian 1.0\nelement vertex {}\nproperty float x\nproperty float y\nproperty float z\n\
         property float nx\nproperty float ny\nproperty float nz\nelement face {}\nproperty list uchar int vertex_indices\nend_header\n",
        mesh.vertices.len(),
        mesh.triangles.len()
    )
    .unwrap();
    for v in &mesh.vertices {
        for c in v.position.iter().chain(&v.normal) {
            w.write_all(&c.to_le_bytes()).unwrap();
        }
    }
    for t in &mesh.triangles {
        w.write_all(&[3u8]).unwrap();
        for i in t.indices {
            w.write_all(&(i as i32).to_le_bytes()).unwrap();
        }
    }
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let input = args.get(1).expect("usage: mesh_ply <in.ply> [cell_mm] [out.ply]");
    let cell: f32 = args.get(2).map(|s| s.parse().unwrap()).unwrap_or(1.0);
    let out = args.get(3).cloned().unwrap_or_else(|| input.replace(".ply", &format!("_mesh_{cell}mm.ply")));

    let t = Instant::now();
    let cloud = read_ascii_ply(input);
    println!("read {} points in {:.2?}", cloud.len(), t.elapsed());

    let t = Instant::now();
    let (mesh, stats) = MarchingCubes::new(cell).reconstruct_with_stats(&cloud);
    println!("reconstruct @ {cell} mm: {:.2?}  {stats:?}", t.elapsed());
    println!("  {:?}", Mesher::analyze(&mesh));

    let t = Instant::now();
    let mut smoothed = mesh.clone();
    Mesher::smooth(&mut smoothed, 3, 0.5);
    println!("smooth x3: {:.2?}  {:?}", t.elapsed(), Mesher::analyze(&smoothed));

    let t = Instant::now();
    let dec = Mesher::decimate(&smoothed, 0.3);
    println!("decimate 0.3: {:.2?}  {:?}", t.elapsed(), Mesher::analyze(&dec));

    write_ply(&smoothed, &out);
    println!("wrote {out}");
}
