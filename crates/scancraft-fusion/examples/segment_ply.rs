//! Runs [`scancraft_fusion::segment_object`] on PLY scans and writes the object points.
//!
//! ```text
//! segment_ply <out_dir> <a.ply> [<b.ply> ...] [--up x,y,z] [--compare <dir>]
//!   --up x,y,z       up hint in the scans' frame (default: none)
//!   --compare <dir>  reference clouds <dir>/prep_S<i>.ply (for example align_ply --prep part
//!                    output): reports how many points both keep
//! ```
//!
//! Scans are numbered S1, S2, ... in argument order. `<out_dir>/S<i>_object.ply` gets the object
//! points of scan i (with normals and colours), `S<i>_labels.ply` all points coloured by label
//! (object green, support grey, other red).

use glam::Vec3;
use scancraft_fusion::{segment_cloud, SegmentConfig, SegmentLabel};
use scancraft_core::Point3D;
use std::collections::HashSet;
use std::io::Write;
use std::path::{Path, PathBuf};
use std::time::Instant;

fn usage() -> ! {
    eprintln!("usage: segment_ply <out_dir> <a.ply> [<b.ply> ...] [--up x,y,z] [--compare <dir>]");
    std::process::exit(2);
}

/// Exact position key, for comparing point sets taken from the same file.
fn key(p: [f32; 3]) -> [u32; 3] {
    p.map(f32::to_bits)
}

/// Binary PLY with x y z nx ny nz red green blue alpha; `colour` replaces the points' colours.
fn write_ply(path: &Path, points: &[Point3D], colour: impl Fn(usize) -> Option<[u8; 3]>) -> std::io::Result<()> {
    let mut w = std::io::BufWriter::new(std::fs::File::create(path)?);
    for line in ["ply", "format binary_little_endian 1.0", "comment ScanCraft segment_ply"] {
        writeln!(w, "{line}")?;
    }
    writeln!(w, "element vertex {}", points.len())?;
    for name in ["x", "y", "z", "nx", "ny", "nz"] {
        writeln!(w, "property float {name}")?;
    }
    for name in ["red", "green", "blue", "alpha"] {
        writeln!(w, "property uchar {name}")?;
    }
    writeln!(w, "end_header")?;
    for (i, p) in points.iter().enumerate() {
        for v in p.position.iter().chain(&p.normal) {
            w.write_all(&v.to_le_bytes())?;
        }
        let c = colour(i).unwrap_or([p.color.r, p.color.g, p.color.b]);
        w.write_all(&[c[0], c[1], c[2], 255])?;
    }
    w.flush()
}

fn main() {
    let mut positional = Vec::new();
    let (mut up, mut compare) = (None, None);
    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        match a.as_str() {
            "--up" => {
                let v: Vec<f32> = args.next().unwrap_or_default().split(',').filter_map(|s| s.trim().parse().ok()).collect();
                if v.len() != 3 {
                    usage();
                }
                up = Some(Vec3::new(v[0], v[1], v[2]));
            }
            "--compare" => compare = args.next().map(PathBuf::from),
            s if s.starts_with("--") => usage(),
            _ => positional.push(PathBuf::from(a)),
        }
    }
    if positional.len() < 2 {
        usage();
    }
    let out_dir = positional.remove(0);
    std::fs::create_dir_all(&out_dir).expect("create the output folder");
    let cfg = SegmentConfig::default();
    for (i, path) in positional.iter().enumerate() {
        let name = format!("S{}", i + 1);
        let (cloud, _) = scancraft_io::load_ply(path).unwrap_or_else(|e| {
            eprintln!("cannot read {}: {e}", path.display());
            std::process::exit(1)
        });
        let t0 = Instant::now();
        let seg = segment_cloud(&cloud, &cfg, None, up);
        let secs = t0.elapsed().as_secs_f32();
        let (object, support, other) = (seg.count(SegmentLabel::Object), seg.count(SegmentLabel::Support), seg.count(SegmentLabel::Other));
        println!("{name}: {} points: object {object}, support {support}, other {other} ({secs:.2} s)", cloud.len());
        match seg.support {
            Some(s) => println!("    support plane: normal {:.3?}, point {:.1?}", s.normal, s.point),
            None => println!("    no support plane"),
        }
        let points = seg.select(&cloud.points, SegmentLabel::Object);
        if let Some(dir) = &compare {
            let reference = dir.join(format!("prep_{name}.ply"));
            match scancraft_io::load_ply(&reference) {
                Ok((r, _)) => {
                    let theirs: HashSet<[u32; 3]> = r.points.iter().map(|p| key(p.position)).collect();
                    let both = points.iter().filter(|p| theirs.contains(&key(p.position))).count();
                    println!(
                        "    vs {}: reference {} points; both {both}, only ours {}, only reference {}",
                        reference.display(),
                        theirs.len(),
                        points.len() - both,
                        theirs.len().saturating_sub(both)
                    );
                }
                Err(e) => println!("    no reference {}: {e}", reference.display()),
            }
        }
        let out = out_dir.join(format!("{name}_object.ply"));
        let labels = out_dir.join(format!("{name}_labels.ply"));
        let by_label = |i: usize| {
            Some(match seg.labels[i] {
                SegmentLabel::Object => [60, 200, 80],
                SegmentLabel::Support => [150, 150, 150],
                SegmentLabel::Other => [220, 60, 50],
            })
        };
        for (file, result) in [(&out, write_ply(&out, &points, |_| None)), (&labels, write_ply(&labels, &cloud.points, by_label))] {
            match result {
                Ok(()) => println!("    wrote {}", file.display()),
                Err(e) => println!("    cannot write {}: {e}", file.display()),
            }
        }
    }
}
