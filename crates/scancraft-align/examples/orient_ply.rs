//! Auto-orientation report on a PLY cloud: prints everything [`scancraft_align::auto_orient`]
//! decided plus [`scancraft_align::rim_report`]'s rim diagnostics, for checking a part the
//! orientation did (not) stand on its rim.
//!
//! ```text
//! orient_ply <a.ply> [<b.ply> ...]
//! ```
//!
//! Normals come from the file when it has usable ones; when they are missing or nearly all the
//! same (a `merged.ply` written without normals loads as all +Z) they are estimated
//! (`scancraft_io::normals::estimate_normals`, default settings) and that is printed per file.

use std::path::Path;

use glam::Vec3;
use scancraft_align::{auto_orient, rim_report, Oriented, OrientConfig};

fn load(path: &Path) -> (Vec<Oriented>, &'static str) {
    let (cloud, _) = scancraft_io::load_ply(path).unwrap_or_else(|e| {
        eprintln!("cannot read {}: {e}", path.display());
        std::process::exit(1)
    });
    // A normals-free file loads as all +Z; near-uniform normals mean they are not real.
    let degenerate = cloud.points.iter().filter(|p| p.normal == [0.0, 0.0, 1.0] || p.normal == [0.0; 3]).count() * 20 > cloud.points.len() * 19;
    if degenerate {
        let positions: Vec<[f32; 3]> = cloud.points.iter().map(|p| p.position).collect();
        let normals = scancraft_io::normals::estimate_normals(&positions, &scancraft_io::NormalEstimation::default());
        let points = cloud
            .points
            .iter()
            .zip(normals)
            .map(|(p, n)| (Vec3::from_array(p.position), Vec3::from_array(n).normalize_or_zero()))
            .filter(|(p, _)| p.is_finite())
            .collect();
        (points, "estimated")
    } else {
        let points = cloud.points.iter().map(|p| (Vec3::from_array(p.position), Vec3::from_array(p.normal).normalize_or_zero())).filter(|(p, _)| p.is_finite()).collect();
        (points, "file")
    }
}

fn report(path: &Path) {
    let (points, normals) = load(path);
    println!("{}: {} points, normals {normals}", path.display(), points.len());
    let config = OrientConfig::default();
    let Some(result) = auto_orient(&points, &config) else {
        println!("  auto_orient: None (too few points or samples)");
        return;
    };
    println!("  up: {} from {}, turned {:.1} deg", result.up, result.up_source.label(), result.turned_deg);
    println!("  faces: {}", result.faces.len());
    for f in &result.faces {
        println!("    {} share {:.1} %", f.dir, f.share * 100.0);
    }
    println!("  axis: {}", result.axis.map(|a| format!("{a}")).unwrap_or_else(|| "—".to_string()));
    match &result.rim {
        Some(r) => println!("  rim: {} points, rms {:.2} mm, share {:.1} %, plane n {} d {:.2}", r.points, r.rms_mm, r.share * 100.0, r.plane.normal, r.plane.d),
        None => println!("  rim: —"),
    }
    if let Some(r) = rim_report(&points, &config) {
        println!(
            "  rim search: {} samples (voxel {:.2} mm), {} boundary, tol {:.2} mm; best plane {} inliers ({:.1} % of boundary, rms {:.2} mm)",
            r.samples,
            r.voxel_mm,
            r.boundary,
            r.tolerance_mm,
            r.inliers,
            r.share * 100.0,
            r.rms_mm
        );
        println!("    sides beyond tol: {} below / {} above of {} samples; in-plane spread {:.1} x {:.1} mm vs extent {:.1} mm", r.sides.0, r.sides.1, r.samples, r.spread_mm[0], r.spread_mm[1], r.extent_mm);
        match r.rejected_by {
            Some(w) => println!("    rejected by: {w}"),
            None => println!("    accepted"),
        }
    }
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    if args.is_empty() {
        eprintln!("usage: orient_ply <a.ply> [<b.ply> ...]");
        std::process::exit(2);
    }
    for a in &args {
        report(Path::new(a));
    }
}
