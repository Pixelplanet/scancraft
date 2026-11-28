//! Runs ScanCraft's Reconstruct on a synthetic scene the way the app does with its default
//! settings, and scores the result against the truth.
//!
//! ```text
//! cargo run --release -p scancraft-synth --example synth_baseline -- <scene dir> [out dir]
//! ```
//!
//! The passes are placed with their true poses (`pass_to_part`), so alignment is taken out of the
//! test; the faults of each pass (offsets, strays, noise) stay. The listed markers come from each
//! pass's `marker_framework.mkf`. Steps as in the app's Reconstruct tool (`apps/scancraft/src/
//! app.rs`, `Tool::Reconstruct`): `prepare_points` (suppress artefacts, remove markers), the
//! implicit mesher (cell 0.7 mm, smoothness 3), the feature-preserving smoother (4 steps, keep
//! edges 0.7) and the marker-site fairing. Writes `reconstruct.ply` and `score.json` to the out
//! dir (default: the scene dir) and prints the score.

use std::path::PathBuf;
use std::time::Instant;

use glam::DVec3;
use scancraft_core::PointCloud;
use scancraft_mesh::solid::{Marker, MarkerSource};
use scancraft_mesh::{FeatureSmoother, Mesher, PrepareConfig, SurfaceConfig};
use scancraft_synth::score::score;
use scancraft_synth::Truth;

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let Some(scene_dir) = args.first().map(PathBuf::from) else {
        eprintln!("usage: synth_baseline <scene dir> [out dir]");
        std::process::exit(2);
    };
    let out = args.get(1).map(PathBuf::from).unwrap_or_else(|| scene_dir.clone());
    std::fs::create_dir_all(&out).expect("create the out dir");
    let truth = Truth::load(&scene_dir).expect("read truth.json");

    let mut points = Vec::new();
    let mut markers = Vec::new();
    for pass in &truth.passes {
        let (cloud, _) = scancraft_io::load_ply(scene_dir.join(&pass.cloud)).expect("read the pass");
        let t = pass.pass_to_part;
        for mut p in cloud.points {
            p.position = t.apply(DVec3::from_array(p.position.map(f64::from))).as_vec3().to_array();
            p.normal = t.apply_dir(DVec3::from_array(p.normal.map(f64::from))).as_vec3().to_array();
            points.push(p);
        }
        if let Some(file) = &pass.marker_file {
            for (k, m) in scancraft_io::read_marker_framework(&scene_dir.join(file)).expect("read the marker file").iter().enumerate() {
                markers.push(Marker {
                    id: format!("S{}-M{k}", pass.id),
                    center: t.apply(DVec3::from_array(m.centre)).as_vec3(),
                    normal: t.apply_dir(DVec3::from_array(m.normal)).as_vec3().normalize(),
                    scan: Some(pass.id as u8),
                    source: MarkerSource::List,
                    radius_mm: m.radius_mm,
                    score: 0.0,
                });
            }
        }
    }
    let cloud = PointCloud::from_points(points);
    let t0 = Instant::now();
    let prepare = PrepareConfig { suppress_artefacts: true, remove_markers: true, ..Default::default() };
    let (prepared, prep) = scancraft_mesh::prepare_points(&cloud, &markers, &prepare);
    let config = SurfaceConfig { cell_mm: 0.7, detail: 3.0, ..Default::default() };
    let (mut mesh, _) = Mesher::reconstruct_with_stats(&prepared, &config);
    FeatureSmoother { iterations: 4, sigma_normal: 1.2 + (0.22 - 1.2) * 0.7, ..Default::default() }.smooth(&mut mesh);
    if !prep.marker_sites.is_empty() {
        scancraft_mesh::solid::fair_marker_sites_on_fills(&mut mesh, &prep.marker_sites, &prep.marker_outcomes, &prepare.markers);
    }
    let seconds = t0.elapsed().as_secs_f64();
    scancraft_io::save_mesh_ply_binary(&mesh, out.join("reconstruct.ply")).expect("write the mesh");
    let report = score(&truth, &mesh);
    std::fs::write(out.join("score.json"), serde_json::to_string_pretty(&report).unwrap()).expect("write score.json");
    let d = &report.distance;
    println!(
        "{}: {} points, {} markers -> {} triangles in {seconds:.1} s; |d| p50 {:.3} p95 {:.3} p99 {:.3} mm, {:.1} % over 0.1 mm, {} extraneous; probes {}/{}",
        truth.scene,
        cloud.len(),
        markers.len(),
        report.triangles,
        d.p50_mm,
        d.p95_mm,
        d.p99_mm,
        100.0 * d.over_0_1mm,
        d.extraneous,
        report.probes_passed,
        report.probes
    );
    for f in &report.features {
        if !f.failed.is_empty() {
            let worst: Vec<String> = f.failed.iter().take(3).map(|r| format!("{} {:?}", r.note, r.value.map(|v| (v * 1000.0).round() / 1000.0))).collect();
            println!("  FAIL {:22} {:14} {}/{}  {}", f.id, f.kind, f.passed, f.probes, worst.join("; "));
        }
    }
}
