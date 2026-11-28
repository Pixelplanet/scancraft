//! Scores a mesh against a synthetic scene's truth.
//!
//! ```text
//! cargo run --release -p scancraft-synth --example synth_score -- <scene dir> <mesh.ply> [report.json]
//! ```
//!
//! The mesh must be in the part frame (pass 1's frame: export from a project whose reference is
//! pass 1). Prints the distance statistics and every failed probe; writes the full report as
//! JSON when a path is given.

use std::path::PathBuf;

use scancraft_synth::score::score;
use scancraft_synth::Truth;

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    if args.len() < 2 {
        eprintln!("usage: synth_score <scene dir> <mesh.ply> [report.json]");
        std::process::exit(2);
    }
    let truth = Truth::load(&PathBuf::from(&args[0])).expect("read truth.json");
    let (_, mesh) = scancraft_io::load_ply(&args[1]).expect("read the mesh");
    let mesh = mesh.expect("the PLY has no faces");
    let report = score(&truth, &mesh);
    let d = &report.distance;
    println!(
        "{}: {} vertices, {} triangles; |d| p50 {:.3} p95 {:.3} p99 {:.3} max {:.3} mm, signed mean {:+.3} mm, {:.1} % over 0.1 mm, {} extraneous",
        report.scene, report.vertices, report.triangles, d.p50_mm, d.p95_mm, d.p99_mm, d.max_mm, d.mean_signed_mm, 100.0 * d.over_0_1mm, d.extraneous
    );
    println!("probes passed: {} of {}", report.probes_passed, report.probes);
    for f in &report.features {
        let mark = if f.failed.is_empty() { "ok  " } else { "FAIL" };
        println!("  {mark} {:22} {:14} {}/{}", f.id, f.kind, f.passed, f.probes);
        for r in &f.failed {
            println!("         {:?} at {:?}: {:?} ({})", r.value, r.point, r.check, r.note);
        }
    }
    if let Some(path) = args.get(2) {
        std::fs::write(path, serde_json::to_string_pretty(&report).unwrap()).expect("write the report");
    }
}
