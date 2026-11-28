//! How much a point cloud looks like a real scan (see `src/realism.rs`): density, spacing, normal
//! error, noise structure and the point density round each listed marker. Run it on a synthetic
//! pass and on a real Revo `raw_preview.ply` to compare them with the same numbers.
//!
//! ```text
//! cargo run --release -p scancraft-synth --example synth_realism -- <cloud.ply | scene dir> [marker_framework.mkf] [--rings] [--json out.json]
//! ```
//!
//! For a scene directory every pass is measured, with the marker file beside it. For a PLY the
//! marker file is the one given, or `marker_framework.mkf` in the same folder.

use std::path::{Path, PathBuf};

use glam::DVec3;
use scancraft_synth::realism::{measure, ring_table, summary, Realism};

fn load(ply: &Path, mkf: Option<&Path>) -> Result<(Vec<DVec3>, Vec<DVec3>, Vec<(DVec3, DVec3)>), String> {
    let (cloud, _) = scancraft_io::load_ply(ply).map_err(|e| format!("{}: {e}", ply.display()))?;
    let points: Vec<DVec3> = cloud.points.iter().map(|p| DVec3::from_array(p.position.map(f64::from))).collect();
    let normals: Vec<DVec3> = cloud.points.iter().map(|p| DVec3::from_array(p.normal.map(f64::from))).collect();
    let mkf = mkf.map(Path::to_path_buf).or_else(|| scancraft_io::marker_framework_beside(ply));
    let markers = match mkf {
        Some(m) if m.is_file() => scancraft_io::read_marker_framework(&m).map_err(|e| e.to_string())?.iter().map(|m| (DVec3::from_array(m.centre), DVec3::from_array(m.normal))).collect(),
        _ => Vec::new(),
    };
    Ok((points, normals, markers))
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let rings = args.iter().any(|a| a == "--rings");
    let json = args.iter().position(|a| a == "--json").and_then(|i| args.get(i + 1)).map(PathBuf::from);
    let positional: Vec<&String> = args.iter().filter(|a| !a.starts_with("--")).filter(|a| json.as_ref().is_none_or(|j| Path::new(a.as_str()) != j)).collect();
    let Some(first) = positional.first() else {
        eprintln!("usage: synth_realism <cloud.ply | scene dir> [marker_framework.mkf] [--rings] [--json out.json]");
        std::process::exit(2);
    };
    let first = PathBuf::from(first);
    let mut clouds: Vec<(PathBuf, Option<PathBuf>)> = Vec::new();
    if first.is_dir() {
        let truth = scancraft_synth::write::Truth::load(&first).unwrap_or_else(|e| panic!("{}: {e}", first.display()));
        for p in &truth.passes {
            clouds.push((first.join(&p.cloud), None));
        }
    } else {
        clouds.push((first, positional.get(1).map(PathBuf::from)));
    }
    let mut all: Vec<(String, Realism)> = Vec::new();
    for (ply, mkf) in clouds {
        let (points, normals, markers) = load(&ply, mkf.as_deref()).unwrap_or_else(|e| panic!("{e}"));
        let r = measure(&points, &normals, &markers, 6000);
        println!("{}\n  {}", ply.display(), summary(&r));
        if rings {
            print!("{}", ring_table(&r));
        }
        all.push((ply.display().to_string(), r));
    }
    if let Some(j) = json {
        std::fs::write(&j, serde_json::to_string_pretty(&all).unwrap()).expect("write the json");
    }
}
