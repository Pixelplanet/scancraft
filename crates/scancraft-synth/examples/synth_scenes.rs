//! Generates the synthetic test scenes.
//!
//! ```text
//! cargo run --release -p scancraft-synth --example synth_scenes -- <out dir> [name filter] [--scale S] [--no-mesh]
//! cargo run --release -p scancraft-synth --example synth_scenes -- --list [name filter]
//! ```
//!
//! Every scene whose name contains the filter is written to `<out dir>/<scene>/` (see the crate
//! README); `<out dir>/index.json` then lists every scene found in the folder.

use std::path::{Path, PathBuf};

use scancraft_synth::scenes::{self, Options};
use scancraft_synth::write::{dir_size, generate, Options as WriteOptions, Truth};

fn main() {
    let mut args = std::env::args().skip(1);
    let mut out: Option<PathBuf> = None;
    let mut filter = String::new();
    let mut scale = 1.0;
    let mut mesh = true;
    let mut list = false;
    while let Some(a) = args.next() {
        match a.as_str() {
            "--scale" => scale = args.next().and_then(|v| v.parse().ok()).expect("--scale needs a number"),
            "--no-mesh" => mesh = false,
            "--list" => list = true,
            _ if out.is_none() => out = Some(PathBuf::from(a)),
            _ => filter = a,
        }
    }
    if list {
        // `--list [filter]`: no output folder, the one free argument is the filter.
        let filter = if filter.is_empty() { out.map(|p| p.to_string_lossy().to_string()).unwrap_or_default() } else { filter };
        for s in scenes::build_matching(&filter, &Options { spacing_scale: scale }) {
            println!("{:28} {}", s.name, s.description);
        }
        return;
    }
    let Some(out) = out else {
        eprintln!("usage: synth_scenes <out dir> [name filter] [--scale S] [--no-mesh] [--list]");
        std::process::exit(2);
    };
    std::fs::create_dir_all(&out).expect("create the output folder");
    let built = scenes::build_matching(&filter, &Options { spacing_scale: scale });
    if built.is_empty() {
        eprintln!("no scene matches '{filter}'; scenes: {}", scenes::names().join(", "));
        std::process::exit(2);
    }
    for s in &built {
        let g = generate(s, &out, &WriteOptions { reference_mesh: mesh }).unwrap_or_else(|e| panic!("{}: {e}", s.name));
        let passes: Vec<String> = g
            .truth
            .passes
            .iter()
            .map(|p| format!("{} {} pts ({} sticker, {} table, {} stray, {} cut), {} markers", p.name, p.stats.points, p.stats.sticker, p.stats.table, p.stats.strays, p.stats.cut, p.listed_markers.len()))
            .collect();
        println!("{:28} {:6.1} MB {:5.1} s  {}", s.name, g.bytes as f64 / 1e6, g.seconds, passes.join("; "));
    }
    write_index(&out);
}

fn write_index(out: &Path) {
    let mut scenes = Vec::new();
    let mut total = 0;
    let mut dirs: Vec<PathBuf> = std::fs::read_dir(out).map(|r| r.flatten().map(|e| e.path()).filter(|p| p.join("truth.json").is_file()).collect()).unwrap_or_default();
    dirs.sort();
    for d in dirs {
        let Ok(t) = Truth::load(&d) else { continue };
        let bytes = dir_size(&d);
        total += bytes;
        scenes.push(serde_json::json!({
            "scene": t.scene,
            "description": t.description,
            "tags": t.tags,
            "passes": t.passes.iter().map(|p| serde_json::json!({"cloud": p.cloud, "points": p.stats.points, "markers": p.listed_markers.len(), "marker_file": p.marker_file})).collect::<Vec<_>>(),
            "stickers": t.stickers.len(),
            "features": t.features.iter().map(|f| format!("{} ({})", f.id, f.kind)).collect::<Vec<_>>(),
            "probes": t.features.iter().map(|f| f.probes.len()).sum::<usize>(),
            "reference_mesh": t.reference_mesh.as_ref().map(|m| &m.file),
            "megabytes": bytes as f64 / 1e6,
        }));
    }
    let index = serde_json::json!({"format": scancraft_synth::write::FORMAT, "scenes": scenes, "total_megabytes": total as f64 / 1e6});
    std::fs::write(out.join("index.json"), serde_json::to_string_pretty(&index).unwrap()).expect("write index.json");
    println!("{} scenes, {:.1} MB in {}", scenes.len(), total as f64 / 1e6, out.display());
}
