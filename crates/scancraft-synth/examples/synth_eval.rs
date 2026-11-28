//! Runs every synthetic scene through the app's pipeline with the app's default settings
//! (Import Scan with the marker lists, object isolation, passes at their true poses, Merge,
//! Reconstruct, Make Solid) and measures both meshes against the truth.
//!
//! ```text
//! cargo run --release -p scancraft-synth --example synth_eval -- <data dir> <out dir> [filter] [--no-align] [--no-solid] [--whole-scene]
//! ```
//!
//! Each scene runs in a child process of its own (so its peak memory is its own and a crash
//! does not stop the rest). Per scene `<out>/<scene>/` gets `eval.json` (everything measured),
//! `reconstruct.ply` and `solid.ply` (vertex property `dev` = signed distance to the true part)
//! and `merged.ply` (the merged points, with `dev`). The run writes `<out>/results.json` and a
//! table `<out>/results.txt`. Renders: `scripts/render_features.py <data dir> <out dir>`.

use std::path::{Path, PathBuf};
use std::process::Command;

use scancraft_synth::eval::{summary_line, MeshEval};
use scancraft_synth::run::{run_scene_dir, write_dev_mesh, write_dev_points, RunOptions, SceneResult};

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    if args.first().map(String::as_str) == Some("--one") {
        let (Some(dir), Some(out)) = (args.get(1), args.get(2)) else { std::process::exit(2) };
        let options = RunOptions { align_check: !args.iter().any(|a| a == "--no-align"), solid: !args.iter().any(|a| a == "--no-solid"), whole_scene: args.iter().any(|a| a == "--whole-scene") };
        one(Path::new(dir), Path::new(out), &options);
        return;
    }
    let positional: Vec<&String> = args.iter().filter(|a| !a.starts_with("--")).collect();
    let (Some(data), Some(out)) = (positional.first(), positional.get(1)) else {
        eprintln!("usage: synth_eval <data dir> <out dir> [filter] [--no-align] [--no-solid] [--whole-scene]");
        std::process::exit(2);
    };
    let filter = positional.get(2).map(|s| s.as_str()).unwrap_or("");
    let (data, out) = (PathBuf::from(data), PathBuf::from(out));
    std::fs::create_dir_all(&out).expect("create the out dir");
    let mut scenes: Vec<PathBuf> = std::fs::read_dir(&data).expect("read the data dir").flatten().map(|e| e.path()).filter(|p| p.join("truth.json").is_file()).collect();
    scenes.sort();
    // Catalogue order when the index is there.
    if let Ok(index) = std::fs::read_to_string(data.join("index.json")) {
        if let Ok(v) = serde_json::from_str::<serde_json::Value>(&index) {
            let names: Vec<String> = v.get("scenes").and_then(|s| s.as_array()).map(|a| a.iter().filter_map(|s| s.get("name").and_then(|n| n.as_str()).map(String::from)).collect()).unwrap_or_default();
            scenes.sort_by_key(|p| names.iter().position(|n| p.file_name().is_some_and(|f| f.to_string_lossy() == *n)).unwrap_or(usize::MAX));
        }
    }
    let scenes: Vec<PathBuf> = scenes.into_iter().filter(|p| p.file_name().is_some_and(|n| n.to_string_lossy().contains(filter))).collect();
    let exe = std::env::current_exe().expect("own path");
    let passthrough: Vec<&String> = args.iter().filter(|a| *a == "--no-align" || *a == "--no-solid" || *a == "--whole-scene").collect();
    let mut results = Vec::new();
    for dir in &scenes {
        let name = dir.file_name().unwrap().to_string_lossy().to_string();
        let scene_out = out.join(&name);
        std::fs::create_dir_all(&scene_out).unwrap();
        let t = std::time::Instant::now();
        let status = Command::new(&exe).arg("--one").arg(dir).arg(&scene_out).args(&passthrough).status();
        let file = scene_out.join("eval.json");
        let res: SceneResult = match (status, std::fs::read_to_string(&file)) {
            (Ok(s), Ok(text)) if s.success() => serde_json::from_str(&text).expect("eval.json"),
            (s, _) => SceneResult { scene: name.clone(), error: Some(format!("the scene's run failed: {s:?}")), ..Default::default() },
        };
        println!("{name} ({:.0} s)", t.elapsed().as_secs_f64());
        for m in &res.evals {
            println!("  {:12} {}", m.mesh, summary_line(m));
        }
        if let Some(e) = &res.error {
            println!("  ERROR {e}");
        }
        results.push(res);
    }
    std::fs::write(out.join("results.json"), serde_json::to_string_pretty(&results).unwrap()).unwrap();
    std::fs::write(out.join("results.txt"), table(&results)).unwrap();
    println!("wrote {}", out.join("results.txt").display());
}

fn one(dir: &Path, out: &Path, options: &RunOptions) {
    let (res, truth, outputs) = match run_scene_dir(dir, options) {
        Ok(r) => r,
        Err(e) => {
            let res = SceneResult { scene: dir.file_name().unwrap().to_string_lossy().into(), error: Some(e), ..Default::default() };
            std::fs::write(out.join("eval.json"), serde_json::to_string_pretty(&res).unwrap()).unwrap();
            return;
        }
    };
    if let Some(o) = outputs {
        write_dev_mesh(&out.join("reconstruct.ply"), &o.reconstruct, &truth).expect("write reconstruct.ply");
        if let Some(s) = &o.solid {
            write_dev_mesh(&out.join("solid.ply"), s, &truth).expect("write solid.ply");
        }
        write_dev_points(&out.join("merged.ply"), &o.merged, &truth).expect("write merged.ply");
    }
    std::fs::write(out.join("eval.json"), serde_json::to_string_pretty(&res).unwrap()).unwrap();
}

fn cell(m: Option<&MeshEval>, solid: bool) -> String {
    let Some(m) = m else { return format!("{:>52}", "-") };
    let g = &m.global;
    let topo = if solid { format!("{}{} g{}", if m.topology.watertight { "W" } else { "open" }, if m.topology.components > 1 { format!(" c{}", m.topology.components) } else { String::new() }, m.topology.genus) } else { format!("loops {}", m.topology.boundary_loops) };
    format!(
        "{:>5.3} {:>5.3} {:>5.3} {:>5.1}% {:>4}/{:<4} {:>2}/{:<2} {:>12}{}",
        g.out_to_truth.rms_mm,
        g.out_to_truth.p99_mm,
        g.truth_to_out.p99_mm,
        100.0 * g.missing_share,
        m.probes_passed,
        m.probes,
        m.features.len() - m.features_failed(),
        m.features.len(),
        topo,
        if g.unseen_area_mm2 > 50.0 { format!(" unseen {:.0} mm² rms {:.2}", g.unseen_area_mm2, g.out_to_truth_unseen.rms_mm) } else { String::new() }
    )
}

fn table(results: &[SceneResult]) -> String {
    let mut s = String::new();
    s.push_str("Synthetic scenes through the app's pipeline (defaults; passes at their true poses)\n");
    s.push_str("o->t: output->truth (area weighted, vertices within 0.75 mm of a measured point) rms / p99; t->o: truth->output p99 over the seen surface; miss: seen surface with no mesh within 1 mm\n");
    s.push_str("probes: truth.json probes passed; feat: features without a failure; W = watertight, c = components, g = genus (truth genus in brackets)\n\n");
    s.push_str(&format!(
        "{:28} {:>8} | {:^52} | {:^52} | {:>5} {:>5} {:>6} | {}\n",
        "scene", "points", "RECONSTRUCT  o->t rms  p99  t->o p99  miss  probes  feat  topo", "MAKE SOLID  o->t rms  p99  t->o p99  miss  probes  feat  topo", "rec s", "sol s", "peakMB", "align (deg / mm)"
    ));
    for r in results {
        let pts: usize = r.passes.iter().map(|p| p.points).sum();
        let align: Vec<String> = r
            .alignment
            .iter()
            .map(|a| match (&a.error, a.rotation_error_deg, a.translation_rms_mm) {
                (Some(e), _, _) => format!("P{}: failed ({})", a.pass, e.chars().take(30).collect::<String>()),
                (_, Some(d), Some(t)) => format!("P{}: {d:.2}/{t:.2}", a.pass),
                _ => format!("P{}: ?", a.pass),
            })
            .collect();
        let tg = r.truth_topology.as_ref().map_or(String::new(), |t| format!(" [g{}]", t.genus));
        s.push_str(&format!(
            "{:28} {:>8} | {} | {}{} | {:>5.1} {:>5.1} {:>6.0} | {}\n",
            r.scene,
            pts,
            cell(r.eval("reconstruct"), false),
            cell(r.eval("solid"), true),
            tg,
            r.reconstruct.seconds,
            r.solid.seconds,
            r.peak_mb.unwrap_or(0.0),
            align.join(", ")
        ));
        if let Some(e) = &r.error {
            s.push_str(&format!("    ERROR {e}\n"));
        }
    }
    s.push_str("\nIsolation (points dropped by label part / sticker / table / stray; fallback = everything kept)\n");
    for r in results {
        for p in &r.passes {
            let d = p.dropped_by_label;
            if d.iter().any(|x| *x > 0) || p.isolation_fallback {
                s.push_str(&format!("  {:28} pass {}: kept {} of {}{}; dropped {} / {} / {} / {}\n", r.scene, p.id, p.isolation_kept, p.points, if p.isolation_fallback { " (fallback)" } else { "" }, d[0], d[1], d[2], d[3]));
            }
        }
    }
    s.push_str("\nFailures (feature: reasons) and the render to look at\n");
    for r in results {
        for m in &r.evals {
            for f in m.features.iter().filter(|f| !f.fails.is_empty()) {
                s.push_str(&format!("  {:28} {:11} {:24} {:14} {}  [img/{}__{}.png]\n", r.scene, m.mesh, f.id, f.kind, f.fails.join("; "), r.scene, f.id));
            }
            if let Some(po) = &m.pass_offset {
                if po.get("handled").and_then(|h| h.as_bool()) == Some(false) {
                    s.push_str(&format!("  {:28} {:11} {:24} {:14} step {} mm between pass-1-only, overlap and other-pass surface\n", r.scene, m.mesh, "(passes)", "pass_offset", po.get("step_mm").cloned().unwrap_or_default()));
                }
            }
        }
    }
    s
}
