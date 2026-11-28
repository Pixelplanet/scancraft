//! Regression tests of Reconstruct and Make Solid on synthetic scenes, run the way the app runs
//! them ([`crate::app_pipeline`]) and measured with [`crate::eval`].
//!
//! The fast ones build small scenes of their own (a few seconds each in a debug build). The
//! heavy one runs the whole catalogue from a generated data folder:
//!
//! ```text
//! set SCANCRAFT_SYNTH_DATA=E:\Claude\testdata\synthetic
//! cargo test --release -p scancraft-synth -- --ignored synth_catalogue
//! ```
//!
//! The limits are what the pipeline achieves now (2026-11-28) with some room, so a change that
//! makes it worse fails. Where the pipeline is known to fall short, an `#[ignore]`d
//! `known_failure_*` test asserts the ideal: run them with `-- --ignored known_failure` and
//! un-ignore one when it passes.

use std::path::{Path, PathBuf};

use glam::DVec3;
use serde_json::json;

use crate::eval::{FeatureEval, MeshEval};
use crate::run::{run_scene_dir, RunOptions, SceneResult};
use crate::scene::{ring_views, view, Feature, PassSpec, Probe, Region, Scene, Sticker, Strays, View};
use crate::scene::LineFamily;
use crate::sdf::{Profile2, Shape};
use crate::write::{generate, Options};

fn tmp_dir(name: &str) -> PathBuf {
    let d = std::env::temp_dir().join(format!("scancraft-synth-eval-{name}-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&d);
    std::fs::create_dir_all(&d).unwrap();
    d
}

/// Eight views on a ring 48° up plus one from above (as the catalogue's `above`).
fn above(center: DVec3) -> Vec<View> {
    let mut v = ring_views(center, 150.0, 168.0, 8, 22.5);
    v.push(view(center + DVec3::new(8.0, 5.0, 228.0), center));
    v
}

fn hit_down(part: &Shape, x: f64, y: f64) -> DVec3 {
    crate::scenes::hit(part, DVec3::new(x, y, 400.0), -DVec3::Z)
}

fn marker_feature(s: &Sticker) -> Feature {
    let c = crate::sdf::v3(s.center);
    let n = crate::sdf::v3(s.normal);
    Feature {
        id: s.id.clone(),
        kind: if s.cut_radius.is_some() { "marker_hole" } else { "marker_site" }.into(),
        expect: "site flush with the true surface".into(),
        params: json!({"center": s.center, "normal": s.normal, "cut_radius_mm": s.cut_radius, "partial": false}),
        probes: vec![Probe::height(c, n, 0.0, 0.05, "site centre on the true surface")],
    }
}

fn scene(name: &str, part: Shape, bounds: [[f64; 3]; 2], stickers: Vec<Sticker>, passes: Vec<PassSpec>, mut features: Vec<Feature>) -> Scene {
    let mut all: Vec<Feature> = stickers.iter().filter(|s| s.on_table_of_pass.is_none()).map(marker_feature).collect();
    all.append(&mut features);
    Scene { name: name.into(), description: name.into(), seed: 7, part, bounds, stickers, passes, features: all, truth_cell_mm: 0.3, tags: Vec::new() }
}

/// Runs a scene through the generator's files and the app's pipeline.
fn run(scene: &Scene, options: &RunOptions) -> SceneResult {
    let dir = tmp_dir(&scene.name);
    generate(scene, &dir, &Options::default()).expect("generate");
    let (res, _, _) = run_scene_dir(&dir.join(&scene.name), options).expect("run");
    let _ = std::fs::remove_dir_all(&dir);
    res
}

fn feat<'a>(m: &'a MeshEval, id: &str) -> &'a FeatureEval {
    m.features.iter().find(|f| f.id == id).unwrap_or_else(|| panic!("no feature {id}"))
}

fn num(f: &FeatureEval, key: &str) -> f64 {
    f.metrics.get(key).and_then(|v| v.as_f64()).unwrap_or_else(|| panic!("{}: no {key} in {:?}", f.id, f.metrics))
}

fn print(res: &SceneResult) {
    for m in &res.evals {
        eprintln!("{} {}: {}", res.scene, m.mesh, crate::eval::summary_line(m));
        for f in &m.features {
            let mt: serde_json::Map<String, serde_json::Value> = f.metrics.iter().filter(|(k, _)| *k != "profile_mm").map(|(k, v)| (k.clone(), v.clone())).collect();
            eprintln!("  {} {} {:?} {}", f.id, f.kind, f.fails, serde_json::Value::Object(mt));
        }
    }
}

/// A 40 x 30 x 3 mm plate: one sticker kept, one cut out, a 3 mm through slot. Top pass only.
fn mini_plate() -> Scene {
    let plate = Shape::Box { center: [0.0, 0.0, -1.5], half: [20.0, 15.0, 1.5], round: 0.0 };
    let slot = Shape::Prism { profile: Profile2::Stadium { a: [-7.0, -7.0], b: [7.0, -7.0], radius: 1.5 }, z0: -10.0, z1: 10.0 };
    let part = Shape::subtract(plate, vec![slot]);
    let mut kept = Sticker::standard("S-kept", hit_down(&part, -9.0, 5.0), DVec3::Z);
    kept.dot_radius = 3.0;
    let mut cut = Sticker::standard("S-cut", hit_down(&part, 9.0, 5.0), DVec3::Z);
    cut.cut_radius = Some(5.5);
    let mut pass = PassSpec::new("top", above(DVec3::ZERO));
    pass.spacing_mm = 0.4;
    let slot_f = Feature {
        id: "slot".into(),
        kind: "through_slot".into(),
        expect: "open, 3 mm wide".into(),
        params: json!({"axis": [1, 0], "center": [0.0, -7.0], "length_mm": 17.0, "width_mm": 3.0, "through": true}),
        probes: vec![Probe::clear(DVec3::new(0.0, -7.0, -1.5), 0.8, "slot open at mid-depth")],
    };
    scene("mini_plate", part, [[-20.0, -15.0, -3.0], [20.0, 15.0, 0.0]], vec![kept, cut], vec![pass], vec![slot_f])
}

#[test]
fn mini_plate_markers_flush_slot_open() {
    let res = run(&mini_plate(), &RunOptions { align_check: false, solid: true, whole_scene: false });
    print(&res);
    assert!(res.error.is_none(), "{:?}", res.error);
    // A plate on its own has no table: isolation keeps everything.
    assert!(res.passes[0].dropped_by_label[0] < 50, "isolation dropped part points: {:?}", res.passes[0]);

    let rec = res.eval("reconstruct").unwrap();
    assert!(rec.global.out_to_truth.rms_mm < 0.15, "Reconstruct rms {} (0.11 on 2026-11-28)", rec.global.out_to_truth.rms_mm);
    let f = feat(rec, "S-kept");
    assert!(num(f, "open_share") < 0.02 && num(f, "centre_dev_mm").abs() < 0.05 && num(f, "site_rms_mm") < 0.05, "Reconstruct S-kept: {:?}", f.metrics);
    let slot = feat(rec, "slot");
    assert_eq!(num(slot, "open"), num(slot, "stations"), "Reconstruct closes the 3 mm slot: {:?}", slot.metrics);
    assert!((num(slot, "width_mid_mm") - 3.0).abs() < 0.3, "Reconstruct slot width: {:?}", slot.metrics);

    let solid = res.eval("solid").unwrap();
    assert!(solid.topology.watertight, "Make Solid is not watertight: {:?}", solid.topology);
    assert_eq!(solid.topology.components, 1);
    assert_eq!(solid.topology.genus, 1, "the slot is one through hole: {:?}", solid.topology);
    assert!(solid.global.out_to_truth.rms_mm < 0.1, "Make Solid rms {} (0.077 on 2026-11-28)", solid.global.out_to_truth.rms_mm);
    for id in ["S-kept", "S-cut"] {
        // Make Solid closes the cut-out with its hole filling (no marker fairing: see the known
        // failure below); 0.044 / 0.063 mm rms on 2026-11-28.
        let f = feat(solid, id);
        assert!(num(f, "open_share") < 0.02 && num(f, "centre_dev_mm").abs() < 0.07 && num(f, "site_rms_mm") < 0.09, "Make Solid {id}: {:?}", f.metrics);
    }
    let slot = feat(solid, "slot");
    assert_eq!(num(slot, "open"), num(slot, "stations"), "Make Solid closes the 3 mm slot: {:?}", slot.metrics);
    assert!((num(slot, "width_mid_mm") - 3.0).abs() < 0.3, "Make Solid slot width: {:?}", slot.metrics);
}

/// Known failure (2026-11-28): Revo lists the stickers it cut out, but `markers_on_surface`
/// keeps only markers with a point within 3 mm of the centre, and a cut-out has none within
/// 5.5 mm. So Reconstruct never removes or fills those sites (left open) and Make Solid fills
/// them as plain holes, without the marker fairing.
#[test]
#[ignore = "known failure: cut-out marker sites are not used (markers_on_surface's 3 mm rule)"]
fn known_failure_cut_out_marker_sites_are_filled() {
    let res = run(&mini_plate(), &RunOptions { align_check: false, solid: true, whole_scene: false });
    print(&res);
    assert_eq!(res.passes[0].markers_on_surface, 2, "a listed cut-out marker is dropped");
    for m in &res.evals {
        let f = feat(m, "S-cut");
        assert!(num(f, "open_share") < 0.02 && num(f, "site_rms_mm") < 0.05, "{} leaves the cut-out open or uneven: {:?}", m.mesh, f.metrics);
    }
}

/// A sphere cap R 50 (height 10): one sticker kept on top, one cut out on the flank.
fn mini_dome() -> Scene {
    let part = Shape::Intersect { parts: vec![Shape::Sphere { center: [0.0, 0.0, -40.0], radius: 50.0 }, Shape::HalfSpace { normal: [0.0, 0.0, -1.0], offset: 0.0 }] };
    let top = hit_down(&part, 0.0, 0.0);
    let kept = Sticker::standard("D-top", top, part.normal(top));
    let flank = hit_down(&part, 16.0, 4.0);
    let mut cut = Sticker::standard("D-cut", flank, part.normal(flank));
    cut.cut_radius = Some(5.5);
    let mut pass = PassSpec::new("outside", above(DVec3::new(0.0, 0.0, 5.0)));
    pass.spacing_mm = 0.4;
    scene("mini_dome", part, [[-31.0, -31.0, 0.0], [31.0, 31.0, 10.0]], vec![kept, cut], vec![pass], Vec::new())
}

#[test]
fn mini_dome_marker_sites_follow_the_curvature() {
    let res = run(&mini_dome(), &RunOptions { align_check: false, solid: true, whole_scene: false });
    print(&res);
    assert!(res.error.is_none(), "{:?}", res.error);
    // Reconstruct: the kept sticker; Make Solid: both sites (the cut-out one by hole filling).
    let sites = [("reconstruct", "D-top"), ("solid", "D-top"), ("solid", "D-cut")];
    for (mesh, id) in sites {
        let f = feat(res.eval(mesh).unwrap(), id);
        assert!(num(f, "open_share") < 0.02, "{mesh} {id} open: {:?}", f.metrics);
        assert!(num(f, "centre_dev_mm").abs() < 0.05, "{mesh} {id} centre: {:?}", f.metrics);
        let k = num(f, "curvature_followed");
        assert!((0.8..=1.2).contains(&k), "{mesh} {id} does not follow the dome (k {k:.2}): {:?}", f.metrics);
    }
    let rec = res.eval("reconstruct").unwrap();
    assert!(rec.global.out_to_truth.rms_mm < 0.02, "Reconstruct rms {} (0.009 on 2026-11-28)", rec.global.out_to_truth.rms_mm);
    let solid = res.eval("solid").unwrap();
    assert!(solid.topology.watertight && solid.topology.genus == 0, "{:?}", solid.topology);
}

/// A 30 x 30 x 4 mm plate with cross-line laser strays behind the top in a 12 mm window.
fn mini_strays() -> Scene {
    let part = Shape::Box { center: [0.0, 0.0, -2.0], half: [15.0, 15.0, 2.0], round: 0.0 };
    let mut pass = PassSpec::new("top", above(DVec3::ZERO));
    pass.spacing_mm = 0.4;
    let (n1, n2) = (glam::DVec2::from_angle(20f64.to_radians()), glam::DVec2::from_angle(90f64.to_radians()));
    pass.strays = Some(Strays {
        region: Region::Sphere { center: [0.0; 3], radius: 12.0 },
        families: vec![LineFamily { normal: [n1.x, n1.y, 0.0], spacing_mm: 9.0, phase: 0.13 }, LineFamily { normal: [n2.x, n2.y, 0.0], spacing_mm: 11.0, phase: 0.41 }],
        width_mm: 2.0,
        depth_mm: [0.6, 1.0],
        fraction: 0.6,
        wrong_way: 0.83,
        noise_mm: 0.05,
    });
    let f = Feature {
        id: "strays".into(),
        kind: "laser_strays".into(),
        expect: "strays dropped".into(),
        params: json!({"window_center": [0, 0, 0], "window_radius_mm": 12, "depth_mm": [0.6, 1.0]}),
        probes: vec![Probe::clear(DVec3::new(2.0, 1.0, -0.8), 0.45, "nothing where the strays are")],
    };
    scene("mini_strays", part, [[-15.0, -15.0, -4.0], [15.0, 15.0, 0.0]], Vec::new(), vec![pass], vec![f])
}

/// Guards what the stray handling achieves now (2026-11-28); the ideal is the known failure
/// below.
#[test]
fn mini_strays_do_not_get_worse() {
    let res = run(&mini_strays(), &RunOptions { align_check: false, solid: true, whole_scene: false });
    print(&res);
    assert!(res.strays.input > 200, "the scene has strays: {:?}", res.strays);
    // 376 of 665 left after Reconstruct's preparation on 2026-11-28.
    assert!(res.strays.after_prepare * 100 <= res.strays.input * 65, "Reconstruct's preparation keeps more strays: {:?}", res.strays);
    let rec = res.eval("reconstruct").unwrap();
    let f = feat(rec, "strays");
    assert!(num(f, "window_p01_mm") > -0.2, "Reconstruct pits deeper: {:?}", f.metrics);
    assert!(num(f, "inner_sheet_share") < 0.45, "Reconstruct keeps more of the stray layer: {:?}", f.metrics);
    let solid = res.eval("solid").unwrap();
    assert!(solid.topology.watertight, "{:?}", solid.topology);
    assert!(num(feat(solid, "strays"), "window_p01_mm") > -0.25, "Make Solid pits deeper: {:?}", feat(solid, "strays").metrics);
}

/// Known failure (2026-11-28): Reconstruct's preparation keeps about 55 % of the strays (a
/// layer under a third of the window, floating pieces, pits to -0.14 mm); Make Solid drops them
/// but measures its wall thickness from them, so the closed plate comes out about 1 mm thick
/// (its made-up bottom lies where the strays were).
#[test]
#[ignore = "known failure: laser strays (see the doc comment)"]
fn known_failure_strays_are_dropped() {
    let res = run(&mini_strays(), &RunOptions { align_check: false, solid: true, whole_scene: false });
    print(&res);
    assert!(res.strays.after_prepare * 20 <= res.strays.input, "Reconstruct's preparation keeps strays: {:?}", res.strays);
    for m in &res.evals {
        let f = feat(m, "strays");
        assert!(num(f, "inner_sheet_share") <= 0.01, "{}: a layer inside the plate: {:?}", m.mesh, f.metrics);
        assert!(num(f, "window_p01_mm") > -0.05, "{}: pits: {:?}", m.mesh, f.metrics);
    }
}

// ------------------------------------------------------------------------- the catalogue

/// Per scene: the most failing features allowed (Reconstruct, Make Solid) and the fewest probes
/// that must pass (Reconstruct, Make Solid; 97 % of what passed). Measured 2026-11-28 on
/// `agent/synth-scenes` with the app's defaults (isolation on, passes at their true poses);
/// tighten them when the pipeline gets better. Make Solid must always be watertight.
const CATALOGUE_LIMITS: &[(&str, usize, usize, usize, usize)] = &[
    ("ball_r15_marker_holes", 8, 8, 7, 15),
    ("ball_r15_markers", 3, 4, 50, 40),
    ("box_edges", 6, 5, 7, 12),
    ("cup_rim_partial", 1, 1, 64, 56),
    ("cylinder_r15_marker_holes", 7, 3, 6, 46),
    ("cylinder_r15_markers", 1, 1, 58, 56),
    ("cylinder_r65_marker_holes", 8, 1, 7, 66),
    ("cylinder_r65_markers", 1, 1, 66, 65),
    ("dome_density_steps", 1, 2, 94, 87),
    ("dome_r60_marker_holes", 9, 3, 11, 77),
    ("dome_r60_markers", 1, 2, 86, 81),
    ("dome_r60_noise_002", 1, 2, 87, 81),
    ("dome_r60_noise_005", 1, 2, 86, 81),
    ("dome_r60_noise_010", 1, 2, 84, 81),
    ("flat_laser_strays", 2, 3, 76, 64),
    ("flat_marker_holes", 9, 8, 7, 46),
    ("flat_markers", 1, 1, 79, 78),
    ("flat_plain", 1, 1, 9, 8),
    ("flat_two_pass_offset", 8, 1, 20, 83),
    ("flat_vents", 8, 12, 63, 73),
    ("grazing_cylinder_r30", 5, 5, 16, 16),
    ("perforated_grille", 3, 2, 108, 112),
    ("plate_screws_bosses_ribs", 8, 8, 52, 55),
    ("relief_fine", 5, 8, 32, 24),
    ("saddle_marker_holes", 8, 3, 4, 52),
    ("saddle_markers", 2, 2, 61, 63),
    ("shell_two_pass", 10, 18, 78, 23),
    ("thin_fins", 4, 4, 6, 5),
];

#[test]
#[ignore = "needs the generated scenes: set SCANCRAFT_SYNTH_DATA (see the module docs)"]
fn synth_catalogue() {
    let Some(data) = std::env::var_os("SCANCRAFT_SYNTH_DATA") else {
        eprintln!("SCANCRAFT_SYNTH_DATA is not set; skipping");
        return;
    };
    let data = PathBuf::from(data);
    let filter = std::env::var("SCANCRAFT_SYNTH_FILTER").unwrap_or_default();
    let mut problems = Vec::new();
    for &(name, max_rec, max_solid, min_rec_probes, min_solid_probes) in CATALOGUE_LIMITS.iter().filter(|l| l.0.contains(filter.as_str())) {
        let dir: &Path = &data.join(name);
        let (res, _, _) = run_scene_dir(dir, &RunOptions { align_check: false, solid: true, whole_scene: false }).unwrap_or_else(|e| panic!("{name}: {e}"));
        let rec = res.eval("reconstruct").expect("Reconstruct ran");
        let solid = res.eval("solid").expect("Make Solid ran");
        let fails = |m: &MeshEval| m.features.iter().filter(|f| !f.fails.is_empty()).map(|f| format!("{} ({})", f.id, f.fails.join("; "))).collect::<Vec<_>>();
        eprintln!("{name}: Reconstruct {} | Make Solid {}", crate::eval::summary_line(rec), crate::eval::summary_line(solid));
        if rec.features_failed() > max_rec {
            problems.push(format!("{name}: Reconstruct fails {} features (limit {max_rec}): {:?}", rec.features_failed(), fails(rec)));
        }
        if solid.features_failed() > max_solid {
            problems.push(format!("{name}: Make Solid fails {} features (limit {max_solid}): {:?}", solid.features_failed(), fails(solid)));
        }
        if rec.probes_passed < min_rec_probes {
            problems.push(format!("{name}: Reconstruct passes {} probes (at least {min_rec_probes})", rec.probes_passed));
        }
        if solid.probes_passed < min_solid_probes {
            problems.push(format!("{name}: Make Solid passes {} probes (at least {min_solid_probes})", solid.probes_passed));
        }
        if !solid.topology.watertight {
            problems.push(format!("{name}: Make Solid is not watertight: {:?}", solid.topology));
        }
    }
    assert!(problems.is_empty(), "{}", problems.join("\n"));
}
