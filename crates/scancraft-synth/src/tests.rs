//! Tests of the generator itself: the catalogue is consistent with its own truth, the simulated
//! scans have the geometry they claim (sticker heights, slot widths, strays, offsets), the files
//! are read back by scancraft-io as the app reads them.

use glam::DVec3;

use crate::scan::{scan_pass, PointKind};
use crate::scene::{view, ProbeKind, Scene};
use crate::scenes::{self, Options};
use crate::sdf::v3;
use crate::truth::reference_mesh;
use crate::write::{generate, Options as WriteOptions, Truth, LABEL_PART};

fn scene(name: &str, scale: f64) -> Scene {
    scenes::build(name, &Options { spacing_scale: scale }).expect(name)
}

fn temp_dir(tag: &str) -> std::path::PathBuf {
    let d = std::env::temp_dir().join(format!("scancraft-synth-{tag}-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&d);
    std::fs::create_dir_all(&d).unwrap();
    d
}

/// Equal JSON, numbers within a relative 1e-12 (serde_json may parse a float one ulp off).
fn json_close(a: &serde_json::Value, b: &serde_json::Value) -> bool {
    use serde_json::Value;
    match (a, b) {
        (Value::Number(x), Value::Number(y)) => {
            let (x, y) = (x.as_f64().unwrap(), y.as_f64().unwrap());
            (x - y).abs() <= 1e-12 * x.abs().max(y.abs()).max(1.0)
        }
        (Value::Array(x), Value::Array(y)) => x.len() == y.len() && x.iter().zip(y).all(|(p, q)| json_close(p, q)),
        (Value::Object(x), Value::Object(y)) => x.len() == y.len() && x.iter().all(|(k, v)| y.get(k).is_some_and(|w| json_close(v, w))),
        _ => a == b,
    }
}

#[test]
fn every_scene_is_consistent_with_its_truth() {
    let names = scenes::names();
    assert!(names.len() >= 25);
    let mut seen = std::collections::HashSet::new();
    for name in names {
        assert!(seen.insert(name), "duplicate {name}");
        let s = scene(name, 1.0);
        let f = |p: DVec3| s.part.sdf(p);
        // The bounds hold the part: every corner of the box grown by 1 mm is outside.
        let (lo, hi) = (v3(s.bounds[0]) - DVec3::ONE, v3(s.bounds[1]) + DVec3::ONE);
        for k in 0..8 {
            let c = DVec3::new(if k & 1 == 0 { lo.x } else { hi.x }, if k & 2 == 0 { lo.y } else { hi.y }, if k & 4 == 0 { lo.z } else { hi.z });
            assert!(f(c) > 0.0, "{name}: bounds corner {c} is inside the part");
        }
        assert!(!s.passes.is_empty(), "{name}");
        assert_eq!(s.passes[0].scan_pose, crate::Rigid::IDENTITY, "{name}: pass 1 is in the part frame");
        for st in &s.stickers {
            let on = if st.on_table_of_pass.is_some() { 0.0 } else { f(v3(st.center)).abs() };
            assert!(on < 1e-3, "{name} {}: centre {on} mm off the surface", st.id);
            assert!((v3(st.normal).length() - 1.0).abs() < 1e-9);
            if st.on_table_of_pass.is_none() {
                assert!(v3(st.normal).dot(s.part.normal(v3(st.center))) > 0.999, "{name} {}", st.id);
            }
        }
        // Every probe holds on the truth itself.
        for feat in &s.features {
            for p in &feat.probes {
                let at = v3(p.point);
                let d = f(at);
                match &p.check {
                    ProbeKind::On { tol } => assert!(d.abs() <= tol * 0.25, "{name} {} '{}': {d} off the surface", feat.id, p.note),
                    ProbeKind::Clear { radius } => assert!(d.abs() >= radius - 1e-6, "{name} {} '{}': surface {d} from a clear probe of radius {radius}", feat.id, p.note),
                    ProbeKind::Through { axis, half_length } => {
                        let n = v3(*axis);
                        let roots = crate::eval::truth_roots_part(&s.part, at - n * *half_length, n, 0.0, 2.0 * half_length);
                        assert!(roots.is_empty(), "{name} {} '{}': the true surface crosses a through probe at {roots:?}", feat.id, p.note);
                    }
                    ProbeKind::Height { normal, expect, tol, .. } => {
                        let n = v3(*normal);
                        let at = at + n * *expect;
                        assert!(f(at).abs() <= tol * 0.2, "{name} {} '{}': {} off", feat.id, p.note, f(at));
                        assert!(s.part.normal(at).dot(n) > 0.9, "{name} {} '{}': normal {} vs {n}", feat.id, p.note, s.part.normal(at));
                    }
                }
            }
        }
        // The scene survives JSON as it is written to truth.json.
        let back: Scene = serde_json::from_str(&serde_json::to_string(&s).unwrap()).unwrap();
        assert!(json_close(&serde_json::to_value(&back).unwrap(), &serde_json::to_value(&s).unwrap()), "{name}: JSON round trip");
    }
}

#[test]
fn stickers_stand_proud_by_their_height_and_are_listed() {
    let mut s = scene("flat_markers", 2.0);
    s.passes[0].noise_mm = 0.0;
    s.passes[0].noise.hatch_mm = 0.0;
    let scan = scan_pass(&s, 0);
    let mut on_disc = 0;
    for p in &scan.points {
        match p.kind {
            PointKind::Sticker(i) => {
                let st = &s.stickers[i as usize];
                let (lateral, _) = st.local(p.true_pos);
                if lateral < st.flat_radius - 0.2 && p.true_normal.z > 0.99 {
                    assert!((p.pos.z - (st.center[2] + st.height)).abs() < 1e-3, "{}: sticker top at {}", st.id, p.pos.z);
                    on_disc += 1;
                }
            }
            PointKind::Part if p.true_normal.z > 0.99 => assert!(p.pos.z.abs() < 0.011, "plate top at {}", p.pos.z),
            _ => {}
        }
    }
    assert!(on_disc > 100, "{on_disc}");
    assert_eq!(scan.listed.len(), s.stickers.len());
    for m in &scan.listed {
        let st = &s.stickers[m.sticker];
        assert!((m.centre - v3(st.center)).length() < 0.4);
        assert!(m.normal.dot(v3(st.normal)) > 0.99);
    }
}

#[test]
fn slots_have_their_width_in_the_sampled_points() {
    let mut s = scene("flat_vents", 1.0);
    s.passes.truncate(1);
    s.passes[0].views = vec![view(DVec3::new(0.0, 5.0, 230.0), DVec3::ZERO)];
    s.passes[0].noise_mm = 0.0;
    s.passes[0].noise.hatch_mm = 0.0;
    let scan = scan_pass(&s, 0);
    let top: Vec<DVec3> = scan.points.iter().filter(|p| p.kind == PointKind::Part && p.pos.z > -0.05).map(|p| p.pos).collect();
    for (w, x) in [(1.5, -45.0), (2.0, -30.0), (3.0, -15.0), (4.0, 0.0), (6.0, 18.0)] {
        let band: Vec<f64> = top.iter().filter(|p| (p.y - 10.0).abs() < 5.0 && (p.x - x).abs() < 6.0).map(|p| p.x).collect();
        let left = band.iter().copied().filter(|v| *v < x).fold(f64::NEG_INFINITY, f64::max);
        let right = band.iter().copied().filter(|v| *v > x).fold(f64::INFINITY, f64::min);
        let gap = right - left;
        assert!(gap >= w - 0.05 && gap <= w + 0.45, "slot {w}: gap {gap} in the sampled top");
        assert!(!band.iter().any(|v| (v - x).abs() < w / 2.0 - 0.01), "points inside the {w} mm slot");
    }
}

#[test]
fn cut_out_stickers_leave_holes_with_surface_round_them() {
    let s = scene("flat_marker_holes", 2.0);
    let scan = scan_pass(&s, 0);
    assert!(scan.stats.cut > 0);
    for st in &s.stickers {
        let r = st.cut_radius.unwrap();
        let mut ring = 0;
        for p in &scan.points {
            let (lateral, axial) = st.local(p.true_pos);
            if axial.abs() < 1.0 {
                assert!(lateral >= r, "{}: a point {lateral} mm from the centre, cut radius {r}", st.id);
                if lateral < r + 2.0 {
                    ring += 1;
                }
            }
        }
        assert!(ring > 10, "{}: no surface round the hole", st.id);
    }
    // The marker file still lists them (Revo lists the markers it tracked).
    assert_eq!(scan.listed.len(), s.stickers.len());
}

#[test]
fn laser_strays_lie_behind_the_surface_and_mostly_face_the_wrong_way() {
    let s = scene("flat_laser_strays", 2.0);
    let scan = scan_pass(&s, 0);
    let strays: Vec<_> = scan.points.iter().filter(|p| p.kind == PointKind::Stray).collect();
    assert!(strays.len() > 100, "{}", strays.len());
    for p in &strays {
        let d = s.part.sdf(p.pos);
        assert!((-1.3..=-0.3).contains(&d), "stray at depth {d}");
    }
    let wrong = strays.iter().filter(|p| p.normal.dot(p.true_normal) < 0.0).count() as f64 / strays.len() as f64;
    assert!((wrong - 0.83).abs() < 0.08, "{wrong}");
}

#[test]
fn a_pass_that_sits_too_deep_is_offset_by_its_bias() {
    let mut s = scene("flat_two_pass_offset", 2.0);
    s.passes[1].noise_mm = 0.0;
    s.passes[1].noise.hatch_mm = 0.0;
    let scan = scan_pass(&s, 1);
    let tops: Vec<f64> = scan.points.iter().filter(|p| p.kind == PointKind::Part && p.true_normal.z > 0.99).map(|p| p.pos.z / p.ray.z.abs()).collect();
    let mean = tops.iter().sum::<f64>() / tops.len() as f64;
    assert!((mean + 0.15).abs() < 1e-3, "{mean}");
    // Pass 2 keeps to its side of the overlap.
    assert!(scan.points.iter().all(|p| p.true_pos.x >= -20.0 - 1e-9));
}

#[test]
fn scans_are_deterministic() {
    let s = scene("dome_r60_markers", 3.0);
    let a = scan_pass(&s, 0);
    let b = scan_pass(&s, 0);
    assert_eq!(a.points.len(), b.points.len());
    assert!(a.points.iter().zip(&b.points).all(|(p, q)| p.pos == q.pos && p.normal == q.normal));
}

#[test]
fn estimated_normals_are_as_rough_as_a_scanner_s() {
    // Revo's raw_preview.ply: 4.7-5.3° median against a 2 mm plane fit (synth_realism).
    let s = scene("dome_r60_markers", 1.0);
    let scan = scan_pass(&s, 0);
    let mut off: Vec<f64> = scan.points.iter().filter(|p| p.kind == PointKind::Part).map(|p| p.normal.dot(p.true_normal).clamp(-1.0, 1.0).acos().to_degrees()).collect();
    off.sort_by(f64::total_cmp);
    let median = off[off.len() / 2];
    assert!((3.0..9.0).contains(&median), "median normal error {median}°");
    assert!(scan.points.iter().all(|p| p.normal.dot(-p.ray) >= 0.0), "normals face the scanner");
}

#[test]
fn raw_passes_look_like_revo_s_raw_preview() {
    // S1 / S2 / S3 raw_preview.ply (2026-11-28, synth_realism): 4.8 / 2.9 / 3.5 points per mm²,
    // NN p90 0.49-0.55, normals median 4.7-5.3°, residual std 0.022-0.030, neighbour
    // correlation 0.25-0.42 (0.2-0.5 mm) and 0.03-0.08 (0.5-0.9 mm).
    let s = scene("dome_r60_markers", 1.0);
    let scan = scan_pass(&s, 0);
    let part: Vec<_> = scan.points.iter().filter(|p| p.kind == PointKind::Part).collect();
    let pos: Vec<DVec3> = part.iter().map(|p| p.pos).collect();
    let nrm: Vec<DVec3> = part.iter().map(|p| p.normal).collect();
    let r = crate::realism::measure(&pos, &nrm, &[], 3000);
    eprintln!("{}", crate::realism::summary(&r));
    assert!((2.5..6.0).contains(&r.density_per_mm2), "{}", crate::realism::summary(&r));
    assert!((0.4..0.62).contains(&r.nn_p90_mm), "{}", crate::realism::summary(&r));
    assert!((3.5..8.0).contains(&r.normal_err_median_deg), "{}", crate::realism::summary(&r));
    assert!((0.018..0.035).contains(&r.residual_std_mm), "{}", crate::realism::summary(&r));
    assert!((0.22..0.5).contains(&r.corr_02_05), "{}", crate::realism::summary(&r));
}

#[test]
fn raw_stickers_keep_their_dot_and_lose_their_ring_fused_ones_are_cut() {
    let s = scene("flat_markers", 1.0);
    let scan = scan_pass(&s, 0);
    let density = |st: &crate::scene::Sticker, a: f64, b: f64| {
        let n = scan.points.iter().filter(|p| {
            let (l, ax) = st.local(p.true_pos);
            l >= a && l < b && ax.abs() < 1.5
        }).count();
        n as f64 / (std::f64::consts::PI * (b * b - a * a))
    };
    for st in s.stickers.iter().filter(|m| m.on_table_of_pass.is_none()) {
        let (dot, ring) = (density(st, 0.0, 2.8), density(st, 3.1, 4.4));
        assert!(dot > 2.0, "{}: dot {dot:.2} pts/mm²", st.id);
        assert!(ring < 0.85 * dot, "{}: ring {ring:.2} vs dot {dot:.2}", st.id);
        if st.halo_radius > 6.0 {
            let halo = density(st, 4.6, st.halo_radius - 0.2);
            assert!(halo < 0.35 * dot, "{}: halo {halo:.2} vs dot {dot:.2}", st.id);
        }
    }
    let f = scene("flat_marker_holes", 1.0);
    let scan = scan_pass(&f, 0);
    for st in &f.stickers {
        let nearest = scan.points.iter().map(|p| (p.true_pos - crate::sdf::v3(st.center)).length()).fold(f64::INFINITY, f64::min);
        assert!((5.45..6.0).contains(&nearest), "{}: nearest point {nearest:.2} mm (the Fused export cuts to 5.5)", st.id);
    }
}

#[test]
fn a_tilted_pass_has_a_gap_that_changes_across_the_overlap() {
    let mut s = scene("flat_two_pass_tilt", 2.0);
    s.passes[1].noise_mm = 0.0;
    s.passes[1].noise.hatch_mm = 0.0;
    let scan = scan_pass(&s, 1);
    let gap_at = |x: f64| {
        let v: Vec<f64> = scan.points.iter().filter(|p| p.kind == PointKind::Part && p.true_normal.z > 0.99 && (p.true_pos.x - x).abs() < 1.0).map(|p| p.pos.z).collect();
        v.iter().sum::<f64>() / v.len().max(1) as f64
    };
    let (a, b) = (gap_at(-19.0), gap_at(19.0));
    assert!((a - 0.017).abs() < 0.01 && (b - 0.183).abs() < 0.01, "gap {a:.3} at x = -19, {b:.3} at x = 19");
}

#[test]
fn tables_show_through_the_holes() {
    let s = scene("perforated_grille", 2.0);
    let scan = scan_pass(&s, 0);
    // Table points right under the 3 mm holes (the plate lies on the table at z = -2).
    let under = scan.points.iter().filter(|p| p.kind == PointKind::Table && p.true_pos.x > 8.0 && p.true_pos.x < 37.0 && p.true_pos.y > -14.0 && p.true_pos.y < 14.0).count();
    assert!(under > 30, "{under} table points under the holes");
    assert!(scan.stats.table > scan.stats.part / 2, "{:?}", scan.stats);
}

#[test]
fn written_files_read_back_in_the_part_frame() {
    let out = temp_dir("files");
    let s = scene("flat_vents", 3.0);
    let g = generate(&s, &out, &WriteOptions { reference_mesh: false }).unwrap();
    let truth = Truth::load(&g.dir).unwrap();
    assert!(json_close(&serde_json::to_value(&truth).unwrap(), &serde_json::to_value(&g.truth).unwrap()));
    assert_eq!(truth.passes.len(), 2);
    for pass in &truth.passes {
        let cloud_path = g.dir.join(&pass.cloud);
        let (cloud, mesh) = scancraft_io::load_ply(&cloud_path).unwrap();
        assert!(mesh.is_none());
        assert_eq!(cloud.points.len(), pass.stats.points);
        let labels = std::fs::read(g.dir.join(&pass.labels)).unwrap();
        assert_eq!(labels.len(), cloud.points.len());
        // The marker file is found beside the cloud, as Import Scan looks for it.
        let mkf = scancraft_io::marker_framework_beside(&cloud_path).expect("marker file beside the pass");
        let markers = scancraft_io::read_marker_framework(&mkf).unwrap();
        assert_eq!(markers.len(), pass.listed_markers.len());
        // The part's stickers are on the top: only the top pass lists them (each pass lists the
        // stickers on its own table and marker block).
        let on_part = pass.listed_markers.iter().filter(|m| !m.sticker.starts_with('T')).count();
        assert_eq!(on_part == 0, pass.id == 2, "pass {}: {on_part} part markers", pass.id);
        let to_part = pass.pass_to_part;
        for m in &markers {
            let c = to_part.apply(DVec3::from_array(m.centre));
            let st = s.stickers.iter().min_by(|a, b| (v3(a.center) - c).length().total_cmp(&(v3(b.center) - c).length())).unwrap();
            let bias = pass.spec.depth_bias_mm;
            assert!((v3(st.center) - c).length() < 0.4 + bias, "{}: listed {c}", st.id);
            assert!(to_part.apply_dir(DVec3::from_array(m.normal)).dot(v3(st.normal)) > 0.99);
        }
        // The points are on the true surface once mapped into the part frame.
        let mut far = 0;
        for (p, l) in cloud.points.iter().zip(&labels) {
            if *l != LABEL_PART {
                continue;
            }
            let q = to_part.apply(DVec3::from_array(p.position.map(f64::from)));
            if s.part.sdf(q).abs() > 0.3 + pass.spec.depth_bias_mm {
                far += 1;
            }
        }
        assert!(far * 1000 < cloud.points.len(), "{far} of {} points off the surface", cloud.points.len());
    }
    // Pass 2 is turned over: its points are not in the part frame as written.
    let (cloud2, _) = scancraft_io::load_ply(g.dir.join(&truth.passes[1].cloud)).unwrap();
    let labels2 = std::fs::read(g.dir.join(&truth.passes[1].labels)).unwrap();
    let part2: Vec<f64> = cloud2.points.iter().zip(&labels2).filter(|(_, l)| **l == LABEL_PART).map(|(p, _)| p.position[2] as f64).collect();
    let mean_z = part2.iter().sum::<f64>() / part2.len() as f64;
    assert!(mean_z > 0.5, "turned-over pass rests on z = 0: mean z {mean_z}");
    let _ = std::fs::remove_dir_all(&out);
}

#[test]
fn the_reference_mesh_passes_its_own_probes() {
    let s = scene("flat_vents", 1.0);
    let f = |p: DVec3| s.part.sdf(p);
    let (mesh, cell) = reference_mesh(&f, s.bounds, 0.12);
    assert!(cell <= 0.15);
    let on = mesh.vertices.iter().filter(|v| f(DVec3::from_array(v.position.map(f64::from))).abs() < 1e-3).count();
    assert!(on as f64 > 0.99 * mesh.vertices.len() as f64, "{on} of {}", mesh.vertices.len());
    let truth = Truth {
        format: String::new(),
        scene: s.name.clone(),
        description: String::new(),
        tags: Vec::new(),
        seed: 0,
        units: "mm".into(),
        frames: String::new(),
        part: s.part.clone(),
        bounds: s.bounds,
        reference_mesh: None,
        passes: Vec::new(),
        stickers: Vec::new(),
        features: s.features.clone(),
    };
    let report = crate::score::score(&truth, &mesh);
    assert!(report.distance.p99_mm < 0.01, "{:?}", report.distance);
    for f in &report.features {
        for r in &f.failed {
            // Sharp edges are bevelled by up to about one cell on the reference mesh.
            assert!(matches!(r.check, ProbeKind::On { .. }), "{} '{}' failed on the reference mesh: {:?}", f.id, r.note, r.value);
        }
    }
    assert!(report.probes_passed as f64 >= 0.9 * report.probes as f64, "{} of {}", report.probes_passed, report.probes);
}
