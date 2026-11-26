//! Real-data regression for "Align all": three Revo Scan 6 scans of one plastic shell part
//! (about 400 x 310 x 140 mm, wall about 3 mm), run through the same pipeline as the
//! `merge_ply` example (import, object isolation, `align_all`, shell-aware refinement).
//!
//! - S1: the outside of the part (no table in the file);
//! - S2, S3: the inside, each standing on a table with marker blocks.
//!
//! S2 shares no surface with S1; it must be placed through S3. Pass criteria:
//!
//! - **Markers** (when given): Revo's marker positions of each scan (`marker_framework.mkf`, read
//!   by `scancraft_io::read_marker_framework`) are matched between every two placed scans under
//!   the computed pose (mutual nearest within 3 mm, normals within 30 degrees). Where at least
//!   four markers match, the best rigid fit of the matched markers (Horn, matched again under the
//!   fit) must lie within [`MAX_MARKER_DEG`] and [`MAX_MARKER_MM`] (RMS displacement of the moving
//!   scan's points) of the computed pose, and every placed scan with markers must be in at least
//!   one such pair. A scan without markers (or any scan, when S1 has none) is judged against the
//!   older verified transforms instead: Sk->S1 within 1.5 degrees and 3 mm (they are themselves
//!   about 1.1 degrees / 2-2.5 mm off the marker fit, `docs/architecture/merge-and-ai.md` §3).
//! - **Wall**: the inside scans 2.8..3.3 mm behind the outside with less than 1 % poking through.
//!
//! The scans are not in the repository. Point `SCANCRAFT_ALIGN_DATA` at either
//! - a folder with `S1.ply`, `S2.ply`, `S3.ply` (Revo's `raw_preview.ply` of each project),
//!   optionally `S1_fused.ply`, `S2_fused.ply` (Revo's `pointclouds/Fused*.ply`) and marker files
//!   `<scan>_markers.mkf` (`S1_markers.mkf`, ..., `S1_fused_markers.mkf`), or
//! - a text file with one `key = path` line per file: `S1`, `S2`, `S3`, `S1_fused`, `S2_fused`
//!   for the scans and `<scan>_markers` for their marker files (blank lines and `#` comments are
//!   ignored). A raw scan without a `_markers` line uses the `marker_framework.mkf` in its own
//!   folder (Revo's project folder) if there is one. A fused cloud is in the frame of its
//!   project's raw preview but sits in a sub-folder, so it gets markers only from a line.
//!
//! Verified transforms: `results.csv` and `composed_S2_S1.txt` in the folder (or a `truth = dir`
//! line in the list) when present, otherwise the values built in below. Run in release, one test
//! at a time (each aligns three scans in about a minute):
//!
//! ```text
//! SCANCRAFT_ALIGN_DATA=<folder or list> cargo test -p scancraft-align --release --test real_data -- --ignored --nocapture --test-threads=1
//! ```

#[allow(dead_code)]
#[path = "../examples/merge_ply.rs"]
mod merge_ply;

use std::collections::HashMap;
use std::path::{Path, PathBuf};

use glam::{Quat, Vec3};
use merge_ply::{align_scans, compare, load_scan, matrix_from_rows, read_truth, refinement_problem, relative, report, Options, Outcome, PairStats, Scan, Truth};
use scancraft_align::{compose, horn::horn, probe, rigid, Agreement, Quality};
use scancraft_core::Transform;

/// Against the older verified transforms (only when no markers are given).
const MAX_ROTATION_DEG: f32 = 1.5;
const MAX_OFFSET_MM: f32 = 3.0;
/// Against the marker fit. Measured on 2026-11-26 (`docs/protocol/revo-marker-framework.md`
/// section 3): at most 0.27 deg, and 0.63 mm for the raw scans, 0.81 mm with S1's fused cloud.
/// The limits leave 1.5 to 2 times that and still fail the older verified transforms, which are
/// 1.05-1.15 deg / 2-2.5 mm off.
const MAX_MARKER_DEG: f32 = 0.5;
const MAX_MARKER_MM: f32 = 1.2;
/// Markers of two scans are the same sticker when they lie this close under the pose and face
/// the same way (cosine of the largest angle).
const MARKER_MATCH_MM: f32 = 3.0;
const MARKER_MATCH_COS: f32 = 0.866;
/// Fewer matched markers do not fix a pose well enough to judge one.
const MIN_MARKER_MATCHES: usize = 4;
const WALL_MM: std::ops::RangeInclusive<f32> = 2.8..=3.3;
const MAX_PIERCING: f32 = 0.01;

/// The verified S2->S1 transform (4x4, row-major): S3->S1 composed with S2->S3, checked by the
/// marker stickers and a 3.1 mm wall with nothing poking through.
const S2_TO_S1: &str = "0.635382 0.742112 -0.213446 57.429396  0.549496 -0.628726 -0.550234 178.606454  -0.542534 0.232321 -0.807269 860.223872  0 0 0 1";

/// The verified S3->S1 transform: quaternion (x, y, z, w) and translation, mm.
const S3_TO_S1: ([f32; 4], [f32; 3]) = ([0.4901299, -0.5595679, 0.263982, 0.6139789], [160.2772, 418.6023, 296.6215]);

/// Scan and marker files and the truth folder named by `SCANCRAFT_ALIGN_DATA`; `None` when it is
/// not set.
fn data() -> Option<(HashMap<String, PathBuf>, Option<PathBuf>)> {
    let root = PathBuf::from(std::env::var_os("SCANCRAFT_ALIGN_DATA")?);
    let mut files = HashMap::new();
    let mut truth = None;
    if root.is_dir() {
        for key in ["S1", "S2", "S3", "S1_fused", "S2_fused"] {
            for (k, path) in [(key.to_string(), root.join(format!("{key}.ply"))), (format!("{key}_markers"), root.join(format!("{key}_markers.mkf")))] {
                if path.is_file() {
                    files.insert(k, path);
                }
            }
        }
        truth = root.join("results.csv").is_file().then(|| root.clone());
    } else {
        let text = std::fs::read_to_string(&root).expect("read SCANCRAFT_ALIGN_DATA list");
        let base = root.parent().unwrap_or(Path::new("."));
        for line in text.lines().map(str::trim).filter(|l| !l.is_empty() && !l.starts_with('#')) {
            let (key, value) = line.split_once('=').expect("list lines are `key = path`");
            let path = base.join(value.trim());
            match key.trim() {
                "truth" => truth = Some(path),
                k => {
                    files.insert(k.to_string(), path);
                }
            }
        }
        // Revo's marker file next to a raw scan that has no `_markers` line.
        let scans: Vec<String> = files.keys().filter(|k| !k.contains('_')).cloned().collect();
        for key in scans {
            let marker_key = format!("{key}_markers");
            if let (false, Some(path)) = (files.contains_key(&marker_key), scancraft_io::marker_framework_beside(&files[&key])) {
                files.insert(marker_key, path);
            }
        }
    }
    Some((files, truth))
}

fn built_in_truth() -> Truth {
    let (q, t) = S3_TO_S1;
    vec![Some(Transform::default()), matrix_from_rows(S2_TO_S1), Some(rigid(Quat::from_array(q), Vec3::from(t)))]
}

/// One marker sticker (scan frame, mm): centre and unit normal.
type Marker = (Vec3, Vec3);

/// The scans, their markers (empty where none are given) and the truth.
struct Setup {
    scans: Vec<Scan>,
    markers: Vec<Vec<Marker>>,
    truth: Truth,
}

/// Loads the scans `keys` (in that order: they are S1, S2, S3 for the truth), their markers and
/// the truth.
fn setup(keys: [&str; 3], whole_scene: bool) -> Option<Setup> {
    let Some((files, truth_dir)) = data() else {
        eprintln!("SCANCRAFT_ALIGN_DATA is not set; skipping");
        return None;
    };
    let mut scans = Vec::new();
    let mut markers = Vec::new();
    for (k, key) in keys.iter().enumerate() {
        let path = files.get(*key).unwrap_or_else(|| panic!("no {key} in SCANCRAFT_ALIGN_DATA"));
        scans.push(load_scan(path, &format!("S{}", k + 1), whole_scene).expect("load scan"));
        let list = files.get(&format!("{key}_markers")).map_or_else(Vec::new, |p| {
            let list = scancraft_io::read_marker_framework(p).unwrap_or_else(|e| panic!("{key}_markers: {e}"));
            list.iter().map(|m| (Vec3::from_array(m.centre.map(|v| v as f32)), Vec3::from_array(m.normal.map(|v| v as f32)))).collect()
        });
        markers.push(list);
    }
    for (scan, list) in scans.iter().zip(&markers) {
        eprintln!("{}", marker_surface_report(scan, list));
    }
    let truth = truth_dir.map_or_else(built_in_truth, |d| read_truth(&d).expect("read truth"));
    Some(Setup { scans, markers, truth })
}

/// How a scan's markers sit on its object points: how many lie within 1 mm and how their normals
/// agree with the points' normals around them (a check of the marker file's layout).
fn marker_surface_report(scan: &Scan, markers: &[Marker]) -> String {
    if markers.is_empty() {
        return format!("{}: no markers", scan.name);
    }
    let points = &scan.pass.points;
    let mut on = Vec::new();
    for (c, n) in markers {
        let d2 = points.iter().map(|(p, _)| p.distance_squared(*c)).fold(f32::INFINITY, f32::min);
        if d2 < 1.0 {
            let around = points.iter().filter(|(p, _)| p.distance_squared(*c) < 36.0).map(|(_, m)| *m).sum::<Vec3>().normalize_or_zero();
            on.push((d2.sqrt(), n.dot(around).clamp(-1.0, 1.0).acos().to_degrees()));
        }
    }
    let median = |mut v: Vec<f32>| {
        v.sort_by(f32::total_cmp);
        v.get(v.len() / 2).copied().unwrap_or(f32::NAN)
    };
    format!(
        "{}: {} markers, {} within 1 mm of its object points (median {:.3} mm); normal vs the points' within 6 mm: median {:.2} deg, {} over 90 deg",
        scan.name,
        markers.len(),
        on.len(),
        median(on.iter().map(|o| o.0).collect()),
        median(on.iter().map(|o| o.1).collect()),
        on.iter().filter(|o| o.1 > 90.0).count()
    )
}

/// Markers of `from` and `to` that are the same sticker under `to <- from` = `t`: mutual nearest
/// within [`MARKER_MATCH_MM`], facing the same way. Entries: (from's marker, it moved by `t`,
/// to's marker).
fn match_markers(from: &[Marker], to: &[Marker], t: &Transform) -> Vec<(Vec3, Vec3, Vec3)> {
    let moved: Vec<Marker> = from.iter().map(|(c, n)| (t.transform_point3(*c), t.rotation * *n)).collect();
    let nearest = |c: Vec3, set: &[Marker]| set.iter().enumerate().map(|(k, m)| (m.0.distance(c), k)).fold((f32::INFINITY, usize::MAX), |a, b| if b.0 < a.0 { b } else { a });
    let mut out = Vec::new();
    for (i, (c, n)) in moved.iter().enumerate() {
        let (d, j) = nearest(*c, to);
        if d < MARKER_MATCH_MM && n.dot(to[j].1) > MARKER_MATCH_COS && nearest(to[j].0, &moved).1 == i {
            out.push((from[i].0, *c, to[j].0));
        }
    }
    out
}

/// A pair of scans checked against their shared markers.
struct MarkerCheck {
    from: usize,
    to: usize,
    matched: usize,
    /// RMS distance of the matched markers under the computed pose and under the marker fit, mm.
    rms_pose_mm: f32,
    rms_fit_mm: f32,
    /// The computed pose against the marker fit, on the moving scan's points.
    agreement: Agreement,
}

/// Every pair of placed scans with at least [`MIN_MARKER_MATCHES`] shared markers: the marker fit
/// (Horn on the matches, matched once more under that fit and fitted again) against the pose.
fn marker_checks(scans: &[Scan], markers: &[Vec<Marker>], poses: &[Option<Transform>]) -> Vec<MarkerCheck> {
    let rms = |pairs: &[(Vec3, Vec3)]| (pairs.iter().map(|(a, b)| a.distance_squared(*b)).sum::<f32>() / pairs.len().max(1) as f32).sqrt();
    let mut out = Vec::new();
    for from in 0..scans.len() {
        for to in from + 1..scans.len() {
            let Some(t) = relative(poses, from, to) else { continue };
            let first = match_markers(&markers[from], &markers[to], &t);
            if first.len() < MIN_MARKER_MATCHES {
                continue;
            }
            // to <- to: the correction the first matches ask for, then the matches under it.
            let Some(correction) = horn(&first.iter().map(|(_, a, b)| (*a, *b)).collect::<Vec<_>>()) else { continue };
            let pairs: Vec<(Vec3, Vec3)> = match_markers(&markers[from], &markers[to], &compose(&correction, &t)).iter().map(|(a, _, b)| (*a, *b)).collect();
            if pairs.len() < MIN_MARKER_MATCHES {
                continue;
            }
            let Some(fit) = horn(&pairs) else { continue };
            let under = |x: &Transform| pairs.iter().map(|(a, b)| (x.transform_point3(*a), *b)).collect::<Vec<_>>();
            out.push(MarkerCheck {
                from,
                to,
                matched: pairs.len(),
                rms_pose_mm: rms(&under(&t)),
                rms_fit_mm: rms(&under(&fit)),
                agreement: Agreement::between(&t, &fit, &probe(&scans[from].pass.points, 4000)),
            });
        }
    }
    out
}

fn marker_report(checks: &[MarkerCheck]) -> String {
    let mut s = String::from("marker check (computed pose against the fit of the shared markers):\n");
    for c in checks {
        s += &format!(
            "  S{}->S{}: {} markers, RMS {:.3} mm under the pose, {:.3} mm under the fit; pose vs fit {:.3} deg / {:.3} mm\n",
            c.from + 1,
            c.to + 1,
            c.matched,
            c.rms_pose_mm,
            c.rms_fit_mm,
            c.agreement.rotation_deg,
            c.agreement.offset_mm
        );
    }
    s
}

/// Everything that misses the pass criteria.
fn failures(setup: &Setup, outcome: &Outcome) -> Vec<String> {
    let Setup { scans, markers, truth } = setup;
    let mut failed: Vec<String> = (0..scans.len()).filter(|&k| outcome.poses[k].is_none()).map(|k| format!("S{} is not placed", k + 1)).collect();
    if markers.iter().any(|m| !m.is_empty()) {
        let checks = marker_checks(scans, markers, &outcome.poses);
        eprintln!("{}", marker_report(&checks));
        for c in &checks {
            if !c.agreement.within(MAX_MARKER_DEG, MAX_MARKER_MM) {
                failed.push(format!("S{}->S{} is {:.2} deg / {:.2} mm from its marker fit", c.from + 1, c.to + 1, c.agreement.rotation_deg, c.agreement.offset_mm));
            }
        }
        for k in (0..scans.len()).filter(|&k| outcome.poses[k].is_some() && !markers[k].is_empty()) {
            if !checks.iter().any(|c| c.from == k || c.to == k) {
                failed.push(format!("S{} shares no {MIN_MARKER_MATCHES} markers with another scan under its pose", k + 1));
            }
        }
    }
    // Scans without markers (or against an S1 without them) are judged by the older truth.
    for (from, to, agreement) in compare(scans, &outcome.poses, truth) {
        match agreement {
            Some(a) if to == 0 && (markers[from].is_empty() || markers[0].is_empty()) && !a.within(MAX_ROTATION_DEG, MAX_OFFSET_MM) => {
                failed.push(format!("S{}->S1 is {:.2} deg / {:.2} mm from the truth", from + 1, a.rotation_deg, a.offset_mm))
            }
            _ => {}
        }
    }
    for k in 1..scans.len() {
        let Some(pair) = outcome.pairs.iter().find(|p| p.from == k && p.to == 0) else { continue };
        match pair.quality.shell {
            None => failed.push(format!("S{}->S1 has no wall pairs", k + 1)),
            Some(s) => {
                if !WALL_MM.contains(&s.thickness_mm()) {
                    failed.push(format!("S{}->S1 wall is {:.2} mm", k + 1, s.thickness_mm()));
                }
                if s.piercing >= MAX_PIERCING {
                    failed.push(format!("S{}->S1 pierces the outside with {:.2} % of its points", k + 1, 100.0 * s.piercing));
                }
            }
        }
    }
    failed
}

/// Runs the pipeline with `reference` and asserts the pass criteria.
fn check(setup: &Setup, reference: usize) {
    let options = Options { reference, ..Options::default() };
    let outcome = align_scans(&setup.scans, &options);
    let text = report(&setup.scans, &outcome, &options, Some(&setup.truth));
    eprintln!("{text}");
    let failed = failures(setup, &outcome);
    assert!(failed.is_empty(), "reference S{}: {}", reference + 1, failed.join("; "));
}

#[test]
#[ignore = "needs the Revo scans (SCANCRAFT_ALIGN_DATA)"]
fn merges_the_revo_part_onto_the_outside_scan() {
    let Some(setup) = setup(["S1", "S2", "S3"], false) else { return };
    check(&setup, 0);
}

#[test]
#[ignore = "needs the Revo scans (SCANCRAFT_ALIGN_DATA)"]
fn any_scan_can_be_the_reference() {
    let Some(setup) = setup(["S1", "S2", "S3"], false) else { return };
    check(&setup, 1);
    check(&setup, 2);
}

#[test]
#[ignore = "needs the Revo scans (SCANCRAFT_ALIGN_DATA)"]
fn merges_the_fused_clouds() {
    // Revo has no fused cloud of S3; its raw preview stands in.
    let Some(setup) = setup(["S1_fused", "S2_fused", "S3"], false) else { return };
    check(&setup, 0);
}

#[test]
#[ignore = "needs the Revo scans (SCANCRAFT_ALIGN_DATA)"]
fn whole_scene_never_places_a_scan_wrongly() {
    // With the tables and marker blocks left in, the scans need not connect; any scan that is
    // placed must still be placed right.
    let Some(setup) = setup(["S1", "S2", "S3"], true) else { return };
    let options = Options::default();
    let outcome = align_scans(&setup.scans, &options);
    eprintln!("{}", report(&setup.scans, &outcome, &options, Some(&setup.truth)));
    for (from, to, agreement) in compare(&setup.scans, &outcome.poses, &setup.truth) {
        if let Some(a) = agreement {
            assert!(a.within(MAX_ROTATION_DEG, MAX_OFFSET_MM), "S{}->S{} placed {:.2} deg / {:.2} mm from the truth", from + 1, to + 1, a.rotation_deg, a.offset_mm);
        }
    }
    let checks = marker_checks(&setup.scans, &setup.markers, &outcome.poses);
    eprintln!("{}", marker_report(&checks));
    for c in &checks {
        assert!(c.agreement.within(MAX_MARKER_DEG, MAX_MARKER_MM), "S{}->S{} placed {:.2} deg / {:.2} mm from its marker fit", c.from + 1, c.to + 1, c.agreement.rotation_deg, c.agreement.offset_mm);
    }
}

#[test]
fn shared_markers_measure_how_far_a_pose_is_from_their_fit() {
    // Six stickers on a curved part seen by two scans: the second scan's frame is the first's
    // moved by `truth`, and it lists two stickers of its own.
    let stickers: Vec<Marker> = (0..6)
        .map(|k| {
            let a = k as f32;
            (Vec3::new(80.0 * a.cos(), 60.0 * a.sin(), 300.0 + 10.0 * a), Vec3::new(a.cos(), a.sin(), 0.3).normalize())
        })
        .collect();
    let truth = rigid(Quat::from_rotation_z(0.8) * Quat::from_rotation_x(0.3), Vec3::new(20.0, -40.0, 15.0));
    let inverse = truth.inverse();
    let mut seen: Vec<Marker> = stickers.iter().map(|(c, n)| (inverse.transform_point3(*c), inverse.rotation * *n)).collect();
    seen.extend([(Vec3::new(500.0, 0.0, 0.0), Vec3::Z), (Vec3::new(0.0, 500.0, 0.0), Vec3::Z)]);
    let scan = |name: &str, markers: &[Marker]| Scan {
        name: name.into(),
        path: PathBuf::new(),
        pass: scancraft_align::PassCloud { points: markers.to_vec(), views: Vec::new(), support: None },
        file_points: markers.len(),
        import_s: 0.0,
        segment_s: 0.0,
    };
    // poses: reference <- scan, so S2's pose is the inverse of S2 <- S1.
    let scans = vec![scan("S1", &stickers), scan("S2", &seen)];
    let markers = vec![stickers.clone(), seen.clone()];
    let poses = |s2_from_s1: Transform| [Some(Transform::default()), Some(s2_from_s1.inverse())];
    // The true pose: every sticker matched, nothing to correct.
    let checks = marker_checks(&scans, &markers, &poses(inverse));
    assert_eq!(checks.len(), 1);
    let c = &checks[0];
    assert_eq!((c.from, c.to, c.matched), (0, 1, 6));
    assert!(c.rms_pose_mm < 1e-3 && c.agreement.rotation_deg < 0.01 && c.agreement.offset_mm < 0.01, "{} {}", c.rms_pose_mm, c.agreement.offset_mm);
    // A pose 0.3 degrees and 1 mm off: still matched, and the difference is measured.
    let off = compose(&rigid(Quat::from_rotation_y(0.3f32.to_radians()), Vec3::new(1.0, 0.0, 0.0)), &inverse);
    let checks = marker_checks(&scans, &markers, &poses(off));
    let c = &checks[0];
    assert_eq!(c.matched, 6);
    assert!((c.agreement.rotation_deg - 0.3).abs() < 0.02, "{}", c.agreement.rotation_deg);
    assert!(c.rms_fit_mm < 1e-3 && c.rms_pose_mm > 0.5, "{} {}", c.rms_fit_mm, c.rms_pose_mm);
    // Far off (10 mm): nothing matches, nothing is judged.
    let far = compose(&rigid(Quat::IDENTITY, Vec3::new(10.0, 0.0, 0.0)), &inverse);
    assert!(marker_checks(&scans, &markers, &poses(far)).is_empty());
}

#[test]
fn refinement_must_not_loosen_same_side_contact() {
    let pair = |overlap: f32, rmse_mm: f32| PairStats { from: 0, to: 1, quality: Quality { overlap, rmse_mm, visibility_conflict: 0.0, inliers: 5000, slide_mm: 3.0, ..Quality::NONE } };
    let name = |k: usize| format!("S{}", k + 1);
    assert!(refinement_problem(&[pair(0.3, 0.10)], &[pair(0.3, 0.11)], name).is_none());
    let why = refinement_problem(&[pair(0.3, 0.076)], &[pair(0.3, 0.111)], name).expect("rise noticed");
    assert!(why.contains("S1->S2"), "{why}");
    // Pairs that meet only at a wall (little same-side overlap) are not watched for RMS.
    assert!(refinement_problem(&[pair(0.005, 0.35)], &[pair(0.005, 0.60)], name).is_none());
}

#[test]
fn built_in_truth_parses() {
    let truth = built_in_truth();
    assert_eq!(truth.len(), 3);
    let s2 = truth[1].expect("S2->S1 parses");
    // The last column of the row-major matrix is the translation.
    assert!((s2.translation - Vec3::new(57.4294, 178.6065, 860.2239)).length() < 1e-3);
    assert!((s2.rotation.length() - 1.0).abs() < 1e-5);
    assert!(matrix_from_rows("1 2 3").is_none());
}
