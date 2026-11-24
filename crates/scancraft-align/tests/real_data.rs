//! Real-data regression for "Align all": three Revo Scan 6 scans of one plastic shell part
//! (about 400 x 310 x 140 mm, wall about 3 mm), run through the same pipeline as the
//! `merge_ply` example (import, object isolation, `align_all`, shell-aware refinement).
//!
//! - S1: the outside of the part (no table in the file);
//! - S2, S3: the inside, each standing on a table with marker blocks.
//!
//! S2 shares no surface with S1; it must be placed through S3. Pass criteria, against the
//! verified transforms: S2->S1 and S3->S1 within 1.5 degrees and 3 mm RMS on the moving scan's
//! points, and the inside scans 2.8..3.3 mm behind the outside with less than 1 % poking through.
//!
//! The scans are not in the repository. Point `SCANCRAFT_ALIGN_DATA` at either
//! - a folder with `S1.ply`, `S2.ply`, `S3.ply` (Revo's `raw_preview.ply` of each project) and
//!   optionally `S1_fused.ply`, `S2_fused.ply` (Revo's `pointclouds/Fused*.ply`), or
//! - a text file with one `key = path` line per scan, keys `S1`, `S2`, `S3`, `S1_fused`,
//!   `S2_fused` (blank lines and `#` comments are ignored).
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
use merge_ply::{align_scans, compare, load_scan, matrix_from_rows, read_truth, refinement_problem, report, Options, Outcome, PairStats, Scan, Truth};
use scancraft_align::{rigid, Quality};
use scancraft_core::Transform;

const MAX_ROTATION_DEG: f32 = 1.5;
const MAX_OFFSET_MM: f32 = 3.0;
const WALL_MM: std::ops::RangeInclusive<f32> = 2.8..=3.3;
const MAX_PIERCING: f32 = 0.01;

/// The verified S2->S1 transform (4x4, row-major): S3->S1 composed with S2->S3, checked by the
/// marker stickers and a 3.1 mm wall with nothing poking through.
const S2_TO_S1: &str = "0.635382 0.742112 -0.213446 57.429396  0.549496 -0.628726 -0.550234 178.606454  -0.542534 0.232321 -0.807269 860.223872  0 0 0 1";

/// The verified S3->S1 transform: quaternion (x, y, z, w) and translation, mm.
const S3_TO_S1: ([f32; 4], [f32; 3]) = ([0.4901299, -0.5595679, 0.263982, 0.6139789], [160.2772, 418.6023, 296.6215]);

/// Scan files and the truth folder named by `SCANCRAFT_ALIGN_DATA`; `None` when it is not set.
fn data() -> Option<(HashMap<String, PathBuf>, Option<PathBuf>)> {
    let root = PathBuf::from(std::env::var_os("SCANCRAFT_ALIGN_DATA")?);
    let mut files = HashMap::new();
    let mut truth = None;
    if root.is_dir() {
        for key in ["S1", "S2", "S3", "S1_fused", "S2_fused"] {
            let path = root.join(format!("{key}.ply"));
            if path.is_file() {
                files.insert(key.to_string(), path);
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
    }
    Some((files, truth))
}

fn built_in_truth() -> Truth {
    let (q, t) = S3_TO_S1;
    vec![Some(Transform::default()), matrix_from_rows(S2_TO_S1), Some(rigid(Quat::from_array(q), Vec3::from(t)))]
}

/// Loads the scans `keys` (in that order: they are S1, S2, S3 for the truth) and the truth.
fn setup(keys: [&str; 3], whole_scene: bool) -> Option<(Vec<Scan>, Truth)> {
    let Some((files, truth_dir)) = data() else {
        eprintln!("SCANCRAFT_ALIGN_DATA is not set; skipping");
        return None;
    };
    let mut scans = Vec::new();
    for (k, key) in keys.iter().enumerate() {
        let path = files.get(*key).unwrap_or_else(|| panic!("no {key} in SCANCRAFT_ALIGN_DATA"));
        scans.push(load_scan(path, &format!("S{}", k + 1), whole_scene).expect("load scan"));
    }
    let truth = truth_dir.map_or_else(built_in_truth, |d| read_truth(&d).expect("read truth"));
    Some((scans, truth))
}

/// Everything that misses the pass criteria.
fn failures(scans: &[Scan], outcome: &Outcome, truth: &Truth) -> Vec<String> {
    let mut failed = Vec::new();
    for (from, to, agreement) in compare(scans, &outcome.poses, truth) {
        if to != 0 {
            continue;
        }
        match agreement {
            None => failed.push(format!("S{} is not placed", from + 1)),
            Some(a) if !a.within(MAX_ROTATION_DEG, MAX_OFFSET_MM) => {
                failed.push(format!("S{}->S1 is {:.2} deg / {:.2} mm from the truth", from + 1, a.rotation_deg, a.offset_mm))
            }
            Some(_) => {}
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
fn check(scans: &[Scan], truth: &Truth, reference: usize) {
    let options = Options { reference, ..Options::default() };
    let outcome = align_scans(scans, &options);
    let text = report(scans, &outcome, &options, Some(truth));
    eprintln!("{text}");
    let failed = failures(scans, &outcome, truth);
    assert!(failed.is_empty(), "reference S{}: {}", reference + 1, failed.join("; "));
}

#[test]
#[ignore = "needs the Revo scans (SCANCRAFT_ALIGN_DATA)"]
fn merges_the_revo_part_onto_the_outside_scan() {
    let Some((scans, truth)) = setup(["S1", "S2", "S3"], false) else { return };
    check(&scans, &truth, 0);
}

#[test]
#[ignore = "needs the Revo scans (SCANCRAFT_ALIGN_DATA)"]
fn any_scan_can_be_the_reference() {
    let Some((scans, truth)) = setup(["S1", "S2", "S3"], false) else { return };
    check(&scans, &truth, 1);
    check(&scans, &truth, 2);
}

#[test]
#[ignore = "needs the Revo scans (SCANCRAFT_ALIGN_DATA)"]
fn merges_the_fused_clouds() {
    // Revo has no fused cloud of S3; its raw preview stands in.
    let Some((scans, truth)) = setup(["S1_fused", "S2_fused", "S3"], false) else { return };
    check(&scans, &truth, 0);
}

#[test]
#[ignore = "needs the Revo scans (SCANCRAFT_ALIGN_DATA)"]
fn whole_scene_never_places_a_scan_wrongly() {
    // With the tables and marker blocks left in, the scans need not connect; any scan that is
    // placed must still be placed right.
    let Some((scans, truth)) = setup(["S1", "S2", "S3"], true) else { return };
    let options = Options::default();
    let outcome = align_scans(&scans, &options);
    eprintln!("{}", report(&scans, &outcome, &options, Some(&truth)));
    for (from, to, agreement) in compare(&scans, &outcome.poses, &truth) {
        if let Some(a) = agreement {
            assert!(a.within(MAX_ROTATION_DEG, MAX_OFFSET_MM), "S{}->S{} placed {:.2} deg / {:.2} mm from the truth", from + 1, to + 1, a.rotation_deg, a.offset_mm);
        }
    }
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
