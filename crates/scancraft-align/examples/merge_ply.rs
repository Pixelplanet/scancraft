//! Merges several scans of one object end to end, the way the app's "Align all" does: import
//! (PLY / OBJ / STL), object isolation (table and fragments removed), automatic alignment of all
//! scans to a reference ([`align_all`]), shell-aware joint refinement ([`refine_global`]) and
//! merging. Writes the result and reports so it can be checked by eye and by numbers.
//!
//! ```text
//! cargo run -p scancraft-align --release --example merge_ply -- <out_dir> <scan>... [options]
//!   --reference N   scan the others are aligned to (1-based, default 1)
//!   --whole-scene   align the scans as they are (no object isolation)
//!   --no-refine     keep the poses of the alignment graph
//!   --pitch MM      surfel pitch of merged.ply (default 0.7)
//!   --truth DIR     compare with verified transforms: DIR/results.csv (row "S3->S1") and
//!                   DIR/composed_S2_S1.txt; the scans must be given in the order S1 S2 S3
//! ```
//!
//! Outputs in `<out_dir>`:
//! - `merged.ply`: all scans fused into one cloud (positions, normals);
//! - `merged_by_scan.ply`: every scan's object points in the reference frame, one colour per scan
//!   (scan 1 grey, 2 blue, 3 orange, 4 green, ...);
//! - `view_top.png`, `view_side.png`, `section_across.png`, `section_along.png`: views of
//!   `merged_by_scan` and 2 mm slices through it (the slices show the wall);
//! - `edges.csv`: every measured pair and every composed pair of the alignment graph;
//! - `transforms.csv`: reference <- scan for every scan (quaternion, translation, 4x4 row-major),
//!   from the graph and after refinement;
//! - `pairs.csv`: every ordered pair at the final poses: same-side overlap and the signed offset
//!   between facing-apart surfaces (wall thickness, IQR, piercing);
//! - `report.txt`: all of the above in words, with timings (also printed).

use std::fmt::Write as _;
use std::io::Write as _;
use std::path::{Path, PathBuf};
use std::time::Instant;

use glam::{Mat4, Quat, Vec3};
use rayon::prelude::*;
use scancraft_align::cloud::transform_oriented;
use scancraft_align::graph::{align_all, GraphConfig, GraphResult};
use scancraft_align::quality::QualityConfig;
use scancraft_align::{compose, evaluate, merge, probe, refine_global, rigid, Agreement, MergeConfig, MergeInput, Oriented, PassCloud, Plane, Quality, RefineConfig, RefineInput, RefineResult, Verdict};
use scancraft_core::Transform;
use scancraft_fusion::segment::{segment_object, SegmentConfig, SegmentLabel};
use scancraft_io::{import_scan, UnitsHint};

/// Colours of the scans in `merged_by_scan.ply` and the views.
const COLOURS: [[u8; 3]; 6] = [[200, 200, 200], [70, 130, 230], [240, 140, 40], [90, 200, 110], [220, 80, 160], [230, 220, 70]];

/// One imported scan, ready to align.
pub struct Scan {
    pub name: String,
    pub path: PathBuf,
    /// Object points (or the whole scene), with the table plane when one was found.
    pub pass: PassCloud,
    /// Points in the file.
    pub file_points: usize,
    pub import_s: f32,
    pub segment_s: f32,
}

/// Reads a scan and isolates the object (unless `whole_scene`). Points without a normal are
/// dropped: alignment needs normals. Files in metres are scaled to mm.
pub fn load_scan(path: &Path, name: &str, whole_scene: bool) -> Result<Scan, String> {
    let t0 = Instant::now();
    let mut imported = import_scan(path).map_err(|e| format!("{}: {e}", path.display()))?;
    if imported.units == UnitsHint::Metres {
        imported.scale(UnitsHint::Metres.to_mm());
    }
    let points: Vec<Vec3> = imported.points.iter().map(|p| Vec3::from(*p)).collect();
    let normals: Vec<Vec3> = match &imported.normals {
        Some(n) => n.iter().map(|v| Vec3::from(*v)).collect(),
        None => return Err(format!("{}: no normals", path.display())),
    };
    let import_s = t0.elapsed().as_secs_f32();

    let t1 = Instant::now();
    let (keep, support): (Vec<bool>, Option<Plane>) = if whole_scene {
        (vec![true; points.len()], None)
    } else {
        let seg = segment_object(&points, Some(&normals), &SegmentConfig::default(), None, None);
        (seg.labels.iter().map(|l| *l == SegmentLabel::Object).collect(), seg.support.map(|s| Plane::through(s.point, s.normal)))
    };
    let segment_s = t1.elapsed().as_secs_f32();
    let oriented: Vec<Oriented> = points.iter().zip(&normals).zip(&keep).filter(|(_, k)| **k).map(|((p, n), _)| (*p, *n)).filter(|(p, n)| p.is_finite() && n.length_squared() > 0.5).collect();
    Ok(Scan { name: name.to_string(), path: path.to_path_buf(), pass: PassCloud { points: oriented, views: Vec::new(), support }, file_points: points.len(), import_s, segment_s })
}

/// Settings of one merge run.
#[derive(Debug, Clone)]
pub struct Options {
    /// Index of the reference scan (0-based).
    pub reference: usize,
    pub refine: bool,
    pub graph: GraphConfig,
    pub refine_config: RefineConfig,
}

impl Default for Options {
    fn default() -> Self {
        Options { reference: 0, refine: true, graph: GraphConfig::default(), refine_config: RefineConfig { sample_mm: 1.0, shell: true, huber_mm: Some(1.0), ..RefineConfig::default() } }
    }
}

/// Facing-apart and same-side statistics of one ordered pair at the final poses.
#[derive(Debug, Clone, Copy)]
pub struct PairStats {
    pub from: usize,
    pub to: usize,
    /// `from` moved onto `to`.
    pub quality: Quality,
}

/// The result of [`align_scans`].
pub struct Outcome {
    pub graph: GraphResult,
    /// reference <- scan from the graph.
    pub graph_poses: Vec<Option<Transform>>,
    /// reference <- scan used for the outputs (refined, unless [`refinement_problem`] objects).
    pub poses: Vec<Option<Transform>>,
    pub refine: Option<RefineResult>,
    /// reference <- scan after refinement (also when not used); `None` without refinement.
    pub refined_poses: Option<Vec<Option<Transform>>>,
    /// Why the refined poses were not used, when refinement ran.
    pub refine_rejected: Option<String>,
    /// Every ordered pair at the graph poses and at the final poses.
    pub graph_pairs: Vec<PairStats>,
    pub pairs: Vec<PairStats>,
    pub align_s: f32,
    pub refine_s: f32,
}

/// Aligns all scans to `options.reference`, refines the connected ones jointly and measures every
/// pair at the result.
pub fn align_scans(scans: &[Scan], options: &Options) -> Outcome {
    let passes: Vec<PassCloud> = scans.iter().map(|s| s.pass.clone()).collect();
    let t0 = Instant::now();
    let graph = align_all(&passes, options.reference, &options.graph);
    let align_s = t0.elapsed().as_secs_f32();
    let graph_poses = graph.poses.clone();

    let t1 = Instant::now();
    let mut poses = graph_poses.clone();
    let graph_pairs = pair_stats(&passes, &poses);
    let mut refine = None;
    let mut refined_poses = None;
    let mut refine_rejected = None;
    let connected: Vec<usize> = (0..scans.len()).filter(|&k| poses[k].is_some()).collect();
    if options.refine && connected.len() > 1 {
        // Refine in a frame centred on the object, so the result does not depend on where the
        // reference scan's origin happens to be.
        let probes: Vec<Vec3> = connected.iter().flat_map(|&k| probe(&passes[k].points, 2000).into_iter().map(|p| poses[k].unwrap().transform_point3(p)).collect::<Vec<_>>()).collect();
        let centre = probes.iter().copied().sum::<Vec3>() / probes.len().max(1) as f32;
        let to_centred = rigid(Quat::IDENTITY, -centre);
        let inputs: Vec<RefineInput> = connected.iter().map(|&k| RefineInput { cloud: &passes[k], pose: compose(&to_centred, &poses[k].unwrap()), fixed: k == options.reference }).collect();
        let result = refine_global(&inputs, &options.refine_config);
        let mut refined = poses.clone();
        for (i, &k) in connected.iter().enumerate() {
            refined[k] = Some(compose(&to_centred.inverse(), &result.poses[i]));
        }
        let after = pair_stats(&passes, &refined);
        refine_rejected = refinement_problem(&graph_pairs, &after, |k| scans[k].name.clone());
        if refine_rejected.is_none() {
            poses = refined.clone();
        }
        refined_poses = Some(refined);
        refine = Some(result);
    }
    let refine_s = t1.elapsed().as_secs_f32();
    let pairs = pair_stats(&passes, &poses);
    Outcome { graph, graph_poses, poses, refine, refined_poses, refine_rejected, graph_pairs, pairs, align_s, refine_s }
}

/// Pairs whose same-side overlap is at least this are watched by [`refinement_problem`].
const WATCHED_OVERLAP: f32 = 0.1;

/// Why refined poses (`after`) are worse than the graph's (`before`, same pairs in the same
/// order), or `None`. Refinement must not make a plausible pair Poor, and must not loosen the
/// fit of surfaces seen from the same side (RMS up by more than 10 % and 0.02 mm): evening out
/// the wall alone is no gain, since a real wall is not equally thick everywhere.
pub fn refinement_problem(before: &[PairStats], after: &[PairStats], name: impl Fn(usize) -> String) -> Option<String> {
    for (b, a) in before.iter().zip(after) {
        let (qb, qa) = (&b.quality, &a.quality);
        let pair = format!("{}->{}", name(b.from), name(b.to));
        if qb.verdict() != Verdict::Poor && qa.verdict() == Verdict::Poor {
            return Some(format!("{pair} became Poor"));
        }
        if qb.overlap >= WATCHED_OVERLAP && qa.rmse_mm > (1.1 * qb.rmse_mm).max(qb.rmse_mm + 0.02) {
            return Some(format!("{pair} same-side RMS rose from {:.3} to {:.3} mm", qb.rmse_mm, qa.rmse_mm));
        }
    }
    None
}

/// `to <- from` from two reference <- scan poses.
pub fn relative(poses: &[Option<Transform>], from: usize, to: usize) -> Option<Transform> {
    Some(compose(&poses[to]?.inverse(), &poses[from]?))
}

/// Every ordered pair of placed passes, scored on a 1 mm grid.
pub fn pair_stats(passes: &[PassCloud], poses: &[Option<Transform>]) -> Vec<PairStats> {
    let n = passes.len();
    let jobs: Vec<(usize, usize)> = (0..n).flat_map(|a| (0..n).filter(move |&b| b != a).map(move |b| (a, b))).collect();
    jobs.par_iter()
        .filter_map(|&(from, to)| {
            let t = relative(poses, from, to)?;
            Some(PairStats { from, to, quality: evaluate(&passes[from], &passes[to], &t, QualityConfig::for_sample(1.0)) })
        })
        .collect()
}

/// Verified transforms of the scans to the first one (`S1 <- Sk`), `None` where unknown.
pub type Truth = Vec<Option<Transform>>;

/// Reads the verified transforms of the Revo test part: `S3->S1` from `results.csv` (columns
/// qx, qy, qz, qw, tx, ty, tz; p' = R p + t) and `composed_S2_S1.txt` (4x4, row-major).
pub fn read_truth(dir: &Path) -> Result<Truth, String> {
    let csv = std::fs::read_to_string(dir.join("results.csv")).map_err(|e| format!("results.csv: {e}"))?;
    let mut lines = csv.lines();
    let head: Vec<&str> = lines.next().unwrap_or_default().split(',').collect();
    let row: Vec<&str> = lines.find(|l| l.starts_with("S3->S1,")).ok_or("results.csv: no S3->S1 row")?.split(',').collect();
    let col = |name: &str| -> Result<f32, String> {
        let i = head.iter().position(|h| *h == name).ok_or(format!("results.csv: no column {name}"))?;
        row.get(i).and_then(|v| v.parse().ok()).ok_or(format!("results.csv: bad {name}"))
    };
    let s3 = rigid(Quat::from_xyzw(col("qx")?, col("qy")?, col("qz")?, col("qw")?), Vec3::new(col("tx")?, col("ty")?, col("tz")?));
    let text = std::fs::read_to_string(dir.join("composed_S2_S1.txt")).map_err(|e| format!("composed_S2_S1.txt: {e}"))?;
    let s2 = matrix_from_rows(&text).ok_or("composed_S2_S1.txt: expected 16 numbers")?;
    Ok(vec![Some(Transform::default()), Some(s2), Some(s3)])
}

/// A rigid transform from 16 numbers, a 4x4 matrix row by row.
pub fn matrix_from_rows(text: &str) -> Option<Transform> {
    let v: Vec<f32> = text.split_whitespace().filter_map(|s| s.parse().ok()).collect();
    let m = Mat4::from_cols_array(v.get(..16)?.try_into().ok()?).transpose();
    let (_, r, t) = m.to_scale_rotation_translation();
    Some(rigid(r, t))
}

/// How far the found transforms are from the truth (whatever the reference): rotation difference
/// and RMS displacement of the moving scan's points, for every `Sk -> S1` first, then for the
/// pairs without S1 (`Si -> Sj`, i < j). Entries: (from, to, `None` when not placed).
pub fn compare(scans: &[Scan], poses: &[Option<Transform>], truth: &Truth) -> Vec<(usize, usize, Option<Agreement>)> {
    let n = scans.len().min(truth.len());
    let pairs = (1..n).map(|k| (k, 0)).chain((1..n).flat_map(|i| (i + 1..n).map(move |j| (i, j))));
    pairs
        .filter_map(|(from, to)| {
            let want = relative(truth, from, to)?;
            let got = relative(poses, from, to);
            Some((from, to, got.map(|g| Agreement::between(&g, &want, &probe(&scans[from].pass.points, 4000)))))
        })
        .collect()
}

fn quality_text(q: &Quality) -> String {
    let shell = q.shell.map_or("no shell pairs".to_string(), |s| {
        format!("shell {} pairs ({:.1} %), offset {:+.2} mm, IQR {:.2} mm, piercing {:.2} %, wall {}", s.pairs, 100.0 * s.fraction, s.offset_mm, s.iqr_mm, 100.0 * s.piercing, s.is_wall())
    });
    format!(
        "overlap {:.3}, rmse {:.3} mm, {shell}; below table {:.2} %, slide {:.1} mm, score {:.4}, {}",
        q.overlap,
        q.rmse_mm,
        100.0 * q.below_support,
        q.slide_mm,
        q.score(),
        q.verdict().label()
    )
}

const QUALITY_HEAD: &str = "overlap,rmse_mm,shell_pairs,shell_fraction,offset_mm,iqr_mm,piercing,wall,below_support,slide_mm,score,verdict";

fn quality_csv(q: &Quality) -> String {
    let s = q.shell.map_or(",,,,,".to_string(), |s| format!("{},{:.4},{:.3},{:.3},{:.5},{}", s.pairs, s.fraction, s.offset_mm, s.iqr_mm, s.piercing, s.is_wall()));
    format!("{:.4},{:.4},{s},{:.5},{:.2},{:.4},{}", q.overlap, q.rmse_mm, q.below_support, q.slide_mm, q.score(), q.verdict().label())
}

fn transform_csv(t: &Transform) -> String {
    let (q, p) = (t.rotation, t.translation);
    let m = t.to_mat4().transpose().to_cols_array();
    let m: Vec<String> = m.iter().map(|v| format!("{v:.6}")).collect();
    format!("{:.7},{:.7},{:.7},{:.7},{:.4},{:.4},{:.4},{}", q.x, q.y, q.z, q.w, p.x, p.y, p.z, m.join(","))
}

/// `edges.csv`: measured pairs (with their state in the graph) and composed pairs.
fn edges_csv(scans: &[Scan], graph: &GraphResult) -> String {
    let mut s = format!("kind,from,to,state,verdict,score,consistent,agree_rot_deg,agree_offset_mm,path,{QUALITY_HEAD},{}\n", QUALITY_HEAD.split(',').map(|h| format!("rev_{h}")).collect::<Vec<_>>().join(","));
    for e in &graph.edges {
        let (rot, off) = e.agreement.map_or((String::new(), String::new()), |a| (format!("{:.3}", a.rotation_deg), format!("{:.3}", a.offset_mm)));
        let _ = writeln!(
            s,
            "measured,{},{},{:?},{},{:.4},{},{rot},{off},,{},{}",
            scans[e.from].name,
            scans[e.to].name,
            e.state,
            e.verdict.label(),
            e.score,
            e.consistent,
            quality_csv(&e.quality),
            quality_csv(&e.reverse)
        );
    }
    for c in &graph.composed {
        let path: Vec<&str> = c.path.iter().map(|&k| scans[k].name.as_str()).collect();
        let q = c.quality.map_or(",".repeat(QUALITY_HEAD.matches(',').count()), |q| quality_csv(&q));
        let _ = writeln!(s, "composed,{},{},composed,,,,,,{},{q},", scans[c.from].name, scans[c.to].name, path.join(" "));
    }
    s
}

/// `transforms.csv`: reference <- scan from the graph and as used.
fn transforms_csv(scans: &[Scan], outcome: &Outcome) -> String {
    let mut s = String::from("scan,file,points,stage,qx,qy,qz,qw,tx,ty,tz,m00,m01,m02,m03,m10,m11,m12,m13,m20,m21,m22,m23,m30,m31,m32,m33\n");
    for (stage, poses) in [("graph", &outcome.graph_poses), ("final", &outcome.poses)] {
        for (k, scan) in scans.iter().enumerate() {
            let file = scan.path.file_name().map(|f| f.to_string_lossy().into_owned()).unwrap_or_default();
            let t = poses[k].map_or(String::new(), |t| transform_csv(&t));
            let _ = writeln!(s, "{},{file},{},{stage},{t}", scan.name, scan.pass.points.len());
        }
    }
    s
}

fn pairs_csv(scans: &[Scan], pairs: &[PairStats]) -> String {
    let mut s = format!("from,to,{QUALITY_HEAD}\n");
    for p in pairs {
        let _ = writeln!(s, "{},{},{}", scans[p.from].name, scans[p.to].name, quality_csv(&p.quality));
    }
    s
}

/// The human-readable report.
pub fn report(scans: &[Scan], outcome: &Outcome, options: &Options, truth: Option<&Truth>) -> String {
    let mut s = String::new();
    let name = |k: usize| scans[k].name.as_str();
    for scan in scans {
        let _ = writeln!(
            s,
            "{}: {} of {} points kept, table {}; import {:.2} s, isolation {:.2} s ({})",
            scan.name,
            scan.pass.points.len(),
            scan.file_points,
            if scan.pass.support.is_some() { "found" } else { "none" },
            scan.import_s,
            scan.segment_s,
            scan.path.display()
        );
    }
    let g = &outcome.graph;
    let _ = writeln!(s, "\nalign_all, reference {}: {:.1} s", name(g.reference), outcome.align_s);
    for e in &g.edges {
        let _ = writeln!(s, "edge {}->{}: {:?}, {}, score {:.4}, consistent {}, agreement {:?}", name(e.from), name(e.to), e.state, e.verdict.label(), e.score, e.consistent, e.agreement);
        let _ = writeln!(s, "    forward: {}", quality_text(&e.quality));
        let _ = writeln!(s, "    reverse: {}", quality_text(&e.reverse));
    }
    for c in &g.composed {
        let path: Vec<&str> = c.path.iter().map(|&k| name(k)).collect();
        let _ = writeln!(s, "composed {}->{} via {}: {}", name(c.from), name(c.to), path.join("-"), c.quality.as_ref().map_or("not scored".into(), quality_text));
    }
    for c in &g.cycles {
        let path: Vec<&str> = c.passes.iter().map(|&k| name(k)).collect();
        let _ = writeln!(s, "cycle {}: {:.2} deg / {:.2} mm, closed {}", path.join("-"), c.residual.rotation_deg, c.residual.offset_mm, c.closed);
    }
    if !g.unconnected.is_empty() {
        let names: Vec<&str> = g.unconnected.iter().map(|&k| name(k)).collect();
        let _ = writeln!(s, "not connected: {}", names.join(", "));
    }
    if let Some(r) = &outcome.refine {
        let _ = writeln!(
            s,
            "\nrefine_global (shell {}, huber {:?}, sample {} mm): {:.1} s, link residual RMS {:.2} -> {:.2} mm{}",
            options.refine_config.shell,
            options.refine_config.huber_mm,
            options.refine_config.sample_mm,
            outcome.refine_s,
            r.rms_before(),
            r.rms_after(),
            outcome.refine_rejected.as_ref().map_or(String::new(), |why| format!("; NOT USED: {why}"))
        );
        let connected: Vec<usize> = (0..scans.len()).filter(|&k| outcome.graph_poses[k].is_some()).collect();
        for e in &r.edges {
            let _ = writeln!(
                s,
                "    link {}-{}: overlap {:.2}, rmse {:.3} mm, residual {:.2} -> {:.2} mm, wall {}",
                name(connected[e.a]),
                name(connected[e.b]),
                e.overlap,
                e.rmse_mm,
                e.residual_before_mm,
                e.residual_after_mm,
                e.thickness_mm.map_or("-".into(), |t| format!("{t:.2} mm"))
            );
        }
        for &k in &connected {
            if let (Some(a), Some(b)) = (outcome.graph_poses[k], outcome.refined_poses.as_ref().and_then(|r| r[k])) {
                let d = Agreement::between(&a, &b, &probe(&scans[k].pass.points, 4000));
                let _ = writeln!(s, "    {} moved by refinement: {:.3} deg / {:.2} mm", name(k), d.rotation_deg, d.offset_mm);
            }
        }
    }
    for (stage, pairs) in [("graph", &outcome.graph_pairs), ("final", &outcome.pairs)] {
        let _ = writeln!(s, "\npairs at the {stage} poses (moved onto):");
        for p in pairs {
            let _ = writeln!(s, "    {}->{}: {}", name(p.from), name(p.to), quality_text(&p.quality));
        }
    }
    if let Some(truth) = truth {
        let _ = writeln!(s, "\nagainst the verified transforms (Sk -> S1):");
        let mut stages = vec![("graph", &outcome.graph_poses)];
        if let Some(r) = &outcome.refined_poses {
            stages.push(("refined", r));
        }
        stages.push(("final", &outcome.poses));
        for (stage, poses) in stages {
            for (from, to, a) in compare(scans, poses, truth) {
                let text = a.map_or("not placed".to_string(), |a| format!("{:.3} deg, {:.2} mm RMS", a.rotation_deg, a.offset_mm));
                let _ = writeln!(s, "    {stage:7} {}->{}: {text}", name(from), name(to));
            }
        }
        let passes: Vec<PassCloud> = scans.iter().map(|s| s.pass.clone()).collect();
        let _ = writeln!(s, "pairs at the verified transforms (same scoring as above):");
        let m = truth.len().min(scans.len());
        for p in pair_stats(&passes[..m], &truth[..m]) {
            let _ = writeln!(s, "    {}->{}: {}", name(p.from), name(p.to), quality_text(&p.quality));
        }
    }
    s
}

/// Writes a binary PLY with positions, normals and colours.
pub fn write_ply(path: &Path, points: &[(Vec3, Vec3, [u8; 3])]) -> std::io::Result<()> {
    let mut f = std::io::BufWriter::new(std::fs::File::create(path)?);
    write!(
        f,
        "ply\nformat binary_little_endian 1.0\ncomment ScanCraft merge_ply\nelement vertex {}\nproperty float x\nproperty float y\nproperty float z\nproperty float nx\nproperty float ny\nproperty float nz\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n",
        points.len()
    )?;
    for (p, n, c) in points {
        for v in [p.x, p.y, p.z, n.x, n.y, n.z] {
            f.write_all(&v.to_le_bytes())?;
        }
        f.write_all(c)?;
    }
    f.flush()
}

/// The placed scans in the reference frame with their colours.
pub fn placed_layers(scans: &[Scan], poses: &[Option<Transform>]) -> Vec<(Vec<Oriented>, [u8; 3])> {
    scans.iter().enumerate().filter_map(|(k, s)| Some((transform_oriented(&s.pass.points, &poses[k]?), COLOURS[k % COLOURS.len()]))).collect()
}

/// Orthographic view of coloured layers: image axes `right`, `up`, looking along -`towards`
/// (nearest to the viewer wins); only points passing `slab` are drawn.
fn render(layers: &[(Vec<Oriented>, [u8; 3])], centre: Vec3, axes: [Vec3; 3], px_per_mm: f32, half: (f32, f32), slab: &dyn Fn(Vec3) -> bool) -> image::RgbImage {
    let [right, up, towards] = axes;
    let (w, h) = ((2.0 * half.0 * px_per_mm).max(1.0) as u32, (2.0 * half.1 * px_per_mm).max(1.0) as u32);
    let mut img = image::RgbImage::from_pixel(w, h, image::Rgb([24, 24, 28]));
    let mut depth = vec![f32::NEG_INFINITY; (w * h) as usize];
    for (pts, colour) in layers {
        for &(p, n) in pts.iter().filter(|(p, _)| slab(*p)) {
            let d = p - centre;
            let (x, y, z) = (((d.dot(right) + half.0) * px_per_mm) as i32, ((half.1 - d.dot(up)) * px_per_mm) as i32, d.dot(towards));
            let shade = 0.35 + 0.65 * n.dot(towards).abs();
            let px = image::Rgb(colour.map(|c| (c as f32 * shade).min(255.0) as u8));
            for (xx, yy) in [(x, y), (x + 1, y), (x, y + 1), (x + 1, y + 1)] {
                if xx < 0 || yy < 0 || xx >= w as i32 || yy >= h as i32 {
                    continue;
                }
                let i = (yy as u32 * w + xx as u32) as usize;
                if z > depth[i] {
                    depth[i] = z;
                    img.put_pixel(xx as u32, yy as u32, px);
                }
            }
        }
    }
    img
}

/// Top and side views and two 2 mm slices through the middle, `up` being the table normal.
pub fn write_views(out: &Path, layers: &[(Vec<Oriented>, [u8; 3])], up: Vec3) -> image::ImageResult<()> {
    let all: Vec<Vec3> = layers.iter().flat_map(|l| l.0.iter().map(|p| p.0)).collect();
    if all.is_empty() {
        return Ok(());
    }
    let centre = all.iter().copied().sum::<Vec3>() / all.len() as f32;
    // Long axis: main direction of the points within the table plane.
    let a = up.any_orthonormal_vector();
    let b = up.cross(a);
    let (mut sxx, mut sxy, mut syy) = (0.0f32, 0.0f32, 0.0f32);
    for p in &all {
        let (x, y) = ((*p - centre).dot(a), (*p - centre).dot(b));
        sxx += x * x;
        sxy += x * y;
        syy += y * y;
    }
    let angle = 0.5 * (2.0 * sxy).atan2(sxx - syy);
    let long = a * angle.cos() + b * angle.sin();
    let side = up.cross(long);
    let extent = |axis: Vec3| all.iter().map(|p| (*p - centre).dot(axis).abs()).fold(0.0f32, f32::max).min(2000.0) + 10.0;
    let (el, es, eu) = (extent(long), extent(side), extent(up));
    let px = (2400.0 / (2.0 * el)).clamp(0.5, 3.0);
    let any = |_: Vec3| true;
    render(layers, centre, [long, side, up], px, (el, es), &any).save(out.join("view_top.png"))?;
    render(layers, centre, [long, up, -side], px, (el, eu), &any).save(out.join("view_side.png"))?;
    let across = |p: Vec3| (p - centre).dot(long).abs() < 1.0;
    render(layers, centre, [side, up, long], 2.0 * px, (es, eu), &across).save(out.join("section_across.png"))?;
    let along = |p: Vec3| (p - centre).dot(side).abs() < 1.0;
    render(layers, centre, [long, up, side], 2.0 * px, (el, eu), &along).save(out.join("section_along.png"))
}

/// Writes every output file of a run.
pub fn write_outputs(out: &Path, scans: &[Scan], outcome: &Outcome, pitch_mm: f32, report_text: &str) -> Result<String, String> {
    let t0 = Instant::now();
    std::fs::create_dir_all(out).map_err(|e| e.to_string())?;
    let layers = placed_layers(scans, &outcome.poses);
    let coloured: Vec<(Vec3, Vec3, [u8; 3])> = layers.iter().flat_map(|(pts, c)| pts.iter().map(move |&(p, n)| (p, n, *c))).collect();
    write_ply(&out.join("merged_by_scan.ply"), &coloured).map_err(|e| e.to_string())?;
    let inputs: Vec<MergeInput> = layers.iter().map(|(pts, _)| MergeInput { points: pts.iter().map(|&(p, n)| (p, n, 1)).collect(), weight: 1.0 }).collect();
    let merged = merge(&inputs, &MergeConfig { pitch_mm, ..MergeConfig::default() });
    let fused: Vec<(Vec3, Vec3, [u8; 3])> = merged.points.iter().map(|p| (Vec3::from(p.position), Vec3::from(p.normal), [p.color.r, p.color.g, p.color.b])).collect();
    write_ply(&out.join("merged.ply"), &fused).map_err(|e| e.to_string())?;
    // "Up" for the views: the first table plane found, in the reference frame.
    let up = scans.iter().enumerate().find_map(|(k, s)| Some(s.pass.support?.transformed(&outcome.poses[k]?).normal)).unwrap_or(Vec3::Z);
    write_views(out, &layers, up).map_err(|e| e.to_string())?;
    std::fs::write(out.join("edges.csv"), edges_csv(scans, &outcome.graph)).map_err(|e| e.to_string())?;
    std::fs::write(out.join("transforms.csv"), transforms_csv(scans, outcome)).map_err(|e| e.to_string())?;
    std::fs::write(out.join("pairs.csv"), pairs_csv(scans, &outcome.pairs)).map_err(|e| e.to_string())?;
    let summary = format!(
        "\nmerged: {} scan points -> {} points at {pitch_mm} mm ({} isolated, {} outliers); writing {:.1} s\n",
        merged.input_points,
        fused.len(),
        merged.isolated,
        merged.outliers,
        t0.elapsed().as_secs_f32()
    );
    std::fs::write(out.join("report.txt"), format!("{report_text}{summary}")).map_err(|e| e.to_string())?;
    Ok(summary)
}

struct Args {
    out: PathBuf,
    scans: Vec<PathBuf>,
    whole_scene: bool,
    reference: usize,
    refine: bool,
    pitch_mm: f32,
    truth: Option<PathBuf>,
}

fn parse_args() -> Result<Args, String> {
    let mut it = std::env::args().skip(1);
    let mut args = Args { out: PathBuf::new(), scans: Vec::new(), whole_scene: false, reference: 1, refine: true, pitch_mm: 0.7, truth: None };
    let mut positional = Vec::new();
    while let Some(a) = it.next() {
        match a.as_str() {
            "--whole-scene" => args.whole_scene = true,
            "--no-refine" => args.refine = false,
            "--reference" => args.reference = it.next().and_then(|v| v.parse().ok()).ok_or("--reference needs a scan number (1-based)")?,
            "--pitch" => args.pitch_mm = it.next().and_then(|v| v.parse().ok()).ok_or("--pitch needs mm")?,
            "--truth" => args.truth = Some(it.next().ok_or("--truth needs a folder")?.into()),
            _ if a.starts_with("--") => return Err(format!("unknown option {a}")),
            _ => positional.push(PathBuf::from(a)),
        }
    }
    if positional.len() < 3 {
        return Err("usage: merge_ply <out_dir> <scan> <scan>... [--reference N] [--whole-scene] [--no-refine] [--pitch MM] [--truth DIR]".into());
    }
    args.out = positional.remove(0);
    args.scans = positional;
    if args.reference == 0 || args.reference > args.scans.len() {
        return Err(format!("--reference must be 1..{}", args.scans.len()));
    }
    Ok(args)
}

fn main() {
    env_logger::init();
    let args = match parse_args() {
        Ok(a) => a,
        Err(e) => {
            eprintln!("{e}");
            std::process::exit(2);
        }
    };
    let t0 = Instant::now();
    let mut scans = Vec::new();
    for (k, path) in args.scans.iter().enumerate() {
        match load_scan(path, &format!("S{}", k + 1), args.whole_scene) {
            Ok(s) => scans.push(s),
            Err(e) => {
                eprintln!("{e}");
                std::process::exit(1);
            }
        }
    }
    let truth = args.truth.as_deref().map(|d| read_truth(d).unwrap_or_else(|e| panic!("truth: {e}")));
    let options = Options { reference: args.reference - 1, refine: args.refine, ..Options::default() };
    let outcome = align_scans(&scans, &options);
    let mut text = report(&scans, &outcome, &options, truth.as_ref());
    let _ = writeln!(text, "\nload + align + refine: {:.1} s{}", t0.elapsed().as_secs_f32(), if args.whole_scene { " (whole scene)" } else { "" });
    print!("{text}");
    match write_outputs(&args.out, &scans, &outcome, args.pitch_mm, &text) {
        Ok(summary) => println!("{summary}wrote {}", args.out.display()),
        Err(e) => {
            eprintln!("writing outputs: {e}");
            std::process::exit(1);
        }
    }
}
