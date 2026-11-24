//! Global alignment test on PLY scans (for example Revo Scan exports): runs
//! [`scancraft_align::align_global`] on pairs of clouds and writes results an agent can inspect.
//!
//! ```text
//! align_ply <out_dir> <a.ply> <b.ply> [<c.ply> ...] [options]
//!   --prep none|plane|part   none (default); plane: remove the dominant plane (table, RANSAC);
//!                            part: also drop everything on/below the table and keep the largest
//!                            connected, not plane-like cluster (the part without marker blocks)
//!   --voxel <mm>             GlobalConfig::voxel_mm (default: auto from size)
//!   --iterations <n>         GlobalConfig::ransac_iterations
//!   --candidates <n>         GlobalConfig::candidates
//!   --fine <mm>              GlobalConfig::fine_voxel_mm
//!   --seed <n>               GlobalConfig::seed
//!   --pairs all|chain        all: every ordered pair i->j (default); chain: S2->S1, S3->S1, S3->S2,
//!                            ... (every later scan onto every earlier one). `--chain` = `--pairs chain`.
//!   --plane-dist <mm>        band around the table plane that counts as table (default 1.5)
//!   --cluster-voxel <mm>     grid of the connected-component search (default 2.0)
//!   --tag <name>             name of the output sub-folder (default derived from the options)
//! Env fallbacks: ALIGN_PREP, ALIGN_VOXEL, ALIGN_ITERATIONS, ALIGN_CANDIDATES, ALIGN_PAIRS.
//! RUST_LOG=debug shows align_global's own log (best coarse hypothesis).
//! ```
//!
//! Scans are numbered S1, S2, ... in argument order. Per pair `Si -> Sj` (source i moved onto
//! target j) it writes `<out_dir>/<tag>/pair_i_j.ply` (moved source red + target grey, with
//! normals) and `pair_i_j_view{1,2,3}.png`: orthographic views along the principal axes of the
//! target's part region (view1 along the smallest axis, i.e. onto the shell; view2/3 side views),
//! source red, target grey, Lambert + depth shading. `results.csv` holds every number printed.

use std::collections::HashMap;
use std::io::Write;
use std::path::{Path, PathBuf};
use std::time::Instant;

use glam::{Quat, Vec3};
use rayon::prelude::*;
use scancraft_align::cloud::downsample;
use scancraft_align::global::auto_voxel;
use scancraft_align::{align_global, GlobalConfig, Oriented, PassCloud};
use scancraft_core::{Point3D, PointCloud, Transform};
use scancraft_fusion::{PlaneFilterMode, RansacPlaneFilter};

#[derive(Clone, Copy, PartialEq, Debug)]
enum Prep {
    None,
    Plane,
    Part,
}

impl Prep {
    fn name(self) -> &'static str {
        match self {
            Prep::None => "none",
            Prep::Plane => "plane",
            Prep::Part => "part",
        }
    }
}

struct Options {
    out_dir: PathBuf,
    inputs: Vec<PathBuf>,
    prep: Prep,
    voxel: Option<f32>,
    iterations: Option<usize>,
    candidates: Option<usize>,
    fine: Option<f32>,
    seed: Option<u64>,
    chain: bool,
    plane_dist: f32,
    cluster_voxel: f32,
    tag: Option<String>,
}

const IMAGE_PX: f32 = 1200.0;
const RENDER_MAX_POINTS: usize = 400_000;
const METRIC_MAX_POINTS: usize = 100_000;
/// (right, up, viewing direction) as indices into the frame's principal axes (largest first):
/// view1 looks along the smallest axis (onto the shell), view2 and view3 are side views.
const VIEWS: [(usize, usize, usize); 3] = [(0, 1, 2), (0, 2, 1), (1, 2, 0)];

fn usage() -> ! {
    eprintln!("usage: align_ply <out_dir> <a.ply> <b.ply> [<c.ply> ...] [--prep none|plane|part] [--voxel mm] [--iterations n] [--candidates n] [--fine mm] [--seed n] [--pairs all|chain | --chain] [--plane-dist mm] [--cluster-voxel mm] [--tag name]");
    std::process::exit(2);
}

fn parse_prep(s: &str) -> Prep {
    match s {
        "none" => Prep::None,
        "plane" => Prep::Plane,
        "part" => Prep::Part,
        _ => {
            eprintln!("unknown --prep {s}");
            usage()
        }
    }
}

fn parse_num<T: std::str::FromStr>(flag: &str, v: Option<String>) -> T {
    v.as_deref().and_then(|s| s.parse().ok()).unwrap_or_else(|| {
        eprintln!("{flag} needs a number");
        usage()
    })
}

fn parse_args() -> Options {
    let env = |k: &str| std::env::var(k).ok().filter(|s| !s.is_empty());
    let mut o = Options {
        out_dir: PathBuf::new(),
        inputs: Vec::new(),
        prep: env("ALIGN_PREP").map(|s| parse_prep(&s)).unwrap_or(Prep::None),
        voxel: env("ALIGN_VOXEL").and_then(|s| s.parse().ok()),
        iterations: env("ALIGN_ITERATIONS").and_then(|s| s.parse().ok()),
        candidates: env("ALIGN_CANDIDATES").and_then(|s| s.parse().ok()),
        fine: None,
        seed: None,
        chain: env("ALIGN_PAIRS").is_some_and(|s| s == "chain"),
        plane_dist: 1.5,
        cluster_voxel: 2.0,
        tag: None,
    };
    let mut positional = Vec::new();
    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        match a.as_str() {
            "--prep" => o.prep = parse_prep(&args.next().unwrap_or_default()),
            "--voxel" => o.voxel = Some(parse_num(&a, args.next())),
            "--iterations" => o.iterations = Some(parse_num(&a, args.next())),
            "--candidates" => o.candidates = Some(parse_num(&a, args.next())),
            "--fine" => o.fine = Some(parse_num(&a, args.next())),
            "--seed" => o.seed = Some(parse_num(&a, args.next())),
            "--plane-dist" => o.plane_dist = parse_num(&a, args.next()),
            "--cluster-voxel" => o.cluster_voxel = parse_num(&a, args.next()),
            "--tag" => o.tag = args.next(),
            "--chain" => o.chain = true,
            "--pairs" => match args.next().as_deref() {
                Some("all") => o.chain = false,
                Some("chain") => o.chain = true,
                _ => usage(),
            },
            "-h" | "--help" => usage(),
            s if s.starts_with("--") => {
                eprintln!("unknown option {s}");
                usage()
            }
            _ => positional.push(PathBuf::from(a)),
        }
    }
    if positional.len() < 3 {
        usage();
    }
    o.out_dir = positional.remove(0);
    o.inputs = positional;
    o
}

fn strategy_name(o: &Options) -> String {
    if let Some(t) = &o.tag {
        return t.clone();
    }
    let mut s = format!("prep-{}", o.prep.name());
    if let Some(v) = o.voxel {
        s += &format!("_v{v}");
    }
    if let Some(n) = o.iterations {
        s += &format!("_it{n}");
    }
    if let Some(n) = o.candidates {
        s += &format!("_c{n}");
    }
    if let Some(f) = o.fine {
        s += &format!("_f{f}");
    }
    if let Some(n) = o.seed {
        s += &format!("_s{n}");
    }
    if o.chain {
        s += "_chain";
    }
    s
}

// ---------------------------------------------------------------------------------------------
// Loading and writing

fn load(path: &Path) -> Vec<Oriented> {
    let (cloud, _) = scancraft_io::load_ply(path).unwrap_or_else(|e| {
        eprintln!("cannot read {}: {e}", path.display());
        std::process::exit(1)
    });
    cloud
        .points
        .iter()
        .map(|p| (Vec3::from_array(p.position), Vec3::from_array(p.normal).normalize_or_zero()))
        .filter(|(p, _)| p.is_finite())
        .collect()
}

/// Binary PLY with x y z nx ny nz red green blue alpha (readable by `scancraft_io::load_ply`).
fn write_ply(path: &Path, parts: &[(&[Oriented], [u8; 3])]) -> std::io::Result<()> {
    let n: usize = parts.iter().map(|p| p.0.len()).sum();
    let mut w = std::io::BufWriter::new(std::fs::File::create(path)?);
    write!(w, "ply\nformat binary_little_endian 1.0\ncomment ScanCraft align_ply\nelement vertex {n}\nproperty float x\nproperty float y\nproperty float z\nproperty float nx\nproperty float ny\nproperty float nz\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nproperty uchar alpha\nend_header\n")?;
    for (pts, c) in parts {
        for (p, nrm) in pts.iter() {
            for v in [p.x, p.y, p.z, nrm.x, nrm.y, nrm.z] {
                w.write_all(&v.to_le_bytes())?;
            }
            w.write_all(&[c[0], c[1], c[2], 255])?;
        }
    }
    w.flush()
}

// ---------------------------------------------------------------------------------------------
// Small geometry helpers

/// Eigen-decomposition of a symmetric 3x3 matrix (Jacobi); eigenvalues descending, with
/// eigenvectors forming a right-handed frame.
fn eigen_sym(m: [[f64; 3]; 3]) -> ([f64; 3], [Vec3; 3]) {
    let mut a = m;
    let mut v = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0f64]];
    for _ in 0..50 {
        let off = a[0][1].abs() + a[0][2].abs() + a[1][2].abs();
        if off < 1e-12 {
            break;
        }
        for (p, q) in [(0usize, 1usize), (0, 2), (1, 2)] {
            if a[p][q].abs() < 1e-15 {
                continue;
            }
            let theta = (a[q][q] - a[p][p]) / (2.0 * a[p][q]);
            let t = theta.signum() / (theta.abs() + (theta * theta + 1.0).sqrt());
            let t = if theta == 0.0 { 1.0 } else { t };
            let c = 1.0 / (t * t + 1.0).sqrt();
            let s = t * c;
            for k in 0..3 {
                let (akp, akq) = (a[k][p], a[k][q]);
                a[k][p] = c * akp - s * akq;
                a[k][q] = s * akp + c * akq;
            }
            for k in 0..3 {
                let (apk, aqk) = (a[p][k], a[q][k]);
                a[p][k] = c * apk - s * aqk;
                a[q][k] = s * apk + c * aqk;
            }
            for row in v.iter_mut() {
                let (vkp, vkq) = (row[p], row[q]);
                row[p] = c * vkp - s * vkq;
                row[q] = s * vkp + c * vkq;
            }
        }
    }
    let mut idx = [0usize, 1, 2];
    idx.sort_by(|&i, &j| a[j][j].total_cmp(&a[i][i]));
    let vals = [a[idx[0]][idx[0]], a[idx[1]][idx[1]], a[idx[2]][idx[2]]];
    let col = |i: usize| Vec3::new(v[0][i] as f32, v[1][i] as f32, v[2][i] as f32).normalize();
    let e0 = col(idx[0]);
    let e1 = col(idx[1]);
    let e2 = e0.cross(e1).normalize();
    (vals, [e0, e1, e2])
}

/// Centroid, eigenvalues (descending, mm^2) and principal axes of the points.
fn pca(points: impl Iterator<Item = Vec3> + Clone) -> (Vec3, [f64; 3], [Vec3; 3]) {
    let (mut sum, mut n) = (glam::DVec3::ZERO, 0usize);
    for p in points.clone() {
        sum += p.as_dvec3();
        n += 1;
    }
    let c = sum / n.max(1) as f64;
    let mut m = [[0.0f64; 3]; 3];
    for p in points {
        let d = p.as_dvec3() - c;
        let d = [d.x, d.y, d.z];
        for i in 0..3 {
            for j in 0..3 {
                m[i][j] += d[i] * d[j];
            }
        }
    }
    for row in m.iter_mut() {
        for v in row.iter_mut() {
            *v /= n.max(1) as f64;
        }
    }
    let (vals, axes) = eigen_sym(m);
    (c.as_vec3(), vals, axes)
}

#[derive(Clone, Copy, Debug)]
struct PlaneFit {
    normal: Vec3,
    d: f32,
}

impl PlaneFit {
    fn dist(&self, p: Vec3) -> f32 {
        self.normal.dot(p) + self.d
    }
}

/// Dominant plane: RANSAC (scancraft-fusion) on a 3 mm subsample, least-squares refit on its
/// inliers, normal oriented towards the side with more points well off the plane.
fn fit_plane(pts: &[Oriented]) -> Option<(PlaneFit, usize)> {
    let sub = downsample(pts, 3.0);
    let cloud = PointCloud::from_points(sub.iter().map(|(p, _)| Point3D::new(p.to_array())).collect());
    let plane = RansacPlaneFilter::new(1.0, 1500, PlaneFilterMode::RemoveInliers).detect_plane(&cloud)?;
    let mut fit = PlaneFit { normal: plane.normal, d: plane.d };
    for band in [2.0f32, 1.5, 1.2] {
        let inl: Vec<Vec3> = sub.iter().map(|p| p.0).filter(|&p| fit.dist(p).abs() <= band).collect();
        if inl.len() < 10 {
            break;
        }
        let (c, _, axes) = pca(inl.iter().copied());
        let mut n = axes[2];
        if n.dot(fit.normal) < 0.0 {
            n = -n;
        }
        fit = PlaneFit { normal: n, d: -n.dot(c) };
    }
    let above = pts.iter().filter(|p| fit.dist(p.0) > 5.0).count();
    let below = pts.iter().filter(|p| fit.dist(p.0) < -5.0).count();
    if below > above {
        fit = PlaneFit { normal: -fit.normal, d: -fit.d };
    }
    let inliers = pts.iter().filter(|p| fit.dist(p.0).abs() <= 1.5).count();
    Some((fit, inliers))
}

/// Connected components of the points on a voxel grid (26-neighbourhood); largest first.
fn components(pts: &[Oriented], voxel: f32) -> Vec<Vec<usize>> {
    let key = |p: Vec3| ((p.x / voxel).floor() as i32, (p.y / voxel).floor() as i32, (p.z / voxel).floor() as i32);
    let mut cells: HashMap<(i32, i32, i32), usize> = HashMap::new();
    let mut members: Vec<Vec<usize>> = Vec::new();
    let mut keys: Vec<(i32, i32, i32)> = Vec::new();
    for (i, (p, _)) in pts.iter().enumerate() {
        let k = key(*p);
        let id = *cells.entry(k).or_insert_with(|| {
            members.push(Vec::new());
            keys.push(k);
            members.len() - 1
        });
        members[id].push(i);
    }
    let mut label = vec![usize::MAX; members.len()];
    let mut comps: Vec<Vec<usize>> = Vec::new();
    for start in 0..members.len() {
        if label[start] != usize::MAX {
            continue;
        }
        let c = comps.len();
        label[start] = c;
        let mut stack = vec![start];
        let mut out = Vec::new();
        while let Some(cell) = stack.pop() {
            out.extend_from_slice(&members[cell]);
            let (x, y, z) = keys[cell];
            for dx in -1..=1 {
                for dy in -1..=1 {
                    for dz in -1..=1 {
                        if let Some(&nb) = cells.get(&(x + dx, y + dy, z + dz)) {
                            if label[nb] == usize::MAX {
                                label[nb] = c;
                                stack.push(nb);
                            }
                        }
                    }
                }
            }
        }
        comps.push(out);
    }
    comps.sort_by_key(|c| std::cmp::Reverse(c.len()));
    comps
}

/// The part: largest connected, not plane-like cluster above the table. `None` if nothing fits.
fn isolate_part(pts: &[Oriented], plane: &PlaneFit, plane_dist: f32, cluster_voxel: f32, verbose: bool) -> Option<Vec<Oriented>> {
    let above: Vec<Oriented> = pts.iter().copied().filter(|p| plane.dist(p.0) > plane_dist).collect();
    let comps = components(&above, cluster_voxel);
    let mut chosen = None;
    for (k, c) in comps.iter().take(12).enumerate() {
        if c.len() < 200 {
            break;
        }
        let (centre, vals, _) = pca(c.iter().map(|&i| above[i].0));
        let sd = vals.map(|v| v.max(0.0).sqrt() as f32);
        let height = plane.dist(centre);
        let max_h = c.iter().map(|&i| plane.dist(above[i].0)).fold(0.0f32, f32::max);
        let plane_like = sd[2] < (0.02 * sd[0]).max(1.0);
        let pick = chosen.is_none() && !plane_like && c.len() >= 1000;
        if verbose && k < 8 {
            println!(
                "      cluster {k}: {:>7} pts, sd {:.1}/{:.1}/{:.1} mm, centre {:.1} mm / top {:.1} mm above table{}{}",
                c.len(),
                sd[0],
                sd[1],
                sd[2],
                height,
                max_h,
                if plane_like { ", plane-like" } else { "" },
                if pick { "  <- part" } else { "" }
            );
        }
        if pick {
            chosen = Some(k);
        }
    }
    chosen.map(|k| comps[k].iter().map(|&i| above[i]).collect())
}

struct Scan {
    name: String,
    raw_len: usize,
    points: Vec<Oriented>,
    /// Part region used to frame the renders (centre, right-handed principal axes, lo/hi per axis).
    frame: Frame,
}

#[derive(Clone, Copy)]
struct Frame {
    centre: Vec3,
    axes: [Vec3; 3],
    lo: Vec3,
    hi: Vec3,
}

fn frame_of(region: &[Oriented]) -> Frame {
    let stride = (region.len() / 200_000).max(1);
    let (centre, _, axes) = pca(region.iter().step_by(stride).map(|p| p.0));
    let (mut lo, mut hi) = (Vec3::splat(f32::MAX), Vec3::splat(f32::MIN));
    for (p, _) in region.iter().step_by(stride) {
        let d = *p - centre;
        let q = Vec3::new(d.dot(axes[0]), d.dot(axes[1]), d.dot(axes[2]));
        lo = lo.min(q);
        hi = hi.max(q);
    }
    let margin = (hi - lo) * 0.15 + Vec3::splat(2.0);
    Frame { centre, axes, lo: lo - margin, hi: hi + margin }
}

fn prepare(index: usize, path: &Path, o: &Options) -> Scan {
    let t0 = Instant::now();
    let raw = load(path);
    let name = format!("S{}", index + 1);
    println!("{name}: {} ({} points, extent {:.1} mm, loaded in {:.2} s)", path.display(), raw.len(), scancraft_align::cloud::extent(&raw), t0.elapsed().as_secs_f32());
    let plane = fit_plane(&raw);
    match &plane {
        Some((p, inl)) => println!("    table plane: normal ({:.3}, {:.3}, {:.3}), d {:.1} mm, {} points within 1.5 mm ({:.0} %)", p.normal.x, p.normal.y, p.normal.z, p.d, inl, 100.0 * *inl as f32 / raw.len() as f32),
        None => println!("    no dominant plane found"),
    }
    let verbose = o.prep == Prep::Part;
    let part = plane.as_ref().and_then(|(p, _)| isolate_part(&raw, p, o.plane_dist, o.cluster_voxel, verbose));
    let points = match (o.prep, &plane) {
        (Prep::None, _) | (_, None) => raw.clone(),
        (Prep::Plane, Some((p, _))) => raw.iter().copied().filter(|q| p.dist(q.0).abs() > o.plane_dist).collect(),
        (Prep::Part, Some(_)) => part.clone().unwrap_or_else(|| {
            println!("    WARNING: no part cluster found, using the whole cloud");
            raw.clone()
        }),
    };
    if o.prep != Prep::None {
        println!("    prep {}: {} -> {} points ({:.2} s)", o.prep.name(), raw.len(), points.len(), t0.elapsed().as_secs_f32());
    }
    let region = part.as_deref().filter(|r| r.len() >= 1000).unwrap_or(&points);
    let frame = frame_of(region);
    println!(
        "    part region for views: {} points, size {:.0} x {:.0} x {:.0} mm along its principal axes",
        region.len(),
        frame.hi.x - frame.lo.x,
        frame.hi.y - frame.lo.y,
        frame.hi.z - frame.lo.z
    );
    Scan { name, raw_len: raw.len(), points, frame }
}

// ---------------------------------------------------------------------------------------------
// Metrics and rendering

/// (fraction within 1 mm, RMS within 1 mm, fraction within 3 mm, RMS within 3 mm, median) of
/// nearest-neighbour distances from the moved source to the target. Distances beyond the grid's
/// search range count as infinite (the median is then reported as inf).
fn nn_metrics(moved: &[Oriented], index: &GridIndex) -> [f32; 5] {
    let stride = (moved.len() / METRIC_MAX_POINTS).max(1);
    let mut d: Vec<f32> = moved.par_iter().step_by(stride).map(|(p, _)| index.nearest(*p)).collect();
    if d.is_empty() {
        return [0.0, f32::NAN, 0.0, f32::NAN, f32::NAN];
    }
    let stats = |gate: f32| {
        let within: Vec<f32> = d.iter().copied().filter(|&x| x <= gate).collect();
        let rms = if within.is_empty() { f32::NAN } else { (within.iter().map(|x| x * x).sum::<f32>() / within.len() as f32).sqrt() };
        (within.len() as f32 / d.len() as f32, rms)
    };
    let (f1, r1) = stats(1.0);
    let (f3, r3) = stats(3.0);
    d.sort_by(f32::total_cmp);
    [f1, r1, f3, r3, d[d.len() / 2]]
}

/// Nearest-neighbour search within a fixed range on a hash grid, over full-resolution points.
/// (`scancraft_align::cloud::Index` cannot be used here: Revo's fused clouds lie on grid planes,
/// hundreds of points share one exact coordinate, `Index`'s micrometre jitter is below f32
/// resolution at a few hundred mm, and kiddo's tree construction overflows a leaf and panics.)
struct GridIndex {
    cell: f32,
    cells: HashMap<(i32, i32, i32), Vec<Vec3>>,
}

impl GridIndex {
    /// Exact nearest neighbour for distances up to `2 * cell`.
    fn new(points: &[Oriented], cell: f32) -> Self {
        let mut cells: HashMap<(i32, i32, i32), Vec<Vec3>> = HashMap::new();
        for (p, _) in points {
            cells.entry(Self::key(*p, cell)).or_default().push(*p);
        }
        GridIndex { cell, cells }
    }

    fn key(p: Vec3, cell: f32) -> (i32, i32, i32) {
        ((p.x / cell).floor() as i32, (p.y / cell).floor() as i32, (p.z / cell).floor() as i32)
    }

    fn nearest(&self, p: Vec3) -> f32 {
        let (x, y, z) = Self::key(p, self.cell);
        let mut best = f32::INFINITY;
        for dx in -2..=2 {
            for dy in -2..=2 {
                for dz in -2..=2 {
                    if let Some(v) = self.cells.get(&(x + dx, y + dy, z + dz)) {
                        for q in v {
                            best = best.min(q.distance_squared(p));
                        }
                    }
                }
            }
        }
        let d = best.sqrt();
        if d <= 2.0 * self.cell { d } else { f32::INFINITY }
    }
}

/// Orthographic render looking along `axes[look]` of the frame, image axes `axes[u]` (right) and
/// `axes[v]` (up). Later layers win only where they are nearer.
fn render(frame: &Frame, layers: &[(&[Oriented], [u8; 3])], u: usize, v: usize, look: usize) -> image::RgbImage {
    let (lo, hi) = (frame.lo.to_array(), frame.hi.to_array());
    let (su, sv) = (hi[u] - lo[u], hi[v] - lo[v]);
    let scale = IMAGE_PX / su.max(sv);
    let (w, h) = ((su * scale).ceil().max(1.0) as u32, (sv * scale).ceil().max(1.0) as u32);
    let mut depth = vec![f32::NEG_INFINITY; (w * h) as usize];
    let mut img = image::RgbImage::from_pixel(w, h, image::Rgb([28, 28, 32]));
    let (au, av, aw) = (frame.axes[u], frame.axes[v], frame.axes[look]);
    let (wlo, whi) = (lo[look], hi[look]);
    for (pts, colour) in layers {
        let stride = (pts.len() / RENDER_MAX_POINTS).max(1);
        let r = ((0.2 * scale * (stride as f32).sqrt()).ceil() as i32).clamp(1, 3);
        for (p, n) in pts.iter().step_by(stride) {
            let d = *p - frame.centre;
            let (pu, pv, pw) = (d.dot(au), d.dot(av), d.dot(aw));
            if pw < wlo || pw > whi {
                continue;
            }
            let x = ((pu - lo[u]) * scale) as i32;
            let y = ((hi[v] - pv) * scale) as i32;
            if x < -r || y < -r || x >= w as i32 + r || y >= h as i32 + r {
                continue;
            }
            let lambert = if n.length_squared() > 0.25 { n.dot(aw).abs() } else { 0.6 };
            let near = ((pw - wlo) / (whi - wlo)).clamp(0.0, 1.0);
            let shade = (0.25 + 0.75 * lambert) * (0.5 + 0.5 * near);
            let px = image::Rgb(colour.map(|c| (c as f32 * shade).min(255.0) as u8));
            for dy in -r..=r {
                for dx in -r..=r {
                    let (xx, yy) = (x + dx, y + dy);
                    if xx < 0 || yy < 0 || xx >= w as i32 || yy >= h as i32 {
                        continue;
                    }
                    let i = (yy as u32 * w + xx as u32) as usize;
                    if pw > depth[i] {
                        depth[i] = pw;
                        img.put_pixel(xx as u32, yy as u32, px);
                    }
                }
            }
        }
    }
    img
}

// ---------------------------------------------------------------------------------------------

fn main() {
    let _ = env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("warn")).try_init();
    let o = parse_args();
    let strategy = strategy_name(&o);
    let dir = o.out_dir.join(&strategy);
    std::fs::create_dir_all(&dir).expect("create output folder");
    let total = Instant::now();

    let scans: Vec<Scan> = o.inputs.iter().enumerate().map(|(i, p)| prepare(i, p, &o)).collect();
    if o.prep != Prep::None {
        for s in &scans {
            let path = dir.join(format!("prep_{}.ply", s.name));
            write_ply(&path, &[(&s.points, [200, 200, 200])]).expect("write prepared PLY");
            let layers: [(&[Oriented], [u8; 3]); 1] = [(&s.points, [200, 200, 200])];
            for (k, (u, v, look)) in VIEWS.into_iter().enumerate() {
                render(&s.frame, &layers, u, v, look).save(dir.join(format!("prep_{}_view{}.png", s.name, k + 1))).expect("write PNG");
            }
            println!("    wrote {} ({} of {} points)", path.display(), s.points.len(), s.raw_len);
        }
    }

    let mut config = GlobalConfig::default();
    config.voxel_mm = o.voxel;
    if let Some(n) = o.iterations {
        config.ransac_iterations = n;
    }
    if let Some(n) = o.candidates {
        config.candidates = n;
    }
    if let Some(f) = o.fine {
        config.fine_voxel_mm = f;
    }
    if let Some(s) = o.seed {
        config.seed = s;
    }
    println!("\nstrategy {strategy}: {config:?}");

    let n = scans.len();
    let pairs: Vec<(usize, usize)> = if o.chain {
        (1..n).flat_map(|i| (0..i).map(move |j| (i, j))).collect()
    } else {
        (0..n).flat_map(|i| (0..n).filter(move |&j| j != i).map(move |j| (i, j))).collect()
    };

    let mut csv = std::fs::File::create(dir.join("results.csv")).expect("results.csv");
    writeln!(csv, "pair,source,target,src_pts,tgt_pts,voxel_mm,status,seconds,rot_deg,axis_x,axis_y,axis_z,tx,ty,tz,qx,qy,qz,qw,overlap,rmse_mm,visibility_conflict,inliers,score,verdict,nn_frac_1mm,nn_rms_1mm,nn_frac_3mm,nn_rms_3mm,nn_median_mm").unwrap();

    let mut rows = Vec::new();
    let passes: Vec<PassCloud> = scans.iter().map(|s| PassCloud { points: s.points.clone(), views: Vec::new() }).collect();
    let mut indices: HashMap<usize, GridIndex> = HashMap::new();
    for &(i, j) in &pairs {
        let (src, tgt) = (&scans[i], &scans[j]);
        let label = format!("{}->{}", src.name, tgt.name);
        let voxel = o.voxel.unwrap_or_else(|| auto_voxel(&src.points, &tgt.points));
        println!("\n== {label}: {} -> {} points, feature voxel {voxel:.2} mm{}", src.points.len(), tgt.points.len(), if o.voxel.is_none() { " (auto)" } else { "" });
        let t0 = Instant::now();
        let result = align_global(&passes[i], &passes[j], &config);
        let secs = t0.elapsed().as_secs_f32();
        let a = match result {
            Ok(a) => a,
            Err(e) => {
                println!("   FAILED after {secs:.2} s: {e}");
                writeln!(csv, "{label},{},{},{},{},{voxel},failed: {e},{secs:.3}", src.name, tgt.name, src.points.len(), tgt.points.len()).unwrap();
                rows.push(format!("| {label} | failed: {e} | | | | | | | | {secs:.1} |"));
                continue;
            }
        };
        let t: Transform = a.transform;
        let mut q: Quat = t.rotation.normalize();
        if q.w < 0.0 {
            q = -q;
        }
        let (axis, angle) = q.to_axis_angle();
        let qa = a.quality;
        println!("   time {secs:.2} s");
        println!("   rotation {:.2} deg about ({:.3}, {:.3}, {:.3}); translation ({:.2}, {:.2}, {:.2}) mm; det +1 (proper rotation, never mirrored)", angle.to_degrees(), axis.x, axis.y, axis.z, t.translation.x, t.translation.y, t.translation.z);
        println!("   quality: overlap {:.3}, rmse {:.3} mm, visibility conflict {:.3}, inliers {}, score {:.4} -> verdict {}", qa.overlap, qa.rmse_mm, qa.visibility_conflict, qa.inliers, qa.score(), qa.verdict().label());

        let moved = scancraft_align::cloud::transform_oriented(&src.points, &t);
        let index = indices.entry(j).or_insert_with(|| GridIndex::new(&tgt.points, 1.5));
        let m = nn_metrics(&moved, index);
        println!("   nearest neighbour: {:.1} % within 1 mm (RMS {:.3} mm), {:.1} % within 3 mm (RMS {:.3} mm), median {:.2} mm (inf = beyond 3 mm)", 100.0 * m[0], m[1], 100.0 * m[2], m[3], m[4]);

        let stem = format!("pair_{}_{}", i + 1, j + 1);
        let red = [230u8, 60, 45];
        let grey = [200u8, 200, 200];
        write_ply(&dir.join(format!("{stem}.ply")), &[(&moved, red), (&tgt.points, grey)]).expect("write pair PLY");
        let layers: [(&[Oriented], [u8; 3]); 2] = [(&tgt.points, grey), (&moved, red)];
        for (k, (u, v, look)) in VIEWS.into_iter().enumerate() {
            let img = render(&tgt.frame, &layers, u, v, look);
            img.save(dir.join(format!("{stem}_view{}.png", k + 1))).expect("write PNG");
        }
        let f = &tgt.frame;
        let inside = |pts: &[Oriented]| {
            let n = pts
                .iter()
                .filter(|(p, _)| {
                    let d = *p - f.centre;
                    let q = Vec3::new(d.dot(f.axes[0]), d.dot(f.axes[1]), d.dot(f.axes[2]));
                    q.cmpge(f.lo).all() && q.cmple(f.hi).all()
                })
                .count();
            100.0 * n as f32 / pts.len().max(1) as f32
        };
        println!("   in the views' frame (target part region + 15 %): {:.1} % of the moved source, {:.1} % of the target", inside(&moved), inside(&tgt.points));
        println!("   wrote {stem}.ply and {stem}_view1..3.png");

        writeln!(
            csv,
            "{label},{},{},{},{},{voxel},ok,{secs:.3},{:.4},{:.5},{:.5},{:.5},{:.4},{:.4},{:.4},{:.7},{:.7},{:.7},{:.7},{:.4},{:.4},{:.4},{},{:.5},{},{:.4},{:.4},{:.4},{:.4},{:.4}",
            src.name,
            tgt.name,
            src.points.len(),
            tgt.points.len(),
            angle.to_degrees(),
            axis.x,
            axis.y,
            axis.z,
            t.translation.x,
            t.translation.y,
            t.translation.z,
            q.x,
            q.y,
            q.z,
            q.w,
            qa.overlap,
            qa.rmse_mm,
            qa.visibility_conflict,
            qa.inliers,
            qa.score(),
            qa.verdict().label(),
            m[0],
            m[1],
            m[2],
            m[3],
            m[4]
        )
        .unwrap();
        rows.push(format!(
            "| {label} | {:.1} deg ({:.2},{:.2},{:.2}) | ({:.1}, {:.1}, {:.1}) | {:.3} | {:.3} | {:.3} | {} | {} | {:.1} % / {:.3} | {:.1} % / {:.3} | {secs:.1} |",
            angle.to_degrees(),
            axis.x,
            axis.y,
            axis.z,
            t.translation.x,
            t.translation.y,
            t.translation.z,
            qa.overlap,
            qa.rmse_mm,
            qa.visibility_conflict,
            qa.inliers,
            qa.verdict().label(),
            100.0 * m[0],
            m[1],
            100.0 * m[2],
            m[3]
        ));
    }

    println!("\nResults ({strategy}, outputs in {}):", dir.display());
    println!("| pair | rotation (axis) | translation mm | overlap | rmse mm | vis. conflict | inliers | verdict | NN <=1 mm (frac / RMS) | NN <=3 mm (frac / RMS) | time s |");
    println!("|---|---|---|---|---|---|---|---|---|---|---|");
    for r in rows {
        println!("{r}");
    }
    println!("total {:.1} s", total.elapsed().as_secs_f32());
}
