//! Marker stickers: detection (raised disc or round hole), cut, smooth infill and mesh fairing.
//!
//! Marker stickers on the part show up in Revo's previews as flat discs about 0.4 mm proud with a
//! radius of about 4.5 mm (Revo's replacement disc, larger than the 3 mm sticker), and in Revo's
//! fused clouds as round holes of radius about 5 mm. Both are removed the same way:
//!
//! 0. **Merge:** copies of one sticker (listed once per scan, or listed and detected) closer than
//!    `merge_mm` become one site whose cut covers all of them.
//! 1. **Cut:** points within `cut_radius` of the marker axis are dropped when they lie within
//!    1.3 mm of the marker's sheet along the axis (any normal: the disc edge and the dark ring carry
//!    wild normals), or within `cut_height` and facing like the sheet (normal within 60°). The
//!    height limit and the normal test keep the opposite wall, one wall thickness behind, intact.
//! 2. **Infill:** a quadric height field is fitted (robustly, three passes) to the same sheet in
//!    the annulus `cut_radius .. cut_radius + fit_width` (points of every scan, so the patch
//!    follows the average the solve makes of overlapping passes) and sampled inside the cut on a
//!    0.3 mm lattice. A site whose annulus is not covered all round (a rim) or too rough (a rib or
//!    edge nearby) is not cut at all (`keep_unfillable`), since an unfilled cut can open a hole.
//!    Points that any site cuts (a neighbouring sticker) count for the coverage of an annulus but
//!    are not fitted. Where the cuts of two sites on one sheet overlap, each lattice point belongs
//!    to the site it lies deepest in and is placed on a blend of the covering patches, so the
//!    overlap carries one layer without a step.
//! 3. **Fairing** ([`fair_sites`], after meshing): the mesh annulus is fitted again and the mesh
//!    inside the cut is blended onto that fit, which removes any dent or bump the solve left.
//!    Overlapping sites share the vertices they both move (weighted mean of their displacements).

use super::grid::{FxMap, PointGrid};
use super::{PointKind, WorkPoint};
use glam::{DVec3, Vec3};
use rayon::prelude::*;

/// Where a marker site comes from.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MarkerSource {
    /// Given by the caller (tracker marker list, e.g. Revo's `marker_framework.mkf`).
    List,
    /// Found by the raised-disc detector.
    DetectedDisc,
    /// Found by the round-hole detector.
    DetectedHole,
}

#[derive(Clone, Debug)]
pub struct Marker {
    pub id: String,
    /// Centre on (or within a fraction of a millimetre of) the surface.
    pub center: Vec3,
    /// Surface normal at the marker, pointing out of the material (re-oriented from the data).
    pub normal: Vec3,
    /// Only points of this scan are cut and used for the fit (`None`: all scans).
    pub scan: Option<u8>,
    pub source: MarkerSource,
    /// Measured radius of the disc or hole, 0 when unknown.
    pub radius_mm: f32,
    /// Detector score (height / annulus RMS for discs, hole radius for holes).
    pub score: f32,
}

#[derive(Clone, Debug)]
pub struct MarkerConfig {
    pub cut_radius_mm: f32,
    pub cut_height_mm: f32,
    /// Width of the annulus outside the cut used for the quadric fit.
    pub fit_width_mm: f32,
    /// Annulus RMS of a clean site; up to `fill_rms_limit_mm` the site is still patched (marked
    /// "rough annulus" in the outcome), beyond it it gets no patch.
    pub fit_rms_max_mm: f32,
    pub fill_rms_limit_mm: f32,
    /// Sites that cannot be patched (annulus too rough or not covered all round, e.g. at a rim)
    /// are not cut either: the sticker stays rather than leaving a hole the solve may not span.
    pub keep_unfillable: bool,
    /// Lattice spacing of the infill points.
    pub fill_spacing_mm: f32,
    /// Quadric patch (M2) or leave the cut to the solve (M1).
    pub quadric_fill: bool,
    /// Fit the annulus with the points of every scan, not only the marker's own scan. Two passes
    /// that sit a fraction of a millimetre apart are averaged by the solve, so a patch fitted to
    /// one of them alone ends up as a dent or a bump of half their offset.
    pub fit_all_scans: bool,
    /// Sites closer than this (centres, same facing) are merged into one site before the cut
    /// (the same sticker listed once per scan).
    pub merge_mm: f32,
    /// After meshing, re-fit each site on the mesh annulus and blend the mesh inside the cut onto
    /// that fit ([`fair_sites`]).
    pub fair_mesh: bool,
    /// Mesh annulus RMS limit for the fairing (the mesh is smoother than the data).
    pub fair_rms_max_mm: f32,
    /// Width of the blend from the faired patch back to the mesh outside the cut.
    pub fair_blend_mm: f32,
    /// A listed marker normal whose direction disagrees with the data ([`orient_markers`]) by
    /// more than this (degrees) is replaced by the data's; closer ones keep their direction (near
    /// ribs and edges the tracker's normal is the better one). 180 or more: only the sign is
    /// taken from the data.
    pub reorient_min_deg: f32,
}

impl Default for MarkerConfig {
    fn default() -> Self {
        Self {
            cut_radius_mm: 6.5,
            cut_height_mm: 1.5,
            fit_width_mm: 6.0,
            fit_rms_max_mm: 0.1,
            fill_rms_limit_mm: 0.25,
            keep_unfillable: true,
            fill_spacing_mm: 0.3,
            quadric_fill: true,
            fit_all_scans: true,
            merge_mm: 5.0,
            fair_mesh: true,
            fair_rms_max_mm: 0.1,
            fair_blend_mm: 2.0,
            reorient_min_deg: 10.0,
        }
    }
}

/// What happened at one marker site.
#[derive(Clone, Debug)]
pub struct MarkerOutcome {
    pub id: String,
    pub scan: Option<u8>,
    pub source: MarkerSource,
    pub center: Vec3,
    pub cut_points: usize,
    pub annulus_points: usize,
    pub annulus_rms_mm: f32,
    pub sectors_covered: u32,
    pub fill_points: usize,
    /// "quadric", or why the site was kept or left to the solve.
    pub fill: String,
    /// Mesh fairing of the site, when it ran.
    pub fair: Option<FairOutcome>,
}

/// What the mesh fairing did at one site.
#[derive(Clone, Debug)]
pub struct FairOutcome {
    /// "faired", or why the site was left as meshed.
    pub status: String,
    /// Mesh annulus RMS against its own quadric (mm).
    pub ring_rms_mm: f32,
    /// Mean and largest deviation of the mesh inside the cut from the annulus quadric before the
    /// fairing (mm; a dent is negative).
    pub mean_before_mm: f32,
    pub max_before_mm: f32,
    /// Vertices moved (inside the cut and in the blend band).
    pub vertices: usize,
}

/// Orthonormal tangent basis for a unit normal.
pub(crate) fn tangent_frame(n: Vec3) -> (Vec3, Vec3) {
    let a = if n.x.abs() < 0.6 { Vec3::X } else { Vec3::Y };
    let u = n.cross(a).normalize();
    (u, n.cross(u))
}

/// Least-squares quadric height field z = c0 + c1 u + c2 v + c3 u² + c4 uv + c5 v².
#[derive(Clone, Copy, Debug)]
pub(crate) struct Quadric(pub [f64; 6]);

impl Quadric {
    #[inline]
    pub fn eval(&self, u: f64, v: f64) -> f64 {
        let c = &self.0;
        c[0] + c[1] * u + c[2] * v + c[3] * u * u + c[4] * u * v + c[5] * v * v
    }
    #[inline]
    pub fn grad(&self, u: f64, v: f64) -> (f64, f64) {
        let c = &self.0;
        (c[1] + 2.0 * c[3] * u + c[4] * v, c[2] + c[4] * u + 2.0 * c[5] * v)
    }

    fn fit_once(s: &[(f64, f64, f64)], keep: &[bool]) -> Option<Quadric> {
        let mut a = [[0f64; 6]; 6];
        let mut b = [0f64; 6];
        let mut n = 0usize;
        for (&(u, v, z), &k) in s.iter().zip(keep) {
            if !k {
                continue;
            }
            n += 1;
            let f = [1.0, u, v, u * u, u * v, v * v];
            for i in 0..6 {
                b[i] += f[i] * z;
                for j in 0..6 {
                    a[i][j] += f[i] * f[j];
                }
            }
        }
        if n < 12 {
            return None;
        }
        // Tiny ridge for stability on narrow annuli.
        for (i, row) in a.iter_mut().enumerate() {
            row[i] += 1e-9 * (1.0 + row[i]);
        }
        solve6(a, b).map(Quadric)
    }

    /// Three-pass fit: each refit drops residuals beyond max(2 σ, 0.03 mm) of the previous fit.
    /// Returns the fit and the RMS over the kept samples.
    pub fn fit_robust(s: &[(f64, f64, f64)]) -> Option<(Quadric, f64, usize)> {
        let mut keep = vec![true; s.len()];
        let mut q = Self::fit_once(s, &keep)?;
        let mut rms = rms_of(s, &keep, &q);
        for _ in 0..2 {
            let lim = (2.0 * rms).max(0.03);
            for (k, &(u, v, z)) in keep.iter_mut().zip(s) {
                *k = (z - q.eval(u, v)).abs() <= lim;
            }
            q = Self::fit_once(s, &keep)?;
            rms = rms_of(s, &keep, &q);
        }
        Some((q, rms, keep.iter().filter(|k| **k).count()))
    }
}

fn rms_of(s: &[(f64, f64, f64)], keep: &[bool], q: &Quadric) -> f64 {
    let (mut acc, mut n) = (0.0, 0usize);
    for (&(u, v, z), &k) in s.iter().zip(keep) {
        if k {
            let r = z - q.eval(u, v);
            acc += r * r;
            n += 1;
        }
    }
    if n == 0 {
        f64::INFINITY
    } else {
        (acc / n as f64).sqrt()
    }
}

/// Gaussian elimination with partial pivoting for a 6×6 system.
pub(crate) fn solve6(mut a: [[f64; 6]; 6], mut b: [f64; 6]) -> Option<[f64; 6]> {
    for c in 0..6 {
        let p = (c..6).max_by(|&i, &j| a[i][c].abs().total_cmp(&a[j][c].abs()))?;
        if a[p][c].abs() < 1e-12 {
            return None;
        }
        a.swap(c, p);
        b.swap(c, p);
        for r in c + 1..6 {
            let f = a[r][c] / a[c][c];
            for k in c..6 {
                a[r][k] -= f * a[c][k];
            }
            b[r] -= f * b[c];
        }
    }
    let mut x = [0f64; 6];
    for r in (0..6).rev() {
        let mut s = b[r];
        for k in r + 1..6 {
            s -= a[r][k] * x[k];
        }
        x[r] = s / a[r][r];
    }
    Some(x)
}

/// Re-orients each marker normal from the data around it (points of its scan, or of every scan
/// when the site has none). First the sign: the summed normals of the points within 1.5 mm of the
/// centre (4 or 8 mm when there are fewer than 5, e.g. at a hole) and within 1 mm of the given
/// tangent plane. Then the direction ([`sheet_normal`]): a coarse mean normal of the sheet around
/// the centre (catches a tilt of up to ~40°), then twice the normal at the centre of a robust
/// quadric fitted to the site's annulus (`cut .. cut + fit_width`, the same points the infill
/// fits), which follows a curved sheet. The data's direction replaces the listed one only when
/// the annulus is covered all round and the two differ by more than `reorient_min_deg`, so a
/// good tracker normal is kept and a tilted one is corrected; the cut, the annulus frame and the
/// fairing axis then follow the sheet.
pub(crate) fn orient_markers(markers: &mut [Marker], pts: &[WorkPoint], pos: &[Vec3], grid: &PointGrid, prm: &MarkerConfig) {
    markers.par_iter_mut().for_each(|m| {
        let n0 = m.normal.normalize_or_zero();
        if n0 == Vec3::ZERO {
            return;
        }
        let ours = |p: &WorkPoint| m.scan.map_or(true, |s| s == p.scan);
        let mut sign = Vec3::ZERO;
        for r in [1.5f32, 4.0, 8.0] {
            let (mut acc, mut count) = (Vec3::ZERO, 0usize);
            grid.for_each_within(pos, m.center, r, |i, _| {
                let p = &pts[i as usize];
                if ours(p) && (n0.dot(p.p - m.center)).abs() < 1.0 {
                    acc += p.n;
                    count += 1;
                }
            });
            sign = acc;
            if count >= 5 {
                break;
            }
        }
        let listed = if sign.dot(n0) < 0.0 { -n0 } else { n0 };
        m.normal = listed;
        if prm.reorient_min_deg >= 180.0 {
            return;
        }
        if let Some(n) = sheet_normal(m, listed, &ours, pts, pos, grid, prm) {
            if n.angle_between(listed).to_degrees() > prm.reorient_min_deg {
                m.normal = n;
            }
        }
    });
}

/// Sheet normal at a site from the data (see [`orient_markers`]); `None` when the annulus is not
/// covered all round or the fit fails.
fn sheet_normal(
    m: &Marker,
    start: Vec3,
    ours: &dyn Fn(&WorkPoint) -> bool,
    pts: &[WorkPoint],
    pos: &[Vec3],
    grid: &PointGrid,
    prm: &MarkerConfig,
) -> Option<Vec3> {
    let mut n = start;
    let (mut acc, mut count) = (Vec3::ZERO, 0usize);
    grid.for_each_within(pos, m.center, 8.0, |i, _| {
        let p = &pts[i as usize];
        if ours(p) && p.n.dot(n) > 0.7 && n.dot(p.p - m.center).abs() < 1.5 {
            acc += p.n;
            count += 1;
        }
    });
    if count >= 20 && acc != Vec3::ZERO {
        n = acc.normalize();
    }
    let rc = cut_radius(m, prm);
    let r_out = rc + prm.fit_width_mm;
    for _ in 0..2 {
        let (eu, ev) = tangent_frame(n);
        let mut ann: Vec<(f64, f64, f64)> = Vec::new();
        let mut sector_count = [0u32; 8];
        grid.for_each_within(pos, m.center, (r_out * r_out + 4.0).sqrt(), |i, _| {
            let p = &pts[i as usize];
            if p.kind != PointKind::Measured || !ours(p) || p.n.dot(n) < 0.8 {
                return;
            }
            let d = p.p - m.center;
            let (u, v, h) = (d.dot(eu), d.dot(ev), d.dot(n));
            let r = (u * u + v * v).sqrt();
            if r >= rc && r < r_out && h.abs() < 2.0 {
                ann.push((u as f64, v as f64, h as f64));
                let s = (((v.atan2(u) + std::f32::consts::PI) / (std::f32::consts::TAU / 8.0)) as usize).min(7);
                sector_count[s] += 1;
            }
        });
        if sector_count.iter().filter(|&&c| c >= 6).count() < 7 {
            return None;
        }
        let (q, _, _) = Quadric::fit_robust(&ann)?;
        let (gu, gv) = q.grad(0.0, 0.0);
        let nl = DVec3::new(-gu, -gv, 1.0).normalize();
        n = (eu * nl.x as f32 + ev * nl.y as f32 + n * nl.z as f32).normalize();
    }
    Some(n)
}

/// Local frame of a site.
#[derive(Clone, Copy)]
struct SiteFrame {
    c: Vec3,
    n: Vec3,
    eu: Vec3,
    ev: Vec3,
    rc: f32,
    r_out: f32,
}

impl SiteFrame {
    fn new(m: &Marker, prm: &MarkerConfig) -> Self {
        let n = m.normal.normalize_or_zero();
        let n = if n == Vec3::ZERO { Vec3::Z } else { n };
        let (eu, ev) = tangent_frame(n);
        let rc = cut_radius(m, prm);
        Self { c: m.center, n, eu, ev, rc, r_out: rc + prm.fit_width_mm }
    }
    /// (u, v, h) of `p` in the frame.
    #[inline]
    fn local(&self, p: Vec3) -> (f32, f32, f32) {
        let d = p - self.c;
        (d.dot(self.eu), d.dot(self.ev), d.dot(self.n))
    }
}

/// An infill patch: the annulus quadric of a fillable site.
struct Patch {
    site: usize,
    f: SiteFrame,
    q: Quadric,
    scan: u8,
}

impl Patch {
    /// Point and normal of the patch under `p` (along the patch normal) and the depth of `p`
    /// inside the cut (1 at the axis, 0 at the lattice edge), when `p` lies inside the cut and
    /// within `hc` of the patch.
    fn cover(&self, p: Vec3, hc: f32) -> Option<(Vec3, Vec3, f32)> {
        let (u, v, h) = self.f.local(p);
        let r = (u * u + v * v).sqrt();
        let edge = self.f.rc + 0.1;
        if r > edge {
            return None;
        }
        let z = self.q.eval(u as f64, v as f64);
        if (h as f64 - z).abs() > hc as f64 {
            return None;
        }
        Some((p + self.f.n * (z as f32 - h), self.normal(u, v), 1.0 - r / edge))
    }
    fn normal(&self, u: f32, v: f32) -> Vec3 {
        let (gu, gv) = self.q.grad(u as f64, v as f64);
        let nl = DVec3::new(-gu, -gv, 1.0).normalize();
        (self.f.eu * nl.x as f32 + self.f.ev * nl.y as f32 + self.f.n * nl.z as f32).normalize()
    }
}

/// Removes marker sites from `pts` and (optionally) adds quadric infill points.
/// Returns the per-site outcomes.
///
/// Three steps: the cut of every site; the annulus fit of every site without the points any site
/// cuts (a neighbour's raised sticker would bias the fit); the infill lattices, where a lattice
/// point inside the cuts of several sites on the same sheet is emitted only by the site it lies
/// deepest in, on the depth-weighted blend of the covering patches.
pub(crate) fn cut_and_fill(pts: &mut Vec<WorkPoint>, markers: &[Marker], prm: &MarkerConfig) -> Vec<MarkerOutcome> {
    if markers.is_empty() {
        return Vec::new();
    }
    let pos: Vec<Vec3> = pts.iter().map(|p| p.p).collect();
    let grid = PointGrid::new(&pos, 2.0);
    let frames: Vec<SiteFrame> = markers.iter().map(|m| SiteFrame::new(m, prm)).collect();

    let cuts: Vec<Vec<u32>> = frames.par_iter().map(|f| site_cut(f, pts, &pos, &grid, prm)).collect();
    let mut in_cut = vec![false; pts.len()];
    for c in &cuts {
        for &i in c {
            in_cut[i as usize] = true;
        }
    }

    // Per site: the outcome, the patch quadric (fillable sites) and whether the cut is applied.
    let fits: Vec<(MarkerOutcome, Option<Quadric>, bool)> = markers
        .par_iter()
        .zip(frames.par_iter())
        .zip(cuts.par_iter())
        .map(|((m, f), cut)| site_fit(m, f, cut.len(), pts, &pos, &grid, &in_cut, prm))
        .collect();

    let patches: Vec<Patch> = fits
        .iter()
        .enumerate()
        .filter_map(|(k, (_, q, _))| q.map(|q| Patch { site: k, f: frames[k], q, scan: markers[k].scan.unwrap_or(0) }))
        .collect();
    let fills: Vec<Vec<WorkPoint>> = (0..patches.len()).into_par_iter().map(|a| lattice(a, &patches, prm)).collect();

    let mut remove = vec![false; pts.len()];
    let mut outcomes = Vec::with_capacity(markers.len());
    for ((out, _, apply), cut) in fits.into_iter().zip(&cuts) {
        if apply {
            for &i in cut {
                remove[i as usize] = true;
            }
        }
        outcomes.push(out);
    }
    for (pa, fill) in patches.iter().zip(&fills) {
        outcomes[pa.site].fill_points = fill.len();
    }
    let mut k = 0usize;
    pts.retain(|_| {
        let keep = !remove[k];
        k += 1;
        keep
    });
    pts.extend(fills.into_iter().flatten());
    outcomes
}

/// Points of a site's cut: every scan's points close to the marker's sheet whatever their normal
/// (the disc edge and the dark ring carry wild normals); further out along the axis only points
/// facing like the sheet, so the opposite wall one thickness away is never cut.
fn site_cut(f: &SiteFrame, pts: &[WorkPoint], pos: &[Vec3], grid: &PointGrid, prm: &MarkerConfig) -> Vec<u32> {
    let hc = prm.cut_height_mm;
    let mut cut = Vec::new();
    grid.for_each_within(pos, f.c, (f.rc * f.rc + hc * hc).sqrt(), |i, _| {
        let p = &pts[i as usize];
        if p.kind != PointKind::Measured {
            return;
        }
        let (u, v, h) = f.local(p.p);
        let facing = p.n.dot(f.n) >= 0.5;
        if u * u + v * v < f.rc * f.rc && (h.abs() < hc.min(1.3) || (facing && h.abs() < hc)) {
            cut.push(i);
        }
    });
    cut
}

/// Annulus fit of one site. Returns the outcome, the patch quadric when the site is filled, and
/// whether its cut is applied.
#[allow(clippy::too_many_arguments)]
fn site_fit(
    m: &Marker,
    f: &SiteFrame,
    cut_points: usize,
    pts: &[WorkPoint],
    pos: &[Vec3],
    grid: &PointGrid,
    in_cut: &[bool],
    prm: &MarkerConfig,
) -> (MarkerOutcome, Option<Quadric>, bool) {
    let mut ann: Vec<(f64, f64, f64)> = Vec::new();
    let mut sector_count = [0u32; 8];
    grid.for_each_within(pos, f.c, (f.r_out * f.r_out + 4.0).sqrt(), |i, _| {
        let p = &pts[i as usize];
        if p.kind != PointKind::Measured || p.n.dot(f.n) < 0.5 {
            return;
        }
        if !prm.fit_all_scans && m.scan.map_or(false, |s| s != p.scan) {
            return;
        }
        let (u, v, h) = f.local(p.p);
        let r = (u * u + v * v).sqrt();
        if r >= f.rc && r < f.r_out && h.abs() < 2.0 {
            // A neighbour's cut is surface all the same (coverage), but its sticker is not fitted.
            let s = (((v.atan2(u) + std::f32::consts::PI) / (std::f32::consts::TAU / 8.0)) as usize).min(7);
            sector_count[s] += 1;
            if !in_cut[i as usize] {
                ann.push((u as f64, v as f64, h as f64));
            }
        }
    });
    let sectors = sector_count.iter().filter(|&&c| c >= 6).count() as u32;
    let mut out = MarkerOutcome {
        id: m.id.clone(),
        scan: m.scan,
        source: m.source,
        center: m.center,
        cut_points,
        annulus_points: ann.len(),
        annulus_rms_mm: f32::NAN,
        sectors_covered: sectors,
        fill_points: 0,
        fill: String::new(),
        fair: None,
    };
    if !prm.quadric_fill {
        out.fill = "implicit (M1)".into();
        return (out, None, true);
    }
    // A site that cannot be patched is either left to the solve (cut, M1 behaviour) or kept.
    let unfillable = |mut out: MarkerOutcome, why: String| {
        if prm.keep_unfillable {
            out.fill = format!("kept (not cut): {why}");
            out.cut_points = 0;
            (out, None, false)
        } else {
            out.fill = format!("implicit: {why}");
            (out, None, true)
        }
    };
    if sectors < 6 {
        return unfillable(out, format!("annulus covers {sectors}/8 sectors"));
    }
    let Some((q, rms, _)) = Quadric::fit_robust(&ann) else {
        return unfillable(out, "quadric fit failed".into());
    };
    out.annulus_rms_mm = rms as f32;
    if rms > prm.fill_rms_limit_mm as f64 {
        return unfillable(out, format!("annulus RMS {:.3} mm", rms));
    }
    out.fill = if rms > prm.fit_rms_max_mm as f64 { format!("quadric, rough annulus ({:.3} mm)", rms) } else { "quadric".into() };
    (out, Some(q), true)
}

/// Infill lattice of patch `a` (inside the cut, slightly overlapping the annulus edge). Where
/// other patches on the same sheet cover a lattice point, the point is left to the patch it lies
/// deepest in (ties to the lower index) and placed on the depth-weighted blend of all covering
/// patches; the owner computes the same blend, so neighbouring lattices meet without a step.
fn lattice(a: usize, patches: &[Patch], prm: &MarkerConfig) -> Vec<WorkPoint> {
    let pa = &patches[a];
    let f = &pa.f;
    let hc = prm.cut_height_mm;
    let others: Vec<usize> = (0..patches.len())
        .filter(|&b| {
            let pb = &patches[b];
            b != a && pb.f.n.dot(f.n) > 0.8 && pb.f.c.distance(f.c) < f.rc + pb.f.rc + 0.5
        })
        .collect();
    let s = prm.fill_spacing_mm;
    let k = ((f.rc + 0.25) / s).ceil() as i32;
    let edge = f.rc + 0.1;
    let mut fill = Vec::new();
    for i in -k..=k {
        'point: for j in -k..=k {
            let (u, v) = (i as f32 * s, j as f32 * s);
            let r = (u * u + v * v).sqrt();
            if r > edge {
                continue;
            }
            let z = pa.q.eval(u as f64, v as f64) as f32;
            let p = f.c + f.eu * u + f.ev * v + f.n * z;
            let own_depth = 1.0 - r / edge;
            let w0 = own_depth + 1e-3;
            let (mut sp, mut sn, mut sw) = (p * w0, pa.normal(u, v) * w0, w0);
            for &b in &others {
                let Some((pb, nb, depth)) = patches[b].cover(p, hc) else { continue };
                if depth > own_depth || (depth == own_depth && b < a) {
                    continue 'point;
                }
                let w = depth + 1e-3;
                sp += pb * w;
                sn += nb * w;
                sw += w;
            }
            let nn = sn.normalize_or_zero();
            let nn = if nn == Vec3::ZERO { f.n } else { nn };
            fill.push(WorkPoint { p: sp / sw, n: nn, scan: pa.scan, kind: PointKind::MarkerFill, area: 0.0 });
        }
    }
    fill
}

/// Radius of the cut at a site.
pub(crate) fn cut_radius(m: &Marker, prm: &MarkerConfig) -> f32 {
    prm.cut_radius_mm.max(m.radius_mm + 1.0)
}

/// Merges sites whose centres lie within `merge_mm` of each other and whose normals agree (the
/// same sticker listed once per scan, or listed and detected). The merged site sits at the mean
/// centre, keeps a scan id only when all members share it, and its radius grows by the spread of
/// the members so the cut still covers every member's disc.
pub(crate) fn merge_sites(sites: &[Marker], prm: &MarkerConfig) -> Vec<Marker> {
    let mut used = vec![false; sites.len()];
    let mut out = Vec::new();
    for i in 0..sites.len() {
        if used[i] {
            continue;
        }
        used[i] = true;
        let mut group = vec![i];
        // Grow transitively so chains of per-scan copies end up together.
        let mut k = 0;
        while k < group.len() {
            let a = &sites[group[k]];
            for j in 0..sites.len() {
                if !used[j]
                    && sites[j].center.distance(a.center) < prm.merge_mm
                    && sites[j].normal.normalize_or_zero().dot(a.normal.normalize_or_zero()) > 0.8
                {
                    used[j] = true;
                    group.push(j);
                }
            }
            k += 1;
        }
        if group.len() == 1 {
            out.push(sites[i].clone());
            continue;
        }
        let c = group.iter().map(|&j| sites[j].center).sum::<Vec3>() / group.len() as f32;
        let n = group.iter().map(|&j| sites[j].normal.normalize_or_zero()).sum::<Vec3>().normalize_or_zero();
        let spread = group.iter().map(|&j| sites[j].center.distance(c)).fold(0f32, f32::max);
        let r = group.iter().map(|&j| cut_radius(&sites[j], prm) - 1.0).fold(0f32, f32::max);
        let scan = sites[i].scan.filter(|s| group.iter().all(|&j| sites[j].scan == Some(*s)));
        out.push(Marker {
            id: group.iter().map(|&j| sites[j].id.as_str()).collect::<Vec<_>>().join("+"),
            center: c,
            normal: if n == Vec3::ZERO { sites[i].normal } else { n },
            scan,
            source: sites[i].source,
            radius_mm: r + spread,
            score: group.iter().map(|&j| sites[j].score).fold(0f32, f32::max),
        });
    }
    out
}

/// Mesh fairing of the marker sites. For every site the mesh vertices on the site's sheet (normal
/// within 60° of the site normal, within 2 mm of the site plane) in the annulus
/// `cut .. cut + fit_width` are fitted with a robust quadric height field; vertices inside the
/// cut of another site on the same sheet count for the coverage but are not fitted. When the
/// annulus is smooth (RMS ≤ `fair_rms_max_mm`) and covered all round, every vertex inside the cut
/// (weight 1) and, fading out, in a `fair_blend_mm` band beyond it that lies on the fitted sheet
/// (within 0.8 mm and facing like it within 25°) is moved along the site normal onto the quadric.
/// A vertex moved by several sites gets the weighted mean of their displacements, applied with
/// the largest of their weights, so overlapping sites neither undo each other nor leave a step.
/// Edges, ribs and the opposite wall are not on the fitted sheet and stay where they are;
/// topology does not change. Marks the moved vertices inside a cut in `inside_cut`.
pub(crate) fn fair_sites(
    pos: &mut [Vec3],
    vnormal: &[Vec3],
    sites: &[Marker],
    prm: &MarkerConfig,
    inside_cut: &mut [bool],
) -> Vec<FairOutcome> {
    let grid = PointGrid::new(pos, 2.0);
    let snapshot: &[Vec3] = pos;
    let frames: Vec<SiteFrame> = sites.iter().map(|m| SiteFrame::new(m, prm)).collect();
    let per_site: Vec<(FairOutcome, Vec<FairMove>)> =
        (0..sites.len()).into_par_iter().map(|k| fair_one(k, &frames, snapshot, vnormal, &grid, prm)).collect();
    // Per moved vertex: Σ w·d, Σ w, max w.
    let mut acc: FxMap<u32, (Vec3, f32, f32)> = FxMap::default();
    let mut out = Vec::with_capacity(sites.len());
    for (o, moves) in per_site {
        for mv in moves {
            let e = acc.entry(mv.vertex).or_insert((Vec3::ZERO, 0.0, 0.0));
            e.0 += mv.delta * mv.weight;
            e.1 += mv.weight;
            e.2 = e.2.max(mv.weight);
            if mv.inside {
                inside_cut[mv.vertex as usize] = true;
            }
        }
        out.push(o);
    }
    for (i, (swd, sw, wmax)) in acc {
        if sw > 0.0 {
            pos[i as usize] += swd / sw * wmax;
        }
    }
    out
}

/// One vertex move of a site: full displacement onto the site's quadric and its blend weight.
struct FairMove {
    vertex: u32,
    delta: Vec3,
    weight: f32,
    inside: bool,
}

fn fair_one(
    k: usize,
    frames: &[SiteFrame],
    pos: &[Vec3],
    vnormal: &[Vec3],
    grid: &PointGrid,
    prm: &MarkerConfig,
) -> (FairOutcome, Vec<FairMove>) {
    let nan = f32::NAN;
    let fail = |status: String, ring_rms_mm: f32| {
        (FairOutcome { status, ring_rms_mm, mean_before_mm: nan, max_before_mm: nan, vertices: 0 }, Vec::new())
    };
    let f = &frames[k];
    let n = f.n;
    // Other sites on this sheet whose cut can reach the annulus.
    let others: Vec<&SiteFrame> = frames
        .iter()
        .enumerate()
        .filter(|&(j, g)| j != k && g.n.dot(n) > 0.5 && (g.c - f.c).dot(n).abs() < 2.0 && g.c.distance(f.c) < f.r_out + g.rc + 1.0)
        .map(|(_, g)| g)
        .collect();
    let in_other_cut = |p: Vec3| {
        others.iter().any(|g| {
            let (u, v, h) = g.local(p);
            u * u + v * v < g.rc * g.rc && h.abs() < 2.0
        })
    };
    let (rc, r_out) = (f.rc, f.r_out);
    let mut ann: Vec<(f64, f64, f64)> = Vec::new();
    let mut cand: Vec<u32> = Vec::new();
    let mut sector_count = [0u32; 8];
    grid.for_each_within(pos, f.c, (r_out * r_out + 4.0).sqrt(), |i, _| {
        let p = pos[i as usize];
        if vnormal[i as usize].dot(n) < 0.5 {
            return;
        }
        let (u, v, h) = f.local(p);
        if h.abs() > 2.0 {
            return;
        }
        let r = (u * u + v * v).sqrt();
        if r < rc + prm.fair_blend_mm {
            cand.push(i);
        }
        if r >= rc && r < r_out {
            let s = (((v.atan2(u) + std::f32::consts::PI) / (std::f32::consts::TAU / 8.0)) as usize).min(7);
            sector_count[s] += 1;
            if !in_other_cut(p) {
                ann.push((u as f64, v as f64, h as f64));
            }
        }
    });
    let sectors = sector_count.iter().filter(|&&c| c >= 6).count();
    if sectors < 7 {
        return fail(format!("annulus covers {sectors}/8 sectors"), nan);
    }
    let Some((q, rms, kept)) = Quadric::fit_robust(&ann) else {
        return fail("quadric fit failed".into(), nan);
    };
    if rms > prm.fair_rms_max_mm as f64 || (kept as f64) < 0.6 * ann.len() as f64 {
        return fail(format!("annulus RMS {:.3} mm ({kept}/{} kept)", rms, ann.len()), rms as f32);
    }
    let (mut sum, mut cnt, mut max_before) = (0f64, 0usize, 0f64);
    let mut moves = Vec::new();
    for &i in &cand {
        let p = pos[i as usize];
        let (u, v, h) = f.local(p);
        let (u, v, h) = (u as f64, v as f64, h as f64);
        let z = q.eval(u, v);
        let dz = z - h;
        if dz.abs() > 0.8 {
            continue;
        }
        let (gu, gv) = q.grad(u, v);
        let nl = DVec3::new(-gu, -gv, 1.0).normalize();
        let nq = (f.eu * nl.x as f32 + f.ev * nl.y as f32 + n * nl.z as f32).normalize();
        if vnormal[i as usize].dot(nq) < 0.9 {
            continue;
        }
        let r = (u * u + v * v).sqrt() as f32;
        if r < rc {
            sum += -dz;
            cnt += 1;
            max_before = max_before.max(dz.abs());
        }
        let w = if r <= rc {
            1.0
        } else {
            let t = ((r - rc) / prm.fair_blend_mm).clamp(0.0, 1.0);
            1.0 - t * t * (3.0 - 2.0 * t)
        };
        if w > 0.0 {
            moves.push(FairMove { vertex: i, delta: n * dz as f32, weight: w, inside: r < rc });
        }
    }
    if cnt < 20 {
        return fail(format!("only {cnt} mesh vertices on the sheet inside the cut"), rms as f32);
    }
    let out = FairOutcome {
        status: "faired".into(),
        ring_rms_mm: rms as f32,
        mean_before_mm: (sum / cnt as f64) as f32,
        max_before_mm: max_before as f32,
        vertices: moves.len(),
    };
    (out, moves)
}

/// Detector settings (raised discs and round holes).
#[derive(Clone, Debug)]
pub struct DetectConfig {
    pub seed_spacing_mm: f32,
    pub window_mm: f32,
    pub annulus_inner_mm: f32,
    pub annulus_rms_max_mm: f32,
    pub disc_height_min_mm: f32,
    pub disc_height_max_mm: f32,
    pub hole_radius_min_mm: f32,
    pub hole_radius_max_mm: f32,
    pub nms_mm: f32,
    pub detect_discs: bool,
    pub detect_holes: bool,
}

impl Default for DetectConfig {
    fn default() -> Self {
        Self {
            seed_spacing_mm: 2.0,
            window_mm: 13.0,
            annulus_inner_mm: 7.0,
            annulus_rms_max_mm: 0.06,
            disc_height_min_mm: 0.15,
            disc_height_max_mm: 0.9,
            hole_radius_min_mm: 3.0,
            hole_radius_max_mm: 7.0,
            nms_mm: 8.0,
            detect_discs: true,
            detect_holes: true,
        }
    }
}

/// Finds marker-like sites per scan: raised flat discs (raw previews) and enclosed round holes
/// (clouds where the marker was already cut out). Detections are sorted by score and
/// non-maximum suppressed per scan.
pub(crate) fn detect(pts: &[WorkPoint], prm: &DetectConfig) -> Vec<Marker> {
    let pos: Vec<Vec3> = pts.iter().map(|p| p.p).collect();
    let grid = PointGrid::new(&pos, 2.0);
    let seeds_grid = PointGrid::new(&pos, prm.seed_spacing_mm);
    let seeds: Vec<u32> = seeds_grid
        .representatives()
        .into_iter()
        .filter(|&i| pts[i as usize].kind == PointKind::Measured)
        .collect();
    drop(seeds_grid);

    let mut found: Vec<Marker> = seeds
        .par_iter()
        .filter_map(|&si| detect_at(si as usize, pts, &pos, &grid, prm))
        .collect();
    found.sort_by(|a, b| b.score.total_cmp(&a.score));
    let mut kept: Vec<Marker> = Vec::new();
    for m in found {
        if kept.iter().any(|k| k.scan == m.scan && k.center.distance(m.center) < prm.nms_mm) {
            continue;
        }
        kept.push(m);
    }
    for (i, m) in kept.iter_mut().enumerate() {
        m.id = format!("D{:02}", i + 1);
    }
    kept
}

fn detect_at(si: usize, pts: &[WorkPoint], pos: &[Vec3], grid: &PointGrid, prm: &DetectConfig) -> Option<Marker> {
    let s = &pts[si];
    let n = s.n;
    let (eu, ev) = tangent_frame(n);
    let w = prm.window_mm;
    let mut loc: Vec<(f64, f64, f64)> = Vec::new();
    grid.for_each_within(pos, s.p, w, |i, _| {
        let p = &pts[i as usize];
        if p.scan != s.scan || p.kind != PointKind::Measured || p.n.dot(n) < 0.6 {
            return;
        }
        let d = p.p - s.p;
        let h = d.dot(n);
        if h.abs() < 3.0 {
            loc.push((d.dot(eu) as f64, d.dot(ev) as f64, h as f64));
        }
    });
    if loc.len() < 60 {
        return None;
    }
    let mk = |cu: f64, cv: f64, q: &Quadric, src: MarkerSource, radius: f32, score: f32| {
        let c = s.p + eu * cu as f32 + ev * cv as f32 + n * q.eval(cu, cv) as f32;
        let (gu, gv) = q.grad(cu, cv);
        let nl = DVec3::new(-gu, -gv, 1.0).normalize();
        let nn = (eu * nl.x as f32 + ev * nl.y as f32 + n * nl.z as f32).normalize();
        Marker { id: String::new(), center: c, normal: nn, scan: Some(s.scan), source: src, radius_mm: radius, score }
    };

    if prm.detect_discs {
        let a_in = prm.annulus_inner_mm as f64;
        let ann: Vec<(f64, f64, f64)> =
            loc.iter().copied().filter(|&(u, v, _)| (u * u + v * v).sqrt() >= a_in).collect();
        if let Some((q, rms, _)) = Quadric::fit_robust(&ann) {
            if rms <= prm.annulus_rms_max_mm as f64 && sectors(&ann, 0.0, 0.0, 8, 4) >= 7 {
                let res = |&(u, v, z): &(f64, f64, f64)| z - q.eval(u, v);
                let mut centre: Vec<f64> =
                    loc.iter().filter(|&&(u, v, _)| u * u + v * v < 9.0).map(res).collect();
                let mut ring: Vec<f64> = loc
                    .iter()
                    .filter(|&&(u, v, _)| {
                        let r = (u * u + v * v).sqrt();
                        (5.7..a_in).contains(&r)
                    })
                    .map(res)
                    .collect();
                if centre.len() >= 10 && ring.len() >= 10 {
                    let mc = median(&mut centre);
                    let mr = median(&mut ring);
                    let mut dev: Vec<f64> = centre.iter().map(|r| (r - mc).abs()).collect();
                    let mad = median(&mut dev);
                    if mc >= prm.disc_height_min_mm as f64
                        && mc <= prm.disc_height_max_mm as f64
                        && mc >= 5.0 * rms
                        && mr < 0.35 * mc
                        && mad < 0.3 * mc
                    {
                        // Recentre on the raised points.
                        let (mut su, mut sv, mut sw) = (0.0, 0.0, 0.0);
                        let mut far = 0usize;
                        for t in &loc {
                            let r2 = t.0 * t.0 + t.1 * t.1;
                            if r2 < 49.0 && res(t) > 0.5 * mc {
                                su += t.0;
                                sv += t.1;
                                sw += 1.0;
                                if r2 > 36.0 {
                                    far += 1;
                                }
                            }
                        }
                        if sw >= 10.0 && (far as f64) < 0.1 * sw {
                            let (cu, cv) = (su / sw, sv / sw);
                            // Disc radius: furthest raised point from the new centre (robust p90).
                            let mut rr: Vec<f64> = loc
                                .iter()
                                .filter(|t| res(t) > 0.5 * mc)
                                .map(|t| ((t.0 - cu).powi(2) + (t.1 - cv).powi(2)).sqrt())
                                .filter(|r| *r < 7.0)
                                .collect();
                            rr.sort_by(|a, b| a.total_cmp(b));
                            let rad = rr[(rr.len() * 9 / 10).min(rr.len() - 1)] as f32;
                            if (2.5..6.0).contains(&rad) {
                                return Some(mk(cu, cv, &q, MarkerSource::DetectedDisc, rad, (mc / rms.max(0.005)) as f32));
                            }
                        }
                    }
                }
            }
        }
    }

    if prm.detect_holes {
        // Occupancy image of the sheet (0.5 mm pixels) around the seed.
        let (q, rms, _) = Quadric::fit_robust(&loc)?;
        if rms > 0.15 {
            return None;
        }
        const PX: f64 = 0.5;
        let half = (w as f64 / PX).floor() as i32;
        let side = (2 * half + 1) as usize;
        let mut occ = vec![false; side * side];
        for &(u, v, z) in &loc {
            if (z - q.eval(u, v)).abs() > 0.6 {
                continue;
            }
            let (iu, iv) = ((u / PX).round() as i32 + half, (v / PX).round() as i32 + half);
            if iu < 0 || iv < 0 || iu >= side as i32 || iv >= side as i32 {
                continue;
            }
            // Dilate by one pixel to close the sampling gaps of sparse scans.
            for dv in -1..=1 {
                for du in -1..=1 {
                    let (a, b) = (iu + du, iv + dv);
                    if a >= 0 && b >= 0 && a < side as i32 && b < side as i32 {
                        occ[b as usize * side + a as usize] = true;
                    }
                }
            }
        }
        // Pixel-disc mask (outside the window circle counts as unknown, not empty).
        let inside = |a: i32, b: i32| {
            let (u, v) = ((a - half) as f64 * PX, (b - half) as f64 * PX);
            u * u + v * v <= (w as f64) * (w as f64)
        };
        let dt = chamfer_dt(&occ, side);
        // Best empty pixel within 6 mm of the seed.
        let lim = (6.0 / PX) as i32;
        let mut best = (0f32, 0i32, 0i32);
        for b in (half - lim).max(0)..=(half + lim).min(side as i32 - 1) {
            for a in (half - lim).max(0)..=(half + lim).min(side as i32 - 1) {
                let d = dt[b as usize * side + a as usize];
                if d > best.0 && inside(a, b) {
                    best = (d, a, b);
                }
            }
        }
        // DT is in pixels to the nearest occupied pixel; occupancy was dilated by one pixel.
        let radius = (best.0 as f64 + 1.0) * PX;
        if radius < prm.hole_radius_min_mm as f64 || radius > prm.hole_radius_max_mm as f64 {
            return None;
        }
        let (cu, cv) = ((best.1 - half) as f64 * PX, (best.2 - half) as f64 * PX);
        if (cu * cu + cv * cv).sqrt() + radius + 3.0 > w as f64 {
            return None;
        }
        // Enclosed: the ring radius+0.5 .. radius+3 must be occupied in at least 11 of 12 sectors.
        let mut hit = [0u32; 12];
        let mut tot = [0u32; 12];
        for b in 0..side as i32 {
            for a in 0..side as i32 {
                let (u, v) = ((a - half) as f64 * PX - cu, (b - half) as f64 * PX - cv);
                let r = (u * u + v * v).sqrt();
                if r < radius + 0.5 || r > radius + 3.0 {
                    continue;
                }
                let s = (((v.atan2(u) + std::f64::consts::PI) / (std::f64::consts::TAU / 12.0)) as usize).min(11);
                tot[s] += 1;
                if occ[b as usize * side + a as usize] {
                    hit[s] += 1;
                }
            }
        }
        let good = (0..12).filter(|&s| tot[s] > 0 && hit[s] as f64 >= 0.6 * tot[s] as f64).count();
        if good >= 11 {
            return Some(mk(cu, cv, &q, MarkerSource::DetectedHole, radius as f32, radius as f32));
        }
    }
    None
}

fn sectors(s: &[(f64, f64, f64)], cu: f64, cv: f64, k: usize, min: u32) -> usize {
    let mut c = vec![0u32; k];
    for &(u, v, _) in s {
        let a = (v - cv).atan2(u - cu) + std::f64::consts::PI;
        let i = ((a / (std::f64::consts::TAU / k as f64)) as usize).min(k - 1);
        c[i] += 1;
    }
    c.iter().filter(|&&x| x >= min).count()
}

fn median(v: &mut [f64]) -> f64 {
    let m = v.len() / 2;
    v.select_nth_unstable_by(m, |a, b| a.total_cmp(b));
    v[m]
}

/// Two-pass 3-4 chamfer distance (in pixels) from every pixel to the nearest occupied pixel.
pub(crate) fn chamfer_dt(occ: &[bool], side: usize) -> Vec<f32> {
    let inf = 1e9f32;
    let mut d: Vec<f32> = occ.iter().map(|&o| if o { 0.0 } else { inf }).collect();
    let s = side as i32;
    let idx = |a: i32, b: i32| (b * s + a) as usize;
    for b in 0..s {
        for a in 0..s {
            let mut m = d[idx(a, b)];
            if a > 0 {
                m = m.min(d[idx(a - 1, b)] + 3.0);
            }
            if b > 0 {
                m = m.min(d[idx(a, b - 1)] + 3.0);
                if a > 0 {
                    m = m.min(d[idx(a - 1, b - 1)] + 4.0);
                }
                if a + 1 < s {
                    m = m.min(d[idx(a + 1, b - 1)] + 4.0);
                }
            }
            d[idx(a, b)] = m;
        }
    }
    for b in (0..s).rev() {
        for a in (0..s).rev() {
            let mut m = d[idx(a, b)];
            if a + 1 < s {
                m = m.min(d[idx(a + 1, b)] + 3.0);
            }
            if b + 1 < s {
                m = m.min(d[idx(a, b + 1)] + 3.0);
                if a + 1 < s {
                    m = m.min(d[idx(a + 1, b + 1)] + 4.0);
                }
                if a > 0 {
                    m = m.min(d[idx(a - 1, b + 1)] + 4.0);
                }
            }
            d[idx(a, b)] = m;
        }
    }
    d.into_iter().map(|x| x / 3.0).collect()
}
