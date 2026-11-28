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
//! 2. **Infill:** the fill surface ([`super::sitefit`]: a robust conic quadric of the sheet in the
//!    annulus `cut_radius .. cut_radius + fit_width` and, where a curvature band runs through the
//!    site, its low angular orders continued into the cut with value and slope matched at its edge,
//!    so the fill carries on the curvature of the surface round it instead of standing out as a
//!    flat spot or a ring) is fitted to the points of every scan (so the patch follows the average
//!    the solve makes of overlapping passes) and sampled inside the cut on a 0.3 mm lattice. A site
//!    whose annulus is not covered in enough sectors (a rim) or too rough (a rib or edge nearby) is
//!    not cut at all (`keep_unfillable`), since an unfilled cut can open a hole.
//!    Points that any site cuts (a neighbouring sticker) count for the coverage of an annulus but
//!    are not fitted. At a site not covered all round, cut points more than 0.3 mm below or 1 mm
//!    above its patch are the surface beyond the sheet (a rim's rolled edge, a side face, a rib
//!    foot): they are not cut and the patch keeps clear of them. Where the cuts of two sites on one
//!    sheet overlap, each lattice point belongs to the site it lies deepest in and is placed on a
//!    blend of the covering patches, so the overlap carries one layer without a step. A patch of a
//!    site not covered all round stays within the convex hull of the sheet round it (the annulus
//!    and the cut's outer 0.4 mm, beyond the sticker) and never reaches behind surface going down
//!    in the cut (a rim's side face under a sticker hanging over the rim: six cut points or more
//!    below the patch, in directions without annulus surface).
//! 3. **Fairing** ([`fair_sites`], after meshing): where the mesh round the cut is smooth sheet,
//!    the mesh inside the cut goes onto the site's fill surface (a fit of the mesh annulus at a
//!    site that has none), met by the mesh round the cut, which removes any dent or bump the solve
//!    left. Overlapping sites share the vertices they both move (weighted mean of their
//!    displacements).

use super::grid::{FxMap, PointGrid};
use super::sitefit::{convex_hull, fit_site, outside_hull, RingSample, SiteFitParams, SiteSurface, RING_NORMAL_MAX_DEG};
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
    /// After meshing, put the mesh inside each site's cut onto the site's fill surface (or, for a
    /// site that was kept or left to the solve, onto a fit of the mesh annulus), met by the mesh
    /// round the cut ([`fair_sites`]).
    pub fair_mesh: bool,
    /// Mesh annulus RMS limit for the fairing (the mesh is smoother than the data): a site whose
    /// mesh annulus is rougher (a rim, an edge) is left as meshed.
    pub fair_rms_max_mm: f32,
    /// Width of the band beyond the cut whose residual against the fit is carried into the faired
    /// patch (see [`fair_sites`]).
    pub fair_blend_mm: f32,
    /// A listed marker normal whose direction disagrees with the data ([`orient_markers`]) by
    /// more than this (degrees) is replaced by the data's; closer ones keep their direction (near
    /// ribs and edges the tracker's normal is the better one). 180 or more: only the sign is
    /// taken from the data.
    pub reorient_min_deg: f32,
    /// A site whose annulus is covered in at least this many of 8 sectors, but not all round (a
    /// sticker near the rim or an edge of the scan), is still cut and patched, and faired on the
    /// mesh. 8 or more disables it. Any site not covered all round is patched only where the cut
    /// took points or the annulus has surface in that direction, so the patch never reaches past
    /// the scanned surface.
    pub partial_min_sectors: u32,
    /// Grow a site's cut over a sparse ring around it by up to this much (mm; 0 = off), see
    /// [`widen_over_sparse_rings`].
    pub widen_max_mm: f32,
    /// Where a curvature band or a ridge runs through a site (the band next to the cut is plain
    /// sheet all round and the ring fit clearly misses it), continue the band's slope into the
    /// cut (C1) instead of filling with the ring fit alone ([`super::sitefit`]). Needs the passes
    /// to agree round the site: off for a merged model whose passes cannot be told apart.
    pub continue_band_slopes: bool,
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
            partial_min_sectors: 4,
            widen_max_mm: 5.0,
            continue_band_slopes: true,
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
    /// The fill surface (filled sites): the mesh fairing puts the mesh inside the cut on it.
    pub(crate) surface: Option<SiteSurface>,
}

/// What the mesh fairing did at one site.
#[derive(Clone, Debug)]
pub struct FairOutcome {
    /// "faired", or why the site was left as meshed.
    pub status: String,
    /// Mesh annulus RMS against a robust quadric of its own (mm; the fairing's quality check).
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

/// Least-squares height field z = c0 + c1 u + c2 v + c3 u² + c4 uv + c5 v², optionally with the
/// cubic terms c6 u³ + c7 u²v + c8 uv² + c9 v³ ([`Quadric::fit_robust_cubic`]; zero otherwise).
///
/// With c10 = κ > 0 the quadratic part Q = c3 u² + c4 uv + c5 v² enters as the sag of a conic,
/// 2Q / (1 + sqrt(1 − 2κ|Q|)), κ the largest principal curvature of Q ([`Quadric::fit_conic`]):
/// exact on a sphere or a cylinder of any radius, where a quadric fitted to a ring round a site
/// misses the centre by about r⁴/8R³ (0.11 mm on a ball of R 15 mm), and the same as Q on a gently
/// curved sheet.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Quadric(pub [f64; 11]);

impl Quadric {
    /// The conic factor of the quadratic part: g = 2 / (1 + s) and d(gQ)/dQ, s = sqrt(1 − 2κ|Q|).
    #[inline]
    fn conic(&self, q2: f64) -> (f64, f64) {
        let k = self.0[10];
        if k == 0.0 {
            return (1.0, 1.0);
        }
        let s = (1.0 - 2.0 * k * q2.abs()).max(0.0025).sqrt();
        let g = 2.0 / (1.0 + s);
        (g, g + 2.0 * k * q2.abs() / (s * (1.0 + s) * (1.0 + s)))
    }
    #[inline]
    pub fn eval(&self, u: f64, v: f64) -> f64 {
        let c = &self.0;
        let q2 = c[3] * u * u + c[4] * u * v + c[5] * v * v;
        c[0] + c[1] * u + c[2] * v + self.conic(q2).0 * q2 + c[6] * u * u * u + c[7] * u * u * v + c[8] * u * v * v + c[9] * v * v * v
    }
    #[inline]
    pub fn grad(&self, u: f64, v: f64) -> (f64, f64) {
        let c = &self.0;
        let d = self.conic(c[3] * u * u + c[4] * u * v + c[5] * v * v).1;
        (
            c[1] + d * (2.0 * c[3] * u + c[4] * v) + 3.0 * c[6] * u * u + 2.0 * c[7] * u * v + c[8] * v * v,
            c[2] + d * (c[4] * u + 2.0 * c[5] * v) + c[7] * u * u + 2.0 * c[8] * u * v + 3.0 * c[9] * v * v,
        )
    }

    /// Largest principal curvature magnitude of the quadratic part at the apex.
    fn kappa_max(&self) -> f64 {
        let (a, b, d) = (self.0[3], 0.5 * self.0[4], self.0[5]);
        let (m, r) = (0.5 * (a + d), (0.25 * (a - d) * (a - d) + b * b).sqrt());
        2.0 * (m.abs() + r)
    }

    /// The quadric over the samples marked in `keep`, refined to a conic (see the type) unless that
    /// fits them clearly worse: a few fixed-point steps, each a linear fit with the conic factor of
    /// the previous fit.
    pub(crate) fn fit_conic(s: &[(f64, f64, f64)], keep: &[bool]) -> Option<Quadric> {
        let quad = Self::fit_once::<6>(s, keep)?;
        let rq = rms_of(s, keep, &quad);
        let mut q = quad;
        for _ in 0..6 {
            let k = q.kappa_max();
            if !(k > 1e-6) {
                return Some(quad);
            }
            let prev = Quadric({
                let mut c = q.0;
                c[10] = k;
                c
            });
            let mut a = [[0f64; 6]; 6];
            let mut b = [0f64; 6];
            let mut n = 0usize;
            for (&(u, v, z), &kk) in s.iter().zip(keep) {
                if !kk {
                    continue;
                }
                let g = prev.conic(prev.0[3] * u * u + prev.0[4] * u * v + prev.0[5] * v * v).0;
                let f = [1.0, u, v, g * u * u, g * u * v, g * v * v];
                for i in 0..6 {
                    b[i] += f[i] * z;
                    for j in 0..6 {
                        a[i][j] += f[i] * f[j];
                    }
                }
                n += 1;
            }
            if n < 12 {
                return Some(quad);
            }
            for (i, row) in a.iter_mut().enumerate() {
                row[i] += 1e-9 * (1.0 + row[i]);
            }
            let x = solve(a, b)?;
            let mut c = [0f64; 11];
            c[..6].copy_from_slice(&x);
            c[10] = k;
            q = Quadric(c);
        }
        let k = q.kappa_max();
        q.0[10] = k;
        // Over a ring the two fit about equally well (the quadric takes up the conic's r⁴ term
        // with its constant and its curvature): they differ in what they extrapolate into the cut.
        // The conic is kept unless it clearly fits worse.
        Some(if rms_of(s, keep, &q) <= 1.05 * rq + 1e-4 { q } else { quad })
    }

    /// One least-squares fit over the samples marked in `keep` (N = 6: quadric, 10: cubic).
    pub(crate) fn fit_once_n<const N: usize>(s: &[(f64, f64, f64)], keep: &[bool]) -> Option<Quadric> {
        Self::fit_once::<N>(s, keep)
    }

    fn fit_once<const N: usize>(s: &[(f64, f64, f64)], keep: &[bool]) -> Option<Quadric> {
        let mut a = [[0f64; N]; N];
        let mut b = [0f64; N];
        let mut n = 0usize;
        for (&(u, v, z), &k) in s.iter().zip(keep) {
            if !k {
                continue;
            }
            n += 1;
            let all = [1.0, u, v, u * u, u * v, v * v, u * u * u, u * u * v, u * v * v, v * v * v];
            let f = &all[..N];
            for i in 0..N {
                b[i] += f[i] * z;
                for j in 0..N {
                    a[i][j] += f[i] * f[j];
                }
            }
        }
        if n < 2 * N {
            return None;
        }
        // Tiny ridge for stability on narrow annuli.
        for (i, row) in a.iter_mut().enumerate() {
            row[i] += 1e-9 * (1.0 + row[i]);
        }
        let x = solve(a, b)?;
        let mut c = [0f64; 11];
        c[..N].copy_from_slice(&x);
        Some(Quadric(c))
    }

    /// Three-pass fit: each refit drops residuals beyond max(2 σ, 0.03 mm) of the previous fit.
    /// Returns the fit and the RMS over the kept samples.
    pub fn fit_robust(s: &[(f64, f64, f64)]) -> Option<(Quadric, f64, usize)> {
        Self::fit_robust_n::<6>(s)
    }

    /// [`fit_robust`](Self::fit_robust) with the cubic terms: follows a sheet whose curvature
    /// changes across the site (a gentle crease, the start of a rim's roll-off) that a quadric
    /// averages into a dent or a bump at the centre.
    pub fn fit_robust_cubic(s: &[(f64, f64, f64)]) -> Option<(Quadric, f64, usize)> {
        Self::fit_robust_n::<10>(s)
    }

    fn fit_robust_n<const N: usize>(s: &[(f64, f64, f64)]) -> Option<(Quadric, f64, usize)> {
        let mut keep = vec![true; s.len()];
        let mut q = Self::fit_once::<N>(s, &keep)?;
        let mut rms = rms_of(s, &keep, &q);
        for _ in 0..2 {
            let lim = (2.0 * rms).max(0.03);
            for (k, &(u, v, z)) in keep.iter_mut().zip(s) {
                *k = (z - q.eval(u, v)).abs() <= lim;
            }
            q = Self::fit_once::<N>(s, &keep)?;
            rms = rms_of(s, &keep, &q);
        }
        Some((q, rms, keep.iter().filter(|k| **k).count()))
    }

    /// The annulus fit of a marker site: the quadric, or the cubic when the quadric leaves an
    /// RMS above `rough_mm` and the cubic does clearly better (an annulus all round the site
    /// constrains the cubic terms; a partial one does not, then the quadric is kept).
    pub fn fit_annulus(s: &[(f64, f64, f64)], rough_mm: f64, covered_all_round: bool) -> Option<(Quadric, f64, usize)> {
        let quad = Self::fit_robust(s)?;
        if quad.1 <= rough_mm || !covered_all_round {
            return Some(quad);
        }
        match Self::fit_robust_cubic(s) {
            Some(cub) if cub.1 < 0.8 * quad.1 && cub.2 as f64 >= 0.9 * quad.2 as f64 => Some(cub),
            _ => Some(quad),
        }
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

/// Gaussian elimination with partial pivoting for an N×N system.
pub(crate) fn solve<const N: usize>(mut a: [[f64; N]; N], mut b: [f64; N]) -> Option<[f64; N]> {
    for c in 0..N {
        let p = (c..N).max_by(|&i, &j| a[i][c].abs().total_cmp(&a[j][c].abs()))?;
        if a[p][c].abs() < 1e-12 {
            return None;
        }
        a.swap(c, p);
        b.swap(c, p);
        for r in c + 1..N {
            let f = a[r][c] / a[c][c];
            for k in c..N {
                a[r][k] -= f * a[c][k];
            }
            b[r] -= f * b[c];
        }
    }
    let mut x = [0f64; N];
    for r in (0..N).rev() {
        let mut s = b[r];
        for k in r + 1..N {
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
    /// A direction in the frame.
    #[inline]
    fn local_dir(&self, n: Vec3) -> DVec3 {
        DVec3::new(n.dot(self.eu) as f64, n.dot(self.ev) as f64, n.dot(self.n) as f64)
    }
    /// A ring sample of point `p` with normal `n`.
    #[inline]
    fn sample(&self, p: Vec3, n: Vec3) -> RingSample {
        let (u, v, h) = self.local(p);
        RingSample { u: u as f64, v: v as f64, h: h as f64, n: self.local_dir(n) }
    }
    /// A direction given in the frame, in world coordinates.
    #[inline]
    fn world_dir(&self, nl: DVec3) -> Vec3 {
        (self.eu * nl.x as f32 + self.ev * nl.y as f32 + self.n * nl.z as f32).normalize()
    }
}

/// Fill-surface settings of a site with cut radius `rc` (see [`super::sitefit`]).
fn fit_params(rc: f32, all_round: bool, rough_mm: f32, band_mm: f32, mls_mm: f64, slopes: bool) -> SiteFitParams {
    SiteFitParams { rc: rc as f64, band: band_mm.max(1.0) as f64, mls_mm, all_round, rough_mm: rough_mm as f64, slopes }
}

/// A mesh vertex inside the cut is faired in full when its normal is within about 18° of the
/// target's (cosine), less up to 25° ([`fair_sites`]).
const FAIR_FULL_FACING: f32 = 0.95;
/// Moving-least-squares width of the band remainder for scan points and for mesh vertices (mm).
const MLS_POINTS_MM: f64 = 1.2;
const MLS_MESH_MM: f64 = 0.7;
/// A patch of a site not covered all round stays within this of the convex hull of the sheet
/// round it and in its cut (mm).
const PARTIAL_HULL_MARGIN_MM: f32 = 0.4;

/// An infill patch: the fill surface of a fillable site.
struct Patch {
    site: usize,
    f: SiteFrame,
    q: SiteSurface,
    scan: u8,
    /// Where the patch of a site not covered all round may go ([`PartialCover`]).
    only: Option<PartialCover>,
}

/// Cut points of a site not covered all round lying more than this below its patch (mm) are real
/// surface falling away from the sheet (a rim's rolled edge, a side face) and are not cut.
const PARTIAL_CUT_BELOW_MM: f64 = 0.3;
/// ... and more than this above it (a rib foot or a rim rising; a sticker is ~0.4 mm proud).
const PARTIAL_CUT_ABOVE_MM: f64 = 1.0;
/// Fewest cut points below the patch, in directions without annulus surface, that make a side face
/// the patch keeps behind ([`PartialCover::behind_rim`]).
const PARTIAL_BELOW_MIN_POINTS: usize = 6;
/// Cut points on the patch count as the sheet for the hull of a partial site only this close to the
/// cut's edge (mm inside it): further in they may be the sticker (its edge ramp, or a thin sticker
/// within 0.15 mm of the sheet), which over a rim is not surface.
const PARTIAL_SHEET_EDGE_MM: f32 = 0.4;

/// For a site not covered all round: the lattice cells (index pairs at `fill_spacing_mm`) the cut
/// took points from, grown by one cell; the mask of the [`FINE_SECTORS`] directions in which the
/// annulus has surface; and the cells (grown by one) of the points inside the cut that are left
/// because they lie off the patch, beyond the sheet. The patch is emitted only in the cut's cells
/// or in directions covered together with both neighbours, so it never reaches past the scanned
/// surface (an open rim, the end of the scan), and never over the surface left in the cut, so it
/// does not stand above a rim's rolled edge.
struct PartialCover {
    cells: std::collections::HashSet<(i32, i32)>,
    mask: u32,
    beyond: std::collections::HashSet<(i32, i32)>,
    /// Convex hull of the sheet round the site and in its cut: the cut's cells include a sticker
    /// hanging over the rim, which is not surface (the patch there stood out as a bulge).
    hull: Vec<(f32, f32)>,
    /// The cells (grown by one) of the cut points left because they lie well below the patch (a
    /// rim's rolled edge or side face going down) ...
    below: std::collections::HashSet<(i32, i32)>,
    /// ... and the centre of the sheet round the site: a lattice point seen from there behind such
    /// a cell lies past the rim (under a sticker hanging over it) and gets no patch.
    origin: (f32, f32),
    spacing: f32,
}

impl PartialCover {
    fn allows(&self, i: i32, j: i32, u: f32, v: f32) -> bool {
        !self.beyond.contains(&(i, j))
            && (self.cells.contains(&(i, j)) || direction_covered(self.mask, u, v))
            && outside_hull(&self.hull, u, v) <= PARTIAL_HULL_MARGIN_MM
            && !self.behind_rim(u, v)
    }
    /// Whether the straight line from the sheet's centre to (u, v) crosses a cell of surface going
    /// down beyond the rim before it reaches the cell of (u, v).
    fn behind_rim(&self, u: f32, v: f32) -> bool {
        if self.below.is_empty() {
            return false;
        }
        let (du, dv) = (u - self.origin.0, v - self.origin.1);
        let steps = ((du * du + dv * dv).sqrt() / (0.5 * self.spacing)).ceil() as i32;
        let end = ((u / self.spacing).round() as i32, (v / self.spacing).round() as i32);
        (0..steps).any(|k| {
            let t = k as f32 / steps.max(1) as f32;
            let c = (((self.origin.0 + t * du) / self.spacing).round() as i32, ((self.origin.1 + t * dv) / self.spacing).round() as i32);
            c != end && self.below.contains(&c)
        })
    }
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
        self.f.world_dir(self.q.normal(u as f64, v as f64))
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

    // Per site: the outcome, the fill surface (fillable sites), whether the cut is applied, the
    // covered directions of the annulus and the sheet samples of the annulus.
    let mut fits: Vec<SiteFitOut> = markers
        .par_iter()
        .zip(frames.par_iter())
        .zip(cuts.par_iter())
        .map(|((m, f), cut)| site_fit(m, f, cut.len(), pts, &pos, &grid, &in_cut, prm))
        .collect();

    // A site not covered all round lies at a rim or an edge, and its cut can reach over the surface
    // beyond the sheet: a rim's rolled edge or a side face falling away, a rib foot rising. Cut
    // points well off the site's patch are that surface, not the sticker (a disc about 0.4 mm
    // proud, or nothing): they stay, and the patch keeps clear of them.
    let mut beyond: Vec<Vec<u32>> = vec![Vec::new(); markers.len()];
    let mut below: Vec<Vec<u32>> = vec![Vec::new(); markers.len()];
    let cuts: Vec<Vec<u32>> = cuts
        .into_iter()
        .enumerate()
        .map(|(k, cut)| {
            let SiteFitOut { out, surf, mask, .. } = &mut fits[k];
            let Some(q) = surf.as_ref() else { return cut };
            let mask = *mask;
            if out.sectors_covered >= 8 {
                return cut;
            }
            let (on, off): (Vec<u32>, Vec<u32>) = cut.into_iter().partition(|&i| {
                let (u, v, h) = frames[k].local(pts[i as usize].p);
                let r = h as f64 - q.eval(u as f64, v as f64);
                (-PARTIAL_CUT_BELOW_MM..=PARTIAL_CUT_ABOVE_MM).contains(&r)
            });
            out.cut_points = on.len();
            // Surface going down beyond the rim lies where the annulus has no sheet: a point below
            // the patch in a direction the sheet covers is a stray (a laser line, the dark border's
            // flare), and one point is no side face either.
            below[k] = off
                .iter()
                .copied()
                .filter(|&i| {
                    let (u, v, h) = frames[k].local(pts[i as usize].p);
                    (h as f64) < q.eval(u as f64, v as f64) - PARTIAL_CUT_BELOW_MM && !direction_covered(mask, u, v)
                })
                .collect();
            if below[k].len() < PARTIAL_BELOW_MIN_POINTS {
                below[k].clear();
            }
            beyond[k] = off;
            on
        })
        .collect();

    let patches: Vec<Patch> = fits
        .iter_mut()
        .enumerate()
        .filter_map(|(k, fo)| {
            let q = fo.surf.take()?;
            let (out, mask) = (&fo.out, &fo.mask);
            let only = (out.sectors_covered < 8).then(|| {
                let s = prm.fill_spacing_mm;
                let cells_of = |idx: &[u32]| {
                    let mut cells = std::collections::HashSet::new();
                    for &i in idx {
                        let (u, v, _) = frames[k].local(pts[i as usize].p);
                        let (a, b) = ((u / s).round() as i32, (v / s).round() as i32);
                        for da in -1..=1 {
                            for db in -1..=1 {
                                cells.insert((a + da, b + db));
                            }
                        }
                    }
                    cells
                };
                // The sheet: the annulus samples and the cut points on the patch at the edge of the
                // cut, beyond the sticker.
                let mut sheet = std::mem::take(&mut fo.ring_uv);
                let origin = if sheet.is_empty() {
                    (0.0, 0.0)
                } else {
                    let (a, b) = sheet.iter().fold((0.0, 0.0), |acc, p| (acc.0 + p.0, acc.1 + p.1));
                    (a / sheet.len() as f32, b / sheet.len() as f32)
                };
                let edge = frames[k].rc - PARTIAL_SHEET_EDGE_MM;
                for &i in &cuts[k] {
                    let (u, v, h) = frames[k].local(pts[i as usize].p);
                    if (h as f64 - q.eval(u as f64, v as f64)).abs() < 0.15 && u * u + v * v >= edge * edge {
                        sheet.push((u, v));
                    }
                }
                PartialCover {
                    cells: cells_of(&cuts[k]),
                    mask: *mask,
                    beyond: cells_of(&beyond[k]),
                    hull: convex_hull(sheet),
                    below: cells_of(&below[k]),
                    origin,
                    spacing: s,
                }
            });
            Some(Patch { site: k, f: frames[k], q, scan: markers[k].scan.unwrap_or(0), only })
        })
        .collect();
    let fills: Vec<Vec<WorkPoint>> = (0..patches.len()).into_par_iter().map(|a| lattice(a, &patches, prm)).collect();

    let mut remove = vec![false; pts.len()];
    let mut outcomes = Vec::with_capacity(markers.len());
    for (SiteFitOut { out, apply, .. }, cut) in fits.into_iter().zip(&cuts) {
        if apply {
            for &i in cut {
                remove[i as usize] = true;
            }
        }
        outcomes.push(out);
    }
    for (pa, fill) in patches.iter().zip(&fills) {
        outcomes[pa.site].fill_points = fill.len();
        outcomes[pa.site].surface = Some(pa.q.clone());
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

/// Directions (sectors of the annulus) a partly covered site's patch is limited to.
const FINE_SECTORS: usize = 24;

fn fine_sector(u: f32, v: f32) -> usize {
    (((v.atan2(u) + std::f32::consts::PI) / (std::f32::consts::TAU / FINE_SECTORS as f32)) as usize).min(FINE_SECTORS - 1)
}

/// Whether the direction of (u, v) and both neighbouring directions have annulus surface in
/// `mask` (a direction at the end of the coverage is partly beyond it).
fn direction_covered(mask: u32, u: f32, v: f32) -> bool {
    let s = fine_sector(u, v);
    [s + FINE_SECTORS - 1, s, s + 1].iter().all(|&k| mask & 1 << (k % FINE_SECTORS) != 0)
}

/// 1 in the directions of (u, v) that `mask` covers, falling linearly to 0 two directions away
/// from the nearest covered one.
fn direction_weight(mask: u32, u: f32, v: f32) -> f32 {
    let s = (v.atan2(u) + std::f32::consts::PI) / (std::f32::consts::TAU / FINE_SECTORS as f32);
    let n = FINE_SECTORS as f32;
    let dist = (0..FINE_SECTORS)
        .filter(|&k| mask & 1 << k != 0)
        .map(|k| {
            let d = (s - (k as f32 + 0.5)).abs() % n;
            d.min(n - d)
        })
        .fold(f32::INFINITY, f32::min);
    ((2.5 - dist) / 2.0).clamp(0.0, 1.0)
}

/// What [`site_fit`] found at one site.
struct SiteFitOut {
    out: MarkerOutcome,
    /// The fill surface, when the site is filled.
    surf: Option<SiteSurface>,
    /// Whether the cut is applied.
    apply: bool,
    /// The [`FINE_SECTORS`] directions in which the annulus has surface (two points or more).
    mask: u32,
    /// (u, v) of the annulus samples the fit kept (the sheet round the site).
    ring_uv: Vec<(f32, f32)>,
}

/// Annulus fit of one site ([`fit_site`] on the points of the sheet in the annulus).
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
) -> SiteFitOut {
    let mut ann: Vec<RingSample> = Vec::new();
    let mut sector_count = [0u32; 8];
    let mut fine = [0u32; FINE_SECTORS];
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
            fine[fine_sector(u, v)] += 1;
            if !in_cut[i as usize] {
                ann.push(f.sample(p.p, p.n));
            }
        }
    });
    let sectors = sector_count.iter().filter(|&&c| c >= 6).count() as u32;
    let mask = fine.iter().enumerate().filter(|e| *e.1 >= 2).fold(0u32, |m, (s, _)| m | 1 << s);
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
        surface: None,
    };
    let done = |out: MarkerOutcome, surf: Option<SiteSurface>, apply: bool| SiteFitOut { out, surf, apply, mask, ring_uv: Vec::new() };
    if !prm.quadric_fill {
        out.fill = "implicit (M1)".into();
        return done(out, None, true);
    }
    // A site that cannot be patched is either left to the solve (cut, M1 behaviour) or kept.
    let unfillable = |mut out: MarkerOutcome, why: String| {
        if prm.keep_unfillable {
            out.fill = format!("kept (not cut): {why}");
            out.cut_points = 0;
            done(out, None, false)
        } else {
            out.fill = format!("implicit: {why}");
            done(out, None, true)
        }
    };
    if sectors < 6 && sectors < prm.partial_min_sectors {
        return unfillable(out, format!("annulus covers {sectors}/8 sectors"));
    }
    let params = fit_params(f.rc, sectors == 8, 0.5 * prm.fit_rms_max_mm, prm.fair_blend_mm, MLS_POINTS_MM, prm.continue_band_slopes);
    let Some(fit) = fit_site(&ann, &params) else {
        return unfillable(out, "quadric fit failed".into());
    };
    let rms = fit.rms_model;
    out.annulus_rms_mm = rms as f32;
    if rms > prm.fill_rms_limit_mm as f64 {
        return unfillable(out, format!("annulus RMS {:.3} mm", rms));
    }
    out.fill = match (rms > prm.fit_rms_max_mm as f64, sectors < 8) {
        (false, false) => "quadric".into(),
        (true, false) => format!("quadric, rough annulus ({:.3} mm)", rms),
        (false, true) => format!("quadric, partial ({sectors}/8 sectors)"),
        (true, true) => format!("quadric, partial ({sectors}/8 sectors), rough annulus ({:.3} mm)", rms),
    };
    let ring_uv = ann.iter().zip(&fit.keep).filter(|e| *e.1).map(|(x, _)| (x.u as f32, x.v as f32)).collect();
    SiteFitOut { out, surf: Some(fit.surf), apply: true, mask, ring_uv }
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
            if r > edge || pa.only.as_ref().is_some_and(|c| !c.allows(i, j, u, v)) {
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

/// Widens each site's cut over an empty ring around it. Revo Scan leaves an empty ring 6–9 mm
/// around a sticker in some scans (the dark border of the marker returns no data); a cut that ends
/// inside that ring leaves the solve or the mesher to bridge it, which shows as a ring of dimples
/// round the patch, and the annulus fit would see only its outer part. Per site the occupancy of
/// 0.5 mm rings from the cut outwards is measured (the share of ~0.7 mm cells along the ring that
/// hold a same-facing point within 1.5 mm of the sheet; occupancy, not density, so where scans
/// overlap counts the same as where one covers) and compared with the occupancy 6–9 mm beyond
/// the cut; while a ring has less than half of that, the cut grows (by at most `widen_max_mm`).
pub(crate) fn widen_over_sparse_rings(pts: &[WorkPoint], sites: &mut [Marker], prm: &MarkerConfig) {
    if prm.widen_max_mm <= 0.0 || sites.is_empty() {
        return;
    }
    let pos: Vec<Vec3> = pts.iter().map(|p| p.p).collect();
    let grid = PointGrid::new(&pos, 2.0);
    const BIN: f32 = 0.5;
    const ARC: f32 = 0.7;
    sites.par_iter_mut().for_each(|m| {
        let f = SiteFrame::new(m, prm);
        let (r0, r_ref0, r_ref1) = (f.rc, f.rc + 6.0, f.rc + 9.0);
        let nb = ((r_ref1 - r0) / BIN).ceil() as usize;
        let cells: Vec<usize> = (0..nb).map(|k| ((std::f32::consts::TAU * (r0 + (k as f32 + 0.5) * BIN) / ARC).ceil() as usize).max(8)).collect();
        let mut occ: Vec<Vec<bool>> = cells.iter().map(|&c| vec![false; c]).collect();
        grid.for_each_within(&pos, f.c, (r_ref1 * r_ref1 + 2.25).sqrt(), |i, _| {
            let p = &pts[i as usize];
            if p.kind != PointKind::Measured || p.n.dot(f.n) < 0.5 {
                return;
            }
            let (u, v, h) = f.local(p.p);
            let r = (u * u + v * v).sqrt();
            if h.abs() < 1.5 && r >= r0 && r < r_ref1 {
                let k = (((r - r0) / BIN) as usize).min(nb - 1);
                let a = ((v.atan2(u) + std::f32::consts::PI) / std::f32::consts::TAU * cells[k] as f32) as usize;
                occ[k][a.min(cells[k] - 1)] = true;
            }
        });
        let share = |k: usize| occ[k].iter().filter(|&&o| o).count() as f32 / cells[k] as f32;
        let refs: Vec<usize> = (0..nb).filter(|&k| r0 + k as f32 * BIN >= r_ref0).collect();
        let reference = refs.iter().map(|&k| share(k)).sum::<f32>() / refs.len().max(1) as f32;
        if reference < 0.3 {
            return;
        }
        let mut k = 0usize;
        while (k as f32) * BIN < prm.widen_max_mm && share(k) < 0.5 * reference {
            k += 1;
        }
        if k > 0 {
            // cut_radius = radius + 1 mm: the cut ends half a millimetre beyond the empty rings.
            m.radius_mm = m.radius_mm.max(r0 + k as f32 * BIN + 0.5 - 1.0);
        }
    });
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
/// `cut .. cut + fit_width` are collected; vertices inside the cut of another site on the same
/// sheet count for the coverage but are not used. The target surface inside the cut is
/// - where `fills` holds the site's fill surface from the scan points ([`cut_and_fill`]): that
///   surface met by the mesh round the cut ([`SiteSurface::met_by`]: the mesh's remainder in the
///   `fair_blend_mm` band carried in harmonically), so the patch keeps the slope and curvature of
///   the scan points round the cut and meets the mesh there without a step;
/// - elsewhere (a site that was kept or left to the solve, or a mesh on its own): the fill surface
///   of the mesh annulus ([`fit_site`]), met by the mesh the same way.
///
/// Either way only where the mesh annulus is smooth sheet: RMS ≤ `fair_rms_max_mm` against a
/// robust quadric with 60 % of its vertices kept, which a rim or an edge in the annulus fails.
///
/// With the annulus covered in at least `partial_min_sectors` of 8 sectors, every vertex inside the
/// cut that lies on the target (within 0.8 mm and facing like it within 25°, in full within 0.6 mm
/// and 18°) is moved along the site normal onto it; the band itself is not moved. A vertex moved by several sites gets the
/// mean of their displacements, so overlapping sites neither undo each other nor leave a step.
/// At a site not covered all round, moves of more than 0.25 mm fade out towards the directions
/// without mesh surface round the cut (an open rim, whose rolled edge is not the sheet the fit
/// extrapolates).
/// Edges, ribs and the opposite wall are not on the fitted sheet and stay where they are;
/// topology does not change. Marks the moved vertices inside a cut in `inside_cut`.
pub(crate) fn fair_sites(
    pos: &mut [Vec3],
    vnormal: &[Vec3],
    sites: &[Marker],
    fills: Option<&[Option<SiteSurface>]>,
    prm: &MarkerConfig,
    inside_cut: &mut [bool],
) -> Vec<FairOutcome> {
    let grid = PointGrid::new(pos, 2.0);
    let snapshot: &[Vec3] = pos;
    let frames: Vec<SiteFrame> = sites.iter().map(|m| SiteFrame::new(m, prm)).collect();
    let per_site: Vec<(FairOutcome, Vec<FairMove>)> = (0..sites.len())
        .into_par_iter()
        .map(|k| {
            let fill = fills.map(|f| f.get(k).and_then(|s| s.as_ref()));
            fair_one(k, &frames, fill, snapshot, vnormal, &grid, prm)
        })
        .collect();
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
    fill: Option<Option<&SiteSurface>>,
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
    let mut ann: Vec<RingSample> = Vec::new();
    let mut cand: Vec<u32> = Vec::new();
    let mut sector_count = [0u32; 8];
    let mut fine = [0u32; FINE_SECTORS];
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
        if r < rc {
            cand.push(i);
        }
        if r >= rc && r < r_out {
            let s = (((v.atan2(u) + std::f32::consts::PI) / (std::f32::consts::TAU / 8.0)) as usize).min(7);
            sector_count[s] += 1;
            fine[fine_sector(u, v)] += 1;
            if !in_other_cut(p) {
                ann.push(f.sample(p, vnormal[i as usize]));
            }
        }
    });
    let sectors = sector_count.iter().filter(|&&c| c >= 6).count();
    let mask = fine.iter().enumerate().filter(|e| *e.1 >= 2).fold(0u32, |m, (s, _)| m | 1 << s);
    if sectors < 7 && (sectors as u32) < prm.partial_min_sectors.min(8) {
        return fail(format!("annulus covers {sectors}/8 sectors"), nan);
    }
    // The surface the mesh inside the cut goes onto. With the site's fill surface from the scan
    // points (Make Solid, Reconstruct): that surface, met by the mesh round the cut (the band's
    // remainder carried in harmonically, [`SiteSurface::met_by`]), so the patch keeps the slope and
    // curvature the scan points show round the cut (the solve's smoothing round the site, carried
    // in C1, would tilt the patch) and meets the mesh round it without a step. Without one (a site
    // that was kept or left to the solve, or a mesh on its own), the fill surface of the mesh
    // annulus ([`fit_site`]), met by the mesh the same way. The surface beyond the cut stays where
    // it is.
    let band = prm.fair_blend_mm.max(1.0) as f64;
    // The mesh round the cut must be smooth sheet (RMS against a robust ring quadric within
    // `fair_rms_max_mm`, most vertices on it): at a rim or an edge the ring holds the rolled edge,
    // and moving the vertices inside the cut onto any surface there folds it (S1-M4, M36).
    let ring: Vec<(f64, f64, f64)> = ann.iter().map(|x| (x.u, x.v, x.h)).collect();
    let Some((_, ring_rms, ring_kept)) = Quadric::fit_annulus(&ring, 0.5 * prm.fair_rms_max_mm as f64, sectors == 8) else {
        return fail("quadric fit failed".into(), nan);
    };
    let ring_rms_mm = ring_rms as f32;
    if ring_rms > prm.fair_rms_max_mm as f64 || (ring_kept as f64) < 0.6 * ann.len() as f64 {
        return fail(format!("annulus RMS {:.3} mm ({ring_kept}/{} kept)", ring_rms, ann.len()), ring_rms_mm);
    }
    let params = fit_params(rc, sectors == 8, 0.5 * prm.fair_rms_max_mm, prm.fair_blend_mm, MLS_MESH_MM, prm.continue_band_slopes);
    let owned;
    let surf = match fill.flatten() {
        Some(data) => match data.met_by(&ann, band, MLS_MESH_MM, 0.5) {
            Some(s) => {
                owned = s;
                &owned
            }
            None => return fail("the mesh round the cut is off the fill surface".into(), ring_rms_mm),
        },
        // A site with no fill surface (kept, or left to the solve) or a mesh on its own: the fill
        // surface of the mesh annulus, met by the mesh round the cut.
        None => {
            let Some(fit) = fit_site(&ann, &params) else {
                return fail("quadric fit failed".into(), ring_rms_mm);
            };
            owned = fit.surf.met_by(&ann, band, MLS_MESH_MM, 0.5).unwrap_or(fit.surf);
            &owned
        }
    };
    // Directions in which the mesh round the cut is there but not on the target (fewer than a
    // quarter of its band vertices within 0.3 mm and facing like it): a rim's rolled edge or a side
    // face beyond the cut, where the sheet does not go on. Inside the cut the mesh there is the
    // start of that edge, not the sheet, and is left where it is (up to 1.5 mm in from the cut,
    // fading out by 3 mm).
    let rolled = {
        let cos_max = RING_NORMAL_MAX_DEG.to_radians().cos();
        let (mut on, mut all) = ([0u32; FINE_SECTORS], [0u32; FINE_SECTORS]);
        for x in &ann {
            let r = x.u.hypot(x.v);
            if r < rc as f64 + band {
                let k = fine_sector(x.u as f32, x.v as f32);
                all[k] += 1;
                on[k] += ((x.h - surf.eval(x.u, x.v)).abs() <= 0.3 && x.n.dot(surf.normal(x.u, x.v)) >= cos_max) as u32;
            }
        }
        (0..FINE_SECTORS).filter(|&k| all[k] >= 4 && 4 * on[k] < all[k]).fold(0u32, |m, k| m | 1 << k)
    };
    // Vertices on the target inside the cut (within 0.8 mm, facing like it within 25°), with a
    // weight that fades to 0 towards both limits: a hard limit moves one vertex of a rolled edge
    // or a rib's flank fully and leaves its neighbour, which folds the edge.
    let mut inner: Vec<(u32, f64, f64, f64, f32)> = Vec::new();
    for &i in &cand {
        let (u, v, h) = f.local(pos[i as usize]);
        let (u, v, h) = (u as f64, v as f64, h as f64);
        let off = (surf.eval(u, v) - h).abs() as f32;
        let facing = vnormal[i as usize].dot(f.world_dir(surf.normal(u, v)));
        if off > 0.8 || facing < 0.9 {
            continue;
        }
        let w = ((0.8 - off) / 0.2).clamp(0.0, 1.0) * ((facing - 0.9) / (FAIR_FULL_FACING - 0.9)).clamp(0.0, 1.0);
        inner.push((i, u, v, h, w));
    }
    if inner.len() < 20 {
        return fail(format!("only {} mesh vertices on the sheet inside the cut", inner.len()), ring_rms_mm);
    }
    let (mut sum, mut max_before) = (0f64, 0f64);
    let mut moves = Vec::with_capacity(inner.len());
    for &(i, u, v, h, w) in &inner {
        let dz = surf.eval(u, v) - h;
        sum += -dz;
        max_before = max_before.max(dz.abs());
        // A site not covered all round (a rim, an edge) is faired fully only towards the
        // directions in which the mesh has surface round it: towards an open rim the fit is an
        // extrapolation and the mesh there is the rim's own rolled edge, which falls away from it.
        // Towards a mere gap in the annulus the sheet goes on, and the moves stay small: up to
        // 0.25 mm they are made in full, fading out by 0.5 mm. In any direction a vertex more than
        // 0.3 mm below the target is the start of a rim's rolled edge inside the cut (the infill
        // leaves such points, [`PARTIAL_CUT_BELOW_MM`]), and its move fades out by 0.5 mm. All
        // weights are continuous (the direction weight fades over two directions), so the patch
        // shows no seam.
        // (Only near the cut's edge: further in, towards the centre, every direction meets.)
        let away = if rolled == 0 { 1.0 } else { 1.0 - direction_weight(rolled, u as f32, v as f32) * ((u.hypot(v) as f32 - (rc - 3.0)) / 1.5).clamp(0.0, 1.0) };
        let weight = w * away * if sectors < 8 {
            let below = ((0.5 - dz as f32) / (0.5 - PARTIAL_CUT_BELOW_MM as f32)).clamp(0.0, 1.0);
            (direction_weight(mask, u as f32, v as f32) * below).max(((0.5 - dz.abs() as f32) / 0.25).clamp(0.0, 1.0))
        } else {
            1.0
        };
        if weight > 0.0 {
            moves.push(FairMove { vertex: i, delta: n * dz as f32, weight, inside: true });
        }
    }
    let out = FairOutcome {
        status: "faired".into(),
        ring_rms_mm,
        mean_before_mm: (sum / inner.len() as f64) as f32,
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
        // A sticker sits on a plain sheet: beyond the ring the sheet goes on all round. A round
        // opening in a narrow plate (a screw hole in a hinge bracket) or beside a rib has the
        // plate's edge or the rib's foot there instead, and must stay open.
        if good >= 11 && plain_surroundings(&occ, side, half, cu, cv, radius, w as f64) {
            let m = mk(cu, cv, &q, MarkerSource::DetectedHole, radius as f32, radius as f32);
            // A screw hole's rim goes on down as the wall of its bore; a cut-out sticker's does not.
            if !bore_wall(pts, pos, grid, m.center, m.normal, radius as f32) {
                return Some(m);
            }
        }
    }
    None
}

/// Whether a round hole in a sheet (centre `c` on the sheet, `axis` its outward normal) is the
/// mouth of a bore: measured points of any scan 0.3–6 mm below the sheet, between 0.6 × `radius`
/// and `radius` + 1 mm from the axis, facing the axis (normal across the axis, pointing in), in
/// at least 3 of 8 sectors. A screw hole through a bracket plate or a boss shows its bore wall; the
/// gap a sticker was cut out of has nothing below its rim but the opposite face of the wall, which
/// faces along the axis.
fn bore_wall(pts: &[WorkPoint], pos: &[Vec3], grid: &PointGrid, c: Vec3, axis: Vec3, radius: f32) -> bool {
    let (eu, ev) = tangent_frame(axis);
    let mut count = [0u32; 8];
    let reach = ((radius + 1.0).powi(2) + 36.0).sqrt();
    grid.for_each_within(pos, c, reach, |i, _| {
        let p = &pts[i as usize];
        if p.kind != PointKind::Measured {
            return;
        }
        let d = p.p - c;
        let h = d.dot(axis);
        if !(-6.0..=-0.3).contains(&h) {
            return;
        }
        let radial = d - axis * h;
        let r = radial.length();
        if r < 0.6 * radius || r > radius + 1.0 || p.n.dot(axis).abs() > 0.6 || p.n.dot(radial / r) > -0.3 {
            return;
        }
        let s = (((radial.dot(ev).atan2(radial.dot(eu)) + std::f32::consts::PI) / (std::f32::consts::TAU / 8.0)) as usize).min(7);
        count[s] += 1;
    });
    count.iter().filter(|&&k| k >= 2).count() >= 3
}

/// Whether the sheet around a detected hole is plain: in the annulus `radius + 3 .. radius + 6` mm
/// around (cu, cv) (as far as the window reaches) at least 85 % of the 0.5 mm pixels are occupied,
/// and at least half of them in each of 12 sectors.
fn plain_surroundings(occ: &[bool], side: usize, half: i32, cu: f64, cv: f64, radius: f64, w: f64) -> bool {
    const PX: f64 = 0.5;
    let (r0, r1) = (radius + 3.0, radius + 6.0);
    let (mut hit, mut tot) = ([0u32; 12], [0u32; 12]);
    for b in 0..side as i32 {
        for a in 0..side as i32 {
            let (x, y) = ((a - half) as f64 * PX, (b - half) as f64 * PX);
            if x * x + y * y > w * w {
                continue;
            }
            let (u, v) = (x - cu, y - cv);
            let r = (u * u + v * v).sqrt();
            if r < r0 || r > r1 {
                continue;
            }
            let s = (((v.atan2(u) + std::f64::consts::PI) / (std::f64::consts::TAU / 12.0)) as usize).min(11);
            tot[s] += 1;
            if occ[b as usize * side + a as usize] {
                hit[s] += 1;
            }
        }
    }
    let (h, t): (u32, u32) = (hit.iter().sum(), tot.iter().sum());
    t > 0 && h as f64 >= 0.85 * t as f64 && (0..12).all(|s| tot[s] == 0 || hit[s] as f64 >= 0.5 * tot[s] as f64)
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
