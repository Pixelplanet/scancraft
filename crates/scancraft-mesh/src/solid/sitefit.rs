//! The surface a marker site is filled with: a ring fit that follows the curvature round the cut,
//! continued into the cut with value and slope matched at its edge (a thin-plate continuation).
//!
//! A sticker hides the surface under it, and a fill that does not continue the surface round it
//! shows under a grazing light: a flat disc on a dome reads as a dent (its sagitta r²/2R is
//! 0.15 mm for an 11 mm disc on R = 100 mm), and a slope step at the edge of the cut as a ring.
//!
//! [`fit_site`] builds the fill surface from samples of the sheet round the cut (scan points for
//! the infill, mesh vertices for the fairing) in the site's tangent frame (u, v, height h):
//!
//! 1. **Ring fit** q(u, v): a robust quadric height field (cubic where the quadric leaves a rough
//!    ring all round the site) on the ring `rc .. rc + fit_width`. The ring keeps only samples
//!    that face like the fit (normal within [`RING_NORMAL_MAX_DEG`]) and lie within 3 σ of it,
//!    σ = 1.4826 × the MAD of the residuals: a rim's roll-off, a side face or a rib foot in the ring
//!    fails one or the other, while a curvature band the fit cannot follow is not trimmed away as
//!    noise (the earlier trim at 2× the RMS of the kept points shrank onto the plain part of the
//!    ring and dropped exactly the samples the quadric could not follow).
//! 2. **Curvature bands** (only where the band `rc .. rc + 4.5 mm` next to the cut is plain sheet
//!    all round, >= 90 % of its samples on the fit, and the step below explains >= 0.01 mm RMS of
//!    it): what q misses (a curvature band, a gentle ridge) is fitted in that band as
//!    Σ_m (a + b t + c t²)·{cos, sin}(m θ), m ≤ 3, t = r − rc. Each order is continued into the cut
//!    by the biharmonic solution of the disc for that order, A r^m + B r^(m+2), matching the value
//!    a and the radial slope b at r = rc: C1 at the cut, and Δ²R = 0 inside like a thin plate. The
//!    slope beyond the harmonic one, (r/rc)^m, is weighted by t²/(t² + 2²), t its distance from 0
//!    in standard errors of the scan noise, so noise does not tilt the fill. Elsewhere the fill is
//!    q alone: on the Revo scans the band next to a cut holds the ragged rim of the gap round the
//!    sticker, and its slopes or its mean carried in made dents and bumps of 0.1-0.2 mm at rim
//!    sites, partial rings and plain sites alike. The quadric is biharmonic too, so a plain site
//!    stays on its ring fit (a leave-out test on the Revo scans found the quadric best on plain
//!    sites, a thin-plate continuation best near curvature bands).
//! 3. **Meeting the mesh** ([`SiteSurface::met_by`], the fairing): what the mesh round the cut
//!    leaves against the fill surface in the band (the solve's offset and smoothing, texture) is
//!    carried in, its low orders C1 where the fill is the ring fit alone, the rest per 0.35 mm
//!    pixel by local linear moving least squares (unbiased at the edge of the data, where a
//!    per-pixel mean or a one-sided smoothing would tilt it) continued harmonically: no step at the
//!    cut, bounded by the band's own values inside.

use super::markers::Quadric;
use glam::DVec3;

/// Ring samples whose normal is further than this off the ring fit's are not the sheet.
pub(crate) const RING_NORMAL_MAX_DEG: f64 = 20.0;
/// Pixel size of the remainder field (mm).
const PX: f64 = 0.35;
/// Highest angular order of the continued ring residual (all round / partial ring).
const MODES_ALL_ROUND: usize = 3;
const MODES_PARTIAL: usize = 1;
/// Width of the band next to the cut whose residual orders are fitted (mm).
const MODAL_BAND_MM: f64 = 4.5;
/// Share of the band's samples that must be on the fit (plain sheet) for its slopes to be continued.
const SLOPE_MIN_PLAIN: f64 = 0.9;
/// RMS of the band the low orders must explain beyond the ring fit for their slopes to be
/// continued (mm).
const SLOPE_MIN_EXPLAINED_MM: f64 = 0.01;

/// The radial slope of an order beyond its harmonic one is continued with the weight
/// t² / (t² + T0²), t its distance from 0 in standard errors: in full when it is clearly there,
/// hardly at all when it is within the noise.
const SLOPE_T0: f64 = 2.0;
/// Area over which the scan noise is taken as correlated for the slope test (mm²).
const NOISE_CORRELATION_MM2: f64 = 0.25;

/// A sample of the sheet round a site in the site frame: (u, v) in the tangent plane, height h
/// along the site normal, and its unit normal in the same frame.
#[derive(Clone, Copy, Debug)]
pub(crate) struct RingSample {
    pub u: f64,
    pub v: f64,
    pub h: f64,
    pub n: DVec3,
}

/// Settings of one [`fit_site`] call.
#[derive(Clone, Copy, Debug)]
pub(crate) struct SiteFitParams {
    /// Cut radius: samples inside it are never used.
    pub rc: f64,
    /// Width of the band beyond the cut whose remainder is carried into the cut.
    pub band: f64,
    /// Gaussian width of the moving-least-squares fit of the remainder (mm).
    pub mls_mm: f64,
    /// The ring is covered all round (cubic terms and the higher angular orders allowed).
    pub all_round: bool,
    /// Ring RMS above which the cubic is tried.
    pub rough_mm: f64,
    /// Continue the band's slopes (C1) where they pass the tests ([`fit_site`]).
    pub slopes: bool,
}

/// One angular order of the continuation: `trig(m θ) (a r^m + b r^(m+2))`.
#[derive(Clone, Copy, Debug)]
struct Mode {
    m: u32,
    sin: bool,
    a: f64,
    b: f64,
}

/// The fitted residual model in the band beyond the cut: per low angular order a value and a radial
/// slope at the cut (`(a + b t) trig(m θ)`, t = r − rc, read up to t = `width`).
#[derive(Clone)]
struct BandModel {
    m_max: usize,
    deg: usize,
    rc: f64,
    width: f64,
    c: Vec<f64>,
}

impl BandModel {
    fn eval(&self, u: f64, v: f64) -> f64 {
        let mut f = Vec::with_capacity(self.c.len());
        basis(self.m_max, self.deg, u, v, self.rc, self.width, &mut f);
        f.iter().zip(&self.c).map(|(a, b)| a * b).sum()
    }
}

/// A fitted band residual and its continuation into the cut.
#[derive(Clone)]
struct Component {
    modes: Vec<Mode>,
    band: BandModel,
}

impl Component {
    #[inline]
    fn at(&self, u: f64, v: f64, r: f64) -> f64 {
        if r >= self.band.rc {
            return self.band.eval(u, v);
        }
        let th = v.atan2(u);
        let mut z = 0.0;
        for md in &self.modes {
            let t = if md.sin { (md.m as f64 * th).sin() } else { (md.m as f64 * th).cos() };
            let rm = r.powi(md.m as i32);
            z += t * (md.a * rm + md.b * rm * r * r);
        }
        z
    }
}

/// Height field of a site's fill: ring fit plus the band residual models, continued into the cut.
/// Beyond the cut it is the fit of the ring itself (ring fit, band models and remainders), inside
/// the cut the C1 continuation of that.
#[derive(Clone)]
pub struct SiteSurface {
    pub(crate) q: Quadric,
    rc: f64,
    comps: Vec<Component>,
    rest: Vec<RemainderField>,
}

impl std::fmt::Debug for SiteSurface {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let orders: Vec<usize> = self.comps.iter().map(|c| c.modes.len()).collect();
        write!(f, "SiteSurface {{ rc: {:.2}, quadric: {:?}, orders: {:?}, remainders: {} }}", self.rc, self.q.0, orders, self.rest.len())
    }
}

impl SiteSurface {
    #[cfg(test)]
    pub(crate) fn from_quadric(q: Quadric) -> Self {
        SiteSurface { q, rc: 0.0, comps: Vec::new(), rest: Vec::new() }
    }
    /// The residual against the ring fit at (u, v): the continued orders inside the cut, the band
    /// models beyond it, plus the remainders.
    #[inline]
    pub fn residual(&self, u: f64, v: f64) -> f64 {
        let r = u.hypot(v);
        self.rest.iter().map(|f| f.at(u, v)).sum::<f64>() + self.comps.iter().map(|c| c.at(u, v, r)).sum::<f64>()
    }
    #[inline]
    pub fn eval(&self, u: f64, v: f64) -> f64 {
        self.q.eval(u, v) + self.residual(u, v)
    }
    #[inline]
    pub fn grad(&self, u: f64, v: f64) -> (f64, f64) {
        let (gu, gv) = self.q.grad(u, v);
        if self.comps.is_empty() && self.rest.is_empty() {
            return (gu, gv);
        }
        let e = 0.5 * PX;
        let du = (self.residual(u + e, v) - self.residual(u - e, v)) / (2.0 * e);
        let dv = (self.residual(u, v + e) - self.residual(u, v - e)) / (2.0 * e);
        (gu + du, gv + dv)
    }
    /// Unit normal in the site frame.
    #[inline]
    pub fn normal(&self, u: f64, v: f64) -> DVec3 {
        let (gu, gv) = self.grad(u, v);
        DVec3::new(-gu, -gv, 1.0).normalize()
    }

    /// This surface plus the remainder of other samples of the same site (mesh vertices round a
    /// filled cut) against it in the band `rc .. rc + band`: its low angular orders continued C1
    /// like the fill's own (value and slope), the rest harmonically. The surface then meets those
    /// samples at the cut without a step or a kink, and keeps its own shape (curvature from the
    /// scan points) inside. Only samples facing like the surface (within [`RING_NORMAL_MAX_DEG`])
    /// and within `max_off` of it count. `None` when fewer than half of the band samples do (a rib,
    /// an edge: the samples are not this sheet).
    pub(crate) fn met_by(&self, s: &[RingSample], band: f64, mls_mm: f64, max_off: f64) -> Option<SiteSurface> {
        let cos_max = RING_NORMAL_MAX_DEG.to_radians().cos();
        let rc = self.rc;
        let mut in_band = 0usize;
        let keep: Vec<bool> = s
            .iter()
            .map(|x| {
                let r = x.u.hypot(x.v);
                let b = r >= rc && r < rc + band;
                in_band += b as usize;
                b && x.n.dot(self.normal(x.u, x.v)) >= cos_max && (x.h - self.eval(x.u, x.v)).abs() <= max_off
            })
            .collect();
        let kept = keep.iter().filter(|k| **k).count();
        if kept < 20 || 2 * kept < in_band {
            return None;
        }
        let mut sectors = [0u32; 8];
        for (x, _) in s.iter().zip(&keep).filter(|e| *e.1) {
            sectors[(((x.v.atan2(x.u) + std::f64::consts::PI) / (std::f64::consts::TAU / 8.0)) as usize).min(7)] += 1;
        }
        let all_round = sectors.iter().all(|&c| c >= 6);
        let p = SiteFitParams { rc, band, mls_mm, all_round, rough_mm: 0.0, slopes: true };
        let resid = |x: &RingSample| x.h - self.eval(x.u, x.v);
        let mut out = self.clone();
        // Where the fill continues the band's slopes from the scan points, the mesh's remainder is
        // matched by value only (continued C1 it would bring back the solve's smoothing round the
        // site); where the fill is the ring fit alone, the remainder's low orders are continued C1,
        // so the patch meets the mesh round the cut without a kink.
        let comp = if self.comps.is_empty() && all_round {
            fit_modes(s, &keep, &resid, rc, band.min(3.0), MODES_ALL_ROUND, 1, true).map(|f| f.0)
        } else {
            None
        };
        let rest = RemainderField::build(s, &keep, |x: &RingSample| resid(x) - comp.as_ref().map_or(0.0, |c| c.band.eval(x.u, x.v)), &p);
        out.comps.extend(comp);
        out.rest.extend(rest);
        Some(out)
    }
}

/// Result of [`fit_site`]. (`rms`, `sigma` and `c1_modes` are diagnostics, read by the tests.)
#[derive(Clone, Debug)]
#[allow(dead_code)]
pub(crate) struct SiteFit {
    pub surf: SiteSurface,
    /// RMS of the kept ring samples against the ring fit q (mm).
    pub rms: f64,
    /// ... and against q plus its fitted low angular orders (what the fill follows; mm).
    pub rms_model: f64,
    /// Ring samples kept by the fit (sheet, within 3 σ).
    pub keep: Vec<bool>,
    pub kept: usize,
    /// Robust noise of the ring against q (1.4826 MAD, mm).
    pub sigma: f64,
    /// Angular orders whose slope is continued with a weight above one half.
    pub c1_modes: usize,
}

fn median(v: &mut [f64]) -> f64 {
    let m = v.len() / 2;
    v.select_nth_unstable_by(m, |a, b| a.total_cmp(b));
    v[m]
}

/// Ring fit and fill surface of a site (see the module docs). `s` holds the samples of the ring
/// `rc .. rc + fit_width` (the band is its inner part). `None` when the ring has too few sheet
/// samples for a fit.
pub(crate) fn fit_site(s: &[RingSample], p: &SiteFitParams) -> Option<SiteFit> {
    let pts: Vec<(f64, f64, f64)> = s.iter().map(|x| (x.u, x.v, x.h)).collect();
    let (mut q, _, _) = Quadric::fit_annulus(&pts, p.rough_mm, p.all_round)?;
    let cos_max = RING_NORMAL_MAX_DEG.to_radians().cos();
    let mut keep = vec![true; s.len()];
    let mut sigma = 0.0;
    let m_max = if p.all_round { MODES_ALL_ROUND } else { MODES_PARTIAL };
    let (width, deg) = (MODAL_BAND_MM, 2);
    let modes_of = |q: &Quadric, keep: &[bool], slopes: bool| fit_modes(s, keep, &|x: &RingSample| x.h - q.eval(x.u, x.v), p.rc, width, m_max, deg, slopes);
    let model_at = |f: &Option<(Component, usize)>, x: &RingSample| match f {
        Some((c, _)) if x.u.hypot(x.v) < p.rc + width => c.band.eval(x.u, x.v),
        _ => 0.0,
    };
    let mut fitted = modes_of(&q, &keep, false);
    for _ in 0..3 {
        // Trimmed against the whole model (ring fit and, in the band, its low orders), so a
        // curvature band the quadric cannot follow is not taken for noise.
        let res: Vec<f64> = s.iter().map(|x| x.h - q.eval(x.u, x.v) - model_at(&fitted, x)).collect();
        let facing: Vec<bool> = s
            .iter()
            .map(|x| {
                let (gu, gv) = q.grad(x.u, x.v);
                x.n.dot(DVec3::new(-gu, -gv, 1.0).normalize()) >= cos_max
            })
            .collect();
        let mut a: Vec<f64> = res.iter().zip(&facing).filter(|(_, &f)| f).map(|(r, _)| *r).collect();
        if a.len() < 24 {
            return None;
        }
        let med = median(&mut a);
        let mut dev: Vec<f64> = a.iter().map(|r| (r - med).abs()).collect();
        sigma = 1.4826 * median(&mut dev);
        let lim = (3.0 * sigma).max(0.02);
        for i in 0..s.len() {
            keep[i] = facing[i] && (res[i] - med).abs() <= lim;
        }
        let quad = Quadric::fit_once_n::<6>(&pts, &keep)?;
        q = quad;
        if p.all_round {
            let rq = rms_kept(&pts, &keep, &quad);
            if rq > p.rough_mm {
                if let Some(c) = Quadric::fit_once_n::<10>(&pts, &keep) {
                    if rms_kept(&pts, &keep, &c) < 0.8 * rq {
                        q = c;
                    }
                }
            }
        }
        fitted = modes_of(&q, &keep, false);
    }
    // The slopes of the band are continued only where the band is plain sheet all round the cut:
    // at a rim, an edge or a seam in the band (a partial ring, or more than a tenth of the band's
    // samples not on the fit) the band's slope belongs to that, and continued into the cut it
    // stands out as a dent or a bump; there the orders are carried in by value only.
    let (in_band, kept_band) = s.iter().zip(&keep).fold((0usize, 0usize), |(a, b), (x, &k)| {
        let r = x.u.hypot(x.v);
        let inb = r >= p.rc && r < p.rc + width;
        (a + inb as usize, b + (inb && k) as usize)
    });
    // ... and only where the orders explain a real share of the band (a curvature band, a ridge),
    // not the texture and noise of a plain sheet, where the ring fit itself is the best guess (a
    // leave-out test on the Revo scans: plain spots best filled by the quadric).
    let explained = {
        let (mut a, mut b, mut n) = (0.0, 0.0, 0usize);
        for (x, &k) in s.iter().zip(&keep) {
            let r = x.u.hypot(x.v);
            if k && r >= p.rc && r < p.rc + width {
                let e = x.h - q.eval(x.u, x.v);
                a += e * e;
                b += (e - model_at(&fitted, x)).powi(2);
                n += 1;
            }
        }
        ((a - b).max(0.0) / n.max(1) as f64).sqrt()
    };
    let slopes = p.slopes && p.all_round && kept_band as f64 >= SLOPE_MIN_PLAIN * in_band as f64 && explained >= SLOPE_MIN_EXPLAINED_MM;
    // Without the slopes the orders are not used at all: next to the cut the scan's band holds the
    // ragged rim of the gap round the sticker (flared points, the dark border's returns), and its
    // mean carried in by value lifted or sank whole fills by 0.1-0.2 mm on the Revo scans. The
    // ring fit, which averages the whole ring, is the fill there (the fairing matches the mesh
    // round the cut).
    let rms_q_only = rms_kept(&pts, &keep, &q);
    let rms_model = {
        let (mut acc, mut n) = (0.0, 0usize);
        for (x, &k) in s.iter().zip(&keep) {
            if k {
                acc += (x.h - q.eval(x.u, x.v) - model_at(&fitted, x)).powi(2);
                n += 1;
            }
        }
        (acc / n.max(1) as f64).sqrt()
    };
    fitted = if slopes { modes_of(&q, &keep, true) } else { None };
    let rms = rms_q_only;
    let kept = keep.iter().filter(|k| **k).count();
    let c1_modes = fitted.as_ref().map_or(0, |f| f.1);
    // No remainder of the scan points: what the orders leave in the band is texture, noise and the
    // ragged edge of the scan round the sticker (a gap's flared rim carried in showed as a blob);
    // the solve and the fairing (which matches the mesh round the cut) see to the continuity.
    let surf = SiteSurface { q, rc: p.rc, comps: fitted.map(|f| f.0).into_iter().collect(), rest: Vec::new() };
    Some(SiteFit { surf, rms, rms_model, keep, kept, sigma, c1_modes })
}

fn rms_kept(s: &[(f64, f64, f64)], keep: &[bool], q: &Quadric) -> f64 {
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

/// Basis of the band residual model: per order m and trig, t^0 and t^1 (t = r − rc, clamped to
/// `0 .. width`).
fn basis(m_max: usize, deg: usize, u: f64, v: f64, rc: f64, width: f64, out: &mut Vec<f64>) {
    out.clear();
    let t = (u.hypot(v) - rc).clamp(0.0, width);
    let th = v.atan2(u);
    for m in 0..=m_max {
        let trigs: &[f64] = if m == 0 { &[1.0] } else { &[(m as f64 * th).cos(), (m as f64 * th).sin()] };
        for &g in trigs {
            let mut tk = 1.0;
            for _ in 0..=deg {
                out.push(g * tk);
                tk *= t;
            }
        }
    }
}

/// Fits `resid` of the kept samples in the band `rc .. rc + width` with the low angular orders
/// (value and radial slope per order, see the module docs) and returns their continuation and the
/// number of orders continued with a slope weight above one half. Only the band next to the cut
/// is used: the ring further out may hold other surface (the start of a rim's roll-off, a seam
/// between two passes) whose shape says nothing about the slope at the cut.
fn fit_modes(s: &[RingSample], keep: &[bool], resid: &dyn Fn(&RingSample) -> f64, rc: f64, width: f64, m_max: usize, deg: usize, slopes: bool) -> Option<(Component, usize)> {
    let k = (deg + 1) * (2 * m_max + 1);
    let in_band = |x: &RingSample| {
        let r = x.u.hypot(x.v);
        r >= rc && r < rc + width
    };
    let band_keep: Vec<bool> = s.iter().zip(keep).map(|(x, &k)| k && in_band(x)).collect();
    let mut ata = vec![vec![0f64; k]; k];
    let mut atb = vec![0f64; k];
    let mut f = Vec::with_capacity(k);
    let mut n = 0usize;
    let mut sum_r2 = 0.0;
    for (x, _) in s.iter().zip(&band_keep).filter(|e| *e.1) {
        let r = resid(x);
        basis(m_max, deg, x.u, x.v, rc, width, &mut f);
        for i in 0..k {
            atb[i] += f[i] * r;
            for j in 0..k {
                ata[i][j] += f[i] * f[j];
            }
        }
        sum_r2 += r * r;
        n += 1;
    }
    if n < 6 * k {
        return None;
    }
    // A small ridge keeps a poorly covered order (a partial ring) from running away.
    for (i, row) in ata.iter_mut().enumerate() {
        row[i] += 1e-6 * (1.0 + row[i]);
    }
    let inv = invert(ata.clone())?;
    let c: Vec<f64> = (0..k).map(|i| (0..k).map(|j| inv[i][j] * atb[j]).sum()).collect();
    // The slope test needs the noise of the samples, not the misfit of the model (a ridge the low
    // orders follow only roughly leaves a smooth misfit as large as the noise of a plain site):
    // the scatter about local planes in cells of about ten samples, inflated for noise correlated
    // over [`NOISE_CORRELATION_MM2`] (scanner texture, the solve's ripples).
    let (noise2, density) = local_noise(s, &band_keep, resid);
    let fit_ss: f64 = c.iter().zip(&atb).map(|(a, b)| a * b).sum();
    let misfit2 = ((sum_r2 - fit_ss).max(0.0) / (n - k) as f64).max(1e-12);
    let s2 = noise2.unwrap_or(misfit2).min(misfit2).max(1e-12) * (density * NOISE_CORRELATION_MM2).max(1.0);
    let mut modes = Vec::new();
    let mut c1 = 0usize;
    let mut idx = 0usize;
    for m in 0..=m_max {
        for sin in if m == 0 { &[false][..] } else { &[false, true][..] } {
            let (val, slope) = (c[idx], c[idx + 1]);
            let se = (s2 * inv[idx + 1][idx + 1]).sqrt();
            let mf = m as f64;
            // The harmonic continuation of the order has slope m val / rc; the slope beyond it is
            // what the C1 continuation adds, weighted by how clearly it is there.
            let harmonic = mf * val / rc;
            let t = (slope - harmonic) / se.max(1e-12);
            let w = if slopes { t * t / (t * t + SLOPE_T0 * SLOPE_T0) } else { 0.0 };
            if w > 0.5 {
                c1 += 1;
            }
            let slope = harmonic + w * (slope - harmonic);
            // a rc^m + b rc^(m+2) = val; m a rc^(m-1) + (m+2) b rc^(m+1) = slope.
            let rm = rc.powi(m as i32);
            let b = (slope - mf * val / rc) / (2.0 * rm * rc);
            let a = (val - b * rm * rc * rc) / rm;
            if val.abs() > 1e-9 || b.abs() > 1e-12 {
                modes.push(Mode { m: m as u32, sin: *sin, a, b });
            }
            idx += deg + 1;
        }
    }
    Some((Component { modes, band: BandModel { m_max, deg, rc, width, c } }, c1))
}

/// Noise of `resid` of the kept samples at the scale of the sample spacing: the pooled scatter
/// about a plane fitted per cell of about ten samples (cells of 6 samples or more), and the
/// samples per mm² of those cells. `None` without such cells.
fn local_noise(s: &[RingSample], keep: &[bool], resid: &dyn Fn(&RingSample) -> f64) -> (Option<f64>, f64) {
    // Cells of about ten samples (1 mm for scan points at 0.3 mm, 1.6 mm at the merged model's
    // 0.5 mm pitch), from the samples' extent.
    let (mut lo, mut hi, mut n) = ((f64::INFINITY, f64::INFINITY), (f64::NEG_INFINITY, f64::NEG_INFINITY), 0usize);
    let mut r_lo = f64::INFINITY;
    let mut r_hi = 0f64;
    for (x, &k) in s.iter().zip(keep) {
        if k {
            lo = (lo.0.min(x.u), lo.1.min(x.v));
            hi = (hi.0.max(x.u), hi.1.max(x.v));
            let r = x.u.hypot(x.v);
            r_lo = r_lo.min(r);
            r_hi = r_hi.max(r);
            n += 1;
        }
    }
    if n < 30 {
        return (None, 1.0);
    }
    let area = std::f64::consts::PI * (r_hi * r_hi - r_lo * r_lo);
    let cell = (10.0 * area / n as f64).sqrt().clamp(0.5, 3.0);
    let mut cells: std::collections::HashMap<(i32, i32), Vec<(f64, f64, f64)>> = std::collections::HashMap::new();
    for (x, &k) in s.iter().zip(keep) {
        if k {
            cells.entry(((x.u / cell).floor() as i32, (x.v / cell).floor() as i32)).or_default().push((x.u, x.v, resid(x)));
        }
    }
    let (mut sse, mut dof, mut count, mut ncell) = (0.0, 0usize, 0usize, 0usize);
    for pts in cells.values() {
        if pts.len() < 6 {
            continue;
        }
        let (cu, cv) = pts.iter().fold((0.0, 0.0), |a, p| (a.0 + p.0, a.1 + p.1));
        let (cu, cv) = (cu / pts.len() as f64, cv / pts.len() as f64);
        let mut m = [[0f64; 3]; 3];
        let mut b = [0f64; 3];
        for &(u, v, r) in pts {
            let f = [1.0, u - cu, v - cv];
            for i in 0..3 {
                b[i] += f[i] * r;
                for j in 0..3 {
                    m[i][j] += f[i] * f[j];
                }
            }
        }
        for (i, row) in m.iter_mut().enumerate() {
            row[i] += 1e-9;
        }
        let Some(c) = super::markers::solve(m, b) else { continue };
        for &(u, v, r) in pts {
            let e = r - c[0] - c[1] * (u - cu) - c[2] * (v - cv);
            sse += e * e;
        }
        dof += pts.len() - 3;
        count += pts.len();
        ncell += 1;
    }
    if dof < 30 {
        return (None, 1.0);
    }
    // Samples per mm².
    (Some(sse / dof as f64), count as f64 / ncell as f64 / (cell * cell))
}

/// Gauss-Jordan inverse with partial pivoting.
fn invert(mut a: Vec<Vec<f64>>) -> Option<Vec<Vec<f64>>> {
    let n = a.len();
    let mut inv: Vec<Vec<f64>> = (0..n).map(|i| (0..n).map(|j| if i == j { 1.0 } else { 0.0 }).collect()).collect();
    for c in 0..n {
        let p = (c..n).max_by(|&i, &j| a[i][c].abs().total_cmp(&a[j][c].abs()))?;
        if a[p][c].abs() < 1e-14 {
            return None;
        }
        a.swap(c, p);
        inv.swap(c, p);
        let d = a[c][c];
        for j in 0..n {
            a[c][j] /= d;
            inv[c][j] /= d;
        }
        for r in 0..n {
            if r != c {
                let f = a[r][c];
                if f != 0.0 {
                    for j in 0..n {
                        a[r][j] -= f * a[c][j];
                        inv[r][j] -= f * inv[c][j];
                    }
                }
            }
        }
    }
    Some(inv)
}

/// Remainder of the surface against the ring fit and its continued orders over the disc
/// `r < rc + band` on a pixel grid: fixed (moving least squares of the samples) in the band,
/// harmonic inside.
#[derive(Clone)]
struct RemainderField {
    half: i32,
    side: usize,
    value: Vec<f64>,
}

impl RemainderField {
    fn build(s: &[RingSample], keep: &[bool], residual: impl Fn(&RingSample) -> f64, p: &SiteFitParams) -> Option<RemainderField> {
        let (rc, band) = (p.rc, p.band);
        let half = ((rc + band) / PX).ceil() as i32 + 1;
        let side = (2 * half + 1) as usize;
        let n = side * side;
        let xy = |k: usize| (((k % side) as i32 - half) as f64 * PX, ((k / side) as i32 - half) as f64 * PX);
        let sm = p.mls_mm;
        let reach = 2.5 * sm;
        let r_max = rc + band + reach;
        let cells = (r_max.ceil() as i32) + 1;
        let cside = (2 * cells + 1) as usize;
        let mut buckets: Vec<Vec<(f64, f64, f64)>> = vec![Vec::new(); cside * cside];
        for (x, &k) in s.iter().zip(keep) {
            let r = x.u.hypot(x.v);
            if k && r < r_max && r >= rc {
                let (a, b) = (x.u.floor() as i32 + cells, x.v.floor() as i32 + cells);
                if a >= 0 && b >= 0 && a < cside as i32 && b < cside as i32 {
                    buckets[b as usize * cside + a as usize].push((x.u, x.v, residual(x)));
                }
            }
        }
        let mut known = vec![false; n];
        let mut value = vec![0f64; n];
        let mut domain = vec![false; n];
        let rb = reach.ceil() as i32;
        for k in 0..n {
            let (u, v) = xy(k);
            let r = u.hypot(v);
            if r >= rc + band {
                continue;
            }
            domain[k] = true;
            if r < rc {
                continue;
            }
            let (ca, cb) = (u.floor() as i32 + cells, v.floor() as i32 + cells);
            let mut m = [[0f64; 3]; 3];
            let mut rhs = [0f64; 3];
            let mut nearest = f64::INFINITY;
            let mut wsum = 0.0;
            for db in -rb..=rb {
                for da in -rb..=rb {
                    let (a, b) = (ca + da, cb + db);
                    if a < 0 || b < 0 || a >= cside as i32 || b >= cside as i32 {
                        continue;
                    }
                    for &(su, sv, sr) in &buckets[b as usize * cside + a as usize] {
                        let (du, dv) = (su - u, sv - v);
                        let d2 = du * du + dv * dv;
                        if d2 > reach * reach {
                            continue;
                        }
                        nearest = nearest.min(d2);
                        let w = (-d2 / (2.0 * sm * sm)).exp();
                        let f = [1.0, du, dv];
                        for i in 0..3 {
                            rhs[i] += w * f[i] * sr;
                            for j in 0..3 {
                                m[i][j] += w * f[i] * f[j];
                            }
                        }
                        wsum += w;
                    }
                }
            }
            // A pixel in a gap of the data (nothing within a millimetre) is left to the solve.
            if wsum < 2.0 || nearest > 1.0 {
                continue;
            }
            for (i, row) in m.iter_mut().enumerate() {
                row[i] += 1e-6 * wsum * if i == 0 { 1.0 } else { sm * sm };
            }
            value[k] = match super::markers::solve(m, rhs) {
                Some(x) if x[0].is_finite() => x[0],
                _ => rhs[0] / wsum,
            };
            known[k] = true;
        }
        let vmax = (0..n).filter(|&k| known[k]).map(|k| value[k].abs()).fold(0.0, f64::max);
        if vmax < 1e-7 {
            return None;
        }
        let solved = harmonic(side, &domain, &known, &value);
        // Outside the domain one pixel ring copies its neighbours, so the bilinear lookup near the
        // band's outer edge stays continuous (the field is read inside the cut only).
        let mut out: Vec<f64> = (0..n).map(|k| if domain[k] { solved[k].clamp(-vmax, vmax) } else { 0.0 }).collect();
        for k in 0..n {
            if domain[k] {
                continue;
            }
            let (a, b) = ((k % side) as i32, (k / side) as i32);
            let (mut s2, mut c) = (0.0, 0);
            for (da, db) in [(1, 0), (-1, 0), (0, 1), (0, -1)] {
                let (x, y) = (a + da, b + db);
                if x >= 0 && y >= 0 && x < side as i32 && y < side as i32 && domain[y as usize * side + x as usize] {
                    s2 += out[y as usize * side + x as usize];
                    c += 1;
                }
            }
            if c > 0 {
                out[k] = s2 / c as f64;
            }
        }
        Some(RemainderField { half, side, value: out })
    }

    /// Bilinear sample at (u, v).
    fn at(&self, u: f64, v: f64) -> f64 {
        let (x, y) = (u / PX + self.half as f64, v / PX + self.half as f64);
        let (x0, y0) = (x.floor().clamp(0.0, (self.side - 2) as f64), y.floor().clamp(0.0, (self.side - 2) as f64));
        let (tx, ty) = ((x - x0).clamp(0.0, 1.0), (y - y0).clamp(0.0, 1.0));
        let k = y0 as usize * self.side + x0 as usize;
        let v = &self.value;
        (v[k] * (1.0 - tx) + v[k + 1] * tx) * (1.0 - ty) + (v[k + self.side] * (1.0 - tx) + v[k + self.side + 1] * tx) * ty
    }
}

/// Harmonic continuation of the `known` pixel values over the other `domain` pixels of a square
/// grid (Laplace equation, Neumann at the domain's edge): successive over-relaxation from the
/// mean of the known values. Pixels outside the domain keep 0.
fn harmonic(side: usize, domain: &[bool], known: &[bool], fixed: &[f64]) -> Vec<f64> {
    let n = domain.len();
    let mut value: Vec<f64> = (0..n).map(|k| if known[k] { fixed[k] } else { 0.0 }).collect();
    let (fs, fc) = (0..n).filter(|&k| known[k]).fold((0.0, 0usize), |(s, c), k| (s + fixed[k], c + 1));
    if fc == 0 {
        return value;
    }
    let start = fs / fc as f64;
    let unknown: Vec<usize> = (0..n).filter(|&k| domain[k] && !known[k]).collect();
    for &k in &unknown {
        value[k] = start;
    }
    for _ in 0..2000 {
        let mut change = 0f64;
        for &k in &unknown {
            let (a, b) = ((k % side) as i32, (k / side) as i32);
            let (mut s, mut c) = (0f64, 0u32);
            for (da, db) in [(1, 0), (-1, 0), (0, 1), (0, -1)] {
                let (x, y) = (a + da, b + db);
                if x < 0 || y < 0 || x >= side as i32 || y >= side as i32 {
                    continue;
                }
                let kk = y as usize * side + x as usize;
                if domain[kk] {
                    s += value[kk];
                    c += 1;
                }
            }
            if c > 0 {
                let new = value[k] + 1.9 * (s / c as f64 - value[k]);
                change = change.max((new - value[k]).abs());
                value[k] = new;
            }
        }
        if change < 1e-7 {
            break;
        }
    }
    value
}

/// Convex hull (counter-clockwise) of 2-D points (Andrew's monotone chain).
pub(crate) fn convex_hull(mut p: Vec<(f32, f32)>) -> Vec<(f32, f32)> {
    p.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.total_cmp(&b.1)));
    p.dedup();
    if p.len() < 3 {
        return p;
    }
    let cross = |o: (f32, f32), a: (f32, f32), b: (f32, f32)| (a.0 - o.0) * (b.1 - o.1) - (a.1 - o.1) * (b.0 - o.0);
    let mut h: Vec<(f32, f32)> = Vec::with_capacity(2 * p.len());
    for &q in &p {
        while h.len() >= 2 && cross(h[h.len() - 2], h[h.len() - 1], q) <= 0.0 {
            h.pop();
        }
        h.push(q);
    }
    let lower = h.len() + 1;
    for &q in p.iter().rev().skip(1) {
        while h.len() >= lower && cross(h[h.len() - 2], h[h.len() - 1], q) <= 0.0 {
            h.pop();
        }
        h.push(q);
    }
    h.pop();
    h
}

/// Distance of (u, v) outside the convex polygon `hull` (counter-clockwise; negative inside):
/// the largest distance beyond any edge line.
pub(crate) fn outside_hull(hull: &[(f32, f32)], u: f32, v: f32) -> f32 {
    if hull.len() < 3 {
        return f32::INFINITY;
    }
    let mut worst = f32::NEG_INFINITY;
    for i in 0..hull.len() {
        let (a, b) = (hull[i], hull[(i + 1) % hull.len()]);
        let (ex, ey) = (b.0 - a.0, b.1 - a.1);
        let len = (ex * ex + ey * ey).sqrt().max(1e-6);
        // Outward normal of a counter-clockwise edge: (ey, -ex).
        worst = worst.max(((u - a.0) * ey - (v - a.1) * ex) / len);
    }
    worst
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Samples of the ring `rc .. rc + w` of the height field `f` on an `s` grid, with
    /// deterministic noise of amplitude `noise`.
    fn ring(f: impl Fn(f64, f64) -> f64, rc: f64, w: f64, s: f64, noise: f64) -> Vec<RingSample> {
        let k = ((rc + w) / s).ceil() as i32;
        let mut out = Vec::new();
        for i in -k..=k {
            for j in -k..=k {
                let (u, v) = (i as f64 * s, j as f64 * s);
                let r = u.hypot(v);
                if r >= rc && r < rc + w {
                    let e = 1e-3;
                    let n = DVec3::new(-(f(u + e, v) - f(u - e, v)) / (2.0 * e), -(f(u, v + e) - f(u, v - e)) / (2.0 * e), 1.0).normalize();
                    let z = noise * ((((i * 7919 + j * 104_729) as f64).sin() * 43_758.545).fract().abs() * 2.0 - 1.0) * 3f64.sqrt();
                    out.push(RingSample { u, v, h: f(u, v) + z, n });
                }
            }
        }
        out
    }

    fn params(rc: f64) -> SiteFitParams {
        SiteFitParams { rc, band: 3.0, mls_mm: 1.0, all_round: true, rough_mm: 0.05, slopes: true }
    }

    fn worst_inside(surf: &SiteSurface, f: impl Fn(f64, f64) -> f64, rc: f64) -> f64 {
        let mut w = 0f64;
        for i in -40..=40 {
            for j in -40..=40 {
                let (u, v) = (i as f64 * rc / 40.0, j as f64 * rc / 40.0);
                if u.hypot(v) < rc {
                    w = w.max((surf.eval(u, v) - f(u, v)).abs());
                }
            }
        }
        w
    }

    #[test]
    fn plain_sphere_stays_on_the_ring_fit_even_with_noise() {
        let f = |u: f64, v: f64| 100.0 - (100.0f64 * 100.0 - u * u - v * v).sqrt();
        for noise in [0.0, 0.03] {
            let fit = fit_site(&ring(f, 6.5, 6.0, 0.3, noise), &params(6.5)).unwrap();
            let w = worst_inside(&fit.surf, f, 5.5);
            eprintln!("noise {noise}: fill worst {w:.4} mm (sigma {:.4}, {} of {} kept, {} orders continued C1)", fit.sigma, fit.kept, fit.keep.len(), fit.c1_modes);
            assert!(w < 0.003 + 0.3 * noise, "noise {noise}: fill {w} mm off the sphere");
        }
    }

    #[test]
    fn ridge_and_bump_round_the_site_are_continued_not_averaged() {
        // A ridge (Gaussian, sigma 5 mm, 0.15 mm high) across a dome, and a bump (0.2 mm,
        // sigma 4 mm) beside the site: the ring fit alone misses the inside by 0.04-0.08 mm.
        let ridge = |u: f64, v: f64| 0.004 * (u * u + v * v) + 0.15 * (-(u - 1.5).powi(2) / 50.0).exp();
        let bump = |u: f64, v: f64| 0.002 * (u * u + v * v) + 0.2 * (-((u - 8.0).powi(2) + (v - 3.0).powi(2)) / 32.0).exp();
        for (name, f, lim) in [("ridge", &ridge as &dyn Fn(f64, f64) -> f64, 0.012), ("bump", &bump, 0.025)] {
            let fit = fit_site(&ring(f, 6.5, 6.0, 0.3, 0.0), &params(6.5)).unwrap();
            let w = worst_inside(&fit.surf, f, 5.5);
            let wq = worst_inside(&SiteSurface::from_quadric(fit.surf.q), f, 6.5);
            assert!(wq > 0.025, "{name}: the ring fit alone would do: {wq}");
            assert!(w < lim, "{name}: fill {w} mm off (ring fit alone {wq})");
        }
    }

    #[test]
    fn rim_roll_off_in_the_ring_does_not_pull_the_fit() {
        // A dome (R 80 mm) that rolls off over a 2 mm radius edge 9 mm from the site: the roll-off
        // samples in the ring face away from the fit and are not used.
        let edge = 9.0;
        let dome = |u: f64, v: f64| -(u * u + v * v) / 160.0;
        let f = |u: f64, v: f64| {
            let d = u - (edge - 2.0);
            dome(u, v) - if d > 0.0 { 2.0 - (4.0 - d.min(2.0).powi(2)).max(0.0).sqrt() } else { 0.0 }
        };
        let s: Vec<RingSample> = ring(f, 6.5, 6.0, 0.3, 0.0).into_iter().filter(|x| x.u < edge).collect();
        let fit = fit_site(&s, &SiteFitParams { all_round: false, ..params(6.5) }).unwrap();
        let w = worst_inside(&fit.surf, dome, 6.0);
        assert!(w < 0.01, "fill {w} mm off the dome");
    }

    #[test]
    fn hull_distance() {
        let h = convex_hull(vec![(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0), (0.5, 0.5)]);
        assert_eq!(h.len(), 4);
        assert!(outside_hull(&h, 0.5, 0.5) < 0.0);
        assert!((outside_hull(&h, 1.5, 0.5) - 0.5).abs() < 1e-6);
    }
}
