//! Filling of holes in a sheet (data gaps that the thin-wall solve cannot span).
//!
//! A Poisson indicator closes a gap in one face of a wall when the other face is present, but a
//! gap through both faces that is wider than about the wall thickness stays open as a tunnel.
//! Such gaps are filled before the solve: around seeds every quarter window, the same-facing
//! points of the sheet (measured, marker fill and completion points) are fitted with a quadric
//! height field and rasterised into a 0.5 mm image of residual heights. Two kinds of empty
//! regions are patched:
//!
//! * **Enclosed holes:** not connected to the window border, not larger than `max_radius_frac` of
//!   the window (inscribed radius) and surrounded by a well-covered ring.
//! * **Bridged gaps:** long narrow gaps that run out of every window (so they never look
//!   enclosed). An empty pixel is bridged when, along one of four directions, the sheet lies on
//!   both sides of it within `bridge_span_mm` in all. The open side of a rim never has sheet
//!   beyond it, so rims stay open.
//!
//! A region whose border carries measured points facing across the sheet (the cut face of the
//! wall seen by the scanner) on more than `edge_wall_fraction` of its length is taken for a real
//! opening (a bore, a slot) and left open; so is a round enclosed hole of 1–6 mm radius through a
//! plate scanned from both sides, with nothing on its axis (`through_bore`). Patches are points on the quadric plus a harmonic
//! (Laplace) interpolation of the residual from the surrounding pixels, so they meet the sheet
//! without a step. Passes run from small to large windows; a second completion pass then builds
//! the opposite face behind them.

use super::grid::{cell_key, FxMap, PointGrid};
use super::markers::{chamfer_dt, tangent_frame, Quadric};
use super::{PointKind, WorkPoint};
use glam::{DVec3, Vec3};
use rayon::prelude::*;

#[derive(Clone, Debug)]
pub struct HoleConfig {
    /// Half widths of the local windows (mm), one pass per entry: small windows fit the sheet
    /// better, large ones catch long gaps.
    pub windows_mm: Vec<f32>,
    /// Largest inscribed radius of a filled hole, as a fraction of the window half width.
    pub max_radius_frac: f32,
    /// Quadric RMS limit for the local sheet at a 24 mm window (scaled with the window).
    pub fit_rms_max_mm: f32,
    /// Fraction of the 3 mm ring around the hole that must be covered.
    pub ring_coverage: f32,
    /// Largest span (sheet to sheet across the gap) of a bridged gap (mm); 0 = no bridging.
    pub bridge_span_mm: f32,
    /// Bridging runs in the passes whose window is at most this (mm).
    pub bridge_max_window_mm: f32,
    /// A region is a real opening when this fraction of its border shows the wall's cut face.
    pub edge_wall_fraction: f32,
    /// Leave round enclosed holes open whose axis is clear of points on both sides and whose plate
    /// was scanned from both sides round them (screw holes through a plate; see `through_bore`).
    /// A gap in a face scanned from one side only is always filled: the data cannot tell it from
    /// a sticker gap or a dropout.
    pub keep_through_holes: bool,
}

impl Default for HoleConfig {
    fn default() -> Self {
        Self {
            windows_mm: vec![24.0, 40.0, 64.0],
            max_radius_frac: 0.45,
            fit_rms_max_mm: 0.25,
            ring_coverage: 0.85,
            bridge_span_mm: 40.0,
            bridge_max_window_mm: 40.0,
            edge_wall_fraction: 0.4,
            keep_through_holes: true,
        }
    }
}

const PX: f64 = 0.5;

/// Returns the fill points (kind `HoleFill`) and the number of separate patches: connected
/// groups of fill points over all passes (a hole seen from several seeds, or grown by a later
/// pass, counts once).
pub(crate) fn fill_holes(pts: &[WorkPoint], prm: &HoleConfig) -> (Vec<WorkPoint>, usize) {
    let mut all = pts.to_vec();
    let mut out = Vec::new();
    let mut bores = Vec::new();
    for &w in &prm.windows_mm {
        let fill = fill_pass(&all, prm, w, &mut bores);
        all.extend_from_slice(&fill);
        out.extend(fill);
    }
    let patches = count_patches(&out, 1.0);
    (out, patches)
}

/// Number of connected groups of points (26-connected occupied cells of size `cell`).
fn count_patches(pts: &[WorkPoint], cell: f32) -> usize {
    let mut cells: FxMap<u64, (glam::IVec3, bool)> = FxMap::default();
    for p in pts {
        let c = (p.p / cell).floor().as_ivec3();
        cells.entry(cell_key(c)).or_insert((c, false));
    }
    let keys: Vec<u64> = cells.keys().copied().collect();
    let mut patches = 0usize;
    let mut stack: Vec<glam::IVec3> = Vec::new();
    for k in keys {
        let (c, done) = cells[&k];
        if done {
            continue;
        }
        patches += 1;
        cells.get_mut(&k).unwrap().1 = true;
        stack.push(c);
        while let Some(c) = stack.pop() {
            for dz in -1..=1 {
                for dy in -1..=1 {
                    for dx in -1..=1 {
                        let nc = c + glam::IVec3::new(dx, dy, dz);
                        if let Some(e) = cells.get_mut(&cell_key(nc)) {
                            if !e.1 {
                                e.1 = true;
                                stack.push(nc);
                            }
                        }
                    }
                }
            }
        }
    }
    patches
}

/// A hole left open in one face ([`through_bore`]): centre on the sheet, axis (the sheet normal)
/// and radius (mm).
#[derive(Clone, Copy, Debug)]
struct Bore {
    c: Vec3,
    a: Vec3,
    r: f32,
}

impl Bore {
    /// Whether a fill point (`p`, normal `n`) of the same face lies in the bore. Each face decides
    /// for itself: a gap through both faces may be a bore in one and too wide in the other, which
    /// is then patched and gets the other face built behind it by the completion.
    fn contains(&self, p: Vec3, n: Vec3) -> bool {
        let d = p - self.c;
        let h = d.dot(self.a);
        n.dot(self.a) > 0.5 && h.abs() < 1.5 && (d - self.a * h).length_squared() < self.r * self.r
    }
}

/// One pass at window half width `w`. `bores` holds the holes left open so far and gains this
/// pass's: no seed fills a bore another seed (or a smaller window) kept open in that face. A seed
/// far from a hole can see only part of it, too small for a bore.
fn fill_pass(pts: &[WorkPoint], prm: &HoleConfig, w: f32, bores: &mut Vec<Bore>) -> Vec<WorkPoint> {
    let pos: Vec<Vec3> = pts.iter().map(|p| p.p).collect();
    let grid = PointGrid::new(&pos, 2.0);
    let seeds = PointGrid::new(&pos, w / 4.0).representatives();
    let per_seed: Vec<(Vec<WorkPoint>, Vec<Bore>)> = seeds
        .par_iter()
        .map(|&s| {
            let (fill, _, b) = holes_at(s as usize, pts, &pos, &grid, prm, w);
            (fill, b)
        })
        .collect();
    for (_, b) in &per_seed {
        bores.extend_from_slice(b);
    }
    // Holes are seen from several seeds: keep one point per 0.35 mm cell.
    let mut seen: FxMap<u64, ()> = FxMap::default();
    let mut out = Vec::new();
    for (fill, _) in per_seed {
        for p in fill {
            if bores.iter().any(|b| b.contains(p.p, p.n)) {
                continue;
            }
            if seen.insert(cell_key((p.p / 0.35).floor().as_ivec3()), ()).is_none() {
                out.push(p);
            }
        }
    }
    out
}

fn holes_at(
    si: usize,
    pts: &[WorkPoint],
    pos: &[Vec3],
    grid: &PointGrid,
    prm: &HoleConfig,
    w: f32,
) -> (Vec<WorkPoint>, usize, Vec<Bore>) {
    let s = &pts[si];
    let n = s.n;
    let (eu, ev) = tangent_frame(n);
    let mut loc: Vec<(f64, f64, f64)> = Vec::new();
    // Sheet pixels carrying measured points (only measured sheets anchor a bridge; the opposite
    // face behind a bridge is built by the completion afterwards).
    let mut loc_measured: Vec<(f64, f64)> = Vec::new();
    // Measured points facing across the sheet: the cut face of the wall at a real opening.
    let mut edge_pts: Vec<(f64, f64)> = Vec::new();
    // Points of any other orientation near the sheet (the opposite face, ribs, plates).
    let mut other: Vec<(f64, f64, f64)> = Vec::new();
    let mut opposite: Vec<(f64, f64, f64)> = Vec::new();
    // Real surface (measured or marker fill, not completion: that is a copy of the seen face) on the
    // sheet and opposite-facing behind it: both scanned faces of a plate round a through hole.
    let mut loc_real: Vec<(f64, f64)> = Vec::new();
    let mut opposite_real: Vec<(f64, f64, f64)> = Vec::new();
    grid.for_each_within(pos, s.p, w, |i, _| {
        let p = &pts[i as usize];
        let d = p.p - s.p;
        let h = d.dot(n);
        let c = p.n.dot(n);
        if c.abs() < 0.5 && p.kind == PointKind::Measured && h.abs() < 4.0 {
            edge_pts.push((d.dot(eu) as f64, d.dot(ev) as f64));
        }
        if c < 0.6 {
            if h.abs() < 9.0 {
                other.push((d.dot(eu) as f64, d.dot(ev) as f64, h as f64));
                if c < -0.6 {
                    opposite.push((d.dot(eu) as f64, d.dot(ev) as f64, h as f64));
                    if matches!(p.kind, PointKind::Measured | PointKind::MarkerFill) {
                        opposite_real.push((d.dot(eu) as f64, d.dot(ev) as f64, h as f64));
                    }
                }
            }
            return;
        }
        if h.abs() < 4.0 {
            loc.push((d.dot(eu) as f64, d.dot(ev) as f64, h as f64));
            if p.kind == PointKind::Measured {
                loc_measured.push((d.dot(eu) as f64, d.dot(ev) as f64));
            }
            if matches!(p.kind, PointKind::Measured | PointKind::MarkerFill) {
                loc_real.push((d.dot(eu) as f64, d.dot(ev) as f64));
            }
        }
    });
    let none = (Vec::new(), 0, Vec::new());
    if loc.len() < 200 {
        return none;
    }
    let Some((q, rms, _)) = Quadric::fit_robust(&loc) else { return none };
    if rms > (prm.fit_rms_max_mm * (w / 24.0).max(1.0)) as f64 {
        return none;
    }
    let half = (w as f64 / PX).floor() as i32;
    let side = (2 * half + 1) as usize;
    let mut occ = vec![false; side * side];
    // Mean residual per pixel (pixels hit by a point).
    let mut rsum = vec![0f64; side * side];
    let mut rcnt = vec![0u32; side * side];
    let lim = 0.8f64.max(3.0 * rms);
    for &(u, v, z) in &loc {
        let r = z - q.eval(u, v);
        if r.abs() > lim {
            continue;
        }
        let (iu, iv) = ((u / PX).round() as i32 + half, (v / PX).round() as i32 + half);
        if iu >= 0 && iv >= 0 && iu < side as i32 && iv < side as i32 {
            rsum[iv as usize * side + iu as usize] += r;
            rcnt[iv as usize * side + iu as usize] += 1;
        }
        for dv in -1..=1 {
            for du in -1..=1 {
                let (a, b) = (iu + du, iv + dv);
                if a >= 0 && b >= 0 && a < side as i32 && b < side as i32 {
                    occ[b as usize * side + a as usize] = true;
                }
            }
        }
    }
    let r_in = (w as f64 / PX - 1.0).powi(2);
    let inside = |a: i32, b: i32| ((a - half).pow(2) + (b - half).pow(2)) as f64 <= r_in;

    // Label empty 4-connected components inside the window circle.
    let mut label = vec![u32::MAX; side * side];
    let mut comps: Vec<(Vec<usize>, bool)> = Vec::new(); // (pixels, touches border)
    for start in 0..side * side {
        if occ[start] || label[start] != u32::MAX {
            continue;
        }
        let (sa, sb) = ((start % side) as i32, (start / side) as i32);
        if !inside(sa, sb) {
            continue;
        }
        let id = comps.len() as u32;
        let mut stack = vec![start];
        label[start] = id;
        let mut pix = Vec::new();
        let mut open = false;
        while let Some(k) = stack.pop() {
            pix.push(k);
            let (a, b) = ((k % side) as i32, (k / side) as i32);
            for (da, db) in [(1, 0), (-1, 0), (0, 1), (0, -1)] {
                let (x, y) = (a + da, b + db);
                if x < 0 || y < 0 || x >= side as i32 || y >= side as i32 || !inside(x, y) {
                    open = true;
                    continue;
                }
                let kk = y as usize * side + x as usize;
                if !occ[kk] && label[kk] == u32::MAX {
                    label[kk] = id;
                    stack.push(kk);
                }
            }
        }
        comps.push((pix, open));
    }

    let mut edge = vec![false; side * side];
    for &(u, v) in &edge_pts {
        let (iu, iv) = ((u / PX).round() as i32 + half, (v / PX).round() as i32 + half);
        for dv in -2..=2 {
            for du in -2..=2 {
                let (a, b) = (iu + du, iv + dv);
                if a >= 0 && b >= 0 && a < side as i32 && b < side as i32 {
                    edge[b as usize * side + a as usize] = true;
                }
            }
        }
    }
    // Fraction of the 1.5 mm sheet border of a region that shows the cut face of the wall.
    let edge_fraction = |dt_region: &[f32]| {
        let (mut ring, mut hit) = (0usize, 0usize);
        for k in 0..side * side {
            let d = dt_region[k] as f64 * PX;
            if d > 0.0 && d <= 1.5 && occ[k] {
                ring += 1;
                if edge[k] {
                    hit += 1;
                }
            }
        }
        if ring == 0 {
            0.0
        } else {
            hit as f32 / ring as f32
        }
    };
    let bridge = prm.bridge_span_mm > 0.0 && w <= prm.bridge_max_window_mm;
    // Bridging is for gaps through the whole wall: a pixel with any surface within 4.5 mm of
    // the sheet along the normal (the opposite face, a rib, a parallel plate, or the sheet itself
    // off the fit) is not bridged; one-sided gaps are left to the completion.
    let mut column = vec![false; if bridge { side * side } else { 0 }];
    if bridge {
        let mut mark = |u: f64, v: f64| {
            let (iu, iv) = ((u / PX).round() as i32 + half, (v / PX).round() as i32 + half);
            for dv in -1..=1 {
                for du in -1..=1 {
                    let (a, b) = (iu + du, iv + dv);
                    if a >= 0 && b >= 0 && a < side as i32 && b < side as i32 {
                        column[b as usize * side + a as usize] = true;
                    }
                }
            }
        };
        for &(u, v, z) in other.iter().chain(&loc) {
            if (z - q.eval(u, v)).abs() < 4.5 {
                mark(u, v);
            }
        }
    }
    // Wall pixels: measured sheet pixels with the opposite face of a wall (opposite-facing points
    // 1.5–5 mm behind). Only gaps with wall on both sides are bridged, so plates, ribs and
    // brackets (thinner, or with nothing behind them) never anchor a bridge.
    let mut wall = vec![false; if bridge { side * side } else { 0 }];
    if bridge {
        for &(u, v, z) in &opposite {
            let d = q.eval(u, v) - z;
            if (1.5..5.0).contains(&d) {
                let (iu, iv) = ((u / PX).round() as i32 + half, (v / PX).round() as i32 + half);
                for dv in -2..=2 {
                    for du in -2..=2 {
                        let (a, b) = (iu + du, iv + dv);
                        if a >= 0 && b >= 0 && a < side as i32 && b < side as i32 {
                            wall[b as usize * side + a as usize] = true;
                        }
                    }
                }
            }
        }
        let mut meas = vec![false; side * side];
        for &(u, v) in &loc_measured {
            let (iu, iv) = ((u / PX).round() as i32 + half, (v / PX).round() as i32 + half);
            for dv in -1..=1 {
                for du in -1..=1 {
                    let (a, b) = (iu + du, iv + dv);
                    if a >= 0 && b >= 0 && a < side as i32 && b < side as i32 {
                        meas[b as usize * side + a as usize] = true;
                    }
                }
            }
        }
        for k in 0..side * side {
            wall[k] &= occ[k] && meas[k];
        }
    }
    let span = if bridge { min_span(&wall, side) } else { Vec::new() };
    let span_max = (prm.bridge_span_mm as f64 / PX) as f32;
    // Patch points over the region's pixels, except those `skip` marks (a bore left open).
    let emit = |pix: &[usize], dt_hole: &[f32], skip: Option<&[bool]>, fill: &mut Vec<WorkPoint>| {
        let resid = harmonic_fill(pix, dt_hole, &rsum, &rcnt, side);
        for (&k, &rk) in pix.iter().zip(&resid) {
            if skip.is_some_and(|s| s[k]) {
                continue;
            }
            let (u, v) = (((k % side) as i32 - half) as f64 * PX, ((k / side) as i32 - half) as f64 * PX);
            let z = q.eval(u, v) + rk;
            let (gu, gv) = q.grad(u, v);
            let nl = DVec3::new(-gu, -gv, 1.0).normalize();
            let p = s.p + eu * u as f32 + ev * v as f32 + n * z as f32;
            let nn = (eu * nl.x as f32 + ev * nl.y as f32 + n * nl.z as f32).normalize();
            fill.push(WorkPoint { p, n: nn, scan: 0, kind: PointKind::HoleFill, area: 0.0 });
        }
    };

    let mut fill = Vec::new();
    let mut count = 0usize;
    let mut bores = Vec::new();
    let dt_occ = chamfer_dt(&occ, side);
    for (pix, open) in &comps {
        if pix.len() < 2 {
            continue;
        }
        let r_ins = pix.iter().map(|&k| dt_occ[k]).fold(0f32, f32::max) as f64 * PX + PX;
        if *open || r_ins > (prm.max_radius_frac * w) as f64 {
            if !bridge {
                continue;
            }
            // Bridged part of an open or oversized region, in the inner half of the window
            // (other seeds take the rest).
            let lim2 = (0.5 * w as f64 / PX).powi(2);
            let bp: Vec<usize> = pix
                .iter()
                .copied()
                .filter(|&k| {
                    let (a, b) = ((k % side) as i32 - half, (k / side) as i32 - half);
                    ((a * a + b * b) as f64) <= lim2 && span[k] <= span_max && !column[k]
                })
                .collect();
            // A real gap is at least 3 mm wide somewhere; notches between samples along a
            // jagged rim are not.
            if bp.len() < 4 || (bp.iter().map(|&k| dt_occ[k]).fold(0f32, f32::max) as f64) * PX < 1.5 {
                continue;
            }
            let mut mask = vec![false; side * side];
            for &k in &bp {
                mask[k] = true;
            }
            let dt_hole = chamfer_dt(&mask, side);
            if edge_fraction(&dt_hole) > prm.edge_wall_fraction {
                continue;
            }
            count += 1;
            emit(&bp, &dt_hole, None, &mut fill);
            continue;
        }
        // Only holes centred in the inner half of the window (other seeds take the rest).
        let (mut cu, mut cv) = (0f64, 0f64);
        for &k in pix {
            cu += (k % side) as f64 - half as f64;
            cv += (k / side) as f64 - half as f64;
        }
        cu *= PX / pix.len() as f64;
        cv *= PX / pix.len() as f64;
        if (cu * cu + cv * cv).sqrt() > 0.5 * w as f64 {
            continue;
        }
        // Ring of 3 mm around the hole must be covered by the sheet.
        let mut mask = vec![false; side * side];
        for &k in pix {
            mask[k] = true;
        }
        let dt_hole = chamfer_dt(&mask, side);
        let (mut ring, mut covered) = (0usize, 0usize);
        for k in 0..side * side {
            let d = dt_hole[k] as f64 * PX;
            if d > 0.0 && d <= 3.0 && inside((k % side) as i32, (k / side) as i32) {
                ring += 1;
                if occ[k] {
                    covered += 1;
                }
            }
        }
        if ring == 0 || (covered as f32) < prm.ring_coverage * ring as f32 {
            continue;
        }
        if edge_fraction(&dt_hole) > prm.edge_wall_fraction {
            continue;
        }
        // A real opening through the part stays open; the rest of the region (a shadowed gap
        // beside it) is still patched.
        // The clear part ends 0.5 mm short of the bore's wall (the axis test's radius); the region
        // within 1 mm of it stays open too, so the bore keeps its size and a later, larger window
        // sees the same hole and not a smaller remainder of it.
        let bore = if prm.keep_through_holes {
            through_bore(pix, side, half, &q, s.p, (eu, ev, n), pts, pos, grid)
                .filter(|open| both_faces_round(open, &occ, &loc_real, &opposite_real, &q, side, half))
                .map(|mut open| {
                    let dt = chamfer_dt(&open, side);
                    for &k in pix {
                        if dt[k] as f64 * PX <= 1.0 {
                            open[k] = true;
                        }
                    }
                    open
                })
        } else {
            None
        };
        if let Some(open) = &bore {
            let (mut su, mut sv, mut cnt) = (0f64, 0f64, 0usize);
            for k in (0..side * side).filter(|&k| open[k]) {
                su += ((k % side) as i32 - half) as f64 * PX;
                sv += ((k / side) as i32 - half) as f64 * PX;
                cnt += 1;
            }
            let (u, v) = (su / cnt as f64, sv / cnt as f64);
            let (gu, gv) = q.grad(u, v);
            let nl = DVec3::new(-gu, -gv, 1.0).normalize();
            bores.push(Bore {
                c: s.p + eu * u as f32 + ev * v as f32 + n * q.eval(u, v) as f32,
                a: (eu * nl.x as f32 + ev * nl.y as f32 + n * nl.z as f32).normalize(),
                r: ((cnt as f64 * PX * PX / std::f64::consts::PI).sqrt()) as f32,
            });
        }
        if bore.as_ref().is_some_and(|open| pix.iter().filter(|&&k| !open[k]).count() < 2) {
            continue;
        }
        count += 1;
        emit(pix, &dt_hole, bore.as_deref(), &mut fill);
    }
    (fill, count, bores)
}

/// The bore of a real opening through the part inside an enclosed hole region (a screw hole in a
/// bracket plate, often merged with a shadowed gap beside it), as a pixel mask: the pixels whose
/// line along the sheet normal is clear of real surface (measured or marker fill points) within
/// 0.5 mm for 30 mm on either side (no opposite face of a wall, no other plate, no rib), connected
/// to the deepest such pixel. `None` when the clear part holds no disc of 1–6 mm radius or is not
/// round (more than [`ROUND_AREA_RATIO`] times its inscribed disc). A data gap in one face of a
/// wall whose other face was scanned has that face behind it and is filled. Completion points do
/// not count: behind a plate they are a guess, and behind a gap in a one-sided scan they have the
/// same gap; the caller therefore also asks for both faces scanned round the bore
/// ([`both_faces_round`]).
#[allow(clippy::too_many_arguments)]
fn through_bore(
    pix: &[usize],
    side: usize,
    half: i32,
    q: &Quadric,
    origin: Vec3,
    (eu, ev, n): (Vec3, Vec3, Vec3),
    pts: &[WorkPoint],
    pos: &[Vec3],
    grid: &PointGrid,
) -> Option<Vec<bool>> {
    const RADIAL: f32 = 0.5;
    const REACH: f32 = 30.0;
    let clear_at = |k: usize| {
        let (u, v) = (((k % side) as i32 - half) as f64 * PX, ((k / side) as i32 - half) as f64 * PX);
        let (gu, gv) = q.grad(u, v);
        let nl = DVec3::new(-gu, -gv, 1.0).normalize();
        let axis = (eu * nl.x as f32 + ev * nl.y as f32 + n * nl.z as f32).normalize();
        let c = origin + eu * u as f32 + ev * v as f32 + n * q.eval(u, v) as f32;
        // Outwards from the sheet in both directions: the opposite face of a wall is met first.
        let mut t = 0.0f32;
        while t <= REACH {
            for x in [c + axis * t, c - axis * t] {
                let mut hit = false;
                grid.for_each_within(pos, x, 2.0 * RADIAL, |i, _| {
                    let p = &pts[i as usize];
                    if !hit && matches!(p.kind, PointKind::Measured | PointKind::MarkerFill) {
                        let d = p.p - c;
                        let h = d.dot(axis);
                        hit = (d - axis * h).length_squared() < RADIAL * RADIAL;
                    }
                });
                if hit {
                    return false;
                }
            }
            t += RADIAL;
        }
        true
    };
    let clear: Vec<(usize, bool)> = pix.par_iter().map(|&k| (k, clear_at(k))).collect();
    let mut blocked = vec![true; side * side];
    for &(k, c) in &clear {
        blocked[k] = !c;
    }
    let dt = chamfer_dt(&blocked, side);
    let (deepest, depth) = clear.iter().filter(|e| e.1).map(|e| (e.0, dt[e.0])).max_by(|a, b| a.1.total_cmp(&b.1))?;
    // chamfer_dt is in pixels, 3-4 weights normalised to one pixel per step.
    let radius = depth as f64 * PX;
    if !(1.0..=6.0).contains(&radius) {
        return None;
    }
    // The clear pixels connected to the deepest one.
    let mut open = vec![false; side * side];
    open[deepest] = true;
    let mut stack = vec![deepest];
    while let Some(k) = stack.pop() {
        let (a, b) = ((k % side) as i32, (k / side) as i32);
        for (da, db) in [(1, 0), (-1, 0), (0, 1), (0, -1)] {
            let (x, y) = (a + da, b + db);
            if x < 0 || y < 0 || x >= side as i32 || y >= side as i32 {
                continue;
            }
            let kk = y as usize * side + x as usize;
            if !blocked[kk] && !open[kk] {
                open[kk] = true;
                stack.push(kk);
            }
        }
    }
    // A drilled or moulded hole is round: its clear part is not much larger than its inscribed
    // disc. An elongated dropout (a laser line's shadow, a glare streak) is a data gap.
    let area = open.iter().filter(|&&o| o).count() as f64;
    let disc = std::f64::consts::PI * (depth as f64).powi(2);
    if area > ROUND_AREA_RATIO * disc {
        return None;
    }
    Some(open)
}

/// Largest area of a bore's clear part, as a multiple of its inscribed disc.
const ROUND_AREA_RATIO: f64 = 2.0;

/// Whether both faces of the plate were scanned round a bore (`open`, from [`through_bore`]): in
/// the 3 mm ring round it, at least half of the sheet pixels carry real (measured or marker fill)
/// points of the sheet itself and have real opposite-facing surface 0.3–9 mm behind them. Only
/// then is a clear axis evidence of a hole through the part: where one face alone was scanned,
/// nothing real lies behind any gap in it (the completion builds the other face from the seen one,
/// gaps included, and seen from that completed face the scanned one is behind), so a sticker gap,
/// a glare or dark-spot dropout looks exactly like a screw hole and is filled instead.
#[allow(clippy::too_many_arguments)]
fn both_faces_round(
    open: &[bool],
    occ: &[bool],
    sheet_real: &[(f64, f64)],
    opposite_real: &[(f64, f64, f64)],
    q: &Quadric,
    side: usize,
    half: i32,
) -> bool {
    let mark = |mask: &mut [bool], u: f64, v: f64| {
        let (iu, iv) = ((u / PX).round() as i32 + half, (v / PX).round() as i32 + half);
        for dv in -1..=1 {
            for du in -1..=1 {
                let (a, b) = (iu + du, iv + dv);
                if a >= 0 && b >= 0 && a < side as i32 && b < side as i32 {
                    mask[b as usize * side + a as usize] = true;
                }
            }
        }
    };
    let mut sheet = vec![false; side * side];
    for &(u, v) in sheet_real {
        mark(&mut sheet, u, v);
    }
    let mut behind = vec![false; side * side];
    for &(u, v, z) in opposite_real {
        if (0.3..9.0).contains(&(q.eval(u, v) - z)) {
            mark(&mut behind, u, v);
        }
    }
    let dt = chamfer_dt(open, side);
    let (mut ring, mut hit) = (0usize, 0usize);
    for k in 0..side * side {
        let d = dt[k] as f64 * PX;
        if d > 0.0 && d <= 3.0 && occ[k] {
            ring += 1;
            if sheet[k] && behind[k] {
                hit += 1;
            }
        }
    }
    ring > 0 && 2 * hit >= ring
}

/// Per pixel, the shortest span between occupied pixels on both sides of it along one of four
/// directions (0°, 45°, 90°, 135°), in pixels; infinite where no direction has sheet on both
/// sides. One sweep each way per direction.
fn min_span(occ: &[bool], side: usize) -> Vec<f32> {
    let inf = f32::INFINITY;
    let s = side as i32;
    let idx = |a: i32, b: i32| (b * s + a) as usize;
    let mut best = vec![inf; side * side];
    // Raster order: rows ascending for directions with db ≥ 0, descending for db < 0, so the
    // predecessor (a − da, b − db) is always visited first.
    for (da, db) in [(1i32, 0i32), (0, 1), (1, 1), (1, -1)] {
        let step = ((da * da + db * db) as f32).sqrt();
        let order: Vec<(i32, i32)> = if db >= 0 {
            (0..s).flat_map(|b| (0..s).map(move |a| (a, b))).collect()
        } else {
            (0..s).rev().flat_map(|b| (0..s).map(move |a| (a, b))).collect()
        };
        let mut back = vec![inf; side * side];
        let mut ahead = vec![inf; side * side];
        for &(a, b) in &order {
            let k = idx(a, b);
            if occ[k] {
                back[k] = 0.0;
            } else {
                let (pa, pb) = (a - da, b - db);
                if pa >= 0 && pb >= 0 && pa < s && pb < s {
                    back[k] = back[idx(pa, pb)] + step;
                }
            }
        }
        for &(a, b) in order.iter().rev() {
            let k = idx(a, b);
            if occ[k] {
                ahead[k] = 0.0;
            } else {
                let (na, nb) = (a + da, b + db);
                if na >= 0 && nb >= 0 && na < s && nb < s {
                    ahead[k] = ahead[idx(na, nb)] + step;
                }
            }
        }
        for k in 0..side * side {
            best[k] = best[k].min(back[k] + ahead[k]);
        }
    }
    best
}

/// Laplace interpolation of the residual heights over the hole pixels `pix` (plus value-less
/// pixels of the 3 mm ring), with the pixels that carry a mean residual as boundary values.
/// Successive over-relaxation; returns the residual at each pixel of `pix`.
fn harmonic_fill(pix: &[usize], dt_hole: &[f32], rsum: &[f64], rcnt: &[u32], side: usize) -> Vec<f64> {
    let n = side * side;
    // Unknown index per pixel.
    let mut uid = vec![u32::MAX; n];
    let mut unknown: Vec<usize> = Vec::new();
    for &k in pix {
        uid[k] = unknown.len() as u32;
        unknown.push(k);
    }
    for k in 0..n {
        if uid[k] == u32::MAX && rcnt[k] == 0 && (dt_hole[k] as f64) * PX <= 3.0 && dt_hole[k] > 0.0 {
            uid[k] = unknown.len() as u32;
            unknown.push(k);
        }
    }
    let known = |k: usize| (rcnt[k] > 0).then(|| rsum[k] / rcnt[k] as f64);
    // Start from the mean of the boundary values.
    let (mut bs, mut bc) = (0f64, 0usize);
    for &k in &unknown {
        for kk in neighbours(k, side) {
            if let Some(v) = known(kk) {
                bs += v;
                bc += 1;
            }
        }
    }
    let start = if bc > 0 { bs / bc as f64 } else { 0.0 };
    let mut x = vec![start; unknown.len()];
    let omega = 1.9;
    for _ in 0..800 {
        let mut change = 0f64;
        for (i, &k) in unknown.iter().enumerate() {
            let (mut s, mut c) = (0f64, 0usize);
            for kk in neighbours(k, side) {
                if uid[kk] != u32::MAX {
                    s += x[uid[kk] as usize];
                    c += 1;
                } else if let Some(v) = known(kk) {
                    s += v;
                    c += 1;
                }
            }
            if c > 0 {
                let new = x[i] + omega * (s / c as f64 - x[i]);
                change = change.max((new - x[i]).abs());
                x[i] = new;
            }
        }
        if change < 1e-5 {
            break;
        }
    }
    x.truncate(pix.len());
    x
}

fn neighbours(k: usize, side: usize) -> impl Iterator<Item = usize> {
    let (a, b) = ((k % side) as i64, (k / side) as i64);
    let s = side as i64;
    [(1i64, 0i64), (-1, 0), (0, 1), (0, -1)]
        .into_iter()
        .filter(move |(da, db)| a + da >= 0 && b + db >= 0 && a + da < s && b + db < s)
        .map(move |(da, db)| ((b + db) * s + a + da) as usize)
}
