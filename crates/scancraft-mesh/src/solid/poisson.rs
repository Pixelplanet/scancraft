//! Screened Poisson indicator solve on a sparse band of 8³ node blocks.
//!
//! Unknown: an indicator χ at the grid nodes (1 in the material, 0 outside). Energy
//!
//! ```text
//!   E(χ) = Σ_edges h (χ_b − χ_a − h V_e)²  +  (σ / h²) Σ_i w_i a_i (χ(p_i) − ½)²
//! ```
//!
//! * `V` is the inward normal field: every sample splats `−a_i n_i` onto the edge midpoints of a
//!   staggered grid with trilinear weights (`a_i` its surface area), so the discrete divergence
//!   of V is exact on the node lattice.
//! * The second term (screening, Kazhdan & Hoppe 2013) pins χ = ½ at the samples, `χ(p)` being
//!   the trilinear interpolation of the 8 nodes around `p`; `σ` is dimensionless.
//! * Nodes outside the band (blocks farther than `ring` blocks from any sample) are Dirichlet
//!   χ = 0, the envelope constraint of Kazhdan et al. 2020. Every grid node is therefore known and
//!   the iso-surface is closed.
//!
//! The normal equations `(L + S) χ = div + S·½` (L the 7-point graph Laplacian) are SPD and
//! solved with Jacobi-preconditioned conjugate gradients, matrix-free; a solve on a grid twice
//! as coarse provides the starting guess.

use super::grid::{cell_key, FxMap};
use super::WorkPoint;
use glam::{IVec3, Vec3};
use rayon::prelude::*;
use std::collections::HashSet;

pub const B: i32 = 8;
pub const BV: usize = 512;

#[inline]
pub fn local_index(x: i32, y: i32, z: i32) -> usize {
    ((z * B + y) * B + x) as usize
}

/// Sparse set of 8³ node blocks.
pub struct BandGrid {
    pub h: f32,
    pub origin: Vec3,
    pub blocks: Vec<IVec3>,
    pub lookup: FxMap<u64, u32>,
    /// For every block, the indices of its 27 neighbours (offset (dx,dy,dz) at 13+dx+3dy+9dz),
    /// `u32::MAX` where absent.
    pub nbr: Vec<[u32; 27]>,
}

impl BandGrid {
    pub fn build(points: &[Vec3], h: f32, ring: i32) -> Self {
        let ring = ring.max(1);
        let (mn, _mx) = points.iter().fold((Vec3::splat(f32::MAX), Vec3::splat(f32::MIN)), |(a, b), p| (a.min(*p), b.max(*p)));
        let margin = ((ring + 2) * B) as f32 * h;
        let origin = ((mn - Vec3::splat(margin)) / (B as f32 * h)).floor() * (B as f32 * h);
        let mut set: HashSet<IVec3> = points
            .par_iter()
            .map(|p| ((*p - origin) / h).floor().as_ivec3() >> 3)
            .collect::<Vec<_>>()
            .into_iter()
            .collect();
        for _ in 0..ring {
            let cur: Vec<IVec3> = set.iter().copied().collect();
            for b in cur {
                for i in 0..27 {
                    set.insert(b + IVec3::new(i % 3 - 1, (i / 3) % 3 - 1, i / 9 - 1));
                }
            }
        }
        let mut blocks: Vec<IVec3> = set.into_iter().collect();
        blocks.sort_unstable_by_key(|b| cell_key(*b));
        let lookup: FxMap<u64, u32> = blocks.iter().enumerate().map(|(i, b)| (cell_key(*b), i as u32)).collect();
        let nbr = blocks
            .par_iter()
            .map(|b| {
                let mut t = [u32::MAX; 27];
                for (i, e) in t.iter_mut().enumerate() {
                    let i = i as i32;
                    let o = IVec3::new(i % 3 - 1, (i / 3) % 3 - 1, i / 9 - 1);
                    if let Some(&j) = lookup.get(&cell_key(*b + o)) {
                        *e = j;
                    }
                }
                t
            })
            .collect();
        Self { h, origin, blocks, lookup, nbr }
    }

    pub fn len(&self) -> usize {
        self.blocks.len() * BV
    }

    /// Index of the node at global grid coordinate `g`, if it is in the band.
    #[inline]
    pub fn node(&self, g: IVec3) -> Option<usize> {
        let b = g >> 3;
        let l = g & 7;
        self.lookup.get(&cell_key(b)).map(|&bi| bi as usize * BV + local_index(l.x, l.y, l.z))
    }

    #[inline]
    pub fn grid_pos(&self, p: Vec3) -> Vec3 {
        (p - self.origin) / self.h
    }

    /// Trilinear interpolation of a node field (nodes outside the band are 0).
    pub fn sample(&self, field: &[f32], p: Vec3) -> f32 {
        let q = self.grid_pos(p);
        let b = q.floor();
        let f = q - b;
        let bi = b.as_ivec3();
        let mut s = 0.0;
        for o in 0..8 {
            let off = IVec3::new(o & 1, (o >> 1) & 1, (o >> 2) & 1);
            let w = (if off.x == 1 { f.x } else { 1.0 - f.x })
                * (if off.y == 1 { f.y } else { 1.0 - f.y })
                * (if off.z == 1 { f.z } else { 1.0 - f.z });
            if let Some(j) = self.node(bi + off) {
                s += w * field[j];
            }
        }
        s
    }
}

#[derive(Clone, Debug)]
pub struct PoissonParams {
    pub cell_mm: f32,
    /// Band: blocks within this many blocks of a sample are solved, the rest is χ = 0.
    pub ring: i32,
    /// Dimensionless screening weight σ.
    pub screen: f32,
    pub tol: f64,
    pub max_iter: usize,
    /// Solve on a 2× coarser grid first and start from its (interpolated) solution.
    pub coarse_start: bool,
    /// Passes of a [¼ ½ ¼] filter per axis applied to the divergence, per sample class
    /// (`class` 0 = sharp, 1 = measured, 2 = soft); each pass adds ½ cell² of variance to the
    /// normal kernel. Applied as a cascade, so the counts must not increase from soft to sharp:
    /// `smooth[2] ≥ smooth[1] ≥ smooth[0]`. A fractional part f is one [f/4, 1 − f/2, f/4] pass
    /// (variance f/2 cell²).
    pub smooth: [f32; 3],
}

pub struct PoissonOut {
    pub grid: BandGrid,
    pub chi: Vec<f32>,
    pub iterations: usize,
    pub rel_residual: f64,
    pub coarse_iterations: usize,
}

/// Point stencil for screening: 8 node indices + trilinear weights, and the coefficient.
struct Screen {
    idx: Vec<[u32; 8]>,
    wts: Vec<[f32; 8]>,
    coef: Vec<f32>,
    /// (node, point*8+k) sorted by node; `block_runs[b]..block_runs[b+1]` are block b's entries.
    entries: Vec<(u32, u32)>,
    block_runs: Vec<u32>,
}

/// `pts`: samples with area set; `conf`: screening confidence per sample; `class`: smoothing
/// class per sample (see [`PoissonParams::smooth`]).
pub fn solve(pts: &[WorkPoint], conf: &[f32], class: &[u8], prm: &PoissonParams) -> PoissonOut {
    let pos: Vec<Vec3> = pts.iter().map(|p| p.p).collect();
    let mut coarse_iterations = 0;
    let start = if prm.coarse_start {
        let cp = PoissonParams { cell_mm: prm.cell_mm * 2.0, coarse_start: false, max_iter: prm.max_iter, tol: prm.tol, ..*prm };
        let c = solve(pts, conf, class, &cp);
        coarse_iterations = c.iterations;
        Some(c)
    } else {
        None
    };

    let grid = BandGrid::build(&pos, prm.cell_mm, prm.ring);
    let n = grid.len();
    let h = prm.cell_mm;

    // ---- right-hand side: divergence of the staggered inward normal field -----------------
    let mut rhs_class = [vec![0f32; n], Vec::new(), Vec::new()];
    for c in 1..3 {
        if class.iter().any(|&k| k as usize == c) {
            rhs_class[c] = vec![0f32; n];
        }
    }
    for (p, &k) in pts.iter().zip(class) {
        let rhs = &mut rhs_class[(k as usize).min(2)];
        let q = grid.grid_pos(p.p);
        let c = -p.n * (p.area / (h * h));
        for axis in 0..3 {
            let mut qa = q;
            qa[axis] -= 0.5;
            let b = qa.floor();
            let f = qa - b;
            let bi = b.as_ivec3();
            let mut e = IVec3::ZERO;
            e[axis] = 1;
            for o in 0..8 {
                let off = IVec3::new(o & 1, (o >> 1) & 1, (o >> 2) & 1);
                let w = (if off.x == 1 { f.x } else { 1.0 - f.x })
                    * (if off.y == 1 { f.y } else { 1.0 - f.y })
                    * (if off.z == 1 { f.z } else { 1.0 - f.z });
                let hv = w * c[axis];
                let lo = bi + off;
                if let Some(j) = grid.node(lo) {
                    rhs[j] -= hv;
                }
                if let Some(j) = grid.node(lo + e) {
                    rhs[j] += hv;
                }
            }
        }
    }

    // Smoothing the divergence equals smoothing V (convolution commutes with the difference
    // operator): a wider normal kernel averages overlapping passes and removes the terracing a
    // one-cell kernel leaves in the marching cubes surface, but also rounds edges, so measured
    // samples get fewer passes than synthetic ones.
    let [rhs, rhs1, rhs2] = rhs_class;
    let mut rhs = rhs;
    let mut acc: Vec<f32> = Vec::new();
    for (c, part) in [(2usize, rhs2), (1, rhs1)] {
        if !part.is_empty() {
            if acc.is_empty() {
                acc = part;
            } else {
                acc.par_iter_mut().zip(&part).for_each(|(a, b)| *a += b);
            }
        }
        if !acc.is_empty() {
            acc = smooth_passes(&grid, acc, prm.smooth[c] - prm.smooth[c - 1]);
        }
    }
    if !acc.is_empty() {
        rhs.par_iter_mut().zip(&acc).for_each(|(a, b)| *a += b);
    }
    drop(acc);
    let mut rhs = smooth_passes(&grid, rhs, prm.smooth[0]);

    // ---- screening stencil ------------------------------------------------------------------
    let s_scale = prm.screen / (h * h);
    let stencil: Vec<([u32; 8], [f32; 8])> = pos
        .par_iter()
        .map(|p| {
            let q = grid.grid_pos(*p);
            let b = q.floor();
            let f = q - b;
            let bi = b.as_ivec3();
            let mut idx = [u32::MAX; 8];
            let mut w = [0f32; 8];
            for o in 0..8 {
                let off = IVec3::new(o & 1, (o >> 1) & 1, (o >> 2) & 1);
                w[o as usize] = (if off.x == 1 { f.x } else { 1.0 - f.x })
                    * (if off.y == 1 { f.y } else { 1.0 - f.y })
                    * (if off.z == 1 { f.z } else { 1.0 - f.z });
                idx[o as usize] = grid.node(bi + off).map_or(u32::MAX, |j| j as u32);
            }
            (idx, w)
        })
        .collect();
    let (idx, wts): (Vec<[u32; 8]>, Vec<[f32; 8]>) = stencil.into_iter().unzip();
    let coef: Vec<f32> = pts.iter().zip(conf).map(|(p, c)| s_scale * p.area * c).collect();
    let mut entries: Vec<(u32, u32)> = idx
        .par_iter()
        .enumerate()
        .flat_map_iter(|(i, ix)| {
            ix.iter().enumerate().filter(|(_, j)| **j != u32::MAX).map(move |(k, j)| (*j, (i * 8 + k) as u32))
        })
        .collect();
    entries.par_sort_unstable();
    let nb = grid.blocks.len();
    let block_runs: Vec<u32> =
        (0..=nb).map(|b| entries.partition_point(|e| (e.0 as usize) < b * BV) as u32).collect();
    let scr = Screen { idx, wts, coef, entries, block_runs };

    let mut diag = vec![6f32; n];
    for (i, ix) in scr.idx.iter().enumerate() {
        for k in 0..8 {
            if ix[k] != u32::MAX {
                let w = scr.wts[i][k];
                diag[ix[k] as usize] += scr.coef[i] * w * w;
                rhs[ix[k] as usize] += scr.coef[i] * w * 0.5;
            }
        }
    }

    // ---- starting guess -----------------------------------------------------------------------
    let mut x = vec![0f32; n];
    if let Some(c) = &start {
        x.par_chunks_mut(BV).enumerate().for_each(|(bi, xb)| {
            let base = grid.blocks[bi] * B;
            for z in 0..B {
                for y in 0..B {
                    for xx in 0..B {
                        let p = grid.origin + (base + IVec3::new(xx, y, z)).as_vec3() * h;
                        xb[local_index(xx, y, z)] = c.grid.sample(&c.chi, p);
                    }
                }
            }
        });
    }
    drop(start);

    let (iterations, rel_residual) = pcg(&grid, &scr, &diag, &rhs, &mut x, prm.tol, prm.max_iter);
    PoissonOut { grid, chi: x, iterations, rel_residual, coarse_iterations }
}

/// `passes` (may be fractional, ≤ 0 = none) [¼ ½ ¼] passes along every axis.
fn smooth_passes(grid: &BandGrid, mut x: Vec<f32>, passes: f32) -> Vec<f32> {
    if passes <= 0.0 {
        return x;
    }
    let full = passes.floor() as usize;
    let frac = passes - full as f32;
    for k in 0..full + usize::from(frac > 1e-3) {
        let a = if k < full { 0.25 } else { 0.25 * frac };
        for axis in 0..3 {
            x = binomial(grid, &x, axis, a);
        }
    }
    x
}

/// One pass of the [a, 1 − 2a, a] filter along `axis` (nodes outside the band are 0).
fn binomial(grid: &BandGrid, x: &[f32], axis: usize, a: f32) -> Vec<f32> {
    let mut y = vec![0f32; x.len()];
    let (km, kp, stride) = match axis {
        0 => (12usize, 14usize, 1usize),
        1 => (10, 16, 8),
        _ => (4, 22, 64),
    };
    y.par_chunks_mut(BV).enumerate().for_each(|(bi, yb)| {
        let xb = &x[bi * BV..(bi + 1) * BV];
        let nb = &grid.nbr[bi];
        let blk = |k: usize| (nb[k] != u32::MAX).then(|| &x[nb[k] as usize * BV..(nb[k] as usize + 1) * BV]);
        let (xm, xp) = (blk(km), blk(kp));
        for z in 0..B {
            for yy in 0..B {
                for xx in 0..B {
                    let l = local_index(xx, yy, z);
                    let c = [xx, yy, z][axis];
                    let lo = if c > 0 { xb[l - stride] } else { xm.map_or(0.0, |b| b[l + 7 * stride]) };
                    let hi = if c < B - 1 { xb[l + stride] } else { xp.map_or(0.0, |b| b[l - 7 * stride]) };
                    yb[l] = a * (lo + hi) + (1.0 - 2.0 * a) * xb[l];
                }
            }
        }
    });
    y
}

/// y = (L + S) x.
fn apply(grid: &BandGrid, scr: &Screen, x: &[f32], y: &mut [f32]) {
    let s: Vec<f32> = scr
        .idx
        .par_iter()
        .zip(&scr.wts)
        .zip(&scr.coef)
        .map(|((ix, w), c)| {
            let mut v = 0f32;
            for k in 0..8 {
                if ix[k] != u32::MAX {
                    v += w[k] * x[ix[k] as usize];
                }
            }
            v * c
        })
        .collect();
    y.par_chunks_mut(BV).enumerate().for_each(|(bi, yb)| {
        let xb = &x[bi * BV..(bi + 1) * BV];
        let nb = &grid.nbr[bi];
        let blk = |k: usize| (nb[k] != u32::MAX).then(|| &x[nb[k] as usize * BV..(nb[k] as usize + 1) * BV]);
        let (xm, xp, ym, yp, zm, zp) = (blk(12), blk(14), blk(10), blk(16), blk(4), blk(22));
        for z in 0..B {
            for yy in 0..B {
                for xx in 0..B {
                    let l = local_index(xx, yy, z);
                    let mut s = 0f32;
                    s += if xx > 0 { xb[l - 1] } else { xm.map_or(0.0, |b| b[local_index(B - 1, yy, z)]) };
                    s += if xx < B - 1 { xb[l + 1] } else { xp.map_or(0.0, |b| b[local_index(0, yy, z)]) };
                    s += if yy > 0 { xb[l - 8] } else { ym.map_or(0.0, |b| b[local_index(xx, B - 1, z)]) };
                    s += if yy < B - 1 { xb[l + 8] } else { yp.map_or(0.0, |b| b[local_index(xx, 0, z)]) };
                    s += if z > 0 { xb[l - 64] } else { zm.map_or(0.0, |b| b[local_index(xx, yy, B - 1)]) };
                    s += if z < B - 1 { xb[l + 64] } else { zp.map_or(0.0, |b| b[local_index(xx, yy, 0)]) };
                    yb[l] = 6.0 * xb[l] - s;
                }
            }
        }
        let base = bi * BV;
        for &(node, e) in &scr.entries[scr.block_runs[bi] as usize..scr.block_runs[bi + 1] as usize] {
            let (i, k) = ((e / 8) as usize, (e % 8) as usize);
            yb[node as usize - base] += s[i] * scr.wts[i][k];
        }
    });
}

fn dot(a: &[f32], b: &[f32]) -> f64 {
    a.par_chunks(4096).zip(b.par_chunks(4096)).map(|(x, y)| x.iter().zip(y).map(|(p, q)| (*p as f64) * (*q as f64)).sum::<f64>()).sum()
}

fn pcg(grid: &BandGrid, scr: &Screen, diag: &[f32], b: &[f32], x: &mut [f32], tol: f64, max_iter: usize) -> (usize, f64) {
    let n = b.len();
    let mut r = vec![0f32; n];
    apply(grid, scr, x, &mut r);
    r.par_iter_mut().zip(b).for_each(|(ri, bi)| *ri = bi - *ri);
    let bnorm = dot(b, b).sqrt().max(1e-30);
    let mut z: Vec<f32> = r.par_iter().zip(diag).map(|(ri, d)| ri / d).collect();
    let mut p = z.clone();
    let mut ap = vec![0f32; n];
    let mut rz = dot(&r, &z);
    let mut rel = dot(&r, &r).sqrt() / bnorm;
    let mut it = 0;
    while it < max_iter && rel > tol {
        apply(grid, scr, &p, &mut ap);
        let pap = dot(&p, &ap);
        if pap <= 0.0 {
            break;
        }
        let alpha = (rz / pap) as f32;
        x.par_iter_mut().zip(&p).for_each(|(xi, pi)| *xi += alpha * pi);
        r.par_iter_mut().zip(&ap).for_each(|(ri, a)| *ri -= alpha * a);
        it += 1;
        rel = dot(&r, &r).sqrt() / bnorm;
        if rel <= tol {
            break;
        }
        z.par_iter_mut().zip(&r).zip(diag).for_each(|((zi, ri), d)| *zi = ri / d);
        let rz_new = dot(&r, &z);
        let beta = (rz_new / rz) as f32;
        rz = rz_new;
        p.par_iter_mut().zip(&z).for_each(|(pi, zi)| *pi = zi + beta * *pi);
    }
    (it, rel)
}
