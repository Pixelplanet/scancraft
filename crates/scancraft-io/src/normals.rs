//! Vertex normals for imported scans: from mesh faces, or estimated for bare point clouds.
//!
//! Estimation fits a plane (PCA) to the `k` nearest neighbours of every point, found with a
//! uniform hash grid (no KD-tree: exact coordinate ties, common in grid-quantised scanner
//! output, are harmless here). PCA gives the normal's line but not its sign; the sign is chosen
//! as follows:
//!
//! - with a viewpoint (for example the scanner position of a single view): towards it;
//! - otherwise outward from the cloud's centroid, which is right for closed or convex-ish
//!   objects. Points whose normal is nearly perpendicular to the centroid direction (a table
//!   plane, the side walls of a shell) are ambiguous; they take the sign that agrees with their
//!   unambiguous neighbours (region growing); a group with no unambiguous neighbour gets one
//!   consistent sign, seeded from its least ambiguous point.
//!
//! The overall sign of an open, one-sided surface is still a guess: on the Revo scans of the
//! inside of a shell, centroid-outward points 80-85 % of the normals away from the scanner
//! (consistently, so one flip fixes it). Pass a viewpoint when the scanner side is known.

use std::collections::HashMap;
use std::hash::{BuildHasherDefault, Hasher};

/// Settings for normal estimation.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct NormalEstimation {
    /// Neighbours used for each plane fit (including the point itself).
    pub neighbours: usize,
    /// Orient normals towards this point instead of outward from the centroid.
    pub viewpoint: Option<[f32; 3]>,
}

impl Default for NormalEstimation {
    fn default() -> Self {
        Self { neighbours: 16, viewpoint: None }
    }
}

/// Below this |cos| between normal and centroid direction a normal's sign is ambiguous.
const AMBIGUOUS_COS: f32 = 0.3;

/// Most neighbour candidates taken from one grid cell for a fit with `k` neighbours. The cell
/// size gives about `k / 9` points per cell, so this only bites where points pile up (many
/// duplicates, e.g. invalid samples written as 0, 0, 0), which would otherwise make the
/// search quadratic in the size of the pile.
fn max_per_cell(k: usize) -> usize {
    (8 * k).max(64)
}

/// Area-weighted vertex normals from triangle faces. Vertices without a usable face get
/// `[0, 0, 0]`. Face indices must be in range.
pub fn vertex_normals_from_faces(points: &[[f32; 3]], faces: &[[u32; 3]]) -> Vec<[f32; 3]> {
    let mut acc = vec![[0.0f32; 3]; points.len()];
    for f in faces {
        let [a, b, c] = f.map(|i| points[i as usize]);
        // The un-normalised cross product is twice the face area, so this weights by area.
        let n = cross(sub(b, a), sub(c, a));
        for &i in f {
            let s = &mut acc[i as usize];
            s[0] += n[0];
            s[1] += n[1];
            s[2] += n[2];
        }
    }
    acc.iter().map(|&n| normalize_or_zero(n)).collect()
}

/// Estimates unit normals for a point cloud (see the module notes). Points with fewer than
/// three neighbours get the orientation direction itself as their normal.
pub fn estimate_normals(points: &[[f32; 3]], settings: &NormalEstimation) -> Vec<[f32; 3]> {
    // Exact duplicates are fitted once (through their first copy) and take its normal: a pile
    // of them (invalid samples written as one point) would skew the grid and the centroid.
    let first = first_copies(points);
    let unique: Vec<usize> = (0..points.len()).filter(|&i| first[i] == i as u32).collect();
    let mut normals = vec![[0.0f32; 3]; points.len()];
    if unique.is_empty() {
        return normals;
    }
    let k = settings.neighbours.max(3);
    let centroid = centroid(points, &unique);
    let grid = Grid::build(points, &unique, choose_cell(points, &unique, k));

    let ctx = FitContext { points, first: &first, grid: &grid, k, per_cell: max_per_cell(k), viewpoint: settings.viewpoint, centroid };
    let threads = std::thread::available_parallelism().map_or(1, |n| n.get()).clamp(1, 16);
    let chunk = points.len().div_ceil(threads).max(1);
    let mut ambiguous: Vec<Ambiguous> = Vec::new();
    std::thread::scope(|s| {
        let handles: Vec<_> = normals
            .chunks_mut(chunk)
            .enumerate()
            .map(|(t, out)| {
                let ctx = &ctx;
                s.spawn(move || ctx.fit_range(t * chunk, out))
            })
            .collect();
        for h in handles {
            ambiguous.extend(h.join().expect("normal estimation thread panicked"));
        }
    });
    if settings.viewpoint.is_none() {
        propagate_signs(&mut normals, ambiguous);
    }
    for (i, &f) in first.iter().enumerate() {
        if f != NOT_FINITE && f as usize != i {
            normals[i] = normals[f as usize];
        }
    }
    normals
}

/// [`first_copies`] entry of a point with a non-finite coordinate.
const NOT_FINITE: u32 = u32::MAX;

/// Per point: the index of the first point at exactly the same position (itself for the first
/// copy), or [`NOT_FINITE`].
fn first_copies(points: &[[f32; 3]]) -> Vec<u32> {
    let mut seen: HashMap<[u32; 3], u32, BuildHasherDefault<FxHasher>> = HashMap::default();
    points
        .iter()
        .enumerate()
        .map(|(i, p)| {
            if !is_finite(*p) {
                return NOT_FINITE;
            }
            // +0.0 and -0.0 are the same position.
            let key = p.map(|v| if v == 0.0 { 0 } else { v.to_bits() });
            *seen.entry(key).or_insert(i as u32)
        })
        .collect()
}

/// A point whose normal sign could not be decided from the centroid direction.
struct Ambiguous {
    index: u32,
    cos_abs: f32,
    neighbours: Vec<u32>,
}

/// Shared read-only state for the per-point plane fits.
struct FitContext<'a> {
    points: &'a [[f32; 3]],
    /// [`first_copies`] of `points`: only first copies are fitted.
    first: &'a [u32],
    grid: &'a Grid,
    k: usize,
    /// Most candidates taken from one grid cell ([`max_per_cell`]).
    per_cell: usize,
    viewpoint: Option<[f32; 3]>,
    centroid: [f32; 3],
}

impl FitContext<'_> {
    /// Fits normals for points `start..start + out.len()`; returns the ambiguous points
    /// (centroid orientation only) with their neighbour lists.
    fn fit_range(&self, start: usize, out: &mut [[f32; 3]]) -> Vec<Ambiguous> {
        let mut candidates: Vec<(f32, u32)> = Vec::new();
        let mut ambiguous = Vec::new();
        for (o, normal) in out.iter_mut().enumerate() {
            let i = start + o;
            let p = self.points[i];
            if self.first[i] != i as u32 {
                continue;
            }
            candidates.clear();
            self.grid.for_each_near(p, self.per_cell, |j| {
                candidates.push((dist2(p, self.points[j as usize]), j));
            });
            if candidates.len() > self.k {
                candidates.select_nth_unstable_by(self.k - 1, |a, b| a.0.total_cmp(&b.0));
                candidates.truncate(self.k);
            }
            let towards = match self.viewpoint {
                Some(v) => sub(v, p),
                None => sub(p, self.centroid),
            };
            let towards = normalize_or_zero(towards);
            let mut n = if candidates.len() >= 3 {
                plane_normal(self.points, candidates.iter().map(|c| c.1))
            } else {
                [0.0; 3]
            };
            if n == [0.0; 3] {
                n = if towards == [0.0; 3] { [0.0, 0.0, 1.0] } else { towards };
            }
            let cos = dot(n, towards);
            if cos < 0.0 {
                n = [-n[0], -n[1], -n[2]];
            }
            *normal = n;
            if self.viewpoint.is_none() && cos.abs() < AMBIGUOUS_COS && candidates.len() > 1 {
                let neighbours = candidates.iter().map(|c| c.1).filter(|&j| j as usize != i).collect();
                ambiguous.push(Ambiguous { index: i as u32, cos_abs: cos.abs(), neighbours });
            }
        }
        ambiguous
    }
}

/// Gives ambiguous normals the sign of their settled neighbours by region growing (breadth
/// first over the neighbour lists). A group with no settled neighbour at all is seeded from its
/// least ambiguous point, so each connected group ends up with one consistent sign.
fn propagate_signs(normals: &mut [[f32; 3]], mut pending: Vec<Ambiguous>) {
    const NONE: u32 = u32::MAX;
    let mut slot = vec![NONE; normals.len()];
    for (s, a) in pending.iter().enumerate() {
        slot[a.index as usize] = s as u32;
    }
    let settled = |slot: &[u32], j: u32| slot[j as usize] == NONE;
    let mut queue: std::collections::VecDeque<u32> = std::collections::VecDeque::new();
    // Start from every pending point that touches a settled one.
    for (s, a) in pending.iter().enumerate() {
        if a.neighbours.iter().any(|&j| settled(&slot, j)) {
            queue.push_back(s as u32);
        }
    }
    let mut seeds: Vec<u32> = (0..pending.len() as u32).collect();
    seeds.sort_by(|&a, &b| pending[b as usize].cos_abs.total_cmp(&pending[a as usize].cos_abs));
    let mut next_seed = 0;
    let mut done = vec![false; pending.len()];
    while let Some(s) = queue.pop_front().or_else(|| loop {
        while next_seed < seeds.len() && done[seeds[next_seed] as usize] {
            next_seed += 1;
        }
        let seed = *seeds.get(next_seed)?;
        // Seed: keep the sign it has; settle it and grow from it.
        done[seed as usize] = true;
        slot[pending[seed as usize].index as usize] = NONE;
        for &j in &pending[seed as usize].neighbours {
            if !settled(&slot, j) {
                queue.push_back(slot[j as usize]);
            }
        }
        if let Some(next) = queue.pop_front() {
            return Some(next);
        }
    }) {
        if done[s as usize] {
            continue;
        }
        let entry = &mut pending[s as usize];
        let i = entry.index as usize;
        let mut n = normals[i];
        let vote: f32 = entry.neighbours.iter().filter(|&&j| settled(&slot, j)).map(|&j| dot(n, normals[j as usize])).sum();
        if entry.neighbours.iter().all(|&j| !settled(&slot, j)) {
            continue; // reached from a neighbour whose own list does not include it; wait
        }
        if vote < 0.0 {
            n = [-n[0], -n[1], -n[2]];
            normals[i] = n;
        }
        done[s as usize] = true;
        slot[i] = NONE;
        let neighbours = std::mem::take(&mut entry.neighbours);
        for &j in &neighbours {
            if !settled(&slot, j) {
                queue.push_back(slot[j as usize]);
            }
        }
    }
}

/// Unit normal of the best-fit plane through the given points (smallest-eigenvalue
/// eigenvector of their covariance), or zero for a degenerate set.
fn plane_normal(points: &[[f32; 3]], idx: impl Iterator<Item = u32> + Clone) -> [f32; 3] {
    let mut mean = [0.0f64; 3];
    let mut n = 0.0f64;
    for i in idx.clone() {
        let p = points[i as usize];
        for a in 0..3 {
            mean[a] += p[a] as f64;
        }
        n += 1.0;
    }
    mean = mean.map(|m| m / n);
    let mut c = [[0.0f64; 3]; 3];
    for i in idx {
        let p = points[i as usize];
        let d = [p[0] as f64 - mean[0], p[1] as f64 - mean[1], p[2] as f64 - mean[2]];
        for r in 0..3 {
            for s in r..3 {
                c[r][s] += d[r] * d[s];
            }
        }
    }
    c[1][0] = c[0][1];
    c[2][0] = c[0][2];
    c[2][1] = c[1][2];
    let trace = c[0][0] + c[1][1] + c[2][2];
    if trace.is_nan() || trace <= 1e-18 {
        return [0.0; 3];
    }
    let (values, vectors) = symmetric_eigen(c);
    let min = (0..3).min_by(|&a, &b| values[a].total_cmp(&values[b])).unwrap_or(0);
    // A line (two vanishing eigenvalues) has no defined plane.
    let mut sorted = values;
    sorted.sort_by(f64::total_cmp);
    if sorted[1] <= 1e-9 * trace {
        return [0.0; 3];
    }
    normalize_or_zero([vectors[0][min] as f32, vectors[1][min] as f32, vectors[2][min] as f32])
}

/// Eigen decomposition of a symmetric 3x3 matrix by cyclic Jacobi rotations. Returns the
/// eigenvalues and the eigenvectors as matrix columns.
fn symmetric_eigen(mut a: [[f64; 3]; 3]) -> ([f64; 3], [[f64; 3]; 3]) {
    let mut v = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    for _ in 0..32 {
        let off = a[0][1].abs() + a[0][2].abs() + a[1][2].abs();
        let scale = a[0][0].abs() + a[1][1].abs() + a[2][2].abs();
        if off <= 1e-15 * scale.max(1e-300) {
            break;
        }
        for (p, q) in [(0, 1), (0, 2), (1, 2)] {
            if a[p][q].abs() < 1e-300 {
                continue;
            }
            let theta = (a[q][q] - a[p][p]) / (2.0 * a[p][q]);
            let t = theta.signum() / (theta.abs() + (theta * theta + 1.0).sqrt());
            let t = if theta == 0.0 { 1.0 } else { t };
            let c = 1.0 / (t * t + 1.0).sqrt();
            let s = t * c;
            // A' = J^T A J with the rotation J in the (p, q) plane.
            for row in a.iter_mut() {
                let (akp, akq) = (row[p], row[q]);
                row[p] = c * akp - s * akq;
                row[q] = s * akp + c * akq;
            }
            let (rp, rq) = (a[p], a[q]);
            a[p] = std::array::from_fn(|k| c * rp[k] - s * rq[k]);
            a[q] = std::array::from_fn(|k| s * rp[k] + c * rq[k]);
            for row in v.iter_mut() {
                let vkp = row[p];
                let vkq = row[q];
                row[p] = c * vkp - s * vkq;
                row[q] = s * vkp + c * vkq;
            }
        }
    }
    ([a[0][0], a[1][1], a[2][2]], v)
}

/// Picks a grid cell edge so that the 27 cells around a point hold about `3k` points.
///
/// Calibrated on a subsample, assuming the points lie on surfaces (count grows with the
/// square of the cell size).
fn choose_cell(points: &[[f32; 3]], finite: &[usize], k: usize) -> f32 {
    let (min, max) = bounds(points, finite);
    let extent = sub(max, min);
    let diag = dot(extent, extent).sqrt().max(1e-6);
    let max_extent = extent[0].max(extent[1]).max(extent[2]).max(1e-6);
    let min_cell = max_extent / (Grid::MAX_CELLS - 4) as f32;

    let stride = (finite.len() / 50_000).max(1);
    let sample: Vec<usize> = finite.iter().copied().step_by(stride).collect();
    let target = (3 * k) as f32 / stride as f32;
    let queries: Vec<usize> = sample.iter().copied().step_by((sample.len() / 2_000).max(1)).collect();

    let mut cell = (diag / (finite.len() as f32).sqrt() * (k as f32).sqrt()).max(min_cell);
    for _ in 0..6 {
        let grid = Grid::build(points, &sample, cell);
        let mut counts: Vec<usize> = queries
            .iter()
            .map(|&q| {
                let mut c = 0;
                grid.for_each_near(points[q], usize::MAX, |_| c += 1);
                c
            })
            .collect();
        let mid = counts.len() / 2;
        let median = *counts.select_nth_unstable(mid).1 as f32;
        let ratio = (target / median.max(1.0)).sqrt().clamp(0.25, 4.0);
        if (0.8..=1.25).contains(&ratio) {
            break;
        }
        cell = (cell * ratio).max(min_cell);
    }
    cell
}

/// Uniform hash grid over point indices.
struct Grid {
    origin: [f32; 3],
    inv_cell: f32,
    /// Point indices sorted by cell.
    order: Vec<u32>,
    /// Cell key -> range in `order`.
    cells: HashMap<u64, (u32, u32), BuildHasherDefault<FxHasher>>,
}

impl Grid {
    /// Cells per axis that fit the 21-bit key fields.
    const MAX_CELLS: u32 = 1 << 21;

    fn build(points: &[[f32; 3]], idx: &[usize], cell: f32) -> Self {
        let (min, _) = bounds(points, idx);
        let inv_cell = 1.0 / cell;
        let mut keyed: Vec<(u64, u32)> = Vec::with_capacity(idx.len());
        let mut grid = Self { origin: min, inv_cell, order: Vec::new(), cells: HashMap::default() };
        for &i in idx {
            keyed.push((grid.key(grid.cell_of(points[i])), i as u32));
        }
        keyed.sort_unstable_by_key(|e| e.0);
        grid.cells.reserve(keyed.len() / 4);
        let mut start = 0usize;
        while start < keyed.len() {
            let key = keyed[start].0;
            let mut end = start + 1;
            while end < keyed.len() && keyed[end].0 == key {
                end += 1;
            }
            grid.cells.insert(key, (start as u32, end as u32));
            start = end;
        }
        grid.order = keyed.into_iter().map(|e| e.1).collect();
        grid
    }

    fn cell_of(&self, p: [f32; 3]) -> [i64; 3] {
        [0, 1, 2].map(|a| ((p[a] - self.origin[a]) * self.inv_cell).floor() as i64)
    }

    /// Key with a +1 offset per axis so that the neighbours of cell 0 are representable.
    fn key(&self, c: [i64; 3]) -> u64 {
        let f = |v: i64| (v + 1).clamp(0, Self::MAX_CELLS as i64 - 1) as u64;
        (f(c[0]) << 42) | (f(c[1]) << 21) | f(c[2])
    }

    /// Calls `f` for every point in the 27 cells around `p`, at most `per_cell` from each cell.
    fn for_each_near(&self, p: [f32; 3], per_cell: usize, mut f: impl FnMut(u32)) {
        let c = self.cell_of(p);
        for dx in -1..=1 {
            for dy in -1..=1 {
                for dz in -1..=1 {
                    let n = [c[0] + dx, c[1] + dy, c[2] + dz];
                    if n.iter().any(|&v| v < -1 || v + 1 >= Self::MAX_CELLS as i64) {
                        continue;
                    }
                    if let Some(&(s, e)) = self.cells.get(&self.key(n)) {
                        let e = e.min(s.saturating_add(per_cell.min(u32::MAX as usize) as u32));
                        for &j in &self.order[s as usize..e as usize] {
                            f(j);
                        }
                    }
                }
            }
        }
    }
}

/// Small fast hasher for integer keys (the Fx hash used by rustc).
#[derive(Default, Clone, Copy)]
pub(crate) struct FxHasher {
    hash: u64,
}

impl Hasher for FxHasher {
    fn write(&mut self, bytes: &[u8]) {
        for &b in bytes {
            self.write_u64(b as u64);
        }
    }
    fn write_u32(&mut self, v: u32) {
        self.write_u64(v as u64);
    }
    fn write_u64(&mut self, v: u64) {
        self.hash = (self.hash.rotate_left(5) ^ v).wrapping_mul(0x51_7c_c1_b7_27_22_0a_95);
    }
    fn finish(&self) -> u64 {
        self.hash
    }
}

fn bounds(points: &[[f32; 3]], idx: &[usize]) -> ([f32; 3], [f32; 3]) {
    let mut min = [f32::INFINITY; 3];
    let mut max = [f32::NEG_INFINITY; 3];
    for &i in idx {
        for a in 0..3 {
            min[a] = min[a].min(points[i][a]);
            max[a] = max[a].max(points[i][a]);
        }
    }
    (min, max)
}

fn centroid(points: &[[f32; 3]], idx: &[usize]) -> [f32; 3] {
    let mut s = [0.0f64; 3];
    for &i in idx {
        for a in 0..3 {
            s[a] += points[i][a] as f64;
        }
    }
    s.map(|v| (v / idx.len() as f64) as f32)
}

pub(crate) fn is_finite(p: [f32; 3]) -> bool {
    p.iter().all(|v| v.is_finite())
}

pub(crate) fn sub(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

pub(crate) fn cross(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}

fn dot(a: [f32; 3], b: [f32; 3]) -> f32 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn dist2(a: [f32; 3], b: [f32; 3]) -> f32 {
    let d = sub(a, b);
    dot(d, d)
}

/// Unit vector, or zero when the input is (nearly) zero or not finite.
pub(crate) fn normalize_or_zero(n: [f32; 3]) -> [f32; 3] {
    let len = dot(n, n).sqrt();
    if len > 1e-20 && len.is_finite() {
        [n[0] / len, n[1] / len, n[2] / len]
    } else {
        [0.0; 3]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Points on a sphere (Fibonacci lattice).
    fn sphere(n: usize, r: f32, c: [f32; 3]) -> Vec<[f32; 3]> {
        let golden = std::f32::consts::PI * (3.0 - 5f32.sqrt());
        (0..n)
            .map(|i| {
                let y = 1.0 - 2.0 * (i as f32 + 0.5) / n as f32;
                let rad = (1.0 - y * y).sqrt();
                let t = golden * i as f32;
                [c[0] + r * rad * t.cos(), c[1] + r * y, c[2] + r * rad * t.sin()]
            })
            .collect()
    }

    #[test]
    fn jacobi_finds_smallest_axis() {
        let (values, vectors) = symmetric_eigen([[4.0, 1.0, 0.0], [1.0, 3.0, 0.0], [0.0, 0.0, 0.5]]);
        let min = (0..3).min_by(|&a, &b| values[a].total_cmp(&values[b])).unwrap();
        assert!((values[min] - 0.5).abs() < 1e-9);
        assert!((vectors[2][min].abs() - 1.0).abs() < 1e-9);
    }

    #[test]
    fn sphere_normals_point_outward() {
        let c = [10.0, -20.0, 300.0];
        let pts = sphere(20_000, 50.0, c);
        let normals = estimate_normals(&pts, &NormalEstimation::default());
        let mut worst = 1.0f32;
        for (p, n) in pts.iter().zip(&normals) {
            let radial = normalize_or_zero(sub(*p, c));
            worst = worst.min(dot(radial, *n));
        }
        assert!(worst > 0.95, "worst cos to radial direction {worst}");
    }

    #[test]
    fn plane_normals_follow_viewpoint_and_neighbours() {
        // A 200 x 200 mm grid-quantised plane with duplicated points (exact ties).
        let mut pts = Vec::new();
        for x in 0..100 {
            for y in 0..100 {
                let p = [x as f32 * 2.0, y as f32 * 2.0, 5.0];
                pts.push(p);
                if (x + y) % 7 == 0 {
                    pts.push(p);
                }
            }
        }
        let up = NormalEstimation { viewpoint: Some([100.0, 100.0, 500.0]), ..Default::default() };
        let normals = estimate_normals(&pts, &up);
        assert!(normals.iter().all(|n| n[2] > 0.999));

        // Centroid orientation: every normal is perpendicular to the centroid direction, so
        // the sign is a guess, but it must be consistent within each patch. A second,
        // disconnected patch of the same plane needs its own seed.
        let first = pts.len();
        let shifted: Vec<[f32; 3]> = pts.iter().map(|p| [p[0] + 1000.0, p[1], 5.0]).collect();
        pts.extend(shifted);
        let normals = estimate_normals(&pts, &NormalEstimation::default());
        for patch in [&normals[..first], &normals[first..]] {
            let positive = patch.iter().filter(|n| n[2] > 0.999).count();
            let negative = patch.iter().filter(|n| n[2] < -0.999).count();
            assert_eq!(positive + negative, patch.len());
            assert!(positive.min(negative) == 0, "{positive} up, {negative} down");
        }
    }

    #[test]
    fn a_pile_of_duplicates_stays_fast_and_leaves_the_surface_alone() {
        // A sphere, plus 60k invalid samples written as one point (a common placeholder), and
        // a smaller pile that is only nearly one point (at the centre, so the centroid stays).
        let c = [0.0, 0.0, 300.0];
        let mut pts = sphere(20_000, 50.0, c);
        pts.extend(std::iter::repeat_n([0.0f32, 0.0, 0.0], 60_000));
        pts.extend((0..5_000).map(|i| [i as f32 * 4e-4, 0.0, 300.0]));
        let t0 = std::time::Instant::now();
        let normals = estimate_normals(&pts, &NormalEstimation::default());
        let seconds = t0.elapsed().as_secs_f32();
        // Without deduplication and the per-cell cap, every pile point scans the whole pile.
        assert!(seconds < 20.0, "{seconds:.1} s");
        let worst = pts[..20_000].iter().zip(&normals).map(|(p, n)| dot(normalize_or_zero(sub(*p, c)), *n)).fold(1.0f32, f32::min);
        assert!(worst > 0.95, "worst cos to radial direction {worst}");
        assert!(normals[20_000..].iter().all(|n| n.iter().all(|v| v.is_finite())));
    }

    #[test]
    fn face_normals_are_area_weighted_and_unit() {
        let pts = [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [5.0, 5.0, 5.0]];
        let normals = vertex_normals_from_faces(&pts, &[[0, 1, 2]]);
        assert_eq!(normals[0], [0.0, 0.0, 1.0]);
        assert_eq!(normals[3], [0.0, 0.0, 0.0]);
    }

    #[test]
    fn tiny_and_degenerate_clouds_do_not_panic() {
        let s = NormalEstimation::default();
        assert!(estimate_normals(&[], &s).is_empty());
        let one = estimate_normals(&[[1.0, 2.0, 3.0]], &s);
        assert_eq!(one.len(), 1);
        let line: Vec<[f32; 3]> = (0..50).map(|i| [i as f32, 0.0, 0.0]).collect();
        let n = estimate_normals(&line, &s);
        assert!(n.iter().all(|v| v.iter().all(|c| c.is_finite())));
        let nan = estimate_normals(&[[f32::NAN, 0.0, 0.0], [0.0, 0.0, 0.0]], &s);
        assert_eq!(nan[0], [0.0; 3]);
    }
}
