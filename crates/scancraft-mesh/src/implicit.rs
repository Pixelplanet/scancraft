//! Surface reconstruction with a robust implicit moving-least-squares surface (RIMLS, Öztireli,
//! Guennebaud and Gross 2009, "Feature preserving point set surfaces based on non-linear kernel
//! regression").
//!
//! Why not the plain tangent-plane field of [`crate::MarchingCubes`]: a merged scan carries
//! sensor noise, small residual misalignment between passes and noisy per-point normals. A
//! narrow plane-distance average turns all three into bumps, and marching cubes adds its own
//! faceting on top. Here:
//!
//! 1. Sparse layers of points stacked on a denser one (ink marks, glints) are dropped. Each
//!    point's normal is re-fitted from the positions around it and smoothed with its neighbours
//!    of similar orientation (bilateral in normal space), which removes normal noise without
//!    rounding edges; then single points far off the surface their neighbours define go.
//! 2. The field at `x` is the RIMLS potential over all points within the support radius `h`
//!    (a multiple of the point spacing, the "detail" setting), iterated with robust weights that
//!    discount points whose plane disagrees with the local fit (sharp edges, a stray pass) or whose
//!    normal disagrees with the field's gradient. Points facing the other way than the nearest
//!    point never take part, so the two sides of a thin wall stay apart.
//! 3. Corners far from every point along the surface stay unknown, so open scans give open
//!    patches (filled later on request by [`crate::holes`]) instead of invented sheets. Corners
//!    further than `h` from every point but beside the surface get a wider kernel: left
//!    unknown they would punch holes into steps and relief.
//! 4. Marching cubes on the known cells, welded by grid edge.
//! 5. Every vertex is projected onto the implicit surface along its gradient, which removes the
//!    cell-sized faceting of marching cubes; the vertex normal is the field gradient there.
//! 6. Tiny disconnected fragments are dropped and the open edges, which follow the voxel grid
//!    in stairs, are rounded.
//!
//! Normal convention as in [`crate::marching_cubes`]: normals face away from the interior.

use crate::marching_cubes::{filter_components, CORNERS, EDGES};
use crate::tables::{EDGE_TABLE, TRI_TABLE};
use glam::{IVec3, Vec3};
use rayon::prelude::*;
use scancraft_core::{ColorRgba, Mesh, Point3D, PointCloud, Triangle};
use std::collections::HashMap;

/// Settings of [`reconstruct`].
#[derive(Debug, Clone)]
pub struct SurfaceConfig {
    /// Marching-cubes cell (mm): the triangle size. Detail finer than the support radius is not
    /// there to resolve, so a cell much below `detail × spacing / 3` only adds triangles.
    pub cell_mm: f32,
    /// Support radius in point spacings. Larger is smoother (noise and pass misalignment are
    /// averaged over a wider area), smaller keeps finer detail. 2–5 is useful.
    pub detail: f32,
    /// Robust refits per field evaluation (0 = plain IMLS).
    pub robust_iterations: usize,
    /// Projection steps per output vertex.
    pub projection_steps: usize,
    /// Re-estimate the input normals from the point positions first (see `refit_normals`).
    pub refit_normals: bool,
    /// Normal smoothing passes over the input (bilateral in normal space).
    pub normal_passes: usize,
    /// Input points further off the fitted surface than this many noise deviations are dropped
    /// before meshing (0 keeps all).
    pub outlier_sigma: f32,
    /// Connected pieces with fewer triangles than this are discarded.
    pub min_component_triangles: usize,
    /// Connected pieces smaller than this fraction of the largest are discarded.
    pub min_component_fraction: f32,
}

impl Default for SurfaceConfig {
    fn default() -> Self {
        SurfaceConfig { cell_mm: 0.7, detail: 3.0, robust_iterations: 3, projection_steps: 2, refit_normals: true, normal_passes: 1, outlier_sigma: 4.0, min_component_triangles: 24, min_component_fraction: 0.002 }
    }
}

/// Facts about a reconstruction, for diagnostics.
#[derive(Debug, Clone, Default)]
pub struct SurfaceStats {
    pub input_points: usize,
    pub point_spacing_mm: f32,
    pub support_radius_mm: f32,
    pub evaluated_corners: usize,
    pub known_corners: usize,
    pub dropped_components: usize,
    pub removed_outliers: usize,
}

/// Oriented points sorted into a uniform grid of side `cell` for radius queries.
struct PointGrid {
    cell: f32,
    /// Sorted by cell: positions, normals, weights.
    p: Vec<Vec3>,
    n: Vec<Vec3>,
    w: Vec<f32>,
    /// Cell -> range into the arrays.
    ranges: HashMap<IVec3, (u32, u32)>,
}

impl PointGrid {
    fn new(points: Vec<(Vec3, Vec3, f32)>, cell: f32) -> Self {
        let mut keyed: Vec<(IVec3, (Vec3, Vec3, f32))> = points.into_par_iter().map(|e| ((e.0 / cell).floor().as_ivec3(), e)).collect();
        keyed.par_sort_unstable_by_key(|(k, _)| (k.x, k.y, k.z));
        let mut ranges = HashMap::new();
        let mut s = 0;
        while s < keyed.len() {
            let mut e = s + 1;
            while e < keyed.len() && keyed[e].0 == keyed[s].0 {
                e += 1;
            }
            ranges.insert(keyed[s].0, (s as u32, e as u32));
            s = e;
        }
        let p = keyed.iter().map(|(_, e)| e.0).collect();
        let n = keyed.iter().map(|(_, e)| e.1).collect();
        let w = keyed.iter().map(|(_, e)| e.2).collect();
        PointGrid { cell, p, n, w, ranges }
    }

    /// Calls `f(index)` for every point in the cells overlapping the cube of half-side `r`.
    #[inline]
    fn around(&self, x: Vec3, r: f32, mut f: impl FnMut(usize)) {
        let lo = ((x - Vec3::splat(r)) / self.cell).floor().as_ivec3();
        let hi = ((x + Vec3::splat(r)) / self.cell).floor().as_ivec3();
        for z in lo.z..=hi.z {
            for y in lo.y..=hi.y {
                for xx in lo.x..=hi.x {
                    if let Some(&(s, e)) = self.ranges.get(&IVec3::new(xx, y, z)) {
                        for i in s..e {
                            f(i as usize);
                        }
                    }
                }
            }
        }
    }
}

/// The RIMLS field over a point grid.
struct Field<'a> {
    grid: &'a PointGrid,
    h: f32,
    /// Robust weight scales: residual (mm) and normal difference.
    sigma_r: f32,
    sigma_n: f32,
    iterations: usize,
    /// A corner is known only if some point lies within this distance of it along that point's
    /// tangent plane.
    tangential: f32,
    /// The side of a thin wall is the common normal of the points within this distance beyond
    /// the nearest one.
    side_vote: f32,
}

impl Field<'_> {
    /// (value, gradient) at `x`, or `None` where the surface is not defined (no points around, or
    /// all of them far off along their tangent planes).
    fn eval(&self, x: Vec3, check_tangential: bool) -> Option<(f32, Vec3)> {
        self.eval_excluding(x, check_tangential, usize::MAX)
    }

    /// [`eval`](Self::eval) without the point at grid index `skip`: a point's own offset from
    /// the surface its neighbours define. Counting the point itself (offset 0, full weight, its
    /// own normal for the side test) would pull the measured offset of a stray point towards 0,
    /// the more so the further off it is.
    fn eval_excluding(&self, x: Vec3, check_tangential: bool, skip: usize) -> Option<(f32, Vec3)> {
        self.eval_within(x, check_tangential, skip, self.h)
    }

    /// The field at a grid corner. A cell the surface crosses has corners up to √3 cells from
    /// it, and where the surface steps (the flank of a raised letter, sampled only at its foot
    /// and its top) further than that from every point: beyond the support radius. Such a corner
    /// only has to tell marching cubes which side it is on, so it is evaluated with a wider
    /// kernel instead of being left unknown, which would cut a hole into the surface.
    fn eval_corner(&self, x: Vec3) -> Option<(f32, Vec3)> {
        [1.0, 1.5, 2.25].iter().find_map(|s| self.eval_within(x, true, usize::MAX, self.h * s))
    }

    fn eval_within(&self, x: Vec3, check_tangential: bool, skip: usize, h: f32) -> Option<(f32, Vec3)> {
        let h2 = h * h;
        let g = self.grid;
        // Which side of a thin wall `x` belongs to: the nearest point's.
        let mut nearest = (f32::MAX, Vec3::ZERO);
        let mut tangential_min = f32::MAX;
        let mut idx: smallvec_like::Buf = smallvec_like::Buf::new();
        g.around(x, h, |i| {
            let d = x - g.p[i];
            let d2 = d.length_squared();
            if d2 < h2 && i != skip {
                idx.push(i as u32);
                if d2 < nearest.0 {
                    nearest = (d2, g.n[i]);
                }
                if check_tangential {
                    let s = d.dot(g.n[i]);
                    tangential_min = tangential_min.min((d2 - s * s).max(0.0));
                }
            }
        });
        if idx.len() < 3 || (check_tangential && tangential_min > self.tangential * self.tangential) {
            return None;
        }
        // Its normal alone is a poor judge on real scans: one tilted normal (a steep flank, a
        // stray point) shuts out the surface around it, the field there comes from a point or
        // two, and the cell gets a pinhole. So the points about as near as the nearest one
        // decide together when they agree; between the faces of a thin wall they do not, and
        // the nearest point decides alone.
        let mut side = nearest.1;
        {
            let reach = nearest.0.sqrt() + self.side_vote;
            let (mut sum, mut count) = (Vec3::ZERO, 0u32);
            for &i in idx.as_slice() {
                if (x - g.p[i as usize]).length_squared() <= reach * reach {
                    sum += g.n[i as usize];
                    count += 1;
                }
            }
            if sum.length() > 0.6 * count as f32 {
                side = sum.normalize();
            }
        }
        let mut f = 0.0f32;
        let mut grad = Vec3::ZERO;
        let inv_h2 = 1.0 / h2;
        for it in 0..=self.iterations {
            let (mut sw, mut sf) = (0.0f32, 0.0f32);
            let (mut sgw, mut sgf, mut sn) = (Vec3::ZERO, Vec3::ZERO, Vec3::ZERO);
            for &i in idx.as_slice() {
                let i = i as usize;
                let n = g.n[i];
                // The other side of a thin wall.
                if n.dot(side) < 0.2 {
                    continue;
                }
                let d = x - g.p[i];
                let t = 1.0 - d.length_squared() * inv_h2;
                if t <= 0.0 {
                    continue;
                }
                let fx = d.dot(n);
                let mut alpha = g.w[i];
                if it > 0 {
                    let r = (fx - f) / self.sigma_r;
                    let dn = (n - grad).length() / self.sigma_n;
                    alpha *= (-r * r).exp() * (-dn * dn).exp();
                }
                // Wendland-like weight (1 - d²/h²)^4 and its gradient.
                let t3 = t * t * t;
                let w = alpha * t3 * t;
                let gw = d * (alpha * -8.0 * inv_h2 * t3);
                sw += w;
                sf += w * fx;
                sgw += gw;
                sgf += gw * fx;
                sn += w * n;
            }
            if sw < 1e-12 {
                return None;
            }
            let nf = sf / sw;
            let ng = (sgf - nf * sgw + sn) / sw;
            let converged = it > 0 && (nf - f).abs() < 1e-4 * self.h;
            f = nf;
            grad = ng;
            if converged {
                break;
            }
        }
        Some((f, grad))
    }
}

/// A growable index buffer that avoids heap allocation for the usual neighbour counts.
mod smallvec_like {
    pub struct Buf {
        inline: [u32; 256],
        len: usize,
        heap: Vec<u32>,
    }
    impl Buf {
        #[inline]
        pub fn new() -> Self {
            Buf { inline: [0; 256], len: 0, heap: Vec::new() }
        }
        #[inline]
        pub fn push(&mut self, v: u32) {
            if self.len < 256 {
                self.inline[self.len] = v;
            } else {
                if self.heap.is_empty() {
                    self.heap.extend_from_slice(&self.inline);
                }
                self.heap.push(v);
            }
            self.len += 1;
        }
        #[inline]
        pub fn len(&self) -> usize {
            self.len
        }
        #[inline]
        pub fn as_slice(&self) -> &[u32] {
            if self.len <= 256 { &self.inline[..self.len] } else { &self.heap }
        }
    }
}

/// The sampling pitch: the side of the square each point stands for on its surface, median over
/// a sample of the grid's points. Measured from the number of points on the same side of the
/// surface within a few nearest-neighbour distances, not from the nearest-neighbour distance
/// itself: that one is the pitch on a regular grid (a merged scan), but well below it on
/// irregular samples (0.39 for a 0.5 mm grid jittered by ±0.15 mm), and every radius here is a
/// multiple of the spacing.
fn point_spacing(grid: &PointGrid) -> f32 {
    let step = (grid.p.len() / 5000).max(1);
    let nearest = nearest_neighbour_distance(grid, step);
    let r = 4.0 * nearest;
    let mut d: Vec<f32> = (0..grid.p.len())
        .into_par_iter()
        .step_by(step)
        .map(|i| {
            let (x, n) = (grid.p[i], grid.n[i]);
            let mut count = 0u32;
            grid.around(x, r, |j| {
                if grid.p[j].distance_squared(x) < r * r && grid.n[j].dot(n) > 0.0 {
                    count += 1;
                }
            });
            (std::f32::consts::PI * r * r / count.max(1) as f32).sqrt()
        })
        .collect();
    if d.is_empty() {
        return nearest;
    }
    let mid = d.len() / 2;
    d.select_nth_unstable_by(mid, |a, b| a.total_cmp(b));
    // Never above the nearest-neighbour figure by more than the two can differ on real samples.
    d[mid].clamp(nearest, 2.0 * nearest)
}

/// Median nearest-neighbour distance over every `step`-th point of the grid.
fn nearest_neighbour_distance(grid: &PointGrid, step: usize) -> f32 {
    let mut d: Vec<f32> = (0..grid.p.len())
        .into_par_iter()
        .step_by(step)
        .map(|i| {
            let x = grid.p[i];
            let mut best = f32::MAX;
            grid.around(x, grid.cell, |j| {
                if j != i {
                    let d2 = grid.p[j].distance_squared(x);
                    if d2 > 1e-12 && d2 < best {
                        best = d2;
                    }
                }
            });
            if best == f32::MAX { grid.cell } else { best.sqrt() }
        })
        .collect();
    if d.is_empty() {
        return 1.0;
    }
    let mid = d.len() / 2;
    d.select_nth_unstable_by(mid, |a, b| a.total_cmp(b));
    d[mid]
}

/// Re-estimates every normal from the positions around it (weighted PCA over `radius`), oriented
/// like the given normal. A merged scan's points are far more accurate than its per-point
/// normals (measured on real data: ~0.03 mm plane residual, but normals ~10–13° off the local
/// plane), and the implicit surface is only as smooth as the normals. Neighbours facing the other
/// way (the far side of a thin wall) are left out; a point with too few neighbours keeps its
/// normal.
fn refit_normals(grid: &mut PointGrid, radius: f32) {
    let r2 = radius * radius;
    let new: Vec<Vec3> = (0..grid.p.len())
        .into_par_iter()
        .map(|i| {
            let x = grid.p[i];
            // Orientation from the neighbourhood, not the point alone: on dark or shiny spots
            // single normals come out flipped, and a flipped normal punches a tunnel into the
            // surface. Within this small radius the far side of a thin wall is not reached.
            let mut vote = Vec3::ZERO;
            grid.around(x, radius, |j| {
                let d2 = grid.p[j].distance_squared(x);
                if d2 < r2 {
                    vote += grid.n[j] * (1.0 - d2 / r2);
                }
            });
            let own = grid.n[i];
            let n0 = if vote.dot(own) < 0.0 && vote.length() > 1.0 { vote.normalize() } else { own };
            let (mut sw, mut sp) = (0.0f32, Vec3::ZERO);
            let mut near: smallvec_like::Buf = smallvec_like::Buf::new();
            grid.around(x, radius, |j| {
                let d2 = grid.p[j].distance_squared(x);
                if d2 < r2 && grid.n[j].dot(n0) > 0.0 {
                    near.push(j as u32);
                    let w = (1.0 - d2 / r2).powi(2);
                    sw += w;
                    sp += grid.p[j] * w;
                }
            });
            if near.len() < 6 || sw <= 0.0 {
                return n0;
            }
            let c = sp / sw;
            let mut cov = glam::Mat3::ZERO;
            for &j in near.as_slice() {
                let d = grid.p[j as usize] - c;
                let w = (1.0 - grid.p[j as usize].distance_squared(x) / r2).powi(2);
                cov += glam::Mat3::from_cols(d * (d.x * w), d * (d.y * w), d * (d.z * w));
            }
            let n = crate::normals::compute_normal_from_covariance(cov);
            if !n.is_finite() || n == Vec3::ZERO {
                return n0;
            }
            if n.dot(n0) < 0.0 { -n } else { n }
        })
        .collect();
    grid.n = new;
}

/// Marks the points of a sparse layer stacked on a denser one. Dark ink, glints and edges of
/// depth frames leave thin scatters of points a millimetre in front of or behind the surface,
/// facing either way. They sit inside the support radius of the real surface: the field then
/// switches between the two layers from corner to corner, which tears the surface into flaps,
/// pits and holes. A point goes when, within `radius` along the surface, the sheet it lies on
/// has at most `STRAY_LAYER_RATIO` as many points as the layer right above or below it within
/// `reach`. Both faces of a thin wall are equally dense and stay; so does relief, whose points
/// have the rest of the surface beside them, not above or below. Returns one flag per point: keep.
fn dense_layers(grid: &PointGrid, radius: f32, reach: f32, spacing: f32) -> Vec<bool> {
    const STRAY_LAYER_RATIO: f32 = 0.6;
    let mut keep = vec![true; grid.p.len()];
    for _ in 0..2 {
        let next: Vec<bool> = (0..grid.p.len())
            .into_par_iter()
            .map(|i| {
                if !keep[i] {
                    return false;
                }
                let (x, n) = (grid.p[i], grid.n[i]);
                // The direction the layers here are stacked along: the common axis of the
                // normals around, whichever way they face. Not the point's own normal: a stray
                // point's normal is often tilted far off, and so is that of a point on a steep
                // flank, for which the top and the foot of the step are no layers.
                let mut axis = n;
                for _ in 0..2 {
                    let mut sum = Vec3::ZERO;
                    grid.around(x, reach, |j| {
                        if keep[j] && grid.p[j].distance_squared(x) < reach * reach {
                            sum += grid.n[j] * grid.n[j].dot(axis).signum();
                        }
                    });
                    axis = sum.normalize_or(n);
                }
                // What is stacked in front of (0) and behind (1) the point: count and summed
                // offset along the surface.
                let (mut own, mut other) = (0u32, [(0u32, Vec3::ZERO); 2]);
                grid.around(x, reach, |j| {
                    if !keep[j] {
                        return;
                    }
                    let d = grid.p[j] - x;
                    let s = d.dot(axis);
                    let lateral2 = d.length_squared() - s * s;
                    if lateral2 > radius * radius || s.abs() > reach {
                        return;
                    }
                    // The same sheet: near the tangent plane (slopes up to 1:2 count) and
                    // facing the same way.
                    if s.abs() < (0.4 * spacing).max(0.5 * lateral2.max(0.0).sqrt()) && grid.n[j].dot(n) > 0.3 {
                        own += 1;
                    } else if s.abs() > 0.6 * spacing {
                        let o = &mut other[(s < 0.0) as usize];
                        o.0 += 1;
                        o.1 += d - axis * s;
                    }
                });
                let (count, offset) = if other[0].0 >= other[1].0 { other[0] } else { other[1] };
                // A layer right above or below surrounds the point's position; the foot or the
                // top of a step next to it lies to one side (a half disc: 0.42 radius off).
                let stacked = count >= 4 && (offset / count as f32).length() < 0.35 * radius;
                !(stacked && own as f32 <= STRAY_LAYER_RATIO * count as f32)
            })
            .collect();
        if next == keep {
            break;
        }
        keep = next;
    }
    keep
}

/// Smooths normals with neighbours of similar orientation (bilateral in normal space), keeping
/// creases: a neighbour across an edge (normals > ~40° apart) barely counts.
fn smooth_normals(grid: &mut PointGrid, radius: f32) {
    let r2 = radius * radius;
    let new: Vec<Vec3> = (0..grid.p.len())
        .into_par_iter()
        .map(|i| {
            let (x, n0) = (grid.p[i], grid.n[i]);
            let mut sum = Vec3::ZERO;
            grid.around(x, radius, |j| {
                let d2 = grid.p[j].distance_squared(x);
                if d2 < r2 {
                    let c = grid.n[j].dot(n0);
                    if c > 0.5 {
                        let s = 1.0 - d2 / r2;
                        let a = (c - 0.5) * 2.0;
                        sum += grid.n[j] * (s * a * a * grid.w[j]);
                    }
                }
            });
            let s = sum.normalize_or_zero();
            if s == Vec3::ZERO { n0 } else { s }
        })
        .collect();
    grid.n = new;
}

/// Rounds the open edges. Marching cubes only meshes cells whose corners are all known, so the
/// rim of an open surface follows the voxel grid in cell-sized stairs, and a small hole is a
/// square. Every rim vertex is moved towards the middle of its two neighbours along the rim (a
/// few passes), never more than `max_move` in total, which turns the stairs into the line they
/// approximate without moving the rim as a whole.
fn round_open_edges(vertices: &mut [Point3D], tris: &[[u32; 3]], max_move: f32) {
    let mut directed: HashMap<(u32, u32), ()> = HashMap::with_capacity(tris.len() * 3);
    for t in tris {
        for k in 0..3 {
            directed.insert((t[k], t[(k + 1) % 3]), ());
        }
    }
    // Rim edge a -> b: next[a] = b, prev[b] = a. A vertex where two rims touch is left alone.
    let (mut next, mut prev): (HashMap<u32, u32>, HashMap<u32, u32>) = (HashMap::new(), HashMap::new());
    let mut shared: Vec<u32> = Vec::new();
    for &(a, b) in directed.keys() {
        if !directed.contains_key(&(b, a)) {
            if next.insert(a, b).is_some() {
                shared.push(a);
            }
            if prev.insert(b, a).is_some() {
                shared.push(b);
            }
        }
    }
    let rim: Vec<(u32, u32, u32)> = next.iter().filter(|(a, _)| !shared.contains(a)).filter_map(|(&a, &b)| prev.get(&a).map(|&p| (a, p, b))).collect();
    let start: Vec<Vec3> = rim.iter().map(|r| Vec3::from_array(vertices[r.0 as usize].position)).collect();
    for _ in 0..4 {
        let moved: Vec<Vec3> = rim
            .iter()
            .zip(&start)
            .map(|(&(a, p, b), &s)| {
                let at = |i: u32| Vec3::from_array(vertices[i as usize].position);
                let target = at(a) + 0.5 * ((at(p) + at(b)) * 0.5 - at(a));
                s + (target - s).clamp_length_max(max_move)
            })
            .collect();
        for (r, m) in rim.iter().zip(moved) {
            vertices[r.0 as usize].position = m.to_array();
        }
    }
}

/// Reconstructs a welded triangle mesh from an oriented point cloud (see the module docs).
/// Points without a usable normal are ignored; `confidence` (observation count) weights them.
pub fn reconstruct(cloud: &PointCloud, config: &SurfaceConfig) -> (Mesh, SurfaceStats) {
    let mut stats = SurfaceStats { input_points: cloud.len(), ..Default::default() };
    let pts: Vec<(Vec3, Vec3, f32)> = cloud
        .points
        .par_iter()
        .filter_map(|p| {
            let pos = Vec3::from_array(p.position);
            let n = Vec3::from_array(p.normal);
            let len = n.length();
            let w = if p.confidence.is_finite() && p.confidence > 0.0 { p.confidence.min(9.0).sqrt() } else { 1.0 };
            (pos.is_finite() && n.is_finite() && len > 1e-3).then(|| (pos, n / len, w))
        })
        .collect();
    if pts.len() < 10 || !(config.cell_mm > 0.0) {
        return (Mesh::new(), stats);
    }
    // A first grid only to measure the spacing.
    let probe_cell = {
        let (mn, mx) = pts.iter().fold((Vec3::splat(f32::MAX), Vec3::splat(f32::MIN)), |(a, b), e| (a.min(e.0), b.max(e.0)));
        // About 4 points per cell for a surface: area / n per point.
        let area = {
            let e = mx - mn;
            2.0 * (e.x * e.y + e.y * e.z + e.z * e.x)
        };
        (area / pts.len() as f32).sqrt().max(1e-3) * 4.0
    };
    let spacing = point_spacing(&PointGrid::new(pts.clone(), probe_cell));
    stats.point_spacing_mm = spacing;
    let h = (config.detail * spacing).max(1.5 * config.cell_mm).max(1e-3);
    stats.support_radius_mm = h;
    let mut grid = PointGrid::new(pts, h);
    if config.outlier_sigma > 0.0 {
        let keep = dense_layers(&grid, 1.5 * spacing, 1.25 * h, spacing);
        let kept: Vec<(Vec3, Vec3, f32)> = (0..grid.p.len()).filter(|&i| keep[i]).map(|i| (grid.p[i], grid.n[i], grid.w[i])).collect();
        stats.removed_outliers = grid.p.len() - kept.len();
        if stats.removed_outliers > 0 {
            grid = PointGrid::new(kept, h);
        }
    }
    if config.refit_normals {
        refit_normals(&mut grid, 2.5 * spacing);
    }
    for _ in 0..config.normal_passes {
        smooth_normals(&mut grid, h * 0.75);
    }
    // Points far off the surface their neighbours agree on (dark ink, glints: scattered depth)
    // fold the mesh into small tunnels; drop them and refit.
    if config.outlier_sigma > 0.0 {
        let probe = Field { grid: &grid, h, sigma_r: 0.5 * h, sigma_n: 0.8, iterations: config.robust_iterations, tangential: f32::MAX, side_vote: spacing };
        let offs: Vec<f32> = grid.p.par_iter().enumerate().map(|(i, &p)| probe.eval_excluding(p, false, i).map_or(f32::MAX, |(f, _)| f.abs())).collect();
        let mut sorted: Vec<f32> = offs.iter().copied().filter(|v| v.is_finite() && *v < f32::MAX).collect();
        if sorted.len() > 100 {
            let mid = sorted.len() / 2;
            sorted.select_nth_unstable_by(mid, |a, b| a.total_cmp(b));
            // 1.4826 × median |offset| estimates the standard deviation of the noise.
            let limit = (config.outlier_sigma * 1.4826 * sorted[mid]).max(0.25 * spacing);
            let keep: Vec<(Vec3, Vec3, f32)> = (0..grid.p.len()).filter(|&i| offs[i] <= limit).map(|i| (grid.p[i], grid.n[i], grid.w[i])).collect();
            let removed = grid.p.len() - keep.len();
            stats.removed_outliers += removed;
            if removed > 0 {
                grid = PointGrid::new(keep, h);
            }
        }
    }
    let field = Field {
        grid: &grid,
        h,
        sigma_r: 0.5 * h,
        sigma_n: 0.8,
        iterations: config.robust_iterations,
        tangential: (1.5 * spacing).max(1.3 * config.cell_mm),
        side_vote: spacing,
    };

    // ---- corners to evaluate: around every point, within the band a crossing cell can need.
    let c = config.cell_mm;
    let band = (2.0f32).max(1.0 + spacing / c).ceil() as i32;
    let mut corners: Vec<IVec3> = grid
        .p
        .par_iter()
        .flat_map_iter(|p| {
            let base = (*p / c).floor().as_ivec3();
            (-band + 1..=band).flat_map(move |z| (-band + 1..=band).flat_map(move |y| (-band + 1..=band).map(move |x| base + IVec3::new(x, y, z))))
        })
        .collect();
    corners.par_sort_unstable_by_key(|k| (k.x, k.y, k.z));
    corners.dedup();
    stats.evaluated_corners = corners.len();
    let values: Vec<f32> = corners.par_iter().map(|k| field.eval_corner(k.as_vec3() * c).map_or(f32::NAN, |(f, _)| f)).collect();
    stats.known_corners = values.iter().filter(|v| !v.is_nan()).count();
    let lookup: HashMap<IVec3, f32> = corners.into_iter().zip(values).filter(|(_, v)| !v.is_nan()).collect();

    // ---- marching cubes over cells whose 8 corners are all known.
    let cells: Vec<IVec3> = {
        let mut v: Vec<IVec3> = lookup.keys().copied().collect();
        v.par_sort_unstable_by_key(|k| (k.x, k.y, k.z));
        v
    };
    let per_cell: Vec<(Vec<(IVec3, u8, Vec3)>, Vec<[(IVec3, u8); 3]>)> = cells
        .par_chunks(4096)
        .map(|chunk| {
            let mut verts = Vec::new();
            let mut tris = Vec::new();
            for &cell in chunk {
                let mut v = [0f32; 8];
                let mut idx = 0usize;
                let mut known = true;
                for (i, o) in CORNERS.iter().enumerate() {
                    match lookup.get(&(cell + IVec3::from_array(*o))) {
                        Some(&val) => {
                            v[i] = val;
                            if val < 0.0 {
                                idx |= 1 << i;
                            }
                        }
                        None => {
                            known = false;
                            break;
                        }
                    }
                }
                if !known || EDGE_TABLE[idx] == 0 {
                    continue;
                }
                let mut ek = [(IVec3::ZERO, 0u8); 12];
                for (e, &(a, b, axis)) in EDGES.iter().enumerate() {
                    if EDGE_TABLE[idx] & (1 << e) == 0 {
                        continue;
                    }
                    let ca = cell + IVec3::from_array(CORNERS[a]);
                    ek[e] = (ca, axis as u8);
                    // Each grid edge is emitted by the cell that owns its lower corner's... any
                    // cell: duplicates are welded below.
                    let (va, vb) = (v[a], v[b]);
                    let t = (va / (va - vb)).clamp(1e-3, 1.0 - 1e-3);
                    let mut p = ca.as_vec3();
                    p[axis as usize] += t;
                    verts.push((ca, axis as u8, p * c));
                }
                let row = &TRI_TABLE[idx];
                let mut t = 0;
                while t < 16 && row[t] >= 0 {
                    tris.push([ek[row[t] as usize], ek[row[t + 2] as usize], ek[row[t + 1] as usize]]);
                    t += 3;
                }
            }
            (verts, tris)
        })
        .collect();

    // ---- weld by grid edge.
    let mut verts: Vec<(IVec3, u8, Vec3)> = Vec::new();
    let mut tri_keys = Vec::new();
    for (v, t) in per_cell {
        verts.extend(v);
        tri_keys.extend(t);
    }
    let key = |k: &(IVec3, u8)| (k.0.x, k.0.y, k.0.z, k.1);
    verts.par_sort_unstable_by_key(|e| (e.0.x, e.0.y, e.0.z, e.1));
    verts.dedup_by_key(|e| (e.0.x, e.0.y, e.0.z, e.1));
    let find = |k: &(IVec3, u8)| verts.binary_search_by_key(&key(k), |e| (e.0.x, e.0.y, e.0.z, e.1)).expect("edge vertex") as u32;
    let tris: Vec<[u32; 3]> = tri_keys.par_iter().map(|t| [find(&t[0]), find(&t[1]), find(&t[2])]).filter(|t| t[0] != t[1] && t[1] != t[2] && t[0] != t[2]).collect();

    // ---- project vertices onto the surface; the gradient there is the vertex normal.
    let max_move = 0.9 * c;
    let projected: Vec<(Vec3, Vec3)> = verts
        .par_iter()
        .map(|&(_, _, start)| {
            let mut x = start;
            let mut normal = Vec3::ZERO;
            for _ in 0..config.projection_steps {
                let Some((f, g)) = field.eval(x, false) else { break };
                let gl2 = g.length_squared();
                if gl2 < 1e-8 {
                    break;
                }
                normal = g / gl2.sqrt();
                let next = x - g * (f / gl2);
                // Stay near the cell the vertex came from (no jumping across thin walls).
                x = start + (next - start).clamp_length_max(max_move);
            }
            if normal == Vec3::ZERO {
                if let Some((_, g)) = field.eval(x, false) {
                    normal = g.normalize_or_zero();
                }
            }
            (x, normal)
        })
        .collect();

    let positions: Vec<Vec3> = projected.iter().map(|p| p.0).collect();
    let normals_by_old: Vec<Vec3> = projected.iter().map(|p| p.1).collect();
    // filter_components renumbers vertices; carry the normals through by position order.
    let (positions_kept, tris, dropped) = filter_components(positions.clone(), tris, config.min_component_triangles, config.min_component_fraction);
    stats.dropped_components = dropped;
    let color = ColorRgba::new(210, 215, 225, 255);
    // Map kept positions back to their normals (filter_components keeps first-use order of the
    // surviving vertices, so a lookup by exact position is unambiguous enough; ties share normals).
    let index: HashMap<[u32; 3], usize> = positions.iter().enumerate().map(|(i, p)| (p.to_array().map(f32::to_bits), i)).collect();
    let vertices: Vec<Point3D> = positions_kept
        .into_iter()
        .map(|p| {
            let n = index.get(&p.to_array().map(f32::to_bits)).map(|&i| normals_by_old[i]).unwrap_or(Vec3::Z);
            Point3D { position: p.to_array(), normal: n.to_array(), color, confidence: 1.0 }
        })
        .collect();
    let mut vertices = vertices;
    round_open_edges(&mut vertices, &tris, 0.6 * c);
    let triangles = tris.into_iter().map(|t| Triangle::new(t[0], t[1], t[2])).collect();
    let mut mesh = Mesh::from_geometry(vertices, triangles);
    mesh.update_bbox();
    (mesh, stats)
}
