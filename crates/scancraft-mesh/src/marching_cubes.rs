//! Surface reconstruction from an oriented point cloud.
//!
//! 1. A signed distance field is evaluated on a *sparse* voxel grid: only grid corners within a
//!    narrow band around the input points get a value. Each value is the Gaussian-weighted mean of
//!    the distances from the corner to the tangent planes `n·(c − p)` of the nearby points.
//!    Corners away from data stay unknown, and cells touching an unknown corner produce no
//!    triangles, so single-view scans give open patches instead of invented sheets.
//! 2. Marching cubes runs on the known cells. Vertices are keyed by grid edge, so every crossing
//!    produces exactly one vertex shared by all cells around that edge (welded, manifold output).
//!    Triangles are wound counter-clockwise seen from outside (from the side the normals face).
//! 3. Tiny disconnected fragments (noise bubbles) are dropped.
//!
//! Normal convention: point normals face away from the surface interior (towards the scanner for
//! a single depth view). Positive distance is outside.

use crate::tables::{EDGE_TABLE, TRI_TABLE};
use glam::{IVec3, Vec3};
use rayon::prelude::*;
use scancraft_core::{ColorRgba, Mesh, Point3D, PointCloud, Triangle};
use std::collections::HashMap;

/// Corners per block side. Blocks are the unit of sparse storage and parallel work.
const B: i32 = 8;
const BV: usize = (B * B * B) as usize;
/// Bits per axis of a packed global corner coordinate.
const COORD_BITS: u32 = 20;
const COORD_LIMIT: i32 = 1 << COORD_BITS;
const BLOCK_BITS: u32 = COORD_BITS - 3;

/// Corner offsets of a cube (Bourke ordering).
pub(crate) const CORNERS: [[i32; 3]; 8] = [
    [0, 0, 0],
    [1, 0, 0],
    [1, 1, 0],
    [0, 1, 0],
    [0, 0, 1],
    [1, 0, 1],
    [1, 1, 1],
    [0, 1, 1],
];

/// For every cube edge: (lower corner, upper corner, axis). "Lower" has the smaller coordinate
/// along the axis, so a grid edge is identified by its lower corner + axis regardless of cell.
pub(crate) const EDGES: [(usize, usize, u64); 12] = [
    (0, 1, 0),
    (1, 2, 1),
    (3, 2, 0),
    (0, 3, 1),
    (4, 5, 0),
    (5, 6, 1),
    (7, 6, 0),
    (4, 7, 1),
    (0, 4, 2),
    (1, 5, 2),
    (2, 6, 2),
    (3, 7, 2),
];

pub struct MarchingCubes {
    /// Grid spacing in millimetres.
    pub cell_size_mm: f32,
    /// Radius around each input point inside which the distance field is evaluated.
    /// `0.0` (the default) picks it from the cell size and the measured point spacing.
    pub max_search_dist_mm: f32,
    /// Connected pieces with fewer triangles than this are discarded.
    pub min_component_triangles: usize,
    /// Connected pieces smaller than this fraction of the largest piece are discarded.
    pub min_component_fraction: f32,
}

impl Default for MarchingCubes {
    fn default() -> Self {
        Self::new(1.5)
    }
}

/// Statistics of the last reconstruction (exposed for diagnostics/examples).
#[derive(Debug, Clone, Default)]
pub struct ReconstructionStats {
    pub input_points: usize,
    pub oriented_points: usize,
    pub point_spacing_mm: f32,
    pub band_radius_mm: f32,
    pub known_corners: usize,
    pub blocks: usize,
    pub dropped_components: usize,
}

#[inline]
fn pack(c: IVec3, bits: u32) -> u64 {
    ((c.x as u64) << (2 * bits)) | ((c.y as u64) << bits) | (c.z as u64)
}

#[inline]
fn unpack(k: u64, bits: u32) -> IVec3 {
    let m = (1u64 << bits) - 1;
    IVec3::new(((k >> (2 * bits)) & m) as i32, ((k >> bits) & m) as i32, (k & m) as i32)
}

#[inline]
fn local_index(l: IVec3) -> usize {
    ((l.z * B + l.y) * B + l.x) as usize
}

impl MarchingCubes {
    pub fn new(cell_size_mm: f32) -> Self {
        Self {
            cell_size_mm,
            max_search_dist_mm: 0.0,
            min_component_triangles: 24,
            min_component_fraction: 0.002,
        }
    }

    pub fn reconstruct(&self, cloud: &PointCloud) -> Mesh {
        self.reconstruct_with_stats(cloud).0
    }

    pub fn reconstruct_with_stats(&self, cloud: &PointCloud) -> (Mesh, ReconstructionStats) {
        let mut stats = ReconstructionStats { input_points: cloud.len(), ..Default::default() };
        let h = self.cell_size_mm;
        if !(h.is_finite() && h > 0.0) {
            return (Mesh::new(), stats);
        }

        // ---- 1. Oriented input points -------------------------------------------------------
        let pts: Vec<(Vec3, Vec3)> = cloud
            .points
            .par_iter()
            .filter_map(|p| {
                let pos = Vec3::from_array(p.position);
                let n = Vec3::from_array(p.normal);
                let len = n.length();
                (pos.is_finite() && n.is_finite() && len > 1e-3).then(|| (pos, n / len))
            })
            .collect();
        stats.oriented_points = pts.len();
        if pts.len() < 10 {
            return (Mesh::new(), stats);
        }

        let (mn, mx) = pts
            .par_iter()
            .fold(
                || (Vec3::splat(f32::MAX), Vec3::splat(f32::MIN)),
                |(a, b), (p, _)| (a.min(*p), b.max(*p)),
            )
            .reduce(|| (Vec3::splat(f32::MAX), Vec3::splat(f32::MIN)), |(a, b), (c, d)| (a.min(c), b.max(d)));

        // Grid origin: one full block of margin below the data.
        let base = (mn / h).floor().as_ivec3() - IVec3::splat(B);
        let origin = base.as_vec3() * h;
        let extent = ((mx - origin) / h).ceil().as_ivec3() + IVec3::splat(2 * B);
        if extent.max_element() >= COORD_LIMIT {
            log::warn!("marching cubes: grid of {:?} cells exceeds the supported size; use a larger cell", extent);
            return (Mesh::new(), stats);
        }

        // Points in grid units (relative to origin), sorted by (block, cell).
        let mut gp: Vec<(u64, Vec3, Vec3)> = pts
            .par_iter()
            .map(|(p, n)| {
                let q = (*p - origin) / h;
                let c = q.floor().as_ivec3();
                let blk = c >> 3;
                let loc = c & 7;
                let key = (pack(blk, BLOCK_BITS) << 9) | ((loc.x << 6 | loc.y << 3 | loc.z) as u64);
                (key, q, *n)
            })
            .collect();
        gp.par_sort_unstable_by_key(|e| e.0);

        let cell_runs = runs(&gp, |k| k);
        let block_runs = runs(&gp, |k| k >> 9);

        // ---- 2. Point spacing and band radius -------------------------------------------------
        let spacing_cells = estimate_spacing(&gp, &cell_runs);
        stats.point_spacing_mm = spacing_cells * h;
        let radius_c = if self.max_search_dist_mm > 0.0 {
            (self.max_search_dist_mm / h).clamp(0.5, 7.5)
        } else {
            // A cell crossed by the surface has all corners within √3 cells of it; add the
            // sampling gap so such cells are always fully known (watertightness).
            (1.75 + spacing_cells).clamp(2.0, 6.0)
        };
        // Corners whose tangential offset to every nearby point exceeds this are left unknown.
        // This trims the extrapolated fringe at the rim of open scans to about one cell.
        let tangential_c = 1.3 + 0.5 * spacing_cells;
        stats.band_radius_mm = radius_c * h;

        // ---- 3. Sparse signed distance field ----------------------------------------------------
        let block_lookup: HashMap<u64, usize> =
            block_runs.iter().enumerate().map(|(i, r)| (r.0, i)).collect();
        let mut active: Vec<u64> = block_runs
            .par_iter()
            .flat_map_iter(|r| {
                let b = unpack(r.0, BLOCK_BITS);
                (0..27).filter_map(move |i| {
                    let nb = b + IVec3::new(i % 3 - 1, (i / 3) % 3 - 1, i / 9 - 1);
                    (nb.min_element() >= 0).then(|| pack(nb, BLOCK_BITS))
                })
            })
            .collect();
        active.par_sort_unstable();
        active.dedup();

        let r2 = radius_c * radius_c;
        let t2max = tangential_c * tangential_c;
        let inv_2s2 = 1.0 / (2.0 * (0.5 * radius_c).powi(2));

        let fields: Vec<(u64, Box<[f32; BV]>)> = active
            .par_iter()
            .filter_map(|&bk| {
                let b = unpack(bk, BLOCK_BITS);
                let lo_b = b * B;
                let hi_b = lo_b + IVec3::splat(B - 1);
                let mut wsum = [0f32; BV];
                let mut wd = [0f32; BV];
                let mut t2min = [f32::MAX; BV];
                let mut touched = false;
                for i in 0..27 {
                    let nb = b + IVec3::new(i % 3 - 1, (i / 3) % 3 - 1, i / 9 - 1);
                    if nb.min_element() < 0 {
                        continue;
                    }
                    let Some(&ri_) = block_lookup.get(&pack(nb, BLOCK_BITS)) else { continue };
                    let (_, s, e) = block_runs[ri_];
                    for &(_, q, n) in &gp[s..e] {
                        let lo = (q - Vec3::splat(radius_c)).ceil().as_ivec3().max(lo_b);
                        let hi = (q + Vec3::splat(radius_c)).floor().as_ivec3().min(hi_b);
                        if lo.cmpgt(hi).any() {
                            continue;
                        }
                        for z in lo.z..=hi.z {
                            for y in lo.y..=hi.y {
                                for x in lo.x..=hi.x {
                                    let v = Vec3::new(x as f32, y as f32, z as f32) - q;
                                    let d2 = v.length_squared();
                                    if d2 > r2 {
                                        continue;
                                    }
                                    let sd = n.dot(v);
                                    let li = local_index(IVec3::new(x, y, z) - lo_b);
                                    let w = (-d2 * inv_2s2).exp();
                                    wsum[li] += w;
                                    wd[li] += w * sd;
                                    let t2 = (d2 - sd * sd).max(0.0);
                                    if t2 < t2min[li] {
                                        t2min[li] = t2;
                                    }
                                    touched = true;
                                }
                            }
                        }
                    }
                }
                if !touched {
                    return None;
                }
                let mut out = Box::new([f32::NAN; BV]);
                let mut any = false;
                for li in 0..BV {
                    if wsum[li] > 1e-12 && t2min[li] <= t2max {
                        out[li] = wd[li] / wsum[li] * h;
                        any = true;
                    }
                }
                any.then_some((bk, out))
            })
            .collect();
        stats.blocks = fields.len();
        stats.known_corners = fields.par_iter().map(|(_, f)| f.iter().filter(|v| !v.is_nan()).count()).sum();

        let field_lookup: HashMap<u64, usize> =
            fields.iter().enumerate().map(|(i, (k, _))| (*k, i)).collect();

        // ---- 4. Marching cubes per block ------------------------------------------------------
        let per_block: Vec<(Vec<(u64, Vec3)>, Vec<[u64; 3]>)> = fields
            .par_iter()
            .map(|(bk, _)| {
                let b = unpack(*bk, BLOCK_BITS);
                // Gather (B+1)^3 corner values from this block and its +x/+y/+z neighbours.
                const S: usize = (B + 1) as usize;
                let mut vals = [f32::NAN; S * S * S];
                for o in 0..8 {
                    let off = IVec3::new(o & 1, (o >> 1) & 1, (o >> 2) & 1);
                    let Some(&fi) = field_lookup.get(&pack(b + off, BLOCK_BITS)) else { continue };
                    let f = &fields[fi].1;
                    let lo = off * B;
                    let hi = (lo + IVec3::splat(B - 1)).min(IVec3::splat(B));
                    for z in lo.z..=hi.z {
                        for y in lo.y..=hi.y {
                            for x in lo.x..=hi.x {
                                let l = IVec3::new(x, y, z) - lo;
                                vals[(z as usize * S + y as usize) * S + x as usize] = f[local_index(l)];
                            }
                        }
                    }
                }
                let at = |x: i32, y: i32, z: i32| vals[(z as usize * S + y as usize) * S + x as usize];

                let mut verts: Vec<(u64, Vec3)> = Vec::new();
                let mut local: HashMap<u64, ()> = HashMap::new();
                let mut tris: Vec<[u64; 3]> = Vec::new();
                let gbase = b * B;
                for z in 0..B {
                    for y in 0..B {
                        for x in 0..B {
                            let mut v = [0f32; 8];
                            let mut idx = 0usize;
                            let mut known = true;
                            for (i, c) in CORNERS.iter().enumerate() {
                                let val = at(x + c[0], y + c[1], z + c[2]);
                                if val.is_nan() {
                                    known = false;
                                    break;
                                }
                                v[i] = val;
                                if val < 0.0 {
                                    idx |= 1 << i;
                                }
                            }
                            if !known || EDGE_TABLE[idx] == 0 {
                                continue;
                            }
                            let cell = gbase + IVec3::new(x, y, z);
                            let mut ek = [0u64; 12];
                            for (e, &(a, bb, axis)) in EDGES.iter().enumerate() {
                                if EDGE_TABLE[idx] & (1 << e) == 0 {
                                    continue;
                                }
                                let ca = cell + IVec3::from_array(CORNERS[a]);
                                let key = (pack(ca, COORD_BITS) << 2) | axis;
                                ek[e] = key;
                                if local.insert(key, ()).is_none() {
                                    let (va, vb) = (v[a], v[bb]);
                                    let t = (va / (va - vb)).clamp(1e-3, 1.0 - 1e-3);
                                    let mut p = ca.as_vec3();
                                    p[axis as usize] += t;
                                    verts.push((key, origin + p * h));
                                }
                            }
                            let row = &TRI_TABLE[idx];
                            let mut t = 0;
                            while t < 16 && row[t] >= 0 {
                                // The Bourke table winds triangles clockwise seen from the
                                // positive (outside) side; swap to make them counter-clockwise.
                                tris.push([ek[row[t] as usize], ek[row[t + 2] as usize], ek[row[t + 1] as usize]]);
                                t += 3;
                            }
                        }
                    }
                }
                (verts, tris)
            })
            .collect();

        // ---- 5. Weld across blocks ------------------------------------------------------------
        let mut verts: Vec<(u64, Vec3)> = Vec::with_capacity(per_block.iter().map(|b| b.0.len()).sum());
        let mut tri_keys: Vec<[u64; 3]> = Vec::with_capacity(per_block.iter().map(|b| b.1.len()).sum());
        for (v, t) in per_block {
            verts.extend(v);
            tri_keys.extend(t);
        }
        verts.par_sort_unstable_by_key(|e| e.0);
        verts.dedup_by_key(|e| e.0);
        let find = |k: u64| verts.binary_search_by_key(&k, |e| e.0).expect("edge vertex") as u32;
        let tris: Vec<[u32; 3]> = tri_keys.par_iter().map(|t| [find(t[0]), find(t[1]), find(t[2])]).collect();
        drop(tri_keys);

        // ---- 6. Drop small fragments, compact ---------------------------------------------------
        let positions: Vec<Vec3> = verts.into_iter().map(|e| e.1).collect();
        let (positions, tris, dropped) =
            filter_components(positions, tris, self.min_component_triangles, self.min_component_fraction);
        stats.dropped_components = dropped;

        let color = ColorRgba::new(210, 215, 225, 255);
        let vertices: Vec<Point3D> = positions
            .into_iter()
            .map(|p| Point3D { position: p.to_array(), normal: [0.0, 0.0, 1.0], color, confidence: 1.0 })
            .collect();
        let triangles = tris.into_iter().map(|t| Triangle::new(t[0], t[1], t[2])).collect();
        let mut mesh = Mesh::from_geometry(vertices, triangles);
        mesh.recompute_normals();
        (mesh, stats)
    }
}

/// Contiguous runs of equal `key_of(entry.0)`: (key, start, end).
fn runs(gp: &[(u64, Vec3, Vec3)], key_of: impl Fn(u64) -> u64) -> Vec<(u64, usize, usize)> {
    let mut out = Vec::new();
    let mut s = 0;
    while s < gp.len() {
        let k = key_of(gp[s].0);
        let mut e = s + 1;
        while e < gp.len() && key_of(gp[e].0) == k {
            e += 1;
        }
        out.push((k, s, e));
        s = e;
    }
    out
}

/// Median nearest-neighbour distance (in cells) over a sample of points, capped at 4 cells.
fn estimate_spacing(gp: &[(u64, Vec3, Vec3)], cell_runs: &[(u64, usize, usize)]) -> f32 {
    let step = (gp.len() / 4000).max(1);
    let cell_of = |q: Vec3| q.floor().as_ivec3();
    let key_of = |c: IVec3| {
        let blk = c >> 3;
        let loc = c & 7;
        (pack(blk, BLOCK_BITS) << 9) | ((loc.x << 6 | loc.y << 3 | loc.z) as u64)
    };
    let mut d: Vec<f32> = (0..gp.len())
        .into_par_iter()
        .step_by(step)
        .map(|i| {
            let q = gp[i].1;
            let c = cell_of(q);
            let mut best = f32::MAX;
            for o in 0..27 {
                let nc = c + IVec3::new(o % 3 - 1, (o / 3) % 3 - 1, o / 9 - 1);
                if nc.min_element() < 0 {
                    continue;
                }
                if let Ok(r) = cell_runs.binary_search_by_key(&key_of(nc), |r| r.0) {
                    let (_, s, e) = cell_runs[r];
                    for (j, p) in gp[s..e].iter().enumerate() {
                        if s + j != i {
                            let dd = p.1.distance_squared(q);
                            if dd > 1e-12 && dd < best {
                                best = dd;
                            }
                        }
                    }
                }
            }
            if best == f32::MAX {
                4.0
            } else {
                best.sqrt().min(4.0)
            }
        })
        .collect();
    if d.is_empty() {
        return 1.0;
    }
    let mid = d.len() / 2;
    d.select_nth_unstable_by(mid, |a, b| a.total_cmp(b));
    d[mid]
}

/// Removes connected components that are tiny (absolute or relative to the largest), then drops
/// unreferenced vertices. Returns the number of removed components.
pub(crate) fn filter_components(
    positions: Vec<Vec3>,
    tris: Vec<[u32; 3]>,
    min_tris: usize,
    min_fraction: f32,
) -> (Vec<Vec3>, Vec<[u32; 3]>, usize) {
    let n = positions.len();
    let mut parent: Vec<u32> = (0..n as u32).collect();
    fn find(p: &mut [u32], mut i: u32) -> u32 {
        while p[i as usize] != i {
            let g = p[p[i as usize] as usize];
            p[i as usize] = g;
            i = g;
        }
        i
    }
    for t in &tris {
        for &o in &t[1..] {
            let a = find(&mut parent, t[0]);
            let b = find(&mut parent, o);
            if a != b {
                let (lo, hi) = if a < b { (a, b) } else { (b, a) };
                parent[hi as usize] = lo;
            }
        }
    }
    let mut count = vec![0usize; n];
    let roots: Vec<u32> = tris.iter().map(|t| find(&mut parent, t[0])).collect();
    for &r in &roots {
        count[r as usize] += 1;
    }
    let largest = count.iter().copied().max().unwrap_or(0);
    let threshold = min_tris.max((largest as f32 * min_fraction) as usize).min(largest);
    let dropped = count.iter().filter(|&&c| c > 0 && c < threshold).count();

    let mut remap = vec![u32::MAX; n];
    let mut out_pos = Vec::with_capacity(n);
    let mut out_tris = Vec::with_capacity(tris.len());
    for (t, &r) in tris.iter().zip(&roots) {
        if count[r as usize] < threshold {
            continue;
        }
        let mut nt = [0u32; 3];
        for k in 0..3 {
            let v = t[k] as usize;
            if remap[v] == u32::MAX {
                remap[v] = out_pos.len() as u32;
                out_pos.push(positions[v]);
            }
            nt[k] = remap[v];
        }
        out_tris.push(nt);
    }
    (out_pos, out_tris, dropped)
}
