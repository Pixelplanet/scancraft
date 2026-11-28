//! Point preparation for the solid solve: isolated outliers, wall thickness map, ghost layers,
//! offset completion of a missing side, and per-point area weights.
//!
//! Everything here works from the oriented points alone (no side labels): "the other side" of a
//! point is the nearest sheet behind it whose normals face the opposite way.

use super::grid::{cell_key, FxMap, PointGrid};
use super::{PointKind, WorkPoint};
use glam::{IVec3, Vec3};
use rayon::prelude::*;

/// Drops points with fewer than `min_neighbours` other points within `radius`.
pub fn drop_isolated(pts: &mut Vec<WorkPoint>, radius: f32, min_neighbours: usize) -> usize {
    let pos: Vec<Vec3> = pts.iter().map(|p| p.p).collect();
    let grid = PointGrid::new(&pos, radius.max(0.25));
    let keep: Vec<bool> = (0..pts.len())
        .into_par_iter()
        .map(|i| {
            let mut c = 0usize;
            grid.for_each_within(&pos, pos[i], radius, |_, _| c += 1);
            c > min_neighbours // includes the point itself
        })
        .collect();
    let before = pts.len();
    let mut k = 0usize;
    pts.retain(|_| {
        let r = keep[k];
        k += 1;
        r
    });
    before - pts.len()
}

/// Drops points that stand off their local sheet: the plane through the centroid of the
/// same-facing neighbours within `radius` (normal = their mean normal) must be closer than
/// max(`min_mm`, `k_sigma` × the neighbours' own RMS distance to it), and a point whose normal
/// agrees with fewer than 3 of at least 10 neighbours is dropped too. Single stray points pinned
/// by the screening would otherwise pull spikes out of the surface.
pub fn drop_off_surface(pts: &mut Vec<WorkPoint>, radius: f32, min_mm: f32, k_sigma: f32) -> usize {
    let pos: Vec<Vec3> = pts.iter().map(|p| p.p).collect();
    let grid = PointGrid::new(&pos, radius.max(0.25));
    let drop: Vec<bool> = (0..pts.len())
        .into_par_iter()
        .map(|i| {
            let p = &pts[i];
            let (mut c, mut nm, mut k, mut all) = (Vec3::ZERO, Vec3::ZERO, 0usize, 0usize);
            let mut nb: [Vec3; 64] = [Vec3::ZERO; 64];
            grid.for_each_within(&pos, p.p, radius, |j, _| {
                let q = &pts[j as usize];
                all += 1;
                if j as usize != i && q.n.dot(p.n) > 0.7 {
                    c += q.p;
                    nm += q.n;
                    if k < 64 {
                        nb[k] = q.p;
                    }
                    k += 1;
                }
            });
            if k < 6 {
                // A point whose normal agrees with almost none of many neighbours is a stray.
                return k < 3 && all >= 10;
            }
            let c = c / k as f32;
            let nm = nm.normalize_or_zero();
            let m = k.min(64);
            let s2 = nb[..m].iter().map(|q| (*q - c).dot(nm).powi(2)).sum::<f32>() / m as f32;
            let d = (p.p - c).dot(nm).abs();
            d > min_mm.max(k_sigma * s2.sqrt())
        })
        .collect();
    let before = pts.len();
    let mut k = 0usize;
    pts.retain(|_| {
        let r = !drop[k];
        k += 1;
        r
    });
    before - pts.len()
}

/// One layer per sheet where scans of different trust overlap. `priority` lists scan ids from
/// the most trusted down; scans not listed share the lowest rank. A point is dropped when, within
/// `radius` and within `tol` of its tangent plane, at least 3 same-facing points of a better
/// ranked scan lie and they are at least half as many as the points of its own rank there.
/// Equal ranks are kept and averaged by the solve. This removes, for example, the sparse,
/// slightly offset strips that an inside scan catches of the outer face where the dense outside
/// scan covers it (they would otherwise be averaged in as blobs).
pub fn drop_covered_layers(pts: &mut Vec<WorkPoint>, priority: &[u8], radius: f32, tol: f32) -> usize {
    if priority.is_empty() {
        return 0;
    }
    let rank = |s: u8| priority.iter().position(|&p| p == s).unwrap_or(priority.len());
    let pos: Vec<Vec3> = pts.iter().map(|p| p.p).collect();
    let ranks: Vec<usize> = pts.iter().map(|p| rank(p.scan)).collect();
    let grid = PointGrid::new(&pos, radius.max(0.25));
    let drop: Vec<bool> = (0..pts.len())
        .into_par_iter()
        .map(|i| {
            let p = &pts[i];
            let ri = ranks[i];
            if ri == 0 || p.kind != PointKind::Measured {
                return false;
            }
            let (mut better, mut own) = (0usize, 0usize);
            grid.for_each_within(&pos, p.p, radius, |j, _| {
                let q = &pts[j as usize];
                if q.kind != PointKind::Measured || q.n.dot(p.n) <= 0.7 || (q.p - p.p).dot(p.n).abs() > tol {
                    return;
                }
                let rj = ranks[j as usize];
                if rj < ri {
                    better += 1;
                } else if rj == ri {
                    own += 1;
                }
            });
            better >= 3 && 2 * better >= own
        })
        .collect();
    let before = pts.len();
    let mut k = 0usize;
    pts.retain(|_| {
        let r = !drop[k];
        k += 1;
        r
    });
    before - pts.len()
}

/// Depth along −n of the nearest opposite-facing sheet behind `p`, searched in a cylinder of
/// radius `r_cyl` between `dmin` and `dmax`. Returns the median depth of the hits within 0.4 mm
/// of the nearest hit, so a single stray point does not decide.
pub fn opposite_depth(
    p: Vec3,
    n: Vec3,
    pts: &[WorkPoint],
    pos: &[Vec3],
    grid: &PointGrid,
    r_cyl: f32,
    dmin: f32,
    dmax: f32,
    dot_max: f32,
) -> Option<f32> {
    let mid = p - n * (0.5 * (dmin + dmax));
    let rq = (0.25 * (dmax - dmin).powi(2) + r_cyl * r_cyl).sqrt();
    let mut hits: Vec<f32> = Vec::new();
    grid.for_each_within(pos, mid, rq, |i, _| {
        let q = &pts[i as usize];
        if q.n.dot(n) > dot_max {
            return;
        }
        let d = (p - q.p).dot(n);
        if d < dmin || d > dmax {
            return;
        }
        let t2 = (p - q.p).length_squared() - d * d;
        if t2 <= r_cyl * r_cyl {
            hits.push(d);
        }
    });
    if hits.is_empty() {
        return None;
    }
    let dn = hits.iter().copied().fold(f32::MAX, f32::min);
    let mut near: Vec<f32> = hits.into_iter().filter(|d| *d <= dn + 0.4).collect();
    let m = near.len() / 2;
    near.select_nth_unstable_by(m, |a, b| a.total_cmp(b));
    Some(near[m])
}

/// Smooth wall thickness map: medians of measured thickness in 5 mm voxels (stored at the
/// mid-wall position), blended with Gaussian weights and clamped around the global median.
pub struct ThicknessMap {
    voxel: f32,
    cells: FxMap<u64, (f32, u32)>,
    pub global_median: f32,
    pub measurements: usize,
    pub lo: f32,
    pub hi: f32,
}

impl ThicknessMap {
    /// Measures the thickness at a subsample of the points (one per `sample_mm` cell).
    pub fn build(pts: &[WorkPoint], pos: &[Vec3], grid: &PointGrid, max_t: f32, default_t: Option<f32>) -> Self {
        let sub = PointGrid::new(pos, 0.75).representatives();
        let meas: Vec<(Vec3, f32)> = sub
            .par_iter()
            .filter_map(|&i| {
                let p = &pts[i as usize];
                if p.kind != PointKind::Measured {
                    return None;
                }
                opposite_depth(p.p, p.n, pts, pos, grid, 0.6, 0.8, max_t, -0.85).map(|t| (p.p - p.n * (0.5 * t), t))
            })
            .collect();
        let mut all: Vec<f32> = meas.iter().map(|m| m.1).collect();
        let global = if let Some(t) = default_t {
            t
        } else if all.is_empty() {
            3.0
        } else {
            let m = all.len() / 2;
            all.select_nth_unstable_by(m, |a, b| a.total_cmp(b));
            all[m]
        };
        // Ribs and thin plates measure their own thickness; keep the wall-like measurements.
        let (lo, hi) = (0.85 * global, 1.15 * global);
        let voxel = 5.0f32;
        let mut buckets: FxMap<u64, Vec<f32>> = FxMap::default();
        for (c, t) in &meas {
            if *t < 0.7 * global || *t > 1.4 * global {
                continue;
            }
            buckets.entry(cell_key((*c / voxel).floor().as_ivec3())).or_default().push(*t);
        }
        let cells = buckets
            .into_iter()
            .filter(|(_, v)| v.len() >= 3)
            .map(|(k, mut v)| {
                let m = v.len() / 2;
                v.select_nth_unstable_by(m, |a, b| a.total_cmp(b));
                (k, (v[m], v.len() as u32))
            })
            .collect();
        Self { voxel, cells, global_median: global, measurements: meas.len(), lo, hi }
    }

    /// Thickness estimate at `x` (mid-wall position or a surface point).
    pub fn at(&self, x: Vec3) -> f32 {
        let c = (x / self.voxel).floor().as_ivec3();
        let sigma = 8.0f32;
        let (mut sw, mut st) = (0f32, 0f32);
        for r in [3i32, 6] {
            // The wider pass sums the whole 13³ neighbourhood again from zero, so the inner 7³
            // cells are not counted twice.
            sw = 0.0;
            st = 0.0;
            for dz in -r..=r {
                for dy in -r..=r {
                    for dx in -r..=r {
                        let cc = c + IVec3::new(dx, dy, dz);
                        if let Some(&(t, n)) = self.cells.get(&cell_key(cc)) {
                            let centre = (cc.as_vec3() + Vec3::splat(0.5)) * self.voxel;
                            let d2 = centre.distance_squared(x);
                            let w = (-d2 / (2.0 * sigma * sigma)).exp() * (n.min(20) as f32);
                            sw += w;
                            st += w * t;
                        }
                    }
                }
            }
            if sw > 1.0 {
                break;
            }
        }
        // Blend towards the global median where the map is thin.
        let prior = 0.5f32;
        ((st + prior * self.global_median) / (sw + prior)).clamp(self.lo, self.hi)
    }
}

/// Drops laser-line strays behind a surface (cross-line laser scans): lines of points about a
/// millimetre *behind* the surface the same scan sees in front of them, inside the wall, with an
/// empty gap between. A point goes when at least 6 points of its own scan lie `min_mm`..`max_mm`
/// in front of it, measured along their own normals, within `radius` across and centred over it
/// (the surface it is behind; the top of a step beside a floor point is off to one side), 1.3
/// times as many as in its own layer there (seen from the surface the strays are a sheet in front
/// too, but a sparser one: lines), and no point of its scan lies in the gap: within 0.6 mm of the line from it towards
/// that surface, 0.25 mm from it up to 0.2 mm short of the surface. Along a line the strays are as
/// dense as the surface, so density tests miss them. Points behind a face near a convex edge
/// (the other face of the edge) have their own face running through that gap and stay. The other
/// face of a thin wall seen from both sides lies behind nearly all of the face in front, not in
/// lines, and stays too ([`behind_layer_is_a_face`]), however much sparser it is sampled.
pub fn drop_behind_strays(pts: &mut Vec<WorkPoint>, radius: f32, min_mm: f32, max_mm: f32) -> usize {
    let pos: Vec<Vec3> = pts.iter().map(|p| p.p).collect();
    let grid = PointGrid::new(&pos, radius.max(0.5));
    let reach = (radius * radius + max_mm * max_mm).sqrt();
    let drop: Vec<bool> = (0..pts.len())
        .into_par_iter()
        .map(|i| {
            let p = &pts[i];
            if p.kind != PointKind::Measured {
                return false;
            }
            let (mut f, mut hs, mut c, mut k) = (Vec3::ZERO, 0f32, Vec3::ZERO, 0usize);
            grid.for_each_within(&pos, p.p, reach, |j, _| {
                let q = &pts[j as usize];
                if j as usize == i || q.scan != p.scan || q.kind != PointKind::Measured {
                    return;
                }
                let d = q.p - p.p;
                let h = d.dot(q.n);
                if h >= min_mm && h <= max_mm && (d - q.n * h).length_squared() <= radius * radius {
                    f += q.n;
                    hs += h;
                    c += d;
                    k += 1;
                }
            });
            if k < 6 {
                return false;
            }
            let f = f.normalize_or_zero();
            let h = hs / k as f32;
            let (t0, t1) = (0.25f32, h - 0.2);
            // The surface lies all round above it, not off to one side (the top of a step beside
            // the floor a point is on).
            let c = c / k as f32;
            if f == Vec3::ZERO || t1 <= t0 || (c - f * c.dot(f)).length() > 0.4 * radius {
                return false;
            }
            // The sheet in front is a surface, the point's own layer only lines: within `radius`
            // across, the sheet in front has clearly more points than the layer through the point
            // (seen from the surface, the strays behind it are such a "sheet in front" too, but a
            // sparser one).
            let mut own = 0usize;
            grid.for_each_within(&pos, p.p, reach, |j, _| {
                let q = &pts[j as usize];
                if q.scan == p.scan && q.kind == PointKind::Measured {
                    let d = q.p - p.p;
                    let t = d.dot(f);
                    if t.abs() < 0.25 && (d - f * t).length_squared() <= radius * radius {
                        own += 1;
                    }
                }
            });
            if (k as f32) < 1.3 * own as f32 {
                return false;
            }
            let mid = p.p + f * (0.5 * (t0 + t1));
            let mut blocked = 0usize;
            grid.for_each_within(&pos, mid, 0.5 * (t1 - t0) + 0.6, |j, _| {
                let q = &pts[j as usize];
                if q.scan != p.scan || q.kind != PointKind::Measured {
                    return;
                }
                let d = q.p - p.p;
                let t = d.dot(f);
                if t >= t0 && t <= t1 && (d - f * t).length_squared() < 0.36 {
                    blocked += 1;
                }
            });
            blocked <= 1 && !behind_layer_is_a_face(p, i, f, &pts[..], &pos, &grid, min_mm, max_mm, radius)
        })
        .collect();
    let before = pts.len();
    let mut k = 0usize;
    pts.retain(|_| {
        let r = !drop[k];
        k += 1;
        r
    });
    before - pts.len()
}

/// Whether the layer through `p` (point `i`), behind the surface facing `f` in front of it, is a
/// face of its own: the other face of a thin wall (a rib, a plate, the wall of a boss) that the
/// scan sees from both sides. Such a face passes every test of [`drop_behind_strays`] whenever it
/// is sampled more sparsely than the face in front (a grazing view), but it is a surface: in
/// nearly every place across where the face in front is, it lies behind it too. Laser strays are
/// lines and lie behind only part of it. Within `2 * radius` across, on 0.6 mm cells: the cells
/// where the same scan has surface facing like `f` (its outermost such point, so curvature does not
/// matter), and among them those with a point `min_mm - 0.2 .. max_mm + 0.4` behind that surface.
#[allow(clippy::too_many_arguments)]
fn behind_layer_is_a_face(
    p: &WorkPoint,
    i: usize,
    f: Vec3,
    pts: &[WorkPoint],
    pos: &[Vec3],
    grid: &PointGrid,
    min_mm: f32,
    max_mm: f32,
    radius: f32,
) -> bool {
    const CELL: f32 = 0.6;
    const FACE_SHARE: f32 = 0.7;
    let r = 2.0 * radius;
    let nc = (r / CELL).ceil() as i32;
    let side = (2 * nc + 1) as usize;
    let (eu, ev) = super::markers::tangent_frame(f);
    let reach = (r * r + (max_mm + 2.0).powi(2)).sqrt();
    // Per cell: the outermost point facing like `f`, and every point's height (along `f`).
    let mut front = vec![f32::NEG_INFINITY; side * side];
    let mut heights: Vec<(usize, f32)> = Vec::new();
    grid.for_each_within(pos, p.p, reach, |j, _| {
        let q = &pts[j as usize];
        if q.scan != p.scan || q.kind != PointKind::Measured {
            return;
        }
        let d = q.p - p.p;
        let (a, b) = (d.dot(eu), d.dot(ev));
        if a * a + b * b > r * r {
            return;
        }
        let k = ((b / CELL).round() as i32 + nc) as usize * side + ((a / CELL).round() as i32 + nc) as usize;
        let t = d.dot(f);
        if q.n.dot(f) >= 0.5 && j as usize != i {
            front[k] = front[k].max(t);
        }
        heights.push((k, t));
    });
    let mut behind = vec![false; side * side];
    for &(k, t) in &heights {
        let depth = front[k] - t;
        if depth >= min_mm - 0.2 && depth <= max_mm + 0.4 {
            behind[k] = true;
        }
    }
    // Only the cells whose surface is the one in front of `p` (not a far-off sheet).
    let (mut cells, mut covered) = (0usize, 0usize);
    for k in 0..side * side {
        if front[k] > -1.5 && front[k] < max_mm + 1.5 {
            cells += 1;
            covered += behind[k] as usize;
        }
    }
    cells >= 8 && covered as f32 >= FACE_SHARE * cells as f32
}

/// Drops the less plausible layer where a sheet is doubled (ghost layers of misregistered
/// passes): a point with a same-facing layer 0.6–2.5 mm in front of or behind it is removed when
/// its own wall thickness deviates from the map clearly more than the other layer's would.
pub fn drop_ghost_layers(pts: &mut Vec<WorkPoint>, map: &ThicknessMap, max_t: f32) -> usize {
    let pos: Vec<Vec3> = pts.iter().map(|p| p.p).collect();
    let grid = PointGrid::new(&pos, 1.0);
    let drop: Vec<bool> = (0..pts.len())
        .into_par_iter()
        .map(|i| {
            let p = &pts[i];
            if p.kind != PointKind::Measured {
                return false;
            }
            let mut offs: Vec<f32> = Vec::new();
            grid.for_each_within(&pos, p.p, 2.6, |j, _| {
                let q = &pts[j as usize];
                if q.kind != PointKind::Measured || q.n.dot(p.n) < 0.8 {
                    return;
                }
                let d = (q.p - p.p).dot(p.n);
                if d.abs() < 0.6 || d.abs() > 2.5 {
                    return;
                }
                let t2 = (q.p - p.p).length_squared() - d * d;
                if t2 <= 0.7 * 0.7 {
                    offs.push(d);
                }
            });
            if offs.len() < 3 {
                return false;
            }
            let m = offs.len() / 2;
            offs.select_nth_unstable_by(m, |a, b| a.total_cmp(b));
            let d_other = offs[m];
            let Some(t) = opposite_depth(p.p, p.n, pts, &pos, &grid, 0.6, 0.3, max_t, -0.7) else {
                return false;
            };
            let t_other = t + d_other;
            let th = map.at(p.p - p.n * (0.5 * t));
            let (e_own, e_other) = ((t - th).abs(), (t_other - th).abs());
            e_own > 0.4 && e_own > e_other + 0.25
        })
        .collect();
    let before = pts.len();
    let mut k = 0usize;
    pts.retain(|_| {
        let r = !drop[k];
        k += 1;
        r
    });
    before - pts.len()
}

#[derive(Clone, Debug)]
pub struct CompletionConfig {
    /// Candidate spacing (one candidate per cell of this size).
    pub spacing_mm: f32,
    /// The other side counts as present when an opposite-facing point lies within this radius of
    /// the −n axis behind the candidate.
    pub gap_radius_mm: f32,
    /// Depth behind the wall to which the "material continues" test reaches (bosses, ribs).
    pub reach_mm: f32,
}

impl Default for CompletionConfig {
    fn default() -> Self {
        Self { spacing_mm: 0.45, gap_radius_mm: 2.0, reach_mm: 20.0 }
    }
}

/// Offset completion: wherever a sheet has no opposite sheet behind it within the wall
/// thickness (plus margin), a synthetic point is placed one local wall thickness behind it, with
/// the opposite normal. Candidates near edges (perpendicular normals within the slab) and where
/// any opposite-facing surface lies further along the ray (a boss or rib continues the material)
/// are skipped.
pub fn completion_points(pts: &[WorkPoint], map: &ThicknessMap, prm: &CompletionConfig) -> Vec<WorkPoint> {
    let pos: Vec<Vec3> = pts.iter().map(|p| p.p).collect();
    let grid = PointGrid::new(&pos, 1.0);
    let cand = PointGrid::new(&pos, prm.spacing_mm).representatives();
    cand.par_iter()
        .filter_map(|&ci| {
            let p = &pts[ci as usize];
            if p.kind == PointKind::Completion {
                return None;
            }
            let t = map.at(p.p - p.n * (0.5 * map.global_median));
            // Opposite sheet present (wall, rib or plate).
            if opposite_depth(p.p, p.n, pts, &pos, &grid, prm.gap_radius_mm, 0.3, t + 1.2, -0.3).is_some() {
                return None;
            }
            // Edges inside the slab (rim lip, rib or boss walls meeting the wall).
            let mut edge = false;
            let mid = p.p - p.n * (0.5 * (t + 0.5));
            let rq = (0.25 * (t + 0.5).powi(2) + 2.0f32.powi(2)).sqrt();
            grid.for_each_within(&pos, mid, rq, |j, _| {
                if edge {
                    return;
                }
                let q = &pts[j as usize];
                if q.kind == PointKind::Completion || q.n.dot(p.n).abs() > 0.5 {
                    return;
                }
                let d = (p.p - q.p).dot(p.n);
                if d < 0.4 || d > t + 0.9 {
                    return;
                }
                let t2 = (p.p - q.p).length_squared() - d * d;
                if t2 <= 4.0 {
                    edge = true;
                }
            });
            if edge {
                return None;
            }
            // Material continues further behind (boss or rib under the missing side).
            let mut dd = t + 1.2;
            while dd < prm.reach_mm {
                let c = p.p - p.n * (dd + 0.75);
                let mut hit = false;
                grid.for_each_within(&pos, c, 1.0, |j, _| {
                    if !hit && pts[j as usize].kind != PointKind::Completion && pts[j as usize].n.dot(p.n) < 0.0 {
                        hit = true;
                    }
                });
                if hit {
                    return None;
                }
                dd += 1.5;
            }
            Some(WorkPoint { p: p.p - p.n * t, n: -p.n, scan: p.scan, kind: PointKind::Completion, area: 0.0 })
        })
        .collect()
}

/// Offset samples between scans, as (scan, other scan, offset along the scan's own normals in mm).
/// One point per 0.75 mm cell of each scan: a point counts when at least 5 same-facing points of other scans lie
/// within 1 mm of it and within 0.6 mm of their mean plane; its offset is its height above that
/// plane, the other scan the one most of those points belong to.
fn offset_samples(pts: &[WorkPoint]) -> Vec<(u8, u8, f32)> {
    let pos: Vec<Vec3> = pts.iter().map(|p| p.p).collect();
    let grid = PointGrid::new(&pos, 1.0);
    // One point per cell of each scan (a common subsample would split the cells of an overlap
    // between the scans).
    let mut sub: Vec<u32> = Vec::new();
    let scans: std::collections::BTreeSet<u8> = pts.iter().map(|p| p.scan).collect();
    for s in scans {
        let idx: Vec<u32> = (0..pts.len() as u32).filter(|&i| pts[i as usize].scan == s && pts[i as usize].kind == PointKind::Measured).collect();
        let sp: Vec<Vec3> = idx.iter().map(|&i| pos[i as usize]).collect();
        sub.extend(PointGrid::new(&sp, 0.75).representatives().into_iter().map(|k| idx[k as usize]));
    }
    sub.par_iter()
        .filter_map(|&i| {
            let p = &pts[i as usize];
            let (mut c, mut nm, mut k) = (Vec3::ZERO, Vec3::ZERO, 0usize);
            let mut by: [(u8, u32); 4] = [(0, 0); 4];
            grid.for_each_within(&pos, p.p, 1.0, |j, _| {
                let q = &pts[j as usize];
                if q.scan != p.scan && q.kind == PointKind::Measured && q.n.dot(p.n) > 0.8 && (q.p - p.p).dot(p.n).abs() < 0.6 {
                    c += q.p;
                    nm += q.n;
                    k += 1;
                    if let Some(e) = by.iter_mut().find(|e| e.1 == 0 || e.0 == q.scan) {
                        *e = (q.scan, e.1 + 1);
                    }
                }
            });
            if k < 5 {
                return None;
            }
            let nm = nm.normalize_or_zero();
            let other = by.iter().max_by_key(|e| e.1).map_or(0, |e| e.0);
            (nm != Vec3::ZERO).then(|| (p.scan, other, (p.p - c / k as f32).dot(nm)))
        })
        .collect()
}

/// Median offset per key of `samples` and the number of samples.
fn medians<K: Ord>(samples: impl Iterator<Item = (K, f32)>) -> Vec<(K, f32, usize)> {
    let mut by: std::collections::BTreeMap<K, Vec<f32>> = std::collections::BTreeMap::new();
    for (k, d) in samples {
        by.entry(k).or_default().push(d);
    }
    by.into_iter()
        .map(|(k, mut v)| {
            let m = v.len() / 2;
            v.select_nth_unstable_by(m, |a, b| a.total_cmp(b));
            (k, v[m], v.len())
        })
        .collect()
}

/// Signed offset of each scan's surface from the other scans' where they overlap, as (scan, median
/// offset along the scan's own normals in mm, samples); see `offset_samples`. For the tests.
#[cfg(test)]
pub fn scan_offsets(pts: &[WorkPoint]) -> Vec<(u8, f32, usize)> {
    medians(offset_samples(pts).into_iter().map(|(s, _, d)| (s, d)))
}

/// Removes the systematic offset between scans along their normals (a scan whose surface sits a
/// tenth of a millimetre above the others everywhere it overlaps them: a depth bias of that pass).
/// Averaged, such offsets leave steps where the coverage changes from one scan to another, and a
/// marker patch fitted to one scan sits low or high against the average.
///
/// Per pair of overlapping scans the median offset is measured (`offset_samples`; pairs with at
/// least `min_samples`, offsets beyond `max_mm` are a misalignment, not a bias, and are left
/// alone). A scan is moved when it lies above (or below) every scan it overlaps by more than
/// `tolerance_mm`: the one with the most such partners first, then the one with the fewest points
/// (an added pass rather than the main one), then the largest offset. It is moved back along its own normals
/// by its median offset against all of them. Moved scans are references from then on: a scan
/// whose partners have all been moved stays, so a correction is never passed along a chain of
/// scans (the outside scan does not follow an inside scan that meets it at the rim). Returns the
/// shift applied per scan (mm).
pub fn equalise_scan_offsets(pts: &mut [WorkPoint], tolerance_mm: f32, max_mm: f32, min_samples: usize) -> Vec<(u8, f32)> {
    let mut applied: Vec<(u8, f32)> = Vec::new();
    let mut counts: std::collections::BTreeMap<u8, usize> = std::collections::BTreeMap::new();
    for p in pts.iter().filter(|p| p.kind == PointKind::Measured) {
        *counts.entry(p.scan).or_default() += 1;
    }
    for _ in 0..4 {
        let samples = offset_samples(pts);
        let pooled = medians(samples.iter().map(|&(s, _, d)| (s, d)));
        let pairs: Vec<((u8, u8), f32, usize)> =
            medians(samples.iter().map(|&(s, o, d)| ((s, o), d))).into_iter().filter(|p| p.2 >= min_samples && p.1.abs() <= max_mm).collect();
        let moved = |s: u8| applied.iter().any(|a| a.0 == s);
        // (scan, partners, offset)
        let pick = counts
            .keys()
            .copied()
            .filter(|&s| !moved(s))
            .filter_map(|s| {
                let mine: Vec<&((u8, u8), f32, usize)> = pairs.iter().filter(|p| p.0 .0 == s).collect();
                if mine.is_empty() || mine.iter().all(|p| moved(p.0 .1)) {
                    return None;
                }
                let sign = mine[0].1.signum();
                if !mine.iter().all(|p| p.1.abs() > tolerance_mm && p.1.signum() == sign) {
                    return None;
                }
                let off = pooled.iter().find(|p| p.0 == s)?.1;
                (off.abs() > tolerance_mm && off.abs() <= max_mm).then_some((s, mine.len(), off))
            })
            .max_by(|a, b| a.1.cmp(&b.1).then(counts[&b.0].cmp(&counts[&a.0])).then(a.2.abs().total_cmp(&b.2.abs())));
        let Some((scan, _, off)) = pick else { break };
        for p in pts.iter_mut().filter(|p| p.scan == scan && p.kind == PointKind::Measured) {
            p.p -= p.n * off;
        }
        applied.push((scan, -off));
    }
    applied
}

/// Drops a scan's points where another scan covers the same sheet much more smoothly: the grazing
/// fringe an inside scan catches of the outer face, streaked along its laser lines, where the
/// outside scan sees that face head on. Per point, the roughness of its own scan around it (RMS
/// distance of the same-scan, same-facing points within `radius` from their plane); a point goes
/// when its roughness is above `floor_mm` and at least 6 same-facing points of one other scan lie
/// within `radius` and 0.6 mm of its tangent plane whose median roughness is at most `ratio` times
/// its own. Overlaps of similar quality are kept and averaged by the solve.
pub fn drop_rough_layers(pts: &mut Vec<WorkPoint>, radius: f32, ratio: f32, floor_mm: f32) -> usize {
    let pos: Vec<Vec3> = pts.iter().map(|p| p.p).collect();
    let grid = PointGrid::new(&pos, radius.max(0.25));
    let rough: Vec<f32> = (0..pts.len())
        .into_par_iter()
        .map(|i| {
            let p = &pts[i];
            if p.kind != PointKind::Measured {
                return f32::NAN;
            }
            let (mut c, mut nm, mut k) = (Vec3::ZERO, Vec3::ZERO, 0usize);
            let mut nb: [Vec3; 96] = [Vec3::ZERO; 96];
            grid.for_each_within(&pos, p.p, radius, |j, _| {
                let q = &pts[j as usize];
                if q.scan == p.scan && q.kind == PointKind::Measured && q.n.dot(p.n) > 0.7 {
                    c += q.p;
                    nm += q.n;
                    if k < 96 {
                        nb[k] = q.p;
                    }
                    k += 1;
                }
            });
            if k < 8 {
                return f32::NAN;
            }
            let (c, nm) = (c / k as f32, nm.normalize_or_zero());
            let m = k.min(96);
            (nb[..m].iter().map(|q| (*q - c).dot(nm).powi(2)).sum::<f32>() / m as f32).sqrt()
        })
        .collect();
    let drop: Vec<bool> = (0..pts.len())
        .into_par_iter()
        .map(|i| {
            let p = &pts[i];
            let own = rough[i];
            if !(own > floor_mm) {
                return false;
            }
            // Per other scan: roughness values of its points on this sheet here.
            let mut others: Vec<(u8, f32)> = Vec::new();
            grid.for_each_within(&pos, p.p, radius, |j, _| {
                let q = &pts[j as usize];
                if q.scan != p.scan && q.kind == PointKind::Measured && q.n.dot(p.n) > 0.7 && (q.p - p.p).dot(p.n).abs() < 0.6 && rough[j as usize].is_finite() {
                    others.push((q.scan, rough[j as usize]));
                }
            });
            others.sort_by(|a, b| a.0.cmp(&b.0).then(a.1.total_cmp(&b.1)));
            others.chunk_by(|a, b| a.0 == b.0).any(|g| g.len() >= 6 && g[g.len() / 2].1 <= ratio * own)
        })
        .collect();
    let before = pts.len();
    let mut k = 0usize;
    pts.retain(|_| {
        let r = !drop[k];
        k += 1;
        r
    });
    before - pts.len()
}

/// Area per point: π R² divided by the number of same-facing points within R (points of
/// overlapping passes share the area, so the passes are averaged rather than summed).
pub fn area_weights(pts: &mut [WorkPoint], radius: f32) {
    let pos: Vec<Vec3> = pts.iter().map(|p| p.p).collect();
    let normals: Vec<Vec3> = pts.iter().map(|p| p.n).collect();
    let grid = PointGrid::new(&pos, radius);
    let disc = std::f32::consts::PI * radius * radius;
    let r2 = radius * radius;
    let areas: Vec<f32> = (0..pts.len())
        .into_par_iter()
        .map(|i| {
            let (p, n) = (pos[i], normals[i]);
            let mut w = 0f32;
            grid.for_each_near(p, radius, |j| {
                let d2 = pos[j as usize].distance_squared(p);
                if d2 <= r2 && normals[j as usize].dot(n) > 0.5 {
                    // Smooth falloff so the count does not jump with quantised coordinates.
                    w += 1.0 - 0.5 * d2 / r2;
                }
            });
            // The kernel integrates to 0.75 of the disc over a uniform sampling.
            (0.75 * disc / w.max(1.0)).min(disc)
        })
        .collect();
    for (p, a) in pts.iter_mut().zip(areas) {
        p.area = a;
    }
}
