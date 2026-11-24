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
