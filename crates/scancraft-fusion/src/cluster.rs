//! Grid clustering helpers shared by object isolation ([`crate::segment`]) and turntable capture:
//! connected components of occupied grid cells, and the spread of a point set along its
//! principal axes.

use glam::{DVec3, Vec3};
use std::collections::{HashMap, VecDeque};

/// Connected components found by [`cell_components`].
#[derive(Debug, Clone, Default, PartialEq)]
pub struct CellComponents<const D: usize> {
    /// Component id of every cell that took part.
    pub label: HashMap<[i32; D], usize>,
    /// Summed occupancy (points) per component id.
    pub sizes: Vec<usize>,
}

/// Coordinates beyond this (mm) are taken as invalid-point markers (some files write FLT_MAX),
/// not as surface: they would overflow the grid keys.
pub const MAX_COORD_MM: f32 = 1e7;

/// A finite point within [`MAX_COORD_MM`] of the origin on every axis.
pub fn usable(p: Vec3) -> bool {
    p.is_finite() && p.abs().max_element() <= MAX_COORD_MM
}

/// Grid cell of `p` for a cubic grid of `cell` mm. Coordinates far outside the `i32` range
/// saturate.
pub fn voxel_key(p: Vec3, cell: f32) -> [i32; 3] {
    [(p.x / cell).floor() as i32, (p.y / cell).floor() as i32, (p.z / cell).floor() as i32]
}

/// All offsets of a (2 `reach` + 1)^D neighbourhood, the centre included.
fn offsets<const D: usize>(reach: i32) -> Vec<[i32; D]> {
    let side = (2 * reach + 1) as usize;
    (0..side.pow(D as u32))
        .map(|mut k| {
            let mut o = [0i32; D];
            for v in o.iter_mut() {
                *v = (k % side) as i32 - reach;
                k /= side;
            }
            o
        })
        .collect()
}

/// Connected components of the cells holding at least `min_count` points. Cells up to `reach`
/// cells apart in every axis are neighbours (1: touching cells; 2 also bridges a one-cell gap).
/// Ids follow the sorted order of each component's first cell, so the result is deterministic.
pub fn cell_components<const D: usize>(occupancy: &HashMap<[i32; D], usize>, reach: i32, min_count: usize) -> CellComponents<D> {
    let neighbourhood = offsets::<D>(reach.max(0));
    let mut starts: Vec<&[i32; D]> = occupancy.iter().filter(|(_, &n)| n >= min_count).map(|(k, _)| k).collect();
    starts.sort_unstable();
    let mut out = CellComponents { label: HashMap::with_capacity(starts.len()), sizes: Vec::new() };
    for &start in starts {
        if out.label.contains_key(&start) {
            continue;
        }
        let id = out.sizes.len();
        out.sizes.push(0);
        out.label.insert(start, id);
        let mut queue = VecDeque::from([start]);
        while let Some(c) = queue.pop_front() {
            out.sizes[id] += occupancy[&c];
            for o in &neighbourhood {
                let mut n = c;
                for (v, d) in n.iter_mut().zip(o) {
                    *v = v.saturating_add(*d);
                }
                if occupancy.get(&n).copied().unwrap_or(0) >= min_count && !out.label.contains_key(&n) {
                    out.label.insert(n, id);
                    queue.push_back(n);
                }
            }
        }
    }
    out
}

/// Standard deviations (mm) of the points along their principal axes, largest first.
pub fn principal_spread(points: impl Iterator<Item = Vec3> + Clone) -> [f32; 3] {
    let (mut sum, mut n) = (DVec3::ZERO, 0usize);
    for p in points.clone() {
        sum += p.as_dvec3();
        n += 1;
    }
    if n == 0 {
        return [0.0; 3];
    }
    let c = sum / n as f64;
    let mut m = [[0.0f64; 3]; 3];
    for p in points {
        let d = (p.as_dvec3() - c).to_array();
        for i in 0..3 {
            for j in 0..3 {
                m[i][j] += d[i] * d[j];
            }
        }
    }
    for row in m.iter_mut() {
        for v in row.iter_mut() {
            *v /= n as f64;
        }
    }
    symmetric_eigenvalues(m).map(|v| v.max(0.0).sqrt() as f32)
}

/// Eigenvalues of a symmetric 3x3 matrix, largest first (closed form).
fn symmetric_eigenvalues(a: [[f64; 3]; 3]) -> [f64; 3] {
    let p1 = a[0][1] * a[0][1] + a[0][2] * a[0][2] + a[1][2] * a[1][2];
    if p1 <= 1e-30 {
        let mut d = [a[0][0], a[1][1], a[2][2]];
        d.sort_by(|x, y| y.total_cmp(x));
        return d;
    }
    let q = (a[0][0] + a[1][1] + a[2][2]) / 3.0;
    let p2 = (a[0][0] - q).powi(2) + (a[1][1] - q).powi(2) + (a[2][2] - q).powi(2) + 2.0 * p1;
    let p = (p2 / 6.0).sqrt();
    let b = |i: usize, j: usize| (a[i][j] - if i == j { q } else { 0.0 }) / p;
    let det = b(0, 0) * (b(1, 1) * b(2, 2) - b(1, 2) * b(2, 1)) - b(0, 1) * (b(1, 0) * b(2, 2) - b(1, 2) * b(2, 0))
        + b(0, 2) * (b(1, 0) * b(2, 1) - b(1, 1) * b(2, 0));
    let phi = (det / 2.0).clamp(-1.0, 1.0).acos() / 3.0;
    let largest = q + 2.0 * p * phi.cos();
    let smallest = q + 2.0 * p * (phi + 2.0 * std::f64::consts::FRAC_PI_3).cos();
    [largest, 3.0 * q - largest - smallest, smallest]
}

#[cfg(test)]
mod tests {
    use super::*;

    fn occupancy2(cells: &[[i32; 2]]) -> HashMap<[i32; 2], usize> {
        cells.iter().map(|c| (*c, 2)).collect()
    }

    #[test]
    fn components_respect_reach() {
        // Two cells with a one-cell gap, and one far away.
        let occ = occupancy2(&[[0, 0], [2, 0], [10, 10]]);
        let touching = cell_components(&occ, 1, 1);
        assert_eq!(touching.sizes.len(), 3);
        let bridged = cell_components(&occ, 2, 1);
        assert_eq!(bridged.sizes.len(), 2);
        assert_eq!(bridged.label[&[0, 0]], bridged.label[&[2, 0]]);
        assert_eq!(bridged.sizes[bridged.label[&[0, 0]]], 4);
    }

    #[test]
    fn components_skip_sparse_cells() {
        let mut occ = occupancy2(&[[0, 0], [2, 0]]);
        occ.insert([1, 0], 1); // single point: neither a component nor a bridge
        let c = cell_components(&occ, 1, 2);
        assert_eq!(c.sizes, vec![2, 2]);
        assert!(!c.label.contains_key(&[1, 0]));
    }

    #[test]
    fn components_in_3d_are_deterministic() {
        let occ: HashMap<[i32; 3], usize> = [[5, 5, 5], [6, 6, 6], [0, 0, 0]].iter().map(|c| (*c, 1)).collect();
        let c = cell_components(&occ, 1, 1);
        assert_eq!(c.label[&[0, 0, 0]], 0, "ids follow the sorted cell order");
        assert_eq!(c.label[&[5, 5, 5]], c.label[&[6, 6, 6]]);
    }

    #[test]
    fn huge_coordinates_neither_overflow_nor_count_as_usable() {
        let far = Vec3::splat(f32::MAX);
        assert!(!usable(far) && !usable(Vec3::NAN) && usable(Vec3::new(1e6, -2e6, 0.0)));
        let key = voxel_key(far, 2.0);
        assert_eq!(key, [i32::MAX; 3]);
        let occ: HashMap<[i32; 3], usize> = [(key, 3), ([0, 0, 0], 3)].into_iter().collect();
        let c = cell_components(&occ, 1, 1);
        assert_eq!(c.sizes, vec![3, 3]);
    }

    #[test]
    fn spread_of_a_flat_patch() {
        let pts: Vec<Vec3> = (0..20).flat_map(|i| (0..10).map(move |j| Vec3::new(i as f32, j as f32 * 0.5, 0.0))).collect();
        let pts: Vec<Vec3> = pts.iter().map(|p| glam::Quat::from_rotation_y(0.7) * *p).collect();
        let sd = principal_spread(pts.iter().copied());
        assert!((sd[0] - 5.766).abs() < 0.01, "{sd:?}"); // sqrt((20^2 - 1) / 12)
        assert!((sd[1] - 1.436).abs() < 0.01, "{sd:?}");
        assert!(sd[2] < 1e-3, "{sd:?}");
    }
}
