//! Robust point-to-plane ICP (Gauss-Newton on SE(3), small-angle linearisation).

use glam::{DQuat, DVec3, Quat, Vec3};
use kiddo::{ImmutableKdTree, SquaredEuclidean};
use scancraft_core::Transform;

/// Oriented target surface with a nearest-neighbour index.
pub struct PlaneTarget {
    points: Vec<[f32; 3]>,
    normals: Vec<Vec3>,
    tree: ImmutableKdTree<f32, 3>,
}

impl PlaneTarget {
    pub fn new(points: Vec<Vec3>, normals: Vec<Vec3>) -> Self {
        let points: Vec<[f32; 3]> = points.iter().map(|p| p.to_array()).collect();
        // kiddo's immutable tree panics when many points share one coordinate on an axis (e.g. a
        // perfectly flat, axis-aligned surface). Break exact ties with a deterministic jitter of
        // about a micrometre, far below sensor precision; lookups still return the true points.
        let jittered: Vec<[f32; 3]> = points
            .iter()
            .enumerate()
            .map(|(i, p)| {
                let h = (i as u32).wrapping_mul(2654435761);
                let j = |shift: u32| ((h >> shift) & 0xff) as f32 * 4e-6;
                [p[0] + j(0), p[1] + j(8), p[2] + j(16)]
            })
            .collect();
        let tree = ImmutableKdTree::new_from_slice(&jittered);
        Self { points, normals, tree }
    }

    pub fn len(&self) -> usize {
        self.points.len()
    }

    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }

    fn nearest(&self, p: Vec3) -> (f32, usize) {
        let nn = self.tree.nearest_one::<SquaredEuclidean>(&p.to_array());
        (nn.distance, nn.item as usize)
    }
}

#[derive(Debug, Clone, Copy)]
pub struct AlignResult {
    pub pose: Transform,
    /// Source points with a correspondence within the final gate.
    pub inliers: usize,
    /// RMS point-to-plane distance of those inliers, mm.
    pub rmse_mm: f32,
    /// `inliers / source points`.
    pub inlier_ratio: f32,
}

/// Aligns oriented `source` points (sensor frame) to `target` (world frame), starting at `guess`.
/// `gates_mm` is the coarse-to-fine schedule of correspondence distances; each stage runs
/// `iterations` Gauss-Newton steps or until the update is negligible.
pub fn align(source: &[(Vec3, Vec3)], target: &PlaneTarget, guess: Transform, gates_mm: &[f32], iterations: usize) -> AlignResult {
    let mut rot = guess.rotation.as_dquat();
    let mut trans = guess.translation.as_dvec3();
    let empty = AlignResult { pose: guess, inliers: 0, rmse_mm: f32::INFINITY, inlier_ratio: 0.0 };
    if source.is_empty() || target.is_empty() {
        return empty;
    }

    for &gate in gates_mm {
        let gate_sq = gate * gate;
        // Huber threshold: a fraction of the gate, but never below the sensor noise level.
        let huber = (gate as f64 * 0.25).max(0.5);
        for _ in 0..iterations {
            let mut ata = [[0.0f64; 6]; 6];
            let mut atb = [0.0f64; 6];
            let mut used = 0usize;
            for &(p, n_src) in source {
                let pw = rot * p.as_dvec3() + trans;
                let (d_sq, i) = target.nearest(pw.as_vec3());
                if d_sq > gate_sq {
                    continue;
                }
                let n = target.normals[i].as_dvec3();
                // Reject pairs whose surfaces face different ways (wrong side / wrong surface).
                if (rot * n_src.as_dvec3()).dot(n) < 0.5 {
                    continue;
                }
                let q = Vec3::from_array(target.points[i]).as_dvec3();
                let r = n.dot(pw - q);
                let w = if r.abs() <= huber { 1.0 } else { huber / r.abs() };
                let c = pw.cross(n);
                let j = [c.x, c.y, c.z, n.x, n.y, n.z];
                for a in 0..6 {
                    atb[a] -= w * j[a] * r;
                    for b in a..6 {
                        ata[a][b] += w * j[a] * j[b];
                    }
                }
                used += 1;
            }
            if used < 12 {
                return empty;
            }
            // Light Levenberg damping: directions the geometry does not constrain (e.g. sliding
            // along a flat wall) stay where the guess put them instead of making the system singular.
            let trace: f64 = (0..6).map(|a| ata[a][a]).sum();
            for a in 0..6 {
                ata[a][a] += 1e-6 * trace + 1e-9;
                for b in 0..a {
                    ata[a][b] = ata[b][a];
                }
            }
            let Some(x) = solve6(ata, atb) else { return empty };
            let omega = DVec3::new(x[0], x[1], x[2]);
            let dt = DVec3::new(x[3], x[4], x[5]);
            let d_rot = if omega.length() > 1e-12 { DQuat::from_scaled_axis(omega) } else { DQuat::IDENTITY };
            rot = (d_rot * rot).normalize();
            trans = d_rot * trans + dt;
            if omega.length() < 1e-5 && dt.length() < 1e-3 {
                break;
            }
        }
    }

    // Score with the finest gate.
    let gate = gates_mm.last().copied().unwrap_or(1.0);
    let (mut inliers, mut sq) = (0usize, 0.0f64);
    for &(p, _) in source {
        let pw = rot * p.as_dvec3() + trans;
        let (d_sq, i) = target.nearest(pw.as_vec3());
        if d_sq <= gate * gate {
            let r = target.normals[i].as_dvec3().dot(pw - Vec3::from_array(target.points[i]).as_dvec3());
            sq += r * r;
            inliers += 1;
        }
    }
    AlignResult {
        pose: Transform { rotation: Quat::from(rot.as_quat()), translation: trans.as_vec3(), scale: Vec3::ONE },
        inliers,
        rmse_mm: if inliers > 0 { (sq / inliers as f64).sqrt() as f32 } else { f32::INFINITY },
        inlier_ratio: inliers as f32 / source.len() as f32,
    }
}

/// Solves the symmetric 6x6 system by Gaussian elimination with partial pivoting.
fn solve6(mut a: [[f64; 6]; 6], mut b: [f64; 6]) -> Option<[f64; 6]> {
    for col in 0..6 {
        let pivot = (col..6).max_by(|&i, &j| a[i][col].abs().total_cmp(&a[j][col].abs()))?;
        if a[pivot][col].abs() < 1e-9 {
            return None;
        }
        a.swap(col, pivot);
        b.swap(col, pivot);
        for row in col + 1..6 {
            let f = a[row][col] / a[col][col];
            for k in col..6 {
                a[row][k] -= f * a[col][k];
            }
            b[row] -= f * b[col];
        }
    }
    let mut x = [0.0; 6];
    for row in (0..6).rev() {
        let s: f64 = (row + 1..6).map(|k| a[row][k] * x[k]).sum();
        x[row] = (b[row] - s) / a[row][row];
    }
    Some(x)
}
