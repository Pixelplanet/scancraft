use glam::{DMat3, DVec3, Mat3, Vec3};
use rayon::prelude::*;
use kdtree::distance::squared_euclidean;
use kdtree::KdTree;
use scancraft_core::PointCloud;

pub struct NormalEstimator {
    pub k_neighbors: usize,
    pub viewpoint: Vec3,
}

impl Default for NormalEstimator {
    fn default() -> Self {
        Self {
            k_neighbors: 12,
            viewpoint: Vec3::new(0.0, 0.0, 0.0), // Scanner camera optical center
        }
    }
}

impl NormalEstimator {
    pub fn new(k_neighbors: usize, viewpoint: Vec3) -> Self {
        Self {
            k_neighbors,
            viewpoint,
        }
    }

    /// Estimate and orient normal vectors for each point in the cloud (PCA over the k nearest
    /// neighbours, flipped to face `viewpoint`).
    pub fn estimate_normals(&self, cloud: &mut PointCloud) {
        let n = cloud.len();
        if n < 4 {
            return;
        }

        let mut kdtree = KdTree::with_capacity(3, 64);
        for (i, p) in cloud.points.iter().enumerate() {
            let _ = kdtree.add(p.position, i);
        }

        let k_query = self.k_neighbors.min(n - 1) + 1;
        let positions: Vec<[f32; 3]> = cloud.points.iter().map(|p| p.position).collect();
        let normals: Vec<Option<Vec3>> = positions
            .par_iter()
            .map(|pos| {
                let neighbors = kdtree.nearest(pos, k_query, &squared_euclidean).ok()?;
                if neighbors.len() < 4 {
                    return None;
                }
                let count = neighbors.len() as f32;
                let mut centroid = Vec3::ZERO;
                for (_, &idx) in &neighbors {
                    centroid += Vec3::from_array(positions[idx]);
                }
                centroid /= count;
                let mut cov = Mat3::ZERO;
                for (_, &idx) in &neighbors {
                    let d = Vec3::from_array(positions[idx]) - centroid;
                    cov.x_axis += d * d.x;
                    cov.y_axis += d * d.y;
                    cov.z_axis += d * d.z;
                }
                let normal = compute_normal_from_covariance(cov * (1.0 / count));
                let view_dir = self.viewpoint - Vec3::from_array(*pos);
                Some(if normal.dot(view_dir) < 0.0 { -normal } else { normal })
            })
            .collect();
        for (p, n) in cloud.points.iter_mut().zip(normals) {
            if let Some(n) = n {
                p.normal = n.to_array();
            }
        }
    }
}

/// Eigenvector of the smallest eigenvalue of a symmetric 3x3 matrix (analytic eigenvalues,
/// eigenvector from the best-conditioned cross product of the rows of `A − λI`).
pub(crate) fn compute_normal_from_covariance(cov: Mat3) -> Vec3 {
    let a = cov.as_dmat3();
    let (a00, a01, a02) = (a.x_axis.x, a.y_axis.x, a.z_axis.x);
    let (a11, a12, a22) = (a.y_axis.y, a.z_axis.y, a.z_axis.z);
    let p1 = a01 * a01 + a02 * a02 + a12 * a12;
    let q = (a00 + a11 + a22) / 3.0;
    let p2 = (a00 - q).powi(2) + (a11 - q).powi(2) + (a22 - q).powi(2) + 2.0 * p1;
    let p = (p2 / 6.0).sqrt();
    if !(p > 1e-18) {
        return Vec3::Z;
    }
    let b = (a - DMat3::from_diagonal(DVec3::splat(q))) * (1.0 / p);
    let r = (b.determinant() / 2.0).clamp(-1.0, 1.0);
    let phi = r.acos() / 3.0;
    // Smallest eigenvalue.
    let lambda = q + 2.0 * p * (phi + 2.0 * std::f64::consts::PI / 3.0).cos();
    let m = a - DMat3::from_diagonal(DVec3::splat(lambda));
    let (r0, r1, r2) = (m.row(0), m.row(1), m.row(2));
    let cands = [r0.cross(r1), r0.cross(r2), r1.cross(r2)];
    let best = cands.iter().copied().max_by(|x, y| x.length_squared().total_cmp(&y.length_squared())).unwrap();
    let v = best.normalize_or_zero();
    if v == DVec3::ZERO {
        Vec3::Z
    } else {
        v.as_vec3()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use scancraft_core::{ColorRgba, Point3D};

    #[test]
    fn test_normal_estimation_planar() {
        let mut cloud = PointCloud::new();
        // Points on an XY plane at Z = 50.0
        for x in -5..=5 {
            for y in -5..=5 {
                cloud.push(Point3D {
                    position: [x as f32 * 5.0, y as f32 * 5.0, 50.0],
                    normal: [0.0, 0.0, 0.0],
                    color: ColorRgba::WHITE,
                    confidence: 1.0,
                });
            }
        }

        let estimator = NormalEstimator::new(12, Vec3::new(0.0, 0.0, 0.0));
        estimator.estimate_normals(&mut cloud);

        // Center point normal should point towards viewpoint (Z- direction towards (0,0,0))
        let center_idx = 12 * 5 + 5; // near center
        let n = Vec3::from_slice(&cloud.points[center_idx].normal);
        assert!((n.z.abs() - 1.0).abs() < 0.15, "Normal was {:?}", n);
        assert!(n.z < 0.0, "Should point towards viewpoint, got {}", n.z);
    }
}
