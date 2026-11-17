use crate::kabsch::kabsch_align;
use glam::Vec3;
use kdtree::distance::squared_euclidean;
use kdtree::KdTree;
use scancraft_core::Transform;

#[derive(Debug, Clone)]
pub struct IcpConfig {
    pub max_iterations: usize,
    pub max_correspondence_dist_mm: f32,
    pub convergence_epsilon_mm: f32,
    pub min_inliers: usize,
    pub sample_stride: usize,
}

impl Default for IcpConfig {
    fn default() -> Self {
        Self {
            max_iterations: 8,
            max_correspondence_dist_mm: 25.0,
            convergence_epsilon_mm: 0.05,
            min_inliers: 10,
            sample_stride: 1,
        }
    }
}

#[derive(Debug, Clone)]
pub struct IcpResult {
    pub transform: Transform,
    pub rmse_mm: f32,
    pub inliers: usize,
    pub iterations: usize,
    pub converged: bool,
}

pub struct Icp;

impl Icp {
    /// Align `source` point cloud to `target` point cloud starting from `initial_guess`
    pub fn align(
        source: &[Vec3],
        target: &[Vec3],
        initial_guess: &Transform,
        config: &IcpConfig,
    ) -> IcpResult {
        if source.is_empty() || target.is_empty() {
            return IcpResult {
                transform: *initial_guess,
                rmse_mm: f32::MAX,
                inliers: 0,
                iterations: 0,
                converged: false,
            };
        }

        // 1. Build KD-tree for target points
        let mut kdtree = KdTree::new(3);
        for (i, pt) in target.iter().enumerate() {
            let _ = kdtree.add([pt.x, pt.y, pt.z], i);
        }

        let max_dist_sq = config.max_correspondence_dist_mm * config.max_correspondence_dist_mm;
        let mut cur_rot = initial_guess.rotation;
        let mut cur_trans = initial_guess.translation;

        let mut last_rmse = f32::MAX;
        let mut final_inliers = 0;
        let mut final_iter = 0;
        let mut converged = false;

        // Dynamically stride so that between 150 and 400 points are queried per iteration
        let dynamic_stride = (source.len() / 350).max(config.sample_stride);
        let mut src_corr = Vec::with_capacity(source.len() / dynamic_stride + 1);
        let mut tgt_corr = Vec::with_capacity(source.len() / dynamic_stride + 1);

        for iter in 0..config.max_iterations {
            final_iter = iter + 1;
            src_corr.clear();
            tgt_corr.clear();

            // Find closest target point for each transformed source point
            for (idx, p) in source.iter().enumerate() {
                if idx % dynamic_stride != 0 {
                    continue;
                }

                let p_trans = cur_rot.mul_vec3(*p) + cur_trans;
                let query = [p_trans.x, p_trans.y, p_trans.z];

                if let Ok(nearest) = kdtree.nearest(&query, 1, &squared_euclidean) {
                    if let Some((dist_sq, &target_idx)) = nearest.first() {
                        if *dist_sq <= max_dist_sq {
                            src_corr.push(*p);
                            tgt_corr.push(target[target_idx]);
                        }
                    }
                }
            }

            final_inliers = src_corr.len();
            if final_inliers < config.min_inliers {
                break;
            }

            // Compute optimal update from original source points to matched target points
            if let Some((r, t)) = kabsch_align(&src_corr, &tgt_corr) {
                cur_rot = r;
                cur_trans = t;

                // Compute updated RMSE with the new pose
                let mut post_dist_sq = 0.0f32;
                for (p, q) in src_corr.iter().zip(tgt_corr.iter()) {
                    let pt_transformed = cur_rot.mul_vec3(*p) + cur_trans;
                    post_dist_sq += pt_transformed.distance_squared(*q);
                }
                let rmse = (post_dist_sq / final_inliers as f32).sqrt();

                if (last_rmse - rmse).abs() < config.convergence_epsilon_mm {
                    converged = true;
                    last_rmse = rmse;
                    break;
                }
                last_rmse = rmse;
            } else {
                break;
            }
        }

        IcpResult {
            transform: Transform {
                translation: cur_trans,
                rotation: cur_rot,
                scale: Vec3::ONE,
            },
            rmse_mm: last_rmse,
            inliers: final_inliers,
            iterations: final_iter,
            converged,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::Quat;

    #[test]
    fn test_icp_alignment() {
        // Create an asymmetric 3D shape so registration is unambiguous
        let mut target = Vec::new();
        for i in 0..100 {
            let t = i as f32 * 0.1;
            target.push(Vec3::new(t * 5.0, (t * 2.0).sin() * 15.0, (t * 3.0).cos() * 10.0));
        }

        // Apply a known translation and small rotation
        let true_trans = Vec3::new(1.0, -0.5, 0.8);
        let true_rot = Quat::from_rotation_y(0.04);

        let source: Vec<Vec3> = target
            .iter()
            .map(|&p| true_rot.inverse().mul_vec3(p - true_trans))
            .collect();

        let config = IcpConfig {
            max_iterations: 30,
            max_correspondence_dist_mm: 5.0,
            convergence_epsilon_mm: 0.0001,
            min_inliers: 20,
            sample_stride: 1,
        };

        let result = Icp::align(&source, &target, &Transform::default(), &config);

        assert!(result.inliers >= 50);
        assert!(result.rmse_mm < 0.1, "RMSE was {}", result.rmse_mm);
        assert!((result.transform.translation - true_trans).length() < 0.15);
    }
}
