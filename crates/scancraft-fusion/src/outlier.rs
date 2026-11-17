use kdtree::distance::squared_euclidean;
use kdtree::KdTree;
use scancraft_core::PointCloud;

/// Statistical Outlier Removal (SOR) filter.
/// Calculates the mean distance to k-nearest neighbors for every point.
/// Points with mean distance > mean + std_dev_multiplier * std_dev are pruned as noise.
pub struct StatisticalOutlierFilter {
    pub k_neighbors: usize,
    pub std_dev_multiplier: f32,
}

impl Default for StatisticalOutlierFilter {
    fn default() -> Self {
        Self {
            k_neighbors: 16,
            std_dev_multiplier: 1.0,
        }
    }
}

impl StatisticalOutlierFilter {
    pub fn new(k_neighbors: usize, std_dev_multiplier: f32) -> Self {
        Self {
            k_neighbors,
            std_dev_multiplier,
        }
    }

    pub fn filter(&self, cloud: &PointCloud) -> PointCloud {
        let n = cloud.len();
        if n <= self.k_neighbors + 1 {
            return cloud.clone();
        }

        // 1. Build KD-tree
        let mut kdtree = KdTree::new(3);
        for (i, p) in cloud.points.iter().enumerate() {
            let _ = kdtree.add(p.position, i);
        }

        // 2. Compute average neighbor distance for each point
        let mut mean_dists = Vec::with_capacity(n);
        let k_plus_one = self.k_neighbors + 1; // Includes query point itself

        for p in &cloud.points {
            let mut sum_dist = 0.0f32;
            let mut count = 0;

            if let Ok(neighbors) = kdtree.nearest(&p.position, k_plus_one, &squared_euclidean) {
                for (dist_sq, &idx) in neighbors {
                    // Skip query point itself (dist_sq == 0 at same index)
                    if dist_sq > 1e-9 {
                        sum_dist += dist_sq.sqrt();
                        count += 1;
                    }
                    let _ = idx;
                }
            }

            let avg = if count > 0 { sum_dist / count as f32 } else { 0.0 };
            mean_dists.push(avg);
        }

        // 3. Compute global mean and standard deviation
        let sum: f32 = mean_dists.iter().sum();
        let global_mean = sum / n as f32;

        let var_sum: f32 = mean_dists
            .iter()
            .map(|&d| {
                let diff = d - global_mean;
                diff * diff
            })
            .sum();
        let global_std_dev = (var_sum / n as f32).sqrt();
        let threshold = global_mean + self.std_dev_multiplier * global_std_dev;

        // 4. Retain inliers
        let mut filtered = PointCloud::new();
        for (idx, p) in cloud.points.iter().enumerate() {
            if mean_dists[idx] <= threshold {
                filtered.push(p.clone());
            }
        }

        filtered
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use scancraft_core::{ColorRgba, Point3D};

    #[test]
    fn test_statistical_outlier_removal() {
        let mut cloud = PointCloud::new();

        // Dense cluster of 50 points
        for i in 0..50 {
            let x = (i % 5) as f32 * 0.5;
            let y = ((i / 5) % 5) as f32 * 0.5;
            let z = 0.0;
            cloud.push(Point3D {
                position: [x, y, z],
                normal: [0.0, 0.0, 1.0],
                color: ColorRgba::WHITE,
                confidence: 1.0,
            });
        }

        // Two far stray points (outliers)
        cloud.push(Point3D {
            position: [100.0, 100.0, 100.0],
            normal: [0.0, 0.0, 1.0],
            color: ColorRgba::RED,
            confidence: 1.0,
        });
        cloud.push(Point3D {
            position: [-50.0, -80.0, 20.0],
            normal: [0.0, 0.0, 1.0],
            color: ColorRgba::RED,
            confidence: 1.0,
        });

        assert_eq!(cloud.len(), 52);

        let sor = StatisticalOutlierFilter::new(8, 1.0);
        let cleaned = sor.filter(&cloud);

        // Stray points should be eliminated
        assert!(cleaned.len() <= 50);
        assert!(!cleaned.points.iter().any(|p| p.position[0] > 50.0));
    }
}
