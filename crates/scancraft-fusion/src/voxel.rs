use scancraft_core::{ColorRgba, Point3D, PointCloud};
use std::collections::HashMap;

/// Voxel grid based point cloud downsampling and fusion filter
pub struct VoxelFilter {
    pub voxel_size_mm: f32,
}

impl VoxelFilter {
    pub fn new(voxel_size_mm: f32) -> Self {
        Self { voxel_size_mm }
    }

    /// Consolidate dense point cloud into voxel grid centroids
    pub fn filter(&self, cloud: &PointCloud) -> PointCloud {
        if self.voxel_size_mm <= 1e-4 {
            return cloud.clone();
        }

        let inv_size = 1.0 / self.voxel_size_mm;
        let mut grid: HashMap<(i32, i32, i32), (Point3D, usize)> = HashMap::new();

        for p in &cloud.points {
            let key = (
                (p.position[0] * inv_size).floor() as i32,
                (p.position[1] * inv_size).floor() as i32,
                (p.position[2] * inv_size).floor() as i32,
            );

            let entry = grid.entry(key).or_insert((
                Point3D {
                    position: [0.0; 3],
                    normal: [0.0; 3],
                    color: ColorRgba::BLACK,
                    confidence: 0.0,
                },
                0,
            ));

            entry.0.position[0] += p.position[0];
            entry.0.position[1] += p.position[1];
            entry.0.position[2] += p.position[2];
            entry.0.normal[0] += p.normal[0];
            entry.0.normal[1] += p.normal[1];
            entry.0.normal[2] += p.normal[2];
            entry.0.confidence += p.confidence;
            entry.0.color = p.color;
            entry.1 += 1;
        }

        let mut out = PointCloud::new();
        for (_, (mut pt, count)) in grid {
            let inv_count = 1.0 / (count as f32);
            pt.position[0] *= inv_count;
            pt.position[1] *= inv_count;
            pt.position[2] *= inv_count;
            pt.normal[0] *= inv_count;
            pt.normal[1] *= inv_count;
            pt.normal[2] *= inv_count;
            pt.confidence *= inv_count;
            out.push(pt);
        }

        out
    }
}
