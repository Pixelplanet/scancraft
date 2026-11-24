pub mod decimate;
pub mod denoise;
pub mod holes;
pub mod implicit;
pub mod marching_cubes;
pub mod metrics;
pub mod normals;
pub mod smooth;
pub mod solid;
pub mod tables;

#[cfg(test)]
mod tests;

pub use decimate::MeshDecimator;
pub use denoise::FeatureSmoother;
pub use holes::{FillReport, HoleFiller, HoleInfo};
pub use implicit::{SurfaceConfig, SurfaceStats};
pub use marching_cubes::{MarchingCubes, ReconstructionStats};
pub use metrics::{MeshAnalysis, MeshMetrics};
pub use normals::NormalEstimator;
pub use smooth::LaplacianSmoother;
pub use solid::{build_solid, fill_holes, remove_markers, Solid, SolidConfig, SolidPoint, SolidReport, VertexOrigin};

use scancraft_core::{Mesh, PointCloud};

pub struct Mesher;

impl Mesher {
    /// Reconstruct a welded triangle mesh from a point cloud (sparse signed distance field +
    /// marching cubes). Normals are used as given when most points have one (they must face away
    /// from the interior, i.e. towards the scanner); otherwise they are estimated and oriented
    /// towards the origin.
    pub fn reconstruct(cloud: &PointCloud, cell_size_mm: f32) -> Mesh {
        let n = cloud.points.len();
        let step = (n / 1000).max(1);
        let sampled = cloud.points.iter().step_by(step).count();
        let with_normals = cloud
            .points
            .iter()
            .step_by(step)
            .filter(|p| p.normal.iter().any(|c| *c != 0.0))
            .count();

        let mc = MarchingCubes::new(cell_size_mm);
        if sampled > 0 && with_normals * 10 >= sampled * 9 {
            mc.reconstruct(cloud)
        } else {
            let mut cloud_with_normals = cloud.clone();
            NormalEstimator::default().estimate_normals(&mut cloud_with_normals);
            mc.reconstruct(&cloud_with_normals)
        }
    }

    /// Reconstructs a welded mesh with the robust implicit surface ([`implicit`]). Points without
    /// normals get estimated ones first, as in [`reconstruct`](Self::reconstruct).
    pub fn reconstruct_with(cloud: &PointCloud, config: &SurfaceConfig) -> Mesh {
        Self::reconstruct_with_stats(cloud, config).0
    }

    pub fn reconstruct_with_stats(cloud: &PointCloud, config: &SurfaceConfig) -> (Mesh, SurfaceStats) {
        let step = (cloud.points.len() / 1000).max(1);
        let sampled = cloud.points.iter().step_by(step).count();
        let with_normals = cloud.points.iter().step_by(step).filter(|p| p.normal.iter().any(|c| *c != 0.0)).count();
        if sampled > 0 && with_normals * 10 >= sampled * 9 {
            implicit::reconstruct(cloud, config)
        } else {
            let mut with = cloud.clone();
            NormalEstimator::default().estimate_normals(&mut with);
            implicit::reconstruct(&with, config)
        }
    }

    /// Smooth mesh surface using Laplacian diffusion
    pub fn smooth(mesh: &mut Mesh, iterations: usize, lambda: f32) {
        let smoother = LaplacianSmoother::new(iterations, lambda);
        smoother.smooth(mesh);
    }

    /// Simplify mesh to target ratio
    pub fn decimate(mesh: &Mesh, target_ratio: f32) -> Mesh {
        let decimator = MeshDecimator::new(target_ratio);
        decimator.decimate(mesh)
    }

    /// Compute geometric metrics and watertight status
    pub fn analyze(mesh: &Mesh) -> MeshAnalysis {
        MeshMetrics::analyze(mesh)
    }
}
