//! Geometric segmentation fallback interface (used when no GPU or AI pack is available).

use crate::runtime::ActiveProvider;
use glam::Vec3;
use scancraft_fusion::{SegmentConfig, SegmentLabel, Segmentation, Seeds, segment_object};

/// Active segmentation strategy chosen at runtime.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum SegmentationBackend {
    Ai(ActiveProvider),
    GeometricFallback { reason: String },
}

/// Unified 3D point-cloud object segmentation interface.
pub trait ObjectSegmenter: Send + Sync {
    fn name(&self) -> &str;
    fn is_gpu_accelerated(&self) -> bool;
    fn segment_points(&self, points: &[Vec3], normals: Option<&[Vec3]>, seeds: Option<&Seeds>, up_hint: Option<Vec3>) -> Segmentation;
}

/// Geometric fallback backed by [`scancraft_fusion::segment_object`].
#[derive(Debug, Clone)]
pub struct GeometricSegmenter {
    pub config: SegmentConfig,
}

impl Default for GeometricSegmenter {
    fn default() -> Self {
        Self { config: SegmentConfig::default() }
    }
}

impl GeometricSegmenter {
    pub fn new(config: SegmentConfig) -> Self {
        Self { config }
    }
}

impl ObjectSegmenter for GeometricSegmenter {
    fn name(&self) -> &str {
        "Geometric (scancraft-fusion::segment_object)"
    }

    fn is_gpu_accelerated(&self) -> bool {
        false
    }

    fn segment_points(&self, points: &[Vec3], normals: Option<&[Vec3]>, seeds: Option<&Seeds>, up_hint: Option<Vec3>) -> Segmentation {
        segment_object(points, normals, &self.config, seeds, up_hint)
    }
}

/// Converts [`Segmentation`] labels into a boolean object mask.
pub fn object_mask(seg: &Segmentation) -> Vec<bool> {
    seg.labels.iter().map(|l| *l == SegmentLabel::Object).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn geometric_segmenter_separates_object_from_support_plane() {
        let mut pts = Vec::new();
        let mut norms = Vec::new();
        // Table plane at z = 0.
        for ix in -40..=40 {
            for iy in -40..=40 {
                let x = ix as f32 * 3.0;
                let y = iy as f32 * 3.0;
                if x * x + y * y > 25.0 * 25.0 {
                    pts.push(Vec3::new(x, y, 0.0));
                    norms.push(Vec3::Z);
                }
            }
        }
        // Cylinder-like block standing on the table (spacing ~1.25 mm < 2 mm cluster voxel).
        for iz in 1..=30 {
            for ia in 0..100 {
                let a = ia as f32 / 100.0 * std::f32::consts::TAU;
                pts.push(Vec3::new(20.0 * a.cos(), 20.0 * a.sin(), iz as f32 * 1.5));
                norms.push(Vec3::new(a.cos(), a.sin(), 0.0));
            }
        }

        let seg = GeometricSegmenter::default();
        assert!(!seg.is_gpu_accelerated());
        let out = seg.segment_points(&pts, Some(&norms), None, Some(Vec3::Z));
        assert!(out.support.is_some());
        assert!(out.count(SegmentLabel::Object) >= 600);
        assert!(out.count(SegmentLabel::Support) > 4000);
    }
}
