//! Depth source for Hybrid HD: speckle + phase-shifted fringe stereo on the host
//! (`scancraft-hybrid`, `docs/protocol/hybrid.md`).

use crate::{DepthSource, FrameGeometry};
use scancraft_core::SensorFrame;
use scancraft_hybrid::{hybrid_depth, HybridParams};

pub struct HybridStereoDepth {
    pub params: HybridParams,
}

impl HybridStereoDepth {
    pub fn new(min_mm: f32, max_mm: f32) -> Self {
        Self { params: HybridParams { min_mm, max_mm, ..HybridParams::default() } }
    }
}

impl DepthSource for HybridStereoDepth {
    fn name(&self) -> &'static str {
        "hybrid speckle + fringe stereo"
    }

    fn process(&mut self, frame: &SensorFrame) -> Option<FrameGeometry> {
        let ir = frame.ir.as_ref().filter(|ir| ir.rectified)?;
        let (right, rig) = (ir.right.as_ref()?, ir.rig.as_ref()?);
        let depth = hybrid_depth(&ir.left, right, frame.fringes.as_ref(), rig, &self.params);
        let cloud = depth.to_point_cloud(self.params.min_mm, self.params.max_mm);
        (!cloud.is_empty()).then(|| FrameGeometry { cloud, depth: Some(depth) })
    }
}
