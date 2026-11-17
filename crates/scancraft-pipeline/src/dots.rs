//! Tracking on the dot pattern for modes whose own geometry is too sparse to track on (Cross
//! Lines with dots, `SensorFrame::dots`).
//!
//! The laser lines of one frame cover a few thousand points on a handful of curves; aligning
//! them to the model is unreliable, and colour features need dense depth to become 3D points.
//! The dot pair of the same frame gives that dense depth. [`DotsPose`] runs a complete inner
//! pipeline on it (stereo depth, optionally colour features, its own geometric tracker) and
//! hands the resulting pose to the outer tracker as a fixed pose, so the outer model is fused
//! from the precise line points only.

use crate::{DepthSource, FrameGeometry, PassPipeline, PoseEstimator};
use scancraft_core::{SensorFrame, Transform};
use scancraft_slam::{HintKind, PoseHint, Tracker, TrackingState};

/// Dense depth from the frame's dot pair (host stereo, as VCSEL Rapid).
pub struct DotsDepth {
    pub stereo: scancraft_stereo::StereoDepth,
}

impl DotsDepth {
    pub fn new(min_mm: f32, max_mm: f32) -> Self {
        Self { stereo: scancraft_stereo::StereoDepth::new(scancraft_stereo::StereoDepthConfig::new(min_mm, max_mm)) }
    }
}

impl DepthSource for DotsDepth {
    fn name(&self) -> &'static str {
        "dot-pattern stereo"
    }

    fn process(&mut self, frame: &SensorFrame) -> Option<FrameGeometry> {
        let depth = self.stereo.compute(frame.dots.as_ref()?)?;
        let cloud = depth.to_point_cloud(self.stereo.config.min_mm, self.stereo.config.max_mm);
        (!cloud.is_empty()).then(|| FrameGeometry { cloud, depth: Some(depth) })
    }
}

/// Pose from the dot pattern: an inner pipeline and tracker on the dense dot depth.
pub struct DotsPose {
    inner: PassPipeline,
    tracker: Tracker,
    /// State of the inner tracker for the last frame (for diagnostics).
    pub last_state: Option<TrackingState>,
}

impl DotsPose {
    /// `color`: also use colour features (Color tracking); otherwise geometry of the dots only.
    pub fn new(min_mm: f32, max_mm: f32, color: bool) -> Self {
        let pose: Option<Box<dyn PoseEstimator>> = if color { Some(Box::new(crate::color::ColorPoseEstimator::new(min_mm, max_mm))) } else { None };
        Self { inner: PassPipeline { depth: Box::new(DotsDepth::new(min_mm, max_mm)), pose, color: None }, tracker: Tracker::new(), last_state: None }
    }
}

impl PoseEstimator for DotsPose {
    fn name(&self) -> &'static str {
        "dot pattern"
    }

    fn estimate(&mut self, frame: &SensorFrame, _geometry: &FrameGeometry, _last_pose: &Transform) -> Option<PoseHint> {
        let status = self.inner.process(frame, &mut self.tracker);
        self.last_state = status.as_ref().map(|s| s.state);
        let status = status?;
        // Only a pose the inner tracker stands behind; while it is lost the outer tracker gets
        // no hint and reports the loss itself.
        matches!(status.state, TrackingState::Tracking | TrackingState::Slow).then_some(PoseHint { pose: status.current_pose, kind: HintKind::Fixed })
    }

    fn pose_required(&self) -> bool {
        true
    }
}
