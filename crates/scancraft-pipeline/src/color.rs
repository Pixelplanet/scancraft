//! Color tracking (Revo tracking mode 3) as a [`PoseEstimator`]: RGB features matched against
//! recent colour keyframes give the sensor pose. The work is in `scancraft_color::ColorTracker`.
//!
//! Weak colour estimates are handed to the geometric tracker as a [`HintKind::Prior`] (ICP
//! refines and validates them). Strong ones are [`HintKind::Fixed`]: on flat or revolving
//! surfaces, exactly where Color tracking is needed, point-to-plane ICP cannot observe sliding
//! or turning along the surface and drifts away from a correct prior (measured with
//! `scancraft-color/examples/color_track_eval.rs`, see docs/protocol/color.md).

use crate::{FrameGeometry, PoseEstimator};
use scancraft_color::{ColorTracker, ColorTrackerConfig};
use scancraft_core::{SensorFrame, Transform};
use scancraft_slam::{HintKind, PoseHint, TrackingState, TrackingStatus};

pub struct ColorPoseEstimator {
    pub tracker: ColorTracker,
    /// Estimates with at least this many inliers ...
    pub fixed_min_inliers: usize,
    /// ... and at most this RMS reprojection error (px) are used as fixed poses. Set
    /// `fixed_min_inliers` to `usize::MAX` to always let ICP refine.
    pub fixed_max_rms_px: f32,
}

impl ColorPoseEstimator {
    /// `min_mm` / `max_mm`: depth range used to give features 3D points.
    pub fn new(min_mm: f32, max_mm: f32) -> Self {
        Self {
            tracker: ColorTracker::new(ColorTrackerConfig { min_depth_mm: min_mm, max_depth_mm: max_mm, ..ColorTrackerConfig::default() }),
            fixed_min_inliers: 100,
            fixed_max_rms_px: 1.8,
        }
    }
}

impl PoseEstimator for ColorPoseEstimator {
    fn name(&self) -> &'static str {
        "colour features"
    }

    fn estimate(&mut self, frame: &SensorFrame, geometry: &FrameGeometry, last_pose: &Transform) -> Option<PoseHint> {
        let color = frame.color.as_ref()?;
        let depth = geometry.depth.as_ref().or(frame.depth.as_ref());
        let pose = self.tracker.estimate(color, depth, last_pose)?;
        let s = self.tracker.last_stats;
        log::trace!("colour tracking: {s:?}");
        let kind = if s.inliers >= self.fixed_min_inliers && s.rms_px <= self.fixed_max_rms_px { HintKind::Fixed } else { HintKind::Prior };
        Some(PoseHint { pose, kind })
    }

    fn observe(&mut self, _frame: &SensorFrame, status: &TrackingStatus) {
        if matches!(status.state, TrackingState::Tracking | TrackingState::Slow) {
            self.tracker.accept(&status.current_pose);
        } else {
            self.tracker.reject();
        }
    }
}
