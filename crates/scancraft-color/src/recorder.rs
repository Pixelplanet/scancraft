//! Per-pass colour recording: picks RGB keyframes from tracked frames and colours freshly fused
//! points for the live view.

use crate::colorize::{colorize_from_view, colorize_points, ColorizeConfig, ColorizeStats};
use crate::keyframe::{frame_sharpness, KeyframeSelector, RgbKeyframe, SelectorConfig};
use scancraft_core::{ColorFrame, Point3D, SensorFrame, Transform};

/// Records colour for one pass. Feed it every frame the tracker accepted, with its pose.
#[derive(Debug, Default)]
pub struct ColorRecorder {
    selector: KeyframeSelector,
    keyframes: Vec<RgbKeyframe>,
    /// The latest accepted colour frame and its sensor pose.
    last_view: Option<(ColorFrame, Transform)>,
    frames_seen: u64,
    frames_with_color: u64,
}

impl ColorRecorder {
    pub fn new(config: SelectorConfig) -> Self {
        Self { selector: KeyframeSelector::new(config), ..Self::default() }
    }

    /// An accepted frame at `sensor_pose` (world <- scanner frame).
    pub fn observe(&mut self, frame: &SensorFrame, sensor_pose: &Transform) {
        self.frames_seen += 1;
        let Some(color) = frame.color.as_ref() else { return };
        let Some(camera) = color.camera.clone() else { return };
        self.frames_with_color += 1;
        let sharpness = frame_sharpness(color);
        if let Some(kf) = self.selector.offer(color, &camera, *sensor_pose, sharpness, frame.frame_index) {
            log::debug!("RGB keyframe {} (t={} ms, sharpness {:.0})", kf.id, kf.timestamp_ms, kf.sharpness);
            self.keyframes.push(kf);
        }
        self.last_view = Some((color.clone(), *sensor_pose));
    }

    /// Colours points just fused from the latest accepted frame (live preview). Points outside
    /// its colour image keep their colour. Returns how many were coloured.
    pub fn colorize_new_points(&self, points: &mut [Point3D]) -> usize {
        let Some((color, pose)) = &self.last_view else { return 0 };
        let Some(camera) = color.camera.as_deref() else { return 0 };
        colorize_from_view(points, &color.image, camera, &camera.world_from_color(pose))
    }

    /// Final vertex colours from all keyframes (view-angle and sharpness weighted, occlusion-aware).
    pub fn colorize_model(&self, points: &mut [Point3D]) -> ColorizeStats {
        colorize_points(points, &self.keyframes, &ColorizeConfig::default())
    }

    /// The RGB keyframes of this pass, for texturing and Gaussian splatting.
    pub fn keyframes(&self) -> &[RgbKeyframe] {
        &self.keyframes
    }

    pub fn take_keyframes(&mut self) -> Vec<RgbKeyframe> {
        std::mem::take(&mut self.keyframes)
    }

    /// Fraction of accepted frames that carried a colour frame (pairing health).
    pub fn color_frame_ratio(&self) -> f32 {
        if self.frames_seen == 0 {
            0.0
        } else {
            self.frames_with_color as f32 / self.frames_seen as f32
        }
    }
}
