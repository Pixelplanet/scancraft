//! Colour for ScanCraft (docs/architecture/scan-sessions.md section 4, docs/protocol/color.md).
//!
//! - [`keyframe`]: [`RgbKeyframe`], the colour images a pass keeps (JPEG + pose + calibration +
//!   timestamp + sharpness) for vertex colours, texturing and Gaussian splatting; selection.
//! - [`colorize`]: vertex colours from keyframes (view-angle, sharpness, border weighting,
//!   occlusion-aware).
//! - [`recorder`]: [`ColorRecorder`], per-pass keyframe selection and live colouring.
//! - [`registration`]: depth projected into the colour image (3D point per colour pixel).
//! - [`features`], [`tracker`]: ORB-style features and RGB-D pose estimation (Color tracking).
//! - [`image_ops`]: grey images, sharpness, sampling, JPEG decode.

pub mod colorize;
pub mod features;
pub mod image_ops;
pub mod keyframe;
pub mod recorder;
pub mod registration;
pub mod synthetic;
pub mod tracker;

pub use colorize::{colorize_from_view, colorize_points, ColorizeConfig, ColorizeStats};
pub use image_ops::{decode_jpeg, sharpness, to_gray, Gray};
pub use keyframe::{frame_sharpness, interpolate_pose, KeyframeSelector, RgbKeyframe, SelectorConfig};
pub use recorder::ColorRecorder;
pub use registration::DepthInColor;
pub use tracker::{ColorTrackStats, ColorTracker, ColorTrackerConfig};
