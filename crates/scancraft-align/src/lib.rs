//! Alignment between scan passes and merging of aligned passes
//! (`docs/architecture/scan-sessions.md`, sections 3 and 6).
//!
//! Every pass has its own coordinate frame (its tracker's world frame). Alignment finds the rigid
//! transform taking a pass into the project frame:
//!
//! - [`global::align_global`]: no initial guess (FPFH features + RANSAC, then point-to-plane ICP);
//! - [`pairs::align_point_pairs`]: from >= 3 user-picked correspondences (Kabsch, then ICP);
//! - [`markers::align_markers`]: from marker dots rigidly attached to the object;
//! - [`refine::refine_global`]: pairwise ICP between overlapping passes + pose-graph optimisation;
//! - [`quality::evaluate`]: overlap, RMS and visibility consistency of any candidate transform.
//!
//! [`merge::merge`] fuses aligned passes into one model with per-pass weights, a confidence filter
//! for isolated single observations and a statistical outlier pass.
//!
//! All transforms map *source* coordinates into *target* coordinates (`target <- source`).
//! Positions are millimetres in the scanner convention (+X right, +Y forward, +Z up).

pub mod cloud;
pub mod features;
pub mod global;
pub mod markers;
pub mod merge;
pub mod pairs;
pub mod quality;
pub mod refine;
pub mod synthetic;

use glam::{Quat, Vec3};
use scancraft_core::Transform;

pub use global::{align_global, GlobalConfig};
pub use markers::{align_markers, Marker, MarkerAlignment, MarkerPlacement};
pub use merge::{merge, MergeConfig, MergeInput, MergeResult};
pub use pairs::align_point_pairs;
pub use quality::{evaluate, Quality, Verdict};
pub use refine::{refine_global, RefineConfig, RefineEdge, RefineInput, RefineResult};

/// A point with its unit normal.
pub type Oriented = (Vec3, Vec3);

/// One pass as alignment sees it, in the pass's own frame.
#[derive(Debug, Clone, Default)]
pub struct PassCloud {
    /// The pass's fused surface.
    pub points: Vec<Oriented>,
    /// Where the scanner was and what it saw (its kept keyframes), for the visibility check.
    pub views: Vec<View>,
}

/// One stored scanner view: pose (pass <- sensor) and the points seen, in the sensor frame.
#[derive(Debug, Clone, Default)]
pub struct View {
    pub pose: Transform,
    pub points: Vec<Vec3>,
}

impl PassCloud {
    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }
}

/// A transform found for a pass and how good it looks.
#[derive(Debug, Clone, Copy)]
pub struct Alignment {
    /// target <- source.
    pub transform: Transform,
    pub quality: Quality,
}

#[derive(Debug, thiserror::Error, PartialEq)]
pub enum AlignError {
    #[error("not enough points to align ({0})")]
    TooFewPoints(&'static str),
    #[error("at least 3 point pairs are needed")]
    TooFewPairs,
    #[error("the points are (nearly) on a line; pick points that span the object")]
    Degenerate,
    #[error("no consistent alignment was found")]
    NotFound,
    #[error("at least 3 markers on the object must be seen in both passes (found {0})")]
    TooFewMarkers(usize),
}

/// `a ∘ b`: apply `b`, then `a`.
pub fn compose(a: &Transform, b: &Transform) -> Transform {
    Transform { rotation: (a.rotation * b.rotation).normalize(), translation: a.rotation * b.translation + a.translation, scale: Vec3::ONE }
}

/// A rigid transform (unit scale) from rotation and translation.
pub fn rigid(rotation: Quat, translation: Vec3) -> Transform {
    Transform { rotation: rotation.normalize(), translation, scale: Vec3::ONE }
}

/// Angle (radians) between two rotations, accurate for small angles too (`Quat::angle_between`
/// goes through `acos` and cannot resolve below ~0.03 degrees in f32).
pub fn rotation_angle(a: Quat, b: Quat) -> f32 {
    let d = (a.inverse() * b).normalize();
    2.0 * d.xyz().length().min(1.0).asin()
}

/// Largest distance between where `a` and `b` put the given points: a frame-independent
/// measure of how different two alignments are on the object.
pub fn displacement_on(a: &Transform, b: &Transform, points: &[Vec3]) -> f32 {
    points.iter().map(|&p| (a.transform_point3(p) - b.transform_point3(p)).length()).fold(0.0, f32::max)
}
