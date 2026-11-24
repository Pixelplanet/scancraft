//! Alignment between scan passes and merging of aligned passes
//! (`docs/architecture/scan-sessions.md`, sections 3 and 6).
//!
//! Every pass has its own coordinate frame (its tracker's world frame). Alignment finds the rigid
//! transform taking a pass into the project frame:
//!
//! - [`global::align_global`]: no initial guess (FPFH features + RANSAC, then point-to-plane ICP);
//! - [`global::align_robust`]: `align_global` on several feature grids in both directions, all
//!   results ranked on one grid, with a check that the two directions agree;
//! - [`graph::align_all`]: every pair of passes measured, a maximum-quality spanning tree from the
//!   reference, missing pairs composed through it, cycles checked;
//! - [`pairs::align_point_pairs`]: from >= 3 user-picked correspondences (Kabsch, then ICP);
//! - [`markers::align_markers`]: from marker dots rigidly attached to the object;
//! - [`refine::refine_global`]: pairwise ICP between overlapping passes + pose-graph optimisation;
//! - [`refine::refine_shell`]: fine ICP that also pulls the inside of a wall onto the outside at
//!   the wall thickness, which it estimates;
//! - [`quality::evaluate`]: how plausible any candidate transform is: same-side overlap and RMS,
//!   wall thickness and piercing where an inside meets an outside scan, visibility consistency,
//!   support-plane (table) violations and how freely the fit could slide.
//!
//! [`merge::merge`] fuses aligned passes into one model with per-pass weights, a confidence filter
//! for isolated single observations and a statistical outlier pass.
//!
//! All transforms map *source* coordinates into *target* coordinates (`target <- source`).
//! Positions are millimetres in the scanner convention (+X right, +Y forward, +Z up).

pub mod cloud;
pub mod features;
pub mod global;
pub mod graph;
pub mod markers;
pub mod merge;
pub mod pairs;
pub mod quality;
pub mod refine;
pub mod synthetic;

use glam::{Quat, Vec3};
use scancraft_core::Transform;

pub use global::{align_global, align_robust, Candidate, GlobalConfig, RobustAlignment, RobustConfig};
pub use graph::{align_all, ComposedEdge, CycleCheck, EdgeState, GraphConfig, GraphEdge, GraphResult, Rejection};
pub use markers::{align_markers, Marker, MarkerAlignment, MarkerPlacement};
pub use merge::{merge, MergeConfig, MergeInput, MergeResult};
pub use pairs::align_point_pairs;
pub use quality::{evaluate, Quality, Shell, Verdict};
pub use refine::{refine_global, refine_shell, RefineConfig, RefineEdge, RefineInput, RefineResult, ShellConfig, ShellFit};

/// A point with its unit normal.
pub type Oriented = (Vec3, Vec3);

/// One pass as alignment sees it, in the pass's own frame.
#[derive(Debug, Clone, Default)]
pub struct PassCloud {
    /// The pass's fused surface.
    pub points: Vec<Oriented>,
    /// Where the scanner was and what it saw (its kept keyframes), for the visibility check.
    pub views: Vec<View>,
    /// The surface the object stood on during the pass (the table), normal pointing from the
    /// table towards the object. Nothing of the object can lie below it, which rules out
    /// alignments that push one pass through the other's table.
    pub support: Option<Plane>,
}

/// An oriented plane: `normal . p + d` is the signed distance of `p`, positive on the side the
/// normal points to.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Plane {
    /// Unit normal.
    pub normal: Vec3,
    pub d: f32,
}

impl Plane {
    /// The plane through `point` with `normal` (normalised here).
    pub fn through(point: Vec3, normal: Vec3) -> Self {
        let normal = normal.normalize();
        Plane { normal, d: -normal.dot(point) }
    }

    /// Signed distance of `p`, mm.
    pub fn distance(&self, p: Vec3) -> f32 {
        self.normal.dot(p) + self.d
    }

    /// The plane in another frame (`t`: new <- old).
    pub fn transformed(&self, t: &Transform) -> Plane {
        let normal = (t.rotation * self.normal).normalize();
        Plane { normal, d: self.d - normal.dot(t.translation) }
    }
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

#[derive(Debug, Clone, thiserror::Error, PartialEq)]
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

/// RMS distance between where `a` and `b` put the given points (0 for no points).
pub fn rms_displacement_on(a: &Transform, b: &Transform, points: &[Vec3]) -> f32 {
    if points.is_empty() {
        return 0.0;
    }
    (points.iter().map(|&p| (a.transform_point3(p) - b.transform_point3(p)).length_squared()).sum::<f32>() / points.len() as f32).sqrt()
}

/// How far apart two alignments of the same points are.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Agreement {
    /// Angle between the two rotations, degrees.
    pub rotation_deg: f32,
    /// RMS distance between where the two put the probe points, mm.
    pub offset_mm: f32,
}

impl Agreement {
    /// `a` against `b` on `probe` (points in the frame both transforms start from).
    pub fn between(a: &Transform, b: &Transform, probe: &[Vec3]) -> Self {
        Agreement { rotation_deg: rotation_angle(a.rotation, b.rotation).to_degrees(), offset_mm: rms_displacement_on(a, b, probe) }
    }

    /// Both measures within the limits.
    pub fn within(&self, rotation_deg: f32, offset_mm: f32) -> bool {
        self.rotation_deg <= rotation_deg && self.offset_mm <= offset_mm
    }
}

/// Up to `count` points of `points`, evenly spread over the list: probe points for
/// [`Agreement::between`].
pub fn probe(points: &[Oriented], count: usize) -> Vec<Vec3> {
    let step = points.len().div_ceil(count.max(1)).max(1);
    points.iter().step_by(step).map(|p| p.0).collect()
}
