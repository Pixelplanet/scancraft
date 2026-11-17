//! Retro-reflective marker tracking (Revo "Marker" and "Global Marker" tracking modes).
//!
//! Pipeline per frame (`docs/protocol/markers.md`):
//! 1. [`detect`]: dots in the left and right rectified IR images (local-contrast threshold,
//!    connected components, ellipse moments, sub-pixel intensity centroid);
//! 2. [`stereo`]: left/right matching along epipolar rows and triangulation into the sensor frame
//!    (+X right, +Y forward, +Z up);
//! 3. [`constellation`]: registration of the frame's markers against the pass's marker map
//!    (guess-based association, else triangle-signature RANSAC; Kabsch on all inliers);
//! 4. [`tracker::MarkerTracker`]: turns the registration into a `PoseHint` for the geometric
//!    tracker (`Fixed` when well constrained, `Prior` otherwise) and grows the [`map::MarkerMap`];
//! 5. [`global`]: two-phase Global Marker workflow with joint map refinement and JSON storage.
//!
//! Marker maps carry an environment / on-object tag per marker; [`map::align_marker_sets`]
//! registers the marker sets of two passes for the alignment stage.

pub mod constellation;
pub mod detect;
pub mod global;
pub mod map;
pub mod stereo;
pub mod tracker;

pub use constellation::{register, MatchConfig, RigidMatch};
pub use detect::{detect_markers, DetectorConfig, Marker2D};
pub use global::{refine_map, GlobalMarkerSession, RefineReport};
pub use map::{align_marker_sets, MapMarker, MarkerMap, MarkerSet, MarkerSetAlignment, MarkerTag, SharedMarkerMap, TagFilter};
pub use stereo::{estimate_right_cx_offset, match_stereo, Marker3D, StereoConfig, StereoGeometry};
pub use tracker::{DotState, FrameObservation, FrameReport, MapMode, MarkerFrameState, MarkerTracker, MarkerTrackerConfig, SharedObservations};
