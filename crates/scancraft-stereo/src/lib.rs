//! Dense host-side stereo for IR-pair scan modes (VCSEL Rapid, Hybrid HD).
//!
//! - [`SgmMatcher`]: census + semi-global matching on a rectified 8-bit pair → [`DisparityImage`].
//! - [`HierarchicalMatcher`]: the same at half resolution plus a full-resolution refinement
//!   (several times faster; the default of [`StereoDepth`]).
//! - [`RectifyMap`]: lookup-table remap for pairs that arrive unrectified.
//! - [`StereoDepth`]: matcher + disparity→depth for a `SensorFrame`'s IR pair and `StereoRig`.

pub mod census;
pub mod depth;
pub mod filter;
pub mod hier;
pub mod rectify;
pub mod sgm;

pub use depth::{disparity_to_depth, disparity_window, StereoDepth, StereoDepthConfig, DEPTH_SCALE_MM};
pub use hier::HierarchicalMatcher;
pub use rectify::{DistortedCamera, RectifyMap};
pub use sgm::{DisparityImage, SgmMatcher, SgmParams};
