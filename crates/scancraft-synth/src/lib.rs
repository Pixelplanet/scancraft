//! Synthetic scan scenes with ground truth, for testing ScanCraft's meshing (Reconstruct, Make
//! Solid), marker handling and alignment on cases whose true answer is known.
//!
//! A scene is a true part (an exact signed distance function, [`sdf::Shape`]), marker stickers
//! on it, and passes of a simulated scanner ([`scan`]) checked against Revo's raw previews
//! ([`realism`]): rays from several views with occlusion, a depth error that grows at grazing
//! incidence and is correlated in space (with hatching and grazing streaks), normals from each
//! view's depth image, blue-noise fusion at Revo's density, tables with clutter, and the faults
//! seen in real Revo scans (stickers 0.4 mm proud whose black rings scan thin or empty, Revo's
//! Fused-export cut-outs, cross-line laser strays behind the surface, passes sitting too deep or
//! tilted against each other).
//! [`write::generate`] writes a scene folder that the app imports as it imports Revo projects;
//! [`score::score`] checks a mesh against the scene's `truth.json`. See `README.md`.

pub mod app_pipeline;
pub mod eval;
pub mod memory;
pub mod mkf;
pub mod realism;
pub mod rng;
pub mod run;
pub mod scan;
pub mod scene;
pub mod scenes;
pub mod score;
pub mod sdf;
pub mod truth;
pub mod write;

pub use scan::{scan_pass, PassScan, PointKind, ScanPoint};
pub use scene::{Feature, PassSpec, Probe, ProbeKind, Scene, Sticker};
pub use sdf::{Rigid, Shape};
pub use write::{generate, Options, Truth};

#[cfg(test)]
mod tests;
#[cfg(test)]
mod eval_tests;
