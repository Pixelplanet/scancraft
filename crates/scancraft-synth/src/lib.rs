//! Synthetic scan scenes with ground truth, for testing ScanCraft's meshing (Reconstruct, Make
//! Solid), marker handling and alignment on cases whose true answer is known.
//!
//! A scene is a true part (an exact signed distance function, [`sdf::Shape`]), marker stickers
//! on it, and passes of a simulated scanner ([`scan`]): rays from several views with occlusion,
//! noise along the ray that grows at grazing incidence, coverage lost past a maximum incidence,
//! the views fused at a point spacing, normals from plane fits of the noisy points, and the
//! faults seen in real Revo scans (stickers 0.4 mm proud, sticker cut-outs with and without an
//! empty ring, cross-line laser strays behind the surface, passes sitting too deep or shifted).
//! [`write::generate`] writes a scene folder that the app imports as it imports Revo projects;
//! [`score::score`] checks a mesh against the scene's `truth.json`. See `README.md`.

pub mod mkf;
pub mod rng;
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
