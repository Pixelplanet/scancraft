//! AI object segmentation (SAM 2.1 video on ONNX Runtime + DirectML) and geometric fallback.
//!
//! This crate implements ScanCraft's object segmentation core:
//!
//! - **Runtime & integrity ([`manifest`], [`runtime`]):** loads ONNX Runtime dynamically via
//!   `ort` (`load-dynamic`, `directml`, `api-24`; never `download-binaries`, `copy-dylibs`, CUDA,
//!   or TensorRT). Every model graph and runtime DLL is pinned by SHA-256 and verified before
//!   loading. On Windows, DirectML hardware adapters are enumerated via DXGI and validated
//!   against CPU output by a self-test before use.
//! - **SAM 2.1 video graphs & loop ([`graphs`], [`memory_bank`], [`video`]):** typed sessions for
//!   the four Apache-2.0 `jax-image-tools/sam21-tiny-video-onnx` graphs (`vision_encoder`,
//!   `mask_decoder`, `memory_attention`, `memory_encoder`) plus the Rust video loop (conditioning
//!   and recent-frame memory bank, 4-token-per-pointer object pointers with 1D sinusoidal temporal
//!   positional encoding, positive/negative point and box prompts, and mid-sequence re-prompting).
//! - **Pre/post-processing ([`preprocess`], [`postprocess`]):** RGB normalisation, coordinate
//!   mapping, bilinear mask resampling, thresholding, stability/IoU scoring, and lost-object
//!   reporting.
//! - **Mask -> 3D & fallback ([`project3d`], [`fallback`]):** projects 2D masks onto depth maps
//!   and 3D points through `scancraft_core::ColorCamera` with a z-buffer occlusion check, plus an
//!   [`ObjectSegmenter`] interface backed by [`scancraft_fusion::segment_object`] for machines
//!   without a usable GPU.
//!
//! # Licences & User Consent Dialog Notes
//!
//! This crate never downloads model or runtime binaries itself; the application's AI pack
//! manager handles consent and downloads externally. When presenting the consent dialog to the
//! user, include these licence and telemetry notices:
//!
//! 1. **SAM 2.1 Hiera-Tiny ONNX (`jax-image-tools/sam21-tiny-video-onnx`):**
//!    Derived from Meta's `facebook/sam2.1-hiera-tiny`, licensed under **Apache-2.0**.
//!    Redistributions must retain the `LICENSE` and a `NOTICE` noting conversion to ONNX (opset 17)
//!    with real-valued RoPE. Never uses Revo Scan's proprietary encrypted models.
//! 2. **ONNX Runtime 1.24.4 (`Microsoft.ML.OnnxRuntime.DirectML`):**
//!    Licensed under **MIT** with `ThirdPartyNotices.txt`. Keep both next to `onnxruntime.dll`.
//! 3. **DirectML 1.15.4 (`Microsoft.AI.DirectML` / `DirectML.dll`):**
//!    Governed by the **Microsoft Software License Terms** (`LICENSE-DirectML.txt`):
//!    - Windows / Xbox platforms only.
//!    - May be distributed **only as part of an application** built with ML frameworks (no
//!      standalone redistribution of `DirectML.dll`).
//!    - **Telemetry clause:** DirectML may collect diagnostic/usage telemetry subject to
//!      Microsoft's privacy terms (with OS/framework opt-outs). Disclose this clearly in the
//!      consent dialog before enabling or downloading DirectML.

pub mod fallback;
pub mod graphs;
pub mod manifest;
pub mod memory_bank;
pub mod postprocess;
pub mod preprocess;
pub mod project3d;
pub mod runtime;
pub mod video;

pub use fallback::{GeometricSegmenter, ObjectSegmenter, SegmentationBackend};
pub use manifest::{ModelFiles, PinnedFile, RuntimeFiles, sha256_file, verify_sha256};
pub use memory_bank::{AssembledMemory, MemoryBank, MemoryEntry, ModelConstants};
pub use postprocess::{FrameMask, FrameTimings, LostReason, Mask2D, PostprocessConfig, TrackingStatus};
pub use preprocess::{BoxPrompt, FramePrompt, ImageTransform, PointPrompt, ResizeMode};
pub use project3d::{ColorDepthBuffer, DepthMaskProjection, MultiViewVoter, PointLabel, Project3DConfig};
pub use runtime::{ActiveProvider, ExecutionPreference, GpuAdapterInfo, OrtRuntime, SelfTestReport, enumerate_dxgi_adapters};
pub use video::Sam2VideoSegmenter;

/// Errors returned by `scancraft-segment`.
#[derive(Debug, thiserror::Error)]
pub enum SegmentError {
    #[error("I/O error on {path}: {source}")]
    Io {
        path: std::path::PathBuf,
        #[source]
        source: std::io::Error,
    },

    #[error("Missing required file: {0}")]
    MissingFile(std::path::PathBuf),

    #[error("SHA-256 mismatch for {path}: expected {expected}, got {actual}")]
    HashMismatch {
        path: std::path::PathBuf,
        expected: String,
        actual: String,
    },

    #[error("Invalid constants.json ({path}): {reason}")]
    InvalidConstants { path: std::path::PathBuf, reason: String },

    #[error("ONNX Runtime error: {0}")]
    Ort(#[from] ort::Error),

    #[error("Dynamic library load error for {path}: {reason}")]
    LibraryLoad { path: std::path::PathBuf, reason: String },

    #[error("Unexpected tensor shape for {name}: {detail}")]
    InvalidShape { name: &'static str, detail: String },

    #[error("Invalid input: {0}")]
    InvalidInput(String),

    #[error("Cannot propagate before at least one frame has been prompted")]
    NoPrompt,

    #[error("Self-test failed for {provider}: {reason}")]
    SelfTestFailed { provider: String, reason: String },

    #[error("No usable inference backend: {0}")]
    NoUsableBackend(String),
}

pub type Result<T> = std::result::Result<T, SegmentError>;

impl From<ort::Error<ort::session::builder::SessionBuilder>> for SegmentError {
    fn from(err: ort::Error<ort::session::builder::SessionBuilder>) -> Self {
        Self::Ort(ort::Error::new_with_code(err.code(), err.message()))
    }
}
