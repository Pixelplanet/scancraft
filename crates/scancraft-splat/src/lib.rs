//! Gaussian splatting for ScanCraft.
//!
//! A splat is a *project output*: it is trained from the RGB keyframes (+ poses) of the aligned
//! colour passes and initialised from the merged point cloud. Points that no colour keyframe
//! sees are excluded. Training uses [Brush](https://github.com/ArthurBrussee/brush) (Apache-2.0)
//! as a library, on its own wgpu device and off the UI thread.
//!
//! * [`SplatDataset`] / [`PosedImage`] — the input contract (see `docs/architecture/splatting.md`).
//! * [`train()`] (blocking) and [`SplatJob`] (background thread, progress, cancel).
//! * [`TrainedSplat`] — export to the standard 3DGS `.ply`, render views for a viewport.
//! * [`export`] — the same inputs as a nerfstudio (`transforms.json`) / COLMAP text dataset, for
//!   training in external tools.
//! * [`synthetic`] — a CPU-rendered test scene with known cameras.

pub mod convert;
pub mod coverage;
pub mod device;
pub mod export;
pub mod synthetic;
pub mod train;
pub mod types;
pub mod viewer;

pub use coverage::CoverageConfig;
pub use device::SplatDevice;
pub use train::{EvalStats, InitColors, MaskMode, Phase, SplatJob, SplatTrainConfig, TrainProgress, TrainReport, train};
pub use types::{CameraAxes, Distortion, Intrinsics, PosedImage, SplatDataset};

use brush_render::gaussian_splats::Splats;
use brush_render::{TextureMode, render_splats};
use std::path::Path;

#[derive(Debug, thiserror::Error)]
pub enum SplatError {
    #[error("invalid input: {0}")]
    InvalidInput(String),
    #[error("cancelled")]
    Cancelled,
    #[error("brush: {0}")]
    Brush(String),
    #[error("io: {0}")]
    Io(#[from] std::io::Error),
    #[error("image: {0}")]
    Image(#[from] image::ImageError),
}

/// A trained splat (on the GPU, in Brush's metre units) and how it was made.
pub struct TrainedSplat {
    splats: Splats,
    device: SplatDevice,
    pub report: TrainReport,
}

fn block_on<F: std::future::Future>(f: F) -> F::Output {
    tokio::runtime::Builder::new_current_thread()
        .enable_all()
        .build()
        .expect("tokio runtime")
        .block_on(f)
}

impl TrainedSplat {
    pub fn num_splats(&self) -> u32 {
        self.splats.num_splats()
    }

    /// Brush's splats (metres, ScanCraft axes), for callers using Brush directly.
    pub fn brush_splats(&self) -> &Splats {
        &self.splats
    }

    pub fn device(&self) -> &SplatDevice {
        &self.device
    }

    /// Standard 3DGS binary PLY (`x y z`, `f_dc_*`, `f_rest_*`, `opacity`, `scale_*`, `rot_*`),
    /// in ScanCraft world units (mm, Z-up). The header carries Brush's comments, including
    /// `Vertical axis: 0 0 1`, which Brush's viewer uses to orient the scene.
    pub fn to_ply_bytes(&self) -> Result<Vec<u8>, SplatError> {
        let splats = self.splats.clone().scaled(1.0 / convert::MM_TO_M);
        block_on(brush_serde::splat_to_ply(splats, Some(brush_glam::Vec3::Z))).map_err(|e| SplatError::Brush(e.to_string()))
    }

    pub fn save_ply(&self, path: impl AsRef<Path>) -> Result<(), SplatError> {
        let bytes = self.to_ply_bytes()?;
        if let Some(dir) = path.as_ref().parent() {
            std::fs::create_dir_all(dir)?;
        }
        std::fs::write(path, bytes)?;
        Ok(())
    }

    /// Render a look-at view (ScanCraft world, mm) to RGBA8 — what a viewport integration
    /// uploads as a texture. `fov_y` in radians; `background` RGB 0..1.
    #[allow(clippy::too_many_arguments)]
    pub fn render_rgba(
        &self,
        eye: glam::Vec3,
        target: glam::Vec3,
        up: glam::Vec3,
        fov_y: f32,
        width: u32,
        height: u32,
        background: [f32; 3],
    ) -> Result<image::RgbaImage, SplatError> {
        let camera = convert::look_at_camera(eye, target, up, fov_y, width, height);
        self.render_camera(&camera, width, height, background)
    }

    /// Render from one of the dataset's posed images.
    pub fn render_view(&self, image: &PosedImage, background: [f32; 3]) -> Result<image::RgbaImage, SplatError> {
        self.render_camera(&convert::brush_camera(image), image.width, image.height, background)
    }

    fn render_camera(
        &self,
        camera: &brush_render::camera::Camera,
        width: u32,
        height: u32,
        background: [f32; 3],
    ) -> Result<image::RgbaImage, SplatError> {
        block_on(async {
            let (img, _) = render_splats(
                self.splats.clone(),
                camera,
                brush_glam::uvec2(width, height),
                brush_glam::Vec3::from_array(background),
                None,
                TextureMode::Float,
            )
            .await;
            let data: Vec<f32> = img
                .into_data_async()
                .await
                .map_err(|e| SplatError::Brush(format!("{e:?}")))?
                .try_into_vec()
                .map_err(|e| SplatError::Brush(format!("{e:?}")))?;
            let bytes = data.iter().map(|v| (v.clamp(0.0, 1.0) * 255.0).round() as u8).collect();
            image::RgbaImage::from_raw(width, height, bytes).ok_or_else(|| SplatError::Brush("render size".into()))
        })
    }
}

/// PSNR (dB) between two RGB8 buffers of equal size, optionally restricted to `mask >= 128`.
pub fn psnr_rgb8(a: &[u8], b: &[u8], mask: Option<&[u8]>) -> f32 {
    let mut se = 0.0f64;
    let mut n = 0usize;
    for (i, (pa, pb)) in a.as_chunks::<3>().0.iter().zip(b.as_chunks::<3>().0).enumerate() {
        if mask.is_some_and(|m| m[i] < 128) {
            continue;
        }
        for k in 0..3 {
            let d = (pa[k] as f64 - pb[k] as f64) / 255.0;
            se += d * d;
        }
        n += 3;
    }
    if n == 0 {
        return f32::NAN;
    }
    let mse = se / n as f64;
    if mse <= 1e-12 { 99.0 } else { (-10.0 * mse.log10()) as f32 }
}
