//! Training a Gaussian splat with Brush's trainer (`brush_train::train::SplatTrainer`).
//!
//! We drive Brush's per-step trainer directly instead of its `brush-process` pipeline: that
//! pipeline expects a dataset on a virtual file system, pulls in rerun logging and writes its
//! own checkpoints. The loop below mirrors `brush_process::train_stream` (step, refine on a
//! schedule, publish) for the parts we need, with in-memory images, our own initialisation,
//! progress callback and cancellation.

use crate::convert::{MM_TO_M, brush_camera};
use crate::coverage::{self, CoverageConfig};
use crate::device::SplatDevice;
use crate::types::{PosedImage, SplatDataset};
use crate::{SplatError, TrainedSplat};
use brush_dataset::scene::{SceneBatch, view_to_packed_data};
use brush_render::AlphaMode;
use brush_render::camera::Camera;
use brush_render::gaussian_splats::{SplatRenderMode, Splats, inverse_sigmoid};
use brush_render::sh::rgb_to_sh;
use brush_train::config::TrainConfig;
use brush_train::train::{BOUND_PERCENTILE, SplatTrainer, get_splat_bounds};
use burn::module::Module;
use image::DynamicImage;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

pub use brush_train::config::TrainConfig as BrushTrainConfig;

/// How the splat treats image masks.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum MaskMode {
    /// Masked-out pixels are a transparent target: the splat learns to be empty there, giving a
    /// clean object splat without background (default for object scans).
    #[default]
    Transparent,
    /// Masked-out pixels are ignored by the loss (background is left unconstrained).
    IgnoreBackground,
}

/// Where the initial splat colours come from.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum InitColors {
    /// Mean colour the RGB keyframes observe at each point (works for grey geometry-only clouds).
    #[default]
    FromImages,
    /// The point cloud's own vertex colours.
    FromCloud,
}

/// Training settings. Defaults are tuned for a turntable/handheld object scan with tens to a
/// few hundred keyframes.
#[derive(Clone)]
pub struct SplatTrainConfig {
    /// Optimisation steps (one image per step).
    pub iterations: u32,
    /// Spherical-harmonics degree of the result (0 = view-independent colour, 3 = full).
    pub sh_degree: u32,
    /// Images are downscaled so their long edge is at most this many pixels.
    pub max_resolution: u32,
    /// Upper bound of initial splats; the covered cloud is subsampled to this.
    pub max_init_points: usize,
    /// Upper bound on the number of splats during growth.
    pub max_splats: u32,
    pub mask_mode: MaskMode,
    pub init_colors: InitColors,
    pub coverage: CoverageConfig,
    /// Hold out every n-th image (n >= 2) for evaluation instead of training on it.
    pub eval_every: Option<usize>,
    /// Call the progress callback every this many steps.
    pub progress_every: u32,
    pub seed: u64,
    /// Full Brush trainer configuration. `None` derives it from the fields above; when set it is
    /// used as is except for `total_train_iters` and `max_splats`.
    pub brush: Option<BrushTrainConfig>,
}

impl Default for SplatTrainConfig {
    fn default() -> Self {
        Self {
            iterations: 7000,
            sh_degree: 3,
            max_resolution: 1280,
            max_init_points: 300_000,
            max_splats: 3_000_000,
            mask_mode: MaskMode::default(),
            init_colors: InitColors::default(),
            coverage: CoverageConfig::default(),
            eval_every: None,
            progress_every: 50,
            seed: 42,
            brush: None,
        }
    }
}

impl SplatTrainConfig {
    /// The Brush trainer configuration these settings amount to.
    pub fn brush_config(&self) -> BrushTrainConfig {
        let mut c = self.brush.clone().unwrap_or_else(|| {
            // Brush's schedule is laid out for 30k steps: growth stops halfway, refinement every
            // 200 steps. Keep the proportions for shorter runs.
            TrainConfig {
                growth_stop_iter: self.iterations / 2,
                refine_every: (self.iterations / 35).clamp(50, 200),
                ..TrainConfig::default()
            }
        });
        c.total_train_iters = self.iterations;
        c.max_splats = self.max_splats;
        c
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Phase {
    Preparing,
    Training,
    Evaluating,
    Done,
}

/// Progress report passed to the callback.
#[derive(Debug, Clone)]
pub struct TrainProgress {
    pub phase: Phase,
    pub iteration: u32,
    pub total_iterations: u32,
    pub num_splats: u32,
    /// Training loss of the latest reported step.
    pub loss: Option<f32>,
    pub elapsed: Duration,
}

/// PSNR / SSIM over a set of views.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EvalStats {
    pub views: usize,
    pub psnr: f32,
    pub ssim: f32,
}

/// What a training run did.
#[derive(Debug, Clone)]
pub struct TrainReport {
    pub iterations: u32,
    pub train_views: usize,
    pub eval_views: usize,
    /// Initial cloud size, and how many of its points survived the RGB-coverage filter.
    pub init_points_in: usize,
    pub init_points_covered: usize,
    pub init_splats: u32,
    pub final_splats: u32,
    pub prepare_time: Duration,
    pub train_time: Duration,
    /// Held-out views (if `eval_every` was set).
    pub eval: Option<EvalStats>,
    /// A sample of training views, for comparison with `eval`.
    pub train_eval: Option<EvalStats>,
}

pub(crate) struct View {
    pub camera: Camera,
    pub image: DynamicImage,
    pub alpha_mode: AlphaMode,
    pub native_focal: f32,
}

fn prepare_view(img: &PosedImage, cfg: &SplatTrainConfig, alpha_mode: AlphaMode) -> View {
    let camera = brush_camera(img);
    let mut image = img.to_dynamic_image();
    let long = img.width.max(img.height);
    if long > cfg.max_resolution {
        // Brush cameras are resolution independent (fov + normalised principal point).
        let s = cfg.max_resolution as f32 / long as f32;
        let (w, h) = (((img.width as f32 * s).round() as u32).max(1), ((img.height as f32 * s).round() as u32).max(1));
        image = image.resize_exact(w, h, image::imageops::FilterType::Triangle);
    }
    let native_focal = camera.focal(brush_glam::uvec2(img.width, img.height)).x;
    View { camera, image, alpha_mode, native_focal }
}

fn batch(view: &View) -> SceneBatch {
    let (img_packed, has_alpha) = view_to_packed_data(view.image.clone(), view.alpha_mode);
    SceneBatch { img_packed, has_alpha, alpha_mode: view.alpha_mode, camera: view.camera }
}

/// Initial splats from the covered cloud, in metres.
fn init_splats(dataset: &SplatDataset, cfg: &SplatTrainConfig, device: &SplatDevice) -> Result<(Splats, usize), SplatError> {
    let cover = coverage::analyse(&dataset.init_cloud, &dataset.images, &cfg.coverage);
    let covered: Vec<(glam::Vec3, [f32; 3])> = dataset
        .init_cloud
        .points
        .iter()
        .zip(&cover)
        .filter(|(_, c)| c.views >= cfg.coverage.min_views)
        .map(|(p, c)| {
            let rgb = match cfg.init_colors {
                InitColors::FromImages => c.mean_rgb,
                InitColors::FromCloud => [p.color.r as f32 / 255.0, p.color.g as f32 / 255.0, p.color.b as f32 / 255.0],
            };
            (glam::Vec3::from_array(p.position) * MM_TO_M, rgb)
        })
        .collect();
    let n_covered = covered.len();
    if covered.len() < 4 {
        return Err(SplatError::InvalidInput(format!(
            "only {} of {} initial points are seen by a colour image",
            covered.len(),
            dataset.init_cloud.points.len()
        )));
    }
    let step = covered.len().div_ceil(cfg.max_init_points.max(1));
    let pts: Vec<_> = covered.into_iter().step_by(step).collect();
    let n = pts.len();

    let means: Vec<f32> = pts.iter().flat_map(|(p, _)| p.to_array()).collect();
    let log_scales = knn_log_scales(&pts.iter().map(|(p, _)| *p).collect::<Vec<_>>());
    let sh: Vec<f32> = pts
        .iter()
        .flat_map(|(_, c)| rgb_to_sh(brush_glam::Vec3::from_array(*c)).to_array())
        .collect();
    let rotations = [1.0f32, 0.0, 0.0, 0.0].repeat(n);
    let opacities = vec![inverse_sigmoid(0.5); n];
    let splats = Splats::from_raw(means, rotations, log_scales, sh, opacities, SplatRenderMode::Default, &device.device)
        .with_sh_degree(cfg.sh_degree);
    Ok((splats, n_covered))
}

/// Isotropic log-scale per point: half the mean distance to its two nearest neighbours (Brush's
/// rule), without Brush's 1 mm lower clamp, which is far above the POP 4's point spacing.
fn knn_log_scales(points: &[glam::Vec3]) -> Vec<f32> {
    use kiddo::{ImmutableKdTree, SquaredEuclidean};
    use rayon::prelude::*;
    // kiddo panics when many points share a coordinate on one axis (flat axis-aligned surfaces);
    // break ties with a deterministic jitter below a micrometre, as scancraft-slam does.
    let jittered: Vec<[f32; 3]> = points
        .iter()
        .enumerate()
        .map(|(i, p)| {
            let h = (i as u32).wrapping_mul(2_654_435_761);
            let j = |shift: u32| ((h >> shift) & 0xff) as f32 * 4e-9;
            [p.x + j(0), p.y + j(8), p.z + j(16)]
        })
        .collect();
    let tree = ImmutableKdTree::<f32, 3>::new_from_slice(&jittered);
    let mut dists: Vec<f32> = jittered
        .par_iter()
        .map(|p| {
            let nn = tree.nearest_n::<SquaredEuclidean>(p, 3);
            let d: Vec<f32> = nn.iter().skip(1).map(|n| n.distance.sqrt()).collect();
            if d.is_empty() { 0.0 } else { d.iter().sum::<f32>() / d.len() as f32 * 0.5 }
        })
        .collect();
    let mut sorted: Vec<f32> = dists.iter().copied().filter(|d| *d > 0.0).collect();
    sorted.sort_by(f32::total_cmp);
    let median = sorted.get(sorted.len() / 2).copied().unwrap_or(1e-3);
    for d in &mut dists {
        // Duplicates get the median spacing; outliers are capped at 10x it.
        *d = if *d > 0.0 { d.clamp(median * 0.1, median * 10.0) } else { median };
    }
    dists.into_iter().flat_map(|d| { let l = d.max(1e-7).ln(); [l, l, l] }).collect()
}

/// Simple deterministic shuffle (splitmix64-driven Fisher-Yates); keeps `rand` out of our API.
fn shuffle(order: &mut [usize], state: &mut u64) {
    for i in (1..order.len()).rev() {
        *state = state.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = *state;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^= z >> 31;
        order.swap(i, (z % (i as u64 + 1)) as usize);
    }
}

pub(crate) async fn evaluate(splats: &Splats, views: &[&View], device: &SplatDevice) -> Result<Option<EvalStats>, SplatError> {
    if views.is_empty() {
        return Ok(None);
    }
    let (mut psnr, mut ssim) = (0.0, 0.0);
    for v in views {
        let s = brush_train::eval::eval_stats(splats.clone(), &v.camera, v.image.clone(), v.alpha_mode, &device.device)
            .await
            .map_err(|e| SplatError::Brush(e.to_string()))?;
        psnr += s.psnr.into_scalar_async::<f32>().await.map_err(|e| SplatError::Brush(format!("{e:?}")))?;
        ssim += s.ssim.into_scalar_async::<f32>().await.map_err(|e| SplatError::Brush(format!("{e:?}")))?;
    }
    let n = views.len() as f32;
    Ok(Some(EvalStats { views: views.len(), psnr: psnr / n, ssim: ssim / n }))
}

/// Train a splat. Blocks the calling thread for the whole run; use [`crate::SplatJob`] to run it
/// in the background.
pub fn train(
    dataset: &SplatDataset,
    cfg: &SplatTrainConfig,
    device: &SplatDevice,
    progress: &mut dyn FnMut(&TrainProgress),
    cancel: &AtomicBool,
) -> Result<TrainedSplat, SplatError> {
    let rt = tokio::runtime::Builder::new_multi_thread()
        .worker_threads(2)
        .enable_all()
        .build()
        .map_err(|e| SplatError::Brush(format!("tokio runtime: {e}")))?;
    rt.block_on(train_async(dataset, cfg, device, progress, cancel))
}

async fn train_async(
    dataset: &SplatDataset,
    cfg: &SplatTrainConfig,
    device: &SplatDevice,
    progress: &mut dyn FnMut(&TrainProgress),
    cancel: &AtomicBool,
) -> Result<TrainedSplat, SplatError> {
    let start = Instant::now();
    dataset.validate()?;
    let total = cfg.iterations;
    let report = |progress: &mut dyn FnMut(&TrainProgress), phase, iteration, num_splats, loss| {
        progress(&TrainProgress { phase, iteration, total_iterations: total, num_splats, loss, elapsed: start.elapsed() });
    };
    report(progress, Phase::Preparing, 0, 0, None);

    let has_masks = dataset.images.iter().any(|i| i.mask.is_some());
    let alpha_mode = match cfg.mask_mode {
        MaskMode::Transparent => AlphaMode::Transparent,
        MaskMode::IgnoreBackground => AlphaMode::Masked,
    };
    let views: Vec<View> = {
        use rayon::prelude::*;
        dataset.images.par_iter().map(|img| prepare_view(img, cfg, alpha_mode)).collect()
    };
    let is_eval = |i: usize| cfg.eval_every.is_some_and(|n| n >= 2 && i % n == n / 2);
    let train_idx: Vec<usize> = (0..views.len()).filter(|&i| !is_eval(i)).collect();
    let eval_idx: Vec<usize> = (0..views.len()).filter(|&i| is_eval(i)).collect();
    if train_idx.is_empty() {
        return Err(SplatError::InvalidInput("no training views left after the eval split".into()));
    }
    // Pack every training view once (CPU); each step only clones the packed buffer.
    let batches: Vec<SceneBatch> = train_idx.iter().map(|&i| batch(&views[i])).collect();
    log::info!(
        "splat: {} train / {} eval views at {}x{} (masks: {has_masks})",
        batches.len(),
        eval_idx.len(),
        views[0].image.width(),
        views[0].image.height()
    );

    let dev = &device.device;
    dev.seed(cfg.seed);
    let (mut splats, n_covered) = init_splats(dataset, cfg, device)?;
    let init_splats = splats.num_splats();
    let bounds = get_splat_bounds(splats.clone(), BOUND_PERCENTILE).await;
    let brush_cfg = cfg.brush_config();
    let mut trainer = SplatTrainer::new_seeded(&brush_cfg, dev, bounds, cfg.seed);
    trainer.set_view_cams(train_idx.iter().map(|&i| (views[i].camera.position, views[i].native_focal)).collect());
    dev.memory_cleanup();
    let prepare_time = start.elapsed();
    log::info!("splat: {init_splats} initial splats ({n_covered} covered points), prepared in {prepare_time:?}");

    let train_start = Instant::now();
    let mut order: Vec<usize> = (0..batches.len()).collect();
    let mut rng = cfg.seed ^ 0x5CA9_C2AF_7000_0001;
    let mut cursor = order.len();
    for iter in 0..total {
        if cancel.load(Ordering::Relaxed) {
            return Err(SplatError::Cancelled);
        }
        if cursor == order.len() {
            shuffle(&mut order, &mut rng);
            cursor = 0;
        }
        let b = batches[order[cursor]].clone();
        cursor += 1;

        let (new_splats, stats) = trainer.step(b, splats.train()).await;
        splats = new_splats.valid();

        let progress_frac = iter as f32 / total as f32;
        if iter > 0 && iter.is_multiple_of(brush_cfg.refine_every) && progress_frac <= 0.95 {
            let (refined, _) = trainer.refine(iter, splats).await;
            splats = refined;
        }

        let done = iter + 1;
        if done.is_multiple_of(cfg.progress_every.max(1)) || done == total {
            let loss = stats.loss.into_scalar_async::<f32>().await.ok();
            report(progress, Phase::Training, done, splats.num_splats(), loss);
        }
    }
    let train_time = train_start.elapsed();

    report(progress, Phase::Evaluating, total, splats.num_splats(), None);
    let eval_views: Vec<&View> = eval_idx.iter().map(|&i| &views[i]).collect();
    let eval = evaluate(&splats, &eval_views, device).await?;
    // Up to 8 evenly spread training views.
    let stride = train_idx.len().div_ceil(8);
    let train_sample: Vec<&View> = train_idx.iter().step_by(stride).map(|&i| &views[i]).collect();
    let train_eval = evaluate(&splats, &train_sample, device).await?;

    let report_out = TrainReport {
        iterations: total,
        train_views: train_idx.len(),
        eval_views: eval_idx.len(),
        init_points_in: dataset.init_cloud.points.len(),
        init_points_covered: n_covered,
        init_splats,
        final_splats: splats.num_splats(),
        prepare_time,
        train_time,
        eval,
        train_eval,
    };
    report(progress, Phase::Done, total, splats.num_splats(), None);
    Ok(TrainedSplat { splats, device: device.clone(), report: report_out })
}

/// A training run on a background thread.
pub struct SplatJob {
    cancel: Arc<AtomicBool>,
    progress: std::sync::mpsc::Receiver<TrainProgress>,
    handle: Option<std::thread::JoinHandle<Result<TrainedSplat, SplatError>>>,
    latest: Option<TrainProgress>,
}

impl SplatJob {
    /// Start training on a new thread with Brush's default device.
    pub fn spawn(dataset: SplatDataset, cfg: SplatTrainConfig) -> Self {
        Self::spawn_on(dataset, cfg, None)
    }

    /// Start training on a new thread, on `device` (or Brush's default device).
    pub fn spawn_on(dataset: SplatDataset, cfg: SplatTrainConfig, device: Option<SplatDevice>) -> Self {
        let cancel = Arc::new(AtomicBool::new(false));
        let (tx, rx) = std::sync::mpsc::channel();
        let flag = cancel.clone();
        let handle = std::thread::Builder::new()
            .name("scancraft-splat".into())
            .spawn(move || {
                let device = device.unwrap_or_else(SplatDevice::default_device);
                train(&dataset, &cfg, &device, &mut |p| { let _ = tx.send(p.clone()); }, &flag)
            })
            .expect("spawn splat thread");
        Self { cancel, progress: rx, handle: Some(handle), latest: None }
    }

    /// Request cancellation; [`SplatJob::join`] then returns [`SplatError::Cancelled`].
    pub fn cancel(&self) {
        self.cancel.store(true, Ordering::Relaxed);
    }

    /// Latest progress (non-blocking; drains the channel). For a UI's per-frame poll.
    pub fn poll(&mut self) -> Option<&TrainProgress> {
        while let Ok(p) = self.progress.try_recv() {
            self.latest = Some(p);
        }
        self.latest.as_ref()
    }

    pub fn is_finished(&self) -> bool {
        self.handle.as_ref().is_none_or(|h| h.is_finished())
    }

    /// Wait for the result.
    pub fn join(mut self) -> Result<TrainedSplat, SplatError> {
        let handle = self.handle.take().expect("joined once");
        handle.join().unwrap_or_else(|_| Err(SplatError::Brush("training thread panicked".into())))
    }
}

impl Drop for SplatJob {
    fn drop(&mut self) {
        // Dropping an unfinished job cancels it rather than leaving a GPU-hogging thread behind.
        self.cancel();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn shuffle_is_a_permutation() {
        let mut v: Vec<usize> = (0..50).collect();
        let mut s = 7;
        shuffle(&mut v, &mut s);
        let mut sorted = v.clone();
        sorted.sort();
        assert_eq!(sorted, (0..50).collect::<Vec<_>>());
        assert_ne!(v, sorted);
    }

    #[test]
    fn knn_scale_matches_spacing() {
        let pts: Vec<glam::Vec3> = (0..20).flat_map(|i| (0..20).map(move |j| glam::Vec3::new(i as f32, j as f32, 0.0) * 1e-4)).collect();
        let s = knn_log_scales(&pts);
        // Grid spacing 0.1 mm -> half the NN distance = 0.05 mm.
        assert!((s[0].exp() - 5e-5).abs() < 1e-6, "{}", s[0].exp());
    }

    #[test]
    fn short_runs_keep_brush_proportions() {
        let c = SplatTrainConfig { iterations: 3000, ..Default::default() }.brush_config();
        assert_eq!(c.total_train_iters, 3000);
        assert_eq!(c.growth_stop_iter, 1500);
        assert!(c.refine_every >= 50 && c.refine_every <= 200);
    }
}
