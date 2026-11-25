//! Stateful SAM 2.1 video segmentation loop and DirectML-vs-CPU adapter self-test.

use crate::graphs::Sam2Graphs;
use crate::manifest::{ModelFiles, ModelVariant};
use crate::memory_bank::{MemoryBank, MemoryEntry};
use crate::postprocess::{FrameMask, FrameTimings, PostprocessConfig, threshold_mask};
use crate::preprocess::{FramePrompt, ResizeMode, preprocess_rgb};
use crate::runtime::{ActiveProvider, ExecutionPreference, OrtRuntime, SelfTestReport, enumerate_dxgi_adapters};
use crate::{Result, SegmentError};
use scancraft_core::RgbImage;
use std::time::Instant;

/// Minimum DirectML-vs-CPU IoU required by the adapter self-test.
pub const SELF_TEST_MIN_IOU: f32 = 0.98;

/// Per-variant evaluation entry emitted by [`Sam2VideoSegmenter::load_fastest_variant`].
#[derive(Debug, Clone)]
pub struct VariantBenchmark {
    pub variant: ModelVariant,
    pub provider: Option<ActiveProvider>,
    pub self_test_passed: bool,
    pub mean_tracked_ms: Option<f32>,
    pub note: String,
}

/// Result of benchmarking installed variants on the current hardware adapter.
#[derive(Debug, Clone)]
pub struct VariantSelectionReport {
    pub chosen: ModelVariant,
    pub provider: ActiveProvider,
    pub mean_tracked_ms: f32,
    pub benchmarks: Vec<VariantBenchmark>,
}

/// Stateful SAM 2.1 video segmenter carrying the memory bank across frames.
pub struct Sam2VideoSegmenter {
    graphs: Sam2Graphs,
    bank: MemoryBank,
    config: PostprocessConfig,
    resize_mode: ResizeMode,
}

impl Sam2VideoSegmenter {
    /// Loads the four SAM 2.1 graphs using `preference`.
    ///
    /// When `preference` is [`ExecutionPreference::Auto`], tries each DXGI DirectML adapter and
    /// validates it against CPU on a 2-frame synthetic clip before accepting it; if no GPU
    /// adapter passes, falls back to CPU.
    pub fn load(
        runtime: &OrtRuntime,
        models: &ModelFiles,
        preference: ExecutionPreference,
        config: PostprocessConfig,
    ) -> Result<(Self, Option<SelfTestReport>)> {
        models.verify_all()?;

        match preference {
            ExecutionPreference::Cpu => {
                let graphs = Sam2Graphs::load(runtime, models, ActiveProvider::Cpu)?;
                Ok((Self::from_graphs(graphs, config), None))
            }
            ExecutionPreference::DirectMl { device_id } => {
                let id = device_id.unwrap_or(0);
                let adapters = enumerate_dxgi_adapters();
                let adapter_name = adapters
                    .iter()
                    .find(|a| a.device_id == id)
                    .map(|a| a.name.clone())
                    .unwrap_or_else(|| format!("DXGI adapter {id}"));
                let graphs = Sam2Graphs::load(runtime, models, ActiveProvider::DirectMl { device_id: id, adapter_name })?;
                Ok((Self::from_graphs(graphs, config), None))
            }
            ExecutionPreference::Auto => {
                let adapters = enumerate_dxgi_adapters();
                let mut last_report = None;
                if !adapters.is_empty() {
                    let mut cpu_ref = Self::from_graphs(Sam2Graphs::load(runtime, models, ActiveProvider::Cpu)?, config.clone());
                    let cpu_clip = run_self_test_clip(&mut cpu_ref)?;

                    for adapter in adapters {
                        let provider = ActiveProvider::DirectMl {
                            device_id: adapter.device_id,
                            adapter_name: adapter.name.clone(),
                        };
                        match Sam2Graphs::load(runtime, models, provider.clone()) {
                            Ok(graphs) => {
                                let mut cand = Self::from_graphs(graphs, config.clone());
                                match run_self_test_clip(&mut cand) {
                                    Ok(cand_clip) => {
                                        let report = compare_self_test(provider.clone(), &cpu_clip, &cand_clip);
                                        if report.passed {
                                            cand.reset();
                                            return Ok((cand, Some(report)));
                                        }
                                        log::warn!("DirectML self-test failed for {provider}: {}", report.message);
                                        last_report = Some(report);
                                    }
                                    Err(e) => {
                                        log::warn!("DirectML inference failed during self-test on {provider}: {e}");
                                        last_report = Some(SelfTestReport {
                                            provider,
                                            frame0_iou: 0.0,
                                            frame1_iou: 0.0,
                                            max_logit_diff: f32::INFINITY,
                                            passed: false,
                                            message: e.to_string(),
                                        });
                                    }
                                }
                            }
                            Err(e) => {
                                log::warn!("DirectML session creation failed on {provider}: {e}");
                                last_report = Some(SelfTestReport {
                                    provider,
                                    frame0_iou: 0.0,
                                    frame1_iou: 0.0,
                                    max_logit_diff: f32::INFINITY,
                                    passed: false,
                                    message: e.to_string(),
                                });
                            }
                        }
                    }

                    cpu_ref.reset();
                    return Ok((cpu_ref, last_report));
                }

                let cpu = Self::from_graphs(Sam2Graphs::load(runtime, models, ActiveProvider::Cpu)?, config);
                Ok((cpu, None))
            }
        }
    }

    fn from_graphs(graphs: Sam2Graphs, config: PostprocessConfig) -> Self {
        let bank = MemoryBank::new(graphs.constants.clone());
        Self { graphs, bank, config, resize_mode: ResizeMode::DirectResize }
    }

    /// Evaluates candidate variants in `candidates` via the DirectML-vs-CPU self-test plus a short
    /// warmup + 3-frame timing clip and returns the fastest passing variant.
    ///
    /// Prefers passing DirectML candidates over CPU fallbacks whenever `preference` allows GPU.
    pub fn load_fastest_variant(
        runtime: &OrtRuntime,
        candidates: &[(ModelVariant, ModelFiles)],
        preference: ExecutionPreference,
        config: PostprocessConfig,
    ) -> Result<(Self, VariantSelectionReport)> {
        if candidates.is_empty() {
            return Err(SegmentError::InvalidInput("no model variants supplied".into()));
        }
        let mut benchmarks = Vec::with_capacity(candidates.len());
        let mut best_gpu: Option<(f32, ModelVariant, Self)> = None;
        let mut best_cpu: Option<(f32, ModelVariant, Self)> = None;

        for (variant, files) in candidates {
            match Self::load(runtime, files, preference.clone(), config.clone()) {
                Ok((mut seg, self_test)) => {
                    let provider = seg.provider().clone();
                    let passed = self_test.as_ref().map(|r| r.passed).unwrap_or(true);
                    match benchmark_short_clip(&mut seg) {
                        Ok(mean_ms) => {
                            seg.reset();
                            let is_gpu = matches!(provider, ActiveProvider::DirectMl { .. });
                            let note = self_test
                                .as_ref()
                                .map(|r| format!("{}; bench={mean_ms:.1} ms", r.message))
                                .unwrap_or_else(|| format!("bench={mean_ms:.1} ms"));
                            benchmarks.push(VariantBenchmark {
                                variant: *variant,
                                provider: Some(provider.clone()),
                                self_test_passed: passed,
                                mean_tracked_ms: Some(mean_ms),
                                note,
                            });
                            let target = if is_gpu && passed { &mut best_gpu } else { &mut best_cpu };
                            let replace = target.as_ref().is_none_or(|(best_ms, _, _)| mean_ms < *best_ms);
                            if replace {
                                *target = Some((mean_ms, *variant, seg));
                            }
                        }
                        Err(e) => {
                            benchmarks.push(VariantBenchmark {
                                variant: *variant,
                                provider: Some(provider),
                                self_test_passed: passed,
                                mean_tracked_ms: None,
                                note: format!("benchmark error: {e}"),
                            });
                        }
                    }
                }
                Err(e) => {
                    benchmarks.push(VariantBenchmark {
                        variant: *variant,
                        provider: None,
                        self_test_passed: false,
                        mean_tracked_ms: None,
                        note: format!("load error: {e}"),
                    });
                }
            }
        }

        let Some((best_ms, chosen, mut winner)) = best_gpu.or(best_cpu) else {
            let summary = benchmarks
                .iter()
                .map(|b| format!("{}: {}", b.variant.dir_name(), b.note))
                .collect::<Vec<_>>()
                .join("; ");
            return Err(SegmentError::NoUsableBackend(format!(
                "no SAM 2.1 variant passed loading: {summary}"
            )));
        };
        winner.reset();
        let report = VariantSelectionReport {
            chosen,
            provider: winner.provider().clone(),
            mean_tracked_ms: best_ms,
            benchmarks,
        };
        Ok((winner, report))
    }

    pub fn set_resize_mode(&mut self, mode: ResizeMode) {
        self.resize_mode = mode;
    }

    pub fn provider(&self) -> &ActiveProvider {
        &self.graphs.provider
    }

    pub fn constants(&self) -> &crate::memory_bank::ModelConstants {
        &self.graphs.constants
    }

    /// Clears the memory bank to start segmenting a new object.
    pub fn reset(&mut self) {
        self.bank.clear();
    }

    /// Segments `image` at `frame_index` using `prompt` (initial prompt or mid-sequence re-prompt),
    /// encoding the resulting mask as a conditioning memory.
    pub fn add_prompt(&mut self, frame_index: u64, image: &RgbImage, prompt: &FramePrompt) -> Result<FrameMask> {
        if prompt.is_empty() {
            return Err(SegmentError::InvalidInput("add_prompt requires at least one point or box".into()));
        }
        self.step_internal(frame_index, image, Some(prompt))
    }

    /// Propagates the tracked object mask into `image` at `frame_index` using the memory bank.
    pub fn propagate(&mut self, frame_index: u64, image: &RgbImage) -> Result<FrameMask> {
        if self.bank.is_empty() {
            return Err(SegmentError::NoPrompt);
        }
        self.step_internal(frame_index, image, None)
    }

    fn step_internal(&mut self, frame_index: u64, image: &RgbImage, prompt: Option<&FramePrompt>) -> Result<FrameMask> {
        let total_t0 = Instant::now();
        let model_size = self.graphs.constants.image_size;

        let t0 = Instant::now();
        let (pixel_values, tf) = preprocess_rgb(image, model_size, self.resize_mode)?;
        let empty = FramePrompt::default();
        let (input_points, input_labels) = prompt.unwrap_or(&empty).encode(&tf);
        let preprocess_ms = t0.elapsed().as_secs_f32() * 1000.0;

        let t1 = Instant::now();
        let vfeats = self.graphs.encode_image(&pixel_values)?;
        let vision_encoder_ms = t1.elapsed().as_secs_f32() * 1000.0;

        let t2 = Instant::now();
        let cond_feats = if let Some(assembled) = self.bank.assemble_for_frame(frame_index) {
            self.graphs.attend_memory(&vfeats.feats2, &vfeats.pos2, &assembled)?
        } else if let Some(no_mem) = &self.graphs.constants.no_memory_embedding {
            let mut out = vfeats.feats2.clone();
            let (_, c, fh, fw) = out.dim();
            if no_mem.len() == c {
                for ch in 0..c {
                    let add = no_mem[ch];
                    for y in 0..fh {
                        for x in 0..fw {
                            out[[0, ch, y, x]] += add;
                        }
                    }
                }
            }
            out
        } else {
            vfeats.feats2.clone()
        };
        let memory_attention_ms = if prompt.is_none() || !self.bank.is_empty() {
            t2.elapsed().as_secs_f32() * 1000.0
        } else {
            0.0
        };

        let t3 = Instant::now();
        let decoded = self
            .graphs
            .decode_mask(&vfeats.feats0, &vfeats.feats1, &cond_feats, &input_points, &input_labels)?;
        let mask_decoder_ms = t3.elapsed().as_secs_f32() * 1000.0;

        let is_from_pts = prompt.is_some();
        let t4 = Instant::now();
        let (memory_tokens, memory_pos_enc) = self.graphs.encode_memory(
            &vfeats.feats2,
            &decoded.high_res_mask,
            decoded.object_score_logit,
            is_from_pts,
        )?;
        let memory_encoder_ms = t4.elapsed().as_secs_f32() * 1000.0;

        self.bank.insert(MemoryEntry {
            frame_index,
            memory_tokens,
            memory_pos_enc,
            object_pointer: decoded.object_pointer,
            object_score_logit: decoded.object_score_logit,
            is_conditioning: is_from_pts,
        })?;

        let t5 = Instant::now();
        let hr_slice = decoded.high_res_mask.as_slice().ok_or_else(|| SegmentError::InvalidShape {
            name: "high_res_mask",
            detail: "non-contiguous mask".into(),
        })?;
        let s = model_size as usize;
        let preliminary_timings = FrameTimings {
            preprocess_ms,
            vision_encoder_ms,
            memory_attention_ms,
            mask_decoder_ms,
            memory_encoder_ms,
            postprocess_ms: 0.0,
            total_ms: 0.0,
        };
        let mut frame_mask = threshold_mask(
            frame_index,
            hr_slice,
            s,
            s,
            &tf,
            decoded.iou_score,
            decoded.object_score_logit,
            &self.config,
            preliminary_timings,
        );
        frame_mask.timings.postprocess_ms = t5.elapsed().as_secs_f32() * 1000.0;
        frame_mask.timings.total_ms = total_t0.elapsed().as_secs_f32() * 1000.0;
        Ok(frame_mask)
    }
}

struct ClipOutput {
    f0: FrameMask,
    f1: FrameMask,
}

fn synthetic_test_frame(cx: usize, cy: usize) -> RgbImage {
    let (w, h) = (128usize, 128usize);
    let mut data = vec![24u8; w * h * 3];
    for y in 0..h {
        for x in 0..w {
            let i = (y * w + x) * 3;
            if x.abs_diff(cx) <= 22 && y.abs_diff(cy) <= 22 {
                data[i] = 230;
                data[i + 1] = 175;
                data[i + 2] = 60;
            } else {
                data[i] = 28;
                data[i + 1] = 36 + ((x + y) % 12) as u8;
                data[i + 2] = 48;
            }
        }
    }
    RgbImage { width: w as u32, height: h as u32, channels: 3, data }
}

fn run_self_test_clip(seg: &mut Sam2VideoSegmenter) -> Result<ClipOutput> {
    seg.reset();
    let img0 = synthetic_test_frame(56, 64);
    let img1 = synthetic_test_frame(66, 64);
    let prompt = FramePrompt::default().with_point(56.0, 64.0, true);
    let f0 = seg.add_prompt(0, &img0, &prompt)?;
    let f1 = seg.propagate(1, &img1)?;
    Ok(ClipOutput { f0, f1 })
}

fn benchmark_short_clip(seg: &mut Sam2VideoSegmenter) -> Result<f32> {
    seg.reset();
    let prompt = FramePrompt::default().with_point(56.0, 64.0, true);
    let img0 = synthetic_test_frame(56, 64);
    let _ = seg.add_prompt(0, &img0, &prompt)?;
    // Warm up on frame 1, then time 3 tracked frames (2..=4).
    let img1 = synthetic_test_frame(60, 64);
    let _ = seg.propagate(1, &img1)?;
    let mut sum_ms = 0.0f32;
    for i in 2..=4usize {
        let img = synthetic_test_frame(60 + i * 2, 64);
        let res = seg.propagate(i as u64, &img)?;
        sum_ms += res.timings.total_ms;
    }
    Ok(sum_ms / 3.0)
}

fn compare_self_test(provider: ActiveProvider, cpu: &ClipOutput, cand: &ClipOutput) -> SelfTestReport {
    let frame0_iou = cpu.f0.mask.iou(&cand.f0.mask);
    let frame1_iou = cpu.f1.mask.iou(&cand.f1.mask);
    let max_logit_diff = (cpu.f0.object_score_logit - cand.f0.object_score_logit)
        .abs()
        .max((cpu.f1.object_score_logit - cand.f1.object_score_logit).abs())
        .max((cpu.f0.iou_score - cand.f0.iou_score).abs())
        .max((cpu.f1.iou_score - cand.f1.iou_score).abs());
    let finite = cand.f0.iou_score.is_finite()
        && cand.f1.iou_score.is_finite()
        && cand.f0.object_score_logit.is_finite()
        && cand.f1.object_score_logit.is_finite();
    let passed = finite && frame0_iou >= SELF_TEST_MIN_IOU && frame1_iou >= SELF_TEST_MIN_IOU;
    let message = if passed {
        format!("passed (f0 IoU={frame0_iou:.4}, f1 IoU={frame1_iou:.4}, max_score_diff={max_logit_diff:.4})")
    } else {
        format!("failed (finite={finite}, f0 IoU={frame0_iou:.4}, f1 IoU={frame1_iou:.4}, max_score_diff={max_logit_diff:.4})")
    };
    SelfTestReport { provider, frame0_iou, frame1_iou, max_logit_diff, passed, message }
}
