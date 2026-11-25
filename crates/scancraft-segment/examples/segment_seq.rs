//! Run SAM 2.1 video segmentation on a folder of PNG/JPG frames, write mask & overlay PNGs,
//! and report per-graph timings and the chosen execution provider/adapter.
//!
//! Example:
//! ```text
//! cargo run --release -p scancraft-segment --example segment_seq -- \
//!   --input C:\temp\frames --output C:\temp\out --point 320,240 --backend auto
//! ```

use anyhow::{Context, Result, bail};
use scancraft_core::RgbImage;
use scancraft_segment::{
    ExecutionPreference, FrameMask, FramePrompt, ModelFiles, ModelVariant, OrtRuntime, PostprocessConfig,
    RuntimeFiles, Sam2VideoSegmenter,
};
use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

struct CliArgs {
    input: PathBuf,
    output: PathBuf,
    model_dir: PathBuf,
    runtime_dir: PathBuf,
    variant: ModelVariant,
    backend: ExecutionPreference,
    save_masks: Option<PathBuf>,
    no_overlay: bool,
    initial_prompt: FramePrompt,
    reprompts: BTreeMap<usize, FramePrompt>,
}

fn default_ai_subdir(sub: &str) -> PathBuf {
    let local = std::env::var_os("LOCALAPPDATA")
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from("."));
    local.join("ScanCraft").join("ai").join(sub)
}

fn parse_xy(s: &str) -> Result<(f32, f32)> {
    let parts: Vec<&str> = s.split(',').collect();
    if parts.len() != 2 {
        bail!("expected x,y pair, got `{s}`");
    }
    Ok((parts[0].trim().parse()?, parts[1].trim().parse()?))
}

fn parse_box(s: &str) -> Result<[f32; 4]> {
    let parts: Vec<&str> = s.split(',').collect();
    if parts.len() != 4 {
        bail!("expected x0,y0,x1,y1 box, got `{s}`");
    }
    Ok([
        parts[0].trim().parse()?,
        parts[1].trim().parse()?,
        parts[2].trim().parse()?,
        parts[3].trim().parse()?,
    ])
}

fn parse_args() -> Result<CliArgs> {
    let mut input = None;
    let mut output = None;
    let mut variant = ModelVariant::Sam21Tiny1024Fp32;
    let mut model_dir: Option<PathBuf> = std::env::var_os("SCANCRAFT_SAM2_DIR").map(PathBuf::from);
    let mut runtime_dir = std::env::var_os("SCANCRAFT_ORT_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|| default_ai_subdir("runtime/ort-1.24.4-dml"));
    let mut backend = ExecutionPreference::Auto;
    let mut save_masks = None;
    let mut no_overlay = false;
    let mut initial_prompt = FramePrompt::default();
    let mut reprompts: BTreeMap<usize, FramePrompt> = BTreeMap::new();

    let mut it = std::env::args().skip(1);
    while let Some(arg) = it.next() {
        match arg.as_str() {
            "--input" => input = Some(PathBuf::from(it.next().context("missing --input value")?)),
            "--output" => output = Some(PathBuf::from(it.next().context("missing --output value")?)),
            "--variant" => {
                let val = it.next().context("missing --variant value")?;
                variant = ModelVariant::from_dir_name(&val)
                    .with_context(|| format!("unknown --variant `{val}`"))?;
            }
            "--model-dir" => model_dir = Some(PathBuf::from(it.next().context("missing --model-dir value")?)),
            "--runtime-dir" => runtime_dir = PathBuf::from(it.next().context("missing --runtime-dir value")?),
            "--save-masks" => save_masks = Some(PathBuf::from(it.next().context("missing --save-masks value")?)),
            "--no-overlay" => no_overlay = true,
            "--backend" => {
                let val = it.next().context("missing --backend value")?;
                backend = match val.to_ascii_lowercase().as_str() {
                    "auto" => ExecutionPreference::Auto,
                    "directml" | "dml" => ExecutionPreference::DirectMl { device_id: None },
                    "cpu" => ExecutionPreference::Cpu,
                    _ => bail!("unknown --backend `{val}` (expected auto|directml|cpu)"),
                };
            }
            "--point" => {
                let (x, y) = parse_xy(&it.next().context("missing --point value")?)?;
                initial_prompt.points.push(scancraft_segment::PointPrompt::positive(x, y));
            }
            "--neg-point" => {
                let (x, y) = parse_xy(&it.next().context("missing --neg-point value")?)?;
                initial_prompt.points.push(scancraft_segment::PointPrompt::negative(x, y));
            }
            "--box" => {
                let b = parse_box(&it.next().context("missing --box value")?)?;
                initial_prompt = initial_prompt.with_box(b[0], b[1], b[2], b[3]);
            }
            "--reprompt" => {
                let spec = it.next().context("missing --reprompt value")?;
                let parts: Vec<&str> = spec.split(':').collect();
                if parts.len() < 2 || parts.len() > 3 {
                    bail!("expected <frame>:<x,y>[:pos|neg], got `{spec}`");
                }
                let frame_idx: usize = parts[0].parse()?;
                let (x, y) = parse_xy(parts[1])?;
                let positive = parts.get(2).map(|s| !s.eq_ignore_ascii_case("neg")).unwrap_or(true);
                reprompts
                    .entry(frame_idx)
                    .or_default()
                    .points
                    .push(scancraft_segment::PointPrompt { x, y, positive });
            }
            "--help" | "-h" => {
                println!(
                    "Usage: segment_seq --input <dir> --output <dir> --point <x,y> [--variant <1024-fp32|512-mem4-fp32|512-mem4-fp16|512-mem2-fp32|512-mem2-fp16>] [--save-masks <dir>] [--no-overlay] [--neg-point <x,y>] [--box <x0,y0,x1,y1>] [--reprompt <idx>:<x,y>[:pos|neg]] [--backend auto|directml|cpu] [--model-dir <dir>] [--runtime-dir <dir>]"
                );
                std::process::exit(0);
            }
            other => bail!("unknown argument `{other}`"),
        }
    }

    let input = input.context("--input <dir> is required")?;
    let output = output.context("--output <dir> is required")?;
    if initial_prompt.is_empty() {
        bail!("at least one --point or --box prompt is required for frame 0");
    }
    let model_dir = model_dir.unwrap_or_else(|| {
        let exports = default_ai_subdir("exports").join(variant.dir_name());
        if exports.is_dir() { exports } else { default_ai_subdir("models").join(variant.dir_name()) }
    });
    Ok(CliArgs { input, output, model_dir, runtime_dir, variant, backend, save_masks, no_overlay, initial_prompt, reprompts })
}

fn collect_frames(dir: &Path) -> Result<Vec<PathBuf>> {
    let mut files: Vec<PathBuf> = std::fs::read_dir(dir)
        .with_context(|| format!("reading {}", dir.display()))?
        .filter_map(|e| e.ok().map(|e| e.path()))
        .filter(|p| {
            p.is_file()
                && p.extension()
                    .and_then(|e| e.to_str())
                    .is_some_and(|ext| matches!(ext.to_ascii_lowercase().as_str(), "png" | "jpg" | "jpeg"))
        })
        .collect();
    files.sort();
    if files.is_empty() {
        bail!("no PNG/JPEG frames found in {}", dir.display());
    }
    Ok(files)
}

fn load_rgb(path: &Path) -> Result<RgbImage> {
    let dyn_img = image::open(path).with_context(|| format!("decoding {}", path.display()))?;
    let rgb = dyn_img.to_rgb8();
    let (width, height) = rgb.dimensions();
    Ok(RgbImage { width, height, channels: 3, data: rgb.into_raw() })
}

fn write_outputs(out_dir: &Path, stem: &str, src: &RgbImage, res: &FrameMask, prompt: Option<&FramePrompt>) -> Result<()> {
    let mask_path = out_dir.join(format!("{stem}_mask.png"));
    let mask_img = image::GrayImage::from_raw(res.mask.width, res.mask.height, res.mask.data.clone())
        .context("failed to build GrayImage")?;
    mask_img.save(&mask_path)?;

    let (w, h) = (src.width as usize, src.height as usize);
    let mut overlay = src.data.clone();
    for y in 0..h {
        for x in 0..w {
            let i = y * w + x;
            if res.mask.data[i] > 127 {
                // Check if on mask boundary.
                let edge = x == 0
                    || y == 0
                    || x + 1 == w
                    || y + 1 == h
                    || res.mask.data[i - 1] <= 127
                    || res.mask.data[i + 1] <= 127
                    || res.mask.data[i - w] <= 127
                    || res.mask.data[i + w] <= 127;
                let px = i * 3;
                if edge {
                    overlay[px] = 40;
                    overlay[px + 1] = 255;
                    overlay[px + 2] = 240;
                } else {
                    overlay[px] = (overlay[px] as f32 * 0.65) as u8;
                    overlay[px + 1] = (overlay[px + 1] as f32 * 0.65 + 255.0 * 0.35) as u8;
                    overlay[px + 2] = (overlay[px + 2] as f32 * 0.65) as u8;
                }
            }
        }
    }

    if let Some(p) = prompt {
        for pt in &p.points {
            let color = if pt.positive { [255u8, 240, 40] } else { [255, 50, 50] };
            let (cx, cy) = (pt.x.round() as isize, pt.y.round() as isize);
            for dy in -4..=4 {
                for dx in -4..=4 {
                    if dx * dx + dy * dy <= 16 {
                        let (xx, yy) = (cx + dx, cy + dy);
                        if xx >= 0 && yy >= 0 && (xx as usize) < w && (yy as usize) < h {
                            let px = (yy as usize * w + xx as usize) * 3;
                            overlay[px..px + 3].copy_from_slice(&color);
                        }
                    }
                }
            }
        }
    }

    let overlay_path = out_dir.join(format!("{stem}_overlay.png"));
    let ov_img = image::RgbImage::from_raw(src.width, src.height, overlay).context("failed to build overlay RgbImage")?;
    ov_img.save(&overlay_path)?;
    Ok(())
}

fn main() -> Result<()> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();
    let args = parse_args()?;
    std::fs::create_dir_all(&args.output)?;
    if let Some(dir) = &args.save_masks {
        std::fs::create_dir_all(dir)?;
    }

    let frames = collect_frames(&args.input)?;
    let models = ModelFiles::for_variant(args.variant, &args.model_dir);
    let runtime_files = RuntimeFiles::ort_1244_dml_from_dir(&args.runtime_dir);

    let load_t0 = std::time::Instant::now();
    let runtime = OrtRuntime::init(&runtime_files)?;
    let (mut segmenter, self_test) = Sam2VideoSegmenter::load(&runtime, &models, args.backend, PostprocessConfig::default())?;
    let load_ms = load_t0.elapsed().as_secs_f32() * 1000.0;

    println!("Variant: {} ({})", args.variant.display_name(), args.model_dir.display());
    println!("Provider: {} (init+load+selftest: {load_ms:.1} ms)", segmenter.provider());
    if let Some(rep) = self_test {
        println!("Self-test [{}]: {}", rep.provider, rep.message);
    }
    println!("Processing {} frames from {} -> {}", frames.len(), args.input.display(), args.output.display());

    let mut tracked_timings = Vec::new();
    for (idx, path) in frames.iter().enumerate() {
        let img = load_rgb(path)?;
        let prompt_used: Option<&FramePrompt> = if idx == 0 {
            Some(&args.initial_prompt)
        } else {
            args.reprompts.get(&idx)
        };
        let res = if let Some(p) = prompt_used {
            segmenter.add_prompt(idx as u64, &img, p)?
        } else {
            segmenter.propagate(idx as u64, &img)?
        };
        let stem = path.file_stem().and_then(|s| s.to_str()).unwrap_or("frame");
        if !args.no_overlay {
            write_outputs(&args.output, stem, &img, &res, prompt_used)?;
        }
        if let Some(mask_dir) = &args.save_masks {
            let mask_img = image::GrayImage::from_raw(res.mask.width, res.mask.height, res.mask.data.clone())
                .context("failed to build GrayImage")?;
            mask_img.save(mask_dir.join(format!("frame_{idx:05}.png")))?;
        }

        let t = res.timings;
        println!(
            "  frame {idx:>2} ({stem}): fg={:5.2}% iou={:.3} stab={:.3} obj={:+.2} {:?} | enc={:.1} mem_attn={:.1} dec={:.1} mem_enc={:.1} pre+post={:.1} total={:.1} ms",
            res.mask.foreground_fraction() * 100.0,
            res.iou_score,
            res.stability_score,
            res.object_score_logit,
            res.status,
            t.vision_encoder_ms,
            t.memory_attention_ms,
            t.mask_decoder_ms,
            t.memory_encoder_ms,
            t.preprocess_ms + t.postprocess_ms,
            t.total_ms,
        );
        if idx > 0 {
            tracked_timings.push(t);
        }
    }

    if !tracked_timings.is_empty() {
        let n = tracked_timings.len() as f32;
        let avg = |f: fn(&scancraft_segment::FrameTimings) -> f32| tracked_timings.iter().map(f).sum::<f32>() / n;
        let peak_rss_mb = query_peak_working_set_mb();
        println!(
            "Mean tracked frame ({} propagated frames): vision_encoder={:.1} ms, memory_attention={:.1} ms, mask_decoder={:.1} ms, memory_encoder={:.1} ms, pre+post={:.1} ms, total={:.1} ms, peak_rss={:.1} MB",
            tracked_timings.len(),
            avg(|t| t.vision_encoder_ms),
            avg(|t| t.memory_attention_ms),
            avg(|t| t.mask_decoder_ms),
            avg(|t| t.memory_encoder_ms),
            avg(|t| t.preprocess_ms + t.postprocess_ms),
            avg(|t| t.total_ms),
            peak_rss_mb.unwrap_or(0.0),
        );
    }

    Ok(())
}

#[cfg(windows)]
fn query_peak_working_set_mb() -> Option<f64> {
    use windows::Win32::System::ProcessStatus::{GetProcessMemoryInfo, PROCESS_MEMORY_COUNTERS};
    use windows::Win32::System::Threading::GetCurrentProcess;
    let mut pmc = PROCESS_MEMORY_COUNTERS::default();
    pmc.cb = std::mem::size_of::<PROCESS_MEMORY_COUNTERS>() as u32;
    let ok = unsafe { GetProcessMemoryInfo(GetCurrentProcess(), &mut pmc, pmc.cb) };
    if ok.is_ok() {
        Some(pmc.PeakWorkingSetSize as f64 / (1024.0 * 1024.0))
    } else {
        None
    }
}

#[cfg(not(windows))]
fn query_peak_working_set_mb() -> Option<f64> {
    None
}
