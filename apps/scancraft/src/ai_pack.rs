//! Optional AI Pack manager for SAM 2.1 video segmentation on ONNX Runtime + DirectML.
//!
//! Nothing is ever downloaded automatically. The pack lives outside the repository under
//! `%LOCALAPPDATA%/ScanCraft/ai/` (or `SCANCRAFT_SAM2_DIR` / `SCANCRAFT_ORT_DIR`), and every
//! model graph and runtime binary is verified against its pinned SHA-256 before loading.

use std::path::{Path, PathBuf};
use std::process::Command;
use std::sync::Mutex;

use scancraft_segment::manifest::{
    DIRECTML_VERSION_PIN, ORT_VERSION_PIN, SAM21_TINY_HF_COMMIT, SAM21_TINY_HF_REPO, sha256_file,
};
use scancraft_segment::{
    ActiveProvider, ExecutionPreference, ModelFiles, ModelVariant, OrtRuntime, PostprocessConfig,
    RuntimeFiles, Sam2VideoSegmenter, SelfTestReport,
};

pub const ESTIMATED_MODELS_MB: f32 = 165.0;
pub const ESTIMATED_RUNTIME_MB: f32 = 215.0;

pub const MODEL_DOWNLOAD_FILES: [&str; 6] = [
    "vision_encoder.onnx",
    "mask_decoder.onnx",
    "memory_attention.onnx",
    "memory_encoder.onnx",
    "constants.json",
    "README.md",
];

/// On-disk installation state of the optional AI Pack.
#[derive(Clone, Debug, PartialEq)]
pub struct AiPackStatus {
    pub models_dir: PathBuf,
    pub runtime_dir: PathBuf,
    pub models_ready: bool,
    pub runtime_ready: bool,
    pub directml_present: bool,
    pub ready: bool,
    pub models_error: Option<String>,
    pub runtime_error: Option<String>,
    pub models_bytes: u64,
    pub runtime_bytes: u64,
    pub provider_summary: String,
    pub live_models_ready: bool,
    pub live_models_bytes: u64,
    pub live_variant_summary: String,
}

impl AiPackStatus {
    pub fn total_mb(&self) -> f32 {
        (self.models_bytes + self.live_models_bytes + self.runtime_bytes) as f32 / 1_000_000.0
    }
}

/// Shared lazy holder for the loaded ONNX Runtime + verified SAM 2.1 segmenters (1024 px post-scan
/// and fast 512 px live capture).
#[derive(Default)]
pub struct SharedAiEngine {
    inner: Mutex<Option<LoadedAiEngine>>,
    live: Mutex<Option<LoadedLiveEngine>>,
}

struct LoadedAiEngine {
    models_dir: PathBuf,
    runtime_dir: PathBuf,
    segmenter: Sam2VideoSegmenter,
    report: Option<SelfTestReport>,
}

struct LoadedLiveEngine {
    variant: ModelVariant,
    models_root: PathBuf,
    runtime_dir: PathBuf,
    segmenter: Sam2VideoSegmenter,
    summary: String,
}

impl SharedAiEngine {
    pub fn provider_summary(&self) -> Option<String> {
        let guard = self.inner.lock().ok()?;
        let loaded = guard.as_ref()?;
        Some(format_provider(loaded.segmenter.provider(), loaded.report.as_ref()))
    }

    pub fn live_summary(&self) -> Option<String> {
        let guard = self.live.lock().ok()?;
        let loaded = guard.as_ref()?;
        Some(loaded.summary.clone())
    }

    pub fn unload(&self) {
        if let Ok(mut guard) = self.inner.lock() {
            *guard = None;
        }
        if let Ok(mut guard) = self.live.lock() {
            *guard = None;
        }
    }

    /// Runs `op` with the loaded, SHA-256 verified [`Sam2VideoSegmenter`] (1024 px post-scan
    /// quality, loading once on first use via DirectML hardware self-test with CPU fallback).
    pub fn with_segmenter<T>(&self, op: impl FnOnce(&mut Sam2VideoSegmenter, &str) -> Result<T, String>) -> Result<T, String> {
        let mut guard = self.inner.lock().map_err(|_| "AI engine lock poisoned".to_string())?;
        let models_dir = effective_models_dir();
        let runtime_dir = effective_runtime_dir();

        let needs_reload = match guard.as_ref() {
            Some(loaded) => loaded.models_dir != models_dir || loaded.runtime_dir != runtime_dir,
            None => true,
        };
        if needs_reload {
            let loaded = load_engine(&models_dir, &runtime_dir)?;
            *guard = Some(loaded);
        }
        let loaded = guard.as_mut().ok_or_else(|| "AI engine unavailable".to_string())?;
        let summary = format_provider(loaded.segmenter.provider(), loaded.report.as_ref());
        op(&mut loaded.segmenter, &summary)
    }

    /// Runs `op` with the fastest passing 512 px variant for live capture segmentation (falling
    /// back to the 1024 px post-scan model only if no fast 512 px variant is installed).
    pub fn with_live_segmenter<T>(&self, op: impl FnOnce(&mut Sam2VideoSegmenter, &str) -> Result<T, String>) -> Result<T, String> {
        let mut guard = self.live.lock().map_err(|_| "Live AI engine lock poisoned".to_string())?;
        let models_dir = effective_models_dir();
        let models_root = live_models_root(&models_dir);
        let runtime_dir = effective_runtime_dir();

        let needs_reload = match guard.as_ref() {
            Some(loaded) => loaded.models_root != models_root || loaded.runtime_dir != runtime_dir,
            None => true,
        };
        if needs_reload {
            let loaded = load_live_engine(&models_dir, &models_root, &runtime_dir)?;
            *guard = Some(loaded);
        }
        let loaded = guard.as_mut().ok_or_else(|| "Live AI engine unavailable".to_string())?;
        let summary = loaded.summary.clone();
        let _ = loaded.variant;
        op(&mut loaded.segmenter, &summary)
    }
}

fn live_models_root(models_dir: &Path) -> PathBuf {
    if models_dir.file_name().is_some_and(|n| n == "sam21-tiny-video-onnx") {
        if let Some(parent) = models_dir.parent() {
            return parent.to_path_buf();
        }
    }
    default_ai_root().join("models")
}

fn discover_live_candidates(models_dir: &Path, models_root: &Path) -> Vec<(ModelVariant, ModelFiles)> {
    let mut out = Vec::new();
    if let Ok(override_dir) = std::env::var("SCANCRAFT_SAM2_LIVE_DIR") {
        let p = PathBuf::from(override_dir.trim());
        if p.is_dir() {
            for &variant in ModelVariant::all_live_fast() {
                let files = ModelFiles::for_variant(variant, &p);
                if files.verify_all().is_ok() {
                    out.push((variant, files));
                    break;
                }
            }
        }
    }
    for &variant in ModelVariant::all_live_fast() {
        let dir = models_root.join(variant.dir_name());
        if dir.is_dir() {
            let files = ModelFiles::for_variant(variant, &dir);
            if files.verify_all().is_ok() && !out.iter().any(|(v, _)| *v == variant) {
                out.push((variant, files));
            }
        }
    }
    if out.is_empty() {
        let fallback = ModelFiles::sam21_tiny_from_dir(models_dir);
        if fallback.verify_all().is_ok() {
            out.push((ModelVariant::Sam21Tiny1024Fp32, fallback));
        }
    }
    out
}

fn load_live_engine(models_dir: &Path, models_root: &Path, runtime_dir: &Path) -> Result<LoadedLiveEngine, String> {
    let candidates = discover_live_candidates(models_dir, models_root);
    if candidates.is_empty() {
        return Err("No verified SAM 2.1 model variants installed".to_string());
    }
    let runtime_files = RuntimeFiles::ort_1244_dml_from_dir(runtime_dir);
    runtime_files.verify_all().map_err(|e| format!("ONNX Runtime check failed: {e}"))?;
    let ort = OrtRuntime::init(&runtime_files).map_err(|e| format!("Could not initialize ONNX Runtime: {e}"))?;

    let (segmenter, sel) = Sam2VideoSegmenter::load_fastest_variant(
        &ort,
        &candidates,
        ExecutionPreference::Auto,
        PostprocessConfig::default(),
    )
    .map_err(|e| format!("Could not load live SAM 2.1 variant: {e}"))?;

    let summary = format!("{} · {} ({:.0} ms/frame)", sel.chosen.display_name(), sel.provider, sel.mean_tracked_ms);
    Ok(LoadedLiveEngine {
        variant: sel.chosen,
        models_root: models_root.to_path_buf(),
        runtime_dir: runtime_dir.to_path_buf(),
        segmenter,
        summary,
    })
}

fn format_provider(provider: &ActiveProvider, report: Option<&SelfTestReport>) -> String {
    match report {
        Some(r) if matches!(provider, ActiveProvider::DirectMl { .. }) => {
            format!("{provider} · self-test IoU {:.3}", r.frame0_iou.min(r.frame1_iou))
        }
        Some(r) => format!("{provider} ({})", r.message),
        None => provider.to_string(),
    }
}

fn load_engine(models_dir: &Path, runtime_dir: &Path) -> Result<LoadedAiEngine, String> {
    let models = ModelFiles::sam21_tiny_from_dir(models_dir);
    models.verify_all().map_err(|e| format!("AI models check failed: {e}"))?;
    let runtime_files = RuntimeFiles::ort_1244_dml_from_dir(runtime_dir);
    runtime_files.verify_all().map_err(|e| format!("ONNX Runtime check failed: {e}"))?;
    let ort = OrtRuntime::init(&runtime_files).map_err(|e| format!("Could not initialize ONNX Runtime: {e}"))?;
    let (segmenter, report) = Sam2VideoSegmenter::load(&ort, &models, ExecutionPreference::Auto, PostprocessConfig::default())
        .map_err(|e| format!("Could not load SAM 2.1 graphs: {e}"))?;
    Ok(LoadedAiEngine {
        models_dir: models_dir.to_path_buf(),
        runtime_dir: runtime_dir.to_path_buf(),
        segmenter,
        report,
    })
}

/// Base `%LOCALAPPDATA%/ScanCraft/ai` folder (or `$XDG_DATA_HOME/ScanCraft/ai` on Unix).
pub fn default_ai_root() -> PathBuf {
    let base = std::env::var_os("LOCALAPPDATA")
        .map(PathBuf::from)
        .or_else(|| std::env::var_os("XDG_DATA_HOME").map(PathBuf::from))
        .or_else(|| std::env::var_os("HOME").map(|h| PathBuf::from(h).join(".local").join("share")))
        .unwrap_or_else(std::env::temp_dir);
    base.join("ScanCraft").join("ai")
}

pub fn default_models_dir() -> PathBuf {
    default_ai_root().join("models").join("sam21-tiny-video-onnx")
}

pub fn default_runtime_dir() -> PathBuf {
    default_ai_root().join("runtime").join(format!("ort-{ORT_VERSION_PIN}-dml"))
}

pub fn effective_models_dir() -> PathBuf {
    std::env::var("SCANCRAFT_SAM2_DIR")
        .ok()
        .map(|s| s.trim().to_string())
        .filter(|s| !s.is_empty())
        .map(PathBuf::from)
        .unwrap_or_else(default_models_dir)
}

pub fn effective_runtime_dir() -> PathBuf {
    std::env::var("SCANCRAFT_ORT_DIR")
        .ok()
        .map(|s| s.trim().to_string())
        .filter(|s| !s.is_empty())
        .map(PathBuf::from)
        .unwrap_or_else(default_runtime_dir)
}

fn dir_bytes(paths: &[&Path]) -> u64 {
    paths.iter().filter_map(|p| std::fs::metadata(p).ok()).map(|m| m.len()).sum()
}

/// Inspects on-disk model and runtime folders and verifies pinned SHA-256 digests.
pub fn inspect_pack_in(models_dir: &Path, runtime_dir: &Path, engine: Option<&SharedAiEngine>) -> AiPackStatus {
    let models = ModelFiles::sam21_tiny_from_dir(models_dir);
    let runtime = RuntimeFiles::ort_1244_dml_from_dir(runtime_dir);
    let models_res = models.verify_all();
    let runtime_res = runtime.verify_all();
    let directml_present = runtime.directml_dll.as_ref().is_some_and(|f| f.verify().is_ok());

    let models_bytes = dir_bytes(&[
        &models.vision_encoder.path,
        &models.mask_decoder.path,
        &models.memory_attention.path,
        &models.memory_encoder.path,
        &models.constants_json.path,
    ]);
    let runtime_bytes = dir_bytes(&[
        &runtime.onnxruntime_dll.path,
        runtime.providers_shared_dll.as_ref().map(|f| f.path.as_path()).unwrap_or_else(|| Path::new("")),
        runtime.directml_dll.as_ref().map(|f| f.path.as_path()).unwrap_or_else(|| Path::new("")),
    ]);

    let models_ready = models_res.is_ok();
    let runtime_ready = runtime_res.is_ok();
    let ready = models_ready && runtime_ready;
    let provider_summary = engine
        .and_then(SharedAiEngine::provider_summary)
        .unwrap_or_else(|| {
            if ready && directml_present {
                "Verified · ready (DirectML self-test runs on first use)".to_string()
            } else if ready {
                "Verified · CPU ready".to_string()
            } else {
                "Not installed".to_string()
            }
        });

    let live_root = live_models_root(models_dir);
    let mut installed_fast = Vec::new();
    let mut live_models_bytes = 0u64;
    for &variant in ModelVariant::all_live_fast() {
        let dir = live_root.join(variant.dir_name());
        let lf = ModelFiles::for_variant(variant, &dir);
        if lf.verify_all().is_ok() {
            installed_fast.push(variant);
            live_models_bytes += dir_bytes(&[
                &lf.vision_encoder.path,
                &lf.mask_decoder.path,
                &lf.memory_attention.path,
                &lf.memory_encoder.path,
                &lf.constants_json.path,
            ]);
        }
    }
    let live_models_ready = !installed_fast.is_empty();
    let live_variant_summary = engine
        .and_then(SharedAiEngine::live_summary)
        .unwrap_or_else(|| {
            if live_models_ready {
                let names = installed_fast
                    .iter()
                    .map(|v| v.display_name())
                    .collect::<Vec<_>>()
                    .join(", ");
                format!("Verified ({names}) · fastest picked on first use")
            } else if models_ready {
                "1024 px fallback (copy 512 px variant folder for ~10x faster live tracking)".to_string()
            } else {
                "Not installed".to_string()
            }
        });

    AiPackStatus {
        models_dir: models_dir.to_path_buf(),
        runtime_dir: runtime_dir.to_path_buf(),
        models_ready,
        runtime_ready,
        directml_present,
        ready,
        models_error: models_res.err().map(|e| e.to_string()),
        runtime_error: runtime_res.err().map(|e| e.to_string()),
        models_bytes,
        runtime_bytes,
        provider_summary,
        live_models_ready,
        live_models_bytes,
        live_variant_summary,
    }
}

pub fn inspect_pack(engine: Option<&SharedAiEngine>) -> AiPackStatus {
    inspect_pack_in(&effective_models_dir(), &effective_runtime_dir(), engine)
}

/// Copies and SHA-256-verifies any matching model graphs or ORT/DirectML DLLs found in `source`
/// into the active AI Pack folders. Accepts either a parent folder containing both or one of them.
pub fn install_from_folder_into(source: &Path, models_dir: &Path, runtime_dir: &Path) -> Result<AiPackStatus, String> {
    if !source.is_dir() {
        return Err(format!("{} is not a folder", source.display()));
    }
    let expected_models = ModelFiles::sam21_tiny_from_dir(Path::new(""));
    let expected_runtime = RuntimeFiles::ort_1244_dml_from_dir(Path::new(""));

    let mut roots = vec![
        source.to_path_buf(),
        source.join("models").join("sam21-tiny-video-onnx"),
        source.join("sam21-tiny-video-onnx"),
        source.join("runtime").join(format!("ort-{ORT_VERSION_PIN}-dml")),
        source.join(format!("ort-{ORT_VERSION_PIN}-dml")),
        source.join("models"),
        source.join("runtime"),
    ];
    roots.retain(|p| p.is_dir());

    let mut copied = 0usize;
    let model_targets = [
        (&expected_models.vision_encoder, "vision_encoder.onnx"),
        (&expected_models.mask_decoder, "mask_decoder.onnx"),
        (&expected_models.memory_attention, "memory_attention.onnx"),
        (&expected_models.memory_encoder, "memory_encoder.onnx"),
        (&expected_models.constants_json, "constants.json"),
    ];
    for (pinned, name) in model_targets {
        if let Some(src) = roots.iter().map(|r| r.join(name)).find(|p| p.is_file()) {
            verify_candidate(&src, &pinned.sha256)?;
            std::fs::create_dir_all(models_dir).map_err(|e| e.to_string())?;
            std::fs::copy(&src, models_dir.join(name)).map_err(|e| format!("copy {name}: {e}"))?;
            copied += 1;
        }
    }
    if let Some(readme) = roots.iter().map(|r| r.join("README.md")).find(|p| p.is_file()) {
        if std::fs::create_dir_all(models_dir).is_ok() {
            let _ = std::fs::copy(readme, models_dir.join("README.md"));
        }
    }

    let mut runtime_targets: Vec<(&scancraft_segment::PinnedFile, &str)> = vec![(&expected_runtime.onnxruntime_dll, "onnxruntime.dll")];
    if let Some(f) = &expected_runtime.providers_shared_dll {
        runtime_targets.push((f, "onnxruntime_providers_shared.dll"));
    }
    if let Some(f) = &expected_runtime.directml_dll {
        runtime_targets.push((f, "DirectML.dll"));
    }
    for (pinned, name) in runtime_targets {
        if let Some(src) = roots.iter().map(|r| r.join(name)).find(|p| p.is_file()) {
            verify_candidate(&src, &pinned.sha256)?;
            std::fs::create_dir_all(runtime_dir).map_err(|e| e.to_string())?;
            std::fs::copy(&src, runtime_dir.join(name)).map_err(|e| format!("copy {name}: {e}"))?;
            copied += 1;
        }
    }
    for notice in ["LICENSE-ORT.txt", "ThirdPartyNotices-ORT.txt", "LICENSE-DirectML.txt", "ThirdPartyNotices-DirectML.txt", "LICENSE.txt", "ThirdPartyNotices.txt"] {
        if let Some(src) = roots.iter().map(|r| r.join(notice)).find(|p| p.is_file()) {
            if std::fs::create_dir_all(runtime_dir).is_ok() {
                let _ = std::fs::copy(src, runtime_dir.join(notice));
            }
        }
    }

    // Also install any verified fast 512 px live variants found under `source`.
    let live_root = live_models_root(models_dir);
    for &variant in ModelVariant::all_live_fast() {
        let cand_dirs = [
            source.join(variant.dir_name()),
            source.join("models").join(variant.dir_name()),
            source.join("exports").join(variant.dir_name()),
            source.join("release-staging").join(variant.dir_name()),
            source.to_path_buf(),
        ];
        for cand_dir in cand_dirs {
            if !cand_dir.is_dir() {
                continue;
            }
            let expected = ModelFiles::for_variant(variant, &cand_dir);
            if expected.verify_all().is_ok() {
                let dst_dir = live_root.join(variant.dir_name());
                std::fs::create_dir_all(&dst_dir).map_err(|e| e.to_string())?;
                for name in ["vision_encoder.onnx", "mask_decoder.onnx", "memory_attention.onnx", "memory_encoder.onnx", "constants.json"] {
                    std::fs::copy(cand_dir.join(name), dst_dir.join(name))
                        .map_err(|e| format!("copy {name}: {e}"))?;
                    copied += 1;
                }
                for opt in ["manifest.json", "LICENSE", "NOTICE"] {
                    let src_opt = cand_dir.join(opt);
                    if src_opt.is_file() {
                        let _ = std::fs::copy(src_opt, dst_dir.join(opt));
                    }
                }
                break;
            }
        }
    }

    if copied == 0 {
        return Err(format!("No SAM 2.1 model files or ONNX Runtime DLLs found in {}", source.display()));
    }
    Ok(inspect_pack_in(models_dir, runtime_dir, None))
}

pub fn install_from_folder(source: &Path) -> Result<AiPackStatus, String> {
    install_from_folder_into(source, &effective_models_dir(), &effective_runtime_dir())
}

fn verify_candidate(path: &Path, expected_sha256: &str) -> Result<(), String> {
    let actual = sha256_file(path).map_err(|e| e.to_string())?;
    if !actual.eq_ignore_ascii_case(expected_sha256) {
        return Err(format!(
            "SHA-256 mismatch for {}: expected {expected_sha256}, got {actual}",
            path.display()
        ));
    }
    Ok(())
}

/// Downloads the pinned SAM 2.1 models and ONNX Runtime + DirectML NuGet packages after user
/// consent, extracts the DLLs and notices, and verifies every SHA-256 checksum.
pub fn download_pack() -> Result<AiPackStatus, String> {
    let models_dir = effective_models_dir();
    let runtime_dir = effective_runtime_dir();
    std::fs::create_dir_all(&models_dir).map_err(|e| e.to_string())?;
    std::fs::create_dir_all(&runtime_dir).map_err(|e| e.to_string())?;

    for file in MODEL_DOWNLOAD_FILES {
        let dest = models_dir.join(file);
        let url = format!("https://huggingface.co/{SAM21_TINY_HF_REPO}/resolve/{SAM21_TINY_HF_COMMIT}/{file}");
        run_curl(&url, &dest)?;
    }

    let tmp_root = std::env::temp_dir().join(format!("scancraft-ai-dl-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&tmp_root);
    let ort_dir = tmp_root.join("ort");
    let dml_dir = tmp_root.join("dml");
    std::fs::create_dir_all(&ort_dir).map_err(|e| e.to_string())?;
    std::fs::create_dir_all(&dml_dir).map_err(|e| e.to_string())?;

    let dl_res = (|| -> Result<(), String> {
        let ort_pkg = tmp_root.join("ort.nupkg");
        let dml_pkg = tmp_root.join("dml.nupkg");
        run_curl(
            &format!("https://www.nuget.org/api/v2/package/Microsoft.ML.OnnxRuntime.DirectML/{ORT_VERSION_PIN}"),
            &ort_pkg,
        )?;
        run_curl(
            &format!("https://www.nuget.org/api/v2/package/Microsoft.AI.DirectML/{DIRECTML_VERSION_PIN}"),
            &dml_pkg,
        )?;
        run_tar_extract(&ort_pkg, &ort_dir)?;
        run_tar_extract(&dml_pkg, &dml_dir)?;

        copy_required(&ort_dir.join("runtimes/win-x64/native/onnxruntime.dll"), &runtime_dir.join("onnxruntime.dll"))?;
        copy_required(
            &ort_dir.join("runtimes/win-x64/native/onnxruntime_providers_shared.dll"),
            &runtime_dir.join("onnxruntime_providers_shared.dll"),
        )?;
        let _ = copy_optional(&ort_dir.join("LICENSE.txt"), &runtime_dir.join("LICENSE-ORT.txt"));
        let _ = copy_optional(&ort_dir.join("ThirdPartyNotices.txt"), &runtime_dir.join("ThirdPartyNotices-ORT.txt"));

        copy_required(&dml_dir.join("bin/x64-win/DirectML.dll"), &runtime_dir.join("DirectML.dll"))?;
        let _ = copy_optional(&dml_dir.join("LICENSE.txt"), &runtime_dir.join("LICENSE-DirectML.txt"));
        let _ = copy_optional(&dml_dir.join("ThirdPartyNotices.txt"), &runtime_dir.join("ThirdPartyNotices-DirectML.txt"));
        Ok(())
    })();
    let _ = std::fs::remove_dir_all(&tmp_root);
    dl_res?;

    ModelFiles::sam21_tiny_from_dir(&models_dir)
        .verify_all()
        .map_err(|e| format!("Downloaded AI model checksum verification failed: {e}"))?;
    RuntimeFiles::ort_1244_dml_from_dir(&runtime_dir)
        .verify_all()
        .map_err(|e| format!("Downloaded ONNX Runtime checksum verification failed: {e}"))?;

    Ok(inspect_pack_in(&models_dir, &runtime_dir, None))
}

fn copy_required(src: &Path, dst: &Path) -> Result<(), String> {
    std::fs::copy(src, dst).map_err(|e| format!("copy {} -> {}: {e}", src.display(), dst.display()))?;
    Ok(())
}

fn copy_optional(src: &Path, dst: &Path) -> Result<(), String> {
    if src.is_file() {
        std::fs::copy(src, dst).map_err(|e| e.to_string())?;
    }
    Ok(())
}

fn run_curl(url: &str, dest: &Path) -> Result<(), String> {
    let tmp = dest.with_extension("part");
    let status = Command::new("curl.exe")
        .args(["-fsSL", "--retry", "2", "-o"])
        .arg(&tmp)
        .arg(url)
        .status()
        .map_err(|e| format!("Failed to launch curl.exe for {url}: {e}"))?;
    if !status.success() {
        let _ = std::fs::remove_file(&tmp);
        return Err(format!("Download failed ({}) for {url}", status));
    }
    let _ = std::fs::remove_file(dest);
    std::fs::rename(&tmp, dest).map_err(|e| format!("Save {}: {e}", dest.display()))?;
    Ok(())
}

fn run_tar_extract(archive: &Path, dest_dir: &Path) -> Result<(), String> {
    let status = Command::new("tar.exe")
        .arg("-xf")
        .arg(archive)
        .arg("-C")
        .arg(dest_dir)
        .status()
        .map_err(|e| format!("Failed to launch tar.exe: {e}"))?;
    if !status.success() {
        return Err(format!("Failed to extract {} ({status})", archive.display()));
    }
    Ok(())
}

/// Removes the downloaded AI models and runtime binaries from disk.
pub fn remove_pack(engine: &SharedAiEngine) -> Result<AiPackStatus, String> {
    engine.unload();
    let models_dir = effective_models_dir();
    let runtime_dir = effective_runtime_dir();
    if models_dir.exists() {
        std::fs::remove_dir_all(&models_dir).map_err(|e| format!("Remove {}: {e}", models_dir.display()))?;
    }
    if runtime_dir.exists() {
        std::fs::remove_dir_all(&runtime_dir).map_err(|e| format!("Remove {}: {e}", runtime_dir.display()))?;
    }
    Ok(inspect_pack(Some(engine)))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn missing_directories_report_not_ready() {
        let root = std::env::temp_dir().join(format!("scancraft-ai-missing-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&root);
        let status = inspect_pack_in(&root.join("models"), &root.join("runtime"), None);
        assert!(!status.ready);
        assert!(!status.models_ready);
        assert!(!status.runtime_ready);
        assert!(status.models_error.is_some());
        assert!(status.runtime_error.is_some());
    }

    #[test]
    fn install_from_folder_rejects_empty_and_bad_checksum() {
        let root = std::env::temp_dir().join(format!("scancraft-ai-bad-{}", std::process::id()));
        let src = root.join("src");
        let dst_m = root.join("dst_m");
        let dst_r = root.join("dst_r");
        let _ = std::fs::remove_dir_all(&root);
        std::fs::create_dir_all(&src).unwrap();

        let err = install_from_folder_into(&src, &dst_m, &dst_r).unwrap_err();
        assert!(err.contains("No SAM 2.1"), "{err}");

        std::fs::write(src.join("constants.json"), b"{}").unwrap();
        let err = install_from_folder_into(&src, &dst_m, &dst_r).unwrap_err();
        assert!(err.contains("SHA-256 mismatch"), "{err}");

        let _ = std::fs::remove_dir_all(&root);
    }

    #[test]
    fn verify_candidate_checks_exact_digest() {
        let p = std::env::temp_dir().join(format!("scancraft-ai-hash-{}.bin", std::process::id()));
        std::fs::write(&p, b"scancraft").unwrap();
        let expected = scancraft_segment::sha256_file(&p).unwrap();
        assert!(verify_candidate(&p, &expected).is_ok());
        assert!(verify_candidate(&p, &"0".repeat(64)).is_err());
        let _ = std::fs::remove_file(&p);
    }

    #[test]
    fn live_models_root_uses_parent_for_standard_subdir() {
        let root = live_models_root(Path::new("/tmp/ai/models/sam21-tiny-video-onnx"));
        assert_eq!(root, Path::new("/tmp/ai/models"));
        let status = inspect_pack_in(
            Path::new("/tmp/ai/nonexistent/sam21-tiny-video-onnx"),
            Path::new("/tmp/ai/nonexistent/runtime"),
            None,
        );
        assert!(!status.live_models_ready);
        assert_eq!(status.live_models_bytes, 0);
    }
}
