//! Dynamic ONNX Runtime loading, DXGI adapter enumeration, and session configuration.

use crate::manifest::RuntimeFiles;
use crate::{Result, SegmentError};
use ort::environment::Environment;
use ort::ep;
use ort::session::Session;
use ort::session::builder::GraphOptimizationLevel;
use std::path::Path;
use std::sync::{Arc, OnceLock};

/// Hardware DXGI GPU adapter discovered on Windows.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct GpuAdapterInfo {
    /// Zero-based DXGI adapter index, passed to `ep::DirectML::with_device_id`.
    pub device_id: i32,
    pub name: String,
    pub vendor_id: u32,
    pub device_pci_id: u32,
    pub dedicated_vram_bytes: u64,
    pub shared_mem_bytes: u64,
}

/// Requested execution backend.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ExecutionPreference {
    /// Try each hardware DirectML adapter after verifying it against CPU; fall back to CPU.
    #[default]
    Auto,
    /// Use DirectML without falling back to CPU (`None` lets ORT pick its default GPU).
    DirectMl { device_id: Option<i32> },
    /// Force CPU execution provider.
    Cpu,
}

/// Execution provider actually selected for the loaded sessions.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ActiveProvider {
    DirectMl { device_id: i32, adapter_name: String },
    Cpu,
}

impl std::fmt::Display for ActiveProvider {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::DirectMl { device_id, adapter_name } => write!(f, "DirectML (device {device_id}: {adapter_name})"),
            Self::Cpu => write!(f, "CPU"),
        }
    }
}

/// Result of comparing a candidate GPU adapter's output against CPU on a 2-frame synthetic clip.
#[derive(Debug, Clone, PartialEq)]
pub struct SelfTestReport {
    pub provider: ActiveProvider,
    pub frame0_iou: f32,
    pub frame1_iou: f32,
    pub max_logit_diff: f32,
    pub passed: bool,
    pub message: String,
}

// Keep preloaded DLL handles alive for the lifetime of the process so onnxruntime.dll resolves
// the verified local DirectML.dll instead of falling back to System32.
static PRELOADED_LIBS: OnceLock<Vec<libloading::Library>> = OnceLock::new();
static ORT_ENV: OnceLock<(std::path::PathBuf, Environment)> = OnceLock::new();

/// Initialised ONNX Runtime environment backed by a verified `onnxruntime.dll`.
#[derive(Debug, Clone)]
pub struct OrtRuntime {
    env: Environment,
    _keep_alive: Arc<()>,
}

impl OrtRuntime {
    /// Verifies `files` (SHA-256), preloads `DirectML.dll` / `onnxruntime_providers_shared.dll`,
    /// and initializes the process-global ONNX Runtime `Environment`.
    ///
    /// Note: ONNX Runtime allows one `Environment` per process; subsequent calls in the same
    /// process reuse the existing environment if it was initialized with the same `onnxruntime.dll`.
    pub fn init(files: &RuntimeFiles) -> Result<Self> {
        files.verify_all()?;

        if let Some((loaded_path, env)) = ORT_ENV.get() {
            if loaded_path != &files.onnxruntime_dll.path {
                return Err(SegmentError::InvalidInput(format!(
                    "ONNX Runtime already initialized from {}; cannot reload from {}",
                    loaded_path.display(),
                    files.onnxruntime_dll.path.display()
                )));
            }
            return Ok(Self { env: env.clone(), _keep_alive: Arc::new(()) });
        }

        let mut libs = Vec::new();
        for pinned in [&files.directml_dll, &files.providers_shared_dll].into_iter().flatten() {
            let lib = unsafe { libloading::Library::new(&pinned.path) }.map_err(|e| SegmentError::LibraryLoad {
                path: pinned.path.clone(),
                reason: e.to_string(),
            })?;
            libs.push(lib);
        }
        let _ = PRELOADED_LIBS.set(libs);

        let ort_path = &files.onnxruntime_dll.path;
        let env = ort::init_from(ort_path)
            .map_err(|e| SegmentError::LibraryLoad { path: ort_path.clone(), reason: e.to_string() })?
            .with_name("scancraft-segment")
            .with_telemetry(false)
            .build()?;

        let stored = ORT_ENV.get_or_init(|| (ort_path.clone(), env.clone()));
        Ok(Self { env: stored.1.clone(), _keep_alive: Arc::new(()) })
    }

    pub fn environment(&self) -> &Environment {
        &self.env
    }

    /// Creates an inference session for `model_path` using `provider`.
    pub fn create_session(&self, model_path: &Path, provider: &ActiveProvider, opt_level: GraphOptimizationLevel) -> Result<Session> {
        let builder = Session::builder(&self.env)?.with_optimization_level(opt_level)?;
        let mut builder = match provider {
            ActiveProvider::DirectMl { device_id, .. } => builder
                // Mandatory per ONNX Runtime DirectML documentation:
                // memory pattern and parallel execution must be disabled.
                .with_memory_pattern(false)?
                .with_parallel_execution(false)?
                .with_execution_providers([ep::DirectML::default().with_device_id(*device_id).build()])?,
            ActiveProvider::Cpu => {
                let threads = std::thread::available_parallelism().map(|n| n.get().min(8)).unwrap_or(4);
                builder
                    .with_intra_threads(threads)?
                    .with_execution_providers([ep::CPU::default().with_arena_allocator(true).build()])?
            }
        };
        let session = builder.commit_from_file(model_path)?;
        Ok(session)
    }
}

/// Enumerates hardware DXGI adapters on Windows (excluding software adapters like WARP).
#[cfg(windows)]
pub fn enumerate_dxgi_adapters() -> Vec<GpuAdapterInfo> {
    use windows::Win32::Graphics::Dxgi::{CreateDXGIFactory1, DXGI_ADAPTER_FLAG_SOFTWARE, IDXGIFactory1};

    let mut out = Vec::new();
    let factory: IDXGIFactory1 = match unsafe { CreateDXGIFactory1() } {
        Ok(f) => f,
        Err(e) => {
            log::warn!("CreateDXGIFactory1 failed: {e}");
            return out;
        }
    };

    let mut idx = 0u32;
    while let Ok(adapter) = unsafe { factory.EnumAdapters1(idx) } {
        if let Ok(desc) = unsafe { adapter.GetDesc1() } {
            let is_software = (desc.Flags & (DXGI_ADAPTER_FLAG_SOFTWARE.0 as u32)) != 0;
            if !is_software {
                let end = desc.Description.iter().position(|&c| c == 0).unwrap_or(desc.Description.len());
                let name = String::from_utf16_lossy(&desc.Description[..end]).trim().to_string();
                out.push(GpuAdapterInfo {
                    device_id: idx as i32,
                    name,
                    vendor_id: desc.VendorId,
                    device_pci_id: desc.DeviceId,
                    dedicated_vram_bytes: desc.DedicatedVideoMemory as u64,
                    shared_mem_bytes: desc.SharedSystemMemory as u64,
                });
            }
        }
        idx += 1;
    }
    out
}

#[cfg(not(windows))]
pub fn enumerate_dxgi_adapters() -> Vec<GpuAdapterInfo> {
    Vec::new()
}
