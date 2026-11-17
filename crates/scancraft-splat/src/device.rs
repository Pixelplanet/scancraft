//! GPU device for Brush.
//!
//! Brush runs on Burn's wgpu backend (CubeCL). By default it gets its own wgpu device on the best
//! adapter, separate from the one egui renders with; training on a worker thread then never
//! competes with the UI's queue submissions. [`SplatDevice::from_wgpu`] shares an existing
//! device instead, which requires that device to have been created with the features and
//! limits CubeCL asks for (all adapter features except `MAPPABLE_PRIMARY_BUFFERS`, the adapter's
//! full limits) — see `docs/architecture/splatting.md`.

use burn_wgpu::graphics::{AutoGraphicsApi, GraphicsApi};
use burn_wgpu::{MemoryConfiguration, RuntimeOptions, WgpuDevice, WgpuSetup};
use std::sync::OnceLock;

/// Burn device handle used by Brush (type-erased dispatch device on wgpu).
pub type BurnDevice = burn::tensor::Device;

/// A device Brush can train and render on. Cheap to clone.
#[derive(Clone, Debug)]
pub struct SplatDevice {
    pub(crate) device: BurnDevice,
}

fn runtime_options() -> RuntimeOptions {
    // Brush's settings (brush-process `burn_options`).
    RuntimeOptions { tasks_max: 64, memory_config: MemoryConfiguration::ExclusivePages }
}

impl SplatDevice {
    /// Brush's own wgpu device on the default adapter. Initialised once per process.
    pub fn default_device() -> Self {
        static INIT: OnceLock<()> = OnceLock::new();
        INIT.get_or_init(|| {
            burn_wgpu::init_setup::<AutoGraphicsApi>(&WgpuDevice::default(), runtime_options());
        });
        Self { device: WgpuDevice::default().into() }
    }

    /// Share an existing wgpu device (e.g. egui's). Call at most once per device.
    pub fn from_wgpu(instance: wgpu::Instance, adapter: wgpu::Adapter, device: wgpu::Device, queue: wgpu::Queue) -> Self {
        let setup = WgpuSetup { instance, adapter, device, queue, backend: AutoGraphicsApi::backend() };
        Self { device: burn_wgpu::init_device(setup, runtime_options()).into() }
    }

    /// The Burn device, for callers that use Brush APIs directly.
    pub fn burn(&self) -> &BurnDevice {
        &self.device
    }
}
