//! ScanCraft's 3D viewport: an orbit camera and a wgpu renderer for point clouds, meshes and the
//! ground grid, drawn inside egui through an `egui_wgpu` paint callback.
pub mod camera;
pub mod scene;

pub use camera::{Camera, CameraPreset};
pub use scene::{GpuMesh, GpuPoint, GpuVertex, GridSpec, MeshLayer, MeshShading, PointLayer, PointShading, SceneFrame, SceneRenderer, gpu_points, install};
