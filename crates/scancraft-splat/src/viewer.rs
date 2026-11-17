//! Viewing a trained splat.
//!
//! Two paths (see `docs/architecture/splatting.md`, "Viewing"):
//!
//! * **In ScanCraft's viewport**: [`crate::TrainedSplat::render_rgba`] renders the orbit
//!   camera's view on Brush's device and returns RGBA8, which the UI shows as a texture
//!   (exactly what Brush's own app does in `splat_backbuffer.rs`).
//! * **External**: [`open_in_brush`] launches the Brush desktop app on the exported `.ply`.

use std::path::{Path, PathBuf};
use std::process::{Child, Command};

/// Environment variable naming the Brush desktop executable (`brush_app[.exe]`). Without it,
/// `brush_app` is looked up on `PATH`.
pub const BRUSH_APP_ENV: &str = "SCANCRAFT_BRUSH_APP";

/// The Brush executable that [`open_in_brush`] would launch.
pub fn brush_executable() -> PathBuf {
    std::env::var_os(BRUSH_APP_ENV).map_or_else(|| PathBuf::from(if cfg!(windows) { "brush_app.exe" } else { "brush_app" }), PathBuf::from)
}

/// Open `ply` in the Brush viewer (`brush_app <ply> --with-viewer`). Returns the child process.
pub fn open_in_brush(ply: &Path) -> std::io::Result<Child> {
    Command::new(brush_executable()).arg(ply).arg("--with-viewer").spawn()
}
