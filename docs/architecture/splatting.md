# Gaussian splatting (`crates/scancraft-splat`)

A Gaussian splat is a **project output** (see `scan-sessions.md` §4): it is trained from the RGB
keyframes + poses of the aligned colour passes and initialised from the merged point cloud,
which may include geometry-only passes. Points that no colour keyframe sees are excluded.

## 1. Approach

```
SplatDataset { images: Vec<PosedImage>, init_cloud: PointCloud }      (ScanCraft world, mm, Z-up)
        │
        ├─ coverage::analyse     per point: #views that see it unoccluded (+ inside mask),
        │                        mean observed colour → drop uncovered points, seed colours
        ├─ init splats           metres; isotropic scale = ½ mean distance to 2 NN; opacity 0.5;
        │                        SH DC from the observed colour, higher bands 0
        ├─ views                 Brush cameras (convert.rs), images capped to max_resolution,
        │                        packed once; masks → alpha (transparent background)
        ├─ Brush SplatTrainer    step / refine loop (same schedule as brush-process), seeded,
        │                        progress callback + cancellation every step
        ├─ eval                  PSNR/SSIM on held-out views (eval_every) and a train sample
        └─ TrainedSplat          .ply (3DGS, mm) · render_rgba(look-at) · render_view(image)
```

Entry points: `scancraft_splat::train()` (blocking) and `SplatJob::spawn()` (own thread,
`poll()` for progress, `cancel()`, `join()`). Nothing runs on the UI thread. Fallback for
external tools: `export::export_dataset()` writes the same inputs as nerfstudio + COLMAP.

## 2. Brush integration decision

**Brush is used as a library, in-process, pinned to a git revision.**

Facts at the time of writing (Brush `main` @ `1388f74`, 2026-10-03; latest release v0.3.0 is
a year older and on an older wgpu):

| | Brush | ScanCraft |
| --- | --- | --- |
| Licence | Apache-2.0 | MIT OR Apache-2.0 |
| wgpu | 30.0.1 (patched fork, see below) | 30.0.1 |
| egui / eframe | 0.36 | 0.36 |
| glam | 0.30 | 0.29 |
| Edition | 2024 | 2021 |
| Compute stack | Burn `main` (git) + CubeCL + CubeK (git revs) | — |
| crates.io | not published | — |

Crate layout used: `brush-train` (`SplatTrainer`: step/refine, eval), `brush-render`
(`Splats`, `Camera`, forward/backward rasteriser), `brush-dataset` (`SceneBatch`, image
packing; its loaders are only used in tests), `brush-serde` (3DGS PLY export). Not used:
`brush-process` (VFS-based pipeline, rerun logging, its own checkpoint writer), `brush-app`
(UI), `brush-cli`.

Why in-process:

- Same wgpu major version as ScanCraft, so Brush's types and ours are the same `wgpu` crate —
  no second wgpu in the binary, and device sharing is possible (§6).
- Brush's training core is cleanly separable: `SplatTrainer::step(SceneBatch, Splats)` takes an
  in-memory packed image and a camera. We feed it our images directly; no files, no VFS.
- Progress, cancellation and our own initialisation (coverage filter, scale init, colours
  from images) are trivial in-process and awkward through a CLI.

Alternatives considered: **separate process** (`brush` CLI on an exported dataset — still
available via `export` + "open in Brush"; costs a file round trip and loses fine-grained
progress/cancel; useful if Brush's dependency on Burn `main` ever becomes unbuildable for us);
**vendoring** (≈20 crates plus Burn/CubeCL git deps — high maintenance, no benefit while git
deps resolve); **pinning a crates.io release** (none exists for the current code).

Notes on the dependency:

- Brush patches wgpu with a fork (`ArthurBrussee/wgpu`, branch `fix/webgpu-errors-v30.0.1`)
  for WebGPU/wasm error scopes and JS interop. `[patch]` sections of a dependency do not apply
  to us; we build against crates.io wgpu 30.0.1 and native training/rendering works
  (validated on Windows, Vulkan, AMD RX 6900 XT). Nothing in ScanCraft is patched.
- Brush tracks Burn's git `main`. `Cargo.lock` pins Burn to the commit Brush's own lockfile
  uses at our revision (`faec398…`; CubeCL `4675224…`, CubeK `da89901…` follow from it).
- The lockfile gains many entries (Burn's optional backends: tch, sqlite, LLVM bundler, …);
  `cargo tree -p scancraft-splat` shows they are not compiled — only the wgpu backend is.
- Debug builds: `Cargo.toml` sets `opt-level = 3` for the Brush/Burn/CubeCL crates in the dev
  profile; their host code records and fuses kernels every step.
- Build cost: a clean build of `scancraft-splat` takes ≈8 min (dev, no debuginfo) and ≈3 GB
  of `target/` with `CARGO_PROFILE_DEV_DEBUG=0`; with full debuginfo the target dir reached
  9 GB (`CARGO_INCREMENTAL=0` saves another ~0.6 GB). Only crates that depend on `scancraft-splat` pay this.

### Bumping Brush

1. Pick a Brush commit; read its `Cargo.lock` for the `burn` git commit.
2. Update the `rev` of the `brush-*` entries in the workspace `Cargo.toml`.
3. `cargo update -p burn --precise <burn-commit-from-brush-lock>`.
4. `cargo test -p scancraft-splat` and `cargo test -p scancraft-splat --test train_gpu -- --ignored`.

## 3. Data contract (what a colour keyframe must provide)

```rust
pub struct PosedImage {
    pub name: String,              // unique; export file name
    pub width: u32, pub height: u32,
    pub rgb: Vec<u8>,              // RGB8, row-major, top row first, len = w*h*3 (decoded JPEG)
    pub mask: Option<Vec<u8>>,     // optional foreground mask, 1 byte/px, 255 = object
    pub intrinsics: Intrinsics,    // fx fy cx cy in pixels of *this* image + Option<Distortion>
    pub camera_to_world: Affine3A, // ScanCraft world units (mm), rigid
    pub axes: CameraAxes,          // camera-local convention of that pose (default ScanCraft)
}
pub struct Distortion { k1, k2, p1, p2, k3 }   // OpenCV 5-coefficient model
pub struct SplatDataset { images: Vec<PosedImage>, init_cloud: PointCloud /* mm, merged */ }
```

Adapting the colour agent's RGB keyframe at merge time:

- `rgb`: decode the stored JPEG (`image::load_from_memory(..).to_rgb8()`). Pick sharp
  keyframes upstream; motion blur hurts splats.
- `intrinsics`: from `Prgb.bin`, scaled to the decoded resolution if the stream is not
  full-res (`fx, cx` × `w_img / w_calib`, `fy, cy` × `h_img / h_calib`).
- `distortion`: from `Distort.bin` if it is the OpenCV `k1 k2 p1 p2 k3` model; any other
  order must be remapped. `None` if the frames are undistorted already.
- `camera_to_world`: `depth_pose(t_rgb) * rgb_extrinsics` where `depth_pose` is the
  tracker's pose (interpolated to the RGB timestamp) and `rgb_extrinsics` (`LC_RT.bin`) maps
  RGB-camera coordinates into the depth camera's frame. Both in the ScanCraft scanner frame →
  `axes: CameraAxes::ScanCraft`. If the extrinsics are applied in OpenCV optical axes instead
  (as calibration files usually are), either convert them with
  `types::opencv_to_scancraft_camera()` or produce the whole pose in OpenCV axes and set
  `axes: CameraAxes::OpenCv`.
- `mask` (optional but recommended for object scans): project the fused model / depth into
  the RGB frame and mark the object; then the splat learns a transparent background.
- `init_cloud`: the merged cloud of all aligned passes in world mm. Its colours are only
  used with `InitColors::FromCloud`; the default takes colours from the keyframes.

Quick self-check for an adapter: `PosedImage::project(world_point)` must land on the pixel
where that point appears in `rgb`. `convert.rs` and `tests/export_roundtrip.rs` pin the rest.

## 4. Coordinate conventions

| Frame | Axes | Units |
| --- | --- | --- |
| ScanCraft world | +X right, +Y forward, +Z up | mm |
| ScanCraft camera (`CameraAxes::ScanCraft`, = `DepthMap::to_point_cloud`) | +X right, +Y optical axis, +Z image-up | mm |
| OpenCV / COLMAP camera (`CameraAxes::OpenCv`), Brush camera | +X right, +Y down, +Z optical axis | — |
| nerfstudio / OpenGL camera (export only) | +X right, +Y up, +Z backwards | — |
| Brush world (training) | ScanCraft world axes | m (× 0.001) |
| Exported `splat.ply` | ScanCraft world axes | mm (overlays mesh/cloud) |
| Exported datasets | ScanCraft world axes | m |

- `p_scancraft_cam = M · p_opencv_cam` with `M = [x | −z | y]` (columns), i.e.
  `(x, y, z)_cv → (x, z, −y)_sc`. `c2w_opencv = c2w_scancraft · M`.
- Brush `Camera { position, rotation }` is camera-to-world in OpenCV axes;
  `fov_x = focal_to_fov(fx, w, model)`, `center_uv = (cx/w, cy/h)` — resolution independent, so
  images can be downscaled without touching the camera.
- nerfstudio `transform_matrix` = `c2w_opencv · diag(1, −1, −1, 1)` (metres).
  COLMAP `images.txt` = inverse of `c2w_opencv` (metres), quaternion `qw qx qy qz`.
- The PLY header carries `comment Vertical axis: 0 0 1`; Brush's viewer uses it to put Z up.
  Other viewers (SuperSplat, …) may assume Y-up and show the object lying on its side.
- Brush's rotation layout in the PLY is the 3DGS one (`rot_0` = w).

## 5. Test data and measured numbers

No ScanCraft colour data exists yet, so `synthetic.rs` renders an object scan with known
cameras: a 70 mm sphere with a 36×24×22 mm box on top, procedural view-independent texture
(hue/brightness bands, thin meridians, 6 mm checkerboard), CPU rasteriser with 2×2
supersampling, black background + mask. 3 camera rings (15°/40°/65° elevation, 230 mm) × 16
views; every 8th view held out. The initial cloud is 40 k grey surface samples (0.1 mm
noise) — the underside (never seen) must be filtered.

Hardware: AMD Radeon RX 6900 XT (Vulkan), Ryzen 7 5800X, Windows 11.

PSNR/SSIM are Brush's eval (render on black vs. ground truth premultiplied by its mask, i.e.
whole frame incl. background). "Object only" is our independent PSNR over mask pixels only,
which is the harder number. Defaults otherwise (`SplatTrainConfig::default()`, SH degree 3).
All runs in the dev profile with the opt-level overrides from §2.

| Run | Views (train/held-out) | Iters | Train time | ms/iter | Splats init → final | Held-out PSNR / SSIM | Train PSNR | Held-out, object only |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| example, 640×480 | 42 / 6 | 3000 | 51.8 s | 17 | 34 680 → 81 318 | **30.40 dB** / 0.978 | 32.16 dB | 23.0–25.2 dB |
| example, 640×480 | 42 / 6 | 7000 | 316 s¹ | 45¹ | 34 680 → 77 349 | **30.78 dB** / 0.980 | 33.27 dB | 23.1–25.9 dB |
| `train_gpu` test, 320×240, SH 1 | 30 / 6 | 1000 | 26.9 s² | 27² | 17 234 → 43 574 | 27.39 dB / 0.963 | 28.68 dB | — |
| example, 640×480, deps at opt-level 1 | 42 / 6 | 3000 | 284.5 s | 95 | 34 680 → 81 191 | 30.38 dB / 0.978 | 32.16 dB | 22.9–25.2 dB |

¹ ran concurrently with a full `cargo clippy` build competing for the CPU; the clean rate is
≈17 ms/iter. ² before the opt-level overrides.

- Coverage filter: 34 680 of 40 000 initial points survive (the hidden underside and the box's
  bottom face, which rests on the sphere, are removed); the unit test checks that no point of
  the underside survives and > 95 % of the visible top does.
- Preparation (coverage, KNN scales, packing 48 images) ≈ 0.2–0.6 s; scene rendering 0.2–0.5 s.
- `splat.ply`: ≈ 18–19 MB for ≈ 80 k splats (SH degree 3, 62 floats per splat).
- Held-out vs. training gap ≈ 2–2.5 dB: views are 22.5° apart per ring, so held-out views are
  genuine interpolations. The residual error is concentrated on the thin dark meridians and
  checker edges (side-by-side images written by the example as `heldout_*.png`).

Reproduce: `cargo run -p scancraft-splat --example train_splat -- --iters 3000 --export --out <dir>`
(`--res WxH`, `--sh N`). GPU test: `cargo test -p scancraft-splat --test train_gpu -- --ignored`.

## 6. Viewing

- **In ScanCraft (prototype API):** `TrainedSplat::render_rgba(eye, target, up, fov_y, w, h,
  bg)` renders the orbit camera's view with Brush's rasteriser on Brush's device and returns
  RGBA8 (≈ 12–14 ms for 640×480 with 80 k splats, including the GPU→CPU readback; measured
  in the example). This is the same structure Brush's own app uses
  (`brush-app/src/ui/splat_backbuffer.rs`: render on the training device, read back RGBA8,
  upload into an egui texture). The UI can show it as an `egui::Image` over the viewport or
  blit it in `scancraft-render`; no change to `scancraft-render` was made, to keep the UI
  untouched while another agent works on it.
  Integration plan: (1) render on a worker when the camera changes (≤ 1 frame in flight, as
  `AsyncMap` does in Brush), (2) show as a viewport layer, (3) optionally composite with the
  mesh using Brush's depth output later.
- **Zero-copy option:** `SplatDevice::from_wgpu(instance, adapter, device, queue)` lets Brush
  run on egui's wgpu device (Burn `init_device`). The device must then be created with what
  CubeCL requests: all adapter features except `MAPPABLE_PRIMARY_BUFFERS`, the adapter's full
  limits and `ExperimentalFeatures::enabled()` (CubeCL's `request_device`). That means
  changing eframe's `WgpuConfiguration` in the app; not done here. Training on the UI device
  would also make UI frames wait behind training submissions, so even then a separate
  device for training is preferable and sharing is only worth it for display.
- **External:** `viewer::open_in_brush(ply)` launches `brush_app <ply> --with-viewer`
  (`SCANCRAFT_BRUSH_APP` env var or `brush_app` on `PATH`).

## 7. Not done / known limits

- No real ScanCraft colour data yet; the contract above is untested against actual keyframes
  (timestamps, extrinsics, `Distort.bin` layout are the colour agent's to confirm).
- Images are kept in RAM (packed RGBA, 4 B/px); 300 keyframes at 1280×800 ≈ 1.2 GB.
- Coverage is decided with a point z-buffer, no mesh; very sparse clouds can over-report
  visibility at silhouettes.
- No pose refinement: pose error shows as blur. Brush has none either at this revision.
- GPU tests are `#[ignore]`d (`--ignored` to run) so `cargo test --workspace` works without
  a GPU.
