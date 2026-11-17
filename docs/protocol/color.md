# POP 4 colour camera: stream, timing, calibration, colour tracking

Status 2026-11-01. Evidence tags as in `modes.md`: **[C]** confirmed (capture, log line or
measurement), **[I]** inferred. Recordings used here are kept outside the repo
(`%LOCALAPPDATA%\scancraft\recordings\color\`, a static desk scene with two mugs and papers).

## 1. Stream

| Item | Value | Evidence |
|---|---|---|
| URL | `GET /cgi-bin/zx_media.cgi?camera_id=50&type_id=20` | [C] Revo `sdk.log` 2026-10-12 09:24:34.232 `streamMain start stream:…camera_id=50&type_id=20`; ScanCraft capture 2026-11-01 |
| Setup (before opening) | `zx_cmd.cgi?cam_type=usb&set_resolution=1&width=1280&height=1024` → `{"result":0}`; `cam_type=usb&set_aa_mode=1` (auto exposure) → `{"result":0}`; `cam_type=usb&set_aeMaxExp=9` → `{"result":9}` (Revo logs "max exp 9000"); `cam_type=usb&set_whitebalance_mode=1` (auto WB) | [C] sdk.log 09:24:33.740–09:24:36.025; replayed by ScanCraft with the same replies |
| Prerequisite | The RGB stream delivers **nothing** unless the MIPI depth stream is configured and streaming: alone it sent 0 bytes in 12 s (two attempts); started 2 s after `camera_id=21` it streamed at once. Revo also starts depth first (09:24:29) and RGB second (09:24:34) | [C] ScanCraft probes 2026-11-01 12:58 / 12:59 |
| Body | Endless HTTP body; packets `[u32 LE 0x11223344][u32 LE len][len bytes]` — the same framing as `camera_id=21` (`captures/2026-11-01-media-stream-layouts.md`), first packet preceded by `\r\n` | [C] capture: magic at offset 2, `len` 0x0004b3b3, next magic at 2+8+len |
| Payload | **Not QuickLZ.** A baseline JFIF JPEG (`FFD8 FFE0 … JFIF`, SOF0 1280×1024, 3 components, luma 2×2 sampling = 4:2:0, standard Huffman tables), `FFD9`, then **4 bytes: u32 LE timestamp** | [C] all 71 packets of a 6 s capture end in `FFD9 xx xx xx xx`; SOF0 bytes `ffc0 0011 08 0400 0500 03 01 22 …` |
| Size / rate | 250–310 kB per frame; 11.8 fps (frame intervals alternate 56/114 ms, the depth stream's cadence) | [C] capture timestamps; Revo logs "paired rate is : 12.6" |
| Formats offered | MJPG (0), RGB8 (1), RGBA (6), all 1280×1024; Revo requests format 1 = RGB8, i.e. the SDK decodes the JPEG on the host (`compressProc … format: 1`, `DecoderMpeg`) | [C] sdk.log `printStreamInfos` 09:24:33.740 |
| `get_resolution` reply | `{"curr-resolution":"1280x800"}{"h264-resolution":"1280x1024"}` even though frames are 1280×1024 | [C] |

The first packet after opening can be stale (a frame from an earlier session: its timestamp lay
5.0 s before the next one). Pairing by timestamp discards it.

## 2. Timing: one clock for depth and colour

The trailer timestamp is on the **same device millisecond clock** as the depth stream's in-band
frame counter (`abcd` header, u32 at offset 20, `media.rs`):

- Capture 2026-11-01 12:59, depth and RGB streams open concurrently: depth counters
  7 869 390 … 7 879 112 (span 9 722 over a 10 s curl), RGB timestamps 7 872 154 … 7 878 039.
- Every one of the 70 live RGB frames lies **1–2 ms after a depth frame** (offsets observed: 1 or
  2, never more), and their intervals copy the depth stream's 56/114 ms cadence. [C]
- So the RGB camera is triggered together with the IR cameras (hardware sync) [I from the
  constant 1–2 ms offset]. Revo pairs the two streams by timestamp with
  `setDepthRgbMatchParam bMakeSureRgbIsAfterDepth = 0, iDifThreshold = 15, iRgbOffset = 0`
  [C sdk.log 09:24:34.231].

ScanCraft pairs a colour frame with the depth frame whose counter is within 15 ms
(`color_stream::ColorSync::take_match`, waiting ≤ 120 ms for the colour frame to arrive over its
own connection) and stores it in `SensorFrame::color` with `ColorFrame::timestamp_ms` = trailer
value, i.e. directly on the depth clock. The pose of a colour frame is therefore the pose of its
depth frame (≤ 2 ms apart: 0.2 mm at 100 mm/s hand motion);
`scancraft_color::interpolate_pose` exists for colour frames between depth frames.

Running the RGB stream does not slow the depth stream: 11.8 fps before and after it starts. [C]

## 3. Calibration

Downloaded with `zx_cmd.cgi?download=/data/camparam/<file>` (Revo: sdk.log 09:22:20.195).
`/data/camparam_blue/Prgb.bin` does not exist (Revo logs "not find").

| File | Size | Layout (LE) | This POP 4 |
|---|---|---|---|
| `Prgb.bin` | 40 | `u16 w, u16 h, f32[9]` row-major K | 1280×1024, fx = fy = 1564.90, cx 635.86, cy 516.44 |
| `Distort.bin` | 20 | `f32[5]` OpenCV `k1 k2 p1 p2 k3` | 0.1339, −0.7150, 0.00118, 0.0000981, 1.3389 |
| `LC_RT.bin` | 48 | `f32[9]` row-major R, `f32[3]` T (mm) | R ≈ 5.8° about y; T = (−24.10, 0.08, −2.71) |
| `Pl.bin` | 40 | as `Prgb.bin` | equals the rectified `pL` of `camparam.yaml` (fx 1484.56, cx 341.21, cy 485.13) |

**Convention** (`calibration::parse_color_calibration`, `scancraft_core::ColorCamera`):
`p_rgb = R · p_left + T`, with `p_left` in the **rectified** left IR optical frame — the frame of
the on-device depth map — and `p_rgb` in the RGB optical frame (OpenCV axes); the RGB image is
distorted by `Distort.bin`. The RGB camera centre is then 24.25 mm to the right of the left IR
camera, 0.3 mm behind it. Evidence [C, `scancraft-color/examples/color_calib_check.rs`, 10 frame
pairs]: depth silhouettes projected into the colour image land on colour edges with this
convention (best extra shift (+0.4…+2.0, −2.6…−3.1) px in 8 of 10 frames; two frames lock onto a
neighbouring edge), while the inverse transform (≈ 35 px off), R = I (≈ 40 px) or no distortion
(≈ 15 px in x) are clearly worse. `Pl.bin` being the rectified matrix, and a rectification
rotation of ~10° (half the 19.9° stereo verge) that would cause hundreds of pixels of error,
show `LC_RT` refers to the rectified frame.

**Residual misalignment, measured** at 1280×1024 (≈ 0.19 mm per pixel at 300 mm):

| Method | Result |
|---|---|
| Depth silhouettes (occluding edges, jump > 25 mm) vs colour gradient | median shift (+1, −2.9) px |
| Left IR image warped through depth vs colour, gradient NCC at 1/4 resolution | (−10.1, −4.6) px; (−13.8, −4.0) px for content < 330 mm, (−5…−10, −2) px for ≥ 380 mm |

The two disagree in x. The silhouette measure uses geometry only; the IR measure compares
IR shading (projector light, speckle) with room-light shading, which shifts gradient maxima.
Depth and its own IR image agree within 1 depth pixel. So the colour-to-depth registration error
is between ~3 px (0.6 mm) and ~12 px (2.3 mm) at 300 mm; not corrected yet. A refinement of the
extrinsic would need a target with sharp colour edges on geometric edges (e.g. a printed
checkerboard folded over a box edge).

## 4. ScanCraft implementation

**Verification status:** the wire protocol, pairing and calibration were verified on captures
(curl, 2026-11-01 12:59) and offline through the driver's own parser/decoder
(`scancraft-color/examples/common`). The end-to-end driver run
(`scancraft-driver-revopoint/examples/color_live.rs`) could not be completed: from 15:26 on the
scanner's media server delivered no data on any stream (`camera_id=21` included, 8 probes over
10 min; control commands still answered; `get_display_type` → -10000). Possibly a flat battery
(see `captures/2026-11-01-media-stream-layouts.md`). Re-run `color_live` once the device streams.

- Driver: `crates/scancraft-driver-revopoint/src/color_stream.rs` (parser, JPEG decode, pairing,
  setup, recording to `pop4_<t>_rgb.bin` next to the depth recording when
  `SCANCRAFT_RECORD_DIR` is set); `device.rs` starts it after the depth stream when
  `ScanConfig::color_texture` is set or tracking is `Color`. Shut-down is the usual
  `close_stream_all` + emitter shutoff. The RGB LEDs (`0xb12`) are not switched on; Revo's June
  sessions send `0xb03 1`, `0xb02 0`, `0xb12 255` around colour scans [C sdk.log_back 14:48:26],
  their role beyond "white LED" is unknown.
- Colour model: `scancraft_core::ColorCamera` (projection, distortion, `world_from_color`).
- Crate `scancraft-color`:
  - `RgbKeyframe` — the per-pass record for texturing and Gaussian splatting: `jpeg` (bytes as
    received), `width`/`height`, `camera` (intrinsics at that size, distortion, extrinsic),
    `sensor_pose` (world ← scanner frame, +X right, +Y forward, +Z up, mm), `camera_pose`
    (world ← colour optical frame, OpenCV axes, mm — the camera-to-world matrix),
    `timestamp_ms` (device clock), `sharpness` (variance of Laplacian at half resolution),
    `depth_frame_index`. Serde-serialisable. JPEGs are not undistorted.
  - `KeyframeSelector`: a new keyframe once the colour camera moved 12 mm / 5°, the sharpest of
    the next 4 frames, frames below 0.6× the running sharpness skipped.
  - `colorize_points`: per-point weighted average over keyframes, weight = cos²(view angle) ×
    relative sharpness × border falloff (60 px), z-buffer occlusion test (4 px cells, 2.5 mm).
  - `ColorRecorder` (in `PassPipeline::color`) records keyframes of accepted frames and colours
    new points live (`PassPipeline::colorize`, called by the app's scan worker).
  - `ColorTracker` (Color tracking): §5.

## 5. Color tracking

Per frame: FAST-9 corners on the half-resolution grey colour image, Harris-ranked and spread
over 32 px cells (≤ 1200), intensity-centroid orientation, 256-bit steered BRIEF. Each feature
gets a 3D point on its own ray at the depth registered into the colour image. Mutual-best
Hamming matching (≤ 64 bits, ratio 0.85) against the 3 most recent of ≤ 10 tracking keyframes
(features with world points), RANSAC over 3D-3D pairs with exact 3-point triad poses (4 mm
gate), then Gauss-Newton on the reprojection error of all inliers (4 px final gate). Accepted
if ≥ 20 inliers and ≥ 20 % of matches. Only colour-tracked frames become tracking keyframes
(new one after 15 mm / 8° or < 80 inliers); geometry-only frames may restart the keyframe set
after 5 colour failures.

`scancraft_pipeline::color::ColorPoseEstimator` (registered in `pipeline_for` for
`TrackingMode::Color` with Full-field HD) hands strong estimates (≥ 100 inliers, ≤ 1.8 px RMS) to
the tracker as `HintKind::Fixed` and weaker ones as `Prior`. Reason, measured below: on flat
surfaces point-to-plane ICP drifts away from a correct colour prior along the directions it
cannot observe.

### Measurements (`scancraft-color/examples/color_track_eval.rs`)

Real POP 4 data, static scene (70 paired frames): 69/69 frames tracked against the first frame;
translation jitter mean 0.21 mm, max 0.50 mm; rotation max 0.06°; 85–180 inliers, 0.8–0.9 px
RMS; 18–35 ms per estimate (release build, 1280×1024 input). (On the synthetic sequence an estimate took 200–330 ms while seven other builds ran on the machine; it matches 600 features with depth against 3 keyframes.)

Synthetic flat textured object (a plane textured with a real colour frame, real calibration and
depth intrinsics, 0.3 mm depth noise, 55 frames sliding 1.5 mm/frame then 1.2 mm/frame along
the plane with a slow turn about its normal and small wobbles):

| Mode | Final error | Worst error | Frames not accepted |
|---|---|---|---|
| ICP only (Feature tracking) | 53.1 mm | 53.1 mm | 0 (it "tracks", but stays put) |
| Colour prior + ICP | 2.2 mm | 8.2 mm | 0 |
| Colour fixed when strong, else prior (shipped) | 0.96 mm (0.18°) | 0.96 mm | 0 |

Not measured: real handheld motion. A moving colour recording (flat textured object, e.g. a
magazine on a table, slid and turned under the scanner, `SCANCRAFT_RECORD_DIR` set) is needed.

### Vertex colours

From 4 keyframes of the static recording, 80 835 of 90 390 points of a depth frame got a colour
in 30 ms; rendered back into the depth view, mug colours and edges are in place
(`color_track_eval` writes `vertex_colors_depth_view.png`).

## 6. Open points

- Extrinsic residual (§3) — needs a calibration target.
- Real moving data for Color tracking.
- The app re-reads points from the tracker on keyframe discard / rebuild (`fused_points()` is
  grey); final colours should come from `ColorRecorder::colorize_model` / `colorize_points` on
  the merged model. RGB keyframes are not yet dropped when the user discards depth keyframes
  (match on `depth_frame_index`).
- `ScanConfig::rgb_exposure_us` / `rgb_gain` are ignored; auto exposure (limit 9 ms) and auto
  white balance as in Revo. Manual control: `cam_type=usb&get_exposureRange` → `{"min":-13,
  "max":-5,…}`, `get_exposure`, `get_whitebalance_val` (2800–7500 K) exist [C sdk.log], setters
  not yet captured.
