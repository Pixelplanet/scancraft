# Marker / Global Marker tracking on the POP 4 (Full-field HD)

Written 2026-11-01 by the marker work package (E). Labels as in `modes.md`: **[C]** confirmed
(log line, DLL string, live measurement), **[I]** inference. Raw recordings are kept outside the
repository (`E:\scancraft-recordings\markers\`, not committed).

## 1. What Revo Scan sends (Full-field HD + Marker)

Source: `%LOCALAPPDATA%\Revopoint\RevoScan6\logs\sdk.log_back`, Global Marker session with
Full-field HD on 2026-07-05 18:42 (fw v5.3.87). All [C] (logged commands):

| Time | Command | SDK function (log) / meaning |
|---|---|---|
| 18:42:27.643 | `close_stream_all` | |
| 18:42:28.072 | `cam_type=mipi&set_display_reso=1&&set_display_width=640&&set_display_height=512&&set_display_type=6` | display type 6 |
| 18:42:28.994 | `cam_type=mipi&set_depth_output_fmt=3` | SDK format `265` = `Z16Y8Y8P (0x109)`, 640x512, fps 5 |
| 18:42:29.148 | `zx_media.cgi?camera_id=21` opened | |
| 18:42:29.648 | `set_trigger_mode=0` | |
| 18:42:33.590 | `0x707 200 600` | depth range |
| 18:42:35.405 | `0xb13 1` | `ledAndLaserCtrl(led 0, ctrl 7)` "set red led type for fill light" |
| 18:42:35.696 | `0xb01 1` | `ledAndLaserCtrl(led 0, ctrl 2)`: depth fill light on |
| 18:42:35.973 | `0x910 7000`, `0x911 3000`, `0x903 0x10`, `0xb07 34` | `laserParamCollection`: frame time, exposure, gain, fill-LED luminance |
| 18:42:36.133 | `0xb03 0` | `ledAndLaserCtrl(led 1, ctrl 3)`: RGB LED off |
| 18:42:36.199 | `0xb05 0` | `setProjectorCtrlInWP` |
| 18:42:36.211 | `0x707 250 500`, `0x705 10`, `0xa04 0`, `0xb04 1` | preset depth window, threshold, no multi-frame fusion, laser gate off |
| 18:42:42 .. 18:44:05 | `0x488 1`, `0x903`, `0x911 4600/5400/5100`, `0xb07 33/15/21` | auto exposure: fill LED is lowered as exposure rises |

Host parameters in `RevoScan6.1.log` [C]: `calcmarker 1`, `markerrecon_epipolarerror 1.0`
(frame assembler config), `quickParams.markerRadius:7`.

## 2. The `z16y8y8p` stream (fmt 3 / display type 6)

Live, 2026-11-01, firmware v5.3.94, static desk scene (two cups, no markers in view), recorded
with `crates/scancraft-driver-revopoint/examples/marker_record.rs`:

- Frame = 640x512x6 = 1 966 080 bytes after QuickLZ [C]. Layout [C]: z16 depth (2 B/px) ‖ left
  IR y8 ‖ right IR y8 ‖ a 2 B/px plane. The first 4 B/px are identical in meaning to type 4
  (depth valid 37 %, same IR images as type 4).
- `3DCamera.dll` names the images it can dump for this format [C, strings @ file offset
  `0x32df480..0x32df500`]: `%d-z16y8y8p.bmp`, `-leftIR.bmp`, `-rightIR.bmp`, `-leftOriIR.bmp`,
  `-rightOriIR.bmp`. So the extra plane is "leftOriIR" + "rightOriIR" (two 640x512 y8 images,
  "original" = presumably un-rectified [I]).
- **In all our sessions the extra plane was never refreshed** [C]: byte-identical in every
  frame of a session, also across register changes and before the depth engine started. Its
  content changed only between sessions and showed unrelated scenes (once the desk with a stripe
  pattern, once dotted laser lines from another agent's laser session), i.e. a stale buffer.
  Tried without effect: Revo's register order (stream first, then `0xb13/0xb01/.../0xb05`), fill
  LED `0xb07` 0/34/70/80, exposure `0x911` 1000/3000/4600/5400, `0xb01 0`. Not tried (stream
  died, section 5): `0x701 1`, `0xb05 1`. What makes the firmware fill it is open.
- The left/right IR planes did **not** change with `0xb07` (0..80), `0xb01 0` or `0x911`
  (1000..5400 µs): mean grey 80.3–81.1 in all segments [C]. In this format they look like a
  normalised texture image computed by the depth engine [I]. Retro-reflective markers are
  therefore detected by local contrast, not by saturation (`scancraft-markers::detect`).
- Packets arrive in pairs ~56 ms apart (device counter deltas 56 / ~200 ms) whose IR planes are
  byte-identical [C]. In one session the depth was identical too, in another it was new. The
  stream (`streams/marker.rs`) drops exact copies; the marker tracker ignores frames whose IR
  repeats (`FrameReport::stale_ir`), so a marker pose is never attached to newer depth.
- ~6.5 packets/s, 0.69–1.1 MB per packet compressed.

Decision: `streams/marker.rs` uses display type 6 like Revo (so a future firmware fill of the
OriIR planes is picked up), decodes depth + IR pair and ignores the stale plane
(`FrameLayout::DepthIrPairOriIr`).

## 3. Rectified stereo geometry (needed to triangulate)

The rectified right image keeps `pR`'s principal point: `cx_R - cx_L = (951.04 - 341.21) / 2 =
304.9 px` at 640x512 (`camparam.yaml` `pL`/`pR`). Measured [C]: block matching left→right on
textured pixels with valid on-device depth gives `x_R - x_L + fx·B/Z` = 304.3 px (median of 25
samples, Python on frame 24 of `t6_led34_exp3000`) and 305.7 px (`estimate_right_cx_offset`, mode
of pooled samples over a recording). So disparity is `(uL - cxL) - (uR - cxR)`, **not** `uL - uR`.
`StereoRig` carries only the left intrinsics; `scancraft-markers` estimates the offset from the
first Full-field HD frames or takes it from `MarkerTrackerConfig::right_cx_offset`. Recommended
shared change: add the right principal point to `StereoRig` (other stereo work hits the same).

## 4. Host pipeline (`crates/scancraft-markers`)

1. `detect`: local-mean candidates (+35 grey levels), connected components, re-segmentation at
   half height, tests: area 6..3000 px, elongation ≤ 3, fill ≥ 0.7, < 4 % bright ring;
   background-subtracted intensity centroid. Rejects speckle, stripes, laser lines, glints
   (unit tests with rendered images).
2. `stereo`: candidates on the same row (±1.5 px), disparity within 120..900 mm, area ratio ≤ 2.5,
   unique; optional depth-map consistency; triangulated size 1.5..14 mm.
3. `constellation`: nearest-neighbour association from a constant-velocity guess, else
   triangle side-length matching + RANSAC, Kabsch refinement.
4. `tracker::MarkerTracker`: `Fixed` hint with ≥ 4 inliers and RMS ≤ 0.6 mm, `Prior` with ≥ 3
   inliers and RMS ≤ 2 mm, else none. The first frame defines the world frame (Fixed at the
   tracker's pose); the map grows with Fixed frames (running mean, merge within 3 mm).
5. `global`: Global Marker phase 1 records observations; `refine_map` alternates per-frame Kabsch
   and per-marker averaging (3D bundle adjustment), prunes markers seen < 2 times, re-anchors to
   the original frame; JSON save/load; phase 2 tracks against the fixed map.
6. `map`: tags `Environment` (default) / `OnObject`, `MarkerSet` snapshots and
   `align_marker_sets(source, target, TagFilter::OnObject, ..)` for pass alignment.

### Using it

```rust
// Marker / Global Marker phase 1: what scancraft_pipeline::pipeline_for builds.
let tracker = scancraft_markers::GlobalMarkerSession::phase1(Default::default());
let (map, observations) = (tracker.map(), tracker.observations()); // handles, before boxing
let mut pipeline = PassPipeline { depth: Box::new(OnDeviceDepth { min_mm, max_mm }), pose: Some(Box::new(tracker)) };
// ... scan phase 1, then refine + save the library entry:
let report = GlobalMarkerSession::finish_phase1(&map, &observations, path)?; // RMS before/after
// Phase 2: track the object against the fixed map (map frame = pass frame).
let tracker = GlobalMarkerSession::phase2(Default::default(), path)?;
let pipeline = PassPipeline { depth: Box::new(OnDeviceDepth { min_mm, max_mm }), pose: Some(Box::new(tracker)) };
// Pass alignment:
map.lock().unwrap().set_tag(id, MarkerTag::OnObject);
let set = map.lock().unwrap().marker_set();
let a = align_marker_sets(&set_b, &set_a, TagFilter::OnObject, &MatchConfig::default());
```

## 5. Device verification status

- Scene during all sessions: two cups and desk clutter, **no marker stickers or blocks visible**
  in either IR image (checked visually and by the detector: the only stereo-consistent bright
  compact feature was a glint at 512 mm, triangulated with 0.069 mm 3D std dev over 26 frames —
  a repeatability figure for the triangulation, not for a marker).
- Offline replay: `cargo run -p scancraft-markers --example marker_replay -- <rec.bin> <camparam.yaml>`.
  Live: `cargo run -p scancraft-markers --example marker_live -- <ip> 30` (driver path, honours
  `SCANCRAFT_RECORD_DIR`).
- From ~13:20 on 2026-11-01 the device's `zx_media.cgi` stopped answering on every camera id
  (connection accepted, no headers within 4 s) while `zx_cmd.cgi` still worked; it was not
  recovered by `close_stream_all`. A power cycle of the scanner is probably needed.

### 5.1 First real marker data (user's orbit, 2026-11-01)

Recording `pop4_1793547942.bin` (not committed): handheld ~90° orbit around a black turntable
with ~25 large white retro dots and an orange figure, Full-field HD + Marker, display type 6, 242
packets = 121 processed frames (every other packet repeats the IR, `stale_ir`). Replay:
`marker_replay <rec> <camparam.yaml>`.

Findings [C, from `examples/marker_diag.rs` on frames 0 and 94]:
- The markers were at Z ≈ 125–270 mm, partly closer than the 250–500 mm depth window, and the
  turntable is black: no on-device depth around them. So the depth-based offset estimate fails;
  the tracker now uses the rig's calibrated `disparity_offset_px` (304.92) and only falls back
  to the estimate.
- At these distances the stereo overlap is small (`uR ≈ uL − 125…370`): most left dots have their
  partner outside the right image. Such a dot can pair with a neighbour on the same row; the
  wrong depth shows up as a wrong physical size (true pairs 6.7–7.3 mm, wrong ones 9–11 mm).
- The old "drop a right dot claimed twice" rule discarded good pairs together with bad ones.

Changes: greedy one-to-one assignment by cost (row error + area ratio) with ambiguity blocking,
min depth 80 mm, size-consistency filter (a dot needs two others within ±20 % diameter), map
growth also from `Prior` frames with RMS ≤ 0.3 mm (otherwise the map stalls once the view moves
on to new markers), merge radius 5 mm, triangle edge tolerance 2 mm, and a guard in
`refine_map` (keep a frame pose unless the Kabsch re-fit is better; `scancraft-slam`'s Kabsch can
return a poor rotation for 3 nearly collinear points, which made the refinement *worse*,
0.064 → 1.117 mm, before the guard).

Result on the recording:

| | before (coordinator's run) | now |
|---|---|---|
| triangulated per processed frame | 3–6 | mean 4.0 (0:5, 1:3, 2:4, 3:23, 4:50, 5:22, 6:7, 7:5, 8:2 frames) |
| Fixed / Prior / none | 41 / 22 / 58 | 79 / 22 / 20 (83 % of frames with a hint) |
| map | 13 | 17 (11 seen ≥ 3 times) |
| match RMS | 0.006–0.03 mm | refinement 0.024 → 0.023 mm |
| final pose vs first (user ended near start) | — | 5.4 mm, 1.95° |
| time per frame (detect + stereo + match) | — | 13.4 ms mean, 16.4 ms max |

The 20 frames without a hint are frames 57–97 (one stretch where fewer than 3 triangulated
markers overlap the map) and a few single frames. The loop residual is an upper bound: the user
only ended *near* the start.

## 6. QuickLZ fix found on the way

`quicklz.rs` used `last_matchstart = d_len - 14`; QuickLZ 1.5.0 uses `last_byte - 6 - 4 =
d_len - 11`. Frames whose encoder placed a match in the last 11..14 bytes failed with "End of
stream in end-literals": 7 of 25 frames in the first type-6 recording [C], 0 after the fix.
Regression test `quicklz::tests::decodes_matches_close_to_the_end` (fails with the old value).

## 7. Open

- What fills the OriIR plane (needs a session with markers in view and Revo's full sequence,
  incl. `0x701 1`), and whether markers appear brighter there with the fill LED.
- Marker contrast in the texture IR images with real stickers / blocks (data from the user,
  see the work-package report).
