# POP 4 blue laser modes (Cross Lines, Single Line)

Status (2026-11-10): **Cross Lines and Single Line work on a static scene** with Marker and
Global Marker tracking, Cross Lines also with Color tracking (lines + dots stream). Sections 1
to 5 are from 2026-11-01 (firmware v5.3.94) and partly superseded; **the dated sections at the
end are the current state** ([documentation map](../README.md)).

Status 2026-11-01. Device: POP 4 S/N `…00E1B`, WiFi `192.168.178.34`. **[C]** = confirmed
(live measurement, log line, file content), **[I]** = inference. Raw recordings are kept outside
the repo (`%TEMP%\laser-agent\rec`, not committed).

## 1. Stream

| Item | Value | Evidence |
|---|---|---|
| Format | `set_display_type=2`, 1280×1024, `set_depth_output_fmt=1` | [C] Revo `sdk.log_back` 2026-07-05 18:38:08.991 / 18:38:09.908 |
| Media URL | `zx_media.cgi?camera_id=33&type_id=20` | [C] same log 18:38:10.082 |
| Framing | same `44 33 22 11` + u32 length packets as `camera_id=21` | [C] live capture `cross2` |
| Payload | **one baseline JPEG** (SOF0 1280×2048, YCbCr 4:2:0), left IR on top, right IR below; the 100-byte in-band header (`abcd`, counter @20, IMU) follows the JPEG EOI marker | [C] SOF0 bytes `ffc0 0011 08 0800 0500 03`, trailer starts `61626364`; Revo logs "mjpeg compress type … start pair hard decode", "Image size 1280 x 2048"; SDK format 266 "CPAIR" |
| Rate | ~80 frames/s from the device (counter steps 12–13 ms); 67.6 packets/s received over WiFi in a 7.5 s capture; 85–180 KB per frame | [C] `cross2`, `single1`; Revo: "DEPTH recv frame rate is 68.50" |
| Rectification | **not rectified** (raw lenses); host maps points with `mapparam{L,R}.bin` | [C] rectified feature rows only agree after applying the maps (§2) |
| Black level | 16 (JPEG luma) | [C] image percentiles |
| Cross lines | consecutive frames alternate the two line families (~30 parallel lines each, "/" and "\") | [C] frames 100/101 of `probe3` |
| Single line | one near-vertical line | [C] `single1` |

`camera_id=21` with the same format delivers the pair as QuickLZ `y8y8` (lossless, 2.6 MB/frame,
~12 fps over WiFi) [C, `probe1`]; ScanCraft uses Revo's JPEG stream for the frame rate.

Decoder: `media.rs` `FrameLayout::JpegIrPair` (luma-only decode with `zune-jpeg`, 3–9 ms/frame).

## 2. Blue calibration set

Downloaded by Revo at connect (`sdk.log_back` 2026-07-05 18:37:43) with
`zx_cmd.cgi?download=<path>`; `LC_RT.bin`, `Distort.bin`, `Prgb.bin` do not exist in the blue set
(`{download:-3}`). ScanCraft downloads (`calibration::download_blue`):

| File | Content |
|---|---|
| `/data/camparam_blue/camparam.yaml` | OpenCV stereo calibration: raw K/dist per camera, R/T, rectified `pL`, `pR`, `Q`. Rectified f = 1482.669, cxL = 340.440, cxR = 950.008, cy = 485.670, baseline 118.63 mm [C] |
| `/data/camparam_blue/mapparamL.bin`, `mapparamR.bin` | 148 B: `i32 h, w, n_dist`, `f32 R[9]` (identity), `f32 cx, cy, fx, fy`, `f32 dist[12]`, `f32 iR[9]` = `(P_rect·R_rect)⁻¹` (OpenCV `initUndistortRectifyMap`) [C: values equal the yaml; `K_rect⁻¹·iR⁻¹` is orthonormal to 1e-7, ~10° rotation per camera] |
| `/data/camparam_blue/metroExtra.bin` | 42 624 B laser-line model, see §4 |
| `/data/brightnessGainMap.json` | IR fill LED level per gain for `lineGeneral/Dark/Reflect/Outdoor` [C: Revo sent exactly these `0xb07` values 60/30/20/11/9] |

The blue and red rectified left frames differ by 0.009° (from both `mapparamL.bin`) [C], so laser
points share the sensor frame of Full-field HD depth.

Core types: `scancraft_core::{RawCamera, StereoRectification}`, attached as
`StereoRig::rectification`; the laser rig is `DeviceCalibration::blue_rig()` (`calibration_set: "blue"`).
Note for other modes: the red rectified right image also has its own cx (951.0 vs 341.2), which
the rig's single `intrinsics` did not express; `StereoRectification::right_cx` does.

## 3. Device setup and registers

Revo's cross-line sequence (log 2026-07-05 18:38:13–16), reproduced by `streams/laser.rs`:
`set_trigger_mode=0`, `0x707 200 600`, `0x3001 7`, `0xb03 0`, `0xb13 0`, `0xb01 1`, `0x910 11000`,
`0x911 300`, `0x903 gain·0x10`, `0xb07 <fill LED>`, `0x48d 255`, `0xb08 950`, `0xb04 1`.

| Register | Meaning | Evidence |
|---|---|---|
| `0x3001 7` / `0x3001 9` | cross lines / single line | [C] live: 9 gives one line (`single1`) |
| `0xb04` | **emitter gate: 1 = ON, 0 = OFF** | [C] live twice (`probe2`, `probe3`): lasers appear after `0xb08`/`0x48d` with `0xb04 1`, vanish within one frame of `0xb04 0`, return with `0xb04 1`. Older notes (`lasers_and_markers.md`) had it inverted. Revo's "led 2 ctrl 3" → `0xb04 0`, "ctrl 2" → `0xb04 1` |
| `0x48d v` | laser luminance; Revo sends `specialLaserLum` (cross 255, single 155) | [I] value match with `deviceScanParam.json` |
| `0xb08 µs` | laser exposure; Revo starts at 950 and nudges it (923…994) during the scan | [C] log; ScanCraft adjusts it from saturation statistics |
| `0xb07 v` | IR fill LED (marker illumination), from `brightnessGainMap.json` | [C] |
| `0x903 g·0x10` | gain index 1..5 per object (cross 1/3/1/1, single 1/2/1/1 for general/dark/reflect/outdoor) | [I] value pattern in log |
| `0x701 1`, `0x483` | sent by Revo when leaving the laser mode (18:42:25) | [C] log; [I] re-enables the on-device depth engine |

**Safety finding [C]:** the previous `Control::turn_off_emitters` sent `0xb04 1` and therefore left
the lasers armed; after it the laser lines stayed visible in the stream (`probe3` frames 444–680).
It now sends `0xb04 0` first and `0x701 1` last. `full_field.rs` still sends `0xb04 1` (as Revo
does in Full-field); after a laser session with `0x3001` still 7/9 that would light the lasers —
Full-field should reset the projector mode first (open item for that module's owner).

After a laser session the Full-field depth plane stayed all zero (three tries with different
projector setups, IR planes black) [C]; Revo sends `0x701 1` + `0x483` before switching, now part
of the shutdown. Not yet verified because the media server stopped answering (§6).

Object presets (`laser_preset`): gain and luminance from `deviceScanParam.json`
(`marker/crosswire_line`, `marker/single_line`); initial `0xb08` 950 for General (logged), scaled
by the table's `exposure` ratio for the others (dark ×2, outdoor ×0.2) [I].

## 4. Host geometry (`crates/scancraft-laser`)

1. **Line centres**: Steger (σ = 1.5 px) at candidate pixels only, ~2–5 ms for both images.
2. **Chains**: samples linked within 3 px with consistent tangents (bridges rows Steger rejects on
   saturated centres), then mapped into the rectified frame with the lens models.
3. **Candidates**: every left/right chain pair of the same family (slope sign) whose rows overlap
   and whose median depth lies in 150–450 mm, triangulated per left sample.
4. **Line identification**: with ~30 lines ~50 px apart and a disparity window of ~400 px, ~8
   pairings per left line are geometrically valid. Only the right one lies on a laser light plane.
   Planes are rigid w.r.t. the scanner, so `LaserPlaneSet` is calibrated online from the first 40
   frames (or loaded from `SCANCRAFT_LASER_PLANES`): single line = RANSAC for the one plane over all
   pairings; cross lines = greedy RANSAC over hypotheses from curved stretches, a plane needing ≥ 4
   distinct supporting stretches, planes through the camera centre rejected.
5. **Output**: stereo-triangulated points of identified pairings (within 1.2 mm of their plane),
   normals by PCA over neighbouring lines within 12 mm (fallback: towards the sensor, orthogonal to
   the line), oriented to the sensor; scanner frame (+X right, +Y forward, +Z up).

### Results on the static scene (two cups on a table, ~260–300 mm)

| | Single line | Cross lines |
|---|---|---|
| Light plane(s) | 1 plane: normal (−1.000, 0.021, 0.001), offset −59.07 mm, i.e. a vertical sheet at x ≈ 59 mm, half-way along the 118.6 mm baseline — physically plausible projector position | 14–20 planes found, but they do not form a consistent pencil (fitted projector centre residuals up to 13 mm); top view of the result shows arcs at several depth layers → many wrong line assignments |
| Points/frame | 180 (mean over 242 laser frames) | 531 (but not trustworthy) |
| Repeatability frame-to-frame | NN distance median 0.010 mm, 90 % 0.21 mm | — |
| Host time | 5.0 ms depth source + 3–7 ms JPEG decode | 11 ms + 9 ms |
| Device rate | ~80 fps (60 packets/s with laser-off frames in the capture) | ~80 fps |

Not measured (needs the device, §6): flat-surface plane-fit RMS of laser vs Full-field HD, and
mean laser↔Full-field distance. `examples/laser_offline.rs` computes both when given a Full-field
recording of the same scene (`ff.bin` argument); the Full-field captures taken after the laser
session had an all-zero depth plane.

### metroExtra.bin (not decoded)

Header: 18 B serial + 14 B timestamp `20260619142306`, then `i32` (656137) and little-endian
`f64` from offset 36. Structure found [C]: 30 blocks of 88 doubles; each block = 2 groups × 5
row bands (`v` 0–204.8–…–1024) × `(v0, v1, c0..c5)` plus 8 doubles (4 image points sharing the `v`
coordinates 111.4, 718.9, 968.4, −289.2 across blocks k and k+15). Blocks 15–29 repeat 0–14 (group
B ≈ group A + 1 px). `RPACalibLaserLog.txt` says `paramType: X_YZ`, `baseline horiztal: true`.
Interpreting a band as `u = c0v² + c1vw + c2w² + c3v + c4w + c5` with w = disparity, total
disparity, depth or right column (raw or rectified) gives chance-level stereo agreement (15–25 %),
and the 15 linear plane models do not share a common axis — so the meaning is still open. The
second half of the file (from double 2641) is not doubles. Decoding it would replace the online
plane calibration and fix cross-line identification.

## 5. What is wired up

- Driver: `streams::for_light` → `LaserLines` for `CrossLines`/`SingleLine`; blue set downloaded
  at connect (laser modes refuse to start without it); auto laser exposure.
- Pipeline: `pipeline_for` returns the `LaserLines` depth source for both; pose comes from the
  marker estimator (not in this branch), so `modes.rs` keeps both NotImplemented.

## 6. Open / needed

- From ~15:15 the scanner's media server stopped answering any `zx_media.cgi` request (control
  commands still work; also seen 13:05–14:50). Needs a power cycle.
- Full-field HD recording of the same static scene **after** `0x701 1` to measure laser accuracy
  and cross-line identification against ground truth (and to calibrate cross planes from it).
- A moving recording with markers for end-to-end laser passes.
- Decode `metroExtra.bin` (§4) or calibrate cross planes from ground truth.

## Laser gate and Full-field HD (integrator test, 2026-11-01)

- `0xb04` gates the emitter of the selected projector mode. Full-field HD needs it open: with
  `0xb04 0` at the end of the Full-field setup the IR images show no pattern and the z16 plane
  stays empty; with `0xb04 1` depth returns (92.8k points, 28 % coverage on the test scene).
- Safety check: `0x3001 7` (cross lines) was selected by hand, then the normal Full-field setup
  ran (`0x701 1` first, `0xb04 1` last): no laser lines in the IR images, normal depth. So the
  Full-field setup does not fire a leftover laser mode. `Control::turn_off_emitters` closes the
  gate (`0xb04 0`) and sends `0x701 1` when a session ends.


## 2026-11-10: firmware v5.3.99

- Revo Scan now starts Cross Lines as **format 7 / display type 4 at 1280x1024 on
  `camera_id=33&type_id=22`** (SDK format 272; the device lists it as `depth+sp+ls-resolution
  1280x1024x4`). One JPEG of 1280x4096 per frame = four 1280x1024 planes; the DSP initialises
  projector type 14 `ZX_MULTI_CROSS_LINE_ADD_SPECKLE` and algorithm 12 `PAIR_N_REMAP` by itself.
  Revo's messages after the first frame (sdk.log 2026-11-07 15:20:03..06): `0x707 200 600`,
  `0x3001 14`, `0x488 0`, `0x903 0x10`, `0xb11 6000`, `0xb03 1`, `0xb01 0`, `0x910 10000`,
  `0x911 300`, `0xb1b 170`, `0x48d 255 1`, `0xb08 951`, `0xb03 1`, `0xb02 0`, `0xb12 255`,
  `0xb1c 50000`, `0xb04 1`. Revo used it with Color tracking (`scanMode 3`).
- Probed with `examples/stream_probe.rs` (same messages, arriving at the DSP): planes 1 and 3
  show the lines of one direction only and very faint (bright only on retro-reflective markers);
  planes 0 and 2 (expected: dot pattern) stay dark. With the settings applied the stream slowed
  to ~2 fps and register writes took >1 s each. The DSP logs `rv_projector_trigger projector is
  busy projector status = 2`, `Unkonw projector flag(0x43), set frame time do nothing!` and, at
  projector init, `rv_projector_send_cmd uart wait ack timeout`.
- The old stream (fmt 1 / type 2, `0x3001 7` / `9`, this driver's `streams/laser.rs`) delivers
  frames, but with the messages actually applied the lasers stay dark; before the register fix
  (defaults) the lines were faint. Geometry: 0 points in both cases.
- Open: what makes the lasers fire at working brightness on this firmware (compare the DSP log
  of a Revo Scan Cross Lines session, which needs Revo running while `tools/scanner/dsplog_*`
  capture), then decode the four-plane stream.

### Same day, later: lasers work with Revo's Marker-mode sequence

Read from the DSP log and Revo's sdk.log while Revo Scan ran Cross Lines + Marker
(2026-11-10 09:23:42..50): **with Marker tracking Revo still uses the classic stream** (fmt 1 /
type 2 at 1280x1024, `camera_id=33&type_id=20`, SDK format 266, projector type 7
`ZX_MULTI_CROSS_LINE`, algorithm `PAIR`). The format-7 lines + dots stream is what it uses with
Color tracking. Messages after the first frame: `0x707 200 600`, `0x3001 7`, `0xb03 0`, `0x488 0`,
`0x903 0x10`, `0xb07 38`, `0xb03 0`, `0xb13 0`, `0xb01 1`, `0xb04 1`, `0xb01 1`, `0x910 11000`,
`0x911 500`, `0x48d 255`, `0xb08 951`, `0xb07 38`, `0xb04 1` (then `0xb04 1` again at scan start).

`streams/laser.rs` now sends this (it had `0x911 300`, no `0x488 0`, another order). Result with
`examples/mode_check.rs` on a static scene (mug on the marker turntable):

| Mode | Frames with geometry | Line points / frame | Stream | Static tracking |
| --- | --- | --- | --- | --- |
| Cross Lines + Marker | 100 / 120 (the first ~20 precede the setup) | ~1,200 | ~24 fps | 52 tracked, 48 lost |
| Single Line + Marker | 100 / 120 | ~250 | faster than 30 fps | 100 tracked, 0 lost |
| Cross Lines + Color | 60 / 80 | ~1,040 | 6.5 fps with RGB | 25 tracked, 35 lost; colour on 10 of 80 |

Both cross directions are visible and the markers are lit (`E:\Claude\eval\cross3-sheet.png`).
The earlier "lines too faint / dark" results came from the wrong exposure and, before that, from
register writes that never arrived. Open now is host-side only: reliable identification of the
crossing lines (half the static frames are rejected), marker-based pose on line frames with a
moving scanner, and the format-7 stream for Color tracking.

### Tracking fixed: markers in raw laser frames (2026-11-10)

Why Cross Lines lost half the static frames: `MarkerTracker::detect` refused unrectified pairs
(the laser stream is raw), so no pose hint ever came and the tracker fell back to aligning line
points to the model. The two line families alternate in blocks of a few frames, and lines of one
family do not lie on the lines of the other, so that alignment failed whenever the family changed.

Now the dots are detected in the raw 1280x1024 images (detector size limits scaled by width /
640), their centres are rectified with the blue calibration's raw camera models
(`RawCamera::rectify`), and stereo runs on those; the right principal-point offset comes from
`StereoRectification::right_cx`. Static scene, `mode_check`: Cross + Marker 60/60 tracked (7-12
markers carry the pose per frame), Cross + Global Marker 60/60, Single + Marker 60/60.
Cross + Marker with the Dark preset on a bright scene: overexposed, 2.2 s per frame, 47 of 60
lost (open). Cross + Color is not offered in the app until the format-7 stream is decoded.

### Dark preset and plane calibration (2026-11-10)

Two separate causes made Cross Lines + Dark unusable (2-5 s per frame, most frames lost):

1. **Markers too dim.** Revo's `brightnessGainMap` lowers the IR fill LED as gain and exposure
   rise (`lineDark` gain 3: level 6), while the laser lines get several times brighter. Markers
   crossed by a line were rejected and only 2-4 of ~10 were triangulated. With the level at 20 or
   more, 7-9 markers carry the pose. `streams/laser.rs` now uses at least 20 (`MIN_FILL_LED`).
2. **Plane calibration on every scan start.** The online calibration of the laser light planes
   took 4 s with the Normal preset and **167 s** with over-bright Dark frames, where it also
   produced 57 planes for a projector with 30 lines. The planes are device geometry, so they are
   now calibrated once and kept per scanner in
   `%LOCALAPPDATA%\ScanCraft\laser_planes_<cross|single>_<rig key>.json`
   (`scancraft_laser::planes_cache_file`; `SCANCRAFT_LASER_PLANES` still overrides). A calibration
   is rejected if it finds more than 15 planes per family, and not attempted on frames with more
   than 300 line pairings (a warning asks for one scan with the Normal preset first).

After both: Cross + Marker + Dark 56/56 frames tracked at 86 ms per frame; Single + Marker + Dark
46/46. The stored cross set has 17 planes (calibrated on a mug scene); lines that scene did not
show well are missing until a better calibration scene is used. Decoding the factory model in
`metroExtra.bin` (section 4) would make the calibration unnecessary.

### Cross Lines + Color on the lines + dots stream (2026-11-10)

Format 7 / display type 4 at 1280x1024, `camera_id=33&type_id=22`: one JPEG of 1280x4096 per
frame = four **raw** 1280x1024 planes: left camera with the dot pattern, left camera with the
laser lines, right camera dots, right camera lines [C] (contact sheet
`E:\Claude\eval\cross22g\sheet300.png`). ~20 fps, ~470 KB per frame. `get_depth_output_fmt`
answers 2 after selecting format 7, so the driver no longer waits for the value to match, only
for the device to stop answering "busy". The dot planes halved to 640x512 and rectified with the
red calibration give host stereo depth like VCSEL (23 % coverage on the test scene); treated as
already rectified they give 0.4 %, so they are raw.

Implementation: `FrameLayout::DotsAndLines`, `streams/lines_dots.rs` (Revo's sequence of
2026-11-07 15:20), `SensorFrame::dots` (the rectified dot pair), and in the pipeline
`dots::DotsPose`: an inner pipeline (dot stereo + colour features + its own tracker) whose pose
is handed to the outer tracker as fixed, so the model is fused from the line points only. A frame
the dots cannot place is counted as lost instead of aligning the sparse lines by themselves
(`PoseEstimator::pose_required`, `Tracker::skip_frame_as_lost`).

Static scene, `mode_check`: Cross + Color + Normal 86/86 frames tracked, ~800 line points per
frame, colour on 84 of 90 frames, ~12 fps; Cross + Color + Dark 46/46. Open: a moving scanner;
the inner tracker keeps its own dense model (memory on long scans); the small rigid offset
between the blue (lines) and red (dots) rectified frames is constant and not corrected.
