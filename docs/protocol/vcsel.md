# VCSEL Rapid (light source 4, "speckle")

Status (2026-11-10): **works.** Available in the app with Feature and Color tracking. The device
sends the IR pair at 640x1024 as four exposures (an ambient and a VCSEL-lit one per camera) on
`camera_id=33&type_id=20` (JPEG); the two lit ones are rectified and matched on the PC with the
coarse-to-fine matcher (24 ms per frame). Setup and start order: `streams/vcsel.rs`,
[device-internals.md](device-internals.md). Measured: [scan-modes.md](../architecture/scan-modes.md).

This file is a lab notebook in the order things were found. Sections 1 to 5 describe the search,
including explanations that turned out wrong (the "latch", "arming" by a Full-field session, the
stream order): they came from asking for the pair at 640x512, which returns one camera twice,
and, after the firmware update, from register writes that never reached the DSP. **Section 6
and the notes after it are the current state** ([documentation map](../README.md)).

Tags: [C] confirmed by measurement or log, [I] inferred, [H] hypothesis.

## 1. Device setup (driver `streams/vcsel.rs`)

`set_trigger_mode=0`, `0x701 1` (leave laser mode), `0x707 300 800`, `0x912 1`, `0x3010 0x17 0`,
`0x3001 12` (VCSEL), `0x48d <lum>`, `0x488 1`, `0x903 <gain>`, `0x911 <exposure>`, `0xb08 2000`,
`0x910 50000`, `0xb07 0`, `0xb04 1` (gate), `0xb03 0`. Then fmt 1 / display type 2, 640x512,
`zx_media.cgi?camera_id=21` (lossless QuickLZ, two u8 planes). After the format switch the device
reports `get_depth_output_fmt` = 1 and `curr-resolution` `640x512x2` [C, 2026-11-01 16:13].
The stream runs at about 13 fps over WiFi [C].

`SCANCRAFT_VCSEL_EXTRA="<cmd>;<cmd>"` appends `rk_preisp` commands to the setup (development
hook).

## 2. Stereo geometry [C]

The red calibration set is rectified without equalising principal points:
`pR[0][2] - pL[0][2]` = +304.9 px at 640x512. `StereoRig::disparity_offset_px` holds this offset.
Depth is `Z = fx * B / (d + disparity_offset_px)` with `d = u_left - u_right`, where fx = 742.28
and B = 118.58 mm. Check: host SGM on the IR pair of a fmt 3 / type 4 frame matches the same
frame's device z16 depth with a median difference of +0.17 mm and a MAD of 0.52 mm. 77 % of
common pixels agree within 1 mm and 95 % within 3 mm (2,608 pixels, test scene `sp34`). The sign
and size of the offset are therefore right.

300–800 mm maps to disparities -196 .. -5, a 192-wide search. The pipeline clamps the VCSEL
stereo window to `VCSEL_RANGE_MM` = 300..800, because the app's default near limit of 150 mm would
double the search to 416.

## 3. Measured on the POP 4 (2026-11-01, after the power cycle)

Recording `target/vcsel-data/r4` (not committed): a static desk scene. It has 60 VCSEL frames and
60 Full-field HD frames.

- **Plane 1 (the "right" image)** shows a dense, sharp dot pattern [C]. Its geometry matches
  neither rectified camera of the Full-field session:
  - it has no rectification border;
  - the box edge is at x ≈ 220, against 195 in rectified left and 90 in rectified right.

  [H] It is an unrectified (raw) image of the left camera, or a differently mapped one.
- **Plane 0 (the "left" image)** lines up with the Full-field rectified left. It is dark: mean
  about 17 against about 75 for plane 1, and it shows no dots [C].
- Host SGM on this pair: **0.2 % coverage** (733 points). With the uniqueness and texture filters
  off it reaches 0.1–4 %, and the extra pixels are mostly wrong. Device depth for the same scene
  covers 28 %. **The pair cannot be used for stereo.**
- Matcher speed at 192 disparities: median 93 ms per frame, **10.7 fps** (max 115 ms), on a CPU
  with rayon. That meets the 5 fps target, so no GPU path is needed for now.

Register experiments (15-frame VCSEL sessions, each one appended to the normal setup):

| Extra | IR mean L / R | Notes |
| --- | --- | --- |
| none (r4) | 18 / 75 | dots in plane 1 only |
| `0xb08 666` | 18 / 75 | no change |
| `0xb08 10000` | 17 / 17 | VCSEL off |
| `0x911 2000`, `0x910 33333`, `0x48d 255`, and the plain baseline | 16–17 / 16–17 | VCSEL stayed off after the `0xb08 10000` session |
| baseline after one Full-field HD session | 18 / 60 | VCSEL back, plane 1 only |

[C] A long `0xb08` latches the VCSEL off across sessions. A Full-field HD session restores it.
The VCSEL never lit plane 0 in any variant tried.

## 4. Open / next steps

- Find which camera and mapping plane 1 is. Rectify it with the red set's `cameraMatrixL/R`,
  `R1/R2` and `pL/pR` (`scancraft-stereo::RectifyMap`) and compare it with the Full-field
  rectified pair. If plane 1 is a raw image, look for a stream (another `camera_id` or display
  type) that delivers both raw or both rectified images *with* the pattern.
  - Revo uses `camera_id=33&type_id=20` (one 640x1024 JPEG pair). Check whether both halves show
    dots there.
- Check the exposure/strobe timing (`0xb08` against `0x911`/`0x910`): the dark plane may be
  exposed outside the VCSEL pulse.
- Once a lit pair exists, rerun `examples/vcsel_eval.rs` against a Full-field recording of the
  same static scene. It reports coverage, plane RMS and the per-pixel difference, and takes SGM
  settings from `SGM_*` environment variables. Then tune coverage against correctness.

## 5. Revo Scan 6 capture (2026-11-01, `captures/2026-11-01-revo-modes-capture.md`)

- **Revo's own stream has a lit pair** [C]. Its second VCSEL session (`camera_id=33&type_id=20`,
  756 frames) starts with both halves dark (mean ~16, ~30 fps). From frame 34 on, both halves show
  the dot pattern (L ~56, R ~51). Both are raw (unrectified) images, one from each camera.
- Revo opens the media stream right after `set_depth_output_fmt=1` and configures everything
  afterwards. Frame 34 arrives ~1.1 s after the stream opened, about 0.5 s after
  `set_trigger_mode=0` and *before* `0x3001 12` and the rest of the VCSEL setup. Revo's connect
  sequence sends `0x48b 10` (we never did).
- Tried on our side, each after a Full-field HD session (which clears the latch):

  | Variant | IR mean L / R |
  | --- | --- |
  | Revo's register order + media (`streams/vcsel.rs`) | 17.7 / 64.3 |
  | + `set_trigger_mode=0` 0.6 s after the stream opens (`ModeStream::after_open`) | 17.7 / 64.2 |
  | + `0x48b 10` before the setup (`SCANCRAFT_VCSEL_PRE`) | 17.7 / 64.3 |
  | `0x48b 10` after the setup (`SCANCRAFT_VCSEL_EXTRA`) | 16.2 / 16.2, latched off until a Full-field session |
  | `0x701 1` after the setup | 16.4 / 16.3 |

- Next: decode Revo's frames before and after frame 34 together with the exact request timing
  (Revo's first VCSEL session at t≈701 s, where Revo itself got no points, is the comparison case),
  and replay Revo's full sequence: connect writes, format, stream open, *then* configuration.
  Revo's lit frames can also feed `examples/vcsel_eval.rs` directly to test the host stereo
  matcher on a real lit pair.

### 2026-11-02 live follow-up (fresh boot, then Full-field arming)

- After a scanner boot, VCSEL stays dark (16/16, ~30 fps) until a Full-field HD session has run.
  Our stop sequence (`turn_off_emitters`) disarms it again, so **every** VCSEL run needs a
  Full-field session first [C].
- Configuring after the stream opened (`SCANCRAFT_CONFIGURE_AFTER_OPEN=1`, Revo's order): both
  halves dark for 10 s at ~30 fps; the `0x910 50000` frame time never takes effect [C].
- `system_cmd=echo '123' > /tmp/use_hard_encoder` (Revo's connect write): no change [C].
- `examples/vcsel_revo_replay.rs` replays Revo's two sessions literally (setup with the stream
  open, no emitter shutdown, `0xb03 0` + `0x48b 10`, second session): dark throughout at ~30 fps.
  Our order (setup before the format switch, starting with `0x701 1`) gives the one-lit-camera
  pair at ~16 fps. Something Revo does is still missing; settings written while a fmt 1 stream
  runs are not applied in our tests.

### Host stereo on Revo's lit pair (offline, 2026-11-02)

`SRC=jpegraw SKIP=150 MIN_MM=120 MAX_MM=450 examples/vcsel_eval.rs` on Revo's VCSEL stream cut
from the capture (raw JPEG pair, rectified with the red calibration's raw intrinsics/distortion):
13.7 % coverage (44.8k points) per frame, plane RMS 1.2 mm, ~170 ms per frame. Row alignment
peaks at zero vertical shift and a vertical-crop model of the raw image only lowers coverage, so
the rectification is right. Only the background matched: the scene was at ~150–400 mm (the VCSEL
presets work at 300–800 mm) with a saturated white box and a black turntable. Revo's own preview
in that session had ~10k points. **The host matcher works on a lit pair; what is missing is
getting the device to send one.** The eval's default 300–800 mm window gave 3 % on this scene.

### Both cameras lit in the Full-field layout (2026-11-02)

The leftover frame of the second Hybrid run (VCSEL configured via `rk_preisp` right after a
Full-field session, device still in fmt 3 / type 4) shows the dot pattern in **both** rectified
IR planes (`E:\Claude\eval\hybrid-vcsel-sheet.png`). [H] VCSEL Rapid = VCSEL setup + fmt 3 /
type 4 stream; test with `SCANCRAFT_VCSEL_FMT3=1` (stream spec switch in `streams/vcsel.rs`).
The default JPEG path is now rectified in the driver (it was passed on as if rectified).

Tested after a power cycle (Full-field arming, then the VCSEL setup with fmt 3 / type 4 on
`camera_id=21`): both IR planes lit (L 64 / R 67) but **without** the dot pattern, at the
Full-field rate (6.9 fps), with device depth at 20.6 % coverage. That is plain Full-field HD; the
VCSEL setup has no effect in that layout. Rejected; the experiment switch was removed. The dotted
leftover frame came from the transitional state of a format switch and is not reproduced this way.

### Re-reading the 2026-11-01 "latch" (offline, sdk.log)

Revo's VCSEL starts (format 266 = `CPAIR` / 0x10A, "inspire2 start mpjg pair compress
processing"): stream opened, `set_trigger_mode=0`, first frame counted 3.3 s later (session 2;
session 1 4.5 s), then the setup. In Revo's session-2 stream the first ~34 frames (~1.1 s at
~30 fps) are dark on both cameras, then both light up. Our "`0x48b 10` after the setup" run
gave dark frames at 32 fps, but recorded only **40 frames (~1.25 s)**, and the "latched" runs
after it were 40 frames too. They may have stopped just before the VCSEL came on.

Next live test (after a Full-field session):
`SCANCRAFT_VCSEL_EXTRA="0x48b 10" SCANCRAFT_EXPERIMENTAL_MODES=1 SCANCRAFT_RECORD_DIR=E:\Claude\recordings\<dir> examples/vcsel_record.rs <ip> 300 general`
and watch for the means to rise after the first second.

SDK property IDs seen with register writes (sdk.log*, "Set property extension type"):
`0B26` → `0x3001` (speckle/projector select, `0x3001 12`), `0B0A` → `0x48b`, `0B06` → `0x488`,
`0B2C` → `0xb1c`, `0B23` → exposure-type writes (`0x911`, `0x910`, `0xb08`, `0xb1b`, `0x48d`),
`0B0B` → `0xb05`, `0A04` → `0xa04`, `0A05` → `0xa05`, `0705` → `0x705`, `0702` → `0x702`,
`AAAA` → `0x707` (depth range). Long commands go through `sh /tmp/.longCmd.sh`.

## 6. Solved (2026-11-02): the 640x1024 pair holds both cameras

Requesting the IR pair (fmt 1 / display type 2) at **640x1024** on `camera_id=21` returns
1,310,720 bytes per frame = four 640x512 y8 slots: left camera ambient, left camera VCSEL-lit,
right camera ambient, right camera VCSEL-lit (`examples/pair_slots.rs`; plane 0 carries the frame
header; planes 0/1 share one viewpoint, 2/3 the other) [C]. At 640x512 the same request returns
only the first two slots, i.e. **one camera twice**, which explains everything in §3–5 (one
"dark" plane, the dotted plane matching neither rectified camera, 0.2 % stereo).

Host stereo on the lit slots (raw → rectified with the red calibration; slot 1 = left, slot 3 =
right; the swapped order gives 0.5 %): 19.9 % coverage on the desk scene, plane at 281 mm with
normal [0.083, −0.751, 0.656], against the device's Full-field depth of the same scene at 20.6 %,
281 mm, [0.085, −0.751, 0.654] [C]. Matcher ~147 ms per frame (200–600 mm window, CPU). The
driver streams it at 15–18 fps. No Full-field "arming" is needed: two VCSEL runs back to back
both gave lit pairs; the earlier arming effect belonged to the wrong layout.

Driver: `FrameLayout::IrSlotPairs`, `streams/vcsel.rs` (640x1024 request via
`FrameLayout::display_rows`, rectification of the lit slots). VCSEL Rapid is now Available in
`modes.rs`; depth window 200–600 mm as Revo sends it. Not yet tested: handheld tracking with it,
the other object presets (exposure/luminance), and the slot order after a fresh boot.

## 7. After the register fix (2026-11-10)

- Since firmware v5.3.99 none of the driver's `rk_preisp` writes had arrived (device-internals §1).
  With them arriving, Revo's VCSEL values apply (`0x3001 12`, `0x911 666`, `0x903 0x20`,
  `0xb08 1366`, `0x910 50000`, `0x48d 100`), and **which of the two exposures per camera is the
  lit one flipped**. The decoder therefore picks the brighter exposure per camera
  (`media.rs::slot_pairs`) instead of assuming a position.
- Stream: JPEG slot pairs on `camera_id=33&type_id=20` at 640x1024 (one 640x2048 image, ~170 KB);
  the lossless `camera_id=21` variant is 620 KB per frame and reached only 2-3 fps.
- Depth coverage on the static test scene rose from 27.7 % to 38.5 % with the real settings.
- Matcher: coarse-to-fine (`scancraft-stereo/src/hier.rs`), 148 ms -> 24 ms per frame, same
  agreement with the device's Full-field depth (MAD 0.28 mm, 96 % within 1 mm).
