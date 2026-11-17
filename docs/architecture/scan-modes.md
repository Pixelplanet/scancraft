# Scan modes: what the POP 4 offers, what combines, what works

The one place to look for "which modes exist and what state are they in". Details per mode live
in `docs/protocol/` (`modes.md` is Revo's full catalogue, `device-internals.md` the device side).
Status as of 2026-11-10, firmware v5.3.99, measured with `examples/mode_check.rs` on a static
desk scene (box on a marker turntable, ~280 mm).

## 1. A scan is four independent choices

| Choice | Code | Options on the POP 4 |
| --- | --- | --- |
| **Light source**: what lights the scene and how depth is obtained | `LightSource` | Full-field HD, VCSEL Rapid, Hybrid HD, Cross Lines, Single Line |
| **Tracking**: how frames are aligned to each other | `TrackingMode` | Feature, Marker, Global Marker, Color |
| **Object preset**: exposure, gain, depth window, filtering | `ObjectType` | Normal, Dark, Face, Shiny, Outdoor (per light source) |
| **Colour**: record the RGB camera for vertex colours / texture / splats | `ScanConfig::color_texture` | on / off (always on with Color tracking) |

Light source and tracking are *not* freely combinable. The device (Revo's parameter table for
model `0E1`) defines the valid pairs, and `pop4_modes()` in
`crates/scancraft-driver-revopoint/src/modes.rs` mirrors them. The UI only offers valid pairs.

One pass uses one combination (see `scan-sessions.md`); a project can hold passes made with
different combinations and align them afterwards.

## 2. Light sources

| Light source | Runs on its own? | Depth computed | Stream (fmt / type, media) | What it is for |
| --- | --- | --- | --- | --- |
| **Full-field HD** | yes | on the scanner | 3 / 4, `camera_id=21`, 640x512 | default: fine structured light, best detail, 250–500 mm |
| **VCSEL Rapid** | yes | on the PC (stereo on the dot pattern) | 1 / 2 at 640x1024, `camera_id=33&type_id=20` | fast capture, larger range (200–600 mm, up to 800), dark objects, outdoors |
| **Hybrid HD** | yes: it *is* the combination of both projectors | on the PC (dots + stripes) | 5 / 10, `camera_id=33&type_id=21` | Full-field detail with the dot projector's robustness |
| **Cross Lines** | needs Marker, Global Marker or Color tracking | on the PC (line triangulation) | v5.3.99: 7 / 4 at 1280x1024, `type_id=22` (lines + dots) | shiny metal, dark surfaces |
| **Single Line** | needs Marker or Global Marker tracking | on the PC | 1 / 2 at 1280x1024 | deep holes, grooves |
| Parallel Lines | — | — | — | not on the POP 4 |
| Auto Turntable | — | — | — | Full-field driven by the Bluetooth turntable; not implemented |

"Combined" light sources are therefore two cases: Hybrid HD (dot projector + stripe projector in
one frame) and, since firmware v5.3.99, Cross Lines (blue lines + dot projector in one frame, so
the dots can carry tracking while the lines carry the surface).

## 3. Valid combinations and their state

✔ = works end to end on the device (frames, geometry, static tracking). ◐ = the scanner side
works (lines lit, line geometry per frame); host-side tracking is not reliable yet. — = not a valid combination on the POP 4.

| Light ↓ / Tracking → | Feature | Marker | Global Marker | Color |
| --- | --- | --- | --- | --- |
| Full-field HD | ✔ | ✔ (Normal only) | ✔ (Normal only) | ✔ |
| VCSEL Rapid | ✔ | — | — | ✔ |
| Hybrid HD | ✔ | — | — | ✔ |
| Cross Lines | — | ✔ | ✔ | ✔ (lines + dots stream) |
| Single Line | — | ✔ | ✔ | — |

Object presets: Full-field Normal / Dark / Face; VCSEL Normal / Dark / Face / Outdoor; Hybrid
Normal / Dark / Face; lines Normal / Dark / Shiny / Outdoor.

### Measured (2026-11-10, 40 frames each, static scene)

| Combination | Frames | Depth coverage | Points / frame | Tracking (lost) | Notes |
| --- | --- | --- | --- | --- | --- |
| Full-field + Feature, Normal | 12.8 fps | 31.8 % | 97k | 0 | |
| Full-field + Feature, Dark | 12.2 fps | 34.6 % | 104k | 0 | |
| Full-field + Marker | 12.6 fps | 31.8 % | 97k | 0 | every second frame repeats the IR pair and is skipped |
| Full-field + Global Marker | 12.8 fps | 31.8 % | 97k | 0 | two-step workflow not exercised |
| Full-field + Color | 13.6 fps | 31.8 % | 97k | 0 | colour image on 35 of 40 frames |
| VCSEL + Feature, Normal | ~10 fps | 38.5 % | 126k | 0 | 24 ms stereo per frame since the coarse-to-fine matcher |
| VCSEL + Feature, Dark | stereo-limited | 42.9 % | 140k | 0 | first frames before the setup are dark |
| VCSEL + Color | stereo-limited | 38.5 % | 126k | 0 | colour on 34 of 40 |
| Hybrid + Feature | ~8 fps | 29.5 % | 97k | 0 | 57 ms depth per frame since the coarse-to-fine matcher (coverage then ~22 %, without the old background noise) |
| Hybrid + Color | stereo-limited | 29.5 % | 97k | 0 | colour on 35 of 40 (after the newest-frame hand-off fix) |

(`mode_check` computes the geometry twice per frame, so its ms/frame is about double the app's.)

Not yet verified for any of these: handheld motion (only Full-field Feature and Marker have been
scanned handheld, on firmware v5.3.94), and the Face / Outdoor presets.

## 4. What is still open

1. **Laser lines (Cross, Single).** Work with Marker and Global Marker tracking since
   2026-11-10: the marker tracker now detects the dots in the raw laser frames and carries the
   pose (static scene: Cross 60/60 frames tracked at ~1,200 line points per frame, Single 60/60 at
   ~220). The Dark preset works too (marker light raised, laser planes calibrated once per
   scanner and stored). Open: a moving scanner (untested), a full plane calibration (17 of 30
   cross planes so far). Cross Lines + Color runs on the combined lines + dots stream (format 7)
   since 2026-11-10: tracking on the dots, model from the lines (86/86 static frames). `docs/protocol/laser.md`.
2. ~~Stereo speed~~ improved 2026-11-10 with a coarse-to-fine matcher (`scancraft-stereo/src/hier.rs`:
   SGM at half resolution, then a +-3 px search at full resolution). VCSEL: 148 ms -> 24 ms per
   frame, same agreement with the device's Full-field depth (MAD 0.28 mm, 96 % within 1 mm),
   coverage 24.8 % -> 23.1 %. Hybrid: 210 ms -> 57 ms with the fringe-phase refinement kept, same
   local noise (0.2 mm), and far fewer random mismatches in the background (its lower coverage
   number is that noise disappearing). Live: VCSEL ~10 fps, Hybrid ~8 fps. A GPU matcher is no
   longer needed for usable rates; the remaining Hybrid time is phase maps and filtering.
3. ~~Hybrid + colour pairing~~ fixed 2026-11-10: the driver handed a slow consumer the oldest
   waiting frame, whose colour partner was already gone; it now hands over the newest.
4. **IMU.** Off on firmware v5.3.99 (the scanner reports status 0, also to Revo Scan). Tracking
   works without it but loses the motion prior.
5. **Global Marker workflow** (scan the marker frame first, then the object) and the **Auto
   Turntable** light source are not built.
6. **Handheld tests** of VCSEL, Hybrid and Color tracking.

## 5. How a mode is started (all modes, driver `device.rs::start_stream`)

close streams → display geometry → output format → open media stream → `set_trigger_mode=0` →
wait for the first frame → the mode's DSP messages (`streams/<mode>.rs::configure`, in Revo Scan's
order). Stop: `0x483`, laser diodes / IR LED / white LED off, `close_stream_all`, then drop the
connection. Why this order and no other: `docs/protocol/device-internals.md` §3.
