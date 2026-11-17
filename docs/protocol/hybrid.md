# Hybrid HD (`speckle_stripe`, light source 6)

Status (2026-11-10): **works.** Available in the app with Feature and Color tracking, ~8 fps,
57 ms depth per frame. The long search below for why the stream delivered one frame and then
hung the scanner ended with a one-line cause: the display type must be set **before** the output
format (last section). The intermediate hypotheses (bandwidth, the VCSEL state, the RGB camera,
waiting for the first frame) were all wrong; they are kept as a record
([documentation map](../README.md)).

Hybrid HD runs both projectors together: the VCSEL speckle projector and the stripe (fringe)
projector. The device does **not** compute depth in this mode. It streams raw image planes, and
the host computes depth by stereo matching. This note covers the stream format, the device setup
and the host algorithm that ScanCraft uses for it.

Evidence tags as in `modes.md`: **[C]** confirmed (log, capture or decoded data),
**[I]** inferred, **[H]** hypothesis.

## 1. Stream selection

| Item | Value | Evidence |
| --- | --- | --- |
| `set_depth_output_fmt` | 5 (`S64Y8Y8`, firmware `DSP_OUTPUT_FMT_S64Y8Y8`) | [C] Revo log `scan_surface=6` |
| `set_display_type` | 10 | [C] same log |
| Media query | `/cgi-bin/zx_media.cgi?camera_id=33&type_id=21` | [C] Revo log |
| Resolution | 640x512 per plane | [C] decoded frames |

## 2. Device setup (Revo Scan 6, 2026-06-19 14:55:14..27, `sdk.log_back`)

The sequence is sent before the stream format is selected
(`streams::hybrid::configure_device`):

```
cam_type=mipi&set_trigger_mode=0
rk_preisp 0x707 <zmin> <zmax>      depth window, 250 500 for all presets
rk_preisp 0x48d 80
rk_preisp 0x903 <gain*0x10>        gain index 1 / 4 / 3 (general / dark / face)
rk_preisp 0x910 7000               frame time, µs
rk_preisp 0x911 <exposure_us>      3000 / 5400 / 5100
rk_preisp 0xb1b 65                 speckle-stripe projector luminance (getSpeckleStripeLaserLuminance(POP4))
rk_preisp 0xb1c 18000
rk_preisp 0xb04 1
```

[C] for the values (log). [I] for the meaning of `0x48d`, `0xb1c` and `0xb04`, which is not
established.

## 3. Packet format

The outer framing is the normal media framing (`framing.md`). The payload is different
from the QuickLZ modes [C]:

- The payload is a **baseline JPEG** (starts `FF D8 FF`). It is greyscale content stored as YCbCr
  with neutral chroma. The image is `640 x (10*512)`: ten planes stacked vertically.
- The usual `abcd` in-band header (device counter, IMU) follows **after** the JPEG end marker
  `FF D9`. The IMU floats can contain `FF D9` themselves, so the decoder looks for
  `FF D9 'a' 'b' 'c' 'd'` first (`media::decode_stripe_speckle`).

Plane order, top to bottom:

| Plane | Content | Geometry |
| --- | --- | --- |
| 0..3 | left camera, fringe images, phase steps 0..3 | rectified by the device (red rig) |
| 4 | left camera, speckle image | **raw** (distorted, unrectified) |
| 5..8 | right camera, fringe images 0..3 | rectified |
| 9 | right camera, speckle image | raw |

[C] for the plane order and the rectified/raw difference: the fringe rows line up across the two
cameras, but the speckle rows do not until they are rectified with `camparam.yaml`.
[I] for the phase step order (consistent with `I_k = A + B cos(φ + kπ/2)`).

**Warm-up**: for about the first 1 s, the stripe planes are dark. Their mean 4-step modulation is
about 0.2 grey levels, against about 17 once the projector is lit [C]. `Hybrid::to_sensor_frame`
drops frames whose modulation is below 3.

## 4. Stereo geometry convention (`StereoRig`)

`disparity_offset_px = cx_right - cx_left` of the rectified pair, in pixels at the rig resolution.
On the POP 4 red set at 640x512 it is about +305 (+304.92). The cameras are verged by about 20°,
so the rectified principal points differ. With `d = u_left - u_right`:

```
Z = fx * baseline_mm / (d + disparity_offset_px)
```

Helpers: `StereoRig::depth_from_disparity(d) -> Option<f32>`, which returns `None` if
`d + offset <= 0`, and `StereoRig::disparity_from_depth(z_mm)`. The VCSEL host stereo uses the
same convention.

## 5. Host depth (`crates/scancraft-hybrid`)

1. **Rectify speckle**: the raw speckle planes are rectified with the red calibration
   (`DeviceCalibration::red_rectify_maps`). The `SensorFrame` then holds `ir` (the rectified
   speckle pair) and `fringes` (the 4-step pairs).
2. **Coarse match**: windowed ZNCC (9x9) on the speckle pair, plus a phase-agreement term
   `λ·mean cos(φL − φR)` (λ = 0.5). The sweep covers the disparity range of the working distance.
   The speckle removes the fringe period ambiguity, and the phase term sharpens the winning
   disparity.
3. **Fine match**: in the same rectified row of the right image, find the sub-pixel position
   whose wrapped phase equals the left pixel's phase, within ±1.5 px of the coarse winner.
4. **Filters**:
   - local speckle contrast, winner score and uniqueness margin against the best non-adjacent
     disparity
   - agreement with the 5x5 median of the disparity map (tolerance 1 px)
   - in fringe-covered areas, pixels without a phase match are dropped
5. **Speckle-only fallback**: without usable fringes, a parabolic sub-pixel fit is used
   (`match_pair`). This is the swap point for the shared `scancraft-stereo` matcher.

The output is a z16 depth map in 0.01 mm units on the left rectified camera. That is the same
camera and units as Full-field HD on-device depth, so the two can be compared pixel by pixel.

## 6. Status

- [C] The decoder and frame layout are tested with synthetic JPEG packets (`media` tests).
- The matcher is tested on synthetic rectified scenes (`scancraft-hybrid` tests):
  - a slanted plane at 300 mm has an RMS error of < 0.3 mm with fringes
  - a speckle-only flat plane at 350 mm has a mean error of < 2 mm
- **Device attempt, 2026-11-01** (after a power cycle, Full-field session first so the
  VCSEL could not still be latched dark by `0xb08`):
  - Full-field HD (`hybrid_capture ... fullfield`) streamed normally, about 7.7 MB in 3.3 s [C].
  - Hybrid HD, configured exactly as in section 2 after opening the media stream: the response
    headers arrived (lighttpd, chunked). The device then sent **532,478 bytes of the first
    packet and stopped** [C]. The packet header (`0d 0a 44 33 22 11`, length `0x0860f2` =
    549,106 B) announced more data, and the payload starts with a JFIF header. No `abcd`
    trailer arrived, so not even one complete frame was received.
  - After that connection was closed by the host, **every** media request hung (including
    Full-field), while the control CGI still answered. The same wedge was seen earlier the
    same day. It needs a power cycle [C].
  - [H] Causes to check next:
    1. Hybrid setup missing a step Revo performs beforehand, e.g. `0x701 1` to leave a
       laser mode (see `laser.md`). Section 2 was logged from a session already in Hybrid.
    2. The device JPEG encoder stalling on the 640x5120 image.
    3. The stream needing Revo's order (open, then configure, then wait about 5 s), with a
       stall of more than 4 s being normal before the first frame. `hybrid_capture` now
       waits 15 s for data before it gives up. Closing a stalled stream should be avoided,
       because it is the likely cause of the wedge.

    Compare against a Wireshark capture of Revo Scan 6 starting Hybrid HD on a freshly
    booted device.
  - Hybrid HD stays `NotImplemented` in `modes.rs` until a full sequence is recorded.
- **Still to do**: evaluation on a real Hybrid HD recording. To do it:
  ```
  cargo run -p scancraft-driver-revopoint --example hybrid_capture -- <ip> 4 hybrid.bin general
  cargo run -p scancraft-driver-revopoint --example hybrid_capture -- <ip> 4 ff.bin fullfield
  cargo run --release -p scancraft-driver-revopoint --example hybrid_eval -- hybrid.bin camparam.yaml ff.bin out
  ```
  `hybrid_eval` reports, for each frame:
  - coverage
  - matching time
  - the device frame interval
  - plane RMS and local RMS

  It also compares the Hybrid depth with Full-field HD on the same static scene.
- [H] Revo's own Hybrid matcher (`RPASpeckleStereo.dll`, S2 `speckle_stripe` dispSize 256 /
  minDisp 100 on CPU) probably works in a similar coarse-speckle / fine-phase way. This has not
  been checked against the binary.

## 2026-11-02: one frame, then silence (good Wi-Fi)

- `examples/hybrid_capture.rs` (Revo's register set incl. `0x488 0`, `0xb11 11000`, `0xb03 0`),
  run after a Full-field HD session: the body is **exactly one complete packet** (522,594 bytes),
  then no data for 15 s [C]. So the stream does not stall mid-frame; it delivers the first frame
  and stops. The faster link changed nothing, so bandwidth was not the cause.
- The media server was wedged afterwards (Full-field gave no frames; control CGI fine). The
  capture now sends `close_stream_all` while still holding the stalled connection; that did not
  prevent the wedge. (The Full-field check afterwards was wrapped in `timeout`, which killed it
  while waiting on the stream; no frames had arrived before that.)
- Revo's own Hybrid stream in the 2026-11-01 capture runs continuously: 366 packets of ~366 KB.
  Its register writes match ours, but Revo started Hybrid directly after its VCSEL session, i.e.
  with the VCSEL selected (`0x3001 12`) and armed, and without an emitter shutdown in between.
  [H] Hybrid needs the VCSEL armed/selected; ours was disarmed by the Full-field session's stop
  sequence. Next test after a power cycle: Full-field (arms) → VCSEL setup (no shutdown) → Hybrid.
- Revo's Hybrid stream is in `E:\Claude\captures\revo-hybrid.bin` (not committed) for decoder work.

### Hybrid waits for the RGB camera? (2026-11-02)

- Second live run with the VCSEL armed and selected first: again exactly one packet (1,036,212
  bytes, the lit dots compress worse), then silence, and the media server wedged again. The VCSEL
  hypothesis is out.
- Same packet format as Revo's (one baseline JPEG, 640×5120, three components); no traffic on
  TCP 8871 in Revo's capture either.
- Revo's Hybrid stream timing (cut capture, per packet): request at t=1073.63, **first byte at
  ≈1080.16**, the first four frames within 70 ms (buffered), then steady ~21 fps. Just before the
  first byte: RGB camera setup (`cam_type=usb&set_resolution=1&width=1280&height=1024` 1079.26,
  `set_aa_mode=1` 1079.73) and the RGB stream `camera_id=50&type_id=20` (1079.74).
  [H] Hybrid frames are gated on the RGB camera; without it one frame gets out and the pipeline
  stalls. Test: `WITH_RGB=1 examples/hybrid_capture.rs`.

### What the single frames are (2026-11-02, third run, `WITH_RGB=1`)

- With the RGB camera and stream running alongside: two packets (1.28 MB each), then silence and
  another wedge. RGB is not the missing piece either.
- Contact sheets of the ten 640×512 planes (`E:\Claude\eval\hybrid-*-sheet.png`): Revo's frame has
  eight dim stripe planes plus two speckle-lit IR planes. **Ours are not Hybrid frames:** plane 0–1
  hold z16 depth bytes, 2–3 a rectified IR pair, the rest noise, i.e. a Full-field (fmt 3 / type 4)
  frame inside the type-10 container. These are leftovers from the previous pipeline; the Hybrid
  pipeline itself never starts, and the stream then stops.
- Stopped live Hybrid tests for 2026-11-02 after three wedges. Next: find what Revo does that
  starts the fmt 5 pipeline (projector selection `0x3001 14/15`? the order of `set_display_type` /
  `set_depth_output_fmt` relative to the previous mode?), offline from the capture and the SDK
  disassembly before touching the scanner again.

### Revo's SDK waits for the first frame before configuring (sdk.log, 2026-11-01)

Revo's `cs::Camera::startStream` for Hybrid (format 267 = `S64Y8Y8` / 0x10B, "pop4 start mpjg
s64y8y8 compress processing", hardware pair decode, 640×5120): `set_output_format` (19:34:07.006),
stream `camera_id=33&type_id=21` opened (07.007), `set_trigger_mode=0` (07.710), then **nothing
until the first frame** ("recv Depth type first frame", 12.614, 5.6 s later; the call has a
20 s timeout). Only after that: `0x707`, the RGB stream, `ledAndLaserCtrl`, and the Hybrid
registers. Every one of our attempts wrote the Hybrid registers ~0.3 s after opening the stream,
i.e. while the device was still bringing up the fmt 5 pipeline.

Next live test: `REVO_ORDER=1 WITH_RGB=1 examples/hybrid_capture.rs <ip> 15 <out> general`
(trigger mode only, wait up to 20 s for the first packet, then the setup while reading).

### Revo's order did not start it either (2026-11-02, fourth run)

`REVO_ORDER=1 WITH_RGB=1`: two packets at once 0.1 s after the response headers (820 KB each, a
Full-field z16 + IR-pair layout again, i.e. buffered leftovers), the setup sent after the first,
then silence and a wedge. Waiting for the "first frame" triggers on leftovers; the Hybrid pipeline
never produces a frame of its own.

Remaining difference to Revo: Revo entered Hybrid directly from a running VCSEL session (fmt 1),
every attempt of ours came from Full-field (fmt 3), and the leftovers are fmt 3 frames. VCSEL now
works (vcsel.md §6), so the next test is VCSEL (640x1024 pair) → `close_stream_all` → Hybrid,
without the emitter shutdown in between.

## Solved (2026-11-10): display type before output format

`set_depth_output_fmt` reloads the DSP and builds the pipeline for the display type that is set
**at that moment** ([device-internals.md](device-internals.md) section 3). The driver used to
send the format first and the display type second, so the DSP built a Full-field layout into the
Hybrid container: the "leftover" frames seen above, then silence and a hung media server. Revo
sends the display type first. With that order (now the only order in the driver,
`control.rs::set_stream_format`) Hybrid streams continuously; reproduced the other way round with
`SCANCRAFT_FMT_FIRST=1` on firmware v5.3.99 (one frame, hang).

Current setup after the first frame (`streams/hybrid.rs`, Revo's sequence of 2026-11-07 15:23):
`0x707`, `0x48d 80`, `0x488 0`, `0x911`, `0x903`, `0xb1b 65`, `0xb11 6000`, `0x910 7000`,
`0xb1c 18000`, `0xb03 0`, `0xb04 1`.

Measured on a static scene: 29.5 % depth coverage and 0.2 mm local RMS with the original sweep
matcher (210 ms per frame); with the coarse-to-fine speckle match plus the unchanged fringe-phase
refinement (`HybridParams::fast_coarse`, default) 57 ms per frame, the same local RMS, about 22 %
coverage and no scattered mismatches in the background. With colour: image on 35 of 40 frames.
