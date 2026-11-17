# 2026-11-01 — `zx_media.cgi?camera_id=21` payload layouts (live sweep)

Device: POP 4, S/N `D00000000B0S00E1B`, firmware as installed, WiFi `192.168.178.34`.
Tool: `cargo run -p scancraft-driver-revopoint --example dump_stream -- <capture.bin> <out_dir>`.
Captures were 2–4 s curl dumps; raw captures are not committed.

## Transport (confirmed)

- `GET /cgi-bin/zx_media.cgi?camera_id=21` → `HTTP/1.1 200`, `Transfer-Encoding: chunked`, never ends.
- De-chunked body = repeated packets: `0d 0a`? then `44 33 22 11` (u32 LE `0x11223344`),
  `u32 LE payload_len`, then `payload_len` bytes of QuickLZ (flags `0x47`, 9-byte header,
  stored size == `payload_len`, decompressed size = W×H×bytes-per-pixel).
- ~15 fps at 640×512×2 (~1.2 MB/s compressed).

## Decompressed payload

Every frame starts with an in-band header overwriting the first pixels: ASCII `abcd`, then
metadata including what looks like a u32 timestamp at byte 20 and f32 values from byte 24
(currently parsed as IMU by `extract_embedded_imu` — **unverified hypothesis**).

`set_display_type` selects bytes per pixel; `set_depth_output_fmt` selects what the planes mean.
Both are sent via `zx_cmd.cgi?cam_type=mipi&...` (URL templates from `3DCamera.dll` @ `0x32f9380`, `0x32f9518`).

| fmt | type | bytes/frame | Layout (observed) |
|---|---|---|---|
| 1 | 1 | 327 680 | one 8-bit IR plane |
| 1 | 2 | 655 360 | **left IR y8 ‖ right IR y8** (laser lines visible in both) — what ScanCraft currently consumes |
| 1 | 3 / 4 | 983 040 / 1 310 720 | y8 ‖ y8 ‖ further planes (not yet identified) |
| 2 | 2 | 655 360 | z16 plane, all zero in laser mode (8 KB compressed) |
| 3 | 3 | 983 040 | z16 (zero) ‖ y8 |
| 3 | 4 | 1 310 720 | **z16 ‖ left y8 ‖ right y8** — firmware string `z16y8y8`; IR plane showed speckle pattern, depth plane all zero |
| 0, 2 | some | — | `503 ----please wait for a moment by tank` when switched too fast; retry after a delay |

Firmware `zxmain` names the formats `DSP_OUTPUT_FMT_CABLI`, `_DEPTH`, `_DEPTH_LEFT_RIGHT`,
`_S64Y8Y8` (strings @ `0xaf454`..`0xaf4a4`) and layouts `z16`, `pair`, `z16y8y8`, `z16y8y8p`
(@ `0xa6aa4`..). Mapping of numeric `fmt` → name is **not yet confirmed**.

## Consequences

- The earlier claim that `camera_id=21` is "16-bit depth in 0.01 mm" is **wrong for fmt=1**:
  it is a stereo IR pair. Depth must either be computed on the host (laser-line triangulation,
  as Revo Scan does) or obtained from the device's z16 plane (fmt 3, type 4) once the
  conditions for on-device depth are found (likely speckle projector + target in 150–400 mm).
- `reconstruction.rs::decode_pop4_stereo_frame` falls back to inventing depth from pixel
  brightness when stereo matching fails (`est_z_mm = 310 - norm_i*60`). That output is not
  real geometry and must be removed.

## On-device depth (confirmed live, same day)

With the speckle projector on (`rk_preisp` `0x912 1`, `0x3010 0x17 0`, `0x3001 12`, `0xb1b <luma>`,
`0xb04 1`) and two paper cups on a table at ~25 cm:

- fmt 3 / type 4: z16 plane 59 % valid, values 20 000–38 000 (= 200–380 mm at 0.01 mm/LSB),
  clean silhouettes of both cups. IR planes show the speckle pattern.
- fmt 2 / type 2 and fmt 3 / type 2: z16 only, same depth content, ~370 KB/frame compressed.
- Frame rate ~6–6.5 fps over WiFi for all three; it is device-limited, not bandwidth-limited.
- The depth engine outputs all-zero z16 frames for ~2 s after reconfiguration.
- Back-projected with the **rectified** left intrinsics (`pL` scaled ×0.5: fx = fy = 742.28,
  cx = 170.60, cy = 242.57) the table fits a plane with 0.26–0.28 mm RMS over ~69 k points,
  so the z16 values are Z depth, not range along the ray. `cameraMatrixL` (cx = 325) is the
  unrectified camera and is wrong for this map. The absolute scale is only checked against the stated
  distance; a ruler measurement of a known object is still open.
- The in-band header is 81 bytes: `abcd`, 16 bytes flags, u32 counter @20, f32 quaternion (w,x,y,z)
  @24 (unit norm), gyro @40, accelerometer @52 (|a| = 1.00 g at rest), Euler @64.
- `cam_type=mipi&get_exposure` / `set_exposure` / `get_gain` → `{"result":-10000}` (unsupported).

Implemented in `crates/scancraft-driver-revopoint/src/{media,calibration,device}.rs`; live check:
`cargo run -p scancraft-driver-revopoint --example live_scan -- <ip> 40 out.ply`.

## Busy replies and asynchronous format switches (confirmed)

- While reconfiguring, `zx_cmd.cgi` answers **HTTP 200** with the body
  `503 Service Unavailable ----please wait for a moment by tank`, not an HTTP 503. Clients must
  check the body.
- `set_depth_output_fmt` returns `{"result":0}` immediately and `get_depth_output_fmt` reports the
  new value at once, but the switch actually happens ~0.5 s later on a firmware thread
  (`set_depth_fmt_thread` in `zxmain`) and the device is busy for ~2 s. A media stream opened in
  that window delivers the *old* plane layout at the new size (e.g. fmt 1 data in a 4 B/px
  frame), which decodes as noise with no error. The driver waits for three consecutive non-busy
  `get_depth_output_fmt` replies before opening the stream.
- After a device reboot the stream config resets to `curr-resolution 1280x1024x3`; the HTTP server
  answers `500 Internal Server Error` for ~15 s while `zxmain` starts. The one reboot observed on
  2026-11-01 was the battery running flat (user confirmed), not a firmware fault.

## Open

- Verify absolute scale with a measured object (e.g. a 100 mm gauge block or ruler).
- Identify planes 3/4 for fmt 1 type 3/4.
- Confirm the `abcd` header layout against `3DCamera.dll`.
