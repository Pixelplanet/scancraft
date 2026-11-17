# Public prior art on Revopoint scanners (web research, 2026-11-02)

What other people have published, and how it bears on our open problems. Pages were read, not
downloaded or run. Tags: [C] confirmed by the source's own measurements, [I] inference.

## Sources

| Source | Device / transport | Most useful for |
| --- | --- | --- |
| [All3DScan RE notes](https://github.com/BR1ZB3AR/All3DScan/blob/main/docs/protocol/RE_NOTES.md) | **POP 4**, USB (UVC + XU) | stream layout, register stalls, speckle/blue modes, exposure |
| [revopoint-pop3-linux-wifi](https://github.com/TamedTornado/revopoint-pop3-linux-wifi) (`src/camera_control.rs`) | POP 3 Plus, USB + Wi-Fi | exposure sequence, `0x912` auto-exposure modes |
| [revopoint-python](https://github.com/HazenBabcock/revopoint-python) | MINI, Wi-Fi | `zx_media` camera ids, gain code |
| [CloudClean metroy-protocol](https://github.com/pranch555/CloudClean/blob/main/docs/metroy-protocol.md) | MetroY, USB HID | register stalls, init/resume writes |
| [rccn-dev/revopoint_camera](https://github.com/rccn-dev/revopoint_camera) (`3DCameraSDK-v3.2.229…/inc/3dcamera/hpp/Types.hpp`) | SDK v3.2.229 headers | `STREAM_FORMAT`, `PROPERTY_EXT` ids, enums |
| [Revopoint/3DViewer](https://github.com/Revopoint/3DViewer) | official viewer (GPL) | SDK usage (`thirdparty/3DCamera`) |

Nobody has published how VCSEL Rapid or Hybrid HD are started over Wi-Fi.

## Findings that bear on our problems

### Register-write stalls (Hybrid, exposure)
- POP 4 / USB [C]: two `0x911` exposure writes with no frame in between left the scanner on the
  bus but sending nothing; six frames apart was harmless (All3DScan `EXPOSURE_GAP_FRAMES = 6`).
- MetroY / USB [C]: writing `0x910` (frame time) alone stalled the stream until a watchdog reset;
  pair it with `0x911`. One register per command, ~150 ms apart; chained writes dropped the device.
- POP 4 / USB [C]: `0xb04` sent during connect did not take; sent after stream start it took
  about five frames later.
- [I] Our Hybrid starts wrote ~12 registers (incl. `0x910`, `0x911`) back to back within 0.3 s of
  opening the stream, before any frame. Together with Revo's SDK waiting for the first frame
  (hybrid.md), this is the leading explanation for "one frame, then silence".

### Stream layout = Hybrid layout
- POP 4 / USB [C]: the 640×2560 Y16 depth buffer is ten 320×512 images; images 0–3 (right camera)
  and 5–8 (left) are one-frequency four-step fringes (from ~1.1 s after stream start), images 4
  (right) and 9 (left) an always-on speckle pattern. That is the same ten-plane structure as
  Revo's Wi-Fi Hybrid frames (`S64Y8Y8`, 640×5120 JPEG: eight stripe planes + two speckle planes).
- Fringe and speckle images of one camera are offset by about ±12–13 px horizontally.

### Speckle (`0x3001 12`) is the blue laser
- POP 4 / USB [C]: `0x3001 12` + `0x48d 180` = **blue-laser speckle** mode, which needs the blue
  calibration (`/data/camparam_blue/camparam.yaml`). Leaving it with `0x3001 7` killed the
  projector until a USB reset. In blue modes only some time slots carry the pattern (planes 0, 2,
  5, 7) and a stereo pair must come from the **same time slot**; cross-slot pairs fell to ~0.6 %
  depth.
- [I] Revo's "VCSEL Rapid" also selects `0x3001 12`, and Revo downloads `camparam_blue`. Our
  one-lit-camera pair may be a pair from two different time slots; the "latch" behaviour matches
  their "projector dead until reset". vcsel.md should be read with this in mind.

### Exposure
- POP 3 / Wi-Fi [C]: manual exposure = `0x912 0` (depth auto-exposure off), `0x910 = exposure +
  2000`, `0x911 = exposure`; range from `cam_type=mipi&get_exposureRange`. `0x912` modes: 0 off,
  1 fixed frame time, 2 high quality, **3 foreground** (their default). The SDK header lists the same
  `AUTO_EXPOSURE_MODE` enum.
- POP 4 / USB [C]: exposure was the largest lever on real scenes (points/frame 7.8k at 1 ms →
  13.6k at 6 ms); values below ~700 µs clamp; a narrower depth range finds more points.
- [I] Our Full-field setup forces `0x912 0` (Revo never sends `0x912`) and our probes changed
  `0x911` without `0x910`. Untested on the POP 4 over Wi-Fi: the `0x912` modes (foreground could
  suit a bright object on a black turntable), and the full `0x912`/`0x910`/`0x911` sequence with
  writes spaced.
- MINI [C]: gain `0x903 = 16 × g` (matches our gain code).

### SDK enums (header v3.2.229)
- `STREAM_FORMAT`: MJPG 0x00, RGB8 0x01, Z16 0x02, Z16Y8Y8 0x03, PAIR 0x04, H264 0x05, RGBA 0x06,
  I8DS 0x100, XZ32 0x101, GRAY 0x102, XYZS 0x103, PMLD 0x104, PSLD 0x105, PPLD 0x106, PRCZ 0x107,
  PMCZ 0x108, Z16Y8Y8P 0x109, CPAIR 0x10A. (Revo 6's SDK adds S64Y8Y8 0x10B = Hybrid.)
- `PROPERTY_EXT` ids equal register numbers in many cases: 0x702 stream resolution, 0x705 contrast
  threshold, 0x707 depth range, 0x708 calibration stream format, 0x904 fake mode, 0x912 depth
  auto-exposure mode, 0x913 depth ROI, 0x914–0x916 HDR, 0xa04 multi-frame fusion, 0xa05 fringe
  pattern (0–6), 0xb00 LED on/off, 0xb01 LED control, 0xb0c stream-start init params, 0xb0e enter
  laser group, 0xb10 laser/LED luminance range. In this (older, multi-product) header 0xb04 is
  `AREA_CODE_LIST`, which does not match the POP 4's projector gate, so ids drift between products.
- `TRIGGER_MODE`: 0 off, 1 hardware, 2 software. A header comment: the SDK sends `0x483` (stop
  stream) and `0x481` (start stream) around a fringe-pattern change, which matches Revo's stop
  sequence `0x483`, `0xb04 0`, `0x481 1`, `0x483`.

### Other
- MINI [C]: `zx_media` camera ids 21 compressed, 22 raw (magic `03 07 02 01`), 32 untested,
  50 MJPEG colour, 80 H.264.
- POP 3 [C]: `set_display_type` is bytes per pixel, not a format enum (matches our notes).
