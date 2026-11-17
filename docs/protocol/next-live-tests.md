# Next live tests on the POP 4 (prepared 2026-11-02)

**Superseded 2026-11-10:** exposure, VCSEL and Hybrid are solved (root causes in
`device-internals.md`); the current open list is in `docs/architecture/scan-modes.md` §4. The text
below is kept for history; its register-level conclusions predate the fix of the register writes.

Status after the 2026-11-02 afternoon run: 1. exposure — no effect (Full-field ignores `0x912`/
`0x910`/`0x911` over Wi-Fi); 2. VCSEL — **solved** via the 640x1024 slot pair (vcsel.md §6);
3. Hybrid in Revo's order — still one leftover frame, wedge (hybrid.md). Open: Hybrid entered from a
running VCSEL session; handheld marker and VCSEL scans.

Ordered so the safe tests come first: a Hybrid wedge needs a power cycle, so it goes last. All
commands run from the repo with `CARGO_TARGET_DIR=E:/Claude/build`, `--release`, and
`-p scancraft-driver-revopoint --example …`; recordings go under `E:\Claude\recordings\<date>-<name>`.
Never wrap a streaming command in `timeout`; examples end their streams themselves.

After a power cycle the first one or two streams deliver nothing; run `live_scan <ip> 10` until it
reports frames (this also arms the VCSEL).

## 1. Exposure, done the POP 3 way (safe)
Background: prior-art.md §Exposure. Full-field stream running, writes spaced ~1 s.

```
PROBE_GAP_MS=1000 PROBE_STEPS="0x912 0+0x910 2700+0x911 700;0x912 0+0x910 8000+0x911 6000;0x912 1;0x912 2;0x912 3;0x912 0+0x910 7000+0x911 3000" exposure_probe <ip>
```
Expect: IR mean and depth coverage move with the exposure. If `0x912 3` (foreground) helps
coverage on a bright object over a dark background, expose it in the UI as an auto-exposure mode.
Also log the full reply body of each write (does the POP 4 ever answer `[ok]`?).

## 2. VCSEL: the 1.25 s that we never waited for (safe)
Background: vcsel.md, "Re-reading the 2026-11-01 latch". Run a Full-field session first.

```
SCANCRAFT_VCSEL_EXTRA="0x48b 10" SCANCRAFT_EXPERIMENTAL_MODES=1 SCANCRAFT_RECORD_DIR=E:\Claude\recordings\<dir> vcsel_record <ip> 300 general
```
Expect: dark frames for ~1 s, then both halves lit. If yes, run `vcsel_eval SRC=jpegraw` on the
recording (MIN_MM/MAX_MM to the scene). Variant: `SCANCRAFT_CONFIGURE_AFTER_OPEN=1` (driver now
waits for the first packet before configuring, as Revo's SDK does).

Keep in mind (prior-art.md): `0x3001 12` is the blue-laser speckle mode, and a stereo pair must come
from the same time slot. Compare the lit images with the blue calibration (`camparam_blue`).

## 3. Hybrid in Revo's order (may wedge; last)
Background: hybrid.md, "Revo's SDK waits for the first frame". Run a Full-field session first.

```
REVO_ORDER=1 WITH_RGB=1 hybrid_capture <ip> 15 E:\Claude\recordings\<dir>\hybrid.bin general
```
Expect: packets keep coming after the setup (Revo: steady ~21 fps, ~366 KB each). The planes must
show eight stripe images and two speckle images (contact sheet as in hybrid.md); a z16 + IR-pair
layout is a leftover frame, not Hybrid. If it streams, `hybrid_eval` the recording, then make
Hybrid available in `modes.rs` (the driver already uses Revo's order for it).

## 4. Handheld marker scan (needs the user)
Marker tracking was only tested on a static scene. Record a slow handheld orbit with
`SCANCRAFT_RECORD_DIR` set and replay with `pipeline_replay REALTIME=1 … marker 4`.
