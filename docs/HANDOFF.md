# Handover (2026-11-17)

For whoever continues this work, in particular on a machine other than the one it was developed
on. Read this, then [architecture/scan-modes.md](architecture/scan-modes.md) (what works) and
[protocol/device-internals.md](protocol/device-internals.md) (how the scanner really behaves).
`AGENTS.md` in the repository root holds the working rules and the dated session log.

The public repository starts from one squashed commit; the commit-by-commit history up to
2026-11-17 exists only on the original development PC.

## 1. Where things stand

**Update 2026-11-26 (branch `agent/project-file`, not merged): projects open with a
double-click.** A project is a folder with `<name>.scancraft` (older `project.json` folders still
open) or one `.scancraftz` bundle, which the app unpacks into a working folder while its tab is
open and writes back on Save. Open Project… takes either; the Save Project card saves as a folder
or a bundle (and moves the tab there); projects are locked while open; working folders left by a
crash are offered for recovery at start; `scancraft.exe <path>` opens a project and a second start
hands it to the running app (one app per user, so two apps never use the scanner). The installer
registers both file types. Details, decisions and what is left: [open-issues.md](open-issues.md)
4.1; layout: [architecture/scan-sessions.md](architecture/scan-sessions.md) section 8.

**Update 2026-11-24 late (original PC): start with [open-issues.md](open-issues.md).** The branch
below is merged into `main`. A review found bugs in it, and the new mesher was compared with the
old one on the Revo shell scans. It is smoother on plain areas but destroys small raised detail.
The user also added requests: New Project in a new tab with a confirmation instead of silently
discarding the project, consistent placement of confirm buttons, and an Undo/Redo timeline. All
of that is written down there, with a suggested order. None of it is fixed yet.

**Update 2026-11-24 evening (branch `feature/clean-per-pass-meshing`, second PC).** On top of the
solid-model commit:
- **File dialogs:** Import Scan, Add Project and Open Project open native Windows dialogs
  (`apps/scancraft/src/file_picker.rs`, `rfd`); Save Project and Export have a Browse button.
- **Workflow order is now Connect → Scan → Clean → Align → Mesh → Export** (user decision).
  Clean works per pass: every tool runs on each *shown* pass separately (and on the merged model
  when shown); hidden passes are untouched. A pass's cleaned surface is `ScanPass::edit` (own
  frame, observation counts in `confidence`) and replaces the tracked / imported surface for
  alignment, merge and saving (`pass-<id>/cleaned.ply` + `cleaned_observations.bin`). The old
  project-wide "cleaned cloud" is gone (old projects open it as the merged model). Isolate Object
  needs exactly one shown pass. A cleaned captured pass cannot be continued or have frames
  discarded until Reset Cleaning.
- **Meshing rework** (`scancraft-mesh`): Reconstruct = `implicit.rs` (normals re-fitted from
  positions, robust implicit MLS, thin-wall side test, vertex projection) + `denoise.rs`
  (fold repair, bilateral normal filtering); Smooth = the edge-preserving smoother; new Fill Holes
  tool (`holes.rs`, Liepa: minimum-weight triangulation, refinement, fairing by sparse Cholesky).
  Measured on the user's two-pass shell scan with `examples/mesh_project.rs` (rebuilds a saved
  project's merge, meshes, prints roughness / holes, renders PNGs); numbers in merge-and-ai.md §7.
- **Done 2026-11-25:** `scancraft-mesh::solid` (`build_solid`) is in the Mesh stage as the
  "Make Solid" tool, next to Reconstruct (it takes the points, not the mesh). Since 2026-11-25 it
  takes the included, placed passes as separate scans (`solid_input` / `solid_points` in
  `app_project.rs`: object only unless "whole scene", scan id per pass), the merge weights as scan
  priority (equal weights: averaged), optionally the most trusted pass as a sharp scan, and the
  tracker's marker map of each pass as listed markers (normal from the pass's surface). Two-pass
  shell project: watertight, wall 3.2 mm, 279 holes filled, 6 markers removed. **Done 2026-11-26:** marker
  maps are part of each pass (`ScanPass::markers`, saved as `pass-<id>/markers.json`, kept
  through tabs, undo, Add Project / Add Passes and deletion), and Import Scan reads Revo's
  `marker_framework.mkf` next to the file (layout: `protocol/revo-marker-framework.md`); a
  listed normal replaces the surfel average in Make Solid. Review fixes: Continue Pass merges the
  new capture's tracker map into the pass's map (`PassMarker::merged`, same sticker within 3 mm)
  instead of replacing it; a non-finite marker is left out when saving and a damaged entry no
  longer makes the whole `markers.json` unreadable; the Make Solid panel counts, and the view
  draws, only the listed markers on the pass's surface (`markers_on_surface`, shared with
  `solid_points`).
- **Done 2026-11-25:** the orientation tools of merge-and-ai.md §8 — Auto Orient (an
  Accept/Reject proposal), Orient by Datums (plane / axis / point grown from a click onto world
  planes and axes), Scale from Lengths for imported passes, and Horn's start plus a
  nearly-collinear warning in Point Pairs (`scancraft-align` `horn`/`fit`/`datum`/`orient`/`scale`,
  app `app_orient.rs` / `ui_orient.rs`, ribbon group "Orient" on the Align stage). On the real
  shell scan Auto Orient finds no rim (correct for that merge, see `examples/orient_ply.rs`) and
  stands the part on its largest face (44.6°); a part with a clean open rim has not been tried.

**Update 2026-11-24 (version 0.2.0).** Merging several scans works: Align All (pose graph,
automatic table removal, wall-aware quality for inside/outside scans), Refine as a preview, Merge,
Isolate Object, Import Scan (PLY/OBJ/STL) and Add Project. Tested on three Revo Scan scans of a
thin shell part; details, the user's decisions on AI segmentation and the backlog:
[architecture/merge-and-ai.md](architecture/merge-and-ai.md). Windows installer:
`packaging/windows/build_release.ps1`. Next on the laptop: test merging with passes scanned in
ScanCraft, then the solid-model pipeline (watertight mesh, hole filling, marker removal; being
finished in `scancraft-mesh`) and the AI segmentation work in the order of merge-and-ai.md §4.

- The POP 4 driver (`crates/scancraft-driver-revopoint`) starts, streams and stops every scan
  mode the scanner has, on **firmware v5.3.99.20261010**. All light source / tracking
  combinations of the POP 4 produce geometry and track a **static** scene without losing frames.
- **Not tested: a moving scanner**, except Full-field HD with Feature and Marker tracking, and
  that was on the older firmware v5.3.94. This is the most important open work and the reason
  for the next test session.
- The scanner's IMU is off on firmware v5.3.99 (it reports status 0 to Revo Scan as well), so
  tracking runs without a motion prior. Earlier handheld results had the IMU.

## 2. First steps on a new machine

1. **Get the program.** Either the release zip (see the repository README) or build from source:
   `cargo build --release -p scancraft` and, for the device tools,
   `cargo build --release -p scancraft-driver-revopoint --examples`.
2. **Network.** Scanner and PC on the same network. The scanner's signal quality matters a lot:
   connected through a repeater it delivered a fifth of the frame rate. Discovery is a UDP
   broadcast (`{intent:0}` to port 8080); `cargo test -p scancraft-driver-revopoint --test
   test_discovery -- --nocapture` prints the scanner if it is found. The address used in all
   notes, `192.168.178.34`, is from the original network; substitute the new one.
3. **Check the firmware version** (`…/cgi-bin/zx_cmd.cgi?download=/data/firmware_version`, or the
   driver's connect log). Everything here was established on v5.3.99. If Revo Scan has updated
   the scanner since, re-check the three things a firmware change broke last time:
   command encoding (device-internals §1), the IMU, and the per-mode message sequences.
4. **Run the mode check** for each mode before debugging anything in the app:
   `mode_check <ip> <fullfield|vcsel|hybrid|cross|single> <feature|marker|global|color> <general|dark|…> [frames]`.
   It goes through exactly the code the app uses and prints frames, geometry and tracking.
   `MODE_CHECK_VERBOSE=1` adds one line per frame.
5. **Laser modes, first run on a machine:** the laser light planes are calibrated once per
   scanner and stored in the user's data directory (`%LOCALAPPDATA%\ScanCraft\laser_planes_*.json`),
   so a new machine has none. Run one Cross Lines and one Single Line scan with the **Normal**
   preset on a matte scene first (about 40 frames, a few seconds); the Dark preset refuses to
   calibrate. See §5 for the quality of that calibration.

## 3. Rules for talking to the scanner

These are not style preferences; breaking them hangs the scanner's media server.

- **One client at a time.** Revo Scan, the app and the tools must not stream at once. The driver
  takes a lock file (`%TEMP%\scancraft-scanner.lock`) while streaming.
- **Never kill a streaming process** (no `timeout` wrappers, no Ctrl-C mid-stream, no closing the
  console). End through the app's Stop or let the tool finish; both stop capture, switch the
  emitters off and close the streams before dropping the connection.
- **If no frames arrive** (the driver reports "delivered no frame within 20 s" twice in a row):
  restart the scanner with `tools/scanner/scanner_reboot.sh <ip>` (`sync` + `reboot` through the
  control interface, back in about 35 s). The first stream after any boot often delivers nothing
  for several seconds; the driver waits and reconnects.
- **Lasers.** The driver switches the laser diodes off at stop (`0x483`, `0xb04 0`). After a
  crashed tool the blue lines can stay on: run any mode once and stop it cleanly, or restart the
  scanner.
- Start order for every mode, and why: device-internals §3. Do not reorder it.

## 4. Tools

Built with `cargo build --release -p scancraft-driver-revopoint --examples`; the main ones are
also in the release zip.

| Tool | Use |
| --- | --- |
| `mode_check` | one light / tracking / object combination end to end; the first thing to run |
| `stream_probe` | open any stream format in Revo's order, send timed DSP messages, per-plane brightness; for exploring |
| `exposure_probe` | step DSP messages on a running Full-field stream and watch IR / depth / colour |
| `live_scan` | minimal Full-field capture with plane-flatness check |
| `pipeline_replay` | replay a recorded **Full-field** stream (feature or marker tracking) through the app's pipeline, optionally in real time |
| `vcsel_eval`, `hybrid_eval` | depth quality and timing on recorded VCSEL / Hybrid streams |
| `laser_offline`, `laser_lines` | line extraction and planes on recorded laser streams |
| `tools/scanner/rd.sh "<cmd>"` | run a shell command on the scanner, print its output |
| `tools/scanner/dsplog_start.sh`, `dsplog_get.sh` | the DSP's own log: what each message meant and whether it was accepted |
| `tools/scanner/scanner_reboot.sh` | soft restart |

Recording: set `SCANCRAFT_RECORD_DIR=<folder>` before starting the app or a tool; every stream is
written as `pop4_<time>.bin` (raw packets) and `pop4_<time>_rgb.bin`. Sizes: Full-field ~5 MB/s,
Hybrid ~7 MB/s, lasers 1–8 MB/s. Keep recordings out of the repository and delete what is not
needed. Other switches: `SCANCRAFT_EXPERIMENTAL_MODES=1` (start modes marked not implemented),
`SCANCRAFT_LASER_PLANES=<file>` (use / write this plane file), `SCANCRAFT_FMT_FIRST=1` (the old,
wrong format order; only to reproduce the Hybrid hang).

## 5. Next: tests with a moving scanner

In this order (most likely to work first). For each: markers in view where the mode needs them,
slow steady motion around a matte object at 25–35 cm, recording on.

1. **Full-field HD + Feature**, then **+ Marker**: the baseline; confirms the new firmware and the
   rewritten start sequence did not change handheld behaviour. Now ~13 fps instead of 6.5, no IMU.
2. **VCSEL Rapid + Feature** (~10 fps) and **Hybrid HD + Feature** (~8 fps). Expect tracking to
   need slower motion than Full-field; Revo Scan struggles at these rates too.
3. **Cross Lines + Marker**, **Single Line + Marker**: pose comes from the markers alone, so at
   least 4–5 must be visible in both cameras all the time.
4. **Color tracking** (Full-field, then Cross Lines): needs a textured object; the white LED is
   not switched on by the driver.

What to look at when it goes wrong: the app's status pill (Tracking / Slow down / Tracking lost),
the frame rates in the status bar, and for marker modes the marker overlay (green = used).
A recording plus the mode and what was done is enough to analyse afterwards — but note the gap
in the next section: only Full-field recordings can be replayed through the pipeline today.

## 6. Known gaps and traps

- **Replay tools cover Full-field only.** `pipeline_replay` cannot yet decode VCSEL, Hybrid or
  laser recordings through the app pipeline (the per-mode evals only compute depth). Extending it
  to build the mode's `ModeStream` offline is the first thing to do before analysing handheld
  recordings of those modes.
- **Laser plane calibration is partial and scene-dependent.** The set made on the original PC had
  17 of the projector's 30 cross-line planes; lines without a plane produce no points. A flat
  matte scene at 25–30 cm gives a better set. Decoding the factory model in
  `/data/camparam_blue/metroExtra.bin` (laser.md §4, not solved) would remove the calibration.
  The app does not yet tell the user that calibration is running or was refused; it shows no points.
- **Cross Lines + Color** keeps a second, dense model inside its pose estimator
  (`pipeline/src/dots.rs`); memory on long scans is untested.
- **Global Marker** runs only its first phase; the scan-markers-then-object workflow and the
  marker library are not built. **Auto Turntable** (Bluetooth) is not built.
- **Exposure and gain** work in Full-field HD since the register fix and the sliders are enabled
  again (`manual_exposure: true`); the app UI around them was last checked while they were hidden.
- **Presets other than Normal and Dark** (Face, Shiny, Outdoor) are untested.
- **USB** is not implemented; Wi-Fi only. The All3DScan project documents the POP 4's USB side
  (protocol/prior-art.md).
- `mode_check` computes the geometry twice per frame, so its ms/frame is about double the app's.
- Test data paths in the notes (`E:\Claude\…`) are from the original PC. Use any folder outside
  the repository; nothing in the code depends on that path.
- The Python client under `client/`, `scripts/`, `tests/` is an early reference client from the
  first protocol work. It predates all of the above and is not maintained.

## 7. Map of the documentation

[docs/README.md](README.md) lists every document with how far it can be trusted. Short version:
`architecture/scan-modes.md`, `protocol/device-internals.md` and the dated final sections of
`protocol/vcsel.md`, `hybrid.md` and `laser.md` are current; the older protocol notes contain
correct structure but several conclusions that were later overturned.
