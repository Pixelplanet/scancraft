# Project: ScanCraft (open 3D scanner software) and its Revopoint POP 4 driver

## Start here

1. `docs/HANDOFF.md` — where the project stands, first steps on a new machine, the rules for
   talking to the scanner, the next tests.
2. `docs/architecture/scan-modes.md` — every scan / tracking mode and its measured status.
3. `docs/protocol/device-internals.md` — how the scanner really behaves; check it before changing
   anything in the driver.
4. `docs/README.md` — map of all documents and which older ones are superseded.

## Purpose

ScanCraft is vendor-neutral 3D scanning software in Rust (capture, tracking, alignment, cleanup,
meshing, export). Scanner support lives in driver crates. The first driver is for the
**Revopoint POP 4** over Wi-Fi, written from our own analysis of how **Revo Scan 6** talks to
hardware we own (interoperability). That analysis is where this repository started, which is why
the tooling notes below are about reverse engineering:

1. **Transport** — HTTP CGI for control (`zx_cmd.cgi`) and media (`zx_media.cgi`), UDP discovery.
2. **Framing** — media packets (magic, length, QuickLZ or JPEG payload, in-band header).
3. **Session** — discovery, calibration download, per-mode start and stop sequences.
4. **Payload formats** — the plane layouts of each stream format.

All four are answered for Wi-Fi and implemented in `crates/scancraft-driver-revopoint`; USB is open.

## Hardware & Software Under Test

| Item | Detail |
| --- | --- |
| Scanner | Revopoint POP 4 |
| Host software | Revo Scan 6 — native C++/Qt Windows app (not Electron) |
| Install path | `E:\Revopoint\Revo Scan 6` |
| Connectivity | USB and WiFi (scanner can join a network); WiFi enables traffic capture from the host |

### Relevant binaries in the install

| File | Why it matters |
| --- | --- |
| `3DCamera.dll` | Core scanner/device SDK — most likely home of the protocol implementation |
| `httpFileTransfer.dll` | HTTP-based transfer paths (possibly WiFi upload/control) |
| `btaccessories.dll` | Bluetooth accessory communication |
| `libssl-3-x64.dll`, `libcrypto-3-x64.dll` | TLS/crypto — indicates encrypted transports in some paths |
| `common.dll`, `controlbase.dll`, `controlcomponent.dll` | Likely shared control/command logic |
| `3DGSScene.exe` | Main UI executable (entry point for module loading) |

## Tooling

### REA (Reverse Engineer Anything)

- Upstream: <https://github.com/morluto/rea> (MIT)
- Installed globally via npm: `rea-agents` v6.4.0
- Executable: `%APPDATA%\npm\rea.cmd`
- Provides MCP tools for static analysis (binaries, .NET, JS/Electron, captures),
  process capture, and evidence-backed reports.
- Available native analysis provider on this machine: **Ghidra (Windows P0)**,
  which accepts approved x86/x86-64 PE `.exe`/`.dll` targets.

Useful commands:

```powershell
rea doctor                                     # environment readiness
rea capabilities                               # what providers/operations are available
rea inspect-artifact "E:\Revopoint\Revo Scan 6\3DCamera.dll"
rea analyze "E:\Revopoint\Revo Scan 6\3DCamera.dll"
rea search <pattern> --binary "E:\Revopoint\Revo Scan 6\3DCamera.dll"   # strings/procedure names
rea function <address> --binary "E:\Revopoint\Revo Scan 6\3DCamera.dll"
rea xrefs <address> --binary "E:\Revopoint\Revo Scan 6\3DCamera.dll"
```

### Traffic capture

- **USB**: USBPcap + Wireshark while Revo Scan 6 connects/starts a scan.
  Capture device enumeration first, then an idle session, then a scan session.
- **WiFi**: put scanner and PC on the same network (the scanner can also be
  reached over WiFi for capture). Find the scanner's IP via the Revo Scan UI,
  router DHCP leases, or `arp -a`; then Wireshark on the Wi-Fi adapter filtered
  by that IP. Use `Get-NetTCPConnection` / `netstat -anob` (elevated) to map
  ports to `3DGSScene.exe` and its helper processes. Note whether traffic is
  plaintext or TLS (OpenSSL DLLs are present, so assess encryption early).

### Other useful tools

- `strings` / Sysinternals `strings64` for quick embedded-string triage.
- Sysinternals Procmon for file/registry/network traces of Revo Scan 6.
- `netstat -anob` (elevated) while the app talks to the scanner.

## Deliverables (original protocol-analysis list; see docs/README.md for what exists now)

1. `docs/protocol/overview.md` — transports, discovery, ports/endpoints.
2. `docs/protocol/framing.md` — byte-level packet structure with annotated hex.
3. `docs/protocol/commands.md` — request/response opcode catalogue.
4. `docs/protocol/captures/` — sanitized capture notes (no personal data).
5. Minimal reference client (language TBD) that performs handshake + one scan.

## Working Rules

- **Several agents work in parallel. Each one works in its own git worktree on its own branch,
  never in the shared checkout** (others' uncommitted changes show up there and get mixed into
  commits). Start with
  `git fetch origin && git worktree add ../scancraft-<task> -b agent/<task> origin/main`, work
  and build only there, with its own `CARGO_TARGET_DIR` (e.g. `../scancraft-target-<task>`).
  Finish by rebasing on `origin/main`, running the tests of what you touched, and pushing
  `agent/<task>`; merge into `main` only when the user says so. Remove the worktree afterwards
  (`git worktree remove`). Commit author: `Pixelplanet <Pixelplanet@users.noreply.github.com>`.
- Analyze only software and hardware we own or are authorized to examine;
  purpose is interoperability and research (see REA's `DISCLAIMER.md`).
- Record evidence (file offset, function address, capture frame number) for
  every protocol claim. Distinguish confirmed facts from hypotheses.
- Do not commit proprietary binaries or raw captures containing personal data.
- Keep notes in this repo (not just chat) so other agents can continue.
- **Test data stays out of the repository and off small system drives.** Recordings
  (`SCANCRAFT_RECORD_DIR`), stream dumps, packet captures, evaluation output and scratch builds go
  to one data folder outside the repo (on the original development PC: `E:\Claude`, with
  `recordings\<date>-<name>`, `captures`, `build`; the notes refer to those paths). Streams are
  several MB per second: on 2026-11-01 test scans filled a whole drive. Delete recordings that are
  no longer needed. Point `CARGO_TARGET_DIR` at a drive with room (a full build is several GB).
- **Scanner safety rules** (one client at a time, never kill a streaming process, how to recover a
  hung media server): `docs/HANDOFF.md` section 3. They are mandatory.
- **Public repository.** No device serial numbers, user names, e-mail addresses, Wi-Fi
  configuration or raw captures in commits. The serial in the notes is a placeholder
  (`D00000000B0S00E1B`; the real one also ends in the model code `0E1` + `B`).

## Status / Next Steps

Kept in `docs/HANDOFF.md` (sections 1, 5 and 6) and `docs/architecture/scan-modes.md` (section 4).
In short: every POP 4 mode works on a static scene on firmware v5.3.99; scanning with a moving
scanner is the next thing to test; Global Marker workflow, Auto Turntable, USB and the factory
laser model are not built.

## Session Log

- 2026-10-12 — Installed `rea-agents` v6.4.0 globally; confirmed Ghidra Windows
  provider. Inventoried `E:\Revopoint\Revo Scan 6`. Identified likely protocol
  binaries. (Scaffolding commit.)
- 2026-11-01 — Live sweep of `zx_media.cgi` output formats: fmt 1/type 2 (what the app used) is a
  stereo IR pair, not depth; the old `decode_pop4_stereo_frame` invented depth from brightness.
  On-device depth confirmed in speckle mode (fmt 3/type 4 = z16‖IR-L‖IR-R, ~6.5 fps, table plane
  0.26 mm RMS with rectified `pL` intrinsics). Driver rewritten (`media.rs`, `calibration.rs`,
  `device.rs`); `DepthMap` now carries `PinholeIntrinsics` + `to_point_cloud`. Laser-line modes
  return an error until host-side triangulation exists. Notes:
  `docs/protocol/captures/2026-11-01-media-stream-layouts.md`.
- 2026-11-01 (later) — Tracking/fusion rewritten (`crates/scancraft-slam`): point-to-plane ICP
  (6-DoF, coarse-to-fine 30→2 mm gates, Huber) against a voxel-map world model; candidate
  guesses = IMU rotation applied as an orbit about the observed surface, constant velocity,
  last pose; lost frames are never fused; fusion merges into existing surfels so holding still
  adds nothing. POP 4 IMU mounting found from data: IMU x,y,z → scanner +X,+Z,−Y (accelerometer
  "up" vs table normal over a handheld sweep, median 16.7° vs ≥41.8° for every other axis
  mapping; `examples/imu_extrinsic_search.rs`). Reported via `DeviceInfo::imu_to_sensor`.
  Handheld recording (raw capture kept out of git): 58 frames tracked through ~50° of orbit,
  top view shows both cup rims as single rings; tracking then lost on a dark towel and on
  never-scanned desk areas (legitimate). Replay tool: `examples/replay_track.rs`
  (env FRAMES, TOP_VIEW, FUSED_PLY, NO_IMU). Meshing fixed by a subagent (welded MC, sparse
  SDF, Taubin smoothing, QEM decimation); default cell 1.0 mm.
  Open: IMU quaternion convention (body→world assumed; replays don't separate the two),
  fine IMU extrinsic, relocalisation after loss away from the model, loop closure/drift.
- 2026-11-01 (later) — User reported points being placed wrongly without "lost" showing.
  Cause: point-to-plane acceptance (inlier ratio + residual) passes frames that slide along the
  table. Added a visibility check (`Tracker::register_depth`): project the model into the new
  depth image; the fraction of comparable model points clearly *in front of* the measurement
  must stay ≤ 15 %. On the handheld recording good frames score 1–13 %, the previously accepted
  bad frames 50–57 score 70–90 % and are now rejected; re-lock at frame 78 scores 9 %. Viewport
  pill now says "Tracking lost · aim back at the scanned area" / "Slow down". POP 4 frames
  arrive in pairs (device counter +56 / +255 ms). kiddo ImmutableKdTree panics on exact
  coordinate ties; targets get ~1 µm deterministic jitter.
- 2026-11-01 (later) — Modes: research subagent wrote `docs/protocol/modes.md` from Revo's
  decoded `deviceScanParam.json`, Revo's own logs (`%LOCALAPPDATA%\Revopoint\RevoScan6\logs`)
  and disassembly. New `scancraft_driver::{LightSource, TrackingMode, ObjectType, ModeSupport}`
  (Revo numeric values); `ScanConfig.{light,tracking,object}`; capabilities report availability.
  POP 4 driver now uses Revo's exact Full-field HD sequence per object preset (0x707 window,
  0x911 exposure, 0x903 gain*0x10 [gain code inferred], 0x910 7000, 0x705 threshold, 0xa04
  fusion). Verified: depth identical with/without the old VCSEL commands (firmware drives the
  coded projector itself in fmt 3/type 4); live plane RMS 0.205 mm at 6.4 fps with the General
  preset. Tracker: keyframes (stored + fused only when moved ≥3 mm/2°), discard/restore +
  rebuild, relocalisation against keyframes (IMU-orientation-ranked, centroid-seeded), tentative
  until the next frame confirms, stricter thresholds; visibility conflict limit 12 %, depth-edge
  pixels excluded. Raw recording: env `SCANCRAFT_RECORD_DIR`. Old `LaserSourceMode` /
  `ScanTrackingMode` to be deleted once the UI is migrated.
- 2026-11-01 (evening) — Parallel mode work merged into `feature/scan-modes` (projects/alignment,
  markers, colour, laser lines, VCSEL, Hybrid, splatting, turntable/noise). **Scanner media-server
  wedge:** twice the POP 4's `zx_media.cgi` stopped answering (control CGI still fine) until a
  power cycle. Triggers seen: a capture killed mid-stream (shell `timeout` wrapper) during heavy
  parallel use, and a Hybrid HD stream that stalled mid-frame and was then closed by the host.
  Rules: one scanner client at a time (lock), always end streams via stop/`close_stream_all`,
  never kill a streaming process, and treat Hybrid HD streaming as unsafe until the stall is
  understood. `0xb04` gates the selected projector (laser modes: 1 = lasers on; Full-field HD
  needs 1) — see `docs/protocol/laser.md`. VCSEL (one lit IR plane, likely wrong stream) and
  Hybrid (stream stalls) remain NotImplemented; a Wireshark capture of Revo Scan starting those
  modes on a fresh scanner is the recommended next step.
- 2026-11-02 — Revo Scan 6 capture analysed (`docs/protocol/captures/2026-11-01-revo-modes-capture.md`):
  UDP discovery `{intent:0}` → port 8080 (Find Scanners fixed), Revo's per-mode start sequences,
  exposure slider = `0x488 1`/`0x911`/`0x903`/`0xb07 0` (no measurable effect in Full-field HD;
  device rates fmt 3 at 5 fps, we get 6.5). **VCSEL:** Revo's stream has a lit stereo pair and our
  host stereo works on it offline; live we still get one lit camera, and every VCSEL run needs a
  Full-field session first (vcsel.md §5). **Hybrid:** one complete frame, then silence, and the
  media server wedged (power cycle); Revo's continuous Hybrid stream decodes fine with our pipeline
  (~21 fps, plane RMS ~0.2 mm). [H] Hybrid needs the VCSEL armed/selected first (hybrid.md).
  Static marker scene: 0 lost frames, 15 ms per frame. Test data now lives under `E:\Claude`.
- 2026-11-10 — **Firmware is now v5.3.99.20261010** (Revo Scan updated it on 2026-11-07). Read
  `docs/protocol/device-internals.md` first: the control CGI is a root shell (`system_cmd` +
  `download`), the DSP logs the meaning and result of every `rk_preisp` message to the kernel
  log (`tools/scanner/dsplog_start.sh` / `dsplog_get.sh`), `sync` + `reboot` restarts the
  scanner in 35 s and clears a hung media server (no power cycle), and a format change reloads
  the DSP. Root causes fixed: (1) the new CGI does not decode `%2F`, so every register write of
  the driver went into a junk file; (2) display type must be set before the output format
  (Hybrid's "one frame then hang"); (3) setup must follow the first frame. One start/stop
  sequence for all modes now (`device.rs::start_stream`), per-mode messages as Revo sends them
  on this firmware. Working end to end (static scene, `examples/mode_check.rs`): Full-field HD,
  VCSEL Rapid (JPEG slot stream, lit exposure chosen by brightness) and Hybrid HD, each with
  Feature and Color tracking, Full-field also with Marker / Global Marker. Exposure and gain work
  (`manual_exposure: true`). IMU is off on this firmware (`has_imu` now read from the device).
  Open: laser lines (too faint, new format-7 stream, `laser.md`), GPU stereo, Hybrid colour
  pairing, handheld tests. Mode overview: `docs/architecture/scan-modes.md`.
- 2026-11-10 (later) — All POP 4 combinations of Revo's table now run end to end on a static
  scene (`mode_check`, see `docs/architecture/scan-modes.md` for numbers). Laser lines: Revo's
  Marker-mode sequence read from the DSP log during a Revo scan; the marker tracker detects dots
  in the raw laser frames; Dark preset (fill LED >= 20); laser planes calibrated once per scanner
  and stored in `%LOCALAPPDATA%/ScanCraft/laser_planes_*.json` (17 of 30 cross planes so far).
  Cross Lines + Color runs on the lines + dots stream (format 7, `streams/lines_dots.rs`,
  `pipeline/dots.rs`: track on the dots, fuse the lines). Stereo: coarse-to-fine matcher
  (`scancraft-stereo/src/hier.rs`), VCSEL 148 -> 24 ms, Hybrid 210 -> 57 ms per frame. Driver:
  a slow consumer gets the newest frame (fixed colour pairing). Not done: any test with a moving
  scanner for VCSEL / Hybrid / lasers / Color, Global Marker two-step workflow, Auto Turntable,
  `metroExtra.bin` (factory laser model), IMU on firmware v5.3.99.
- 2026-11-17 — Published to `github.com/Pixelplanet/scancraft` as one squashed commit (the earlier
  history stays on the original PC); serial number and user name replaced by placeholders. Added
  `README.md`, `docs/HANDOFF.md` (handover for work on another machine), `docs/README.md`
  (document map with trust levels), status banners and closing sections on the older notes
  (`vcsel.md`, `hybrid.md`, `laser.md`, `modes.md` section 8, the first protocol write-ups). A
  statically linked Windows build of the app and the main check tools is attached to the GitHub
  release. No scanner tests in this session (scanner offline).
- 2026-11-24 — Merging and AI plan (`docs/architecture/merge-and-ai.md`, user decisions in §1;
  research in `docs/research/2026-11-24-*`). Licence files added. Alignment rebuilt and tested on
  three Revo Scan scans of a thin shell part (one outside, two inside, which share only the rim
  with the outside): exact k-d tree (kiddo mis-handles exact ties), table veto, wall-aware
  ("shell") quality with piercing and slide checks, `align_robust`, `align_all` pose graph
  (composes S1↔S2 through S3), `refine_shell`; object isolation `scancraft-fusion::segment_object`
  used by default. App: Align All, Refine preview, stale cleaned/mesh fixes, Isolate Object, Import
  Scan (PLY/OBJ/STL), Add Project. Checked against Revo's `marker_framework.mkf` marker positions:
  0.2–0.27° / ≤0.6 mm. Real-data test: `crates/scancraft-align/tests/real_data.rs`
  (`SCANCRAFT_ALIGN_DATA`). Solid-model meshing (watertight wall, holes, marker removal) in progress
  in `scancraft-mesh`.
- 2026-11-24 (evening, second PC) — Branch `feature/clean-per-pass-meshing` on top of the
  solid-model commit. Native file dialogs (`file_picker.rs`). Stage order Connect → Scan → Clean →
  Align → Mesh → Export; Clean per shown pass (`ScanPass::edit`, saved as `pass-<id>/cleaned.ply`
  + observation counts), the single "cleaned cloud" removed. Meshing: the user's scan was rough
  because merged normals were 10–13° off the surface (points fine: ±0.04 mm between passes);
  new `implicit.rs` (normal refit, RIMLS, projection), `denoise.rs` (bilateral normal filtering),
  `holes.rs` (Liepa fill, written by a subagent), Fill Holes tool. Measuring tool
  `crates/scancraft-mesh/examples/mesh_project.rs`. Not wired yet: `solid::build_solid` in the
  app. See `docs/HANDOFF.md` §1 (top update). Test data stays in an untracked `Scans/` folder
  (excluded locally via `.git/info/exclude`).
- 2026-11-24 (late, original PC) — Branch merged into `main` (author e-mail rewritten to the
  noreply address). Workspace tests: 303 passed, 7 ignored. Review plus adversarial
  verification and an old-vs-new mesher comparison on the Revo shell scans; results and the
  user's new requests (project tabs, button placement, history timeline) are in
  `docs/open-issues.md`. Nothing fixed yet. Commit author must be the GitHub noreply address on
  every machine.
- 2026-11-25 (second PC) — Fixed open-issues 1.1 (cleaning drops stale merged model / mesh),
  1.2 (cleaned passes seed Same Position), 1.3 (leave-one-out stray filter in `implicit.rs`)
  and 1.6 (`mesh_project` defaults), with tests. Next per `docs/open-issues.md`: 3.1 (project
  tabs, confirm before discarding), then 2.2 (mesher destroys small relief).
- 2026-11-25 (second PC, later) — Fill Holes: patches are blended into their surroundings
  (`HoleFiller::blend_rings`, default 2) to remove the crease at stair-stepped rims; the direct
  fairing solve now covers holes up to 1.5 M new vertices (open-issues 1.7: a 1 193 mm hole fills
  in 28 s). `mesh_project` close-ups use a grazing light and take `--fill MM --blend N`.
- 2026-11-25 (second PC, mesher) — Open-issues 2.2 (`scancraft-mesh/src/implicit.rs`), measured
  on a synthetic relief and the two-pass shell project (the three Revo scans are on the original
  PC: check the moulded lettering there). Causes: grid corners beyond the support radius stayed
  unknown (holes along every step: wider kernel for those, `eval_corner`); "spacing" was the
  nearest-neighbour distance, below the real pitch, so the normal refit used ~10 points and relief
  was thrown out as outliers (`point_spacing` now from density); sparse second layers behind the
  surface at pen marks (`dense_layers`); the thin-wall side test trusted one normal (vote);
  rims followed the voxel grid (`round_open_edges`). Two-pass project at app defaults: 381 → 123
  boundary loops (old mesher 110), roughness 2.14° → 1.64° (old 2.65°). Three new tests.
  `mesh_project` prints loop sizes and takes `--zoom U,V,S`. Work in a shared checkout with
  another agent: use `git worktree` for your own changes.
- 2026-11-25 (second PC, mesher, later) — Mesh stage has a **Make Solid** tool (`Tool::MakeSolid`,
  ribbon group Create): `scancraft-mesh::solid::build_solid` on the working points as an undoable
  background job whose result becomes the mesh; settings cell size (0.4 mm) and marker removal.
  All points go in as one scan (no scan priority / sharp scans, no listed markers yet). On the
  two-pass shell project `solid_ply` takes 18 s and gives a watertight solid (wall 3.16 mm, 241
  holes filled, 7 marker sites). App test
  `make_solid_replaces_the_mesh_with_a_closed_solid_and_undo_takes_it_back`; not clicked through
  in the app.
- 2026-11-25 (second PC, later) — Project tabs (open-issues 3.1, `app_tabs.rs`): New / Open in a
  new tab, close and quit ask Save / Discard / Cancel, per-tab history; previews no longer mark a
  project changed. Script steps `newtab`, `tab:N`, `closetab:N` (`--auto`). Parallel agents on
  `scancraft-mesh` (open-issues 2.2, then Make Solid) and a new `scancraft-segment` crate (SAM 2.1
  on ONNX Runtime).
- 2026-11-25 (second PC, UI cleanup) — Open-issues 3.2 (`crates/scancraft-ui/src/kit`,
  `apps/scancraft`): one button rule via `Actions`/`action_row`/`dialog_footer`, `Button::Danger`,
  `confirm_modal`, `menu_item`, `close_button`, `note_with_tip`; wording glossary
  `docs/architecture/ui-wording.md`; notes shortened to 1–2 lines with tooltips, Fill Holes form,
  disabled buttons everywhere show their reason (`rb`, `edit_blocked`), tab strip ellipsises
  96–220 pt with a chevron overflow menu. New `--auto` steps `tool`, `closetool`, `size`,
  `rename`, `askquit`, `selectnode`, `rclick`, `hover` for screenshot checks.
- 2026-11-25 (second PC, branch `agent/history`) — Undo/Redo timeline (open-issues 3.3): shared
  layers make snapshots cheap, history bounded by 2 GiB / 200 steps, History button + timeline
  card with jump to any step (`ui_history.rs`). Open Project picked while busy now opens when free
  (1.4). Script steps `history`, `undo`, `removeplane`, `outliers`.
- 2026-11-25 (second PC, `scancraft-segment`) — Built `crates/scancraft-segment` (SAM 2.1 video
  segmentation on `ort` 2.0.0-rc.15 `load-dynamic` + `directml` + `api-24`, no CUDA/TensorRT) and
  example `segment_seq`: SHA-256 verification for all 4 `jax-image-tools/sam21-tiny-video-onnx`
  graphs + `constants.json` + ORT 1.24.4 / DirectML 1.15.4 DLLs; DXGI adapter enumeration +
  DirectML-vs-CPU self-test; Rust memory bank (conditioning + 6 recent frames, 16 object pointers
  with 1D sine temporal PE), point/box prompts + mid-sequence re-prompting; mask resampling,
  stability/IoU lost-object detection; `ColorCamera` + depth-buffer occlusion mask→3D projection +
  `MultiViewVoter`; `GeometricSegmenter` fallback. Verified: 15 model-free unit tests +
  `#[ignore]` live test (`sam2_live`, `SCANCRAFT_SAM2_DIR`). Measured on AMD Radeon 890M (iGPU,
  1024 px fp32, 6-frame 640×480 clip): self-test IoU 1.0000 vs CPU; seed 476.4 ms, tracked mean
  875.1 ms (vision_encoder 405.3 ms, memory_attention 397.5 ms, mask_decoder 23.4 ms,
  memory_encoder 24.4 ms) vs CPU 8 threads 1807.3 ms (~2.1× speedup).
- 2026-11-25 (second PC, `feature/scancraft-segment`, full app integration) — Wired `scancraft-segment`
  into `apps/scancraft`, `scancraft-color`, `scancraft-driver`, and `scancraft-pipeline`:
  `RgbKeyframe::new` encodes raw RGB frames to JPEG when needed; `MockDevice` emits calibrated RGB
  frames when `color_texture` is set; `PassPipeline::take_new_rgb_keyframes` streams keyframes live;
  `ScanCraftApp` enables `color_texture` by default on RGB-capable scanners and persists
  `pass-<id>/rgb/keyframes.json` + JPEGs; `ai_pack.rs` + consent modal handle opt-in download/folder
  install/removal and DirectML licence/telemetry notices; `Tool::AiSegment` provides split-view
  interactive prompting, SAM 2.1 propagation, multi-view depth-buffered 3D labeling, and Clean
  apply/undo; `worker.rs` + `camera_inset` add `Colour` view and experimental non-blocking live AI
  masking. Rebased on `main` (`2de56c0`); verified with unit tests across all touched crates,
  `#[ignore]` live SAM 2.1 app workflow test on Radeon 890M, and `--demo-project`/`--auto` GUI
  screenshot checks.
- 2026-11-25 (second PC, branch `agent/data-fidelity`) — Saving and exports: point PLY with
  normals and observation counts (`quality`), read back; project keeps every alignment quality
  field and the per-pass object labels (`pass-<id>/object.bin`), still format version 1 (all
  additions optional); imported passes named after the file stem or, for generic export names,
  their folder; Add Passes from another open tab (tab context menu); history entries in Title
  Case. merge-and-ai.md "Known gaps" updated.
