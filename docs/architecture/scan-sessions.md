# Scan sessions: projects, passes, modes and alignment

Status: adopted 2026-11-01 after review. This is the model the scan-mode work builds against.

## 1. Terms

| Term | Meaning |
|---|---|
| **Project** | One object (or scene) being digitised. Owns a single *project frame* (coordinate system) and all passes, alignments and outputs. Saved to disk as a folder. |
| **Pass** | One continuous capture session: a single scan mode, one tracker, one coordinate frame of its own. Can be paused and resumed (same mode, same tracker). |
| **Scan mode** | Light source × tracking mode × object preset (+ options such as colour), see `docs/protocol/modes.md`. **Fixed for the lifetime of a pass.** |
| **Keyframe** | A stored, posed frame of a pass: depth/points, optional IR, optional RGB image, optional marker observations. Can be discarded; the pass model is rebuilt from the kept keyframes. |
| **Alignment** | The rigid transform from a pass's frame into the project frame, with how it was obtained (reference, auto, manual, markers, same-position start). |
| **Output** | Something computed from aligned passes: merged point cloud, mesh, vertex colours, texture, Gaussian splat. Re-computable at any time. |

## 2. Rules

1. **One pass = one scan mode.** Changing light source, tracking mode, object preset or colour on/off ends the pass. Reasons: the device needs ~2 s to switch stream formats; modes differ in resolution, noise, calibration set (laser modes use the *blue* calibration, the others *red*), and tracking method (laser lines cannot track by geometry on the POP 4). Revo Scan also fixes the mode per scan.
2. **Exposure, gain, depth window** may change inside a pass (they don't change what the data is).
   **Pause / continue** keeps the pass (same mode, same tracker); continuing after Stop is allowed as
   long as the object has not moved relative to whatever the tracker locks onto.
3. **A new pass starts in one of two ways:**
   - **Object moved** (flipped, repositioned): the pass gets its own frame; it is *unaligned* until the user aligns it.
   - **Same position** (object untouched): the new pass starts already registered to the project
     frame. This is how a user "switches mode" for a detail without an alignment step. How it
     registers depends on the new pass's tracking mode:
     - geometry (Feature) with dense depth: re-locate against the project model;
     - Marker / Global Marker: match observed markers against the project's marker map;
     - laser lines are too sparse for geometric re-location and need markers;
     - if registration does not succeed within a few seconds the pass falls back to *unaligned* and
       is aligned afterwards like any other.
4. **Alignment happens between passes, never inside one.** Inside a pass, tracking (+ keyframe discard) is responsible for consistency.
5. **Outputs never destroy inputs.** Merge, mesh, texture and splat are derived; passes, keyframes and alignments stay editable.

## 3. Alignment workflow (Align stage)

- The first pass is the **reference** (identity in the project frame); the user can choose another.
- Per unaligned pass, the user picks a method:
  - **Auto**: global registration against the project model (FPFH-style features + RANSAC, then point-to-plane ICP), with an overlap/visibility score shown before accepting.
  - **Point pairs**: user clicks ≥3 corresponding points in a split view, then ICP refines.
  - **Markers**: if both passes observed the same marker dots *that are rigidly attached to the
    object*, solve directly from marker correspondences (exact, no geometry needed). Markers on
    blocks or the table around the object only help tracking within a pass (and same-position
    passes); once the object is flipped they no longer move with it, so the alignment must ignore
    them (marker sets are tagged per pass as "on object" / "environment", defaulting to environment).
- **Global refinement** after all passes are aligned: pairwise ICP between overlapping passes, then a pose-graph optimisation so the errors are spread instead of accumulated.
- **Merge** fuses kept keyframes of all aligned passes into one model (same surfel fusion as
  tracking, with a confidence filter, see section 6). Passes carry a weight by expected accuracy
  (laser lines > Full-field HD > VCSEL) so a precise pass wins where passes overlap; the user can
  exclude a pass or a region of it (lasso in the viewport) from the merge.

## 4. Colour, texture and Gaussian splatting

Colour is a **pass option** ("record colour"), not a scan mode; Gaussian splatting is a **project output**.

- A colour pass stores RGB keyframes (JPEG as received) with their camera pose (depth pose × RGB
  extrinsics `LC_RT.bin`, intrinsics `Prgb.bin`, distortion `Distort.bin`) and capture time.
  The RGB camera is a separate stream with its own timing, so an RGB frame's pose is interpolated
  between the depth poses around its timestamp; establishing a common clock between the RGB and
  depth streams is part of the colour work. RGB keyframes are selected for sharpness (motion blur
  ruins textures and splats).
- **Geometry** of the outputs comes from all aligned passes.
- **Appearance** (vertex colours, texture atlas, splat colours) comes only from RGB keyframes of aligned colour passes. Surfaces that no colour pass saw stay untextured / grey; the UI shows colour coverage so the user can add a colour pass where needed.
- So geometry-only passes (e.g. blue laser lines on a shiny part) and colour passes combine naturally once aligned. The constraint is coverage, not mode compatibility.
- **Gaussian splat** is trained from the RGB keyframes + poses of colour passes, initialised from the merged point cloud (which may include geometry-only passes). Areas without RGB coverage are excluded from the splat.
- Colour *tracking* (Revo "Color" tracking mode) is separate: it uses RGB features for pose and implies colour recording.

## 5. Software structure

```
driver (per vendor)  ─▶  SensorFrame { depth?, ir_left?, ir_right?, rgb?, imu?, timestamps, calib refs }
                              │
depth source (per light)      ├─ OnDevice (Full-field HD)        depth already in the frame
                              ├─ StereoMatch (VCSEL, Hybrid)     IR pair → depth
                              └─ LaserLine (Cross, Single)       IR pair → sparse 3D line points
                              ▼
tracking (per tracking mode)  Geometry (ICP+IMU) | Marker | GlobalMarker | Color(RGB-D)  → pose
                              ▼
pass                          keyframes (+rgb, +markers), tracker, model
project                       passes, alignments, outputs
```

- Each light source is a **depth source** module; each tracking mode a **tracker** behind one trait. A pass instantiates the pair its mode requires. New vendors add drivers; new modes add a depth source or tracker without touching the others.
- Driver work per mode: device setup sequence, stream format, decoding into `SensorFrame`. Host work: the depth source / tracker.
- **Raw recordings** are kept per pass (`pass-<id>/raw.bin`, section 8) so a pass can be re-processed with improved algorithms; `SCANCRAFT_RECORD_DIR` still records elsewhere when the project's recording is off.

## 6. Noise (known issue)

Isolated noise points survive fusion because one observation creates a surfel. Planned:
- drop flying pixels at depth discontinuities already at back-projection (edge test as in the
  visibility check) — most of the noise comes from object silhouettes;
- surfel confidence from observation count and viewing-angle consistency, used as a *weight*
  rather than a hard cut: a surfel seen once is kept if its neighbours agree (surface support) and
  dropped if it is isolated, so legitimate edge-of-coverage surface survives;
- free-space carving: a surfel that later frames see *through* (as in the visibility check) is
  removed;
- a statistical outlier pass on merge.

Implemented (work package I): `DepthMap::to_point_cloud_with(BackProjection { edge: Some(EdgeFilter) })`
drops pixels with a depth step > 1.5 mm + 0.4 % of depth per pixel within 2 px, and pixels next to
holes (used by `OnDeviceDepth` and `Tracker::register_depth`). The fused model
(`scancraft_slam::SurfaceModel`) keeps per surfel `Σ|cos view angle|` and the normal agreement;
confidence = their product. A surfel is output if confidence >= 2.5 or the summed confidence of its
3 x 3 x 3 coarse cells (4 point spacings wide) is >= 24. Each keyframe stores a 1/4-resolution
conservative free-space view (`FreeSpaceView`, 40 KB) and carves both the fused and the registration
model; rebuilds replay the same carving. Outputs can now shrink: `Tracker::model_revision()` /
`take_update()` (Append | Replace) / `SurfaceModel::withdrawn_pending()` tell consumers when to
refresh. Settings: `TrackerConfig::noise` (`NoiseConfig::off()` restores the old behaviour).
Measured on the handheld recording `pop4_1793532555.bin` (first 175 frames): isolated points
(< 5 neighbours within 2 mm) 9675 -> 1152, lost frames 19 -> 5; over the full 455 frames tracking no
longer gets stuck in "lost" after frame ~170 (288 -> 5 lost frames).

## 7. Turntable capture

The available turntable is manual (rotates continuously, direction switch, no control link), so
there is nothing to drive; the mode is about processing. With a fixed scanner and a rotating
object, the static table and background dominate geometric tracking, which then reports "no
motion" and smears the object. Turntable capture therefore:
- fits the turntable / table plane in the first frames and removes it and everything outside a
  user-adjustable cylinder above it *before* tracking and fusion;
- tracks only the object (plus anything on the turntable, e.g. marker blocks placed on it);
- uses no IMU prior (the scanner does not move; the object does).
Revo's Bluetooth "Auto Turntable" (light source value 5) is out of scope until such hardware is
available.

Implemented as `scancraft_pipeline::TurntableFilter`, wrapped around any depth source by
`pipeline_for` when `ScanConfig::turntable_mode` is a turntable mode (default is now
`HandheldFreeform`). It pools the first 3 frames, fits the plane by RANSAC (normal with >= 0.3
along sensor +Z, i.e. the scanner looks down at less than ~70 degrees), raises it to a turntable disc
if one stands on the table, picks the nearest sizeable above-plane cluster within 15 degrees of the
optical axis as the object, and keeps only points inside a cylinder (radius = 1.5 x object extent +
25 mm, height = object top + 25 mm; both overridable in `TurntableConfig`). The depth image is passed
on unfiltered (free-space evidence for visibility check and carving). `DepthSource::sensor_static()`
makes `PassPipeline` withhold the IMU from the tracker. Verified on synthetic data only so far;
a real turntable recording is needed (`replay_track` with `TURNTABLE=1`).

**Simulator check (2026-11-28).** The MockDevice has a turntable scene for `TurntableContinuous`
/ `TurntableStep` passes: the scanner fixed at 330 mm and 30 degrees elevation, the turntable
(105 mm disc, vase, canister, block, 16 marker stickers on the rim) turning under it, a static
table 8 mm below the disc top and a static wall behind; its IMU reports a scanner standing still.
Results (`crates/scancraft-pipeline/tests/mock_turntable.rs`, 2 degrees per frame, a full turn):
Feature tracking 179/179 frames tracked, pose error max 0.51 deg / 2.2 mm, vase surface median
0.06 mm / 95 % 0.21 mm from the true one; Marker tracking (markers on the turntable) 0.49 deg /
2.1 mm, 0.15 / 0.39 mm; no point of the static table or the disc top fused. Without the turntable
mode the static table and wall hold the pose still (0 of 30 degrees). Through the app (`app.rs`
test `turntable_passes_…`, 2.5 degrees per frame at 7 fps): both modes track, the capture record
stores the mode, the filter settings and the region, replays reproduce the live poses, and Align
All places both turntable passes onto a handheld pass of the scene within 0.4 mm / 0.06 deg.
Two things found: the plane RANSAC with 2 mm inliers chose a plane tilted ~2 degrees through
disc top and table (now 1 mm); and a depth window whose near limit cuts the turning object is an
edge fixed to the sensor that biases tracking to "no motion" (3.7 % too little turn with the POP 4
Full-field window 250-500 mm and the object at 230-260 mm): keep the whole object inside the
window.

Not known for a real turntable: the plane fit needs the table normal to have >= 0.3 along the
sensor's up axis (a view more than ~70 degrees down fails; `TurntableConfig::min_up`); the object
is the nearest sizeable cluster within 15 degrees of the optical axis (aim at it at the start);
markers belong on the turntable (they turn with the part), markers on the static table would track
the table instead; real depth noise, a turntable whose top is not flat, and hands in view while
turning a manual turntable are untested.

Single-shot capture (frames taken on click rather than continuously) is a capture option of any
pass, useful for step-wise turntable scans; the frames are tracked like any other.

## 8. Persistence

Project folder (form A): `<name>.scancraft`, the project file (JSON: passes, modes, alignments, settings, output metadata; relative paths only; opened by double-click) + per pass `pass-<n>/keyframes.bin` (packed points), `surface.ply` (imported), `cleaned.ply` + `cleaned_observations.bin`, `object.bin`, `rgb/` (colour keyframes), `raw.bin` / `raw_rgb.bin` (raw streams), `capture.json`, `tracking.csv`, `calibration/` (see below), `markers.json` (optional marker map: the tracker's, or Revo's `marker_framework.mkf` read with an imported scan) + `outputs/`. Opening a project restores everything editable. One project per folder. Older folders with `project.json` keep opening; saving one writes the `.scancraft` file and turns `project.json` into a stub that older versions refuse ("saved by a newer version") instead of opening a stale copy. Every file is saved atomically (temporary file, then rename), the project file last.

**What a captured pass records about itself** (2026-11-28, `scancraft_io::capture`,
`apps/scancraft/src/capture.rs`), so it can be decoded, tracked, fused and compared again offline:

```text
pass-<id>/raw.bin          main wire stream (POP 4: the media packets as received, framing
                           u32 0x11223344, u32 length, packet; simulator: its own records)
pass-<id>/raw_rgb.bin      colour wire stream (POP 4: JPEG + u32 time stamp per packet)
pass-<id>/capture.json     capture record: app (version, git commit, dirty, profile, target),
                           device (vendor, model, firmware, transport kind, resolutions, IMU
                           mounting, serial only as sha256 hash, no IP), capabilities, scan
                           settings, app settings (depth window, point pitch, budget), tracker
                           settings, turntable (mode, filter settings, region found), calibration
                           file list, raw file names, log file name, and one entry per capture
                           session: start / end (UTC), continued / seeded / live AI, byte offsets
                           into the raw files, the session's scan settings, stream layout and the
                           control commands sent, recording statistics (bytes, dropped records,
                           most queued, errors), frame counts (received, skipped for newer,
                           paused, processed, without geometry, tracking, slow, lost, relocalised,
                           keyframes, fused points), events (exposure / gain changes, pause,
                           recording warnings)
pass-<id>/tracking.csv     one row per frame the live tracker processed: session, frame, device
                           ms, host ms, state, relocalised, inliers, RMS, visibility conflict,
                           keyframe, new / total points, pose (quaternion, translation), IMU
                           quaternion, processing ms
pass-<id>/calibration/     the calibration files the driver downloaded (POP 4: camparam.yaml,
                           Prgb.bin, Distort.bin, LC_RT.bin, camparam_blue/*; serial blanked)
```

The raw files are recorded while scanning (writer thread, queue bounded at 192 MB; a full queue
drops records and counts them, it never blocks the stream: HANDOFF §3). They go into the project
folder, a bundle's working folder, or for a project not saved yet a staging folder under the
working-folder root that the first save moves into the project. They belong to the bundle item
"Raw stream recordings" (off by default in bundles; the Save card warns when a project has
recordings a bundle would leave out). `capture.json`, `tracking.csv` and `calibration/` are core
files (always saved and bundled). The project file gains `notes` and, in `settings`, the
processing parameters, the processing history (the steps of each session, oldest first) and the
build that saved it. Every addition is optional: older projects open as before, and the previous
build opens these projects (it keeps `notes` as an unknown field and ignores the new files; it
does not copy them when saving elsewhere). Replay: HANDOFF §4 "Replaying a pass".

Bundle (form B): `<name>.scancraftz`, a plain ZIP64 of the same layout, entries stored, the project file first and a manifest (`scancraft-bundle.json`: contents, size and SHA-256 per entry) last. Which items go in is a setting (`scancraft_io::bundle::BundleContents`, table in [open-issues.md](../open-issues.md) 4.1). A bundle is opened into a working folder (`scancraft_io::workdir`, a temporary form A under a configurable root) and saved back with unchanged entries copied raw from the old bundle. Lock files (`.scancraft.lock` in a project folder, `<bundle>.lock` beside a bundle) keep two apps from saving over each other. Code: `crates/scancraft-io/src/{project,bundle,workdir,lock,fsutil}.rs`.

In the app (`apps/scancraft/src/storage.rs`, `app_project.rs`): each tab's `ProjectInfo::store` holds its folder lock or its bundle's working folder. Open Project… takes a `.scancraft` file, a bundle or an older `project.json`; the Save Project card saves as a project folder or a bundle, and saving somewhere else moves the tab there (Save As Project Folder, Save as Bundle…). Closing a tab (and quitting) releases the lock and removes a bundle's working folder; at start, working folders a crash left with unsaved changes are offered for recovery (Open / Later / Discard). Bundle contents and the working-folder root are chosen in the Save card and kept in `%LOCALAPPDATA%\ScanCraft\settings.json`. `scancraft.exe <path>` opens a project; a second start hands its path to the running app (`single_instance.rs`), which opens it as a tab; the second start's other options (`--mock`, `--connect`, …) are not applied there but named in the running app's status bar. The installer registers `.scancraft` and `.scancraftz` for the user (double-click).

Sharing a project between computers (a network folder): a project whose lock names another computer is asked about on open, *Take Over* (only from the holder shown; never from a running ScanCraft on this PC) or *Open Read-Only* (a copy with no place on disk; a bundle is read through an unpacked copy, `workdir::UnpackedCopy`, kept while the tab is open so that Save to a new place brings its raw recordings and unknown fields along). A save over a project file or bundle that someone else saved since it was opened here asks first: *Save Anyway*, *Save As…* (both kept) or Cancel (the folder tab keeps its project file's size and time stamp for this, `project::check_unchanged`). Add Project takes bundles too (through the same temporary copy), and the app bar's Open / Add buttons have *Open Project Folder…* / *Add Project Folder…* on right-click, since the Windows dialog picks files or folders but not both. Fields of the project file this version does not know (a later version's additions) are kept on save (`ProjectFile::other`, `PassRecord::other`). A file a project names (keyframes, surfaces, outputs) must be a plain relative path inside its folder (`project::resolve`): a project or bundle from someone else never makes the app read files elsewhere or on a network share. A free name proposed for Save As… that someone takes before Save is never saved over: the next free one is proposed. Details and tests: [open-issues.md](../open-issues.md) 4.1, status 2026-11-27.

## 9. Implementation plan (parallel work)

Each item is owned by one agent in its own git worktree/branch. Shared interfaces (`SensorFrame`, depth-source and tracker traits, pass/project model) land first so agents build against them.

| Work package | Depends on | Device access |
|---|---|---|
| A. Session core: `SensorFrame`, traits, project/pass model, persistence, same-position passes | — | none |
| B. VCSEL Rapid: IR-pair stream, rectification, host stereo (GPU SGM) | A | recordings + short live tests |
| C. Hybrid HD: decode fmt 5 / type 10, stereo | A, B (stereo) | live format experiments |
| D. Laser lines (Cross, Single): 1280×1024 IR pair, blue calibration, line extraction, matching, triangulation | A; E for posed (moving) data | recordings |
| E. Marker + Global Marker tracking: IR LED, display type 6, dot detection, constellation tracking, marker map (stickers and marker blocks available) | A | recordings with markers |
| F. Colour: RGB stream, sync, extrinsics, vertex colours, texture; Color tracking | A | recordings |
| G. Gaussian splatting output | F | none (offline) |
| H. Multi-pass alignment (auto, point pairs, markers, global refinement) + merge + noise filter | A (E for markers) | none |
| I. Turntable capture (background removal, object-only tracking) + noise reduction (section 6) | A | recordings on the turntable |

Only one process may stream from the scanner at a time: live experiments go through a lock file (`%TEMP%\scancraft-scanner.lock`) and each mode agent works mostly from raw recordings made once per mode.
