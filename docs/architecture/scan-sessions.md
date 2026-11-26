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
- **Raw recordings** (`SCANCRAFT_RECORD_DIR`) are kept per pass so a pass can be re-processed with improved algorithms.

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

Single-shot capture (frames taken on click rather than continuously) is a capture option of any
pass, useful for step-wise turntable scans; the frames are tracked like any other.

## 8. Persistence

Project folder (form A): `<name>.scancraft`, the project file (JSON: passes, modes, alignments, settings, output metadata; relative paths only; opened by double-click) + per pass `pass-<n>/keyframes.bin` (packed points), `surface.ply` (imported), `cleaned.ply` + `cleaned_observations.bin`, `object.bin`, `rgb/` (colour keyframes), `raw.bin` (optional recording), `markers.json` (optional marker map: the tracker's, or Revo's `marker_framework.mkf` read with an imported scan) + `outputs/`. Opening a project restores everything editable. One project per folder. Older folders with `project.json` keep opening; saving one writes the `.scancraft` file and turns `project.json` into a stub that older versions refuse ("saved by a newer version") instead of opening a stale copy. Every file is saved atomically (temporary file, then rename), the project file last.

Bundle (form B): `<name>.scancraftz`, a plain ZIP64 of the same layout, entries stored, the project file first and a manifest (`scancraft-bundle.json`: contents, size and SHA-256 per entry) last. Which items go in is a setting (`scancraft_io::bundle::BundleContents`, table in [open-issues.md](../open-issues.md) 4.1). A bundle is opened into a working folder (`scancraft_io::workdir`, a temporary form A under a configurable root) and saved back with unchanged entries copied raw from the old bundle. Lock files (`.scancraft.lock` in a project folder, `<bundle>.lock` beside a bundle) keep two apps from saving over each other. Code: `crates/scancraft-io/src/{project,bundle,workdir,lock,fsutil}.rs`.

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
