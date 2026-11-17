# POP 4 scan modes, tracking modes and presets (Revo Scan 6)

Catalogue of every user-selectable option in Revo Scan 6 that changes scanning on the POP 4, what each one does on the device and on the host, and what ScanCraft needs to implement it. Written 2026-11-01 from static analysis plus Revo Scan's own logs. No commands were sent to the scanner.

Labels: **[C]** = confirmed (direct string, decoded config, disassembly or logged command). **[I]** = inference (stated reasoning, not verified).

## 0. Sources and how to reproduce

| ID | Source | What it gives |
|---|---|---|
| S1 | `E:\Revopoint\Revo Scan 6\Resources\config\deviceScanParam.json` (359 KB, obfuscated) | Per-model matrix of tracking mode × light source × accuracy × object type, with exposure, gain, Z range and laser settings |
| S2 | `...\Resources\config\depthSearchRangeParam.json` (plain JSON) | Host-side stereo disparity search per light source. Entry `"model": "0E1", "comment": "POP4"` |
| S3 | `...\Resources\config\gainExposureMap_POP4.json`, `brightnessGainMap_POP4.json` (plain) | Gain index 1..10 → exposure (µs) / laser brightness per light-source family |
| S4 | `...\Resources\lang\RevoScan_en_US.qm` (1512 messages, Chinese source → English) | UI strings |
| S5 | `common.dll`, `revoConfig.dll`, `rvscan.dll`, `rvcore.dll`, `3DCamera.dll` | Enum names, string tables, `DeviceTypes::*` model checks |
| S6 | `%LOCALAPPDATA%\Revopoint\RevoScan6\logs\RevoScan6*.log`, `sdk.log`, `sdk.log_back` | Revo Scan sessions with this POP 4 (S/N `…00E1B`, fw `v5.3.94.20260808`), 2026-06-19 … 2026-10-12. App-side `scan_mode`/`scan_surface` changes plus every `zx_cmd.cgi` request, with timestamps |
| S7 | `%APPDATA%\Revopoint\RevoScan6\UserConfig\AppParamTemplate.json` | Saved scan-parameter template for model `0E1` (field names of the scan settings) |
| S8 | `firmware/zxmain` strings | Firmware stream-format names |

**Decoding S1 and `devicePostProcessParam.json` / `metroScanParam.json`:** these files are XORed with a repeating 72-byte key. Autocorrelation peaks at lag 72. To recover the key, assume the most common plaintext byte at each key position is a space (`0x20`), because the JSON is pretty-printed: `key[i] = most_common(data[i::72]) ^ 0x20`. After XOR the files parse as JSON. Sketch: `python -c "import collections,sys;d=open(f,'rb').read();k=bytes(collections.Counter(d[i::72]).most_common(1)[0][0]^32 for i in range(72));open(o,'wb').write(bytes(b^k[i%72] for i,b in enumerate(d)))"`. Do not commit the decoded files (they are proprietary).

**POP 4 identity.** The model code is `0E1` [C]. Evidence: S2 comment `"POP4"`, the serial `D00000000B0S00E1B` ends in `0E1B`, and `calibration/0E1/` plus `calibration/res/pop4/` exist. `DeviceTypes::getDeviceType("0E1")` returns `DeviceType = 0x70000` [C] (`common.dll` getDeviceType @ `0x180044645`). The SDK-internal camera type is `32` [C] (`sdk.log`: `surface POP4/POP4 Pro type[32]`).

## 1. Settings model in Revo Scan

A scan is defined by these independent parameters. All of them are logged as `ScanManager::setScanParam name=…` (S6), and the stored template is S7.

| Parameter (log / template key) | Revo enum | Values |
|---|---|---|
| `scan_mode` / `scanMode` | `ScanMode` ("Tracking Mode", 拼接模式) | §3 |
| `scan_surface` / `surfaceType` | `ScanSurfaceType` (light source, "Scan Mode", 扫描模式) | §2 |
| `scan_accuracy` / `scanAccuracy` | accuracy ("Accuracy", 扫描精度) | §4 |
| `scan_object` / `objectType` | `ScanObject` ("Object Type", 扫描对象) | §5 |
| `scan_texture`, `enableColorMode` | bool ("Color Scanning", 彩色扫描) | §6 |
| `marker_regist_mode`, `rotate_state`, `fg_segment`, `bg_segment`, `clip_plane`, `enableErasePlane`, `previewPointPitch`, `zmin`/`zmax`, `gain`, `laserLuminance`, `targetQuality`, RGB exposure / white balance, `enableDepthFlashing`, `enableRGBFlashing` | — | §7 |

Each (tracking mode, light source, accuracy, object) combination selects one row of S1. That row gives the exposure, gain, Z range, distance-bar bands (`histogram1..4`), depth threshold, `multiFrameFusion`, laser luminance and target quality, which are then pushed to the device (§8).

## 2. Light sources (`ScanSurfaceType`)

| Value | Internal key | English UI (S4) | POP 4? | Evidence for the value |
|---|---|---|---|---|
| 0 | `surface` | **Full-field HD** (全场扫描). Button labels: "Full-field HD—High Accuracy" / "FFHD High" | **yes** | [C] `getGainExposureMapCategory` maps 0 to `"surface"`. Log `surface type: 0` with `scanDistance 250..500` = S1 `feature/surface` |
| 1 | `single_line` | **Single Line** (一字线扫描), "Single-Line Deep Hole Scan" | **yes**, marker / global marker only | [C] `isLineLaserSurface` returns true for 1..3 (`common.dll` `0x180045770`: `lea eax,[rcx-1]; cmp eax,2; setbe`). The 1 = single assignment comes from string-table order [I] |
| 2 | `crosswire_line` | **Cross Lines** (交叉线扫描), "30-Cross Blue Laser Lines Scan" | **yes**, marker / global marker / color | [C] log `scan_surface=2` → `echo s 0x03001 7` (cross), `cameramultiparam\blue`, `scanDistance 200..350` = S1 `marker/crosswire_line` |
| 3 | `parallel_line` | **Parallel Lines** (平行线扫描), "invisible parallel infrared laser lines" | **no** | [C] no `parallel_line` key for `0E1` in S1. [C] `3DCamera.dll` `laserScanLineModeStart` (`0x1812c2dd0`) errors out for camera type 32 on line-mode argument 2 |
| 4 | `speckle` | **VCSEL Rapid** (散斑扫描). Labels: "VCSEL High", "VCSEL—High Accuracy" | **yes** | [C] `DepthSearchRangeManager::surfaceKey` (`revoConfig.dll` `0x180065cc0`): 4 → `"speckle"`. Log `SCAN_SURFACE_TYPE_SPECKLE` with `scan_surface=4` |
| 5 | `turntable` | **Auto Turntable** (转台自动扫描) | probably | [I] Only value left over, and it is the only remaining name in the 7-entry string tables (`common.dll` `0xbd170..0xbd1a0`; `rvscan.dll` label list `0x2679xx`). `getGainExposureMapCategory` maps 5 to `"surface"`, which fits a turntable mode that uses the full-field projector |
| 6 | `speckle_stripe` | **Hybrid HD** (混合HD扫描). Labels: "Hybrid High", "Hybrid HD—High Accuracy". Help text: "Powered by coordinated dual projectors…" | **yes** | [C] `surfaceKey`: 6 → `"speckle_stripe"`. `getGainExposureMapCategory`: 6 → `"Hybrid"`. Log `scan_surface=6` → `set_display_type=10`, `set_depth_output_fmt=5` |

UI help text (S4): Full-field HD "projects fine structured light patterns" (Chinese source: a *micro-structured blue coded pattern*; markerless for geometric objects, markers for featureless ones). VCSEL Rapid "projects high-density dot patterns… markerless scanning in feature-tracking mode". Cross lines "flawlessly handles high-reflectivity metal surfaces and dark objects" in marker tracking. Single line "deep holes, grooves" in marker tracking. Revo also warns "Do not install the outdoor blue filter" for VCSEL and "Recommended to install the outdoor blue filter" outdoors (`needShowSpeckleOutdoorFilterTips` is true for POP 4 [C]).

### 2.1 What each light source does on the device and on the host

All device commands below were logged by Revo Scan for this POP 4 (S6, `sdk.log_back`, timestamps given). Register meanings are labelled.

| | Full-field HD (0) | VCSEL Rapid (4) | Hybrid HD (6) | Cross lines (2) | Single line (1) |
|---|---|---|---|---|---|
| Log example | 2026-10-12 09:24:27 (feature), 2026-07-05 18:42:25 (marker) | 2026-07-05 18:44:41 | 2026-06-19 14:55:12 | 2026-07-05 18:38:08 | not captured |
| `set_display_type` | 4; 6 in marker mode | 2 | 10 | 2 | [I] 2 |
| `set_depth_output_fmt` | 3 | 1 | 5 | 1 | [I] 1 |
| Resolution | 640×512 | 640×512 | 640×512 | 1280×1024 | [I] 1280×1024 |
| Plane layout | `z16y8y8` (4 B/px); `z16y8y8p` (6 B/px) in marker mode [I, byte count vs. firmware names S8] | `y8y8` IR pair | `s64y8y8` (10 B/px) [I, 8+1+1 B matches type 10 and fw `DSP_OUTPUT_FMT_S64Y8Y8`] | `y8y8` IR pair | IR pair |
| Media URL | `zx_media.cgi?camera_id=21` | `camera_id=33&type_id=20` | `camera_id=33&type_id=21` | `camera_id=33&type_id=20` | — |
| SDK format / fps | 3 (265 marker) / fps 5 | 266 | 267 / fps 15 | 266 (4 in older fw) / 15 | — |
| Projector cmd | none logged. `0xa05 3` follows `SCANNER_PARAM_TYPE_FRINGE_PATTERN=3` (2026-06-20 08:27:56) | `0x3001 12` [C] | `0xb1b 65` (speckle-stripe luma; `getSpeckleStripeLaserLuminance(POP4)` = 65/70 [C]). `0x48d 80`. `0x3001 14/15` exist in `speckleLaserScanLineModeStart` [I] | `0x3001 7` [C], `0xb04 0/1` toggled (laser gate) | `0x3001 9` [I: `laserScanLineModeStart` arg 1 for type 32 → `9`] |
| Exposure / frame | `0x911 3000…6000` (surface table, S3), `0x910 7000` | `0x911 500…2000`, `0x910 50000` (log: `frameTimeus(50000)`) | `0x911 3000…`, `0x910 7000`, `0xb1c 18000` | `0x911 300`, `0x910 11000`, `0xb08 ~950` (laser exposure, auto-adjusted during scan), `0x48d 255` | S1: 500 µs, `laserLuminance 5..20` |
| Z range | `0x707 250 500` (S1 zmin/zmax) | S1 300..800 mm (range 300..1000) | 250..500 | `0x707 200 600` at stream start; S1 200..350 | 200..350 |
| Depth computed by | **device** (z16 plane) | **host**: stereo match on the IR pair (`RPASpeckleStereo.dll`; S2 `speckle` dispSize 288 CPU / 352 GPU, minDisp 0) | **host** stereo (S2 `speckle_stripe` 256/100 CPU, 352/0 GPU) on a format we have not decoded | **host**: line extraction (Steger, `FExtractStegerShader`), L/R line matching (`RPALaserMatch`, `RPALineDetect`), triangulation (`RPALaserRecon`) using `cameramultiparam\blue` (`metroExtra.bin`, `Q.bin`) | host, as cross |
| Calibration set | `red` (IR/speckle) | `red` | `red` | `blue` | `blue` |

Notes:
- `0x3001` line-mode mapping [C] from `3DCamera.dll` `laserScanLineModeStart` @ `0x1812c2dd0`: argument 0 → `0x3001 7` for all models. For type 32, argument 1 → `0x3001 9` and argument 2 → error. `0x3001 8`/`11` are used by other models (bit masks on the type at `[this+0xbc]`). `laserScanSpeckleStart` sends `0x3001 12`, or `13` only for type `0x28`.
- `0x911` = IR exposure in µs [C]: logged values equal S3 entries (e.g. 4600/5100/5400 for gain 2/3/4 in surface; 833 for speckle gain 3). `0x903` changes together with gain (0x10…0x50) [I: analog gain code]. `0x705` = S1 `threshold` (5 or 10) [I, value match]. `0xa04 0` matches `multiFrameFusion 0` [I]. `0x488 1` precedes each exposure/gain change during a scan [I: AE apply].
- `usesNewPlatformHalfStreamResolution` (`common.dll` `0x180045e40`) [C]: half resolution for surface types 0, 4 and 6; full resolution for 1–3 and 5.

## 3. Tracking modes (`ScanMode`)

| Value | Key | UI (S4) | Help text (S4) | Evidence |
|---|---|---|---|---|
| 0 | `feature` | **Feature** (特征拼接) | "automatically stitches… based on their geometric features… not for… revolving bodies or flat objects" | [C] log `SCAN_MODE_FEATURE = 0` (`rvscan.dll` `0x25f448`) |
| 1 | `marker` | **Marker** (标记点拼接) | "…based on markers placed on or around the object… mechanical parts, planes, cubes, cylinders" | [C] `isLineLaserMarkerOrGlobalMarker` checks mode 1..2. Log `scan_mode=1` with lines |
| 2 | `globalMarker` | **Global Marker** (全局标记点拼接) | "large objects… two-step: 1) scan markers to generate global marker file 2) scan point cloud based on the file". Has a Global Markers Library with import/export | [C] `rvscan.dll` `0x25f3f0` `SCAN_MODE_MARKER_GENRAGE = 2`. Log `loadGlobalMarkerScanSceneFromTemplate … scanMode: 2` |
| 3 | `colortexture` | **Color** (彩色拼接) | "objects with distinct patterns, colors, or textures. Alignment… relies on image texture features" | [C] `isLineLaserColorTexture` checks `mode == 3`. Log `scan_mode=3` starts the RGB stream |

Valid light sources per tracking mode for `0E1` [C, keys present in S1]:

| Tracking ↓ / light → | Full-field HD | VCSEL Rapid | Hybrid HD | Cross lines | Single line |
|---|---|---|---|---|---|
| Feature | ✔ | ✔ | ✔ | — | — |
| Marker | ✔ (object `general` only) | — | — | ✔ | ✔ |
| Global Marker | ✔ (`general` only) | — | — | ✔ | ✔ |
| Color | ✔ | ✔ | ✔ | ✔ | — |

So the lasers are never used with geometry-only tracking on POP 4. They need markers, or RGB texture in Color mode.

Host-side work per tracking mode:
- **Feature**: frame-to-model geometric registration. Revo DLLs: `RPARegisterCoarse/Fine/GO`, `RPAFusionGeoFeature`, `RPAFeature3D`. Relocalisation uses a visual vocabulary (`Resources/algorithm/xfeat_dynamic_256.bin`, `vocab_tree…` per the earlier analysis) [I]. "Intelligent Tracking" (智能跟踪模式) is a toggle [C string].
- **Marker**: detect retro-reflective dots in the IR images and track the dot constellation. DLLs: `RPAMarkerDetect`, `RPANNMarkerDetect` (a neural-network detector), `markerDetectionByJH`, `markerMatchByEpipolar`, `markerAlignByTriangle`, `markerFusionByTriangle`, `RPAMarkerRecon`. Device side [C, 2026-07-05 18:42:35 and 18:38:15]: IR LED flash `0xb07 <luma>` (S1 `fillLightLuminanceMin/Max` 66..82 for surface; logged 30–60 with lines), `0xb13`, `0xb01 1`, `0xb05 0`. With Full-field HD, the display type switches from 4 to 6 (extra plane) [C]. UI options: marker inner diameter, "Remove marker points", marker removal offset (mm).
- **Global Marker**: phase 1 scans markers only and saves them to the library (`*.revox`, `GlobalMarkers/`). Phase 2 scans the cloud against them. `GLOBAL_MARKER_PAIR_FRAME_TIME_US` and `isGlobalMarkerPairStream` (stream type 9) point to a dedicated paired-frame stream [I]. DLLs: `RPAGlobalFusion`, `RPARegisterLandmark`, `ceres.dll` (bundle adjustment) [I].
- **Color**: RGB-D registration. DLLs: `RPARegisterRGBD`, `RPAFusionRGBD`, `Resources/algorithm/rgbd_orb.bow` (ORB bag of words). Needs the RGB stream `zx_media.cgi?camera_id=50&type_id=20` (SDK format 1, 1280×1024, MJPEG [I from `mjpg` / `set_aeMaxExp` and the `DecoderMpeg` note in `overview.md`]). RGB setup logged: `cam_type=usb&set_resolution=1&width=1280&height=1024`, `set_aa_mode`, `set_whitebalance_mode`, `set_aeMaxExp`.

## 4. Accuracy (`scan_accuracy`)

| Value | Key | UI | POP 4 | Evidence |
|---|---|---|---|---|
| 0 | `fastmode` | **Standard Accuracy** (标准精度): "model processing speed is faster" | [I] UI likely; no `fastmode` rows for `0E1`, so the device settings probably fall back to `accuracymode`. Differs only in post-processing | string-table order `common.dll` `0xbd138`, UI order in `rvscan.dll` |
| 1 | `accuracymode` | **High Accuracy** (高精度) | ✔ all rows | [C] every logged session uses `scan_accuracy=1`, with `accuracymode` values (zmin/zmax) loaded |
| 2 | `highSpeedmode` | **High-speed** (高速模式): "low-precision high-speed… large objects and people" | only with marker/global-marker + cross/single line (S1). `isSupportSurfaceHighSpeed` is true only for `0x60000` → no "FFHD Fast" on POP 4 [C] | S1, `common.dll` `0x1800459a0` |

Point spacing ("Target Point Distance", 预设点距) is a separate setting. Range for `0E1`: 0.05–2.0 mm (`devicePostProcessParam.json`, decoded). Mesh edge length range: 0.1–8.0 mm. A small spacing triggers the warning "Small point distance requires substantial RAM".

## 5. Object type (`ScanObject`)

String table (`common.dll` `0xbd1a8…`, order [I] = enum): `general, face, head, body, dark, reflect, outdoor, large, body_large`, plus `_near`/`_far` variants used only by near/far models (`isSupportsNearFar` = `0x50000` only → not POP 4 [C]). `general` = 0 [C] (logged `scan_object=0` with general values).

UI labels (S4): Normal (普通物体), Dark/Black (深/黑色物体), Metallic Shiny Object (金属反光物体), Outdoor object (户外物体), Face (人脸), Hair/Head (头发/头部), Body (人体扫描), Body (18 fps), Large Object (大物体).

Object presets available for `0E1` [C, S1]. Values are for `accuracymode`; exposure in µs; Z in mm.

| Light | Object | Exposure | Gain | zmin–zmax (bar range) | Other |
|---|---|---|---|---|---|
| Full-field HD | general | 3000 | 1 | 250–500 (200–500) | threshold 10 |
| | dark | 5400 | 4 | 250–500 | threshold 5, **multiFrameFusion 1** |
| | face | 4600 | 2 | 250–500 | threshold 5 |
| VCSEL Rapid | general | 666 | 2 | 300–800 (300–1000) | specialLaserLum 100 |
| | dark | 1166 | 5 | 300–800 | |
| | face | 1000 | 4 | 300–800 | |
| | outdoor | 60 | 1 | 300–800 | specialLaserLum 255. `getSpeckleOutdoorExposure(POP4)=60` [C] |
| Hybrid HD | general / dark / face | 3000 / 5400 / 5100 | 1 / 4 / 3 | 250–500 | |
| Cross lines (marker) | general / dark / reflect / outdoor | 500 / 1000 / 500 / 100 | 1 / 3 / 1 / 1 | 200–350 (170–370) | laserLuminance 64 / 102 / 115 / 25, targetQuality 8–18 |
| Single line (marker) | general / dark / reflect / outdoor | 500 / 1000 / 500 / 100 | 1 / 2 / 1 / 1 | 200–350 | laserLuminance 5 / 15 / 20 / 5, targetQuality 60 |
| Full-field HD (marker) | general | 3000 | 1 | 250–500 | IR fill LED 66–82 |

`head`, `body`, `body_large` ("Body (18 fps)") and `large` have no `0E1` rows, so they are presumably hidden for POP 4 [I]. They exist for other models, e.g. `0A8` `highSpeedmode` body/large. "Face" is the only person preset on POP 4. `brightnessGainMap_POP4.json` gives laser-brightness tables `lineGeneral`, `lineDark`, `lineReflect` and `lineOutdoor` (gain 1..5), and `surface` (gain 1..10) [C].

## 6. Colour / texture

| Option | UI | Mechanism | POP 4 |
|---|---|---|---|
| Color Scanning (`scan_texture=1`, `enableColorMode`) | "Color Scanning" / Color / No Color | Starts the RGB stream (`camera_id=50&type_id=20`). Revo logs `ledAndLaserCtrl` with `0xb03 1`, `0xb02 0`, `0xb12 255` around it [C; `0xb12` = RGB white LED per `lasers_and_markers.md`, the rest is unknown]. Texture is baked by `rvtexture.dll`/`RPAMeshTexture` using `Prgb.bin`, `Distort.bin` and `LC_RT.bin` | ✔ (logged) |
| Color tracking (`scan_mode=3`) | Color | §3 | ✔ |
| Texture Capture (拍照贴图) / "Add Image" | extra RGB photos for texture and 3DGS | post-scan | ✔ |
| 3DGS (Gaussian splatting) | "requires Color Scan" | `rv3DGS.dll`, `revosplat.exe`. `isSupport3DGS` true for new-platform models including `0x70000` [C] | ✔ |
| Vertex coloring / Texture mapping | post-processing | — | ✔ |

## 7. Other options that change acquisition

| Option (UI) | Key | Notes |
|---|---|---|
| Handheld vs turntable ("Turntable Scan", "Auto Turntable", "Turntable Sync") | surface type 5 [I], BLE accessories (`btaccessories.dll`, `simpleble.dll`) | Dual-axis turntable (speed 28–88 or 35–90 s/rev, step angle, rotations, tilt) and Large Turntable. "Turntable Sync" links scanner start/pause to turntable rotation |
| Single Shot (拍照) vs Scan | — | "Click Scan or Single Shot to start scanning". Firmware has `id_msg_proj_create_one_trigger_t` (0x0485) [I link] |
| Scanning Distance (扫描采集距离) | `zmin`/`zmax` | → `0x707 zmin zmax` [C value match]. Distance bar from `histogram1..4` |
| Base Removal (消除平面) | `enableErasePlane` | host-side plane removal |
| Object Segmentation / background exclusion | `fg_segment`, `bg_segment`, `enableForegroundSegment`, `enableBackgroundSegment` | selection made in the RGB view, AI segmentation (`foregroundSegment.dll`, ONNX). `isSupportAISamElim` true for POP 4 [C]. "Line Laser Object Segmentation" masks laser data with RGB |
| Depth exposure / gain, laser brightness | `gain` (1..10, `gainSteps 10`), `laserLuminance` | gain index → exposure via S3. +/- and M buttons on the scanner (`isSupportsMButton` lists POP 4 [C]) |
| Depth / RGB "flashing" | `enableDepthFlashing`, `enableRGBFlashing` | unknown [I: projector/LED strobe] |
| Line laser type switch | `SCANNER_PARAM_TYPE_LASER_LINE_DIRECTION` (logged value 0), `laser_line_type` | `laserLineDirectionChange`. +/- buttons switch line type in marker mode |
| Compute acceleration CPU/GPU | `useGpuAcceleration` | selects S2 `cpu` or `gpu` disparity ranges |
| Near/Far camera | — | not on POP 4 |

## 8. Implementation assessment for ScanCraft

> **Superseded.** This assessment is from 2026-11-01. The current state of every mode is in
> [scan-modes.md](../architecture/scan-modes.md).

Current state: the driver sets `0x912 1, 0x3010 0x17 0, 0x3001 12, 0xb1b, 0xb04 1`, then fmt 3 / type 4 on `camera_id=21`. That gives device z16 depth at about 6 fps, tracked with point-to-plane ICP plus an IMU prior. The **stream configuration is identical to Revo's Full-field HD (surface 0)**: fmt 3, type 4, 640×512, `camera_id=21`, Revo fps 5. The projector commands come from Revo's VCSEL mode, though. Revo sends no `0x3001` in surface mode, but does send `0xa05 3` (fringe pattern), `0x707`, `0x705` and `0x903`/`0x910`/`0x911`. Open question: is ScanCraft's depth actually lit by the VCSEL or by the full-field projector? Verify by toggling `0x3001 12` off.

| Mode | Status / what is needed | Difficulty |
|---|---|---|
| Full-field HD + Feature, objects General/Dark/Face | **Works with what we have.** Add Revo's per-object presets (exposure, gain, Z range, threshold via `0x911`, `0x903`, `0x707`, `0x705`) and the `multiFrameFusion` = averaging for Dark. Align the projector setup with Revo (fringe `0xa05 3`, no `0x3001 12`) after a test | low |
| Accuracy Standard/High | host-only: point pitch 0.05–2.0 mm + post-processing | low |
| Color Scanning (texture) | **Implemented** (2026-11-01): JPEG stream + device-clock timestamp, paired with depth, calibration verified; see [color.md](color.md) | done |
| VCSEL Rapid + Feature (longer range 300–800 mm, faster) | needs fmt 1 / type 2 IR pair (already decoded for lasers), the `red` rectification maps, and **host stereo matching** (SGM/census, disparity 0..288/352 at 640×512) → depth. Then the same ICP | medium–high (GPU SGM for frame rate) |
| Marker (with Full-field HD) | needs display type 6 plane decoding (`z16y8y8p`, the 6th byte pair is unidentified), IR LED control `0xb07`/`0xb13`/`0xb01`, a dot detector (threshold + ellipse centroid), stereo/depth lift, and constellation matching (Kabsch). Fuse with ICP | medium |
| Cross / Single line + Marker | needs everything from Marker plus a Steger line extractor on 1280×1024 IR pairs, L/R line correspondence, triangulation with the `blue` calibration, and laser-exposure control (`0xb08`, `0x48d`, `0xb04`). Line data is sparse, so tracking must come from markers | high |
| Global Marker | Marker pipeline plus a two-phase workflow, marker map with bundle adjustment, and library storage | high |
| Color tracking | **Implemented** with Full-field HD (ORB-style features + 3D-3D RANSAC + PnP refinement); measured on synthetic flat textured data and static real data only, see [color.md](color.md) | done (needs a moving recording) |
| Hybrid HD | `s64y8y8` stream format (fmt 5 / type 10) undocumented; host stereo on it | unknown / high |
| High-speed | only meaningful with lines + markers on POP 4 | after lines |
| Turntable | BLE protocol not yet analysed; software-only "turntable" (static scanner, ICP) works today | medium (BLE RE) |
| Object segmentation, base removal | host-only (plane RANSAC is easy; AI segmentation is not) | low / high |

## 9. Recommended UI set

Expose now: **Light source:** Full-field HD. **Tracking:** Feature. **Object:** General / Dark / Face (Revo presets). **Accuracy:** Standard / High (point pitch). **Capture:** Handheld / Turntable-assisted (software only). **Toggles:** Base Removal.

Show as "coming later" (disabled, with reason): VCSEL Rapid (host stereo), Color Scanning (RGB decode), Marker, Cross Lines, Single Line ("requires Marker tracking"), Global Marker, Color tracking, Hybrid HD, High-speed. Do **not** offer Parallel Lines, Head, Body, Large Object, or near/far for POP 4.

### Proposed Rust enums

Discriminants mirror Revo's numeric values, so logs and templates map 1:1.

```rust
/// Revo `ScanSurfaceType`. Values confirmed except `Turntable` (inferred).
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum LightSource {
    FullFieldHd = 0,   // "surface": on-device depth, fmt 3 / type 4, 640x512
    SingleLine = 1,    // blue single laser line, 0x3001 9 (POP 4), marker modes only
    CrossLines = 2,    // blue cross lines, 0x3001 7, 1280x1024 IR pair
    ParallelLines = 3, // not available on POP 4
    VcselRapid = 4,    // "speckle": 0x3001 12, IR pair fmt 1 / type 2, host stereo
    Turntable = 5,     // inferred: auto-turntable variant of FullFieldHd
    HybridHd = 6,      // "speckle_stripe": fmt 5 / type 10 (s64y8y8), host stereo
}

/// Revo `ScanMode` ("Tracking Mode").
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum TrackingMode { Feature = 0, Marker = 1, GlobalMarker = 2, Color = 3 }

/// Revo scan accuracy. POP 4 has no `fastmode` rows; Standard reuses High-accuracy device params.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum AccuracyMode { Standard = 0, High = 1, HighSpeed = 2 }

/// Revo `ScanObject`. Order after `General` is inferred from the string table.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ObjectType {
    General = 0, Face = 1, Head = 2, Body = 3, Dark = 4,
    Reflective = 5, Outdoor = 6, Large = 7, BodyLarge = 8,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum CaptureMode { Handheld, Turntable, SingleShot }

pub struct ScanSettings {
    pub light: LightSource,
    pub tracking: TrackingMode,
    pub accuracy: AccuracyMode,
    pub object: ObjectType,
    pub capture: CaptureMode,
    pub color_texture: bool,   // Revo `scan_texture`
    pub point_pitch_mm: f32,   // 0.05..=2.0 on POP 4
    pub base_removal: bool,
}

impl ScanSettings {
    /// POP 4 (`0E1`) validity matrix from deviceScanParam.json.
    pub fn valid_on_pop4(&self) -> bool {
        use LightSource::*; use TrackingMode::*; use ObjectType::*;
        let light_ok = match self.tracking {
            Feature => matches!(self.light, FullFieldHd | VcselRapid | HybridHd),
            Marker | GlobalMarker => matches!(self.light, FullFieldHd | CrossLines | SingleLine),
            Color => matches!(self.light, FullFieldHd | VcselRapid | HybridHd | CrossLines),
        };
        let object_ok = match self.light {
            FullFieldHd if self.tracking != Feature && self.tracking != Color => self.object == General,
            FullFieldHd | HybridHd => matches!(self.object, General | Dark | Face),
            VcselRapid => matches!(self.object, General | Dark | Face | Outdoor),
            CrossLines | SingleLine => matches!(self.object, General | Dark | Reflective | Outdoor),
            _ => false,
        };
        let speed_ok = self.accuracy != AccuracyMode::HighSpeed
            || (matches!(self.light, CrossLines | SingleLine) && matches!(self.tracking, Marker | GlobalMarker));
        light_ok && object_ok && speed_ok
    }
}
```

These replace `LaserSourceMode` (`ParallelInfrared` does not exist on POP 4; "StructuredLight" is ambiguous between Full-field HD, VCSEL and Hybrid) and `ScanTrackingMode` (`DarkReflective` and `BodyFace` are object presets, not tracking modes; `Color` was missing).

## 10. Open items

- Confirm surface type 5 = turntable and the `ScanObject` order beyond `general` (disassemble the `ScanObject`↔string converter, or capture a Revo session with those options).
- Identify the 6th byte pair of `z16y8y8p` (display type 6, marker mode) and the `s64` plane of fmt 5.
- Capture a single-line session to confirm `0x3001 9`.
- Check whether `0x3001 12` changes depth quality in Full-field HD; compare with Revo's `0xa05 3` fringe setting.
- Verify the RGB stream framing on `camera_id=50&type_id=20`.
