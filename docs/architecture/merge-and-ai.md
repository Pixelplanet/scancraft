# Merging scans and AI object segmentation: decisions and plan

Status 2026-11-24. This note records what the user decided, what we measured on real scans and
the order of work. Implementation status is tracked at the end.

## 1. Decisions (user, 2026-11-24)

| # | Topic | Decision |
| --- | --- | --- |
| 1 | Segmentation model | SAM 2.1 (Meta, Apache-2.0). Use a good community ONNX export if one exists that we may use; otherwise our own export, hosted by the project owner or attached to a GitHub release. Never Revo's encrypted models. |
| 2 | ONNX Runtime | Downloaded on first install, only after the user agrees (`ort` with `load-dynamic`). |
| 3 | First AI release | SAM 2 **video** mode from the start (memory attention + memory encoder, mask carried from frame to frame), as Revo Scan 6 does. |
| 4 | Colour | Colour keyframes are recorded and stored by default; segmentation needs them. |
| 5 | GPUs | AMD now, including mobile / integrated AMD GPUs (Windows, DirectML or successor). Linux acceleration later. No CUDA dependency anywhere. |
| 6 | Test tool | `crates/scancraft-align/examples/align_ply.rs` is committed as the base for alignment tests. |
| 7 | Meaning of "merging" | All of it, optimised for the main case: the user scans one object several times in ScanCraft from different sides (e.g. flipped between passes) and merges the passes. Also: importing someone else's scan (PLY/OBJ/STL) and adding another ScanCraft project. |
| 8 | Licence | MIT or Apache-2.0 (`LICENSE-MIT`, `LICENSE-APACHE`); can be revisited. |
| 9 | AI assistant | Later (soon): an AI API interface like SolidCraft's agent chat, so a model can help with tasks such as judging alignments. Design note: section 6. |

## 2. What Revo Scan 6 does (analysis of the installed app, 2026-11-24)

- AI segmentation is SAM 2 video segmentation on the RGB camera stream, live while scanning. The
  user places positive/negative points in the RGB view; the mask is projected to 3D through the
  colour camera calibration and a depth buffer. Enabling it switches colour on.
- Four ONNX graphs: image encoder, memory attention, mask decoder, memory encoder.
- The models ship as an optional installer component (about 1 GB), stored encrypted ("RPA1"
  containers): one bundle for CUDA, one for DirectML on AMD. On a PC without an NVIDIA GPU Revo
  loads its AMD module: ONNX Runtime 1.24 + DirectML, 48–105 ms per frame on an RX 6900 XT. Revo
  requires a discrete GPU; we also allow integrated GPUs.
- We do not decrypt or reuse these files. The architecture (SAM 2 video on ONNX Runtime +
  DirectML) is what we copy.

## 3. Alignment test on three Revo scans of one part (2026-11-24)

Data: three Revo Scan 6 projects of a plastic shell part (principal extents about
307 × 237 × 113 mm, wall about 3.0–3.1 mm). Scan 1 shows the outside, scans 2 and 3 the inside. Revo Scan had trouble aligning
them. The scans are not in the repository.

Findings (each pair checked independently from the geometry; there is no ground truth):

- With the table removed and the part isolated (`align_ply --prep part`) on Revo's raw previews,
  `align_global` aligns 2 ↔ 3 (three marker stickers on the inner face line up within 1–2 mm)
  and 3 ↔ 1 (scan 3 sits 3.08 mm inside scan 1, IQR ±0.15 mm, under 0.1 % poking through).
- 1 ↔ 2 was never found directly (about 30 attempts): an outside and an inside scan share only
  the rim. Chaining 2 → 3 → 1 gives the same clean 3.08 mm wall.
- Failure causes: the table and detached fragments; the quality metric counted overlap only for
  normals facing the same way, so a correct inside/outside nesting scored Poor and some fits
  that cut through the wall scored Good; sliding along a smooth shell; sensitivity to the feature
  voxel size and to the direction (i → j vs j → i); a k-d tree panic on Revo's grid-quantised
  coordinates. Revo's meshes are worse input than its point clouds (smoothing removes features).
- Correction found later: scan 1 contains no table. `align_ply --prep part` took the part's flat
  rim for a table and cut 61k real points; the library's `segment_object` keeps them.

Result of the library work (same day, `merge_ply` example, `align_all` with S1 as reference):

| Edge | Result |
| --- | --- |
| S1 ↔ S3 | measured, Good, directions agree within 0.52° / 1.3 mm; wall 3.03–3.06 mm, IQR 0.22 mm |
| S2 ↔ S3 | measured, Good, directions agree within 0.34° / 0.54 mm; same-side RMS 0.12 mm |
| S1 ↔ S2 | measured edge rejected (Poor, directions disagree); composed through S3: wall 3.11 mm, IQR 0.21 mm, 0.01 % piercing |

Independent check against the marker positions Revo stores per project (`marker_framework.mkf`:
7 markers shared by S2/S3, 6 by S3/S1, fitted with Kabsch and compared on the part): our poses are
within 0.20–0.27° / 0.37–0.61 mm of the marker fit; the earlier "verified" chained transforms were
1.05–1.15° / 2–2.5 mm off and inconsistent between directions. Whole-scene alignment (no object
isolation) rejects every edge and places nothing, rather than placing a pass wrongly. Joint
shell-aware refinement lowered the loop residual (0.87 → 0.42 mm) but loosened the same-side fit
(S2/S3 RMS 0.12 → 0.16 mm), so it is only kept when no pair gets worse; on this data the graph
poses are kept. Runtime: about 60 s for `align_all` on three passes (six global alignments per
pair).

## 4. Plan (order of work)

1. **Alignment and merge core** (in progress on `feature/align-merge`): k-d tree tie fix; shared
   resampling grid; support-plane plausibility veto; thickness-aware ("shell") quality with wall
   thickness and piercing; sliding/degeneracy warning; `align_robust` (both directions, several
   voxel sizes); `align_all` pose graph with composed edges and cycle checks; thickness-aware fine
   refinement; geometric object isolation (`scancraft-fusion` `segment_object`) used by default
   before alignment, with a "Use whole scene" toggle; app fixes (stale cleaned/mesh after
   re-align, undo after a failed job, Refine as a preview); "Align all"; Isolate Object tool;
   PLY/OBJ/STL import as static passes; "Add project". Real-data regression: `merge_ply` example
   and ignored tests (`SCANCRAFT_ALIGN_DATA`).
2. **Colour keyframes stored by default**, with poses, in the project.
3. **Segmentation crate** (`scancraft-segment`): prompt types, pre/post-processing, mask → depth
   projection with occlusion; geometric fallback.
4. **AI pack manager**: consent dialog, download with SHA-256 check, install from file, remove;
   files under `%LOCALAPPDATA%/ScanCraft/ai`. No network access without consent; CI never
   downloads.
5. **ONNX Runtime backend**: DirectML (any DX12 GPU, integrated AMD included) → CPU → geometric;
   per-adapter self-test against CPU; no CUDA/TensorRT features (checked in CI).
6. **SAM 2.1 video loop** (memory bank, object pointers) live while scanning, plus post-scan
   "Segment object" over stored keyframes with a per-point vote.
7. **Merge colour and robustness**, then the AI assistant interface.

Solid model test case (user, 2026-11-24): the merged shell part is the benchmark for turning
points into a watertight solid: outer and inner surface joined at the rim with the 3 mm wall
kept, holes filled, raised marker stickers removed with a smooth surface, ribs and bosses kept.
Two meshing pipelines were prototyped against each other and judged on this data (a banded
screened Poisson solve won over a material-band field). The result is `scancraft-mesh::solid`
(`build_solid`, plus standalone `remove_markers`, `fill_holes`, `fair_marker_sites`):
clean (outliers, ghost layers, one layer per sheet with a trusted scan first), cut and refill
marker stickers from a quadric fitted to the surrounding ring, complete faces that were never
scanned by offsetting the other face by the measured wall, fill and bridge holes, solve a banded
screened Poisson field at 0.4 mm, extract, collapse slivers, repair self-intersections, fair the
marker sites on the mesh. Every vertex is tagged measured / marker fill / hole fill / completed.

On the three Revo scans with the `align_all` poses (1.22 M points, about 41 s, 0.9 GB): watertight,
manifold, one component; wall median 3.10 mm; outside scan to mesh median 0.011 mm, p95 0.044 mm;
82.8 % of vertices measured, 13.4 % completed; 20 of 23 marker sites faired (median deviation
from the ring fit about 25 µm on the earlier run). Open: genus 23 (the real part has about 5
openings; data gaps near the hinge bracket stay open as tunnels), a two-sided gap next to the
hinge bracket only partly bridged, a few marker sites at edges and ridges not smooth, inside
accuracy limited by the remaining S2/S3 misfit, marker positions come from Revo's
`marker_framework.mkf` or a geometric detector.

Research on community exports, the runtime download and integrated-GPU speed: section 5.

## 5. Model and runtime research (2026-11-24, desk research, not yet tested on hardware)

Model:
- Public SAM 2.1 exports that include the video graphs: `jax-image-tools/sam21-tiny-video-onnx`
  on Hugging Face (four graphs: vision encoder 109.5 MB, mask decoder with object pointer
  17.8 MB, memory attention 32.4 MB, memory encoder 5.6 MB; fp32, opset 17, Apache-2.0, reported
  per-frame IoU 1.0 against PyTorch; its export script is not public), `square-zero-labs` (five
  static graphs), Diffusion Studio's fp16 exports (script public, MPL-2.0), ailia (Apache-2.0
  files, but a 6-D MatMul in memory attention). samexporter, onnx-community, SharpAI and
  ibaiGorordo are image-only.
- No export ships the video loop (memory bank, object pointers, temporal encodings): we write
  that in Rust.
- Plan: start with the jax-image-tools export pinned to a commit and SHA-256, mirrored to a
  GitHub release with LICENSE and NOTICE (pinned Hugging Face URL as fallback). Write our own
  `export_sam2_video.py` (own code) to produce variants we need, notably tiny at 512 px with four
  memories for integrated GPUs, with an IoU ≥ 0.99 gate against PyTorch on CPU and DirectML.

Runtime:
- ONNX Runtime 1.24.4 `Microsoft.ML.OnnxRuntime.DirectML` from nuget.org (the last DirectML
  build; Microsoft calls DirectML "sustained engineering" and recommends Windows ML) plus
  `Microsoft.AI.DirectML` 1.15.4 for `DirectML.dll`. `ort =2.0.0-rc.15` with `load-dynamic`,
  `directml`, `api-24`, no `download-binaries`/`copy-dylibs`/CUDA. Pin SHA-256 per file and check
  the Authenticode signature before loading. DirectML.dll's licence allows distribution as part
  of an application on Windows only and has a telemetry clause: show that in the consent dialog.
- Optional second backend: ORT's WebGPU plugin EP (also the likely Linux path later, over
  Vulkan). Spike worth doing: Windows ML 2.x (ORT 1.28 + vendor EPs; licence to review).
- Backend order per adapter, chosen by a self-test against CPU: DirectML → WebGPU → CPU →
  geometric.

Integrated AMD GPUs (estimates, nothing measured yet): DirectML covers Radeon 680M/780M/890M.
Tiny at 1024 px is estimated at 250–500 ms per tracked frame on a 780M, tiny at 512 px at
60–120 ms; memory attention at 1024 px needs about 1–1.5 GB of shared memory. Live tracking on an
iGPU therefore runs on a subset of colour frames, with a post-scan pass over stored keyframes.
First hardware task: time each graph on the laptop's GPU.

## 6. AI assistant interface (later)

Design study of SolidCraft's agent chat (2026-11-24). SolidCraft has one MCP server used two
ways: external coding agents start the CLI as an MCP server (headless, or connected to the
running app over a loopback port with a per-launch token), and an in-app chat panel talks either
to coding agents over ACP (they use their own logins) or to OpenAI-/Anthropic-compatible API
endpoints (key in the OS keychain). Tools are thin wrappers over engine commands, each marked
read-only or not; a gate decides Run / Ask / Refuse, with permission modes, per-agent "always
allow", turn limits and one undo step per turn. Both projects are MIT OR Apache-2.0, so the
protocol, process, discovery and keychain layers can be adapted with attribution.

ScanCraft work packages, in order: a command layer over the app's actions (with job ids);
projects loadable without a window; an inspection kit (signed-offset/wall statistics, sections,
offset maps, CPU renders, the tools our verification agents used); a headless MCP server
(milestone 1); bundled skills for aligning, cleaning and merging; a loopback control endpoint
for the open window; the agent crate (ACP + API backends); the gate with a separate scanner
category; the assistant panel (milestone 2); an assistant benchmark that checks results, not
answers. Constraints: scans and API keys stay local unless the user picks a cloud provider; an
assistant never controls the scanner without explicit approval, and the scanner safety rules
(`docs/HANDOFF.md` section 3) apply to it as to any client.

## 7. Status

| Item | State |
| --- | --- |
| Licence files, `align_ply` committed | done 2026-11-24 |
| Alignment core: exact k-d tree, shared grid, table veto, shell (wall) quality, slide check, `align_robust`, `align_all`, `refine_shell` | done 2026-11-24 |
| Object isolation (`scancraft-fusion::segment_object`), used by default; "Use whole scene" | done 2026-11-24 |
| App: stale-result fixes, Refine preview, Align All, Isolate Object, wall read-outs, Import Scan (PLY/OBJ/STL), Add Project | done 2026-11-24; native file / folder dialogs (Import Scan, Add Project, Open Project; Browse on Save Project and Export) 2026-11-24 |
| Workflow order Connect → Scan → Clean → Align → Mesh → Export (user, 2026-11-24). Clean works per pass: each tool runs on every shown pass separately (and on the merged model when shown); hidden passes are untouched. A pass's cleaned surface (`ScanPass::edit`, own frame, observation counts in `confidence`) replaces its tracked / imported surface for alignment, merge and saving (`pass-<id>/cleaned.ply` + `cleaned_observations.bin`). Isolate Object works on one shown pass at a time. A cleaned captured pass cannot be continued or have frames discarded until Reset Cleaning. Old projects' single `outputs/cleaned.ply` opens as the merged model. | done 2026-11-24 |
| Real-data regression (`merge_ply`, `tests/real_data.rs`, `SCANCRAFT_ALIGN_DATA`) | done; criteria loose (1.5° / 3 mm): tighten with the `.mkf` marker positions |
| Meshing rework (2026-11-24, measured on the user's two-pass shell scan, `crates/scancraft-mesh/examples/mesh_project.rs`): the input was fine (passes agree to ±0.04 mm, plane residual 0.03 mm) but the merged points' normals were 10–13° off the local plane, and the old narrow tangent-plane field turned that, plus marching-cubes faceting, into grain and a scaly band where passes overlap. New `implicit.rs`: normals re-fitted from positions (orientation by neighbourhood vote), robust implicit MLS (RIMLS) over a support radius of `detail × spacing`, thin-wall side test, outlier drop, marching cubes, vertex projection onto the surface. New `denoise.rs`: fold repair + bilateral normal filtering (edge-preserving). Reconstruct = both; Smooth = the new smoother; Fill Holes tool (`holes.rs`). Remaining gaps on that scan are missing data (pen ink, marker stickers), closed by Fill Holes. | done |
| Solid model library (`scancraft-mesh::solid`, example `solid_ply`) | done 2026-11-24; not yet wired into the app's Mesh stage |
| Segmentation crate, ORT DirectML/CPU backend, SAM 2.1 video loop (`crates/scancraft-segment`, example `segment_seq`) | done 2026-11-25 (`ort` 2.0.0-rc.15 `load-dynamic` + `directml` + `api-24`, no CUDA/TensorRT; SHA-256 verification on `jax-image-tools/sam21-tiny-video-onnx` 1024 px graphs + ScanCraft 512 px static-memory exports, `onnxruntime.dll` 1.24.4, `DirectML.dll` 1.15.4; DXGI adapter self-test vs CPU + `load_fastest_variant` timing selector). **512 px static-memory export & live iGPU optimisation (`tools/sam2/export_sam2_video.py`, `validate_sam2_video.py`, 2026-11-25):** Exported `sam21-tiny-512-mem4-fp32`, `sam21-tiny-512-mem4-fp16`, `sam21-tiny-512-mem2-fp32`, `sam21-tiny-512-mem2-fp16` from `facebook/sam2.1-hiera-tiny` (`de431c404385`, Apache-2.0) with fixed memory blocks (`M=4` or `2`), `max_pointers=8`, static additive `memory_attention_mask`, exact `no_memory_embedding`, linear-projected pointer temporal PE, and antialiased scaled-triangle RGB downsampling in `preprocess.rs`. Keeping `mask_decoder.onnx` in FP32 while converting `vision_encoder`, `memory_attention`, and `memory_encoder` to FP16 (~106 MB on disk vs ~192 MB FP32) preserved mask accuracy (`worst IoU >= 0.99` gate; full-FP16 `mask_decoder` dipped to 0.970 on `bedroom`). **Measured on AMD Radeon 890M (25-frame sequences: `bedroom` 960×540 pt `360,270`, `blocks` 1280×720 pt `200,180`, 24 tracked frames mean):** DirectML: `1024-fp32` 1003.5–1033.9 ms/frame (ve 333–341, mem_attn 608–620, dec 22–23, mem_enc 21, pre+post 20–29, peak 1188 MB); `512-mem4-fp32` 144.3–158.7 ms (ve 73–82, mem_attn 49–50, dec 9–10, peak 416–420 MB); `512-mem4-fp16` 113.5–117.8 ms (ve 64–65, mem_attn 30, peak 365–367 MB); `512-mem2-fp32` 124.4–127.7 ms (ve 71, mem_attn 34, peak 410–411 MB); **`512-mem2-fp16` 106.4–110.8 ms/frame (~9.4× faster, ~9.2 fps)** (ve 63.7–64.9, mem_attn 23.1–23.2, dec 8.1–8.2, mem_enc 4.3–4.4, pre+post 6.9–10.4, peak **361–362 MB**, 3.3× lower RAM). **CPU (8 threads):** `1024-fp32` `bedroom` 2290.4 ms (peak 3715 MB); `512-mem4-fp32` 365.2–375.2 ms (peak 748–762 MB); `512-mem4-fp16` 410.9–430.4 ms (peak 1018–1022 MB); `512-mem2-fp32` 326.1–347.1 ms (peak 658–672 MB); `512-mem2-fp16` 380.3–405.7 ms (peak 964–967 MB). **PyTorch IoU >= 0.99 gate (`validate_sam2_video.py` vs PyTorch @ matching `size/mem`):** `512-mem2-fp16` DirectML PASS on `bedroom` (mean 0.99770, worst 0.99251) & `blocks` (mean 0.99842, worst 0.99604) and CPU PASS (`bedroom` mean 0.99775/worst 0.99214; `blocks` mean 0.99835/worst 0.99604); `512-mem2-fp32` PASS on both DirectML & CPU (mean 0.9978–0.9984, worst 0.9920–0.9961); `512-mem4-fp32` PASS on both DirectML & CPU (mean 0.9977–0.9985, worst 0.9920–0.9957); `512-mem4-fp16` PASS on `bedroom` DirectML/CPU (worst 0.9921–0.9925) & `blocks` CPU (worst 0.99576), dipped on one `blocks` DirectML frame (mean 0.99511, worst 0.95510). PyTorch 512 vs 1024 (`mem7`) IoU: `bedroom` mean 0.971 (`mem4`/`mem2`), `blocks` mean 0.972 (`mem4`) / 0.939 (`mem2`). `load_fastest_variant` on Radeon 890M chose `512 px · mem2 · FP16` (102.3 ms on test clip). Release staging prepared under `%LOCALAPPDATA%\ScanCraft\ai\release-staging\` (`sam21-tiny-512-mem2-fp16`, `sam21-tiny-512-mem2-fp32`, `sam21-tiny-512-mem4-fp32`, `sam21-tiny-512-mem4-fp16`, each with 4 `.onnx`, `constants.json`, `manifest.json`, `LICENSE`, `NOTICE`). |
| Colour keyframes in projects, AI pack downloader & consent dialog, app UI wiring | done 2026-11-25 (`scancraft-color` encodes JPEG fallback when frame only has raw RGB; `MockDevice` emits calibrated RGB frames when `color_texture` is on; `PassPipeline::take_new_rgb_keyframes` exports new keyframes during scan; `ScanCraftApp` defaults `color_texture = true` on RGB-capable scanners and persists `pass-<id>/rgb/keyframes.json` + `kf-*.jpg` in projects; `ai_pack.rs` verifies SHA-256 and supports explicit-consent download/local-folder install/removal with Apache-2.0 / MIT / DirectML Microsoft licence + telemetry notices; `Tool::AiSegment` interactive split-view prompter + forward/backward SAM 2.1 propagation + multi-view 3D projection + Clean apply/undo using 1024 px model; `worker.rs` live capture tracking wired to `SharedAiEngine::with_live_segmenter` selecting fastest verified 512 px variant with 1024 px fallback + `ui_ai.rs` live variant status row; `camera_inset` `Colour` view. Verified live + unit tests). |

## 8. Backlog (later)

Ideas from the ScanRuler study (`docs/research/2026-11-24-scanruler.md`). ScanRuler is AGPL:
re-implement from the ideas, never copy its code.

| Item | Where | Effort |
| --- | --- | --- |
| Auto-orient a merged part to its own faces (area-weighted normal histogram, axis from wall normals, open rim as the base) — done 2026-11-25 (`scancraft_align::auto_orient`, app "Auto Orient" proposal via `move_project_frame`) | after merging, before meshing/export | S–M |
| "Ambiguous" warning when a clearly different pose scores almost as well | `scancraft-align` quality | S |
| Score poses by a capped mean over all samples; facing-aware closest-point search | refine / quality | S |
| Manual 3-point alignment (Horn's method, collinear warning) as the fallback when Align All rejects an edge — done 2026-11-25 (`scancraft_align::fit_pairs` as the `align_point_pairs` start, Pick-fit row + nearly-collinear warning in the Point Pairs card) | `align_point_pairs`, app | S |
| Datum alignment to world axes (plane / axis / origin, live preview) and primitive fits grown from a click — done 2026-11-25 (`scancraft_align::fit` / `datum`, app "Orient by Datums" tool) | app | M |
| Scale from several measured lengths (imported photogrammetry scans) — done 2026-11-25 (`scancraft_align::scale_from_lengths`, app "Scale from Lengths" tool on imported passes) | import | S |
| Volume-error check (excess / missing material regions) | solid-model benchmark | M |
| Review tool for an AI agent (ranked worst places on renders, trend, target) with guides served by the app | AI assistant, §6 | M |
| STEP export of fitted primitives (analytic AP214, own writer or `truck-stepio`) | export | M |
| Scan-to-CAD in SolidCraft: ScanCraft hands over the watertight mesh plus fitted primitives | SolidCraft | L |

Known gaps: `scancraft_slam::point_to_plane` still linearises about the frame origin (slow
convergence far from it).

Closed 2026-11-25: per-pass object labels (`pass-<id>/object.bin`, with the fingerprint that
decides whether they still fit) and every alignment quality field (inliers, table violation,
slide, wall pairs / offset / IQR / piercing) are saved. The project format stays version 1:
every addition is optional, older projects open as before and older versions of the app ignore
the new parts. `save_ply_binary` writes normals (when the cloud has them) and the observation
count as `quality`, and the PLY reader takes `quality` / `confidence` back. Imported passes are
named after the file without its extension, or after their folder when the file has a generic
export name (`raw_preview`, `scan`, `mesh`, … with or without a counter); duplicates get
"(2)", "(3)".
