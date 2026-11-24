# SAM 2.1 video segmentation: models, ONNX Runtime delivery, AMD iGPU outlook, licences

Research for `docs/architecture/merge-and-ai.md` section 5. Date: 2026-11-24.
Tags: **[C]** = confirmed at the URL given (checked on 2026-11-24); **[I]** = inferred / estimate, needs a test.
No model or runtime binary was downloaded; sizes and hashes come from the Hugging Face API, the NuGet
catalog and HTTP HEAD requests. (The `ort` / `ort-sys` 2.0.0-rc.15 crate sources were fetched from crates.io
into the session scratchpad and read, not built or run.)

---

## 0. Short answer

| Question | Recommendation |
| --- | --- |
| Model | **Start from `jax-image-tools/sam21-tiny-video-onnx` (SAM 2.1 Hiera-Tiny, 4 graphs, fp32, opset 17, Apache-2.0)**, pinned to commit `3cb090d9…` and mirrored in our GitHub release (Apache-2.0 allows it). In parallel, write our own `export_sam2_video.py` following the Diffusion Studio script layout (transformers `Sam2VideoModel`), because no community export has a reproducible public script **and** fp16 **and** a 512 px variant **and** DirectML testing. Our own export becomes the shipped model once it matches PyTorch. |
| Runtime (Windows, now) | **ONNX Runtime 1.24.4 from NuGet**: `Microsoft.ML.OnnxRuntime.DirectML` 1.24.4 (the **last** DirectML build Microsoft published) + `Microsoft.AI.DirectML` 1.15.4. `ort = 2.0.0-rc.15` with `default-features = false`, features `load-dynamic`, `directml`, `api-24`. Optional second GPU path: the **WebGPU plugin EP** `Microsoft.ML.OnnxRuntime.EP.WebGpu` 0.6.0 (needs ORT >= 1.24.4, so the same ORT build works). |
| Download source | **nuget.org flat container** (immutable package versions, Microsoft-signed `.nupkg`, SHA-512 in the catalog). Not GitHub: `github.com/microsoft/onnxruntime` has returned 404 since about 2026-11-17, so all GitHub release zips are currently unreachable. |
| Linux (later) | ORT 1.24.4 `libonnxruntime.so` (in `Microsoft.ML.OnnxRuntime` 1.24.4 NuGet) + WebGPU plugin EP `.so` (Vulkan via Dawn). DirectML is Windows-only. |
| AMD iGPU outlook | Works in principle (DirectML runs on any DX12 GPU; 680M/780M/890M are DX12). Expected **~0.25–0.5 s per tracked frame on a 780M at 1024 px, ~0.5–1 s on a 680M** [I]; a 512 px export is roughly 4–5x faster. Live tracking on iGPUs must therefore run on a subset of colour frames (2–4 fps), with a post-scan pass over keyframes; discrete Radeon reaches Revo's ~50–100 ms. |

---

## 1. Public SAM 2 / 2.1 ONNX exports: what includes the video graphs

SAM 2 video needs: image encoder (Hiera + FPN), prompt encoder + mask decoder **with the object-pointer
output**, memory encoder, memory attention (RoPE cross-attention over the memory bank), plus small
pieces (object-pointer projection MLP, temporal position encoding of the pointers) and a host-side loop
(memory bank of 1 conditioning + 6 recent frames, up to 16 object pointers, temporal encodings).

SAM 2.1 tiny config [C] <https://raw.githubusercontent.com/facebookresearch/sam2/main/sam2/configs/sam2.1/sam2.1_hiera_t.yaml>:
memory attention 4 layers, d_model 256, self- and cross-attention **1 head**, kv_in_dim 64,
RoPE feat_sizes 64x64; num_maskmem 7; image_size 1024; Hiera embed_dim 96, stages [1,2,7,2].

### 1.1 Overview table

| Export | Video graphs? | Sizes | Precision / opset / shapes | Licence | Script public? | Notes |
| --- | --- | --- | --- | --- | --- | --- |
| **jax-image-tools/sam21-tiny-video-onnx** | **Yes**: vision_encoder, mask_decoder (with `object_pointer`), memory_attention, memory_encoder | 109.5 / 17.8 / 32.4 / 5.6 MB (165 MB) | fp32, **opset 17**, encoder input static 1x3x1024x1024; memory_attention **dynamic** axes `spatial_memory`, `pointer_memory` (no padding) | Apache-2.0 | Named (`browser-onnx-tools/export/export_sam_video_onnx.py`) but the GitHub repo returns **404** | Best validated: worst per-frame IoU 1.000000 vs PyTorch `propagate_in_video_iterator`, 8 frames. Target: onnxruntime-web WebGPU. Commit `3cb090d9d0b4c897281fdcac7765c0f3a888c49f`, 2026-09-01. |
| **square-zero-labs/sam2.1-tiny-video-onnx** | **Yes**: vision_encoder, mask_decoder, memory_encoder, memory_attention, pointer_tpos | 134.3 / 17.8 / 5.6 / 32.3 / 0.07 MB (190 MB) | fp32; all **static** (memory 28736x1x64 = 7x4096 + 64 pointer tokens, padded by duplication on early frames) | Apache-2.0 | No (says "transformers v5.11") | Loop documented on the card (seed frame, binarize flag, bank build). Worst IoU 0.9967. Commit `3b2984dd865f6e9d2cc6aed0be6a5a5c2eb352ce`, 2026-06-11. |
| **diffusionstudio/sam2.1-{tiny,small,base-plus,large}-video-onnx-fp16** | **Yes**, same 5-graph layout as square-zero | tiny: 58.4 / 8.9 / 13.0 / 2.8 / 0.03 MB; small: 83.7 / 8.9 / 16.2 / 2.8 / 0.03 MB | **fp16 compute, fp32 I/O**, opset 17, static; **tiny repo is 512 px** (32x32 features), small is 1024 | Apache-2.0 | **Yes**: <https://raw.githubusercontent.com/diffusionstudio/editor/main/packages/sam2/scripts/export.py> (repo MPL-2.0) | No accuracy validation reported. Timings (M1 GPU, ORT-web WebGPU): tiny 1024 1.3 s/frame, tiny 512 0.25 s, small 1024 1.9 s. Commits `66673b5d…` (tiny), `927ecec6…` (small), 2026-09. |
| **ailia (axinc-ai) SAM 2.1** | **Yes**: image_encoder, prompt_encoder_with_mask, mask_decoder, memory_attention (6D or 4D `.opt`), memory_encoder, mlp, obj_ptr_tpos_proj | tiny: 109.5 / 0.09 / 16.9 / 133.0 (6D) or 99.4 (opt) / 5.6 / 0.8 / 0.07 MB | fp32, **opset 17**; 6D matmul variant = dynamic batch, `.opt` = 4D, batch 1 | Model folder LICENSE = **Apache-2.0** (SAM 2's); ailia SDK itself is commercial but **not needed** (`--onnx` runs on onnxruntime) | Yes: fork <https://github.com/ailia-ai/segment-anything-2> (Apache-2.0), branches `onnx`, `onnx_2_1`; `export_video_predictor.py --framework onnx` | Hosted on `storage.googleapis.com/ailia-models/segment-anything-2.1/` (no SHA-256 published, MD5 only in headers). Loop in `sam2_video_predictor.py` (Python, ailia-models). Oldest and most used video export (2024-12, memory_attention_6d updated 2026-03-30). |
| glowskeleton/goblin-models (`sam2-video/`) | memory_attention, memory_encoder, decoder with `obj_ptr` (vision encoder = onnx-community's) | 28.1 / 5.6 / 22.0 MB | fp32 | Apache-2.0 | Part of "Goblin" tool (not checked) | States RoPE had to be rewritten to real-valued ops (complex tensors have no ONNX form). |
| s-tornqvist/sam2-hiera-base-plus-video-ONNX | memory attention/encoder, pointer proj, decoder (SAM 2, base+) | 133 / 5.6 / 0.8 / 17 MB | fp32 | **none stated** | No | Not usable (no licence, base+ only). |
| PRVSM01/sam2.1-memory-attention-dynamic-ONNX | memory attention + decoder only (base+) | 33 / 17.5 MB | fp32 | Apache-2.0 | No | Incomplete. |
| vietanhdev/samexporter + `vietanhdev/segment-anything-2.1-onnx-models` | **No** (encoder + decoder only) | zips 111 MB (tiny), 136 MB (small) | — | MIT tool, Apache-2.0 models | Yes | README: image ONNX exports only; no video. |
| onnx-community/sam2.1-hiera-tiny-ONNX (transformers.js) | **No** (vision_encoder + prompt_encoder_mask_decoder, no object pointer) | fp32 134 MB + 21 MB; fp16 67 + 10.5 MB; q4/int8 variants | external data (`.onnx_data`) | no licence on card | — | Image only. |
| SharpAI/sam2-hiera-tiny-onnx | **No** (encoder, decoder) | — | fp32 | Apache-2.0 | — | Image only. |
| ibaiGorordo/ONNX-SAM2-Segment-Anything | **No** ("video prediction will be available soon") | — | — | MIT code / Apache models | Colab | Image only. |
| SegmentAnything.Onnx (NuGet 0.1.0.1, C#, MIT, <https://github.com/Agredo/SegmentAnything>) | Claims SAM 2 video with temporal memory, DirectML | models not stated | — | MIT | — | Tiny, 2025-08, few downloads; model source unclear. |
| rlx-sam2 (Rust crate, docs.rs 0.2.x) | Re-implements memory attention/encoder in its own IR, not ONNX | — | — | — | — | Useful as a Rust reading of `memory_attention.py` (RoPE, pointer tokens without RoPE). |

Sources [C]: <https://huggingface.co/jax-image-tools/sam21-tiny-video-onnx> (README + `constants.json`: opset 17,
`memory_axes_dynamic`, `memory_length_traced` 28736, `tokens_per_pointer` 4),
<https://huggingface.co/square-zero-labs/sam2.1-tiny-video-onnx>,
<https://huggingface.co/diffusionstudio/sam2.1-tiny-video-onnx-fp16>,
<https://huggingface.co/diffusionstudio/sam2.1-small-video-onnx-fp16>,
<https://github.com/axinc-ai/ailia-models/tree/master/image_segmentation/segment-anything-2>,
<https://raw.githubusercontent.com/axinc-ai/ailia-models/master/image_segmentation/segment-anything-2/LICENSE> (Apache-2.0),
<https://raw.githubusercontent.com/axinc-ai/ailia-models/master/LICENSE.md> (model licence = upstream's),
<https://huggingface.co/glowskeleton/goblin-models>, <https://github.com/vietanhdev/samexporter>,
<https://huggingface.co/vietanhdev/segment-anything-2.1-onnx-models>, <https://huggingface.co/onnx-community/sam2.1-hiera-tiny-ONNX>,
<https://huggingface.co/SharpAI/sam2-hiera-tiny-onnx>, <https://github.com/ibaiGorordo/ONNX-SAM2-Segment-Anything>,
<https://www.nuget.org/packages/SegmentAnything.Onnx>. File sizes and SHA-256 from
`https://huggingface.co/api/models/<repo>?blobs=true` (LFS `sha256`).

### 1.2 Pinning and hashes (Hugging Face LFS SHA-256)

Pinned URL form [C]: `https://huggingface.co/<repo>/resolve/<commit>/<path>` (a commit hash never changes;
the repo can still be deleted by its owner, hence mirroring).

jax-image-tools/sam21-tiny-video-onnx @ `3cb090d9d0b4c897281fdcac7765c0f3a888c49f`:

| File | Bytes | SHA-256 |
| --- | --- | --- |
| vision_encoder.onnx | 109 497 134 | `98311f41182f40c57cac2522180fff34b53931ce981412ddd8643ec27fb830e3` |
| mask_decoder.onnx | 17 800 545 | `6e75023b52586314ab1dcf5a49fb9731a983ffe2b7b2a419bd42cd2b2e77effa` |
| memory_attention.onnx | 32 364 264 | `afd5045a8fe48628d4da49a82b77351a43f7554f1f101a5052ab74aae69e3737` |
| memory_encoder.onnx | 5 586 836 | `3a97e8ba25180b39e938ab52a0c4d085007b25f3e919f5672d4ecbcdd2ab9ee3` |
| constants.json | 845 | (not LFS; hash at pin time) |

square-zero-labs/sam2.1-tiny-video-onnx @ `3b2984dd865f6e9d2cc6aed0be6a5a5c2eb352ce`:
vision_encoder `aa7a8542…bb873` (134 335 567 B), mask_decoder `0461896d…070c6`, memory_attention `791e648c…4baf`,
memory_encoder `580d246c…834cd`, pointer_tpos `7e71df1d…101fc` (full hashes from the API; see §1.1 source).

diffusionstudio tiny-fp16 (512 px) @ `66673b5db39371b7dd7847f4d3bc0d4f4179b79e`: vision_encoder `24e9ccf5…0b133`
(58 385 353 B), memory_attention `67c4e4da…342c9` (13 029 470 B).

Upstream checkpoints [C] (`facebook/sam2.1-hiera-tiny` @ `de431c40…`, Apache-2.0): `sam2.1_hiera_tiny.pt` 156 008 466 B,
SHA-256 `7402e0d864fa82708a20fbd15bc84245c2f26dff0eb43a4b5b93452deb34be69`; `facebook/sam2.1-hiera-small`:
`sam2.1_hiera_small.pt` 184 416 285 B, `6d1aa6f30de5c92224f8172114de081d104bbd23dd9dc5c58996f0cad5dc4d38`.
Meta's direct URLs [C] <https://github.com/facebookresearch/sam2>:
`https://dl.fbaipublicfiles.com/segment_anything_2/092824/sam2.1_hiera_tiny.pt` (and `_small.pt`).

### 1.3 Runtime loop: documented or to be re-implemented?

- No export ships a native runtime loop we can reuse. [C] jax-image-tools: "memory bank assembly is done in JS";
  square-zero-labs documents the steps in prose; ailia has a Python `SAM2VideoPredictor`. **We re-implement the
  loop in Rust** (in `scancraft-segment`) [I: small, ~400–700 lines]:
  1. Seed frame: vision encoder -> decoder on `feats2_no_mem` with the user's points -> memory encoder with
     `binarize = 1` (conditioning memory, temporal slot 6 / "cond").
  2. Every later frame: vision encoder -> memory attention over [cond memory + up to 6 recent memories,
     each 4096 tokens x 64, plus temporal PE per slot] + [up to 16 object pointers, each split into 4x64 tokens,
     with sine PE of the normalised frame offset (pointer_tpos)] -> decoder with a padding point (label -1)
     -> memory encoder with `binarize = 0`; push memory and pointer.
  3. Occlusion: `object_score_logits <= 0` suppresses the mask and uses the learned no-object pointer (inside the
     square-zero/diffusionstudio graphs).
- Graph contract differences matter for the loop: square-zero/diffusionstudio need a fixed 28736-row bank
  (pad by duplication), jax-image-tools takes spatial blocks and pointers as separate dynamic inputs.
  **For DirectML the static form is better** ([C] DirectML EP docs: best with fixed shapes; free dimensions can be
  fixed with `AddFreeDimensionOverrideByName`). With the dynamic jax-image-tools graph we would fix the
  dimensions to the full bank size and pad ourselves [I].
- Scanner-specific choice [I]: frames come from a moving handheld scanner at a few fps and we can choose
  which colour frames to feed; SAM 2 tolerates frame skipping (temporal PE uses frame offsets), so feeding
  every k-th colour keyframe is a supported pattern.

### 1.4 Known DirectML risks for these graphs

- [C] DirectML EP: opset <= 20 supported (all exports are opset 17 — fine); memory pattern and parallel
  execution must be disabled; one `Run` per session at a time; unsupported ops: GridSample-20 5D,
  DeformConv (SAM 2 uses neither [I]).
  <https://onnxruntime.ai/docs/execution-providers/DirectML-ExecutionProvider.html>
- [C] RoPE: SAM 2's complex-number RoPE has no ONNX form; every video export rewrote it to real ops
  (goblin card; ailia fork README "complex tensor in RotaryEnc replaced with matmul").
- [C] ailia's default memory_attention uses **6-D MatMul**; [I] 6-D MatMul/broadcast is a classic DirectML weak
  spot — prefer 4-D graphs (`.opt`, or the transformers-based exports).
- [I] Memory: memory attention is single-head (config above); one cross-attention score matrix at 1024 px with
  7 memories is 4096 x 28736 = 117.7 M elements = **471 MB fp32 / 235 MB fp16** per layer if materialised
  (ORT's plain MatMul+Softmax graph does materialise it). Budget **~1–1.5 GB GPU memory** for tiny at 1024;
  fits on 680M/780M (shared memory), but bandwidth-bound. 512 px cuts this 16x.
- [I] fp16: DirectML supports fp16 tensors on RDNA2/3 (native 16-bit shader ops). Risks: softmax/LayerNorm
  overflow in fp16 attention; keep memory attention fp32 or keep `Softmax`/`LayerNorm` in fp32 (onnxconverter
  `op_block_list`). [C] A Geekbench AI DirectML run on a 780M showed fp16 ≈ fp32 speed for image
  classification (545.9 vs 548.4 IPS) — fp16 is not a guaranteed speed-up on DirectML iGPUs
  (<https://browser.geekbench.com/ai/v1/313079>, user upload).
- No public report of these specific video graphs on DirectML was found [C: searched]. Every graph must pass
  our per-adapter self-test against CPU (plan item 5) before use.

### 1.5 Recommendation and our own export

1. **Now:** pin `jax-image-tools/sam21-tiny-video-onnx` @ `3cb090d9…` (best validated, opset 17, Apache-2.0)
   and **mirror the four files + constants.json into a ScanCraft GitHub release** with LICENSE + a NOTICE that
   says what was changed (Apache-2.0 §4). Primary download = our release; fallback = the pinned HF URL; both
   checked against the same SHA-256. Use square-zero-labs as the second reference (static shapes = closest to
   what DirectML wants).
2. **Next (recommended to own the pipeline): `tools/ai/export_sam2_video.py`**, modelled on Diffusion Studio's
   script (structure only; write our own code — their repo is MPL-2.0 [C]):
   - Pins: Python >= 3.10, torch, `transformers` with `Sam2VideoModel` (Diffusion Studio used torch 2.14.0,
     transformers 5.17.0, onnx 1.23.0, onnxslim 0.1.96 [C]); checkpoint `facebook/sam2.1-hiera-tiny` /
     `-small` at a fixed HF commit, SHA-256 checked.
   - Arguments: model size, `image_size` (1024 and **512**), `memory_frames` (7, also 4 for iGPUs), output dir.
   - `attn_implementation="eager"`, TorchScript exporter (`dynamo=False`), **opset 17**.
   - Five wrappers: `vision_encoder` (pixel_values 1x3xSxS -> feats0/1/2, feats2_no_mem, vision_pos_embed,
     backbone PE precomputed so no `If` nodes), `mask_decoder` (feats + points/labels with dynamic `num_points`
     -> low/high-res mask, iou, object_score_logits, **object_pointer**), `memory_encoder` (feats2, high-res
     mask, score, `binarize` -> memory tokens, memory_pos), `memory_attention` (**static** rows =
     memory_frames*f*f + 16*4, real-valued RoPE with feat_sizes [f,f]), `pointer_tpos`.
   - Post-processing: constant folding, de-duplicate RoPE tables, onnxslim, `onnx.checker`; fp16 variant with
     fp32 I/O (Cast at boundaries), Resize scales kept fp32, Softmax/LayerNorm optionally fp32.
   - `constants.json`: image size, mean/std, memory temporal PE (7x64), no-object pointer, shapes, opset.
   - Validation gate (`validate_sam2_video.py`): run PyTorch `propagate_in_video_iterator` and the ONNX graphs on
     a fixed clip (include a real ScanCraft colour sequence), require worst per-frame IoU >= 0.99 on CPU EP **and**
     on DirectML (fp32 and fp16), record max |Δlogit|.
   - Publish: GitHub release asset per variant + `manifest.json` (file, bytes, SHA-256, opset, precision,
     image size, source checkpoint hash, exporter versions).

---

## 2. ONNX Runtime + DirectML delivery

### 2.1 State of things (important, new)

- [C] **`Microsoft.ML.OnnxRuntime.DirectML` stops at 1.24.4** (published 2026-03-17). The main
  `Microsoft.ML.OnnxRuntime` package continues: 1.25 … 1.31.1 (2026-10-28), **1.32.0 (2026-11-13)**, all without
  the DirectML EP. PyPI `onnxruntime-directml` also ends at 1.24.4. (NuGet API
  `https://api.nuget.org/v3-flatcontainer/microsoft.ml.onnxruntime.directml/index.json`, PyPI JSON API.)
  Revo Scan 6's ORT 1.24 + DirectML is exactly this last line.
- [C] Microsoft: "DirectML is in sustained engineering … new feature development has moved to Windows ML"
  (<https://learn.microsoft.com/windows/ai/directml/dml>). Windows ML WebGPU page: DirectML is in **maintenance
  mode** — critical fixes and security updates only, no new ops/optimisations; recommended production path is
  Windows ML with vendor EPs, **DirectML as the fallback for broad compatibility**
  (<https://learn.microsoft.com/windows/ai/new-windows-ml/webgpu-ep>).
- [C] **`github.com/microsoft/onnxruntime` returns HTTP 404** (checked 2026-11-24 with the web and the GitHub API;
  press reports say since about 2026-11-17/18, reason not stated: <https://byteiota.com/?p=27347>,
  <https://zeli.app/es/story/50539610>). GitHub release zips (`onnxruntime-win-x64-*.zip`) cannot be fetched
  right now. NuGet and PyPI are unaffected. → **Use nuget.org, never GitHub releases.**

### 2.2 What to fetch on first install (Windows x64)

| Package (nuget.org) | Version / date | .nupkg size | Files we extract | SHA-512 (NuGet catalog) |
| --- | --- | --- | --- | --- |
| `Microsoft.ML.OnnxRuntime.DirectML` | **1.24.4**, 2026-03-17 | 12 458 649 B | `runtimes/win-x64/native/onnxruntime.dll` (17 328 152 B), `onnxruntime_providers_shared.dll` (22 040 B), `LICENSE` (MIT), `ThirdPartyNotices.txt` | `ZjOyv495vhfVXoTJp2utRyn8ir1TFIvCj0B9JP8QZGBXS0oFx+lY7ZCZqSdnVShQX6EtxYi2T3VOtYXcmBTV4A==` |
| `Microsoft.AI.DirectML` (dependency, `[1.15.4,)`) | **1.15.4**, 2024-10-28 | **202 292 617 B** (all arches + PDBs) | `bin/x64-win/DirectML.dll` (18 527 776 B), `LICENSE.txt`, `ThirdPartyNotices.txt` | `/edn9WkEq8kP1T9l2HKckYq39uPF4ezdR5kI/AK0U1zysIYPerKsubcx1suAm3LD1dTQKFP7j16gIqR7xE7yhQ==` |
| optional: `Microsoft.ML.OnnxRuntime.EP.WebGpu` | **0.6.0**, 2026-11-16 | 55 622 619 B | `runtimes/win-x64/native/onnxruntime_providers_webgpu.dll` (14 026 040 B), `dxcompiler.dll` (17 986 360 B), `D3D12/D3D12Core.dll` + `d3d12SDKLayers.dll` (~11.5 MB); `linux-x64/libonnxruntime_providers_webgpu.so` (17 387 560 B) | `CmJq9btUPdnN9+8AN8l1B4sH8yHsHEo4XWOVxtZBPLiWxAilOdxO7uZZ2wuyEH5/EUHAvE7usN1e9Jc9eBhqFg==` |

URLs [C] (immutable; a NuGet version can be unlisted but never replaced):
- `https://api.nuget.org/v3-flatcontainer/microsoft.ml.onnxruntime.directml/1.24.4/microsoft.ml.onnxruntime.directml.1.24.4.nupkg`
- `https://api.nuget.org/v3-flatcontainer/microsoft.ai.directml/1.15.4/microsoft.ai.directml.1.15.4.nupkg`
- `https://api.nuget.org/v3-flatcontainer/microsoft.ml.onnxruntime.ep.webgpu/0.6.0/microsoft.ml.onnxruntime.ep.webgpu.0.6.0.nupkg`
- Catalog entries (hash, file list): `https://api.nuget.org/v3/catalog0/data/2026.03.17.21.54.27/microsoft.ml.onnxruntime.directml.1.24.4.json`,
  `https://api.nuget.org/v3/catalog0/data/2024.10.28.16.43.22/microsoft.ai.directml.1.15.4.json`,
  `https://api.nuget.org/v3/catalog0/data/2026.11.16.22.47.21/microsoft.ml.onnxruntime.ep.webgpu.0.6.0.json`.

Download volume: ~215 MB of packages for ~36 MB of DLLs (the DirectML package is huge). Mitigation [I]: an HTTP
Range read of the zip central directory + just the `bin/x64-win/DirectML.dll` entry (nuget.org supports range
requests [I, test]); or rely on the plain download once, behind the consent dialog that shows the size.

Alternative bundle [C]: `Microsoft.Windows.AI.MachineLearning` **2.5.96** (2026-10-26, 52 171 134 B nupkg) contains
`onnxruntime.dll` (**ONNX Runtime 1.28.3**, 23.6 MB), `DirectML.dll` (18.7 MB), `Microsoft.Windows.AI.MachineLearning.dll`,
`WinMLEpCatalog.h` (a C header for the EP catalog), min OS Windows 10 19H1. This is the supported successor (newer ORT
with DirectML, plus on-demand vendor EPs like `AMDGPUExecutionProvider` on Windows 11 24H2+). Licence is a
Microsoft licence (`license.txt`), not MIT. [I] Its `onnxruntime.dll` exports the normal C API and should load
with `ort` `load-dynamic` + `api-28`; the EP catalog needs FFI to `WinMLEpCatalog.h`. **Worth a spike, not the
first choice**, because its licence and packaging (WinAppSDK) need a separate review.

### 2.3 `ort` crate configuration

[C] Latest `ort` is **2.0.0-rc.15** (2026-10-19, "wrapper for ONNX Runtime 1.31"); rc.14 targets 1.31.0, rc.13 1.28,
rc.12 1.17–1.24 (<https://github.com/pykeio/ort/releases>, <https://docs.rs/crate/ort/latest>).
[C] From the rc.15 sources: `ORT_API_VERSION = 17 + number of enabled api-18..api-31 features`
(`ort-sys/src/version.rs`); defaults enable `api-31`, `download-binaries`, `copy-dylibs`. With `load-dynamic` the
loader reads the DLL's version string and **fails if its minor version is lower than `ORT_API_VERSION`**, logs a
warning if higher (`ort/src/lib.rs` `load_dynamic`). `load-dynamic` enables `ort-sys/disable-linking` and
`preload-dylibs`, downloads nothing at build time. So for the 1.24.4 DirectML build:

```toml
ort = { version = "=2.0.0-rc.15", default-features = false, features = ["std", "ndarray", "tracing", "load-dynamic", "directml", "api-24"] }
```

- Load: `ort::init_from(<ai dir>/onnxruntime.dll)` (rc.11+: loads immediately so errors are catchable [C]);
  `ort::util::probe` (rc.14+) checks the version first [C]. Put `DirectML.dll` next to `onnxruntime.dll` and
  preload it from that path (`preload-dylibs`) so the System32 copy is not picked [I].
- DirectML EP: `ort::ep::DirectML` (uses `SessionOptionsAppendExecutionProvider_DML`/`_DML2`; rc.13 exposed
  `OrtDmlApi`) [C]. Session options: disable mem pattern, sequential execution [C, ORT docs].
- WebGPU plugin EP: `Environment::register_ep_library("webgpu", path)` + `SessionBuilder::with_devices(...)`
  — present in rc.15 behind `api-22` [C, `ort/src/environment.rs`]. The plugin needs ORT >= 1.24.4 [C, NuGet README],
  so **one ORT build (1.24.4) serves DirectML and WebGPU**.
- Do not enable `download-binaries`/`copy-dylibs`/`cuda`/`tensorrt` (CI check). Note pyke's CDN has its own
  1.31.0 `+directml` and `+webgpu` builds (`ort-sys/build/download/dist.tsv` [C]) but they are for static linking at
  build time, not a runtime download — not what we want.

### 2.4 Verification

- [C] NuGet packages carry a Microsoft author + repository signature (`.signature.p7s` in the package) and the
  catalog publishes SHA-512. [I] Plan: hard-code SHA-256 of each `.nupkg` (computed once when pinning) and of each
  extracted DLL; additionally check the Authenticode signature of `onnxruntime.dll`/`DirectML.dll`
  (`WinVerifyTrust`, signer "Microsoft Corporation") before loading. Models: SHA-256 from §1.2.

### 2.5 Licences of the runtime pieces

- ONNX Runtime: MIT (`LICENSE`, 1094 B, in each package) + `ThirdPartyNotices.txt` (~331–341 KB) — keep both next
  to the DLLs [C file list; MIT terms].
- DirectML 1.15.4 [C] (<https://www.nuget.org/packages/Microsoft.AI.DirectML/1.15.4/License>): Microsoft software
  licence; may be distributed **as part of applications** built with ML frameworks on Windows/Xbox; **no standalone
  distribution**; Windows/Xbox only; may collect telemetry (some opt-outs). [I] Our app downloading Microsoft's own
  package from nuget.org on the user's request is not us redistributing it; if we ever bundle it, it must be inside
  the app installer, not as a separate download from our servers.
- WebGPU plugin EP via NuGet: `LICENSE` 1094 B = ORT MIT [I: same size as ORT's MIT file]; the **Windows ML**
  WebGPU EP (MSIX) has its own Microsoft licence [C, webgpu-ep page]. Bundled Dawn/DXC/Agility SDK notices are in its
  `ThirdPartyNotices.txt`.

### 2.6 What Microsoft recommends now, and the alternatives

- [C] Windows ML (<https://learn.microsoft.com/windows/ai/new-windows-ml/supported-execution-providers>, updated
  2026-11-20): included EPs = CPU, **DirectML (legacy)**, WinML CG EP (experimental). Downloadable on Windows 11 24H2+:
  **`AMDGPUExecutionProvider`** (AMD, needs Adrenalin 24.12.1+, licence = Ryzen AI terms), VitisAI (AMD NPU),
  MIGraphX (**deprecated**, RDNA3+), OpenVINO, QNN, NvTensorRtRtx, WebGPU (experimental).
- [C] AMD's plugin EP source: <https://github.com/onnxruntime/onnxruntime-ep-amdgpu> (MIT; plugin EP ABI, ORT >= 1.24;
  backends AMDGPU/ROCm, DirectML, MIGraphX). Which iGPUs it covers is **not documented** [C: not stated]; [I] the
  iGPU path will likely be DirectML underneath anyway.
- [C] WebGPU EP: Dawn over D3D12 (or Vulkan) on Windows, Vulkan on Linux, Metal on macOS; plugin library
  (`onnxruntime_providers_webgpu.{dll,so}`), NuGet `Microsoft.ML.OnnxRuntime.EP.WebGpu` / PyPI
  `onnxruntime-ep-webgpu` 0.6.0 (win_amd64 wheel 18.1 MB, manylinux x86_64 7.2 MB); fp32/fp16; graph capture for
  static shapes; Linux needs `libvulkan.so.1`
  (<https://onnxruntime.ai/docs/execution-providers/WebGPU-ExecutionProvider.html>). Microsoft calls it experimental,
  "not yet recommended for performance-critical production".
- **Usable from Rust?** Yes [C for the API: `register_ep_library`, `with_devices` in ort rc.15]; [I] not yet tried with
  the NuGet plugin. Covers AMD iGPUs on Windows (any DX12 GPU; [C] third-party test: Radeon 780M passed 164/164 gates
  with the WebGPU EP over D3D12, 2.4x faster than CPU on PP-OCR fp32 —
  <https://raw.githubusercontent.com/arcships/light-ocr/main/docs/linux-device-acceleration.md>). On Linux with AMD
  (RADV/AMDVLK Vulkan) no measurement found [C: none found]; it is the **most plausible Linux path for AMD iGPUs**
  [I], since ROCm/MIGraphX do not officially support 680M/780M [I].

**Decision proposal:** backend order **DirectML (ORT 1.24.4) → WebGPU plugin EP (same ORT) → CPU → geometric**,
picked per adapter by the self-test; WebGPU becomes the Linux GPU path. Keep a spike on Windows ML 2.x
(ORT 1.28.3 + DirectML + AMDGPU EP) for when DirectML 1.24.4 ages out.

---

## 3. Expected performance on AMD integrated GPUs

No published SAM 2 benchmark on 680M/780M/890M (DirectML or WebGPU) was found [C: searched]. Data points:

| Data point | Source |
| --- | --- |
| Revo Scan 6 (SAM 2 video, ORT 1.24 + DirectML), RX 6900 XT: 48–105 ms/frame | our analysis, `docs/architecture/merge-and-ai.md` §2 |
| SAM 2.1 tiny 38.9 M params, 91.2 FPS; small 46 M, 84.8 FPS (A100, torch, compiled) | [C] <https://github.com/facebookresearch/sam2> |
| Full tracking loop, Apple M1 8-core GPU, ORT-web WebGPU fp16: tiny 1024 px 1.3 s/frame, tiny 512 px 0.25 s, small 1024 px 1.9 s | [C] diffusionstudio model cards |
| SAM 2 tiny image encoder RTX 3080 Ti: CUDA 65 ms, WebGPU 135 ms | [C, search excerpt] osam on PyPI <https://pypi.org/project/osam/0.11.8/> |
| Radeon 780M WebGPU EP over D3D12: 2.4x vs CPU (OCR, fp32) | [C] light-ocr doc above |
| Radeon 780M DirectML (Geekbench AI): fp16 not faster than fp32 in one run | [C] Geekbench browser (user upload) |

Estimate [I] (peak FP32: 6900 XT ~23 TFLOPS, 512 GB/s; 780M 12 CU RDNA3 ~4–8.6 TFLOPS, ~90–120 GB/s shared LPDDR5(X);
680M 12 CU RDNA2 ~3.4 TFLOPS, ~100 GB/s; 890M 16 CU RDNA3.5 ~1.3x 780M; the M1 GPU is ~2.6 TFLOPS):

| GPU | tiny @1024, 7 memories | tiny @512 | small @1024 |
| --- | --- | --- | --- |
| RX 6900 XT (reference) | ~50–100 ms (Revo) | — | ~70–140 ms |
| Radeon 890M | ~200–400 ms | ~50–100 ms | ~300–600 ms |
| Radeon 780M | **~250–500 ms** | **~60–120 ms** | ~350–750 ms |
| Radeon 680M | **~500–1000 ms** | ~120–250 ms | ~0.7–1.5 s |

Consequences [I]:
- Live tracking on a 780M at 1024 px ≈ 2–4 fps — acceptable if we feed only every 2nd–3rd colour frame and show
  the last mask; the post-scan "Segment object" pass over stored keyframes has no latency limit.
- Offer **tiny @512 with 4 memories** as the iGPU profile (needs our own export, §1.5); keep tiny @1024 as default
  where the self-test measures < 150 ms.
- Memory: ~0.2 GB weights (fp32) + up to ~0.5 GB per cross-attention score matrix at 1024/7 memories → ~1–1.5 GB;
  APUs take it from shared RAM (fine on 16 GB laptops, tight on 8 GB). fp16 halves it.
- fp16 on DirectML: supported on RDNA2/RDNA3 [I], speed gain not guaranteed [C, Geekbench]; ship fp32 first, fp16 only
  after the self-test shows IoU parity and a speed gain on that adapter.
- First DirectML session creation compiles shaders (seconds) [I]; create sessions in the background when the user
  enables AI segmentation.

---

## 4. Licence summary

| Item | Licence | Obligations |
| --- | --- | --- |
| SAM 2 / 2.1 code (facebookresearch/sam2) | **Apache-2.0** (`LICENSE`); optional GPU connected-components code under BSD-3 (`LICENSE_cctorch`) — we do not use it | [C] <https://github.com/facebookresearch/sam2>; root has **no NOTICE file** [C, GitHub API listing] |
| SAM 2.1 checkpoints (`sam2.1_hiera_*.pt`, HF `facebook/sam2.1-hiera-*`) | **Apache-2.0** | [C] HF API `license: apache-2.0`; README says checkpoints are Apache-2.0 |
| jax-image-tools, square-zero-labs, diffusionstudio, goblin exports | Apache-2.0 (inherited) | Redistribute with a copy of Apache-2.0, keep attribution, **state that the files were modified** (converted to ONNX, fp16, etc.) [Apache §4] |
| ailia SAM 2.1 ONNX files | Apache-2.0 (folder LICENSE = SAM 2's); ailia SDK commercial but not required | [C] ailia-models LICENSE.md: each model's licence comes from its source |
| ailia export fork (ailia-ai/segment-anything-2) | Apache-2.0 + BSD-3 | [C] GitHub API |
| Diffusion Studio export script | repo MPL-2.0 (`diffusionstudio/editor`) | [C] GitHub API; if we copy code, MPL-2.0 file-level copyleft applies to those files — write our own |
| samexporter | MIT | [C] |
| ONNX Runtime (incl. DirectML EP, WebGPU plugin from NuGet) | MIT + ThirdPartyNotices | ship `LICENSE` + `ThirdPartyNotices.txt` with the DLLs |
| DirectML.dll 1.15.4 | Microsoft software licence (redistributable inside an app, Windows only, telemetry clause) | [C] NuGet licence page; show in the consent dialog |
| Windows ML 2.x package | Microsoft licence (`license.txt`) | review before use |
| Revo's encrypted models | proprietary | **not used** (project decision) |

Suggested `NOTICE` for our model release [I]:
"ScanCraft SAM 2.1 ONNX models. Converted from Meta's Segment Anything 2.1 checkpoints
(https://github.com/facebookresearch/sam2), Copyright (c) Meta Platforms, Inc. and affiliates, licensed under
Apache-2.0. Modified: exported to ONNX (opset 17), RoPE rewritten with real-valued ops, [fp16 conversion],
[input size 512]. [Based on / mirrored from <export repo> @ <commit>.]"

---

## 5. Open questions (need tests on hardware)

1. Do the jax-image-tools / square-zero graphs run fully on DirectML 1.24.4 (no CPU fallback) and match CPU (IoU >= 0.99)?
   Which nodes fall back?
2. Real ms per graph on a Radeon 680M / 780M / 890M, DirectML vs WebGPU plugin EP, fp32 vs fp16, 1024 vs 512, 7 vs 4 memories.
3. Does the WebGPU plugin EP 0.6.0 load into Microsoft's ORT 1.24.4 DirectML build (it requires >= 1.24.4) via `ort` rc.15
   `register_ep_library`, and does its D3D12 Agility path need Windows Developer Mode on normal PCs?
4. Does `onnxruntime.dll` from `Microsoft.Windows.AI.MachineLearning` 2.5.96 (ORT 1.28.3) load with `ort` `load-dynamic`
   (`api-28`), and is its licence acceptable for a first-install download? Does `AMDGPUExecutionProvider` cover iGPUs?
5. Quality of SAM 2.1 tiny at 512 px on our colour camera (resolution, texture-poor parts) vs 1024 px.
6. Can we fetch only the x64 `DirectML.dll` out of the 202 MB `Microsoft.AI.DirectML` package with HTTP range requests?
7. Status of `github.com/microsoft/onnxruntime` (404 since ~2026-11-17): temporary takedown or move? Affects links, issues,
   and any future GitHub-release-based download.
8. Linux: AMD iGPU through the WebGPU plugin EP on Vulkan (RADV) — untested anywhere we found.
