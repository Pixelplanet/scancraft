#!/usr/bin/env python3
"""
Accuracy gate and sequence validator for exported SAM 2.1 Tiny ONNX graphs.

Modes:
  1. PyTorch reference:
     Runs Meta `Sam2VideoModel` (at the target image_size and num_maskmem) over a sequence
     and writes ground-truth binary mask PNGs (`frame_00000.png`, ...).
  2. Compare masks directory (e.g. from Rust `segment_seq --save-masks`) against PyTorch
     ground-truth PNGs, computing per-frame, mean, and worst IoU and checking `worst >= 0.99`.
  3. Quick self-check using Python ORT CPU EP against PyTorch on a sequence.

License: Apache-2.0
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
from pathlib import Path

import numpy as np
from PIL import Image
import onnxruntime as ort
ort.set_default_logger_severity(3)
import torch
from transformers import Sam2VideoProcessor

# Reuse loader & helpers from sibling export script
sys.path.insert(0, str(Path(__file__).resolve().parent))
import export_sam2_video as exp  # noqa: E402


def list_frames(seq_dir: Path) -> list[Path]:
    exts = {".jpg", ".jpeg", ".png", ".bmp"}
    frames = [p for p in sorted(seq_dir.iterdir()) if p.is_file() and p.suffix.lower() in exts]
    if not frames:
        raise FileNotFoundError(f"No image frames found in {seq_dir}")
    return frames


def load_sequence_pils(seq_dir: Path, max_frames: int | None = None) -> list[Image.Image]:
    paths = list_frames(seq_dir)
    if max_frames is not None:
        paths = paths[:max_frames]
    return [Image.open(p).convert("RGB") for p in paths]


def binary_iou(a: np.ndarray, b: np.ndarray) -> float:
    a_bool = a > 0
    b_bool = b > 0
    inter = int(np.logical_and(a_bool, b_bool).sum())
    union = int(np.logical_or(a_bool, b_bool).sum())
    if union == 0:
        return 1.0
    return inter / union


def save_mask_png(mask_bool: np.ndarray, out_path: Path):
    arr = (mask_bool.astype(np.uint8)) * 255
    Image.fromarray(arr, mode="L").save(out_path)


def run_pytorch_sequence(
    checkpoint_dir: Path,
    seq_dir: Path,
    point_xy: tuple[float, float],
    image_size: int,
    num_maskmem: int,
    max_pointers: int,
    max_frames: int | None = None,
) -> list[np.ndarray]:
    """Run HuggingFace Sam2VideoModel in streaming mode, mirroring our exact exported memory budget."""
    images = load_sequence_pils(seq_dir, max_frames=max_frames)
    model, _ = exp.load_sam2_for_size(checkpoint_dir, image_size, num_maskmem, max_pointers)
    processor = Sam2VideoProcessor.from_pretrained(str(checkpoint_dir))
    processor.image_processor.size = {"height": image_size, "width": image_size}
    if hasattr(processor, "video_processor") and processor.video_processor is not None:
        processor.video_processor.size = {"height": image_size, "width": image_size}
    if hasattr(processor, "target_size"):
        processor.target_size = int(image_size)

    # Build a streaming inference session cleanly frame-by-frame
    sess = processor.init_video_session(
        inference_device="cpu",
        dtype=torch.float32,
        max_vision_features_cache_size=1,
    )
    orig_w, orig_h = images[0].size
    # Point coordinates are given in original pixel space [x, y]; scale into processor if needed:
    # Sam2VideoProcessor scales coordinates relative to original_size when original_size is passed!
    masks_out: list[np.ndarray] = []

    with torch.no_grad():
        for idx, pil_img in enumerate(images):
            inputs = processor(images=pil_img, return_tensors="pt")
            if idx == 0:
                processor.add_inputs_to_inference_session(
                    inference_session=sess,
                    frame_idx=0,
                    obj_ids=1,
                    input_points=[[[[point_xy[0], point_xy[1]]]]],
                    input_labels=[[[1]]],
                    original_size=(orig_h, orig_w),
                )
            out = model(
                inference_session=sess,
                frame=inputs.pixel_values[0],
                frame_idx=idx,
            )
            res_masks = processor.post_process_masks(
                [out.pred_masks], original_sizes=[(orig_h, orig_w)], binarize=True
            )[0]
            mask_np = res_masks[0, 0].cpu().numpy().astype(bool)
            masks_out.append(mask_np)
    return masks_out


def preprocess_rgb_exact(pil_img: Image.Image, target_size: int) -> np.ndarray:
    """Matches scancraft-segment preprocess.rs bilinear resize + ImageNet normalization."""
    resized = pil_img.resize((target_size, target_size), Image.Resampling.BILINEAR)
    arr = np.asarray(resized, dtype=np.float32) / 255.0
    mean = np.array([0.485, 0.456, 0.406], dtype=np.float32)
    std = np.array([0.229, 0.224, 0.225], dtype=np.float32)
    norm = (arr - mean) / std
    return np.transpose(norm, (2, 0, 1))[None, ...]  # [1, 3, S, S]


def sine_pe_1d(pos: float, dim: int = 256) -> np.ndarray:
    half = dim // 2
    pe = np.zeros(dim, dtype=np.float32)
    for i in range(half):
        denom = 10000.0 ** (2.0 * (i // 2) / half)
        angle = pos / denom
        pe[i] = math.sin(angle)
        pe[half + i] = math.cos(angle)
    return pe


def run_onnx_python_sequence(
    model_dir: Path,
    seq_dir: Path,
    point_xy: tuple[float, float],
    max_frames: int | None = None,
) -> list[np.ndarray]:
    """Quick CPU-ORT reference runner matching Rust memory_bank.rs logic."""
    constants = json.loads((model_dir / "constants.json").read_text(encoding="utf-8"))
    s = int(constants["image_size"])
    hw = int(constants["spatial_tokens_per_memory_block"])
    s16 = s // 16
    num_maskmem = int(constants["num_maskmem"])
    max_ptrs = int(constants["max_object_pointers_in_encoder"])
    tpp = int(constants["tokens_per_pointer"])
    static_blocks = constants.get("static_memory_blocks")
    static_ptrs = constants.get("static_pointer_tokens")
    has_mask = bool(constants.get("has_memory_attn_mask", False))

    no_mem_embed = np.array(constants["no_memory_embedding"], dtype=np.float32).reshape(1, 256, 1, 1)
    tpos_pe = np.array(constants["memory_temporal_positional_encoding"], dtype=np.float32).reshape(num_maskmem, 1, 1, 64)
    ptr_w = np.array(constants["pointer_tpos_proj_weight"], dtype=np.float32).reshape(64, 256)
    ptr_b = np.array(constants["pointer_tpos_proj_bias"], dtype=np.float32).reshape(64)

    ve = ort.InferenceSession(str(model_dir / "vision_encoder.onnx"), providers=["CPUExecutionProvider"])
    md = ort.InferenceSession(str(model_dir / "mask_decoder.onnx"), providers=["CPUExecutionProvider"])
    ma = ort.InferenceSession(str(model_dir / "memory_attention.onnx"), providers=["CPUExecutionProvider"])
    me = ort.InferenceSession(str(model_dir / "memory_encoder.onnx"), providers=["CPUExecutionProvider"])

    images = load_sequence_pils(seq_dir, max_frames=max_frames)
    orig_w, orig_h = images[0].size
    sx, sy = s / orig_w, s / orig_h

    cond_mems = []
    noncond_mems = []
    ptrs = []  # list of (frame_idx, vec[256])
    masks_out: list[np.ndarray] = []

    for idx, pil_img in enumerate(images):
        px = preprocess_rgb_exact(pil_img, s)
        f0, f1, f2, _p0, _p1, p2 = ve.run(None, {"pixel_values": px})

        if idx == 0:
            dec_f2 = f2 + no_mem_embed
            pts = np.array([[[[point_xy[0] * sx, point_xy[1] * sy]]]], dtype=np.float32)
            lbl = np.array([[[1]]], dtype=np.int32)
            is_pt = True
        else:
            # Assemble memory: cond + up to num_maskmem-1 most recent noncond
            selected_spatial = []
            selected_spatial_pe = []
            for m in cond_mems:
                selected_spatial.append(m[0])
                selected_spatial_pe.append(m[1] + tpos_pe[num_maskmem - 1])
            recent = noncond_mems[-(num_maskmem - 1) :]
            for m in reversed(recent):
                t_diff = idx - m[2]
                pe_slot = t_diff - 1 if 1 <= t_diff <= num_maskmem - 1 else num_maskmem - 2
                selected_spatial.append(m[0])
                selected_spatial_pe.append(m[1] + tpos_pe[pe_slot])

            real_blocks = len(selected_spatial)
            if static_blocks is not None:
                while len(selected_spatial) < static_blocks:
                    selected_spatial.append(np.zeros((hw, 1, 64), dtype=np.float32))
                    selected_spatial_pe.append(np.zeros((hw, 1, 64), dtype=np.float32))
            smem = np.concatenate(selected_spatial, axis=0)
            smem_pe = np.concatenate(selected_spatial_pe, axis=0)

            # Select pointers matching SAM 2: conditioning ptrs + up to max_ptrs-len(cond) recent
            sel_ptrs = []
            for pf, pvec in ptrs:
                if pf == 0:
                    sel_ptrs.append((pf, pvec))
            rem = max_ptrs - len(sel_ptrs)
            noncond_ptrs = [(pf, pvec) for pf, pvec in ptrs if pf != 0]
            if rem > 0 and noncond_ptrs:
                sel_ptrs.extend(noncond_ptrs[-rem:])

            pmem_list = []
            pmem_pe_list = []
            norm_denom = float(max(max_ptrs - 1, 1))
            for pf, pvec in sel_ptrs:
                pmem_list.append(pvec.reshape(tpp, 1, 64))
                td = float(idx - pf) / norm_denom
                proj = ptr_w @ sine_pe_1d(td, 256) + ptr_b
                pmem_pe_list.append(np.tile(proj.reshape(1, 1, 64), (tpp, 1, 1)))
            real_ptr_tokens = len(pmem_list) * tpp
            if static_ptrs is not None:
                pad_ptrs = (static_ptrs // tpp) - len(pmem_list)
                for _ in range(pad_ptrs):
                    pmem_list.append(np.zeros((tpp, 1, 64), dtype=np.float32))
                    pmem_pe_list.append(np.zeros((tpp, 1, 64), dtype=np.float32))
            pmem = np.concatenate(pmem_list, axis=0)
            pmem_pe = np.concatenate(pmem_pe_list, axis=0)

            cur = np.transpose(f2.reshape(1, 256, hw), (2, 0, 1))
            cur_pos = np.transpose(p2.reshape(1, 256, hw), (2, 0, 1))
            feeds = {
                "current_vision_features": cur,
                "current_vision_position_embeddings": cur_pos,
                "spatial_memory": smem,
                "spatial_memory_position_embeddings": smem_pe,
                "pointer_memory": pmem,
                "pointer_memory_position_embeddings": pmem_pe,
            }
            if has_mask:
                total_len = smem.shape[0] + pmem.shape[0]
                attn_mask = np.full((1, 1, 1, total_len), -10000.0, dtype=np.float32)
                attn_mask[0, 0, 0, : real_blocks * hw] = 0.0
                attn_mask[0, 0, 0, smem.shape[0] : smem.shape[0] + real_ptr_tokens] = 0.0
                feeds["memory_attention_mask"] = attn_mask

            cond_flat = ma.run(None, feeds)[0]
            dec_f2 = np.transpose(cond_flat.reshape(1, hw, 256), (0, 2, 1)).reshape(1, 256, s16, s16)
            pts = np.array([[[[0.0, 0.0]]]], dtype=np.float32)
            lbl = np.array([[[-1]]], dtype=np.int32)
            is_pt = False

        _low, hi, _iou, obj_ptr, obj_score = md.run(
            None,
            {
                "feats0": f0,
                "feats1": f1,
                "feats2": dec_f2,
                "input_points": pts,
                "input_labels": lbl,
            },
        )
        # Upsample logits to original size bilinearly, then threshold > 0
        logit_t = torch.from_numpy(hi)
        up = torch.nn.functional.interpolate(
            logit_t, size=(orig_h, orig_w), mode="bilinear", align_corners=False
        )[0, 0].numpy()
        masks_out.append(up > 0.0)

        # Encode memory
        mtok, mpos = me.run(
            None,
            {
                "vision_features": f2,
                "pred_masks_high_res": hi,
                "object_score_logits": obj_score.reshape(1, 1),
                "is_mask_from_pts": np.array([is_pt], dtype=bool),
            },
        )
        if idx == 0:
            cond_mems.append((mtok, mpos, idx))
        else:
            noncond_mems.append((mtok, mpos, idx))
            if len(noncond_mems) > (num_maskmem - 1):
                noncond_mems.pop(0)
        ptrs.append((idx, obj_ptr.reshape(256)))

    return masks_out


def load_masks_dir(mask_dir: Path, count: int) -> list[np.ndarray]:
    pngs = sorted(p for p in mask_dir.iterdir() if p.suffix.lower() == ".png" and not p.name.endswith("_overlay.png"))
    if len(pngs) < count:
        # Also support mask_00000.png naming if saved alongside overlays
        raise FileNotFoundError(f"Expected >= {count} mask PNGs in {mask_dir}, found {len(pngs)}")
    return [(np.asarray(Image.open(p).convert("L")) > 127) for p in pngs[:count]]


def summarize_iou(label: str, ref_masks: list[np.ndarray], cand_masks: list[np.ndarray], threshold: float):
    ious = [binary_iou(r, c) for r, c in zip(ref_masks, cand_masks)]
    worst = min(ious)
    mean = sum(ious) / len(ious)
    worst_idx = ious.index(worst)
    passed = worst >= threshold
    status = "PASS" if passed else "FAIL"
    print(
        f"[{label}] {status}: frames={len(ious)} mean_IoU={mean:.5f} "
        f"worst_IoU={worst:.5f} (frame {worst_idx}, gate>={threshold:.2f})"
    )
    return passed, worst, mean, ious


def main():
    p = argparse.ArgumentParser(description="Validate SAM 2.1 ONNX masks against PyTorch.")
    p.add_argument("--checkpoint", type=Path, required=True)
    p.add_argument("--seq-dir", type=Path, required=True)
    p.add_argument("--point", type=str, required=True, help="x,y prompt on frame 0 in original pixels")
    p.add_argument("--image-size", type=int, default=512)
    p.add_argument("--num-maskmem", type=int, default=4)
    p.add_argument("--max-pointers", type=int, default=8)
    p.add_argument("--max-frames", type=int, default=None)
    p.add_argument("--onnx-dir", type=Path, default=None, help="Run Python ORT CPU check on this model dir.")
    p.add_argument("--cand-masks", type=Path, default=None, help="Compare precomputed mask PNGs in this dir.")
    p.add_argument("--save-ref-masks", type=Path, default=None, help="Save PyTorch mask PNGs here.")
    p.add_argument("--ref-masks", type=Path, default=None, help="Load existing PyTorch ref PNGs instead of re-running PyTorch.")
    p.add_argument("--gate", type=float, default=0.99)
    args = p.parse_args()

    px, py = [float(v.strip()) for v in args.point.split(",")]

    if args.ref_masks is not None and args.ref_masks.exists():
        n_frames = len(list_frames(args.seq_dir)[: args.max_frames])
        ref_masks = load_masks_dir(args.ref_masks, n_frames)
    else:
        ref_masks = run_pytorch_sequence(
            args.checkpoint,
            args.seq_dir,
            (px, py),
            args.image_size,
            args.num_maskmem,
            args.max_pointers,
            max_frames=args.max_frames,
        )
        if args.save_ref_masks is not None:
            args.save_ref_masks.mkdir(parents=True, exist_ok=True)
            for i, m in enumerate(ref_masks):
                save_mask_png(m, args.save_ref_masks / f"frame_{i:05d}.png")

    all_passed = True
    if args.onnx_dir is not None:
        onnx_masks = run_onnx_python_sequence(
            args.onnx_dir, args.seq_dir, (px, py), max_frames=args.max_frames
        )
        ok, _, _, _ = summarize_iou(f"ORT-CPU:{args.onnx_dir.name}", ref_masks, onnx_masks, args.gate)
        all_passed = all_passed and ok

    if args.cand_masks is not None:
        cand_masks = load_masks_dir(args.cand_masks, len(ref_masks))
        ok, _, _, _ = summarize_iou(f"CAND:{args.cand_masks.name}", ref_masks, cand_masks, args.gate)
        all_passed = all_passed and ok

    if not all_passed:
        sys.exit(1)


if __name__ == "__main__":
    main()
