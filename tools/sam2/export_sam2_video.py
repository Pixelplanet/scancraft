#!/usr/bin/env python3
"""
Export SAM 2.1 Hiera Tiny video segmentation into four DirectML-friendly ONNX graphs:
  1. vision_encoder.onnx
  2. mask_decoder.onnx
  3. memory_attention.onnx
  4. memory_encoder.onnx

Also emits constants.json, manifest.json (with SHA-256 checksums), LICENSE, and NOTICE
so the exported variant can be verified and loaded directly by `scancraft-segment`.

License: Apache-2.0
"""

from __future__ import annotations

import argparse
import contextlib
import hashlib
import json
import math
import os
import shutil
import sys
from pathlib import Path

import numpy as np
import onnx
import onnxruntime as ort
from onnxruntime.transformers import float16 as onnx_float16
import torch
import torch.nn as nn
import torch.nn.functional as F
from transformers import Sam2VideoConfig, Sam2VideoModel
from transformers.modeling_utils import ALL_ATTENTION_FUNCTIONS
from transformers.models.sam2_video import modeling_sam2_video as sam2_mod

DEFAULT_CHECKPOINT_COMMIT = "de431c4043854a71d8101e17995dfe596bf101a5"
DEFAULT_HF_REPO = "facebook/sam2.1-hiera-tiny"

DEFAULT_FP16_BLOCK_LIST = [
    "Softmax",
    "LayerNorm",
    "LayerNormalization",
    "SimplifiedLayerNormalization",
    "Resize",
    "Range",
    "NonZero",
    "ArgMax",
    "Where",
    "OneHot",
    "CumSum",
    "ReduceMean",
    "Pow",
    "Sqrt",
    "Div",
    "Sin",
    "Cos",
]

APACHE_LICENSE_URL = "https://www.apache.org/licenses/LICENSE-2.0"

NOTICE_TEXT = """ScanCraft AI Segmentation Pack (SAM 2.1 Tiny Video ONNX)

Upstream Model:
  facebook/sam2.1-hiera-tiny (Segment Anything 2.1)
  Copyright (c) Meta Platforms, Inc. and affiliates.
  Licensed under the Apache License, Version 2.0 (Apache-2.0).

Export Tooling:
  Exported from upstream Safetensors checkpoint into ONNX using ScanCraft's
  tools/sam2/export_sam2_video.py (Apache-2.0).

Optional Windows GPU Acceleration:
  When DirectML GPU execution is enabled on Windows, ONNX Runtime uses DirectML.dll
  (Microsoft DirectML Redistributable). Review Microsoft's DirectML license and
  telemetry notices before enabling GPU acceleration.
"""


def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        while True:
            chunk = f.read(1024 * 1024)
            if not chunk:
                break
            h.update(chunk)
    return h.hexdigest()


class VisionEncoderWrapper(nn.Module):
    def __init__(self, model: Sam2VideoModel, image_size: int):
        super().__init__()
        self.vision_encoder = model.vision_encoder
        self.conv_s0 = model.mask_decoder.conv_s0
        self.conv_s1 = model.mask_decoder.conv_s1
        s4 = int(image_size) // 4
        with torch.no_grad():
            cached_pe = model.vision_encoder.backbone._get_pos_embed((s4, s4)).detach().clone()
        self.register_buffer("cached_pos_embed", cached_pe)

    def forward(self, pixel_values: torch.Tensor):
        backbone = self.vision_encoder.backbone
        orig_get_pe = backbone._get_pos_embed
        backbone._get_pos_embed = lambda hw: self.cached_pos_embed
        try:
            out = self.vision_encoder(pixel_values)
        finally:
            backbone._get_pos_embed = orig_get_pe
        feats = out.fpn_hidden_states
        pos = out.fpn_position_encoding
        feats0 = self.conv_s0(feats[0])
        feats1 = self.conv_s1(feats[1])
        feats2 = feats[2]
        return feats0, feats1, feats2, pos[0], pos[1], pos[2]


class MaskDecoderWrapper(nn.Module):
    def __init__(self, model: Sam2VideoModel, image_size: int):
        super().__init__()
        self.prompt_encoder = model.prompt_encoder
        self.mask_decoder = model.mask_decoder
        self.object_pointer_proj = model.object_pointer_proj
        self.register_buffer("no_object_pointer", model.no_object_pointer.detach().clone())
        self.image_size = int(image_size)
        dense_pe = model.get_image_wide_positional_embeddings().detach().clone()
        self.register_buffer("image_pe", dense_pe)

    def forward(
        self,
        feats0: torch.Tensor,
        feats1: torch.Tensor,
        feats2: torch.Tensor,
        input_points: torch.Tensor,
        input_labels: torch.Tensor,
    ):
        # Call prompt_encoder with pad=True (same as Sam2VideoModel when input_boxes is None)
        # so both click prompts and the empty (-1) tracking token match PyTorch and jax-image-tools.
        sparse_embeddings = self.prompt_encoder._embed_points(
            input_points, input_labels, pad=True
        )
        dense_embeddings = self.prompt_encoder.no_mask_embed.weight.reshape(1, -1, 1, 1).expand(
            1, -1, self.prompt_encoder.image_embedding_size[0], self.prompt_encoder.image_embedding_size[1]
        )

        low_res_multimasks, iou_scores, sam_output_tokens, object_score_logits = self.mask_decoder(
            image_embeddings=feats2,
            image_positional_embeddings=self.image_pe,
            sparse_prompt_embeddings=sparse_embeddings,
            dense_prompt_embeddings=dense_embeddings,
            multimask_output=True,
            high_resolution_features=[feats0, feats1],
        )

        s = self.image_size
        s4 = s // 4
        is_obj_appearing = object_score_logits > 0
        low_res_multimasks = torch.where(
            is_obj_appearing[:, :, None, None],
            low_res_multimasks,
            low_res_multimasks.new_full((), -1024.0),
        )

        high_res_multimasks = (
            F.interpolate(
                low_res_multimasks.reshape(1, 3, s4, s4).float(),
                size=(s, s),
                mode="bilinear",
                align_corners=False,
            )
            .reshape(1, 1, 3, s, s)
            .to(low_res_multimasks.dtype)
        )

        # Select the best of 3 predicted masks using OneHot + sum (DirectML-safe; avoids GatherElements)
        best_iou_inds = torch.argmax(iou_scores, dim=-1)  # [1, 1]
        weights = F.one_hot(best_iou_inds, num_classes=3).to(low_res_multimasks.dtype)  # [1, 1, 3]

        pred_masks = (low_res_multimasks * weights[:, :, :, None, None]).sum(dim=2)  # [1, 1, H/4, W/4]
        high_res_masks = (high_res_multimasks * weights[:, :, :, None, None]).sum(dim=2)  # [1, 1, H, W]
        sam_output_token = (sam_output_tokens * weights[:, :, :, None]).sum(dim=2)  # [1, 1, 256]

        object_pointer = self.object_pointer_proj(sam_output_token)
        lambda_appearing = is_obj_appearing.to(object_pointer.dtype)
        object_pointer = lambda_appearing * object_pointer + (1.0 - lambda_appearing) * self.no_object_pointer.unsqueeze(0)

        return pred_masks, high_res_masks, iou_scores, object_pointer, object_score_logits


class MemoryAttentionWrapper(nn.Module):
    def __init__(
        self,
        model: Sam2VideoModel,
        num_pointer_tokens: int,
        use_static_mask: bool,
    ):
        super().__init__()
        self.memory_attention = model.memory_attention
        self.num_pointer_tokens = int(num_pointer_tokens)
        self.use_static_mask = bool(use_static_mask)

    def forward(
        self,
        current_vision_features: torch.Tensor,
        current_vision_position_embeddings: torch.Tensor,
        spatial_memory: torch.Tensor,
        spatial_memory_position_embeddings: torch.Tensor,
        pointer_memory: torch.Tensor,
        pointer_memory_position_embeddings: torch.Tensor,
        memory_attention_mask: torch.Tensor | None = None,
    ):
        memory = torch.cat([spatial_memory, pointer_memory], dim=0)
        memory_pos = torch.cat(
            [spatial_memory_position_embeddings, pointer_memory_position_embeddings], dim=0
        )

        # Attach additive bias mask to cross-attention modules only (consumed by patched eager_attention)
        if self.use_static_mask:
            for layer in self.memory_attention.layers:
                layer.cross_attn_image._export_attn_mask = memory_attention_mask
        try:
            conditioned = self.memory_attention(
                current_vision_features=current_vision_features,
                memory=memory,
                current_vision_position_embeddings=current_vision_position_embeddings,
                memory_posision_embeddings=memory_pos,
                num_object_pointer_tokens=self.num_pointer_tokens,
            )
        finally:
            for layer in self.memory_attention.layers:
                if hasattr(layer.cross_attn_image, "_export_attn_mask"):
                    delattr(layer.cross_attn_image, "_export_attn_mask")
        return conditioned


@contextlib.contextmanager
def patch_eager_cross_attn_mask():
    """Temporarily teaches eager attention to apply `module._export_attn_mask` when present."""
    orig_mod_fn = sam2_mod.eager_attention_forward
    orig_global_fn = ALL_ATTENTION_FUNCTIONS._global_mapping.get("eager", orig_mod_fn)

    def masked_eager_attention(
        module,
        query,
        key,
        value,
        attention_mask,
        scaling,
        dropout=0.0,
        **kwargs,
    ):
        export_mask = getattr(module, "_export_attn_mask", None)
        if export_mask is not None:
            attention_mask = export_mask if attention_mask is None else (attention_mask + export_mask)
        return orig_mod_fn(
            module,
            query,
            key,
            value,
            attention_mask=attention_mask,
            scaling=scaling,
            dropout=dropout,
            **kwargs,
        )

    sam2_mod.eager_attention_forward = masked_eager_attention
    ALL_ATTENTION_FUNCTIONS._global_mapping["eager"] = masked_eager_attention
    try:
        yield
    finally:
        sam2_mod.eager_attention_forward = orig_mod_fn
        ALL_ATTENTION_FUNCTIONS._global_mapping["eager"] = orig_global_fn


class MemoryEncoderWrapper(nn.Module):
    def __init__(self, model: Sam2VideoModel, mask_mem_size: int):
        super().__init__()
        self.memory_encoder = model.memory_encoder
        self.sigmoid_scale = float(model.config.sigmoid_scale_for_mem_enc)
        self.sigmoid_bias = float(model.config.sigmoid_bias_for_mem_enc)
        self.mask_mem_size = int(mask_mem_size)
        if model.occlusion_spatial_embedding_parameter is not None:
            self.register_buffer(
                "occlusion_embed",
                model.occlusion_spatial_embedding_parameter.detach().clone().view(1, -1, 1, 1),
            )
        else:
            self.occlusion_embed = None

    def forward(
        self,
        vision_features: torch.Tensor,
        pred_masks_high_res: torch.Tensor,
        object_score_logits: torch.Tensor,
        is_mask_from_pts: torch.Tensor,
    ):
        masks_in = pred_masks_high_res
        if masks_in.shape[-2] != self.mask_mem_size or masks_in.shape[-1] != self.mask_mem_size:
            masks_in = F.interpolate(
                masks_in.float(),
                size=(self.mask_mem_size, self.mask_mem_size),
                mode="bilinear",
                align_corners=False,
            ).to(pred_masks_high_res.dtype)

        binary_mask = (masks_in > 0).to(masks_in.dtype)
        prob_mask = torch.sigmoid(masks_in)
        mask_for_mem = torch.where(is_mask_from_pts.view(1, 1, 1, 1), binary_mask, prob_mask)
        mask_for_mem = mask_for_mem * self.sigmoid_scale + self.sigmoid_bias

        mem_feats, mem_pos = self.memory_encoder(vision_features, mask_for_mem)
        if self.occlusion_embed is not None:
            is_appearing = (object_score_logits > 0).to(mem_feats.dtype).view(1, 1, 1, 1)
            mem_feats = mem_feats + (1.0 - is_appearing) * self.occlusion_embed

        mem_tokens = mem_feats.flatten(2).permute(2, 0, 1)
        mem_pos_enc = mem_pos.flatten(2).permute(2, 0, 1)
        return mem_tokens, mem_pos_enc


def audit_onnx_graph(path: Path) -> dict:
    model = onnx.load(str(path), load_external_data=False)
    onnx.checker.check_model(model)
    ops: dict[str, int] = {}
    control_flow_ops = []
    for node in model.graph.node:
        ops[node.op_type] = ops.get(node.op_type, 0) + 1
        if node.op_type in ("If", "Loop", "Scan"):
            control_flow_ops.append(node.op_type)
    if control_flow_ops:
        raise RuntimeError(f"{path.name} contains unsupported control-flow ops: {control_flow_ops}")
    return {
        "nodes": len(model.graph.node),
        "op_counts": dict(sorted(ops.items())),
        "inputs": [
            {
                "name": i.name,
                "shape": [d.dim_value or d.dim_param for d in i.type.tensor_type.shape.dim],
                "dtype": onnx.TensorProto.DataType.Name(i.type.tensor_type.elem_type),
            }
            for i in model.graph.input
        ],
        "outputs": [
            {
                "name": o.name,
                "shape": [d.dim_value or d.dim_param for d in o.type.tensor_type.shape.dim],
                "dtype": onnx.TensorProto.DataType.Name(o.type.tensor_type.elem_type),
            }
            for o in model.graph.output
        ],
    }


def topo_sort_graph(graph: onnx.GraphProto):
    """Topologically sort nodes in an ONNX GraphProto so onnx.checker passes after FP16 cast injection."""
    available = {i.name for i in graph.input}
    available.update(i.name for i in graph.initializer)
    available.add("")
    nodes = list(graph.node)
    ordered = []
    # Kahn-style iterative pass (graph sizes are a few thousand nodes; fast in O(N) via producer map)
    produced_by: dict[str, int] = {}
    for idx, n in enumerate(nodes):
        for out in n.output:
            if out:
                produced_by[out] = idx
    in_deg = [0] * len(nodes)
    dependents: list[list[int]] = [[] for _ in nodes]
    for idx, n in enumerate(nodes):
        seen_parents = set()
        for inp in n.input:
            if inp and inp not in available and inp in produced_by:
                p = produced_by[inp]
                if p not in seen_parents:
                    seen_parents.add(p)
                    in_deg[idx] += 1
                    dependents[p].append(idx)
    queue = [i for i, d in enumerate(in_deg) if d == 0]
    head = 0
    while head < len(queue):
        u = queue[head]
        head += 1
        ordered.append(nodes[u])
        for v in dependents[u]:
            in_deg[v] -= 1
            if in_deg[v] == 0:
                queue.append(v)
    if len(ordered) == len(nodes):
        del graph.node[:]
        graph.node.extend(ordered)


def convert_to_fp16_if_requested(path: Path, fp16_enabled: bool, block_list: list[str]):
    if not fp16_enabled:
        return
    model = onnx.load(str(path), load_external_data=False)
    model = onnx.shape_inference.infer_shapes(model)
    model_fp16 = onnx_float16.convert_float_to_float16(
        model,
        min_positive_val=1e-7,
        max_finite_val=1e4,
        keep_io_types=True,
        disable_shape_infer=True,
        op_block_list=block_list,
    )
    topo_sort_graph(model_fp16.graph)
    onnx.save(model_fp16, str(path))


def export_onnx(
    module: nn.Module,
    args: tuple,
    out_path: Path,
    input_names: list[str],
    output_names: list[str],
    dynamic_axes: dict | None,
    opset: int,
    use_fp16: bool,
    fp16_block_list: list[str],
):
    module.eval()
    with torch.no_grad():
        torch.onnx.export(
            module,
            args,
            str(out_path),
            input_names=input_names,
            output_names=output_names,
            dynamic_axes=dynamic_axes or {},
            opset_version=opset,
            do_constant_folding=True,
            dynamo=False,
        )
    # Collapse any accidental external data into single .onnx file (well under 2 GB)
    model = onnx.load(str(out_path), load_external_data=True)
    onnx.save(model, str(out_path))
    data_sidecar = out_path.with_suffix(".onnx.data")
    if data_sidecar.exists():
        data_sidecar.unlink()

    convert_to_fp16_if_requested(out_path, use_fp16, fp16_block_list)

    # Sanity check ONNX Runtime can load the serialized model on CPU right away
    _ = ort.InferenceSession(str(out_path), providers=["CPUExecutionProvider"])
    return audit_onnx_graph(out_path)


def load_sam2_for_size(checkpoint_dir: Path, image_size: int, num_maskmem: int, max_pointers: int):
    if image_size % 16 != 0:
        raise ValueError(f"image_size ({image_size}) must be a multiple of 16")
    s4 = image_size // 4
    s8 = image_size // 8
    s16 = image_size // 16

    cfg = Sam2VideoConfig.from_pretrained(str(checkpoint_dir))
    # Keep num_maskmem=7 during from_pretrained so temporal PE [7,1,1,64] loads cleanly without shape mismatch.
    cfg.image_size = image_size
    cfg.vision_config.backbone_feature_sizes = [[s4, s4], [s8, s8], [s16, s16]]
    cfg.vision_config.backbone_config.image_size = image_size
    cfg.prompt_encoder_config.image_size = image_size
    cfg.memory_attention_rope_feat_sizes = [s16, s16]
    cfg._attn_implementation = "eager"

    model = Sam2VideoModel.from_pretrained(str(checkpoint_dir), config=cfg)
    model.eval()

    # Slice memory_temporal_positional_encoding [7, 1, 1, 64] -> [M, 1, 1, 64]:
    # index 0..M-2 = recent frames (offset 1..M-1), index M-1 = conditioning frame (offset 0 -> original index -1)
    orig_pe = model.memory_temporal_positional_encoding.detach().clone()
    if num_maskmem < 2 or num_maskmem > orig_pe.shape[0]:
        raise ValueError(f"num_maskmem must be in 2..{orig_pe.shape[0]}, got {num_maskmem}")
    sliced_pe = torch.cat([orig_pe[: num_maskmem - 1], orig_pe[-1:]], dim=0)
    model.memory_temporal_positional_encoding = nn.Parameter(sliced_pe, requires_grad=False)
    model.num_maskmem = num_maskmem
    model.config.num_maskmem = num_maskmem
    model.max_object_pointers_in_encoder = max_pointers
    model.config.max_object_pointers_in_encoder = max_pointers
    return model, cfg


def build_variant_name(
    image_size: int,
    num_maskmem: int,
    fp16_graphs: set[str],
    static_memory: bool,
) -> str:
    if len(fp16_graphs) == 4:
        prec = "fp16"
    elif len(fp16_graphs) == 0:
        prec = "fp32"
    else:
        prec = "mixed"
    mem_tag = f"mem{num_maskmem}" if static_memory else f"dynmem{num_maskmem}"
    return f"sam21-tiny-{image_size}-{mem_tag}-{prec}"


def parse_args():
    default_ckpt = Path(os.path.expandvars(r"%LOCALAPPDATA%\ScanCraft\ai\source-checkpoints\sam2.1-hiera-tiny"))
    p = argparse.ArgumentParser(description="Export SAM 2.1 Tiny Video graphs to ONNX.")
    p.add_argument("--checkpoint", type=Path, default=default_ckpt)
    p.add_argument("--out-dir", type=Path, required=True)
    p.add_argument("--image-size", type=int, default=512)
    p.add_argument("--num-maskmem", type=int, default=4)
    p.add_argument("--max-pointers", type=int, default=8)
    p.add_argument("--opset", type=int, default=17)
    p.add_argument(
        "--dynamic-memory",
        action="store_true",
        help="Use dynamic spatial/pointer lengths instead of fixed static shapes + attention bias mask.",
    )
    p.add_argument(
        "--fp16",
        action="store_true",
        help="Convert all 4 ONNX graphs to FP16 (with FP32 I/O and FP32 block-listed sensitive ops).",
    )
    p.add_argument(
        "--fp16-graphs",
        type=str,
        default="",
        help="Comma-separated subset of graphs to convert to FP16 (vision_encoder,mask_decoder,memory_attention,memory_encoder).",
    )
    p.add_argument("--variant-name", type=str, default="")
    return p.parse_args()


def main():
    args = parse_args()
    static_memory = not args.dynamic_memory
    all_graphs = ["vision_encoder", "mask_decoder", "memory_attention", "memory_encoder"]
    if args.fp16:
        fp16_graphs = set(all_graphs)
    elif args.fp16_graphs.strip():
        fp16_graphs = {g.strip() for g in args.fp16_graphs.split(",") if g.strip()}
        unknown = fp16_graphs - set(all_graphs)
        if unknown:
            raise ValueError(f"Unknown graph names in --fp16-graphs: {unknown}")
    else:
        fp16_graphs = set()

    variant_name = args.variant_name or build_variant_name(
        args.image_size, args.num_maskmem, fp16_graphs, static_memory
    )

    out_dir: Path = args.out_dir
    out_dir.mkdir(parents=True, exist_ok=True)

    print(
        f"[export] Loading {args.checkpoint} (image_size={args.image_size}, "
        f"num_maskmem={args.num_maskmem}, max_pointers={args.max_pointers}, "
        f"static_memory={static_memory}, fp16={sorted(fp16_graphs)})"
    )
    model, cfg = load_sam2_for_size(
        args.checkpoint, args.image_size, args.num_maskmem, args.max_pointers
    )

    s = args.image_size
    s4, s8, s16 = s // 4, s // 8, s // 16
    hw = s16 * s16
    tpp = 4
    static_mem_blocks = args.num_maskmem if static_memory else None
    static_ptr_tokens = args.max_pointers * tpp
    traced_ptr_tokens = static_ptr_tokens

    audits = {}

    # 1. Vision encoder
    print("[export] Exporting vision_encoder.onnx ...")
    ve_wrap = VisionEncoderWrapper(model, s)
    dummy_pixel = torch.randn(1, 3, s, s, dtype=torch.float32)
    audits["vision_encoder.onnx"] = export_onnx(
        ve_wrap,
        (dummy_pixel,),
        out_dir / "vision_encoder.onnx",
        input_names=["pixel_values"],
        output_names=["feats0", "feats1", "feats2", "pos0", "pos1", "pos2"],
        dynamic_axes=None,
        opset=args.opset,
        use_fp16="vision_encoder" in fp16_graphs,
        fp16_block_list=DEFAULT_FP16_BLOCK_LIST,
    )

    # 2. Mask decoder
    print("[export] Exporting mask_decoder.onnx ...")
    md_wrap = MaskDecoderWrapper(model, s)
    d_f0 = torch.randn(1, 32, s4, s4, dtype=torch.float32)
    d_f1 = torch.randn(1, 64, s8, s8, dtype=torch.float32)
    d_f2 = torch.randn(1, 256, s16, s16, dtype=torch.float32)
    d_pts = torch.tensor([[[[s * 0.5, s * 0.5], [s * 0.25, s * 0.25]]]], dtype=torch.float32)
    d_lbl = torch.tensor([[[1, 0]]], dtype=torch.int32)
    audits["mask_decoder.onnx"] = export_onnx(
        md_wrap,
        (d_f0, d_f1, d_f2, d_pts, d_lbl),
        out_dir / "mask_decoder.onnx",
        input_names=["feats0", "feats1", "feats2", "input_points", "input_labels"],
        output_names=["pred_masks", "high_res_masks", "iou_scores", "object_pointer", "object_score_logits"],
        dynamic_axes={
            "input_points": {2: "num_points"},
            "input_labels": {2: "num_points"},
        },
        opset=args.opset,
        use_fp16="mask_decoder" in fp16_graphs,
        fp16_block_list=DEFAULT_FP16_BLOCK_LIST,
    )

    # 3. Memory attention
    print("[export] Exporting memory_attention.onnx ...")
    ma_wrap = MemoryAttentionWrapper(
        model,
        num_pointer_tokens=traced_ptr_tokens,
        use_static_mask=static_memory,
    )
    spatial_len = (args.num_maskmem * hw) if static_memory else hw
    d_cur = torch.randn(hw, 1, 256, dtype=torch.float32)
    d_cur_pos = torch.randn(hw, 1, 256, dtype=torch.float32)
    d_smem = torch.randn(spatial_len, 1, 64, dtype=torch.float32)
    d_smem_pos = torch.randn(spatial_len, 1, 64, dtype=torch.float32)
    d_pmem = torch.randn(traced_ptr_tokens, 1, 64, dtype=torch.float32)
    d_pmem_pos = torch.randn(traced_ptr_tokens, 1, 64, dtype=torch.float32)

    if static_memory:
        d_mask = torch.zeros(1, 1, 1, spatial_len + traced_ptr_tokens, dtype=torch.float32)
        ma_args = (d_cur, d_cur_pos, d_smem, d_smem_pos, d_pmem, d_pmem_pos, d_mask)
        ma_in_names = [
            "current_vision_features",
            "current_vision_position_embeddings",
            "spatial_memory",
            "spatial_memory_position_embeddings",
            "pointer_memory",
            "pointer_memory_position_embeddings",
            "memory_attention_mask",
        ]
        ma_dyn_axes = None
    else:
        ma_args = (d_cur, d_cur_pos, d_smem, d_smem_pos, d_pmem, d_pmem_pos)
        ma_in_names = [
            "current_vision_features",
            "current_vision_position_embeddings",
            "spatial_memory",
            "spatial_memory_position_embeddings",
            "pointer_memory",
            "pointer_memory_position_embeddings",
        ]
        ma_dyn_axes = {
            "spatial_memory": {0: "spatial_len"},
            "spatial_memory_position_embeddings": {0: "spatial_len"},
            "pointer_memory": {0: "pointer_len"},
            "pointer_memory_position_embeddings": {0: "pointer_len"},
        }

    with patch_eager_cross_attn_mask():
        audits["memory_attention.onnx"] = export_onnx(
            ma_wrap,
            ma_args,
            out_dir / "memory_attention.onnx",
            input_names=ma_in_names,
            output_names=["conditioned_features"],
            dynamic_axes=ma_dyn_axes,
            opset=args.opset,
            use_fp16="memory_attention" in fp16_graphs,
            fp16_block_list=DEFAULT_FP16_BLOCK_LIST,
        )

    # 4. Memory encoder
    print("[export] Exporting memory_encoder.onnx ...")
    mask_mem_size = int(model.prompt_encoder.mask_input_size[0]) * 4
    me_wrap = MemoryEncoderWrapper(model, mask_mem_size=mask_mem_size)
    d_vf = torch.randn(1, 256, s16, s16, dtype=torch.float32)
    d_pm = torch.randn(1, 1, s, s, dtype=torch.float32)
    d_osl = torch.ones(1, 1, dtype=torch.float32)
    d_pts_flag = torch.tensor([True], dtype=torch.bool)
    audits["memory_encoder.onnx"] = export_onnx(
        me_wrap,
        (d_vf, d_pm, d_osl, d_pts_flag),
        out_dir / "memory_encoder.onnx",
        input_names=["vision_features", "pred_masks_high_res", "object_score_logits", "is_mask_from_pts"],
        output_names=["memory_tokens", "memory_pos_enc"],
        dynamic_axes=None,
        opset=args.opset,
        use_fp16="memory_encoder" in fp16_graphs,
        fp16_block_list=DEFAULT_FP16_BLOCK_LIST,
    )

    # 5. constants.json
    no_mem = model.no_memory_embedding.detach().cpu().reshape(-1).tolist()
    tpos_pe = model.memory_temporal_positional_encoding.detach().cpu().reshape(args.num_maskmem, 64).tolist()
    proj_layer = model.temporal_positional_encoding_projection_layer
    ptr_w = proj_layer.weight.detach().cpu().reshape(-1).tolist()
    ptr_b = proj_layer.bias.detach().cpu().reshape(-1).tolist()

    constants = {
        "model": "sam2.1-tiny",
        "variant": variant_name,
        "hf_repo": DEFAULT_HF_REPO,
        "source_commit": DEFAULT_CHECKPOINT_COMMIT,
        "license": "apache-2.0",
        "image_size": s,
        "backbone_feature_sizes": [[s4, s4], [s8, s8], [s16, s16]],
        "hidden_dim": 256,
        "mem_dim": 64,
        "num_maskmem": args.num_maskmem,
        "max_object_pointers_in_encoder": args.max_pointers,
        "max_cond_frame_num": -1,
        "num_object_pointers_traced": args.max_pointers,
        "tokens_per_pointer": tpp,
        "num_object_pointer_tokens_traced": static_ptr_tokens,
        "memory_length_traced": (args.num_maskmem * hw + static_ptr_tokens) if static_memory else None,
        "static_memory_blocks": static_mem_blocks,
        "static_pointer_tokens": static_ptr_tokens if static_memory else None,
        "has_memory_attn_mask": static_memory,
        "memory_axes_dynamic": [] if static_memory else ["spatial_memory", "pointer_memory"],
        "spatial_tokens_per_memory_block": hw,
        "sigmoid_scale_for_mem_enc": float(cfg.sigmoid_scale_for_mem_enc),
        "sigmoid_bias_for_mem_enc": float(cfg.sigmoid_bias_for_mem_enc),
        "enable_temporal_pos_encoding_for_object_pointers": True,
        "no_memory_embedding": no_mem,
        "memory_temporal_positional_encoding": tpos_pe,
        "pointer_tpos_proj_weight": ptr_w,
        "pointer_tpos_proj_bias": ptr_b,
        "fp16_graphs": sorted(fp16_graphs),
        "opset": args.opset,
    }
    constants_path = out_dir / "constants.json"
    constants_path.write_text(json.dumps(constants, indent=2) + "\n", encoding="utf-8")

    # LICENSE & NOTICE
    repo_root = Path(__file__).resolve().parents[2]
    apache_src = next(
        (p for p in [repo_root / "LICENSE-APACHE", repo_root / "LICENSES" / "Apache-2.0.txt"] if p.exists()),
        None,
    )
    if apache_src is not None:
        shutil.copy2(apache_src, out_dir / "LICENSE")
    else:
        (out_dir / "LICENSE").write_text(f"Apache License 2.0 ({APACHE_LICENSE_URL})\n", encoding="utf-8")
    (out_dir / "NOTICE").write_text(NOTICE_TEXT, encoding="utf-8")

    files_to_pin = [
        "vision_encoder.onnx",
        "mask_decoder.onnx",
        "memory_attention.onnx",
        "memory_encoder.onnx",
        "constants.json",
    ]
    manifest = {
        "variant": variant_name,
        "files": {},
        "audits": audits,
    }
    for fname in files_to_pin:
        fpath = out_dir / fname
        manifest["files"][fname] = {
            "bytes": fpath.stat().st_size,
            "sha256": sha256_file(fpath),
        }
    (out_dir / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")

    print(f"\n[export] Done -> {out_dir}")
    for fname, meta in manifest["files"].items():
        print(f"  {fname:24s} {meta['bytes']:10d} B  sha256={meta['sha256']}")


if __name__ == "__main__":
    main()
