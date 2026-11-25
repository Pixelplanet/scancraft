//! Typed ONNX Runtime session wrappers for the four SAM 2.1 video graphs.

use crate::manifest::ModelFiles;
use crate::memory_bank::{AssembledMemory, ModelConstants};
use crate::runtime::{ActiveProvider, OrtRuntime};
use crate::{Result, SegmentError};
use ndarray::{Array1, Array3, Array4};
use ort::session::Session;
use ort::session::builder::GraphOptimizationLevel;
use ort::value::TensorRef;

/// Visual features extracted by `vision_encoder.onnx` for one frame.
#[derive(Debug, Clone)]
pub struct VisionFeatures {
    pub feats0: Array4<f32>,
    pub feats1: Array4<f32>,
    pub feats2: Array4<f32>,
    pub pos2: Array4<f32>,
}

/// Raw predictions emitted by `mask_decoder.onnx`.
#[derive(Debug, Clone)]
pub struct MaskDecoderOutput {
    /// Selected high-resolution mask logits `[1, 1, 1024, 1024]`.
    pub high_res_mask: Array4<f32>,
    /// Selected IoU score for the chosen mask.
    pub iou_score: f32,
    /// Object presence score logit (`<= 0` indicates occluded / missing object).
    pub object_score_logit: f32,
    /// Object pointer vector of length `hidden_dim` (256).
    pub object_pointer: Vec<f32>,
}

/// The four SAM 2.1 ONNX graph sessions bound to one [`ActiveProvider`].
pub struct Sam2Graphs {
    pub vision_encoder: Session,
    pub mask_decoder: Session,
    pub memory_attention: Session,
    pub memory_encoder: Session,
    pub constants: ModelConstants,
    pub provider: ActiveProvider,
}

impl Sam2Graphs {
    /// Verifies `models` SHA-256 digests and loads all four ONNX graphs onto `provider`.
    pub fn load(runtime: &OrtRuntime, models: &ModelFiles, provider: ActiveProvider) -> Result<Self> {
        models.verify_all()?;
        let constants = ModelConstants::from_file(&models.constants_json.path)?;
        let opt = GraphOptimizationLevel::Level3;

        let vision_encoder = runtime.create_session(&models.vision_encoder.path, &provider, opt)?;
        let mask_decoder = runtime.create_session(&models.mask_decoder.path, &provider, opt)?;
        let memory_attention = runtime.create_session(&models.memory_attention.path, &provider, opt)?;
        let memory_encoder = runtime.create_session(&models.memory_encoder.path, &provider, opt)?;

        Ok(Self { vision_encoder, mask_decoder, memory_attention, memory_encoder, constants, provider })
    }

    /// Runs `vision_encoder.onnx` on `pixel_values` `[1, 3, image_size, image_size]`.
    pub fn encode_image(&mut self, pixel_values: &Array4<f32>) -> Result<VisionFeatures> {
        let outputs = self.vision_encoder.run(ort::inputs! {
            "pixel_values" => TensorRef::from_array_view(pixel_values.view())?
        })?;
        Ok(VisionFeatures {
            feats0: extract_4d(&outputs, "feats0")?,
            feats1: extract_4d(&outputs, "feats1")?,
            feats2: extract_4d(&outputs, "feats2")?,
            pos2: extract_4d(&outputs, "pos2")?,
        })
    }

    /// Runs `memory_attention.onnx` on `feats2` and `pos2` conditioned on `assembled`.
    /// Returns conditioned features shaped `[1, hidden_dim, f_h, f_w]` (`[1, 256, 64, 64]`).
    pub fn attend_memory(&mut self, feats2: &Array4<f32>, pos2: &Array4<f32>, assembled: &AssembledMemory) -> Result<Array4<f32>> {
        let (b, c, fh, fw) = feats2.dim();
        if b != 1 || c != self.constants.hidden_dim || fh * fw != self.constants.spatial_tokens_per_memory_block {
            return Err(SegmentError::InvalidShape {
                name: "feats2",
                detail: format!("unexpected shape [{b}, {c}, {fh}, {fw}]"),
            });
        }
        let hw = fh * fw;
        let curr_feats = nchw_to_seq_1_c(feats2, hw, c);
        let curr_pos = nchw_to_seq_1_c(pos2, hw, c);

        let mem_dim = self.constants.mem_dim;
        let spatial_mem = Array3::from_shape_vec((assembled.spatial_len, 1, mem_dim), assembled.spatial_memory.clone())
            .map_err(|e| SegmentError::InvalidShape { name: "spatial_memory", detail: e.to_string() })?;
        let spatial_pos = Array3::from_shape_vec((assembled.spatial_len, 1, mem_dim), assembled.spatial_memory_pos.clone())
            .map_err(|e| SegmentError::InvalidShape { name: "spatial_memory_position_embeddings", detail: e.to_string() })?;
        let pointer_mem = Array3::from_shape_vec((assembled.pointer_len, 1, mem_dim), assembled.pointer_memory.clone())
            .map_err(|e| SegmentError::InvalidShape { name: "pointer_memory", detail: e.to_string() })?;
        let pointer_pos = Array3::from_shape_vec((assembled.pointer_len, 1, mem_dim), assembled.pointer_memory_pos.clone())
            .map_err(|e| SegmentError::InvalidShape { name: "pointer_memory_position_embeddings", detail: e.to_string() })?;

        let outputs = self.memory_attention.run(ort::inputs! {
            "current_vision_features" => TensorRef::from_array_view(curr_feats.view())?,
            "current_vision_position_embeddings" => TensorRef::from_array_view(curr_pos.view())?,
            "spatial_memory" => TensorRef::from_array_view(spatial_mem.view())?,
            "spatial_memory_position_embeddings" => TensorRef::from_array_view(spatial_pos.view())?,
            "pointer_memory" => TensorRef::from_array_view(pointer_mem.view())?,
            "pointer_memory_position_embeddings" => TensorRef::from_array_view(pointer_pos.view())?,
        })?;

        let cond_dyn = outputs["conditioned_features"].try_extract_array::<f32>()?;
        conditioned_to_nchw(cond_dyn.shape(), cond_dyn.as_slice().ok_or_else(|| SegmentError::InvalidShape {
            name: "conditioned_features",
            detail: "non-contiguous output tensor".into(),
        })?, c, fh, fw)
    }

    /// Runs `mask_decoder.onnx` with features and encoded points/labels.
    pub fn decode_mask(
        &mut self,
        feats0: &Array4<f32>,
        feats1: &Array4<f32>,
        feats2_or_cond: &Array4<f32>,
        input_points: &Array4<f32>,
        input_labels: &ndarray::Array3<i32>,
    ) -> Result<MaskDecoderOutput> {
        let outputs = self.mask_decoder.run(ort::inputs! {
            "feats0" => TensorRef::from_array_view(feats0.view())?,
            "feats1" => TensorRef::from_array_view(feats1.view())?,
            "feats2" => TensorRef::from_array_view(feats2_or_cond.view())?,
            "input_points" => TensorRef::from_array_view(input_points.view())?,
            "input_labels" => TensorRef::from_array_view(input_labels.view())?,
        })?;

        let iou_arr = outputs["iou_scores"].try_extract_array::<f32>()?;
        let iou_slice = iou_arr.as_slice().ok_or_else(|| SegmentError::InvalidShape {
            name: "iou_scores",
            detail: "non-contiguous iou_scores".into(),
        })?;
        let (best_idx, best_iou) = iou_slice
            .iter()
            .copied()
            .enumerate()
            .max_by(|a, b| a.1.total_cmp(&b.1))
            .unwrap_or((0, 0.0));

        let obj_arr = outputs["object_score_logits"].try_extract_array::<f32>()?;
        let object_score_logit = obj_arr.iter().copied().next().unwrap_or(0.0);

        let ptr_arr = outputs["object_pointer"].try_extract_array::<f32>()?;
        let ptr_slice = ptr_arr.as_slice().ok_or_else(|| SegmentError::InvalidShape {
            name: "object_pointer",
            detail: "non-contiguous object_pointer".into(),
        })?;
        let hidden = self.constants.hidden_dim;
        if ptr_slice.len() < hidden {
            return Err(SegmentError::InvalidShape {
                name: "object_pointer",
                detail: format!("expected at least {hidden} floats, got {}", ptr_slice.len()),
            });
        }
        let object_pointer = ptr_slice[..hidden].to_vec();

        let hr_arr = outputs["high_res_masks"].try_extract_array::<f32>()?;
        let s = self.constants.image_size as usize;
        let plane = s * s;
        let hr_slice = hr_arr.as_slice().ok_or_else(|| SegmentError::InvalidShape {
            name: "high_res_masks",
            detail: "non-contiguous high_res_masks".into(),
        })?;
        let num_masks = hr_slice.len() / plane;
        if num_masks == 0 || hr_slice.len() % plane != 0 {
            return Err(SegmentError::InvalidShape {
                name: "high_res_masks",
                detail: format!("unexpected shape {:?} for {s}x{s} mask", hr_arr.shape()),
            });
        }
        let chosen = if num_masks == 1 { 0 } else { best_idx.min(num_masks - 1) };
        let mask_data = hr_slice[chosen * plane..(chosen + 1) * plane].to_vec();
        let high_res_mask = Array4::from_shape_vec((1, 1, s, s), mask_data)
            .map_err(|e| SegmentError::InvalidShape { name: "high_res_masks", detail: e.to_string() })?;

        Ok(MaskDecoderOutput { high_res_mask, iou_score: best_iou, object_score_logit, object_pointer })
    }

    /// Runs `memory_encoder.onnx` to encode the current frame's mask into spatial memory.
    pub fn encode_memory(
        &mut self,
        vision_feats2: &Array4<f32>,
        high_res_mask: &Array4<f32>,
        object_score_logit: f32,
        is_mask_from_pts: bool,
    ) -> Result<(Vec<f32>, Vec<f32>)> {
        let obj_logits = ndarray::Array2::from_elem((1, 1), object_score_logit);
        let from_pts = Array1::from_vec(vec![is_mask_from_pts]);

        let outputs = self.memory_encoder.run(ort::inputs! {
            "vision_features" => TensorRef::from_array_view(vision_feats2.view())?,
            "pred_masks_high_res" => TensorRef::from_array_view(high_res_mask.view())?,
            "object_score_logits" => TensorRef::from_array_view(obj_logits.view())?,
            "is_mask_from_pts" => TensorRef::from_array_view(from_pts.view())?,
        })?;

        let mem_tokens = outputs["memory_tokens"].try_extract_array::<f32>()?.iter().copied().collect::<Vec<_>>();
        let mem_pos = outputs["memory_pos_enc"].try_extract_array::<f32>()?.iter().copied().collect::<Vec<_>>();
        Ok((mem_tokens, mem_pos))
    }
}

fn extract_4d(outputs: &ort::session::SessionOutputs<'_>, name: &'static str) -> Result<Array4<f32>> {
    let view = outputs[name].try_extract_array::<f32>()?;
    let owned = view.to_owned();
    owned.into_dimensionality::<ndarray::Ix4>().map_err(|e| SegmentError::InvalidShape { name, detail: e.to_string() })
}

fn nchw_to_seq_1_c(arr: &Array4<f32>, hw: usize, c: usize) -> Array3<f32> {
    let mut out = Array3::<f32>::zeros((hw, 1, c));
    let (_, _, h, w) = arr.dim();
    for ch in 0..c {
        for y in 0..h {
            for x in 0..w {
                out[[y * w + x, 0, ch]] = arr[[0, ch, y, x]];
            }
        }
    }
    out
}

fn conditioned_to_nchw(shape: &[usize], data: &[f32], c: usize, fh: usize, fw: usize) -> Result<Array4<f32>> {
    let hw = fh * fw;
    let total = c * hw;
    if data.len() != total {
        return Err(SegmentError::InvalidShape {
            name: "conditioned_features",
            detail: format!("expected {total} elements, got {} with shape {shape:?}", data.len()),
        });
    }
    if shape == [1, c, fh, fw] {
        return Array4::from_shape_vec((1, c, fh, fw), data.to_vec())
            .map_err(|e| SegmentError::InvalidShape { name: "conditioned_features", detail: e.to_string() });
    }
    if shape == [1, fh, fw, c] {
        let mut out = Array4::<f32>::zeros((1, c, fh, fw));
        for y in 0..fh {
            for x in 0..fw {
                let base = (y * fw + x) * c;
                for ch in 0..c {
                    out[[0, ch, y, x]] = data[base + ch];
                }
            }
        }
        return Ok(out);
    }
    // Common `memory_attention` export layouts: [1, 1, 4096, 256], [4096, 1, 256], or [1, 4096, 256].
    if shape.last() == Some(&c) {
        let mut out = Array4::<f32>::zeros((1, c, fh, fw));
        for idx in 0..hw {
            let y = idx / fw;
            let x = idx % fw;
            let base = idx * c;
            for ch in 0..c {
                out[[0, ch, y, x]] = data[base + ch];
            }
        }
        return Ok(out);
    }
    Err(SegmentError::InvalidShape {
        name: "conditioned_features",
        detail: format!("unrecognized output layout {shape:?}"),
    })
}
