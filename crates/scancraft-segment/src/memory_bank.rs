//! SAM 2.1 video memory bank, pointer tokens, and 1D sinusoidal temporal positional encoding.

use crate::{Result, SegmentError};
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;
use std::path::Path;

/// Hyperparameters and tensor geometry loaded from `constants.json`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ModelConstants {
    pub model: String,
    #[serde(default)]
    pub variant: Option<String>,
    pub image_size: u32,
    pub hidden_dim: usize,
    pub mem_dim: usize,
    pub num_maskmem: usize,
    pub max_object_pointers_in_encoder: usize,
    #[serde(default)]
    pub max_cond_frame_num: i64,
    pub tokens_per_pointer: usize,
    pub spatial_tokens_per_memory_block: usize,
    #[serde(default)]
    pub enable_temporal_pos_encoding_for_object_pointers: bool,
    #[serde(default = "default_opset")]
    pub opset: u32,
    /// Fixed number of spatial memory blocks expected by static-shape exports (padded with zeros + attention bias).
    #[serde(default)]
    pub static_memory_blocks: Option<usize>,
    /// Fixed number of pointer tokens expected by static-shape exports (padded with zeros + attention bias).
    #[serde(default)]
    pub static_pointer_tokens: Option<usize>,
    /// Whether `memory_attention.onnx` expects a `memory_attention_mask` additive bias tensor `[1, 1, 1, total_len]`.
    #[serde(default)]
    pub has_memory_attn_mask: bool,
    /// Learned `no_memory_embedding` (`hidden_dim`) added to `feats2` on unconditioned frames when exported raw.
    #[serde(default)]
    pub no_memory_embedding: Option<Vec<f32>>,
    /// Per-slot spatial temporal position embeddings (`[num_maskmem][mem_dim]`).
    #[serde(default)]
    pub memory_temporal_positional_encoding: Option<Vec<Vec<f32>>>,
    /// Optional linear projection matrix `[mem_dim * hidden_dim]` mapping `hidden_dim`-d sine PE to `mem_dim`.
    #[serde(default)]
    pub pointer_tpos_proj_weight: Option<Vec<f32>>,
    /// Optional linear projection bias `[mem_dim]`.
    #[serde(default)]
    pub pointer_tpos_proj_bias: Option<Vec<f32>>,
}

fn default_opset() -> u32 {
    17
}

impl Default for ModelConstants {
    fn default() -> Self {
        Self {
            model: "sam2.1-tiny".into(),
            variant: None,
            image_size: 1024,
            hidden_dim: 256,
            mem_dim: 64,
            num_maskmem: 7,
            max_object_pointers_in_encoder: 16,
            max_cond_frame_num: -1,
            tokens_per_pointer: 4,
            spatial_tokens_per_memory_block: 4096,
            enable_temporal_pos_encoding_for_object_pointers: true,
            opset: 17,
            static_memory_blocks: None,
            static_pointer_tokens: None,
            has_memory_attn_mask: false,
            no_memory_embedding: None,
            memory_temporal_positional_encoding: None,
            pointer_tpos_proj_weight: None,
            pointer_tpos_proj_bias: None,
        }
    }
}

impl ModelConstants {
    pub fn from_file(path: &Path) -> Result<Self> {
        let text = std::fs::read_to_string(path).map_err(|source| SegmentError::Io { path: path.to_path_buf(), source })?;
        let parsed: Self = serde_json::from_str(&text)
            .map_err(|e| SegmentError::InvalidConstants { path: path.to_path_buf(), reason: e.to_string() })?;
        if parsed.image_size == 0 || parsed.hidden_dim == 0 || parsed.mem_dim == 0 || parsed.num_maskmem == 0 {
            return Err(SegmentError::InvalidConstants {
                path: path.to_path_buf(),
                reason: "zero dimension in constants.json".into(),
            });
        }
        if parsed.tokens_per_pointer * parsed.mem_dim != parsed.hidden_dim {
            return Err(SegmentError::InvalidConstants {
                path: path.to_path_buf(),
                reason: format!(
                    "tokens_per_pointer ({}) * mem_dim ({}) != hidden_dim ({})",
                    parsed.tokens_per_pointer, parsed.mem_dim, parsed.hidden_dim
                ),
            });
        }
        if let Some(nme) = &parsed.no_memory_embedding {
            if nme.len() != parsed.hidden_dim {
                return Err(SegmentError::InvalidConstants {
                    path: path.to_path_buf(),
                    reason: format!("no_memory_embedding len {} != hidden_dim {}", nme.len(), parsed.hidden_dim),
                });
            }
        }
        if let Some(tpos) = &parsed.memory_temporal_positional_encoding {
            if tpos.len() < parsed.num_maskmem || tpos.iter().any(|v| v.len() != parsed.mem_dim) {
                return Err(SegmentError::InvalidConstants {
                    path: path.to_path_buf(),
                    reason: "invalid memory_temporal_positional_encoding shape".into(),
                });
            }
        }
        Ok(parsed)
    }
}

/// Memory produced for one tracked or prompted frame.
#[derive(Debug, Clone)]
pub struct MemoryEntry {
    pub frame_index: u64,
    /// Flattened `[spatial_tokens_per_memory_block, mem_dim]` (`4096 * 64`).
    pub memory_tokens: Vec<f32>,
    /// Flattened `[spatial_tokens_per_memory_block, mem_dim]` (`4096 * 64`).
    pub memory_pos_enc: Vec<f32>,
    /// Object pointer vector of length `hidden_dim` (`256`).
    pub object_pointer: Vec<f32>,
    pub object_score_logit: f32,
    pub is_conditioning: bool,
}

/// Concatenated spatial and pointer memory tensors ready for `memory_attention.onnx`.
#[derive(Debug, Clone)]
pub struct AssembledMemory {
    /// `[spatial_len, 1, mem_dim]` row-major.
    pub spatial_memory: Vec<f32>,
    /// `[spatial_len, 1, mem_dim]` row-major.
    pub spatial_memory_pos: Vec<f32>,
    pub spatial_len: usize,
    /// `[pointer_len, 1, mem_dim]` row-major.
    pub pointer_memory: Vec<f32>,
    /// `[pointer_len, 1, mem_dim]` row-major.
    pub pointer_memory_pos: Vec<f32>,
    pub pointer_len: usize,
    /// Optional `[spatial_len + pointer_len]` additive attention mask (`0.0` valid, `-10000.0` padded).
    pub attention_bias: Option<Vec<f32>>,
}

/// Computes SAM 2's 1D sinusoidal positional encoding `get_1d_sine_pe(pos, dim)` for a single scalar
/// relative frame position `pos` (typically `(t_diff / max_pos)`), matching `sam2_utils.py`.
pub fn sine_pe_1d(pos: f32, dim: usize) -> Vec<f32> {
    let half = dim / 2;
    let mut out = vec![0.0f32; dim];
    let tau = std::f32::consts::TAU;
    let pos_scaled = pos * tau;
    for i in 0..half {
        let exponent = (2 * i) as f32 / dim as f32;
        let omega = 1.0 / 10000.0f32.powf(exponent);
        let angle = pos_scaled * omega;
        out[i] = angle.sin();
        out[half + i] = angle.cos();
    }
    out
}

/// Exact PyTorch `get_1d_sine_pe(pos, dim)` used before `temporal_positional_encoding_projection_layer`.
fn sine_pe_1d_exact(pos: f32, dim: usize) -> Vec<f32> {
    let half = dim / 2;
    let mut out = vec![0.0f32; dim];
    for i in 0..half {
        let exponent = (2 * (i / 2)) as f32 / half as f32;
        let denom = 10000.0f32.powf(exponent);
        let angle = pos / denom;
        out[i] = angle.sin();
        out[half + i] = angle.cos();
    }
    out
}

/// Rolling memory bank holding conditioning frames (prompted) and recent non-conditioning frames.
#[derive(Debug, Clone)]
pub struct MemoryBank {
    constants: ModelConstants,
    cond_frames: BTreeMap<u64, MemoryEntry>,
    non_cond_frames: BTreeMap<u64, MemoryEntry>,
}

impl MemoryBank {
    pub fn new(constants: ModelConstants) -> Self {
        Self { constants, cond_frames: BTreeMap::new(), non_cond_frames: BTreeMap::new() }
    }

    pub fn clear(&mut self) {
        self.cond_frames.clear();
        self.non_cond_frames.clear();
    }

    pub fn is_empty(&self) -> bool {
        self.cond_frames.is_empty() && self.non_cond_frames.is_empty()
    }

    pub fn cond_len(&self) -> usize {
        self.cond_frames.len()
    }

    pub fn non_cond_len(&self) -> usize {
        self.non_cond_frames.len()
    }

    /// Inserts or updates a frame's memory entry and prunes non-conditioning entries older than the
    /// object-pointer horizon so memory stays bounded on long scans.
    pub fn insert(&mut self, entry: MemoryEntry) -> Result<()> {
        let expected_spatial = self.constants.spatial_tokens_per_memory_block * self.constants.mem_dim;
        if entry.memory_tokens.len() != expected_spatial || entry.memory_pos_enc.len() != expected_spatial {
            return Err(SegmentError::InvalidShape {
                name: "memory_entry",
                detail: format!(
                    "expected {expected_spatial} spatial floats, got {} and {}",
                    entry.memory_tokens.len(),
                    entry.memory_pos_enc.len()
                ),
            });
        }
        if entry.object_pointer.len() != self.constants.hidden_dim {
            return Err(SegmentError::InvalidShape {
                name: "object_pointer",
                detail: format!("expected {}, got {}", self.constants.hidden_dim, entry.object_pointer.len()),
            });
        }

        let idx = entry.frame_index;
        if entry.is_conditioning {
            // Re-prompting an existing frame promotes it to a conditioning frame.
            self.non_cond_frames.remove(&idx);
            self.cond_frames.insert(idx, entry);
        } else {
            self.non_cond_frames.insert(idx, entry);
            let keep_limit = self.constants.max_object_pointers_in_encoder.max(self.constants.num_maskmem) + 2;
            while self.non_cond_frames.len() > keep_limit {
                if let Some(&oldest) = self.non_cond_frames.keys().next() {
                    self.non_cond_frames.remove(&oldest);
                } else {
                    break;
                }
            }
        }
        Ok(())
    }

    /// Assembles spatial memory blocks and object-pointer tokens for `current_frame`.
    ///
    /// Returns `None` if no prior memories exist (the initial prompted frame).
    pub fn assemble_for_frame(&self, current_frame: u64) -> Option<AssembledMemory> {
        if self.is_empty() {
            return None;
        }

        let max_cond = if self.constants.max_cond_frame_num > 0 {
            self.constants.max_cond_frame_num as usize
        } else {
            usize::MAX
        };
        let mut selected_cond: Vec<&MemoryEntry> = self.cond_frames.values().filter(|e| e.frame_index < current_frame).collect();
        if selected_cond.is_empty() {
            selected_cond = self.cond_frames.values().collect();
        }
        if selected_cond.len() > max_cond {
            selected_cond.sort_by_key(|e| e.frame_index.abs_diff(current_frame));
            selected_cond.truncate(max_cond);
            selected_cond.sort_by_key(|e| e.frame_index);
        }

        // Up to `num_maskmem - 1` most recent non-conditioning frames strictly before `current_frame`.
        let max_recent = self.constants.num_maskmem.saturating_sub(1);
        let selected_recent: Vec<&MemoryEntry> = self
            .non_cond_frames
            .range(..current_frame)
            .rev()
            .take(max_recent)
            .map(|(_, e)| e)
            .collect::<Vec<_>>()
            .into_iter()
            .rev()
            .collect();

        if selected_cond.is_empty() && selected_recent.is_empty() {
            return None;
        }

        let block_tokens = self.constants.spatial_tokens_per_memory_block;
        let mem_dim = self.constants.mem_dim;
        let block_floats = block_tokens * mem_dim;
        let mut spatial_entries: Vec<(usize, &MemoryEntry)> = Vec::new();
        let num_maskmem = self.constants.num_maskmem;
        let tpos = self.constants.memory_temporal_positional_encoding.as_deref();

        for entry in selected_cond {
            let slot = num_maskmem.saturating_sub(1);
            spatial_entries.push((slot, entry));
        }
        // PyTorch iterates recent frames nearest-first or preserves chronological order; slot depends only on t_diff.
        for entry in selected_recent.into_iter().rev() {
            let t_diff = current_frame.saturating_sub(entry.frame_index) as usize;
            let slot = t_diff.saturating_sub(1).min(num_maskmem.saturating_sub(2));
            spatial_entries.push((slot, entry));
        }

        let real_blocks = spatial_entries.len();
        let target_blocks = self.constants.static_memory_blocks.unwrap_or(real_blocks);
        if spatial_entries.len() > target_blocks {
            spatial_entries.truncate(target_blocks);
        }
        let used_blocks = spatial_entries.len();
        let spatial_len = target_blocks * block_tokens;

        let mut spatial_memory = vec![0.0f32; spatial_len * mem_dim];
        let mut spatial_memory_pos = vec![0.0f32; spatial_len * mem_dim];
        for (block_idx, (slot, entry)) in spatial_entries.into_iter().enumerate() {
            let dst = block_idx * block_floats;
            spatial_memory[dst..dst + block_floats].copy_from_slice(&entry.memory_tokens);
            spatial_memory_pos[dst..dst + block_floats].copy_from_slice(&entry.memory_pos_enc);
            if let Some(tpos_table) = tpos {
                if let Some(tvec) = tpos_table.get(slot) {
                    for tok in 0..block_tokens {
                        let base = dst + tok * mem_dim;
                        for c in 0..mem_dim {
                            spatial_memory_pos[base + c] += tvec[c];
                        }
                    }
                }
            }
        }

        // Object pointers: conditioning pointers first, then recent non-conditioning pointers up to
        // `max_object_pointers_in_encoder`.
        let max_ptrs = self.constants.max_object_pointers_in_encoder;
        let tpp = self.constants.tokens_per_pointer;
        let mut ptr_entries: Vec<(i64, &MemoryEntry)> = Vec::new();
        for e in self.cond_frames.values().filter(|e| e.frame_index < current_frame) {
            if ptr_entries.len() < max_ptrs {
                let pos = (current_frame as i64 - e.frame_index as i64).max(0);
                ptr_entries.push((pos, e));
            }
        }
        if ptr_entries.is_empty() {
            for e in self.cond_frames.values() {
                if ptr_entries.len() < max_ptrs {
                    let pos = (current_frame as i64 - e.frame_index as i64).max(0);
                    ptr_entries.push((pos, e));
                }
            }
        }
        let rem_ptrs = max_ptrs.saturating_sub(ptr_entries.len());
        for (_, e) in self.non_cond_frames.range(..current_frame).rev().take(rem_ptrs) {
            let pos = (current_frame as i64 - e.frame_index as i64).max(0);
            ptr_entries.push((pos, e));
        }

        let real_ptr_tokens = ptr_entries.len() * tpp;
        let pointer_len = self
            .constants
            .static_pointer_tokens
            .map(|cap| cap.max(tpp))
            .unwrap_or(real_ptr_tokens);
        let used_ptr_tokens = real_ptr_tokens.min(pointer_len);
        let used_ptrs = used_ptr_tokens / tpp;

        let mut pointer_memory = vec![0.0f32; pointer_len * mem_dim];
        let mut pointer_memory_pos = vec![0.0f32; pointer_len * mem_dim];
        let max_pos_norm = (max_ptrs.saturating_sub(1)).max(1) as f32;
        let ptr_proj = match (
            self.constants.pointer_tpos_proj_weight.as_deref(),
            self.constants.pointer_tpos_proj_bias.as_deref(),
        ) {
            (Some(w), Some(b))
                if w.len() == mem_dim * self.constants.hidden_dim && b.len() == mem_dim =>
            {
                Some((w, b))
            }
            _ => None,
        };

        for (ptr_idx, (t_diff, entry)) in ptr_entries.into_iter().take(used_ptrs).enumerate() {
            // Split the hidden_dim pointer into `tpp` x `mem_dim` tokens.
            let mem_dst = ptr_idx * tpp * mem_dim;
            let copy_len = (tpp * mem_dim).min(entry.object_pointer.len());
            pointer_memory[mem_dst..mem_dst + copy_len].copy_from_slice(&entry.object_pointer[..copy_len]);

            if self.constants.enable_temporal_pos_encoding_for_object_pointers {
                let pe = if let Some((w, b)) = ptr_proj {
                    let raw = sine_pe_1d_exact(t_diff as f32 / max_pos_norm, self.constants.hidden_dim);
                    let mut proj = vec![0.0f32; mem_dim];
                    for m in 0..mem_dim {
                        let row = &w[m * self.constants.hidden_dim..(m + 1) * self.constants.hidden_dim];
                        let mut acc = b[m];
                        for (k, &rv) in raw.iter().enumerate() {
                            acc += row[k] * rv;
                        }
                        proj[m] = acc;
                    }
                    proj
                } else {
                    sine_pe_1d(t_diff as f32 / max_pos_norm, mem_dim)
                };
                for t in 0..tpp {
                    let pdst = mem_dst + t * mem_dim;
                    pointer_memory_pos[pdst..pdst + mem_dim].copy_from_slice(&pe);
                }
            }
        }

        let attention_bias = if self.constants.has_memory_attn_mask {
            let mut bias = vec![-10000.0f32; spatial_len + pointer_len];
            let real_spatial = used_blocks * block_tokens;
            bias[..real_spatial].fill(0.0);
            bias[spatial_len..spatial_len + used_ptr_tokens].fill(0.0);
            Some(bias)
        } else {
            None
        };

        Some(AssembledMemory {
            spatial_memory,
            spatial_memory_pos,
            spatial_len,
            pointer_memory,
            pointer_memory_pos,
            pointer_len,
            attention_bias,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn dummy_entry(frame_index: u64, is_conditioning: bool, constants: &ModelConstants) -> MemoryEntry {
        MemoryEntry {
            frame_index,
            memory_tokens: vec![frame_index as f32; constants.spatial_tokens_per_memory_block * constants.mem_dim],
            memory_pos_enc: vec![0.5; constants.spatial_tokens_per_memory_block * constants.mem_dim],
            object_pointer: vec![frame_index as f32; constants.hidden_dim],
            object_score_logit: 5.0,
            is_conditioning,
        }
    }

    #[test]
    fn empty_bank_returns_none_and_respects_window_limits() {
        let cfg = ModelConstants::default();
        let mut bank = MemoryBank::new(cfg.clone());
        assert!(bank.assemble_for_frame(0).is_none());

        // Frame 0 conditioning, frames 1..=10 non-conditioning.
        bank.insert(dummy_entry(0, true, &cfg)).unwrap();
        for t in 1..=10 {
            bank.insert(dummy_entry(t, false, &cfg)).unwrap();
        }

        let assembled = bank.assemble_for_frame(11).expect("memories present");
        // 1 conditioning + 6 recent non-conditioning = 7 spatial blocks * 4096 = 28672.
        assert_eq!(assembled.spatial_len, 7 * cfg.spatial_tokens_per_memory_block);
        // 1 cond + 10 non-cond = 11 pointers * 4 tokens = 44 pointer tokens.
        assert_eq!(assembled.pointer_len, 11 * cfg.tokens_per_pointer);
        assert_eq!(assembled.spatial_memory.len(), assembled.spatial_len * cfg.mem_dim);
        assert_eq!(assembled.pointer_memory.len(), assembled.pointer_len * cfg.mem_dim);
    }

    #[test]
    fn reprompt_promotes_frame_and_clear_resets() {
        let cfg = ModelConstants::default();
        let mut bank = MemoryBank::new(cfg.clone());
        bank.insert(dummy_entry(0, true, &cfg)).unwrap();
        bank.insert(dummy_entry(1, false, &cfg)).unwrap();
        bank.insert(dummy_entry(2, false, &cfg)).unwrap();
        assert_eq!(bank.cond_len(), 1);
        assert_eq!(bank.non_cond_len(), 2);

        // Re-prompt frame 2 -> promoted to conditioning frames.
        bank.insert(dummy_entry(2, true, &cfg)).unwrap();
        assert_eq!(bank.cond_len(), 2);
        assert_eq!(bank.non_cond_len(), 1);

        // Exceed pointer horizon -> non-conditioning frames are bounded.
        for t in 3..30 {
            bank.insert(dummy_entry(t, false, &cfg)).unwrap();
        }
        assert!(bank.non_cond_len() <= cfg.max_object_pointers_in_encoder + 2);
        let assembled = bank.assemble_for_frame(30).unwrap();
        assert_eq!(assembled.pointer_len, cfg.max_object_pointers_in_encoder * cfg.tokens_per_pointer);

        bank.clear();
        assert!(bank.is_empty());
        assert!(bank.assemble_for_frame(31).is_none());
    }

    #[test]
    fn sine_pe_1d_is_bounded_and_distinct() {
        let p0 = sine_pe_1d(0.0, 64);
        let p1 = sine_pe_1d(0.5, 64);
        assert_eq!(p0.len(), 64);
        assert!(p0.iter().all(|v| v.is_finite() && v.abs() <= 1.0));
        assert_ne!(p0, p1);
    }

    #[test]
    fn static_memory_pads_zeros_applies_attention_bias_and_temporal_pe() {
        let mut cfg = ModelConstants {
            image_size: 512,
            num_maskmem: 4,
            max_object_pointers_in_encoder: 8,
            spatial_tokens_per_memory_block: 16, // small synthetic block size for test
            static_memory_blocks: Some(4),
            static_pointer_tokens: Some(8 * 4),
            has_memory_attn_mask: true,
            no_memory_embedding: Some(vec![0.25; 256]),
            memory_temporal_positional_encoding: Some(vec![
                vec![1.0; 64],
                vec![2.0; 64],
                vec![3.0; 64],
                vec![10.0; 64], // conditioning slot (num_maskmem - 1)
            ]),
            pointer_tpos_proj_weight: Some(vec![0.5; 64 * 256]),
            pointer_tpos_proj_bias: Some(vec![0.1; 64]),
            ..ModelConstants::default()
        };
        let mut bank = MemoryBank::new(cfg.clone());
        bank.insert(dummy_entry(0, true, &cfg)).unwrap();
        bank.insert(dummy_entry(1, false, &cfg)).unwrap();

        let assembled = bank.assemble_for_frame(2).expect("assembled");
        // 4 static blocks * 16 = 64 spatial tokens; 8 pointers * 4 = 32 pointer tokens
        assert_eq!(assembled.spatial_len, 64);
        assert_eq!(assembled.pointer_len, 32);
        // Block 0 (cond frame 0) should get +10.0 added to dummy pos_enc (0.5 -> 10.5)
        assert!((assembled.spatial_memory_pos[0] - 10.5).abs() < 1e-5);
        // Block 1 (recent frame 1, t_diff=1 -> slot 0 = 1.0) -> 0.5 + 1.0 = 1.5
        assert!((assembled.spatial_memory_pos[16 * 64] - 1.5).abs() < 1e-5);
        // Block 2 & 3 padded with zeros
        assert_eq!(assembled.spatial_memory[2 * 16 * 64], 0.0);

        let bias = assembled.attention_bias.expect("attention_bias present");
        assert_eq!(bias.len(), 64 + 32);
        assert!(bias[..32].iter().all(|&v| v == 0.0));
        assert!(bias[32..64].iter().all(|&v| v == -10000.0));
        assert!(bias[64..64 + 8].iter().all(|&v| v == 0.0));
        assert!(bias[64 + 8..].iter().all(|&v| v == -10000.0));

        // Verify evicting old non-conditioning frames beyond num_maskmem - 1 window works when assembling
        for t in 2..=6 {
            bank.insert(dummy_entry(t, false, &cfg)).unwrap();
        }
        let assembled_late = bank.assemble_for_frame(7).unwrap();
        let bias_late = assembled_late.attention_bias.unwrap();
        // 1 conditioning + 3 recent = 4 active blocks => all 64 spatial tokens active (0.0)
        assert!(bias_late[..64].iter().all(|&v| v == 0.0));
        cfg.static_memory_blocks = None;
    }
}
