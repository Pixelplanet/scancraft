//! SAM 2.1 video memory bank, pointer tokens, and 1D sinusoidal temporal positional encoding.

use crate::{Result, SegmentError};
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;
use std::path::Path;

/// Hyperparameters and tensor geometry loaded from `constants.json`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ModelConstants {
    pub model: String,
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
}

fn default_opset() -> u32 {
    17
}

impl Default for ModelConstants {
    fn default() -> Self {
        Self {
            model: "sam2.1-tiny".into(),
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
        let total_blocks = selected_cond.len() + selected_recent.len();
        let mut spatial_memory = Vec::with_capacity(total_blocks * block_floats);
        let mut spatial_memory_pos = Vec::with_capacity(total_blocks * block_floats);

        for entry in selected_cond.iter().chain(selected_recent.iter()) {
            spatial_memory.extend_from_slice(&entry.memory_tokens);
            spatial_memory_pos.extend_from_slice(&entry.memory_pos_enc);
        }

        // Object pointers: conditioning pointers first, then recent non-conditioning pointers up to
        // `max_object_pointers_in_encoder` (16).
        let max_ptrs = self.constants.max_object_pointers_in_encoder;
        let tpp = self.constants.tokens_per_pointer;
        let mut ptr_entries: Vec<(i64, &MemoryEntry)> = Vec::new();
        for e in &selected_cond {
            if ptr_entries.len() < max_ptrs {
                let pos = (current_frame as i64 - e.frame_index as i64).max(0);
                ptr_entries.push((pos, e));
            }
        }
        let rem_ptrs = max_ptrs.saturating_sub(ptr_entries.len());
        for (_, e) in self.non_cond_frames.range(..current_frame).rev().take(rem_ptrs) {
            let pos = (current_frame as i64 - e.frame_index as i64).max(0);
            ptr_entries.push((pos, e));
        }

        let pointer_len = ptr_entries.len() * tpp;
        let mut pointer_memory = Vec::with_capacity(pointer_len * mem_dim);
        let mut pointer_memory_pos = Vec::with_capacity(pointer_len * mem_dim);
        let max_pos_norm = (max_ptrs.saturating_sub(1)).max(1) as f32;

        for (t_diff, entry) in ptr_entries {
            // Split the 256-d pointer into `tpp` (4) x `mem_dim` (64) tokens.
            pointer_memory.extend_from_slice(&entry.object_pointer);
            if self.constants.enable_temporal_pos_encoding_for_object_pointers {
                let pe = sine_pe_1d(t_diff as f32 / max_pos_norm, mem_dim);
                for _ in 0..tpp {
                    pointer_memory_pos.extend_from_slice(&pe);
                }
            } else {
                pointer_memory_pos.resize(pointer_memory_pos.len() + tpp * mem_dim, 0.0);
            }
        }

        Some(AssembledMemory {
            spatial_memory,
            spatial_memory_pos,
            spatial_len: total_blocks * block_tokens,
            pointer_memory,
            pointer_memory_pos,
            pointer_len,
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
}
