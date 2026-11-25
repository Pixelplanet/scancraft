//! Mask upsampling, thresholding, stability/IoU scoring, and lost-object detection.

use crate::preprocess::ImageTransform;

/// Configuration for converting decoder logits into a binary mask and tracking status.
#[derive(Debug, Clone, PartialEq)]
pub struct PostprocessConfig {
    /// Logit threshold above which a pixel is foreground (SAM 2 uses `0.0`).
    pub mask_threshold: f32,
    /// Offset around `mask_threshold` used for SAM's stability score (`iou(logits > t + d, logits > t - d)`).
    pub stability_delta: f32,
    /// Minimum predicted IoU below which the frame is marked lost/low-confidence.
    pub min_iou: f32,
    /// Minimum stability score below which the frame is marked low-confidence/lost.
    pub min_stability: f32,
    /// Minimum `object_score_logit` (`<= 0` means SAM 2 predicts the object is occluded/absent).
    pub min_object_score: f32,
    /// Minimum foreground pixel count before reporting [`LostReason::EmptyMask`].
    pub min_foreground_pixels: usize,
}

impl Default for PostprocessConfig {
    fn default() -> Self {
        Self {
            mask_threshold: 0.0,
            stability_delta: 1.0,
            min_iou: 0.5,
            min_stability: 0.7,
            min_object_score: 0.0,
            min_foreground_pixels: 16,
        }
    }
}

/// Reason a tracked frame is considered lost.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LostReason {
    NoObjectScore,
    EmptyMask,
    LowIoU,
    LowStability,
}

/// Tracking health for a frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TrackingStatus {
    Tracked,
    LowConfidence,
    Lost { reason: LostReason },
}

impl TrackingStatus {
    pub fn is_lost(self) -> bool {
        matches!(self, Self::Lost { .. })
    }
}

/// Per-stage wall-clock timings for one frame, in milliseconds.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct FrameTimings {
    pub preprocess_ms: f32,
    pub vision_encoder_ms: f32,
    pub memory_attention_ms: f32,
    pub mask_decoder_ms: f32,
    pub memory_encoder_ms: f32,
    pub postprocess_ms: f32,
    pub total_ms: f32,
}

/// Binary 2D segmentation mask (`0` = background, `255` = foreground).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Mask2D {
    pub width: u32,
    pub height: u32,
    pub data: Vec<u8>,
}

impl Mask2D {
    pub fn foreground_count(&self) -> usize {
        self.data.iter().filter(|&&v| v > 127).count()
    }

    pub fn foreground_fraction(&self) -> f32 {
        self.foreground_count() as f32 / self.data.len().max(1) as f32
    }

    #[inline]
    pub fn is_foreground(&self, x: u32, y: u32) -> bool {
        if x < self.width && y < self.height {
            self.data[(y * self.width + x) as usize] > 127
        } else {
            false
        }
    }

    /// Intersection-over-Union against another mask of identical dimensions.
    pub fn iou(&self, other: &Mask2D) -> f32 {
        if self.width != other.width || self.height != other.height {
            return 0.0;
        }
        let mut inter = 0usize;
        let mut union = 0usize;
        for (&a, &b) in self.data.iter().zip(&other.data) {
            let fa = a > 127;
            let fb = b > 127;
            if fa && fb {
                inter += 1;
            }
            if fa || fb {
                union += 1;
            }
        }
        if union == 0 { 1.0 } else { inter as f32 / union as f32 }
    }

    /// Erodes the foreground mask by `radius_px` (Chebyshev L-inf disk) to guard boundary silhouettes.
    pub fn eroded(&self, radius_px: usize) -> Self {
        if radius_px == 0 {
            return self.clone();
        }
        let (w, h) = (self.width as usize, self.height as usize);
        let r = radius_px as isize;
        let mut out = vec![0u8; self.data.len()];
        for y in 0..h {
            for x in 0..w {
                if self.data[y * w + x] <= 127 {
                    continue;
                }
                let mut keep = true;
                'check: for dy in -r..=r {
                    let yy = y as isize + dy;
                    if yy < 0 || yy >= h as isize {
                        keep = false;
                        break;
                    }
                    for dx in -r..=r {
                        let xx = x as isize + dx;
                        if xx < 0 || xx >= w as isize || self.data[yy as usize * w + xx as usize] <= 127 {
                            keep = false;
                            break 'check;
                        }
                    }
                }
                if keep {
                    out[y * w + x] = 255;
                }
            }
        }
        Self { width: self.width, height: self.height, data: out }
    }
}

/// Complete segmentation result for one video frame.
#[derive(Debug, Clone, PartialEq)]
pub struct FrameMask {
    pub frame_index: u64,
    pub mask: Mask2D,
    pub iou_score: f32,
    pub stability_score: f32,
    pub object_score_logit: f32,
    pub status: TrackingStatus,
    pub timings: FrameTimings,
}

/// Computes SAM's mask stability score: `IoU(logits > threshold + delta, logits > threshold - delta)`.
pub fn compute_stability_score(logits: &[f32], threshold: f32, delta: f32) -> f32 {
    let hi_t = threshold + delta;
    let lo_t = threshold - delta;
    let mut inter = 0usize;
    let mut union = 0usize;
    for &v in logits {
        let lo = v > lo_t;
        if lo {
            union += 1;
            if v > hi_t {
                inter += 1;
            }
        }
    }
    if union == 0 { 0.0 } else { inter as f32 / union as f32 }
}

/// Bilinearly resamples `src_w x src_h` model-space logits into original image resolution.
pub fn resample_mask_logits(logits: &[f32], src_w: usize, src_h: usize, tf: &ImageTransform) -> Vec<f32> {
    let (dst_w, dst_h) = (tf.orig_width as usize, tf.orig_height as usize);
    let mut out = vec![0.0f32; dst_w * dst_h];
    if src_w == 0 || src_h == 0 || logits.len() < src_w * src_h {
        return out;
    }
    let sx_model = src_w as f32 / tf.model_size as f32;
    let sy_model = src_h as f32 / tf.model_size as f32;

    for y in 0..dst_h {
        let my = ((y as f32 + 0.5) * tf.scale_y + tf.pad_y) * sy_model - 0.5;
        let my_c = my.clamp(0.0, (src_h - 1) as f32);
        let y0 = my_c.floor() as usize;
        let y1 = (y0 + 1).min(src_h - 1);
        let wy = my_c - y0 as f32;

        for x in 0..dst_w {
            let mx = ((x as f32 + 0.5) * tf.scale_x + tf.pad_x) * sx_model - 0.5;
            let mx_c = mx.clamp(0.0, (src_w - 1) as f32);
            let x0 = mx_c.floor() as usize;
            let x1 = (x0 + 1).min(src_w - 1);
            let wx = mx_c - x0 as f32;

            let v00 = logits[y0 * src_w + x0];
            let v10 = logits[y0 * src_w + x1];
            let v01 = logits[y1 * src_w + x0];
            let v11 = logits[y1 * src_w + x1];
            let top = v00 + (v10 - v00) * wx;
            let bot = v01 + (v11 - v01) * wx;
            out[y * dst_w + x] = top + (bot - top) * wy;
        }
    }
    out
}

/// Resamples, thresholds, scores stability, and determines tracking status for one frame.
pub fn threshold_mask(
    frame_index: u64,
    model_logits: &[f32],
    src_w: usize,
    src_h: usize,
    tf: &ImageTransform,
    iou_score: f32,
    object_score_logit: f32,
    cfg: &PostprocessConfig,
    timings: FrameTimings,
) -> FrameMask {
    let stability_score = compute_stability_score(model_logits, cfg.mask_threshold, cfg.stability_delta);
    let orig_logits = if src_w == tf.orig_width as usize && src_h == tf.orig_height as usize && tf.pad_x == 0.0 && tf.pad_y == 0.0 {
        model_logits[..src_w * src_h].to_vec()
    } else {
        resample_mask_logits(model_logits, src_w, src_h, tf)
    };

    let suppressed = object_score_logit <= cfg.min_object_score;
    let data: Vec<u8> = orig_logits
        .iter()
        .map(|&v| if !suppressed && v > cfg.mask_threshold { 255 } else { 0 })
        .collect();
    let mask = Mask2D { width: tf.orig_width, height: tf.orig_height, data };
    let fg = mask.foreground_count();

    let status = if suppressed {
        TrackingStatus::Lost { reason: LostReason::NoObjectScore }
    } else if fg < cfg.min_foreground_pixels {
        TrackingStatus::Lost { reason: LostReason::EmptyMask }
    } else if iou_score < cfg.min_iou && stability_score < cfg.min_stability {
        TrackingStatus::Lost { reason: LostReason::LowIoU }
    } else if iou_score < cfg.min_iou {
        TrackingStatus::LowConfidence
    } else if stability_score < cfg.min_stability {
        TrackingStatus::LowConfidence
    } else {
        TrackingStatus::Tracked
    };

    FrameMask { frame_index, mask, iou_score, stability_score, object_score_logit, status, timings }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::preprocess::ResizeMode;

    #[test]
    fn stability_score_and_iou_behave_as_expected() {
        // 4 pixels well inside (+5), 1 pixel in transition (0.5), rest well outside (-5).
        let mut logits = vec![-5.0f32; 16];
        logits[0..4].fill(5.0);
        logits[4] = 0.5;
        // > +1.0 => 4; > -1.0 => 5 => 4/5 = 0.8
        let stab = compute_stability_score(&logits, 0.0, 1.0);
        assert!((stab - 0.8).abs() < 1e-6, "{stab}");

        let a = Mask2D { width: 2, height: 2, data: vec![255, 255, 0, 0] };
        let b = Mask2D { width: 2, height: 2, data: vec![255, 0, 255, 0] };
        assert!((a.iou(&b) - 1.0 / 3.0).abs() < 1e-6);
        assert!((a.iou(&a) - 1.0).abs() < 1e-6);
    }

    #[test]
    fn threshold_mask_detects_lost_states() {
        let tf = ImageTransform::new(32, 32, 32, ResizeMode::DirectResize).unwrap();
        let cfg = PostprocessConfig::default();
        let logits = vec![5.0f32; 32 * 32];

        let no_obj = threshold_mask(0, &logits, 32, 32, &tf, 0.95, -0.5, &cfg, FrameTimings::default());
        assert_eq!(no_obj.status, TrackingStatus::Lost { reason: LostReason::NoObjectScore });
        assert_eq!(no_obj.mask.foreground_count(), 0);

        let low_iou = threshold_mask(1, &logits, 32, 32, &tf, 0.2, 4.0, &cfg, FrameTimings::default());
        assert_eq!(low_iou.status, TrackingStatus::LowConfidence);

        let neg_logits = vec![-5.0f32; 32 * 32];
        let empty = threshold_mask(2, &neg_logits, 32, 32, &tf, 0.9, 4.0, &cfg, FrameTimings::default());
        assert_eq!(empty.status, TrackingStatus::Lost { reason: LostReason::EmptyMask });

        let tracked = threshold_mask(3, &logits, 32, 32, &tf, 0.9, 4.0, &cfg, FrameTimings::default());
        assert_eq!(tracked.status, TrackingStatus::Tracked);
        assert_eq!(tracked.mask.foreground_count(), 1024);
    }

    #[test]
    fn erosion_shrinks_foreground_border() {
        let mut data = vec![0u8; 7 * 7];
        for y in 1..6 {
            for x in 1..6 {
                data[y * 7 + x] = 255;
            }
        }
        let m = Mask2D { width: 7, height: 7, data };
        let e = m.eroded(1);
        assert_eq!(e.foreground_count(), 9);
        assert!(e.is_foreground(3, 3));
        assert!(!e.is_foreground(1, 1));
    }
}
