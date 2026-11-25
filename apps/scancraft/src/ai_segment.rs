//! Post-scan interactive SAM 2.1 segmentation (`Tool::AiSegment`) and 3D multi-view voting.

use std::sync::Arc;

use eframe::egui::{self, Color32, ColorImage};
use glam::Vec3;
use scancraft_color::RgbKeyframe;
use scancraft_core::{ColorRgba, Point3D, PointCloud, RgbImage, Transform};
use scancraft_segment::{
    ColorDepthBuffer, FrameMask, FramePrompt, Mask2D, MultiViewVoter, PointLabel, PointPrompt,
    Project3DConfig,
};

use crate::ai_pack::SharedAiEngine;
use crate::model::CloudLayer;

/// A user click on the selected keyframe: `(x_px, y_px, positive)`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ImagePick {
    pub x: f32,
    pub y: f32,
    pub positive: bool,
}

/// Decoded RGB keyframe ready for display and SAM 2.1 inference.
#[derive(Clone)]
pub struct DecodedKeyframe {
    pub kf: RgbKeyframe,
    pub rgb: RgbImage,
    pub display: ColorImage,
}

/// State of the interactive AI Segment tool for one target pass.
pub struct AiPicking {
    pub pass: u32,
    pub pass_name: String,
    /// Pass points in the pass's own coordinate frame (`source.pass_cloud()`).
    pub input: Arc<PointCloud>,
    /// Pass placement `project <- pass` (for aligning the 3D preview with the pass).
    pub pass_transform: Transform,
    pub keyframes: Vec<DecodedKeyframe>,
    pub selected: usize,
    /// Click prompts per keyframe index (`selected -> prompts`).
    pub prompts: Vec<Vec<ImagePick>>,
    /// Per-keyframe masks from prompt preview or full propagation.
    pub masks: Vec<Option<FrameMask>>,
    /// Per-keyframe masked RGBA image ready for upload to an egui texture.
    pub masked_displays: Vec<Option<ColorImage>>,
    /// Cached texture for the currently selected keyframe.
    pub preview_texture: Option<(usize, u64, egui::TextureHandle)>,
    pub texture_rev: u64,
    /// 3D per-point labels produced by propagating across all keyframes.
    pub labels: Option<Vec<PointLabel>>,
    /// 3D tinted preview cloud placed in the project frame (green = object, red = background).
    pub preview: Option<CloudLayer>,
    pub provider_label: String,
}

impl AiPicking {
    pub fn from_pass(
        pass: u32,
        pass_name: String,
        input: Arc<PointCloud>,
        pass_transform: Transform,
        raw_keyframes: &[RgbKeyframe],
    ) -> Result<Self, String> {
        let mut keyframes = Vec::with_capacity(raw_keyframes.len());
        for kf in raw_keyframes {
            if let Ok(decoded) = decode_keyframe(kf) {
                keyframes.push(decoded);
            }
        }
        if keyframes.is_empty() {
            return Err("No decodable RGB keyframes in the selected pass".to_string());
        }
        let n = keyframes.len();
        Ok(Self {
            pass,
            pass_name,
            input,
            pass_transform,
            keyframes,
            selected: 0,
            prompts: vec![Vec::new(); n],
            masks: vec![None; n],
            masked_displays: vec![None; n],
            preview_texture: None,
            texture_rev: 1,
            labels: None,
            preview: None,
            provider_label: String::new(),
        })
    }

    pub fn current_keyframe(&self) -> &DecodedKeyframe {
        &self.keyframes[self.selected.min(self.keyframes.len() - 1)]
    }

    pub fn current_prompts(&self) -> &[ImagePick] {
        &self.prompts[self.selected.min(self.prompts.len() - 1)]
    }

    pub fn current_mask(&self) -> Option<&FrameMask> {
        self.masks.get(self.selected).and_then(|m| m.as_ref())
    }

    pub fn any_prompts(&self) -> bool {
        self.prompts.iter().any(|p| !p.is_empty())
    }

    pub fn build_prompt_for(&self, kf_idx: usize) -> Option<FramePrompt> {
        let picks = self.prompts.get(kf_idx)?;
        if picks.is_empty() {
            return None;
        }
        let mut fp = FramePrompt::default();
        for p in picks {
            fp.points.push(PointPrompt {
                x: p.x,
                y: p.y,
                positive: p.positive,
            });
        }
        Some(fp)
    }

    pub fn set_prompt_result(&mut self, kf_idx: usize, mask: FrameMask, overlay: ColorImage) {
        if kf_idx < self.masks.len() {
            self.masks[kf_idx] = Some(mask);
            self.masked_displays[kf_idx] = Some(overlay);
            self.texture_rev += 1;
            self.preview_texture = None;
        }
    }

    pub fn set_propagation_result(
        &mut self,
        masks: Vec<FrameMask>,
        labels: Vec<PointLabel>,
        provider_label: String,
        generation: u64,
    ) {
        for (i, m) in masks.into_iter().enumerate() {
            if i < self.keyframes.len() {
                let overlay = build_overlay_image(&self.keyframes[i].rgb, &m.mask, &self.prompts[i]);
                self.masks[i] = Some(m);
                self.masked_displays[i] = Some(overlay);
            }
        }
        let colored = colored_preview_points(&self.input.points, &labels, &self.pass_transform);
        self.preview = Some(CloudLayer::new(PointCloud::from_points(colored), generation));
        self.labels = Some(labels);
        self.provider_label = provider_label;
        self.texture_rev += 1;
        self.preview_texture = None;
    }

    pub fn kept_count(&self) -> usize {
        self.labels
            .as_ref()
            .map(|l| l.iter().filter(|lbl| **lbl == PointLabel::Object).count())
            .unwrap_or(0)
    }

    pub fn filter_pass_cloud(&self) -> Option<PointCloud> {
        let labels = self.labels.as_ref()?;
        let kept: Vec<Point3D> = self
            .input
            .points
            .iter()
            .zip(labels)
            .filter(|(_, lbl)| **lbl == PointLabel::Object)
            .map(|(p, _)| *p)
            .collect();
        Some(PointCloud::from_points(kept))
    }

    pub fn texture(&mut self, ctx: &egui::Context) -> &egui::TextureHandle {
        let idx = self.selected.min(self.keyframes.len() - 1);
        let stale = self
            .preview_texture
            .as_ref()
            .is_none_or(|(k, rev, _)| *k != idx || *rev != self.texture_rev);
        if stale {
            let img = if let Some(overlay) = self.masked_displays[idx].as_ref() {
                overlay.clone()
            } else if !self.prompts[idx].is_empty() {
                let empty = Mask2D {
                    width: self.keyframes[idx].rgb.width,
                    height: self.keyframes[idx].rgb.height,
                    data: vec![0u8; (self.keyframes[idx].rgb.width * self.keyframes[idx].rgb.height) as usize],
                };
                build_overlay_image(&self.keyframes[idx].rgb, &empty, &self.prompts[idx])
            } else {
                self.keyframes[idx].display.clone()
            };
            let tex = ctx.load_texture(
                format!("ai-kf-{}-{}", self.pass, idx),
                img,
                egui::TextureOptions::LINEAR,
            );
            self.preview_texture = Some((idx, self.texture_rev, tex));
        }
        &self.preview_texture.as_ref().unwrap().2
    }
}

pub fn decode_keyframe(kf: &RgbKeyframe) -> Result<DecodedKeyframe, String> {
    let img = image::load_from_memory(&kf.jpeg)
        .map_err(|e| format!("Decode keyframe {}: {e}", kf.id))?
        .to_rgb8();
    let (w, h) = (img.width(), img.height());
    let data = img.into_raw();
    let mut rgba = Vec::with_capacity((w * h * 4) as usize);
    for px in data.chunks_exact(3) {
        rgba.extend_from_slice(&[px[0], px[1], px[2], 255]);
    }
    let display = ColorImage::from_rgba_unmultiplied([w as usize, h as usize], &rgba);
    Ok(DecodedKeyframe {
        kf: kf.clone(),
        rgb: RgbImage {
            width: w,
            height: h,
            channels: 3,
            data,
        },
        display,
    })
}

/// Composites a translucent green object tint and green/red prompt markers over `rgb`.
pub fn build_overlay_image(rgb: &RgbImage, mask: &Mask2D, picks: &[ImagePick]) -> ColorImage {
    let (w, h) = (rgb.width as usize, rgb.height as usize);
    let mut pixels = vec![Color32::BLACK; w * h];
    for y in 0..h {
        for x in 0..w {
            let idx = y * w + x;
            let off = idx * 3;
            let (r, g, b) = (rgb.data[off], rgb.data[off + 1], rgb.data[off + 2]);
            let inside = mask.is_foreground(x as u32, y as u32);
            pixels[idx] = if inside {
                Color32::from_rgb(
                    ((r as u16 * 65 + 45 * 35) / 100) as u8,
                    ((g as u16 * 65 + 220 * 35) / 100) as u8,
                    ((b as u16 * 65 + 110 * 35) / 100) as u8,
                )
            } else {
                Color32::from_rgb((r as u16 * 45 / 100) as u8, (g as u16 * 45 / 100) as u8, (b as u16 * 45 / 100) as u8)
            };
        }
    }
    for pick in picks {
        let color = if pick.positive { Color32::from_rgb(40, 235, 110) } else { Color32::from_rgb(245, 75, 75) };
        draw_crosshair(&mut pixels, w, h, pick.x.round() as i32, pick.y.round() as i32, color);
    }
    ColorImage {
        size: [w, h],
        source_size: egui::vec2(w as f32, h as f32),
        pixels,
    }
}

fn draw_crosshair(pixels: &mut [Color32], w: usize, h: usize, cx: i32, cy: i32, color: Color32) {
    for dy in -6..=6 {
        for dx in -6..=6 {
            let d2 = dx * dx + dy * dy;
            if d2 <= 36 {
                let x = cx + dx;
                let y = cy + dy;
                if x >= 0 && y >= 0 && x < w as i32 && y < h as i32 {
                    let px = &mut pixels[(y as usize) * w + (x as usize)];
                    *px = if d2 <= 16 { color } else { Color32::WHITE };
                }
            }
        }
    }
}

/// Colours 3D preview points in green (kept object), red (dropped background), or muted grey (unseen).
pub fn colored_preview_points(
    pass_points: &[Point3D],
    labels: &[PointLabel],
    pass_transform: &Transform,
) -> Vec<Point3D> {
    let keep = ColorRgba::new(72, 200, 120, 255);
    let drop = ColorRgba::new(220, 75, 75, 255);
    let unseen = ColorRgba::new(135, 140, 150, 255);
    pass_points
        .iter()
        .zip(labels)
        .map(|(p, lbl)| {
            let mut q = *p;
            q.position = pass_transform.transform_point3(Vec3::from(p.position)).to_array();
            q.normal = (pass_transform.rotation * Vec3::from(p.normal)).to_array();
            q.color = match lbl {
                PointLabel::Object => keep,
                PointLabel::Background => drop,
                PointLabel::Occluded | PointLabel::OutOfView => unseen,
            };
            q
        })
        .collect()
}

/// Evaluates SAM 2.1 on a single prompted keyframe (fast immediate visual feedback).
pub fn run_single_keyframe(
    engine: &SharedAiEngine,
    rgb: &RgbImage,
    prompt: &FramePrompt,
    picks: &[ImagePick],
    frame_index: u64,
) -> Result<(FrameMask, ColorImage), String> {
    let mask = engine.with_segmenter(|seg, _| {
        seg.reset();
        seg.add_prompt(frame_index, rgb, prompt)
            .map_err(|e| format!("SAM 2.1 prompt failed: {e}"))
    })?;
    let overlay = build_overlay_image(rgb, &mask.mask, picks);
    Ok((mask, overlay))
}

/// Propagates SAM 2.1 forward and backward from the prompted keyframes across the whole keyframe
/// sequence, then projects every view's 2D mask onto `pass_points` with depth-buffer occlusion.
pub fn propagate_and_project_3d(
    engine: &SharedAiEngine,
    keyframes: &[(RgbKeyframe, RgbImage)],
    prompts_by_kf: &[Option<FramePrompt>],
    pass_points: &[Point3D],
) -> Result<(Vec<FrameMask>, Vec<PointLabel>, String), String> {
    let n = keyframes.len();
    if n == 0 {
        return Err("No RGB keyframes to propagate".to_string());
    }
    let anchor = prompts_by_kf
        .iter()
        .position(|p| p.is_some())
        .ok_or_else(|| "Click on a keyframe first to mark the object".to_string())?;

    let mut out_masks: Vec<Option<FrameMask>> = vec![None; n];
    let provider_label = engine.with_segmenter(|seg, provider| {
        // Forward pass from the first prompted keyframe to the end.
        seg.reset();
        for idx in anchor..n {
            let (kf, rgb) = &keyframes[idx];
            let m = match &prompts_by_kf[idx] {
                Some(prompt) => seg.add_prompt(kf.id, rgb, prompt),
                None => seg.propagate(kf.id, rgb),
            }
            .map_err(|e| format!("SAM 2.1 failed on keyframe {}: {e}", kf.id))?;
            out_masks[idx] = Some(m);
        }
        // Backward pass from the anchor back to frame 0 if needed.
        if anchor > 0 {
            seg.reset();
            for idx in (0..=anchor).rev() {
                let (kf, rgb) = &keyframes[idx];
                let m = match &prompts_by_kf[idx] {
                    Some(prompt) => seg.add_prompt(kf.id, rgb, prompt),
                    None => seg.propagate(kf.id, rgb),
                }
                .map_err(|e| format!("SAM 2.1 failed on keyframe {}: {e}", kf.id))?;
                out_masks[idx].get_or_insert(m);
            }
        }
        Ok(provider.to_string())
    })?;

    let masks: Vec<FrameMask> = out_masks
        .into_iter()
        .collect::<Option<Vec<_>>>()
        .ok_or_else(|| "Incomplete SAM 2.1 propagation".to_string())?;

    let labels = project_masks_to_points(keyframes, &masks, pass_points, &Project3DConfig::default());
    Ok((masks, labels, provider_label))
}

/// Model-free multi-view 3D labeling given masks and calibrated posed keyframes.
pub fn project_masks_to_points(
    keyframes: &[(RgbKeyframe, RgbImage)],
    masks: &[FrameMask],
    pass_points: &[Point3D],
    cfg: &Project3DConfig,
) -> Vec<PointLabel> {
    let world_pts: Vec<Vec3> = pass_points.iter().map(|p| Vec3::from(p.position)).collect();
    let mut voter = MultiViewVoter::new(world_pts.len());
    for ((kf, _), fm) in keyframes.iter().zip(masks) {
        // Keyframes store sensor poses in the pass coordinate frame; transform into sensor frame
        // for z-buffer construction and 3D projection.
        let sensor_pts: Vec<Vec3> = world_pts
            .iter()
            .map(|p| kf.sensor_pose.inverse().transform_point3(*p))
            .collect();
        let zbuf = ColorDepthBuffer::from_sensor_points(&sensor_pts, &kf.camera, 1);
        let view_labels = scancraft_segment::project3d::label_points_from_mask(
            &sensor_pts,
            None,
            &kf.camera,
            &zbuf,
            &fm.mask,
            cfg,
        );
        voter.add_view(&view_labels);
    }
    voter.resolve(0.5)
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::Mat3;
    use scancraft_core::{ColorCamera, ColorFrame, PinholeIntrinsics};
    use scancraft_segment::{FrameTimings, TrackingStatus};

    #[test]
    fn projects_and_votes_masks_onto_3d_pass_points() {
        let cam = Arc::new(ColorCamera {
            width: 64,
            height: 64,
            intrinsics: PinholeIntrinsics { fx: 100.0, fy: 100.0, cx: 32.0, cy: 32.0 },
            distortion: [0.0; 5],
            rotation: Mat3::IDENTITY,
            translation: Vec3::ZERO,
        });
        let rgb = RgbImage { width: 64, height: 64, channels: 3, data: vec![120; 64 * 64 * 3] };
        let frame = ColorFrame { image: rgb.clone(), jpeg: None, timestamp_ms: 0, camera: Some(cam.clone()) };
        let kf = RgbKeyframe::new(0, &frame, cam, Transform::default(), 50.0, 0).unwrap();

        let mut mask_data = vec![0u8; 64 * 64];
        for y in 24..40 {
            for x in 24..40 {
                mask_data[y * 64 + x] = 255;
            }
        }
        let fm = FrameMask {
            frame_index: 0,
            mask: Mask2D { width: 64, height: 64, data: mask_data },
            iou_score: 0.98,
            stability_score: 0.99,
            object_score_logit: 5.0,
            status: TrackingStatus::Tracked,
            timings: FrameTimings::default(),
        };

        // Scanner +Y forward maps to optical +Z = 200 mm.
        let pts = vec![
            Point3D::new([0.0, 200.0, 0.0]),   // center -> (32, 32) inside mask
            Point3D::new([40.0, 200.0, 0.0]),  // right -> (52, 32) outside mask
            Point3D::new([0.0, 300.0, 0.0]),   // occluded behind center point
        ];
        let labels = project_masks_to_points(&[(kf, rgb)], &[fm], &pts, &Project3DConfig::default());
        assert_eq!(labels[0], PointLabel::Object);
        assert_eq!(labels[1], PointLabel::Background);
        assert_eq!(labels[2], PointLabel::OutOfView);

        let colored = colored_preview_points(&pts, &labels, &Transform::default());
        assert_ne!(colored[0].color, colored[1].color);
    }

    #[test]
    fn overlay_highlights_masked_region_and_markers() {
        let rgb = RgbImage { width: 16, height: 16, channels: 3, data: vec![100; 16 * 16 * 3] };
        let mut data = vec![0u8; 16 * 16];
        data[0] = 255;
        let mask = Mask2D { width: 16, height: 16, data };
        let overlay = build_overlay_image(&rgb, &mask, &[ImagePick { x: 8.0, y: 8.0, positive: true }]);
        assert_eq!(overlay.size, [16, 16]);
        // Masked pixel receives greenish boost while unmasked pixel is dimmed.
        assert!(overlay.pixels[0].g() > overlay.pixels[1].g());
        // Crosshair center is white/green.
        assert!(overlay.pixels[8 * 16 + 8].g() > 200);
    }
}
