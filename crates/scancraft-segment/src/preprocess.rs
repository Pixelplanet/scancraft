//! Image preprocessing and prompt coordinate encoding for SAM 2.1.

use crate::{Result, SegmentError};
use ndarray::Array4;
use scancraft_core::RgbImage;

/// ImageNet channel mean and standard deviation expected by Hiera-Tiny.
pub const IMAGENET_MEAN: [f32; 3] = [0.485, 0.456, 0.406];
pub const IMAGENET_STD: [f32; 3] = [0.229, 0.224, 0.225];

/// How an arbitrary-aspect image is mapped onto the square `model_size x model_size` input.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ResizeMode {
    /// Stretch directly to `(model_size, model_size)` (matches SAM 2.1's `ResizeLongestSide`/square transform).
    #[default]
    DirectResize,
    /// Preserve aspect ratio and pad with zeros on the right/bottom (or centered).
    Letterbox,
}

/// Bidirectional coordinate mapping between original image pixels and SAM 2.1 model pixels.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ImageTransform {
    pub orig_width: u32,
    pub orig_height: u32,
    pub model_size: u32,
    pub mode: ResizeMode,
    pub scale_x: f32,
    pub scale_y: f32,
    pub pad_x: f32,
    pub pad_y: f32,
}

impl ImageTransform {
    pub fn new(orig_width: u32, orig_height: u32, model_size: u32, mode: ResizeMode) -> Result<Self> {
        if orig_width == 0 || orig_height == 0 || model_size == 0 {
            return Err(SegmentError::InvalidInput("image and model dimensions must be non-zero".into()));
        }
        let (scale_x, scale_y, pad_x, pad_y) = match mode {
            ResizeMode::DirectResize => (
                model_size as f32 / orig_width as f32,
                model_size as f32 / orig_height as f32,
                0.0,
                0.0,
            ),
            ResizeMode::Letterbox => {
                let s = (model_size as f32 / orig_width as f32).min(model_size as f32 / orig_height as f32);
                let rw = orig_width as f32 * s;
                let rh = orig_height as f32 * s;
                (s, s, (model_size as f32 - rw) * 0.5, (model_size as f32 - rh) * 0.5)
            }
        };
        Ok(Self { orig_width, orig_height, model_size, mode, scale_x, scale_y, pad_x, pad_y })
    }

    /// Maps an original-image `(x, y)` coordinate into model pixel coordinates `[0, model_size]`.
    #[inline]
    pub fn point_to_model(&self, x: f32, y: f32) -> [f32; 2] {
        [x * self.scale_x + self.pad_x, y * self.scale_y + self.pad_y]
    }

    /// Maps a model-space `(mx, my)` coordinate back to original image pixels.
    #[inline]
    pub fn point_from_model(&self, mx: f32, my: f32) -> [f32; 2] {
        [(mx - self.pad_x) / self.scale_x, (my - self.pad_y) / self.scale_y]
    }

    /// Maps a box `[x0, y0, x1, y1]` in original pixels into model coordinates `[[mx0, my0], [mx1, my1]]`.
    pub fn box_to_model(&self, box_xyxy: [f32; 4]) -> [[f32; 2]; 2] {
        let (x0, x1) = (box_xyxy[0].min(box_xyxy[2]), box_xyxy[0].max(box_xyxy[2]));
        let (y0, y1) = (box_xyxy[1].min(box_xyxy[3]), box_xyxy[1].max(box_xyxy[3]));
        [self.point_to_model(x0, y0), self.point_to_model(x1, y1)]
    }
}

/// Positive or negative click prompt in original image pixel coordinates.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PointPrompt {
    pub x: f32,
    pub y: f32,
    pub positive: bool,
}

impl PointPrompt {
    pub fn positive(x: f32, y: f32) -> Self {
        Self { x, y, positive: true }
    }

    pub fn negative(x: f32, y: f32) -> Self {
        Self { x, y, positive: false }
    }
}

/// Bounding-box prompt `[x0, y0, x1, y1]` in original image pixel coordinates.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BoxPrompt {
    pub x0: f32,
    pub y0: f32,
    pub x1: f32,
    pub y1: f32,
}

impl BoxPrompt {
    pub fn new(x0: f32, y0: f32, x1: f32, y1: f32) -> Self {
        Self { x0, y0, x1, y1 }
    }
}

/// All point and box prompts supplied for one frame.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct FramePrompt {
    pub points: Vec<PointPrompt>,
    pub boxes: Vec<BoxPrompt>,
}

impl FramePrompt {
    pub fn is_empty(&self) -> bool {
        self.points.is_empty() && self.boxes.is_empty()
    }

    pub fn with_point(mut self, x: f32, y: f32, positive: bool) -> Self {
        self.points.push(PointPrompt { x, y, positive });
        self
    }

    pub fn with_box(mut self, x0: f32, y0: f32, x1: f32, y1: f32) -> Self {
        self.boxes.push(BoxPrompt { x0, y0, x1, y1 });
        self
    }

    /// Encodes prompts into `(input_points [1, 1, N, 2], input_labels [1, 1, N])`.
    ///
    /// SAM 2 label convention: `1` = positive point, `0` = negative point, `2` = box top-left,
    /// `3` = box bottom-right, `-1` = padding point (used when propagating with no new prompts).
    pub fn encode(&self, transform: &ImageTransform) -> (ndarray::Array4<f32>, ndarray::Array3<i32>) {
        let mut coords: Vec<[f32; 2]> = Vec::with_capacity(self.points.len() + self.boxes.len() * 2 + 1);
        let mut labels: Vec<i32> = Vec::with_capacity(coords.capacity());

        for b in &self.boxes {
            let [tl, br] = transform.box_to_model([b.x0, b.y0, b.x1, b.y1]);
            coords.push(tl);
            labels.push(2);
            coords.push(br);
            labels.push(3);
        }
        for p in &self.points {
            coords.push(transform.point_to_model(p.x, p.y));
            labels.push(if p.positive { 1 } else { 0 });
        }
        if coords.is_empty() {
            coords.push([0.0, 0.0]);
            labels.push(-1);
        }

        let n = coords.len();
        let mut pts = ndarray::Array4::<f32>::zeros((1, 1, n, 2));
        let mut lbls = ndarray::Array3::<i32>::zeros((1, 1, n));
        for (i, (c, l)) in coords.into_iter().zip(labels).enumerate() {
            pts[[0, 0, i, 0]] = c[0];
            pts[[0, 0, i, 1]] = c[1];
            lbls[[0, 0, i]] = l;
        }
        (pts, lbls)
    }
}

/// Resizes and ImageNet-normalises `image` into an NCHW `f32` tensor `[1, 3, model_size, model_size]`.
pub fn preprocess_rgb(image: &RgbImage, model_size: u32, mode: ResizeMode) -> Result<(Array4<f32>, ImageTransform)> {
    let tf = ImageTransform::new(image.width, image.height, model_size, mode)?;
    let ch = image.channels as usize;
    if !matches!(ch, 1 | 3 | 4) {
        return Err(SegmentError::InvalidInput(format!("unsupported channel count {} (expected 1, 3 or 4)", image.channels)));
    }
    let expected_len = (image.width as usize) * (image.height as usize) * ch;
    if image.data.len() < expected_len {
        return Err(SegmentError::InvalidInput(format!(
            "image buffer too small: {} bytes for {}x{}x{}",
            image.data.len(),
            image.width,
            image.height,
            ch
        )));
    }

    let s = model_size as usize;
    let plane = s * s;
    let mut buf = vec![0.0f32; 3 * plane];
    let (w, h) = (image.width as usize, image.height as usize);
    let data = &image.data;

    let sample_rgb = |ix: usize, iy: usize| -> [f32; 3] {
        let base = (iy * w + ix) * ch;
        if ch == 1 {
            let v = data[base] as f32 / 255.0;
            [v, v, v]
        } else {
            [data[base] as f32 / 255.0, data[base + 1] as f32 / 255.0, data[base + 2] as f32 / 255.0]
        }
    };

    for my in 0..s {
        let oy = (my as f32 + 0.5 - tf.pad_y) / tf.scale_y - 0.5;
        if matches!(mode, ResizeMode::Letterbox) && (oy < -0.5 || oy > h as f32 - 0.5) {
            continue;
        }
        let oy_c = oy.clamp(0.0, (h - 1) as f32);
        let y0 = oy_c.floor() as usize;
        let y1 = (y0 + 1).min(h - 1);
        let wy = oy_c - y0 as f32;

        for mx in 0..s {
            let ox = (mx as f32 + 0.5 - tf.pad_x) / tf.scale_x - 0.5;
            if matches!(mode, ResizeMode::Letterbox) && (ox < -0.5 || ox > w as f32 - 0.5) {
                continue;
            }
            let ox_c = ox.clamp(0.0, (w - 1) as f32);
            let x0 = ox_c.floor() as usize;
            let x1 = (x0 + 1).min(w - 1);
            let wx = ox_c - x0 as f32;

            let c00 = sample_rgb(x0, y0);
            let c10 = sample_rgb(x1, y0);
            let c01 = sample_rgb(x0, y1);
            let c11 = sample_rgb(x1, y1);

            let dst = my * s + mx;
            for c in 0..3 {
                let top = c00[c] + (c10[c] - c00[c]) * wx;
                let bot = c01[c] + (c11[c] - c01[c]) * wx;
                let val = top + (bot - top) * wy;
                buf[c * plane + dst] = (val - IMAGENET_MEAN[c]) / IMAGENET_STD[c];
            }
        }
    }

    let tensor = Array4::from_shape_vec((1, 3, s, s), buf).map_err(|e| SegmentError::InvalidShape {
        name: "pixel_values",
        detail: e.to_string(),
    })?;
    Ok((tensor, tf))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn transform_round_trips_direct_and_letterbox() {
        for mode in [ResizeMode::DirectResize, ResizeMode::Letterbox] {
            let tf = ImageTransform::new(1280, 1024, 1024, mode).unwrap();
            for (x, y) in [(0.0, 0.0), (640.0, 512.0), (1279.0, 1023.0)] {
                let m = tf.point_to_model(x, y);
                let back = tf.point_from_model(m[0], m[1]);
                assert!((back[0] - x).abs() < 1e-4 && (back[1] - y).abs() < 1e-4, "{mode:?}: {x},{y} -> {back:?}");
            }
        }
    }

    #[test]
    fn preprocess_normalises_solid_rgb_and_gray() {
        let solid = RgbImage { width: 4, height: 4, channels: 3, data: [255, 0, 128].repeat(16) };
        let (t, _) = preprocess_rgb(&solid, 8, ResizeMode::DirectResize).unwrap();
        assert_eq!(t.shape(), &[1, 3, 8, 8]);
        assert!((t[[0, 0, 3, 3]] - (1.0 - IMAGENET_MEAN[0]) / IMAGENET_STD[0]).abs() < 1e-4);
        assert!((t[[0, 1, 3, 3]] - (0.0 - IMAGENET_MEAN[1]) / IMAGENET_STD[1]).abs() < 1e-4);
        assert!((t[[0, 2, 3, 3]] - (128.0 / 255.0 - IMAGENET_MEAN[2]) / IMAGENET_STD[2]).abs() < 1e-4);

        let gray = RgbImage { width: 2, height: 2, channels: 1, data: vec![0, 0, 0, 0] };
        let (tg, _) = preprocess_rgb(&gray, 4, ResizeMode::DirectResize).unwrap();
        for c in 0..3 {
            assert!((tg[[0, c, 1, 1]] - (-IMAGENET_MEAN[c] / IMAGENET_STD[c])).abs() < 1e-5);
        }
    }

    #[test]
    fn prompt_encoding_handles_points_boxes_and_empty_padding() {
        let tf = ImageTransform::new(512, 512, 1024, ResizeMode::DirectResize).unwrap();
        let (pts, lbls) = FramePrompt::default().encode(&tf);
        assert_eq!(pts.shape(), &[1, 1, 1, 2]);
        assert_eq!(lbls[[0, 0, 0]], -1);

        let prompt = FramePrompt::default().with_box(100.0, 50.0, 200.0, 150.0).with_point(150.0, 100.0, true).with_point(10.0, 20.0, false);
        let (pts, lbls) = prompt.encode(&tf);
        assert_eq!(pts.shape(), &[1, 1, 4, 2]);
        assert_eq!(lbls.as_slice().unwrap(), &[2, 3, 1, 0]);
        assert_eq!([pts[[0, 0, 0, 0]], pts[[0, 0, 0, 1]]], [200.0, 100.0]);
        assert_eq!([pts[[0, 0, 2, 0]], pts[[0, 0, 2, 1]]], [300.0, 200.0]);
    }
}
