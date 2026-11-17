//! Small image helpers: grey conversion, pyramids, sharpness, bilinear sampling, JPEG decode.

use glam::Vec2;
use scancraft_core::RgbImage;

/// 8-bit single-channel image.
#[derive(Debug, Clone, PartialEq)]
pub struct Gray {
    pub width: usize,
    pub height: usize,
    pub data: Vec<u8>,
}

impl Gray {
    pub fn new(width: usize, height: usize) -> Self {
        Self { width, height, data: vec![0; width * height] }
    }

    #[inline]
    pub fn at(&self, x: usize, y: usize) -> u8 {
        self.data[y * self.width + x]
    }

    /// Bilinear sample; `None` outside the image.
    #[inline]
    pub fn sample(&self, p: Vec2) -> Option<f32> {
        let (x0, y0) = (p.x.floor(), p.y.floor());
        if x0 < 0.0 || y0 < 0.0 || x0 as usize + 1 >= self.width || y0 as usize + 1 >= self.height {
            return None;
        }
        let (fx, fy) = (p.x - x0, p.y - y0);
        let i = y0 as usize * self.width + x0 as usize;
        let d = &self.data;
        let top = d[i] as f32 * (1.0 - fx) + d[i + 1] as f32 * fx;
        let bottom = d[i + self.width] as f32 * (1.0 - fx) + d[i + self.width + 1] as f32 * fx;
        Some(top * (1.0 - fy) + bottom * fy)
    }
}

/// Luma (BT.601) of an RGB or grey image.
pub fn to_gray(img: &RgbImage) -> Gray {
    let (w, h) = (img.width as usize, img.height as usize);
    let data = match img.channels {
        1 => img.data.clone(),
        c => img
            .data
            .chunks_exact(c as usize)
            .map(|p| ((77 * p[0] as u32 + 150 * p[1] as u32 + 29 * p[2] as u32) >> 8) as u8)
            .collect(),
    };
    Gray { width: w, height: h, data }
}

/// Halves the resolution with a 2x2 box filter.
pub fn downsample2(img: &Gray) -> Gray {
    let (w, h) = (img.width / 2, img.height / 2);
    let mut out = Gray::new(w, h);
    for y in 0..h {
        let r0 = &img.data[2 * y * img.width..];
        let r1 = &img.data[(2 * y + 1) * img.width..];
        for x in 0..w {
            let s = r0[2 * x] as u32 + r0[2 * x + 1] as u32 + r1[2 * x] as u32 + r1[2 * x + 1] as u32;
            out.data[y * w + x] = ((s + 2) / 4) as u8;
        }
    }
    out
}

/// 3x3 binomial blur (separable 1-2-1).
pub fn blur3(img: &Gray) -> Gray {
    let (w, h) = (img.width, img.height);
    let mut tmp = vec![0u16; w * h];
    for y in 0..h {
        for x in 0..w {
            let l = img.at(x.saturating_sub(1), y) as u16;
            let r = img.at((x + 1).min(w - 1), y) as u16;
            tmp[y * w + x] = l + 2 * img.at(x, y) as u16 + r;
        }
    }
    let mut out = Gray::new(w, h);
    for y in 0..h {
        for x in 0..w {
            let u = tmp[y.saturating_sub(1) * w + x];
            let d = tmp[(y + 1).min(h - 1) * w + x];
            out.data[y * w + x] = ((u + 2 * tmp[y * w + x] + d + 8) / 16) as u8;
        }
    }
    out
}

/// Sobel gradient magnitude (L1), as f32.
pub fn gradient_magnitude(img: &Gray) -> Vec<f32> {
    let (w, h) = (img.width, img.height);
    let mut out = vec![0f32; w * h];
    for y in 1..h - 1 {
        for x in 1..w - 1 {
            let p = |dx: isize, dy: isize| img.at((x as isize + dx) as usize, (y as isize + dy) as usize) as f32;
            let gx = p(1, -1) + 2.0 * p(1, 0) + p(1, 1) - p(-1, -1) - 2.0 * p(-1, 0) - p(-1, 1);
            let gy = p(-1, 1) + 2.0 * p(0, 1) + p(1, 1) - p(-1, -1) - 2.0 * p(0, -1) - p(1, -1);
            out[y * w + x] = gx.abs() + gy.abs();
        }
    }
    out
}

/// Image sharpness: variance of the 4-neighbour Laplacian. Motion blur and defocus lower it; it
/// also depends on scene texture, so compare it between nearby frames of the same view.
pub fn sharpness(img: &Gray) -> f32 {
    let (w, h) = (img.width, img.height);
    if w < 3 || h < 3 {
        return 0.0;
    }
    let (mut sum, mut sum2, mut n) = (0f64, 0f64, 0f64);
    for y in 1..h - 1 {
        for x in 1..w - 1 {
            let c = img.at(x, y) as i32;
            let l = img.at(x - 1, y) as i32 + img.at(x + 1, y) as i32 + img.at(x, y - 1) as i32 + img.at(x, y + 1) as i32 - 4 * c;
            sum += l as f64;
            sum2 += (l * l) as f64;
            n += 1.0;
        }
    }
    let mean = sum / n;
    (sum2 / n - mean * mean) as f32
}

/// Bilinear RGB sample; `None` outside the image.
#[inline]
pub fn sample_rgb(img: &RgbImage, p: Vec2) -> Option<[f32; 3]> {
    let (w, h) = (img.width as usize, img.height as usize);
    let c = img.channels as usize;
    let (x0, y0) = (p.x.floor(), p.y.floor());
    if x0 < 0.0 || y0 < 0.0 || x0 as usize + 1 >= w || y0 as usize + 1 >= h {
        return None;
    }
    let (fx, fy) = (p.x - x0, p.y - y0);
    let i = (y0 as usize * w + x0 as usize) * c;
    let row = w * c;
    let d = &img.data;
    let mut out = [0f32; 3];
    for (k, o) in out.iter_mut().enumerate() {
        let k = if c == 1 { 0 } else { k };
        let top = d[i + k] as f32 * (1.0 - fx) + d[i + c + k] as f32 * fx;
        let bottom = d[i + row + k] as f32 * (1.0 - fx) + d[i + row + c + k] as f32 * fx;
        *o = top * (1.0 - fy) + bottom * fy;
    }
    Some(out)
}

/// Decodes a JPEG into an RGB image.
pub fn decode_jpeg(jpeg: &[u8]) -> Option<RgbImage> {
    let rgb = image::load_from_memory_with_format(jpeg, image::ImageFormat::Jpeg).ok()?.to_rgb8();
    let (width, height) = rgb.dimensions();
    Some(RgbImage { width, height, channels: 3, data: rgb.into_raw() })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sharpness_drops_with_blur() {
        let mut img = Gray::new(64, 64);
        for y in 0..64 {
            for x in 0..64 {
                img.data[y * 64 + x] = if (x / 4 + y / 4) % 2 == 0 { 200 } else { 30 };
            }
        }
        let blurred = blur3(&blur3(&img));
        assert!(sharpness(&img) > 3.0 * sharpness(&blurred));
        assert_eq!(downsample2(&img).width, 32);
        assert!((img.sample(Vec2::new(1.5, 1.0)).unwrap() - 200.0).abs() < 1e-3);
    }
}
