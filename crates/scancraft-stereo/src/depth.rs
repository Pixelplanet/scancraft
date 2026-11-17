//! IR pair → dense depth map, for any rectified [`StereoRig`].

use crate::sgm::{DisparityImage, SgmMatcher, SgmParams};
use scancraft_core::{DepthMap, IrImages, StereoRig};

/// Depth unit of the maps produced here: 0.02 mm keeps sub-0.1 mm resolution and reaches
/// 1.31 m in a u16 (the POP 4's on-device 0.01 mm unit would overflow at 655 mm).
pub const DEPTH_SCALE_MM: f32 = 0.02;

#[derive(Debug, Clone, PartialEq)]
pub struct StereoDepthConfig {
    /// Working range, mm. Sets the disparity search window; depth outside is dropped.
    pub min_mm: f32,
    pub max_mm: f32,
    /// Matcher settings. `min_disparity` / `num_disparities` are derived from the range and rig.
    pub sgm: SgmParams,
    /// Match at half resolution first and refine at full resolution (`hier.rs`): several times
    /// faster than full-resolution SGM at about the same coverage and noise.
    pub coarse_to_fine: bool,
}

impl StereoDepthConfig {
    pub fn new(min_mm: f32, max_mm: f32) -> Self {
        Self { min_mm, max_mm, sgm: SgmParams::default(), coarse_to_fine: true }
    }
}

/// Matcher plus disparity→depth conversion. Keeps its buffers across frames.
pub struct StereoDepth {
    pub config: StereoDepthConfig,
    matcher: SgmMatcher,
    hierarchical: crate::hier::HierarchicalMatcher,
    /// The most recent disparity image (for diagnostics and other consumers).
    pub last_disparity: Option<DisparityImage>,
}

/// Disparity window (`x_left - x_right`, px) that covers depths `min_mm..max_mm` on `rig`,
/// rounded out to whole pixels plus one pixel of margin for sub-pixel interpolation.
pub fn disparity_window(rig: &StereoRig, min_mm: f32, max_mm: f32) -> (i32, usize) {
    let near = rig.disparity_from_depth(min_mm.max(1.0));
    let far = rig.disparity_from_depth(max_mm.max(min_mm + 1.0));
    let lo = far.floor() as i32 - 1;
    let hi = near.ceil() as i32 + 1;
    // Multiples of 16 keep the per-pixel disparity loops vector-friendly.
    let n = ((hi - lo + 1) as usize).div_ceil(16) * 16;
    (lo, n)
}

impl StereoDepth {
    pub fn new(config: StereoDepthConfig) -> Self {
        let matcher = SgmMatcher::new(config.sgm.clone());
        Self { config, matcher, hierarchical: crate::hier::HierarchicalMatcher::new(), last_disparity: None }
    }

    /// Dense depth of the left (rectified) image, with the rig's intrinsics attached, or `None`
    /// if the frame has no usable rectified pair.
    pub fn compute(&mut self, ir: &IrImages) -> Option<DepthMap> {
        let rig = ir.rig.as_ref()?;
        let right = ir.right.as_ref()?;
        let (w, h) = (ir.left.width as usize, ir.left.height as usize);
        if !ir.rectified || ir.left.channels != 1 || right.channels != 1 || right.width != ir.left.width || right.height != ir.left.height {
            return None;
        }
        if rig.width != ir.left.width || rig.height != ir.left.height {
            return None;
        }
        let (lo, n) = disparity_window(rig, self.config.min_mm, self.config.max_mm);
        let params = SgmParams { min_disparity: lo, num_disparities: n, ..self.config.sgm.clone() };
        let disp = if self.config.coarse_to_fine {
            self.hierarchical.compute(&ir.left.data, &right.data, w, h, &params)
        } else {
            self.matcher.params = params;
            self.matcher.compute(&ir.left.data, &right.data, w, h)
        };
        let depth = disparity_to_depth(&disp, rig, self.config.min_mm, self.config.max_mm);
        self.last_disparity = Some(disp);
        Some(depth)
    }
}

/// Converts disparities to Z depth (`Z = fx * B / (d + disparity_offset_px)`), keeping only
/// `min_mm..=max_mm`.
pub fn disparity_to_depth(disp: &DisparityImage, rig: &StereoRig, min_mm: f32, max_mm: f32) -> DepthMap {
    let data = disp
        .data
        .iter()
        .map(|&d| match rig.depth_from_disparity(d) {
            Some(z) if !d.is_nan() && (min_mm..=max_mm).contains(&z) => (z / DEPTH_SCALE_MM).round().min(u16::MAX as f32) as u16,
            _ => 0,
        })
        .collect();
    DepthMap::new(disp.width as u32, disp.height as u32, DEPTH_SCALE_MM, data).with_intrinsics(rig.intrinsics)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sgm::tests::{render_right, texture};
    use scancraft_core::{PinholeIntrinsics, RgbImage};
    use std::sync::Arc;

    fn rig(w: u32, h: u32, offset: f32) -> StereoRig {
        StereoRig {
            width: w,
            height: h,
            intrinsics: PinholeIntrinsics { fx: 200.0, fy: 200.0, cx: w as f32 / 2.0, cy: h as f32 / 2.0 },
            baseline_mm: 50.0,
            disparity_offset_px: offset,
            calibration_set: "test".into(),
            rectification: None,
        }
    }

    #[test]
    fn window_covers_range() {
        let r = rig(160, 64, 0.0);
        // fB = 10000: 250 mm -> 40 px, 500 mm -> 20 px.
        let (lo, n) = disparity_window(&r, 250.0, 500.0);
        assert!(lo <= 20 && lo >= 18, "{lo}");
        assert!(lo + n as i32 >= 41);
        let shifted = rig(160, 64, 30.0);
        let (lo2, _) = disparity_window(&shifted, 250.0, 500.0);
        assert_eq!(lo2, lo - 30);
    }

    #[test]
    fn plane_at_known_depth_with_principal_point_offset() {
        let (w, h) = (160u32, 64u32);
        let offset = 12.0;
        let r = Arc::new(rig(w, h, offset));
        let z = 400.0; // true disparity 25 px, raw disparity 13 px
        let left = texture(w as usize, h as usize, 7);
        let right = render_right(&left, w as usize, h as usize, |_, _| 10000.0 / z - offset);
        let ir = IrImages {
            left: RgbImage { width: w, height: h, channels: 1, data: left },
            right: Some(RgbImage { width: w, height: h, channels: 1, data: right }),
            rectified: true,
            rig: Some(r),
        };
        let mut sd = StereoDepth::new(StereoDepthConfig::new(250.0, 600.0));
        let depth = sd.compute(&ir).expect("depth");
        let vals: Vec<f32> = (0..depth.data.len()).filter_map(|i| depth.get_depth_mm(i as u32 % w, i as u32 / w)).collect();
        assert!(vals.len() > (w * h) as usize / 2, "coverage {}", vals.len());
        let mean = vals.iter().sum::<f32>() / vals.len() as f32;
        assert!((mean - z).abs() < 2.0, "mean depth {mean}");
        assert!(depth.intrinsics.is_some());
    }
}
