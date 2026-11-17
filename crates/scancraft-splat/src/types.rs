//! The data contract: posed colour images plus an initial point cloud.
//!
//! These types are deliberately independent of the colour-pass code. Whatever the colour pass
//! stores per RGB keyframe has to be convertible into a [`PosedImage`]; see
//! `docs/architecture/splatting.md` ("Data contract") for the mapping.

use glam::{Affine3A, Mat3, Vec3};
use scancraft_core::PointCloud;

/// Pinhole intrinsics in pixels of the image they belong to (not normalised).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Intrinsics {
    pub fx: f32,
    pub fy: f32,
    /// Principal point, pixel coordinates with (0, 0) at the *corner* of the top-left pixel
    /// (OpenCV convention: the centre of the top-left pixel is (0.5, 0.5) in this system only if
    /// the calibration says so; we pass the value through unchanged).
    pub cx: f32,
    pub cy: f32,
    /// Lens distortion, OpenCV model. `None` means the image is already undistorted.
    pub distortion: Option<Distortion>,
}

/// OpenCV radial-tangential distortion (`k1 k2 p1 p2 k3`, the 5-coefficient form used by
/// `cv::calibrateCamera` and, presumably, the POP 4 `Distort.bin`).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Distortion {
    pub k1: f32,
    pub k2: f32,
    pub p1: f32,
    pub p2: f32,
    pub k3: f32,
}

impl Distortion {
    pub fn is_zero(&self) -> bool {
        [self.k1, self.k2, self.p1, self.p2, self.k3].iter().all(|v| *v == 0.0)
    }
}

/// Axis convention of the *camera-local* frame that `camera_to_world` maps from.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum CameraAxes {
    /// ScanCraft scanner frame, the one `DepthMap::to_point_cloud` produces and the tracker's
    /// poses map from: +X right, +Y forward (optical axis), +Z up (towards the top image row).
    #[default]
    ScanCraft,
    /// OpenCV / COLMAP optical frame: +X right, +Y down, +Z forward.
    OpenCv,
}

/// One colour keyframe with its calibration and pose.
#[derive(Debug, Clone)]
pub struct PosedImage {
    /// Stable name, used for exported file names (`<name>.png`). Must be unique in a dataset.
    pub name: String,
    pub width: u32,
    pub height: u32,
    /// Tightly packed RGB8, row-major, top row first. `len == width * height * 3`.
    pub rgb: Vec<u8>,
    /// Optional foreground mask, one byte per pixel (255 = object, 0 = background). With masks
    /// the splat is trained to be transparent outside the object (clean object splat).
    pub mask: Option<Vec<u8>>,
    pub intrinsics: Intrinsics,
    /// Camera-to-world transform in ScanCraft world units (mm, Z-up world). Rigid: rotation and
    /// translation only.
    pub camera_to_world: Affine3A,
    /// Convention of the camera-local frame of `camera_to_world`.
    pub axes: CameraAxes,
}

/// `p_scancraft_camera = M * p_opencv_camera`: x stays, OpenCV +Z (forward) becomes +Y, OpenCV +Y
/// (down) becomes −Z. Same mapping as `DepthMap::to_point_cloud`.
pub fn opencv_to_scancraft_camera() -> Mat3 {
    Mat3::from_cols(Vec3::X, Vec3::NEG_Z, Vec3::Y)
}

impl PosedImage {
    /// Camera-to-world with an OpenCV camera-local frame (x right, y down, z forward), still in
    /// ScanCraft world units. This is what COLMAP-style tools and Brush consume.
    pub fn camera_to_world_opencv(&self) -> Affine3A {
        match self.axes {
            CameraAxes::OpenCv => self.camera_to_world,
            CameraAxes::ScanCraft => {
                self.camera_to_world * Affine3A::from_mat3(opencv_to_scancraft_camera())
            }
        }
    }

    /// Camera centre in world units.
    pub fn position(&self) -> Vec3 {
        self.camera_to_world.translation.into()
    }

    /// Project a world point to pixel coordinates (distortion ignored). Returns `(u, v, depth)`
    /// with `depth` along the optical axis; `None` behind the camera.
    pub fn project(&self, world: Vec3) -> Option<(f32, f32, f32)> {
        let local = self.camera_to_world_opencv().inverse().transform_point3(world);
        if local.z <= 1e-6 {
            return None;
        }
        let k = &self.intrinsics;
        Some((k.fx * local.x / local.z + k.cx, k.fy * local.y / local.z + k.cy, local.z))
    }

    pub(crate) fn validate(&self) -> Result<(), crate::SplatError> {
        let n = self.width as usize * self.height as usize;
        if self.width == 0 || self.height == 0 || self.rgb.len() != n * 3 {
            return Err(crate::SplatError::InvalidInput(format!(
                "image '{}': {}x{} needs {} RGB bytes, got {}",
                self.name,
                self.width,
                self.height,
                n * 3,
                self.rgb.len()
            )));
        }
        if let Some(mask) = &self.mask {
            if mask.len() != n {
                return Err(crate::SplatError::InvalidInput(format!(
                    "image '{}': mask needs {n} bytes, got {}",
                    self.name,
                    mask.len()
                )));
            }
        }
        let k = &self.intrinsics;
        if !(k.fx > 0.0 && k.fy > 0.0 && k.cx.is_finite() && k.cy.is_finite()) {
            return Err(crate::SplatError::InvalidInput(format!("image '{}': bad intrinsics {k:?}", self.name)));
        }
        if !self.camera_to_world.is_finite() {
            return Err(crate::SplatError::InvalidInput(format!("image '{}': non-finite pose", self.name)));
        }
        Ok(())
    }

    /// RGB, or RGBA when a mask is present (alpha = mask).
    pub fn to_dynamic_image(&self) -> image::DynamicImage {
        match &self.mask {
            None => image::DynamicImage::ImageRgb8(
                image::RgbImage::from_raw(self.width, self.height, self.rgb.clone()).expect("validated size"),
            ),
            Some(mask) => {
                let rgba: Vec<u8> = self
                    .rgb
                    .as_chunks::<3>()
                    .0
                    .iter()
                    .zip(mask)
                    .flat_map(|(c, &a)| [c[0], c[1], c[2], a])
                    .collect();
                image::DynamicImage::ImageRgba8(
                    image::RgbaImage::from_raw(self.width, self.height, rgba).expect("validated size"),
                )
            }
        }
    }
}

/// Everything a splat is trained from.
#[derive(Debug, Clone, Default)]
pub struct SplatDataset {
    /// Colour keyframes of the aligned colour passes.
    pub images: Vec<PosedImage>,
    /// Merged point cloud of all aligned passes (geometry-only passes included), ScanCraft world
    /// units (mm). Colours seed the splat colours; points no image sees are dropped.
    pub init_cloud: PointCloud,
}

impl SplatDataset {
    pub fn validate(&self) -> Result<(), crate::SplatError> {
        if self.images.is_empty() {
            return Err(crate::SplatError::InvalidInput("no colour images".into()));
        }
        let mut names = std::collections::HashSet::new();
        for img in &self.images {
            img.validate()?;
            if !names.insert(img.name.as_str()) {
                return Err(crate::SplatError::InvalidInput(format!("duplicate image name '{}'", img.name)));
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn image(axes: CameraAxes, pose: Affine3A) -> PosedImage {
        PosedImage {
            name: "a".into(),
            width: 100,
            height: 80,
            rgb: vec![0; 100 * 80 * 3],
            mask: None,
            intrinsics: Intrinsics { fx: 100.0, fy: 100.0, cx: 50.0, cy: 40.0, distortion: None },
            camera_to_world: pose,
            axes,
        }
    }

    #[test]
    fn scancraft_axes_look_along_plus_y() {
        // Identity pose in the ScanCraft frame looks along world +Y with +Z up.
        let img = image(CameraAxes::ScanCraft, Affine3A::IDENTITY);
        let (u, v, d) = img.project(Vec3::new(0.0, 500.0, 0.0)).unwrap();
        assert!((u - 50.0).abs() < 1e-4 && (v - 40.0).abs() < 1e-4 && (d - 500.0).abs() < 1e-3);
        // A point above the axis lands in the upper half of the image (smaller v).
        let (_, v_up, _) = img.project(Vec3::new(0.0, 500.0, 50.0)).unwrap();
        assert!(v_up < 40.0);
        // A point to the right lands right.
        let (u_r, _, _) = img.project(Vec3::new(50.0, 500.0, 0.0)).unwrap();
        assert!(u_r > 50.0);
        assert!(img.project(Vec3::new(0.0, -500.0, 0.0)).is_none());
    }

    #[test]
    fn axes_conventions_agree() {
        let pose = Affine3A::from_rotation_translation(glam::Quat::from_rotation_z(0.7), Vec3::new(10.0, -300.0, 40.0));
        let a = image(CameraAxes::ScanCraft, pose);
        let b = image(CameraAxes::OpenCv, a.camera_to_world_opencv());
        for p in [Vec3::new(5.0, 20.0, 3.0), Vec3::new(-40.0, 10.0, 60.0)] {
            assert_eq!(a.project(p).map(|x| (x.0 * 1e3).round()), b.project(p).map(|x| (x.0 * 1e3).round()));
        }
    }
}
