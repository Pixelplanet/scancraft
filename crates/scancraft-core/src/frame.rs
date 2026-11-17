use crate::imu::ImuSample;
use crate::point::{ColorRgba, Point3D};
use crate::point_cloud::PointCloud;
use glam::Vec3;
use serde::{Deserialize, Serialize};

/// 2D Grayscale or RGB image buffer (e.g. from Preview / RGB Camera)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RgbImage {
    pub width: u32,
    pub height: u32,
    pub channels: u8, // 1 = Gray/IR, 3 = RGB, 4 = RGBA
    pub data: Vec<u8>,
}

/// Pinhole camera model of the image a depth map was rendered in, at the depth map's resolution.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct PinholeIntrinsics {
    pub fx: f32,
    pub fy: f32,
    pub cx: f32,
    pub cy: f32,
}

/// Options for [`DepthMap::to_point_cloud_with`].
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct BackProjection {
    pub min_mm: f32,
    pub max_mm: f32,
    /// Drop "flying pixels" at depth discontinuities (`None` keeps every pixel in range).
    pub edge: Option<EdgeFilter>,
}

/// Flying-pixel test: depth sensors interpolate between foreground and background at
/// silhouettes, producing points floating in free space. A pixel is dropped when, along any of
/// the four image directions within `radius_px`, the depth jumps by more than
/// `(step_mm + step_per_mm_depth * depth) * distance_px`.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct EdgeFilter {
    pub step_mm: f32,
    pub step_per_mm_depth: f32,
    pub radius_px: u32,
    /// Also drop pixels next to a pixel without depth (the borders of holes and of the
    /// measured area, where the sensor is least reliable).
    pub drop_hole_borders: bool,
}

impl Default for EdgeFilter {
    fn default() -> Self {
        Self { step_mm: 1.5, step_per_mm_depth: 0.004, radius_px: 2, drop_hole_borders: true }
    }
}

impl BackProjection {
    /// Every pixel in `[min_mm, max_mm]`, no edge filter (the behaviour of `to_point_cloud`).
    pub fn new(min_mm: f32, max_mm: f32) -> Self {
        Self { min_mm, max_mm, edge: None }
    }

    /// In range, with the default flying-pixel filter.
    pub fn filtered(min_mm: f32, max_mm: f32) -> Self {
        Self { min_mm, max_mm, edge: Some(EdgeFilter::default()) }
    }
}

/// 16-bit depth map buffer in camera space
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DepthMap {
    pub width: u32,
    pub height: u32,
    /// Scale factor to convert raw uint16 to millimeters (e.g. 0.01 mm for POP 4)
    pub scale_mm: f32,
    /// 16-bit depth values per pixel (row-major), 0 = no measurement
    pub data: Vec<u16>,
    /// Camera model needed to back-project pixels; supplied by the driver from device calibration.
    pub intrinsics: Option<PinholeIntrinsics>,
}

impl DepthMap {
    pub fn new(width: u32, height: u32, scale_mm: f32, data: Vec<u16>) -> Self {
        Self {
            width,
            height,
            scale_mm,
            data,
            intrinsics: None,
        }
    }

    pub fn with_intrinsics(mut self, intrinsics: PinholeIntrinsics) -> Self {
        self.intrinsics = Some(intrinsics);
        self
    }

    pub fn valid_pixel_count(&self) -> usize {
        self.data.iter().filter(|&&d| d != 0).count()
    }

    /// Back-projects every pixel with depth in `[min_mm, max_mm]` into scanner space, in mm.
    ///
    /// Axes are Z-up: +X right, +Y away from the scanner (optical axis), +Z up.
    /// Normals come from neighbouring pixels and face the scanner. Returns an empty cloud
    /// when the map carries no intrinsics. Keeps every pixel, including "flying pixels" at depth
    /// discontinuities; see [`to_point_cloud_with`](Self::to_point_cloud_with) to drop those.
    pub fn to_point_cloud(&self, min_mm: f32, max_mm: f32) -> PointCloud {
        self.to_point_cloud_with(&BackProjection::new(min_mm, max_mm))
    }

    /// Pixels kept by `options`: in range and, with the edge filter enabled, not at a depth
    /// discontinuity. Row-major, same size as `data`.
    pub fn valid_mask(&self, options: &BackProjection) -> Vec<bool> {
        let (w, h) = (self.width as usize, self.height as usize);
        let in_range = |raw: u16| {
            let z = raw as f32 * self.scale_mm;
            raw != 0 && z >= options.min_mm && z <= options.max_mm
        };
        let mut mask: Vec<bool> = self.data.iter().map(|&r| in_range(r)).collect();
        let Some(edge) = options.edge else { return mask };
        let r = edge.radius_px.max(1) as i64;
        let src = mask.clone();
        for v in 0..h {
            for u in 0..w {
                let i = v * w + u;
                if !src[i] {
                    continue;
                }
                let z = self.data[i] as f32 * self.scale_mm;
                // A step larger than this between pixels `d` apart is a discontinuity: an absolute
                // part (noise) plus a part growing with depth (pixel footprint, slanted surfaces).
                let limit = edge.step_mm + edge.step_per_mm_depth * z;
                let mut flying = false;
                'dirs: for (du, dv) in [(1i64, 0i64), (-1, 0), (0, 1), (0, -1)] {
                    for d in 1..=r {
                        let (nu, nv) = (u as i64 + du * d, v as i64 + dv * d);
                        if nu < 0 || nv < 0 || nu >= w as i64 || nv >= h as i64 {
                            break;
                        }
                        let j = nv as usize * w + nu as usize;
                        if !src[j] {
                            if edge.drop_hole_borders && d == 1 {
                                flying = true;
                                break 'dirs;
                            }
                            break;
                        }
                        let nz = self.data[j] as f32 * self.scale_mm;
                        if (nz - z).abs() > limit * d as f32 {
                            flying = true;
                            break 'dirs;
                        }
                    }
                }
                if flying {
                    mask[i] = false;
                }
            }
        }
        mask
    }

    /// Back-projection with options (range, flying-pixel filter). See
    /// [`to_point_cloud`](Self::to_point_cloud) for axes and normals; normals are estimated only
    /// from kept pixels, so they do not bend across discontinuities.
    pub fn to_point_cloud_with(&self, options: &BackProjection) -> PointCloud {
        let Some(k) = self.intrinsics else {
            return PointCloud::new();
        };
        let (w, h) = (self.width as usize, self.height as usize);
        let mask = self.valid_mask(options);
        let unproject = |u: usize, v: usize| -> Option<Vec3> {
            if !mask[v * w + u] {
                return None;
            }
            let z = self.data[v * w + u] as f32 * self.scale_mm;
            let x = (u as f32 - k.cx) * z / k.fx;
            let y = (v as f32 - k.cy) * z / k.fy;
            // Optical frame (x right, y down, z forward) -> Z-up scanner frame.
            Some(Vec3::new(x, z, -y))
        };

        let mut cloud = PointCloud::new();
        for v in 0..h {
            for u in 0..w {
                let Some(p) = unproject(u, v) else { continue };
                // Tangents over a few pixels: one-pixel differences are dominated by depth noise
                // (~0.3 mm noise vs ~0.35 mm pixel pitch on the POP 4). Central difference where
                // both sides exist, one-sided otherwise.
                const SPAN: usize = 2;
                let tangent = |fwd: Option<Vec3>, back: Option<Vec3>| match (fwd, back) {
                    (Some(f), Some(b)) => Some(f - b),
                    (Some(f), None) => Some(f - p),
                    (None, Some(b)) => Some(p - b),
                    (None, None) => None,
                };
                let along_u = tangent(
                    (u + SPAN < w).then(|| unproject(u + SPAN, v)).flatten(),
                    (u >= SPAN).then(|| unproject(u - SPAN, v)).flatten(),
                );
                let along_v = tangent(
                    (v + SPAN < h).then(|| unproject(u, v + SPAN)).flatten(),
                    (v >= SPAN).then(|| unproject(u, v - SPAN)).flatten(),
                );
                let mut normal = match (along_u, along_v) {
                    (Some(tu), Some(tv)) => tv.cross(tu).normalize_or_zero(),
                    _ => Vec3::ZERO,
                };
                if normal == Vec3::ZERO {
                    normal = -p.normalize_or_zero();
                } else if normal.dot(p) > 0.0 {
                    normal = -normal;
                }
                cloud.push(Point3D::new(p.to_array()).with_normal(normal.to_array()).with_color(ColorRgba::GRAY));
            }
        }
        cloud
    }

    #[inline]
    pub fn get_depth_mm(&self, x: u32, y: u32) -> Option<f32> {
        if x < self.width && y < self.height {
            let raw = self.data[(y * self.width + x) as usize];
            if raw > 0 {
                Some((raw as f32) * self.scale_mm)
            } else {
                None
            }
        } else {
            None
        }
    }
}

/// High-level preview frame containing RGB/IR imagery and telemetry
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PreviewFrame {
    pub frame_index: u64,
    pub timestamp_ms: u64,
    pub ir_image: Option<RgbImage>,
    pub rgb_image: Option<RgbImage>,
    pub average_distance_mm: Option<f32>,
    pub imu: Option<ImuSample>,
}

/// Rectified stereo geometry shared by every frame of a stream (cheap to clone).
/// Both images of a rectified pair use `intrinsics` except for the right principal point
/// (`cx + disparity_offset_px`); the right camera sits `baseline_mm` along +x of the left one.
/// With d = u_left - u_right, Z = fx * baseline_mm / (d + disparity_offset_px).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct StereoRig {
    pub width: u32,
    pub height: u32,
    pub intrinsics: PinholeIntrinsics,
    pub baseline_mm: f32,
    /// Rectified principal-point offset between the cameras: `cx_right - cx_left` in pixels at this
    /// rig's resolution (POP 4 red set at 640x512: ~ +305). With d = u_left - u_right:
    /// Z = fx * baseline_mm / (d + disparity_offset_px).
    #[serde(default)]
    pub disparity_offset_px: f32,
    /// Free-form name of the calibration set the rig came from (e.g. "red" or "blue" on
    /// Revopoint devices), so host algorithms can tell which projector it belongs to.
    pub calibration_set: String,
    /// Lens models mapping raw pixels into this rig, for streams whose images are not
    /// rectified (`IrImages::rectified == false`). Also carries the right image's rectified
    /// principal point when it differs from `intrinsics.cx`.
    #[serde(default)]
    pub rectification: Option<crate::stereo::StereoRectification>,
}

impl StereoRig {
    /// Z depth (mm) of disparity `d = u_left - u_right`; `None` if `d + disparity_offset_px <= 0`.
    pub fn depth_from_disparity(&self, d: f32) -> Option<f32> {
        let t = d + self.disparity_offset_px;
        (t > 0.0).then(|| self.intrinsics.fx * self.baseline_mm / t)
    }

    /// Disparity `u_left - u_right` of a point at Z depth `z_mm`.
    pub fn disparity_from_depth(&self, z_mm: f32) -> f32 {
        self.intrinsics.fx * self.baseline_mm / z_mm - self.disparity_offset_px
    }
}

/// Phase-shifted fringe images of one exposure set (e.g. Revopoint Hybrid HD), rectified to the
/// same rig as `SensorFrame::ir`. `left[k]` / `right[k]` were lit with the pattern shifted by
/// `k * 2π / left.len()`.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FringeImages {
    pub left: Vec<RgbImage>,
    pub right: Vec<RgbImage>,
}

/// Infrared images of one exposure.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct IrImages {
    pub left: RgbImage,
    pub right: Option<RgbImage>,
    /// True if both images are already rectified to `rig` (row-aligned epipolar lines).
    pub rectified: bool,
    pub rig: Option<std::sync::Arc<StereoRig>>,
}

/// One colour camera image.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ColorFrame {
    pub image: RgbImage,
    /// The compressed image as received (kept for texture / splat storage), if any.
    pub jpeg: Option<std::sync::Arc<Vec<u8>>>,
    /// Capture time on the device clock, ms: the same clock as `SensorFrame::timestamp_ms` (on the
    /// POP 4 both streams carry the device's millisecond counter; docs/protocol/color.md).
    pub timestamp_ms: u64,
    /// Calibration of the camera that took the image (intrinsics at `image`'s resolution,
    /// distortion, extrinsic from the depth frame), if known.
    pub camera: Option<std::sync::Arc<crate::color_camera::ColorCamera>>,
}

/// Everything a scanner delivers for one capture instant. Which parts are present depends on
/// the scan mode: Full-field HD fills `depth` (+ `ir`), stereo and laser modes fill `ir` and leave
/// depth to a host depth source, colour recording adds `color`.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SensorFrame {
    pub frame_index: u64,
    /// Device clock, ms.
    pub timestamp_ms: u64,
    pub depth: Option<DepthMap>,
    pub ir: Option<IrImages>,
    pub color: Option<ColorFrame>,
    pub imu: Option<ImuSample>,
    /// Phase-shifted fringe images (Hybrid HD), if the mode delivers them.
    #[serde(default)]
    pub fringes: Option<FringeImages>,
    /// A second IR pair of the same instant lit by the dot projector, for modes whose main pair
    /// (`ir`) shows something sparse (Cross Lines with dots: `ir` = laser lines, `dots` = dense
    /// texture for depth and tracking).
    #[serde(default)]
    pub dots: Option<IrImages>,
}

impl SensorFrame {
    pub fn new(frame_index: u64, timestamp_ms: u64) -> Self {
        Self { frame_index, timestamp_ms, depth: None, ir: None, color: None, imu: None, fringes: None, dots: None }
    }
}
