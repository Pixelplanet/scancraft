//! Parsing of the stereo calibration stored on the scanner at `/data/camparam/camparam.yaml`.
//!
//! The on-device depth map is computed in the *rectified* left IR camera, so back-projection
//! must use the rectified projection matrix `pL`, not the raw `cameraMatrixL`. The calibration
//! is at full sensor resolution (`width` x `height`, 1280x1024 on the POP 4) and is scaled to the
//! streamed depth resolution.
//!
//! Some streams carry *unrectified* IR images (the speckle planes of Hybrid HD, see
//! `docs/protocol/hybrid.md`); [`RectifyMaps`] reproduces the device's rectification for them from
//! the raw intrinsics, distortion and stereo extrinsics in the same file (OpenCV/Bouguet
//! `stereoRectify` with `alpha = -1`, which reproduces `pL` / `pR` exactly).

use glam::{DMat3, DVec3};
use scancraft_core::{ColorCamera, PinholeIntrinsics, RawCamera, RgbImage, StereoRectification, StereoRig};
use std::sync::{Arc, Mutex};

/// All calibration a POP 4 stream may need. The "red" set (`camparam.yaml`) belongs to the
/// IR/coded-light/VCSEL path; the laser modes use the separate "blue" set
/// (`/data/camparam_blue/`, see `docs/protocol/laser.md`).
#[derive(Debug)]
pub struct DeviceCalibration {
    pub red: StereoCalibration,
    /// RGB camera relative to the rectified left IR camera of the red set (`Prgb.bin`,
    /// `Distort.bin`, `LC_RT.bin`), at the RGB sensor resolution. `None` if not downloaded.
    pub color: Option<Arc<ColorCamera>>,
    /// `None` if the device has no blue set or it could not be downloaded.
    pub blue: Option<BlueCalibration>,
    rigs: Mutex<Vec<Arc<StereoRig>>>,
    blue_rig: std::sync::OnceLock<Option<Arc<StereoRig>>>,
    maps: Mutex<Vec<Arc<RectifyMaps>>>,
}

impl DeviceCalibration {
    pub fn new(red: StereoCalibration) -> Self {
        Self { red, color: None, blue: None, rigs: Mutex::new(Vec::new()), blue_rig: std::sync::OnceLock::new(), maps: Mutex::new(Vec::new()) }
    }


    pub fn with_color(mut self, color: ColorCamera) -> Self {
        self.color = Some(Arc::new(color));
        self
    }


    pub fn with_blue(mut self, blue: BlueCalibration) -> Self {
        self.blue = Some(blue);
        self
    }

    /// The laser stream's rig (full resolution, unrectified images + lens models).
    pub fn blue_rig(&self) -> Option<Arc<StereoRig>> {
        self.blue_rig.get_or_init(|| self.blue.as_ref().map(|b| Arc::new(b.rig()))).clone()
    }

    /// Rectification maps of the red set at a stream resolution (cached), `None` if the
    /// calibration file lacks the raw camera model.
    pub fn red_rectify_maps(&self, width: u32, height: u32) -> Option<Arc<RectifyMaps>> {
        let mut maps = self.maps.lock().unwrap();
        if let Some(m) = maps.iter().find(|m| m.width == width && m.height == height) {
            return Some(m.clone());
        }
        let m = Arc::new(RectifyMaps::new(&self.red, width, height)?);
        maps.push(m.clone());
        Some(m)
    }

    /// Rectified stereo rig of the red set at a stream resolution (cached per resolution).
    pub fn red_rig(&self, width: u32, height: u32) -> Arc<StereoRig> {
        let mut rigs = self.rigs.lock().unwrap();
        if let Some(r) = rigs.iter().find(|r| r.width == width && r.height == height && r.calibration_set == "red") {
            return r.clone();
        }
        let rig = Arc::new(StereoRig {
            width,
            height,
            intrinsics: self.red.depth_intrinsics(width, height),
            baseline_mm: self.red.baseline_mm() as f32,
            disparity_offset_px: self.red.disparity_offset(width) as f32,
            calibration_set: "red".into(),
            rectification: None,
        });
        rigs.push(rig.clone());
        rig
    }
}

/// Device paths of the blue (laser) calibration set, as downloaded by Revo Scan
/// (`sdk.log_back` 2026-07-05 18:37:43, `zx_cmd.cgi?download=<path>`).
pub const BLUE_FILES: [&str; 4] = [
    "/data/camparam_blue/camparam.yaml",
    "/data/camparam_blue/mapparamL.bin",
    "/data/camparam_blue/mapparamR.bin",
    "/data/camparam_blue/metroExtra.bin",
];

/// The blue calibration set: stereo calibration of the IR cameras for the laser modes, the raw
/// lens models (the laser stream is not rectified) and the laser-line models in `metroExtra.bin`.
#[derive(Debug, Clone, PartialEq)]
pub struct BlueCalibration {
    pub stereo: StereoCalibration,
    pub rectification: StereoRectification,
    /// Raw `metroExtra.bin` (laser line models; layout in `docs/protocol/laser.md`).
    pub metro_extra: Vec<u8>,
}

impl BlueCalibration {
    /// Builds the set from the four files in [`BLUE_FILES`] order.
    pub fn parse(yaml: &str, map_left: &[u8], map_right: &[u8], metro_extra: Vec<u8>) -> Result<Self, String> {
        let stereo = StereoCalibration::parse(yaml).ok_or("blue camparam.yaml is missing pL/pR")?;
        let left = parse_map_param(map_left).ok_or("mapparamL.bin is malformed")?;
        let right = parse_map_param(map_right).ok_or("mapparamR.bin is malformed")?;
        Ok(Self { rectification: StereoRectification { left, right, right_cx: stereo.p_right[2] }, stereo, metro_extra })
    }

    /// Stereo rig of the laser stream at full sensor resolution, with the raw lens models.
    pub fn rig(&self) -> StereoRig {
        StereoRig {
            width: self.stereo.width,
            height: self.stereo.height,
            intrinsics: self.stereo.depth_intrinsics(self.stereo.width, self.stereo.height),
            baseline_mm: self.stereo.baseline_mm() as f32,
            disparity_offset_px: self.stereo.disparity_offset(self.stereo.width) as f32,
            calibration_set: "blue".into(),
            rectification: Some(self.rectification.clone()),
        }
    }
}

/// Downloads and parses the blue set from the scanner.
pub async fn download_blue(ctl: &crate::control::Control) -> Result<BlueCalibration, String> {
    download_blue_files(ctl).await.map(|(blue, _)| blue)
}

/// [`download_blue`], also returning the files as received, named `camparam_blue/<file>`.
pub async fn download_blue_files(ctl: &crate::control::Control) -> Result<(BlueCalibration, Vec<(String, Vec<u8>)>), String> {
    let mut files = Vec::new();
    for path in BLUE_FILES {
        files.push(ctl.download(path).await.map_err(|e| e.to_string())?);
    }
    let blue = parse_blue_files(&files)?;
    let named = BLUE_FILES.iter().zip(files).map(|(p, b)| (blue_file_name(p), b)).collect();
    Ok((blue, named))
}

/// A calibration file with the device's serial number blanked out, for copies that are saved
/// with a project: every occurrence of `serial` becomes as many `0`, and a `deviceID:` line of a
/// YAML file is emptied whatever it holds. The calibration values are untouched (nothing reads
/// the serial back).
pub fn redact_serial(bytes: &[u8], serial: &str) -> Vec<u8> {
    let mut out = bytes.to_vec();
    let serial = serial.trim().as_bytes();
    if serial.len() >= 4 {
        let mut i = 0;
        while i + serial.len() <= out.len() {
            if &out[i..i + serial.len()] == serial {
                out[i..i + serial.len()].fill(b'0');
                i += serial.len();
            } else {
                i += 1;
            }
        }
    }
    if let Ok(text) = std::str::from_utf8(&out) {
        if text.lines().any(|l| l.trim_start().starts_with("deviceID:")) {
            let mut lines: Vec<String> = text.split('\n').map(str::to_string).collect();
            for l in lines.iter_mut().filter(|l| l.trim_start().starts_with("deviceID:")) {
                *l = "deviceID: \"\"".into();
            }
            return lines.join("\n").into_bytes();
        }
    }
    out
}

/// `/data/camparam_blue/x.bin` -> `camparam_blue/x.bin` (the name in a pass's calibration folder).
pub fn blue_file_name(device_path: &str) -> String {
    device_path.trim_start_matches("/data/").to_string()
}

/// The blue set from its four files in [`BLUE_FILES`] order.
pub fn parse_blue_files(files: &[Vec<u8>]) -> Result<BlueCalibration, String> {
    if files.len() < 4 {
        return Err("the blue calibration needs four files".into());
    }
    let yaml = String::from_utf8(files[0].clone()).map_err(|_| "blue camparam.yaml is not text".to_string())?;
    BlueCalibration::parse(&yaml, &files[1], &files[2], files[3].clone())
}

/// `mapparam{L,R}.bin` (148 bytes, little endian): `i32 height, i32 width, i32 n_dist`,
/// `f32 R[9]` (identity on the POP 4), `f32 cx, cy, fx, fy`, `f32 dist[12]` (k1 k2 p1 p2 k3, rest 0),
/// `f32 iR[9]` = `(P_rect[:3,:3] * R_rect)^-1` as used by OpenCV `initUndistortRectifyMap`.
/// Layout inferred from the values: they reproduce `camparam.yaml`'s camera matrix and distortion,
/// and `K_rect^-1 * iR^-1` is a rotation (orthonormal to 1e-7).
pub fn parse_map_param(bytes: &[u8]) -> Option<RawCamera> {
    if bytes.len() != 148 {
        return None;
    }
    let f = |i: usize| f32::from_le_bytes(bytes[4 * i..4 * i + 4].try_into().unwrap()) as f64;
    let i32_at = |i: usize| i32::from_le_bytes(bytes[4 * i..4 * i + 4].try_into().unwrap());
    if i32_at(0) <= 0 || i32_at(1) <= 0 {
        return None;
    }
    let ir: [f64; 9] = std::array::from_fn(|k| f(28 + k));
    let raw_to_rectified = scancraft_core::stereo::invert3(&ir);
    if raw_to_rectified.iter().all(|&v| v == 0.0) {
        return None;
    }
    Some(RawCamera { cx: f(12), cy: f(13), fx: f(14), fy: f(15), distortion: [f(16), f(17), f(18), f(19), f(20)], raw_to_rectified })
}

#[derive(Debug, Clone, PartialEq)]
pub struct StereoCalibration {
    pub width: u32,
    pub height: u32,
    /// Rectified left projection matrix, row-major 3x4.
    pub p_left: [f64; 12],
    /// Rectified right projection matrix, row-major 3x4 (`p_right[3] = -f * baseline`).
    pub p_right: [f64; 12],
    /// Raw camera models and extrinsics, if present in the file.
    pub raw: Option<RawStereo>,
}

/// Unrectified camera models (`cameraMatrixL/R`, `distCoeffL/R`) and the right camera's pose
/// in the left frame (`steroRotation`, `steroTranslation`; X_right = R X_left + T), full resolution.
#[derive(Debug, Clone, PartialEq)]
pub struct RawStereo {
    pub k_left: [f64; 9],
    pub dist_left: [f64; 5],
    pub k_right: [f64; 9],
    pub dist_right: [f64; 5],
    pub rotation: [f64; 9],
    pub translation: [f64; 3],
}

impl StereoCalibration {
    /// Parses the OpenCV-style YAML served by `zx_cmd.cgi?download=/data/camparam/camparam.yaml`.
    pub fn parse(yaml: &str) -> Option<Self> {
        let width = scalar(yaml, "width")?;
        let height = scalar(yaml, "height")?;
        Some(Self {
            width,
            height,
            p_left: matrix(yaml, "pL")?.try_into().ok()?,
            p_right: matrix(yaml, "pR")?.try_into().ok()?,
            raw: raw_stereo(yaml),
        })
    }

    /// `cx_right - cx_left` of the rectified pair at a stream width (+305 px at 640x512 on the
    /// POP 4, whose cameras are verged by ~20°); see `StereoRig::disparity_offset_px`.
    pub fn disparity_offset_px(&self, width: u32) -> f64 {
        (self.p_right[2] - self.p_left[2]) * width as f64 / self.width as f64
    }

    /// Rectified left-camera intrinsics at the given stream resolution.
    pub fn depth_intrinsics(&self, width: u32, height: u32) -> PinholeIntrinsics {
        let sx = width as f64 / self.width as f64;
        let sy = height as f64 / self.height as f64;
        let p = &self.p_left;
        PinholeIntrinsics {
            fx: (p[0] * sx) as f32,
            fy: (p[5] * sy) as f32,
            cx: (p[2] * sx) as f32,
            cy: (p[6] * sy) as f32,
        }
    }

    /// Right minus left rectified principal point x at a stream width, px (`pR[0][2] - pL[0][2]`,
    /// scaled). Non-zero on the POP 4: the pair is rectified without equalising principal points.
    pub fn disparity_offset(&self, width: u32) -> f64 {
        (self.p_right[2] - self.p_left[2]) * width as f64 / self.width as f64
    }

    /// Stereo baseline in mm, from `pR[0][3] = -f * B`.
    pub fn baseline_mm(&self) -> f64 {
        -self.p_right[3] / self.p_right[0]
    }
}

/// Parses the RGB camera calibration files of `/data/camparam/` (docs/protocol/color.md):
///
/// - `Prgb.bin` (40 B): `u16 width, u16 height`, then `f32[9]` row-major K at that size.
/// - `Distort.bin` (20 B): `f32[5]` OpenCV `k1 k2 p1 p2 k3`.
/// - `LC_RT.bin` (48 B): `f32[9]` row-major R, `f32[3]` T (mm), mapping the rectified left IR
///   optical frame to the RGB optical frame, `p_rgb = R p_left + T` (direction verified by
///   projecting depth into RGB, see `examples/color_calib_check.rs` in `scancraft-color`).
///
/// All little-endian. Returns `None` for files of the wrong size or a non-rotation R.
pub fn parse_color_calibration(prgb: &[u8], distort: &[u8], lc_rt: &[u8]) -> Option<ColorCamera> {
    if prgb.len() != 40 || distort.len() != 20 || lc_rt.len() != 48 {
        return None;
    }
    let f = |b: &[u8], i: usize| f32::from_le_bytes(b[4 * i..4 * i + 4].try_into().unwrap());
    let width = u16::from_le_bytes([prgb[0], prgb[1]]) as u32;
    let height = u16::from_le_bytes([prgb[2], prgb[3]]) as u32;
    let k = |i: usize| f(&prgb[4..], i);
    let intrinsics = PinholeIntrinsics { fx: k(0), fy: k(4), cx: k(2), cy: k(5) };
    let distortion = [f(distort, 0), f(distort, 1), f(distort, 2), f(distort, 3), f(distort, 4)];
    let r = |row: usize, col: usize| f(lc_rt, row * 3 + col);
    let rotation = glam::Mat3::from_cols(
        glam::Vec3::new(r(0, 0), r(1, 0), r(2, 0)),
        glam::Vec3::new(r(0, 1), r(1, 1), r(2, 1)),
        glam::Vec3::new(r(0, 2), r(1, 2), r(2, 2)),
    );
    let translation = glam::Vec3::new(f(lc_rt, 9), f(lc_rt, 10), f(lc_rt, 11));
    let orthonormal = (rotation * rotation.transpose() - glam::Mat3::IDENTITY).to_cols_array().iter().all(|v| v.abs() < 1e-3);
    let sane = width > 0 && height > 0 && intrinsics.fx > 0.0 && intrinsics.fy > 0.0 && orthonormal && (rotation.determinant() - 1.0).abs() < 1e-3;
    sane.then_some(ColorCamera { width, height, intrinsics, distortion, rotation, translation })
}

fn raw_stereo(yaml: &str) -> Option<RawStereo> {
    Some(RawStereo {
        k_left: matrix(yaml, "cameraMatrixL")?.try_into().ok()?,
        dist_left: matrix(yaml, "distCoeffL")?.try_into().ok()?,
        k_right: matrix(yaml, "cameraMatrixR")?.try_into().ok()?,
        dist_right: matrix(yaml, "distCoeffR")?.try_into().ok()?,
        rotation: matrix(yaml, "steroRotation")?.try_into().ok()?,
        translation: matrix(yaml, "steroTranslation")?.try_into().ok()?,
    })
}

/// Per-pixel source coordinates that turn raw (distorted, unrectified) images into the rectified
/// pair described by `pL` / `pR`, at a stream resolution.
#[derive(Debug)]
pub struct RectifyMaps {
    pub width: u32,
    pub height: u32,
    /// For every rectified pixel (row-major), the raw-image position to sample, in pixels.
    pub left: Vec<[f32; 2]>,
    pub right: Vec<[f32; 2]>,
}

impl RectifyMaps {
    pub fn new(cal: &StereoCalibration, width: u32, height: u32) -> Option<Self> {
        let raw = cal.raw.as_ref()?;
        let (r1, r2) = bouguet_rectification(&raw.rotation, &raw.translation);
        let sx = width as f64 / cal.width as f64;
        let sy = height as f64 / cal.height as f64;
        let map = |k: &[f64; 9], dist: &[f64; 5], rect: DMat3, p: &[f64; 12]| -> Vec<[f32; 2]> {
            let (fx, fy, cx, cy) = (k[0] * sx, k[4] * sy, k[2] * sx, k[5] * sy);
            let (pfx, pfy, pcx, pcy) = (p[0] * sx, p[5] * sy, p[2] * sx, p[6] * sy);
            let rt = rect.transpose();
            let [k1, k2, p1, p2, k3] = *dist;
            let mut out = Vec::with_capacity((width * height) as usize);
            for v in 0..height {
                for u in 0..width {
                    let ray = rt * DVec3::new((u as f64 - pcx) / pfx, (v as f64 - pcy) / pfy, 1.0);
                    let (x, y) = (ray.x / ray.z, ray.y / ray.z);
                    let r2 = x * x + y * y;
                    let radial = 1.0 + r2 * (k1 + r2 * (k2 + r2 * k3));
                    let xd = x * radial + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x);
                    let yd = y * radial + p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y;
                    out.push([(fx * xd + cx) as f32, (fy * yd + cy) as f32]);
                }
            }
            out
        };
        Some(Self {
            width,
            height,
            left: map(&raw.k_left, &raw.dist_left, r1, &cal.p_left),
            right: map(&raw.k_right, &raw.dist_right, r2, &cal.p_right),
        })
    }

    /// Rectifies a raw 8-bit image of the left (`left = true`) or right camera (bilinear; pixels
    /// that map outside the raw image become 0).
    pub fn rectify(&self, raw: &RgbImage, left: bool) -> RgbImage {
        let map = if left { &self.left } else { &self.right };
        let (w, h) = (raw.width as usize, raw.height as usize);
        let data = map
            .iter()
            .map(|&[x, y]| {
                if !(x >= 0.0 && y >= 0.0 && x < (w - 1) as f32 && y < (h - 1) as f32) {
                    return 0;
                }
                let (x0, y0) = (x as usize, y as usize);
                let (fx, fy) = (x - x0 as f32, y - y0 as f32);
                let p = |xx: usize, yy: usize| raw.data[yy * w + xx] as f32;
                let top = p(x0, y0) + (p(x0 + 1, y0) - p(x0, y0)) * fx;
                let bottom = p(x0, y0 + 1) + (p(x0 + 1, y0 + 1) - p(x0, y0 + 1)) * fx;
                (top + (bottom - top) * fy + 0.5) as u8
            })
            .collect();
        RgbImage { width: self.width, height: self.height, channels: 1, data }
    }
}

/// OpenCV `stereoRectify` rotations (Bouguet, horizontal rig): splits the stereo rotation in
/// half, then aligns the baseline with the image x axis. Returns (R1, R2): raw camera frame ->
/// rectified frame for the left and right camera.
fn bouguet_rectification(rotation: &[f64; 9], translation: &[f64; 3]) -> (DMat3, DMat3) {
    let r = DMat3::from_cols_array(rotation).transpose(); // row-major -> glam column-major
    let half = rodrigues_vector(&r) * -0.5;
    let r_r = rodrigues_matrix(half);
    let t = r_r * DVec3::from_array(*translation);
    let uu = DVec3::new(if t.x > 0.0 { 1.0 } else { -1.0 }, 0.0, 0.0);
    let mut ww = t.cross(uu);
    let nw = ww.length();
    if nw > 0.0 {
        ww *= (t.x.abs() / t.length()).acos() / nw;
    }
    let w_r = rodrigues_matrix(ww);
    (w_r * r_r.transpose(), w_r * r_r)
}

fn rodrigues_matrix(v: DVec3) -> DMat3 {
    let angle = v.length();
    if angle < 1e-12 {
        return DMat3::IDENTITY;
    }
    DMat3::from_axis_angle(v / angle, angle)
}

fn rodrigues_vector(r: &DMat3) -> DVec3 {
    let cos = ((r.x_axis.x + r.y_axis.y + r.z_axis.z - 1.0) * 0.5).clamp(-1.0, 1.0);
    let angle = cos.acos();
    if angle < 1e-12 {
        return DVec3::ZERO;
    }
    // Axis from the skew-symmetric part (the rig's rotation is far below pi).
    let axis = DVec3::new(r.y_axis.z - r.z_axis.y, r.z_axis.x - r.x_axis.z, r.x_axis.y - r.y_axis.x) / (2.0 * angle.sin());
    axis * angle
}

/// Reads `key: "123"` or `key: 123` at the start of a line.
fn scalar(yaml: &str, key: &str) -> Option<u32> {
    let prefix = format!("{key}:");
    yaml.lines()
        .map(str::trim_start)
        .find_map(|l| l.strip_prefix(&prefix))
        .and_then(|v| v.trim().trim_matches('"').parse().ok())
}

/// Reads the `data: [ ... ]` list of the `key: !!opencv-matrix` block (may span lines).
fn matrix(yaml: &str, key: &str) -> Option<Vec<f64>> {
    let header = format!("{key}: !!opencv-matrix");
    let block = &yaml[yaml.find(&header)? + header.len()..];
    let data = &block[block.find("data:")? + 5..];
    let list = &data[data.find('[')? + 1..data.find(']')?];
    list.split(',').map(|s| s.trim().parse().ok()).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    // Excerpt of a real POP 4 camparam.yaml (device ID removed).
    const YAML: &str = r#"%YAML:1.0
---
width: "1280"
height: "1024"
cameraMatrixL: !!opencv-matrix
   rows: 3
   cols: 3
   dt: d
   data: [ 1.4851854240864188e+03, 0., 6.5040311256885661e+02, 0.,
       1.4851854240864188e+03, 4.7756338666589892e+02, 0., 0., 1. ]
pL: !!opencv-matrix
   rows: 3
   cols: 4
   dt: d
   data: [ 1.4845567582803628e+03, 0., 3.4120636749267578e+02, 0., 0.,
       1.4845567582803628e+03, 4.8513208770751953e+02, 0., 0., 0., 1.,
       0. ]
pR: !!opencv-matrix
   rows: 3
   cols: 4
   dt: d
   data: [ 1.4845567582803628e+03, 0., 9.5103775787353516e+02,
       -1.7603769057400405e+05, 0., 1.4845567582803628e+03,
       4.8513208770751953e+02, 0., 0., 0., 1., 0. ]
"#;

    #[test]
    fn parses_rectified_intrinsics_at_stream_resolution() {
        let cal = StereoCalibration::parse(YAML).expect("parses");
        assert_eq!((cal.width, cal.height), (1280, 1024));

        let k = cal.depth_intrinsics(640, 512);
        assert!((k.fx - 742.278).abs() < 1e-2);
        assert!((k.fy - 742.278).abs() < 1e-2);
        assert!((k.cx - 170.603).abs() < 1e-2, "uses pL, not cameraMatrixL: {}", k.cx);
        assert!((k.cy - 242.566).abs() < 1e-2);

        assert!((cal.baseline_mm() - 118.58).abs() < 0.05);
        assert!((cal.disparity_offset(640) - 304.916).abs() < 1e-2);
        assert!(cal.raw.is_none(), "excerpt has no raw camera model");
    }

    /// The colour calibration files of the POP 4 used for development (values, not device ID).
    #[allow(clippy::excessive_precision)]
    pub(crate) fn pop4_color_files() -> (Vec<u8>, Vec<u8>, Vec<u8>) {
        let le = |v: &[f32]| v.iter().flat_map(|x| x.to_le_bytes()).collect::<Vec<u8>>();
        let mut prgb = vec![0x00, 0x05, 0x00, 0x04];
        prgb.extend(le(&[1564.9022, 0.0, 635.86377, 0.0, 1564.9022, 516.4419, 0.0, 0.0, 1.0]));
        let distort = le(&[0.13387248, -0.71496224, 0.0011797593, 9.81343e-5, 1.3389184]);
        let lc_rt = le(&[
            0.9948592, 0.0018471666, -0.10125059, -0.0019294930, 0.99999785, -0.00071516924, 0.10124905, 0.00090685504, 0.9948607,
            -24.099226, 0.08077016, -2.7144995,
        ]);
        (prgb, distort, lc_rt)
    }

    #[test]
    fn parses_color_calibration() {
        let (prgb, distort, lc_rt) = pop4_color_files();
        let cam = parse_color_calibration(&prgb, &distort, &lc_rt).expect("parses");
        assert_eq!((cam.width, cam.height), (1280, 1024));
        assert!((cam.intrinsics.fx - 1564.90).abs() < 0.01 && (cam.intrinsics.cy - 516.44).abs() < 0.01);
        assert!((cam.distortion[4] - 1.3389).abs() < 1e-4);
        // R is stored row-major: element (0, 2) is -0.1013.
        assert!((cam.rotation.col(2).x + 0.10125).abs() < 1e-5);
        assert!((cam.translation.x + 24.099).abs() < 1e-3);
        assert!(parse_color_calibration(&prgb[..36], &distort, &lc_rt).is_none());
    }

    /// Synthetic `mapparamL.bin` with the POP 4 layout (values of the real blue set, rounded).
    fn map_param_bytes() -> Vec<u8> {
        let mut b = Vec::new();
        for v in [1024i32, 1280, 5] {
            b.extend_from_slice(&v.to_le_bytes());
        }
        let mut f = vec![1.0f32, 0., 0., 0., 1., 0., 0., 0., 1.];
        f.extend([649.654, 478.18, 1484.032, 1483.45, 0.0712, -0.2102, 0.00064, -0.00016, 0.2219]);
        f.extend([0.0; 7]);
        f.extend([6.643_032e-4, 1.545_197e-6, -0.399_777, -1.528_299e-6, 6.744_576e-4, -0.326_701, 1.165_95e-4, 3.682_48e-8, 0.945_233]);
        for v in f {
            b.extend_from_slice(&v.to_le_bytes());
        }
        b
    }

    #[test]
    fn parses_map_param_lens_model() {
        let bytes = map_param_bytes();
        assert_eq!(bytes.len(), 148);
        let cam = parse_map_param(&bytes).expect("parses");
        assert!((cam.fx - 1484.032).abs() < 1e-2 && (cam.cx - 649.654).abs() < 1e-3);
        // The rectified left principal point (340.44, 485.67) maps back onto the raw image and
        // round-trips.
        let (u, v) = cam.unrectify(340.44, 485.67);
        let (ru, rv) = cam.rectify(u, v);
        assert!((ru - 340.44).abs() < 1e-3 && (rv - 485.67).abs() < 1e-3);
        // ~10° rectifying rotation: the raw optical centre lands ~255 px right of the rectified one.
        let (cu, _) = cam.rectify(cam.cx, cam.cy);
        assert!((cu - 340.44 - 257.0).abs() < 10.0, "{cu}");
        assert!(parse_map_param(&bytes[..100]).is_none());
    }

    // Raw camera model of the same device (from its camparam.yaml).
    const RAW_YAML: &str = r#"
cameraMatrixL: !!opencv-matrix
   data: [ 1.4851854240864188e+03, 0., 6.5040311256885661e+02, 0.,
       1.4851854240864188e+03, 4.7756338666589892e+02, 0., 0., 1. ]
distCoeffL: !!opencv-matrix
   data: [ 6.9401782453770391e-02, -1.6895497388500108e-01,
       5.3464555409248277e-04, -4.7178604968801869e-05,
       1.1223580877263613e-01 ]
cameraMatrixR: !!opencv-matrix
   data: [ 1.4839280924743068e+03, 0., 6.4139622503977716e+02, 0.,
       1.4839280924743068e+03, 4.9261761341204169e+02, 0., 0., 1. ]
distCoeffR: !!opencv-matrix
   data: [ 7.3250610489181883e-02, -1.8890928493546671e-01,
       5.1246701607524602e-04, 8.2623276916015263e-05,
       1.7252672946973535e-01 ]
steroRotation: !!opencv-matrix
   data: [ 9.4031199621009809e-01, -1.1826325881060748e-03,
       3.4031155014712955e-01, 1.4405042124902014e-03,
       9.9999883490891006e-01, -5.0510240187268888e-04,
       -3.4031055630261386e-01, 9.6517406934143327e-04,
       9.4031260424819441e-01 ]
steroTranslation: !!opencv-matrix
   data: [ -1.1679730866656732e+02, 1.0430025026597525e-01,
       2.0479904773800339e+01 ]
"#;

    #[test]
    fn rectification_matches_opencv() {
        let cal = StereoCalibration::parse(&format!("{YAML}{RAW_YAML}")).expect("parses");
        let raw = cal.raw.as_ref().expect("raw model");
        let (r1, r2) = bouguet_rectification(&raw.rotation, &raw.translation);
        // cv2.stereoRectify(K1, D1, K2, D2, (1280, 1024), R, T, alpha=-1), row-major.
        let r1_cv = [
            9.849551740e-01, -2.211137366e-03, 1.727958800e-01, 2.252383834e-03, 9.999974625e-01, -4.262460348e-05,
            -1.727953473e-01, 4.311859704e-04, 9.849576549e-01,
        ];
        let r2_cv = [
            9.849722146e-01, -8.795823265e-04, -1.727106334e-01, 9.208083070e-04, 9.999995635e-01, 1.585811728e-04,
            1.727104185e-01, -3.152314349e-04, 9.849725945e-01,
        ];
        for (m, cv) in [(r1, r1_cv), (r2, r2_cv)] {
            let rows = m.transpose().to_cols_array();
            for (a, b) in rows.iter().zip(cv) {
                assert!((a - b).abs() < 1e-7, "{rows:?} vs {cv:?}");
            }
        }
        // cv2.initUndistortRectifyMap at 640x512 with K and P scaled by 0.5.
        let maps = RectifyMaps::new(&cal, 640, 512).unwrap();
        for (map, (u, v), expect) in [
            (&maps.left, (100, 50), [119.06855, 39.10466]),
            (&maps.left, (320, 256), [343.76196, 251.95258]),
            (&maps.right, (100, 50), [93.66931, 65.687]),
            (&maps.right, (320, 256), [296.25302, 259.3657]),
        ] {
            let got = map[v * 640 + u];
            assert!((got[0] - expect[0]).abs() < 1e-3 && (got[1] - expect[1]).abs() < 1e-3, "{got:?} vs {expect:?}");
        }
    }

    #[test]
    fn rejects_incomplete_yaml() {
        assert!(StereoCalibration::parse("width: \"1280\"\nheight: \"1024\"\n").is_none());
    }
}
