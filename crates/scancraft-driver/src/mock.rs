use crate::traits::{ScannerDevice, ScannerDriver};
use crate::types::{
    DeviceInfo, DeviceParam, DeviceTransport, DiscoveredDevice, DriverError, ScanConfig,
    ScannerCapabilities,
};
use async_trait::async_trait;
use glam::{Mat3, Quat, Vec3};
use scancraft_core::{
    ColorCamera, ColorFrame, DepthMap, ImuSample, IrImages, PinholeIntrinsics, PreviewFrame,
    RgbImage, SensorFrame, Transform,
};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use tokio::sync::{broadcast, mpsc};
use tokio::time::{interval, Duration};

/// How far the simulated scanner orbits the desk scene per frame (0.7°: a full turn in ~45 s at
/// the simulator's ~11 fps). At 1.8° per frame the camera moved ~9 mm between frames and the
/// tracker kept reporting "Slow down", keeping only a few keyframes.
const ORBIT_STEP_RAD: f32 = 0.012;

/// Mock scanner driver generating synthetic multi-object scenes for development and CI testing.
pub struct MockDriver;

#[async_trait]
impl ScannerDriver for MockDriver {
    fn id(&self) -> &'static str {
        "mock"
    }

    fn name(&self) -> &'static str {
        "ScanCraft Simulated Virtual Scanner"
    }

    async fn discover(&self) -> Result<Vec<DiscoveredDevice>, DriverError> {
        let virt_transport = DeviceTransport::Virtual {
            name: "simulated_scanner_0".to_string(),
        };
        Ok(vec![DiscoveredDevice {
            driver_id: "mock".to_string(),
            model_name: "ScanCraft Virtual Pro 3D".to_string(),
            serial_number: "SIM-SCAN-001".to_string(),
            available_transports: vec![virt_transport.clone()],
            selected_transport: virt_transport,
        }])
    }

    async fn connect(&self, transport: &DeviceTransport) -> Result<Box<dyn ScannerDevice>, DriverError> {
        Ok(Box::new(MockDevice::new(transport.clone())))
    }
}

pub struct MockDevice {
    info: DeviceInfo,
    capabilities: ScannerCapabilities,
    streaming: Arc<AtomicBool>,
    paused: Arc<AtomicBool>,
    imu_tx: broadcast::Sender<ImuSample>,
    /// Where the next scan is recorded (`set_recording`), and what the running one records to.
    recording: Option<crate::recording::RecordingTarget>,
    recording_files: Vec<std::path::PathBuf>,
    motion: MockMotion,
}

/// One rendered synthetic multi-object view from [`MockDevice::render_synthetic_frame`].
#[derive(Debug, Clone)]
pub struct SyntheticFrame {
    pub depth: DepthMap,
    pub ir: RgbImage,
    pub color: ColorFrame,
    pub sensor_pose: Transform,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum SceneMaterial {
    Turntable,
    Vase,
    Canister,
    Block,
    /// The static table the turntable stands on (turntable scene only).
    Table,
}

#[derive(Clone, Copy, Debug)]
struct RayHit {
    t: f32,
    /// Hit point in the frame of what was hit (the turntable's for everything turning with it):
    /// texture and marker stickers are looked up here.
    pos: Vec3,
    /// Surface normal in the world (lighting is fixed in the world).
    normal: Vec3,
    material: SceneMaterial,
}

struct CameraRig {
    width: u32,
    height: u32,
    intrinsics: PinholeIntrinsics,
    sensor_pose: Transform,
    rot_mat: Mat3,
    color_camera: Arc<ColorCamera>,
    color_origin: Vec3,
}

/// Baseline of the simulated rectified IR pair, mm, and the right camera's principal point offset
/// (`cx_right - cx_left`), px.
pub const MOCK_BASELINE_MM: f32 = 50.0;
pub const MOCK_RIGHT_CX_OFFSET: f32 = 2.0;

/// Camera elevation above the turntable plane in the orbiting (handheld) and turntable scenes.
const ORBIT_ELEVATION_DEG: f32 = 22.0;
const TURNTABLE_ELEVATION_DEG: f32 = 30.0;
/// The turntable scene is seen from 330 mm, so all of it lies inside the POP 4 Full-field depth
/// window (250-500 mm, which the app applies for this mode): a near limit cutting through the
/// turning objects is an edge fixed to the sensor and pulls tracking towards "no motion" (3.7 %
/// too little turn measured at 280 mm, 2026-11-28).
const TURNTABLE_DISTANCE_MM: f32 = 330.0;

impl CameraRig {
    fn new(width: u32, height: u32, yaw_rad: f32) -> Self {
        Self::looking(width, height, yaw_rad, ORBIT_ELEVATION_DEG, 280.0)
    }

    fn looking(width: u32, height: u32, yaw_rad: f32, elevation_deg: f32, dist: f32) -> Self {
        // Focus at mid-height of the turntable scene (~44 mm above z = 0).
        let target = Vec3::new(0.0, 0.0, 44.0);
        let elev = elevation_deg.to_radians();
        let cos_e = elev.cos();
        let sin_e = elev.sin();
        let (sin_y, cos_y) = yaw_rad.sin_cos();

        let cam_pos = target + Vec3::new(dist * cos_e * sin_y, -dist * cos_e * cos_y, dist * sin_e);
        let fwd = (target - cam_pos).normalize();
        let right = fwd.cross(Vec3::Z).normalize();
        let up = right.cross(fwd).normalize();

        // Scanner axes: +X right, +Y forward, +Z up.
        let rot_mat = Mat3::from_cols(right, fwd, up);
        let sensor_pose = Transform::from_rotation_translation(Quat::from_mat3(&rot_mat), cam_pos);

        // ~0.42 mm per pixel at 275 mm for 640 px width, scaled cleanly to other widths.
        let focal = (width as f32) * (275.0 / 0.42) / 640.0;
        let intrinsics = PinholeIntrinsics {
            fx: focal,
            fy: focal,
            cx: width as f32 * 0.5,
            cy: height as f32 * 0.5,
        };
        let color_camera = Arc::new(ColorCamera {
            width,
            height,
            intrinsics,
            distortion: [0.0; 5],
            rotation: Mat3::IDENTITY,
            translation: Vec3::new(-12.0, 0.0, 0.0),
        });
        let color_origin = sensor_pose.transform_point3(color_camera.color_to_sensor(Vec3::ZERO));

        Self {
            width,
            height,
            intrinsics,
            sensor_pose,
            rot_mat,
            color_camera,
            color_origin,
        }
    }

    #[inline]
    fn ray_dir(&self, x: u32, y: u32) -> Vec3 {
        self.ray_dir_f(x as f32, y as f32, 0.0)
    }

    /// Ray through pixel (x, y) of a camera whose principal point is `cx_offset` px right of the
    /// depth camera's (the right IR camera).
    #[inline]
    fn ray_dir_f(&self, x: f32, y: f32, cx_offset: f32) -> Vec3 {
        let dx = (x - self.intrinsics.cx - cx_offset) / self.intrinsics.fx;
        let dz = -(y - self.intrinsics.cy) / self.intrinsics.fy;
        // Local ray (dx, 1, dz) guarantees t == depth along the optical axis (+Y).
        self.rot_mat * Vec3::new(dx, 1.0, dz)
    }

    /// The rectified IR pair: the left camera is the depth camera, the right one sits
    /// [`MOCK_BASELINE_MM`] along the sensor's +X.
    fn stereo_rig(&self) -> scancraft_core::StereoRig {
        scancraft_core::StereoRig {
            width: self.width,
            height: self.height,
            intrinsics: self.intrinsics,
            baseline_mm: MOCK_BASELINE_MM,
            disparity_offset_px: MOCK_RIGHT_CX_OFFSET,
            calibration_set: "mock".into(),
            rectification: None,
        }
    }
}

/// What the simulated scene looks like at one instant: how far the turntable (and everything on
/// it) has turned, and whether the static table around it is there.
#[derive(Clone, Copy, Debug, Default)]
struct SceneState {
    /// Turntable angle about the world z axis, radians.
    angle: f32,
    static_table: bool,
}

impl SceneState {
    /// A world vector in the turntable's frame.
    #[inline]
    fn to_turntable(&self, v: Vec3) -> Vec3 {
        if self.angle == 0.0 {
            return v;
        }
        let (s, c) = (-self.angle).sin_cos();
        Vec3::new(c * v.x - s * v.y, s * v.x + c * v.y, v.z)
    }

    #[inline]
    fn to_world(&self, v: Vec3) -> Vec3 {
        if self.angle == 0.0 {
            return v;
        }
        let (s, c) = self.angle.sin_cos();
        Vec3::new(c * v.x - s * v.y, s * v.x + c * v.y, v.z)
    }
}

/// Marker stickers on the turntable top (z = 0, turntable frame): white 6 mm dots in a 1.5 mm
/// black ring, in an irregular pattern on the free rim of the disc, so they turn with the objects.
pub const MOCK_MARKERS: [[f32; 2]; 16] = [
    [97.0, 0.0],
    [91.0, 28.0],
    [88.0, 52.0],
    [72.0, 71.0],
    [40.0, 92.0],
    [6.0, 98.0],
    [-31.0, 90.0],
    [-58.0, 80.0],
    [-95.0, 26.0],
    [-100.0, -20.0],
    [-78.0, -60.0],
    [-30.0, -80.0],
    [-2.0, -97.0],
    [33.0, -91.0],
    [64.0, -70.0],
    [96.0, -36.0],
];
const MARKER_DOT_MM: f32 = 3.0;
const MARKER_RING_MM: f32 = 4.5;

/// 0 outside every sticker, 1 on a white dot, -1 on its black ring (`p`: turntable frame).
fn marker_at(p: Vec3) -> i8 {
    if p.z < -0.5 {
        return 0;
    }
    for m in MOCK_MARKERS {
        let d2 = (p.x - m[0]).powi(2) + (p.y - m[1]).powi(2);
        if d2 <= MARKER_DOT_MM * MARKER_DOT_MM {
            return 1;
        }
        if d2 <= MARKER_RING_MM * MARKER_RING_MM {
            return -1;
        }
    }
    0
}

#[inline]
fn vase_radius(z: f32, angle: f32) -> f32 {
    let dz = z - 55.0;
    let contour = 32.0 + 9.0 * (dz * 0.055).cos() - (dz * 0.015).powi(2) * 8.0;
    let flutes = 2.5 * (6.0 * angle).sin();
    (contour + flutes).max(14.0)
}

#[inline]
fn vase_sdf_eval(p: Vec3) -> f32 {
    let angle = p.y.atan2(p.x);
    let r = (p.x * p.x + p.y * p.y).sqrt();
    r - vase_radius(p.z, angle)
}

fn vase_normal(p: Vec3) -> Vec3 {
    let eps = 0.25f32;
    let gx = vase_sdf_eval(p + Vec3::X * eps) - vase_sdf_eval(p - Vec3::X * eps);
    let gy = vase_sdf_eval(p + Vec3::Y * eps) - vase_sdf_eval(p - Vec3::Y * eps);
    let gz = vase_sdf_eval(p + Vec3::Z * eps) - vase_sdf_eval(p - Vec3::Z * eps);
    Vec3::new(gx, gy, gz).normalize_or_zero()
}

fn intersect_cylinder_finite(
    orig: Vec3,
    dir: Vec3,
    cx: f32,
    cy: f32,
    radius: f32,
    z_min: f32,
    z_max: f32,
    material: SceneMaterial,
) -> Option<RayHit> {
    let ox = orig.x - cx;
    let oy = orig.y - cy;
    let a = dir.x * dir.x + dir.y * dir.y;
    let mut best: Option<RayHit> = None;

    if a > 1e-8 {
        let b = 2.0 * (ox * dir.x + oy * dir.y);
        let c = ox * ox + oy * oy - radius * radius;
        let disc = b * b - 4.0 * a * c;
        if disc >= 0.0 {
            let sqrt_d = disc.sqrt();
            let inv = 0.5 / a;
            for t in [(-b - sqrt_d) * inv, (-b + sqrt_d) * inv] {
                if t > 1e-3 && best.is_none_or(|b_hit| t < b_hit.t) {
                    let pos = orig + dir * t;
                    if pos.z >= z_min && pos.z <= z_max {
                        let normal = Vec3::new((pos.x - cx) / radius, (pos.y - cy) / radius, 0.0);
                        best = Some(RayHit { t, pos, normal, material });
                    }
                }
            }
        }
    }

    // Top cap at z_max (and bottom cap if seen from underneath).
    if dir.z.abs() > 1e-6 {
        let r2 = radius * radius;
        for (cap_z, nz) in [(z_max, 1.0f32), (z_min, -1.0f32)] {
            let t = (cap_z - orig.z) / dir.z;
            if t > 1e-3 && best.is_none_or(|b_hit| t < b_hit.t) {
                let pos = orig + dir * t;
                let dx = pos.x - cx;
                let dy = pos.y - cy;
                if dx * dx + dy * dy <= r2 {
                    best = Some(RayHit { t, pos, normal: Vec3::new(0.0, 0.0, nz), material });
                }
            }
        }
    }

    best
}

fn intersect_vase(orig: Vec3, dir: Vec3) -> Option<RayHit> {
    const MAX_R: f32 = 46.0;
    const Z_MAX: f32 = 115.0;

    let mut t_enter = 0.0f32;
    let mut t_exit = f32::INFINITY;

    // Intersect with bounding cylinder (x^2 + y^2 <= MAX_R^2).
    let a = dir.x * dir.x + dir.y * dir.y;
    if a > 1e-8 {
        let b = 2.0 * (orig.x * dir.x + orig.y * dir.y);
        let c = orig.x * orig.x + orig.y * orig.y - MAX_R * MAX_R;
        let disc = b * b - 4.0 * a * c;
        if disc < 0.0 {
            return None;
        }
        let sqrt_d = disc.sqrt();
        let inv = 0.5 / a;
        t_enter = t_enter.max((-b - sqrt_d) * inv);
        t_exit = t_exit.min((-b + sqrt_d) * inv);
    } else if orig.x * orig.x + orig.y * orig.y > MAX_R * MAX_R {
        return None;
    }

    // Intersect with z slab [0, Z_MAX].
    if dir.z.abs() > 1e-6 {
        let tz0 = (0.0 - orig.z) / dir.z;
        let tz1 = (Z_MAX - orig.z) / dir.z;
        t_enter = t_enter.max(tz0.min(tz1));
        t_exit = t_exit.min(tz0.max(tz1));
    } else if orig.z < 0.0 || orig.z > Z_MAX {
        return None;
    }

    if t_exit <= t_enter.max(1e-3) {
        return None;
    }
    t_enter = t_enter.max(1e-3);

    // Check top cap first if the ray enters at z = Z_MAX.
    if dir.z < -1e-6 {
        let t_top = (Z_MAX - orig.z) / dir.z;
        if t_top >= t_enter - 1e-3 && t_top <= t_exit + 1e-3 {
            let p_top = orig + dir * t_top;
            if vase_sdf_eval(p_top) <= 0.0 {
                return Some(RayHit {
                    t: t_top,
                    pos: p_top,
                    normal: Vec3::Z,
                    material: SceneMaterial::Vase,
                });
            }
        }
    }

    const STEPS: usize = 16;
    let dt = (t_exit - t_enter) / STEPS as f32;
    let mut prev_t = t_enter;
    let mut prev_f = vase_sdf_eval(orig + dir * prev_t);
    if prev_f <= 0.0 {
        let pos = orig + dir * prev_t;
        return Some(RayHit { t: prev_t, pos, normal: vase_normal(pos), material: SceneMaterial::Vase });
    }

    for step in 1..=STEPS {
        let curr_t = t_enter + dt * step as f32;
        let curr_f = vase_sdf_eval(orig + dir * curr_t);
        if curr_f <= 0.0 {
            let (mut lo, mut hi) = (prev_t, curr_t);
            for _ in 0..5 {
                let mid = 0.5 * (lo + hi);
                if vase_sdf_eval(orig + dir * mid) <= 0.0 {
                    hi = mid;
                } else {
                    lo = mid;
                }
            }
            let t = hi;
            let pos = orig + dir * t;
            return Some(RayHit {
                t,
                pos,
                normal: vase_normal(pos),
                material: SceneMaterial::Vase,
            });
        }
        prev_t = curr_t;
        prev_f = curr_f;
    }
    let _ = prev_f;
    None
}

fn intersect_block(orig: Vec3, dir: Vec3) -> Option<RayHit> {
    // Rotated box sitting on z = 0: center (-62, -14, 24), yaw 22 deg, half-extents (18, 15, 24).
    let center = Vec3::new(-62.0, -14.0, 24.0);
    let half = Vec3::new(18.0, 15.0, 24.0);
    let (s, c) = (-22.0f32.to_radians()).sin_cos();

    let rel = orig - center;
    let lo = Vec3::new(c * rel.x - s * rel.y, s * rel.x + c * rel.y, rel.z);
    let ld = Vec3::new(c * dir.x - s * dir.y, s * dir.x + c * dir.y, dir.z);

    let mut t_min = 1e-3f32;
    let mut t_max = f32::INFINITY;
    let mut hit_normal_local = Vec3::ZERO;

    let axes = [
        (lo.x, ld.x, half.x, Vec3::X),
        (lo.y, ld.y, half.y, Vec3::Y),
        (lo.z, ld.z, half.z, Vec3::Z),
    ];

    for (o, d, h, axis) in axes {
        if d.abs() < 1e-7 {
            if o.abs() > h {
                return None;
            }
        } else {
            let inv = 1.0 / d;
            let mut t1 = (-h - o) * inv;
            let mut t2 = (h - o) * inv;
            let mut n = -axis;
            if t1 > t2 {
                std::mem::swap(&mut t1, &mut t2);
                n = axis;
            }
            if t1 > t_min {
                t_min = t1;
                hit_normal_local = n;
            }
            t_max = t_max.min(t2);
            if t_min > t_max {
                return None;
            }
        }
    }

    if hit_normal_local == Vec3::ZERO {
        return None;
    }
    let (sw, cw) = (22.0f32.to_radians()).sin_cos();
    let normal = Vec3::new(
        cw * hit_normal_local.x - sw * hit_normal_local.y,
        sw * hit_normal_local.x + cw * hit_normal_local.y,
        hit_normal_local.z,
    );
    Some(RayHit {
        t: t_min,
        pos: orig + dir * t_min,
        normal,
        material: SceneMaterial::Block,
    })
}

/// Nearest hit of a world ray. The turntable and the objects on it are traced in the turntable's
/// frame (turned by `scene.angle`); the static table in the world.
fn trace_scene_at(orig: Vec3, dir: Vec3, scene: &SceneState) -> Option<RayHit> {
    let (o, d) = (scene.to_turntable(orig), scene.to_turntable(dir));
    let mut best: Option<RayHit> = None;
    let candidates = [
        intersect_cylinder_finite(o, d, 0.0, 0.0, 105.0, -8.0, 0.0, SceneMaterial::Turntable),
        intersect_vase(o, d),
        intersect_cylinder_finite(o, d, 66.0, 14.0, 21.0, 0.0, 76.0, SceneMaterial::Canister),
        intersect_block(o, d),
    ];
    for hit in candidates.into_iter().flatten() {
        if best.is_none_or(|b| hit.t < b.t) {
            best = Some(hit);
        }
    }
    // (The ray parameter is the same in both frames: the rotation keeps lengths.)
    let mut best = best.map(|h| RayHit { normal: scene.to_world(h.normal), ..h });
    if scene.static_table && dir.z < -1e-6 {
        let t = (-8.0 - orig.z) / dir.z;
        let pos = orig + dir * t;
        if t > 1e-3 && pos.x.abs() < 400.0 && pos.y.abs() < 400.0 && best.is_none_or(|b| t < b.t) {
            best = Some(RayHit { t, pos, normal: Vec3::Z, material: SceneMaterial::Table });
        }
    }
    // A static wall behind the turntable (in range of the depth window above the objects).
    if scene.static_table && dir.y > 1e-6 {
        let t = (STATIC_WALL_Y - orig.y) / dir.y;
        let pos = orig + dir * t;
        if t > 1e-3 && pos.x.abs() < 400.0 && (-8.0..400.0).contains(&pos.z) && best.is_none_or(|b| t < b.t) {
            best = Some(RayHit { t, pos, normal: -Vec3::Y, material: SceneMaterial::Table });
        }
    }
    best
}

/// The static wall of the turntable scene: the plane y = 160 mm (world) behind the turntable.
const STATIC_WALL_Y: f32 = 160.0;

fn shade_rgb(hit: Option<RayHit>, y: u32, height: u32) -> [u8; 3] {
    let Some(h) = hit else {
        // Cool grey-blue studio backdrop gradient (B > R).
        let t = y as f32 / height.max(1) as f32;
        return [
            (28.0 + t * 10.0) as u8,
            (35.0 + t * 19.0) as u8,
            (49.0 + t * 25.0) as u8,
        ];
    };

    let n = h.normal.normalize_or_zero();
    let key_dir = Vec3::new(0.42, -0.68, 0.60).normalize();
    let fill_dir = Vec3::new(-0.55, -0.35, 0.40).normalize();
    let key = n.dot(key_dir).max(0.0);
    let fill = n.dot(fill_dir).max(0.0);
    let mut light = 0.28 + 0.62 * key + 0.18 * fill;

    let base = match h.material {
        SceneMaterial::Turntable => {
            let r = (h.pos.x * h.pos.x + h.pos.y * h.pos.y).sqrt();
            let ring = if (r % 24.0) < 1.2 { 0.86 } else { 1.0 };
            // Soft ambient occlusion halo around objects resting on the turntable.
            let d_vase = (r - 38.0).clamp(0.0, 14.0) / 14.0;
            let d_can = ((Vec3::new(h.pos.x - 66.0, h.pos.y - 14.0, 0.0).length()) - 21.0).clamp(0.0, 10.0) / 10.0;
            let d_blk = ((Vec3::new(h.pos.x + 62.0, h.pos.y + 14.0, 0.0).length()) - 18.0).clamp(0.0, 10.0) / 10.0;
            let ao = 0.55 + 0.45 * d_vase.min(d_can).min(d_blk);
            light *= ao * ring;
            if h.pos.z < -0.5 {
                Vec3::new(110.0, 115.0, 124.0)
            } else {
                Vec3::new(168.0, 173.0, 180.0)
            }
        }
        SceneMaterial::Vase => {
            let glaze = (h.pos.z * 0.14).sin() * 0.5 + 0.5;
            Vec3::new(196.0 + 28.0 * glaze, 92.0 + 24.0 * (1.0 - glaze), 60.0 + 16.0 * glaze)
        }
        SceneMaterial::Canister => {
            if n.z > 0.8 {
                Vec3::new(82.0, 144.0, 228.0)
            } else {
                Vec3::new(46.0, 106.0, 214.0)
            }
        }
        SceneMaterial::Block => Vec3::new(68.0, 174.0, 98.0),
        SceneMaterial::Table => {
            let grain = ((h.pos.x * 0.07).sin() * (h.pos.y * 0.05).cos()) * 0.06 + 1.0;
            Vec3::new(120.0, 96.0, 74.0) * grain
        }
    };
    let base = match (h.material, marker_at(h.pos)) {
        (SceneMaterial::Turntable, 1) => Vec3::splat(235.0),
        (SceneMaterial::Turntable, -1) => Vec3::splat(14.0),
        _ => base,
    };

    let c = base * light;
    [
        c.x.clamp(0.0, 255.0) as u8,
        c.y.clamp(0.0, 255.0) as u8,
        c.z.clamp(0.0, 255.0) as u8,
    ]
}

fn shade_ir(hit: Option<RayHit>) -> u8 {
    let Some(h) = hit else {
        return 16;
    };
    let n = h.normal.normalize_or_zero();
    let key = n.dot(Vec3::new(0.30, -0.78, 0.55).normalize()).max(0.0);
    let albedo = match h.material {
        SceneMaterial::Turntable => match marker_at(h.pos) {
            // Retro-reflective dot (bright whatever the light) in a black ring.
            1 => return 250,
            -1 => return 12,
            _ => 145.0,
        },
        SceneMaterial::Vase => 195.0,
        SceneMaterial::Canister => 165.0,
        SceneMaterial::Block => 175.0,
        SceneMaterial::Table => 120.0,
    };
    (albedo * (0.30 + 0.70 * key)).clamp(24.0, 255.0) as u8
}

/// IR grey level of pixel (x, y), with marker stickers anti-aliased (3 x 3 samples) so their
/// centres are found with sub-pixel accuracy.
fn ir_pixel(rig: &CameraRig, origin: Vec3, x: u32, y: u32, cx_offset: f32, scene: &SceneState, hit: Option<RayHit>) -> u8 {
    let near_marker = hit.is_some_and(|h| {
        h.material == SceneMaterial::Turntable
            && h.pos.z > -0.5
            && MOCK_MARKERS.iter().any(|m| (h.pos.x - m[0]).powi(2) + (h.pos.y - m[1]).powi(2) < (MARKER_RING_MM + 2.0).powi(2))
    });
    if !near_marker {
        return shade_ir(hit);
    }
    let mut sum = 0u32;
    for sy in [-1.0f32 / 3.0, 0.0, 1.0 / 3.0] {
        for sx in [-1.0f32 / 3.0, 0.0, 1.0 / 3.0] {
            let dir = rig.ray_dir_f(x as f32 + sx, y as f32 + sy, cx_offset);
            sum += shade_ir(trace_scene_at(origin, dir, scene)) as u32;
        }
    }
    (sum / 9) as u8
}

/// How the simulated scene moves from one frame to the next.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MockMotion {
    /// Handheld: the camera orbits the static desk scene by this much per frame, radians.
    pub orbit_step_rad: f32,
    /// Turntable: the camera is fixed and the turntable (with everything on it) turns by this much
    /// per frame, radians.
    pub turntable_step_rad: f32,
    /// Time between frames, ms.
    pub period_ms: u64,
}

impl Default for MockMotion {
    fn default() -> Self {
        // A manual turntable turned by hand: ~18 degrees a second (a turn in 20 s) at 15 fps.
        Self { orbit_step_rad: ORBIT_STEP_RAD, turntable_step_rad: 1.2f32.to_radians(), period_ms: 66 }
    }
}

/// Identifies the simulator's recorded streams (`capture.json` `stream.format`).
pub const MOCK_STREAM_FORMAT: &str = "scancraft-mock-v1";
/// Name of the simulator's calibration file in a pass's `calibration/` folder.
pub const MOCK_CALIBRATION_FILE: &str = "mock.json";
const MAIN_MAGIC: &[u8; 4] = b"SCMK";
const RGB_MAGIC: &[u8; 4] = b"SCMC";
const RECORD_VERSION: u8 = 1;

/// What the simulator's recorded streams need to be decoded: the depth camera, the IR pair and
/// the colour camera (as a real scanner's downloaded calibration files would give them).
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct MockCalibration {
    pub depth_intrinsics: PinholeIntrinsics,
    pub stereo: scancraft_core::StereoRig,
    pub color: ColorCamera,
}

impl MockCalibration {
    fn of(rig: &CameraRig) -> Self {
        Self { depth_intrinsics: rig.intrinsics, stereo: rig.stereo_rig(), color: (*rig.color_camera).clone() }
    }
}

impl MockDevice {
    pub fn new(transport: DeviceTransport) -> Self {
        let (imu_tx, _) = broadcast::channel(128);
        Self {
            info: DeviceInfo {
                imu_to_sensor: None,
                vendor: "ScanCraft Simulation".to_string(),
                model: "Virtual Pro 3D".to_string(),
                serial_number: "SIM-SCAN-001".to_string(),
                firmware_version: "v1.0.0-mock".to_string(),
                active_transport: transport,
                depth_resolution: (640, 512),
                rgb_resolution: Some((640, 512)),
                recommended_distance_range_mm: (150.0, 400.0),
            },
            capabilities: ScannerCapabilities {
                has_rgb_camera: true,
                has_imu: true,
                has_hardware_turntable_control: true,
                supported_laser_modes: vec!["Cross Blue".to_string(), "Parallel Infrared".to_string()],
                supports_single_shot: true,
                supports_continuous_scan: true,
                supports_marker_tracking: true,
                supports_feature_tracking: true,
                supported_transports: vec!["Virtual".to_string()],
                manual_exposure: true,
                modes: crate::modes::ModeSupport {
                    light_sources: vec![(crate::modes::LightSource::FullFieldHd, crate::modes::Availability::Available)],
                    // Marker tracking: the stickers on the turntable, seen by a rectified IR pair.
                    tracking_modes: vec![
                        (crate::modes::TrackingMode::Feature, crate::modes::Availability::Available),
                        (crate::modes::TrackingMode::Marker, crate::modes::Availability::Available),
                    ],
                    object_types: vec![(crate::modes::ObjectType::General, crate::modes::Availability::Available)],
                    combinations: Vec::new(),
                },
            },
            streaming: Arc::new(AtomicBool::new(false)),
            paused: Arc::new(AtomicBool::new(false)),
            imu_tx,
            recording: None,
            recording_files: Vec::new(),
            motion: MockMotion::default(),
        }
    }

    /// The simulator with another motion (tests: faster turntables, other frame rates).
    pub fn with_motion(mut self, motion: MockMotion) -> Self {
        self.motion = motion;
        self
    }

    fn render_depth_ir(rig: &CameraRig) -> (DepthMap, RgbImage) {
        let (depth, ir, _) = Self::render_depth_ir_at(rig, &SceneState::default(), false);
        (depth, ir)
    }

    /// Depth and left IR image of `scene` from `rig`, and with `stereo` the right IR image too.
    fn render_depth_ir_at(rig: &CameraRig, scene: &SceneState, stereo: bool) -> (DepthMap, RgbImage, Option<RgbImage>) {
        let (w, h) = (rig.width, rig.height);
        let len = (w * h) as usize;
        let mut depth_data = vec![0u16; len];
        let mut ir_data = vec![16u8; len];
        let orig = rig.sensor_pose.translation;

        for y in 0..h {
            for x in 0..w {
                let dir = rig.ray_dir(x, y);
                let hit = trace_scene_at(orig, dir, scene);
                let idx = (y * w + x) as usize;
                if let Some(hit_info) = hit {
                    depth_data[idx] = (hit_info.t / 0.1).round().clamp(1000.0, 60000.0) as u16;
                }
                ir_data[idx] = ir_pixel(rig, orig, x, y, 0.0, scene, hit);
            }
        }
        let right = stereo.then(|| {
            let origin = orig + rig.rot_mat * Vec3::new(MOCK_BASELINE_MM, 0.0, 0.0);
            let mut data = vec![16u8; len];
            for y in 0..h {
                for x in 0..w {
                    let hit = trace_scene_at(origin, rig.ray_dir_f(x as f32, y as f32, MOCK_RIGHT_CX_OFFSET), scene);
                    data[(y * w + x) as usize] = ir_pixel(rig, origin, x, y, MOCK_RIGHT_CX_OFFSET, scene, hit);
                }
            }
            RgbImage { width: w, height: h, channels: 1, data }
        });

        let depth = DepthMap::new(w, h, 0.1, depth_data).with_intrinsics(rig.intrinsics);
        let ir = RgbImage { width: w, height: h, channels: 1, data: ir_data };
        (depth, ir, right)
    }

    fn render_color_from_rig(rig: &CameraRig, timestamp_ms: u64) -> ColorFrame {
        Self::render_color_at(rig, &SceneState::default(), timestamp_ms)
    }

    fn render_color_at(rig: &CameraRig, scene: &SceneState, timestamp_ms: u64) -> ColorFrame {
        let (w, h) = (rig.width, rig.height);
        let mut rgb_data = vec![0u8; (w * h * 3) as usize];
        let orig = rig.color_origin;

        for y in 0..h {
            for x in 0..w {
                let dir = rig.ray_dir(x, y);
                let hit = trace_scene_at(orig, dir, scene);
                let px = shade_rgb(hit, y, h);
                let off = ((y * w + x) * 3) as usize;
                rgb_data[off] = px[0];
                rgb_data[off + 1] = px[1];
                rgb_data[off + 2] = px[2];
            }
        }

        ColorFrame {
            image: RgbImage { width: w, height: h, channels: 3, data: rgb_data },
            jpeg: None,
            timestamp_ms,
            camera: Some(rig.color_camera.clone()),
        }
    }

    /// Renders a complete synchronized multi-object turntable view (depth, IR, calibrated RGB, and
    /// sensor pose) for simulator capture and offline AI test project generation.
    pub fn render_synthetic_frame(width: u32, height: u32, yaw_rad: f32, timestamp_ms: u64) -> SyntheticFrame {
        let rig = CameraRig::new(width, height, yaw_rad);
        let (depth, ir) = Self::render_depth_ir(&rig);
        let color = Self::render_color_from_rig(&rig, timestamp_ms);
        SyntheticFrame {
            depth,
            ir,
            color,
            sensor_pose: rig.sensor_pose,
        }
    }

    /// Generate a synthetic depth map and IR camera image of the multi-object turntable scene.
    pub fn generate_synthetic_scan(width: u32, height: u32, yaw_rad: f32) -> (DepthMap, RgbImage) {
        let rig = CameraRig::new(width, height, yaw_rad);
        Self::render_depth_ir(&rig)
    }

    /// Generates a calibrated synthetic RGB frame of the multi-object scene.
    pub fn generate_synthetic_color(depth: &DepthMap, yaw_rad: f32, timestamp_ms: u64) -> ColorFrame {
        let rig = CameraRig::new(depth.width, depth.height, yaw_rad);
        Self::render_color_from_rig(&rig, timestamp_ms)
    }

    /// The camera and scene of scan frame `frame_index` (from 1) under `config`: handheld passes
    /// orbit the static desk scene; turntable passes (`ScanConfig::turntable_mode`) look at the
    /// turntable from a fixed place, with the turntable turning and a static table around it.
    fn scan_view(config: &ScanConfig, motion: &MockMotion, frame_index: u64) -> (CameraRig, SceneState, bool) {
        let step = frame_index.saturating_sub(1) as f32;
        if config.turntable_mode == crate::types::TurntableMode::HandheldFreeform {
            (CameraRig::new(640, 512, step * motion.orbit_step_rad), SceneState::default(), false)
        } else {
            let scene = SceneState { angle: step * motion.turntable_step_rad, static_table: true };
            (CameraRig::looking(640, 512, 0.0, TURNTABLE_ELEVATION_DEG, TURNTABLE_DISTANCE_MM), scene, true)
        }
    }

    /// Ground truth of scan frame `frame_index`: the sensor pose in the frame of what is scanned
    /// (the desk scene; for a turntable pass the turntable, which turns under the fixed sensor).
    pub fn scan_truth(config: &ScanConfig, motion: &MockMotion, frame_index: u64) -> Transform {
        let (rig, scene, _) = Self::scan_view(config, motion, frame_index);
        // turntable <- world (a turn by -angle about z), then world <- sensor.
        let q = Quat::from_rotation_z(-scene.angle);
        Transform::from_rotation_translation((q * rig.sensor_pose.rotation).normalize(), q * rig.sensor_pose.translation)
    }

    /// Everything the simulator sends for scan frame `frame_index` (from 1) under `config`.
    pub fn scan_frame(config: &ScanConfig, motion: &MockMotion, frame_index: u64) -> SensorFrame {
        let (rig, scene, turntable) = Self::scan_view(config, motion, frame_index);
        let ts = frame_index * motion.period_ms;
        let stereo = matches!(config.tracking, crate::modes::TrackingMode::Marker | crate::modes::TrackingMode::GlobalMarker);
        let emit_color = config.color_texture || config.tracking == crate::modes::TrackingMode::Color;
        let (depth, ir, right) = Self::render_depth_ir_at(&rig, &scene, stereo);
        let color = emit_color.then(|| Self::render_color_at(&rig, &scene, ts));
        // The IMU measures the scanner: it turns with an orbit and stands still on a turntable.
        let (rate, yaw) = if turntable { (0.0, 0.0) } else { (motion.orbit_step_rad * 1000.0 / motion.period_ms as f32, step_angle(frame_index, motion.orbit_step_rad)) };
        let imu_sample = ImuSample {
            timestamp_ms: ts,
            orientation: rig.sensor_pose.rotation,
            angular_velocity: Vec3::new(0.0, 0.0, rate),
            linear_acceleration: Vec3::new(0.0, 0.0, 1.0),
            euler_degrees: Vec3::new(0.0, 0.0, yaw.to_degrees()),
        };
        let mut frame = SensorFrame::new(frame_index, ts);
        frame.depth = Some(depth);
        frame.color = color;
        frame.ir = Some(IrImages { left: ir, right, rectified: true, rig: Some(Arc::new(rig.stereo_rig())) });
        frame.imu = Some(imu_sample);
        frame
    }

    /// The calibration a recording of this simulator needs ([`decode_recording`]).
    pub fn calibration() -> MockCalibration {
        MockCalibration::of(&CameraRig::new(640, 512, 0.0))
    }
}

fn step_angle(frame_index: u64, step: f32) -> f32 {
    frame_index.saturating_sub(1) as f32 * step
}

fn put_u32(out: &mut Vec<u8>, v: u32) {
    out.extend_from_slice(&v.to_le_bytes());
}

fn put_u64(out: &mut Vec<u8>, v: u64) {
    out.extend_from_slice(&v.to_le_bytes());
}

fn put_f32s(out: &mut Vec<u8>, v: &[f32]) {
    for x in v {
        out.extend_from_slice(&x.to_le_bytes());
    }
}

/// One recorded main-stream record (depth, IR pair, IMU) of the simulator's wire format.
pub fn encode_main_record(frame: &SensorFrame) -> Vec<u8> {
    let depth = frame.depth.as_ref();
    let ir = frame.ir.as_ref();
    let (w, h) = depth.map_or((0, 0), |d| (d.width, d.height));
    let mut payload = Vec::with_capacity((w * h * 4) as usize + 128);
    payload.push(RECORD_VERSION);
    put_u64(&mut payload, frame.frame_index);
    put_u64(&mut payload, frame.timestamp_ms);
    put_u32(&mut payload, w);
    put_u32(&mut payload, h);
    put_f32s(&mut payload, &[depth.map_or(0.1, |d| d.scale_mm)]);
    let right = ir.and_then(|i| i.right.as_ref()).filter(|r| r.width == w && r.height == h && r.channels == 1);
    let left = ir.map(|i| &i.left).filter(|l| l.width == w && l.height == h && l.channels == 1);
    let flags = left.is_some() as u8 | (right.is_some() as u8) << 1 | (frame.imu.is_some() as u8) << 2;
    payload.push(flags);
    if let Some(d) = depth {
        for v in &d.data {
            payload.extend_from_slice(&v.to_le_bytes());
        }
    }
    for img in [left, right].into_iter().flatten() {
        payload.extend_from_slice(&img.data);
    }
    if let Some(imu) = &frame.imu {
        put_u64(&mut payload, imu.timestamp_ms);
        put_f32s(&mut payload, &imu.orientation.to_array());
        put_f32s(&mut payload, &imu.angular_velocity.to_array());
        put_f32s(&mut payload, &imu.linear_acceleration.to_array());
        put_f32s(&mut payload, &imu.euler_degrees.to_array());
    }
    frame_record(MAIN_MAGIC, payload)
}

/// One recorded colour-stream record of the simulator's wire format.
pub fn encode_rgb_record(color: &ColorFrame) -> Vec<u8> {
    let img = &color.image;
    let mut payload = Vec::with_capacity(img.data.len() + 32);
    payload.push(RECORD_VERSION);
    put_u64(&mut payload, color.timestamp_ms);
    put_u32(&mut payload, img.width);
    put_u32(&mut payload, img.height);
    payload.push(img.channels);
    payload.extend_from_slice(&img.data);
    frame_record(RGB_MAGIC, payload)
}

fn frame_record(magic: &[u8; 4], payload: Vec<u8>) -> Vec<u8> {
    let mut out = Vec::with_capacity(payload.len() + 8);
    out.extend_from_slice(magic);
    put_u32(&mut out, payload.len() as u32);
    out.extend_from_slice(&payload);
    out
}

/// Reads the simulator's records one after the other (`magic`, `u32` length, payload). A cut-off
/// last record (the recording ended mid-write) ends the iteration.
struct Records<'a> {
    bytes: &'a [u8],
    at: usize,
    magic: &'a [u8; 4],
}

impl<'a> Iterator for Records<'a> {
    type Item = &'a [u8];
    fn next(&mut self) -> Option<&'a [u8]> {
        let head = self.bytes.get(self.at..self.at + 8)?;
        if &head[..4] != self.magic {
            return None;
        }
        let len = u32::from_le_bytes([head[4], head[5], head[6], head[7]]) as usize;
        let payload = self.bytes.get(self.at + 8..self.at + 8 + len)?;
        self.at += 8 + len;
        Some(payload)
    }
}

struct Reader<'a> {
    b: &'a [u8],
    at: usize,
}

impl<'a> Reader<'a> {
    fn take(&mut self, n: usize) -> Option<&'a [u8]> {
        let s = self.b.get(self.at..self.at + n)?;
        self.at += n;
        Some(s)
    }
    fn u8(&mut self) -> Option<u8> {
        self.take(1).map(|s| s[0])
    }
    fn u32(&mut self) -> Option<u32> {
        self.take(4).map(|s| u32::from_le_bytes([s[0], s[1], s[2], s[3]]))
    }
    fn u64(&mut self) -> Option<u64> {
        self.take(8).map(|s| u64::from_le_bytes(s.try_into().unwrap()))
    }
    fn f32(&mut self) -> Option<f32> {
        self.take(4).map(|s| f32::from_le_bytes([s[0], s[1], s[2], s[3]]))
    }
    fn vec3(&mut self) -> Option<Vec3> {
        Some(Vec3::new(self.f32()?, self.f32()?, self.f32()?))
    }
}

fn decode_main(payload: &[u8], cal: &MockCalibration) -> Option<SensorFrame> {
    let mut r = Reader { b: payload, at: 0 };
    if r.u8()? != RECORD_VERSION {
        return None;
    }
    let (index, ts, w, h, scale, flags) = (r.u64()?, r.u64()?, r.u32()?, r.u32()?, r.f32()?, r.u8()?);
    let n = (w * h) as usize;
    let depth: Vec<u16> = r.take(n * 2)?.chunks_exact(2).map(|c| u16::from_le_bytes([c[0], c[1]])).collect();
    let left = if flags & 1 != 0 { Some(RgbImage { width: w, height: h, channels: 1, data: r.take(n)?.to_vec() }) } else { None };
    let right = if flags & 2 != 0 { Some(RgbImage { width: w, height: h, channels: 1, data: r.take(n)?.to_vec() }) } else { None };
    let imu = if flags & 4 != 0 {
        let t = r.u64()?;
        let q = Quat::from_xyzw(r.f32()?, r.f32()?, r.f32()?, r.f32()?);
        Some(ImuSample { timestamp_ms: t, orientation: q, angular_velocity: r.vec3()?, linear_acceleration: r.vec3()?, euler_degrees: r.vec3()? })
    } else {
        None
    };
    let mut frame = SensorFrame::new(index, ts);
    frame.depth = Some(DepthMap::new(w, h, scale, depth).with_intrinsics(cal.depth_intrinsics));
    frame.ir = left.map(|left| IrImages { left, right, rectified: true, rig: Some(Arc::new(cal.stereo.clone())) });
    frame.imu = imu;
    Some(frame)
}

fn decode_rgb(payload: &[u8], cal: &MockCalibration) -> Option<ColorFrame> {
    let mut r = Reader { b: payload, at: 0 };
    if r.u8()? != RECORD_VERSION {
        return None;
    }
    let (ts, w, h, ch) = (r.u64()?, r.u32()?, r.u32()?, r.u8()?);
    let data = r.take((w * h * ch as u32) as usize)?.to_vec();
    Some(ColorFrame { image: RgbImage { width: w, height: h, channels: ch, data }, jpeg: None, timestamp_ms: ts, camera: Some(Arc::new(cal.color.clone())) })
}

/// Decodes a recording of the simulator: `calibration` is its `mock.json`, `main` the main
/// stream, `rgb` the colour stream (paired to depth frames by timestamp). Frames come in recorded
/// order through `each`; returning `false` stops.
pub fn decode_recording(calibration: &[u8], main: &[u8], rgb: Option<&[u8]>, mut each: impl FnMut(SensorFrame) -> bool) -> Result<usize, String> {
    let cal: MockCalibration = serde_json::from_slice(calibration).map_err(|e| format!("{MOCK_CALIBRATION_FILE}: {e}"))?;
    let colours: std::collections::HashMap<u64, &[u8]> = rgb
        .map(|b| Records { bytes: b, at: 0, magic: RGB_MAGIC }.filter_map(|p| Some((u64::from_le_bytes(p.get(1..9)?.try_into().ok()?), p))).collect())
        .unwrap_or_default();
    let mut n = 0;
    for payload in (Records { bytes: main, at: 0, magic: MAIN_MAGIC }) {
        let Some(mut frame) = decode_main(payload, &cal) else { continue };
        frame.color = colours.get(&frame.timestamp_ms).and_then(|p| decode_rgb(p, &cal));
        n += 1;
        if !each(frame) {
            break;
        }
    }
    Ok(n)
}

fn open_recorder(path: &std::path::Path, append: bool, stats: &Arc<crate::recording::RecordStats>, queue: usize) -> Option<crate::recording::RawRecorder> {
    match crate::recording::RawRecorder::open(path, append, stats.clone(), queue) {
        Ok(r) => Some(r),
        Err(e) => {
            log::warn!("simulator: cannot record to {}: {e}", path.display());
            None
        }
    }
}

#[async_trait]
impl ScannerDevice for MockDevice {
    fn info(&self) -> &DeviceInfo {
        &self.info
    }

    fn capabilities(&self) -> &ScannerCapabilities {
        &self.capabilities
    }

    async fn start_preview(&mut self) -> Result<mpsc::Receiver<PreviewFrame>, DriverError> {
        let (tx, rx) = mpsc::channel(16);
        let streaming = self.streaming.clone();
        streaming.store(true, Ordering::SeqCst);

        tokio::spawn(async move {
            let mut ticker = interval(Duration::from_millis(66)); // ~15 FPS
            let mut frame_idx = 0u64;

            while streaming.load(Ordering::Relaxed) {
                ticker.tick().await;
                frame_idx += 1;
                let yaw = (frame_idx.saturating_sub(1) as f32) * ORBIT_STEP_RAD;
                let ts = frame_idx * 66;
                let frame = Self::render_synthetic_frame(320, 256, yaw, ts);

                let preview = PreviewFrame {
                    frame_index: frame_idx,
                    timestamp_ms: ts,
                    ir_image: Some(frame.ir),
                    rgb_image: Some(frame.color.image),
                    average_distance_mm: Some(265.0),
                    imu: Some(ImuSample {
                        timestamp_ms: ts,
                        orientation: frame.sensor_pose.rotation,
                        ..Default::default()
                    }),
                };

                if tx.send(preview).await.is_err() {
                    break;
                }
            }
        });

        Ok(rx)
    }

    async fn stop_preview(&mut self) -> Result<(), DriverError> {
        self.streaming.store(false, Ordering::SeqCst);
        Ok(())
    }

    async fn start_scan(&mut self, config: &ScanConfig) -> Result<mpsc::Receiver<SensorFrame>, DriverError> {
        let (tx, rx) = mpsc::channel(16);
        let streaming = Arc::new(AtomicBool::new(true));
        // A new scan ends the previous one's frame loop.
        std::mem::replace(&mut self.streaming, streaming.clone()).store(false, Ordering::SeqCst);
        let paused = self.paused.clone();
        paused.store(false, Ordering::SeqCst);
        let config = config.clone();
        let motion = self.motion;
        let emit_color = config.color_texture || config.tracking == crate::modes::TrackingMode::Color;
        // Recording as a real scanner's driver does it: the wire records of every frame sent,
        // through the non-blocking writer.
        let target = self.recording.take();
        let main = target.as_ref().and_then(|t| open_recorder(&t.main, t.append, &t.main_stats, t.queue_bytes));
        let rgb = target.as_ref().filter(|_| emit_color).and_then(|t| open_recorder(&t.rgb, t.append, &t.rgb_stats, t.queue_bytes));
        self.recording_files = main.iter().chain(rgb.iter()).map(|r| r.path().to_path_buf()).collect();

        tokio::spawn(async move {
            let mut ticker = interval(Duration::from_millis(motion.period_ms.max(1)));
            let mut frame_idx = 0u64;
            while streaming.load(Ordering::Relaxed) {
                ticker.tick().await;
                frame_idx += 1;
                // Paused like the POP 4: nothing is sent or recorded, the scene keeps moving.
                if paused.load(Ordering::Relaxed) {
                    continue;
                }
                let cfg = config.clone();
                let Ok(raw_frame) = tokio::task::spawn_blocking(move || Self::scan_frame(&cfg, &motion, frame_idx)).await else { break };
                if let Some(rec) = &main {
                    rec.push(encode_main_record(&raw_frame));
                }
                if let (Some(rec), Some(c)) = (&rgb, &raw_frame.color) {
                    rec.push(encode_rgb_record(c));
                }
                if tx.send(raw_frame).await.is_err() {
                    break;
                }
            }
            // The recorders close here: the writer threads flush in the background.
        });

        Ok(rx)
    }

    async fn pause_scan(&mut self) -> Result<(), DriverError> {
        self.paused.store(true, Ordering::SeqCst);
        Ok(())
    }

    async fn resume_scan(&mut self) -> Result<(), DriverError> {
        self.paused.store(false, Ordering::SeqCst);
        Ok(())
    }

    async fn stop_scan(&mut self) -> Result<(), DriverError> {
        self.streaming.store(false, Ordering::SeqCst);
        Ok(())
    }

    fn imu_stream(&mut self) -> Option<broadcast::Receiver<ImuSample>> {
        Some(self.imu_tx.subscribe())
    }

    async fn set_parameter(&mut self, _param: DeviceParam) -> Result<(), DriverError> {
        Ok(())
    }

    async fn disconnect(&mut self) -> Result<(), DriverError> {
        self.streaming.store(false, Ordering::SeqCst);
        Ok(())
    }

    fn set_recording(&mut self, target: Option<crate::recording::RecordingTarget>) -> bool {
        self.recording = target;
        true
    }

    fn recording_info(&self, config: &ScanConfig) -> Option<crate::recording::RecordingInfo> {
        let calibration = serde_json::to_vec_pretty(&Self::calibration()).unwrap_or_default();
        Some(crate::recording::RecordingInfo {
            stream: serde_json::json!({
                "format": MOCK_STREAM_FORMAT,
                "main": "records 'SCMK' u32 length, payload: version u8, frame index u64, timestamp ms u64, width u32, height u32, depth scale f32, flags u8 (1 left IR, 2 right IR, 4 IMU), depth u16[w*h], IR u8[w*h] per image, IMU",
                "rgb": "records 'SCMC' u32 length, payload: version u8, timestamp ms u64, width u32, height u32, channels u8, pixels",
                "width": 640,
                "height": 512,
                "turntable": config.turntable_mode != crate::types::TurntableMode::HandheldFreeform,
                "stereo_ir": matches!(config.tracking, crate::modes::TrackingMode::Marker | crate::modes::TrackingMode::GlobalMarker),
                "motion": { "orbit_step_rad": self.motion.orbit_step_rad, "turntable_step_rad": self.motion.turntable_step_rad, "period_ms": self.motion.period_ms },
                "commands": [],
            }),
            calibration: vec![(MOCK_CALIBRATION_FILE.to_string(), calibration)],
            files: self.recording_files.clone(),
        })
    }
}

#[cfg(test)]
mod intrinsics_tests {
    use super::*;

    #[test]
    fn synthetic_depth_back_projects_to_multi_object_scene() {
        let frame = MockDevice::render_synthetic_frame(640, 512, 0.0, 0);
        let cloud = frame.depth.to_point_cloud(150.0, 420.0);
        assert!(cloud.len() > 20_000, "dense scene points, got {}", cloud.len());

        // Transform sensor-frame points into world coordinates and verify each object exists.
        let mut turntable_pts = 0usize;
        let mut vase_pts = 0usize;
        let mut canister_pts = 0usize;
        let mut block_pts = 0usize;
        for p in &cloud.points {
            let w = frame.sensor_pose.transform_point3(Vec3::from(p.position));
            if w.z.abs() < 1.5 && (w.x * w.x + w.y * w.y).sqrt() <= 106.0 {
                turntable_pts += 1;
            }
            if w.z > 80.0 && w.z <= 116.0 && (w.x * w.x + w.y * w.y).sqrt() < 47.0 {
                vase_pts += 1;
            }
            if w.z > 5.0 && w.z <= 77.0 && ((w.x - 66.0).hypot(w.y - 14.0) < 22.5) {
                canister_pts += 1;
            }
            if w.z > 5.0 && w.z <= 49.0 && ((w.x + 62.0).hypot(w.y + 14.0) < 25.0) {
                block_pts += 1;
            }
        }
        assert!(turntable_pts > 5_000, "turntable_pts={turntable_pts}");
        assert!(vase_pts > 500, "vase_pts={vase_pts}");
        assert!(canister_pts > 500, "canister_pts={canister_pts}");
        assert!(block_pts > 500, "block_pts={block_pts}");
    }

    #[test]
    fn synthetic_color_projects_distinct_objects_and_backdrop() {
        let frame = MockDevice::render_synthetic_frame(640, 512, 0.0, 100);
        let cam = frame.color.camera.as_ref().expect("calibrated ColorCamera");
        assert_eq!((frame.color.image.width, frame.color.image.height, frame.color.image.channels), (640, 512, 3));

        // Top-left corner pixel belongs to cool backdrop (B > R).
        let img = &frame.color.image.data;
        assert!(img[2] > img[0], "corner backdrop should be cool: R={}, B={}", img[0], img[2]);

        let sample_world = |world: Vec3| -> [u8; 3] {
            let sensor = frame.sensor_pose.inverse().transform_point3(world);
            let (px, _) = cam.project_sensor(sensor).expect("in color view");
            let x = (px.x.round() as u32).min(639);
            let y = (px.y.round() as u32).min(511);
            let off = ((y * 640 + x) * 3) as usize;
            [img[off], img[off + 1], img[off + 2]]
        };

        // Terracotta vase center front face -> R > B and R > G.
        let vase_rgb = sample_world(Vec3::new(0.0, -30.0, 55.0));
        assert!(vase_rgb[0] > vase_rgb[1] && vase_rgb[0] > vase_rgb[2], "vase_rgb={vase_rgb:?}");

        // Cobalt canister front face -> B > R and B > G.
        let can_rgb = sample_world(Vec3::new(66.0, -7.0, 40.0));
        assert!(can_rgb[2] > can_rgb[0] && can_rgb[2] > can_rgb[1], "can_rgb={can_rgb:?}");

        // Sage-green block top/front face -> G > R and G > B.
        let blk_rgb = sample_world(Vec3::new(-62.0, -14.0, 48.0));
        assert!(blk_rgb[1] > blk_rgb[0] && blk_rgb[1] > blk_rgb[2], "blk_rgb={blk_rgb:?}");
    }
}

#[cfg(test)]
mod recording_tests {
    use super::*;
    use crate::modes::TrackingMode;
    use crate::types::TurntableMode;

    #[test]
    fn a_recorded_frame_decodes_to_the_frame_sent() {
        let config = ScanConfig { turntable_mode: TurntableMode::TurntableContinuous, tracking: TrackingMode::Marker, color_texture: true, ..ScanConfig::default() };
        let motion = MockMotion::default();
        let frame = MockDevice::scan_frame(&config, &motion, 5);
        let mut main = encode_main_record(&frame);
        // A cut-off record at the end (the recording stopped mid-write) is ignored.
        let second = encode_main_record(&MockDevice::scan_frame(&config, &motion, 6));
        main.extend_from_slice(&second[..second.len() / 2]);
        let rgb = encode_rgb_record(frame.color.as_ref().unwrap());
        let cal = serde_json::to_vec(&MockDevice::calibration()).unwrap();
        let mut got = Vec::new();
        let n = decode_recording(&cal, &main, Some(&rgb), |f| {
            got.push(f);
            true
        })
        .unwrap();
        assert_eq!(n, 1);
        let g = &got[0];
        assert_eq!((g.frame_index, g.timestamp_ms), (5, 5 * motion.period_ms));
        assert_eq!(g.depth.as_ref().unwrap().data, frame.depth.as_ref().unwrap().data);
        assert_eq!(g.depth.as_ref().unwrap().intrinsics, frame.depth.as_ref().unwrap().intrinsics);
        let (gi, fi) = (g.ir.as_ref().unwrap(), frame.ir.as_ref().unwrap());
        assert_eq!(gi.left.data, fi.left.data);
        assert_eq!(gi.right.as_ref().unwrap().data, fi.right.as_ref().unwrap().data, "the right IR image of a marker pass");
        assert_eq!(gi.rig, fi.rig);
        assert_eq!(g.imu.unwrap().orientation, frame.imu.unwrap().orientation);
        assert_eq!(g.color.as_ref().unwrap().image.data, frame.color.as_ref().unwrap().image.data);
    }

    #[test]
    fn the_turntable_turns_under_a_fixed_scanner_and_the_imu_stands_still() {
        let config = ScanConfig { turntable_mode: TurntableMode::TurntableContinuous, ..ScanConfig::default() };
        let motion = MockMotion::default();
        let (a, b) = (MockDevice::scan_frame(&config, &motion, 1), MockDevice::scan_frame(&config, &motion, 31));
        assert_eq!(a.imu.unwrap().orientation, b.imu.unwrap().orientation, "the scanner does not move");
        assert_eq!(b.imu.unwrap().angular_velocity, Vec3::ZERO);
        // The static table stays where it was; the turntable top changed (it turned 36 degrees).
        let (da, db) = (&a.depth.as_ref().unwrap().data, &b.depth.as_ref().unwrap().data);
        let changed = da.iter().zip(db).filter(|(x, y)| x != y).count();
        assert!(changed > 2_000 && changed < da.len() / 2, "{changed} of {} pixels changed", da.len());
        // Ground truth: the sensor orbits the turntable by the turn, about the turntable axis.
        let (t1, t31) = (MockDevice::scan_truth(&config, &motion, 1), MockDevice::scan_truth(&config, &motion, 31));
        let turned = t1.rotation.angle_between(t31.rotation);
        assert!((turned - 30.0 * motion.turntable_step_rad).abs() < 1e-4, "{}", turned.to_degrees());
        assert!((t1.translation.length() - t31.translation.length()).abs() < 1e-3);
        // Marker stickers are bright in the IR image.
        let bright = a.ir.as_ref().unwrap().left.data.iter().filter(|&&v| v >= 240).count();
        assert!(bright > 100, "{bright} marker pixels");
        // Handheld passes orbit the static scene as before.
        let hand = MockDevice::scan_frame(&ScanConfig::default(), &motion, 31);
        assert!(hand.imu.unwrap().angular_velocity.z > 0.0);
        assert!(hand.ir.as_ref().unwrap().right.is_none(), "an IR pair only for marker tracking");
    }
}
