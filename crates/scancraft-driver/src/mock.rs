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
    imu_tx: broadcast::Sender<ImuSample>,
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
}

#[derive(Clone, Copy, Debug)]
struct RayHit {
    t: f32,
    pos: Vec3,
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

impl CameraRig {
    fn new(width: u32, height: u32, yaw_rad: f32) -> Self {
        // Focus at mid-height of the turntable scene (~44 mm above z = 0).
        let target = Vec3::new(0.0, 0.0, 44.0);
        let dist = 280.0f32;
        let elev = 22.0f32.to_radians();
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
        let dx = (x as f32 - self.intrinsics.cx) / self.intrinsics.fx;
        let dz = -(y as f32 - self.intrinsics.cy) / self.intrinsics.fy;
        // Local ray (dx, 1, dz) guarantees t == depth along the optical axis (+Y).
        self.rot_mat * Vec3::new(dx, 1.0, dz)
    }
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

fn trace_scene(orig: Vec3, dir: Vec3) -> Option<RayHit> {
    let mut best: Option<RayHit> = None;
    let candidates = [
        intersect_cylinder_finite(orig, dir, 0.0, 0.0, 105.0, -8.0, 0.0, SceneMaterial::Turntable),
        intersect_vase(orig, dir),
        intersect_cylinder_finite(orig, dir, 66.0, 14.0, 21.0, 0.0, 76.0, SceneMaterial::Canister),
        intersect_block(orig, dir),
    ];
    for hit in candidates.into_iter().flatten() {
        if best.is_none_or(|b| hit.t < b.t) {
            best = Some(hit);
        }
    }
    best
}

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
        SceneMaterial::Turntable => 145.0,
        SceneMaterial::Vase => 195.0,
        SceneMaterial::Canister => 165.0,
        SceneMaterial::Block => 175.0,
    };
    (albedo * (0.30 + 0.70 * key)).clamp(24.0, 255.0) as u8
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
                    tracking_modes: vec![(crate::modes::TrackingMode::Feature, crate::modes::Availability::Available)],
                    object_types: vec![(crate::modes::ObjectType::General, crate::modes::Availability::Available)],
                    combinations: Vec::new(),
                },
            },
            streaming: Arc::new(AtomicBool::new(false)),
            imu_tx,
        }
    }

    fn render_depth_ir(rig: &CameraRig) -> (DepthMap, RgbImage) {
        let (w, h) = (rig.width, rig.height);
        let len = (w * h) as usize;
        let mut depth_data = vec![0u16; len];
        let mut ir_data = vec![16u8; len];
        let orig = rig.sensor_pose.translation;

        for y in 0..h {
            for x in 0..w {
                let dir = rig.ray_dir(x, y);
                let hit = trace_scene(orig, dir);
                let idx = (y * w + x) as usize;
                if let Some(hit_info) = hit {
                    depth_data[idx] = (hit_info.t / 0.1).round().clamp(1000.0, 60000.0) as u16;
                }
                ir_data[idx] = shade_ir(hit);
            }
        }

        let depth = DepthMap::new(w, h, 0.1, depth_data).with_intrinsics(rig.intrinsics);
        let ir = RgbImage { width: w, height: h, channels: 1, data: ir_data };
        (depth, ir)
    }

    fn render_color_from_rig(rig: &CameraRig, timestamp_ms: u64) -> ColorFrame {
        let (w, h) = (rig.width, rig.height);
        let mut rgb_data = vec![0u8; (w * h * 3) as usize];
        let orig = rig.color_origin;

        for y in 0..h {
            for x in 0..w {
                let dir = rig.ray_dir(x, y);
                let hit = trace_scene(orig, dir);
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
        let streaming = self.streaming.clone();
        let emit_color = config.color_texture || config.tracking == crate::modes::TrackingMode::Color;
        streaming.store(true, Ordering::SeqCst);

        tokio::spawn(async move {
            let mut ticker = interval(Duration::from_millis(66)); // ~15 FPS
            let mut frame_idx = 0u64;

            while streaming.load(Ordering::Relaxed) {
                ticker.tick().await;
                frame_idx += 1;
                let yaw = (frame_idx.saturating_sub(1) as f32) * ORBIT_STEP_RAD;
                let ts = frame_idx * 66;
                let rig = CameraRig::new(640, 512, yaw);
                let (depth, ir) = Self::render_depth_ir(&rig);
                let color = emit_color.then(|| Self::render_color_from_rig(&rig, ts));

                let imu_sample = ImuSample {
                    timestamp_ms: ts,
                    orientation: rig.sensor_pose.rotation,
                    angular_velocity: Vec3::new(0.0, 0.0, 0.032 * 15.0),
                    linear_acceleration: Vec3::new(0.0, 0.0, 1.0),
                    euler_degrees: Vec3::new(0.0, 0.0, yaw.to_degrees()),
                };

                let mut raw_frame = SensorFrame::new(frame_idx, ts);
                raw_frame.depth = Some(depth);
                raw_frame.color = color;
                raw_frame.ir = Some(IrImages { left: ir, right: None, rectified: true, rig: None });
                raw_frame.imu = Some(imu_sample);

                if tx.send(raw_frame).await.is_err() {
                    break;
                }
            }
        });

        Ok(rx)
    }

    async fn pause_scan(&mut self) -> Result<(), DriverError> {
        Ok(())
    }

    async fn resume_scan(&mut self) -> Result<(), DriverError> {
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
