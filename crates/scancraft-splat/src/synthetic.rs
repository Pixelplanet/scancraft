//! A synthetic object scan with known cameras, for testing without colour data from a scanner.
//!
//! The object (a sphere with a box on top, ~90 mm tall) carries a procedural, view-independent
//! texture evaluated per pixel from the surface position, so the ground truth has fine detail
//! independent of the mesh resolution. Views are rendered with a small CPU rasteriser (2x2
//! supersampled, perspective-correct) on a black background, with a foreground mask. Cameras sit
//! on rings around the object at elevations of 15°–65°, so the underside is never seen — the
//! coverage filter has to remove it from the initial cloud.

use crate::types::{CameraAxes, Intrinsics, PosedImage, SplatDataset};
use glam::{Affine3A, Mat3, Vec3};
use rayon::prelude::*;
use scancraft_core::{ColorRgba, Point3D, PointCloud};

#[derive(Debug, Clone)]
pub struct SceneConfig {
    pub width: u32,
    pub height: u32,
    pub focal: f32,
    /// Camera distance from the object centre, mm.
    pub distance: f32,
    /// Camera ring elevations in degrees, and views per ring.
    pub elevations_deg: Vec<f32>,
    pub views_per_ring: usize,
    /// Points sampled on the surface for the initial cloud.
    pub init_points: usize,
    /// Gaussian-ish positional noise of the initial cloud, mm.
    pub init_noise_mm: f32,
    /// Give the initial cloud flat grey colours (like a geometry-only pass).
    pub grey_init: bool,
    pub seed: u32,
}

impl Default for SceneConfig {
    fn default() -> Self {
        Self {
            width: 640,
            height: 480,
            focal: 760.0,
            distance: 230.0,
            elevations_deg: vec![15.0, 40.0, 65.0],
            views_per_ring: 16,
            init_points: 40_000,
            init_noise_mm: 0.1,
            grey_init: true,
            seed: 1,
        }
    }
}

/// Object centre (mm, ScanCraft world).
pub const CENTER: Vec3 = Vec3::new(0.0, 0.0, 45.0);

struct Tri {
    v: [Vec3; 3],
}

fn mesh() -> Vec<Tri> {
    let mut tris = Vec::new();
    // Sphere, radius 35 mm, centred at z = 35.
    let (r, c) = (35.0, Vec3::new(0.0, 0.0, 35.0));
    let (nu, nv) = (96, 48);
    let p = |i: usize, j: usize| {
        let th = std::f32::consts::TAU * i as f32 / nu as f32;
        let ph = std::f32::consts::PI * j as f32 / nv as f32;
        c + r * Vec3::new(ph.sin() * th.cos(), ph.sin() * th.sin(), ph.cos())
    };
    for i in 0..nu {
        for j in 0..nv {
            let (a, b, cc, d) = (p(i, j), p(i + 1, j), p(i + 1, j + 1), p(i, j + 1));
            tris.push(Tri { v: [a, d, cc] });
            tris.push(Tri { v: [a, cc, b] });
        }
    }
    // Box 36 x 24 x 22 mm sitting on top of the sphere, rotated 30° about Z.
    let rot = Mat3::from_rotation_z(30f32.to_radians());
    let (h, base) = (Vec3::new(18.0, 12.0, 11.0), Vec3::new(0.0, 0.0, 68.0 + 11.0));
    let corner = |s: [f32; 3]| base + rot * (h * Vec3::from_array(s));
    let faces = [
        [[-1., -1., -1.], [1., -1., -1.], [1., 1., -1.], [-1., 1., -1.]],
        [[-1., -1., 1.], [-1., 1., 1.], [1., 1., 1.], [1., -1., 1.]],
        [[-1., -1., -1.], [-1., -1., 1.], [1., -1., 1.], [1., -1., -1.]],
        [[-1., 1., -1.], [1., 1., -1.], [1., 1., 1.], [-1., 1., 1.]],
        [[-1., -1., -1.], [-1., 1., -1.], [-1., 1., 1.], [-1., -1., 1.]],
        [[1., -1., -1.], [1., -1., 1.], [1., 1., 1.], [1., 1., -1.]],
    ];
    for f in faces {
        let q = f.map(corner);
        tris.push(Tri { v: [q[0], q[1], q[2]] });
        tris.push(Tri { v: [q[0], q[2], q[3]] });
    }
    tris
}

fn hsv(h: f32, s: f32, v: f32) -> Vec3 {
    let k = |n: f32| {
        let k = (n + h * 6.0) % 6.0;
        v - v * s * k.min(4.0 - k).clamp(0.0, 1.0)
    };
    Vec3::new(k(5.0), k(3.0), k(1.0))
}

/// The surface colour at a world point (linear 0..1, displayed as is).
pub fn texture(p: Vec3) -> Vec3 {
    if p.z > 68.0 {
        // Box: checkerboard, 6 mm squares, two hues.
        let q = (p / 6.0).floor();
        let odd = ((q.x + q.y + q.z) as i32).rem_euclid(2) == 1;
        return if odd { Vec3::new(0.95, 0.85, 0.2) } else { Vec3::new(0.15, 0.25, 0.75) };
    }
    // Sphere: hue by azimuth, lighter/darker bands by height, thin dark meridians.
    let az = p.y.atan2(p.x) / std::f32::consts::TAU + 0.5;
    let band = ((p.z / 7.0).floor() as i32).rem_euclid(2) as f32;
    let mut c = hsv(az, 0.75, 0.55 + 0.35 * band);
    let meridian = (az * 24.0).fract();
    if meridian < 0.06 {
        c *= 0.15;
    }
    c
}

/// Cameras looking at [`CENTER`] from the configured rings.
pub fn cameras(cfg: &SceneConfig) -> Vec<(String, Affine3A)> {
    let mut out = Vec::new();
    for (ri, el) in cfg.elevations_deg.iter().enumerate() {
        for k in 0..cfg.views_per_ring {
            // Offset alternate rings so views interleave.
            let az = std::f32::consts::TAU * (k as f32 + 0.5 * ri as f32) / cfg.views_per_ring as f32;
            let el = el.to_radians();
            let eye = CENTER + cfg.distance * Vec3::new(el.cos() * az.cos(), el.cos() * az.sin(), el.sin());
            // ScanCraft camera axes: +Y forward, +Z up, +X right.
            let forward = (CENTER - eye).normalize();
            let right = forward.cross(Vec3::Z).normalize();
            let up = right.cross(forward);
            let pose = Affine3A::from_mat3_translation(Mat3::from_cols(right, forward, up), eye);
            out.push((format!("view_{ri}_{k:02}"), pose));
        }
    }
    out
}

/// Render one view: RGB8 (on black) and an 8-bit coverage mask.
pub fn render(width: u32, height: u32, intr: &Intrinsics, camera_to_world: Affine3A) -> (Vec<u8>, Vec<u8>) {
    const SS: u32 = 2;
    let (w, h) = (width * SS, height * SS);
    let (fx, fy, cx, cy) = (intr.fx * SS as f32, intr.fy * SS as f32, intr.cx * SS as f32, intr.cy * SS as f32);
    // World -> OpenCV camera.
    let c2w_cv = camera_to_world * Affine3A::from_mat3(crate::types::opencv_to_scancraft_camera());
    let w2c = c2w_cv.inverse();
    let mut depth = vec![f32::INFINITY; (w * h) as usize];
    let mut pos = vec![Vec3::ZERO; (w * h) as usize];
    for tri in mesh() {
        let cam = tri.v.map(|v| w2c.transform_point3(v));
        if cam.iter().any(|c| c.z < 1.0) {
            continue;
        }
        let scr = cam.map(|c| glam::Vec2::new(fx * c.x / c.z + cx, fy * c.y / c.z + cy));
        let area = (scr[1] - scr[0]).perp_dot(scr[2] - scr[0]);
        if area.abs() < 1e-9 {
            continue;
        }
        let min = scr.iter().fold(glam::Vec2::splat(f32::INFINITY), |a, b| a.min(*b)).max(glam::Vec2::ZERO);
        let max = scr.iter().fold(glam::Vec2::splat(f32::NEG_INFINITY), |a, b| a.max(*b)).min(glam::Vec2::new(w as f32 - 1.0, h as f32 - 1.0));
        if min.x > max.x || min.y > max.y {
            continue;
        }
        for py in min.y.floor() as u32..=max.y.ceil() as u32 {
            for px in min.x.floor() as u32..=max.x.ceil() as u32 {
                if px >= w || py >= h {
                    continue;
                }
                let s = glam::Vec2::new(px as f32 + 0.5, py as f32 + 0.5);
                let b0 = (scr[2] - scr[1]).perp_dot(s - scr[1]) / area;
                let b1 = (scr[0] - scr[2]).perp_dot(s - scr[2]) / area;
                let b2 = 1.0 - b0 - b1;
                if b0 < 0.0 || b1 < 0.0 || b2 < 0.0 {
                    continue;
                }
                // Perspective-correct interpolation.
                let iz = b0 / cam[0].z + b1 / cam[1].z + b2 / cam[2].z;
                let z = 1.0 / iz;
                let idx = (py * w + px) as usize;
                if z < depth[idx] {
                    depth[idx] = z;
                    pos[idx] = (tri.v[0] * (b0 / cam[0].z) + tri.v[1] * (b1 / cam[1].z) + tri.v[2] * (b2 / cam[2].z)) * z;
                }
            }
        }
    }
    let mut rgb = vec![0u8; (width * height * 3) as usize];
    let mut mask = vec![0u8; (width * height) as usize];
    for y in 0..height {
        for x in 0..width {
            let (mut c, mut cover) = (Vec3::ZERO, 0.0);
            for sy in 0..SS {
                for sx in 0..SS {
                    let idx = ((y * SS + sy) * w + x * SS + sx) as usize;
                    if depth[idx].is_finite() {
                        c += texture(pos[idx]);
                        cover += 1.0;
                    }
                }
            }
            let n = (SS * SS) as f32;
            let o = ((y * width + x) * 3) as usize;
            for k in 0..3 {
                rgb[o + k] = ((c[k] / n).clamp(0.0, 1.0) * 255.0).round() as u8;
            }
            mask[(y * width + x) as usize] = (cover / n * 255.0).round() as u8;
        }
    }
    (rgb, mask)
}

fn hash(mut x: u32) -> u32 {
    x ^= x >> 16;
    x = x.wrapping_mul(0x7feb_352d);
    x ^= x >> 15;
    x = x.wrapping_mul(0x846c_a68b);
    x ^ (x >> 16)
}

fn unit(seed: u32, i: u32) -> f32 {
    hash(seed.wrapping_mul(0x9E37_79B9) ^ i) as f32 / u32::MAX as f32
}

/// Area-uniform samples on the object surface (the "merged point cloud").
pub fn surface_cloud(cfg: &SceneConfig) -> PointCloud {
    let tris = mesh();
    let areas: Vec<f32> = tris.iter().map(|t| (t.v[1] - t.v[0]).cross(t.v[2] - t.v[0]).length() * 0.5).collect();
    let total: f32 = areas.iter().sum();
    let mut cdf = Vec::with_capacity(areas.len());
    let mut acc = 0.0;
    for a in &areas {
        acc += a / total;
        cdf.push(acc);
    }
    let pts = (0..cfg.init_points as u32)
        .map(|i| {
            let r = unit(cfg.seed, i * 6);
            let ti = cdf.partition_point(|c| *c < r).min(tris.len() - 1);
            let t = &tris[ti];
            let (mut u, mut v) = (unit(cfg.seed, i * 6 + 1), unit(cfg.seed, i * 6 + 2));
            if u + v > 1.0 {
                (u, v) = (1.0 - u, 1.0 - v);
            }
            let p = t.v[0] + (t.v[1] - t.v[0]) * u + (t.v[2] - t.v[0]) * v;
            let n = (t.v[1] - t.v[0]).cross(t.v[2] - t.v[0]).normalize();
            let noise = Vec3::new(unit(cfg.seed, i * 6 + 3), unit(cfg.seed, i * 6 + 4), unit(cfg.seed, i * 6 + 5)) * 2.0 - 1.0;
            let p = p + noise * cfg.init_noise_mm;
            let color = if cfg.grey_init {
                ColorRgba::GRAY
            } else {
                let c = texture(p);
                ColorRgba::new((c.x * 255.0) as u8, (c.y * 255.0) as u8, (c.z * 255.0) as u8, 255)
            };
            Point3D::new(p.to_array()).with_normal(n.to_array()).with_color(color)
        })
        .collect();
    PointCloud::from_points(pts)
}

/// The full synthetic dataset: rendered views (with masks) and the initial cloud.
pub fn dataset(cfg: &SceneConfig) -> SplatDataset {
    let intr = Intrinsics {
        fx: cfg.focal,
        fy: cfg.focal,
        cx: cfg.width as f32 / 2.0,
        cy: cfg.height as f32 / 2.0,
        distortion: None,
    };
    let images = cameras(cfg)
        .into_par_iter()
        .map(|(name, pose)| {
            let (rgb, mask) = render(cfg.width, cfg.height, &intr, pose);
            PosedImage {
                name,
                width: cfg.width,
                height: cfg.height,
                rgb,
                mask: Some(mask),
                intrinsics: intr,
                camera_to_world: pose,
                axes: CameraAxes::ScanCraft,
            }
        })
        .collect();
    SplatDataset { images, init_cloud: surface_cloud(cfg) }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn object_is_centred_and_bottom_is_unseen() {
        let cfg = SceneConfig { width: 160, height: 120, focal: 190.0, views_per_ring: 6, init_points: 4000, ..Default::default() };
        let ds = dataset(&cfg);
        assert_eq!(ds.images.len(), 18);
        for img in &ds.images {
            let m = img.mask.as_ref().unwrap();
            let fg = m.iter().filter(|&&v| v > 128).count();
            // Object covers a reasonable part of the frame and is around the centre.
            assert!(fg > 1000 && fg < 160 * 120 / 2, "{}: {fg}", img.name);
            let centre = m[60 * 160 + 80];
            assert_eq!(centre, 255, "{}", img.name);
        }
        let counts = crate::coverage::view_counts(&ds.init_cloud, &ds.images, &Default::default());
        let (mut bottom_seen, mut bottom, mut top_seen, mut top) = (0, 0, 0, 0);
        for (p, c) in ds.init_cloud.points.iter().zip(&counts) {
            if p.position[2] < 3.0 {
                bottom += 1;
                bottom_seen += (*c > 0) as i32;
            } else if p.position[2] > 72.0 {
                // The box above its bottom face (which rests on the sphere and is hidden).
                top += 1;
                top_seen += (*c > 0) as i32;
            }
        }
        assert!(bottom > 10 && bottom_seen == 0, "bottom {bottom_seen}/{bottom}");
        assert!(top_seen as f32 > 0.95 * top as f32, "top {top_seen}/{top}");
    }
}
