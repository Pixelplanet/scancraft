//! Verifies the RGB calibration (`Prgb.bin`, `Distort.bin`, `LC_RT.bin`) on real data by
//! measuring how far projected depth-frame content sits from where the colour image shows it.
//!
//! `cargo run --release -p scancraft-color --example color_calib_check -- <dir> <depth.bin> <rgb.bin> [out_dir]`
//!
//! `<dir>` holds the calibration files and both recordings (docs/protocol/color.md). Two
//! measurements per paired frame and candidate extrinsic convention, each searching the image
//! shift (du, dv) that best aligns the projection with the colour image (a correct model peaks
//! at (0, 0); the peak offset is the misalignment in colour pixels at 1280x1024):
//!
//! 1. **IR gradients**: the left IR image (same frame as depth) is warped into the colour image
//!    through the depth map; normalised cross-correlation of the two gradient-magnitude images.
//!    Dense, uses every pixel with depth.
//! 2. **Silhouettes**: depth discontinuities (occluding edges) projected into the colour image;
//!    mean colour-gradient magnitude under them.

mod common;

use glam::{Mat3, Vec2, Vec3};
use scancraft_color::image_ops::{blur3, gradient_magnitude, to_gray, Gray};
use scancraft_core::{ColorCamera, DepthMap, RgbImage};
use std::path::PathBuf;

fn unproject(depth: &DepthMap, u: usize, v: usize) -> Option<Vec3> {
    let k = depth.intrinsics?;
    let d = depth.data[v * depth.width as usize + u] as f32 * depth.scale_mm;
    (d > 0.0).then(|| Vec3::new((u as f32 - k.cx) * d / k.fx, d, -(v as f32 - k.cy) * d / k.fy))
}

/// Scanner-frame points on the near side of depth jumps (> 25 mm over 2 px).
fn silhouette_points(depth: &DepthMap) -> Vec<Vec3> {
    let (w, h) = (depth.width as usize, depth.height as usize);
    let z = |u: usize, v: usize| depth.data[v * w + u] as f32 * depth.scale_mm;
    let mut out = Vec::new();
    for v in 3..h - 3 {
        for u in 3..w - 3 {
            let d = z(u, v);
            if d <= 0.0 || v * w + u < 64 {
                continue;
            }
            let edge = [(2isize, 0isize), (-2, 0), (0, 2), (0, -2)]
                .iter()
                .any(|&(du, dv)| z((u as isize + du) as usize, (v as isize + dv) as usize) > d + 25.0);
            if edge {
                out.extend(unproject(depth, u, v));
            }
        }
    }
    out
}

/// The IR image warped into the colour camera (z-buffered 3x3 splats); `None` where nothing lands.
fn warp_ir(depth: &DepthMap, ir: &RgbImage, cam: &ColorCamera) -> (Vec<Option<f32>>, Vec<f32>) {
    let (w, h) = (cam.width as usize, cam.height as usize);
    let mut out = vec![None; w * h];
    let mut zbuf = vec![f32::INFINITY; w * h];
    for v in 0..depth.height as usize {
        for u in 0..depth.width as usize {
            if v * depth.width as usize + u < 64 {
                continue;
            }
            let Some(p) = unproject(depth, u, v) else { continue };
            let Some((q, z)) = cam.project_sensor(p) else { continue };
            let val = ir.data[v * ir.width as usize + u] as f32;
            for dy in -1..=1 {
                for dx in -1..=1 {
                    let (x, y) = (q.x.round() as isize + dx, q.y.round() as isize + dy);
                    if x < 0 || y < 0 || x as usize >= w || y as usize >= h {
                        continue;
                    }
                    let i = y as usize * w + x as usize;
                    if z < zbuf[i] {
                        zbuf[i] = z;
                        out[i] = Some(val);
                    }
                }
            }
        }
    }
    (out, zbuf)
}

/// Gradient magnitude of a sparse image, defined only where the full 3x3 neighbourhood is.
fn sparse_gradient(img: &[Option<f32>], w: usize, h: usize) -> Vec<Option<f32>> {
    let mut out = vec![None; w * h];
    for y in 1..h - 1 {
        for x in 1..w - 1 {
            let p = |dx: isize, dy: isize| img[(y as isize + dy) as usize * w + (x as isize + dx) as usize];
            let mut n = [0f32; 9];
            let mut ok = true;
            for (k, (dx, dy)) in (-1..=1).flat_map(|dy| (-1..=1).map(move |dx| (dx, dy))).enumerate() {
                match p(dx, dy) {
                    Some(v) => n[k] = v,
                    None => ok = false,
                }
            }
            if ok {
                let gx = n[2] + 2.0 * n[5] + n[8] - n[0] - 2.0 * n[3] - n[6];
                let gy = n[6] + 2.0 * n[7] + n[8] - n[0] - 2.0 * n[1] - n[2];
                out[y * w + x] = Some(gx.abs() + gy.abs());
            }
        }
    }
    out
}

/// Shift maximising `score`, searched over +-`range` px at 1 px then refined at 1/8 px.
fn search(range: i32, score: impl Fn(Vec2) -> f32) -> (Vec2, f32) {
    let mut best = (Vec2::ZERO, f32::MIN);
    for dv in -range..=range {
        for du in -range..=range {
            let s = Vec2::new(du as f32, dv as f32);
            let sc = score(s);
            if sc > best.1 {
                best = (s, sc);
            }
        }
    }
    let coarse = best.0;
    for dv in -8..=8 {
        for du in -8..=8 {
            let s = coarse + Vec2::new(du as f32, dv as f32) * 0.125;
            let sc = score(s);
            if sc > best.1 {
                best = (s, sc);
            }
        }
    }
    best
}

fn bilinear(grad: &[f32], w: usize, h: usize, q: Vec2) -> Option<f32> {
    if q.x < 0.0 || q.y < 0.0 || q.x >= (w - 1) as f32 || q.y >= (h - 1) as f32 {
        return None;
    }
    let (x0, y0) = (q.x as usize, q.y as usize);
    let (fx, fy) = (q.x - x0 as f32, q.y - y0 as f32);
    let g = |x: usize, y: usize| grad[y * w + x];
    Some((g(x0, y0) * (1.0 - fx) + g(x0 + 1, y0) * fx) * (1.0 - fy) + (g(x0, y0 + 1) * (1.0 - fx) + g(x0 + 1, y0 + 1) * fx) * fy)
}

/// NCC between the warped-IR gradient samples and the colour gradient image shifted by `s`.
fn ncc(samples: &[(Vec2, f32)], grad: &[f32], w: usize, h: usize, s: Vec2) -> f32 {
    let (mut sa, mut sb, mut saa, mut sbb, mut sab, mut n) = (0f64, 0f64, 0f64, 0f64, 0f64, 0f64);
    for &(p, a) in samples {
        let Some(b) = bilinear(grad, w, h, p + s) else { continue };
        let (a, b) = (a as f64, b as f64);
        sa += a;
        sb += b;
        saa += a * a;
        sbb += b * b;
        sab += a * b;
        n += 1.0;
    }
    if n < 100.0 {
        return f32::MIN;
    }
    let cov = sab / n - sa / n * sb / n;
    let va = saa / n - (sa / n).powi(2);
    let vb = sbb / n - (sb / n).powi(2);
    (cov / (va * vb).sqrt().max(1e-9)) as f32
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let dir = PathBuf::from(args.get(1).expect("usage: color_calib_check <dir> <depth.bin> <rgb.bin> [out_dir]"));
    let out_dir = args.get(4).map(PathBuf::from);
    let (pairs, cam) = common::load_pairs(&dir, &args[2], &args[3], 40);
    println!("{} paired frames", pairs.len());

    let as_given = (*cam).clone();
    let inverse = ColorCamera { rotation: cam.rotation.transpose(), translation: -(cam.rotation.transpose() * cam.translation), ..(*cam).clone() };
    let translation_only = ColorCamera { rotation: Mat3::IDENTITY, ..(*cam).clone() };
    let no_distortion = ColorCamera { distortion: [0.0; 5], ..(*cam).clone() };
    let variants = [("p_rgb = R p + T", as_given), ("inverse", inverse), ("T only (R = I)", translation_only), ("no distortion", no_distortion)];

    let mut ir_results = vec![Vec::new(); variants.len()];
    let mut edge_results = vec![Vec::new(); variants.len()];
    for (i, pair) in pairs.iter().enumerate().step_by(4) {
        let gray: Gray = blur3(&to_gray(&pair.color.image));
        let (w, h) = (gray.width, gray.height);
        let grad = gradient_magnitude(&gray);
        let edges = silhouette_points(&pair.depth);
        {
            // Depth vs its own IR image (no colour involved): are the planes pixel-registered?
            let k = pair.depth.intrinsics.unwrap();
            let ir = scancraft_color::image_ops::downsample2(&scancraft_color::image_ops::downsample2(&to_gray(&pair.ir_left)));
            let ir = blur3(&ir);
            let (iw, ih) = (ir.width, ir.height);
            let igrad = gradient_magnitude(&ir);
            let px: Vec<Vec2> = edges.iter().map(|p| Vec2::new((k.fx * p.x / p.y + k.cx - 1.5) / 4.0, (-k.fy * p.z / p.y + k.cy - 1.5) / 4.0)).collect();
            let (s_ir, _) = search(16, |s| {
                let v: Vec<f32> = px.iter().filter_map(|q| bilinear(&igrad, iw, ih, *q + s / 4.0)).collect();
                v.iter().sum::<f32>() / v.len().max(1) as f32
            });
            println!("    depth silhouettes vs own IR image: shift ({:+.2}, {:+.2}) depth px", s_ir.x, s_ir.y);
        }
        for (vi, (name, model)) in variants.iter().enumerate() {
            // 1. Warped IR gradients at 1/4 resolution (the IR image carries projector speckle,
            //    which a 4x4 box filter removes).
            let (warped, zbuf) = warp_ir(&pair.depth, &pair.ir_left, model);
            let (qw, qh) = (w / 4, h / 4);
            let quarter = |src: &dyn Fn(usize) -> Option<f32>| -> Vec<Option<f32>> {
                (0..qw * qh)
                    .map(|i| {
                        let (x, y) = (i % qw * 4, i / qw * 4);
                        let mut sum = 0.0;
                        for dy in 0..4 {
                            for dx in 0..4 {
                                sum += src((y + dy) * w + x + dx)?;
                            }
                        }
                        Some(sum / 16.0)
                    })
                    .collect()
            };
            let wq = quarter(&|i| warped[i]);
            let gq = quarter(&|i| Some(gray.data[i] as f32));
            let wgrad = sparse_gradient(&wq, qw, qh);
            let cgrad: Vec<f32> = sparse_gradient(&gq, qw, qh).into_iter().map(|g| g.unwrap_or(0.0)).collect();
            let samples: Vec<(Vec2, f32)> =
                (0..qw * qh).filter_map(|i| wgrad[i].map(|g| (Vec2::new((i % qw) as f32, (i / qw) as f32), g))).collect();
            let (ir_shift, ir_peak) = search(20, |s| ncc(&samples, &cgrad, qw, qh, s / 4.0));
            let ir_zero = ncc(&samples, &cgrad, qw, qh, Vec2::ZERO);
            // The same split by depth: a translation error shifts near content more than far.
            let z_of = |p: Vec2| zbuf[(p.y as usize * 4 + 2) * w + p.x as usize * 4 + 2];
            let near: Vec<(Vec2, f32)> = samples.iter().copied().filter(|(p, _)| z_of(*p) < 330.0).collect();
            let far: Vec<(Vec2, f32)> = samples.iter().copied().filter(|(p, _)| z_of(*p) >= 380.0 && z_of(*p).is_finite()).collect();
            let (near_shift, _) = search(20, |s| ncc(&near, &cgrad, qw, qh, s / 4.0));
            let (far_shift, _) = search(20, |s| ncc(&far, &cgrad, qw, qh, s / 4.0));
            if vi == 0 {
                println!("    IR by depth: < 330 mm ({} cells) shift ({:+.2}, {:+.2}); >= 380 mm ({} cells) shift ({:+.2}, {:+.2})", near.len(), near_shift.x, near_shift.y, far.len(), far_shift.x, far_shift.y);
            }

            // 2. Silhouettes.
            let px: Vec<Vec2> = edges.iter().filter_map(|p| model.project_sensor(*p).map(|(q, _)| q)).collect();
            let edge_score = |s: Vec2| {
                let v: Vec<f32> = px.iter().filter_map(|q| bilinear(&grad, w, h, *q + s)).collect();
                v.iter().sum::<f32>() / v.len().max(1) as f32
            };
            let (edge_shift, _) = search(20, edge_score);

            println!(
                "pair {i:2} (t={}) {name:16} IR: shift ({:+.2}, {:+.2}) px, NCC {:.3} (at 0: {:.3}), {} px | silhouettes: shift ({:+.2}, {:+.2}) px, {} px",
                pair.depth_ts,
                ir_shift.x,
                ir_shift.y,
                ir_peak,
                ir_zero,
                samples.len(),
                edge_shift.x,
                edge_shift.y,
                px.len()
            );
            ir_results[vi].push(ir_shift);
            edge_results[vi].push(edge_shift);

            if vi == 0 && i == 0 {
                if let Some(out) = &out_dir {
                    std::fs::create_dir_all(out).unwrap();
                    let mut img = image::RgbImage::from_raw(w as u32, h as u32, pair.color.image.data.clone()).unwrap();
                    for q in &px {
                        img.put_pixel(q.x as u32, q.y as u32, image::Rgb([255, 0, 0]));
                    }
                    img.save(out.join("calib_overlay.png")).unwrap();
                    let blend = image::RgbImage::from_fn(w as u32, h as u32, |x, y| {
                        let c = &pair.color.image.data[(y as usize * w + x as usize) * 3..][..3];
                        match warped[y as usize * w + x as usize] {
                            Some(v) => image::Rgb([v as u8, c[1], c[1]]),
                            None => image::Rgb([0, c[1] / 2, c[1] / 2]),
                        }
                    });
                    blend.save(out.join("calib_ir_blend.png")).unwrap();
                    // 3x zooms of a region with both kinds of edges (green mug rim in the dev scene).
                    let zoom = |src: &image::RgbImage, name: &str| {
                        let (x0, y0, cw, ch) = (720, 400, 300, 220);
                        let z = image::RgbImage::from_fn(cw * 3, ch * 3, |x, y| *src.get_pixel(x0 + x / 3, y0 + y / 3));
                        z.save(out.join(name)).unwrap();
                    };
                    zoom(&img, "calib_overlay_zoom.png");
                    zoom(&blend, "calib_ir_blend_zoom.png");
                    println!("wrote calib_overlay.png, calib_ir_blend.png to {}", out.display());
                }
            }
        }
    }
    let summary = |label: &str, results: &[Vec<Vec2>]| {
        for (vi, (name, _)) in variants.iter().enumerate() {
            let r = &results[vi];
            let n = r.len().max(1) as f32;
            let mean = r.iter().copied().sum::<Vec2>() / n;
            let rms = (r.iter().map(|s| s.length_squared()).sum::<f32>() / n).sqrt();
            println!("{label} {name:16}: mean shift ({:+.2}, {:+.2}) px, rms |shift| {:.2} px over {} frames", mean.x, mean.y, rms, r.len());
        }
    };
    summary("IR         ", &ir_results);
    summary("silhouettes", &edge_results);
}
