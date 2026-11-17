//! Offline evaluation of Hybrid HD depth on a recording made with `hybrid_capture`:
//! `cargo run --release -p scancraft-driver-revopoint --example hybrid_eval -- <hybrid.bin> <camparam.yaml> [fullfield.bin] [out_dir]`
//!
//! Decodes every packet, turns it into a `SensorFrame` through the Hybrid mode stream
//! (speckle rectification, warm-up drop), computes host depth and reports per frame: coverage,
//! matching time, device frame interval, RMS of the dominant plane (RANSAC, 1 mm) and the median
//! RMS of local 16x16 planes. With a Full-field HD recording of the same static scene, both depth
//! maps (same rectified left camera) are compared pixel by pixel. `out_dir` receives a depth PNG.

use glam::Vec3;
use scancraft_core::{DepthMap, SensorFrame};
use scancraft_driver::ObjectType;
use scancraft_driver_revopoint::media::{FrameDecoder, FrameLayout, PacketParser};
use scancraft_driver_revopoint::streams::{hybrid, FullField, ModeStream};
use scancraft_driver_revopoint::{DeviceCalibration, StereoCalibration};
use scancraft_hybrid::{hybrid_depth, HybridParams};
use std::sync::Arc;
use std::time::Instant;

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let rec = args.get(1).expect("usage: hybrid_eval <hybrid.bin> <camparam.yaml> [fullfield.bin] [out_dir]");
    let yaml = std::fs::read_to_string(args.get(2).expect("camparam.yaml")).unwrap();
    let ff = args.get(3).filter(|s| !s.is_empty() && *s != "-");
    let out_dir = args.get(4);
    let cal = Arc::new(DeviceCalibration::new(StereoCalibration::parse(&yaml).expect("calibration")));
    // HYBRID_SWEEP=1: the full disparity sweep instead of the fast coarse matcher.
    let params = HybridParams { min_mm: 150.0, max_mm: 600.0, fast_coarse: std::env::var_os("HYBRID_SWEEP").is_none(), ..HybridParams::default() };

    let mut mode = hybrid::Hybrid::new(ObjectType::General, cal.clone()).unwrap();
    let frames = load_frames(rec, FrameLayout::StripeSpeckle, &mut mode);
    println!("{} hybrid frames after warm-up", frames.len());
    let mut last_depth = None;
    let mut prev_counter = None;
    let mut times = Vec::new();
    for f in &frames {
        let ir = f.ir.as_ref().unwrap();
        let t = Instant::now();
        let depth = hybrid_depth(&ir.left, ir.right.as_ref().unwrap(), f.fringes.as_ref(), ir.rig.as_ref().unwrap(), &params);
        let ms = t.elapsed().as_secs_f32() * 1000.0;
        times.push(ms);
        let dt = prev_counter.map(|p| f.timestamp_ms as i64 - p).unwrap_or(0);
        prev_counter = Some(f.timestamp_ms as i64);
        let (plane_rms, inliers) = dominant_plane(&depth);
        println!(
            "frame {:3} (dev counter {:8}, +{:4}): coverage {:5.1}%  match {:6.1} ms  plane RMS {:.3} mm over {:6} px  local RMS {:.3} mm",
            f.frame_index,
            f.timestamp_ms,
            dt,
            100.0 * depth.valid_pixel_count() as f32 / depth.data.len() as f32,
            ms,
            plane_rms,
            inliers,
            local_plane_rms(&depth)
        );
        last_depth = Some(depth);
    }
    if !times.is_empty() {
        times.sort_by(f32::total_cmp);
        println!("matching time median {:.1} ms", times[times.len() / 2]);
    }
    let Some(hd) = last_depth else { return };
    if let Some(dir) = out_dir {
        std::fs::create_dir_all(dir).unwrap();
        save_depth_png(&hd, &format!("{dir}/hybrid_depth.png"));
    }

    if let Some(ff) = ff {
        let mut ffmode = FullField::new(ObjectType::General, cal.clone()).unwrap();
        let ff_frames = load_frames(ff, FrameLayout::DepthIrPair, &mut ffmode);
        let Some(fd) = ff_frames.last().and_then(|f| f.depth.clone()) else {
            println!("no full-field depth frames");
            return;
        };
        let (plane_rms, inliers) = dominant_plane(&fd);
        println!(
            "full-field: {} frames, coverage {:.1}%, plane RMS {:.3} mm over {} px, local RMS {:.3} mm",
            ff_frames.len(),
            100.0 * fd.valid_pixel_count() as f32 / fd.data.len() as f32,
            plane_rms,
            inliers,
            local_plane_rms(&fd)
        );
        let mut diffs: Vec<f32> = hd
            .data
            .iter()
            .zip(&fd.data)
            .filter(|(&a, &b)| a > 0 && b > 0)
            .map(|(&a, &b)| a as f32 * hd.scale_mm - b as f32 * fd.scale_mm)
            .collect();
        let both = diffs.len();
        let only_h = hd.data.iter().zip(&fd.data).filter(|(&a, &b)| a > 0 && b == 0).count();
        let only_f = hd.data.iter().zip(&fd.data).filter(|(&a, &b)| a == 0 && b > 0).count();
        diffs.sort_by(f32::total_cmp);
        let pct = |p: f32| diffs[((diffs.len() - 1) as f32 * p) as usize];
        if both > 0 {
            let within = diffs.iter().filter(|d| d.abs() < 1.0).count();
            println!(
                "hybrid - full-field over {both} common px: median {:.3} mm, p5 {:.3}, p95 {:.3}, |diff|<1 mm {:.1}%; only hybrid {only_h} px, only full-field {only_f} px",
                pct(0.5),
                pct(0.05),
                pct(0.95),
                100.0 * within as f32 / both as f32
            );
        }
        if let Some(dir) = out_dir {
            save_depth_png(&fd, &format!("{dir}/fullfield_depth.png"));
        }
    }
}

fn load_frames(path: &str, layout: FrameLayout, mode: &mut dyn ModeStream) -> Vec<SensorFrame> {
    let data = std::fs::read(path).unwrap();
    let spec = mode.spec();
    let mut parser = PacketParser::default();
    parser.push(&data);
    let mut decoder = FrameDecoder::new(layout, spec.width, spec.height);
    let mut out = Vec::new();
    let mut index = 0;
    while let Some(p) = parser.next_packet() {
        match decoder.decode(&p) {
            Ok(d) => {
                index += 1;
                if std::env::var_os("HYBRID_EVAL_VERBOSE").is_some() {
                    let mean = |i: &scancraft_core::RgbImage| i.data.iter().map(|&v| v as f64).sum::<f64>() / i.data.len().max(1) as f64;
                    let planes: Vec<String> = d.extra_planes.iter().map(|p| format!("{:.0}", mean(p))).collect();
                    println!(
                        "packet {index}: {} B, counter {}, speckle L/R mean {:.1}/{:.1}, stripe means [{}]",
                        p.len(),
                        d.device_counter,
                        mean(&d.ir_left),
                        mean(&d.ir_right),
                        planes.join(" ")
                    );
                }
                if let Some(f) = mode.to_sensor_frame(d, index) {
                    out.push(f);
                }
            }
            Err(e) => eprintln!("decode: {e}"),
        }
    }
    out
}

fn points(depth: &DepthMap) -> Vec<(usize, Vec3)> {
    let k = depth.intrinsics.unwrap();
    let w = depth.width as usize;
    depth
        .data
        .iter()
        .enumerate()
        .filter(|(_, &v)| v > 0)
        .map(|(i, &v)| {
            let z = v as f32 * depth.scale_mm;
            let (u, vv) = ((i % w) as f32, (i / w) as f32);
            (i, Vec3::new((u - k.cx) * z / k.fx, (vv - k.cy) * z / k.fy, z))
        })
        .collect()
}

/// RANSAC plane (1 mm) refined by least squares; returns (RMS mm, inlier count).
fn dominant_plane(depth: &DepthMap) -> (f32, usize) {
    let pts: Vec<Vec3> = points(depth).into_iter().map(|p| p.1).collect();
    if pts.len() < 100 {
        return (f32::NAN, 0);
    }
    let mut seed = 12345u64;
    let mut rnd = || {
        seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        (seed >> 33) as usize % pts.len()
    };
    let mut best = (0usize, Vec3::Z, 0.0f32);
    for _ in 0..400 {
        let (a, b, c) = (pts[rnd()], pts[rnd()], pts[rnd()]);
        let n = (b - a).cross(c - a);
        if n.length() < 1e-3 {
            continue;
        }
        let n = n.normalize();
        let d = n.dot(a);
        let count = pts.iter().step_by(4).filter(|p| (n.dot(**p) - d).abs() < 1.0).count();
        if count > best.0 {
            best = (count, n, d);
        }
    }
    let inl: Vec<Vec3> = pts.iter().copied().filter(|p| (best.1.dot(*p) - best.2).abs() < 1.0).collect();
    (fit_rms(&inl), inl.len())
}

fn fit_rms(p: &[Vec3]) -> f32 {
    if p.len() < 3 {
        return f32::NAN;
    }
    let c = p.iter().copied().sum::<Vec3>() / p.len() as f32;
    let mut m = glam::Mat3::ZERO;
    for q in p {
        let d = *q - c;
        m += glam::Mat3::from_cols(d * d.x, d * d.y, d * d.z);
    }
    // Smallest eigenvalue of the scatter matrix by inverse power iteration on (m + eps).
    let inv = (m + glam::Mat3::IDENTITY * 1e-6).inverse();
    let mut n = Vec3::new(0.3, 0.5, 0.8).normalize();
    for _ in 0..50 {
        n = (inv * n).normalize();
    }
    (p.iter().map(|q| (n.dot(*q - c)).powi(2)).sum::<f32>() / p.len() as f32).sqrt()
}

fn local_plane_rms(depth: &DepthMap) -> f32 {
    let w = depth.width as usize;
    let all = points(depth);
    let mut grid: std::collections::HashMap<(usize, usize), Vec<Vec3>> = Default::default();
    for (i, p) in all {
        grid.entry(((i % w) / 16, (i / w) / 16)).or_default().push(p);
    }
    let mut rms: Vec<f32> = grid.values().filter(|v| v.len() > 180).map(|v| fit_rms(v)).filter(|r| r.is_finite()).collect();
    if rms.is_empty() {
        return f32::NAN;
    }
    rms.sort_by(f32::total_cmp);
    rms[rms.len() / 2]
}

fn save_depth_png(depth: &DepthMap, path: &str) {
    let img: Vec<u8> = depth
        .data
        .iter()
        .map(|&v| if v == 0 { 0 } else { (255.0 - ((v as f32 * depth.scale_mm - 150.0) / 450.0 * 254.0).clamp(0.0, 254.0)) as u8 })
        .collect();
    image::GrayImage::from_raw(depth.width, depth.height, img).unwrap().save(path).unwrap();
    println!("wrote {path}");
}
