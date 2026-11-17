//! Offline evaluation of host stereo on a recorded VCSEL Rapid session, optionally against a
//! Full-field HD recording of the same (static) scene:
//! `cargo run --release -p scancraft-driver-revopoint --example vcsel_eval -- <camparam.yaml> <vcsel.bin> [fullfield.bin] [out_dir]`
//!
//! Per VCSEL frame: matcher time, depth coverage, depth range. For the last frame: largest plane
//! (RANSAC + least-squares refit) RMS, a PLY of the cloud and a false-colour disparity PNG. With a
//! Full-field recording: the same statistics for the device depth and the per-pixel difference
//! between the two depth maps (both live in the rectified left IR camera).
//!
//! With `SRC=depthir` the first capture is a fmt 3 / type 4 recording (device z16 + IR pair, as
//! with the VCSEL on): stereo runs on its IR pair and is compared with the same frame's device
//! depth, which removes any scene or timing difference from the comparison.
//!
//! With `SRC=jpegraw` the capture is the raw (unrectified) JPEG pair of `camera_id=33&type_id=20`
//! (e.g. Revo's own VCSEL stream cut from a packet capture); both images are rectified with the
//! red calibration first. `SKIP` drops that many leading frames (Revo's stream starts dark).
//!
//! Env: `SGM_P1`, `SGM_P2`, `SGM_UNIQ`, `SGM_TEX`, `SGM_LR`, `SGM_SPECKLE`, `SGM_HOLE`, `SGM_DIAG=0`, `MIN_MM`, `MAX_MM`, `FRAMES`, `SRC`.

use glam::Vec3;
use scancraft_core::{DepthMap, IrImages};
use scancraft_driver_revopoint::{FrameDecoder, FrameLayout, PacketParser, StereoCalibration};
use scancraft_stereo::{StereoDepth, StereoDepthConfig};
use std::io::Write;
use std::path::PathBuf;
use std::time::Instant;

fn env<T: std::str::FromStr>(k: &str, default: T) -> T {
    std::env::var(k).ok().and_then(|v| v.parse().ok()).unwrap_or(default)
}

fn main() {
    env_logger::init();
    let args: Vec<String> = std::env::args().collect();
    let yaml = std::fs::read_to_string(&args[1]).expect("camparam.yaml");
    let cal = StereoCalibration::parse(&yaml).expect("calibration");
    let calibration = scancraft_driver_revopoint::DeviceCalibration::new(cal);
    let rig = calibration.red_rig(640, 512);
    println!("rig: fx {:.2} cx {:.2} cy {:.2}, baseline {:.2} mm, disparity offset (cx_right - cx_left) {:.3} px", rig.intrinsics.fx, rig.intrinsics.cx, rig.intrinsics.cy, rig.baseline_mm, rig.disparity_offset_px);
    let out = PathBuf::from(args.get(4).map(String::as_str).unwrap_or("vcsel_eval_out"));
    std::fs::create_dir_all(&out).unwrap();
    let (min_mm, max_mm) = (env("MIN_MM", 300.0f32), env("MAX_MM", 800.0f32));

    let mut config = StereoDepthConfig::new(min_mm, max_mm);
    config.sgm.p1 = env("SGM_P1", config.sgm.p1);
    config.sgm.p2 = env("SGM_P2", config.sgm.p2);
    config.sgm.uniqueness = env("SGM_UNIQ", config.sgm.uniqueness);
    config.sgm.diagonal_paths = env("SGM_DIAG", 1u8) != 0;
    config.sgm.min_texture = env("SGM_TEX", config.sgm.min_texture);
    config.sgm.lr_max_diff = env("SGM_LR", config.sgm.lr_max_diff);
    config.sgm.speckle_window = env("SGM_SPECKLE", config.sgm.speckle_window);
    config.sgm.max_hole = env("SGM_HOLE", config.sgm.max_hole);
    // SGM_C2F=0: full-resolution SGM instead of the coarse-to-fine matcher.
    config.coarse_to_fine = env("SGM_C2F", 1u8) != 0;
    println!("sgm: {:?}", config.sgm);
    let mut stereo = StereoDepth::new(config);
    let window = scancraft_stereo::disparity_window(&rig, min_mm, max_mm);
    println!("disparity window: {} .. {} ({} disparities)", window.0, window.0 + window.1 as i32 - 1, window.1);

    let max_frames: usize = env("FRAMES", 40usize);
    let depth_ir = std::env::var("SRC").is_ok_and(|v| v == "depthir");
    let jpeg_raw = std::env::var("SRC").is_ok_and(|v| v == "jpegraw");
    // SRC=slots: the driver's VCSEL recording (640x1024 pair, four raw slots; lit slots used).
    let slots = std::env::var("SRC").is_ok_and(|v| v == "slots");
    let layout = if depth_ir {
        FrameLayout::DepthIrPair
    } else if jpeg_raw {
        FrameLayout::JpegIrPair
    } else if slots {
        FrameLayout::IrSlotPairs
    } else {
        FrameLayout::IrPair
    };
    let maps = (jpeg_raw || slots).then(|| calibration.red_rectify_maps(640, 512).expect("raw calibration for rectification"));
    let mut last: Option<(DepthMap, IrImages)> = None;
    let mut reference: Option<DepthMap> = None;
    let mut times = Vec::new();
    let source = frames(&args[2], layout).filter(|f| !depth_ir || f.depth.as_ref().is_some_and(|d| d.valid_pixel_count() > 0));
    for (i, frame) in source.skip(env("SKIP", 0usize)).take(max_frames).enumerate() {
        reference = frame.depth.map(|d| d.with_intrinsics(rig.intrinsics));
        let (left, right) = match &maps {
            Some(m) => (m.rectify(&frame.ir_left, true), m.rectify(&frame.ir_right, false)),
            None => (frame.ir_left, frame.ir_right),
        };
        let ir = IrImages { left, right: Some(right), rectified: true, rig: Some(rig.clone()) };
        let t = Instant::now();
        let depth = stereo.compute(&ir).expect("depth");
        let ms = t.elapsed().as_secs_f64() * 1e3;
        times.push(ms);
        let (cov, lo, hi) = stats(&depth);
        println!("vcsel frame {i:3}: {ms:6.1} ms, coverage {:5.1}%, depth {lo:.0}..{hi:.0} mm", cov * 100.0);
        last = Some((depth, ir));
    }
    times.sort_by(|a, b| a.partial_cmp(b).unwrap());
    if !times.is_empty() {
        let med = times[times.len() / 2];
        println!("matcher: median {med:.1} ms ({:.1} fps), max {:.1} ms", 1000.0 / med, times.last().unwrap());
    }
    let (vdepth, ir) = last.expect("no VCSEL frames");
    report("VCSEL", &vdepth, min_mm, max_mm, &out.join("vcsel.ply"));
    image::GrayImage::from_raw(640, 512, ir.left.data.clone()).unwrap().save(out.join("vcsel_left.png")).unwrap();
    image::GrayImage::from_raw(640, 512, ir.right.as_ref().unwrap().data.clone()).unwrap().save(out.join("vcsel_right.png")).unwrap();
    save_depth_png(&vdepth, min_mm, max_mm, &out.join("vcsel_depth.png"));

    let ff_depth = match args.get(3).filter(|s| s.as_str() != "-") {
        Some(ff) => frames(ff, FrameLayout::DepthIrPair)
            .filter(|f| f.depth.as_ref().is_some_and(|d| d.valid_pixel_count() > 0))
            .nth(5)
            .and_then(|f| f.depth)
            .map(|d| d.with_intrinsics(rig.intrinsics)),
        None => reference,
    };
    if let Some(fdepth) = ff_depth {
        report("Device depth", &fdepth, min_mm, max_mm, &out.join("fullfield.ply"));
        save_depth_png(&fdepth, min_mm, max_mm, &out.join("fullfield_depth.png"));
        // Per-pixel comparison where both have depth.
        let mut diffs: Vec<f32> = Vec::new();
        for v in 0..512u32 {
            for u in 0..640u32 {
                if let (Some(a), Some(b)) = (vdepth.get_depth_mm(u, v), fdepth.get_depth_mm(u, v)) {
                    diffs.push(a - b);
                }
            }
        }
        if !diffs.is_empty() {
            diffs.sort_by(|a, b| a.partial_cmp(b).unwrap());
            let med = diffs[diffs.len() / 2];
            let mut abs_dev: Vec<f32> = diffs.iter().map(|d| (d - med).abs()).collect();
            abs_dev.sort_by(|a, b| a.partial_cmp(b).unwrap());
            let within = |t: f32| diffs.iter().filter(|d| d.abs() < t).count() as f32 / diffs.len() as f32 * 100.0;
            println!(
                "stereo - device depth on {} common pixels: median {med:+.2} mm, MAD {:.2} mm, |diff| < 1 mm {:.0}%, < 3 mm {:.0}%",
                diffs.len(),
                abs_dev[abs_dev.len() / 2],
                within(1.0),
                within(3.0)
            );
        }
    }
}

fn frames(path: &str, layout: FrameLayout) -> impl Iterator<Item = scancraft_driver_revopoint::DecodedFrame> {
    let capture = std::fs::read(path).expect("capture");
    let mut parser = PacketParser::default();
    parser.push(&capture);
    let mut decoder = FrameDecoder::new(layout, 640, 512);
    std::iter::from_fn(move || loop {
        let p = parser.next_packet()?;
        if let Ok(f) = decoder.decode(&p) {
            return Some(f);
        }
    })
}

fn stats(d: &DepthMap) -> (f32, f32, f32) {
    let vals: Vec<f32> = d.data.iter().filter(|&&v| v != 0).map(|&v| v as f32 * d.scale_mm).collect();
    let lo = vals.iter().copied().fold(f32::MAX, f32::min);
    let hi = vals.iter().copied().fold(f32::MIN, f32::max);
    (vals.len() as f32 / d.data.len() as f32, lo, hi)
}

fn report(name: &str, depth: &DepthMap, min_mm: f32, max_mm: f32, ply: &std::path::Path) {
    let cloud = depth.to_point_cloud(min_mm, max_mm);
    let pts: Vec<Vec3> = cloud.points.iter().map(|p| Vec3::from(p.position)).collect();
    let (cov, lo, hi) = stats(depth);
    println!("{name}: coverage {:.1}%, {} points, depth {lo:.0}..{hi:.0} mm", cov * 100.0, pts.len());
    if let Some(fit) = plane_fit(&pts) {
        println!(
            "{name}: largest plane normal {:.3}, {} inliers ({:.0}% of points), RMS {:.3} mm (inliers within 3 mm), plane at {:.0} mm",
            fit.normal,
            fit.inliers,
            100.0 * fit.inliers as f32 / pts.len() as f32,
            fit.rms,
            fit.distance
        );
    }
    let mut f = std::io::BufWriter::new(std::fs::File::create(ply).unwrap());
    writeln!(f, "ply\nformat ascii 1.0\nelement vertex {}\nproperty float x\nproperty float y\nproperty float z\nend_header", pts.len()).unwrap();
    for p in &pts {
        writeln!(f, "{} {} {}", p.x, p.y, p.z).unwrap();
    }
}

fn save_depth_png(depth: &DepthMap, min_mm: f32, max_mm: f32, path: &std::path::Path) {
    let img: Vec<u8> = depth
        .data
        .iter()
        .map(|&v| if v == 0 { 0 } else { (255.0 - 254.0 * ((v as f32 * depth.scale_mm - min_mm) / (max_mm - min_mm)).clamp(0.0, 1.0)) as u8 })
        .collect();
    image::GrayImage::from_raw(depth.width, depth.height, img).unwrap().save(path).unwrap();
}

struct PlaneFit {
    normal: Vec3,
    distance: f32,
    inliers: usize,
    rms: f32,
}

/// RANSAC (2 mm) for the dominant plane, least-squares refit on the points within 3 mm,
/// RMS of the refit residuals of those points.
fn plane_fit(pts: &[Vec3]) -> Option<PlaneFit> {
    if pts.len() < 100 {
        return None;
    }
    let mut seed = 0x2545F491u32;
    let mut rand = |n: usize| {
        seed ^= seed << 13;
        seed ^= seed >> 17;
        seed ^= seed << 5;
        seed as usize % n
    };
    let sample: Vec<Vec3> = (0..4000.min(pts.len())).map(|_| pts[rand(pts.len())]).collect();
    let mut best: Option<(Vec3, f32, usize)> = None;
    for _ in 0..500 {
        let (a, b, c) = (sample[rand(sample.len())], sample[rand(sample.len())], sample[rand(sample.len())]);
        let n = (b - a).cross(c - a).normalize_or_zero();
        if n == Vec3::ZERO {
            continue;
        }
        let d = n.dot(a);
        let count = sample.iter().filter(|p| (n.dot(**p) - d).abs() < 2.0).count();
        if best.is_none_or(|b| count > b.2) {
            best = Some((n, d, count));
        }
    }
    let (mut n, mut d, _) = best?;
    let mut inl: Vec<Vec3> = Vec::new();
    for _ in 0..3 {
        inl = pts.iter().copied().filter(|p| (n.dot(*p) - d).abs() < 3.0).collect();
        if inl.len() < 3 {
            return None;
        }
        let c = inl.iter().copied().sum::<Vec3>() / inl.len() as f32;
        let mut cov = glam::Mat3::ZERO;
        for p in &inl {
            let q = *p - c;
            cov += glam::Mat3::from_cols(q * q.x, q * q.y, q * q.z);
        }
        n = smallest_eigenvector(cov);
        d = n.dot(c);
    }
    let rms = (inl.iter().map(|p| (n.dot(*p) - d).powi(2)).sum::<f32>() / inl.len() as f32).sqrt();
    Some(PlaneFit { normal: n, distance: d.abs(), inliers: inl.len(), rms })
}

/// Eigenvector of the smallest eigenvalue of a symmetric 3x3 matrix (inverse power iteration).
fn smallest_eigenvector(m: glam::Mat3) -> Vec3 {
    let shift = m + glam::Mat3::IDENTITY * (m.x_axis.x + m.y_axis.y + m.z_axis.z) * 1e-6;
    let inv = shift.inverse();
    let mut v = Vec3::new(0.3, 0.5, 0.8).normalize();
    for _ in 0..50 {
        v = (inv * v).normalize_or_zero();
    }
    v
}
