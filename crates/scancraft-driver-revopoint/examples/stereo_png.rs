//! Host stereo on two raw (unrectified) 640x512 IR images saved as PNG, e.g. the lit slots from
//! `pair_slots`: rectifies both with the red calibration and reports coverage and plane RMS.
//! `cargo run --release -p scancraft-driver-revopoint --example stereo_png -- <camparam.yaml> <left.png> <right.png> [min_mm] [max_mm] [out_depth.png]`

use scancraft_core::{IrImages, RgbImage};
use scancraft_driver_revopoint::{DeviceCalibration, StereoCalibration};
use scancraft_stereo::{StereoDepth, StereoDepthConfig};

fn load(path: &str) -> RgbImage {
    let img = image::open(path).expect("png").to_luma8();
    RgbImage { width: img.width(), height: img.height(), channels: 1, data: img.into_raw() }
}

fn main() {
    let a: Vec<String> = std::env::args().collect();
    let cal = DeviceCalibration::new(StereoCalibration::parse(&std::fs::read_to_string(&a[1]).unwrap()).unwrap());
    let (min_mm, max_mm) = (a.get(4).and_then(|s| s.parse().ok()).unwrap_or(200.0f32), a.get(5).and_then(|s| s.parse().ok()).unwrap_or(600.0f32));
    let maps = cal.red_rectify_maps(640, 512).expect("raw calibration");
    let rig = cal.red_rig(640, 512);
    // RECTIFIED=1: the inputs are already rectified (skip the remap).
    let (left, right) = if std::env::var_os("RECTIFIED").is_some() {
        (load(&a[2]), load(&a[3]))
    } else {
        (maps.rectify(&load(&a[2]), true), maps.rectify(&load(&a[3]), false))
    };
    let ir = IrImages { left, right: Some(right), rectified: true, rig: Some(rig) };
    let mut stereo = StereoDepth::new(StereoDepthConfig::new(min_mm, max_mm));
    let depth = stereo.compute(&ir).expect("depth");
    let valid: Vec<f32> = depth.data.iter().filter(|&&v| v != 0).map(|&v| v as f32 * depth.scale_mm).collect();
    let cov = valid.len() as f32 / depth.data.len() as f32;
    let (lo, hi) = valid.iter().fold((f32::MAX, f32::MIN), |(l, h), &v| (l.min(v), h.max(v)));
    println!("coverage {:.1}% ({} px), depth {lo:.0}..{hi:.0} mm", cov * 100.0, valid.len());
    if let Some(out) = a.get(6) {
        let px: Vec<u8> = depth
            .data
            .iter()
            .map(|&v| if v == 0 { 0 } else { (255.0 - ((v as f32 * depth.scale_mm - min_mm) / (max_mm - min_mm) * 230.0).clamp(0.0, 230.0)) as u8 })
            .collect();
        image::GrayImage::from_raw(640, 512, px).unwrap().save(out).unwrap();
        let l = ir.left.data.clone();
        image::GrayImage::from_raw(640, 512, l).unwrap().save(std::path::Path::new(out).with_file_name("rect_left.png")).unwrap();
        image::GrayImage::from_raw(640, 512, ir.right.unwrap().data).unwrap().save(std::path::Path::new(out).with_file_name("rect_right.png")).unwrap();
    }
}
