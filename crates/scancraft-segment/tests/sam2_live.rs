//! Integration tests requiring local SAM 2.1 ONNX models and ONNX Runtime DLLs.
//!
//! Run with:
//! ```text
//! SCANCRAFT_SAM2_DIR=%LOCALAPPDATA%\ScanCraft\ai\models\sam21-tiny-video-onnx \
//! SCANCRAFT_ORT_DIR=%LOCALAPPDATA%\ScanCraft\ai\runtime\ort-1.24.4-dml \
//! cargo test -p scancraft-segment --release --test sam2_live -- --ignored --nocapture
//! ```

use scancraft_core::RgbImage;
use scancraft_segment::{
    ExecutionPreference, FramePrompt, ModelFiles, OrtRuntime, PostprocessConfig, RuntimeFiles, Sam2VideoSegmenter,
};
use std::path::PathBuf;

fn synthetic_moving_frame(cx: usize, cy: usize) -> RgbImage {
    let (w, h) = (256usize, 256usize);
    let mut data = vec![20u8; w * h * 3];
    for y in 0..h {
        for x in 0..w {
            let i = (y * w + x) * 3;
            let dx = x as isize - cx as isize;
            let dy = y as isize - cy as isize;
            let dist2 = (dx * dx + dy * dy) as f32;
            if dist2 <= 34.0 * 34.0 {
                let shade = 1.0 - (dist2.sqrt() / 42.0) * 0.25;
                data[i] = (230.0 * shade) as u8;
                data[i + 1] = (165.0 * shade) as u8;
                data[i + 2] = (55.0 * shade) as u8;
            } else {
                data[i] = 32;
                data[i + 1] = 44;
                data[i + 2] = 68;
            }
        }
    }
    RgbImage { width: w as u32, height: h as u32, channels: 3, data }
}

#[test]
#[ignore]
fn cpu_and_directml_self_test_and_tracking() {
    let model_dir = PathBuf::from(
        std::env::var("SCANCRAFT_SAM2_DIR").expect("SCANCRAFT_SAM2_DIR must be set for ignored tests"),
    );
    let rt_dir = std::env::var("SCANCRAFT_ORT_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|_| {
            PathBuf::from(std::env::var("LOCALAPPDATA").unwrap())
                .join("ScanCraft/ai/runtime/ort-1.24.4-dml")
        });

    let models = ModelFiles::sam21_tiny_from_dir(&model_dir);
    let rt_files = RuntimeFiles::ort_1244_dml_from_dir(&rt_dir);
    let runtime = OrtRuntime::init(&rt_files).expect("OrtRuntime::init");

    let (mut seg, report) = Sam2VideoSegmenter::load(&runtime, &models, ExecutionPreference::Auto, PostprocessConfig::default())
        .expect("Sam2VideoSegmenter::load");
    println!("Chosen provider: {}", seg.provider());
    if let Some(rep) = &report {
        println!("Self-test: {rep:?}");
        assert!(rep.passed, "self-test failed: {}", rep.message);
    }

    let f0 = synthetic_moving_frame(96, 128);
    let f1 = synthetic_moving_frame(118, 128);
    let f2 = synthetic_moving_frame(140, 128);

    let r0 = seg.add_prompt(0, &f0, &FramePrompt::default().with_point(96.0, 128.0, true)).unwrap();
    let r1 = seg.propagate(1, &f1).unwrap();
    // Re-prompt on frame 2 to exercise mid-sequence re-prompting.
    let r2 = seg.add_prompt(2, &f2, &FramePrompt::default().with_point(140.0, 128.0, true)).unwrap();

    for r in [&r0, &r1, &r2] {
        println!("frame {}: fg={} iou={:.3} stab={:.3} status={:?}", r.frame_index, r.mask.foreground_count(), r.iou_score, r.stability_score, r.status);
        assert!(!r.status.is_lost(), "frame {} unexpectedly lost: {:?}", r.frame_index, r.status);
        assert!(r.mask.foreground_count() > 1000 && r.mask.foreground_count() < 6000);
    }
    assert!(r0.mask.is_foreground(96, 128));
    assert!(r1.mask.is_foreground(118, 128));
    assert!(r2.mask.is_foreground(140, 128));
    assert!(!r1.mask.is_foreground(10, 10));
}

#[test]
#[ignore]
fn load_fastest_variant_selects_fast_512_variant() {
    use scancraft_segment::ModelVariant;

    let local = PathBuf::from(std::env::var("LOCALAPPDATA").unwrap()).join("ScanCraft/ai");
    let rt_dir = std::env::var("SCANCRAFT_ORT_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|_| local.join("runtime/ort-1.24.4-dml"));
    let rt_files = RuntimeFiles::ort_1244_dml_from_dir(&rt_dir);
    let runtime = OrtRuntime::init(&rt_files).expect("OrtRuntime::init");

    let mut candidates = Vec::new();
    for &variant in ModelVariant::all() {
        let dir = [
            local.join("models").join(variant.dir_name()),
            local.join("exports").join(variant.dir_name()),
        ]
        .into_iter()
        .find(|d| d.join("constants.json").is_file());
        if let Some(d) = dir {
            let files = ModelFiles::for_variant(variant, &d);
            if files.verify_all().is_ok() {
                candidates.push((variant, files));
            }
        }
    }
    assert!(!candidates.is_empty(), "expected at least one verified variant under ScanCraft/ai");
    let (mut seg, report) =
        Sam2VideoSegmenter::load_fastest_variant(&runtime, &candidates, ExecutionPreference::Auto, PostprocessConfig::default())
            .expect("load_fastest_variant");
    println!("Selected variant: {} on {} ({:.1} ms/frame)", report.chosen.display_name(), report.provider, report.mean_tracked_ms);
    for b in &report.benchmarks {
        println!("  - {:24} ok={} mean={:?} note={}", b.variant.dir_name(), b.self_test_passed, b.mean_tracked_ms, b.note);
    }
    assert!(report.chosen.is_live_fast(), "expected a fast 512px variant to beat 1024px baseline, got {:?}", report.chosen);

    let f0 = synthetic_moving_frame(96, 128);
    let f1 = synthetic_moving_frame(118, 128);
    let r0 = seg.add_prompt(0, &f0, &FramePrompt::default().with_point(96.0, 128.0, true)).unwrap();
    let r1 = seg.propagate(1, &f1).unwrap();
    assert!(r0.mask.is_foreground(96, 128));
    assert!(r1.mask.is_foreground(118, 128));
}
