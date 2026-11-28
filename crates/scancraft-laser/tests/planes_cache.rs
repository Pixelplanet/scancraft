//! The per-scanner laser plane cache lives outside any project: a replay must be able to process
//! a recording without it (`LaserLineDepth::without_cache`, `with_planes`), and a live run must
//! say where its planes came from so the replay can do the same.
//!
//! One test in this file: it sets `LOCALAPPDATA`, which is process-wide.

use scancraft_core::frame::*;
use scancraft_laser::planes::LaserPlaneSet;
use scancraft_laser::{planes_cache_file, LaserLineDepth, Pattern, PlanesOrigin};
use std::sync::Arc;

fn rig() -> StereoRig {
    StereoRig {
        width: 640,
        height: 512,
        intrinsics: PinholeIntrinsics { fx: 600.0, fy: 600.0, cx: 320.0, cy: 256.0 },
        baseline_mm: 50.0,
        disparity_offset_px: 0.0,
        calibration_set: "blue".into(),
        rectification: None,
    }
}

fn frame(i: u64, rig: &Arc<StereoRig>) -> SensorFrame {
    let img = || RgbImage { width: 640, height: 512, channels: 1, data: vec![0; 640 * 512] };
    let mut f = SensorFrame::new(i, i * 100);
    f.ir = Some(IrImages { left: img(), right: Some(img()), rectified: true, rig: Some(rig.clone()) });
    f
}

#[test]
fn a_replay_does_not_depend_on_the_plane_cache_of_the_computer() {
    let root = std::env::temp_dir().join(format!("scancraft-laser-cache-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&root);
    std::env::remove_var(scancraft_laser::PLANES_ENV);
    std::env::set_var("LOCALAPPDATA", &root);
    let rig = Arc::new(rig());
    let cached = LaserPlaneSet::default();
    let path = planes_cache_file(Pattern::Cross, &rig).unwrap();
    std::fs::create_dir_all(path.parent().unwrap()).unwrap();
    std::fs::write(&path, serde_json::to_string(&cached).unwrap()).unwrap();

    // Live: the cache is read at the first frame, and the depth source says so.
    let mut live = LaserLineDepth::new(Pattern::Cross);
    assert_eq!(live.origin(), &PlanesOrigin::Unknown);
    live.process(&frame(1, &rig)).unwrap();
    assert_eq!(live.origin(), &PlanesOrigin::Cache(path.clone()));
    assert_eq!(live.start_planes(), Some(&cached));

    // Replay of a session that calibrated online: the cache on this computer is not used.
    let mut replay = LaserLineDepth::without_cache(Pattern::Cross);
    replay.process(&frame(1, &rig)).unwrap();
    assert!(replay.planes().is_none(), "the cache of this computer was read");
    assert_eq!(replay.origin(), &PlanesOrigin::Online);

    // Replay of a session that started with planes: exactly those, whatever the cache holds.
    std::fs::remove_file(&path).unwrap();
    let mut given = LaserLineDepth::with_planes(Pattern::Cross, cached.clone());
    given.process(&frame(1, &rig)).unwrap();
    assert_eq!((given.planes(), given.origin()), (Some(&cached), &PlanesOrigin::Given));
    assert!(!path.exists(), "a replay writes nothing into the cache");
    let _ = std::fs::remove_dir_all(&root);
}
