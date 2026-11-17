//! End-to-end check of one scan mode through exactly what the app uses: the driver's
//! `start_scan` for a `ScanConfig`, then `scancraft_pipeline::pipeline_for` and the tracker.
//! Reports whether frames arrive, the frame rate, what each frame carries, the geometry the
//! mode's depth source produces and whether a static scene tracks.
//!
//! `cargo run --release -p scancraft-driver-revopoint --example mode_check -- <ip> <light> <tracking> <object> [frames] [color]`
//!   light:    fullfield | vcsel | hybrid | cross | single
//!   tracking: feature | marker | global | color
//!   object:   general | dark | face | reflective | outdoor
//!
//! Starts modes still marked NotImplemented too (it sets `SCANCRAFT_EXPERIMENTAL_MODES`). Gives up
//! if no frame arrives for 25 s; always ends through `stop_scan` (never kill it mid-stream).
//! Exit code 0 = frames with geometry arrived, 2 = no frames, 3 = frames without geometry.

use scancraft_driver::{DeviceTransport, LightSource, ObjectType, ScanConfig, ScannerDriver, TrackingMode};
use scancraft_driver_revopoint::RevopointDriver;
use scancraft_slam::{Tracker, TrackerConfig, TrackingState};
use std::time::{Duration, Instant};

#[tokio::main]
async fn main() {
    env_logger::init();
    let a: Vec<String> = std::env::args().collect();
    let arg = |i: usize, d: &str| a.get(i).cloned().unwrap_or_else(|| d.to_string());
    let ip = arg(1, "192.168.178.34");
    let light = match arg(2, "fullfield").as_str() {
        "vcsel" => LightSource::VcselRapid,
        "hybrid" => LightSource::HybridHd,
        "cross" => LightSource::CrossLines,
        "single" => LightSource::SingleLine,
        _ => LightSource::FullFieldHd,
    };
    let tracking = match arg(3, "feature").as_str() {
        "marker" => TrackingMode::Marker,
        "global" => TrackingMode::GlobalMarker,
        "color" => TrackingMode::Color,
        _ => TrackingMode::Feature,
    };
    let object = match arg(4, "general").as_str() {
        "dark" => ObjectType::Dark,
        "face" => ObjectType::Face,
        "reflective" => ObjectType::Reflective,
        "outdoor" => ObjectType::Outdoor,
        _ => ObjectType::General,
    };
    let frames: usize = arg(5, "40").parse().unwrap_or(40);
    let color = a.get(6).is_some_and(|s| s == "color");
    // MODE_CHECK_VERBOSE=1: one line per frame (tracking state, points, marker overlay counts).
    let verbose = std::env::var_os("MODE_CHECK_VERBOSE").is_some();
    std::env::set_var("SCANCRAFT_EXPERIMENTAL_MODES", "1");

    let config = ScanConfig { light, tracking, object, color_texture: color, ..ScanConfig::default() };
    let label = format!("{} + {} + {}{}", light.label(), tracking.label(), object.label(), if color { " + colour" } else { "" });
    let transport = DeviceTransport::WiFi { ip, control_port: 80, data_port: 8871, imu_port: 9999 };
    let mut dev = RevopointDriver.connect(&transport).await.expect("connect");
    let support = dev.capabilities().modes.clone();
    println!(
        "{label}: valid combination {}, driver says {}",
        support.is_valid_combination(light, tracking, object),
        if support.can_scan(light, tracking, object) { "available" } else { "not available" }
    );
    let mut pipeline = match scancraft_pipeline::pipeline_for(&config, 150.0, 600.0) {
        Ok(p) => Some(p),
        Err(e) => {
            println!("no pipeline: {e}");
            None
        }
    };
    let imu_mount = dev.info().imu_to_sensor.map(glam::Quat::from_array);
    let mut tracker = Tracker::with_config(TrackerConfig { imu_to_sensor: imu_mount, ..TrackerConfig::default() });

    let t0 = Instant::now();
    let mut rx = match dev.start_scan(&config).await {
        Ok(rx) => rx,
        Err(e) => {
            println!("start_scan failed: {e}");
            dev.disconnect().await.ok();
            std::process::exit(2);
        }
    };
    let started = t0.elapsed().as_secs_f32();
    let (mut n, mut with_geometry, mut tracked, mut lost) = (0usize, 0usize, 0usize, 0usize);
    let (mut points, mut coverage, mut ms) = (0usize, 0.0f32, 0.0f32);
    let (mut has_depth, mut has_ir, mut has_fringes, mut has_color, mut has_imu) = (0, 0, 0, 0, 0);
    let mut first = None;
    let mut last_arrival = Instant::now();
    let (mut t_first, mut t_last) = (Instant::now(), Instant::now());
    while n < frames {
        let Ok(Some(frame)) = tokio::time::timeout(Duration::from_secs(25), rx.recv()).await else {
            println!("no frame for 25 s after {n} frames");
            break;
        };
        if first.is_none() {
            first = Some(t0.elapsed().as_secs_f32());
            t_first = Instant::now();
        }
        t_last = Instant::now();
        let _ = last_arrival.elapsed();
        last_arrival = Instant::now();
        n += 1;
        has_depth += frame.depth.is_some() as usize;
        has_ir += frame.ir.is_some() as usize;
        has_fringes += frame.fringes.is_some() as usize;
        has_color += frame.color.is_some() as usize;
        has_imu += frame.imu.is_some() as usize;
        let Some(p) = pipeline.as_mut() else { continue };
        let t = Instant::now();
        // Geometry first (independent of tracking), then the full pipeline step.
        let geometry = p.depth.process(&frame);
        if let Some(g) = &geometry {
            with_geometry += 1;
            points += g.cloud.len();
            if let Some(d) = &g.depth {
                coverage += d.valid_pixel_count() as f32 / d.data.len().max(1) as f32;
            }
        }
        if let Some(status) = p.process(&frame, &mut tracker) {
            match status.state {
                TrackingState::Lost => lost += 1,
                _ => tracked += 1,
            }
            if verbose {
                use scancraft_pipeline::OverlayKind;
                let o = p.overlay();
                let count = |k: OverlayKind| o.as_ref().map_or(0, |o| o.count(k));
                println!(
                    "  frame {n:3}: {:?} inliers {:5} rmse {:.2} | points {:5} | markers seen {:2} in 3D {:2} tracking {:2}{}",
                    status.state,
                    status.inliers,
                    status.rmse_mm,
                    geometry.as_ref().map_or(0, |g| g.cloud.len()),
                    count(OverlayKind::Detected),
                    count(OverlayKind::Triangulated),
                    count(OverlayKind::Tracking),
                    if o.as_ref().is_some_and(|o| o.stale) { " (stale)" } else { "" }
                );
            }
        } else if verbose {
            println!("  frame {n:3}: no status (points {})", geometry.as_ref().map_or(0, |g| g.cloud.len()));
        }
        ms += t.elapsed().as_secs_f32() * 1000.0;
    }
    let span = t_last.duration_since(t_first).as_secs_f32();
    dev.stop_scan().await.ok();
    dev.disconnect().await.ok();

    let per = |v: usize| if n > 0 { format!("{v}/{n}") } else { "-".into() };
    println!(
        "{label}: {n} frames, stream setup {started:.1} s, first frame after {:.1} s, {:.1} fps delivered",
        first.unwrap_or(f32::NAN),
        if n > 1 && span > 0.0 { (n - 1) as f32 / span } else { 0.0 }
    );
    println!(
        "  carries: depth {}, IR {}, fringes {}, colour {}, IMU {}",
        per(has_depth),
        per(has_ir),
        per(has_fringes),
        per(has_color),
        per(has_imu)
    );
    if let Some(p) = &pipeline {
        let g = with_geometry.max(1) as f32;
        println!(
            "  {}: geometry in {} frames, {:.0} points/frame, depth coverage {:.1}%, {:.0} ms/frame (geometry + tracking)",
            p.depth.name(),
            per(with_geometry),
            points as f32 / g,
            100.0 * coverage / g,
            ms / n.max(1) as f32
        );
        println!("  tracking (static scene): {tracked} tracked, {lost} lost, {} keyframes", tracker.keyframes().len());
    }
    std::process::exit(if n == 0 {
        2
    } else if pipeline.is_some() && with_geometry == 0 {
        3
    } else {
        0
    });
}
