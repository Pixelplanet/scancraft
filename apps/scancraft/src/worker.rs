//! Background workers: turn raw scan frames into camera previews, a distance reading, a tracked
//! pose and world-space points; and turn preview frames into previews and a distance reading.

use eframe::egui::ColorImage;
use scancraft_core::{ColorCamera, DepthMap, PreviewFrame, RgbImage, SensorFrame};
use scancraft_segment::{Mask2D, PostprocessConfig};
use std::sync::{Arc, Mutex};
use std::sync::atomic::{AtomicBool, Ordering};

use scancraft_slam::{Tracker, TrackerConfig};
use tokio::sync::mpsc;

use crate::ai_pack::SharedAiEngine;
use crate::ai_segment::build_overlay_image;
use crate::model::{FrameRates, KeyframeInfo, ScanUpdate, SharedTracker};

/// Shared control state for optional non-blocking live SAM 2.1 segmentation during capture.
pub struct LiveAiState {
    pub enabled: AtomicBool,
    pub engine: Arc<SharedAiEngine>,
    pending_prompt: Mutex<Option<(f32, f32)>>,
    reset_requested: AtomicBool,
}

impl LiveAiState {
    pub fn new(engine: Arc<SharedAiEngine>) -> Self {
        Self {
            enabled: AtomicBool::new(false),
            engine,
            pending_prompt: Mutex::new(None),
            reset_requested: AtomicBool::new(false),
        }
    }

    pub fn set_enabled(&self, on: bool) {
        let prev = self.enabled.swap(on, Ordering::SeqCst);
        if !on || !prev {
            self.reset();
        }
    }

    pub fn set_prompt_normalized(&self, nx: f32, ny: f32) {
        if let Ok(mut g) = self.pending_prompt.lock() {
            *g = Some((nx.clamp(0.0, 1.0), ny.clamp(0.0, 1.0)));
        }
    }

    pub fn reset(&self) {
        self.reset_requested.store(true, Ordering::SeqCst);
        if let Ok(mut g) = self.pending_prompt.lock() {
            *g = None;
        }
    }
}

struct LiveAiJobInput {
    frame_index: u64,
    rgb: RgbImage,
    prompt_norm: Option<(f32, f32)>,
    reset: bool,
}

struct LiveAiJobOutput {
    mask: Mask2D,
    tracking: bool,
    iou: f32,
    provider: String,
    overlay: ColorImage,
}

fn spawn_live_ai_helper(
    engine: Arc<SharedAiEngine>,
) -> (std::sync::mpsc::SyncSender<LiveAiJobInput>, std::sync::mpsc::Receiver<LiveAiJobOutput>) {
    let (in_tx, in_rx) = std::sync::mpsc::sync_channel::<LiveAiJobInput>(1);
    let (out_tx, out_rx) = std::sync::mpsc::sync_channel::<LiveAiJobOutput>(1);
    std::thread::spawn(move || {
        let mut active = false;
        while let Ok(job) = in_rx.recv() {
            if job.reset {
                active = false;
                let _ = engine.with_live_segmenter(|seg, _| {
                    seg.reset();
                    Ok(())
                });
            }
            if job.prompt_norm.is_none() && !active {
                continue;
            }
            let res = engine.with_live_segmenter(|seg, provider| {
                let fm = if let Some((nx, ny)) = job.prompt_norm {
                    seg.reset();
                    active = true;
                    let prompt = scancraft_segment::FramePrompt::default().with_point(
                        nx * (job.rgb.width.saturating_sub(1) as f32),
                        ny * (job.rgb.height.saturating_sub(1) as f32),
                        true,
                    );
                    seg.add_prompt(job.frame_index, &job.rgb, &prompt)
                        .map_err(|e| format!("Live AI prompt: {e}"))?
                } else {
                    seg.propagate(job.frame_index, &job.rgb)
                        .map_err(|e| format!("Live AI propagate: {e}"))?
                };
                let overlay = build_overlay_image(&job.rgb, &fm.mask, &[]);
                Ok(LiveAiJobOutput {
                    tracking: !fm.status.is_lost(),
                    iou: fm.iou_score,
                    provider: provider.to_string(),
                    mask: fm.mask,
                    overlay,
                })
            });
            if let Ok(out) = res {
                let _ = out_tx.try_send(out);
            }
        }
    });
    (in_tx, out_rx)
}

fn mask_depth_map(depth: &DepthMap, cam: &ColorCamera, mask: &Mask2D) -> DepthMap {
    let Some(k) = depth.intrinsics else {
        return depth.clone();
    };
    let mut masked = depth.data.clone();
    for y in 0..depth.height {
        for x in 0..depth.width {
            let idx = (y * depth.width + x) as usize;
            let raw = masked[idx];
            if raw == 0 {
                continue;
            }
            let z = raw as f32 * depth.scale_mm;
            let optical = glam::Vec3::new((x as f32 - k.cx) * z / k.fx, (y as f32 - k.cy) * z / k.fy, z);
            let c_pt = cam.depth_optical_to_color(optical);
            let keep = cam.project_color(c_pt).is_some_and(|px| {
                let mx = (px.x.round() as i32).clamp(0, mask.width as i32 - 1) as u32;
                let my = (px.y.round() as i32).clamp(0, mask.height as i32 - 1) as u32;
                mask.is_foreground(mx, my)
            });
            if !keep {
                masked[idx] = 0;
            }
        }
    }
    DepthMap::new(depth.width, depth.height, depth.scale_mm, masked).with_intrinsics(k)
}

/// Events per second over a sliding window.
#[derive(Debug, Default)]
pub struct RateWindow {
    stamps: std::collections::VecDeque<std::time::Instant>,
}

impl RateWindow {
    const WINDOW: std::time::Duration = std::time::Duration::from_secs(2);

    pub fn tick(&mut self, now: std::time::Instant) {
        self.stamps.push_back(now);
        self.trim(now);
    }

    fn trim(&mut self, now: std::time::Instant) {
        while self.stamps.front().is_some_and(|t| now.duration_since(*t) > Self::WINDOW) {
            self.stamps.pop_front();
        }
    }

    /// Events per second; 0 until there are two events in the window.
    pub fn rate(&mut self, now: std::time::Instant) -> f32 {
        self.trim(now);
        match (self.stamps.front(), self.stamps.len()) {
            (Some(first), n) if n >= 2 => {
                // Over the window once it is full, else over the span seen so far.
                let span = now.duration_since(*first).as_secs_f32().max(1e-3);
                (n - 1) as f32 / span
            }
            _ => 0.0,
        }
    }
}

/// Settings the worker needs.
#[derive(Clone, Copy, Debug)]
pub struct WorkerConfig {
    pub min_mm: f32,
    pub max_mm: f32,
    /// Spacing of the fused surface.
    pub point_pitch_mm: f32,
    /// IMU mounting reported by the device, for the tracker's motion prior.
    pub imu_to_sensor: Option<[f32; 4]>,
}

/// Grey preview of an 8-bit IR (or RGB) image, at most `max_w` wide.
pub fn ir_preview(img: &RgbImage, max_w: usize) -> Option<ColorImage> {
    let (w, h) = (img.width as usize, img.height as usize);
    if w == 0 || h == 0 || img.data.len() < w * h * img.channels.max(1) as usize {
        return None;
    }
    let step = (w / max_w.max(1)).max(1);
    let (pw, ph) = (w / step, h / step);
    let ch = img.channels.max(1) as usize;
    let mut out = Vec::with_capacity(pw * ph * 4);
    for py in 0..ph {
        for px in 0..pw {
            let (x0, y0) = (px * step, py * step);
            // Max-pool the cell so one-pixel laser lines survive downsampling.
            let mut v = [0u8; 3];
            for y in y0..(y0 + step).min(h) {
                for x in x0..(x0 + step).min(w) {
                    let i = (y * w + x) * ch;
                    for c in 0..3 {
                        v[c] = v[c].max(img.data[i + if ch >= 3 { c } else { 0 }]);
                    }
                }
            }
            out.extend_from_slice(&[v[0], v[1], v[2], 255]);
        }
    }
    Some(ColorImage::from_rgba_unmultiplied([pw, ph], &out))
}

/// Colour preview of an 8-bit RGB image, at most `max_w` wide (nearest-cell sample).
pub fn rgb_preview(img: &RgbImage, max_w: usize) -> Option<ColorImage> {
    let (w, h) = (img.width as usize, img.height as usize);
    let ch = img.channels.max(1) as usize;
    if w == 0 || h == 0 || ch < 3 || img.data.len() < w * h * ch {
        return None;
    }
    let step = (w / max_w.max(1)).max(1);
    let (pw, ph) = ((w / step).max(1), (h / step).max(1));
    let mut out = Vec::with_capacity(pw * ph * 4);
    for py in 0..ph {
        for px in 0..pw {
            let x = (px * step + step / 2).min(w - 1);
            let y = (py * step + step / 2).min(h - 1);
            let i = (y * w + x) * ch;
            out.extend_from_slice(&[img.data[i], img.data[i + 1], img.data[i + 2], 255]);
        }
    }
    Some(ColorImage::from_rgba_unmultiplied([pw, ph], &out))
}

/// Depth preview: grey by distance inside the working range, amber when too close, blue when too
/// far, dark where there is no measurement. Returns the image and the mean in-range distance.
pub fn depth_preview(depth: &DepthMap, min_mm: f32, max_mm: f32, max_w: usize) -> (ColorImage, Option<f32>) {
    let (w, h) = (depth.width as usize, depth.height as usize);
    let step = (w / max_w.max(1)).max(1);
    let (pw, ph) = ((w / step).max(1), (h / step).max(1));
    let mut out = Vec::with_capacity(pw * ph * 4);
    let (mut sum, mut count) = (0.0f64, 0usize);
    for py in 0..ph {
        for px in 0..pw {
            let mut d = None;
            'cell: for y in py * step..((py + 1) * step).min(h) {
                for x in px * step..((px + 1) * step).min(w) {
                    if let Some(v) = depth.get_depth_mm(x as u32, y as u32) {
                        d = Some(v);
                        break 'cell;
                    }
                }
            }
            let rgba = match d {
                None => [24, 27, 33, 255],
                Some(d) if d < min_mm => [236, 170, 70, 255],
                Some(d) if d > max_mm => [70, 110, 170, 255],
                Some(d) => {
                    sum += d as f64;
                    count += 1;
                    let t = ((d - min_mm) / (max_mm - min_mm).max(1.0)).clamp(0.0, 1.0);
                    let g = (235.0 - t * 150.0) as u8;
                    [g, g, g.saturating_add(8), 255]
                }
            };
            out.extend_from_slice(&rgba);
        }
    }
    let mean = (count > 0).then(|| (sum / count as f64) as f32);
    (ColorImage::from_rgba_unmultiplied([pw, ph], &out), mean)
}

/// A tracker for a new pass.
pub fn new_tracker(config: &WorkerConfig) -> SharedTracker {
    Arc::new(std::sync::Mutex::new(Tracker::with_config(TrackerConfig {
        fusion_voxel_mm: config.point_pitch_mm,
        imu_to_sensor: config.imu_to_sensor.map(glam::Quat::from_array),
        ..TrackerConfig::default()
    })))
}

/// Process scan frames until the channel closes or the UI drops its receiver. The tracker
/// belongs to the pass and outlives this worker; while `paused` is set frames are previewed but
/// not registered, so the model only changes while the user is capturing.
pub async fn scan_worker(
    mut frames: mpsc::Receiver<SensorFrame>,
    ui: mpsc::Sender<ScanUpdate>,
    config: WorkerConfig,
    scan_config: scancraft_driver::ScanConfig,
    tracker: SharedTracker,
    paused: Arc<AtomicBool>,
    live_ai: Arc<LiveAiState>,
    capture: Option<(crate::capture::SharedCapture, usize)>,
) {
    // The scan mode decides how frames become geometry and poses (docs/architecture/scan-sessions.md).
    let mut pipeline = match scancraft_pipeline::pipeline_for(&scan_config, config.min_mm, config.max_mm) {
        Ok(p) => p,
        Err(e) => {
            log::error!("scan worker: {e}");
            return;
        }
    };
    let (ai_in, ai_out) = spawn_live_ai_helper(Arc::clone(&live_ai.engine));
    let mut latest_ai: Option<LiveAiJobOutput> = None;
    let min_iou = PostprocessConfig::default().min_iou;

    // Points and keyframes are handed over once; an update the interface could not take yet
    // carries them into the next one.
    let mut carried_points: Vec<scancraft_core::Point3D> = Vec::new();
    let mut carried_keyframes: Vec<KeyframeInfo> = Vec::new();
    let mut carried_rgb_keyframes: Vec<scancraft_color::RgbKeyframe> = Vec::new();
    let (mut incoming, mut processed, mut marker) = (RateWindow::default(), RateWindow::default(), RateWindow::default());
    let mut region_logged = false;
    while let Some(mut frame) = frames.recv().await {
        incoming.tick(std::time::Instant::now());
        let mut skipped = 0u64;
        // Always work on the newest frame.
        while let Ok(newer) = frames.try_recv() {
            incoming.tick(std::time::Instant::now());
            frame = newer;
            skipped += 1;
        }
        let host_ms = capture.as_ref().and_then(|(c, seg)| {
            let mut log = c.lock().ok()?;
            let ms = log.now_ms();
            let f = &mut log.record.captures.get_mut(*seg)?.frames;
            f.received += skipped + 1;
            f.skipped_for_newer += skipped;
            Some(ms)
        });
        processed.tick(std::time::Instant::now());

        while let Ok(res) = ai_out.try_recv() {
            latest_ai = Some(res);
        }
        if !live_ai.enabled.load(Ordering::Relaxed) {
            latest_ai = None;
        } else if let Some(c) = frame.color.as_ref() {
            let prompt_norm = live_ai.pending_prompt.lock().ok().and_then(|mut g| g.take());
            let reset = live_ai.reset_requested.swap(false, Ordering::Relaxed);
            if reset {
                latest_ai = None;
            }
            let _ = ai_in.try_send(LiveAiJobInput {
                frame_index: frame.frame_index,
                rgb: c.image.clone(),
                prompt_norm,
                reset,
            });
        }

        if let (Some(ai), Some(cam), Some(depth)) = (
            latest_ai.as_ref().filter(|a| a.tracking && a.iou >= min_iou),
            frame.color.as_ref().and_then(|c| c.camera.as_ref()),
            frame.depth.as_ref(),
        ) {
            frame.depth = Some(mask_depth_map(depth, cam, &ai.mask));
        }

        let (depth_image, distance) = match frame.depth.as_ref() {
            Some(d) => {
                let (img, dist) = depth_preview(d, config.min_mm, config.max_mm, 320);
                (Some(img), dist)
            }
            None => (None, None),
        };
        let ir_image = frame.ir.as_ref().and_then(|ir| ir_preview(&ir.left, 320));
        let color_image = frame.color.as_ref().and_then(|c| rgb_preview(&c.image, 320));
        let (ai_mask_overlay, ai_status) = match latest_ai.as_ref() {
            Some(ai) => (Some(ai.overlay.clone()), Some((ai.iou, ai.provider.clone()))),
            None => (None, None),
        };

        // The tracker fuses only frames it could align and hands back just the newly seen
        // surface, so holding still or losing track never stacks duplicate points.
        let mut new_points = Vec::new();
        let mut tracking = None;
        let mut keyframe = None;
        let mut seed = None;
        let mut overlay = None;
        if !paused.load(Ordering::Relaxed) {
            let Ok(mut tracker) = tracker.lock() else { break };
            let t0 = std::time::Instant::now();
            let status = pipeline.process(&frame, &mut tracker);
            let proc_ms = t0.elapsed().as_secs_f32() * 1000.0;
            if let Some(status) = status {
                new_points = tracker.take_new_points();
                pipeline.colorize(&mut new_points);
                keyframe = status.keyframe.and_then(|id| tracker.keyframes().iter().find(|k| k.id == id)).map(KeyframeInfo::of);
                tracking = Some(status);
                seed = Some(tracker.seed_state()).filter(|s| *s != scancraft_slam::SeedState::None);
            }
            if let Some((c, seg)) = capture.as_ref() {
                log_frame(c, *seg, &frame, tracking.as_ref(), new_points.len(), &tracker.current_pose, host_ms.unwrap_or(0), proc_ms);
                if !region_logged {
                    if let Some(region) = pipeline.turntable_region() {
                        region_logged = true;
                        if let Ok(mut log) = c.lock() {
                            log.set_turntable_region(&scan_config, Some(region));
                        }
                    }
                }
            }
            overlay = pipeline.overlay();
            carried_rgb_keyframes.extend(pipeline.take_new_rgb_keyframes());
        } else if let Some((c, seg)) = capture.as_ref() {
            if let Some(s) = c.lock().ok().as_mut().and_then(|l| l.record.captures.get_mut(*seg)) {
                s.frames.paused += 1;
            }
        }
        let now = std::time::Instant::now();
        // Count a marker update only for frames the pipeline actually processed (repeated IR
        // pairs are skipped and leave the previous overlay in place).
        if tracking.is_some() && overlay.as_ref().is_some_and(|o| !o.stale) {
            marker.tick(now);
        }
        let rates = Some(FrameRates {
            incoming: incoming.rate(now),
            processed: processed.rate(now),
            marker: pipeline.overlay_capable().then(|| marker.rate(now)),
        });
        carried_points.extend(new_points);
        carried_keyframes.extend(keyframe);
        let update = ScanUpdate {
            ir_image,
            depth_image,
            color_image,
            distance_mm: distance,
            tracking,
            new_points: std::mem::take(&mut carried_points),
            keyframes: std::mem::take(&mut carried_keyframes),
            rgb_keyframes: std::mem::take(&mut carried_rgb_keyframes),
            ai_mask_overlay,
            ai_status,
            seed,
            overlay,
            rates,
        };
        match ui.try_send(update) {
            Ok(()) => {}
            Err(mpsc::error::TrySendError::Full(update)) => {
                carried_points = update.new_points;
                carried_keyframes = update.keyframes;
                carried_rgb_keyframes = update.rgb_keyframes;
            }
            Err(mpsc::error::TrySendError::Closed(_)) => break,
        }
    }
}

/// One processed frame into the pass's tracking log and frame counts.
#[allow(clippy::too_many_arguments)]
fn log_frame(
    capture: &crate::capture::SharedCapture,
    segment: usize,
    frame: &SensorFrame,
    status: Option<&scancraft_slam::TrackingStatus>,
    new_points: usize,
    current_pose: &scancraft_core::Transform,
    host_ms: u64,
    proc_ms: f32,
) {
    let Ok(mut log) = capture.lock() else { return };
    let pose = status.map_or(*current_pose, |s| s.current_pose);
    let row = scancraft_io::capture::TrackRow {
        segment: segment as u32,
        frame: frame.frame_index,
        device_ms: frame.timestamp_ms,
        host_ms,
        state: status.map_or("no_geometry", |s| crate::capture::state_name(s.state)).to_string(),
        relocalized: status.is_some_and(|s| s.relocalized),
        inliers: status.map_or(0, |s| s.inliers as u64),
        rmse_mm: status.map_or(0.0, |s| s.rmse_mm),
        conflict: status.map_or(0.0, |s| s.visibility_conflict),
        keyframe: status.and_then(|s| s.keyframe),
        new_points: new_points as u64,
        total_points: status.map_or(0, |s| s.total_points_fused as u64),
        rotation: pose.rotation.to_array(),
        translation: pose.translation.to_array(),
        imu: frame.imu.map(|i| i.orientation.to_array()),
        proc_ms,
    };
    log.rows.push(row);
    let Some(f) = log.record.captures.get_mut(segment).map(|s| &mut s.frames) else { return };
    f.processed += 1;
    f.fused_points += new_points as u64;
    match status {
        None => f.no_geometry += 1,
        Some(s) => {
            match s.state {
                scancraft_slam::TrackingState::Tracking => f.tracking += 1,
                scancraft_slam::TrackingState::Slow => f.slow += 1,
                scancraft_slam::TrackingState::Lost => f.lost += 1,
                scancraft_slam::TrackingState::Idle => {}
            }
            f.relocalized += s.relocalized as u64;
            f.keyframes += s.keyframe.is_some() as u64;
        }
    }
}

/// Process preview frames (aiming before a scan).
pub async fn preview_worker(mut frames: mpsc::Receiver<PreviewFrame>, ui: mpsc::Sender<ScanUpdate>) {
    while let Some(mut frame) = frames.recv().await {
        while let Ok(newer) = frames.try_recv() {
            frame = newer;
        }
        let ir_image = frame.ir_image.as_ref().or(frame.rgb_image.as_ref()).and_then(|img| ir_preview(img, 320));
        let color_image = frame.rgb_image.as_ref().and_then(|img| rgb_preview(img, 320));
        let update = ScanUpdate {
            ir_image,
            depth_image: None,
            color_image,
            distance_mm: frame.average_distance_mm,
            tracking: None,
            new_points: Vec::new(),
            keyframes: Vec::new(),
            rgb_keyframes: Vec::new(),
            ai_mask_overlay: None,
            ai_status: None,
            seed: None,
            overlay: None,
            rates: None,
        };
        match ui.try_send(update) {
            Ok(()) | Err(mpsc::error::TrySendError::Full(_)) => {}
            Err(mpsc::error::TrySendError::Closed(_)) => break,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn depth_preview_marks_range() {
        let mut data = vec![0u16; 4];
        data[0] = 500; // 50 mm: too close
        data[1] = 2500; // in range
        data[2] = 9000; // too far
        let depth = DepthMap::new(2, 2, 0.1, data);
        let (img, mean) = depth_preview(&depth, 100.0, 400.0, 2);
        assert_eq!(img.size, [2, 2]);
        assert_eq!(mean, Some(250.0));
        assert_eq!(img.pixels[0], eframe::egui::Color32::from_rgb(236, 170, 70));
    }

    #[test]
    fn rate_window_counts_events_per_second() {
        let t0 = std::time::Instant::now();
        let mut w = RateWindow::default();
        assert_eq!(w.rate(t0), 0.0);
        // 15 frames a second for one second.
        for i in 0..=15 {
            w.tick(t0 + std::time::Duration::from_millis(i * 1000 / 15));
        }
        let r = w.rate(t0 + std::time::Duration::from_millis(1000));
        assert!((r - 15.0).abs() < 0.5, "{r}");
        // Old events fall out of the window.
        let later = t0 + std::time::Duration::from_secs(5);
        assert_eq!(w.rate(later), 0.0);
    }

    #[test]
    fn ir_preview_keeps_thin_lines() {
        let mut data = vec![0u8; 8 * 8];
        data[3 * 8 + 5] = 255;
        let img = RgbImage { width: 8, height: 8, channels: 1, data };
        let p = ir_preview(&img, 4).unwrap();
        assert_eq!(p.size, [4, 4]);
        assert!(p.pixels.iter().any(|c| c.r() == 255));
    }
}
