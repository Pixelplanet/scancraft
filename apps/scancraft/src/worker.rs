//! Background workers: turn raw scan frames into camera previews, a distance reading, a tracked
//! pose and world-space points; and turn preview frames into previews and a distance reading.

use eframe::egui::ColorImage;
use scancraft_core::{DepthMap, PreviewFrame, RgbImage, SensorFrame};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};

use scancraft_slam::{Tracker, TrackerConfig};
use tokio::sync::mpsc;

use crate::model::{FrameRates, KeyframeInfo, ScanUpdate, SharedTracker};

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
) {
    // The scan mode decides how frames become geometry and poses (docs/architecture/scan-sessions.md).
    let mut pipeline = match scancraft_pipeline::pipeline_for(&scan_config, config.min_mm, config.max_mm) {
        Ok(p) => p,
        Err(e) => {
            log::error!("scan worker: {e}");
            return;
        }
    };
    // Points and keyframes are handed over once; an update the interface could not take yet
    // carries them into the next one.
    let mut carried_points: Vec<scancraft_core::Point3D> = Vec::new();
    let mut carried_keyframes: Vec<KeyframeInfo> = Vec::new();
    let (mut incoming, mut processed, mut marker) = (RateWindow::default(), RateWindow::default(), RateWindow::default());
    while let Some(mut frame) = frames.recv().await {
        incoming.tick(std::time::Instant::now());
        // Always work on the newest frame.
        while let Ok(newer) = frames.try_recv() {
            incoming.tick(std::time::Instant::now());
            frame = newer;
        }
        processed.tick(std::time::Instant::now());
        let (depth_image, distance) = match frame.depth.as_ref() {
            Some(d) => {
                let (img, dist) = depth_preview(d, config.min_mm, config.max_mm, 320);
                (Some(img), dist)
            }
            None => (None, None),
        };
        let ir_image = frame.ir.as_ref().and_then(|ir| ir_preview(&ir.left, 320));


        // The tracker fuses only frames it could align and hands back just the newly seen
        // surface, so holding still or losing track never stacks duplicate points.
        let mut new_points = Vec::new();
        let mut tracking = None;
        let mut keyframe = None;
        let mut seed = None;
        let mut overlay = None;
        if !paused.load(Ordering::Relaxed) {
            let Ok(mut tracker) = tracker.lock() else { break };
            if let Some(status) = pipeline.process(&frame, &mut tracker) {
                new_points = tracker.take_new_points();
                pipeline.colorize(&mut new_points);
                keyframe = status.keyframe.and_then(|id| tracker.keyframes().iter().find(|k| k.id == id)).map(KeyframeInfo::of);
                tracking = Some(status);
                seed = Some(tracker.seed_state()).filter(|s| *s != scancraft_slam::SeedState::None);
            }
            overlay = pipeline.overlay();
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
            distance_mm: distance,
            tracking,
            new_points: std::mem::take(&mut carried_points),
            keyframes: std::mem::take(&mut carried_keyframes),
            seed,
            overlay,
            rates,
        };
        match ui.try_send(update) {
            Ok(()) => {}
            Err(mpsc::error::TrySendError::Full(update)) => {
                carried_points = update.new_points;
                carried_keyframes = update.keyframes;
            }
            Err(mpsc::error::TrySendError::Closed(_)) => break,
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
        let update = ScanUpdate { ir_image, depth_image: None, distance_mm: frame.average_distance_mm, tracking: None, new_points: Vec::new(), keyframes: Vec::new(), seed: None, overlay: None, rates: None };
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
