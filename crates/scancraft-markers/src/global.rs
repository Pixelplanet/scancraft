//! Global Marker workflow (Revo's "Global Markers": scan the marker frame first, then the object
//! against it).
//!
//! Phase 1: track with a growing map and `record_observations: true`
//! ([`GlobalMarkerSession::phase1`]), then [`refine_map`] adjusts all marker positions and frame
//! poses jointly (least squares over every observation) and the map is saved as JSON.
//! Phase 2: load the map and track against it unchanged ([`GlobalMarkerSession::phase2`]).

use crate::map::{MarkerMap, SharedMarkerMap};
use crate::tracker::{FrameObservation, MarkerTracker, MarkerTrackerConfig, SharedObservations};
use glam::Vec3;
use scancraft_core::Transform;
use scancraft_slam::kabsch::kabsch_align;
use std::collections::HashMap;
use std::path::Path;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RefineReport {
    /// RMS distance between each observation (mapped by its frame pose) and its marker, mm.
    pub rms_before_mm: f32,
    pub rms_after_mm: f32,
    pub iterations: usize,
    /// Markers with fewer than `min_observations` that were removed.
    pub pruned: usize,
}

/// Joint refinement of marker positions and frame poses ("3D bundle adjustment"): alternates
/// (a) every frame pose = Kabsch fit of its observations to the current marker positions and
/// (b) every marker position = mean of its observations mapped by the frame poses, until the
/// residual stops improving. The result is re-anchored to the original map frame (Kabsch over
/// all markers) so the pass frame does not drift. Markers seen fewer than `min_observations`
/// times are removed first.
pub fn refine_map(map: &mut MarkerMap, frames: &mut [FrameObservation], min_observations: u32, max_iterations: usize) -> RefineReport {
    let before = map.len();
    map.prune(min_observations);
    let pruned = before - map.len();
    let index: HashMap<u32, usize> = map.markers.iter().enumerate().map(|(i, m)| (m.id, i)).collect();
    let original: Vec<Vec3> = map.positions();
    let mut positions = original.clone();
    let residual = |positions: &[Vec3], frames: &[FrameObservation]| -> f32 {
        let (mut sq, mut n) = (0.0f64, 0usize);
        for f in frames {
            for (id, p) in &f.markers {
                if let Some(&i) = index.get(id) {
                    sq += f.pose.transform_point3(*p).distance_squared(positions[i]) as f64;
                    n += 1;
                }
            }
        }
        if n == 0 { 0.0 } else { (sq / n as f64).sqrt() as f32 }
    };
    let rms_before = residual(&positions, frames);
    let mut rms = rms_before;
    let mut iterations = 0;
    for _ in 0..max_iterations {
        iterations += 1;
        // (a) poses.
        for f in frames.iter_mut() {
            let (src, tgt): (Vec<Vec3>, Vec<Vec3>) = f.markers.iter().filter_map(|(id, p)| index.get(id).map(|&i| (*p, positions[i]))).unzip();
            // Keep the old pose unless the fit really is better: with few, nearly collinear
            // points the SVD-based Kabsch can return a poor rotation.
            if let Some((q, t)) = kabsch_align(&src, &tgt) {
                let cand = Transform::from_rotation_translation(q, t);
                let err = |tf: &Transform| src.iter().zip(&tgt).map(|(s, g)| tf.transform_point3(*s).distance_squared(*g)).sum::<f32>();
                if err(&cand) < err(&f.pose) {
                    f.pose = cand;
                }
            }
        }
        // (b) positions.
        let mut sum = vec![(Vec3::ZERO, 0u32); positions.len()];
        for f in frames.iter() {
            for (id, p) in &f.markers {
                if let Some(&i) = index.get(id) {
                    sum[i].0 += f.pose.transform_point3(*p);
                    sum[i].1 += 1;
                }
            }
        }
        for (i, (s, n)) in sum.iter().enumerate() {
            if *n > 0 {
                positions[i] = *s / *n as f32;
            }
        }
        let next = residual(&positions, frames);
        log::debug!("refine iteration {iterations}: rms {next:.4} mm");
        let converged = rms - next < 1e-4;
        rms = next;
        if converged {
            break;
        }
    }
    // Re-anchor to the original frame.
    if let Some((q, t)) = kabsch_align(&positions, &original) {
        let anchor = Transform::from_rotation_translation(q, t);
        positions.iter_mut().for_each(|p| *p = anchor.transform_point3(*p));
        for f in frames.iter_mut() {
            f.pose = crate::constellation::compose(&anchor, &f.pose);
        }
    }
    for (m, p) in map.markers.iter_mut().zip(&positions) {
        m.position = p.to_array();
    }
    RefineReport { rms_before_mm: rms_before, rms_after_mm: residual(&positions, frames), iterations, pruned }
}

/// Convenience wrapper for the two phases.
pub struct GlobalMarkerSession;

impl GlobalMarkerSession {
    /// Phase 1 tracker: growing map, observations recorded for refinement.
    pub fn phase1(mut config: MarkerTrackerConfig) -> MarkerTracker {
        config.record_observations = true;
        MarkerTracker::new(config)
    }

    /// Ends phase 1: refines the map with the recorded observations and saves it. Takes the
    /// handles (`tracker.map()`, `tracker.observations()`) so it works while the tracker is boxed
    /// inside a `PassPipeline`.
    pub fn finish_phase1(map: &SharedMarkerMap, observations: &SharedObservations, path: &Path) -> std::io::Result<RefineReport> {
        let mut frames = std::mem::take(&mut *observations.lock().unwrap());
        let mut map = map.lock().unwrap();
        let report = refine_map(&mut map, &mut frames, 2, 50);
        map.save_json(path)?;
        Ok(report)
    }

    /// Phase 2 tracker: the saved map, unchanged while scanning.
    pub fn phase2(config: MarkerTrackerConfig, path: &Path) -> std::io::Result<MarkerTracker> {
        Ok(MarkerTracker::with_fixed_map(config, MarkerMap::load_json(path)?))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::constellation::tests::{constellation, pose};
    use crate::detect::tests::Lcg;

    #[test]
    fn two_phase_workflow_saves_and_tracks_against_the_map() {
        use crate::tracker::tests::{render, OFFSET};
        use scancraft_slam::HintKind;
        let world = constellation(31, 24);
        let cfg = MarkerTrackerConfig { right_cx_offset: Some(OFFSET), ..Default::default() };
        let mut p1 = GlobalMarkerSession::phase1(cfg.clone());
        let (map, obs) = (p1.map(), p1.observations());
        let mut last = Transform::default();
        for i in 0..12u64 {
            let truth = pose(0.0, 0.0, 0.02 * i as f32, Vec3::new(2.0 * i as f32, 0.0, 0.0));
            last = p1.estimate(&render(&world, &truth, i), &last).expect("phase 1 hint").pose;
        }
        let path = std::env::temp_dir().join(format!("scancraft-global-{}.json", std::process::id()));
        let report = GlobalMarkerSession::finish_phase1(&map, &obs, &path).unwrap();
        assert!(report.rms_after_mm <= report.rms_before_mm + 1e-4 && report.rms_after_mm < 0.3, "{report:?}");
        let mut p2 = GlobalMarkerSession::phase2(cfg, &path).unwrap();
        std::fs::remove_file(&path).ok();
        let truth = pose(0.05, 0.0, 0.1, Vec3::new(10.0, 5.0, 0.0));
        let hint = p2.estimate(&render(&world, &truth, 99), &Transform::default()).expect("phase 2 hint");
        assert_eq!(hint.kind, HintKind::Fixed);
        assert!(hint.pose.translation.distance(truth.translation) < 0.5);
    }

    #[test]
    fn refinement_reduces_accumulated_error() {
        let truth_world = constellation(77, 25);
        let mut rng = Lcg(9);
        let mut noise = || Vec3::new(rng.next_f32() - 0.5, rng.next_f32() - 0.5, rng.next_f32() - 0.5);
        // Map built by drift-prone tracking: positions off by up to ~1 mm.
        let mut map = MarkerMap::new();
        for p in &truth_world {
            map.add(*p + noise() * 2.0, 6.0);
        }
        for i in 0..map.markers.len() {
            map.markers[i].observations = 5;
        }
        // 30 frames with exact-ish observations (0.05 mm noise) and slightly wrong poses.
        let mut frames = Vec::new();
        for f in 0..30u64 {
            let t = f as f32 * 0.1;
            let truth = pose(0.1 * t.sin(), 0.05 * t, 0.2 * t.cos(), Vec3::new(10.0 * t, 5.0 * t.sin(), 2.0));
            let markers = truth_world
                .iter()
                .enumerate()
                .filter(|(i, _)| !(*i as u64 + f).is_multiple_of(3))
                .map(|(i, p)| (i as u32, truth.inverse().transform_point3(*p) + noise() * 0.1))
                .collect();
            let rough = Transform::from_rotation_translation(truth.rotation, truth.translation + noise());
            frames.push(FrameObservation { frame_index: f, pose: rough, markers });
        }
        let report = refine_map(&mut map, &mut frames, 2, 100);
        assert!(report.rms_after_mm < 0.1 && report.rms_after_mm < report.rms_before_mm / 5.0, "{report:?}");
        // Inter-marker distances are now right (the gauge is free, distances are not).
        let mut worst = 0.0f32;
        for i in 0..truth_world.len() {
            for j in i + 1..truth_world.len() {
                let d_true = truth_world[i].distance(truth_world[j]);
                let d_map = map.markers[i].pos().distance(map.markers[j].pos());
                worst = worst.max((d_true - d_map).abs());
            }
        }
        assert!(worst < 0.15, "worst distance error {worst} mm");
    }
}
