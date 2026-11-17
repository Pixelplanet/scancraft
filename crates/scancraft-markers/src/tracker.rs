//! Marker tracking: per frame, detect + triangulate the dots, register them against the pass's
//! marker map, return a pose hint for the geometric tracker and grow the map.

use crate::constellation::{compose, register, MatchConfig};
use crate::detect::{detect_markers, DetectorConfig};
use crate::map::{MarkerMap, SharedMarkerMap};
use crate::stereo::{match_stereo, offset_from_samples, right_cx_offset_samples, Marker3D, StereoConfig, StereoGeometry};
use glam::Vec3;
use scancraft_core::{SensorFrame, Transform};
use scancraft_slam::{HintKind, PoseHint};
use serde::{Deserialize, Serialize};
use std::sync::{Arc, Mutex};

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MarkerTrackerConfig {
    pub detector: DetectorConfig,
    pub stereo: StereoConfig,
    pub matching: MatchConfig,
    /// `cx_right - cx_left` of the rectified pair, px. `None` = estimate it from the first frames
    /// that carry on-device depth (Full-field HD); without depth it falls back to 0.
    pub right_cx_offset: Option<f32>,
    /// A hint is `Fixed` with at least this many inliers and an RMS residual below
    /// `fixed_max_rms_mm`, `Prior` with at least 3 inliers below `prior_max_rms_mm`.
    pub fixed_min_inliers: usize,
    pub fixed_max_rms_mm: f32,
    pub prior_max_rms_mm: f32,
    /// New markers closer than this to an existing one are treated as the same marker, mm.
    pub merge_mm: f32,
    /// Also grow the map from `Prior` frames whose residual is below this, mm. With only three
    /// shared markers the pose is still exact enough to place new ones, and without it a pass
    /// that moves on to new markers loses the map (seen on a 90 deg orbit around a turntable).
    pub grow_max_rms_mm: f32,
    /// Keep per-frame observations for Global Marker refinement (`global::refine_map`).
    pub record_observations: bool,
}

impl Default for MarkerTrackerConfig {
    fn default() -> Self {
        Self {
            detector: DetectorConfig::default(),
            stereo: StereoConfig::default(),
            matching: MatchConfig::default(),
            right_cx_offset: None,
            fixed_min_inliers: 4,
            fixed_max_rms_mm: 0.6,
            prior_max_rms_mm: 2.0,
            merge_mm: 5.0,
            grow_max_rms_mm: 0.3,
            record_observations: false,
        }
    }
}

/// Whether the map grows (Marker, Global Marker phase 1) or stays as loaded (phase 2).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MapMode {
    Build,
    Fixed,
}

/// One frame's markers with the pose they were registered at, for map refinement.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FrameObservation {
    pub frame_index: u64,
    /// World <- sensor at registration time.
    pub pose: Transform,
    /// (map marker id, position in the sensor frame).
    pub markers: Vec<(u32, Vec3)>,
}

/// Observations recorded by a tracker, shared with the app.
pub type SharedObservations = Arc<Mutex<Vec<FrameObservation>>>;

/// What happened in the last [`MarkerTracker::estimate`] call (for UI / logs / tests).
#[derive(Debug, Clone, Default, PartialEq)]
pub struct FrameReport {
    pub detected_left: usize,
    pub detected_right: usize,
    pub triangulated: usize,
    pub inliers: usize,
    pub rms_mm: f32,
    pub hint: Option<HintKind>,
    pub map_size: usize,
    pub relocalized: bool,
    /// The IR pair was byte-identical to the previous frame's (the POP 4's z16y8y8p stream
    /// repeats the IR planes on every other packet while the depth is new); no hint is given
    /// because the markers belong to an earlier instant than the depth.
    pub stale_ir: bool,
}

/// One dot detected in the left IR image, and what became of it in this frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DotState {
    /// Centre in the left (rectified) IR image, px.
    pub x: f32,
    pub y: f32,
    /// Semi-major axis of the dot, px.
    pub radius: f32,
    /// Matched with the right image and triangulated.
    pub triangulated: bool,
    /// An inlier of the pose this frame handed to the tracker.
    pub tracking: bool,
}

/// The markers of the last frame as the user should see them: every dot of the left IR image,
/// which were triangulated, which carried the pose, and which map markers they matched.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct MarkerFrameState {
    /// Size of the left IR image the dots are in, px.
    pub image_size: (u32, u32),
    pub dots: Vec<DotState>,
    /// Ids of the map markers matched by the tracking dots.
    pub matched_map: Vec<u32>,
    /// The IR pair repeated the previous frame's: these are that frame's dots and nothing new
    /// was observed.
    pub stale: bool,
}

impl MarkerFrameState {
    pub fn detected(&self) -> usize {
        self.dots.len()
    }

    pub fn triangulated(&self) -> usize {
        self.dots.iter().filter(|d| d.triangulated).count()
    }

    pub fn tracking(&self) -> usize {
        self.dots.iter().filter(|d| d.tracking).count()
    }
}

pub struct MarkerTracker {
    pub config: MarkerTrackerConfig,
    mode: MapMode,
    map: SharedMarkerMap,
    right_cx_offset: Option<f32>,
    offset_attempts: u32,
    offset_samples: Vec<f32>,
    frames_seen: u64,
    /// Markers of a frame seen before the map existed, placed once its pose is known.
    pending: Option<Vec<Marker3D>>,
    previous_pose: Option<Transform>,
    observations: SharedObservations,
    last_markers: Vec<Marker3D>,
    last_ir_fingerprint: Option<u64>,
    report: FrameReport,
    state: MarkerFrameState,
    /// For each marker of `last_markers`, its dot in `state.dots`.
    marker_dots: Vec<Option<usize>>,
}

fn ir_fingerprint(ir: &scancraft_core::IrImages) -> u64 {
    let mut h = 0xcbf29ce484222325u64;
    for &v in ir.left.data.iter().step_by(3).chain(ir.right.iter().flat_map(|r| r.data.iter().step_by(3))) {
        h ^= v as u64;
        h = h.wrapping_mul(0x100000001b3);
    }
    h
}

impl MarkerTracker {
    /// Marker tracking / Global Marker phase 1: starts with an empty map that grows.
    pub fn new(config: MarkerTrackerConfig) -> Self {
        Self::with_map(config, MarkerMap::new(), MapMode::Build)
    }

    /// Global Marker phase 2: track against a finished map without changing it. The map's frame
    /// becomes the pass frame (the first frame's pose comes from the map).
    pub fn with_fixed_map(config: MarkerTrackerConfig, map: MarkerMap) -> Self {
        Self::with_map(config, map, MapMode::Fixed)
    }

    pub fn with_map(config: MarkerTrackerConfig, map: MarkerMap, mode: MapMode) -> Self {
        Self {
            right_cx_offset: config.right_cx_offset,
            config,
            mode,
            map: Arc::new(Mutex::new(map)),
            offset_attempts: 0,
            offset_samples: Vec::new(),
            frames_seen: 0,
            pending: None,
            previous_pose: None,
            observations: Arc::new(Mutex::new(Vec::new())),
            last_markers: Vec::new(),
            last_ir_fingerprint: None,
            report: FrameReport::default(),
            state: MarkerFrameState::default(),
            marker_dots: Vec::new(),
        }
    }

    /// Handle to the live map (keep it before boxing the tracker into a pipeline).
    pub fn map(&self) -> SharedMarkerMap {
        self.map.clone()
    }

    pub fn mode(&self) -> MapMode {
        self.mode
    }

    /// Handle to the recorded observations (Global Marker phase 1), shared like [`Self::map`].
    pub fn observations(&self) -> SharedObservations {
        self.observations.clone()
    }

    /// Markers triangulated in the last frame (sensor frame).
    pub fn last_markers(&self) -> &[Marker3D] {
        &self.last_markers
    }

    pub fn report(&self) -> &FrameReport {
        &self.report
    }

    /// The last frame's dots and their fate (for the camera view).
    /// True if this frame's IR pair is a repeat of the last one passed to `estimate` (the POP 4
    /// repeats the IR planes on every other packet): it carries no new marker information.
    pub fn is_repeated_ir(&self, frame: &SensorFrame) -> bool {
        frame.ir.as_ref().is_some_and(|ir| self.last_ir_fingerprint == Some(ir_fingerprint(ir)))
    }

    pub fn frame_state(&self) -> &MarkerFrameState {
        &self.state
    }

    /// Marks the dots of the markers that carried the pose (`pairs`: (marker index, map index)).
    fn mark_tracking(&mut self, pairs: &[(usize, usize)], map_ids: &[u32]) {
        for &(s, t) in pairs {
            if let Some(d) = self.marker_dots.get(s).copied().flatten() {
                self.state.dots[d].tracking = true;
            }
            if let Some(&id) = map_ids.get(t) {
                self.state.matched_map.push(id);
            }
        }
    }

    /// The right principal-point offset in use (estimated or configured).
    pub fn right_cx_offset(&self) -> Option<f32> {
        self.right_cx_offset
    }

    /// Detection + stereo for one frame, in the sensor frame. `None` if the frame has no IR pair.
    pub fn detect(&mut self, frame: &SensorFrame) -> Option<Vec<Marker3D>> {
        let ir = frame.ir.as_ref()?;
        let right = ir.right.as_ref()?;
        let rig = ir.rig.as_ref()?;
        // Raw (unrectified) pairs, as the laser-line modes deliver them, need the raw camera
        // models: dots are found in the raw images and only their centres are rectified.
        let raw_cameras = match (ir.rectified, rig.rectification.as_ref()) {
            (true, _) => None,
            (false, Some(r)) => Some(r),
            (false, None) => {
                log::warn!("marker tracking needs a rectified IR pair or the raw camera models");
                return None;
            }
        };
        if self.right_cx_offset.is_none() {
            if let Some(r) = raw_cameras {
                self.right_cx_offset = Some(r.right_cx as f32 - rig.intrinsics.cx);
            }
        }
        // The rig's calibrated offset (cx_right - cx_left) when the driver provides one; the
        // depth-based estimate is only a fallback (it fails when markers sit outside the depth
        // window or on black surfaces without depth).
        if self.right_cx_offset.is_none() && rig.disparity_offset_px != 0.0 {
            self.right_cx_offset = Some(rig.disparity_offset_px);
        }
        if self.right_cx_offset.is_none() && self.offset_attempts < 20 {
            if let Some(depth) = frame.depth.as_ref().filter(|d| d.width == rig.width && d.height == rig.height) {
                self.offset_attempts += 1;
                self.offset_samples.extend(right_cx_offset_samples(ir, depth, rig.intrinsics.fx, rig.baseline_mm));
                self.right_cx_offset = offset_from_samples(&mut self.offset_samples);
                if let Some(o) = self.right_cx_offset {
                    log::info!("marker stereo: right principal point offset {o:.2} px (from on-device depth)");
                }
            } else {
                self.offset_attempts = u32::MAX;
            }
        }
        let geo = StereoGeometry::from_rig(rig, self.right_cx_offset.unwrap_or(0.0));
        // The detector's size limits are for the 640-wide images; scale them with the image.
        let scale = (ir.left.width as f32 / 640.0).max(1.0);
        let detector = if scale > 1.0 {
            let mut d = self.config.detector.clone();
            d.min_area = (d.min_area as f32 * scale * scale) as usize;
            d.max_area = (d.max_area as f32 * scale * scale) as usize;
            d.background_radius = (d.background_radius as f32 * scale) as usize;
            d
        } else {
            self.config.detector.clone()
        };
        let raw_left = detect_markers(&ir.left, &detector);
        let mut left = raw_left.clone();
        let mut right = detect_markers(right, &detector);
        if let Some(r) = raw_cameras {
            for (dots, camera) in [(&mut left, &r.left), (&mut right, &r.right)] {
                for d in dots.iter_mut() {
                    let (u, v) = camera.rectify(d.x as f64, d.y as f64);
                    (d.x, d.y) = (u as f32, v as f32);
                }
            }
        }
        let markers = match_stereo(&left, &right, &geo, &self.config.stereo, frame.depth.as_ref());
        self.report = FrameReport { detected_left: left.len(), detected_right: right.len(), triangulated: markers.len(), ..Default::default() };
        // Which left dot each triangulated marker came from (its left centre is that dot's).
        // Drawn over the left image as delivered (raw coordinates for raw pairs).
        let mut dots: Vec<DotState> = raw_left.iter().map(|d| DotState { x: d.x, y: d.y, radius: d.major.max(d.minor), triangulated: false, tracking: false }).collect();
        self.marker_dots = markers
            .iter()
            .map(|m| {
                let i = left.iter().position(|d| (d.x - m.left.x).abs() < 1e-3 && (d.y - m.left.y).abs() < 1e-3);
                if let Some(i) = i {
                    dots[i].triangulated = true;
                }
                i
            })
            .collect();
        self.state = MarkerFrameState { image_size: (ir.left.width, ir.left.height), dots, matched_map: Vec::new(), stale: false };
        self.last_markers = markers.clone();
        Some(markers)
    }

    /// Pose hint for this frame. `last_pose` is the tracker's latest accepted pose
    /// (world <- sensor).
    pub fn estimate(&mut self, frame: &SensorFrame, last_pose: &Transform) -> Option<PoseHint> {
        let fp = ir_fingerprint(frame.ir.as_ref()?);
        if self.last_ir_fingerprint.replace(fp) == Some(fp) {
            self.report = FrameReport { stale_ir: true, map_size: self.map.lock().unwrap().len(), ..Default::default() };
            // The same image as last frame: same dots, but they observe nothing new.
            self.state.stale = true;
            self.last_markers.clear();
            return None;
        }
        let markers = self.detect(frame)?;
        self.frames_seen += 1;
        let mut map = self.map.lock().unwrap();
        self.report.map_size = map.len();

        // Markers from a frame before the map existed: that frame's pose is now `last_pose`.
        if let Some(p) = self.pending.take() {
            if map.is_empty() && self.mode == MapMode::Build {
                for m in &p {
                    map.add(last_pose.transform_point3(m.position), m.diameter_mm);
                }
            }
        }
        if markers.len() < 3 {
            return None;
        }
        if map.is_empty() {
            if self.mode == MapMode::Fixed {
                return None;
            }
            if self.frames_seen == 1 {
                // First frame of the pass: it defines the world frame together with the tracker.
                let obs: Vec<(u32, Vec3)> =
                    markers.iter().map(|m| (map.add(last_pose.transform_point3(m.position), m.diameter_mm), m.position)).collect();
                if self.config.record_observations {
                    self.observations.lock().unwrap().push(FrameObservation { frame_index: frame.frame_index, pose: *last_pose, markers: obs.clone() });
                }
                self.report.map_size = map.len();
                self.report.hint = Some(HintKind::Fixed);
                self.report.inliers = markers.len();
                // Every marker defines the map, so every one carries the (identity) pose.
                let ids: Vec<u32> = obs.iter().map(|(id, _)| *id).collect();
                let pairs: Vec<(usize, usize)> = (0..markers.len()).map(|i| (i, i)).collect();
                drop(map);
                self.mark_tracking(&pairs, &ids);
                self.previous_pose = Some(*last_pose);
                return Some(PoseHint { pose: *last_pose, kind: HintKind::Fixed });
            }
            self.pending = Some(markers);
            return None;
        }

        // Constant-velocity guess from the last two accepted poses.
        let guess = match self.previous_pose {
            Some(prev) => compose(&compose(last_pose, &prev.inverse()), last_pose),
            None => *last_pose,
        };
        self.previous_pose = Some(*last_pose);
        let positions: Vec<Vec3> = markers.iter().map(|m| m.position).collect();
        let world = map.positions();
        let m = register(&positions, &world, &self.config.matching, Some(&guess))
            .or_else(|| register(&positions, &world, &self.config.matching, Some(last_pose)))?;
        self.report.inliers = m.inliers();
        self.report.rms_mm = m.rms_mm;
        self.report.relocalized = m.transform.translation.distance(guess.translation) > self.config.matching.guess_gate_mm;
        let kind = if m.inliers() >= self.config.fixed_min_inliers && m.rms_mm <= self.config.fixed_max_rms_mm {
            HintKind::Fixed
        } else if m.inliers() >= 3 && m.rms_mm <= self.config.prior_max_rms_mm {
            HintKind::Prior
        } else {
            return None;
        };
        self.report.hint = Some(kind);
        let map_ids: Vec<u32> = map.markers.iter().map(|mm| mm.id).collect();

        if kind == HintKind::Fixed || m.rms_mm <= self.config.grow_max_rms_mm {
            let ids = map_ids.clone();
            let mut obs = Vec::with_capacity(markers.len());
            let mut matched = vec![false; markers.len()];
            for &(s, t) in &m.pairs {
                matched[s] = true;
                obs.push((ids[t], markers[s].position));
                if self.mode == MapMode::Build {
                    map.observe(t, m.transform.transform_point3(markers[s].position), markers[s].diameter_mm);
                }
            }
            if self.mode == MapMode::Build {
                for (s, mk) in markers.iter().enumerate().filter(|(s, _)| !matched[*s]) {
                    let w = m.transform.transform_point3(mk.position);
                    if map.markers.iter().all(|e| e.pos().distance(w) > self.config.merge_mm) {
                        let id = map.add(w, mk.diameter_mm);
                        obs.push((id, markers[s].position));
                    }
                }
            }
            if self.config.record_observations {
                self.observations.lock().unwrap().push(FrameObservation { frame_index: frame.frame_index, pose: m.transform, markers: obs });
            }
            self.report.map_size = map.len();
        }
        drop(map);
        self.mark_tracking(&m.pairs, &map_ids);
        Some(PoseHint { pose: m.transform, kind })
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    use crate::constellation::rotation_delta;
    use crate::constellation::tests::{constellation, pose};
    use crate::detect::tests::{blank, draw_ellipse};
    use scancraft_core::{IrImages, PinholeIntrinsics, StereoRig};

    pub fn rig() -> Arc<StereoRig> {
        Arc::new(StereoRig {
            width: 640,
            height: 512,
            intrinsics: PinholeIntrinsics { fx: 742.278, fy: 742.278, cx: 170.603, cy: 242.566 },
            baseline_mm: 118.58,
            disparity_offset_px: OFFSET,
            calibration_set: "red".into(),
            rectification: None,
        })
    }

    pub const OFFSET: f32 = 304.916;

    /// Renders world markers seen from `pose` (world <- sensor) into a rectified IR pair.
    pub fn render(world: &[Vec3], pose: &Transform, index: u64) -> SensorFrame {
        let rig = rig();
        let geo = StereoGeometry::from_rig(&rig, OFFSET);
        let (mut l, mut r) = (blank(640, 512, 20), blank(640, 512, 20));
        let inv = pose.inverse();
        for p in world {
            let s = inv.transform_point3(*p);
            if let Some((pl, pr)) = geo.project(s) {
                let radius = 3.0 * geo.fx / s.y;
                draw_ellipse(&mut l, pl.x, pl.y, radius, radius, 0.0, 250.0);
                draw_ellipse(&mut r, pr.x, pr.y, radius, radius, 0.0, 250.0);
            }
        }
        let mut f = SensorFrame::new(index, index * 150);
        f.ir = Some(IrImages { left: l, right: Some(r), rectified: true, rig: Some(rig) });
        f
    }

    #[test]
    fn tracks_a_moving_sensor_and_grows_the_map() {
        // Markers spread over a large area; each view sees a subset.
        let world: Vec<Vec3> = constellation(42, 40).into_iter().map(|p| Vec3::new(p.x * 1.5, p.y, p.z)).collect();
        let cfg = MarkerTrackerConfig { right_cx_offset: Some(OFFSET), record_observations: true, ..Default::default() };
        let mut tracker = MarkerTracker::new(cfg);
        let mut last = Transform::default();
        let mut max_err = 0.0f32;
        for i in 0..25u64 {
            let t = i as f32 * 0.04;
            let truth = pose(0.05 * t.sin(), 0.0, 0.25 * t, Vec3::new(30.0 * t, 8.0 * t, 3.0 * t.sin()));
            let frame = render(&world, &truth, i);
            let hint = tracker.estimate(&frame, &last);
            let r = tracker.report().clone();
            let hint = hint.unwrap_or_else(|| panic!("frame {i}: no hint, {r:?}"));
            assert_eq!(hint.kind, HintKind::Fixed, "frame {i}: {r:?}");
            // The first frame defines the world: compare relative to frame 0's truth (identity).
            let err = hint.pose.translation.distance(truth.translation);
            max_err = max_err.max(err);
            assert!(rotation_delta(&hint.pose, &truth) < 0.004, "frame {i}");
            last = hint.pose;
        }
        assert!(max_err < 0.6, "max translation error {max_err} mm");
        assert!(tracker.map().lock().unwrap().len() > 12);
        assert_eq!(tracker.observations().lock().unwrap().len(), 25);
    }

    #[test]
    fn frame_state_tells_detected_triangulated_and_tracking_dots_apart() {
        let world = constellation(42, 12);
        let cfg = MarkerTrackerConfig { right_cx_offset: Some(OFFSET), ..Default::default() };
        let mut tracker = MarkerTracker::new(cfg);
        let mut last = Transform::default();
        for i in 0..3u64 {
            let truth = pose(0.0, 0.0, 0.03 * i as f32, Vec3::new(4.0 * i as f32, 0.0, 0.0));
            let mut frame = render(&world, &truth, i);
            if i == 2 {
                // A dot only the left camera sees: detected, never triangulated.
                if let Some(ir) = frame.ir.as_mut() {
                    draw_ellipse(&mut ir.left, 40.0, 40.0, 6.0, 6.0, 0.0, 250.0);
                }
            }
            last = tracker.estimate(&frame, &last).expect("hint").pose;
        }
        let r = tracker.report().clone();
        let s = tracker.frame_state().clone();
        assert_eq!(s.image_size, (640, 512));
        assert_eq!(s.detected(), r.detected_left);
        assert_eq!(s.triangulated(), r.triangulated);
        assert_eq!(s.tracking(), r.inliers);
        assert_eq!(s.matched_map.len(), r.inliers);
        assert!(s.detected() > s.triangulated(), "the extra left-only dot is detected but not triangulated");
        assert!(s.dots.iter().all(|d| !d.tracking || d.triangulated), "tracking dots are triangulated");
        let lone = s.dots.iter().find(|d| (d.x - 40.0).abs() < 1.0 && (d.y - 40.0).abs() < 1.0).expect("lone dot detected");
        assert!(!lone.triangulated && !lone.tracking);
        assert!(!s.stale);

        // The same IR pair again (the POP 4 repeats it on every other packet): same dots, stale.
        let truth = pose(0.0, 0.0, 0.06, Vec3::new(8.0, 0.0, 0.0));
        let frame = render(&world, &truth, 3);
        tracker.estimate(&frame, &last);
        let fresh = tracker.frame_state().clone();
        assert!(tracker.estimate(&frame, &last).is_none());
        let repeated = tracker.frame_state();
        assert!(repeated.stale);
        assert_eq!(repeated.dots, fresh.dots);
    }

    #[test]
    fn too_few_markers_give_no_hint() {
        let world = constellation(5, 2);
        let mut tracker = MarkerTracker::new(MarkerTrackerConfig { right_cx_offset: Some(OFFSET), ..Default::default() });
        assert!(tracker.estimate(&render(&world, &Transform::default(), 0), &Transform::default()).is_none());
        assert!(tracker.estimate(&SensorFrame::new(1, 0), &Transform::default()).is_none(), "frames without IR");
    }

    #[test]
    fn fixed_map_relocates_without_guess() {
        let world = constellation(8, 20);
        let mut map = MarkerMap::new();
        for p in &world {
            map.add(*p, 6.0);
        }
        let mut tracker = MarkerTracker::with_fixed_map(MarkerTrackerConfig { right_cx_offset: Some(OFFSET), ..Default::default() }, map);
        let truth = pose(0.1, -0.05, 0.3, Vec3::new(15.0, -10.0, 5.0));
        let hint = tracker.estimate(&render(&world, &truth, 0), &Transform::default()).expect("located");
        assert_eq!(hint.kind, HintKind::Fixed);
        assert!(hint.pose.translation.distance(truth.translation) < 0.5);
        assert_eq!(tracker.map().lock().unwrap().len(), 20, "fixed map does not grow");
    }
}
