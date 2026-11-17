//! Per-pass processing: sensor frame -> geometry (depth source) -> pose (tracking) -> fused model.
//!
//! A scan mode is a light source (which [`DepthSource`] turns frames into points) plus a tracking
//! mode (an optional [`PoseEstimator`] in front of the shared geometric [`Tracker`], which owns
//! keyframes, fusion, discard and re-location). See `docs/architecture/scan-sessions.md`.
//!
//! Adding a light source or tracking mode: implement the trait in its own crate/module and
//! return it from [`pipeline_for`].

pub mod color;
pub mod dots;
pub mod hybrid;
pub mod turntable;

use scancraft_core::{BackProjection, DepthMap, EdgeFilter, Point3D, PointCloud, SensorFrame, Transform};
use scancraft_driver::{LightSource, ScanConfig, TrackingMode, TurntableMode};
use scancraft_slam::{PoseHint, Tracker, TrackingState, TrackingStatus};
pub use turntable::{TurntableConfig, TurntableFilter, TurntableRegion};

/// What a depth source extracts from one frame, in the sensor frame (+X right, +Y forward, +Z up).
#[derive(Debug, Clone, Default)]
pub struct FrameGeometry {
    /// Points with normals facing the sensor. May be sparse (laser lines).
    pub cloud: PointCloud,
    /// The dense depth image the points came from, with intrinsics, if there is one. Enables the
    /// tracker's visibility check.
    pub depth: Option<DepthMap>,
}

/// Turns sensor frames into geometry: on-device depth, host stereo, laser-line triangulation.
pub trait DepthSource: Send {
    fn name(&self) -> &'static str;
    /// `None` if the frame yields no usable geometry.
    fn process(&mut self, frame: &SensorFrame) -> Option<FrameGeometry>;
    /// True if the sensor stays still while the object moves (turntable capture). The pipeline
    /// then gives the tracker no IMU motion prior: the IMU measures the scanner, not the object.
    fn sensor_static(&self) -> bool {
        false
    }
}

/// How an overlay point took part in this frame's tracking.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum OverlayKind {
    /// Found in the image only.
    Detected,
    /// Found and turned into 3D (e.g. a marker seen by both cameras).
    Triangulated,
    /// Used for the pose this frame.
    Tracking,
}

/// A feature in the camera image, px.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OverlayPoint {
    pub x: f32,
    pub y: f32,
    pub radius: f32,
    pub kind: OverlayKind,
}

/// What a pose estimator saw in the last frame, for drawing over the left IR image and in the 3D
/// view: features in the image, and its map in the pass frame.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct ImageOverlay {
    /// Size of the image the points are in (the left IR image), px.
    pub width: u32,
    pub height: u32,
    pub points: Vec<OverlayPoint>,
    /// The estimator's map (pass frame, mm) and whether each entry was matched this frame.
    pub map: Vec<([f32; 3], bool)>,
    /// Nothing new was observed this frame (the camera repeated the previous image).
    pub stale: bool,
}

impl ImageOverlay {
    pub fn count(&self, kind: OverlayKind) -> usize {
        self.points.iter().filter(|p| p.kind == kind).count()
    }
}

/// The overlay of a marker tracker's last frame.
pub fn marker_overlay(tracker: &scancraft_markers::MarkerTracker) -> ImageOverlay {
    let state = tracker.frame_state();
    let points = state
        .dots
        .iter()
        .map(|d| OverlayPoint {
            x: d.x,
            y: d.y,
            radius: d.radius,
            kind: if d.tracking {
                OverlayKind::Tracking
            } else if d.triangulated {
                OverlayKind::Triangulated
            } else {
                OverlayKind::Detected
            },
        })
        .collect();
    let map = tracker.map();
    let map = map.lock().map(|m| m.markers.iter().map(|mm| (mm.position, state.matched_map.contains(&mm.id))).collect()).unwrap_or_default();
    ImageOverlay { width: state.image_size.0, height: state.image_size.1, points, map, stale: state.stale }
}

/// Estimates the sensor pose from something other than geometry (markers, colour features).
pub trait PoseEstimator: Send {
    fn name(&self) -> &'static str;
    /// `last_pose` is the tracker's latest accepted pose (world <- sensor).
    fn estimate(&mut self, frame: &SensorFrame, geometry: &FrameGeometry, last_pose: &Transform) -> Option<PoseHint>;
    /// The tracker's verdict on the frame last passed to `estimate` (e.g. to keep it as a
    /// reference once its pose is final).
    fn observe(&mut self, _frame: &SensorFrame, _status: &TrackingStatus) {}
    /// True if the frame adds nothing for this estimator and should be skipped entirely (e.g. a
    /// repeated IR pair in marker tracking). Skipped frames are neither registered nor fused.
    fn repeats_previous(&self, _frame: &SensorFrame) -> bool {
        false
    }
    /// What the last frame looked like to the estimator, for the camera view. `None` if it has
    /// nothing to show.
    fn overlay(&self) -> Option<ImageOverlay> {
        None
    }
    /// True if a frame without a pose from this estimator must not be aligned by its geometry
    /// alone (sparse laser lines): it is then counted as lost.
    fn pose_required(&self) -> bool {
        false
    }
}

impl PoseEstimator for scancraft_markers::MarkerTracker {
    fn name(&self) -> &'static str {
        "markers"
    }

    fn estimate(&mut self, frame: &SensorFrame, _geometry: &FrameGeometry, last_pose: &Transform) -> Option<PoseHint> {
        scancraft_markers::MarkerTracker::estimate(self, frame, last_pose)
    }

    /// Repeated IR pairs carry no marker information; registering them by geometry alone fails
    /// on the dark, sparse surfaces typical around markers and made the state flicker
    /// Tracking/Lost on every other frame (replay of a real scan, 2026-11-01).
    fn repeats_previous(&self, frame: &SensorFrame) -> bool {
        self.is_repeated_ir(frame)
    }

    fn overlay(&self) -> Option<ImageOverlay> {
        Some(marker_overlay(self))
    }
}

/// Depth computed by the scanner itself (Full-field HD and the mock).
pub struct OnDeviceDepth {
    pub min_mm: f32,
    pub max_mm: f32,
    /// Flying-pixel filter at back-projection (`None` keeps every pixel in range).
    pub edge: Option<EdgeFilter>,
}

impl OnDeviceDepth {
    /// Depth range with the default flying-pixel filter.
    pub fn new(min_mm: f32, max_mm: f32) -> Self {
        Self { min_mm, max_mm, edge: Some(EdgeFilter::default()) }
    }
}

impl DepthSource for OnDeviceDepth {
    fn name(&self) -> &'static str {
        "on-device depth"
    }

    fn process(&mut self, frame: &SensorFrame) -> Option<FrameGeometry> {
        let depth = frame.depth.as_ref().filter(|d| d.intrinsics.is_some())?;
        let cloud = depth.to_point_cloud_with(&BackProjection { min_mm: self.min_mm, max_mm: self.max_mm, edge: self.edge });
        (!cloud.is_empty()).then(|| FrameGeometry { cloud, depth: Some(depth.clone()) })
    }
}

/// Blue laser lines triangulated on the host (`scancraft-laser`). Sparse points; tracking needs a
/// pose estimator (markers / colour) in front of the geometric tracker.
pub struct LaserLines(pub scancraft_laser::LaserLineDepth);

impl DepthSource for LaserLines {
    fn name(&self) -> &'static str {
        "laser lines"
    }

    fn process(&mut self, frame: &SensorFrame) -> Option<FrameGeometry> {
        let result = self.0.process(frame)?;
        (!result.cloud.is_empty()).then_some(FrameGeometry { cloud: result.cloud, depth: None })
    }
}

/// Depth from host stereo matching of the IR pair (VCSEL Rapid; reusable for Hybrid HD).
pub struct StereoMatchDepth {
    pub stereo: scancraft_stereo::StereoDepth,
}

impl StereoMatchDepth {
    pub fn new(min_mm: f32, max_mm: f32) -> Self {
        Self { stereo: scancraft_stereo::StereoDepth::new(scancraft_stereo::StereoDepthConfig::new(min_mm, max_mm)) }
    }
}

impl DepthSource for StereoMatchDepth {
    fn name(&self) -> &'static str {
        "stereo matching"
    }

    fn process(&mut self, frame: &SensorFrame) -> Option<FrameGeometry> {
        let depth = self.stereo.compute(frame.ir.as_ref()?)?;
        let cloud = depth.to_point_cloud(self.stereo.config.min_mm, self.stereo.config.max_mm);
        (!cloud.is_empty()).then(|| FrameGeometry { cloud, depth: Some(depth) })
    }
}

pub struct PassPipeline {
    pub depth: Box<dyn DepthSource>,
    pub pose: Option<Box<dyn PoseEstimator>>,
    /// Colour recording (colour passes and Color tracking): RGB keyframes + vertex colours.
    pub color: Option<scancraft_color::ColorRecorder>,
}

impl PassPipeline {
    /// Runs one frame through depth source, pose estimator and tracker. Returns the tracking
    /// status, or `None` if the frame had no usable geometry.
    pub fn process(&mut self, frame: &SensorFrame, tracker: &mut Tracker) -> Option<TrackingStatus> {
        if let Some(imu) = frame.imu.filter(|_| !self.depth.sensor_static()) {
            tracker.update_imu(imu);
        }
        if self.pose.as_ref().is_some_and(|p| p.repeats_previous(frame)) {
            return None;
        }
        let geometry = self.depth.process(frame)?;
        let hint = self.pose.as_mut().and_then(|p| p.estimate(frame, &geometry, &tracker.current_pose));
        let status = if hint.is_none() && self.pose.as_ref().is_some_and(|p| p.pose_required()) {
            tracker.skip_frame_as_lost()
        } else {
            tracker.register_with(&geometry.cloud, geometry.depth.as_ref(), hint)
        };
        if let Some(p) = self.pose.as_mut() {
            p.observe(frame, &status);
        }
        if let Some(c) = self.color.as_mut().filter(|_| matches!(status.state, TrackingState::Tracking | TrackingState::Slow)) {
            c.observe(frame, &status.current_pose);
        }
        Some(status)
    }

    /// Gives freshly fused points (`Tracker::take_new_points`) the colour of the frame that
    /// produced them, in colour passes. No-op otherwise.
    pub fn colorize(&self, points: &mut [Point3D]) {
        if let Some(c) = &self.color {
            c.colorize_new_points(points);
        }
    }

    /// True if the pose estimator reports an overlay (marker tracking).
    pub fn overlay_capable(&self) -> bool {
        self.pose.as_ref().is_some_and(|p| p.overlay().is_some())
    }

    /// The pose estimator's view of the last frame (markers), if it has one.
    pub fn overlay(&self) -> Option<ImageOverlay> {
        self.pose.as_ref().and_then(|p| p.overlay())
    }

    /// RGB keyframes recorded so far in this pass (empty without colour).
    pub fn rgb_keyframes(&self) -> &[scancraft_color::RgbKeyframe] {
        self.color.as_ref().map_or(&[], |c| c.keyframes())
    }
}

/// Settings of a pass pipeline that are not part of [`ScanConfig`].
#[derive(Debug, Clone, Default)]
pub struct PipelineOptions {
    /// Used when `ScanConfig::turntable_mode` is a turntable mode.
    pub turntable: TurntableConfig,
}

/// The pipeline for a scan configuration, or why there is none yet.
pub fn pipeline_for(config: &ScanConfig, min_mm: f32, max_mm: f32) -> Result<PassPipeline, String> {
    pipeline_with(config, min_mm, max_mm, &PipelineOptions::default())
}

/// VCSEL Rapid working range, mm: Revo Scan 6 sends `0x707 200 600` for it (capture 2026-11-01).
pub const VCSEL_RANGE_MM: (f32, f32) = (200.0, 600.0);

/// [`pipeline_for`] with explicit options.
pub fn pipeline_with(config: &ScanConfig, min_mm: f32, max_mm: f32, options: &PipelineOptions) -> Result<PassPipeline, String> {
    let depth: Box<dyn DepthSource> = match config.light {
        LightSource::FullFieldHd => Box::new(OnDeviceDepth::new(min_mm, max_mm)),
        LightSource::CrossLines => Box::new(LaserLines(scancraft_laser::LaserLineDepth::new(scancraft_laser::Pattern::Cross))),
        LightSource::SingleLine => Box::new(LaserLines(scancraft_laser::LaserLineDepth::new(scancraft_laser::Pattern::Single))),
        LightSource::VcselRapid => {
            // The stereo search covers the VCSEL working range (`VCSEL_RANGE_MM`) at most:
            // a wider window (the app's default near limit is 150 mm) doubles the matcher cost.
            let (lo, hi) = (min_mm.max(VCSEL_RANGE_MM.0), max_mm.min(VCSEL_RANGE_MM.1));
            let (lo, hi) = if lo + 20.0 <= hi { (lo, hi) } else { VCSEL_RANGE_MM };
            Box::new(StereoMatchDepth::new(lo, hi))
        }
        LightSource::HybridHd => Box::new(hybrid::HybridStereoDepth::new(min_mm, max_mm)),
        other => return Err(format!("no depth source for {} yet", other.label())),
    };
    // Turntable capture: only the object on the turntable is tracked and fused.
    let depth: Box<dyn DepthSource> = match config.turntable_mode {
        TurntableMode::HandheldFreeform => depth,
        TurntableMode::TurntableContinuous | TurntableMode::TurntableStep => {
            Box::new(TurntableFilter::new(depth, options.turntable.clone()))
        }
    };
    let pose: Option<Box<dyn PoseEstimator>> = match config.tracking {
        // Cross Lines + Color: the driver streams lines + dots; tracking (colour features and
        // geometry) runs on the dots, the model is fused from the lines (`dots.rs`).
        TrackingMode::Color if config.light == LightSource::CrossLines => {
            let (lo, hi) = (min_mm.max(VCSEL_RANGE_MM.0), max_mm.min(VCSEL_RANGE_MM.1));
            let (lo, hi) = if lo + 20.0 <= hi { (lo, hi) } else { VCSEL_RANGE_MM };
            Some(Box::new(dots::DotsPose::new(lo, hi, true)))
        }
        TrackingMode::Feature => None,
        // Marker tracking; Global Marker defaults to phase 1 (build + record), phase 2 is built by
        // the app from a saved map (`scancraft_markers::GlobalMarkerSession::phase2`).
        TrackingMode::Marker => Some(Box::new(scancraft_markers::MarkerTracker::new(Default::default()))),
        TrackingMode::GlobalMarker => Some(Box::new(scancraft_markers::GlobalMarkerSession::phase1(Default::default()))),
        TrackingMode::Color => Some(Box::new(color::ColorPoseEstimator::new(min_mm, max_mm))),
    };
    // Color tracking implies colour recording (docs/architecture/scan-sessions.md section 4).
    let color = (config.color_texture || config.tracking == TrackingMode::Color).then(scancraft_color::ColorRecorder::default);
    Ok(PassPipeline { depth, pose, color })
}

#[cfg(test)]
mod tests {
    use super::*;
    use scancraft_core::PinholeIntrinsics;

    #[test]
    fn full_field_feature_pipeline_tracks_on_device_depth() {
        let handheld = ScanConfig { turntable_mode: TurntableMode::HandheldFreeform, ..ScanConfig::default() };
        let mut pipeline = pipeline_for(&handheld, 150.0, 400.0).expect("default mode has a pipeline");
        let k = PinholeIntrinsics { fx: 80.0, fy: 80.0, cx: 32.0, cy: 24.0 };
        let mut data = vec![30000u16; 64 * 48];
        for v in 10..30 {
            for u in 20..44 {
                data[v * 64 + u] = 27000; // a box in front of a wall
            }
        }
        let mut frame = SensorFrame::new(1, 0);
        frame.depth = Some(DepthMap::new(64, 48, 0.01, data).with_intrinsics(k));

        let mut tracker = Tracker::new();
        let status = pipeline.process(&frame, &mut tracker).expect("geometry");
        assert_eq!(status.state, scancraft_slam::TrackingState::Tracking);
        assert!(tracker.fused_points().len() > 1000);

        // Frames without depth yield nothing (instead of an empty registration).
        assert!(pipeline.process(&SensorFrame::new(2, 66), &mut tracker).is_none());

        let vcsel = ScanConfig { light: LightSource::VcselRapid, ..ScanConfig::default() };
        assert_eq!(pipeline_for(&vcsel, 300.0, 800.0).expect("VCSEL has a pipeline").depth.name(), "stereo matching");

        let color = ScanConfig { tracking: TrackingMode::Color, ..ScanConfig::default() };
        let p = pipeline_for(&color, 150.0, 400.0).expect("Color tracking has a pipeline");
        assert!(p.pose.is_some() && p.color.is_some(), "Color tracking records colour too");
        let textured = ScanConfig { color_texture: true, ..ScanConfig::default() };
        let p = pipeline_for(&textured, 150.0, 400.0).unwrap();
        assert!(p.pose.is_none() && p.color.is_some() && p.rgb_keyframes().is_empty());

        let unsupported = ScanConfig { light: LightSource::ParallelLines, ..ScanConfig::default() };
        assert!(pipeline_for(&unsupported, 150.0, 400.0).is_err());
        // Laser lines have a depth source; their tracking (markers) comes from another module.
        let laser = ScanConfig { light: LightSource::CrossLines, ..ScanConfig::default() };
        assert_eq!(pipeline_for(&laser, 150.0, 400.0).unwrap().depth.name(), "laser lines");
        for tracking in [TrackingMode::Marker, TrackingMode::GlobalMarker] {
            let marker = ScanConfig { tracking, ..ScanConfig::default() };
            assert_eq!(pipeline_for(&marker, 150.0, 400.0).unwrap().pose.unwrap().name(), "markers");
        }
    }

    #[test]
    fn marker_overlay_classifies_dots_and_flags_matched_map_markers() {
        use scancraft_markers::{MarkerTracker, MarkerTrackerConfig};
        let mut tracker = MarkerTracker::new(MarkerTrackerConfig::default());
        assert_eq!(tracker.overlay().map(|o| o.points.len()), Some(0), "nothing seen yet");
        // A frame without an IR pair gives no observation; the overlay stays empty.
        assert!(PoseEstimator::estimate(&mut tracker, &SensorFrame::new(1, 0), &FrameGeometry::default(), &Transform::default()).is_none());
        let o = tracker.overlay().unwrap();
        assert!(o.points.is_empty() && o.map.is_empty());
        let feature = pipeline_for(&ScanConfig::default(), 150.0, 400.0).unwrap();
        assert!(feature.overlay().is_none(), "geometric tracking has no overlay");
        let marker = pipeline_for(&ScanConfig { tracking: TrackingMode::Marker, ..ScanConfig::default() }, 150.0, 400.0).unwrap();
        assert!(marker.overlay().is_some());

        let overlay = ImageOverlay {
            width: 640,
            height: 512,
            points: vec![
                OverlayPoint { x: 1.0, y: 1.0, radius: 3.0, kind: OverlayKind::Tracking },
                OverlayPoint { x: 2.0, y: 1.0, radius: 3.0, kind: OverlayKind::Triangulated },
                OverlayPoint { x: 3.0, y: 1.0, radius: 3.0, kind: OverlayKind::Detected },
                OverlayPoint { x: 4.0, y: 1.0, radius: 3.0, kind: OverlayKind::Tracking },
            ],
            map: vec![],
            stale: false,
        };
        assert_eq!(overlay.count(OverlayKind::Tracking), 2);
        assert_eq!(overlay.count(OverlayKind::Detected), 1);
    }

    #[test]
    fn turntable_modes_filter_the_depth_source_and_drop_the_imu_prior() {
        let handheld = pipeline_for(&ScanConfig { turntable_mode: TurntableMode::HandheldFreeform, ..ScanConfig::default() }, 150.0, 400.0).unwrap();
        assert_eq!(handheld.depth.name(), "on-device depth");
        assert!(!handheld.depth.sensor_static());
        for mode in [TurntableMode::TurntableContinuous, TurntableMode::TurntableStep] {
            let p = pipeline_for(&ScanConfig { turntable_mode: mode, ..ScanConfig::default() }, 150.0, 400.0).unwrap();
            assert_eq!(p.depth.name(), "turntable");
            assert!(p.depth.sensor_static());
        }
    }
}
