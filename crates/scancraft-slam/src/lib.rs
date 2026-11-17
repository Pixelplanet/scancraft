//! Frame-to-model tracking and fusion for handheld / turntable scanning.
//!
//! Each depth frame (sensor-frame points with normals) is subsampled and aligned to the
//! accumulated world model with robust point-to-plane ICP, seeded by a constant-velocity
//! prediction. Frames that align well are fused into two voxel maps: a coarse one that serves
//! as the registration target and a fine one whose newly occupied voxels are handed to the
//! caller as fresh surface. Frames that fail are not fused, so a bad frame cannot smear the model.
//!
//! Noise handling (docs/architecture/scan-sessions.md section 6): flying pixels at depth
//! discontinuities are dropped at back-projection, output surfels need confidence or neighbour
//! support ([`surface`]), and keyframes carve away model points they see through ([`carve`]).

pub mod carve;
mod fusion;
pub mod icp;
pub mod kabsch;
pub mod point_to_plane;
pub mod surface;
pub mod voxel_map;

use glam::{Quat, Vec3};
pub use icp::{Icp, IcpConfig, IcpResult};
use point_to_plane::{align, PlaneTarget};
pub use carve::FreeSpaceView;
use scancraft_core::{BackProjection, DepthMap, EdgeFilter, ImuSample, PinholeIntrinsics, Point3D, PointCloud, Transform};
pub use surface::{SurfaceFilter, SurfaceModel, SurfaceUpdate};
pub use voxel_map::{Surfel, VoxelMap};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TrackingState {
    Idle,
    Tracking,
    /// Aligned, but the scanner moved a lot between frames; slow down to stay locked.
    Slow,
    /// The last frame could not be aligned; it was not fused.
    Lost,
}

#[derive(Debug, Clone)]
pub struct TrackingStatus {
    pub state: TrackingState,
    pub current_pose: Transform,
    pub inliers: usize,
    pub rmse_mm: f32,
    /// Surface voxels in the fused model.
    pub total_points_fused: usize,
    /// Of the model points that should be visible from the estimated pose and have a depth
    /// measurement, the fraction lying clearly *in front of* the measured surface. A correct pose
    /// gives only a few percent (silhouette edges); a wrong one sees "through" known surfaces.
    pub visibility_conflict: f32,
    /// Id of the keyframe this frame was stored as, if it was.
    pub keyframe: Option<u64>,
    /// The frame was placed by re-locating against stored keyframes after tracking was lost.
    pub relocalized: bool,
}

/// A pose for the next frame from another source (marker or colour tracking).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PoseHint {
    /// Sensor pose in the world frame (world <- sensor).
    pub pose: Transform,
    pub kind: HintKind,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HintKind {
    /// A good starting point: tried first, then refined by ICP and validated as usual.
    Prior,
    /// Trusted as is (e.g. solved from markers): no ICP refinement, the frame is fused at this
    /// pose. On the first frame it also defines the world frame.
    Fixed,
}

/// An accepted frame kept with its pose, so the model can be rebuilt without frames the user
/// discards. Points are stored in the sensor frame, thinned to the fusion pitch.
#[derive(Debug, Clone)]
pub struct Keyframe {
    pub id: u64,
    pub pose: Transform,
    pub rmse_mm: f32,
    pub visibility_conflict: f32,
    /// Surface voxels this frame added to the model when it was fused.
    pub new_points: usize,
    pub discarded: bool,
    imu: Option<Quat>,
    centroid: Vec3,
    points: Vec<PackedPoint>,
    /// Free space this frame observed, so a rebuild carves exactly as live fusion did.
    view: Option<FreeSpaceView>,
}

#[derive(Debug, Clone, Copy)]
struct PackedPoint {
    p: [f32; 3],
    n: [i8; 3],
}

impl PackedPoint {
    fn new(p: Vec3, n: Vec3) -> Self {
        let q = |v: f32| (v.clamp(-1.0, 1.0) * 127.0).round() as i8;
        Self { p: p.to_array(), n: [q(n.x), q(n.y), q(n.z)] }
    }

    fn unpack(&self) -> (Vec3, Vec3) {
        (Vec3::from(self.p), Vec3::new(self.n[0] as f32, self.n[1] as f32, self.n[2] as f32).normalize_or_zero())
    }
}

impl Keyframe {
    pub fn point_count(&self) -> usize {
        self.points.len()
    }

    /// The frame's points placed in the world at its pose (e.g. to highlight it).
    pub fn world_points(&self) -> Vec<Point3D> {
        self.points
            .iter()
            .map(|pp| {
                let (p, n) = pp.unpack();
                Point3D::new(self.pose.transform_point3(p).to_array()).with_normal((self.pose.rotation * n).to_array())
            })
            .collect()
    }

    fn oriented(&self) -> impl Iterator<Item = (Vec3, Vec3)> + '_ {
        self.points.iter().map(PackedPoint::unpack)
    }

    /// The frame's points and normals in the sensor frame.
    pub fn sensor_points(&self) -> Vec<(Vec3, Vec3)> {
        self.oriented().collect()
    }

    /// Mean of the frame's registration sample, sensor frame.
    pub fn centroid(&self) -> Vec3 {
        self.centroid
    }

    /// IMU orientation when the frame was stored.
    pub fn imu(&self) -> Option<Quat> {
        self.imu
    }

    /// Everything stored about the frame, as plain data (for saving a pass).
    pub fn to_data(&self) -> KeyframeData {
        KeyframeData {
            id: self.id,
            pose: self.pose,
            rmse_mm: self.rmse_mm,
            visibility_conflict: self.visibility_conflict,
            new_points: self.new_points,
            discarded: self.discarded,
            imu: self.imu,
            centroid: self.centroid,
            points: self.points.iter().map(|p| (p.p, p.n)).collect(),
        }
    }

    /// A keyframe from saved data (see [`Tracker::from_keyframes`]).
    pub fn from_data(d: KeyframeData) -> Self {
        Keyframe {
            id: d.id,
            pose: d.pose,
            rmse_mm: d.rmse_mm,
            visibility_conflict: d.visibility_conflict,
            new_points: d.new_points,
            discarded: d.discarded,
            imu: d.imu,
            centroid: d.centroid,
            points: d.points.into_iter().map(|(p, n)| PackedPoint { p, n }).collect(),
            // Free-space views are not persisted; reloaded keyframes don't carve.
            view: None,
        }
    }
}

/// Plain data of a [`Keyframe`], for persistence. Points are in the sensor frame; normals are
/// quantised to `i8` (unit vector x 127).
#[derive(Debug, Clone, PartialEq)]
pub struct KeyframeData {
    pub id: u64,
    pub pose: Transform,
    pub rmse_mm: f32,
    pub visibility_conflict: f32,
    pub new_points: usize,
    pub discarded: bool,
    pub imu: Option<Quat>,
    pub centroid: Vec3,
    pub points: Vec<([f32; 3], [i8; 3])>,
}

/// A stored view of an existing model a new pass can start from (see [`Tracker::seed_model`]):
/// a keyframe pose in the model's frame, its IMU orientation, and the centroid of what it saw
/// (sensor frame).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SeedView {
    pub pose: Transform,
    pub imu: Option<Quat>,
    pub centroid: Vec3,
}

/// Whether a tracker started from a seeded model has registered into it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SeedState {
    /// Not seeded: the first frame defines the world frame.
    None,
    /// Seeded; the first frames are being located in the seed's frame. `attempts` frames failed.
    Pending { attempts: usize },
    /// Located: the tracker's world frame is the seed's frame.
    Registered,
    /// No frame could be located in time; the seed was dropped and the pass has its own frame.
    Abandoned,
}

struct Seed {
    points: Vec<(Vec3, Vec3)>,
    views: Vec<SeedView>,
}

#[derive(Debug, Clone)]
pub struct TrackerConfig {
    /// Grid used to subsample frames and to build the registration model.
    pub registration_voxel_mm: f32,
    /// Grid of the fused output surface.
    pub fusion_voxel_mm: f32,
    /// Coarse-to-fine correspondence gates for ICP.
    pub gates_mm: Vec<f32>,
    pub iterations_per_gate: usize,
    /// A frame is accepted when at least this fraction of its points find the model...
    pub min_inlier_ratio: f32,
    /// ...with at most this RMS point-to-plane residual.
    pub max_rmse_mm: f32,
    /// Inter-frame motion above which the state is reported as `Slow`.
    pub slow_translation_mm: f32,
    pub slow_rotation_rad: f32,
    /// Largest accepted `visibility_conflict` (only checked when a depth map is supplied).
    pub max_visibility_conflict: f32,
    /// An accepted frame becomes a keyframe (stored and fused) once the scanner has moved this
    /// far from the previous keyframe.
    pub keyframe_translation_mm: f32,
    pub keyframe_rotation_rad: f32,
    /// Keyframes tried when re-locating: those whose IMU orientation is within this angle.
    pub relocalize_max_angle_rad: f32,
    pub relocalize_candidates: usize,
    /// Stricter acceptance for re-located poses: a wrong re-location duplicates the object.
    pub relocalize_min_inlier_ratio: f32,
    pub relocalize_max_visibility_conflict: f32,
    /// Largest deviation of a re-located rotation from what the keyframe + IMU predict.
    pub relocalize_max_rotation_error_rad: f32,
    /// Rotation taking IMU-body axes to sensor axes. `None` disables the IMU motion prior.
    pub imu_to_sensor: Option<Quat>,
    /// Quaternion convention of the IMU: `true` if it maps body to world (the usual one).
    pub imu_body_to_world: bool,
    /// A seeded tracker gives up locating itself in the seed after this many frames (about three
    /// seconds at 15 fps) and continues in a frame of its own.
    pub seed_max_frames: usize,
    pub noise: NoiseConfig,
}

/// Noise suppression (docs/architecture/scan-sessions.md section 6).
#[derive(Debug, Clone, PartialEq)]
pub struct NoiseConfig {
    /// Flying-pixel filter used by [`Tracker::register_depth`] (`None` keeps every pixel).
    pub edge_filter: Option<EdgeFilter>,
    /// Which fused surfels are output.
    pub surface: SurfaceFilter,
    /// Free-space carving with each keyframe's depth image (needs depth with intrinsics).
    pub carve: bool,
    /// A model point is seen through when it lies this far in front of the measured surface...
    pub carve_margin_mm: f32,
    /// ...plus this fraction of the measured depth.
    pub carve_margin_per_mm: f32,
    /// Depth pixels per free-space cell (resolution of the stored free-space views).
    pub carve_block_px: u32,
}

impl Default for NoiseConfig {
    fn default() -> Self {
        Self {
            edge_filter: Some(EdgeFilter::default()),
            surface: SurfaceFilter::default(),
            carve: true,
            carve_margin_mm: 3.0,
            carve_margin_per_mm: 0.005,
            carve_block_px: 4,
        }
    }
}

impl NoiseConfig {
    /// Everything off: every in-range pixel is fused and output (the behaviour before noise
    /// handling, for comparisons).
    pub fn off() -> Self {
        Self { edge_filter: None, surface: SurfaceFilter { enabled: false, ..SurfaceFilter::default() }, carve: false, ..Self::default() }
    }
}

impl Default for TrackerConfig {
    fn default() -> Self {
        Self {
            registration_voxel_mm: 2.5,
            fusion_voxel_mm: 0.5,
            gates_mm: vec![30.0, 15.0, 8.0, 4.0, 2.0],
            iterations_per_gate: 6,
            min_inlier_ratio: 0.4,
            max_rmse_mm: 1.0,
            slow_translation_mm: 25.0,
            slow_rotation_rad: 0.15,
            max_visibility_conflict: 0.12,
            keyframe_translation_mm: 3.0,
            keyframe_rotation_rad: 0.035,
            relocalize_max_angle_rad: 0.55,
            relocalize_candidates: 4,
            relocalize_min_inlier_ratio: 0.5,
            relocalize_max_visibility_conflict: 0.08,
            relocalize_max_rotation_error_rad: 0.17,
            imu_to_sensor: None,
            imu_body_to_world: true,
            seed_max_frames: 45,
            noise: NoiseConfig::default(),
        }
    }
}

pub struct Tracker {
    pub config: TrackerConfig,
    pub current_pose: Transform,
    pub state: TrackingState,
    /// Pose change between the last two accepted frames (world frame), for prediction.
    velocity: Transform,
    registration_model: VoxelMap,
    /// The display surface, merged on a background thread (`fusion.rs`).
    fused: fusion::Fusion,
    lost_frames: usize,
    latest_imu: Option<ImuSample>,
    /// IMU orientation when `current_pose` was last updated.
    imu_at_pose: Option<Quat>,
    keyframes: Vec<Keyframe>,
    next_keyframe_id: u64,
    /// After a re-location, the last confirmed pose and IMU orientation to fall back to if the
    /// next frame does not confirm it. Nothing is fused while this is set.
    unconfirmed: Option<(Transform, Option<Quat>)>,
    /// Pose hint for the frame being registered (set by `register_with`).
    hint: Option<PoseHint>,
    /// Model of earlier passes this pass registers into (see [`seed_model`](Self::seed_model)).
    seed: Option<Seed>,
    seed_state: SeedState,
}

impl Default for Tracker {
    fn default() -> Self {
        Self::new()
    }
}

impl Tracker {
    pub fn new() -> Self {
        Self::with_config(TrackerConfig::default())
    }

    pub fn with_config(config: TrackerConfig) -> Self {
        Self {
            registration_model: VoxelMap::new(config.registration_voxel_mm),
            fused: fusion::Fusion::new(SurfaceModel::new(config.fusion_voxel_mm, config.noise.surface.clone())),
            config,
            current_pose: Transform::default(),
            state: TrackingState::Idle,
            velocity: Transform::default(),
            lost_frames: 0,
            latest_imu: None,
            imu_at_pose: None,
            keyframes: Vec::new(),
            next_keyframe_id: 0,
            unconfirmed: None,
            hint: None,
            seed: None,
            seed_state: SeedState::None,
        }
    }

    /// A tracker holding saved keyframes (e.g. a pass loaded from a project), with its model
    /// rebuilt from the kept ones. Capture can continue from the last kept keyframe.
    pub fn from_keyframes(config: TrackerConfig, keyframes: Vec<Keyframe>) -> Self {
        let mut t = Self::with_config(config);
        t.next_keyframe_id = keyframes.iter().map(|k| k.id + 1).max().unwrap_or(0);
        t.keyframes = keyframes;
        t.rebuild();
        t
    }

    /// Starts this (empty) tracker inside an existing model: `points` (with normals) and `views`
    /// are in the model's frame, e.g. the project frame built from earlier passes. The first
    /// frames are located against the model from the stored views; once one is, the tracker's
    /// world frame is the model's frame and tracking continues as usual (the seed stays in the
    /// registration target but is not part of [`fused_points`](Self::fused_points)). If no frame
    /// can be located within `config.seed_max_frames` frames, the seed is dropped and the next
    /// frame defines a world frame of its own ([`SeedState::Abandoned`]).
    ///
    /// Returns false (and does nothing) if the tracker already holds keyframes or `points` is
    /// empty.
    pub fn seed_model(&mut self, points: &[(Vec3, Vec3)], views: &[SeedView]) -> bool {
        if !self.keyframes.is_empty() || points.is_empty() {
            return false;
        }
        let sample = VoxelMap::from_points(self.config.registration_voxel_mm, points.iter().copied().filter(|(_, n)| n.length_squared() > 0.5));
        let points: Vec<(Vec3, Vec3)> = sample.surfels().iter().map(|s| (s.position, s.normal)).collect();
        if points.is_empty() {
            return false;
        }
        self.registration_model.clear();
        for &(p, n) in &points {
            self.registration_model.insert(p, n);
        }
        self.seed = Some(Seed { points, views: views.to_vec() });
        self.seed_state = SeedState::Pending { attempts: 0 };
        self.state = TrackingState::Lost;
        true
    }

    pub fn seed_state(&self) -> SeedState {
        self.seed_state
    }

    /// Stored frames in capture order (discarded ones included, flagged).
    pub fn keyframes(&self) -> &[Keyframe] {
        &self.keyframes
    }

    /// Discards or restores one keyframe and rebuilds the model. Returns false for unknown ids.
    pub fn set_discarded(&mut self, id: u64, discarded: bool) -> bool {
        let Some(kf) = self.keyframes.iter_mut().find(|k| k.id == id) else { return false };
        kf.discarded = discarded;
        self.rebuild();
        true
    }

    /// Applies several discard/restore flags with a single rebuild (e.g. undoing a range discard).
    pub fn set_discarded_many(&mut self, flags: &[(u64, bool)]) {
        let mut changed = false;
        for &(id, discarded) in flags {
            if let Some(kf) = self.keyframes.iter_mut().find(|k| k.id == id && k.discarded != discarded) {
                kf.discarded = discarded;
                changed = true;
            }
        }
        if changed {
            self.rebuild();
        }
    }

    /// Discards `id` and every keyframe after it (e.g. after the scan drifted), then rebuilds.
    pub fn discard_from(&mut self, id: u64) {
        for kf in self.keyframes.iter_mut().filter(|k| k.id >= id) {
            kf.discarded = true;
        }
        self.rebuild();
    }

    /// Re-fuses all kept keyframes at their stored poses. Pending new points are dropped and
    /// [`model_revision`](Self::model_revision) increases; callers should replace their copy of the
    /// model with [`fused_points`](Self::fused_points) (or use [`take_update`](Self::take_update)).
    fn rebuild(&mut self) {
        self.registration_model.clear();
        if let Some(seed) = &self.seed {
            for &(p, n) in &seed.points {
                self.registration_model.insert(p, n);
            }
        }
        let jobs: Vec<(Vec<(Vec3, Vec3)>, fusion::FuseJob)> = self
            .keyframes
            .iter()
            .filter(|k| !k.discarded)
            .map(|kf| {
                let sample: Vec<(Vec3, Vec3)> = VoxelMap::from_points(self.config.registration_voxel_mm, kf.oriented())
                    .surfels()
                    .iter()
                    .map(|s| (s.position, s.normal))
                    .collect();
                let points: Vec<(Vec3, Vec3)> = kf.oriented().collect();
                (sample, self.fuse_job(&kf.pose, &points, kf.view.clone()))
            })
            .collect();
        // Synchronous, after any pending background merges, so the result is deterministic.
        for (sample, job) in &jobs {
            self.registration_model_add(&job.pose, sample, job.view.as_ref());
        }
        let mut model = self.fused.lock();
        model.clear();
        for (_, job) in &jobs {
            fusion::fuse_into(&mut model, job);
        }
        model.refresh();
        self.fused.mark_seen(&mut model);
        let visible = model.len();
        drop(model);
        self.fused.set_visible(visible);
        // Continue from the last kept keyframe; if none are left, start over with the next frame.
        match self.keyframes.iter().rev().find(|k| !k.discarded) {
            Some(kf) => {
                self.current_pose = kf.pose;
                self.imu_at_pose = kf.imu;
                self.velocity = Transform::default();
                self.state = TrackingState::Lost;
            }
            None => {
                self.current_pose = Transform::default();
                self.state = TrackingState::Idle;
                // Nothing of this pass is left: locate the next frame in the seed again.
                if self.seed.is_some() {
                    self.seed_state = SeedState::Pending { attempts: 0 };
                    self.state = TrackingState::Lost;
                }
            }
        }
        self.unconfirmed = None;
    }

    /// Records the latest IMU sample; with `imu_to_sensor` configured, the orientation change
    /// since the last accepted frame seeds the next alignment.
    pub fn update_imu(&mut self, imu: ImuSample) {
        if imu.orientation.is_finite() && (imu.orientation.length() - 1.0).abs() < 0.1 {
            self.latest_imu = Some(imu);
        }
    }

    /// Sensor-frame rotation since the last accepted frame, according to the IMU.
    fn imu_rotation_since_pose(&self) -> Option<Quat> {
        let x = self.config.imu_to_sensor?;
        let (q0, q1) = (self.imu_at_pose?, self.latest_imu?.orientation.normalize());
        let body = if self.config.imu_body_to_world { q0.inverse() * q1 } else { q0 * q1.inverse() };
        Some((x * body * x.inverse()).normalize())
    }

    /// The fused surface (world frame): one point per occupied fusion voxel that passes the noise
    /// filter (confident, or supported by its neighbours, and not carved away).
    pub fn fused_points(&self) -> Vec<Point3D> {
        self.fused.lock().points()
    }

    /// Waits until every keyframe handed to the background fusion is merged into the surface.
    pub fn flush(&self) {
        self.fused.flush();
    }

    /// The fused surface with each surfel's observation count (world frame), e.g. for merging
    /// passes with a confidence filter. Carved and filtered surfels are not included.
    pub fn fused_surfels(&self) -> Vec<Surfel> {
        self.fused.lock().output_surfels()
    }

    /// Surface that became part of [`fused_points`](Self::fused_points) since the previous call
    /// (world frame). Holding the scanner still over already-scanned surface yields nothing.
    ///
    /// Points handed out here can later be withdrawn (carved as noise, or by a keyframe discard);
    /// [`model_revision`](Self::model_revision) increases when that happens. Consumers keeping a
    /// copy should then replace it with `fused_points()`, or simply use
    /// [`take_update`](Self::take_update), which does the bookkeeping.
    ///
    /// Never blocks: points still being merged in the background arrive with a later call
    /// (use [`flush`](Self::flush) first to get everything).
    pub fn take_new_points(&mut self) -> Vec<Point3D> {
        self.fused.take_new()
    }

    /// Increases whenever points already handed out by [`take_new_points`](Self::take_new_points)
    /// were removed from the model (noise carving, keyframe discard / restore).
    pub fn model_revision(&self) -> u64 {
        self.fused.lock().revision()
    }

    /// The change to apply to a consumer's copy of the model since the previous call: new points
    /// to append, or (after a removal) the full model to replace the copy with.
    ///
    /// Only waits for the background merge when a full replacement is due (rare).
    pub fn take_update(&mut self) -> SurfaceUpdate {
        if self.fused.withdrawn_since_seen() {
            let mut model = self.fused.lock();
            self.fused.mark_seen(&mut model);
            SurfaceUpdate::Replace(model.points())
        } else {
            SurfaceUpdate::Append(self.fused.take_new())
        }
    }

    /// The fused surface model (surfel confidences, stored vs. output counts).
    ///
    /// Holds the fusion lock: drop the guard before calling other tracker methods that read the
    /// surface (`fused_points`, `model_revision`, ...), or they wait for it forever.
    pub fn surface(&self) -> std::sync::MutexGuard<'_, SurfaceModel> {
        self.fused.lock()
    }

    /// Registers a depth frame. Preferred over [`register_frame`](Self::register_frame): with the
    /// depth image the tracker can also reject poses that contradict what the sensor sees.
    /// Flying pixels are dropped as configured in [`NoiseConfig::edge_filter`].
    pub fn register_depth(&mut self, depth: &DepthMap, min_mm: f32, max_mm: f32) -> TrackingStatus {
        let cloud = depth.to_point_cloud_with(&BackProjection { min_mm, max_mm, edge: self.config.noise.edge_filter });
        self.register(&cloud, depth.intrinsics.map(|k| (depth, k)))
    }

    /// Registers a sensor-frame point cloud (with normals) without the visibility check.
    pub fn register_frame(&mut self, frame: &PointCloud) -> TrackingStatus {
        self.register(frame, None)
    }

    /// General entry point for any depth source: sensor-frame points with normals, the depth image
    /// they came from if there is one (enables the visibility check), and an optional pose hint
    /// from a marker / colour pose estimator.
    pub fn register_with(&mut self, frame: &PointCloud, depth: Option<&DepthMap>, hint: Option<PoseHint>) -> TrackingStatus {
        self.hint = hint;
        let status = self.register(frame, depth.and_then(|d| d.intrinsics.map(|k| (d, k))));
        self.hint = None;
        status
    }

    /// Counts a frame as lost without trying to align it: for frames whose pose source (dot
    /// pattern, markers) could not place them and whose own geometry is too sparse to be aligned
    /// safely. Nothing is fused.
    pub fn skip_frame_as_lost(&mut self) -> TrackingStatus {
        self.lost_frames += 1;
        self.state = TrackingState::Lost;
        self.velocity = Transform::default();
        self.status(0, f32::INFINITY, 0.0, None, false)
    }

    fn register(&mut self, frame: &PointCloud, depth: Option<(&DepthMap, PinholeIntrinsics)>) -> TrackingStatus {
        let oriented: Vec<(Vec3, Vec3)> = frame
            .points
            .iter()
            .map(|p| (Vec3::from(p.position), Vec3::from(p.normal)))
            .filter(|(_, n)| n.length_squared() > 0.5)
            .collect();
        let sample = VoxelMap::from_points(self.config.registration_voxel_mm, oriented.iter().copied());
        let source: Vec<(Vec3, Vec3)> = sample.surfels().iter().map(|s| (s.position, s.normal)).collect();

        if source.len() < 50 {
            return self.status(0, 0.0, 0.0, None, false);
        }

        if let Some(status) = self.seeded_start(&oriented, &source, depth) {
            return status;
        }

        if self.registration_model.is_empty() {
            if let Some((d, k)) = depth {
                // Pixel footprint at the typical depth of the first frame.
                let mut sample: Vec<u16> = d.data.iter().step_by(97).copied().filter(|&r| r != 0).collect();
                if !sample.is_empty() {
                    let mid = sample.len() / 2;
                    let median = *sample.select_nth_unstable(mid).1 as f32 * d.scale_mm;
                    self.fused.lock().set_point_spacing(median / k.fx.max(1e-3));
                }
            }
            self.current_pose = self.hint.map(|h| h.pose).unwrap_or_default();
            self.velocity = Transform::default();
            self.imu_at_pose = self.latest_imu.map(|s| s.orientation.normalize());
            let id = self.store_keyframe(&oriented, &source, 0.0, 0.0, depth.map(|(d, _)| d));
            self.state = TrackingState::Tracking;
            return self.status(source.len(), 0.0, 0.0, Some(id), false);
        }

        let target = PlaneTarget::new(
            self.registration_model.surfels().iter().map(|s| s.position).collect(),
            self.registration_model.surfels().iter().map(|s| s.normal).collect(),
        );

        // Candidate starting poses, best first: the IMU rotation applied as an orbit around the
        // observed surface (the object stays where it was in view), constant velocity, and the
        // last pose. If none aligns, re-locate against stored keyframes.
        if let Some(PoseHint { pose, kind: HintKind::Fixed }) = self.hint {
            let conflict = depth.map_or(0.0, |(d, k)| self.visibility_conflict(&pose, d, k));
            let step = compose(&pose, &self.current_pose.inverse());
            self.velocity = step;
            self.current_pose = pose;
            self.imu_at_pose = self.latest_imu.map(|s| s.orientation.normalize());
            self.unconfirmed = None;
            self.lost_frames = 0;
            self.state = TrackingState::Tracking;
            let keyframe = self.is_new_keyframe().then(|| self.store_keyframe(&oriented, &source, 0.0, conflict, depth.map(|(d, _)| d)));
            return self.status(source.len(), 0.0, conflict, keyframe, false);
        }

        let mut guesses = Vec::with_capacity(4);
        if let Some(PoseHint { pose, kind: HintKind::Prior }) = self.hint {
            guesses.push(pose);
        }
        if let Some(delta) = self.imu_rotation_since_pose() {
            let pivot = source.iter().map(|(p, _)| *p).sum::<Vec3>() / source.len() as f32;
            let rotation = (self.current_pose.rotation * delta).normalize();
            let translation = self.current_pose.transform_point3(pivot) - rotation * pivot;
            guesses.push(Transform { rotation, translation, scale: Vec3::ONE });
        }
        if self.state != TrackingState::Lost {
            guesses.push(compose(&self.velocity, &self.current_pose));
        }
        guesses.push(self.current_pose);

        let mut best: Option<(point_to_plane::AlignResult, f32)> = None;
        let mut keep_best = |r: point_to_plane::AlignResult, c: f32| {
            if best.as_ref().is_none_or(|(b, _)| r.inliers > b.inliers) {
                best = Some((r, c));
            }
        };
        let mut accepted: Option<(point_to_plane::AlignResult, f32, bool)> = None;
        for guess in guesses {
            let r = align(&source, &target, guess, &self.config.gates_mm, self.config.iterations_per_gate);
            let c = depth.map_or(0.0, |(d, k)| self.visibility_conflict(&r.pose, d, k));
            if self.accepted(r.inlier_ratio, r.rmse_mm) && c <= self.config.max_visibility_conflict && self.imu_consistent(&r.pose) {
                accepted = Some((r, c, false));
                break;
            }
            keep_best(r, c);
        }
        // Re-location is only tried from a settled state, not to confirm a previous re-location.
        // Re-locating tries several keyframes (each a full alignment) and can take longer than a
        // frame interval; while lost, try it on every third frame only so the app keeps up.
        let relocate_now = self.state != TrackingState::Lost || self.lost_frames % 3 == 0;
        if accepted.is_none() && self.unconfirmed.is_none() && relocate_now {
            for (guess, imu_predicted) in self.relocalization_guesses(&source) {
                let r = align(&source, &target, guess, &self.config.gates_mm, self.config.iterations_per_gate);
                let c = depth.map_or(0.0, |(d, k)| self.visibility_conflict(&r.pose, d, k));
                let rotation_ok = !imu_predicted
                    || r.pose.rotation.angle_between(guess.rotation) <= self.config.relocalize_max_rotation_error_rad;
                if r.inlier_ratio >= self.config.relocalize_min_inlier_ratio
                    && r.rmse_mm <= self.config.max_rmse_mm
                    && c <= self.config.relocalize_max_visibility_conflict
                    && rotation_ok
                {
                    accepted = Some((r, c, true));
                    break;
                }
                keep_best(r, c);
            }
        }

        let Some((result, conflict, relocalized)) = accepted else {
            // An unconfirmed re-location that the next frame cannot follow was probably wrong.
            if let Some((pose, imu)) = self.unconfirmed.take() {
                self.current_pose = pose;
                self.imu_at_pose = imu;
            }
            self.lost_frames += 1;
            self.state = TrackingState::Lost;
            self.velocity = Transform::default();
            let (inliers, rmse, conflict) = best.map_or((0, f32::INFINITY, 0.0), |(r, c)| (r.inliers, r.rmse_mm, c));
            return self.status(inliers, rmse, conflict, None, false);
        };

        if relocalized {
            // Take the pose, but fuse nothing until the next frame confirms it.
            self.unconfirmed = Some((self.current_pose, self.imu_at_pose));
            self.current_pose = result.pose;
            self.imu_at_pose = self.latest_imu.map(|s| s.orientation.normalize());
            self.velocity = Transform::default();
            self.state = TrackingState::Lost;
            return self.status(result.inliers, result.rmse_mm, conflict, None, true);
        }

        let step = compose(&result.pose, &self.current_pose.inverse());
        self.velocity = if self.unconfirmed.take().is_some() { Transform::default() } else { step };
        self.current_pose = result.pose;
        self.imu_at_pose = self.latest_imu.map(|s| s.orientation.normalize());
        self.lost_frames = 0;
        self.state = if step.translation.length() > self.config.slow_translation_mm
            || step.rotation.angle_between(Quat::IDENTITY) > self.config.slow_rotation_rad
        {
            TrackingState::Slow
        } else {
            TrackingState::Tracking
        };
        let keyframe =
            self.is_new_keyframe().then(|| self.store_keyframe(&oriented, &source, result.rmse_mm, conflict, depth.map(|(d, _)| d)));
        self.status(result.inliers, result.rmse_mm, conflict, keyframe, false)
    }

    /// While a seed is pending: tries to locate the frame in the seed from its stored views.
    /// Returns `None` when there is nothing to do (not seeded, or the seed was just abandoned and
    /// the frame should start a frame of its own).
    fn seeded_start(&mut self, oriented: &[(Vec3, Vec3)], source: &[(Vec3, Vec3)], depth: Option<(&DepthMap, PinholeIntrinsics)>) -> Option<TrackingStatus> {
        let SeedState::Pending { attempts } = self.seed_state else { return None };
        if !self.keyframes.iter().all(|k| k.discarded) {
            return None;
        }
        let target = PlaneTarget::new(
            self.registration_model.surfels().iter().map(|s| s.position).collect(),
            self.registration_model.surfels().iter().map(|s| s.normal).collect(),
        );
        let mut guesses: Vec<Transform> = self.hint.map(|h| h.pose).into_iter().collect();
        guesses.extend(self.seed_guesses(source, attempts));
        // Try every guess and keep the best acceptable fit: an early guess can settle in a
        // nearby wrong minimum (sliding along flat faces) that still passes the thresholds.
        let mut best: Option<(point_to_plane::AlignResult, f32)> = None;
        let mut accepted: Option<(point_to_plane::AlignResult, f32)> = None;
        for guess in guesses {
            let r = align(source, &target, guess, &self.config.gates_mm, self.config.iterations_per_gate);
            let c = depth.map_or(0.0, |(d, k)| self.visibility_conflict(&r.pose, d, k));
            if r.inlier_ratio >= self.config.relocalize_min_inlier_ratio
                && r.rmse_mm <= self.config.max_rmse_mm
                && c <= self.config.relocalize_max_visibility_conflict
                && accepted.as_ref().is_none_or(|(a, _)| r.inliers > a.inliers || (r.inliers == a.inliers && r.rmse_mm < a.rmse_mm))
            {
                accepted = Some((r, c));
            }
            if best.as_ref().is_none_or(|(b, _)| r.inliers > b.inliers) {
                best = Some((r, c));
            }
        }
        if let Some((r, c)) = accepted {
            self.current_pose = r.pose;
            self.velocity = Transform::default();
            self.imu_at_pose = self.latest_imu.map(|s| s.orientation.normalize());
            self.lost_frames = 0;
            self.state = TrackingState::Tracking;
            self.seed_state = SeedState::Registered;
            let id = self.store_keyframe(oriented, source, r.rmse_mm, c, depth.map(|(d, _)| d));
            return Some(self.status(r.inliers, r.rmse_mm, c, Some(id), false));
        }
        if attempts + 1 >= self.config.seed_max_frames {
            // Give up: this pass gets a frame of its own, starting with this frame.
            self.seed = None;
            self.seed_state = SeedState::Abandoned;
            self.registration_model.clear();
            self.current_pose = Transform::default();
            self.state = TrackingState::Idle;
            return None;
        }
        self.seed_state = SeedState::Pending { attempts: attempts + 1 };
        self.state = TrackingState::Lost;
        let (inliers, rmse, conflict) = best.map_or((0, f32::INFINITY, 0.0), |(r, c)| (r.inliers, r.rmse_mm, c));
        Some(self.status(inliers, rmse, conflict, None, false))
    }

    /// Starting poses in the seed for a frame: stored views (closest IMU orientation first when
    /// known), a different handful on each attempt, each placed by matching centroids. The first
    /// attempt also tries the seed frame's origin (a pass started where the first one started).
    fn seed_guesses(&self, source: &[(Vec3, Vec3)], attempt: usize) -> Vec<Transform> {
        let Some(seed) = &self.seed else { return Vec::new() };
        let mut out: Vec<Transform> = Vec::new();
        if attempt == 0 || seed.views.is_empty() {
            out.push(Transform::default());
        }
        if seed.views.is_empty() {
            return out;
        }
        let centroid = source.iter().map(|(p, _)| *p).sum::<Vec3>() / source.len() as f32;
        let now = self.latest_imu.map(|s| s.orientation.normalize());
        let mut ranked: Vec<(f32, &SeedView)> = seed
            .views
            .iter()
            .map(|v| {
                let a = match (now, v.imu, self.config.imu_to_sensor) {
                    (Some(q), Some(qv), Some(_)) => qv.angle_between(q),
                    _ => 0.0,
                };
                (a, v)
            })
            .collect();
        ranked.sort_by(|a, b| a.0.total_cmp(&b.0));
        let mut spread: Vec<&SeedView> = Vec::new();
        for (_, v) in ranked {
            if spread.iter().any(|c| (c.pose.translation - v.pose.translation).length() < 15.0) {
                continue;
            }
            spread.push(v);
        }
        let per_frame = self.config.relocalize_candidates.max(1).min(spread.len());
        let start = (attempt * per_frame) % spread.len();
        out.extend((0..per_frame).map(|i| {
            let v = spread[(start + i) % spread.len()];
            let rotation = v.pose.rotation;
            let translation = v.pose.transform_point3(v.centroid) - rotation * centroid;
            Transform { rotation, translation, scale: Vec3::ONE }
        }));
        out
    }

    fn is_new_keyframe(&self) -> bool {
        let Some(last) = self.keyframes.iter().rev().find(|k| !k.discarded) else { return true };
        let step = compose(&self.current_pose, &last.pose.inverse());
        step.translation.length() >= self.config.keyframe_translation_mm
            || step.rotation.angle_between(Quat::IDENTITY) >= self.config.keyframe_rotation_rad
    }

    /// Starting poses for re-locating against stored keyframes: those viewing from a similar
    /// direction (by IMU orientation; without an IMU, a spread of keyframes), each seeded by
    /// matching the frame's centroid to the keyframe's.
    fn relocalization_guesses(&self, source: &[(Vec3, Vec3)]) -> Vec<(Transform, bool)> {
        let centroid = source.iter().map(|(p, _)| *p).sum::<Vec3>() / source.len() as f32;
        let kept: Vec<&Keyframe> = self.keyframes.iter().filter(|k| !k.discarded).collect();
        if kept.is_empty() {
            return Vec::new();
        }
        let now = self.latest_imu.map(|s| s.orientation.normalize());
        let mut ranked: Vec<(f32, &Keyframe)> = match (now, self.config.imu_to_sensor) {
            (Some(q), Some(_)) => kept
                .iter()
                .filter_map(|k| k.imu.map(|qk| (qk.angle_between(q), *k)))
                .filter(|(a, _)| *a <= self.config.relocalize_max_angle_rad)
                .collect(),
            _ => {
                // No orientation to go by: the first keyframe (people return to where they
                // started) plus a spread over the rest, newest first.
                let step = (kept.len() / self.config.relocalize_candidates.max(1)).max(1);
                std::iter::once((0.0, kept[0])).chain(kept.iter().rev().step_by(step).map(|k| (1.0, *k))).collect()
            }
        };
        ranked.sort_by(|a, b| a.0.total_cmp(&b.0));

        let mut chosen: Vec<&Keyframe> = Vec::new();
        for (_, kf) in ranked {
            // Spread the attempts: skip keyframes almost at an already chosen pose.
            if chosen.iter().any(|c| (c.pose.translation - kf.pose.translation).length() < 15.0) {
                continue;
            }
            chosen.push(kf);
            if chosen.len() >= self.config.relocalize_candidates {
                break;
            }
        }
        chosen
            .into_iter()
            .map(|kf| {
                // The rotation is trustworthy when it comes from keyframe + IMU; then the result
                // may not stray far from it (`.1`).
                let (rotation, imu_predicted) = match (kf.imu, now, self.config.imu_to_sensor) {
                    (Some(qk), Some(q), Some(x)) => {
                        let body = if self.config.imu_body_to_world { qk.inverse() * q } else { qk * q.inverse() };
                        ((kf.pose.rotation * (x * body * x.inverse())).normalize(), true)
                    }
                    _ => (kf.pose.rotation, false),
                };
                let translation = kf.pose.transform_point3(kf.centroid) - rotation * centroid;
                (Transform { rotation, translation, scale: Vec3::ONE }, imu_predicted)
            })
            .collect()
    }

    /// Rejects alignments whose rotation since the last accepted frame grossly contradicts the
    /// IMU. Tolerant on purpose: the IMU sample is not taken at the exact exposure time.
    fn imu_consistent(&self, pose: &Transform) -> bool {
        let Some(imu_delta) = self.imu_rotation_since_pose() else { return true };
        let icp_delta = (self.current_pose.rotation.inverse() * pose.rotation).normalize();
        let imu_angle = imu_delta.angle_between(Quat::IDENTITY);
        icp_delta.angle_between(imu_delta) <= 0.14 + 0.8 * imu_angle
    }

    pub fn reset(&mut self) {
        *self = Self::with_config(self.config.clone());
    }

    fn accepted(&self, inlier_ratio: f32, rmse_mm: f32) -> bool {
        inlier_ratio >= self.config.min_inlier_ratio && rmse_mm <= self.config.max_rmse_mm
    }

    /// Fuses the frame at the current pose and keeps it as a keyframe. Returns its id.
    fn store_keyframe(
        &mut self,
        dense: &[(Vec3, Vec3)],
        sample: &[(Vec3, Vec3)],
        rmse_mm: f32,
        conflict: f32,
        depth: Option<&DepthMap>,
    ) -> u64 {
        let pose = self.current_pose;
        let view = if self.config.noise.carve { depth.and_then(|d| FreeSpaceView::from_depth(d, self.config.noise.carve_block_px)) } else { None };
        let thinned = VoxelMap::from_points(self.config.fusion_voxel_mm, dense.iter().copied());
        let points: Vec<(Vec3, Vec3)> = thinned.surfels().iter().map(|s| (s.position, s.normal)).collect();
        // Tracking needs the registration model now; the fine surface is merged in the background.
        // The new-point count is estimated from the coarse model (new registration voxels scaled
        // to the fusion pitch) so it is known immediately.
        let new_coarse = self.registration_model_add(&pose, sample, view.as_ref());
        let ratio = self.config.registration_voxel_mm / self.config.fusion_voxel_mm.max(1e-3);
        let new_points = (new_coarse as f32 * ratio * ratio) as usize;
        self.fused.submit(self.fuse_job(&pose, &points, view.clone()));
        let id = self.next_keyframe_id;
        self.next_keyframe_id += 1;
        let centroid = sample.iter().map(|(p, _)| *p).sum::<Vec3>() / sample.len().max(1) as f32;
        self.keyframes.push(Keyframe {
            id,
            pose,
            rmse_mm,
            visibility_conflict: conflict,
            new_points,
            discarded: false,
            imu: self.latest_imu.map(|s| s.orientation.normalize()),
            centroid,
            points: points.iter().map(|&(p, n)| PackedPoint::new(p, n)).collect(),
            view,
        });
        id
    }

    /// Adds one keyframe to both maps at `pose` (registration sample and fusion-pitch points,
    /// sensor frame), then carves both with the frame's free-space view. Returns the number of
    /// new fusion surfels. Shared by live fusion and rebuilds so both give the same model.
    /// Adds a keyframe's registration sample (sensor frame) at `pose` to the registration model
    /// and carves it with the keyframe's free-space view. Returns the number of new voxels.
    fn registration_model_add(&mut self, pose: &Transform, sample: &[(Vec3, Vec3)], view: Option<&FreeSpaceView>) -> usize {
        let mut new_voxels = 0;
        for &(p, n) in sample {
            if self.registration_model.insert(pose.transform_point3(p), pose.rotation * n).is_some() {
                new_voxels += 1;
            }
        }
        if let Some(view) = view {
            let to_sensor = pose.inverse();
            let (margin, per_mm) = (self.config.noise.carve_margin_mm, self.config.noise.carve_margin_per_mm);
            // Registration surfels are averages over a coarser voxel: allow for that.
            let extra = self.config.registration_voxel_mm;
            self.registration_model.carve(move |p: Vec3| {
                let q = to_sensor.transform_point3(p);
                view.sees_through(q, margin + extra + per_mm * q.y)
            });
        }
        new_voxels
    }

    /// The fusion job for a keyframe's fusion-pitch points (sensor frame) at `pose`.
    fn fuse_job(&self, pose: &Transform, points: &[(Vec3, Vec3)], view: Option<FreeSpaceView>) -> fusion::FuseJob {
        fusion::FuseJob {
            pose: *pose,
            points: points.to_vec(),
            view,
            carve_margin_mm: self.config.noise.carve_margin_mm,
            carve_margin_per_mm: self.config.noise.carve_margin_per_mm,
        }
    }

    fn status(&self, inliers: usize, rmse_mm: f32, visibility_conflict: f32, keyframe: Option<u64>, relocalized: bool) -> TrackingStatus {
        TrackingStatus {
            state: self.state,
            current_pose: self.current_pose,
            inliers,
            rmse_mm,
            total_points_fused: self.fused.visible(),
            visibility_conflict,
            keyframe,
            relocalized,
        }
    }

    /// Projects the model into the depth image as seen from `pose` and returns the fraction of
    /// comparable points that lie clearly in front of the measured surface (see
    /// [`TrackingStatus::visibility_conflict`]). Model points behind the measurement are just
    /// occluded and do not count; points without a measurement are skipped.
    fn visibility_conflict(&self, pose: &Transform, depth: &DepthMap, k: PinholeIntrinsics) -> f32 {
        const AGREE_MM: f32 = 4.0;
        const IN_FRONT_MM: f32 = 8.0;
        const EDGE_MM: f32 = 10.0;
        let to_sensor = pose.inverse();
        let surfels = self.registration_model.surfels();
        let stride = (surfels.len() / 20_000).max(1);
        let (w, h) = (depth.width as i64, depth.height as i64);
        let (mut agree, mut conflict) = (0usize, 0usize);
        for s in surfels.iter().step_by(stride) {
            let p = to_sensor.transform_point3(s.position);
            // Only surfaces facing the sensor can be seen.
            if (to_sensor.rotation * s.normal).dot(p) >= 0.0 || p.y <= 1.0 {
                continue;
            }
            // Scanner frame (x right, y forward, z up) -> optical (x right, y down, z forward).
            let u = (k.fx * p.x / p.y + k.cx).round() as i64;
            let v = (k.fy * -p.z / p.y + k.cy).round() as i64;
            if u < 0 || v < 0 || u >= w || v >= h {
                continue;
            }
            let raw = depth.data[(v * w + u) as usize];
            if raw == 0 {
                continue;
            }
            // Skip depth discontinuities: at silhouettes a sub-pixel projection error lands model
            // points on the background behind an object, which is not a real contradiction.
            if u == 0 || v == 0 || u + 1 >= w || v + 1 >= h {
                continue;
            }
            let at_edge = [(-1i64, 0i64), (1, 0), (0, -1), (0, 1)].iter().any(|(du, dv)| {
                let n = depth.data[((v + dv) * w + u + du) as usize];
                n == 0 || (n as f32 - raw as f32).abs() * depth.scale_mm > EDGE_MM
            });
            if at_edge {
                continue;
            }
            let diff = p.y - raw as f32 * depth.scale_mm;
            if diff.abs() <= AGREE_MM {
                agree += 1;
            } else if diff < -IN_FRONT_MM {
                conflict += 1;
            }
        }
        if agree + conflict < 200 {
            return 0.0; // too little shared view to judge either way
        }
        conflict as f32 / (agree + conflict) as f32
    }
}

/// `a ∘ b`: apply `b`, then `a`.
fn compose(a: &Transform, b: &Transform) -> Transform {
    Transform {
        rotation: (a.rotation * b.rotation).normalize(),
        translation: a.rotation * b.translation + a.translation,
        scale: Vec3::ONE,
    }
}

pub(crate) fn surfel_point(s: &Surfel) -> Point3D {
    Point3D::new(s.position.to_array()).with_normal(s.normal.to_array()).with_color(scancraft_core::ColorRgba::GRAY)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A bumpy, non-symmetric surface patch ~120 x 120 mm with normals, in world coordinates.
    fn scene() -> Vec<(Vec3, Vec3)> {
        let height = |x: f32, z: f32| {
            8.0 * (x * 0.05).sin() * (z * 0.07).cos() + 15.0 * (-((x - 20.0).powi(2) + (z + 10.0).powi(2)) / 400.0).exp()
                + 0.04 * x
        };
        let mut pts = Vec::new();
        let step = 0.6;
        let n = (120.0 / step) as i32;
        for i in 0..n {
            for j in 0..n {
                let (x, z) = (-60.0 + i as f32 * step, -60.0 + j as f32 * step);
                let y = 250.0 - height(x, z);
                let dx = (height(x + 0.1, z) - height(x - 0.1, z)) / 0.2;
                let dz = (height(x, z + 0.1) - height(x, z - 0.1)) / 0.2;
                // Surface y = 250 - h(x,z); normal facing the scanner (-Y).
                let normal = Vec3::new(-dx, -1.0, -dz).normalize();
                pts.push((Vec3::new(x, y, z), normal));
            }
        }
        pts
    }

    /// What the sensor sees from `pose` (world <- sensor): the scene in sensor coordinates,
    /// cropped to a 90 mm wide window around the optical axis, with deterministic noise.
    fn observe(scene: &[(Vec3, Vec3)], pose: &Transform, seed: u32) -> PointCloud {
        let inv = pose.inverse();
        let mut s = seed.wrapping_mul(2654435761).max(1);
        let mut noise = || {
            s ^= s << 13;
            s ^= s >> 17;
            s ^= s << 5;
            (s as f32 / u32::MAX as f32 - 0.5) * 0.5 // +-0.25 mm
        };
        let mut cloud = PointCloud::new();
        for &(p, n) in scene {
            let q = inv.transform_point3(p);
            if q.x.abs() < 45.0 && q.z.abs() < 45.0 {
                let q = q + Vec3::new(0.0, noise(), 0.0);
                cloud.push(Point3D::new(q.to_array()).with_normal((inv.rotation * n).to_array()));
            }
        }
        cloud
    }

    fn pose_at(k: usize) -> Transform {
        let t = k as f32;
        Transform {
            translation: Vec3::new(t * 1.5, (t * 0.3).sin() * 4.0, -t * 1.0),
            rotation: Quat::from_euler(glam::EulerRot::XYZ, t * 0.008, 0.0, t * 0.012),
            scale: Vec3::ONE,
        }
    }

    #[test]
    fn follows_a_moving_scanner_without_drift() {
        let scene = scene();
        let mut tracker = Tracker::new();
        for k in 0..25 {
            let status = tracker.register_frame(&observe(&scene, &pose_at(k), k as u32 + 1));
            assert_ne!(status.state, TrackingState::Lost, "frame {k}");
            let truth = pose_at(k);
            let err_t = (status.current_pose.translation - truth.translation).length();
            let err_r = status.current_pose.rotation.angle_between(truth.rotation);
            assert!(err_t < 0.6, "frame {k}: translation error {err_t:.3} mm");
            assert!(err_r < 0.006, "frame {k}: rotation error {err_r:.4} rad");
        }
    }

    #[test]
    fn holding_still_adds_no_new_surface() {
        let scene = scene();
        let mut tracker = Tracker::new();
        tracker.register_frame(&observe(&scene, &pose_at(0), 1));
        tracker.flush();
        let first = tracker.take_new_points().len();
        assert!(first > 1000);
        for k in 0..10 {
            tracker.register_frame(&observe(&scene, &pose_at(0), 100 + k));
        }
        tracker.flush();
        let later = tracker.take_new_points().len();
        // Noise touches a few neighbouring voxels; the model must not keep growing like a stack.
        assert!(later < first / 3, "first {first}, after 10 still frames {later}");
        assert!(tracker.current_pose.translation.length() < 0.3);
    }

    /// Depth image of a wall at 300 mm, optionally with a 120 x 100 mm box face at 270 mm.
    fn wall_depth(with_box: bool) -> DepthMap {
        let k = PinholeIntrinsics { fx: 80.0, fy: 80.0, cx: 32.0, cy: 24.0 };
        let mut data = vec![0u16; 64 * 48];
        for v in 0..48 {
            for u in 0..64 {
                let (x, z) = ((u as f32 - k.cx) * 270.0 / k.fx, -(v as f32 - k.cy) * 270.0 / k.fy);
                let on_box = with_box && x.abs() < 60.0 && z.abs() < 50.0;
                data[v * 64 + u] = if on_box { 27000 } else { 30000 };
            }
        }
        DepthMap::new(64, 48, 0.01, data).with_intrinsics(k)
    }

    #[test]
    fn frame_contradicting_visible_model_is_rejected() {
        let mut tracker = Tracker::new();
        tracker.register_depth(&wall_depth(true), 150.0, 400.0);
        let fused = tracker.fused_points().len();

        // Plain wall only: point-to-plane fits it perfectly, but the box would have to be seen.
        let status = tracker.register_depth(&wall_depth(false), 150.0, 400.0);
        assert!(status.visibility_conflict > 0.15, "conflict {}", status.visibility_conflict);
        assert_eq!(status.state, TrackingState::Lost);
        assert_eq!(tracker.fused_points().len(), fused, "a contradicting frame must not be fused");

        // The same view again is consistent and accepted.
        let status = tracker.register_depth(&wall_depth(true), 150.0, 400.0);
        assert_ne!(status.state, TrackingState::Lost);
        assert!(status.visibility_conflict < 0.05, "conflict {}", status.visibility_conflict);
    }

    #[test]
    fn relocalizes_when_returning_to_an_earlier_view() {
        let scene = scene();
        let mut tracker = Tracker::new();
        for k in 0..30 {
            assert_ne!(tracker.register_frame(&observe(&scene, &pose_at(k), k as u32 + 1)).state, TrackingState::Lost);
        }
        // Lose tracking (look at something unrelated)...
        let mut wall = PointCloud::new();
        for i in 0..80 {
            for j in 0..80 {
                wall.push(Point3D::new([i as f32 - 40.0, 380.0 + (i * j % 7) as f32, j as f32 - 40.0]).with_normal([0.0, -1.0, 0.0]));
            }
        }
        assert_eq!(tracker.register_frame(&wall).state, TrackingState::Lost);

        // ...then come back near the start: ~55 mm and ~20 degrees from the last good pose, beyond
        // what tracking from the last pose can bridge.
        let back = observe(&scene, &pose_at(1), 99);
        // The keyframe candidates alone must already put the frame close to where it belongs.
        let source: Vec<(Vec3, Vec3)> = VoxelMap::from_points(2.5, back.points.iter().map(|p| (Vec3::from(p.position), Vec3::from(p.normal))))
            .surfels()
            .iter()
            .map(|s| (s.position, s.normal))
            .collect();
        let guesses = tracker.relocalization_guesses(&source);
        let best = guesses.iter().map(|(g, _)| (g.translation - pose_at(1).translation).length()).fold(f32::MAX, f32::min);
        assert!(best < 20.0, "closest relocalization guess is {best:.1} mm off (first ICP gate is 30 mm)");

        let status = tracker.register_frame(&back);
        assert_ne!(status.state, TrackingState::Lost, "should re-locate against stored keyframes");
        let err = (status.current_pose.translation - pose_at(1).translation).length();
        assert!(err < 0.8, "relocalized pose error {err:.3} mm");
    }

    #[test]
    fn discarding_keyframes_rebuilds_the_model() {
        let scene = scene();
        let mut tracker = Tracker::new();
        for k in 0..12 {
            tracker.register_frame(&observe(&scene, &pose_at(k * 2), k as u32 + 1));
        }
        let kfs: Vec<u64> = tracker.keyframes().iter().map(|k| k.id).collect();
        assert!(kfs.len() >= 10, "moving frames become keyframes: {}", kfs.len());
        let full = tracker.fused_points().len();

        tracker.discard_from(kfs[6]);
        let trimmed = tracker.fused_points().len();
        assert!(trimmed < full, "discarding later frames removes their surface ({trimmed} vs {full})");
        assert!(tracker.keyframes()[6..].iter().all(|k| k.discarded));

        for &id in &kfs[6..] {
            assert!(tracker.set_discarded(id, false));
        }
        assert_eq!(tracker.fused_points().len(), full, "restoring gives the same model back");

        // Tracking continues from the last kept keyframe.
        let status = tracker.register_frame(&observe(&scene, &pose_at(23), 77));
        assert_ne!(status.state, TrackingState::Lost);
    }

    #[test]
    fn fixed_pose_hints_define_the_world_frame_and_are_fused_as_given() {
        let scene = scene();
        let offset = Transform { translation: Vec3::new(100.0, -20.0, 5.0), rotation: Quat::from_rotation_z(0.3), scale: Vec3::ONE };
        let mut tracker = Tracker::new();
        for k in 0..4 {
            let truth = compose(&offset, &pose_at(k * 3));
            let hint = PoseHint { pose: truth, kind: HintKind::Fixed };
            let status = tracker.register_with(&observe(&scene, &pose_at(k * 3), k as u32 + 1), None, Some(hint));
            assert_eq!(status.current_pose, truth, "fixed hints are used unchanged");
        }
        // The model lives in the hint's frame: a prior near the truth is refined onto it.
        let truth = compose(&offset, &pose_at(10));
        let rough = Transform { translation: truth.translation + Vec3::new(3.0, -2.0, 1.0), ..truth };
        let status = tracker.register_with(&observe(&scene, &pose_at(10), 50), None, Some(PoseHint { pose: rough, kind: HintKind::Prior }));
        assert_ne!(status.state, TrackingState::Lost);
        assert!((status.current_pose.translation - truth.translation).length() < 0.6);
    }

    #[test]
    fn keyframes_round_trip_through_data() {
        let scene = scene();
        let mut tracker = Tracker::new();
        for k in 0..10 {
            tracker.register_frame(&observe(&scene, &pose_at(k * 2), k as u32 + 1));
        }
        let ids: Vec<u64> = tracker.keyframes().iter().map(|k| k.id).collect();
        tracker.set_discarded(ids[3], true);
        let data: Vec<KeyframeData> = tracker.keyframes().iter().map(Keyframe::to_data).collect();
        let mut restored = Tracker::from_keyframes(TrackerConfig::default(), data.iter().cloned().map(Keyframe::from_data).collect());
        assert_eq!(restored.keyframes().len(), tracker.keyframes().len());
        assert_eq!(restored.fused_points().len(), tracker.fused_points().len());
        assert!(restored.keyframes()[3].discarded);
        let again: Vec<KeyframeData> = restored.keyframes().iter().map(Keyframe::to_data).collect();
        assert_eq!(again, data);
        // Capture continues; new keyframes get fresh ids.
        let status = restored.register_frame(&observe(&scene, &pose_at(21), 99));
        assert_ne!(status.state, TrackingState::Lost);
        if let Some(id) = status.keyframe {
            assert!(id > *ids.last().unwrap());
        }
    }

    /// A model of the scene from an earlier pass, as a project would seed it: the earlier
    /// tracker's world points and its keyframes as views, expressed in a "project frame".
    fn earlier_pass(scene: &[(Vec3, Vec3)], offset: &Transform) -> (Vec<(Vec3, Vec3)>, Vec<SeedView>) {
        let mut earlier = Tracker::new();
        for k in 0..12 {
            earlier.register_frame(&observe(scene, &pose_at(k * 2), k as u32 + 1));
        }
        let points = earlier
            .fused_points()
            .iter()
            .map(|p| (offset.transform_point3(Vec3::from(p.position)), offset.rotation * Vec3::from(p.normal)))
            .collect();
        let views = earlier.keyframes().iter().map(|k| SeedView { pose: compose(offset, &k.pose), imu: None, centroid: k.centroid() }).collect();
        (points, views)
    }

    #[test]
    fn seeded_tracker_registers_into_the_seed_frame() {
        let scene = scene();
        let offset = Transform { translation: Vec3::new(40.0, -15.0, 8.0), rotation: Quat::from_rotation_z(0.25), scale: Vec3::ONE };
        let (points, views) = earlier_pass(&scene, &offset);
        let mut tracker = Tracker::new();
        assert!(tracker.seed_model(&points, &views));
        assert_eq!(tracker.seed_state(), SeedState::Pending { attempts: 0 });
        // The new pass starts near a later view of the earlier pass, not at its origin.
        let mut registered_at = None;
        for k in 0..10 {
            let truth = compose(&offset, &pose_at(15 + k));
            let status = tracker.register_frame(&observe(&scene, &pose_at(15 + k), 200 + k as u32));
            if tracker.seed_state() == SeedState::Registered {
                registered_at.get_or_insert(k);
                let err = (status.current_pose.translation - truth.translation).length();
                assert!(err < 0.8, "frame {k}: pose error {err:.3} mm in the seed frame");
            }
        }
        assert!(registered_at.is_some_and(|k| k <= 2), "located within the first frames: {registered_at:?}");
        // The pass's own model holds only its own frames.
        let own = tracker.fused_points();
        assert!(!own.is_empty());
        assert!(own.len() < points.len() * 2);
    }

    #[test]
    fn seeded_tracker_falls_back_to_its_own_frame() {
        let scene = scene();
        let (points, views) = earlier_pass(&scene, &Transform::default());
        let config = TrackerConfig { seed_max_frames: 3, ..TrackerConfig::default() };
        let mut tracker = Tracker::with_config(config);
        assert!(tracker.seed_model(&points, &views));
        // Something unrelated to the seed.
        let mut wall = PointCloud::new();
        for i in 0..80 {
            for j in 0..80 {
                wall.push(Point3D::new([i as f32 - 40.0, 380.0 + (i * j % 7) as f32, j as f32 - 40.0]).with_normal([0.0, -1.0, 0.0]));
            }
        }
        for _ in 0..2 {
            assert_eq!(tracker.register_frame(&wall).state, TrackingState::Lost);
            assert!(tracker.fused_points().is_empty());
        }
        let status = tracker.register_frame(&wall);
        assert_eq!(tracker.seed_state(), SeedState::Abandoned);
        assert_eq!(status.state, TrackingState::Tracking, "the frame starts a frame of its own");
        assert!(status.current_pose.translation.length() < 1e-6);
        assert!(!tracker.fused_points().is_empty());
    }

    #[test]
    fn seeding_needs_an_empty_tracker() {
        let scene = scene();
        let mut tracker = Tracker::new();
        tracker.register_frame(&observe(&scene, &pose_at(0), 1));
        assert!(!tracker.seed_model(&[(Vec3::ZERO, Vec3::Y)], &[]));
        assert_eq!(tracker.seed_state(), SeedState::None);
    }

    #[test]
    fn unrelated_frame_is_lost_and_not_fused() {
        let scene = scene();
        let mut tracker = Tracker::new();
        tracker.register_frame(&observe(&scene, &pose_at(0), 1));
        let fused = tracker.fused_points().len();

        // A wall far behind the model: no overlap.
        let mut wall = PointCloud::new();
        for i in 0..80 {
            for j in 0..80 {
                wall.push(Point3D::new([i as f32 - 40.0, 380.0 + (i * j % 7) as f32, j as f32 - 40.0]).with_normal([0.0, -1.0, 0.0]));
            }
        }
        let status = tracker.register_frame(&wall);
        assert_eq!(status.state, TrackingState::Lost);
        assert_eq!(tracker.fused_points().len(), fused, "a lost frame must not be fused");

        // Coming back over the scanned surface recovers.
        let status = tracker.register_frame(&observe(&scene, &pose_at(1), 2));
        assert_ne!(status.state, TrackingState::Lost);
        assert!((status.current_pose.translation - pose_at(1).translation).length() < 0.6);
    }

    /// Depth image (64 x 48) seen from a sensor shifted by `tx` mm along X: wall at 300 mm, a box
    /// face at 270 mm and optionally a floating patch at 200 mm (e.g. something that moved away).
    fn carving_scene(tx: f32, with_patch: bool) -> DepthMap {
        let k = PinholeIntrinsics { fx: 80.0, fy: 80.0, cx: 31.5, cy: 23.5 };
        let mut data = vec![0u16; 64 * 48];
        for v in 0..48 {
            for u in 0..64 {
                let at = |d: f32| ((u as f32 - k.cx) * d / k.fx + tx, -(v as f32 - k.cy) * d / k.fy);
                let (px, pz) = at(200.0);
                let (bx, bz) = at(270.0);
                let d = if with_patch && (-45.0..-5.0).contains(&px) && pz.abs() < 20.0 {
                    200.0
                } else if (10.0..50.0).contains(&bx) && bz.abs() < 30.0 {
                    270.0
                } else {
                    300.0
                };
                data[v * 64 + u] = (d * 100.0) as u16;
            }
        }
        DepthMap::new(64, 48, 0.01, data).with_intrinsics(k)
    }

    #[test]
    fn surface_seen_through_later_is_carved_and_rebuilds_carve_the_same() {
        let mut tracker = Tracker::new();
        let fixed = |tx: f32| PoseHint { pose: Transform::from_translation(Vec3::new(tx, 0.0, 0.0)), kind: HintKind::Fixed };
        let geometry = |d: &DepthMap| d.to_point_cloud_with(&BackProjection::filtered(150.0, 400.0));
        let on_patch = |pts: &[Point3D]| pts.iter().filter(|p| (p.position[1] - 200.0).abs() < 2.0).count();

        let first = carving_scene(0.0, true);
        tracker.register_with(&geometry(&first), Some(&first), Some(fixed(0.0)));
        tracker.flush();
        let shown = tracker.take_new_points();
        assert!(on_patch(&shown) > 100, "the patch is fused and shown: {}", on_patch(&shown));
        assert_eq!(tracker.model_revision(), 0);

        // The patch has gone: the next view looks through where it was.
        let second = carving_scene(5.0, false);
        let status = tracker.register_with(&geometry(&second), Some(&second), Some(fixed(5.0)));
        assert!(status.keyframe.is_some());
        assert_eq!(on_patch(&tracker.fused_points()), 0, "seen-through surface is carved");
        assert!(tracker.fused_points().len() > 1000, "the rest stays");
        assert!(tracker.model_revision() > 0, "handed-out points were withdrawn");
        tracker.flush();
        match tracker.take_update() {
            SurfaceUpdate::Replace(points) => assert_eq!(on_patch(&points), 0),
            SurfaceUpdate::Append(_) => panic!("withdrawn points require a full replacement"),
        }
        let carved = tracker.fused_points().len();

        // Rebuilds carve with the stored views exactly as live fusion did.
        let ids: Vec<u64> = tracker.keyframes().iter().map(|k| k.id).collect();
        tracker.set_discarded(ids[1], true);
        assert!(on_patch(&tracker.fused_points()) > 100, "without the second view the patch is back");
        tracker.set_discarded(ids[1], false);
        assert_eq!(tracker.fused_points().len(), carved);
        assert_eq!(on_patch(&tracker.fused_points()), 0);
    }

    #[test]
    fn noise_handling_can_be_switched_off() {
        let config = TrackerConfig { noise: NoiseConfig::off(), ..TrackerConfig::default() };
        let mut tracker = Tracker::with_config(config);
        let depth = carving_scene(0.0, true);
        tracker.register_depth(&depth, 150.0, 400.0);
        // Every pixel becomes a point (64 x 48 pixels, each in its own fusion voxel).
        assert_eq!(tracker.fused_points().len(), 64 * 48);
        assert_eq!(Tracker::new().config.noise, NoiseConfig::default());
    }
}
