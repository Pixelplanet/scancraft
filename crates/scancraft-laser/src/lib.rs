//! Blue laser line geometry for the POP 4 Cross Lines / Single Line modes: sub-pixel line
//! centres in both IR images (Steger), linking into chains, left/right pairing disambiguated by
//! the laser light planes, stereo triangulation, normals from neighbouring lines.
//! See `docs/protocol/laser.md`.
//!
//! Output is in the scanner frame (+X right, +Y forward, +Z up), mm, like
//! `DepthMap::to_point_cloud`.

pub mod chains;
pub mod normals;
pub mod planes;
pub mod steger;
pub mod stereo;

use glam::Vec3;
use planes::{CalibrationParams, FrameCandidates, LaserPlaneSet};
use scancraft_core::{ColorRgba, Point3D, PointCloud, SensorFrame};
use std::path::PathBuf;
use stereo::Rectified;

/// Environment variable naming a JSON file with calibrated laser planes ([`LaserPlaneSet`]).
/// Loaded at start if it exists; written after online calibration if it does not. Without it the
/// planes are kept per scanner in the user's data directory ([`planes_cache_file`]).
pub const PLANES_ENV: &str = "SCANCRAFT_LASER_PLANES";

/// The POP 4 projects 15 lines per family; a calibration that finds more planes than this per
/// family has split lines into duplicates (seen with over-bright frames: 57 planes) and is
/// rejected.
pub const MAX_PLANES_PER_FAMILY: usize = 15;

/// Most line pairings per frame (average over the calibration frames) a cross-line calibration is
/// attempted with; the Normal preset gives ~150, an over-exposed Dark preset ~550.
pub const MAX_CALIBRATION_CANDIDATES: usize = 300;

/// Where the laser planes of the scanner behind `rig` are kept between scans. The planes are
/// fixed device geometry, and calibrating them takes seconds to minutes, so it is done once per
/// scanner: `<data dir>/ScanCraft/laser_planes_<pattern>_<rig key>.json`, where the key is
/// derived from the rig's calibration (different scanners have different values).
pub fn planes_cache_file(pattern: Pattern, rig: &scancraft_core::StereoRig) -> Option<PathBuf> {
    let base = std::env::var_os("LOCALAPPDATA")
        .or_else(|| std::env::var_os("XDG_DATA_HOME"))
        .map(PathBuf::from)
        .or_else(|| std::env::var_os("HOME").map(|h| PathBuf::from(h).join(".local/share")))?;
    let k = rig.intrinsics;
    let mut key = 0xcbf29ce484222325u64; // FNV-1a over the calibration values
    for v in [k.fx, k.fy, k.cx, k.cy, rig.baseline_mm, rig.width as f32, rig.height as f32] {
        for b in v.to_bits().to_le_bytes() {
            key = (key ^ b as u64).wrapping_mul(0x100000001b3);
        }
    }
    let name = match pattern {
        Pattern::Cross => "cross",
        Pattern::Single => "single",
    };
    Some(base.join("ScanCraft").join(format!("laser_planes_{name}_{key:016x}.json")))
}

#[derive(Debug, Clone, Copy)]
pub struct LaserParams {
    pub steger: steger::StegerParams,
    /// Working depth window, mm (Revo: zmin/zmax 200..350, bar 170..370 for the line modes).
    pub z_min: f32,
    pub z_max: f32,
    /// Samples closer than this are linked into one line (bridges rows Steger rejects on
    /// saturated line centres), px.
    pub link_px: f32,
    /// Shortest line stretch used, samples.
    pub min_chain: usize,
    /// A pairing must put this fraction of its points within `plane_tolerance_mm` of a plane.
    pub min_plane_fraction: f32,
    pub plane_tolerance_mm: f32,
    /// Neighbourhood radius for normals, mm (adjacent lines are ~8 mm apart at 250 mm).
    pub normal_radius_mm: f32,
    /// Frames collected before the planes are calibrated online.
    pub calibration_frames: usize,
}

impl Default for LaserParams {
    fn default() -> Self {
        Self {
            steger: steger::StegerParams::default(),
            z_min: 150.0,
            z_max: 450.0,
            link_px: 3.0,
            min_chain: 20,
            min_plane_fraction: 0.85,
            plane_tolerance_mm: 0.6,
            normal_radius_mm: 12.0,
            calibration_frames: 40,
        }
    }
}

/// Per-frame statistics, for logging and the verification tools.
#[derive(Debug, Clone, Default)]
pub struct FrameStats {
    pub left_samples: usize,
    pub right_samples: usize,
    pub left_chains: usize,
    pub right_chains: usize,
    pub candidates: usize,
    pub matched_chains: usize,
    pub ambiguous_chains: usize,
    pub points: usize,
}

/// One frame's laser geometry.
#[derive(Debug, Clone, Default)]
pub struct LaserFrame {
    pub cloud: PointCloud,
    pub stats: FrameStats,
}

/// Linked lines of one frame and their possible pairings.
pub struct FrameLines {
    pub left: Vec<chains::Chain>,
    pub right: Vec<chains::Chain>,
    pub candidates: Vec<stereo::Candidate>,
    pub geometry: Rectified,
    pub stats: FrameStats,
}

/// Which line pattern the frames show.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Pattern {
    /// Two families of parallel lines in alternating frames (POP 4 `0x3001 7`).
    Cross,
    /// One line (POP 4 `0x3001 9`).
    Single,
}

/// Where the laser planes a [`LaserLineDepth`] started with came from: what a replay needs to
/// process the same frames the same way (the per-scanner cache can change or be missing on
/// another computer).
#[derive(Debug, Clone, PartialEq)]
pub enum PlanesOrigin {
    /// No frame processed yet.
    Unknown,
    /// Given when made ([`LaserLineDepth::with_planes`]).
    Given,
    /// Read from the file in [`PLANES_ENV`].
    Env(PathBuf),
    /// Read from the per-scanner cache ([`planes_cache_file`]) at the first frame.
    Cache(PathBuf),
    /// None at the start: calibrated online from the first frames.
    Online,
}

/// Turns IR pairs with laser lines into sparse point clouds.
pub struct LaserLineDepth {
    pub params: LaserParams,
    pattern: Pattern,
    planes: Option<LaserPlaneSet>,
    pending: Vec<FrameCandidates>,
    planes_file: Option<PathBuf>,
    /// Look the planes up in (and save calibrated ones to) the per-scanner cache.
    use_cache: bool,
    origin: PlanesOrigin,
    /// The planes at the first frame (`None`: calibrated online).
    start_planes: Option<LaserPlaneSet>,
}

impl Default for LaserLineDepth {
    fn default() -> Self {
        Self::new(Pattern::Cross)
    }
}

impl LaserLineDepth {
    /// Loads planes from [`PLANES_ENV`] (with `_single` appended to the file stem for the single
    /// line) if set and present; otherwise calibrates them online from the first frames.
    pub fn new(pattern: Pattern) -> Self {
        let planes_file = std::env::var_os(PLANES_ENV).map(PathBuf::from).map(|p| match pattern {
            Pattern::Cross => p,
            Pattern::Single => p.with_file_name(format!("{}_single.json", p.file_stem().and_then(|s| s.to_str()).unwrap_or("laser_planes"))),
        });
        let planes: Option<LaserPlaneSet> = planes_file.as_ref().and_then(|p| std::fs::read_to_string(p).ok()).and_then(|s| serde_json::from_str(&s).ok());
        let origin = match (&planes, &planes_file) {
            (Some(_), Some(f)) => PlanesOrigin::Env(f.clone()),
            _ => PlanesOrigin::Unknown,
        };
        Self { params: LaserParams::default(), pattern, start_planes: planes.clone(), planes, pending: Vec::new(), planes_file, use_cache: true, origin }
    }

    pub fn with_planes(pattern: Pattern, planes: LaserPlaneSet) -> Self {
        Self { params: LaserParams::default(), pattern, start_planes: Some(planes.clone()), planes: Some(planes), pending: Vec::new(), planes_file: None, use_cache: false, origin: PlanesOrigin::Given }
    }

    /// Calibrates the planes online from the first frames and never reads or writes the
    /// per-scanner cache or [`PLANES_ENV`]: a replay of a session that calibrated online.
    pub fn without_cache(pattern: Pattern) -> Self {
        Self { params: LaserParams::default(), pattern, planes: None, pending: Vec::new(), planes_file: None, use_cache: false, origin: PlanesOrigin::Unknown, start_planes: None }
    }

    /// Where the planes this started with came from.
    pub fn origin(&self) -> &PlanesOrigin {
        &self.origin
    }

    /// The planes at the first frame (`None` before it, and when they were calibrated online).
    pub fn start_planes(&self) -> Option<&LaserPlaneSet> {
        self.start_planes.as_ref()
    }

    pub fn pattern(&self) -> Pattern {
        self.pattern
    }

    pub fn planes(&self) -> Option<&LaserPlaneSet> {
        self.planes.as_ref()
    }

    /// Lines in both images, linked and rectified, plus all geometrically possible pairings.
    pub fn candidates(&self, frame: &SensorFrame) -> Option<FrameLines> {
        let ir = frame.ir.as_ref()?;
        let right = ir.right.as_ref()?;
        let rig = ir.rig.as_ref()?;
        let (w, h) = (ir.left.width as usize, ir.left.height as usize);
        if ir.left.channels != 1 || right.channels != 1 || ir.left.data.len() != w * h || right.data.len() != w * h {
            return None;
        }
        let (cam_l, cam_r) = match (&rig.rectification, ir.rectified) {
            (Some(r), false) => (Some(&r.left), Some(&r.right)),
            (_, true) => (None, None),
            (None, false) => return None,
        };
        let p = &self.params;
        let t0 = std::time::Instant::now();
        let lp = steger::extract(&ir.left.data, w, h, &p.steger, 0);
        let rp = steger::extract(&right.data, w, h, &p.steger, 0);
        let t1 = std::time::Instant::now();
        let lc = chains::link(&lp, w, h, cam_l, p.link_px, p.min_chain);
        let rc = chains::link(&rp, w, h, cam_r, p.link_px, p.min_chain);
        let t2 = std::time::Instant::now();
        let geo = Rectified::from_rig(rig);
        let cands = stereo::candidates(&lc, &rc, &geo, p.z_min, p.z_max, p.min_chain / 2, self.pattern == Pattern::Cross);
        let ms = |a: std::time::Instant, b: std::time::Instant| (b - a).as_secs_f64() * 1e3;
        log::debug!(
            "laser lines: {} + {} samples, {} + {} chains, {} candidates; extract {:.0} ms, link {:.0}, pair {:.0}",
            lp.len(),
            rp.len(),
            lc.len(),
            rc.len(),
            cands.len(),
            ms(t0, t1),
            ms(t1, t2),
            ms(t2, std::time::Instant::now())
        );
        let stats = FrameStats {
            left_samples: lp.len(),
            right_samples: rp.len(),
            left_chains: lc.len(),
            right_chains: rc.len(),
            candidates: cands.len(),
            ..Default::default()
        };
        Some(FrameLines { left: lc, right: rc, candidates: cands, geometry: geo, stats })
    }

    /// Full processing of one frame. `None` if the frame has no IR pair or no rig; an empty cloud
    /// while the planes are still being calibrated.
    pub fn process(&mut self, frame: &SensorFrame) -> Option<LaserFrame> {
        if self.use_cache && self.planes.is_none() && self.planes_file.is_none() {
            // First frame: the planes calibrated earlier for this scanner, if any.
            self.planes_file = frame.ir.as_ref().and_then(|ir| ir.rig.as_ref()).and_then(|rig| planes_cache_file(self.pattern, rig));
            self.planes = self.planes_file.as_ref().and_then(|p| std::fs::read_to_string(p).ok()).and_then(|s| serde_json::from_str(&s).ok());
            if let (Some(set), Some(path)) = (&self.planes, &self.planes_file) {
                log::info!("laser planes: {} loaded from {}", set.planes.len(), path.display());
                self.origin = PlanesOrigin::Cache(path.clone());
                self.start_planes = Some(set.clone());
            }
        }
        if self.origin == PlanesOrigin::Unknown {
            self.origin = PlanesOrigin::Online;
        }
        let FrameLines { left: lc, candidates: cands, mut stats, .. } = self.candidates(frame)?;
        let families: Vec<i8> = cands.iter().map(|c| if lc[c.left].slope > 0.0 { 1 } else { -1 }).collect();

        if self.planes.is_none() {
            self.pending.push(FrameCandidates { candidates: cands, families });
            if self.pending.len() >= self.params.calibration_frames {
                // Over-bright frames (Dark preset on a light scene) give several times the
                // pairings; calibrating on them took minutes and split lines into duplicate planes.
                let per_frame = self.pending.iter().map(|f| f.candidates.len()).sum::<usize>() / self.pending.len().max(1);
                if self.pattern == Pattern::Cross && per_frame > MAX_CALIBRATION_CANDIDATES {
                    log::warn!(
                        "laser planes not calibrated: {per_frame} line pairings per frame is too many (over-exposed?). Run one scan with the Normal preset first; the planes are then kept for this scanner."
                    );
                    self.pending.drain(..self.pending.len() / 2);
                    return Some(LaserFrame { cloud: PointCloud::new(), stats });
                }
                let started = std::time::Instant::now();
                let set = match self.pattern {
                    Pattern::Cross => LaserPlaneSet::calibrate(&self.pending, &CalibrationParams::default()),
                    Pattern::Single => LaserPlaneSet::calibrate_single(&self.pending, &CalibrationParams::default()),
                };
                log::debug!("laser plane calibration took {:.1} s", started.elapsed().as_secs_f32());
                let enough = match self.pattern {
                    Pattern::Cross => (4..=2 * MAX_PLANES_PER_FAMILY).contains(&set.planes.len()),
                    Pattern::Single => set.planes.len() == 1,
                };
                log::info!("laser planes calibrated online: {} planes from {} frames", set.planes.len(), self.pending.len());
                if enough {
                    if let Some(path) = &self.planes_file {
                        if !path.exists() {
                            if let Ok(json) = serde_json::to_string_pretty(&set) {
                                if let Some(dir) = path.parent() {
                                    let _ = std::fs::create_dir_all(dir);
                                }
                                match std::fs::write(path, json) {
                                    Ok(()) => log::info!("laser planes saved to {}", path.display()),
                                    Err(e) => log::warn!("laser planes not saved to {}: {e}", path.display()),
                                }
                            }
                        }
                    }
                    self.planes = Some(set);
                    self.pending.clear();
                } else {
                    // Keep the newest frames and try again later.
                    self.pending.drain(..self.pending.len() / 2);
                }
            }
            return Some(LaserFrame { cloud: PointCloud::new(), stats });
        }
        let planes = self.planes.as_ref().unwrap();

        // Best pairing per left chain; a chain whose best two pairings both fit a plane is ambiguous.
        let mut per_left: Vec<Vec<(usize, f32, f32, usize)>> = vec![Vec::new(); lc.len()];
        for (ci, (c, fam)) in cands.iter().zip(&families).enumerate() {
            if let Some((plane, frac, rms)) = planes.best_fit(*fam, &c.points, self.params.plane_tolerance_mm) {
                if frac >= self.params.min_plane_fraction {
                    per_left[c.left].push((ci, frac, rms, plane));
                }
            }
        }
        let mut chosen = Vec::new();
        for options in per_left.iter_mut() {
            if options.is_empty() {
                continue;
            }
            options.sort_by(|a, b| b.1.total_cmp(&a.1).then(a.2.total_cmp(&b.2)));
            if options.len() > 1 && options[1].1 >= options[0].1 - 0.05 && options[1].2 < options[0].2 * 2.0 {
                stats.ambiguous_chains += 1;
                continue;
            }
            chosen.push(options[0]);
        }
        stats.matched_chains = chosen.len();

        let mut pts: Vec<(Vec3, usize)> = Vec::new();
        for (ci, _, _, plane) in &chosen {
            let pl = &planes.planes[*plane];
            for p in &cands[*ci].points {
                if pl.distance(*p).abs() < self.params.plane_tolerance_mm * 2.0 {
                    pts.push((*p, *ci));
                }
            }
        }
        let normals = normals::estimate(&pts, self.params.normal_radius_mm);
        let mut cloud = PointCloud::new();
        for ((p, _), n) in pts.iter().zip(normals) {
            // Optical frame (x right, y down, z forward) -> scanner frame (x right, y forward, z up).
            let pos = Vec3::new(p.x, p.z, -p.y);
            let nrm = Vec3::new(n.x, n.z, -n.y);
            cloud.push(Point3D::new(pos.to_array()).with_normal(nrm.to_array()).with_color(ColorRgba::new(90, 160, 255, 255)));
        }
        stats.points = cloud.len();
        Some(LaserFrame { cloud, stats })
    }
}
