//! What a synthetic scene is: the true part, the marker stickers on it, the passes that scan it
//! and the features a mesher must get right (with probes that say how to check them).

use glam::DVec3;
use serde::{Deserialize, Serialize};

use crate::sdf::{ramp, v3, Rigid, Shape, V3};

/// A marker sticker: a disc `height` thick that follows the surface. It is flat out to
/// `flat_radius` and back on the surface at `outer_radius`. Revo lists only the white dot
/// (`dot_radius`) in its marker file. Not part of the true surface.
///
/// How well it scans (measured on Revo's `raw_preview.ply` with `examples/synth_realism.rs`):
/// the white dot is measured, the black ring round it (`dot_radius` to `ring_radius`) is thin or
/// empty, and some stickers have an empty halo round them out to 8-12 mm (S2). The `*_keep`
/// fields are the shares of the points kept there. Revo's Fused export instead cuts the whole
/// site out to exactly 5.5 mm (`cut_radius`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Sticker {
    pub id: String,
    /// Centre on the true surface (part frame).
    pub center: V3,
    /// Outward unit normal of the true surface there.
    pub normal: V3,
    pub flat_radius: f64,
    pub outer_radius: f64,
    pub height: f64,
    pub dot_radius: f64,
    /// The sticker only raises surface within this depth behind its centre plane (so a sticker on
    /// one face of a thin wall does not raise the other face).
    pub back_limit: f64,
    /// Points within this radius are cut out of the scan (as Revo's Fused export does), `None` =
    /// kept.
    pub cut_radius: Option<f64>,
    /// On the table of this pass (index), not on the part.
    pub on_table_of_pass: Option<usize>,
    /// Share of the points kept on the white dot.
    #[serde(default = "one")]
    pub dot_keep: f64,
    /// Share of the points kept on the black ring (`dot_radius` to `ring_radius`).
    #[serde(default = "one")]
    pub ring_keep: f64,
    #[serde(default = "default_ring_radius")]
    pub ring_radius: f64,
    /// An empty halo from `ring_radius` out to this radius (0 = none), keeping `halo_keep` of
    /// every point there (sticker and surface).
    #[serde(default)]
    pub halo_radius: f64,
    #[serde(default)]
    pub halo_keep: f64,
}

fn one() -> f64 {
    1.0
}

fn default_ring_radius() -> f64 {
    4.5
}

/// How a sticker comes out in a raw Revo scan (`raw_preview.ply` of S1, S2, S3).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RingStyle {
    /// S1 (outside): the ring at 3-4.5 mm thinned to about half, the rest measured.
    Thin,
    /// S3 (inside): the ring nearly empty, a sparse halo out to 6.5-8.5 mm.
    Empty,
    /// S2 (inside): the dot thinned, almost nothing from 3 mm out to 9-12 mm.
    Halo,
    /// Every point kept (an even sticker, not seen in the real scans).
    Full,
}

impl Sticker {
    /// A standard sticker (11 mm across, 0.4 mm thick, 3 mm dot) at `center` with `normal`; the
    /// ring scans thin as in S1 ([`RingStyle::Thin`]).
    pub fn standard(id: impl Into<String>, center: DVec3, normal: DVec3) -> Self {
        Sticker {
            id: id.into(),
            center: center.to_array(),
            normal: normal.normalize().to_array(),
            flat_radius: 4.3,
            outer_radius: 5.6,
            height: 0.4,
            dot_radius: 3.0,
            back_limit: 1.5,
            cut_radius: None,
            on_table_of_pass: None,
            dot_keep: 1.0,
            ring_keep: 0.55,
            ring_radius: 4.5,
            halo_radius: 0.0,
            halo_keep: 0.0,
        }
    }

    /// The sticker as it scans in `style`; `variant` (any number) varies the shares a little.
    pub fn styled(mut self, style: RingStyle, variant: usize) -> Self {
        let v = [0.0, 0.5, 1.0, 0.25, 0.75][variant % 5];
        (self.dot_keep, self.ring_keep, self.ring_radius, self.halo_radius, self.halo_keep) = match style {
            RingStyle::Thin => (1.0, 0.3 + 0.4 * v, 4.5, 0.0, 0.0),
            RingStyle::Empty => (0.9, 0.02 + 0.15 * v, 4.5, 6.5 + 2.0 * v, 0.1 + 0.2 * v),
            RingStyle::Halo => (0.6 + 0.3 * v, 0.02, 3.0, 9.0 + 3.0 * v, 0.01 + 0.04 * v),
            RingStyle::Full => (1.0, 1.0, 4.5, 0.0, 0.0),
        };
        self
    }

    /// Share of the points kept at lateral distance `lateral` (1 outside the sticker's reach).
    pub fn keep_share(&self, lateral: f64) -> f64 {
        if lateral < self.dot_radius {
            self.dot_keep
        } else if lateral < self.ring_radius {
            self.ring_keep
        } else if lateral < self.halo_radius {
            self.halo_keep
        } else {
            1.0
        }
    }

    /// Radius of everything the sticker changes in a scan: the disc, the cut, the halo.
    pub fn site_radius(&self) -> f64 {
        self.outer_radius.max(self.cut_radius.unwrap_or(0.0)).max(self.halo_radius)
    }

    /// (lateral distance from the axis, height along the normal above the centre).
    pub fn local(&self, p: DVec3) -> (f64, f64) {
        let d = p - v3(self.center);
        let n = v3(self.normal);
        let axial = d.dot(n);
        ((d - n * axial).length(), axial)
    }

    /// How far the sticker raises the surface at `p`.
    pub fn thickness(&self, p: DVec3) -> f64 {
        let reach = self.outer_radius + self.back_limit + 1.0;
        if (p - v3(self.center)).length_squared() > reach * reach {
            return 0.0;
        }
        let (lateral, axial) = self.local(p);
        if axial < -self.back_limit {
            return 0.0;
        }
        self.height * ramp(lateral, self.flat_radius, self.outer_radius)
    }
}

/// A camera position of a pass (part frame), looking at `target`.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct View {
    pub eye: V3,
    pub target: V3,
}

/// A region of space (part frame).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum Region {
    /// normal · p <= offset.
    HalfSpace { normal: V3, offset: f64 },
    Sphere { center: V3, radius: f64 },
    Aabb { min: V3, max: V3 },
    Outside { region: Box<Region> },
}

impl Region {
    pub fn contains(&self, p: DVec3) -> bool {
        match self {
            Region::HalfSpace { normal, offset } => v3(*normal).normalize().dot(p) <= *offset,
            Region::Sphere { center, radius } => (p - v3(*center)).length() <= *radius,
            Region::Aabb { min, max } => (0..3).all(|i| p[i] >= min[i] && p[i] <= max[i]),
            Region::Outside { region } => !region.contains(p),
        }
    }
}

/// Patchy loss of coverage (dark spots, reflections): value noise on a `cell_mm` lattice, points
/// where it is below `fraction` are lost.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Dropout {
    pub cell_mm: f64,
    pub fraction: f64,
    /// Only inside this region (`None` = everywhere).
    #[serde(default)]
    pub only_z_above: Option<f64>,
}

/// A part of the pass sampled at another point spacing.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct DensityRegion {
    pub region: Region,
    pub spacing_mm: f64,
}

/// One family of cross-line laser artefacts: lines where the planes normal · p = k spacing
/// (+ phase) cut the surface.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct LineFamily {
    pub normal: V3,
    pub spacing_mm: f64,
    pub phase: f64,
}

/// Stray points behind the surface along the laser lines (as in Revo's cross-line scans: lines
/// about 2 mm wide, 0.6-1.0 mm behind the surface, most with normals facing the wrong way).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Strays {
    pub region: Region,
    pub families: Vec<LineFamily>,
    pub width_mm: f64,
    pub depth_mm: [f64; 2],
    /// Share of the surface points on a line that get a stray behind them.
    pub fraction: f64,
    /// Share of strays whose normal faces into the surface.
    pub wrong_way: f64,
    pub noise_mm: f64,
}

/// How the pass's normals are made.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum NormalMode {
    /// As a scanner makes them: a plane fit over the measured (noisy) neighbours in each view's
    /// depth image (`normal_window` pixels each way, no neighbours across a depth jump), facing
    /// the scanner. They blend at edges and carry the noise; nothing of the true surface is used.
    #[serde(alias = "estimated")]
    DepthImage,
    /// The true surface normal.
    True,
}

/// The depth error's structure. Real Revo noise is correlated over about 0.5 mm (neighbour
/// correlation 0.25-0.42 at 0.2-0.5 mm in S1-S3), with fine straight hatching on top.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct NoiseModel {
    /// Share of the noise variance that is correlated in space (the rest is white per point).
    pub correlated_share: f64,
    /// Correlation length of the correlated part (Gaussian correlation exp(-r^2 / L^2)).
    pub correlation_mm: f64,
    /// Hatching: straight parallel ridges of this amplitude ...
    pub hatch_mm: f64,
    /// ... and this period, along one direction per pass.
    pub hatch_period_mm: f64,
    /// Extra white noise of a single frame (standard deviation at normal incidence, grows at
    /// grazing like the rest): the normals come from single depth images, which are noisier than
    /// the fused points, so real normals are 5° off where the points are smooth.
    #[serde(default)]
    pub frame_noise_mm: f64,
}

impl Default for NoiseModel {
    fn default() -> Self {
        NoiseModel { correlated_share: 0.8, correlation_mm: 0.55, hatch_mm: 0.012, hatch_period_mm: 1.5, frame_noise_mm: 0.04 }
    }
}

/// Streaks along the laser lines in grazing fringes (S3's outer face near the rim: +-0.5 mm,
/// 4 x as rough as S1 there): points within `width_mm` of a line move along the ray by up to
/// `amplitude_mm`, varying smoothly along the line (over `cell_mm`); fully from
/// `full_incidence_deg` on, fading out towards `from_incidence_deg`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Streaks {
    pub families: Vec<LineFamily>,
    pub width_mm: f64,
    pub amplitude_mm: f64,
    pub from_incidence_deg: f64,
    pub full_incidence_deg: f64,
    pub cell_mm: f64,
}

/// How the views are fused into one cloud: a blue-noise thinning to the pass's `spacing_mm` (no
/// two points of a sheet closer than that; about 0.72 / spacing^2 points per mm^2), then `keep`
/// of the points kept at random, and `duplicates` of them keep a second point from another view
/// close by (the real previews have nearest-neighbour p10 of 0.04-0.3 mm).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Fusion {
    pub keep: f64,
    pub duplicates: f64,
}

impl Default for Fusion {
    fn default() -> Self {
        Fusion { keep: 0.8, duplicates: 0.12 }
    }
}

fn default_window() -> u32 {
    1
}

/// One pass: where the scanner looked from and what went wrong in it.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PassSpec {
    pub name: String,
    pub views: Vec<View>,
    /// Smallest distance between fused points of a sheet (see [`Fusion`]). 0.4 gives about
    /// 4 points per mm^2 as in Revo's `raw_preview.ply` (2.9-4.8).
    pub spacing_mm: f64,
    /// Noise along the view ray (standard deviation) at normal incidence.
    pub noise_mm: f64,
    /// sigma = noise / cos(incidence)^grazing_noise (cos clamped at 0.15).
    pub grazing_noise: f64,
    /// Points seen more obliquely than this are lost (and coverage thins out over the last 6°).
    pub max_incidence_deg: f64,
    /// Every point moved this far away from the scanner along its ray (a scan that sits too deep).
    pub depth_bias_mm: f64,
    /// Every point shifted by this (part frame): a registration error between passes.
    pub offset_mm: V3,
    /// Every point moved by this rigid motion (part frame) after the offset: what an alignment
    /// leaves (real Align All: 0.2-0.27 deg and 0.4-0.8 mm), so the gap between two passes changes
    /// across the part.
    #[serde(default)]
    pub pose_error: Rigid,
    /// Part frame -> the frame the pass is written in. Pass 1 is the identity.
    pub scan_pose: Rigid,
    /// A table (or marker block) only this pass sees, part frame. Not part of the truth.
    pub table: Option<Shape>,
    /// Points are kept only inside all of these.
    pub clip: Vec<Region>,
    pub dropout: Option<Dropout>,
    pub density: Vec<DensityRegion>,
    pub strays: Option<Strays>,
    #[serde(default)]
    pub streaks: Option<Streaks>,
    pub normals: NormalMode,
    /// Half-width (pixels) of the depth-image window of [`NormalMode::DepthImage`].
    #[serde(default = "default_window")]
    pub normal_window: u32,
    #[serde(default)]
    pub noise: NoiseModel,
    #[serde(default)]
    pub fusion: Fusion,
}

impl PassSpec {
    pub fn new(name: impl Into<String>, views: Vec<View>) -> Self {
        PassSpec {
            name: name.into(),
            views,
            spacing_mm: 0.4,
            noise_mm: 0.025,
            grazing_noise: 1.0,
            max_incidence_deg: 78.0,
            depth_bias_mm: 0.0,
            offset_mm: [0.0; 3],
            pose_error: Rigid::IDENTITY,
            scan_pose: Rigid::IDENTITY,
            table: None,
            clip: Vec::new(),
            dropout: None,
            density: Vec::new(),
            strays: None,
            streaks: None,
            normals: NormalMode::DepthImage,
            normal_window: 1,
            noise: NoiseModel::default(),
            fusion: Fusion::default(),
        }
    }

    /// A pass as Revo's Fused export gives it (the "marker holes" scenes): denser (0.23 mm,
    /// about 10.8 points per mm^2), smoother, every point kept; the sticker sites are cut out to
    /// 5.5 mm (set on the stickers).
    pub fn fused_export(mut self) -> Self {
        self.spacing_mm = 0.23;
        self.noise_mm = 0.018;
        self.fusion = Fusion { keep: 1.0, duplicates: 0.0 };
        self.normal_window = 2;
        self.noise.hatch_mm = 0.01;
        self.noise.frame_noise_mm = 0.0;
        self
    }
}

/// How a probe checks a mesh (part frame = pass 1's frame).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum ProbeKind {
    /// The mesh must pass within `tol` mm of the point (edges, corners).
    On { tol: f64 },
    /// No mesh within `radius` mm of the point (an open hole or slot, the inside of a wall).
    Clear { radius: f64 },
    /// Along the line through the point with `normal`, the mesh must cross at `expect` mm
    /// (signed along the normal, 0 = on the true surface) within `tol`; searched over ±`range`.
    Height { normal: V3, expect: f64, tol: f64, range: f64 },
    /// The line through the point along `axis` must not cross the mesh within ±`half_length`
    /// (a hole or slot open all the way through: a cap or a bottom anywhere on its axis fails,
    /// which a `clear` sphere at mid-depth does not see).
    Through { axis: V3, half_length: f64 },
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Probe {
    pub point: V3,
    #[serde(flatten)]
    pub check: ProbeKind,
    pub note: String,
}

impl Probe {
    pub fn on(point: DVec3, tol: f64, note: impl Into<String>) -> Self {
        Probe { point: point.to_array(), check: ProbeKind::On { tol }, note: note.into() }
    }
    pub fn clear(point: DVec3, radius: f64, note: impl Into<String>) -> Self {
        Probe { point: point.to_array(), check: ProbeKind::Clear { radius }, note: note.into() }
    }
    pub fn through(point: DVec3, axis: DVec3, half_length: f64, note: impl Into<String>) -> Self {
        Probe { point: point.to_array(), check: ProbeKind::Through { axis: axis.normalize().to_array(), half_length }, note: note.into() }
    }
    pub fn height(point: DVec3, normal: DVec3, expect: f64, tol: f64, note: impl Into<String>) -> Self {
        Probe {
            point: point.to_array(),
            check: ProbeKind::Height { normal: normal.normalize().to_array(), expect, tol, range: 2.0 },
            note: note.into(),
        }
    }
}

/// Something a mesher must keep, remove or reproduce, with the expected outcome in words and as
/// probes.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Feature {
    pub id: String,
    pub kind: String,
    pub expect: String,
    pub params: serde_json::Value,
    pub probes: Vec<Probe>,
}

/// A complete scene.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Scene {
    pub name: String,
    pub description: String,
    pub seed: u64,
    /// The true part (no stickers, no table), part frame.
    pub part: Shape,
    /// A box that holds the part.
    pub bounds: [V3; 2],
    pub stickers: Vec<Sticker>,
    pub passes: Vec<PassSpec>,
    pub features: Vec<Feature>,
    /// Cell size of the reference mesh.
    pub truth_cell_mm: f64,
    pub tags: Vec<String>,
}

/// Views on a ring round `center`: `count` eyes at horizontal distance `radius` and height
/// `height` above it, looking at the centre, the first at `start_deg`.
pub fn ring_views(center: DVec3, radius: f64, height: f64, count: usize, start_deg: f64) -> Vec<View> {
    (0..count)
        .map(|k| {
            let a = (start_deg + 360.0 * k as f64 / count as f64).to_radians();
            View { eye: (center + DVec3::new(radius * a.cos(), radius * a.sin(), height)).to_array(), target: center.to_array() }
        })
        .collect()
}

pub fn view(eye: DVec3, target: DVec3) -> View {
    View { eye: eye.to_array(), target: target.to_array() }
}
