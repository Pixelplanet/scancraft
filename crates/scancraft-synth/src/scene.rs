//! What a synthetic scene is: the true part, the marker stickers on it, the passes that scan it
//! and the features a mesher must get right (with probes that say how to check them).

use glam::DVec3;
use serde::{Deserialize, Serialize};

use crate::sdf::{ramp, v3, Rigid, Shape, V3};

/// A marker sticker: a disc `height` thick that follows the surface. It is flat out to
/// `flat_radius` and back on the surface at `outer_radius`. Revo lists only the white dot
/// (`dot_radius`) in its marker file. Not part of the true surface.
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
    /// Points within this radius are cut out of the scan (as Revo does), `None` = kept.
    pub cut_radius: Option<f64>,
    /// On the table of this pass (index), not on the part.
    pub on_table_of_pass: Option<usize>,
}

impl Sticker {
    /// A standard sticker (11 mm across, 0.4 mm thick, 3 mm dot) at `center` with `normal`.
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
        }
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
    /// Plane fit of the noisy neighbours on the same sheet, facing the scanner (as a scanner's
    /// depth images give them; rounded at edges, noisy).
    Estimated,
    /// The true surface normal.
    True,
}

/// One pass: where the scanner looked from and what went wrong in it.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PassSpec {
    pub name: String,
    pub views: Vec<View>,
    /// Point spacing of the fused cloud.
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
    /// Part frame -> the frame the pass is written in. Pass 1 is the identity.
    pub scan_pose: Rigid,
    /// A table (or marker block) only this pass sees, part frame. Not part of the truth.
    pub table: Option<Shape>,
    /// Points are kept only inside all of these.
    pub clip: Vec<Region>,
    pub dropout: Option<Dropout>,
    pub density: Vec<DensityRegion>,
    pub strays: Option<Strays>,
    pub normals: NormalMode,
}

impl PassSpec {
    pub fn new(name: impl Into<String>, views: Vec<View>) -> Self {
        PassSpec {
            name: name.into(),
            views,
            spacing_mm: 0.3,
            noise_mm: 0.03,
            grazing_noise: 1.0,
            max_incidence_deg: 78.0,
            depth_bias_mm: 0.0,
            offset_mm: [0.0; 3],
            scan_pose: Rigid::IDENTITY,
            table: None,
            clip: Vec::new(),
            dropout: None,
            density: Vec::new(),
            strays: None,
            normals: NormalMode::Estimated,
        }
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
