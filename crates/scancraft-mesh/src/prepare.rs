//! Scanner-data preparation before [`crate::implicit::reconstruct`] (the app's Reconstruct): the
//! same point clean-up and marker removal Make Solid does ([`crate::solid`]), for an open surface.
//!
//! Why: the implicit surface keeps what the points say. On the Revo scans of the shell part two
//! things in the points are not the part (the user, who owns it, confirmed the top is smooth):
//!
//! 1. **Laser-line strays** (cross-line laser mode): a lattice of curved lines of points about
//!    0.8 mm *behind* an inside surface, inside the wall, most of them facing the other way. Along
//!    a line the stray layer is as dense as the surface, so the mesher's own sparse-layer test
//!    ([`crate::implicit`] `dense_layers`) keeps them, and the surface becomes a field of pits up to
//!    1 mm deep that earlier notes mistook for moulded lettering. They are recognised by the wall:
//!    measured from a stray, the opposite face of the wall is ~0.8 mm away instead of the wall's
//!    3 mm (Make Solid's ghost test, `prep::drop_ghost_layers`), and the rest by the mesher's
//!    layer test. Single points off their local sheet (`prep::drop_off_surface`) go too. Make
//!    Solid also recognises them directly in each scan (`prep::drop_behind_strays`, see
//!    [`PrepareConfig::laser_strays`]).
//! 2. **Marker stickers:** flat discs ~0.4 mm proud, ~11 mm across. Every listed marker is cut out
//!    to ≥ 6.5 mm and refilled from a quadric fitted to the ring around it (the solid's marker
//!    stage, [`crate::solid::markers`]); after meshing [`crate::solid::fair_marker_sites`] blends the
//!    mesh inside the cut onto the ring fit of the mesh, so the site ends flush.
//!
//! **Trade-off** ([`PrepareConfig::suppress_artefacts`]): the clean-up drops points standing more
//! than max(0.15 mm, 4 × local noise) off their local sheet within 1 mm, and a second layer
//! 0.6–2.5 mm in front of or behind a sheet when its wall thickness disagrees with the wall map.
//! Raised or sunken relief narrower than about a millimetre on a thin wall can look the same and is
//! then smoothed away; relief on a plain sheet (the synthetic ribs of the mesher's tests) stays (test
//! `prepare_keeps_small_relief`). For scanner data the default is on; turn it off for parts with
//! fine engraving.

use crate::solid::markers::{Marker, MarkerConfig, MarkerOutcome};
use crate::solid::{grid, prep, PointKind, WorkPoint};
use glam::Vec3;
use scancraft_core::{ColorRgba, Point3D, PointCloud};

/// Settings of [`prepare_points`].
#[derive(Clone, Debug)]
pub struct PrepareConfig {
    /// Drop laser-line strays and points off their local sheet (see the module docs).
    pub suppress_artefacts: bool,
    /// With `suppress_artefacts`: also drop points 0.4–1.6 mm behind a denser sheet with an empty
    /// gap between (`prep::drop_behind_strays`). Off by default here: Make Solid runs it on each
    /// scan's own points, where it works (inner lattice relief 0.041 → 0.005 mm on the Revo
    /// scans); on the merged model, where the scans are mixed, it took only part of the strays,
    /// and the wall test and the mesher's own layer test then missed the rest (0.004 → 0.064 mm).
    pub laser_strays: bool,
    /// A point further off the plane of its same-facing neighbours within 1 mm than
    /// max(this, 4 × their RMS) is dropped.
    pub off_surface_mm: f32,
    /// Largest wall thickness searched for the wall map.
    pub wall_search_mm: f32,
    /// Cut the listed markers out and refill them from the ring around them.
    pub remove_markers: bool,
    pub markers: MarkerConfig,
}

impl Default for PrepareConfig {
    fn default() -> Self {
        PrepareConfig {
            suppress_artefacts: true,
            laser_strays: false,
            off_surface_mm: 0.15,
            wall_search_mm: 6.0,
            remove_markers: true,
            // The merged model has a 0.5 mm pitch: a denser fill would outweigh the points around
            // it in the implicit surface.
            markers: MarkerConfig { fill_spacing_mm: 0.45, ..MarkerConfig::default() },
        }
    }
}

/// What [`prepare_points`] did.
#[derive(Clone, Debug, Default)]
pub struct PrepareReport {
    pub input_points: usize,
    pub dropped_off_surface: usize,
    pub dropped_ghosts: usize,
    pub dropped_laser_strays: usize,
    pub wall_median_mm: f32,
    pub wall_measurements: usize,
    /// The marker sites that were cut (copies of one sticker merged) and per site what happened.
    pub marker_sites: Vec<Marker>,
    pub marker_outcomes: Vec<MarkerOutcome>,
    pub marker_cut_points: usize,
    pub marker_fill_points: usize,
}

impl PrepareReport {
    /// Sites that were cut and refilled.
    pub fn markers_filled(&self) -> usize {
        self.marker_outcomes.iter().filter(|o| o.fill.starts_with("quadric")).count()
    }
}

/// Cleans an oriented cloud (normals out of the material) for Reconstruct and removes the listed
/// markers (see the module docs). Markers are matched against every point whatever its pass (the
/// merged model has no pass ids). Points without a usable normal are passed through unchanged.
pub fn prepare_points(cloud: &PointCloud, markers: &[Marker], cfg: &PrepareConfig) -> (PointCloud, PrepareReport) {
    let mut rep = PrepareReport { input_points: cloud.len(), ..Default::default() };
    let usable = |p: &Point3D| {
        let n = Vec3::from_array(p.normal);
        Vec3::from_array(p.position).is_finite() && n.is_finite() && n.length() > 1e-3
    };
    let mut passthrough: Vec<Point3D> = Vec::new();
    let mut pts: Vec<WorkPoint> = Vec::with_capacity(cloud.len());
    // Per working point: the input point it came from (fill points get `None`).
    let mut source: Vec<Option<u32>> = Vec::with_capacity(cloud.len());
    for (i, p) in cloud.points.iter().enumerate() {
        if usable(p) {
            pts.push(WorkPoint { p: Vec3::from_array(p.position), n: Vec3::from_array(p.normal).normalize(), scan: 0, kind: PointKind::Measured, area: 0.0 });
            source.push(Some(i as u32));
        } else {
            passthrough.push(*p);
        }
    }
    // The clean-up functions drop points in place: carry the source index in `area` (not used
    // before the solve) as an f32 bit pattern.
    for (w, s) in pts.iter_mut().zip(&source) {
        w.area = f32::from_bits(s.unwrap_or(u32::MAX));
    }
    if cfg.suppress_artefacts && pts.len() > 50 {
        if cfg.laser_strays {
            rep.dropped_laser_strays = prep::drop_behind_strays(&mut pts, 2.0, 0.4, 1.6);
        }
        rep.dropped_off_surface = prep::drop_off_surface(&mut pts, 1.0, cfg.off_surface_mm, 4.0);
        let map = {
            let pos: Vec<Vec3> = pts.iter().map(|p| p.p).collect();
            let g = grid::PointGrid::new(&pos, 1.0);
            prep::ThicknessMap::build(&pts, &pos, &g, cfg.wall_search_mm, None)
        };
        rep.wall_median_mm = map.global_median;
        rep.wall_measurements = map.measurements;
        // Without a measured wall (a single sheet, a solid block) there is no ghost test.
        if map.measurements >= 20 {
            rep.dropped_ghosts = prep::drop_ghost_layers(&mut pts, &map, cfg.wall_search_mm);
        }
    }
    if cfg.remove_markers && !markers.is_empty() {
        let sites: Vec<Marker> = markers.iter().cloned().map(|mut m| {
            m.scan = None;
            m
        }).collect();
        let (sites, _, outcomes) = crate::solid::marker_stage(&mut pts, &sites, &cfg.markers, None, 0.0);
        rep.marker_cut_points = outcomes.iter().map(|o| o.cut_points).sum();
        rep.marker_fill_points = outcomes.iter().map(|o| o.fill_points).sum();
        rep.marker_sites = sites;
        rep.marker_outcomes = outcomes;
    }
    // Fill points stand for as many observations as a typical kept point.
    let fill_confidence = {
        let mut c: Vec<f32> = pts.iter().filter(|w| w.kind == PointKind::Measured).filter_map(|w| cloud.points.get(w.area.to_bits() as usize)).map(|p| p.confidence).filter(|c| c.is_finite() && *c > 0.0).collect();
        if c.is_empty() {
            1.0
        } else {
            let m = c.len() / 2;
            *c.select_nth_unstable_by(m, |a, b| a.total_cmp(b)).1
        }
    };
    let mut out: Vec<Point3D> = pts
        .iter()
        .map(|w| match (w.kind, cloud.points.get(w.area.to_bits() as usize)) {
            (PointKind::Measured, Some(p)) => *p,
            _ => Point3D { position: w.p.to_array(), normal: w.n.to_array(), color: ColorRgba::GRAY, confidence: fill_confidence },
        })
        .collect();
    out.extend(passthrough);
    (PointCloud::from_points(out), rep)
}
