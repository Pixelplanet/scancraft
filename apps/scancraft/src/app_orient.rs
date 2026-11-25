//! Orientation of the whole project: Auto Orient (a proposal to accept or reject), Orient by
//! Datums (a plane, an axis and a point clicked on the model become world datums) and Scale
//! from Lengths on an imported pass. Auto Orient and the datums share
//! [`ScanCraftApp::move_project_frame`], which turns the frame every placed pass, the merged
//! model and the mesh live in.
//!
//! A child module of `app`, so it shares the app's private state.

use std::sync::Arc;

use glam::Vec3;
use scancraft_align::{
    AutoOrient, AxisDatum, DatumError, DatumFrame, Datums, Fit, FitConfig, OrientConfig, PlaneDatum, Primitive, ScaleEstimate, Surface, WorldAxis, WorldPlane,
    compose, datum_frame, scale_from_lengths,
};
use scancraft_core::{BoundingBox, Point3D, PointCloud, Transform};
use scancraft_render::{CameraPreset, GpuPoint};

use super::{Capture, ScanCraftApp, Tone};
use crate::model::{CloudLayer, JobResult, MergedLayer, MeshLayer, Node, Stage, Tool, tinted_points};
use crate::project::{self, AlignState};

/// The display sample's colours: the model, the plane datum's inliers, the axis datum's.
const DATUM_COLOURS: [[u8; 3]; 3] = [[170, 178, 190], [70, 140, 225], [225, 145, 50]];

/// An Auto Orient proposal: the candidate frame, what it was found on, and the model drawn
/// where it would sit.
pub struct OrientPreview {
    pub result: AutoOrient,
    /// "Merged model" or "Scan passes".
    pub source: &'static str,
    /// A sample of the model placed by `result.transform`.
    pub layer: CloudLayer,
}

/// Which datum the next click on the model fits.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DatumKind {
    Plane,
    Axis,
    Point,
}

/// The Orient by Datums tool: the surface the picks fit on, the picked datums, the frame they
/// imply and the model drawn where it would go.
pub struct DatumPicking {
    /// Working points in the project frame (the frame the datums are defined in): the merged
    /// model if there is one, else the placed passes, at most 600k points.
    pub surface: Arc<Surface>,
    /// Click candidates in the project frame (at most 150k).
    pub targets: Arc<Vec<Vec3>>,
    pub kind: DatumKind,
    pub tolerance_mm: f32,
    /// (seed, fit, onto, flip); the seed is kept so a tolerance change can refit.
    pub plane: Option<(Vec3, Fit, WorldPlane, bool)>,
    pub axis: Option<(Vec3, Fit, WorldAxis, bool)>,
    pub origin: Option<Vec3>,
    /// The frame the picked datums imply; `None` before the first datum.
    pub frame: Option<Result<DatumFrame, DatumError>>,
    /// Draw the model where the frame would put it.
    pub preview: bool,
    /// The display sample (at most 400k), inliers tinted, placed by `frame` when previewed.
    pub layer: CloudLayer,
    /// Indices into `surface`'s points that `layer` samples, kept for the inlier tinting.
    display: Vec<u32>,
    /// The transform `layer` is placed by (the camera is refit only when it changes).
    applied: Option<Transform>,
}

/// One measured length: two picked points and the true distance between them, mm.
#[derive(Clone, Copy, Debug)]
pub struct ScalePair {
    pub a: Vec3,
    pub b: Option<Vec3>,
    pub true_mm: f32,
}

/// The Scale from Lengths tool: the pass it scales and the lengths picked on it so far.
pub struct ScalePicking {
    pub pass: u32,
    pub pairs: Vec<ScalePair>,
    /// Click candidates: a spread of the pass's layer points, in the frame the layer is drawn
    /// in (its own for an unaligned pass, the project's for a placed one).
    pub targets: Arc<Vec<Vec3>>,
    /// The pass drawn while the tool is open: a copy, so imported points whose file had no
    /// normals get estimated ones (the stored [0, 0, 1] placeholders shade flat).
    pub layer: CloudLayer,
}

/// The project-frame points orientation tools work on: the merged model's when there is one,
/// else every placed pass's layer points (both are already placed). None when there are none.
/// Points loaded from a file without normals carry [0, 0, 1] everywhere (a saved merged.ply
/// has none): auto-orient's evidence, the datum fits' normal gates and lit shading all need
/// real ones, so estimate them then. An in-session merge keeps its own.
fn with_real_normals(mut points: Vec<Point3D>) -> Vec<Point3D> {
    let missing = points.iter().filter(|p| p.normal == [0.0, 0.0, 1.0] || p.normal == [0.0; 3]).count();
    if points.is_empty() || missing * 20 <= points.len() * 19 {
        return points;
    }
    let positions: Vec<[f32; 3]> = points.iter().map(|p| p.position).collect();
    for (p, n) in points.iter_mut().zip(scancraft_io::normals::estimate_normals(&positions, &scancraft_io::NormalEstimation::default())) {
        p.normal = n;
    }
    points
}

fn orient_points(app: &ScanCraftApp) -> Option<(&'static str, Vec<Point3D>)> {
    if let Some(m) = &app.merged {
        if m.layer.len() > 0 {
            return Some(("Merged model", with_real_normals(m.layer.cloud.points.clone())));
        }
    }
    let points: Vec<Point3D> = app.passes.iter().filter(|p| p.meta.alignment.is_placed()).flat_map(|p| p.layer.cloud.points.iter().copied()).collect();
    (!points.is_empty()).then_some(("Scan passes", with_real_normals(points)))
}

fn oriented(p: &(Vec3, Vec3)) -> Point3D {
    Point3D { position: p.0.to_array(), normal: p.1.to_array(), ..Point3D::new([0.0; 3]) }
}

/// [`tinted_points`](crate::model::tinted_points) for several colours at once: `marks[i]`
/// picks `colours`' entry for point `i`.
fn tinted_marks(cloud: &PointCloud, marks: &[u8], colours: &[[u8; 3]; 3]) -> Vec<GpuPoint> {
    let light = Vec3::new(-0.35, -0.75, 0.55).normalize();
    cloud
        .points
        .iter()
        .zip(marks)
        .map(|(p, &m)| {
            let n = Vec3::from(p.normal);
            let k = 0.35 + 0.65 * n.dot(light).abs();
            let c = |v: u8| (v as f32 * k).min(255.0) as u8;
            let colour = colours[m as usize];
            let mut g = GpuPoint::from_point(p);
            g.color = [c(colour[0]), c(colour[1]), c(colour[2]), 255];
            g
        })
        .collect()
}

impl DatumPicking {
    fn new(points: Vec<Point3D>, generation: u64) -> Self {
        // The working surface at most 600k points, the display layer at most 400k, the click
        // candidates at most 150k — even strides of the same order.
        let stride = (points.len() / 600_000).max(1);
        let surface = Arc::new(Surface::new(points.iter().step_by(stride).map(|p| (Vec3::from(p.position), Vec3::from(p.normal).normalize_or_zero())).collect()));
        let stride = (surface.points().len() / 150_000).max(1);
        let targets = Arc::new(surface.points().iter().step_by(stride).map(|p| p.0).collect());
        let stride = (surface.points().len() / 400_000).max(1);
        let display: Vec<u32> = (0..surface.points().len()).step_by(stride).map(|i| i as u32).collect();
        let cloud = PointCloud::from_points(display.iter().map(|&i| oriented(&surface.points()[i as usize])).collect());
        let mut layer = CloudLayer::new(cloud, generation);
        layer.gpu = Arc::new(tinted_points(&layer.cloud, DATUM_COLOURS[0]));
        DatumPicking {
            surface,
            targets,
            kind: DatumKind::Plane,
            tolerance_mm: FitConfig::default().tolerance_mm,
            plane: None,
            axis: None,
            origin: None,
            frame: None,
            preview: true,
            layer,
            display,
            applied: None,
        }
    }

    /// The picked datums as [`datum_frame`] takes them.
    pub fn datums(&self) -> Datums {
        Datums {
            plane: self.plane.as_ref().and_then(|(_, fit, onto, flip)| match fit.primitive {
                Primitive::Plane { point, normal } => Some(PlaneDatum { point, normal, onto: *onto, flip: *flip }),
                _ => None,
            }),
            axis: self.axis.as_ref().and_then(|(_, fit, onto, flip)| match fit.primitive {
                Primitive::Cylinder { point, axis, .. } => Some(AxisDatum { point, dir: axis, onto: *onto, flip: *flip }),
                _ => None,
            }),
            origin: self.origin,
        }
    }

    /// The transform the layer is shown placed by (the frame's when Preview is on).
    pub fn applied_transform(&self) -> Option<Transform> {
        match (&self.frame, self.preview) {
            (Some(Ok(f)), true) => Some(f.transform),
            _ => None,
        }
    }
}

impl ScanCraftApp {
    /// Whether orientation tools have something to work on.
    fn has_orient_input(&self) -> bool {
        self.merged.as_ref().is_some_and(|m| m.layer.len() > 0) || self.passes.iter().any(|p| p.meta.alignment.is_placed() && p.layer.len() > 0)
    }

    /// Why the project cannot be re-oriented now.
    pub fn orient_blocked(&self) -> Option<&'static str> {
        if self.busy.is_some() {
            return Some("Another operation is running");
        }
        if self.capture != Capture::Idle {
            return Some("Stop the capture first");
        }
        if self.align_preview.is_some() || self.poses_preview.is_some() {
            return Some("Accept or reject the proposed alignment first");
        }
        if !self.has_orient_input() {
            return Some("No points to orient: scan or import first");
        }
        None
    }

    /// Moves the project frame by `t` (new <- old): every placed pass, the merged model and the
    /// mesh move together; passes not yet placed stay in their own frames (undoable).
    pub(super) fn move_project_frame(&mut self, t: Transform, label: &str) {
        if !self.can_edit() {
            return;
        }
        self.record(self.snapshot(label));
        self.clear_previews();
        let placed: Vec<(u32, Transform, AlignState)> = self.passes.iter().filter(|p| p.meta.alignment.is_placed()).map(|p| (p.id, p.meta.transform, p.meta.alignment)).collect();
        for (id, old, state) in placed {
            self.place_pass(id, compose(&t, &old), state);
        }
        let merged = self.merged.as_ref().map(|m| {
            let cloud = PointCloud::from_points(project::place(m.layer.cloud.points.clone(), &t));
            (cloud, m.layer.visible, m.passes.clone(), m.summary.clone())
        });
        if let Some((cloud, visible, passes, summary)) = merged {
            let mut layer = MergedLayer::new(cloud, self.generation(), passes, summary);
            layer.layer.visible = visible;
            self.merged = Some(layer);
        }
        let mesh = self.mesh.as_ref().map(|m| {
            let mut mesh = (*m.mesh).clone();
            mesh.vertices = project::place(std::mem::take(&mut mesh.vertices), &t);
            mesh.bbox = BoundingBox::default();
            for v in &mesh.vertices {
                mesh.bbox.extend(Vec3::from(v.position));
            }
            (mesh, m.visible, m.analysis.clone())
        });
        if let Some((mesh, visible, analysis)) = mesh {
            let mut layer = MeshLayer::new(mesh, analysis, self.generation());
            layer.visible = visible;
            self.mesh = Some(layer);
        }
        self.holes = None;
        // Picking state in the old frame no longer holds (as invalidate_results does for the
        // Isolate Object and AI Segment tools).
        self.picking = None;
        self.datum_picking = None;
        self.scale_picking = None;
        if self.object_picking.take().is_some() && self.tool == Some(Tool::IsolateObject) {
            self.tool = None;
        }
        if self.ai_picking.take().is_some() && self.tool == Some(Tool::AiSegment) {
            self.tool = None;
        }
        if matches!(self.tool, Some(Tool::PointPairs(_) | Tool::Datums | Tool::Scale(_))) {
            self.tool = None;
        }
        self.framed_radius = 0.0;
        self.fit_view();
    }

    // ------------------------------------------------------------------ Auto Orient

    /// Finds an orientation of the placed model from its own faces (a job), shown as a
    /// proposal before anything moves.
    pub fn auto_orient(&mut self) {
        if self.orient_blocked().is_some() {
            return;
        }
        let Some((source, points)) = orient_points(self) else { return };
        let points: Vec<scancraft_align::Oriented> = points.iter().map(|p| (Vec3::from(p.position), Vec3::from(p.normal).normalize_or_zero())).collect();
        self.spawn_job("Finding the orientation", move || match scancraft_align::auto_orient(&points, &OrientConfig::default()) {
            Some(result) if result.turned_deg < 0.3 && result.transform.translation.length() < 0.2 => JobResult::NoChange("Already square to the axes".into()),
            Some(result) => JobResult::Oriented(Box::new(result), source),
            None => JobResult::Failed("Could not find an orientation: too few points".into()),
        });
    }

    /// The Auto Orient job's result: the model drawn where it would sit, as a proposal.
    pub(super) fn show_orient(&mut self, result: AutoOrient, source: &'static str) {
        let Some((_, points)) = orient_points(self) else { return };
        let up = result.up_source.label();
        let stride = (points.len() / 400_000).max(1);
        let sample: Vec<Point3D> = points.into_iter().step_by(stride).collect();
        let g = self.preview_generation();
        let layer = CloudLayer::new(PointCloud::from_points(project::place(sample, &result.transform)), g);
        // With at most one face direction and no axis or rim the evidence is thin: warn.
        let weak = result.faces.len() <= 1 && result.axis.is_none() && result.rim.is_none();
        self.clear_previews();
        self.close_tool();
        let bbox = layer.cloud.bbox;
        self.orient_preview = Some(OrientPreview { result, source, layer });
        self.stage = Stage::Align;
        // The preview shows the model where it would sit: frame the camera on that, not on
        // where the data still is, and look at the floor from the usual angle.
        if !bbox.is_empty() {
            self.camera.fit_bounding_box(&bbox);
            self.camera.set_view_preset(CameraPreset::Isometric);
        }
        self.say(if weak { Tone::Warning } else { Tone::Success }, format!("Found an orientation from the {up} · check it, then Accept"));
    }

    /// Accepts the proposed orientation: the whole project moves into it (undoable).
    pub fn accept_orient(&mut self) {
        let Some(preview) = self.orient_preview.take() else { return };
        if !self.can_edit() {
            self.orient_preview = Some(preview);
            return;
        }
        let turned = preview.result.turned_deg;
        self.move_project_frame(preview.result.transform, "orient the project");
        self.say(Tone::Success, format!("Oriented the project: turned {turned:.1}°, base on the grid"));
    }

    /// Rejects the proposed orientation; the project stays as it is.
    pub fn reject_orient(&mut self) {
        if self.orient_preview.take().is_some() {
            self.say(Tone::Normal, "Rejected the proposed orientation; the project is unchanged");
        }
    }

    // ------------------------------------------------------------------ Orient by Datums

    /// Why Orient by Datums cannot start now.
    pub fn datums_blocked(&self) -> Option<&'static str> {
        self.orient_blocked()
    }

    /// Opens the tool: the surface the picks fit on, built synchronously (~60 ms per 200k).
    pub(super) fn start_datum_picking(&mut self) -> bool {
        if self.datum_picking.is_some() {
            return true;
        }
        if self.datums_blocked().is_some() {
            return false;
        }
        let Some((_, points)) = orient_points(self) else { return false };
        let g = self.preview_generation();
        self.datum_picking = Some(DatumPicking::new(points, g));
        true
    }

    /// A click on the model fits the kind selected now; a same-kind pick replaces the old one
    /// (keeping its assignment).
    pub fn datum_pick(&mut self, p: Vec3) {
        let Some(d) = &self.datum_picking else { return };
        let surface = Arc::clone(&d.surface);
        let kind = d.kind;
        let config = FitConfig { tolerance_mm: d.tolerance_mm, ..FitConfig::default() };
        match kind {
            DatumKind::Plane => {
                let Some(fit) = surface.fit_plane(p, &config) else {
                    self.say(Tone::Warning, "No plane found there: try a flatter spot or a larger tolerance");
                    return;
                };
                let onto = self.datum_picking.as_ref().and_then(|d| d.plane.as_ref().map(|(_, _, onto, _)| *onto)).unwrap_or(WorldPlane::XY);
                let flip = self.datum_plane_flip(&fit, onto);
                if let Some(d) = &mut self.datum_picking {
                    d.plane = Some((p, fit, onto, flip));
                }
            }
            DatumKind::Axis => {
                let Some(fit) = surface.fit_cylinder(p, &config) else {
                    self.say(Tone::Warning, "No cylinder found there: click on a round wall, or raise the tolerance");
                    return;
                };
                let (onto, flip) = match self.datum_picking.as_ref().and_then(|d| d.axis.as_ref().map(|(_, _, onto, flip)| (*onto, *flip))) {
                    Some(kept) => kept,
                    None => {
                        let onto = self.default_axis_onto(&fit);
                        (onto, self.datum_axis_flip(&fit, onto))
                    }
                };
                if let Some(d) = &mut self.datum_picking {
                    d.axis = Some((p, fit, onto, flip));
                }
            }
            DatumKind::Point => {
                let Some((i, _)) = surface.nearest(p) else { return };
                let point = surface.points()[i as usize].0;
                if let Some(d) = &mut self.datum_picking {
                    d.origin = Some(point);
                }
            }
        }
        self.refresh_datums();
    }

    /// Whether a fresh plane needs flipping for the model's centroid to end up at z >= 0 (the
    /// part sits ON the floor, not hanging under it).
    fn datum_plane_flip(&self, fit: &Fit, onto: WorldPlane) -> bool {
        let Primitive::Plane { point, normal } = fit.primitive else { return false };
        let datums = Datums { plane: Some(PlaneDatum { point, normal, onto, flip: false }), ..Datums::default() };
        let Ok(frame) = datum_frame(&datums) else { return false };
        let Some(d) = &self.datum_picking else { return false };
        let n = d.display.len().max(1) as f32;
        let centroid = d.display.iter().map(|&i| d.surface.points()[i as usize].0).sum::<Vec3>() / n;
        frame.transform.transform_point3(centroid).z < 0.0
    }

    /// The world axis a fresh axis datum goes onto: the plane's world normal when there is no
    /// plane or the axis is within 20° of the plane's normal (it can only position, not turn),
    /// else X (it lies in the plane).
    fn default_axis_onto(&self, fit: &Fit) -> WorldAxis {
        let Primitive::Cylinder { axis, .. } = fit.primitive else { return WorldAxis::Z };
        match self.datum_picking.as_ref().and_then(|d| d.plane.as_ref()).map(|(_, f, onto, _)| (f.primitive, *onto)) {
            Some((Primitive::Plane { normal, .. }, onto)) if axis.dot(normal).abs() >= 20.0f32.to_radians().cos() => match onto {
                WorldPlane::XY => WorldAxis::Z,
                WorldPlane::YZ => WorldAxis::X,
                WorldPlane::ZX => WorldAxis::Y,
            },
            Some((Primitive::Plane { .. }, _)) => WorldAxis::X,
            _ => WorldAxis::Z,
        }
    }

    /// The flip a fresh axis datum needs: a cylinder's direction is undirected but the frame's
    /// parallel-axis check is directed, so an anti-parallel axis would Conflict. Take the first
    /// flip the frame accepts.
    fn datum_axis_flip(&self, fit: &Fit, onto: WorldAxis) -> bool {
        let Primitive::Cylinder { point, axis, .. } = fit.primitive else { return false };
        let Some(d) = &self.datum_picking else { return false };
        for flip in [false, true] {
            let datums = Datums { plane: d.datums().plane, axis: Some(AxisDatum { point, dir: axis, onto, flip }), ..Datums::default() };
            if datum_frame(&datums).is_ok() {
                return flip;
            }
        }
        false
    }

    /// Recomputes the frame and the display layer after a datum or an option changed.
    pub fn refresh_datums(&mut self) {
        let Some(d) = self.datum_picking.as_mut() else { return };
        let any = d.plane.is_some() || d.axis.is_some() || d.origin.is_some();
        d.frame = any.then(|| datum_frame(&d.datums()));
        let applied = d.applied_transform();
        let changed = applied != d.applied;
        d.applied = applied;
        let surface = Arc::clone(&d.surface);
        let display = std::mem::take(&mut d.display);
        let plane_inliers: Vec<u32> = d.plane.as_ref().map(|(_, f, ..)| f.inliers.clone()).unwrap_or_default();
        let axis_inliers: Vec<u32> = d.axis.as_ref().map(|(_, f, ..)| f.inliers.clone()).unwrap_or_default();
        let g = self.preview_generation();
        let mut marks = vec![0u8; surface.points().len()];
        for &i in &plane_inliers {
            marks[i as usize] = 1;
        }
        for &i in &axis_inliers {
            marks[i as usize] = 2;
        }
        let mut points: Vec<Point3D> = display.iter().map(|&i| oriented(&surface.points()[i as usize])).collect();
        if let Some(t) = applied {
            points = project::place(points, &t);
        }
        let mut layer = CloudLayer::new(PointCloud::from_points(points), g);
        let layer_marks: Vec<u8> = display.iter().map(|&i| marks[i as usize]).collect();
        layer.gpu = Arc::new(tinted_marks(&layer.cloud, &layer_marks, &DATUM_COLOURS));
        if let Some(d) = self.datum_picking.as_mut() {
            d.display = display;
            d.layer = layer;
        }
        // The camera follows the model where the frame puts it (not while only tinting changed).
        if changed {
            if let Some(d) = &self.datum_picking {
                let b = d.layer.cloud.bbox;
                if !b.is_empty() {
                    self.camera.fit_bounding_box(&b);
                }
            }
        }
    }

    /// A new fit tolerance: the picked plane and axis refit from their kept seeds.
    pub fn set_datum_tolerance(&mut self, tolerance_mm: f32) {
        let Some(d) = &mut self.datum_picking else { return };
        if (d.tolerance_mm - tolerance_mm).abs() < 1e-6 {
            return;
        }
        d.tolerance_mm = tolerance_mm;
        let config = FitConfig { tolerance_mm, ..FitConfig::default() };
        let surface = Arc::clone(&d.surface);
        let plane = d.plane.as_ref().map(|(s, _, onto, flip)| (*s, *onto, *flip));
        let axis = d.axis.as_ref().map(|(s, _, onto, flip)| (*s, *onto, *flip));
        let mut lost = Vec::new();
        if let Some((seed, onto, flip)) = plane {
            match surface.fit_plane(seed, &config) {
                Some(fit) => d.plane = Some((seed, fit, onto, flip)),
                None => {
                    d.plane = None;
                    lost.push("plane");
                }
            }
        }
        if let Some((seed, onto, flip)) = axis {
            match surface.fit_cylinder(seed, &config) {
                Some(fit) => d.axis = Some((seed, fit, onto, flip)),
                None => {
                    d.axis = None;
                    lost.push("axis");
                }
            }
        }
        if !lost.is_empty() {
            self.say(Tone::Warning, format!("The {} no longer fits within the tolerance", lost.join(" and ")));
        }
        self.refresh_datums();
    }

    /// The Pick switch: what the next click fits.
    pub fn set_datum_kind(&mut self, kind: DatumKind) {
        if let Some(d) = &mut self.datum_picking {
            d.kind = kind;
        }
    }

    /// The picked plane's new world assignment.
    pub fn set_plane_onto(&mut self, onto: WorldPlane) {
        if let Some(d) = &mut self.datum_picking {
            if let Some((s, f, _, flip)) = d.plane.take() {
                d.plane = Some((s, f, onto, flip));
            }
        }
        self.refresh_datums();
    }

    /// The picked axis's new world assignment.
    pub fn set_axis_onto(&mut self, onto: WorldAxis) {
        if let Some(d) = &mut self.datum_picking {
            if let Some((s, f, _, flip)) = d.axis.take() {
                d.axis = Some((s, f, onto, flip));
            }
        }
        self.refresh_datums();
    }

    /// Turns the model over to the other side of the plane.
    pub fn set_plane_flip(&mut self, flip: bool) {
        if let Some(d) = &mut self.datum_picking {
            if let Some((s, f, onto, _)) = d.plane.take() {
                d.plane = Some((s, f, onto, flip));
            }
        }
        self.refresh_datums();
    }

    /// Turns the model around on its axis.
    pub fn set_axis_flip(&mut self, flip: bool) {
        if let Some(d) = &mut self.datum_picking {
            if let Some((s, f, onto, _)) = d.axis.take() {
                d.axis = Some((s, f, onto, flip));
            }
        }
        self.refresh_datums();
    }

    /// Removes one picked datum.
    pub fn remove_datum(&mut self, kind: DatumKind) {
        if let Some(d) = &mut self.datum_picking {
            match kind {
                DatumKind::Plane => d.plane = None,
                DatumKind::Axis => d.axis = None,
                DatumKind::Point => d.origin = None,
            }
        }
        self.refresh_datums();
    }

    /// Whether the model is drawn where the frame would put it.
    pub fn set_datum_preview(&mut self, preview: bool) {
        if let Some(d) = &mut self.datum_picking {
            d.preview = preview;
        }
        self.refresh_datums();
    }

    /// Why Apply cannot run in the Datums card (the generic tool reasons come first).
    pub fn datum_apply_blocked(&self) -> Option<String> {
        match &self.datum_picking {
            Some(d) => match &d.frame {
                None => Some("Pick a datum first".to_string()),
                Some(Err(e)) => Some(e.to_string()),
                Some(Ok(_)) => None,
            },
            None => Some("Pick a datum first".to_string()),
        }
    }

    /// Apply: the picked datums become the world frame; everything placed moves (undoable).
    pub(super) fn apply_datums(&mut self) {
        let Some(d) = &self.datum_picking else { return };
        let Some(Ok(frame)) = &d.frame else { return };
        let t = frame.transform;
        let plane_rms = d.plane.as_ref().map(|(_, f, ..)| f.rms_mm);
        let axis_rms = d.axis.as_ref().map(|(_, f, ..)| f.rms_mm);
        self.move_project_frame(t, "orient the project by datums");
        let mut message = "Oriented the project by datums".to_string();
        if let Some(rms) = plane_rms {
            message.push_str(&format!(" · plane {rms:.2} mm RMS"));
        }
        if let Some(rms) = axis_rms {
            message.push_str(&format!(" · axis {rms:.2} mm RMS"));
        }
        self.say(Tone::Success, message);
    }

    // ------------------------------------------------------------------ Scale from Lengths

    /// The pass Scale from Lengths acts on: the selected one if it is imported, else the only
    /// imported pass when there is exactly one.
    pub fn scale_target(&self) -> Option<u32> {
        let selected = match self.selection {
            Some(Node::Pass(id)) | Some(Node::Frame(id, _)) => Some(id),
            _ => None,
        };
        selected
            .filter(|id| self.passes.iter().any(|p| p.id == *id && p.meta.origin.is_imported()))
            .or_else(|| {
                let imported: Vec<u32> = self.passes.iter().filter(|p| p.meta.origin.is_imported()).map(|p| p.id).collect();
                (imported.len() == 1).then(|| imported[0])
            })
    }

    /// Why `id` cannot be scaled now: it works on an imported pass's stored surface (a captured
    /// pass already has its true size).
    pub fn scale_blocked(&self, id: Option<u32>) -> Option<&'static str> {
        if self.busy.is_some() {
            return Some("Another operation is running");
        }
        if self.capture != Capture::Idle {
            return Some("Stop the capture first");
        }
        let Some(id) = id else { return Some("Select an imported pass") };
        match self.passes.iter().find(|p| p.id == id) {
            Some(p) if p.meta.origin.is_imported() && p.surface.is_some() => None,
            Some(_) => Some("Only an imported pass can be scaled: a captured pass has its true size"),
            None => Some("Select an imported pass"),
        }
    }

    /// Opens the tool on `id`: click candidates are a spread of the pass's layer points.
    pub(super) fn start_scale_picking(&mut self, id: u32) -> bool {
        if self.scale_picking.as_ref().is_some_and(|s| s.pass == id) {
            return true;
        }
        if self.scale_blocked(Some(id)).is_some() {
            return false;
        }
        let Some(p) = self.passes.iter().find(|p| p.id == id) else { return false };
        let stride = (p.layer.len() / 150_000).max(1);
        let targets = Arc::new(p.layer.cloud.points.iter().step_by(stride).map(|p| Vec3::from(p.position)).collect());
        let points = with_real_normals(p.layer.cloud.points.clone());
        let g = self.preview_generation();
        let layer = CloudLayer::new(PointCloud::from_points(points), g);
        self.scale_picking = Some(ScalePicking { pass: id, pairs: Vec::new(), targets, layer });
        true
    }

    /// A click on the pass: the start of a new pair, or the far end of the open one (whose true
    /// length starts at the measured one).
    pub fn scale_pick(&mut self, p: Vec3) {
        let Some(s) = &mut self.scale_picking else { return };
        match s.pairs.last_mut() {
            Some(last) if last.b.is_none() => {
                last.b = Some(p);
                last.true_mm = (p - last.a).length();
            }
            _ => s.pairs.push(ScalePair { a: p, b: None, true_mm: 0.0 }),
        }
    }

    /// Removes length `i` (an open pair loses its start too).
    pub fn remove_scale_pair(&mut self, i: usize) {
        if let Some(s) = &mut self.scale_picking {
            if i < s.pairs.len() {
                s.pairs.remove(i);
            }
        }
    }

    /// The true length typed for pair `i`.
    pub fn set_scale_true(&mut self, i: usize, true_mm: f32) {
        if let Some(s) = &mut self.scale_picking {
            if let Some(pair) = s.pairs.get_mut(i) {
                pair.true_mm = true_mm;
            }
        }
    }

    /// The lengths' scale estimate (None until a pair is complete).
    pub fn scale_estimate(&self) -> Option<ScaleEstimate> {
        let s = self.scale_picking.as_ref()?;
        let pairs: Vec<(f32, f32)> = s.pairs.iter().filter_map(|pair| pair.b.map(|b| ((b - pair.a).length(), pair.true_mm))).collect();
        scale_from_lengths(&pairs)
    }

    /// Why Apply cannot run in the Scale card.
    pub fn scale_apply_blocked(&self) -> Option<&'static str> {
        match self.scale_estimate() {
            None => Some("Pick two points first"),
            Some(e) if (e.factor - 1.0).abs() < 1e-4 => Some("The factor is 1: type the true distance"),
            _ => None,
        }
    }

    /// Apply: scales the imported pass's surface by the measured factor and takes it back to
    /// its own frame (its alignment no longer fits).
    pub(super) fn apply_scale(&mut self, id: u32) {
        let Some(estimate) = self.scale_estimate() else { return };
        self.scale_pass(id, estimate.factor);
        self.close_tool();
    }
}
