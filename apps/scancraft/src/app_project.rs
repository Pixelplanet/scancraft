//! Project-level actions of the app: alignment (auto, point pairs, Align All, global
//! refinement) with a preview before accepting, merging, the reference pass, same-position
//! seeding, and saving / opening the project folder. A child module of `app`, so it shares the
//! app's private state.
//!
//! Alignment, refinement and merging use each pass's object only (`crate::isolation`) unless the
//! user turns on "Use whole scene".

use std::path::PathBuf;
use std::sync::Arc;

use glam::Vec3;
use scancraft_align::{Alignment, MergeConfig, MergeInput, PassCloud};
use scancraft_core::{PointCloud, Transform};
use scancraft_mesh::Mesher;
use scancraft_render::{Camera, CameraPreset};
use scancraft_slam::SeedState;

use super::{AlignPreview, PairPicking, PosesPreview, Proposal, ScanCraftApp, ToastAction, Tone};
use crate::alignment::{self, AlignAllOutcome, PassInput, RefineOutcome};
use crate::isolation::{self, MaskCache};
use crate::model::*;
use crate::project::{self, AlignMethod, AlignState, LoadedPass, PassMarker, PassSource, ProjectInfo, StoredPass, StoredProject};
use crate::storage::{same_path, OpenRequest, OpenTarget, Store};
use scancraft_io::bundle::Item;
use scancraft_io::lock::LockFile;
use scancraft_io::workdir::{self, WorkFolder};

/// What a background job needs of one pass: (id, project <- pass, placed, reference, surface,
/// isolation cache).
type JobPass = (u32, Transform, bool, bool, PassSource, MaskCache);

/// Said when a save into the project's own place is refused because someone else holds it.
const SAVE_ELSEWHERE: &str = "To keep your changes, save under another name in Save Project";

/// The name a project is shown by in the status and the history, from the path it was picked as:
/// a bundle or project file without its extension, an older `project.json` or a project folder
/// by the folder's name.
pub(crate) fn project_label(path: &std::path::Path) -> String {
    use scancraft_io::project::{has_extension, BUNDLE_EXTENSION, PROJECT_EXTENSION};
    let name = |p: &std::path::Path| p.file_name().map(|n| n.to_string_lossy().into_owned());
    let label = if has_extension(path, BUNDLE_EXTENSION) || (has_extension(path, PROJECT_EXTENSION) && !path.is_dir()) {
        path.file_stem().map(|n| n.to_string_lossy().into_owned())
    } else if path.is_file() {
        path.parent().and_then(name)
    } else {
        name(path)
    };
    label.filter(|l| !l.is_empty()).unwrap_or_else(|| "project".into())
}

/// Passes with fewer points take no part in Align All.
const ALIGN_ALL_MIN_POINTS: usize = 200;

/// Passes placed in the project frame, as (transform, surface, isolation), optionally leaving one
/// out.
fn placed(app: &ScanCraftApp, except: Option<u32>) -> Vec<(Transform, PassSource, MaskCache)> {
    app.passes.iter().filter(|p| p.meta.alignment.is_placed() && Some(p.id) != except).map(|p| (p.meta.transform, p.source(), p.mask.clone())).collect()
}

/// A pass as alignment sees it, in its own frame: its object only unless `whole_scene`.
fn pass_input(source: &PassSource, mask: &MaskCache, whole_scene: bool) -> Result<PassCloud, String> {
    Ok(isolation::pass_input(source.pass_cloud()?, mask, whole_scene))
}

/// The union of passes in the project frame, for aligning against. It keeps a table plane only
/// when it is a single pass (the passes stood differently on the table).
fn project_model(passes: &[(Transform, PassSource, MaskCache)], whole_scene: bool) -> Result<PassCloud, String> {
    let mut model = PassCloud::default();
    for (t, source, mask) in passes {
        let moved = scancraft_align::quality::moved(&pass_input(source, mask, whole_scene)?, t);
        model.points.extend(moved.points);
        model.views.extend(moved.views);
        model.support = if passes.len() == 1 { moved.support } else { None };
    }
    Ok(model)
}

/// The passes' clouds for a job, in their own frames.
fn job_inputs(passes: &[JobPass], whole_scene: bool) -> Result<Vec<PassInput>, String> {
    use rayon::prelude::*;
    passes
        .par_iter()
        .map(|(id, pose, placed, reference, source, mask)| Ok(PassInput { id: *id, pose: *pose, placed: *placed, reference: *reference, cloud: pass_input(source, mask, whole_scene)? }))
        .collect()
}

/// What Make Solid is built from: the placed passes as separate scans (the solid keeps one layer
/// per surface, from the most trusted pass), or, when no pass is placed (an opened older project
/// with only a merged model), the working cloud as one scan.
pub enum SolidInput {
    Passes {
        /// (scan id from 1, project <- pass, surface, isolation, merge weight, marker map in the pass frame)
        passes: Vec<(u8, Transform, PassSource, MaskCache, f32, Vec<PassMarker>)>,
        whole_scene: bool,
    },
    Cloud(PointCloud),
}

/// A listed marker lies on a pass's surface when one of its points is closer than this (mm).
pub const MARKER_ON_SURFACE_MM: f32 = 3.0;

/// A listed marker with a known radius below this (mm) is not a sticker: Revo Scan lists ~1 mm
/// entries on bracket edges and screw bosses (stickers are ~3 mm, see
/// `docs/protocol/revo-marker-framework.md`). Cutting those would erase real geometry.
pub const MIN_STICKER_RADIUS_MM: f32 = 2.0;

/// For each marker of `map` (pass frame), the normal Make Solid uses for it, or `None` when it
/// does not lie on `surfels` (no point within [`MARKER_ON_SURFACE_MM`]; Revo lists markers on
/// parts it cut from the preview), its centre is not finite or it is too small for a sticker
/// ([`MIN_STICKER_RADIUS_MM`]). The normal is the listed one
/// (Revo's marker file has it), else the sum of the surface normals around it (the solid's marker
/// stage selects the sticker's points by it, then refines it).
pub fn markers_on_surface(map: &[PassMarker], surfels: impl Iterator<Item = (Vec3, Vec3)>) -> Vec<Option<Vec3>> {
    use std::collections::HashMap;
    let r = MARKER_ON_SURFACE_MM;
    let cell = |p: Vec3| (p / r).floor().as_ivec3();
    // Each marker in the cells around its own, so one lookup per point finds every marker that
    // can be within `r`.
    let mut cells: HashMap<glam::IVec3, Vec<usize>> = HashMap::new();
    let (mut lo, mut hi) = (Vec3::splat(f32::MAX), Vec3::splat(f32::MIN));
    for (k, m) in map.iter().enumerate().filter(|(_, m)| m.centre.is_finite()) {
        let c = cell(m.centre);
        for d in (-1..=1).flat_map(|x| (-1..=1).flat_map(move |y| (-1..=1).map(move |z| glam::IVec3::new(x, y, z)))) {
            cells.entry(c + d).or_default().push(k);
        }
        lo = lo.min(m.centre - r);
        hi = hi.max(m.centre + r);
    }
    let mut near = vec![false; map.len()];
    let mut normal_sum = vec![Vec3::ZERO; map.len()];
    if !cells.is_empty() {
        for (p, n) in surfels.filter(|(p, _)| p.cmpge(lo).all() && p.cmple(hi).all()) {
            for &k in cells.get(&cell(p)).into_iter().flatten() {
                if p.distance_squared(map[k].centre) < r * r {
                    near[k] = true;
                    normal_sum[k] += n;
                }
            }
        }
    }
    map.iter()
        .zip(near.into_iter().zip(normal_sum))
        .map(|(m, (near, sum))| {
            let n = m.normal.unwrap_or(sum);
            let sticker = m.radius_mm.is_none_or(|r| !(r > 0.0 && r < MIN_STICKER_RADIUS_MM));
            (near && sticker && m.centre.is_finite() && n.is_finite() && n.length() > 1e-3).then_some(n)
        })
        .collect()
}

/// Which of a pass's markers Make Solid would use, for the interface (see
/// [`ScanCraftApp::marker_use`]).
#[derive(Debug, Clone, PartialEq)]
pub struct MarkerUse {
    key: u64,
    /// Per marker: on the pass's scanned surface (table and blocks included).
    pub on_scan: Vec<bool>,
    /// Per marker: on the isolated object; `None` until the pass's object has been isolated.
    pub on_object: Option<Vec<bool>>,
}

/// The solid's input points, its listed markers and the scan priority (most trusted first;
/// `None` when all passes weigh the same). Runs in the background job.
pub fn solid_points(input: SolidInput) -> Result<(Vec<scancraft_mesh::SolidPoint>, Vec<scancraft_mesh::solid::markers::Marker>, Option<Vec<u8>>), String> {
    use scancraft_mesh::SolidPoint;
    let ok = |p: Vec3, n: Vec3| p.is_finite() && n.is_finite() && n.length() > 1e-3;
    match input {
        SolidInput::Cloud(cloud) => Ok((
            cloud
                .points
                .iter()
                .map(|p| (Vec3::from_array(p.position), Vec3::from_array(p.normal)))
                .filter(|(p, n)| ok(*p, *n))
                .map(|(position, n)| SolidPoint { position, normal: n.normalize(), scan: 0 })
                .collect(),
            Vec::new(),
            None,
        )),
        SolidInput::Passes { passes, whole_scene } => {
            let mut points = Vec::new();
            let mut markers = Vec::new();
            for (scan, t, source, mask, _, map) in &passes {
                let surfels = solid_surfels(source, mask, whole_scene)?;
                markers.extend(placed_markers(*scan, t, map, &surfels));
                let m = t.to_mat4();
                points.extend(surfels.into_iter().map(|(p, n, _)| (m.transform_point3(p), (t.rotation * n).normalize_or_zero())).filter(|(p, n)| ok(*p, *n)).map(|(position, normal)| SolidPoint { position, normal, scan: *scan }));
            }
            let weights: Vec<f32> = passes.iter().map(|p| p.4).collect();
            let priority = (weights.iter().any(|w| (w - weights[0]).abs() > 1e-6)).then(|| {
                let mut order: Vec<(u8, f32)> = passes.iter().map(|p| (p.0, p.4)).collect();
                order.sort_by(|a, b| b.1.total_cmp(&a.1));
                order.into_iter().map(|o| o.0).collect()
            });
            Ok((points, markers, priority))
        }
    }
}

/// One pass as Save Project collects it: (id, name, frames, shown, meta, tracker, imported
/// surface, cleaned surface, colour keyframes, object mask, marker map).
type SavePass = (u32, String, u64, bool, project::PassMeta, SharedTracker, Option<Arc<PointCloud>>, Option<Arc<PointCloud>>, Vec<scancraft_color::RgbKeyframe>, Option<Arc<isolation::ObjectMask>>, Vec<PassMarker>);

/// What Save Project writes, collected from the app (see `ScanCraftApp::save_input`).
struct SaveInput {
    passes: Vec<SavePass>,
    reference: Option<u32>,
    next_pass_id: u32,
    merged: Option<(PointCloud, Vec<u32>)>,
    mesh: Option<scancraft_core::Mesh>,
    settings: serde_json::Value,
    notes: String,
}

impl SaveInput {
    /// Waits (at most `limit`) until the raw recordings of the passes have been written out: a
    /// capture that just stopped may still be flushing them. `false` when one is still writing.
    fn wait_for_recordings(&self, limit: std::time::Duration) -> bool {
        let stats: Vec<_> = self.passes.iter().filter_map(|p| p.4.capture.as_ref()).filter_map(|c| c.lock().ok().map(|l| l.recording_stats())).flatten().collect();
        scancraft_driver::recording::wait_finished(&stats, limit)
    }

    /// The project to write, named `name` (runs in the job: it locks the passes' trackers).
    fn into_project(self, name: String) -> Result<StoredProject, String> {
        let mut stored = Vec::with_capacity(self.passes.len());
        for (id, pass_name, frames, visible, meta, tracker, surface, edit, rgb_keyframes, mask, markers) in self.passes {
            let Ok(t) = tracker.lock() else { return Err("A pass's tracker is unavailable".into()) };
            stored.push(StoredPass { id, name: pass_name, frames, visible, meta, keyframes: t.keyframes().iter().map(scancraft_slam::Keyframe::to_data).collect(), surface, edit, rgb_keyframes, mask, markers });
        }
        Ok(StoredProject { name, reference: self.reference, next_pass_id: self.next_pass_id, passes: stored, merged: self.merged, cleaned: None, mesh: self.mesh, settings: self.settings, notes: self.notes })
    }
}

/// How long a save waits for raw recordings that are still being written out.
const RECORDING_FLUSH: std::time::Duration = std::time::Duration::from_secs(30);

/// Moves the raw recordings (and capture files written next to them) of a staging folder
/// (`ProjectInfo::staging`) into the project folder `to`: renamed when on the same drive, else
/// copied and then removed. Files the project already has are not replaced.
pub(crate) fn move_raw_files(from: &std::path::Path, to: &std::path::Path) -> std::io::Result<usize> {
    let mut moved = 0;
    for entry in std::fs::read_dir(from)?.flatten() {
        let name = entry.file_name().to_string_lossy().to_string();
        if !entry.path().is_dir() || !name.starts_with("pass-") {
            continue;
        }
        for file in std::fs::read_dir(entry.path())?.flatten() {
            let fname = file.file_name().to_string_lossy().to_string();
            if !matches!(fname.as_str(), "raw.bin" | "raw_rgb.bin") {
                continue;
            }
            let dest = to.join(&name).join(&fname);
            if dest.exists() && !scancraft_io::fsutil::stamp(&dest).is_some_and(|s| s.size == 0) {
                // Recorded into the project already (a continued pass after the first save).
                continue;
            }
            std::fs::create_dir_all(dest.parent().unwrap_or(to))?;
            if std::fs::rename(file.path(), &dest).is_err() {
                std::fs::copy(file.path(), &dest)?;
                let _ = std::fs::remove_file(file.path());
            }
            moved += 1;
        }
    }
    Ok(moved)
}

/// What a save says about moving the staged raw recordings: nothing when they moved, else why
/// they stayed where they were (nothing is lost: they are moved on the next save).
fn staging_note(staging: &std::path::Path, moved: std::io::Result<usize>) -> Option<String> {
    match moved {
        Ok(_) => None,
        Err(e) => {
            log::warn!("raw recordings in {} were not moved into the project: {e}", staging.display());
            Some(format!(" · the raw recordings stay in {} ({e}); the next save tries again", staging.display()))
        }
    }
}

/// The status message of a bundle save.
fn bundle_message(passes: &str, report: &scancraft_io::bundle::BundleReport) -> String {
    let mut message = format!("Saved {passes} to {} ({})", report.path.display(), scancraft_io::fsutil::format_bytes(report.bytes));
    if !report.left_out.is_empty() {
        let items: Vec<&str> = report.left_out.iter().map(|l| l.item.label()).collect();
        message.push_str(&format!(" · left out: {}", items.join(", ")));
    }
    message
}

/// Reads the project in `dir` and rebuilds its passes (runs in a job). The caller fills in the
/// path shown and the store.
fn read_opened(dir: &std::path::Path) -> Result<OpenedProject, String> {
    let stored = project::load(dir)?;
    let passes: Vec<LoadedPass> = {
        use rayon::prelude::*;
        stored.passes.into_par_iter().map(project::rebuild).collect()
    };
    let mesh = stored.mesh.map(|m| {
        let a = Mesher::analyze(&m);
        (m, a)
    });
    Ok(OpenedProject {
        path: dir.to_path_buf(),
        store: Store::None,
        recovered: false,
        read_only: false,
        notice: None,
        name: stored.name,
        next_pass_id: stored.next_pass_id,
        passes,
        merged: stored.merged,
        cleaned: stored.cleaned,
        mesh,
        settings: stored.settings,
        notes: stored.notes,
    })
}

/// Removes a leftover working folder that holds nothing to recover.
fn discard_quietly(dir: &std::path::Path) {
    match workdir::discard(dir) {
        Ok(()) => log::info!("removed the working folder {} (nothing unsaved in it)", dir.display()),
        Err(e) => log::warn!("could not remove the working folder {}: {e}", dir.display()),
    }
}

/// A pass's surface as Make Solid and Reconstruct's marker removal take it: its object only unless
/// `whole_scene`.
fn solid_surfels(source: &PassSource, mask: &MaskCache, whole_scene: bool) -> Result<Vec<(Vec3, Vec3, u32)>, String> {
    let surfels = source.surfels()?;
    if whole_scene {
        return Ok(surfels);
    }
    let oriented: Vec<(Vec3, Vec3)> = surfels.iter().map(|s| (s.0, s.1)).collect();
    Ok(isolation::mask_for(mask, &oriented).keep(surfels))
}

/// A pass's listed markers that lie on `surfels` ([`markers_on_surface`]), in the project frame.
fn placed_markers(scan: u8, t: &Transform, map: &[PassMarker], surfels: &[(Vec3, Vec3, u32)]) -> Vec<scancraft_mesh::solid::markers::Marker> {
    use scancraft_mesh::solid::markers::{Marker, MarkerSource};
    let m = t.to_mat4();
    let normals = markers_on_surface(map, surfels.iter().map(|s| (s.0, s.1)));
    map.iter()
        .zip(normals)
        .enumerate()
        .filter_map(|(k, (marker, n))| {
            Some(Marker {
                id: format!("S{scan}-M{k}"),
                center: m.transform_point3(marker.centre),
                normal: (t.rotation * n?).normalize(),
                scan: Some(scan),
                source: MarkerSource::List,
                radius_mm: marker.radius_mm.unwrap_or(0.0),
                score: 0.0,
            })
        })
        .collect()
}

/// The listed markers Reconstruct removes: those of [`solid_points`], without building the
/// points. Runs in the background job.
pub fn listed_markers(input: SolidInput) -> Result<Vec<scancraft_mesh::solid::markers::Marker>, String> {
    let SolidInput::Passes { passes, whole_scene } = input else { return Ok(Vec::new()) };
    let mut markers = Vec::new();
    for (scan, t, source, mask, _, map) in passes.iter().filter(|p| !p.5.is_empty()) {
        markers.extend(placed_markers(*scan, t, map, &solid_surfels(source, mask, whole_scene)?));
    }
    Ok(markers)
}

/// A pass read from a project (opened or added), shown by `layer`.
fn loaded_pass(lp: LoadedPass, layer: CloudLayer) -> ScanPass {
    ScanPass {
        id: lp.stored.id,
        name: lp.stored.name,
        layer,
        frames: lp.stored.frames,
        tracker: Arc::new(std::sync::Mutex::new(lp.tracker)),
        keyframes: lp.keyframes,
        meta: lp.stored.meta,
        tinted: None,
        mask: isolation::cache_with(lp.stored.mask),
        surface: lp.stored.surface,
        edit: lp.stored.edit,
        rgb_keyframes: lp.stored.rgb_keyframes,
        markers: lp.stored.markers,
    }
}

/// Display samples of a cloud for the point-pair panes.
fn sample(points: &[scancraft_core::Point3D], max: usize, colour: [u8; 3], to_frame: &Transform) -> Vec<(Vec3, Vec3, [u8; 3])> {
    let stride = (points.len() / max.max(1)).max(1);
    let m = to_frame.to_mat4();
    points.iter().step_by(stride).map(|p| (m.transform_point3(Vec3::from(p.position)), to_frame.rotation * Vec3::from(p.normal), colour)).collect()
}

fn fitted_camera(points: &[(Vec3, Vec3, [u8; 3])]) -> Camera {
    let mut camera = Camera::default();
    let mut b = scancraft_core::BoundingBox::default();
    for (p, _, _) in points {
        b.extend(*p);
    }
    camera.set_view_preset(CameraPreset::Isometric);
    if !b.is_empty() {
        camera.fit_bounding_box(&b);
    }
    camera
}

impl ScanCraftApp {
    pub fn pass_colour(&self, id: u32) -> [u8; 3] {
        if self.passes.iter().any(|p| p.id == id && p.meta.alignment == AlignState::Reference) {
            return PASS_COLOURS[0];
        }
        let k = self.passes.iter().filter(|p| p.meta.alignment != AlignState::Reference).position(|p| p.id == id).unwrap_or(0);
        PASS_COLOURS[1 + k % (PASS_COLOURS.len() - 1)]
    }

    /// Unsaved changes since the project was last saved or opened.
    pub fn dirty(&self) -> bool {
        self.has_geometry() && self.next_generation != self.project.saved_generation
    }

    pub fn reference_pass(&self) -> Option<u32> {
        self.passes.iter().find(|p| p.meta.alignment == AlignState::Reference).map(|p| p.id)
    }

    /// The pass the Align commands act on: the selected one if it is not the reference, else the
    /// first unaligned pass.
    pub fn align_target(&self) -> Option<u32> {
        let selected = match self.selection {
            Some(Node::Pass(id)) | Some(Node::Frame(id, _)) => Some(id),
            _ => None,
        };
        selected
            .filter(|id| self.passes.iter().any(|p| p.id == *id && p.meta.alignment != AlignState::Reference))
            .or_else(|| self.passes.iter().find(|p| p.meta.alignment == AlignState::Unaligned).map(|p| p.id))
    }

    /// Why the pass cannot be aligned now.
    pub fn align_blocked(&self, id: Option<u32>) -> Option<&'static str> {
        if self.busy.is_some() {
            return Some("Another operation is running");
        }
        if self.capture != super::Capture::Idle {
            return Some("Stop the capture first");
        }
        let Some(id) = id else { return Some("Every pass is aligned") };
        let Some(p) = self.passes.iter().find(|p| p.id == id) else { return Some("Select a pass") };
        if p.meta.alignment == AlignState::Reference {
            return Some("This pass is the reference: select another pass");
        }
        if p.layer.len() < 200 {
            return Some("The pass has too few points");
        }
        if placed(self, Some(id)).is_empty() {
            return Some("There is nothing to align to yet: no other pass is in the project frame");
        }
        None
    }

    pub fn pairs_blocked(&self, id: u32) -> Option<&'static str> {
        self.align_blocked(Some(id))
    }

    // ------------------------------------------------------------------ same position

    /// Seeds a new pass's tracker with the placed passes, so it starts in the project frame.
    /// Imported passes add their surface but no views. A cleaned pass adds its cleaned surface
    /// and, if it was captured, its keyframes as views (cleaning removes points, it does not move
    /// the pass, so the keyframe poses still hold).
    pub(super) fn seed_tracker(&self, tracker: &SharedTracker) -> bool {
        let sources = placed(self, None);
        let tracked: Vec<(Transform, &SharedTracker)> = sources.iter().filter_map(|(t, s, _)| if let PassSource::Tracked(tr) = s { Some((*t, tr)) } else { None }).collect();
        let guards: Vec<_> = tracked.iter().filter_map(|(t, tr)| tr.try_lock().ok().map(|g| (*t, g))).collect();
        if guards.len() != tracked.len() {
            return false;
        }
        let refs: Vec<(Transform, &scancraft_slam::Tracker)> = guards.iter().map(|(t, g)| (*t, &**g)).collect();
        let (mut points, mut views) = project::seed_from(&refs);
        drop(refs);
        drop(guards);
        let place = |t: &Transform, surface: &scancraft_core::PointCloud| project::fixed_cloud(surface).points.into_iter().map(move |(p, n)| (t.transform_point3(p), t.rotation * n)).collect::<Vec<_>>();
        for (t, source, _) in &sources {
            match source {
                PassSource::Fixed(surface) => points.extend(place(t, surface)),
                PassSource::Edited { surface, tracker: own } => {
                    points.extend(place(t, surface));
                    if let Some(own) = own {
                        let Ok(g) = own.try_lock() else { return false };
                        views.extend(project::seed_views(t, &g));
                    }
                }
                PassSource::Tracked(_) => {}
            }
        }
        tracker.lock().map(|mut t| t.seed_model(&points, &views)).unwrap_or(false)
    }

    /// The active pass's progress in locating itself in the project model.
    pub(super) fn seed_progress(&mut self, seed: SeedState) {
        let Some(pass) = self.active_pass.and_then(|id| self.passes.iter_mut().find(|p| p.id == id)) else { return };
        if pass.meta.alignment != AlignState::Locating {
            return;
        }
        match seed {
            SeedState::Registered => {
                pass.meta.alignment = AlignState::Aligned { method: AlignMethod::SamePosition, quality: None };
                pass.meta.transform = Transform::default();
                let name = pass.name.clone();
                self.say(Tone::Success, format!("{name} found its place in the project; capturing in the project frame"));
            }
            SeedState::Abandoned => {
                pass.meta.alignment = AlignState::Unaligned;
                let name = pass.name.clone();
                self.say(Tone::Warning, format!("{name} could not be located in the project; it is captured on its own and needs aligning afterwards"));
            }
            _ => {}
        }
    }

    // ------------------------------------------------------------------ placing passes

    /// Moves a pass to a new project <- pass transform (its layer follows).
    pub(super) fn place_pass(&mut self, id: u32, transform: Transform, alignment: AlignState) {
        let g = self.generation();
        let Some(p) = self.passes.iter_mut().find(|p| p.id == id) else { return };
        let delta = scancraft_align::compose(&transform, &p.meta.transform.inverse());
        let visible = p.layer.visible;
        p.layer = CloudLayer::new(PointCloud::from_points(project::place(p.layer.cloud.points.clone(), &delta)), g);
        p.layer.visible = visible;
        p.meta.transform = transform;
        p.meta.alignment = alignment;
        p.tinted = None;
    }

    /// Makes pass `id` the reference without moving anything. The project frame stays where it
    /// is: an orientation from Auto Orient or Datums is kept, and the merged model and mesh still
    /// fit. (It used to rebuild the frame around the new reference at the origin, which silently
    /// undid any orientation.) Align All and refinement hold the reference at its current pose,
    /// so it does not need to sit at the origin.
    pub(super) fn rebase_project(&mut self, id: u32) {
        for p in &mut self.passes {
            if p.id == id {
                p.meta.alignment = AlignState::Reference;
            } else if p.meta.alignment == AlignState::Reference {
                p.meta.alignment = AlignState::Aligned { method: AlignMethod::Auto, quality: None };
            }
        }
        // Only a label changed: the project counts as edited (saving records the reference).
        let _ = self.generation();
    }

    pub fn set_reference(&mut self, id: u32) {
        if !self.can_edit() || !self.passes.iter().any(|p| p.id == id && p.meta.alignment.is_placed()) {
            return;
        }
        let name = self.pass_name(id);
        self.record(self.snapshot(&format!("Make {name} the Reference")));
        self.clear_previews();
        self.rebase_project(id);
        self.say(Tone::Normal, format!("{name} is the reference; nothing moved"));
    }

    /// Back to its own frame (e.g. a wrong alignment).
    pub fn reset_alignment(&mut self, id: u32) {
        if !self.can_edit() || !self.passes.iter().any(|p| p.id == id && matches!(p.meta.alignment, AlignState::Aligned { .. })) {
            return;
        }
        let name = self.pass_name(id);
        self.record(self.snapshot(&format!("Reset Alignment of {name}")));
        self.clear_previews();
        self.place_pass(id, Transform::default(), AlignState::Unaligned);
        self.merged = None;
        self.invalidate_derived(&format!("{name} was moved"));
        self.say(Tone::Normal, format!("{name} is back in its own frame"));
    }

    pub fn set_included(&mut self, id: u32, included: bool) {
        if let Some(p) = self.passes.iter_mut().find(|p| p.id == id) {
            p.meta.included = included;
            self.next_generation += 1;
        }
    }

    pub fn set_weight(&mut self, id: u32, weight: f32) {
        if let Some(p) = self.passes.iter_mut().find(|p| p.id == id) {
            p.meta.weight = weight.clamp(0.1, 10.0);
            self.next_generation += 1;
        }
    }

    /// Make Solid's input (see [`SolidInput`]): the included, placed passes, as Merge takes them.
    /// Which of `pass`'s markers lie on its scan and on its isolated object, as Make Solid filters
    /// them ([`markers_on_surface`]). Computed once per content of the pass (its points, markers
    /// and isolation) and cached. `None` while the pass is being captured (the tracker's markers
    /// are triangulated on its surface) or when it has no markers.
    pub fn marker_use(&self, pass: &ScanPass) -> Option<Arc<MarkerUse>> {
        use std::hash::{Hash, Hasher};
        if pass.markers.is_empty() || (self.active_pass == Some(pass.id) && self.capture != super::Capture::Idle) {
            return None;
        }
        let mask = isolation::cached(&pass.mask);
        let mut h = std::collections::hash_map::DefaultHasher::new();
        (pass.layer.generation, pass.layer.len(), mask.as_ref().map(|m| (m.key, m.object.len()))).hash(&mut h);
        pass.edit.as_ref().map(Arc::as_ptr).hash(&mut h);
        pass.surface.as_ref().map(Arc::as_ptr).hash(&mut h);
        for m in &pass.markers {
            m.centre.to_array().map(f32::to_bits).hash(&mut h);
            m.normal.map(|n| n.to_array().map(f32::to_bits)).hash(&mut h);
        }
        let key = h.finish();
        let mut cache = self.marker_use.lock().ok()?;
        if let Some(u) = cache.get(&pass.id).filter(|u| u.key == key) {
            return Some(Arc::clone(u));
        }
        // Without waiting for a tracker a background job holds (asked again next frame).
        let surfels: Vec<(Vec3, Vec3, u32)> = match pass.source() {
            PassSource::Tracked(t) => t.try_lock().ok()?.fused_surfels().iter().map(|s| (s.position, s.normal.normalize_or_zero(), s.count)).collect(),
            source => source.surfels().ok()?,
        };
        let on =|normals: Vec<Option<Vec3>>| normals.iter().map(Option::is_some).collect::<Vec<bool>>();
        let on_scan = on(markers_on_surface(&pass.markers, surfels.iter().map(|s| (s.0, s.1))));
        // The object as Make Solid isolates it, if it has been isolated for these surfels.
        let fingerprint = || isolation::fingerprint(surfels.iter().map(|s| s.0));
        let on_object = mask.filter(|m| m.object.len() == surfels.len() && m.key == fingerprint()).map(|m| {
            let kept = surfels.iter().zip(&m.object).filter(|(_, o)| **o).map(|(s, _)| (s.0, s.1));
            on(markers_on_surface(&pass.markers, kept))
        });
        let u = Arc::new(MarkerUse { key, on_scan, on_object });
        cache.insert(pass.id, Arc::clone(&u));
        Some(u)
    }

    /// The listed markers Make Solid would use from the included, placed passes, and whether the
    /// number is exact (`false`: a pass is not isolated yet, so its markers on the table and
    /// blocks are still counted).
    pub fn solid_marker_count(&self) -> (usize, bool) {
        let mut exact = true;
        let mut count = 0;
        for p in self.passes.iter().filter(|p| p.meta.included && p.meta.alignment.is_placed()) {
            match self.marker_use(p) {
                Some(u) => {
                    let on = if self.settings.whole_scene { Some(&u.on_scan) } else { u.on_object.as_ref() };
                    exact &= on.is_some();
                    count += on.unwrap_or(&u.on_scan).iter().filter(|o| **o).count();
                }
                None => count += p.markers.len(),
            }
        }
        (count, exact)
    }

    pub(super) fn solid_input(&self) -> SolidInput {
        let placed: Vec<&ScanPass> = self.passes.iter().filter(|p| p.meta.included && p.meta.alignment.is_placed()).collect();
        if placed.is_empty() {
            return SolidInput::Cloud(self.working_cloud());
        }
        let passes = placed
            .iter()
            .enumerate()
            .map(|(i, p)| {
                ((i + 1).min(255) as u8, p.meta.transform, p.source(), p.mask.clone(), p.meta.weight, p.markers.clone())
            })
            .collect();
        SolidInput::Passes { passes, whole_scene: self.settings.whole_scene }
    }

    pub(crate) fn pass_name(&self, id: u32) -> String {
        self.passes.iter().find(|p| p.id == id).map(|p| p.name.clone()).unwrap_or_else(|| format!("Pass {id}"))
    }

    // ------------------------------------------------------------------ aligning

    /// Global registration of a pass against the project model (several feature grids, both
    /// directions), shown as a preview.
    pub fn align_auto(&mut self, id: u32) {
        if self.align_blocked(Some(id)).is_some() {
            return;
        }
        let Some((pass_source, mask)) = self.passes.iter().find(|p| p.id == id).map(|p| (p.source(), p.mask.clone())) else { return };
        let targets = placed(self, Some(id));
        let name = self.pass_name(id);
        let whole_scene = self.settings.whole_scene;
        self.clear_previews();
        self.spawn_job(&format!("Aligning {name}"), move || {
            let t0 = std::time::Instant::now();
            let source = match pass_input(&pass_source, &mask, whole_scene) {
                Ok(s) => s,
                Err(e) => return JobResult::Failed(e),
            };
            let target = match project_model(&targets, whole_scene) {
                Ok(m) => m,
                Err(e) => return JobResult::Failed(e),
            };
            match scancraft_align::align_robust(&source, &target, &scancraft_align::RobustConfig::default()) {
                Ok(r) => {
                    let check = AlignCheck {
                        reverse: r.best.reverse,
                        agreement: r.agreement,
                        consistent: r.consistent,
                        runs: r.candidates.len(),
                        agreeing: r.agreeing,
                        object_only: !whole_scene,
                    };
                    let alignment = Alignment { transform: r.transform(), quality: r.best.alignment.quality };
                    let message = format!("Found a placement for {name} in {:.1} s · check it, then Accept", t0.elapsed().as_secs_f32());
                    JobResult::Aligned { pass: id, method: AlignMethod::Auto, alignment, check: Some(check), message }
                }
                Err(e) => JobResult::Failed(format!("Could not align {name}: {e}. Try point pairs.")),
            }
        });
    }

    /// Why Align All cannot run now.
    pub fn align_all_blocked(&self) -> Option<&'static str> {
        if self.busy.is_some() {
            return Some("Another operation is running");
        }
        if self.capture != super::Capture::Idle {
            return Some("Stop the capture first");
        }
        if self.align_preview.is_some() || self.poses_preview.is_some() {
            return Some("Accept or reject the proposed alignment first");
        }
        if self.passes.iter().filter(|p| p.layer.len() >= ALIGN_ALL_MIN_POINTS).count() < 2 {
            return Some("Capture at least two passes first");
        }
        // Every pass is placed in the reference's frame, so the reference must take part.
        if !self.passes.iter().any(|p| p.meta.alignment == AlignState::Reference && p.layer.len() >= ALIGN_ALL_MIN_POINTS) {
            return Some("The reference pass has too few points · make another pass the reference");
        }
        None
    }

    /// Aligns every pass against every other at once and places them all from the reference;
    /// shown as a preview with a report of every pair.
    pub fn align_all(&mut self) {
        if self.align_all_blocked().is_some() {
            return;
        }
        let passes: Vec<JobPass> = self
            .passes
            .iter()
            .filter(|p| p.layer.len() >= ALIGN_ALL_MIN_POINTS)
            .map(|p| (p.id, p.meta.transform, p.meta.alignment.is_placed(), p.meta.alignment == AlignState::Reference, p.source(), p.mask.clone()))
            .collect();
        let whole_scene = self.settings.whole_scene;
        self.clear_previews();
        let n = passes.len();
        self.spawn_job(&format!("Aligning all {n} passes"), move || match job_inputs(&passes, whole_scene) {
            Ok(inputs) => JobResult::AlignedAll(Box::new(alignment::align_all_passes(&inputs, &scancraft_align::GraphConfig::default()))),
            Err(e) => JobResult::Failed(e),
        });
    }

    /// Preview layers: each pass placed at a proposed project <- pass transform, in its colour.
    fn preview_layers(&mut self, poses: &[(u32, Transform)]) -> Vec<(u32, CloudLayer)> {
        let mut layers = Vec::new();
        for (id, pose) in poses {
            let g = self.preview_generation();
            let colour = self.pass_colour(*id);
            let Some(p) = self.passes.iter().find(|p| p.id == *id) else { continue };
            let delta = scancraft_align::compose(pose, &p.meta.transform.inverse());
            let mut layer = CloudLayer::new(PointCloud::from_points(project::place(p.layer.cloud.points.clone(), &delta)), g);
            layer.gpu = Arc::new(tinted_points(&layer.cloud, colour));
            layers.push((*id, layer));
        }
        layers
    }

    pub(super) fn show_align_all(&mut self, outcome: AlignAllOutcome) {
        let poses: Vec<(u32, Transform)> = outcome.moved().filter_map(|p| p.pose.map(|t| (p.id, t))).collect();
        let placed = poses.len();
        let unconnected: Vec<String> = outcome.unconnected().map(|p| self.pass_name(p.id)).collect();
        let worst = outcome.moved().filter_map(|p| p.quality).map(|q| q.verdict()).max_by_key(|v| match v {
            scancraft_align::Verdict::Good => 0,
            scancraft_align::Verdict::Fair => 1,
            scancraft_align::Verdict::Poor => 2,
        });
        let seconds = outcome.seconds;
        if placed == 0 {
            self.say(Tone::Error, format!("Align All found no pass it could place ({:.0} s) · try point pairs", seconds));
            return;
        }
        let layers = self.preview_layers(&poses);
        self.poses_preview = Some(PosesPreview { proposal: Proposal::AlignAll(Box::new(outcome)), layers });
        self.picking = None;
        self.stage = Stage::Align;
        let mut message = format!("Placed {placed} pass{} in {:.0} s", if placed == 1 { "" } else { "es" }, seconds);
        if !unconnected.is_empty() {
            message.push_str(&format!(" · could not place {}", unconnected.join(", ")));
        }
        message.push_str(" · check the report, then Accept");
        let tone = if unconnected.is_empty() && worst == Some(scancraft_align::Verdict::Good) { Tone::Success } else { Tone::Warning };
        self.say(tone, message);
    }

    /// Accepts the proposed Align All or Refine All placements (undoable).
    pub fn accept_poses(&mut self) {
        let Some(preview) = self.poses_preview.take() else { return };
        if !self.can_edit() {
            self.poses_preview = Some(preview);
            return;
        }
        let (label, placements): (&str, Vec<(u32, Transform, AlignState)>) = match &preview.proposal {
            Proposal::AlignAll(o) => (
                "Align All",
                o.moved().filter_map(|p| p.pose.map(|t| (p.id, t, AlignState::Aligned { method: AlignMethod::AllPasses, quality: p.quality }))).collect(),
            ),
            Proposal::Refine(r) => (
                "Refine All",
                r.poses
                    .iter()
                    .map(|(id, t)| {
                        let quality = r.qualities.iter().find(|(q, _)| q == id).map(|(_, q)| *q);
                        (*id, *t, AlignState::Aligned { method: AlignMethod::Refined, quality })
                    })
                    .collect(),
            ),
        };
        self.record(self.snapshot(label));
        let n = placements.len();
        for (id, pose, state) in placements {
            self.place_pass(id, pose, state);
        }
        self.merged = None;
        self.invalidate_derived("The alignment changed");
        let next = self.passes.iter().find(|p| p.meta.alignment == AlignState::Unaligned).map(|p| p.name.clone());
        match (&preview.proposal, next) {
            (Proposal::Refine(_), _) => self.say(Tone::Success, format!("Refined {n} pass{}. Next: Merge", if n == 1 { "" } else { "es" })),
            (_, Some(name)) => self.say(Tone::Warning, format!("Placed {n} pass{}; {name} is still not aligned (try point pairs)", if n == 1 { "" } else { "es" })),
            (_, None) => self.say(Tone::Success, format!("Placed {n} pass{}. Next: Merge", if n == 1 { "" } else { "es" })),
        }
    }

    pub fn reject_poses(&mut self) {
        if self.poses_preview.take().is_some() {
            self.say(Tone::Normal, "Rejected the proposed alignment; the passes are unchanged");
        }
    }

    /// Opens the split view for picking pairs on `id` and the project model.
    pub(super) fn start_picking(&mut self, id: u32) -> bool {
        if self.pairs_blocked(id).is_some() {
            return false;
        }
        let Some(p) = self.passes.iter().find(|p| p.id == id) else { return false };
        // The pass in its own frame, the model in the project frame.
        let moving = sample(&p.layer.cloud.points, 60_000, self.pass_colour(id), &p.meta.transform.inverse());
        let mut model = Vec::new();
        let others: Vec<&ScanPass> = self.passes.iter().filter(|q| q.id != id && q.meta.alignment.is_placed()).collect();
        let per = 80_000 / others.len().max(1);
        for q in others {
            model.extend(sample(&q.layer.cloud.points, per, self.pass_colour(q.id), &Transform::default()));
        }
        self.align_preview = None;
        self.picking = Some(PairPicking {
            pass: id,
            pairs: Vec::new(),
            model_camera: fitted_camera(&model),
            moving_camera: fitted_camera(&moving),
            model: Arc::new(model),
            moving: Arc::new(moving),
        });
        true
    }

    /// Complete pairs picked so far.
    pub fn picked_pairs(&self) -> Vec<(Vec3, Vec3)> {
        self.picking.as_ref().map(|p| p.pairs.iter().filter_map(|(a, b)| Some(((*b)?, (*a)?))).collect()).unwrap_or_default()
    }

    /// Kabsch on the picked pairs, then ICP; shown as a preview.
    pub(super) fn align_point_pairs(&mut self, id: u32) {
        let pairs = self.picked_pairs();
        if pairs.len() < 3 {
            self.say(Tone::Warning, "Pick at least three pairs: a point on the model, then the same point on the pass");
            return;
        }
        if self.align_blocked(Some(id)).is_some() {
            return;
        }
        let Some(p) = self.passes.iter().find(|p| p.id == id) else { return };
        let (pass_source, mask) = (p.source(), p.mask.clone());
        let fine = p.meta.scan_config.accuracy.point_pitch_mm().max(1.0);
        let targets = placed(self, Some(id));
        let name = self.pass_name(id);
        let whole_scene = self.settings.whole_scene;
        self.spawn_job(&format!("Aligning {name} from {} pairs", pairs.len()), move || {
            let source = match pass_input(&pass_source, &mask, whole_scene) {
                Ok(s) => s,
                Err(e) => return JobResult::Failed(e),
            };
            let target = match project_model(&targets, whole_scene) {
                Ok(m) => m,
                Err(e) => return JobResult::Failed(e),
            };
            match scancraft_align::align_point_pairs(&source, &target, &pairs, fine) {
                Ok(alignment) => JobResult::Aligned { pass: id, method: AlignMethod::PointPairs, alignment, check: None, message: format!("Placed {name} from {} point pairs · check it, then Accept", pairs.len()) },
                Err(e) => JobResult::Failed(format!("Could not align {name}: {e}")),
            }
        });
    }

    pub(super) fn show_preview(&mut self, pass: u32, method: AlignMethod, alignment: Alignment, check: Option<AlignCheck>, message: String) {
        let g = self.preview_generation();
        let colour = self.pass_colour(pass);
        let Some(p) = self.passes.iter().find(|p| p.id == pass) else { return };
        let delta = scancraft_align::compose(&alignment.transform, &p.meta.transform.inverse());
        let cloud = PointCloud::from_points(project::place(p.layer.cloud.points.clone(), &delta));
        let mut layer = CloudLayer::new(cloud, g);
        layer.gpu = Arc::new(tinted_points(&layer.cloud, colour));
        self.align_preview = Some(AlignPreview { pass, method, alignment, check, layer });
        self.picking = None;
        if matches!(self.tool, Some(Tool::PointPairs(_))) {
            self.tool = None;
        }
        self.selection = Some(Node::Pass(pass));
        let tone = match alignment::checked_verdict(&alignment.quality, check.as_ref()) {
            scancraft_align::Verdict::Good => Tone::Success,
            _ => Tone::Warning,
        };
        self.say(tone, message);
    }

    pub fn accept_preview(&mut self) {
        let Some(preview) = self.align_preview.take() else { return };
        if !self.can_edit() {
            self.align_preview = Some(preview);
            return;
        }
        let name = self.pass_name(preview.pass);
        self.record(self.snapshot(&format!("Align {name}")));
        self.place_pass(preview.pass, preview.alignment.transform, AlignState::Aligned { method: preview.method, quality: Some(preview.alignment.quality) });
        self.merged = None;
        self.invalidate_derived(&format!("{name} was aligned"));
        let next = self.passes.iter().find(|p| p.meta.alignment == AlignState::Unaligned).map(|p| p.name.clone());
        match next {
            Some(n) => self.say(Tone::Success, format!("Aligned {name}. Next: {n}")),
            None => self.say(Tone::Success, format!("Aligned {name}. All passes are aligned: Refine, then Merge")),
        }
    }

    pub fn discard_preview(&mut self) {
        if self.align_preview.take().is_some() {
            self.say(Tone::Normal, "Rejected the proposed alignment; the pass is unchanged");
        }
    }

    /// Pairwise ICP between overlapping placed passes (the two faces of a wall paired too) and
    /// pose-graph optimisation, shown as a preview with every link before and after.
    pub fn refine_all(&mut self) {
        if self.refine_blocked().is_some() {
            return;
        }
        let passes: Vec<JobPass> = self
            .passes
            .iter()
            .filter(|p| p.meta.alignment.is_placed())
            .map(|p| (p.id, p.meta.transform, true, p.meta.alignment == AlignState::Reference, p.source(), p.mask.clone()))
            .collect();
        let whole_scene = self.settings.whole_scene;
        self.clear_previews();
        self.spawn_job("Refining all alignments", move || {
            let inputs = match job_inputs(&passes, whole_scene) {
                Ok(i) => i,
                Err(e) => return JobResult::Failed(e),
            };
            match alignment::refine_passes(&inputs) {
                Some(outcome) => JobResult::Refined(Box::new(outcome)),
                None => JobResult::NoChange("No passes overlap enough to refine".into()),
            }
        });
    }

    pub fn refine_blocked(&self) -> Option<&'static str> {
        if self.busy.is_some() {
            return Some("Another operation is running");
        }
        if self.capture != super::Capture::Idle {
            return Some("Stop the capture first");
        }
        if self.align_preview.is_some() || self.poses_preview.is_some() {
            return Some("Accept or reject the proposed alignment first");
        }
        (self.passes.iter().filter(|p| p.meta.alignment.is_placed()).count() < 2).then_some("Align at least two passes first")
    }

    /// A finished refinement: previewed, or rejected at once when it made the fit worse.
    pub(super) fn show_refinement(&mut self, outcome: RefineOutcome) {
        let (before, after) = (outcome.median_before(), outcome.median_after());
        let mm = |v: Option<f32>| v.map_or("—".to_string(), |v| format!("{v:.2} mm"));
        if outcome.worse() {
            self.say(Tone::Warning, format!("Refinement would make the fit worse (median pair error {} → {}); the alignment is unchanged", mm(before), mm(after)));
            return;
        }
        let message = format!("Refined {} links in {:.1} s · median pair error {} → {} · check it, then Accept", outcome.edges.len(), outcome.seconds, mm(before), mm(after));
        let layers = self.preview_layers(&outcome.poses);
        self.poses_preview = Some(PosesPreview { proposal: Proposal::Refine(Box::new(outcome)), layers });
        self.say(Tone::Success, message);
    }

    // ------------------------------------------------------------------ merging

    pub fn merge_blocked(&self) -> Option<&'static str> {
        if self.busy.is_some() {
            return Some("Another operation is running");
        }
        if self.capture != super::Capture::Idle {
            return Some("Stop the capture first");
        }
        if !self.passes.iter().any(|p| p.meta.included && p.meta.alignment.is_placed() && p.layer.len() > 0) {
            return Some("No aligned pass is included");
        }
        None
    }

    /// Fuses all included, placed passes into one model.
    pub fn merge_passes(&mut self) {
        if self.merge_blocked().is_some() {
            return;
        }
        let inputs: Vec<(u32, Transform, f32, PassSource, MaskCache)> = self
            .passes
            .iter()
            .filter(|p| p.meta.included && p.meta.alignment.is_placed())
            .map(|p| (p.id, p.meta.transform, p.meta.weight, p.source(), p.mask.clone()))
            .collect();
        // Imported scans have no scan settings: the captured passes set the pitch.
        let pitch = self
            .passes
            .iter()
            .filter(|p| p.meta.included && p.meta.alignment.is_placed() && !p.is_fixed())
            .map(|p| p.meta.scan_config.accuracy.point_pitch_mm())
            .reduce(f32::min)
            .unwrap_or_else(|| scancraft_driver::ScanConfig::default().accuracy.point_pitch_mm());
        let skipped = self.passes.iter().filter(|p| p.meta.included && !p.meta.alignment.is_placed()).count();
        let config = MergeConfig { pitch_mm: pitch, outliers: self.settings.merge_outliers.then_some((12, self.settings.merge_sigma)), ..MergeConfig::default() };
        let whole_scene = self.settings.whole_scene;
        self.clear_previews();
        self.spawn_undoable("Merging passes", "Merge", move || {
            let t0 = std::time::Instant::now();
            let mut merge_inputs = Vec::new();
            let mut left_out = 0;
            for (_, t, weight, source, mask) in &inputs {
                let surfels = match source.surfels() {
                    Ok(s) => s,
                    Err(e) => return JobResult::Failed(e),
                };
                let surfels = if whole_scene {
                    surfels
                } else {
                    let points: Vec<(Vec3, Vec3)> = surfels.iter().map(|s| (s.0, s.1)).collect();
                    let before = surfels.len();
                    let kept = isolation::mask_for(mask, &points).keep(surfels);
                    left_out += before - kept.len();
                    kept
                };
                let m = t.to_mat4();
                let points = surfels.into_iter().map(|(p, n, count)| (m.transform_point3(p), (t.rotation * n).normalize_or_zero(), count)).collect();
                merge_inputs.push(MergeInput { points, weight: *weight });
            }
            let r = scancraft_align::merge(&merge_inputs, &config);
            if r.points.is_empty() {
                return JobResult::Failed("The merge produced no points".into());
            }
            let mut message = format!(
                "Merged {} passes · {} points ({} outweighed by more accurate passes, {} isolated, {} outliers removed) in {:.1} s",
                inputs.len(),
                thousands(r.points.len()),
                thousands(r.outweighed),
                thousands(r.isolated),
                thousands(r.outliers),
                t0.elapsed().as_secs_f32()
            );
            if left_out > 0 {
                message.push_str(&format!(" · {} table and loose points left out", thousands(left_out)));
            }
            if skipped > 0 {
                message.push_str(&format!(" · {skipped} unaligned pass{} left out", if skipped == 1 { "" } else { "es" }));
            }
            JobResult::Merged { cloud: PointCloud::from_points(r.points), passes: inputs.iter().map(|i| i.0).collect(), message }
        });
    }

    pub fn discard_merged(&mut self) {
        if !self.can_edit() || self.merged.is_none() {
            return;
        }
        self.record(self.snapshot("Delete Merged Model"));
        self.merged = None;
        for p in &mut self.passes {
            p.layer.visible = true;
        }
        if self.selection == Some(Node::Merged) {
            self.selection = None;
        }
        self.say(Tone::Normal, "Deleted the merged model; the passes are unchanged");
    }

    // ------------------------------------------------------------------ project files

    /// What a save writes, collected on the UI thread (the job reads the trackers).
    fn save_input(&self) -> SaveInput {
        SaveInput {
            passes: self
                .passes
                .iter()
                .map(|p| (p.id, p.name.clone(), p.frames, p.layer.visible, p.meta.clone(), p.tracker.clone(), p.surface.clone(), p.edit.clone(), p.rgb_keyframes.clone(), isolation::cached(&p.mask), p.markers.clone()))
                .collect(),
            reference: self.reference_pass(),
            next_pass_id: self.next_pass_id,
            merged: self.merged.as_ref().map(|m| ((*m.layer.cloud).clone(), m.passes.clone())),
            mesh: self.mesh.as_ref().map(|m| (*m.mesh).clone()),
            settings: serde_json::json!({
                "min_mm": self.settings.min_mm,
                "max_mm": self.settings.max_mm,
                "merge_outliers": self.settings.merge_outliers,
                "merge_sigma": self.settings.merge_sigma,
                "whole_scene": self.settings.whole_scene,
                // The processing parameters of the later steps (informative: what the outputs
                // were last made with) and the app that saved them.
                "processing": self.processing_settings(),
                "saved_by": crate::capture::app_build(),
            }),
            notes: self.project.notes.clone(),
        }
    }

    /// The project's processing history: what was saved with it before, then this session's
    /// done steps (oldest first; at most `MAX_HISTORY` of a session).
    pub(crate) fn history_log(&self) -> Vec<serde_json::Value> {
        let mut log = self.project.history_before.clone();
        let utc = |t: std::time::SystemTime| scancraft_io::fsutil::format_utc(t.duration_since(std::time::UNIX_EPOCH).map(|d| d.as_secs()).unwrap_or(0));
        log.extend(self.history.timeline().into_iter().filter(|s| s.done).map(|s| serde_json::json!({ "utc": utc(s.at), "step": s.label, "app": crate::capture::version_label() })));
        log
    }

    /// Clean / Reconstruct / Make Solid / Fill Holes settings as saved with the project.
    pub(crate) fn processing_settings(&self) -> serde_json::Value {
        let s = &self.settings;
        serde_json::json!({
            "point_budget": s.point_budget,
            "plane_tolerance_mm": s.plane_tolerance_mm,
            "sor_neighbors": s.sor_neighbors,
            "sor_std_dev": s.sor_std_dev,
            "voxel_mm": s.voxel_mm,
            "mc_cell_mm": s.mc_cell_mm,
            "mesh_smoothness": s.mesh_smoothness,
            "mesh_auto_smooth": s.mesh_auto_smooth,
            "mesh_suppress_artefacts": s.mesh_suppress_artefacts,
            "mesh_remove_markers": s.mesh_remove_markers,
            "smooth_iterations": s.smooth_iterations,
            "smooth_keep_edges": s.smooth_keep_edges,
            "fill_max_mm": s.fill_max_mm,
            "fill_keep_largest": s.fill_keep_largest,
            "solid_cell_mm": s.solid_cell_mm,
            "solid_remove_markers": s.solid_remove_markers,
            "solid_sharp_top": s.solid_sharp_top,
            "decimate_ratio": s.decimate_ratio,
            "history": self.history_log(),
        })
    }

    /// Save Project into `target`: a project folder, or a bundle when `target` ends in
    /// `.scancraftz`. Saving where the project already is writes it there (a bundle tab: into its
    /// working folder, then back into the bundle). Saving anywhere else moves the tab there
    /// (Save As Project Folder, Save as Bundle…): the new place is locked, the old one unlocked,
    /// and a bundle's old working folder removed. The project is named after the folder or file.
    ///
    /// When the project file or bundle was changed by someone else since it was opened or last
    /// saved here, nothing is saved: the conflict question (`save_conflict`) asks first.
    pub fn save_project(&mut self, target: PathBuf) {
        self.save_project_with(target, false);
    }

    /// The conflict question's Save Anyway: saves over the version someone else saved.
    pub fn save_anyway(&mut self) {
        let Some(conflict) = self.save_conflict.take() else { return };
        if !self.project.path.as_deref().is_some_and(|p| same_path(p, &conflict.target)) {
            self.pending_close = false;
            self.say(Tone::Error, "Not saved: the project shown is no longer the one asked about");
            return;
        }
        self.save_project_with(conflict.target, true);
    }

    /// The conflict question's Save As…: the Save Project card, proposing a free name beside the
    /// changed project (both versions are kept).
    pub fn save_conflict_as(&mut self) {
        let Some(conflict) = self.save_conflict.take() else { return };
        self.open_tool(Tool::SaveProject);
        self.propose_beside(&conflict.target);
        self.say(Tone::Normal, "Choose where to save this version, then Save");
    }

    /// Fills the Save card with a free name beside `path` (`X (2)`, `X (2).scancraftz`, …) and
    /// remembers it as proposed ([`ProjectInfo::proposed`]).
    fn propose_beside(&mut self, path: &std::path::Path) {
        let free = crate::storage::free_sibling(path);
        self.settings.project_path = free.display().to_string();
        self.project.proposed = Some((path.to_path_buf(), free));
    }

    /// The conflict question's Cancel: nothing is saved (a close or quit waiting for the save
    /// stops too).
    pub fn cancel_save_conflict(&mut self) {
        if self.save_conflict.take().is_some() {
            self.pending_close = false;
            self.say(Tone::Normal, "Not saved");
        }
    }

    /// A save that did not start: as a failed one, it neither closes the tab it was for nor
    /// quits.
    fn save_refused(&mut self, message: String) {
        self.pending_close = false;
        if self.save_all_queue.take().is_some() {
            self.quit_confirmed = false;
        }
        self.say(Tone::Error, message);
    }

    /// Asks about a save that would replace someone else's version (stops a Save All on quit:
    /// the question is about one tab).
    fn ask_save_conflict(&mut self, target: PathBuf, why: String) {
        if self.save_all_queue.take().is_some() {
            self.quit_confirmed = false;
        }
        self.save_conflict = Some(crate::storage::SaveConflict { target, message: why });
    }

    /// [`Self::save_project`]; `overwrite`: save even over a version someone else saved (the
    /// conflict question's Save Anyway).
    fn save_project_with(&mut self, target: PathBuf, overwrite: bool) {
        if self.tool_blocked(Tool::SaveProject).is_some() || target.as_os_str().is_empty() {
            self.save_refused("Enter a folder or bundle file for the project".into());
            return;
        }
        let target = std::path::absolute(&target).unwrap_or(target);
        // A free name proposed earlier that is taken by now (another tab, app or computer saved
        // there meanwhile): what is there now is never replaced; the next free name is proposed.
        if let Some((beside, proposal)) = self.project.proposed.clone() {
            if same_path(&proposal, &target) && target.exists() {
                self.propose_beside(&beside);
                self.save_refused(format!("Not saved: {} was created meanwhile by another save. The Save card now proposes {}", target.display(), self.settings.project_path));
                return;
            }
        }
        let as_bundle = scancraft_io::project::has_extension(&target, scancraft_io::project::BUNDLE_EXTENSION);
        if !as_bundle && target.is_file() {
            self.save_refused(format!("{} is a file: enter a folder, or a .scancraftz file to save a bundle", target.display()));
            return;
        }
        let name = if as_bundle { target.file_stem() } else { target.file_name() }.map(|n| n.to_string_lossy().to_string()).unwrap_or_else(|| self.project.name.clone());
        let input = self.save_input();
        let contents = self.prefs.bundle_contents;
        let old_dir = self.project.store.data_dir();
        // Raw streams recorded before the project had a folder of its own (`ProjectInfo::staging`).
        let staging = self.project.staging.clone().filter(|d| d.is_dir());
        let n = input.passes.len();
        let passes = format!("{n} pass{}", if n == 1 { "" } else { "es" });
        if as_bundle {
            if let Store::Bundle { work, dir } = &self.project.store {
                // A bundle tab: into its working folder, then back into this or another bundle.
                let (work, dir) = (work.clone(), dir.clone());
                // Still this tab's to write: a lock another computer took over and has let go
                // since is taken again (as a project folder's); one someone else holds now
                // refuses the save, naming them.
                let relocked = match work.lock() {
                    Ok(mut w) if same_path(w.bundle(), &target) => w.relock().map_err(|e| e.to_string()),
                    Ok(_) => Ok(()),
                    Err(_) => Err("the working folder is unavailable".to_string()),
                };
                if let Err(e) = relocked {
                    self.save_refused(format!("Could not save the bundle: {e}. {SAVE_ELSEWHERE}"));
                    return;
                }
                if !overwrite {
                    let changed = work.lock().ok().filter(|w| same_path(w.bundle(), &target) && w.bundle().exists() && w.bundle_changed()).map(|w| w.bundle().to_path_buf());
                    if let Some(bundle) = changed {
                        self.ask_save_conflict(target, format!("{} was changed by someone else since it was opened here.", bundle.display()));
                        return;
                    }
                }
                self.spawn_job("Saving the bundle", move || {
                    let saved = (|| {
                        let flushed = input.wait_for_recordings(RECORDING_FLUSH);
                        project::save(&dir, &input.into_project(name.clone())?)?;
                        if !flushed {
                            log::warn!("a raw recording was still being written while saving");
                        }
                        let mut work = work.lock().map_err(|_| "the working folder is unavailable".to_string())?;
                        // Save keeps everything the bundle holds (the working folder, its only
                        // other copy, goes when the tab closes) and adds what the settings ask
                        // for; Save as Bundle… writes what the settings ask for.
                        let report = if same_path(work.bundle(), &target) {
                            let keep = work.held_contents().union(&contents);
                            if overwrite {
                                work.save_overwriting(&keep)
                            } else {
                                work.save(&keep)
                            }
                        } else {
                            work.save_as(&target, &contents)
                        };
                        report.map_err(|e| e.to_string())
                    })();
                    match saved {
                        Ok(report) => JobResult::Saved { message: bundle_message(&passes, &report), path: target, name, store: None },
                        Err(e) => JobResult::Failed(format!("Could not save the bundle: {e}")),
                    }
                });
            } else {
                // Save as Bundle… of a project folder or a new project: a new working folder.
                let root = self.prefs.work_root();
                self.spawn_job("Saving the bundle", move || {
                    let mut work = match WorkFolder::create(&target, &root) {
                        Ok(w) => w,
                        Err(e) => return JobResult::Failed(format!("Could not save the bundle: {e}")),
                    };
                    let saved = (|| {
                        input.wait_for_recordings(RECORDING_FLUSH);
                        project::save_carrying(work.dir(), &input.into_project(name.clone())?, old_dir.as_deref())?;
                        // From now on the folder holds the project (for a new one, its only copy
                        // on disk): a crash while the bundle is written leaves it recoverable.
                        work.project_written().map_err(|e| e.to_string())?;
                        // The raw recordings the app does not hold, only when the bundle takes them.
                        if let (Some(old), true) = (&old_dir, contents.includes(Item::RawRecordings)) {
                            workdir::copy_item_files(old, work.dir(), Item::RawRecordings).map_err(|e| e.to_string())?;
                        }
                        // Recorded before the project had a place: into the working folder (the
                        // tab's data from now on), whatever the bundle takes.
                        let note = staging.as_deref().and_then(|st| staging_note(st, move_raw_files(st, work.dir())));
                        work.save(&contents).map(|r| (r, note)).map_err(|e| e.to_string())
                    })();
                    match saved {
                        Ok((report, note)) => JobResult::Saved { message: format!("{}{}", bundle_message(&passes, &report), note.unwrap_or_default()), path: target, name, store: Some(Store::bundle(work)) },
                        Err(e) => {
                            let _ = work.close();
                            JobResult::Failed(format!("Could not save the bundle: {e}"))
                        }
                    }
                });
            }
            return;
        }
        // Never into the working folders (closing a bundle tab removes its working folder), also
        // those under an earlier root.
        if let Some(root) = self.prefs.work_roots().into_iter().find(|root| target.starts_with(root) || std::path::absolute(root).is_ok_and(|r| target.starts_with(r))) {
            self.save_refused(format!("{} is where opened bundles are unpacked: choose another folder", root.display()));
            return;
        }
        // A project folder: locked first unless it is this tab's folder already.
        let (lock, seen) = if self.project.store.is_folder(&target) {
            // This tab's folder: still locked by this tab (a lock broken by another computer that
            // has since let go is taken again; one held by someone else refuses the save), and
            // not saved by someone else meanwhile.
            let Store::Folder { lock, seen } = &mut self.project.store else { unreachable!("is_folder") };
            if !lock.is_held() {
                match LockFile::lock_project(&target) {
                    Ok(again) => *lock = again,
                    Err(e) => {
                        self.save_refused(format!("Could not save the project: {e}. {SAVE_ELSEWHERE}"));
                        return;
                    }
                }
            }
            let seen = seen.clone();
            let recorded = seen.lock().ok().and_then(|s| *s);
            if !overwrite {
                if let Err(scancraft_io::IoError::Conflict(why)) = scancraft_io::project::check_unchanged(&target, recorded) {
                    self.ask_save_conflict(target, why);
                    return;
                }
            }
            (None, Some(seen))
        } else {
            let locked = std::fs::create_dir_all(&target).map_err(scancraft_io::IoError::from).and_then(|_| LockFile::lock_project(&target));
            match locked {
                Ok(lock) => (Some(lock), None),
                Err(e) => {
                    self.save_refused(format!("Could not save the project: {e}"));
                    return;
                }
            }
        };
        self.spawn_job("Saving the project", move || {
            let saved = (|| {
                let flushed = input.wait_for_recordings(RECORDING_FLUSH);
                project::save_carrying(&target, &input.into_project(name.clone())?, old_dir.as_deref())?;
                // From a bundle (or another folder): bring the files the app does not hold.
                if let (Some(_), Some(old)) = (&lock, &old_dir) {
                    workdir::copy_item_files(old, &target, Item::RawRecordings).map_err(|e| e.to_string())?;
                }
                // Recorded before the project had a folder: moved in.
                let mut note = staging.as_deref().and_then(|st| staging_note(st, move_raw_files(st, &target)));
                if !flushed {
                    log::warn!("a raw recording was still being written while saving; save again once the capture has stopped");
                    note.get_or_insert_with(|| " · a raw recording was still being written: save again".into());
                }
                Ok::<_, String>(note)
            })();
            match saved {
                Ok(note) => {
                    // What the project file is now: the next save checks nobody else saved since.
                    let stamp = scancraft_io::project::project_file_stamp(&target);
                    if let Some(Ok(mut s)) = seen.as_ref().map(|s| s.lock()) {
                        *s = stamp;
                    }
                    JobResult::Saved { message: format!("Saved {passes} to {}{}", target.display(), note.unwrap_or_default()), path: target, name, store: lock.map(|l| Store::folder(l, stamp)) }
                }
                Err(e) => JobResult::Failed(format!("Could not save the project: {e}")),
            }
        });
    }

    /// The tab that has `path` open (its project folder or bundle).
    pub fn tab_with_path(&self, path: &std::path::Path) -> Option<usize> {
        self.tab_labels().iter().position(|t| t.path.as_deref().is_some_and(|p| same_path(p, path)))
    }

    /// Opens a project folder, project file or bundle in a new tab (or in the current one when it
    /// is empty). A project that is open in a tab already is switched to. The project is locked
    /// while open. One open in another app on this computer is refused; one open on another
    /// computer is asked about (`lock_prompt`: Take Over, Open Read-Only or Cancel). A lock left
    /// by a ScanCraft that has ended is taken over, and the status says so. A bundle is unpacked
    /// into a working folder (under the working-folder root) and the tab works there.
    pub fn open_project(&mut self, path: PathBuf) {
        if !self.can_edit() {
            return;
        }
        let Some(target) = OpenTarget::of(&path) else {
            let why = scancraft_io::project::locate(&path).err().map(|e| e.to_string()).unwrap_or_else(|| format!("{} is not a ScanCraft project", path.display()));
            self.say(Tone::Error, format!("Could not open the project: {why}"));
            return;
        };
        if let Some(i) = self.tab_with_path(target.path()) {
            // (Also when more opens wait in the queue.)
            if self.show_tab(i) {
                self.say(Tone::Normal, format!("{} is already open", self.project.name));
            }
            return;
        }
        match LockFile::lock_project(target.path()) {
            Ok(lock) => self.open_locked(target, lock),
            Err(scancraft_io::IoError::Locked { owner: Some(owner), .. }) if owner.may_be_taken_over() => {
                self.lock_prompt = Some(crate::storage::LockPrompt { target, owner: *owner });
            }
            Err(e) => self.say(Tone::Error, format!("Could not open the project: {e}")),
        }
    }

    /// Opens `target` with its lock taken.
    fn open_locked(&mut self, target: OpenTarget, lock: LockFile) {
        // Never silently: a lock taken from an ended ScanCraft (or, after the question, from
        // another computer) is named in the status.
        let notice = lock.replaced().map(|o| format!("its lock was taken over from {}", o.describe()));
        match target {
            OpenTarget::Bundle(bundle) => {
                let root = self.prefs.work_root();
                self.spawn_job("Opening the bundle", move || {
                    let work = match WorkFolder::open_with_lock(&bundle, &root, lock) {
                        Ok(w) => w,
                        Err(e) => return JobResult::Failed(format!("Could not open the bundle: {e}")),
                    };
                    match read_opened(work.dir()) {
                        Ok(mut opened) => {
                            opened.path = bundle;
                            opened.store = Store::bundle(work);
                            opened.notice = notice;
                            JobResult::Opened(Box::new(opened))
                        }
                        Err(e) => {
                            let _ = work.close();
                            JobResult::Failed(format!("Could not open the bundle: {e}"))
                        }
                    }
                });
            }
            OpenTarget::Folder(dir) => {
                self.spawn_job("Opening the project", move || {
                    // Taken before reading: a save by someone else after this is a conflict.
                    let seen = scancraft_io::project::project_file_stamp(&dir);
                    match read_opened(&dir) {
                        Ok(mut opened) => {
                            opened.store = Store::folder(lock, seen);
                            opened.notice = notice;
                            JobResult::Opened(Box::new(opened))
                        }
                        Err(e) => JobResult::Failed(format!("Could not open the project: {e}")),
                    }
                });
            }
        }
    }

    /// The lock question's message: who has the project, and what each answer does.
    pub fn lock_prompt_message(&self) -> String {
        let Some(p) = &self.lock_prompt else { return String::new() };
        format!(
            "{} is in use by {}.\nTake Over only if that ScanCraft is no longer running or has the project open by mistake: if both save, one loses its changes. Open Read-Only opens a copy here; Save then asks for a new place.",
            p.target.path().display(),
            p.owner.describe()
        )
    }

    /// The lock question's Take Over: takes the lock from the holder shown (refused when someone
    /// else holds it by now) and opens the project.
    pub fn take_over_and_open(&mut self) {
        let Some(p) = self.lock_prompt.take() else { return };
        if !self.can_edit() {
            return;
        }
        match LockFile::take_over_from(&LockFile::path_for(p.target.path()), &p.owner) {
            Ok(lock) => self.open_locked(p.target, lock),
            Err(e) => self.say(Tone::Error, format!("Could not take over the project: {e}")),
        }
    }

    /// The lock question's Open Read-Only: reads the project without its lock into a tab with
    /// no place on disk (a bundle through an unpacked copy under the working-folder root, kept
    /// until the tab closes or is saved elsewhere). Nothing is written where it is; Save asks for
    /// a new place and brings the raw recordings and the fields of later versions along.
    pub fn open_read_only(&mut self) {
        let Some(p) = self.lock_prompt.take() else { return };
        if !self.can_edit() {
            return;
        }
        match p.target {
            OpenTarget::Bundle(bundle) => {
                let root = self.prefs.work_root();
                self.spawn_job("Opening a read-only copy", move || {
                    let copy = match workdir::UnpackedCopy::unpack(&bundle, &root) {
                        Ok(c) => c,
                        Err(e) => return JobResult::Failed(format!("Could not open the bundle: {e}")),
                    };
                    // The copy stays while the tab is open: a save to a new place takes the raw
                    // recordings and the fields of later versions from it.
                    match read_opened(copy.dir()) {
                        Ok(mut opened) => {
                            opened.path = bundle.clone();
                            opened.store = Store::read_only_bundle(bundle, copy);
                            opened.read_only = true;
                            JobResult::Opened(Box::new(opened))
                        }
                        Err(e) => JobResult::Failed(format!("Could not open the bundle: {e}")),
                    }
                });
            }
            OpenTarget::Folder(dir) => {
                self.spawn_job("Opening a read-only copy", move || match read_opened(&dir) {
                    Ok(mut opened) => {
                        opened.store = Store::read_only_folder(dir);
                        opened.read_only = true;
                        JobResult::Opened(Box::new(opened))
                    }
                    Err(e) => JobResult::Failed(format!("Could not open the project: {e}")),
                });
            }
        }
    }

    /// Opens a working folder left behind by a crash as a tab with its unsaved changes (Save
    /// writes them into its bundle).
    pub fn recover_project(&mut self, dir: PathBuf) {
        if !self.can_edit() {
            return;
        }
        self.spawn_job("Recovering unsaved work", move || {
            let work = match WorkFolder::recover(&dir) {
                Ok(w) => w,
                Err(e) => return JobResult::Failed(format!("Could not recover {}: {e}", dir.display())),
            };
            match read_opened(work.dir()) {
                Ok(mut opened) => {
                    opened.path = work.bundle().to_path_buf();
                    opened.store = Store::bundle(work);
                    opened.recovered = true;
                    JobResult::Opened(Box::new(opened))
                }
                // The folder stays (listed again at the next start).
                Err(e) => JobResult::Failed(format!("Could not recover {}: {e}", dir.display())),
            }
        });
    }

    /// A request of a later start ([`crate::single_instance`]): open its project, or say why it
    /// was refused or which of its options were not used. With a project those options are
    /// named with the messages of its open ([`ScanCraftApp::carried_note`]), which would
    /// otherwise replace them at once.
    pub fn take_handover(&mut self, handover: crate::single_instance::Handover) {
        use crate::single_instance::{ignored_message, Handover};
        match handover {
            Handover::Open(p) => self.request_open(OpenRequest::Path(p)),
            Handover::Refused { path, why } => self.say(Tone::Error, format!("Could not open {path}: {why}")),
            Handover::Ignored { options, project: true } => self.carried_note = Some(ignored_message(&options, true)),
            Handover::Ignored { options, project: false } => self.say(Tone::Warning, ignored_message(&options, false)),
            Handover::Focus => {}
        }
    }

    /// The lock question's Cancel: nothing is opened.
    pub fn cancel_lock_prompt(&mut self) {
        if self.lock_prompt.take().is_some() {
            self.say(Tone::Normal, "Not opened");
        }
    }

    /// Opens (or recovers) a project now, or once the running operation has finished.
    pub fn request_open(&mut self, request: OpenRequest) {
        if self.can_edit() && self.tabs_held().is_none() && self.pending_open.is_empty() {
            match request {
                OpenRequest::Path(p) => self.open_project(p),
                OpenRequest::Recover(d) => self.recover_project(d),
            }
        } else {
            let name = match &request {
                OpenRequest::Path(p) | OpenRequest::Recover(p) => p.file_name().map(|n| n.to_string_lossy().to_string()).unwrap_or_default(),
            };
            self.pending_open.push_back(request);
            self.say(Tone::Normal, format!("{name} opens when the running operation has finished"));
        }
    }

    /// At start: the working folders that a crashed ScanCraft left behind, under the
    /// working-folder root and the earlier ones (an earlier root with nothing left in it is
    /// forgotten). Those with nothing to recover (no change since the last save, bundle
    /// unchanged) and stopped removals are removed at once; the others are put to the user in one
    /// question (`recovery`). Folders of a running ScanCraft and folders that are not working
    /// folders are left alone.
    pub fn check_leftovers(&mut self) {
        use scancraft_io::workdir::LeftoverStatus as S;
        let mut ask = Vec::new();
        let mut emptied = Vec::new();
        for root in self.prefs.work_roots() {
            for l in workdir::leftovers(&root) {
                match &l.status {
                    S::InUse(_) => {}
                    S::Orphaned { changed, bundle_changed: false } if changed.is_empty() => discard_quietly(&l.dir),
                    S::Closing => discard_quietly(&l.dir),
                    // Not a working folder (no state file): never touched.
                    S::Damaged(_) if !l.dir.join(workdir::STATE_FILE).is_file() => {}
                    _ => ask.push(l),
                }
            }
            if workdir::leftovers(&root).is_empty() {
                emptied.push(root);
            }
        }
        let before = self.prefs.past_work_roots.len();
        self.prefs.past_work_roots.retain(|p| !emptied.iter().any(|e| same_path(e, p)));
        if self.prefs.past_work_roots.len() != before {
            self.save_prefs();
        }
        self.recovery = (!ask.is_empty()).then_some(ask);
    }

    /// The recovery question's message.
    pub fn recovery_message(&self) -> String {
        use scancraft_io::workdir::LeftoverStatus as S;
        let Some(list) = &self.recovery else { return String::new() };
        let mut lines = vec!["ScanCraft did not close normally. These opened bundles have changes that were not saved:".to_string()];
        for l in list {
            let bundle = l.state.as_ref().and_then(|s| s.bundle.file_name()).map(|n| n.to_string_lossy().to_string()).unwrap_or_else(|| l.dir.display().to_string());
            let what = match &l.status {
                S::Orphaned { changed, bundle_changed } => {
                    let n = changed.len();
                    let mut w = format!("{n} changed file{}", if n == 1 { "" } else { "s" });
                    if *bundle_changed {
                        w.push_str("; the bundle was changed or moved since, so save it under a new name");
                    }
                    w
                }
                S::Incomplete => "unpacking stopped part-way: nothing to recover".to_string(),
                S::Closing => "its removal stopped part-way: nothing to recover".to_string(),
                S::Damaged(why) => format!("cannot be read ({why}): nothing to recover"),
                S::InUse(_) => "in use".to_string(),
            };
            lines.push(format!("• {bundle}: {what}"));
        }
        lines.push("Open them as tabs to save them, discard them, or decide at the next start.".into());
        lines.join("\n")
    }

    /// The recovery question's Open: the recoverable folders open as tabs, one after another.
    pub fn recover_leftovers(&mut self) {
        use scancraft_io::workdir::LeftoverStatus as S;
        for l in self.recovery.take().unwrap_or_default() {
            if matches!(l.status, S::Orphaned { .. }) {
                self.request_open(OpenRequest::Recover(l.dir));
            }
        }
    }

    /// The recovery question's Discard: removes every listed working folder.
    pub fn discard_leftovers(&mut self) {
        let mut failed = Vec::new();
        let list = self.recovery.take().unwrap_or_default();
        for l in &list {
            if let Err(e) = workdir::discard(&l.dir) {
                failed.push(format!("{} ({e})", l.dir.display()));
            }
        }
        if failed.is_empty() {
            self.say(Tone::Normal, format!("Discarded {} working folder{}", list.len(), if list.len() == 1 { "" } else { "s" }));
        } else {
            self.say(Tone::Error, format!("Could not remove {}", failed.join(", ")));
        }
    }

    pub(super) fn install_project(&mut self, mut opened: OpenedProject) {
        // Opens into a new tab unless the current one is empty: nothing is replaced. When no tab
        // can be added now, the open goes back to the front of the queue.
        if self.has_geometry() && !self.add_empty_tab() {
            let request = match std::mem::take(&mut opened.store) {
                // A recovered working folder is kept as it is (its lock let go meanwhile).
                Store::Bundle { dir, .. } if opened.recovered => OpenRequest::Recover(dir),
                // A freshly unpacked one holds nothing yet: removed, and unpacked again.
                store => {
                    store.close();
                    OpenRequest::Path(opened.path.clone())
                }
            };
            self.pending_open.push_front(request);
            let why = self.tabs_held().unwrap_or("the app is free");
            self.say(Tone::Warning, format!("{} opens when possible: {why}", opened.path.display()));
            return;
        }
        // An empty tab's own folder or bundle (nothing unsaved in it) is let go.
        std::mem::take(&mut self.project.store).close();
        self.clear_previews();
        self.picking = None;
        self.object_picking = None;
        self.ai_picking = None;
        self.highlight = None;
        self.passes.clear();
        // The open project's results are replaced, not invalidated, by the opened ones.
        self.merged = None;
        self.mesh = None;
        let n = opened.passes.len();
        for mut lp in opened.passes {
            let g = self.generation();
            let mut layer = CloudLayer::new(PointCloud::from_points(std::mem::take(&mut lp.points)), g);
            layer.visible = lp.stored.visible;
            self.passes.push(loaded_pass(lp, layer));
        }
        // A project without a reference (e.g. edited by hand) gets the first pass.
        if !self.passes.iter().any(|p| p.meta.alignment == AlignState::Reference) {
            match self.passes.iter().find(|p| p.meta.alignment.is_placed()).map(|p| p.id) {
                Some(id) => self.rebase_project(id),
                None => {
                    if let Some(id) = self.passes.first().map(|p| p.id) {
                        self.place_pass(id, Transform::default(), AlignState::Reference);
                    }
                }
            }
        }
        self.next_pass_id = opened.next_pass_id.max(self.passes.iter().map(|p| p.id + 1).max().unwrap_or(1));
        // A project saved before cleaning became per pass has one cleaned cloud of everything:
        // it was made from the merged model (or from all passes), so it becomes the merged model.
        let all: Vec<u32> = self.passes.iter().map(|p| p.id).collect();
        let merged = match (opened.merged, opened.cleaned) {
            (Some((_, passes)), Some(cleaned)) => Some((cleaned, passes, "Cleaned merged model, opened with the project".to_string())),
            (None, Some(cleaned)) => Some((cleaned, all, "Cleaned cloud from an older project, opened as the merged model".to_string())),
            (Some((cloud, passes)), None) => Some((cloud, passes, "Opened with the project".to_string())),
            (None, None) => None,
        };
        self.merged = merged.map(|(cloud, passes, summary)| {
            let g = self.generation();
            MergedLayer::new(cloud, g, passes, summary)
        });
        self.mesh = opened.mesh.map(|(m, a)| {
            let g = self.generation();
            MeshLayer::new(m, a, g)
        });
        // The newest result is shown; the inputs stay in the browser.
        if self.mesh.is_some() {
            if let Some(m) = &mut self.merged {
                m.layer.visible = false;
            }
        }
        if self.merged.is_some() || self.mesh.is_some() {
            for p in &mut self.passes {
                p.layer.visible = false;
            }
        }
        if let Some(v) = opened.settings.get("merge_sigma").and_then(|v| v.as_f64()) {
            self.settings.merge_sigma = v as f32;
        }
        if let Some(v) = opened.settings.get("merge_outliers").and_then(|v| v.as_bool()) {
            self.settings.merge_outliers = v;
        }
        self.settings.whole_scene = opened.settings.get("whole_scene").and_then(|v| v.as_bool()).unwrap_or(false);
        let history_before: Vec<serde_json::Value> = opened.settings.get("processing").and_then(|p| p.get("history")).and_then(|h| h.as_array()).cloned().unwrap_or_default();
        // A recovered project holds changes its bundle does not have yet.
        let saved_generation = if opened.recovered { 0 } else { self.next_generation };
        if opened.read_only {
            // A copy with no place on disk: Save proposes a new one beside the original.
            self.project = ProjectInfo { name: format!("{} (read-only)", opened.name), path: None, saved_generation, store: opened.store, proposed: None, notes: opened.notes.clone(), staging: None, history_before: history_before.clone() };
            self.propose_beside(&opened.path);
        } else {
            self.settings.project_path = opened.path.display().to_string();
            self.project = ProjectInfo { name: opened.name, path: Some(opened.path), saved_generation, store: opened.store, proposed: None, notes: opened.notes.clone(), staging: None, history_before: history_before.clone() };
        }
        self.selection = None;
        self.tool = None;
        self.framed_radius = 0.0;
        self.fit_view();
        let passes = format!("{n} pass{}", if n == 1 { "" } else { "es" });
        if opened.recovered {
            self.say(Tone::Success, format!("Recovered {} · {passes} · Save writes the changes into the bundle", self.project.name));
        } else if opened.read_only {
            self.say(Tone::Warning, format!("Opened {} · {passes} · nothing is written where it is: Save asks for a new place", self.project.name));
        } else if let Some(notice) = opened.notice {
            self.say(Tone::Warning, format!("Opened {} · {passes} · {notice}", self.project.name));
        } else {
            self.say(Tone::Success, format!("Opened {} · {passes}", self.project.name));
        }
    }

    // ------------------------------------------------------------------ imports

    /// Why a scan file or another project cannot be added now.
    pub fn import_blocked(&self) -> Option<&'static str> {
        if self.busy.is_some() {
            return Some("Another operation is running");
        }
        if self.capture != super::Capture::Idle {
            return Some("Stop the capture first");
        }
        None
    }

    /// Reads a PLY, OBJ or STL file as a new, unaligned pass (undoable).
    pub fn import_scan(&mut self, path: PathBuf) {
        if self.import_blocked().is_some() {
            return;
        }
        if path.as_os_str().is_empty() {
            self.say(Tone::Error, "Enter the scan file to import");
            return;
        }
        let file = path.file_name().map(|n| n.to_string_lossy().to_string()).unwrap_or_default();
        self.spawn_undoable(&format!("Importing {file}"), &format!("Import {file}"), move || match crate::imports::read_scan(&path) {
            Ok(scan) => JobResult::Imported(Box::new(scan)),
            Err(e) => JobResult::Failed(e),
        });
    }

    /// Appends the passes of the project in `dir` (a project folder, its project file or a
    /// bundle), unaligned and with fresh ids (undoable). A bundle is unpacked into a temporary
    /// folder under the working-folder root, read, and the folder removed again; it is not
    /// locked, so a bundle open in another tab or app can be added too.
    pub fn add_project(&mut self, dir: PathBuf) {
        if self.import_blocked().is_some() {
            return;
        }
        let label = project_label(&dir);
        if scancraft_io::project::is_bundle(&dir) {
            let root = self.prefs.work_root();
            self.spawn_undoable(&format!("Adding {label}"), &format!("Add Project {label}"), move || {
                let copy = match workdir::UnpackedCopy::unpack(&dir, &root) {
                    Ok(c) => c,
                    Err(e) => return JobResult::Failed(format!("Could not add the bundle: {e}")),
                };
                let read = crate::imports::read_project(copy.dir());
                drop(copy);
                match read {
                    Ok((name, passes)) if passes.is_empty() => JobResult::Failed(format!("{name} holds no passes")),
                    Ok((name, passes)) => JobResult::AddedProject { name, passes },
                    Err(e) => JobResult::Failed(e),
                }
            });
            return;
        }
        if let Err(e) = scancraft_io::project::locate(&dir) {
            self.say(Tone::Error, e.to_string());
            return;
        }
        self.spawn_undoable(&format!("Adding {label}"), &format!("Add Project {label}"), move || match crate::imports::read_project(&dir) {
            Ok((name, passes)) if passes.is_empty() => JobResult::Failed(format!("{name} holds no passes")),
            Ok((name, passes)) => JobResult::AddedProject { name, passes },
            Err(e) => JobResult::Failed(e),
        });
    }

    /// Adds new passes to the project. If no pass is in the project frame yet, the first of them
    /// becomes the reference (as the first captured pass does).
    fn append_passes(&mut self, passes: Vec<ScanPass>) -> usize {
        let n = passes.len();
        let first = passes.first().map(|p| p.id);
        let any_placed = self.passes.iter().any(|p| p.meta.alignment.is_placed());
        self.clear_previews();
        self.passes.extend(passes);
        if let (false, Some(id)) = (any_placed, first) {
            self.place_pass(id, Transform::default(), AlignState::Reference);
        }
        // New passes are shown with the passes, not hidden behind an older result.
        for p in &mut self.passes {
            p.layer.visible = true;
        }
        if let Some(m) = &mut self.merged {
            m.layer.visible = false;
        }
        if let Some(m) = &mut self.mesh {
            m.visible = false;
        }
        // New passes are usually cleaned first (table, fragments), then aligned.
        self.stage = Stage::Clean;
        self.framed_radius = 0.0;
        self.fit_view();
        n
    }

    pub(super) fn install_imported(&mut self, scan: crate::imports::ImportedScan) {
        let id = self.next_pass_id;
        self.next_pass_id += 1;
        let g = self.generation();
        let points = scan.surface.len();
        let taken: Vec<String> = self.passes.iter().map(|p| p.name.clone()).collect();
        let pass = ScanPass {
            id,
            name: crate::imports::unique_name(&scan.name, &taken),
            layer: CloudLayer::new(PointCloud::from_points(scan.surface.points.clone()), g),
            frames: 0,
            tracker: Arc::new(std::sync::Mutex::new(scancraft_slam::Tracker::new())),
            keyframes: Vec::new(),
            meta: scan.meta,
            tinted: None,
            mask: scan.mask,
            surface: Some(scan.surface),
            edit: None,
            rgb_keyframes: Vec::new(),
            markers: scan.markers,
        };
        let markers = pass.markers.len();
        let name = pass.name.clone();
        self.append_passes(vec![pass]);
        self.selection = Some(Node::Pass(id));
        let normals = match scan.normals {
            scancraft_io::NormalSource::File => "normals from the file",
            scancraft_io::NormalSource::Faces => "normals from its faces",
            scancraft_io::NormalSource::Estimated => "normals estimated",
            scancraft_io::NormalSource::None => "no normals",
        };
        let (tone, marker_note) = match &scan.marker_file {
            None => (Tone::Success, String::new()),
            Some((file, None)) => (Tone::Success, format!(", {markers} marker{} from {file}", if markers == 1 { "" } else { "s" })),
            Some((file, Some(e))) => (Tone::Warning, format!("; its {file} could not be read, so it has no markers ({e})")),
        };
        self.say(tone, format!("Imported {name} · {} points, {normals}{marker_note}. Next: clean it, then Align All", thousands(points)));
        let no_markers = match &scan.marker_file {
            Some((file, Some(_))) => Some(format!("{name} was imported without markers: its {file} could not be read")),
            _ => None,
        };
        if scan.likely_metres {
            let metres = format!("{name} is under {} units across, so probably in metres.", crate::imports::METRES_MAX_EXTENT);
            // Both in the one toast, and the marker warning outlasts the choice.
            match no_markers {
                Some(note) => {
                    self.show_choice_toast(format!("{metres} {note}."), ToastAction::MetresToMm(id));
                    self.toast_note = Some(note);
                }
                None => self.show_choice_toast(metres, ToastAction::MetresToMm(id)),
            }
        } else if let Some(note) = no_markers {
            self.show_toast(note);
        }
    }

    pub(super) fn install_added(&mut self, project: String, mut passes: Vec<LoadedPass>) {
        let taken: Vec<String> = self.passes.iter().map(|p| p.name.clone()).collect();
        self.next_pass_id = crate::imports::remap(&mut passes, self.next_pass_id, &taken, &project);
        let added: Vec<ScanPass> = passes
            .into_iter()
            .map(|mut lp| {
                let g = self.generation();
                let layer = CloudLayer::new(PointCloud::from_points(std::mem::take(&mut lp.points)), g);
                loaded_pass(lp, layer)
            })
            .collect();
        let n = self.append_passes(added);
        self.say(Tone::Success, format!("Added {n} pass{} from {project}, unaligned. Next: clean them, then Align All", if n == 1 { "" } else { "es" }));
    }

    /// Multiplies an imported pass's coordinates by `factor` (e.g. metres to mm; undoable). An
    /// aligned pass goes back to its own frame, as its alignment no longer fits.
    pub fn scale_pass(&mut self, id: u32, factor: f32) {
        if !self.can_edit() {
            return;
        }
        let Some(surface) = self.passes.iter().find(|p| p.id == id).and_then(|p| p.surface.clone()) else { return };
        let name = self.pass_name(id);
        self.record(self.snapshot(&format!("Scale {name}")));
        self.clear_previews();
        let surface = Arc::new(crate::imports::scaled(&surface, factor));
        let g = self.generation();
        let Some(p) = self.passes.iter_mut().find(|p| p.id == id) else { return };
        if matches!(p.meta.alignment, AlignState::Aligned { .. }) {
            p.meta.alignment = AlignState::Unaligned;
            p.meta.transform = Transform::default();
        }
        // Its cleaning and its markers are scaled with it.
        p.edit = p.edit.as_ref().map(|e| Arc::new(crate::imports::scaled(e, factor)));
        p.markers = p.markers.iter().map(|m| m.scaled(factor)).collect();
        let shown = p.edit.as_ref().unwrap_or(&surface).points.clone();
        let visible = p.layer.visible;
        p.layer = CloudLayer::new(PointCloud::from_points(project::place(shown, &p.meta.transform)), g);
        p.layer.visible = visible;
        p.surface = Some(surface);
        p.mask = isolation::new_cache();
        p.tinted = None;
        self.merged = None;
        self.invalidate_derived(&format!("{name} was scaled"));
        self.framed_radius = 0.0;
        self.fit_view();
        let note = if factor == 1000.0 { " (metres to mm)" } else { "" };
        self.say(Tone::Success, format!("Scaled {name} by {factor}{note}"));
    }
}
