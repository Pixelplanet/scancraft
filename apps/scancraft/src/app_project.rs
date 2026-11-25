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
use crate::project::{self, AlignMethod, AlignState, LoadedPass, PassSource, ProjectInfo, StoredPass, StoredProject};

/// What a background job needs of one pass: (id, project <- pass, placed, reference, surface,
/// isolation cache).
type JobPass = (u32, Transform, bool, bool, PassSource, MaskCache);

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
        mask: isolation::new_cache(),
        surface: lp.stored.surface,
        edit: lp.stored.edit,
        rgb_keyframes: lp.stored.rgb_keyframes,
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
    fn place_pass(&mut self, id: u32, transform: Transform, alignment: AlignState) {
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

    /// Makes `id` (a placed pass) the frame of the project: every placed pass is re-expressed
    /// relative to it.
    pub(super) fn rebase_project(&mut self, id: u32) {
        let Some(base) = self.passes.iter().find(|p| p.id == id).map(|p| p.meta.transform) else { return };
        let inv = base.inverse();
        let ids: Vec<(u32, Transform, AlignState)> = self.passes.iter().filter(|p| p.meta.alignment.is_placed()).map(|p| (p.id, p.meta.transform, p.meta.alignment)).collect();
        for (pid, t, state) in ids {
            let state = if pid == id {
                AlignState::Reference
            } else if state == AlignState::Reference {
                AlignState::Aligned { method: AlignMethod::Auto, quality: None }
            } else {
                state
            };
            self.place_pass(pid, scancraft_align::compose(&inv, &t), state);
        }
        self.merged = None;
        self.invalidate_derived("The project frame changed");
    }

    pub fn set_reference(&mut self, id: u32) {
        if !self.can_edit() || !self.passes.iter().any(|p| p.id == id && p.meta.alignment.is_placed()) {
            return;
        }
        let name = self.pass_name(id);
        self.record(self.snapshot(&format!("make {name} the reference")));
        self.clear_previews();
        self.rebase_project(id);
        self.say(Tone::Normal, format!("{name} is the reference; the project frame is now its frame"));
    }

    /// Back to its own frame (e.g. a wrong alignment).
    pub fn reset_alignment(&mut self, id: u32) {
        if !self.can_edit() || !self.passes.iter().any(|p| p.id == id && matches!(p.meta.alignment, AlignState::Aligned { .. })) {
            return;
        }
        let name = self.pass_name(id);
        self.record(self.snapshot(&format!("reset alignment of {name}")));
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
                "align all passes",
                o.moved().filter_map(|p| p.pose.map(|t| (p.id, t, AlignState::Aligned { method: AlignMethod::AllPasses, quality: p.quality }))).collect(),
            ),
            Proposal::Refine(r) => (
                "refine alignments",
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
        self.record(self.snapshot(&format!("align {name}")));
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
        self.spawn_undoable("Merging passes", "merge", move || {
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
        self.record(self.snapshot("Delete merged model"));
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

    pub fn save_project(&mut self, dir: PathBuf) {
        if self.tool_blocked(Tool::SaveProject).is_some() || dir.as_os_str().is_empty() {
            self.say(Tone::Error, "Enter a folder for the project");
            return;
        }
        let name = dir.file_name().map(|n| n.to_string_lossy().to_string()).unwrap_or_else(|| self.project.name.clone());
        let passes: Vec<_> = self
            .passes
            .iter()
            .map(|p| (p.id, p.name.clone(), p.frames, p.layer.visible, p.meta.clone(), p.tracker.clone(), p.surface.clone(), p.edit.clone(), p.rgb_keyframes.clone()))
            .collect();
        let reference = self.reference_pass();
        let next_pass_id = self.next_pass_id;
        let merged = self.merged.as_ref().map(|m| ((*m.layer.cloud).clone(), m.passes.clone()));
        let mesh = self.mesh.as_ref().map(|m| (*m.mesh).clone());
        let settings = serde_json::json!({
            "min_mm": self.settings.min_mm,
            "max_mm": self.settings.max_mm,
            "merge_outliers": self.settings.merge_outliers,
            "merge_sigma": self.settings.merge_sigma,
            "whole_scene": self.settings.whole_scene,
        });
        self.spawn_job("Saving the project", move || {
            let mut stored = Vec::new();
            for (id, name, frames, visible, meta, tracker, surface, edit, rgb_keyframes) in passes {
                let Ok(t) = tracker.lock() else { return JobResult::Failed("A pass's tracker is unavailable".into()) };
                stored.push(StoredPass { id, name, frames, visible, meta, keyframes: t.keyframes().iter().map(scancraft_slam::Keyframe::to_data).collect(), surface, edit, rgb_keyframes });
            }
            let n = stored.len();
            let project = StoredProject { name: name.clone(), reference, next_pass_id, passes: stored, merged, cleaned: None, mesh, settings };
            match project::save(&dir, &project) {
                Ok(()) => JobResult::Saved { message: format!("Saved {n} pass{} to {}", if n == 1 { "" } else { "es" }, dir.display()), path: dir, name },
                Err(e) => JobResult::Failed(format!("Could not save the project: {e}")),
            }
        });
    }

    pub fn open_project(&mut self, dir: PathBuf) {
        if !self.can_edit() {
            return;
        }
        self.spawn_job("Opening the project", move || {
            let stored = match project::load(&dir) {
                Ok(s) => s,
                Err(e) => return JobResult::Failed(format!("Could not open the project: {e}")),
            };
            let passes: Vec<LoadedPass> = {
                use rayon::prelude::*;
                stored.passes.into_par_iter().map(project::rebuild).collect()
            };
            let mesh = stored.mesh.map(|m| {
                let a = Mesher::analyze(&m);
                (m, a)
            });
            JobResult::Opened(Box::new(OpenedProject {
                path: dir,
                name: stored.name,
                next_pass_id: stored.next_pass_id,
                passes,
                merged: stored.merged,
                cleaned: stored.cleaned,
                mesh,
                settings: stored.settings,
            }))
        });
    }

    pub(super) fn install_project(&mut self, opened: OpenedProject) {
        // Opens into a new tab unless the current one is empty: nothing is replaced.
        if self.has_geometry() {
            self.new_tab();
        }
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
        self.settings.project_path = opened.path.display().to_string();
        self.project = ProjectInfo { name: opened.name, path: Some(opened.path), saved_generation: self.next_generation };
        self.selection = None;
        self.tool = None;
        self.framed_radius = 0.0;
        self.fit_view();
        self.say(Tone::Success, format!("Opened {} · {n} pass{}", self.project.name, if n == 1 { "" } else { "es" }));
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
        self.spawn_undoable(&format!("Importing {file}"), &format!("import {file}"), move || match crate::imports::read_scan(&path) {
            Ok(scan) => JobResult::Imported(Box::new(scan)),
            Err(e) => JobResult::Failed(e),
        });
    }

    /// Appends the passes of the project in `dir`, unaligned and with fresh ids (undoable).
    pub fn add_project(&mut self, dir: PathBuf) {
        if self.import_blocked().is_some() {
            return;
        }
        if !scancraft_io::project::is_project(&dir) {
            self.say(Tone::Error, format!("{} holds no {}", dir.display(), scancraft_io::project::PROJECT_FILE));
            return;
        }
        let label = dir.file_name().map(|n| n.to_string_lossy().to_string()).unwrap_or_else(|| "project".into());
        self.spawn_undoable(&format!("Adding {label}"), &format!("add project {label}"), move || match crate::imports::read_project(&dir) {
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
        };
        let name = pass.name.clone();
        self.append_passes(vec![pass]);
        self.selection = Some(Node::Pass(id));
        let normals = match scan.normals {
            scancraft_io::NormalSource::File => "normals from the file",
            scancraft_io::NormalSource::Faces => "normals from its faces",
            scancraft_io::NormalSource::Estimated => "normals estimated",
            scancraft_io::NormalSource::None => "no normals",
        };
        self.say(Tone::Success, format!("Imported {name} · {} points, {normals}. Next: clean it, then Align All", thousands(points)));
        if scan.likely_metres {
            self.show_choice_toast(format!("{name} is under {} units across, so probably in metres.", crate::imports::METRES_MAX_EXTENT), ToastAction::MetresToMm(id));
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
        self.record(self.snapshot(&format!("scale {name}")));
        self.clear_previews();
        let surface = Arc::new(crate::imports::scaled(&surface, factor));
        let g = self.generation();
        let Some(p) = self.passes.iter_mut().find(|p| p.id == id) else { return };
        if matches!(p.meta.alignment, AlignState::Aligned { .. }) {
            p.meta.alignment = AlignState::Unaligned;
            p.meta.transform = Transform::default();
        }
        // Its cleaning is scaled with it.
        p.edit = p.edit.as_ref().map(|e| Arc::new(crate::imports::scaled(e, factor)));
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
