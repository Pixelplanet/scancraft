//! Project-level actions of the app: alignment (auto, point pairs, global refinement) with a
//! preview before accepting, merging, the reference pass, same-position seeding, and saving /
//! opening the project folder. A child module of `app`, so it shares the app's private state.

use std::path::PathBuf;
use std::sync::Arc;

use glam::Vec3;
use scancraft_align::{Alignment, MergeConfig, MergeInput, PassCloud};
use scancraft_core::{PointCloud, Transform};
use scancraft_mesh::Mesher;
use scancraft_render::{Camera, CameraPreset};
use scancraft_slam::SeedState;

use super::{AlignPreview, PairPicking, ScanCraftApp, Tone};
use crate::model::*;
use crate::project::{self, AlignMethod, AlignState, LoadedPass, ProjectInfo, StoredPass, StoredProject};

/// Passes placed in the project frame, as (transform, tracker), optionally leaving one out.
fn placed(app: &ScanCraftApp, except: Option<u32>) -> Vec<(Transform, SharedTracker)> {
    app.passes.iter().filter(|p| p.meta.alignment.is_placed() && Some(p.id) != except).map(|p| (p.meta.transform, p.tracker.clone())).collect()
}

/// The union of passes in the project frame, for aligning against.
fn project_model(passes: &[(Transform, SharedTracker)]) -> Result<PassCloud, String> {
    let mut model = PassCloud::default();
    for (t, tracker) in passes {
        let tracker = tracker.lock().map_err(|_| "A pass's tracker is unavailable".to_string())?;
        let moved = scancraft_align::quality::moved(&project::pass_cloud(&tracker), t);
        model.points.extend(moved.points);
        model.views.extend(moved.views);
    }
    Ok(model)
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
    pub(super) fn seed_tracker(&self, tracker: &SharedTracker) -> bool {
        let sources = placed(self, None);
        let guards: Vec<_> = sources.iter().filter_map(|(t, tr)| tr.try_lock().ok().map(|g| (*t, g))).collect();
        if guards.len() != sources.len() {
            return false;
        }
        let refs: Vec<(Transform, &scancraft_slam::Tracker)> = guards.iter().map(|(t, g)| (*t, &**g)).collect();
        let (points, views) = project::seed_from(&refs);
        drop(refs);
        drop(guards);
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
        p.layer = CloudLayer::new(PointCloud::from_points(project::place(std::mem::take(&mut p.layer.cloud.points), &delta)), g);
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
    }

    pub fn set_reference(&mut self, id: u32) {
        if !self.can_edit() || !self.passes.iter().any(|p| p.id == id && p.meta.alignment.is_placed()) {
            return;
        }
        let name = self.pass_name(id);
        self.history.push(self.snapshot(&format!("make {name} the reference")));
        self.align_preview = None;
        self.rebase_project(id);
        self.say(Tone::Normal, format!("{name} is the reference; the project frame is now its frame"));
    }

    /// Back to its own frame (e.g. a wrong alignment).
    pub fn reset_alignment(&mut self, id: u32) {
        if !self.can_edit() || !self.passes.iter().any(|p| p.id == id && matches!(p.meta.alignment, AlignState::Aligned { .. })) {
            return;
        }
        let name = self.pass_name(id);
        self.history.push(self.snapshot(&format!("reset alignment of {name}")));
        self.place_pass(id, Transform::default(), AlignState::Unaligned);
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

    fn pass_name(&self, id: u32) -> String {
        self.passes.iter().find(|p| p.id == id).map(|p| p.name.clone()).unwrap_or_else(|| format!("Pass {id}"))
    }

    // ------------------------------------------------------------------ aligning

    /// Global registration of a pass against the project model, shown as a preview.
    pub fn align_auto(&mut self, id: u32) {
        if self.align_blocked(Some(id)).is_some() {
            return;
        }
        let Some(tracker) = self.passes.iter().find(|p| p.id == id).map(|p| p.tracker.clone()) else { return };
        let targets = placed(self, Some(id));
        let name = self.pass_name(id);
        self.align_preview = None;
        self.spawn_job(&format!("Aligning {name}"), move || {
            let t0 = std::time::Instant::now();
            let source = match tracker.lock() {
                Ok(t) => project::pass_cloud(&t),
                Err(_) => return JobResult::Failed("The pass's tracker is unavailable".into()),
            };
            let target = match project_model(&targets) {
                Ok(m) => m,
                Err(e) => return JobResult::Failed(e),
            };
            match scancraft_align::align_global(&source, &target, &scancraft_align::GlobalConfig::default()) {
                Ok(alignment) => {
                    let message = format!("Found a placement for {name} in {:.1} s · check it, then Accept", t0.elapsed().as_secs_f32());
                    JobResult::Aligned { pass: id, method: AlignMethod::Auto, alignment, message }
                }
                Err(e) => JobResult::Failed(format!("Could not align {name}: {e}. Try point pairs.")),
            }
        });
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
        let tracker = p.tracker.clone();
        let fine = p.meta.scan_config.accuracy.point_pitch_mm().max(1.0);
        let targets = placed(self, Some(id));
        let name = self.pass_name(id);
        self.spawn_job(&format!("Aligning {name} from {} pairs", pairs.len()), move || {
            let source = match tracker.lock() {
                Ok(t) => project::pass_cloud(&t),
                Err(_) => return JobResult::Failed("The pass's tracker is unavailable".into()),
            };
            let target = match project_model(&targets) {
                Ok(m) => m,
                Err(e) => return JobResult::Failed(e),
            };
            match scancraft_align::align_point_pairs(&source, &target, &pairs, fine) {
                Ok(alignment) => JobResult::Aligned { pass: id, method: AlignMethod::PointPairs, alignment, message: format!("Placed {name} from {} point pairs · check it, then Accept", pairs.len()) },
                Err(e) => JobResult::Failed(format!("Could not align {name}: {e}")),
            }
        });
    }

    pub(super) fn show_preview(&mut self, pass: u32, method: AlignMethod, alignment: Alignment, message: String) {
        let g = self.generation();
        let colour = self.pass_colour(pass);
        let Some(p) = self.passes.iter().find(|p| p.id == pass) else { return };
        let delta = scancraft_align::compose(&alignment.transform, &p.meta.transform.inverse());
        let cloud = PointCloud::from_points(project::place(p.layer.cloud.points.clone(), &delta));
        let mut layer = CloudLayer::new(cloud, g);
        layer.gpu = Arc::new(tinted_points(&layer.cloud, colour));
        self.align_preview = Some(AlignPreview { pass, method, alignment, layer });
        self.picking = None;
        if matches!(self.tool, Some(Tool::PointPairs(_))) {
            self.tool = None;
        }
        self.selection = Some(Node::Pass(pass));
        let tone = match alignment.quality.verdict() {
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
        self.history.push(self.snapshot(&format!("align {name}")));
        self.place_pass(preview.pass, preview.alignment.transform, AlignState::Aligned { method: preview.method, quality: Some(preview.alignment.quality) });
        self.merged = None;
        let next = self.passes.iter().find(|p| p.meta.alignment == AlignState::Unaligned).map(|p| p.name.clone());
        match next {
            Some(n) => self.say(Tone::Success, format!("Aligned {name}. Next: {n}")),
            None => self.say(Tone::Success, format!("Aligned {name}. All passes are aligned: Refine, then Merge")),
        }
    }

    pub fn discard_preview(&mut self) {
        if self.align_preview.take().is_some() {
            self.say(Tone::Normal, "Discarded the proposed alignment");
        }
    }

    /// Pairwise ICP between overlapping placed passes and pose-graph optimisation.
    pub fn refine_all(&mut self) {
        if self.refine_blocked().is_some() {
            return;
        }
        let inputs: Vec<(u32, Transform, bool, SharedTracker)> = self
            .passes
            .iter()
            .filter(|p| p.meta.alignment.is_placed())
            .map(|p| (p.id, p.meta.transform, p.meta.alignment == AlignState::Reference, p.tracker.clone()))
            .collect();
        self.align_preview = None;
        self.spawn_job("Refining all alignments", move || {
            let t0 = std::time::Instant::now();
            let mut clouds = Vec::new();
            for (_, _, _, tracker) in &inputs {
                match tracker.lock() {
                    Ok(t) => clouds.push(project::pass_cloud(&t)),
                    Err(_) => return JobResult::Failed("A pass's tracker is unavailable".into()),
                }
            }
            let refine_inputs: Vec<scancraft_align::RefineInput> = inputs.iter().zip(&clouds).map(|((_, pose, fixed, _), cloud)| scancraft_align::RefineInput { cloud, pose: *pose, fixed: *fixed }).collect();
            let result = scancraft_align::refine_global(&refine_inputs, &scancraft_align::RefineConfig::default());
            if result.edges.is_empty() {
                return JobResult::NoChange("No passes overlap enough to refine".into());
            }
            let message = format!(
                "Refined {} links in {:.1} s · mismatch {:.2} → {:.2} mm",
                result.edges.len(),
                t0.elapsed().as_secs_f32(),
                result.rms_before(),
                result.rms_after()
            );
            JobResult::Refined { poses: inputs.iter().map(|i| i.0).zip(result.poses).collect(), message }
        });
    }

    pub fn refine_blocked(&self) -> Option<&'static str> {
        if self.busy.is_some() {
            return Some("Another operation is running");
        }
        if self.capture != super::Capture::Idle {
            return Some("Stop the capture first");
        }
        (self.passes.iter().filter(|p| p.meta.alignment.is_placed()).count() < 2).then_some("Align at least two passes first")
    }

    pub(super) fn apply_refinement(&mut self, poses: Vec<(u32, Transform)>, message: String) {
        self.history.push(self.snapshot("refine alignments"));
        for (id, pose) in poses {
            let Some(state) = self.passes.iter().find(|p| p.id == id).map(|p| p.meta.alignment) else { continue };
            let state = match state {
                AlignState::Aligned { quality, .. } => AlignState::Aligned { method: AlignMethod::Refined, quality },
                other => other,
            };
            self.place_pass(id, pose, state);
        }
        self.merged = None;
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
        let inputs: Vec<(u32, Transform, f32, SharedTracker)> =
            self.passes.iter().filter(|p| p.meta.included && p.meta.alignment.is_placed()).map(|p| (p.id, p.meta.transform, p.meta.weight, p.tracker.clone())).collect();
        let pitch = self.passes.iter().filter(|p| p.meta.included && p.meta.alignment.is_placed()).map(|p| p.meta.scan_config.accuracy.point_pitch_mm()).fold(f32::MAX, f32::min);
        let skipped = self.passes.iter().filter(|p| p.meta.included && !p.meta.alignment.is_placed()).count();
        let config = MergeConfig { pitch_mm: pitch, outliers: self.settings.merge_outliers.then_some((12, self.settings.merge_sigma)), ..MergeConfig::default() };
        self.history.push(self.snapshot("merge"));
        self.align_preview = None;
        self.spawn_job("Merging passes", move || {
            let t0 = std::time::Instant::now();
            let mut merge_inputs = Vec::new();
            for (_, t, weight, tracker) in &inputs {
                let Ok(tracker) = tracker.lock() else { return JobResult::Failed("A pass's tracker is unavailable".into()) };
                let m = t.to_mat4();
                let points = tracker.fused_surfels().iter().map(|s| (m.transform_point3(s.position), (t.rotation * s.normal).normalize_or_zero(), s.count)).collect();
                merge_inputs.push(MergeInput { points, weight: *weight });
            }
            let r = scancraft_align::merge(&merge_inputs, &config);
            let mut message = format!(
                "Merged {} passes · {} points ({} outweighed by more accurate passes, {} isolated, {} outliers removed) in {:.1} s",
                inputs.len(),
                thousands(r.points.len()),
                thousands(r.outweighed),
                thousands(r.isolated),
                thousands(r.outliers),
                t0.elapsed().as_secs_f32()
            );
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
        self.history.push(self.snapshot("Discard merged model"));
        self.merged = None;
        for p in &mut self.passes {
            p.layer.visible = true;
        }
        if self.selection == Some(Node::Merged) {
            self.selection = None;
        }
        self.say(Tone::Normal, "Discarded the merged model; the passes are unchanged");
    }

    // ------------------------------------------------------------------ project files

    pub fn new_project(&mut self) {
        if !self.can_edit() {
            return;
        }
        if self.has_geometry() {
            self.history.push(self.snapshot("New project"));
        }
        self.passes.clear();
        self.merged = None;
        self.cleaned = None;
        self.mesh = None;
        self.align_preview = None;
        self.picking = None;
        self.selection = None;
        self.highlight = None;
        self.next_pass_id = 1;
        self.framed_radius = 0.0;
        self.project = ProjectInfo { saved_generation: self.next_generation, ..ProjectInfo::default() };
        self.settings.project_path = super::default_project_path();
        self.say(Tone::Normal, "New project");
    }

    pub fn save_project(&mut self, dir: PathBuf) {
        if self.tool_blocked(Tool::SaveProject).is_some() || dir.as_os_str().is_empty() {
            self.say(Tone::Error, "Enter a folder for the project");
            return;
        }
        let name = dir.file_name().map(|n| n.to_string_lossy().to_string()).unwrap_or_else(|| self.project.name.clone());
        let passes: Vec<(u32, String, u64, bool, crate::project::PassMeta, SharedTracker)> =
            self.passes.iter().map(|p| (p.id, p.name.clone(), p.frames, p.layer.visible, p.meta.clone(), p.tracker.clone())).collect();
        let reference = self.reference_pass();
        let next_pass_id = self.next_pass_id;
        let merged = self.merged.as_ref().map(|m| (m.layer.cloud.clone(), m.passes.clone()));
        let cleaned = self.cleaned.as_ref().map(|c| c.cloud.clone());
        let mesh = self.mesh.as_ref().map(|m| m.mesh.clone());
        let settings = serde_json::json!({
            "min_mm": self.settings.min_mm,
            "max_mm": self.settings.max_mm,
            "merge_outliers": self.settings.merge_outliers,
            "merge_sigma": self.settings.merge_sigma,
        });
        self.spawn_job("Saving the project", move || {
            let mut stored = Vec::new();
            for (id, name, frames, visible, meta, tracker) in passes {
                let Ok(t) = tracker.lock() else { return JobResult::Failed("A pass's tracker is unavailable".into()) };
                stored.push(StoredPass { id, name, frames, visible, meta, keyframes: t.keyframes().iter().map(scancraft_slam::Keyframe::to_data).collect() });
            }
            let n = stored.len();
            let project = StoredProject { name: name.clone(), reference, next_pass_id, passes: stored, merged, cleaned, mesh, settings };
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
        if self.has_geometry() {
            self.history.push(self.snapshot("Open project"));
        }
        self.align_preview = None;
        self.picking = None;
        self.highlight = None;
        self.passes.clear();
        let n = opened.passes.len();
        for lp in opened.passes {
            let g = self.generation();
            let mut layer = CloudLayer::new(PointCloud::from_points(lp.points), g);
            layer.visible = lp.stored.visible;
            self.passes.push(ScanPass {
                id: lp.stored.id,
                name: lp.stored.name,
                layer,
                frames: lp.stored.frames,
                tracker: Arc::new(std::sync::Mutex::new(lp.tracker)),
                keyframes: lp.keyframes,
                meta: lp.stored.meta,
                tinted: None,
            });
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
        self.merged = opened.merged.map(|(cloud, passes)| {
            let g = self.generation();
            MergedLayer::new(cloud, g, passes, "Opened with the project".into())
        });
        self.cleaned = opened.cleaned.map(|c| {
            let g = self.generation();
            CloudLayer::new(c, g)
        });
        self.mesh = opened.mesh.map(|(m, a)| {
            let g = self.generation();
            MeshLayer::new(m, a, g)
        });
        // The newest result is shown; the inputs stay in the browser.
        if self.mesh.is_some() {
            if let Some(c) = &mut self.cleaned {
                c.visible = false;
            }
            if let Some(m) = &mut self.merged {
                m.layer.visible = false;
            }
        }
        if self.merged.is_some() || self.cleaned.is_some() || self.mesh.is_some() {
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
        self.settings.project_path = opened.path.display().to_string();
        self.project = ProjectInfo { name: opened.name, path: Some(opened.path), saved_generation: self.next_generation };
        self.selection = None;
        self.tool = None;
        self.framed_radius = 0.0;
        self.fit_view();
        self.say(Tone::Success, format!("Opened {} · {n} pass{}", self.project.name, if n == 1 { "" } else { "es" }));
    }
}
