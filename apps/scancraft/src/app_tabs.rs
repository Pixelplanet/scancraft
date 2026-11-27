//! Project tabs: several projects open at once, one of them active.
//!
//! The active project lives in the app's own fields (passes, merged model, mesh, history, …), so
//! the rest of the app reads and writes it as before. Every other open project is parked in a
//! [`ProjectTab`]. Switching tabs swaps the two, which is only allowed while nothing runs (no job,
//! no capture, no open file dialog): a job's result always belongs to the tab that started it.
//!
//! Nothing is ever thrown away without asking: closing a tab with unsaved changes, and quitting
//! with any, asks Save / Discard / Cancel first. New Project opens a new tab; Open Project opens
//! into a new tab unless the current one is empty.

use scancraft_render::{Camera, CameraPreset};

use super::{AlignPreview, DatumPicking, Highlight, OrientPreview, PairPicking, PosesPreview, ScalePicking, ScanCraftApp, Tone};
use crate::ai_segment::AiPicking;
use crate::isolation::ObjectPicking;
use crate::model::*;
use crate::project::{PassStart, ProjectInfo};

/// A project that is open but not active: everything that belongs to one project.
pub struct ProjectTab {
    passes: Vec<ScanPass>,
    next_pass_id: u32,
    merged: Option<MergedLayer>,
    mesh: Option<MeshLayer>,
    holes: Option<HolePreview>,
    history: History,
    pub project: ProjectInfo,
    next_start: PassStart,
    align_preview: Option<AlignPreview>,
    poses_preview: Option<PosesPreview>,
    orient_preview: Option<OrientPreview>,
    picking: Option<PairPicking>,
    datum_picking: Option<DatumPicking>,
    scale_picking: Option<ScalePicking>,
    object_picking: Option<ObjectPicking>,
    ai_picking: Option<AiPicking>,
    highlight: Option<Highlight>,
    selection: Option<Node>,
    stage: Stage,
    tool: Option<Tool>,
    camera: Camera,
    framed_radius: f32,
    project_path: String,
    /// Unsaved changes when it was parked (the active tab's are computed live).
    pub dirty: bool,
}

impl ProjectTab {
    /// An empty project called `name`.
    pub fn empty(name: String) -> Self {
        let mut camera = Camera::default();
        camera.set_view_preset(CameraPreset::Isometric);
        camera.target = glam::Vec3::new(0.0, 250.0, 0.0);
        camera.set_view_preset(CameraPreset::Isometric);
        ProjectTab {
            passes: Vec::new(),
            next_pass_id: 1,
            merged: None,
            mesh: None,
            holes: None,
            history: History::default(),
            project: ProjectInfo { name, ..ProjectInfo::default() },
            next_start: PassStart::Moved,
            align_preview: None,
            poses_preview: None,
            orient_preview: None,
            picking: None,
            datum_picking: None,
            scale_picking: None,
            object_picking: None,
            ai_picking: None,
            highlight: None,
            selection: None,
            stage: Stage::Connect,
            tool: None,
            camera,
            framed_radius: 0.0,
            project_path: String::new(),
            dirty: false,
        }
    }
}

/// What a confirmation dialog is asking about.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ClosePrompt {
    /// Close the active tab, which has unsaved changes.
    Tab,
    /// Quit while projects have unsaved changes.
    Quit,
}

/// One tab as the tab strip shows it.
pub struct TabLabel {
    pub name: String,
    pub dirty: bool,
    /// Its project folder or bundle file.
    pub path: Option<std::path::PathBuf>,
    /// A bundle's working folder (`None`: not a bundle).
    pub work_dir: Option<std::path::PathBuf>,
    /// What a read-only copy was opened from.
    pub copy_of: Option<std::path::PathBuf>,
    pub active: bool,
}

impl ScanCraftApp {
    /// Swaps the active project's state with `tab`'s.
    fn swap_project(&mut self, tab: &mut ProjectTab) {
        use std::mem::swap;
        swap(&mut self.passes, &mut tab.passes);
        swap(&mut self.next_pass_id, &mut tab.next_pass_id);
        swap(&mut self.merged, &mut tab.merged);
        swap(&mut self.mesh, &mut tab.mesh);
        swap(&mut self.holes, &mut tab.holes);
        swap(&mut self.history, &mut tab.history);
        swap(&mut self.project, &mut tab.project);
        swap(&mut self.next_start, &mut tab.next_start);
        swap(&mut self.align_preview, &mut tab.align_preview);
        swap(&mut self.poses_preview, &mut tab.poses_preview);
        swap(&mut self.orient_preview, &mut tab.orient_preview);
        swap(&mut self.picking, &mut tab.picking);
        swap(&mut self.datum_picking, &mut tab.datum_picking);
        swap(&mut self.scale_picking, &mut tab.scale_picking);
        swap(&mut self.object_picking, &mut tab.object_picking);
        swap(&mut self.ai_picking, &mut tab.ai_picking);
        swap(&mut self.highlight, &mut tab.highlight);
        swap(&mut self.selection, &mut tab.selection);
        swap(&mut self.stage, &mut tab.stage);
        swap(&mut self.tool, &mut tab.tool);
        swap(&mut self.camera, &mut tab.camera);
        swap(&mut self.framed_radius, &mut tab.framed_radius);
        swap(&mut self.settings.project_path, &mut tab.project_path);
    }

    /// Parks the active project in its slot (`tabs[active_tab]`, which holds an empty
    /// placeholder while the project is active), remembering whether it had unsaved changes.
    fn park_active(&mut self) {
        let dirty = self.dirty();
        let i = self.active_tab;
        let mut slot = std::mem::replace(&mut self.tabs[i], ProjectTab::empty(String::new()));
        self.swap_project(&mut slot);
        slot.dirty = dirty;
        self.tabs[i] = slot;
    }

    /// Makes `tabs[i]` the active project (its slot keeps a placeholder).
    fn activate(&mut self, i: usize) {
        let mut slot = std::mem::replace(&mut self.tabs[i], ProjectTab::empty(String::new()));
        let dirty = slot.dirty;
        self.swap_project(&mut slot);
        self.tabs[i] = slot;
        self.active_tab = i;
        // The parked project's change state carries over: `dirty()` compares generations.
        self.project.saved_generation = if dirty { 0 } else { self.next_generation };
        if self.settings.project_path.is_empty() {
            self.settings.project_path = super::default_project_path_for(&self.project.name);
        }
    }

    /// Why tabs cannot be switched, opened or closed now (by the user).
    pub fn tabs_blocked(&self) -> Option<&'static str> {
        self.tabs_held().or_else(|| (!self.pending_imports.is_empty() || !self.pending_open.is_empty()).then_some("Wait for the imports to finish"))
    }

    /// Why the app itself cannot switch or add a tab now. Queued opens and imports do not hold
    /// it: an open that finishes while others wait, and Save All, move between tabs themselves.
    pub fn tabs_held(&self) -> Option<&'static str> {
        if self.busy.is_some() {
            Some("Another operation is running")
        } else if self.capture != super::Capture::Idle {
            Some("Stop the capture first")
        } else if self.picker.is_open() {
            Some("Close the file dialog first")
        } else if self.lock_prompt.is_some() || self.save_conflict.is_some() {
            Some("Answer the question first")
        } else {
            None
        }
    }

    /// The tab strip's entries, in order.
    pub fn tab_labels(&self) -> Vec<TabLabel> {
        self.tabs
            .iter()
            .enumerate()
            .map(|(i, t)| {
                if i == self.active_tab {
                    let copy_of = self.project.store.read_only_source().map(std::path::Path::to_path_buf);
                    TabLabel { name: self.project.name.clone(), dirty: self.dirty(), path: self.project.path.clone(), work_dir: self.project.store.work_dir(), copy_of, active: true }
                } else {
                    let copy_of = t.project.store.read_only_source().map(std::path::Path::to_path_buf);
                    TabLabel { name: t.project.name.clone(), dirty: t.dirty, path: t.project.path.clone(), work_dir: t.project.store.work_dir(), copy_of, active: false }
                }
            })
            .collect()
    }

    /// Switches to tab `i`.
    pub fn switch_tab(&mut self, i: usize) {
        if self.tabs_blocked().is_none() {
            self.show_tab(i);
        }
    }

    /// Makes tab `i` the active one unless the app is held ([`Self::tabs_held`]); whether it is
    /// active now.
    pub(super) fn show_tab(&mut self, i: usize) -> bool {
        if i == self.active_tab {
            return true;
        }
        if i >= self.tabs.len() || self.tabs_held().is_some() {
            return false;
        }
        self.park_active();
        self.activate(i);
        self.close_prompt = None;
        self.pending_close = false;
        self.toast = None;
        self.say(Tone::Normal, format!("Switched to {}", self.project.name));
        true
    }

    /// A name for a new project that no open tab uses: "Untitled project", "Untitled project 2", …
    fn untitled_name(&self) -> String {
        let taken: Vec<String> = self.tab_labels().into_iter().map(|t| t.name).collect();
        let base = ProjectInfo::default().name;
        (1..).map(|n| if n == 1 { base.clone() } else { format!("{base} {n}") }).find(|c| !taken.contains(c)).unwrap_or(base)
    }

    /// Opens a new, empty project in a new tab and switches to it (New Project). The open
    /// project stays as it is in its own tab.
    pub fn new_tab(&mut self) {
        if self.tabs_blocked().is_none() {
            self.add_empty_tab();
        }
    }

    /// Adds an empty project in a new tab and makes it active unless the app is held
    /// ([`Self::tabs_held`]); whether it did.
    pub(super) fn add_empty_tab(&mut self) -> bool {
        if self.tabs_held().is_some() {
            return false;
        }
        let name = self.untitled_name();
        self.park_active();
        self.tabs.push(ProjectTab::empty(name));
        let i = self.tabs.len() - 1;
        self.activate(i);
        self.settings.project_path = super::default_project_path_for(&self.project.name);
        self.close_prompt = None;
        self.pending_close = false;
        self.say(Tone::Normal, format!("New project: {}", self.project.name));
        true
    }

    /// Closes tab `i`: at once if it has no unsaved changes, otherwise after asking (it becomes
    /// the active tab so the question is about what is on screen).
    pub fn request_close_tab(&mut self, i: usize) {
        if i >= self.tabs.len() || self.tabs_blocked().is_some() {
            return;
        }
        if i != self.active_tab {
            self.switch_tab(i);
        }
        if self.dirty() {
            self.close_prompt = Some(ClosePrompt::Tab);
        } else {
            self.close_active_tab();
        }
    }

    /// Closes the active tab without asking; the neighbour to its left (or right) becomes active.
    /// Closing the last tab leaves one empty project. The project's lock is released and a
    /// bundle's working folder removed (unsaved changes in it are lost: the caller asked first).
    pub fn close_active_tab(&mut self) {
        self.close_prompt = None;
        self.pending_close = false;
        let closing = self.project.name.clone();
        std::mem::take(&mut self.project.store).close();
        if self.tabs.len() == 1 {
            let name = ProjectInfo::default().name;
            // Park the closing project in its slot, then replace it there.
            self.park_active();
            self.tabs[0] = ProjectTab::empty(name);
            self.activate(0);
            self.say(Tone::Normal, format!("Closed {closing}"));
            return;
        }
        // Park the closing project in its slot and drop it.
        self.park_active();
        let i = self.active_tab;
        self.tabs.remove(i);
        let next = i.saturating_sub(1).min(self.tabs.len() - 1);
        self.activate(next);
        self.say(Tone::Normal, format!("Closed {closing}"));
    }

    /// The close dialog's Save: saves the active project and closes the tab once saving has
    /// worked. A project without a folder opens Save Project first.
    pub fn save_then_close(&mut self) {
        self.close_prompt = None;
        self.pending_close = true;
        match self.project.path.clone() {
            Some(path) => self.save_project(path),
            None => self.open_tool(Tool::SaveProject),
        }
    }

    /// Copies the passes of open tab `i` into the active project, unaligned and with fresh ids,
    /// as Add Project does with a project folder (undoable). The other tab is not changed: its
    /// passes are rebuilt from their keyframes, so the two projects never share a tracker.
    pub fn add_tab_passes(&mut self, i: usize) {
        if i == self.active_tab || i >= self.tabs.len() || self.import_blocked().is_some() {
            return;
        }
        let tab = &self.tabs[i];
        let name = tab.project.name.clone();
        if tab.passes.is_empty() {
            self.say(Tone::Warning, format!("{name} has no passes to add"));
            return;
        }
        let mut stored = Vec::with_capacity(tab.passes.len());
        for p in &tab.passes {
            let Ok(t) = p.tracker.lock() else {
                self.say(Tone::Error, "A pass's tracker is unavailable");
                return;
            };
            stored.push(crate::project::StoredPass {
                id: p.id,
                name: p.name.clone(),
                frames: p.frames,
                visible: true,
                meta: p.meta.clone(),
                keyframes: t.keyframes().iter().map(scancraft_slam::Keyframe::to_data).collect(),
                surface: p.surface.clone(),
                edit: p.edit.clone(),
                rgb_keyframes: p.rgb_keyframes.clone(),
                mask: crate::isolation::cached(&p.mask),
                markers: p.markers.clone(),
            });
        }
        let label = name.clone();
        self.spawn_undoable(&format!("Adding the passes of {label}"), &format!("Add Passes of {label}"), move || {
            let passes = crate::imports::prepare_added(&name, stored);
            JobResult::AddedProject { name, passes }
        });
    }

    /// Projects with unsaved changes that have never been saved (no folder): Save All cannot
    /// save those without asking where.
    pub fn dirty_without_folder(&self) -> Vec<String> {
        self.tab_labels().into_iter().filter(|t| t.dirty && t.path.is_none()).map(|t| t.name).collect()
    }

    /// Why the quit dialog's Save All cannot start now (it switches between tabs).
    pub fn save_all_blocked(&self) -> Option<&'static str> {
        self.tabs_blocked()
    }

    /// The quit dialog's Save All: saves every project with unsaved changes into its folder, one
    /// after another (each in its own tab), then closes the window. Stops at the first failure.
    pub fn save_all_then_quit(&mut self) {
        if !self.dirty_without_folder().is_empty() {
            return;
        }
        // (The quit dialog shows Save All disabled with this reason; the window's close button
        // can open it over the lock or conflict question.)
        if let Some(why) = self.save_all_blocked() {
            self.say(Tone::Warning, format!("Save All cannot start: {}", why.to_lowercase()));
            return;
        }
        self.close_prompt = None;
        let queue: std::collections::VecDeque<usize> = self.tab_labels().iter().enumerate().filter(|(_, t)| t.dirty).map(|(i, _)| i).collect();
        self.save_all_queue = Some(queue);
        self.save_next_for_quit();
    }

    /// Saves the next project of the Save All queue, or quits when none is left. A project that
    /// was asked to open meanwhile (a double-click, a drop) stops the quit: it opens instead, and
    /// the user quits again when ready.
    pub(super) fn save_next_for_quit(&mut self) {
        let Some(queue) = self.save_all_queue.as_mut() else { return };
        let Some(i) = queue.pop_front() else {
            self.save_all_queue = None;
            if !self.pending_open.is_empty() {
                self.quit_confirmed = false;
                self.say(Tone::Warning, "Saved everything; not quitting, since a project was opened meanwhile");
                return;
            }
            self.quit_confirmed = true;
            self.close_window = true;
            return;
        };
        // Never save another tab in its place.
        if !self.show_tab(i) {
            let why = self.tabs_held().unwrap_or("the tab is gone");
            self.save_all_queue = None;
            self.quit_confirmed = false;
            self.say(Tone::Error, format!("Save All stopped: could not switch to the next project ({why})"));
            return;
        }
        match self.project.path.clone() {
            Some(path) => self.save_project(path),
            None => {
                self.save_all_queue = None;
                self.say(Tone::Error, format!("{} has no folder yet: save it from its tab first", self.project.name));
            }
        }
    }

    /// Whether any open project has unsaved changes.
    pub fn any_dirty(&self) -> bool {
        self.tab_labels().iter().any(|t| t.dirty)
    }

    /// Names of the projects with unsaved changes.
    pub fn dirty_names(&self) -> Vec<String> {
        self.tab_labels().into_iter().filter(|t| t.dirty).map(|t| t.name).collect()
    }
}
