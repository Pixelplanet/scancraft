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

use std::collections::HashMap;

use scancraft_render::{Camera, CameraPreset};

use super::{AlignPreview, Highlight, PairPicking, PosesPreview, ScanCraftApp, Tone};
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
    picking: Option<PairPicking>,
    object_picking: Option<ObjectPicking>,
    highlight: Option<Highlight>,
    selection: Option<Node>,
    stage: Stage,
    tool: Option<Tool>,
    camera: Camera,
    framed_radius: f32,
    marker_maps: HashMap<u32, Vec<([f32; 3], bool)>>,
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
            picking: None,
            object_picking: None,
            highlight: None,
            selection: None,
            stage: Stage::Connect,
            tool: None,
            camera,
            framed_radius: 0.0,
            marker_maps: HashMap::new(),
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
    pub path: Option<std::path::PathBuf>,
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
        swap(&mut self.picking, &mut tab.picking);
        swap(&mut self.object_picking, &mut tab.object_picking);
        swap(&mut self.highlight, &mut tab.highlight);
        swap(&mut self.selection, &mut tab.selection);
        swap(&mut self.stage, &mut tab.stage);
        swap(&mut self.tool, &mut tab.tool);
        swap(&mut self.camera, &mut tab.camera);
        swap(&mut self.framed_radius, &mut tab.framed_radius);
        swap(&mut self.marker_maps, &mut tab.marker_maps);
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

    /// Why tabs cannot be switched, opened or closed now.
    pub fn tabs_blocked(&self) -> Option<&'static str> {
        if self.busy.is_some() {
            Some("Another operation is running")
        } else if self.capture != super::Capture::Idle {
            Some("Stop the capture first")
        } else if self.picker.is_open() {
            Some("Close the file dialog first")
        } else if !self.pending_imports.is_empty() {
            Some("Wait for the imports to finish")
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
                    TabLabel { name: self.project.name.clone(), dirty: self.dirty(), path: self.project.path.clone(), active: true }
                } else {
                    TabLabel { name: t.project.name.clone(), dirty: t.dirty, path: t.project.path.clone(), active: false }
                }
            })
            .collect()
    }

    /// Switches to tab `i`.
    pub fn switch_tab(&mut self, i: usize) {
        if i == self.active_tab || i >= self.tabs.len() || self.tabs_blocked().is_some() {
            return;
        }
        self.park_active();
        self.activate(i);
        self.close_prompt = None;
        self.pending_close = false;
        self.toast = None;
        self.say(Tone::Normal, format!("Switched to {}", self.project.name));
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
        if self.tabs_blocked().is_some() {
            return;
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
    /// Closing the last tab leaves one empty project.
    pub fn close_active_tab(&mut self) {
        self.close_prompt = None;
        self.pending_close = false;
        let closing = self.project.name.clone();
        if self.tabs.len() == 1 {
            let name = ProjectInfo::default().name;
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

    /// Whether any open project has unsaved changes.
    pub fn any_dirty(&self) -> bool {
        self.tab_labels().iter().any(|t| t.dirty)
    }

    /// Names of the projects with unsaved changes.
    pub fn dirty_names(&self) -> Vec<String> {
        self.tab_labels().into_iter().filter(|t| t.dirty).map(|t| t.name).collect()
    }
}
