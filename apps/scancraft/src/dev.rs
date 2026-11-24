//! Development aids for checking the app without clicking through it:
//!
//! - `--demo-project <dir>` writes a project with two synthetic passes of a test object (the
//!   second flipped upside down and moved, unaligned) and exits;
//! - `--open <dir>` opens a project at start;
//! - `--auto <steps>` runs comma-separated steps once the app is idle: `stage:<name>`,
//!   `select:<pass>`, `start`, `stop`, `same`, `moved`, `align`, `alignall`, `accept` (the open
//!   proposal: one pass, Align All or Refine All), `pairs`, `refine`, `isolate`, `keep`,
//!   `merge`, `save:<dir>`, `import:<scan file>`, `addproject:<dir>`, `metres` (the toast's
//!   metres-to-mm conversion), `wholescene:<on|off>`, `fit`, `wait:<frames>`,
//!   `drag:<x>,<y>,<dx>,<dy>` (a left-button drag in points, injected as egui input so it works
//!   without window focus), `shot:<png path>`, `quit`.

use std::collections::VecDeque;
use std::path::Path;

use eframe::egui;
use glam::Vec3;
use scancraft_align::synthetic;
use scancraft_core::Transform;
use scancraft_driver::ScanConfig;
use scancraft_slam::{KeyframeData, VoxelMap};

use crate::app::{Capture, ScanCraftApp};
use crate::model::{Node, Stage, Tool};
use crate::project::{self, AlignState, PassMeta, PassStart, StoredPass, StoredProject};

/// Keyframes of a pass seen from `views` (pass frame), with the object placed by `object_pose`.
fn keyframes(object_pose: &Transform, views: &[Transform], seed: u64) -> Vec<KeyframeData> {
    let obj: Vec<(Vec3, Vec3)> = synthetic::object(0.5).into_iter().map(|(p, n)| (object_pose.transform_point3(p), object_pose.rotation * n)).collect();
    let mut rng = scancraft_align::cloud::Rng::new(seed);
    views
        .iter()
        .enumerate()
        .map(|(i, pose)| {
            let seen = synthetic::observe(&obj, pose, 0.12, &mut rng);
            let thinned = VoxelMap::from_points(0.5, seen);
            let q = |v: f32| (v.clamp(-1.0, 1.0) * 127.0).round() as i8;
            let points: Vec<([f32; 3], [i8; 3])> = thinned.surfels().iter().map(|s| { let n = s.normal.normalize_or_zero(); (s.position.to_array(), [q(n.x), q(n.y), q(n.z)]) }).collect();
            let centroid = thinned.surfels().iter().map(|s| s.position).sum::<Vec3>() / thinned.len().max(1) as f32;
            KeyframeData { id: i as u64, pose: *pose, rmse_mm: 0.15, visibility_conflict: 0.01, new_points: points.len(), discarded: false, imu: None, centroid, points }
        })
        .collect()
}

pub fn write_demo_project(dir: &Path) -> Result<(), String> {
    let upright = Transform::default();
    let a_views = synthetic::ring(Vec3::ZERO, 0.6, 0.0, 6.0, 14, 260.0);
    // Flipped upside down and moved on the table; scanned from above again.
    let flipped = scancraft_align::rigid(glam::Quat::from_rotation_x(std::f32::consts::PI) * glam::Quat::from_rotation_z(0.9), Vec3::new(30.0, -20.0, 0.0));
    let b_views = synthetic::ring(flipped.transform_point3(Vec3::ZERO), 0.6, 0.2, 6.2, 14, 260.0);
    let mut reference = PassMeta::new(ScanConfig::default(), None, PassStart::Moved);
    reference.alignment = AlignState::Reference;
    let second = PassMeta::new(ScanConfig::default(), None, PassStart::Moved);
    let project = StoredProject {
        name: dir.file_name().map(|n| n.to_string_lossy().to_string()).unwrap_or_else(|| "Demo".into()),
        reference: Some(1),
        next_pass_id: 3,
        passes: vec![
            StoredPass { id: 1, name: "Pass 1".into(), frames: 14, visible: true, meta: reference, keyframes: keyframes(&upright, &a_views, 1), surface: None },
            StoredPass { id: 2, name: "Pass 2 (flipped)".into(), frames: 14, visible: true, meta: second, keyframes: keyframes(&flipped, &b_views, 2), surface: None },
        ],
        merged: None,
        cleaned: None,
        mesh: None,
        settings: serde_json::json!({}),
    };
    project::save(dir, &project)
}

/// Runs `--auto` steps.
pub struct Script {
    steps: VecDeque<String>,
    cooldown: u32,
    waiting_for_shot: Option<String>,
    /// Pointer events still to inject, one per frame (`drag`).
    pub pending_input: VecDeque<egui::Event>,
}

impl Script {
    pub fn new(steps: &str) -> Self {
        // `drag:x,y,dx,dy` contains commas: re-join its arguments after splitting the steps.
        let mut list: Vec<String> = Vec::new();
        for part in steps.split(',').map(|s| s.trim().to_string()).filter(|s| !s.is_empty()) {
            match list.last_mut() {
                Some(last) if last.starts_with("drag:") && last.matches(',').count() < 3 => {
                    last.push(',');
                    last.push_str(&part);
                }
                _ => list.push(part),
            }
        }
        Script { steps: list.into(), cooldown: 60, waiting_for_shot: None, pending_input: VecDeque::new() }
    }

    pub fn active(&self) -> bool {
        !self.steps.is_empty() || self.waiting_for_shot.is_some() || !self.pending_input.is_empty()
    }

    /// Queue a left-button drag from (x, y) by (dx, dy), points.
    fn queue_drag(&mut self, x: f32, y: f32, dx: f32, dy: f32) {
        let at = |t: f32| egui::pos2(x + dx * t, y + dy * t);
        let button = |pressed: bool, pos: egui::Pos2| egui::Event::PointerButton { pos, button: egui::PointerButton::Primary, pressed, modifiers: egui::Modifiers::NONE };
        self.pending_input.push_back(egui::Event::PointerMoved(at(0.0)));
        self.pending_input.push_back(button(true, at(0.0)));
        for i in 1..=12 {
            self.pending_input.push_back(egui::Event::PointerMoved(at(i as f32 / 12.0)));
        }
        self.pending_input.push_back(button(false, at(1.0)));
    }

    pub fn tick(&mut self, app: &mut ScanCraftApp, ctx: &egui::Context) {
        if let Some(path) = &self.waiting_for_shot {
            let shot = ctx.input(|i| {
                i.raw.events.iter().find_map(|e| match e {
                    egui::Event::Screenshot { image, .. } => Some(image.clone()),
                    _ => None,
                })
            });
            if let Some(image) = shot {
                let [w, h] = image.size;
                let bytes: Vec<u8> = image.pixels.iter().flat_map(|c| c.to_array()).collect();
                match image::save_buffer(path, &bytes, w as u32, h as u32, image::ColorType::Rgba8) {
                    Ok(()) => log::info!("screenshot saved to {path}"),
                    Err(e) => log::error!("screenshot {path}: {e}"),
                }
                eprintln!("screenshot: {path}");
                self.waiting_for_shot = None;
                self.cooldown = 10;
            }
            ctx.request_repaint();
            return;
        }
        ctx.request_repaint();
        if self.cooldown > 0 {
            self.cooldown -= 1;
            return;
        }
        // Steps wait for jobs; during a capture only `wait`, `shot` and `stop` run.
        let capturing = !matches!(app.capture, Capture::Idle);
        let next = self.steps.front().map(|s| s.split(':').next().unwrap_or("").to_string()).unwrap_or_default();
        if !self.pending_input.is_empty() || app.busy.is_some() || (capturing && !matches!(next.as_str(), "wait" | "shot" | "stop" | "drag")) {
            return;
        }
        let Some(step) = self.steps.pop_front() else { return };
        eprintln!("auto: {step} ({})", app.status);
        self.cooldown = 30;
        let (cmd, arg) = step.split_once(':').unwrap_or((step.as_str(), ""));
        match cmd {
            "stage" => {
                if let Some(s) = Stage::ALL.iter().find(|s| s.label().eq_ignore_ascii_case(arg)) {
                    app.stage = *s;
                    app.tool = None;
                }
            }
            "select" => {
                if let Ok(id) = arg.parse() {
                    app.selection = Some(Node::Pass(id));
                }
            }
            "align" => {
                if let Some(id) = app.align_target() {
                    app.align_auto(id);
                }
            }
            "alignall" => app.align_all(),
            "accept" => {
                if app.poses_preview.is_some() {
                    app.accept_poses();
                } else {
                    app.accept_preview();
                }
            }
            "isolate" => {
                app.stage = Stage::Clean;
                app.open_tool(Tool::IsolateObject);
            }
            "keep" => app.run_tool(Tool::IsolateObject),
            "pairs" => {
                if let Some(id) = app.align_target() {
                    app.stage = Stage::Align;
                    app.open_tool(Tool::PointPairs(id));
                }
            }
            "start" => {
                app.stage = Stage::Scan;
                app.start_scan();
            }
            "stop" => app.stop_scan(),
            "same" => app.next_start = PassStart::SamePosition,
            "moved" => app.next_start = PassStart::Moved,
            "save" => app.save_project(std::path::PathBuf::from(arg)),
            "import" => app.import_scan(std::path::PathBuf::from(arg)),
            "addproject" => app.add_project(std::path::PathBuf::from(arg)),
            "metres" => app.run_toast_action(),
            "wholescene" => app.settings.whole_scene = arg == "on",
            "refine" => app.refine_all(),
            "merge" => app.merge_passes(),
            "fit" => app.fit_view(),
            "wait" => self.cooldown = arg.parse().unwrap_or(30),
            "drag" => {
                let v: Vec<f32> = arg.split(',').filter_map(|n| n.trim().parse().ok()).collect();
                if let [x, y, dx, dy] = v[..] {
                    self.queue_drag(x, y, dx, dy);
                }
            }
            "shot" => {
                ctx.send_viewport_cmd(egui::ViewportCommand::Screenshot(egui::UserData::default()));
                self.waiting_for_shot = Some(arg.to_string());
            }
            "quit" => ctx.send_viewport_cmd(egui::ViewportCommand::Close),
            other => eprintln!("auto: unknown step {other}"),
        }
    }
}
