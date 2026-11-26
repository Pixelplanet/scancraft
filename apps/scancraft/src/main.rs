//! ScanCraft: a desktop workbench for 3D scanners, in SolidCraft's look.
//!
//! `scancraft <project>` opens a project file, bundle or project folder (what a double-click in
//! Explorer does); `scancraft --mock` connects the built-in simulated scanner at start;
//! `scancraft --connect <ip>` connects a network scanner at start. See `cli.rs`. A start while
//! ScanCraft runs hands its project to the running app and ends (`single_instance.rs`).

mod ai_pack;
mod ai_segment;
mod alignment;
mod app;
mod cli;
mod dev;
mod file_picker;
mod imports;
mod isolation;
mod model;
mod project;
mod replay;
mod scan_modes;
mod single_instance;
mod storage;
mod ui_ai;
mod ui_history;
mod ui_orient;
mod ui_panels;
mod ui_shell;
mod ui_viewport;
mod worker;

use std::sync::Arc;

use eframe::egui;
use tokio::runtime::Runtime;

use crate::app::{Capture, ScanCraftApp, StartConnection};
use crate::model::Stage;

struct ScanCraft {
    app: ScanCraftApp,
    script: Option<dev::Script>,
    /// Projects handed over by later starts (`single_instance`).
    handovers: Option<std::sync::mpsc::Receiver<single_instance::Handover>>,
}

impl ScanCraft {
    fn shortcuts(&mut self, ctx: &egui::Context) {
        let app = &mut self.app;
        if ctx.egui_wants_keyboard_input() {
            return;
        }
        let (undo, redo, space, fit, esc) = ctx.input(|i| {
            let cmd = i.modifiers.command;
            (
                cmd && !i.modifiers.shift && i.key_pressed(egui::Key::Z),
                cmd && ((i.modifiers.shift && i.key_pressed(egui::Key::Z)) || i.key_pressed(egui::Key::Y)),
                i.key_pressed(egui::Key::Space),
                !cmd && i.key_pressed(egui::Key::F),
                i.key_pressed(egui::Key::Escape),
            )
        });
        if undo {
            app.undo();
        }
        if redo {
            app.redo();
        }
        if space && app.connection.is_some() {
            match app.capture {
                Capture::Idle => {
                    app.stage = Stage::Scan;
                    app.start_scan();
                }
                Capture::Running | Capture::Paused | Capture::Starting => app.stop_scan(),
                Capture::Stopping => {}
            }
        }
        if fit {
            app.fit_view();
        }
        if esc {
            if app.display_menu {
                app.display_menu = false;
            } else {
                app.close_tool();
            }
        }
    }
}

impl eframe::App for ScanCraft {
    fn raw_input_hook(&mut self, _ctx: &egui::Context, raw_input: &mut egui::RawInput) {
        // `--auto drag:…` injects one pointer event per frame.
        if let Some(event) = self.script.as_mut().and_then(|s| s.pending_input.pop_front()) {
            raw_input.events.push(event);
        }
    }

    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        let ctx = ui.ctx().clone();
        self.shortcuts(&ctx);
        // Project files and bundles dropped on the window open in tabs; scan files and project
        // folders are added as passes.
        let dropped: Vec<std::path::PathBuf> = ctx.input(|i| i.raw.dropped_files.iter().map(|f| f.path().to_path_buf()).filter(|p| !p.as_os_str().is_empty()).collect());
        let (open, add): (Vec<_>, Vec<_>) = dropped.into_iter().partition(|p| p.is_file() && storage::OpenTarget::of(p).is_some());
        for p in open {
            self.app.request_open(storage::OpenRequest::Path(p));
        }
        if !add.is_empty() {
            self.app.queue_imports(add);
        }
        // A later start handed over a project (or asked for the window).
        while let Some(h) = self.handovers.as_ref().and_then(|rx| rx.try_recv().ok()) {
            ctx.send_viewport_cmd(egui::ViewportCommand::Minimized(false));
            ctx.send_viewport_cmd(egui::ViewportCommand::Focus);
            if let single_instance::Handover::Open(p) = h {
                self.app.request_open(storage::OpenRequest::Path(p));
            }
        }
        self.app.poll_picker();
        self.app.poll_jobs();
        self.app.poll_device_events();
        self.app.poll_updates(&ctx);
        if let Some(script) = self.script.as_mut().filter(|s| s.active()) {
            script.tick(&mut self.app, &ctx);
        }
        // Quitting with unsaved projects asks first.
        if ctx.input(|i| i.viewport().close_requested()) && !self.app.quit_confirmed && self.app.any_dirty() {
            ctx.send_viewport_cmd(egui::ViewportCommand::CancelClose);
            self.app.close_prompt = Some(app::ClosePrompt::Quit);
        }
        crate::ui_ai::ai_pack_modal(&mut self.app, &ctx);
        ui_shell::recovery_prompt(&mut self.app, &ctx);
        ui_shell::close_prompt(&mut self.app, &ctx);
        // Save All on quit has saved everything.
        if std::mem::take(&mut self.app.close_window) {
            ctx.send_viewport_cmd(egui::ViewportCommand::Close);
        }
        ui_shell::app_bar(&mut self.app, ui);
        ui_shell::status_bar(&mut self.app, ui);
        ui_viewport::central(&mut self.app, ui);
        if self.app.needs_repaint() {
            ctx.request_repaint_after(std::time::Duration::from_millis(16));
        }
    }

    fn on_exit(&mut self) {
        self.app.shutdown();
    }
}

fn main() -> eframe::Result<()> {
    env_logger::init();
    let args: Vec<String> = std::env::args().skip(1).collect();
    let cli = cli::parse(&args);
    for a in &cli.unknown {
        eprintln!("scancraft: ignoring the argument {a}");
    }
    if let Some(dir) = &cli.demo_project {
        match dev::write_demo_project(std::path::Path::new(dir)) {
            Ok(()) => println!("wrote a demo project to {dir}"),
            Err(e) => eprintln!("demo project: {e}"),
        }
        return Ok(());
    }
    // One app per user: a later start hands its project to the running one and ends.
    let mut handovers = None;
    let mut wake = None;
    let _instance_guard = if cli.single_instance() {
        match single_instance::start(&single_instance::default_dir(), cli.project.as_deref()) {
            single_instance::Start::HandedOver => {
                if cli.start != StartConnection::None {
                    eprintln!("scancraft: ScanCraft is already running; the project (if any) opens there, the connection options are ignored");
                }
                return Ok(());
            }
            single_instance::Start::Primary(primary) => {
                let (guard, rx, w) = primary.split();
                handovers = Some(rx);
                wake = Some(w);
                Some(guard)
            }
            single_instance::Start::Alone(why) => {
                log::warn!("starting without the single-instance check: {why}");
                None
            }
        }
    } else {
        None
    };
    let script = cli.auto.as_deref().map(dev::Script::new);
    let rt = Arc::new(Runtime::new().expect("failed to start the async runtime"));
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default().with_title("ScanCraft").with_inner_size([1440.0, 900.0]).with_min_inner_size([1100.0, 680.0]),
        depth_buffer: 32,
        ..Default::default()
    };
    eframe::run_native(
        "ScanCraft",
        options,
        Box::new(move |cc| {
            scancraft_ui::theme::apply(&cc.egui_ctx);
            if let Some(rs) = cc.wgpu_render_state.as_ref() {
                scancraft_render::install(rs, Some(eframe::wgpu::TextureFormat::Depth32Float), 1);
            }
            if let Some(Ok(mut w)) = wake.as_ref().map(|w| w.lock()) {
                *w = Some(cc.egui_ctx.clone());
            }
            let mut app = ScanCraftApp::new(rt, cli.start.clone());
            let prefs_path = storage::Prefs::default_path();
            app.prefs = storage::Prefs::load(&prefs_path);
            app.prefs_path = Some(prefs_path);
            // Development: `--mock-modes pop4` makes the simulator report the POP 4's mode table.
            if cli.mock_modes.as_deref() == Some("pop4") {
                app.mock_modes = Some(scancraft_driver_revopoint::modes::pop4_modes());
            }
            // Bundles a crash left open: one question (not in scripted runs).
            if script.is_none() {
                app.check_leftovers();
            }
            if let Some(path) = cli.project.clone() {
                app.request_open(storage::OpenRequest::Path(path));
            }
            Ok(Box::new(ScanCraft { app, script, handovers }))
        }),
    )
}
