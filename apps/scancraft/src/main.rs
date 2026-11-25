//! ScanCraft: a desktop workbench for 3D scanners, in SolidCraft's look.
//!
//! `scancraft --mock` connects the built-in simulated scanner at start;
//! `scancraft --connect <ip>` connects a network scanner at start.

mod alignment;
mod app;
mod dev;
mod file_picker;
mod imports;
mod isolation;
mod model;
mod project;
mod replay;
mod scan_modes;
mod ui_history;
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
        // Scan files and project folders dropped on the window are added as passes.
        let dropped: Vec<std::path::PathBuf> = ctx.input(|i| i.raw.dropped_files.iter().map(|f| f.path().to_path_buf()).filter(|p| !p.as_os_str().is_empty()).collect());
        if !dropped.is_empty() {
            self.app.queue_imports(dropped);
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
        ui_shell::close_prompt(&mut self.app, &ctx);
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
    let args: Vec<String> = std::env::args().collect();
    let value = |flag: &str| args.iter().position(|a| a == flag).and_then(|i| args.get(i + 1)).cloned();
    if let Some(dir) = value("--demo-project") {
        match dev::write_demo_project(std::path::Path::new(&dir)) {
            Ok(()) => println!("wrote a demo project to {dir}"),
            Err(e) => eprintln!("demo project: {e}"),
        }
        return Ok(());
    }
    let open = value("--open");
    let script = value("--auto").map(|s| dev::Script::new(&s));
    let start = if let Some(i) = args.iter().position(|a| a == "--replay") {
        // Development: play a recorded marker scan as if it came from a scanner.
        let recording = args.get(i + 1).cloned().unwrap_or_default();
        let camparam = args.get(i + 2).cloned().unwrap_or_default();
        StartConnection::Replay(recording.into(), camparam.into())
    } else if args.iter().any(|a| a == "--mock") {
        StartConnection::Mock
    } else if let Some(ip) = args.iter().position(|a| a == "--connect").and_then(|i| args.get(i + 1)) {
        StartConnection::Ip(ip.clone())
    } else {
        StartConnection::None
    };
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
            let mut app = ScanCraftApp::new(rt, start);
            // Development: `--mock-modes pop4` makes the simulator report the POP 4's mode table.
            if args.windows(2).any(|w| w[0] == "--mock-modes" && w[1] == "pop4") {
                app.mock_modes = Some(scancraft_driver_revopoint::modes::pop4_modes());
            }
            if let Some(dir) = open {
                app.open_project(std::path::PathBuf::from(dir));
            }
            Ok(Box::new(ScanCraft { app, script }))
        }),
    )
}
