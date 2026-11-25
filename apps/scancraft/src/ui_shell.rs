//! The window chrome: application bar, the workspace and ribbon cards, and the status bar.

use eframe::egui::{self, Rect, pos2, vec2};
use scancraft_ui::kit::{self, RibbonButton, RibbonGroup};
use scancraft_ui::{Tokens, fonts};

use crate::app::{Capture, ClosePrompt, ScanCraftApp, Tone};
use crate::file_picker::PickPurpose;
use crate::model::{ExportFormat, Node, Stage, Tool, thousands};

/// The application bar along the top of the window (`toolbar::app_bar`).
pub fn app_bar(app: &mut ScanCraftApp, root: &mut egui::Ui) {
    let t = Tokens::get();
    egui::Panel::top("app_bar").exact_size(kit::APP_BAR_HEIGHT).show_separator_line(false).frame(egui::Frame::NONE.fill(t.app_bar)).show(root, |ui| {
        let r = ui.max_rect();
        kit::brand_mark(ui.painter(), pos2(r.left() + 20.0, r.center().y), 18.0);
        ui.painter().text(pos2(r.left() + 36.0, r.center().y), egui::Align2::LEFT_CENTER, "ScanCraft", fonts::semibold(t.font_heading), t.app_bar_text);
        let mut x = r.left() + 128.0;
        kit::app_bar_separator(ui.painter(), r, x - 18.0);
        let undo_tip = match app.history.undo_label() {
            Some(l) => format!("Undo {l} (Ctrl+Z)"),
            None => "Undo (Ctrl+Z)".to_string(),
        };
        let redo_tip = match app.history.redo_label() {
            Some(l) => format!("Redo {l} (Ctrl+Y)"),
            None => "Redo (Ctrl+Y)".to_string(),
        };
        let undo = Rect::from_center_size(pos2(x, r.center().y), vec2(32.0, 32.0));
        if kit::app_bar_button(ui, undo, "undo", &undo_tip, app.history.can_undo() && app.can_edit()).clicked() {
            app.undo();
        }
        x += 32.0;
        let redo = Rect::from_center_size(pos2(x, r.center().y), vec2(32.0, 32.0));
        if kit::app_bar_button(ui, redo, "redo", &redo_tip, app.history.can_redo() && app.can_edit()).clicked() {
            app.redo();
        }
        x += 32.0;
        let history = Rect::from_center_size(pos2(x, r.center().y), vec2(32.0, 32.0));
        let steps = app.history.timeline().len();
        let history_tip = if steps == 0 { "History (nothing to undo yet)".to_string() } else { format!("History: {steps} step{} · jump to any of them", if steps == 1 { "" } else { "s" }) };
        let resp = kit::app_bar_button(ui, history, "rollback", &history_tip, true);
        app.history_button_hovered = resp.hovered();
        if resp.clicked() {
            app.show_history = !app.show_history;
        }
        let history_anchor = pos2(history.left(), r.bottom() + 4.0);
        x += 26.0;
        kit::app_bar_separator(ui.painter(), r, x);
        // Project files.
        x += 22.0;
        let new = Rect::from_center_size(pos2(x, r.center().y), vec2(32.0, 32.0));
        if kit::app_bar_button(ui, new, "new", app.tabs_blocked().unwrap_or("New Project (in a new tab)"), app.tabs_blocked().is_none()).clicked() {
            app.new_tab();
        }
        x += 32.0;
        let open = Rect::from_center_size(pos2(x, r.center().y), vec2(32.0, 32.0));
        if kit::app_bar_button(ui, open, "open", app.edit_blocked().unwrap_or("Open Project…"), app.can_edit()).clicked() {
            app.pick(&ui.ctx().clone(), PickPurpose::OpenProject);
        }
        x += 32.0;
        let save = Rect::from_center_size(pos2(x, r.center().y), vec2(32.0, 32.0));
        let save_tip = match &app.project.path {
            Some(p) => format!("Save Project ({})", p.display()),
            None => "Save Project…".to_string(),
        };
        let save_why = app.tool_blocked(Tool::SaveProject);
        if kit::app_bar_button(ui, save, "save", save_why.unwrap_or(&save_tip), save_why.is_none()).clicked() {
            app.open_tool(Tool::SaveProject);
        }
        x += 26.0;
        kit::app_bar_separator(ui.painter(), r, x);
        // Passes from outside the project.
        x += 22.0;
        let import = Rect::from_center_size(pos2(x, r.center().y), vec2(32.0, 32.0));
        if kit::app_bar_button(ui, import, "import", app.import_blocked().unwrap_or("Import Scan… (PLY, OBJ, STL as a new pass)"), app.import_blocked().is_none()).clicked() {
            app.pick(&ui.ctx().clone(), PickPurpose::ImportScans);
        }
        x += 32.0;
        let add = Rect::from_center_size(pos2(x, r.center().y), vec2(32.0, 32.0));
        if kit::app_bar_button(ui, add, "folder", app.import_blocked().unwrap_or("Add Project… (append another project's passes)"), app.import_blocked().is_none()).clicked() {
            app.pick(&ui.ctx().clone(), PickPurpose::AddProjects);
        }
        x += 26.0;
        kit::app_bar_separator(ui.painter(), r, x);
        // The project tabs, as SolidCraft's document tabs: name, a dot while it has unsaved
        // changes, a close button; the pill on the right limits how wide they may get.
        project_tabs(app, ui, r, x + 10.0, r.right() - 220.0);
        // Connection pill at the right.
        let (dot, text) = match &app.connection {
            Some(c) => (t.agent_ok, format!("{} connected", c.short_name())),
            None if app.connecting => (t.agent_busy, "Connecting…".to_string()),
            None => (t.text_dim, "No scanner".to_string()),
        };
        crate::ui_history::timeline(app, ui.ctx(), history_anchor);
        let pill = kit::app_bar_pill(ui, r.right() - 10.0, r, dot, &text);
        let pill = match &app.connection {
            Some(c) => pill.on_hover_text(format!("{} · {}\nSerial {}\nFirmware {}", c.info.model, c.info.active_transport.description(), c.info.serial_number, c.info.firmware_version)),
            None => pill.on_hover_text("Open the Connect tab to find a scanner"),
        };
        if pill.clicked() {
            app.stage = Stage::Connect;
            app.tool = None;
        }
    });
}

/// The tab strip between `left` and `right`: click a tab to switch to it, × to close it (asks
/// first if it has unsaved changes). A tab keeps its own width (96–220 pt, the name ellipsised);
/// when they do not all fit, the run of whole tabs around the active one shows and a chevron at
/// the right end opens the full list.
fn project_tabs(app: &mut ScanCraftApp, ui: &mut egui::Ui, bar: Rect, left: f32, right: f32) {
    let t = Tokens::get();
    let labels = app.tab_labels();
    let blocked = app.tabs_blocked();
    let (gap, close_w, dot_w, pad) = (4.0, 18.0, 12.0, 12.0);
    let (min_w, max_w, overflow_w) = (96.0, 220.0, 28.0);
    let font = fonts::semibold(t.font_label);
    let widths: Vec<f32> = labels
        .iter()
        .map(|l| (ui.painter().layout_no_wrap(l.name.clone(), font.clone(), t.app_bar_text).size().x + pad + dot_w + close_w + 6.0).clamp(min_w, max_w))
        .collect();
    let room = (right - left).max(0.0);
    let fits = widths.iter().sum::<f32>() + gap * labels.len().saturating_sub(1) as f32 <= room;
    // When they do not all fit the last `overflow_w` points hold the list button; the tabs must
    // stay inside [left, tabs_right].
    let tabs_right = if fits { right } else { right - overflow_w - gap };
    let active = labels.iter().position(|l| l.active).unwrap_or(0);
    let (mut first, mut last) = (0usize, labels.len());
    if !fits {
        // The window of consecutive tabs containing the active one, as many whole tabs as fit.
        let avail = (tabs_right - left).max(0.0);
        let mut used = 0.0f32;
        first = active;
        last = active + 1;
        used += widths[active];
        loop {
            if last < labels.len() && used + gap + widths[last] <= avail {
                used += gap + widths[last];
                last += 1;
            } else if first > 0 && used + gap + widths[first - 1] <= avail {
                used += gap + widths[first - 1];
                first -= 1;
            } else {
                break;
            }
        }
    }
    let mut x = left;
    let mut switch_to = None;
    let mut close = None;
    for (i, l) in labels.iter().enumerate().take(last).skip(first) {
        let w = widths[i].min(tabs_right - x);
        if w < 40.0 {
            break;
        }
        let tab = Rect::from_min_size(pos2(x, bar.center().y - 13.0), vec2(w, 26.0));
        x += w + gap;
        let resp = ui.interact(tab, ui.id().with(("project_tab", i)), egui::Sense::click());
        let fill = if l.active {
            t.app_bar.lerp_to_gamma(egui::Color32::WHITE, 0.14)
        } else if resp.hovered() {
            t.app_bar_hover_button
        } else {
            t.app_bar.lerp_to_gamma(egui::Color32::WHITE, 0.04)
        };
        ui.painter().rect_filled(tab, t.radius_sm, fill);
        let ink = if l.active { t.app_bar_text } else { t.app_bar_text.gamma_multiply(0.7) };
        let text_rect = Rect::from_min_max(pos2(tab.left() + pad, tab.top()), pos2(tab.right() - dot_w - close_w - 4.0, tab.bottom()));
        kit::paint_text(ui.painter(), text_rect, &l.name, font.clone(), ink);
        if l.dirty {
            ui.painter().circle_filled(pos2(tab.right() - close_w - dot_w / 2.0 - 2.0, tab.center().y), 3.0, ink);
        }
        let close_rect = Rect::from_center_size(pos2(tab.right() - close_w / 2.0 - 3.0, tab.center().y), vec2(close_w - 2.0, close_w - 2.0));
        let close_resp = ui.interact(close_rect, ui.id().with(("project_tab_close", i)), egui::Sense::click());
        if close_resp.hovered() {
            ui.painter().rect_filled(close_rect, t.radius_sm, t.app_bar.lerp_to_gamma(egui::Color32::WHITE, 0.2));
        }
        let c = close_rect.center();
        let s = 3.5;
        let stroke = egui::Stroke::new(1.3, ink);
        ui.painter().line_segment([pos2(c.x - s, c.y - s), pos2(c.x + s, c.y + s)], stroke);
        ui.painter().line_segment([pos2(c.x - s, c.y + s), pos2(c.x + s, c.y - s)], stroke);
        let state = match &l.path {
            Some(p) => format!("{}{}", p.display(), if l.dirty { " · unsaved changes" } else { "" }),
            None if l.dirty => "Not saved yet · unsaved changes".to_string(),
            None => "Not saved yet".to_string(),
        };
        let tip = match blocked {
            Some(why) if !l.active => format!("{}\n{state}\n{why}", l.name),
            _ => format!("{}\n{state}", l.name),
        };
        if close_resp.on_hover_text(if l.dirty { "Close (asks to save first)" } else { "Close" }).clicked() {
            close = Some(i);
        } else if resp.on_hover_text(tip).clicked() {
            switch_to = Some(i);
        }
    }
    if !fits {
        // The "all tabs" chevron; its menu lists every tab so hidden ones stay reachable.
        let more = Rect::from_min_max(pos2(tabs_right + gap, bar.center().y - 13.0), pos2(right, bar.center().y + 13.0));
        let more_resp = ui.interact(more, ui.id().with("project_tabs_more"), egui::Sense::click());
        if more_resp.hovered() {
            ui.painter().rect_filled(more, t.radius_sm, t.app_bar_hover_button);
        }
        kit::dropdown_indicator(ui.painter(), Rect::from_center_size(more.center(), vec2(16.0, 16.0)), t.app_bar_text);
        let mut from_menu = None;
        egui::Popup::menu(&more_resp.on_hover_text("All open tabs")).close_behavior(egui::PopupCloseBehavior::CloseOnClick).show(|ui| {
            ui.set_min_width(200.0);
            for (i, l) in labels.iter().enumerate() {
                let text = format!("{}{}", if l.dirty { "• " } else { "" }, l.name);
                let enabled = l.active || blocked.is_none();
                let r = ui.add_enabled_ui(enabled, |ui| ui.selectable_label(l.active, text)).inner;
                let r = match (l.active, blocked) {
                    (false, Some(why)) => r.on_disabled_hover_text(why),
                    _ => r,
                };
                if r.clicked() {
                    from_menu = Some(i);
                    ui.close();
                }
            }
        });
        if let Some(i) = from_menu {
            switch_to = Some(i);
        }
    }
    if let Some(i) = close {
        app.request_close_tab(i);
    } else if let Some(i) = switch_to {
        app.switch_tab(i);
    }
}

/// The close dialog for a dirty project, or for quitting with dirty projects (`kit::confirm_modal`):
/// the header's ×, a click outside and Esc all answer `Choice::Cancel`.
pub fn close_prompt(app: &mut ScanCraftApp, ctx: &egui::Context) {
    let Some(prompt) = app.close_prompt else { return };
    let warning = if scancraft_ui::icons::exists("warning") {
        "warning"
    } else if scancraft_ui::icons::exists("alert") {
        "alert"
    } else {
        "save"
    };
    let (icon, title, message, primary, destructive) = match prompt {
        ClosePrompt::Tab => (
            "save",
            "Close Project",
            format!("{} has unsaved changes. Save them before closing?", app.project.name),
            Some(if app.project.path.is_some() { "Save" } else { "Save…" }),
            "Discard Changes",
        ),
        ClosePrompt::Quit => (
            warning,
            "Quit ScanCraft",
            format!("Unsaved changes in {}. Quit anyway? Save each project from its tab first to keep them.", app.dirty_names().join(", ")),
            None,
            "Quit Without Saving",
        ),
    };
    let actions = kit::Actions { primary: primary.map(|label| (label, true)), cancel: Some("Cancel"), destructive: Some((destructive, true)), ..Default::default() };
    let choice = kit::confirm_modal(ctx, egui::Id::new("close_prompt"), icon, title, &message, &actions);
    match (prompt, choice) {
        (_, Some(kit::Choice::Cancel)) => app.close_prompt = None,
        (ClosePrompt::Tab, Some(kit::Choice::Primary)) => app.save_then_close(),
        (ClosePrompt::Tab, Some(kit::Choice::Destructive)) => app.close_active_tab(),
        (ClosePrompt::Quit, Some(kit::Choice::Destructive)) => {
            app.close_prompt = None;
            app.quit_confirmed = true;
            ctx.send_viewport_cmd(egui::ViewportCommand::Close);
        }
        _ => {}
    }
}

/// The status bar along the bottom (`status_bar::paint`): the message at the left; tracking,
/// frame rate, points and distance at the right. Unknown values are "—".
pub fn status_bar(app: &mut ScanCraftApp, root: &mut egui::Ui) {
    let t = Tokens::get();
    egui::Panel::bottom("status_bar").exact_size(kit::STATUS_BAR_HEIGHT).show_separator_line(false).frame(egui::Frame::NONE.fill(t.toolbar)).show(root, |ui| {
        let r = ui.max_rect();
        let painter = ui.painter().clone();
        painter.hline(r.x_range(), r.top() + 0.5, t.primary_border(t.border));
        let mut right = r.right() - t.space_6;
        let live = app.capture != Capture::Idle || app.previewing;
        let distance = match (live, app.distance_mm) {
            (true, Some(d)) => format!("{d:.0} mm"),
            _ => "—".to_string(),
        };
        right = kit::status_item(&painter, r, right, None, &format!("Distance {distance}"), t.text_dim, false);
        let points = thousands(app.total_points());
        right = kit::status_item(&painter, r, right, None, &format!("{points} points"), t.text_dim, true);
        // Honest rates: frames arriving from the scanner, and (in marker modes) frames that carried
        // a new marker observation, which the POP 4 sends only on every other packet.
        let (fps, fps_tip) = match (live, app.rates, app.fps.fps()) {
            (true, Some(rates), _) if rates.incoming > 0.0 => match rates.marker {
                Some(m) => (
                    format!("{:.0} fps in · {:.0} marker updates/s", rates.incoming, m),
                    format!(
                        "{:.1} frames/s arrive from the scanner, {:.1} are processed.\n{:.1}/s carry a new marker observation: the scanner repeats the IR images on every other frame in marker modes, so the pose from markers updates at this rate.",
                        rates.incoming, rates.processed, m
                    ),
                ),
                None => (
                    format!("{:.0} fps in", rates.incoming),
                    format!("{:.1} frames/s arrive from the scanner, {:.1} are processed (the rest are skipped to stay current).", rates.incoming, rates.processed),
                ),
            },
            (true, _, Some(f)) => (format!("{f:.0} fps"), "Frames shown per second".to_string()),
            _ => ("— fps".to_string(), "No frames are arriving".to_string()),
        };
        let before = right;
        right = kit::status_item(&painter, r, right, None, &fps, t.text_dim, true);
        ui.interact(Rect::from_min_max(pos2(right, r.top()), pos2(before, r.bottom())), ui.id().with("status_fps"), egui::Sense::hover()).on_hover_text(fps_tip);
        let (dot, tracking) = match (&app.tracking, app.capture) {
            (Some(s), Capture::Running | Capture::Paused) => match s.state {
                scancraft_slam::TrackingState::Tracking => (t.agent_ok, "Tracking".to_string()),
                scancraft_slam::TrackingState::Slow => (t.warning, "Tracking (slow down)".to_string()),
                scancraft_slam::TrackingState::Lost => (t.error, "Tracking lost".to_string()),
                scancraft_slam::TrackingState::Idle => (t.text_dim, "Tracking idle".to_string()),
            },
            _ => (t.text_dim.gamma_multiply(0.6), "Tracking —".to_string()),
        };
        right = kit::status_item(&painter, r, right, Some(dot), &tracking, t.text_dim, true);
        let colour = match app.tone {
            Tone::Normal => t.text,
            Tone::Success => t.form_success,
            Tone::Warning => t.warning,
            Tone::Error => t.error,
        };
        let message = match &app.busy {
            Some(b) => format!("{b}…"),
            None => app.status.clone(),
        };
        kit::status_message(&painter, r, right - t.space_6, &message, colour);
        ui.interact(Rect::from_min_max(r.min, pos2(right, r.max.y)), ui.id().with("status_msg"), egui::Sense::hover()).on_hover_text(&message);
    });
}

/// The workspace card: the connected scanner, or "No scanner".
pub fn workspace_card(app: &mut ScanCraftApp, ui: &mut egui::Ui, rect: Rect) {
    let (title, caption, tip) = match &app.connection {
        Some(c) => (c.short_name(), c.link(), format!("{} · serial {}", c.info.model, c.info.serial_number)),
        None => ("Scanner".to_string(), "Not connected".to_string(), "Open the Connect tab".to_string()),
    };
    if kit::workspace_card(ui, rect, "camera", &title, &caption, &tip).clicked() {
        app.stage = Stage::Connect;
        app.tool = None;
        app.selection = Some(Node::Device);
    }
}

/// A ribbon command: `blocked` is why it cannot run right now (None = it can); a disabled
/// button's tooltip is that reason, so a disabled button can never be left unexplained.
fn rb(id: &'static str, icon: &'static str, label: &'static str, blocked: Option<&'static str>, pressed: bool, tip: &'static str) -> RibbonButton<'static> {
    RibbonButton { id, icon, label, enabled: blocked.is_none(), pressed, tip: blocked.unwrap_or(tip) }
}

/// The button groups of the current stage's ribbon, split out of [`ribbon`] so the disabled-tip
/// rule stays structural (`rb`): enabled == `blocked.is_none()` and the tip is the reason.
fn ribbon_groups(app: &ScanCraftApp) -> Vec<RibbonGroup<'static>> {
    let connected = app.connection.is_some();
    let idle = app.capture == Capture::Idle;
    let tool = app.tool;
    let pressed = |t: Tool| tool == Some(t);
    let edit = app.edit_blocked();
    match app.stage {
        Stage::Connect => vec![
            RibbonGroup {
                label: "Device",
                buttons: vec![
                    rb(
                        "discover",
                        "web",
                        "Find Scanners",
                        if connected { Some("Disconnect first") } else if app.discovering { Some("Searching…") } else { None },
                        app.discovering,
                        "Search USB and the local network for scanners",
                    ),
                    rb(
                        "connect",
                        "drive",
                        "Connect",
                        if connected {
                            Some("Disconnect first")
                        } else if app.connecting {
                            Some("Connecting…")
                        } else if app.discovered.is_empty() {
                            Some("Find scanners first")
                        } else {
                            None
                        },
                        app.connecting,
                        "Connect to the scanner picked in the panel",
                    ),
                    rb(
                        "ip",
                        "open",
                        "Connect by IP",
                        if connected { Some("Disconnect first") } else if app.connecting { Some("Connecting…") } else { None },
                        pressed(Tool::ConnectIp),
                        "Connect to a scanner at a known network address",
                    ),
                    rb("disconnect", "cut", "Disconnect", (!connected).then_some("Connect a scanner first"), false, "Stop streaming and release the scanner"),
                ],
            },
            RibbonGroup {
                label: "Simulator",
                buttons: vec![rb(
                    "mock",
                    "box",
                    "Simulated Scanner",
                    if connected { Some("Disconnect first") } else if app.connecting { Some("Connecting…") } else { None },
                    false,
                    "Connect the built-in simulator: synthetic depth and IR frames for trying the workflow",
                )],
            },
        ],
        Stage::Scan => vec![
            RibbonGroup {
                label: "Capture",
                buttons: vec![
                    rb(
                        "start",
                        "play",
                        "Start",
                        if !connected { Some("Connect a scanner first") } else if !idle { Some("Stop the capture first") } else { None },
                        matches!(app.capture, Capture::Running | Capture::Starting),
                        "Start a new pass (Space)",
                    ),
                    rb(
                        "pause",
                        "pause",
                        if app.capture == Capture::Paused { "Resume" } else { "Pause" },
                        (!matches!(app.capture, Capture::Running | Capture::Paused)).then_some("No capture is running"),
                        app.capture == Capture::Paused,
                        "Pause or resume adding points to this pass",
                    ),
                    rb(
                        "stop",
                        "stop_capture",
                        "Stop",
                        (!matches!(app.capture, Capture::Running | Capture::Paused | Capture::Starting)).then_some("No capture is running"),
                        false,
                        "Stop capturing and switch the projector off (Space)",
                    ),
                ],
            },
            RibbonGroup {
                label: "Aim",
                buttons: vec![
                    rb(
                        "preview",
                        "camera",
                        "Live Preview",
                        if !connected {
                            Some("Connect a scanner first")
                        } else if !idle {
                            Some("The capture shows its own preview")
                        } else if app.preview_starting {
                            Some("Starting the preview…")
                        } else {
                            None
                        },
                        app.previewing,
                        "Show the camera image without capturing",
                    ),
                    rb("inset", "inspect", "Camera Inset", None, app.show_inset, "Show the camera image over the view while live"),
                ],
            },
            RibbonGroup {
                label: "Add Passes",
                buttons: vec![
                    rb("import_scan", "import", "Import Scan", app.import_blocked(), false, "Add PLY, OBJ or STL scans (yours or from other scan software) as new passes to align with the others"),
                    rb("add_project", "folder", "Add Project", app.import_blocked(), false, "Append the passes of other ScanCraft projects, unaligned"),
                ],
            },
            RibbonGroup {
                label: "Session",
                buttons: vec![rb(
                    "clear",
                    "delete",
                    "Clear All",
                    edit.or((!app.has_geometry()).then_some("Nothing to clear")),
                    false,
                    "Remove every pass and result",
                )],
            },
        ],
        Stage::Align => {
            let target = app.align_target();
            let blocked = app.align_blocked(target);
            let picking = matches!(app.tool, Some(Tool::PointPairs(_)));
            let proposed = app.align_preview.is_some() || app.poses_preview.is_some();
            let all_blocked = app.align_all_blocked();
            vec![
                RibbonGroup {
                    label: "Register",
                    buttons: vec![
                        rb(
                            "align_all",
                            "align",
                            "Align All",
                            all_blocked,
                            app.busy.as_deref().is_some_and(|b| b.starts_with("Aligning all")),
                            "Align every pass against every other and place them all from the reference (also inside and outside scans of one part)",
                        ),
                        rb(
                            "align_auto",
                            "checkpoint",
                            "Align Pass",
                            if proposed { Some("Accept or reject the proposed alignment first") } else { blocked },
                            app.busy.as_deref().is_some_and(|b| b.starts_with("Aligning") && !b.starts_with("Aligning all")),
                            "Find the selected (or next unaligned) pass's place in the project automatically",
                        ),
                        rb(
                            "align_pick",
                            "point",
                            "Point Pairs",
                            blocked.filter(|_| !picking),
                            picking,
                            "Click three or more matching points on the model and the pass, side by side",
                        ),
                        rb("align_markers", "snap", "Markers", Some("No markers on the object were recorded in these passes"), false, ""),
                    ],
                },
                RibbonGroup {
                    label: "Project",
                    buttons: vec![
                        rb("refine", "update", "Refine All", app.refine_blocked(), false, "Fine-tune all aligned passes together so errors do not add up"),
                        rb("merge", "combine", "Merge", app.merge_blocked(), false, "Fuse the included aligned passes into one model; the passes stay unchanged"),
                    ],
                },
                RibbonGroup {
                    label: "Inspect",
                    buttons: vec![
                        rb("colour_passes", "inspect", "Pass Colours", None, app.colour_passes, "Show each pass in its own colour on this tab"),
                        rb("deviation", "zebra", "Deviation", Some("Not available yet: pass-to-pass deviation map"), false, ""),
                    ],
                },
            ]
        }
        Stage::Clean => vec![
            RibbonGroup {
                label: "Filter",
                buttons: vec![
                    rb(
                        "isolate",
                        "body",
                        "Isolate Object",
                        app.tool_blocked(Tool::IsolateObject).filter(|_| !pressed(Tool::IsolateObject)),
                        pressed(Tool::IsolateObject),
                        "Keep the object: the table, marker blocks and loose fragments go. Click a cluster to keep it, Shift+click to leave one out",
                    ),
                    rb("plane", "plane", "Remove Plane", app.tool_blocked(Tool::RemovePlane), pressed(Tool::RemovePlane), "Remove the turntable or table plane (RANSAC)"),
                    rb("outliers", "filter", "Outliers", app.tool_blocked(Tool::Outliers), pressed(Tool::Outliers), "Remove isolated points (statistical outlier removal)"),
                    rb("crop", "box", "Crop Box", app.tool_blocked(Tool::Crop), pressed(Tool::Crop), "Keep only the points inside a box"),
                    rb("downsample", "snap", "Downsample", app.tool_blocked(Tool::Downsample), pressed(Tool::Downsample), "Merge points into a voxel grid"),
                ],
            },
            RibbonGroup {
                label: "Reset",
                buttons: vec![rb(
                    "revert",
                    "revert",
                    "Reset Cleaning",
                    edit.or((!app.can_reset_cleaning()).then_some("No shown pass is cleaned")),
                    false,
                    "Take the shown passes back to how they were scanned or imported",
                )],
            },
        ],
        Stage::Mesh => vec![
            RibbonGroup {
                label: "Create",
                buttons: vec![
                    rb("reconstruct", "mesh_body", "Reconstruct", app.tool_blocked(Tool::Reconstruct), pressed(Tool::Reconstruct), "Build a smooth surface from the points"),
                    rb("make_solid", "body", "Make Solid", app.tool_blocked(Tool::MakeSolid), pressed(Tool::MakeSolid), "Build a closed, watertight solid of a thin-walled part"),
                ],
            },
            RibbonGroup {
                label: "Refine",
                buttons: vec![
                    rb("smooth", "surface", "Smooth", app.tool_blocked(Tool::Smooth), pressed(Tool::Smooth), "Smooth the surface while keeping edges"),
                    rb("decimate", "reach", "Decimate", app.tool_blocked(Tool::Decimate), pressed(Tool::Decimate), "Reduce the triangle count"),
                    rb("fill_holes", "patch", "Fill Holes", app.tool_blocked(Tool::FillHoles), pressed(Tool::FillHoles), "Close holes up to a size, shaped to continue the surface"),
                ],
            },
            RibbonGroup {
                label: "Manage",
                buttons: vec![rb(
                    "delete_mesh",
                    "delete",
                    "Delete Mesh",
                    edit.or(app.mesh.is_none().then_some("No mesh")),
                    false,
                    "Delete the mesh",
                )],
            },
        ],
        Stage::Export => vec![
            RibbonGroup {
                label: "Points",
                buttons: vec![rb(
                    "export_ply_cloud",
                    "export",
                    "Point Cloud PLY",
                    app.tool_blocked(Tool::Export(ExportFormat::PlyCloud)),
                    pressed(Tool::Export(ExportFormat::PlyCloud)),
                    "Binary PLY of the merged model when shown, else of the shown passes",
                )],
            },
            RibbonGroup {
                label: "Mesh",
                buttons: vec![
                    rb("export_ply_mesh", "export", "Mesh PLY", app.tool_blocked(Tool::Export(ExportFormat::PlyMesh)), pressed(Tool::Export(ExportFormat::PlyMesh)), "Binary PLY mesh"),
                    rb("export_obj", "export", "OBJ", app.tool_blocked(Tool::Export(ExportFormat::ObjMesh)), pressed(Tool::Export(ExportFormat::ObjMesh)), "Wavefront OBJ"),
                    rb("export_stl", "export", "STL", app.tool_blocked(Tool::Export(ExportFormat::StlMesh)), pressed(Tool::Export(ExportFormat::StlMesh)), "Binary STL for printing"),
                ],
            },
        ],
    }
}

/// The ribbon card for the current stage; runs the clicked command.
pub fn ribbon(app: &mut ScanCraftApp, ui: &mut egui::Ui, rect: Rect) {
    let tabs: Vec<&str> = Stage::ALL.iter().map(|s| s.label()).collect();
    let mut active = app.stage.index();
    let groups = ribbon_groups(app);
    let clicked = kit::ribbon_card(ui, rect, &tabs, &mut active, &groups).map(str::to_string);
    if active != app.stage.index() {
        app.stage = Stage::ALL[active];
        if app.tool.is_some_and(|t| t.stage() != app.stage && !t.is_file()) {
            app.close_tool();
        }
    }
    let Some(id) = clicked else { return };
    match id.as_str() {
        "discover" => app.discover(),
        "connect" => app.connect_discovered(),
        "ip" => app.open_tool(Tool::ConnectIp),
        "disconnect" => app.disconnect(),
        "mock" => app.connect_mock(),
        "start" => app.start_scan(),
        "pause" => app.toggle_pause(),
        "stop" => app.stop_scan(),
        "preview" => {
            if app.previewing {
                app.stop_preview();
            } else {
                app.start_preview();
            }
        }
        "inset" => app.show_inset = !app.show_inset,
        "clear" => app.clear_session(),
        "import_scan" => app.pick(&ui.ctx().clone(), PickPurpose::ImportScans),
        "add_project" => app.pick(&ui.ctx().clone(), PickPurpose::AddProjects),
        "align_all" => app.align_all(),
        "align_auto" => {
            if let Some(id) = app.align_target() {
                app.align_auto(id);
            }
        }
        "align_pick" => {
            if matches!(app.tool, Some(Tool::PointPairs(_))) {
                app.tool = None;
                app.picking = None;
            } else if let Some(id) = app.align_target() {
                app.open_tool(Tool::PointPairs(id));
            }
        }
        "refine" => app.refine_all(),
        "merge" => app.merge_passes(),
        "colour_passes" => app.colour_passes = !app.colour_passes,
        "isolate" => {
            if app.tool == Some(Tool::IsolateObject) {
                app.close_tool();
            } else {
                app.open_tool(Tool::IsolateObject);
            }
        }
        "plane" => app.open_tool(Tool::RemovePlane),
        "outliers" => app.open_tool(Tool::Outliers),
        "crop" => app.open_tool(Tool::Crop),
        "downsample" => app.open_tool(Tool::Downsample),
        "revert" => app.reset_cleaning(),
        "reconstruct" => app.open_tool(Tool::Reconstruct),
        "smooth" => app.open_tool(Tool::Smooth),
        "decimate" => app.open_tool(Tool::Decimate),
        "fill_holes" => app.open_tool(Tool::FillHoles),
        "make_solid" => app.open_tool(Tool::MakeSolid),
        "delete_mesh" => app.discard_mesh(),
        "export_ply_cloud" => app.open_tool(Tool::Export(ExportFormat::PlyCloud)),
        "export_ply_mesh" => app.open_tool(Tool::Export(ExportFormat::PlyMesh)),
        "export_obj" => app.open_tool(Tool::Export(ExportFormat::ObjMesh)),
        "export_stl" => app.open_tool(Tool::Export(ExportFormat::StlMesh)),
        _ => {}
    }
}
