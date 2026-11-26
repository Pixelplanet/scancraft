//! The floating browser card (left) and the properties card (right).

use eframe::egui::{self, Margin, Rect, pos2, vec2};
use scancraft_driver::{AccuracyPreset, DeviceParam, LightSource, TurntableMode};
use scancraft_ui::kit::{self, Button, Mark, Row, Surface};
use scancraft_ui::Tokens;

use crate::alignment::{self, EdgeKind};
use crate::app::{Capture, Proposal, ScanCraftApp, Tone};
use crate::project::{AlignState, PassStart};
use crate::scan_modes::{self, OptionState};
use crate::file_picker::PickPurpose;
use crate::model::{ExportFormat, Node, Quality, Stage, Tool, thousands};

/// Status-dot colour of a pass's alignment.
fn align_colour(state: AlignState, t: &Tokens) -> egui::Color32 {
    match state {
        AlignState::Reference => t.accent,
        AlignState::Aligned { .. } => t.agent_ok,
        AlignState::Unaligned => t.warning,
        AlignState::Locating => t.agent_busy,
    }
}

fn verdict_colour(v: scancraft_align::Verdict, t: &Tokens) -> egui::Color32 {
    match v {
        scancraft_align::Verdict::Good => t.agent_ok,
        scancraft_align::Verdict::Fair => t.warning,
        scancraft_align::Verdict::Poor => t.error,
    }
}

/// Quality read-outs of an alignment, under `verdict` (the quality's own, or one the caller
/// checked further). Wall thickness and piercing show where an inside meets an outside scan.
fn quality_rows(ui: &mut egui::Ui, q: &scancraft_align::Quality, verdict: scancraft_align::Verdict) {
    let t = Tokens::get();
    kit::row(ui, "Verdict", |ui| kit::status_line(ui, verdict.label(), verdict_colour(verdict, &t)));
    kit::row(ui, "Overlap", |ui| kit::value_text(ui, &format!("{:.0} % of the pass", q.overlap * 100.0), false));
    let rms = if q.rmse_mm.is_finite() { format!("{:.2} mm", q.rmse_mm) } else { "—".to_string() };
    kit::row(ui, "RMS error", |ui| kit::value_text(ui, &rms, false));
    if let Some(s) = q.shell.filter(|s| s.significant()) {
        let wall = if s.is_wall() { format!("{:.2} mm (spread {:.2})", s.thickness_mm(), s.iqr_mm) } else { format!("{:.1} mm, uneven (spread {:.1})", s.thickness_mm(), s.iqr_mm) };
        kit::row(ui, "Wall", |ui| kit::value_text(ui, &wall, !s.is_wall()));
        kit::row(ui, "Piercing", |ui| kit::value_text(ui, &format!("{:.1} % pokes through", s.piercing * 100.0), false));
    }
    kit::row(ui, "See-through", |ui| kit::value_text(ui, &format!("{:.1} %", q.visibility_conflict * 100.0), false));
    if q.below_support > 0.005 {
        kit::row(ui, "Under table", |ui| kit::value_text(ui, &format!("{:.1} % of the object", q.below_support * 100.0), false));
    }
    if q.sliding() {
        kit::status_line(ui, &format!("May slide: the surfaces do not pin the fit down in one direction (about {:.0} mm of play)", q.slide_mm.min(999.0)), t.warning);
    }
}

/// One line on a fit: verdict, then the measure that matters (wall or RMS).
fn quality_summary(q: &scancraft_align::Quality) -> String {
    let mut parts = vec![q.verdict().label().to_string()];
    if let Some(w) = q.wall() {
        parts.push(format!("wall {:.2} mm", w.thickness_mm()));
    }
    if q.inliers >= 100 && q.rmse_mm.is_finite() {
        parts.push(format!("RMS {:.2} mm", q.rmse_mm));
    }
    if q.sliding() {
        parts.push("may slide".into());
    }
    parts.join(" · ")
}

pub const BROWSER_WIDTH: f32 = 248.0;
pub const PROPERTIES_WIDTH: f32 = 280.0;

fn dash(v: Option<String>) -> String {
    v.unwrap_or_else(|| "—".to_string())
}

/// The browser: session tree of scanner, passes and results (`browser::browser`).
pub fn browser(app: &mut ScanCraftApp, ctx: &egui::Context, bounds: Rect) -> Rect {
    let t = Tokens::get();
    let frame = kit::frame(Surface::Panel).inner_margin(kit::CARD_INSET as i8);
    let inner_w = BROWSER_WIDTH - 2.0 * (kit::CARD_INSET + 1.0);
    let max_h = (bounds.height() - 2.0 * (kit::CARD_INSET + 1.0)).max(60.0);
    let resp = egui::Area::new(egui::Id::new("browser_card")).fixed_pos(bounds.min).order(egui::Order::Middle).show(ctx, |ui| {
        frame.show(ui, |ui| {
            ui.set_width(inner_w);
            ui.set_max_height(max_h);
            ui.spacing_mut().item_spacing.y = 0.0;
            let count = format!("{} points", thousands(app.total_points()));
            kit::browser_header(ui, "Browser", &count);
            ui.add_space(t.space_3);
            egui::ScrollArea::vertical().auto_shrink([false, true]).max_height(max_h - kit::HEADER_H - t.space_3).show(ui, |ui| {
                ui.spacing_mut().item_spacing.y = 0.0;
                tree(app, ui);
            });
        });
    });
    resp.response.rect
}

fn tree(app: &mut ScanCraftApp, ui: &mut egui::Ui) {
    let t = Tokens::get();
    let sel = app.selection;
    let branch = |inside: bool| if inside { Mark::Branch } else { Mark::None };

    // Scanner.
    let (device_label, device_status, device_tip) = match &app.connection {
        Some(c) => (c.info.model.clone(), Some(t.agent_ok), format!("{} · {}", c.info.active_transport.description(), c.info.serial_number)),
        None if app.connecting => ("Connecting…".to_string(), Some(t.agent_busy), "Connecting".to_string()),
        None => ("No scanner".to_string(), None, "Not connected".to_string()),
    };
    let live = match app.capture {
        Capture::Running => "recording",
        Capture::Paused => "paused",
        Capture::Starting => "starting",
        Capture::Stopping => "stopping",
        Capture::Idle if app.previewing => "preview",
        Capture::Idle => "",
    };
    let r = kit::tree_row(
        ui,
        egui::Id::new("row_device"),
        &Row {
            depth: 0,
            icon: "camera",
            label: &device_label,
            selected: sel == Some(Node::Device),
            dim: app.connection.is_none(),
            status: device_status,
            count: live,
            tip: Some(&device_tip),
            mark: if sel == Some(Node::Device) { Mark::Here } else { Mark::None },
            ..Default::default()
        },
    );
    if r.clicked {
        app.selection = Some(Node::Device);
        if app.connection.is_none() {
            app.stage = Stage::Connect;
        }
    }

    // Scan passes.
    let open = app.folded("passes", true);
    let pass_count = app.passes.len().to_string();
    let in_passes = matches!(sel, Some(Node::Pass(_) | Node::Frame(..)));
    let r = kit::tree_row(
        ui,
        egui::Id::new("row_passes"),
        &Row { depth: 0, fold: Some(open), eye: Some(app.passes.iter().any(|p| p.layer.visible)), icon: "folder", label: "Scan passes", count: &pass_count, mark: branch(in_passes), dim: app.passes.is_empty(), ..Default::default() },
    );
    if r.fold || r.double {
        app.folds.insert("passes".into(), !open);
    }
    if r.eye {
        let any = app.passes.iter().any(|p| p.layer.visible);
        for p in &mut app.passes {
            p.layer.visible = !any;
        }
    }
    if open {
        if app.passes.is_empty() {
            empty_row(ui, "No passes yet");
        }
        let limits = scancraft_slam::TrackerConfig::default();
        let can_frames = app.can_edit_frames();
        let can_continue = app.connection.is_some() && app.capture == Capture::Idle && app.busy.is_none();
        let mut actions: Vec<BrowserAction> = Vec::new();
        for p in &app.passes {
            let imported = p.meta.origin.is_imported();
            // Imported passes say so where the point count would be (the name needs the room).
            let count = if imported { "imported".to_string() } else { thousands(p.layer.len()) };
            let recording = app.active_pass == Some(p.id) && app.capture == Capture::Running;
            let pass_key = format!("pass:{}", p.id);
            // A fixed surface has no frames to list.
            let pass_open = !p.is_fixed() && app.folded(&pass_key, true);
            let left_out = format!("{}{}", if p.is_cleaned() { " · cleaned" } else { "" }, if p.meta.included { "" } else { " · left out of Merge" });
            let tip = if p.is_fixed() {
                format!("{} · {}{left_out}\n{} points · a fixed surface: no frames, not continued", p.meta.alignment.label(), p.meta.origin.label(), thousands(p.layer.len()))
            } else {
                format!(
                    "{} · {}{left_out}{}\n{} frames captured · {} keyframes stored",
                    p.meta.alignment.label(),
                    p.meta.scan_config.light.label(),
                    if imported { format!("\n{}", p.meta.origin.label()) } else { String::new() },
                    p.frames,
                    p.keyframes.len()
                )
            };
            let frame_selected = matches!(sel, Some(Node::Frame(id, _)) if id == p.id);
            let r = kit::tree_row(
                ui,
                egui::Id::new(("row_pass", p.id)),
                &Row {
                    depth: 1,
                    fold: (!p.is_fixed()).then_some(pass_open),
                    eye: Some(p.layer.visible),
                    icon: if imported { "import" } else { "capture" },
                    label: &p.name,
                    count: &count,
                    selected: sel == Some(Node::Pass(p.id)),
                    dim: !p.layer.visible,
                    status: Some(if recording { t.error } else { align_colour(p.meta.alignment, &t) }),
                    mark: if sel == Some(Node::Pass(p.id)) { Mark::Here } else { branch(frame_selected) },
                    tip: Some(&tip),
                },
            );
            if r.clicked {
                actions.push(BrowserAction::Select(Node::Pass(p.id)));
            }
            if r.eye {
                actions.push(BrowserAction::TogglePass(p.id));
            }
            if r.fold || r.double {
                actions.push(BrowserAction::Fold(pass_key.clone(), !pass_open));
            }
            if let Some(resp) = r.response {
                resp.context_menu(|ui| {
                    let continue_why = if p.is_fixed() {
                        "An imported pass cannot be continued"
                    } else if app.connection.is_none() {
                        "Connect a scanner first"
                    } else if app.capture != Capture::Idle {
                        "Stop the capture first"
                    } else if app.busy.is_some() {
                        "Another operation is running"
                    } else {
                        "A cleaned pass cannot be continued: reset its cleaning first"
                    };
                    if kit::menu_item(ui, "Continue Pass", Button::Normal, can_continue && !p.is_fixed() && !p.is_cleaned(), continue_why) {
                        actions.push(BrowserAction::Continue(p.id));
                    }
                    let reset_why = if !p.is_cleaned() { "Not cleaned" } else { app.edit_blocked().unwrap_or("") };
                    if kit::menu_item(ui, "Reset Cleaning", Button::Normal, app.can_edit() && p.is_cleaned(), reset_why) {
                        actions.push(BrowserAction::ResetCleaning(p.id));
                    }
                    ui.separator();
                    let align_why = app
                        .align_blocked(Some(p.id))
                        .or(app.align_preview.is_some().then_some("Accept or reject the proposed alignment first"))
                        .unwrap_or("");
                    let align_ok = align_why.is_empty();
                    if kit::menu_item(ui, "Align Pass", Button::Normal, align_ok, align_why) {
                        actions.push(BrowserAction::AlignAuto(p.id));
                    }
                    if kit::menu_item(ui, "Point Pairs…", Button::Normal, align_ok, align_why) {
                        actions.push(BrowserAction::AlignPairs(p.id));
                    }
                    let aligned = matches!(p.meta.alignment, AlignState::Aligned { .. });
                    let ref_why = if !aligned { "Only an aligned pass can define the project frame" } else { app.edit_blocked().unwrap_or("") };
                    let reset_why = if !aligned { "Not aligned" } else { app.edit_blocked().unwrap_or("") };
                    if kit::menu_item(ui, "Make Reference", Button::Normal, app.can_edit() && aligned, ref_why) {
                        actions.push(BrowserAction::Reference(p.id));
                    }
                    if kit::menu_item(ui, "Reset Alignment", Button::Normal, app.can_edit() && aligned, reset_why) {
                        actions.push(BrowserAction::ResetAlign(p.id));
                    }
                    let include = if p.meta.included { "Leave Out of Merge" } else { "Include in Merge" };
                    if kit::menu_item(ui, include, Button::Normal, true, "") {
                        actions.push(BrowserAction::Include(p.id, !p.meta.included));
                    }
                    ui.separator();
                    let delete_why = if recording { "Stop the capture first" } else { app.edit_blocked().unwrap_or("") };
                    if kit::menu_item(ui, "Delete Pass", Button::Danger, app.can_edit() && !recording, delete_why) {
                        actions.push(BrowserAction::DeletePass(p.id));
                    }
                });
            }
            if !pass_open {
                continue;
            }
            // Its stored keyframes.
            let frames_key = format!("frames:{}", p.id);
            let frames_open = app.folded(&frames_key, false);
            let kept = p.kept_frames();
            let frames_label = format!("Frames ({})", p.keyframes.len());
            let frames_count = if kept == p.keyframes.len() { String::new() } else { format!("{kept} kept") };
            let r = kit::tree_row(
                ui,
                egui::Id::new(("row_frames", p.id)),
                &Row {
                    depth: 2,
                    fold: Some(frames_open),
                    icon: "list",
                    label: &frames_label,
                    count: &frames_count,
                    dim: p.keyframes.is_empty(),
                    mark: branch(frame_selected),
                    tip: Some("Frames the tracker stored as it moved. Select one to see where it landed; discard frames that landed wrong."),
                    ..Default::default()
                },
            );
            if r.fold || r.double || r.clicked {
                actions.push(BrowserAction::Fold(frames_key.clone(), !frames_open));
            }
            if !frames_open {
                continue;
            }
            if p.keyframes.is_empty() {
                empty_row_at(ui, 3, "No keyframes stored yet");
            }
            for k in &p.keyframes {
                let node = Node::Frame(p.id, k.id);
                let quality = k.quality(limits.max_rmse_mm, limits.max_visibility_conflict);
                let dot = if k.discarded {
                    t.text_dim.gamma_multiply(0.5)
                } else {
                    match quality {
                        Quality::Good => t.agent_ok,
                        Quality::Fair => t.warning,
                        Quality::Poor => t.error,
                    }
                };
                let label = format!("Frame {}", k.id);
                let count = if k.discarded { "discarded".to_string() } else { format!("+{}", thousands(k.new_points)) };
                let tip = format!(
                    "{} · RMS {:.2} mm · {:.0} % seen through known surface\n{} points, {} new when fused",
                    quality.label(),
                    k.rmse_mm,
                    k.visibility_conflict * 100.0,
                    thousands(k.points),
                    thousands(k.new_points)
                );
                let r = kit::tree_row(
                    ui,
                    egui::Id::new(("row_frame", p.id, k.id)),
                    &Row {
                        depth: 3,
                        icon: "point",
                        label: &label,
                        count: &count,
                        selected: sel == Some(node),
                        dim: k.discarded,
                        status: Some(dot),
                        mark: if sel == Some(node) { Mark::Here } else { Mark::None },
                        tip: Some(&tip),
                        ..Default::default()
                    },
                );
                if r.clicked {
                    actions.push(BrowserAction::Select(node));
                }
                if r.double && can_frames {
                    actions.push(BrowserAction::Discard(p.id, k.id, !k.discarded));
                }
                if let Some(resp) = r.response {
                    resp.context_menu(|ui| {
                        let frames_why = if app.busy.is_some() { "Another operation is running" } else { "Pause or stop the capture first" };
                        let label = if k.discarded { "Restore Frame" } else { "Discard Frame" };
                        if kit::menu_item(ui, label, Button::Normal, can_frames, if can_frames { "" } else { frames_why }) {
                            actions.push(BrowserAction::Discard(p.id, k.id, !k.discarded));
                        }
                        if kit::menu_item(ui, "Discard This and Later Frames", Button::Normal, can_frames, if can_frames { "" } else { frames_why }) {
                            actions.push(BrowserAction::DiscardFrom(p.id, k.id));
                        }
                    });
                }
            }
        }
        for action in actions {
            match action {
                BrowserAction::Select(node) => app.selection = Some(node),
                BrowserAction::TogglePass(id) => {
                    if let Some(p) = app.passes.iter_mut().find(|p| p.id == id) {
                        p.layer.visible = !p.layer.visible;
                    }
                }
                BrowserAction::Fold(key, open) => {
                    app.folds.insert(key, open);
                }
                BrowserAction::ResetCleaning(id) => app.reset_cleaning_of(&[id]),
                BrowserAction::Continue(id) => {
                    app.stage = Stage::Scan;
                    app.continue_scan(id);
                }
                BrowserAction::DeletePass(id) => app.delete_pass(id),
                BrowserAction::Discard(pass, frame, discarded) => {
                    app.selection = Some(Node::Frame(pass, frame));
                    app.set_frame_discarded(pass, frame, discarded);
                }
                BrowserAction::DiscardFrom(pass, frame) => {
                    app.selection = Some(Node::Frame(pass, frame));
                    app.discard_frames_from(pass, frame);
                }
                BrowserAction::AlignAuto(id) => {
                    app.stage = Stage::Align;
                    app.selection = Some(Node::Pass(id));
                    app.align_auto(id);
                }
                BrowserAction::AlignPairs(id) => {
                    app.selection = Some(Node::Pass(id));
                    app.open_tool(Tool::PointPairs(id));
                }
                BrowserAction::Reference(id) => app.set_reference(id),
                BrowserAction::ResetAlign(id) => app.reset_alignment(id),
                BrowserAction::Include(id, on) => app.set_included(id, on),
            }
        }
    }

    // Results.
    let open = app.folded("results", true);
    let in_results = matches!(sel, Some(Node::Merged | Node::Mesh));
    let n = app.merged.is_some() as usize + app.mesh.is_some() as usize;
    let n_label = n.to_string();
    let r = kit::tree_row(ui, egui::Id::new("row_results"), &Row { depth: 0, fold: Some(open), icon: "folder", label: "Results", count: &n_label, mark: branch(in_results), dim: n == 0, ..Default::default() });
    if r.fold || r.double {
        app.folds.insert("results".into(), !open);
    }
    if open {
        if n == 0 {
            empty_row(ui, "Merge or mesh to add results");
        }
        if let Some(m) = &app.merged {
            let count = thousands(m.layer.len());
            let tip = format!("{}\nFrom {}", m.summary, m.passes.iter().map(|id| format!("Pass {id}")).collect::<Vec<_>>().join(", "));
            let r = kit::tree_row(
                ui,
                egui::Id::new("row_merged"),
                &Row {
                    depth: 1,
                    eye: Some(m.layer.visible),
                    icon: "combine",
                    label: "Merged model",
                    count: &count,
                    selected: sel == Some(Node::Merged),
                    dim: !m.layer.visible,
                    mark: if sel == Some(Node::Merged) { Mark::Here } else { Mark::None },
                    tip: Some(&tip),
                    ..Default::default()
                },
            );
            if r.clicked {
                app.selection = Some(Node::Merged);
            }
            if r.eye {
                if let Some(m) = &mut app.merged {
                    m.layer.visible = !m.layer.visible;
                }
            }
            let mut discard = false;
            if let Some(resp) = r.response {
                resp.context_menu(|ui| {
                    if kit::menu_item(ui, "Delete Merged Model", Button::Danger, app.can_edit(), app.edit_blocked().unwrap_or("")) {
                        discard = true;
                    }
                });
            }
            if discard {
                app.discard_merged();
            }
        }
        if let Some(m) = &app.mesh {
            let count = format!("{} tris", thousands(m.analysis.triangle_count));
            let r = kit::tree_row(
                ui,
                egui::Id::new("row_mesh"),
                &Row {
                    depth: 1,
                    eye: Some(m.visible),
                    icon: "mesh_body",
                    label: "Mesh",
                    count: &count,
                    selected: sel == Some(Node::Mesh),
                    dim: !m.visible,
                    mark: if sel == Some(Node::Mesh) { Mark::Here } else { Mark::None },
                    ..Default::default()
                },
            );
            if r.clicked {
                app.selection = Some(Node::Mesh);
            }
            if r.eye {
                if let Some(m) = &mut app.mesh {
                    m.visible = !m.visible;
                }
            }
            let mut discard = false;
            if let Some(resp) = r.response {
                resp.context_menu(|ui| {
                    if kit::menu_item(ui, "Delete Mesh", Button::Danger, app.can_edit(), app.edit_blocked().unwrap_or("")) {
                        discard = true;
                    }
                });
            }
            if discard {
                app.discard_mesh();
            }
        }
    }
}

/// What a click in the browser asked for, applied after the rows are drawn.
enum BrowserAction {
    Select(Node),
    TogglePass(u32),
    Fold(String, bool),
    Continue(u32),
    ResetCleaning(u32),
    DeletePass(u32),
    Discard(u32, u64, bool),
    DiscardFrom(u32, u64),
    AlignAuto(u32),
    AlignPairs(u32),
    Reference(u32),
    ResetAlign(u32),
    Include(u32, bool),
}

fn empty_row(ui: &mut egui::Ui, text: &str) {
    empty_row_at(ui, 1, text);
}

/// A dim placeholder line where rows at `depth` would start.
fn empty_row_at(ui: &mut egui::Ui, depth: usize, text: &str) {
    let t = Tokens::get();
    let (rect, _) = ui.allocate_exact_size(vec2(ui.available_width(), kit::TREE_ROW_HEIGHT), egui::Sense::hover());
    let x = rect.left() + 2.0 + depth as f32 * 14.0 + 14.0 + 18.0;
    ui.painter().text(pos2(x, rect.center().y), egui::Align2::LEFT_CENTER, text, egui::FontId::proportional(t.font_body), t.text_dim);
}

/// The properties card: the open tool's parameters, else the stage's settings, then the
/// selection's facts.
pub fn properties(app: &mut ScanCraftApp, ctx: &egui::Context, bounds: Rect) -> Rect {
    let t = Tokens::get();
    let x = bounds.right() - PROPERTIES_WIDTH;
    let max_h = bounds.height().max(80.0);
    let tool = app.tool;
    let frame = match (tool, app.selection) {
        (None, Some(Node::Frame(pass, frame))) => Some((pass, frame)),
        _ => None,
    };
    let (icon, title) = match (tool, frame) {
        (Some(tool), _) => (tool.icon(), tool.title().to_string()),
        (None, Some((_, id))) => ("point", format!("Frame {id}")),
        (None, None) => match app.stage {
            Stage::Connect => ("drive", "Scanner".to_string()),
            Stage::Scan => ("capture", "Scan Settings".to_string()),
            Stage::Align => ("align", "Alignment".to_string()),
            Stage::Clean => ("filter", "Cleaning".to_string()),
            Stage::Mesh => ("mesh_body", "Mesh".to_string()),
            Stage::Export => ("export", "Export".to_string()),
        },
    };
    let resp = egui::Area::new(egui::Id::new("properties_card")).fixed_pos(pos2(x, bounds.top())).order(egui::Order::Middle).show(ctx, |ui| {
        kit::frame(Surface::Feature).show(ui, |ui| {
            ui.set_width(PROPERTIES_WIDTH - 2.0);
            ui.set_max_height(max_h);
            ui.spacing_mut().item_spacing.y = 0.0;
            if kit::dialog_header(ui, icon, &title, tool.is_some() || frame.is_some()) {
                if tool.is_some() {
                    app.close_tool();
                } else if let Some((pass, _)) = frame {
                    app.selection = Some(Node::Pass(pass));
                }
            }
            // A pending alignment decision gets the footer too (the Accept/Reject rule).
            let decision = tool.is_none() && app.stage == Stage::Align && (app.poses_preview.is_some() || app.align_preview.is_some() || app.orient_preview.is_some());
            let footer_h = if tool.is_some() || decision { t.form_footer_height } else { 0.0 };
            egui::ScrollArea::vertical().auto_shrink([false, true]).max_height((max_h - t.form_header_height - footer_h - 2.0).max(40.0)).show(ui, |ui| {
                egui::Frame::NONE.inner_margin(Margin { left: 12, right: 12, top: 8, bottom: 10 }).show(ui, |ui| {
                    kit::scope(ui, |ui| match (tool, frame) {
                        (Some(tool), _) => tool_body(app, ui, tool),
                        (None, Some((pass, id))) => frame_body(app, ui, pass, id),
                        (None, None) => stage_body(app, ui),
                    });
                });
            });
            if tool.is_some() || decision {
                let pairs_err = if matches!(tool, Some(Tool::PointPairs(_))) && app.picked_pairs().len() >= 3 {
                    scancraft_align::fit_pairs(&app.picked_pairs()).err().map(|e| e.to_string())
                } else {
                    None
                };
                let datum_why = if tool == Some(Tool::Datums) { app.datum_apply_blocked() } else { None };
                let actions = if let Some(tool) = tool {
                    let blocked = app.tool_blocked(tool);
                    let bad_ip = tool == Tool::ConnectIp && !app.ip_valid();
                    let few_pairs = matches!(tool, Tool::PointPairs(_)) && app.picked_pairs().len() < 3;
                    let scale_why = if matches!(tool, Tool::Scale(_)) { app.scale_apply_blocked() } else { None };
                    let no_ai_object = tool == Tool::AiSegment && app.ai_picking.as_ref().is_none_or(|a| a.kept_count() == 0);
                    let label = match tool {
                        Tool::ConnectIp => "Connect",
                        Tool::Export(_) => "Export",
                        Tool::PointPairs(_) => "Align",
                        Tool::SaveProject => "Save",
                        Tool::AiSegment => "Keep object",
                        _ => "Apply",
                    };
                    kit::Actions {
                        primary: Some((label, blocked.is_none() && !bad_ip && !few_pairs && !no_ai_object && pairs_err.is_none() && datum_why.is_none() && scale_why.is_none())),
                        primary_tip: blocked
                            .or(bad_ip.then_some("Enter a valid address"))
                            .or(few_pairs.then_some("Pick at least 3 pairs"))
                            .or(pairs_err.as_deref())
                            .or(datum_why.as_deref())
                            .or(scale_why)
                            .or(no_ai_object.then_some("Click a keyframe and run Propagate all first")),
                        cancel: Some("Cancel"),
                        ..Default::default()
                    }
                } else {
                    kit::Actions { primary: Some(("Accept", app.can_edit())), primary_tip: app.edit_blocked(), cancel: Some("Reject"), ..Default::default() }
                };
                let rule = ui.cursor().top();
                t.secondary_hline(ui.painter(), ui.max_rect().x_range(), rule, t.dialog_rule);
                let r = egui::Frame::NONE.fill(t.dialog_footer).corner_radius(egui::CornerRadius { nw: 0, ne: 0, sw: t.radius_xl, se: t.radius_xl }).show(ui, |ui| kit::dialog_footer(ui, &actions)).inner;
                match r.choice {
                    Some(kit::Choice::Cancel) => {
                        if tool.is_some() {
                            app.close_tool();
                        } else if app.poses_preview.is_some() {
                            app.reject_poses();
                        } else if app.orient_preview.is_some() {
                            app.reject_orient();
                        } else {
                            app.discard_preview();
                        }
                    }
                    Some(kit::Choice::Primary) => {
                        if let Some(tool) = tool {
                            app.run_tool(tool);
                        } else if app.poses_preview.is_some() {
                            app.accept_poses();
                        } else if app.orient_preview.is_some() {
                            app.accept_orient();
                        } else {
                            app.accept_preview();
                        }
                    }
                    _ => {}
                }
            }
        });
    });
    resp.response.rect
}

/// A selected keyframe: how well it fitted, and discard / restore.
fn frame_body(app: &mut ScanCraftApp, ui: &mut egui::Ui, pass: u32, id: u64) {
    let t = Tokens::get();
    let Some(p) = app.passes.iter().find(|p| p.id == pass) else { return };
    let Some(k) = p.keyframes.iter().find(|k| k.id == id).cloned() else {
        kit::note(ui, "This frame is no longer stored.");
        return;
    };
    let pass_name = p.name.clone();
    let later = p.keyframes.iter().filter(|f| f.id > id && !f.discarded).count();
    let limits = scancraft_slam::TrackerConfig::default();
    let quality = k.quality(limits.max_rmse_mm, limits.max_visibility_conflict);
    let colour = match quality {
        Quality::Good => t.agent_ok,
        Quality::Fair => t.warning,
        Quality::Poor => t.error,
    };
    kit::row(ui, "Pass", |ui| kit::value_text(ui, &pass_name, false));
    kit::row(ui, "State", |ui| kit::value_text(ui, if k.discarded { "Discarded" } else { "Kept" }, k.discarded));
    kit::row(ui, "Fit", |ui| {
        kit::status_line(ui, quality.label(), if k.discarded { t.text_dim } else { colour });
    });
    kit::row(ui, "RMS error", |ui| kit::value_text(ui, &format!("{:.2} mm (limit {:.1})", k.rmse_mm, limits.max_rmse_mm), false));
    kit::row(ui, "See-through", |ui| kit::value_text(ui, &format!("{:.1} % (limit {:.0} %)", k.visibility_conflict * 100.0, limits.max_visibility_conflict * 100.0), false));
    kit::row(ui, "Points", |ui| kit::value_text(ui, &thousands(k.points), false));
    kit::row(ui, "New points", |ui| kit::value_text(ui, &thousands(k.new_points), false));
    ui.add_space(t.space_6);
    kit::note_with_tip(ui, "Its points are highlighted in the view.", "Discard a frame that landed in the wrong place; the pass is rebuilt from the frames that are kept.");
    ui.add_space(t.space_8);
    let can = app.can_edit_frames();
    let why = if can { "" } else if app.busy.is_some() { "Another operation is running" } else { "Pause or stop the capture first" };
    let main = if k.discarded { "Restore Frame" } else { "Discard Frame" };
    if kit::wide_button(ui, main, if k.discarded { Button::Primary } else { Button::Normal }, can).on_hover_text(if can { "" } else { why }).clicked() {
        app.set_frame_discarded(pass, id, !k.discarded);
    }
    ui.add_space(t.space_4);
    let from = if later > 0 { format!("Discard This and {later} Later Frame{}", if later == 1 { "" } else { "s" }) } else { "Discard This and All Later Frames".to_string() };
    if kit::wide_button(ui, &from, Button::Normal, can && (!k.discarded || later > 0)).on_hover_text(if can { "" } else { why }).clicked() {
        app.discard_frames_from(pass, id);
    }
    if !can {
        ui.add_space(t.space_4);
        kit::note(ui, if app.busy.is_some() { "Another operation is running." } else { "Pause or stop the capture first." });
    }
}

fn tool_body(app: &mut ScanCraftApp, ui: &mut egui::Ui, tool: Tool) {
    let t = Tokens::get();
    let input = thousands(app.working_len());
    let source = app.working_source();
    if let Some(reason) = app.tool_blocked(tool) {
        kit::status_line(ui, reason, t.warning);
        ui.add_space(t.space_6);
    }
    let s = &mut app.settings;
    match tool {
        Tool::ConnectIp => {
            kit::row(ui, "Address", |ui| {
                ui.add(egui::TextEdit::singleline(&mut s.ip).hint_text("192.168.1.20").desired_width(f32::INFINITY));
            });
            kit::note(ui, "The scanner joins the network from its own Wi-Fi settings; its address is shown by your router.");
        }
        Tool::RemovePlane => {
            kit::row(ui, "Input", |ui| kit::value_field(ui, &format!("{source} · {input} points"), true));
            kit::slider_row(ui, "Tolerance", &mut s.plane_tolerance_mm, 0.5..=6.0, 1, "mm");
            kit::note(ui, "Finds the largest flat surface (a turntable or table) and removes it and everything below it.");
        }
        Tool::Outliers => {
            kit::row(ui, "Input", |ui| kit::value_field(ui, &format!("{source} · {input} points"), true));
            kit::slider_row(ui, "Neighbours", &mut s.sor_neighbors, 4..=48, 0, "");
            kit::slider_row(ui, "Threshold", &mut s.sor_std_dev, 0.5..=3.0, 2, "σ");
            kit::note(ui, "Removes points whose mean distance to their neighbours is far above the average.");
        }
        Tool::Downsample => {
            kit::row(ui, "Input", |ui| kit::value_field(ui, &format!("{source} · {input} points"), true));
            kit::slider_row(ui, "Voxel size", &mut s.voxel_mm, 0.2..=5.0, 1, "mm");
        }
        Tool::Crop => {
            kit::row(ui, "Input", |ui| kit::value_field(ui, &format!("{source} · {input} points"), true));
            kit::section_heading(ui, "Minimum");
            for (i, axis) in ["X", "Y", "Z"].iter().enumerate() {
                kit::row(ui, axis, |ui| {
                    ui.add(egui::DragValue::new(&mut s.crop_min[i]).speed(1.0).suffix(" mm").range(-2000.0..=2000.0));
                });
            }
            kit::section_heading(ui, "Maximum");
            for (i, axis) in ["X", "Y", "Z"].iter().enumerate() {
                kit::row(ui, axis, |ui| {
                    ui.add(egui::DragValue::new(&mut s.crop_max[i]).speed(1.0).suffix(" mm").range(-2000.0..=2000.0));
                });
            }
            kit::note(ui, "Starts at the bounds of the visible geometry. Scanner coordinates: +Y points away from the scanner, +Z up.");
        }
        Tool::Reconstruct => {
            kit::row(ui, "Input", |ui| kit::value_field(ui, &format!("{source} · {input} points"), true));
            kit::slider_row(ui, "Triangle size", &mut s.mc_cell_mm, 0.3..=3.0, 1, "mm").on_hover_text("Smaller keeps finer detail, takes longer and makes a bigger file.");
            kit::slider_row(ui, "Smoothness", &mut s.mesh_smoothness, 2.0..=6.0, 1, "").on_hover_text(
                "How far the surface averages over neighbouring points, in point spacings. More hides noise and small misalignment between passes; less keeps fine detail.",
            );
            kit::slider_row(ui, "Then smooth", &mut s.mesh_auto_smooth, 0..=10, 0, "").on_hover_text("Edge-preserving smoothing steps right after reconstruction (0 = off).");
            kit::slider_row(ui, "Keep edges", &mut s.smooth_keep_edges, 0.0..=1.0, 2, "").on_hover_text("High keeps edges and ribs sharp; low rounds everything like a plain smoothing filter.");
            kit::note(ui, "Open areas stay open; Fill Holes closes them.");
        }
        Tool::Smooth => {
            kit::slider_row(ui, "Strength", &mut s.smooth_iterations, 1..=15, 0, "").on_hover_text("Number of smoothing steps.");
            kit::slider_row(ui, "Keep edges", &mut s.smooth_keep_edges, 0.0..=1.0, 2, "").on_hover_text("High keeps edges and ribs sharp; low rounds everything like a plain smoothing filter.");
            kit::note(ui, "Folded spots from bad scan data are relaxed first.");
        }
        Tool::FillHoles => fill_holes_form(app, ui),
        Tool::MakeSolid => {
            // The placed passes go in as separate scans; without any, the working cloud.
            let placed: Vec<(String, f32)> = app.passes.iter().filter(|p| p.meta.included && p.meta.alignment.is_placed()).map(|p| (p.name.clone(), p.meta.weight)).collect();
            let input = if placed.is_empty() {
                format!("{} · {} points", app.working_source(), thousands(app.working_len()))
            } else {
                format!("{} placed pass{}", placed.len(), if placed.len() == 1 { "" } else { "es" })
            };
            let weights_differ = placed.iter().any(|p| (p.1 - placed[0].1).abs() > 1e-6);
            let top = placed.iter().max_by(|a, b| a.1.total_cmp(&b.1)).map(|p| p.0.clone());
            let markers: usize = app.passes.iter().filter(|p| p.meta.included && p.meta.alignment.is_placed()).map(|p| p.markers.len()).sum();
            let s = &mut app.settings;
            kit::row(ui, "Input", |ui| kit::value_field(ui, &input, true));
            if !placed.is_empty() {
                let order = if weights_differ { "the higher merge weight wins where passes overlap" } else { "equal merge weights: overlapping passes are averaged" };
                kit::note_with_tip(ui, &format!("Each pass is a scan of its own: {order}."), "Set a pass's merge weight in its Selected Pass panel. The solid keeps one layer per surface, from the most trusted pass covering it.");
            }
            kit::slider_row(ui, "Cell size", &mut s.solid_cell_mm, 0.25..=1.5, 2, "mm").on_hover_text("The wall should be at least 6 cells thick. Smaller cells keep finer detail and take longer.");
            let marker_label = if markers > 0 { format!("Remove marker stickers ({markers} from tracking + detected)") } else { "Find and remove marker stickers".to_string() };
            kit::checkbox(ui, &marker_label, &mut s.solid_remove_markers);
            if weights_differ {
                if let Some(top) = top {
                    kit::checkbox(ui, &format!("Sharp edges from {top}"), &mut s.solid_sharp_top).on_hover_text("Treat the most trusted pass as clean and dense: a narrower kernel keeps its edges sharp. Leave off for noisy scans (thin plates can open slits).");
                }
            }
            kit::note_with_tip(
                ui,
                "Builds a closed, watertight solid of a thin-walled part and replaces the mesh. Takes longer than Reconstruct.",
                "Both faces of the wall, the rim between them, enclosed holes filled and the unscanned side of the wall completed at the measured thickness. For printing or CAD.",
            );
        }
        Tool::Decimate => {
            let tris = app.mesh.as_ref().map(|m| m.analysis.triangle_count);
            let s = &mut app.settings;
            kit::row(ui, "Triangles", |ui| kit::value_field(ui, &dash(tris.map(thousands)), true));
            kit::slider_row(ui, "Keep", &mut s.decimate_ratio, 0.05..=0.95, 2, "");
            if let Some(n) = tris {
                kit::row(ui, "Result", |ui| kit::value_field(ui, &format!("≈ {}", thousands((n as f32 * s.decimate_ratio) as usize)), true));
            }
        }
        Tool::Export(_) => export_form(app, ui),
        Tool::PointPairs(id) => pairs_form(app, ui, id),
        Tool::Datums => crate::ui_orient::datums_form(app, ui),
        Tool::Scale(id) => crate::ui_orient::scale_form(app, ui, id),
        Tool::IsolateObject => isolate_form(app, ui),
        Tool::AiSegment => crate::ui_ai::ai_segment_form(app, ui),
        Tool::SaveProject => {
            kit::row(ui, "Folder", |ui| path_field(app, ui, PickPurpose::SaveFolder));
            let n = app.passes.len();
            kit::row(ui, "Contents", |ui| kit::value_field(ui, &format!("{n} pass{}{}", if n == 1 { "" } else { "es" }, if app.merged.is_some() || app.mesh.is_some() { " + results" } else { "" }), true));
            kit::note(ui, "Saves project.json and each pass's keyframes into the folder (created if needed). Saving again overwrites it.");
        }
    }
}

/// The point-pair card: the picked pairs and how to pick.
fn pairs_form(app: &mut ScanCraftApp, ui: &mut egui::Ui, id: u32) {
    let t = Tokens::get();
    let name = app.passes.iter().find(|p| p.id == id).map(|p| p.name.clone()).unwrap_or_default();
    kit::row(ui, "Moving", |ui| kit::value_text(ui, &name, false));
    kit::row(ui, "Onto", |ui| kit::value_text(ui, "Project model", false));
    ui.add_space(t.space_4);
    kit::note_with_tip(
        ui,
        "Click a spot on the model (left), then the same spot on the pass (right). Three or more pairs, spread over the object.",
        "Picks need only be close: the surfaces are fitted afterwards.",
    );
    ui.add_space(t.space_6);
    let mut remove = None;
    if let Some(p) = &app.picking {
        if p.pairs.is_empty() {
            kit::note(ui, "No pairs yet.");
        }
        for (i, (a, b)) in p.pairs.iter().enumerate() {
            let state = match (a, b) {
                (Some(_), Some(_)) => "model + pass",
                (Some(_), None) => "model · now click the pass",
                (None, Some(_)) => "pass · now click the model",
                (None, None) => "—",
            };
            ui.horizontal(|ui| {
                kit::value_text(ui, &format!("{}  {state}", i + 1), a.is_none() || b.is_none());
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    if kit::close_button(ui, "Remove this pair").clicked() {
                        remove = Some(i);
                    }
                });
            });
        }
    }
    if let (Some(i), Some(p)) = (remove, app.picking.as_mut()) {
        p.pairs.remove(i);
    }
    let complete = app.picked_pairs().len();
    ui.add_space(t.space_4);
    kit::row(ui, "Pairs", |ui| kit::value_text(ui, &format!("{complete} complete (3 needed)"), complete < 3));
    if complete >= 3 {
        match scancraft_align::fit_pairs(&app.picked_pairs()) {
            Ok(fit) => {
                kit::row(ui, "Pick fit", |ui| kit::value_text(ui, &format!("{:.1} mm RMS", fit.rms_mm), false))
                    .on_hover_text("How well the picked pairs agree before the surfaces are fitted. A few mm is normal.");
                if fit.spread.nearly_collinear() {
                    kit::status_line(ui, "The picked points are nearly on a line: add a pair well off that line", t.warning);
                }
            }
            Err(e) => kit::status_line(ui, &e.to_string(), t.error),
        }
    }
}

/// The Isolate Object card: the result so far and the picks.
fn isolate_form(app: &mut ScanCraftApp, ui: &mut egui::Ui) {
    let t = Tokens::get();
    let source = app.working_source();
    let Some(o) = &app.object_picking else { return };
    kit::row(ui, "Input", |ui| kit::value_field(ui, &format!("{source} · {} points", thousands(o.input.len())), true));
    let (object, table) = match &o.result {
        Some((seg, _)) => (
            format!("{} points", thousands(seg.count(scancraft_fusion::SegmentLabel::Object))),
            if seg.support.is_some() { format!("found · {} points", thousands(seg.count(scancraft_fusion::SegmentLabel::Support))) } else { "none found".to_string() },
        ),
        None => ("…".to_string(), "…".to_string()),
    };
    kit::row(ui, "Object", |ui| kit::value_text(ui, &object, !o.current()));
    kit::row(ui, "Table", |ui| kit::value_text(ui, &table, !o.current()));
    ui.add_space(t.space_4);
    kit::note(ui, "Green is kept, grey is the table, red is left out. Click a cluster in the view to keep it; Shift+click one to leave it out.");
    ui.add_space(t.space_6);
    let mut remove = None;
    if o.picks.is_empty() {
        kit::note(ui, "No picks: the largest object is kept (the one standing on the table, if a table was found).");
    }
    for (i, (_, keep)) in o.picks.iter().enumerate() {
        ui.horizontal(|ui| {
            kit::value_text(ui, &format!("{}  {}", i + 1, if *keep { "keep this cluster" } else { "leave this cluster out" }), false);
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                if kit::close_button(ui, "Remove this pick").clicked() {
                    remove = Some(i);
                }
            });
        });
    }
    let can_undo = !o.picks.is_empty();
    if let Some(i) = remove {
        app.remove_object_pick(i);
    }
    ui.add_space(t.space_4);
    if kit::wide_button(ui, "Undo Last Pick", Button::Normal, can_undo).clicked() {
        app.undo_object_pick();
    }
}

/// A path that can be typed, with a Browse button that opens the native dialog.
fn path_field(app: &mut ScanCraftApp, ui: &mut egui::Ui, purpose: PickPurpose) {
    let browsing = app.picker.is_open();
    ui.horizontal(|ui| {
        let button = ui.add_enabled(!browsing, egui::Button::new("Browse…")).on_hover_text("Choose with the Windows file browser");
        let text = match purpose {
            PickPurpose::ExportFile => &mut app.settings.export_path,
            _ => &mut app.settings.project_path,
        };
        ui.add(egui::TextEdit::singleline(text).desired_width((ui.available_width() - button.rect.width() - ui.spacing().item_spacing.x).max(40.0)));
        if button.clicked() {
            app.pick(&ui.ctx().clone(), purpose);
        }
    });
}

fn fill_holes_form(app: &mut ScanCraftApp, ui: &mut egui::Ui) {
    let t = Tokens::get();
    app.refresh_holes();
    let s = &mut app.settings;
    // Slider in boundary length; the holes are listed by it.
    kit::slider_row(ui, "Max. size", &mut s.fill_max_mm, 5.0..=1000.0, 0, "mm").on_hover_text("Largest hole to fill, measured around its rim (boundary length).");
    kit::checkbox(ui, "Keep the largest opening", &mut s.fill_keep_largest).on_hover_text("The largest opening is usually the open border of the scan, not a hole.");
    let Some(h) = &app.holes else { return };
    let (fill, skip) = h.split(app.settings.fill_max_mm, app.settings.fill_keep_largest);
    kit::row(ui, "To fill", |ui| kit::value_text(ui, &format!("{fill} of {} holes", h.holes.len()), false));
    kit::row(ui, "Left open", |ui| kit::value_text(ui, &format!("{skip}"), false));
    if let Some(big) = h.holes.first() {
        kit::row(ui, "Largest", |ui| kit::value_text(ui, &format!("{:.0} mm rim · {:.0} mm across", big.perimeter_mm, big.diameter_mm), false));
    }
    ui.add_space(t.space_4);
    kit::note_with_tip(
        ui,
        "Holes to fill are outlined in the view; those left open are grey.",
        "Each hole is closed with triangles of the surrounding size and shaped to continue the surface around it.",
    );
}

fn export_form(app: &mut ScanCraftApp, ui: &mut egui::Ui) {
    let labels: Vec<&str> = ExportFormat::ALL.iter().map(|f| f.label()).collect();
    let enabled: Vec<bool> = ExportFormat::ALL.iter().map(|f| app.tool_blocked(Tool::Export(*f)).is_none()).collect();
    let mut index = ExportFormat::ALL.iter().position(|f| *f == app.settings.export_format).unwrap_or(0);
    kit::row(ui, "Format", |ui| {
        if kit::dropdown(ui, "export_format", &mut index, &labels, &enabled) {
            app.set_export_format(ExportFormat::ALL[index]);
            if app.tool.is_some_and(|t| matches!(t, Tool::Export(_))) {
                app.tool = Some(Tool::Export(ExportFormat::ALL[index]));
            }
        }
    });
    kit::row(ui, "File", |ui| path_field(app, ui, PickPurpose::ExportFile));
    let source = if app.settings.export_format.needs_mesh() {
        app.mesh.as_ref().map(|m| format!("Mesh · {} triangles", thousands(m.analysis.triangle_count))).unwrap_or_else(|| "—".into())
    } else {
        format!("{} · {} points", app.working_source(), thousands(app.working_len()))
    };
    kit::row(ui, "Source", |ui| kit::value_field(ui, &source, true));
    let path = std::path::Path::new(&app.settings.export_path);
    let full = if path.is_absolute() { path.to_path_buf() } else { std::env::current_dir().map(|d| d.join(path)).unwrap_or_else(|_| path.to_path_buf()) };
    kit::note(ui, &format!("Saves to {}", full.display()));
}

fn stage_body(app: &mut ScanCraftApp, ui: &mut egui::Ui) {
    match app.stage {
        Stage::Connect => connect_body(app, ui),
        Stage::Scan => scan_body(app, ui),
        Stage::Align => align_body(app, ui),
        Stage::Clean => clean_body(app, ui),
        Stage::Mesh => mesh_body(app, ui),
        Stage::Export => {
            export_form(app, ui);
            ui.add_space(Tokens::get().space_8);
            let format = app.settings.export_format;
            let blocked = app.tool_blocked(Tool::Export(format));
            if kit::wide_button(ui, "Export", Button::Primary, blocked.is_none()).on_hover_text(blocked.unwrap_or("Export as the chosen format")).clicked() {
                app.run_tool(Tool::Export(format));
            }
            if let Some(reason) = blocked {
                kit::note(ui, reason);
            }
        }
    }
    selection_section(app, ui);
}

fn connect_body(app: &mut ScanCraftApp, ui: &mut egui::Ui) {
    let t = Tokens::get();
    if let Some(c) = &app.connection {
        let info = c.info.clone();
        let caps = c.capabilities.clone();
        kit::row(ui, "Model", |ui| kit::value_text(ui, &info.model, false));
        kit::row(ui, "Vendor", |ui| kit::value_text(ui, &info.vendor, false));
        kit::row(ui, "Serial", |ui| kit::value_text(ui, &info.serial_number, false));
        kit::row(ui, "Firmware", |ui| kit::value_text(ui, &info.firmware_version, false));
        kit::row(ui, "Link", |ui| kit::value_text(ui, &info.active_transport.description(), false));
        kit::row(ui, "Depth", |ui| kit::value_text(ui, &format!("{} × {}", info.depth_resolution.0, info.depth_resolution.1), false));
        kit::row(ui, "Range", |ui| kit::value_text(ui, &format!("{:.0}–{:.0} mm", info.recommended_distance_range_mm.0, info.recommended_distance_range_mm.1), false));
        kit::section(ui, "caps", "Capabilities", false, |ui| {
            let yn = |b: bool| if b { "Yes" } else { "No" };
            kit::row(ui, "IMU", |ui| kit::value_text(ui, yn(caps.has_imu), true));
            kit::row(ui, "RGB camera", |ui| kit::value_text(ui, yn(caps.has_rgb_camera), true));
            kit::row(ui, "Turntable", |ui| kit::value_text(ui, yn(caps.has_hardware_turntable_control), true));
            kit::row(ui, "Markers", |ui| kit::value_text(ui, yn(caps.supports_marker_tracking), true));
        });
        ui.add_space(t.space_8);
        if kit::wide_button(ui, "Disconnect", Button::Normal, true).clicked() {
            app.disconnect();
        }
        return;
    }
    kit::section_heading(ui, "Found scanners");
    ui.add_space(t.space_4);
    if app.discovered.is_empty() {
        let text = if app.discovering { "Searching…" } else { "None yet. Use Find Scanners, Connect by IP, or the simulated scanner." };
        kit::note(ui, text);
    } else {
        let mut pick = None;
        let mut connect = false;
        for (i, d) in app.discovered.iter().enumerate() {
            let label = format!("{} · {}", d.model_name, d.selected_transport.description());
            let r = kit::tree_row(ui, egui::Id::new(("found", i)), &Row { icon: if d.driver_id == "mock" { "box" } else { "camera" }, label: &label, selected: app.selected_discovered == i, tip: Some(&d.serial_number), ..Default::default() });
            if r.clicked {
                pick = Some(i);
            }
            if r.double {
                pick = Some(i);
                connect = true;
            }
        }
        if let Some(i) = pick {
            app.selected_discovered = i;
        }
        if connect {
            app.connect_discovered();
        }
    }
    ui.add_space(t.space_8);
    let tip = if app.connecting { "Connecting…" } else if app.discovered.is_empty() { "Find scanners first" } else { "Connect to the selected scanner" };
    let r = kit::action_row(
        ui,
        &kit::Actions {
            primary: Some(("Connect", !app.discovered.is_empty() && !app.connecting)),
            primary_tip: Some(tip),
            cancel: Some(if app.discovering { "Searching…" } else { "Find Scanners" }),
            cancel_enabled: !app.discovering,
            ..Default::default()
        },
    );
    match r.choice {
        Some(kit::Choice::Primary) => app.connect_discovered(),
        Some(kit::Choice::Cancel) => app.discover(),
        _ => {}
    }
}

fn scan_body(app: &mut ScanCraftApp, ui: &mut egui::Ui) {
    let t = Tokens::get();
    let live = app.capture != Capture::Idle;
    let connected = app.connection.is_some();

    // Scan mode, tracking and object, in Revo Scan's order, as the device offers them.
    let choice = app.mode_choice();
    let opts = scan_modes::options(app.modes(), choice);
    let mut next = choice;
    let live_reason = live.then_some("Stop the capture first");
    mode_row(ui, "Scan mode", "scan_mode", &opts.lights, choice.light, live_reason, &mut next.light, scan_modes::light_help(choice.light));
    mode_row(ui, "Tracking", "tracking_mode", &opts.tracking, choice.tracking, live_reason, &mut next.tracking, scan_modes::tracking_help(choice.tracking));
    mode_row(ui, "Object", "object_type", &opts.objects, choice.object, live_reason, &mut next.object, scan_modes::object_help(choice.object));
    if next != choice {
        app.set_mode_choice(next);
    }
    if app.connection.is_none() {
        kit::note(ui, "What a scanner offers shows once it is connected.");
    } else if let Some(reason) = app.mode_blocked() {
        kit::status_line(ui, &reason, t.warning);
    }
    ui.add_space(t.space_4);
    ui.add_enabled_ui(!live, |ui| {
        let accuracies = [AccuracyPreset::HighDetail, AccuracyPreset::Standard, AccuracyPreset::FastPreview];
        let mut accuracy = accuracies.iter().position(|a| *a == app.scan_config.accuracy).unwrap_or(1);
        kit::row(ui, "Point pitch", |ui| {
            if kit::segmented(ui, &mut accuracy, &["0.3 mm", "0.5 mm", "1.0 mm"], &[], &[]) {
                app.scan_config.accuracy = accuracies[accuracy];
            }
        });
        let motions = [TurntableMode::HandheldFreeform, TurntableMode::TurntableContinuous, TurntableMode::TurntableStep];
        let mut motion = motions.iter().position(|m| *m == app.scan_config.turntable_mode).unwrap_or(0);
        kit::row(ui, "Motion", |ui| {
            if kit::dropdown(ui, "motion", &mut motion, &["Handheld", "Turntable, continuous", "Turntable, step"], &[]) {
                app.scan_config.turntable_mode = motions[motion];
            }
        });
    });

    // Exposure, gain and depth range live in the camera view, next to the image they change.
    kit::section(ui, "camera_summary", "Camera", true, |ui| {
        let exposure = match app.connection.as_ref().map(|c| c.capabilities.manual_exposure) {
            Some(false) => "automatic".to_string(),
            _ => format!("{:.2} ms, gain {}", app.scan_config.depth_exposure_us as f32 / 1000.0, app.scan_config.depth_gain),
        };
        kit::row(ui, "Exposure", |ui| kit::value_text(ui, &exposure, true));
        kit::row(ui, "Depth", |ui| kit::value_text(ui, &format!("{:.0}–{:.0} mm", app.settings.min_mm, app.settings.max_mm), true));
        let note = if app.connection.is_some() { "Set these in the camera view at the bottom left." } else { "Set these in the camera view once a scanner is connected." };
        kit::note(ui, note);
        if app.connection.is_some() && !app.show_inset && kit::wide_button(ui, "Show Camera View", Button::Normal, true).clicked() {
            app.show_inset = true;
        }
    });
    crate::ui_ai::scan_live_ai_controls(app, ui);
    kit::section(ui, "density", "Density", false, |ui| {
        ui.add_enabled_ui(!live, |ui| {
            let s = &mut app.settings;
            let mut budget_m = s.point_budget as f32 / 1e6;
            if kit::slider_row(ui, "Budget", &mut budget_m, 0.5..=10.0, 1, "M pts").changed() {
                s.point_budget = (budget_m * 1e6) as usize;
            }
        });
    });
    ui.add_space(t.space_8);
    // How the next pass relates to the ones before (docs/architecture/scan-sessions.md, 2.3).
    if app.passes.iter().any(|p| p.meta.alignment.is_placed()) {
        ui.add_enabled_ui(!live, |ui| {
            let mut start = if app.next_start == PassStart::SamePosition { 1 } else { 0 };
            kit::row(ui, "Next pass", |ui| {
                if kit::segmented(ui, &mut start, &["Object moved", "Same position"], &[], &[]) {
                    app.next_start = if start == 1 { PassStart::SamePosition } else { PassStart::Moved };
                }
            });
        });
        let help = match app.next_start {
            PassStart::Moved => "The object was flipped or moved: the new pass is aligned afterwards on the Align tab.",
            PassStart::SamePosition => "The object was not touched: the new pass places itself in the project as it starts (e.g. another mode for a detail).",
        };
        kit::note(ui, help);
        ui.add_space(t.space_6);
    }
    match app.capture {
        Capture::Idle => {
            let r = kit::wide_button(ui, "Start Capture", Button::Primary, connected);
            let r = if connected { r } else { r.on_hover_text("Connect a scanner first") };
            if r.clicked() {
                app.start_scan();
            }
            if !connected {
                kit::note(ui, "Connect a scanner to capture.");
            }
        }
        Capture::Running | Capture::Paused | Capture::Starting => {
            if kit::wide_button(ui, "Stop Capture", Button::Normal, true).clicked() {
                app.stop_scan();
            }
        }
        Capture::Stopping => {
            kit::wide_button(ui, "Stopping…", Button::Normal, false);
        }
    }
}

/// One mode choice: a dropdown whose disabled options give their reason, and a line on what the
/// chosen option does.
#[allow(clippy::too_many_arguments)]
fn mode_row<T: Copy + PartialEq>(ui: &mut egui::Ui, label: &str, id: &str, options: &[OptionState<T>], current: T, live: Option<&str>, out: &mut T, help: &str) {
    let labels: Vec<&str> = options.iter().map(|o| o.label).collect();
    let enabled: Vec<bool> = options.iter().map(|o| o.enabled() && live.is_none()).collect();
    let reasons: Vec<Option<&str>> = options.iter().map(|o| o.disabled.as_deref().or(live)).collect();
    let mut index = options.iter().position(|o| o.value == current).unwrap_or(0);
    kit::row(ui, label, |ui| {
        ui.add_enabled_ui(live.is_none(), |ui| {
            if kit::dropdown_with_reasons(ui, id, &mut index, &labels, &enabled, &reasons) {
                if let Some(o) = options.get(index) {
                    *out = o.value;
                }
            }
        });
    });
    // The description sits under the control column.
    let t = Tokens::get();
    let indent = t.form_label_width + t.space_8;
    let galley = ui.painter().layout(help.to_string(), egui::FontId::proportional(t.font_small), t.text_dim, (ui.available_width() - indent).max(40.0));
    let (rect, _) = ui.allocate_exact_size(vec2(ui.available_width(), galley.size().y), egui::Sense::hover());
    ui.painter().galley(pos2(rect.left() + indent, rect.top()), galley, t.text_dim);
    ui.add_space(t.space_4);
}

/// The name of pass `id`, for reports.
fn pass_name(app: &ScanCraftApp, id: u32) -> String {
    app.passes.iter().find(|p| p.id == id).map(|p| p.name.clone()).unwrap_or_else(|| format!("Pass {id}"))
}

/// Align All or Refine All waiting for a decision: per pass and per pair, then Accept / Reject.
fn poses_preview_body(app: &mut ScanCraftApp, ui: &mut egui::Ui) {
    let t = Tokens::get();
    let Some(preview) = &app.poses_preview else { return };
    match &preview.proposal {
        Proposal::AlignAll(o) => {
            kit::section_heading(ui, "Align All");
            kit::note(ui, "Every pass was aligned against every other; the passes are shown in their colours where they would go.");
            ui.add_space(t.space_4);
            for p in &o.placements {
                let name = pass_name(app, p.id);
                let (text, colour) = match (p.reference, p.pose, p.quality) {
                    (true, _, _) => ("reference".to_string(), t.accent),
                    (false, None, _) => ("could not be placed".to_string(), t.error),
                    (false, Some(_), q) => {
                        let from = p.parent.map(|id| format!(" · from {}", pass_name(app, id))).unwrap_or_default();
                        match q {
                            Some(q) => (format!("{}{from}", quality_summary(&q)), verdict_colour(q.verdict(), &t)),
                            None => (format!("placed{from}"), t.warning),
                        }
                    }
                };
                kit::row(ui, &name, |ui| kit::status_line(ui, &text, colour));
            }
            let unplaced: Vec<String> = o.placements.iter().filter(|p| !p.reference && p.pose.is_none()).map(|p| pass_name(app, p.id)).collect();
            if !unplaced.is_empty() {
                let it = if unplaced.len() == 1 { "it" } else { "them" };
                kit::status_line(ui, &format!("{} could not be placed: after Accept, use Point Pairs for {it}.", unplaced.join(", ")), t.warning);
            }
            kit::section(ui, "align_all_pairs", "Pairs", true, |ui| {
                for e in &o.edges {
                    let pair = format!("{} ↔ {}", pass_name(app, e.from), pass_name(app, e.to));
                    let fit = e.quality.map(|q| format!(" · {}", quality_summary(&q))).unwrap_or_default();
                    let (text, colour) = match &e.kind {
                        EdgeKind::Placed => (format!("{pair}: measured, places a pass{fit}"), t.agent_ok),
                        EdgeKind::Confirmed => (format!("{pair}: measured, agrees{fit}"), t.agent_ok),
                        EdgeKind::Composed { via } => {
                            let via: Vec<String> = via.iter().map(|id| pass_name(app, *id)).collect();
                            (format!("{pair}: composed via {}{fit}", via.join(" → ")), e.verdict.map_or(t.text_dim, |v| verdict_colour(v, &t)))
                        }
                        EdgeKind::Rejected(reason) => (format!("{pair}: rejected, {reason}"), t.error),
                    };
                    kit::status_line(ui, &text, colour);
                    if let Some(a) = e.agreement.filter(|_| matches!(e.kind, EdgeKind::Placed | EdgeKind::Confirmed)) {
                        kit::note(ui, &format!("    the other way round: {:.1}°, {:.1} mm apart", a.rotation_deg, a.offset_mm));
                    }
                }
                if let Some(r) = o.refine {
                    let mm = |v: Option<f32>| v.map_or("—".to_string(), |v| format!("{v:.2} mm"));
                    let text = if r.kept {
                        format!("Refined together: median pair error {} → {}", mm(r.before_mm), mm(r.after_mm))
                    } else {
                        format!("Joint refinement skipped: it made the fit worse ({} → {})", mm(r.before_mm), mm(r.after_mm))
                    };
                    ui.add_space(t.space_4);
                    kit::note(ui, &text);
                }
            });
        }
        Proposal::Refine(r) => {
            kit::section_heading(ui, "Proposed refinement");
            let mm = |v: Option<f32>| v.map_or("—".to_string(), |v| format!("{v:.2} mm"));
            kit::row(ui, "Pair error", |ui| kit::value_text(ui, &format!("{} → {} (median)", mm(r.median_before()), mm(r.median_after())), false));
            kit::section(ui, "refine_pairs", "Pairs", true, |ui| {
                for e in &r.edges {
                    let (b, a) = (e.error_before(), e.error_after());
                    let colour = if a <= b + 0.005 { t.agent_ok } else { t.warning };
                    let fmt = |v: f32| if v.is_finite() { format!("{v:.2}") } else { "—".into() };
                    let wall = e.thickness_mm.map(|w| format!(" · wall {w:.2} mm")).unwrap_or_default();
                    kit::status_line(ui, &format!("{} ↔ {}: {} → {} mm{wall}", pass_name(app, e.a), pass_name(app, e.b), fmt(b), fmt(a)), colour);
                }
            });
            kit::section(ui, "refine_passes", "Passes after", true, |ui| {
                for (id, q) in &r.qualities {
                    kit::row(ui, &pass_name(app, *id), |ui| kit::status_line(ui, &quality_summary(q), verdict_colour(q.verdict(), &t)));
                }
            });
        }
    }
}

fn align_body(app: &mut ScanCraftApp, ui: &mut egui::Ui) {
    let t = Tokens::get();
    if app.orient_preview.is_some() {
        crate::ui_orient::orient_preview_body(app, ui);
        return;
    }
    if app.poses_preview.is_some() {
        poses_preview_body(app, ui);
        return;
    }
    // A proposed alignment waits for a decision.
    if let Some(preview) = &app.align_preview {
        let name = app.passes.iter().find(|p| p.id == preview.pass).map(|p| p.name.clone()).unwrap_or_default();
        let q = preview.alignment.quality;
        let check = preview.check;
        let verdict = alignment::checked_verdict(&q, check.as_ref());
        let method = preview.method.label();
        kit::section_heading(ui, "Proposed alignment");
        kit::row(ui, "Pass", |ui| kit::value_text(ui, &name, false));
        kit::row(ui, "Method", |ui| kit::value_text(ui, method, false));
        quality_rows(ui, &q, verdict);
        if let Some(c) = check {
            let both = match c.agreement {
                Some(a) if c.consistent => format!("Yes · {:.1}°, {:.1} mm apart", a.rotation_deg, a.offset_mm),
                Some(a) => format!("No · {:.0}°, {:.0} mm apart", a.rotation_deg, a.offset_mm),
                None => "Not found the other way".to_string(),
            };
            kit::row(ui, "Both ways", |ui| kit::value_text(ui, &both, !c.consistent));
            kit::row(ui, "Runs", |ui| kit::value_text(ui, &format!("{} of {} agree", c.agreeing, c.runs), true));
            if c.reverse.contradicted() {
                kit::status_line(ui, "Seen from the other pass this fit contradicts what was measured.", t.error);
            }
            if !c.object_only {
                kit::note(ui, "Aligned with the whole scene (table included).");
            }
        }
        ui.add_space(t.space_4);
        match verdict {
            scancraft_align::Verdict::Good => {
                kit::note(ui, "The pass is shown in its colour where it would go. It fits closely and hides nothing that was measured.");
            }
            scancraft_align::Verdict::Fair => {
                kit::note_with_tip(
                    ui,
                    "Check the fit in the view before accepting.",
                    "The overlap is small, the surfaces do not agree everywhere, or aligning the other way round gives another result.",
                );
            }
            scancraft_align::Verdict::Poor => {
                kit::note_with_tip(
                    ui,
                    "This looks wrong; try Point Pairs instead.",
                    "Little overlap, it puts surface where the scanner saw through, below the table or through the other surface.",
                );
            }
        }
        return;
    }
    let placed = app.passes.iter().filter(|p| p.meta.alignment.is_placed()).count();
    let unaligned: Vec<String> = app.passes.iter().filter(|p| p.meta.alignment == AlignState::Unaligned).map(|p| p.name.clone()).collect();
    let reference = app.reference_pass().and_then(|id| app.passes.iter().find(|p| p.id == id)).map(|p| p.name.clone());
    kit::row(ui, "Reference", |ui| kit::value_text(ui, &dash(reference), false));
    kit::row(ui, "Aligned", |ui| kit::value_text(ui, &format!("{placed} of {}", app.passes.len()), false));
    if !unaligned.is_empty() {
        kit::row(ui, "To align", |ui| kit::value_text(ui, &unaligned.join(", "), false));
    }
    ui.add_space(t.space_4);
    if app.passes.len() < 2 {
        kit::note(ui, "Capture another pass (flip the object, or keep it in place and choose Same position) to align it here.");
    } else if !unaligned.is_empty() {
        kit::note_with_tip(
            ui,
            "Align All places every pass at once. Each result is previewed before it is accepted.",
            "Also works for inside and outside scans of a shell. Align Pass places the selected pass; use Point Pairs when the automatic result is wrong.",
        );
    } else {
        kit::note(ui, "All passes are aligned. Refine All spreads the remaining errors; Merge fuses the included passes into one model.");
    }
    ui.add_space(t.space_6);
    let blocked = app.align_all_blocked();
    if kit::wide_button(ui, "Align All", if unaligned.is_empty() { Button::Normal } else { Button::Primary }, blocked.is_none()).on_hover_text(blocked.unwrap_or("Align every pass against every other and place them all from the reference")).clicked() {
        app.align_all();
    }
    kit::section(ui, "isolation_settings", "Object", true, |ui| {
        let mut whole = app.settings.whole_scene;
        if kit::checkbox(ui, "Use whole scene", &mut whole).on_hover_text("Turn on if the object itself is cut off. When off, no fit may push one pass below another's table.").changed() {
            app.settings.whole_scene = whole;
        }
        let note = if whole {
            "Alignment and Merge use everything the passes saw, the table included."
        } else {
            "Alignment and Merge use the object only, without the table and loose fragments."
        };
        kit::note(ui, note);
    });
    kit::section(ui, "merge_settings", "Merge", true, |ui| {
        kit::checkbox(ui, "Remove outliers", &mut app.settings.merge_outliers);
        ui.add_enabled_ui(app.settings.merge_outliers, |ui| {
            kit::slider_row(ui, "Threshold", &mut app.settings.merge_sigma, 1.0..=5.0, 1, "σ");
        });
        kit::note_with_tip(
            ui,
            "Where passes overlap, the more accurate one wins.",
            "Laser > Full-field HD > VCSEL; set a pass's Merge weight in its properties. Single observations without surface around them are dropped.",
        );
    });
    if let Some(s) = app.tracking.as_ref().filter(|_| app.capture != Capture::Idle) {
        kit::section(ui, "live_tracking", "Live tracking", false, |ui| {
            kit::row(ui, "State", |ui| kit::value_text(ui, &format!("{:?}", s.state), false));
            kit::row(ui, "RMS error", |ui| kit::value_text(ui, &format!("{:.2} mm", s.rmse_mm), false));
        });
    }
}

fn clean_body(app: &mut ScanCraftApp, ui: &mut egui::Ui) {
    let t = Tokens::get();
    let targets = app.clean_targets();
    let label = app.clean_targets_label();
    kit::row(ui, "Works on", |ui| kit::value_text(ui, &label, targets.is_empty()));
    kit::row(ui, "Points", |ui| kit::value_text(ui, &thousands(app.clean_len()), targets.is_empty()));
    let cleaned = app.passes.iter().filter(|p| p.is_cleaned()).count();
    kit::row(ui, "Cleaned", |ui| kit::value_text(ui, &format!("{cleaned} of {} passes", app.passes.len()), false));
    ui.add_space(t.space_6);
    kit::note_with_tip(
        ui,
        "Tools work on each shown pass separately; hidden passes are left alone.",
        "Hide the others (eye in the browser) to work on one pass. Isolate Object needs exactly one shown pass and keeps the object, dropping the table, marker blocks and loose fragments. Clean each pass, then align them.",
    );
}

fn mesh_body(app: &mut ScanCraftApp, ui: &mut egui::Ui) {
    let t = Tokens::get();
    let a = app.mesh.as_ref().map(|m| m.analysis.clone());
    kit::row(ui, "Vertices", |ui| kit::value_text(ui, &dash(a.as_ref().map(|a| thousands(a.vertex_count))), a.is_none()));
    kit::row(ui, "Triangles", |ui| kit::value_text(ui, &dash(a.as_ref().map(|a| thousands(a.triangle_count))), a.is_none()));
    kit::row(ui, "Area", |ui| kit::value_text(ui, &dash(a.as_ref().map(|a| format!("{:.1} cm²", a.surface_area_mm2 / 100.0))), a.is_none()));
    kit::row(ui, "Volume", |ui| kit::value_text(ui, &dash(a.as_ref().filter(|a| a.is_watertight).map(|a| format!("{:.1} cm³", a.volume_mm3 / 1000.0))), a.as_ref().is_none_or(|a| !a.is_watertight)));
    kit::row(ui, "Watertight", |ui| kit::value_text(ui, &dash(a.as_ref().map(|a| if a.is_watertight { "Yes".into() } else { format!("No · {} open edges", thousands(a.open_edges)) })), a.is_none()));
    ui.add_space(t.space_6);
    if a.is_none() {
        kit::note(ui, "Reconstruct builds a surface from the merged model when it is shown, else from the shown passes.");
    }
}

/// Facts about the browser selection, below the stage settings.
fn selection_section(app: &mut ScanCraftApp, ui: &mut egui::Ui) {
    let Some(node) = app.selection else { return };
    let title = match node {
        Node::Device | Node::Frame(..) => return,
        Node::Pass(_) => "Selected pass",
        Node::Merged => "Merged model",
        Node::Mesh => return,
    };
    let mut delete = None;
    let mut continue_pass = None;
    let mut weight_change = None;
    let mut include_change = None;
    let mut scale = None;
    let mut scale_tool = None;
    let mut reset = None;
    kit::section(ui, "selection", title, true, |ui| match node {
        Node::Pass(id) => {
            let Some(p) = app.passes.iter().find(|p| p.id == id) else { return };
            kit::row(ui, "Name", |ui| kit::value_text(ui, &p.name, false));
            if p.meta.origin.is_imported() {
                kit::row(ui, "Source", |ui| kit::value_text(ui, &p.meta.origin.label(), false));
            }
            kit::row(ui, "Points", |ui| kit::value_text(ui, &thousands(p.layer.len()), false));
            let b = p.layer.cloud.bbox;
            let size = if b.is_empty() { "—".to_string() } else { let e = b.max - b.min; format!("{:.0} × {:.0} × {:.0} mm", e.x, e.y, e.z) };
            let precise = if b.is_empty() { size.clone() } else { let e = b.max - b.min; format!("{:.3} × {:.3} × {:.3}", e.x, e.y, e.z) };
            let recording = app.active_pass == Some(id);
            if let Some(surface) = &p.surface {
                let tiny = crate::imports::largest_extent(surface) < crate::imports::METRES_MAX_EXTENT;
                kit::row(ui, "Size", |ui| kit::value_text(ui, if tiny { &precise } else { &size }, false));
                if tiny {
                    kit::status_line(ui, "Under 2 units across: probably in metres", Tokens::get().warning);
                    let why = app.edit_blocked();
                    if kit::wide_button(ui, "Convert Metres to mm", Button::Primary, why.is_none()).on_hover_text(why.unwrap_or("Multiply the coordinates by 1000")).clicked() {
                        scale = Some(id);
                    }
                    ui.add_space(Tokens::get().space_4);
                }
                let why = app.scale_blocked(Some(id));
                if kit::wide_button(ui, "Scale from Lengths…", Button::Normal, why.is_none()).on_hover_text(why.unwrap_or("Pick lengths on the pass, then type their true distances")).clicked() {
                    scale_tool = Some(id);
                }
            } else {
                kit::row(ui, "Size", |ui| kit::value_text(ui, &size, false));
                kit::row(ui, "Frames", |ui| kit::value_text(ui, &thousands(p.frames as usize), false));
                let kept = p.kept_frames();
                let stored = p.keyframes.len();
                kit::row(ui, "Keyframes", |ui| kit::value_text(ui, &if kept == stored { thousands(stored) } else { format!("{} of {} kept", thousands(kept), thousands(stored)) }, false));
                kit::row(ui, "Mode", |ui| kit::value_text(ui, &format!("{} · {}", p.meta.scan_config.light.label(), p.meta.scan_config.tracking.label()), false));
            }
            let t = Tokens::get();
            kit::row(ui, "Alignment", |ui| kit::status_line(ui, &p.meta.alignment.label(), align_colour(p.meta.alignment, &t)));
            if let AlignState::Aligned { quality: Some(q), .. } = p.meta.alignment {
                quality_rows(ui, &q, q.verdict());
            }
            if let Some(m) = crate::isolation::cached(&p.mask) {
                let text = if m.fallback {
                    "whole pass (no object told apart)".to_string()
                } else {
                    format!("{} of {} points{}", thousands(m.kept), thousands(m.object.len()), if m.support.is_some() { " · on a table" } else { "" })
                };
                kit::row(ui, "Object", |ui| kit::value_text(ui, &text, true));
            }
            kit::row(ui, "Cleaning", |ui| kit::value_text(ui, if p.is_cleaned() { "Cleaned" } else { "As scanned" }, !p.is_cleaned()));
            let why = app.edit_blocked();
            if p.is_cleaned() && kit::wide_button(ui, "Reset Cleaning", Button::Normal, why.is_none()).on_hover_text(why.unwrap_or("Take the pass back to how it was scanned or imported")).clicked() {
                reset = Some(id);
            }
            let (mut weight, mut included) = (p.meta.weight, p.meta.included);
            if kit::slider_row(ui, "Merge weight", &mut weight, 0.5..=5.0, 1, "").changed() {
                weight_change = Some((id, weight));
            }
            if kit::checkbox(ui, "Include in Merge", &mut included).changed() {
                include_change = Some((id, included));
            }
            ui.add_space(Tokens::get().space_4);
            if !p.is_fixed() {
                let can_continue = app.connection.is_some() && app.capture == Capture::Idle && app.busy.is_none() && !p.is_cleaned();
                let why = if app.connection.is_none() {
                    Some("Connect a scanner first")
                } else if app.capture != Capture::Idle {
                    Some("Stop the capture first")
                } else if app.busy.is_some() {
                    Some("Another operation is running")
                } else if p.is_cleaned() {
                    Some("A cleaned pass cannot be continued: reset its cleaning first")
                } else {
                    None
                };
                if kit::wide_button(ui, "Continue Pass", Button::Normal, can_continue).on_hover_text(why.unwrap_or("Capture more into this pass, carrying on from its last kept frame")).clicked() {
                    continue_pass = Some(id);
                }
            }
            ui.add_space(Tokens::get().space_8);
            let why = if recording { Some("Stop the capture first") } else { app.edit_blocked() };
            if kit::wide_button(ui, "Delete Pass", Button::Danger, why.is_none()).on_hover_text(why.unwrap_or("")).clicked() {
                delete = Some(id);
            }
        }
        Node::Merged => {
            let Some(m) = &app.merged else { return };
            kit::row(ui, "Points", |ui| kit::value_text(ui, &thousands(m.layer.len()), false));
            let from = m.passes.iter().filter_map(|id| app.passes.iter().find(|p| p.id == *id)).map(|p| p.name.clone()).collect::<Vec<_>>().join(", ");
            kit::row(ui, "From", |ui| kit::value_text(ui, &from, false));
            kit::note(ui, &m.summary);
        }
        _ => {}
    });
    if let Some(id) = scale {
        app.scale_pass(id, 1000.0);
    }
    if let Some(id) = scale_tool {
        app.open_tool(Tool::Scale(id));
    }
    if let Some(id) = reset {
        app.reset_cleaning_of(&[id]);
    }
    if let Some((id, w)) = weight_change {
        app.set_weight(id, w);
    }
    if let Some((id, on)) = include_change {
        app.set_included(id, on);
    }
    if let Some(id) = continue_pass {
        app.stage = Stage::Scan;
        app.continue_scan(id);
    }
    if let Some(id) = delete {
        app.delete_pass(id);
        if app.tone != Tone::Error {
            app.selection = None;
        }
    }
}

/// Exposure, gain (laser power for the laser lines) and depth range, as the camera view shows
/// them under the image. Scanners that expose automatically get a read-only line instead of
/// sliders that would do nothing.
pub fn camera_controls(app: &mut ScanCraftApp, ui: &mut egui::Ui) {
    let t = Tokens::get();
    let live = app.capture != Capture::Idle;
    let manual = app.connection.as_ref().is_none_or(|c| c.capabilities.manual_exposure);
    kit::scope(ui, |ui| {
        if manual {
            let mut exposure_ms = app.scan_config.depth_exposure_us as f32 / 1000.0;
            if kit::slider_row(ui, "Exposure", &mut exposure_ms, 0.05..=20.0, 2, "ms").changed() {
                app.scan_config.depth_exposure_us = (exposure_ms * 1000.0).round() as u32;
                app.camera_from_preset = false;
                app.scan_config.camera_override = true;
                if live {
                    app.set_parameter(DeviceParam::DepthExposure(app.scan_config.depth_exposure_us));
                }
            }
            let mut gain = app.scan_config.depth_gain.clamp(1, 10);
            if kit::slider_row(ui, "Gain", &mut gain, 1..=10, 0, "").changed() {
                app.scan_config.depth_gain = gain;
                app.camera_from_preset = false;
                app.scan_config.camera_override = true;
                if live {
                    app.set_parameter(DeviceParam::DepthGain(gain));
                }
            }
        } else {
            kit::row(ui, "Exposure", |ui| kit::value_field(ui, "Automatic", true))
                .on_hover_text("This scanner sets exposure and gain itself in this mode; values sent to it have no effect.");
            kit::note(ui, "Exposed automatically by the scanner in this mode.");
        }
        // Laser power applies to the blue laser lines only.
        if matches!(app.scan_config.light, LightSource::CrossLines | LightSource::SingleLine) {
            let mut power = app.scan_config.laser_power;
            if kit::slider_row(ui, "Laser", &mut power, 1..=10, 0, "").changed() {
                app.scan_config.laser_power = power;
                if live {
                    app.set_parameter(DeviceParam::LaserPower(power));
                }
            }
        }
        ui.add_enabled_ui(!live, |ui| {
            let s = &mut app.settings;
            kit::slider_row(ui, "Near", &mut s.min_mm, 50.0..=600.0, 0, "mm").on_disabled_hover_text("Set the depth range before capturing");
            kit::slider_row(ui, "Far", &mut s.max_mm, 100.0..=1500.0, 0, "mm").on_disabled_hover_text("Set the depth range before capturing");
            if s.max_mm < s.min_mm + 20.0 {
                s.max_mm = s.min_mm + 20.0;
            }
        });
        if manual && !app.camera_from_preset && scan_modes::preset(app.scan_config.light, app.scan_config.object).is_some() {
            ui.horizontal(|ui| {
                kit::note(ui, "Changed from the preset.");
                if ui.small_button("Reset").on_hover_text("Back to the preset's exposure, gain and depth range").clicked() {
                    app.apply_preset();
                }
            });
        }
        let _ = t;
    });
}
