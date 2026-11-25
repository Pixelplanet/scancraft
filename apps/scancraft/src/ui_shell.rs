//! The window chrome: application bar, the workspace and ribbon cards, and the status bar.

use eframe::egui::{self, Rect, pos2, vec2};
use scancraft_ui::kit::{self, RibbonButton, RibbonGroup};
use scancraft_ui::{Tokens, fonts};

use crate::app::{Capture, ScanCraftApp, Tone};
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
        x += 26.0;
        kit::app_bar_separator(ui.painter(), r, x);
        // Project files.
        x += 22.0;
        let new = Rect::from_center_size(pos2(x, r.center().y), vec2(32.0, 32.0));
        if kit::app_bar_button(ui, new, "new", "New project", app.can_edit()).clicked() {
            app.new_project();
        }
        x += 32.0;
        let open = Rect::from_center_size(pos2(x, r.center().y), vec2(32.0, 32.0));
        if kit::app_bar_button(ui, open, "open", "Open project…", app.can_edit()).clicked() {
            app.pick(&ui.ctx().clone(), PickPurpose::OpenProject);
        }
        x += 32.0;
        let save = Rect::from_center_size(pos2(x, r.center().y), vec2(32.0, 32.0));
        let save_tip = match &app.project.path {
            Some(p) => format!("Save project ({})", p.display()),
            None => "Save project…".to_string(),
        };
        if kit::app_bar_button(ui, save, "save", &save_tip, app.tool_blocked(Tool::SaveProject).is_none()).clicked() {
            app.open_tool(Tool::SaveProject);
        }
        x += 26.0;
        kit::app_bar_separator(ui.painter(), r, x);
        // Passes from outside the project.
        x += 22.0;
        let import = Rect::from_center_size(pos2(x, r.center().y), vec2(32.0, 32.0));
        if kit::app_bar_button(ui, import, "import", "Import scan… (PLY, OBJ, STL as a new pass)", app.import_blocked().is_none()).clicked() {
            app.pick(&ui.ctx().clone(), PickPurpose::ImportScans);
        }
        x += 32.0;
        let add = Rect::from_center_size(pos2(x, r.center().y), vec2(32.0, 32.0));
        if kit::app_bar_button(ui, add, "folder", "Add project… (append another project's passes)", app.import_blocked().is_none()).clicked() {
            app.pick(&ui.ctx().clone(), PickPurpose::AddProjects);
        }
        x += 26.0;
        kit::app_bar_separator(ui.painter(), r, x);
        // The project tab, as SolidCraft's document tab: name, and a dot while it has unsaved changes.
        let name = app.project.name.clone();
        let dirty = app.dirty();
        let galley = ui.painter().layout_no_wrap(name, fonts::semibold(t.font_label), t.app_bar_text);
        let tab = Rect::from_min_size(pos2(x + 10.0, r.center().y - 13.0), vec2(galley.size().x + if dirty { 34.0 } else { 24.0 }, 26.0));
        ui.painter().rect_filled(tab, t.radius_sm, t.app_bar.lerp_to_gamma(egui::Color32::WHITE, 0.08));
        ui.painter().galley(pos2(tab.left() + 12.0, tab.center().y - galley.size().y / 2.0), galley, t.app_bar_text);
        if dirty {
            ui.painter().circle_filled(pos2(tab.right() - 12.0, tab.center().y), 3.0, t.app_bar_text);
        }
        let tip = match &app.project.path {
            Some(p) => format!("{}{}", p.display(), if dirty { " · unsaved changes" } else { "" }),
            None => "Not saved yet".to_string(),
        };
        ui.interact(tab, ui.id().with("project_tab"), egui::Sense::hover()).on_hover_text(tip);
        // Connection pill at the right.
        let (dot, text) = match &app.connection {
            Some(c) => (t.agent_ok, format!("{} connected", c.short_name())),
            None if app.connecting => (t.agent_busy, "Connecting…".to_string()),
            None => (t.text_dim, "No scanner".to_string()),
        };
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

fn reason(blocked: Option<&'static str>, tip: &'static str) -> &'static str {
    blocked.unwrap_or(tip)
}

/// The ribbon card for the current stage; runs the clicked command.
pub fn ribbon(app: &mut ScanCraftApp, ui: &mut egui::Ui, rect: Rect) {
    let tabs: Vec<&str> = Stage::ALL.iter().map(|s| s.label()).collect();
    let mut active = app.stage.index();
    let connected = app.connection.is_some();
    let idle = app.capture == Capture::Idle;
    let tool = app.tool;
    let pressed = |t: Tool| tool == Some(t);
    let groups: Vec<RibbonGroup> = match app.stage {
        Stage::Connect => vec![
            RibbonGroup {
                label: "Device",
                buttons: vec![
                    RibbonButton {
                        id: "discover",
                        icon: "web",
                        label: "Find Scanners",
                        enabled: !app.discovering && !connected,
                        pressed: app.discovering,
                        tip: if connected { "Disconnect first" } else { "Search USB and the local network for scanners" },
                    },
                    RibbonButton {
                        id: "connect",
                        icon: "drive",
                        label: "Connect",
                        enabled: !connected && !app.connecting && !app.discovered.is_empty(),
                        pressed: app.connecting,
                        tip: if connected { "Already connected" } else if app.discovered.is_empty() { "Find scanners first, then pick one" } else { "Connect to the scanner picked in the panel" },
                    },
                    RibbonButton { id: "ip", icon: "open", label: "Connect by IP", enabled: !connected && !app.connecting, pressed: pressed(Tool::ConnectIp), tip: "Connect to a scanner at a known network address" },
                    RibbonButton { id: "disconnect", icon: "cut", label: "Disconnect", enabled: connected, pressed: false, tip: if connected { "Stop streaming and release the scanner" } else { "Not connected" } },
                ],
            },
            RibbonGroup {
                label: "Simulator",
                buttons: vec![RibbonButton {
                    id: "mock",
                    icon: "box",
                    label: "Simulated Scanner",
                    enabled: !connected && !app.connecting,
                    pressed: false,
                    tip: "Connect the built-in simulator: synthetic depth and IR frames for trying the workflow",
                }],
            },
        ],
        Stage::Scan => vec![
            RibbonGroup {
                label: "Capture",
                buttons: vec![
                    RibbonButton {
                        id: "start",
                        icon: "play",
                        label: "Start",
                        enabled: connected && idle,
                        pressed: matches!(app.capture, Capture::Running | Capture::Starting),
                        tip: if !connected { "Connect a scanner first" } else if !idle { "Capturing" } else { "Start a new scan pass (Space)" },
                    },
                    RibbonButton {
                        id: "pause",
                        icon: "pause",
                        label: if app.capture == Capture::Paused { "Resume" } else { "Pause" },
                        enabled: matches!(app.capture, Capture::Running | Capture::Paused),
                        pressed: app.capture == Capture::Paused,
                        tip: "Pause or resume adding points to this pass",
                    },
                    RibbonButton {
                        id: "stop",
                        icon: "stop_capture",
                        label: "Stop",
                        enabled: matches!(app.capture, Capture::Running | Capture::Paused | Capture::Starting),
                        pressed: false,
                        tip: "Stop capturing and switch the projector off (Space)",
                    },
                ],
            },
            RibbonGroup {
                label: "Aim",
                buttons: vec![
                    RibbonButton {
                        id: "preview",
                        icon: "camera",
                        label: "Live Preview",
                        enabled: connected && idle && !app.preview_starting,
                        pressed: app.previewing,
                        tip: if !connected { "Connect a scanner first" } else if !idle { "The capture shows its own preview" } else { "Show the camera image without capturing" },
                    },
                    RibbonButton { id: "inset", icon: "inspect", label: "Camera Inset", enabled: true, pressed: app.show_inset, tip: "Show the camera image over the view while live" },
                ],
            },
            RibbonGroup {
                label: "Add Passes",
                buttons: vec![
                    RibbonButton {
                        id: "import_scan",
                        icon: "import",
                        label: "Import Scan",
                        enabled: app.import_blocked().is_none(),
                        pressed: false,
                        tip: reason(app.import_blocked(), "Add PLY, OBJ or STL scans (yours or from other scan software) as new passes to align with the others"),
                    },
                    RibbonButton {
                        id: "add_project",
                        icon: "folder",
                        label: "Add Project",
                        enabled: app.import_blocked().is_none(),
                        pressed: false,
                        tip: reason(app.import_blocked(), "Append the passes of other ScanCraft projects, unaligned"),
                    },
                ],
            },
            RibbonGroup {
                label: "Session",
                buttons: vec![RibbonButton {
                    id: "clear",
                    icon: "delete",
                    label: "Clear All",
                    enabled: app.can_edit() && app.has_geometry(),
                    pressed: false,
                    tip: if app.has_geometry() { "Remove every pass and result (undoable)" } else { "Nothing to clear" },
                }],
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
                        RibbonButton {
                            id: "align_all",
                            icon: "align",
                            label: "Align All",
                            enabled: all_blocked.is_none(),
                            pressed: app.busy.as_deref().is_some_and(|b| b.starts_with("Aligning all")),
                            tip: reason(all_blocked, "Align every pass against every other and place them all from the reference (also inside and outside scans of one part)"),
                        },
                        RibbonButton {
                            id: "align_auto",
                            icon: "checkpoint",
                            label: "Align Pass",
                            enabled: blocked.is_none() && !proposed,
                            pressed: app.busy.as_deref().is_some_and(|b| b.starts_with("Aligning") && !b.starts_with("Aligning all")),
                            tip: if proposed { "Accept or reject the proposed alignment first" } else { reason(blocked, "Find the selected (or next unaligned) pass's place in the project automatically") },
                        },
                        RibbonButton {
                            id: "align_pick",
                            icon: "point",
                            label: "Point Pairs",
                            enabled: blocked.is_none() || picking,
                            pressed: picking,
                            tip: reason(blocked.filter(|_| !picking), "Click three or more matching points on the model and the pass, side by side"),
                        },
                        RibbonButton { id: "align_markers", icon: "snap", label: "Markers", enabled: false, pressed: false, tip: "No markers on the object were recorded in these passes" },
                    ],
                },
                RibbonGroup {
                    label: "Project",
                    buttons: vec![
                        RibbonButton { id: "refine", icon: "update", label: "Refine All", enabled: app.refine_blocked().is_none(), pressed: false, tip: reason(app.refine_blocked(), "Fine-tune all aligned passes together so errors do not add up") },
                        RibbonButton { id: "merge", icon: "combine", label: "Merge", enabled: app.merge_blocked().is_none(), pressed: false, tip: reason(app.merge_blocked(), "Fuse the included aligned passes into one model; the passes stay unchanged") },
                    ],
                },
                RibbonGroup {
                    label: "Inspect",
                    buttons: vec![
                        RibbonButton { id: "colour_passes", icon: "inspect", label: "Pass Colours", enabled: true, pressed: app.colour_passes, tip: "Show each pass in its own colour on this tab" },
                        RibbonButton { id: "deviation", icon: "zebra", label: "Deviation", enabled: false, pressed: false, tip: "Not available yet: pass-to-pass deviation map" },
                    ],
                },
            ]
        }
        Stage::Clean => vec![
            RibbonGroup {
                label: "Filter",
                buttons: vec![
                    RibbonButton {
                        id: "isolate",
                        icon: "body",
                        label: "Isolate Object",
                        enabled: app.tool_blocked(Tool::IsolateObject).is_none() || pressed(Tool::IsolateObject),
                        pressed: pressed(Tool::IsolateObject),
                        tip: reason(app.tool_blocked(Tool::IsolateObject).filter(|_| !pressed(Tool::IsolateObject)), "Keep the object: the table, marker blocks and loose fragments go. Click a cluster to keep it, Shift+click to leave one out"),
                    },
                    RibbonButton { id: "plane", icon: "plane", label: "Remove Plane", enabled: app.tool_blocked(Tool::RemovePlane).is_none(), pressed: pressed(Tool::RemovePlane), tip: reason(app.tool_blocked(Tool::RemovePlane), "Remove the turntable or table plane (RANSAC)") },
                    RibbonButton { id: "outliers", icon: "filter", label: "Outliers", enabled: app.tool_blocked(Tool::Outliers).is_none(), pressed: pressed(Tool::Outliers), tip: reason(app.tool_blocked(Tool::Outliers), "Remove isolated points (statistical outlier removal)") },
                    RibbonButton { id: "crop", icon: "box", label: "Crop Box", enabled: app.tool_blocked(Tool::Crop).is_none(), pressed: pressed(Tool::Crop), tip: reason(app.tool_blocked(Tool::Crop), "Keep only the points inside a box") },
                    RibbonButton { id: "downsample", icon: "snap", label: "Downsample", enabled: app.tool_blocked(Tool::Downsample).is_none(), pressed: pressed(Tool::Downsample), tip: reason(app.tool_blocked(Tool::Downsample), "Merge points into a voxel grid") },
                ],
            },
            RibbonGroup {
                label: "Reset",
                buttons: vec![RibbonButton {
                    id: "revert",
                    icon: "revert",
                    label: "Reset Cleaning",
                    enabled: app.can_reset_cleaning() && app.can_edit(),
                    pressed: false,
                    tip: if app.can_reset_cleaning() { "Take the shown scans back to how they were scanned or imported (undoable)" } else { "No shown scan is cleaned" },
                }],
            },
        ],
        Stage::Mesh => vec![
            RibbonGroup {
                label: "Create",
                buttons: vec![
                    RibbonButton { id: "reconstruct", icon: "mesh_body", label: "Reconstruct", enabled: app.tool_blocked(Tool::Reconstruct).is_none(), pressed: pressed(Tool::Reconstruct), tip: reason(app.tool_blocked(Tool::Reconstruct), "Build a smooth surface from the points") },
                    RibbonButton { id: "make_solid", icon: "body", label: "Make Solid", enabled: app.tool_blocked(Tool::MakeSolid).is_none(), pressed: pressed(Tool::MakeSolid), tip: reason(app.tool_blocked(Tool::MakeSolid), "Build a closed, watertight solid of a thin-walled part") },
                ],
            },
            RibbonGroup {
                label: "Refine",
                buttons: vec![
                    RibbonButton { id: "smooth", icon: "surface", label: "Smooth", enabled: app.tool_blocked(Tool::Smooth).is_none(), pressed: pressed(Tool::Smooth), tip: reason(app.tool_blocked(Tool::Smooth), "Smooth the surface while keeping edges") },
                    RibbonButton { id: "decimate", icon: "reach", label: "Decimate", enabled: app.tool_blocked(Tool::Decimate).is_none(), pressed: pressed(Tool::Decimate), tip: reason(app.tool_blocked(Tool::Decimate), "Reduce the triangle count") },
                    RibbonButton { id: "fill_holes", icon: "patch", label: "Fill Holes", enabled: app.tool_blocked(Tool::FillHoles).is_none(), pressed: pressed(Tool::FillHoles), tip: reason(app.tool_blocked(Tool::FillHoles), "Close holes up to a size, shaped to continue the surface") },
                ],
            },
            RibbonGroup {
                label: "Manage",
                buttons: vec![RibbonButton { id: "delete_mesh", icon: "delete", label: "Delete Mesh", enabled: app.mesh.is_some() && app.can_edit(), pressed: false, tip: if app.mesh.is_some() { "Delete the mesh (undoable)" } else { "No mesh" } }],
            },
        ],
        Stage::Export => vec![
            RibbonGroup {
                label: "Points",
                buttons: vec![RibbonButton {
                    id: "export_ply_cloud",
                    icon: "export",
                    label: "Point Cloud PLY",
                    enabled: app.tool_blocked(Tool::Export(ExportFormat::PlyCloud)).is_none(),
                    pressed: pressed(Tool::Export(ExportFormat::PlyCloud)),
                    tip: reason(app.tool_blocked(Tool::Export(ExportFormat::PlyCloud)), "Binary PLY of the merged model when shown, else of the shown passes"),
                }],
            },
            RibbonGroup {
                label: "Mesh",
                buttons: vec![
                    RibbonButton { id: "export_ply_mesh", icon: "export", label: "Mesh PLY", enabled: app.tool_blocked(Tool::Export(ExportFormat::PlyMesh)).is_none(), pressed: pressed(Tool::Export(ExportFormat::PlyMesh)), tip: reason(app.tool_blocked(Tool::Export(ExportFormat::PlyMesh)), "Binary PLY mesh") },
                    RibbonButton { id: "export_obj", icon: "export", label: "OBJ", enabled: app.tool_blocked(Tool::Export(ExportFormat::ObjMesh)).is_none(), pressed: pressed(Tool::Export(ExportFormat::ObjMesh)), tip: reason(app.tool_blocked(Tool::Export(ExportFormat::ObjMesh)), "Wavefront OBJ") },
                    RibbonButton { id: "export_stl", icon: "export", label: "STL", enabled: app.tool_blocked(Tool::Export(ExportFormat::StlMesh)).is_none(), pressed: pressed(Tool::Export(ExportFormat::StlMesh)), tip: reason(app.tool_blocked(Tool::Export(ExportFormat::StlMesh)), "Binary STL for printing") },
                ],
            },
        ],
    };
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
            if pressed(Tool::IsolateObject) {
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
