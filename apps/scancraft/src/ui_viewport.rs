//! The full-bleed 3D view and everything floating over it: the ribbon cards, the navigation bar,
//! the axis triad, the camera inset and the status pill (SolidCraft's floating layout).

use std::sync::Arc;

use eframe::egui::{self, Margin, Rect, Sense, pos2, vec2};
use scancraft_render::{CameraPreset, GridSpec, MeshLayer as GpuMeshLayer, MeshShading, PointLayer, PointShading, SceneFrame};
use scancraft_ui::kit::{self, Cards, NavButton, NavKind, Surface, TriadPick};
use scancraft_ui::{Tokens, fonts};

use crate::app::{Capture, NavTool, ScanCraftApp};
use crate::ui_panels;

const TRIAD_RADIUS: f32 = 44.0;
/// Default width of the camera view's image, points (1.6 x the first version's).
pub const INSET_DEFAULT_W: f32 = 410.0;
const INSET_MIN_W: f32 = 240.0;
const GAUGE_W: f32 = 26.0;
/// Width of the controls column beside the image (wide layout).
const INSET_CONTROLS_W: f32 = 250.0;
/// Height the camera view adds to its image: header + legend (wide), + controls (stacked).
const INSET_EXTRA_WIDE: f32 = 60.0;
const INSET_EXTRA_STACKED: f32 = 205.0;
/// Height of the marker legend under the camera image.
const MARKER_LEGEND_H: f32 = 22.0;

fn rgba(c: egui::Color32, a: u8) -> [u8; 4] {
    [c.r(), c.g(), c.b(), a]
}

pub fn central(app: &mut ScanCraftApp, root: &mut egui::Ui) {
    let t = Tokens::get();
    egui::CentralPanel::default().frame(egui::Frame::NONE.fill(t.viewport_top)).show(root, |ui| {
        let ctx = &ui.ctx().clone();
        let view = ui.max_rect();
        let response = ui.interact(view, ui.id().with("viewport"), Sense::click_and_drag());
        let ppp = ctx.pixels_per_point();
        app.camera.aspect_ratio = (view.width() / view.height().max(1.0)).max(0.1);

        // The 3D scene.
        let mut frame = SceneFrame::new(&app.camera, [view.width() * ppp, view.height() * ppp]);
        frame.point_size_px = app.point_size * ppp;
        frame.point_shading = app.point_shading;
        // Align stage: each pass in its colour (light baked in, so drawn in colour mode).
        let colour_passes = app.stage == crate::model::Stage::Align && app.colour_passes;
        if colour_passes {
            frame.point_shading = PointShading::Vertex;
        }
        frame.mesh_shading = app.mesh_shading;
        frame.body = [t.body.r() as f32 / 255.0, t.body.g() as f32 / 255.0, t.body.b() as f32 / 255.0];
        frame.edge = [t.body_edge.r() as f32 / 255.0, t.body_edge.g() as f32 / 255.0, t.body_edge.b() as f32 / 255.0, 1.0];
        let bbox = app.data_bbox();
        // Passes drawn where a proposal would put them instead of where they are.
        let mut previewed: Vec<u32> = app.align_preview.as_ref().map(|a| a.pass).into_iter().collect();
        if let Some(pp) = &app.poses_preview {
            previewed.extend(pp.layers.iter().map(|(id, _)| *id));
        }
        // Isolate Object or AI Segment preview shows points coloured by object / background label.
        let isolating = app.tool == Some(crate::model::Tool::IsolateObject) && app.object_picking.is_some();
        let ai_picking = app.tool == Some(crate::model::Tool::AiSegment) && app.ai_picking.is_some();
        let ai_has_preview = ai_picking && app.ai_picking.as_ref().is_some_and(|a| a.preview.is_some());
        // The orientation tools show only what they work on: the proposed orientation, the
        // datum preview, or the pass being scaled (plus the grid and their pick markers).
        let datuming = app.tool == Some(crate::model::Tool::Datums) && app.datum_picking.is_some();
        let scaling = matches!(app.tool, Some(crate::model::Tool::Scale(_))) && app.scale_picking.is_some();
        let exclusive = app.orient_preview.is_some() || datuming || scaling;
        // An orientation tool shows its own layer in another frame: the grid and the height
        // ramp follow it, not the project's (old-frame) data.
        let shown_bbox = if let Some(o) = &app.orient_preview {
            Some(o.layer.cloud.bbox)
        } else if datuming {
            app.datum_picking.as_ref().map(|d| d.layer.cloud.bbox)
        } else if scaling {
            app.scale_picking.as_ref().map(|s| s.layer.cloud.bbox)
        } else {
            bbox
        };
        if let Some(b) = &shown_bbox {
            frame.height_range = [b.min.z, b.max.z.max(b.min.z + 1.0)];
        }
        if let Some(o) = &app.orient_preview {
            // Lit like the model normally is, so the proposal can be judged.
            frame.point_shading = app.point_shading;
            frame.points.push(PointLayer { generation: o.layer.generation, points: Arc::clone(&o.layer.gpu), overlay: false });
        } else if datuming {
            // The datum tints bake light into the colours, so they keep their own colours.
            frame.point_shading = PointShading::Vertex;
            if let Some(d) = &app.datum_picking {
                frame.points.push(PointLayer { generation: d.layer.generation, points: Arc::clone(&d.layer.gpu), overlay: false });
            }
        } else if scaling {
            frame.point_shading = app.point_shading;
            if let Some(s) = &app.scale_picking {
                frame.points.push(PointLayer { generation: s.layer.generation, points: Arc::clone(&s.layer.gpu), overlay: false });
            }
        } else if isolating {
            frame.point_shading = PointShading::Vertex;
            if let Some(o) = &app.object_picking {
                frame.points.push(PointLayer { generation: o.generation | 1 << 60, points: Arc::clone(&o.layer), overlay: false });
            }
        } else if ai_has_preview {
            frame.point_shading = PointShading::Vertex;
            if let Some(layer) = app.ai_picking.as_ref().and_then(|a| a.preview.as_ref()) {
                frame.points.push(PointLayer { generation: layer.generation | 1 << 60, points: Arc::clone(&layer.gpu), overlay: false });
            }
        } else if colour_passes {
            let colours: Vec<[u8; 3]> = app.passes.iter().map(|p| app.pass_colour(p.id)).collect();
            for (p, colour) in app.passes.iter_mut().zip(colours) {
                if !p.layer.visible || p.layer.len() == 0 || previewed.contains(&p.id) {
                    continue;
                }
                let fresh = p.tinted.as_ref().is_some_and(|(g, n, _)| *g == p.layer.generation && *n == p.layer.len());
                if !fresh {
                    p.tinted = Some((p.layer.generation, p.layer.len(), Arc::new(crate::model::tinted_points(&p.layer.cloud, colour))));
                }
                if let Some((g, _, gpu)) = &p.tinted {
                    // A generation of its own, so the renderer re-uploads when switching.
                    frame.points.push(PointLayer { generation: *g | 1 << 62, points: Arc::clone(gpu), overlay: false });
                }
            }
        } else {
            for p in app.passes.iter().filter(|p| p.layer.visible && p.layer.len() > 0 && !previewed.contains(&p.id)) {
                frame.points.push(PointLayer { generation: p.layer.generation, points: Arc::clone(&p.layer.gpu), overlay: false });
            }
        }
        if !isolating && !ai_has_preview && !exclusive {
            if let Some(a) = &app.align_preview {
                frame.points.push(PointLayer { generation: a.layer.generation, points: Arc::clone(&a.layer.gpu), overlay: false });
            }
            for (_, layer) in app.poses_preview.iter().flat_map(|pp| pp.layers.iter()) {
                frame.points.push(PointLayer { generation: layer.generation, points: Arc::clone(&layer.gpu), overlay: false });
            }
            if let Some(m) = app.merged.as_ref().filter(|m| m.layer.visible && m.layer.len() > 0) {
                let (generation, points) = if colour_passes { (m.layer.generation | 1 << 62, Arc::clone(&m.tinted)) } else { (m.layer.generation, Arc::clone(&m.layer.gpu)) };
                frame.points.push(PointLayer { generation, points, overlay: false });
            }
        }
        // Fill Holes: the hole outlines, over everything.
        if app.tool == Some(crate::model::Tool::FillHoles) {
            app.refresh_holes();
            if let Some((_, layer)) = app.holes.as_ref().and_then(|h| h.outline.as_ref()) {
                frame.points.push(PointLayer { generation: layer.generation, points: Arc::clone(&layer.gpu), overlay: true });
            }
        }
        // The selected keyframe, over everything.
        app.update_highlight([t.sel_edge.r(), t.sel_edge.g(), t.sel_edge.b()], [t.warning.r(), t.warning.g(), t.warning.b()]);
        if let Some(h) = app.highlight.as_ref().filter(|h| h.layer.len() > 0) {
            frame.points.push(PointLayer { generation: h.layer.generation, points: Arc::clone(&h.layer.gpu), overlay: true });
        }
        // The marker map of the pass being captured, or of the selected pass.
        if let Some((generation, points)) = marker_map_layer(app, &t) {
            frame.points.push(PointLayer { generation, points, overlay: true });
        }
        if let Some(m) = app.mesh.as_ref().filter(|m| m.visible && !isolating && !exclusive) {
            frame.mesh = Some(GpuMeshLayer { generation: m.generation, mesh: Arc::clone(&m.gpu), faces: true, edges: app.show_edges });
        }
        if app.show_grid {
            frame.grid = Some(GridSpec::around(shown_bbox.as_ref(), rgba(t.grid, 255), rgba(t.grid_major, 255), rgba(t.axis_x.gamma_multiply(0.7), 255), rgba(t.axis_y.gamma_multiply(0.7), 255)));
        }
        let cards = Cards::new(view);
        let picking = app.picking.is_some() && matches!(app.tool, Some(crate::model::Tool::PointPairs(_)));
        if picking {
            // The split view replaces the 3D scene between the cards.
            let bottom = view.bottom() - kit::nav::bar_height() - 2.0 * kit::SHELL_GAP;
            let area = Rect::from_min_max(pos2(view.left() + ui_panels::BROWSER_WIDTH + 2.0 * kit::SHELL_GAP, cards.bottom + kit::SHELL_GAP), pos2(view.right() - ui_panels::PROPERTIES_WIDTH - 2.0 * kit::SHELL_GAP, bottom));
            pair_panes(app, ui, area);
        } else if ai_picking {
            let bottom = view.bottom() - kit::nav::bar_height() - 2.0 * kit::SHELL_GAP;
            let area = Rect::from_min_max(pos2(view.left() + ui_panels::BROWSER_WIDTH + 2.0 * kit::SHELL_GAP, cards.bottom + kit::SHELL_GAP), pos2(view.right() - ui_panels::PROPERTIES_WIDTH - 2.0 * kit::SHELL_GAP, bottom));
            let mut scene = Some(frame);
            crate::ui_ai::ai_segment_panes(app, ctx, ui, area, &mut |app, _, ui, right_rect| {
                if let Some(mut frame) = scene.take() {
                    app.camera.aspect_ratio = (right_rect.width() / right_rect.height().max(1.0)).max(0.1);
                    frame.view_proj = app.camera.view_projection();
                    frame.eye = app.camera.eye;
                    frame.viewport_px = [right_rect.width() * ppp, right_rect.height() * ppp];
                    let r_resp = ui.interact(right_rect, ui.id().with("ai_3d_viewport"), Sense::click_and_drag());
                    ui.painter().add(frame.into_shape(right_rect));
                    camera_input(app, ui, &r_resp, right_rect);
                }
            });
        } else {
            ui.painter().add(frame.into_shape(view));
            camera_input(app, ui, &response, view);
            if isolating {
                object_picks(app, ui, &response, view);
            }
            if datuming {
                datum_picks(app, ui, &response, view);
            }
            if scaling {
                scale_picks(app, ui, &response, view);
            }
        }

        // Ribbon cards.
        crate::ui_shell::workspace_card(app, ui, cards.workspace);
        crate::ui_shell::ribbon(app, ui, cards.ribbon);

        // Triad at the top right, under the ribbon.
        let triad_center = pos2(view.right() - kit::SHELL_GAP - TRIAD_RADIUS - 4.0, cards.bottom + TRIAD_RADIUS + 4.0);
        match kit::triad(ui, triad_center, TRIAD_RADIUS, app.camera.view_matrix()) {
            Some(TriadPick::PosX) => app.camera.set_view_preset(CameraPreset::Right),
            Some(TriadPick::NegX) => app.camera.set_view_preset(CameraPreset::Left),
            Some(TriadPick::PosY) => app.camera.set_view_preset(CameraPreset::Back),
            Some(TriadPick::NegY) => app.camera.set_view_preset(CameraPreset::Front),
            Some(TriadPick::PosZ) => app.camera.set_view_preset(CameraPreset::Top),
            Some(TriadPick::NegZ) => app.camera.set_view_preset(CameraPreset::Bottom),
            Some(TriadPick::Home) => app.home_view(),
            None => {}
        }

        // Navigation bar.
        let nav = [
            NavButton { id: "orbit", icon: "orbit", label: "Orbit (left drag)", kind: NavKind::Tool, selected: app.nav_tool == NavTool::Orbit, enabled: true },
            NavButton { id: "pan", icon: "pan", label: "Pan (left drag; right or middle drag always pans)", kind: NavKind::Tool, selected: app.nav_tool == NavTool::Pan, enabled: true },
            NavButton { id: "zoom", icon: "zoom", label: "Zoom (left drag; the wheel always zooms)", kind: NavKind::Tool, selected: app.nav_tool == NavTool::Zoom, enabled: true },
            NavButton::separator(),
            NavButton { id: "fit", icon: "fit", label: "Fit to geometry (F)", kind: NavKind::Action, selected: false, enabled: true },
            NavButton { id: "home", icon: "home", label: "Home view", kind: NavKind::Action, selected: false, enabled: true },
            NavButton { id: "top", icon: "lookat", label: "Look from above", kind: NavKind::Action, selected: false, enabled: true },
            NavButton::separator(),
            NavButton { id: "grid", icon: "grid", label: "Ground grid", kind: NavKind::Toggle, selected: app.show_grid, enabled: true },
            NavButton { id: "display", icon: "display", label: "Display options", kind: NavKind::Toggle, selected: app.display_menu, enabled: true },
        ];
        // Centred in the free space right of the camera view (last frame's place), so the two
        // never overlap.
        let mut nav_area = view;
        if let Some(inset) = app.inset_rect.filter(|_| app.show_inset && app.connection.is_some()) {
            nav_area.min.x = (inset.right() + kit::SHELL_GAP).min(view.right() - 100.0);
            nav_area.max.x = (view.right() - ui_panels::PROPERTIES_WIDTH - kit::SHELL_GAP).max(nav_area.min.x + 100.0);
        }
        let (bar, clicked) = kit::nav_bar(ui, nav_area, &nav);
        match clicked {
            Some("orbit") => app.nav_tool = NavTool::Orbit,
            Some("pan") => app.nav_tool = NavTool::Pan,
            Some("zoom") => app.nav_tool = NavTool::Zoom,
            Some("fit") => app.fit_view(),
            Some("home") => app.home_view(),
            Some("top") => app.camera.set_view_preset(CameraPreset::Top),
            Some("grid") => app.show_grid = !app.show_grid,
            Some("display") => app.display_menu = !app.display_menu,
            _ => {}
        }
        if app.display_menu {
            display_menu(app, ctx, bar);
        }

        // Status pill above the bar while something runs.
        let pill = match (&app.busy, app.capture) {
            (Some(b), _) => Some((t.agent_busy, format!("{b}…"))),
            // While recording, tracking problems take over the pill: nothing is being added while
            // lost, and the user has to know where to point the scanner.
            (None, Capture::Running) if app.relocated_at.is_some_and(|at| at.elapsed() < std::time::Duration::from_millis(1500)) => {
                Some((t.agent_busy, "Re-locating…".to_string()))
            }
            (None, Capture::Running) => match app.tracking.as_ref().map(|s| s.state) {
                Some(scancraft_slam::TrackingState::Lost) => {
                    Some((t.error, "Tracking lost · aim back at the scanned area".to_string()))
                }
                Some(scancraft_slam::TrackingState::Slow) => Some((t.warning, "Slow down · moving too fast".to_string())),
                _ => Some((t.error, format!("Recording · {}", app.active_pass.map(|i| format!("Pass {i}")).unwrap_or_default()))),
            },
            (None, Capture::Paused) => Some((t.warning, "Paused".to_string())),
            (None, Capture::Starting) => Some((t.agent_busy, "Starting capture…".to_string())),
            (None, Capture::Stopping) => Some((t.agent_busy, "Stopping…".to_string())),
            (None, Capture::Idle) if app.previewing => Some((t.agent_ok, "Live preview".to_string())),
            _ => None,
        };
        if let Some((dot, text)) = pill {
            kit::viewport_status(ui.painter(), pos2(bar.center().x, bar.top() - 8.0), Some(dot), &text);
        }
        toast(app, ctx, pos2(view.center().x, cards.bottom + kit::SHELL_GAP));

        // Empty view hint.
        if !app.has_geometry() && app.capture == Capture::Idle {
            let c = view.center() + vec2(0.0, 30.0);
            let (title, hint) = match &app.connection {
                None => ("No scan data", "Connect a scanner on the Connect tab, or use the simulated scanner"),
                Some(_) => ("No scan data", "Press Start on the Scan tab to capture a pass"),
            };
            ui.painter().text(c - vec2(0.0, 10.0), egui::Align2::CENTER_CENTER, title, fonts::semibold(t.font_heading), t.text_dim);
            ui.painter().text(c + vec2(0.0, 12.0), egui::Align2::CENTER_CENTER, hint, egui::FontId::proportional(t.font_body), t.text_dim);
        }

        // Floating cards (their own layers, so they own the pointer over them).
        // The camera view shows whenever a scanner is connected: its controls (exposure, depth
        // range) are needed before the first preview too.
        let inset_shown = app.show_inset && app.connection.is_some();
        let content_top = cards.bottom;
        let content_bottom = view.bottom() - kit::SHELL_GAP;
        // Controls beside the image when the window is wide enough, else under it. The view may
        // take at most 62 % of the height under the ribbon (the browser keeps the rest above it)
        // and must leave the settings card its width.
        let room_w = view.width() - ui_panels::PROPERTIES_WIDTH - 4.0 * kit::SHELL_GAP - GAUGE_W - 4.0 * kit::CARD_INSET;
        let wide = room_w >= INSET_MIN_W + INSET_CONTROLS_W + 12.0;
        let (side_w, extra_h) = if wide { (INSET_CONTROLS_W + 12.0, INSET_EXTRA_WIDE) } else { (0.0, INSET_EXTRA_STACKED) };
        let max_w = (room_w - side_w).max(INSET_MIN_W).min(((content_bottom - content_top) * 0.62 - extra_h).max(INSET_MIN_W * 0.8) / 0.8);
        app.inset_width = app.inset_width.clamp(INSET_MIN_W, max_w.max(INSET_MIN_W));
        let inset_top = app.inset_rect.filter(|_| inset_shown).map(|r| r.top());
        let browser_bottom = inset_top.map_or(content_bottom, |top| top - kit::SHELL_GAP).max(content_top + 120.0);
        let browser_bounds = Rect::from_min_max(pos2(view.left() + kit::SHELL_GAP, content_top), pos2(view.left() + kit::SHELL_GAP + ui_panels::BROWSER_WIDTH, browser_bottom));
        ui_panels::browser(app, ctx, browser_bounds);
        let props_top = triad_center.y + TRIAD_RADIUS + kit::SHELL_GAP + 4.0;
        let props_bounds = Rect::from_min_max(pos2(view.right() - kit::SHELL_GAP - ui_panels::PROPERTIES_WIDTH, props_top), pos2(view.right() - kit::SHELL_GAP, bar.top() - kit::SHELL_GAP - 30.0));
        ui_panels::properties(app, ctx, props_bounds);
        if inset_shown {
            app.inset_rect = Some(camera_inset(app, ctx, pos2(view.left() + kit::SHELL_GAP, content_bottom), max_w, wide));
        } else {
            app.inset_rect = None;
        }
    });
}

fn camera_input(app: &mut ScanCraftApp, ui: &egui::Ui, response: &egui::Response, view: Rect) {
    let (scroll, primary, secondary, middle) = ui.input(|i| (i.smooth_scroll_delta.y, i.pointer.primary_down(), i.pointer.secondary_down(), i.pointer.middle_down()));
    let distance = (app.camera.eye - app.camera.target).length();
    let pan_scale = 2.0 * distance * (app.camera.fov_y_rad * 0.5).tan() / view.height().max(1.0);
    if response.dragged() {
        let d = response.drag_delta();
        let mode = if secondary || middle {
            NavTool::Pan
        } else if primary {
            app.nav_tool
        } else {
            NavTool::Orbit
        };
        match mode {
            NavTool::Orbit => app.camera.orbit(-d.x * 0.008, d.y * 0.008),
            NavTool::Pan => app.camera.pan(-d.x * pan_scale, d.y * pan_scale),
            NavTool::Zoom => app.camera.zoom((d.y * 0.01).exp()),
        }
    }
    if response.hovered() && scroll != 0.0 {
        app.camera.zoom((-scroll * 0.0015).exp());
    }
    if response.double_clicked() {
        app.fit_view();
    }
    if response.dragged() {
        let cursor = match if secondary || middle { NavTool::Pan } else { app.nav_tool } {
            NavTool::Orbit => egui::CursorIcon::Grabbing,
            NavTool::Pan => egui::CursorIcon::AllScroll,
            NavTool::Zoom => egui::CursorIcon::ResizeVertical,
        };
        ui.ctx().set_cursor_icon(cursor);
    }
}

/// The toast under the ribbon: its message, Undo and a close button.
fn toast(app: &mut ScanCraftApp, ctx: &egui::Context, top_centre: egui::Pos2) {
    let Some(message) = app.current_toast().map(str::to_string) else { return };
    let t = Tokens::get();
    let mut undo = false;
    let mut close = false;
    let mut act = false;
    let action = app.toast_action;
    egui::Area::new(egui::Id::new("toast")).pivot(egui::Align2::CENTER_TOP).fixed_pos(top_centre).order(egui::Order::Foreground).show(ctx, |ui| {
        kit::frame(Surface::Toast).inner_margin(Margin::symmetric(12, 8)).show(ui, |ui| {
            ui.horizontal(|ui| {
                ui.set_max_width(520.0);
                ui.add(egui::Label::new(egui::RichText::new(&message).size(t.font_body).color(t.text)).wrap());
                if let Some(a) = action {
                    if kit::button(ui, a.label(), kit::Button::Normal, app.can_edit()).clicked() {
                        act = true;
                    }
                }
                if kit::button(ui, "Undo", kit::Button::Normal, app.history.can_undo() && app.can_edit()).clicked() {
                    undo = true;
                }
                if kit::close_button(ui, "Dismiss").clicked() {
                    close = true;
                }
            });
        });
    });
    if act {
        app.run_toast_action();
    }
    if undo {
        app.undo();
    }
    if undo || close {
        app.toast = None;
        app.toast_action = None;
    }
    ctx.request_repaint_after(std::time::Duration::from_millis(250));
}

/// The point under `pos` nearest to the eye, of candidates given as (screen position, view
/// depth, point): what a click picks, in the main view and in the point-pair panes.
fn pick_point(candidates: impl Iterator<Item = (egui::Pos2, f32, glam::Vec3)>, pos: egui::Pos2) -> Option<glam::Vec3> {
    candidates.filter(|(s, ..)| s.distance(pos) <= 6.0).min_by(|a, b| a.1.total_cmp(&b.1)).map(|(.., p)| p)
}

/// Isolate Object in the main view: a click keeps the cluster under the cursor, Shift+click
/// leaves it out; the picks are drawn as numbered rings.
fn object_picks(app: &mut ScanCraftApp, ui: &egui::Ui, response: &egui::Response, view: Rect) {
    let t = Tokens::get();
    let vp = app.camera.view_projection();
    if response.clicked() && app.busy.is_none() {
        if let (Some(pos), Some(o)) = (response.interact_pointer_pos(), app.object_picking.as_ref()) {
            let targets = Arc::clone(&o.targets);
            let shift = ui.input(|i| i.modifiers.shift);
            let hit = pick_point(targets.iter().filter_map(|&p| project(&vp, view, p).map(|(s, d)| (s, d, p))), pos);
            if let Some(p) = hit {
                app.add_object_pick(p, !shift);
            }
        }
    }
    let Some(o) = app.object_picking.as_ref() else { return };
    for (i, (p, keep)) in o.picks.iter().enumerate() {
        if let Some((s, _)) = project(&vp, view, *p) {
            let colour = if *keep { t.agent_ok } else { t.error };
            ui.painter().circle(s, 7.0, colour, egui::Stroke::new(1.5, egui::Color32::WHITE));
            ui.painter().text(s, egui::Align2::CENTER_CENTER, (i + 1).to_string(), egui::FontId::proportional(t.font_small), egui::Color32::WHITE);
        }
    }
    if response.hovered() {
        ui.ctx().set_cursor_icon(egui::CursorIcon::Crosshair);
    }
}

/// Orient by Datums in the main view: a click fits the datum kind selected in the card. The
/// picks are drawn in the frame the preview transform puts them (old frame when it is off).
fn datum_picks(app: &mut ScanCraftApp, ui: &egui::Ui, response: &egui::Response, view: Rect) {
    let t = Tokens::get();
    let vp = app.camera.view_projection();
    let preview_t = app.datum_picking.as_ref().and_then(|d| d.applied_transform());
    if response.clicked() && app.busy.is_none() {
        if let (Some(pos), Some(d)) = (response.interact_pointer_pos(), app.datum_picking.as_ref()) {
            let targets = Arc::clone(&d.targets);
            let place = move |p: glam::Vec3| preview_t.map(|t| t.transform_point3(p)).unwrap_or(p);
            let hit = pick_point(targets.iter().filter_map(|&p| project(&vp, view, place(p)).map(|(s, dep)| (s, dep, p))), pos);
            if let Some(p) = hit {
                app.datum_pick(p);
            }
        }
    }
    let Some(d) = app.datum_picking.as_ref() else { return };
    let place = |p: glam::Vec3| preview_t.map(|t| t.transform_point3(p)).unwrap_or(p);
    if let Some(origin) = d.origin {
        if let Some((s, _)) = project(&vp, view, place(origin)) {
            ui.painter().circle(s, 7.0, t.accent, egui::Stroke::new(1.5, egui::Color32::WHITE));
        }
    }
    if let Some((_, fit, _, _)) = &d.axis {
        if let scancraft_align::Primitive::Cylinder { point, axis, length, .. } = fit.primitive {
            let a = project(&vp, view, place(point - axis * length * 0.5));
            let b = project(&vp, view, place(point + axis * length * 0.5));
            if let (Some((a, _)), Some((b, _))) = (a, b) {
                ui.painter().line_segment([a, b], egui::Stroke::new(2.0, t.warning));
            }
        }
    }
    if response.hovered() {
        ui.ctx().set_cursor_icon(egui::CursorIcon::Crosshair);
    }
}

/// Scale from Lengths in the main view: clicks pick the ends of each length; completed pairs
/// are drawn as numbered rings joined by a line with the measured length.
fn scale_picks(app: &mut ScanCraftApp, ui: &egui::Ui, response: &egui::Response, view: Rect) {
    let t = Tokens::get();
    let vp = app.camera.view_projection();
    if response.clicked() && app.busy.is_none() {
        if let (Some(pos), Some(s)) = (response.interact_pointer_pos(), app.scale_picking.as_ref()) {
            let targets = Arc::clone(&s.targets);
            let hit = pick_point(targets.iter().filter_map(|&p| project(&vp, view, p).map(|(s, d)| (s, d, p))), pos);
            if let Some(p) = hit {
                app.scale_pick(p);
            }
        }
    }
    let Some(s) = app.scale_picking.as_ref() else { return };
    for (i, pair) in s.pairs.iter().enumerate() {
        for p in [pair.a].into_iter().chain(pair.b) {
            if let Some((pt, _)) = project(&vp, view, p) {
                ui.painter().circle(pt, 7.0, t.accent, egui::Stroke::new(1.5, egui::Color32::WHITE));
                ui.painter().text(pt, egui::Align2::CENTER_CENTER, (i + 1).to_string(), egui::FontId::proportional(t.font_small), egui::Color32::WHITE);
            }
        }
        if let Some(b) = pair.b {
            if let (Some((a, _)), Some((bb, _))) = (project(&vp, view, pair.a), project(&vp, view, b)) {
                ui.painter().line_segment([a, bb], egui::Stroke::new(1.5, t.accent));
                let mid = egui::pos2((a.x + bb.x) * 0.5, (a.y + bb.y) * 0.5 - 9.0);
                ui.painter().text(mid, egui::Align2::CENTER_BOTTOM, format!("{:.1} mm", (b - pair.a).length()), egui::FontId::proportional(t.font_small), egui::Color32::WHITE);
            }
        }
    }
    if response.hovered() {
        ui.ctx().set_cursor_icon(egui::CursorIcon::Crosshair);
    }
}

/// Projects `p` into `rect` with `vp`; returns the screen position and view depth.
fn project(vp: &glam::Mat4, rect: Rect, p: glam::Vec3) -> Option<(egui::Pos2, f32)> {
    let c = *vp * p.extend(1.0);
    if c.w <= 1e-3 {
        return None;
    }
    let n = c.truncate() / c.w;
    if n.x.abs() > 1.2 || n.y.abs() > 1.2 {
        return None;
    }
    Some((pos2(rect.left() + (n.x + 1.0) * 0.5 * rect.width(), rect.top() + (1.0 - n.y) * 0.5 * rect.height()), c.w))
}

/// The point-pair split view: the project model on the left, the moving pass on the right.
/// Drag to orbit (right / middle drag pans), wheel to zoom, click to pick.
fn pair_panes(app: &mut ScanCraftApp, ui: &mut egui::Ui, area: Rect) {
    let t = Tokens::get();
    let moving_name = app.picking.as_ref().and_then(|p| app.passes.iter().find(|q| q.id == p.pass)).map(|p| p.name.clone()).unwrap_or_default();
    let Some(picking) = app.picking.as_mut() else { return };
    let gap = kit::SHELL_GAP;
    let half = (area.width() - gap) / 2.0;
    let panes = [Rect::from_min_size(area.min, vec2(half, area.height())), Rect::from_min_size(pos2(area.left() + half + gap, area.top()), vec2(half, area.height()))];
    let mut clicked: [Option<glam::Vec3>; 2] = [None, None];
    for (side, rect) in panes.into_iter().enumerate() {
        let (points, camera, title) = if side == 0 {
            (Arc::clone(&picking.model), &mut picking.model_camera, "Project model")
        } else {
            (Arc::clone(&picking.moving), &mut picking.moving_camera, moving_name.as_str())
        };
        ui.painter().rect_filled(rect, t.radius_lg, t.viewport_bottom);
        ui.painter().rect_stroke(rect, t.radius_lg, egui::Stroke::new(1.0, t.border), egui::StrokeKind::Inside);
        camera.aspect_ratio = (rect.width() / rect.height().max(1.0)).max(0.1);
        let response = ui.interact(rect, ui.id().with(("pair_pane", side)), Sense::click_and_drag());
        if response.dragged() {
            let d = response.drag_delta();
            let (secondary, middle) = ui.input(|i| (i.pointer.secondary_down(), i.pointer.middle_down()));
            let distance = (camera.eye - camera.target).length();
            let scale = 2.0 * distance * (camera.fov_y_rad * 0.5).tan() / rect.height().max(1.0);
            if secondary || middle {
                camera.pan(-d.x * scale, d.y * scale);
            } else {
                camera.orbit(-d.x * 0.008, d.y * 0.008);
            }
        }
        let scroll = ui.input(|i| i.smooth_scroll_delta.y);
        if response.hovered() && scroll != 0.0 {
            camera.zoom((-scroll * 0.0015).exp());
        }
        let vp = camera.view_projection();
        let view_dir = (camera.target - camera.eye).normalize_or_zero();
        // Far points first, so near ones cover them.
        let mut shown: Vec<(egui::Pos2, f32, egui::Color32, glam::Vec3)> = points
            .iter()
            .filter_map(|&(p, n, c)| {
                let (s, depth) = project(&vp, rect, p)?;
                if !rect.shrink(2.0).contains(s) {
                    return None;
                }
                let k = 0.3 + 0.7 * n.dot(view_dir).abs();
                Some((s, depth, egui::Color32::from_rgb((c[0] as f32 * k) as u8, (c[1] as f32 * k) as u8, (c[2] as f32 * k) as u8), p))
            })
            .collect();
        shown.sort_by(|a, b| b.1.total_cmp(&a.1));
        let mut mesh = egui::Mesh::default();
        let r = 1.4;
        for (s, _, c, _) in &shown {
            mesh.add_colored_rect(Rect::from_center_size(*s, vec2(2.0 * r, 2.0 * r)), *c);
        }
        ui.painter().add(egui::Shape::mesh(mesh));
        ui.painter().text(rect.left_top() + vec2(10.0, 8.0), egui::Align2::LEFT_TOP, title, fonts::semibold(t.font_label), t.text);
        // Picked points, numbered.
        for (i, pair) in picking.pairs.iter().enumerate() {
            let Some(p) = (if side == 0 { pair.0 } else { pair.1 }) else { continue };
            if let Some((s, _)) = project(&vp, rect, p) {
                ui.painter().circle(s, 7.0, t.accent, egui::Stroke::new(1.5, egui::Color32::WHITE));
                ui.painter().text(s, egui::Align2::CENTER_CENTER, (i + 1).to_string(), egui::FontId::proportional(t.font_small), egui::Color32::WHITE);
            }
        }
        if response.clicked() {
            if let Some(pos) = response.interact_pointer_pos() {
                // The nearest-to-the-eye point under the cursor.
                clicked[side] = pick_point(shown.iter().map(|(s, d, _, p)| (*s, *d, *p)), pos);
            }
        }
    }
    for (side, hit) in clicked.into_iter().enumerate() {
        let Some(p) = hit else { continue };
        // Fill the open pair's missing side, or start a new pair.
        let open = picking.pairs.last_mut().filter(|(a, b)| if side == 0 { a.is_none() || b.is_none() && a.is_some() } else { b.is_none() || a.is_none() && b.is_some() });
        match open {
            Some(pair) => {
                if side == 0 {
                    pair.0 = Some(p);
                } else {
                    pair.1 = Some(p);
                }
            }
            None => picking.pairs.push(if side == 0 { (Some(p), None) } else { (None, Some(p)) }),
        }
    }
    let hint = "Drag to turn · right-drag to pan · wheel to zoom · click a point to pick it";
    ui.painter().text(pos2(area.center().x, area.bottom() - 10.0), egui::Align2::CENTER_BOTTOM, hint, egui::FontId::proportional(t.font_small), t.text_dim);
}

fn display_menu(app: &mut ScanCraftApp, ctx: &egui::Context, bar: Rect) {
    let t = Tokens::get();
    let width = 244.0;
    let pos = pos2(bar.right() - width, bar.top() - 6.0);
    egui::Area::new(egui::Id::new("display_menu")).pivot(egui::Align2::LEFT_BOTTOM).fixed_pos(pos).order(egui::Order::Foreground).show(ctx, |ui| {
        kit::frame(Surface::Toast).inner_margin(Margin::same(10)).show(ui, |ui| {
            ui.set_width(width - 20.0);
            kit::scope(ui, |ui| {
                kit::section_heading(ui, "Points");
                let shadings = [PointShading::Lit, PointShading::Height, PointShading::Normals, PointShading::Vertex];
                let mut i = shadings.iter().position(|s| *s == app.point_shading).unwrap_or(0);
                kit::row(ui, "Colour", |ui| {
                    if kit::dropdown(ui, "point_shading", &mut i, &["Shaded", "Height", "Normals", "Scanner colour"], &[]) {
                        app.point_shading = shadings[i];
                    }
                });
                kit::slider_row(ui, "Size", &mut app.point_size, 1.0..=6.0, 1, "px");
                ui.add_space(t.space_4);
                kit::section_heading(ui, "Mesh");
                let mesh = [MeshShading::Lit, MeshShading::Height];
                let mut j = mesh.iter().position(|s| *s == app.mesh_shading).unwrap_or(0);
                kit::row(ui, "Colour", |ui| {
                    if kit::segmented(ui, &mut j, &["Shaded", "Height"], &[], &[]) {
                        app.mesh_shading = mesh[j];
                    }
                });
                kit::checkbox(ui, "Show edges", &mut app.show_edges);
                kit::checkbox(ui, "Ground grid", &mut app.show_grid);
            });
        });
    });
}

/// The camera view, anchored at its bottom-left corner `pos`: header with the IR / Depth switch,
/// the image with marker rings and the distance gauge, the marker legend, then exposure / gain /
/// depth range. The grip at the top right resizes it. Returns where it was drawn.
fn camera_inset(app: &mut ScanCraftApp, ctx: &egui::Context, pos: egui::Pos2, max_w: f32, wide: bool) -> Rect {
    let t = Tokens::get();
    let w = app.inset_width;
    let h = w * 0.8;
    let live = app.capture != Capture::Idle || app.previewing;
    let shown = egui::Area::new(egui::Id::new("camera_inset")).pivot(egui::Align2::LEFT_BOTTOM).fixed_pos(pos).order(egui::Order::Middle).show(ctx, |ui| {
        kit::frame(Surface::Panel).inner_margin(Margin::same(kit::CARD_INSET as i8)).show(ui, |ui| {
            ui.set_width(w + GAUGE_W + 4.0 + if wide { INSET_CONTROLS_W + 12.0 } else { 0.0 });
            ui.spacing_mut().item_spacing = vec2(0.0, 0.0);
            // Header: title and the IR / Depth / Colour switch (the grip sits in the top-right corner).
            let (head, _) = ui.allocate_exact_size(vec2(ui.available_width(), 26.0), Sense::hover());
            ui.painter().text(pos2(head.left() + t.space_6, head.center().y), egui::Align2::LEFT_CENTER, "Camera", fonts::semibold(t.font_label), t.text);
            let has_depth = app.capture != Capture::Idle;
            let has_rgb = app.connection.as_ref().is_some_and(|c| c.capabilities.has_rgb_camera);
            let seg = Rect::from_min_max(pos2(head.right() - 184.0, head.top() + 2.0), pos2(head.right() - 20.0, head.bottom() - 2.0));
            let mut child = ui.new_child(egui::UiBuilder::new().max_rect(seg));
            child.spacing_mut().interact_size.y = seg.height();
            let mut index = match app.inset_view {
                crate::app::InsetView::Ir => 0,
                crate::app::InsetView::Depth if has_depth => 1,
                crate::app::InsetView::Color if has_rgb => 2,
                _ => 0,
            };
            let reasons = [None, Some("Depth is shown while capturing"), Some("No RGB camera on this scanner")];
            kit::form::scope(&mut child, |ui| {
                if kit::segmented(ui, &mut index, &["IR", "Depth", "Colour"], &[true, has_depth, has_rgb], &reasons) {
                    app.inset_view = match index {
                        1 => crate::app::InsetView::Depth,
                        2 => crate::app::InsetView::Color,
                        _ => crate::app::InsetView::Ir,
                    };
                }
            });
            ui.add_space(2.0);
            ui.horizontal(|ui| {
                ui.spacing_mut().item_spacing.x = 4.0;
                let (img_rect, img_resp) = ui.allocate_exact_size(vec2(w, h), Sense::click());
                ui.painter().rect_filled(img_rect, t.radius_sm, t.control_fill);
                let active_view = match app.inset_view {
                    crate::app::InsetView::Depth if has_depth => crate::app::InsetView::Depth,
                    crate::app::InsetView::Color if has_rgb => crate::app::InsetView::Color,
                    _ => crate::app::InsetView::Ir,
                };
                let tex = match active_view {
                    crate::app::InsetView::Ir => app.ir_texture.as_ref(),
                    crate::app::InsetView::Depth => app.depth_texture.as_ref(),
                    crate::app::InsetView::Color => app.ai_mask_texture.as_ref().filter(|_| app.settings.live_ai_segment).or(app.color_texture.as_ref()),
                };
                match tex.filter(|_| live) {
                    Some(tex) => {
                        let size = tex.size_vec2();
                        let scale = (img_rect.width() / size.x).min(img_rect.height() / size.y);
                        let r = Rect::from_center_size(img_rect.center(), size * scale);
                        ui.painter().image(tex.id(), r, Rect::from_min_max(pos2(0.0, 0.0), pos2(1.0, 1.0)), egui::Color32::WHITE);
                        if active_view == crate::app::InsetView::Ir {
                            if let Some(o) = &app.overlay {
                                paint_marker_dots(ui.painter(), r, o);
                            }
                        } else if active_view == crate::app::InsetView::Color && app.settings.live_ai_segment && img_resp.clicked() {
                            if let Some(pos) = img_resp.interact_pointer_pos() {
                                if r.contains(pos) {
                                    let nx = (pos.x - r.min.x) / r.width().max(1.0);
                                    let ny = (pos.y - r.min.y) / r.height().max(1.0);
                                    app.trigger_live_ai_prompt(nx, ny);
                                }
                            }
                        }
                    }
                    None => {
                        let text = if live { "Waiting for frames…" } else { "Start Live Preview or a capture to see the camera" };
                        ui.painter().text(img_rect.center(), egui::Align2::CENTER_CENTER, text, egui::FontId::proportional(t.font_body), t.text_dim);
                    }
                }
                let (gauge, _) = ui.allocate_exact_size(vec2(GAUGE_W, h), Sense::hover());
                let (range, rec) = match &app.connection {
                    Some(c) => ((app.settings.min_mm, app.settings.max_mm), c.info.recommended_distance_range_mm),
                    None => ((app.settings.min_mm, app.settings.max_mm), (app.settings.min_mm, app.settings.max_mm)),
                };
                let distance = app.distance_mm.filter(|_| live);
                kit::distance_gauge(ui, gauge, distance, range, rec).on_hover_text("Distance to the surface; the green band is the scanner's recommended range, the track the depth range");
                if wide {
                    ui.add_space(8.0);
                    ui.allocate_ui_with_layout(vec2(INSET_CONTROLS_W, h), egui::Layout::top_down(egui::Align::Min), |ui| {
                        ui.set_width(INSET_CONTROLS_W);
                        crate::ui_panels::camera_controls(app, ui);
                    });
                }
            });
            if let Some(o) = app.overlay.as_ref().filter(|_| live) {
                ui.allocate_ui_with_layout(vec2(w + GAUGE_W + 4.0, MARKER_LEGEND_H), egui::Layout::top_down(egui::Align::Min), |ui| {
                    ui.set_width(w + GAUGE_W + 4.0);
                    marker_legend(ui, o);
                });
            } else if wide {
                ui.add_space(4.0);
            }
            if !wide {
                ui.add_space(t.space_4);
                egui::Frame::NONE.inner_margin(Margin { left: 6, right: 6, top: 0, bottom: 2 }).show(ui, |ui| {
                    crate::ui_panels::camera_controls(app, ui);
                });
            }
        });
    });
    let rect = shown.response.rect;
    // Resize grip in the top-right corner: drag up / right to enlarge.
    let grip = Rect::from_min_size(pos2(rect.right() - 18.0, rect.top() + 2.0), vec2(16.0, 16.0));
    egui::Area::new(egui::Id::new("camera_inset_grip_area")).fixed_pos(grip.min).order(egui::Order::Foreground).show(ctx, |ui| {
        let (r, response) = ui.allocate_exact_size(grip.size(), Sense::drag());
        let hot = response.hovered() || response.dragged();
        let ink = if hot { t.text } else { t.text_dim };
        for i in 0..3 {
            let o = 4.0 + i as f32 * 4.0;
            ui.painter().line_segment([pos2(r.left() + 2.0, r.top() + o), pos2(r.left() + o, r.top() + 2.0)].map(|p| pos2(r.right() - (p.x - r.left()), p.y)), egui::Stroke::new(1.2, ink));
        }
        if response.dragged() {
            let d = response.drag_delta();
            app.inset_width = (app.inset_width + (d.x - d.y / 0.8) * 0.5).clamp(INSET_MIN_W, max_w.max(INSET_MIN_W));
        }
        if hot {
            ui.ctx().set_cursor_icon(egui::CursorIcon::ResizeNeSw);
        }
        response.on_hover_text("Drag to resize the camera view");
    });
    rect
}

/// Marker colours: tracking (used for the pose), triangulated (3D, not used), detected only.
fn marker_colours(t: &Tokens) -> (egui::Color32, egui::Color32, egui::Color32) {
    (t.agent_ok, t.warning, t.text_dim)
}

/// The dots of the last frame over the IR image shown in `r`: green rings for markers that
/// carried the pose, amber for triangulated markers that did not, grey outlines for dots seen in
/// this image only.
fn paint_marker_dots(painter: &egui::Painter, r: Rect, o: &scancraft_pipeline::ImageOverlay) {
    use scancraft_pipeline::OverlayKind;
    if o.width == 0 || o.height == 0 {
        return;
    }
    let t = Tokens::get();
    let (green, amber, grey) = marker_colours(&t);
    let sx = r.width() / o.width as f32;
    let sy = r.height() / o.height as f32;
    let painter = painter.with_clip_rect(r);
    // Detected first, tracking last, so the important ones are on top.
    let mut points: Vec<_> = o.points.iter().collect();
    points.sort_by_key(|p| match p.kind {
        OverlayKind::Detected => 0,
        OverlayKind::Triangulated => 1,
        OverlayKind::Tracking => 2,
    });
    for p in points {
        let c = pos2(r.left() + p.x * sx, r.top() + p.y * sy);
        let radius = (p.radius * sx).max(2.5) + 2.0;
        // A dark halo under every ring: the dots themselves are the brightest thing in the image.
        painter.circle_stroke(c, radius, egui::Stroke::new(if p.kind == OverlayKind::Tracking { 4.0 } else { 3.0 }, egui::Color32::from_black_alpha(170)));
        match p.kind {
            OverlayKind::Tracking => {
                painter.circle_stroke(c, radius, egui::Stroke::new(2.0, green));
                painter.circle_filled(c, 1.5, green);
            }
            OverlayKind::Triangulated => {
                painter.circle_stroke(c, radius, egui::Stroke::new(1.5, amber));
            }
            OverlayKind::Detected => {
                painter.circle_stroke(c, radius, egui::Stroke::new(1.0, grey));
            }
        }
    }
}

/// "Markers  ● 6 tracking  ● 3 in 3D  ○ 11 seen" under the camera image.
fn marker_legend(ui: &mut egui::Ui, o: &scancraft_pipeline::ImageOverlay) {
    use scancraft_pipeline::OverlayKind;
    let t = Tokens::get();
    let (green, amber, grey) = marker_colours(&t);
    let (rect, response) = ui.allocate_exact_size(vec2(ui.available_width(), MARKER_LEGEND_H), Sense::hover());
    let painter = ui.painter();
    let font = egui::FontId::proportional(t.font_small);
    let cy = rect.center().y + 1.0;
    let mut x = rect.left() + t.space_6;
    let title = painter.text(pos2(x, cy), egui::Align2::LEFT_CENTER, "Markers", fonts::semibold(t.font_small), t.text);
    x = title.right() + t.space_10;
    let tracking = o.count(OverlayKind::Tracking);
    let triangulated = o.count(OverlayKind::Triangulated);
    let seen = o.points.len();
    for (n, label, colour, filled) in [(tracking, "tracking", green, true), (triangulated, "in 3D, unused", amber, false), (seen, "seen", grey, false)] {
        if filled {
            painter.circle_stroke(pos2(x + 4.0, cy), 4.0, egui::Stroke::new(2.0, colour));
        } else {
            painter.circle_stroke(pos2(x + 4.0, cy), 4.0, egui::Stroke::new(1.5, colour));
        }
        let r = painter.text(pos2(x + 12.0, cy), egui::Align2::LEFT_CENTER, format!("{n} {label}"), font.clone(), t.text_dim);
        x = r.right() + t.space_10;
    }
    let map_size = o.map.len();
    let matched = o.map.iter().filter(|m| m.1).count();
    let mut tip = format!(
        "{tracking} of {seen} dots in the left IR image carry this frame's pose (green).\n{triangulated} more were triangulated but not used (amber); the rest were seen by one camera only (grey).\nMarker map: {map_size} markers, {matched} matched this frame."
    );
    if o.stale {
        tip.push_str("\nThe scanner repeated the previous IR image: no new marker observation this frame.");
    }
    response.on_hover_text(tip);
}

/// GPU points for the marker map of the pass being captured (matched markers green) or of the
/// selected pass (grey), placed in the project frame. The alpha channel sets the disc size in the
/// overlay shader (alpha / 32 × point size).
fn marker_map_layer(app: &ScanCraftApp, t: &Tokens) -> Option<(u64, Arc<Vec<scancraft_render::GpuPoint>>)> {
    use crate::model::Node;
    let pass_id = match (app.active_pass, app.selection) {
        (Some(id), _) if app.capture != Capture::Idle => id,
        (_, Some(Node::Pass(id))) | (_, Some(Node::Frame(id, _))) => id,
        _ => return None,
    };
    let map: &[([f32; 3], bool)] = match (&app.overlay, app.active_pass) {
        (Some(o), Some(active)) if active == pass_id && !o.map.is_empty() => &o.map,
        _ => app.marker_maps.get(&pass_id)?,
    };
    let pass = app.passes.iter().find(|p| p.id == pass_id)?;
    if map.is_empty() || !pass.layer.visible {
        return None;
    }
    let (green, _, _) = marker_colours(t);
    let grey = t.timeline_marker;
    let points: Vec<scancraft_render::GpuPoint> = map
        .iter()
        .map(|(p, matched)| {
            let w = pass.meta.transform.transform_point3(glam::Vec3::from(*p));
            let c = if *matched { green } else { grey };
            scancraft_render::GpuPoint { position: w.to_array(), normal: [0, 0, 127, 0], color: [c.r(), c.g(), c.b(), 128] }
        })
        .collect();
    // A generation from the content, so the renderer re-uploads only when the map changes.
    let mut h = std::collections::hash_map::DefaultHasher::new();
    use std::hash::{Hash, Hasher};
    for g in &points {
        g.position.map(f32::to_bits).hash(&mut h);
        g.color.hash(&mut h);
    }
    Some((h.finish() | 1 << 61, Arc::new(points)))
}
