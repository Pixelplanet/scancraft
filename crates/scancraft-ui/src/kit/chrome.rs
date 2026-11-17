//! Card frames, headers, disclosure arrows and icon buttons.
//!
//! Ported from SolidCraft `crates/ui-egui/src/ui_kit/container.rs` (`Container::frame` for the
//! floating Panel/Feature/Navigation/ViewportStatus surfaces), `ui_kit/panel.rs`
//! (`panel_header_hover`, `header_actions`), `ui_kit/disclosure.rs`, `ui_kit/actions.rs`
//! (`icon_button_presentation`) and `browser.rs` (`browser_header`, `CARD_INSET`, `HEADER_H`).

use crate::fonts;
use crate::icons;
use crate::theme::Tokens;
use egui::{Align2, Color32, FontId, Frame, Painter, Pos2, Rect, Response, Sense, Ui, pos2, vec2};

/// The gap from the viewport edge to the floating cards and between the cards (`SHELL_GAP`).
pub const SHELL_GAP: f32 = 7.0;
/// The floating browser card's inner margin (`browser::CARD_INSET`).
pub const CARD_INSET: f32 = 4.0;
/// The browser card's header row (`browser::HEADER_H`).
pub const HEADER_H: f32 = 26.0;
/// Application bar height (`toolbar::APP_BAR_HEIGHT`).
pub const APP_BAR_HEIGHT: f32 = 34.0;
/// Status bar height (`status_bar::HEIGHT`).
pub const STATUS_BAR_HEIGHT: f32 = 24.0;

/// Which floating surface a frame is for (`ui_kit::Surface`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Surface {
    /// Browser, ribbon and workspace cards.
    Panel,
    /// Feature dialogs (the right-hand properties card).
    Feature,
    /// The viewport navigation bar.
    Navigation,
    /// The pill over the viewport saying what is running.
    ViewportStatus,
    /// Tooltips and toasts.
    Toast,
}

/// Tokenised outer chrome of a floating surface (`Container::frame`).
pub fn frame(surface: Surface) -> Frame {
    let t = Tokens::get();
    let flat = Frame::NONE.fill(t.panel);
    match surface {
        Surface::Feature => flat.fill(t.dialog_body).stroke(t.primary_border(t.dialog_border)).corner_radius(t.radius_xl).shadow(t.shell_shadow()),
        Surface::Panel => flat.stroke(t.primary_border(t.border)).corner_radius(t.radius_lg).shadow(t.shell_shadow()),
        Surface::Navigation => flat.fill(t.dialog_body.gamma_multiply(0.97)).stroke(t.primary_border(t.dialog_border)).corner_radius(10),
        Surface::ViewportStatus => flat.fill(t.overlay).stroke(t.primary_border(t.border)).corner_radius((t.control_height / 2.0 + t.space_4) as u8),
        Surface::Toast => flat.stroke(t.primary_border(t.border)).corner_radius(t.radius_md).shadow(t.shell_shadow()),
    }
}

/// Paint a surface's frame into an already-placed rect.
pub fn paint_frame(painter: &Painter, surface: Surface, rect: Rect) {
    painter.add(frame(surface).paint(rect));
}

/// The soft accent behind a hovered card header (`panel_header_hover`).
pub fn panel_header_hover(painter: &Painter, rect: Rect) {
    let t = Tokens::get();
    painter.rect_filled(rect, t.radius_lg.saturating_sub(t.space_4 as u8), t.accent_soft);
}

/// Content visible: down. Content hidden: right (`ui_kit::disclosure`).
pub fn disclosure(painter: &Painter, center: Pos2, open: bool, ink: Color32) {
    let size = Tokens::get().form_section_icon_size;
    let rect = Rect::from_center_size(center, vec2(size, size));
    icons::paint_ink(painter, rect, if open { "chevron" } else { "chevron_right" }, ink);
}

/// Dropdown chevron (always down).
pub fn dropdown_indicator(painter: &Painter, rect: Rect, ink: Color32) {
    icons::paint_ink(painter, rect, "chevron", ink);
}

/// A square icon button: hover fill, ink icon, tooltip (`icon_button_presentation`).
pub fn icon_button_at(ui: &mut Ui, rect: Rect, icon: &str, label: &str, enabled: bool) -> Response {
    let t = Tokens::get();
    let response = ui.interact(rect, ui.make_persistent_id(("icon_button", icon, label)), if enabled { Sense::click() } else { Sense::hover() });
    if enabled && response.hovered() {
        ui.painter().rect_filled(rect, t.control_radius, t.hover);
    }
    let ink = if !enabled {
        t.form_dim.gamma_multiply(0.45)
    } else if response.hovered() || response.has_focus() {
        t.form_ink
    } else {
        t.form_dim
    };
    icons::paint_ink(ui.painter(), rect.shrink(t.form_icon_inset), icon, ink);
    response.widget_info(|| egui::WidgetInfo::labeled(egui::WidgetType::Button, enabled, label));
    if enabled {
        response.on_hover_cursor(egui::CursorIcon::PointingHand).on_hover_text(label)
    } else {
        response.on_hover_text(label)
    }
}

/// The browser card's title row: semibold title at the left, a dim count at the right.
pub fn browser_header(ui: &mut Ui, title: &str, count: &str) -> Response {
    let t = Tokens::get();
    let (rect, _) = ui.allocate_exact_size(vec2(ui.available_width(), HEADER_H), Sense::hover());
    let response = ui.interact(rect, ui.id().with(("browser_header", title)), Sense::click());
    if response.hovered() {
        panel_header_hover(ui.painter(), rect);
    }
    let galley = ui.painter().layout_no_wrap(title.to_string(), fonts::semibold(t.font_heading), t.text);
    ui.painter().galley(pos2(rect.left() + t.space_6, rect.center().y - galley.size().y / 2.0), galley, t.text);
    let right = rect.right() - t.space_6;
    let galley = ui.painter().layout_no_wrap(count.to_string(), FontId::proportional(t.font_body), t.text_dim);
    ui.painter().galley(pos2(right - galley.size().x, rect.center().y - galley.size().y / 2.0), galley, t.text_dim);
    response
}

/// A feature card's title band: 12 pt inset, 19 pt icon, semibold title, close target and a
/// rule below (`header_actions`). Returns whether close was clicked.
pub fn dialog_header(ui: &mut Ui, icon: &str, title: &str, closeable: bool) -> bool {
    let t = Tokens::get();
    let (rect, _) = ui.allocate_exact_size(vec2(ui.available_width(), t.form_header_height), Sense::hover());
    let icon_rect = Rect::from_center_size(pos2(rect.left() + t.space_12 + t.form_icon_size / 2.0, rect.center().y), vec2(t.form_icon_size, t.form_icon_size));
    if icon.is_empty() {
        ui.painter().circle_filled(icon_rect.center(), t.space_3, t.selected_ink);
    } else {
        icons::paint_icon(ui.painter(), icon_rect, icon, 1.0);
    }
    let close_rect =
        Rect::from_center_size(pos2((rect.right() - t.space_8 - t.form_close_size / 2.0).max(rect.left()), rect.center().y), vec2(t.form_close_size, t.form_close_size));
    let mut closed = false;
    let right = if closeable {
        closed = icon_button_at(ui, close_rect, "close", "Close", true).clicked();
        close_rect.left()
    } else {
        rect.right() - t.space_8
    };
    let left = (icon_rect.right() + t.space_8).min(right);
    let title_rect = Rect::from_min_max(pos2(left, rect.top()), pos2(right.max(left), rect.bottom()));
    paint_text(ui.painter(), title_rect, title, fonts::semibold(if icon.is_empty() { t.font_label } else { t.font_heading }), t.form_ink);
    t.secondary_hline(ui.painter(), rect.x_range(), rect.bottom(), t.dialog_rule);
    closed
}

/// One line of text, vertically centred and cut with an ellipsis to its rect (`text::paint_text`).
pub fn paint_text(painter: &Painter, rect: Rect, text: &str, font: FontId, color: Color32) {
    let mut job = egui::text::LayoutJob::simple(text.to_string(), font, color, rect.width().max(0.0));
    job.wrap.max_rows = 1;
    job.wrap.break_anywhere = true;
    job.wrap.overflow_character = Some('\u{2026}');
    let galley = painter.layout_job(job);
    painter.with_clip_rect(painter.clip_rect().intersect(rect.expand(1.0))).galley(pos2(rect.left(), rect.center().y - galley.size().y / 2.0), galley, color);
}

/// Centred text pixel-snapped at `at` (`toolbar::ribbon_galley`).
pub fn centred_text(painter: &Painter, at: Pos2, text: &str, font: FontId, color: Color32) -> Rect {
    let galley = painter.layout_no_wrap(text.to_string(), font, color);
    let origin = at - galley.size() / 2.0;
    let ppp = painter.ctx().pixels_per_point().max(0.5);
    let origin = pos2((origin.x * ppp).round() / ppp, (origin.y * ppp).round() / ppp);
    let rect = Rect::from_min_size(origin, galley.size());
    painter.galley(origin, galley, color);
    rect
}

/// The application mark: three faces of an isometric cube in the logo's blues, drawn in code
/// (SolidCraft draws its own mark the same way in `icons::glyphs::brand_mark`).
pub fn brand_mark(painter: &Painter, center: Pos2, size: f32) {
    let t = Tokens::get();
    let r = size / 2.0;
    let c = center;
    let h = r * 0.866;
    let top = pos2(c.x, c.y - r);
    let ul = pos2(c.x - h, c.y - r / 2.0);
    let ur = pos2(c.x + h, c.y - r / 2.0);
    let ll = pos2(c.x - h, c.y + r / 2.0);
    let lr = pos2(c.x + h, c.y + r / 2.0);
    let bottom = pos2(c.x, c.y + r);
    let stroke = egui::Stroke::NONE;
    painter.add(egui::Shape::convex_polygon(vec![top, ur, c, ul], t.logo[0], stroke));
    painter.add(egui::Shape::convex_polygon(vec![ul, c, bottom, ll], t.logo[1], stroke));
    painter.add(egui::Shape::convex_polygon(vec![c, ur, lr, bottom], t.logo[2], stroke));
}

/// A vertical separator in the application bar (`toolbar::app_bar_separator`).
pub fn app_bar_separator(painter: &Painter, bar: Rect, x: f32) {
    let t = Tokens::get();
    painter.vline(x, (bar.top() + 9.0)..=(bar.bottom() - 9.0), egui::Stroke::new(1.0, t.app_bar.lerp_to_gamma(t.app_bar_separator, 0.4)));
}

/// A flat icon button in the application bar (home, save, undo, redo).
pub fn app_bar_button(ui: &mut Ui, rect: Rect, icon: &str, label: &str, enabled: bool) -> Response {
    let t = Tokens::get();
    let response = ui.interact(rect, ui.make_persistent_id(("appbar", icon)), if enabled { Sense::click() } else { Sense::hover() });
    if enabled && (response.hovered() || response.has_focus()) {
        ui.painter().rect_filled(rect.shrink(3.0), t.radius_sm, t.app_bar_hover_button);
    }
    icons::paint_ink(ui.painter(), rect.shrink(8.0), icon, t.app_bar_text.gamma_multiply(if enabled { 1.0 } else { 0.35 }));
    response.widget_info(|| egui::WidgetInfo::labeled(egui::WidgetType::Button, enabled, label));
    response.on_hover_text(label)
}

/// A status pill in the application bar: a coloured dot and a label (SolidCraft's
/// "Agent connected" chip in the bar).
pub fn app_bar_pill(ui: &mut Ui, right: f32, bar: Rect, dot: Color32, text: &str) -> Response {
    let t = Tokens::get();
    let galley = ui.painter().layout_no_wrap(text.to_string(), fonts::medium(t.font_body), t.app_bar_text);
    let width = t.space_10 + t.space_8 + t.space_6 + galley.size().x + t.space_10;
    let rect = Rect::from_min_max(pos2(right - width, bar.center().y - 11.0), pos2(right, bar.center().y + 11.0));
    let response = ui.interact(rect, ui.id().with("appbar_pill"), Sense::click());
    let fill = if response.hovered() { t.app_bar_hover_button } else { Color32::TRANSPARENT };
    ui.painter().rect(rect, 11, fill, egui::Stroke::new(1.0, t.app_bar.lerp_to_gamma(t.app_bar_separator, 0.35)), egui::StrokeKind::Inside);
    ui.painter().circle_filled(pos2(rect.left() + t.space_10 + t.space_4, rect.center().y), t.space_4, dot);
    ui.painter().galley(pos2(rect.left() + t.space_10 + t.space_8 + t.space_6, rect.center().y - galley.size().y / 2.0), galley, t.app_bar_text);
    response
}

/// A one-line message over the viewport in its rounded pill (`Surface::ViewportStatus`).
pub fn viewport_status(painter: &Painter, center_bottom: Pos2, dot: Option<Color32>, text: &str) -> Rect {
    let t = Tokens::get();
    let galley = painter.layout_no_wrap(text.to_string(), fonts::medium(t.font_body), t.text);
    let dot_w = if dot.is_some() { t.space_8 + t.space_6 } else { 0.0 };
    let size = vec2(galley.size().x + dot_w + 2.0 * t.space_12, t.control_height);
    let rect = Rect::from_min_size(pos2(center_bottom.x - size.x / 2.0, center_bottom.y - size.y), size);
    paint_frame(painter, Surface::ViewportStatus, rect);
    let mut x = rect.left() + t.space_12;
    if let Some(dot) = dot {
        painter.circle_filled(pos2(x + t.space_4, rect.center().y), t.space_4, dot);
        x += dot_w;
    }
    painter.galley(pos2(x, rect.center().y - galley.size().y / 2.0), galley, t.text);
    rect
}

/// Status bar item helper: a dot (optional), a label, right-aligned against `right`; returns the
/// item's left edge. A rule separates it from the item on its right (`status_bar::paint`).
pub fn status_item(painter: &Painter, bar: Rect, right: f32, dot: Option<Color32>, text: &str, color: Color32, rule: bool) -> f32 {
    let t = Tokens::get();
    let galley = painter.layout_no_wrap(text.to_string(), FontId::proportional(t.font_body), color);
    let dot_width = if dot.is_some() { t.space_8 + t.space_6 } else { 0.0 };
    let width = dot_width + galley.size().x;
    let mut right = right;
    if rule {
        let x = right - t.space_6;
        t.secondary_vline(painter, x, (bar.top() + t.space_6)..=(bar.bottom() - t.space_6), t.border);
        right = x - t.space_6;
    }
    let item = Rect::from_min_max(pos2(right - width, bar.top()), pos2(right, bar.bottom()));
    if let Some(dot) = dot {
        painter.circle_filled(pos2(item.left() + t.space_4, item.center().y), t.space_3 + 0.5, dot);
    }
    painter.galley(pos2(item.left() + dot_width, item.center().y - galley.size().y / 2.0), galley, color);
    item.left()
}

/// Left-aligned status bar message.
pub fn status_message(painter: &Painter, bar: Rect, max_right: f32, text: &str, color: Color32) {
    let t = Tokens::get();
    let rect = Rect::from_min_max(pos2(bar.left() + t.space_8, bar.top()), pos2(max_right - t.space_8, bar.bottom()));
    paint_text(painter, rect, text, FontId::proportional(t.font_body), color);
}

/// Text anchored at `pos` with the given alignment in the theme's body font.
pub fn label_at(painter: &Painter, pos: Pos2, align: Align2, text: &str, size: f32, color: Color32) -> Rect {
    painter.text(pos, align, text, FontId::proportional(size), color)
}
