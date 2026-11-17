//! Viewport navigation: the bar at the bottom of the view and the axis triad at its top right.
//!
//! The bar is ported from SolidCraft `crates/ui-egui/src/ui_kit/navigation.rs` (the `NAV_*`
//! metrics, `NavigationMetrics::for_size(Regular)`, `navigation_visual`, `navigation_fill`,
//! `navigation_ink`, `navigation_button`, `navigation_separator`, `navigation_surface`) and
//! `navigation_bar_view.rs` (`measure_row`: centred, `NAV_BOTTOM` above the view's bottom).

use super::chrome::{Surface, paint_frame};
use crate::icons;
use crate::theme::{Tokens, contrast_ratio};
use egui::{Color32, FontId, Pos2, Rect, Response, Sense, Ui, pos2, vec2};
use glam::{Mat4, Vec3};

pub const NAV_BORDER: f32 = 1.0;
pub const NAV_PADDING: f32 = 3.0;
pub const NAV_GAP: f32 = 2.0;
pub const NAV_BOTTOM: f32 = 7.0;
pub const NAV_SEPARATOR: f32 = 9.0;
pub const NAV_SEPARATOR_HEIGHT: f32 = 18.0;
const BUTTON: f32 = 32.0;
const ICON: f32 = 18.0;
const BUTTON_RADIUS: u8 = 7;

/// How a slot behaves.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Kind {
    /// A latching mode (orbit, pan, zoom with the left button).
    Tool,
    /// A latching on/off option.
    Toggle,
    /// A one-shot action (fit, home).
    Action,
    Separator,
}

#[derive(Clone, Copy, Debug)]
pub struct NavButton<'a> {
    pub id: &'a str,
    pub icon: &'a str,
    pub label: &'a str,
    pub kind: Kind,
    pub selected: bool,
    pub enabled: bool,
}

impl<'a> NavButton<'a> {
    pub const fn separator() -> Self {
        NavButton { id: "", icon: "", label: "", kind: Kind::Separator, selected: false, enabled: true }
    }
}

/// Bar height (`NavigationMetrics::height`).
pub fn bar_height() -> f32 {
    BUTTON + 2.0 * (NAV_BORDER + NAV_PADDING)
}

fn slot_width(b: &NavButton) -> f32 {
    if b.kind == Kind::Separator { NAV_SEPARATOR } else { BUTTON }
}

/// Where the bar sits in `content`: centred, `NAV_BOTTOM` above its bottom edge.
pub fn bar_rect(content: Rect, buttons: &[NavButton]) -> Rect {
    let inset = NAV_BORDER + NAV_PADDING;
    let width = buttons.iter().map(slot_width).sum::<f32>() + NAV_GAP * buttons.len().saturating_sub(1) as f32 + 2.0 * inset;
    Rect::from_min_size(pos2(content.center().x - width / 2.0, content.bottom() - bar_height() - NAV_BOTTOM), vec2(width, bar_height()))
}

/// Draw the bar; returns the id of the clicked button.
pub fn nav_bar<'a>(ui: &mut Ui, content: Rect, buttons: &[NavButton<'a>]) -> (Rect, Option<&'a str>) {
    let t = Tokens::get();
    let rect = bar_rect(content, buttons);
    paint_frame(ui.painter(), Surface::Navigation, rect.shrink(NAV_BORDER));
    ui.interact(rect, ui.id().with("nav_bar_bg"), Sense::click_and_drag());
    let mut clicked = None;
    let mut x = rect.left() + NAV_BORDER + NAV_PADDING;
    for b in buttons {
        let w = slot_width(b);
        let bounds = Rect::from_min_size(pos2(x, rect.top() + NAV_BORDER + NAV_PADDING), vec2(w, BUTTON));
        x += w + NAV_GAP;
        if b.kind == Kind::Separator {
            ui.painter().vline(
                bounds.center().x,
                bounds.center().y - NAV_SEPARATOR_HEIGHT / 2.0..=bounds.center().y + NAV_SEPARATOR_HEIGHT / 2.0,
                egui::Stroke::new(NAV_BORDER, t.control_border),
            );
            continue;
        }
        let response = ui.interact(bounds, ui.id().with(("nav_slot", b.id)), if b.enabled { Sense::click() } else { Sense::hover() });
        let pressed = response.is_pointer_button_down_on();
        let fill = if pressed {
            Some(t.nav_press)
        } else if b.selected && matches!(b.kind, Kind::Tool | Kind::Toggle) {
            Some(t.selected_bg)
        } else if response.hovered() && b.enabled {
            Some(t.nav_hover)
        } else {
            None
        };
        if let Some(fill) = fill {
            ui.painter().rect_filled(bounds, BUTTON_RADIUS, fill);
        }
        let active = b.selected && !pressed && matches!(b.kind, Kind::Tool | Kind::Toggle);
        let ink = if !b.enabled {
            t.form_ink.gamma_multiply(0.35)
        } else if active {
            if contrast_ratio(t.form_primary, t.selected_bg) >= 4.5 { t.form_primary } else { t.selected_ink }
        } else {
            t.form_ink
        };
        icons::paint_ink(ui.painter(), Rect::from_center_size(bounds.center(), vec2(ICON, ICON)), b.icon, ink);
        response.widget_info(|| egui::WidgetInfo::selected(egui::WidgetType::Button, b.enabled, b.selected, b.label));
        let response = if b.enabled { response.on_hover_cursor(egui::CursorIcon::PointingHand).on_hover_text(b.label) } else { response.on_hover_text(b.label) };
        if response.clicked() && b.enabled {
            clicked = Some(b.id);
        }
    }
    (rect, clicked)
}

/// Axis the triad was clicked on: the view looks along it towards the origin.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TriadPick {
    PosX,
    NegX,
    PosY,
    NegY,
    PosZ,
    NegZ,
    Home,
}

/// The orientation triad: a soft disc, the three axes from its centre to lettered discs for the
/// positive directions and rings for the negative ones, ordered back to front; a home button
/// above it at the left (the look of SolidCraft's navigation sphere in its dark theme).
pub fn triad(ui: &mut Ui, center: Pos2, radius: f32, view: Mat4) -> Option<TriadPick> {
    let t = Tokens::get();
    let painter = ui.painter().clone();
    let bg = Rect::from_center_size(center, vec2(2.0 * radius, 2.0 * radius));
    let hovered_bg = ui.rect_contains_pointer(bg);
    painter.circle_filled(center, radius, if hovered_bg { t.nav_viewport.gamma_multiply(0.95) } else { t.nav_viewport.gamma_multiply(0.7) });
    let axes = [
        (Vec3::X, t.axis_x, "X", TriadPick::PosX, TriadPick::NegX),
        (Vec3::Y, t.axis_y, "Y", TriadPick::PosY, TriadPick::NegY),
        (Vec3::Z, t.axis_z, "Z", TriadPick::PosZ, TriadPick::NegZ),
    ];
    let arm = radius * 0.62;
    let disc = radius * 0.2;
    let mut ends = Vec::new();
    for (dir, colour, letter, pos, neg) in axes {
        let v = view.transform_vector3(dir);
        let p = center + vec2(v.x, -v.y) * arm;
        let n = center - vec2(v.x, -v.y) * arm;
        ends.push((v.z, p, colour, Some(letter), pos));
        ends.push((-v.z, n, colour, None, neg));
    }
    ends.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
    let pointer = ui.input(|i| i.pointer.hover_pos());
    let mut hit = None;
    for (_, p, colour, letter, pick) in &ends {
        if pointer.is_some_and(|q| q.distance(*p) <= disc + 2.0) {
            hit = Some((*pick, *p));
        }
        let _ = colour;
        let _ = letter;
    }
    for (_, p, colour, letter, pick) in &ends {
        let hot = hit.is_some_and(|(h, _)| h == *pick);
        painter.line_segment([center, *p], egui::Stroke::new(if letter.is_some() { 2.0 } else { 1.5 }, colour.gamma_multiply(if letter.is_some() { 1.0 } else { 0.7 })));
        match letter {
            Some(letter) => {
                painter.circle_filled(*p, disc + if hot { 1.5 } else { 0.0 }, *colour);
                painter.text(*p, egui::Align2::CENTER_CENTER, *letter, crate::fonts::bold(t.font_label), Color32::WHITE);
            }
            None => {
                painter.circle(*p, disc * 0.62 + if hot { 1.5 } else { 0.0 }, t.nav_viewport, egui::Stroke::new(1.5, *colour));
            }
        }
    }
    painter.circle_filled(center, 2.0, t.origin_point);
    let response = ui.interact(bg, ui.id().with("triad"), Sense::click());
    let mut picked = None;
    if let Some((pick, _)) = hit {
        let response = response.clone().on_hover_cursor(egui::CursorIcon::PointingHand);
        if response.clicked() {
            picked = Some(pick);
        }
    }
    // Home button at the triad's top left (SolidCraft's view home control).
    let home = Rect::from_center_size(center + vec2(-radius * 0.82, -radius * 0.82), vec2(22.0, 22.0));
    let h = ui.interact(home, ui.id().with("triad_home"), Sense::click());
    painter.circle_filled(home.center(), 11.0, if h.hovered() { t.nav_hover } else { t.nav_viewport });
    icons::paint_ink(&painter, home.shrink(5.0), "home", t.form_ink);
    if h.on_hover_text("Home view").clicked() {
        picked = Some(TriadPick::Home);
    }
    picked
}

/// Distance read-out for live aiming: a slim vertical gauge with the recommended band shaded,
/// the current distance as a marker and its value below. `None` shows "—".
pub fn distance_gauge(ui: &Ui, rect: Rect, distance_mm: Option<f32>, range_mm: (f32, f32), recommended_mm: (f32, f32)) -> Response {
    let t = Tokens::get();
    let painter = ui.painter();
    let track = Rect::from_min_max(pos2(rect.center().x - 3.0, rect.top() + 4.0), pos2(rect.center().x + 3.0, rect.bottom() - 22.0));
    painter.rect_filled(track, 3, t.control_fill);
    let span = (range_mm.1 - range_mm.0).max(1.0);
    let y_of = |d: f32| track.top() + ((d - range_mm.0) / span).clamp(0.0, 1.0) * track.height();
    let band = Rect::from_min_max(pos2(track.left(), y_of(recommended_mm.0)), pos2(track.right(), y_of(recommended_mm.1)));
    painter.rect_filled(band, 3, t.sketch_soft.lerp_to_gamma(t.sketch_ink, 0.35));
    painter.rect_stroke(track, 3, egui::Stroke::new(1.0, t.control_border), egui::StrokeKind::Outside);
    let (text, colour) = match distance_mm {
        Some(d) => {
            let ok = d >= recommended_mm.0 && d <= recommended_mm.1;
            let colour = if ok { t.sketch_ink } else { t.warning };
            let y = y_of(d);
            painter.add(egui::Shape::convex_polygon(
                vec![pos2(track.left() - 2.0, y), pos2(track.left() - 8.0, y - 5.0), pos2(track.left() - 8.0, y + 5.0)],
                colour,
                egui::Stroke::NONE,
            ));
            painter.hline((track.left() - 2.0)..=(track.right() + 2.0), y, egui::Stroke::new(2.0, colour));
            (format!("{d:.0}"), colour)
        }
        None => ("—".to_string(), t.text_dim),
    };
    painter.text(pos2(rect.center().x, rect.bottom() - 15.0), egui::Align2::CENTER_CENTER, text, FontId::monospace(t.font_small), colour);
    painter.text(pos2(rect.center().x, rect.bottom() - 4.0), egui::Align2::CENTER_CENTER, "mm", FontId::proportional(t.font_micro), t.text_dim);
    ui.interact(rect, ui.id().with("distance_gauge"), Sense::hover())
}
