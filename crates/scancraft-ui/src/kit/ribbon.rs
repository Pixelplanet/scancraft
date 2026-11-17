//! The ribbon: the workspace card and the tab/group card in SolidCraft's floating layout.
//!
//! Ported from SolidCraft `crates/ui-egui/src/toolbar.rs`: the constants (`TAB_ROW_HEIGHT`,
//! `GROUP_HEIGHT`, `WORKSPACE_WIDTH`, `BUTTON_*`, `SECTION_HEIGHT`, `RIBBON_TAB_INSET`),
//! `ribbon_tabs` / `ribbon_tab_style`, the button painting in `toolbar()` (hover fill, pressed
//! soft-accent with outline, 26 pt icon centred 16 pt below the top, 12 pt label at 32 pt),
//! `ribbon_button_width` / `ribbon_label_lines`, the section label row with its chevron, and the
//! panel separators; and `shell.rs` (`Cards::new`).

use super::chrome::{SHELL_GAP, Surface, centred_text, paint_frame};
use crate::fonts;
use crate::icons;
use crate::theme::Tokens;
use egui::{Color32, FontId, Painter, Rect, Response, Sense, Stroke, Ui, pos2, vec2};

pub const TAB_ROW_HEIGHT: f32 = 30.0;
const RIBBON_TAB_INSET: f32 = 4.0;
pub const GROUP_HEIGHT: f32 = 86.0;
pub const RIBBON_HEIGHT: f32 = TAB_ROW_HEIGHT + GROUP_HEIGHT + 1.0;
pub const WORKSPACE_WIDTH: f32 = 88.0;
const BUTTON_HEIGHT: f32 = 60.0;
const BUTTON_GAP: f32 = 3.0;
const BUTTON_MIN_WIDTH: f32 = 54.0;
const BUTTON_MAX_WIDTH: f32 = 76.0;
const BUTTON_LABEL_PADDING: f32 = 10.0;
const BUTTON_LABEL_TOP: f32 = 32.0;
const BUTTON_LABEL_BOTTOM: f32 = 2.0;
const SECTION_HEIGHT: f32 = 20.0;
const COLLAPSED_WIDTH: f32 = 72.0;
const PANEL_GAP: f32 = 1.0;

/// The workspace card and the ribbon card, `SHELL_GAP` from the viewport's edges (`Cards::new`).
#[derive(Clone, Copy, Debug)]
pub struct Cards {
    pub workspace: Rect,
    pub ribbon: Rect,
    /// Where the content under the ribbon starts.
    pub bottom: f32,
}

impl Cards {
    pub fn new(available: Rect) -> Self {
        let margin = SHELL_GAP;
        let left = available.left() + margin;
        let top = available.top() + margin;
        let right = (available.right() - margin).max(left);
        let workspace = Rect::from_min_size(pos2(left, top), vec2(WORKSPACE_WIDTH.min((right - left).max(0.0)), RIBBON_HEIGHT));
        let ribbon = Rect::from_min_max(pos2((workspace.right() + margin).min(right), top), pos2(right, top + RIBBON_HEIGHT));
        Self { workspace, ribbon, bottom: ribbon.bottom() + margin }
    }
}

/// One command button.
#[derive(Clone, Copy, Debug)]
pub struct RibbonButton<'a> {
    pub id: &'a str,
    pub icon: &'a str,
    pub label: &'a str,
    pub enabled: bool,
    /// Its tool or dialog is open (or its mode is on): soft accent with an accent outline.
    pub pressed: bool,
    /// Hover text; for a disabled button, why it is disabled.
    pub tip: &'a str,
}

/// A labelled panel of buttons.
#[derive(Clone, Debug)]
pub struct RibbonGroup<'a> {
    pub label: &'a str,
    pub buttons: Vec<RibbonButton<'a>>,
}

/// The workspace switcher card: an accent bar on top, a 30 pt icon, a semibold name and a dim
/// caption (SolidCraft's Design / Workspace card).
pub fn workspace_card(ui: &mut Ui, rect: Rect, icon: &str, title: &str, caption: &str, tip: &str) -> Response {
    let t = Tokens::get();
    paint_frame(ui.painter(), Surface::Panel, rect);
    let response = ui.interact(rect, ui.id().with("workspace_card"), Sense::click());
    if response.hovered() {
        ui.painter().rect_filled(rect, t.radius_lg, t.hover);
    }
    let inset = f32::from(t.radius_lg);
    ui.painter().rect_filled(
        Rect::from_min_size(rect.left_top() + vec2(inset, 0.0), vec2((rect.width() - 2.0 * inset).max(0.0), t.space_3)),
        egui::CornerRadius { nw: 0, ne: 0, sw: t.radius_sm / 2, se: t.radius_sm / 2 },
        t.accent,
    );
    icons::paint_icon(ui.painter(), Rect::from_center_size(pos2(rect.center().x, rect.top() + 39.0), vec2(30.0, 30.0)), icon, 1.0);
    centred_text(ui.painter(), pos2(rect.center().x, rect.top() + 69.0), title, fonts::semibold(t.font_button), t.text);
    let mut job = egui::text::LayoutJob::simple(caption.to_string(), FontId::proportional(t.font_small), t.text_dim, rect.width() - 2.0 * t.space_8);
    job.wrap.max_rows = 1;
    job.wrap.overflow_character = Some('\u{2026}');
    job.halign = egui::Align::Center;
    let galley = ui.painter().layout_job(job);
    ui.painter().galley(pos2(rect.center().x, rect.top() + 89.0 - galley.size().y / 2.0), galley, t.text_dim);
    response.on_hover_text(tip)
}

fn label_width(painter: &Painter, text: &str) -> f32 {
    painter.layout_no_wrap(text.to_owned(), FontId::proportional(Tokens::get().font_body), Color32::WHITE).size().x
}

/// A label broken into at most two lines no wider than `width` (`ribbon_label_lines`).
fn label_lines(painter: &Painter, label: &str, width: f32) -> Vec<String> {
    let normalized = label.split_whitespace().collect::<Vec<_>>().join(" ");
    let label = normalized.as_str();
    let mut end = 0;
    for (at, ch) in label.char_indices() {
        let next = at + ch.len_utf8();
        let is_break = next == label.len() || ch.is_whitespace() || ch == '/';
        if !is_break {
            continue;
        }
        if end > 0 && label_width(painter, label.get(..next).unwrap_or_default().trim()) > width {
            break;
        }
        end = next;
    }
    let (first, rest) = label.split_at(end);
    let mut lines = vec![first.trim().to_owned()];
    if !rest.trim().is_empty() {
        lines.push(rest.trim().to_owned());
    }
    lines
}

/// A button's width from its label (`ribbon_button_width`).
fn button_width(painter: &Painter, label: &str) -> f32 {
    let widest = label_lines(painter, label, BUTTON_MAX_WIDTH - BUTTON_LABEL_PADDING).iter().map(|l| label_width(painter, l)).fold(0.0_f32, f32::max);
    (widest + BUTTON_LABEL_PADDING).clamp(BUTTON_MIN_WIDTH, BUTTON_MAX_WIDTH)
}

/// A panel's width from its buttons (`panel_width`).
fn panel_width(buttons: &[f32]) -> f32 {
    (buttons.iter().sum::<f32>() + buttons.len().saturating_sub(1) as f32 * BUTTON_GAP + 8.0).max(COLLAPSED_WIDTH)
}

fn button_galley(painter: &Painter, label: &str, width: f32, color: Color32) -> std::sync::Arc<egui::Galley> {
    let lines = label_lines(painter, label, width - BUTTON_LABEL_PADDING / 2.0);
    let size = Tokens::get().font_body;
    let mut job = egui::text::LayoutJob::simple(lines.join("\n"), FontId::proportional(size), color, f32::INFINITY);
    job.halign = egui::Align::Center;
    // Two lines sit closer than the font's own spacing (SolidCraft #8391); one line uses the same
    // line height so every label's first line starts at the same place.
    if let Some(section) = job.sections.first_mut() {
        section.format.line_height = Some(size + 2.0);
    }
    painter.layout_job(job)
}

/// The ribbon card: tab row, then the active tab's panels. Returns the clicked button's id.
pub fn ribbon_card<'a>(ui: &mut Ui, card: Rect, tabs: &[&str], active: &mut usize, groups: &[RibbonGroup<'a>]) -> Option<&'a str> {
    let t = Tokens::get();
    paint_frame(ui.painter(), Surface::Panel, card);
    // Blank card space owns clicks so they never reach the viewport under it.
    ui.interact(card, ui.id().with("ribbon_card_bg"), Sense::click_and_drag());
    let row = Rect::from_min_max(card.left_top(), pos2(card.right(), card.top() + TAB_ROW_HEIGHT));
    ribbon_tabs(ui, row, tabs, active);
    let painter = ui.painter().with_clip_rect(card);
    let mut clicked = None;
    let top = card.top() + TAB_ROW_HEIGHT;
    let mut px = card.left();
    for group in groups {
        let widths: Vec<f32> = group.buttons.iter().map(|b| button_width(&painter, b.label)).collect();
        let width = panel_width(&widths);
        if px + width > card.right() {
            break;
        }
        let buttons_width = widths.iter().sum::<f32>() + widths.len().saturating_sub(1) as f32 * BUTTON_GAP;
        let inset = ((width - 8.0 - buttons_width) / 2.0).max(0.0).floor();
        let mut bx = px + 4.0 + inset;
        for (b, w) in group.buttons.iter().zip(&widths) {
            let whole = Rect::from_min_size(pos2(bx, top + 2.0), vec2(*w, BUTTON_HEIGHT));
            bx += w + BUTTON_GAP;
            let resp = ui.interact(whole, ui.id().with(("ribbon_cmd", b.id)), if b.enabled { Sense::click() } else { Sense::hover() });
            if b.enabled && (resp.is_pointer_button_down_on() || b.pressed) {
                painter.rect(whole, t.radius_sm, t.accent_soft, Stroke::new(1.0, t.accent), egui::StrokeKind::Inside);
            } else if resp.hovered() && b.enabled {
                painter.rect_filled(whole, t.radius_sm, t.hover);
            }
            let icon_rect = Rect::from_center_size(pos2(whole.center().x, whole.top() + 16.0), vec2(26.0, 26.0));
            icons::paint_icon(&painter, icon_rect, b.icon, if b.enabled { 1.0 } else { 0.35 });
            let ink = if b.enabled { t.text } else { t.text_dim.gamma_multiply(0.75) };
            let galley = button_galley(&painter, b.label, *w, ink);
            let label_top = (whole.top() + BUTTON_LABEL_TOP).min(whole.bottom() - BUTTON_LABEL_BOTTOM - galley.size().y);
            let ppp = ui.ctx().pixels_per_point();
            let x = ((whole.center().x) * ppp).round() / ppp;
            painter.galley(pos2(x, (label_top * ppp).round() / ppp), galley, ink);
            resp.widget_info(|| egui::WidgetInfo::labeled(egui::WidgetType::Button, b.enabled, b.label));
            let tip = if b.tip.is_empty() { b.label } else { b.tip };
            let resp = if b.enabled { resp.on_hover_cursor(egui::CursorIcon::PointingHand).on_hover_text(tip) } else { resp.on_hover_text(tip) };
            if resp.clicked() && b.enabled {
                clicked = Some(b.id);
            }
        }
        // The group's label row, RIBBON_TAB_INSET from the card's bottom edge.
        let lr = Rect::from_min_size(
            pos2(px + RIBBON_TAB_INSET, top + GROUP_HEIGHT - SECTION_HEIGHT - (RIBBON_TAB_INSET - 1.0)),
            vec2((width - 2.0 * RIBBON_TAB_INSET).max(0.0), SECTION_HEIGHT),
        );
        // SolidCraft's label row also opens the panel's command list; ScanCraft panels show all
        // their commands, so the label is a caption without the drop-down chevron.
        centred_text(&painter, lr.center(), group.label, FontId::proportional(t.font_caption), t.text_dim);
        px += width;
        // The separator after each panel.
        painter.vline(px, (top + 6.0)..=(top + GROUP_HEIGHT - 6.0), t.control_border_stroke(t.border.lerp_to_gamma(t.toolbar, 0.25)));
        px += PANEL_GAP;
    }
    clicked
}

/// The tab row (`ribbon_tabs`): semibold 13 pt labels, 24 pt padding, the active tab in the soft
/// accent with accent ink, a hover fill, a rule under the row.
fn ribbon_tabs(ui: &mut Ui, row: Rect, tabs: &[&str], active: &mut usize) {
    let t = Tokens::get();
    let inset = RIBBON_TAB_INSET;
    let corner = t.radius_lg.saturating_sub(inset as u8).max(t.radius_sm / 2);
    let mut x = row.left() + inset;
    for (index, tab) in tabs.iter().enumerate() {
        let width = ui.painter().layout_no_wrap(tab.to_string(), fonts::semibold(t.font_button), t.text).size().x + 24.0;
        let tr = Rect::from_min_size(pos2(x, row.top() + inset), vec2(width, TAB_ROW_HEIGHT - 2.0 * inset));
        let response = ui.interact(tr, ui.id().with(("ribbon_tab", index)), Sense::click());
        let is_active = *active == index;
        let (fill, ink) = if is_active {
            (t.accent_soft, t.selected_ink)
        } else if response.hovered() {
            (t.hover, t.text)
        } else {
            (Color32::TRANSPARENT, t.text)
        };
        ui.painter().rect_filled(tr, corner, fill);
        let font = if is_active { fonts::semibold(t.font_button) } else { FontId::proportional(t.font_button) };
        centred_text(ui.painter(), tr.center(), tab, font, ink);
        response.widget_info(|| egui::WidgetInfo::selected(egui::WidgetType::Button, true, is_active, *tab));
        if response.on_hover_cursor(egui::CursorIcon::PointingHand).clicked() {
            *active = index;
        }
        x += width + 2.0;
    }
    t.secondary_hline(ui.painter(), (row.left())..=(row.right()), row.bottom() - 0.5, t.toolbar.lerp_to_gamma(t.border, 0.5));
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cards_sit_a_shell_gap_from_the_view_and_each_other() {
        let cards = Cards::new(Rect::from_min_size(pos2(0.0, 34.0), vec2(1200.0, 700.0)));
        assert_eq!(cards.workspace.min, pos2(SHELL_GAP, 34.0 + SHELL_GAP));
        assert_eq!(cards.workspace.width(), WORKSPACE_WIDTH);
        assert_eq!(cards.ribbon.left(), cards.workspace.right() + SHELL_GAP);
        assert_eq!(cards.ribbon.right(), 1200.0 - SHELL_GAP);
        assert_eq!(cards.ribbon.height(), RIBBON_HEIGHT);
    }

    #[test]
    fn panel_width_has_a_floor() {
        assert_eq!(panel_width(&[54.0]), COLLAPSED_WIDTH);
        assert_eq!(panel_width(&[60.0, 60.0]), 60.0 + 60.0 + BUTTON_GAP + 8.0);
    }
}
