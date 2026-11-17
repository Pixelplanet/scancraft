//! Browser tree rows. Ported from SolidCraft `crates/ui-egui/src/ui_kit/tree.rs` (`tree_row`):
//! 26 pt rows, 14 pt indent, fold arrow, eye, 16 pt icon, label with ellipsis, trailing dim
//! count, the lit-branch bar, and the selection / hover fills.

use super::chrome::disclosure;
use crate::icons;
use crate::theme::Tokens;
use egui::{Color32, FontId, Pos2, Rect, Sense, pos2, vec2};

/// Row height (`ui_kit::history::TREE_ROW_HEIGHT`).
pub const TREE_ROW_HEIGHT: f32 = 26.0;
const INDENT: f32 = 14.0;
const DIM_ICON_OPACITY: f32 = 0.4;

/// How a row relates to the selection: a bar down its left edge.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Mark {
    #[default]
    None,
    /// A row the selected item is inside.
    Branch,
    /// The selected item's own row.
    Here,
}

/// One row's look.
#[derive(Default, Clone, Copy)]
pub struct Row<'a> {
    pub depth: usize,
    /// `Some(open)` for rows that fold.
    pub fold: Option<bool>,
    /// `Some(visible)` for rows with an eye.
    pub eye: Option<bool>,
    pub icon: &'a str,
    pub label: &'a str,
    pub selected: bool,
    pub dim: bool,
    pub mark: Mark,
    /// Dim trailing text (a count).
    pub count: &'a str,
    /// A status dot at the row's end (connection state, a running job).
    pub status: Option<Color32>,
    pub tip: Option<&'a str>,
}

/// What happened on a row.
#[derive(Default)]
pub struct RowResp {
    pub rect: Option<Rect>,
    pub clicked: bool,
    pub double: bool,
    pub secondary: Option<Pos2>,
    pub fold: bool,
    pub eye: bool,
    pub hovered: bool,
    /// The row's response (for a context menu).
    pub response: Option<egui::Response>,
}

pub fn tree_row(ui: &mut egui::Ui, id: egui::Id, row: &Row) -> RowResp {
    let t = Tokens::get();
    let (r, _) = ui.allocate_exact_size(vec2(ui.available_width(), TREE_ROW_HEIGHT), Sense::hover());
    if !ui.is_rect_visible(r) {
        return RowResp { rect: Some(r), ..Default::default() };
    }
    let resp = ui.interact(r, id, Sense::click());
    let mut out = RowResp { rect: Some(r), hovered: resp.hovered(), ..Default::default() };
    if row.selected {
        ui.painter().rect_filled(r, t.radius_sm.saturating_sub(1), t.accent_soft);
    } else if resp.hovered() || row.mark == Mark::Here {
        ui.painter().rect_filled(r, t.radius_sm.saturating_sub(1), t.hover);
    }
    let bar = match row.mark {
        Mark::None => None,
        Mark::Branch => Some((2.0, t.accent.gamma_multiply(0.4))),
        Mark::Here => Some((3.0, t.accent)),
    };
    if let Some((w, colour)) = bar {
        ui.painter().rect_filled(Rect::from_min_size(r.left_top() + vec2(0.0, t.space_2), vec2(w, r.height() - 4.0)), t.radius_sm / 4, colour);
    }
    let mut x = r.left() + 2.0 + row.depth as f32 * INDENT;
    let cy = r.center().y;
    let fold_r = Rect::from_min_max(pos2(x - 4.0, r.top()), pos2(x + 15.0, r.bottom()));
    let eye_r = Rect::from_center_size(pos2(x + 14.0 + 8.0, cy), vec2(t.space_16 + t.space_3, r.height()));
    let at = |p: Pos2| -> u8 {
        if row.fold.is_some() && fold_r.contains(p) {
            1
        } else if row.eye.is_some() && eye_r.contains(p) {
            2
        } else {
            0
        }
    };
    let click_on = resp.interact_pointer_pos().map_or(0, at);
    let hover_on = resp.hover_pos().map_or(0, at);
    if let Some(open) = row.fold {
        disclosure(ui.painter(), pos2(x + 6.0, cy), open, if hover_on == 1 { t.text } else { t.text_dim });
        out.fold = resp.clicked() && click_on == 1;
    }
    x += 14.0;
    if let Some(vis) = row.eye {
        let size = t.space_12 + t.space_3;
        let er = Rect::from_center_size(eye_r.center(), vec2(size, size));
        let ink = if hover_on == 2 { t.text } else { t.text_dim };
        icons::paint_ink(ui.painter(), er, if vis { "eye" } else { "suppress" }, ink.gamma_multiply(if vis && !row.dim { 1.0 } else { DIM_ICON_OPACITY + 0.2 }));
        out.eye = resp.clicked() && click_on == 2;
    }
    x += 18.0;
    if !row.icon.is_empty() {
        let ir = Rect::from_min_size(pos2(x, cy - 8.0), vec2(t.space_16, t.space_16));
        icons::paint_icon(ui.painter(), ir, row.icon, if row.dim { DIM_ICON_OPACITY } else { 1.0 });
        x += 21.0;
    }
    let col = if row.selected { t.selected_ink } else if row.dim { t.text_dim } else { t.text };
    let font = if row.selected { crate::fonts::semibold(t.font_label) } else { FontId::proportional(t.font_label) };
    let mut right = r.right() - t.space_4;
    if let Some(dot) = row.status {
        right -= t.space_8;
        ui.painter().circle_filled(pos2(right, cy), t.space_3 + 0.5, dot);
        right -= t.space_8;
    }
    if !row.count.is_empty() {
        let max_width = (right - x).max(0.0) * 0.45;
        let mut job = egui::text::LayoutJob::simple(row.count.to_string(), FontId::proportional(t.font_small), t.text_dim, max_width);
        job.wrap.max_rows = 1;
        job.wrap.overflow_character = Some('\u{2026}');
        let galley = ui.painter().layout_job(job);
        right -= galley.size().x;
        ui.painter().with_clip_rect(ui.clip_rect().intersect(r)).galley(pos2(right, cy - galley.size().y / 2.0), galley, t.text_dim);
        right -= t.space_6;
    }
    let room = (right - x).max(0.0);
    let mut job = egui::text::LayoutJob::simple(row.label.to_string(), font, col, room);
    job.wrap.max_rows = 1;
    job.wrap.break_anywhere = true;
    job.wrap.overflow_character = Some('\u{2026}');
    let galley = ui.painter().layout_job(job);
    let elided = galley.elided;
    ui.painter().with_clip_rect(ui.painter().clip_rect().intersect(r)).galley(pos2(x, cy - galley.size().y / 2.0), galley, col);
    out.clicked = resp.clicked() && click_on == 0;
    out.double = resp.double_clicked() && click_on == 0;
    if resp.secondary_clicked() {
        out.secondary = resp.interact_pointer_pos();
    }
    resp.widget_info(|| egui::WidgetInfo::selected(egui::WidgetType::SelectableLabel, true, row.selected, row.label));
    let resp = match (row.tip, elided) {
        (Some(tip), true) => resp.on_hover_text(format!("{}\n{tip}", row.label)),
        (Some(tip), false) => resp.on_hover_text(tip),
        (None, true) => resp.on_hover_text(row.label),
        (None, false) => resp,
    };
    out.response = Some(resp);
    out
}
