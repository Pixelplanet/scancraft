//! Dense form elements for feature cards.
//!
//! Ported from SolidCraft `crates/ui-egui/src/ui_kit/layout.rs` (`scope`, `sized_row_body`,
//! `section_heading`, `section_with_badge`), `choice.rs` (`segmented`), `checks.rs`
//! (`paint_box`, `checkbox_indeterminate`), `fields.rs` (`slider`, `dropdown`) and
//! `actions.rs` (`button_widget`).

use super::chrome::{dropdown_indicator, paint_text};
use crate::fonts;
use crate::theme::Tokens;
use egui::{Rect, Response, RichText, Sense, Ui, pos2, vec2};

/// Inherited dense-form styling: 28 pt controls, 5 pt row gap, control fill and border.
pub fn scope<R>(ui: &mut Ui, add: impl FnOnce(&mut Ui) -> R) -> R {
    ui.scope(|ui| {
        let t = Tokens::get();
        ui.spacing_mut().item_spacing = vec2(t.space_4, t.form_gap_y);
        ui.spacing_mut().interact_size.y = t.control_height;
        ui.spacing_mut().button_padding = vec2(t.space_8, t.space_4);
        ui.style_mut().text_styles.insert(egui::TextStyle::Body, egui::FontId::proportional(t.font_label));
        ui.style_mut().text_styles.insert(egui::TextStyle::Button, egui::FontId::proportional(t.font_label));
        ui.style_mut().text_styles.insert(egui::TextStyle::Monospace, egui::FontId::monospace(t.font_body));
        let visuals = ui.visuals_mut();
        visuals.weak_text_color = Some(t.form_dim);
        visuals.selection.stroke = egui::Stroke::new(1.0, t.form_primary);
        visuals.selection.bg_fill = t.selected_bg;
        visuals.widgets.noninteractive.bg_stroke = egui::Stroke::new(1.0, t.dialog_rule);
        for state in [&mut visuals.widgets.inactive, &mut visuals.widgets.hovered, &mut visuals.widgets.active, &mut visuals.widgets.open] {
            state.bg_fill = t.control_fill;
            state.weak_bg_fill = t.control_fill;
            state.bg_stroke = t.control_border_stroke(t.control_border);
            state.corner_radius = t.control_radius.into();
            state.expansion = 0.0;
            state.fg_stroke.color = t.form_ink;
        }
        visuals.widgets.hovered.bg_stroke = egui::Stroke::new(1.0, t.form_primary.gamma_multiply(0.7));
        visuals.widgets.active.bg_stroke = egui::Stroke::new(1.0, t.form_primary);
        ui.style_mut().wrap_mode = Some(egui::TextWrapMode::Truncate);
        add(ui)
    })
    .inner
}

/// A labelled row: the label column at the left (70 pt, or wider for a longer label), the control
/// filling the rest (`sized_row_body`).
pub fn row<R>(ui: &mut Ui, text: &str, control: impl FnOnce(&mut Ui) -> R) -> R {
    sized_row(ui, text, Tokens::get().control_height, control)
}

/// [`row`] at a given height.
pub fn sized_row<R>(ui: &mut Ui, text: &str, height: f32, control: impl FnOnce(&mut Ui) -> R) -> R {
    let t = Tokens::get();
    let width = ui.available_width();
    let (rect, _) = ui.allocate_exact_size(vec2(width, height), Sense::hover());
    let natural = ui.painter().layout_no_wrap(text.to_string(), egui::FontId::proportional(t.font_label), t.dialog_label).size().x.ceil();
    let label_width = if text.is_empty() { 0.0 } else { natural.max(t.form_label_width).min(width * 0.45) };
    let label_rect = Rect::from_min_size(rect.min, vec2(label_width, rect.height()));
    if !text.is_empty() {
        paint_text(ui.painter(), label_rect, text, egui::FontId::proportional(t.font_label), t.dialog_label);
    }
    let gap = if text.is_empty() { 0.0 } else { t.space_8 };
    let control_rect = Rect::from_min_max(pos2((label_rect.right() + gap).min(rect.right()), rect.top()), rect.max);
    ui.scope_builder(egui::UiBuilder::new().id_salt(text).max_rect(control_rect).layout(egui::Layout::left_to_right(egui::Align::Center)), |ui| {
        ui.set_clip_rect(ui.clip_rect().intersect(control_rect.expand(2.0)));
        ui.spacing_mut().text_edit_width = control_rect.width();
        control(ui)
    })
    .inner
}

/// Upper-case semibold caption (`section_heading`).
pub fn section_heading(ui: &mut Ui, title: &str) {
    let t = Tokens::get();
    let (rect, _) = ui.allocate_exact_size(vec2(ui.available_width(), t.space_16), Sense::hover());
    paint_text(ui.painter(), rect, &title.to_uppercase(), fonts::semibold(t.font_caption), t.form_dim);
}

/// A folding section: chevron, upper-case tracked caption and a rule to the right
/// (`section_with_badge`). Its open state persists.
pub fn section<R>(ui: &mut Ui, id: impl egui::AsIdSalt, title: &str, default_open: bool, add: impl FnOnce(&mut Ui) -> R) -> Option<R> {
    let t = Tokens::get();
    ui.add_space(t.space_2);
    let id = ui.make_persistent_id(id);
    let mut open = ui.ctx().data_mut(|data| data.get_persisted::<bool>(id)).unwrap_or(default_open);
    let (rect, response) = ui.allocate_exact_size(vec2(ui.available_width(), t.control_height - t.space_4), Sense::click());
    if response.hovered() || response.has_focus() {
        ui.painter().rect_filled(rect.shrink2(vec2(0.0, 1.0)), t.radius_sm, t.hover);
    }
    let icon_rect = Rect::from_center_size(pos2(rect.left() + t.form_section_icon_size / 2.0, rect.center().y), vec2(t.form_section_icon_size, t.form_section_icon_size));
    super::chrome::disclosure(ui.painter(), icon_rect.center(), open, t.form_dim);
    let text_rect = Rect::from_min_max(pos2((icon_rect.right() + t.form_gap_y).min(rect.right()), rect.top()), rect.max);
    let caption = title.to_uppercase();
    let mut format = egui::TextFormat::simple(fonts::semibold(t.font_caption), t.form_dim);
    format.extra_letter_spacing = t.font_caption * t.font_section_tracking;
    let galley = ui.fonts_mut(|f| f.layout_job(egui::text::LayoutJob::single_section(caption, format)));
    let title_width = galley.size().x.ceil().min(text_rect.width());
    ui.painter().galley(pos2(text_rect.left(), text_rect.center().y - galley.size().y / 2.0), galley, t.form_dim);
    let rule_left = text_rect.left() + title_width + t.space_12;
    if rule_left < rect.right() {
        t.secondary_hline(ui.painter(), rule_left..=rect.right(), rect.center().y, t.dialog_rule);
    }
    response.widget_info(|| egui::WidgetInfo::selected(egui::WidgetType::CollapsingHeader, ui.is_enabled(), open, title));
    if response.clicked() {
        open = !open;
    }
    ui.ctx().data_mut(|data| data.insert_persisted(id, open));
    open.then(|| ui.push_id(id, |ui| add(ui)).inner)
}

/// Equal-width segments in one control (`segmented`); `enabled` may be shorter than `options`
/// (missing entries are enabled). `reasons` give disabled segments their tooltip.
pub fn segmented(ui: &mut Ui, selected: &mut usize, options: &[&str], enabled: &[bool], reasons: &[Option<&str>]) -> bool {
    let t = Tokens::get();
    let font_size = t.font_label;
    let (rect, _) = ui.allocate_exact_size(vec2(ui.available_width(), t.control_height), Sense::hover());
    ui.painter().rect_filled(rect, t.control_radius, t.control_fill);
    let width = rect.width() / options.len().max(1) as f32;
    let mut clicked = false;
    for (index, text) in options.iter().enumerate() {
        let segment = Rect::from_min_size(pos2(rect.left() + index as f32 * width, rect.top()), vec2(width, rect.height()));
        let segment_enabled = ui.is_enabled() && enabled.get(index).copied().unwrap_or(true);
        let id = ui.make_persistent_id(("segment", index, *text));
        let response = ui.interact(segment, id, if segment_enabled { Sense::click() } else { Sense::hover() });
        let on = *selected == index;
        let rounding = egui::CornerRadius {
            nw: if index == 0 { t.control_radius } else { 0 },
            sw: if index == 0 { t.control_radius } else { 0 },
            ne: if index + 1 == options.len() { t.control_radius } else { 0 },
            se: if index + 1 == options.len() { t.control_radius } else { 0 },
        };
        if on || (response.hovered() && segment_enabled) {
            ui.painter().rect_filled(segment, rounding, if on { t.selected_bg } else { t.hover });
        }
        if index > 0 {
            ui.painter().vline(segment.left(), segment.y_range(), t.control_border_stroke(t.control_border));
        }
        let font = if on { fonts::semibold(font_size) } else { egui::FontId::proportional(font_size) };
        let ink = if !segment_enabled {
            t.form_dim.gamma_multiply(0.6)
        } else if on {
            t.selected_ink
        } else {
            t.form_ink
        };
        let galley = ui.painter().layout_no_wrap((*text).to_string(), font.clone(), ink);
        let text_rect = Rect::from_center_size(segment.center(), vec2(galley.size().x.min(segment.width() - t.space_8), segment.height()));
        paint_text(ui.painter(), text_rect, text, font, ink);
        response.widget_info(|| egui::WidgetInfo::selected(egui::WidgetType::Button, segment_enabled, on, *text));
        let response = match reasons.get(index).copied().flatten() {
            Some(reason) if !segment_enabled => response.on_hover_text(reason),
            _ => response.on_hover_text(*text),
        };
        if response.clicked() && segment_enabled {
            *selected = index;
            clicked = true;
        }
    }
    ui.painter().rect_stroke(rect, t.control_radius, t.control_border_stroke(t.control_border), egui::StrokeKind::Inside);
    clicked
}

fn paint_box(ui: &Ui, square: Rect, response: &Response, on: bool) {
    let t = Tokens::get();
    let border = if on || response.hovered() || response.has_focus() { t.form_primary } else { t.control_border };
    let fill = if on {
        t.form_primary_fill
    } else if response.hovered() {
        t.hover
    } else {
        t.control_fill
    };
    ui.painter().rect(square, t.radius_sm, fill, egui::Stroke::new(1.0, border), egui::StrokeKind::Inside);
    if on {
        let mark = square.shrink(t.space_3);
        let points = vec![mark.left_center(), mark.center_bottom(), mark.right_top()];
        ui.painter().add(egui::Shape::line(points, egui::Stroke::new(t.space_2, t.on_accent)));
    }
}

/// The kit checkbox (`checkbox_indeterminate` without the mixed state).
pub fn checkbox(ui: &mut Ui, label: &str, value: &mut bool) -> Response {
    let t = Tokens::get();
    let font = egui::FontId::proportional(t.font_label);
    let galley = ui.painter().layout_no_wrap(label.to_owned(), font.clone(), t.form_ink);
    let gap = if label.is_empty() { 0.0 } else { t.space_6 };
    let width = (t.space_14 + gap + galley.size().x).max(ui.spacing().interact_size.y).min(ui.available_width());
    let height = t.form_checkbox_height.max(galley.size().y);
    let (rect, mut response) = ui.allocate_exact_size(vec2(width, height), Sense::click());
    if response.clicked() {
        *value = !*value;
        response.mark_changed();
    }
    let ppp = ui.ctx().pixels_per_point();
    let min = pos2(rect.left(), rect.center().y - t.space_14 / 2.0);
    let square = Rect::from_min_size(pos2((min.x * ppp).round() / ppp, (min.y * ppp).round() / ppp), vec2(t.space_14, t.space_14));
    paint_box(ui, square, &response, *value);
    let ink = if ui.is_enabled() { t.form_ink } else { t.form_dim };
    let text_rect = Rect::from_min_max(pos2((square.right() + gap).min(rect.right()), rect.top()), rect.max);
    paint_text(ui.painter(), text_rect, label, font, ink);
    response.widget_info(|| egui::WidgetInfo::selected(egui::WidgetType::Checkbox, ui.is_enabled(), *value, label));
    response
}

/// The kit slider: a 4 pt track with the filled part in the primary colour and a ring thumb,
/// with egui's value editor at the right (`fields::slider`).
pub fn slider<T: egui::emath::Numeric>(ui: &mut Ui, value: &mut T, range: std::ops::RangeInclusive<T>, decimals: usize, suffix: &str) -> Response {
    scope(ui, |ui| {
        let t = Tokens::get();
        let limits = (range.start().to_f64(), range.end().to_f64());
        let font = egui::TextStyle::Monospace.resolve(ui.style());
        let fmt = |v: f64| if suffix.is_empty() { format!("{v:.decimals$}") } else { format!("{v:.decimals$} {suffix}") };
        let text_width = [limits.0, limits.1, value.to_f64()]
            .into_iter()
            .map(|v| ui.painter().layout_no_wrap(fmt(v), font.clone(), t.form_ink).size().x.ceil())
            .fold(0.0, f32::max);
        let editor_width = (text_width + ui.spacing().button_padding.x * 2.0).max(ui.spacing().interact_size.x);
        let reserved = (editor_width + ui.spacing().item_spacing.x).max(t.space_24 * 2.0 + t.space_4);
        ui.spacing_mut().slider_width = (ui.available_width() - reserved).max(t.space_24);
        let suffix = suffix.to_string();
        let mut widget = egui::Slider::new(value, range)
            .handle_shape(egui::style::HandleShape::Circle)
            .custom_formatter(move |v, _| if suffix.is_empty() { format!("{v:.decimals$}") } else { format!("{v:.decimals$} {suffix}") })
            .custom_parser(|s| s.split_whitespace().next().and_then(|n| n.parse::<f64>().ok()));
        if decimals == 0 {
            widget = widget.integer();
        }
        ui.style_mut().drag_value_text_style = egui::TextStyle::Monospace;
        let response = ui.add(widget);
        let slider = Rect::from_min_size(response.rect.min, vec2(ui.spacing().slider_width, response.rect.height()));
        ui.painter().rect_filled(slider.with_min_x(slider.left() - 2.0).with_max_x(slider.right() + 1.0), 0.0, t.dialog_body);
        let input_radius = ui.text_style_height(&egui::TextStyle::Body).max(ui.spacing().interact_size.y) / 2.5;
        let track = Rect::from_center_size(slider.center(), vec2((slider.width() - input_radius * 2.0).max(0.0), t.space_4));
        let fraction = if limits.1 != limits.0 { ((value.to_f64() - limits.0) / (limits.1 - limits.0)).clamp(0.0, 1.0) as f32 } else { 0.0 };
        let primary = if ui.is_enabled() { t.form_primary } else { t.form_dim.gamma_multiply(0.6) };
        let center = pos2(egui::lerp(track.x_range(), fraction), track.center().y);
        ui.painter().rect_filled(track, t.control_radius, t.dialog_rule);
        ui.painter().rect_filled(Rect::from_min_max(track.min, pos2(center.x, track.bottom())), t.control_radius, primary);
        ui.painter().circle(center, t.space_8, t.control_fill, egui::Stroke::new(t.space_2, primary));
        response
    })
}

/// A labelled slider row.
pub fn slider_row<T: egui::emath::Numeric>(ui: &mut Ui, label: &str, value: &mut T, range: std::ops::RangeInclusive<T>, decimals: usize, suffix: &str) -> Response {
    row(ui, label, |ui| slider(ui, value, range, decimals, suffix))
}

/// Button kinds (`ui_kit::Button`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Button {
    Normal,
    Primary,
    Ghost,
}

/// The kit button (`button_widget`).
pub fn button(ui: &mut Ui, label: &str, kind: Button, enabled: bool) -> Response {
    let t = Tokens::get();
    let text = RichText::new(label).size(t.font_label);
    let widget = match kind {
        Button::Primary => egui::Button::new(text.color(t.on_accent)).fill(t.form_primary_fill).stroke(egui::Stroke::NONE),
        Button::Ghost => egui::Button::new(text).frame(false),
        Button::Normal => egui::Button::new(text.color(t.form_ink)).fill(t.control_fill).stroke(t.control_border_stroke(t.control_border)),
    }
    .corner_radius(t.control_radius)
    .min_size(vec2(t.space_24 * 2.5, t.form_button_height));
    let response = ui.add_enabled(enabled, widget);
    if enabled {
        response.on_hover_cursor(egui::CursorIcon::PointingHand)
    } else {
        response
    }
}

/// A full-width button (used for actions inside a section).
pub fn wide_button(ui: &mut Ui, label: &str, kind: Button, enabled: bool) -> Response {
    let t = Tokens::get();
    let text = RichText::new(label).size(t.font_label);
    let widget = match kind {
        Button::Primary => egui::Button::new(text.color(t.on_accent)).fill(t.form_primary_fill).stroke(egui::Stroke::NONE),
        Button::Ghost => egui::Button::new(text).frame(false),
        Button::Normal => egui::Button::new(text.color(t.form_ink)).fill(t.control_fill).stroke(t.control_border_stroke(t.control_border)),
    }
    .corner_radius(t.control_radius)
    .min_size(vec2(ui.available_width(), t.form_button_height));
    ui.add_enabled(enabled, widget)
}

/// A read-only value in a row, drawn in a field-shaped well (SolidCraft's selection field look),
/// dim when `dim`.
pub fn value_field(ui: &mut Ui, text: &str, dim: bool) -> Response {
    let t = Tokens::get();
    let (rect, response) = ui.allocate_exact_size(vec2(ui.available_width(), t.control_height), Sense::hover());
    ui.painter().rect(rect, t.control_radius, t.field, t.control_border_stroke(t.dialog_rule), egui::StrokeKind::Inside);
    let inner = rect.shrink2(vec2(t.form_inset, 0.0));
    paint_text(ui.painter(), inner, text, egui::FontId::proportional(t.font_label), if dim { t.form_dim } else { t.form_ink });
    response
}

/// Plain value text at the control column (no well): for read-outs in a properties list.
pub fn value_text(ui: &mut Ui, text: &str, dim: bool) -> Response {
    let t = Tokens::get();
    let (rect, response) = ui.allocate_exact_size(vec2(ui.available_width(), t.control_height), Sense::hover());
    paint_text(ui.painter(), rect, text, egui::FontId::proportional(t.font_label), if dim { t.form_dim } else { t.form_ink });
    response
}

/// The kit dropdown: a control-shaped button with a chevron; options in a popup list.
pub fn dropdown(ui: &mut Ui, id: impl egui::AsIdSalt, selected: &mut usize, options: &[&str], enabled: &[bool]) -> bool {
    dropdown_with_reasons(ui, id, selected, options, enabled, &[])
}

/// [`dropdown`] whose disabled options say why under their label, in the list itself (SolidCraft's
/// drop-down rows carry their disabled reason the same way).
pub fn dropdown_with_reasons(ui: &mut Ui, id: impl egui::AsIdSalt, selected: &mut usize, options: &[&str], enabled: &[bool], reasons: &[Option<&str>]) -> bool {
    let t = Tokens::get();
    let current = options.get(*selected).copied().unwrap_or("—");
    let mut changed = false;
    let (rect, response) = ui.allocate_exact_size(vec2(ui.available_width(), t.control_height), Sense::click());
    let popup_id = ui.make_persistent_id(id).with("popup");
    let open = egui::Popup::is_id_open(ui.ctx(), popup_id);
    let border = if response.hovered() || open { t.form_primary.gamma_multiply(0.7) } else { t.control_border };
    let fill = if ui.is_enabled() { t.control_fill } else { t.field };
    ui.painter().rect(rect, t.control_radius, fill, egui::Stroke::new(1.0, border), egui::StrokeKind::Inside);
    let chevron = Rect::from_center_size(pos2(rect.right() - t.form_inset - 5.0, rect.center().y), vec2(10.0, 10.0));
    dropdown_indicator(ui.painter(), chevron, t.text_dim);
    let text_rect = Rect::from_min_max(pos2(rect.left() + t.form_inset, rect.top()), pos2(chevron.left() - t.space_4, rect.bottom()));
    paint_text(ui.painter(), text_rect, current, egui::FontId::proportional(t.font_label), if ui.is_enabled() { t.form_ink } else { t.form_dim });
    let width = rect.width().max(220.0);
    egui::Popup::menu(&response).id(popup_id).close_behavior(egui::PopupCloseBehavior::CloseOnClick).show(|ui| {
        ui.set_min_width(width);
        ui.spacing_mut().item_spacing.y = 1.0;
        for (index, option) in options.iter().enumerate() {
            let ok = enabled.get(index).copied().unwrap_or(true);
            let reason = if ok { None } else { reasons.get(index).copied().flatten() };
            let reason_galley = reason.map(|r| ui.painter().layout(r.to_string(), egui::FontId::proportional(t.font_small), t.text_dim, width - 2.0 * t.space_8));
            let h = 24.0 + reason_galley.as_ref().map_or(0.0, |g| g.size().y + t.space_2);
            let (row, resp) = ui.allocate_exact_size(vec2(width, h), if ok { Sense::click() } else { Sense::hover() });
            let on = *selected == index;
            if on {
                ui.painter().rect_filled(row, t.radius_sm, t.selected_bg);
            } else if ok && resp.hovered() {
                ui.painter().rect_filled(row, t.radius_sm, t.hover);
            }
            let ink = if !ok { t.text_dim } else if on { t.selected_ink } else { t.form_ink };
            let font = if on { crate::fonts::semibold(t.font_label) } else { egui::FontId::proportional(t.font_label) };
            ui.painter().text(pos2(row.left() + t.space_8, row.top() + 12.0), egui::Align2::LEFT_CENTER, *option, font, ink);
            if let Some(g) = reason_galley {
                ui.painter().galley(pos2(row.left() + t.space_8, row.top() + 22.0), g, t.text_dim);
            }
            resp.widget_info(|| egui::WidgetInfo::selected(egui::WidgetType::SelectableLabel, ok, on, *option));
            if ok && resp.clicked() {
                *selected = index;
                changed = true;
            }
        }
    });
    response.widget_info(|| egui::WidgetInfo::labeled(egui::WidgetType::ComboBox, true, current));
    changed
}

/// A dim, wrapping note under the controls (`Text::Note`).
pub fn note(ui: &mut Ui, text: &str) {
    let t = Tokens::get();
    ui.add(egui::Label::new(RichText::new(text).size(t.font_small).color(t.text_dim)).wrap());
}

/// A coloured one-line status message with a dot (`status_line`).
pub fn status_line(ui: &mut Ui, text: &str, color: egui::Color32) {
    let t = Tokens::get();
    let galley = ui.painter().layout(text.to_string(), egui::FontId::proportional(t.font_body), color, (ui.available_width() - t.space_14).max(10.0));
    let (rect, _) = ui.allocate_exact_size(vec2(ui.available_width(), galley.size().y.max(t.space_16)), Sense::hover());
    ui.painter().circle_filled(pos2(rect.left() + t.space_4, rect.top() + galley.size().y.min(t.space_16) / 2.0 + 1.0), t.space_3, color);
    ui.painter().galley(pos2(rect.left() + t.space_14, rect.top()), galley, color);
}

/// The card's footer: the dialog-footer fill, Cancel and the primary action at the right.
/// Returns (cancel, ok).
pub fn dialog_footer(ui: &mut Ui, ok: &str, ok_enabled: bool, cancel: Option<&str>) -> (bool, bool) {
    let t = Tokens::get();
    let (rect, _) = ui.allocate_exact_size(vec2(ui.available_width(), t.form_footer_height), Sense::hover());
    let mut cancelled = false;
    let mut accepted = false;
    let button_h = t.form_button_height;
    let y = rect.center().y - button_h / 2.0;
    let ok_width = ui.painter().layout_no_wrap(ok.to_string(), egui::FontId::proportional(t.font_label), t.on_accent).size().x + 2.0 * t.space_16;
    let ok_rect = Rect::from_min_size(pos2(rect.right() - t.space_12 - ok_width, y), vec2(ok_width, button_h));
    let mut child = ui.new_child(egui::UiBuilder::new().max_rect(rect));
    let r = child.put(ok_rect, |ui: &mut Ui| button(ui, ok, Button::Primary, ok_enabled));
    accepted |= r.clicked();
    if let Some(cancel) = cancel {
        let w = child.painter().layout_no_wrap(cancel.to_string(), egui::FontId::proportional(t.font_label), t.form_ink).size().x + 2.0 * t.space_16;
        let c_rect = Rect::from_min_size(pos2(ok_rect.left() - t.space_6 - w, y), vec2(w, button_h));
        cancelled |= child.put(c_rect, |ui: &mut Ui| button(ui, cancel, Button::Normal, true)).clicked();
    }
    (cancelled, accepted)
}
