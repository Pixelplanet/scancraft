//! The history timeline: every action that changed the scans, merged model or mesh, oldest
//! first, with the current position marked. Clicking a step jumps straight there (as many undos
//! or redos as needed in one go); the steps after it stay available until the next new action.

use eframe::egui::{self, Margin, pos2, vec2};
use scancraft_ui::kit::{self, Surface};
use scancraft_ui::{Tokens, fonts};

use crate::app::ScanCraftApp;

/// "3 min ago" for a step's time (no time zone needed).
fn ago(at: std::time::SystemTime) -> String {
    let s = std::time::SystemTime::now().duration_since(at).map(|d| d.as_secs()).unwrap_or(0);
    match s {
        0..=9 => "just now".into(),
        10..=59 => format!("{s} s ago"),
        60..=3599 => format!("{} min ago", s / 60),
        _ => format!("{} h {} min ago", s / 3600, (s % 3600) / 60),
    }
}

/// A step's label as a sentence ("align Pass 2" → "Align Pass 2").
fn sentence(label: &str) -> String {
    let mut c = label.chars();
    match c.next() {
        Some(f) => f.to_uppercase().chain(c).collect(),
        None => "Action".into(),
    }
}

/// The timeline card, hanging below `anchor` (the History button). Closes on a click outside.
pub fn timeline(app: &mut ScanCraftApp, ctx: &egui::Context, anchor: egui::Pos2) {
    if !app.show_history {
        return;
    }
    let t = Tokens::get();
    let steps = app.history.timeline();
    let done = app.history.done_count();
    let blocked = (!app.can_edit()).then_some(if app.busy.is_some() { "Wait for the running operation to finish" } else { "Stop the capture first" });
    let width = 320.0;
    let mut jump = None;
    let area = egui::Area::new(egui::Id::new("history_timeline")).pivot(egui::Align2::LEFT_TOP).fixed_pos(anchor).order(egui::Order::Foreground).show(ctx, |ui| {
        kit::frame(Surface::Toast).inner_margin(Margin::same(10)).show(ui, |ui| {
            ui.set_width(width - 20.0);
            kit::scope(ui, |ui| {
                kit::section_heading(ui, "History");
                if steps.is_empty() {
                    kit::note(ui, "Nothing to undo yet. Every change to the scans, the merged model or the mesh appears here.");
                    return;
                }
                if let Some(why) = blocked {
                    kit::status_line(ui, why, t.warning);
                }
                egui::ScrollArea::vertical().max_height(380.0).stick_to_bottom(true).show(ui, |ui| {
                    // Position k = the state after k steps; 0 is before the oldest step kept.
                    let rows = std::iter::once(("Start".to_string(), None, true)).chain(steps.iter().map(|s| (sentence(&s.label), Some(s.at), s.done)));
                    for (k, (label, at, is_done)) in rows.enumerate() {
                        let current = k == done;
                        let (rect, resp) = ui.allocate_exact_size(vec2(ui.available_width(), 26.0), egui::Sense::click());
                        let hovered = resp.hovered() && blocked.is_none() && !current;
                        if current {
                            ui.painter().rect_filled(rect, t.radius_sm, t.selected_bg);
                        } else if hovered {
                            ui.painter().rect_filled(rect, t.radius_sm, t.hover);
                        }
                        // Timeline rail and node.
                        let x = rect.left() + 10.0;
                        let ink = if is_done || current { t.text } else { t.text_dim };
                        ui.painter().line_segment([pos2(x, rect.top()), pos2(x, rect.bottom())], egui::Stroke::new(1.0, t.text_dim.gamma_multiply(0.5)));
                        if current {
                            ui.painter().circle_filled(pos2(x, rect.center().y), 4.5, t.accent);
                        } else {
                            ui.painter().circle_stroke(pos2(x, rect.center().y), 3.5, egui::Stroke::new(1.2, ink));
                        }
                        let font = if current { fonts::semibold(t.font_body) } else { egui::FontId::proportional(t.font_body) };
                        let right = match at {
                            Some(at) if !current => ago(at),
                            _ if current => "now".to_string(),
                            _ => String::new(),
                        };
                        let r_galley = ui.painter().layout_no_wrap(right, egui::FontId::proportional(t.font_label), t.text_dim);
                        let text_room = (rect.width() - 30.0 - r_galley.size().x - 8.0).max(20.0);
                        let galley = ui.painter().layout(label, font, ink, text_room);
                        ui.painter().with_clip_rect(rect).galley(pos2(x + 14.0, rect.center().y - galley.size().y.min(18.0) / 2.0), galley, ink);
                        ui.painter().galley(pos2(rect.right() - 6.0 - r_galley.size().x, rect.center().y - r_galley.size().y / 2.0), r_galley, t.text_dim);
                        let tip = if current {
                            "The project as it is now".to_string()
                        } else if k < done {
                            format!("Go back to here (undo {} step{})", done - k, if done - k == 1 { "" } else { "s" })
                        } else {
                            format!("Go forward to here (redo {} step{})", k - done, if k - done == 1 { "" } else { "s" })
                        };
                        if resp.on_hover_text(tip).clicked() && blocked.is_none() && !current {
                            jump = Some(k);
                        }
                    }
                });
                ui.add_space(t.space_4);
                let held = app.history_memory();
                kit::note(ui, &format!("{} step{} kept · {} held by the history. Greyed steps can be redone until the next change.", steps.len(), if steps.len() == 1 { "" } else { "s" }, mib(held)));
            });
        });
    });
    if let Some(k) = jump {
        app.jump_to_step(k);
    }
    // A click outside the card (and not on its button) closes it.
    if ctx.input(|i| i.pointer.any_pressed()) && !area.response.contains_pointer() && !app.history_button_hovered {
        app.show_history = false;
    }
}

fn mib(bytes: usize) -> String {
    let m = bytes as f64 / (1u64 << 20) as f64;
    if m < 10.0 { format!("{m:.1} MB") } else { format!("{m:.0} MB") }
}
