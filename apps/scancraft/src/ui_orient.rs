//! The properties-card bodies of the orientation commands: the Auto Orient proposal (Accept /
//! Reject come from the shared footer), the Orient by Datums tool, and Scale from Lengths.

use eframe::egui;
use scancraft_align::{DatumError, Primitive, WorldAxis, WorldPlane};
use scancraft_ui::Tokens;
use scancraft_ui::kit;

use crate::app::{DatumKind, ScanCraftApp};

/// The Auto Orient proposal: what the candidate frame came from.
pub fn orient_preview_body(app: &mut ScanCraftApp, ui: &mut egui::Ui) {
    let t = Tokens::get();
    let Some(preview) = &app.orient_preview else { return };
    kit::section_heading(ui, "Proposed orientation");
    kit::row(ui, "Source", |ui| kit::value_text(ui, preview.source, false));
    kit::row(ui, "Up from", |ui| kit::value_text(ui, preview.result.up_source.label(), false));
    kit::row(ui, "Turned", |ui| kit::value_text(ui, &format!("{:.1}°", preview.result.turned_deg), false));
    let faces = if preview.result.faces.is_empty() { "—".to_string() } else { format!("{} directions", preview.result.faces.len()) };
    kit::row(ui, "Faces", |ui| kit::value_text(ui, &faces, preview.result.faces.is_empty()));
    kit::row(ui, "Main axis", |ui| kit::value_text(ui, if preview.result.axis.is_some() { "found" } else { "—" }, preview.result.axis.is_none()));
    let rim = preview.result.rim.as_ref().map(|r| format!("{} points · {:.2} mm", r.points, r.rms_mm)).unwrap_or_else(|| "—".to_string());
    kit::row(ui, "Rim", |ui| kit::value_text(ui, &rim, preview.result.rim.is_none()));
    if preview.result.faces.len() <= 1 && preview.result.axis.is_none() && preview.result.rim.is_none() {
        kit::status_line(ui, "Little to go on: check the result, or use Datums", t.warning);
    }
    ui.add_space(t.space_4);
    kit::note_with_tip(
        ui,
        "The model is shown where it would sit: base on the grid, faces square to the axes.",
        "Every placed pass, the merged model and the mesh move together. Unaligned passes stay as they are.",
    );
}

/// Orient by Datums: the Pick switch, the fit tolerance, one block per picked datum, and the
/// Preview switch. Apply / Cancel come from the shared footer.
pub fn datums_form(app: &mut ScanCraftApp, ui: &mut egui::Ui) {
    let t = Tokens::get();
    let Some(d) = &app.datum_picking else { return };
    let mut kind = match d.kind {
        DatumKind::Plane => 0,
        DatumKind::Axis => 1,
        DatumKind::Point => 2,
    };
    let mut tolerance = d.tolerance_mm;
    let plane = d.plane.as_ref().map(|(_, f, onto, flip)| (f.rms_mm, f.inliers.len(), *onto, *flip));
    let axis = d.axis.as_ref().map(|(_, f, onto, flip)| {
        let radius = match f.primitive {
            Primitive::Cylinder { radius, .. } => radius,
            _ => 0.0,
        };
        (radius, f.rms_mm, f.inliers.len(), *onto, *flip)
    });
    let origin = d.origin;
    let frame = d.frame;
    let mut preview = d.preview;

    let mut remove = None;
    let mut plane_onto = None;
    let mut axis_onto = None;
    let mut plane_flip = None;
    let mut axis_flip = None;
    let mut new_tolerance = false;

    kit::row(ui, "Pick", |ui| {
        if kit::segmented(ui, &mut kind, &["Plane", "Axis", "Point"], &[], &[]) {
            app.set_datum_kind(match kind {
                0 => DatumKind::Plane,
                1 => DatumKind::Axis,
                _ => DatumKind::Point,
            });
        }
    });
    let r = kit::slider_row(ui, "Tolerance", &mut tolerance, 0.05..=2.0, 2, "mm")
        .on_hover_text("How far a point may lie from the fitted plane or cylinder to count as part of it.");
    // Refitting on every slider step would stutter on a big surface: the picks refit when the
    // drag ends (a typed or single-step change refits at once).
    if r.drag_stopped() || (r.changed() && !r.dragged()) {
        new_tolerance = true;
    }
    ui.add_space(t.space_4);

    if let Some((rms, n, onto, flip)) = plane {
        kit::row(ui, "Plane", |ui| {
            if datum_value_row(ui, &format!("{n} points"), "Remove the plane") {
                remove = Some(DatumKind::Plane);
            }
        });
        kit::row(ui, "Fit", |ui| kit::value_text(ui, &format!("{rms:.2} mm RMS"), false));
        kit::row(ui, "Onto", |ui| {
            let mut sel = match onto {
                WorldPlane::XY => 0,
                WorldPlane::YZ => 1,
                WorldPlane::ZX => 2,
            };
            if kit::dropdown(ui, "plane_onto", &mut sel, &["XY (floor)", "YZ", "ZX"], &[]) {
                plane_onto = Some([WorldPlane::XY, WorldPlane::YZ, WorldPlane::ZX][sel]);
            }
        });
        let mut f = flip;
        if kit::checkbox(ui, "Flip", &mut f).on_hover_text("Turn the model over to the other side of the plane.").changed() {
            plane_flip = Some(f);
        }
    }
    if let Some((radius, rms, _n, onto, flip)) = axis {
        kit::row(ui, "Axis", |ui| {
            if datum_value_row(ui, &format!("\u{00D8} {:.2} mm", radius * 2.0), "Remove the axis") {
                remove = Some(DatumKind::Axis);
            }
        });
        kit::row(ui, "Fit", |ui| kit::value_text(ui, &format!("{rms:.2} mm RMS"), false));
        kit::row(ui, "Onto", |ui| {
            let mut sel = match onto {
                WorldAxis::X => 0,
                WorldAxis::Y => 1,
                WorldAxis::Z => 2,
            };
            if kit::dropdown(ui, "axis_onto", &mut sel, &["X", "Y", "Z"], &[]) {
                axis_onto = Some([WorldAxis::X, WorldAxis::Y, WorldAxis::Z][sel]);
            }
        });
        let mut f = flip;
        if kit::checkbox(ui, "Flip", &mut f).on_hover_text("Point the axis the other way along the world axis.").changed() {
            axis_flip = Some(f);
        }
    }
    if let Some(p) = origin {
        kit::row(ui, "Origin", |ui| {
            if datum_value_row(ui, &format!("{:.1}, {:.1}, {:.1}", p.x, p.y, p.z), "Remove the origin") {
                remove = Some(DatumKind::Point);
            }
        });
    }
    if plane.is_none() && axis.is_none() && origin.is_none() {
        kit::note(ui, "No datums yet.");
    }
    if plane.is_some() && axis.is_some() {
        if let Some(Ok(f)) = frame {
            if let Some(off) = f.axis_off_deg {
                kit::row(ui, "Square", |ui| kit::value_text(ui, &format!("{off:.1}° off"), false));
            }
        }
    }
    if let Some(Err(e)) = frame {
        let colour = if e == DatumError::Conflict { t.error } else { t.warning };
        kit::status_line(ui, &e.to_string(), colour);
    }
    ui.add_space(t.space_4);
    if kit::checkbox(ui, "Preview", &mut preview).on_hover_text("Show the model where it would go.").changed() {
        app.set_datum_preview(preview);
    }
    ui.add_space(t.space_4);
    kit::note_with_tip(
        ui,
        "Click the model to fit the chosen datum, then choose the world plane or axis it goes onto.",
        "The plane goes first, the axis turns the model about it, the point sets what is left of the origin. Fits grow from the click over the surface within the tolerance.",
    );

    if new_tolerance {
        app.set_datum_tolerance(tolerance);
    }
    if let Some(onto) = plane_onto {
        app.set_plane_onto(onto);
    }
    if let Some(onto) = axis_onto {
        app.set_axis_onto(onto);
    }
    if let Some(flip) = plane_flip {
        app.set_plane_flip(flip);
    }
    if let Some(flip) = axis_flip {
        app.set_axis_flip(flip);
    }
    if let Some(kind) = remove {
        app.remove_datum(kind);
    }
}

/// Scale from Lengths: the picked pairs, the factor they give, and Apply / Cancel in the
/// footer.
/// The value of a picked datum's row: its text fills the width left of the remove (×) button
/// (the button's width is reserved first so text can never run underneath it), ellipsised
/// beyond that, with the full text on hover when it does not fit. Returns the button click.
fn datum_value_row(ui: &mut egui::Ui, text: &str, remove_tip: &str) -> bool {
    let t = Tokens::get();
    let font = egui::FontId::proportional(t.font_label);
    let natural = ui.painter().layout_no_wrap(text.to_string(), font.clone(), t.form_ink).size().x;
    let text_w = (ui.available_width() - t.form_close_size - t.space_4).max(1.0);
    let mut clicked = false;
    ui.horizontal(|ui| {
        let (rect, response) = ui.allocate_exact_size(egui::vec2(text_w, t.control_height), egui::Sense::hover());
        kit::paint_text(ui.painter(), rect, text, font, t.form_ink);
        if natural > text_w {
            response.on_hover_text(text);
        }
        clicked = kit::close_button(ui, remove_tip).clicked();
    });
    clicked
}

pub fn scale_form(app: &mut ScanCraftApp, ui: &mut egui::Ui, id: u32) {
    let t = Tokens::get();
    let name = app.passes.iter().find(|p| p.id == id).map(|p| p.name.clone()).unwrap_or_default();
    kit::row(ui, "Pass", |ui| kit::value_text(ui, &name, false));
    ui.add_space(t.space_4);
    let Some(s) = &app.scale_picking else { return };
    if s.pairs.is_empty() {
        kit::note(ui, "No lengths yet.");
    }
    let mut remove = None;
    let mut edited: Vec<(usize, f32)> = Vec::new();
    for (i, pair) in s.pairs.iter().enumerate() {
        let measured = pair.b.map(|b| (b - pair.a).length());
        let mut true_mm = pair.true_mm;
        kit::row(ui, &format!("{}", i + 1), |ui| {
            ui.horizontal(|ui| {
                ui.spacing_mut().text_edit_width = 72.0;
                let measured_text = measured.map_or("…".to_string(), |m| format!("{m:.3}"));
                ui.label(egui::RichText::new(measured_text).size(t.font_label).color(if measured.is_none() { t.form_dim } else { t.form_ink }));
                ui.label(egui::RichText::new("→").size(t.font_label).color(t.form_dim));
                if ui.add_enabled(measured.is_some(), egui::DragValue::new(&mut true_mm).suffix(" mm").speed(0.5).range(0.001..=1e6)).changed() {
                    edited.push((i, true_mm));
                }
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    if kit::close_button(ui, "Remove this length").clicked() {
                        remove = Some(i);
                    }
                });
            });
        });
    }
    let estimate = app.scale_estimate();
    let complete = s.pairs.iter().filter(|p| p.b.is_some()).count();
    kit::row(ui, "Factor", |ui| kit::value_text(ui, &estimate.as_ref().map_or("—".to_string(), |e| format!("× {:.4}", e.factor)), estimate.is_none()));
    let spread = match &estimate {
        Some(e) if complete > 1 => (format!("± {:.1} %", e.spread * 100.0), false),
        Some(_) => ("one length".to_string(), true),
        None => ("—".to_string(), true),
    };
    kit::row(ui, "Spread", |ui| kit::value_text(ui, &spread.0, spread.1));
    if let Some(e) = &estimate {
        if e.spread > 0.02 {
            kit::status_line(ui, "The lengths disagree by more than 2 %: check the picks and the typed values", t.warning);
        }
    }
    ui.add_space(t.space_4);
    kit::note_with_tip(
        ui,
        "Click two points on the pass, then type their true distance.",
        "Several lengths are averaged, long ones counting more. An aligned pass goes back to its own frame, as its alignment no longer fits.",
    );
    for (i, mm) in edited {
        app.set_scale_true(i, mm);
    }
    if let Some(i) = remove {
        app.remove_scale_pair(i);
    }
}
