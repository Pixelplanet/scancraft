//! AI Pack settings/consent modal and AI Object Segmentation tool UI.

use eframe::egui::{self, Align, Layout, Margin, Rect, Sense, pos2, vec2};
use scancraft_segment::manifest::{DIRECTML_VERSION_PIN, ORT_VERSION_PIN, SAM21_TINY_HF_REPO};
use scancraft_segment::PostprocessConfig;
use scancraft_ui::kit::{self, Button, Surface};
use scancraft_ui::Tokens;

use crate::ai_pack::{ESTIMATED_MODELS_MB, ESTIMATED_RUNTIME_MB};
use crate::ai_segment::ImagePick;
use crate::app::{Capture, ScanCraftApp};
use crate::file_picker::PickPurpose;
use crate::model::thousands;

/// Modal dialog for installing, consenting to licences, inspecting, or removing the AI Pack.
pub fn ai_pack_modal(app: &mut ScanCraftApp, ctx: &egui::Context) {
    if !app.ai_pack_modal {
        return;
    }
    let t = Tokens::get();
    let status = app.ai_status.clone();
    let busy = app.busy.is_some();
    let mut close = false;
    let mut action_download = false;
    let mut action_install_folder = false;
    let mut action_remove = false;
    let mut action_refresh = false;

    egui::Area::new(egui::Id::new("ai-pack-backdrop"))
        .fixed_pos(pos2(0.0, 0.0))
        .order(egui::Order::Foreground)
        .show(ctx, |ui| {
            let screen = ctx.content_rect();
            ui.allocate_rect(screen, Sense::click());
            ui.painter().rect_filled(screen, 0.0, egui::Color32::from_black_alpha(150));
        });

    egui::Window::new("ai-pack-dialog")
        .title_bar(false)
        .collapsible(false)
        .resizable(false)
        .anchor(egui::Align2::CENTER_CENTER, vec2(0.0, 0.0))
        .fixed_size(vec2(580.0, 0.0))
        .order(egui::Order::Foreground)
        .frame(kit::frame(Surface::Feature).inner_margin(Margin::same(t.space_16 as i8)))
        .show(ctx, |ui| {
            kit::scope(ui, |ui| {
                ui.horizontal(|ui| {
                    ui.label(egui::RichText::new("AI Object Segmentation Pack (SAM 2.1 + DirectML)").size(15.0).strong().color(t.text));
                    ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                        if kit::close_button(ui, "Close").clicked() {
                            close = true;
                        }
                    });
                });
                ui.add_space(t.space_8);

                kit::frame(Surface::Panel).inner_margin(Margin::same(t.space_8 as i8)).show(ui, |ui| {
                    kit::row(ui, "Status", |ui| {
                        let msg = if status.ready {
                            format!("Installed ({:.0} MB)", status.total_mb())
                        } else {
                            "Not installed".to_string()
                        };
                        kit::status_line(ui, &msg, if status.ready { t.agent_ok } else { t.warning });
                    });
                    kit::row(ui, "Backend", |ui| kit::value_text(ui, &status.provider_summary, !status.ready));
                    kit::row(ui, "Models", |ui| {
                        kit::value_text(
                            ui,
                            &if status.models_ready {
                                format!("Verified · {}", status.models_dir.display())
                            } else {
                                format!("Missing (~{ESTIMATED_MODELS_MB:.0} MB) · {}", status.models_dir.display())
                            },
                            !status.models_ready,
                        );
                    });
                    kit::row(ui, "Runtime", |ui| {
                        kit::value_text(
                            ui,
                            &if status.runtime_ready {
                                format!("Verified ORT {ORT_VERSION_PIN} + DirectML {DIRECTML_VERSION_PIN}")
                            } else {
                                format!("Missing (~{ESTIMATED_RUNTIME_MB:.0} MB) · {}", status.runtime_dir.display())
                            },
                            !status.runtime_ready,
                        );
                    });
                });

                ui.add_space(t.space_8);
                kit::note(
                    ui,
                    "Before downloading (~380 MB total from Hugging Face & NuGet, verified by SHA-256), please review the bundled third-party terms:",
                );
                ui.add_space(t.space_4);
                kit::frame(Surface::Panel).inner_margin(Margin::same(t.space_8 as i8)).show(ui, |ui| {
                    kit::note(ui, &format!("• SAM 2.1 Hiera-Tiny ONNX ({SAM21_TINY_HF_REPO}): Apache-2.0 licence. Never uses Revo Scan's proprietary encrypted models."));
                    kit::note(ui, &format!("• ONNX Runtime {ORT_VERSION_PIN} (Microsoft.ML.OnnxRuntime.DirectML): MIT licence + ThirdPartyNotices."));
                    kit::note(ui, &format!("• DirectML {DIRECTML_VERSION_PIN} (Microsoft.AI.DirectML): Microsoft Software License — Windows/Xbox only, app-bundled only, includes Microsoft diagnostic/error telemetry notice."));
                });

                ui.add_space(t.space_12);
                ui.horizontal(|ui| {
                    let can_dl = !busy && !status.ready;
                    if kit::button(
                        ui,
                        if busy { "Working…" } else if status.ready { "Installed" } else { "Accept & Download (~380 MB)" },
                        Button::Primary,
                        can_dl,
                    )
                    .clicked()
                    {
                        action_download = true;
                    }
                    if kit::button(ui, "From folder…", Button::Normal, !busy)
                        .on_hover_text("Copy and SHA-256-verify pre-downloaded model/runtime files from a local folder")
                        .clicked()
                    {
                        action_install_folder = true;
                    }
                    if status.models_dir.exists() || status.runtime_dir.exists() {
                        if kit::button(ui, "Remove", Button::Danger, !busy).clicked() {
                            action_remove = true;
                        }
                    }
                    ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                        if kit::button(ui, "Recheck", Button::Normal, !busy).clicked() {
                            action_refresh = true;
                        }
                    });
                });
            });
        });

    if close {
        app.ai_pack_modal = false;
    }
    if action_refresh {
        app.refresh_ai_status();
    }
    if action_install_folder {
        app.pick(ctx, PickPurpose::AiPackFolder);
    }
    if action_download {
        app.download_ai_pack();
    }
    if action_remove {
        app.remove_ai_pack();
    }
}

/// Interactive split view for `Tool::AiSegment`: selected RGB keyframe + mask on the left,
/// 3D pass preview on the right (rendered inside `central` viewport pane).
pub fn ai_segment_panes(
    app: &mut ScanCraftApp,
    ctx: &egui::Context,
    ui: &mut egui::Ui,
    rect: Rect,
    render_3d: &mut dyn FnMut(&mut ScanCraftApp, &egui::Context, &mut egui::Ui, Rect),
) {
    let t = Tokens::get();
    let mid = rect.center().x;
    let gap = 1.0;
    let left_rect = Rect::from_min_max(rect.min, pos2(mid - gap, rect.max.y));
    let right_rect = Rect::from_min_max(pos2(mid + gap, rect.min.y), rect.max);

    ui.painter().line_segment([pos2(mid, rect.min.y), pos2(mid, rect.max.y)], egui::Stroke::new(1.0, t.border));

    // Left pane: interactive RGB keyframe canvas.
    let mut left_ui = ui.new_child(egui::UiBuilder::new().max_rect(left_rect));
    left_ui.painter().rect_filled(left_rect, 0.0, t.viewport_bottom);
    let inner = left_rect.shrink(t.space_12);
    let mut click_pick: Option<ImagePick> = None;

    if let Some(ai) = app.ai_picking.as_mut() {
        let tex_id = ai.texture(ctx).id();
        let kf = ai.current_keyframe();
        let (iw, ih) = (kf.rgb.width as f32, kf.rgb.height as f32);
        let scale = (inner.width() / iw.max(1.0)).min(inner.height() / ih.max(1.0));
        let size = vec2(iw * scale, ih * scale);
        let img_rect = Rect::from_center_size(inner.center(), size);
        left_ui.painter().image(
            tex_id,
            img_rect,
            Rect::from_min_max(pos2(0.0, 0.0), pos2(1.0, 1.0)),
            egui::Color32::WHITE,
        );
        let resp = left_ui.allocate_rect(img_rect, Sense::click());
        if app.busy.is_none() {
            let positive = if resp.clicked() {
                Some(!left_ui.input(|i| i.modifiers.shift))
            } else if resp.secondary_clicked() {
                Some(false)
            } else {
                None
            };
            if let (Some(pos), Some(pointer)) = (positive, resp.interact_pointer_pos()) {
                let rel_x = ((pointer.x - img_rect.min.x) / img_rect.width().max(1.0)) * iw;
                let rel_y = ((pointer.y - img_rect.min.y) / img_rect.height().max(1.0)) * ih;
                click_pick = Some(ImagePick {
                    x: rel_x.clamp(0.0, iw - 1.0),
                    y: rel_y.clamp(0.0, ih - 1.0),
                    positive: pos,
                });
            }
        }
        let pill_pos = pos2(left_rect.min.x + t.space_12, left_rect.min.y + t.space_12);
        left_ui.painter().text(
            pill_pos,
            egui::Align2::LEFT_TOP,
            format!(
                "Keyframe {} of {} · Left click: include object · Shift/Right click: exclude background",
                ai.selected + 1,
                ai.keyframes.len()
            ),
            egui::FontId::proportional(t.font_body),
            t.text,
        );
    }

    if let Some(pick) = click_pick {
        app.add_ai_pick(pick);
    }

    // Right pane: normal 3D viewport showing the pass & green/red AI 3D preview.
    render_3d(app, ctx, ui, right_rect);
}

/// Right-hand property card form when `Tool::AiSegment` is open.
pub fn ai_segment_form(app: &mut ScanCraftApp, ui: &mut egui::Ui) {
    let t = Tokens::get();
    if !app.ai_status.ready {
        kit::status_line(ui, "AI Pack is not installed", t.warning);
        kit::note(ui, "Download or install the SAM 2.1 + ONNX Runtime pack to run AI segmentation.");
        if kit::wide_button(ui, "Configure AI Pack…", Button::Primary, true).clicked() {
            app.ai_pack_modal = true;
        }
        return;
    }

    let busy = app.busy.is_some();
    let Some(ai) = app.ai_picking.as_ref() else {
        kit::note(ui, "Select a pass with colour keyframes to start AI segmentation.");
        return;
    };

    kit::row(ui, "Pass", |ui| kit::value_text(ui, &ai.pass_name, false));
    kit::row(ui, "Keyframes", |ui| kit::value_text(ui, &thousands(ai.keyframes.len()), false));
    let mut select_kf = None;
    kit::row(ui, "Frame", |ui| {
        let cur = ai.selected;
        let max = ai.keyframes.len().saturating_sub(1);
        ui.horizontal(|ui| {
            if ui.add_enabled(cur > 0 && !busy, egui::Button::new("  <  ")).clicked() {
                select_kf = Some(cur - 1);
            }
            ui.label(egui::RichText::new(format!("{} / {}", cur + 1, ai.keyframes.len())).color(t.text));
            if ui.add_enabled(cur < max && !busy, egui::Button::new("  >  ")).clicked() {
                select_kf = Some(cur + 1);
            }
        });
    });

    if let Some(m) = ai.current_mask() {
        kit::row(ui, "Mask score", |ui| {
            let label = format!(
                "IoU {:.2} · stability {:.2} ({})",
                m.iou_score,
                m.stability_score,
                match m.status {
                    scancraft_segment::TrackingStatus::Tracked => "Tracking",
                    scancraft_segment::TrackingStatus::LowConfidence => "Low confidence",
                    scancraft_segment::TrackingStatus::Lost { .. } => "Lost",
                }
            );
            kit::value_text(ui, &label, m.status.is_lost());
        });
    }

    if let Some(labels) = &ai.labels {
        let kept = labels.iter().filter(|l| **l == scancraft_segment::PointLabel::Object).count();
        kit::row(ui, "3D object", |ui| {
            kit::value_text(ui, &format!("{} of {} pts", thousands(kept), thousands(ai.input.len())), kept == 0);
        });
        if !ai.provider_label.is_empty() {
            kit::row(ui, "Adapter", |ui| kit::value_text(ui, &ai.provider_label, false));
        }
    }

    ui.add_space(t.space_4);
    let picks_len = ai.current_prompts().len();
    let any_prompts = ai.any_prompts();
    let mut do_undo = false;
    let mut do_propagate = false;

    ui.horizontal(|ui| {
        if kit::button(ui, "Undo point", Button::Normal, picks_len > 0 && !busy).clicked() {
            do_undo = true;
        }
        if kit::button(ui, "Propagate all", Button::Primary, any_prompts && !busy)
            .on_hover_text("Track the prompted mask across all RGB keyframes and project onto 3D points")
            .clicked()
        {
            do_propagate = true;
        }
    });

    ui.add_space(t.space_4);
    kit::note(ui, "Click pixels on the left keyframe to segment the object with SAM 2.1, then run Propagate all before clicking Keep object.");

    if let Some(idx) = select_kf {
        app.select_ai_keyframe(idx);
    } else if do_undo {
        app.undo_ai_pick();
    } else if do_propagate {
        app.propagate_ai_segment();
    }
}

/// Renders the optional Live AI segmentation controls inside the Scan stage properties card.
pub fn scan_live_ai_controls(app: &mut ScanCraftApp, ui: &mut egui::Ui) {
    let t = Tokens::get();
    kit::section(ui, "live_ai", "AI Mask (Experimental)", false, |ui| {
        if !app.ai_status.ready {
            kit::note(ui, "AI Pack required for live SAM 2.1 colour segmentation.");
            if kit::wide_button(ui, "Setup AI Pack…", Button::Normal, app.capture == Capture::Idle).clicked() {
                app.ai_pack_modal = true;
            }
            return;
        }
        let can_toggle = app.connection.as_ref().is_some_and(|c| c.capabilities.has_rgb_camera);
        let mut enabled = app.settings.live_ai_segment;
        if ui.add_enabled(can_toggle, egui::Checkbox::new(&mut enabled, "Segment live RGB frames")).changed() {
            app.settings.live_ai_segment = enabled;
            app.sync_live_ai_setting();
        }
        if let Some((iou, provider)) = &app.live_ai_status {
            kit::row(ui, "SAM 2.1", |ui| {
                let unstable = *iou < PostprocessConfig::default().min_iou;
                kit::value_text(ui, &format!("IoU {iou:.2} · {provider}"), unstable);
            });
        }
        kit::note(ui, "Left-click the Colour camera view while capturing to lock SAM 2.1 onto the object.");
        ui.add_space(t.space_2);
    });
}
