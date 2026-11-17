//! SolidCraft's UI icon masters (`assets/icons/ui/design/svg`), rasterised at the pixel size
//! they are shown.
//!
//! The masters name palette roles by their stable source hex (`palette.toml`); this module maps
//! each role to its dark-theme value, as SolidCraft's `icon_svg.rs` does, and rasterises with
//! resvg. The `gap` role (`#ffffff`) erases the artwork drawn before it to transparency; that is
//! expressed as an SVG mask so the icon reads on any background.

use std::collections::HashMap;
use std::sync::Arc;

use egui::{Color32, ColorImage, Painter, Rect, TextureHandle, TextureOptions, pos2};

include!(concat!(env!("OUT_DIR"), "/icon_data.rs"));

/// Dark-theme values of the palette roles, keyed by their source hex (`palette.toml`).
const ROLES: &[(&str, &str)] = &[
    ("#34465b", "#1f2021"), // ink
    ("#e2e8ef", "#8da3c3"), // passive-1
    ("#b8c3cf", "#6d81a2"), // passive-2
    ("#8e9faf", "#546782"), // passive-3
    ("#6f879c", "#8093a4"), // guide
    ("#b9ddf8", "#4ea1dc"), // active-1
    ("#328bd3", "#3287d1"), // active-2
    ("#236ead", "#2976b8"), // active-3
];
/// `line` role: art drawn straight on the UI background (`currentColor`).
const LINE_DARK: &str = "#dbe4ee";
const GAP: &str = "#ffffff";

/// Whether an icon of this name exists.
pub fn exists(name: &str) -> bool {
    ICONS.iter().any(|(n, _, _)| *n == name)
}

fn source(name: &str) -> &'static str {
    ICONS
        .iter()
        .find(|(n, _, _)| *n == name)
        .or_else(|| ICONS.iter().find(|(n, _, _)| *n == "placeholder"))
        .map(|(_, _, s)| *s)
        .unwrap_or("<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"0 0 64 64\"/>")
}

/// Split the children of the root `<svg>` element into top-level element strings.
fn top_level_elements(svg: &str) -> (String, Vec<String>) {
    let open_end = svg.find('>').map(|i| i + 1).unwrap_or(0);
    let header = svg[..open_end].to_string();
    let close = svg.rfind("</svg>").unwrap_or(svg.len());
    let body = &svg[open_end..close];
    let mut elements = Vec::new();
    let mut depth = 0i32;
    let mut start = None;
    let mut i = 0;
    let bytes = body.as_bytes();
    while i < bytes.len() {
        if bytes[i] == b'<' {
            let end = body[i..].find('>').map(|e| i + e + 1).unwrap_or(bytes.len());
            let tag = &body[i..end];
            let closing = tag.starts_with("</");
            let self_closing = tag.ends_with("/>");
            if !closing && depth == 0 {
                start = Some(i);
            }
            if closing {
                depth -= 1;
            } else if !self_closing {
                depth += 1;
            }
            if depth == 0 {
                if let Some(s) = start.take() {
                    elements.push(body[s..end].to_string());
                }
            }
            i = end;
        } else {
            i += 1;
        }
    }
    (header, elements)
}

/// The master with roles resolved (or every colour set to white for `ink`), gaps as masks.
pub fn resolve(svg: &str, ink: bool) -> String {
    let (header, elements) = top_level_elements(svg);
    let mut acc = String::new();
    let mut masks = 0;
    for el in elements {
        if el.starts_with("<title") {
            continue;
        }
        let is_gap = el.contains(GAP) && !el.starts_with("<g") && !el.starts_with("<mask") && !el.starts_with("<clipPath");
        if is_gap && !acc.is_empty() {
            let cut = el.replace(GAP, "#000000");
            acc = format!(
                "<mask id=\"gap{masks}\" maskUnits=\"userSpaceOnUse\" x=\"-8\" y=\"-8\" width=\"80\" height=\"80\"><rect x=\"-8\" y=\"-8\" width=\"80\" height=\"80\" fill=\"#ffffff\"/>{cut}</mask><g mask=\"url(#gap{masks})\">{acc}</g>"
            );
            masks += 1;
            continue;
        }
        let mut el = el;
        if ink {
            for (src, _) in ROLES {
                el = el.replace(src, "#ffffff");
            }
            el = el.replace("currentColor", "#ffffff");
        } else {
            for (src, dark) in ROLES {
                el = el.replace(src, dark);
            }
            el = el.replace("currentColor", LINE_DARK);
        }
        acc.push_str(&el);
    }
    format!("{header}{acc}</svg>")
}

/// Rasterise a resolved SVG into a square image `px` pixels wide.
pub fn rasterise(svg: &str, px: u32) -> Option<ColorImage> {
    let px = px.clamp(1, 512);
    let tree = resvg::usvg::Tree::from_str(svg, &resvg::usvg::Options::default()).ok()?;
    let mut pixmap = resvg::tiny_skia::Pixmap::new(px, px)?;
    let size = tree.size();
    let scale = px as f32 / size.width().max(size.height());
    resvg::render(&tree, resvg::tiny_skia::Transform::from_scale(scale, scale), &mut pixmap.as_mut());
    Some(ColorImage::from_rgba_premultiplied([px as usize, px as usize], pixmap.data()))
}

#[derive(Clone, Default)]
struct Cache(Arc<std::sync::Mutex<HashMap<(String, u32, bool), TextureHandle>>>);

fn texture(ctx: &egui::Context, name: &str, px: u32, ink: bool) -> Option<TextureHandle> {
    let cache = ctx.data_mut(|d| d.get_temp_mut_or_default::<Cache>(egui::Id::new("scancraft_icon_cache")).clone());
    let key = (name.to_string(), px, ink);
    if let Some(tex) = cache.0.lock().ok()?.get(&key) {
        return Some(tex.clone());
    }
    let image = rasterise(&resolve(source(name), ink), px)?;
    let tex = ctx.load_texture(format!("icon:{name}:{px}:{ink}"), image, TextureOptions::LINEAR);
    cache.0.lock().ok()?.insert(key, tex.clone());
    Some(tex)
}

fn paint(painter: &Painter, rect: Rect, name: &str, ink: bool, tint: Color32) {
    if !painter.is_visible() || !painter.clip_rect().intersects(rect) {
        return;
    }
    let ppp = painter.ctx().pixels_per_point();
    let px = (rect.width().max(rect.height()) * ppp).round() as u32;
    // Snap to whole pixels so the raster maps one to one.
    let min = pos2((rect.min.x * ppp).round() / ppp, (rect.min.y * ppp).round() / ppp);
    let rect = Rect::from_min_size(min, egui::vec2(px as f32 / ppp, px as f32 / ppp));
    if let Some(tex) = texture(painter.ctx(), name, px, ink) {
        painter.image(tex.id(), rect, Rect::from_min_max(pos2(0.0, 0.0), pos2(1.0, 1.0)), tint);
    }
}

/// Full-colour icon; `opacity` fades it (a disabled command draws at 0.35).
pub fn paint_icon(painter: &Painter, rect: Rect, name: &str, opacity: f32) {
    paint(painter, rect, name, false, Color32::WHITE.gamma_multiply(opacity.clamp(0.0, 1.0)));
}

/// One-colour icon in `ink` (flat line art: chevrons, close, navigation tools).
pub fn paint_ink(painter: &Painter, rect: Rect, name: &str, ink: Color32) {
    paint(painter, rect, name, true, ink);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_master_resolves_and_rasterises() {
        for (name, _, svg) in ICONS {
            for ink in [false, true] {
                let resolved = resolve(svg, ink);
                assert!(rasterise(&resolved, 24).is_some(), "{name} (ink {ink}) failed to render");
            }
        }
    }

    #[test]
    fn gaps_become_masks_and_roles_resolve() {
        let resolved = resolve(source("play"), false);
        assert!(resolved.contains("<mask"), "{resolved}");
        assert!(!resolved.contains("#34465b"));
        assert!(resolved.contains("#1f2021"));
    }
}
