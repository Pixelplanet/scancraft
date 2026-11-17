//! Bundled IBM Plex interface faces (OFL), installed at real weights from the variable font.
//! Ported from SolidCraft `crates/ui-egui/src/fonts.rs` (`base_font_definitions`, weight helpers).

use std::sync::Arc;

use egui::{FontDefinitions, FontFamily, FontId, RichText};

const PLEX_SANS: &[u8] = include_bytes!("../assets/fonts/IBMPlexSans-Variable.ttf");
const PLEX_MONO: &[u8] = include_bytes!("../assets/fonts/IBMPlexMono-Regular.ttf");
const WEIGHTS: [(&str, f32); 4] = [("plex-sans", 400.0), ("ui-medium", 500.0), ("ui-semibold", 600.0), ("ui-bold", 700.0)];

fn sans_families() -> [FontFamily; 4] {
    [FontFamily::Proportional, medium_family(), semibold_family(), bold_family()]
}

/// IBM Plex Sans at 400/500/600/700 and Plex Mono Regular, followed by egui's symbol fallbacks.
pub fn base_font_definitions() -> FontDefinitions {
    let mut fonts = FontDefinitions::default();
    let fallbacks = fonts.families.get(&FontFamily::Proportional).cloned().unwrap_or_default();
    for ((name, weight), family) in WEIGHTS.into_iter().zip(sans_families()) {
        let mut data = egui::FontData::from_static(PLEX_SANS);
        data.tweak.coords = egui::epaint::text::VariationCoords::new([(b"wght", weight)]);
        fonts.font_data.insert(name.to_owned(), Arc::new(data));
        fonts.families.insert(family, std::iter::once(name.to_owned()).chain(fallbacks.iter().cloned()).collect());
    }
    fonts.font_data.insert("plex-mono".to_owned(), Arc::new(egui::FontData::from_static(PLEX_MONO)));
    fonts.families.entry(FontFamily::Monospace).or_default().insert(0, "plex-mono".to_owned());
    fonts
}

pub fn install(ctx: &egui::Context) {
    ctx.set_fonts(base_font_definitions());
}

fn medium_family() -> FontFamily {
    FontFamily::Name("ui-medium".into())
}

fn semibold_family() -> FontFamily {
    FontFamily::Name("ui-semibold".into())
}

fn bold_family() -> FontFamily {
    FontFamily::Name("ui-bold".into())
}

/// Medium (500) interface text.
pub fn medium(size: f32) -> FontId {
    FontId::new(size, medium_family())
}

/// Semibold (600) interface text.
pub fn semibold(size: f32) -> FontId {
    FontId::new(size, semibold_family())
}

/// Bold (700) interface text.
pub fn bold(size: f32) -> FontId {
    FontId::new(size, bold_family())
}

/// Select a real interface weight without changing the text's size or colour.
pub trait RichTextExt {
    fn medium(self) -> Self;
    fn semibold(self) -> Self;
    fn bold_face(self) -> Self;
}

impl RichTextExt for RichText {
    fn medium(self) -> Self {
        self.family(medium_family())
    }
    fn semibold(self) -> Self {
        self.family(semibold_family())
    }
    fn bold_face(self) -> Self {
        self.family(bold_family())
    }
}
