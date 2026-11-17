//! Colours, spacing and type sizes, ported from SolidCraft's dark palette.
//!
//! Source: `scratch/solidcraft/assets/styles/ui/palettes.rs` (`Tokens::light()` for the shared
//! dimensions, `Tokens::dark()` for the colours) and `crates/ui-egui/src/theme.rs` (`visuals`,
//! `apply`, `shell_shadow`). SolidCraft is MIT OR Apache-2.0; see `apps/scancraft/ATTRIBUTION.md`.
//!
//! Only the tokens ScanCraft draws with are carried over; every value is SolidCraft's own.

use egui::{Color32, Stroke};

/// The design tokens every ScanCraft widget draws with.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Tokens {
    // Spacing scale.
    pub space_2: f32,
    pub space_3: f32,
    pub space_4: f32,
    pub space_6: f32,
    pub space_8: f32,
    pub space_10: f32,
    pub space_12: f32,
    pub space_14: f32,
    pub space_16: f32,
    pub space_24: f32,
    // Corner radii.
    pub radius_sm: u8,
    pub radius_md: u8,
    pub radius_lg: u8,
    pub radius_xl: u8,
    // Type scale.
    pub font_badge: f32,
    pub font_micro: f32,
    pub font_small: f32,
    pub font_caption: f32,
    pub font_body: f32,
    pub font_label: f32,
    pub font_button: f32,
    pub font_heading: f32,
    pub font_section_tracking: f32,
    // Dense form metrics.
    pub control_height: f32,
    pub control_radius: u8,
    pub form_label_width: f32,
    pub form_gap_y: f32,
    pub form_footer_height: f32,
    pub form_header_height: f32,
    pub form_card_width: f32,
    pub form_icon_size: f32,
    pub form_close_size: f32,
    pub form_inset: f32,
    pub form_icon_inset: f32,
    pub form_section_icon_size: f32,
    pub form_checkbox_height: f32,
    pub form_button_height: f32,
    // Shell surfaces.
    pub app_bar: Color32,
    pub app_bar_separator: Color32,
    pub app_bar_text: Color32,
    pub app_bar_hover: Color32,
    pub app_bar_hover_button: Color32,
    pub toolbar: Color32,
    pub panel: Color32,
    pub panel_header: Color32,
    pub border: Color32,
    pub text: Color32,
    pub text_dim: Color32,
    pub accent: Color32,
    pub accent_soft: Color32,
    pub on_accent: Color32,
    pub icon: Color32,
    pub hover: Color32,
    pub timeline: Color32,
    pub timeline_item: Color32,
    pub timeline_marker: Color32,
    pub overlay: Color32,
    pub field: Color32,
    // Status colours.
    pub agent_ok: Color32,
    pub agent_busy: Color32,
    pub error: Color32,
    pub warning: Color32,
    // Dialogs and dense forms.
    pub dialog_bg: Color32,
    pub dialog_border: Color32,
    pub dialog_body: Color32,
    pub dialog_footer: Color32,
    pub dialog_rule: Color32,
    pub dialog_label: Color32,
    pub control_fill: Color32,
    pub control_border: Color32,
    pub selected_ink: Color32,
    pub selected_bg: Color32,
    pub form_ink: Color32,
    pub form_dim: Color32,
    pub form_primary: Color32,
    pub form_primary_fill: Color32,
    pub form_error: Color32,
    pub form_error_fill: Color32,
    pub form_success: Color32,
    pub form_success_fill: Color32,
    pub tab_on_fill_dark: Color32,
    // Viewport navigation chrome.
    pub nav_hover: Color32,
    pub nav_press: Color32,
    pub nav_dim: Color32,
    pub nav_viewport: Color32,
    pub nav_tip: Color32,
    pub nav_tip_ink: Color32,
    // The 3D view.
    pub viewport_top: Color32,
    pub viewport_bottom: Color32,
    pub grid: Color32,
    pub grid_major: Color32,
    pub body: Color32,
    pub body_edge: Color32,
    pub body_selected: Color32,
    pub axis_x: Color32,
    pub axis_y: Color32,
    pub axis_z: Color32,
    pub origin_point: Color32,
    pub rubber_band: Color32,
    pub sel_edge: Color32,
    pub mesh_accent: Color32,
    pub mesh_ink: Color32,
    pub mesh_soft: Color32,
    pub sketch_ink: Color32,
    pub sketch_soft: Color32,
    /// The application mark's three faces: lighter, the accent, darker.
    pub logo: [Color32; 3],
}

impl Default for Tokens {
    fn default() -> Self {
        Self::dark()
    }
}

impl Tokens {
    /// SolidCraft's built-in dark palette (the default theme). Dimensions are shared with its
    /// light palette; colours are the dark overrides.
    pub const fn dark() -> Tokens {
        let text = Color32::from_rgb(222, 226, 232);
        let accent = Color32::from_rgb(58, 136, 238);
        let error = Color32::from_rgb(240, 100, 96);
        Tokens {
            space_2: 2.0,
            space_3: 3.0,
            space_4: 4.0,
            space_6: 6.0,
            space_8: 8.0,
            space_10: 10.0,
            space_12: 12.0,
            space_14: 14.0,
            space_16: 16.0,
            space_24: 24.0,
            radius_sm: 4,
            radius_md: 6,
            radius_lg: 8,
            radius_xl: 10,
            font_badge: 9.5,
            font_micro: 10.0,
            font_small: 11.0,
            font_caption: 11.5,
            font_body: 12.0,
            font_label: 12.5,
            font_button: 13.0,
            font_heading: 14.0,
            font_section_tracking: 0.04,
            control_height: 28.0,
            control_radius: 6,
            form_label_width: 70.0,
            form_gap_y: 5.0,
            form_footer_height: 48.0,
            form_header_height: 40.0,
            form_card_width: 264.0,
            form_icon_size: 19.0,
            form_close_size: 26.0,
            form_inset: 9.0,
            form_icon_inset: 4.5,
            form_section_icon_size: 11.0,
            form_checkbox_height: 22.0,
            form_button_height: 28.0,
            app_bar: Color32::from_rgb(28, 31, 38),
            app_bar_separator: Color32::from_rgb(136, 145, 160),
            app_bar_text: Color32::from_rgb(230, 234, 242),
            app_bar_hover: Color32::from_rgba_premultiplied(28, 28, 28, 28),
            app_bar_hover_button: Color32::from_rgba_premultiplied(30, 30, 30, 30),
            toolbar: Color32::from_rgb(39, 43, 52),
            panel: Color32::from_rgb(39, 43, 52),
            panel_header: Color32::from_rgb(47, 52, 62),
            border: Color32::from_rgb(63, 69, 82),
            text,
            text_dim: Color32::from_rgb(148, 155, 168),
            accent,
            accent_soft: Color32::from_rgb(42, 70, 110),
            on_accent: Color32::WHITE,
            icon: Color32::from_rgb(208, 214, 226),
            hover: Color32::from_rgb(54, 61, 74),
            timeline: Color32::from_rgb(33, 37, 45),
            timeline_item: Color32::from_rgb(52, 58, 70),
            timeline_marker: Color32::from_rgb(170, 178, 192),
            // rgba(34, 38, 46, 240), premultiplied.
            overlay: Color32::from_rgba_premultiplied(32, 36, 43, 240),
            field: Color32::from_rgb(30, 34, 41),
            agent_ok: Color32::from_rgb(76, 194, 122),
            agent_busy: Color32::from_rgb(108, 180, 240),
            error,
            warning: Color32::from_rgb(236, 170, 70),
            dialog_bg: Color32::from_rgb(39, 43, 52),
            dialog_border: Color32::from_rgb(63, 69, 82),
            dialog_body: Color32::from_rgb(39, 43, 52),
            dialog_footer: Color32::from_rgb(47, 52, 62),
            dialog_rule: Color32::from_rgb(53, 59, 71),
            dialog_label: text,
            control_fill: Color32::from_rgb(28, 31, 38),
            control_border: Color32::from_rgb(74, 82, 98),
            selected_ink: Color32::from_rgb(156, 199, 245),
            selected_bg: Color32::from_rgb(42, 70, 110),
            form_ink: text,
            form_dim: Color32::from_rgb(180, 187, 200),
            form_primary: accent,
            form_primary_fill: accent,
            form_error: error,
            form_error_fill: Color32::from_rgb(110, 50, 54),
            form_success: Color32::from_rgb(143, 219, 154),
            form_success_fill: Color32::from_rgb(44, 74, 51),
            tab_on_fill_dark: Color32::BLACK,
            nav_hover: Color32::from_rgb(58, 65, 80),
            nav_press: Color32::from_rgb(70, 78, 96),
            nav_dim: Color32::from_rgb(124, 135, 154),
            nav_viewport: Color32::from_rgb(38, 43, 53),
            nav_tip: Color32::from_rgb(238, 241, 245),
            nav_tip_ink: Color32::from_rgb(27, 36, 48),
            viewport_top: Color32::from_rgb(46, 52, 64),
            viewport_bottom: Color32::from_rgb(46, 52, 64),
            grid: Color32::from_rgb(59, 65, 76),
            grid_major: Color32::from_rgb(73, 79, 90),
            body: Color32::from_rgb(152, 156, 162),
            body_edge: Color32::from_rgb(16, 16, 18),
            body_selected: Color32::from_rgb(110, 170, 235),
            axis_x: Color32::from_rgb(220, 60, 60),
            axis_y: Color32::from_rgb(60, 170, 60),
            axis_z: Color32::from_rgb(60, 100, 230),
            origin_point: Color32::from_rgb(214, 215, 218),
            rubber_band: Color32::from_rgb(92, 160, 255),
            sel_edge: Color32::from_rgb(0, 127, 255),
            mesh_accent: Color32::from_rgb(133, 113, 0),
            mesh_ink: Color32::from_rgb(212, 183, 38),
            mesh_soft: Color32::from_rgb(65, 57, 26),
            sketch_ink: Color32::from_rgb(70, 190, 120),
            sketch_soft: Color32::from_rgb(31, 59, 43),
            logo: [Color32::from_rgb(96, 158, 236), Color32::from_rgb(58, 136, 238), Color32::from_rgb(55, 108, 197)],
        }
    }

    /// The tokens in use. ScanCraft ships SolidCraft's dark theme only.
    pub fn get() -> Tokens {
        Tokens::dark()
    }

    /// One-pixel border in the primary role (SolidCraft's `primary_border` with legacy borders).
    pub fn primary_border(&self, color: Color32) -> Stroke {
        Stroke::new(1.0, color)
    }

    /// Control outline (`control_border_stroke`).
    pub fn control_border_stroke(&self, color: Color32) -> Stroke {
        Stroke::new(1.0, color)
    }

    /// Subtle shell elevation, derived from the application-bar colour (`shell_shadow`).
    pub fn shell_shadow(&self) -> egui::Shadow {
        egui::Shadow { offset: [0, self.space_2 as i8], blur: self.space_8 as u8, spread: 0, color: self.app_bar.gamma_multiply(0.18) }
    }

    /// A one-pixel horizontal rule (`secondary_hline`).
    pub fn secondary_hline(&self, painter: &egui::Painter, range: impl Into<egui::emath::Rangef>, y: f32, color: Color32) {
        painter.hline(range, y, Stroke::new(1.0, color));
    }

    /// A one-pixel vertical rule (`secondary_vline`).
    pub fn secondary_vline(&self, painter: &egui::Painter, x: f32, range: impl Into<egui::emath::Rangef>, color: Color32) {
        painter.vline(x, range, Stroke::new(1.0, color));
    }

    /// The X, Y and Z axis colours, in that order.
    pub const fn axes(&self) -> [Color32; 3] {
        [self.axis_x, self.axis_y, self.axis_z]
    }
}

/// WCAG contrast ratio between two colours.
pub fn contrast_ratio(a: Color32, b: Color32) -> f64 {
    fn lum(c: Color32) -> f64 {
        let ch = |v: u8| {
            let v = v as f64 / 255.0;
            if v <= 0.03928 { v / 12.92 } else { ((v + 0.055) / 1.055).powf(2.4) }
        };
        0.2126 * ch(c.r()) + 0.7152 * ch(c.g()) + 0.0722 * ch(c.b())
    }
    let (la, lb) = (lum(a), lum(b));
    let (hi, lo) = if la > lb { (la, lb) } else { (lb, la) };
    (hi + 0.05) / (lo + 0.05)
}

/// egui visuals from the tokens (SolidCraft `theme::visuals(dark = true)`).
pub fn visuals(t: &Tokens) -> egui::Visuals {
    let mut v = egui::Visuals::dark();
    v.panel_fill = t.panel;
    v.window_fill = t.panel;
    v.window_stroke = t.primary_border(t.border);
    v.extreme_bg_color = t.field;
    v.faint_bg_color = t.panel_header;
    v.override_text_color = Some(t.text);
    v.widgets.inactive.weak_bg_fill = t.panel_header;
    v.widgets.inactive.bg_fill = t.panel_header;
    v.widgets.active.weak_bg_fill = t.accent_soft;
    v.selection.bg_fill = t.accent_soft;
    v.selection.stroke = Stroke::new(1.0, t.accent);
    v.widgets.noninteractive.bg_stroke = Stroke::new(1.0, t.border);
    v.widgets.noninteractive.fg_stroke = Stroke::new(1.0, t.text);
    v.widgets.inactive.fg_stroke = Stroke::new(1.0, t.text);
    v.widgets.hovered.weak_bg_fill = t.hover;
    v.widgets.hovered.bg_fill = t.hover;
    v.widgets.open.weak_bg_fill = t.panel_header;
    v.window_corner_radius = egui::CornerRadius::same(t.radius_md);
    v.menu_corner_radius = egui::CornerRadius::same(t.radius_md);
    for widget in [&mut v.widgets.noninteractive, &mut v.widgets.inactive, &mut v.widgets.active, &mut v.widgets.open] {
        widget.corner_radius = egui::CornerRadius::same(t.radius_sm / 2);
    }
    v.widgets.hovered.corner_radius = egui::CornerRadius::same(t.radius_sm.saturating_sub(1));
    v.window_shadow = t.shell_shadow();
    v.popup_shadow = t.shell_shadow();
    v
}

/// Install the theme: visuals, spacing and the IBM Plex faces (SolidCraft `theme::apply` and
/// `fonts::setup_bundled_fonts`).
pub fn apply(ctx: &egui::Context) {
    let t = Tokens::get();
    ctx.set_theme(egui::Theme::Dark);
    ctx.set_visuals_of(egui::Theme::Dark, visuals(&t));
    crate::fonts::install(ctx);
    ctx.all_styles_mut(|s| {
        s.spacing.item_spacing = egui::vec2(t.space_6, t.space_4);
        s.spacing.button_padding = egui::vec2(t.space_6, t.space_3);
        s.text_styles.insert(egui::TextStyle::Body, egui::FontId::proportional(t.font_label));
        s.text_styles.insert(egui::TextStyle::Button, egui::FontId::proportional(t.font_label));
        s.text_styles.insert(egui::TextStyle::Small, egui::FontId::proportional(t.font_small));
        s.text_styles.insert(egui::TextStyle::Heading, crate::fonts::semibold(t.font_heading));
        s.text_styles.insert(egui::TextStyle::Monospace, egui::FontId::monospace(t.font_body));
        s.interaction.tooltip_delay = 0.5;
    });
    ctx.request_repaint();
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn text_reads_on_every_surface() {
        let t = Tokens::dark();
        for bg in [t.panel, t.toolbar, t.app_bar, t.control_fill, t.viewport_top] {
            assert!(contrast_ratio(t.text, bg) >= 7.0, "text on {bg:?}");
        }
        for bg in [t.panel, t.toolbar, t.app_bar, t.control_fill] {
            assert!(contrast_ratio(t.text_dim, bg) >= 4.5, "dim text on {bg:?}");
        }
        assert!(contrast_ratio(t.selected_ink, t.selected_bg) >= 4.5);
        assert!(contrast_ratio(t.on_accent, t.form_primary_fill) >= 3.0);
    }

    #[test]
    fn palette_matches_solidcraft_dark() {
        let t = Tokens::dark();
        assert_eq!(t.viewport_top, Color32::from_rgb(46, 52, 64));
        assert_eq!(t.panel, Color32::from_rgb(39, 43, 52));
        assert_eq!(t.accent, Color32::from_rgb(58, 136, 238));
        assert_eq!(t.control_height, 28.0);
    }
}
