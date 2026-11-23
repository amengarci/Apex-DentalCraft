//! Identidad Visual de **Apex DentalCraft** (estética *Craft + paleta Apex Dental Lab)
//!
//! - Fondo principal ciruela imperial oscuro: `#190321`
//! - Paneles y tarjetas flotantes: `#23062E` / `#21052C`
//! - Tipografía principal y contraste noble: Crema `#F2EAD3`
//! - Acento interactivo CAD: Amatista `#9D4EDD`
//! - Acento clínico / validación / línea de margen: Esmeralda `#34D399`

use egui::{Color32, CornerRadius, Margin, Stroke, Style, Visuals};

pub const BG_DEEP: Color32 = Color32::from_rgb(0x12, 0x02, 0x18);
pub const BG_MAIN: Color32 = Color32::from_rgb(0x19, 0x03, 0x21);
pub const BG_PANEL: Color32 = Color32::from_rgb(0x23, 0x06, 0x2E);
pub const BG_CARD: Color32 = Color32::from_rgb(0x2B, 0x07, 0x38);
pub const BG_WIDGET: Color32 = Color32::from_rgb(0x31, 0x0A, 0x40);
pub const BG_WIDGET_HOVER: Color32 = Color32::from_rgb(0x43, 0x10, 0x57);
pub const BORDER_PLUM: Color32 = Color32::from_rgb(0x4E, 0x18, 0x64);

pub const CREAM: Color32 = Color32::from_rgb(0xF2, 0xEA, 0xD3);
pub const CREAM_MUTED: Color32 = Color32::from_rgb(0xC9, 0xBE, 0xA3);
pub const AMETHYST: Color32 = Color32::from_rgb(0x9D, 0x4E, 0xDD);
pub const AMETHYST_BRIGHT: Color32 = Color32::from_rgb(0xC7, 0x7D, 0xFF);
pub const AMETHYST_DARK: Color32 = Color32::from_rgb(0x6B, 0x21, 0xA8);
pub const EMERALD: Color32 = Color32::from_rgb(0x34, 0xD3, 0x99);
pub const TEAL: Color32 = Color32::from_rgb(0x14, 0xB8, 0xA6);
pub const AMBER: Color32 = Color32::from_rgb(0xF5, 0x9E, 0x0B);
pub const CYAN: Color32 = Color32::from_rgb(0x38, 0xBD, 0xF8);
pub const DANGER_BG: Color32 = Color32::from_rgb(0x58, 0x15, 0x28);
pub const DANGER_FG: Color32 = Color32::from_rgb(0xFE, 0xCD, 0xD3);

/// Colores RGBA normalizados (0..1) para el degradado del lienzo OpenGL 3D.
pub const VIEWPORT_BG_BOTTOM: [f32; 3] = [0x12 as f32 / 255.0, 0x02 as f32 / 255.0, 0x18 as f32 / 255.0];
pub const VIEWPORT_BG_TOP: [f32; 3] = [0x2C as f32 / 255.0, 0x08 as f32 / 255.0, 0x38 as f32 / 255.0];

/// Aplica el tema visual completo de Apex DentalCraft sobre el contexto `egui`.
pub fn apply_apex_theme(ctx: &egui::Context) {
    let mut visuals = Visuals::dark();

    visuals.override_text_color = Some(CREAM);
    visuals.panel_fill = BG_MAIN;
    visuals.window_fill = Color32::from_rgba_unmultiplied(0x21, 0x05, 0x2C, 242);
    visuals.window_stroke = Stroke::new(1.5_f32, AMETHYST);
    visuals.window_corner_radius = CornerRadius::same(8);
    visuals.extreme_bg_color = BG_DEEP;
    visuals.faint_bg_color = BG_PANEL;
    visuals.code_bg_color = BG_DEEP;
    visuals.selection.bg_fill = AMETHYST_DARK;
    visuals.selection.stroke = Stroke::new(1.2_f32, CREAM);
    visuals.hyperlink_color = EMERALD;
    visuals.warn_fg_color = AMBER;
    visuals.error_fg_color = Color32::from_rgb(0xF8, 0x71, 0x71);

    // Widgets inactivos / hover / activos
    visuals.widgets.noninteractive.bg_fill = BG_PANEL;
    visuals.widgets.noninteractive.weak_bg_fill = BG_PANEL;
    visuals.widgets.noninteractive.bg_stroke = Stroke::new(1.0_f32, BORDER_PLUM);
    visuals.widgets.noninteractive.fg_stroke = Stroke::new(1.0_f32, CREAM);
    visuals.widgets.noninteractive.corner_radius = CornerRadius::same(6);

    visuals.widgets.inactive.bg_fill = BG_WIDGET;
    visuals.widgets.inactive.weak_bg_fill = BG_WIDGET;
    visuals.widgets.inactive.bg_stroke = Stroke::new(1.0_f32, Color32::from_rgb(0x57, 0x1C, 0x70));
    visuals.widgets.inactive.fg_stroke = Stroke::new(1.0_f32, CREAM);
    visuals.widgets.inactive.corner_radius = CornerRadius::same(6);

    visuals.widgets.hovered.bg_fill = BG_WIDGET_HOVER;
    visuals.widgets.hovered.weak_bg_fill = BG_WIDGET_HOVER;
    visuals.widgets.hovered.bg_stroke = Stroke::new(1.2_f32, AMETHYST);
    visuals.widgets.hovered.fg_stroke = Stroke::new(1.2_f32, Color32::WHITE);
    visuals.widgets.hovered.corner_radius = CornerRadius::same(6);

    visuals.widgets.active.bg_fill = AMETHYST_DARK;
    visuals.widgets.active.weak_bg_fill = AMETHYST_DARK;
    visuals.widgets.active.bg_stroke = Stroke::new(1.5_f32, CREAM);
    visuals.widgets.active.fg_stroke = Stroke::new(1.5_f32, Color32::WHITE);
    visuals.widgets.active.corner_radius = CornerRadius::same(6);

    visuals.widgets.open.bg_fill = BG_CARD;
    visuals.widgets.open.weak_bg_fill = BG_CARD;
    visuals.widgets.open.bg_stroke = Stroke::new(1.2_f32, AMETHYST_BRIGHT);
    visuals.widgets.open.fg_stroke = Stroke::new(1.2_f32, CREAM);
    visuals.widgets.open.corner_radius = CornerRadius::same(6);

    let mut style = Style::default();
    style.visuals = visuals;
    style.spacing.item_spacing = egui::vec2(7.0, 6.0);
    style.spacing.button_padding = egui::vec2(10.0, 5.0);
    style.spacing.window_margin = Margin::same(9);
    style.spacing.slider_width = 128.0;
    ctx.set_style(style);
}

/// Dibuja el emblema vectorial original de Apex DentalCraft (molar estilizado minimalista en crema `#F2EAD3`).
pub fn draw_apex_tooth_emblem(painter: &egui::Painter, rect: egui::Rect) {
    let p = |fx: f32, fy: f32| egui::pos2(rect.left() + rect.width() * fx, rect.top() + rect.height() * fy);
    let cubic = |p0: egui::Pos2, p1: egui::Pos2, p2: egui::Pos2, p3: egui::Pos2, steps: usize, out: &mut Vec<egui::Pos2>| {
        for i in 1..=steps {
            let t = i as f32 / steps as f32;
            let u = 1.0 - t;
            let pt = p0.to_vec2() * (u * u * u)
                + p1.to_vec2() * (3.0 * u * u * t)
                + p2.to_vec2() * (3.0 * u * t * t)
                + p3.to_vec2() * (t * t * t);
            out.push(pt.to_pos2());
        }
    };

    let mut pts = Vec::with_capacity(64);
    let s0 = p(0.50, 0.21);
    pts.push(s0);
    cubic(s0, p(0.36, 0.08), p(0.14, 0.14), p(0.17, 0.39), 10, &mut pts);
    cubic(p(0.17, 0.39), p(0.19, 0.56), p(0.24, 0.74), p(0.32, 0.90), 10, &mut pts);
    cubic(p(0.32, 0.90), p(0.37, 0.94), p(0.42, 0.73), p(0.50, 0.62), 10, &mut pts);
    cubic(p(0.50, 0.62), p(0.58, 0.73), p(0.63, 0.94), p(0.68, 0.90), 10, &mut pts);
    cubic(p(0.68, 0.90), p(0.76, 0.74), p(0.81, 0.56), p(0.83, 0.39), 10, &mut pts);
    cubic(p(0.83, 0.39), p(0.86, 0.14), p(0.64, 0.08), s0, 10, &mut pts);

    painter.add(egui::Shape::convex_polygon(
        pts.clone(),
        Color32::from_rgba_unmultiplied(0x9D, 0x4E, 0xDD, 42),
        Stroke::NONE,
    ));
    painter.add(egui::Shape::closed_line(pts, Stroke::new(2.2_f32, CREAM)));
}

/// Color identificador en el Odontograma para cada proceso protésico.
pub fn process_color(process_type: &str) -> Color32 {
    match process_type {
        "Corona Individual" => AMETHYST,
        "Pilar de Puente" => Color32::from_rgb(0x0D, 0x94, 0x88),
        "Póntico de Puente" => Color32::from_rgb(0x2D, 0xD4, 0xBF),
        "Carilla Estética (Veneer)" => AMBER,
        "Incrustación (Inlay / Onlay)" => Color32::from_rgb(0xE1, 0x1D, 0x48),
        "Cofia / Estructura" => Color32::from_rgb(0x63, 0x66, 0xF1),
        _ => AMETHYST,
    }
}

/// Botón estilizado de acento principal (Crema con texto Ciruela Oscuro).
pub fn primary_button(ui: &mut egui::Ui, label: &str) -> egui::Response {
    ui.add(
        egui::Button::new(egui::RichText::new(label).strong().color(Color32::from_rgb(0x23, 0x03, 0x2E)))
            .fill(CREAM)
            .stroke(Stroke::new(1.0_f32, Color32::WHITE))
            .corner_radius(CornerRadius::same(6)),
    )
}

/// Botón estilizado de acento CAD (Amatista vibrante).
pub fn accent_button(ui: &mut egui::Ui, label: &str) -> egui::Response {
    ui.add(
        egui::Button::new(egui::RichText::new(label).strong().color(CREAM))
            .fill(Color32::from_rgb(0x7B, 0x2C, 0xBF))
            .stroke(Stroke::new(1.0_f32, AMETHYST_BRIGHT))
            .corner_radius(CornerRadius::same(6)),
    )
}

/// Botón estilizado de acento Esmeralda / Smart Tool.
#[allow(dead_code)]
pub fn emerald_button(ui: &mut egui::Ui, label: &str) -> egui::Response {
    ui.add(
        egui::Button::new(egui::RichText::new(label).strong().color(Color32::from_rgb(0x0B, 0x22, 0x1B)))
            .fill(EMERALD)
            .stroke(Stroke::new(1.0_f32, Color32::from_rgb(0xA7, 0xF3, 0xD0)))
            .corner_radius(CornerRadius::same(6)),
    )
}

/// Botón estilizado de acción destructiva / borrar.
pub fn danger_button(ui: &mut egui::Ui, label: &str) -> egui::Response {
    ui.add(
        egui::Button::new(egui::RichText::new(label).strong().color(DANGER_FG))
            .fill(DANGER_BG)
            .stroke(Stroke::new(1.0_f32, Color32::from_rgb(0x9F, 0x12, 0x39)))
            .corner_radius(CornerRadius::same(6)),
    )
}
