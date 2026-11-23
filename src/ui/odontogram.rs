//! Odontograma Dental Anatómico FDI (11-48) en Óvalo Vertical con `egui::Painter`
//!
//! - Distribución anatómica en **Óvalo Vertical** (11/21 alejados verticalmente de 41/31;
//!   18/48 cercanos horizontalmente a 28/38).
//! - Siluetas dentarias anatómicas por número FDI (`1..8`) con surcos oclusales/incisales.
//! - Selección por Puro Clic: clic selecciona un diente, volver a darle clic lo deselecciona.
//! - Círculos conectores interproximales (`●` puente / `○` individual).

use crate::core::case_manager::ToothRestorationUnit;
use crate::core::i18n::{self, tr};
use crate::theme::{self, AMETHYST, AMETHYST_BRIGHT, CREAM, CREAM_MUTED, CYAN, TEAL};
use egui::{Align2, Color32, CornerRadius, FontId, Pos2, Rect, Sense, Shape, Stroke, Vec2};
use std::collections::{BTreeMap, BTreeSet};

pub const UPPER_ARCH: [i32; 16] = [18, 17, 16, 15, 14, 13, 12, 11, 21, 22, 23, 24, 25, 26, 27, 28];
pub const LOWER_ARCH: [i32; 16] = [48, 47, 46, 45, 44, 43, 42, 41, 31, 32, 33, 34, 35, 36, 37, 38];

const ANGULAR_WEIGHTS: [f32; 16] = [
    1.00, 1.12, 1.22, 0.92, 0.90, 0.92, 0.82, 0.95,
    0.95, 0.82, 0.92, 0.90, 0.92, 1.22, 1.12, 1.00,
];

pub fn fdi_tooth_name(fdi: i32) -> &'static str {
    match fdi {
        18 => "Tercer Molar Sup. Der.", 17 => "Segundo Molar Sup. Der.", 16 => "Primer Molar Sup. Der.",
        15 => "Segundo Premolar Sup. Der.", 14 => "Primer Premolar Sup. Der.", 13 => "Canino Sup. Der.",
        12 => "Incisivo Lateral Sup. Der.", 11 => "Incisivo Central Sup. Der.",
        21 => "Incisivo Central Sup. Izq.", 22 => "Incisivo Lateral Sup. Izq.", 23 => "Canino Sup. Izq.",
        24 => "Primer Premolar Sup. Izq.", 25 => "Segundo Premolar Sup. Izq.", 26 => "Primer Molar Sup. Izq.",
        27 => "Segundo Molar Sup. Izq.", 28 => "Tercer Molar Sup. Izq.",
        48 => "Tercer Molar Inf. Der.", 47 => "Segundo Molar Inf. Der.", 46 => "Primer Molar Inf. Der.",
        45 => "Segundo Premolar Inf. Der.", 44 => "Primer Premolar Inf. Der.", 43 => "Canino Inf. Der.",
        42 => "Incisivo Lateral Inf. Der.", 41 => "Incisivo Central Inf. Der.",
        31 => "Incisivo Central Inf. Izq.", 32 => "Incisivo Lateral Inf. Izq.", 33 => "Canino Inf. Izq.",
        34 => "Primer Premolar Inf. Izq.", 35 => "Segundo Premolar Inf. Izq.", 36 => "Primer Molar Inf. Izq.",
        37 => "Segundo Molar Inf. Izq.", 38 => "Tercer Molar Inf. Izq.",
        _ => "",
    }
}

#[derive(Debug, Clone)]
pub enum OdontogramEvent {
    SelectionChanged(Option<i32>),
    ConnectorToggled(i32, i32, bool),
}

#[derive(Debug, Clone)]
pub struct FdiOdontogramState {
    pub selected_tooth: Option<i32>,
    pub selected_teeth: BTreeSet<i32>,
}

impl Default for FdiOdontogramState {
    fn default() -> Self {
        let mut s = BTreeSet::new();
        s.insert(36);
        Self {
            selected_tooth: Some(36),
            selected_teeth: s,
        }
    }
}

impl FdiOdontogramState {
    pub fn set_single(&mut self, fdi: i32) {
        self.selected_teeth.clear();
        self.selected_teeth.insert(fdi);
        self.selected_tooth = Some(fdi);
    }

    pub fn clear(&mut self) {
        self.selected_teeth.clear();
        self.selected_tooth = None;
    }

    pub fn get_selected_vec(&self) -> Vec<i32> {
        self.selected_teeth.iter().copied().collect()
    }

    /// Alterna un diente con un solo clic (selecciona si no estaba; deselecciona si ya estaba).
    pub fn toggle_tooth_click(&mut self, fdi: i32, shift_held: bool) {
        if shift_held {
            if let Some(anchor) = self.selected_tooth {
                for arch in [&UPPER_ARCH, &LOWER_ARCH] {
                    if let (Some(i1), Some(i2)) = (
                        arch.iter().position(|&t| t == anchor),
                        arch.iter().position(|&t| t == fdi),
                    ) {
                        let (lo, hi) = (i1.min(i2), i1.max(i2));
                        for &t in &arch[lo..=hi] {
                            self.selected_teeth.insert(t);
                        }
                        self.selected_tooth = Some(fdi);
                        return;
                    }
                }
            }
        }

        if self.selected_teeth.contains(&fdi) {
            self.selected_teeth.remove(&fdi);
            if self.selected_tooth == Some(fdi) {
                self.selected_tooth = self.selected_teeth.iter().next_back().copied();
            }
        } else {
            self.selected_teeth.insert(fdi);
            self.selected_tooth = Some(fdi);
        }
    }
}

fn eval_cubic(p0: Vec2, p1: Vec2, p2: Vec2, p3: Vec2, steps: usize, out: &mut Vec<Vec2>) {
    for i in 1..=steps {
        let t = i as f32 / steps as f32;
        let u = 1.0 - t;
        out.push(
            p0 * (u * u * u)
                + p1 * (3.0 * u * u * t)
                + p2 * (3.0 * u * t * t)
                + p3 * (t * t * t),
        );
    }
}

/// Genera el polígono anatómico exterior y los segmentos de surcos internos para el diente `fdi`.
fn build_anatomical_tooth_geometry(fdi: i32, scale: f32) -> (Vec<Vec2>, Vec<(Vec2, Vec2)>) {
    let num = fdi % 10;
    let is_upper = matches!(fdi / 10, 1 | 2);
    let mut outer = Vec::with_capacity(48);
    let mut fissures = Vec::new();
    let v = |x: f32, y: f32| Vec2::new(x, y);

    match num {
        1 | 2 => {
            let w = (if num == 1 { if is_upper { 10.2 } else { 7.2 } } else if is_upper { 8.4 } else { 7.6 }) * scale;
            let h = (if num == 1 { if is_upper { 7.0 } else { 6.4 } } else { 6.5 }) * scale;
            let s0 = v(-w * 0.86, -h * 0.72);
            outer.push(s0);
            eval_cubic(s0, v(-w * 0.35, -h * 1.02), v(w * 0.35, -h * 1.02), v(w * 0.86, -h * 0.70), 8, &mut outer);
            eval_cubic(v(w * 0.86, -h * 0.70), v(w * 1.02, -h * 0.18), v(w * 0.80, h * 0.66), v(w * 0.44, h * 0.92), 8, &mut outer);
            eval_cubic(v(w * 0.44, h * 0.92), v(0.0, h * 1.05), v(0.0, h * 1.05), v(-w * 0.44, h * 0.92), 6, &mut outer);
            eval_cubic(v(-w * 0.44, h * 0.92), v(-w * 0.80, h * 0.66), v(-w * 1.02, -h * 0.18), s0, 8, &mut outer);

            fissures.push((v(-w * 0.65, -h * 0.18), v(w * 0.65, -h * 0.18)));
            fissures.push((v(-w * 0.35, h * 0.48), v(0.0, h * 0.62)));
            fissures.push((v(0.0, h * 0.62), v(w * 0.35, h * 0.48)));
        }
        3 => {
            let w = 9.0 * scale;
            let h = 8.2 * scale;
            let s0 = v(0.0, -h * 1.08);
            outer.push(s0);
            eval_cubic(s0, v(w * 0.55, -h * 0.92), v(w * 0.98, -h * 0.45), v(w * 0.95, 0.0), 8, &mut outer);
            eval_cubic(v(w * 0.95, 0.0), v(w * 0.92, h * 0.55), v(w * 0.52, h * 0.98), v(0.0, h * 1.02), 8, &mut outer);
            eval_cubic(v(0.0, h * 1.02), v(-w * 0.52, h * 0.98), v(-w * 0.92, h * 0.55), v(-w * 0.95, 0.0), 8, &mut outer);
            eval_cubic(v(-w * 0.95, 0.0), v(-w * 0.98, -h * 0.45), v(-w * 0.55, -h * 0.92), s0, 8, &mut outer);

            fissures.push((v(-w * 0.62, -h * 0.05), v(0.0, -h * 0.48)));
            fissures.push((v(0.0, -h * 0.48), v(w * 0.62, -h * 0.05)));
            fissures.push((v(0.0, -h * 0.48), v(0.0, h * 0.58)));
        }
        4 | 5 => {
            let w = (if num == 4 { 9.2 } else { 9.6 }) * scale;
            let h = (if num == 4 { 9.4 } else { 9.6 }) * scale;
            let ln = if num == 4 { 0.82 } else { 0.92 };
            let s0 = v(0.0, -h * 1.02);
            outer.push(s0);
            eval_cubic(s0, v(w * 0.75, -h * 1.00), v(w * 1.02, -h * 0.48), v(w * 0.96, 0.0), 8, &mut outer);
            eval_cubic(v(w * 0.96, 0.0), v(w * 0.92, h * 0.52), v(w * 0.68 * ln, h * 1.00), v(0.0, h * 1.00), 8, &mut outer);
            eval_cubic(v(0.0, h * 1.00), v(-w * 0.68 * ln, h * 1.00), v(-w * 0.92, h * 0.52), v(-w * 0.96, 0.0), 8, &mut outer);
            eval_cubic(v(-w * 0.96, 0.0), v(-w * 1.02, -h * 0.48), v(-w * 0.75, -h * 1.00), s0, 8, &mut outer);

            fissures.push((v(-w * 0.52, 0.0), v(w * 0.52, 0.0)));
            fissures.push((v(-w * 0.52, 0.0), v(-w * 0.68, -h * 0.28)));
            fissures.push((v(-w * 0.52, 0.0), v(-w * 0.68, h * 0.28)));
            fissures.push((v(w * 0.52, 0.0), v(w * 0.68, -h * 0.28)));
            fissures.push((v(w * 0.52, 0.0), v(w * 0.68, h * 0.28)));
        }
        6 => {
            let w = 12.2 * scale;
            let h = 10.8 * scale;
            let s0 = v(-w * 0.95, -h * 0.35);
            outer.push(s0);
            if !is_upper {
                eval_cubic(s0, v(-w * 0.85, -h * 1.02), v(-w * 0.38, -h * 1.04), v(-w * 0.30, -h * 0.82), 6, &mut outer);
                eval_cubic(v(-w * 0.30, -h * 0.82), v(-w * 0.15, -h * 1.05), v(w * 0.15, -h * 1.05), v(w * 0.30, -h * 0.82), 6, &mut outer);
                eval_cubic(v(w * 0.30, -h * 0.82), v(w * 0.38, -h * 1.04), v(w * 0.85, -h * 1.02), v(w * 0.95, -h * 0.35), 6, &mut outer);
            } else {
                eval_cubic(s0, v(-w * 0.90, -h * 1.04), v(-w * 0.12, -h * 1.04), v(0.0, -h * 0.80), 8, &mut outer);
                eval_cubic(v(0.0, -h * 0.80), v(w * 0.12, -h * 1.04), v(w * 0.90, -h * 1.04), v(w * 0.95, -h * 0.35), 8, &mut outer);
            }
            eval_cubic(v(w * 0.95, -h * 0.35), v(w * 1.04, h * 0.15), v(w * 0.90, h * 0.98), v(w * 0.42, h * 0.98), 7, &mut outer);
            eval_cubic(v(w * 0.42, h * 0.98), v(w * 0.12, h * 0.98), v(0.05, h * 0.78), v(0.0, h * 0.78), 5, &mut outer);
            eval_cubic(v(0.0, h * 0.78), v(-0.05, h * 0.78), v(-w * 0.12, h * 0.98), v(-w * 0.42, h * 0.98), 5, &mut outer);
            eval_cubic(v(-w * 0.42, h * 0.98), v(-w * 0.90, h * 0.98), v(-w * 1.04, h * 0.15), s0, 7, &mut outer);

            fissures.push((v(-w * 0.68, 0.0), v(w * 0.68, 0.0)));
            fissures.push((v(0.0, -h * 0.68), v(0.0, h * 0.68)));
            if !is_upper {
                fissures.push((v(w * 0.26, 0.0), v(w * 0.42, -h * 0.62)));
            }
        }
        7 => {
            let w = 11.2 * scale;
            let h = 10.2 * scale;
            let s0 = v(-w * 0.94, -h * 0.30);
            outer.push(s0);
            eval_cubic(s0, v(-w * 0.88, -h * 1.02), v(-w * 0.12, -h * 1.02), v(0.0, -h * 0.80), 7, &mut outer);
            eval_cubic(v(0.0, -h * 0.80), v(w * 0.12, -h * 1.02), v(w * 0.88, -h * 1.02), v(w * 0.94, -h * 0.30), 7, &mut outer);
            eval_cubic(v(w * 0.94, -h * 0.30), v(w * 1.02, h * 0.20), v(w * 0.88, h * 1.00), v(w * 0.38, h * 0.98), 7, &mut outer);
            eval_cubic(v(w * 0.38, h * 0.98), v(0.0, h * 0.78), v(0.0, h * 0.78), v(-w * 0.38, h * 0.98), 6, &mut outer);
            eval_cubic(v(-w * 0.38, h * 0.98), v(-w * 0.88, h * 1.00), v(-w * 1.02, h * 0.20), s0, 7, &mut outer);

            fissures.push((v(-w * 0.64, 0.0), v(w * 0.64, 0.0)));
            fissures.push((v(0.0, -h * 0.64), v(0.0, h * 0.64)));
        }
        _ => {
            let w = 10.0 * scale;
            let h = 9.4 * scale;
            let s0 = v(-w * 0.90, -h * 0.25);
            outer.push(s0);
            eval_cubic(s0, v(-w * 0.82, -h * 0.98), v(-w * 0.10, -h * 0.98), v(0.0, -h * 0.78), 7, &mut outer);
            eval_cubic(v(0.0, -h * 0.78), v(w * 0.10, -h * 0.98), v(w * 0.82, -h * 0.98), v(w * 0.90, -h * 0.25), 7, &mut outer);
            eval_cubic(v(w * 0.90, -h * 0.25), v(w * 0.98, h * 0.35), v(w * 0.62, h * 0.98), v(0.0, h * 0.98), 7, &mut outer);
            eval_cubic(v(0.0, h * 0.98), v(-w * 0.62, h * 0.98), v(-w * 0.98, h * 0.35), s0, 7, &mut outer);

            fissures.push((v(-w * 0.52, -h * 0.05), v(w * 0.52, -h * 0.05)));
            fissures.push((v(0.0, -h * 0.55), v(0.0, h * 0.52)));
        }
    }
    (outer, fissures)
}

fn compute_arch_angles(start_deg: f32, end_deg: f32) -> [f32; 16] {
    let total_w: f32 = ANGULAR_WEIGHTS.iter().sum();
    let span = end_deg - start_deg;
    let mut out = [0.0_f32; 16];
    let mut acc = 0.0_f32;
    for (i, &w) in ANGULAR_WEIGHTS.iter().enumerate() {
        let mid_frac = (acc + w * 0.5) / total_w;
        out[i] = (start_deg + mid_frac * span).to_radians();
        acc += w;
    }
    out
}

fn point_in_poly(pt: Pos2, poly: &[Pos2]) -> bool {
    let n = poly.len();
    if n < 3 {
        return false;
    }
    let mut inside = false;
    let mut j = n - 1;
    for i in 0..n {
        let pi = poly[i];
        let pj = poly[j];
        if ((pi.y > pt.y) != (pj.y > pt.y))
            && (pt.x < (pj.x - pi.x) * (pt.y - pi.y) / ((pj.y - pi.y) + 1e-6) + pi.x)
        {
            inside = !inside;
        }
        j = i;
    }
    inside
}

fn are_connected(rests: &BTreeMap<i32, ToothRestorationUnit>, t1: i32, t2: i32) -> bool {
    let c1 = rests.get(&t1).map(|u| u.connected_to.contains(&t2)).unwrap_or(false);
    let c2 = rests.get(&t2).map(|u| u.connected_to.contains(&t1)).unwrap_or(false);
    c1 || c2
}

/// Dibuja el odontograma completo en Óvalo Vertical y devuelve eventos de selección o conector.
pub fn show_fdi_odontogram(
    ui: &mut egui::Ui,
    state: &mut FdiOdontogramState,
    restorations: &BTreeMap<i32, ToothRestorationUnit>,
) -> Vec<OdontogramEvent> {
    let mut events = Vec::new();
    let avail = ui.available_size();
    let desired = Vec2::new(avail.x.max(300.0), avail.y.max(460.0));
    let (rect, response) = ui.allocate_exact_size(desired, Sense::click());
    let painter = ui.painter_at(rect);

    // Marco y fondo profundo ciruela
    painter.rect(
        rect,
        CornerRadius::same(9),
        Color32::from_rgb(0x14, 0x03, 0x1B),
        Stroke::new(1.3_f32, Color32::from_rgb(0x4E, 0x18, 0x64)),
        egui::StrokeKind::Inside,
    );

    // Indicador superior izquierdo
    let hint_rect = Rect::from_min_size(rect.min + Vec2::new(8.0, 7.0), Vec2::new(205.0, 22.0));
    painter.rect(
        hint_rect,
        CornerRadius::same(5),
        Color32::from_rgb(0x22, 0x05, 0x2E),
        Stroke::new(1.0_f32, Color32::from_rgb(0x5A, 0x1D, 0x75)),
        egui::StrokeKind::Inside,
    );
    painter.text(
        hint_rect.center(),
        Align2::CENTER_CENTER,
        tr("Clic: Seleccionar / Deseleccionar", "Click: Select / Deselect", "Clique: Selecionar / Desmarcar"),
        FontId::proportional(10.5),
        CREAM,
    );

    // Botón superior derecho: Limpiar selección
    let clear_rect = if !state.selected_teeth.is_empty() {
        let r = Rect::from_min_size(
            Pos2::new(rect.right() - 114.0, rect.top() + 7.0),
            Vec2::new(106.0, 22.0),
        );
        painter.rect(
            r,
            CornerRadius::same(5),
            Color32::from_rgb(0x32, 0x09, 0x42),
            Stroke::new(1.2_f32, AMETHYST),
            egui::StrokeKind::Inside,
        );
        painter.text(
            r.center(),
            Align2::CENTER_CENTER,
            format!("✕ {} ({})", tr("Limpiar", "Clear", "Limpar"), state.selected_teeth.len()),
            FontId::proportional(10.5),
            CREAM,
        );
        Some(r)
    } else {
        None
    };

    // ==================================================================
    // GEOMETRÍA DEL ÓVALO VERTICAL (ry > rx)
    // ==================================================================
    let w = rect.width();
    let _h = rect.height();
    let cx = rect.left() + w * 0.50;
    let usable_top = rect.top() + 34.0;
    let usable_bottom = rect.bottom() - 38.0;
    let mid_y = 0.5 * (usable_top + usable_bottom);
    let half_h = ((usable_bottom - usable_top) * 0.50).max(120.0);

    let arch_gap = (half_h * 0.065).max(10.0);
    let ry = (half_h - arch_gap - 24.0).max(85.0);
    let rx = (w * 0.33).min(ry * 0.68).max(62.0);
    let scale = (rx / 128.0).min(ry / 172.0).clamp(0.68, 1.08);

    // Líneas guía anatómicas centrales
    painter.line_segment(
        [Pos2::new(cx, usable_top + 12.0), Pos2::new(cx, usable_bottom - 12.0)],
        Stroke::new(1.0_f32, Color32::from_rgb(0x35, 0x0D, 0x47)),
    );
    painter.line_segment(
        [Pos2::new(cx - rx * 0.68, mid_y), Pos2::new(cx + rx * 0.68, mid_y)],
        Stroke::new(1.0_f32, Color32::from_rgb(0x35, 0x0D, 0x47)),
    );

    // Etiquetas de cuadrantes
    let q_col = Color32::from_rgb(0x6E, 0x45, 0x82);
    let q_font = FontId::proportional(9.5);
    painter.text(Pos2::new(cx - 8.0, mid_y - 16.0), Align2::RIGHT_CENTER, tr("SUP. DER.", "UPPER R.", "SUP. DIR."), q_font.clone(), q_col);
    painter.text(Pos2::new(cx + 8.0, mid_y - 16.0), Align2::LEFT_CENTER, tr("SUP. IZQ.", "UPPER L.", "SUP. ESQ."), q_font.clone(), q_col);
    painter.text(Pos2::new(cx - 8.0, mid_y + 16.0), Align2::RIGHT_CENTER, tr("INF. DER.", "LOWER R.", "INF. DIR."), q_font.clone(), q_col);
    painter.text(Pos2::new(cx + 8.0, mid_y + 16.0), Align2::LEFT_CENTER, tr("INF. IZQ.", "LOWER L.", "INF. ESQ."), q_font, q_col);

    let upper_angles = compute_arch_angles(183.0, 357.0);
    let lower_angles = compute_arch_angles(177.0, 3.0);

    let mut tooth_centers: BTreeMap<i32, Pos2> = BTreeMap::new();
    let mut tooth_normals: BTreeMap<i32, Vec2> = BTreeMap::new();
    let mut tooth_draw: Vec<(i32, Pos2, f32, Pos2, Vec<Pos2>, Vec<(Pos2, Pos2)>)> = Vec::with_capacity(32);

    for (arch_idx, (arch, angles)) in [(&UPPER_ARCH, upper_angles), (&LOWER_ARCH, lower_angles)]
        .into_iter()
        .enumerate()
    {
        let cy_arch = if arch_idx == 0 { mid_y - arch_gap } else { mid_y + arch_gap };
        for (&fdi, theta) in arch.iter().zip(angles.iter()) {
            let tx = cx + rx * theta.cos();
            let ty = cy_arch + ry * theta.sin();
            let center = Pos2::new(tx, ty);
            tooth_centers.insert(fdi, center);

            let mut nx = theta.cos() / rx.max(1.0);
            let mut ny = theta.sin() / ry.max(1.0);
            let nlen = nx.hypot(ny).max(1e-6);
            nx /= nlen;
            ny /= nlen;
            tooth_normals.insert(fdi, Vec2::new(nx, ny));

            let rot_rad = nx.atan2(-ny);
            let (cos_r, sin_r) = (rot_rad.cos(), rot_rad.sin());
            let xform = |p: Vec2| -> Pos2 {
                Pos2::new(
                    center.x + p.x * cos_r - p.y * sin_r,
                    center.y + p.x * sin_r + p.y * cos_r,
                )
            };

            let (outer_loc, fiss_loc) = build_anatomical_tooth_geometry(fdi, scale);
            let outer_world: Vec<Pos2> = outer_loc.into_iter().map(&xform).collect();
            let fiss_world: Vec<(Pos2, Pos2)> = fiss_loc
                .into_iter()
                .map(|(a, b)| (xform(a), xform(b)))
                .collect();

            let lbl_pt = Pos2::new(tx + nx * 19.5 * scale, ty + ny * 19.5 * scale);
            tooth_draw.push((fdi, center, rot_rad, lbl_pt, outer_world, fiss_world));
        }
    }

    // Calcular posiciones de los círculos conectores interproximales estilo Exocad
    let mut connectors: Vec<((i32, i32), Pos2, f32)> = Vec::new();
    for arch in [&UPPER_ARCH, &LOWER_ARCH] {
        for i in 0..(arch.len() - 1) {
            let (t1, t2) = (arch[i], arch[i + 1]);
            let (Some(&p1), Some(&p2), Some(&n1), Some(&n2)) = (
                tooth_centers.get(&t1),
                tooth_centers.get(&t2),
                tooth_normals.get(&t1),
                tooth_normals.get(&t2),
            ) else {
                continue;
            };
            let avg_n = (n1 + n2).normalized();
            let mid = Pos2::new(0.5 * (p1.x + p2.x), 0.5 * (p1.y + p2.y));
            let c_pt = mid - avg_n * (12.2 * scale);
            let both_active = (restorations.contains_key(&t1) && restorations.contains_key(&t2))
                || (state.selected_teeth.contains(&t1) && state.selected_teeth.contains(&t2));
            let linked = are_connected(restorations, t1, t2);
            let rad = (if both_active || linked { 6.8 } else { 4.4 }) * scale;
            connectors.push(((t1, t2), c_pt, rad));
        }
    }

    // Detección de hover y clic
    let hover_pos = response.hover_pos();
    let mut hovered_connector: Option<(i32, i32)> = None;
    let mut hovered_tooth: Option<i32> = None;

    if let Some(hp) = hover_pos {
        // Prioridad 1: círculo conector
        let mut best_d = 999.0_f32;
        for &(pair, cpt, rad) in &connectors {
            let d = hp.distance(cpt);
            if d <= (rad + 3.5).max(8.0) && d < best_d {
                best_d = d;
                hovered_connector = Some(pair);
            }
        }
        // Prioridad 2: diente anatómico
        if hovered_connector.is_none() {
            for &(fdi, center, _, _, ref poly, _) in &tooth_draw {
                if point_in_poly(hp, poly) || hp.distance(center) < 13.0 * scale {
                    hovered_tooth = Some(fdi);
                    break;
                }
            }
        }
    }

    if response.clicked() {
        if let Some(cp) = response.interact_pointer_pos() {
            if let Some(cr) = clear_rect {
                if cr.contains(cp) {
                    state.clear();
                    events.push(OdontogramEvent::SelectionChanged(None));
                }
            }
            if events.is_empty() {
                if let Some((t1, t2)) = hovered_connector {
                    let linked = are_connected(restorations, t1, t2);
                    events.push(OdontogramEvent::ConnectorToggled(t1, t2, !linked));
                } else if let Some(fdi) = hovered_tooth {
                    let shift = ui.input(|i| i.modifiers.shift);
                    state.toggle_tooth_click(fdi, shift);
                    events.push(OdontogramEvent::SelectionChanged(state.selected_tooth));
                }
            }
        }
    }

    // 1. Dibujar barras de puente debajo de los dientes
    for &((t1, t2), c_pt, _) in &connectors {
        let (Some(&p1), Some(&p2)) = (tooth_centers.get(&t1), tooth_centers.get(&t2)) else {
            continue;
        };
        let mid = Pos2::new(0.5 * (p1.x + p2.x), 0.5 * (p1.y + p2.y));
        let linked = are_connected(restorations, t1, t2);
        let both_active = (restorations.contains_key(&t1) && restorations.contains_key(&t2))
            || (state.selected_teeth.contains(&t1) && state.selected_teeth.contains(&t2));
        if linked {
            painter.line_segment([p1, p2], Stroke::new(5.5 * scale, Color32::from_rgb(0x0F, 0x76, 0x6E)));
            painter.line_segment([p1, p2], Stroke::new(2.6 * scale, Color32::from_rgb(0x2D, 0xD4, 0xBF)));
            painter.line_segment([mid, c_pt], Stroke::new(2.2 * scale, Color32::from_rgb(0x2D, 0xD4, 0xBF)));
        } else if both_active {
            painter.line_segment([mid, c_pt], Stroke::new(1.1_f32, Color32::from_rgb(0x6B, 0x21, 0xA8)));
        }
    }

    // 2. Dibujar cada diente anatómico y su etiqueta FDI
    for (fdi, _center, _rot, lbl_pt, poly, fissures) in tooth_draw {
        let is_active = state.selected_tooth == Some(fdi);
        let is_sel = state.selected_teeth.contains(&fdi);
        let is_hov = hovered_tooth == Some(fdi);
        let unit = restorations.get(&fdi);

        let (fill, border, border_w, fiss_col) = if let Some(u) = unit {
            let base = theme::process_color(&u.process_type);
            if is_active || is_sel {
                (base.gamma_multiply(1.22), Color32::WHITE, 2.2, Color32::from_rgb(0x18, 0x03, 0x20))
            } else {
                (base, CREAM, 1.4_f32, Color32::from_rgb(0x1D, 0x04, 0x27))
            }
        } else if is_active {
            (CREAM, Color32::WHITE, 2.2_f32, Color32::from_rgb(0x3D, 0x0F, 0x52))
        } else if is_sel {
            (Color32::from_rgb(0xD8, 0xB4, 0xE2), CREAM, 1.9_f32, Color32::from_rgb(0x2D, 0x08, 0x3C))
        } else if is_hov {
            (Color32::from_rgb(0x7B, 0x2C, 0xBF), CREAM, 1.5_f32, CREAM)
        } else {
            let is_molar = (fdi % 10) >= 6;
            (
                if is_molar { Color32::from_rgb(0x2F, 0x0A, 0x3D) } else { Color32::from_rgb(0x25, 0x07, 0x31) },
                Color32::from_rgb(0x69, 0x23, 0x87),
                1.1_f32,
                Color32::from_rgb(0x8D, 0x49, 0xAD),
            )
        };

        painter.add(Shape::convex_polygon(poly.clone(), fill, Stroke::NONE));
        painter.add(Shape::closed_line(poly, Stroke::new(border_w, border)));
        for (fa, fb) in fissures {
            painter.line_segment([fa, fb], Stroke::new(1.0_f32, fiss_col));
        }

        let txt_col = if is_active || is_sel {
            Color32::WHITE
        } else if let Some(u) = unit {
            theme::process_color(&u.process_type)
        } else {
            Color32::from_rgb(0xC8, 0xB6, 0xD8)
        };
        painter.text(
            lbl_pt,
            Align2::CENTER_CENTER,
            fdi.to_string(),
            FontId::proportional(10.0),
            txt_col,
        );
    }

    // 3. Dibujar los círculos conectores estilo Exocad encima
    for ((t1, t2), c_pt, rad) in connectors {
        let linked = are_connected(restorations, t1, t2);
        let is_hov = hovered_connector == Some((t1, t2));
        let both_rest = restorations.contains_key(&t1) && restorations.contains_key(&t2);
        let both_sel = state.selected_teeth.contains(&t1) && state.selected_teeth.contains(&t2);

        if linked {
            painter.circle(
                c_pt,
                rad,
                if is_hov { CYAN } else { TEAL },
                Stroke::new(if is_hov { 1.8_f32 } else { 1.4_f32 }, Color32::WHITE),
            );
            painter.circle_filled(c_pt, rad * 0.36, Color32::WHITE);
        } else if both_rest || both_sel || is_hov {
            painter.circle(
                c_pt,
                rad,
                if is_hov { Color32::from_rgb(0x3B, 0x0A, 0x52) } else { Color32::from_rgb(0x18, 0x03, 0x20) },
                Stroke::new(if is_hov { 1.8_f32 } else { 1.4_f32 }, if is_hov { CYAN } else { CREAM }),
            );
        } else {
            painter.circle(
                c_pt,
                rad,
                Color32::from_rgb(0x1E, 0x04, 0x28),
                Stroke::new(1.0_f32, Color32::from_rgb(0x4C, 0x16, 0x63)),
            );
        }
    }

    // 4. Leyenda central dentro del óvalo
    let badge_w = (rx * 1.38).min(182.0);
    let badge_rect = Rect::from_center_size(Pos2::new(cx, mid_y - 38.0), Vec2::new(badge_w, 34.0));
    painter.rect(
        badge_rect,
        CornerRadius::same(6),
        Color32::from_rgba_unmultiplied(25, 4, 34, 220),
        Stroke::new(1.0_f32, Color32::from_rgb(0x5B, 0x1E, 0x78)),
        egui::StrokeKind::Inside,
    );
    painter.text(
        badge_rect.center(),
        Align2::CENTER_CENTER,
        tr(
            "● Círculo Marcado = Puente\n○ Sin Marcar = Individuales",
            "● Filled Circle = Bridge\n○ Empty Circle = Single Units",
            "● Círculo Marcado = Ponte\n○ Vazio = Unitários",
        ),
        FontId::proportional(9.5),
        CREAM_MUTED,
    );

    // 5. Leyenda inferior de estado
    let unit_w = tr("Pieza", "Unit", "Dente");
    let status_txt = if let Some((t1, t2)) = hovered_connector {
        if are_connected(restorations, t1, t2) {
            format!("{} #{} ↔ #{}: {}", tr("Conector", "Connector", "Conector"), t1, t2, tr("UNIDAS EN PUENTE (Clic para separar)", "LINKED IN BRIDGE (Click to split)", "UNIDOS EM PONTE (Clique para separar)"))
        } else {
            format!("{} #{} ↔ #{}: {}", tr("Conector", "Connector", "Conector"), t1, t2, tr("INDIVIDUALES (Clic para unir en puente)", "SINGLE UNITS (Click to connect)", "UNITÁRIOS (Clique para conectar)"))
        }
    } else if state.selected_teeth.is_empty() {
        if let Some(h) = hovered_tooth {
            format!("{} #{} ({}) — {}", unit_w, h, i18n::tr_fdi_tooth_name(h), tr("Clic para seleccionar", "Click to select", "Clique para selecionar"))
        } else {
            tr("Ningún diente seleccionado — Haz clic en un diente para seleccionarlo", "No tooth selected — Click a tooth to select", "Nenhum dente selecionado — Clique em um dente").to_string()
        }
    } else if state.selected_teeth.len() > 1 {
        let sel_list = state
            .selected_teeth
            .iter()
            .map(|t| format!("#{}", t))
            .collect::<Vec<_>>()
            .join(", ");
        format!(
            "{} ({}): {} — {}",
            tr("Seleccionadas", "Selected", "Selecionados"),
            state.selected_teeth.len(),
            sel_list,
            tr("Clic en cualquier diente para deseleccionarlo", "Click any tooth to toggle", "Clique em um dente para desmarcar")
        )
    } else if let Some(fdi) = state.selected_tooth {
        if let Some(u) = restorations.get(&fdi) {
            let mat_t = i18n::tr_material_name(&u.material);
            let mat_s = mat_t.split('(').next().unwrap_or(&mat_t).trim();
            format!(
                "{} #{} ({}) • {} • {} — {}",
                unit_w,
                fdi,
                i18n::tr_fdi_tooth_name(fdi),
                i18n::tr_process_type(&u.process_type),
                mat_s,
                tr("Clic para deseleccionar", "Click to deselect", "Clique para desmarcar")
            )
        } else {
            format!("{} #{} — {} ({})", unit_w, fdi, i18n::tr_fdi_tooth_name(fdi), tr("Clic de nuevo para deseleccionar", "Click again to deselect", "Clique novamente para desmarcar"))
        }
    } else {
        String::new()
    };

    painter.text(
        Pos2::new(rect.center().x, rect.bottom() - 18.0),
        Align2::CENTER_CENTER,
        status_txt,
        FontId::proportional(10.5),
        AMETHYST_BRIGHT,
    );

    events
}
