//! Sistema de Colocación de Dientes por **Flechas Direccionales Fijas en Plano de Vista**
//! (estilo 3Shape Smile Composer — *Individual Transformation*).
//!
//! A diferencia de las esferas o gizmos 3D acoplados al diente:
//! - Cada control es una **flecha vectorial nítida** que indica exactamente su acción:
//!   - **4 Flechas Verdes Direccionales (`◀ ▶ ▲ ▼`)**: Estirar/encoger la cara en el plano
//!     exacto que estás viendo (`Ctrl` = escala proporcional global).
//!   - **4 Flechas Curvas Azules (`↻`)** en las esquinas: Rotar el diente 1:1 en el plano
//!     de vista alrededor de su centro anatómico.
//!   - **Flecha Cruzada Central (`✥`) + Arrastre directo sobre el diente**: Mover libre 1:1
//!     en el plano de vista (`Shift` = giro libre 3D, `Ctrl` = escala global).
//! - **Fijas respecto al diente**: Las flechas NO rotan con el diente, permanecen siempre
//!   fijas en pantalla durante el arrastre y siempre accesibles en el plano actual de cámara.

use crate::geometry::mesh::TriMeshData;
use crate::render::camera::OrbitCamera;
use egui::{Align2, Color32, FontId, Painter, Pos2, Rect, Shape, Stroke, Vec2 as EguiVec2};
use std::f32::consts::PI;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TransformSubmode {
    All,
    MoveRotate,
    Scale,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ArrowKind {
    MoveCenter,
    ScaleUniformCenter,
    ScaleRight,
    ScaleLeft,
    ScaleUp,
    ScaleDown,
    RotateCorner,
}

#[derive(Debug, Clone)]
pub struct ArrowControl {
    pub id: &'static str,
    pub label: &'static str,
    pub kind: ArrowKind,
    pub pos: Pos2,
    /// Dirección 2D unitaria en pantalla (hacia donde apunta la flecha).
    pub dir_2d: EguiVec2,
    /// Ángulo base en pantalla (radianes) para las flechas curvas de rotación en esquinas.
    pub corner_angle: f32,
    pub hit_radius: f32,
}

#[derive(Debug, Clone)]
pub struct PlacementOverlay {
    pub tooth_rect: Rect,
    pub center: Pos2,
    pub controls: Vec<ArrowControl>,
}

impl PlacementOverlay {
    /// Proyecta la malla del diente al plano de pantalla de la cámara actual para obtener
    /// su silueta 2D (`tooth_rect`). Las flechas se colocan siempre rectas en el plano de vista.
    pub fn compute_tooth_screen_rect(
        crown: &TriMeshData,
        camera: &OrbitCamera,
        viewport_rect: Rect,
    ) -> Option<Rect> {
        if crown.is_empty() {
            return None;
        }
        let mut min_p = Pos2::new(f32::INFINITY, f32::INFINITY);
        let mut max_p = Pos2::new(f32::NEG_INFINITY, f32::NEG_INFINITY);
        let step = (crown.positions.len() / 600).max(1);
        let mut any = false;
        for &p in crown.positions.iter().step_by(step) {
            if let Some(sp) = camera.world_to_screen(p, viewport_rect) {
                min_p.x = min_p.x.min(sp.x);
                min_p.y = min_p.y.min(sp.y);
                max_p.x = max_p.x.max(sp.x);
                max_p.y = max_p.y.max(sp.y);
                any = true;
            }
        }
        if !any {
            return None;
        }
        // Asegurar un tamaño mínimo cómodo para que las flechas jamás se encimen
        let mut rect = Rect::from_min_max(min_p, max_p);
        if rect.width() < 68.0 {
            let cx = rect.center().x;
            rect.min.x = cx - 34.0;
            rect.max.x = cx + 34.0;
        }
        if rect.height() < 68.0 {
            let cy = rect.center().y;
            rect.min.y = cy - 34.0;
            rect.max.y = cy + 34.0;
        }
        Some(rect)
    }

    /// Construye el juego de flechas direccionales fijas alrededor de `tooth_rect`.
    pub fn build(tooth_rect: Rect, viewport_rect: Rect, submode: TransformSubmode) -> Self {
        let c = tooth_rect.center();
        let pad_edge = 24.0_f32;
        let pad_corner = 20.0_f32;
        let clamp_pt = |p: Pos2| -> Pos2 {
            Pos2::new(
                p.x.clamp(viewport_rect.left() + 26.0, viewport_rect.right() - 26.0),
                p.y.clamp(viewport_rect.top() + 26.0, viewport_rect.bottom() - 26.0),
            )
        };

        let mut controls = Vec::new();
        let show_rot = matches!(submode, TransformSubmode::All | TransformSubmode::MoveRotate);
        let show_scale = matches!(submode, TransformSubmode::All | TransformSubmode::Scale);

        // 1. Control Central (4 Flechas Cruzadas Mover / o Escala Uniforme en submodo Escalar)
        if submode == TransformSubmode::Scale {
            controls.push(ArrowControl {
                id: "scale_uniform",
                label: "⤢ Flechas Centrales: Escala Proporcional Global (X+Y+Z)",
                kind: ArrowKind::ScaleUniformCenter,
                pos: clamp_pt(c),
                dir_2d: EguiVec2::new(1.0, 0.0),
                corner_angle: 0.0,
                hit_radius: 19.0,
            });
        } else {
            controls.push(ArrowControl {
                id: "move_center",
                label: "✥ Mover Libre en Plano de Vista (Shift = Rotar 3D, Ctrl = Escalar)",
                kind: ArrowKind::MoveCenter,
                pos: clamp_pt(c),
                dir_2d: EguiVec2::new(1.0, 0.0),
                corner_angle: 0.0,
                hit_radius: 19.0,
            });
        }

        // 2. Cuatro Flechas Verdes de Escala Unilateral en el plano exacto de la cámara
        if show_scale {
            controls.push(ArrowControl {
                id: "scale_right",
                label: "▶ Estirar Lado Derecho en Plano Actual (Ctrl = Proporcional)",
                kind: ArrowKind::ScaleRight,
                pos: clamp_pt(Pos2::new(tooth_rect.right() + pad_edge, c.y)),
                dir_2d: EguiVec2::new(1.0, 0.0),
                corner_angle: 0.0,
                hit_radius: 22.0,
            });
            controls.push(ArrowControl {
                id: "scale_left",
                label: "◀ Estirar Lado Izquierdo en Plano Actual (Ctrl = Proporcional)",
                kind: ArrowKind::ScaleLeft,
                pos: clamp_pt(Pos2::new(tooth_rect.left() - pad_edge, c.y)),
                dir_2d: EguiVec2::new(-1.0, 0.0),
                corner_angle: 0.0,
                hit_radius: 22.0,
            });
            controls.push(ArrowControl {
                id: "scale_up",
                label: "▲ Estirar Borde Superior en Plano Actual (Ctrl = Proporcional)",
                kind: ArrowKind::ScaleUp,
                pos: clamp_pt(Pos2::new(c.x, tooth_rect.top() - pad_edge)),
                dir_2d: EguiVec2::new(0.0, -1.0),
                corner_angle: 0.0,
                hit_radius: 22.0,
            });
            controls.push(ArrowControl {
                id: "scale_down",
                label: "▼ Estirar Borde Inferior en Plano Actual (Ctrl = Proporcional)",
                kind: ArrowKind::ScaleDown,
                pos: clamp_pt(Pos2::new(c.x, tooth_rect.bottom() + pad_edge)),
                dir_2d: EguiVec2::new(0.0, 1.0),
                corner_angle: 0.0,
                hit_radius: 22.0,
            });
        }

        // 3. Cuatro Flechas Curvas Azules de Rotación en las esquinas (siempre visibles y rectas)
        if show_rot {
            let corners = [
                (
                    "rot_tr",
                    Pos2::new(tooth_rect.right() + pad_corner, tooth_rect.top() - pad_corner),
                    -PI * 0.25,
                ),
                (
                    "rot_tl",
                    Pos2::new(tooth_rect.left() - pad_corner, tooth_rect.top() - pad_corner),
                    -PI * 0.75,
                ),
                (
                    "rot_br",
                    Pos2::new(tooth_rect.right() + pad_corner, tooth_rect.bottom() + pad_corner),
                    PI * 0.25,
                ),
                (
                    "rot_bl",
                    Pos2::new(tooth_rect.left() - pad_corner, tooth_rect.bottom() + pad_corner),
                    PI * 0.75,
                ),
            ];
            for (id, pt, ang) in corners {
                controls.push(ArrowControl {
                    id,
                    label: "↻ Rotar Diente en Plano de Vista (Arrastra en círculo)",
                    kind: ArrowKind::RotateCorner,
                    pos: clamp_pt(pt),
                    dir_2d: EguiVec2::new(ang.cos(), ang.sin()),
                    corner_angle: ang,
                    hit_radius: 21.0,
                });
            }
        }

        Self {
            tooth_rect,
            center: c,
            controls,
        }
    }

    pub fn hit_test(&self, pos: Pos2) -> Option<&ArrowControl> {
        let mut best: Option<&ArrowControl> = None;
        let mut best_d = f32::INFINITY;
        for ctrl in &self.controls {
            let d = ctrl.pos.distance(pos);
            if d <= ctrl.hit_radius && d < best_d {
                best_d = d;
                best = Some(ctrl);
            }
        }
        best
    }

    /// Dibuja las flechas vectoriales nítidas estilo 3Shape directamente sobre el visor 3D.
    pub fn paint(
        &self,
        painter: &Painter,
        hovered_id: Option<&str>,
        active_id: Option<&str>,
        active_angle_deg: Option<f32>,
    ) {
        let dark_halo = Color32::from_rgba_unmultiplied(0x14, 0x02, 0x1D, 225);
        let gold_hi = Color32::from_rgb(0xFA, 0xCC, 0x15);
        let green = Color32::from_rgb(0x22, 0xC5, 0x5E);
        let blue = Color32::from_rgb(0x38, 0xBD, 0xF8);
        let amethyst = Color32::from_rgb(0xC0, 0x84, 0xFC);
        let cream = Color32::from_rgb(0xF2, 0xEA, 0xD3);

        // Marco guía ultra sutil alrededor de la silueta del diente
        painter.rect_stroke(
            self.tooth_rect.expand(4.0),
            6.0,
            Stroke::new(1.0_f32, Color32::from_rgba_unmultiplied(0xF2, 0xEA, 0xD3, 55)),
            egui::StrokeKind::Outside,
        );

        for ctrl in &self.controls {
            let is_active = active_id == Some(ctrl.id);
            let is_hov = hovered_id == Some(ctrl.id) || is_active;
            let scale_vis = if is_hov { 1.18 } else { 1.0 };

            match ctrl.kind {
                ArrowKind::ScaleRight
                | ArrowKind::ScaleLeft
                | ArrowKind::ScaleUp
                | ArrowKind::ScaleDown => {
                    let col = if is_hov { gold_hi } else { green };
                    draw_directional_arrow(painter, ctrl.pos, ctrl.dir_2d, 15.0 * scale_vis, col, dark_halo);
                }
                ArrowKind::RotateCorner => {
                    let col = if is_hov { gold_hi } else { blue };
                    draw_curved_rotate_arrow(painter, ctrl.pos, ctrl.corner_angle, 13.5 * scale_vis, col, dark_halo);
                }
                ArrowKind::MoveCenter => {
                    let col = if is_hov { gold_hi } else { amethyst };
                    draw_four_way_move_arrows(painter, ctrl.pos, 13.5 * scale_vis, col, cream, dark_halo);
                }
                ArrowKind::ScaleUniformCenter => {
                    let col = if is_hov { gold_hi } else { cream };
                    draw_diagonal_expand_arrows(painter, ctrl.pos, 13.5 * scale_vis, col, dark_halo);
                }
            }
        }

        // Indicador en vivo de ángulo al rotar con una flecha curva azul
        if let Some(deg) = active_angle_deg {
            let badge_pos = Pos2::new(self.center.x, self.tooth_rect.top() - 48.0);
            let rect = Rect::from_center_size(badge_pos, EguiVec2::new(72.0, 22.0));
            painter.rect_filled(rect, 5.0, dark_halo);
            painter.rect_stroke(rect, 5.0, Stroke::new(1.2_f32, blue), egui::StrokeKind::Outside);
            painter.text(
                badge_pos,
                Align2::CENTER_CENTER,
                format!("{:+.1}°", deg),
                FontId::proportional(11.5),
                Color32::WHITE,
            );
        }
    }
}

/// Dibuja una flecha direccional nítida con vástago, cabeza triangular y contorno de contraste.
fn draw_directional_arrow(
    painter: &Painter,
    center: Pos2,
    dir: EguiVec2,
    size: f32,
    fill: Color32,
    halo: Color32,
) {
    let d = dir.normalized();
    let perp = EguiVec2::new(-d.y, d.x);

    // Píldora circular oscura sutil detrás para visibilidad garantizada sobre cerámica blanca
    painter.circle_filled(center, size * 0.92, Color32::from_rgba_unmultiplied(0x16, 0x03, 0x1F, 155));

    let tail = center - d * (size * 0.62);
    let neck = center + d * (size * 0.10);
    let tip = center + d * (size * 0.80);
    let sw = size * 0.24; // medio ancho del vástago
    let hw = size * 0.56; // medio ancho de la punta

    // Polígono completo de flecha (7 vértices)
    let pts = vec![
        tail + perp * sw,
        neck + perp * sw,
        neck + perp * hw,
        tip,
        neck - perp * hw,
        neck - perp * sw,
        tail - perp * sw,
    ];
    // Contorno oscuro grueso + relleno brillante
    painter.add(Shape::closed_line(pts.clone(), Stroke::new(3.2_f32, halo)));
    // Triángulo punta + rectángulo vástago (convexos para renderizado exacto en egui)
    let shaft = vec![
        tail + perp * sw,
        neck + perp * sw,
        neck - perp * sw,
        tail - perp * sw,
    ];
    let head = vec![neck + perp * hw, tip, neck - perp * hw];
    painter.add(Shape::convex_polygon(shaft, fill, Stroke::NONE));
    painter.add(Shape::convex_polygon(head, fill, Stroke::NONE));
    painter.add(Shape::closed_line(pts, Stroke::new(1.2_f32, Color32::WHITE)));
}

/// Dibuja una flecha curva doble de rotación estilo 3Shape en la esquina `corner_angle`.
fn draw_curved_rotate_arrow(
    painter: &Painter,
    center: Pos2,
    corner_angle: f32,
    radius: f32,
    color: Color32,
    halo: Color32,
) {
    painter.circle_filled(center, radius * 1.05, Color32::from_rgba_unmultiplied(0x16, 0x03, 0x1F, 160));

    // Arco de 115° centrado en la dirección `corner_angle`
    let arc_r = radius * 0.72;
    let half_span = 0.98_f32;
    let steps = 14usize;
    let mut arc_pts = Vec::with_capacity(steps + 1);
    for i in 0..=steps {
        let t = i as f32 / steps as f32;
        let a = corner_angle - half_span + t * (2.0 * half_span);
        arc_pts.push(Pos2::new(center.x + a.cos() * arc_r, center.y + a.sin() * arc_r));
    }

    painter.add(Shape::line(arc_pts.clone(), Stroke::new(4.8_f32, halo)));
    painter.add(Shape::line(arc_pts.clone(), Stroke::new(2.6_f32, color)));

    // Dos puntas de flecha en los extremos del arco (indican giro horario y antihorario)
    let draw_tip = |end_pt: Pos2, tan_dir: EguiVec2| {
        let td = tan_dir.normalized();
        let pr = EguiVec2::new(-td.y, td.x);
        let tip = end_pt + td * (radius * 0.34);
        let b1 = end_pt - td * (radius * 0.10) + pr * (radius * 0.28);
        let b2 = end_pt - td * (radius * 0.10) - pr * (radius * 0.28);
        painter.add(Shape::convex_polygon(vec![tip, b1, b2], color, Stroke::new(1.0_f32, halo)));
    };
    if arc_pts.len() >= 2 {
        let p_first = arc_pts[0];
        let p_second = arc_pts[1];
        draw_tip(p_first, p_first - p_second);

        let p_last = arc_pts[arc_pts.len() - 1];
        let p_prev = arc_pts[arc_pts.len() - 2];
        draw_tip(p_last, p_last - p_prev);
    }
}

/// Dibuja el emblema central de 4 flechas direccionales cruzadas (`✥` Mover Libre).
fn draw_four_way_move_arrows(
    painter: &Painter,
    center: Pos2,
    size: f32,
    color: Color32,
    accent: Color32,
    halo: Color32,
) {
    painter.circle_filled(center, size * 1.08, Color32::from_rgba_unmultiplied(0x16, 0x03, 0x1F, 175));
    painter.circle_stroke(center, size * 1.08, Stroke::new(1.2_f32, color));

    for dir in [
        EguiVec2::new(1.0, 0.0),
        EguiVec2::new(-1.0, 0.0),
        EguiVec2::new(0.0, 1.0),
        EguiVec2::new(0.0, -1.0),
    ] {
        let perp = EguiVec2::new(-dir.y, dir.x);
        let tip = center + dir * (size * 0.88);
        let base = center + dir * (size * 0.42);
        painter.line_segment([center, base], Stroke::new(2.2_f32, accent));
        let tri = vec![tip, base + perp * (size * 0.26), base - perp * (size * 0.26)];
        painter.add(Shape::convex_polygon(tri, color, Stroke::new(0.8_f32, halo)));
    }
    painter.circle_filled(center, 2.6, accent);
}

/// Dibuja 4 flechas diagonales salientes (`⤢` Escala Proporcional Global).
fn draw_diagonal_expand_arrows(
    painter: &Painter,
    center: Pos2,
    size: f32,
    color: Color32,
    halo: Color32,
) {
    painter.circle_filled(center, size * 1.08, Color32::from_rgba_unmultiplied(0x16, 0x03, 0x1F, 175));
    painter.circle_stroke(center, size * 1.08, Stroke::new(1.2_f32, color));

    for (dx, dy) in [(1.0_f32, 1.0), (-1.0, 1.0), (1.0, -1.0), (-1.0, -1.0)] {
        let dir = EguiVec2::new(dx, dy).normalized();
        let perp = EguiVec2::new(-dir.y, dir.x);
        let tip = center + dir * (size * 0.86);
        let base = center + dir * (size * 0.42);
        painter.line_segment([center, base], Stroke::new(2.0_f32, color));
        let tri = vec![tip, base + perp * (size * 0.26), base - perp * (size * 0.26)];
        painter.add(Shape::convex_polygon(tri, color, Stroke::new(0.8_f32, halo)));
    }
}
