//! Cámara Trackball 360° sin restricciones (`OrbitCamera`) basada en Cuaterniones (`glam::Quat`).
//!
//! Permite rotar, voltear completamente boca abajo (sin bloqueo de polos ni gimbal lock),
//! girar en plano de pantalla (`roll`) y panear 1:1 los modelos 3D en cualquier orientación.

use egui::{Pos2, Rect};
use glam::{Mat3, Mat4, Quat, Vec3, Vec4};

#[derive(Debug, Clone)]
pub struct OrbitCamera {
    pub target: Vec3,
    /// Orientación cámara -> mundo (sin bloqueo de pitch; rotación libre 360° en todos los ejes).
    pub orientation: Quat,
    pub distance: f32, // distancia al target en mm
    pub fov_deg: f32,
    pub rotate_sensitivity: f32,
    pub zoom_sensitivity: f32,
    pub invert_zoom: bool,
}

impl Default for OrbitCamera {
    fn default() -> Self {
        let mut cam = Self {
            target: Vec3::ZERO,
            orientation: Quat::IDENTITY,
            distance: 48.0,
            fov_deg: 38.0,
            rotate_sensitivity: 1.0,
            zoom_sensitivity: 1.0,
            invert_zoom: false,
        };
        cam.set_view_isometric();
        cam
    }
}

impl OrbitCamera {
    fn quat_from_back_and_up(back_dir: Vec3, hint_up: Vec3) -> Quat {
        let back = back_dir.normalize_or_zero();
        let back = if back.length_squared() < 1e-6 { Vec3::Z } else { back };
        let mut up_ref = hint_up.normalize_or_zero();
        if up_ref.cross(back).length_squared() < 1e-4 {
            up_ref = if back.y.abs() < 0.9 { Vec3::Y } else { Vec3::X };
        }
        let right = up_ref.cross(back).normalize_or_zero();
        let up = back.cross(right).normalize_or_zero();
        Quat::from_mat3(&Mat3::from_cols(right, up, back)).normalize()
    }

    pub fn right_and_up(&self) -> (Vec3, Vec3) {
        let q = self.orientation.normalize();
        (q * Vec3::X, q * Vec3::Y)
    }

    pub fn back_dir(&self) -> Vec3 {
        self.orientation.normalize() * Vec3::Z
    }

    pub fn eye_position(&self) -> Vec3 {
        self.target + self.back_dir() * self.distance
    }

    pub fn forward(&self) -> Vec3 {
        -self.back_dir()
    }

    pub fn view_matrix(&self) -> Mat4 {
        let eye = self.eye_position();
        let (_, up) = self.right_and_up();
        Mat4::look_at_rh(eye, self.target, up)
    }

    pub fn proj_matrix(&self, aspect: f32) -> Mat4 {
        let near = (self.distance * 0.008).clamp(0.08, 1.5);
        let far = (self.distance * 30.0).max(500.0);
        Mat4::perspective_rh_gl(self.fov_deg.to_radians(), aspect.max(0.1), near, far)
    }

    pub fn view_proj(&self, rect: Rect) -> Mat4 {
        let aspect = rect.width() / rect.height().max(1.0);
        self.proj_matrix(aspect) * self.view_matrix()
    }

    /// Rotación libre 360° tipo Trackball en espacio de cámara (cero gimbal lock, permite
    /// voltear por completo los modelos boca abajo y posicionarlos en cualquier ángulo).
    pub fn orbit(&mut self, delta_px: egui::Vec2) {
        if delta_px.length_sq() < 1e-8 {
            return;
        }
        let s = self.rotate_sensitivity.clamp(0.4, 2.5);
        let (right, up) = self.right_and_up();
        let q_h = Quat::from_axis_angle(up, -delta_px.x * 0.0075 * s);
        let q_v = Quat::from_axis_angle(right, -delta_px.y * 0.0075 * s);
        self.orientation = (q_h * q_v * self.orientation).normalize();
    }

    /// Giro en el plano de la pantalla (Roll 360°) para orientar la arcada libremente.
    pub fn roll(&mut self, delta_px_x: f32) {
        if delta_px_x.abs() < 1e-6 {
            return;
        }
        let s = self.rotate_sensitivity.clamp(0.4, 2.5);
        let fwd = self.forward();
        let q_r = Quat::from_axis_angle(fwd, delta_px_x * 0.0075 * s);
        self.orientation = (q_r * self.orientation).normalize();
    }

    /// Paneo exacto 1:1 en pantalla respetando la orientación actual (incluso boca abajo).
    pub fn pan(&mut self, delta_px: egui::Vec2, rect: Rect) {
        let (right, up) = self.right_and_up();
        let world_h = 2.0 * self.distance * (0.5 * self.fov_deg.to_radians()).tan();
        let mm_per_px = world_h / rect.height().max(1.0);
        self.target -= right * (delta_px.x * mm_per_px);
        self.target += up * (delta_px.y * mm_per_px);
    }

    pub fn zoom(&mut self, scroll_delta: f32) {
        let s = self.zoom_sensitivity.clamp(0.4, 2.5);
        let dir = if self.invert_zoom { -1.0 } else { 1.0 };
        let factor = (1.0 - scroll_delta * dir * 0.0018 * s).clamp(0.68, 1.38);
        self.distance = (self.distance * factor).clamp(2.0, 320.0);
    }

    pub fn frame_bounds(&mut self, min_b: Vec3, max_b: Vec3) {
        self.target = 0.5 * (min_b + max_b);
        let rad = 0.5 * (max_b - min_b).length().max(4.0);
        self.distance = (rad / (0.5 * self.fov_deg.to_radians()).tan() * 1.35).clamp(8.0, 220.0);
    }

    pub fn set_view_occlusal_top(&mut self) {
        self.orientation = Self::quat_from_back_and_up(Vec3::Z, Vec3::Y);
    }

    pub fn set_view_occlusal_bottom(&mut self) {
        self.orientation = Self::quat_from_back_and_up(Vec3::NEG_Z, Vec3::Y);
    }

    pub fn set_view_frontal(&mut self) {
        self.orientation = Self::quat_from_back_and_up(Vec3::new(0.0, -1.0, 0.08), Vec3::Z);
    }

    pub fn set_view_lingual(&mut self) {
        self.orientation = Self::quat_from_back_and_up(Vec3::new(0.0, 1.0, 0.08), Vec3::Z);
    }

    pub fn set_view_lateral(&mut self) {
        self.orientation = Self::quat_from_back_and_up(Vec3::new(1.0, 0.0, 0.08), Vec3::Z);
    }

    pub fn set_view_isometric(&mut self) {
        self.orientation = Self::quat_from_back_and_up(Vec3::new(0.62, -0.66, 0.44), Vec3::Z);
    }

    /// Genera el rayo 3D `(origin, dir_normalized)` en coordenadas del mundo desde un pixel en `rect`.
    pub fn screen_ray(&self, pos: Pos2, rect: Rect) -> (Vec3, Vec3) {
        let nx = ((pos.x - rect.left()) / rect.width().max(1.0)) * 2.0 - 1.0;
        let ny = 1.0 - ((pos.y - rect.top()) / rect.height().max(1.0)) * 2.0;
        let inv_vp = self.view_proj(rect).inverse();

        let near_ndc = Vec4::new(nx, ny, -1.0, 1.0);
        let far_ndc = Vec4::new(nx, ny, 1.0, 1.0);

        let mut near_w = inv_vp * near_ndc;
        let mut far_w = inv_vp * far_ndc;
        if near_w.w.abs() > 1e-7 {
            near_w /= near_w.w;
        }
        if far_w.w.abs() > 1e-7 {
            far_w /= far_w.w;
        }
        let origin = near_w.truncate();
        let dir = (far_w.truncate() - origin).normalize_or_zero();
        (origin, dir)
    }

    /// Proyecta un punto 3D del mundo a coordenadas de pantalla `egui::Pos2` dentro de `rect`.
    pub fn world_to_screen(&self, world_pt: Vec3, rect: Rect) -> Option<Pos2> {
        let clip = self.view_proj(rect) * Vec4::new(world_pt.x, world_pt.y, world_pt.z, 1.0);
        if clip.w <= 1e-5 {
            return None;
        }
        let ndc = clip.truncate() / clip.w;
        if ndc.z < -1.0 || ndc.z > 1.0 {
            return None;
        }
        let sx = rect.left() + (ndc.x * 0.5 + 0.5) * rect.width();
        let sy = rect.top() + (0.5 - ndc.y * 0.5) * rect.height();
        Some(Pos2::new(sx, sy))
    }

    /// Convierte un movimiento 2D del cursor `(start -> curr)` en el vector 3D exacto paralelo
    /// al plano de vista de la cámara a la profundidad de `ref_world_pt` (arrastre 1:1).
    pub fn screen_delta_to_world(
        &self,
        ref_world_pt: Vec3,
        start_pos: Pos2,
        curr_pos: Pos2,
        rect: Rect,
    ) -> Vec3 {
        let plane_n = self.forward();
        let intersect_plane = |p: Pos2| -> Option<Vec3> {
            let (ro, rd) = self.screen_ray(p, rect);
            let denom = rd.dot(plane_n);
            if denom.abs() < 1e-6 {
                return None;
            }
            let t = (ref_world_pt - ro).dot(plane_n) / denom;
            Some(ro + rd * t)
        };
        if let (Some(w0), Some(w1)) = (intersect_plane(start_pos), intersect_plane(curr_pos)) {
            w1 - w0
        } else {
            Vec3::ZERO
        }
    }

    /// Devuelve `(screen_unit_dir_2d, mm_per_pixel)` para proyectar arrastres axiales/rotacionales.
    pub fn screen_axis_dir_and_scale(
        &self,
        center_3d: Vec3,
        axis_3d: Vec3,
        rect: Rect,
    ) -> (egui::Vec2, f32) {
        let p0 = self.world_to_screen(center_3d, rect);
        let p1 = self.world_to_screen(center_3d + axis_3d, rect);
        if let (Some(a), Some(b)) = (p0, p1) {
            let d = b - a;
            let px_per_mm = d.length();
            if px_per_mm > 1e-2 {
                return (d / px_per_mm, (1.0 / px_per_mm).clamp(0.005, 0.14));
            }
        }
        (egui::vec2(1.0, 0.0), 0.032)
    }

    /// Controlador uniforme de navegación 3D para todos los visores:
    /// - Clic Derecho (`Secondary`) + arrastrar: Rotación orbital 360° (Alt/Shift + Der = Roll).
    /// - Rueda presionada (`Middle`) + arrastrar: Paneo 1:1 (Shift + Medio = Roll).
    /// - Rueda girar (scroll): Zoom (si `allow_zoom` es true).
    /// - Clic Izquierdo (`Primary`): NUNCA rota ni mueve la cámara, reservado 100% para acciones.
    pub fn handle_navigation_input(
        &mut self,
        resp: &egui::Response,
        ui: &egui::Ui,
        rect: Rect,
        allow_zoom: bool,
    ) {
        let primary_down = ui.input(|i| i.pointer.primary_down());
        let alt_down = ui.input(|i| i.modifiers.alt);
        let shift_down = ui.input(|i| i.modifiers.shift);

        // Clic derecho + arrastrar = órbita 360° (suprimido si Primary también está abajo,
        // p. ej. acorde Izq+Der estilo Exocad para borrar nodo de margen)
        if resp.dragged_by(egui::PointerButton::Secondary) && !primary_down {
            if alt_down || shift_down {
                self.roll(resp.drag_delta().x);
            } else {
                self.orbit(resp.drag_delta());
            }
        } else if resp.dragged_by(egui::PointerButton::Middle) {
            if shift_down {
                self.roll(resp.drag_delta().x);
            } else {
                self.pan(resp.drag_delta(), rect);
            }
        }

        if allow_zoom && resp.hovered() {
            let scroll = ui.input(|i| i.smooth_scroll_delta.y);
            if scroll.abs() > 0.01 {
                self.zoom(scroll);
            }
        }
    }
}
