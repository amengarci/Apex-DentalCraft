//! Módulo 3 (UI + Picking `parry3d`): Trazado y Edición de Línea de Margen por Puro Clic.
//!
//! Interacción sin tecla `Shift`:
//! - Pasar el mouse cerca de cualquier nodo de control lo resalta automáticamente (`#C77DFF`).
//! - Clic Izquierdo sobre un nodo resaltado lo agarra para moverlo libremente sobre el muñón; clic de nuevo lo fija.
//! - Clic Izquierdo + Clic Derecho simultáneos sobre un nodo lo eliminan al instante.
//! - Los márgenes trazados de otras piezas de la orden se visualizan en gris; clic en cualquiera los activa.
//! - Los pónticos de puente NO requieren trazado de margen cervical.
//! - Arrastrar con Clic Derecho rota la cámara 3D; rueda presionada panea y girar hace zoom.

use crate::core::case_manager::{CaseManager, DentalCase};
use crate::core::i18n::{self, tr};
use crate::geometry::crown::{crop_mesh_to_margin_roi, CrownDesigner, MarginAuditResult};
use crate::geometry::margin::{build_tube_mesh, build_uv_sphere, MarginLineController};
use crate::geometry::mesh::{MeshCollider, TriMeshData};
use crate::render::camera::OrbitCamera;
use crate::render::gl_renderer::{
    make_viewport_paint_callback, ColorMapKind, GlViewportRenderer, RenderLayerSpec,
};
use crate::theme::{self, AMETHYST, AMETHYST_BRIGHT, CREAM, CREAM_MUTED, CYAN, EMERALD};
use egui::{Align2, Area, Color32, CornerRadius, FontId, Frame, Margin, Order, Pos2, RichText, Sense, Stroke, Window};
use glam::Vec3;
use std::sync::{Arc, Mutex};

pub enum MarginAction {
    None,
    ProceedToDesign,
    StatusMessage(String),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MarginEditMode {
    Add,
    Insert,
    Move,
    AutoDetect,
}

pub struct MarginPanelState {
    pub camera: OrbitCamera,
    pub prep_mesh: Option<(TriMeshData, u64)>,
    pub ant_mesh: Option<(TriMeshData, u64)>,
    pub margin_ctrl: MarginLineController,
    pub margin_rev: u64,
    pub other_margins_rev: u64,
    pub next_rev: u64,

    pub mode: MarginEditMode,
    pub magnetic_snap: bool,
    pub show_curvature: bool,
    pub line_radius_mm: f32,
    pub node_radius_mm: f32,
    pub live_red_validation: bool,

    pub show_prep: bool,
    pub prep_opacity: f32,
    pub show_ant: bool,
    pub ant_opacity: f32,

    pub selected_node: Option<usize>,
    pub hovered_node: Option<usize>,
    pub grabbed_node: Option<usize>,
    pub hovered_other_margin: Option<i32>,
    pub chord_delete_consumed: bool,
    pub press_start_pos: Option<Pos2>,

    /// Auditoría clínica en vivo ultrarrápida (< 2 ms, cero pausas)
    pub margin_audit: MarginAuditResult,

    /// Caché local recortada del muñón (~10K tris) para auditoría en vivo instantánea a 60 FPS
    /// incluso sobre escaneos intraorales de 360K+ triángulos.
    die_roi_cache: Option<(Vec3, TriMeshData, MeshCollider)>,

    pub undo_stack: Vec<Vec<[f32; 3]>>,
}

impl Default for MarginPanelState {
    fn default() -> Self {
        Self {
            camera: OrbitCamera::default(),
            prep_mesh: None,
            ant_mesh: None,
            margin_ctrl: MarginLineController::default(),
            margin_rev: 1,
            other_margins_rev: 1,
            next_rev: 1,
            mode: MarginEditMode::Add,
            magnetic_snap: true,
            show_curvature: false,
            line_radius_mm: 0.032,
            node_radius_mm: 0.095,
            live_red_validation: true,
            show_prep: true,
            prep_opacity: 1.0,
            show_ant: false,
            ant_opacity: 0.45,
            selected_node: None,
            hovered_node: None,
            grabbed_node: None,
            hovered_other_margin: None,
            chord_delete_consumed: false,
            press_start_pos: None,
            margin_audit: MarginAuditResult::default(),
            die_roi_cache: None,
            undo_stack: Vec::new(),
        }
    }
}

fn unit_arch(fdi: i32) -> &'static str {
    if matches!(fdi / 10, 1 | 2) {
        "maxilla"
    } else {
        "mandible"
    }
}

fn dist_pt_to_segment_2d(p: Pos2, a: Pos2, b: Pos2) -> f32 {
    let ab = b - a;
    let l2 = ab.length_sq();
    if l2 < 1e-6 {
        return p.distance(a);
    }
    let t = ((p - a).dot(ab) / l2).clamp(0.0, 1.0);
    p.distance(a + ab * t)
}

impl MarginPanelState {
    fn bump(&mut self) -> u64 {
        self.next_rev += 1;
        self.next_rev
    }

    pub fn apply_settings(&mut self, settings: &crate::core::settings::AppSettings) {
        let changed = (self.line_radius_mm - settings.margin_line_radius_mm).abs() > 1e-4
            || (self.node_radius_mm - settings.margin_node_radius_mm).abs() > 1e-4
            || self.live_red_validation != settings.margin_live_red_validation;
        self.line_radius_mm = settings.margin_line_radius_mm;
        self.node_radius_mm = settings.margin_node_radius_mm;
        self.live_red_validation = settings.margin_live_red_validation;
        self.camera.rotate_sensitivity = settings.camera_rotate_sensitivity;
        self.camera.zoom_sensitivity = settings.camera_zoom_sensitivity;
        self.camera.invert_zoom = settings.camera_invert_zoom;
        if changed {
            self.margin_rev = self.bump();
            self.other_margins_rev = self.bump();
        }
    }

    /// Garantiza una submalla ROI recortada (~10K tris) alrededor del margen para que la auditoría
    /// corra en < 1.5 ms sin pausas incluso sobre escaneos intraorales de 360K+ triángulos.
    fn ensure_die_roi(&mut self) {
        let Some((prep, _)) = &self.prep_mesh else {
            self.die_roi_cache = None;
            return;
        };
        if self.margin_ctrl.control_points.is_empty() {
            return;
        }
        let center: Vec3 = self.margin_ctrl.control_points.iter().copied().sum::<Vec3>()
            / self.margin_ctrl.control_points.len() as f32;
        let needs_rebuild = match &self.die_roi_cache {
            None => true,
            Some((c, _, _)) => c.distance_squared(center) > 2.5 * 2.5,
        };
        if needs_rebuild {
            let ref_pts = if self.margin_ctrl.curve_points.len() >= 4 {
                &self.margin_ctrl.curve_points
            } else {
                &self.margin_ctrl.control_points
            };
            let roi = crop_mesh_to_margin_roi(prep, ref_pts, 6.0);
            if let Some(col) = MeshCollider::from_mesh(&roi) {
                self.die_roi_cache = Some((center, roi, col));
            }
        }
    }

    /// Re-audita instantáneamente (< 1.5 ms, CERO pausas) los nodos de control de la línea de margen.
    pub fn refresh_audit(&mut self, case: &DentalCase, run_full_cement_check: bool) {
        let is_pontic = case.get_active_unit().map(|u| u.is_pontic()).unwrap_or(false);
        if is_pontic || !self.live_red_validation {
            self.margin_audit = MarginAuditResult::default();
            return;
        }
        if self.prep_mesh.is_none() || self.margin_ctrl.control_points.is_empty() {
            self.margin_audit = MarginAuditResult::default();
            return;
        }
        self.ensure_die_roi();
        let ax = Vec3::from_array(case.insertion_axis);
        if let Some((_, roi_mesh, roi_col)) = &self.die_roi_cache {
            self.margin_audit = CrownDesigner::audit_margin_nodes(
                roi_mesh,
                Some(roi_col),
                &self.margin_ctrl.control_points,
                &self.margin_ctrl.curve_points,
                ax,
                &case.design_params,
                run_full_cement_check,
            );
        } else if let Some((prep, _)) = &self.prep_mesh {
            self.margin_audit = CrownDesigner::audit_margin_nodes(
                prep,
                self.margin_ctrl.collider.as_ref(),
                &self.margin_ctrl.control_points,
                &self.margin_ctrl.curve_points,
                ax,
                &case.design_params,
                run_full_cement_check,
            );
        }
    }

    /// Calcula automáticamente el eje de inserción óptimo del muñón actual (< 15 ms, hasta 38°),
    /// lo guarda en el caso y en la pieza activa, y refresca la auditoría.
    pub fn auto_optimize_insertion_axis(
        &mut self,
        case: &mut DentalCase,
        case_manager: &CaseManager,
    ) -> Option<(f32, usize)> {
        if case.get_active_unit().map(|u| u.is_pontic()).unwrap_or(false) {
            return None;
        }
        if self.margin_ctrl.curve_points.len() < 6 {
            return None;
        }
        self.ensure_die_roi();
        let cur_ax = Vec3::from_array(case.insertion_axis);
        let (opt_ax, rem_ov, tilt_deg) = if let Some((_, roi_mesh, _)) = &self.die_roi_cache {
            CrownDesigner::compute_optimal_insertion_axis(
                roi_mesh,
                &self.margin_ctrl.curve_points,
                cur_ax,
            )
        } else if let Some((prep, _)) = &self.prep_mesh {
            CrownDesigner::compute_optimal_insertion_axis(
                prep,
                &self.margin_ctrl.curve_points,
                cur_ax,
            )
        } else {
            return None;
        };
        case.insertion_axis = opt_ax.to_array();
        if let Some(u) = case.restorations.get_mut(&case.tooth_number) {
            u.insertion_axis = case.insertion_axis;
        }
        self.refresh_audit(case, false);
        self.margin_rev = self.bump();
        let _ = case_manager.save_case(case);
        Some((tilt_deg, rem_ov))
    }

    /// Fija el eje de inserción de la pieza activa directamente desde la dirección actual de la cámara 3D.
    pub fn set_axis_from_camera(&mut self, case: &mut DentalCase, case_manager: &CaseManager) {
        let dir = (self.camera.eye_position() - self.camera.target).normalize_or_zero();
        if dir.length_squared() < 1e-5 {
            return;
        }
        case.insertion_axis = dir.to_array();
        if let Some(u) = case.restorations.get_mut(&case.tooth_number) {
            u.insertion_axis = case.insertion_axis;
        }
        self.refresh_audit(case, false);
        self.margin_rev = self.bump();
        let _ = case_manager.save_case(case);
    }

    pub fn load_case(&mut self, case: &mut DentalCase, case_manager: &CaseManager) {
        self.undo_stack.clear();
        self.selected_node = None;
        self.hovered_node = None;
        self.grabbed_node = None;
        self.hovered_other_margin = None;
        self.chord_delete_consumed = false;

        // Si la pieza activa es un póntico (no lleva margen), enfocar automáticamente el primer pilar
        if case.get_active_unit().map(|u| u.is_pontic()).unwrap_or(false) {
            if let Some(&abutment_fdi) = case
                .restorations
                .iter()
                .find(|(_, u)| !u.is_pontic())
                .map(|(f, _)| f)
            {
                case.set_active_tooth(abutment_fdi);
                let _ = case_manager.save_case(case);
            }
        }

        self.load_arches(case, case_manager);
        self.load_margin_from_case(case);
        self.other_margins_rev = self.bump();

        if let Some((m, _)) = &self.prep_mesh {
            let (mn, mx) = m.bounds();
            self.camera.frame_bounds(mn, mx);
        }
    }

    fn load_arches(&mut self, case: &DentalCase, case_manager: &CaseManager) {
        self.die_roi_cache = None;
        let prep_path = case_manager
            .get_active_scan_path(case, &case.prep_arch)
            .or_else(|| case_manager.get_active_scan_path(case, "mandible"))
            .or_else(|| case_manager.get_active_scan_path(case, "maxilla"));

        if let Some(p) = prep_path {
            if let Ok(mut m) = TriMeshData::load_from_path(&p) {
                self.margin_ctrl.set_target_mesh(m.clone());
                m.custom_scalars = self.margin_ctrl.curvature_weights.clone();
                let rev = self.bump();
                self.prep_mesh = Some((m, rev));
            }
        } else {
            self.prep_mesh = None;
        }

        let ant_key = case.get_antagonist_arch();
        if let Some(p) = case_manager.get_active_scan_path(case, ant_key) {
            if let Ok(m) = TriMeshData::load_from_path(&p) {
                let rev = self.bump();
                self.ant_mesh = Some((m, rev));
            }
        } else {
            self.ant_mesh = None;
        }
    }

    fn load_margin_from_case(&mut self, case: &DentalCase) {
        self.undo_stack.clear();
        self.selected_node = None;
        self.hovered_node = None;
        self.grabbed_node = None;

        let is_pontic = case.get_active_unit().map(|u| u.is_pontic()).unwrap_or(false);
        if is_pontic {
            self.margin_ctrl.clear();
        } else if !case.margin_control_points.is_empty() {
            self.margin_ctrl.load_points(&case.margin_control_points);
        } else if !case.margin_points.is_empty() {
            let step = (case.margin_points.len() / 16).max(1);
            let sub: Vec<[f32; 3]> = case.margin_points.iter().step_by(step).copied().collect();
            self.margin_ctrl.load_points(&sub);
        } else {
            self.margin_ctrl.clear();
        }
        self.margin_rev = self.bump();
        self.other_margins_rev = self.bump();
        self.die_roi_cache = None;
        self.refresh_audit(case, false);
    }

    fn push_undo(&mut self) {
        let snap: Vec<[f32; 3]> = self
            .margin_ctrl
            .control_points
            .iter()
            .map(|p| p.to_array())
            .collect();
        self.undo_stack.push(snap);
        if self.undo_stack.len() > 40 {
            self.undo_stack.remove(0);
        }
    }

    pub fn undo(&mut self, case: &mut DentalCase, case_manager: &CaseManager) -> bool {
        let Some(prev) = self.undo_stack.pop() else {
            return false;
        };
        self.grabbed_node = None;
        self.hovered_node = None;
        self.margin_ctrl.load_points(&prev);
        self.selected_node = self.margin_ctrl.control_points.len().checked_sub(1);
        self.margin_rev = self.bump();
        self.other_margins_rev = self.bump();
        self.refresh_audit(case, false);
        self.persist_margin(case, case_manager);
        true
    }

    pub fn persist_margin(&self, case: &mut DentalCase, case_manager: &CaseManager) {
        if case.get_active_unit().map(|u| u.is_pontic()).unwrap_or(false) {
            let _ = case_manager.save_case(case);
            return;
        }
        let ctrl: Vec<[f32; 3]> = self.margin_ctrl.control_points.iter().map(|p| p.to_array()).collect();
        let curve: Vec<[f32; 3]> = self.margin_ctrl.curve_points.iter().map(|p| p.to_array()).collect();
        case.margin_control_points = ctrl.clone();
        case.margin_points = curve.clone();
        if let Some(u) = case.restorations.get_mut(&case.tooth_number) {
            u.margin_control_points = ctrl;
            u.margin_points = curve;
        }
        let _ = case_manager.save_case(case);
    }

    fn switch_tooth_in_margin(
        &mut self,
        target_fdi: i32,
        case: &mut DentalCase,
        case_manager: &CaseManager,
    ) {
        self.persist_margin(case, case_manager);
        let prev_arch = case.prep_arch.clone();
        case.set_active_tooth(target_fdi);
        let _ = case_manager.save_case(case);
        if case.prep_arch != prev_arch {
            self.load_arches(case, case_manager);
        }
        self.hovered_other_margin = None;
        self.load_margin_from_case(case);
    }

    pub fn auto_detect_default(&mut self, case: &mut DentalCase, case_manager: &CaseManager) {
        if case.get_active_unit().map(|u| u.is_pontic()).unwrap_or(false) {
            return;
        }
        let ax = Vec3::from_array(case.insertion_axis).normalize_or_zero();
        let ax = if ax.length_squared() > 1e-6 { ax } else { Vec3::Z };

        // Buscar cima del muñón cerca del centro antes de mutar self
        let seed = {
            let Some((prep, _)) = &self.prep_mesh else {
                return;
            };
            let mut best_h = f32::NEG_INFINITY;
            let mut best_seed = prep.center();
            for &p in &prep.positions {
                if p.x.abs() < 4.5 && p.y.abs() < 4.5 {
                    let h = p.dot(ax);
                    if h > best_h {
                        best_h = h;
                        best_seed = p;
                    }
                }
            }
            best_seed
        };
        self.push_undo();
        self.margin_ctrl.auto_detect_margin_from_stump_top(seed, ax, 16, 6.2);
        self.selected_node = Some(0);
        self.grabbed_node = None;
        self.margin_rev = self.bump();
        self.other_margins_rev = self.bump();
        self.refresh_audit(case, false);
        self.persist_margin(case, case_manager);
    }

    fn find_hovered_node(&self, pos: Pos2, rect: egui::Rect) -> Option<usize> {
        self.find_hovered_node_with_radius(pos, rect, 14.0)
    }

    fn find_hovered_node_with_radius(&self, pos: Pos2, rect: egui::Rect, max_px: f32) -> Option<usize> {
        let mut best = None;
        let mut best_d = max_px;
        for (i, &pt) in self.margin_ctrl.control_points.iter().enumerate() {
            if let Some(sp) = self.camera.world_to_screen(pt, rect) {
                let d = pos.distance(sp);
                if d <= best_d {
                    best_d = d;
                    best = Some(i);
                }
            }
        }
        best
    }

    fn find_hovered_other_margin(&self, pos: Pos2, rect: egui::Rect, case: &DentalCase) -> Option<i32> {
        let mut best_fdi: Option<i32> = None;
        let mut best_d = 12.0_f32;
        for (&fdi, unit) in &case.restorations {
            if fdi == case.tooth_number || unit.is_pontic() || unit_arch(fdi) != case.prep_arch {
                continue;
            }
            let pts: &[ [f32; 3] ] = if unit.margin_points.len() >= 2 {
                &unit.margin_points
            } else if unit.margin_control_points.len() >= 2 {
                &unit.margin_control_points
            } else {
                continue;
            };
            let closed = unit.margin_control_points.len() >= 3 || unit.margin_points.len() >= 4;
            let n = pts.len();
            let step = if n > 90 { 2 } else { 1 };
            let screen_pts: Vec<Option<Pos2>> = pts
                .iter()
                .step_by(step)
                .map(|p| self.camera.world_to_screen(Vec3::from_array(*p), rect))
                .collect();
            let m = screen_pts.len();
            let segs = if closed { m } else { m.saturating_sub(1) };
            for i in 0..segs {
                if let (Some(a), Some(b)) = (screen_pts[i], screen_pts[(i + 1) % m]) {
                    let d = dist_pt_to_segment_2d(pos, a, b);
                    if d < best_d {
                        best_d = d;
                        best_fdi = Some(fdi);
                    }
                }
            }
            for p in &unit.margin_control_points {
                if let Some(sp) = self.camera.world_to_screen(Vec3::from_array(*p), rect) {
                    let d = pos.distance(sp);
                    if d < best_d {
                        best_d = d;
                        best_fdi = Some(fdi);
                    }
                }
            }
        }
        best_fdi
    }

    fn pick_on_prep(&self, pos: Pos2, rect: egui::Rect) -> Option<Vec3> {
        let col = self.margin_ctrl.collider.as_ref()?;
        let (ro, rd) = self.camera.screen_ray(pos, rect);
        col.cast_ray(ro, rd, 500.0).map(|(_, hit)| hit)
    }

    pub fn ui(
        &mut self,
        ctx: &egui::Context,
        case: &mut DentalCase,
        case_manager: &CaseManager,
        renderer: &Arc<Mutex<GlViewportRenderer>>,
    ) -> MarginAction {
        let mut action = MarginAction::None;
        let active_is_pontic = case.get_active_unit().map(|u| u.is_pontic()).unwrap_or(false);

        // ==================================================================
        // LIENZO 3D A PANTALLA COMPLETA + INTERACCIÓN PURO CLIC CON `parry3d`
        // ==================================================================
        egui::CentralPanel::default()
            .frame(Frame::NONE.fill(theme::BG_DEEP))
            .show(ctx, |ui| {
                let rect = ui.available_rect_before_wrap();
                let resp = ui.allocate_rect(rect, Sense::click_and_drag());

                // Detección prioritaria: Acorde Exocad (Clic Izq + Der simultáneos sobre nodo => Eliminar nodo)
                let prim_down = ui.input(|i| i.pointer.primary_down());
                let sec_down = ui.input(|i| i.pointer.secondary_down());
                let both_down = prim_down && sec_down;
                if !both_down {
                    self.chord_delete_consumed = false;
                }
                if both_down && !self.chord_delete_consumed && resp.hovered() && !active_is_pontic {
                    self.chord_delete_consumed = true;
                    self.press_start_pos = None;
                    let del_target = self.grabbed_node.or_else(|| {
                        ui.input(|i| i.pointer.interact_pos())
                            .and_then(|p| self.find_hovered_node_with_radius(p, rect, 16.5))
                    });
                    if let Some(del_idx) = del_target {
                        self.push_undo();
                        self.margin_ctrl.remove_control_point(Some(del_idx));
                        self.grabbed_node = None;
                        self.hovered_node = None;
                        self.selected_node = self.margin_ctrl.control_points.len().checked_sub(1);
                        self.refresh_audit(case, false);
                        self.margin_rev = self.bump();
                        self.other_margins_rev = self.bump();
                        self.persist_margin(case, case_manager);
                        action = MarginAction::StatusMessage(format!(
                            "{} #{} ({}).",
                            tr("Nodo", "Node", "Nó"),
                            del_idx + 1,
                            tr("Clic Izq + Der", "Left + Right Click", "Clique Esq + Dir")
                        ));
                    }
                }

                // Hover y movimiento libre de nodo agarrado (o detección de otros márgenes en gris)
                if let Some(hover_pos) = resp.hover_pos() {
                    if !active_is_pontic {
                        if let Some(grab_idx) = self.grabbed_node {
                            if !both_down {
                                if let Some(hit) = self.pick_on_prep(hover_pos, rect) {
                                    self.margin_ctrl.move_control_point(grab_idx, hit, self.magnetic_snap);
                                    self.refresh_audit(case, false);
                                    self.margin_rev = self.bump();
                                }
                            }
                        } else if !resp.dragged() {
                            let new_hov = self.find_hovered_node(hover_pos, rect);
                            if new_hov != self.hovered_node {
                                self.hovered_node = new_hov;
                                self.margin_rev = self.bump();
                            }
                            let new_other_hov = if self.hovered_node.is_none() {
                                self.find_hovered_other_margin(hover_pos, rect, case)
                            } else {
                                None
                            };
                            if new_other_hov != self.hovered_other_margin {
                                self.hovered_other_margin = new_other_hov;
                                self.other_margins_rev = self.bump();
                            }
                        }
                    } else if !resp.dragged() {
                        let new_other_hov = self.find_hovered_other_margin(hover_pos, rect, case);
                        if new_other_hov != self.hovered_other_margin {
                            self.hovered_other_margin = new_other_hov;
                            self.other_margins_rev = self.bump();
                        }
                    }
                }

                // Press inicial: fijar nodo agarrado / agarrar nodo / cambiar a otro margen gris / preparar clic en muñón
                let primary_pressed = ui.input(|i| i.pointer.primary_pressed());
                if primary_pressed && resp.hovered() && !both_down && !self.chord_delete_consumed {
                    if let Some(p_pos) = ui.input(|i| i.pointer.interact_pos()) {
                        if !active_is_pontic {
                            if let Some(grab_idx) = self.grabbed_node {
                                if let Some(hit) = self.pick_on_prep(p_pos, rect) {
                                    self.margin_ctrl.move_control_point(grab_idx, hit, self.magnetic_snap);
                                }
                                self.selected_node = Some(grab_idx);
                                self.grabbed_node = None;
                                self.press_start_pos = None;
                                self.refresh_audit(case, false);
                                self.margin_rev = self.bump();
                                self.other_margins_rev = self.bump();
                                self.persist_margin(case, case_manager);
                            } else if let Some(hov_idx) = self.find_hovered_node(p_pos, rect) {
                                self.push_undo();
                                self.grabbed_node = Some(hov_idx);
                                self.selected_node = Some(hov_idx);
                                self.hovered_node = Some(hov_idx);
                                self.press_start_pos = None;
                                self.margin_rev = self.bump();
                            } else if let Some(other_fdi) = self.find_hovered_other_margin(p_pos, rect, case) {
                                self.press_start_pos = None;
                                self.switch_tooth_in_margin(other_fdi, case, case_manager);
                                action = MarginAction::StatusMessage(format!(
                                    "Margen de Pieza #{} activado para edición.",
                                    other_fdi
                                ));
                            } else {
                                self.press_start_pos = Some(p_pos);
                            }
                        } else if let Some(other_fdi) = self.find_hovered_other_margin(p_pos, rect, case) {
                            self.press_start_pos = None;
                            self.switch_tooth_in_margin(other_fdi, case, case_manager);
                            action = MarginAction::StatusMessage(format!(
                                "Margen de Pieza #{} activado para edición.",
                                other_fdi
                            ));
                        }
                    }
                }

                // Cámara 3D libre 360° (Clic Der = Rotar 360°; Rueda = Pan/Zoom; Clic Izq = Solo acción)
                self.camera.handle_navigation_input(&resp, ui, rect, true);

                // Release sin arrastre (<= 5 px) = Puro Clic en muñón
                let primary_released = ui.input(|i| i.pointer.primary_released());
                if primary_released && !both_down && !self.chord_delete_consumed && !active_is_pontic {
                    if let (Some(start_p), Some(end_p)) = (self.press_start_pos.take(), ui.input(|i| i.pointer.interact_pos())) {
                        if start_p.distance(end_p) <= 5.5 && rect.contains(end_p) {
                            if let Some(picked) = self.pick_on_prep(end_p, rect) {
                                self.push_undo();
                                match self.mode {
                                    MarginEditMode::Add => {
                                        let idx = self.margin_ctrl.add_control_point(picked, self.magnetic_snap, false);
                                        self.selected_node = Some(idx);
                                    }
                                    MarginEditMode::Insert => {
                                        let idx = self.margin_ctrl.add_control_point(picked, self.magnetic_snap, true);
                                        self.selected_node = Some(idx);
                                    }
                                    MarginEditMode::Move => {
                                        if let Some(near) = self.margin_ctrl.find_nearest_control_point(picked, 1.6) {
                                            self.grabbed_node = Some(near);
                                            self.selected_node = Some(near);
                                        } else if let Some(sel) = self.selected_node {
                                            self.margin_ctrl.move_control_point(sel, picked, self.magnetic_snap);
                                        }
                                    }
                                    MarginEditMode::AutoDetect => {
                                        let ax = Vec3::from_array(case.insertion_axis);
                                        self.margin_ctrl.auto_detect_margin_from_stump_top(picked, ax, 16, 6.2);
                                        self.selected_node = Some(0);
                                    }
                                }
                                self.refresh_audit(case, false);
                                self.margin_rev = self.bump();
                                self.other_margins_rev = self.bump();
                                self.persist_margin(case, case_manager);
                            }
                        }
                    }
                } else if primary_released {
                    self.press_start_pos = None;
                }

                // Subir y ensamblar capas 3D
                let mut layers = Vec::new();
                let active_after = case.tooth_number;
                let active_is_pontic_now = case.get_active_unit().map(|u| u.is_pontic()).unwrap_or(false);

                if let Ok(mut guard) = renderer.lock() {
                    if self.show_prep {
                        if let Some((prep, rev)) = &self.prep_mesh {
                            let cmap = if self.show_curvature {
                                ColorMapKind::CurvatureMagma
                            } else {
                                ColorMapKind::Solid
                            };
                            let k = if self.show_curvature { "m3_prep_curv" } else { "m3_prep" };
                            guard.ensure_mesh_uploaded(k, *rev, prep, cmap);
                            let mut sp = RenderLayerSpec::solid(k, *rev, [0.94, 0.89, 0.81], self.prep_opacity);
                            sp.use_vertex_color = self.show_curvature;
                            sp.specular = 0.30;
                            layers.push(sp);
                        }
                    }
                    if self.show_ant {
                        if let Some((ant, rev)) = &self.ant_mesh {
                            guard.ensure_mesh_uploaded("m3_ant", *rev, ant, ColorMapKind::Solid);
                            layers.push(RenderLayerSpec::solid("m3_ant", *rev, [0.85, 0.80, 0.89], self.ant_opacity));
                        }
                    }

                    // Otros márgenes ya trazados en la misma arcada (en gris clicable)
                    let r_line = self.line_radius_mm.clamp(0.015, 0.120);
                    let r_node = self.node_radius_mm.clamp(0.050, 0.220);
                    let mut gray_lines = TriMeshData::default();
                    let mut gray_nodes = TriMeshData::default();
                    let mut hov_other_mesh = TriMeshData::default();

                    for (&fdi, unit) in &case.restorations {
                        if fdi == active_after || unit.is_pontic() || unit_arch(fdi) != case.prep_arch {
                            continue;
                        }
                        let is_hov = self.hovered_other_margin == Some(fdi);
                        let closed = unit.margin_control_points.len() >= 3 || unit.margin_points.len() >= 4;
                        let curve_pts: Vec<Vec3> = if unit.margin_points.len() >= 2 {
                            unit.margin_points.iter().map(|p| Vec3::from_array(*p)).collect()
                        } else {
                            unit.margin_control_points.iter().map(|p| Vec3::from_array(*p)).collect()
                        };
                        if curve_pts.len() >= 2 {
                            let rad = if is_hov { r_line * 1.32 } else { r_line * 0.92 };
                            let tube = build_tube_mesh(&curve_pts, rad, closed, 8);
                            if is_hov {
                                hov_other_mesh.merge_in_place(&tube);
                            } else {
                                gray_lines.merge_in_place(&tube);
                            }
                        }
                        for p in &unit.margin_control_points {
                            let sp = build_uv_sphere(Vec3::from_array(*p), r_node * 0.78, 6, 8);
                            if is_hov {
                                hov_other_mesh.merge_in_place(&sp);
                            } else {
                                gray_nodes.merge_in_place(&sp);
                            }
                        }
                    }

                    if !gray_lines.is_empty() {
                        guard.ensure_mesh_uploaded("m3_other_lines", self.other_margins_rev, &gray_lines, ColorMapKind::Solid);
                        let mut sp = RenderLayerSpec::solid("m3_other_lines", self.other_margins_rev, [0.56, 0.57, 0.63], 0.94);
                        sp.specular = 0.25;
                        layers.push(sp);
                    }
                    if !gray_nodes.is_empty() {
                        guard.ensure_mesh_uploaded("m3_other_nodes", self.other_margins_rev, &gray_nodes, ColorMapKind::Solid);
                        layers.push(RenderLayerSpec::solid("m3_other_nodes", self.other_margins_rev, [0.64, 0.65, 0.71], 0.94));
                    }
                    if !hov_other_mesh.is_empty() {
                        guard.ensure_mesh_uploaded("m3_other_hov", self.other_margins_rev, &hov_other_mesh, ColorMapKind::Solid);
                        let mut sp = RenderLayerSpec::solid("m3_other_hov", self.other_margins_rev, [0.80, 0.87, 0.96], 1.0);
                        sp.specular = 0.65;
                        layers.push(sp);
                    }

                    // Línea vectorial activa + nodos de control (solo para piezas no pónticas)
                    if !active_is_pontic_now {
                        if self.margin_ctrl.curve_points.len() >= 2 {
                            let closed = self.margin_ctrl.control_points.len() >= 3;
                            let tube = build_tube_mesh(&self.margin_ctrl.curve_points, r_line, closed, 10);
                            guard.ensure_mesh_uploaded("m3_tube", self.margin_rev, &tube, ColorMapKind::Solid);
                            let mut sp = RenderLayerSpec::solid("m3_tube", self.margin_rev, [0.20, 0.85, 0.62], 1.0);
                            sp.specular = 0.55;
                            layers.push(sp);

                            let n_cp = self.margin_ctrl.curve_points.len();
                            if self.margin_audit.bad_curve_mask.len() == n_cp
                                && self.margin_audit.bad_curve_mask.iter().any(|&b| b)
                            {
                                let mut bad_tube = TriMeshData::default();
                                let mut i = 0usize;
                                while i < n_cp {
                                    if !self.margin_audit.bad_curve_mask[i] {
                                        i += 1;
                                        continue;
                                    }
                                    let start = i;
                                    while i < n_cp && self.margin_audit.bad_curve_mask[i] {
                                        i += 1;
                                    }
                                    let end = i;
                                    let mut seg_pts: Vec<Vec3> = Vec::with_capacity(end - start + 2);
                                    if start > 0 {
                                        seg_pts.push(self.margin_ctrl.curve_points[start - 1]);
                                    } else if closed {
                                        seg_pts.push(self.margin_ctrl.curve_points[n_cp - 1]);
                                    }
                                    seg_pts.extend_from_slice(&self.margin_ctrl.curve_points[start..end]);
                                    if end < n_cp {
                                        seg_pts.push(self.margin_ctrl.curve_points[end]);
                                    } else if closed {
                                        seg_pts.push(self.margin_ctrl.curve_points[0]);
                                    }
                                    if seg_pts.len() >= 2 {
                                        bad_tube.merge_in_place(&build_tube_mesh(&seg_pts, r_line * 1.30, false, 10));
                                    }
                                }
                                if !bad_tube.is_empty() {
                                    guard.ensure_mesh_uploaded("m3_bad_tube", self.margin_rev, &bad_tube, ColorMapKind::Solid);
                                    let alert_rgb = if self.margin_audit.blocks_design {
                                        [0.96, 0.22, 0.22]
                                    } else {
                                        [0.98, 0.62, 0.10]
                                    };
                                    let mut sp_b = RenderLayerSpec::solid("m3_bad_tube", self.margin_rev, alert_rgb, 1.0);
                                    sp_b.specular = 0.60;
                                    layers.push(sp_b);
                                }
                            }
                        }

                        if !self.margin_ctrl.control_points.is_empty() {
                            let is_bad_node = |idx: usize| self.margin_audit.bad_node_indices.contains(&idx);
                            let is_blocking = self.margin_audit.blocks_design;
                            let r_bad = (r_node * 1.25).clamp(0.068, 0.250);
                            let r_hi = (r_node * 1.50).clamp(0.085, 0.300);
                            let mut regular = TriMeshData::default();
                            let mut bad_mesh = TriMeshData::default();
                            let active_hi = self.grabbed_node.or(self.selected_node);
                            for (idx, &pt) in self.margin_ctrl.control_points.iter().enumerate() {
                                if Some(idx) == active_hi || Some(idx) == self.hovered_node {
                                    continue;
                                }
                                if is_bad_node(idx) {
                                    bad_mesh.merge_in_place(&build_uv_sphere(pt, r_bad, 10, 12));
                                } else {
                                    regular.merge_in_place(&build_uv_sphere(pt, r_node, 8, 10));
                                }
                            }
                            if !regular.is_empty() {
                                guard.ensure_mesh_uploaded("m3_nodes", self.margin_rev, &regular, ColorMapKind::Solid);
                                layers.push(RenderLayerSpec::solid("m3_nodes", self.margin_rev, [0.95, 0.92, 0.83], 1.0));
                            }
                            if !bad_mesh.is_empty() {
                                guard.ensure_mesh_uploaded("m3_bad_nodes", self.margin_rev, &bad_mesh, ColorMapKind::Solid);
                                let bad_rgb = if is_blocking {
                                    [0.97, 0.16, 0.19]
                                } else {
                                    [0.99, 0.64, 0.12]
                                };
                                let mut sp_bad = RenderLayerSpec::solid("m3_bad_nodes", self.margin_rev, bad_rgb, 1.0);
                                sp_bad.specular = 0.65;
                                layers.push(sp_bad);
                            }

                            if let Some(hi) = active_hi {
                                if let Some(&pt) = self.margin_ctrl.control_points.get(hi) {
                                    let sp_m = build_uv_sphere(pt, r_hi, 10, 12);
                                    guard.ensure_mesh_uploaded("m3_sel_node", self.margin_rev, &sp_m, ColorMapKind::Solid);
                                    let col = if self.grabbed_node.is_some() {
                                        if is_bad_node(hi) {
                                            if is_blocking { [1.00, 0.24, 0.18] } else { [1.00, 0.68, 0.12] }
                                        } else {
                                            [0.24, 0.90, 0.64]
                                        }
                                    } else if is_bad_node(hi) {
                                        if is_blocking { [1.00, 0.16, 0.20] } else { [0.99, 0.64, 0.12] }
                                    } else {
                                        [0.78, 0.49, 1.00]
                                    };
                                    layers.push(RenderLayerSpec::solid("m3_sel_node", self.margin_rev, col, 1.0));
                                }
                            }

                            if let Some(hov) = self.hovered_node {
                                if Some(hov) != active_hi {
                                    if let Some(&pt) = self.margin_ctrl.control_points.get(hov) {
                                        let sp_m = build_uv_sphere(pt, r_hi, 10, 12);
                                        guard.ensure_mesh_uploaded("m3_hov_node", self.margin_rev, &sp_m, ColorMapKind::Solid);
                                        let col = if is_bad_node(hov) {
                                            if is_blocking { [1.00, 0.34, 0.34] } else { [1.00, 0.76, 0.28] }
                                        } else {
                                            [0.78, 0.49, 1.00]
                                        };
                                        layers.push(RenderLayerSpec::solid("m3_hov_node", self.margin_rev, col, 1.0));
                                    }
                                }
                            }
                        }
                    }
                }

                ui.painter().add(make_viewport_paint_callback(
                    Arc::clone(renderer),
                    rect,
                    &self.camera,
                    layers,
                    None,
                ));

                // Etiquetas FDI flotantes discretas sobre los márgenes en gris para seleccionarlos directo
                let painter = ui.painter().with_clip_rect(rect);
                for (&fdi, unit) in &case.restorations {
                    if fdi == active_after || unit.is_pontic() || unit_arch(fdi) != case.prep_arch {
                        continue;
                    }
                    let pts: &[ [f32; 3] ] = if !unit.margin_points.is_empty() {
                        &unit.margin_points
                    } else if !unit.margin_control_points.is_empty() {
                        &unit.margin_control_points
                    } else {
                        continue;
                    };
                    let ctr: Vec3 = pts.iter().map(|p| Vec3::from_array(*p)).sum::<Vec3>() / pts.len() as f32;
                    if let Some(sp) = self.camera.world_to_screen(ctr, rect) {
                        let is_hov = self.hovered_other_margin == Some(fdi);
                        let badge_r = egui::Rect::from_center_size(sp, egui::vec2(34.0, 17.0));
                        painter.rect(
                            badge_r,
                            CornerRadius::same(4),
                            if is_hov {
                                Color32::from_rgba_unmultiplied(0x2D, 0x10, 0x45, 235)
                            } else {
                                Color32::from_rgba_unmultiplied(0x26, 0x26, 0x2C, 210)
                            },
                            Stroke::new(1.0_f32, if is_hov { CYAN } else { Color32::from_rgb(0x9A, 0x9E, 0xAA) }),
                            egui::StrokeKind::Inside,
                        );
                        painter.text(
                            badge_r.center(),
                            Align2::CENTER_CENTER,
                            format!("#{}", fdi),
                            FontId::proportional(10.0),
                            if is_hov { Color32::WHITE } else { Color32::from_rgb(0xE2, 0xE5, 0xEC) },
                        );
                    }
                }
            });

        // ==================================================================
        // BARRA FLOTANTE SUPERIOR CENTRAL: Vistas Rápidas
        // ==================================================================
        Area::new(egui::Id::new("m3_cam_bar"))
            .anchor(Align2::CENTER_TOP, egui::vec2(0.0, 68.0))
            .order(Order::Foreground)
            .show(ctx, |ui| {
                Frame::NONE
                    .fill(egui::Color32::from_rgba_unmultiplied(0x21, 0x05, 0x2C, 238))
                    .stroke(Stroke::new(1.3_f32, AMETHYST))
                    .corner_radius(CornerRadius::same(8))
                    .inner_margin(Margin::symmetric(8, 5))
                    .show(ui, |ui| {
                        ui.horizontal(|ui| {
                            if ui.button(tr("Oclusal (+Z)", "Occlusal (+Z)", "Oclusal (+Z)")).clicked() { self.camera.set_view_occlusal_top(); }
                            if ui.button(tr("Inferior (-Z)", "Apical (-Z)", "Inferior (-Z)")).clicked() { self.camera.set_view_occlusal_bottom(); }
                            if ui.button(tr("Vestibular", "Buccal", "Vestibular")).clicked() { self.camera.set_view_frontal(); }
                            if ui.button("Lingual").clicked() { self.camera.set_view_lingual(); }
                            if ui.button("Mesial/Distal").clicked() { self.camera.set_view_lateral(); }
                            if ui.button(tr("Centrar Muñón", "Center Die", "Centralizar Preparo")).clicked() {
                                if let Some((m, _)) = &self.prep_mesh {
                                    let (mn, mx) = m.bounds();
                                    self.camera.frame_bounds(mn, mx);
                                }
                            }
                        });
                    });
            });

        // ==================================================================
        // VENTANA FLOTANTE IZQ 1: Pieza Activa y Herramienta Puro Clic
        // ==================================================================
        Window::new(tr("1. Pieza Activa y Margen (Puro Clic)", "1. Active Unit & Margin Line", "1. Dente Ativo e Linha de Término"))
            .id(egui::Id::new("m3_card_tools"))
            .default_pos(egui::pos2(14.0, 74.0))
            .default_width(330.0)
            .collapsible(true)
            .resizable(false)
            .show(ctx, |ui| {
                let prev_tooth = case.tooth_number;
                let mut chosen_tooth = prev_tooth;
                ui.horizontal(|ui| {
                    ui.label(tr("Pieza:", "Unit:", "Dente:"));
                    egui::ComboBox::from_id_salt("m3_tooth_combo")
                        .width(240.0)
                        .selected_text(format!(
                            "#{} — {}",
                            case.tooth_number,
                            case.get_active_unit()
                                .map(|u| i18n::tr_process_type(&u.process_type))
                                .unwrap_or_default()
                        ))
                        .show_ui(ui, |ui| {
                            for (&fdi, u) in &case.restorations {
                                let suffix = if u.is_pontic() {
                                    tr("Sin margen", "No margin", "Sem término").to_string()
                                } else if !u.margin_control_points.is_empty() {
                                    format!("✔ {} {}", u.margin_control_points.len(), tr("nodos", "nodes", "nós"))
                                } else {
                                    tr("Pendiente", "Pending", "Pendente").to_string()
                                };
                                ui.selectable_value(
                                    &mut chosen_tooth,
                                    fdi,
                                    format!("#{} — {} [{}]", fdi, i18n::tr_process_type(&u.process_type), suffix),
                                );
                            }
                        });
                });

                // Lista rápida clicable de todas las piezas de la orden
                if case.restorations.len() > 1 {
                    ui.horizontal_wrapped(|ui| {
                        for (&fdi, u) in &case.restorations {
                            let active = fdi == case.tooth_number;
                            let tag = if u.is_pontic() {
                                format!("#{} {}", fdi, tr("Pón.", "Pon.", "Pôn."))
                            } else if !u.margin_control_points.is_empty() {
                                format!("#{} ✔", fdi)
                            } else {
                                format!("#{} ○", fdi)
                            };
                            if ui.selectable_label(active, RichText::new(tag).size(10.5)).clicked() {
                                chosen_tooth = fdi;
                            }
                        }
                    });
                }

                if chosen_tooth != prev_tooth {
                    self.switch_tooth_in_margin(chosen_tooth, case, case_manager);
                }

                let is_pontic_now = case.get_active_unit().map(|u| u.is_pontic()).unwrap_or(false);
                if is_pontic_now {
                    Frame::NONE
                        .fill(Color32::from_rgba_unmultiplied(0x0D, 0x33, 0x3A, 220))
                        .stroke(Stroke::new(1.2_f32, CYAN))
                        .corner_radius(CornerRadius::same(6))
                        .inner_margin(Margin::same(8))
                        .show(ui, |ui| {
                            ui.label(
                                RichText::new(format!(
                                    "ℹ {} #{} {}\n{}\n{}",
                                    tr("Pieza", "Unit", "Dente"),
                                    case.tooth_number,
                                    tr("es un Póntico de Puente.", "is a Bridge Pontic.", "é um Pôntico de Ponte."),
                                    tr("No requiere línea de margen cervical.", "Does not require a cervical margin line.", "Não requer linha de término cervical."),
                                    tr("Haz clic en otro pilar o en un margen gris del modelo para editarlo, o continúa directo al Diseño.", "Click another abutment or a gray margin line on the model to edit, or proceed straight to Design.", "Clique em outro pilar ou em um término cinza no modelo para editar, ou avance direto ao Desenho.")
                                ))
                                .size(10.5)
                                .color(CREAM),
                            );
                        });
                } else {
                    Frame::NONE
                        .fill(theme::BG_DEEP)
                        .stroke(Stroke::new(1.0_f32, theme::BORDER_PLUM))
                        .corner_radius(CornerRadius::same(5))
                        .inner_margin(Margin::same(6))
                        .show(ui, |ui| {
                            ui.label(
                                RichText::new(tr(
                                    "• Puro Clic Izq: Ejecutar herramienta en muñón\n\
                                     • Clic en Nodo: Mover libremente y Clic para fijar\n\
                                     • Clic Izq + Der en Nodo: Eliminar nodo\n\
                                     • Clic en Margen Gris: Activar esa pieza\n\
                                     • Clic Der + Arrastrar: Rotar 360° | Rueda: Pan/Zoom",
                                    "• Left-Click: Place / apply tool on die\n\
                                     • Click Node: Move freely & click to place\n\
                                     • Left + Right Click Node: Delete node\n\
                                     • Click Gray Margin: Switch to that unit\n\
                                     • Right-Drag: Orbit 360° | Wheel: Pan/Zoom",
                                    "• Clique Esq: Aplicar ferramenta no preparo\n\
                                     • Clique no Nó: Mover livremente e clicar para fixar\n\
                                     • Clique Esq + Dir no Nó: Excluir nó\n\
                                     • Clique no Término Cinza: Ativar esse dente\n\
                                     • Clique Dir + Arrastar: Girar 360° | Roda: Pan/Zoom",
                                ))
                                .size(10.2)
                                .color(CREAM),
                            );
                        });

                    ui.add_space(4.0);
                    egui::ComboBox::from_id_salt("m3_mode_combo")
                        .width(ui.available_width())
                        .selected_text(match self.mode {
                            MarginEditMode::Add => tr("1. Agregar Puntos de Margen (Clic)", "1. Add Margin Points (Click)", "1. Adicionar Pontos de Término (Clique)"),
                            MarginEditMode::Insert => tr("2. Insertar Punto en Tramo (Clic)", "2. Insert Point on Segment (Click)", "2. Inserir Ponto no Trecho (Clique)"),
                            MarginEditMode::Move => tr("3. Mover Nodos (Clic -> Mover -> Clic)", "3. Move Nodes (Click -> Move -> Click)", "3. Mover Nós (Clique -> Mover -> Clique)"),
                            MarginEditMode::AutoDetect => tr("4. ★ Auto-Detectar Margen (Clic Cima)", "4. ★ Auto-Detect Margin (Click Occlusal)", "4. ★ Auto-Detectar Término (Clique Topo)"),
                        })
                        .show_ui(ui, |ui| {
                            ui.selectable_value(&mut self.mode, MarginEditMode::Add, tr("1. Agregar Puntos de Margen (Clic)", "1. Add Margin Points (Click)", "1. Adicionar Pontos de Término (Clique)"));
                            ui.selectable_value(&mut self.mode, MarginEditMode::Insert, tr("2. Insertar Punto en Tramo (Clic)", "2. Insert Point on Segment (Click)", "2. Inserir Ponto no Trecho (Clique)"));
                            ui.selectable_value(&mut self.mode, MarginEditMode::Move, tr("3. Mover Nodos (Clic -> Mover -> Clic)", "3. Move Nodes (Click -> Move -> Click)", "3. Mover Nós (Clique -> Mover -> Clique)"));
                            ui.selectable_value(&mut self.mode, MarginEditMode::AutoDetect, tr("4. ★ Auto-Detectar Margen (Clic Cima)", "4. ★ Auto-Detect Margin (Click Occlusal)", "4. ★ Auto-Detectar Término (Clique Topo)"));
                        });

                    ui.checkbox(&mut self.magnetic_snap, tr("Imán Magnético a Cresta de Hombro / Chamfer", "Magnetic Snap to Finish-Line Crest", "Ímã Magnético à Crista do Término"));
                    ui.checkbox(&mut self.show_curvature, tr("Resaltar Mapa de Curvatura Cervical", "Highlight Cervical Curvature Map", "Destacar Mapa de Curvatura Cervical"));

                    if theme::accent_button(ui, tr("★ Auto-Detectar Margen (Muñón Centrado)", "★ Auto-Detect Margin (Centered Die)", "★ Auto-Detectar Término (Preparo Central)")).clicked() {
                        self.auto_detect_default(case, case_manager);
                        action = MarginAction::StatusMessage(tr(
                            "Línea de margen detectada automáticamente (16 nodos).",
                            "Margin line auto-detected (16 control nodes).",
                            "Linha de término detectada automaticamente (16 nós).",
                        ).to_string());
                    }
                }
            });

        // ==================================================================
        // VENTANA FLOTANTE IZQ 2: Control y Edición de Nodos
        // ==================================================================
        let active_is_pontic_card = case.get_active_unit().map(|u| u.is_pontic()).unwrap_or(false);
        Window::new(tr("2. Control y Edición de Nodos", "2. Control Node Editing", "2. Controle e Edição de Nós"))
            .id(egui::Id::new("m3_card_nodes"))
            .default_pos(egui::pos2(14.0, 415.0))
            .default_width(330.0)
            .collapsible(true)
            .resizable(false)
            .show(ctx, |ui| {
                if active_is_pontic_card {
                    ui.label(
                        RichText::new(tr("Póntico seleccionado — Sin línea de terminación requerida.", "Pontic selected — No finish line required.", "Pôntico selecionado — Nenhuma linha de término necessária."))
                            .size(10.5)
                            .color(CYAN),
                    );
                } else {
                    ui.label(
                        RichText::new(format!(
                            "{}: {}  |  {}: {}",
                            tr("Nodos de control", "Control nodes", "Nós de controle"),
                            self.margin_ctrl.control_points.len(),
                            tr("Puntos spline", "Spline points", "Pontos spline"),
                            self.margin_ctrl.curve_points.len()
                        ))
                        .strong()
                        .color(AMETHYST_BRIGHT),
                    );

                    let status_str = if let Some(g) = self.grabbed_node {
                        format!(
                            "✥ {} #{} — {}",
                            tr("Moviendo Nodo", "Moving Node", "Movendo Nó"),
                            g + 1,
                            tr("Haz Clic para fijarlo (Izq+Der lo borra)", "Click to drop (Left+Right deletes)", "Clique para fixar (Esq+Dir exclui)")
                        )
                    } else if let Some(h) = self.hovered_node {
                        format!(
                            "◉ {} #{} — {}",
                            tr("Nodo", "Node", "Nó"),
                            h + 1,
                            tr("Clic: mover | Clic Izq+Der: borrar", "Click: move | Left+Right: delete", "Clique: mover | Esq+Dir: excluir")
                        )
                    } else if let Some(s) = self.selected_node {
                        format!("{}: #{}", tr("Nodo activo", "Active node", "Nó ativo"), s + 1)
                    } else {
                        tr("Pasa el mouse sobre un nodo para resaltarlo", "Hover over a node to highlight it", "Passe o mouse sobre um nó para destacá-lo").to_string()
                    };
                    ui.label(RichText::new(status_str).size(10.5).color(EMERALD));

                    if !self.margin_ctrl.control_points.is_empty() {
                        if !self.margin_audit.valid_for_cement {
                            let (fill_c, stroke_c, txt_c) = if self.margin_audit.blocks_design {
                                (
                                    Color32::from_rgba_unmultiplied(0x52, 0x0A, 0x16, 220),
                                    Color32::from_rgb(0xF8, 0x38, 0x42),
                                    Color32::from_rgb(0xFF, 0xD2, 0xD5),
                                )
                            } else {
                                (
                                    Color32::from_rgba_unmultiplied(0x47, 0x2B, 0x06, 220),
                                    Color32::from_rgb(0xF5, 0x9E, 0x0B),
                                    Color32::from_rgb(0xFE, 0xF3, 0xC7),
                                )
                            };
                            Frame::NONE
                                .fill(fill_c)
                                .stroke(Stroke::new(1.2_f32, stroke_c))
                                .corner_radius(CornerRadius::same(6))
                                .inner_margin(Margin::same(6))
                                .show(ui, |ui| {
                                    ui.label(RichText::new(&self.margin_audit.message).strong().size(10.5).color(txt_c));
                                });
                        } else if self.margin_ctrl.control_points.len() >= 4 {
                            ui.label(
                                RichText::new(tr("✔ Margen verificado: cementado 100% libre de interferencias", "✔ Margin verified: 100% interference-free cementation", "✔ Término verificado: cimentação 100% livre de interferências"))
                                    .strong()
                                    .size(10.5)
                                    .color(EMERALD),
                            );
                        }
                    }

                    ui.horizontal(|ui| {
                        if ui.button(tr("Ordenar 360°", "Sort 360°", "Ordenar 360°")).clicked() && self.margin_ctrl.control_points.len() >= 3 {
                            self.push_undo();
                            self.margin_ctrl.order_points_angularly(Vec3::from_array(case.insertion_axis));
                            self.refresh_audit(case, false);
                            self.margin_rev = self.bump();
                            self.other_margins_rev = self.bump();
                            self.persist_margin(case, case_manager);
                        }
                        if ui.button(tr("Borrar Nodo / Último", "Delete Node / Last", "Excluir Nó / Último")).clicked() && !self.margin_ctrl.control_points.is_empty() {
                            self.push_undo();
                            self.grabbed_node = None;
                            let target = self.hovered_node.or(self.selected_node);
                            self.margin_ctrl.remove_control_point(target);
                            self.selected_node = self.margin_ctrl.control_points.len().checked_sub(1);
                            self.hovered_node = None;
                            self.refresh_audit(case, false);
                            self.margin_rev = self.bump();
                            self.other_margins_rev = self.bump();
                            self.persist_margin(case, case_manager);
                        }
                    });
                    ui.horizontal(|ui| {
                        if ui
                            .button(tr("🎯 Auto-Eje Muñón", "🎯 Auto Insertion Axis", "🎯 Auto-Eixo Preparo"))
                            .on_hover_text(tr("Calcula automáticamente el eje de inserción óptimo para este diente", "Automatically computes the optimal path of insertion for this die", "Calcula automaticamente o eixo de inserção ideal para este preparo"))
                            .clicked()
                            && self.margin_ctrl.control_points.len() >= 4
                        {
                            if let Some((tilt_deg, rem_ov)) = self.auto_optimize_insertion_axis(case, case_manager) {
                                action = MarginAction::StatusMessage(if rem_ov == 0 {
                                    format!("{} ({:.1}°): 0 {}.", tr("Eje de inserción optimizado", "Insertion axis optimized", "Eixo de inserção otimizado"), tilt_deg, tr("socavados detectados", "undercuts detected", "retenções detectadas"))
                                } else {
                                    format!("{} ({:.1}°): {} {}.", tr("Eje ajustado", "Axis adjusted", "Eixo ajustado"), tilt_deg, rem_ov, tr("tramo(s) con bloqueo virtual automático", "segment(s) with virtual undercut blockout", "trecho(s) com bloqueio virtual automático"))
                                });
                            }
                        }
                        if ui
                            .button(tr("👁 Eje desde Vista", "👁 Set Axis from View", "👁 Eixo da Vista"))
                            .on_hover_text(tr("Fija el eje de inserción siguiendo tu vista 3D actual", "Sets the insertion path along your current 3D viewing direction", "Define o eixo de inserção conforme a vista 3D atual"))
                            .clicked()
                        {
                            self.set_axis_from_camera(case, case_manager);
                            action = MarginAction::StatusMessage(
                                tr("Eje de inserción de la pieza actualizado desde tu vista 3D.", "Unit insertion axis updated from current 3D view.", "Eixo de inserção do dente atualizado a partir da vista 3D.").to_string(),
                            );
                        }
                    });
                    ui.horizontal(|ui| {
                        if theme::danger_button(ui, tr("Limpiar Margen", "Clear Margin", "Limpar Término")).clicked() && !self.margin_ctrl.control_points.is_empty() {
                            self.push_undo();
                            self.margin_ctrl.clear();
                            self.selected_node = None;
                            self.hovered_node = None;
                            self.grabbed_node = None;
                            self.die_roi_cache = None;
                            self.refresh_audit(case, false);
                            self.margin_rev = self.bump();
                            self.other_margins_rev = self.bump();
                            self.persist_margin(case, case_manager);
                        }
                        if ui.button(tr("Guardar Margen", "Save Margin", "Salvar Término")).clicked() {
                            self.refresh_audit(case, false);
                            self.persist_margin(case, case_manager);
                            action = MarginAction::StatusMessage(tr("Línea de margen guardada en disco.", "Margin line saved to disk.", "Linha de término salva no disco.").to_string());
                        }
                    });
                }
            });

        // ==================================================================
        // ALERTA FLOTANTE CLÍNICA EN VIVO (ROJO si cruza; ÁMBAR informativo si hay socavado)
        // ==================================================================
        if !active_is_pontic_card
            && !self.margin_audit.valid_for_cement
            && !self.margin_ctrl.control_points.is_empty()
        {
            let is_blocking = self.margin_audit.blocks_design;
            let (bg_c, bd_c, fg_c) = if is_blocking {
                (
                    Color32::from_rgba_unmultiplied(0x46, 0x09, 0x15, 245),
                    Color32::from_rgb(0xFF, 0x3B, 0x47),
                    Color32::from_rgb(0xFF, 0xE5, 0xE8),
                )
            } else {
                (
                    Color32::from_rgba_unmultiplied(0x3D, 0x24, 0x05, 242),
                    Color32::from_rgb(0xF5, 0x9E, 0x0B),
                    Color32::from_rgb(0xFE, 0xF3, 0xC7),
                )
            };
            Area::new(egui::Id::new("m3_audit_alert"))
                .anchor(Align2::CENTER_BOTTOM, egui::vec2(0.0, -40.0))
                .order(Order::Foreground)
                .show(ctx, |ui| {
                    Frame::NONE
                        .fill(bg_c)
                        .stroke(Stroke::new(1.6_f32, bd_c))
                        .corner_radius(CornerRadius::same(8))
                        .inner_margin(Margin::symmetric(12, 7))
                        .show(ui, |ui| {
                            ui.horizontal(|ui| {
                                ui.label(RichText::new(&self.margin_audit.message).strong().size(11.0).color(fg_c));
                                if is_blocking && self.margin_ctrl.control_points.len() >= 3 {
                                    if ui.button(tr("↻ Ordenar 360°", "↻ Sort 360°", "↻ Ordenar 360°")).clicked() {
                                        self.push_undo();
                                        self.margin_ctrl
                                            .order_points_angularly(Vec3::from_array(case.insertion_axis));
                                        self.refresh_audit(case, false);
                                        self.margin_rev = self.bump();
                                        self.other_margins_rev = self.bump();
                                        self.persist_margin(case, case_manager);
                                    }
                                } else if self.margin_audit.has_undercut_warning
                                    && self.margin_ctrl.control_points.len() >= 4
                                {
                                    if ui.button(tr("🎯 Auto-Alinear Eje", "🎯 Auto-Align Axis", "🎯 Auto-Alinhar Eixo")).clicked() {
                                        if let Some((tilt_deg, rem_ov)) =
                                            self.auto_optimize_insertion_axis(case, case_manager)
                                        {
                                            action = MarginAction::StatusMessage(if rem_ov == 0 {
                                                format!(
                                                    "{} ({:.1}°): 0 {}.",
                                                    tr("Eje alineado automáticamente", "Axis auto-aligned", "Eixo alinhado automaticamente"),
                                                    tilt_deg,
                                                    tr("socavados", "undercuts", "retenções")
                                                )
                                            } else {
                                                format!(
                                                    "{} ({:.1}°): {} {}.",
                                                    tr("Eje alineado", "Axis aligned", "Eixo alinhado"),
                                                    tilt_deg,
                                                    rem_ov,
                                                    tr("tramo(s) con bloqueo virtual", "segment(s) with virtual blockout", "trecho(s) com bloqueio virtual")
                                                )
                                            });
                                        }
                                    }
                                }
                            });
                        });
                });
        }

        // ==================================================================
        // VENTANA FLOTANTE SUP. DERECHA: Transparencias
        // ==================================================================
        let right_x = (ctx.screen_rect().width() - 305.0).max(360.0);
        Window::new(tr("Transparencias (Modelo y Antagonista)", "Visibility (Model & Antagonist)", "Transparências (Modelo e Antagonista)"))
            .id(egui::Id::new("m3_card_trans"))
            .default_pos(egui::pos2(right_x, 74.0))
            .default_width(290.0)
            .collapsible(true)
            .resizable(false)
            .show(ctx, |ui| {
                ui.horizontal(|ui| {
                    ui.checkbox(&mut self.show_prep, tr("Modelo", "Model", "Modelo"));
                    ui.add(egui::Slider::new(&mut self.prep_opacity, 0.15..=1.0).show_value(false));
                    ui.label(RichText::new(format!("{:.0}%", self.prep_opacity * 100.0)).size(10.5).color(CREAM_MUTED));
                });
                ui.horizontal(|ui| {
                    ui.checkbox(&mut self.show_ant, tr("Antagonista", "Antagonist", "Antagonista"));
                    ui.add(egui::Slider::new(&mut self.ant_opacity, 0.10..=1.0).show_value(false));
                    ui.label(RichText::new(format!("{:.0}%", self.ant_opacity * 100.0)).size(10.5).color(CREAM_MUTED));
                });
            });

        // ==================================================================
        // BOTÓN FLOTANTE INFERIOR DERECHO: Continuar a Diseño de Corona
        // ==================================================================
        let can_proceed = if active_is_pontic_card {
            true
        } else {
            self.margin_ctrl.control_points.len() >= 4 && !self.margin_audit.blocks_design
        };
        Area::new(egui::Id::new("m3_next_bar"))
            .anchor(Align2::RIGHT_BOTTOM, egui::vec2(-14.0, -34.0))
            .order(Order::Foreground)
            .show(ctx, |ui| {
                Frame::NONE
                    .fill(egui::Color32::from_rgba_unmultiplied(0x21, 0x05, 0x2C, 240))
                    .stroke(Stroke::new(
                        1.5_f32,
                        if can_proceed {
                            if active_is_pontic_card || self.margin_audit.valid_for_cement {
                                EMERALD
                            } else {
                                Color32::from_rgb(0xF5, 0x9E, 0x0B)
                            }
                        } else {
                            Color32::from_rgb(0xFF, 0x45, 0x52)
                        },
                    ))
                    .corner_radius(CornerRadius::same(8))
                    .inner_margin(Margin::same(6))
                    .show(ui, |ui| {
                        let btn_resp = ui
                            .add_enabled_ui(can_proceed, |ui| {
                                theme::primary_button(ui, tr("Generar y Diseñar Restauración  →", "Generate & Design Restoration  →", "Gerar e Desenhar Restauração  →"))
                            })
                            .inner;
                        if btn_resp.clicked() {
                            if active_is_pontic_card {
                                self.persist_margin(case, case_manager);
                                action = MarginAction::ProceedToDesign;
                            } else {
                                self.refresh_audit(case, false);
                                if self.margin_ctrl.control_points.len() < 4 {
                                    action = MarginAction::StatusMessage(
                                        tr(
                                            "Traza al menos 4 puntos de margen o usa '★ Auto-Detectar Margen' primero.",
                                            "Trace at least 4 margin points or click '★ Auto-Detect Margin' first.",
                                            "Trace ao menos 4 pontos de término ou use '★ Auto-Detectar Término' primeiro.",
                                        ).to_string(),
                                    );
                                } else if self.margin_audit.blocks_design {
                                    action = MarginAction::StatusMessage(self.margin_audit.message.clone());
                                } else {
                                    if self.margin_audit.has_undercut_warning {
                                        let _ = self.auto_optimize_insertion_axis(case, case_manager);
                                    }
                                    self.persist_margin(case, case_manager);
                                    action = MarginAction::ProceedToDesign;
                                }
                            }
                        } else if !can_proceed && self.margin_ctrl.control_points.len() >= 4 {
                            ui.label(
                                RichText::new(tr("La curva está cruzada — pulsa 'Ordenar 360°'", "Curve is self-intersecting — click 'Sort 360°'", "A curva está cruzada — clique em 'Ordenar 360°'"))
                                    .size(10.0)
                                    .color(Color32::from_rgb(0xFF, 0x8A, 0x92)),
                            );
                        }
                    });
            });

        action
    }
}
