//! Módulo 4 (UI + Estudio CAD 3D Nativo): Transformación Individual, Morphing 3D,
//! Esculpido sin Conflictos, Conectores de Puente, Transparencias Individuales y Smart Tools.

use crate::core::case_manager::{
    BridgeConnectorConfig, CaseManager, DentalCase, DesignParameters, APEX_MATERIAL_PRESETS,
};
use crate::core::i18n::{self, tr};
use crate::core::library_manager::ToothLibraryManager;
use crate::geometry::connector::{
    build_connector_mesh, build_connector_waist_ring_3d, compute_connector_frame_with_colliders,
    compute_cross_section_area_mm2, compute_profile_dimensions_mm, control_node_positions_3d,
    ensure_pontic_closed, ConnectorFrame,
};
use crate::geometry::crown::{CrownDesigner, DieInterfaceAudit};
use crate::geometry::gizmo::{PlacementOverlay, TransformSubmode};
use crate::geometry::margin::{build_tube_mesh, build_uv_sphere};
use crate::geometry::mesh::{build_orthonormal_basis, MeshCollider, TriMeshData};
use crate::geometry::sculpt::{
    CrownSculptor, MorphSession, SculptMode, SculptStrokeCache, TransformSession,
};
use crate::geometry::smart_tools::SmartToolsEngine;
use crate::render::camera::OrbitCamera;
use crate::render::gl_renderer::{
    make_viewport_paint_callback, ColorMapKind, GlViewportRenderer, RenderLayerSpec,
};
use crate::theme::{self, AMBER, AMETHYST, AMETHYST_BRIGHT, CREAM, CREAM_MUTED, CYAN, EMERALD, TEAL};
use egui::{Align2, Area, Color32, CornerRadius, Frame, Margin, Order, Pos2, RichText, Sense, Stroke, Window};
use glam::{Quat, Vec3, Vec4};
use std::collections::BTreeMap;
use std::f32::consts::PI;
use std::sync::{Arc, Mutex};

pub enum DesignAction {
    None,
    ProceedToExport,
    ReturnToMargin,
    StatusMessage(String),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DesignToolMode {
    Transform, // Transformación Individual estilo 3Shape
    Morph,     // Morphing 3D interactivo estilo 3Shape
    Add,       // Aumentar cera sin conflictos
    Remove,    // Quitar / Tallar material
    Smooth,    // Suavizar topológico Taubin
    Connector, // Edición de Conectores de Puente
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CrownColorMode {
    Ceramic,
    OcclusalHeatmap,
    ThicknessHeatmap,
}

pub struct OtherToothState {
    pub mesh: TriMeshData,
    pub rev: u64,
    pub collider: Option<MeshCollider>,
    pub base_generated_crown: Option<TriMeshData>,
    pub positioned_source_mesh: Option<TriMeshData>,
    pub is_adapted_to_margin: bool,
}

pub struct ConnectorRuntime {
    pub tooth_a: i32,
    pub tooth_b: i32,
    pub frame: ConnectorFrame,
    pub mesh: TriMeshData,
    pub waist_ring: Vec<Vec3>,
    pub node_pts: [Vec3; 8],
    pub center_pt: Vec3,
    pub ring_a_3d: Vec<Vec3>,
    pub ring_a_nodes: [Vec3; 8],
    pub anchor_a: Vec3,
    pub ring_b_3d: Vec<Vec3>,
    pub ring_b_nodes: [Vec3; 8],
    pub anchor_b: Vec3,
    pub area_mm2: f32,
    pub min_area_mm2: f32,
    pub height_mm: f32,
    pub width_mm: f32,
    pub rev: u64,
}

#[derive(Clone)]
struct DesignUndoSnapshot {
    crown_mesh: TriMeshData,
    base_crown: Option<TriMeshData>,
    positioned_source: Option<TriMeshData>,
    is_adapted: bool,
    params: DesignParameters,
}

pub struct DesignPanelState {
    pub camera: OrbitCamera,
    pub next_rev: u64,

    pub prep_mesh: Option<(TriMeshData, u64)>,
    pub ant_mesh: Option<(TriMeshData, u64)>,
    pub crown_mesh: Option<(TriMeshData, u64)>,
    pub crown_collider: Option<MeshCollider>,
    pub base_generated_crown: Option<TriMeshData>,
    /// Malla del diente libre exactamente en la posición, rotación, escala y esculpido
    /// definidos por el usuario antes de sellar al margen (jamás se resetea al cambiar herramienta).
    pub positioned_source_mesh: Option<TriMeshData>,
    pub cement_spacer_mesh: Option<(TriMeshData, u64)>,
    pub min_thickness_mesh: Option<(TriMeshData, u64)>,
    pub other_restorations: BTreeMap<i32, OtherToothState>,
    pub custom_override_mesh: Option<TriMeshData>,
    pub custom_stl_label: String,

    // Conectores de puente en tiempo real (estilo Exocad)
    pub connectors: Vec<ConnectorRuntime>,
    pub active_connector_idx: usize,
    pub conn_overlay_rev: u64,
    pub hovered_conn_node: Option<(usize, usize)>,
    pub active_conn_node: Option<(usize, usize)>,
    pub drag_conn_start_cfg: Option<BridgeConnectorConfig>,
    pub design_dirty_for_bridge: bool,

    pub is_adapted_to_margin: bool,
    pub tool_mode: DesignToolMode,
    pub transform_submode: TransformSubmode,
    pub show_transform_gizmo: bool,

    // Controles deslizantes en mm (estilo barra horizontal)
    pub brush_radius_mm: f32,
    pub brush_strength_mm: f32,
    pub morph_radius_mm: f32,
    pub morph_normal_only: bool,

    // Smart Tools sliders (mm)
    pub smart_occ_clearance_mm: f32,
    pub smart_prox_clearance_mm: f32,
    pub smart_emergence_mm: f32,
    pub smart_gingival_offset_mm: f32,

    // Capas de transparencia (esquina superior derecha + individuales por pieza FDI)
    pub vis_cement: bool,
    pub op_cement: f32,
    pub vis_min_thick: bool,
    pub op_min_thick: f32,
    pub vis_restorations: bool,
    pub op_restorations: f32,
    pub unit_visible: BTreeMap<i32, bool>,
    pub unit_opacity: BTreeMap<i32, f32>,
    pub vis_connectors: bool,
    pub op_connectors: f32,
    pub vis_model: bool,
    pub op_model: f32,
    pub vis_antagonist: bool,
    pub op_antagonist: f32,
    pub margin_line_radius_mm: f32,
    pub margin_vis_rev: u64,
    pub default_op_restorations: f32,
    pub default_op_model: f32,
    pub default_vis_antagonist: bool,
    pub default_vis_cement: bool,
    pub default_op_cement: f32,
    pub default_vis_min_thick: bool,
    pub default_op_min_thick: f32,

    pub color_mode: CrownColorMode,
    pub cross_section: bool,
    pub audit_text: String,
    pub die_audit: DieInterfaceAudit,

    // Continuidad de sesión Etapa 4 <-> Etapa 5 (jamás reinicia al volver de Exportar)
    pub loaded_case_id: Option<String>,
    pub loaded_tooth_fdi: Option<i32>,
    pub loaded_margin_sig: u64,
    pub loaded_order_sig: u64,

    // Interacción 3D: Flechas Direccionales Fijas estilo 3Shape (en plano de vista)
    pub hovered_handle: Option<String>,
    pub hovered_label: String,
    pub active_handle: Option<String>,
    pub drag_locked_rect: Option<egui::Rect>,
    pub drag_start_pos: Pos2,
    pub drag_origin_3d: Vec3,
    pub drag_start_angle: f32,
    pub active_rot_angle_deg: Option<f32>,
    pub drag_cam_right: Vec3,
    pub drag_cam_up: Vec3,
    pub drag_cam_fwd: Vec3,
    pub drag_mm_per_px: f32,
    pub drag_tooth_spans_cam: egui::Vec2,
    pub drag_start_crown: Option<TriMeshData>,
    pub transform_session: Option<TransformSession>,

    // Interacción 3D: Morphing 3D
    pub morph_session: Option<MorphSession>,
    pub morph_cursor_3d: Option<Vec3>,

    // Interacción 3D: Esculpido libre
    pub is_sculpting: bool,
    pub last_sculpt_pt: Option<Vec3>,
    pub stroke_adjacency: Option<Vec<Vec<usize>>>,
    pub sculpt_cache: Option<SculptStrokeCache>,
    pub sculpt_dab_count: usize,
    pub hover_cursor_3d: Option<Vec3>,
    pub cursor_rev: u64,

    // Deshacer (Ctrl+Z)
    undo_stack: Vec<DesignUndoSnapshot>,
}

impl Default for DesignPanelState {
    fn default() -> Self {
        Self {
            camera: OrbitCamera::default(),
            next_rev: 1,
            prep_mesh: None,
            ant_mesh: None,
            crown_mesh: None,
            crown_collider: None,
            base_generated_crown: None,
            positioned_source_mesh: None,
            cement_spacer_mesh: None,
            min_thickness_mesh: None,
            other_restorations: BTreeMap::new(),
            custom_override_mesh: None,
            custom_stl_label: "Posición aproximada lista para Transformación Individual".to_string(),
            connectors: Vec::new(),
            active_connector_idx: 0,
            conn_overlay_rev: 1,
            hovered_conn_node: None,
            active_conn_node: None,
            drag_conn_start_cfg: None,
            design_dirty_for_bridge: false,
            is_adapted_to_margin: false,
            tool_mode: DesignToolMode::Transform,
            transform_submode: TransformSubmode::All,
            show_transform_gizmo: true,
            brush_radius_mm: 1.45,
            brush_strength_mm: 0.14,
            morph_radius_mm: 2.40,
            morph_normal_only: false,
            smart_occ_clearance_mm: 0.00,
            smart_prox_clearance_mm: -0.02,
            smart_emergence_mm: 0.12,
            smart_gingival_offset_mm: 0.00,
            vis_cement: true,
            op_cement: 0.55,
            vis_min_thick: true,
            op_min_thick: 0.45,
            vis_restorations: true,
            op_restorations: 1.00,
            unit_visible: BTreeMap::new(),
            unit_opacity: BTreeMap::new(),
            vis_connectors: true,
            op_connectors: 1.00,
            vis_model: true,
            op_model: 0.80,
            vis_antagonist: false,
            op_antagonist: 0.40,
            margin_line_radius_mm: 0.028,
            margin_vis_rev: 1,
            default_op_restorations: 1.00,
            default_op_model: 0.80,
            default_vis_antagonist: false,
            default_vis_cement: true,
            default_op_cement: 0.55,
            default_vis_min_thick: true,
            default_op_min_thick: 0.45,
            color_mode: CrownColorMode::Ceramic,
            cross_section: false,
            audit_text: "Estado: Transformación Individual Activa".to_string(),
            die_audit: DieInterfaceAudit::default(),
            loaded_case_id: None,
            loaded_tooth_fdi: None,
            loaded_margin_sig: 0,
            loaded_order_sig: 0,
            hovered_handle: None,
            hovered_label: String::new(),
            active_handle: None,
            drag_locked_rect: None,
            drag_start_pos: Pos2::ZERO,
            drag_origin_3d: Vec3::ZERO,
            drag_start_angle: 0.0,
            active_rot_angle_deg: None,
            drag_cam_right: Vec3::X,
            drag_cam_up: Vec3::Y,
            drag_cam_fwd: Vec3::NEG_Z,
            drag_mm_per_px: 0.032,
            drag_tooth_spans_cam: egui::vec2(8.0, 8.0),
            drag_start_crown: None,
            transform_session: None,
            morph_session: None,
            morph_cursor_3d: None,
            is_sculpting: false,
            last_sculpt_pt: None,
            stroke_adjacency: None,
            sculpt_cache: None,
            sculpt_dab_count: 0,
            hover_cursor_3d: None,
            cursor_rev: 1,
            undo_stack: Vec::new(),
        }
    }
}

impl DesignPanelState {
    fn bump(&mut self) -> u64 {
        self.next_rev += 1;
        self.next_rev
    }

    fn unit_arch(fdi: i32) -> &'static str {
        if matches!(fdi / 10, 1 | 2) {
            "maxilla"
        } else {
            "mandible"
        }
    }

    pub fn is_unit_visible(&self, fdi: i32) -> bool {
        self.vis_restorations && self.unit_visible.get(&fdi).copied().unwrap_or(true)
    }

    fn effective_unit_opacity(&self, fdi: i32) -> f32 {
        (self.op_restorations * self.unit_opacity.get(&fdi).copied().unwrap_or(1.0)).clamp(0.05, 1.0)
    }

    fn set_crown_mesh(&mut self, mesh: TriMeshData, _axis: Vec3) {
        self.crown_collider = MeshCollider::from_mesh(&mesh);
        let rev = self.bump();
        self.crown_mesh = Some((mesh, rev));
    }

    fn set_crown_mesh_fast(&mut self, mesh: TriMeshData) {
        let rev = self.bump();
        self.crown_mesh = Some((mesh, rev));
    }

    fn refresh_crown_collider(&mut self) {
        self.crown_collider = self
            .crown_mesh
            .as_ref()
            .and_then(|(m, _)| MeshCollider::from_mesh(m));
    }

    fn margin_vec3(case: &DentalCase) -> Vec<Vec3> {
        case.margin_points.iter().map(|p| Vec3::from_array(*p)).collect()
    }

    fn compute_margin_sig(case: &DentalCase) -> u64 {
        let mut h: u64 = 14695981039346656037;
        for pt in &case.margin_points {
            for &c in pt {
                h ^= c.to_bits() as u64;
                h = h.wrapping_mul(1099511628211);
            }
        }
        h ^ (case.margin_points.len() as u64)
    }

    fn compute_order_sig(case: &DentalCase) -> u64 {
        let mut h: u64 = 14695981039346656037;
        for (&fdi, u) in &case.restorations {
            h ^= fdi as u64;
            h = h.wrapping_mul(1099511628211);
            for b in u.process_type.as_bytes() {
                h ^= *b as u64;
                h = h.wrapping_mul(1099511628211);
            }
            for &c in &u.connected_to {
                h ^= (c as u64) << 8;
                h = h.wrapping_mul(1099511628211);
            }
            for pt in &u.margin_points {
                for &v in pt {
                    h ^= v.to_bits() as u64;
                    h = h.wrapping_mul(1099511628211);
                }
            }
        }
        h
    }

    /// Reconstruye la máscara `is_external` al recargar un STL guardado en disco para que
    /// el intaglio y el borde marginal sigan bloqueados al esculpir o hacer Morphing.
    fn classify_external_vertices(crown: &mut TriMeshData, prep: Option<&TriMeshData>, margin: &[Vec3]) {
        let n = crown.positions.len();
        if n == 0 {
            return;
        }
        let Some(prep_m) = prep else {
            crown.is_external = vec![true; n];
            return;
        };
        let Some(prep_col) = MeshCollider::from_mesh(prep_m) else {
            crown.is_external = vec![true; n];
            return;
        };
        let mut is_ext = vec![true; n];
        let mut thick = vec![1.2_f32; n];
        for (i, &p) in crown.positions.iter().enumerate() {
            let (_, d_prep) = prep_col.closest_point_and_dist(p);
            thick[i] = d_prep;
            let near_margin = margin.iter().any(|&m| m.distance_squared(p) < 0.14 * 0.14);
            if near_margin || d_prep < 0.24 {
                is_ext[i] = false;
            }
        }
        let ext_cnt = is_ext.iter().filter(|&&b| b).count();
        if ext_cnt < n / 8 {
            crown.is_external = vec![true; n];
        } else {
            crown.is_external = is_ext;
        }
        crown.thickness_mm = thick;
    }

    pub fn apply_settings(&mut self, settings: &crate::core::settings::AppSettings) {
        let new_r = (settings.margin_line_radius_mm * 0.90).clamp(0.015, 0.120);
        if (self.margin_line_radius_mm - new_r).abs() > 1e-4 {
            self.margin_line_radius_mm = new_r;
            self.margin_vis_rev = self.bump();
        }
        self.default_op_restorations = settings.stage4_restoration_opacity;
        self.default_op_model = settings.stage4_model_opacity;
        self.default_vis_antagonist = settings.stage4_show_antagonist;
        self.default_vis_cement = settings.stage4_show_cement;
        self.default_op_cement = settings.stage4_cement_opacity;
        self.default_vis_min_thick = settings.stage4_show_min_thickness;
        self.default_op_min_thick = settings.stage4_min_thickness_opacity;
        self.camera.rotate_sensitivity = settings.camera_rotate_sensitivity;
        self.camera.zoom_sensitivity = settings.camera_zoom_sensitivity;
        self.camera.invert_zoom = settings.camera_invert_zoom;
    }

    pub fn load_case(
        &mut self,
        case: &mut DentalCase,
        case_manager: &CaseManager,
        library_manager: &ToothLibraryManager,
    ) {
        case.ensure_restorations_initialized();
        let sig = Self::compute_margin_sig(case);
        let order_sig = Self::compute_order_sig(case);
        let same_session = self.loaded_case_id.as_deref() == Some(case.case_id.as_str())
            && self.loaded_tooth_fdi == Some(case.tooth_number)
            && self.loaded_order_sig == order_sig
            && self.crown_mesh.is_some();

        if same_session {
            if self.loaded_margin_sig != sig {
                self.loaded_margin_sig = sig;
                self.rebuild_auxiliary_shells(case);
            }
            self.rebuild_connectors(case);
            return;
        }

        self.loaded_case_id = Some(case.case_id.clone());
        self.loaded_tooth_fdi = Some(case.tooth_number);
        self.loaded_margin_sig = sig;
        self.loaded_order_sig = order_sig;
        self.custom_override_mesh = None;
        self.transform_session = None;
        self.sculpt_cache = None;
        self.sculpt_dab_count = 0;
        self.undo_stack.clear();
        self.tool_mode = DesignToolMode::Transform;

        self.vis_restorations = true;
        self.op_restorations = self.default_op_restorations;
        self.vis_connectors = true;
        self.op_connectors = 1.00;
        self.vis_model = true;
        self.op_model = self.default_op_model;
        self.vis_antagonist = self.default_vis_antagonist;
        self.vis_cement = self.default_vis_cement;
        self.op_cement = self.default_op_cement;
        self.vis_min_thick = self.default_vis_min_thick;
        self.op_min_thick = self.default_op_min_thick;

        for &fdi in case.restorations.keys() {
            self.unit_visible.entry(fdi).or_insert(true);
            self.unit_opacity.entry(fdi).or_insert(self.default_op_restorations);
        }

        self.load_all_restorations(case, case_manager, library_manager);

        if let Some((crown, _)) = &self.crown_mesh {
            let (mn, mx) = crown.bounds();
            self.camera.frame_bounds(mn - Vec3::splat(4.0), mx + Vec3::splat(4.0));
        } else if let Some((prep, _)) = &self.prep_mesh {
            let (mn, mx) = prep.bounds();
            self.camera.frame_bounds(mn, mx);
        }
    }

    /// Calcula una malla inicial de póntico posicionada exactamente en el espacio edéntulo entre sus pilares.
    fn place_pontic_world(
        &self,
        fdi: i32,
        case: &DentalCase,
        library_manager: &ToothLibraryManager,
        placed_cache: &BTreeMap<i32, TriMeshData>,
        same_arch_prep: Option<&TriMeshData>,
        same_arch_ant: Option<&TriMeshData>,
    ) -> TriMeshData {
        let unit = case.restorations.get(&fdi).cloned().unwrap_or_default();
        let lib_tooth = library_manager.load_tooth_mesh(&unit.design_params.library_id, fdi);
        let ax = Vec3::from_array(unit.insertion_axis);
        let (u, v, _) = build_orthonormal_basis(ax);

        // Buscar centros de pilares conectados o vecinos en el mismo arco
        let mut neigh_pts: Vec<Vec3> = Vec::new();
        for &nb in &unit.connected_to {
            if let Some(m) = placed_cache.get(&nb) {
                neigh_pts.push(m.center());
            } else if let Some(nu) = case.restorations.get(&nb) {
                if !nu.margin_points.is_empty() {
                    let c = nu.margin_points.iter().map(|p| Vec3::from_array(*p)).sum::<Vec3>()
                        / nu.margin_points.len() as f32;
                    neigh_pts.push(c);
                }
            }
        }
        if neigh_pts.is_empty() {
            for (&ofdi, m) in placed_cache {
                if ofdi != fdi && Self::unit_arch(ofdi) == Self::unit_arch(fdi) {
                    neigh_pts.push(m.center());
                }
            }
        }
        let origin = if !neigh_pts.is_empty() {
            let c = neigh_pts.iter().copied().sum::<Vec3>() / neigh_pts.len() as f32;
            // Ajustar altura cervical cerca del margen de vecinos o superficie de reborde
            if let Some(prep) = same_arch_prep {
                if let Some(col) = MeshCollider::from_mesh(prep) {
                    col.project_point(c)
                } else {
                    c
                }
            } else {
                c
            }
        } else {
            same_arch_prep.map(|m| m.center()).unwrap_or(Vec3::ZERO)
        };

        let synthetic_ring: Vec<Vec3> = (0..16)
            .map(|i| {
                let ang = 2.0 * PI * (i as f32) / 16.0;
                origin + u * (4.1 * ang.cos()) + v * (3.7 * ang.sin())
            })
            .collect();

        let placed = CrownDesigner::place_library_tooth_world(
            &lib_tooth,
            &synthetic_ring,
            ax,
            &unit.design_params,
            same_arch_prep,
            same_arch_ant,
        );
        ensure_pontic_closed(&placed)
    }

    fn load_arches_for_active(&mut self, case: &DentalCase, case_manager: &CaseManager) {
        if let Some(p) = case_manager.get_active_scan_path(case, &case.prep_arch) {
            if let Ok(m) = TriMeshData::load_from_path(&p) {
                let rev = self.bump();
                self.prep_mesh = Some((m, rev));
            }
        } else {
            self.prep_mesh = None;
        }

        if let Some(p) = case_manager.get_active_scan_path(case, case.get_antagonist_arch()) {
            if let Ok(m) = TriMeshData::load_from_path(&p) {
                let rev = self.bump();
                self.ant_mesh = Some((m, rev));
            }
        } else {
            self.ant_mesh = None;
        }
    }

    /// Carga simultáneamente TODAS las restauraciones de la orden (coronas, pilares, carillas y pónticos)
    /// en el visor OpenGL del Módulo 4 y construye sus conectores activos.
    pub fn load_all_restorations(
        &mut self,
        case: &mut DentalCase,
        case_manager: &CaseManager,
        library_manager: &ToothLibraryManager,
    ) {
        self.loaded_case_id = Some(case.case_id.clone());
        self.loaded_tooth_fdi = Some(case.tooth_number);
        self.loaded_margin_sig = Self::compute_margin_sig(case);
        self.loaded_order_sig = Self::compute_order_sig(case);
        self.smart_occ_clearance_mm = case.design_params.occlusal_contact_offset_mm;
        self.smart_prox_clearance_mm = case.design_params.proximal_contact_offset_mm;

        self.load_arches_for_active(case, case_manager);

        let mand_mesh = case_manager
            .get_active_scan_path(case, "mandible")
            .and_then(|p| TriMeshData::load_from_path(&p).ok());
        let max_mesh = case_manager
            .get_active_scan_path(case, "maxilla")
            .and_then(|p| TriMeshData::load_from_path(&p).ok());

        let case_dir = case_manager.get_case_dir(&case.case_id);
        let mut built_map: BTreeMap<i32, (TriMeshData, Option<TriMeshData>, Option<TriMeshData>, bool)> =
            BTreeMap::new();

        // Pasada 1: Cargar STLs guardados en disco y colocar primero todos los pilares / individuales
        let fdi_order: Vec<i32> = {
            let mut non_pontics: Vec<i32> = case
                .restorations
                .iter()
                .filter(|(_, u)| !u.is_pontic())
                .map(|(&f, _)| f)
                .collect();
            let pontics: Vec<i32> = case
                .restorations
                .iter()
                .filter(|(_, u)| u.is_pontic())
                .map(|(&f, _)| f)
                .collect();
            non_pontics.extend(pontics);
            non_pontics
        };

        for fdi in fdi_order {
            let Some(unit) = case.restorations.get(&fdi).cloned() else {
                continue;
            };
            self.unit_visible.entry(fdi).or_insert(true);
            self.unit_opacity.entry(fdi).or_insert(self.default_op_restorations);

            let (same_prep, same_ant) = if Self::unit_arch(fdi) == "maxilla" {
                (max_mesh.as_ref(), mand_mesh.as_ref())
            } else {
                (mand_mesh.as_ref(), max_mesh.as_ref())
            };

            let disk_path = unit
                .crown_design_path
                .as_ref()
                .map(|rel| case_dir.join(rel))
                .filter(|p| p.exists());

            if let Some(cpath) = disk_path {
                if let Ok(mut loaded) = TriMeshData::load_from_path(&cpath) {
                    if unit.is_pontic() {
                        loaded = ensure_pontic_closed(&loaded);
                    } else {
                        let um: Vec<Vec3> = unit.margin_points.iter().map(|p| Vec3::from_array(*p)).collect();
                        Self::classify_external_vertices(&mut loaded, same_prep, &um);
                    }
                    built_map.insert(fdi, (loaded.clone(), Some(loaded.clone()), Some(loaded), true));
                    continue;
                }
            }

            if unit.is_pontic() {
                let placed_only: BTreeMap<i32, TriMeshData> =
                    built_map.iter().map(|(&k, v)| (k, v.0.clone())).collect();
                let pontic_m = self.place_pontic_world(
                    fdi,
                    case,
                    library_manager,
                    &placed_only,
                    same_prep,
                    same_ant,
                );
                // Guardar póntico inicial en disco para que Etapa 5 lo tenga listo
                let unit_rel = format!("design/crown_{}_designed.stl", fdi);
                let _ = pontic_m.save_stl_binary(&case_dir.join(&unit_rel));
                if let Some(u) = case.restorations.get_mut(&fdi) {
                    u.crown_design_path = Some(unit_rel);
                }
                built_map.insert(fdi, (pontic_m.clone(), Some(pontic_m.clone()), Some(pontic_m), true));
            } else {
                let lib_tooth = if fdi == case.tooth_number {
                    if let Some(ov) = &self.custom_override_mesh {
                        ov.clone()
                    } else {
                        library_manager.load_tooth_mesh(&unit.design_params.library_id, fdi)
                    }
                } else {
                    library_manager.load_tooth_mesh(&unit.design_params.library_id, fdi)
                };
                let um: Vec<Vec3> = unit.margin_points.iter().map(|p| Vec3::from_array(*p)).collect();
                let uax = Vec3::from_array(unit.insertion_axis);
                let placed = CrownDesigner::place_library_tooth_world(
                    &lib_tooth,
                    &um,
                    uax,
                    &unit.design_params,
                    same_prep,
                    same_ant,
                );
                built_map.insert(fdi, (placed.clone(), Some(placed.clone()), Some(placed), false));
            }
        }

        self.other_restorations.clear();
        self.crown_mesh = None;
        self.crown_collider = None;
        self.base_generated_crown = None;
        self.positioned_source_mesh = None;

        for (fdi, (mesh, base_c, src_c, adapted)) in built_map {
            if fdi == case.tooth_number {
                self.base_generated_crown = base_c;
                self.positioned_source_mesh = src_c;
                self.is_adapted_to_margin = adapted;
                let ax = Vec3::from_array(case.insertion_axis);
                self.set_crown_mesh(mesh.clone(), ax);
                let audit = mesh.inspect_cam();
                self.audit_text = if adapted {
                    format!(
                        "{} #{}: ✔ {}\nDim: {:.1}×{:.1}×{:.1} mm | Vol: {:.1} mm³",
                        tr("Pieza", "Unit", "Dente"),
                        fdi,
                        tr("Lista", "Ready", "Pronto"),
                        audit.dims_mm.x, audit.dims_mm.y, audit.dims_mm.z, audit.volume_mm3
                    )
                } else {
                    format!(
                        "{} #{}: {}\n{}",
                        tr("Pieza", "Unit", "Dente"),
                        fdi,
                        tr("Colocación Libre (Gizmo)", "Free Placement (Gizmo)", "Posicionamento Livre (Gizmo)"),
                        tr("Posiciona y pulsa '★ Adaptar al Margen'.", "Position and click '★ Adapt to Margin'.", "Posicione e clique em '★ Adaptar ao Término'.")
                    )
                };
            } else {
                let col = MeshCollider::from_mesh(&mesh);
                let rev = self.bump();
                self.other_restorations.insert(
                    fdi,
                    OtherToothState {
                        mesh,
                        rev,
                        collider: col,
                        base_generated_crown: base_c,
                        positioned_source_mesh: src_c,
                        is_adapted_to_margin: adapted,
                    },
                );
            }
        }

        self.rebuild_auxiliary_shells(case);
        self.rebuild_connectors(case);
        self.design_dirty_for_bridge = false;
        let _ = case_manager.save_case(case);
    }

    /// Cambia la pieza activa al clic sobre otro diente en 3D, fila del menú de transparencias o selector,
    /// conservando intacta la posición, esculpido y transparencia de todas las piezas en pantalla.
    pub fn switch_active_tooth(
        &mut self,
        target_fdi: i32,
        case: &mut DentalCase,
        case_manager: &CaseManager,
        library_manager: &ToothLibraryManager,
    ) {
        if target_fdi == case.tooth_number {
            return;
        }
        self.active_handle = None;
        self.drag_start_crown = None;
        self.transform_session = None;
        self.drag_locked_rect = None;
        self.active_rot_angle_deg = None;
        self.morph_session = None;
        self.is_sculpting = false;
        self.last_sculpt_pt = None;
        self.stroke_adjacency = None;
        self.sculpt_cache = None;
        self.sculpt_dab_count = 0;
        self.hovered_handle = None;
        self.hover_cursor_3d = None;
        self.undo_stack.clear();

        // Guardar pieza saliente en `other_restorations` y en disco si ya fue adaptada / póntico
        let old_fdi = case.tooth_number;
        if let Some((old_mesh, _)) = self.crown_mesh.take() {
            if self.is_adapted_to_margin {
                let case_dir = case_manager.get_case_dir(&case.case_id);
                let unit_rel = format!("design/crown_{}_designed.stl", old_fdi);
                let _ = old_mesh.save_stl_binary(&case_dir.join(&unit_rel));
                if let Some(u) = case.restorations.get_mut(&old_fdi) {
                    u.crown_design_path = Some(unit_rel);
                    u.design_params = case.design_params.clone();
                }
            }
            let rev = self.bump();
            let col = MeshCollider::from_mesh(&old_mesh);
            self.other_restorations.insert(
                old_fdi,
                OtherToothState {
                    mesh: old_mesh,
                    rev,
                    collider: col,
                    base_generated_crown: self.base_generated_crown.take(),
                    positioned_source_mesh: self.positioned_source_mesh.take(),
                    is_adapted_to_margin: self.is_adapted_to_margin,
                },
            );
        }

        let prev_arch = case.prep_arch.clone();
        case.set_active_tooth(target_fdi);
        if case.prep_arch != prev_arch {
            self.load_arches_for_active(case, case_manager);
        }
        self.loaded_tooth_fdi = Some(case.tooth_number);
        self.loaded_margin_sig = Self::compute_margin_sig(case);
        self.smart_occ_clearance_mm = case.design_params.occlusal_contact_offset_mm;
        self.smart_prox_clearance_mm = case.design_params.proximal_contact_offset_mm;

        if let Some(st) = self.other_restorations.remove(&target_fdi) {
            self.base_generated_crown = st.base_generated_crown;
            self.positioned_source_mesh = st.positioned_source_mesh;
            self.is_adapted_to_margin = st.is_adapted_to_margin;
            let ax = Vec3::from_array(case.insertion_axis);
            self.set_crown_mesh(st.mesh.clone(), ax);
            let audit = st.mesh.inspect_cam();
            self.audit_text = if self.is_adapted_to_margin {
                format!(
                    "{} #{}: ✔ {}\nDim: {:.1}×{:.1}×{:.1} mm | Vol: {:.1} mm³",
                    tr("Pieza", "Unit", "Dente"),
                    target_fdi,
                    tr("Activa", "Active", "Ativo"),
                    audit.dims_mm.x, audit.dims_mm.y, audit.dims_mm.z, audit.volume_mm3
                )
            } else {
                format!(
                    "{} #{}: {}\n{}",
                    tr("Pieza", "Unit", "Dente"),
                    target_fdi,
                    tr("Colocación Libre (Gizmo)", "Free Placement (Gizmo)", "Posicionamento Livre (Gizmo)"),
                    tr("Posiciona y pulsa '★ Adaptar al Margen'.", "Position and click '★ Adapt to Margin'.", "Posicione e clique em '★ Adaptar ao Término'.")
                )
            };
        } else {
            self.place_approximate_tooth(case, library_manager, false);
        }

        self.rebuild_auxiliary_shells(case);
        self.rebuild_connectors(case);
        self.design_dirty_for_bridge = false;
        let _ = case_manager.save_case(case);
    }

    pub fn save_all_restorations(&mut self, case: &mut DentalCase, case_manager: &CaseManager) {
        self.save_active_crown(case, case_manager);
        let case_dir = case_manager.get_case_dir(&case.case_id);
        for (&fdi, st) in &self.other_restorations {
            let is_p = case.restorations.get(&fdi).map(|u| u.is_pontic()).unwrap_or(false);
            if st.is_adapted_to_margin || is_p {
                let unit_rel = format!("design/crown_{}_designed.stl", fdi);
                let _ = st.mesh.save_stl_binary(&case_dir.join(&unit_rel));
                if let Some(u) = case.restorations.get_mut(&fdi) {
                    u.crown_design_path = Some(unit_rel);
                }
            }
        }
        if self.design_dirty_for_bridge {
            let design_dir = case_dir.join("design");
            if let Ok(entries) = std::fs::read_dir(&design_dir) {
                for ent in entries.flatten() {
                    let p = ent.path();
                    if let Some(name) = p.file_name().and_then(|s| s.to_str()) {
                        if name.starts_with("bridge_") && name.ends_with("_final.stl") {
                            let _ = std::fs::remove_file(p);
                        }
                    }
                }
            }
            self.design_dirty_for_bridge = false;
        }
        let _ = case_manager.save_case(case);
    }

    pub fn rebuild_connectors(&mut self, case: &mut DentalCase) {
        case.sync_bridge_connectors();
        self.connectors.clear();
        let active_fdi = case.tooth_number;
        let ax = Vec3::from_array(case.insertion_axis);

        let get_mesh_and_col = |fdi: i32| -> (Option<&TriMeshData>, Option<&MeshCollider>) {
            if fdi == active_fdi {
                (
                    self.crown_mesh.as_ref().map(|(m, _)| m),
                    self.crown_collider.as_ref(),
                )
            } else {
                (
                    self.other_restorations.get(&fdi).map(|s| &s.mesh),
                    self.other_restorations.get(&fdi).and_then(|s| s.collider.as_ref()),
                )
            }
        };

        // Snapshot rápido de mallas reutilizando BVHs en memoria (cero reconstrucción por fotograma)
        let mut temp = Vec::with_capacity(case.bridge_connectors.len());
        for cfg in &case.bridge_connectors {
            if let ((Some(ma), col_a), (Some(mb), col_b)) =
                (get_mesh_and_col(cfg.tooth_a), get_mesh_and_col(cfg.tooth_b))
            {
                let frame = compute_connector_frame_with_colliders(ma, col_a, mb, col_b, ax, cfg);
                let mesh = build_connector_mesh(&frame, cfg);
                let waist_ring = build_connector_waist_ring_3d(&frame, cfg);
                let (node_pts, center_pt) = control_node_positions_3d(&frame, cfg);
                let area_mm2 = compute_cross_section_area_mm2(cfg);
                let (height_mm, width_mm) = compute_profile_dimensions_mm(cfg);
                let min_a = case
                    .restorations
                    .get(&cfg.tooth_a)
                    .map(|u| u.design_params.connector_area_mm2)
                    .unwrap_or(9.0)
                    .max(
                        case.restorations
                            .get(&cfg.tooth_b)
                            .map(|u| u.design_params.connector_area_mm2)
                            .unwrap_or(9.0),
                    );
                temp.push((
                    cfg.tooth_a,
                    cfg.tooth_b,
                    frame,
                    mesh,
                    waist_ring,
                    node_pts,
                    center_pt,
                    area_mm2,
                    min_a,
                    height_mm,
                    width_mm,
                ));
            }
        }

        for (tooth_a, tooth_b, frame, mesh, waist_ring, node_pts, center_pt, area_mm2, min_area_mm2, height_mm, width_mm) in temp {
            let rev = self.bump();
            let ring_a_3d = frame.ring_a_3d.clone();
            let ring_a_nodes = frame.ring_a_nodes;
            let anchor_a = frame.anchor_a;
            let ring_b_3d = frame.ring_b_3d.clone();
            let ring_b_nodes = frame.ring_b_nodes;
            let anchor_b = frame.anchor_b;
            self.connectors.push(ConnectorRuntime {
                tooth_a,
                tooth_b,
                frame,
                mesh,
                waist_ring,
                node_pts,
                center_pt,
                ring_a_3d,
                ring_a_nodes,
                anchor_a,
                ring_b_3d,
                ring_b_nodes,
                anchor_b,
                area_mm2,
                min_area_mm2,
                height_mm,
                width_mm,
                rev,
            });
        }
        if self.active_connector_idx >= self.connectors.len() {
            self.active_connector_idx = 0;
        }
        self.conn_overlay_rev = self.bump();
    }

    fn push_undo(&mut self, case: &DentalCase) {
        self.design_dirty_for_bridge = true;
        if let Some((crown, _)) = &self.crown_mesh {
            self.undo_stack.push(DesignUndoSnapshot {
                crown_mesh: crown.clone(),
                base_crown: self.base_generated_crown.clone(),
                positioned_source: self.positioned_source_mesh.clone(),
                is_adapted: self.is_adapted_to_margin,
                params: case.design_params.clone(),
            });
            if self.undo_stack.len() > 25 {
                self.undo_stack.remove(0);
            }
        }
    }

    pub fn undo(&mut self, case: &mut DentalCase, case_manager: &CaseManager) -> bool {
        let Some(snap) = self.undo_stack.pop() else {
            return false;
        };
        case.design_params = snap.params;
        self.is_adapted_to_margin = snap.is_adapted;
        self.base_generated_crown = snap.base_crown;
        self.positioned_source_mesh = snap.positioned_source;
        let ax = Vec3::from_array(case.insertion_axis);
        self.set_crown_mesh(snap.crown_mesh, ax);
        self.rebuild_auxiliary_shells(case);
        self.rebuild_connectors(case);
        self.design_dirty_for_bridge = true;
        if self.is_adapted_to_margin {
            self.save_active_crown(case, case_manager);
        } else {
            let _ = case_manager.save_case(case);
            self.audit_text = format!(
                "{} #{}: {}",
                tr("Pieza", "Unit", "Dente"),
                case.tooth_number,
                tr("Colocación Libre (Gizmo, sin sellar aún)", "Free Placement (Gizmo, not adapted yet)", "Posicionamento Livre (Gizmo, ainda não adaptado)")
            );
        }
        true
    }

    pub fn rebuild_auxiliary_shells(&mut self, case: &DentalCase) {
        if case.get_active_unit().map(|u| u.is_pontic()).unwrap_or(false) {
            self.cement_spacer_mesh = None;
            self.min_thickness_mesh = None;
            self.die_audit = DieInterfaceAudit {
                valid_for_production: true,
                ..Default::default()
            };
            return;
        }
        let margin = Self::margin_vec3(case);
        let ax = Vec3::from_array(case.insertion_axis);
        if let Some((prep, _)) = &self.prep_mesh {
            if margin.len() >= 8 {
                if let Some((cem, min_t, audit)) = CrownDesigner::generate_auxiliary_shells(
                    prep,
                    &margin,
                    ax,
                    &case.design_params,
                ) {
                    let r1 = self.bump();
                    let r2 = self.bump();
                    self.cement_spacer_mesh = Some((cem, r1));
                    self.min_thickness_mesh = Some((min_t, r2));
                    self.die_audit = audit;
                }
            } else {
                self.cement_spacer_mesh = None;
                self.min_thickness_mesh = None;
                self.die_audit = DieInterfaceAudit {
                    valid_for_production: false,
                    warning_message: format!(
                        "{} #{} {}",
                        tr("Falta trazar la línea de margen para la pieza", "Margin line missing for unit", "Falta traçar a linha de término para o dente"),
                        case.tooth_number,
                        tr("en Etapa 3.", "in Stage 3.", "na Etapa 3.")
                    ),
                    ..Default::default()
                };
            }
        }
    }

    pub fn place_approximate_tooth(
        &mut self,
        case: &mut DentalCase,
        library_manager: &ToothLibraryManager,
        record_undo: bool,
    ) {
        if record_undo {
            self.push_undo(case);
        }
        let is_pontic = case.get_active_unit().map(|u| u.is_pontic()).unwrap_or(false);
        if is_pontic {
            let cache: BTreeMap<i32, TriMeshData> = self
                .other_restorations
                .iter()
                .map(|(&k, st)| (k, st.mesh.clone()))
                .collect();
            let placed = self.place_pontic_world(
                case.tooth_number,
                case,
                library_manager,
                &cache,
                self.prep_mesh.as_ref().map(|(m, _)| m),
                self.ant_mesh.as_ref().map(|(m, _)| m),
            );
            self.base_generated_crown = Some(placed.clone());
            self.positioned_source_mesh = Some(placed.clone());
            self.is_adapted_to_margin = true;
            let ax = Vec3::from_array(case.insertion_axis);
            self.set_crown_mesh(placed, ax);
            self.rebuild_auxiliary_shells(case);
            self.rebuild_connectors(case);
            self.design_dirty_for_bridge = true;
            self.audit_text = format!(
                "{} #{} ({}): ✔ {}\n{}",
                tr("Pieza", "Unit", "Dente"),
                case.tooth_number,
                tr("Póntico de Puente", "Bridge Pontic", "Pôntico de Ponte"),
                tr("Sólido Watertight", "Watertight Solid", "Sólido Watertight"),
                tr("Ajusta su forma y los conectores con las piezas vecinas.", "Sculpt shape and adjust bridge connectors.", "Ajuste a forma e os conectores com os dentes vizinhos.")
            );
            return;
        }

        let lib_tooth = if let Some(ov) = &self.custom_override_mesh {
            ov.clone()
        } else {
            library_manager.load_tooth_mesh(&case.design_params.library_id, case.tooth_number)
        };
        let margin = Self::margin_vec3(case);
        let ax = Vec3::from_array(case.insertion_axis);
        let placed = CrownDesigner::place_library_tooth_world(
            &lib_tooth,
            &margin,
            ax,
            &case.design_params,
            self.prep_mesh.as_ref().map(|(m, _)| m),
            self.ant_mesh.as_ref().map(|(m, _)| m),
        );
        self.base_generated_crown = Some(placed.clone());
        self.positioned_source_mesh = Some(placed.clone());
        self.is_adapted_to_margin = false;
        self.set_crown_mesh(placed, ax);
        self.rebuild_auxiliary_shells(case);
        self.rebuild_connectors(case);
        self.design_dirty_for_bridge = true;
        self.audit_text = format!(
            "{} #{}: {}\n{}",
            tr("Pieza", "Unit", "Dente"),
            case.tooth_number,
            tr("Colocación Libre (Gizmo)", "Free Placement (Gizmo)", "Posicionamento Livre (Gizmo)"),
            tr("Posiciona, moldea o esculpe y pulsa '★ Adaptar al Margen'.", "Position, morph or sculpt and click '★ Adapt to Margin'.", "Posicione, molde ou esculpa e clique em '★ Adaptar ao Término'.")
        );
    }

    pub fn generate_crown_now(
        &mut self,
        case: &mut DentalCase,
        case_manager: &CaseManager,
        library_manager: &ToothLibraryManager,
    ) -> Result<(), String> {
        if case.get_active_unit().map(|u| u.is_pontic()).unwrap_or(false) {
            if let Some((cur, _)) = self.crown_mesh.take() {
                if self.crown_mesh.is_none() {
                    let closed = ensure_pontic_closed(&cur);
                    let ax = Vec3::from_array(case.insertion_axis);
                    self.base_generated_crown = Some(closed.clone());
                    self.is_adapted_to_margin = true;
                    self.set_crown_mesh(closed, ax);
                }
            }
            self.rebuild_connectors(case);
            self.save_active_crown(case, case_manager);
            return Ok(());
        }

        let Some((prep, _)) = self.prep_mesh.as_ref().cloned() else {
            return Err("No hay malla de preparación cargada.".to_string());
        };
        let margin = Self::margin_vec3(case);
        if margin.len() < 8 {
            return Err(format!("Define primero la línea de margen de #{} en el Módulo 3.", case.tooth_number));
        }

        if self.crown_mesh.is_some() {
            self.push_undo(case);
        }
        let ax = Vec3::from_array(case.insertion_axis);

        let source_tooth = if !self.is_adapted_to_margin {
            if let Some((cur, _)) = &self.crown_mesh {
                self.positioned_source_mesh = Some(cur.clone());
                cur.clone()
            } else {
                let lib_tooth = if let Some(ov) = &self.custom_override_mesh {
                    ov.clone()
                } else {
                    library_manager.load_tooth_mesh(&case.design_params.library_id, case.tooth_number)
                };
                CrownDesigner::place_library_tooth_world(
                    &lib_tooth,
                    &margin,
                    ax,
                    &case.design_params,
                    Some(&prep),
                    self.ant_mesh.as_ref().map(|(m, _)| m),
                )
            }
        } else if let Some(src) = &self.positioned_source_mesh {
            src.clone()
        } else if let Some((cur, _)) = &self.crown_mesh {
            cur.clone()
        } else {
            let lib_tooth = if let Some(ov) = &self.custom_override_mesh {
                ov.clone()
            } else {
                library_manager.load_tooth_mesh(&case.design_params.library_id, case.tooth_number)
            };
            CrownDesigner::place_library_tooth_world(
                &lib_tooth,
                &margin,
                ax,
                &case.design_params,
                Some(&prep),
                self.ant_mesh.as_ref().map(|(m, _)| m),
            )
        };

        let (base_crown, audit) = CrownDesigner::generate_crown_from_positioned_tooth(
            &prep,
            &margin,
            &source_tooth,
            ax,
            &case.design_params,
            self.ant_mesh.as_ref().map(|(m, _)| m),
        )?;

        self.die_audit = audit.clone();
        if audit.auto_adjusted_axis {
            case.insertion_axis = audit.effective_axis.to_array();
            if let Some(u) = case.restorations.get_mut(&case.tooth_number) {
                u.insertion_axis = case.insertion_axis;
            }
        }

        self.base_generated_crown = Some(base_crown.clone());
        self.is_adapted_to_margin = true;

        let final_crown = CrownSculptor::apply_regional_morphing(
            &base_crown,
            &margin,
            audit.effective_axis,
            case.design_params.cusp_height_delta_mm,
            case.design_params.fossa_depth_delta_mm,
            case.design_params.equator_bulge_delta_mm,
        );
        self.set_crown_mesh(final_crown, audit.effective_axis);
        self.rebuild_auxiliary_shells(case);
        self.rebuild_connectors(case);
        self.design_dirty_for_bridge = true;
        self.save_active_crown(case, case_manager);
        Ok(())
    }

    pub fn save_active_crown(&mut self, case: &mut DentalCase, case_manager: &CaseManager) {
        let Some((crown, _)) = &self.crown_mesh else {
            return;
        };
        let fdi = case.tooth_number;
        let case_dir = case_manager.get_case_dir(&case.case_id);
        let unit_rel = format!("design/crown_{}_designed.stl", fdi);
        let _ = crown.save_stl_binary(&case_dir.join(&unit_rel));

        let main_path = case_dir.join("design").join("crown_designed.stl");
        let _ = crown.save_stl_binary(&main_path);
        case.crown_design_path = Some("design/crown_designed.stl".to_string());
        if let Some(u) = case.restorations.get_mut(&fdi) {
            u.crown_design_path = Some(unit_rel);
            u.design_params = case.design_params.clone();
        }
        let _ = case_manager.save_case(case);

        let audit = crown.inspect_cam();
        let wt = if audit.watertight {
            tr("✔ Sólido Adaptado (Watertight)", "✔ Adapted Solid (Watertight)", "✔ Sólido Adaptado (Watertight)")
        } else {
            tr("✔ Adaptado al Margen", "✔ Adapted to Margin", "✔ Adaptado ao Término")
        };
        self.audit_text = format!(
            "{} #{}: {}\nDim: {:.1}×{:.1}×{:.1} mm | Vol: {:.1} mm³",
            tr("Pieza", "Unit", "Dente"),
            fdi, wt, audit.dims_mm.x, audit.dims_mm.y, audit.dims_mm.z, audit.volume_mm3
        );
    }

    fn pick_on_crown(&self, pos: Pos2, rect: egui::Rect, active_fdi: i32) -> Option<Vec3> {
        if !self.is_unit_visible(active_fdi) {
            return None;
        }
        let col = self.crown_collider.as_ref()?;
        let (ro, rd) = self.camera.screen_ray(pos, rect);
        col.cast_ray(ro, rd, 500.0).map(|(_, hit)| hit)
    }

    fn pick_closest_visible_restoration(
        &self,
        active_fdi: i32,
        pos: Pos2,
        rect: egui::Rect,
    ) -> Option<(i32, Vec3)> {
        let (ro, rd) = self.camera.screen_ray(pos, rect);
        let mut best: Option<(f32, i32, Vec3)> = None;

        if self.is_unit_visible(active_fdi) {
            if let Some(col) = &self.crown_collider {
                if let Some((t, hit)) = col.cast_ray(ro, rd, 500.0) {
                    best = Some((t, active_fdi, hit));
                }
            }
        }

        for (&fdi, st) in &self.other_restorations {
            if !self.is_unit_visible(fdi) {
                continue;
            }
            if let Some(col) = &st.collider {
                if let Some((t, hit)) = col.cast_ray(ro, rd, 500.0) {
                    if best.map(|(bt, _, _)| t < bt).unwrap_or(true) {
                        best = Some((t, fdi, hit));
                    }
                }
            }
        }

        best.map(|(_, fdi, hit)| (fdi, hit))
    }

    fn find_hovered_conn_node(&self, pos: Pos2, rect: egui::Rect) -> Option<(usize, usize)> {
        if !self.vis_connectors || self.connectors.is_empty() {
            return None;
        }
        let ci = self.active_connector_idx.min(self.connectors.len() - 1);
        let cr = &self.connectors[ci];
        let mut best: Option<(usize, usize)> = None;
        let mut best_d = 18.0_f32;
        let mut check = |ni: usize, pt: Vec3| {
            if let Some(sp) = self.camera.world_to_screen(pt, rect) {
                let d = pos.distance(sp);
                if d <= best_d {
                    best_d = d;
                    best = Some((ci, ni));
                }
            }
        };
        for (k, &pt) in cr.node_pts.iter().enumerate() {
            check(k, pt);
        }
        check(8, cr.center_pt);
        for (k, &pt) in cr.ring_a_nodes.iter().enumerate() {
            check(9 + k, pt);
        }
        check(17, cr.anchor_a);
        for (k, &pt) in cr.ring_b_nodes.iter().enumerate() {
            check(18 + k, pt);
        }
        check(26, cr.anchor_b);
        best
    }

    fn pick_connector_body(&self, pos: Pos2, rect: egui::Rect) -> Option<usize> {
        if !self.vis_connectors || self.connectors.is_empty() {
            return None;
        }
        let mut best_ci = None;
        let mut best_d = 26.0_f32;
        for (ci, cr) in self.connectors.iter().enumerate() {
            if let Some(sp) = self.camera.world_to_screen(cr.center_pt, rect) {
                let d = pos.distance(sp);
                if d < best_d {
                    best_d = d;
                    best_ci = Some(ci);
                }
            }
        }
        best_ci
    }

    fn ensure_adapted_for_smart(
        &mut self,
        case: &mut DentalCase,
        case_manager: &CaseManager,
        library_manager: &ToothLibraryManager,
    ) -> bool {
        if self.crown_mesh.is_none() {
            return false;
        }
        if case.get_active_unit().map(|u| u.is_pontic()).unwrap_or(false) {
            return true;
        }
        if !self.is_adapted_to_margin && case.margin_points.len() >= 8 {
            let _ = self.generate_crown_now(case, case_manager, library_manager);
        }
        self.crown_mesh.is_some()
    }

    pub fn ui(
        &mut self,
        ctx: &egui::Context,
        case: &mut DentalCase,
        case_manager: &CaseManager,
        library_manager: &mut ToothLibraryManager,
        renderer: &Arc<Mutex<GlViewportRenderer>>,
    ) -> DesignAction {
        let mut action = DesignAction::None;
        let active_visible = self.is_unit_visible(case.tooth_number);

        // Si la pieza activa fue ocultada en Transparencias, cancelar cualquier manipulación en curso
        // para garantizar que las herramientas de transformación JAMÁS actúen sobre piezas desactivadas!
        if !active_visible {
            self.active_handle = None;
            self.drag_start_crown = None;
            self.transform_session = None;
            self.drag_locked_rect = None;
            self.active_rot_angle_deg = None;
            self.morph_session = None;
            self.morph_cursor_3d = None;
            self.is_sculpting = false;
            self.last_sculpt_pt = None;
            self.stroke_adjacency = None;
            self.sculpt_cache = None;
            self.sculpt_dab_count = 0;
            self.hovered_handle = None;
            self.hovered_label.clear();
            self.hover_cursor_3d = None;
        }

        // ==================================================================
        // LIENZO 3D A PANTALLA COMPLETA + INTERACCIÓN 3D (`parry3d` + `glow`)
        // ==================================================================
        egui::CentralPanel::default()
            .frame(Frame::NONE.fill(theme::BG_DEEP))
            .show(ctx, |ui| {
                let rect = ui.available_rect_before_wrap();
                let resp = ui.allocate_rect(rect, Sense::click_and_drag());
                let ctrl_down = ui.input(|i| i.modifiers.ctrl);
                let shift_down = ui.input(|i| i.modifiers.shift);

                // Construir las flechas fijas 3Shape SOLO si la restauración activa está VISIBLE
                let overlay = if active_visible
                    && self.tool_mode == DesignToolMode::Transform
                    && self.show_transform_gizmo
                {
                    let t_rect = self.drag_locked_rect.or_else(|| {
                        self.crown_mesh.as_ref().and_then(|(cm, _)| {
                            PlacementOverlay::compute_tooth_screen_rect(cm, &self.camera, rect)
                        })
                    });
                    t_rect.map(|tr| PlacementOverlay::build(tr, rect, self.transform_submode))
                } else {
                    None
                };

                // Ctrl + Rueda ajusta radio de Morphing o Pincel (el zoom estándar se maneja abajo)
                let mut ctrl_handled_scroll = false;
                if resp.hovered() {
                    let scroll = ui.input(|i| i.smooth_scroll_delta.y);
                    if scroll.abs() > 0.01 {
                        if ctrl_down && self.tool_mode == DesignToolMode::Morph {
                            self.morph_radius_mm = (self.morph_radius_mm + scroll.signum() * 0.20).clamp(0.50, 5.50);
                            self.cursor_rev = self.bump();
                            ctrl_handled_scroll = true;
                        } else if ctrl_down
                            && matches!(self.tool_mode, DesignToolMode::Add | DesignToolMode::Remove | DesignToolMode::Smooth)
                        {
                            self.brush_radius_mm = (self.brush_radius_mm + scroll.signum() * 0.18).clamp(0.35, 4.50);
                            self.cursor_rev = self.bump();
                            ctrl_handled_scroll = true;
                        }
                    }
                }

                // Press inicial sobre el lienzo 3D (Clic Izquierdo = Solo acción, nunca cámara)
                if ui.input(|i| i.pointer.primary_pressed()) && resp.hovered() {
                    if let Some(p_pos) = ui.input(|i| i.pointer.interact_pos()) {
                        match self.tool_mode {
                            DesignToolMode::Connector => {
                                if let Some((ci, ni)) = self.find_hovered_conn_node(p_pos, rect) {
                                    self.active_connector_idx = ci;
                                    self.active_conn_node = Some((ci, ni));
                                    self.drag_start_pos = p_pos;
                                    self.drag_conn_start_cfg = case.bridge_connectors.get(ci).cloned();
                                } else if let Some(ci) = self.pick_connector_body(p_pos, rect) {
                                    self.active_connector_idx = ci;
                                    self.conn_overlay_rev = self.bump();
                                } else if let Some((hit_fdi, _)) =
                                    self.pick_closest_visible_restoration(case.tooth_number, p_pos, rect)
                                {
                                    if hit_fdi != case.tooth_number {
                                        self.switch_active_tooth(hit_fdi, case, case_manager, library_manager);
                                        action = DesignAction::StatusMessage(format!(
                                            "{} #{} {}.",
                                            tr("Pieza", "Unit", "Dente"),
                                            hit_fdi,
                                            tr("activada", "activated", "ativado")
                                        ));
                                    }
                                }
                            }
                            DesignToolMode::Transform => {
                                let hit_arrow = if active_visible {
                                    overlay
                                        .as_ref()
                                        .and_then(|ov| ov.hit_test(p_pos))
                                        .map(|c| c.id.to_string())
                                } else {
                                    None
                                };

                                if let Some(hid) = hit_arrow {
                                    self.start_transform_drag(hid, None, p_pos, rect, overlay.as_ref(), case);
                                } else if let Some((hit_fdi, picked_pt)) =
                                    self.pick_closest_visible_restoration(case.tooth_number, p_pos, rect)
                                {
                                    if hit_fdi != case.tooth_number {
                                        self.switch_active_tooth(hit_fdi, case, case_manager, library_manager);
                                        action = DesignAction::StatusMessage(format!(
                                            "{} #{} {}.",
                                            tr("Pieza", "Unit", "Dente"),
                                            hit_fdi,
                                            tr("activada (Transf. Individual lista)", "activated (Individual Transform ready)", "ativado (Transf. Individual pronta)")
                                        ));
                                    } else if active_visible {
                                        self.start_transform_drag(
                                            "body_direct".to_string(),
                                            Some(picked_pt),
                                            p_pos,
                                            rect,
                                            overlay.as_ref(),
                                            case,
                                        );
                                    }
                                }
                            }
                            DesignToolMode::Morph => {
                                if active_visible {
                                    if let Some(picked) = self.pick_on_crown(p_pos, rect, case.tooth_number) {
                                        if let Some((cm, _)) = self.crown_mesh.as_ref().cloned() {
                                            self.push_undo(case);
                                            self.drag_start_pos = p_pos;
                                            self.morph_session = Some(CrownSculptor::prepare_morph_session(
                                                &cm,
                                                picked,
                                                self.morph_radius_mm,
                                            ));
                                            self.morph_cursor_3d = Some(picked);
                                            self.cursor_rev = self.bump();
                                        }
                                    } else if let Some((hit_fdi, _)) =
                                        self.pick_closest_visible_restoration(case.tooth_number, p_pos, rect)
                                    {
                                        if hit_fdi != case.tooth_number {
                                            self.switch_active_tooth(hit_fdi, case, case_manager, library_manager);
                                        }
                                    }
                                } else if let Some((hit_fdi, _)) =
                                    self.pick_closest_visible_restoration(case.tooth_number, p_pos, rect)
                                {
                                    self.switch_active_tooth(hit_fdi, case, case_manager, library_manager);
                                }
                            }
                            DesignToolMode::Add | DesignToolMode::Remove | DesignToolMode::Smooth => {
                                if active_visible {
                                    if let Some(picked) = self.pick_on_crown(p_pos, rect, case.tooth_number) {
                                        let cache = self
                                            .crown_mesh
                                            .as_ref()
                                            .map(|(m, _)| CrownSculptor::prepare_sculpt_cache(m));
                                        self.push_undo(case);
                                        self.is_sculpting = true;
                                        self.last_sculpt_pt = Some(picked);
                                        self.sculpt_dab_count = 0;
                                        self.sculpt_cache = cache;
                                        self.apply_brush_dab(picked, case);
                                    } else if let Some((hit_fdi, _)) =
                                        self.pick_closest_visible_restoration(case.tooth_number, p_pos, rect)
                                    {
                                        if hit_fdi != case.tooth_number {
                                            self.switch_active_tooth(hit_fdi, case, case_manager, library_manager);
                                        }
                                    }
                                } else if let Some((hit_fdi, _)) =
                                    self.pick_closest_visible_restoration(case.tooth_number, p_pos, rect)
                                {
                                    self.switch_active_tooth(hit_fdi, case, case_manager, library_manager);
                                }
                            }
                        }
                    }
                }

                // Movimiento / Arrastre activo (exclusivo Clic Izquierdo)
                let is_primary_down = ui.input(|i| i.pointer.primary_down());
                if is_primary_down {
                    if let Some(curr_pos) = ui.input(|i| i.pointer.interact_pos()) {
                                if let Some((ci, ni)) = self.active_conn_node {
                            if let (Some((tooth_a, tooth_b, fr)), Some(cfg)) = (
                                self.connectors.get(ci).map(|c| (c.tooth_a, c.tooth_b, c.frame.clone())),
                                case.bridge_connectors.get_mut(ci),
                            ) {
                                cfg.ensure_free_profile_nodes_initialized();
                                cfg.ensure_free_attach_nodes_initialized();
                                let eff_sc = |raw: f32| if raw <= 0.01 { 1.0 } else { raw.clamp(0.55, 1.85) };
                                let (ro, rd) = self.camera.screen_ray(curr_pos, rect);
                                let cam_f = self.camera.forward();
                                let active_col = if (9..=17).contains(&ni) {
                                    if tooth_a == case.tooth_number {
                                        self.crown_collider.as_ref()
                                    } else {
                                        self.other_restorations.get(&tooth_a).and_then(|s| s.collider.as_ref())
                                    }
                                } else if ni >= 18 {
                                    if tooth_b == case.tooth_number {
                                        self.crown_collider.as_ref()
                                    } else {
                                        self.other_restorations.get(&tooth_b).and_then(|s| s.collider.as_ref())
                                    }
                                } else {
                                    None
                                };

                                let hit_opt = if ni <= 8 {
                                    let plane_n = if rd.dot(fr.t_axis).abs() >= 0.25 {
                                        fr.t_axis
                                    } else {
                                        cam_f
                                    };
                                    let denom = rd.dot(plane_n);
                                    if denom.abs() > 1e-5 {
                                        let t_hit = (fr.center - ro).dot(plane_n) / denom;
                                        Some(ro + rd * t_hit)
                                    } else {
                                        None
                                    }
                                } else {
                                    let plane_pt = match ni {
                                        17 => fr.anchor_a,
                                        9..=16 => fr.ring_a_nodes[ni - 9],
                                        26 => fr.anchor_b,
                                        18..=25 => fr.ring_b_nodes[ni - 18],
                                        _ => fr.center,
                                    };
                                    active_col
                                        .and_then(|c| c.cast_ray(ro, rd, 250.0).map(|(_, p)| p))
                                        .or_else(|| {
                                            let denom = rd.dot(cam_f);
                                            if denom.abs() > 1e-5 {
                                                let t_hit = (plane_pt - ro).dot(cam_f) / denom;
                                                let raw = ro + rd * t_hit;
                                                Some(active_col.map(|c| c.project_point(raw)).unwrap_or(raw))
                                            } else {
                                                None
                                            }
                                        })
                                };

                                if let Some(hit_w) = hit_opt {
                                    if ni == 8 {
                                        let off = (hit_w - fr.base_center).clamp_length_max(5.5);
                                        cfg.center_offset = off.to_array();
                                    } else if ni < 8 {
                                        let sc = eff_sc(cfg.thickness_scale);
                                        let rel = (hit_w - fr.center) / sc;
                                        let mut v2 = glam::Vec2::new(rel.dot(fr.u_occ), rel.dot(fr.v_bl));
                                        let len = v2.length();
                                        if len > 6.0 {
                                            v2 = v2 / len * 6.0;
                                        } else if len < 0.35 {
                                            v2 = v2.normalize_or_zero() * 0.35;
                                        }
                                        cfg.profile_nodes_2d[ni] = [v2.x, v2.y];
                                        cfg.profile_radii_mm[ni] = v2.length().clamp(0.45, 5.8);
                                    } else if ni == 17 {
                                        let off = (hit_w - fr.base_anchor_a).clamp_length_max(7.5);
                                        cfg.attach_a_offset = off.to_array();
                                    } else if ni < 17 {
                                        let k = ni - 9;
                                        let sc = eff_sc(cfg.attach_a_scale);
                                        let rel = (hit_w - fr.anchor_a) / sc;
                                        let mut loc = Vec3::new(rel.dot(fr.u_occ), rel.dot(fr.v_bl), rel.dot(fr.t_axis));
                                        let len = loc.length();
                                        if len > 7.5 {
                                            loc = loc / len * 7.5;
                                        } else if len < 0.30 {
                                            loc = loc.normalize_or_zero() * 0.30;
                                        }
                                        cfg.attach_a_nodes_local[k] = loc.to_array();
                                        cfg.attach_a_radii_mm[k] = loc.x.hypot(loc.y).clamp(0.45, 5.8);
                                    } else if ni == 26 {
                                        let off = (hit_w - fr.base_anchor_b).clamp_length_max(7.5);
                                        cfg.attach_b_offset = off.to_array();
                                    } else if ni < 26 {
                                        let k = ni - 18;
                                        let sc = eff_sc(cfg.attach_b_scale);
                                        let rel = (hit_w - fr.anchor_b) / sc;
                                        let mut loc = Vec3::new(rel.dot(fr.u_occ), rel.dot(fr.v_bl), rel.dot(-fr.t_axis));
                                        let len = loc.length();
                                        if len > 7.5 {
                                            loc = loc / len * 7.5;
                                        } else if len < 0.30 {
                                            loc = loc.normalize_or_zero() * 0.30;
                                        }
                                        cfg.attach_b_nodes_local[k] = loc.to_array();
                                        cfg.attach_b_radii_mm[k] = loc.x.hypot(loc.y).clamp(0.45, 5.8);
                                    }
                                    self.design_dirty_for_bridge = true;
                                    self.rebuild_connectors(case);
                                }
                            }
                        } else if active_visible {
                            if let Some(hid) = self.active_handle.clone() {
                                let d2 = curr_pos - self.drag_start_pos;
                                let cam_r = self.drag_cam_right;
                                let cam_u = self.drag_cam_up;
                                let cam_f = self.drag_cam_fwd;
                                let spans = self.drag_tooth_spans_cam;

                                let mut delta_world_move = Vec3::ZERO;
                                let mut rot_world: Option<Quat> = None;
                                let mut scale_cam = Vec3::ONE;
                                let mut anchor_cam: Option<Vec3> = None;

                                if (hid == "move_center" || hid == "body_direct") && shift_down {
                                    let q_h = Quat::from_axis_angle(cam_u, d2.x * 0.011);
                                    let q_v = Quat::from_axis_angle(cam_r, d2.y * 0.011);
                                    rot_world = Some((q_h * q_v).normalize());
                                    self.active_rot_angle_deg = None;
                                } else if hid == "scale_uniform"
                                    || ((hid == "move_center" || hid == "body_direct") && ctrl_down)
                                {
                                    let sf = (1.0 + (d2.x - d2.y) * 0.0035).clamp(0.45, 2.2);
                                    scale_cam = Vec3::splat(sf);
                                    self.active_rot_angle_deg = None;
                                } else if hid == "move_center" || hid == "body_direct" {
                                    delta_world_move = self.camera.screen_delta_to_world(
                                        self.drag_origin_3d,
                                        self.drag_start_pos,
                                        curr_pos,
                                        rect,
                                    );
                                    self.active_rot_angle_deg = None;
                                } else if hid.starts_with("rot_") {
                                    let c = overlay
                                        .as_ref()
                                        .map(|ov| ov.center)
                                        .unwrap_or(self.drag_start_pos);
                                    let curr_ang = (curr_pos.y - c.y).atan2(curr_pos.x - c.x);
                                    let raw_diff = curr_ang - self.drag_start_angle;
                                    let delta_ang = raw_diff.sin().atan2(raw_diff.cos());
                                    rot_world = Some(Quat::from_axis_angle(cam_f, delta_ang));
                                    self.active_rot_angle_deg = Some(delta_ang.to_degrees());
                                } else if hid == "scale_right"
                                    || hid == "scale_left"
                                    || hid == "scale_up"
                                    || hid == "scale_down"
                                {
                                    self.active_rot_angle_deg = None;
                                    let (dir_px, anchor, is_x) = match hid.as_str() {
                                        "scale_right" => (egui::vec2(1.0, 0.0), Vec3::X, true),
                                        "scale_left" => (egui::vec2(-1.0, 0.0), Vec3::NEG_X, true),
                                        "scale_up" => (egui::vec2(0.0, -1.0), Vec3::Y, false),
                                        _ => (egui::vec2(0.0, 1.0), Vec3::NEG_Y, false),
                                    };
                                    let proj_px = d2.x * dir_px.x + d2.y * dir_px.y;
                                    let delta_mm = proj_px * self.drag_mm_per_px;
                                    if ctrl_down {
                                        let ref_span = 0.5 * (spans.x + spans.y);
                                        let sf = ((ref_span + delta_mm) / ref_span).clamp(0.45, 2.2);
                                        scale_cam = Vec3::splat(sf);
                                    } else {
                                        anchor_cam = Some(anchor);
                                        if is_x {
                                            scale_cam.x = ((spans.x + delta_mm) / spans.x).clamp(0.45, 2.2);
                                        } else {
                                            scale_cam.y = ((spans.y + delta_mm) / spans.y).clamp(0.45, 2.2);
                                        }
                                    }
                                }

                                let xform_updated = self.transform_session.as_ref().map(|sess| {
                                    CrownSculptor::apply_transform_session(
                                        sess,
                                        delta_world_move,
                                        rot_world,
                                        scale_cam,
                                        anchor_cam,
                                        case.design_params.min_thickness_mm,
                                    )
                                });
                                if let Some(updated) = xform_updated {
                                    self.set_crown_mesh_fast(updated);
                                }
                            } else if self.morph_session.is_some() {
                                let morph_res = self.morph_session.as_ref().map(|sess| {
                                    let dw = self.camera.screen_delta_to_world(
                                        sess.grab_point,
                                        self.drag_start_pos,
                                        curr_pos,
                                        rect,
                                    );
                                    let deformed = CrownSculptor::apply_morph_drag(
                                        sess,
                                        dw,
                                        case.design_params.min_thickness_mm,
                                        self.morph_normal_only,
                                    );
                                    let cur_pt = sess.grab_point
                                        + if self.morph_normal_only {
                                            sess.grab_normal * dw.dot(sess.grab_normal)
                                        } else {
                                            dw
                                        };
                                    (cur_pt, deformed)
                                });
                                if let Some((cur_pt, deformed)) = morph_res {
                                    self.morph_cursor_3d = Some(cur_pt);
                                    self.cursor_rev = self.bump();
                                    self.set_crown_mesh_fast(deformed);
                                }
                            } else if self.is_sculpting {
                                if let Some(picked) = self.pick_on_crown(curr_pos, rect, case.tooth_number) {
                                    self.hover_cursor_3d = Some(picked);
                                    self.cursor_rev = self.bump();
                                    let min_step = (self.brush_radius_mm * 0.16).max(0.07);
                                    let should_apply = self
                                        .last_sculpt_pt
                                        .map(|lp| lp.distance(picked) >= min_step)
                                        .unwrap_or(true);
                                    if should_apply {
                                        self.last_sculpt_pt = Some(picked);
                                        self.apply_brush_dab(picked, case);
                                        self.sculpt_dab_count += 1;
                                        if self.sculpt_dab_count % 6 == 0 {
                                            self.refresh_crown_collider();
                                        }
                                    }
                                }
                            }
                        }
                    }
                } else {
                    // Hover sin botón presionado
                    if let Some(hp) = resp.hover_pos() {
                        if self.tool_mode == DesignToolMode::Connector {
                            let new_cn = self.find_hovered_conn_node(hp, rect);
                            if new_cn != self.hovered_conn_node {
                                self.hovered_conn_node = new_cn;
                                self.conn_overlay_rev = self.bump();
                            }
                            self.hovered_handle = None;
                            self.hovered_label.clear();
                            self.hover_cursor_3d = None;
                        } else if self.tool_mode == DesignToolMode::Transform {
                            self.hovered_conn_node = None;
                            if active_visible {
                                if let Some(hit_c) = overlay.as_ref().and_then(|ov| ov.hit_test(hp)) {
                                    self.hovered_handle = Some(hit_c.id.to_string());
                                    self.hovered_label = hit_c.label.to_string();
                                } else {
                                    self.hovered_handle = None;
                                    self.hovered_label.clear();
                                }
                            } else {
                                self.hovered_handle = None;
                                self.hovered_label.clear();
                            }
                            self.hover_cursor_3d = None;
                        } else {
                            self.hovered_conn_node = None;
                            self.hovered_handle = None;
                            self.hovered_label.clear();
                            let prev_c = self.hover_cursor_3d;
                            self.hover_cursor_3d = if active_visible {
                                self.pick_on_crown(hp, rect, case.tooth_number)
                            } else {
                                None
                            };
                            if let Some(nc) = self.hover_cursor_3d {
                                let moved = prev_c
                                    .map(|pc| pc.distance_squared(nc) > 0.005 * 0.005)
                                    .unwrap_or(true);
                                if moved {
                                    self.cursor_rev = self.bump();
                                }
                            }
                        }
                    }
                }

                // Cámara 360° uniforme (Der+arrastre = Rotar; Rueda = Pan/Zoom; Izq = Solo acción)
                self.camera.handle_navigation_input(&resp, ui, rect, !ctrl_handled_scroll);

                // Release: finalizar arrastre, actualizar BVH/conectores una sola vez y persistir
                if ui.input(|i| i.pointer.primary_released()) {
                    if self.active_conn_node.take().is_some() {
                        self.drag_conn_start_cfg = None;
                        self.design_dirty_for_bridge = true;
                        let _ = case_manager.save_case(case);
                    }
                    if self.active_handle.take().is_some() {
                        self.drag_start_crown = None;
                        self.transform_session = None;
                        self.drag_locked_rect = None;
                        self.active_rot_angle_deg = None;
                        self.refresh_crown_collider();
                        self.base_generated_crown = self.crown_mesh.as_ref().map(|(m, _)| m.clone());
                        self.design_dirty_for_bridge = true;
                        self.rebuild_connectors(case);
                        if self.is_adapted_to_margin {
                            self.save_active_crown(case, case_manager);
                        } else {
                            self.positioned_source_mesh = self.crown_mesh.as_ref().map(|(m, _)| m.clone());
                            let _ = case_manager.save_case(case);
                        }
                    }
                    if self.morph_session.take().is_some() {
                        self.morph_cursor_3d = None;
                        self.refresh_crown_collider();
                        self.base_generated_crown = self.crown_mesh.as_ref().map(|(m, _)| m.clone());
                        self.design_dirty_for_bridge = true;
                        self.rebuild_connectors(case);
                        if self.is_adapted_to_margin {
                            self.save_active_crown(case, case_manager);
                        } else {
                            self.positioned_source_mesh = self.crown_mesh.as_ref().map(|(m, _)| m.clone());
                        }
                    }
                    if self.is_sculpting {
                        self.is_sculpting = false;
                        self.last_sculpt_pt = None;
                        self.stroke_adjacency = None;
                        self.sculpt_cache = None;
                        self.sculpt_dab_count = 0;
                        self.refresh_crown_collider();
                        self.base_generated_crown = self.crown_mesh.as_ref().map(|(m, _)| m.clone());
                        self.design_dirty_for_bridge = true;
                        self.rebuild_connectors(case);
                        if self.is_adapted_to_margin {
                            self.save_active_crown(case, case_manager);
                        } else {
                            self.positioned_source_mesh = self.crown_mesh.as_ref().map(|(m, _)| m.clone());
                        }
                    }
                }

                // ==========================================================
                // ENSAMBLAR TODAS LAS CAPAS PARA EL RENDERER OPENGL
                // ==========================================================
                let mut layers = Vec::new();
                if let Ok(mut guard) = renderer.lock() {
                    // 1. El Modelo (Muñón)
                    if self.vis_model {
                        if let Some((prep, rev)) = &self.prep_mesh {
                            guard.ensure_mesh_uploaded("d4_prep", *rev, prep, ColorMapKind::Solid);
                            let mut sp = RenderLayerSpec::solid("d4_prep", *rev, [0.90, 0.84, 0.75], self.op_model);
                            sp.specular = 0.25;
                            layers.push(sp);
                        }
                    }
                    // 2. Espacio de Cementado (solo si la pieza activa tiene muñón/margen y está activa)
                    if self.vis_cement {
                        if let Some((cem, rev)) = &self.cement_spacer_mesh {
                            guard.ensure_mesh_uploaded("d4_cem", *rev, cem, ColorMapKind::Solid);
                            layers.push(RenderLayerSpec::solid("d4_cem", *rev, [0.02, 0.71, 0.83], self.op_cement));
                        }
                    }
                    // 3. Grosor Mínimo
                    if self.vis_min_thick {
                        if let Some((mt, rev)) = &self.min_thickness_mesh {
                            guard.ensure_mesh_uploaded("d4_minth", *rev, mt, ColorMapKind::Solid);
                            layers.push(RenderLayerSpec::solid("d4_minth", *rev, [0.96, 0.62, 0.04], self.op_min_thick));
                        }
                    }
                    // 4. El Antagonista
                    if self.vis_antagonist {
                        if let Some((ant, rev)) = &self.ant_mesh {
                            guard.ensure_mesh_uploaded("d4_ant", *rev, ant, ColorMapKind::Solid);
                            layers.push(RenderLayerSpec::solid("d4_ant", *rev, [0.85, 0.80, 0.89], self.op_antagonist));
                        }
                    }
                    // Líneas de margen de piezas no pónticas en la arcada activa (solo reconstruye si cambia revisión)
                    let has_other_margins = case.restorations.iter().any(|(&ofdi, u)| {
                        ofdi != case.tooth_number
                            && !u.is_pontic()
                            && Self::unit_arch(ofdi) == case.prep_arch
                            && u.margin_points.len() >= 4
                    });
                    if has_other_margins {
                        let m_rev = (self.loaded_order_sig ^ self.margin_vis_rev.wrapping_mul(37)).max(1);
                        if !guard.is_mesh_current("d4_other_margins", m_rev) {
                            let mut gray_margins = TriMeshData::default();
                            for (&ofdi, u) in &case.restorations {
                                if ofdi == case.tooth_number
                                    || u.is_pontic()
                                    || Self::unit_arch(ofdi) != case.prep_arch
                                    || u.margin_points.len() < 4
                                {
                                    continue;
                                }
                                let omp: Vec<Vec3> = u.margin_points.iter().map(|p| Vec3::from_array(*p)).collect();
                                gray_margins.merge_in_place(&build_tube_mesh(&omp, self.margin_line_radius_mm * 0.9, true, 8));
                            }
                            guard.ensure_mesh_uploaded("d4_other_margins", m_rev, &gray_margins, ColorMapKind::Solid);
                        }
                        let mut sp = RenderLayerSpec::solid("d4_other_margins", m_rev, [0.58, 0.59, 0.64], 0.90);
                        sp.clip_affected = false;
                        layers.push(sp);
                    }
                    if !case.get_active_unit().map(|u| u.is_pontic()).unwrap_or(false)
                        && case.margin_points.len() >= 4
                    {
                        let m_rev = (self.loaded_margin_sig ^ self.margin_vis_rev.wrapping_mul(31)).max(1);
                        if !guard.is_mesh_current("d4_margin_ref", m_rev) {
                            let mp: Vec<Vec3> = case.margin_points.iter().map(|p| Vec3::from_array(*p)).collect();
                            let tube = build_tube_mesh(&mp, self.margin_line_radius_mm, true, 10);
                            guard.ensure_mesh_uploaded("d4_margin_ref", m_rev, &tube, ColorMapKind::Solid);
                        }
                        let mut sp = RenderLayerSpec::solid("d4_margin_ref", m_rev, [0.20, 0.85, 0.62], 1.0);
                        sp.clip_affected = false;
                        layers.push(sp);
                    }

                    // 5. TODAS las Restauraciones del caso simultáneamente con sus opacidades individuales
                    for (&fdi, st) in &self.other_restorations {
                        if !self.is_unit_visible(fdi) {
                            continue;
                        }
                        let key = format!("d4_other_{}", fdi);
                        guard.ensure_mesh_uploaded(&key, st.rev, &st.mesh, ColorMapKind::Solid);
                        let op = self.effective_unit_opacity(fdi);
                        let tint = if st.is_adapted_to_margin {
                            [0.97, 0.95, 0.88]
                        } else {
                            [0.93, 0.87, 0.76]
                        };
                        let mut sp = RenderLayerSpec::solid(key, st.rev, tint, op);
                        sp.specular = 0.50;
                        sp.shininess = 32.0;
                        layers.push(sp);
                    }
                    if self.is_unit_visible(case.tooth_number) {
                        if let Some((crown, rev)) = &self.crown_mesh {
                            let (cmap, key, use_vc) = match self.color_mode {
                                CrownColorMode::Ceramic => (ColorMapKind::Solid, "d4_crown_cer", false),
                                CrownColorMode::OcclusalHeatmap => (ColorMapKind::OcclusalJetR, "d4_crown_occ", true),
                                CrownColorMode::ThicknessHeatmap => (ColorMapKind::ThicknessViridis, "d4_crown_th", true),
                            };
                            guard.ensure_mesh_uploaded(key, *rev, crown, cmap);
                            let tint = if self.is_adapted_to_margin {
                                [1.0, 0.99, 0.97]
                            } else {
                                [0.96, 0.90, 0.78]
                            };
                            let mut sp = RenderLayerSpec::solid(
                                key,
                                *rev,
                                tint,
                                self.effective_unit_opacity(case.tooth_number),
                            );
                            sp.use_vertex_color = use_vc;
                            sp.specular = 0.58;
                            sp.shininess = 38.0;
                            layers.push(sp);
                        }
                    }

                    // 6. Conectores de Puente
                    if self.vis_connectors && !self.connectors.is_empty() {
                        let in_conn_mode = self.tool_mode == DesignToolMode::Connector;
                        for (ci, cr) in self.connectors.iter().enumerate() {
                            if cr.mesh.is_empty() {
                                continue;
                            }
                            let key = format!("d4_conn_{}_{}", cr.tooth_a, cr.tooth_b);
                            guard.ensure_mesh_uploaded(&key, cr.rev, &cr.mesh, ColorMapKind::Solid);
                            let is_act_c = ci == self.active_connector_idx;
                            let low_area = cr.area_mm2 + 0.05 < cr.min_area_mm2;
                            let tint = if in_conn_mode && is_act_c {
                                if low_area { [0.98, 0.62, 0.14] } else { [0.22, 0.86, 0.82] }
                            } else if low_area {
                                [0.96, 0.72, 0.35]
                            } else {
                                [0.97, 0.93, 0.84]
                            };
                            let mut sp = RenderLayerSpec::solid(key, cr.rev, tint, self.op_connectors);
                            sp.specular = 0.54;
                            sp.shininess = 34.0;
                            layers.push(sp);

                            // Contornos y 27 nodos editables estilo 3Shape (construcción condicional si cambió revisión)
                            if in_conn_mode && is_act_c {
                                let orev = (cr.rev ^ self.conn_overlay_rev.wrapping_mul(17)).max(1);
                                let any_hi = (0..27).any(|ni| {
                                    self.active_conn_node == Some((ci, ni))
                                        || self.hovered_conn_node == Some((ci, ni))
                                });
                                let need_rebuild = !guard.is_mesh_current("d4_conn_ring", orev)
                                    || (any_hi && !guard.is_mesh_current("d4_conn_hi", orev));
                                if need_rebuild {
                                    let ring_m = build_tube_mesh(&cr.waist_ring, 0.055, true, 10);
                                    guard.ensure_mesh_uploaded("d4_conn_ring", orev, &ring_m, ColorMapKind::Solid);

                                    let mut attach_rings = build_tube_mesh(&cr.ring_a_3d, 0.050, true, 10);
                                    attach_rings.merge_in_place(&build_tube_mesh(&cr.ring_b_3d, 0.050, true, 10));
                                    guard.ensure_mesh_uploaded("d4_conn_attach_rings", orev, &attach_rings, ColorMapKind::Solid);

                                    let mut reg_center_nodes = TriMeshData::default();
                                    let mut reg_attach_nodes = TriMeshData::default();
                                    let mut hi_nodes = TriMeshData::default();
                                    let mut push_node = |ni: usize, pt: Vec3, base_r: f32, is_attach: bool| {
                                        let is_hi = self.active_conn_node == Some((ci, ni))
                                            || self.hovered_conn_node == Some((ci, ni));
                                        let sph = build_uv_sphere(pt, if is_hi { base_r + 0.08 } else { base_r }, 8, 10);
                                        if is_hi {
                                            hi_nodes.merge_in_place(&sph);
                                        } else if is_attach {
                                            reg_attach_nodes.merge_in_place(&sph);
                                        } else {
                                            reg_center_nodes.merge_in_place(&sph);
                                        }
                                    };

                                    for (k, &pt) in cr.node_pts.iter().enumerate() {
                                        push_node(k, pt, 0.16, false);
                                    }
                                    push_node(8, cr.center_pt, 0.21, false);
                                    for (k, &pt) in cr.ring_a_nodes.iter().enumerate() {
                                        push_node(9 + k, pt, 0.16, true);
                                    }
                                    push_node(17, cr.anchor_a, 0.21, true);
                                    for (k, &pt) in cr.ring_b_nodes.iter().enumerate() {
                                        push_node(18 + k, pt, 0.16, true);
                                    }
                                    push_node(26, cr.anchor_b, 0.21, true);

                                    if !reg_center_nodes.is_empty() {
                                        guard.ensure_mesh_uploaded("d4_conn_nodes", orev, &reg_center_nodes, ColorMapKind::Solid);
                                    }
                                    if !reg_attach_nodes.is_empty() {
                                        guard.ensure_mesh_uploaded("d4_conn_attach_nodes", orev, &reg_attach_nodes, ColorMapKind::Solid);
                                    }
                                    if !hi_nodes.is_empty() {
                                        guard.ensure_mesh_uploaded("d4_conn_hi", orev, &hi_nodes, ColorMapKind::Solid);
                                    }
                                }

                                let mut sp_r = RenderLayerSpec::solid("d4_conn_ring", orev, [0.20, 0.96, 0.74], 1.0);
                                sp_r.always_on_top = true;
                                sp_r.clip_affected = false;
                                sp_r.unlit = true;
                                layers.push(sp_r);

                                let mut sp_ar = RenderLayerSpec::solid("d4_conn_attach_rings", orev, [0.98, 0.78, 0.22], 1.0);
                                sp_ar.always_on_top = true;
                                sp_ar.clip_affected = false;
                                sp_ar.unlit = true;
                                layers.push(sp_ar);

                                let mut sp_n = RenderLayerSpec::solid("d4_conn_nodes", orev, [0.96, 0.94, 0.86], 1.0);
                                sp_n.always_on_top = true;
                                sp_n.clip_affected = false;
                                layers.push(sp_n);

                                let mut sp_an = RenderLayerSpec::solid("d4_conn_attach_nodes", orev, [0.99, 0.80, 0.24], 1.0);
                                sp_an.always_on_top = true;
                                sp_an.clip_affected = false;
                                layers.push(sp_an);

                                if any_hi {
                                    let mut sp_h = RenderLayerSpec::solid("d4_conn_hi", orev, [0.78, 0.49, 1.00], 1.0);
                                    sp_h.always_on_top = true;
                                    sp_h.clip_affected = false;
                                    layers.push(sp_h);
                                }
                            }
                        }
                    }

                    // Esfera cursor de Morphing 3D o Pincel de Esculpido (solo si visible)
                    if active_visible {
                        let cur_pt = self.morph_cursor_3d.or(self.hover_cursor_3d);
                        if let Some(cp) = cur_pt {
                            let (rad, col) = match self.tool_mode {
                                DesignToolMode::Morph => (self.morph_radius_mm * 0.36, [0.78, 0.49, 1.0]),
                                DesignToolMode::Add => (self.brush_radius_mm * 0.36, [0.20, 0.83, 0.60]),
                                DesignToolMode::Remove => (self.brush_radius_mm * 0.36, [0.96, 0.25, 0.37]),
                                DesignToolMode::Smooth => (self.brush_radius_mm * 0.36, [0.22, 0.74, 0.97]),
                                DesignToolMode::Transform | DesignToolMode::Connector => (0.0, [1.0, 1.0, 1.0]),
                            };
                            if rad > 0.0 {
                                if !guard.is_mesh_current("d4_cursor", self.cursor_rev) {
                                    let sph = build_uv_sphere(cp, rad, 12, 16);
                                    guard.ensure_mesh_uploaded("d4_cursor", self.cursor_rev, &sph, ColorMapKind::Solid);
                                }
                                let mut sp = RenderLayerSpec::solid("d4_cursor", self.cursor_rev, col, 0.38);
                                sp.clip_affected = false;
                                layers.push(sp);
                            }
                        }
                    }
                }

                // Plano GPU de corte transversal 50%
                let clip_plane = if self.cross_section {
                    let orig = if !case.margin_points.is_empty() {
                        let m_pts = Self::margin_vec3(case);
                        m_pts.iter().copied().sum::<Vec3>() / m_pts.len() as f32
                    } else if let Some((c, _)) = &self.crown_mesh {
                        c.center()
                    } else {
                        Vec3::ZERO
                    };
                    let n = Vec3::X;
                    Some(Vec4::new(n.x, n.y, n.z, -n.dot(orig)))
                } else {
                    None
                };

                ui.painter().add(make_viewport_paint_callback(
                    Arc::clone(renderer),
                    rect,
                    &self.camera,
                    layers,
                    clip_plane,
                ));

                // Dibujar las flechas direccionales fijas estilo 3Shape encima del visor 3D (solo si pieza activa visible)
                if let Some(ov) = &overlay {
                    let clip_painter = ui.painter().with_clip_rect(rect);
                    ov.paint(
                        &clip_painter,
                        self.hovered_handle.as_deref(),
                        self.active_handle.as_deref(),
                        self.active_rot_angle_deg,
                    );
                }
            });

        // ==================================================================
        // BARRA FLOTANTE SUPERIOR CENTRAL: Vistas Rápidas de Cámara
        // ==================================================================
        Area::new(egui::Id::new("d4_cam_bar"))
            .anchor(Align2::CENTER_TOP, egui::vec2(0.0, 68.0))
            .order(Order::Foreground)
            .show(ctx, |ui| {
                Frame::NONE
                    .fill(Color32::from_rgba_unmultiplied(0x21, 0x05, 0x2C, 238))
                    .stroke(Stroke::new(1.3_f32, AMETHYST))
                    .corner_radius(CornerRadius::same(8))
                    .inner_margin(Margin::symmetric(8, 5))
                    .show(ui, |ui| {
                        ui.horizontal(|ui| {
                            if ui.button(tr("Oclusal (+Z)", "Occlusal (+Z)", "Oclusal (+Z)")).clicked() { self.camera.set_view_occlusal_top(); }
                            if ui.button(tr("Inferior (-Z)", "Apical (-Z)", "Inferior (-Z)")).clicked() { self.camera.set_view_occlusal_bottom(); }
                            if ui.button(tr("Vestibular", "Buccal", "Vestibular")).clicked() { self.camera.set_view_frontal(); }
                            if ui.button(tr("Lingual", "Lingual", "Lingual")).clicked() { self.camera.set_view_lingual(); }
                            if ui.button(tr("Proximal", "Proximal", "Proximal")).clicked() { self.camera.set_view_lateral(); }
                            if ui.button("Iso").clicked() { self.camera.set_view_isometric(); }
                            if ui.button(tr("Centrar", "Center", "Centralizar")).clicked() {
                                if let Some((c, _)) = &self.crown_mesh {
                                    let (mn, mx) = c.bounds();
                                    self.camera.frame_bounds(mn - Vec3::splat(3.0), mx + Vec3::splat(3.0));
                                }
                            }
                        });
                    });
            });

        // ==================================================================
        // VENTANA FLOTANTE IZQ 1: Pieza Activa, Material y Librería
        // ==================================================================
        Window::new(tr("1. Pieza Activa, Material y Librería", "1. Active Unit, Material & Library", "1. Dente Ativo, Material e Biblioteca"))
            .id(egui::Id::new("d4_card_lib"))
            .default_pos(egui::pos2(14.0, 74.0))
            .default_width(335.0)
            .collapsible(true)
            .resizable(false)
            .show(ctx, |ui| {
                let mut chosen_fdi = case.tooth_number;
                ui.horizontal(|ui| {
                    ui.label(tr("Pieza:", "Unit:", "Dente:"));
                    egui::ComboBox::from_id_salt("d4_tooth_cb")
                        .width(245.0)
                        .selected_text(format!(
                            "#{} — {}",
                            case.tooth_number,
                            case.get_active_unit()
                                .map(|u| i18n::tr_process_type(&u.process_type))
                                .unwrap_or_default()
                        ))
                        .show_ui(ui, |ui| {
                            for (&fdi, u) in &case.restorations {
                                let mat_t = i18n::tr_material_name(&u.material);
                                let ms = mat_t.split('(').next().unwrap_or(&mat_t).trim();
                                ui.selectable_value(
                                    &mut chosen_fdi,
                                    fdi,
                                    format!("#{} — {} ({})", fdi, i18n::tr_process_type(&u.process_type), ms),
                                );
                            }
                        });
                });
                if chosen_fdi != case.tooth_number {
                    self.switch_active_tooth(chosen_fdi, case, case_manager, library_manager);
                }

                // Selector de Material con aplicación automática de valores clínicos
                let mut cur_mat = case.design_params.material_preset.clone();
                let mut mat_changed = false;
                ui.horizontal(|ui| {
                    ui.label(tr("Material:", "Material:", "Material:"));
                    egui::ComboBox::from_id_salt("d4_mat_cb")
                        .width(235.0)
                        .selected_text(i18n::tr_material_name(&cur_mat))
                        .show_ui(ui, |ui| {
                            for m in APEX_MATERIAL_PRESETS {
                                if ui.selectable_value(&mut cur_mat, m.name.to_string(), i18n::tr_material_name(m.name)).clicked() {
                                    mat_changed = true;
                                }
                            }
                        });
                });
                if mat_changed {
                    let proc = case
                        .get_active_unit()
                        .map(|u| u.process_type.clone())
                        .unwrap_or_else(|| "Corona Individual".to_string());
                    case.design_params.apply_material_and_process_preset(&cur_mat, &proc);
                    if let Some(u) = case.restorations.get_mut(&case.tooth_number) {
                        u.material = cur_mat;
                        u.design_params = case.design_params.clone();
                    }
                    self.smart_occ_clearance_mm = case.design_params.occlusal_contact_offset_mm;
                    self.smart_prox_clearance_mm = case.design_params.proximal_contact_offset_mm;
                    self.rebuild_auxiliary_shells(case);
                    self.rebuild_connectors(case);
                    let _ = case_manager.save_case(case);
                }

                // Selector de Librería Dental
                let libs = library_manager.list_libraries();
                let mut cur_lib = case.design_params.library_id.clone();
                let mut lib_changed = false;
                let cur_lib_name = libs
                    .iter()
                    .find(|l| l.library_id == cur_lib)
                    .map(|l| format!("{} ({} FDI)", l.display_name, l.fdi_map.len()))
                    .unwrap_or_else(|| cur_lib.clone());
                ui.horizontal(|ui| {
                    ui.label(tr("Librería:", "Library:", "Biblioteca:"));
                    egui::ComboBox::from_id_salt("d4_lib_cb")
                        .width(235.0)
                        .selected_text(cur_lib_name)
                        .show_ui(ui, |ui| {
                            for l in &libs {
                                let lbl = format!("{} ({} FDI)", l.display_name, l.fdi_map.len());
                                if ui.selectable_value(&mut cur_lib, l.library_id.clone(), lbl).clicked() {
                                    lib_changed = true;
                                }
                            }
                        });
                });
                if lib_changed {
                    case.design_params.library_id = cur_lib;
                    self.custom_override_mesh = None;
                    self.place_approximate_tooth(case, library_manager, true);
                }

                let active_is_pontic = case.get_active_unit().map(|u| u.is_pontic()).unwrap_or(false);
                let p = &case.design_params;
                Frame::NONE
                    .fill(theme::BG_DEEP)
                    .stroke(Stroke::new(1.0_f32, theme::BORDER_PLUM))
                    .corner_radius(CornerRadius::same(5))
                    .inner_margin(Margin::symmetric(6, 4))
                    .show(ui, |ui| {
                        if active_is_pontic {
                            ui.label(
                                RichText::new(tr(
                                    "✔ Póntico Anatómico Sólido Watertight (Sin margen cervical)",
                                    "✔ Watertight Solid Anatomical Pontic (No cervical margin required)",
                                    "✔ Pôntico Anatômico Sólido Watertight (Sem término cervical)",
                                ))
                                .size(9.8)
                                .color(CYAN),
                            );
                        } else {
                            let fresa_txt = if p.drill_compensation_enabled {
                                format!("{} R={:.2} mm", tr("Fresa", "Bur", "Broca"), p.drill_radius_mm)
                            } else {
                                tr("Sin comp. fresa", "No bur comp.", "Sem comp. broca").to_string()
                            };
                            ui.label(
                                RichText::new(format!(
                                    "✔ {} {:.0} µm | {} {:.1} mm | {} | {}. {:.2} mm",
                                    tr("Cemento", "Cement", "Cimento"),
                                    p.cement_gap_mm * 1000.0,
                                    tr("Banda", "Band", "Faixa"),
                                    p.margin_ramp_width_mm,
                                    fresa_txt,
                                    tr("Mín", "Min", "Mín"),
                                    p.min_thickness_mm
                                ))
                                .size(9.8)
                                .color(EMERALD),
                            );
                        }
                    });

                ui.horizontal(|ui| {
                    if ui.button(tr("+ Librería...", "+ Library...", "+ Biblioteca...")).clicked() {
                        if let Some(folder) = rfd::FileDialog::new()
                            .set_title(tr("Seleccionar Carpeta con Librería Dental", "Select Dental Library Folder", "Selecionar Pasta de Biblioteca Dental"))
                            .pick_folder()
                        {
                            match library_manager.import_library_from_folder(&folder) {
                                Ok(info) => {
                                    case.design_params.library_id = info.library_id;
                                    self.custom_override_mesh = None;
                                    self.custom_stl_label = format!("{}: {}", tr("Librería activa", "Active library", "Biblioteca ativa"), info.display_name);
                                    self.place_approximate_tooth(case, library_manager, true);
                                }
                                Err(e) => action = DesignAction::StatusMessage(e),
                            }
                        }
                    }
                    if ui.button(tr("Diente STL Libre...", "Custom Tooth STL...", "Dente STL Livre...")).clicked() {
                        if let Some(file) = rfd::FileDialog::new()
                            .set_title(tr("Seleccionar Malla de Diente (.STL/.OBJ/.PLY)", "Select Tooth Mesh (.STL/.OBJ/.PLY)", "Selecionar Malha Dentária (.STL/.OBJ/.PLY)"))
                            .add_filter(tr("Mallas 3D", "3D Meshes", "Malhas 3D"), &["stl", "obj", "ply"])
                            .pick_file()
                        {
                            match TriMeshData::load_from_path(&file) {
                                Ok(m) => {
                                    self.custom_override_mesh = Some(m);
                                    self.custom_stl_label = format!(
                                        "{}: {}",
                                        tr("Archivo libre", "Custom file", "Arquivo livre"),
                                        file.file_name().and_then(|s| s.to_str()).unwrap_or("STL")
                                    );
                                    self.place_approximate_tooth(case, library_manager, true);
                                }
                                Err(e) => action = DesignAction::StatusMessage(e),
                            }
                        }
                    }
                });
                ui.label(RichText::new(&self.custom_stl_label).size(10.0).color(CREAM_MUTED));
                if ui.checkbox(&mut case.design_params.flip_library_z, tr("Invertir eje vertical (Z) de librería", "Flip library vertical axis (Z)", "Inverter eixo vertical (Z) da biblioteca")).changed() {
                    self.place_approximate_tooth(case, library_manager, true);
                }

                if !active_is_pontic {
                    if !self.die_audit.valid_for_production {
                        ui.label(
                            RichText::new(tr("⚠ Revisar Margen en Etapa 3 (Cemento incompleto)", "⚠ Check Margin in Stage 3 (Incomplete cement space)", "⚠ Revisar Término na Etapa 3 (Cimento incompleto)"))
                                .strong()
                                .size(10.0)
                                .color(Color32::from_rgb(0xFB, 0x71, 0x85)),
                        );
                    } else if self.die_audit.auto_adjusted_axis && self.die_audit.blockout_applied {
                        ui.label(
                            RichText::new(tr("✔ Cobertura 100% (eje auto-optimizado + bloqueo virtual)", "✔ 100% Coverage (auto-axis + virtual blockout)", "✔ Cobertura 100% (eixo auto-otimizado + bloqueio virtual)"))
                                .size(9.5)
                                .color(CYAN),
                        );
                    } else if self.die_audit.auto_adjusted_axis {
                        ui.label(
                            RichText::new(tr("✔ Cobertura 100% del muñón (eje auto-optimizado)", "✔ 100% Die coverage (auto-optimized axis)", "✔ Cobertura 100% do preparo (eixo auto-otimizado)"))
                                .size(9.5)
                                .color(CYAN),
                        );
                    } else if self.die_audit.blockout_applied {
                        ui.label(
                            RichText::new(tr("✔ Cobertura 100% (bloqueo virtual de socavados activo)", "✔ 100% Coverage (virtual undercut blockout active)", "✔ Cobertura 100% (bloqueio virtual de retenções ativo)"))
                                .size(9.5)
                                .color(CYAN),
                        );
                    }
                }
                ui.horizontal(|ui| {
                    if ui.button(tr("Posición Aproximada", "Initial Placement", "Posição Inicial")).clicked() {
                        self.place_approximate_tooth(case, library_manager, true);
                    }
                    if active_is_pontic {
                        if theme::accent_button(ui, tr("✔ Sellar Póntico Sólido", "✔ Seal Solid Pontic", "✔ Selar Pôntico Sólido")).clicked() {
                            if let Err(e) = self.generate_crown_now(case, case_manager, library_manager) {
                                action = DesignAction::StatusMessage(e);
                            } else {
                                action = DesignAction::StatusMessage(tr("Póntico cerrado watertight y listo para puente.", "Pontic sealed watertight and ready for bridge.", "Pôntico fechado watertight e pronto para a ponte.").to_string());
                            }
                        }
                    } else {
                        let can_adapt = self.die_audit.valid_for_production;
                        ui.add_enabled_ui(can_adapt, |ui| {
                            if theme::accent_button(ui, tr("★ Adaptar al Margen", "★ Adapt to Margin", "★ Adaptar ao Término")).clicked() {
                                if let Err(e) = self.generate_crown_now(case, case_manager, library_manager) {
                                    action = DesignAction::StatusMessage(e);
                                } else {
                                    action = DesignAction::StatusMessage(format!(
                                        "{} #{} {}",
                                        tr("Pieza", "Unit", "Dente"),
                                        case.tooth_number,
                                        tr("adaptada y sellada al margen cervical con cobertura 100%.", "adapted & sealed to cervical margin with 100% die coverage.", "adaptado e selado ao término cervical com cobertura 100%.")
                                    ));
                                }
                            }
                        });
                    }
                });
            });

        // ==================================================================
        // VENTANA FLOTANTE IZQ 2: Transformación Individual, Morphing, Esculpido y Conectores
        // ==================================================================
        Window::new(tr("2. Transformación, Morphing, Esculpido y Puentes", "2. Transform, Morphing, Sculpt & Bridges", "2. Transformação, Morphing, Escultura e Pontes"))
            .id(egui::Id::new("d4_card_modeling"))
            .default_pos(egui::pos2(14.0, 340.0))
            .default_width(335.0)
            .collapsible(true)
            .resizable(false)
            .show(ctx, |ui| {
                // Fila 1: Transformación Individual + Morphing 3D + Conectores (si hay puente)
                ui.horizontal(|ui| {
                    if ui
                        .selectable_label(
                            self.tool_mode == DesignToolMode::Transform,
                            RichText::new(tr("✥ Transformar", "✥ Transform", "✥ Transformar")).strong(),
                        )
                        .clicked()
                    {
                        self.tool_mode = DesignToolMode::Transform;
                    }
                    if ui
                        .selectable_label(
                            self.tool_mode == DesignToolMode::Morph,
                            RichText::new("🧬 Morphing").strong(),
                        )
                        .clicked()
                    {
                        self.tool_mode = DesignToolMode::Morph;
                    }
                    let conn_lbl = if self.connectors.is_empty() {
                        tr("🔗 Conectores", "🔗 Connectors", "🔗 Conectores").to_string()
                    } else {
                        format!("{} ({})", tr("🔗 Conectores", "🔗 Connectors", "🔗 Conectores"), self.connectors.len())
                    };
                    if ui
                        .selectable_label(
                            self.tool_mode == DesignToolMode::Connector,
                            RichText::new(conn_lbl).strong(),
                        )
                        .clicked()
                    {
                        self.tool_mode = DesignToolMode::Connector;
                    }
                });

                // Fila 2: Esculpido sin conflictos
                ui.horizontal(|ui| {
                    for (mode, lbl) in [
                        (DesignToolMode::Add, tr("➕ Aumentar", "➕ Add", "➕ Adicionar")),
                        (DesignToolMode::Remove, tr("➖ Quitar", "➖ Remove", "➖ Remover")),
                        (DesignToolMode::Smooth, tr("≈ Suavizar", "≈ Smooth", "≈ Suavizar")),
                    ] {
                        if ui.selectable_label(self.tool_mode == mode, RichText::new(lbl).strong()).clicked() {
                            self.tool_mode = mode;
                        }
                    }
                });

                if !active_visible && self.tool_mode != DesignToolMode::Connector {
                    Frame::NONE
                        .fill(Color32::from_rgba_unmultiplied(0x47, 0x2B, 0x06, 225))
                        .stroke(Stroke::new(1.2_f32, AMBER))
                        .corner_radius(CornerRadius::same(5))
                        .inner_margin(Margin::same(6))
                        .show(ui, |ui| {
                            ui.label(
                                RichText::new(format!(
                                    "⚠ {} #{} {}\n{}",
                                    tr("Pieza", "Unit", "Dente"),
                                    case.tooth_number,
                                    tr("oculta en Transparencias.", "hidden in Visibility Menu.", "oculta no Menu de Transparências."),
                                    tr("Edición bloqueada hasta reactivar su casilla.", "Editing locked until visibility is restored.", "Edição bloqueada até reativar a visualização.")
                                ))
                                .strong()
                                .size(10.0)
                                .color(Color32::from_rgb(0xFE, 0xF3, 0xC7)),
                            );
                        });
                }

                ui.separator();

                match self.tool_mode {
                    DesignToolMode::Transform => {
                        ui.checkbox(&mut self.show_transform_gizmo, RichText::new(tr("Mostrar Gizmo", "Show Gizmo", "Mostrar Gizmo")).color(CYAN).strong().size(10.5));
                        ui.horizontal(|ui| {
                            for (sm, lbl) in [
                                (TransformSubmode::All, tr("✥ Completo", "✥ All Handles", "✥ Completo")),
                                (TransformSubmode::MoveRotate, tr("↻ Mover / Rotar", "↻ Move / Rotate", "↻ Mover / Girar")),
                                (TransformSubmode::Scale, tr("⤢ Escalar", "⤢ Scale", "⤢ Escalar")),
                            ] {
                                if ui.selectable_label(self.transform_submode == sm, lbl).clicked() {
                                    self.transform_submode = sm;
                                }
                            }
                        });
                        let info_txt = if !self.hovered_label.is_empty() {
                            self.hovered_label.clone()
                        } else {
                            tr(
                                "• Clic Izq en otra pieza = Activarla | Flechas/Cuerpo = Editar\n• Shift = Girar 3D | Ctrl = Escala Proporcional\n• Clic Der + Arrastrar = Rotar cámara 360° | Rueda = Pan/Zoom",
                                "• Left-Click another unit = Activate | Gizmo/Body = Edit\n• Shift = Rotate 3D | Ctrl = Uniform Scale\n• Right-Drag = Orbit 360° | Wheel = Pan/Zoom",
                                "• Clique Esq em outro dente = Ativar | Gizmo/Corpo = Editar\n• Shift = Girar 3D | Ctrl = Escala Uniforme\n• Clique Dir + Arrastar = Girar câmera 360° | Roda = Pan/Zoom",
                            ).to_string()
                        };
                        Frame::NONE
                            .fill(theme::BG_DEEP)
                            .stroke(Stroke::new(1.0_f32, theme::BORDER_PLUM))
                            .corner_radius(CornerRadius::same(5))
                            .inner_margin(Margin::same(5))
                            .show(ui, |ui| {
                                ui.label(RichText::new(info_txt).size(10.0).color(CREAM_MUTED));
                            });
                    }
                    DesignToolMode::Morph => {
                        ui.horizontal(|ui| {
                            ui.label(RichText::new(tr("Zona Morphing:", "Morph Zone:", "Zona Morphing:")).strong().size(11.0));
                            ui.add(egui::Slider::new(&mut self.morph_radius_mm, 0.50..=5.50).show_value(false));
                            ui.label(RichText::new(format!("{:.2} mm", self.morph_radius_mm)).strong().color(AMETHYST_BRIGHT));
                        });
                        ui.checkbox(&mut self.morph_normal_only, tr("Restringir a normal de superficie", "Constrain to surface normal", "Restringir à normal da superfície"));
                        ui.label(
                            RichText::new(tr(
                                "Clic Izq y arrastra cualquier cúspide, reborde o pared (Ctrl + Rueda ajusta radio).",
                                "Left-drag any cusp, ridge or axial wall (Ctrl + Wheel adjusts radius).",
                                "Clique Esq e arraste qualquer cúspide, crista ou parede (Ctrl + Roda ajusta o raio).",
                            ))
                            .size(10.0)
                            .color(CREAM_MUTED),
                        );
                    }
                    DesignToolMode::Add | DesignToolMode::Remove | DesignToolMode::Smooth => {
                        ui.horizontal(|ui| {
                            ui.label(RichText::new(tr("Radio de pincel:", "Brush radius:", "Raio do pincel:")).strong().size(11.0));
                            ui.add(egui::Slider::new(&mut self.brush_radius_mm, 0.35..=4.50).show_value(false));
                            ui.label(RichText::new(format!("{:.2} mm", self.brush_radius_mm)).strong().color(EMERALD));
                        });
                        ui.horizontal(|ui| {
                            ui.label(RichText::new(tr("Intensidad:", "Strength:", "Intensidade:")).strong().size(11.0));
                            ui.add(egui::Slider::new(&mut self.brush_strength_mm, 0.02..=0.50).show_value(false));
                            ui.label(RichText::new(format!("{:.2} mm", self.brush_strength_mm)).strong().color(EMERALD));
                        });
                    }
                    DesignToolMode::Connector => {
                        if self.connectors.is_empty() || case.bridge_connectors.is_empty() {
                            ui.label(
                                RichText::new(tr(
                                    "No hay pares conectados (●) en este caso.\nUne dos o más piezas desde el odontograma del Módulo 1 para generar conectores de puente.",
                                    "No connected bridge pairs (●) in this case.\nLink two or more units in Module 1's odontogram to generate bridge connectors.",
                                    "Não há pares conectados (●) neste caso.\nConecte dois ou mais dentes no odontograma do Módulo 1 para gerar conectores de ponte.",
                                ))
                                .size(10.5)
                                .color(CREAM_MUTED),
                            );
                        } else {
                            let mut conn_changed = false;
                            let mut reset_conn = false;
                            let mut fit_min_conn = false;

                            ui.label(RichText::new(tr("Conectores:", "Connectors:", "Conectores:")).strong().size(10.8).color(CYAN));
                            let mut clicked_conn = None;
                            for (ci, cr) in self.connectors.iter().enumerate() {
                                let ok_area = cr.area_mm2 + 0.05 >= cr.min_area_mm2;
                                let col = if ok_area { EMERALD } else { AMBER };
                                let badge = format!(
                                    "#{} ↔ #{}  |  {:.1} mm² ({:.1}×{:.1} mm)  {}",
                                    cr.tooth_a,
                                    cr.tooth_b,
                                    cr.area_mm2,
                                    cr.height_mm,
                                    cr.width_mm,
                                    if ok_area { "✔" } else { tr("⚠ Bajo mín.", "⚠ Below min.", "⚠ Abaixo mín.") }
                                );
                                if ui
                                    .selectable_label(self.active_connector_idx == ci, RichText::new(badge).size(10.5).color(col))
                                    .clicked()
                                {
                                    clicked_conn = Some(ci);
                                }
                            }
                            if let Some(ci_sel) = clicked_conn {
                                self.active_connector_idx = ci_sel;
                                self.conn_overlay_rev = self.bump();
                            }

                            let ci = self.active_connector_idx.min(case.bridge_connectors.len() - 1);
                            let min_req = self.connectors.get(ci).map(|c| c.min_area_mm2).unwrap_or(9.0);
                            if let Some(cfg) = case.bridge_connectors.get_mut(ci) {
                                ui.separator();
                                conn_changed |= ui
                                    .add(egui::Slider::new(&mut cfg.thickness_scale, 0.55..=1.85).text(tr("Grosor Conector", "Connector Scale", "Espessura Conector")))
                                    .changed();
                                let (a_fdi, b_fdi) = (cfg.tooth_a, cfg.tooth_b);
                                let unit_lbl = tr("Unión Diente", "Attachment", "União Dente");
                                conn_changed |= ui
                                    .add(
                                        egui::Slider::new(&mut cfg.attach_a_scale, 0.60..=1.65)
                                            .text(format!("{} #{}", unit_lbl, a_fdi)),
                                    )
                                    .changed();
                                conn_changed |= ui
                                    .add(
                                        egui::Slider::new(&mut cfg.attach_b_scale, 0.60..=1.65)
                                            .text(format!("{} #{}", unit_lbl, b_fdi)),
                                    )
                                    .changed();
                                conn_changed |= ui
                                    .add(egui::Slider::new(&mut cfg.end_flare, 0.00..=0.45).text(tr("Transición Pilares", "End Flare / Taper", "Transição Pilares")))
                                    .changed();

                                let (h_og, w_vl) = compute_profile_dimensions_mm(cfg);
                                let scale_axis = |cfg: &mut BridgeConnectorConfig, primary: [usize; 2], axis_idx: usize, factor: f32| {
                                    let has_free_2d = cfg.profile_nodes_2d.iter().any(|p| p[0] * p[0] + p[1] * p[1] > 1e-4);
                                    if has_free_2d {
                                        for k in 0..8 {
                                            cfg.profile_nodes_2d[k][axis_idx] =
                                                (cfg.profile_nodes_2d[k][axis_idx] * factor).clamp(-6.0, 6.0);
                                            let r = cfg.profile_nodes_2d[k][0].hypot(cfg.profile_nodes_2d[k][1]);
                                            cfg.profile_radii_mm[k] = r.clamp(0.45, 5.8);
                                        }
                                    } else {
                                        let diag_factor = 0.5 * (1.0 + factor);
                                        for idx in primary {
                                            cfg.profile_radii_mm[idx] =
                                                (cfg.profile_radii_mm[idx] * factor).clamp(0.55, 5.2);
                                        }
                                        for idx in [1usize, 3, 5, 7] {
                                            cfg.profile_radii_mm[idx] =
                                                (cfg.profile_radii_mm[idx] * diag_factor).clamp(0.55, 5.2);
                                        }
                                    }
                                };
                                ui.horizontal(|ui| {
                                    ui.label(RichText::new(format!("{} {:.1}mm", tr("Alt. O-G", "H. O-G", "Alt. O-G"), h_og)).size(10.0).color(CREAM_MUTED));
                                    if ui.small_button("−").clicked() {
                                        scale_axis(cfg, [0, 4], 0, 0.90);
                                        conn_changed = true;
                                    }
                                    if ui.small_button("+").clicked() {
                                        scale_axis(cfg, [0, 4], 0, 1.10);
                                        conn_changed = true;
                                    }
                                    ui.separator();
                                    ui.label(RichText::new(format!("{} {:.1}mm", tr("Anc. V-L", "W. B-L", "Larg. V-L"), w_vl)).size(10.0).color(CREAM_MUTED));
                                    if ui.small_button("−").clicked() {
                                        scale_axis(cfg, [2, 6], 1, 0.90);
                                        conn_changed = true;
                                    }
                                    if ui.small_button("+").clicked() {
                                        scale_axis(cfg, [2, 6], 1, 1.10);
                                        conn_changed = true;
                                    }
                                });

                                ui.horizontal(|ui| {
                                    if ui.button(format!("{} ≥ {:.1} mm²", tr("Ajustar", "Fit", "Ajustar"), min_req)).clicked() {
                                        fit_min_conn = true;
                                    }
                                    if ui.button(tr("⟲ Restablecer", "⟲ Reset", "⟲ Redefinir")).clicked() {
                                        reset_conn = true;
                                    }
                                });
                            }
                            if fit_min_conn {
                                if let Some(cfg) = case.bridge_connectors.get_mut(ci) {
                                    let cur_a = compute_cross_section_area_mm2(cfg).max(0.5);
                                    if cur_a < min_req {
                                        let ratio = (min_req / cur_a).sqrt();
                                        cfg.thickness_scale = (cfg.thickness_scale * ratio).clamp(0.55, 1.85);
                                        conn_changed = true;
                                    }
                                }
                            }
                            if reset_conn {
                                if let Some(cfg) = case.bridge_connectors.get_mut(ci) {
                                    let (a, b) = (cfg.tooth_a, cfg.tooth_b);
                                    *cfg = BridgeConnectorConfig::for_pair_and_area(a, b, min_req);
                                    conn_changed = true;
                                }
                            }
                            if conn_changed {
                                self.design_dirty_for_bridge = true;
                                self.rebuild_connectors(case);
                                let _ = case_manager.save_case(case);
                            }

                            ui.label(
                                RichText::new(tr(
                                    "Arrastra libremente los nodos dorados por toda la superficie del diente para delimitar la zona de unión y los nodos centrales para el ancho/área (mm²).",
                                    "Freely drag gold attachment nodes anywhere across the tooth surface and center waist nodes to shape cross-section width/area (mm²).",
                                    "Arraste livremente os nós dourados por toda a superfície do dente para delimitar a zona de união e os nós centrais para largura/área (mm²).",
                                ))
                                .size(9.8)
                                .color(CREAM_MUTED),
                            );
                        }
                    }
                }

                ui.add_space(4.0);
                ui.horizontal(|ui| {
                    let can_undo = !self.undo_stack.is_empty();
                    if ui.add_enabled(can_undo, egui::Button::new(tr("↶ Deshacer (Ctrl+Z)", "↶ Undo (Ctrl+Z)", "↶ Desfazer (Ctrl+Z)"))).clicked() {
                        self.undo(case, case_manager);
                    }
                    if ui.button(tr("⟲ Reiniciar Transf.", "⟲ Reset Transform", "⟲ Redefinir Transf.")).clicked() {
                        self.push_undo(case);
                        case.design_params.reset_transform();
                        self.custom_override_mesh = None;
                        self.place_approximate_tooth(case, library_manager, false);
                    }
                });
            });

        // ==================================================================
        // VENTANA FLOTANTE IZQ 3 (Colapsada): Cemento 3 Zonas, Compensación de Fresado y Zonas
        // ==================================================================
        Window::new(tr("3. Cemento, Compensación de Fresado y Zonas", "3. Cement Space, Bur Compensation & Zones", "3. Cimento, Compensação de Broca e Zonas"))
            .id(egui::Id::new("d4_card_params"))
            .default_pos(egui::pos2(14.0, 610.0))
            .default_width(335.0)
            .default_open(false)
            .collapsible(true)
            .resizable(false)
            .show(ctx, |ui| {
                let mut die_changed = false;
                {
                    let p = &mut case.design_params;
                    ui.label(RichText::new(tr("Interfaz de Muñón (3 Zonas)", "Die Interface (3 Zones)", "Interface de Preparo (3 Zonas)")).strong().size(10.5).color(CYAN));
                    die_changed |= ui.add(egui::Slider::new(&mut p.cement_gap_mm, 0.0..=0.35).text(tr("Cemento Extra (mm)", "Extra Cement Gap (mm)", "Espaço de Cimento (mm)"))).changed();
                    die_changed |= ui.add(egui::Slider::new(&mut p.marginal_gap_mm, 0.0..=0.15).text(tr("Gap Marginal (mm)", "Marginal Gap (mm)", "Gap Marginal (mm)"))).changed();
                    die_changed |= ui.add(egui::Slider::new(&mut p.margin_ramp_width_mm, 0.2..=2.2).text(tr("Dist. a Margen (mm)", "Dist. to Margin (mm)", "Dist. ao Término (mm)"))).changed();
                    die_changed |= ui.add(egui::Slider::new(&mut p.smooth_distance_mm, 0.05..=1.0).text(tr("Transición Suave (mm)", "Smooth Ramp (mm)", "Transição Suave (mm)"))).changed();
                    die_changed |= ui.add(egui::Slider::new(&mut p.margin_bevel_mm, 0.00..=0.25).text(tr("Bisel Marginal (mm)", "Margin Bevel (mm)", "Bisel Marginal (mm)"))).changed();
                    if p.margin_bevel_mm > 0.005 {
                        die_changed |= ui.add(egui::Slider::new(&mut p.margin_bevel_angle_deg, 25.0..=70.0).text(tr("Ángulo Bisel (°)", "Bevel Angle (°)", "Ângulo Bisel (°)"))).changed();
                    }
                    die_changed |= ui.add(egui::Slider::new(&mut p.min_thickness_mm, 0.25..=2.5).text(tr("Grosor Mín. (mm)", "Min. Thickness (mm)", "Espessura Mín. (mm)"))).changed();
                    die_changed |= ui.checkbox(&mut p.remove_undercuts, tr("Bloqueo automático de socavados (1.5°)", "Automatic undercut blockout (1.5°)", "Bloqueio automático de retenções (1.5°)")).changed();
                    ui.checkbox(&mut p.auto_adapt_antagonist, tr("Recortar oclusión al antagonista al adaptar", "Trim occlusal collisions on adapt", "Recortar oclusão ao antagonista ao adaptar"));

                    ui.separator();
                    ui.label(RichText::new(tr("Compensación de Fresado (Drill Compensation)", "Bur Milling Compensation (Drill Comp.)", "Compensação de Fresagem (Broca)")).strong().size(10.5).color(EMERALD));
                    die_changed |= ui.checkbox(&mut p.drill_compensation_enabled, tr("Activar compensación de fresa en aristas", "Enable sharp-edge bur compensation", "Ativar compensação de broca em arestas")).changed();
                    if p.drill_compensation_enabled {
                        let drill_diam = p.drill_radius_mm * 2.0;
                        die_changed |= ui
                            .add(
                                egui::Slider::new(&mut p.drill_radius_mm, 0.20..=1.00)
                                    .text(format!("{} (Ø{:.2})", tr("Radio Fresa", "Bur Radius", "Raio da Broca"), drill_diam)),
                            )
                            .changed();
                        die_changed |= ui
                            .add(egui::Slider::new(&mut p.drill_compensation_offset_mm, 0.00..=0.20).text(tr("Alivio Extra (mm)", "Extra Relief (mm)", "Alívio Extra (mm)")))
                            .changed();
                    }
                }
                if die_changed {
                    self.rebuild_auxiliary_shells(case);
                    let _ = case_manager.save_case(case);
                }
                if self.is_adapted_to_margin && !case.get_active_unit().map(|u| u.is_pontic()).unwrap_or(false) {
                    if ui.add_sized([ui.available_width(), 22.0], egui::Button::new(tr("↻ Re-aplicar Cemento y Fresado a Corona", "↻ Re-apply Cement & Bur Comp.", "↻ Reaplicar Cimento e Fresagem na Coroa"))).clicked() {
                        if let Err(e) = self.generate_crown_now(case, case_manager, library_manager) {
                            action = DesignAction::StatusMessage(e);
                        } else {
                            action = DesignAction::StatusMessage(tr("Intaglio actualizado con nuevo espacio de cemento y compensación de fresado.", "Intaglio updated with new cement space and bur compensation.", "Intaglio atualizado com novo espaço de cimento e compensação de broca.").to_string());
                        }
                    }
                }

                ui.separator();
                let mut zone_changed = false;
                {
                    let p = &mut case.design_params;
                    zone_changed |= ui.add(egui::Slider::new(&mut p.cusp_height_delta_mm, -1.5..=1.5).text(tr("Cúspides (mm)", "Cusps (mm)", "Cúspides (mm)"))).changed();
                    zone_changed |= ui.add(egui::Slider::new(&mut p.fossa_depth_delta_mm, -1.2..=1.2).text(tr("Fosa (mm)", "Fossa (mm)", "Fossa (mm)"))).changed();
                    zone_changed |= ui.add(egui::Slider::new(&mut p.equator_bulge_delta_mm, -1.2..=1.2).text(tr("Ecuador (mm)", "Height of Contour (mm)", "Equador Protético (mm)"))).changed();
                }
                if zone_changed {
                    if let Some(base) = self.base_generated_crown.clone() {
                        let margin = Self::margin_vec3(case);
                        let ax = Vec3::from_array(case.insertion_axis);
                        let morphed = CrownSculptor::apply_regional_morphing(
                            &base,
                            &margin,
                            ax,
                            case.design_params.cusp_height_delta_mm,
                            case.design_params.fossa_depth_delta_mm,
                            case.design_params.equator_bulge_delta_mm,
                        );
                        self.set_crown_mesh(morphed, ax);
                        self.rebuild_connectors(case);
                    }
                }
            });

        // ==================================================================
        // VENTANA FLOTANTE ESQUINA SUP. DERECHA: Menú de Transparencias 3D (con Restauraciones Individuales)
        // ==================================================================
        let right_x = (ctx.screen_rect().width() - 342.0).max(380.0);
        Window::new(tr("Menú de Transparencias 3D", "3D Visibility & Opacity", "Menu de Transparências 3D"))
            .id(egui::Id::new("d4_card_trans"))
            .default_pos(egui::pos2(right_x, 74.0))
            .default_width(328.0)
            .collapsible(true)
            .resizable(false)
            .show(ctx, |ui| {
                fn row_layer(ui: &mut egui::Ui, vis: &mut bool, op: &mut f32, lbl: &str, col: Color32) {
                    ui.horizontal(|ui| {
                        ui.checkbox(vis, RichText::new(lbl).color(col).strong().size(10.5));
                        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                            ui.label(RichText::new(format!("{:.0}%", *op * 100.0)).size(10.0).color(CREAM_MUTED));
                            ui.add(egui::Slider::new(op, 0.05..=1.0).show_value(false));
                        });
                    });
                }
                row_layer(ui, &mut self.vis_cement, &mut self.op_cement, tr("Espacio de Cementado", "Cement Space", "Espaço de Cimento"), Color32::from_rgb(0x06, 0xB6, 0xD4));
                row_layer(ui, &mut self.vis_min_thick, &mut self.op_min_thick, tr("Grosor Mínimo", "Min. Thickness", "Espessura Mínima"), AMBER);
                row_layer(ui, &mut self.vis_restorations, &mut self.op_restorations, tr("Restauraciones (Global)", "Restorations (Global)", "Restaurações (Global)"), Color32::WHITE);

                // Sub-lista individual de transparencia por cada restauración (#FDI)
                let unit_list: Vec<(i32, String)> = case
                    .restorations
                    .iter()
                    .map(|(&fdi, u)| (fdi, u.process_type.clone()))
                    .collect();
                let mut fdi_to_activate: Option<i32> = None;
                for (fdi, proc) in unit_list {
                    let is_act = fdi == case.tooth_number;
                    let short_p = if proc.to_lowercase().contains("póntico") || proc.to_lowercase().contains("pontico") {
                        tr("Póntico", "Pontic", "Pôntico")
                    } else if proc.contains("Pilar") {
                        tr("Pilar", "Abutment", "Pilar")
                    } else if proc.contains("Carilla") {
                        tr("Carilla", "Veneer", "Faceta")
                    } else {
                        tr("Corona", "Crown", "Coroa")
                    };
                    ui.horizontal(|ui| {
                        ui.add_space(12.0);
                        let vis = self.unit_visible.entry(fdi).or_insert(true);
                        ui.checkbox(vis, "");
                        let lbl = if is_act {
                            RichText::new(format!("★ #{} {}", fdi, short_p)).strong().size(10.2).color(EMERALD)
                        } else {
                            RichText::new(format!("#{} {}", fdi, short_p)).size(10.2).color(CREAM)
                        };
                        if ui.selectable_label(is_act, lbl).clicked() {
                            fdi_to_activate = Some(fdi);
                        }
                        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                            let op = self.unit_opacity.entry(fdi).or_insert(1.0);
                            ui.label(RichText::new(format!("{:.0}%", *op * 100.0)).size(9.8).color(CREAM_MUTED));
                            ui.add_sized([76.0, 16.0], egui::Slider::new(op, 0.05..=1.0).show_value(false));
                        });
                    });
                }
                if let Some(target_fdi) = fdi_to_activate {
                    self.switch_active_tooth(target_fdi, case, case_manager, library_manager);
                }

                if !self.connectors.is_empty() {
                    row_layer(ui, &mut self.vis_connectors, &mut self.op_connectors, tr("Conectores de Puente", "Bridge Connectors", "Conectores da Ponte"), CYAN);
                }

                row_layer(ui, &mut self.vis_model, &mut self.op_model, tr("El Modelo (Muñón)", "Master Model (Die)", "Modelo (Preparo)"), CREAM);
                row_layer(ui, &mut self.vis_antagonist, &mut self.op_antagonist, tr("El Antagonista", "Opposing Arch", "Antagonista"), Color32::from_rgb(0xD8, 0xCC, 0xE2));

                ui.add_space(3.0);
                egui::ComboBox::from_id_salt("d4_color_mode")
                    .width(ui.available_width())
                    .selected_text(match self.color_mode {
                        CrownColorMode::Ceramic => tr("Acabado Cerámico / Zirconia Marfil", "Ceramic / Ivory Zirconia Finish", "Acabamento Cerâmico / Zircônia Marfim"),
                        CrownColorMode::OcclusalHeatmap => tr("Mapa Térmico de Contactos Oclusales (mm)", "Occlusal Contacts Heatmap (mm)", "Mapa Térmico de Contatos Oclusais (mm)"),
                        CrownColorMode::ThicknessHeatmap => tr("Mapa Térmico de Grosor de Pared (mm)", "Wall Thickness Heatmap (mm)", "Mapa Térmico de Espessura de Parede (mm)"),
                    })
                    .show_ui(ui, |ui| {
                        ui.selectable_value(&mut self.color_mode, CrownColorMode::Ceramic, tr("Acabado Cerámico / Zirconia Marfil", "Ceramic / Ivory Zirconia Finish", "Acabamento Cerâmico / Zircônia Marfim"));
                        ui.selectable_value(&mut self.color_mode, CrownColorMode::OcclusalHeatmap, tr("Mapa Térmico de Contactos Oclusales (mm)", "Occlusal Contacts Heatmap (mm)", "Mapa Térmico de Contatos Oclusais (mm)"));
                        ui.selectable_value(&mut self.color_mode, CrownColorMode::ThicknessHeatmap, tr("Mapa Térmico de Grosor de Pared (mm)", "Wall Thickness Heatmap (mm)", "Mapa Térmico de Espessura de Parede (mm)"));
                    });

                ui.checkbox(&mut self.cross_section, tr("Corte Transversal 50% (Inspeccionar Cemento)", "50% Cross-Section (Inspect Cement Space)", "Corte Transversal 50% (Inspecionar Cimento)"));
                ui.label(
                    RichText::new(&self.audit_text)
                        .size(10.0)
                        .color(if self.is_adapted_to_margin { EMERALD } else { CYAN }),
                );
            });

        // ==================================================================
        // VENTANA FLOTANTE DER 2: Herramientas Inteligentes (Smart Tools)
        // ==================================================================
        let active_vis_for_smart = self.is_unit_visible(case.tooth_number);
        Window::new(tr("Herramientas Inteligentes (Smart Tools)", "Smart Tools", "Ferramentas Inteligentes (Smart Tools)"))
            .id(egui::Id::new("d4_card_smart"))
            .default_pos(egui::pos2(right_x, 385.0))
            .default_width(328.0)
            .collapsible(true)
            .resizable(false)
            .show(ctx, |ui| {
                ui.add_enabled_ui(active_vis_for_smart, |ui| {
                    ui.horizontal(|ui| {
                        ui.label(RichText::new(tr("Holgura Oclusal:", "Occlusal Clearance:", "Espaço Oclusal:")).size(10.5).strong());
                        ui.add(egui::Slider::new(&mut self.smart_occ_clearance_mm, -0.10..=0.30).show_value(false));
                        ui.label(RichText::new(format!("{:+.2} mm", self.smart_occ_clearance_mm)).size(10.5).color(TEAL).strong());
                    });
                    ui.horizontal(|ui| {
                        if ui.button(tr("✂ Recortar Oclusión", "✂ Cut Intersections", "✂ Cortar Oclusão")).clicked() {
                            if self.ensure_adapted_for_smart(case, case_manager, library_manager) {
                                if let (Some((cr, _)), Some((ant, _))) = (self.crown_mesh.clone(), self.ant_mesh.clone()) {
                                    self.push_undo(case);
                                    let ax = Vec3::from_array(case.insertion_axis);
                                    let res = SmartToolsEngine::smart_occlusal_contacts(
                                        &cr, &ant, ax, self.smart_occ_clearance_mm, false, case.design_params.min_thickness_mm,
                                    );
                                    self.base_generated_crown = Some(res.clone());
                                    self.set_crown_mesh(res, ax);
                                    self.rebuild_connectors(case);
                                    self.save_active_crown(case, case_manager);
                                    action = DesignAction::StatusMessage(tr("Interferencias oclusales recortadas inteligentemente.", "Occlusal intersections trimmed intelligently.", "Interferências oclusais removidas com precisão.").to_string());
                                } else {
                                    action = DesignAction::StatusMessage(tr("Importa un escaneo antagonista primero.", "Import an opposing arch scan first.", "Importe um escaneamento antagonista primeiro.").to_string());
                                }
                            }
                        }
                        if ui.button(tr("⤒ Adaptar a Antagonista", "⤒ Adapt to Antagonist", "⤒ Adaptar ao Antagonista")).clicked() {
                            if self.ensure_adapted_for_smart(case, case_manager, library_manager) {
                                if let (Some((cr, _)), Some((ant, _))) = (self.crown_mesh.clone(), self.ant_mesh.clone()) {
                                    self.push_undo(case);
                                    let ax = Vec3::from_array(case.insertion_axis);
                                    let res = SmartToolsEngine::smart_occlusal_contacts(
                                        &cr, &ant, ax, self.smart_occ_clearance_mm, true, case.design_params.min_thickness_mm,
                                    );
                                    self.base_generated_crown = Some(res.clone());
                                    self.set_crown_mesh(res, ax);
                                    self.rebuild_connectors(case);
                                    self.save_active_crown(case, case_manager);
                                    action = DesignAction::StatusMessage(tr("Oclusión adaptada al antagonista.", "Occlusion adapted to opposing arch.", "Oclusão adaptada ao antagonista.").to_string());
                                } else {
                                    action = DesignAction::StatusMessage(tr("Importa un escaneo antagonista primero.", "Import an opposing arch scan first.", "Importe um escaneamento antagonista primeiro.").to_string());
                                }
                            }
                        }
                    });

                    ui.separator();
                    ui.horizontal(|ui| {
                        ui.label(RichText::new(tr("Contacto Proximal:", "Proximal Contact:", "Contato Proximal:")).size(10.5).strong());
                        ui.add(egui::Slider::new(&mut self.smart_prox_clearance_mm, -0.08..=0.12).show_value(false));
                        ui.label(RichText::new(format!("{:+.2} mm", self.smart_prox_clearance_mm)).size(10.5).color(TEAL).strong());
                    });
                    if ui.add_sized([ui.available_width(), 24.0], egui::Button::new(tr("⇄ Ajustar Contactos Proximales (Mesial/Distal)", "⇄ Adjust Proximal Contacts (Mesial/Distal)", "⇄ Ajustar Contatos Proximais (Mesial/Distal)"))).clicked() {
                        if self.ensure_adapted_for_smart(case, case_manager, library_manager) {
                            if let Some((cr, _)) = self.crown_mesh.clone() {
                                self.push_undo(case);
                                let ax = Vec3::from_array(case.insertion_axis);
                                let margin = Self::margin_vec3(case);
                                let others: BTreeMap<i32, TriMeshData> = self
                                    .other_restorations
                                    .iter()
                                    .map(|(&k, st)| (k, st.mesh.clone()))
                                    .collect();
                                let res = SmartToolsEngine::smart_proximal_contacts(
                                    &cr,
                                    self.prep_mesh.as_ref().map(|(m, _)| m),
                                    &margin,
                                    ax,
                                    &others,
                                    self.smart_prox_clearance_mm,
                                    case.design_params.min_thickness_mm,
                                );
                                self.base_generated_crown = Some(res.clone());
                                self.set_crown_mesh(res, ax);
                                self.rebuild_connectors(case);
                                self.save_active_crown(case, case_manager);
                                action = DesignAction::StatusMessage(tr("Contactos proximales Mesial/Distal ajustados.", "Mesial/Distal proximal contacts adjusted.", "Contatos proximais Mesial/Distal ajustados.").to_string());
                            }
                        }
                    }

                    ui.separator();
                    ui.horizontal(|ui| {
                        if ui.button(tr("🛡 Grosor Mínimo", "🛡 Min. Wall Thickness", "🛡 Espessura Mínima")).clicked() {
                            if self.ensure_adapted_for_smart(case, case_manager, library_manager) {
                                if let Some((cr, _)) = self.crown_mesh.clone() {
                                    self.push_undo(case);
                                    let ax = Vec3::from_array(case.insertion_axis);
                                    let res = SmartToolsEngine::smart_enforce_min_thickness(
                                        &cr,
                                        case.design_params.min_thickness_mm,
                                    );
                                    self.base_generated_crown = Some(res.clone());
                                    self.set_crown_mesh(res, ax);
                                    self.rebuild_connectors(case);
                                    self.save_active_crown(case, case_manager);
                                    action = DesignAction::StatusMessage(tr("Grosor mínimo reforzado según material activo.", "Minimum wall thickness enforced for active material.", "Espessura mínima reforçada conforme material ativo.").to_string());
                                }
                            }
                        }
                        if ui.button(tr("✨ Suavizado Inteligente", "✨ Smart Smooth", "✨ Suavização Inteligente")).clicked() {
                            if let Some((cr, _)) = self.crown_mesh.clone() {
                                self.push_undo(case);
                                let ax = Vec3::from_array(case.insertion_axis);
                                let res = SmartToolsEngine::smart_anatomical_smooth(
                                    &cr,
                                    4,
                                    case.design_params.min_thickness_mm,
                                );
                                self.base_generated_crown = Some(res.clone());
                                self.set_crown_mesh(res, ax);
                                self.rebuild_connectors(case);
                                if self.is_adapted_to_margin {
                                    self.save_active_crown(case, case_manager);
                                }
                                action = DesignAction::StatusMessage(tr("Suavizado anatómico Taubin aplicado.", "Anatomical Taubin smoothing applied.", "Suavização anatômica Taubin aplicada.").to_string());
                            }
                        }
                    });

                    ui.separator();
                    ui.horizontal(|ui| {
                        ui.label(RichText::new(tr("Emergencia:", "Emergence:", "Emergência:")).size(10.5).strong());
                        ui.add(egui::Slider::new(&mut self.smart_emergence_mm, -0.30..=0.40).show_value(false));
                        ui.label(RichText::new(format!("{:+.2} mm", self.smart_emergence_mm)).size(10.5).color(TEAL).strong());
                    });
                    if ui.add_sized([ui.available_width(), 24.0], egui::Button::new(tr("◠ Optimizar Perfil de Emergencia Cervical", "◠ Optimize Cervical Emergence Profile", "◠ Otimizar Perfil de Emergência Cervical"))).clicked() {
                        if self.ensure_adapted_for_smart(case, case_manager, library_manager) {
                            if let Some((cr, _)) = self.crown_mesh.clone() {
                                self.push_undo(case);
                                let ax = Vec3::from_array(case.insertion_axis);
                                let margin = Self::margin_vec3(case);
                                let res = SmartToolsEngine::smart_emergence_profile(
                                    &cr,
                                    &margin,
                                    ax,
                                    self.smart_emergence_mm,
                                    case.design_params.min_thickness_mm,
                                );
                                self.base_generated_crown = Some(res.clone());
                                self.set_crown_mesh(res, ax);
                                self.rebuild_connectors(case);
                                self.save_active_crown(case, case_manager);
                                action = DesignAction::StatusMessage(tr("Perfil de emergencia cervical optimizado.", "Cervical emergence profile optimized.", "Perfil de emergência cervical otimizado.").to_string());
                            }
                        }
                    }

                    ui.separator();
                    ui.horizontal(|ui| {
                        ui.label(RichText::new(tr("Espacio Gingival:", "Gingival Offset:", "Espaço Gengival:")).size(10.5).strong());
                        ui.add(egui::Slider::new(&mut self.smart_gingival_offset_mm, -0.50..=0.50).show_value(false));
                        ui.label(RichText::new(format!("{:+.2} mm", self.smart_gingival_offset_mm)).size(10.5).color(CYAN).strong());
                    });
                    if ui.add_sized([ui.available_width(), 24.0], egui::Button::new(tr("🦷 Adaptar Póntico a la Encía", "🦷 Adapt Pontic to Gingiva", "🦷 Adaptar Pôntico à Gengiva"))).clicked() {
                        let is_p = case.get_active_unit().map(|u| u.is_pontic()).unwrap_or(false);
                        if !is_p {
                            action = DesignAction::StatusMessage(tr(
                                "Selecciona una pieza marcada como Póntico de Puente para adaptar su base a la encía.",
                                "Select a unit assigned as Bridge Pontic to adapt its base to the gingiva.",
                                "Selecione um dente definido como Pôntico de Ponte para adaptá-lo à gengiva.",
                            ).to_string());
                        } else if let Some((cr, _)) = self.crown_mesh.clone() {
                            if let Some((prep, _)) = self.prep_mesh.as_ref() {
                                let ax = Vec3::from_array(case.insertion_axis);
                                let adapted = SmartToolsEngine::smart_adapt_pontic_to_gingiva(
                                    &cr,
                                    prep,
                                    ax,
                                    self.smart_gingival_offset_mm,
                                    None,
                                );
                                let closed = ensure_pontic_closed(&adapted);
                                self.push_undo(case);
                                self.base_generated_crown = Some(closed.clone());
                                self.set_crown_mesh(closed, ax);
                                self.rebuild_connectors(case);
                                self.design_dirty_for_bridge = true;
                                self.save_active_crown(case, case_manager);
                                action = DesignAction::StatusMessage(tr(
                                    "Base del póntico reducida y adaptada limpiamente al reborde gingival.",
                                    "Pontic base reduced and cleanly adapted to the gingival ridge.",
                                    "Base do pôntico reduzida e adaptada perfeitamente ao rebordo gengival.",
                                ).to_string());
                            } else {
                                action = DesignAction::StatusMessage(tr(
                                    "No hay malla de modelo/encía cargada para adaptar el póntico.",
                                    "No master model / gingiva mesh loaded to adapt the pontic.",
                                    "Nenhuma malha de modelo/gengiva carregada para adaptar o pôntico.",
                                ).to_string());
                            }
                        }
                    }
                });
            });

        // ==================================================================
        // ALERTA BLOQUEANTE DE ÚLTIMO RECURSO (solo si el margen de un pilar impide cubrir el muñón)
        // ==================================================================
        if !case.get_active_unit().map(|u| u.is_pontic()).unwrap_or(false) && !self.die_audit.valid_for_production {
            Area::new(egui::Id::new("d4_margin_blocker"))
                .anchor(Align2::CENTER_TOP, egui::vec2(0.0, 108.0))
                .order(Order::Foreground)
                .show(ctx, |ui| {
                    Frame::NONE
                        .fill(Color32::from_rgba_unmultiplied(0x4C, 0x05, 0x19, 245))
                        .stroke(Stroke::new(1.8_f32, Color32::from_rgb(0xFB, 0x71, 0x85)))
                        .corner_radius(CornerRadius::same(8))
                        .inner_margin(Margin::symmetric(12, 8))
                        .show(ui, |ui| {
                            ui.set_max_width(520.0);
                            ui.label(
                                RichText::new(&self.die_audit.warning_message)
                                    .strong()
                                    .size(11.0)
                                    .color(CREAM),
                            );
                            ui.add_space(4.0);
                            if ui.button(tr("← Volver a Etapa 3 para Corregir la Línea de Margen", "← Return to Stage 3 to Fix Margin Line", "← Voltar à Etapa 3 para Corrigir Término")).clicked() {
                                action = DesignAction::ReturnToMargin;
                            }
                        });
                });
        }

        // ==================================================================
        // BOTÓN FLOTANTE INFERIOR DERECHO: Validar CAM y Exportar STL
        // ==================================================================
        Area::new(egui::Id::new("d4_next_bar"))
            .anchor(Align2::RIGHT_BOTTOM, egui::vec2(-14.0, -34.0))
            .order(Order::Foreground)
            .show(ctx, |ui| {
                let is_p = case.get_active_unit().map(|u| u.is_pontic()).unwrap_or(false);
                let can_proceed = is_p || self.die_audit.valid_for_production;
                Frame::NONE
                    .fill(Color32::from_rgba_unmultiplied(0x21, 0x05, 0x2C, 240))
                    .stroke(Stroke::new(
                        1.5_f32,
                        if can_proceed {
                            EMERALD
                        } else {
                            Color32::from_rgb(0xFB, 0x71, 0x85)
                        },
                    ))
                    .corner_radius(CornerRadius::same(8))
                    .inner_margin(Margin::same(6))
                    .show(ui, |ui| {
                        let next_lbl = if case.has_bridge() {
                            tr("Siguiente: Esculpir Puente Unido  →", "Next: Final Bridge Sculpting  →", "Próximo: Escultura Final da Ponte  →")
                        } else {
                            tr("Validar CAM y Exportar STL Final  →", "CAM Check & Export Final STL  →", "Validar CAM e Exportar STL Final  →")
                        };
                        ui.add_enabled_ui(can_proceed, |ui| {
                            if theme::primary_button(ui, next_lbl).clicked() {
                                let mut ok_to_go = true;
                                if !is_p && !self.is_adapted_to_margin && case.margin_points.len() >= 8 {
                                    if let Err(e) = self.generate_crown_now(case, case_manager, library_manager) {
                                        action = DesignAction::StatusMessage(e);
                                        ok_to_go = false;
                                    }
                                }
                                if ok_to_go {
                                    self.save_all_restorations(case, case_manager);
                                    action = DesignAction::ProceedToExport;
                                }
                            }
                        });
                    });
            });

        action
    }

    fn start_transform_drag(
        &mut self,
        hid: String,
        picked_direct: Option<Vec3>,
        p_pos: Pos2,
        rect: egui::Rect,
        overlay: Option<&PlacementOverlay>,
        case: &DentalCase,
    ) {
        self.push_undo(case);
        self.active_handle = Some(hid);
        self.drag_start_pos = p_pos;
        let crown_c = self.crown_mesh.as_ref().map(|(m, _)| m.center()).unwrap_or(Vec3::ZERO);
        self.drag_origin_3d = picked_direct.unwrap_or(crown_c);
        self.drag_start_crown = None;
        self.active_rot_angle_deg = None;

        let (cam_r, cam_u) = self.camera.right_and_up();
        let cam_f = self.camera.forward();
        self.drag_cam_right = cam_r;
        self.drag_cam_up = cam_u;
        self.drag_cam_fwd = cam_f;
        let (_, mm_px) = self.camera.screen_axis_dir_and_scale(crown_c, cam_r, rect);
        self.drag_mm_per_px = mm_px;

        self.transform_session = self.crown_mesh.as_ref().map(|(cm, _)| {
            let ax = Vec3::from_array(case.insertion_axis);
            let lock_cervical = self.is_adapted_to_margin
                && !case.get_active_unit().map(|u| u.is_pontic()).unwrap_or(false);
            CrownSculptor::prepare_transform_session(cm, ax, lock_cervical, cam_r, cam_u, cam_f)
        });

        if let Some(ov) = overlay {
            self.drag_locked_rect = Some(ov.tooth_rect);
            self.drag_start_angle = (p_pos.y - ov.center.y).atan2(p_pos.x - ov.center.x);
        } else {
            self.drag_locked_rect = None;
            self.drag_start_angle = 0.0;
        }

        if let Some((cm, _)) = &self.crown_mesh {
            let mut min_x = f32::INFINITY;
            let mut max_x = f32::NEG_INFINITY;
            let mut min_y = f32::INFINITY;
            let mut max_y = f32::NEG_INFINITY;
            for &p in &cm.positions {
                let r = p - crown_c;
                let lx = r.dot(cam_r);
                let ly = r.dot(cam_u);
                min_x = min_x.min(lx);
                max_x = max_x.max(lx);
                min_y = min_y.min(ly);
                max_y = max_y.max(ly);
            }
            self.drag_tooth_spans_cam = egui::vec2(
                (max_x - min_x).max(3.0),
                (max_y - min_y).max(3.0),
            );
        }
    }

    fn apply_brush_dab(&mut self, center: Vec3, case: &DentalCase) {
        let mode = match self.tool_mode {
            DesignToolMode::Add => SculptMode::Add,
            DesignToolMode::Remove => SculptMode::Remove,
            DesignToolMode::Smooth => SculptMode::Smooth,
            _ => return,
        };
        let Some((mut crown, _)) = self.crown_mesh.take() else {
            return;
        };
        if let Some(sc) = &self.sculpt_cache {
            CrownSculptor::apply_brush_stroke_cached(
                &mut crown,
                center,
                mode,
                self.brush_radius_mm,
                self.brush_strength_mm,
                case.design_params.min_thickness_mm,
                sc,
            );
        } else {
            CrownSculptor::apply_brush_stroke(
                &mut crown,
                center,
                mode,
                self.brush_radius_mm,
                self.brush_strength_mm,
                case.design_params.min_thickness_mm,
                self.stroke_adjacency.as_deref(),
            );
        }
        self.set_crown_mesh_fast(crown);
    }
}
