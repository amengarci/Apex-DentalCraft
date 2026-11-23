//! Módulo 5 (Flujo de Puentes estilo 3Shape): Esculpido Libre, Morphing 3D
//! y Herramientas Inteligentes sobre el Puente Unido con Conectores antes de Exportar.

use crate::core::case_manager::{BridgeConnectorConfig, CaseManager, DentalCase};
use crate::core::i18n::tr;
use crate::core::settings::AppSettings;
use crate::geometry::connector::{
    compute_cross_section_area_mm2, compute_profile_dimensions_mm, fuse_bridge_watertight,
};
use crate::geometry::margin::{build_tube_mesh, build_uv_sphere};
use crate::geometry::mesh::{build_orthonormal_basis, CamAudit, MeshCollider, TriMeshData};
use crate::geometry::sculpt::{CrownSculptor, MorphSession, SculptMode, SculptStrokeCache};
use crate::geometry::smart_tools::SmartToolsEngine;
use crate::render::camera::OrbitCamera;
use crate::render::gl_renderer::{
    make_viewport_paint_callback, ColorMapKind, GlViewportRenderer, RenderLayerSpec,
};
use crate::theme::{self, AMBER, AMETHYST, AMETHYST_BRIGHT, CREAM, CREAM_MUTED, CYAN, EMERALD};
use crate::ui::design_panel::CrownColorMode;
use egui::{Align2, Area, Color32, CornerRadius, Frame, Margin, Order, Pos2, RichText, Sense, Stroke, Window};
use glam::{Vec3, Vec4};
use std::collections::BTreeSet;
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};

pub enum BridgeSculptAction {
    None,
    ReturnToDesign,
    ProceedToExport,
    StatusMessage(String),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BridgeToolMode {
    Morph,
    Add,
    Remove,
    Smooth,
}

pub struct BridgeGroupRuntime {
    pub fdis: Vec<i32>,
    pub pontic_zones: Vec<(Vec3, f32)>,
    pub label: String,
    pub file_tag: String,
    pub connector_summary: Vec<String>,
    pub mesh: TriMeshData,
    pub collider: Option<MeshCollider>,
    pub rev: u64,
    pub audit: CamAudit,
}

pub struct BridgeSculptPanelState {
    pub camera: OrbitCamera,
    pub next_rev: u64,

    pub prep_mesh: Option<(TriMeshData, u64)>,
    pub ant_mesh: Option<(TriMeshData, u64)>,
    pub bridges: Vec<BridgeGroupRuntime>,
    pub active_bridge_idx: usize,

    pub tool_mode: BridgeToolMode,
    pub brush_radius_mm: f32,
    pub brush_strength_mm: f32,
    pub morph_radius_mm: f32,
    pub morph_normal_only: bool,
    pub smart_occ_clearance_mm: f32,
    pub smart_gingival_offset_mm: f32,
    pub color_mode: CrownColorMode,

    pub vis_bridge: bool,
    pub op_bridge: f32,
    pub vis_model: bool,
    pub op_model: f32,
    pub vis_antagonist: bool,
    pub op_antagonist: f32,
    pub vis_margins: bool,
    pub cross_section: bool,
    pub margin_line_radius_mm: f32,

    pub morph_session: Option<MorphSession>,
    pub morph_cursor_3d: Option<Vec3>,
    pub drag_start_pos: Pos2,
    pub is_sculpting: bool,
    pub last_sculpt_pt: Option<Vec3>,
    pub stroke_adjacency: Option<Vec<Vec<usize>>>,
    pub sculpt_cache: Option<SculptStrokeCache>,
    pub sculpt_dab_count: usize,
    pub hover_cursor_3d: Option<Vec3>,
    pub cursor_rev: u64,

    pub undo_stack: Vec<(usize, TriMeshData)>,
    pub loaded_case_id: Option<String>,
    pub loaded_sig: u64,
}

impl Default for BridgeSculptPanelState {
    fn default() -> Self {
        Self {
            camera: OrbitCamera::default(),
            next_rev: 1,
            prep_mesh: None,
            ant_mesh: None,
            bridges: Vec::new(),
            active_bridge_idx: 0,
            tool_mode: BridgeToolMode::Morph,
            brush_radius_mm: 1.60,
            brush_strength_mm: 0.14,
            morph_radius_mm: 2.60,
            morph_normal_only: false,
            smart_occ_clearance_mm: 0.00,
            smart_gingival_offset_mm: 0.00,
            color_mode: CrownColorMode::Ceramic,
            vis_bridge: true,
            op_bridge: 1.00,
            vis_model: true,
            op_model: 0.85,
            vis_antagonist: false,
            op_antagonist: 0.45,
            vis_margins: true,
            cross_section: false,
            margin_line_radius_mm: 0.028,
            morph_session: None,
            morph_cursor_3d: None,
            drag_start_pos: Pos2::ZERO,
            is_sculpting: false,
            last_sculpt_pt: None,
            stroke_adjacency: None,
            sculpt_cache: None,
            sculpt_dab_count: 0,
            hover_cursor_3d: None,
            cursor_rev: 1,
            undo_stack: Vec::new(),
            loaded_case_id: None,
            loaded_sig: 0,
        }
    }
}

/// Resuelve la ruta exacta de diseño guardado para una pieza FDI individual (corona, pilar o póntico).
pub fn resolve_unit_design_path(case: &DentalCase, case_manager: &CaseManager, fdi: i32) -> Option<PathBuf> {
    let case_dir = case_manager.get_case_dir(&case.case_id);
    if let Some(unit) = case.restorations.get(&fdi) {
        if let Some(rel) = &unit.crown_design_path {
            let p = case_dir.join(rel);
            if p.exists() {
                return Some(p);
            }
        }
    }
    let unit_p = case_dir.join("design").join(format!("crown_{}_designed.stl", fdi));
    if unit_p.exists() {
        return Some(unit_p);
    }
    if fdi == case.tooth_number || case.restorations.len() <= 1 {
        let fallback = case_manager.get_crown_design_path(case, Some(fdi));
        if fallback.exists() {
            return Some(fallback);
        }
    }
    None
}

/// Recopila las configuraciones de conectores activas dentro de un grupo FDI junto con su resumen `mm²`.
pub fn collect_group_connectors(
    case: &DentalCase,
    loaded_set: &BTreeSet<i32>,
) -> (Vec<BridgeConnectorConfig>, Vec<String>) {
    let mut group_cfgs: Vec<BridgeConnectorConfig> = case
        .bridge_connectors
        .iter()
        .filter(|c| loaded_set.contains(&c.tooth_a) && loaded_set.contains(&c.tooth_b))
        .cloned()
        .collect();

    for (a, b) in case.connected_pairs() {
        if loaded_set.contains(&a)
            && loaded_set.contains(&b)
            && !group_cfgs
                .iter()
                .any(|c| (c.tooth_a.min(c.tooth_b), c.tooth_a.max(c.tooth_b)) == (a, b))
        {
            let min_a = case
                .restorations
                .get(&a)
                .map(|u| u.design_params.connector_area_mm2)
                .unwrap_or(12.0)
                .max(
                    case.restorations
                        .get(&b)
                        .map(|u| u.design_params.connector_area_mm2)
                        .unwrap_or(12.0),
                );
            group_cfgs.push(BridgeConnectorConfig::for_pair_and_area(a, b, min_a));
        }
    }
    for c in &mut group_cfgs {
        c.ensure_attachment_radii_initialized();
    }
    group_cfgs.sort_by_key(|c| (c.tooth_a, c.tooth_b));

    let summary = group_cfgs
        .iter()
        .map(|c| {
            let area = compute_cross_section_area_mm2(c);
            let (h_og, w_vl) = compute_profile_dimensions_mm(c);
            format!("#{}–#{}: {:.1} mm² ({:.1}×{:.1} mm)", c.tooth_a, c.tooth_b, area, h_og, w_vl)
        })
        .collect();

    (group_cfgs, summary)
}

/// Comprueba si `bridge_{fdis}_final.stl` existe y es al menos tan reciente como todas las piezas miembro.
pub fn is_final_bridge_up_to_date(final_path: &Path, member_paths: &[PathBuf]) -> bool {
    let Ok(final_meta) = std::fs::metadata(final_path) else {
        return false;
    };
    let Ok(final_mod) = final_meta.modified() else {
        return false;
    };
    for mp in member_paths {
        if let Ok(mm) = std::fs::metadata(mp) {
            if let Ok(mmod) = mm.modified() {
                if mmod > final_mod {
                    return false;
                }
            }
        }
    }
    true
}

impl BridgeSculptPanelState {
    fn bump(&mut self) -> u64 {
        self.next_rev += 1;
        self.next_rev
    }

    pub fn apply_settings(&mut self, settings: &AppSettings) {
        self.margin_line_radius_mm = (settings.margin_line_radius_mm * 0.90).clamp(0.015, 0.120);
        self.camera.rotate_sensitivity = settings.camera_rotate_sensitivity;
        self.camera.zoom_sensitivity = settings.camera_zoom_sensitivity;
        self.camera.invert_zoom = settings.camera_invert_zoom;
        self.op_bridge = settings.stage4_restoration_opacity;
        self.op_model = settings.stage4_model_opacity;
        self.vis_antagonist = settings.stage4_show_antagonist;
    }

    fn compute_case_sig(case: &DentalCase) -> u64 {
        let mut h: u64 = 14695981039346656037;
        for g in case.connected_bridge_groups() {
            for f in g {
                h ^= f as u64;
                h = h.wrapping_mul(1099511628211);
            }
        }
        for c in &case.bridge_connectors {
            h ^= (c.tooth_a as u64) | ((c.tooth_b as u64) << 16);
            h = h.wrapping_mul(1099511628211);
            h ^= c.thickness_scale.to_bits() as u64;
        }
        h
    }

    /// Bloquea únicamente intaglio y líneas de margen de los pilares con preparación;
    /// pónticos, conectores y paredes externas permanecen 100% moldeables.
    pub fn classify_bridge_external_vertices(
        bridge: &mut TriMeshData,
        prep: Option<&TriMeshData>,
        abutment_margins: &[Vec<Vec3>],
    ) {
        let prep_col = prep.and_then(MeshCollider::from_mesh);
        Self::classify_bridge_external_vertices_with_col(bridge, prep_col.as_ref(), abutment_margins);
    }

    pub fn classify_bridge_external_vertices_with_col(
        bridge: &mut TriMeshData,
        prep_col: Option<&MeshCollider>,
        abutment_margins: &[Vec<Vec3>],
    ) {
        let n = bridge.positions.len();
        if n == 0 {
            return;
        }
        if abutment_margins.is_empty() {
            bridge.is_external = vec![true; n];
            return;
        }

        // Centro, radio y AABB de cada margen/pilar para descartar vértices lejanos al instante
        struct MarginZone<'a> {
            pts: &'a [Vec3],
            mn: Vec3,
            mx: Vec3,
            center: Vec3,
            radius_sq: f32,
        }
        let zones: Vec<MarginZone<'_>> = abutment_margins
            .iter()
            .filter(|m| m.len() >= 4)
            .map(|m| {
                let mut mn = Vec3::splat(f32::INFINITY);
                let mut mx = Vec3::splat(f32::NEG_INFINITY);
                let mut sum = Vec3::ZERO;
                for &p in m {
                    mn = mn.min(p);
                    mx = mx.max(p);
                    sum += p;
                }
                let c = sum / m.len() as f32;
                let r = m
                    .iter()
                    .map(|&p| p.distance(c))
                    .fold(0.0_f32, f32::max)
                    + 1.35;
                MarginZone {
                    pts: m,
                    mn: mn - Vec3::splat(0.20),
                    mx: mx + Vec3::splat(0.20),
                    center: c,
                    radius_sq: r * r,
                }
            })
            .collect();

        let mut is_ext = vec![true; n];
        let mut thick = vec![1.5_f32; n];
        let margin_r2 = 0.18_f32 * 0.18_f32;

        for (i, &p) in bridge.positions.iter().enumerate() {
            let mut near_margin = false;
            let mut in_die_zone = false;
            for z in &zones {
                if p.distance_squared(z.center) <= z.radius_sq {
                    in_die_zone = true;
                    if p.x >= z.mn.x
                        && p.x <= z.mx.x
                        && p.y >= z.mn.y
                        && p.y <= z.mx.y
                        && p.z >= z.mn.z
                        && p.z <= z.mx.z
                        && z.pts.iter().any(|&mp| mp.distance_squared(p) < margin_r2)
                    {
                        near_margin = true;
                        break;
                    }
                }
            }
            if near_margin {
                is_ext[i] = false;
                thick[i] = 0.25;
                continue;
            }
            if in_die_zone {
                if let Some(col) = prep_col {
                    let (_, d_prep) = col.closest_point_and_dist(p);
                    thick[i] = d_prep;
                    if d_prep < 0.24 {
                        is_ext[i] = false;
                    }
                }
            }
        }

        let ext_cnt = is_ext.iter().filter(|&&b| b).count();
        if ext_cnt < n / 10 {
            bridge.is_external = vec![true; n];
        } else {
            bridge.is_external = is_ext;
        }
        bridge.thickness_mm = thick;
    }

    pub fn load_case(&mut self, case: &DentalCase, case_manager: &CaseManager, force_refuse: bool) {
        self.loaded_case_id = Some(case.case_id.clone());
        self.loaded_sig = Self::compute_case_sig(case);
        self.smart_occ_clearance_mm = case.design_params.occlusal_contact_offset_mm;
        self.undo_stack.clear();
        self.morph_session = None;
        self.morph_cursor_3d = None;
        self.is_sculpting = false;
        self.last_sculpt_pt = None;
        self.stroke_adjacency = None;
        self.sculpt_cache = None;
        self.sculpt_dab_count = 0;
        self.hover_cursor_3d = None;
        self.bridges.clear();
        self.active_bridge_idx = 0;

        self.prep_mesh = case_manager
            .get_active_scan_path(case, &case.prep_arch)
            .and_then(|p| TriMeshData::load_from_path(&p).ok())
            .map(|m| {
                let rev = self.bump();
                (m, rev)
            });

        self.ant_mesh = case_manager
            .get_active_scan_path(case, case.get_antagonist_arch())
            .and_then(|p| TriMeshData::load_from_path(&p).ok())
            .map(|m| {
                let rev = self.bump();
                (m, rev)
            });

        let prep_col = self
            .prep_mesh
            .as_ref()
            .and_then(|(m, _)| MeshCollider::from_mesh(m));

        let arch_axis = Vec3::from_array(case.insertion_axis);
        let case_dir = case_manager.get_case_dir(&case.case_id);
        let design_dir = case_dir.join("design");

        for group in case.connected_bridge_groups() {
            if group.len() < 2 {
                continue;
            }
            let mut members: Vec<(i32, TriMeshData, bool)> = Vec::new();
            let mut member_paths: Vec<PathBuf> = Vec::new();
            for &fdi in &group {
                if let Some(path) = resolve_unit_design_path(case, case_manager, fdi) {
                    if let Ok(m) = TriMeshData::load_from_path(&path) {
                        if !m.is_empty() {
                            let is_pontic = case
                                .restorations
                                .get(&fdi)
                                .map(|u| u.is_pontic())
                                .unwrap_or(false);
                            members.push((fdi, m, is_pontic));
                            member_paths.push(path);
                        }
                    }
                }
            }
            if members.len() < 2 {
                continue;
            }

            let loaded_set: BTreeSet<i32> = members.iter().map(|(f, _, _)| *f).collect();
            let (group_cfgs, connector_summary) = collect_group_connectors(case, &loaded_set);

            let (pu, pv, _pa) = build_orthonormal_basis(arch_axis);
            let mut pontic_zones: Vec<(Vec3, f32)> = Vec::new();
            for (fdi, m, is_p) in &mut members {
                if *is_p {
                    let c = m.center();
                    let max_r = m
                        .positions
                        .iter()
                        .map(|&p| {
                            let d = p - c;
                            d.dot(pu).hypot(d.dot(pv))
                        })
                        .fold(0.0_f32, f32::max);
                    pontic_zones.push((c, (max_r + 0.35).clamp(2.5, 7.5)));
                } else if m.is_external.iter().all(|&b| b) {
                    if let Some(u) = case.restorations.get(fdi) {
                        if u.margin_points.len() >= 4 {
                            let mpts: Vec<Vec3> = u.margin_points.iter().map(|p| Vec3::from_array(*p)).collect();
                            Self::classify_bridge_external_vertices_with_col(
                                m,
                                prep_col.as_ref(),
                                &[mpts],
                            );
                        }
                    }
                }
            }

            let fdis: Vec<i32> = members.iter().map(|(f, _, _)| *f).collect();
            let fdi_tag = fdis.iter().map(|d| d.to_string()).collect::<Vec<_>>().join("_");
            let fdi_label = fdis.iter().map(|d| format!("#{}", d)).collect::<Vec<_>>().join("–");

            let final_path = design_dir.join(format!("bridge_{}_final.stl", fdi_tag));
            let base_path = design_dir.join(format!("bridge_{}.stl", fdi_tag));

            let mut bridge_mesh = if !force_refuse && is_final_bridge_up_to_date(&final_path, &member_paths) {
                TriMeshData::load_from_path(&final_path).unwrap_or_default()
            } else {
                TriMeshData::default()
            };

            if bridge_mesh.is_empty() {
                let conn_refs: Vec<(&BridgeConnectorConfig, Vec3)> =
                    group_cfgs.iter().map(|c| (c, arch_axis)).collect();
                bridge_mesh = fuse_bridge_watertight(&members, &conn_refs);
                if !bridge_mesh.is_empty() {
                    let _ = std::fs::create_dir_all(&design_dir);
                    let _ = bridge_mesh.save_stl_binary(&base_path);
                    let _ = bridge_mesh.save_stl_binary(&final_path);
                }
            }
            if bridge_mesh.is_empty() {
                continue;
            }

            let abutment_margins: Vec<Vec<Vec3>> = fdis
                .iter()
                .filter_map(|fdi| {
                    let u = case.restorations.get(fdi)?;
                    if u.is_pontic() || u.margin_points.len() < 4 {
                        None
                    } else {
                        Some(u.margin_points.iter().map(|p| Vec3::from_array(*p)).collect())
                    }
                })
                .collect();

            Self::classify_bridge_external_vertices_with_col(
                &mut bridge_mesh,
                prep_col.as_ref(),
                &abutment_margins,
            );
            let col = MeshCollider::from_mesh(&bridge_mesh);
            let audit = bridge_mesh.inspect_cam();
            let rev = self.bump();

            self.bridges.push(BridgeGroupRuntime {
                fdis,
                pontic_zones,
                label: format!("{} {}", tr("Puente", "Bridge", "Ponte"), fdi_label),
                file_tag: fdi_tag,
                connector_summary,
                mesh: bridge_mesh,
                collider: col,
                rev,
                audit,
            });
        }

        if let Some(br) = self.bridges.first() {
            let (mn, mx) = br.mesh.bounds();
            self.camera.frame_bounds(mn - Vec3::splat(3.5), mx + Vec3::splat(3.5));
        } else if let Some((prep, _)) = &self.prep_mesh {
            let (mn, mx) = prep.bounds();
            self.camera.frame_bounds(mn, mx);
        }
    }

    pub fn save_all_bridges(&self, case: &DentalCase, case_manager: &CaseManager) {
        let design_dir = case_manager.get_case_dir(&case.case_id).join("design");
        let _ = std::fs::create_dir_all(&design_dir);
        for br in &self.bridges {
            let final_path = design_dir.join(format!("bridge_{}_final.stl", br.file_tag));
            let base_path = design_dir.join(format!("bridge_{}.stl", br.file_tag));
            let _ = br.mesh.save_stl_binary(&final_path);
            let _ = br.mesh.save_stl_binary(&base_path);
        }
    }

    fn push_undo(&mut self, idx: usize) {
        if let Some(br) = self.bridges.get(idx) {
            self.undo_stack.push((idx, br.mesh.clone()));
            if self.undo_stack.len() > 25 {
                self.undo_stack.remove(0);
            }
        }
    }

    pub fn undo(&mut self, case: &DentalCase, case_manager: &CaseManager) -> bool {
        let Some((idx, prev)) = self.undo_stack.pop() else {
            return false;
        };
        self.morph_session = None;
        self.is_sculpting = false;
        self.sculpt_cache = None;
        self.sculpt_dab_count = 0;
        let rev = self.bump();
        if let Some(br) = self.bridges.get_mut(idx) {
            br.mesh = prev;
            br.collider = MeshCollider::from_mesh(&br.mesh);
            br.audit = br.mesh.inspect_cam();
            br.rev = rev;
            self.save_all_bridges(case, case_manager);
            true
        } else {
            false
        }
    }

    fn set_active_bridge_mesh(&mut self, idx: usize, mesh: TriMeshData) {
        let rev = self.bump();
        if let Some(br) = self.bridges.get_mut(idx) {
            br.collider = MeshCollider::from_mesh(&mesh);
            br.audit = mesh.inspect_cam();
            br.rev = rev;
            br.mesh = mesh;
        }
    }

    fn set_active_bridge_mesh_fast(&mut self, idx: usize, mesh: TriMeshData) {
        let rev = self.bump();
        if let Some(br) = self.bridges.get_mut(idx) {
            br.rev = rev;
            br.mesh = mesh;
        }
    }

    fn refresh_active_bridge_collider_and_audit(&mut self, idx: usize, recompute_audit: bool) {
        if let Some(br) = self.bridges.get_mut(idx) {
            br.collider = MeshCollider::from_mesh(&br.mesh);
            if recompute_audit {
                br.audit = br.mesh.inspect_cam();
            }
        }
    }

    fn pick_on_active_bridge(&self, pos: Pos2, rect: egui::Rect) -> Option<Vec3> {
        if !self.vis_bridge {
            return None;
        }
        let br = self.bridges.get(self.active_bridge_idx)?;
        let col = br.collider.as_ref()?;
        let (ro, rd) = self.camera.screen_ray(pos, rect);
        col.cast_ray(ro, rd, 500.0).map(|(_, hit)| hit)
    }

    fn apply_brush_dab(&mut self, center: Vec3, case: &DentalCase) {
        let mode = match self.tool_mode {
            BridgeToolMode::Add => SculptMode::Add,
            BridgeToolMode::Remove => SculptMode::Remove,
            BridgeToolMode::Smooth => SculptMode::Smooth,
            BridgeToolMode::Morph => return,
        };
        let bi = self.active_bridge_idx;
        let Some(br) = self.bridges.get(bi) else {
            return;
        };
        let mut mesh = br.mesh.clone();
        if let Some(sc) = &self.sculpt_cache {
            CrownSculptor::apply_brush_stroke_cached(
                &mut mesh,
                center,
                mode,
                self.brush_radius_mm,
                self.brush_strength_mm,
                case.design_params.min_thickness_mm,
                sc,
            );
        } else {
            CrownSculptor::apply_brush_stroke(
                &mut mesh,
                center,
                mode,
                self.brush_radius_mm,
                self.brush_strength_mm,
                case.design_params.min_thickness_mm,
                self.stroke_adjacency.as_deref(),
            );
        }
        self.set_active_bridge_mesh_fast(bi, mesh);
    }

    pub fn ui(
        &mut self,
        ctx: &egui::Context,
        case: &mut DentalCase,
        case_manager: &CaseManager,
        renderer: &Arc<Mutex<GlViewportRenderer>>,
    ) -> BridgeSculptAction {
        let mut action = BridgeSculptAction::None;
        let win_style = |title: &str, id: &str, pos: Pos2, w: f32, open: bool| {
            Window::new(title)
                .id(egui::Id::new(id))
                .default_pos(pos)
                .default_width(w)
                .default_open(open)
                .collapsible(true)
                .resizable(false)
        };

        // ==================================================================
        // VISOR 3D FULLSCREEN DEL PUENTE UNIDO
        // ==================================================================
        egui::CentralPanel::default()
            .frame(Frame::NONE.fill(theme::BG_DEEP))
            .show(ctx, |ui| {
                let rect = ui.available_rect_before_wrap();
                let resp = ui.allocate_rect(rect, Sense::click_and_drag());

                let mut ctrl_handled_scroll = false;
                if resp.hovered() {
                    let ctrl_down = ui.input(|i| i.modifiers.ctrl);
                    let scroll = ui.input(|i| i.smooth_scroll_delta.y);
                    if scroll.abs() > 0.1 && ctrl_down {
                        if self.tool_mode == BridgeToolMode::Morph {
                            self.morph_radius_mm = (self.morph_radius_mm + scroll.signum() * 0.20).clamp(0.50, 5.50);
                        } else {
                            self.brush_radius_mm = (self.brush_radius_mm + scroll.signum() * 0.18).clamp(0.35, 4.50);
                        }
                        self.cursor_rev = self.bump();
                        ctrl_handled_scroll = true;
                    }
                }

                // Clic Izquierdo = Esculpido / Morphing libre sobre el puente unido
                let bi = self.active_bridge_idx;
                if ui.input(|i| i.pointer.primary_pressed()) && resp.hovered() {
                    if let Some(p_pos) = ui.input(|i| i.pointer.interact_pos()) {
                        if let Some(picked) = self.pick_on_active_bridge(p_pos, rect) {
                            match self.tool_mode {
                                BridgeToolMode::Morph => {
                                    if let Some(sess) = self.bridges.get(bi).map(|b| {
                                        CrownSculptor::prepare_morph_session(
                                            &b.mesh,
                                            picked,
                                            self.morph_radius_mm,
                                        )
                                    }) {
                                        self.push_undo(bi);
                                        self.drag_start_pos = p_pos;
                                        self.morph_session = Some(sess);
                                        self.morph_cursor_3d = Some(picked);
                                        self.cursor_rev = self.bump();
                                    }
                                }
                                BridgeToolMode::Add | BridgeToolMode::Remove | BridgeToolMode::Smooth => {
                                    let sc = self
                                        .bridges
                                        .get(bi)
                                        .map(|b| CrownSculptor::prepare_sculpt_cache(&b.mesh));
                                    self.push_undo(bi);
                                    self.is_sculpting = true;
                                    self.last_sculpt_pt = Some(picked);
                                    self.sculpt_dab_count = 0;
                                    self.sculpt_cache = sc;
                                    self.apply_brush_dab(picked, case);
                                }
                            }
                        }
                    }
                }

                if ui.input(|i| i.pointer.primary_down()) {
                    if let Some(curr_pos) = ui.input(|i| i.pointer.interact_pos()) {
                        if self.morph_session.is_some() {
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
                                self.set_active_bridge_mesh_fast(bi, deformed);
                            }
                        } else if self.is_sculpting {
                            if let Some(picked) = self.pick_on_active_bridge(curr_pos, rect) {
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
                                        self.refresh_active_bridge_collider_and_audit(bi, false);
                                    }
                                }
                            }
                        }
                    }
                } else if let Some(hp) = resp.hover_pos() {
                    let prev_c = self.hover_cursor_3d;
                    self.hover_cursor_3d = self.pick_on_active_bridge(hp, rect);
                    if let Some(nc) = self.hover_cursor_3d {
                        let moved = prev_c
                            .map(|pc| pc.distance_squared(nc) > 0.005 * 0.005)
                            .unwrap_or(true);
                        if moved {
                            self.cursor_rev = self.bump();
                        }
                    }
                }

                self.camera.handle_navigation_input(&resp, ui, rect, !ctrl_handled_scroll);

                if ui.input(|i| i.pointer.primary_released()) {
                    if self.morph_session.take().is_some() {
                        self.morph_cursor_3d = None;
                        self.refresh_active_bridge_collider_and_audit(bi, true);
                        self.save_all_bridges(case, case_manager);
                    }
                    if self.is_sculpting {
                        self.is_sculpting = false;
                        self.last_sculpt_pt = None;
                        self.stroke_adjacency = None;
                        self.sculpt_cache = None;
                        self.sculpt_dab_count = 0;
                        self.refresh_active_bridge_collider_and_audit(bi, true);
                        self.save_all_bridges(case, case_manager);
                    }
                }

                let mut layers = Vec::new();
                if let Ok(mut guard) = renderer.lock() {
                    if self.vis_model {
                        if let Some((prep, rev)) = &self.prep_mesh {
                            guard.ensure_mesh_uploaded("bs_prep", *rev, prep, ColorMapKind::Solid);
                            let mut sp = RenderLayerSpec::solid("bs_prep", *rev, [0.90, 0.84, 0.75], self.op_model);
                            sp.specular = 0.25;
                            layers.push(sp);
                        }
                    }
                    if self.vis_antagonist {
                        if let Some((ant, rev)) = &self.ant_mesh {
                            guard.ensure_mesh_uploaded("bs_ant", *rev, ant, ColorMapKind::Solid);
                            layers.push(RenderLayerSpec::solid("bs_ant", *rev, [0.85, 0.80, 0.89], self.op_antagonist));
                        }
                    }
                    if self.vis_margins {
                        let has_margins = case
                            .restorations
                            .values()
                            .any(|u| !u.is_pontic() && u.margin_points.len() >= 4);
                        if has_margins {
                            let m_rev = (self.loaded_sig ^ (self.margin_line_radius_mm.to_bits() as u64)).max(1);
                            if !guard.is_mesh_current("bs_margins", m_rev) {
                                let mut m_tube = TriMeshData::default();
                                for u in case.restorations.values() {
                                    if !u.is_pontic() && u.margin_points.len() >= 4 {
                                        let pts: Vec<Vec3> = u.margin_points.iter().map(|p| Vec3::from_array(*p)).collect();
                                        m_tube.merge_in_place(&build_tube_mesh(&pts, self.margin_line_radius_mm, true, 8));
                                    }
                                }
                                guard.ensure_mesh_uploaded("bs_margins", m_rev, &m_tube, ColorMapKind::Solid);
                            }
                            let mut sp = RenderLayerSpec::solid("bs_margins", m_rev, [0.20, 0.85, 0.62], 1.0);
                            sp.clip_affected = false;
                            layers.push(sp);
                        }
                    }
                    if self.vis_bridge {
                        for (idx, br) in self.bridges.iter().enumerate() {
                            let is_act = idx == self.active_bridge_idx;
                            let (cmap, key, use_vc) = if is_act {
                                match self.color_mode {
                                    CrownColorMode::Ceramic => (ColorMapKind::Solid, format!("bs_br_{}", idx), false),
                                    CrownColorMode::OcclusalHeatmap => (ColorMapKind::OcclusalJetR, format!("bs_br_occ_{}", idx), true),
                                    CrownColorMode::ThicknessHeatmap => (ColorMapKind::ThicknessViridis, format!("bs_br_th_{}", idx), true),
                                }
                            } else {
                                (ColorMapKind::Solid, format!("bs_br_{}", idx), false)
                            };
                            guard.ensure_mesh_uploaded(&key, br.rev, &br.mesh, cmap);
                            let tint = if is_act { [0.99, 0.97, 0.93] } else { [0.92, 0.89, 0.82] };
                            let mut sp = RenderLayerSpec::solid(key, br.rev, tint, self.op_bridge);
                            sp.use_vertex_color = use_vc;
                            sp.specular = 0.58;
                            sp.shininess = 38.0;
                            layers.push(sp);
                        }
                    }

                    if self.vis_bridge {
                        if let Some(cp) = self.morph_cursor_3d.or(self.hover_cursor_3d) {
                            let (rad, col) = match self.tool_mode {
                                BridgeToolMode::Morph => (self.morph_radius_mm * 0.36, [0.78, 0.49, 1.0]),
                                BridgeToolMode::Add => (self.brush_radius_mm * 0.36, [0.20, 0.83, 0.60]),
                                BridgeToolMode::Remove => (self.brush_radius_mm * 0.36, [0.96, 0.25, 0.37]),
                                BridgeToolMode::Smooth => (self.brush_radius_mm * 0.36, [0.22, 0.74, 0.97]),
                            };
                            if !guard.is_mesh_current("bs_cursor", self.cursor_rev) {
                                let sph = build_uv_sphere(cp, rad, 12, 16);
                                guard.ensure_mesh_uploaded("bs_cursor", self.cursor_rev, &sph, ColorMapKind::Solid);
                            }
                            let mut sp = RenderLayerSpec::solid("bs_cursor", self.cursor_rev, col, 0.38);
                            sp.clip_affected = false;
                            layers.push(sp);
                        }
                    }
                }

                let clip_plane = if self.cross_section {
                    let orig = self
                        .bridges
                        .get(self.active_bridge_idx)
                        .map(|b| b.mesh.center())
                        .unwrap_or(Vec3::ZERO);
                    Some(Vec4::new(1.0, 0.0, 0.0, -orig.x))
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
            });

        // Barra superior de vistas de cámara
        Area::new(egui::Id::new("bs_cam_bar"))
            .anchor(Align2::CENTER_TOP, egui::vec2(25.0, 68.0))
            .order(Order::Foreground)
            .show(ctx, |ui| {
                Frame::NONE
                    .fill(egui::Color32::from_rgba_unmultiplied(0x21, 0x05, 0x2C, 238))
                    .stroke(Stroke::new(1.3_f32, AMETHYST))
                    .corner_radius(CornerRadius::same(8))
                    .inner_margin(Margin::symmetric(8, 5))
                    .show(ui, |ui| {
                        ui.horizontal(|ui| {
                            if ui.button(tr("Oclusal", "Occlusal", "Oclusal")).clicked() { self.camera.set_view_occlusal_top(); }
                            if ui.button("Intaglio").clicked() { self.camera.set_view_occlusal_bottom(); }
                            if ui.button(tr("Vestibular", "Buccal", "Vestibular")).clicked() { self.camera.set_view_frontal(); }
                            if ui.button("Lingual").clicked() { self.camera.set_view_lingual(); }
                            if ui.button(tr("Isométrica", "Isometric", "Isométrica")).clicked() { self.camera.set_view_isometric(); }
                            if ui.button(tr("Centrar Puente", "Center Bridge", "Centralizar Ponte")).clicked() {
                                if let Some(b) = self.bridges.get(self.active_bridge_idx) {
                                    let (mn, mx) = b.mesh.bounds();
                                    self.camera.frame_bounds(mn - Vec3::splat(3.5), mx + Vec3::splat(3.5));
                                }
                            }
                        });
                    });
            });

        // ==================================================================
        // TARJETA IZQ 1: Puente Unido y Auditoría
        // ==================================================================
        win_style(tr("1. Puente Unido (Pieza Única)", "1. United Bridge (Single Piece)", "1. Ponte Unida (Peça Única)"), "bs_card_info", egui::pos2(14.0, 72.0), 335.0, true)
            .show(ctx, |ui| {
                if self.bridges.len() > 1 {
                    ui.horizontal_wrapped(|ui| {
                        for (idx, br) in self.bridges.iter().enumerate() {
                            if ui.selectable_label(self.active_bridge_idx == idx, &br.label).clicked() {
                                self.active_bridge_idx = idx;
                            }
                        }
                    });
                    ui.separator();
                }

                if let Some(br) = self.bridges.get(self.active_bridge_idx) {
                    ui.label(RichText::new(&br.label).strong().size(13.0).color(CYAN));
                    let wt_ok = br.audit.watertight && br.audit.folded_edges == 0;
                    ui.label(
                        RichText::new(format!(
                            "{} | {} tris | {:.1} cm³ | {}: {}",
                            if br.audit.watertight { "✔ Watertight" } else { tr("⚠ Revisar borde", "⚠ Check border", "⚠ Verificar borda") },
                            br.audit.n_faces,
                            br.audit.volume_mm3 / 1000.0,
                            tr("Pliegues", "Folds", "Dobras"),
                            br.audit.folded_edges
                        ))
                        .size(10.8)
                        .color(if wt_ok { EMERALD } else { AMBER }),
                    );
                    if !br.connector_summary.is_empty() {
                        ui.add_space(2.0);
                        for line in &br.connector_summary {
                            ui.label(RichText::new(format!("• {}", line)).size(10.2).color(CREAM_MUTED));
                        }
                    }
                } else {
                    ui.label(RichText::new(tr("⚠ No hay puentes fusionados activos.", "⚠ No active fused bridges.", "⚠ Nenhuma ponte fusionada ativa.")).color(AMBER));
                }

                ui.add_space(4.0);
                ui.horizontal(|ui| {
                    if ui.button(tr("↻ Re-fusionar desde Diseño", "↻ Re-fuse from Design", "↻ Re-fusionar do Desenho")).clicked() {
                        self.load_case(case, case_manager, true);
                        action = BridgeSculptAction::StatusMessage(
                            tr(
                                "Puente re-fusionado en una sola pieza (conectores + contactos interproximales).",
                                "Bridge cleanly re-fused into one piece (connectors + interproximal contacts).",
                                "Ponte re-fusionada em uma única peça (conectores + contatos interproximais).",
                            ).to_string(),
                        );
                    }
                    if ui.button(tr("← Volver a Diseño", "← Back to Design", "← Voltar ao Desenho")).clicked() {
                        self.save_all_bridges(case, case_manager);
                        action = BridgeSculptAction::ReturnToDesign;
                    }
                });
            });

        // ==================================================================
        // TARJETA IZQ 2: Transformación Libre y Esculpido del Puente
        // ==================================================================
        win_style(tr("2. Transformación Libre y Esculpido", "2. Free Transform & Sculpting", "2. Transformação Livre e Escultura"), "bs_card_tools", egui::pos2(14.0, 278.0), 335.0, true)
            .show(ctx, |ui| {
                ui.horizontal_wrapped(|ui| {
                    for (mode, lbl) in [
                        (BridgeToolMode::Morph, tr("✥ Morphing 3D", "✥ Morphing 3D", "✥ Morphing 3D")),
                        (BridgeToolMode::Add, tr("➕ Agregar", "➕ Add", "➕ Adicionar")),
                        (BridgeToolMode::Remove, tr("➖ Quitar", "➖ Remove", "➖ Remover")),
                        (BridgeToolMode::Smooth, tr("〰 Suavizar", "〰 Smooth", "〰 Suavizar")),
                    ] {
                        if ui.selectable_label(self.tool_mode == mode, lbl).clicked() {
                            self.tool_mode = mode;
                        }
                    }
                });

                ui.separator();
                match self.tool_mode {
                    BridgeToolMode::Morph => {
                        ui.horizontal(|ui| {
                            ui.label(RichText::new(tr("Zona Morphing:", "Morph Zone:", "Zona Morphing:")).strong().size(11.0));
                            ui.add(egui::Slider::new(&mut self.morph_radius_mm, 0.50..=5.50).show_value(false));
                            ui.label(RichText::new(format!("{:.2} mm", self.morph_radius_mm)).strong().color(AMETHYST_BRIGHT));
                        });
                        ui.checkbox(&mut self.morph_normal_only, tr("Restringir a normal de superficie", "Constrain to surface normal", "Restringir à normal da superfície"));
                        ui.label(
                            RichText::new(tr(
                                "Clic Izq y arrastra cualquier cúspide, póntico o conector unido (Ctrl + Rueda ajusta radio).",
                                "Left-drag any cusp, pontic or united connector (Ctrl + Wheel adjusts radius).",
                                "Clique Esq e arraste qualquer cúspide, pôntico ou conector unido (Ctrl + Roda ajusta o raio).",
                            ))
                            .size(10.0)
                            .color(CREAM_MUTED),
                        );
                    }
                    BridgeToolMode::Add | BridgeToolMode::Remove | BridgeToolMode::Smooth => {
                        ui.horizontal(|ui| {
                            ui.label(RichText::new(tr("Radio pincel:", "Brush radius:", "Raio pincel:")).strong().size(11.0));
                            ui.add(egui::Slider::new(&mut self.brush_radius_mm, 0.35..=4.50).show_value(false));
                            ui.label(RichText::new(format!("{:.2} mm", self.brush_radius_mm)).strong().color(EMERALD));
                        });
                        ui.horizontal(|ui| {
                            ui.label(RichText::new(tr("Intensidad:", "Strength:", "Intensidade:")).strong().size(11.0));
                            ui.add(egui::Slider::new(&mut self.brush_strength_mm, 0.03..=0.45).show_value(false));
                            ui.label(RichText::new(format!("{:.2} mm", self.brush_strength_mm)).strong().color(CREAM));
                        });
                    }
                }

                ui.add_space(4.0);
                let can_undo = !self.undo_stack.is_empty();
                if ui.add_enabled(can_undo, egui::Button::new(tr("↶ Deshacer (Ctrl+Z)", "↶ Undo (Ctrl+Z)", "↶ Desfazer (Ctrl+Z)"))).clicked() {
                    self.undo(case, case_manager);
                }
            });

        // ==================================================================
        // TARJETA IZQ 3: Herramientas Inteligentes del Puente Unido
        // ==================================================================
        win_style(tr("3. Herramientas Inteligentes (Puente)", "3. Smart Tools (Bridge)", "3. Ferramentas Inteligentes (Ponte)"), "bs_card_smart", egui::pos2(14.0, 495.0), 335.0, true)
            .show(ctx, |ui| {
                ui.horizontal(|ui| {
                    ui.label(RichText::new(tr("Claro Oclusal:", "Occlusal Clearance:", "Espaço Oclusal:")).size(10.5).strong());
                    ui.add(egui::Slider::new(&mut self.smart_occ_clearance_mm, -0.15..=0.50).show_value(false));
                    ui.label(RichText::new(format!("{:+.2} mm", self.smart_occ_clearance_mm)).size(10.5).color(EMERALD).strong());
                });

                let bi = self.active_bridge_idx;
                if ui.add_sized([ui.available_width(), 24.0], egui::Button::new(tr("⚡ Adaptar Oclusión al Antagonista", "⚡ Adapt Occlusion to Antagonist", "⚡ Adaptar Oclusão ao Antagonista"))).clicked() {
                    if let (Some(br_m), Some((ant, _))) = (
                        self.bridges.get(bi).map(|b| b.mesh.clone()),
                        self.ant_mesh.as_ref(),
                    ) {
                        let res = SmartToolsEngine::smart_occlusal_contacts(
                            &br_m,
                            ant,
                            Vec3::from_array(case.insertion_axis),
                            self.smart_occ_clearance_mm,
                            true,
                            case.design_params.min_thickness_mm,
                        );
                        self.push_undo(bi);
                        self.set_active_bridge_mesh(bi, res);
                        self.save_all_bridges(case, case_manager);
                        action = BridgeSculptAction::StatusMessage(tr(
                            "Oclusión del puente adaptada al antagonista.",
                            "Bridge occlusion adapted to opposing arch.",
                            "Oclusão da ponte adaptada ao antagonista.",
                        ).to_string());
                    } else {
                        action = BridgeSculptAction::StatusMessage(tr(
                            "Importa o activa un antagonista para ajustar oclusión.",
                            "Import or enable an opposing arch scan first.",
                            "Importe ou ative um antagonista primeiro.",
                        ).to_string());
                    }
                }

                ui.add_space(3.0);
                ui.horizontal(|ui| {
                    ui.label(RichText::new(tr("Espacio Gingival:", "Gingival Offset:", "Espaço Gengival:")).size(10.5).strong());
                    ui.add(egui::Slider::new(&mut self.smart_gingival_offset_mm, -0.50..=0.50).show_value(false));
                    ui.label(RichText::new(format!("{:+.2} mm", self.smart_gingival_offset_mm)).size(10.5).color(CYAN).strong());
                });
                if ui.add_sized([ui.available_width(), 24.0], egui::Button::new(tr("🦷 Adaptar Póntico a la Encía", "🦷 Adapt Pontic to Gingiva", "🦷 Adaptar Pôntico à Gengiva"))).clicked() {
                    let zones_opt = self.bridges.get(bi).map(|b| b.pontic_zones.clone());
                    if let (Some(br_m), Some((prep, _)), Some(zones)) = (
                        self.bridges.get(bi).map(|b| b.mesh.clone()),
                        self.prep_mesh.as_ref(),
                        zones_opt,
                    ) {
                        if zones.is_empty() {
                            action = BridgeSculptAction::StatusMessage(tr(
                                "Este puente no contiene piezas marcadas como Póntico.",
                                "This bridge contains no units marked as Pontic.",
                                "Esta ponte não contém dentes definidos como Pôntico.",
                            ).to_string());
                        } else {
                            let res = SmartToolsEngine::smart_adapt_pontic_to_gingiva(
                                &br_m,
                                prep,
                                Vec3::from_array(case.insertion_axis),
                                self.smart_gingival_offset_mm,
                                Some(&zones),
                            );
                            self.push_undo(bi);
                            self.set_active_bridge_mesh(bi, res);
                            self.save_all_bridges(case, case_manager);
                            action = BridgeSculptAction::StatusMessage(tr(
                                "Zona basal del póntico reducida y adaptada a la encía sin alterar los pilares.",
                                "Pontic basal area reduced and adapted to gingiva while keeping abutments intact.",
                                "Base do pôntico reduzida e adaptada à gengiva preservando os pilares.",
                            ).to_string());
                        }
                    } else {
                        action = BridgeSculptAction::StatusMessage(tr(
                            "No hay malla de modelo/encía cargada para adaptar el póntico.",
                            "No master model / gingiva mesh loaded to adapt the pontic.",
                            "Nenhuma malha de modelo/gengiva carregada para adaptar o pôntico.",
                        ).to_string());
                    }
                }

                ui.add_space(3.0);
                ui.horizontal(|ui| {
                    if ui.button(tr("🛡 Grosor Mínimo", "🛡 Min. Thickness", "🛡 Espessura Mínima")).clicked() {
                        if let Some(br_m) = self.bridges.get(bi).map(|b| b.mesh.clone()) {
                            self.push_undo(bi);
                            let res = SmartToolsEngine::smart_enforce_min_thickness(
                                &br_m,
                                case.design_params.min_thickness_mm,
                            );
                            self.set_active_bridge_mesh(bi, res);
                            self.save_all_bridges(case, case_manager);
                            action = BridgeSculptAction::StatusMessage(tr(
                                "Grosor mínimo reforzado en todo el puente.",
                                "Minimum wall thickness enforced across the entire bridge.",
                                "Espessura mínima reforçada em toda a ponte.",
                            ).to_string());
                        }
                    }
                    if ui.button(tr("✨ Suavizar Uniones", "✨ Smooth Junctions", "✨ Suavizar Uniões")).clicked() {
                        if let Some(br_m) = self.bridges.get(bi).map(|b| b.mesh.clone()) {
                            self.push_undo(bi);
                            let res = SmartToolsEngine::smart_anatomical_smooth(
                                &br_m,
                                4,
                                case.design_params.min_thickness_mm,
                            );
                            self.set_active_bridge_mesh(bi, res);
                            self.save_all_bridges(case, case_manager);
                            action = BridgeSculptAction::StatusMessage(tr(
                                "Uniones y superficie del puente suavizadas sin perder sellado.",
                                "Bridge junctions and surface smoothed while preserving marginal seal.",
                                "Uniões e superfície da ponte suavizadas preservando o selamento.",
                            ).to_string());
                        }
                    }
                });

                ui.separator();
                ui.label(RichText::new(tr("Mapa de Calor 3D:", "3D Heatmap:", "Mapa Térmico 3D:")).size(10.5).strong());
                ui.horizontal(|ui| {
                    ui.radio_value(&mut self.color_mode, CrownColorMode::Ceramic, tr("Cerámica", "Ceramic", "Cerâmica"));
                    ui.radio_value(&mut self.color_mode, CrownColorMode::OcclusalHeatmap, tr("Oclusión", "Occlusion", "Oclusão"));
                    ui.radio_value(&mut self.color_mode, CrownColorMode::ThicknessHeatmap, tr("Espesor", "Thickness", "Espessura"));
                });
            });

        // ==================================================================
        // TARJETA SUP. DERECHA: Transparencias 3D
        // ==================================================================
        let right_x = (ctx.screen_rect().width() - 325.0).max(360.0);
        win_style(tr("Menú de Transparencias 3D", "3D Visibility & Opacity", "Menu de Transparências 3D"), "bs_card_vis", egui::pos2(right_x, 72.0), 310.0, true)
            .show(ctx, |ui| {
                let mut layer_row = |vis: &mut bool, op: &mut f32, lbl: &str| {
                    ui.horizontal(|ui| {
                        ui.checkbox(vis, lbl);
                        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                            ui.add(
                                egui::Slider::new(op, 0.05..=1.0)
                                    .custom_formatter(|v, _| format!("{:.0}%", v * 100.0))
                                    .show_value(true),
                            );
                        });
                    });
                };
                layer_row(&mut self.vis_bridge, &mut self.op_bridge, tr("Puente Unido", "United Bridge", "Ponte Unida"));
                layer_row(&mut self.vis_model, &mut self.op_model, tr("Modelo / Muñones", "Master Model / Dies", "Modelo / Preparos"));
                layer_row(&mut self.vis_antagonist, &mut self.op_antagonist, tr("Antagonista", "Opposing Arch", "Antagonista"));
                ui.checkbox(&mut self.vis_margins, tr("Líneas de Margen Cervical", "Cervical Margin Lines", "Linhas de Término Cervical"));
                ui.checkbox(&mut self.cross_section, tr("Corte Transversal 50% (GPU)", "50% Cross-Section (GPU)", "Corte Transversal 50% (GPU)"));
            });

        // ==================================================================
        // BOTÓN PRINCIPAL FLOTANTE INF. DERECHA: Exportar Puente Final
        // ==================================================================
        let has_br = !self.bridges.is_empty();
        Area::new(egui::Id::new("bs_next_cta"))
            .anchor(Align2::RIGHT_BOTTOM, egui::vec2(-16.0, -34.0))
            .order(Order::Foreground)
            .show(ctx, |ui| {
                Frame::NONE
                    .fill(egui::Color32::from_rgba_unmultiplied(0x21, 0x05, 0x2C, 240))
                    .stroke(Stroke::new(1.5_f32, if has_br { EMERALD } else { Color32::from_rgb(0xFB, 0x71, 0x85) }))
                    .corner_radius(CornerRadius::same(8))
                    .inner_margin(Margin::same(6))
                    .show(ui, |ui| {
                        ui.add_enabled_ui(has_br, |ui| {
                            if theme::primary_button(ui, tr("Validar CAM y Exportar STL Final  →", "CAM Check & Export Final STL  →", "Validar CAM e Exportar STL Final  →")).clicked() {
                                self.save_all_bridges(case, case_manager);
                                action = BridgeSculptAction::ProceedToExport;
                            }
                        });
                    });
            });

        action
    }
}
