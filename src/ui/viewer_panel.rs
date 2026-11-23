//! Módulo 2 (UI + Visor 3D OpenGL): Preparación de Modelo, Limpieza Taubin,
//! Recorte 3D Múltiple con Plano Visible, Zócalo Sólido, Hollow Imprimible (1–3 mm) y Socavados.

use crate::core::case_manager::{CaseManager, DentalCase};
use crate::core::i18n::tr;
use crate::geometry::mesh::TriMeshData;
use crate::geometry::mesh_ops::{
    build_clip_box_wireframe, build_clip_plane_visuals, clean_mesh, clip_by_box, clip_by_plane,
    compute_undercut_angles_deg, create_hollow_base_checked, create_solid_base,
};
use crate::render::camera::OrbitCamera;
use crate::render::gl_renderer::{
    make_viewport_paint_callback, ColorMapKind, GlViewportRenderer, RenderLayerSpec,
};
use crate::theme::{self, AMETHYST, CREAM_MUTED, EMERALD};
use egui::{Align2, Area, CornerRadius, Frame, Margin, Order, RichText, Sense, Stroke, Window};
use glam::{Vec3, Vec4};
use std::collections::BTreeMap;
use std::sync::{Arc, Mutex};

pub enum ViewerAction {
    None,
    ProceedToMargin,
    StatusMessage(String),
}

pub struct ViewerPanelState {
    pub camera: OrbitCamera,
    pub meshes: BTreeMap<String, (TriMeshData, u64)>,
    pub next_rev: u64,
    pub clip_vis_rev: u64,
    pub active_arch: String,

    pub remove_islands: bool,
    pub taubin_iters: usize,

    pub clip_plane_enabled: bool,
    pub clip_box_enabled: bool,
    pub clip_invert: bool,
    pub plane_offset: f32,
    pub plane_tilt_x: f32,
    pub plane_tilt_y: f32,
    pub box_min: Vec3,
    pub box_max: Vec3,

    pub base_height_mm: f32,
    pub base_down: bool,
    pub hollow_wall_mm: f32,

    pub show_undercuts: bool,
    pub wireframe: bool,
    pub visible: BTreeMap<String, bool>,
    pub opacity: BTreeMap<String, f32>,
    pub mesh_watertight: BTreeMap<String, bool>,

    pub undo_stack: Vec<(String, TriMeshData, [f32; 3])>,
}

impl Default for ViewerPanelState {
    fn default() -> Self {
        let mut visible = BTreeMap::new();
        let mut opacity = BTreeMap::new();
        for (k, op) in [("mandible", 1.0), ("maxilla", 0.75), ("bite", 0.65)] {
            visible.insert(k.to_string(), true);
            opacity.insert(k.to_string(), op);
        }
        Self {
            camera: OrbitCamera::default(),
            meshes: BTreeMap::new(),
            next_rev: 1,
            clip_vis_rev: 1,
            active_arch: "mandible".to_string(),
            remove_islands: true,
            taubin_iters: 6,
            clip_plane_enabled: false,
            clip_box_enabled: false,
            clip_invert: false,
            plane_offset: 0.0,
            plane_tilt_x: 0.0,
            plane_tilt_y: 0.0,
            box_min: Vec3::splat(-20.0),
            box_max: Vec3::splat(20.0),
            base_height_mm: 4.5,
            base_down: true,
            hollow_wall_mm: 2.0,
            show_undercuts: false,
            wireframe: false,
            visible,
            opacity,
            mesh_watertight: BTreeMap::new(),
            undo_stack: Vec::new(),
        }
    }
}

impl ViewerPanelState {
    fn bump_rev(&mut self) -> u64 {
        self.next_rev += 1;
        self.next_rev
    }

    fn insert_mesh(&mut self, key: &str, mesh: TriMeshData) {
        let watertight = mesh.inspect_cam().watertight;
        self.mesh_watertight.insert(key.to_string(), watertight);
        let rev = self.bump_rev();
        self.meshes.insert(key.to_string(), (mesh, rev));
    }

    pub fn load_case(&mut self, case: &DentalCase, case_manager: &CaseManager) {
        self.meshes.clear();
        self.mesh_watertight.clear();
        self.undo_stack.clear();
        self.clip_plane_enabled = false;
        self.clip_box_enabled = false;

        let mut overall_min = Vec3::splat(f32::INFINITY);
        let mut overall_max = Vec3::splat(f32::NEG_INFINITY);

        for scan_key in ["mandible", "maxilla", "bite"] {
            if let Some(path) = case_manager.get_active_scan_path(case, scan_key) {
                if let Ok(mut mesh) = TriMeshData::load_from_path(&path) {
                    if scan_key == case.prep_arch {
                        mesh.custom_scalars = compute_undercut_angles_deg(
                            &mesh,
                            Vec3::from_array(case.insertion_axis),
                        );
                    }
                    let (mn, mx) = mesh.bounds();
                    overall_min = overall_min.min(mn);
                    overall_max = overall_max.max(mx);
                    self.insert_mesh(scan_key, mesh);
                }
            }
        }

        if self.meshes.contains_key(&case.prep_arch) {
            self.active_arch = case.prep_arch.clone();
        } else if let Some(first) = self.meshes.keys().next().cloned() {
            self.active_arch = first;
        }
        self.base_down = self.active_arch != "maxilla";

        if overall_min.x.is_finite() {
            self.camera.frame_bounds(overall_min, overall_max);
            self.reset_clip_bounds();
        }
    }

    fn reset_clip_bounds(&mut self) {
        self.plane_offset = 0.0;
        self.plane_tilt_x = 0.0;
        self.plane_tilt_y = 0.0;
        if let Some((m, _)) = self.meshes.get(&self.active_arch) {
            let (mn, mx) = m.bounds();
            self.box_min = mn;
            self.box_max = mx;
        }
        self.clip_vis_rev = self.bump_rev();
    }

    fn reset_clip_offset_only(&mut self) {
        self.plane_offset = 0.0;
        if let Some((m, _)) = self.meshes.get(&self.active_arch) {
            let (mn, mx) = m.bounds();
            self.box_min = mn;
            self.box_max = mx;
        }
        self.clip_vis_rev = self.bump_rev();
    }

    fn switch_active_arch(&mut self, arch: &str) {
        self.active_arch = arch.to_string();
        self.base_down = self.active_arch != "maxilla";
        self.reset_clip_offset_only();
    }

    fn push_undo(&mut self, key: &str, case: &DentalCase) {
        if let Some((m, _)) = self.meshes.get(key) {
            self.undo_stack.push((key.to_string(), m.clone(), case.insertion_axis));
            if self.undo_stack.len() > 15 {
                self.undo_stack.remove(0);
            }
        }
    }

    pub fn undo(&mut self, case: &mut DentalCase, case_manager: &CaseManager) -> bool {
        let Some((key, prev_mesh, prev_axis)) = self.undo_stack.pop() else {
            return false;
        };
        case.insertion_axis = prev_axis;
        self.insert_mesh(&key, prev_mesh);
        self.save_working_mesh(&key, case, case_manager);
        self.reset_clip_offset_only();
        true
    }

    fn save_working_mesh(&self, key: &str, case: &mut DentalCase, case_manager: &CaseManager) {
        if let Some((mesh, _)) = self.meshes.get(key) {
            let case_dir = case_manager.get_case_dir(&case.case_id);
            let work_rel = format!("working/{}_work.stl", key);
            let work_path = case_dir.join(&work_rel);
            if mesh.save_stl_binary(&work_path).is_ok() {
                case.scans_working.insert(key.to_string(), Some(work_rel));
                let _ = case_manager.save_case(case);
            }
        }
    }

    fn active_plane_normal(&self) -> Vec3 {
        let rx = self.plane_tilt_x.to_radians();
        let ry = self.plane_tilt_y.to_radians();
        (glam::Mat3::from_rotation_y(ry) * glam::Mat3::from_rotation_x(rx) * Vec3::Z).normalize_or_zero()
    }

    fn active_plane_origin_and_normal(&self) -> (Vec3, Vec3, f32) {
        let n = self.active_plane_normal();
        let (center, half_diag) = self
            .meshes
            .get(&self.active_arch)
            .map(|(m, _)| {
                let (mn, mx) = m.bounds();
                (0.5 * (mn + mx), (0.5 * (mx - mn).length()).max(16.0))
            })
            .unwrap_or((Vec3::ZERO, 24.0));
        let orig = center + n * self.plane_offset;
        (orig, n, half_diag)
    }

    pub fn ui(
        &mut self,
        ctx: &egui::Context,
        case: &mut DentalCase,
        case_manager: &CaseManager,
        renderer: &Arc<Mutex<GlViewportRenderer>>,
    ) -> ViewerAction {
        let mut action = ViewerAction::None;

        // ==================================================================
        // LIENZO 3D A PANTALLA COMPLETA (OpenGL 3.3 Glow)
        // ==================================================================
        egui::CentralPanel::default()
            .frame(Frame::NONE.fill(theme::BG_DEEP))
            .show(ctx, |ui| {
                let rect = ui.available_rect_before_wrap();
                let resp = ui.allocate_rect(rect, Sense::click_and_drag());

                self.camera.handle_navigation_input(&resp, ui, rect, true);

                // Preparar capas para GPU
                let mut layers = Vec::new();
                let colors: [(&str, [f32; 3]); 3] = [
                    ("mandible", [0.94, 0.89, 0.81]),
                    ("maxilla", [0.85, 0.80, 0.89]),
                    ("bite", [0.78, 0.49, 1.0]),
                ];

                if let Ok(mut guard) = renderer.lock() {
                    for (k, tint) in colors {
                        if !self.visible.get(k).copied().unwrap_or(true) {
                            continue;
                        }
                        if let Some((mesh, rev)) = self.meshes.get(k) {
                            let is_prep = k == case.prep_arch;
                            let use_undercut = self.show_undercuts && is_prep && !mesh.custom_scalars.is_empty();
                            let cmap = if use_undercut {
                                ColorMapKind::UndercutRdYlGn
                            } else {
                                ColorMapKind::Solid
                            };
                            let gpu_key = format!("v2_{}_{}", k, if use_undercut { "uc" } else { "s" });
                            guard.ensure_mesh_uploaded(&gpu_key, *rev, mesh, cmap);
                            let mut spec = RenderLayerSpec::solid(
                                gpu_key,
                                *rev,
                                tint,
                                self.opacity.get(k).copied().unwrap_or(1.0),
                            );
                            spec.wireframe = self.wireframe;
                            spec.use_vertex_color = use_undercut;
                            spec.specular = 0.28;
                            spec.shininess = 22.0;
                            // Solo aplicar el plano de recorte GPU al arco seleccionado para recortar
                            spec.clip_affected = self.clip_plane_enabled && (k == self.active_arch);
                            layers.push(spec);
                        }
                    }

                    // Renderizar plano semitransparente visible con borde y flecha direccional en OpenGL
                    if self.clip_plane_enabled {
                        let (orig, n, half_diag) = self.active_plane_origin_and_normal();
                        let kept_n = if self.clip_invert { -n } else { n };
                        let (plane_fill, plane_border) =
                            build_clip_plane_visuals(orig, n, kept_n, half_diag * 1.15);

                        guard.ensure_mesh_uploaded(
                            "v2_clip_plane_fill",
                            self.clip_vis_rev,
                            &plane_fill,
                            ColorMapKind::Solid,
                        );
                        let mut sp_fill = RenderLayerSpec::solid(
                            "v2_clip_plane_fill",
                            self.clip_vis_rev,
                            [0.18, 0.86, 0.96],
                            0.26,
                        );
                        sp_fill.unlit = true;
                        sp_fill.clip_affected = false;
                        layers.push(sp_fill);

                        guard.ensure_mesh_uploaded(
                            "v2_clip_plane_border",
                            self.clip_vis_rev,
                            &plane_border,
                            ColorMapKind::Solid,
                        );
                        let mut sp_bd = RenderLayerSpec::solid(
                            "v2_clip_plane_border",
                            self.clip_vis_rev,
                            [0.20, 0.95, 0.68],
                            1.0,
                        );
                        sp_bd.unlit = true;
                        sp_bd.clip_affected = false;
                        layers.push(sp_bd);
                    } else if self.clip_box_enabled {
                        let box_mesh = build_clip_box_wireframe(self.box_min, self.box_max);
                        guard.ensure_mesh_uploaded(
                            "v2_clip_box",
                            self.clip_vis_rev,
                            &box_mesh,
                            ColorMapKind::Solid,
                        );
                        let mut sp_bx = RenderLayerSpec::solid(
                            "v2_clip_box",
                            self.clip_vis_rev,
                            [0.20, 0.95, 0.68],
                            1.0,
                        );
                        sp_bx.unlit = true;
                        sp_bx.clip_affected = false;
                        layers.push(sp_bx);
                    }
                }

                // Vista previa del plano de recorte en tiempo real mediante GPU clip plane
                let gpu_clip = if self.clip_plane_enabled {
                    let (orig, n_raw, _) = self.active_plane_origin_and_normal();
                    let n = if self.clip_invert { -n_raw } else { n_raw };
                    Some(Vec4::new(n.x, n.y, n.z, -n.dot(orig)))
                } else {
                    None
                };

                ui.painter().add(make_viewport_paint_callback(
                    Arc::clone(renderer),
                    rect,
                    &self.camera,
                    layers,
                    gpu_clip,
                ));
            });

        // ==================================================================
        // BARRA FLOTANTE SUPERIOR CENTRAL: Vistas Rápidas de Cámara
        // ==================================================================
        Area::new(egui::Id::new("v2_cam_bar"))
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
                            if ui.button(tr("Inferior (-Z)", "Bottom (-Z)", "Inferior (-Z)")).clicked() { self.camera.set_view_occlusal_bottom(); }
                            if ui.button(tr("Vestibular", "Buccal", "Vestibular")).clicked() { self.camera.set_view_frontal(); }
                            if ui.button("Lingual").clicked() { self.camera.set_view_lingual(); }
                            if ui.button("Lateral").clicked() { self.camera.set_view_lateral(); }
                            if ui.button("Iso").clicked() { self.camera.set_view_isometric(); }
                            if ui.button(tr("Centrar", "Center", "Centralizar")).clicked() {
                                if let Some((m, _)) = self.meshes.get(&self.active_arch) {
                                    let (mn, mx) = m.bounds();
                                    self.camera.frame_bounds(mn, mx);
                                }
                            }
                        });
                    });
            });

        // ==================================================================
        // VENTANA FLOTANTE IZQ 1: Arcada Activa, Limpieza y Suavizado Taubin
        // ==================================================================
        Window::new(tr(
            "1. Arcada Activa y Limpieza",
            "1. Active Arch & Cleanup",
            "1. Arcada Ativa e Limpeza",
        ))
            .id(egui::Id::new("v2_card_clean"))
            .default_pos(egui::pos2(14.0, 74.0))
            .default_width(318.0)
            .collapsible(true)
            .resizable(false)
            .show(ctx, |ui| {
                ui.horizontal(|ui| {
                    ui.label(tr("Editar capa:", "Edit layer:", "Editar camada:"));
                    egui::ComboBox::from_id_salt("v2_arch_combo")
                        .selected_text(match self.active_arch.as_str() {
                            "maxilla" => tr("Maxilar (Superior)", "Maxilla (Upper)", "Maxila (Superior)"),
                            "bite" => tr("Mordida (Bite)", "Bite Scan", "Mordida (Bite)"),
                            _ => tr("Mandíbula (Inferior)", "Mandible (Lower)", "Mandíbula (Inferior)"),
                        })
                        .show_ui(ui, |ui| {
                            if ui.selectable_label(self.active_arch == "mandible", tr("Mandíbula (Inferior)", "Mandible (Lower)", "Mandíbula (Inferior)")).clicked() {
                                self.switch_active_arch("mandible");
                            }
                            if ui.selectable_label(self.active_arch == "maxilla", tr("Maxilar (Superior)", "Maxilla (Upper)", "Maxila (Superior)")).clicked() {
                                self.switch_active_arch("maxilla");
                            }
                            if ui.selectable_label(self.active_arch == "bite", tr("Mordida (Bite)", "Bite Scan", "Mordida (Bite)")).clicked() {
                                self.switch_active_arch("bite");
                            }
                        });
                });

                if let Some((m, _)) = self.meshes.get(&self.active_arch) {
                    let is_wt = self.mesh_watertight.get(&self.active_arch).copied().unwrap_or(false);
                    let wt = if is_wt {
                        tr("✔ Cerrado", "✔ Closed", "✔ Fechado")
                    } else {
                        tr("Abierto", "Open", "Aberto")
                    };
                    ui.label(
                        RichText::new(format!(
                            "{} {}  |  {} {} ({})",
                            tr("Vértices:", "Vertices:", "Vértices:"),
                            m.positions.len(),
                            tr("Triángulos:", "Triangles:", "Triângulos:"),
                            m.indices.len(),
                            wt
                        ))
                        .size(11.0)
                        .color(theme::AMETHYST_BRIGHT),
                    );
                } else {
                    ui.label(RichText::new(tr("Sin malla en esta capa", "No mesh on this layer", "Sem malha nesta camada")).size(11.0).color(CREAM_MUTED));
                }

                ui.checkbox(&mut self.remove_islands, tr("Eliminar islas flotantes desconectadas", "Remove disconnected floating islands", "Remover ilhas flutuantes desconectadas"));
                ui.horizontal(|ui| {
                    ui.label(tr("Pasadas Taubin:", "Taubin passes:", "Passadas Taubin:"));
                    ui.add(egui::Slider::new(&mut self.taubin_iters, 0..=25));
                });

                if ui.add_sized([ui.available_width(), 28.0], egui::Button::new(tr("Limpiar y Reparar Malla Activa", "Clean & Repair Active Mesh", "Limpar e Reparar Malha Ativa"))).clicked() {
                    let key = self.active_arch.clone();
                    if let Some((m, _)) = self.meshes.get(&key).cloned() {
                        self.push_undo(&key, case);
                        let mut cleaned = clean_mesh(&m, self.remove_islands, self.taubin_iters);
                        if key == case.prep_arch {
                            cleaned.custom_scalars = compute_undercut_angles_deg(
                                &cleaned,
                                Vec3::from_array(case.insertion_axis),
                            );
                        }
                        self.insert_mesh(&key, cleaned);
                        self.save_working_mesh(&key, case, case_manager);
                        self.reset_clip_offset_only();
                        action = ViewerAction::StatusMessage("Malla limpiada y suavizada con filtro Taubin.".to_string());
                    }
                }

                ui.horizontal(|ui| {
                    if ui.button("Restaurar Raw").clicked() {
                        let key = self.active_arch.clone();
                        self.push_undo(&key, case);
                        if let Some(p) = case_manager.reset_working_scan_to_raw(case, &key) {
                            if let Ok(mut m) = TriMeshData::load_from_path(&p) {
                                if key == case.prep_arch {
                                    m.custom_scalars = compute_undercut_angles_deg(
                                        &m,
                                        Vec3::from_array(case.insertion_axis),
                                    );
                                }
                                self.insert_mesh(&key, m);
                                self.reset_clip_bounds();
                                action = ViewerAction::StatusMessage("Escaneo original restaurado.".to_string());
                            }
                        }
                    }
                    if ui.button("Guardar Modelo").clicked() {
                        let key = self.active_arch.clone();
                        self.save_working_mesh(&key, case, case_manager);
                        action = ViewerAction::StatusMessage("Modelo de trabajo guardado en disco.".to_string());
                    }
                });
            });

        // ==================================================================
        // VENTANA FLOTANTE IZQ 2: Recorte 3D Múltiple, Zócalo Sólido y Hollow
        // ==================================================================
        Window::new(tr(
            "2. Recorte 3D, Zócalo y Hollow",
            "2. 3D Trim, Base & Hollow",
            "2. Recorte 3D, Base e Hollow",
        ))
            .id(egui::Id::new("v2_card_crop"))
            .default_pos(egui::pos2(14.0, 290.0))
            .default_width(318.0)
            .collapsible(true)
            .resizable(false)
            .show(ctx, |ui| {
                // Selector rápido directo de modelo a recortar (para cortar Maxilar y Antagonista sin bloqueos)
                ui.horizontal(|ui| {
                    ui.label(RichText::new(tr("Modelo:", "Model:", "Modelo:")).size(11.0).color(CREAM_MUTED));
                    for (k, lbl) in [
                        ("mandible", tr("Mandíbula", "Mandible", "Mandíbula")),
                        ("maxilla", tr("Maxilar", "Maxilla", "Maxila")),
                        ("bite", tr("Mordida", "Bite", "Mordida")),
                    ] {
                        let exists = self.meshes.contains_key(k);
                        if ui
                            .add_enabled(exists, egui::Button::new(lbl).selected(self.active_arch == k))
                            .clicked()
                        {
                            self.switch_active_arch(k);
                        }
                    }
                });

                let mut clip_changed = false;
                clip_changed |= ui
                    .checkbox(
                        &mut self.clip_invert,
                        tr(
                            "Invertir región conservada en recorte",
                            "Invert kept trim region",
                            "Inverter região mantida no recorte",
                        ),
                    )
                    .changed();
                ui.horizontal(|ui| {
                    if ui.selectable_label(self.clip_plane_enabled, tr("📐 Plano 3D Visible", "📐 Visible 3D Plane", "📐 Plano 3D Visível")).clicked() {
                        self.clip_plane_enabled = !self.clip_plane_enabled;
                        if self.clip_plane_enabled {
                            self.clip_box_enabled = false;
                            self.reset_clip_bounds();
                        }
                    }
                    if ui.selectable_label(self.clip_box_enabled, tr("📦 Caja 3D", "📦 3D Box", "📦 Caixa 3D")).clicked() {
                        self.clip_box_enabled = !self.clip_box_enabled;
                        if self.clip_box_enabled {
                            self.clip_plane_enabled = false;
                            self.reset_clip_bounds();
                        }
                    }
                });

                if self.clip_plane_enabled {
                    let half_diag = self
                        .meshes
                        .get(&self.active_arch)
                        .map(|(m, _)| {
                            let (a, b) = m.bounds();
                            (0.5 * (b - a).length()).max(16.0)
                        })
                        .unwrap_or(24.0);
                    ui.horizontal(|ui| {
                        if ui.small_button("Base (Z)").clicked() {
                            self.plane_tilt_x = 0.0;
                            self.plane_tilt_y = 0.0;
                            self.plane_offset = 0.0;
                            clip_changed = true;
                        }
                        if ui.small_button("Lateral (X)").clicked() {
                            self.plane_tilt_x = 0.0;
                            self.plane_tilt_y = 90.0;
                            self.plane_offset = 0.0;
                            clip_changed = true;
                        }
                        if ui.small_button("Frontal (Y)").clicked() {
                            self.plane_tilt_x = -90.0;
                            self.plane_tilt_y = 0.0;
                            self.plane_offset = 0.0;
                            clip_changed = true;
                        }
                        if ui.small_button(tr("Centrar", "Center", "Centralizar")).clicked() {
                            self.plane_offset = 0.0;
                            clip_changed = true;
                        }
                    });
                    clip_changed |= ui
                        .add(
                            egui::Slider::new(&mut self.plane_offset, -half_diag..=half_diag)
                                .text(tr("Posición (mm)", "Position (mm)", "Posição (mm)")),
                        )
                        .changed();
                    clip_changed |= ui
                        .add(egui::Slider::new(&mut self.plane_tilt_x, -90.0..=90.0).text(tr("Inclin. X (°)", "Tilt X (°)", "Inclin. X (°)")))
                        .changed();
                    clip_changed |= ui
                        .add(egui::Slider::new(&mut self.plane_tilt_y, -90.0..=90.0).text(tr("Inclin. Y (°)", "Tilt Y (°)", "Inclin. Y (°)")))
                        .changed();
                } else if self.clip_box_enabled {
                    let (mn, mx) = self
                        .meshes
                        .get(&self.active_arch)
                        .map(|(m, _)| m.bounds())
                        .unwrap_or((Vec3::splat(-20.0), Vec3::splat(20.0)));
                    clip_changed |= ui.add(egui::Slider::new(&mut self.box_min.x, mn.x..=mx.x).text("Min X")).changed();
                    clip_changed |= ui.add(egui::Slider::new(&mut self.box_max.x, mn.x..=mx.x).text("Max X")).changed();
                    clip_changed |= ui.add(egui::Slider::new(&mut self.box_min.y, mn.y..=mx.y).text("Min Y")).changed();
                    clip_changed |= ui.add(egui::Slider::new(&mut self.box_max.y, mn.y..=mx.y).text("Max Y")).changed();
                }
                if clip_changed {
                    self.clip_vis_rev = self.bump_rev();
                }

                ui.add_enabled_ui(self.clip_plane_enabled || self.clip_box_enabled, |ui| {
                    ui.horizontal(|ui| {
                        if theme::accent_button(ui, tr("✂ Aplicar Recorte (Multi-corte)", "✂ Apply Trim (Multi-cut)", "✂ Aplicar Recorte (Multi-corte)")).clicked() {
                            let key = self.active_arch.clone();
                            if let Some((m, _)) = self.meshes.get(&key).cloned() {
                                let (orig, norm, _) = self.active_plane_origin_and_normal();
                                let mut clipped = if self.clip_plane_enabled {
                                    clip_by_plane(&m, orig, norm, self.clip_invert)
                                } else {
                                    clip_by_box(&m, self.box_min, self.box_max, self.clip_invert)
                                };
                                if !clipped.is_empty() {
                                    self.push_undo(&key, case);
                                    if key == case.prep_arch {
                                        clipped.custom_scalars = compute_undercut_angles_deg(
                                            &clipped,
                                            Vec3::from_array(case.insertion_axis),
                                        );
                                    }
                                    self.insert_mesh(&key, clipped);
                                    self.save_working_mesh(&key, case, case_manager);
                                    // Mantener la herramienta abierta para permitir recortes consecutivos
                                    // (base, laterales, antagonista, etc.) sin bloqueos!
                                    self.reset_clip_offset_only();
                                    action = ViewerAction::StatusMessage(
                                        tr(
                                            "Recorte 3D aplicado. Puedes mover el plano y cortar otra cara o cambiar de modelo.",
                                            "3D trim applied. Move the plane to cut another side or switch model.",
                                            "Recorte 3D aplicado. Mova o plano para cortar outro lado ou trocar de modelo.",
                                        )
                                        .to_string(),
                                    );
                                } else {
                                    action = ViewerAction::StatusMessage(
                                        tr(
                                            "El recorte eliminó toda la malla; prueba invertir o centrar el plano.",
                                            "Trim removed the entire mesh; try inverting or centering the plane.",
                                            "O recorte removeu toda a malha; tente inverter ou centralizar o plano.",
                                        )
                                        .to_string(),
                                    );
                                }
                            }
                        }
                        if ui.button(tr("✕ Cerrar", "✕ Close", "✕ Fechar")).clicked() {
                            self.clip_plane_enabled = false;
                            self.clip_box_enabled = false;
                        }
                    });
                });

                ui.separator();
                ui.add(egui::Slider::new(&mut self.base_height_mm, 1.0..=20.0).text(tr("Altura zócalo (mm)", "Base height (mm)", "Altura da base (mm)")));
                ui.horizontal(|ui| {
                    ui.radio_value(&mut self.base_down, true, tr("Inferior (-Z)", "Lower (-Z)", "Inferior (-Z)"));
                    ui.radio_value(&mut self.base_down, false, tr("Superior (+Z)", "Upper (+Z)", "Superior (+Z)"));
                });
                if ui.add_sized([ui.available_width(), 26.0], egui::Button::new(tr("🧱 Generar Zócalo Sólido Imprimible", "🧱 Generate Printable Solid Base", "🧱 Gerar Base Sólida Imprimível"))).clicked() {
                    let key = self.active_arch.clone();
                    if let Some((m, _)) = self.meshes.get(&key).cloned() {
                        self.push_undo(&key, case);
                        let mut rebased = create_solid_base(&m, self.base_height_mm, self.base_down);
                        if key == case.prep_arch {
                            rebased.custom_scalars = compute_undercut_angles_deg(
                                &rebased,
                                Vec3::from_array(case.insertion_axis),
                            );
                        }
                        let audit = rebased.inspect_cam();
                        self.insert_mesh(&key, rebased);
                        self.save_working_mesh(&key, case, case_manager);
                        self.reset_clip_offset_only();
                        action = ViewerAction::StatusMessage(format!(
                            "{} ({} | {} tris, {:.1} cm³).",
                            tr("Zócalo sólido generado", "Solid base generated", "Base sólida gerada"),
                            if audit.watertight {
                                tr("Watertight ✔", "Watertight ✔", "Estanque ✔")
                            } else {
                                tr("Revisar borde", "Check border", "Verificar borda")
                            },
                            audit.n_faces,
                            audit.volume_mm3 / 1000.0
                        ));
                    }
                }

                ui.add_space(4.0);
                ui.add(
                    egui::Slider::new(&mut self.hollow_wall_mm, 1.0..=3.0)
                        .step_by(0.1)
                        .text(tr("Pared Hollow (mm)", "Hollow wall (mm)", "Parede Hollow (mm)")),
                );
                if theme::accent_button(ui, tr("🕳 Generar Modelo Hollow (1–3 mm Imprimible)", "🕳 Generate Hollow Model (1–3 mm Printable)", "🕳 Gerar Modelo Oco (1–3 mm Imprimível)")).clicked() {
                    let key = self.active_arch.clone();
                    if let Some((m, _)) = self.meshes.get(&key).cloned() {
                        self.push_undo(&key, case);
                        let (mut hollowed, ok, mut msg) = create_hollow_base_checked(
                            &m,
                            self.base_height_mm,
                            self.hollow_wall_mm,
                            self.base_down,
                        );
                        if key == case.prep_arch {
                            hollowed.custom_scalars = compute_undercut_angles_deg(
                                &hollowed,
                                Vec3::from_array(case.insertion_axis),
                            );
                        }
                        if ok {
                            msg.push(' ');
                            msg.push_str(tr(
                                "Pulsa '👁 Ver Base / Ahuecado' para inspeccionar el interior.",
                                "Click '👁 View Base / Hollow' to inspect the interior.",
                                "Clique em '👁 Ver Base / Oco' para inspecionar o interior.",
                            ));
                        }
                        self.insert_mesh(&key, hollowed);
                        self.save_working_mesh(&key, case, case_manager);
                        self.reset_clip_offset_only();
                        action = ViewerAction::StatusMessage(msg);
                    }
                }
                if ui
                    .add_sized([ui.available_width(), 24.0], egui::Button::new(tr("👁 Ver Base / Ahuecado", "👁 View Base / Hollow", "👁 Ver Base / Oco")))
                    .on_hover_text(tr(
                        "Rota automáticamente hacia la base inferior/superior para ver la cavidad hollow abierta",
                        "Automatically rotates toward the lower/upper base to inspect the open hollow cavity",
                        "Rotaciona automaticamente para a base inferior/superior para ver a cavidade oca aberta",
                    ))
                    .clicked()
                {
                    if let Some((m, _)) = self.meshes.get(&self.active_arch) {
                        let (mn, mx) = m.bounds();
                        self.camera.frame_bounds(mn, mx);
                    }
                    if self.base_down {
                        self.camera.set_view_occlusal_bottom();
                    } else {
                        self.camera.set_view_occlusal_top();
                    }
                    action = ViewerAction::StatusMessage(
                        tr(
                            "Vista enfocada en la base para inspeccionar el ahuecado (hollow).",
                            "View focused on the base to inspect the hollow cavity.",
                            "Vista focada na base para inspecionar a cavidade oca.",
                        )
                        .to_string(),
                    );
                }
            });

        // ==================================================================
        // VENTANA FLOTANTE IZQ 3: Eje de Inserción y Mapa de Socavados
        // ==================================================================
        Window::new(tr(
            "3. Eje de Inserción y Socavados",
            "3. Insertion Axis & Undercuts",
            "3. Eixo de Inserção e Retenções",
        ))
            .id(egui::Id::new("v2_card_axis"))
            .default_pos(egui::pos2(14.0, 625.0))
            .default_width(318.0)
            .default_open(false)
            .collapsible(true)
            .resizable(false)
            .show(ctx, |ui| {
                let ax = case.insertion_axis;
                ui.label(
                    RichText::new(format!("{} [{:.2}, {:.2}, {:.2}]", tr("Eje actual:", "Current axis:", "Eixo atual:"), ax[0], ax[1], ax[2]))
                        .size(11.0)
                        .color(CREAM_MUTED),
                );
                if ui.add_sized([ui.available_width(), 28.0], egui::Button::new(tr("Fijar Eje desde Vista Actual", "Set Axis from Current View", "Definir Eixo da Vista Atual"))).clicked() {
                    let key = case.prep_arch.clone();
                    self.push_undo(&key, case);
                    let dir = (self.camera.eye_position() - self.camera.target).normalize_or_zero();
                    case.insertion_axis = [dir.x, dir.y, dir.z];
                    if let Some((m, rev)) = self.meshes.get_mut(&key) {
                        m.custom_scalars = compute_undercut_angles_deg(m, dir);
                        *rev = self.next_rev + 1;
                        self.next_rev += 1;
                    }
                    let _ = case_manager.save_case(case);
                    action = ViewerAction::StatusMessage(
                        tr(
                            "Eje de inserción actualizado desde cámara.",
                            "Insertion axis updated from camera.",
                            "Eixo de inserção atualizado pela câmera.",
                        )
                        .to_string(),
                    );
                }
                ui.checkbox(
                    &mut self.show_undercuts,
                    tr(
                        "Mapa Térmico de Socavados (Rojo = Retención)",
                        "Undercut Heatmap (Red = Undercut)",
                        "Mapa Térmico de Retenções (Vermelho = Retenção)",
                    ),
                );
            });

        // ==================================================================
        // VENTANA FLOTANTE SUP. DERECHA: Transparencias y Capas 3D
        // ==================================================================
        let right_x = (ctx.screen_rect().width() - 305.0).max(360.0);
        Window::new(tr(
            "Transparencias y Capas 3D",
            "3D Layers & Transparencies",
            "Transparências e Camadas 3D",
        ))
            .id(egui::Id::new("v2_card_trans"))
            .default_pos(egui::pos2(right_x, 74.0))
            .default_width(290.0)
            .collapsible(true)
            .resizable(false)
            .show(ctx, |ui| {
                for (key, lbl) in [
                    ("mandible", tr("Mandíbula", "Mandible", "Mandíbula")),
                    ("maxilla", tr("Maxilar", "Maxilla", "Maxila")),
                    ("bite", tr("Mordida", "Bite", "Mordida")),
                ] {
                    ui.horizontal(|ui| {
                        let vis = self.visible.entry(key.to_string()).or_insert(true);
                        ui.checkbox(vis, lbl);
                        let op = self.opacity.entry(key.to_string()).or_insert(1.0);
                        ui.add(egui::Slider::new(op, 0.10..=1.0).show_value(false));
                        ui.label(RichText::new(format!("{:.0}%", *op * 100.0)).size(10.5).color(CREAM_MUTED));
                    });
                }
                ui.checkbox(
                    &mut self.wireframe,
                    tr(
                        "Ver malla triangular (Wireframe)",
                        "Show triangle mesh (Wireframe)",
                        "Mostrar malha triangular (Wireframe)",
                    ),
                );
            });

        // ==================================================================
        // BOTÓN FLOTANTE INFERIOR DERECHO: Continuar a Línea de Margen
        // ==================================================================
        Area::new(egui::Id::new("v2_next_bar"))
            .anchor(Align2::RIGHT_BOTTOM, egui::vec2(-14.0, -34.0))
            .order(Order::Foreground)
            .show(ctx, |ui| {
                Frame::NONE
                    .fill(egui::Color32::from_rgba_unmultiplied(0x21, 0x05, 0x2C, 240))
                    .stroke(Stroke::new(1.5_f32, EMERALD))
                    .corner_radius(CornerRadius::same(8))
                    .inner_margin(Margin::same(6))
                    .show(ui, |ui| {
                        if theme::primary_button(ui, tr("Continuar a Línea de Margen  →", "Continue to Margin Line  →", "Continuar para Linha de Margem  →")).clicked() {
                            action = ViewerAction::ProceedToMargin;
                        }
                    });
            });

        action
    }
}
