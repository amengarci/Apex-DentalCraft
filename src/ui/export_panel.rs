//! Módulo 5 (UI + Auditoría CAM): Fusión Watertight de Puentes, Modelos Imprimibles y Exportación STL / OBJ.

use crate::core::case_manager::{BridgeConnectorConfig, CaseManager, DentalCase};
use crate::core::i18n::tr;
use crate::core::scene_file::{
    self, ApexSceneFile, ApexSceneModels, ApexSceneRestoration, ApexSceneViewState,
    APEX_SCENE_EXTENSION, APEX_SCENE_FORMAT_ID, APEX_SCENE_FORMAT_VERSION,
};
use crate::geometry::connector::fuse_bridge_watertight;
use crate::ui::bridge_sculpt_panel::{
    collect_group_connectors, is_final_bridge_up_to_date, resolve_unit_design_path,
    BridgeSculptPanelState,
};
use crate::geometry::mesh::{CamAudit, TriMeshData};
use crate::render::camera::OrbitCamera;
use crate::render::gl_renderer::{
    make_viewport_paint_callback, ColorMapKind, GlViewportRenderer, RenderLayerSpec,
};
use crate::theme::{self, AMBER, AMETHYST, BG_PANEL, BORDER_PLUM, CREAM, CREAM_MUTED, CYAN, EMERALD};
use egui::{Align2, Area, CornerRadius, Frame, Margin, Order, RichText, Sense, Stroke};
use glam::Vec3;
use std::collections::{BTreeMap, BTreeSet};
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};

pub enum ExportAction {
    None,
    OpenInStlViewer,
    StatusMessage(String),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ExportFormat {
    StlBinary,
    StlAscii,
    Obj,
}

#[derive(Clone)]
pub struct ExportPiece {
    pub label: String,
    pub file_tag: String,
    pub fdis: Vec<i32>,
    pub is_bridge: bool,
    pub connector_summary: Vec<String>,
    pub mesh: TriMeshData,
    pub rev: u64,
    pub audit: CamAudit,
}

pub struct ExportPanelState {
    pub camera: OrbitCamera,
    pub next_rev: u64,
    pub export_pieces: Vec<ExportPiece>,
    pub selected_piece_idx: usize,
    pub crown_mesh: Option<(TriMeshData, u64)>,
    pub prep_mesh: Option<(TriMeshData, u64)>,
    pub mandible_mesh: Option<(TriMeshData, u64)>,
    pub maxilla_mesh: Option<(TriMeshData, u64)>,
    pub bite_mesh: Option<(TriMeshData, u64)>,
    pub audit: CamAudit,

    pub isolated_crown: bool,
    pub show_wireframe: bool,
    pub format: ExportFormat,
    pub export_scene: bool,
    pub export_die_model: bool,
    pub export_report: bool,
    pub last_export_dir: Option<PathBuf>,
    pub last_export_msg: String,
}

impl Default for ExportPanelState {
    fn default() -> Self {
        Self {
            camera: OrbitCamera::default(),
            next_rev: 1,
            export_pieces: Vec::new(),
            selected_piece_idx: 0,
            crown_mesh: None,
            prep_mesh: None,
            mandible_mesh: None,
            maxilla_mesh: None,
            bite_mesh: None,
            audit: CamAudit::default(),
            isolated_crown: true,
            show_wireframe: false,
            format: ExportFormat::StlBinary,
            export_scene: true,
            export_die_model: true,
            export_report: true,
            last_export_dir: None,
            last_export_msg: String::new(),
        }
    }
}

impl ExportPanelState {
    fn bump(&mut self) -> u64 {
        self.next_rev += 1;
        self.next_rev
    }

    fn sync_selected_piece(&mut self) {
        if self.export_pieces.is_empty() {
            self.selected_piece_idx = 0;
            self.crown_mesh = None;
            self.audit = CamAudit::default();
            return;
        }
        if self.selected_piece_idx >= self.export_pieces.len() {
            self.selected_piece_idx = 0;
        }
        let p = &self.export_pieces[self.selected_piece_idx];
        self.crown_mesh = Some((p.mesh.clone(), p.rev));
        self.audit = p.audit.clone();
    }

    pub fn load_case(&mut self, case: &DentalCase, case_manager: &CaseManager) {
        self.export_pieces.clear();
        self.selected_piece_idx = 0;
        self.crown_mesh = None;
        self.audit = CamAudit::default();
        self.last_export_dir = None;
        self.last_export_msg.clear();

        if let Some(p) = case_manager.get_active_scan_path(case, &case.prep_arch) {
            if let Ok(m) = TriMeshData::load_from_path(&p) {
                let rev = self.bump();
                self.prep_mesh = Some((m, rev));
            } else {
                self.prep_mesh = None;
            }
        } else {
            self.prep_mesh = None;
        }

        self.mandible_mesh = case_manager
            .get_active_scan_path(case, "mandible")
            .and_then(|p| TriMeshData::load_from_path(&p).ok())
            .map(|m| {
                let rev = self.bump();
                (m, rev)
            });

        self.maxilla_mesh = case_manager
            .get_active_scan_path(case, "maxilla")
            .and_then(|p| TriMeshData::load_from_path(&p).ok())
            .map(|m| {
                let rev = self.bump();
                (m, rev)
            });

        self.bite_mesh = case_manager
            .get_active_scan_path(case, "bite")
            .and_then(|p| TriMeshData::load_from_path(&p).ok())
            .map(|m| {
                let rev = self.bump();
                (m, rev)
            });

        let mut groups = case.connected_bridge_groups();
        if groups.is_empty() {
            groups.push(vec![case.tooth_number]);
        }

        let arch_axis = Vec3::from_array(case.insertion_axis);
        let case_dir = case_manager.get_case_dir(&case.case_id);
        let design_dir = case_dir.join("design");

        for group in groups {
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

            if members.is_empty() {
                continue;
            }

            if members.len() >= 2 {
                let loaded_set: BTreeSet<i32> = members.iter().map(|(f, _, _)| *f).collect();
                let (group_cfgs, connector_summary) = collect_group_connectors(case, &loaded_set);

                let fdis: Vec<i32> = members.iter().map(|(f, _, _)| *f).collect();
                let fdi_tag = fdis
                    .iter()
                    .map(|d| d.to_string())
                    .collect::<Vec<_>>()
                    .join("_");
                let fdi_label = fdis
                    .iter()
                    .map(|d| format!("#{}", d))
                    .collect::<Vec<_>>()
                    .join("–");

                let final_path = design_dir.join(format!("bridge_{}_final.stl", fdi_tag));
                let mut fused = if is_final_bridge_up_to_date(&final_path, &member_paths) {
                    TriMeshData::load_from_path(&final_path).unwrap_or_default()
                } else {
                    TriMeshData::default()
                };
                if fused.is_empty() {
                    for (fdi, m, is_p) in &mut members {
                        if !*is_p && m.is_external.iter().all(|&b| b) {
                            if let Some(u) = case.restorations.get(fdi) {
                                if u.margin_points.len() >= 4 {
                                    let mpts: Vec<Vec3> = u.margin_points.iter().map(|p| Vec3::from_array(*p)).collect();
                                    BridgeSculptPanelState::classify_bridge_external_vertices(
                                        m,
                                        self.prep_mesh.as_ref().map(|(pm, _)| pm),
                                        &[mpts],
                                    );
                                }
                            }
                        }
                    }
                    let conn_refs: Vec<(&BridgeConnectorConfig, Vec3)> =
                        group_cfgs.iter().map(|c| (c, arch_axis)).collect();
                    fused = fuse_bridge_watertight(&members, &conn_refs);
                    if !fused.is_empty() {
                        let _ = std::fs::create_dir_all(&design_dir);
                        let _ = fused.save_stl_binary(&design_dir.join(format!("bridge_{}.stl", fdi_tag)));
                    }
                }
                if fused.is_empty() {
                    continue;
                }

                let audit = fused.inspect_cam();
                let rev = self.bump();
                self.export_pieces.push(ExportPiece {
                    label: format!("{} {}", tr("Puente", "Bridge", "Ponte"), fdi_label),
                    file_tag: format!("Puente_FDI{}", fdi_tag),
                    fdis,
                    is_bridge: true,
                    connector_summary,
                    mesh: fused,
                    rev,
                    audit,
                });
            } else {
                let (fdi, mesh, _is_p) = members.into_iter().next().unwrap();
                let audit = mesh.inspect_cam();
                let rev = self.bump();
                self.export_pieces.push(ExportPiece {
                    label: format!("{} #{}", tr("Corona", "Crown", "Coroa"), fdi),
                    file_tag: format!("FDI{}_Corona", fdi),
                    fdis: vec![fdi],
                    is_bridge: false,
                    connector_summary: Vec::new(),
                    mesh,
                    rev,
                    audit,
                });
            }
        }

        self.sync_selected_piece();

        let mut mn = Vec3::splat(f32::INFINITY);
        let mut mx = Vec3::splat(f32::NEG_INFINITY);
        for piece in &self.export_pieces {
            let (pmn, pmx) = piece.mesh.bounds();
            mn = mn.min(pmn);
            mx = mx.max(pmx);
        }
        if mn.x.is_finite() {
            self.camera.frame_bounds(mn, mx);
        } else if let Some((prep, _)) = &self.prep_mesh {
            let (pmn, pmx) = prep.bounds();
            self.camera.frame_bounds(pmn, pmx);
        }
    }

    fn save_mesh_in_format(&self, mesh: &TriMeshData, path: &Path) -> Result<(), String> {
        match self.format {
            ExportFormat::StlBinary => mesh.save_stl_binary(path),
            ExportFormat::StlAscii => mesh.save_stl_ascii(path),
            ExportFormat::Obj => mesh.save_obj(path),
        }
    }

    pub fn export_bundle_to_dir(
        &mut self,
        case: &mut DentalCase,
        case_manager: &CaseManager,
        out_dir: &Path,
    ) -> Result<PathBuf, String> {
        if self.export_pieces.is_empty() {
            return Err(tr(
                "No hay restauraciones diseñadas para exportar.",
                "No designed restorations to export.",
                "Não há restaurações desenhadas para exportar.",
            )
            .to_string());
        }
        std::fs::create_dir_all(out_dir).map_err(|e| {
            format!("No se pudo crear carpeta de exportación '{}': {}", out_dir.display(), e)
        })?;

        let safe_p_raw: String = case
            .patient_name
            .chars()
            .map(|c| if c.is_alphanumeric() { c } else { '_' })
            .collect();
        let safe_p = safe_p_raw.trim_matches('_');
        let safe_p = if safe_p.is_empty() { "Paciente" } else { safe_p };
        let stem = format!("ApexDentalCraft_{}", safe_p);
        let ext = match self.format {
            ExportFormat::Obj => "obj",
            _ => "stl",
        };

        // 1. Exportar cada restauración / puente diseñado
        let mut exported_records: Vec<(String, String)> = Vec::new();
        let mut restorations_meta: Vec<ApexSceneRestoration> = Vec::new();
        let mut first_mesh_path: Option<PathBuf> = None;

        for piece in &self.export_pieces {
            let fname = format!("{}_{}.{}", stem, piece.file_tag, ext);
            let piece_path = out_dir.join(&fname);
            self.save_mesh_in_format(&piece.mesh, &piece_path)?;
            exported_records.push((piece.label.clone(), fname.clone()));
            restorations_meta.push(ApexSceneRestoration {
                label: piece.label.clone(),
                file_name: fname,
                fdis: piece.fdis.clone(),
                is_bridge: piece.is_bridge,
                connector_summary: piece.connector_summary.clone(),
                watertight: piece.audit.watertight,
                n_faces: piece.audit.n_faces,
                volume_mm3: piece.audit.volume_mm3,
            });
            if first_mesh_path.is_none() {
                first_mesh_path = Some(piece_path);
            }
        }

        // 2. Exportar STL de Muñón (Modelo de Trabajo), Antagonista y Mordida
        let die_arch = if case.prep_arch == "maxilla" {
            "maxilla"
        } else {
            "mandible"
        };
        let ant_arch = case.get_antagonist_arch();
        let mut models_meta = ApexSceneModels {
            die_model: None,
            die_arch: die_arch.to_string(),
            antagonist_model: None,
            antagonist_arch: ant_arch.to_string(),
            bite_model: None,
        };
        let mut exported_models: Vec<String> = Vec::new();

        if self.export_die_model {
            let die_mesh_opt = self.prep_mesh.as_ref().map(|(m, _)| m).or_else(|| {
                if die_arch == "maxilla" {
                    self.maxilla_mesh.as_ref().map(|(m, _)| m)
                } else {
                    self.mandible_mesh.as_ref().map(|(m, _)| m)
                }
            });
            if let Some(die_mesh) = die_mesh_opt {
                let arch_tag = if die_arch == "maxilla" { "Maxilar" } else { "Mandibula" };
                let fname = format!("{}_Munon_{}.{}", stem, arch_tag, ext);
                self.save_mesh_in_format(die_mesh, &out_dir.join(&fname))?;
                let au = die_mesh.inspect_cam();
                exported_models.push(format!(
                    "Muñón / Modelo de Trabajo {} ({} — {} tris, {})",
                    arch_tag,
                    fname,
                    au.n_faces,
                    if au.watertight { "Watertight" } else { "Abierto" }
                ));
                models_meta.die_model = Some(fname);
            }

            let ant_mesh_opt = if ant_arch == "maxilla" {
                self.maxilla_mesh.as_ref().map(|(m, _)| m)
            } else {
                self.mandible_mesh.as_ref().map(|(m, _)| m)
            };
            if let Some(ant_mesh) = ant_mesh_opt {
                let arch_tag = if ant_arch == "maxilla" { "Maxilar" } else { "Mandibula" };
                let fname = format!("{}_Antagonista_{}.{}", stem, arch_tag, ext);
                self.save_mesh_in_format(ant_mesh, &out_dir.join(&fname))?;
                let au = ant_mesh.inspect_cam();
                exported_models.push(format!(
                    "Antagonista {} ({} — {} tris, {})",
                    arch_tag,
                    fname,
                    au.n_faces,
                    if au.watertight { "Watertight" } else { "Abierto" }
                ));
                models_meta.antagonist_model = Some(fname);
            }

            if let Some((bite_mesh, _)) = &self.bite_mesh {
                let fname = format!("{}_Mordida.{}", stem, ext);
                self.save_mesh_in_format(bite_mesh, &out_dir.join(&fname))?;
                exported_models.push(format!("Mordida ({})", fname));
                models_meta.bite_model = Some(fname);
            }
        }

        // 3. Guardar mallas unitarias individuales (para reabrir puentes editables en Módulo 4) y escena .apexscene
        let mut scene_fname_opt: Option<String> = None;
        let mut primary_path = first_mesh_path.unwrap_or_else(|| out_dir.to_path_buf());
        if self.export_scene {
            let mut unit_designs: BTreeMap<i32, String> = BTreeMap::new();
            for (idx, piece) in self.export_pieces.iter().enumerate() {
                if !piece.is_bridge && piece.fdis.len() == 1 {
                    if let Some((_, fname)) = exported_records.get(idx) {
                        unit_designs.insert(piece.fdis[0], fname.clone());
                    }
                } else if piece.is_bridge {
                    let units_dir = out_dir.join("unidades");
                    for &fdi in &piece.fdis {
                        if let Some(src) = resolve_unit_design_path(case, case_manager, fdi) {
                            if src.is_file() {
                                let _ = std::fs::create_dir_all(&units_dir);
                                let rel = format!("unidades/unit_FDI{}.stl", fdi);
                                let dst = out_dir.join(&rel);
                                let is_stl = src
                                    .extension()
                                    .and_then(|s| s.to_str())
                                    .map(|e| e.eq_ignore_ascii_case("stl"))
                                    .unwrap_or(false);
                                if is_stl {
                                    if std::fs::copy(&src, &dst).is_ok() {
                                        unit_designs.insert(fdi, rel);
                                    }
                                } else if let Ok(m) = TriMeshData::load_from_path(&src) {
                                    if m.save_stl_binary(&dst).is_ok() {
                                        unit_designs.insert(fdi, rel);
                                    }
                                }
                            }
                        }
                    }
                }
            }

            case.sync_active_to_unit();
            let scene_fname = format!("{}.{}", stem, APEX_SCENE_EXTENSION);
            let scene_path = out_dir.join(&scene_fname);
            let scene = ApexSceneFile {
                format: APEX_SCENE_FORMAT_ID.to_string(),
                version: APEX_SCENE_FORMAT_VERSION,
                application: "APEX DENTALCRAFT".to_string(),
                exported_at: chrono::Local::now().format("%Y-%m-%dT%H:%M:%S").to_string(),
                case_data: case.clone(),
                models: models_meta.clone(),
                restorations: restorations_meta,
                unit_designs,
                view_state: ApexSceneViewState {
                    camera_target: self.camera.target.to_array(),
                    camera_orientation: self.camera.orientation.to_array(),
                    camera_distance: self.camera.distance,
                    isolated_crown: self.isolated_crown,
                },
            };
            scene_file::save_apex_scene(&scene_path, &scene)?;
            scene_fname_opt = Some(scene_fname);
            primary_path = scene_path;
        }

        // 4. Ficha técnica .txt
        if self.export_report {
            let report_path = out_dir.join(format!("{}_FichaOrden_ApexDentalCraft.txt", stem));
            let p = &case.design_params;
            let mut pieces_txt = String::new();
            for (idx, piece) in self.export_pieces.iter().enumerate() {
                let fname = exported_records
                    .get(idx)
                    .map(|(_, f)| f.as_str())
                    .unwrap_or("");
                let status = if piece.audit.watertight {
                    "SÍ (2-Manifold Cerrado)"
                } else {
                    "VERIFICAR"
                };
                pieces_txt.push_str(&format!(
                    "\n- {} -> Archivo: {}\n  Watertight CAM: {} | Tris: {} | Pliegues: {} | Volumen: {:.2} mm³\n",
                    piece.label, fname, status, piece.audit.n_faces, piece.audit.folded_edges, piece.audit.volume_mm3
                ));
                if !piece.connector_summary.is_empty() {
                    pieces_txt.push_str(&format!(
                        "  Conectores Puente: {}\n",
                        piece.connector_summary.join(" ; ")
                    ));
                }
            }
            let models_txt = if exported_models.is_empty() {
                "No exportados".to_string()
            } else {
                exported_models.join("\n                   ")
            };
            let scene_txt = scene_fname_opt
                .as_deref()
                .unwrap_or("No exportada");
            let report = format!(
                "============================================================\n\
                 APEX DENTAL LAB — FICHA TÉCNICA DE PRODUCCIÓN\n\
                 Generado por APEX DENTALCRAFT (Offline)\n\
                 ============================================================\n\
                 ID de Caso       : {}\n\
                 Carpeta Export   : {}\n\
                 Escena Guardada  : {}\n\
                 Paciente         : {}\n\
                 Odontólogo       : {}\n\
                 Orden Completa   : {}\n\
                 Color VITA       : {}\n\
                 Librería Usada   : {}\n\
                 Espacio Cemento  : {:.3} mm ({:.0} µm)\n\
                 Sellado Marginal : {:.3} mm ({:.0} µm)\n\
                 Grosor Mínimo    : {:.2} mm\n\
                 Modelos Trabajo  : {}\n\
                 Notas            : {}\n\
                 ------------------------------------------------------------\n\
                 PIEZAS Y PUENTES EXPORTADOS:{}\n",
                case.case_id,
                out_dir.display(),
                scene_txt,
                case.patient_name,
                case.doctor_name,
                case.format_order_summary(),
                case.shade,
                p.library_id,
                p.cement_gap_mm,
                p.cement_gap_mm * 1000.0,
                p.marginal_gap_mm,
                p.marginal_gap_mm * 1000.0,
                p.min_thickness_mm,
                models_txt,
                case.notes,
                pieces_txt,
            );
            std::fs::write(&report_path, report).map_err(|e| e.to_string())?;
        }

        case.last_export_path = Some(primary_path.to_string_lossy().to_string());
        self.last_export_dir = Some(out_dir.to_path_buf());
        let mut parts_summary: Vec<String> = Vec::new();
        if let Some(sf) = &scene_fname_opt {
            parts_summary.push(format!("Escena: {}", sf));
        }
        parts_summary.push(format!("{} pieza(s) {}", exported_records.len(), ext.to_uppercase()));
        if models_meta.die_model.is_some() {
            parts_summary.push("Muñón".to_string());
        }
        if models_meta.antagonist_model.is_some() {
            parts_summary.push("Antagonista".to_string());
        }
        self.last_export_msg = format!(
            "✔ Exportado en:\n{}\nContenido: {}",
            out_dir.display(),
            parts_summary.join(" • ")
        );
        let _ = case_manager.save_case(case);
        Ok(primary_path)
    }

    pub fn ui(
        &mut self,
        ctx: &egui::Context,
        case: &mut DentalCase,
        case_manager: &CaseManager,
        renderer: &Arc<Mutex<GlViewportRenderer>>,
    ) -> ExportAction {
        let mut action = ExportAction::None;

        let card_frame = Frame::NONE
            .fill(BG_PANEL)
            .stroke(Stroke::new(1.0_f32, BORDER_PLUM))
            .corner_radius(CornerRadius::same(8))
            .inner_margin(Margin::same(10));

        // ==================================================================
        // PANEL IZQUIERDO: Auditoría CAM y Exportación
        // ==================================================================
        egui::SidePanel::left("export_left_col")
            .exact_width(385.0)
            .resizable(false)
            .frame(Frame::NONE.fill(theme::BG_MAIN).inner_margin(Margin::same(10)))
            .show(ctx, |ui| {
                egui::ScrollArea::vertical().show(ui, |ui| {
                    card_frame.show(ui, |ui| {
                        ui.label(RichText::new(tr("Ficha de Orden — Apex Dental Lab", "Order Sheet — Apex Dental Lab", "Ficha da Ordem — Apex Dental Lab")).strong().size(13.0).color(CREAM));
                        let p = &case.design_params;
                        ui.label(format!("{}: {}", tr("Paciente", "Patient", "Paciente"), case.patient_name));
                        ui.label(format!(
                            "{}: {}",
                            tr("Odontólogo", "Doctor", "Dentista"),
                            if case.doctor_name.is_empty() { tr("No especificado", "Not specified", "Não especificado") } else { &case.doctor_name }
                        ));
                        ui.label(format!("{}: #{} ({})", tr("Pieza Activa", "Active Unit", "Dente Ativo"), case.tooth_number, case.shade));
                        ui.label(RichText::new(format!("{}: {}", tr("Orden", "Order", "Ordem"), case.format_order_summary())).size(11.0).color(CREAM_MUTED));
                        ui.label(
                            RichText::new(format!(
                                "{} {:.0} µm | {} {:.0} µm | {}. {:.2} mm",
                                tr("Cemento", "Cement", "Cimento"),
                                p.cement_gap_mm * 1000.0,
                                tr("Margen", "Margin", "Término"),
                                p.marginal_gap_mm * 1000.0,
                                tr("Espesor mín", "Min. wall", "Espessura mín"),
                                p.min_thickness_mm
                            ))
                            .size(11.0)
                            .color(EMERALD),
                        );
                    });

                    ui.add_space(8.0);
                    card_frame.show(ui, |ui| {
                        ui.label(RichText::new(tr("Auditoría de Calidad CAM / Impresión 3D", "CAM / 3D Print Quality Audit", "Auditoria de Qualidade CAM / Impressão 3D")).strong().size(13.0).color(CREAM));

                        if self.export_pieces.len() > 1 {
                            ui.add_space(2.0);
                            ui.label(
                                RichText::new(format!("{} ({}):", tr("Entregas CAM", "CAM Units", "Unidades CAM"), self.export_pieces.len()))
                                    .size(10.8)
                                    .color(CREAM_MUTED),
                            );
                            let mut next_idx: Option<usize> = None;
                            ui.horizontal_wrapped(|ui| {
                                for (idx, piece) in self.export_pieces.iter().enumerate() {
                                    let wt_icon = if piece.audit.watertight { "✔" } else { "⚠" };
                                    if ui
                                        .selectable_label(
                                            self.selected_piece_idx == idx,
                                            RichText::new(format!("{} {}", wt_icon, piece.label)).size(10.8),
                                        )
                                        .clicked()
                                    {
                                        next_idx = Some(idx);
                                    }
                                }
                            });
                            if let Some(i) = next_idx {
                                self.selected_piece_idx = i;
                                self.sync_selected_piece();
                            }
                            ui.separator();
                        }

                        if let Some(piece) = self.export_pieces.get(self.selected_piece_idx) {
                            let tag_type = if piece.is_bridge {
                                tr("PUENTE FUSIONADO (MALLA ÚNICA)", "FUSED BRIDGE (SINGLE SOLID)", "PONTE FUSIONADA (PEÇA ÚNICA)")
                            } else {
                                tr("RESTAURACIÓN INDIVIDUAL", "SINGLE RESTORATION", "RESTAURAÇÃO UNITÁRIA")
                            };
                            ui.label(RichText::new(format!("{} — {}", piece.label, tag_type)).size(11.0).strong().color(CYAN));

                            if !piece.connector_summary.is_empty() {
                                ui.label(RichText::new(tr("Conectores:", "Connectors:", "Conectores:")).size(10.5).color(CREAM_MUTED));
                                for line in &piece.connector_summary {
                                    ui.label(RichText::new(format!("  • {}", line)).size(10.5).color(EMERALD));
                                }
                            }
                        }

                        if self.audit.valid {
                            if self.audit.watertight {
                                ui.label(RichText::new(tr("✔ MALLA CERRADA (WATERTIGHT) — LISTA PARA CAM", "✔ WATERTIGHT CLOSED MESH — CAM READY", "✔ MALHA FECHADA (WATERTIGHT) — PRONTA PARA CAM")).strong().color(EMERALD));
                            } else {
                                ui.label(
                                    RichText::new(format!("⚠ {} {} — {}", self.audit.open_edges, tr("aristas abiertas", "open edges", "arestas abertas"), tr("Verificar en slicer", "Check in slicer", "Verificar no slicer")))
                                        .strong()
                                        .color(AMBER),
                                );
                            }
                            let vol_cm3 = self.audit.volume_mm3 / 1000.0;
                            ui.label(format!(
                                "• {}: {:.2} × {:.2} × {:.2} mm",
                                tr("Dimensiones", "Dimensions", "Dimensões"),
                                self.audit.dims_mm.x, self.audit.dims_mm.y, self.audit.dims_mm.z
                            ));
                            ui.label(format!("• {}: {:.1} mm³ ({:.3} cm³)", tr("Volumen neto", "Net volume", "Volume líquido"), self.audit.volume_mm3, vol_cm3));
                            ui.label(format!(
                                "• {} Zirconia: {:.2} g | Disilicato: {:.2} g | Resina: {:.2} g",
                                tr("Masa est.", "Est. mass", "Massa est."),
                                vol_cm3 * 6.05,
                                vol_cm3 * 2.50,
                                vol_cm3 * 1.18
                            ));
                            ui.label(format!(
                                "• {}: {} vtx / {} tris  |  {}: {}",
                                tr("Geometría", "Geometry", "Geometria"),
                                self.audit.n_vertices, self.audit.n_faces,
                                tr("Pliegues", "Folds", "Dobras"),
                                self.audit.folded_edges
                            ));
                        } else {
                            ui.label(RichText::new(tr("⚠ Genera primero la(s) restauración(es) en el Módulo 4.", "⚠ Generate restoration(s) in Module 4 first.", "⚠ Gere primeiro a(s) restauração(ões) no Módulo 4.")).color(AMBER));
                        }
                    });

                    ui.add_space(8.0);
                    card_frame.show(ui, |ui| {
                        ui.label(RichText::new(tr("Inspección Pre-Fresado", "Pre-Milling Inspection", "Inspeção Pré-Fresagem")).strong().size(13.0));
                        ui.checkbox(&mut self.isolated_crown, tr("Aislar Pieza / Puente Seleccionado (Intaglio Interno)", "Isolate Selected Unit / Bridge (Internal Intaglio)", "Isolar Dente / Ponte Selecionada (Intaglio Interno)"));
                        ui.checkbox(&mut self.show_wireframe, tr("Mostrar Malla Triangular STL", "Show STL Triangle Wireframe", "Mostrar Malha Triangular STL"));
                        ui.add_space(4.0);
                        if theme::accent_button(ui, tr("🔍 Abrir Escena Completa en Visor STL", "🔍 Open Full Scene in 3D Viewer", "🔍 Abrir Cena Completa no Visor STL")).clicked() {
                            action = ExportAction::OpenInStlViewer;
                        }
                    });

                    ui.add_space(8.0);
                    card_frame.show(ui, |ui| {
                        ui.label(RichText::new(tr("Opciones de Exportación Local", "Local Export Options", "Opções de Exportação Local")).strong().size(13.0));
                        egui::ComboBox::from_id_salt("exp_fmt_cb")
                            .width(ui.available_width())
                            .selected_text(match self.format {
                                ExportFormat::StlBinary => tr("STL Binario (.stl) — Estándar CAM / Slicers", "Binary STL (.stl) — CAM / Slicer Standard", "STL Binário (.stl) — Padrão CAM / Slicers"),
                                ExportFormat::StlAscii => tr("STL ASCII (.stl) — Texto Legible", "ASCII STL (.stl) — Human Readable", "STL ASCII (.stl) — Texto Legível"),
                                ExportFormat::Obj => "Wavefront OBJ (.obj)",
                            })
                            .show_ui(ui, |ui| {
                                ui.selectable_value(&mut self.format, ExportFormat::StlBinary, tr("STL Binario (.stl) — Estándar CAM / Slicers", "Binary STL (.stl) — CAM / Slicer Standard", "STL Binário (.stl) — Padrão CAM / Slicers"));
                                ui.selectable_value(&mut self.format, ExportFormat::StlAscii, tr("STL ASCII (.stl) — Texto Legible", "ASCII STL (.stl) — Human Readable", "STL ASCII (.stl) — Texto Legível"));
                                ui.selectable_value(&mut self.format, ExportFormat::Obj, "Wavefront OBJ (.obj)");
                            });

                        ui.checkbox(
                            &mut self.export_scene,
                            tr(
                                "Guardar Escena Apex DentalCraft (.apexscene)",
                                "Save Apex DentalCraft Scene (.apexscene)",
                                "Salvar Cena Apex DentalCraft (.apexscene)",
                            ),
                        );
                        ui.checkbox(
                            &mut self.export_die_model,
                            tr(
                                "Exportar STL de Muñón (Modelo Trabajo) y Antagonista",
                                "Export Die (Working Model) & Antagonist STLs",
                                "Exportar STL do Troquel (Modelo Trabalho) e Antagonista",
                            ),
                        );
                        ui.checkbox(
                            &mut self.export_report,
                            tr(
                                "Incluir Ficha Técnica de Orden (.txt)",
                                "Include Technical Order Sheet (.txt)",
                                "Incluir Ficha Técnica da Ordem (.txt)",
                            ),
                        );

                        let default_dir = scene_file::default_case_export_dir(case);
                        ui.add_space(4.0);
                        Frame::NONE
                            .fill(theme::BG_DEEP)
                            .stroke(Stroke::new(1.0_f32, BORDER_PLUM))
                            .corner_radius(CornerRadius::same(6))
                            .inner_margin(Margin::symmetric(8, 6))
                            .show(ui, |ui| {
                                ui.label(
                                    RichText::new(tr(
                                        "📁 Carpeta de exportación (en carpeta del programa):",
                                        "📁 Export folder (in program directory):",
                                        "📁 Pasta de exportação (na pasta do programa):",
                                    ))
                                    .strong()
                                    .size(10.5)
                                    .color(CYAN),
                                );
                                ui.add(
                                    egui::Label::new(
                                        RichText::new(default_dir.display().to_string())
                                            .size(10.0)
                                            .color(CREAM_MUTED),
                                    )
                                    .wrap(),
                                );
                            });

                        ui.add_space(6.0);
                        let has_crown = !self.export_pieces.is_empty();
                        ui.add_enabled_ui(has_crown, |ui| {
                            if ui
                                .add_sized(
                                    [ui.available_width(), 38.0],
                                    egui::Button::new(
                                        RichText::new(tr(
                                            "★ Exportar en Carpeta Casos (.apexscene + STLs)",
                                            "★ Export to Casos Folder (.apexscene + STLs)",
                                            "★ Exportar na Pasta Casos (.apexscene + STLs)",
                                        ))
                                        .strong()
                                        .color(egui::Color32::from_rgb(0x23, 0x03, 0x2E)),
                                    )
                                    .fill(CREAM),
                                )
                                .clicked()
                            {
                                match self.export_bundle_to_dir(case, case_manager, &default_dir) {
                                    Ok(_) => {
                                        action = ExportAction::StatusMessage(format!(
                                            "{} {}",
                                            tr(
                                                "✔ Escena (.apexscene) y STLs exportados en",
                                                "✔ Scene (.apexscene) & STLs exported to",
                                                "✔ Cena (.apexscene) e STLs exportados em",
                                            ),
                                            default_dir.display()
                                        ))
                                    }
                                    Err(e) => {
                                        action = ExportAction::StatusMessage(format!(
                                            "{}: {}",
                                            tr("Error al exportar", "Export error", "Erro ao exportar"),
                                            e
                                        ))
                                    }
                                }
                            }

                            ui.add_space(4.0);
                            ui.horizontal(|ui| {
                                if ui
                                    .button(tr(
                                        "Exportar Copia en Otra Carpeta...",
                                        "Export Copy to Folder...",
                                        "Exportar Cópia em Outra Pasta...",
                                    ))
                                    .clicked()
                                {
                                    let mut dlg = rfd::FileDialog::new().set_title(tr(
                                        "Seleccionar carpeta para guardar Escena (.apexscene) y STLs",
                                        "Select folder to save Scene (.apexscene) & STLs",
                                        "Selecionar pasta para salvar Cena (.apexscene) e STLs",
                                    ));
                                    let casos_root = scene_file::default_casos_dir();
                                    if casos_root.exists() {
                                        dlg = dlg.set_directory(&casos_root);
                                    }
                                    if let Some(folder) = dlg.pick_folder() {
                                        let dest = folder.join(&case.case_id);
                                        match self.export_bundle_to_dir(case, case_manager, &dest) {
                                            Ok(_) => {
                                                action = ExportAction::StatusMessage(format!(
                                                    "{} {}",
                                                    tr("✔ Guardado en", "✔ Saved to", "✔ Salvo em"),
                                                    dest.display()
                                                ))
                                            }
                                            Err(e) => {
                                                action = ExportAction::StatusMessage(format!(
                                                    "{}: {}",
                                                    tr("Error al exportar", "Export error", "Erro ao exportar"),
                                                    e
                                                ))
                                            }
                                        }
                                    }
                                }
                                if ui
                                    .button(tr(
                                        "📂 Abrir Carpeta Casos",
                                        "📂 Open Casos Folder",
                                        "📂 Abrir Pasta Casos",
                                    ))
                                    .clicked()
                                {
                                    let open_dir = if let Some(d) = &self.last_export_dir {
                                        d.clone()
                                    } else if default_dir.exists() {
                                        default_dir.clone()
                                    } else {
                                        let root = scene_file::default_casos_dir();
                                        let _ = std::fs::create_dir_all(&root);
                                        root
                                    };
                                    let _ = std::fs::create_dir_all(&open_dir);
                                    #[cfg(target_os = "windows")]
                                    let _ = std::process::Command::new("explorer").arg(&open_dir).spawn();
                                    #[cfg(target_os = "linux")]
                                    let _ = std::process::Command::new("xdg-open").arg(&open_dir).spawn();
                                }
                            });
                        });

                        if !self.last_export_msg.is_empty() {
                            ui.add_space(4.0);
                            ui.add(
                                egui::Label::new(
                                    RichText::new(&self.last_export_msg)
                                        .size(10.5)
                                        .color(EMERALD),
                                )
                                .wrap(),
                            );
                        }
                    });
                });
            });

        // ==================================================================
        // VISOR 3D DE INSPECCIÓN PRE-FRESADO
        // ==================================================================
        egui::CentralPanel::default()
            .frame(Frame::NONE.fill(theme::BG_DEEP))
            .show(ctx, |ui| {
                let rect = ui.available_rect_before_wrap();
                let resp = ui.allocate_rect(rect, Sense::click_and_drag());
                self.camera.handle_navigation_input(&resp, ui, rect, true);

                let mut layers = Vec::new();
                if let Ok(mut guard) = renderer.lock() {
                    if !self.isolated_crown {
                        let mut rendered_any_arch = false;
                        if let Some((mand, rev)) = &self.mandible_mesh {
                            guard.ensure_mesh_uploaded("e5_mand", *rev, mand, ColorMapKind::Solid);
                            let op = if case.prep_arch == "mandible" { 1.0 } else { 0.50 };
                            layers.push(RenderLayerSpec::solid("e5_mand", *rev, [0.90, 0.84, 0.75], op));
                            rendered_any_arch = true;
                        }
                        if let Some((max, rev)) = &self.maxilla_mesh {
                            guard.ensure_mesh_uploaded("e5_max", *rev, max, ColorMapKind::Solid);
                            let op = if case.prep_arch == "maxilla" { 1.0 } else { 0.50 };
                            layers.push(RenderLayerSpec::solid("e5_max", *rev, [0.90, 0.84, 0.75], op));
                            rendered_any_arch = true;
                        }
                        if !rendered_any_arch {
                            if let Some((prep, rev)) = &self.prep_mesh {
                                guard.ensure_mesh_uploaded("e5_prep", *rev, prep, ColorMapKind::Solid);
                                layers.push(RenderLayerSpec::solid("e5_prep", *rev, [0.90, 0.84, 0.75], 1.0));
                            }
                        }

                        for (idx, piece) in self.export_pieces.iter().enumerate() {
                            let id = format!("e5_piece_{}", idx);
                            guard.ensure_mesh_uploaded(&id, piece.rev, &piece.mesh, ColorMapKind::Solid);
                            let col = if idx == self.selected_piece_idx {
                                [0.97, 0.93, 0.82]
                            } else {
                                [0.91, 0.88, 0.81]
                            };
                            let mut sp = RenderLayerSpec::solid(&id, piece.rev, col, 1.0);
                            sp.specular = 0.62;
                            sp.shininess = 42.0;
                            sp.wireframe = self.show_wireframe && idx == self.selected_piece_idx;
                            layers.push(sp);
                        }
                    } else if let Some(piece) = self.export_pieces.get(self.selected_piece_idx) {
                        let id = format!("e5_iso_{}", self.selected_piece_idx);
                        guard.ensure_mesh_uploaded(&id, piece.rev, &piece.mesh, ColorMapKind::Solid);
                        let mut sp = RenderLayerSpec::solid(&id, piece.rev, [0.95, 0.92, 0.83], 1.0);
                        sp.specular = 0.62;
                        sp.shininess = 42.0;
                        sp.wireframe = self.show_wireframe;
                        layers.push(sp);
                    }
                }

                ui.painter().add(make_viewport_paint_callback(
                    Arc::clone(renderer),
                    rect,
                    &self.camera,
                    layers,
                    None,
                ));
            });

        // Barra superior de vistas de cámara en Módulo 5
        Area::new(egui::Id::new("e5_cam_bar"))
            .anchor(Align2::CENTER_TOP, egui::vec2(190.0, 68.0))
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
                            if ui.button("Intaglio (-Z)").clicked() { self.camera.set_view_occlusal_bottom(); }
                            if ui.button(tr("Vestibular", "Buccal", "Vestibular")).clicked() { self.camera.set_view_frontal(); }
                            if ui.button("Lingual").clicked() { self.camera.set_view_lingual(); }
                            if ui.button(tr("Isométrica", "Isometric", "Isométrica")).clicked() { self.camera.set_view_isometric(); }
                            if ui.button(tr("Centrar Pieza", "Center Unit", "Centralizar Peça")).clicked() {
                                if let Some(piece) = self.export_pieces.get(self.selected_piece_idx) {
                                    let (mn, mx) = piece.mesh.bounds();
                                    self.camera.frame_bounds(mn, mx);
                                } else if let Some((prep, _)) = &self.prep_mesh {
                                    let (mn, mx) = prep.bounds();
                                    self.camera.frame_bounds(mn, mx);
                                }
                            }
                        });
                    });
            });

        action
    }
}
