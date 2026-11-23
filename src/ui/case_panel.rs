//! Módulo 1 (UI): Gestor de Casos, Odontograma en Óvalo Vertical, Escaneos y Librerías Dentales.

use crate::core::case_manager::{
    find_material_preset, CaseManager, DentalCase, ToothRestorationUnit, APEX_MATERIAL_PRESETS,
    APEX_PROCESS_TYPES, VITA_SHADES,
};
use crate::core::i18n::{self, tr};
use crate::core::library_manager::ToothLibraryManager;
use crate::core::scene_file;
use crate::geometry::mesh_ops::generate_parametric_demo_arch;
use crate::theme::{self, BG_PANEL, BORDER_PLUM, CREAM, CREAM_MUTED, EMERALD};
use crate::ui::odontogram::{
    show_fdi_odontogram, FdiOdontogramState, OdontogramEvent, LOWER_ARCH, UPPER_ARCH,
};
use egui::{CornerRadius, Frame, Margin, RichText, Stroke};
use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

pub enum CasePanelAction {
    None,
    OpenInViewer(DentalCase),
    CaseUpdated(DentalCase),
    OpenApexScene(DentalCase, PathBuf),
    StatusMessage(String),
}

pub struct CasePanelState {
    pub search_query: String,
    pub cases_cache: Vec<DentalCase>,
    pub current_case: Option<DentalCase>,
    pub draft_restorations: BTreeMap<i32, ToothRestorationUnit>,
    pub odontogram: FdiOdontogramState,
    pub undo_stack: Vec<(BTreeMap<i32, ToothRestorationUnit>, Vec<i32>)>,

    pub selected_process_idx: usize,
    pub selected_material_idx: usize,
    pub selected_shade_idx: usize,
    pub patient_name: String,
    pub doctor_name: String,
    pub notes: String,
}

impl CasePanelState {
    pub fn new(case_manager: &CaseManager) -> Self {
        let mut st = Self {
            search_query: String::new(),
            cases_cache: Vec::new(),
            current_case: None,
            draft_restorations: BTreeMap::new(),
            odontogram: FdiOdontogramState::default(),
            undo_stack: Vec::new(),
            selected_process_idx: 0,
            selected_material_idx: 0,
            selected_shade_idx: 1, // A2
            patient_name: String::new(),
            doctor_name: String::new(),
            notes: String::new(),
        };
        st.refresh_cases(case_manager);
        if let Some(first) = st.cases_cache.first().cloned() {
            st.set_active_case(first);
        }
        st
    }

    pub fn refresh_cases(&mut self, case_manager: &CaseManager) {
        self.cases_cache = case_manager.list_cases();
    }

    pub fn set_active_case(&mut self, case: DentalCase) {
        self.undo_stack.clear();
        self.draft_restorations = case.restorations.clone();
        self.odontogram.set_single(case.tooth_number);
        self.patient_name = case.patient_name.clone();
        self.doctor_name = case.doctor_name.clone();
        self.notes = case.notes.clone();
        if let Some(u) = case.get_active_unit() {
            if let Some(pi) = APEX_PROCESS_TYPES.iter().position(|&p| p == u.process_type) {
                self.selected_process_idx = pi;
            }
            if let Some(mi) = APEX_MATERIAL_PRESETS.iter().position(|m| m.name == u.material) {
                self.selected_material_idx = mi;
            }
            if let Some(si) = VITA_SHADES.iter().position(|&s| s == u.shade) {
                self.selected_shade_idx = si;
            }
        }
        self.current_case = Some(case);
    }

    pub fn push_undo(&mut self) {
        self.undo_stack
            .push((self.draft_restorations.clone(), self.odontogram.get_selected_vec()));
        if self.undo_stack.len() > 25 {
            self.undo_stack.remove(0);
        }
    }

    pub fn undo(&mut self, case_manager: &CaseManager) -> bool {
        let Some((prev_draft, prev_sel)) = self.undo_stack.pop() else {
            return false;
        };
        self.draft_restorations = prev_draft;
        self.odontogram.selected_teeth = prev_sel.iter().copied().collect();
        self.odontogram.selected_tooth = prev_sel.last().copied();
        self.sync_draft_to_case(case_manager);
        true
    }

    fn sync_draft_to_case(&mut self, case_manager: &CaseManager) {
        if let Some(case) = &mut self.current_case {
            if !self.draft_restorations.is_empty() {
                case.restorations = self.draft_restorations.clone();
                if let Some(sel) = self.odontogram.selected_tooth {
                    if case.restorations.contains_key(&sel) {
                        case.set_active_tooth(sel);
                    } else if let Some(&first) = case.restorations.keys().next() {
                        case.set_active_tooth(first);
                    }
                }
                let _ = case_manager.save_case(case);
                self.cases_cache = case_manager.list_cases();
            }
        }
    }

    fn toggle_connector(&mut self, t1: i32, t2: i32, connect: bool, case_manager: &CaseManager) {
        self.push_undo();
        let mat = APEX_MATERIAL_PRESETS[self.selected_material_idx].name;
        let shade = VITA_SHADES[self.selected_shade_idx];

        for fdi in [t1, t2] {
            self.draft_restorations.entry(fdi).or_insert_with(|| {
                ToothRestorationUnit::from_material_preset(
                    fdi,
                    if connect { "Pilar de Puente" } else { "Corona Individual" },
                    mat,
                    shade,
                    Vec::new(),
                )
            });
        }
        if connect {
            if let Some(u1) = self.draft_restorations.get_mut(&t1) {
                if !u1.connected_to.contains(&t2) {
                    u1.connected_to.push(t2);
                    u1.connected_to.sort_unstable();
                }
                if u1.process_type == "Corona Individual" {
                    u1.process_type = "Pilar de Puente".to_string();
                }
            }
            if let Some(u2) = self.draft_restorations.get_mut(&t2) {
                if !u2.connected_to.contains(&t1) {
                    u2.connected_to.push(t1);
                    u2.connected_to.sort_unstable();
                }
                if u2.process_type == "Corona Individual" {
                    u2.process_type = "Pilar de Puente".to_string();
                }
            }
        } else {
            if let Some(u1) = self.draft_restorations.get_mut(&t1) {
                u1.connected_to.retain(|&x| x != t2);
                if u1.connected_to.is_empty()
                    && matches!(u1.process_type.as_str(), "Pilar de Puente" | "Póntico de Puente")
                {
                    u1.process_type = "Corona Individual".to_string();
                }
            }
            if let Some(u2) = self.draft_restorations.get_mut(&t2) {
                u2.connected_to.retain(|&x| x != t1);
                if u2.connected_to.is_empty()
                    && matches!(u2.process_type.as_str(), "Pilar de Puente" | "Póntico de Puente")
                {
                    u2.process_type = "Corona Individual".to_string();
                }
            }
        }
        self.sync_draft_to_case(case_manager);
    }

    pub fn create_demo_case(
        &mut self,
        case_manager: &CaseManager,
        library_manager: &ToothLibraryManager,
    ) -> Result<DentalCase, String> {
        let mut units = BTreeMap::new();
        units.insert(
            36,
            ToothRestorationUnit::from_material_preset(
                36,
                "Corona Individual",
                APEX_MATERIAL_PRESETS[0].name,
                "A2",
                Vec::new(),
            ),
        );
        units.insert(
            11,
            ToothRestorationUnit::from_material_preset(
                11,
                "Carilla Estética (Veneer)",
                APEX_MATERIAL_PRESETS[2].name,
                "A1",
                Vec::new(),
            ),
        );
        units.insert(
            21,
            ToothRestorationUnit::from_material_preset(
                21,
                "Carilla Estética (Veneer)",
                APEX_MATERIAL_PRESETS[2].name,
                "A1",
                Vec::new(),
            ),
        );
        units.insert(
            24,
            ToothRestorationUnit::from_material_preset(
                24,
                "Pilar de Puente",
                APEX_MATERIAL_PRESETS[1].name,
                "A2",
                vec![25],
            ),
        );
        units.insert(
            25,
            ToothRestorationUnit::from_material_preset(
                25,
                "Póntico de Puente",
                APEX_MATERIAL_PRESETS[1].name,
                "A2",
                vec![24, 26],
            ),
        );
        units.insert(
            26,
            ToothRestorationUnit::from_material_preset(
                26,
                "Pilar de Puente",
                APEX_MATERIAL_PRESETS[1].name,
                "A2",
                vec![25],
            ),
        );

        let mut demo = case_manager.create_case(
            "Paciente_Demo_MultiProceso",
            "Apex Dental Lab — TD Alejandro Mendoza",
            36,
            "Corona Individual — Zirconia Monolítica (3Y-TZP Alta Resistencia)",
            "A2",
            "Orden multi-proceso: Corona #36, Carillas individuales #11/#21 (○) y Puente #24-●-#25-●-#26.",
            units,
        )?;

        let case_dir = case_manager.get_case_dir(&demo.case_id);
        let mand_tmp = case_dir.join("raw").join("_src_mand.stl");
        let max_tmp = case_dir.join("raw").join("_src_max.stl");

        if let Some(real_mand) = library_manager.find_demo_asset("demo_mandible_prep36.stl") {
            let _ = case_manager.import_scan(&mut demo, "mandible", &real_mand);
        } else {
            let m = generate_parametric_demo_arch(false);
            let _ = m.save_stl_binary(&mand_tmp);
            let _ = case_manager.import_scan(&mut demo, "mandible", &mand_tmp);
            let _ = std::fs::remove_file(&mand_tmp);
        }

        if let Some(real_max) = library_manager.find_demo_asset("demo_maxilla_antagonist.stl") {
            let _ = case_manager.import_scan(&mut demo, "maxilla", &real_max);
        } else {
            let m = generate_parametric_demo_arch(true);
            let _ = m.save_stl_binary(&max_tmp);
            let _ = case_manager.import_scan(&mut demo, "maxilla", &max_tmp);
            let _ = std::fs::remove_file(&max_tmp);
        }

        self.refresh_cases(case_manager);
        self.set_active_case(demo.clone());
        Ok(demo)
    }

    pub fn ui(
        &mut self,
        ctx: &egui::Context,
        case_manager: &CaseManager,
        library_manager: &mut ToothLibraryManager,
    ) -> CasePanelAction {
        let mut action = CasePanelAction::None;

        let card_frame = Frame::NONE
            .fill(BG_PANEL)
            .stroke(Stroke::new(1.0_f32, BORDER_PLUM))
            .corner_radius(CornerRadius::same(8))
            .inner_margin(Margin::same(10));

        // ==================================================================
        // COLUMNA IZQUIERDA (260 px): Órdenes Locales Guardadas
        // ==================================================================
        egui::SidePanel::left("case_left_col")
            .exact_width(265.0)
            .resizable(false)
            .frame(Frame::NONE.fill(theme::BG_MAIN).inner_margin(Margin::same(8)))
            .show(ctx, |ui| {
                card_frame.show(ui, |ui| {
                    ui.set_min_height(ui.available_height());
                    ui.label(RichText::new(tr("Órdenes Locales — Apex Lab", "Local Orders — Apex Lab", "Ordens Locais — Apex Lab")).strong().size(13.5).color(CREAM));
                    ui.label(
                        RichText::new(format!("{}: {}", tr("Ruta", "Path", "Pasta"), case_manager.root_dir.display()))
                            .size(10.0)
                            .color(CREAM_MUTED),
                    );
                    ui.add(
                        egui::TextEdit::singleline(&mut self.search_query)
                            .hint_text(tr("Filtrar orden o diente...", "Filter order or tooth...", "Filtrar ordem ou dente..."))
                            .desired_width(f32::INFINITY),
                    );
                    ui.add_space(4.0);

                    let q = self.search_query.trim().to_lowercase();
                    let active_id = self.current_case.as_ref().map(|c| c.case_id.clone());
                    let mut clicked_case: Option<String> = None;

                    let list_h = (ui.available_height() - 134.0).max(120.0);
                    egui::ScrollArea::vertical().max_height(list_h).show(ui, |ui| {
                        for c in &self.cases_cache {
                            let summary = c.format_order_summary();
                            let hay = format!("{} {} {} {}", c.patient_name, c.doctor_name, summary, c.restoration_type)
                                .to_lowercase();
                            if !q.is_empty() && !hay.contains(&q) {
                                continue;
                            }
                            let is_sel = active_id.as_deref() == Some(c.case_id.as_str());
                            let has_max = if c.scans_raw.get("maxilla").and_then(|o| o.as_ref()).is_some() { "Max✔" } else { "Max-" };
                            let has_man = if c.scans_raw.get("mandible").and_then(|o| o.as_ref()).is_some() { "Mand✔" } else { "Mand-" };
                            let label = format!(
                                "{} ({} {})\n{}\n[{} / {}]",
                                c.patient_name,
                                c.restorations.len(),
                                tr("pza", "unit", "dente"),
                                summary,
                                has_max,
                                has_man
                            );
                            if ui.selectable_label(is_sel, RichText::new(label).size(11.0)).clicked() {
                                clicked_case = Some(c.case_id.clone());
                            }
                            ui.separator();
                        }
                    });

                    if let Some(cid) = clicked_case {
                        if let Ok(loaded) = case_manager.load_case(&cid) {
                            self.set_active_case(loaded.clone());
                            action = CasePanelAction::CaseUpdated(loaded);
                        }
                    }

                    ui.add_space(6.0);
                    ui.horizontal(|ui| {
                        if ui.button(tr("+ Nueva", "+ New", "+ Nova")).clicked() {
                            self.current_case = None;
                            self.draft_restorations.clear();
                            self.undo_stack.clear();
                            self.odontogram.clear();
                            self.patient_name.clear();
                            self.doctor_name.clear();
                            self.notes.clear();
                        }
                        if theme::danger_button(ui, tr("Eliminar", "Delete", "Excluir")).clicked() {
                            if let Some(c) = self.current_case.take() {
                                let _ = case_manager.delete_case(&c.case_id);
                                self.draft_restorations.clear();
                                self.odontogram.clear();
                                self.refresh_cases(case_manager);
                                action = CasePanelAction::StatusMessage(tr("Caso eliminado del disco local.", "Case deleted from local disk.", "Caso excluído do disco local.").to_string());
                            }
                        }
                    });
                    ui.add_space(4.0);
                    if ui
                        .add_sized(
                            [ui.available_width(), 28.0],
                            egui::Button::new(
                                RichText::new(tr(
                                    "📂 Abrir Escena (.apexscene)",
                                    "📂 Open Scene (.apexscene)",
                                    "📂 Abrir Cena (.apexscene)",
                                ))
                                .strong()
                                .size(11.0)
                                .color(CREAM),
                            )
                            .fill(egui::Color32::from_rgb(0x2B, 0x07, 0x38))
                            .stroke(Stroke::new(1.0_f32, EMERALD)),
                        )
                        .clicked()
                    {
                        let mut dlg = rfd::FileDialog::new()
                            .set_title(tr(
                                "Abrir Escena Guardada de Apex DentalCraft (.apexscene)",
                                "Open Saved Apex DentalCraft Scene (.apexscene)",
                                "Abrir Cena Salva do Apex DentalCraft (.apexscene)",
                            ))
                            .add_filter("Escena Apex DentalCraft (.apexscene)", &["apexscene"]);
                        let casos_root = scene_file::default_casos_dir();
                        if casos_root.exists() {
                            dlg = dlg.set_directory(&casos_root);
                        }
                        if let Some(scene_path) = dlg.pick_file() {
                            match scene_file::import_scene_into_case_manager(&scene_path, case_manager)
                            {
                                Ok((loaded, _scene)) => {
                                    self.refresh_cases(case_manager);
                                    self.set_active_case(loaded.clone());
                                    action = CasePanelAction::OpenApexScene(loaded, scene_path);
                                }
                                Err(e) => {
                                    action = CasePanelAction::StatusMessage(format!(
                                        "{}: {}",
                                        tr("Error abriendo escena", "Error opening scene", "Erro ao abrir cena"),
                                        e
                                    ));
                                }
                            }
                        }
                    }
                    ui.add_space(4.0);
                    if ui
                        .add_sized(
                            [ui.available_width(), 32.0],
                            egui::Button::new(RichText::new(tr("★ Crear Caso Demo Multi-Proceso", "★ Create Multi-Unit Demo Case", "★ Criar Caso Demo Multi-Processo")).strong().color(CREAM))
                                .fill(egui::Color32::from_rgb(0x7B, 0x2C, 0xBF)),
                        )
                        .clicked()
                    {
                        match self.create_demo_case(case_manager, library_manager) {
                            Ok(demo) => action = CasePanelAction::CaseUpdated(demo),
                            Err(e) => action = CasePanelAction::StatusMessage(format!("{} {}", tr("Error demo:", "Demo error:", "Erro demo:"), e)),
                        }
                    }
                });
            });

        // ==================================================================
        // COLUMNA DERECHA ESTRECHA (255 px — Sección 2): Escaneos y Librerías
        // ==================================================================
        egui::SidePanel::right("case_right_col")
            .exact_width(260.0)
            .resizable(false)
            .frame(Frame::NONE.fill(theme::BG_MAIN).inner_margin(Margin::same(8)))
            .show(ctx, |ui| {
                let mut any_scan = false;
                card_frame.show(ui, |ui| {
                    ui.label(RichText::new(tr("2. Escaneos 3D (STL/OBJ/PLY)", "2. 3D Scans (STL/OBJ/PLY)", "2. Escaneamentos 3D (STL/OBJ/PLY)")).strong().size(13.0).color(CREAM));
                    if let Some(case) = &self.current_case {
                        ui.label(RichText::new(&case.patient_name).strong().color(CREAM));
                        ui.label(RichText::new(case.format_order_summary()).size(10.5).color(CREAM_MUTED));
                    } else {
                        ui.label(RichText::new(tr("Ningún caso activo", "No active case", "Nenhum caso ativo")).color(CREAM_MUTED));
                    }
                    ui.separator();

                    let has_case = self.current_case.is_some();
                    let mut scan_to_import: Option<&'static str> = None;

                    for (key, title) in [
                        ("maxilla", tr("Maxilar Superior", "Upper Jaw (Maxilla)", "Arcada Superior (Maxila)")),
                        ("mandible", tr("Mandíbula Inferior", "Lower Jaw (Mandible)", "Arcada Inferior (Mandíbula)")),
                        ("bite", tr("Mordida / Oclusión", "Bite / Buccal Occlusion", "Registro Oclusal / Mordida")),
                    ] {
                        let existing = self
                            .current_case
                            .as_ref()
                            .and_then(|c| c.scans_raw.get(key))
                            .and_then(|o| o.as_ref())
                            .cloned();
                        if existing.is_some() {
                            any_scan = true;
                        }
                        ui.horizontal(|ui| {
                            ui.vertical(|ui| {
                                ui.label(RichText::new(title).strong().size(11.0));
                                if let Some(rel) = existing {
                                    let fname = Path::new(&rel)
                                        .file_name()
                                        .and_then(|s| s.to_str())
                                        .unwrap_or(&rel);
                                    ui.label(RichText::new(format!("✔ {}", fname)).size(10.0).color(EMERALD));
                                } else {
                                    ui.label(RichText::new(tr("Pendiente", "Pending", "Pendente")).size(10.0).color(CREAM_MUTED));
                                }
                            });
                            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                                if ui.add_enabled(has_case, egui::Button::new(tr("Cargar...", "Load...", "Carregar..."))).clicked() {
                                    scan_to_import = Some(key);
                                }
                            });
                        });
                        ui.add_space(3.0);
                    }

                    if let Some(scan_key) = scan_to_import {
                        if let Some(path) = rfd::FileDialog::new()
                            .set_title(format!("{} ({})", tr("Importar escaneo", "Import scan", "Importar escaneamento"), scan_key.to_uppercase()))
                            .add_filter(tr("Mallas 3D", "3D Meshes", "Malhas 3D"), &["stl", "obj", "ply"])
                            .pick_file()
                        {
                            if let Some(case) = &mut self.current_case {
                                match case_manager.import_scan(case, scan_key, &path) {
                                    Ok(_) => {
                                        let c = case.clone();
                                        self.refresh_cases(case_manager);
                                        action = CasePanelAction::CaseUpdated(c);
                                    }
                                    Err(e) => action = CasePanelAction::StatusMessage(e),
                                }
                            }
                        }
                    }

                    ui.separator();
                    if let Some(case) = &mut self.current_case {
                        ui.label(RichText::new(tr("Arcada de Trabajo:", "Working Arch:", "Arcada de Trabalho:")).size(11.0).color(CREAM_MUTED));
                        let mut changed = false;
                        egui::ComboBox::from_id_salt("prep_arch_combo")
                            .width(ui.available_width())
                            .selected_text(if case.prep_arch == "maxilla" {
                                tr("Maxilar (11-28)", "Upper / Maxilla (11-28)", "Maxila (11-28)")
                            } else {
                                tr("Mandíbula (31-48)", "Lower / Mandible (31-48)", "Mandíbula (31-48)")
                            })
                            .show_ui(ui, |ui| {
                                if ui.selectable_value(&mut case.prep_arch, "mandible".to_string(), tr("Mandíbula (31-48)", "Lower / Mandible (31-48)", "Mandíbula (31-48)")).clicked() {
                                    case.insertion_axis = [0.0, 0.0, 1.0];
                                    changed = true;
                                }
                                if ui.selectable_value(&mut case.prep_arch, "maxilla".to_string(), tr("Maxilar (11-28)", "Upper / Maxilla (11-28)", "Maxila (11-28)")).clicked() {
                                    case.insertion_axis = [0.0, 0.0, -1.0];
                                    changed = true;
                                }
                            });
                        if changed {
                            let _ = case_manager.save_case(case);
                        }
                    }
                });

                ui.add_space(8.0);
                card_frame.show(ui, |ui| {
                    ui.label(RichText::new(tr("Librerías Dentales", "Tooth Libraries", "Bibliotecas Dentárias")).strong().size(13.0).color(CREAM));
                    for lib in library_manager.list_libraries() {
                        ui.label(
                            RichText::new(format!("• {} ({} FDI)", lib.display_name, lib.fdi_map.len()))
                                .size(10.5)
                                .color(CREAM_MUTED),
                        );
                    }
                    ui.add_space(4.0);
                    if ui
                        .add_sized([ui.available_width(), 28.0], egui::Button::new(tr("+ Carpeta de Dientes...", "+ Tooth Folder...", "+ Pasta de Dentes...")))
                        .clicked()
                    {
                        if let Some(folder) = rfd::FileDialog::new()
                            .set_title(tr("Seleccionar Carpeta con Librería Dental (.STL/.OBJ/.PLY)", "Select Dental Library Folder (.STL/.OBJ/.PLY)", "Selecionar Pasta de Biblioteca Dental (.STL/.OBJ/.PLY)"))
                            .pick_folder()
                        {
                            match library_manager.import_library_from_folder(&folder) {
                                Ok(info) => {
                                    action = CasePanelAction::StatusMessage(format!(
                                        "{} '{}' ({} FDI).",
                                        tr("Librería instalada:", "Library installed:", "Biblioteca instalada:"),
                                        info.display_name,
                                        info.fdi_map.len()
                                    ));
                                }
                                Err(e) => action = CasePanelAction::StatusMessage(e),
                            }
                        }
                    }
                });

                ui.add_space(10.0);
                let open_enabled = self.current_case.is_some() && any_scan;
                ui.add_enabled_ui(open_enabled, |ui| {
                    if ui
                        .add_sized(
                            [ui.available_width(), 38.0],
                            egui::Button::new(
                                RichText::new(tr("Abrir en Visor 3D  →", "Open in 3D Model  →", "Abrir no Modelo 3D  →"))
                                    .strong()
                                    .color(egui::Color32::from_rgb(0x23, 0x03, 0x2E)),
                            )
                            .fill(CREAM),
                        )
                        .clicked()
                    {
                        if let Some(c) = self.current_case.clone() {
                            action = CasePanelAction::OpenInViewer(c);
                        }
                    }
                });
            });

        // ==================================================================
        // ZONA CENTRAL: Sub-columna 1A (Odontograma Vertical) + 1B (Multi-Proceso)
        // ==================================================================
        egui::CentralPanel::default()
            .frame(Frame::NONE.fill(theme::BG_MAIN).inner_margin(Margin::same(8)))
            .show(ctx, |ui| {
                card_frame.show(ui, |ui| {
                    ui.label(
                        RichText::new(tr("1. Odontograma en Óvalo Vertical y Configuración Multi-Proceso", "1. Dental Arch Chart & Multi-Unit Order Setup", "1. Odontograma em Arco e Configuração Multi-Processo"))
                            .strong()
                            .size(13.5)
                            .color(CREAM),
                    );
                    ui.add_space(4.0);

                    let total_w = ui.available_width();
                    let odonto_w = (total_w * 0.52).clamp(320.0, 520.0);
                    let form_w = (total_w - odonto_w - 14.0).max(260.0);

                    ui.horizontal_top(|ui| {
                        // Sub-columna 1A: Lienzo Vertical Completo del Odontograma
                        ui.allocate_ui(egui::vec2(odonto_w, ui.available_height()), |ui| {
                            let events = show_fdi_odontogram(ui, &mut self.odontogram, &self.draft_restorations);
                            for ev in events {
                                match ev {
                                    OdontogramEvent::SelectionChanged(Some(fdi)) => {
                                        if let Some(u) = self.draft_restorations.get(&fdi) {
                                            if let Some(pi) = APEX_PROCESS_TYPES.iter().position(|&p| p == u.process_type) {
                                                self.selected_process_idx = pi;
                                            }
                                            if let Some(mi) = APEX_MATERIAL_PRESETS.iter().position(|m| m.name == u.material) {
                                                self.selected_material_idx = mi;
                                            }
                                            if let Some(si) = VITA_SHADES.iter().position(|&s| s == u.shade) {
                                                self.selected_shade_idx = si;
                                            }
                                        }
                                        if let Some(case) = &mut self.current_case {
                                            if case.restorations.contains_key(&fdi) {
                                                case.set_active_tooth(fdi);
                                                let _ = case_manager.save_case(case);
                                            }
                                        }
                                    }
                                    OdontogramEvent::SelectionChanged(None) => {}
                                    OdontogramEvent::ConnectorToggled(t1, t2, link) => {
                                        self.toggle_connector(t1, t2, link, case_manager);
                                    }
                                }
                            }
                        });

                        ui.add_space(10.0);

                        // Sub-columna 1B: Controles de Proceso, Material, Tabla y Paciente
                        ui.allocate_ui(egui::vec2(form_w, ui.available_height()), |ui| {
                            ui.vertical(|ui| {
                                ui.label(RichText::new(tr("Asignar Proceso y Material a Pieza(s)", "Assign Restoration Type & Material", "Atribuir Processo e Material ao(s) Dente(s)")).strong().color(CREAM));
                                ui.horizontal(|ui| {
                                    ui.label(tr("Proceso:", "Type:", "Processo:"));
                                    egui::ComboBox::from_id_salt("combo_proc")
                                        .width((form_w - 155.0).max(120.0))
                                        .selected_text(i18n::tr_process_type(APEX_PROCESS_TYPES[self.selected_process_idx]))
                                        .show_ui(ui, |ui| {
                                            for (i, &p) in APEX_PROCESS_TYPES.iter().enumerate() {
                                                ui.selectable_value(&mut self.selected_process_idx, i, i18n::tr_process_type(p));
                                            }
                                        });
                                    ui.label("VITA:");
                                    egui::ComboBox::from_id_salt("combo_shade")
                                        .width(52.0)
                                        .selected_text(VITA_SHADES[self.selected_shade_idx])
                                        .show_ui(ui, |ui| {
                                            for (i, &s) in VITA_SHADES.iter().enumerate() {
                                                ui.selectable_value(&mut self.selected_shade_idx, i, s);
                                            }
                                        });
                                });

                                ui.horizontal(|ui| {
                                    ui.label(tr("Material:", "Material:", "Material:"));
                                    egui::ComboBox::from_id_salt("combo_mat")
                                        .width((form_w - 70.0).max(150.0))
                                        .selected_text(i18n::tr_material_name(APEX_MATERIAL_PRESETS[self.selected_material_idx].name))
                                        .show_ui(ui, |ui| {
                                            for (i, m) in APEX_MATERIAL_PRESETS.iter().enumerate() {
                                                ui.selectable_value(&mut self.selected_material_idx, i, i18n::tr_material_name(m.name));
                                            }
                                        });
                                });

                                let mat = &APEX_MATERIAL_PRESETS[self.selected_material_idx];
                                Frame::NONE
                                    .fill(theme::BG_DEEP)
                                    .stroke(Stroke::new(1.0_f32, BORDER_PLUM))
                                    .corner_radius(CornerRadius::same(5))
                                    .inner_margin(Margin::symmetric(7, 4))
                                    .show(ui, |ui| {
                                        ui.label(
                                            RichText::new(format!("⚙ {}", i18n::tr_material_desc(mat.name, mat.description)))
                                                .size(10.0)
                                                .color(EMERALD),
                                        );
                                    });

                                ui.add_space(4.0);
                                let sel_teeth = self.odontogram.get_selected_vec();
                                let assign_lbl = match sel_teeth.len() {
                                    0 => tr("+ Asignar a Pieza(s)", "+ Assign to Unit(s)", "+ Atribuir a Dente(s)").to_string(),
                                    1 => format!("{} #{}", tr("+ Asignar a", "+ Assign to", "+ Atribuir a"), sel_teeth[0]),
                                    n => format!("{} {} {}", tr("+ Asignar a", "+ Assign to", "+ Atribuir a"), n, tr("Piezas", "Units", "Dentes")),
                                };
                                ui.horizontal(|ui| {
                                    if theme::accent_button(ui, &assign_lbl).clicked() {
                                        if sel_teeth.is_empty() {
                                            action = CasePanelAction::StatusMessage(
                                                tr(
                                                    "Haz clic en uno o más dientes en el odontograma para seleccionarlos.",
                                                    "Click one or more teeth in the dental chart to select them.",
                                                    "Clique em um ou mais dentes no odontograma para selecioná-los.",
                                                ).to_string(),
                                            );
                                        } else {
                                            self.push_undo();
                                            let proc = APEX_PROCESS_TYPES[self.selected_process_idx];
                                            let shade = VITA_SHADES[self.selected_shade_idx];
                                            for &fdi in &sel_teeth {
                                                let mut u = ToothRestorationUnit::from_material_preset(
                                                    fdi, proc, mat.name, shade, Vec::new(),
                                                );
                                                if let Some(prev) = self.draft_restorations.get(&fdi) {
                                                    u.margin_points = prev.margin_points.clone();
                                                    u.margin_control_points = prev.margin_control_points.clone();
                                                    u.crown_design_path = prev.crown_design_path.clone();
                                                }
                                                self.draft_restorations.insert(fdi, u);
                                            }
                                            self.sync_draft_to_case(case_manager);
                                        }
                                    }

                                    if ui.button(tr("🔗 Armar Puente", "🔗 Create Bridge", "🔗 Montar Ponte")).clicked() {
                                        if sel_teeth.len() < 2 {
                                            action = CasePanelAction::StatusMessage(
                                                tr(
                                                    "Selecciona 2 o más piezas del mismo arco (o pulsa el círculo ○ entre dientes).",
                                                    "Select 2 or more units in the same arch (or click the ○ connector circle between teeth).",
                                                    "Selecione 2 ou mais dentes do mesmo arco (ou clique no círculo ○ entre dentes).",
                                                ).to_string(),
                                            );
                                        } else {
                                            self.push_undo();
                                            let shade = VITA_SHADES[self.selected_shade_idx];
                                            for arch in [&UPPER_ARCH, &LOWER_ARCH] {
                                                let mut idxs: Vec<usize> = sel_teeth
                                                    .iter()
                                                    .filter_map(|t| arch.iter().position(|x| x == t))
                                                    .collect();
                                                idxs.sort_unstable();
                                                if idxs.len() >= 2 {
                                                    let (lo, hi) = (idxs[0], *idxs.last().unwrap());
                                                    let span = &arch[lo..=hi];
                                                    for (pi, &fdi) in span.iter().enumerate() {
                                                        let proc = if pi == 0 || pi + 1 == span.len() {
                                                            "Pilar de Puente"
                                                        } else {
                                                            "Póntico de Puente"
                                                        };
                                                        let mut neigh = Vec::new();
                                                        if pi > 0 { neigh.push(span[pi - 1]); }
                                                        if pi + 1 < span.len() { neigh.push(span[pi + 1]); }
                                                        let mut u = ToothRestorationUnit::from_material_preset(
                                                            fdi, proc, mat.name, shade, neigh,
                                                        );
                                                        if let Some(prev) = self.draft_restorations.get(&fdi) {
                                                            u.margin_points = prev.margin_points.clone();
                                                            u.crown_design_path = prev.crown_design_path.clone();
                                                        }
                                                        self.draft_restorations.insert(fdi, u);
                                                    }
                                                }
                                            }
                                            self.sync_draft_to_case(case_manager);
                                        }
                                    }

                                    if ui.button(tr("✕ Quitar", "✕ Remove", "✕ Remover")).clicked() && !sel_teeth.is_empty() {
                                        self.push_undo();
                                        for &fdi in &sel_teeth {
                                            self.draft_restorations.remove(&fdi);
                                            for u in self.draft_restorations.values_mut() {
                                                u.connected_to.retain(|&x| x != fdi);
                                            }
                                        }
                                        self.sync_draft_to_case(case_manager);
                                    }
                                });

                                ui.separator();
                                ui.label(RichText::new(tr("Piezas en la Orden (Clic en fila para activar):", "Order Units (Click row to activate):", "Dentes na Ordem (Clique na linha para ativar):")).strong().size(11.0));

                                let mut row_clicked: Option<i32> = None;
                                egui::ScrollArea::vertical().max_height(155.0).show(ui, |ui| {
                                    for (&fdi, u) in &self.draft_restorations {
                                        let mat_t = i18n::tr_material_name(&u.material);
                                        let mat_s = mat_t.split('(').next().unwrap_or(&mat_t).trim();
                                        let conn_txt = if u.connected_to.is_empty() {
                                            format!("[○ {}]", tr("Individual", "Single", "Unitário"))
                                        } else {
                                            let cs = u.connected_to.iter().map(|c| format!("#{}", c)).collect::<Vec<_>>().join(",");
                                            format!("[● {} ↔ {}]", tr("Puente", "Bridge", "Ponte"), cs)
                                        };
                                        let col = theme::process_color(&u.process_type);
                                        let txt = format!(
                                            "#{} • {} {} | {} [{}] ({:.2}mm)",
                                            fdi, i18n::tr_process_type(&u.process_type), conn_txt, mat_s, u.shade, u.design_params.min_thickness_mm
                                        );
                                        let sel = self.odontogram.selected_tooth == Some(fdi);
                                        if ui.selectable_label(sel, RichText::new(txt).size(10.5).color(col)).clicked() {
                                            row_clicked = Some(fdi);
                                        }
                                    }
                                });
                                if let Some(fdi) = row_clicked {
                                    self.odontogram.set_single(fdi);
                                }

                                ui.separator();
                                ui.label(RichText::new(tr("Datos Clínicos de la Orden", "Clinical Order Details", "Dados Clínicos da Ordem")).strong().size(11.5));
                                egui::Grid::new("patient_form_grid").num_columns(2).spacing([8.0, 5.0]).show(ui, |ui| {
                                    ui.label(tr("Paciente *:", "Patient *:", "Paciente *:"));
                                    ui.add(egui::TextEdit::singleline(&mut self.patient_name).hint_text(tr("Nombre del paciente", "Patient name", "Nome do paciente")).desired_width(form_w - 105.0));
                                    ui.end_row();
                                    ui.label(tr("Odontólogo:", "Doctor:", "Dentista:"));
                                    ui.add(egui::TextEdit::singleline(&mut self.doctor_name).hint_text(tr("Clínica / Odontólogo", "Clinic / Doctor", "Clínica / Dentista")).desired_width(form_w - 105.0));
                                    ui.end_row();
                                    ui.label(tr("Notas Lab:", "Lab Notes:", "Notas Lab:"));
                                    ui.add(egui::TextEdit::multiline(&mut self.notes).desired_rows(2).desired_width(form_w - 105.0));
                                    ui.end_row();
                                });

                                ui.add_space(6.0);
                                ui.horizontal(|ui| {
                                    if theme::primary_button(ui, tr("Crear Nueva Orden", "Create New Order", "Criar Nova Ordem")).clicked() {
                                        if self.patient_name.trim().is_empty() {
                                            action = CasePanelAction::StatusMessage(tr("Ingresa el nombre del paciente.", "Enter patient name.", "Digite o nome do paciente.").to_string());
                                        } else {
                                            if self.draft_restorations.is_empty() {
                                                let fdi = self.odontogram.selected_tooth.unwrap_or(36);
                                                let mat_n = find_material_preset(APEX_MATERIAL_PRESETS[self.selected_material_idx].name).name;
                                                self.draft_restorations.insert(
                                                    fdi,
                                                    ToothRestorationUnit::from_material_preset(
                                                        fdi,
                                                        APEX_PROCESS_TYPES[self.selected_process_idx],
                                                        mat_n,
                                                        VITA_SHADES[self.selected_shade_idx],
                                                        Vec::new(),
                                                    ),
                                                );
                                            }
                                            let primary_fdi = self
                                                .odontogram
                                                .selected_tooth
                                                .filter(|t| self.draft_restorations.contains_key(t))
                                                .or_else(|| self.draft_restorations.keys().next().copied())
                                                .unwrap_or(36);
                                            let rest_desc = self
                                                .draft_restorations
                                                .get(&primary_fdi)
                                                .map(|u| format!("{} — {}", u.process_type, u.material))
                                                .unwrap_or_else(|| "Corona Individual".to_string());
                                            match case_manager.create_case(
                                                &self.patient_name,
                                                &self.doctor_name,
                                                primary_fdi,
                                                &rest_desc,
                                                VITA_SHADES[self.selected_shade_idx],
                                                &self.notes,
                                                self.draft_restorations.clone(),
                                            ) {
                                                Ok(new_c) => {
                                                    self.refresh_cases(case_manager);
                                                    self.set_active_case(new_c.clone());
                                                    action = CasePanelAction::CaseUpdated(new_c);
                                                }
                                                Err(e) => action = CasePanelAction::StatusMessage(e),
                                            }
                                        }
                                    }

                                    let can_save = self.current_case.is_some();
                                    if ui.add_enabled(can_save, egui::Button::new(tr("Guardar Cambios", "Save Changes", "Salvar Alterações"))).clicked() {
                                        if let Some(case) = &mut self.current_case {
                                            if !self.patient_name.trim().is_empty() {
                                                case.patient_name = self.patient_name.trim().to_string();
                                            }
                                            case.doctor_name = self.doctor_name.trim().to_string();
                                            case.notes = self.notes.trim().to_string();
                                            if !self.draft_restorations.is_empty() {
                                                case.restorations = self.draft_restorations.clone();
                                            }
                                            let _ = case_manager.save_case(case);
                                            let cloned = case.clone();
                                            self.refresh_cases(case_manager);
                                            action = CasePanelAction::CaseUpdated(cloned);
                                        }
                                    }
                                });
                            });
                        });
                    });
                });
            });

        action
    }
}
