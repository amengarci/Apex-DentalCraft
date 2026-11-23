//! Aplicación Principal **Apex DentalCraft** (`eframe::App`)
//!
//! Integra los 5 módulos CAD dentales nativos en Rust, cabecera con identidad Apex DentalCraft,
//! barra de etapas, atajo global `Ctrl + Z`, barra de estado y contexto OpenGL 3.3 `glow`.

use crate::core::case_manager::{CaseManager, DentalCase};
use crate::core::i18n::{self, tr, AppLanguage};
use crate::core::library_manager::ToothLibraryManager;
use crate::core::scene_file;
use crate::core::settings::AppSettings;
use crate::render::gl_renderer::GlViewportRenderer;
use crate::theme::{
    self, draw_apex_tooth_emblem, AMETHYST, AMETHYST_BRIGHT, BG_CARD, BG_DEEP, BG_WIDGET,
    BORDER_PLUM, CREAM, CREAM_MUTED, CYAN, EMERALD,
};
use crate::ui::bridge_sculpt_panel::{BridgeSculptAction, BridgeSculptPanelState};
use crate::ui::case_panel::{CasePanelAction, CasePanelState};
use crate::ui::design_panel::{DesignAction, DesignPanelState};
use crate::ui::export_panel::{ExportAction, ExportPanelState};
use crate::ui::margin_panel::{MarginAction, MarginPanelState};
use crate::ui::stl_viewer_panel::StlViewerPanelState;
use crate::ui::viewer_panel::{ViewerAction, ViewerPanelState};
use eframe::glow;
use egui::{Color32, CornerRadius, Frame, Margin, RichText, Stroke, Window};
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WorkflowStep {
    Cases = 0,
    Viewer = 1,
    Margin = 2,
    Design = 3,
    BridgeSculpt = 4,
    Export = 5,
    StlViewer = 6,
}

pub struct ApexDentalCraftApp {
    pub case_manager: CaseManager,
    pub library_manager: ToothLibraryManager,
    pub settings: AppSettings,
    pub show_settings_window: bool,
    pub renderer: Arc<Mutex<GlViewportRenderer>>,
    pub app_icon_tex: Option<egui::TextureHandle>,

    pub step: WorkflowStep,
    pub active_case: Option<DentalCase>,
    pub status_message: String,

    pub case_panel: CasePanelState,
    pub viewer_panel: ViewerPanelState,
    pub margin_panel: MarginPanelState,
    pub design_panel: DesignPanelState,
    pub bridge_sculpt_panel: BridgeSculptPanelState,
    pub export_panel: ExportPanelState,
    pub stl_viewer_panel: StlViewerPanelState,
}

impl ApexDentalCraftApp {
    pub fn new(cc: &eframe::CreationContext<'_>) -> Self {
        theme::apply_apex_theme(&cc.egui_ctx);
        let gl = cc
            .gl
            .as_ref()
            .expect("Apex DentalCraft requiere el backend OpenGL Glow de eframe");
        let renderer = Arc::new(Mutex::new(GlViewportRenderer::new(gl)));
        let app_icon_tex = eframe::icon_data::from_png_bytes(include_bytes!(
            "../../assets/apex_dentalcraft.png"
        ))
        .ok()
        .map(|icon| {
            let color_img = egui::ColorImage::from_rgba_unmultiplied(
                [icon.width as usize, icon.height as usize],
                &icon.rgba,
            );
            cc.egui_ctx.load_texture(
                "apex_dentalcraft_brand_icon",
                color_img,
                egui::TextureOptions::LINEAR,
            )
        });

        let case_manager = CaseManager::new(None);
        let settings = AppSettings::load_from_root(&case_manager.root_dir);
        i18n::set_language(settings.language);
        let library_manager = ToothLibraryManager::new();
        let case_panel = CasePanelState::new(&case_manager);
        let active_case = case_panel.current_case.clone();
        let mode_tag = if case_manager.is_portable {
            tr("MODO PORTABLE USB • Sin instalación", "PORTABLE USB MODE • No install", "MODO PORTÁTIL USB • Sem instalação")
        } else {
            "Offline"
        };
        let status_message = format!(
            "Apex DentalCraft {} ({})  |  Ctrl+Z {}  |  {}: {}  |  {}: {}",
            tr("listo", "ready", "pronto"),
            mode_tag,
            tr("Deshacer activo", "Undo active", "Desfazer ativo"),
            tr("Casos", "Cases", "Casos"),
            case_manager.root_dir.display(),
            tr("Librerías", "Libraries", "Bibliotecas"),
            library_manager.list_libraries().len()
        );

        let mut app = Self {
            case_manager,
            library_manager,
            settings,
            show_settings_window: false,
            renderer,
            app_icon_tex,
            step: WorkflowStep::Cases,
            active_case,
            status_message: String::new(),
            case_panel,
            viewer_panel: ViewerPanelState::default(),
            margin_panel: MarginPanelState::default(),
            design_panel: DesignPanelState::default(),
            bridge_sculpt_panel: BridgeSculptPanelState::default(),
            export_panel: ExportPanelState::default(),
            stl_viewer_panel: StlViewerPanelState::default(),
        };
        app.apply_settings_everywhere();

        app.status_message = status_message;

        // Si se lanzó ApexDentalCraft.exe pasando una escena .apexscene o mallas .stl/.obj/.ply
        // por línea de comandos ("Abrir con Apex DentalCraft"), abrirlas directamente!
        let cli_args: Vec<PathBuf> = std::env::args_os()
            .skip(1)
            .map(PathBuf::from)
            .filter(|p| p.exists())
            .collect();
        if let Some(scene_arg) = cli_args.iter().find(|p| scene_file::is_apex_scene_path(p)) {
            app.open_apex_scene_path(scene_arg);
        } else {
            let cli_meshes: Vec<PathBuf> = cli_args
                .into_iter()
                .filter(|p| {
                    p.is_file()
                        && p.extension()
                            .and_then(|s| s.to_str())
                            .map(|e| matches!(e.to_lowercase().as_str(), "stl" | "obj" | "ply"))
                            .unwrap_or(false)
                })
                .collect();
            if !cli_meshes.is_empty() {
                app.status_message = app.stl_viewer_panel.open_files(&cli_meshes);
                app.step = WorkflowStep::StlViewer;
            }
        }
        app
    }

    fn find_dropped_apex_scene(paths: &[PathBuf]) -> Option<PathBuf> {
        for p in paths {
            if scene_file::is_apex_scene_path(p) {
                return Some(p.clone());
            }
            if p.is_dir() {
                if let Ok(entries) = std::fs::read_dir(p) {
                    let mut scenes: Vec<PathBuf> = entries
                        .flatten()
                        .map(|e| e.path())
                        .filter(|cp| scene_file::is_apex_scene_path(cp))
                        .collect();
                    scenes.sort();
                    if let Some(first) = scenes.into_iter().next() {
                        return Some(first);
                    }
                }
            }
        }
        None
    }

    pub fn open_apex_scene_path(&mut self, scene_path: &Path) {
        match scene_file::import_scene_into_case_manager(scene_path, &self.case_manager) {
            Ok((loaded, _scene)) => {
                let patient = loaded.patient_name.clone();
                self.case_panel.refresh_cases(&self.case_manager);
                self.case_panel.set_active_case(loaded.clone());
                self.active_case = Some(loaded);
                let _ = self.stl_viewer_panel.open_apex_scene(scene_path);
                self.step = WorkflowStep::StlViewer;
                self.status_message = format!(
                    "✔ {}: '{}' ({})",
                    tr("Escena reabierta", "Scene reopened", "Cena reaberta"),
                    patient,
                    tr(
                        "editable en Módulos 1–6 y Visor 3D",
                        "editable in Modules 1–6 & 3D Viewer",
                        "editável nos Módulos 1–6 e Visor 3D",
                    )
                );
            }
            Err(e) => {
                self.status_message = format!(
                    "{}: {}",
                    tr("Error abriendo escena", "Error opening scene", "Erro ao abrir cena"),
                    e
                );
            }
        }
    }

    pub fn apply_settings_everywhere(&mut self) {
        self.settings.clamp_all();
        i18n::set_language(self.settings.language);
        self.viewer_panel.camera.rotate_sensitivity = self.settings.camera_rotate_sensitivity;
        self.viewer_panel.camera.zoom_sensitivity = self.settings.camera_zoom_sensitivity;
        self.viewer_panel.camera.invert_zoom = self.settings.camera_invert_zoom;
        self.export_panel.camera.rotate_sensitivity = self.settings.camera_rotate_sensitivity;
        self.export_panel.camera.zoom_sensitivity = self.settings.camera_zoom_sensitivity;
        self.export_panel.camera.invert_zoom = self.settings.camera_invert_zoom;
        self.margin_panel.apply_settings(&self.settings);
        self.design_panel.apply_settings(&self.settings);
        self.bridge_sculpt_panel.apply_settings(&self.settings);
        self.stl_viewer_panel.apply_settings(&self.settings);
    }

    fn show_settings_modal(&mut self, ctx: &egui::Context) {
        if !self.show_settings_window {
            return;
        }
        let mut open = self.show_settings_window;
        let mut changed = false;
        let mut apply_bevel_to_active = false;
        let mut reload_libs = false;

        Window::new(tr("⚙ Configuración — Apex DentalCraft", "⚙ Settings — Apex DentalCraft", "⚙ Configurações — Apex DentalCraft"))
            .id(egui::Id::new("apex_settings_window"))
            .open(&mut open)
            .default_pos(egui::pos2(380.0, 92.0))
            .default_width(470.0)
            .resizable(false)
            .collapsible(false)
            .show(ctx, |ui| {
                egui::ScrollArea::vertical().max_height(520.0).show(ui, |ui| {
                    ui.label(RichText::new(tr("0. Idioma de la Interfaz / Language", "0. Interface Language", "0. Idioma da Interface")).strong().color(CYAN));
                    ui.horizontal(|ui| {
                        for lang in AppLanguage::ALL {
                            let lbl = format!("{}  {}", lang.code(), lang.label());
                            if ui.selectable_label(self.settings.language == lang, lbl).clicked() {
                                self.settings.language = lang;
                                i18n::set_language(lang);
                                changed = true;
                            }
                        }
                    });
                    ui.separator();

                    ui.label(RichText::new(tr("1. Línea de Margen (Etapa 3 y 4)", "1. Finish Line / Margin (Stage 3 & 4)", "1. Linha de Término / Margem (Etapa 3 e 4)")).strong().color(EMERALD));
                    changed |= ui
                        .add(
                            egui::Slider::new(&mut self.settings.margin_line_radius_mm, 0.015..=0.120)
                                .text(tr("Grosor línea vectorial (mm)", "Vector margin thickness (mm)", "Espessura linha vetorial (mm)")),
                        )
                        .changed();
                    changed |= ui
                        .add(
                            egui::Slider::new(&mut self.settings.margin_node_radius_mm, 0.050..=0.220)
                                .text(tr("Tamaño de nodos de control (mm)", "Control node size (mm)", "Tamanho dos nós de controle (mm)")),
                        )
                        .changed();
                    changed |= ui
                        .checkbox(
                            &mut self.settings.margin_magnetic_snap_default,
                            tr("Imán magnético a cresta de hombro activo por defecto", "Magnetic margin snap enabled by default", "Ímã magnético no término ativo por padrão"),
                        )
                        .changed();
                    changed |= ui
                        .checkbox(
                            &mut self.settings.margin_live_red_validation,
                            tr("Validación en vivo de nodos en ROJO (socavado/cruce/cementado)", "Live RED node validation (undercut/cross/cement)", "Validação ao vivo de nós em VERMELHO (retenção/cruzamento/cimento)"),
                        )
                        .changed();

                    ui.separator();
                    ui.label(RichText::new(tr("2. Cementado y Bisel Marginal por Defecto", "2. Default Cement Space & Margin Bevel", "2. Espaço de Cimento e Bisel Marginal Padrão")).strong().color(CYAN));
                    changed |= ui
                        .add(
                            egui::Slider::new(&mut self.settings.default_margin_bevel_mm, 0.00..=0.25)
                                .text(tr("Bisel marginal por defecto (mm)", "Default margin bevel (mm)", "Bisel marginal padrão (mm)")),
                        )
                        .changed();
                    changed |= ui
                        .add(
                            egui::Slider::new(&mut self.settings.default_margin_bevel_angle_deg, 25.0..=70.0)
                                .text(tr("Ángulo de bisel marginal (°)", "Margin bevel angle (°)", "Ângulo do bisel marginal (°)")),
                        )
                        .changed();
                    changed |= ui
                        .checkbox(
                            &mut self.settings.default_auto_adapt_antagonist,
                            tr("Recortar oclusión al antagonista automáticamente al adaptar", "Auto-trim occlusal interferences when adapting", "Recortar oclusão ao antagonista automaticamente ao adaptar"),
                        )
                        .changed();
                    if self.active_case.is_some() {
                        if ui.button(tr("Aplicar este bisel y opciones al caso activo ahora", "Apply bevel & options to active case now", "Aplicar este bisel e opções ao caso ativo agora")).clicked() {
                            apply_bevel_to_active = true;
                        }
                    }

                    ui.separator();
                    ui.label(RichText::new(tr("3. Transparencias Iniciales al Entrar a Etapa 4", "3. Initial Opacity When Entering Stage 4", "3. Transparências Iniciais ao Entrar na Etapa 4")).strong().color(AMETHYST_BRIGHT));
                    changed |= ui
                        .add(
                            egui::Slider::new(&mut self.settings.stage4_restoration_opacity, 0.10..=1.00)
                                .custom_formatter(|v, _| format!("{:.0}%", v * 100.0))
                                .text(tr("Opacidad Restauración", "Restoration Opacity", "Opacidade da Restauração")),
                        )
                        .changed();
                    changed |= ui
                        .add(
                            egui::Slider::new(&mut self.settings.stage4_model_opacity, 0.10..=1.00)
                                .custom_formatter(|v, _| format!("{:.0}%", v * 100.0))
                                .text(tr("Opacidad Modelo / Muñón", "Model / Die Opacity", "Opacidade Modelo / Preparo")),
                        )
                        .changed();
                    changed |= ui
                        .checkbox(&mut self.settings.stage4_show_antagonist, tr("Mostrar Antagonista al entrar a Etapa 4", "Show Opposing Arch on Stage 4 entry", "Mostrar Antagonista ao entrar na Etapa 4"))
                        .changed();
                    changed |= ui
                        .checkbox(&mut self.settings.stage4_show_cement, tr("Mostrar Espacio de Cementado al entrar", "Show Cement Space on entry", "Mostrar Espaço de Cimento ao entrar"))
                        .changed();
                    changed |= ui
                        .add(
                            egui::Slider::new(&mut self.settings.stage4_cement_opacity, 0.05..=1.00)
                                .custom_formatter(|v, _| format!("{:.0}%", v * 100.0))
                                .text(tr("Opacidad Espacio de Cementado", "Cement Space Opacity", "Opacidade Espaço de Cimento")),
                        )
                        .changed();
                    changed |= ui
                        .checkbox(&mut self.settings.stage4_show_min_thickness, tr("Mostrar Grosor Mínimo al entrar", "Show Min. Thickness on entry", "Mostrar Espessura Mínima ao entrar"))
                        .changed();
                    changed |= ui
                        .add(
                            egui::Slider::new(&mut self.settings.stage4_min_thickness_opacity, 0.05..=1.00)
                                .custom_formatter(|v, _| format!("{:.0}%", v * 100.0))
                                .text(tr("Opacidad Grosor Mínimo", "Min. Thickness Opacity", "Opacidade Espessura Mínima")),
                        )
                        .changed();

                    ui.separator();
                    ui.label(RichText::new(tr("4. Cámara 3D Libre 360° y Visor STL", "4. 360° 3D Camera & STL Viewer", "4. Câmera 3D Livre 360° e Visor STL")).strong().color(CREAM));
                    changed |= ui
                        .add(
                            egui::Slider::new(&mut self.settings.camera_rotate_sensitivity, 0.40..=2.20)
                                .text(tr("Sensibilidad de rotación", "Orbit sensitivity", "Sensibilidade de rotação")),
                        )
                        .changed();
                    changed |= ui
                        .add(
                            egui::Slider::new(&mut self.settings.camera_zoom_sensitivity, 0.40..=2.20)
                                .text(tr("Sensibilidad de zoom rueda", "Wheel zoom sensitivity", "Sensibilidade de zoom")),
                        )
                        .changed();
                    changed |= ui
                        .checkbox(&mut self.settings.camera_invert_zoom, tr("Invertir dirección del zoom", "Invert zoom direction", "Inverter direção do zoom"))
                        .changed();
                    changed |= ui
                        .add(
                            egui::Slider::new(&mut self.settings.viewer_min_thickness_warn_mm, 0.25..=1.50)
                                .text(tr("Umbral alerta grosor Visor STL (mm)", "STL Viewer thin-wall alert (mm)", "Limite alerta espessura Visor STL (mm)")),
                        )
                        .changed();

                    ui.separator();
                    ui.label(RichText::new(tr("5. Almacenamiento y Portabilidad", "5. Storage & Portability", "5. Armazenamento e Portabilidade")).strong().color(CREAM));
                    ui.label(
                        RichText::new(format!(
                            "{}: {}  |  {}: {}",
                            tr("Modo", "Mode", "Modo"),
                            if self.case_manager.is_portable { tr("USB Portable", "Portable USB", "USB Portátil") } else { tr("Local", "Local", "Local") },
                            tr("Raíz", "Root", "Pasta"),
                            self.case_manager.root_dir.display()
                        ))
                        .size(10.0)
                        .color(CREAM_MUTED),
                    );
                    ui.horizontal(|ui| {
                        if ui.button(tr("↻ Recargar Librerías Dentales", "↻ Reload Tooth Libraries", "↻ Recarregar Bibliotecas Dentárias")).clicked() {
                            reload_libs = true;
                        }
                        if ui.button(tr("Restaurar Valores por Defecto", "Restore Defaults", "Restaurar Padrões")).clicked() {
                            self.settings = AppSettings::default();
                            i18n::set_language(self.settings.language);
                            changed = true;
                        }
                    });
                    if ui
                        .button(tr("📌 Asociar .STL/.OBJ/.PLY con Apex 3D Viewer en Windows", "📌 Associate .STL/.OBJ/.PLY with Apex 3D Viewer in Windows", "📌 Associar .STL/.OBJ/.PLY ao Apex 3D Viewer no Windows"))
                        .on_hover_text(tr(
                            "Registra Apex3DViewer.exe en clic derecho → Abrir con... para cualquier STL del PC",
                            "Registers Apex3DViewer.exe in Windows right-click → Open with... for any 3D mesh",
                            "Registra Apex3DViewer.exe no clique direito → Abrir com... no Windows",
                        ))
                        .clicked()
                    {
                        if let Some(vexe) = crate::core::win_assoc::find_standalone_viewer_exe() {
                            match crate::core::win_assoc::register_windows_open_with(&vexe, true) {
                                Ok(m) => self.status_message = m,
                                Err(e) => self.status_message = e,
                            }
                        } else {
                            self.status_message = tr(
                                "Asegúrate de tener Apex3DViewer.exe en la misma carpeta que ApexDentalCraft.exe.",
                                "Make sure Apex3DViewer.exe is in the same folder as ApexDentalCraft.exe.",
                                "Certifique-se de que Apex3DViewer.exe está na mesma pasta de ApexDentalCraft.exe.",
                            ).to_string();
                        }
                    }
                });
            });

        self.show_settings_window = open;
        if reload_libs {
            let n = self.library_manager.refresh_libraries().len();
            self.status_message = format!("{} {}", tr("Librerías recargadas:", "Libraries reloaded:", "Bibliotecas recarregadas:"), n);
        }
        if apply_bevel_to_active {
            if let Some(case) = &mut self.active_case {
                case.design_params.margin_bevel_mm = self.settings.default_margin_bevel_mm;
                case.design_params.margin_bevel_angle_deg = self.settings.default_margin_bevel_angle_deg;
                case.design_params.auto_adapt_antagonist = self.settings.default_auto_adapt_antagonist;
                let _ = self.case_manager.save_case(case);
                self.design_panel.rebuild_auxiliary_shells(case);
                self.status_message = format!(
                    "{} ({:.2} mm / {:.0}°) {}.",
                    tr("Bisel marginal", "Margin bevel", "Bisel marginal"),
                    case.design_params.margin_bevel_mm, case.design_params.margin_bevel_angle_deg,
                    tr("aplicado al caso activo", "applied to active case", "aplicado ao caso ativo")
                );
            }
        }
        if changed {
            self.apply_settings_everywhere();
            self.margin_panel.magnetic_snap = self.settings.margin_magnetic_snap_default;
            if let Some(case) = &self.active_case {
                self.margin_panel.refresh_audit(case, false);
            }
            let _ = self.settings.save_to_root(&self.case_manager.root_dir);
            self.status_message = tr(
                "Configuración guardada y aplicada en vivo.",
                "Settings saved and applied live.",
                "Configurações salvas e aplicadas ao vivo.",
            ).to_string();
        }
    }

    pub fn switch_step(&mut self, target: WorkflowStep) {
        if target == self.step {
            return;
        }

        // Antes de salir de Etapa 4 (Diseño), persistir cualquier ajuste en disco
        // para que Puente Final / Etapa 5 vea siempre todas las coronas, pónticos y conectores al día.
        if self.step == WorkflowStep::Design {
            if let Some(case) = &mut self.active_case {
                self.design_panel.save_all_restorations(case, &self.case_manager);
            }
        }
        if self.step == WorkflowStep::BridgeSculpt {
            if let Some(case) = &self.active_case {
                self.bridge_sculpt_panel.save_all_bridges(case, &self.case_manager);
            }
        }

        if self.step == WorkflowStep::Margin && target == WorkflowStep::Design {
            if let Some(c) = &mut self.active_case {
                let is_pontic = c.get_active_unit().map(|u| u.is_pontic()).unwrap_or(false);
                if !is_pontic {
                    self.margin_panel.refresh_audit(c, false);
                    if self.margin_panel.margin_ctrl.control_points.len() >= 4
                        && self.margin_panel.margin_audit.blocks_design
                    {
                        self.status_message = self.margin_panel.margin_audit.message.clone();
                        return;
                    }
                    if self.margin_panel.margin_ctrl.control_points.len() >= 4
                        && self.margin_panel.margin_audit.has_undercut_warning
                    {
                        let _ = self
                            .margin_panel
                            .auto_optimize_insertion_axis(c, &self.case_manager);
                    }
                    if self.margin_panel.margin_ctrl.control_points.len() >= 4 {
                        self.margin_panel.persist_margin(c, &self.case_manager);
                    }
                }
            }
        }

        if target == WorkflowStep::StlViewer {
            // El Visor STL puede abrirse en cualquier momento (con o sin caso activo).
            // Si está vacío y existe un caso activo, ofrecer/precargar la escena automáticamente.
            if self.stl_viewer_panel.objects.is_empty() {
                if let Some(c) = &self.active_case {
                    self.status_message = self
                        .stl_viewer_panel
                        .load_active_case_scene(c, &self.case_manager);
                } else {
                    self.status_message = tr(
                        "Visor STL 3D listo: arrastra archivos .stl/.obj aquí o pulsa '📂 Abrir STL / OBJ...'.",
                        "3D STL Viewer ready: drag .stl/.obj/.ply files here or click '📂 Open STL / OBJ...'.",
                        "Visor STL 3D pronto: arraste arquivos .stl/.obj aqui ou clique em '📂 Abrir STL / OBJ...'.",
                    ).to_string();
                }
            }
            self.step = WorkflowStep::StlViewer;
            return;
        }

        if target != WorkflowStep::Cases {
            // Sincronizar desde Etapa 1 únicamente al venir de Casos o si no hay caso activo
            if self.step == WorkflowStep::Cases || self.active_case.is_none() {
                if let Some(c) = &self.case_panel.current_case {
                    if let Ok(reloaded) = self.case_manager.load_case(&c.case_id) {
                        self.active_case = Some(reloaded);
                    } else {
                        self.active_case = Some(c.clone());
                    }
                }
            }
            let Some(case_clone) = self.active_case.clone() else {
                self.status_message = tr(
                    "Primero crea o selecciona un caso dental en el Paso 1 (o pulsa '★ Crear Caso Demo').",
                    "First create or select a dental case in Step 1 (or click '★ Create Demo Case').",
                    "Primeiro crie ou selecione um caso odontológico na Etapa 1 (ou clique em '★ Criar Caso Demo').",
                ).to_string();
                return;
            };

            match target {
                WorkflowStep::Viewer => {
                    self.viewer_panel.load_case(&case_clone, &self.case_manager);
                    self.status_message = tr(
                        "Paso 2: Ventanas flotantes desplegables para limpiar, recortar, añadir zócalo o fijar eje de inserción.",
                        "Step 2: Floating tool panels to clean scans, trim, build solid/hollow bases, and set insertion axis.",
                        "Etapa 2: Janelas flutuantes para limpar malhas, recortar, gerar base sólida/oca e definir eixo de inserção.",
                    ).to_string();
                }
                WorkflowStep::Margin => {
                    if let Some(c_mut) = &mut self.active_case {
                        self.margin_panel.load_case(c_mut, &self.case_manager);
                    }
                    self.status_message = tr(
                        "Paso 3: Puro Clic Izq en muñón para margen (Izq+Der borra nodo; Clic Der rota). Pónticos sin margen.",
                        "Step 3: Left-Click on die for finish line (Left+Right deletes node; Right-Drag orbits). Pontics need no margin.",
                        "Etapa 3: Clique Esq no preparo para traçar término (Esq+Dir exclui nó; Clique Dir gira). Pônticos sem término.",
                    ).to_string();
                }
                WorkflowStep::Design => {
                    if let Some(c_mut) = &mut self.active_case {
                        self.design_panel
                            .load_case(c_mut, &self.case_manager, &self.library_manager);
                    }
                    self.status_message = tr(
                        "Paso 4: Todas las piezas visibles | Clic Der rota 360° | Clic Izq edita Gizmo, Morphing, Esculpido y Conectores.",
                        "Step 4: All units visible | Right-Drag orbits 360° | Left-Click edits Gizmo, Morphing, Sculpting & Connectors.",
                        "Etapa 4: Todos os dentes visíveis | Clique Dir gira 360° | Clique Esq edita Gizmo, Morphing, Escultura e Conectores.",
                    ).to_string();
                }
                WorkflowStep::BridgeSculpt => {
                    self.bridge_sculpt_panel.load_case(&case_clone, &self.case_manager, false);
                    self.status_message = tr(
                        "Paso 5 (Puente Final): Esculpido libre y Morphing 3D sobre el puente unido con conectores antes de exportar.",
                        "Step 5 (Final Bridge): Free sculpting, 3D Morphing & Smart Tools on the single united bridge prior to CAM export.",
                        "Etapa 5 (Ponte Final): Escultura livre, Morphing 3D e ferramentas inteligentes na ponte unida antes da exportação.",
                    ).to_string();
                }
                WorkflowStep::Export => {
                    self.export_panel.load_case(&case_clone, &self.case_manager);
                    self.status_message = tr(
                        "Exportación CAM: Revisa la auditoría watertight y exporta el STL final (o ábrelo en el Visor STL).",
                        "CAM Export: Verify watertight audit and export final STL/OBJ (or inspect in 3D Viewer).",
                        "Exportação CAM: Verifique a auditoria watertight e exporte o STL final (ou abra no Visor STL).",
                    ).to_string();
                }
                WorkflowStep::Cases | WorkflowStep::StlViewer => {}
            }
        } else {
            self.case_panel.refresh_cases(&self.case_manager);
            self.library_manager.refresh_libraries();
        }
        self.step = target;
    }

    pub fn trigger_global_undo(&mut self) {
        let did_undo = match self.step {
            WorkflowStep::Cases => {
                let ok = self.case_panel.undo(&self.case_manager);
                if ok {
                    self.active_case = self.case_panel.current_case.clone();
                }
                ok
            }
            WorkflowStep::Viewer => {
                if let Some(case) = &mut self.active_case {
                    self.viewer_panel.undo(case, &self.case_manager)
                } else {
                    false
                }
            }
            WorkflowStep::Margin => {
                if let Some(case) = &mut self.active_case {
                    self.margin_panel.undo(case, &self.case_manager)
                } else {
                    false
                }
            }
            WorkflowStep::Design => {
                if let Some(case) = &mut self.active_case {
                    self.design_panel.undo(case, &self.case_manager)
                } else {
                    false
                }
            }
            WorkflowStep::BridgeSculpt => {
                if let Some(case) = &self.active_case {
                    self.bridge_sculpt_panel.undo(case, &self.case_manager)
                } else {
                    false
                }
            }
            WorkflowStep::Export | WorkflowStep::StlViewer => false,
        };

        if did_undo {
            self.status_message = format!("↶ {} {} (Ctrl + Z).", tr("Deshacer ejecutado en Módulo", "Undo applied in Module", "Desfazer executado no Módulo"), self.step as usize + 1);
        } else {
            self.status_message = tr(
                "No hay más operaciones recientes para deshacer en este módulo.",
                "No more recent actions to undo in this module.",
                "Não há mais ações recentes para desfazer neste módulo.",
            ).to_string();
        }
    }
}

impl eframe::App for ApexDentalCraftApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        // Arrastrar y soltar archivos .apexscene / .stl / .obj / .ply directamente en cualquier parte de Apex DentalCraft
        let dropped_paths: Vec<PathBuf> = ctx.input(|i| {
            i.raw
                .dropped_files
                .iter()
                .filter_map(|f| f.path.clone())
                .filter(|p| {
                    p.is_dir()
                        || p.extension()
                            .and_then(|s| s.to_str())
                            .map(|e| matches!(e.to_lowercase().as_str(), "stl" | "obj" | "ply" | "apexscene"))
                            .unwrap_or(false)
                })
                .collect()
        });
        if !dropped_paths.is_empty() {
            if let Some(scene_p) = Self::find_dropped_apex_scene(&dropped_paths) {
                self.open_apex_scene_path(&scene_p);
            } else {
                self.status_message = self.stl_viewer_panel.open_files(&dropped_paths);
                self.step = WorkflowStep::StlViewer;
            }
        }

        // Atajo global Ctrl + Z (cuando ningún campo de texto tiene el foco de teclado)
        let ctrl_z = ctx.input(|i| i.modifiers.ctrl && i.key_pressed(egui::Key::Z));
        if ctrl_z && !ctx.wants_keyboard_input() {
            self.trigger_global_undo();
        }

        // ==================================================================
        // CABECERA SUPERIOR: Identidad Apex DentalCraft + Stepper 1..5 + Visor STL + Config + Ctrl+Z
        // ==================================================================
        egui::TopBottomPanel::top("apex_top_bar")
            .exact_height(60.0)
            .frame(
                Frame::NONE
                    .fill(BG_CARD)
                    .stroke(Stroke::new(1.0_f32, Color32::from_rgb(0x57, 0x1B, 0x6E)))
                    .inner_margin(Margin::symmetric(12, 8)),
            )
            .show(ctx, |ui| {
                ui.horizontal_centered(|ui| {
                    // Icono oficial Apex DentalCraft
                    if let Some(tex) = &self.app_icon_tex {
                        ui.add(
                            egui::Image::from_texture(tex)
                                .fit_to_exact_size(egui::vec2(38.0, 38.0)),
                        );
                    } else {
                        let (emblem_rect, _) =
                            ui.allocate_exact_size(egui::vec2(36.0, 36.0), egui::Sense::hover());
                        draw_apex_tooth_emblem(ui.painter(), emblem_rect);
                    }

                    ui.label(RichText::new("APEX DENTALCRAFT").strong().size(15.0).color(CREAM));

                    ui.add_space(8.0);

                    let has_bridge = self
                        .active_case
                        .as_ref()
                        .map(|c| c.has_bridge())
                        .unwrap_or(false);
                    let mut steps = vec![
                        (WorkflowStep::Cases, tr("1. Casos", "1. Cases", "1. Casos")),
                        (WorkflowStep::Viewer, tr("2. Modelo 3D", "2. 3D Model", "2. Modelo 3D")),
                        (WorkflowStep::Margin, tr("3. Margen", "3. Margin", "3. Término")),
                        (WorkflowStep::Design, tr("4. Diseño", "4. Design", "4. Desenho")),
                    ];
                    if has_bridge {
                        steps.push((WorkflowStep::BridgeSculpt, tr("5. Puente Final", "5. Final Bridge", "5. Ponte Final")));
                        steps.push((WorkflowStep::Export, tr("6. Exportar", "6. Export", "6. Exportar")));
                    } else {
                        steps.push((WorkflowStep::Export, tr("5. Exportar", "5. Export", "5. Exportar")));
                    }
                    steps.push((WorkflowStep::StlViewer, tr("🔍 Visor STL", "🔍 3D Viewer", "🔍 Visor STL")));
                    let mut wanted_step = None;
                    for (st, title) in steps {
                        let active = self.step == st;
                        let is_stl_v = st == WorkflowStep::StlViewer;
                        let (fill, txt_col, stroke) = if active {
                            (CREAM, Color32::from_rgb(0x26, 0x04, 0x33), Stroke::new(1.2_f32, Color32::WHITE))
                        } else if is_stl_v {
                            (Color32::from_rgb(0x13, 0x31, 0x3D), CYAN, Stroke::new(1.0_f32, CYAN))
                        } else {
                            (BG_WIDGET, CREAM_MUTED, Stroke::new(1.0_f32, BORDER_PLUM))
                        };
                        if ui
                            .add(
                                egui::Button::new(RichText::new(title).strong().size(11.2).color(txt_col))
                                    .fill(fill)
                                    .stroke(stroke)
                                    .corner_radius(CornerRadius::same(6)),
                            )
                            .clicked()
                        {
                            wanted_step = Some(st);
                        }
                    }
                    if let Some(ws) = wanted_step {
                        self.switch_step(ws);
                    }

                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        // Botón ⚙ Configuración
                        let cfg_fill = if self.show_settings_window {
                            AMETHYST
                        } else {
                            Color32::from_rgb(0x2B, 0x07, 0x38)
                        };
                        if ui
                            .add(
                                egui::Button::new(RichText::new(tr("⚙ Config.", "⚙ Settings", "⚙ Config.")).strong().size(11.0).color(CREAM))
                                    .fill(cfg_fill)
                                    .stroke(Stroke::new(1.0_f32, AMETHYST_BRIGHT))
                                    .corner_radius(CornerRadius::same(6)),
                            )
                            .on_hover_text(tr("Abrir configuración global de Apex DentalCraft", "Open Apex DentalCraft global settings", "Abrir configurações globais do Apex DentalCraft"))
                            .clicked()
                        {
                            self.show_settings_window = !self.show_settings_window;
                        }

                        ui.add_space(4.0);
                        if ui
                            .add(
                                egui::Button::new(RichText::new("↶ Ctrl+Z").strong().size(11.0).color(CREAM))
                                    .fill(Color32::from_rgb(0x2B, 0x07, 0x38))
                                    .stroke(Stroke::new(1.0_f32, AMETHYST))
                                    .corner_radius(CornerRadius::same(6)),
                            )
                            .on_hover_text(tr("Deshacer última acción (Ctrl + Z)", "Undo last action (Ctrl + Z)", "Desfazer última ação (Ctrl + Z)"))
                            .clicked()
                        {
                            self.trigger_global_undo();
                        }

                        if self.case_manager.is_portable {
                            ui.add_space(4.0);
                            Frame::NONE
                                .fill(Color32::from_rgba_unmultiplied(0x34, 0xD3, 0x99, 28))
                                .stroke(Stroke::new(1.0_f32, EMERALD))
                                .corner_radius(CornerRadius::same(5))
                                .inner_margin(Margin::symmetric(5, 2))
                                .show(ui, |ui| {
                                    ui.label(RichText::new("USB").strong().size(9.5).color(EMERALD));
                                });
                        }

                        ui.add_space(6.0);
                        let badge = if let Some(c) = &self.active_case {
                            format!("● {} | {}", c.patient_name, c.format_order_summary())
                        } else if self.case_manager.is_portable {
                            "● Portable USB".to_string()
                        } else {
                            "● Offline Nativo".to_string()
                        };
                        ui.label(RichText::new(badge).strong().size(10.5).color(EMERALD));
                    });
                });
            });

        // Ventana flotante de Configuración Global (cuando está abierta)
        self.show_settings_modal(ctx);

        // ==================================================================
        // BARRA DE ESTADO INFERIOR
        // ==================================================================
        egui::TopBottomPanel::bottom("apex_status_bar")
            .exact_height(24.0)
            .frame(
                Frame::NONE
                    .fill(BG_DEEP)
                    .stroke(Stroke::new(1.0_f32, BORDER_PLUM))
                    .inner_margin(Margin::symmetric(12, 3)),
            )
            .show(ctx, |ui| {
                ui.horizontal_centered(|ui| {
                    ui.label(RichText::new(&self.status_message).size(10.5).color(CREAM_MUTED));
                });
            });

        // ==================================================================
        // CONTENIDO PRINCIPAL DEL MÓDULO ACTIVO
        // ==================================================================
        match self.step {
            WorkflowStep::Cases => {
                match self
                    .case_panel
                    .ui(ctx, &self.case_manager, &mut self.library_manager)
                {
                    CasePanelAction::None => {}
                    CasePanelAction::CaseUpdated(c) => {
                        self.active_case = Some(c);
                    }
                    CasePanelAction::OpenInViewer(c) => {
                        self.active_case = Some(c);
                        self.switch_step(WorkflowStep::Viewer);
                    }
                    CasePanelAction::OpenApexScene(c, scene_path) => {
                        let patient = c.patient_name.clone();
                        self.active_case = Some(c);
                        let _ = self.stl_viewer_panel.open_apex_scene(&scene_path);
                        self.step = WorkflowStep::StlViewer;
                        self.status_message = format!(
                            "✔ {}: '{}' ({})",
                            tr("Escena reabierta", "Scene reopened", "Cena reaberta"),
                            patient,
                            tr(
                                "editable en Módulos 1–6 y Visor 3D",
                                "editable in Modules 1–6 & 3D Viewer",
                                "editável nos Módulos 1–6 e Visor 3D",
                            )
                        );
                    }
                    CasePanelAction::StatusMessage(msg) => {
                        self.status_message = msg;
                    }
                }
            }
            WorkflowStep::Viewer => {
                if let Some(mut case) = self.active_case.take() {
                    let act = self
                        .viewer_panel
                        .ui(ctx, &mut case, &self.case_manager, &self.renderer);
                    self.active_case = Some(case);
                    match act {
                        ViewerAction::None => {}
                        ViewerAction::ProceedToMargin => self.switch_step(WorkflowStep::Margin),
                        ViewerAction::StatusMessage(m) => self.status_message = m,
                    }
                }
            }
            WorkflowStep::Margin => {
                if let Some(mut case) = self.active_case.take() {
                    let act = self
                        .margin_panel
                        .ui(ctx, &mut case, &self.case_manager, &self.renderer);
                    self.active_case = Some(case);
                    match act {
                        MarginAction::None => {}
                        MarginAction::ProceedToDesign => self.switch_step(WorkflowStep::Design),
                        MarginAction::StatusMessage(m) => self.status_message = m,
                    }
                }
            }
            WorkflowStep::Design => {
                if let Some(mut case) = self.active_case.take() {
                    let act = self.design_panel.ui(
                        ctx,
                        &mut case,
                        &self.case_manager,
                        &mut self.library_manager,
                        &self.renderer,
                    );
                    self.active_case = Some(case);
                    match act {
                        DesignAction::None => {}
                        DesignAction::ProceedToExport => {
                            let has_bridge = self
                                .active_case
                                .as_ref()
                                .map(|c| c.has_bridge())
                                .unwrap_or(false);
                            if has_bridge {
                                self.switch_step(WorkflowStep::BridgeSculpt);
                            } else {
                                self.switch_step(WorkflowStep::Export);
                            }
                        }
                        DesignAction::ReturnToMargin => self.switch_step(WorkflowStep::Margin),
                        DesignAction::StatusMessage(m) => self.status_message = m,
                    }
                }
            }
            WorkflowStep::BridgeSculpt => {
                if let Some(mut case) = self.active_case.take() {
                    let act = self
                        .bridge_sculpt_panel
                        .ui(ctx, &mut case, &self.case_manager, &self.renderer);
                    self.active_case = Some(case);
                    match act {
                        BridgeSculptAction::None => {}
                        BridgeSculptAction::ReturnToDesign => self.switch_step(WorkflowStep::Design),
                        BridgeSculptAction::ProceedToExport => self.switch_step(WorkflowStep::Export),
                        BridgeSculptAction::StatusMessage(m) => self.status_message = m,
                    }
                }
            }
            WorkflowStep::Export => {
                if let Some(mut case) = self.active_case.take() {
                    let act = self
                        .export_panel
                        .ui(ctx, &mut case, &self.case_manager, &self.renderer);
                    self.active_case = Some(case);
                    match act {
                        ExportAction::None => {}
                        ExportAction::OpenInStlViewer => {
                            if let Some(c) = &self.active_case {
                                self.status_message = self
                                    .stl_viewer_panel
                                    .load_active_case_scene(c, &self.case_manager);
                            }
                            self.step = WorkflowStep::StlViewer;
                        }
                        ExportAction::StatusMessage(m) => {
                            self.status_message = m;
                        }
                    }
                }
            }
            WorkflowStep::StlViewer => {
                let msg = self.stl_viewer_panel.ui(
                    ctx,
                    self.active_case.as_ref(),
                    &self.case_manager,
                    &self.renderer,
                );
                if let Some(m) = msg {
                    self.status_message = m;
                }
            }
        }
    }

    fn on_exit(&mut self, gl: Option<&glow::Context>) {
        if let Some(gl_ctx) = gl {
            if let Ok(mut guard) = self.renderer.lock() {
                guard.destroy(gl_ctx);
            }
        }
    }
}
