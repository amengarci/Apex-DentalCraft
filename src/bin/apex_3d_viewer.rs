//! **Apex 3D Viewer** — Programa Independiente de Visualización 3D Dental (estilo 3Shape 3D Viewer).
//!
//! Ultra-ligero, arranque instantáneo en OpenGL 3.3 Core + `parry3d` BVH:
//! - Abre cualquier `.stl`, `.obj` o `.ply` del PC (vía clic derecho → "Abrir con...", doble clic tras
//!   asociar en Windows, arrastrar y soltar archivos/carpetas, o desde la barra superior).
//! - Mapas de grosores de pared (mm), distancia/oclusión contra referencia, socavados (° con eje
//!   desde vista actual), curvatura, corte seccional GPU, regla 3D + goniómetro de ángulos,
//!   auditoría CAM, inversión de normales, conversión STL/OBJ/PLY y captura PNG.

#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

use apex_dentalcraft_lib::core::case_manager::CaseManager;
use apex_dentalcraft_lib::core::i18n::{self, tr, AppLanguage};
use apex_dentalcraft_lib::core::scene_file;
use apex_dentalcraft_lib::core::settings::AppSettings;
use apex_dentalcraft_lib::core::win_assoc;
use apex_dentalcraft_lib::render::gl_renderer::GlViewportRenderer;
use apex_dentalcraft_lib::theme::{
    self, AMETHYST, AMETHYST_BRIGHT, BG_CARD, BG_DEEP, BORDER_PLUM, CREAM, CREAM_MUTED, CYAN,
    EMERALD,
};
use apex_dentalcraft_lib::ui::stl_viewer_panel::StlViewerPanelState;
use eframe::glow;
use egui::{Color32, CornerRadius, Frame, Margin, RichText, Stroke, TextureHandle};
use std::path::PathBuf;
use std::sync::{Arc, Mutex};

struct Apex3DViewerApp {
    case_manager: CaseManager,
    settings: AppSettings,
    viewer: StlViewerPanelState,
    renderer: Arc<Mutex<GlViewportRenderer>>,
    app_icon_tex: Option<TextureHandle>,
    status_message: String,
    show_settings_window: bool,
}

impl Apex3DViewerApp {
    fn new(cc: &eframe::CreationContext<'_>) -> Self {
        theme::apply_apex_theme(&cc.egui_ctx);

        let gl = cc
            .gl
            .as_ref()
            .expect("Apex 3D Viewer requiere soporte OpenGL 3.3 Core (glow).");
        let renderer = Arc::new(Mutex::new(GlViewportRenderer::new(gl)));

        let app_icon_tex =
            eframe::icon_data::from_png_bytes(include_bytes!("../../assets/apex_dentalcraft.png"))
                .ok()
                .map(|icon| {
                    let color_img = egui::ColorImage::from_rgba_unmultiplied(
                        [icon.width as usize, icon.height as usize],
                        &icon.rgba,
                    );
                    cc.egui_ctx.load_texture(
                        "apex_3d_viewer_icon",
                        color_img,
                        egui::TextureOptions::LINEAR,
                    )
                });

        let case_manager = CaseManager::new(None);
        let settings = AppSettings::load_from_root(&case_manager.root_dir);
        i18n::set_language(settings.language);
        let mut viewer = StlViewerPanelState::default();
        viewer.standalone_mode = true;
        viewer.apply_settings(&settings);

        // Abrir de inmediato cualquier archivo .stl / .obj / .ply o carpeta pasada por Windows ("Abrir con...")
        let cli_paths: Vec<PathBuf> = std::env::args_os()
            .skip(1)
            .map(PathBuf::from)
            .filter(|p| p.exists())
            .collect();

        let status_message = if !cli_paths.is_empty() {
            viewer.open_files(&cli_paths)
        } else {
            tr(
                "Apex 3D Viewer listo — Arrastra archivos/carpetas .APEXSCENE/.STL/.OBJ/.PLY o pulsa '📂 Abrir' (Ctrl+O).",
                "Apex 3D Viewer ready — Drag & drop .APEXSCENE/.STL/.OBJ/.PLY files/folders or click '📂 Open STL' (Ctrl+O).",
                "Apex 3D Viewer pronto — Arraste arquivos/pastas .APEXSCENE/.STL/.OBJ/.PLY ou clique em '📂 Abrir STL' (Ctrl+O).",
            ).to_string()
        };

        Self {
            case_manager,
            settings,
            viewer,
            renderer,
            app_icon_tex,
            status_message,
            show_settings_window: false,
        }
    }

    fn show_settings_modal(&mut self, ctx: &egui::Context) {
        if !self.show_settings_window {
            return;
        }
        let mut open = self.show_settings_window;
        let mut changed = false;
        let mut register_win = false;

        egui::Window::new(tr("⚙ Configuración — Apex 3D Viewer", "⚙ Settings — Apex 3D Viewer", "⚙ Configurações — Apex 3D Viewer"))
            .id(egui::Id::new("apex_viewer_settings_modal"))
            .open(&mut open)
            .collapsible(false)
            .resizable(false)
            .default_width(430.0)
            .anchor(egui::Align2::CENTER_CENTER, egui::vec2(0.0, 0.0))
            .show(ctx, |ui| {
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

                ui.label(RichText::new(tr("1. Cámara 3D Libre 360°", "1. 360° 3D Camera", "1. Câmera 3D Livre 360°")).strong().color(CREAM));
                changed |= ui
                    .add(
                        egui::Slider::new(&mut self.settings.camera_rotate_sensitivity, 0.40..=2.20)
                            .text(tr("Sensibilidad rotación", "Orbit sensitivity", "Sensibilidade rotação")),
                    )
                    .changed();
                changed |= ui
                    .add(
                        egui::Slider::new(&mut self.settings.camera_zoom_sensitivity, 0.40..=2.20)
                            .text(tr("Sensibilidad zoom", "Zoom sensitivity", "Sensibilidade zoom")),
                    )
                    .changed();
                changed |= ui
                    .checkbox(&mut self.settings.camera_invert_zoom, tr("Invertir rueda de zoom", "Invert scroll zoom", "Inverter roda do zoom"))
                    .changed();

                ui.separator();
                ui.label(RichText::new(tr("2. Análisis de Grosores", "2. Wall Thickness Analysis", "2. Análise de Espessura")).strong().color(CREAM));
                changed |= ui
                    .add(
                        egui::Slider::new(&mut self.settings.viewer_min_thickness_warn_mm, 0.25..=1.50)
                            .text(tr("Umbral alerta pared delgada (mm)", "Thin-wall warning threshold (mm)", "Limite alerta parede fina (mm)")),
                    )
                    .changed();

                ui.separator();
                ui.label(RichText::new(tr("3. Integración con Windows ('Abrir con...')", "3. Windows Integration ('Open with...')", "3. Integração com Windows ('Abrir com...')")).strong().color(CREAM));
                ui.label(
                    RichText::new(tr(
                        "Registra Apex 3D Viewer en el menú clic derecho → 'Abrir con...' para archivos .STL, .OBJ y .PLY sin permisos de administrador.",
                        "Registers Apex 3D Viewer in Windows right-click → 'Open with...' for .STL, .OBJ & .PLY files without admin privileges.",
                        "Registra Apex 3D Viewer no menu clique direito → 'Abrir com...' para arquivos .STL, .OBJ e .PLY sem privilégios de admin.",
                    ))
                    .size(10.0)
                    .color(CREAM_MUTED),
                );
                if theme::accent_button(ui, tr("📌 Registrar Apex 3D Viewer para .STL / .OBJ / .PLY en Windows", "📌 Register Apex 3D Viewer for .STL / .OBJ / .PLY in Windows", "📌 Registrar Apex 3D Viewer para .STL / .OBJ / .PLY no Windows")).clicked() {
                    register_win = true;
                }

                ui.add_space(6.0);
                if ui.button(tr("Restaurar Valores por Defecto", "Restore Defaults", "Restaurar Padrões")).clicked() {
                    self.settings = AppSettings::default();
                    i18n::set_language(self.settings.language);
                    changed = true;
                }
            });

        self.show_settings_window = open;
        if register_win {
            if let Ok(exe) = std::env::current_exe() {
                match win_assoc::register_windows_open_with(&exe, true) {
                    Ok(m) => self.status_message = m,
                    Err(e) => self.status_message = e,
                }
            }
        }
        if changed {
            self.viewer.apply_settings(&self.settings);
            let _ = self.settings.save_to_root(&self.case_manager.root_dir);
            self.status_message = tr(
                "Configuración de Apex 3D Viewer guardada.",
                "Apex 3D Viewer settings saved.",
                "Configurações do Apex 3D Viewer salvas.",
            ).to_string();
        }
    }
}

impl eframe::App for Apex3DViewerApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        // Arrastrar y soltar archivos (.apexscene/.stl/.obj/.ply) o carpetas completas sobre la ventana
        let dropped: Vec<PathBuf> = ctx.input(|i| {
            i.raw
                .dropped_files
                .iter()
                .filter_map(|f| f.path.clone())
                .filter(|p| {
                    p.is_dir()
                        || p.extension()
                            .and_then(|s| s.to_str())
                            .map(|e| {
                                matches!(
                                    e.to_ascii_lowercase().as_str(),
                                    "stl" | "obj" | "ply" | "apexscene"
                                )
                            })
                            .unwrap_or(false)
                })
                .collect()
        });
        if !dropped.is_empty() {
            self.status_message = self.viewer.open_files(&dropped);
        }

        // Cabecera compacta dedicada (estilo 3Shape 3D Viewer)
        egui::TopBottomPanel::top("apex_viewer_top_bar")
            .exact_height(48.0)
            .frame(
                Frame::NONE
                    .fill(BG_CARD)
                    .stroke(Stroke::new(1.0_f32, Color32::from_rgb(0x57, 0x1B, 0x6E)))
                    .inner_margin(Margin::symmetric(12, 6)),
            )
            .show(ctx, |ui| {
                ui.horizontal_centered(|ui| {
                    if let Some(tex) = &self.app_icon_tex {
                        ui.add(
                            egui::Image::from_texture(tex)
                                .fit_to_exact_size(egui::vec2(30.0, 30.0)),
                        );
                    }
                    ui.vertical(|ui| {
                        ui.spacing_mut().item_spacing.y = 0.0;
                        ui.label(RichText::new("APEX 3D VIEWER").strong().size(13.5).color(CREAM));
                        ui.label(
                            RichText::new(tr("VISOR STL / OBJ / PLY • ULTRA-LIGERO", "STL / OBJ / PLY VIEWER • FAST CAM", "VISOR STL / OBJ / PLY • ULTRA-LEVE"))
                                .size(8.2)
                                .color(CYAN),
                        );
                    });

                    ui.add_space(10.0);
                    if theme::primary_button(ui, tr("📂 Abrir STL / Escena", "📂 Open STL / Scene", "📂 Abrir STL / Cena")).clicked() {
                        let mut dlg = rfd::FileDialog::new()
                            .set_title(tr("Abrir Escena o Modelo(s) 3D (reemplazar escena)", "Open Scene or 3D Model(s) (replace scene)", "Abrir Cena ou Modelo(s) 3D (substituir cena)"))
                            .add_filter(tr("Escenas y Mallas 3D", "3D Scenes & Meshes", "Cenas e Malhas 3D"), &["apexscene", "stl", "obj", "ply"])
                            .add_filter("Escena Apex DentalCraft (.apexscene)", &["apexscene"]);
                        let casos_root = scene_file::default_casos_dir();
                        if casos_root.exists() {
                            dlg = dlg.set_directory(&casos_root);
                        }
                        if let Some(files) = dlg.pick_files() {
                            self.status_message = self.viewer.replace_with_files(&files);
                        }
                    }
                    if ui.button(tr("➕ Añadir", "➕ Add Mesh", "➕ Adicionar")).on_hover_text(tr("Añadir otro archivo 3D a la escena actual", "Append another 3D mesh to scene", "Adicionar outra malha 3D à cena atual")).clicked() {
                        if let Some(files) = rfd::FileDialog::new()
                            .set_title(tr("Añadir Modelo(s) 3D a la escena", "Add 3D Model(s) to Scene", "Adicionar Modelo(s) 3D à Cena"))
                            .add_filter(tr("Mallas 3D", "3D Meshes", "Malhas 3D"), &["stl", "obj", "ply", "apexscene"])
                            .pick_files()
                        {
                            self.status_message = self.viewer.open_files(&files);
                        }
                    }
                    if ui.button(tr("📁 Carpeta", "📁 Folder", "📁 Pasta")).on_hover_text(tr("Cargar todos los modelos 3D de una carpeta", "Load all 3D models from a folder", "Carregar todos os modelos 3D de uma pasta")).clicked() {
                        if let Some(folder) = rfd::FileDialog::new()
                            .set_title(tr("Seleccionar carpeta con modelos 3D", "Select Folder with 3D Models", "Selecionar Pasta com Modelos 3D"))
                            .pick_folder()
                        {
                            self.status_message = self.viewer.open_files(&[folder]);
                        }
                    }

                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        if ui
                            .add(
                                egui::Button::new(RichText::new(tr("⚙ Config.", "⚙ Settings", "⚙ Config.")).strong().size(10.8).color(CREAM))
                                    .fill(Color32::from_rgb(0x2B, 0x07, 0x38))
                                    .stroke(Stroke::new(1.0_f32, AMETHYST_BRIGHT))
                                    .corner_radius(CornerRadius::same(6)),
                            )
                            .clicked()
                        {
                            self.show_settings_window = !self.show_settings_window;
                        }

                        if ui
                            .add(
                                egui::Button::new(RichText::new(tr("📌 Asociar .STL", "📌 Associate .STL", "📌 Associar .STL")).strong().size(10.5).color(EMERALD))
                                    .fill(Color32::from_rgb(0x12, 0x2F, 0x2A))
                                    .stroke(Stroke::new(1.0_f32, EMERALD))
                                    .corner_radius(CornerRadius::same(6)),
                            )
                            .on_hover_text(tr("Registrar Apex 3D Viewer en el menú 'Abrir con...' de Windows para .STL/.OBJ/.PLY", "Register Apex 3D Viewer in Windows 'Open with...' menu", "Registrar Apex 3D Viewer no menu 'Abrir com...' do Windows"))
                            .clicked()
                        {
                            if let Ok(exe) = std::env::current_exe() {
                                match win_assoc::register_windows_open_with(&exe, true) {
                                    Ok(m) => self.status_message = m,
                                    Err(e) => self.status_message = e,
                                }
                            }
                        }

                        if self.viewer.objects.len() > 0 {
                            let tot_tris: usize = self.viewer.objects.iter().map(|o| o.audit.n_faces).sum();
                            ui.label(
                                RichText::new(format!("{} obj | {} tris", self.viewer.objects.len(), tot_tris))
                                    .size(10.0)
                                    .color(CREAM_MUTED),
                            );
                        }
                    });
                });
            });

        // Barra inferior de estado
        egui::TopBottomPanel::bottom("apex_viewer_status")
            .exact_height(24.0)
            .frame(
                Frame::NONE
                    .fill(BG_DEEP)
                    .stroke(Stroke::new(1.0_f32, BORDER_PLUM))
                    .inner_margin(Margin::symmetric(12, 3)),
            )
            .show(ctx, |ui| {
                ui.horizontal_centered(|ui| {
                    ui.label(RichText::new(&self.status_message).size(10.2).color(CREAM));
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        ui.label(
                            RichText::new(tr(
                                "Der: Rotar 360° | Alt+Der: Rodar | Rueda: Pan / Zoom | Izq: Medir / Acción | [H] Pantalla limpia",
                                "Right-Drag: Orbit 360° | Alt+Right: Roll | Wheel: Pan/Zoom | Left: Measure/Act | [H] Clean view",
                                "Dir: Girar 360° | Alt+Dir: Rolar | Roda: Pan/Zoom | Esq: Medir/Ação | [H] Tela limpa",
                            ))
                            .size(9.5)
                            .color(AMETHYST),
                        );
                    });
                });
            });

        self.show_settings_modal(ctx);

        if let Some(msg) = self.viewer.ui(ctx, None, &self.case_manager, &self.renderer) {
            self.status_message = msg;
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

fn main() -> eframe::Result {
    let mut viewport = egui::ViewportBuilder::default()
        .with_title("Apex 3D Viewer — Visor Dental STL / OBJ / PLY (Apex DentalCraft)")
        .with_inner_size([1360.0, 860.0])
        .with_min_inner_size([900.0, 600.0]);

    if let Ok(icon) =
        eframe::icon_data::from_png_bytes(include_bytes!("../../assets/apex_dentalcraft.png"))
    {
        viewport = viewport.with_icon(Arc::new(icon));
    }

    let options = eframe::NativeOptions {
        renderer: eframe::Renderer::Glow,
        depth_buffer: 24,
        multisampling: 4,
        viewport,
        ..Default::default()
    };

    eframe::run_native(
        "Apex 3D Viewer",
        options,
        Box::new(|cc| Ok(Box::new(Apex3DViewerApp::new(cc)))),
    )
}
