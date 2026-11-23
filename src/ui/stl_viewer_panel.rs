//! **Apex 3D Viewer** — Visor 3D Nativo de Ultra-Alto Rendimiento (estilo 3Shape 3D Viewer).
//!
//! Diseñado mediante REA del flujo profesional de 3Shape 3D Viewer sobre el motor nativo
//! OpenGL 3.3 Core (`glow`) + `parry3d` BVH de **Apex DentalCraft**:
//! - Funciona tanto **independiente (`Apex3DViewer.exe`, "Abrir con..." en Windows)** como integrado en Apex DentalCraft.
//! - Carga instantánea multi-archivo y de carpetas completas (`.stl` binario/ASCII, `.obj`, `.ply`) y Arrastrar/Soltar.
//! - Auto-detección inteligente de roles dentales (Corona, Muñón, Antagonista, Mordida, Cementado, Encía).
//! - Transparencias independientes, presets de escenas rápidas (Clínica, Solo Corona, Ajuste, Oclusión, Rayos X, Intaglio).
//! - Mapas de color clínicos en tiempo real con **leyenda graduada en pantalla**:
//!   Grosor de Pared (mm), Distancia 3D / Oclusión / Cemento contra cualquier objeto de referencia,
//!   Mapa de Socavados (° con "Fijar eje desde vista actual") y Mapa de Curvatura / Filos.
//! - Corte transversal GPU multi-eje (`X`, `Y`, `Z`, `Cámara`) con inversión de lado.
//! - Mediciones 3D persistentes múltiples: **Regla de Distancia (2 clics)** y **Goniómetro de Ángulo (3 clics)** con etiquetas flotantes en pantalla.
//! - Auditoría CAM completa, inversión de normales en 1 clic, conversión/exportación STL/OBJ/PLY y Captura PNG.

use crate::core::case_manager::{CaseManager, DentalCase};
use crate::core::i18n::{self, tr, AppLanguage};
use crate::core::scene_file;
use crate::core::settings::AppSettings;
use crate::core::win_assoc;
use crate::geometry::crown::CrownDesigner;
use crate::geometry::margin::{build_tube_mesh, build_uv_sphere};
use crate::geometry::mesh::{CamAudit, MeshCollider, TriMeshData};
use crate::geometry::mesh_ops::{compute_mean_curvature_weights, compute_undercut_angles_deg};
use crate::render::camera::OrbitCamera;
use crate::render::gl_renderer::{
    make_viewport_paint_callback, ColorMapKind, GlViewportRenderer, RenderLayerSpec,
};
use crate::theme::{
    self, AMBER, AMETHYST, AMETHYST_BRIGHT, BG_DEEP, BG_PANEL, BORDER_PLUM, CREAM, CREAM_MUTED,
    CYAN, EMERALD,
};
use egui::{
    Align2, Area, Color32, CornerRadius, Frame, Key, Margin, Order, Pos2, Rect, RichText, Sense,
    Stroke, Window,
};
use glam::{Vec3, Vec4};
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SceneRole {
    Restoration,
    ModelDie,
    Antagonist,
    CementSpace,
    MinThickness,
    GenericStl,
}

impl SceneRole {
    pub fn label(self) -> &'static str {
        match self {
            Self::Restoration => tr("Restauración", "Restoration", "Restauração"),
            Self::ModelDie => tr("Modelo / Muñón", "Model / Die", "Modelo / Troquel"),
            Self::Antagonist => tr("Antagonista", "Antagonist", "Antagonista"),
            Self::CementSpace => tr("Cementado", "Cement Space", "Cimentação"),
            Self::MinThickness => tr("Grosor Mín.", "Min. Thickness", "Espessura Mín."),
            Self::GenericStl => tr("Malla 3D", "3D Mesh", "Malha 3D"),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StlColorMode {
    Solid,
    WallThickness,
    OcclusalDistance,
    DieClearance,
    Undercut,
    Curvature,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ClipAxisMode {
    X,
    Y,
    Z,
    Camera,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MeasureToolMode {
    None,
    Distance2Pt,
    Angle3Pt,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ShadingStyle {
    CeramicGloss,
    PlasterMatte,
    HighContrast,
}

impl ShadingStyle {
    pub fn label(self) -> &'static str {
        match self {
            Self::CeramicGloss => tr("Cerámico (Brillo)", "Ceramic (Gloss)", "Cerâmica (Brilho)"),
            Self::PlasterMatte => tr("Yeso Mate", "Matte Plaster", "Gesso Fosco"),
            Self::HighContrast => tr("Alto Contraste", "High Contrast", "Alto Contraste"),
        }
    }
}

#[derive(Debug, Clone)]
pub struct DistanceMeasurement {
    pub p1: Vec3,
    pub p2: Vec3,
}

#[derive(Debug, Clone)]
pub struct AngleMeasurement {
    pub a: Vec3,
    pub vertex: Vec3,
    pub b: Vec3,
}

impl AngleMeasurement {
    pub fn angle_deg(&self) -> f32 {
        let u = (self.a - self.vertex).normalize_or_zero();
        let v = (self.b - self.vertex).normalize_or_zero();
        u.dot(v).clamp(-1.0, 1.0).acos().to_degrees()
    }
}

pub struct SceneObject {
    pub id: u64,
    pub name: String,
    #[allow(dead_code)]
    pub path: Option<PathBuf>,
    pub role: SceneRole,
    pub mesh: TriMeshData,
    pub collider: Option<MeshCollider>,
    pub rev: u64,
    pub audit: CamAudit,
    pub area_mm2: f32,
    pub max_edge_mm: f32,

    pub visible: bool,
    pub opacity: f32,
    pub tint: [f32; 3],
    pub wireframe: bool,
    pub color_mode: StlColorMode,
    pub thickness_stats: Option<(f32, f32, f32)>,
    pub thin_vert_pct: f32,
    pub distance_stats: Option<(f32, f32, f32)>,
}

pub struct StlViewerPanelState {
    pub standalone_mode: bool,
    pub hide_panels: bool,
    pub shading_style: ShadingStyle,

    pub camera: OrbitCamera,
    pub next_id: u64,
    pub next_rev: u64,
    pub objects: Vec<SceneObject>,
    pub selected_idx: Option<usize>,
    /// Objeto de referencia opcional para mapas de distancia/oclusión/holgura (ID del objeto).
    pub reference_obj_id: Option<u64>,
    /// Eje personalizado para el mapa de socavados (se puede fijar desde la vista actual como en 3Shape).
    pub undercut_axis: Vec3,

    pub margin_curve: Vec<Vec3>,
    pub margin_rev: u64,
    pub show_margin: bool,
    pub margin_line_radius_mm: f32,

    // Corte transversal interactivo GPU (estilo 3Shape 3D Viewer)
    pub clip_enabled: bool,
    pub clip_axis: ClipAxisMode,
    pub clip_pos_norm: f32,
    pub clip_invert: bool,

    // Herramientas de medición 3D persistentes (Regla 2 pts + Goniómetro 3 pts)
    pub measure_tool: MeasureToolMode,
    pub pending_pts: Vec<Vec3>,
    pub distances: Vec<DistanceMeasurement>,
    pub angles: Vec<AngleMeasurement>,
    pub measure_rev: u64,
    pub press_start_pos: Option<Pos2>,

    // Umbral clínico de alerta de espesor
    pub min_thickness_warn_mm: f32,

    // Captura PNG pendiente (para ViewportCommand::Screenshot)
    pub pending_screenshot_path: Option<PathBuf>,
}

impl Default for StlViewerPanelState {
    fn default() -> Self {
        Self {
            standalone_mode: false,
            hide_panels: false,
            shading_style: ShadingStyle::CeramicGloss,
            camera: OrbitCamera::default(),
            next_id: 1,
            next_rev: 1,
            objects: Vec::new(),
            selected_idx: None,
            reference_obj_id: None,
            undercut_axis: Vec3::Z,
            margin_curve: Vec::new(),
            margin_rev: 1,
            show_margin: true,
            margin_line_radius_mm: 0.028,
            clip_enabled: false,
            clip_axis: ClipAxisMode::X,
            clip_pos_norm: 0.0,
            clip_invert: false,
            measure_tool: MeasureToolMode::None,
            pending_pts: Vec::new(),
            distances: Vec::new(),
            angles: Vec::new(),
            measure_rev: 1,
            press_start_pos: None,
            min_thickness_warn_mm: 0.60,
            pending_screenshot_path: None,
        }
    }
}

const MATERIAL_SWATCHES: &[(&str, [f32; 3])] = &[
    ("Cerámica A2", [0.96, 0.94, 0.87]),
    ("Zirconia A1", [0.98, 0.98, 0.99]),
    ("Yeso / Muñón", [0.91, 0.85, 0.75]),
    ("Antagonista Lila", [0.85, 0.80, 0.90]),
    ("Cemento Cian", [0.06, 0.78, 0.88]),
    ("Grosor Ámbar", [0.95, 0.60, 0.08]),
    ("Encía Rosa", [0.88, 0.58, 0.64]),
    ("Metal CoCr", [0.70, 0.74, 0.78]),
    ("Cera Azul", [0.22, 0.55, 0.86]),
    ("Resina Modelo", [0.78, 0.72, 0.66]),
];

impl StlViewerPanelState {
    fn alloc_id(&mut self) -> u64 {
        self.next_id += 1;
        self.next_id
    }

    fn bump(&mut self) -> u64 {
        self.next_rev += 1;
        self.next_rev
    }

    pub fn apply_settings(&mut self, settings: &AppSettings) {
        let r = (settings.margin_line_radius_mm * 0.90).clamp(0.015, 0.120);
        if (self.margin_line_radius_mm - r).abs() > 1e-4 {
            self.margin_line_radius_mm = r;
            self.margin_rev = self.bump();
        }
        self.min_thickness_warn_mm = settings.viewer_min_thickness_warn_mm;
        self.camera.rotate_sensitivity = settings.camera_rotate_sensitivity;
        self.camera.zoom_sensitivity = settings.camera_zoom_sensitivity;
        self.camera.invert_zoom = settings.camera_invert_zoom;
    }

    fn infer_role_and_style(name: &str) -> (SceneRole, [f32; 3], f32) {
        let low = name.to_lowercase();
        if low.contains("corona")
            || low.contains("crown")
            || low.contains("restor")
            || low.contains("carilla")
            || low.contains("cofia")
            || low.contains("puente")
            || low.contains("inlay")
            || low.contains("onlay")
        {
            (SceneRole::Restoration, [0.96, 0.94, 0.87], 1.00)
        } else if low.contains("antag") || low.contains("upper") || low.contains("oppos") {
            (SceneRole::Antagonist, [0.85, 0.80, 0.90], 0.55)
        } else if low.contains("cement") {
            (SceneRole::CementSpace, [0.06, 0.78, 0.88], 0.55)
        } else if low.contains("grosor") || low.contains("thick") {
            (SceneRole::MinThickness, [0.95, 0.60, 0.08], 0.45)
        } else if low.contains("encia") || low.contains("gingiv") || low.contains("gum") {
            (SceneRole::ModelDie, [0.88, 0.58, 0.64], 0.85)
        } else if low.contains("modelo")
            || low.contains("mand")
            || low.contains("maxil")
            || low.contains("die")
            || low.contains("work")
            || low.contains("lower")
            || low.contains("bite")
        {
            (SceneRole::ModelDie, [0.91, 0.85, 0.75], 0.85)
        } else {
            (SceneRole::GenericStl, [0.96, 0.94, 0.87], 1.00)
        }
    }

    pub fn add_mesh_object(
        &mut self,
        name: String,
        path: Option<PathBuf>,
        role: SceneRole,
        mesh: TriMeshData,
        tint: [f32; 3],
        opacity: f32,
        visible: bool,
    ) -> usize {
        let audit = mesh.inspect_cam();
        let area_mm2 = mesh.surface_area_mm2();
        let max_edge_mm = mesh.max_edge_length_mm();
        let collider = MeshCollider::from_mesh(&mesh);
        let id = self.alloc_id();
        let rev = self.bump();
        self.objects.push(SceneObject {
            id,
            name,
            path,
            role,
            mesh,
            collider,
            rev,
            audit,
            area_mm2,
            max_edge_mm,
            visible,
            opacity,
            tint,
            wireframe: false,
            color_mode: StlColorMode::Solid,
            thickness_stats: None,
            thin_vert_pct: 0.0,
            distance_stats: None,
        });
        let idx = self.objects.len() - 1;
        self.selected_idx = Some(idx);
        idx
    }

    /// Expande carpetas y filtra extensiones 3D válidas (`.stl`, `.obj`, `.ply`).
    fn collect_3d_paths(inputs: &[PathBuf]) -> Vec<PathBuf> {
        let mut out = Vec::new();
        for p in inputs {
            if p.is_dir() {
                if let Ok(entries) = std::fs::read_dir(p) {
                    let mut children: Vec<PathBuf> = entries
                        .flatten()
                        .map(|e| e.path())
                        .filter(|cp| {
                            cp.is_file()
                                && matches!(
                                    cp.extension()
                                        .and_then(|e| e.to_str())
                                        .map(|s| s.to_ascii_lowercase())
                                        .as_deref(),
                                    Some("stl") | Some("obj") | Some("ply")
                                )
                        })
                        .collect();
                    children.sort();
                    out.extend(children);
                }
            } else if p.is_file() {
                out.push(p.clone());
            }
        }
        out
    }

    fn find_first_apex_scene(inputs: &[PathBuf]) -> Option<PathBuf> {
        for p in inputs {
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

    /// Abre un archivo de escena nativo `.apexscene`, cargando restauraciones, muñón,
    /// antagonista, mordida y línea de margen con sus roles clínicos.
    pub fn open_apex_scene(&mut self, scene_path: &Path) -> String {
        let scene = match scene_file::load_apex_scene(scene_path) {
            Ok(s) => s,
            Err(e) => return e,
        };
        let Some(base_dir) = scene_path.parent() else {
            return "Ruta de escena inválida.".to_string();
        };

        self.clear_scene();
        let mut loaded_count = 0usize;
        let mut first_rest_idx = None;

        for rest in &scene.restorations {
            let p = base_dir.join(&rest.file_name);
            if p.is_file() {
                if let Ok(m) = TriMeshData::load_from_path(&p) {
                    let idx = self.add_mesh_object(
                        rest.label.clone(),
                        Some(p),
                        SceneRole::Restoration,
                        m,
                        [0.96, 0.94, 0.87],
                        1.00,
                        true,
                    );
                    if first_rest_idx.is_none() {
                        first_rest_idx = Some(idx);
                    }
                    loaded_count += 1;
                }
            }
        }

        if let Some(rel) = &scene.models.die_model {
            let p = base_dir.join(rel);
            if p.is_file() {
                if let Ok(m) = TriMeshData::load_from_path(&p) {
                    self.add_mesh_object(
                        format!(
                            "{} ({})",
                            tr("Muñón / Modelo Trabajo", "Die / Working Model", "Troquel / Modelo Trabalho"),
                            i18n::tr_arch_name(&scene.models.die_arch)
                        ),
                        Some(p),
                        SceneRole::ModelDie,
                        m,
                        [0.91, 0.85, 0.75],
                        0.85,
                        true,
                    );
                    loaded_count += 1;
                }
            }
        }

        if let Some(rel) = &scene.models.antagonist_model {
            let p = base_dir.join(rel);
            if p.is_file() {
                if let Ok(m) = TriMeshData::load_from_path(&p) {
                    self.add_mesh_object(
                        tr("Antagonista", "Antagonist", "Antagonista").to_string(),
                        Some(p),
                        SceneRole::Antagonist,
                        m,
                        [0.85, 0.80, 0.90],
                        0.45,
                        true,
                    );
                    loaded_count += 1;
                }
            }
        }

        if let Some(rel) = &scene.models.bite_model {
            let p = base_dir.join(rel);
            if p.is_file() {
                if let Ok(m) = TriMeshData::load_from_path(&p) {
                    self.add_mesh_object(
                        tr("Registro Mordida", "Bite Registration", "Registro de Mordida").to_string(),
                        Some(p),
                        SceneRole::GenericStl,
                        m,
                        [0.78, 0.84, 0.90],
                        0.35,
                        false,
                    );
                    loaded_count += 1;
                }
            }
        }

        if scene.case_data.margin_points.len() >= 4 {
            self.margin_curve = scene
                .case_data
                .margin_points
                .iter()
                .map(|pt| Vec3::from_array(*pt))
                .collect();
            self.margin_rev = self.bump();
        }

        if let Some(ridx) = first_rest_idx {
            self.selected_idx = Some(ridx);
        }
        self.frame_scene();
        let q = glam::Quat::from_array(scene.view_state.camera_orientation);
        if scene.view_state.camera_distance > 1.0 && q.length_squared() > 0.5 && !q.is_nan() {
            self.camera.target = Vec3::from_array(scene.view_state.camera_target);
            self.camera.orientation = q.normalize();
            self.camera.distance = scene.view_state.camera_distance.clamp(2.0, 320.0);
        }

        match i18n::current_language() {
            AppLanguage::Es => format!(
                "Escena '{}' abierta en Apex 3D Viewer ({} objetos 3D).",
                scene.case_data.patient_name, loaded_count
            ),
            AppLanguage::En => format!(
                "Scene '{}' opened in Apex 3D Viewer ({} 3D objects).",
                scene.case_data.patient_name, loaded_count
            ),
            AppLanguage::Pt => format!(
                "Cena '{}' aberta no Apex 3D Viewer ({} objetos 3D).",
                scene.case_data.patient_name, loaded_count
            ),
        }
    }

    pub fn open_files(&mut self, paths: &[PathBuf]) -> String {
        if let Some(scene_p) = Self::find_first_apex_scene(paths) {
            return self.open_apex_scene(&scene_p);
        }
        let expanded = Self::collect_3d_paths(paths);
        let mut loaded = 0usize;
        let mut errors = Vec::new();
        let t0 = std::time::Instant::now();
        for p in &expanded {
            match TriMeshData::load_from_path(p) {
                Ok(mesh) => {
                    let fname = p
                        .file_name()
                        .and_then(|s| s.to_str())
                        .unwrap_or("Malla.stl")
                        .to_string();
                    let (role, tint, op) = Self::infer_role_and_style(&fname);
                    self.add_mesh_object(fname, Some(p.clone()), role, mesh, tint, op, true);
                    loaded += 1;
                }
                Err(e) => errors.push(e),
            }
        }
        if loaded > 0 {
            self.frame_scene();
            let ms = t0.elapsed().as_secs_f64() * 1000.0;
            match i18n::current_language() {
                AppLanguage::Es => format!("Apex 3D Viewer: {} malla(s) cargada(s) en {:.0} ms.", loaded, ms),
                AppLanguage::En => format!("Apex 3D Viewer: {} mesh(es) loaded in {:.0} ms.", loaded, ms),
                AppLanguage::Pt => format!("Apex 3D Viewer: {} malha(s) carregada(s) em {:.0} ms.", loaded, ms),
            }
        } else if !errors.is_empty() {
            format!(
                "{} {}",
                tr(
                    "Error al cargar malla 3D:",
                    "Error loading 3D mesh:",
                    "Erro ao carregar malha 3D:",
                ),
                errors[0]
            )
        } else {
            tr(
                "No se encontraron archivos .STL, .OBJ, .PLY o .APEXSCENE.",
                "No .STL, .OBJ, .PLY or .APEXSCENE files found.",
                "Nenhum arquivo .STL, .OBJ, .PLY ou .APEXSCENE encontrado.",
            )
            .to_string()
        }
    }

    pub fn replace_with_files(&mut self, paths: &[PathBuf]) -> String {
        if let Some(scene_p) = Self::find_first_apex_scene(paths) {
            return self.open_apex_scene(&scene_p);
        }
        let expanded = Self::collect_3d_paths(paths);
        if expanded.is_empty() {
            return tr(
                "No se encontraron archivos .STL, .OBJ, .PLY o .APEXSCENE.",
                "No .STL, .OBJ, .PLY or .APEXSCENE files found.",
                "Nenhum arquivo .STL, .OBJ, .PLY ou .APEXSCENE encontrado.",
            )
            .to_string();
        }
        self.clear_scene();
        self.open_files(&expanded)
    }

    pub fn clear_scene(&mut self) {
        self.objects.clear();
        self.selected_idx = None;
        self.reference_obj_id = None;
        self.margin_curve.clear();
        self.pending_pts.clear();
        self.distances.clear();
        self.angles.clear();
        self.measure_rev = self.bump();
    }

    /// Carga toda la escena clínica del caso activo (Restauraciones, Muñón, Antagonista, Cemento, Grosor y Margen).
    pub fn load_active_case_scene(
        &mut self,
        case: &DentalCase,
        case_manager: &CaseManager,
    ) -> String {
        self.clear_scene();

        let case_dir = case_manager.get_case_dir(&case.case_id);
        let mut loaded_count = 0usize;

        let mut first_rest_idx = None;
        for (&fdi, unit) in &case.restorations {
            if let Some(rel) = &unit.crown_design_path {
                let p = case_dir.join(rel);
                if p.exists() {
                    if let Ok(m) = TriMeshData::load_from_path(&p) {
                        let idx = self.add_mesh_object(
                            format!(
                                "{} #{} ({})",
                                tr("Corona", "Crown", "Coroa"),
                                fdi,
                                i18n::tr_process_type(&unit.process_type)
                            ),
                            Some(p),
                            SceneRole::Restoration,
                            m,
                            [0.96, 0.94, 0.87],
                            1.00,
                            true,
                        );
                        if first_rest_idx.is_none() {
                            first_rest_idx = Some(idx);
                        }
                        loaded_count += 1;
                    }
                }
            }
        }

        let mut prep_for_aux: Option<TriMeshData> = None;
        if let Some(p) = case_manager.get_active_scan_path(case, &case.prep_arch) {
            if let Ok(m) = TriMeshData::load_from_path(&p) {
                prep_for_aux = Some(m.clone());
                self.add_mesh_object(
                    format!(
                        "{} ({})",
                        tr("Modelo Trabajo", "Working Model", "Modelo de Trabalho"),
                        i18n::tr_arch_name(&case.prep_arch)
                    ),
                    Some(p),
                    SceneRole::ModelDie,
                    m,
                    [0.91, 0.85, 0.75],
                    0.80,
                    true,
                );
                loaded_count += 1;
            }
        }

        if let Some(p) = case_manager.get_active_scan_path(case, case.get_antagonist_arch()) {
            if let Ok(m) = TriMeshData::load_from_path(&p) {
                self.add_mesh_object(
                    tr("Antagonista", "Antagonist", "Antagonista").to_string(),
                    Some(p),
                    SceneRole::Antagonist,
                    m,
                    [0.85, 0.80, 0.90],
                    0.45,
                    false,
                );
                loaded_count += 1;
            }
        }

        if case.margin_points.len() >= 8 {
            let margin: Vec<Vec3> = case
                .margin_points
                .iter()
                .map(|p| Vec3::from_array(*p))
                .collect();
            self.margin_curve = margin.clone();
            self.margin_rev = self.bump();
            if let Some(prep) = &prep_for_aux {
                let ax = Vec3::from_array(case.insertion_axis);
                if let Some((cem, min_t, _)) =
                    CrownDesigner::generate_auxiliary_shells(prep, &margin, ax, &case.design_params)
                {
                    self.add_mesh_object(
                        tr("Espacio de Cementado", "Cement Space", "Espaço de Cimentação").to_string(),
                        None,
                        SceneRole::CementSpace,
                        cem,
                        [0.06, 0.78, 0.88],
                        0.50,
                        false,
                    );
                    self.add_mesh_object(
                        tr("Capa Grosor Mínimo", "Minimum Thickness Shell", "Camada de Espessura Mínima").to_string(),
                        None,
                        SceneRole::MinThickness,
                        min_t,
                        [0.95, 0.60, 0.08],
                        0.40,
                        false,
                    );
                    loaded_count += 2;
                }
            }
        }

        if let Some(ridx) = first_rest_idx {
            self.selected_idx = Some(ridx);
        }
        self.frame_scene();
        match i18n::current_language() {
            AppLanguage::Es => format!(
                "Escena del caso '{}' cargada en Visor 3D ({} objetos).",
                case.patient_name, loaded_count
            ),
            AppLanguage::En => format!(
                "Case scene '{}' loaded in 3D Viewer ({} objects).",
                case.patient_name, loaded_count
            ),
            AppLanguage::Pt => format!(
                "Cena do caso '{}' carregada no Visualizador 3D ({} objetos).",
                case.patient_name, loaded_count
            ),
        }
    }

    pub fn scene_bounds(&self) -> Option<(Vec3, Vec3)> {
        let mut mn = Vec3::splat(f32::INFINITY);
        let mut mx = Vec3::splat(f32::NEG_INFINITY);
        let mut any = false;
        for obj in &self.objects {
            if !obj.visible || obj.mesh.is_empty() {
                continue;
            }
            let (a, b) = obj.mesh.bounds();
            mn = mn.min(a);
            mx = mx.max(b);
            any = true;
        }
        if !any {
            for obj in &self.objects {
                if !obj.mesh.is_empty() {
                    let (a, b) = obj.mesh.bounds();
                    mn = mn.min(a);
                    mx = mx.max(b);
                    any = true;
                }
            }
        }
        if any { Some((mn, mx)) } else { None }
    }

    pub fn frame_scene(&mut self) {
        if let Some((mn, mx)) = self.scene_bounds() {
            self.camera.frame_bounds(mn, mx);
        }
    }

    pub fn frame_selected(&mut self) {
        if let Some(sel) = self.selected_idx.filter(|&i| i < self.objects.len()) {
            let (mn, mx) = self.objects[sel].mesh.bounds();
            self.camera.frame_bounds(mn, mx);
        } else {
            self.frame_scene();
        }
    }

    pub fn apply_scene_preset(&mut self, preset: &str) {
        match preset {
            "clinical" => {
                for obj in &mut self.objects {
                    match obj.role {
                        SceneRole::Restoration | SceneRole::GenericStl => {
                            obj.visible = true;
                            obj.opacity = 1.00;
                        }
                        SceneRole::ModelDie => {
                            obj.visible = true;
                            obj.opacity = 0.80;
                        }
                        SceneRole::Antagonist => {
                            obj.visible = false;
                            obj.opacity = 0.45;
                        }
                        SceneRole::CementSpace | SceneRole::MinThickness => {
                            obj.visible = false;
                        }
                    }
                }
            }
            "restoration_only" | "intaglio" => {
                let has_rest = self
                    .objects
                    .iter()
                    .any(|o| matches!(o.role, SceneRole::Restoration));
                for (idx, obj) in self.objects.iter_mut().enumerate() {
                    let keep = if has_rest {
                        obj.role == SceneRole::Restoration
                    } else {
                        Some(idx) == self.selected_idx || idx == 0
                    };
                    obj.visible = keep;
                    if keep {
                        obj.opacity = 1.00;
                    }
                }
                if preset == "intaglio" {
                    self.camera.set_view_occlusal_bottom();
                }
            }
            "fit_cement" => {
                for obj in &mut self.objects {
                    match obj.role {
                        SceneRole::Restoration | SceneRole::GenericStl => {
                            obj.visible = true;
                            obj.opacity = 0.58;
                        }
                        SceneRole::ModelDie => {
                            obj.visible = true;
                            obj.opacity = 1.00;
                        }
                        SceneRole::CementSpace => {
                            obj.visible = true;
                            obj.opacity = 0.55;
                        }
                        SceneRole::MinThickness => {
                            obj.visible = true;
                            obj.opacity = 0.40;
                        }
                        SceneRole::Antagonist => {
                            obj.visible = false;
                        }
                    }
                }
            }
            "occlusion" => {
                for obj in &mut self.objects {
                    match obj.role {
                        SceneRole::Restoration | SceneRole::GenericStl => {
                            obj.visible = true;
                            obj.opacity = 1.00;
                        }
                        SceneRole::Antagonist => {
                            obj.visible = true;
                            obj.opacity = 0.45;
                        }
                        SceneRole::ModelDie => {
                            obj.visible = true;
                            obj.opacity = 0.75;
                        }
                        SceneRole::CementSpace | SceneRole::MinThickness => {
                            obj.visible = false;
                        }
                    }
                }
            }
            "xray" => {
                for obj in &mut self.objects {
                    obj.visible = true;
                    obj.opacity = match obj.role {
                        SceneRole::Restoration | SceneRole::GenericStl => 0.42,
                        SceneRole::ModelDie => 0.78,
                        _ => 0.32,
                    };
                }
            }
            _ => {
                for obj in &mut self.objects {
                    obj.visible = true;
                    obj.opacity = 1.00;
                }
            }
        }
        self.frame_scene();
    }

    fn resolve_reference_mesh(&self, active_idx: usize, preferred_role: SceneRole) -> Option<TriMeshData> {
        if let Some(ref_id) = self.reference_obj_id {
            if let Some(o) = self
                .objects
                .iter()
                .enumerate()
                .find(|(i, o)| *i != active_idx && o.id == ref_id)
                .map(|(_, o)| o)
            {
                return Some(o.mesh.clone());
            }
        }
        self.objects
            .iter()
            .enumerate()
            .find(|(i, o)| *i != active_idx && o.role == preferred_role)
            .or_else(|| self.objects.iter().enumerate().find(|(i, _)| *i != active_idx))
            .map(|(_, o)| o.mesh.clone())
    }

    /// Calcula (o actualiza) el análisis de color/grosor/distancia/socavado/curvatura del objeto seleccionado.
    pub fn ensure_color_analysis(&mut self, idx: usize, mode: StlColorMode) {
        if idx >= self.objects.len() {
            return;
        }
        let warn_mm = self.min_thickness_warn_mm;
        match mode {
            StlColorMode::Solid => {}
            StlColorMode::WallThickness => {
                let stats = self.objects[idx].mesh.compute_self_wall_thickness();
                let n = self.objects[idx].mesh.thickness_mm.len().max(1);
                let thin_cnt = self.objects[idx]
                    .mesh
                    .thickness_mm
                    .iter()
                    .filter(|&&t| t > 0.04 && t < warn_mm)
                    .count();
                self.objects[idx].thickness_stats = Some(stats);
                self.objects[idx].thin_vert_pct = (thin_cnt as f32 / n as f32) * 100.0;
            }
            StlColorMode::OcclusalDistance => {
                if let Some(other) = self.resolve_reference_mesh(idx, SceneRole::Antagonist) {
                    let stats = self.objects[idx].mesh.compute_distance_to_mesh(&other, true);
                    self.objects[idx].distance_stats = Some(stats);
                }
            }
            StlColorMode::DieClearance => {
                if let Some(die) = self.resolve_reference_mesh(idx, SceneRole::ModelDie) {
                    let stats = self.objects[idx].mesh.compute_distance_to_mesh(&die, false);
                    let n = self.objects[idx].mesh.thickness_mm.len().max(1);
                    let thin_cnt = self.objects[idx]
                        .mesh
                        .thickness_mm
                        .iter()
                        .filter(|&&t| t > 0.04 && t < warn_mm)
                        .count();
                    self.objects[idx].thickness_stats = Some(stats);
                    self.objects[idx].thin_vert_pct = (thin_cnt as f32 / n as f32) * 100.0;
                }
            }
            StlColorMode::Undercut => {
                let angs = compute_undercut_angles_deg(&self.objects[idx].mesh, self.undercut_axis);
                self.objects[idx].mesh.custom_scalars = angs;
            }
            StlColorMode::Curvature => {
                let curv = compute_mean_curvature_weights(&self.objects[idx].mesh);
                self.objects[idx].mesh.custom_scalars = curv;
            }
        }
        self.objects[idx].color_mode = mode;
        self.objects[idx].rev = self.bump();
    }

    pub fn flip_selected_normals(&mut self) -> Option<String> {
        let sel = self.selected_idx.filter(|&i| i < self.objects.len())?;
        self.objects[sel].mesh.flip_faces();
        self.objects[sel].audit = self.objects[sel].mesh.inspect_cam();
        self.objects[sel].collider = MeshCollider::from_mesh(&self.objects[sel].mesh);
        let mode = self.objects[sel].color_mode;
        self.ensure_color_analysis(sel, mode);
        Some(format!(
            "{} '{}'.",
            tr("Normales invertidas en", "Flipped normals on", "Normais invertidas em"),
            self.objects[sel].name
        ))
    }

    pub fn export_selected_to(&self, path: &Path) -> Result<String, String> {
        let sel = self
            .selected_idx
            .filter(|&i| i < self.objects.len())
            .ok_or_else(|| {
                tr(
                    "Selecciona un objeto 3D para exportar.",
                    "Select a 3D object to export.",
                    "Selecione um objeto 3D para exportar.",
                )
                .to_string()
            })?;
        let obj = &self.objects[sel];
        let ext = path
            .extension()
            .and_then(|s| s.to_str())
            .unwrap_or("stl")
            .to_ascii_lowercase();
        match ext.as_str() {
            "obj" => obj.mesh.save_obj(path)?,
            "ply" => obj.mesh.save_ply(path)?,
            _ => obj.mesh.save_stl_binary(path)?,
        }
        Ok(format!(
            "{} {}",
            tr("✔ Exportado correctamente:", "✔ Exported successfully:", "✔ Exportado com sucesso:"),
            path.display()
        ))
    }

    fn pick_visible_surface(&self, pos: Pos2, rect: Rect) -> Option<Vec3> {
        let (ro, rd) = self.camera.screen_ray(pos, rect);
        let mut best_t = f32::INFINITY;
        let mut best_hit = None;
        for obj in &self.objects {
            if !obj.visible {
                continue;
            }
            if let Some(col) = &obj.collider {
                if let Some((t, hit)) = col.cast_ray(ro, rd, best_t) {
                    if t < best_t {
                        best_t = t;
                        best_hit = Some(hit);
                    }
                }
            }
        }
        best_hit
    }

    fn paint_colorbar_legend(&self, painter: &egui::Painter, rect: Rect, mode: StlColorMode) {
        if mode == StlColorMode::Solid {
            return;
        }
        let bar_w = 14.0_f32;
        let bar_h = 165.0_f32;
        let bx = rect.right() - if self.hide_panels { 66.0 } else { 366.0 };
        let by = rect.center().y - bar_h * 0.5;
        if bx < rect.left() + 80.0 {
            return;
        }
        let bg_rect = Rect::from_min_size(Pos2::new(bx - 10.0, by - 28.0), egui::vec2(72.0, bar_h + 44.0));
        painter.rect_filled(bg_rect, CornerRadius::same(7), Color32::from_rgba_unmultiplied(0x16, 0x04, 0x1E, 215));
        painter.rect_stroke(bg_rect, CornerRadius::same(7), Stroke::new(1.0_f32, BORDER_PLUM), egui::StrokeKind::Inside);

        let (title, ticks, stops): (&str, [&str; 4], [Color32; 4]) = match mode {
            StlColorMode::WallThickness | StlColorMode::DieClearance => (
                tr("Grosor", "Thickness", "Espessura"),
                [">2.0 mm", "1.3 mm", "0.7 mm", "<0.3 mm"],
                [
                    Color32::from_rgb(0xFD, 0xE7, 0x25),
                    Color32::from_rgb(0x21, 0x91, 0x8C),
                    Color32::from_rgb(0x3B, 0x52, 0x8B),
                    Color32::from_rgb(0x44, 0x01, 0x54),
                ],
            ),
            StlColorMode::OcclusalDistance => (
                tr("Distancia", "Distance", "Distância"),
                ["+1.5 mm", "0.8 mm", "0.2 mm", "≤0.0 mm"],
                [
                    Color32::from_rgb(0x1D, 0x4E, 0xD8),
                    Color32::from_rgb(0x10, 0xB9, 0x81),
                    Color32::from_rgb(0xF5, 0x9E, 0x0B),
                    Color32::from_rgb(0xEF, 0x44, 0x44),
                ],
            ),
            StlColorMode::Undercut => (
                tr("Socavado", "Undercut", "Retenção"),
                ["+30°", "0°", "-10°", "-30°"],
                [
                    Color32::from_rgb(0x10, 0xB9, 0x81),
                    Color32::from_rgb(0xFA, 0xCC, 0x15),
                    Color32::from_rgb(0xF9, 0x73, 0x16),
                    Color32::from_rgb(0xEF, 0x44, 0x44),
                ],
            ),
            StlColorMode::Curvature => (
                tr("Curvatura", "Curvature", "Curvatura"),
                [
                    tr("Alta", "High", "Alta"),
                    tr("Media", "Medium", "Média"),
                    tr("Suave", "Soft", "Suave"),
                    tr("Plana", "Flat", "Plana"),
                ],
                [
                    Color32::from_rgb(0xFC, 0xFD, 0xBF),
                    Color32::from_rgb(0xF1, 0x60, 0x5D),
                    Color32::from_rgb(0x72, 0x1F, 0x81),
                    Color32::from_rgb(0x18, 0x0F, 0x3E),
                ],
            ),
            StlColorMode::Solid => return,
        };

        painter.text(
            Pos2::new(bg_rect.center().x, by - 14.0),
            Align2::CENTER_CENTER,
            title,
            egui::FontId::proportional(10.5),
            CREAM,
        );

        let steps = 24usize;
        let seg_h = bar_h / steps as f32;
        for s in 0..steps {
            let t = s as f32 / (steps - 1) as f32;
            let scaled = t * 3.0;
            let idx = (scaled.floor() as usize).min(2);
            let f = scaled - idx as f32;
            let c0 = stops[idx];
            let c1 = stops[idx + 1];
            let col = Color32::from_rgb(
                (c0.r() as f32 * (1.0 - f) + c1.r() as f32 * f) as u8,
                (c0.g() as f32 * (1.0 - f) + c1.g() as f32 * f) as u8,
                (c0.b() as f32 * (1.0 - f) + c1.b() as f32 * f) as u8,
            );
            let r = Rect::from_min_size(Pos2::new(bx, by + s as f32 * seg_h), egui::vec2(bar_w, seg_h + 0.8));
            painter.rect_filled(r, CornerRadius::ZERO, col);
        }
        painter.rect_stroke(
            Rect::from_min_size(Pos2::new(bx, by), egui::vec2(bar_w, bar_h)),
            CornerRadius::ZERO,
            Stroke::new(1.0_f32, CREAM_MUTED),
            egui::StrokeKind::Outside,
        );

        for (i, tick) in ticks.iter().enumerate() {
            let ty = by + (i as f32 / 3.0) * bar_h;
            painter.text(
                Pos2::new(bx + bar_w + 5.0, ty),
                Align2::LEFT_CENTER,
                *tick,
                egui::FontId::proportional(9.2),
                CREAM,
            );
        }
    }

    pub fn ui(
        &mut self,
        ctx: &egui::Context,
        active_case: Option<&DentalCase>,
        case_manager: &CaseManager,
        renderer: &Arc<Mutex<GlViewportRenderer>>,
    ) -> Option<String> {
        let mut status_msg = None;

        // Procesar captura PNG si el SO acaba de entregar el framebuffer
        if self.pending_screenshot_path.is_some() {
            let mut captured = None;
            ctx.input(|i| {
                for ev in &i.events {
                    if let egui::Event::Screenshot { image, .. } = ev {
                        captured = Some(Arc::clone(image));
                    }
                }
            });
            if let (Some(img), Some(out_path)) = (captured, self.pending_screenshot_path.take()) {
                let mut rgba = Vec::with_capacity(img.pixels.len() * 4);
                for c in &img.pixels {
                    rgba.extend_from_slice(&[c.r(), c.g(), c.b(), c.a()]);
                }
                match win_assoc::write_png_rgba(&out_path, img.size[0] as u32, img.size[1] as u32, &rgba) {
                    Ok(()) => {
                        status_msg = Some(format!(
                            "{} {}",
                            tr("📷 Captura PNG guardada:", "📷 PNG screenshot saved:", "📷 Captura PNG salva:"),
                            out_path.display()
                        ));
                    }
                    Err(e) => {
                        status_msg = Some(e);
                    }
                }
            }
        }

        // Atajos de teclado estilo 3Shape 3D Viewer (cuando ningún campo de texto tiene foco)
        if !ctx.wants_keyboard_input() {
            let (ctrl, k1, k2, k3, k4, k5, kf, kr, kw, kc, km, ka, kh, ko) = ctx.input(|i| {
                (
                    i.modifiers.ctrl,
                    i.key_pressed(Key::Num1),
                    i.key_pressed(Key::Num2),
                    i.key_pressed(Key::Num3),
                    i.key_pressed(Key::Num4),
                    i.key_pressed(Key::Num5),
                    i.key_pressed(Key::F),
                    i.key_pressed(Key::R),
                    i.key_pressed(Key::W),
                    i.key_pressed(Key::C),
                    i.key_pressed(Key::M),
                    i.key_pressed(Key::A),
                    i.key_pressed(Key::H),
                    i.key_pressed(Key::O),
                )
            });
            if ctrl && ko {
                let mut dlg = rfd::FileDialog::new()
                    .set_title(tr(
                        "Abrir Escena (.apexscene) o Malla 3D en Apex 3D Viewer",
                        "Open Scene (.apexscene) or 3D Mesh in Apex 3D Viewer",
                        "Abrir Cena (.apexscene) ou Malha 3D no Apex 3D Viewer",
                    ))
                    .add_filter(
                        tr("Escenas y Mallas 3D", "3D Scenes & Meshes", "Cenas e Malhas 3D"),
                        &["apexscene", "stl", "obj", "ply"],
                    )
                    .add_filter("Escena Apex DentalCraft (.apexscene)", &["apexscene"]);
                let casos_root = scene_file::default_casos_dir();
                if casos_root.exists() {
                    dlg = dlg.set_directory(&casos_root);
                }
                if let Some(files) = dlg.pick_files() {
                    status_msg = Some(self.open_files(&files));
                }
            } else {
                if k1 { self.camera.set_view_occlusal_top(); }
                if k2 { self.camera.set_view_occlusal_bottom(); }
                if k3 { self.camera.set_view_frontal(); }
                if k4 { self.camera.set_view_lingual(); }
                if k5 { self.camera.set_view_lateral(); }
                if kf { self.frame_selected(); }
                if kr { self.frame_scene(); }
                if kw {
                    if let Some(sel) = self.selected_idx.filter(|&i| i < self.objects.len()) {
                        self.objects[sel].wireframe = !self.objects[sel].wireframe;
                    }
                }
                if kc { self.clip_enabled = !self.clip_enabled; }
                if km {
                    self.measure_tool = if self.measure_tool == MeasureToolMode::Distance2Pt {
                        MeasureToolMode::None
                    } else {
                        self.pending_pts.clear();
                        MeasureToolMode::Distance2Pt
                    };
                }
                if ka {
                    self.measure_tool = if self.measure_tool == MeasureToolMode::Angle3Pt {
                        MeasureToolMode::None
                    } else {
                        self.pending_pts.clear();
                        MeasureToolMode::Angle3Pt
                    };
                }
                if kh { self.hide_panels = !self.hide_panels; }
            }
        }

        // ==================================================================
        // LIENZO 3D A PANTALLA COMPLETA (Visor 3D Multi-Objeto OpenGL 3.3)
        // ==================================================================
        egui::CentralPanel::default()
            .frame(Frame::NONE.fill(BG_DEEP))
            .show(ctx, |ui| {
                let rect = ui.available_rect_before_wrap();
                let resp = ui.allocate_rect(rect, Sense::click_and_drag());

                // Cámara 360° libre (Der+arrastre = Rotar 360°; Alt+Der = Rodar; Rueda = Pan/Zoom; Izq = Solo acción)
                self.camera.handle_navigation_input(&resp, ui, rect, true);

                // Clic limpio en modo Regla 3D (2 pts) o Goniómetro (3 pts)
                if self.measure_tool != MeasureToolMode::None {
                    if ui.input(|i| i.pointer.primary_pressed()) && resp.hovered() {
                        self.press_start_pos = ui.input(|i| i.pointer.interact_pos());
                    }
                    if ui.input(|i| i.pointer.primary_released()) {
                        if let (Some(sp), Some(ep)) =
                            (self.press_start_pos.take(), ui.input(|i| i.pointer.interact_pos()))
                        {
                            if sp.distance(ep) <= 5.0 && rect.contains(ep) {
                                if let Some(hit) = self.pick_visible_surface(ep, rect) {
                                    self.pending_pts.push(hit);
                                    match self.measure_tool {
                                        MeasureToolMode::Distance2Pt if self.pending_pts.len() >= 2 => {
                                            let p1 = self.pending_pts[0];
                                            let p2 = self.pending_pts[1];
                                            self.distances.push(DistanceMeasurement { p1, p2 });
                                            self.pending_pts.clear();
                                        }
                                        MeasureToolMode::Angle3Pt if self.pending_pts.len() >= 3 => {
                                            let a = self.pending_pts[0];
                                            let vertex = self.pending_pts[1];
                                            let b = self.pending_pts[2];
                                            self.angles.push(AngleMeasurement { a, vertex, b });
                                            self.pending_pts.clear();
                                        }
                                        _ => {}
                                    }
                                    self.measure_rev = self.bump();
                                }
                            }
                        }
                    }
                }

                // Plano de corte transversal GPU
                let clip_plane = if self.clip_enabled {
                    let (mn, mx) = self
                        .scene_bounds()
                        .unwrap_or((Vec3::splat(-10.0), Vec3::splat(10.0)));
                    let center = 0.5 * (mn + mx);
                    let half = (0.5 * (mx - mn)).max(Vec3::splat(1.0));
                    let mut n = match self.clip_axis {
                        ClipAxisMode::X => Vec3::X,
                        ClipAxisMode::Y => Vec3::Y,
                        ClipAxisMode::Z => Vec3::Z,
                        ClipAxisMode::Camera => self.camera.forward().normalize_or_zero(),
                    };
                    if self.clip_invert {
                        n = -n;
                    }
                    let span =
                        (half.x * n.x.abs() + half.y * n.y.abs() + half.z * n.z.abs()).max(1.0);
                    let pt = center + n * (self.clip_pos_norm * span);
                    Some(Vec4::new(n.x, n.y, n.z, -n.dot(pt)))
                } else {
                    None
                };

                // Subir y ensamblar capas OpenGL 3.3
                let (spec_scale, shininess_val) = match self.shading_style {
                    ShadingStyle::CeramicGloss => (1.0_f32, 40.0_f32),
                    ShadingStyle::PlasterMatte => (0.28_f32, 14.0_f32),
                    ShadingStyle::HighContrast => (1.55_f32, 72.0_f32),
                };

                let mut layers = Vec::new();
                if let Ok(mut guard) = renderer.lock() {
                    for obj in &self.objects {
                        if !obj.visible || obj.mesh.is_empty() {
                            continue;
                        }
                        let (cmap, suffix, use_vc) = match obj.color_mode {
                            StlColorMode::Solid => (ColorMapKind::Solid, "s", false),
                            StlColorMode::WallThickness | StlColorMode::DieClearance => {
                                (ColorMapKind::ThicknessViridis, "th", true)
                            }
                            StlColorMode::OcclusalDistance => {
                                (ColorMapKind::OcclusalJetR, "occ", true)
                            }
                            StlColorMode::Undercut => (ColorMapKind::UndercutRdYlGn, "und", true),
                            StlColorMode::Curvature => (ColorMapKind::CurvatureMagma, "crv", true),
                        };
                        let key = format!("sv_obj_{}_{}", obj.id, suffix);
                        guard.ensure_mesh_uploaded(&key, obj.rev, &obj.mesh, cmap);
                        let mut sp = RenderLayerSpec::solid(key, obj.rev, obj.tint, obj.opacity);
                        sp.use_vertex_color = use_vc;
                        sp.wireframe = obj.wireframe;
                        let base_spec = match obj.role {
                            SceneRole::Restoration | SceneRole::GenericStl => 0.48,
                            SceneRole::CementSpace | SceneRole::MinThickness => 0.24,
                            _ => 0.30,
                        };
                        sp.specular = (base_spec * spec_scale).clamp(0.08, 0.90);
                        sp.shininess = shininess_val;
                        layers.push(sp);
                    }

                    // Línea de margen vectorial
                    if self.show_margin && self.margin_curve.len() >= 4 {
                        let tube =
                            build_tube_mesh(&self.margin_curve, self.margin_line_radius_mm, true, 10);
                        guard.ensure_mesh_uploaded("sv_margin", self.margin_rev, &tube, ColorMapKind::Solid);
                        let mut sp =
                            RenderLayerSpec::solid("sv_margin", self.margin_rev, [0.20, 0.85, 0.62], 1.0);
                        sp.clip_affected = false;
                        layers.push(sp);
                    }

                    // Pines y tubos 3D de TODAS las mediciones (Distancias + Ángulos + Puntos pendientes)
                    if !self.distances.is_empty() || !self.angles.is_empty() || !self.pending_pts.is_empty() {
                        let mut m_mesh = TriMeshData::default();
                        for dm in &self.distances {
                            m_mesh.merge_in_place(&build_uv_sphere(dm.p1, 0.14, 10, 12));
                            m_mesh.merge_in_place(&build_uv_sphere(dm.p2, 0.14, 10, 12));
                            m_mesh.merge_in_place(&build_tube_mesh(&[dm.p1, dm.p2], 0.048, false, 8));
                        }
                        for am in &self.angles {
                            m_mesh.merge_in_place(&build_uv_sphere(am.a, 0.13, 8, 10));
                            m_mesh.merge_in_place(&build_uv_sphere(am.vertex, 0.16, 10, 12));
                            m_mesh.merge_in_place(&build_uv_sphere(am.b, 0.13, 8, 10));
                            m_mesh.merge_in_place(&build_tube_mesh(&[am.a, am.vertex, am.b], 0.048, false, 8));
                        }
                        for &pt in &self.pending_pts {
                            m_mesh.merge_in_place(&build_uv_sphere(pt, 0.15, 10, 12));
                        }
                        if self.pending_pts.len() == 2 {
                            m_mesh.merge_in_place(&build_tube_mesh(
                                &[self.pending_pts[0], self.pending_pts[1]],
                                0.045,
                                false,
                                8,
                            ));
                        }
                        if !m_mesh.is_empty() {
                            guard.ensure_mesh_uploaded(
                                "sv_measure",
                                self.measure_rev,
                                &m_mesh,
                                ColorMapKind::Solid,
                            );
                            let mut sp = RenderLayerSpec::solid(
                                "sv_measure",
                                self.measure_rev,
                                [0.20, 0.95, 0.68],
                                1.0,
                            );
                            sp.unlit = true;
                            sp.clip_affected = false;
                            sp.always_on_top = true;
                            layers.push(sp);
                        }
                    }
                }

                ui.painter().add(make_viewport_paint_callback(
                    Arc::clone(renderer),
                    rect,
                    &self.camera,
                    layers,
                    clip_plane,
                ));

                // Etiquetas flotantes en pantalla para cada distancia (mm) y ángulo (°)
                let painter = ui.painter();
                for (i, dm) in self.distances.iter().enumerate() {
                    let mid = 0.5 * (dm.p1 + dm.p2);
                    if let Some(sp) = self.camera.world_to_screen(mid, rect) {
                        let txt = format!("#{} {:.2} mm", i + 1, dm.p1.distance(dm.p2));
                        let bg = Rect::from_center_size(sp + egui::vec2(0.0, -14.0), egui::vec2(84.0, 20.0));
                        painter.rect_filled(bg, CornerRadius::same(5), Color32::from_rgba_unmultiplied(0x16, 0x04, 0x1E, 232));
                        painter.rect_stroke(bg, CornerRadius::same(5), Stroke::new(1.0_f32, EMERALD), egui::StrokeKind::Inside);
                        painter.text(bg.center(), Align2::CENTER_CENTER, txt, egui::FontId::proportional(10.5), EMERALD);
                    }
                }
                for (i, am) in self.angles.iter().enumerate() {
                    if let Some(sp) = self.camera.world_to_screen(am.vertex, rect) {
                        let txt = format!("∠{} {:.1}°", i + 1, am.angle_deg());
                        let bg = Rect::from_center_size(sp + egui::vec2(0.0, -16.0), egui::vec2(72.0, 20.0));
                        painter.rect_filled(bg, CornerRadius::same(5), Color32::from_rgba_unmultiplied(0x16, 0x04, 0x1E, 232));
                        painter.rect_stroke(bg, CornerRadius::same(5), Stroke::new(1.0_f32, AMBER), egui::StrokeKind::Inside);
                        painter.text(bg.center(), Align2::CENTER_CENTER, txt, egui::FontId::proportional(10.5), AMBER);
                    }
                }

                // Leyenda graduada flotante en el borde derecho del viewport si hay mapa de color activo
                if let Some(sel) = self.selected_idx.filter(|&i| i < self.objects.len()) {
                    let cm = self.objects[sel].color_mode;
                    if cm != StlColorMode::Solid {
                        self.paint_colorbar_legend(painter, rect, cm);
                    }
                }

                // Pantalla de bienvenida si aún no hay STL cargado
                if self.objects.is_empty() {
                    painter.text(
                        rect.center() - egui::vec2(0.0, 22.0),
                        Align2::CENTER_CENTER,
                        tr(
                            "Apex 3D Viewer — Visor Dental Nativo 60 FPS",
                            "Apex 3D Viewer — Native 60 FPS Dental Viewer",
                            "Apex 3D Viewer — Visualizador Dental Nativo 60 FPS",
                        ),
                        egui::FontId::proportional(22.0),
                        CREAM,
                    );
                    painter.text(
                        rect.center() + egui::vec2(0.0, 10.0),
                        Align2::CENTER_CENTER,
                        tr(
                            "Arrastra archivos o carpetas .STL / .OBJ / .PLY aquí, o pulsa '📂 Abrir STL/OBJ...' (Ctrl+O)",
                            "Drag .STL / .OBJ / .PLY files or folders here, or click '📂 Open STL/OBJ...' (Ctrl+O)",
                            "Arraste arquivos ou pastas .STL / .OBJ / .PLY aqui, ou clique em '📂 Abrir STL/OBJ...' (Ctrl+O)",
                        ),
                        egui::FontId::proportional(13.0),
                        CREAM_MUTED,
                    );
                    painter.text(
                        rect.center() + egui::vec2(0.0, 34.0),
                        Align2::CENTER_CENTER,
                        tr(
                            "Atajos: [1-5] Vistas | [F] Centrar | [W] Malla | [C] Corte | [M] Regla | [A] Ángulo | [H] Ocultar paneles",
                            "Shortcuts: [1-5] Views | [F] Frame | [W] Wireframe | [C] Clip | [M] Ruler | [A] Angle | [H] Hide panels",
                            "Atalhos: [1-5] Vistas | [F] Enquadrar | [W] Malha | [C] Corte | [M] Régua | [A] Ângulo | [H] Ocultar painéis",
                        ),
                        egui::FontId::proportional(11.0),
                        AMETHYST_BRIGHT,
                    );
                }
            });

        // ==================================================================
        // BARRA FLOTANTE SUPERIOR CENTRAL: Vistas Rápidas 360° + Modo Presentación
        // ==================================================================
        let top_y = if self.standalone_mode { 56.0 } else { 68.0 };
        Area::new(egui::Id::new("sv_cam_bar"))
            .anchor(Align2::CENTER_TOP, egui::vec2(0.0, top_y))
            .order(Order::Foreground)
            .show(ctx, |ui| {
                Frame::NONE
                    .fill(Color32::from_rgba_unmultiplied(0x21, 0x05, 0x2C, 238))
                    .stroke(Stroke::new(1.3_f32, AMETHYST))
                    .corner_radius(CornerRadius::same(8))
                    .inner_margin(Margin::symmetric(8, 5))
                    .show(ui, |ui| {
                        ui.horizontal(|ui| {
                            if ui.button(tr("1 Oclusal", "1 Occlusal", "1 Oclusal")).clicked() { self.camera.set_view_occlusal_top(); }
                            if ui.button("2 Intaglio").clicked() { self.camera.set_view_occlusal_bottom(); }
                            if ui.button(tr("3 Vestibular", "3 Buccal", "3 Vestibular")).clicked() { self.camera.set_view_frontal(); }
                            if ui.button("4 Lingual").clicked() { self.camera.set_view_lingual(); }
                            if ui.button("5 Lateral").clicked() { self.camera.set_view_lateral(); }
                            if ui.button(tr("F Centrar", "F Frame", "F Enquadrar")).clicked() { self.frame_scene(); }
                            ui.separator();
                            let ui_lbl = if self.hide_panels {
                                tr("👁 Mostrar Paneles [H]", "👁 Show Panels [H]", "👁 Mostrar Painéis [H]")
                            } else {
                                tr("👁 Pantalla Limpia [H]", "👁 Clean View [H]", "👁 Tela Limpa [H]")
                            };
                            if ui.button(ui_lbl).clicked() {
                                self.hide_panels = !self.hide_panels;
                            }
                        });
                    });
            });

        if self.hide_panels {
            return status_msg;
        }

        let card_y = if self.standalone_mode { 62.0 } else { 74.0 };

        // ==================================================================
        // VENTANA FLOTANTE IZQ 1: Archivos y Escenas (estilo 3Shape 3D Viewer)
        // ==================================================================
        Window::new(tr(
            "1. Escena y Objetos 3D",
            "1. 3D Scene & Objects",
            "1. Cena e Objetos 3D",
        ))
            .id(egui::Id::new("sv_card_scene"))
            .default_pos(egui::pos2(14.0, card_y))
            .default_width(342.0)
            .collapsible(true)
            .resizable(false)
            .show(ctx, |ui| {
                ui.horizontal(|ui| {
                    if theme::primary_button(ui, tr("📂 Abrir STL / Escena...", "📂 Open STL / Scene...", "📂 Abrir STL / Cena...")).clicked() {
                        let mut dlg = rfd::FileDialog::new()
                            .set_title(tr(
                                "Abrir Escena (.apexscene) o Malla(s) 3D",
                                "Open Scene (.apexscene) or 3D Mesh(es)",
                                "Abrir Cena (.apexscene) ou Malha(s) 3D",
                            ))
                            .add_filter(
                                tr("Escenas y Mallas 3D", "3D Scenes & Meshes", "Cenas e Malhas 3D"),
                                &["apexscene", "stl", "obj", "ply"],
                            )
                            .add_filter("Escena Apex DentalCraft (.apexscene)", &["apexscene"]);
                        let casos_root = scene_file::default_casos_dir();
                        if casos_root.exists() {
                            dlg = dlg.set_directory(&casos_root);
                        }
                        if let Some(files) = dlg.pick_files() {
                            status_msg = Some(self.open_files(&files));
                        }
                    }
                    if ui
                        .button(tr("📁 Carpeta", "📁 Folder", "📁 Pasta"))
                        .on_hover_text(tr(
                            "Cargar todos los STL/OBJ/PLY de una carpeta",
                            "Load all STL/OBJ/PLY files from a folder",
                            "Carregar todos os STL/OBJ/PLY de uma pasta",
                        ))
                        .clicked()
                    {
                        if let Some(folder) = rfd::FileDialog::new()
                            .set_title(tr("Abrir carpeta con modelos 3D", "Open folder with 3D models", "Abrir pasta com modelos 3D"))
                            .pick_folder()
                        {
                            status_msg = Some(self.open_files(&[folder]));
                        }
                    }
                    if ui.button(tr("Limpiar", "Clear", "Limpar")).clicked() && !self.objects.is_empty() {
                        self.clear_scene();
                        status_msg = Some(
                            tr(
                                "Escena del Visor 3D limpiada.",
                                "3D Viewer scene cleared.",
                                "Cena do Visualizador 3D limpa.",
                            )
                            .to_string(),
                        );
                    }
                });

                if let Some(case) = active_case {
                    if theme::accent_button(ui, tr("★ Cargar Escena del Caso Activo", "★ Load Active Case Scene", "★ Carregar Cena do Caso Ativo")).clicked() {
                        status_msg = Some(self.load_active_case_scene(case, case_manager));
                    }
                }

                // Integración Windows ("Abrir con Apex 3D Viewer") y Captura PNG rápida
                ui.horizontal(|ui| {
                    if ui
                        .button(tr("📷 Captura PNG", "📷 PNG Snapshot", "📷 Captura PNG"))
                        .on_hover_text(tr(
                            "Guardar imagen limpia del visor 3D",
                            "Save clean image of the 3D viewport",
                            "Salvar imagem limpa da vista 3D",
                        ))
                        .clicked()
                    {
                        if let Some(out) = rfd::FileDialog::new()
                            .set_title(tr("Guardar captura PNG de Apex 3D Viewer", "Save Apex 3D Viewer PNG Snapshot", "Salvar captura PNG do Apex 3D Viewer"))
                            .set_file_name("Apex3DViewer_Captura.png")
                            .add_filter(tr("Imagen PNG", "PNG Image", "Imagem PNG"), &["png"])
                            .save_file()
                        {
                            self.pending_screenshot_path = Some(out);
                            ctx.send_viewport_cmd(egui::ViewportCommand::Screenshot(Default::default()));
                        }
                    }
                    if ui
                        .button(tr("📌 Asociar en Windows", "📌 Associate in Windows", "📌 Associar no Windows"))
                        .on_hover_text(tr(
                            "Añade 'Apex 3D Viewer' al menú clic derecho → Abrir con... para archivos .STL/.OBJ/.PLY",
                            "Adds 'Apex 3D Viewer' to right-click → Open with... for .STL/.OBJ/.PLY files",
                            "Adiciona 'Apex 3D Viewer' ao menu clique direito → Abrir com... para arquivos .STL/.OBJ/.PLY",
                        ))
                        .clicked()
                    {
                        if let Some(exe) = win_assoc::find_standalone_viewer_exe() {
                            match win_assoc::register_windows_open_with(&exe, true) {
                                Ok(m) => status_msg = Some(m),
                                Err(e) => status_msg = Some(e),
                            }
                        } else {
                            status_msg = Some(
                                tr(
                                    "Coloca Apex3DViewer.exe junto a ApexDentalCraft.exe para asociarlo en Windows.",
                                    "Place Apex3DViewer.exe next to ApexDentalCraft.exe to associate it in Windows.",
                                    "Coloque Apex3DViewer.exe ao lado de ApexDentalCraft.exe para associá-lo no Windows.",
                                )
                                .to_string(),
                            );
                        }
                    }
                });

                ui.add_space(4.0);
                ui.label(
                    RichText::new(tr(
                        "Escenas Rápidas (estilo 3Shape):",
                        "Quick Presets (3Shape style):",
                        "Cenas Rápidas (estilo 3Shape):",
                    ))
                    .strong()
                    .size(10.5)
                    .color(AMETHYST_BRIGHT),
                );
                ui.horizontal_wrapped(|ui| {
                    if ui.small_button(tr("Clínica", "Clinical", "Clínica")).clicked() { self.apply_scene_preset("clinical"); }
                    if ui.small_button(tr("Solo Corona", "Crown Only", "Só Coroa")).clicked() { self.apply_scene_preset("restoration_only"); }
                    if ui.small_button("Intaglio").clicked() { self.apply_scene_preset("intaglio"); }
                    if ui.small_button(tr("Ajuste/Cemento", "Fit / Cement", "Ajuste/Cimento")).clicked() { self.apply_scene_preset("fit_cement"); }
                    if ui.small_button(tr("Oclusión", "Occlusion", "Oclusão")).clicked() { self.apply_scene_preset("occlusion"); }
                    if ui.small_button(tr("Rayos X", "X-Ray", "Raio-X")).clicked() { self.apply_scene_preset("xray"); }
                    if ui.small_button(tr("Todo 100%", "All 100%", "Tudo 100%")).clicked() { self.apply_scene_preset("all"); }
                });

                ui.horizontal(|ui| {
                    ui.label(RichText::new(tr("Sombreado:", "Shading:", "Sombreamento:")).size(10.0).color(CREAM_MUTED));
                    egui::ComboBox::from_id_salt("sv_shading_style")
                        .width(150.0)
                        .selected_text(self.shading_style.label())
                        .show_ui(ui, |ui| {
                            ui.selectable_value(&mut self.shading_style, ShadingStyle::CeramicGloss, ShadingStyle::CeramicGloss.label());
                            ui.selectable_value(&mut self.shading_style, ShadingStyle::PlasterMatte, ShadingStyle::PlasterMatte.label());
                            ui.selectable_value(&mut self.shading_style, ShadingStyle::HighContrast, ShadingStyle::HighContrast.label());
                        });
                });

                if !self.margin_curve.is_empty() {
                    ui.checkbox(&mut self.show_margin, tr("Mostrar Línea de Margen Vectorial", "Show Vector Margin Line", "Mostrar Linha de Margem Vetorial"));
                }

                ui.separator();
                ui.label(
                    RichText::new(format!("{} ({}):", tr("Objetos en Escena", "Scene Objects", "Objetos na Cena"), self.objects.len()))
                        .strong()
                        .size(11.0)
                        .color(CREAM),
                );

                let mut remove_idx = None;
                let mut isolate_idx = None;
                egui::ScrollArea::vertical().max_height(260.0).show(ui, |ui| {
                    for idx in 0..self.objects.len() {
                        let is_sel = self.selected_idx == Some(idx);
                        let frame_stroke = if is_sel {
                            Stroke::new(1.4_f32, EMERALD)
                        } else {
                            Stroke::new(1.0_f32, BORDER_PLUM)
                        };
                        Frame::NONE
                            .fill(if is_sel { Color32::from_rgb(0x2B, 0x09, 0x3A) } else { BG_PANEL })
                            .stroke(frame_stroke)
                            .corner_radius(CornerRadius::same(6))
                            .inner_margin(Margin::same(6))
                            .show(ui, |ui| {
                                ui.horizontal(|ui| {
                                    ui.checkbox(&mut self.objects[idx].visible, "");
                                    let lbl = format!(
                                        "{} [{}]",
                                        self.objects[idx].name,
                                        self.objects[idx].role.label()
                                    );
                                    if ui
                                        .selectable_label(is_sel, RichText::new(lbl).size(10.5).strong())
                                        .clicked()
                                    {
                                        self.selected_idx = Some(idx);
                                    }
                                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                                        if ui.small_button("✕").on_hover_text(tr("Quitar de la escena", "Remove from scene", "Remover da cena")).clicked() {
                                            remove_idx = Some(idx);
                                        }
                                        if ui.small_button(tr("Solo", "Solo", "Isolar")).on_hover_text(tr("Aislar este objeto", "Isolate this object", "Isolar este objeto")).clicked() {
                                            isolate_idx = Some(idx);
                                        }
                                    });
                                });
                                ui.horizontal(|ui| {
                                    ui.label(RichText::new(tr("Opacidad:", "Opacity:", "Opacidade:")).size(9.8).color(CREAM_MUTED));
                                    ui.add(
                                        egui::Slider::new(&mut self.objects[idx].opacity, 0.05..=1.0)
                                            .show_value(false),
                                    );
                                    ui.label(
                                        RichText::new(format!("{:.0}%", self.objects[idx].opacity * 100.0))
                                            .size(9.8)
                                            .color(CREAM_MUTED),
                                    );
                                    ui.checkbox(&mut self.objects[idx].wireframe, RichText::new(tr("Malla", "Wire", "Malha")).size(9.8));
                                });
                            });
                        ui.add_space(3.0);
                    }
                });

                if let Some(iso) = isolate_idx {
                    for (i, o) in self.objects.iter_mut().enumerate() {
                        o.visible = i == iso;
                        if i == iso {
                            o.opacity = 1.0;
                        }
                    }
                    self.selected_idx = Some(iso);
                    self.frame_scene();
                }
                if let Some(rm) = remove_idx {
                    self.objects.remove(rm);
                    if self.objects.is_empty() {
                        self.selected_idx = None;
                    } else {
                        self.selected_idx = Some(rm.min(self.objects.len() - 1));
                    }
                }
            });

        // ==================================================================
        // VENTANA FLOTANTE DER 1: Análisis 3Shape (Grosores, Distancias, Socavados, CAM)
        // ==================================================================
        let right_x = (ctx.screen_rect().width() - 344.0).max(380.0);
        Window::new(tr(
            "2. Análisis 3Shape y Auditoría CAM",
            "2. 3Shape Analysis & CAM Audit",
            "2. Análise 3Shape e Auditoria CAM",
        ))
            .id(egui::Id::new("sv_card_analysis"))
            .default_pos(egui::pos2(right_x, card_y))
            .default_width(330.0)
            .collapsible(true)
            .resizable(false)
            .show(ctx, |ui| {
                if let Some(sel) = self.selected_idx.filter(|&i| i < self.objects.len()) {
                    let obj_name = self.objects[sel].name.clone();
                    let cur_mode = self.objects[sel].color_mode;
                    ui.label(
                        RichText::new(format!("{} {}", tr("Objeto activo:", "Active object:", "Objeto ativo:"), obj_name))
                            .strong()
                            .color(EMERALD)
                            .size(11.0),
                    );

                    let mode_label = |m: StlColorMode| -> &'static str {
                        match m {
                            StlColorMode::Solid => tr("Acabado Sólido / Material PBR", "Solid Finish / PBR Material", "Acabamento Sólido / Material PBR"),
                            StlColorMode::WallThickness => tr("★ Mapa de Grosor de Pared (mm)", "★ Wall Thickness Map (mm)", "★ Mapa de Espessura de Parede (mm)"),
                            StlColorMode::DieClearance => tr("Holgura / Espesor contra Referencia (mm)", "Clearance / Thickness vs Reference (mm)", "Folga / Espessura vs Referência (mm)"),
                            StlColorMode::OcclusalDistance => tr("Contactos / Distancia Oclusal (mm)", "Occlusal Contacts / Distance (mm)", "Contatos / Distância Oclusal (mm)"),
                            StlColorMode::Undercut => tr("Mapa de Socavados (° Eje Inserción)", "Undercut Map (° Insertion Axis)", "Mapa de Retenções (° Eixo Inserção)"),
                            StlColorMode::Curvature => tr("Mapa de Curvatura / Líneas y Filos", "Curvature / Margins & Sharp Edges", "Mapa de Curvatura / Margens e Arestas"),
                        }
                    };
                    let mut wanted_mode = cur_mode;
                    egui::ComboBox::from_id_salt("sv_color_mode")
                        .width(ui.available_width())
                        .selected_text(mode_label(cur_mode))
                        .show_ui(ui, |ui| {
                            ui.selectable_value(&mut wanted_mode, StlColorMode::Solid, mode_label(StlColorMode::Solid));
                            ui.selectable_value(&mut wanted_mode, StlColorMode::WallThickness, mode_label(StlColorMode::WallThickness));
                            ui.selectable_value(&mut wanted_mode, StlColorMode::DieClearance, mode_label(StlColorMode::DieClearance));
                            ui.selectable_value(&mut wanted_mode, StlColorMode::OcclusalDistance, mode_label(StlColorMode::OcclusalDistance));
                            ui.selectable_value(&mut wanted_mode, StlColorMode::Undercut, mode_label(StlColorMode::Undercut));
                            ui.selectable_value(&mut wanted_mode, StlColorMode::Curvature, mode_label(StlColorMode::Curvature));
                        });
                    if wanted_mode != cur_mode {
                        self.ensure_color_analysis(sel, wanted_mode);
                    }

                    // Selector de objeto de referencia para mapas de distancia/oclusión
                    if matches!(
                        self.objects[sel].color_mode,
                        StlColorMode::OcclusalDistance | StlColorMode::DieClearance
                    ) && self.objects.len() >= 2
                    {
                        let sel_id = self.objects[sel].id;
                        let auto_lbl = tr("Automático (según rol)", "Automatic (by role)", "Automático (por função)");
                        let ref_label = self
                            .reference_obj_id
                            .and_then(|rid| self.objects.iter().find(|o| o.id == rid && o.id != sel_id))
                            .map(|o| o.name.clone())
                            .unwrap_or_else(|| auto_lbl.to_string());
                        let mut changed_ref = false;
                        ui.horizontal(|ui| {
                            ui.label(RichText::new(tr("Referencia:", "Reference:", "Referência:")).size(10.0).color(CREAM_MUTED));
                            egui::ComboBox::from_id_salt("sv_ref_obj")
                                .width(215.0)
                                .selected_text(ref_label)
                                .show_ui(ui, |ui| {
                                    if ui.selectable_value(&mut self.reference_obj_id, None, auto_lbl).clicked() {
                                        changed_ref = true;
                                    }
                                    let others: Vec<(u64, String)> = self
                                        .objects
                                        .iter()
                                        .filter(|o| o.id != sel_id)
                                        .map(|o| (o.id, o.name.clone()))
                                        .collect();
                                    for (oid, oname) in others {
                                        if ui.selectable_value(&mut self.reference_obj_id, Some(oid), oname).clicked() {
                                            changed_ref = true;
                                        }
                                    }
                                });
                        });
                        if changed_ref {
                            let m = self.objects[sel].color_mode;
                            self.ensure_color_analysis(sel, m);
                        }
                    }

                    // Botón 3Shape "Fijar eje de inserción desde vista actual" en modo Socavados
                    if self.objects[sel].color_mode == StlColorMode::Undercut {
                        ui.horizontal(|ui| {
                            if ui.small_button(tr("🧭 Fijar Eje desde Vista Actual", "🧭 Set Axis from Current View", "🧭 Definir Eixo da Vista Atual")).clicked() {
                                self.undercut_axis = (-self.camera.forward()).normalize_or_zero();
                                self.ensure_color_analysis(sel, StlColorMode::Undercut);
                            }
                            if ui.small_button(tr("Eje +Z", "+Z Axis", "Eixo +Z")).clicked() {
                                self.undercut_axis = Vec3::Z;
                                self.ensure_color_analysis(sel, StlColorMode::Undercut);
                            }
                        });
                    }

                    ui.horizontal(|ui| {
                        ui.label(RichText::new(tr("Umbral grosor:", "Thickness threshold:", "Limiar de espessura:")).size(10.0).color(CREAM_MUTED));
                        if ui
                            .add(egui::Slider::new(&mut self.min_thickness_warn_mm, 0.25..=1.50).suffix(" mm"))
                            .changed()
                            && matches!(
                                self.objects[sel].color_mode,
                                StlColorMode::WallThickness | StlColorMode::DieClearance
                            )
                        {
                            self.ensure_color_analysis(sel, self.objects[sel].color_mode);
                        }
                    });

                    // Estadísticas clínicas del modo activo
                    match self.objects[sel].color_mode {
                        StlColorMode::WallThickness | StlColorMode::DieClearance => {
                            if let Some((mn, avg, mx)) = self.objects[sel].thickness_stats {
                                let pct = self.objects[sel].thin_vert_pct;
                                Frame::NONE
                                    .fill(BG_DEEP)
                                    .stroke(Stroke::new(1.0_f32, if pct > 5.0 { AMBER } else { EMERALD }))
                                    .corner_radius(CornerRadius::same(6))
                                    .inner_margin(Margin::same(6))
                                    .show(ui, |ui| {
                                        ui.label(
                                            RichText::new(format!(
                                                "{} {:.2} mm | {} {:.2} mm | {} {:.2} mm",
                                                tr("Mín:", "Min:", "Mín:"),
                                                mn,
                                                tr("Prom:", "Avg:", "Méd:"),
                                                avg,
                                                tr("Máx:", "Max:", "Máx:"),
                                                mx
                                            ))
                                            .strong()
                                            .size(10.5)
                                            .color(CREAM),
                                        );
                                        ui.label(
                                            RichText::new(format!(
                                                "{} < {:.2} mm: {:.1}% {}",
                                                tr("Zona", "Zone", "Zona"),
                                                self.min_thickness_warn_mm,
                                                pct,
                                                tr("de la superficie", "of surface", "da superfície")
                                            ))
                                            .size(10.0)
                                            .color(if pct > 5.0 { AMBER } else { EMERALD }),
                                        );
                                    });
                            }
                        }
                        StlColorMode::OcclusalDistance => {
                            if let Some((mn, avg, mx)) = self.objects[sel].distance_stats {
                                ui.label(
                                    RichText::new(format!(
                                        "{} {:.2} mm | {} {:.2} mm | {} {:.2} mm",
                                        tr("Distancia Mín:", "Min Dist:", "Distância Mín:"),
                                        mn,
                                        tr("Prom:", "Avg:", "Méd:"),
                                        avg,
                                        tr("Máx:", "Max:", "Máx:"),
                                        mx
                                    ))
                                    .size(10.2)
                                    .color(CYAN),
                                );
                            } else {
                                ui.label(
                                    RichText::new(tr(
                                        "Carga una segunda malla (ej. Antagonista) para medir contactos.",
                                        "Load a second mesh (e.g. Antagonist) to measure contacts.",
                                        "Carregue uma segunda malha (ex. Antagonista) para medir contatos.",
                                    ))
                                    .size(9.8)
                                    .color(AMBER),
                                );
                            }
                        }
                        StlColorMode::Solid => {
                            ui.label(RichText::new(tr("Material / Color:", "Material / Shade:", "Material / Cor:")).size(10.0).color(CREAM_MUTED));
                            ui.horizontal_wrapped(|ui| {
                                for &(lbl, rgb) in MATERIAL_SWATCHES {
                                    let c32 = Color32::from_rgb(
                                        (rgb[0] * 255.0) as u8,
                                        (rgb[1] * 255.0) as u8,
                                        (rgb[2] * 255.0) as u8,
                                    );
                                    if ui
                                        .add(egui::Button::new(RichText::new("●").color(c32)).small())
                                        .on_hover_text(lbl)
                                        .clicked()
                                    {
                                        self.objects[sel].tint = rgb;
                                    }
                                }
                            });
                        }
                        StlColorMode::Undercut => {
                            ui.label(
                                RichText::new(tr(
                                    "Rojo = Socavado retentivo (< 0°) | Verde = Expulsivo (> 0°)",
                                    "Red = Retentive undercut (< 0°) | Green = Non-undercut (> 0°)",
                                    "Vermelho = Retenção (< 0°) | Verde = Expulsivo (> 0°)",
                                ))
                                .size(9.8)
                                .color(CREAM_MUTED),
                            );
                        }
                        StlColorMode::Curvature => {
                            ui.label(
                                RichText::new(tr(
                                    "Amarillo/Rojo = Filos, márgenes o crestas agudas",
                                    "Yellow/Red = Sharp edges, margins, or ridges",
                                    "Amarelo/Vermelho = Margens, arestas ou cristas agudas",
                                ))
                                .size(9.8)
                                .color(CREAM_MUTED),
                            );
                        }
                    }

                    // Auditoría CAM + Herramientas rápidas de malla
                    ui.separator();
                    let a = &self.objects[sel].audit;
                    if a.watertight && a.folded_edges == 0 {
                        ui.label(
                            RichText::new(tr(
                                "✔ MALLA WATERTIGHT 2-MANIFOLD (0 PLIEGUES)",
                                "✔ WATERTIGHT 2-MANIFOLD MESH (0 FOLDS)",
                                "✔ MALHA ESTANQUE 2-MANIFOLD (0 DOBRAS)",
                            ))
                            .strong()
                            .size(10.2)
                            .color(EMERALD),
                        );
                    } else {
                        ui.label(
                            RichText::new(format!(
                                "⚠ {}: {} | {}: {} | Inv: {} | {}: {}",
                                tr("Abiertas", "Open", "Abertas"),
                                a.open_edges,
                                tr("No-manif", "Non-manif", "Não-manif"),
                                a.non_manifold_edges,
                                a.flipped_winding_edges,
                                tr("Pliegues", "Folds", "Dobras"),
                                a.folded_edges
                            ))
                            .strong()
                            .size(10.0)
                            .color(AMBER),
                        );
                    }
                    let vol_cm3 = a.volume_mm3 / 1000.0;
                    ui.label(
                        RichText::new(format!(
                            "Dim: {:.2}×{:.2}×{:.2} mm | {} {:.1} mm² | Vol: {:.1} mm³\n{} {} | Tris: {} | {} {:.2} mm\nZirconia: {:.2} g | {}: {:.2} g",
                            a.dims_mm.x,
                            a.dims_mm.y,
                            a.dims_mm.z,
                            tr("Área:", "Area:", "Área:"),
                            self.objects[sel].area_mm2,
                            a.volume_mm3,
                            tr("Vért:", "Verts:", "Vért:"),
                            a.n_vertices,
                            a.n_faces,
                            tr("Arista máx:", "Max edge:", "Aresta máx:"),
                            self.objects[sel].max_edge_mm,
                            vol_cm3 * 6.05,
                            tr("Disilicato", "LiSi₂", "Dissilicato"),
                            vol_cm3 * 2.50,
                        ))
                        .size(9.6)
                        .color(CREAM_MUTED),
                    );

                    ui.add_space(3.0);
                    ui.horizontal(|ui| {
                        if ui.small_button(tr("🔄 Invertir Normales", "🔄 Flip Normals", "🔄 Inverter Normais")).clicked() {
                            status_msg = self.flip_selected_normals();
                        }
                        if ui.small_button(tr("💾 Guardar / Convertir STL·OBJ·PLY", "💾 Save / Convert STL·OBJ·PLY", "💾 Salvar / Converter STL·OBJ·PLY")).clicked() {
                            let def_name = format!("{}.stl", self.objects[sel].name.trim_end_matches(".stl"));
                            if let Some(out) = rfd::FileDialog::new()
                                .set_title(tr("Exportar / Convertir Malla 3D", "Export / Convert 3D Mesh", "Exportar / Converter Malha 3D"))
                                .set_file_name(&def_name)
                                .add_filter(tr("STL Binario", "Binary STL", "STL Binário"), &["stl"])
                                .add_filter("Wavefront OBJ", &["obj"])
                                .add_filter("PLY ASCII", &["ply"])
                                .save_file()
                            {
                                match self.export_selected_to(&out) {
                                    Ok(m) => status_msg = Some(m),
                                    Err(e) => status_msg = Some(e),
                                }
                            }
                        }
                    });
                } else {
                    ui.label(
                        RichText::new(tr(
                            "Carga o selecciona un objeto 3D para analizar grosores, distancias y malla.",
                            "Load or select a 3D object to inspect thickness, distances, and mesh quality.",
                            "Carregue ou selecione um objeto 3D para analisar espessuras, distâncias e malha.",
                        ))
                        .size(10.5)
                        .color(CREAM_MUTED),
                    );
                }
            });

        // ==================================================================
        // VENTANA FLOTANTE DER 2: Corte Transversal 2D/3D y Mediciones 3D (Regla + Ángulo)
        // ==================================================================
        Window::new(tr(
            "3. Corte Transversal y Mediciones 3D",
            "3. Cross-Section & 3D Measurements",
            "3. Corte Transversal e Medições 3D",
        ))
            .id(egui::Id::new("sv_card_tools"))
            .default_pos(egui::pos2(right_x, card_y + 392.0))
            .default_width(330.0)
            .collapsible(true)
            .resizable(false)
            .show(ctx, |ui| {
                ui.checkbox(
                    &mut self.clip_enabled,
                    RichText::new(tr(
                        "Activar Corte Transversal [C]",
                        "Enable Cross-Section [C]",
                        "Ativar Corte Transversal [C]",
                    ))
                    .strong(),
                );
                if self.clip_enabled {
                    ui.horizontal(|ui| {
                        ui.label(tr("Eje:", "Axis:", "Eixo:"));
                        ui.selectable_value(&mut self.clip_axis, ClipAxisMode::X, "X");
                        ui.selectable_value(&mut self.clip_axis, ClipAxisMode::Y, "Y");
                        ui.selectable_value(&mut self.clip_axis, ClipAxisMode::Z, "Z");
                        ui.selectable_value(&mut self.clip_axis, ClipAxisMode::Camera, tr("Cámara", "Camera", "Câmera"));
                        ui.checkbox(&mut self.clip_invert, tr("Invertir", "Invert", "Inverter"));
                    });
                    ui.add(egui::Slider::new(&mut self.clip_pos_norm, -1.0..=1.0).text(tr("Plano", "Plane", "Plano")));
                }

                ui.separator();
                ui.horizontal(|ui| {
                    if ui
                        .selectable_label(
                            self.measure_tool == MeasureToolMode::Distance2Pt,
                            RichText::new(tr("📏 Regla [M]", "📏 Ruler [M]", "📏 Régua [M]")).strong().color(CYAN),
                        )
                        .clicked()
                    {
                        self.pending_pts.clear();
                        self.measure_tool = if self.measure_tool == MeasureToolMode::Distance2Pt {
                            MeasureToolMode::None
                        } else {
                            MeasureToolMode::Distance2Pt
                        };
                    }
                    if ui
                        .selectable_label(
                            self.measure_tool == MeasureToolMode::Angle3Pt,
                            RichText::new(tr("📐 Ángulo [A]", "📐 Angle [A]", "📐 Ângulo [A]")).strong().color(AMBER),
                        )
                        .clicked()
                    {
                        self.pending_pts.clear();
                        self.measure_tool = if self.measure_tool == MeasureToolMode::Angle3Pt {
                            MeasureToolMode::None
                        } else {
                            MeasureToolMode::Angle3Pt
                        };
                    }
                    if ui.small_button(tr("Borrar todo", "Clear all", "Limpar tudo")).clicked() {
                        self.pending_pts.clear();
                        self.distances.clear();
                        self.angles.clear();
                        self.measure_rev = self.bump();
                    }
                });

                match self.measure_tool {
                    MeasureToolMode::Distance2Pt => {
                        ui.label(
                            RichText::new(if self.pending_pts.is_empty() {
                                tr(
                                    "Haz clic en el Punto 1 sobre cualquier malla...",
                                    "Click Point 1 on any mesh...",
                                    "Clique no Ponto 1 em qualquer malha...",
                                )
                            } else {
                                tr(
                                    "Punto 1 fijado — Haz clic en el Punto 2...",
                                    "Point 1 set — Click Point 2...",
                                    "Ponto 1 definido — Clique no Ponto 2...",
                                )
                            })
                            .size(9.8)
                            .color(CYAN),
                        );
                    }
                    MeasureToolMode::Angle3Pt => {
                        let msg = match self.pending_pts.len() {
                            0 => tr("Haz clic en Punto A (lado 1)...", "Click Point A (side 1)...", "Clique no Ponto A (lado 1)..."),
                            1 => tr("Haz clic en el Vértice del ángulo...", "Click the Angle Vertex...", "Clique no Vértice do ângulo..."),
                            _ => tr("Haz clic en Punto B (lado 2)...", "Click Point B (side 2)...", "Clique no Ponto B (lado 2)..."),
                        };
                        ui.label(RichText::new(msg).size(9.8).color(AMBER));
                    }
                    MeasureToolMode::None => {}
                }

                let mut del_dist = None;
                for (i, dm) in self.distances.iter().enumerate() {
                    let d = dm.p1.distance(dm.p2);
                    let dv = (dm.p2 - dm.p1).abs();
                    ui.horizontal(|ui| {
                        ui.label(
                            RichText::new(format!(
                                "#{}: {:.3} mm (Δ {:.1}/{:.1}/{:.1})",
                                i + 1, d, dv.x, dv.y, dv.z
                            ))
                            .size(10.2)
                            .color(EMERALD),
                        );
                        if ui.small_button("✕").clicked() {
                            del_dist = Some(i);
                        }
                    });
                }
                if let Some(i) = del_dist {
                    self.distances.remove(i);
                    self.measure_rev = self.bump();
                }

                let mut del_ang = None;
                for (i, am) in self.angles.iter().enumerate() {
                    ui.horizontal(|ui| {
                        ui.label(
                            RichText::new(format!("∠{}: {:.2}°", i + 1, am.angle_deg()))
                                .size(10.2)
                                .color(AMBER),
                        );
                        if ui.small_button("✕").clicked() {
                            del_ang = Some(i);
                        }
                    });
                }
                if let Some(i) = del_ang {
                    self.angles.remove(i);
                    self.measure_rev = self.bump();
                }
            });

        status_msg
    }
}
