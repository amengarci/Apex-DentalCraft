//! Formato de Escena Nativo de **Apex DentalCraft** (`.apexscene`)
//!
//! Guarda de forma autocontenida la orden dental completa, parámetros clínicos,
//! curvas de margen, ejes de inserción, conectores de puente, referencias relativas
//! a los archivos STL/OBJ exportados (muñón/modelo de trabajo, antagonista, mordida y
//! restauraciones/puentes diseñados) y el estado de vista para poder reabrir cualquier
//! escena en el programa o en Apex 3D Viewer.

use crate::core::case_manager::{CaseManager, DentalCase};
use crate::geometry::mesh::TriMeshData;
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;
use std::fs;
use std::path::{Path, PathBuf};

pub const APEX_SCENE_FORMAT_ID: &str = "ApexDentalCraftScene";
pub const APEX_SCENE_FORMAT_VERSION: u32 = 1;
pub const APEX_SCENE_EXTENSION: &str = "apexscene";

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
#[serde(default)]
pub struct ApexSceneModels {
    pub die_model: Option<String>,
    pub die_arch: String,
    pub antagonist_model: Option<String>,
    pub antagonist_arch: String,
    pub bite_model: Option<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
#[serde(default)]
pub struct ApexSceneRestoration {
    pub label: String,
    pub file_name: String,
    pub fdis: Vec<i32>,
    pub is_bridge: bool,
    pub connector_summary: Vec<String>,
    pub watertight: bool,
    pub n_faces: usize,
    pub volume_mm3: f32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default)]
pub struct ApexSceneViewState {
    pub camera_target: [f32; 3],
    pub camera_orientation: [f32; 4],
    pub camera_distance: f32,
    pub isolated_crown: bool,
}

impl Default for ApexSceneViewState {
    fn default() -> Self {
        Self {
            camera_target: [0.0, 0.0, 0.0],
            camera_orientation: [0.0, 0.0, 0.0, 1.0],
            camera_distance: 48.0,
            isolated_crown: false,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default)]
pub struct ApexSceneFile {
    pub format: String,
    pub version: u32,
    pub application: String,
    pub exported_at: String,
    pub case_data: DentalCase,
    pub models: ApexSceneModels,
    pub restorations: Vec<ApexSceneRestoration>,
    pub unit_designs: BTreeMap<i32, String>,
    pub view_state: ApexSceneViewState,
}

impl Default for ApexSceneFile {
    fn default() -> Self {
        Self {
            format: APEX_SCENE_FORMAT_ID.to_string(),
            version: APEX_SCENE_FORMAT_VERSION,
            application: "APEX DENTALCRAFT".to_string(),
            exported_at: String::new(),
            case_data: DentalCase::default(),
            models: ApexSceneModels::default(),
            restorations: Vec::new(),
            unit_designs: BTreeMap::new(),
            view_state: ApexSceneViewState::default(),
        }
    }
}

/// Directorio raíz donde se encuentra el ejecutable del programa.
pub fn program_dir() -> PathBuf {
    std::env::current_exe()
        .ok()
        .and_then(|p| p.parent().map(|d| d.to_path_buf()))
        .unwrap_or_else(|| std::env::current_dir().unwrap_or_else(|_| PathBuf::from(".")))
}

/// Carpeta `Casos` dentro del directorio del programa.
pub fn default_casos_dir() -> PathBuf {
    program_dir().join("Casos")
}

/// Carpeta específica de un caso dentro de una carpeta raíz `Casos`.
pub fn case_export_dir_in(casos_root: &Path, case: &DentalCase) -> PathBuf {
    casos_root.join(&case.case_id)
}

/// Carpeta de exportación por defecto del caso dentro de `<carpeta_programa>/Casos/<case_id>`.
pub fn default_case_export_dir(case: &DentalCase) -> PathBuf {
    case_export_dir_in(&default_casos_dir(), case)
}

/// Comprueba si una ruta corresponde al formato de escena `.apexscene`.
pub fn is_apex_scene_path(path: &Path) -> bool {
    path.is_file()
        && path
            .extension()
            .and_then(|s| s.to_str())
            .map(|e| e.eq_ignore_ascii_case(APEX_SCENE_EXTENSION))
            .unwrap_or(false)
}

pub fn save_apex_scene(path: &Path, scene: &ApexSceneFile) -> Result<(), String> {
    if let Some(parent) = path.parent() {
        fs::create_dir_all(parent).map_err(|e| format!("No se pudo crear carpeta '{}': {}", parent.display(), e))?;
    }
    let json = serde_json::to_string_pretty(scene)
        .map_err(|e| format!("Error serializando escena Apex DentalCraft: {}", e))?;
    fs::write(path, json).map_err(|e| format!("No se pudo guardar escena '{}': {}", path.display(), e))
}

pub fn load_apex_scene(path: &Path) -> Result<ApexSceneFile, String> {
    let raw = fs::read_to_string(path)
        .map_err(|e| format!("No se pudo leer archivo de escena '{}': {}", path.display(), e))?;
    let mut scene: ApexSceneFile = serde_json::from_str(&raw)
        .map_err(|e| format!("Formato inválido en '{}': {}", path.display(), e))?;
    if scene.format != APEX_SCENE_FORMAT_ID || scene.version < 1 {
        return Err(format!(
            "El archivo '{}' no corresponde a una escena válida de Apex DentalCraft.",
            path.display()
        ));
    }
    scene.case_data.ensure_restorations_initialized();
    Ok(scene)
}

fn copy_if_distinct(src: &Path, dst: &Path) -> Result<(), String> {
    if let Some(parent) = dst.parent() {
        fs::create_dir_all(parent).map_err(|e| e.to_string())?;
    }
    let same = match (fs::canonicalize(src), fs::canonicalize(dst)) {
        (Ok(a), Ok(b)) => a == b,
        _ => src == dst,
    };
    if !same {
        fs::copy(src, dst).map_err(|e| format!("Error copiando '{}' -> '{}': {}", src.display(), dst.display(), e))?;
    }
    Ok(())
}

/// Reabre un archivo `.apexscene` importando sus modelos STL/OBJ (muñón, antagonista, mordida)
/// y restauraciones diseñadas en el `CaseManager` para poder seguir editando o inspeccionando el caso.
pub fn import_scene_into_case_manager(
    scene_path: &Path,
    case_manager: &CaseManager,
) -> Result<(DentalCase, ApexSceneFile), String> {
    let scene = load_apex_scene(scene_path)?;
    let base_dir = scene_path
        .parent()
        .ok_or_else(|| "La ruta de escena no tiene directorio padre.".to_string())?;

    let mut dental_case = scene.case_data.clone();
    if dental_case.case_id.trim().is_empty() {
        let stem = scene_path
            .file_stem()
            .and_then(|s| s.to_str())
            .unwrap_or("EscenaReabierta");
        dental_case.case_id = stem
            .chars()
            .map(|c| if c.is_alphanumeric() || c == '_' || c == '-' { c } else { '_' })
            .collect();
    }
    dental_case.ensure_restorations_initialized();

    let case_dir = case_manager.get_case_dir(&dental_case.case_id);
    for sub in ["raw", "working", "design", "exports"] {
        fs::create_dir_all(case_dir.join(sub)).map_err(|e| e.to_string())?;
    }
    case_manager.save_case(&mut dental_case)?;

    // 1. Restaurar modelos anatómicos (Muñón, Antagonista y Mordida)
    let die_arch = if scene.models.die_arch == "maxilla" || scene.models.die_arch == "mandible" {
        scene.models.die_arch.as_str()
    } else if dental_case.prep_arch == "maxilla" {
        "maxilla"
    } else {
        "mandible"
    };
    let ant_arch = if scene.models.antagonist_arch == "maxilla" || scene.models.antagonist_arch == "mandible" {
        scene.models.antagonist_arch.as_str()
    } else if die_arch == "maxilla" {
        "mandible"
    } else {
        "maxilla"
    };

    if let Some(rel) = &scene.models.die_model {
        let src = base_dir.join(rel);
        if src.is_file() {
            case_manager.import_scan(&mut dental_case, die_arch, &src)?;
        }
    }
    if let Some(rel) = &scene.models.antagonist_model {
        let src = base_dir.join(rel);
        if src.is_file() {
            case_manager.import_scan(&mut dental_case, ant_arch, &src)?;
        }
    }
    if let Some(rel) = &scene.models.bite_model {
        let src = base_dir.join(rel);
        if src.is_file() {
            case_manager.import_scan(&mut dental_case, "bite", &src)?;
        }
    }

    // 2. Restaurar mallas unitarias para continuidad editable en Módulo 4
    let design_dir = case_dir.join("design");
    let mut restored_units = std::collections::BTreeSet::new();
    for (&fdi, rel) in &scene.unit_designs {
        let src = base_dir.join(rel);
        if src.is_file() {
            let ext = src
                .extension()
                .and_then(|s| s.to_str())
                .unwrap_or("stl")
                .to_ascii_lowercase();
            let dst_rel = format!("design/crown_fdi_{}.{}", fdi, ext);
            let dst = case_dir.join(&dst_rel);
            copy_if_distinct(&src, &dst)?;
            if let Some(unit) = dental_case.restorations.get_mut(&fdi) {
                unit.crown_design_path = Some(dst_rel);
            }
            restored_units.insert(fdi);
        }
    }

    // 3. Restaurar piezas exportadas (coronas individuales sin entrada previa o puentes fusionados finales)
    for rest in &scene.restorations {
        let src = base_dir.join(&rest.file_name);
        if !src.is_file() {
            continue;
        }
        let ext = src
            .extension()
            .and_then(|s| s.to_str())
            .unwrap_or("stl")
            .to_ascii_lowercase();
        if rest.is_bridge && rest.fdis.len() >= 2 {
            let mut sorted_fdis = rest.fdis.clone();
            sorted_fdis.sort_unstable();
            let fdi_tag = sorted_fdis
                .iter()
                .map(|d| d.to_string())
                .collect::<Vec<_>>()
                .join("_");
            let final_stl = design_dir.join(format!("bridge_{}_final.stl", fdi_tag));
            if ext == "stl" {
                copy_if_distinct(&src, &final_stl)?;
            } else if let Ok(mesh) = TriMeshData::load_from_path(&src) {
                mesh.save_stl_binary(&final_stl)?;
            }
        } else if !rest.is_bridge && rest.fdis.len() == 1 {
            let fdi = rest.fdis[0];
            if !restored_units.contains(&fdi) {
                let dst_rel = format!("design/crown_fdi_{}.{}", fdi, ext);
                let dst = case_dir.join(&dst_rel);
                copy_if_distinct(&src, &dst)?;
                if let Some(unit) = dental_case.restorations.get_mut(&fdi) {
                    unit.crown_design_path = Some(dst_rel);
                }
            }
        }
    }

    // Sincronizar campos planos del diente activo antes de guardar
    let active_fdi = dental_case.tooth_number;
    dental_case.set_active_tooth(active_fdi);
    case_manager.save_case(&mut dental_case)?;
    Ok((dental_case, scene))
}
