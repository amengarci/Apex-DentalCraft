//! Configuración Global Persistente de **Apex DentalCraft** (`settings.json`).
//!
//! Se guarda automáticamente dentro de la carpeta raíz activa (`C:\LabCAD\` en modo local
//! o junto al ejecutable en modo Portable USB), permitiendo personalizar grosores de margen,
//! bisel de cementado por defecto, transparencias de Etapa 4, sensibilidad de cámara y visor STL.

use crate::core::i18n::AppLanguage;
use serde::{Deserialize, Serialize};
use std::fs;
use std::path::{Path, PathBuf};

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default)]
pub struct AppSettings {
    // --- Idioma / Language ---
    #[serde(default)]
    pub language: AppLanguage,

    // --- Línea de Margen (Etapa 3 y 4) ---
    pub margin_line_radius_mm: f32,
    pub margin_node_radius_mm: f32,
    pub margin_magnetic_snap_default: bool,
    pub margin_live_red_validation: bool,

    // --- Cementado y Bisel Marginal por Defecto ---
    pub default_margin_bevel_mm: f32,
    pub default_margin_bevel_angle_deg: f32,
    pub default_auto_adapt_antagonist: bool,

    // --- Transparencias Iniciales en Etapa 4 ---
    pub stage4_restoration_opacity: f32,
    pub stage4_model_opacity: f32,
    pub stage4_show_antagonist: bool,
    pub stage4_show_cement: bool,
    pub stage4_cement_opacity: f32,
    pub stage4_show_min_thickness: bool,
    pub stage4_min_thickness_opacity: f32,

    // --- Cámara 3D Libre 360° ---
    pub camera_rotate_sensitivity: f32,
    pub camera_zoom_sensitivity: f32,
    pub camera_invert_zoom: bool,

    // --- Visor STL Propio ---
    pub viewer_min_thickness_warn_mm: f32,
}

impl Default for AppSettings {
    fn default() -> Self {
        Self {
            language: AppLanguage::En,
            margin_line_radius_mm: 0.032,
            margin_node_radius_mm: 0.095,
            margin_magnetic_snap_default: true,
            margin_live_red_validation: true,
            default_margin_bevel_mm: 0.06,
            default_margin_bevel_angle_deg: 45.0,
            default_auto_adapt_antagonist: true,
            stage4_restoration_opacity: 1.00,
            stage4_model_opacity: 0.80,
            stage4_show_antagonist: false,
            stage4_show_cement: true,
            stage4_cement_opacity: 0.55,
            stage4_show_min_thickness: true,
            stage4_min_thickness_opacity: 0.45,
            camera_rotate_sensitivity: 1.00,
            camera_zoom_sensitivity: 1.00,
            camera_invert_zoom: false,
            viewer_min_thickness_warn_mm: 0.60,
        }
    }
}

impl AppSettings {
    pub fn settings_path(root_dir: &Path) -> PathBuf {
        root_dir.join("settings.json")
    }

    pub fn load_from_root(root_dir: &Path) -> Self {
        let p = Self::settings_path(root_dir);
        if let Ok(text) = fs::read_to_string(&p) {
            if let Ok(mut parsed) = serde_json::from_str::<Self>(&text) {
                parsed.clamp_all();
                return parsed;
            }
        }
        Self::default()
    }

    pub fn save_to_root(&self, root_dir: &Path) -> Result<(), String> {
        let p = Self::settings_path(root_dir);
        if let Some(parent) = p.parent() {
            let _ = fs::create_dir_all(parent);
        }
        let mut clean = self.clone();
        clean.clamp_all();
        let json = serde_json::to_string_pretty(&clean).map_err(|e| e.to_string())?;
        fs::write(&p, json).map_err(|e| format!("No se pudo guardar {:?}: {}", p, e))
    }

    pub fn clamp_all(&mut self) {
        self.margin_line_radius_mm = self.margin_line_radius_mm.clamp(0.015, 0.120);
        self.margin_node_radius_mm = self.margin_node_radius_mm.clamp(0.050, 0.220);
        self.default_margin_bevel_mm = self.default_margin_bevel_mm.clamp(0.00, 0.25);
        self.default_margin_bevel_angle_deg = self.default_margin_bevel_angle_deg.clamp(25.0, 70.0);
        self.stage4_restoration_opacity = self.stage4_restoration_opacity.clamp(0.10, 1.00);
        self.stage4_model_opacity = self.stage4_model_opacity.clamp(0.10, 1.00);
        self.stage4_cement_opacity = self.stage4_cement_opacity.clamp(0.05, 1.00);
        self.stage4_min_thickness_opacity = self.stage4_min_thickness_opacity.clamp(0.05, 1.00);
        self.camera_rotate_sensitivity = self.camera_rotate_sensitivity.clamp(0.40, 2.20);
        self.camera_zoom_sensitivity = self.camera_zoom_sensitivity.clamp(0.40, 2.20);
        self.viewer_min_thickness_warn_mm = self.viewer_min_thickness_warn_mm.clamp(0.25, 1.50);
    }
}
