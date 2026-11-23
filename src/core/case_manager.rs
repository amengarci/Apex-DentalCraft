//! Módulo 1 (Núcleo de Datos): Gestor de Casos Multi-Pieza y Materiales Clínicos
//!
//! 100% compatible con la estructura local `C:/LabCAD/Casos/<case_id>/case_metadata.json`,
//! preservando órdenes multi-proceso (Coronas individuales, Pilares/Pónticos de puente,
//! Carillas, Incrustaciones y Cofias) y los 7 presets clínicos de materiales CAD/CAM.

use chrono::Local;
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet, VecDeque};
use std::fs;
use std::path::{Path, PathBuf};

/// Tipos de proceso protésico asignables en el Odontograma FDI.
pub const APEX_PROCESS_TYPES: &[&str] = &[
    "Corona Individual",
    "Pilar de Puente",
    "Póntico de Puente",
    "Carilla Estética (Veneer)",
    "Incrustación (Inlay / Onlay)",
    "Cofia / Estructura",
];

/// Tonos guía VITA Classical / Bleached.
pub const VITA_SHADES: &[&str] = &[
    "A1", "A2", "A3", "A3.5", "A4", "B1", "B2", "B3", "BL1", "BL2", "BL3", "C1", "C2", "D2", "D3",
];

#[derive(Debug, Clone)]
pub struct MaterialPreset {
    pub name: &'static str,
    pub cement_gap_mm: f32,
    pub marginal_gap_mm: f32,
    pub margin_ramp_width_mm: f32,
    pub smooth_distance_mm: f32,
    pub drill_compensation_enabled: bool,
    pub drill_radius_mm: f32,
    pub drill_compensation_offset_mm: f32,
    pub min_thickness_mm: f32,
    pub min_thickness_veneer_mm: f32,
    pub proximal_contact_offset_mm: f32,
    pub occlusal_contact_offset_mm: f32,
    pub connector_area_mm2: f32,
    pub description: &'static str,
}

/// Valores predeterminados clínicos investigados por material dental CAD/CAM.
pub const APEX_MATERIAL_PRESETS: &[MaterialPreset] = &[
    MaterialPreset {
        name: "Zirconia Monolítica (3Y-TZP Alta Resistencia)",
        cement_gap_mm: 0.045,
        marginal_gap_mm: 0.020,
        margin_ramp_width_mm: 1.00,
        smooth_distance_mm: 0.25,
        drill_compensation_enabled: true,
        drill_radius_mm: 0.55,
        drill_compensation_offset_mm: 0.05,
        min_thickness_mm: 0.60,
        min_thickness_veneer_mm: 0.45,
        proximal_contact_offset_mm: -0.02,
        occlusal_contact_offset_mm: 0.00,
        connector_area_mm2: 9.0,
        description: "1100-1200 MPa | Espesor mín. 0.60 mm | Cemento 45 µm | Fresa R=0.55 mm | Conector 9 mm²",
    },
    MaterialPreset {
        name: "Zirconia Translúcida Multicapa (4Y/5Y-PSZ Estética)",
        cement_gap_mm: 0.050,
        marginal_gap_mm: 0.020,
        margin_ramp_width_mm: 1.00,
        smooth_distance_mm: 0.25,
        drill_compensation_enabled: true,
        drill_radius_mm: 0.55,
        drill_compensation_offset_mm: 0.05,
        min_thickness_mm: 0.80,
        min_thickness_veneer_mm: 0.50,
        proximal_contact_offset_mm: -0.02,
        occlusal_contact_offset_mm: 0.00,
        connector_area_mm2: 12.0,
        description: "650-850 MPa | Espesor mín. 0.80 mm | Cemento 50 µm | Fresa R=0.55 mm | Conector 12 mm²",
    },
    MaterialPreset {
        name: "e.max (Disilicato de Litio IPS e.max CAD/Press)",
        cement_gap_mm: 0.060,
        marginal_gap_mm: 0.025,
        margin_ramp_width_mm: 0.90,
        smooth_distance_mm: 0.25,
        drill_compensation_enabled: true,
        drill_radius_mm: 0.60,
        drill_compensation_offset_mm: 0.06,
        min_thickness_mm: 1.00,
        min_thickness_veneer_mm: 0.45,
        proximal_contact_offset_mm: -0.01,
        occlusal_contact_offset_mm: 0.02,
        connector_area_mm2: 16.0,
        description: "400-530 MPa (Adhesivo) | Espesor 1.00 mm (0.45 Carilla) | Cemento 60 µm | Fresa R=0.60 mm",
    },
    MaterialPreset {
        name: "PMMA Fresado (Provisional Larga Duración)",
        cement_gap_mm: 0.065,
        marginal_gap_mm: 0.025,
        margin_ramp_width_mm: 0.85,
        smooth_distance_mm: 0.25,
        drill_compensation_enabled: true,
        drill_radius_mm: 0.55,
        drill_compensation_offset_mm: 0.05,
        min_thickness_mm: 0.80,
        min_thickness_veneer_mm: 0.60,
        proximal_contact_offset_mm: -0.01,
        occlusal_contact_offset_mm: 0.00,
        connector_area_mm2: 12.0,
        description: "PMMA CAD/CAM | Espesor mín. 0.80 mm | Cemento 65 µm | Fresa R=0.55 mm | Conector 12 mm²",
    },
    MaterialPreset {
        name: "Resina Impresa 3D (Provisional / Híbrida)",
        cement_gap_mm: 0.080,
        marginal_gap_mm: 0.035,
        margin_ramp_width_mm: 0.85,
        smooth_distance_mm: 0.20,
        drill_compensation_enabled: false,
        drill_radius_mm: 0.40,
        drill_compensation_offset_mm: 0.00,
        min_thickness_mm: 1.00,
        min_thickness_veneer_mm: 0.65,
        proximal_contact_offset_mm: 0.00,
        occlusal_contact_offset_mm: 0.03,
        connector_area_mm2: 14.0,
        description: "Compensación DLP/MSLA | Espesor mín. 1.00 mm | Cemento 80 µm | Sin fresado | Conector 14 mm²",
    },
    MaterialPreset {
        name: "Cera / Resina Castable (Calcinable para Prensado)",
        cement_gap_mm: 0.040,
        marginal_gap_mm: 0.015,
        margin_ramp_width_mm: 0.80,
        smooth_distance_mm: 0.25,
        drill_compensation_enabled: true,
        drill_radius_mm: 0.50,
        drill_compensation_offset_mm: 0.04,
        min_thickness_mm: 0.65,
        min_thickness_veneer_mm: 0.50,
        proximal_contact_offset_mm: -0.02,
        occlusal_contact_offset_mm: 0.00,
        connector_area_mm2: 14.0,
        description: "Patrón Calcinable | Espesor mín. 0.65 mm | Cemento 40 µm | Fresa R=0.50 mm",
    },
    MaterialPreset {
        name: "CoCr / Aleación Metálica (Estructura / Cofia)",
        cement_gap_mm: 0.040,
        marginal_gap_mm: 0.015,
        margin_ramp_width_mm: 0.75,
        smooth_distance_mm: 0.25,
        drill_compensation_enabled: true,
        drill_radius_mm: 0.60,
        drill_compensation_offset_mm: 0.05,
        min_thickness_mm: 0.40,
        min_thickness_veneer_mm: 0.40,
        proximal_contact_offset_mm: -0.01,
        occlusal_contact_offset_mm: 0.00,
        connector_area_mm2: 7.0,
        description: "Alta Rigidez | Espesor mín. 0.40 mm | Cemento 40 µm | Fresa R=0.60 mm | Conector 7 mm²",
    },
];

pub fn find_material_preset(name: &str) -> &'static MaterialPreset {
    let low = name.to_lowercase();
    for p in APEX_MATERIAL_PRESETS {
        let p_low = p.name.to_lowercase();
        if p.name == name || p_low.contains(&low) || low.contains(&p_low) {
            return p;
        }
    }
    &APEX_MATERIAL_PRESETS[0]
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default)]
pub struct DesignParameters {
    pub library_id: String,
    pub custom_tooth_file: String,
    pub material_preset: String,
    pub process_type: String,
    pub cement_gap_mm: f32,
    pub marginal_gap_mm: f32,
    pub margin_ramp_width_mm: f32,
    pub smooth_distance_mm: f32,
    pub margin_bevel_mm: f32,
    pub margin_bevel_angle_deg: f32,
    pub drill_compensation_enabled: bool,
    pub drill_radius_mm: f32,
    pub drill_compensation_offset_mm: f32,
    pub remove_undercuts: bool,
    pub auto_adapt_antagonist: bool,
    pub min_thickness_mm: f32,
    pub proximal_contact_offset_mm: f32,
    pub occlusal_contact_offset_mm: f32,
    pub connector_area_mm2: f32,
    pub anatomy_tx: f32,
    pub anatomy_ty: f32,
    pub anatomy_tz: f32,
    pub anatomy_rx: f32,
    pub anatomy_ry: f32,
    pub anatomy_rz: f32,
    pub anatomy_sx: f32,
    pub anatomy_sy: f32,
    pub anatomy_sz: f32,
    pub cusp_height_delta_mm: f32,
    pub fossa_depth_delta_mm: f32,
    pub equator_bulge_delta_mm: f32,
    pub flip_library_z: bool,
}

impl Default for DesignParameters {
    fn default() -> Self {
        Self {
            library_id: "LIBRERIA APEX DETAILED".to_string(),
            custom_tooth_file: String::new(),
            material_preset: APEX_MATERIAL_PRESETS[0].name.to_string(),
            process_type: APEX_PROCESS_TYPES[0].to_string(),
            cement_gap_mm: 0.045,
            marginal_gap_mm: 0.020,
            margin_ramp_width_mm: 1.00,
            smooth_distance_mm: 0.25,
            margin_bevel_mm: 0.06,
            margin_bevel_angle_deg: 45.0,
            drill_compensation_enabled: true,
            drill_radius_mm: 0.55,
            drill_compensation_offset_mm: 0.05,
            remove_undercuts: true,
            auto_adapt_antagonist: true,
            min_thickness_mm: 0.60,
            proximal_contact_offset_mm: -0.02,
            occlusal_contact_offset_mm: 0.00,
            connector_area_mm2: 9.0,
            anatomy_tx: 0.0,
            anatomy_ty: 0.0,
            anatomy_tz: 0.0,
            anatomy_rx: 0.0,
            anatomy_ry: 0.0,
            anatomy_rz: 0.0,
            anatomy_sx: 1.0,
            anatomy_sy: 1.0,
            anatomy_sz: 1.0,
            cusp_height_delta_mm: 0.0,
            fossa_depth_delta_mm: 0.0,
            equator_bulge_delta_mm: 0.0,
            flip_library_z: false,
        }
    }
}

impl DesignParameters {
    pub fn apply_material_and_process_preset(&mut self, material_name: &str, process_type: &str) {
        let preset = find_material_preset(material_name);
        self.material_preset = preset.name.to_string();
        if !process_type.is_empty() {
            self.process_type = process_type.to_string();
        }
        self.cement_gap_mm = preset.cement_gap_mm;
        self.marginal_gap_mm = preset.marginal_gap_mm;
        self.margin_ramp_width_mm = preset.margin_ramp_width_mm;
        self.smooth_distance_mm = preset.smooth_distance_mm;
        self.drill_compensation_enabled = preset.drill_compensation_enabled;
        self.drill_radius_mm = preset.drill_radius_mm;
        self.drill_compensation_offset_mm = preset.drill_compensation_offset_mm;
        self.remove_undercuts = true;

        if self.process_type.contains("Carilla") {
            self.min_thickness_mm = preset.min_thickness_veneer_mm;
            self.cement_gap_mm = self.cement_gap_mm.min(0.035);
        } else if self.process_type.contains("Cofia") {
            self.min_thickness_mm = (preset.min_thickness_mm * 0.75).max(0.40);
        } else {
            self.min_thickness_mm = preset.min_thickness_mm;
        }
        self.proximal_contact_offset_mm = preset.proximal_contact_offset_mm;
        self.occlusal_contact_offset_mm = preset.occlusal_contact_offset_mm;
        self.connector_area_mm2 = preset.connector_area_mm2;
    }

    pub fn reset_transform(&mut self) {
        self.anatomy_tx = 0.0;
        self.anatomy_ty = 0.0;
        self.anatomy_tz = 0.0;
        self.anatomy_rx = 0.0;
        self.anatomy_ry = 0.0;
        self.anatomy_rz = 0.0;
        self.anatomy_sx = 1.0;
        self.anatomy_sy = 1.0;
        self.anatomy_sz = 1.0;
        self.cusp_height_delta_mm = 0.0;
        self.fossa_depth_delta_mm = 0.0;
        self.equator_bulge_delta_mm = 0.0;
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default)]
pub struct ToothRestorationUnit {
    pub tooth_number: i32,
    pub process_type: String,
    #[serde(alias = "material_preset")]
    pub material: String,
    pub shade: String,
    pub connected_to: Vec<i32>,
    pub insertion_axis: [f32; 3],
    pub margin_control_points: Vec<[f32; 3]>,
    pub margin_points: Vec<[f32; 3]>,
    pub design_params: DesignParameters,
    pub crown_design_path: Option<String>,
}

impl Default for ToothRestorationUnit {
    fn default() -> Self {
        Self {
            tooth_number: 36,
            process_type: APEX_PROCESS_TYPES[0].to_string(),
            material: APEX_MATERIAL_PRESETS[0].name.to_string(),
            shade: "A2".to_string(),
            connected_to: Vec::new(),
            insertion_axis: [0.0, 0.0, 1.0],
            margin_control_points: Vec::new(),
            margin_points: Vec::new(),
            design_params: DesignParameters::default(),
            crown_design_path: None,
        }
    }
}

impl ToothRestorationUnit {
    pub fn from_material_preset(
        tooth_number: i32,
        process_type: &str,
        material: &str,
        shade: &str,
        connected_to: Vec<i32>,
    ) -> Self {
        let is_upper = matches!(tooth_number / 10, 1 | 2);
        let default_ax = if is_upper { [0.0, 0.0, -1.0] } else { [0.0, 0.0, 1.0] };
        let mut params = DesignParameters::default();
        params.apply_material_and_process_preset(material, process_type);
        Self {
            tooth_number,
            process_type: process_type.to_string(),
            material: params.material_preset.clone(),
            shade: shade.to_string(),
            connected_to,
            insertion_axis: default_ax,
            margin_control_points: Vec::new(),
            margin_points: Vec::new(),
            design_params: params,
            crown_design_path: None,
        }
    }

    pub fn is_pontic(&self) -> bool {
        let lo = self.process_type.to_lowercase();
        lo.contains("póntico") || lo.contains("pontico")
    }
}

fn default_one() -> f32 {
    1.0
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default)]
pub struct BridgeConnectorConfig {
    pub tooth_a: i32,
    pub tooth_b: i32,
    pub center_offset: [f32; 3],
    /// 8 radios nodales radiales (cada 45° en el plano ocluso-gingival / vestíbulo-lingual) en el centro.
    pub profile_radii_mm: [f32; 8],
    /// 8 coordenadas 2D libres `(du, dv)` en el plano central (`[0.0, 0.0]` = derivar de `profile_radii_mm`).
    #[serde(default)]
    pub profile_nodes_2d: [[f32; 2]; 8],
    pub thickness_scale: f32,
    pub end_flare: f32,
    /// Anillo de inserción en Diente A (desplazamiento 3D libre, 8 radios, 8 nodos libres 3D `(du, dv, dt)` y escala).
    pub attach_a_offset: [f32; 3],
    pub attach_a_radii_mm: [f32; 8],
    #[serde(default)]
    pub attach_a_nodes_local: [[f32; 3]; 8],
    #[serde(default = "default_one")]
    pub attach_a_scale: f32,
    /// Anillo de inserción en Diente B (desplazamiento 3D libre, 8 radios, 8 nodos libres 3D `(du, dv, dt)` y escala).
    pub attach_b_offset: [f32; 3],
    pub attach_b_radii_mm: [f32; 8],
    #[serde(default)]
    pub attach_b_nodes_local: [[f32; 3]; 8],
    #[serde(default = "default_one")]
    pub attach_b_scale: f32,
}

impl Default for BridgeConnectorConfig {
    fn default() -> Self {
        Self::for_pair_and_area(24, 25, 12.0)
    }
}

impl BridgeConnectorConfig {
    pub fn for_pair_and_area(tooth_a: i32, tooth_b: i32, target_area_mm2: f32) -> Self {
        let area = target_area_mm2.clamp(5.0, 28.0);
        // Elipse anatómica (altura ocluso-gingival = 1.15 * ancho vestíbulo-lingual)
        let ry = (area / (std::f32::consts::PI * 1.15)).sqrt();
        let rx = ry * 1.15;
        let mut radii = [1.8_f32; 8];
        for (k, r) in radii.iter_mut().enumerate() {
            let th = (k as f32) * std::f32::consts::FRAC_PI_4;
            let c = th.cos();
            let s = th.sin();
            let el_r = (rx * ry) / ((ry * c) * (ry * c) + (rx * s) * (rx * s)).sqrt().max(1e-4);
            *r = el_r.clamp(0.85, 4.5);
        }
        let mut attach_radii = radii;
        for r in &mut attach_radii {
            *r = (*r * 1.14).clamp(0.75, 5.2);
        }
        Self {
            tooth_a: tooth_a.min(tooth_b),
            tooth_b: tooth_a.max(tooth_b),
            center_offset: [0.0, 0.0, 0.0],
            profile_radii_mm: radii,
            profile_nodes_2d: [[0.0, 0.0]; 8],
            thickness_scale: 1.0,
            end_flare: 0.18,
            attach_a_offset: [0.0, 0.0, 0.0],
            attach_a_radii_mm: attach_radii,
            attach_a_nodes_local: [[0.0, 0.0, 0.0]; 8],
            attach_a_scale: 1.0,
            attach_b_offset: [0.0, 0.0, 0.0],
            attach_b_radii_mm: attach_radii,
            attach_b_nodes_local: [[0.0, 0.0, 0.0]; 8],
            attach_b_scale: 1.0,
        }
    }

    pub fn ensure_attachment_radii_initialized(&mut self) {
        let flare_mul = 1.0 + self.end_flare.clamp(0.05, 0.35);
        if self.attach_a_radii_mm.iter().any(|&r| r <= 0.05) {
            for (k, dst) in self.attach_a_radii_mm.iter_mut().enumerate() {
                *dst = (self.profile_radii_mm[k] * flare_mul).clamp(0.65, 5.2);
            }
        }
        if self.attach_b_radii_mm.iter().any(|&r| r <= 0.05) {
            for (k, dst) in self.attach_b_radii_mm.iter_mut().enumerate() {
                *dst = (self.profile_radii_mm[k] * flare_mul).clamp(0.65, 5.2);
            }
        }
        if self.attach_a_scale <= 0.01 {
            self.attach_a_scale = 1.0;
        } else {
            self.attach_a_scale = self.attach_a_scale.clamp(0.55, 1.85);
        }
        if self.attach_b_scale <= 0.01 {
            self.attach_b_scale = 1.0;
        } else {
            self.attach_b_scale = self.attach_b_scale.clamp(0.55, 1.85);
        }
    }

    pub fn ensure_free_profile_nodes_initialized(&mut self) {
        for k in 0..8 {
            let [du, dv] = self.profile_nodes_2d[k];
            if du * du + dv * dv <= 1e-4 {
                let th = (k as f32) * std::f32::consts::FRAC_PI_4;
                let r = self.profile_radii_mm[k].clamp(0.45, 5.8);
                self.profile_nodes_2d[k] = [th.cos() * r, th.sin() * r];
            }
        }
    }

    pub fn ensure_free_attach_nodes_initialized(&mut self) {
        self.ensure_attachment_radii_initialized();
        for k in 0..8 {
            let th = (k as f32) * std::f32::consts::FRAC_PI_4;
            let [ax, ay, az] = self.attach_a_nodes_local[k];
            if ax * ax + ay * ay + az * az <= 1e-4 {
                let r = self.attach_a_radii_mm[k].clamp(0.45, 5.8);
                self.attach_a_nodes_local[k] = [th.cos() * r, th.sin() * r, 0.0];
            }
            let [bx, by, bz] = self.attach_b_nodes_local[k];
            if bx * bx + by * by + bz * bz <= 1e-4 {
                let r = self.attach_b_radii_mm[k].clamp(0.45, 5.8);
                self.attach_b_nodes_local[k] = [th.cos() * r, th.sin() * r, 0.0];
            }
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default)]
pub struct DentalCase {
    pub case_id: String,
    pub patient_name: String,
    pub doctor_name: String,
    pub tooth_number: i32,
    pub restoration_type: String,
    pub shade: String,
    pub notes: String,
    pub created_at: String,
    pub updated_at: String,
    pub scans_raw: BTreeMap<String, Option<String>>,
    pub scans_working: BTreeMap<String, Option<String>>,
    pub prep_arch: String,
    pub insertion_axis: [f32; 3],
    pub margin_control_points: Vec<[f32; 3]>,
    pub margin_points: Vec<[f32; 3]>,
    pub design_params: DesignParameters,
    pub crown_design_path: Option<String>,
    pub last_export_path: Option<String>,
    pub restorations: BTreeMap<i32, ToothRestorationUnit>,
    pub bridge_connectors: Vec<BridgeConnectorConfig>,
}

impl Default for DentalCase {
    fn default() -> Self {
        let now = Local::now().format("%Y-%m-%dT%H:%M:%S").to_string();
        let mut scans = BTreeMap::new();
        scans.insert("maxilla".to_string(), None);
        scans.insert("mandible".to_string(), None);
        scans.insert("bite".to_string(), None);
        Self {
            case_id: String::new(),
            patient_name: String::new(),
            doctor_name: String::new(),
            tooth_number: 36,
            restoration_type: "Corona Individual — Zirconia Monolítica".to_string(),
            shade: "A2".to_string(),
            notes: String::new(),
            created_at: now.clone(),
            updated_at: now,
            scans_raw: scans.clone(),
            scans_working: scans,
            prep_arch: "mandible".to_string(),
            insertion_axis: [0.0, 0.0, 1.0],
            margin_control_points: Vec::new(),
            margin_points: Vec::new(),
            design_params: DesignParameters::default(),
            crown_design_path: None,
            last_export_path: None,
            restorations: BTreeMap::new(),
            bridge_connectors: Vec::new(),
        }
    }
}

impl DentalCase {
    pub fn ensure_restorations_initialized(&mut self) {
        if self.restorations.is_empty() {
            let mut unit = ToothRestorationUnit::from_material_preset(
                self.tooth_number,
                &self.design_params.process_type,
                &self.design_params.material_preset,
                &self.shade,
                Vec::new(),
            );
            unit.insertion_axis = self.insertion_axis;
            unit.margin_control_points = self.margin_control_points.clone();
            unit.margin_points = self.margin_points.clone();
            unit.design_params = self.design_params.clone();
            unit.crown_design_path = self.crown_design_path.clone();
            self.restorations.insert(self.tooth_number, unit);
        } else if !self.restorations.contains_key(&self.tooth_number) {
            if let Some((&first_fdi, u0)) = self.restorations.iter().next() {
                let u0 = u0.clone();
                self.tooth_number = first_fdi;
                self.shade = u0.shade;
                let is_upper = matches!(first_fdi / 10, 1 | 2);
                self.prep_arch = if is_upper { "maxilla" } else { "mandible" }.to_string();
                self.insertion_axis = u0.insertion_axis;
                self.margin_control_points = u0.margin_control_points;
                self.margin_points = u0.margin_points;
                self.design_params = u0.design_params;
                self.crown_design_path = u0.crown_design_path;
            }
        }
        self.sync_bridge_connectors();
    }

    /// Devuelve todos los pares conectados únicos `(fdi_menor, fdi_mayor)` definidos en el odontograma.
    pub fn connected_pairs(&self) -> Vec<(i32, i32)> {
        let mut pairs = BTreeSet::new();
        for (&fdi, unit) in &self.restorations {
            for &other in &unit.connected_to {
                if other != fdi && self.restorations.contains_key(&other) {
                    pairs.insert((fdi.min(other), fdi.max(other)));
                }
            }
        }
        pairs.into_iter().collect()
    }

    /// Agrupa las piezas en componentes conexas de puente (tamaño >= 2) y piezas individuales (tamaño == 1).
    pub fn connected_bridge_groups(&self) -> Vec<Vec<i32>> {
        let pairs = self.connected_pairs();
        let mut adj: BTreeMap<i32, Vec<i32>> = BTreeMap::new();
        for &fdi in self.restorations.keys() {
            adj.entry(fdi).or_default();
        }
        for (a, b) in pairs {
            adj.entry(a).or_default().push(b);
            adj.entry(b).or_default().push(a);
        }
        let mut visited = BTreeSet::new();
        let mut groups = Vec::new();
        for &start in self.restorations.keys() {
            if visited.contains(&start) {
                continue;
            }
            let mut grp = Vec::new();
            let mut q = VecDeque::new();
            visited.insert(start);
            q.push_back(start);
            while let Some(u) = q.pop_front() {
                grp.push(u);
                if let Some(neighs) = adj.get(&u) {
                    for &v in neighs {
                        if visited.insert(v) {
                            q.push_back(v);
                        }
                    }
                }
            }
            grp.sort_unstable();
            groups.push(grp);
        }
        groups
    }

    pub fn has_bridge(&self) -> bool {
        self.connected_bridge_groups().iter().any(|g| g.len() >= 2)
    }

    /// Sincroniza la lista de configuraciones de conectores 3D con los enlaces `●` del odontograma.
    pub fn sync_bridge_connectors(&mut self) {
        let active_pairs = self.connected_pairs();
        let pair_set: BTreeSet<(i32, i32)> = active_pairs.iter().copied().collect();
        self.bridge_connectors.retain(|c| {
            let k = (c.tooth_a.min(c.tooth_b), c.tooth_a.max(c.tooth_b));
            pair_set.contains(&k)
        });
        for (a, b) in active_pairs {
            let exists = self
                .bridge_connectors
                .iter()
                .any(|c| c.tooth_a.min(c.tooth_b) == a && c.tooth_a.max(c.tooth_b) == b);
            if !exists {
                let area_a = self
                    .restorations
                    .get(&a)
                    .map(|u| u.design_params.connector_area_mm2)
                    .unwrap_or(12.0);
                let area_b = self
                    .restorations
                    .get(&b)
                    .map(|u| u.design_params.connector_area_mm2)
                    .unwrap_or(12.0);
                self.bridge_connectors
                    .push(BridgeConnectorConfig::for_pair_and_area(a, b, area_a.max(area_b)));
            }
        }
        for c in &mut self.bridge_connectors {
            c.ensure_attachment_radii_initialized();
        }
        self.bridge_connectors.sort_by_key(|c| (c.tooth_a, c.tooth_b));
    }

    pub fn sync_active_to_unit(&mut self) {
        if let Some(u) = self.restorations.get_mut(&self.tooth_number) {
            u.insertion_axis = self.insertion_axis;
            u.margin_control_points = self.margin_control_points.clone();
            u.margin_points = self.margin_points.clone();
            u.design_params = self.design_params.clone();
            if self.crown_design_path.is_some() {
                u.crown_design_path = self.crown_design_path.clone();
            }
        }
    }

    pub fn set_active_tooth(&mut self, fdi: i32) {
        self.ensure_restorations_initialized();
        self.sync_active_to_unit();
        if !self.restorations.contains_key(&fdi) {
            self.restorations.insert(
                fdi,
                ToothRestorationUnit::from_material_preset(
                    fdi,
                    "Corona Individual",
                    APEX_MATERIAL_PRESETS[0].name,
                    &self.shade,
                    Vec::new(),
                ),
            );
        }
        if let Some(u) = self.restorations.get(&fdi).cloned() {
            self.tooth_number = u.tooth_number;
            self.shade = u.shade;
            let is_upper = matches!(fdi / 10, 1 | 2);
            self.prep_arch = if is_upper { "maxilla" } else { "mandible" }.to_string();
            self.insertion_axis = u.insertion_axis;
            self.margin_control_points = u.margin_control_points;
            self.margin_points = u.margin_points;
            self.design_params = u.design_params;
            self.crown_design_path = u.crown_design_path;
        }
    }

    pub fn get_active_unit(&self) -> Option<&ToothRestorationUnit> {
        self.restorations.get(&self.tooth_number)
    }

    pub fn get_antagonist_arch(&self) -> &'static str {
        if self.prep_arch == "mandible" {
            "maxilla"
        } else {
            "mandible"
        }
    }

    pub fn format_order_summary(&self) -> String {
        if self.restorations.is_empty() {
            return format!("Pieza #{} ({})", self.tooth_number, self.restoration_type);
        }
        if self.restorations.len() == 1 {
            if let Some(u) = self.restorations.values().next() {
                let mat_short = u.material.split('(').next().unwrap_or(&u.material).trim();
                return format!("#{} {} ({})", u.tooth_number, u.process_type, mat_short);
            }
        }
        self.restorations
            .values()
            .map(|u| {
                let p_short = u.process_type.split('(').next().unwrap_or(&u.process_type).trim();
                format!("#{}:{}", u.tooth_number, p_short)
            })
            .collect::<Vec<_>>()
            .join(" | ")
    }
}

/// Detecta si el ejecutable corre en **Modo Portable USB** comprobando la presencia de
/// `portable.flag` (o `PORTABLE.txt`) junto al ejecutable `ApexDentalCraft.exe`.
/// Si existe, devuelve la carpeta raíz del ejecutable en la memoria USB para guardar allí
/// todos los casos (`Casos/`) y librerías (`Librerias/`) sin escribir nada en `C:/`.
pub fn detect_portable_root() -> Option<PathBuf> {
    let exe = std::env::current_exe().ok()?;
    let dir = exe.parent()?.to_path_buf();
    if dir.join("portable.flag").exists() || dir.join("PORTABLE.txt").exists() {
        Some(dir)
    } else {
        None
    }
}

pub struct CaseManager {
    pub root_dir: PathBuf,
    pub is_portable: bool,
}

impl CaseManager {
    pub fn new(custom_root: Option<PathBuf>) -> Self {
        let mut is_portable = false;
        let preferred = if let Some(custom) = custom_root {
            custom
        } else if let Some(portable_root) = detect_portable_root() {
            is_portable = true;
            portable_root.join("Casos")
        } else if cfg!(windows) {
            PathBuf::from("C:/LabCAD/Casos")
        } else {
            let home = std::env::var("HOME").unwrap_or_else(|_| ".".to_string());
            PathBuf::from(home).join("LabCAD").join("Casos")
        };

        let root_dir = if fs::create_dir_all(&preferred).is_ok() {
            preferred
        } else {
            let fallback = std::env::current_dir()
                .unwrap_or_else(|_| PathBuf::from("."))
                .join("LabCAD_Local")
                .join("Casos");
            let _ = fs::create_dir_all(&fallback);
            fallback
        };

        Self {
            root_dir,
            is_portable,
        }
    }

    pub fn get_case_dir(&self, case_id: &str) -> PathBuf {
        self.root_dir.join(case_id)
    }

    pub fn create_case(
        &self,
        patient_name: &str,
        doctor_name: &str,
        tooth_number: i32,
        restoration_type: &str,
        shade: &str,
        notes: &str,
        restorations: BTreeMap<i32, ToothRestorationUnit>,
    ) -> Result<DentalCase, String> {
        let timestamp = Local::now().format("%Y%m%d_%H%M%S").to_string();
        let safe_name: String = patient_name
            .trim()
            .chars()
            .map(|c| if c.is_alphanumeric() { c } else { '_' })
            .collect::<String>()
            .trim_matches('_')
            .to_string();
        let safe_name = if safe_name.is_empty() {
            "Paciente".to_string()
        } else {
            safe_name
        };

        let case_id = format!("{}_{}_FDI{}", timestamp, safe_name, tooth_number);
        let case_dir = self.get_case_dir(&case_id);
        for sub in ["raw", "working", "design", "exports"] {
            fs::create_dir_all(case_dir.join(sub)).map_err(|e| e.to_string())?;
        }

        let is_upper = matches!(tooth_number / 10, 1 | 2);
        let mut params = DesignParameters::default();
        params.apply_material_and_process_preset(restoration_type, "Corona Individual");

        let mut dental_case = DentalCase {
            case_id,
            patient_name: patient_name.trim().to_string(),
            doctor_name: doctor_name.trim().to_string(),
            tooth_number,
            restoration_type: restoration_type.to_string(),
            shade: shade.to_string(),
            notes: notes.trim().to_string(),
            prep_arch: if is_upper { "maxilla" } else { "mandible" }.to_string(),
            insertion_axis: if is_upper { [0.0, 0.0, -1.0] } else { [0.0, 0.0, 1.0] },
            design_params: params,
            restorations,
            ..Default::default()
        };
        dental_case.ensure_restorations_initialized();
        dental_case.set_active_tooth(tooth_number);
        self.save_case(&mut dental_case)?;
        Ok(dental_case)
    }

    pub fn save_case(&self, dental_case: &mut DentalCase) -> Result<(), String> {
        dental_case.ensure_restorations_initialized();
        dental_case.sync_active_to_unit();
        dental_case.updated_at = Local::now().format("%Y-%m-%dT%H:%M:%S").to_string();

        let case_dir = self.get_case_dir(&dental_case.case_id);
        for sub in ["raw", "working", "design", "exports"] {
            fs::create_dir_all(case_dir.join(sub)).map_err(|e| e.to_string())?;
        }

        let meta_path = case_dir.join("case_metadata.json");
        let text = serde_json::to_string_pretty(dental_case).map_err(|e| e.to_string())?;
        fs::write(meta_path, text).map_err(|e| e.to_string())
    }

    pub fn load_case(&self, case_id: &str) -> Result<DentalCase, String> {
        let meta_path = self.get_case_dir(case_id).join("case_metadata.json");
        let content = fs::read_to_string(&meta_path)
            .map_err(|e| format!("No se pudo leer {:?}: {}", meta_path, e))?;
        let mut case: DentalCase = serde_json::from_str(&content)
            .map_err(|e| format!("JSON inválido en {:?}: {}", meta_path, e))?;
        case.ensure_restorations_initialized();
        Ok(case)
    }

    pub fn delete_case(&self, case_id: &str) -> Result<(), String> {
        let dir = self.get_case_dir(case_id);
        if dir.exists() {
            fs::remove_dir_all(dir).map_err(|e| e.to_string())?;
        }
        Ok(())
    }

    pub fn list_cases(&self) -> Vec<DentalCase> {
        let mut cases = Vec::new();
        if let Ok(entries) = fs::read_dir(&self.root_dir) {
            for entry in entries.flatten() {
                let p = entry.path();
                if p.is_dir() && p.join("case_metadata.json").exists() {
                    if let Some(name) = p.file_name().and_then(|n| n.to_str()) {
                        if let Ok(c) = self.load_case(name) {
                            cases.push(c);
                        }
                    }
                }
            }
        }
        cases.sort_by(|a, b| b.updated_at.cmp(&a.updated_at));
        cases
    }

    pub fn import_scan(
        &self,
        dental_case: &mut DentalCase,
        scan_type: &str,
        source_path: &Path,
    ) -> Result<PathBuf, String> {
        if !matches!(scan_type, "maxilla" | "mandible" | "bite") {
            return Err(format!("Tipo de escaneo inválido: {}", scan_type));
        }
        if !source_path.exists() {
            return Err(format!("El archivo no existe: {:?}", source_path));
        }
        let ext = source_path
            .extension()
            .and_then(|s| s.to_str())
            .unwrap_or("stl")
            .to_lowercase();
        if !matches!(ext.as_str(), "stl" | "obj" | "ply") {
            return Err(format!("Formato .{} no soportado (use STL, OBJ o PLY).", ext));
        }

        let case_dir = self.get_case_dir(&dental_case.case_id);
        let raw_rel = format!("raw/{}.{}", scan_type, ext);
        let work_rel = format!("working/{}_work.{}", scan_type, ext);

        fs::copy(source_path, case_dir.join(&raw_rel)).map_err(|e| e.to_string())?;
        fs::copy(source_path, case_dir.join(&work_rel)).map_err(|e| e.to_string())?;

        dental_case.scans_raw.insert(scan_type.to_string(), Some(raw_rel));
        dental_case.scans_working.insert(scan_type.to_string(), Some(work_rel.clone()));
        self.save_case(dental_case)?;
        Ok(case_dir.join(work_rel))
    }

    pub fn get_active_scan_path(&self, dental_case: &DentalCase, scan_type: &str) -> Option<PathBuf> {
        let case_dir = self.get_case_dir(&dental_case.case_id);
        if let Some(Some(rel)) = dental_case.scans_working.get(scan_type) {
            let p = case_dir.join(rel);
            if p.exists() {
                return Some(p);
            }
        }
        if let Some(Some(rel)) = dental_case.scans_raw.get(scan_type) {
            let p = case_dir.join(rel);
            if p.exists() {
                return Some(p);
            }
        }
        None
    }

    pub fn get_crown_design_path(&self, dental_case: &DentalCase, fdi: Option<i32>) -> PathBuf {
        let case_dir = self.get_case_dir(&dental_case.case_id);
        let target_fdi = fdi.unwrap_or(dental_case.tooth_number);
        if let Some(unit) = dental_case.restorations.get(&target_fdi) {
            if let Some(rel) = &unit.crown_design_path {
                let p = case_dir.join(rel);
                if p.exists() {
                    return p;
                }
            }
        }
        case_dir.join("design").join("crown_designed.stl")
    }

    pub fn reset_working_scan_to_raw(
        &self,
        dental_case: &mut DentalCase,
        scan_type: &str,
    ) -> Option<PathBuf> {
        let case_dir = self.get_case_dir(&dental_case.case_id);
        let rel_raw = dental_case.scans_raw.get(scan_type)?.as_ref()?;
        let raw_path = case_dir.join(rel_raw);
        if !raw_path.exists() {
            return None;
        }
        let ext = raw_path
            .extension()
            .and_then(|s| s.to_str())
            .unwrap_or("stl")
            .to_lowercase();
        let work_rel = format!("working/{}_work.{}", scan_type, ext);
        let work_path = case_dir.join(&work_rel);
        fs::copy(&raw_path, &work_path).ok()?;
        dental_case
            .scans_working
            .insert(scan_type.to_string(), Some(work_rel));
        let _ = self.save_case(dental_case);
        Some(work_path)
    }
}
