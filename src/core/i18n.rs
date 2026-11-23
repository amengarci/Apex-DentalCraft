//! Internacionalización (**ES / EN / PT**) con terminología dental CAD/CAM clínica exacta.
//!
//! La preferencia de idioma (`AppLanguage`) se guarda en `settings.json` sin alterar los valores
//! canónicos internos en español persistidos en `case_metadata.json` (`process_type`, `material`,
//! `prep_arch`), preservando 100% la compatibilidad retroactiva con todos los casos existentes.

use serde::{Deserialize, Serialize};
use std::sync::atomic::{AtomicU8, Ordering};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
pub enum AppLanguage {
    Es,
    #[default]
    En,
    Pt,
}

impl AppLanguage {
    pub const ALL: [AppLanguage; 3] = [AppLanguage::En, AppLanguage::Es, AppLanguage::Pt];

    pub fn from_u8(v: u8) -> Self {
        match v {
            1 => AppLanguage::En,
            2 => AppLanguage::Pt,
            _ => AppLanguage::Es,
        }
    }

    pub fn as_u8(self) -> u8 {
        match self {
            AppLanguage::Es => 0,
            AppLanguage::En => 1,
            AppLanguage::Pt => 2,
        }
    }

    pub fn code(self) -> &'static str {
        match self {
            AppLanguage::Es => "ES",
            AppLanguage::En => "EN",
            AppLanguage::Pt => "PT",
        }
    }

    pub fn label(self) -> &'static str {
        match self {
            AppLanguage::Es => "Español",
            AppLanguage::En => "English",
            AppLanguage::Pt => "Português",
        }
    }
}

// Default active language is English (AppLanguage::En = 1)
static ACTIVE_LANG: AtomicU8 = AtomicU8::new(1);

pub fn set_language(lang: AppLanguage) {
    ACTIVE_LANG.store(lang.as_u8(), Ordering::Relaxed);
}

pub fn current_language() -> AppLanguage {
    AppLanguage::from_u8(ACTIVE_LANG.load(Ordering::Relaxed))
}

#[inline]
pub fn tr_for(lang: AppLanguage, es: &'static str, en: &'static str, pt: &'static str) -> &'static str {
    match lang {
        AppLanguage::Es => es,
        AppLanguage::En => en,
        AppLanguage::Pt => pt,
    }
}

#[inline]
pub fn tr(es: &'static str, en: &'static str, pt: &'static str) -> &'static str {
    tr_for(current_language(), es, en, pt)
}

pub fn tr_process_type_for(lang: AppLanguage, raw: &str) -> String {
    let translated = if raw.contains("Corona Individual") {
        Some(tr_for(lang, "Corona Individual", "Single Crown", "Coroa Unitária"))
    } else if raw.contains("Pilar de Puente") {
        Some(tr_for(lang, "Pilar de Puente", "Bridge Abutment", "Pilar de Ponte"))
    } else if raw.contains("Póntico") || raw.contains("Pontico") {
        Some(tr_for(lang, "Póntico de Puente", "Bridge Pontic", "Pôntico de Ponte"))
    } else if raw.contains("Carilla") {
        Some(tr_for(lang, "Carilla Estética (Veneer)", "Veneer", "Faceta / Lente de Contato"))
    } else if raw.contains("Incrustación") {
        Some(tr_for(lang, "Incrustación (Inlay / Onlay)", "Inlay / Onlay", "Inlay / Onlay"))
    } else if raw.contains("Cofia") {
        Some(tr_for(lang, "Cofia / Estructura", "Coping / Framework", "Coping / Infraestrutura"))
    } else {
        None
    };
    translated.map(|s| s.to_string()).unwrap_or_else(|| raw.to_string())
}

pub fn tr_process_type(raw: &str) -> String {
    tr_process_type_for(current_language(), raw)
}

pub fn tr_material_name_for(lang: AppLanguage, raw: &str) -> String {
    let translated = if raw.contains("Monolítica") {
        Some(tr_for(
            lang,
            "Zirconia Monolítica (3Y-TZP Alta Resistencia)",
            "Monolithic Zirconia (High-Strength 3Y-TZP)",
            "Zircônia Monolítica (3Y-TZP Alta Resistência)",
        ))
    } else if raw.contains("Multicapa") || raw.contains("Translúcida") {
        Some(tr_for(
            lang,
            "Zirconia Translúcida Multicapa (4Y/5Y-PSZ Estética)",
            "Multilayer Translucent Zirconia (4Y/5Y-PSZ)",
            "Zircônia Translúcida Multicamadas (4Y/5Y-PSZ Estética)",
        ))
    } else if raw.contains("e.max") {
        Some(tr_for(
            lang,
            "e.max (Disilicato de Litio IPS e.max CAD/Press)",
            "e.max (IPS e.max CAD/Press Lithium Disilicate)",
            "e.max (Dissilicato de Lítio IPS e.max CAD/Press)",
        ))
    } else if raw.contains("PMMA") {
        Some(tr_for(
            lang,
            "PMMA Fresado (Provisional Larga Duración)",
            "Milled PMMA (Long-Term Provisional)",
            "PMMA Fresado (Provisório de Longa Duração)",
        ))
    } else if raw.contains("Resina Impresa") {
        Some(tr_for(
            lang,
            "Resina Impresa 3D (Provisional / Híbrida)",
            "3D Printed Resin (Provisional / Hybrid)",
            "Resina Impressa 3D (Provisório / Híbrida)",
        ))
    } else if raw.contains("Cera") || raw.contains("Castable") {
        Some(tr_for(
            lang,
            "Cera / Resina Castable (Calcinable para Prensado)",
            "Castable Wax / Resin (Pressing / Casting)",
            "Cera / Resina Calcinável (Para Injeção / Fundição)",
        ))
    } else if raw.contains("CoCr") || raw.contains("Aleación") {
        Some(tr_for(
            lang,
            "CoCr / Aleación Metálica (Estructura / Cofia)",
            "CoCr / Metal Alloy (Framework / Coping)",
            "CoCr / Liga Metálica (Infraestrutura / Coping)",
        ))
    } else {
        None
    };
    translated.map(|s| s.to_string()).unwrap_or_else(|| raw.to_string())
}

pub fn tr_material_name(raw: &str) -> String {
    tr_material_name_for(current_language(), raw)
}

pub fn tr_material_desc_for(lang: AppLanguage, material_name: &str, raw_desc: &str) -> String {
    let translated = if material_name.contains("Monolítica") {
        Some(tr_for(
            lang,
            "1100-1200 MPa | Espesor mín. 0.60 mm | Cemento 45 µm | Fresa R=0.55 mm | Conector 9 mm²",
            "1100-1200 MPa | Min. wall 0.60 mm | Cement 45 µm | Bur R=0.55 mm | Connector 9 mm²",
            "1100-1200 MPa | Espessura mín. 0.60 mm | Cimento 45 µm | Broca R=0.55 mm | Conector 9 mm²",
        ))
    } else if material_name.contains("Multicapa") || material_name.contains("Translúcida") {
        Some(tr_for(
            lang,
            "650-850 MPa | Espesor mín. 0.80 mm | Cemento 50 µm | Fresa R=0.55 mm | Conector 12 mm²",
            "650-850 MPa | Min. wall 0.80 mm | Cement 50 µm | Bur R=0.55 mm | Connector 12 mm²",
            "650-850 MPa | Espessura mín. 0.80 mm | Cimento 50 µm | Broca R=0.55 mm | Conector 12 mm²",
        ))
    } else if material_name.contains("e.max") {
        Some(tr_for(
            lang,
            "400-530 MPa (Adhesivo) | Espesor 1.00 mm (0.45 Carilla) | Cemento 60 µm | Fresa R=0.60 mm",
            "400-530 MPa (Adhesive) | Wall 1.00 mm (0.45 Veneer) | Cement 60 µm | Bur R=0.60 mm",
            "400-530 MPa (Adesivo) | Espessura 1.00 mm (0.45 Faceta) | Cimento 60 µm | Broca R=0.60 mm",
        ))
    } else if material_name.contains("PMMA") {
        Some(tr_for(
            lang,
            "PMMA CAD/CAM | Espesor mín. 0.80 mm | Cemento 65 µm | Fresa R=0.55 mm | Conector 12 mm²",
            "PMMA CAD/CAM | Min. wall 0.80 mm | Cement 65 µm | Bur R=0.55 mm | Connector 12 mm²",
            "PMMA CAD/CAM | Espessura mín. 0.80 mm | Cimento 65 µm | Broca R=0.55 mm | Conector 12 mm²",
        ))
    } else if material_name.contains("Resina Impresa") {
        Some(tr_for(
            lang,
            "Compensación DLP/MSLA | Espesor mín. 1.00 mm | Cemento 80 µm | Sin fresado | Conector 14 mm²",
            "DLP/MSLA Comp. | Min. wall 1.00 mm | Cement 80 µm | No bur comp. | Connector 14 mm²",
            "Compensação DLP/MSLA | Espessura mín. 1.00 mm | Cimento 80 µm | Sem fresagem | Conector 14 mm²",
        ))
    } else if material_name.contains("Cera") || material_name.contains("Castable") {
        Some(tr_for(
            lang,
            "Patrón Calcinable | Espesor mín. 0.65 mm | Cemento 40 µm | Fresa R=0.50 mm",
            "Castable Pattern | Min. wall 0.65 mm | Cement 40 µm | Bur R=0.50 mm",
            "Padrão Calcinável | Espessura mín. 0.65 mm | Cimento 40 µm | Broca R=0.50 mm",
        ))
    } else if material_name.contains("CoCr") || material_name.contains("Aleación") {
        Some(tr_for(
            lang,
            "Alta Rigidez | Espesor mín. 0.40 mm | Cemento 40 µm | Fresa R=0.60 mm | Conector 7 mm²",
            "High Stiffness | Min. wall 0.40 mm | Cement 40 µm | Bur R=0.60 mm | Connector 7 mm²",
            "Alta Rigidez | Espessura mín. 0.40 mm | Cimento 40 µm | Broca R=0.60 mm | Conector 7 mm²",
        ))
    } else {
        None
    };
    translated
        .map(|s| s.to_string())
        .unwrap_or_else(|| raw_desc.to_string())
}

pub fn tr_material_desc(material_name: &str, raw_desc: &str) -> String {
    tr_material_desc_for(current_language(), material_name, raw_desc)
}

pub fn tr_fdi_tooth_name_for(lang: AppLanguage, fdi: i32) -> &'static str {
    let unit = fdi.abs() % 10;
    match unit {
        1 => tr_for(lang, "Incisivo Central", "Central Incisor", "Incisivo Central"),
        2 => tr_for(lang, "Incisivo Lateral", "Lateral Incisor", "Incisivo Lateral"),
        3 => tr_for(lang, "Canino", "Canine", "Canino"),
        4 => tr_for(lang, "1er Premolar", "First Premolar", "1º Pré-Molar"),
        5 => tr_for(lang, "2do Premolar", "Second Premolar", "2º Pré-Molar"),
        6 => tr_for(lang, "1er Molar", "First Molar", "1º Molar"),
        7 => tr_for(lang, "2do Molar", "Second Molar", "2º Molar"),
        8 => tr_for(lang, "3er Molar", "Third Molar", "3º Molar"),
        _ => tr_for(lang, "Pieza Dental", "Dental Unit", "Dente"),
    }
}

pub fn tr_fdi_tooth_name(fdi: i32) -> &'static str {
    tr_fdi_tooth_name_for(current_language(), fdi)
}

pub fn tr_arch_name_for(lang: AppLanguage, arch: &str) -> &'static str {
    if arch.eq_ignore_ascii_case("maxilla") {
        tr_for(lang, "Maxilar Superior", "Upper Arch (Maxilla)", "Arcada Superior (Maxila)")
    } else {
        tr_for(lang, "Mandíbula", "Lower Arch (Mandible)", "Arcada Inferior (Mandíbula)")
    }
}

pub fn tr_arch_name(arch: &str) -> &'static str {
    tr_arch_name_for(current_language(), arch)
}
