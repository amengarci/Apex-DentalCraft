//! Gestor de Librerías Dentales (`ToothLibraryManager`) — Apex DentalCraft
//!
//! Descubre automáticamente las librerías anatómicas en:
//! - `../libraries/` (librerías del repositorio principal, ej. `BodyParts3D_Humana_FDI`, `Apex_Esculpida_FDI`)
//! - `./libraries/`
//! - `C:/LabCAD/Librerias/` (librerías del usuario)
//! - `~/Downloads/Dientes 1` (si está disponible)
//! Permite importar nuevas carpetas de dientes (.STL/.OBJ/.PLY) con un clic y genera
//! coronas anatómicas HD procedurales FDI (11–48) como respaldo garantizado 100% offline.

use crate::core::case_manager::detect_portable_root;
use crate::geometry::mesh::TriMeshData;
use glam::Vec3;
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, HashSet};
use std::f32::consts::PI;
use std::fs;
use std::path::{Path, PathBuf};

pub const ALL_FDI_TEETH: &[i32] = &[
    18, 17, 16, 15, 14, 13, 12, 11, 21, 22, 23, 24, 25, 26, 27, 28,
    48, 47, 46, 45, 44, 43, 42, 41, 31, 32, 33, 34, 35, 36, 37, 38,
];

fn universal_to_fdi(num: i32) -> Option<i32> {
    match num {
        1 => Some(18), 2 => Some(17), 3 => Some(16), 4 => Some(15),
        5 => Some(14), 6 => Some(13), 7 => Some(12), 8 => Some(11),
        9 => Some(21), 10 => Some(22), 11 => Some(23), 12 => Some(24),
        13 => Some(25), 14 => Some(26), 15 => Some(27), 16 => Some(28),
        17 => Some(38), 18 => Some(37), 19 => Some(36), 20 => Some(35),
        21 => Some(34), 22 => Some(33), 23 => Some(32), 24 => Some(31),
        25 => Some(41), 26 => Some(42), 27 => Some(43), 28 => Some(44),
        29 => Some(45), 30 => Some(46), 31 => Some(47), 32 => Some(48),
        _ => None,
    }
}

fn is_valid_fdi(n: i32) -> bool {
    ALL_FDI_TEETH.contains(&n)
}

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
struct LibraryMetaJson {
    name: Option<String>,
    license: Option<String>,
    description: Option<String>,
}

#[derive(Debug, Clone)]
#[allow(dead_code)]
pub struct DentalLibraryInfo {
    pub library_id: String,
    pub display_name: String,
    pub folder_path: PathBuf,
    pub license_info: String,
    pub description: String,
    pub fdi_map: BTreeMap<i32, PathBuf>,
    pub all_files: Vec<PathBuf>,
}

pub struct ToothLibraryManager {
    pub user_lib_dir: PathBuf,
    pub bundled_dirs: Vec<PathBuf>,
    pub libraries: BTreeMap<String, DentalLibraryInfo>,
}

impl ToothLibraryManager {
    pub fn new() -> Self {
        let portable_root = detect_portable_root();

        let preferred_user = if let Some(ref usb_dir) = portable_root {
            usb_dir.join("Librerias")
        } else if cfg!(windows) {
            PathBuf::from("C:/LabCAD/Librerias")
        } else {
            let home = std::env::var("HOME").unwrap_or_else(|_| ".".to_string());
            PathBuf::from(home).join("LabCAD").join("Librerias")
        };

        let user_lib_dir = if fs::create_dir_all(&preferred_user).is_ok() {
            preferred_user
        } else {
            let fb = std::env::current_dir()
                .unwrap_or_else(|_| PathBuf::from("."))
                .join("LabCAD_Local")
                .join("Librerias");
            let _ = fs::create_dir_all(&fb);
            fb
        };

        let mut bundled_dirs = Vec::new();
        if let Some(ref usb_dir) = portable_root {
            bundled_dirs.push(usb_dir.join("libraries"));
        } else {
            if let Ok(cwd) = std::env::current_dir() {
                bundled_dirs.push(cwd.join("libraries"));
                if let Some(parent) = cwd.parent() {
                    bundled_dirs.push(parent.join("libraries"));
                }
            }
            if let Ok(exe) = std::env::current_exe() {
                if let Some(dir) = exe.parent() {
                    bundled_dirs.push(dir.join("libraries"));
                    if let Some(p1) = dir.parent() {
                        bundled_dirs.push(p1.join("libraries"));
                        if let Some(p2) = p1.parent() {
                            bundled_dirs.push(p2.join("libraries"));
                            if let Some(p3) = p2.parent() {
                                bundled_dirs.push(p3.join("libraries"));
                            }
                        }
                    }
                }
            }
        }

        let mut mgr = Self {
            user_lib_dir,
            bundled_dirs,
            libraries: BTreeMap::new(),
        };
        mgr.refresh_libraries();
        mgr
    }

    /// Busca los escaneos del caso demo real (`demo_mandible_prep36.stl` / `demo_maxilla_antagonist.stl`).
    pub fn find_demo_asset(&self, filename: &str) -> Option<PathBuf> {
        for root in &self.bundled_dirs {
            let candidate = root.join("_demo_assets").join(filename);
            if candidate.exists() {
                return Some(candidate);
            }
        }
        None
    }

    pub fn refresh_libraries(&mut self) -> Vec<DentalLibraryInfo> {
        self.libraries.clear();

        // 1. Detectar ~/Downloads/Dientes 1 automáticamente
        let home_dir = std::env::var("USERPROFILE")
            .or_else(|_| std::env::var("HOME"))
            .ok()
            .map(PathBuf::from);
        if let Some(home) = home_dir {
            let d1 = home.join("Downloads").join("Dientes 1");
            if d1.is_dir() {
                self.index_single_library_folder(&d1, "downloads_");
            }
        }

        // 2. Librerías incluidas y del usuario
        let mut roots = self.bundled_dirs.clone();
        roots.push(self.user_lib_dir.clone());
        let mut visited = HashSet::new();

        for root in roots {
            if !root.is_dir() {
                continue;
            }
            let canon = fs::canonicalize(&root).unwrap_or(root.clone());
            if !visited.insert(canon) {
                continue;
            }
            if let Ok(entries) = fs::read_dir(&root) {
                let mut subs: Vec<PathBuf> = entries
                    .flatten()
                    .map(|e| e.path())
                    .filter(|p| {
                        p.is_dir()
                            && !p
                                .file_name()
                                .and_then(|s| s.to_str())
                                .unwrap_or("")
                                .starts_with('_')
                    })
                    .collect();
                subs.sort();
                for sub in subs {
                    self.index_single_library_folder(&sub, "");
                }
            }
        }

        // 3. Librería procedural garantizada siempre disponible en memoria
        if !self.libraries.contains_key("Apex_Esculpida_FDI") {
            self.libraries.insert(
                "Apex_Esculpida_FDI".to_string(),
                DentalLibraryInfo {
                    library_id: "Apex_Esculpida_FDI".to_string(),
                    display_name: "Apex Esculpida FDI (Procedural HD)".to_string(),
                    folder_path: self.user_lib_dir.clone(),
                    license_info: "100% Nativa Open-Source (Apex DentalCraft)".to_string(),
                    description: "32 piezas FDI anatómicas HD generadas nativamente en Rust.".to_string(),
                    fdi_map: ALL_FDI_TEETH.iter().map(|&t| (t, PathBuf::new())).collect(),
                    all_files: Vec::new(),
                },
            );
        }

        self.list_libraries()
    }

    fn collect_mesh_files_recursive(dir: &Path, out: &mut Vec<PathBuf>) {
        if let Ok(entries) = fs::read_dir(dir) {
            for entry in entries.flatten() {
                let p = entry.path();
                if p.is_dir() {
                    Self::collect_mesh_files_recursive(&p, out);
                } else if p.is_file() {
                    if let Some(ext) = p.extension().and_then(|s| s.to_str()) {
                        let low = ext.to_lowercase();
                        if matches!(low.as_str(), "stl" | "obj" | "ply") {
                            out.push(p);
                        }
                    }
                }
            }
        }
    }

    fn index_single_library_folder(&mut self, folder: &Path, prefix: &str) {
        let mut mesh_files = Vec::new();
        Self::collect_mesh_files_recursive(folder, &mut mesh_files);
        mesh_files.sort();
        if mesh_files.is_empty() {
            return;
        }

        let folder_name = folder
            .file_name()
            .and_then(|s| s.to_str())
            .unwrap_or("Libreria")
            .to_string();
        let lib_id = format!("{}{}", prefix, folder_name);
        let mut display_name = folder_name.replace('_', " ");
        let mut license_info = "Librería Local".to_string();
        let mut description = format!("{} mallas dentales en {:?}", mesh_files.len(), folder);

        let meta_path = folder.join("library_meta.json");
        if let Ok(text) = fs::read_to_string(&meta_path) {
            if let Ok(meta) = serde_json::from_str::<LibraryMetaJson>(&text) {
                if let Some(n) = meta.name {
                    display_name = n;
                }
                if let Some(l) = meta.license {
                    license_info = l;
                }
                if let Some(d) = meta.description {
                    description = d;
                }
            }
        }

        let fdi_map = Self::map_files_to_fdi(&mesh_files);
        self.libraries.insert(
            lib_id.clone(),
            DentalLibraryInfo {
                library_id: lib_id,
                display_name,
                folder_path: folder.to_path_buf(),
                license_info,
                description,
                fdi_map,
                all_files: mesh_files,
            },
        );
    }

    fn extract_number_tokens(stem: &str) -> Vec<i32> {
        let mut nums = Vec::new();
        let mut cur = String::new();
        for ch in stem.chars() {
            if ch.is_ascii_digit() {
                cur.push(ch);
            } else if !cur.is_empty() {
                if cur.len() <= 2 {
                    if let Ok(v) = cur.parse::<i32>() {
                        nums.push(v);
                    }
                }
                cur.clear();
            }
        }
        if !cur.is_empty() && cur.len() <= 2 {
            if let Ok(v) = cur.parse::<i32>() {
                nums.push(v);
            }
        }
        nums
    }

    /// Detecta notación Palmer (`UR1..UR8`, `UL1..UL8`, `LL1..LL8`, `LR1..LR8`) en el nombre del archivo.
    fn extract_palmer_fdi(stem: &str) -> Option<i32> {
        let up = stem.to_uppercase();
        let bytes = up.as_bytes();
        for i in 0..bytes.len().saturating_sub(2) {
            let prefix = &up[i..i + 2];
            let quad = match prefix {
                "UR" => 1,
                "UL" => 2,
                "LL" => 3,
                "LR" => 4,
                _ => continue,
            };
            let rest = &up[i + 2..];
            let trimmed = rest.trim_start_matches(|c| c == '_' || c == '-' || c == ' ');
            if let Some(ch) = trimmed.chars().next() {
                if ('1'..='8').contains(&ch) {
                    let unit = (ch as u8 - b'0') as i32;
                    return Some(quad * 10 + unit);
                }
            }
        }
        None
    }

    fn map_files_to_fdi(files: &[PathBuf]) -> BTreeMap<i32, PathBuf> {
        let mut fdi_map = BTreeMap::new();
        let mut extracted: Vec<(PathBuf, i32)> = Vec::new();

        for f in files {
            let stem = f.file_stem().and_then(|s| s.to_str()).unwrap_or("");
            // 1. Notación Palmer (UR1..LR8)
            if let Some(fdi) = Self::extract_palmer_fdi(stem) {
                fdi_map.entry(fdi).or_insert_with(|| f.clone());
                continue;
            }
            // 2. Subcarpetas tipo upper/1.stl..8.stl o lower/1.stl..8.stl
            let tokens = Self::extract_number_tokens(stem);
            if tokens.len() == 1 && (1..=8).contains(&tokens[0]) {
                let parent_low = f
                    .parent()
                    .and_then(|p| p.file_name())
                    .and_then(|s| s.to_str())
                    .unwrap_or("")
                    .to_lowercase();
                if parent_low.contains("upper") || parent_low.contains("maxil") || parent_low.contains("sup") {
                    fdi_map.entry(10 + tokens[0]).or_insert_with(|| f.clone());
                    continue;
                } else if parent_low.contains("lower") || parent_low.contains("mandib") || parent_low.contains("inf") {
                    fdi_map.entry(40 + tokens[0]).or_insert_with(|| f.clone());
                    continue;
                }
            }
            if !tokens.is_empty() {
                let mut chosen = tokens[0];
                for &t in &tokens {
                    if is_valid_fdi(t) {
                        chosen = t;
                        break;
                    }
                }
                extracted.push((f.clone(), chosen));
            }
        }

        let all_vals: HashSet<i32> = extracted.iter().map(|(_, n)| *n).collect();
        let has_universal_only = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 19, 20, 29, 30]
            .iter()
            .any(|v| all_vals.contains(v));
        let has_high_fdi = [33, 34, 35, 36, 37, 38, 41, 42, 43, 44, 45, 46, 47, 48]
            .iter()
            .any(|v| all_vals.contains(v));

        if has_universal_only && !has_high_fdi {
            for (f, num) in extracted {
                if let Some(fdi) = universal_to_fdi(num) {
                    fdi_map.entry(fdi).or_insert(f);
                }
            }
        } else {
            for (f, num) in extracted {
                if is_valid_fdi(num) {
                    fdi_map.entry(num).or_insert(f);
                }
            }
        }
        fdi_map
    }

    pub fn import_library_from_folder(&mut self, source_folder: &Path) -> Result<DentalLibraryInfo, String> {
        if !source_folder.is_dir() {
            return Err(format!("La carpeta no existe: {:?}", source_folder));
        }
        let mut meshes = Vec::new();
        Self::collect_mesh_files_recursive(source_folder, &mut meshes);
        if meshes.is_empty() {
            return Err("La carpeta seleccionada no contiene archivos .STL, .OBJ ni .PLY.".to_string());
        }

        let raw_name = source_folder
            .file_name()
            .and_then(|s| s.to_str())
            .unwrap_or("Libreria_Personal");
        let safe_name: String = raw_name
            .chars()
            .map(|c| if c.is_alphanumeric() || matches!(c, ' ' | '_' | '-') { c } else { '_' })
            .collect::<String>()
            .trim()
            .to_string();
        let safe_name = if safe_name.is_empty() {
            "Libreria_Personal".to_string()
        } else {
            safe_name
        };

        let dest = self.user_lib_dir.join(&safe_name);
        if dest.exists() {
            let _ = fs::remove_dir_all(&dest);
        }
        fs::create_dir_all(&dest).map_err(|e| e.to_string())?;

        for m in &meshes {
            let fname = m.file_name().and_then(|s| s.to_str()).unwrap_or("tooth.stl");
            let mut target = dest.join(fname);
            if target.exists() {
                let p_name = m
                    .parent()
                    .and_then(|p| p.file_name())
                    .and_then(|s| s.to_str())
                    .unwrap_or("sub");
                target = dest.join(format!("{}_{}", p_name, fname));
            }
            fs::copy(m, &target).map_err(|e| e.to_string())?;
        }

        let meta = serde_json::json!({
            "name": raw_name,
            "license": "Librería Importada por Usuario (Apex DentalCraft)",
            "description": format!("Importada desde {:?} ({} piezas).", source_folder, meshes.len()),
        });
        let _ = fs::write(
            dest.join("library_meta.json"),
            serde_json::to_string_pretty(&meta).unwrap_or_default(),
        );

        self.refresh_libraries();
        self.libraries
            .get(&safe_name)
            .cloned()
            .ok_or_else(|| "No fue posible indexar la librería recién importada.".to_string())
    }

    pub fn list_libraries(&self) -> Vec<DentalLibraryInfo> {
        self.libraries.values().cloned().collect()
    }

    fn load_from_single_library(lib: &DentalLibraryInfo, fdi_tooth: i32) -> Option<TriMeshData> {
        if let Some(path) = lib.fdi_map.get(&fdi_tooth) {
            if !path.as_os_str().is_empty() && path.exists() {
                if let Ok(m) = TriMeshData::load_from_path(path) {
                    return Some(m);
                }
            }
        }
        // Simetría contralateral (1X <-> 2X, 3X <-> 4X)
        let quad = fdi_tooth / 10;
        let unit = fdi_tooth % 10;
        let mirror_q = match quad {
            1 => 2,
            2 => 1,
            3 => 4,
            4 => 3,
            q => q,
        };
        let contra = mirror_q * 10 + unit;
        if let Some(path) = lib.fdi_map.get(&contra) {
            if !path.as_os_str().is_empty() && path.exists() {
                if let Ok(mut m) = TriMeshData::load_from_path(path) {
                    for p in &mut m.positions {
                        p.x = -p.x;
                    }
                    m.flip_faces();
                    return Some(m);
                }
            }
        }
        None
    }

    pub fn load_tooth_mesh(&self, library_id: &str, fdi_tooth: i32) -> TriMeshData {
        if let Some(lib) = self.libraries.get(library_id) {
            if let Some(m) = Self::load_from_single_library(lib, fdi_tooth) {
                return m;
            }
        }
        for fallback_id in ["LIBRERIA APEX DETAILED", "BodyParts3D_Humana_FDI"] {
            if fallback_id == library_id {
                continue;
            }
            if let Some(lib) = self.libraries.get(fallback_id) {
                if let Some(m) = Self::load_from_single_library(lib, fdi_tooth) {
                    return m;
                }
            }
        }
        Self::generate_procedural_tooth_crown(fdi_tooth)
    }

    /// Genera matemáticamente una corona dental anatómica cerrada HD en mm según su FDI (11-48).
    pub fn generate_procedural_tooth_crown(fdi: i32) -> TriMeshData {
        let quad = fdi / 10;
        let unit = fdi % 10;
        let is_upper = matches!(quad, 1 | 2);
        let is_right = matches!(quad, 1 | 4);

        let (md, vl, height): (f32, f32, f32) = match (unit, is_upper) {
            (1, true) => (8.4, 7.0, 10.2),
            (1, false) => (5.4, 6.0, 8.8),
            (2, true) => (6.6, 6.2, 9.2),
            (2, false) => (5.9, 6.3, 9.2),
            (3, true) => (7.6, 8.0, 10.4),
            (3, false) => (6.8, 7.6, 10.2),
            (4, true) => (7.1, 9.1, 8.3),
            (4, false) => (7.0, 7.7, 8.4),
            (5, true) => (6.8, 9.2, 7.9),
            (5, false) => (7.2, 8.2, 8.1),
            (6, true) => (10.4, 11.2, 7.7),
            (6, false) => (11.0, 10.3, 7.7),
            (7, true) => (9.6, 10.8, 7.3),
            (7, false) => (10.2, 9.8, 7.3),
            (8, true) => (8.8, 10.0, 6.8),
            (8, false) => (9.4, 9.2, 6.8),
            _ => (9.5, 9.5, 7.5),
        };
        let rx = md * 0.5;
        let ry = vl * 0.5;

        let n_theta = 64usize;
        let n_phi = 36usize;
        let mut positions = Vec::with_capacity(2 + (n_phi - 1) * n_theta);

        let deform = |x0: f32, y0: f32, z0: f32| -> Vec3 {
            let h_norm = 0.5 * (z0 + 1.0);
            let equator_bulge = 0.80 + 0.24 * (PI * (h_norm * 1.15).clamp(0.0, 1.0)).sin();
            let angle = y0.atan2(x0);
            let squaring = 1.0 + (if unit >= 6 { 0.08 } else { 0.03 }) * (4.0 * angle).cos();

            let mut x = x0 * rx * equator_bulge * squaring;
            let mut y = y0 * ry * equator_bulge * squaring;
            let mut z = (h_norm - 0.5) * height;

            let occ_mask = ((h_norm - 0.45) / 0.55).clamp(0.0, 1.0);
            let nx_u = x / rx.max(1e-3);
            let ny_u = y / ry.max(1e-3);

            match unit {
                1 | 2 => {
                    let lingual_scoop = -0.30 * height * (-((nx_u / 0.7).powi(2) + ((ny_u - 0.35) / 0.48).powi(2))).exp();
                    let cingulum = 0.14 * height * (-((nx_u / 0.65).powi(2) + ((ny_u - 0.78) / 0.28).powi(2))).exp();
                    let incisal = 0.14 * height * (-((ny_u + 0.15) / 0.22).powi(2)).exp();
                    y *= 1.0 - 0.52 * occ_mask.powf(1.5);
                    z += occ_mask * (lingual_scoop + incisal) + (1.0 - occ_mask * 0.5) * cingulum;
                }
                3 => {
                    let peak = 0.24 * height * (-((nx_u / 0.42).powi(2) + ((ny_u + 0.10) / 0.38).powi(2))).exp();
                    let fossa = -0.18 * height * (-((nx_u / 0.65).powi(2) + ((ny_u - 0.38) / 0.42).powi(2))).exp();
                    y *= 1.0 - 0.36 * occ_mask.powf(1.4);
                    z += occ_mask * (peak + fossa);
                }
                4 | 5 => {
                    let c_bucc = 0.22 * height * (-((nx_u / 0.62).powi(2) + ((ny_u + 0.46) / 0.36).powi(2))).exp();
                    let c_ling = 0.19 * height * (-((nx_u / 0.58).powi(2) + ((ny_u - 0.46) / 0.36).powi(2))).exp();
                    let groove = -0.16 * height * (-(ny_u / 0.18).powi(2)).exp() * (-(nx_u / 0.72).powi(4)).exp();
                    z += occ_mask * (c_bucc + c_ling + groove);
                }
                _ => {
                    let cusps = 0.22 * height * (-(((nx_u + 0.44) / 0.42).powi(2) + ((ny_u + 0.44) / 0.40).powi(2))).exp()
                        + 0.20 * height * (-(((nx_u - 0.44) / 0.42).powi(2) + ((ny_u + 0.44) / 0.40).powi(2))).exp()
                        + 0.21 * height * (-(((nx_u + 0.42) / 0.42).powi(2) + ((ny_u - 0.44) / 0.40).powi(2))).exp()
                        + 0.18 * height * (-(((nx_u - 0.42) / 0.42).powi(2) + ((ny_u - 0.44) / 0.40).powi(2))).exp();
                    let fossa = -0.18 * height * ((-(ny_u / 0.16).powi(2)).exp() * (-(nx_u / 0.80).powi(4)).exp()
                        + 0.75 * (-(nx_u / 0.16).powi(2)).exp() * (-(ny_u / 0.80).powi(4)).exp());
                    z += occ_mask * (cusps + fossa);
                }
            }

            let base_mask = ((0.15 - h_norm) / 0.15).clamp(0.0, 1.0);
            z = z * (1.0 - 0.35 * base_mask) - (0.35 * base_mask * height * 0.45);

            if is_right {
                x = -x;
            }
            if is_upper {
                z = -z;
            }
            Vec3::new(x, y, z)
        };

        // Polo oclusal (+1)
        positions.push(deform(0.0, 0.0, 1.0));
        for i in 1..n_phi {
            let phi = PI * (i as f32) / (n_phi as f32);
            let z0 = phi.cos();
            let s_phi = phi.sin();
            for j in 0..n_theta {
                let th = 2.0 * PI * (j as f32) / (n_theta as f32);
                positions.push(deform(s_phi * th.cos(), s_phi * th.sin(), z0));
            }
        }
        // Polo cervical (-1)
        let south_idx = positions.len() as u32;
        positions.push(deform(0.0, 0.0, -1.0));

        let mut indices = Vec::with_capacity(n_theta * 2 + (n_phi - 2) * n_theta * 2);
        let ring_idx = |ri: usize, j: usize| -> u32 { (1 + (ri - 1) * n_theta + (j % n_theta)) as u32 };

        for j in 0..n_theta {
            indices.push([0, ring_idx(1, j), ring_idx(1, j + 1)]);
        }
        for ri in 1..(n_phi - 1) {
            for j in 0..n_theta {
                let a0 = ring_idx(ri, j);
                let a1 = ring_idx(ri, j + 1);
                let b0 = ring_idx(ri + 1, j);
                let b1 = ring_idx(ri + 1, j + 1);
                indices.push([a0, b0, b1]);
                indices.push([a0, b1, a1]);
            }
        }
        for j in 0..n_theta {
            indices.push([south_idx, ring_idx(n_phi - 1, j + 1), ring_idx(n_phi - 1, j)]);
        }

        let mut mesh = TriMeshData::new(positions, indices);
        mesh.ensure_outward_winding_if_closed();
        mesh
    }
}
