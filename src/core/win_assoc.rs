//! Integración nativa de **Apex 3D Viewer** con Windows ("Abrir con...") y utilidades de
//! captura PNG sin dependencias externas (100% offline, sin privilegios de administrador).

use std::path::{Path, PathBuf};

/// Localiza el ejecutable independiente `Apex3DViewer.exe` junto al binario actual.
pub fn find_standalone_viewer_exe() -> Option<PathBuf> {
    let cur = std::env::current_exe().ok()?;
    let cur_name = cur
        .file_name()
        .and_then(|s| s.to_str())
        .unwrap_or("")
        .to_ascii_lowercase();
    if cur_name.contains("viewer") {
        return Some(cur);
    }
    let dir = cur.parent()?;
    for cand in ["Apex3DViewer.exe", "apex-3d-viewer.exe"] {
        let p = dir.join(cand);
        if p.exists() {
            return Some(p);
        }
    }
    None
}

/// Registra `Apex3DViewer.exe` en el registro de usuario de Windows (`HKCU\Software\Classes`)
/// para que aparezca de inmediato en clic derecho → **"Abrir con → Apex 3D Viewer (DentalCraft)"**
/// para archivos `.stl`, `.obj` y `.ply` (sin requerir permisos de administrador).
#[cfg(target_os = "windows")]
pub fn register_windows_open_with(viewer_exe: &Path, set_default_stl: bool) -> Result<String, String> {
    use std::os::windows::process::CommandExt;
    use std::process::Command;

    const CREATE_NO_WINDOW: u32 = 0x0800_0000;

    let exe_str = viewer_exe
        .canonicalize()
        .unwrap_or_else(|_| viewer_exe.to_path_buf())
        .to_string_lossy()
        .trim_start_matches(r"\\?\")
        .to_string();
    let exe_name = viewer_exe
        .file_name()
        .and_then(|s| s.to_str())
        .unwrap_or("Apex3DViewer.exe");
    let open_cmd = format!("\"{}\" \"%1\"", exe_str);
    let icon_val = format!("\"{}\",0", exe_str);
    let prog_id = "ApexDentalCraft.3DModel";

    let run_reg = |args: &[&str]| -> Result<(), String> {
        let out = Command::new("reg.exe")
            .args(args)
            .creation_flags(CREATE_NO_WINDOW)
            .output()
            .map_err(|e| format!("No se pudo ejecutar reg.exe: {}", e))?;
        if !out.status.success() {
            let err = String::from_utf8_lossy(&out.stderr);
            return Err(format!("Error registrando asociación: {}", err.trim()));
        }
        Ok(())
    };

    // 1. ProgID dedicado con icono y comando de apertura
    run_reg(&["add", &format!(r"HKCU\Software\Classes\{}", prog_id), "/ve", "/d", "Modelo Dental 3D (Apex 3D Viewer)", "/f"])?;
    run_reg(&["add", &format!(r"HKCU\Software\Classes\{}\DefaultIcon", prog_id), "/ve", "/d", &icon_val, "/f"])?;
    run_reg(&["add", &format!(r"HKCU\Software\Classes\{}\shell\open\command", prog_id), "/ve", "/d", &open_cmd, "/f"])?;

    // 2. Entrada en Applications\<exe> para el menú contextual "Abrir con..."
    let app_key = format!(r"HKCU\Software\Classes\Applications\{}", exe_name);
    run_reg(&["add", &app_key, "/v", "FriendlyAppName", "/t", "REG_SZ", "/d", "Apex 3D Viewer (DentalCraft)", "/f"])?;
    run_reg(&["add", &format!(r"{}\DefaultIcon", app_key), "/ve", "/d", &icon_val, "/f"])?;
    run_reg(&["add", &format!(r"{}\shell\open", app_key), "/v", "FriendlyAppName", "/t", "REG_SZ", "/d", "Apex 3D Viewer (DentalCraft)", "/f"])?;
    run_reg(&["add", &format!(r"{}\shell\open\command", app_key), "/ve", "/d", &open_cmd, "/f"])?;

    // 3. Registrar formatos admitidos (.stl, .obj, .ply, .apexscene) en OpenWithProgids y SupportedTypes
    for ext in [".stl", ".obj", ".ply", ".apexscene"] {
        run_reg(&["add", &format!(r"{}\SupportedTypes", app_key), "/v", ext, "/t", "REG_SZ", "/d", "", "/f"])?;
        run_reg(&["add", &format!(r"HKCU\Software\Classes\{}\OpenWithProgids", ext), "/v", prog_id, "/t", "REG_NONE", "/f"])?;
        if set_default_stl && ext == ".stl" {
            run_reg(&["add", r"HKCU\Software\Classes\.stl", "/ve", "/d", prog_id, "/f"])?;
        }
    }

    if set_default_stl {
        Ok(format!(
            "✔ '{}' registrado en Windows para .STL/.OBJ/.PLY/.APEXSCENE (clic derecho → Abrir con → Apex 3D Viewer).",
            exe_name
        ))
    } else {
        Ok(format!(
            "✔ '{}' añadido al menú 'Abrir con...' de Windows para .STL, .OBJ, .PLY y .APEXSCENE.",
            exe_name
        ))
    }
}

#[cfg(not(target_os = "windows"))]
pub fn register_windows_open_with(_viewer_exe: &Path, _set_default_stl: bool) -> Result<String, String> {
    Err("La integración con el menú 'Abrir con...' solo aplica en Windows.".to_string())
}

/// Codificador PNG estándar 100% puro en Rust (RGBA8, bloques zlib sin compresión, CRC32 + Adler32)
/// para guardar capturas clínicas del visor 3D sin dependencias externas.
pub fn write_png_rgba(path: &Path, width: u32, height: u32, rgba: &[u8]) -> Result<(), String> {
    if width == 0 || height == 0 || rgba.len() < (width as usize * height as usize * 4) {
        return Err("Imagen de captura vacía o inválida.".to_string());
    }
    if let Some(parent) = path.parent() {
        let _ = std::fs::create_dir_all(parent);
    }

    let crc32 = |data: &[u8]| -> u32 {
        let mut crc = 0xFFFF_FFFFu32;
        for &b in data {
            crc ^= b as u32;
            for _ in 0..8 {
                if (crc & 1) != 0 {
                    crc = (crc >> 1) ^ 0xEDB8_8320;
                } else {
                    crc >>= 1;
                }
            }
        }
        !crc
    };

    let push_chunk = |out: &mut Vec<u8>, tag: &[u8; 4], payload: &[u8]| {
        out.extend_from_slice(&(payload.len() as u32).to_be_bytes());
        let start = out.len();
        out.extend_from_slice(tag);
        out.extend_from_slice(payload);
        let c = crc32(&out[start..]);
        out.extend_from_slice(&c.to_be_bytes());
    };

    // Construir filas filtradas (filtro 0 = None por fila)
    let row_bytes = width as usize * 4;
    let mut raw = Vec::with_capacity(height as usize * (1 + row_bytes));
    for y in 0..height as usize {
        raw.push(0u8);
        let s = y * row_bytes;
        raw.extend_from_slice(&rgba[s..s + row_bytes]);
    }

    // Stream zlib válido con bloques DEFLATE almacenados (BTYPE = 00) + Adler32
    let mut zlib = Vec::with_capacity(raw.len() + (raw.len() / 65535 + 1) * 5 + 6);
    zlib.push(0x78);
    zlib.push(0x01);
    let mut pos = 0usize;
    while pos < raw.len() {
        let len = (raw.len() - pos).min(65535);
        let is_final = if pos + len >= raw.len() { 1u8 } else { 0u8 };
        zlib.push(is_final);
        let l16 = len as u16;
        zlib.extend_from_slice(&l16.to_le_bytes());
        zlib.extend_from_slice(&(!l16).to_le_bytes());
        zlib.extend_from_slice(&raw[pos..pos + len]);
        pos += len;
    }
    let mut a = 1u32;
    let mut b = 0u32;
    for &byte in &raw {
        a = (a + byte as u32) % 65521;
        b = (b + a) % 65521;
    }
    let adler = (b << 16) | a;
    zlib.extend_from_slice(&adler.to_be_bytes());

    let mut png = Vec::with_capacity(zlib.len() + 64);
    png.extend_from_slice(&[137, 80, 78, 71, 13, 10, 26, 10]);

    let mut ihdr = Vec::with_capacity(13);
    ihdr.extend_from_slice(&width.to_be_bytes());
    ihdr.extend_from_slice(&height.to_be_bytes());
    ihdr.push(8); // bit depth
    ihdr.push(6); // RGBA
    ihdr.push(0); // compression
    ihdr.push(0); // filter
    ihdr.push(0); // interlace
    push_chunk(&mut png, b"IHDR", &ihdr);
    push_chunk(&mut png, b"IDAT", &zlib);
    push_chunk(&mut png, b"IEND", &[]);

    std::fs::write(path, &png).map_err(|e| format!("No se pudo guardar captura PNG: {}", e))
}
