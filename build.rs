use std::env;
use std::path::PathBuf;
use std::process::Command;

fn main() {
    println!("cargo:rerun-if-changed=assets/app.rc");
    println!("cargo:rerun-if-changed=assets/apex_dentalcraft.ico");

    let target_os = env::var("CARGO_CFG_TARGET_OS").unwrap_or_default();
    if target_os != "windows" {
        return;
    }

    let out_dir = PathBuf::from(env::var("OUT_DIR").expect("OUT_DIR no definido"));
    let rc_path = PathBuf::from("assets/app.rc");
    let obj_path = out_dir.join("apex_dentalcraft_res.o");

    // Intentar windres (LLVM-MinGW / MinGW) para incrustar icono y metadatos en el .exe
    let windres_candidates = ["x86_64-w64-mingw32-windres", "llvm-windres", "windres"];
    let mut compiled = false;

    for tool in windres_candidates {
        let status = Command::new(tool)
            .args([
                "-I",
                "assets",
                "-O",
                "coff",
                "-i",
                rc_path.to_str().unwrap(),
                "-o",
                obj_path.to_str().unwrap(),
            ])
            .status();

        if let Ok(st) = status {
            if st.success() {
                println!("cargo:rustc-link-arg={}", obj_path.display());
                compiled = true;
                break;
            }
        }
    }

    if !compiled {
        println!(
            "cargo:warning=No se encontró windres; el ejecutable se compilará sin icono PE incrustado."
        );
    }
}
