<div align="center">

# APEX DENTALCRAFT

**Native Rust / OpenGL Dental CAD/CAM Suite & 3D Mesh Viewer (`v0.3.0-alpha`)**

Ultra-fast · 100% Offline · Portable (~20 MB) · OpenGL 3.3 Core + `parry3d` BVH

[English](#english) · [Español](#español) · [Português](#português)

</div>

---

<a id="english"></a>

## English

### Overview

**Apex DentalCraft** is a **100% native Rust / OpenGL** dental CAD/CAM design suite and 3D mesh viewer engineered for speed, low memory footprint, and completely **offline, portable operation (~20 MB)** with zero Python or heavy external runtime dependencies.

The release includes both the full multi-stage CAD suite (`ApexDentalCraft.exe`) and the standalone inspection utility (`Apex3DViewer.exe`), sharing a unified GPU rendering core and proprietary `.apexscene` project bundle format.

### Current Features (`v0.3.0-alpha`)

- **Case Management & Dental Orders**: Local case storage under `Casos/`, patient/doctor metadata, VITA shade presets, and 7 clinical CAD/CAM material presets (monolithic/multilayer zirconia, lithium disilicate `e.max`, milled PMMA, 3D printed resin, castable wax/resin, CoCr).<img width="1867" height="1105" alt="image" src="https://github.com/user-attachments/assets/2e409b95-3537-44f1-9fa7-ade906d732a1" />

- **Interactive FDI Odontogram**: Multi-unit order planning with individual crowns, veneers, inlays/onlays, copings, bridge abutments, bridge pontics, and inter-tooth connectors (`●`).
- **Ultra-Fast 3D Mesh Viewer (`Apex 3D Viewer`)**: Instant loading of `.stl` (binary & ASCII), `.obj`, `.ply`, and `.apexscene` files/folders with independent layer opacities, quick scene presets, real-time wall-thickness/occlusal-distance/die-clearance/undercut/curvature maps, GPU cross-section plane, 3D distance ruler, 3-point angle goniometer, CAM watertightness audit, normal flipping, and PNG snapshot export.
- **Model Trimmer & Printable Base Builder**: Multi-cut free trimming plane visible in OpenGL and printable base generator supporting both solid bases and **1–3 mm hollow bases** with manifold healing.
- **Cervical Margin Line Tracing**: Magnetic snapping on prep dies, interactive control nodes (Left-Click to place/edit, Left+Right Click to delete), live undercut/collar validation, pontic margin bypass, and multi-unit greyed reference curves.<img width="1864" height="1101" alt="image" src="https://github.com/user-attachments/assets/905d2e4d-4ad1-45d5-b5f5-65d0f198c2a0" />

- **Cement Space, Minimum Thickness & Marginal Bevel**: Configurable internal cement gap, ramp width, bur drill compensation, minimum wall thickness safety shell, and adjustable marginal bevel (width in mm and angle in degrees).
- **CAD Placement, Occlusal/Proximal Adaptation & Sculpting**: Isotropic tooth library insertion (`LIBRERIA APEX DETAILED` procedural/custom STL support), 3D gizmo transforms, local morphing, smooth/add/subtract/pinch brushes, proximal/occlusal contact adaptation, and smart pontic-to-gingiva adaptation.<img width="1862" height="1097" alt="image" src="https://github.com/user-attachments/assets/8ad14606-9fca-4b40-9078-696e7786bc8b" />

- **Bridges, Connectors & Final Bridge Sculpting**: Editable multi-node bridge connectors with live cross-section area readout ($\text{mm}^2$), crisp topological union preserving crown anatomy, and a dedicated final bridge sculpting pass before export.
- **Self-Contained Export & Reopening**: One-click export into `<program_dir>/Casos/<case_id>/` generating the `.apexscene` file, designed restoration/bridge STLs, working die STL, antagonist STL, optional bite scan, and `.txt` lab sheet — reopenable from Module 1, drag-and-drop, or `Apex 3D Viewer`.<img width="1870" height="1108" alt="image" src="https://github.com/user-attachments/assets/94b1dfbf-04c8-4eab-ad45-10b52e5b8f4c" />

- **Multi-Language Support**: Full dental terminology in **English (default)**, **Spanish**, and **Portuguese** (configurable under `⚙ Settings`).

### Download

Pre-built portable releases are available on GitHub Releases:

- 👉 **[https://github.com/amengarci/Apex-DentalCraft/releases](https://github.com/amengarci/Apex-DentalCraft/releases)**

Download `ApexDentalCraft-v0.3.0-alpha-Portable.zip`, extract it to any folder or USB drive, and run `ApexDentalCraft.exe` or `Apex3DViewer.exe`. No installation is required.

### Build from Source

**Prerequisites:**
- [Rust toolchain](https://rustup.rs/) (`stable`, Edition 2021)
- OpenGL 3.3+ capable GPU drivers
- *Windows*: MinGW/LLVM-MinGW (`x86_64-pc-windows-gnullvm`) or MSVC toolchain
- *Linux (X11/Wayland)*: standard native dialog/window dev packages (`libgtk-3-dev`, `libxkbcommon-dev`, `libgl1-mesa-dev`)

```bash
git clone https://github.com/amengarci/Apex-DentalCraft.git
cd Apex-DentalCraft

# Run the full CAD suite in release mode
cargo run --release --bin apex-dentalcraft

# Or build both executables (CAD suite + standalone 3D Viewer)
cargo build --release --bin apex-dentalcraft --bin apex-3d-viewer
```

### License

Licensed under the **GNU General Public License v3.0 (GPLv3)**. See the [LICENSE](LICENSE) file for complete terms and conditions.

### Clinical Disclaimer

> **IMPORTANT CLINICAL DISCLAIMER**
>
> **Apex DentalCraft (`v0.3.0-alpha`) is an experimental geometric CAD/CAM design and 3D mesh inspection software tool.** It is **not** a certified or cleared medical device (FDA, CE, COFEPRIS, ANVISA, etc.). All geometric calculations, margin lines, cement spaces, thickness maps, bridge connectors, and exported STL/OBJ meshes must be independently inspected and validated before manufacturing or intraoral placement. **Clinical validation, fit verification, material compliance, and final clinical responsibility rest exclusively with the licensed dental technician or dentist.**

---

<a id="español"></a>

## Español

### Descripción

**Apex DentalCraft** es una suite CAD/CAM dental y visor de mallas 3D **100% nativo en Rust y OpenGL**, diseñado para ofrecer respuesta instantánea, bajo consumo de recursos y funcionamiento **completamente offline y portable (~20 MB)**, sin depender de Python ni librerías externas pesadas.

Incluye el flujo completo de diseño protésico (`ApexDentalCraft.exe`) y el visor independiente (`Apex3DViewer.exe`), compartiendo el mismo motor OpenGL 3.3 Core y el formato propio de escena `.apexscene`.

### Funciones Actuales (`v0.3.0-alpha`)

- **Gestor de Casos y Órdenes**: Almacenamiento local en `Casos/`, datos de paciente/odontólogo, guía de color VITA y 7 presets clínicos de materiales (zirconia monolítica/multicapa, disilicato de litio `e.max`, PMMA fresado, resina 3D, cera/resina calcinable y CoCr).
- **Odontograma FDI Interactivo**: Planificación de coronas individuales, carillas, incrustaciones, cofias, pilares y pónticos de puente con conectores entre piezas (`●`).
- **Visor 3D Ultrarrápido (`Apex 3D Viewer`)**: Carga inmediata de archivos y carpetas `.stl`, `.obj`, `.ply` y `.apexscene` con opacidades independientes, mapas de grosor de pared, distancia oclusal/muñón, socavados y curvatura, plano de corte GPU, regla 3D, goniómetro, auditoría watertight CAM, inversión de normales y captura PNG.
- **Recortador de Modelos y Zócalo Imprimible**: Plano visible en OpenGL con cortes múltiples y generador de zócalo sólido o **ahuecado (hollow) de 1 mm a 3 mm** sin conflictos geométricos.
- **Trazado de Línea de Margen Cervical**: Ajuste magnético al muñón, edición de nodos (Clic Izq para añadir/mover, Clic Izq+Der para eliminar), validación en tiempo real de socavados, omisión automática de margen en pónticos y visualización de márgenes vecinos en gris.
- **Espacio de Cementado, Grosor Mínimo y Bisel Marginal**: Control de espacio de cemento, rampa marginal, compensación de fresa CAM, capa de grosor mínimo y bisel marginal configurable.
- **Diseño Anatómico, Cortes Oclusales/Proximales y Esculpido**: Inserción isotrópica de librería dental, gizmo 3D, morphing local, pinceles de esculpido, ajuste automático a antagonista y contactos proximales, y herramienta inteligente para adaptar póntico a la encía.
- **Puentes, Conectores y Esculpido Final**: Conectores editables por nodos con área transversal ($\text{mm}^2$), unión topológica nítida y módulo dedicado para esculpido libre sobre el puente unido antes de exportar.
- **Exportación en `Casos/` y Reapertura de Escena**: Exportación directa a `<carpeta_del_programa>/Casos/<case_id>/` generando el archivo `.apexscene` y los STL de restauraciones/puentes, muñón/modelo de trabajo, antagonista, mordida y ficha técnica `.txt`.
- **Soporte Multi-Idioma**: Interfaz completa en **Inglés (por defecto)**, **Español** y **Portugués** seleccionable desde `⚙ Settings / Configuración`.

### Descarga

Los paquetes portables precompilados están disponibles en GitHub Releases:

- 👉 **[https://github.com/amengarci/Apex-DentalCraft/releases](https://github.com/amengarci/Apex-DentalCraft/releases)**

Descarga `ApexDentalCraft-v0.3.0-alpha-Portable.zip`, descomprímelo en cualquier carpeta o memoria USB y ejecuta `ApexDentalCraft.exe` o `Apex3DViewer.exe` sin instalación.

### Compilación desde el Código Fuente

```bash
git clone https://github.com/amengarci/Apex-DentalCraft.git
cd Apex-DentalCraft

cargo run --release
```

*(Para compilar ambos ejecutables en modo release: `cargo build --release --bin apex-dentalcraft --bin apex-3d-viewer`.)*

### Licencia

Distribuido bajo la licencia **GNU General Public License v3.0 (GPLv3)**. Consulta el archivo [LICENSE](LICENSE).

### Aviso Clínico (Clinical Disclaimer)

> **AVISO DE RESPONSABILIDAD CLÍNICA**
>
> **Apex DentalCraft (`v0.3.0-alpha`) es exclusivamente una herramienta geométrica de diseño CAD/CAM e inspección de mallas 3D**; **no** es un dispositivo médico certificado. La validación clínica, verificación de ajuste marginal, contactos oclusales/proximales, idoneidad del material y la responsabilidad clínica final corresponden exclusivamente al técnico dental u odontólogo tratante.

---

<a id="português"></a>

## Português

### Visão Geral

**Apex DentalCraft** é uma suíte CAD/CAM odontológica e visualizador de malhas 3D **100% nativo em Rust e OpenGL**, projetado para alto desempenho, funcionamento **totalmente offline e portátil (~20 MB)**, sem dependências pesadas de Python ou runtimes externos.

### Funcionalidades Atuais (`v0.3.0-alpha`)

- **Gestão de Casos e Ordens**: Armazenamento local em `Casos/`, dados de paciente/dentista, escala VITA e 7 predefinições clínicas de materiais (zircônia monolítica/multicamadas, dissilicato de lítio `e.max`, PMMA fresado, resina 3D, cera calcinável e CoCr).
- **Odontograma FDI Interativo**: Planejamento de coroas unitárias, facetas, inlays/onlays, copings, pilares e pônticos de ponte com conectores (`●`).
- **Visualizador 3D Ultrarrápido (`Apex 3D Viewer`)**: Leitura instantânea de `.stl`, `.obj`, `.ply` e `.apexscene`, mapas de espessura de parede, distância oclusal/troquel, retenções e curvatura, corte transversal GPU, régua 3D, goniômetro, auditoria watertight CAM e captura PNG.
- **Recorte de Modelos e Base Impressa 3D**: Plano visível em OpenGL com múltiplos cortes e criação de base sólida ou **oca (hollow) de 1 mm a 3 mm**.
- **Traçado de Margem Cervical**: Ajuste magnético no troquel, edição por nós, validação de retenções em tempo real, pônticos sem exigência de término e visualização de termos vizinhos em cinza.
- **Espaço de Cimentação, Espessura Mínima e Bisel Marginal**: Parâmetros de cimento, compensação de broca CAM, camada de espessura mínima e bisel marginal configurável.
- **Modelagem, Adaptação Oclusal/Proximal e Escultura**: Biblioteca dentária isotrópica, gizmo 3D, morphing, pincéis de escultura, adaptação a antagonista/contatos proximais e adaptação inteligente de pôntico à gengiva.
- **Pontes, Conectores e Escultura Final**: Conectores editáveis por nós com área em $\text{mm}^2$, união topológica precisa e etapa dedicada de escultura na ponte unida antes da exportação.
- **Exportação em `Casos/` e Reabertura**: Salva arquivo `.apexscene`, STLs de restaurações/pontes, troquel, antagonista, registro oclusal e ficha técnica `.txt`.
- **Suporte Multi-Idioma**: Interface técnica completa em **Inglês (padrão)**, **Espanhol** e **Português**.

### Download e Compilação

- **Releases**: **[https://github.com/amengarci/Apex-DentalCraft/releases](https://github.com/amengarci/Apex-DentalCraft/releases)**

```bash
git clone https://github.com/amengarci/Apex-DentalCraft.git
cd Apex-DentalCraft
cargo run --release
```

### Licença e Isenção Clínica (GPLv3)

Licenciado sob a **GNU General Public License v3.0 (GPLv3)** ([LICENSE](LICENSE)).

> **AVISO CLÍNICO**: O **Apex DentalCraft (`v0.3.0-alpha`)** é exclusivamente uma ferramenta geométrica de design CAD/CAM e não constitui um dispositivo médico certificado. A validação clínica, adaptação marginal, ajuste oclusal e responsabilidade clínica final pertencem exclusivamente ao cirurgião-dentista ou técnico em prótese dentária responsável.
