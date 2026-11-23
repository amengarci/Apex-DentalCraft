//! Motor de Renderizado 3D Nativo OpenGL 3.3 Core (`glow`) para **Apex DentalCraft**
//!
//! Renderiza directamente dentro de `egui::PaintCallback` con:
//! - Fondo degradado de estudio ciruela imperial (`#120218` -> `#2C0838`).
//! - Iluminación de laboratorio dental de 3 puntos + brillo cerámico especular + halo amatista.
//! - Múltiples capas con ordenación de transparencia y plano de corte transversal GPU 50%.
//! - Mapas térmicos por vértice (Socavados RdYlGn, Curvatura Magma, Oclusión Jet, Espesor Viridis).

use crate::geometry::mesh::TriMeshData;
use crate::render::camera::OrbitCamera;
use crate::theme::{VIEWPORT_BG_BOTTOM, VIEWPORT_BG_TOP};
use bytemuck::{Pod, Zeroable};
use eframe::glow::{self, HasContext};
use glam::{Mat4, Vec3, Vec4};
use std::collections::HashMap;
use std::sync::{Arc, Mutex};

#[repr(C)]
#[derive(Copy, Clone, Debug, Pod, Zeroable)]
pub struct GpuVertex {
    pub pos: [f32; 3],
    pub normal: [f32; 3],
    pub color: [f32; 3],
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ColorMapKind {
    Solid,
    UndercutRdYlGn,
    CurvatureMagma,
    OcclusalJetR,
    ThicknessViridis,
}

#[derive(Debug, Clone)]
pub struct RenderLayerSpec {
    pub key: String,
    #[allow(dead_code)]
    pub revision: u64,
    pub tint: [f32; 3],
    pub opacity: f32,
    pub specular: f32,
    pub shininess: f32,
    pub wireframe: bool,
    pub use_vertex_color: bool,
    pub unlit: bool,
    pub clip_affected: bool,
    pub always_on_top: bool,
}

impl RenderLayerSpec {
    pub fn solid(key: impl Into<String>, revision: u64, tint: [f32; 3], opacity: f32) -> Self {
        Self {
            key: key.into(),
            revision,
            tint,
            opacity,
            specular: 0.42,
            shininess: 32.0,
            wireframe: false,
            use_vertex_color: false,
            unlit: false,
            clip_affected: true,
            always_on_top: false,
        }
    }
}

struct GpuMeshSlot {
    revision: u64,
    vao: glow::VertexArray,
    vbo: glow::Buffer,
    ebo: glow::Buffer,
    index_count: i32,
}

pub struct GlViewportRenderer {
    bg_program: glow::Program,
    bg_vao: glow::VertexArray,
    mesh_program: glow::Program,
    slots: HashMap<String, GpuMeshSlot>,
    pending_uploads: HashMap<String, (u64, Vec<GpuVertex>, Vec<u32>)>,
}

const BG_VERT: &str = r#"#version 330 core
out vec2 v_uv;
void main() {
    vec2 pos[3] = vec2[](vec2(-1.0, -1.0), vec2(3.0, -1.0), vec2(-1.0, 3.0));
    vec2 p = pos[gl_VertexID];
    v_uv = p * 0.5 + 0.5;
    gl_Position = vec4(p, 0.9999, 1.0);
}
"#;

const BG_FRAG: &str = r#"#version 330 core
in vec2 v_uv;
out vec4 FragColor;
uniform vec3 u_bottom;
uniform vec3 u_top;
void main() {
    vec3 col = mix(u_bottom, u_top, clamp(v_uv.y, 0.0, 1.0));
    FragColor = vec4(col, 1.0);
}
"#;

const MESH_VERT: &str = r#"#version 330 core
layout(location = 0) in vec3 a_pos;
layout(location = 1) in vec3 a_normal;
layout(location = 2) in vec3 a_color;

uniform mat4 u_mvp;

out vec3 v_world_pos;
out vec3 v_normal;
out vec3 v_color;

void main() {
    v_world_pos = a_pos;
    v_normal = a_normal;
    v_color = a_color;
    gl_Position = u_mvp * vec4(a_pos, 1.0);
}
"#;

const MESH_FRAG: &str = r#"#version 330 core
in vec3 v_world_pos;
in vec3 v_normal;
in vec3 v_color;

out vec4 FragColor;

uniform vec3 u_eye;
uniform vec3 u_tint;
uniform float u_opacity;
uniform float u_specular;
uniform float u_shininess;
uniform float u_use_vertex_color;
uniform float u_unlit;
uniform float u_clip_enabled;
uniform vec4 u_clip_plane;

void main() {
    if (u_clip_enabled > 0.5) {
        if (dot(v_world_pos, u_clip_plane.xyz) + u_clip_plane.w < 0.0) {
            discard;
        }
    }

    vec3 base_col = mix(u_tint, v_color, u_use_vertex_color);
    if (u_unlit > 0.5) {
        FragColor = vec4(base_col, u_opacity);
        return;
    }

    vec3 N = normalize(v_normal);
    if (!gl_FrontFacing) {
        N = -N;
    }
    vec3 V = normalize(u_eye - v_world_pos);
    vec3 up_ref = abs(V.z) > 0.95 ? vec3(0.0, 1.0, 0.0) : vec3(0.0, 0.0, 1.0);
    vec3 right_v = normalize(cross(up_ref, V));
    vec3 cam_up = normalize(cross(V, right_v));

    // Estudio dental 3 puntos (luz principal cálida, relleno frío, halo amatista Apex)
    vec3 L1 = normalize(V * 0.72 + right_v * 0.45 + cam_up * 0.52);
    vec3 L2 = normalize(V * 0.35 - right_v * 0.78 - cam_up * 0.28);
    vec3 L3 = normalize(-V * 0.35 + cam_up * 0.85);

    float diff1 = max(dot(N, L1), 0.0);
    float diff2 = max(dot(N, L2), 0.0);
    float diff3 = max(dot(N, L3), 0.0);

    vec3 H1 = normalize(L1 + V);
    float spec1 = pow(max(dot(N, H1), 0.0), max(u_shininess, 4.0)) * u_specular;
    float fresnel = pow(1.0 - max(dot(N, V), 0.0), 3.2);

    vec3 ambient = base_col * 0.26;
    vec3 lit = ambient
             + base_col * (diff1 * 0.72 + diff2 * 0.24 + diff3 * 0.14)
             + vec3(1.0, 0.98, 0.94) * spec1
             + vec3(0.62, 0.31, 0.87) * (fresnel * 0.16);

    FragColor = vec4(clamp(lit, 0.0, 1.0), u_opacity);
}
"#;

unsafe fn compile_program(gl: &glow::Context, vert: &str, frag: &str) -> glow::Program {
    let program = gl.create_program().expect("create_program");
    let vs = gl.create_shader(glow::VERTEX_SHADER).expect("vs");
    gl.shader_source(vs, vert);
    gl.compile_shader(vs);
    assert!(gl.get_shader_compile_status(vs), "VS error: {}", gl.get_shader_info_log(vs));

    let fs = gl.create_shader(glow::FRAGMENT_SHADER).expect("fs");
    gl.shader_source(fs, frag);
    gl.compile_shader(fs);
    assert!(gl.get_shader_compile_status(fs), "FS error: {}", gl.get_shader_info_log(fs));

    gl.attach_shader(program, vs);
    gl.attach_shader(program, fs);
    gl.link_program(program);
    assert!(gl.get_program_link_status(program), "Link error: {}", gl.get_program_info_log(program));

    gl.detach_shader(program, vs);
    gl.detach_shader(program, fs);
    gl.delete_shader(vs);
    gl.delete_shader(fs);
    program
}

impl GlViewportRenderer {
    pub fn new(gl: &glow::Context) -> Self {
        unsafe {
            let bg_program = compile_program(gl, BG_VERT, BG_FRAG);
            let bg_vao = gl.create_vertex_array().expect("bg_vao");
            let mesh_program = compile_program(gl, MESH_VERT, MESH_FRAG);
            Self {
                bg_program,
                bg_vao,
                mesh_program,
                slots: HashMap::new(),
                pending_uploads: HashMap::new(),
            }
        }
    }

    /// Indica si la malla `key` ya está subida o en cola con la revisión exacta `revision`.
    pub fn is_mesh_current(&self, key: &str, revision: u64) -> bool {
        if let Some(slot) = self.slots.get(key) {
            if slot.revision == revision {
                return true;
            }
        }
        if let Some((pend_rev, _, _)) = self.pending_uploads.get(key) {
            if *pend_rev == revision {
                return true;
            }
        }
        false
    }

    /// Programa la subida a GPU de `mesh` solo si cambió su `revision`.
    pub fn ensure_mesh_uploaded(
        &mut self,
        key: &str,
        revision: u64,
        mesh: &TriMeshData,
        colormap: ColorMapKind,
    ) {
        if let Some(slot) = self.slots.get(key) {
            if slot.revision == revision {
                return;
            }
        }
        if let Some((pend_rev, _, _)) = self.pending_uploads.get(key) {
            if *pend_rev == revision {
                return;
            }
        }
        let verts = build_gpu_vertices(mesh, colormap);
        let mut idx = Vec::with_capacity(mesh.indices.len() * 3);
        for &[a, b, c] in &mesh.indices {
            idx.push(a);
            idx.push(b);
            idx.push(c);
        }
        self.pending_uploads.insert(key.to_string(), (revision, verts, idx));
    }

    fn flush_uploads(&mut self, gl: &glow::Context) {
        let pending = std::mem::take(&mut self.pending_uploads);
        for (key, (rev, verts, indices)) in pending {
            unsafe {
                if let Some(old) = self.slots.remove(&key) {
                    gl.delete_vertex_array(old.vao);
                    gl.delete_buffer(old.vbo);
                    gl.delete_buffer(old.ebo);
                }
                if verts.is_empty() || indices.is_empty() {
                    continue;
                }
                let vao = gl.create_vertex_array().expect("vao");
                let vbo = gl.create_buffer().expect("vbo");
                let ebo = gl.create_buffer().expect("ebo");

                gl.bind_vertex_array(Some(vao));
                gl.bind_buffer(glow::ARRAY_BUFFER, Some(vbo));
                gl.buffer_data_u8_slice(
                    glow::ARRAY_BUFFER,
                    bytemuck::cast_slice(&verts),
                    glow::STATIC_DRAW,
                );

                gl.bind_buffer(glow::ELEMENT_ARRAY_BUFFER, Some(ebo));
                gl.buffer_data_u8_slice(
                    glow::ELEMENT_ARRAY_BUFFER,
                    bytemuck::cast_slice(&indices),
                    glow::STATIC_DRAW,
                );

                let stride = std::mem::size_of::<GpuVertex>() as i32;
                gl.enable_vertex_attrib_array(0);
                gl.vertex_attrib_pointer_f32(0, 3, glow::FLOAT, false, stride, 0);
                gl.enable_vertex_attrib_array(1);
                gl.vertex_attrib_pointer_f32(1, 3, glow::FLOAT, false, stride, 12);
                gl.enable_vertex_attrib_array(2);
                gl.vertex_attrib_pointer_f32(2, 3, glow::FLOAT, false, stride, 24);
                gl.bind_vertex_array(None);

                self.slots.insert(
                    key,
                    GpuMeshSlot {
                        revision: rev,
                        vao,
                        vbo,
                        ebo,
                        index_count: indices.len() as i32,
                    },
                );
            }
        }
    }

    pub fn paint_scene(
        &mut self,
        gl: &glow::Context,
        mvp: Mat4,
        eye: Vec3,
        layers: &[RenderLayerSpec],
        clip_plane: Option<Vec4>,
    ) {
        self.flush_uploads(gl);

        unsafe {
            // 1. Fondo degradado ciruela imperial + limpieza de depth buffer
            gl.disable(glow::DEPTH_TEST);
            gl.depth_mask(false);
            gl.disable(glow::CULL_FACE);
            gl.use_program(Some(self.bg_program));
            let u_bot = gl.get_uniform_location(self.bg_program, "u_bottom");
            let u_top = gl.get_uniform_location(self.bg_program, "u_top");
            gl.uniform_3_f32(u_bot.as_ref(), VIEWPORT_BG_BOTTOM[0], VIEWPORT_BG_BOTTOM[1], VIEWPORT_BG_BOTTOM[2]);
            gl.uniform_3_f32(u_top.as_ref(), VIEWPORT_BG_TOP[0], VIEWPORT_BG_TOP[1], VIEWPORT_BG_TOP[2]);
            gl.bind_vertex_array(Some(self.bg_vao));
            gl.draw_arrays(glow::TRIANGLES, 0, 3);
            gl.bind_vertex_array(None);

            gl.depth_mask(true);
            gl.clear(glow::DEPTH_BUFFER_BIT);
            gl.enable(glow::DEPTH_TEST);
            gl.depth_func(glow::LESS);

            // 2. Renderizar capas: primero opacas, luego transparentes, luego overlays always_on_top
            gl.use_program(Some(self.mesh_program));
            let loc_mvp = gl.get_uniform_location(self.mesh_program, "u_mvp");
            let loc_eye = gl.get_uniform_location(self.mesh_program, "u_eye");
            let loc_tint = gl.get_uniform_location(self.mesh_program, "u_tint");
            let loc_op = gl.get_uniform_location(self.mesh_program, "u_opacity");
            let loc_spec = gl.get_uniform_location(self.mesh_program, "u_specular");
            let loc_shin = gl.get_uniform_location(self.mesh_program, "u_shininess");
            let loc_vcol = gl.get_uniform_location(self.mesh_program, "u_use_vertex_color");
            let loc_unlit = gl.get_uniform_location(self.mesh_program, "u_unlit");
            let loc_clip_en = gl.get_uniform_location(self.mesh_program, "u_clip_enabled");
            let loc_clip_pl = gl.get_uniform_location(self.mesh_program, "u_clip_plane");

            gl.uniform_matrix_4_f32_slice(loc_mvp.as_ref(), false, &mvp.to_cols_array());
            gl.uniform_3_f32(loc_eye.as_ref(), eye.x, eye.y, eye.z);

            let draw_layer = |layer: &RenderLayerSpec| {
                let Some(slot) = self.slots.get(&layer.key) else {
                    return;
                };
                if slot.index_count <= 0 {
                    return;
                }
                gl.uniform_3_f32(loc_tint.as_ref(), layer.tint[0], layer.tint[1], layer.tint[2]);
                gl.uniform_1_f32(loc_op.as_ref(), layer.opacity.clamp(0.0, 1.0));
                gl.uniform_1_f32(loc_spec.as_ref(), layer.specular);
                gl.uniform_1_f32(loc_shin.as_ref(), layer.shininess);
                gl.uniform_1_f32(loc_vcol.as_ref(), if layer.use_vertex_color { 1.0 } else { 0.0 });
                gl.uniform_1_f32(loc_unlit.as_ref(), if layer.unlit { 1.0 } else { 0.0 });

                if layer.clip_affected {
                    if let Some(cp) = clip_plane {
                        gl.uniform_1_f32(loc_clip_en.as_ref(), 1.0);
                        gl.uniform_4_f32(loc_clip_pl.as_ref(), cp.x, cp.y, cp.z, cp.w);
                    } else {
                        gl.uniform_1_f32(loc_clip_en.as_ref(), 0.0);
                    }
                } else {
                    gl.uniform_1_f32(loc_clip_en.as_ref(), 0.0);
                }

                gl.bind_vertex_array(Some(slot.vao));
                gl.draw_elements(glow::TRIANGLES, slot.index_count, glow::UNSIGNED_INT, 0);

                if layer.wireframe {
                    gl.polygon_mode(glow::FRONT_AND_BACK, glow::LINE);
                    gl.enable(glow::POLYGON_OFFSET_LINE);
                    gl.polygon_offset(-1.0, -1.0);
                    gl.uniform_3_f32(loc_tint.as_ref(), 0.35, 0.12, 0.48);
                    gl.uniform_1_f32(loc_vcol.as_ref(), 0.0);
                    gl.uniform_1_f32(loc_unlit.as_ref(), 1.0);
                    gl.draw_elements(glow::TRIANGLES, slot.index_count, glow::UNSIGNED_INT, 0);
                    gl.disable(glow::POLYGON_OFFSET_LINE);
                    gl.polygon_mode(glow::FRONT_AND_BACK, glow::FILL);
                }
                gl.bind_vertex_array(None);
            };

            // Pasada A: Opacas
            gl.disable(glow::BLEND);
            gl.depth_mask(true);
            for l in layers.iter().filter(|l| !l.always_on_top && l.opacity >= 0.99) {
                draw_layer(l);
            }

            // Pasada B: Translúcidas (cemento, grosor mínimo, antagonista, corona semitransparente)
            gl.enable(glow::BLEND);
            gl.blend_func(glow::SRC_ALPHA, glow::ONE_MINUS_SRC_ALPHA);
            gl.depth_mask(false);
            for l in layers.iter().filter(|l| !l.always_on_top && l.opacity < 0.99) {
                draw_layer(l);
            }

            // Pasada C: Controles 3D Always-on-top (Gizmo Transformación Individual / nodos resaltados)
            gl.clear(glow::DEPTH_BUFFER_BIT);
            gl.depth_mask(true);
            for l in layers.iter().filter(|l| l.always_on_top) {
                if l.opacity < 0.99 {
                    gl.enable(glow::BLEND);
                } else {
                    gl.disable(glow::BLEND);
                }
                draw_layer(l);
            }

            // Restaurar estado OpenGL limpio para egui
            gl.depth_mask(true);
            gl.disable(glow::DEPTH_TEST);
            gl.enable(glow::BLEND);
            gl.use_program(None);
        }
    }

    pub fn destroy(&mut self, gl: &glow::Context) {
        unsafe {
            for (_, slot) in self.slots.drain() {
                gl.delete_vertex_array(slot.vao);
                gl.delete_buffer(slot.vbo);
                gl.delete_buffer(slot.ebo);
            }
            gl.delete_vertex_array(self.bg_vao);
            gl.delete_program(self.bg_program);
            gl.delete_program(self.mesh_program);
        }
    }
}

/// Envuelve `GlViewportRenderer::paint_scene` en un `egui::PaintCallback` listo para `ui.painter().add(...)`.
pub fn make_viewport_paint_callback(
    renderer: Arc<Mutex<GlViewportRenderer>>,
    rect: egui::Rect,
    camera: &OrbitCamera,
    layers: Vec<RenderLayerSpec>,
    clip_plane: Option<Vec4>,
) -> egui::PaintCallback {
    let mvp = camera.view_proj(rect);
    let eye = camera.eye_position();
    egui::PaintCallback {
        rect,
        callback: Arc::new(eframe::egui_glow::CallbackFn::new(move |_info, painter| {
            if let Ok(mut guard) = renderer.lock() {
                guard.paint_scene(painter.gl(), mvp, eye, &layers, clip_plane);
            }
        })),
    }
}

fn lerp3(a: [f32; 3], b: [f32; 3], t: f32) -> [f32; 3] {
    let t = t.clamp(0.0, 1.0);
    [
        a[0] + (b[0] - a[0]) * t,
        a[1] + (b[1] - a[1]) * t,
        a[2] + (b[2] - a[2]) * t,
    ]
}

fn sample_ramp(stops: &[[f32; 3]], t: f32) -> [f32; 3] {
    let t = t.clamp(0.0, 1.0);
    let scaled = t * (stops.len() - 1) as f32;
    let i0 = (scaled.floor() as usize).min(stops.len() - 2);
    lerp3(stops[i0], stops[i0 + 1], scaled - i0 as f32)
}

fn build_gpu_vertices(mesh: &TriMeshData, colormap: ColorMapKind) -> Vec<GpuVertex> {
    let n = mesh.positions.len();
    let mut out = Vec::with_capacity(n);

    const RDYLGN: [[f32; 3]; 5] = [
        [0.85, 0.14, 0.18],
        [0.96, 0.45, 0.22],
        [0.99, 0.92, 0.52],
        [0.45, 0.82, 0.42],
        [0.10, 0.65, 0.36],
    ];
    const MAGMA: [[f32; 3]; 5] = [
        [0.05, 0.03, 0.14],
        [0.35, 0.08, 0.48],
        [0.75, 0.22, 0.45],
        [0.98, 0.55, 0.25],
        [0.99, 0.96, 0.72],
    ];
    const JET_R: [[f32; 3]; 5] = [
        [0.95, 0.12, 0.12], // rojo = colisión/contacto fuerte (-0.05 mm)
        [0.98, 0.62, 0.10], // naranja = contacto suave
        [0.24, 0.86, 0.46], // verde = holgura óptima
        [0.16, 0.64, 0.95], // azul claro
        [0.12, 0.22, 0.68], // azul profundo = libre (>1.2 mm)
    ];
    const VIRIDIS: [[f32; 3]; 5] = [
        [0.27, 0.01, 0.33],
        [0.23, 0.32, 0.55],
        [0.13, 0.57, 0.55],
        [0.37, 0.79, 0.38],
        [0.99, 0.91, 0.15],
    ];

    for i in 0..n {
        let p = mesh.positions[i];
        let nm = mesh.normals.get(i).copied().unwrap_or(Vec3::Z);
        let col = match colormap {
            ColorMapKind::Solid => [1.0, 1.0, 1.0],
            ColorMapKind::UndercutRdYlGn => {
                let ang = mesh.custom_scalars.get(i).copied().unwrap_or(15.0);
                let t = ((ang + 25.0) / 60.0).clamp(0.0, 1.0);
                sample_ramp(&RDYLGN, t)
            }
            ColorMapKind::CurvatureMagma => {
                let c = mesh.custom_scalars.get(i).copied().unwrap_or(0.0);
                sample_ramp(&MAGMA, c)
            }
            ColorMapKind::OcclusalJetR => {
                let cl = mesh.occlusal_clearance_mm.get(i).copied().unwrap_or(1.5);
                let t = ((cl + 0.05) / 1.25).clamp(0.0, 1.0);
                sample_ramp(&JET_R, t)
            }
            ColorMapKind::ThicknessViridis => {
                let th = mesh.thickness_mm.get(i).copied().unwrap_or(1.0);
                let t = ((th - 0.20) / 2.00).clamp(0.0, 1.0);
                sample_ramp(&VIRIDIS, t)
            }
        };
        out.push(GpuVertex {
            pos: [p.x, p.y, p.z],
            normal: [nm.x, nm.y, nm.z],
            color: col,
        });
    }
    out
}
