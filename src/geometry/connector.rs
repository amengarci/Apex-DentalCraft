//! Módulo 4 (Puentes y Conectores): Modelado Interactivo de Conectores 3D de 3 Anillos
//! Estilo 3Shape (Nodos sobre Diente A + Nodos Centrales de Ancho/Área + Nodos sobre Diente B),
//! Cálculo Exacto de Área Transversal ($\text{mm}^2$), Cierre de Pónticos y Fusión Watertight sin Pliegues.

use crate::core::case_manager::BridgeConnectorConfig;
use crate::geometry::mesh::{build_orthonormal_basis, smoothstep, MeshCollider, TriMeshData};
use crate::geometry::mesh_ops::{
    close_small_holes_in_mesh, extract_largest_component, extract_ordered_boundary_loops,
};
use glam::{Vec2, Vec3};
use std::collections::{BTreeMap, HashMap, VecDeque};
use std::f32::consts::PI;

#[derive(Debug, Clone)]
pub struct ConnectorFrame {
    pub base_center: Vec3,
    pub center: Vec3,
    pub p_start: Vec3,
    pub p_end: Vec3,
    pub t_axis: Vec3,
    pub u_occ: Vec3,
    pub v_bl: Vec3,
    pub span_mm: f32,
    pub tooth_a_center: Vec3,
    pub tooth_b_center: Vec3,
    pub base_anchor_a: Vec3,
    pub anchor_a: Vec3,
    pub base_anchor_b: Vec3,
    pub anchor_b: Vec3,
    pub ring_a_nodes: [Vec3; 8],
    pub ring_b_nodes: [Vec3; 8],
    pub ring_a_3d: Vec<Vec3>,
    pub ring_c_3d: Vec<Vec3>,
    pub ring_b_3d: Vec<Vec3>,
}

fn proximal_surface_centroid(mesh: &TriMeshData, dir: Vec3) -> Vec3 {
    if mesh.positions.is_empty() {
        return Vec3::ZERO;
    }
    let c = mesh.center();
    let d = if dir.length_squared() > 1e-6 { dir.normalize() } else { Vec3::X };
    let mut projs: Vec<(f32, Vec3)> = mesh
        .positions
        .iter()
        .map(|&p| ((p - c).dot(d), p))
        .collect();
    projs.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap_or(std::cmp::Ordering::Equal));
    let k = ((projs.len() as f32 * 0.14).ceil() as usize).clamp(4, projs.len());
    projs.iter().take(k).map(|x| x.1).sum::<Vec3>() / k as f32
}

/// Proyecta un punto objetivo sobre la cara proximal exterior inicial siguiendo `-outward_dir`.
fn project_onto_proximal_surface(col: &MeshCollider, target: Vec3, outward_dir: Vec3) -> Vec3 {
    let dir = if outward_dir.length_squared() > 1e-6 {
        outward_dir.normalize()
    } else {
        Vec3::X
    };
    let origin = target + dir * 4.5;
    if let Some((_, hit)) = col.cast_ray(origin, -dir, 10.0) {
        if hit.distance(target) <= 5.5 {
            return hit;
        }
    }
    col.project_point(target)
}

fn effective_attach_radii(base_radii: &[f32; 8], fallback_radii: &[f32; 8], end_flare: f32) -> [f32; 8] {
    if base_radii.iter().any(|&r| r <= 0.05) {
        let mul = 1.0 + end_flare.clamp(0.05, 0.35);
        let mut out = [1.8_f32; 8];
        for k in 0..8 {
            out[k] = (fallback_radii[k] * mul).clamp(0.65, 5.2);
        }
        out
    } else {
        *base_radii
    }
}

fn effective_scale(raw: f32) -> f32 {
    if raw <= 0.01 {
        1.0
    } else {
        raw.clamp(0.55, 1.85)
    }
}

fn effective_profile_nodes_2d(cfg: &BridgeConnectorConfig) -> [Vec2; 8] {
    let scale = effective_scale(cfg.thickness_scale);
    let mut out = [Vec2::ZERO; 8];
    for k in 0..8 {
        let [du, dv] = cfg.profile_nodes_2d[k];
        if du * du + dv * dv > 1e-4 {
            let p = Vec2::new(du * scale, dv * scale);
            let len = p.length();
            out[k] = if len < 0.35 {
                p.normalize_or_zero() * 0.35
            } else if len > 6.5 {
                p / len * 6.5
            } else {
                p
            };
        } else {
            let th = (k as f32) * std::f32::consts::FRAC_PI_4;
            let r = (cfg.profile_radii_mm[k] * scale).clamp(0.45, 5.8);
            out[k] = Vec2::new(th.cos() * r, th.sin() * r);
        }
    }
    out
}

fn effective_attach_nodes_local(
    nodes: &[[f32; 3]; 8],
    radii: &[f32; 8],
    fallback_radii: &[f32; 8],
    end_flare: f32,
) -> ([Vec3; 8], [bool; 8]) {
    let eff_r = effective_attach_radii(radii, fallback_radii, end_flare);
    let mut out = [Vec3::ZERO; 8];
    let mut is_free = [false; 8];
    for k in 0..8 {
        let [dx, dy, dz] = nodes[k];
        if dx * dx + dy * dy + dz * dz > 1e-4 {
            out[k] = Vec3::new(dx, dy, dz);
            is_free[k] = true;
        } else {
            let th = (k as f32) * std::f32::consts::FRAC_PI_4;
            let r = eff_r[k].clamp(0.45, 5.8);
            out[k] = Vec3::new(th.cos() * r, th.sin() * r, 0.0);
        }
    }
    (out, is_free)
}

/// Construye el anillo de unión de 8 nodos libres 3D + 48 puntos suavizados adheridos a cualquier cara del diente.
fn build_surface_contour_3d(
    col: Option<&MeshCollider>,
    anchor: Vec3,
    outward_dir: Vec3,
    u_occ: Vec3,
    v_bl: Vec3,
    nodes_local: &[Vec3; 8],
    is_free_3d: &[bool; 8],
    anchor_moved: bool,
    scale: f32,
) -> ([Vec3; 8], Vec<Vec3>) {
    let eff_scale = effective_scale(scale);
    let mut nodes_3d = [Vec3::ZERO; 8];
    for k in 0..8 {
        let loc = nodes_local[k] * eff_scale;
        let raw_pt = anchor + u_occ * loc.x + v_bl * loc.y + outward_dir * loc.z;
        nodes_3d[k] = if let Some(c) = col {
            if is_free_3d[k] || anchor_moved {
                c.project_point(raw_pt)
            } else {
                project_onto_proximal_surface(c, raw_pt, outward_dir)
            }
        } else {
            raw_pt
        };
    }

    let n_ctrl = 8usize;
    let n_out = 48usize;
    let mut curve = Vec::with_capacity(n_out);
    for s in 0..n_out {
        let u = (s as f32) / (n_out as f32) * (n_ctrl as f32);
        let seg = (u.floor() as usize) % n_ctrl;
        let t = u - u.floor();
        let p0 = nodes_3d[(seg + n_ctrl - 1) % n_ctrl];
        let p1 = nodes_3d[seg];
        let p2 = nodes_3d[(seg + 1) % n_ctrl];
        let p3 = nodes_3d[(seg + 2) % n_ctrl];
        let t2 = t * t;
        let t3 = t2 * t;
        let pt = 0.5
            * ((2.0 * p1)
                + (-p0 + p2) * t
                + (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * t2
                + (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * t3);
        let snapped = if let Some(c) = col {
            c.project_point(pt)
        } else {
            pt
        };
        curve.push(snapped);
    }

    // Dos pasadas suaves + re-proyección más cercana sobre cualquier flanco del diente
    for _ in 0..2 {
        let prev = curve.clone();
        for s in 0..n_out {
            let p_prev = prev[(s + n_out - 1) % n_out];
            let p_next = prev[(s + 1) % n_out];
            let smooth = p_prev * 0.20 + prev[s] * 0.60 + p_next * 0.20;
            curve[s] = if let Some(c) = col {
                c.project_point(smooth)
            } else {
                smooth
            };
        }
    }

    (nodes_3d, curve)
}

/// Calcula el marco anatómico de 3 anillos (`Ring A`, `Centro`, `Ring B`) reutilizando BVH ya construidos.
pub fn compute_connector_frame_with_colliders(
    mesh_a: &TriMeshData,
    col_a: Option<&MeshCollider>,
    mesh_b: &TriMeshData,
    col_b: Option<&MeshCollider>,
    insertion_axis: Vec3,
    cfg: &BridgeConnectorConfig,
) -> ConnectorFrame {
    let ca = mesh_a.center();
    let cb = mesh_b.center();
    let mut t_axis = (cb - ca).normalize_or_zero();
    if t_axis.length_squared() < 1e-6 {
        t_axis = Vec3::X;
    }

    let pa_prox = proximal_surface_centroid(mesh_a, t_axis);
    let pb_prox = proximal_surface_centroid(mesh_b, -t_axis);
    let base_center = 0.5 * (pa_prox + pb_prox);
    let center = base_center + Vec3::from_array(cfg.center_offset);

    let ax = if insertion_axis.length_squared() > 1e-6 {
        insertion_axis.normalize()
    } else {
        Vec3::Z
    };
    let u_raw = ax - t_axis * ax.dot(t_axis);
    let u_occ = if u_raw.length_squared() > 1e-5 {
        u_raw.normalize()
    } else {
        build_orthonormal_basis(t_axis).0
    };
    // Base dextrógira: t_axis × u_occ = v_bl  =>  (u_occ, v_bl, t_axis) mano derecha
    let v_bl = t_axis.cross(u_occ).normalize_or_zero();

    let base_anchor_a = if let Some(c) = col_a {
        project_onto_proximal_surface(c, pa_prox, t_axis)
    } else {
        pa_prox
    };
    let off_a = Vec3::from_array(cfg.attach_a_offset);
    let a_moved = off_a.length_squared() > 1e-6;
    let raw_anchor_a = base_anchor_a + off_a;
    let anchor_a = if let Some(c) = col_a {
        if a_moved {
            c.project_point(raw_anchor_a)
        } else {
            project_onto_proximal_surface(c, raw_anchor_a, t_axis)
        }
    } else {
        raw_anchor_a
    };

    let base_anchor_b = if let Some(c) = col_b {
        project_onto_proximal_surface(c, pb_prox, -t_axis)
    } else {
        pb_prox
    };
    let off_b = Vec3::from_array(cfg.attach_b_offset);
    let b_moved = off_b.length_squared() > 1e-6;
    let raw_anchor_b = base_anchor_b + off_b;
    let anchor_b = if let Some(c) = col_b {
        if b_moved {
            c.project_point(raw_anchor_b)
        } else {
            project_onto_proximal_surface(c, raw_anchor_b, -t_axis)
        }
    } else {
        raw_anchor_b
    };

    let (nodes_a_local, free_a) = effective_attach_nodes_local(
        &cfg.attach_a_nodes_local,
        &cfg.attach_a_radii_mm,
        &cfg.profile_radii_mm,
        cfg.end_flare,
    );
    let (nodes_b_local, free_b) = effective_attach_nodes_local(
        &cfg.attach_b_nodes_local,
        &cfg.attach_b_radii_mm,
        &cfg.profile_radii_mm,
        cfg.end_flare,
    );
    let scale_a = effective_scale(cfg.attach_a_scale);
    let scale_b = effective_scale(cfg.attach_b_scale);

    let (ring_a_nodes, ring_a_3d) = build_surface_contour_3d(
        col_a,
        anchor_a,
        t_axis,
        u_occ,
        v_bl,
        &nodes_a_local,
        &free_a,
        a_moved,
        scale_a,
    );
    let (ring_b_nodes, ring_b_3d) = build_surface_contour_3d(
        col_b,
        anchor_b,
        -t_axis,
        u_occ,
        v_bl,
        &nodes_b_local,
        &free_b,
        b_moved,
        scale_b,
    );

    let ring_c_3d: Vec<Vec3> = sample_profile_2d(cfg, 48)
        .into_iter()
        .map(|p| center + u_occ * p.x + v_bl * p.y)
        .collect();

    let span_mm = anchor_a.distance(anchor_b).max(2.0);
    let p_start = anchor_a - t_axis * 0.35;
    let p_end = anchor_b + t_axis * 0.35;

    ConnectorFrame {
        base_center,
        center,
        p_start,
        p_end,
        t_axis,
        u_occ,
        v_bl,
        span_mm,
        tooth_a_center: ca,
        tooth_b_center: cb,
        base_anchor_a,
        anchor_a,
        base_anchor_b,
        anchor_b,
        ring_a_nodes,
        ring_b_nodes,
        ring_a_3d,
        ring_c_3d,
        ring_b_3d,
    }
}

/// Calcula el marco anatómico de 3 anillos (`Ring A sobre Diente A`, `Centro`, `Ring B sobre Diente B`).
pub fn compute_connector_frame(
    mesh_a: &TriMeshData,
    mesh_b: &TriMeshData,
    insertion_axis: Vec3,
    cfg: &BridgeConnectorConfig,
) -> ConnectorFrame {
    let col_a = MeshCollider::from_mesh(mesh_a);
    let col_b = MeshCollider::from_mesh(mesh_b);
    compute_connector_frame_with_colliders(
        mesh_a,
        col_a.as_ref(),
        mesh_b,
        col_b.as_ref(),
        insertion_axis,
        cfg,
    )
}

/// Evalúa la curva cerrada suave (Catmull-Rom periódica) de los 8 nodos libres 2D del perfil central (`u_occ`, `v_bl`).
pub fn sample_profile_2d(cfg: &BridgeConnectorConfig, n_samples: usize) -> Vec<Vec2> {
    let n_ctrl = 8usize;
    let ctrl_pts = effective_profile_nodes_2d(cfg);

    let n_out = n_samples.max(16);
    let mut out = Vec::with_capacity(n_out);
    for s in 0..n_out {
        let u = (s as f32) / (n_out as f32) * (n_ctrl as f32);
        let seg = (u.floor() as usize) % n_ctrl;
        let t = u - u.floor();
        let p0 = ctrl_pts[(seg + n_ctrl - 1) % n_ctrl];
        let p1 = ctrl_pts[seg];
        let p2 = ctrl_pts[(seg + 1) % n_ctrl];
        let p3 = ctrl_pts[(seg + 2) % n_ctrl];
        let t2 = t * t;
        let t3 = t2 * t;
        let pt = 0.5
            * ((2.0 * p1)
                + (-p0 + p2) * t
                + (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * t2
                + (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * t3);
        out.push(pt);
    }
    out
}

/// Área transversal exacta en $\text{mm}^2$ en el cuello central del conector (Fórmula de Gauss / Shoelace).
pub fn compute_cross_section_area_mm2(cfg: &BridgeConnectorConfig) -> f32 {
    let pts = sample_profile_2d(cfg, 64);
    let n = pts.len();
    if n < 3 {
        return 0.0;
    }
    let mut acc = 0.0_f64;
    for i in 0..n {
        let a = pts[i].as_dvec2();
        let b = pts[(i + 1) % n].as_dvec2();
        acc += a.x * b.y - b.x * a.y;
    }
    (0.5 * acc).abs() as f32
}

/// Altura ocluso-gingival y ancho vestíbulo-lingual del perfil del conector central (mm).
pub fn compute_profile_dimensions_mm(cfg: &BridgeConnectorConfig) -> (f32, f32) {
    let pts = sample_profile_2d(cfg, 64);
    let mut min_u = f32::INFINITY;
    let mut max_u = f32::NEG_INFINITY;
    let mut min_v = f32::INFINITY;
    let mut max_v = f32::NEG_INFINITY;
    for p in pts {
        min_u = min_u.min(p.x);
        max_u = max_u.max(p.x);
        min_v = min_v.min(p.y);
        max_v = max_v.max(p.y);
    }
    ((max_u - min_u).max(0.0), (max_v - min_v).max(0.0))
}

/// Posiciones 3D en mundo de los 8 nodos perimetrales libres del cuello central y del nodo central.
pub fn control_node_positions_3d(frame: &ConnectorFrame, cfg: &BridgeConnectorConfig) -> ([Vec3; 8], Vec3) {
    let p2d = effective_profile_nodes_2d(cfg);
    let mut nodes = [Vec3::ZERO; 8];
    for k in 0..8 {
        nodes[k] = frame.center + frame.u_occ * p2d[k].x + frame.v_bl * p2d[k].y;
    }
    (nodes, frame.center)
}

/// Puntos 3D del anillo ecuatorial (cintura central) del conector para resaltado visual.
pub fn build_connector_waist_ring_3d(frame: &ConnectorFrame, _cfg: &BridgeConnectorConfig) -> Vec<Vec3> {
    frame.ring_c_3d.clone()
}

/// Construye un conector anatómico 3D cerrado que interpola suavemente los 3 anillos (`Ring A -> Ring C -> Ring B`).
pub fn build_connector_mesh(frame: &ConnectorFrame, _cfg: &BridgeConnectorConfig) -> TriMeshData {
    let sides = frame.ring_c_3d.len().max(16);
    let n_rings = 21usize;

    let in_ctr_a = {
        let d = (frame.tooth_a_center - frame.anchor_a).normalize_or_zero();
        if d.length_squared() > 1e-6 { d } else { -frame.t_axis }
    };
    let in_ctr_b = {
        let d = (frame.tooth_b_center - frame.anchor_b).normalize_or_zero();
        if d.length_squared() > 1e-6 { d } else { frame.t_axis }
    };

    let ring_a: Vec<Vec3> = if frame.ring_a_3d.len() == sides {
        frame
            .ring_a_3d
            .iter()
            .map(|&p| {
                let d = (frame.tooth_a_center - p).normalize_or_zero();
                let in_dir = if d.length_squared() > 1e-6 { d } else { -frame.t_axis };
                p + in_dir * 0.35
            })
            .collect()
    } else {
        frame.ring_c_3d.iter().map(|&p| p - frame.t_axis * 2.0).collect()
    };
    let ring_b: Vec<Vec3> = if frame.ring_b_3d.len() == sides {
        frame
            .ring_b_3d
            .iter()
            .map(|&p| {
                let d = (frame.tooth_b_center - p).normalize_or_zero();
                let in_dir = if d.length_squared() > 1e-6 { d } else { frame.t_axis };
                p + in_dir * 0.35
            })
            .collect()
    } else {
        frame.ring_c_3d.iter().map(|&p| p + frame.t_axis * 2.0).collect()
    };

    let mut positions = Vec::with_capacity(n_rings * sides + 2);
    let mut indices = Vec::with_capacity((n_rings - 1) * sides * 2 + sides * 2);

    for ir in 0..n_rings {
        let t = (ir as f32) / ((n_rings - 1) as f32);
        let omt = 1.0 - t;
        for s in 0..sides {
            let p0 = ring_a[s];
            let pm = frame.ring_c_3d[s];
            let p2 = ring_b[s];
            let b1 = 2.0 * pm - 0.5 * (p0 + p2);
            let mut pt = p0 * (omt * omt) + b1 * (2.0 * omt * t) + p2 * (t * t);
            // Bisel redondeado sutil en extremos de inserción dentro de las coronas
            let edge = (2.0 * t - 1.0).abs();
            if edge > 0.90 {
                let end_ctr = frame.anchor_a * omt + frame.anchor_b * t;
                let shrink = (1.0 - ((edge - 0.90) / 0.10).powi(2) * 0.18).clamp(0.75, 1.0);
                pt = end_ctr + (pt - end_ctr) * shrink;
            }
            positions.push(pt);
        }
    }

    for ir in 0..(n_rings - 1) {
        let base_a = (ir * sides) as u32;
        let base_b = ((ir + 1) * sides) as u32;
        for s in 0..sides {
            let ns = (s + 1) % sides;
            let a0 = base_a + s as u32;
            let a1 = base_a + ns as u32;
            let b0 = base_b + s as u32;
            let b1 = base_b + ns as u32;
            indices.push([a0, a1, b1]);
            indices.push([a0, b1, b0]);
        }
    }

    let c_start = positions.len() as u32;
    positions.push(frame.anchor_a + in_ctr_a * 0.48);
    let c_end = positions.len() as u32;
    positions.push(frame.anchor_b + in_ctr_b * 0.48);
    let last_base = ((n_rings - 1) * sides) as u32;
    for s in 0..sides {
        let ns = (s + 1) % sides;
        indices.push([c_start, ns as u32, s as u32]);
        indices.push([c_end, last_base + s as u32, last_base + ns as u32]);
    }

    let mut out = TriMeshData::new(positions, indices);
    out.ensure_outward_winding_if_closed();
    out
}

/// Garantiza que un póntico (que no tiene margen cervical) sea un sólido cerrado 100% watertight.
pub fn ensure_pontic_closed(mesh: &TriMeshData) -> TriMeshData {
    if mesh.is_empty() {
        return mesh.clone();
    }
    let audit = mesh.inspect_cam();
    if audit.watertight {
        let mut out = mesh.clone();
        out.ensure_outward_winding_if_closed();
        return out;
    }
    let (mut closed, _) = close_small_holes_in_mesh(mesh, false);
    closed.ensure_outward_winding_if_closed();
    closed
}

/// Zipper topológico cíclico entre `ring_a` (borde abierto del diente) y `ring_b` (anillo del tubo conector).
fn zipper_stitch_loops(
    positions: &[Vec3],
    ring_a: &[u32],
    ring_b: &[u32],
) -> Vec<[u32; 3]> {
    let na = ring_a.len();
    let nb = ring_b.len();
    if na < 3 || nb < 3 {
        return Vec::new();
    }
    let mut tris = Vec::with_capacity(na + nb);
    let mut ia = 0usize;
    let mut jb = 0usize;
    while ia < na || jb < nb {
        let ac = ring_a[ia % na];
        let bc = ring_b[jb % nb];
        let an = ring_a[(ia + 1) % na];
        let bn = ring_b[(jb + 1) % nb];

        let take_a = if ia == na {
            false
        } else if jb == nb {
            true
        } else {
            let d_a = positions[an as usize].distance_squared(positions[bc as usize]);
            let d_b = positions[ac as usize].distance_squared(positions[bn as usize]);
            d_a <= d_b
        };

        if take_a {
            tris.push([an, ac, bc]);
            ia += 1;
        } else {
            tris.push([ac, bc, bn]);
            jb += 1;
        }
    }
    tris
}

fn orient_and_rotate_loop_for_zipper(
    mesh: &TriMeshData,
    loop_verts: &mut Vec<u32>,
    origin: Vec3,
    u_occ: Vec3,
    _v_bl: Vec3,
) {
    if loop_verts.len() < 3 {
        return;
    }
    let best_start = loop_verts
        .iter()
        .enumerate()
        .max_by(|(_, &va), (_, &vb)| {
            let da = (mesh.positions[va as usize] - origin).dot(u_occ);
            let db = (mesh.positions[vb as usize] - origin).dot(u_occ);
            da.partial_cmp(&db).unwrap_or(std::cmp::Ordering::Equal)
        })
        .map(|(idx, _)| idx)
        .unwrap_or(0);
    loop_verts.rotate_left(best_start);
}

/// Muestrea una polilínea cerrada 3D exactamente en los `n_out` ángulos CCW del plano `(u_occ, v_bl)`
/// para que cada anillo extremo del conector siga de cerca el agujero perforado en la superficie curva.
fn sample_closed_3d_loop_at_angles(
    pts: &[Vec3],
    origin: Vec3,
    u_occ: Vec3,
    v_bl: Vec3,
    n_out: usize,
) -> Vec<Vec3> {
    let m = pts.len();
    if m < 3 || n_out == 0 {
        return Vec::new();
    }
    let p2: Vec<Vec2> = pts
        .iter()
        .map(|&p| {
            let d = p - origin;
            Vec2::new(d.dot(u_occ), d.dot(v_bl))
        })
        .collect();

    let mut out = Vec::with_capacity(n_out);
    for k in 0..n_out {
        let th = (k as f32) * (2.0 * PI / n_out as f32);
        let dir = Vec2::new(th.cos(), th.sin());
        let mut best_t = f32::INFINITY;
        let mut best_p3 = None;

        for i in 0..m {
            let j = (i + 1) % m;
            let a = p2[i];
            let b = p2[j];
            let ab = b - a;
            // Resolver t * dir = a + s * ab => t * (dir × ab) = (a × ab), s * (dir × ab) = (a × dir)
            let denom = dir.x * ab.y - dir.y * ab.x;
            if denom.abs() > 1e-6 {
                let s = (a.x * dir.y - a.y * dir.x) / denom;
                let t = (a.x * ab.y - a.y * ab.x) / denom;
                if s >= -0.02 && s <= 1.02 && t > 0.05 && t < best_t {
                    best_t = t;
                    best_p3 = Some(pts[i].lerp(pts[j], s.clamp(0.0, 1.0)));
                }
            }
        }

        if let Some(p) = best_p3 {
            out.push(p);
        } else {
            // Respaldo angular más cercano si ningún rayo cruzó por borde paralelo
            let fallback = pts
                .iter()
                .zip(p2.iter())
                .max_by(|(_, a), (_, b)| {
                    let da = a.normalize_or_zero().dot(dir);
                    let db = b.normalize_or_zero().dot(dir);
                    da.partial_cmp(&db).unwrap_or(std::cmp::Ordering::Equal)
                })
                .map(|(&p3, _)| p3)
                .unwrap_or(pts[0]);
            out.push(fallback);
        }
    }

    // Suavizado cíclico gentil para absorber dientes triangulares locales del contorno perforado
    let prev = out.clone();
    for k in 0..n_out {
        let p_prev = prev[(k + n_out - 1) % n_out];
        let p_next = prev[(k + 1) % n_out];
        out[k] = p_prev * 0.22 + prev[k] * 0.56 + p_next * 0.22;
    }
    out
}

#[allow(clippy::too_many_arguments)]
fn build_adaptive_bridge_tube(
    sampled_a: &[Vec3],
    sampled_b: &[Vec3],
    opt_ring_c: Option<&[Vec3]>,
    t_axis: Vec3,
) -> TriMeshData {
    let sides = sampled_a
        .len()
        .min(sampled_b.len())
        .min(opt_ring_c.map(|r| r.len()).unwrap_or(usize::MAX));
    if sides < 4 {
        return TriMeshData::default();
    }
    let mut start_ring = Vec::with_capacity(sides);
    let mut mid_ring = Vec::with_capacity(sides);
    let mut end_ring = Vec::with_capacity(sides);
    let mut max_gap = 0.0_f32;

    for k in 0..sides {
        let pa = sampled_a[k];
        let pb = sampled_b[k];
        let chord = pb - pa;
        let gap_k = chord.length();
        max_gap = max_gap.max(gap_k);
        let span_k = chord.dot(t_axis);

        let mut p0 = pa.lerp(pb, 0.16);
        let mut p2 = pb.lerp(pa, 0.16);
        let chord_mid = 0.5 * (p0 + p2);

        let pm = if let Some(rc) = opt_ring_c {
            if span_k > 0.55 {
                let blend = smoothstep(0.55, 1.25, span_k);
                if span_k > 0.72 {
                    let adv_a = (p0 - pa).dot(t_axis);
                    if adv_a < 0.24 {
                        p0 = pa + t_axis * (0.24_f32.min(span_k * 0.30));
                    }
                    let adv_b = (p2 - pb).dot(-t_axis);
                    if adv_b < 0.24 {
                        p2 = pb + (-t_axis) * (0.24_f32.min(span_k * 0.30));
                    }
                }
                chord_mid.lerp(rc[k], blend)
            } else {
                chord_mid
            }
        } else {
            chord_mid
        };

        start_ring.push(p0);
        mid_ring.push(pm);
        end_ring.push(p2);
    }

    let n_rings = if max_gap < 0.80 { 5usize } else { 13usize };
    let mut positions = Vec::with_capacity(n_rings * sides);
    let mut indices = Vec::with_capacity((n_rings - 1) * sides * 2);
    for ir in 0..n_rings {
        let t = (ir as f32) / ((n_rings - 1) as f32);
        let omt = 1.0 - t;
        for k in 0..sides {
            let p0 = start_ring[k];
            let pm = mid_ring[k];
            let p2 = end_ring[k];
            let b1 = 2.0 * pm - 0.5 * (p0 + p2);
            positions.push(p0 * (omt * omt) + b1 * (2.0 * omt * t) + p2 * (t * t));
        }
    }

    for ir in 0..(n_rings - 1) {
        let base_a = (ir * sides) as u32;
        let base_b = ((ir + 1) * sides) as u32;
        for s in 0..sides {
            let ns = (s + 1) % sides;
            let a0 = base_a + s as u32;
            let a1 = base_a + ns as u32;
            let b0 = base_b + s as u32;
            let b1 = base_b + ns as u32;
            indices.push([a0, a1, b1]);
            indices.push([a0, b1, b0]);
        }
    }

    TriMeshData::new(positions, indices)
}

fn aabb_dist(p: Vec3, mn: Vec3, mx: Vec3) -> f32 {
    let dx = (mn.x - p.x).max(0.0).max(p.x - mx.x);
    let dy = (mn.y - p.y).max(0.0).max(p.y - mx.y);
    let dz = (mn.z - p.z).max(0.0).max(p.z - mx.z);
    (dx * dx + dy * dy + dz * dz).sqrt()
}

fn point_in_poly_2d(p: Vec2, poly: &[Vec2]) -> bool {
    if poly.len() < 3 {
        return false;
    }
    let mut inside = false;
    let n = poly.len();
    let mut j = n - 1;
    for i in 0..n {
        let a = poly[i];
        let b = poly[j];
        let cond = (a.y > p.y) != (b.y > p.y);
        if cond {
            let x_int = (b.x - a.x) * (p.y - a.y) / (b.y - a.y + 1e-12) + a.x;
            if p.x < x_int {
                inside = !inside;
            }
        }
        j = i;
    }
    inside
}

fn min_dist_to_poly_2d(p: Vec2, poly: &[Vec2]) -> f32 {
    let n = poly.len();
    if n < 2 {
        return f32::INFINITY;
    }
    let mut best = f32::INFINITY;
    for i in 0..n {
        let a = poly[i];
        let b = poly[(i + 1) % n];
        let ab = b - a;
        let den = ab.length_squared();
        let t = if den > 1e-10 {
            ((p - a).dot(ab) / den).clamp(0.0, 1.0)
        } else {
            0.0
        };
        best = best.min(p.distance(a + ab * t));
    }
    best
}

#[inline]
fn aabb_gap(a_mn: Vec3, a_mx: Vec3, b_mn: Vec3, b_mx: Vec3) -> f32 {
    let dx = (a_mn.x - b_mx.x).max(b_mn.x - a_mx.x).max(0.0);
    let dy = (a_mn.y - b_mx.y).max(b_mn.y - a_mx.y).max(0.0);
    let dz = (a_mn.z - b_mx.z).max(b_mn.z - a_mx.z).max(0.0);
    (dx * dx + dy * dy + dz * dz).sqrt()
}

/// Abre un disco limpio único que unifica tanto la zona del **conector** (`opt_contour_3d`)
/// como cualquier zona de **contacto/solape interproximal** (< 0.16 mm o dentro de `other_col`),
/// preservando intactos los bordes previos, el intaglio y los márgenes cervicales.
#[allow(clippy::too_many_arguments)]
fn punch_proximal_contact_or_connector_disk(
    unit_mesh: &TriMeshData,
    other_mn: Vec3,
    other_mx: Vec3,
    other_col: Option<&MeshCollider>,
    other_center: Vec3,
    anchor: Vec3,
    outward_dir: Vec3,
    u_occ: Vec3,
    v_bl: Vec3,
    opt_contour_3d: Option<&[Vec3]>,
    is_pontic: bool,
) -> Option<(TriMeshData, Vec<u32>, usize)> {
    let clean = unit_mesh.weld_vertices_preserve_watertight(1e-4);
    let nf = clean.indices.len();
    if nf < 12 {
        return None;
    }
    let existing_loops = extract_ordered_boundary_loops(&clean);
    let existing_boundary_pts: Vec<Vec3> = existing_loops
        .iter()
        .flatten()
        .map(|&vi| clean.positions[vi as usize])
        .collect();

    let (mn, mx) = clean.bounds();
    let unit_ctr = clean.center();
    let occ_span = (mx - mn).dot(u_occ).abs().max(4.0);
    let prox_dir = {
        let d = (other_center - unit_ctr).normalize_or_zero();
        if d.length_squared() > 1e-6 { d } else { outward_dir }
    };
    let anchor_dir = {
        let d = (anchor - unit_ctr).normalize_or_zero();
        if d.length_squared() > 1e-6 && d.dot(outward_dir) < 0.70 {
            d
        } else {
            outward_dir
        }
    };
    let lu_raw = u_occ - anchor_dir * u_occ.dot(anchor_dir);
    let lu = if lu_raw.length_squared() > 1e-5 {
        lu_raw.normalize()
    } else {
        build_orthonormal_basis(anchor_dir).0
    };
    let lv = anchor_dir.cross(lu).normalize_or_zero();

    let poly2d: Option<Vec<Vec2>> = opt_contour_3d.filter(|c| c.len() >= 8).map(|c| {
        c.iter()
            .map(|&p| {
                let r = p - anchor;
                Vec2::new(r.dot(lu), r.dot(lv))
            })
            .collect()
    });
    let _ = v_bl;

    let mut face_centers = Vec::with_capacity(nf);
    let mut face_normals = Vec::with_capacity(nf);
    for &[a, b, c] in &clean.indices {
        let pa = clean.positions[a as usize];
        let pb = clean.positions[b as usize];
        let pc = clean.positions[c as usize];
        face_centers.push((pa + pb + pc) / 3.0);
        face_normals.push((pb - pa).cross(pc - pa).normalize_or_zero());
    }

    let check_contact = other_col.is_some() && aabb_gap(mn, mx, other_mn, other_mx) <= 0.22;
    let mut eligible = vec![false; nf];
    let mut contact_mask = vec![false; nf];
    let mut conn_mask = vec![false; nf];
    let mut raw = vec![false; nf];
    let mut best_contact_seed: Option<(usize, f32)> = None;
    let mut best_conn_seed: Option<(usize, f32)> = None;

    for fi in 0..nf {
        let tri = clean.indices[fi];
        let touches_existing_hole = tri.iter().any(|&vi| {
            let vp = clean.positions[vi as usize];
            existing_boundary_pts
                .iter()
                .any(|&bp| bp.distance_squared(vp) < 0.08 * 0.08)
        });
        if touches_existing_hole {
            continue;
        }
        if !is_pontic {
            let all_ext = clean.is_external.get(tri[0] as usize).copied().unwrap_or(true)
                && clean.is_external.get(tri[1] as usize).copied().unwrap_or(true)
                && clean.is_external.get(tri[2] as usize).copied().unwrap_or(true);
            if !all_ext {
                continue;
            }
        }
        eligible[fi] = true;

        let fc = face_centers[fi];
        let fnm = face_normals[fi];

        if check_contact {
            if let Some(ocol) = other_col {
                let sample_pts = [
                    fc,
                    clean.positions[tri[0] as usize],
                    clean.positions[tri[1] as usize],
                    clean.positions[tri[2] as usize],
                ];
                let mut is_touch = false;
                let mut best_q = f32::INFINITY;
                for &q in &sample_pts {
                    let db = aabb_dist(q, other_mn, other_mx);
                    if db > 0.18 {
                        continue;
                    }
                    if db == 0.0 && ocol.contains_point_closed(q) {
                        is_touch = true;
                        best_q = -1.0;
                        break;
                    }
                    let d = ocol.closest_point_and_dist(q).1;
                    if d < 0.16 && (fc - unit_ctr).dot(prox_dir) > -0.25 {
                        is_touch = true;
                        best_q = best_q.min(d);
                    }
                }
                if is_touch {
                    contact_mask[fi] = true;
                    raw[fi] = true;
                    let score = best_q - 0.15 * (fc - unit_ctr).dot(prox_dir);
                    if best_contact_seed.map(|(_, bs)| score < bs).unwrap_or(true) {
                        best_contact_seed = Some((fi, score));
                    }
                }
            }
        }

        if let Some(p2_poly) = &poly2d {
            if fnm.dot(anchor_dir) >= -0.15
                && (fc - unit_ctr).dot(anchor_dir) >= 0.08
                && (fc - anchor).dot(anchor_dir) >= -2.2
            {
                let cerv_guard = !is_pontic
                    && outward_dir.dot(u_occ).abs() < 0.5
                    && (fc - anchor).dot(u_occ) < -(occ_span * 0.42);
                if !cerv_guard {
                    let rel = fc - anchor;
                    let pt2 = Vec2::new(rel.dot(lu), rel.dot(lv));
                    let inside = point_in_poly_2d(pt2, p2_poly);
                    let edge_d = min_dist_to_poly_2d(pt2, p2_poly);
                    if inside || (rel.dot(anchor_dir) > -0.15 && edge_d <= 0.14) {
                        conn_mask[fi] = true;
                        raw[fi] = true;
                        let score = pt2.length() - 0.18 * rel.dot(anchor_dir);
                        if best_conn_seed.map(|(_, bs)| score < bs).unwrap_or(true) {
                            best_conn_seed = Some((fi, score));
                        }
                    }
                }
            }
        }
    }

    let (seed_fi, _) = best_contact_seed.or(best_conn_seed)?;

    let mut e2f: HashMap<(u32, u32), Vec<usize>> = HashMap::with_capacity(nf * 3);
    for (fi, &[a, b, c]) in clean.indices.iter().enumerate() {
        for (u, v) in [(a, b), (b, c), (c, a)] {
            let k = if u < v { (u, v) } else { (v, u) };
            e2f.entry(k).or_default().push(fi);
        }
    }

    // Etiquetar componentes conexas de `raw`; si el contacto y el conector están ligeramente separados,
    // puentearlos sobre caras proximales externas válidas para abrir un solo disco limpio.
    let mut comp_id = vec![usize::MAX; nf];
    let mut comps: Vec<Vec<usize>> = Vec::new();
    for start in 0..nf {
        if !raw[start] || comp_id[start] != usize::MAX {
            continue;
        }
        let cid = comps.len();
        let mut list = Vec::new();
        let mut q = VecDeque::new();
        comp_id[start] = cid;
        q.push_back(start);
        while let Some(fi) = q.pop_front() {
            list.push(fi);
            let [a, b, c] = clean.indices[fi];
            for (u, v) in [(a, b), (b, c), (c, a)] {
                let k = if u < v { (u, v) } else { (v, u) };
                if let Some(neighs) = e2f.get(&k) {
                    for &nj in neighs {
                        if raw[nj] && comp_id[nj] == usize::MAX {
                            comp_id[nj] = cid;
                            q.push_back(nj);
                        }
                    }
                }
            }
        }
        comps.push(list);
    }

    let main_cid = comp_id[seed_fi];
    if comps.len() > 1 {
        let mut parent = vec![usize::MAX; nf];
        let mut visited = vec![false; nf];
        let mut bfs = VecDeque::new();
        for &fi in &comps[main_cid] {
            visited[fi] = true;
            parent[fi] = fi;
            bfs.push_back(fi);
        }
        let mut found_targets: HashMap<usize, usize> = HashMap::new();
        while let Some(fi) = bfs.pop_front() {
            if comp_id[fi] != usize::MAX && comp_id[fi] != main_cid {
                found_targets.entry(comp_id[fi]).or_insert(fi);
                if found_targets.len() == comps.len() - 1 {
                    break;
                }
            }
            let [a, b, c] = clean.indices[fi];
            for (u, v) in [(a, b), (b, c), (c, a)] {
                let k = if u < v { (u, v) } else { (v, u) };
                if let Some(neighs) = e2f.get(&k) {
                    for &nj in neighs {
                        if !visited[nj] && eligible[nj] {
                            let fc = face_centers[nj];
                            if raw[nj] || (fc - unit_ctr).dot(prox_dir) > -0.25 {
                                visited[nj] = true;
                                parent[nj] = fi;
                                bfs.push_back(nj);
                            }
                        }
                    }
                }
            }
        }
        for (_cid, mut cur) in found_targets {
            while cur != parent[cur] && parent[cur] != usize::MAX {
                raw[cur] = true;
                let [a, b, c] = clean.indices[cur];
                for (u, v) in [(a, b), (b, c), (c, a)] {
                    let k = if u < v { (u, v) } else { (v, u) };
                    if let Some(neighs) = e2f.get(&k) {
                        for &nj in neighs {
                            if eligible[nj] && (face_centers[nj] - unit_ctr).dot(prox_dir) > -0.15 {
                                raw[nj] = true;
                            }
                        }
                    }
                }
                cur = parent[cur];
            }
        }
    }

    // Crecer una sola isla conexa desde seed_fi dentro de `raw`
    let mut deleted = vec![false; nf];
    let mut q = VecDeque::new();
    deleted[seed_fi] = true;
    q.push_back(seed_fi);
    while let Some(fi) = q.pop_front() {
        let [a, b, c] = clean.indices[fi];
        for (u, v) in [(a, b), (b, c), (c, a)] {
            let k = if u < v { (u, v) } else { (v, u) };
            if let Some(neighs) = e2f.get(&k) {
                for &nj in neighs {
                    if !deleted[nj] && raw[nj] {
                        deleted[nj] = true;
                        q.push_back(nj);
                    }
                }
            }
        }
    }

    // Si alguna cara en solape interproximal quedó desconectada fuera del disco principal, abortar limpiamente
    if (0..nf).any(|fi| contact_mask[fi] && !deleted[fi]) {
        return None;
    }

    // Si es un contacto puntual minúsculo (< 4 caras), expandir 1 anillo proximal limpio para evitar cuellos de 3 vértices
    if deleted.iter().filter(|&&d| d).count() < 4 {
        let seed_c = face_centers[seed_fi];
        let snap = deleted.clone();
        for fi in 0..nf {
            if !snap[fi] {
                continue;
            }
            let [a, b, c] = clean.indices[fi];
            for (u, v) in [(a, b), (b, c), (c, a)] {
                let k = if u < v { (u, v) } else { (v, u) };
                if let Some(neighs) = e2f.get(&k) {
                    for &nj in neighs {
                        if !deleted[nj]
                            && eligible[nj]
                            && face_centers[nj].distance(seed_c) <= 0.55
                        {
                            deleted[nj] = true;
                        }
                    }
                }
            }
        }
    }

    // Absorber islotes interiores atrapados dentro del hueco (evita generar 2 bucles en lugar de 1)
    let mut seen_kept = vec![false; nf];
    let mut kept_comps: Vec<Vec<usize>> = Vec::new();
    for fi in 0..nf {
        if deleted[fi] || seen_kept[fi] {
            continue;
        }
        let mut comp = Vec::new();
        let mut qk = VecDeque::new();
        seen_kept[fi] = true;
        qk.push_back(fi);
        while let Some(cur) = qk.pop_front() {
            comp.push(cur);
            let [a, b, c] = clean.indices[cur];
            for (u, v) in [(a, b), (b, c), (c, a)] {
                let k = if u < v { (u, v) } else { (v, u) };
                if let Some(neighs) = e2f.get(&k) {
                    for &nj in neighs {
                        if !deleted[nj] && !seen_kept[nj] {
                            seen_kept[nj] = true;
                            qk.push_back(nj);
                        }
                    }
                }
            }
        }
        kept_comps.push(comp);
    }
    if kept_comps.len() > 1 {
        let max_kept = (nf / 12).max(120);
        for comp in kept_comps {
            if comp.len() < max_kept && comp.iter().all(|&fi| eligible[fi]) {
                for fi in comp {
                    deleted[fi] = true;
                }
            }
        }
    }

    // Podar penínsulas (>=2 aristas pegadas al hueco) para garantizar disco topológico simple único
    for _pass in 0..10 {
        let mut changed = false;
        for fi in 0..nf {
            if deleted[fi] || !eligible[fi] {
                continue;
            }
            let [a, b, c] = clean.indices[fi];
            let mut del_edges = 0usize;
            for (u, v) in [(a, b), (b, c), (c, a)] {
                let k = if u < v { (u, v) } else { (v, u) };
                if let Some(neighs) = e2f.get(&k) {
                    if neighs.iter().any(|&nj| nj != fi && deleted[nj]) {
                        del_edges += 1;
                    }
                }
            }
            if del_edges >= 2 {
                deleted[fi] = true;
                changed = true;
            }
        }
        if !changed {
            break;
        }
    }

    let kept_tris: Vec<[u32; 3]> = clean
        .indices
        .iter()
        .enumerate()
        .filter(|&(fi, _)| !deleted[fi])
        .map(|(_, &t)| t)
        .collect();
    if kept_tris.len() < 8 {
        return None;
    }

    let mut punched = TriMeshData::new(clean.positions, kept_tris);
    if clean.is_external.len() == punched.positions.len() {
        punched.is_external = clean.is_external;
    }
    let loops = extract_ordered_boundary_loops(&punched);
    if loops.len() != existing_loops.len() + 1 {
        return None;
    }
    let new_loop = loops
        .into_iter()
        .find(|lp| {
            !lp.iter().any(|&vi| {
                let vp = punched.positions[vi as usize];
                existing_boundary_pts
                    .iter()
                    .any(|&bp| bp.distance_squared(vp) < 0.06 * 0.06)
            })
        })?;
    if new_loop.len() < 4 {
        return None;
    }
    if let Some(ocol) = other_col {
        if new_loop.iter().any(|&vi| {
            let p = punched.positions[vi as usize];
            aabb_dist(p, other_mn, other_mx) == 0.0 && ocol.contains_point_closed(p)
        }) {
            return None;
        }
    }
    Some((punched, new_loop, existing_loops.len()))
}

/// Detecta pares adicionales en `unit_map` que se tocan o superponen en contacto interproximal (< 0.18 mm)
/// fuera de los conectores explícitos.
fn find_touching_proximal_pairs(
    unit_map: &BTreeMap<i32, (TriMeshData, bool)>,
    existing_pairs: &[(i32, i32)],
) -> Vec<(i32, i32)> {
    struct Entry<'a> {
        fdi: i32,
        mesh: &'a TriMeshData,
        mn: Vec3,
        mx: Vec3,
        col: MeshCollider,
    }
    let entries: Vec<Entry<'_>> = unit_map
        .iter()
        .filter_map(|(&fdi, (m, _))| {
            let col = MeshCollider::from_mesh(m)?;
            let (mn, mx) = m.bounds();
            Some(Entry { fdi, mesh: m, mn, mx, col })
        })
        .collect();

    let mut touching = Vec::new();
    for i in 0..entries.len() {
        for j in (i + 1)..entries.len() {
            let a = &entries[i];
            let b = &entries[j];
            let pair = (a.fdi.min(b.fdi), a.fdi.max(b.fdi));
            if existing_pairs.contains(&pair) {
                continue;
            }
            if aabb_gap(a.mn, a.mx, b.mn, b.mx) > 0.22 {
                continue;
            }
            let check_pts = |src: &TriMeshData, dst: &Entry<'_>| -> bool {
                let step = (src.positions.len() / 600).max(1);
                for &p in src.positions.iter().step_by(step) {
                    let db = aabb_dist(p, dst.mn, dst.mx);
                    if db > 0.22 {
                        continue;
                    }
                    if db == 0.0 && dst.col.contains_point_closed(p) {
                        return true;
                    }
                    if dst.col.closest_point_and_dist(p).1 < 0.18 {
                        return true;
                    }
                }
                false
            };
            if check_pts(a.mesh, b) || check_pts(b.mesh, a) {
                touching.push(pair);
            }
        }
    }
    touching
}

#[inline]
fn eval_union_sdf(
    p: Vec3,
    parts: &[(Vec3, Vec3, MeshCollider, bool)],
    intaglio_col: Option<&MeshCollider>,
    intaglio_bounds: Option<(Vec3, Vec3)>,
) -> f32 {
    let mut best_signed = f32::NEG_INFINITY;
    let mut unit_d1 = f32::INFINITY;
    let mut unit_d2 = f32::INFINITY;
    for (bmin, bmax, col, is_unit) in parts {
        let db = aabb_dist(p, *bmin, *bmax);
        if (!*is_unit || db > 0.20) && -db <= best_signed {
            continue;
        }
        let (proj, d, tri_n) = col.closest_point_dist_and_tri_normal(p);
        let inside = if db == 0.0 {
            let delta = p - proj;
            if d > 1e-4 && delta.dot(tri_n).abs() > 0.85 * d {
                delta.dot(tri_n) < 0.0
            } else {
                col.contains_point_closed(p)
            }
        } else {
            false
        };
        let s = if inside { d } else { -d };
        if s > best_signed {
            best_signed = s;
        }
        if *is_unit {
            let ud = if inside { 0.0 } else { d };
            if ud < unit_d1 {
                unit_d2 = unit_d1;
                unit_d1 = ud;
            } else if ud < unit_d2 {
                unit_d2 = ud;
            }
        }
    }
    // Fusión de contactos interproximales rasantes (< 0.14 mm) fuera de la cavidad intaglio
    if unit_d1 < 0.14 && unit_d2 < 0.14 {
        let near_intaglio = match (intaglio_col, intaglio_bounds) {
            (Some(icol), Some((ib_mn, ib_mx))) => {
                aabb_dist(p, ib_mn, ib_mx) <= 0.35 && icol.closest_point_and_dist(p).1 <= 0.35
            }
            _ => false,
        };
        if !near_intaglio {
            let s_kiss = 0.14 - 0.5 * (unit_d1 + unit_d2);
            if s_kiss > best_signed {
                best_signed = s_kiss;
            }
        }
    }
    if best_signed.abs() < 1e-4 { -1e-4 } else { best_signed }
}

/// Respaldo booleano 3D de banda estrecha (multihilo + Kuhn Marching-Tetrahedra + re-proyección fina de paredes)
/// cuando un puente presenta geometría extrema donde la costura topológica directa no aplica.
fn fuse_bridge_boolean_union_preserving_intaglios(
    unit_map: &BTreeMap<i32, (TriMeshData, bool)>,
    connector_meshes: &[TriMeshData],
) -> TriMeshData {
    let mut mn = Vec3::splat(f32::INFINITY);
    let mut mx = Vec3::splat(f32::NEG_INFINITY);
    let mut parts: Vec<(Vec3, Vec3, MeshCollider, bool)> = Vec::new();

    for (m, _is_pontic) in unit_map.values() {
        if m.is_empty() {
            continue;
        }
        let mut oriented = m.clone();
        oriented.ensure_outward_winding_if_closed();
        let (a, b) = oriented.bounds();
        mn = mn.min(a);
        mx = mx.max(b);
        if let Some(c) = MeshCollider::from_mesh(&oriented) {
            parts.push((a, b, c, true));
        }
    }
    for cm in connector_meshes {
        if cm.is_empty() {
            continue;
        }
        let mut oriented = cm.clone();
        oriented.ensure_outward_winding_if_closed();
        let (a, b) = oriented.bounds();
        mn = mn.min(a);
        mx = mx.max(b);
        if let Some(c) = MeshCollider::from_mesh(&oriented) {
            parts.push((a, b, c, false));
        }
    }
    if parts.is_empty() {
        return TriMeshData::default();
    }

    // Construir malla maestra de intaglio + margen únicamente de los pilares (!is_pontic & !is_external)
    let mut intaglio_master = TriMeshData::default();
    for (m, is_pontic) in unit_map.values() {
        if *is_pontic || m.is_external.len() != m.positions.len() {
            continue;
        }
        let mut ipos = Vec::new();
        let mut itris = Vec::new();
        let mut vmap: HashMap<u32, u32> = HashMap::new();
        for &[a, b, c] in &m.indices {
            let a_ext = m.is_external[a as usize];
            let b_ext = m.is_external[b as usize];
            let c_ext = m.is_external[c as usize];
            if !a_ext && !b_ext && !c_ext {
                let mut map_v = |old: u32| -> u32 {
                    *vmap.entry(old).or_insert_with(|| {
                        let id = ipos.len() as u32;
                        ipos.push(m.positions[old as usize]);
                        id
                    })
                };
                itris.push([map_v(a), map_v(b), map_v(c)]);
            }
        }
        if !itris.is_empty() {
            intaglio_master.merge_in_place(&TriMeshData::new(ipos, itris));
        }
    }
    let intaglio_bounds = if !intaglio_master.is_empty() {
        Some(intaglio_master.bounds())
    } else {
        None
    };
    let intaglio_col = if !intaglio_master.is_empty() {
        MeshCollider::from_mesh(&intaglio_master)
    } else {
        None
    };

    let span = mx - mn;
    let max_span = span.x.max(span.y).max(span.z).max(6.0);
    let mut step = (max_span / 148.0).clamp(0.16, 0.22);
    let pad = Vec3::splat(step * 2.5);
    let p0 = mn - pad;
    let p1 = mx + pad;
    let box_span = p1 - p0;
    let raw_voxels = (box_span.x / step) * (box_span.y / step) * (box_span.z / step);
    if raw_voxels > 3_400_000.0 {
        step = ((box_span.x * box_span.y * box_span.z) / 3_000_000.0).cbrt().max(step);
    }

    let nx = (((p1.x - p0.x) / step).ceil() as usize + 1).clamp(10, 220);
    let ny = (((p1.y - p0.y) / step).ceil() as usize + 1).clamp(10, 220);
    let nz = (((p1.z - p0.z) / step).ceil() as usize + 1).clamp(10, 220);
    let dx = (p1.x - p0.x) / (nx - 1) as f32;
    let dy = (p1.y - p0.y) / (ny - 1) as f32;
    let dz = (p1.z - p0.z) / (nz - 1) as f32;

    let gidx = |ix: usize, iy: usize, iz: usize| -> usize { (ix * ny + iy) * nz + iz };
    let gpos = |ix: usize, iy: usize, iz: usize| -> Vec3 {
        Vec3::new(
            p0.x + ix as f32 * dx,
            p0.y + iy as f32 * dy,
            p0.z + iz as f32 * dz,
        )
    };

    // Evaluación 2 niveles (coarse-to-fine banda estrecha, stride = 4)
    let stride = 4usize;
    let mut cx_coords: Vec<usize> = (0..nx).step_by(stride).collect();
    if *cx_coords.last().unwrap_or(&0) != nx - 1 { cx_coords.push(nx - 1); }
    let mut cy_coords: Vec<usize> = (0..ny).step_by(stride).collect();
    if *cy_coords.last().unwrap_or(&0) != ny - 1 { cy_coords.push(ny - 1); }
    let mut cz_coords: Vec<usize> = (0..nz).step_by(stride).collect();
    if *cz_coords.last().unwrap_or(&0) != nz - 1 { cz_coords.push(nz - 1); }

    let ncx = cx_coords.len();
    let ncy = cy_coords.len();
    let ncz = cz_coords.len();
    let cidx = |x: usize, y: usize, z: usize| -> usize { (x * ncy + y) * ncz + z };
    let mut cphi = vec![-2.0_f32; ncx * ncy * ncz];

    for (ixc, &ix) in cx_coords.iter().enumerate() {
        for (iyc, &iy) in cy_coords.iter().enumerate() {
            for (izc, &iz) in cz_coords.iter().enumerate() {
                if ix == 0 || ix == nx - 1 || iy == 0 || iy == ny - 1 || iz == 0 || iz == nz - 1 {
                    cphi[cidx(ixc, iyc, izc)] = -2.0;
                } else {
                    cphi[cidx(ixc, iyc, izc)] = eval_union_sdf(
                        gpos(ix, iy, iz),
                        &parts,
                        intaglio_col.as_ref(),
                        intaglio_bounds,
                    );
                }
            }
        }
    }

    let band = (stride as f32) * step * 1.45;
    let total_fine = nx * ny * nz;
    let mut phi = vec![-2.0_f32; total_fine];
    let mut need_eval = vec![false; total_fine];

    for ixc in 0..(ncx - 1) {
        let bx0 = cx_coords[ixc];
        let bx1 = cx_coords[ixc + 1];
        for iyc in 0..(ncy - 1) {
            let by0 = cy_coords[iyc];
            let by1 = cy_coords[iyc + 1];
            for izc in 0..(ncz - 1) {
                let bz0 = cz_coords[izc];
                let bz1 = cz_coords[izc + 1];
                let corners = [
                    cphi[cidx(ixc, iyc, izc)],
                    cphi[cidx(ixc + 1, iyc, izc)],
                    cphi[cidx(ixc, iyc + 1, izc)],
                    cphi[cidx(ixc + 1, iyc + 1, izc)],
                    cphi[cidx(ixc, iyc, izc + 1)],
                    cphi[cidx(ixc + 1, iyc, izc + 1)],
                    cphi[cidx(ixc, iyc + 1, izc + 1)],
                    cphi[cidx(ixc + 1, iyc + 1, izc + 1)],
                ];
                let any_pos = corners.iter().any(|&v| v > 0.0);
                let any_neg = corners.iter().any(|&v| v < 0.0);
                let near_zero = corners.iter().any(|&v| v.abs() <= band);
                if (any_pos && any_neg) || near_zero {
                    for ix in bx0..=bx1 {
                        if ix == 0 || ix == nx - 1 { continue; }
                        for iy in by0..=by1 {
                            if iy == 0 || iy == ny - 1 { continue; }
                            for iz in bz0..=bz1 {
                                if iz == 0 || iz == nz - 1 { continue; }
                                need_eval[gidx(ix, iy, iz)] = true;
                            }
                        }
                    }
                } else if !any_neg {
                    for ix in bx0..=bx1 {
                        if ix == 0 || ix == nx - 1 { continue; }
                        for iy in by0..=by1 {
                            if iy == 0 || iy == ny - 1 { continue; }
                            for iz in bz0..=bz1 {
                                if iz == 0 || iz == nz - 1 { continue; }
                                phi[gidx(ix, iy, iz)] = 2.0;
                            }
                        }
                    }
                }
            }
        }
    }

    // Evaluar la banda estrecha fina en paralelo entre todos los núcleos de CPU
    let parts_ref = &parts;
    let int_col_ref = intaglio_col.as_ref();
    let need_ref = &need_eval;
    let n_workers = std::thread::available_parallelism()
        .map(|n| n.get())
        .unwrap_or(4)
        .clamp(1, 12);
    let ix_slices: Vec<usize> = (1..(nx - 1)).collect();
    let chunk_len = ix_slices.len().div_ceil(n_workers).max(1);
    let evaluated_chunks: Vec<Vec<(usize, f32)>> = std::thread::scope(|s| {
        let mut handles = Vec::new();
        for chunk in ix_slices.chunks(chunk_len) {
            handles.push(s.spawn(move || {
                let mut local = Vec::new();
                for &ix in chunk {
                    let px = p0.x + ix as f32 * dx;
                    for iy in 1..(ny - 1) {
                        let py = p0.y + iy as f32 * dy;
                        let base = (ix * ny + iy) * nz;
                        for iz in 1..(nz - 1) {
                            let id = base + iz;
                            if need_ref[id] {
                                let p = Vec3::new(px, py, p0.z + iz as f32 * dz);
                                local.push((id, eval_union_sdf(p, parts_ref, int_col_ref, intaglio_bounds)));
                            }
                        }
                    }
                }
                local
            }));
        }
        handles.into_iter().map(|h| h.join().unwrap_or_default()).collect()
    });
    for local in evaluated_chunks {
        for (id, val) in local {
            phi[id] = val;
        }
    }

    let tets: [[usize; 4]; 6] = [
        [0, 1, 3, 7],
        [0, 3, 2, 7],
        [0, 2, 6, 7],
        [0, 6, 4, 7],
        [0, 4, 5, 7],
        [0, 5, 1, 7],
    ];
    let mut out_pos = Vec::new();
    let mut out_tris = Vec::new();
    let mut ecache: HashMap<(u64, u64), u32> = HashMap::new();

    let interp = |ga: usize,
                  pa: Vec3,
                  va: f32,
                  gb: usize,
                  pb: Vec3,
                  vb: f32,
                  pos: &mut Vec<Vec3>,
                  cache: &mut HashMap<(u64, u64), u32>|
     -> u32 {
        let key = if ga < gb { (ga as u64, gb as u64) } else { (gb as u64, ga as u64) };
        *cache.entry(key).or_insert_with(|| {
            let denom = va - vb;
            let t = if denom.abs() > 1e-9 { (va / denom).clamp(0.02, 0.98) } else { 0.5 };
            let id = pos.len() as u32;
            pos.push(pa.lerp(pb, t));
            id
        })
    };

    let emit_tri = |a: u32, b: u32, c: u32, in_hint: Vec3, pos: &[Vec3], tris: &mut Vec<[u32; 3]>| {
        if a == b || b == c || c == a {
            return;
        }
        let n = (pos[b as usize] - pos[a as usize]).cross(pos[c as usize] - pos[a as usize]);
        let ctr = (pos[a as usize] + pos[b as usize] + pos[c as usize]) / 3.0;
        if n.dot(ctr - in_hint) >= 0.0 {
            tris.push([a, b, c]);
        } else {
            tris.push([a, c, b]);
        }
    };

    for ix in 0..(nx - 1) {
        for iy in 0..(ny - 1) {
            for iz in 0..(nz - 1) {
                let cids = [
                    gidx(ix, iy, iz),
                    gidx(ix + 1, iy, iz),
                    gidx(ix, iy + 1, iz),
                    gidx(ix + 1, iy + 1, iz),
                    gidx(ix, iy, iz + 1),
                    gidx(ix + 1, iy, iz + 1),
                    gidx(ix, iy + 1, iz + 1),
                    gidx(ix + 1, iy + 1, iz + 1),
                ];
                let mut any_pos = false;
                let mut any_neg = false;
                for &g in &cids {
                    if phi[g] > 0.0 { any_pos = true; } else { any_neg = true; }
                }
                if !any_pos || !any_neg {
                    continue;
                }
                let cpos = [
                    gpos(ix, iy, iz),
                    gpos(ix + 1, iy, iz),
                    gpos(ix, iy + 1, iz),
                    gpos(ix + 1, iy + 1, iz),
                    gpos(ix, iy, iz + 1),
                    gpos(ix + 1, iy, iz + 1),
                    gpos(ix, iy + 1, iz + 1),
                    gpos(ix + 1, iy + 1, iz + 1),
                ];
                for &tet in &tets {
                    let mut inv = Vec::with_capacity(4);
                    let mut outv = Vec::with_capacity(4);
                    for &k in &tet {
                        if phi[cids[k]] > 0.0 { inv.push(k); } else { outv.push(k); }
                    }
                    let hint: Vec3 = inv.iter().map(|&k| cpos[k]).sum::<Vec3>() / inv.len().max(1) as f32;
                    match inv.len() {
                        0 | 4 => {}
                        1 => {
                            let a = interp(cids[inv[0]], cpos[inv[0]], phi[cids[inv[0]]], cids[outv[0]], cpos[outv[0]], phi[cids[outv[0]]], &mut out_pos, &mut ecache);
                            let b = interp(cids[inv[0]], cpos[inv[0]], phi[cids[inv[0]]], cids[outv[1]], cpos[outv[1]], phi[cids[outv[1]]], &mut out_pos, &mut ecache);
                            let c = interp(cids[inv[0]], cpos[inv[0]], phi[cids[inv[0]]], cids[outv[2]], cpos[outv[2]], phi[cids[outv[2]]], &mut out_pos, &mut ecache);
                            emit_tri(a, b, c, hint, &out_pos, &mut out_tris);
                        }
                        3 => {
                            let a = interp(cids[inv[0]], cpos[inv[0]], phi[cids[inv[0]]], cids[outv[0]], cpos[outv[0]], phi[cids[outv[0]]], &mut out_pos, &mut ecache);
                            let b = interp(cids[inv[1]], cpos[inv[1]], phi[cids[inv[1]]], cids[outv[0]], cpos[outv[0]], phi[cids[outv[0]]], &mut out_pos, &mut ecache);
                            let c = interp(cids[inv[2]], cpos[inv[2]], phi[cids[inv[2]]], cids[outv[0]], cpos[outv[0]], phi[cids[outv[0]]], &mut out_pos, &mut ecache);
                            emit_tri(a, b, c, hint, &out_pos, &mut out_tris);
                        }
                        2 => {
                            let v00 = interp(cids[inv[0]], cpos[inv[0]], phi[cids[inv[0]]], cids[outv[0]], cpos[outv[0]], phi[cids[outv[0]]], &mut out_pos, &mut ecache);
                            let v01 = interp(cids[inv[0]], cpos[inv[0]], phi[cids[inv[0]]], cids[outv[1]], cpos[outv[1]], phi[cids[outv[1]]], &mut out_pos, &mut ecache);
                            let v10 = interp(cids[inv[1]], cpos[inv[1]], phi[cids[inv[1]]], cids[outv[0]], cpos[outv[0]], phi[cids[outv[0]]], &mut out_pos, &mut ecache);
                            let v11 = interp(cids[inv[1]], cpos[inv[1]], phi[cids[inv[1]]], cids[outv[1]], cpos[outv[1]], phi[cids[outv[1]]], &mut out_pos, &mut ecache);
                            emit_tri(v00, v01, v11, hint, &out_pos, &mut out_tris);
                            emit_tri(v00, v11, v10, hint, &out_pos, &mut out_tris);
                        }
                        _ => {}
                    }
                }
            }
        }
    }

    let mut fused = TriMeshData::new(out_pos, out_tris).weld_vertices_preserve_watertight(1e-5);
    fused = extract_largest_component(&fused).compact_unreferenced_vertices();
    fused.ensure_outward_winding_if_closed();
    fused.compute_normals();

    // Restaurar con precisión de micras la cavidad intaglio + línea de margen de los pilares
    let nv = fused.positions.len();
    let mut is_ext = vec![true; nv];
    if let (Some(icol), Some((ib_mn, ib_mx))) = (&intaglio_col, intaglio_bounds) {
        for i in 0..nv {
            let p = fused.positions[i];
            if aabb_dist(p, ib_mn, ib_mx) > 0.30 {
                continue;
            }
            let (proj, d, n_int) = icol.closest_point_dist_and_tri_normal(p);
            let n_v = fused.normals.get(i).copied().unwrap_or(Vec3::Z);
            if d <= 0.24 && n_v.dot(n_int) > 0.20 {
                is_ext[i] = false;
                fused.positions[i] = proj;
            } else if d <= 0.16 && n_v.dot(n_int) > 0.0 {
                is_ext[i] = false;
                fused.positions[i] = proj;
            } else if d <= 0.14 && n_v.dot(n_int) > -0.15 {
                is_ext[i] = false;
            }
        }
    }
    fused.is_external = is_ext;

    // 4 pasadas Taubin suaves SOLO sobre la superficie externa (elimina escalones de tetraedros)
    let adj = fused.build_adjacency();
    let (lam, mu) = (0.38_f32, -0.40_f32);
    let mut tmp = fused.positions.clone();
    for _pass in 0..4 {
        for i in 0..nv {
            if !fused.is_external[i] || adj[i].is_empty() {
                tmp[i] = fused.positions[i];
            } else {
                let avg: Vec3 = adj[i].iter().map(|&j| fused.positions[j]).sum::<Vec3>() / adj[i].len() as f32;
                tmp[i] = fused.positions[i] + (avg - fused.positions[i]) * lam;
            }
        }
        for i in 0..nv {
            if fused.is_external[i] && !adj[i].is_empty() {
                let avg: Vec3 = adj[i].iter().map(|&j| tmp[j]).sum::<Vec3>() / adj[i].len() as f32;
                fused.positions[i] = tmp[i] + (avg - tmp[i]) * mu;
            }
        }
    }

    // Re-proyectar paredes externas limpias (lejos de costuras entre partes) sobre las coronas originales
    // para conservar la definición oclusal exacta de biblioteca incluso en la ruta de respaldo
    for i in 0..nv {
        if !fused.is_external[i] {
            continue;
        }
        let p = fused.positions[i];
        let mut d1 = f32::INFINITY;
        let mut d2 = f32::INFINITY;
        let mut proj1 = p;
        for (bmin, bmax, col, _is_unit) in &parts {
            let db = aabb_dist(p, *bmin, *bmax);
            if db >= d2 && db > 0.42 {
                continue;
            }
            let (cand_p, cand_d) = col.closest_point_and_dist(p);
            if cand_d < d1 {
                d2 = d1;
                d1 = cand_d;
                proj1 = cand_p;
            } else if cand_d < d2 {
                d2 = cand_d;
            }
        }
        if d2 > 0.22 && d1 < 0.28 {
            let blend = smoothstep(0.22, 0.42, d2);
            fused.positions[i] = p.lerp(proj1, blend);
        }
    }

    if let (Some(icol), Some((ib_mn, ib_mx))) = (&intaglio_col, intaglio_bounds) {
        for i in 0..nv {
            if !fused.is_external[i] {
                let p = fused.positions[i];
                if aabb_dist(p, ib_mn, ib_mx) <= 0.30 {
                    fused.positions[i] = icol.project_point(p);
                }
            }
        }
    }

    fused.ensure_outward_winding_if_closed();
    fused.compute_normals();
    fused
}

/// Fusiona todas las coronas, pónticos y conectores de un puente en UNA SOLA malla cerrada watertight
/// uniendo tanto **conectores** como **contactos interproximales** con prioridad en unión topológica
/// directa (cero pixelación y sellado marginal/intaglio exacto).
pub fn fuse_bridge_watertight(
    units: &[(i32, TriMeshData, bool)],
    connectors: &[(&BridgeConnectorConfig, Vec3)],
) -> TriMeshData {
    let mut unit_map: BTreeMap<i32, (TriMeshData, bool)> = BTreeMap::new();
    for (fdi, mesh, is_pontic) in units {
        if !mesh.is_empty() {
            let mut m = ensure_pontic_closed(mesh);
            m.ensure_outward_winding_if_closed();
            unit_map.insert(*fdi, (m, *is_pontic));
        }
    }
    if unit_map.is_empty() {
        return TriMeshData::default();
    }
    if unit_map.len() == 1 {
        return unit_map.into_values().next().unwrap().0;
    }

    // Construir colliders una sola vez sobre las piezas cerradas de entrada
    let mut unit_bounds: BTreeMap<i32, (Vec3, Vec3)> = BTreeMap::new();
    let mut unit_cols: BTreeMap<i32, MeshCollider> = BTreeMap::new();
    for (&fdi, (m, _)) in &unit_map {
        unit_bounds.insert(fdi, m.bounds());
        if let Some(col) = MeshCollider::from_mesh(m) {
            unit_cols.insert(fdi, col);
        }
    }

    let mut conn_pairs_keys: Vec<(i32, i32)> = Vec::new();
    let mut work_pairs: Vec<(i32, i32, Option<(&BridgeConnectorConfig, Vec3)>)> = Vec::new();
    let mut connector_solid_meshes: Vec<TriMeshData> = Vec::with_capacity(connectors.len());

    for &(cfg, ax) in connectors {
        if let (Some((ma, _)), Some((mb, _))) = (unit_map.get(&cfg.tooth_a), unit_map.get(&cfg.tooth_b)) {
            let pair = (cfg.tooth_a.min(cfg.tooth_b), cfg.tooth_a.max(cfg.tooth_b));
            conn_pairs_keys.push(pair);
            work_pairs.push((cfg.tooth_a, cfg.tooth_b, Some((cfg, ax))));
            let fr = compute_connector_frame_with_colliders(
                ma,
                unit_cols.get(&cfg.tooth_a),
                mb,
                unit_cols.get(&cfg.tooth_b),
                ax,
                cfg,
            );
            connector_solid_meshes.push(build_connector_mesh(&fr, cfg));
        }
    }

    let default_ax = connectors
        .first()
        .map(|&(_, ax)| ax)
        .filter(|a| a.length_squared() > 1e-6)
        .unwrap_or(Vec3::Z);
    for (fa, fb) in find_touching_proximal_pairs(&unit_map, &conn_pairs_keys) {
        work_pairs.push((fa, fb, None));
    }

    if work_pairs.is_empty() {
        return unit_map.into_values().next().unwrap().0;
    }

    let mut active_meshes: BTreeMap<i32, TriMeshData> = unit_map
        .iter()
        .map(|(&k, (m, _))| (k, m.clone()))
        .collect();
    let is_pontic_map: BTreeMap<i32, bool> = unit_map
        .iter()
        .map(|(&k, &(_, p))| (k, p))
        .collect();

    let mut connector_bridges: Vec<TriMeshData> = Vec::new();
    let mut tube_shaft_pts: Vec<Vec3> = Vec::new();
    let mut topo_ok = true;

    for &(fa, fb, opt_cfg_ax) in &work_pairs {
        let (Some(ma), Some(mb)) = (active_meshes.get(&fa).cloned(), active_meshes.get(&fb).cloned()) else {
            continue;
        };
        let (Some(&(mn_a, mx_a)), Some(&(mn_b, mx_b))) = (unit_bounds.get(&fa), unit_bounds.get(&fb)) else {
            topo_ok = false;
            break;
        };
        let ca = unit_map.get(&fa).map(|(m, _)| m.center()).unwrap_or_else(|| ma.center());
        let cb = unit_map.get(&fb).map(|(m, _)| m.center()).unwrap_or_else(|| mb.center());
        let col_a = unit_cols.get(&fa);
        let col_b = unit_cols.get(&fb);

        let pa_is_p = is_pontic_map.get(&fa).copied().unwrap_or(false);
        let pb_is_p = is_pontic_map.get(&fb).copied().unwrap_or(false);

        let fr_opt = opt_cfg_ax.map(|(cfg, ax)| {
            compute_connector_frame_with_colliders(&ma, col_a, &mb, col_b, ax, cfg)
        });

        let (t_axis, u_occ, v_bl, anchor_a, anchor_b) = if let Some(fr) = &fr_opt {
            (fr.t_axis, fr.u_occ, fr.v_bl, fr.anchor_a, fr.anchor_b)
        } else {
            let mut ta = (cb - ca).normalize_or_zero();
            if ta.length_squared() < 1e-6 {
                ta = Vec3::X;
            }
            let u_raw = default_ax - ta * default_ax.dot(ta);
            let uo = if u_raw.length_squared() > 1e-5 {
                u_raw.normalize()
            } else {
                build_orthonormal_basis(ta).0
            };
            let vb = ta.cross(uo).normalize_or_zero();
            // Centro de contacto proximal para pares unidos únicamente por contacto interproximal
            let mut sum_c = Vec3::ZERO;
            let mut cnt_c = 0usize;
            if let Some(cb_col) = col_b {
                let step = (ma.positions.len() / 500).max(1);
                for &p in ma.positions.iter().step_by(step) {
                    if aabb_dist(p, mn_b, mx_b) <= 0.25 && cb_col.closest_point_and_dist(p).1 < 0.28 {
                        sum_c += p;
                        cnt_c += 1;
                    }
                }
            }
            let ctr_touch = if cnt_c > 0 {
                sum_c / cnt_c as f32
            } else {
                0.5 * (ca + cb)
            };
            (ta, uo, vb, ctr_touch, ctr_touch)
        };

        let hole_a = punch_proximal_contact_or_connector_disk(
            &ma,
            mn_b,
            mx_b,
            col_b,
            cb,
            anchor_a,
            t_axis,
            u_occ,
            v_bl,
            fr_opt.as_ref().map(|f| f.ring_a_3d.as_slice()),
            pa_is_p,
        );
        let hole_b = punch_proximal_contact_or_connector_disk(
            &mb,
            mn_a,
            mx_a,
            col_a,
            ca,
            anchor_b,
            -t_axis,
            u_occ,
            v_bl,
            fr_opt.as_ref().map(|f| f.ring_b_3d.as_slice()),
            pb_is_p,
        );

        let (Some((pa_mesh, mut loop_a, prev_a_loops)), Some((pb_mesh, mut loop_b, prev_b_loops))) =
            (hole_a, hole_b)
        else {
            topo_ok = false;
            break;
        };

        let ctr_a: Vec3 = loop_a.iter().map(|&v| pa_mesh.positions[v as usize]).sum::<Vec3>() / loop_a.len() as f32;
        let ctr_b: Vec3 = loop_b.iter().map(|&v| pb_mesh.positions[v as usize]).sum::<Vec3>() / loop_b.len() as f32;
        if (ctr_b - ctr_a).dot(t_axis) < 0.04 {
            topo_ok = false;
            break;
        }

        // Alinear orden angular para costura limpia sin torsión ni pliegues
        orient_and_rotate_loop_for_zipper(&pa_mesh, &mut loop_a, ctr_a, u_occ, v_bl);
        orient_and_rotate_loop_for_zipper(&pb_mesh, &mut loop_b, ctr_b, u_occ, v_bl);

        let loop_a_pts: Vec<Vec3> = loop_a.iter().map(|&v| pa_mesh.positions[v as usize]).collect();
        let loop_b_pts: Vec<Vec3> = loop_b.iter().map(|&v| pb_mesh.positions[v as usize]).collect();
        let gap_ab = ctr_a.distance(ctr_b);
        let (orig_a, orig_b) = if opt_cfg_ax.is_none() || gap_ab < 0.60 {
            let shared = 0.5 * (ctr_a + ctr_b);
            (shared, shared)
        } else {
            (ctr_a, ctr_b)
        };
        let sampled_a = sample_closed_3d_loop_at_angles(&loop_a_pts, orig_a, u_occ, v_bl, 48);
        let sampled_b = sample_closed_3d_loop_at_angles(&loop_b_pts, orig_b, u_occ, v_bl, 48);

        let tube = build_adaptive_bridge_tube(
            &sampled_a,
            &sampled_b,
            fr_opt.as_ref().map(|f| f.ring_c_3d.as_slice()),
            t_axis,
        );
        let sides = 48usize;
        if tube.positions.len() < sides * 2 {
            topo_ok = false;
            break;
        }
        tube_shaft_pts.extend_from_slice(&tube.positions);

        let tube_start_ring: Vec<u32> = (0..sides as u32).collect();
        let tube_last_base = ((tube.positions.len() / sides) - 1) * sides;
        let tube_end_ring: Vec<u32> = (0..sides as u32).map(|k| tube_last_base as u32 + k).collect();

        let mut tube_start_cw = tube_start_ring;
        tube_start_cw.reverse();
        let start_best = tube_start_cw
            .iter()
            .enumerate()
            .min_by(|(_, &va), (_, &vb)| {
                let da = tube.positions[va as usize].distance_squared(pa_mesh.positions[loop_a[0] as usize]);
                let db = tube.positions[vb as usize].distance_squared(pa_mesh.positions[loop_a[0] as usize]);
                da.partial_cmp(&db).unwrap_or(std::cmp::Ordering::Equal)
            })
            .map(|(i, _)| i)
            .unwrap_or(0);
        tube_start_cw.rotate_left(start_best);

        let mut tube_end_ccw = tube_end_ring;
        let end_best = tube_end_ccw
            .iter()
            .enumerate()
            .min_by(|(_, &va), (_, &vb)| {
                let da = tube.positions[va as usize].distance_squared(pb_mesh.positions[loop_b[0] as usize]);
                let db = tube.positions[vb as usize].distance_squared(pb_mesh.positions[loop_b[0] as usize]);
                da.partial_cmp(&db).unwrap_or(std::cmp::Ordering::Equal)
            })
            .map(|(i, _)| i)
            .unwrap_or(0);
        tube_end_ccw.rotate_left(end_best);

        let off_t = pa_mesh.positions.len() as u32;
        let mut check_pos = pa_mesh.positions.clone();
        check_pos.extend_from_slice(&tube.positions);
        let off_b = check_pos.len() as u32;
        check_pos.extend_from_slice(&pb_mesh.positions);

        let tube_start_shifted: Vec<u32> = tube_start_cw.iter().map(|&v| v + off_t).collect();
        let tube_end_shifted: Vec<u32> = tube_end_ccw.iter().map(|&v| v + off_t).collect();
        let loop_b_shifted: Vec<u32> = loop_b.iter().map(|&v| v + off_b).collect();

        let zip_a = zipper_stitch_loops(&check_pos, &loop_a, &tube_start_shifted);
        let zip_b = zipper_stitch_loops(&check_pos, &loop_b_shifted, &tube_end_shifted);

        let mut bridge_part_tris = Vec::with_capacity(tube.indices.len() + zip_a.len() + zip_b.len());
        for &[x, y, z] in &tube.indices {
            bridge_part_tris.push([x + off_t, y + off_t, z + off_t]);
        }
        bridge_part_tris.extend_from_slice(&zip_a);
        bridge_part_tris.extend_from_slice(&zip_b);

        let mut test_tris = pa_mesh.indices.clone();
        test_tris.extend_from_slice(&bridge_part_tris);
        for &[x, y, z] in &pb_mesh.indices {
            test_tris.push([x + off_b, y + off_b, z + off_b]);
        }
        let test_merged = TriMeshData::new(check_pos.clone(), test_tris).weld_vertices_preserve_watertight(1e-5);
        let au = test_merged.inspect_cam();
        let open_loops_after = extract_ordered_boundary_loops(&test_merged).len();
        if au.non_manifold_edges > 0
            || au.flipped_winding_edges > 0
            || open_loops_after != prev_a_loops + prev_b_loops
        {
            topo_ok = false;
            break;
        }

        active_meshes.insert(fa, pa_mesh);
        active_meshes.insert(fb, pb_mesh);

        let mut conn_only_pos = Vec::new();
        let mut conn_map = HashMap::new();
        let map_v = |old: u32, pos_src: &[Vec3], out_p: &mut Vec<Vec3>, map: &mut HashMap<u32, u32>| -> u32 {
            *map.entry(old).or_insert_with(|| {
                let id = out_p.len() as u32;
                out_p.push(pos_src[old as usize]);
                id
            })
        };
        let mut conn_only_tris = Vec::with_capacity(bridge_part_tris.len());
        for &[x, y, z] in &bridge_part_tris {
            let nx = map_v(x, &check_pos, &mut conn_only_pos, &mut conn_map);
            let ny = map_v(y, &check_pos, &mut conn_only_pos, &mut conn_map);
            let nz = map_v(z, &check_pos, &mut conn_only_pos, &mut conn_map);
            conn_only_tris.push([nx, ny, nz]);
        }
        let mut conn_mesh = TriMeshData::new(conn_only_pos, conn_only_tris);
        conn_mesh.is_external = vec![true; conn_mesh.positions.len()];
        connector_bridges.push(conn_mesh);
    }

    if topo_ok {
        let mut assembled = TriMeshData::default();
        for m in active_meshes.values() {
            assembled.merge_in_place(m);
        }
        for cb in &connector_bridges {
            assembled.merge_in_place(cb);
        }
        assembled = assembled
            .weld_vertices_preserve_watertight(1e-5)
            .compact_unreferenced_vertices();

        // Suavizado Taubin (4 pasadas) SOLO sobre anillos internos del conector/contacto (dientes y márgenes congelados)
        if !tube_shaft_pts.is_empty() {
            let inv = 1.0 / 1e-4_f32;
            let key_pt = |p: Vec3| -> (i64, i64, i64) {
                (
                    (p.x * inv).round() as i64,
                    (p.y * inv).round() as i64,
                    (p.z * inv).round() as i64,
                )
            };
            let mut shaft_keys = std::collections::HashSet::with_capacity(tube_shaft_pts.len());
            for &p in &tube_shaft_pts {
                shaft_keys.insert(key_pt(p));
            }
            let nv = assembled.positions.len();
            let smooth_mask: Vec<bool> = assembled
                .positions
                .iter()
                .map(|&p| shaft_keys.contains(&key_pt(p)))
                .collect();
            let adj = assembled.build_adjacency();
            let (lam, mu) = (0.42_f32, -0.44_f32);
            let mut tmp = assembled.positions.clone();
            for _pass in 0..4 {
                for i in 0..nv {
                    if !smooth_mask[i] || adj[i].is_empty() {
                        tmp[i] = assembled.positions[i];
                    } else {
                        let avg: Vec3 = adj[i].iter().map(|&j| assembled.positions[j]).sum::<Vec3>()
                            / adj[i].len() as f32;
                        tmp[i] = assembled.positions[i] + (avg - assembled.positions[i]) * lam;
                    }
                }
                for i in 0..nv {
                    if smooth_mask[i] && !adj[i].is_empty() {
                        let avg: Vec3 = adj[i].iter().map(|&j| tmp[j]).sum::<Vec3>() / adj[i].len() as f32;
                        assembled.positions[i] = tmp[i] + (avg - tmp[i]) * mu;
                    }
                }
            }
        }

        assembled.ensure_outward_winding_if_closed();
        assembled.compute_normals();
        let audit = assembled.inspect_cam();

        // Confirmar componente conexo único (todas las piezas y conectores quedaron soldados en una pieza)
        let mut adj_cc: Vec<Vec<u32>> = vec![Vec::new(); assembled.positions.len()];
        for &[a, b, c] in &assembled.indices {
            adj_cc[a as usize].extend_from_slice(&[b, c]);
            adj_cc[b as usize].extend_from_slice(&[a, c]);
            adj_cc[c as usize].extend_from_slice(&[a, b]);
        }
        let mut seen = vec![false; assembled.positions.len()];
        let mut q = VecDeque::new();
        if !seen.is_empty() {
            seen[0] = true;
            q.push_back(0u32);
            while let Some(v) = q.pop_front() {
                for &nb in &adj_cc[v as usize] {
                    if !seen[nb as usize] {
                        seen[nb as usize] = true;
                        q.push_back(nb);
                    }
                }
            }
        }
        let single_cc = seen.iter().filter(|&&s| s).count() == assembled.positions.len();

        if audit.watertight
            && audit.open_edges == 0
            && audit.non_manifold_edges == 0
            && audit.flipped_winding_edges == 0
            && audit.folded_edges <= 16
            && single_cc
        {
            return assembled;
        }
    }

    fuse_bridge_boolean_union_preserving_intaglios(&unit_map, &connector_solid_meshes)
}
