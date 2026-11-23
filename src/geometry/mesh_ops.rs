//! Módulo 2 (Núcleo Geométrico): Limpieza, Filtro Taubin, Recorte 3D, Zócalo Sólido,
//! Modelo Hollow Imprimible (1–3 mm sin conflictos geométricos) y Socavados.

use crate::geometry::margin::{build_tube_mesh, build_uv_sphere};
use crate::geometry::mesh::{build_orthonormal_basis, to_pvec, MeshCollider, TriMeshData};
use glam::{Vec2, Vec3};
use parry3d::query::{Ray, RayCast};
use std::collections::{HashMap, HashSet, VecDeque};
use std::f32::consts::PI;

/// Repara y limpia un escaneo intraoral:
/// 1. Suelda vértices duplicados y elimina triángulos degenerados.
/// 2. Opcionalmente conserva solo la componente conexa principal (elimina islas flotantes).
/// 3. Aplica suavizado Taubin ($\lambda / \mu$) preservando el volumen exacto del muñón.
pub fn clean_mesh(
    mesh: &TriMeshData,
    keep_only_largest: bool,
    taubin_iterations: usize,
) -> TriMeshData {
    if mesh.is_empty() {
        return mesh.clone();
    }
    let mut out = mesh.weld_vertices(1e-4);

    if keep_only_largest && !out.indices.is_empty() {
        out = extract_largest_component(&out);
    }

    if taubin_iterations > 0 && !out.positions.is_empty() {
        taubin_smooth_in_place(&mut out, taubin_iterations, 0.45, -0.47);
    }
    out.compute_normals();
    out
}

/// Conserva únicamente la isla triangular conexa más grande mediante BFS sobre vértices.
pub fn extract_largest_component(mesh: &TriMeshData) -> TriMeshData {
    let n = mesh.positions.len();
    if n == 0 || mesh.indices.is_empty() {
        return mesh.clone();
    }
    let adj = mesh.build_adjacency();
    let mut comp_id = vec![usize::MAX; n];
    let mut best_comp = 0usize;
    let mut best_count = 0usize;
    let mut cur_comp = 0usize;
    let mut queue = VecDeque::new();

    for start in 0..n {
        if comp_id[start] != usize::MAX {
            continue;
        }
        let mut count = 0usize;
        comp_id[start] = cur_comp;
        queue.push_back(start);
        while let Some(u) = queue.pop_front() {
            count += 1;
            for &v in &adj[u] {
                if comp_id[v] == usize::MAX {
                    comp_id[v] = cur_comp;
                    queue.push_back(v);
                }
            }
        }
        if count > best_count {
            best_count = count;
            best_comp = cur_comp;
        }
        cur_comp += 1;
    }

    let mut remap = vec![u32::MAX; n];
    let mut new_pos = Vec::with_capacity(best_count);
    let mut new_ext = Vec::with_capacity(best_count);
    for (i, &c) in comp_id.iter().enumerate() {
        if c == best_comp {
            remap[i] = new_pos.len() as u32;
            new_pos.push(mesh.positions[i]);
            new_ext.push(mesh.is_external.get(i).copied().unwrap_or(true));
        }
    }

    let mut new_tris = Vec::with_capacity(mesh.indices.len());
    for &[a, b, c] in &mesh.indices {
        let ra = remap[a as usize];
        let rb = remap[b as usize];
        let rc = remap[c as usize];
        if ra != u32::MAX && rb != u32::MAX && rc != u32::MAX {
            new_tris.push([ra, rb, rc]);
        }
    }
    let mut out = TriMeshData::new(new_pos, new_tris);
    out.is_external = new_ext;
    out
}

/// Filtro Taubin sin contracción volumétrica sobre la conectividad topológica 1-anillo.
pub fn taubin_smooth_in_place(mesh: &mut TriMeshData, iterations: usize, lambda: f32, mu: f32) {
    let adj = mesh.build_adjacency();
    let n = mesh.positions.len();
    let mut tmp = vec![Vec3::ZERO; n];

    for _ in 0..iterations {
        for i in 0..n {
            if adj[i].is_empty() {
                tmp[i] = mesh.positions[i];
            } else {
                let avg: Vec3 = adj[i].iter().map(|&j| mesh.positions[j]).sum::<Vec3>()
                    / adj[i].len() as f32;
                tmp[i] = mesh.positions[i] + lambda * (avg - mesh.positions[i]);
            }
        }
        for i in 0..n {
            if adj[i].is_empty() {
                mesh.positions[i] = tmp[i];
            } else {
                let avg: Vec3 = adj[i].iter().map(|&j| tmp[j]).sum::<Vec3>()
                    / adj[i].len() as f32;
                mesh.positions[i] = tmp[i] + mu * (avg - tmp[i]);
            }
        }
    }
    mesh.compute_normals();
}

/// Construye las mallas OpenGL del plano visual de recorte:
/// `(plano_semitransparente, marco_con_borde_y_flecha_hacia_region_conservada)`.
pub fn build_clip_plane_visuals(
    origin: Vec3,
    normal: Vec3,
    kept_normal: Vec3,
    half_size: f32,
) -> (TriMeshData, TriMeshData) {
    let n = if normal.length_squared() > 1e-8 {
        normal.normalize()
    } else {
        Vec3::Z
    };
    let kn = if kept_normal.length_squared() > 1e-8 {
        kept_normal.normalize()
    } else {
        n
    };
    let hs = half_size.clamp(8.0, 95.0);
    let (u, v, _) = build_orthonormal_basis(n);

    let c0 = origin - u * hs - v * hs;
    let c1 = origin + u * hs - v * hs;
    let c2 = origin + u * hs + v * hs;
    let c3 = origin - u * hs + v * hs;

    let mut quad = TriMeshData::new(vec![c0, c1, c2, c3], vec![[0, 1, 2], [0, 2, 3]]);
    quad.normals = vec![n, n, n, n];

    let border_r = (hs * 0.012).clamp(0.18, 0.45);
    let mut border = build_tube_mesh(&[c0, c1, c2, c3], border_r, true, 8);
    // Retícula central sutil sobre el plano
    border.merge_in_place(&build_tube_mesh(
        &[origin - u * hs, origin + u * hs],
        border_r * 0.42,
        false,
        6,
    ));
    border.merge_in_place(&build_tube_mesh(
        &[origin - v * hs, origin + v * hs],
        border_r * 0.42,
        false,
        6,
    ));
    // Indicador direccional apuntando hacia el lado que se CONSERVARÁ al cortar
    let arrow_end = origin + kn * (hs * 0.16).clamp(3.0, 9.0);
    border.merge_in_place(&build_tube_mesh(
        &[origin, arrow_end],
        border_r * 0.72,
        false,
        8,
    ));
    border.merge_in_place(&build_uv_sphere(arrow_end, border_r * 1.9, 8, 10));

    (quad, border)
}

/// Construye el marco wireframe 3D visible para la herramienta Caja 3D (`[min_b, max_b]`).
pub fn build_clip_box_wireframe(min_b: Vec3, max_b: Vec3) -> TriMeshData {
    let mn = min_b.min(max_b);
    let mx = min_b.max(max_b);
    let diag = (mx - mn).length().max(4.0);
    let r = (diag * 0.0045).clamp(0.14, 0.38);

    let p000 = Vec3::new(mn.x, mn.y, mn.z);
    let p100 = Vec3::new(mx.x, mn.y, mn.z);
    let p110 = Vec3::new(mx.x, mx.y, mn.z);
    let p010 = Vec3::new(mn.x, mx.y, mn.z);
    let p001 = Vec3::new(mn.x, mn.y, mx.z);
    let p101 = Vec3::new(mx.x, mn.y, mx.z);
    let p111 = Vec3::new(mx.x, mx.y, mx.z);
    let p011 = Vec3::new(mn.x, mx.y, mx.z);

    let mut out = build_tube_mesh(&[p000, p100, p110, p010], r, true, 6);
    out.merge_in_place(&build_tube_mesh(&[p001, p101, p111, p011], r, true, 6));
    for (a, b) in [(p000, p001), (p100, p101), (p110, p111), (p010, p011)] {
        out.merge_in_place(&build_tube_mesh(&[a, b], r, false, 6));
    }
    out
}

/// Recorta una malla triangular exacta contra un plano `(origin, normal)`.
/// Conserva el semiespacio `(p - origin)·normal >= 0` (o invertido si `invert = true`).
/// Comparte vértices de intersección por arista canónica para evitar grietas en recortes múltiples.
pub fn clip_by_plane(
    mesh: &TriMeshData,
    plane_origin: Vec3,
    plane_normal: Vec3,
    invert: bool,
) -> TriMeshData {
    if mesh.is_empty() {
        return mesh.clone();
    }
    let mut n = plane_normal.normalize_or_zero();
    if n.length_squared() < 1e-6 {
        n = Vec3::Z;
    }
    if invert {
        n = -n;
    }

    let mut new_pos: Vec<Vec3> = Vec::with_capacity(mesh.positions.len());
    let mut new_tris: Vec<[u32; 3]> = Vec::with_capacity(mesh.indices.len());
    let mut orig_map: HashMap<u32, u32> = HashMap::with_capacity(mesh.positions.len());
    let mut edge_split: HashMap<(u32, u32), u32> = HashMap::new();

    let signed_d: Vec<f32> = mesh
        .positions
        .iter()
        .map(|&p| {
            let d = (p - plane_origin).dot(n);
            if d.abs() < 1e-5 { 0.0 } else { d }
        })
        .collect();

    let get_orig = |vi: u32, pos_out: &mut Vec<Vec3>, map: &mut HashMap<u32, u32>| -> u32 {
        *map.entry(vi).or_insert_with(|| {
            let id = pos_out.len() as u32;
            pos_out.push(mesh.positions[vi as usize]);
            id
        })
    };

    let get_edge_split = |ua: u32,
                          ub: u32,
                          da: f32,
                          db: f32,
                          pos_out: &mut Vec<Vec3>,
                          ecache: &mut HashMap<(u32, u32), u32>|
     -> u32 {
        let key = if ua < ub { (ua, ub) } else { (ub, ua) };
        *ecache.entry(key).or_insert_with(|| {
            let (p0, d0, p1, d1) = if ua < ub {
                (mesh.positions[ua as usize], da, mesh.positions[ub as usize], db)
            } else {
                (mesh.positions[ub as usize], db, mesh.positions[ua as usize], da)
            };
            let denom = d0 - d1;
            let t = if denom.abs() > 1e-9 {
                (d0 / denom).clamp(0.0, 1.0)
            } else {
                0.5
            };
            let id = pos_out.len() as u32;
            pos_out.push(p0.lerp(p1, t));
            id
        })
    };

    let push_tri_idx = |a: u32, b: u32, c: u32, pos_out: &[Vec3], tris_out: &mut Vec<[u32; 3]>| {
        if a == b || b == c || c == a {
            return;
        }
        let e1 = pos_out[b as usize] - pos_out[a as usize];
        let e2 = pos_out[c as usize] - pos_out[a as usize];
        if e1.cross(e2).length_squared() < 1e-11 {
            return;
        }
        tris_out.push([a, b, c]);
    };

    for &[i0, i1, i2] in &mesh.indices {
        let verts = [
            (i0, signed_d[i0 as usize]),
            (i1, signed_d[i1 as usize]),
            (i2, signed_d[i2 as usize]),
        ];
        let mut inside = Vec::with_capacity(3);
        let mut outside = Vec::with_capacity(3);
        for (idx, &(_, d)) in verts.iter().enumerate() {
            if d >= 0.0 {
                inside.push(idx);
            } else {
                outside.push(idx);
            }
        }

        match inside.len() {
            3 => {
                let a = get_orig(i0, &mut new_pos, &mut orig_map);
                let b = get_orig(i1, &mut new_pos, &mut orig_map);
                let c = get_orig(i2, &mut new_pos, &mut orig_map);
                push_tri_idx(a, b, c, &new_pos, &mut new_tris);
            }
            0 => {}
            1 => {
                let ia = inside[0];
                let ib = (ia + 1) % 3;
                let ic = (ia + 2) % 3;
                let (va, da) = verts[ia];
                let (vb, db) = verts[ib];
                let (vc, dc) = verts[ic];
                let a = get_orig(va, &mut new_pos, &mut orig_map);
                let ab = get_edge_split(va, vb, da, db, &mut new_pos, &mut edge_split);
                let ac = get_edge_split(va, vc, da, dc, &mut new_pos, &mut edge_split);
                push_tri_idx(a, ab, ac, &new_pos, &mut new_tris);
            }
            2 => {
                let io = outside[0];
                let ia = (io + 1) % 3;
                let ib = (io + 2) % 3;
                let (vo, do_) = verts[io];
                let (va, da) = verts[ia];
                let (vb, db) = verts[ib];
                let a = get_orig(va, &mut new_pos, &mut orig_map);
                let b = get_orig(vb, &mut new_pos, &mut orig_map);
                let ao = get_edge_split(va, vo, da, do_, &mut new_pos, &mut edge_split);
                let bo = get_edge_split(vb, vo, db, do_, &mut new_pos, &mut edge_split);
                push_tri_idx(a, b, bo, &new_pos, &mut new_tris);
                push_tri_idx(a, bo, ao, &new_pos, &mut new_tris);
            }
            _ => {}
        }
    }

    if new_tris.is_empty() {
        return TriMeshData::default();
    }
    TriMeshData::new(new_pos, new_tris).weld_vertices(1e-4)
}

/// Recorta la malla conservando el interior (o exterior si `invert = true`) de la caja `[min_b, max_b]`.
pub fn clip_by_box(mesh: &TriMeshData, min_b: Vec3, max_b: Vec3, invert: bool) -> TriMeshData {
    if mesh.is_empty() {
        return mesh.clone();
    }
    let mn = min_b.min(max_b);
    let mx = min_b.max(max_b);
    if !invert {
        let mut cur = mesh.clone();
        let planes = [
            (Vec3::new(mn.x, 0.0, 0.0), Vec3::X),
            (Vec3::new(mx.x, 0.0, 0.0), -Vec3::X),
            (Vec3::new(0.0, mn.y, 0.0), Vec3::Y),
            (Vec3::new(0.0, mx.y, 0.0), -Vec3::Y),
            (Vec3::new(0.0, 0.0, mn.z), Vec3::Z),
            (Vec3::new(0.0, 0.0, mx.z), -Vec3::Z),
        ];
        for (orig, norm) in planes {
            cur = clip_by_plane(&cur, orig, norm, false);
            if cur.is_empty() {
                break;
            }
        }
        cur
    } else {
        let mut new_pos = Vec::new();
        let mut new_tris = Vec::new();
        for &[a, b, c] in &mesh.indices {
            let p0 = mesh.positions[a as usize];
            let p1 = mesh.positions[b as usize];
            let p2 = mesh.positions[c as usize];
            let ctr = (p0 + p1 + p2) / 3.0;
            let inside = ctr.x >= mn.x
                && ctr.x <= mx.x
                && ctr.y >= mn.y
                && ctr.y <= mx.y
                && ctr.z >= mn.z
                && ctr.z <= mx.z;
            if !inside {
                let base = new_pos.len() as u32;
                new_pos.push(p0);
                new_pos.push(p1);
                new_pos.push(p2);
                new_tris.push([base, base + 1, base + 2]);
            }
        }
        if new_tris.is_empty() {
            return TriMeshData::default();
        }
        TriMeshData::new(new_pos, new_tris).weld_vertices(1e-4)
    }
}

// ============================================================================
// TRIANGULACIÓN 2D ROBUSTA CON AGUJEROS (EBERLY EAR-CLIPPING + DELAUNAY FLIPS)
// ============================================================================

fn signed_area_2d(pts: &[Vec2], loop_idx: &[usize]) -> f32 {
    let n = loop_idx.len();
    if n < 3 {
        return 0.0;
    }
    let mut s = 0.0_f64;
    for i in 0..n {
        let a = pts[loop_idx[i]].as_dvec2();
        let b = pts[loop_idx[(i + 1) % n]].as_dvec2();
        s += a.x * b.y - b.x * a.y;
    }
    (0.5 * s) as f32
}

fn dedup_loop_indices(pts: &[Vec2], raw: Vec<usize>) -> Vec<usize> {
    let mut out: Vec<usize> = Vec::with_capacity(raw.len());
    for idx in raw {
        if let Some(&last) = out.last() {
            if last == idx || pts[last].distance_squared(pts[idx]) < 1e-10 {
                continue;
            }
        }
        out.push(idx);
    }
    while out.len() >= 2 && (out[0] == *out.last().unwrap() || pts[out[0]].distance_squared(pts[*out.last().unwrap()]) < 1e-10) {
        out.pop();
    }
    out
}

fn orient_2d(a: Vec2, b: Vec2, c: Vec2) -> f64 {
    let ab = (b - a).as_dvec2();
    let ac = (c - a).as_dvec2();
    ab.x * ac.y - ab.y * ac.x
}

fn point_in_triangle_2d_inclusive(p: Vec2, a: Vec2, b: Vec2, c: Vec2) -> bool {
    let c1 = orient_2d(a, b, p);
    let c2 = orient_2d(b, c, p);
    let c3 = orient_2d(c, a, p);
    c1 >= -1e-9 && c2 >= -1e-9 && c3 >= -1e-9
}

fn segments_strict_intersect_2d(a: Vec2, b: Vec2, c: Vec2, d: Vec2) -> bool {
    if a.distance_squared(c) < 1e-10
        || a.distance_squared(d) < 1e-10
        || b.distance_squared(c) < 1e-10
        || b.distance_squared(d) < 1e-10
    {
        return false;
    }
    let d1 = orient_2d(c, d, a);
    let d2 = orient_2d(c, d, b);
    let d3 = orient_2d(a, b, c);
    let d4 = orient_2d(a, b, d);
    ((d1 > 1e-9 && d2 < -1e-9) || (d1 < -1e-9 && d2 > 1e-9))
        && ((d3 > 1e-9 && d4 < -1e-9) || (d3 < -1e-9 && d4 > 1e-9))
}

fn segment_intersects_any_loop(
    pts: &[Vec2],
    p_a: Vec2,
    p_b: Vec2,
    loops: &[&[usize]],
    skip_idx_a: Option<usize>,
    skip_idx_b: Option<usize>,
) -> bool {
    for lp in loops {
        let m = lp.len();
        for i in 0..m {
            let u = lp[i];
            let v = lp[(i + 1) % m];
            if Some(u) == skip_idx_a
                || Some(v) == skip_idx_a
                || Some(u) == skip_idx_b
                || Some(v) == skip_idx_b
            {
                continue;
            }
            if segments_strict_intersect_2d(p_a, p_b, pts[u], pts[v]) {
                return true;
            }
        }
    }
    false
}

fn in_interior_wedge_ccw(pts: &[Vec2], poly_loop: &[usize], pos: usize, target: Vec2) -> bool {
    let n = poly_loop.len();
    let p = pts[poly_loop[(pos + n - 1) % n]];
    let c = pts[poly_loop[pos]];
    let nx = pts[poly_loop[(pos + 1) % n]];
    let is_convex = orient_2d(p, c, nx) > 0.0;
    let left_prev = orient_2d(p, c, target) >= -1e-9;
    let left_next = orient_2d(c, nx, target) >= -1e-9;
    if is_convex {
        left_prev && left_next
    } else {
        left_prev || left_next
    }
}

fn find_visible_outer_vertex(
    pts: &[Vec2],
    poly_loop: &[usize],
    hole_m_idx: usize,
    rem_holes: &[&[usize]],
) -> usize {
    let pm = pts[hole_m_idx];
    let n = poly_loop.len();
    let pick_best_copy = |default_pos: usize| -> usize {
        let v_id = poly_loop[default_pos];
        (0..n)
            .find(|&k| poly_loop[k] == v_id && in_interior_wedge_ccw(pts, poly_loop, k, pm))
            .unwrap_or(default_pos)
    };

    // Método 1 (Eberly +X ray): buscar arista de `poly_loop` más cercana al rayo +X desde `pm`
    let mut best_x = f32::INFINITY;
    let mut best_edge_i: Option<usize> = None;
    for i in 0..n {
        let ua = poly_loop[i];
        let ub = poly_loop[(i + 1) % n];
        let a = pts[ua];
        let b = pts[ub];
        // La arista orientada CCW debe apuntar hacia abajo a la derecha de M o cruzar y = pm.y
        let crosses = (a.y <= pm.y && b.y > pm.y) || (b.y <= pm.y && a.y > pm.y);
        if !crosses {
            continue;
        }
        let dy = b.y - a.y;
        if dy.abs() < 1e-9 {
            continue;
        }
        let t = (pm.y - a.y) / dy;
        let x_int = a.x + t * (b.x - a.x);
        if x_int >= pm.x - 1e-5 && x_int < best_x {
            best_x = x_int;
            best_edge_i = Some(i);
        }
    }

    if let Some(ei) = best_edge_i {
        let ua = poly_loop[ei];
        let ub = poly_loop[(ei + 1) % n];
        let p_pos = if pts[ua].x >= pts[ub].x { ei } else { (ei + 1) % n };
        let mut vis_pos = p_pos;
        let pv = pts[poly_loop[p_pos]];
        let i_pt = Vec2::new(best_x, pm.y);

        // Si el triángulo (pm, i_pt, pv) es válido, comprobar vértices reflejos estrictamente dentro
        let tri_ccw = orient_2d(pm, i_pt, pv);
        let mut best_ang = f64::INFINITY;
        let mut best_dist = f32::INFINITY;
        for j in 0..n {
            if j == p_pos {
                continue;
            }
            let r_idx = poly_loop[j];
            let pr = pts[r_idx];
            if pr.distance_squared(pv) < 1e-10 || pr.distance_squared(pm) < 1e-10 {
                continue;
            }
            let inside_tri = if tri_ccw >= 0.0 {
                point_in_triangle_2d_inclusive(pr, pm, i_pt, pv)
            } else {
                point_in_triangle_2d_inclusive(pr, pm, pv, i_pt)
            };
            if inside_tri {
                let d_vec = pr - pm;
                let ang = (d_vec.y.abs() as f64).atan2((d_vec.x.max(1e-7)) as f64);
                let dist = d_vec.length_squared();
                if ang < best_ang - 1e-6 || ((ang - best_ang).abs() <= 1e-6 && dist < best_dist) {
                    best_ang = ang;
                    best_dist = dist;
                    vis_pos = j;
                }
            }
        }

        let candidate_idx = poly_loop[vis_pos];
        let mut all_loops: Vec<&[usize]> = Vec::with_capacity(1 + rem_holes.len());
        all_loops.push(poly_loop);
        all_loops.extend_from_slice(rem_holes);
        if !segment_intersects_any_loop(
            pts,
            pm,
            pts[candidate_idx],
            &all_loops,
            Some(hole_m_idx),
            Some(candidate_idx),
        ) {
            return pick_best_copy(vis_pos);
        }
    }

    // Respaldo geométrico: vértice visible más cercano de `poly_loop`
    let mut candidates: Vec<(usize, f32)> = poly_loop
        .iter()
        .enumerate()
        .map(|(j, &idx)| (j, pts[idx].distance_squared(pm)))
        .collect();
    candidates.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));

    let mut all_loops: Vec<&[usize]> = Vec::with_capacity(1 + rem_holes.len());
    all_loops.push(poly_loop);
    all_loops.extend_from_slice(rem_holes);

    for (j, _) in &candidates {
        let v_idx = poly_loop[*j];
        if !segment_intersects_any_loop(
            pts,
            pm,
            pts[v_idx],
            &all_loops,
            Some(hole_m_idx),
            Some(v_idx),
        ) {
            return pick_best_copy(*j);
        }
    }
    pick_best_copy(candidates.first().map(|c| c.0).unwrap_or(0))
}

fn min_tri_angle_cos(a: Vec2, b: Vec2, c: Vec2) -> f64 {
    let ab = (b - a).as_dvec2();
    let bc = (c - b).as_dvec2();
    let ca = (a - c).as_dvec2();
    let l1 = ab.length();
    let l2 = bc.length();
    let l3 = ca.length();
    if l1 < 1e-8 || l2 < 1e-8 || l3 < 1e-8 {
        return 1.0;
    }
    let cos_a = (ab.dot(-ca) / (l1 * l3)).clamp(-1.0, 1.0);
    let cos_b = ((-ab).dot(bc) / (l1 * l2)).clamp(-1.0, 1.0);
    let cos_c = ((-bc).dot(ca) / (l2 * l3)).clamp(-1.0, 1.0);
    // Menor ángulo => MAYOR coseno; buscamos MINIMIZAR el max(cos)
    cos_a.max(cos_b).max(cos_c)
}

/// Triangulación 2D exacta de un polígono arbitrario (cóncavo / en U) con 0 o más agujeros.
/// Índice `0..outer.len()` corresponde a `outer`; cada agujero continúa secuencialmente.
/// Todos los triángulos devueltos están orientados CCW en 2D (`orient_2d >= 0`).
pub fn triangulate_polygon_2d_with_holes(outer: &[Vec2], holes: &[Vec<Vec2>]) -> Vec<[usize; 3]> {
    if outer.len() < 3 {
        return Vec::new();
    }
    let mut pts: Vec<Vec2> = Vec::with_capacity(outer.len() + holes.iter().map(|h| h.len()).sum::<usize>());
    pts.extend_from_slice(outer);
    let mut hole_starts = Vec::with_capacity(holes.len());
    for h in holes {
        hole_starts.push(pts.len());
        pts.extend_from_slice(h);
    }

    let mut poly_loop = dedup_loop_indices(&pts, (0..outer.len()).collect());
    if poly_loop.len() < 3 {
        return Vec::new();
    }
    if signed_area_2d(&pts, &poly_loop) < 0.0 {
        poly_loop.reverse();
    }

    // Recopilar aristas originales de contorno (prohibidas para flips Delaunay)
    let mut boundary_edges: HashSet<(usize, usize)> = HashSet::new();
    let record_edges = |lp: &[usize], set: &mut HashSet<(usize, usize)>| {
        for i in 0..lp.len() {
            let u = lp[i];
            let v = lp[(i + 1) % lp.len()];
            set.insert(if u < v { (u, v) } else { (v, u) });
        }
    };
    record_edges(&poly_loop, &mut boundary_edges);

    let mut prep_holes: Vec<(f32, usize, Vec<usize>)> = Vec::new();
    for (hi, h) in holes.iter().enumerate() {
        let start = hole_starts[hi];
        let mut h_lp = dedup_loop_indices(&pts, (start..start + h.len()).collect());
        if h_lp.len() < 3 {
            continue;
        }
        // Los agujeros deben ir CW en 2D para que al empalmar queden consistentes con el contorno CCW
        if signed_area_2d(&pts, &h_lp) > 0.0 {
            h_lp.reverse();
        }
        record_edges(&h_lp, &mut boundary_edges);

        let mut max_x = f32::NEG_INFINITY;
        let mut m_pos = 0usize;
        for (idx_pos, &vi) in h_lp.iter().enumerate() {
            if pts[vi].x > max_x {
                max_x = pts[vi].x;
                m_pos = idx_pos;
            }
        }
        prep_holes.push((max_x, m_pos, h_lp));
    }

    // Ordenar agujeros por max_x descendente (Eberly) y empalmarlos en `poly_loop`
    prep_holes.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap_or(std::cmp::Ordering::Equal));
    for h_i in 0..prep_holes.len() {
        let m_pos = prep_holes[h_i].1;
        let h_lp = &prep_holes[h_i].2;
        let hole_m_idx = h_lp[m_pos];
        let rem_refs: Vec<&[usize]> = prep_holes[(h_i + 1)..]
            .iter()
            .map(|x| x.2.as_slice())
            .collect();
        let vis_pos = find_visible_outer_vertex(&pts, &poly_loop, hole_m_idx, &rem_refs);
        let vis_idx = poly_loop[vis_pos];

        let hn = h_lp.len();
        let mut splice = Vec::with_capacity(hn + 2);
        for k in 0..hn {
            splice.push(h_lp[(m_pos + k) % hn]);
        }
        splice.push(hole_m_idx);
        splice.push(vis_idx);

        let mut next_poly = Vec::with_capacity(poly_loop.len() + splice.len());
        next_poly.extend_from_slice(&poly_loop[..=vis_pos]);
        next_poly.extend_from_slice(&splice);
        next_poly.extend_from_slice(&poly_loop[(vis_pos + 1)..]);
        poly_loop = next_poly;
    }

    // Ear-clipping robusto con selección por calidad angular y listas doblemente enlazadas
    let m = poly_loop.len();
    if m < 3 {
        return Vec::new();
    }
    let mut prev: Vec<usize> = (0..m).map(|i| if i == 0 { m - 1 } else { i - 1 }).collect();
    let mut next: Vec<usize> = (0..m).map(|i| (i + 1) % m).collect();
    let mut alive = vec![true; m];
    let mut rem = m;
    let mut tris: Vec<[usize; 3]> = Vec::with_capacity(m.saturating_sub(2));

    while rem > 3 {
        let mut best_ear: Option<(usize, f64)> = None;

        for area_thresh in [1e-8_f64, -1e-9_f64] {
            for cur in 0..m {
                if !alive[cur] {
                    continue;
                }
                let ip = prev[cur];
                let inx = next[cur];
                let va = poly_loop[ip];
                let vb = poly_loop[cur];
                let vc = poly_loop[inx];
                if va == vb || vb == vc || vc == va {
                    continue;
                }
                let pa = pts[va];
                let pb = pts[vb];
                let pc = pts[vc];
                let area2 = orient_2d(pa, pb, pc);
                if area2 <= area_thresh {
                    continue;
                }

                let mut contains_other = false;
                let mut walk = next[inx];
                while walk != ip {
                    let vw = poly_loop[walk];
                    if vw != va && vw != vb && vw != vc {
                        let pw = pts[vw];
                        if pw.distance_squared(pa) > 1e-11
                            && pw.distance_squared(pb) > 1e-11
                            && pw.distance_squared(pc) > 1e-11
                            && point_in_triangle_2d_inclusive(pw, pa, pb, pc)
                        {
                            contains_other = true;
                            break;
                        }
                    }
                    walk = next[walk];
                }
                if !contains_other {
                    let q_cost = min_tri_angle_cos(pa, pb, pc);
                    if best_ear.map(|(_, bc)| q_cost < bc).unwrap_or(true) {
                        best_ear = Some((cur, q_cost));
                    }
                }
            }
            if best_ear.is_some() {
                break;
            }
        }

        let clip_cur = if let Some((best_i, _)) = best_ear {
            best_i
        } else {
            // Pasada de respaldo para vértices colineales / puentes repetidos
            let mut fallback = None;
            let mut best_score = f64::NEG_INFINITY;
            for cur in 0..m {
                if !alive[cur] {
                    continue;
                }
                let va = poly_loop[prev[cur]];
                let vb = poly_loop[cur];
                let vc = poly_loop[next[cur]];
                let area2 = orient_2d(pts[va], pts[vb], pts[vc]);
                if area2 >= -1e-6 && area2 > best_score {
                    best_score = area2;
                    fallback = Some(cur);
                }
            }
            match fallback {
                Some(idx) => idx,
                None => break,
            }
        };

        let ip = prev[clip_cur];
        let inx = next[clip_cur];
        let va = poly_loop[ip];
        let vb = poly_loop[clip_cur];
        let vc = poly_loop[inx];
        if va != vb && vb != vc && vc != va {
            tris.push([va, vb, vc]);
        }
        alive[clip_cur] = false;
        next[ip] = inx;
        prev[inx] = ip;
        rem -= 1;
    }

    if rem == 3 {
        if let Some(first) = (0..m).find(|&i| alive[i]) {
            let va = poly_loop[first];
            let vb = poly_loop[next[first]];
            let vc = poly_loop[next[next[first]]];
            if va != vb && vb != vc && vc != va {
                tris.push([va, vb, vc]);
            }
        }
    }

    // Optimizador Delaunay por intercambio de diagonales en aristas estrictamente interiores
    if tris.len() >= 2 {
        for _pass in 0..6 {
            let mut edge_to_tris: HashMap<(usize, usize), Vec<usize>> = HashMap::with_capacity(tris.len() * 2);
            for (ti, &[a, b, c]) in tris.iter().enumerate() {
                for (u, v) in [(a, b), (b, c), (c, a)] {
                    let k = if u < v { (u, v) } else { (v, u) };
                    edge_to_tris.entry(k).or_default().push(ti);
                }
            }
            let mut flipped_any = false;
            let mut touched_tris = vec![false; tris.len()];
            for (&(eu, ev), tlist) in &edge_to_tris {
                if boundary_edges.contains(&(eu, ev)) || tlist.len() != 2 {
                    continue;
                }
                let t0 = tlist[0];
                let t1 = tlist[1];
                if touched_tris[t0] || touched_tris[t1] {
                    continue;
                }
                let tri0 = tris[t0];
                let tri1 = tris[t1];
                let opp0 = tri0.iter().copied().find(|&x| x != eu && x != ev);
                let opp1 = tri1.iter().copied().find(|&x| x != eu && x != ev);
                let (Some(d0), Some(d1)) = (opp0, opp1) else {
                    continue;
                };
                if d0 == d1 {
                    continue;
                }
                // Ordenar d0 -> p_u -> d1 -> p_v tal que (d0, eu, ev) es CCW
                let (ea, eb) = if (tri0[0] == eu && tri0[1] == ev)
                    || (tri0[1] == eu && tri0[2] == ev)
                    || (tri0[2] == eu && tri0[0] == ev)
                {
                    (eu, ev)
                } else {
                    (ev, eu)
                };
                // tri0 = CCW(ea, eb, d0), tri1 = CCW(eb, ea, d1)
                let pa = pts[ea];
                let pb = pts[eb];
                let p0 = pts[d0];
                let p1 = pts[d1];
                // Verificar estricta convexidad del cuadrilátero (ea, d1, eb, d0)
                if orient_2d(p0, p1, pa) <= 1e-8 || orient_2d(p0, p1, pb) >= -1e-8 {
                    continue;
                }
                let before_q = min_tri_angle_cos(pa, pb, p0).max(min_tri_angle_cos(pb, pa, p1));
                let after_q = min_tri_angle_cos(p0, p1, pb).max(min_tri_angle_cos(p1, p0, pa));
                if after_q + 1e-4 < before_q {
                    tris[t0] = [d0, ea, d1];
                    tris[t1] = [d1, eb, d0];
                    touched_tris[t0] = true;
                    touched_tris[t1] = true;
                    flipped_any = true;
                }
            }
            if !flipped_any {
                break;
            }
        }
    }

    tris
}

/// Orienta todas las caras de una malla de forma topológicamente consistente mediante BFS
/// por aristas compartidas y elimina aletas non-manifold si las hubiera.
fn orient_mesh_manifold_consistent(mesh: &mut TriMeshData) {
    if mesh.indices.is_empty() {
        return;
    }
    let n_v = mesh.positions.len();

    // Eliminar triángulos finos / duplicados topológicos que causen >2 caras por arista
    let mut edge_faces: HashMap<(u32, u32), Vec<usize>> = HashMap::with_capacity(mesh.indices.len() * 3);
    let mut face_areas: Vec<f32> = Vec::with_capacity(mesh.indices.len());
    for (fi, &[a, b, c]) in mesh.indices.iter().enumerate() {
        let area = if (a as usize) < n_v && (b as usize) < n_v && (c as usize) < n_v {
            (mesh.positions[b as usize] - mesh.positions[a as usize])
                .cross(mesh.positions[c as usize] - mesh.positions[a as usize])
                .length()
        } else {
            0.0
        };
        face_areas.push(area);
        for (u, v) in [(a, b), (b, c), (c, a)] {
            let k = if u < v { (u, v) } else { (v, u) };
            edge_faces.entry(k).or_default().push(fi);
        }
    }

    let mut drop_face = vec![false; mesh.indices.len()];
    for list in edge_faces.values() {
        if list.len() > 2 {
            let mut sorted = list.clone();
            sorted.sort_by(|&i, &j| {
                face_areas[j]
                    .partial_cmp(&face_areas[i])
                    .unwrap_or(std::cmp::Ordering::Equal)
            });
            for &extra in &sorted[2..] {
                drop_face[extra] = true;
            }
        }
    }
    if drop_face.iter().any(|&d| d) {
        let mut kept = Vec::with_capacity(mesh.indices.len());
        for (fi, &tri) in mesh.indices.iter().enumerate() {
            if !drop_face[fi] {
                kept.push(tri);
            }
        }
        mesh.indices = kept;
    }

    // BFS de consistencia de devanado por componente conexa
    let nf = mesh.indices.len();
    if nf == 0 {
        return;
    }
    let mut e2f: HashMap<(u32, u32), Vec<usize>> = HashMap::with_capacity(nf * 3);
    for (fi, &[a, b, c]) in mesh.indices.iter().enumerate() {
        for (u, v) in [(a, b), (b, c), (c, a)] {
            let k = if u < v { (u, v) } else { (v, u) };
            e2f.entry(k).or_default().push(fi);
        }
    }

    let mut visited = vec![false; nf];
    let mut queue = VecDeque::new();
    for start in 0..nf {
        if visited[start] {
            continue;
        }
        visited[start] = true;
        queue.push_back(start);
        while let Some(fi) = queue.pop_front() {
            let [a, b, c] = mesh.indices[fi];
            for (u, v) in [(a, b), (b, c), (c, a)] {
                let k = if u < v { (u, v) } else { (v, u) };
                if let Some(neighs) = e2f.get(&k) {
                    for &nj in neighs {
                        if visited[nj] {
                            continue;
                        }
                        let [na, nb, nc] = mesh.indices[nj];
                        let same_dir = (na == u && nb == v) || (nb == u && nc == v) || (nc == u && na == v);
                        if same_dir {
                            mesh.indices[nj].swap(1, 2);
                        }
                        visited[nj] = true;
                        queue.push_back(nj);
                    }
                }
            }
        }
    }
    mesh.compute_normals();
}

/// Extrae los bucles de contorno abiertos ordenados `(u0 -> u1 -> ... -> uk -> u0)` siguiendo las
/// semiaristas abiertas del `TriMeshData`. Maneja vértices con múltiples aristas salientes (pinch).
pub fn extract_ordered_boundary_loops(mesh: &TriMeshData) -> Vec<Vec<u32>> {
    let mut edge_count: HashMap<(u32, u32), usize> = HashMap::with_capacity(mesh.indices.len() * 2);
    let mut directed: Vec<(u32, u32)> = Vec::with_capacity(mesh.indices.len() * 3);

    for &[a, b, c] in &mesh.indices {
        for (u, v) in [(a, b), (b, c), (c, a)] {
            directed.push((u, v));
            let k = if u < v { (u, v) } else { (v, u) };
            *edge_count.entry(k).or_insert(0) += 1;
        }
    }

    let mut boundary_edges: Vec<(u32, u32)> = directed
        .into_iter()
        .filter(|&(u, v)| {
            let k = if u < v { (u, v) } else { (v, u) };
            edge_count.get(&k).copied().unwrap_or(0) == 1
        })
        .collect();

    if boundary_edges.is_empty() {
        return Vec::new();
    }

    let mut out_map: HashMap<u32, Vec<usize>> = HashMap::new();
    for (ei, &(u, _)) in boundary_edges.iter().enumerate() {
        out_map.entry(u).or_default().push(ei);
    }

    let mut used = vec![false; boundary_edges.len()];
    let mut loops = Vec::new();

    for start_ei in 0..boundary_edges.len() {
        if used[start_ei] {
            continue;
        }
        let mut lp = Vec::new();
        let (start_u, mut cur_v) = boundary_edges[start_ei];
        let mut prev_u = start_u;
        used[start_ei] = true;
        lp.push(start_u);

        let mut guard = 0usize;
        while cur_v != start_u && guard <= boundary_edges.len() {
            guard += 1;
            lp.push(cur_v);
            let Some(cands) = out_map.get(&cur_v) else {
                break;
            };
            let unused_cands: Vec<usize> = cands.iter().copied().filter(|&ei| !used[ei]).collect();
            if unused_cands.is_empty() {
                break;
            }
            let chosen_ei = if unused_cands.len() == 1 {
                unused_cands[0]
            } else {
                // Elegir la arista que continúa de manera más suave (menor giro) o vuelve a start_u
                let in_dir = (mesh.positions[cur_v as usize] - mesh.positions[prev_u as usize]).normalize_or_zero();
                let mut best_ei = unused_cands[0];
                let mut best_dot = f32::NEG_INFINITY;
                for &cei in &unused_cands {
                    let nv = boundary_edges[cei].1;
                    if nv == start_u && lp.len() >= 3 {
                        best_ei = cei;
                        break;
                    }
                    let out_dir = (mesh.positions[nv as usize] - mesh.positions[cur_v as usize]).normalize_or_zero();
                    let d = in_dir.dot(out_dir);
                    if d > best_dot {
                        best_dot = d;
                        best_ei = cei;
                    }
                }
                best_ei
            };
            used[chosen_ei] = true;
            prev_u = cur_v;
            cur_v = boundary_edges[chosen_ei].1;
        }

        if cur_v == start_u && lp.len() >= 3 {
            loops.push(lp);
        } else {
            // Desmarcar si no cerró (no debería ocurrir en mallas limpias)
            let _ = &mut boundary_edges;
        }
    }

    loops
}

fn loop_perimeter_3d(mesh: &TriMeshData, lp: &[u32]) -> f32 {
    if lp.len() < 2 {
        return 0.0;
    }
    let mut perim = 0.0_f32;
    for i in 0..lp.len() {
        let a = mesh.positions[lp[i] as usize];
        let b = mesh.positions[lp[(i + 1) % lp.len()] as usize];
        perim += a.distance(b);
    }
    perim
}

/// Cierra automáticamente agujeros internos (o TODOS los agujeros si `keep_largest_boundary_open == false`)
/// usando proyección plano de Newell + `triangulate_polygon_2d_with_holes` respetando el devanado exacto.
/// Devuelve `(malla_reparada, bucle_perimetral_externo_abierto)`.
pub fn close_small_holes_in_mesh(
    mesh: &TriMeshData,
    keep_largest_boundary_open: bool,
) -> (TriMeshData, Vec<u32>) {
    if mesh.is_empty() {
        return (mesh.clone(), Vec::new());
    }
    let mut clean = extract_largest_component(&mesh.weld_vertices_preserve_watertight(1e-4));
    orient_mesh_manifold_consistent(&mut clean);

    let loops = extract_ordered_boundary_loops(&clean);
    if loops.is_empty() {
        clean.ensure_outward_winding_if_closed();
        return (clean, Vec::new());
    }

    let mut largest_idx = 0usize;
    let mut largest_score = f32::NEG_INFINITY;
    for (li, lp) in loops.iter().enumerate() {
        let perim = loop_perimeter_3d(&clean, lp);
        if perim > largest_score {
            largest_score = perim;
            largest_idx = li;
        }
    }

    for (li, lp) in loops.iter().enumerate() {
        if keep_largest_boundary_open && li == largest_idx {
            continue;
        }
        if lp.len() == 3 {
            // Semiaristas del bucle son (u->v), la tapa necesita (v->u): [lp[0], lp[2], lp[1]]
            clean.indices.push([lp[0], lp[2], lp[1]]);
            continue;
        }
        // Plano de Newell del bucle cerrado para proyectar limpiamente a 2D
        let mut newell_n = Vec3::ZERO;
        let mut ctr = Vec3::ZERO;
        for i in 0..lp.len() {
            let p = clean.positions[lp[i] as usize];
            let q = clean.positions[lp[(i + 1) % lp.len()] as usize];
            ctr += p;
            newell_n.x += (p.y - q.y) * (p.z + q.z);
            newell_n.y += (p.z - q.z) * (p.x + q.x);
            newell_n.z += (p.x - q.x) * (p.y + q.y);
        }
        ctr /= lp.len() as f32;
        let axis = if newell_n.length_squared() > 1e-8 {
            newell_n.normalize()
        } else {
            Vec3::Z
        };
        let (u_ax, v_ax, _) = build_orthonormal_basis(axis);
        let pts_2d: Vec<Vec2> = lp
            .iter()
            .map(|&vi| {
                let d = clean.positions[vi as usize] - ctr;
                Vec2::new(d.dot(u_ax), d.dot(v_ax))
            })
            .collect();

        let area_2d = signed_area_2d(&pts_2d, &(0..pts_2d.len()).collect::<Vec<_>>());
        let cap_tris = triangulate_polygon_2d_with_holes(&pts_2d, &[]);
        // Si el contorno 2D en `pts_2d` es CCW (area_2d > 0), el bucle va (u->v) CCW, así que la tapa
        // debe tener semiaristas (v->u) = CW => invertir [a, c, b]. Si va CW (area_2d < 0), mantener CCW.
        let flip_cap = area_2d > 0.0;
        for [a, b, c] in cap_tris {
            if flip_cap {
                clean.indices.push([lp[a], lp[c], lp[b]]);
            } else {
                clean.indices.push([lp[a], lp[b], lp[c]]);
            }
        }
    }

    clean.compute_normals();
    if keep_largest_boundary_open {
        let rem_loops = extract_ordered_boundary_loops(&clean);
        let outer = rem_loops
            .into_iter()
            .max_by(|a, b| {
                loop_perimeter_3d(&clean, a)
                    .partial_cmp(&loop_perimeter_3d(&clean, b))
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
            .unwrap_or_default();
        (clean, outer)
    } else {
        clean.ensure_outward_winding_if_closed();
        (clean, Vec::new())
    }
}

/// Desenreda cualquier auto-intersección 2D en un contorno gingival enrollado antes de extruir zócalo.
fn untangle_2d_footprint(raw_xy: &[Vec2]) -> Vec<Vec2> {
    let mut xy = raw_xy.to_vec();
    let n = xy.len();
    if n < 4 {
        return xy;
    }
    // Evitar aristas de longitud nula
    for i in 0..n {
        let nxt = (i + 1) % n;
        if xy[i].distance_squared(xy[nxt]) < 1e-8 {
            let prev = (i + n - 1) % n;
            let mid = 0.5 * (xy[prev] + xy[nxt]);
            let tangent = (xy[nxt] - xy[prev]).normalize_or_zero();
            let perp = Vec2::new(-tangent.y, tangent.x);
            xy[i] = mid + perp * 0.015;
        }
    }

    for _iter in 0..16 {
        let mut crossing_verts = vec![false; n];
        let mut any_cross = false;
        for i in 0..n {
            let i2 = (i + 1) % n;
            for j in (i + 2)..n {
                if i == 0 && j + 1 == n {
                    continue;
                }
                let j2 = (j + 1) % n;
                if segments_strict_intersect_2d(xy[i], xy[i2], xy[j], xy[j2]) {
                    any_cross = true;
                    crossing_verts[i] = true;
                    crossing_verts[i2] = true;
                    crossing_verts[j] = true;
                    crossing_verts[j2] = true;
                }
            }
        }
        if !any_cross {
            break;
        }
        let mut next_xy = xy.clone();
        for i in 0..n {
            let prev = (i + n - 1) % n;
            let nxt = (i + 1) % n;
            if crossing_verts[i] || crossing_verts[prev] || crossing_verts[nxt] {
                next_xy[i] = 0.30 * xy[prev] + 0.40 * xy[i] + 0.30 * xy[nxt];
            }
        }
        xy = next_xy;
    }
    xy
}

/// Construye la envolvente externa (escaneo + faldón lateral hasta `target_z`) sin tapa basal plana,
/// devolviendo `(outer_shell, target_z, outer_base_ring_vertex_indices, outer_base_xy)`.
fn build_outer_skirt_shell(
    mesh: &TriMeshData,
    base_thickness_mm: f32,
    direction_down: bool,
) -> Result<(TriMeshData, f32, Vec<u32>, Vec<Vec2>), String> {
    if mesh.is_empty() {
        return Err("La malla activa está vacía.".to_string());
    }
    let mut clean = extract_largest_component(&mesh.weld_vertices_preserve_watertight(1e-4));
    orient_mesh_manifold_consistent(&mut clean);

    // Detectar primero si la malla YA posee una tapa basal plana (aunque el escaneo tenga pequeños defectos lejanos)
    {
        let (mn0, mx0) = clean.bounds();
        let ext_z = if direction_down { mn0.z } else { mx0.z };
        let mut flat_count = 0usize;
        let mut flat_area = 0.0_f32;
        let mut stripped_tris = Vec::with_capacity(clean.indices.len());
        for &[a, b, c] in &clean.indices {
            let pa = clean.positions[a as usize];
            let pb = clean.positions[b as usize];
            let pc = clean.positions[c as usize];
            if (pa.z - ext_z).abs() < 0.12
                && (pb.z - ext_z).abs() < 0.12
                && (pc.z - ext_z).abs() < 0.12
            {
                flat_count += 1;
                let area2d = 0.5 * ((pb.x - pa.x) * (pc.y - pa.y) - (pb.y - pa.y) * (pc.x - pa.x)).abs();
                flat_area += area2d;
            } else {
                stripped_tris.push([a, b, c]);
            }
        }
        if flat_count >= 6 && flat_area > 50.0 && stripped_tris.len() >= 12 {
            clean.indices = stripped_tris;
            clean = clean.compact_unreferenced_vertices();
            orient_mesh_manifold_consistent(&mut clean);
            let avg_nz: f32 = clean.normals.iter().map(|n| n.z).sum::<f32>() / clean.normals.len().max(1) as f32;
            let expect_pos_z = direction_down;
            if (expect_pos_z && avg_nz < -0.02) || (!expect_pos_z && avg_nz > 0.02) {
                clean.flip_faces();
            }
            let (mut shell, fallback_loop) = close_small_holes_in_mesh(&clean, true);
            let loops = extract_ordered_boundary_loops(&shell);
            let chosen_loop = loops
                .into_iter()
                .filter(|lp| {
                    lp.len() >= 6 && {
                        let mean_diff = lp
                            .iter()
                            .map(|&vi| (shell.positions[vi as usize].z - ext_z).abs())
                            .sum::<f32>()
                            / lp.len() as f32;
                        mean_diff < 0.8
                    }
                })
                .max_by(|a, b| {
                    loop_perimeter_3d(&shell, a)
                        .partial_cmp(&loop_perimeter_3d(&shell, b))
                        .unwrap_or(std::cmp::Ordering::Equal)
                })
                .unwrap_or(fallback_loop);
            if chosen_loop.len() >= 3 {
                for &vi in &chosen_loop {
                    shell.positions[vi as usize].z = ext_z;
                }
                shell.compute_normals();
                let base_xy: Vec<Vec2> = chosen_loop
                    .iter()
                    .map(|&vi| Vec2::new(shell.positions[vi as usize].x, shell.positions[vi as usize].y))
                    .collect();
                return Ok((shell, ext_z, chosen_loop, base_xy));
            }
        }
    }

    // Orientar el escaneo abierto para que sus normales miren hacia afuera (+Z mandibular, -Z maxilar)
    let audit = clean.inspect_cam();
    if audit.open_edges == 0 {
        return Err(
            "El modelo ya es un sólido cerrado sin base plana. Aplica un recorte plano en Z antes de ahuecar."
                .to_string(),
        );
    }

    // Escaneo abierto: verificar dirección promedio de normales
    let avg_nz: f32 = clean.normals.iter().map(|n| n.z).sum::<f32>() / clean.normals.len().max(1) as f32;
    let expect_pos_z = direction_down;
    if (expect_pos_z && avg_nz < -0.02) || (!expect_pos_z && avg_nz > 0.02) {
        clean.flip_faces();
    }

    let (mut shell, outer_loop) = close_small_holes_in_mesh(&clean, true);
    if outer_loop.len() < 3 {
        return Err("No se encontró un perímetro abierto válido para generar el zócalo.".to_string());
    }

    let (mn, mx) = shell.bounds();
    let base_th = base_thickness_mm.abs().max(1.0);
    let target_z = if direction_down {
        mn.z - base_th
    } else {
        mx.z + base_th
    };

    let raw_xy: Vec<Vec2> = outer_loop
        .iter()
        .map(|&vi| Vec2::new(shell.positions[vi as usize].x, shell.positions[vi as usize].y))
        .collect();
    let clean_xy = untangle_2d_footprint(&raw_xy);
    let max_shift = raw_xy
        .iter()
        .zip(clean_xy.iter())
        .map(|(a, b)| a.distance(*b))
        .fold(0.0_f32, f32::max);

    let k = outer_loop.len();
    let use_mid_ring = max_shift > 0.30;
    let mut base_ring = Vec::with_capacity(k);

    if use_mid_ring {
        let mut mid_ring = Vec::with_capacity(k);
        for i in 0..k {
            let p_top = shell.positions[outer_loop[i] as usize];
            let p_mid_xy = raw_xy[i].lerp(clean_xy[i], 0.5);
            let p_mid = Vec3::new(p_mid_xy.x, p_mid_xy.y, 0.5 * (p_top.z + target_z));
            let id = shell.positions.len() as u32;
            shell.positions.push(p_mid);
            mid_ring.push(id);
        }
        for i in 0..k {
            let j = (i + 1) % k;
            let u = outer_loop[i];
            let v = outer_loop[j];
            let um = mid_ring[i];
            let vm = mid_ring[j];
            shell.indices.push([v, u, um]);
            shell.indices.push([v, um, vm]);
        }
        for i in 0..k {
            let id = shell.positions.len() as u32;
            shell.positions.push(Vec3::new(clean_xy[i].x, clean_xy[i].y, target_z));
            base_ring.push(id);
        }
        for i in 0..k {
            let j = (i + 1) % k;
            let um = mid_ring[i];
            let vm = mid_ring[j];
            let ub = base_ring[i];
            let vb = base_ring[j];
            shell.indices.push([vm, um, ub]);
            shell.indices.push([vm, ub, vb]);
        }
    } else {
        for i in 0..k {
            let id = shell.positions.len() as u32;
            shell.positions.push(Vec3::new(clean_xy[i].x, clean_xy[i].y, target_z));
            base_ring.push(id);
        }
        for i in 0..k {
            let j = (i + 1) % k;
            let u = outer_loop[i];
            let v = outer_loop[j];
            let ub = base_ring[i];
            let vb = base_ring[j];
            shell.indices.push([v, u, ub]);
            shell.indices.push([v, ub, vb]);
        }
    }

    shell.compute_normals();
    Ok((shell, target_z, base_ring, clean_xy))
}

/// Rebase de Zócalo Dental Sólido Imprimible (100% libre de geometrías conflictivas):
/// Repara agujeros internos, suaviza rebordes encrespados y triangula la tapa basal
/// con ear-clipping 2D cóncavo + optimización Delaunay (jamás cruza en abanico sobre arcos en U).
pub fn create_solid_base(mesh: &TriMeshData, base_thickness_mm: f32, direction_down: bool) -> TriMeshData {
    if mesh.is_empty() {
        return mesh.clone();
    }
    let check_audit = mesh.inspect_cam();
    if check_audit.open_edges == 0 && check_audit.watertight {
        let mut out = mesh.clone();
        out.ensure_outward_winding_if_closed();
        return out;
    }

    let Ok((mut out, _target_z, base_ring, base_xy)) =
        build_outer_skirt_shell(mesh, base_thickness_mm, direction_down)
    else {
        return mesh.clone();
    };

    let cap_tris = triangulate_polygon_2d_with_holes(&base_xy, &[]);
    if !cap_tris.is_empty() {
        // El faldón termina en semiarista (ub -> vb); la tapa necesita (vb -> ub)
        let u0 = base_ring[0];
        let v0 = base_ring[1];
        let needs_flip = cap_tris.iter().any(|&[a, b, c]| {
            let ga = base_ring[a];
            let gb = base_ring[b];
            let gc = base_ring[c];
            (ga == u0 && gb == v0) || (gb == u0 && gc == v0) || (gc == u0 && ga == v0)
        });
        let flip = if needs_flip {
            true
        } else {
            let area = signed_area_2d(&base_xy, &(0..base_xy.len()).collect::<Vec<_>>());
            area > 0.0
        };
        for [a, b, c] in cap_tris {
            if flip {
                out.indices.push([base_ring[a], base_ring[c], base_ring[b]]);
            } else {
                out.indices.push([base_ring[a], base_ring[b], base_ring[c]]);
            }
        }
    }

    out = out.weld_vertices_preserve_watertight(1e-5);
    out.ensure_outward_winding_if_closed();
    out.compute_normals();
    out
}

fn point_in_poly_vec2(x: f32, y: f32, poly: &[Vec2]) -> bool {
    let n = poly.len();
    if n < 3 {
        return false;
    }
    let mut inside = false;
    let mut j = n - 1;
    for i in 0..n {
        let pi = poly[i];
        let pj = poly[j];
        if ((pi.y > y) != (pj.y > y)) && (x < (pj.x - pi.x) * (y - pi.y) / (pj.y - pi.y + 1e-12) + pi.x) {
            inside = !inside;
        }
        j = i;
    }
    inside
}

/// Genera un Modelo Hollow Imprimible con pared controlada (`1.0 ..= 3.0 mm`) mediante campo SDF exacto
/// y extracción Kuhn Marching-Tetrahedra sin auto-intersecciones ni bolsas internas cerradas.
/// Devuelve `(malla_final, hollow_aplicado, mensaje_descriptivo)`.
pub fn create_hollow_base_checked(
    mesh: &TriMeshData,
    base_thickness_mm: f32,
    wall_thickness_mm: f32,
    direction_down: bool,
) -> (TriMeshData, bool, String) {
    if mesh.is_empty() {
        return (mesh.clone(), false, "La malla activa está vacía.".to_string());
    }
    let wall = wall_thickness_mm.clamp(1.0, 3.0);

    let (outer_shell, target_z, outer_base_ring, outer_base_xy) =
        match build_outer_skirt_shell(mesh, base_thickness_mm, direction_down) {
            Ok(res) => res,
            Err(e) => {
                return (
                    create_solid_base(mesh, base_thickness_mm, direction_down),
                    false,
                    e,
                );
            }
        };

    let outer_audit = outer_shell.inspect_cam();
    let pre_scan_open = outer_audit.open_edges.saturating_sub(outer_base_ring.len());
    let pre_scan_nm = outer_audit.non_manifold_edges;

    let Some(shell_col) = MeshCollider::from_mesh(&outer_shell) else {
        return (
            create_solid_base(mesh, base_thickness_mm, direction_down),
            false,
            "No se pudo construir el colisionador BVH del modelo.".to_string(),
        );
    };

    let (mn, mx) = outer_shell.bounds();
    let xy_span = (mx.x - mn.x).max(mx.y - mn.y);
    let step = (xy_span / 96.0).clamp(0.60, 0.90);
    let x0 = mn.x - step * 1.5;
    let y0 = mn.y - step * 1.5;
    let z_lo = target_z.min(mn.z);
    let z_hi = target_z.max(mx.z);
    let span_z = (z_hi - z_lo).max(step * 2.0);
    let nz = ((span_z / step).ceil() as usize + 2).clamp(6, 96);
    // Para mandible (direction_down = true): z0 = target_z (iz = 0 en plano base).
    // Para maxilla (direction_down = false): iz = nz - 1 está exactamente en target_z, manteniendo z creciente en +iz (mano derecha!).
    let dz = span_z / (nz - 2) as f32;
    let z0 = if direction_down {
        target_z
    } else {
        target_z - ((nz - 1) as f32) * dz
    };

    let nx = (((mx.x - mn.x + step * 3.0) / step).ceil() as usize + 1).clamp(8, 150);
    let ny = (((mx.y - mn.y + step * 3.0) / step).ceil() as usize + 1).clamp(8, 150);

    let mut xy_inside = vec![false; nx * ny];
    for ix in 0..nx {
        let x = x0 + (ix as f32) * step;
        for iy in 0..ny {
            let y = y0 + (iy as f32) * step;
            xy_inside[ix * ny + iy] = point_in_poly_vec2(x, y, &outer_base_xy);
        }
    }

    let grid_idx = |ix: usize, iy: usize, iz: usize| -> usize { (ix * ny + iy) * nz + iz };
    let grid_pos = |ix: usize, iy: usize, iz: usize| -> Vec3 {
        Vec3::new(
            x0 + (ix as f32) * step,
            y0 + (iy as f32) * step,
            z0 + (iz as f32) * dz,
        )
    };

    // Campo escalar phi: phi > 0 dentro de la cavidad hueca (d_to_outer > wall), phi < 0 en pared sólida / aire exterior
    let mut phi = vec![-wall; nx * ny * nz];
    let ray_dir = if direction_down { Vec3::Z } else { -Vec3::Z };
    let ray_len = span_z + 10.0;

    for ix in 1..(nx - 1) {
        let x = x0 + (ix as f32) * step;
        for iy in 1..(ny - 1) {
            let y = y0 + (iy as f32) * step;
            // Verificar si columna o algún vecino 4-conexo cae dentro del contorno basal
            let col_active = xy_inside[ix * ny + iy]
                || xy_inside[(ix - 1) * ny + iy]
                || xy_inside[(ix + 1) * ny + iy]
                || xy_inside[ix * ny + iy - 1]
                || xy_inside[ix * ny + iy + 1];
            if !col_active {
                continue;
            }

            // Lanzar UN rayo vertical desde el plano de base hacia la cara oclusal para paridad exacta en la columna
            let mut z_hits: Vec<f32> = Vec::with_capacity(8);
            if xy_inside[ix * ny + iy] {
                let mut cur_orig = Vec3::new(x, y, target_z) + ray_dir * 0.02;
                let mut rem_t = ray_len;
                for _ in 0..16 {
                    let ray = Ray::new(to_pvec(cur_orig), to_pvec(ray_dir));
                    if let Some(toi) = shell_col.bvh.cast_local_ray(&ray, rem_t, false) {
                        let hit_pt = cur_orig + ray_dir * toi;
                        if z_hits.last().map(|&pz| (pz - hit_pt.z).abs() > 1e-3).unwrap_or(true) {
                            z_hits.push(hit_pt.z);
                        }
                        let step_t = toi + 1e-3;
                        if step_t >= rem_t {
                            break;
                        }
                        cur_orig += ray_dir * step_t;
                        rem_t -= step_t;
                    } else {
                        break;
                    }
                }
            }

            let (iz_start, iz_end) = if direction_down {
                (0usize, nz - 1)
            } else {
                (1usize, nz)
            };
            for iz in iz_start..iz_end {
                let is_on_base = if direction_down { iz == 0 } else { iz == nz - 1 };
                let z = z0 + (iz as f32) * dz;
                let is_inside = if !xy_inside[ix * ny + iy] || z_hits.is_empty() {
                    false
                } else if is_on_base {
                    true
                } else {
                    let crossings_ahead = if direction_down {
                        z_hits.iter().filter(|&&hz| hz > z).count()
                    } else {
                        z_hits.iter().filter(|&&hz| hz < z).count()
                    };
                    crossings_ahead % 2 == 1
                };
                let d_outer = shell_col.closest_point_and_dist(Vec3::new(x, y, z)).1;
                let mut v = if is_inside { d_outer - wall } else { -d_outer - wall };
                if v.abs() < 1e-4 {
                    v = -1e-4;
                }
                phi[grid_idx(ix, iy, iz)] = v;
            }
        }
    }

    // Extraer isosuperficie phi == 0 mediante Kuhn 6-Tetrahedra Marching Tetrahedra (mano derecha estricta)
    let mut cav_pos: Vec<Vec3> = Vec::new();
    let mut cav_tris: Vec<[u32; 3]> = Vec::new();
    let mut edge_vert_cache: HashMap<(u64, u64), u32> = HashMap::new();

    let tets: [[usize; 4]; 6] = [
        [0, 1, 3, 7],
        [0, 3, 2, 7],
        [0, 2, 6, 7],
        [0, 6, 4, 7],
        [0, 4, 5, 7],
        [0, 5, 1, 7],
    ];

    let interp_vert = |ga: usize,
                       pa: Vec3,
                       va: f32,
                       gb: usize,
                       pb: Vec3,
                       vb: f32,
                       out_pos: &mut Vec<Vec3>,
                       cache: &mut HashMap<(u64, u64), u32>|
     -> u32 {
        let key = if ga < gb {
            (ga as u64, gb as u64)
        } else {
            (gb as u64, ga as u64)
        };
        *cache.entry(key).or_insert_with(|| {
            let denom = va - vb;
            let t = if denom.abs() > 1e-9 {
                (va / denom).clamp(0.02, 0.98)
            } else {
                0.5
            };
            let mut p = pa.lerp(pb, t);
            // Bloquear coordenadas en target_z solo cuando ambas puntas ya están en target_z
            if (pa.z - target_z).abs() < 1e-4 && (pb.z - target_z).abs() < 1e-4 {
                p.z = target_z;
            }
            let id = out_pos.len() as u32;
            out_pos.push(p);
            id
        })
    };

    let emit_oriented_tri = |a: u32,
                             b: u32,
                             c: u32,
                             inside_hint: Vec3,
                             pos: &[Vec3],
                             tris: &mut Vec<[u32; 3]>| {
        if a == b || b == c || c == a {
            return;
        }
        let pa = pos[a as usize];
        let pb = pos[b as usize];
        let pc = pos[c as usize];
        let n_tri = (pb - pa).cross(pc - pa);
        let ctr = (pa + pb + pc) / 3.0;
        // La normal del hueco debe apuntar hacia la cavidad (donde phi > 0, i.e., hacia el centro abierto)
        if n_tri.dot(inside_hint - ctr) >= 0.0 {
            tris.push([a, b, c]);
        } else {
            tris.push([a, c, b]);
        }
    };

    for ix in 0..(nx - 1) {
        for iy in 0..(ny - 1) {
            for iz in 0..(nz - 1) {
                let c_ids = [
                    grid_idx(ix, iy, iz),
                    grid_idx(ix + 1, iy, iz),
                    grid_idx(ix, iy + 1, iz),
                    grid_idx(ix + 1, iy + 1, iz),
                    grid_idx(ix, iy, iz + 1),
                    grid_idx(ix + 1, iy, iz + 1),
                    grid_idx(ix, iy + 1, iz + 1),
                    grid_idx(ix + 1, iy + 1, iz + 1),
                ];
                let mut has_pos = false;
                let mut has_neg = false;
                for &gid in &c_ids {
                    if phi[gid] > 0.0 {
                        has_pos = true;
                    } else {
                        has_neg = true;
                    }
                }
                if !has_pos || !has_neg {
                    continue;
                }
                let c_pos = [
                    grid_pos(ix, iy, iz),
                    grid_pos(ix + 1, iy, iz),
                    grid_pos(ix, iy + 1, iz),
                    grid_pos(ix + 1, iy + 1, iz),
                    grid_pos(ix, iy, iz + 1),
                    grid_pos(ix + 1, iy, iz + 1),
                    grid_pos(ix, iy + 1, iz + 1),
                    grid_pos(ix + 1, iy + 1, iz + 1),
                ];

                for &tet in &tets {
                    let mut in_v = Vec::with_capacity(4);
                    let mut out_v = Vec::with_capacity(4);
                    for &k in &tet {
                        let gid = c_ids[k];
                        if phi[gid] > 0.0 {
                            in_v.push(k);
                        } else {
                            out_v.push(k);
                        }
                    }
                    let hint_in: Vec3 = in_v.iter().map(|&k| c_pos[k]).sum::<Vec3>()
                        / in_v.len().max(1) as f32;

                    match in_v.len() {
                        0 | 4 => {}
                        1 => {
                            let i0 = in_v[0];
                            let (o0, o1, o2) = (out_v[0], out_v[1], out_v[2]);
                            let a = interp_vert(c_ids[i0], c_pos[i0], phi[c_ids[i0]], c_ids[o0], c_pos[o0], phi[c_ids[o0]], &mut cav_pos, &mut edge_vert_cache);
                            let b = interp_vert(c_ids[i0], c_pos[i0], phi[c_ids[i0]], c_ids[o1], c_pos[o1], phi[c_ids[o1]], &mut cav_pos, &mut edge_vert_cache);
                            let c = interp_vert(c_ids[i0], c_pos[i0], phi[c_ids[i0]], c_ids[o2], c_pos[o2], phi[c_ids[o2]], &mut cav_pos, &mut edge_vert_cache);
                            emit_oriented_tri(a, b, c, hint_in, &cav_pos, &mut cav_tris);
                        }
                        3 => {
                            let o0 = out_v[0];
                            let (i0, i1, i2) = (in_v[0], in_v[1], in_v[2]);
                            let a = interp_vert(c_ids[i0], c_pos[i0], phi[c_ids[i0]], c_ids[o0], c_pos[o0], phi[c_ids[o0]], &mut cav_pos, &mut edge_vert_cache);
                            let b = interp_vert(c_ids[i1], c_pos[i1], phi[c_ids[i1]], c_ids[o0], c_pos[o0], phi[c_ids[o0]], &mut cav_pos, &mut edge_vert_cache);
                            let c = interp_vert(c_ids[i2], c_pos[i2], phi[c_ids[i2]], c_ids[o0], c_pos[o0], phi[c_ids[o0]], &mut cav_pos, &mut edge_vert_cache);
                            emit_oriented_tri(a, b, c, hint_in, &cav_pos, &mut cav_tris);
                        }
                        2 => {
                            let (i0, i1) = (in_v[0], in_v[1]);
                            let (o0, o1) = (out_v[0], out_v[1]);
                            let v00 = interp_vert(c_ids[i0], c_pos[i0], phi[c_ids[i0]], c_ids[o0], c_pos[o0], phi[c_ids[o0]], &mut cav_pos, &mut edge_vert_cache);
                            let v01 = interp_vert(c_ids[i0], c_pos[i0], phi[c_ids[i0]], c_ids[o1], c_pos[o1], phi[c_ids[o1]], &mut cav_pos, &mut edge_vert_cache);
                            let v10 = interp_vert(c_ids[i1], c_pos[i1], phi[c_ids[i1]], c_ids[o0], c_pos[o0], phi[c_ids[o0]], &mut cav_pos, &mut edge_vert_cache);
                            let v11 = interp_vert(c_ids[i1], c_pos[i1], phi[c_ids[i1]], c_ids[o1], c_pos[o1], phi[c_ids[o1]], &mut cav_pos, &mut edge_vert_cache);
                            emit_oriented_tri(v00, v01, v11, hint_in, &cav_pos, &mut cav_tris);
                            emit_oriented_tri(v00, v11, v10, hint_in, &cav_pos, &mut cav_tris);
                        }
                        _ => {}
                    }
                }
            }
        }
    }

    if cav_tris.is_empty() {
        return (
            create_solid_base(mesh, base_thickness_mm, direction_down),
            false,
            format!(
                "El modelo es más delgado que {:.1} mm en toda su extensión; se generó zócalo sólido cerrado.",
                wall * 2.0
            ),
        );
    }

    let mut cav_mesh = TriMeshData::new(cav_pos, cav_tris).weld_vertices_preserve_watertight(1e-5);
    orient_mesh_manifold_consistent(&mut cav_mesh);

    // Conservar ÚNICAMENTE componentes conexas que abren hacia el plano base (evita bolsas internas atrapadas)
    let nv_c = cav_mesh.positions.len();
    let adj_c = cav_mesh.build_adjacency();
    let mut comp_c = vec![usize::MAX; nv_c];
    let mut vented_comps: HashSet<usize> = HashSet::new();
    let mut q = VecDeque::new();
    let mut cur_cid = 0usize;
    for start in 0..nv_c {
        if comp_c[start] != usize::MAX {
            continue;
        }
        let mut reaches_base = false;
        let mut v_count = 0usize;
        comp_c[start] = cur_cid;
        q.push_back(start);
        while let Some(u) = q.pop_front() {
            v_count += 1;
            if (cav_mesh.positions[u].z - target_z).abs() < 1e-3 {
                reaches_base = true;
            }
            for &v in &adj_c[u] {
                if comp_c[v] == usize::MAX {
                    comp_c[v] = cur_cid;
                    q.push_back(v);
                }
            }
        }
        if reaches_base && v_count >= 12 {
            vented_comps.insert(cur_cid);
        }
        cur_cid += 1;
    }

    if vented_comps.is_empty() {
        return (
            create_solid_base(mesh, base_thickness_mm, direction_down),
            false,
            format!(
                "El grosor disponible es insuficiente para abrir cavidad hollow de {:.1} mm; se generó base sólida.",
                wall
            ),
        );
    }

    let mut kept_tris = Vec::with_capacity(cav_mesh.indices.len());
    for &[a, b, c] in &cav_mesh.indices {
        if vented_comps.contains(&comp_c[a as usize]) {
            kept_tris.push([a, b, c]);
        }
    }
    cav_mesh = TriMeshData::new(cav_mesh.positions, kept_tris)
        .weld_vertices_preserve_watertight(1e-5)
        .compact_unreferenced_vertices();
    orient_mesh_manifold_consistent(&mut cav_mesh);

    // Suavizado Taubin gentil solo sobre los vértices internos de la cavidad (bloqueando borde en z = target_z)
    let adj_s = cav_mesh.build_adjacency();
    let n_s = cav_mesh.positions.len();
    let is_base_v: Vec<bool> = cav_mesh
        .positions
        .iter()
        .map(|p| (p.z - target_z).abs() < 1e-3)
        .collect();
    for i in 0..n_s {
        if is_base_v[i] {
            cav_mesh.positions[i].z = target_z;
        }
    }
    let mut tmp = cav_mesh.positions.clone();
    for _pass in 0..2 {
        for i in 0..n_s {
            if is_base_v[i] || adj_s[i].is_empty() {
                tmp[i] = cav_mesh.positions[i];
            } else {
                let avg: Vec3 = adj_s[i].iter().map(|&j| cav_mesh.positions[j]).sum::<Vec3>()
                    / adj_s[i].len() as f32;
                tmp[i] = cav_mesh.positions[i] + 0.35 * (avg - cav_mesh.positions[i]);
            }
        }
        for i in 0..n_s {
            if !is_base_v[i] && !adj_s[i].is_empty() {
                let avg: Vec3 = adj_s[i].iter().map(|&j| tmp[j]).sum::<Vec3>() / adj_s[i].len() as f32;
                cav_mesh.positions[i] = tmp[i] - 0.37 * (avg - tmp[i]);
            }
        }
    }
    cav_mesh.compute_normals();

    // Asegurar orientación de cavidad hacia el centro de drenaje de base
    let mut inner_loops = extract_ordered_boundary_loops(&cav_mesh);
    inner_loops.retain(|lp| lp.len() >= 3);
    if inner_loops.is_empty() {
        return (
            create_solid_base(mesh, base_thickness_mm, direction_down),
            false,
            "No se obtuvo un contorno de drenaje válido para la cavidad hollow; se generó base sólida.".to_string(),
        );
    }

    // Construir mapa único de índice local de contorno -> índice global tras fusionar outer + cavity
    let cav_offset = outer_shell.positions.len() as u32;
    let mut out_pos = outer_shell.positions.clone();
    out_pos.extend_from_slice(&cav_mesh.positions);

    let mut holes_xy: Vec<Vec<Vec2>> = Vec::with_capacity(inner_loops.len());
    for lp in &inner_loops {
        holes_xy.push(
            lp.iter()
                .map(|&vi| Vec2::new(cav_mesh.positions[vi as usize].x, cav_mesh.positions[vi as usize].y))
                .collect(),
        );
    }

    let rim_tris = triangulate_polygon_2d_with_holes(&outer_base_xy, &holes_xy);
    if rim_tris.is_empty() {
        return (
            create_solid_base(mesh, base_thickness_mm, direction_down),
            false,
            "Error al triangular anillo basal del hollow; se generó base sólida.".to_string(),
        );
    }

    // Tabla de mapeo de índice local de triangulador 2D -> índice en `out_pos`
    let mut flat_global_idx: Vec<u32> = Vec::with_capacity(outer_base_ring.len() + inner_loops.iter().map(|l| l.len()).sum::<usize>());
    flat_global_idx.extend_from_slice(&outer_base_ring);
    for lp in &inner_loops {
        for &vi in lp {
            flat_global_idx.push(vi + cav_offset);
        }
    }

    // Verificar orientación del anillo basal respecto a todas las semiaristas inferiores (ub -> vb) de outer_shell
    let mut outer_fwd_edges: HashSet<(u32, u32)> = HashSet::with_capacity(outer_base_ring.len());
    let mut outer_rev_edges: HashSet<(u32, u32)> = HashSet::with_capacity(outer_base_ring.len());
    for i in 0..outer_base_ring.len() {
        let u = outer_base_ring[i];
        let v = outer_base_ring[(i + 1) % outer_base_ring.len()];
        outer_fwd_edges.insert((u, v));
        outer_rev_edges.insert((v, u));
    }
    let mut fwd_hits = 0usize;
    let mut rev_hits = 0usize;
    for &[a, b, c] in &rim_tris {
        let ga = flat_global_idx[a];
        let gb = flat_global_idx[b];
        let gc = flat_global_idx[c];
        for (x, y) in [(ga, gb), (gb, gc), (gc, ga)] {
            if outer_fwd_edges.contains(&(x, y)) {
                fwd_hits += 1;
            }
            if outer_rev_edges.contains(&(x, y)) {
                rev_hits += 1;
            }
        }
    }
    let rim_needs_flip = fwd_hits > rev_hits;

    // Verificar también cada componente conexa de la cavidad contra el sentido del anillo basal
    let rim_directed: HashSet<(u32, u32)> = rim_tris
        .iter()
        .flat_map(|&[a, b, c]| {
            let (ia, ib, ic) = if rim_needs_flip {
                (flat_global_idx[a], flat_global_idx[c], flat_global_idx[b])
            } else {
                (flat_global_idx[a], flat_global_idx[b], flat_global_idx[c])
            };
            [(ia, ib), (ib, ic), (ic, ia)]
        })
        .collect();

    // Si alguna componente de `cav_mesh` tiene una arista en `inner_loops` en el MISMO sentido que el anillo, invertir esa componente
    let mut cav_tris_final = cav_mesh.indices.clone();
    {
        let cav_adj_tris = {
            let nf = cav_tris_final.len();
            let mut e2t: HashMap<(u32, u32), Vec<usize>> = HashMap::with_capacity(nf * 3);
            for (fi, &[a, b, c]) in cav_tris_final.iter().enumerate() {
                for (u, v) in [(a, b), (b, c), (c, a)] {
                    let k = if u < v { (u, v) } else { (v, u) };
                    e2t.entry(k).or_default().push(fi);
                }
            }
            e2t
        };
        let mut comp_id_f = vec![usize::MAX; cav_tris_final.len()];
        let mut qf = VecDeque::new();
        let mut cid = 0usize;
        for sf in 0..cav_tris_final.len() {
            if comp_id_f[sf] != usize::MAX {
                continue;
            }
            comp_id_f[sf] = cid;
            qf.push_back(sf);
            let mut faces_in_comp = Vec::new();
            let mut flip_this_comp = false;
            while let Some(fi) = qf.pop_front() {
                faces_in_comp.push(fi);
                let [a, b, c] = cav_tris_final[fi];
                for (u, v) in [(a, b), (b, c), (c, a)] {
                    if rim_directed.contains(&(u + cav_offset, v + cav_offset)) {
                        flip_this_comp = true;
                    }
                    let k = if u < v { (u, v) } else { (v, u) };
                    if let Some(neighs) = cav_adj_tris.get(&k) {
                        for &nj in neighs {
                            if comp_id_f[nj] == usize::MAX {
                                comp_id_f[nj] = cid;
                                qf.push_back(nj);
                            }
                        }
                    }
                }
            }
            if flip_this_comp {
                for fi in faces_in_comp {
                    cav_tris_final[fi].swap(1, 2);
                }
            }
            cid += 1;
        }
    }

    let mut out_tris = outer_shell.indices.clone();
    for [a, b, c] in cav_tris_final {
        out_tris.push([a + cav_offset, b + cav_offset, c + cav_offset]);
    }
    for [a, b, c] in rim_tris {
        if rim_needs_flip {
            out_tris.push([flat_global_idx[a], flat_global_idx[c], flat_global_idx[b]]);
        } else {
            out_tris.push([flat_global_idx[a], flat_global_idx[b], flat_global_idx[c]]);
        }
    }

    let mut final_mesh = TriMeshData::new(out_pos, out_tris)
        .weld_vertices_preserve_watertight(1e-5)
        .compact_unreferenced_vertices();
    final_mesh.ensure_outward_winding_if_closed();
    final_mesh.compute_normals();

    let final_audit = final_mesh.inspect_cam();
    let no_new_defects = final_audit.watertight
        || (final_audit.open_edges <= pre_scan_open + 8
            && final_audit.non_manifold_edges <= pre_scan_nm + 4);
    if no_new_defects && final_audit.volume_mm3.abs() > 10.0 {
        let status_tag = if final_audit.watertight {
            "Watertight ✔"
        } else {
            "Hollow generado ✔"
        };
        (
            final_mesh,
            true,
            format!(
                "Modelo Hollow ({:.1} mm pared) imprimible — {} ({} tris, {:.1} cm³).",
                wall,
                status_tag,
                final_audit.n_faces,
                final_audit.volume_mm3.abs() / 1000.0
            ),
        )
    } else {
        let solid = create_solid_base(mesh, base_thickness_mm, direction_down);
        (
            solid,
            false,
            "Se aplicó base sólida cerrada para garantizar cero conflictos de impresión.".to_string(),
        )
    }
}

/// Versión simplificada que devuelve directamente la malla `hollow` imprimible.
pub fn create_hollow_base(
    mesh: &TriMeshData,
    base_thickness_mm: f32,
    wall_thickness_mm: f32,
    direction_down: bool,
) -> TriMeshData {
    create_hollow_base_checked(mesh, base_thickness_mm, wall_thickness_mm, direction_down).0
}

/// Calcula el ángulo de expulsividad / socavado (en grados) respecto al eje de inserción.
/// Valores `> 0°` son zonas expulsivas; valores `< 0°` indican socavados retentivos.
pub fn compute_undercut_angles_deg(mesh: &TriMeshData, insertion_axis: Vec3) -> Vec<f32> {
    let ax = if insertion_axis.length_squared() > 1e-8 {
        insertion_axis.normalize()
    } else {
        Vec3::Z
    };
    mesh.normals
        .iter()
        .map(|&n| n.dot(ax).clamp(-1.0, 1.0).asin().to_degrees())
        .collect()
}

/// Calcula un mapa normalizado `[0.0 .. 1.0]` de curvatura superficial media discreta (|Laplaciano|)
/// para el imán cervical de línea de margen en el Módulo 3.
pub fn compute_mean_curvature_weights(mesh: &TriMeshData) -> Vec<f32> {
    let n = mesh.positions.len();
    if n == 0 {
        return Vec::new();
    }
    let adj = mesh.build_adjacency();
    let mut raw = vec![0.0_f32; n];
    for i in 0..n {
        if adj[i].is_empty() {
            continue;
        }
        let avg: Vec3 = adj[i].iter().map(|&j| mesh.positions[j]).sum::<Vec3>() / adj[i].len() as f32;
        raw[i] = (avg - mesh.positions[i]).length();
    }
    let mut sorted = raw.clone();
    sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let p98 = sorted[((sorted.len() as f32 * 0.98) as usize).min(sorted.len() - 1)].max(1e-5);
    raw.into_iter().map(|v| (v / p98).clamp(0.0, 1.0)).collect()
}

/// Genera una arcada dental sintética de demostración con muñón #36 tallado (o antagonista maxilar)
/// cuando los archivos reales `_demo_assets/` no están presentes.
pub fn generate_parametric_demo_arch(is_maxilla: bool) -> TriMeshData {
    let nu = 120usize;
    let nv = 54usize;
    let radius = 26.0_f32;
    let mut positions = Vec::with_capacity(nu * nv);

    for iv in 0..nv {
        let v = -6.5 + 13.0 * (iv as f32) / ((nv - 1) as f32);
        for iu in 0..nu {
            let u = -PI * 0.42 + (PI * 0.84) * (iu as f32) / ((nu - 1) as f32);
            let r = radius + v;
            let x = r * u.sin();
            let y = r * u.cos() - radius * 0.5;

            let gingiva = 5.5 * (-(v * v) / (2.0 * 3.8 * 3.8)).exp();
            let tooth_wave = 0.5 + 0.5 * (u * 7.0).cos();
            let crown_profile = 4.2 * tooth_wave.powf(1.6) * (-(v * v) / (2.0 * 2.4 * 2.4)).exp();
            let is_prep_zone = (-(u / 0.16).powi(2)).exp();
            let radial_dist = ((u * radius).powi(2) + v * v).sqrt();
            let shoulder = (1.0 - (radial_dist / 4.2).powi(4)).clamp(0.0, 1.0);
            let stump_top = 3.8 * (1.0 - (radial_dist / 3.1).powi(2)).clamp(0.0, 1.0);
            let prep_height = 1.2 * shoulder + stump_top;

            let mut z = gingiva + (1.0 - is_prep_zone) * crown_profile + is_prep_zone * prep_height;
            if is_maxilla {
                z = 16.0 - z;
            }
            positions.push(Vec3::new(x, y, z));
        }
    }

    let mut indices = Vec::with_capacity((nu - 1) * (nv - 1) * 2);
    for iv in 0..(nv - 1) {
        for iu in 0..(nu - 1) {
            let a = (iv * nu + iu) as u32;
            let b = (iv * nu + iu + 1) as u32;
            let c = ((iv + 1) * nu + iu) as u32;
            let d = ((iv + 1) * nu + iu + 1) as u32;
            if is_maxilla {
                indices.push([a, c, b]);
                indices.push([b, c, d]);
            } else {
                indices.push([a, b, c]);
                indices.push([b, d, c]);
            }
        }
    }
    TriMeshData::new(positions, indices)
}
