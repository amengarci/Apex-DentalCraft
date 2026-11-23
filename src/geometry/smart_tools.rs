//! Módulo 4: Motor de Herramientas Inteligentes (`SmartToolsEngine` estilo 3Shape)
//!
//! 1. `smart_occlusal_contacts`: Recortar interferencias / Adaptar cúspides a antagonista.
//! 2. `smart_proximal_contacts`: Detectar vecinos Mesial/Distal y ajustar contacto interproximal.
//! 3. `smart_enforce_min_thickness`: Refuerzo automático de espesor mínimo según material.
//! 4. `smart_anatomical_smooth`: Suavizado Taubin global preservando volumen y sellado marginal.
//! 5. `smart_emergence_profile`: Optimización del perfil de emergencia gingival en tercio cervical.
//! 6. `smart_adapt_pontic_to_gingiva`: Reducción y adaptación basal del póntico sobre la encía/reborde.

use crate::geometry::mesh::{build_orthonormal_basis, smoothstep, MeshCollider, TriMeshData};
use crate::geometry::sculpt::{collect_intaglio_pts, PointSpatialGrid};
use glam::{Vec2, Vec3};
use std::collections::BTreeMap;

pub struct SmartToolsEngine;

impl SmartToolsEngine {
    /// Recorta colisiones contra el antagonista (o adapta cúspides en infraoclusión leve).
    pub fn smart_occlusal_contacts(
        crown: &TriMeshData,
        antagonist: &TriMeshData,
        insertion_axis: Vec3,
        target_clearance_mm: f32,
        adapt_underocclusion: bool,
        min_thickness_mm: f32,
    ) -> TriMeshData {
        if crown.is_empty() || antagonist.is_empty() {
            return crown.clone();
        }
        let mut out = crown.clone();
        let n = out.positions.len();
        let has_ext = out.is_external.len() == n;
        let intaglio = collect_intaglio_pts(&out);
        let int_grid = (!intaglio.is_empty()).then(|| PointSpatialGrid::from_points(&intaglio, 0.65));

        let (u, v, a) = build_orthonormal_basis(insertion_axis);
        let origin = out.center();

        let ant_local_pos: Vec<Vec3> = antagonist
            .positions
            .iter()
            .map(|&p| {
                let r = p - origin;
                Vec3::new(r.dot(u), r.dot(v), r.dot(a))
            })
            .collect();
        let ant_local = TriMeshData::new(ant_local_pos, antagonist.indices.clone());
        let Some(ant_col) = MeshCollider::from_mesh(&ant_local) else {
            return out;
        };

        let mut local: Vec<Vec3> = out
            .positions
            .iter()
            .map(|&p| {
                let r = p - origin;
                Vec3::new(r.dot(u), r.dot(v), r.dot(a))
            })
            .collect();

        let mut z_min = f32::INFINITY;
        let mut z_max = f32::NEG_INFINITY;
        for i in 0..n {
            if !has_ext || out.is_external[i] {
                z_min = z_min.min(local[i].z);
                z_max = z_max.max(local[i].z);
            }
        }
        let z_span = (z_max - z_min).max(2.0);
        let mut clearances = vec![3.0_f32; n];
        let mut touched = Vec::new();
        let floor_th = min_thickness_mm * 0.90;

        for i in 0..n {
            if has_ext && !out.is_external[i] {
                continue;
            }
            let p = local[i];
            let z_norm = (p.z - z_min) / z_span;
            if z_norm < 0.38 {
                continue;
            }
            if let Some((_, hit)) = ant_col.cast_ray(Vec3::new(p.x, p.y, p.z - 4.5), Vec3::Z, 18.5) {
                let z_ant = hit.z;
                let mut z_target = z_ant - target_clearance_mm;
                if p.z > z_target {
                    if let Some(ig) = &int_grid {
                        let w_pt = origin + u * p.x + v * p.y + a * z_target;
                        if let Some((d_in, q_in)) = ig.nearest_within(w_pt, floor_th + 0.05) {
                            if d_in < floor_th {
                                let q_in_z = (q_in - origin).dot(a);
                                z_target = z_target.max(q_in_z + floor_th);
                            }
                        }
                    }
                    local[i].z = z_target;
                    touched.push(i);
                } else if adapt_underocclusion && (z_target - p.z) > 0.0 && (z_target - p.z) <= 0.85 && z_norm > 0.62 {
                    let blend = smoothstep(0.85, 0.10, z_target - p.z) * 0.78;
                    local[i].z = p.z + blend * (z_target - p.z);
                    touched.push(i);
                }
                clearances[i] = z_ant - local[i].z;
            }
        }

        for i in 0..n {
            out.positions[i] = origin + u * local[i].x + v * local[i].y + a * local[i].z;
        }

        if !touched.is_empty() {
            let adj = out.build_adjacency();
            for _ in 0..2 {
                let prev = out.positions.clone();
                for &ti in &touched {
                    if !adj[ti].is_empty() {
                        let avg: Vec3 = adj[ti].iter().map(|&j| prev[j]).sum::<Vec3>() / adj[ti].len() as f32;
                        out.positions[ti] = prev[ti] * 0.65 + avg * 0.35;
                    }
                }
            }
        }

        out.occlusal_clearance_mm = clearances;
        if let Some(ig) = &int_grid {
            if out.thickness_mm.len() == n {
                for i in 0..n {
                    if !has_ext || out.is_external[i] {
                        out.thickness_mm[i] = ig.nearest(out.positions[i]).0;
                    }
                }
            }
        }
        out.compute_normals();
        out
    }

    /// Detecta dientes vecinos Mesial/Distal sobre `prep_mesh` y ajusta el área de contacto proximal.
    pub fn smart_proximal_contacts(
        crown: &TriMeshData,
        prep_mesh: Option<&TriMeshData>,
        margin_curve: &[Vec3],
        insertion_axis: Vec3,
        other_restorations: &BTreeMap<i32, TriMeshData>,
        target_clearance_mm: f32,
        min_thickness_mm: f32,
    ) -> TriMeshData {
        if crown.is_empty() || margin_curve.len() < 4 {
            return crown.clone();
        }
        let mut out = crown.clone();
        let n = out.positions.len();
        let has_ext = out.is_external.len() == n;
        let intaglio = collect_intaglio_pts(&out);
        let int_grid = (!intaglio.is_empty()).then(|| PointSpatialGrid::from_points(&intaglio, 0.65));

        let (u, v, a) = build_orthonormal_basis(insertion_axis);
        let origin = margin_curve.iter().copied().sum::<Vec3>() / margin_curve.len() as f32;

        let mut m_rad_u = 2.5_f32;
        let mut m_rad_v = 2.5_f32;
        let mut m_z_min = f32::INFINITY;
        for &mp in margin_curve {
            let r = mp - origin;
            m_rad_u = m_rad_u.max(r.dot(u).abs());
            m_rad_v = m_rad_v.max(r.dot(v).abs());
            m_z_min = m_z_min.min(r.dot(a));
        }

        let mut neigh_pts: Vec<Vec3> = Vec::new();
        if let Some(prep) = prep_mesh {
            for &p in &prep.positions {
                let r = p - origin;
                let lx = r.dot(u);
                let ly = r.dot(v);
                let lz = r.dot(a);
                if (lx.abs() > m_rad_u * 1.14 || ly.abs() > m_rad_v * 1.18)
                    && lx.abs() < m_rad_u * 3.2
                    && ly.abs() < m_rad_v * 2.1
                    && lz > m_z_min + 0.6
                {
                    neigh_pts.push(Vec3::new(lx, ly, lz));
                }
            }
        }
        for other in other_restorations.values() {
            for &p in &other.positions {
                let r = p - origin;
                neigh_pts.push(Vec3::new(r.dot(u), r.dot(v), r.dot(a)));
            }
        }
        if neigh_pts.is_empty() {
            return out;
        }

        let mut local: Vec<Vec3> = out
            .positions
            .iter()
            .map(|&p| {
                let r = p - origin;
                Vec3::new(r.dot(u), r.dot(v), r.dot(a))
            })
            .collect();

        let mut z_min = f32::INFINITY;
        let mut z_max = f32::NEG_INFINITY;
        for i in 0..n {
            if !has_ext || out.is_external[i] {
                z_min = z_min.min(local[i].z);
                z_max = z_max.max(local[i].z);
            }
        }
        let z_span = (z_max - z_min).max(2.0);

        for sign_u in [1.0_f32, -1.0_f32] {
            let mut side_n_u: Vec<f32> = neigh_pts
                .iter()
                .filter(|p| {
                    p.x * sign_u > m_rad_u * 1.08
                        && p.x * sign_u < m_rad_u * 2.85
                        && p.y.abs() < m_rad_v * 1.05
                })
                .map(|p| p.x * sign_u)
                .collect();
            if side_n_u.len() < 12 {
                continue;
            }
            side_n_u.sort_by(|x, y| x.partial_cmp(y).unwrap_or(std::cmp::Ordering::Equal));
            let min_n_u = side_n_u[((side_n_u.len() as f32 * 0.04) as usize).min(side_n_u.len() - 1)];

            let mut side_c_u: Vec<f32> = Vec::new();
            for i in 0..n {
                if has_ext && !out.is_external[i] {
                    continue;
                }
                let p = local[i];
                let zn = ((p.z - z_min) / z_span).clamp(0.0, 1.0);
                if p.x * sign_u > m_rad_u * 0.55 && (0.35..=0.86).contains(&zn) && p.y.abs() <= m_rad_v * 0.65 {
                    side_c_u.push(p.x * sign_u);
                }
            }
            if side_c_u.is_empty() {
                continue;
            }
            side_c_u.sort_by(|x, y| x.partial_cmp(y).unwrap_or(std::cmp::Ordering::Equal));
            let max_c_u = side_c_u[((side_c_u.len() as f32 * 0.96) as usize).min(side_c_u.len() - 1)];
            let signed_gap = min_n_u - max_c_u;
            let shift = (signed_gap - target_clearance_mm).clamp(-0.65, 0.85);
            if shift.abs() < 0.005 {
                continue;
            }

            for i in 0..n {
                if has_ext && !out.is_external[i] {
                    continue;
                }
                let p = local[i];
                let zn = ((p.z - z_min) / z_span).clamp(0.0, 1.0);
                let m_lock = smoothstep(0.08, 0.35, zn);
                let u_w = smoothstep(m_rad_u * 0.35, m_rad_u * 0.95, p.x * sign_u);
                let z_w = (-9.0 * (zn - 0.62).powi(2)).exp();
                let v_w = (-3.5 * (p.y / (m_rad_v * 0.72).max(1.0)).powi(2)).exp();
                local[i].x += sign_u * shift * (u_w * z_w * v_w * m_lock);
            }
        }

        let has_th = out.thickness_mm.len() == n;
        for i in 0..n {
            let mut wp = origin + u * local[i].x + v * local[i].y + a * local[i].z;
            if let Some(ig) = &int_grid {
                if !has_ext || out.is_external[i] {
                    let zn = ((local[i].z - z_min) / z_span).clamp(0.0, 1.0);
                    let m_lock = smoothstep(0.08, 0.35, zn);
                    let req = min_thickness_mm * m_lock * 0.92;
                    let probe = (req + 0.05).max(0.10);
                    if let Some((di, q_in)) = ig.nearest_within(wp, probe) {
                        if di < req {
                            let push = (wp - q_in).normalize_or_zero();
                            if push.length_squared() > 1e-6 {
                                wp = q_in + push * req;
                                if has_th {
                                    out.thickness_mm[i] = req;
                                }
                            } else if has_th {
                                out.thickness_mm[i] = di;
                            }
                        } else if has_th {
                            out.thickness_mm[i] = di;
                        }
                    } else if has_th {
                        out.thickness_mm[i] = ig.nearest(wp).0;
                    }
                }
            }
            out.positions[i] = wp;
        }
        out.compute_normals();
        out
    }

    /// Refuerza suavemente todas las zonas externas por debajo de `min_thickness_mm`.
    pub fn smart_enforce_min_thickness(crown: &TriMeshData, min_thickness_mm: f32) -> TriMeshData {
        if crown.is_empty() {
            return crown.clone();
        }
        let mut out = crown.clone();
        let n = out.positions.len();
        let has_ext = out.is_external.len() == n;
        let intaglio = collect_intaglio_pts(&out);
        if intaglio.is_empty() {
            return out;
        }
        let int_grid = PointSpatialGrid::from_points(&intaglio, 0.65);

        let adj = out.build_adjacency();
        let mut deficit = vec![0.0_f32; n];
        let mut q_nearest = vec![Vec3::ZERO; n];
        let mut any_def = false;
        let probe_r = (min_thickness_mm + 0.15).max(0.8);

        for i in 0..n {
            if has_ext && !out.is_external[i] {
                continue;
            }
            if let Some((di, qi)) = int_grid.nearest_within(out.positions[i], probe_r) {
                q_nearest[i] = qi;
                let taper = smoothstep(0.08, 0.58, di);
                let def = (min_thickness_mm * taper - di).max(0.0);
                if def > 1e-4 {
                    deficit[i] = def;
                    any_def = true;
                }
            }
        }

        if any_def {
            for _ in 0..2 {
                let prev = deficit.clone();
                for i in 0..n {
                    if has_ext && !out.is_external[i] {
                        deficit[i] = 0.0;
                        continue;
                    }
                    if !adj[i].is_empty() {
                        let avg: f32 = adj[i].iter().map(|&j| prev[j]).sum::<f32>() / adj[i].len() as f32;
                        deficit[i] = prev[i].max(0.75 * avg);
                    }
                }
            }

            for i in 0..n {
                if deficit[i] > 1e-5 {
                    let out_v = (out.positions[i] - q_nearest[i]).normalize_or_zero();
                    let push = if out_v.length_squared() > 1e-6 {
                        (out_v * 0.65 + out.normals[i] * 0.35).normalize_or_zero()
                    } else {
                        out.normals[i]
                    };
                    out.positions[i] += push * deficit[i];
                }
            }

            // 1 pasada de relajación tangencial suave
            let prev = out.positions.clone();
            for i in 0..n {
                if (!has_ext || out.is_external[i]) && !adj[i].is_empty() {
                    let avg: Vec3 = adj[i].iter().map(|&j| prev[j]).sum::<Vec3>() / adj[i].len() as f32;
                    let lap = avg - prev[i];
                    let tan_lap = lap - out.normals[i] * lap.dot(out.normals[i]);
                    out.positions[i] = prev[i] + tan_lap * 0.25;
                }
            }
        }

        if out.thickness_mm.len() != n {
            out.thickness_mm = vec![0.0; n];
        }
        for i in 0..n {
            if !has_ext || out.is_external[i] {
                out.thickness_mm[i] = int_grid.nearest(out.positions[i]).0;
            }
        }
        out.compute_normals();
        out
    }

    /// Suavizado Anatómico Inteligente global (filtro Taubin sin pérdida de volumen ni margen).
    pub fn smart_anatomical_smooth(
        crown: &TriMeshData,
        iterations: usize,
        min_thickness_mm: f32,
    ) -> TriMeshData {
        if crown.is_empty() {
            return crown.clone();
        }
        let mut out = crown.clone();
        let n = out.positions.len();
        let has_ext = out.is_external.len() == n;
        let intaglio = collect_intaglio_pts(&out);
        let int_grid = (!intaglio.is_empty()).then(|| PointSpatialGrid::from_points(&intaglio, 0.65));
        let adj = out.build_adjacency();

        let mut base_d = vec![2.0_f32; n];
        let mut guard = vec![1.0_f32; n];
        if let Some(ig) = &int_grid {
            for i in 0..n {
                if !has_ext || out.is_external[i] {
                    let di = ig.nearest_within(out.positions[i], 1.45).map(|x| x.0).unwrap_or(1.45);
                    base_d[i] = di;
                    guard[i] = smoothstep(0.15, 0.55, di);
                } else {
                    guard[i] = 0.0;
                }
            }
        }

        let lam = 0.46_f32;
        let mu = -0.34_f32;
        for _ in 0..iterations.max(1) {
            let p1 = out.positions.clone();
            for i in 0..n {
                if guard[i] > 1e-4 && !adj[i].is_empty() {
                    let avg: Vec3 = adj[i].iter().map(|&j| p1[j]).sum::<Vec3>() / adj[i].len() as f32;
                    out.positions[i] = p1[i] + (avg - p1[i]) * (lam * guard[i]);
                }
            }
            let p2 = out.positions.clone();
            for i in 0..n {
                // Desactivar inflado Taubin negativo cerca del margen cervical para evitar labios o filos
                let w_mu = if int_grid.is_some() {
                    smoothstep(0.55, 1.35, base_d[i])
                } else {
                    1.0
                };
                if guard[i] > 1e-4 && w_mu > 1e-4 && !adj[i].is_empty() {
                    let mut sum_ext = Vec3::ZERO;
                    let mut cnt = 0usize;
                    for &j in &adj[i] {
                        if !has_ext || out.is_external[j] {
                            sum_ext += p2[j];
                            cnt += 1;
                        }
                    }
                    if cnt >= 3 {
                        let avg = sum_ext / cnt as f32;
                        out.positions[i] = p2[i] + (avg - p2[i]) * (mu * guard[i] * w_mu);
                    }
                }
            }
        }

        if let Some(ig) = &int_grid {
            let has_th = out.thickness_mm.len() == n;
            let th_ceil = min_thickness_mm * 0.92;
            for i in 0..n {
                if !has_ext || out.is_external[i] {
                    let floor_th = base_d[i].min(th_ceil);
                    let probe = (floor_th + 0.05).max(0.10);
                    if let Some((di, qi)) = ig.nearest_within(out.positions[i], probe) {
                        if di < floor_th {
                            let push = (out.positions[i] - qi).normalize_or_zero();
                            if push.length_squared() > 1e-6 {
                                out.positions[i] = qi + push * floor_th;
                                if has_th {
                                    out.thickness_mm[i] = floor_th;
                                }
                            } else if has_th {
                                out.thickness_mm[i] = di;
                            }
                        } else if has_th {
                            out.thickness_mm[i] = di;
                        }
                    } else if has_th {
                        out.thickness_mm[i] = ig.nearest(out.positions[i]).0;
                    }
                }
            }
        }
        out.compute_normals();
        out
    }

    /// Optimiza el perfil de emergencia gingival (0.25 a 2.60 mm sobre el margen cervical).
    pub fn smart_emergence_profile(
        crown: &TriMeshData,
        margin_curve: &[Vec3],
        insertion_axis: Vec3,
        convexity_mm: f32,
        min_thickness_mm: f32,
    ) -> TriMeshData {
        if crown.is_empty() || margin_curve.len() < 4 {
            return crown.clone();
        }
        let mut out = crown.clone();
        let n = out.positions.len();
        let has_ext = out.is_external.len() == n;
        let intaglio = collect_intaglio_pts(&out);
        let int_grid = (!intaglio.is_empty()).then(|| PointSpatialGrid::from_points(&intaglio, 0.65));

        let (u, v, a) = build_orthonormal_basis(insertion_axis);
        let origin = margin_curve.iter().copied().sum::<Vec3>() / margin_curve.len() as f32;

        let mut local: Vec<Vec3> = out
            .positions
            .iter()
            .map(|&p| {
                let r = p - origin;
                Vec3::new(r.dot(u), r.dot(v), r.dot(a))
            })
            .collect();

        let mut z_min = f32::INFINITY;
        for i in 0..n {
            if !has_ext || out.is_external[i] {
                z_min = z_min.min(local[i].z);
            }
        }

        let mut cerv_indices = Vec::new();
        for i in 0..n {
            if has_ext && !out.is_external[i] {
                continue;
            }
            let h = (local[i].z - z_min).max(0.0);
            let band_w = smoothstep(0.12, 0.65, h) * (1.0 - smoothstep(1.35, 2.85, h));
            if band_w > 0.05 {
                let r_unit = Vec2::new(local[i].x, local[i].y).normalize_or_zero();
                local[i].x += r_unit.x * convexity_mm * band_w;
                local[i].y += r_unit.y * convexity_mm * band_w;
                cerv_indices.push(i);
            }
            out.positions[i] = origin + u * local[i].x + v * local[i].y + a * local[i].z;
        }

        if !cerv_indices.is_empty() {
            let adj = out.build_adjacency();
            let prev = out.positions.clone();
            for &ci in &cerv_indices {
                if !adj[ci].is_empty() {
                    let avg: Vec3 = adj[ci].iter().map(|&j| prev[j]).sum::<Vec3>() / adj[ci].len() as f32;
                    out.positions[ci] = prev[ci] * 0.72 + avg * 0.28;
                }
            }
        }

        if let Some(ig) = &int_grid {
            let has_th = out.thickness_mm.len() == n;
            let probe = (min_thickness_mm * 0.92 + 0.05).max(0.65);
            for i in 0..n {
                if !has_ext || out.is_external[i] {
                    if let Some((di, qi)) = ig.nearest_within(out.positions[i], probe) {
                        let req = min_thickness_mm * smoothstep(0.10, 0.58, di) * 0.92;
                        if di < req {
                            let push = (out.positions[i] - qi).normalize_or_zero();
                            if push.length_squared() > 1e-6 {
                                out.positions[i] = qi + push * req;
                            }
                        }
                    }
                    if has_th {
                        out.thickness_mm[i] = ig.nearest(out.positions[i]).0;
                    }
                }
            }
        }
        out.compute_normals();
        out
    }

    /// Adapta la superficie basal del póntico hacia el reborde gingival del modelo maestro:
    /// - **Reducción gingival**: recorta excesos si el póntico penetra la encía.
    /// - **Adaptación gingival**: desciende suavemente la base si el póntico queda flotando por encima.
    /// Con `pontic_zones = Some(...)` actúa exclusivamente sobre zonas de póntico sin tocar pilares.
    pub fn smart_adapt_pontic_to_gingiva(
        mesh: &TriMeshData,
        gingiva_mesh: &TriMeshData,
        insertion_axis: Vec3,
        gingival_offset_mm: f32,
        pontic_zones: Option<&[(Vec3, f32)]>,
    ) -> TriMeshData {
        if mesh.is_empty() || gingiva_mesh.is_empty() {
            return mesh.clone();
        }
        let mut out = mesh.clone();
        out.compute_normals();
        let n = out.positions.len();
        let has_ext = out.is_external.len() == n;
        let (u, v, a) = build_orthonormal_basis(insertion_axis);
        let origin = out.center();

        let ging_local_pos: Vec<Vec3> = gingiva_mesh
            .positions
            .iter()
            .map(|&p| {
                let r = p - origin;
                Vec3::new(r.dot(u), r.dot(v), r.dot(a))
            })
            .collect();
        let ging_local = TriMeshData::new(ging_local_pos, gingiva_mesh.indices.clone());
        let Some(ging_col) = MeshCollider::from_mesh(&ging_local) else {
            return out;
        };

        let mut local: Vec<Vec3> = out
            .positions
            .iter()
            .map(|&p| {
                let r = p - origin;
                Vec3::new(r.dot(u), r.dot(v), r.dot(a))
            })
            .collect();
        let local_n: Vec<Vec3> = out
            .normals
            .iter()
            .map(|&nm| Vec3::new(nm.dot(u), nm.dot(v), nm.dot(a)).normalize_or_zero())
            .collect();

        let local_zones: Option<Vec<(Vec2, f32)>> = pontic_zones.map(|zs| {
            zs.iter()
                .map(|&(zc, zr)| {
                    let r = zc - origin;
                    (Vec2::new(r.dot(u), r.dot(v)), zr.max(1.5))
                })
                .collect()
        });

        let mut zone_w = vec![0.0_f32; n];
        let mut z_min = f32::INFINITY;
        let mut z_max = f32::NEG_INFINITY;
        for i in 0..n {
            if has_ext && !out.is_external[i] {
                continue;
            }
            let w = if let Some(zs) = &local_zones {
                let p2 = Vec2::new(local[i].x, local[i].y);
                let mut best = 0.0_f32;
                for &(zc, zr) in zs {
                    let d = p2.distance(zc);
                    if d <= zr {
                        best = best.max(smoothstep(zr, zr * 0.68, d));
                    }
                }
                best
            } else {
                1.0
            };
            zone_w[i] = w;
            if w > 0.05 {
                z_min = z_min.min(local[i].z);
                z_max = z_max.max(local[i].z);
            }
        }
        if !z_min.is_finite() || !z_max.is_finite() {
            return out;
        }
        let z_span = (z_max - z_min).max(2.0);
        let offset = gingival_offset_mm.clamp(-0.60, 0.60);

        let mut touched = vec![false; n];
        let mut target_z_per_v: Vec<Option<f32>> = vec![None; n];

        for i in 0..n {
            let w = zone_w[i];
            if w <= 0.01 {
                continue;
            }
            let p = local[i];
            let z_norm = ((p.z - z_min) / z_span).clamp(0.0, 1.0);
            if z_norm > 0.72 {
                continue;
            }
            let ray_hit = ging_col
                .cast_ray(Vec3::new(p.x, p.y, z_max + 4.0), -Vec3::Z, z_span + 16.0)
                .or_else(|| {
                    for (ox, oy) in [
                        (0.008_f32, 0.005_f32),
                        (-0.008_f32, 0.005_f32),
                        (0.005_f32, -0.008_f32),
                        (-0.005_f32, -0.008_f32),
                    ] {
                        if let Some(h) = ging_col.cast_ray(
                            Vec3::new(p.x + ox, p.y + oy, z_max + 4.0),
                            -Vec3::Z,
                            z_span + 16.0,
                        ) {
                            return Some(h);
                        }
                    }
                    None
                });
            if let Some((_, hit)) = ray_hit {
                let z_ging = hit.z;
                if z_ging > z_max - 1.2 || (z_ging - p.z).abs() > 6.5 {
                    continue;
                }
                let z_target = (z_ging + offset).min(z_max - 1.6);
                target_z_per_v[i] = Some(z_target);

                if p.z < z_target {
                    // Reducción gingival: retirar intersección del póntico con la encía
                    local[i].z = p.z + (z_target - p.z) * w;
                    touched[i] = true;
                } else if p.z > z_target
                    && local_n[i].z < -0.02
                    && z_norm < 0.48
                    && (p.z - z_target) <= 5.0
                {
                    // Adaptación gingival: asentar base flotante suavemente hasta el reborde
                    let under_w = smoothstep(-0.02, -0.25, local_n[i].z)
                        * smoothstep(0.48, 0.18, z_norm)
                        * w;
                    if under_w > 1e-3 {
                        local[i].z = p.z - (p.z - z_target) * under_w;
                        touched[i] = true;
                    }
                }
            }
        }

        if touched.iter().any(|&t| t) {
            let adj = out.build_adjacency();
            let mut smooth_mask = touched.clone();
            for i in 0..n {
                if touched[i] {
                    for &nb in &adj[i] {
                        if zone_w[nb] > 0.05 {
                            smooth_mask[nb] = true;
                        }
                    }
                }
            }
            for _ in 0..3 {
                let prev = local.clone();
                for i in 0..n {
                    if !smooth_mask[i] || adj[i].is_empty() {
                        continue;
                    }
                    let z_norm = ((prev[i].z - z_min) / z_span).clamp(0.0, 1.0);
                    if z_norm > 0.72 {
                        continue;
                    }
                    let avg_z: f32 =
                        adj[i].iter().map(|&j| prev[j].z).sum::<f32>() / adj[i].len() as f32;
                    let mut next_z = prev[i].z * 0.55 + avg_z * 0.45;
                    if let Some(zt) = target_z_per_v[i] {
                        if next_z < zt - 0.02 {
                            next_z = zt;
                        }
                    }
                    local[i].z = next_z;
                }
            }
        }

        for i in 0..n {
            out.positions[i] = origin + u * local[i].x + v * local[i].y + a * local[i].z;
        }
        out.compute_normals();
        out
    }
}
