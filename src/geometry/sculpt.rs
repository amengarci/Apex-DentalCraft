//! Módulo 4 (Esculpido Libre y Morphing 3D sin Conflictos de Triángulos — Estilo 3Shape)
//!
//! - `apply_brush_stroke`: Pinceles `Add` (Aumentar), `Remove` (Quitar) y `Smooth` (Suavizar)
//!   usando normal coherente de plano de cera + 2 pasadas de relajación tangencial 1-anillo
//!   y suavizado Taubin topológico (cero pliegues ni inversiones de triángulos).
//! - `MorphSession` / `apply_morph_drag`: Herramienta interactiva **Morphing 3D** para agarrar
//!   y deformar cúspides, rebordes y contornos con kernel $C^2$ Wendland, bloqueo de margen
//!   cervical y protección de grosor mínimo.
//! - `transform_adapted_crown_body`: Permite aplicar **Transformación Individual** sobre una
//!   corona ya adaptada manteniendo sellado el borde cervical.

use crate::geometry::mesh::{build_orthonormal_basis, smoothstep, wendland_c2, TriMeshData};
use glam::{Mat3, Quat, Vec2, Vec3};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SculptMode {
    Add,
    Remove,
    Smooth,
}

/// Rejilla espacial densa (CSR) para consultas exactas O(1) de punto más cercano en lugar de O(N·M).
#[derive(Debug, Clone)]
pub struct PointSpatialGrid {
    pub cell_size: f32,
    pub inv_cell: f32,
    pub min_c: [i32; 3],
    pub max_c: [i32; 3],
    pub dims: [usize; 3],
    pub cells: Vec<(u32, u32)>,
    pub points: Vec<Vec3>,
}

impl PointSpatialGrid {
    pub fn from_points(pts: &[Vec3], target_cell_size: f32) -> Self {
        if pts.is_empty() {
            return Self {
                cell_size: target_cell_size.max(0.25),
                inv_cell: 1.0 / target_cell_size.max(0.25),
                min_c: [0; 3],
                max_c: [-1; 3],
                dims: [0, 0, 0],
                cells: Vec::new(),
                points: Vec::new(),
            };
        }
        let mut mn = Vec3::splat(f32::INFINITY);
        let mut mx = Vec3::splat(f32::NEG_INFINITY);
        for &p in pts {
            mn = mn.min(p);
            mx = mx.max(p);
        }
        let mut cs = target_cell_size.max(0.25);
        let span = mx - mn;
        while ((span.x / cs + 3.0) * (span.y / cs + 3.0) * (span.z / cs + 3.0)) > 220_000.0 {
            cs *= 1.25;
        }
        let inv = 1.0 / cs;
        let min_c = [
            (mn.x * inv).floor() as i32,
            (mn.y * inv).floor() as i32,
            (mn.z * inv).floor() as i32,
        ];
        let max_c = [
            (mx.x * inv).floor() as i32,
            (mx.y * inv).floor() as i32,
            (mx.z * inv).floor() as i32,
        ];
        let dx = (max_c[0] - min_c[0] + 1).max(1) as usize;
        let dy = (max_c[1] - min_c[1] + 1).max(1) as usize;
        let dz = (max_c[2] - min_c[2] + 1).max(1) as usize;
        let n_cells = dx * dy * dz;

        let mut counts = vec![0u32; n_cells];
        for &p in pts {
            let ix = ((p.x * inv).floor() as i32 - min_c[0]).clamp(0, dx as i32 - 1) as usize;
            let iy = ((p.y * inv).floor() as i32 - min_c[1]).clamp(0, dy as i32 - 1) as usize;
            let iz = ((p.z * inv).floor() as i32 - min_c[2]).clamp(0, dz as i32 - 1) as usize;
            counts[(ix * dy + iy) * dz + iz] += 1;
        }

        let mut cells = Vec::with_capacity(n_cells);
        let mut cursor = vec![0u32; n_cells];
        let mut offset = 0u32;
        for ci in 0..n_cells {
            let c = counts[ci];
            cells.push((offset, offset + c));
            cursor[ci] = offset;
            offset += c;
        }

        let mut sorted_pts = vec![Vec3::ZERO; pts.len()];
        for &p in pts {
            let ix = ((p.x * inv).floor() as i32 - min_c[0]).clamp(0, dx as i32 - 1) as usize;
            let iy = ((p.y * inv).floor() as i32 - min_c[1]).clamp(0, dy as i32 - 1) as usize;
            let iz = ((p.z * inv).floor() as i32 - min_c[2]).clamp(0, dz as i32 - 1) as usize;
            let ci = (ix * dy + iy) * dz + iz;
            let dst = cursor[ci] as usize;
            sorted_pts[dst] = p;
            cursor[ci] += 1;
        }

        Self {
            cell_size: cs,
            inv_cell: inv,
            min_c,
            max_c,
            dims: [dx, dy, dz],
            cells,
            points: sorted_pts,
        }
    }

    #[inline]
    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }

    #[inline]
    fn cell_coords(&self, q: Vec3) -> [i32; 3] {
        [
            (q.x * self.inv_cell).floor() as i32,
            (q.y * self.inv_cell).floor() as i32,
            (q.z * self.inv_cell).floor() as i32,
        ]
    }

    /// Consulta exacta dentro de un radio máximo `max_radius`; devuelve `None` al instante si no hay puntos cercanos.
    #[inline]
    pub fn nearest_within(&self, q: Vec3, max_radius: f32) -> Option<(f32, Vec3)> {
        if self.points.is_empty() || max_radius <= 0.0 {
            return None;
        }
        let [cx, cy, cz] = self.cell_coords(q);
        let rc = (max_radius * self.inv_cell).ceil() as i32;
        let ix0 = (cx - rc).max(self.min_c[0]);
        let ix1 = (cx + rc).min(self.max_c[0]);
        if ix0 > ix1 {
            return None;
        }
        let iy0 = (cy - rc).max(self.min_c[1]);
        let iy1 = (cy + rc).min(self.max_c[1]);
        if iy0 > iy1 {
            return None;
        }
        let iz0 = (cz - rc).max(self.min_c[2]);
        let iz1 = (cz + rc).min(self.max_c[2]);
        if iz0 > iz1 {
            return None;
        }

        let dy = self.dims[1];
        let dz = self.dims[2];
        let r_max2 = max_radius * max_radius;
        let mut best_d2 = r_max2;
        let mut best_p = Vec3::ZERO;
        let mut found = false;

        for ix in ix0..=ix1 {
            let ux = (ix - self.min_c[0]) as usize;
            for iy in iy0..=iy1 {
                let uy = (iy - self.min_c[1]) as usize;
                let base = (ux * dy + uy) * dz;
                for iz in iz0..=iz1 {
                    let uz = (iz - self.min_c[2]) as usize;
                    let (start, end) = self.cells[base + uz];
                    for &p in &self.points[start as usize..end as usize] {
                        let d2 = p.distance_squared(q);
                        if d2 < best_d2 {
                            best_d2 = d2;
                            best_p = p;
                            found = true;
                        }
                    }
                }
            }
        }
        if found {
            Some((best_d2.sqrt(), best_p))
        } else {
            None
        }
    }

    /// Consulta exacta global (anillos de Chebyshev expansivos + respaldo lineal exacto si queda fuera de radio local).
    #[inline]
    pub fn nearest(&self, q: Vec3) -> (f32, Vec3) {
        if self.points.is_empty() {
            return (f32::INFINITY, Vec3::ZERO);
        }
        if self.points.len() <= 48 {
            return nearest_intaglio_dist_and_pt(&self.points, q);
        }
        let [cx, cy, cz] = self.cell_coords(q);
        let in_box = cx >= self.min_c[0]
            && cx <= self.max_c[0]
            && cy >= self.min_c[1]
            && cy <= self.max_c[1]
            && cz >= self.min_c[2]
            && cz <= self.max_c[2];

        if in_box {
            let dy = self.dims[1];
            let dz = self.dims[2];
            let mut best_d2 = f32::INFINITY;
            let mut best_p = Vec3::ZERO;

            for r in 0_i32..=5 {
                let ix0 = (cx - r).max(self.min_c[0]);
                let ix1 = (cx + r).min(self.max_c[0]);
                let iy0 = (cy - r).max(self.min_c[1]);
                let iy1 = (cy + r).min(self.max_c[1]);
                let iz0 = (cz - r).max(self.min_c[2]);
                let iz1 = (cz + r).min(self.max_c[2]);

                for ix in ix0..=ix1 {
                    let ux = (ix - self.min_c[0]) as usize;
                    let bx = (ix - cx).abs() == r;
                    for iy in iy0..=iy1 {
                        let uy = (iy - self.min_c[1]) as usize;
                        let by = (iy - cy).abs() == r;
                        let base = (ux * dy + uy) * dz;
                        for iz in iz0..=iz1 {
                            if r > 0 && !bx && !by && (iz - cz).abs() != r {
                                continue;
                            }
                            let uz = (iz - self.min_c[2]) as usize;
                            let (start, end) = self.cells[base + uz];
                            for &p in &self.points[start as usize..end as usize] {
                                let d2 = p.distance_squared(q);
                                if d2 < best_d2 {
                                    best_d2 = d2;
                                    best_p = p;
                                }
                            }
                        }
                    }
                }

                if r >= 1 && best_d2 <= ((r as f32) * self.cell_size).powi(2) {
                    return (best_d2.sqrt(), best_p);
                }
            }
        }

        nearest_intaglio_dist_and_pt(&self.points, q)
    }
}

#[derive(Debug, Clone)]
pub struct SculptStrokeCache {
    pub adjacency: Vec<Vec<usize>>,
    pub intaglio_grid: Option<PointSpatialGrid>,
    pub margin_seam_grid: Option<PointSpatialGrid>,
    pub open_boundary: Vec<bool>,
    pub has_intaglio: bool,
}

#[derive(Debug, Clone)]
pub struct MorphSession {
    pub base_mesh: TriMeshData,
    pub grab_point: Vec3,
    pub grab_normal: Vec3,
    pub active_indices: Vec<usize>,
    pub weights: Vec<f32>,
    pub base_d_intaglio: Vec<f32>,
    pub active_adjacency: Vec<Vec<usize>>,
    pub intaglio_grid: Option<PointSpatialGrid>,
}

#[derive(Debug, Clone)]
pub struct TransformSession {
    pub base_mesh: TriMeshData,
    pub is_adapted: bool,
    pub u: Vec3,
    pub v: Vec3,
    pub w: Vec3,
    pub origin: Vec3,
    pub mins: Vec3,
    pub maxs: Vec3,
    pub active_indices: Vec<usize>,
    pub cam_local: Vec<Vec3>,
    pub base_lock: Vec<f32>,
    pub intaglio_grid: Option<PointSpatialGrid>,
}

#[inline]
pub fn collect_intaglio_pts(mesh: &TriMeshData) -> Vec<Vec3> {
    if mesh.is_external.len() != mesh.positions.len() {
        return Vec::new();
    }
    mesh.positions
        .iter()
        .enumerate()
        .filter_map(|(i, &p)| if !mesh.is_external[i] { Some(p) } else { None })
        .collect()
}

#[inline]
pub fn nearest_intaglio_dist_and_pt(intaglio: &[Vec3], q: Vec3) -> (f32, Vec3) {
    let mut best_d2 = f32::INFINITY;
    let mut best_p = Vec3::ZERO;
    for &p in intaglio {
        let d2 = p.distance_squared(q);
        if d2 < best_d2 {
            best_d2 = d2;
            best_p = p;
        }
    }
    (best_d2.sqrt(), best_p)
}

pub struct CrownSculptor;

impl CrownSculptor {
    /// Precalcula una sola vez al iniciar un trazo la adyacencia y rejillas espaciales de intaglio/margen.
    pub fn prepare_sculpt_cache(mesh: &TriMeshData) -> SculptStrokeCache {
        let n = mesh.positions.len();
        let adjacency = mesh.build_adjacency();
        let has_ext_mask = mesh.is_external.len() == n;
        let intaglio = collect_intaglio_pts(mesh);
        let has_intaglio = !intaglio.is_empty();

        let mut margin_seam_pts: Vec<Vec3> = Vec::new();
        let mut open_boundary = vec![false; n];
        if has_intaglio && has_ext_mask {
            for i in 0..n {
                if !mesh.is_external[i] && adjacency[i].iter().any(|&nb| mesh.is_external[nb]) {
                    margin_seam_pts.push(mesh.positions[i]);
                }
            }
        } else {
            use std::collections::HashMap;
            let mut edge_cnt: HashMap<(u32, u32), u32> = HashMap::with_capacity(mesh.indices.len() * 2);
            for &[a, b, c] in &mesh.indices {
                for (u, v) in [(a, b), (b, c), (c, a)] {
                    let key = if u < v { (u, v) } else { (v, u) };
                    *edge_cnt.entry(key).or_insert(0) += 1;
                }
            }
            for ((u, v), cnt) in edge_cnt {
                if cnt == 1 {
                    if (u as usize) < n { open_boundary[u as usize] = true; }
                    if (v as usize) < n { open_boundary[v as usize] = true; }
                }
            }
        }

        let intaglio_grid = if has_intaglio {
            Some(PointSpatialGrid::from_points(&intaglio, 0.65))
        } else {
            None
        };
        let margin_seam_grid = if !margin_seam_pts.is_empty() {
            Some(PointSpatialGrid::from_points(&margin_seam_pts, 0.65))
        } else {
            None
        };

        SculptStrokeCache {
            adjacency,
            intaglio_grid,
            margin_seam_grid,
            open_boundary,
            has_intaglio,
        }
    }

    /// Aplica un trazo de pincel 3D usando la caché espacial precalculada de `prepare_sculpt_cache`.
    pub fn apply_brush_stroke_cached(
        mesh: &mut TriMeshData,
        center_3d: Vec3,
        mode: SculptMode,
        radius_mm: f32,
        strength_mm: f32,
        min_thickness_mm: f32,
        cache: &SculptStrokeCache,
    ) {
        if mesh.is_empty() {
            return;
        }
        let n = mesh.positions.len();
        let rad = radius_mm.max(0.20);
        let rad2 = rad * rad;
        let has_ext_mask = mesh.is_external.len() == n;
        let has_intaglio = cache.has_intaglio && cache.intaglio_grid.is_some();

        let mut brush_indices = Vec::new();
        let mut brush_dists = Vec::new();
        for i in 0..n {
            if has_ext_mask && !mesh.is_external[i] {
                continue;
            }
            let d2 = mesh.positions[i].distance_squared(center_3d);
            if d2 < rad2 {
                brush_indices.push(i);
                brush_dists.push(d2.sqrt());
            }
        }
        if brush_indices.is_empty() {
            return;
        }

        let adj = &cache.adjacency;
        let open_boundary = &cache.open_boundary;

        // Distancias al intaglio y al borde marginal + kernel Wendland C2
        let m_count = brush_indices.len();
        let mut d_intaglio = vec![2.5_f32; m_count];
        let mut d_margin_seam = vec![5.0_f32; m_count];
        let mut near_open_rim = vec![false; m_count];
        let mut falloff = vec![0.0_f32; m_count];
        let has_margin_seam = cache.margin_seam_grid.is_some();
        let int_probe_r = (min_thickness_mm * 1.35 + strength_mm).max(2.0);

        for (k, &vi) in brush_indices.iter().enumerate() {
            let p = mesh.positions[vi];
            if let Some(mg) = &cache.margin_seam_grid {
                if let Some((dm, _)) = mg.nearest_within(p, 1.65) {
                    d_margin_seam[k] = dm;
                }
            }
            if !has_intaglio {
                near_open_rim[k] = open_boundary[vi] || adj[vi].iter().any(|&nb| open_boundary[nb]);
            }
            let guard = if let Some(ig) = &cache.intaglio_grid {
                let di = ig
                    .nearest_within(p, int_probe_r)
                    .map(|(d, _)| d)
                    .unwrap_or(int_probe_r);
                d_intaglio[k] = di;
                if mode == SculptMode::Smooth {
                    let g_seam = if has_margin_seam {
                        smoothstep(0.08, 0.55, d_margin_seam[k])
                    } else {
                        smoothstep(0.14, 0.55, di)
                    };
                    g_seam * smoothstep(0.06, 0.28, di)
                } else {
                    smoothstep(0.20, 0.62, di)
                }
            } else if open_boundary[vi] {
                0.0
            } else if near_open_rim[k] {
                0.35
            } else {
                1.0
            };
            falloff[k] = wendland_c2(brush_dists[k] / rad) * guard;
        }

        // Suavizar normales en 1-anillo para eliminar ruido de faceta individual
        let mut smooth_normals = vec![Vec3::Z; m_count];
        for (k, &vi) in brush_indices.iter().enumerate() {
            let mut acc = mesh.normals[vi];
            for &nb in &adj[vi] {
                acc += mesh.normals[nb];
            }
            smooth_normals[k] = acc.normalize_or_zero();
            if smooth_normals[k].length_squared() < 1e-6 {
                smooth_normals[k] = Vec3::Z;
            }
        }

        match mode {
            SculptMode::Add | SculptMode::Remove => {
                // Normal coherente de plano de cera (evita cruce de paredes en surcos profundos)
                let mut w_sum = 0.0_f32;
                let mut avg_n = Vec3::ZERO;
                let mut plane_center = Vec3::ZERO;
                for (k, &vi) in brush_indices.iter().enumerate() {
                    let w = falloff[k];
                    w_sum += w;
                    avg_n += smooth_normals[k] * w;
                    plane_center += mesh.positions[vi] * w;
                }
                if w_sum > 1e-6 {
                    avg_n = avg_n.normalize_or_zero();
                    plane_center /= w_sum;
                } else {
                    avg_n = smooth_normals[0];
                    plane_center = center_3d;
                }

                let safe_floor = min_thickness_mm * 0.94;
                for (k, &vi) in brush_indices.iter().enumerate() {
                    let dir = (avg_n * 0.78 + smooth_normals[k] * 0.22).normalize_or_zero();
                    let signed_h = (mesh.positions[vi] - plane_center).dot(avg_n);
                    let denom_h = (rad * 0.35).max(0.15);

                    if mode == SculptMode::Add {
                        let boost = (1.0 - 0.35 * (signed_h / denom_h)).clamp(0.55, 1.35);
                        let disp = strength_mm * falloff[k] * boost;
                        mesh.positions[vi] += dir * disp;
                    } else {
                        let boost = (1.0 + 0.30 * (signed_h / denom_h)).clamp(0.55, 1.35);
                        let disp = strength_mm * falloff[k] * boost;
                        let cand = mesh.positions[vi] - dir * disp;
                        if let Some(ig) = &cache.intaglio_grid {
                            let new_d = ig
                                .nearest_within(cand, safe_floor + 0.04)
                                .map(|(d, _)| d)
                                .unwrap_or(safe_floor + 0.04);
                            let factor = if new_d >= safe_floor {
                                1.0
                            } else {
                                ((d_intaglio[k] - safe_floor) / disp.max(1e-4)).clamp(0.0, 1.0)
                            };
                            mesh.positions[vi] -= dir * (disp * factor);
                        } else {
                            mesh.positions[vi] = cand;
                        }
                    }
                }

                // 2 pasadas de RELAJACIÓN TANGENCIAL TOPOLÓGICA (cero conflictos de triángulos)
                let mut next_pts = vec![Vec3::ZERO; m_count];
                for _ in 0..2 {
                    for (k, &vi) in brush_indices.iter().enumerate() {
                        if adj[vi].is_empty() {
                            next_pts[k] = mesh.positions[vi];
                            continue;
                        }
                        let avg_1ring: Vec3 = adj[vi].iter().map(|&nb| mesh.positions[nb]).sum::<Vec3>()
                            / adj[vi].len() as f32;
                        let lap = avg_1ring - mesh.positions[vi];
                        let n_loc = smooth_normals[k];
                        let tangent_lap = lap - n_loc * lap.dot(n_loc);
                        let rw = (0.48 * falloff[k]).clamp(0.0, 0.48);
                        next_pts[k] = mesh.positions[vi] + tangent_lap * rw;
                    }
                    for (k, &vi) in brush_indices.iter().enumerate() {
                        mesh.positions[vi] = next_pts[k];
                    }
                }
            }
            SculptMode::Smooth => {
                let t_str = ((strength_mm - 0.01) / 0.44).clamp(0.06, 1.0);
                let n_cycles = (2 + (t_str * 8.0).round() as usize).clamp(2, 10);
                let lambda = (0.34 + 0.30 * t_str).clamp(0.34, 0.64);
                let mu_base = -(lambda * 0.34);

                let init_d_intaglio = d_intaglio.clone();
                let mut next_pts = vec![Vec3::ZERO; m_count];
                let mut was_clamped = vec![false; m_count];
                let taubin_probe = (min_thickness_mm * 1.30).max(1.5);

                for _cycle in 0..n_cycles {
                    // --- Pasada A: Suavizado Convexo Positivo (`lambda > 0`) ---
                    for (k, &vi) in brush_indices.iter().enumerate() {
                        was_clamped[k] = false;
                        if adj[vi].is_empty() || falloff[k] <= 1e-5 {
                            next_pts[k] = mesh.positions[vi];
                            continue;
                        }
                        let gain = (lambda * falloff[k]).clamp(0.0, 0.62);
                        let avg_1ring: Vec3 =
                            adj[vi].iter().map(|&nb| mesh.positions[nb]).sum::<Vec3>()
                                / adj[vi].len() as f32;
                        let mut cand = mesh.positions[vi] + (avg_1ring - mesh.positions[vi]) * gain;

                        if let Some(ig) = &cache.intaglio_grid {
                            let collar_taper = if has_margin_seam {
                                smoothstep(0.05, 0.90, d_margin_seam[k])
                            } else {
                                1.0
                            };
                            let req_floor = (min_thickness_mm * 0.92 * collar_taper)
                                .min(init_d_intaglio[k]);
                            let mut cur_d = ig
                                .nearest_within(cand, taubin_probe)
                                .map(|x| x.0)
                                .unwrap_or(taubin_probe);
                            if cur_d < req_floor && req_floor > 1e-4 {
                                if let Some((d_chk, q_in)) = ig.nearest_within(cand, req_floor + 0.05) {
                                    if d_chk < req_floor {
                                        let push_dir = (cand - q_in).normalize_or_zero();
                                        if push_dir.length_squared() > 1e-6 {
                                            cand = q_in + push_dir * req_floor;
                                            cur_d = req_floor;
                                        } else {
                                            cand = mesh.positions[vi];
                                            cur_d = init_d_intaglio[k];
                                        }
                                        was_clamped[k] = true;
                                    }
                                }
                            }
                            d_intaglio[k] = cur_d;
                        }
                        next_pts[k] = cand;
                    }
                    for (k, &vi) in brush_indices.iter().enumerate() {
                        mesh.positions[vi] = next_pts[k];
                    }

                    // --- Pasada B: Compensación Taubin (`mu < 0`) SOLO lejos del margen y sin trinquete ---
                    for (k, &vi) in brush_indices.iter().enumerate() {
                        if adj[vi].is_empty() || falloff[k] <= 1e-5 || was_clamped[k] || near_open_rim[k] {
                            next_pts[k] = mesh.positions[vi];
                            continue;
                        }
                        let w_taubin = if has_intaglio && has_margin_seam {
                            smoothstep(0.75, 1.55, d_margin_seam[k])
                                * smoothstep(min_thickness_mm * 0.95, min_thickness_mm * 1.25, d_intaglio[k])
                        } else if has_intaglio {
                            smoothstep(min_thickness_mm * 0.95, min_thickness_mm * 1.25, d_intaglio[k])
                        } else {
                            1.0
                        };
                        if w_taubin <= 1e-4 {
                            next_pts[k] = mesh.positions[vi];
                            continue;
                        }
                        let mut sum_ext = Vec3::ZERO;
                        let mut cnt_ext = 0usize;
                        for &nb in &adj[vi] {
                            if !has_ext_mask || mesh.is_external[nb] {
                                sum_ext += mesh.positions[nb];
                                cnt_ext += 1;
                            }
                        }
                        if cnt_ext < 3 {
                            next_pts[k] = mesh.positions[vi];
                            continue;
                        }
                        let avg_ext = sum_ext / cnt_ext as f32;
                        let gain = (mu_base * falloff[k] * w_taubin).clamp(-0.24, 0.0);
                        let cand = mesh.positions[vi] + (avg_ext - mesh.positions[vi]) * gain;
                        if let Some(ig) = &cache.intaglio_grid {
                            d_intaglio[k] = ig
                                .nearest_within(cand, taubin_probe)
                                .map(|x| x.0)
                                .unwrap_or(taubin_probe);
                        }
                        next_pts[k] = cand;
                    }
                    for (k, &vi) in brush_indices.iter().enumerate() {
                        mesh.positions[vi] = next_pts[k];
                    }
                }
            }
        }

        if let Some(ig) = &cache.intaglio_grid {
            if mesh.thickness_mm.len() == n {
                for &vi in &brush_indices {
                    mesh.thickness_mm[vi] = ig.nearest(mesh.positions[vi]).0;
                }
            }
        }
        mesh.compute_normals();
    }

    /// Aplica un trazo de pincel 3D libre de conflictos triangulares sobre la corona.
    pub fn apply_brush_stroke(
        mesh: &mut TriMeshData,
        center_3d: Vec3,
        mode: SculptMode,
        radius_mm: f32,
        strength_mm: f32,
        min_thickness_mm: f32,
        cached_adjacency: Option<&[Vec<usize>]>,
    ) {
        if mesh.is_empty() {
            return;
        }
        let n = mesh.positions.len();
        let has_ext_mask = mesh.is_external.len() == n;
        let intaglio = collect_intaglio_pts(mesh);
        let has_intaglio = !intaglio.is_empty();
        let adjacency = match cached_adjacency {
            Some(a) if a.len() == n => a.to_vec(),
            _ => mesh.build_adjacency(),
        };

        let mut margin_seam_pts: Vec<Vec3> = Vec::new();
        let mut open_boundary = vec![false; n];
        if has_intaglio && has_ext_mask {
            for i in 0..n {
                if !mesh.is_external[i] && adjacency[i].iter().any(|&nb| mesh.is_external[nb]) {
                    margin_seam_pts.push(mesh.positions[i]);
                }
            }
        } else {
            use std::collections::HashMap;
            let mut edge_cnt: HashMap<(u32, u32), u32> = HashMap::with_capacity(mesh.indices.len() * 2);
            for &[a, b, c] in &mesh.indices {
                for (u, v) in [(a, b), (b, c), (c, a)] {
                    let key = if u < v { (u, v) } else { (v, u) };
                    *edge_cnt.entry(key).or_insert(0) += 1;
                }
            }
            for ((u, v), cnt) in edge_cnt {
                if cnt == 1 {
                    if (u as usize) < n { open_boundary[u as usize] = true; }
                    if (v as usize) < n { open_boundary[v as usize] = true; }
                }
            }
        }

        let cache = SculptStrokeCache {
            adjacency,
            intaglio_grid: if has_intaglio {
                Some(PointSpatialGrid::from_points(&intaglio, 0.65))
            } else {
                None
            },
            margin_seam_grid: if !margin_seam_pts.is_empty() {
                Some(PointSpatialGrid::from_points(&margin_seam_pts, 0.65))
            } else {
                None
            },
            open_boundary,
            has_intaglio,
        };
        Self::apply_brush_stroke_cached(
            mesh,
            center_3d,
            mode,
            radius_mm,
            strength_mm,
            min_thickness_mm,
            &cache,
        );
    }

    /// Prepara una sesión de **Morphing 3D** estilo 3Shape al presionar el ratón sobre la corona.
    pub fn prepare_morph_session(
        crown_mesh: &TriMeshData,
        grab_point_3d: Vec3,
        radius_mm: f32,
    ) -> MorphSession {
        let base_mesh = crown_mesh.clone();
        let n = base_mesh.positions.len();
        let has_ext = base_mesh.is_external.len() == n;
        let intaglio_pts = collect_intaglio_pts(&base_mesh);
        let intaglio_grid = if !intaglio_pts.is_empty() {
            Some(PointSpatialGrid::from_points(&intaglio_pts, 0.65))
        } else {
            None
        };
        let rad = radius_mm.max(0.35);
        let rad2 = rad * rad;

        // Normal en el vértice externo más cercano al punto agarrado
        let mut best_d2 = f32::INFINITY;
        let mut grab_normal = Vec3::Z;
        for i in 0..n {
            if has_ext && !base_mesh.is_external[i] {
                continue;
            }
            let d2 = base_mesh.positions[i].distance_squared(grab_point_3d);
            if d2 < best_d2 {
                best_d2 = d2;
                grab_normal = base_mesh.normals[i];
            }
        }
        grab_normal = grab_normal.normalize_or_zero();
        if grab_normal.length_squared() < 1e-6 {
            grab_normal = Vec3::Z;
        }

        let mut active_indices = Vec::new();
        let mut weights = Vec::new();
        let mut base_d_intaglio = Vec::new();
        let mut closest_k = 0usize;
        let mut closest_d = f32::INFINITY;

        for i in 0..n {
            if has_ext && !base_mesh.is_external[i] {
                continue;
            }
            let d2 = base_mesh.positions[i].distance_squared(grab_point_3d);
            if d2 < rad2 {
                let d = d2.sqrt();
                let w_rad = wendland_c2(d / rad);
                let (di, m_guard) = if let Some(ig) = &intaglio_grid {
                    let dist_i = ig
                        .nearest_within(base_mesh.positions[i], 2.5)
                        .map(|x| x.0)
                        .unwrap_or(2.5);
                    (dist_i, smoothstep(0.18, 0.65, dist_i))
                } else {
                    (2.5, 1.0)
                };
                let cos_sim = base_mesh.normals[i].dot(grab_normal);
                let dir_guard = (0.62 + 0.38 * cos_sim).clamp(0.18, 1.0);
                let w = w_rad * m_guard * dir_guard;

                if d < closest_d {
                    closest_d = d;
                    closest_k = active_indices.len();
                }
                active_indices.push(i);
                weights.push(w);
                base_d_intaglio.push(di);
            }
        }

        if !weights.is_empty() {
            let w_max = weights.iter().copied().fold(0.0_f32, f32::max);
            if w_max > 0.25 {
                let guard_at_grab = if intaglio_grid.is_some() {
                    smoothstep(0.18, 0.65, base_d_intaglio[closest_k])
                } else {
                    1.0
                };
                for w in &mut weights {
                    *w = (*w / w_max).clamp(0.0, 1.0) * guard_at_grab;
                }
            }
        }

        // Adyacencia local exclusiva de vértices activos en una sola pasada sobre triángulos
        let mut vert_to_k = vec![None::<usize>; n];
        for (k, &vi) in active_indices.iter().enumerate() {
            vert_to_k[vi] = Some(k);
        }
        let mut active_adjacency = vec![Vec::<usize>::new(); active_indices.len()];
        for &[a, b, c] in &base_mesh.indices {
            let (ua, ub, uc) = (a as usize, b as usize, c as usize);
            if let Some(ka) = vert_to_k.get(ua).copied().flatten() {
                if !active_adjacency[ka].contains(&ub) { active_adjacency[ka].push(ub); }
                if !active_adjacency[ka].contains(&uc) { active_adjacency[ka].push(uc); }
            }
            if let Some(kb) = vert_to_k.get(ub).copied().flatten() {
                if !active_adjacency[kb].contains(&ua) { active_adjacency[kb].push(ua); }
                if !active_adjacency[kb].contains(&uc) { active_adjacency[kb].push(uc); }
            }
            if let Some(kc) = vert_to_k.get(uc).copied().flatten() {
                if !active_adjacency[kc].contains(&ua) { active_adjacency[kc].push(ua); }
                if !active_adjacency[kc].contains(&ub) { active_adjacency[kc].push(ub); }
            }
        }

        MorphSession {
            base_mesh,
            grab_point: grab_point_3d,
            grab_normal,
            active_indices,
            weights,
            base_d_intaglio,
            active_adjacency,
            intaglio_grid,
        }
    }

    /// Evalúa en tiempo real el estiramiento / moldeado **Morphing 3D** desde la malla inicial del trazo.
    pub fn apply_morph_drag(
        session: &MorphSession,
        delta_world_3d: Vec3,
        min_thickness_mm: f32,
        constrain_to_normal: bool,
    ) -> TriMeshData {
        let mut out = session.base_mesh.clone();
        if session.active_indices.is_empty() {
            return out;
        }
        let delta = if constrain_to_normal {
            session.grab_normal * delta_world_3d.dot(session.grab_normal)
        } else {
            delta_world_3d
        };

        let floor_th = min_thickness_mm * 0.92;
        let has_th = out.thickness_mm.len() == out.positions.len();

        for (k, &vi) in session.active_indices.iter().enumerate() {
            let w = session.weights[k];
            let mut cand = session.base_mesh.positions[vi] + delta * w;
            if let Some(ig) = &session.intaglio_grid {
                let base_d = session.base_d_intaglio[k];
                let probe = (floor_th + 0.05).min(base_d + 0.05).max(0.20);
                if let Some((new_d, q_int)) = ig.nearest_within(cand, probe) {
                    if new_d < floor_th && new_d < base_d {
                        let push_dir = (cand - q_int).normalize_or_zero();
                        let push_dir = if push_dir.length_squared() > 1e-6 {
                            push_dir
                        } else {
                            session.base_mesh.normals[vi]
                        };
                        let target_d = base_d.min(floor_th).max(new_d);
                        cand = q_int + push_dir * target_d;
                    }
                }
            }
            out.positions[vi] = cand;
        }

        // 1 pasada rápida de relajación tangencial para mantener triángulos limpios al estirar
        let mut relaxed = Vec::with_capacity(session.active_indices.len());
        for (k, &vi) in session.active_indices.iter().enumerate() {
            let nbs = &session.active_adjacency[k];
            if nbs.is_empty() {
                relaxed.push(out.positions[vi]);
                continue;
            }
            let avg: Vec3 = nbs.iter().map(|&nb| out.positions[nb]).sum::<Vec3>() / nbs.len() as f32;
            let lap = avg - out.positions[vi];
            let n_loc = session.base_mesh.normals[vi];
            let tan_lap = lap - n_loc * lap.dot(n_loc);
            relaxed.push(out.positions[vi] + tan_lap * (0.28 * session.weights[k]));
        }
        for (k, &vi) in session.active_indices.iter().enumerate() {
            out.positions[vi] = relaxed[k];
            if has_th && session.weights[k] > 0.01 {
                if let Some(ig) = &session.intaglio_grid {
                    out.thickness_mm[vi] = ig.nearest(out.positions[vi]).0;
                }
            }
        }

        out.compute_normals();
        out
    }

    /// Transforma directamente un diente libre (aún no sellado al margen) en el sistema de colocación
    /// estilo 3Shape: traslación libre, rotación anatómica o trackball 3D, y escala unilateral/proporcional.
    /// Preserva al 100% cualquier edición previa de Morphing o Esculpido (cero salto ni reseteo).
    #[allow(dead_code)]
    pub fn transform_free_tooth_mesh(
        base_mesh: &TriMeshData,
        insertion_axis: Vec3,
        delta_t_local: Vec3,
        delta_r_deg: Vec3,
        free_rot_world: Option<Quat>,
        scale_factors: Vec3,
        anchor_dir_local: Option<Vec3>,
    ) -> TriMeshData {
        let mut out = base_mesh.clone();
        let n = out.positions.len();
        if n == 0 {
            return out;
        }
        let (u, v, a) = build_orthonormal_basis(insertion_axis);
        let origin = out.center();

        let mut local_pts = Vec::with_capacity(n);
        let mut mins = Vec3::splat(f32::INFINITY);
        let mut maxs = Vec3::splat(f32::NEG_INFINITY);
        for &p in &out.positions {
            let r = p - origin;
            let lp = Vec3::new(r.dot(u), r.dot(v), r.dot(a));
            mins = mins.min(lp);
            maxs = maxs.max(lp);
            local_pts.push(lp);
        }

        let mut pivot = 0.5 * (mins + maxs);
        if let Some(ad) = anchor_dir_local {
            if ad.x > 0.5 { pivot.x = mins.x; } else if ad.x < -0.5 { pivot.x = maxs.x; }
            if ad.y > 0.5 { pivot.y = mins.y; } else if ad.y < -0.5 { pivot.y = maxs.y; }
            if ad.z > 0.5 { pivot.z = mins.z; } else if ad.z < -0.5 { pivot.z = maxs.z; }
        }

        let basis_mat = Mat3::from_cols(u, v, a);
        let rot_local = if let Some(qw) = free_rot_world {
            basis_mat.transpose() * Mat3::from_quat(qw.normalize()) * basis_mat
        } else {
            Mat3::from_rotation_z(delta_r_deg.z.to_radians())
                * Mat3::from_rotation_y(delta_r_deg.y.to_radians())
                * Mat3::from_rotation_x(delta_r_deg.x.to_radians())
        };

        for (i, lp) in local_pts.into_iter().enumerate() {
            let transformed = rot_local * ((lp - pivot) * scale_factors) + pivot + delta_t_local;
            out.positions[i] = origin + u * transformed.x + v * transformed.y + a * transformed.z;
        }
        out.compute_normals();
        out
    }

    /// Precalcula una sola vez al iniciar el arrastre de Transformación (Gizmo) la geometría local
    /// y la rejilla espacial de intaglio para responder a 60 FPS en milisegundos.
    pub fn prepare_transform_session(
        base_mesh: &TriMeshData,
        insertion_axis: Vec3,
        is_adapted: bool,
        cam_right: Vec3,
        cam_up: Vec3,
        cam_fwd: Vec3,
    ) -> TransformSession {
        let base_mesh = base_mesh.clone();
        let n = base_mesh.positions.len();
        let has_ext = is_adapted && base_mesh.is_external.len() == n;
        let intaglio = if is_adapted {
            collect_intaglio_pts(&base_mesh)
        } else {
            Vec::new()
        };
        let intaglio_grid = if !intaglio.is_empty() {
            Some(PointSpatialGrid::from_points(&intaglio, 0.65))
        } else {
            None
        };

        let u = cam_right.normalize_or_zero();
        let v = cam_up.normalize_or_zero();
        let w = (-cam_fwd).normalize_or_zero();
        let ins_ax = insertion_axis.normalize_or_zero();

        let mut active_indices = Vec::with_capacity(n);
        let mut mins = Vec3::splat(f32::INFINITY);
        let mut maxs = Vec3::splat(f32::NEG_INFINITY);
        let mut sum_pos = Vec3::ZERO;
        let mut z_ins_min = f32::INFINITY;
        let mut z_ins_max = f32::NEG_INFINITY;

        for i in 0..n {
            if has_ext && !base_mesh.is_external[i] {
                continue;
            }
            let p = base_mesh.positions[i];
            sum_pos += p;
            let zi = p.dot(ins_ax);
            z_ins_min = z_ins_min.min(zi);
            z_ins_max = z_ins_max.max(zi);
            active_indices.push(i);
        }
        if active_indices.is_empty() {
            return TransformSession {
                base_mesh,
                is_adapted,
                u,
                v,
                w,
                origin: Vec3::ZERO,
                mins: Vec3::ZERO,
                maxs: Vec3::ZERO,
                active_indices,
                cam_local: Vec::new(),
                base_lock: Vec::new(),
                intaglio_grid,
            };
        }

        let origin = sum_pos / active_indices.len() as f32;
        let z_ins_span = (z_ins_max - z_ins_min).max(2.0);
        let mut cam_local = Vec::with_capacity(active_indices.len());
        let mut base_lock = Vec::with_capacity(active_indices.len());

        for &vi in &active_indices {
            let p = base_mesh.positions[vi];
            let r = p - origin;
            let lp = Vec3::new(r.dot(u), r.dot(v), r.dot(w));
            mins = mins.min(lp);
            maxs = maxs.max(lp);
            cam_local.push(lp);

            let lock = if is_adapted {
                let z_norm = ((p.dot(ins_ax) - z_ins_min) / z_ins_span).clamp(0.0, 1.0);
                if let Some(ig) = &intaglio_grid {
                    let di = ig.nearest_within(p, 0.90).map(|x| x.0).unwrap_or(0.90);
                    smoothstep(0.08, 0.85, di) * smoothstep(0.02, 0.22, z_norm)
                } else {
                    1.0
                }
            } else {
                1.0
            };
            base_lock.push(lock);
        }

        TransformSession {
            base_mesh,
            is_adapted,
            u,
            v,
            w,
            origin,
            mins,
            maxs,
            active_indices,
            cam_local,
            base_lock,
            intaglio_grid,
        }
    }

    /// Evalúa la transformación en vista en `< 1 ms` aprovechando la caché `TransformSession`.
    pub fn apply_transform_session(
        session: &TransformSession,
        delta_world_move: Vec3,
        rot_world: Option<Quat>,
        scale_cam: Vec3,
        anchor_cam: Option<Vec3>,
        min_thickness_mm: f32,
    ) -> TriMeshData {
        let mut out = session.base_mesh.clone();
        if session.active_indices.is_empty() {
            return out;
        }
        let n = out.positions.len();
        let has_th = out.thickness_mm.len() == n;
        let mut pivot = 0.5 * (session.mins + session.maxs);
        if let Some(ad) = anchor_cam {
            if ad.x > 0.5 { pivot.x = session.mins.x; } else if ad.x < -0.5 { pivot.x = session.maxs.x; }
            if ad.y > 0.5 { pivot.y = session.mins.y; } else if ad.y < -0.5 { pivot.y = session.maxs.y; }
            if ad.z > 0.5 { pivot.z = session.mins.z; } else if ad.z < -0.5 { pivot.z = session.maxs.z; }
        }

        let rot_q = rot_world.map(|q| q.normalize()).unwrap_or(Quat::IDENTITY);
        let has_rot = rot_world.is_some();
        let u = session.u;
        let v = session.v;
        let w = session.w;
        let origin = session.origin;

        for (k, &vi) in session.active_indices.iter().enumerate() {
            let lp = session.cam_local[k];
            let scaled_lp = (lp - pivot) * scale_cam + pivot;
            let mut world_target = origin + u * scaled_lp.x + v * scaled_lp.y + w * scaled_lp.z;
            if has_rot {
                world_target = origin + rot_q * (world_target - origin);
            }
            world_target += delta_world_move;

            if session.is_adapted {
                let lock = session.base_lock[k];
                let mut world_p = session.base_mesh.positions[vi].lerp(world_target, lock);
                if let Some(ig) = &session.intaglio_grid {
                    let req = min_thickness_mm * lock * 0.92;
                    let probe = (req + 0.05).max(0.10);
                    if let Some((di, q_in)) = ig.nearest_within(world_p, probe) {
                        if di < req {
                            let push = (world_p - q_in).normalize_or_zero();
                            if push.length_squared() > 1e-6 {
                                world_p = q_in + push * req;
                                if has_th {
                                    out.thickness_mm[vi] = req;
                                }
                            } else if has_th {
                                out.thickness_mm[vi] = di;
                            }
                        } else if has_th {
                            out.thickness_mm[vi] = di;
                        }
                    } else if has_th {
                        out.thickness_mm[vi] = ig.nearest(world_p).0;
                    }
                }
                out.positions[vi] = world_p;
            } else {
                out.positions[vi] = world_target;
            }
        }

        out.compute_normals();
        out
    }

    /// Transforma el diente directamente en el **plano de vista actual de la cámara**
    /// `(cam_right, cam_up, -cam_fwd)` para las flechas fijas estilo 3Shape:
    /// - Traslación 1:1 (`delta_world_move`)
    /// - Rotación en plano de pantalla o libre 3D (`rot_world`)
    /// - Escala unilateral o proporcional estrictamente en el plano que el técnico está viendo
    ///   (`scale_cam`, `anchor_cam`) — funciona tanto en diente libre como en corona adaptada al margen.
    #[allow(clippy::too_many_arguments)]
    pub fn transform_in_view_frame(
        base_mesh: &TriMeshData,
        insertion_axis: Vec3,
        is_adapted: bool,
        cam_right: Vec3,
        cam_up: Vec3,
        cam_fwd: Vec3,
        delta_world_move: Vec3,
        rot_world: Option<Quat>,
        scale_cam: Vec3,
        anchor_cam: Option<Vec3>,
        min_thickness_mm: f32,
    ) -> TriMeshData {
        let sess = Self::prepare_transform_session(
            base_mesh,
            insertion_axis,
            is_adapted,
            cam_right,
            cam_up,
            cam_fwd,
        );
        Self::apply_transform_session(
            &sess,
            delta_world_move,
            rot_world,
            scale_cam,
            anchor_cam,
            min_thickness_mm,
        )
    }

    /// Aplica **Transformación Individual** sobre una corona ya adaptada manteniendo fijo el margen cervical.
    #[allow(dead_code)]
    pub fn transform_adapted_crown_body(
        base_crown: &TriMeshData,
        margin_curve: &[Vec3],
        insertion_axis: Vec3,
        delta_t_local: Vec3,
        delta_r_deg: Vec3,
        free_rot_world: Option<Quat>,
        scale_factors: Vec3,
        anchor_dir_local: Option<Vec3>,
        min_thickness_mm: f32,
    ) -> TriMeshData {
        let mut out = base_crown.clone();
        let n = out.positions.len();
        if n == 0 {
            return out;
        }
        let has_ext = out.is_external.len() == n;
        let intaglio = collect_intaglio_pts(&out);
        let intaglio_grid = if !intaglio.is_empty() {
            Some(PointSpatialGrid::from_points(&intaglio, 0.65))
        } else {
            None
        };

        let (u, v, a) = build_orthonormal_basis(insertion_axis);
        let origin = if !margin_curve.is_empty() {
            margin_curve.iter().copied().sum::<Vec3>() / margin_curve.len() as f32
        } else {
            out.center()
        };

        let mut ext_indices = Vec::new();
        let mut ext_local = Vec::new();
        let mut mins = Vec3::splat(f32::INFINITY);
        let mut maxs = Vec3::splat(f32::NEG_INFINITY);

        for i in 0..n {
            if has_ext && !out.is_external[i] {
                continue;
            }
            let r = out.positions[i] - origin;
            let lp = Vec3::new(r.dot(u), r.dot(v), r.dot(a));
            mins = mins.min(lp);
            maxs = maxs.max(lp);
            ext_indices.push(i);
            ext_local.push(lp);
        }
        if ext_indices.is_empty() {
            return out;
        }

        let z_span = (maxs.z - mins.z).max(2.0);
        let mut pivot = 0.5 * (mins + maxs);
        if let Some(ad) = anchor_dir_local {
            if ad.x > 0.5 { pivot.x = mins.x; } else if ad.x < -0.5 { pivot.x = maxs.x; }
            if ad.y > 0.5 { pivot.y = mins.y; } else if ad.y < -0.5 { pivot.y = maxs.y; }
            if ad.z > 0.5 { pivot.z = mins.z; } else if ad.z < -0.5 { pivot.z = maxs.z; }
        }

        let basis_mat = Mat3::from_cols(u, v, a);
        let rot_mat = if let Some(qw) = free_rot_world {
            basis_mat.transpose() * Mat3::from_quat(qw.normalize()) * basis_mat
        } else {
            Mat3::from_rotation_z(delta_r_deg.z.to_radians())
                * Mat3::from_rotation_y(delta_r_deg.y.to_radians())
                * Mat3::from_rotation_x(delta_r_deg.x.to_radians())
        };

        for (k, &vi) in ext_indices.iter().enumerate() {
            let lp = ext_local[k];
            let z_norm = ((lp.z - mins.z) / z_span).clamp(0.0, 1.0);
            let lock = if let Some(ig) = &intaglio_grid {
                let di = ig.nearest_within(out.positions[vi], 0.90).map(|x| x.0).unwrap_or(0.90);
                smoothstep(0.08, 0.85, di) * smoothstep(0.02, 0.22, z_norm)
            } else {
                1.0
            };

            let transformed = rot_mat * ((lp - pivot) * scale_factors) + pivot + delta_t_local;
            let blended = lp.lerp(transformed, lock);
            let mut world_p = origin + u * blended.x + v * blended.y + a * blended.z;

            if let Some(ig) = &intaglio_grid {
                let req = min_thickness_mm * lock * 0.92;
                let (di, q_in) = ig.nearest(world_p);
                if di < req {
                    let push = (world_p - q_in).normalize_or_zero();
                    if push.length_squared() > 1e-6 {
                        world_p = q_in + push * req;
                    }
                }
                if out.thickness_mm.len() == n {
                    out.thickness_mm[vi] = ig.nearest(world_p).0;
                }
            }
            out.positions[vi] = world_p;
        }

        out.compute_normals();
        out
    }

    /// Modelado anatómico paramétrico por zonas (Cúspides, Fosa, Ecuador).
    pub fn apply_regional_morphing(
        base_crown: &TriMeshData,
        margin_curve: &[Vec3],
        insertion_axis: Vec3,
        cusp_delta_mm: f32,
        fossa_delta_mm: f32,
        equator_delta_mm: f32,
    ) -> TriMeshData {
        if base_crown.is_empty()
            || (cusp_delta_mm.abs() < 1e-4 && fossa_delta_mm.abs() < 1e-4 && equator_delta_mm.abs() < 1e-4)
        {
            return base_crown.clone();
        }
        let mut out = base_crown.clone();
        let n = out.positions.len();
        let has_ext = out.is_external.len() == n;
        let (u, v, a) = build_orthonormal_basis(insertion_axis);
        let origin = if !margin_curve.is_empty() {
            margin_curve.iter().copied().sum::<Vec3>() / margin_curve.len() as f32
        } else {
            out.center()
        };

        let mut ext_local = Vec::new();
        let mut ext_idx = Vec::new();
        let mut z_min = f32::INFINITY;
        let mut z_max = f32::NEG_INFINITY;
        let mut r_list = Vec::new();

        for i in 0..n {
            if has_ext && !out.is_external[i] {
                continue;
            }
            let rel = out.positions[i] - origin;
            let lp = Vec3::new(rel.dot(u), rel.dot(v), rel.dot(a));
            z_min = z_min.min(lp.z);
            z_max = z_max.max(lp.z);
            r_list.push(lp.x.hypot(lp.y));
            ext_local.push(lp);
            ext_idx.push(i);
        }
        if ext_idx.is_empty() {
            return out;
        }

        r_list.sort_by(|x, y| x.partial_cmp(y).unwrap_or(std::cmp::Ordering::Equal));
        let r_max = r_list[((r_list.len() as f32 * 0.95) as usize).min(r_list.len() - 1)].max(2.5);
        let z_span = (z_max - z_min).max(2.0);

        for (k, &vi) in ext_idx.iter().enumerate() {
            let mut lp = ext_local[k];
            let r_xy = lp.x.hypot(lp.y);
            let r_norm = (r_xy / r_max).clamp(0.0, 1.2);
            let z_norm = ((lp.z - z_min) / z_span).clamp(0.0, 1.0);
            let m_lock = smoothstep(0.05, 0.30, z_norm);

            if fossa_delta_mm.abs() >= 1e-4 {
                let fw = (-5.0 * r_norm * r_norm).exp() * smoothstep(0.35, 0.70, z_norm) * m_lock;
                lp.z += fossa_delta_mm * fw;
            }
            if cusp_delta_mm.abs() >= 1e-4 {
                let cw = (-6.5 * (r_norm - 0.62).powi(2)).exp() * smoothstep(0.45, 0.80, z_norm) * m_lock;
                lp.z += cusp_delta_mm * cw;
            }
            if equator_delta_mm.abs() >= 1e-4 {
                let ew = (std::f32::consts::PI * z_norm).sin() * m_lock;
                let dir2 = Vec2::new(lp.x, lp.y).normalize_or_zero();
                lp.x += dir2.x * equator_delta_mm * ew;
                lp.y += dir2.y * equator_delta_mm * ew;
            }

            out.positions[vi] = origin + u * lp.x + v * lp.y + a * lp.z;
        }

        out.compute_normals();
        out
    }
}
