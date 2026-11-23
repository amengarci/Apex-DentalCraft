//! Núcleo de Malla Triangular (`TriMeshData`), I/O STL (`stl_io`), BVH (`parry3d`) y Auditoría CAM.

use glam::Vec3;
use parry3d::math::Vector as PVec;
use parry3d::query::{PointQuery, PointQueryWithLocation, Ray, RayCast};
use parry3d::shape::TriMesh as ParryTriMesh;
use std::collections::{HashMap, HashSet};
use std::fs::File;
use std::io::{BufRead, BufReader, BufWriter, Write};
use std::path::Path;

#[inline]
pub fn to_pvec(v: Vec3) -> PVec {
    PVec::new(v.x, v.y, v.z)
}

#[inline]
pub fn from_pvec(v: PVec) -> Vec3 {
    Vec3::new(v.x, v.y, v.z)
}

/// Resultado de auditoría CAM previa a fresado / impresión 3D.
#[derive(Debug, Clone, Default)]
pub struct CamAudit {
    pub valid: bool,
    pub watertight: bool,
    pub volume_mm3: f32,
    pub dims_mm: Vec3,
    pub n_vertices: usize,
    pub n_faces: usize,
    pub open_edges: usize,
    pub non_manifold_edges: usize,
    pub flipped_winding_edges: usize,
    pub folded_edges: usize,
}

/// Estructura de malla 3D optimizada para renderizado OpenGL, raycasting BVH y esculpido libre.
#[derive(Debug, Clone, Default)]
pub struct TriMeshData {
    pub positions: Vec<Vec3>,
    pub indices: Vec<[u32; 3]>,
    pub normals: Vec<Vec3>,
    /// `true` para vértices externos editables; `false` para intaglio / anillo marginal sellado.
    pub is_external: Vec<bool>,
    pub thickness_mm: Vec<f32>,
    pub occlusal_clearance_mm: Vec<f32>,
    /// Escalar auxiliar (ej. ángulo de socavado en Módulo 2 o curvatura cervical en Módulo 3).
    pub custom_scalars: Vec<f32>,
}

/// Envoltorio acelerado BVH sobre `parry3d::shape::TriMesh`.
pub struct MeshCollider {
    pub bvh: ParryTriMesh,
}

impl MeshCollider {
    pub fn from_mesh(mesh: &TriMeshData) -> Option<Self> {
        if mesh.positions.is_empty() || mesh.indices.is_empty() {
            return None;
        }
        let verts: Vec<PVec> = mesh.positions.iter().copied().map(to_pvec).collect();
        let bvh = ParryTriMesh::new(verts, mesh.indices.clone()).ok()?;
        Some(Self { bvh })
    }

    /// Lanza un rayo 3D `(origin, dir)` y devuelve `(distancia_t, punto_impacto)` si impacta la malla.
    pub fn cast_ray(&self, origin: Vec3, dir: Vec3, max_toi: f32) -> Option<(f32, Vec3)> {
        let len = dir.length();
        if len < 1e-7 {
            return None;
        }
        let d = dir / len;
        let ray = Ray::new(to_pvec(origin), to_pvec(d));
        let toi = self.bvh.cast_local_ray(&ray, max_toi, true)?;
        Some((toi, origin + d * toi))
    }

    /// Lanza un rayo 3D y devuelve `(distancia_t, punto_impacto, normal_opuesta_al_rayo)`.
    /// Como `normal · dir <= 0` independientemente del devanado del STL, sobre un rayo `+Z`
    /// la normal del impacto apunta siempre hacia abajo (`-Z`):
    /// - Voladizo del propio muñón (`/`): componente radial positiva (`normal · out_dir > 0`).
    /// - Encía externa / diente vecino (`\`): componente radial negativa (`normal · out_dir < 0`).
    pub fn cast_ray_with_normal(
        &self,
        origin: Vec3,
        dir: Vec3,
        max_toi: f32,
    ) -> Option<(f32, Vec3, Vec3)> {
        let len = dir.length();
        if len < 1e-7 {
            return None;
        }
        let d = dir / len;
        let ray = Ray::new(to_pvec(origin), to_pvec(d));
        let hit = self.bvh.cast_local_ray_and_get_normal(&ray, max_toi, false)?;
        let mut n = from_pvec(hit.normal).normalize_or_zero();
        if n.dot(d) > 0.0 {
            n = -n;
        }
        Some((hit.time_of_impact, origin + d * hit.time_of_impact, n))
    }

    /// Determina exactamente si un punto 3D está dentro de una malla cerrada watertight
    /// mediante paridad de Jordan-Brouwer con 3 rayos no colineales (voto mayoritario,
    /// inmune a socavados, voladizos y rozaduras de aristas).
    pub fn contains_point_closed(&self, point: Vec3) -> bool {
        let dirs = [
            Vec3::new(0.031, 0.019, 1.0).normalize(),
            Vec3::new(1.0, 0.043, 0.057).normalize(),
            Vec3::new(-0.491, 0.870, 0.041).normalize(),
        ];
        let mut inside_votes = 0usize;
        for d in dirs {
            let mut crossings = 0usize;
            let mut cur_orig = point;
            let mut rem = 45.0_f32;
            for _ in 0..12 {
                let ray = Ray::new(to_pvec(cur_orig), to_pvec(d));
                if let Some(toi) = self.bvh.cast_local_ray(&ray, rem, false) {
                    crossings += 1;
                    let step = toi + 5e-4;
                    if step >= rem {
                        break;
                    }
                    cur_orig += d * step;
                    rem -= step;
                } else {
                    break;
                }
            }
            if crossings % 2 == 1 {
                inside_votes += 1;
            }
        }
        inside_votes >= 2
    }

    /// Proyecta un punto 3D arbitrario al punto exacto más cercano sobre la superficie triangular.
    pub fn project_point(&self, point: Vec3) -> Vec3 {
        let proj = self.bvh.project_local_point(to_pvec(point), false);
        from_pvec(proj.point)
    }

    /// Devuelve `(punto_más_cercano, distancia_euclídea)` hacia la malla.
    #[allow(dead_code)]
    pub fn closest_point_and_dist(&self, point: Vec3) -> (Vec3, f32) {
        let closest = self.project_point(point);
        (closest, closest.distance(point))
    }

    /// Devuelve `(punto_más_cercano, distancia_euclídea, normal_geométrica_triángulo)`.
    pub fn closest_point_dist_and_tri_normal(&self, point: Vec3) -> (Vec3, f32, Vec3) {
        let (proj, (tri_idx, _)) = self.bvh.project_local_point_and_get_location(to_pvec(point), false);
        let closest = from_pvec(proj.point);
        let tri = self.bvh.triangle(tri_idx);
        let n = (tri.b - tri.a).cross(tri.c - tri.a);
        (closest, closest.distance(point), from_pvec(n).normalize_or_zero())
    }
}

impl TriMeshData {
    pub fn new(positions: Vec<Vec3>, indices: Vec<[u32; 3]>) -> Self {
        let n = positions.len();
        let mut m = Self {
            positions,
            indices,
            normals: vec![Vec3::Z; n],
            is_external: vec![true; n],
            thickness_mm: Vec::new(),
            occlusal_clearance_mm: Vec::new(),
            custom_scalars: Vec::new(),
        };
        m.compute_normals();
        m
    }

    pub fn is_empty(&self) -> bool {
        self.positions.is_empty() || self.indices.is_empty()
    }

    pub fn bounds(&self) -> (Vec3, Vec3) {
        if self.positions.is_empty() {
            return (Vec3::ZERO, Vec3::ZERO);
        }
        let mut min_v = Vec3::splat(f32::INFINITY);
        let mut max_v = Vec3::splat(f32::NEG_INFINITY);
        for &p in &self.positions {
            min_v = min_v.min(p);
            max_v = max_v.max(p);
        }
        (min_v, max_v)
    }

    pub fn center(&self) -> Vec3 {
        let (mn, mx) = self.bounds();
        0.5 * (mn + mx)
    }

    #[allow(dead_code)]
    pub fn radius(&self) -> f32 {
        let (mn, mx) = self.bounds();
        0.5 * (mx - mn).length().max(1.0)
    }

    /// Recalcula normales suaves ponderadas por área triangular.
    pub fn compute_normals(&mut self) {
        let n = self.positions.len();
        let mut acc = vec![Vec3::ZERO; n];
        for &[i0, i1, i2] in &self.indices {
            let (a, b, c) = (i0 as usize, i1 as usize, i2 as usize);
            if a >= n || b >= n || c >= n {
                continue;
            }
            let e1 = self.positions[b] - self.positions[a];
            let e2 = self.positions[c] - self.positions[a];
            let fn_area = e1.cross(e2);
            acc[a] += fn_area;
            acc[b] += fn_area;
            acc[c] += fn_area;
        }
        self.normals = acc
            .into_iter()
            .map(|v| {
                let len = v.length();
                if len > 1e-8 { v / len } else { Vec3::Z }
            })
            .collect();
    }

    /// Invierte la orientación de todos los triángulos y recalcula normales.
    pub fn flip_faces(&mut self) {
        for tri in &mut self.indices {
            tri.swap(1, 2);
        }
        self.compute_normals();
    }

    /// Verifica el volumen con signo y corrige orientación si las caras de un sólido cerrado apuntan hacia adentro.
    pub fn ensure_outward_winding_if_closed(&mut self) {
        let vol = self.signed_volume();
        if vol < -1e-4 {
            self.flip_faces();
        }
    }

    /// Volumen con signo usando suma tetraédrica exacta ($\frac{1}{6}\sum v_0 \cdot (v_1 \times v_2)$).
    pub fn signed_volume(&self) -> f32 {
        let n = self.positions.len();
        let mut vol6 = 0.0_f64;
        for &[i0, i1, i2] in &self.indices {
            let (a, b, c) = (i0 as usize, i1 as usize, i2 as usize);
            if a >= n || b >= n || c >= n {
                continue;
            }
            let p0 = self.positions[a].as_dvec3();
            let p1 = self.positions[b].as_dvec3();
            let p2 = self.positions[c].as_dvec3();
            vol6 += p0.dot(p1.cross(p2));
        }
        (vol6 / 6.0) as f32
    }

    /// Construye la lista de adyacencia topológica 1-anillo (operador paraguas).
    pub fn build_adjacency(&self) -> Vec<Vec<usize>> {
        let n = self.positions.len();
        let mut sets: Vec<HashSet<usize>> = vec![HashSet::new(); n];
        for &[i0, i1, i2] in &self.indices {
            let (a, b, c) = (i0 as usize, i1 as usize, i2 as usize);
            if a >= n || b >= n || c >= n {
                continue;
            }
            sets[a].insert(b);
            sets[a].insert(c);
            sets[b].insert(a);
            sets[b].insert(c);
            sets[c].insert(a);
            sets[c].insert(b);
        }
        sets.into_iter().map(|s| s.into_iter().collect()).collect()
    }

    /// Suelda vértices duplicados dentro de `tol` mm y descarta triángulos degenerados.
    pub fn weld_vertices(&self, tol: f32) -> Self {
        let inv_tol = 1.0 / tol.max(1e-6);
        let mut map: HashMap<(i64, i64, i64), u32> = HashMap::with_capacity(self.positions.len());
        let mut new_pos: Vec<Vec3> = Vec::with_capacity(self.positions.len());
        let mut remap: Vec<u32> = Vec::with_capacity(self.positions.len());
        let mut new_ext: Vec<bool> = Vec::with_capacity(self.positions.len());

        for (i, &p) in self.positions.iter().enumerate() {
            let key = (
                (p.x * inv_tol).round() as i64,
                (p.y * inv_tol).round() as i64,
                (p.z * inv_tol).round() as i64,
            );
            let new_idx = *map.entry(key).or_insert_with(|| {
                let id = new_pos.len() as u32;
                new_pos.push(p);
                new_ext.push(self.is_external.get(i).copied().unwrap_or(true));
                id
            });
            remap.push(new_idx);
        }

        let mut new_indices = Vec::with_capacity(self.indices.len());
        let mut seen_tris: HashSet<(u32, u32, u32)> = HashSet::with_capacity(self.indices.len());
        for &[i0, i1, i2] in &self.indices {
            let a = remap[i0 as usize];
            let b = remap[i1 as usize];
            let c = remap[i2 as usize];
            if a == b || b == c || c == a {
                continue;
            }
            // Descartar triángulos de área casi nula
            let e1 = new_pos[b as usize] - new_pos[a as usize];
            let e2 = new_pos[c as usize] - new_pos[a as usize];
            if e1.cross(e2).length_squared() < 1e-12 {
                continue;
            }
            let mut sorted = [a, b, c];
            sorted.sort_unstable();
            if seen_tris.insert((sorted[0], sorted[1], sorted[2])) {
                new_indices.push([a, b, c]);
            }
        }

        let mut out = TriMeshData::new(new_pos, new_indices);
        out.is_external = new_ext;
        out
    }

    /// Suelda vértices duplicados al cargar STL preservando 100% la integridad watertight
    /// (nunca borra triángulos válidos delgados en el margen o bisel que dejarían micro-huecos).
    pub fn weld_vertices_preserve_watertight(&self, tol: f32) -> Self {
        let inv_tol = 1.0 / tol.max(1e-6);
        let mut map: HashMap<(i64, i64, i64), u32> = HashMap::with_capacity(self.positions.len());
        let mut new_pos: Vec<Vec3> = Vec::with_capacity(self.positions.len());
        let mut remap: Vec<u32> = Vec::with_capacity(self.positions.len());
        let mut new_ext: Vec<bool> = Vec::with_capacity(self.positions.len());

        for (i, &p) in self.positions.iter().enumerate() {
            let key = (
                (p.x * inv_tol).round() as i64,
                (p.y * inv_tol).round() as i64,
                (p.z * inv_tol).round() as i64,
            );
            let new_idx = *map.entry(key).or_insert_with(|| {
                let id = new_pos.len() as u32;
                new_pos.push(p);
                new_ext.push(self.is_external.get(i).copied().unwrap_or(true));
                id
            });
            remap.push(new_idx);
        }

        let mut new_indices = Vec::with_capacity(self.indices.len());
        let mut seen_tris: HashSet<(u32, u32, u32)> = HashSet::with_capacity(self.indices.len());
        for &[i0, i1, i2] in &self.indices {
            let a = remap[i0 as usize];
            let b = remap[i1 as usize];
            let c = remap[i2 as usize];
            if a == b || b == c || c == a {
                continue;
            }
            let mut sorted = [a, b, c];
            sorted.sort_unstable();
            if seen_tris.insert((sorted[0], sorted[1], sorted[2])) {
                new_indices.push([a, b, c]);
            }
        }

        let mut out = TriMeshData::new(new_pos, new_indices);
        out.is_external = new_ext;
        out
    }

    /// Elimina vértices huérfanos no referenciados por ningún triángulo y renumera índices.
    pub fn compact_unreferenced_vertices(&self) -> Self {
        let n = self.positions.len();
        let mut used = vec![false; n];
        for &[a, b, c] in &self.indices {
            if let Some(u) = used.get_mut(a as usize) {
                *u = true;
            }
            if let Some(u) = used.get_mut(b as usize) {
                *u = true;
            }
            if let Some(u) = used.get_mut(c as usize) {
                *u = true;
            }
        }
        if used.iter().all(|&u| u) {
            return self.clone();
        }
        let has_scalars = self.custom_scalars.len() == n;
        let mut new_pos = Vec::with_capacity(n);
        let mut new_ext = Vec::with_capacity(n);
        let mut new_scalars = if has_scalars { Vec::with_capacity(n) } else { Vec::new() };
        let mut remap = vec![u32::MAX; n];
        for i in 0..n {
            if used[i] {
                let id = new_pos.len() as u32;
                remap[i] = id;
                new_pos.push(self.positions[i]);
                new_ext.push(self.is_external.get(i).copied().unwrap_or(true));
                if has_scalars {
                    new_scalars.push(self.custom_scalars[i]);
                }
            }
        }
        let mut new_indices = Vec::with_capacity(self.indices.len());
        for &[a, b, c] in &self.indices {
            let ra = remap[a as usize];
            let rb = remap[b as usize];
            let rc = remap[c as usize];
            if ra != u32::MAX && rb != u32::MAX && rc != u32::MAX {
                new_indices.push([ra, rb, rc]);
            }
        }
        let mut out = TriMeshData::new(new_pos, new_indices);
        out.is_external = new_ext;
        out.custom_scalars = new_scalars;
        out
    }

    /// Calcula el grosor real de pared (mm) de cualquier malla STL individual (ej. corona aislada)
    /// lanzando rayos internos desde cada vértice a lo largo de `-normal` sobre su propio BVH `parry3d`.
    /// Devuelve `(min_mm, avg_mm, max_mm)` y rellena `self.thickness_mm`.
    pub fn compute_self_wall_thickness(&mut self) -> (f32, f32, f32) {
        let n = self.positions.len();
        if n == 0 || self.indices.is_empty() {
            self.thickness_mm.clear();
            return (0.0, 0.0, 0.0);
        }
        if self.normals.len() != n {
            self.compute_normals();
        }
        let Some(col) = MeshCollider::from_mesh(self) else {
            self.thickness_mm = vec![1.0; n];
            return (1.0, 1.0, 1.0);
        };
        let mut thick = vec![1.5_f32; n];
        let mut min_t = f32::INFINITY;
        let mut max_t = 0.0_f32;
        let mut sum_t = 0.0_f64;
        let mut cnt = 0usize;

        for i in 0..n {
            let p = self.positions[i];
            let nrm = self.normals[i].normalize_or_zero();
            if nrm.length_squared() < 0.25 {
                continue;
            }
            let dir = -nrm;
            let orig = p + dir * 0.025;
            let mut best = f32::INFINITY;
            // Rayo directo + 4 rayos cónicos de respaldo (±12°) para capturar espesor real en bordes
            let ray = Ray::new(to_pvec(orig), to_pvec(dir));
            if let Some(toi) = col.bvh.cast_local_ray(&ray, 10.0, false) {
                best = best.min(toi + 0.025);
            }
            let (t1, t2, _) = build_orthonormal_basis(dir);
            for (a, b) in [(0.18_f32, 0.0), (-0.18, 0.0), (0.0, 0.18), (0.0, -0.18)] {
                let d_alt = (dir + t1 * a + t2 * b).normalize_or_zero();
                let r_alt = Ray::new(to_pvec(p + d_alt * 0.025), to_pvec(d_alt));
                if let Some(toi) = col.bvh.cast_local_ray(&r_alt, 10.0, false) {
                    let est = (toi + 0.025) * 0.98;
                    if est < best {
                        best = est;
                    }
                }
            }
            let val = if best.is_finite() {
                best.clamp(0.0, 3.0)
            } else {
                2.0
            };
            thick[i] = val;
            if val > 0.04 {
                min_t = min_t.min(val);
                max_t = max_t.max(val);
                sum_t += val as f64;
                cnt += 1;
            }
        }
        self.thickness_mm = thick;
        if cnt == 0 {
            (0.0, 0.0, 0.0)
        } else {
            (min_t, (sum_t / cnt as f64) as f32, max_t)
        }
    }

    /// Calcula la distancia exacta (mm) de cada vértice hacia otra malla (ej. Corona vs Antagonista
    /// o Corona vs Modelo/Muñón) rellenando `occlusal_clearance_mm` o `thickness_mm`.
    pub fn compute_distance_to_mesh(&mut self, other: &TriMeshData, store_as_occlusal: bool) -> (f32, f32, f32) {
        let n = self.positions.len();
        if n == 0 || other.is_empty() {
            return (0.0, 0.0, 0.0);
        }
        let Some(other_col) = MeshCollider::from_mesh(other) else {
            return (0.0, 0.0, 0.0);
        };
        let mut vals = vec![2.5_f32; n];
        let mut min_d = f32::INFINITY;
        let mut max_d = 0.0_f32;
        let mut sum_d = 0.0_f64;
        for i in 0..n {
            let p = self.positions[i];
            let (_, d) = other_col.closest_point_and_dist(p);
            vals[i] = d;
            min_d = min_d.min(d);
            max_d = max_d.max(d);
            sum_d += d as f64;
        }
        if store_as_occlusal {
            self.occlusal_clearance_mm = vals;
        } else {
            self.thickness_mm = vals;
        }
        (min_d, (sum_d / n as f64) as f32, max_d)
    }

    /// Subdivisión uniforme 1-a-4 (midpoint + ligera relajación Loop) para evitar triángulos gruesos al esculpir.
    pub fn subdivide_uniform(&self) -> Self {
        let mut new_pos = self.positions.clone();
        let mut new_ext = if self.is_external.len() == self.positions.len() {
            self.is_external.clone()
        } else {
            vec![true; self.positions.len()]
        };
        let mut edge_mid: HashMap<(u32, u32), u32> = HashMap::new();
        let mut new_tris = Vec::with_capacity(self.indices.len() * 4);

        let mut get_mid = |u: u32, v: u32, pos: &mut Vec<Vec3>, ext: &mut Vec<bool>| -> u32 {
            let key = if u < v { (u, v) } else { (v, u) };
            *edge_mid.entry(key).or_insert_with(|| {
                let id = pos.len() as u32;
                pos.push(0.5 * (pos[u as usize] + pos[v as usize]));
                ext.push(ext[u as usize] && ext[v as usize]);
                id
            })
        };

        for &[a, b, c] in &self.indices {
            let ab = get_mid(a, b, &mut new_pos, &mut new_ext);
            let bc = get_mid(b, c, &mut new_pos, &mut new_ext);
            let ca = get_mid(c, a, &mut new_pos, &mut new_ext);
            new_tris.push([a, ab, ca]);
            new_tris.push([b, bc, ab]);
            new_tris.push([c, ca, bc]);
            new_tris.push([ab, bc, ca]);
        }

        let mut out = TriMeshData::new(new_pos, new_tris);
        out.is_external = new_ext;
        // 1 pasada suave Taubin sobre toda la malla subdividida para redondear facetas planas
        let adj = out.build_adjacency();
        let n = out.positions.len();
        let mut tmp = out.positions.clone();
        for i in 0..n {
            if !adj[i].is_empty() {
                let avg: Vec3 = adj[i].iter().map(|&j| out.positions[j]).sum::<Vec3>()
                    / adj[i].len() as f32;
                tmp[i] = out.positions[i] + 0.35 * (avg - out.positions[i]);
            }
        }
        for i in 0..n {
            if !adj[i].is_empty() {
                let avg: Vec3 = adj[i].iter().map(|&j| tmp[j]).sum::<Vec3>()
                    / adj[i].len() as f32;
                out.positions[i] = tmp[i] - 0.36 * (avg - tmp[i]);
            }
        }
        out.compute_normals();
        out
    }

    /// Asegura una densidad mínima de vértices para esculpido / Morphing fluido.
    pub fn ensure_sculpt_density(&self, min_vertices: usize) -> Self {
        let mut out = self.weld_vertices(1e-4);
        let mut passes = 0;
        while out.positions.len() < min_vertices && out.positions.len() > 0 && passes < 2 {
            out = out.subdivide_uniform();
            passes += 1;
        }
        out
    }

    /// Combina otra malla en esta misma instancia (útil para construir gizmos 3D).
    pub fn merge_in_place(&mut self, other: &TriMeshData) {
        let offset = self.positions.len() as u32;
        self.positions.extend_from_slice(&other.positions);
        self.normals.extend_from_slice(&other.normals);
        if other.is_external.len() == other.positions.len() {
            self.is_external.extend_from_slice(&other.is_external);
        } else {
            self.is_external.extend(std::iter::repeat(true).take(other.positions.len()));
        }
        for &[a, b, c] in &other.indices {
            self.indices.push([a + offset, b + offset, c + offset]);
        }
    }

    /// Auditoría geométrica CAM (verifica condición watertight 2-manifold exacta, pliegues y volumen).
    pub fn inspect_cam(&self) -> CamAudit {
        if self.is_empty() {
            return CamAudit::default();
        }
        let n_v = self.positions.len();
        let mut face_normals: Vec<Vec3> = Vec::with_capacity(self.indices.len());
        let mut edge_faces: HashMap<(u32, u32), Vec<(usize, bool)>> =
            HashMap::with_capacity(self.indices.len() * 3);

        for (fi, &[a, b, c]) in self.indices.iter().enumerate() {
            let fnrm = if (a as usize) < n_v && (b as usize) < n_v && (c as usize) < n_v {
                (self.positions[b as usize] - self.positions[a as usize])
                    .cross(self.positions[c as usize] - self.positions[a as usize])
                    .normalize_or_zero()
            } else {
                Vec3::ZERO
            };
            face_normals.push(fnrm);
            for (u, v) in [(a, b), (b, c), (c, a)] {
                let (key, dir) = if u < v { ((u, v), true) } else { ((v, u), false) };
                edge_faces.entry(key).or_default().push((fi, dir));
            }
        }

        let mut open_edges = 0usize;
        let mut non_manifold_edges = 0usize;
        let mut flipped_winding_edges = 0usize;
        let mut folded_edges = 0usize;

        for list in edge_faces.values() {
            match list.len() {
                1 => open_edges += 1,
                2 => {
                    if list[0].1 == list[1].1 {
                        flipped_winding_edges += 1;
                    }
                    let n1 = face_normals[list[0].0];
                    let n2 = face_normals[list[1].0];
                    // Pliegue/rendija real (< 15° entre caras adyacentes; no marca falsamente biseles marginales limpios de 25°-45°)
                    if n1.length_squared() > 0.5 && n2.length_squared() > 0.5 && n1.dot(n2) < -0.965 {
                        folded_edges += 1;
                    }
                }
                _ => non_manifold_edges += 1,
            }
        }
        let watertight =
            open_edges == 0 && non_manifold_edges == 0 && flipped_winding_edges == 0 && !self.indices.is_empty();
        let (mn, mx) = self.bounds();
        let dims = mx - mn;
        let vol = self.signed_volume().abs();

        CamAudit {
            valid: true,
            watertight,
            volume_mm3: vol,
            dims_mm: dims,
            n_vertices: self.positions.len(),
            n_faces: self.indices.len(),
            open_edges,
            non_manifold_edges,
            flipped_winding_edges,
            folded_edges,
        }
    }

    // ======================================================================
    // Carga y Exportación Multi-Formato (STL Binario/ASCII con `stl_io`, OBJ, PLY)
    // ======================================================================

    pub fn load_from_path(path: &Path) -> Result<Self, String> {
        let ext = path
            .extension()
            .and_then(|s| s.to_str())
            .unwrap_or("")
            .to_lowercase();
        match ext.as_str() {
            "stl" => Self::load_stl(path),
            "obj" => Self::load_obj(path),
            "ply" => Self::load_ply(path),
            _ => Self::load_stl(path),
        }
    }

    pub fn load_stl(path: &Path) -> Result<Self, String> {
        let file = File::open(path).map_err(|e| format!("No se pudo abrir {:?}: {}", path, e))?;
        let mut reader = BufReader::new(file);
        let indexed = stl_io::read_stl(&mut reader)
            .map_err(|e| format!("Error al leer STL {:?}: {}", path, e))?;

        let positions: Vec<Vec3> = indexed
            .vertices
            .iter()
            .map(|v| Vec3::new(v[0], v[1], v[2]))
            .collect();
        let indices: Vec<[u32; 3]> = indexed
            .faces
            .iter()
            .map(|f| [f.vertices[0] as u32, f.vertices[1] as u32, f.vertices[2] as u32])
            .collect();

        let raw = TriMeshData::new(positions, indices);
        Ok(raw.weld_vertices_preserve_watertight(1e-5))
    }

    pub fn save_stl_binary(&self, path: &Path) -> Result<(), String> {
        if let Some(parent) = path.parent() {
            let _ = std::fs::create_dir_all(parent);
        }
        let file = File::create(path).map_err(|e| format!("No se pudo crear {:?}: {}", path, e))?;
        let mut writer = BufWriter::new(file);
        let n = self.positions.len();
        let mut tris = Vec::with_capacity(self.indices.len());
        for &[i0, i1, i2] in &self.indices {
            let (a, b, c) = (i0 as usize, i1 as usize, i2 as usize);
            if a >= n || b >= n || c >= n {
                continue;
            }
            let p0 = self.positions[a];
            let p1 = self.positions[b];
            let p2 = self.positions[c];
            let fnrm = (p1 - p0).cross(p2 - p0).normalize_or_zero();
            tris.push(stl_io::Triangle {
                normal: stl_io::Normal::new([fnrm.x, fnrm.y, fnrm.z]),
                vertices: [
                    stl_io::Vertex::new([p0.x, p0.y, p0.z]),
                    stl_io::Vertex::new([p1.x, p1.y, p1.z]),
                    stl_io::Vertex::new([p2.x, p2.y, p2.z]),
                ],
            });
        }
        stl_io::write_stl(&mut writer, tris.iter())
            .map_err(|e| format!("Error al escribir STL binario {:?}: {}", path, e))
    }

    pub fn save_stl_ascii(&self, path: &Path) -> Result<(), String> {
        if let Some(parent) = path.parent() {
            let _ = std::fs::create_dir_all(parent);
        }
        let file = File::create(path).map_err(|e| format!("No se pudo crear {:?}: {}", path, e))?;
        let mut w = BufWriter::new(file);
        writeln!(w, "solid ApexDentalCraft").map_err(|e| e.to_string())?;
        let n = self.positions.len();
        for &[i0, i1, i2] in &self.indices {
            let (a, b, c) = (i0 as usize, i1 as usize, i2 as usize);
            if a >= n || b >= n || c >= n {
                continue;
            }
            let p0 = self.positions[a];
            let p1 = self.positions[b];
            let p2 = self.positions[c];
            let fnrm = (p1 - p0).cross(p2 - p0).normalize_or_zero();
            writeln!(w, "  facet normal {:.6} {:.6} {:.6}", fnrm.x, fnrm.y, fnrm.z)
                .map_err(|e| e.to_string())?;
            writeln!(w, "    outer loop").map_err(|e| e.to_string())?;
            writeln!(w, "      vertex {:.6} {:.6} {:.6}", p0.x, p0.y, p0.z).map_err(|e| e.to_string())?;
            writeln!(w, "      vertex {:.6} {:.6} {:.6}", p1.x, p1.y, p1.z).map_err(|e| e.to_string())?;
            writeln!(w, "      vertex {:.6} {:.6} {:.6}", p2.x, p2.y, p2.z).map_err(|e| e.to_string())?;
            writeln!(w, "    endloop").map_err(|e| e.to_string())?;
            writeln!(w, "  endfacet").map_err(|e| e.to_string())?;
        }
        writeln!(w, "endsolid ApexDentalCraft").map_err(|e| e.to_string())
    }

    pub fn save_obj(&self, path: &Path) -> Result<(), String> {
        if let Some(parent) = path.parent() {
            let _ = std::fs::create_dir_all(parent);
        }
        let file = File::create(path).map_err(|e| e.to_string())?;
        let mut w = BufWriter::new(file);
        writeln!(w, "# Apex DentalCraft Wavefront OBJ Export").map_err(|e| e.to_string())?;
        for p in &self.positions {
            writeln!(w, "v {:.6} {:.6} {:.6}", p.x, p.y, p.z).map_err(|e| e.to_string())?;
        }
        for &[a, b, c] in &self.indices {
            writeln!(w, "f {} {} {}", a + 1, b + 1, c + 1).map_err(|e| e.to_string())?;
        }
        Ok(())
    }

    pub fn load_obj(path: &Path) -> Result<Self, String> {
        let file = File::open(path).map_err(|e| format!("No se pudo abrir OBJ {:?}: {}", path, e))?;
        let reader = BufReader::new(file);
        let mut positions = Vec::new();
        let mut indices = Vec::new();
        for line in reader.lines().map_while(Result::ok) {
            let trimmed = line.trim();
            if let Some(rest) = trimmed.strip_prefix("v ") {
                let parts: Vec<f32> = rest
                    .split_whitespace()
                    .filter_map(|s| s.parse::<f32>().ok())
                    .collect();
                if parts.len() >= 3 {
                    positions.push(Vec3::new(parts[0], parts[1], parts[2]));
                }
            } else if let Some(rest) = trimmed.strip_prefix("f ") {
                let verts: Vec<u32> = rest
                    .split_whitespace()
                    .filter_map(|tok| {
                        let idx_str = tok.split('/').next()?;
                        let v: i64 = idx_str.parse().ok()?;
                        if v > 0 {
                            Some((v - 1) as u32)
                        } else if v < 0 {
                            Some((positions.len() as i64 + v).max(0) as u32)
                        } else {
                            None
                        }
                    })
                    .collect();
                if verts.len() >= 3 {
                    for k in 1..(verts.len() - 1) {
                        indices.push([verts[0], verts[k], verts[k + 1]]);
                    }
                }
            }
        }
        if positions.is_empty() || indices.is_empty() {
            return Err(format!("Archivo OBJ vacío o sin geometría: {:?}", path));
        }
        Ok(TriMeshData::new(positions, indices).weld_vertices(1e-4))
    }

    pub fn load_ply(path: &Path) -> Result<Self, String> {
        let file = File::open(path).map_err(|e| format!("No se pudo abrir PLY {:?}: {}", path, e))?;
        let mut reader = BufReader::new(file);
        let mut n_verts = 0usize;
        let mut n_faces = 0usize;
        let mut is_ascii = false;
        let mut line = String::new();
        loop {
            line.clear();
            if reader.read_line(&mut line).map_err(|e| e.to_string())? == 0 {
                break;
            }
            let t = line.trim();
            if t.starts_with("format ascii") {
                is_ascii = true;
            } else if let Some(rest) = t.strip_prefix("element vertex ") {
                n_verts = rest.trim().parse().unwrap_or(0);
            } else if let Some(rest) = t.strip_prefix("element face ") {
                n_faces = rest.trim().parse().unwrap_or(0);
            } else if t == "end_header" {
                break;
            }
        }
        if !is_ascii {
            return Err("Solo se admiten PLY ASCII o archivos STL/OBJ en el lector nativo.".to_string());
        }
        let mut positions = Vec::with_capacity(n_verts);
        for _ in 0..n_verts {
            line.clear();
            if reader.read_line(&mut line).unwrap_or(0) == 0 {
                break;
            }
            let nums: Vec<f32> = line
                .split_whitespace()
                .filter_map(|s| s.parse::<f32>().ok())
                .collect();
            if nums.len() >= 3 {
                positions.push(Vec3::new(nums[0], nums[1], nums[2]));
            }
        }
        let mut indices = Vec::with_capacity(n_faces);
        for _ in 0..n_faces {
            line.clear();
            if reader.read_line(&mut line).unwrap_or(0) == 0 {
                break;
            }
            let nums: Vec<u32> = line
                .split_whitespace()
                .filter_map(|s| s.parse::<u32>().ok())
                .collect();
            if nums.len() >= 4 {
                let cnt = nums[0] as usize;
                for k in 1..(cnt.saturating_sub(1)) {
                    if 1 + k + 1 < nums.len() {
                        indices.push([nums[1], nums[1 + k], nums[1 + k + 1]]);
                    }
                }
            }
        }
        Ok(TriMeshData::new(positions, indices).weld_vertices(1e-4))
    }

    pub fn surface_area_mm2(&self) -> f32 {
        let mut area = 0.0_f32;
        for &[a, b, c] in &self.indices {
            let (pa, pb, pc) = (
                self.positions[a as usize],
                self.positions[b as usize],
                self.positions[c as usize],
            );
            area += 0.5 * (pb - pa).cross(pc - pa).length();
        }
        area
    }

    pub fn max_edge_length_mm(&self) -> f32 {
        let mut mx = 0.0_f32;
        for &[a, b, c] in &self.indices {
            let (pa, pb, pc) = (
                self.positions[a as usize],
                self.positions[b as usize],
                self.positions[c as usize],
            );
            mx = mx.max(pa.distance(pb)).max(pb.distance(pc)).max(pc.distance(pa));
        }
        mx
    }

    pub fn save_ply(&self, path: &Path) -> Result<(), String> {
        use std::io::Write;
        if let Some(parent) = path.parent() {
            let _ = std::fs::create_dir_all(parent);
        }
        let file = File::create(path).map_err(|e| format!("No se pudo crear PLY {:?}: {}", path, e))?;
        let mut w = std::io::BufWriter::new(file);
        writeln!(
            w,
            "ply\nformat ascii 1.0\ncomment Apex 3D Viewer / Apex DentalCraft\nelement vertex {}\nproperty float x\nproperty float y\nproperty float z\nelement face {}\nproperty list uchar int vertex_indices\nend_header",
            self.positions.len(),
            self.indices.len()
        )
        .map_err(|e| e.to_string())?;
        for p in &self.positions {
            writeln!(w, "{:.6} {:.6} {:.6}", p.x, p.y, p.z).map_err(|e| e.to_string())?;
        }
        for &[a, b, c] in &self.indices {
            writeln!(w, "3 {} {} {}", a, b, c).map_err(|e| e.to_string())?;
        }
        w.flush().map_err(|e| e.to_string())
    }
}

/// Construye una base ortonormal derecha `(u, v, a)` a partir del eje de inserción `axis`.
pub fn build_orthonormal_basis(axis: Vec3) -> (Vec3, Vec3, Vec3) {
    let a = if axis.length_squared() > 1e-10 {
        axis.normalize()
    } else {
        Vec3::Z
    };
    let ref_v = if a.y.abs() < 0.85 { Vec3::Y } else { Vec3::X };
    let u = ref_v.cross(a).normalize();
    let v = a.cross(u).normalize();
    (u, v, a)
}

/// Interpolación Hermite C1 suave entre `edge0` y `edge1`.
#[inline]
pub fn smoothstep(edge0: f32, edge1: f32, x: f32) -> f32 {
    let denom = edge1 - edge0;
    if denom.abs() < 1e-7 {
        return if x >= edge1 { 1.0 } else { 0.0 };
    }
    let t = ((x - edge0) / denom).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

/// Kernel de soporte compacto C2 de Wendland para deformación suave libre de pliegues.
#[inline]
pub fn wendland_c2(q: f32) -> f32 {
    let qc = q.clamp(0.0, 1.0);
    let o = 1.0 - qc;
    o * o * o * o * (4.0 * qc + 1.0)
}
