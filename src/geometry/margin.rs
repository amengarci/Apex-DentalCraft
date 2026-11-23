//! Módulo 3 (Núcleo Geométrico): Controlador de Línea de Margen Cervical con `parry3d`
//!
//! - Proyección superficial exacta (`MeshCollider::project_point` con BVH de `parry3d`).
//! - Imán magnético hacia la cresta de curvatura cervical / chamfer.
//! - Curva cerrada suave Catmull-Rom periódica (120 muestras adheridas al muñón).
//! - Auto-Detección de 1 clic desde la cara oclusal mediante 16 perfiles radiales por raycasting.

use crate::geometry::mesh::{build_orthonormal_basis, MeshCollider, TriMeshData};
use crate::geometry::mesh_ops::compute_mean_curvature_weights;
use glam::Vec3;
use std::f32::consts::PI;

pub struct MarginLineController {
    pub target_mesh: Option<TriMeshData>,
    pub collider: Option<MeshCollider>,
    pub curvature_weights: Vec<f32>,
    pub control_points: Vec<Vec3>,
    pub curve_points: Vec<Vec3>,
}

impl Default for MarginLineController {
    fn default() -> Self {
        Self {
            target_mesh: None,
            collider: None,
            curvature_weights: Vec::new(),
            control_points: Vec::new(),
            curve_points: Vec::new(),
        }
    }
}

impl MarginLineController {
    pub fn set_target_mesh(&mut self, mesh: TriMeshData) {
        self.curvature_weights = compute_mean_curvature_weights(&mesh);
        self.collider = MeshCollider::from_mesh(&mesh);
        self.target_mesh = Some(mesh);
    }

    pub fn project_to_surface(&self, point: Vec3) -> Vec3 {
        if let Some(col) = &self.collider {
            col.project_point(point)
        } else {
            point
        }
    }

    /// Imán magnético hacia la cresta del hombro / chamfer cervical dentro de `snap_radius_mm`.
    pub fn snap_to_cervical_ridge(&self, point: Vec3, snap_radius_mm: f32) -> Vec3 {
        let proj = self.project_to_surface(point);
        let Some(mesh) = &self.target_mesh else {
            return proj;
        };
        if self.curvature_weights.len() != mesh.positions.len() || snap_radius_mm <= 0.0 {
            return proj;
        }
        let r2 = snap_radius_mm * snap_radius_mm;
        let sigma = snap_radius_mm * 0.6;
        let mut best_score = 0.0_f32;
        let mut best_pt = proj;

        for (i, &p) in mesh.positions.iter().enumerate() {
            let d2 = p.distance_squared(proj);
            if d2 <= r2 {
                let prox = (-(d2 / (sigma * sigma))).exp();
                let score = self.curvature_weights[i] * prox;
                if score > best_score {
                    best_score = score;
                    best_pt = p;
                }
            }
        }
        if best_score < 0.15 {
            return proj;
        }
        let blended = best_pt * 0.65 + proj * 0.35;
        self.project_to_surface(blended)
    }

    pub fn load_points(&mut self, pts: &[[f32; 3]]) {
        self.control_points = pts.iter().map(|p| Vec3::from_array(*p)).collect();
        self.recompute_spline(180);
    }

    pub fn clear(&mut self) {
        self.control_points.clear();
        self.curve_points.clear();
    }

    pub fn add_control_point(
        &mut self,
        point: Vec3,
        use_magnetic_snap: bool,
        insert_in_nearest_segment: bool,
    ) -> usize {
        let pt = if use_magnetic_snap {
            self.snap_to_cervical_ridge(point, 0.55)
        } else {
            self.project_to_surface(point)
        };

        let idx = if !insert_in_nearest_segment || self.control_points.len() < 3 {
            self.control_points.push(pt);
            self.control_points.len() - 1
        } else {
            let n = self.control_points.len();
            let mut best_seg = n;
            let mut best_dist = f32::INFINITY;
            for i in 0..n {
                let a = self.control_points[i];
                let b = self.control_points[(i + 1) % n];
                let ab = b - a;
                let denom = ab.length_squared();
                let t = if denom > 1e-8 {
                    (pt - a).dot(ab) / denom
                } else {
                    0.0
                }
                .clamp(0.0, 1.0);
                let proj_seg = a + ab * t;
                let d = pt.distance(proj_seg);
                if d < best_dist {
                    best_dist = d;
                    best_seg = i + 1;
                }
            }
            self.control_points.insert(best_seg, pt);
            best_seg
        };

        self.recompute_spline(180);
        idx
    }

    pub fn move_control_point(&mut self, index: usize, new_point: Vec3, use_magnetic_snap: bool) {
        if index < self.control_points.len() {
            let pt = if use_magnetic_snap {
                self.snap_to_cervical_ridge(new_point, 0.55)
            } else {
                self.project_to_surface(new_point)
            };
            self.control_points[index] = pt;
            self.recompute_spline(180);
        }
    }

    pub fn remove_control_point(&mut self, index: Option<usize>) {
        if self.control_points.is_empty() {
            return;
        }
        match index {
            Some(i) if i < self.control_points.len() => {
                self.control_points.remove(i);
            }
            _ => {
                self.control_points.pop();
            }
        }
        self.recompute_spline(180);
    }

    pub fn find_nearest_control_point(&self, point: Vec3, max_dist_mm: f32) -> Option<usize> {
        let mut best_idx = None;
        let mut best_d = max_dist_mm;
        for (i, &p) in self.control_points.iter().enumerate() {
            let d = p.distance(point);
            if d <= best_d {
                best_d = d;
                best_idx = Some(i);
            }
        }
        best_idx
    }

    pub fn order_points_angularly(&mut self, insertion_axis: Vec3) {
        if self.control_points.len() < 3 {
            return;
        }
        let (u, v, _) = build_orthonormal_basis(insertion_axis);
        let center: Vec3 =
            self.control_points.iter().copied().sum::<Vec3>() / self.control_points.len() as f32;
        self.control_points.sort_by(|a, b| {
            let ra = *a - center;
            let rb = *b - center;
            let ang_a = ra.dot(v).atan2(ra.dot(u));
            let ang_b = rb.dot(v).atan2(rb.dot(u));
            ang_a.partial_cmp(&ang_b).unwrap_or(std::cmp::Ordering::Equal)
        });
        self.recompute_spline(180);
    }

    /// Interpola una spline cúbica periódica Catmull-Rom cerrada y proyecta cada punto al muñón.
    pub fn recompute_spline(&mut self, n_samples: usize) {
        let n = self.control_points.len();
        if n == 0 {
            self.curve_points.clear();
            return;
        }
        if n == 1 {
            self.curve_points = vec![self.control_points[0]];
            return;
        }
        if n == 2 {
            let mut pts = Vec::with_capacity(25);
            for i in 0..25 {
                let t = i as f32 / 24.0;
                pts.push(self.project_to_surface(self.control_points[0].lerp(self.control_points[1], t)));
            }
            self.curve_points = pts;
            return;
        }

        // Evaluación densa Catmull-Rom cerrada y remuestreo por longitud de arco
        let sub_per_seg = 12usize;
        let mut dense = Vec::with_capacity(n * sub_per_seg);
        for i in 0..n {
            let p0 = self.control_points[(i + n - 1) % n];
            let p1 = self.control_points[i];
            let p2 = self.control_points[(i + 1) % n];
            let p3 = self.control_points[(i + 2) % n];
            for s in 0..sub_per_seg {
                let t = s as f32 / sub_per_seg as f32;
                let t2 = t * t;
                let t3 = t2 * t;
                let pt = 0.5
                    * ((2.0 * p1)
                        + (-p0 + p2) * t
                        + (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * t2
                        + (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * t3);
                dense.push(pt);
            }
        }

        // Remuestreo uniforme por longitud de arco cerrada
        let m = dense.len();
        let mut cum = Vec::with_capacity(m + 1);
        cum.push(0.0_f32);
        for i in 0..m {
            let d = dense[i].distance(dense[(i + 1) % m]);
            cum.push(cum[i] + d);
        }
        let total = *cum.last().unwrap_or(&0.0);
        if total < 1e-5 {
            self.curve_points = self.control_points.clone();
            return;
        }

        let mut resampled = Vec::with_capacity(n_samples);
        let mut seg_idx = 0usize;
        for k in 0..n_samples {
            let target = total * (k as f32) / (n_samples as f32);
            while seg_idx + 1 < m && cum[seg_idx + 1] < target {
                seg_idx += 1;
            }
            let seg_len = (cum[seg_idx + 1] - cum[seg_idx]).max(1e-7);
            let frac = ((target - cum[seg_idx]) / seg_len).clamp(0.0, 1.0);
            let raw_pt = dense[seg_idx].lerp(dense[(seg_idx + 1) % m], frac);
            resampled.push(self.project_to_surface(raw_pt));
        }
        self.curve_points = resampled;
    }

    /// Asistente semiautomático 1-clic sobre la cima del muñón:
    /// Lanza perfiles radiales paralelos al eje de inserción con `parry3d` y detecta la inflexión del hombro.
    pub fn auto_detect_margin_from_stump_top(
        &mut self,
        seed_point: Vec3,
        insertion_axis: Vec3,
        n_control_nodes: usize,
        search_radius_mm: f32,
    ) {
        let Some(collider) = &self.collider else {
            return;
        };
        let (u, v, a) = build_orthonormal_basis(insertion_axis);
        let seed = collider.project_point(seed_point);
        let h_seed = seed.dot(a);

        let n_rad = 38usize;
        let radii: Vec<f32> = (0..n_rad)
            .map(|i| 1.6 + (search_radius_mm - 1.6) * (i as f32) / ((n_rad - 1) as f32))
            .collect();

        let mut detected = Vec::with_capacity(n_control_nodes);

        for k in 0..n_control_nodes {
            let theta = 2.0 * PI * (k as f32) / (n_control_nodes as f32);
            let radial_dir = theta.cos() * u + theta.sin() * v;

            let mut profile_pts = Vec::with_capacity(n_rad);
            let mut profile_h = Vec::with_capacity(n_rad);

            for &r in &radii {
                let ray_start = seed + radial_dir * r + a * 5.5;
                let hp = if let Some((_, hit)) = collider.cast_ray(ray_start, -a, 16.0) {
                    hit
                } else {
                    collider.project_point(seed + radial_dir * r - a * 2.0)
                };
                profile_h.push(hp.dot(a));
                profile_pts.push(hp);
            }

            // Primera y segunda derivada discreta dh/dr y d2h/dr2
            let mut dh = vec![0.0_f32; n_rad];
            let dr = (search_radius_mm - 1.6) / (n_rad - 1) as f32;
            for i in 0..n_rad {
                dh[i] = if i == 0 {
                    (profile_h[1] - profile_h[0]) / dr
                } else if i == n_rad - 1 {
                    (profile_h[n_rad - 1] - profile_h[n_rad - 2]) / dr
                } else {
                    (profile_h[i + 1] - profile_h[i - 1]) / (2.0 * dr)
                };
            }
            let mut d2h = vec![0.0_f32; n_rad];
            for i in 0..n_rad {
                d2h[i] = if i == 0 {
                    (dh[1] - dh[0]) / dr
                } else if i == n_rad - 1 {
                    (dh[n_rad - 1] - dh[n_rad - 2]) / dr
                } else {
                    (dh[i + 1] - dh[i - 1]) / (2.0 * dr)
                };
            }

            let max_abs = d2h.iter().map(|v| v.abs()).fold(1e-6_f32, f32::max);
            let mut best_i = (n_rad as f32 * 0.55) as usize;
            let mut best_score = f32::NEG_INFINITY;

            for i in 1..(n_rad - 1) {
                let drop = h_seed - profile_h[i];
                if drop > 0.9 && drop < 6.8 {
                    let curv_bonus = self.sample_curvature_near(profile_pts[i]);
                    let score = 0.65 * (d2h[i] / max_abs) + 0.35 * curv_bonus;
                    if score > best_score {
                        best_score = score;
                        best_i = i;
                    }
                }
            }
            detected.push(profile_pts[best_i]);
        }

        // Suavizado cíclico ligero de nodos vecinos
        let mut smoothed = Vec::with_capacity(n_control_nodes);
        for k in 0..n_control_nodes {
            let prev = detected[(k + n_control_nodes - 1) % n_control_nodes];
            let curr = detected[k];
            let next = detected[(k + 1) % n_control_nodes];
            let avg = prev * 0.2 + curr * 0.6 + next * 0.2;
            smoothed.push(self.project_to_surface(avg));
        }

        self.control_points = smoothed;
        self.recompute_spline(180);
    }

    fn sample_curvature_near(&self, pt: Vec3) -> f32 {
        let Some(mesh) = &self.target_mesh else {
            return 0.0;
        };
        if self.curvature_weights.len() != mesh.positions.len() {
            return 0.0;
        }
        let mut best_d2 = f32::INFINITY;
        let mut best_c = 0.0;
        for (i, &p) in mesh.positions.iter().enumerate() {
            let d2 = p.distance_squared(pt);
            if d2 < best_d2 {
                best_d2 = d2;
                best_c = self.curvature_weights[i];
            }
        }
        best_c
    }
}

/// Construye un tubo 3D suave a lo largo de una curva (cerrada o abierta) para renderizar la línea de margen.
pub fn build_tube_mesh(points: &[Vec3], radius: f32, closed: bool, sides: usize) -> TriMeshData {
    let n = points.len();
    if n < 2 {
        return TriMeshData::default();
    }
    let sides = sides.max(6);
    let mut positions = Vec::with_capacity(n * sides);
    let mut indices = Vec::with_capacity(n * sides * 2);

    for i in 0..n {
        let prev = if i == 0 {
            if closed { points[n - 1] } else { points[0] }
        } else {
            points[i - 1]
        };
        let next = if i + 1 == n {
            if closed { points[0] } else { points[n - 1] }
        } else {
            points[i + 1]
        };
        let tangent = (next - prev).normalize_or_zero();
        let (u, v, _) = build_orthonormal_basis(if tangent.length_squared() > 1e-6 { tangent } else { Vec3::Z });
        for s in 0..sides {
            let ang = 2.0 * PI * (s as f32) / (sides as f32);
            positions.push(points[i] + (ang.cos() * u + ang.sin() * v) * radius);
        }
    }

    let segs = if closed { n } else { n - 1 };
    for i in 0..segs {
        let ni = (i + 1) % n;
        for s in 0..sides {
            let ns = (s + 1) % sides;
            let a0 = (i * sides + s) as u32;
            let a1 = (i * sides + ns) as u32;
            let b0 = (ni * sides + s) as u32;
            let b1 = (ni * sides + ns) as u32;
            indices.push([a0, b0, b1]);
            indices.push([a0, b1, a1]);
        }
    }
    TriMeshData::new(positions, indices)
}

/// Construye una esfera 3D ligera para nodos de margen o cursores de pincel/Morphing.
pub fn build_uv_sphere(center: Vec3, radius: f32, stacks: usize, slices: usize) -> TriMeshData {
    let mut positions = Vec::with_capacity(2 + (stacks - 1) * slices);
    positions.push(center + Vec3::new(0.0, 0.0, radius));
    for i in 1..stacks {
        let phi = PI * (i as f32) / (stacks as f32);
        for j in 0..slices {
            let th = 2.0 * PI * (j as f32) / (slices as f32);
            positions.push(
                center
                    + Vec3::new(
                        radius * phi.sin() * th.cos(),
                        radius * phi.sin() * th.sin(),
                        radius * phi.cos(),
                    ),
            );
        }
    }
    let south = positions.len() as u32;
    positions.push(center + Vec3::new(0.0, 0.0, -radius));

    let ring = |ri: usize, j: usize| -> u32 { (1 + (ri - 1) * slices + (j % slices)) as u32 };
    let mut indices = Vec::new();
    for j in 0..slices {
        indices.push([0, ring(1, j), ring(1, j + 1)]);
    }
    for ri in 1..(stacks - 1) {
        for j in 0..slices {
            let a0 = ring(ri, j);
            let a1 = ring(ri, j + 1);
            let b0 = ring(ri + 1, j);
            let b1 = ring(ri + 1, j + 1);
            indices.push([a0, b0, b1]);
            indices.push([a0, b1, a1]);
        }
    }
    for j in 0..slices {
        indices.push([south, ring(stacks - 1, j + 1), ring(stacks - 1, j)]);
    }
    TriMeshData::new(positions, indices)
}
