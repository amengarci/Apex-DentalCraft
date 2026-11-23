//! Punto de Entrada — **Apex DentalCraft** (100% Nativo en Rust, Offline)
//!
//! Suite CAD dental local inspirada en el rendimiento y estética de la suite *Craft,
//! manteniendo la identidad visual de Apex Dental Lab (#190321, #F2EAD3, #9D4EDD, #34D399).

#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

use apex_dentalcraft_lib::ui::app::ApexDentalCraftApp;

fn main() -> eframe::Result {
    let mut viewport = egui::ViewportBuilder::default()
        .with_title("APEX DENTALCRAFT")
        .with_inner_size([1500.0, 910.0])
        .with_min_inner_size([1120.0, 700.0]);

    if let Ok(icon) = eframe::icon_data::from_png_bytes(include_bytes!("../assets/apex_dentalcraft.png")) {
        viewport = viewport.with_icon(std::sync::Arc::new(icon));
    }

    let options = eframe::NativeOptions {
        renderer: eframe::Renderer::Glow,
        depth_buffer: 24,
        multisampling: 4,
        viewport,
        ..Default::default()
    };

    eframe::run_native(
        "Apex DentalCraft",
        options,
        Box::new(|cc| Ok(Box::new(ApexDentalCraftApp::new(cc)))),
    )
}

#[cfg(test)]
mod tests {
    use apex_dentalcraft_lib::core::case_manager::{DesignParameters, APEX_MATERIAL_PRESETS};
    use apex_dentalcraft_lib::core::library_manager::ToothLibraryManager;
    use apex_dentalcraft_lib::geometry::crown::CrownDesigner;
    use apex_dentalcraft_lib::geometry::gizmo::{PlacementOverlay, TransformSubmode};
    use apex_dentalcraft_lib::geometry::margin::MarginLineController;
    use apex_dentalcraft_lib::geometry::mesh_ops::{clean_mesh, create_solid_base, generate_parametric_demo_arch};
    use apex_dentalcraft_lib::geometry::sculpt::{CrownSculptor, SculptMode};
    use apex_dentalcraft_lib::geometry::smart_tools::SmartToolsEngine;
    use apex_dentalcraft_lib::render::camera::OrbitCamera;
    use apex_dentalcraft_lib::ui::odontogram::FdiOdontogramState;
    use glam::Vec3;
    use std::collections::BTreeMap;

    #[test]
    fn test_odontogram_click_toggle_select_deselect() {
        let mut od = FdiOdontogramState::default();
        assert_eq!(od.get_selected_vec(), vec![36]);
        // Volver a hacer clic en #36 deselecciona -> vacío
        od.toggle_tooth_click(36, false);
        assert!(od.get_selected_vec().is_empty());
        assert_eq!(od.selected_tooth, None);
        // Seleccionar #11 y #21 con clics simples, luego deseleccionar #11
        od.toggle_tooth_click(11, false);
        od.toggle_tooth_click(21, false);
        assert_eq!(od.get_selected_vec(), vec![11, 21]);
        od.toggle_tooth_click(11, false);
        assert_eq!(od.get_selected_vec(), vec![21]);
    }

    #[test]
    fn test_material_presets_and_full_cad_pipeline_watertight() {
        assert_eq!(APEX_MATERIAL_PRESETS.len(), 7);
        let mut params = DesignParameters::default();
        params.apply_material_and_process_preset("e.max", "Corona Individual");
        assert!((params.min_thickness_mm - 1.00).abs() < 1e-4);
        assert!((params.cement_gap_mm - 0.060).abs() < 1e-4);

        // 1. Arcadas demo + limpieza Taubin + Zócalo
        let mand_raw = generate_parametric_demo_arch(false);
        let max_ant = generate_parametric_demo_arch(true);
        let mand = clean_mesh(&mand_raw, true, 2);
        let with_base = create_solid_base(&mand, 4.0, true);
        assert!(with_base.positions.len() > mand.positions.len());

        // 2. Auto-detección de margen cervical con parry3d
        let mut mctrl = MarginLineController::default();
        mctrl.set_target_mesh(mand.clone());
        let seed = mand
            .positions
            .iter()
            .filter(|p| p.x.abs() < 4.5 && p.y.abs() < 4.5)
            .max_by(|a, b| a.z.partial_cmp(&b.z).unwrap())
            .copied()
            .unwrap_or(Vec3::ZERO);
        mctrl.auto_detect_margin_from_stump_top(seed, Vec3::Z, 16, 6.2);
        assert_eq!(mctrl.control_points.len(), 16);
        assert_eq!(mctrl.curve_points.len(), 180);

        // 3. Diente de librería procedural + colocación aproximada + Transformación Individual
        let lib_tooth = ToothLibraryManager::generate_procedural_tooth_crown(36);
        let approx = CrownDesigner::place_library_tooth_world(
            &lib_tooth,
            &mctrl.curve_points,
            Vec3::Z,
            &params,
            Some(&mand),
            Some(&max_ant),
        );
        let mut cam = OrbitCamera::default();
        let (mn, mx) = approx.bounds();
        cam.frame_bounds(mn, mx);
        let vp = egui::Rect::from_min_size(egui::pos2(0.0, 0.0), egui::vec2(1280.0, 720.0));
        let t_rect = PlacementOverlay::compute_tooth_screen_rect(&approx, &cam, vp)
            .expect("El diente debe proyectarse dentro del viewport");
        let overlay = PlacementOverlay::build(t_rect, vp, TransformSubmode::All);
        assert_eq!(overlay.controls.len(), 9); // 1 cruz central + 4 flechas verdes escala + 4 flechas curvas azules rotar

        // 4. Generación de corona isotrópica 100% watertight (valencia central 8)
        let mut crown = CrownDesigner::generate_crown(
            &mand,
            &mctrl.curve_points,
            &lib_tooth,
            Vec3::Z,
            &params,
            Some(&max_ant),
        )
        .expect("generate_crown debe tener éxito");
        let audit0 = crown.inspect_cam();
        assert!(audit0.watertight, "Corona debe ser 100% watertight (open_edges={})", audit0.open_edges);
        let v0_valence = crown.indices.iter().filter(|t| t.contains(&0)).count();
        assert_eq!(v0_valence, 8, "El centro oclusal debe tener valencia isotrópica 8");

        let locked_orig: Vec<Vec3> = crown
            .positions
            .iter()
            .enumerate()
            .filter_map(|(i, &p)| if !crown.is_external[i] { Some(p) } else { None })
            .collect();

        // 5. 18 pasadas de esculpido libre (Aumentar / Quitar / Suavizar) sin conflictos triangulares
        let center_pt = crown.positions[0];
        let top_pt = crown
            .positions
            .iter()
            .enumerate()
            .filter(|(i, _)| crown.is_external[*i])
            .max_by(|a, b| a.1.z.partial_cmp(&b.1.z).unwrap())
            .map(|(_, &p)| p)
            .unwrap();
        let adj = crown.build_adjacency();
        for (mode, target) in [
            (SculptMode::Add, center_pt),
            (SculptMode::Add, top_pt),
            (SculptMode::Remove, top_pt),
            (SculptMode::Smooth, center_pt),
            (SculptMode::Smooth, top_pt),
        ] {
            for _ in 0..4 {
                CrownSculptor::apply_brush_stroke(&mut crown, target, mode, 1.65, 0.12, 0.60, Some(&adj));
            }
        }
        assert!(crown.inspect_cam().watertight, "Malla debe seguir siendo watertight tras esculpir");

        // 6. Morphing 3D interactivo estilo 3Shape
        let sess = CrownSculptor::prepare_morph_session(&crown, top_pt, 2.5);
        let morphed = CrownSculptor::apply_morph_drag(&sess, Vec3::new(0.35, -0.25, 0.55), 0.60, false);
        assert!(morphed.inspect_cam().watertight, "Morphing 3D debe mantener watertight");

        // 7. Las 5 Herramientas Inteligentes (Smart Tools)
        let s1 = SmartToolsEngine::smart_occlusal_contacts(&morphed, &max_ant, Vec3::Z, 0.0, true, 0.60);
        let s2 = SmartToolsEngine::smart_proximal_contacts(
            &s1,
            Some(&mand),
            &mctrl.curve_points,
            Vec3::Z,
            &BTreeMap::new(),
            -0.02,
            0.60,
        );
        let s3 = SmartToolsEngine::smart_enforce_min_thickness(&s2, 0.60);
        let s4 = SmartToolsEngine::smart_anatomical_smooth(&s3, 4, 0.60);
        let s5 = SmartToolsEngine::smart_emergence_profile(&s4, &mctrl.curve_points, Vec3::Z, 0.12, 0.60);

        let final_audit = s5.inspect_cam();
        assert!(final_audit.watertight, "Tras todas las Smart Tools debe seguir 100% watertight");
        let locked_final: Vec<Vec3> = s5
            .positions
            .iter()
            .enumerate()
            .filter_map(|(i, &p)| if !s5.is_external[i] { Some(p) } else { None })
            .collect();
        for (a, b) in locked_orig.iter().zip(locked_final.iter()) {
            assert!(a.distance(*b) < 1e-5, "El sellado del margen y el intaglio deben permanecer intactos");
        }
    }

    #[test]
    fn test_portable_case_roundtrip_relative_paths() {
        use apex_dentalcraft_lib::core::case_manager::CaseManager;
        let tmp_usb = std::env::temp_dir().join("apex_dentalcraft_usb_test_casos");
        let _ = std::fs::remove_dir_all(&tmp_usb);
        let cm = CaseManager::new(Some(tmp_usb.clone()));
        let case = cm
            .create_case(
                "Paciente USB",
                "Dr. Prueba",
                36,
                "Zirconia Monolítica (3Y-TZP)",
                "A2",
                "Prueba portable",
                BTreeMap::new(),
            )
            .expect("create_case portable debe funcionar");
        let reloaded = cm.load_case(&case.case_id).expect("load_case portable debe funcionar");
        assert_eq!(reloaded.patient_name, "Paciente USB");
        assert_eq!(reloaded.tooth_number, 36);
        assert!(cm.get_case_dir(&case.case_id).join("case_metadata.json").exists());
        let _ = std::fs::remove_dir_all(&tmp_usb);
    }

    #[test]
    fn test_360_quaternion_camera_full_flip_and_roll() {
        use apex_dentalcraft_lib::render::camera::OrbitCamera;
        let mut cam = OrbitCamera::default();
        cam.set_view_frontal();
        let (_, up0) = cam.right_and_up();
        assert!(up0.z > 0.9, "Vista frontal inicia con +Z arriba");

        // Voltear 180 grados verticales completos sin bloqueo de polo ni gimbal lock
        let step_px = (std::f32::consts::PI / 0.0075) / 60.0;
        for _ in 0..60 {
            cam.orbit(egui::vec2(0.0, step_px));
        }
        let (_, up180) = cam.right_and_up();
        assert!(
            up180.z < -0.9,
            "La cámara debe voltear completamente boca abajo (up.z={})",
            up180.z
        );

        // Girar otros 180 grados (360° total) + roll + pan
        for _ in 0..60 {
            cam.orbit(egui::vec2(0.0, step_px));
        }
        cam.roll(120.0);
        let rect = egui::Rect::from_min_size(egui::pos2(0.0, 0.0), egui::vec2(1280.0, 720.0));
        cam.pan(egui::vec2(45.0, -30.0), rect);
        let vp = cam.view_proj(rect);
        assert!(vp.is_finite(), "Matriz view_proj debe permanecer 100% finita en cualquier rotación 360°");
    }

    #[test]
    fn test_step4_placement_preserved_across_tools_and_margin_adapt() {
        let mand = clean_mesh(&generate_parametric_demo_arch(false), true, 1);
        let max_ant = generate_parametric_demo_arch(true);
        let mut mctrl = MarginLineController::default();
        mctrl.set_target_mesh(mand.clone());
        let seed = mand
            .positions
            .iter()
            .filter(|p| p.x.abs() < 4.5 && p.y.abs() < 4.5)
            .max_by(|a, b| a.z.partial_cmp(&b.z).unwrap())
            .copied()
            .unwrap_or(Vec3::ZERO);
        mctrl.auto_detect_margin_from_stump_top(seed, Vec3::Z, 16, 6.2);

        let params = DesignParameters::default();
        let lib_tooth = ToothLibraryManager::generate_procedural_tooth_crown(36);
        let placed = CrownDesigner::place_library_tooth_world(
            &lib_tooth,
            &mctrl.curve_points,
            Vec3::Z,
            &params,
            Some(&mand),
            Some(&max_ant),
        );
        let c0 = placed.center();

        // 1. Mover y rotar con Transformación Individual estilo 3Shape (sin reset)
        let moved = CrownSculptor::transform_free_tooth_mesh(
            &placed,
            Vec3::Z,
            Vec3::new(1.35, -0.85, 0.60),
            Vec3::new(0.0, 0.0, 14.0),
            None,
            Vec3::ONE,
            None,
        );
        let c1 = moved.center();
        assert!(
            (c1.x - (c0.x + 1.35)).abs() < 0.05 && (c1.y - (c0.y - 0.85)).abs() < 0.05,
            "La traslación libre debe mover el diente exactamente al desplazamiento solicitado"
        );

        // 2. Pasar a Morphing y Aumentar/Suavizar: el centro NO debe devolverse al origen
        let top_pt = moved
            .positions
            .iter()
            .max_by(|a, b| a.z.partial_cmp(&b.z).unwrap())
            .copied()
            .unwrap();
        let sess = CrownSculptor::prepare_morph_session(&moved, top_pt, 2.2);
        let mut sculpted = CrownSculptor::apply_morph_drag(&sess, Vec3::new(0.0, 0.0, 0.35), 0.60, false);
        CrownSculptor::apply_brush_stroke(&mut sculpted, top_pt, SculptMode::Add, 1.5, 0.08, 0.60, None);
        let c2 = sculpted.center();
        assert!(
            (c2.x - c1.x).abs() < 0.25 && (c2.y - c1.y).abs() < 0.25,
            "Cambiar a Morphing/Aumentar NO debe resetear la posición del diente"
        );

        // 3. Al pulsar 'Adaptar al Margen', el sello debe respetar la posición desplazada
        let (adapted, audit_ad) = CrownDesigner::generate_crown_from_positioned_tooth(
            &mand,
            &mctrl.curve_points,
            &sculpted,
            Vec3::Z,
            &params,
            Some(&max_ant),
        )
        .expect("generate_crown_from_positioned_tooth debe tener éxito");
        assert!(audit_ad.valid_for_production, "La auditoría de cementado debe ser válida");
        assert!(adapted.inspect_cam().watertight, "Corona adaptada debe ser 100% watertight");
        let occ_pole = adapted.positions[0];
        let margin_origin = mctrl.curve_points.iter().copied().sum::<Vec3>() / mctrl.curve_points.len() as f32;
        assert!(
            (occ_pole.x - margin_origin.x) > 0.7 && (occ_pole.y - margin_origin.y) < -0.4,
            "La corona adaptada debe conservar la posición desplazada del usuario (occ_pole={:?}, margin_origin={:?})",
            occ_pole,
            margin_origin
        );
    }

    #[test]
    fn test_cement_spacer_coverage_and_3shape_drill_compensation() {
        use apex_dentalcraft_lib::geometry::mesh::TriMeshData;
        // Construir un muñón cilíndrico con arista axio-oclusal aguda de 90° (peor caso para fresa)
        let n_seg = 64usize;
        let r_die = 3.2_f32;
        let h_die = 4.2_f32;
        let mut verts = Vec::new();
        let mut faces = Vec::new();
        verts.push(Vec3::new(0.0, 0.0, h_die)); // 0: centro techo plano
        for k in 0..n_seg {
            let th = 2.0 * std::f32::consts::PI * k as f32 / n_seg as f32;
            verts.push(Vec3::new(r_die * th.cos(), r_die * th.sin(), h_die)); // 1..=n_seg: borde 90°
        }
        for k in 0..n_seg {
            let th = 2.0 * std::f32::consts::PI * k as f32 / n_seg as f32;
            verts.push(Vec3::new((r_die + 0.35) * th.cos(), (r_die + 0.35) * th.sin(), 0.0)); // cuello
        }
        verts.push(Vec3::new(0.0, 0.0, 0.0)); // base
        let base_c = (1 + 2 * n_seg) as u32;
        for k in 0..n_seg {
            let nk = (k + 1) % n_seg;
            let t0 = (1 + k) as u32;
            let t1 = (1 + nk) as u32;
            let b0 = (1 + n_seg + k) as u32;
            let b1 = (1 + n_seg + nk) as u32;
            faces.push([0, t0, t1]);
            faces.push([t0, b0, b1]);
            faces.push([t0, b1, t1]);
            faces.push([base_c, b1, b0]);
        }
        let sharp_die = TriMeshData::new(verts, faces);
        let margin: Vec<Vec3> = (0..64)
            .map(|k| {
                let th = 2.0 * std::f32::consts::PI * k as f32 / 64.0;
                Vec3::new((r_die + 0.35) * th.cos(), (r_die + 0.35) * th.sin(), 0.0)
            })
            .collect();

        let mut p_no_drill = DesignParameters::default();
        p_no_drill.drill_compensation_enabled = false;
        let mut p_with_drill = p_no_drill.clone();
        p_with_drill.drill_compensation_enabled = true;
        p_with_drill.drill_radius_mm = 0.60;
        p_with_drill.drill_compensation_offset_mm = 0.06;

        let (cem_no, _, aud_no) = CrownDesigner::generate_auxiliary_shells(&sharp_die, &margin, Vec3::Z, &p_no_drill)
            .expect("aux shells sin fresa");
        let (cem_yes, _, aud_yes) = CrownDesigner::generate_auxiliary_shells(&sharp_die, &margin, Vec3::Z, &p_with_drill)
            .expect("aux shells con fresa");
        assert!(aud_no.valid_for_production && aud_yes.valid_for_production);
        assert_eq!(aud_yes.exposed_die_points, 0, "Cero puntos expuestos sobre el muñón cilíndrico 90°");

        // 1. Cobertura completa: todo el cemento en el techo debe superar h_die + cement_gap_mm
        let top_no = cem_no.positions[0].z;
        assert!(
            top_no >= h_die + p_no_drill.cement_gap_mm - 0.005,
            "El espacio de cementado debe cubrir completamente el techo del muñón ({} vs {})",
            top_no,
            h_die + p_no_drill.cement_gap_mm
        );

        // 2. Compensación de Fresado debe crear alivio positivo medible en la arista aguda de 90°
        let mut max_drill_relief = 0.0_f32;
        for (a, b) in cem_no.positions.iter().zip(cem_yes.positions.iter()) {
            max_drill_relief = max_drill_relief.max(b.z - a.z);
        }
        assert!(
            max_drill_relief > 0.025,
            "La compensación de fresado estilo 3Shape debe generar alivio positivo sobre aristas vivas (max_relief={:.4} mm)",
            max_drill_relief
        );

        // 3. La corona generada con compensación de fresado debe ser 100% watertight
        let lib_tooth = ToothLibraryManager::generate_procedural_tooth_crown(36);
        let crown = CrownDesigner::generate_crown(&sharp_die, &margin, &lib_tooth, Vec3::Z, &p_with_drill, None)
            .expect("generate_crown con compensación de fresado");
        assert!(crown.inspect_cam().watertight, "Corona con compensación de fresado debe ser watertight");
    }

    #[test]
    fn test_strong_smooth_brush_and_stage4_to_stage5_persistence() {
        use apex_dentalcraft_lib::core::case_manager::CaseManager;
        use apex_dentalcraft_lib::ui::design_panel::DesignPanelState;

        // 1. Verificar que el pincel Suavizar a intensidad alta alisa visiblemente una protuberancia
        let mand = clean_mesh(&generate_parametric_demo_arch(false), true, 1);
        let mut mctrl = MarginLineController::default();
        mctrl.set_target_mesh(mand.clone());
        let seed = mand
            .positions
            .iter()
            .filter(|p| p.x.abs() < 4.5 && p.y.abs() < 4.5)
            .max_by(|a, b| a.z.partial_cmp(&b.z).unwrap())
            .copied()
            .unwrap_or(Vec3::ZERO);
        mctrl.auto_detect_margin_from_stump_top(seed, Vec3::Z, 16, 6.2);

        let params = DesignParameters::default();
        let lib_tooth = ToothLibraryManager::generate_procedural_tooth_crown(36);
        let mut crown = CrownDesigner::generate_crown(&mand, &mctrl.curve_points, &lib_tooth, Vec3::Z, &params, None)
            .expect("generate_crown");
        let bump_idx = 0usize; // polo oclusal externo
        let z_orig = crown.positions[bump_idx].z;
        crown.positions[bump_idx].z += 0.65; // pico agudo artificial
        crown.compute_normals();
        let spike_pt = crown.positions[bump_idx];
        let adj = crown.build_adjacency();
        CrownSculptor::apply_brush_stroke(&mut crown, spike_pt, SculptMode::Smooth, 1.8, 0.45, 0.60, Some(&adj));
        let drop = (z_orig + 0.65) - crown.positions[bump_idx].z;
        assert!(
            drop > 0.30,
            "Suavizar a intensidad alta debe rebajar claramente una protuberancia aguda (drop={:.3} mm)",
            drop
        );
        assert!(crown.inspect_cam().watertight, "Debe seguir watertight tras suavizado fuerte");

        // 2. Verificar escala unilateral en plano de cámara (flechas fijas 3Shape)
        let (cr, cu) = OrbitCamera::default().right_and_up();
        let cf = OrbitCamera::default().forward();
        let stretched = CrownSculptor::transform_in_view_frame(
            &crown,
            Vec3::Z,
            false,
            cr,
            cu,
            cf,
            Vec3::ZERO,
            None,
            Vec3::new(1.25, 1.0, 1.0),
            Some(Vec3::X),
            0.60,
        );
        let (mn0, mx0) = crown.bounds();
        let (mn1, mx1) = stretched.bounds();
        assert!((mx1 - mn1).length() > (mx0 - mn0).length() + 0.3, "Escala en plano de vista debe estirar la corona");

        // 3. Verificar continuidad Etapa 4 -> Etapa 5 -> Etapa 4 (cero reinicio) + recarga desde disco
        let tmp = std::env::temp_dir().join("apex_dentalcraft_stage45_test");
        let _ = std::fs::remove_dir_all(&tmp);
        let cm = CaseManager::new(Some(tmp.clone()));
        let lm = ToothLibraryManager::new();
        let mut case = cm
            .create_case("Test 4-5", "Dr.", 36, "Zirconia Monolítica (3Y-TZP)", "A2", "", BTreeMap::new())
            .expect("create_case");
        case.margin_points = mctrl.curve_points.iter().map(|p| p.to_array()).collect();
        let _ = cm.save_case(&mut case);

        let mut dp = DesignPanelState::default();
        dp.prep_mesh = Some((mand.clone(), 1));
        dp.loaded_case_id = Some(case.case_id.clone());
        dp.loaded_tooth_fdi = Some(case.tooth_number);
        dp.is_adapted_to_margin = true;
        dp.crown_mesh = Some((crown.clone(), 10));
        dp.save_active_crown(&mut case, &cm);
        let saved_pt0 = dp.crown_mesh.as_ref().unwrap().0.positions[0];

        // Volver a entrar a Etapa 4 en la misma sesión: NO debe reiniciarse ni cambiar el vértice 0
        dp.load_case(&mut case, &cm, &lm);
        assert_eq!(dp.crown_mesh.as_ref().unwrap().0.positions[0], saved_pt0);
        assert!(dp.is_adapted_to_margin);

        // Incluso al reiniciar la app (DesignPanelState nuevo), debe recargar la corona guardada en disco
        let mut dp_reopened = DesignPanelState::default();
        dp_reopened.load_case(&mut case, &cm, &lm);
        assert!(dp_reopened.is_adapted_to_margin, "Al reabrir el caso debe restaurar la corona ya diseñada");
        assert!(
            dp_reopened.crown_mesh.as_ref().unwrap().0.positions[0].distance(saved_pt0) < 1e-3,
            "La corona restaurada desde disco debe conservar exactamente los vértices guardados"
        );
        let _ = std::fs::remove_dir_all(&tmp);
    }

    #[test]
    fn test_smooth_over_margin_no_spikes_or_protrusions() {
        let mand = clean_mesh(&generate_parametric_demo_arch(false), true, 1);
        let mut mctrl = MarginLineController::default();
        mctrl.set_target_mesh(mand.clone());
        let seed = mand
            .positions
            .iter()
            .filter(|p| p.x.abs() < 4.5 && p.y.abs() < 4.5)
            .max_by(|a, b| a.z.partial_cmp(&b.z).unwrap())
            .copied()
            .unwrap_or(Vec3::ZERO);
        mctrl.auto_detect_margin_from_stump_top(seed, Vec3::Z, 16, 6.2);

        let params = DesignParameters::default();
        let lib_tooth = ToothLibraryManager::generate_procedural_tooth_crown(36);
        let mut crown = CrownDesigner::generate_crown(&mand, &mctrl.curve_points, &lib_tooth, Vec3::Z, &params, None)
            .expect("generate_crown");
        let orig_positions = crown.positions.clone();
        let (orig_mn, orig_mx) = crown.bounds();
        let adj = crown.build_adjacency();

        // Pasar el pincel Suavizar a MÁXIMA intensidad (0.50 mm) repetidamente sobre TODO el margen cervical
        for pass in 0..3 {
            for idx in (0..mctrl.curve_points.len()).step_by(8) {
                let pt = mctrl.curve_points[idx] + Vec3::new(0.0, 0.0, 0.25 * (pass as f32 % 2.0));
                CrownSculptor::apply_brush_stroke(
                    &mut crown,
                    pt,
                    SculptMode::Smooth,
                    2.2,
                    0.50,
                    params.min_thickness_mm,
                    Some(&adj),
                );
            }
        }

        // 1. Debe seguir 100% watertight 2-manifold
        assert!(crown.inspect_cam().watertight, "Suavizar sobre el margen debe mantener 100% watertight");

        // 2. El anillo marginal y el intaglio deben permanecer 100% bloqueados (0.000 mm)
        for (i, (&p0, &p1)) in orig_positions.iter().zip(crown.positions.iter()).enumerate() {
            if !crown.is_external[i] {
                assert!(
                    p0.distance(p1) < 1e-5,
                    "Margen e intaglio jamás deben moverse al pasar Suavizar (v={})",
                    i
                );
            }
        }

        // 3. CERO picos, filos o salidas hacia afuera (el bounding box NO debe explotar)
        let (new_mn, new_mx) = crown.bounds();
        let max_outward_growth = (orig_mn - new_mn).max(Vec3::ZERO).max_element()
            .max((new_mx - orig_mx).max(Vec3::ZERO).max_element());
        assert!(
            max_outward_growth < 0.05,
            "Suavizar sobre el margen NO debe crear filos ni salidas hacia afuera (crecimiento={:.4} mm)",
            max_outward_growth
        );

        // 4. Ningún vértice externo debe saltar más de 0.45 mm respecto a sus vecinos 1-anillo
        let mut max_spike = 0.0_f32;
        for i in 0..crown.positions.len() {
            if crown.is_external[i] && !adj[i].is_empty() {
                let avg: Vec3 = adj[i].iter().map(|&nb| crown.positions[nb]).sum::<Vec3>() / adj[i].len() as f32;
                max_spike = max_spike.max(crown.positions[i].distance(avg));
            }
        }
        assert!(
            max_spike < 0.45,
            "No debe haber picos locales agudos tras suavizar el margen (max_spike={:.4} mm)",
            max_spike
        );
    }

    #[test]
    fn test_asymmetric_sloped_die_zero_bare_spots_and_margin_blocker() {
        use apex_dentalcraft_lib::geometry::mesh::TriMeshData;
        // 1. Reproducir exactamente el escenario de la Imagen 1:
        //    Muñón molar asimétrico e inclinado (pared izquierda alta z=4.6 mm casi vertical hasta el
        //    94% del margen; pared derecha baja z=2.1 mm).
        let n_seg = 72usize;
        let r_margin = 3.9_f32;
        let mut verts = Vec::new();
        let mut faces = Vec::new();
        verts.push(Vec3::new(-0.5, 0.0, 3.8)); // polo techo inclinado
        for k in 0..n_seg {
            let th = 2.0 * std::f32::consts::PI * k as f32 / n_seg as f32;
            let cx = th.cos();
            let sy = th.sin();
            // Lado izquierdo (cx < 0) casi vertical hasta r=3.65 mm y altura 4.65 mm!
            let r_top = if cx < 0.0 { 3.65 } else { 3.15 };
            let z_top = 3.35 - 1.30 * cx;
            verts.push(Vec3::new(r_top * cx, r_top * sy, z_top));
        }
        for k in 0..n_seg {
            let th = 2.0 * std::f32::consts::PI * k as f32 / n_seg as f32;
            verts.push(Vec3::new(r_margin * th.cos(), r_margin * th.sin(), 0.0));
        }
        verts.push(Vec3::new(0.0, 0.0, 0.0));
        let base_c = (1 + 2 * n_seg) as u32;
        for k in 0..n_seg {
            let nk = (k + 1) % n_seg;
            let t0 = (1 + k) as u32;
            let t1 = (1 + nk) as u32;
            let b0 = (1 + n_seg + k) as u32;
            let b1 = (1 + n_seg + nk) as u32;
            faces.push([0, t0, t1]);
            faces.push([t0, b0, b1]);
            faces.push([t0, b1, t1]);
            faces.push([base_c, b1, b0]);
        }
        let sloped_die = TriMeshData::new(verts, faces);
        let margin: Vec<Vec3> = (0..72)
            .map(|k| {
                let th = 2.0 * std::f32::consts::PI * k as f32 / 72.0;
                Vec3::new(r_margin * th.cos(), r_margin * th.sin(), 0.0)
            })
            .collect();

        let params = DesignParameters::default();
        let (_cem, _min_t, audit) = CrownDesigner::generate_auxiliary_shells(
            &sloped_die,
            &margin,
            Vec3::Z,
            &params,
        )
        .expect("generate_auxiliary_shells sobre muñón inclinado");

        assert!(
            audit.valid_for_production,
            "El espacio de cementado debe resolver y cubrir el muñón inclinado: {}",
            audit.warning_message
        );
        assert_eq!(
            audit.exposed_die_points, 0,
            "CERO esquinas del muñón inclinado deben quedar descubiertas (exposed={}, pen={:.3} mm)",
            audit.exposed_die_points, audit.max_penetration_mm
        );

        // 2. Adaptar corona sobre este muñón asimétrico -> watertight y sin picos agudos en oclusal
        let lib_tooth = ToothLibraryManager::generate_procedural_tooth_crown(36);
        let placed = CrownDesigner::place_library_tooth_world(
            &lib_tooth,
            &margin,
            Vec3::Z,
            &params,
            Some(&sloped_die),
            None,
        );
        let (crown, crown_audit) = CrownDesigner::generate_crown_from_positioned_tooth(
            &sloped_die,
            &margin,
            &placed,
            Vec3::Z,
            &params,
            None,
        )
        .expect("generate_crown_from_positioned_tooth sobre muñón inclinado");
        assert!(crown_audit.valid_for_production);
        assert!(crown.inspect_cam().watertight, "Corona sobre muñón asimétrico debe ser 100% watertight");

        // Verificar suavidad oclusal (sin geometría de picos agudos en los vértices oclusales)
        let adj = crown.build_adjacency();
        let mut max_occ_spike = 0.0_f32;
        for i in 0..crown.positions.len() {
            if crown.is_external[i] && crown.positions[i].z > 4.2 && !adj[i].is_empty() {
                let avg: Vec3 = adj[i].iter().map(|&nb| crown.positions[nb]).sum::<Vec3>() / adj[i].len() as f32;
                max_occ_spike = max_occ_spike.max(crown.positions[i].distance(avg));
            }
        }
        assert!(
            max_occ_spike < 0.25,
            "La anatomía oclusal adaptada no debe presentar picos agudos (max_occ_spike={:.4} mm)",
            max_occ_spike
        );

        // 3. Último recurso: una curva de margen cruzada en forma de 8 DEBE bloquear el avance
        let crossed_margin: Vec<Vec3> = (0..48)
            .map(|k| {
                let t = 2.0 * std::f32::consts::PI * k as f32 / 48.0;
                Vec3::new(3.5 * t.sin(), 2.8 * (2.0 * t).sin(), 0.0)
            })
            .collect();
        let (_, _, bad_audit) = CrownDesigner::generate_auxiliary_shells(
            &sloped_die,
            &crossed_margin,
            Vec3::Z,
            &params,
        )
        .expect("auditoría de margen cruzado");
        assert!(bad_audit.margin_self_intersecting, "Debe detectar curva de margen auto-intersecante");
        assert!(
            !bad_audit.valid_for_production,
            "Debe bloquear producción si la línea de margen está cruzada"
        );
    }

    #[test]
    fn test_adjacent_teeth_never_explode_cement_spacer() {
        use apex_dentalcraft_lib::geometry::margin::build_uv_sphere;
        use apex_dentalcraft_lib::geometry::mesh::TriMeshData;

        // Reproducir exactamente el escenario de las Imágenes 4 y 5:
        // Muñón preparado (r=3.8 mm, altura 4.2 mm) flanqueado a milímetros por un premolar
        // izquierdo alto (z=7.2 mm) y un molar derecho ancho con corona en voladizo (z=8.2 mm)
        // que sobresalen sobre la huella XY del margen.
        let n_seg = 64usize;
        let r_margin = 3.8_f32;
        let mut verts = Vec::new();
        let mut faces = Vec::new();
        verts.push(Vec3::new(0.0, 0.0, 4.2));
        for k in 0..n_seg {
            let th = 2.0 * std::f32::consts::PI * k as f32 / n_seg as f32;
            verts.push(Vec3::new(3.25 * th.cos(), 3.25 * th.sin(), 4.1 - 0.55 * th.cos()));
        }
        for k in 0..n_seg {
            let th = 2.0 * std::f32::consts::PI * k as f32 / n_seg as f32;
            verts.push(Vec3::new(r_margin * th.cos(), r_margin * th.sin(), 0.0));
        }
        verts.push(Vec3::new(0.0, 0.0, 0.0));
        let base_c = (1 + 2 * n_seg) as u32;
        for k in 0..n_seg {
            let nk = (k + 1) % n_seg;
            let t0 = (1 + k) as u32;
            let t1 = (1 + nk) as u32;
            let b0 = (1 + n_seg + k) as u32;
            let b1 = (1 + n_seg + nk) as u32;
            faces.push([0, t0, t1]);
            faces.push([t0, b0, b1]);
            faces.push([t0, b1, t1]);
            faces.push([base_c, b1, b0]);
        }
        let mut arch = TriMeshData::new(verts, faces);

        // Añadir molar vecino derecho (sobresaliendo hasta x=+3.35 mm en z=6.2..8.2 mm)
        let mut right_molar = build_uv_sphere(Vec3::new(6.8, 0.0, 5.2), 3.45, 16, 20);
        for p in &mut right_molar.positions {
            p.z = (p.z - 5.2) * 0.9 + 5.2;
        }
        arch.merge_in_place(&right_molar);
        // Añadir premolar vecino izquierdo (rozando x=-3.55 mm en z=5.5..7.5 mm)
        let left_premolar = build_uv_sphere(Vec3::new(-6.4, 0.0, 4.8), 2.85, 16, 20);
        arch.merge_in_place(&left_premolar);

        let margin: Vec<Vec3> = (0..64)
            .map(|k| {
                let th = 2.0 * std::f32::consts::PI * k as f32 / 64.0;
                Vec3::new(r_margin * th.cos(), r_margin * th.sin(), 0.0)
            })
            .collect();

        let params = DesignParameters::default();
        // Verificar también que la auditoría de Etapa 3 NO pinte nodos en rojo por los dientes vecinos en voladizo!
        let ctrl_16: Vec<Vec3> = margin.iter().step_by(4).copied().collect();
        let m_audit = CrownDesigner::audit_margin_nodes(
            &arch,
            None,
            &ctrl_16,
            &margin,
            Vec3::Z,
            &params,
            true,
        );
        assert!(
            m_audit.valid_for_cement && m_audit.bad_node_indices.is_empty(),
            "Un margen limpio junto a dientes vecinos en voladizo NO debe marcar nodos rojos (bad={:?}, msg={})",
            m_audit.bad_node_indices,
            m_audit.message
        );

        let (cem, min_t, audit) = CrownDesigner::generate_auxiliary_shells(
            &arch,
            &margin,
            Vec3::Z,
            &params,
        )
        .expect("generate_auxiliary_shells con dientes vecinos");

        assert!(
            audit.valid_for_production,
            "El muñón con vecinos cercanos debe pasar válido: {}",
            audit.warning_message
        );
        assert_eq!(audit.exposed_die_points, 0, "Cero puntos expuestos sobre el muñón");

        let (cem_mn, cem_mx) = cem.bounds();
        let (min_mn, min_mx) = min_t.bounds();
        assert!(
            cem_mx.z < 5.6 && cem_mx.x < 4.7 && cem_mn.x > -4.7,
            "El espaciador cian JAMÁS debe dispararse/explotar hacia los dientes vecinos (bounds={:?}..{:?})",
            cem_mn,
            cem_mx
        );
        assert!(
            min_mx.z < 6.6 && min_mx.x < 5.6 && min_mn.x > -5.6,
            "La capa de grosor mínimo JAMÁS debe dispararse hacia los dientes vecinos (bounds={:?}..{:?})",
            min_mn,
            min_mx
        );

        // Verificar que la superficie del espacio de cementado + bisel 45° sea completamente lisa
        // (sin piquitos ni dientes triangulares locales en el borde marginal)
        let cem_adj = cem.build_adjacency();
        let mut max_cem_spike = 0.0_f32;
        let margin_start = cem.positions.len().saturating_sub(144);
        for i in 1..margin_start {
            if !cem_adj[i].is_empty() {
                let avg: Vec3 = cem_adj[i].iter().map(|&nb| cem.positions[nb]).sum::<Vec3>()
                    / cem_adj[i].len() as f32;
                max_cem_spike = max_cem_spike.max(cem.positions[i].distance(avg));
            }
        }
        assert!(
            max_cem_spike < 0.16,
            "El espacio de cementado con bisel 45° no debe presentar piquitos locales (max_cem_spike={:.4} mm)",
            max_cem_spike
        );
    }

    #[test]
    fn test_tight_overhang_margin_flags_red_nodes_and_clears_when_fixed() {
        use apex_dentalcraft_lib::geometry::crown::MarginNodeIssue;
        use apex_dentalcraft_lib::geometry::mesh::TriMeshData;
        use apex_dentalcraft_lib::ui::design_panel::DesignPanelState;

        // 1. Verificar preferencias por defecto de Etapa 4 (Restauración 100%, Modelo 80%, Antagonista OFF)
        let dp = DesignPanelState::default();
        assert!((dp.op_restorations - 1.00).abs() < 1e-4, "Restauración por default debe ser 100%");
        assert!((dp.op_model - 0.80).abs() < 1e-4, "Modelo/muñón por default debe ser 80%");
        assert!(!dp.vis_antagonist, "Antagonista por default debe iniciar desactivado");

        // 2. Construir muñón con voladizo/abombamiento en pared superior (+Y) como en Imagen 1:
        //    Base/cuello en r=3.45 mm (z=0.0), pared +Y abomba hacia afuera hasta r=4.20 mm en z=2.8 mm,
        //    y hombro exterior limpio en r=4.55 mm (z=0.0).
        let n_seg = 64usize;
        let mut verts = Vec::new();
        let mut faces = Vec::new();
        verts.push(Vec3::new(0.0, 0.0, 3.8)); // polo techo
        // Anillo superior abombado (+Y sobresale hasta r=4.20)
        for k in 0..n_seg {
            let th = 2.0 * std::f32::consts::PI * k as f32 / n_seg as f32;
            let sy = th.sin();
            let r_bulge = if sy > 0.2 { 4.20 } else { 3.30 };
            verts.push(Vec3::new(r_bulge * th.cos(), r_bulge * sy, 2.8));
        }
        // Cuello retraído en +Y (socavado real)
        for k in 0..n_seg {
            let th = 2.0 * std::f32::consts::PI * k as f32 / n_seg as f32;
            let sy = th.sin();
            let r_neck = if sy > 0.2 { 3.45 } else { 3.65 };
            verts.push(Vec3::new(r_neck * th.cos(), r_neck * sy, 0.0));
        }
        // Faldón gingival plano exterior hasta r=5.8 mm
        for k in 0..n_seg {
            let th = 2.0 * std::f32::consts::PI * k as f32 / n_seg as f32;
            verts.push(Vec3::new(5.8 * th.cos(), 5.8 * th.sin(), -0.1));
        }
        for k in 0..n_seg {
            let nk = (k + 1) % n_seg;
            let t0 = (1 + k) as u32;
            let t1 = (1 + nk) as u32;
            let n0 = (1 + n_seg + k) as u32;
            let n1 = (1 + n_seg + nk) as u32;
            let g0 = (1 + 2 * n_seg + k) as u32;
            let g1 = (1 + 2 * n_seg + nk) as u32;
            faces.push([0, t0, t1]);
            faces.push([t0, n0, n1]);
            faces.push([t0, n1, t1]);
            faces.push([n0, g0, g1]);
            faces.push([n0, g1, n1]);
        }
        let undercut_die = TriMeshData::new(verts, faces);

        let mut mctrl = MarginLineController::default();
        mctrl.set_target_mesh(undercut_die.clone());

        // 3. Trazar 12 nodos donde los nodos en +Y (índices 2, 3, 4) quedan MUY AJUSTADOS bajo el voladizo!
        let mut tight_nodes = Vec::new();
        for k in 0..12 {
            let th = 2.0 * std::f32::consts::PI * k as f32 / 12.0;
            let sy = th.sin();
            let r = if sy > 0.35 { 3.55 } else { 4.25 };
            tight_nodes.push([r * th.cos(), r * sy, 0.0]);
        }
        mctrl.load_points(&tight_nodes);

        let params = DesignParameters::default();
        let bad_audit = CrownDesigner::audit_margin_nodes(
            &undercut_die,
            mctrl.collider.as_ref(),
            &mctrl.control_points,
            &mctrl.curve_points,
            Vec3::Z,
            &params,
            true,
        );
        assert!(
            !bad_audit.valid_for_cement,
            "Un margen ajustado bajo voladizo DEBE avisar e impedir continuar"
        );
        assert!(
            bad_audit.issues.contains(&MarginNodeIssue::TightOverhang),
            "Debe reportar TightOverhang (issues={:?})",
            bad_audit.issues
        );
        assert!(
            bad_audit.bad_node_indices.contains(&3),
            "El nodo superior #3 metido bajo el voladizo +Y debe marcarse en ROJO (bad={:?})",
            bad_audit.bad_node_indices
        );
        assert!(
            !bad_audit.bad_node_indices.contains(&9),
            "El nodo opuesto #9 en zona limpia (-Y) debe permanecer blanco normal (bad={:?})",
            bad_audit.bad_node_indices
        );

        // 4. Mover TODOS los nodos rojos bajo el voladizo (+Y: 1..=5) hacia afuera (r=4.45 mm) -> todos vuelven a blanco!
        for k in [1usize, 2, 3, 4, 5] {
            let th = 2.0 * std::f32::consts::PI * k as f32 / 12.0;
            tight_nodes[k] = [4.45 * th.cos(), 4.45 * th.sin(), -0.05];
        }
        mctrl.load_points(&tight_nodes);
        let ok_audit = CrownDesigner::audit_margin_nodes(
            &undercut_die,
            mctrl.collider.as_ref(),
            &mctrl.control_points,
            &mctrl.curve_points,
            Vec3::Z,
            &params,
            true,
        );
        assert!(
            ok_audit.valid_for_cement && ok_audit.bad_node_indices.is_empty(),
            "Al mover los nodos rojos hacia afuera, todos deben volver a blanco normal (bad={:?}, msg={})",
            ok_audit.bad_node_indices,
            ok_audit.message
        );
    }

    #[test]
    fn test_configurable_bevel_zero_stl_folds_settings_and_stl_viewer() {
        use apex_dentalcraft_lib::core::settings::AppSettings;
        use apex_dentalcraft_lib::geometry::mesh::TriMeshData;
        use apex_dentalcraft_lib::ui::stl_viewer_panel::{SceneRole, StlColorMode, StlViewerPanelState};

        // 1. Verificar bisel marginal configurable (0.00 mm vs 0.06 mm vs 0.18 mm) y STL 0 pliegues / 0 huecos
        let mand = clean_mesh(&generate_parametric_demo_arch(false), true, 1);
        let mut mctrl = MarginLineController::default();
        mctrl.set_target_mesh(mand.clone());
        // Muñón tallado #36 real de la arcada demo (centrado en x=0, y=13.0)
        let seed = mand
            .positions
            .iter()
            .filter(|p| p.x.abs() < 4.5 && (p.y - 13.0).abs() < 4.5)
            .max_by(|a, b| a.z.partial_cmp(&b.z).unwrap())
            .copied()
            .unwrap_or(Vec3::new(0.0, 13.0, 10.0));
        mctrl.auto_detect_margin_from_stump_top(seed, Vec3::Z, 16, 4.6);

        let lib_tooth = ToothLibraryManager::generate_procedural_tooth_crown(36);
        let mut p_zero = DesignParameters::default();
        p_zero.margin_bevel_mm = 0.00;
        let mut p_high = DesignParameters::default();
        p_high.margin_bevel_mm = 0.18;

        let (cem_zero, _, aud_z) =
            CrownDesigner::generate_auxiliary_shells(&mand, &mctrl.curve_points, Vec3::Z, &p_zero)
                .expect("cem bevel 0.00");
        let (cem_high, _, aud_h) =
            CrownDesigner::generate_auxiliary_shells(&mand, &mctrl.curve_points, Vec3::Z, &p_high)
                .expect("cem bevel 0.18");
        assert!(aud_z.valid_for_production && aud_h.valid_for_production);

        // El bisel reducido (0.00 mm) debe ser menos pronunciado que el bisel alto (0.18 mm)
        let n_v = cem_zero.positions.len();
        let bevel_start = n_v - 288;
        let bevel_end = n_v - 144;
        let mut diff_sum = 0.0_f32;
        for i in bevel_start..bevel_end {
            diff_sum += cem_zero.positions[i].distance(cem_high.positions[i]);
        }
        assert!(
            diff_sum / 144.0 > 0.015,
            "Reducir el bisel marginal debe modificar apreciablemente el perfil cervical (avg_diff={:.4})",
            diff_sum / 144.0
        );

        // Generar corona final con bisel por defecto (0.06 mm), exportar STL binario y recargar
        let p_def = DesignParameters::default();
        assert!((p_def.margin_bevel_mm - 0.06).abs() < 1e-4);
        let crown = CrownDesigner::generate_crown(&mand, &mctrl.curve_points, &lib_tooth, Vec3::Z, &p_def, None)
            .expect("generate_crown default bevel");

        let tmp = std::env::temp_dir().join("apex_dentalcraft_test_stl_viewer_v1");
        let _ = std::fs::remove_dir_all(&tmp);
        std::fs::create_dir_all(&tmp).unwrap();
        let crown_stl = tmp.join("Corona_36_Test.stl");
        let die_stl = tmp.join("Modelo_Trabajo.stl");
        crown.save_stl_binary(&crown_stl).expect("save crown stl");
        mand.save_stl_binary(&die_stl).expect("save die stl");

        let reloaded = TriMeshData::load_stl(&crown_stl).expect("reload crown stl");
        let audit = reloaded.inspect_cam();
        assert!(
            audit.watertight
                && audit.open_edges == 0
                && audit.non_manifold_edges == 0
                && audit.flipped_winding_edges == 0
                && audit.folded_edges == 0,
            "El STL exportado y recargado debe ser 100% watertight sin aristas abiertas, invertidas ni pliegues: {:?}",
            audit
        );
        // Verificar 0 pliegues en el INTERIOR del intaglio o exterior (incluso a umbral estricto dot < -0.88)
        // y verificar también que no existe ninguna arista anómala larga (regresión de la rendija de 4.8 mm)
        let mut max_edge_len = 0.0_f32;
        for &[a, b, c] in &reloaded.indices {
            let pa = reloaded.positions[a as usize];
            let pb = reloaded.positions[b as usize];
            let pc = reloaded.positions[c as usize];
            max_edge_len = max_edge_len.max(pa.distance(pb)).max(pb.distance(pc)).max(pc.distance(pa));
        }
        assert!(
            max_edge_len < 1.8,
            "No debe existir ningún triángulo estirado o rendija radial en el STL (max_edge_len={:.3} mm)",
            max_edge_len
        );
        let mut interior_folds = 0usize;
        {
            let mut emap: std::collections::HashMap<(u32, u32), Vec<usize>> = std::collections::HashMap::new();
            let mut fnrms = Vec::with_capacity(crown.indices.len());
            for (fi, &[a, b, c]) in crown.indices.iter().enumerate() {
                let n = (crown.positions[b as usize] - crown.positions[a as usize])
                    .cross(crown.positions[c as usize] - crown.positions[a as usize])
                    .normalize_or_zero();
                fnrms.push(n);
                for (u, v) in [(a, b), (b, c), (c, a)] {
                    let k = if u < v { (u, v) } else { (v, u) };
                    emap.entry(k).or_default().push(fi);
                }
            }
            for fl in emap.values() {
                if fl.len() == 2 && fnrms[fl[0]].dot(fnrms[fl[1]]) < -0.88 {
                    let f0_ext = crown.indices[fl[0]].iter().any(|&vi| crown.is_external[vi as usize]);
                    let f1_ext = crown.indices[fl[1]].iter().any(|&vi| crown.is_external[vi as usize]);
                    if f0_ext == f1_ext {
                        interior_folds += 1;
                    }
                }
            }
        }
        assert_eq!(
            interior_folds, 0,
            "El STL NO debe tener pliegues ni rendijas dobladas en el interior del intaglio/exterior (interior_folds={})",
            interior_folds
        );

        // Regresión específica de muñón de paredes casi verticales (90°) + hombro chamfer:
        // antes, `ray_z_on_collider` teletransportaba el anillo de bisel +4.5 mm hasta el techo!
        {
            let mut vpos = Vec::new();
            let mut vidx = Vec::new();
            let nt = 64usize;
            let z_levels = [0.0_f32, 0.0, 0.35, 1.5, 3.0, 4.3, 4.6];
            let r_levels = [4.8_f32, 3.4, 3.28, 3.26, 3.25, 3.15, 0.0];
            for (li, (&z, &r)) in z_levels.iter().zip(r_levels.iter()).enumerate() {
                if li == z_levels.len() - 1 {
                    vpos.push(Vec3::new(0.0, 0.0, z));
                } else {
                    for k in 0..nt {
                        let th = 2.0 * std::f32::consts::PI * k as f32 / nt as f32;
                        vpos.push(Vec3::new(r * th.cos(), r * th.sin(), z));
                    }
                }
            }
            for li in 0..(z_levels.len() - 2) {
                let b0 = (li * nt) as u32;
                let b1 = ((li + 1) * nt) as u32;
                for k in 0..nt {
                    let nk = (k + 1) % nt;
                    vidx.push([b0 + k as u32, b1 + k as u32, b1 + nk as u32]);
                    vidx.push([b0 + k as u32, b1 + nk as u32, b0 + nk as u32]);
                }
            }
            let top_ring = ((z_levels.len() - 2) * nt) as u32;
            let pole = (vpos.len() - 1) as u32;
            for k in 0..nt {
                let nk = (k + 1) % nt;
                vidx.push([top_ring + k as u32, pole, top_ring + nk as u32]);
            }
            let steep_die = TriMeshData::new(vpos, vidx);
            let steep_margin: Vec<Vec3> = (0..64)
                .map(|k| {
                    let th = 2.0 * std::f32::consts::PI * k as f32 / 64.0;
                    Vec3::new(3.38 * th.cos(), 3.38 * th.sin(), 0.02)
                })
                .collect();
            let steep_crown = CrownDesigner::generate_crown(
                &steep_die,
                &steep_margin,
                &lib_tooth,
                Vec3::Z,
                &p_def,
                None,
            )
            .expect("generate_crown on steep vertical-walled die");
            let sa = steep_crown.inspect_cam();
            assert!(
                sa.watertight && sa.open_edges == 0 && sa.non_manifold_edges == 0 && sa.folded_edges == 0,
                "Corona sobre muñón de paredes verticales debe salir 100% limpia y sin rendijas: {:?}",
                sa
            );
            let mut smax_e = 0.0_f32;
            for &[a, b, c] in &steep_crown.indices {
                let pa = steep_crown.positions[a as usize];
                let pb = steep_crown.positions[b as usize];
                let pc = steep_crown.positions[c as usize];
                smax_e = smax_e.max(pa.distance(pb)).max(pb.distance(pc)).max(pc.distance(pa));
            }
            assert!(
                smax_e < 1.8,
                "Muñón de paredes verticales NO debe generar teletransporte ni rendija en el bisel (max_edge={:.3} mm)",
                smax_e
            );
        }

        // 2. Verificar persistencia de Configuración Global (settings.json)
        let mut cfg = AppSettings::default();
        cfg.margin_line_radius_mm = 0.024;
        cfg.default_margin_bevel_mm = 0.03;
        cfg.stage4_restoration_opacity = 0.95;
        cfg.camera_rotate_sensitivity = 1.45;
        cfg.save_to_root(&tmp).expect("save settings");
        let cfg2 = AppSettings::load_from_root(&tmp);
        assert!((cfg2.margin_line_radius_mm - 0.024).abs() < 1e-4);
        assert!((cfg2.default_margin_bevel_mm - 0.03).abs() < 1e-4);
        assert!((cfg2.stage4_restoration_opacity - 0.95).abs() < 1e-4);
        assert!((cfg2.camera_rotate_sensitivity - 1.45).abs() < 1e-4);

        // 3. Verificar Visor STL Propio estilo 3Shape (escenas, transparencias y mapa real de grosores)
        let mut sv = StlViewerPanelState::default();
        sv.apply_settings(&cfg2);
        let msg = sv.open_files(&[crown_stl.clone(), die_stl.clone()]);
        assert!(msg.contains("2 mesh(es)"), "Must load both STL files: {}", msg);
        assert_eq!(sv.objects.len(), 2);
        assert_eq!(sv.objects[0].role, SceneRole::Restoration);
        assert_eq!(sv.objects[1].role, SceneRole::ModelDie);

        // Probar escenas rápidas
        sv.apply_scene_preset("restoration_only");
        assert!(sv.objects[0].visible && !sv.objects[1].visible);
        sv.apply_scene_preset("fit_cement");
        assert!(sv.objects[0].visible && sv.objects[0].opacity < 0.75 && sv.objects[1].visible);

        // Probar análisis de grosor de pared en STL independiente
        sv.ensure_color_analysis(0, StlColorMode::WallThickness);
        let (mn, avg, mx) = sv.objects[0]
            .thickness_stats
            .expect("debe calcular estadísticas reales de grosor de pared");
        assert!(
            mn > 0.05 && avg > 0.45 && mx <= 3.01,
            "El mapa de grosores del Visor STL debe medir espesores clínicos reales (min={:.2}, avg={:.2}, max={:.2})",
            mn,
            avg,
            mx
        );

        let _ = std::fs::remove_dir_all(&tmp);
    }

    #[test]
    fn test_concentric_layers_zero_micro_spikes_and_standalone_3d_viewer_suite() {
        use apex_dentalcraft_lib::core::win_assoc::write_png_rgba;
        use apex_dentalcraft_lib::geometry::mesh::TriMeshData;
        use apex_dentalcraft_lib::ui::stl_viewer_panel::{
            AngleMeasurement, StlColorMode, StlViewerPanelState,
        };

        // 1. Verificar apilamiento concéntrico limpio (Cemento < Intaglio Corona < Grosor Mínimo)
        //    y 0 micro-picos locales en intaglio y grosor mínimo
        let mand = clean_mesh(&generate_parametric_demo_arch(false), true, 1);
        let mut mctrl = MarginLineController::default();
        mctrl.set_target_mesh(mand.clone());
        let seed = mand
            .positions
            .iter()
            .filter(|p| p.x.abs() < 4.5 && (p.y - 13.0).abs() < 4.5)
            .max_by(|a, b| a.z.partial_cmp(&b.z).unwrap())
            .copied()
            .unwrap_or(Vec3::new(0.0, 13.0, 10.0));
        mctrl.auto_detect_margin_from_stump_top(seed, Vec3::Z, 16, 4.6);

        let params = DesignParameters::default();
        let lib_tooth = ToothLibraryManager::generate_procedural_tooth_crown(36);
        let (cem, min_t, aud) =
            CrownDesigner::generate_auxiliary_shells(&mand, &mctrl.curve_points, Vec3::Z, &params)
                .expect("aux shells");
        assert!(aud.valid_for_production);
        assert_eq!(cem.positions.len(), min_t.positions.len());

        let crown = CrownDesigner::generate_crown(
            &mand,
            &mctrl.curve_points,
            &lib_tooth,
            Vec3::Z,
            &params,
            None,
        )
        .expect("crown");

        // Misma topología 144×32 entre capa de cementado y el intaglio real de la corona (0 desajuste)
        let n_dome_inner = cem.positions.len() - 144;
        let crown_int_start = crown.positions.len() - n_dome_inner;
        let mut max_layer_drift = 0.0_f32;
        for k in 0..n_dome_inner {
            let d = cem.positions[k].distance(crown.positions[crown_int_start + k]);
            max_layer_drift = max_layer_drift.max(d);
        }
        assert!(
            max_layer_drift < 0.025,
            "La capa de cementado debe coincidir 1:1 con el intaglio real (offset visual <= 0.006 mm, max_drift={:.4})",
            max_layer_drift
        );

        // Verificar que la capa naranja de grosor mínimo no tenga picos/pellizcos bruscos
        // y que NINGÚN vértice naranja quede por detrás del espacio de cementado cian
        let min_adj = min_t.build_adjacency();
        let mut max_min_spike = 0.0_f32;
        let mut min_below_cem = 0usize;
        for vi in 0..n_dome_inner {
            if (min_t.positions[vi] - cem.positions[vi]).dot(cem.normals[vi]) < 0.015 {
                min_below_cem += 1;
            }
            if vi >= 1 && !min_adj[vi].is_empty() {
                let avg: Vec3 = min_adj[vi].iter().map(|&nb| min_t.positions[nb]).sum::<Vec3>()
                    / min_adj[vi].len() as f32;
                max_min_spike = max_min_spike.max(min_t.positions[vi].distance(avg));
            }
        }
        assert_eq!(
            min_below_cem, 0,
            "Ningún vértice de la capa naranja de grosor mínimo debe atravesar ni quedar por debajo del cementado cian"
        );
        assert!(
            max_min_spike < 0.22,
            "La capa naranja de grosor mínimo debe ser lisa y sin pellizcos cerca del margen (max_spike={:.4} mm)",
            max_min_spike
        );

        // Verificar que el intaglio de la corona no presente micro-picos ni burbujas en chamfers cóncavos
        let crown_adj = crown.build_adjacency();
        let mut max_intaglio_spike = 0.0_f32;
        for vi in crown_int_start..crown.positions.len() {
            if !crown_adj[vi].is_empty() {
                let avg: Vec3 = crown_adj[vi]
                    .iter()
                    .map(|&nb| crown.positions[nb])
                    .sum::<Vec3>()
                    / crown_adj[vi].len() as f32;
                max_intaglio_spike = max_intaglio_spike.max(crown.positions[vi].distance(avg));
            }
        }
        assert!(
            max_intaglio_spike < 0.16,
            "El intaglio de la corona debe ser liso y sin micro-picos ni lomas (max_intaglio_spike={:.4} mm)",
            max_intaglio_spike
        );

        // 2. Verificar suite completa de Apex 3D Viewer (Curvatura, Socavados con eje libre, Ángulo 3 pts, OBJ/PLY, PNG)
        let tmp = std::env::temp_dir().join("apex_3d_viewer_suite_test_v2");
        let _ = std::fs::remove_dir_all(&tmp);
        std::fs::create_dir_all(&tmp).unwrap();
        let stl_p = tmp.join("corona_test.stl");
        let obj_p = tmp.join("corona_conv.obj");
        let ply_p = tmp.join("corona_conv.ply");
        let png_p = tmp.join("captura_test.png");
        crown.save_stl_binary(&stl_p).unwrap();

        let mut viewer = StlViewerPanelState::default();
        viewer.standalone_mode = true;
        viewer.open_files(&[tmp.clone()]); // carga por carpeta
        assert_eq!(viewer.objects.len(), 1);
        viewer.ensure_color_analysis(0, StlColorMode::Curvature);
        assert_eq!(viewer.objects[0].mesh.custom_scalars.len(), viewer.objects[0].mesh.positions.len());
        viewer.undercut_axis = Vec3::new(0.2, -0.1, 0.97).normalize();
        viewer.ensure_color_analysis(0, StlColorMode::Undercut);
        assert_eq!(viewer.objects[0].mesh.custom_scalars.len(), viewer.objects[0].mesh.positions.len());

        // Exportación / conversión OBJ y PLY + recarga
        viewer.export_selected_to(&obj_p).expect("export obj");
        viewer.export_selected_to(&ply_p).expect("export ply");
        let reloaded_obj = TriMeshData::load_from_path(&obj_p).expect("reload obj");
        let reloaded_ply = TriMeshData::load_from_path(&ply_p).expect("reload ply");
        assert_eq!(reloaded_obj.indices.len(), crown.indices.len());
        assert_eq!(reloaded_ply.indices.len(), crown.indices.len());

        // Goniómetro 3 puntos exacto (90.0°)
        let ang = AngleMeasurement {
            a: Vec3::new(1.0, 0.0, 0.0),
            vertex: Vec3::ZERO,
            b: Vec3::new(0.0, 1.0, 0.0),
        };
        assert!((ang.angle_deg() - 90.0).abs() < 1e-3);

        // Codificador PNG nativo válido
        let rgba = vec![180u8; 8 * 8 * 4];
        write_png_rgba(&png_p, 8, 8, &rgba).expect("write png");
        let png_bytes = std::fs::read(&png_p).unwrap();
        assert_eq!(&png_bytes[..8], &[137, 80, 78, 71, 13, 10, 26, 10]);

        let _ = std::fs::remove_dir_all(&tmp);
    }

    #[test]
    fn test_intraoral_gingival_cuff_outside_margin_never_flags_red_while_inner_die_undercut_does() {
        use apex_dentalcraft_lib::geometry::crown::MarginNodeIssue;
        use apex_dentalcraft_lib::geometry::mesh::TriMeshData;

        // Construir escaneo intraoral SIN troquelar:
        // 1) Muñón tallado limpio expulsivo dentro del margen (radio base 3.2 mm -> techo 2.6 mm, z=0..4.4 mm)
        // 2) Surco gingival en r=3.35 mm, z=0.0 mm
        // 3) Encía / tejido blando externo que sube hasta z=2.3 mm y SE INCLINA HACIA ADENTRO (voladizo
        //    de tejido blando desde AFUERA del margen, llegando a r=3.15 mm encima del surco r=3.35 mm!)
        let nt = 72usize;
        let profile: &[(f32, f32)] = &[
            (0.00, 4.40), // polo techo muñón
            (1.60, 4.35), // techo muñón
            (2.60, 4.10), // hombro oclusal muñón
            (2.90, 2.50), // pared axial expulsiva muñón
            (3.15, 0.80), // pared cervical muñón
            (3.35, 0.00), // fondo del surco gingival (LÍNEA DE MARGEN!)
            (3.55, 0.30), // pared externa del surco (tejido blando subiendo!)
            (3.30, 1.35), // encía externa inclinándose sobre el surco (socavado EXTERNO!)
            (3.18, 2.10), // cresta gingival en voladizo sobre el margen!
            (3.95, 2.35), // cresta gingival externa
            (5.40, 1.60), // encía vestibular/palatina externa
            (6.50, 0.00), // borde base encía
        ];
        let mut verts = Vec::new();
        let mut tris = Vec::new();
        verts.push(Vec3::new(0.0, 0.0, profile[0].1));
        for &(r, z) in &profile[1..] {
            for k in 0..nt {
                let th = 2.0 * std::f32::consts::PI * k as f32 / nt as f32;
                verts.push(Vec3::new(r * th.cos(), r * th.sin(), z));
            }
        }
        // Tapa superior del muñón (CCW mirando desde +Z -> normales estrictamente hacia afuera!)
        for k in 0..nt {
            let nk = (k + 1) % nt;
            tris.push([0, 1 + k as u32, 1 + nk as u32]);
        }
        let n_rings = profile.len() - 1;
        for ri in 0..(n_rings - 1) {
            let b0 = 1 + (ri * nt) as u32;
            let b1 = 1 + ((ri + 1) * nt) as u32;
            for k in 0..nt {
                let nk = (k + 1) % nt;
                tris.push([b0 + k as u32, b1 + k as u32, b1 + nk as u32]);
                tris.push([b0 + k as u32, b1 + nk as u32, b0 + nk as u32]);
            }
        }
        let ios_scan = TriMeshData::new(verts, tris);

        // 8 nodos de margen en el surco gingival (r=3.35, z=0.02) — justo DEBAJO de la encía externa en voladizo!
        let ctrl_8: Vec<Vec3> = (0..8)
            .map(|k| {
                let th = 2.0 * std::f32::consts::PI * k as f32 / 8.0;
                Vec3::new(3.35 * th.cos(), 3.35 * th.sin(), 0.02)
            })
            .collect();
        let curve_96: Vec<Vec3> = (0..96)
            .map(|k| {
                let th = 2.0 * std::f32::consts::PI * k as f32 / 96.0;
                Vec3::new(3.35 * th.cos(), 3.35 * th.sin(), 0.02)
            })
            .collect();

        let params = DesignParameters::default();
        let audit = CrownDesigner::audit_margin_nodes(
            &ios_scan,
            None,
            &ctrl_8,
            &curve_96,
            Vec3::Z,
            &params,
            true,
        );
        assert!(
            audit.valid_for_cement && audit.bad_node_indices.is_empty(),
            "Un socavado EXTERNO de tejido blando/encía por fuera del margen JAMÁS debe pintar nodos en rojo ni bloquear (bad={:?}, issues={:?}, msg={})",
            audit.bad_node_indices,
            audit.issues,
            audit.message
        );

        // El espacio de cementado y la corona deben generarse 100% limpios ignorando la encía externa!
        let (cem, _, die_aud) =
            CrownDesigner::generate_auxiliary_shells(&ios_scan, &curve_96, Vec3::Z, &params)
                .expect("aux shells sobre escaneo intraoral con encía en voladizo");
        assert!(die_aud.valid_for_production && die_aud.exposed_die_points == 0);
        let (cmn, cmx) = cem.bounds();
        assert!(
            cmx.x < 3.65 && cmn.x > -3.65 && cmx.y < 3.65 && cmn.y > -3.65,
            "El cementado solo debe abarcar del margen hacia adentro (sin invadir la encía externa): {:?}..{:?}",
            cmn,
            cmx
        );

        let lib = ToothLibraryManager::generate_procedural_tooth_crown(36);
        let crown = CrownDesigner::generate_crown(&ios_scan, &curve_96, &lib, Vec3::Z, &params, None)
            .expect("corona sobre escaneo intraoral sin troquelar");
        let ca = crown.inspect_cam();
        assert!(
            ca.watertight && ca.open_edges == 0 && ca.non_manifold_edges == 0 && ca.folded_edges == 0,
            "Corona sobre escaneo intraoral debe ser 100% watertight y limpia: {:?}",
            ca
        );

        // Contraste: muñón con socavado RETENTIVO REAL hacia adentro (pared del propio muñón
        // abombada sobresaliendo por encima del margen cervical en x=+3.45 sobre el nodo en x=+2.90)
        // mientras mantiene la misma encía externa en voladizo -> SÍ debe marcarse en ROJO!
        let undercut_profile: &[(f32, f32)] = &[
            (0.00, 4.40),
            (1.60, 4.35),
            (2.80, 3.80),
            (3.45, 2.30), // abombamiento real del propio muñón (socavado retentivo interno!)
            (2.85, 0.45), // cuello cervical estrecho del muñón bajo su propio abombamiento
            (2.90, 0.00), // margen cervical atrapado bajo el abombamiento del muñón
            (3.40, 0.40),
            (3.20, 1.80), // encía externa
            (4.40, 2.10),
            (6.00, 0.00),
        ];
        let mut uverts = Vec::new();
        let mut utris = Vec::new();
        uverts.push(Vec3::new(0.0, 0.0, undercut_profile[0].1));
        for &(r, z) in &undercut_profile[1..] {
            for k in 0..nt {
                let th = 2.0 * std::f32::consts::PI * k as f32 / nt as f32;
                uverts.push(Vec3::new(r * th.cos(), r * th.sin(), z));
            }
        }
        for k in 0..nt {
            let nk = (k + 1) % nt;
            utris.push([0, 1 + k as u32, 1 + nk as u32]);
        }
        for ri in 0..(undercut_profile.len() - 2) {
            let b0 = 1 + (ri * nt) as u32;
            let b1 = 1 + ((ri + 1) * nt) as u32;
            for k in 0..nt {
                let nk = (k + 1) % nt;
                utris.push([b0 + k as u32, b1 + k as u32, b1 + nk as u32]);
                utris.push([b0 + k as u32, b1 + nk as u32, b0 + nk as u32]);
            }
        }
        let undercut_ios = TriMeshData::new(uverts, utris);
        let tight_ctrl: Vec<Vec3> = (0..8)
            .map(|k| {
                let th = 2.0 * std::f32::consts::PI * k as f32 / 8.0;
                Vec3::new(2.90 * th.cos(), 2.90 * th.sin(), 0.02)
            })
            .collect();
        let tight_curve: Vec<Vec3> = (0..96)
            .map(|k| {
                let th = 2.0 * std::f32::consts::PI * k as f32 / 96.0;
                Vec3::new(2.90 * th.cos(), 2.90 * th.sin(), 0.02)
            })
            .collect();
        let bad_aud = CrownDesigner::audit_margin_nodes(
            &undercut_ios,
            None,
            &tight_ctrl,
            &tight_curve,
            Vec3::Z,
            &params,
            false,
        );
        assert!(
            !bad_aud.valid_for_cement && bad_aud.issues.contains(&MarginNodeIssue::TightOverhang),
            "Un socavado RETENTIVO REAL del propio muñón (del nodo hacia adentro) SÍ debe marcarse (bad={:?}, issues={:?})",
            bad_aud.bad_node_indices,
            bad_aud.issues
        );
        assert!(
            !bad_aud.blocks_design && bad_aud.has_undercut_warning,
            "Un socavado del muñón debe mostrar aviso en ámbar sin bloquear el avance a Etapa 4"
        );
    }

    #[test]
    fn test_proclined_anterior_auto_axis_and_bilateral_virtual_blockout_never_trap_user() {
        use apex_dentalcraft_lib::core::case_manager::DesignParameters;
        use apex_dentalcraft_lib::core::library_manager::ToothLibraryManager;
        use apex_dentalcraft_lib::geometry::crown::{CrownDesigner, MarginNodeIssue};
        use apex_dentalcraft_lib::geometry::mesh::TriMeshData;
        use glam::{Mat3, Vec3};

        // 1) Incisivo superior #11 proclivado 22° hacia vestibular (+X) respecto al eje por defecto -Z:
        //    El eje natural del diente es rot_y(-22°) * (0,0,-1).
        let rot = Mat3::from_rotation_y(-22.0_f32.to_radians());
        let nt = 64usize;
        let profile: &[(f32, f32)] = &[
            (0.0, 5.2),
            (1.1, 4.9),
            (1.9, 3.8),
            (2.4, 2.2),
            (2.9, 0.8),
            (3.25, 0.0), // hombro cervical exacto
            (3.9, -0.4),
            (5.5, -1.2),
        ];
        let mut verts = Vec::new();
        let mut tris = Vec::new();
        verts.push(rot * Vec3::new(0.0, 0.0, -5.25));
        for &(r, h) in &profile[1..] {
            for k in 0..nt {
                let th = 2.0 * std::f32::consts::PI * k as f32 / nt as f32;
                // En maxilar, la corona apunta hacia -Z local
                let p_tooth = Vec3::new(r * th.cos(), r * 0.82 * th.sin(), -h);
                verts.push(rot * p_tooth);
            }
        }
        for k in 0..nt {
            let nk = (k + 1) % nt;
            tris.push([0, 1 + nk as u32, 1 + k as u32]);
        }
        for ring in 0..(profile.len() - 2) {
            let b0 = 1 + ring * nt;
            let b1 = 1 + (ring + 1) * nt;
            for k in 0..nt {
                let nk = (k + 1) % nt;
                tris.push([(b0 + k) as u32, (b1 + nk) as u32, (b1 + k) as u32]);
                tris.push([(b0 + k) as u32, (b0 + nk) as u32, (b1 + nk) as u32]);
            }
        }
        let incisor = TriMeshData::new(verts, tris);

        let ctrl_12: Vec<Vec3> = (0..12)
            .map(|k| {
                let th = 2.0 * std::f32::consts::PI * k as f32 / 12.0;
                rot * Vec3::new(3.25 * th.cos(), 3.25 * 0.82 * th.sin(), 0.0)
            })
            .collect();
        let curve_96: Vec<Vec3> = (0..96)
            .map(|k| {
                let th = 2.0 * std::f32::consts::PI * k as f32 / 96.0;
                rot * Vec3::new(3.25 * th.cos(), 3.25 * 0.82 * th.sin(), 0.0)
            })
            .collect();

        let params = DesignParameters::default();
        // Con el eje vertical por defecto (0,0,-1), hay socavado labial, PERO NO debe bloquear el avance!
        let aud_vert = CrownDesigner::audit_margin_nodes(
            &incisor,
            None,
            &ctrl_12,
            &curve_96,
            Vec3::NEG_Z,
            &params,
            false,
        );
        assert!(
            !aud_vert.blocks_design,
            "Un incisivo superior proclivado 22° JAMÁS debe bloquear el avance a Etapa 4 (issues={:?})",
            aud_vert.issues
        );

        // Auto-Eje del Muñón debe encontrar un eje con 0 socavados
        let (opt_ax, rem_ov, tilt_deg) =
            CrownDesigner::compute_optimal_insertion_axis(&incisor, &curve_96, Vec3::NEG_Z);
        assert_eq!(
            rem_ov, 0,
            "Auto-Eje debe eliminar todos los socavados en el incisivo proclivado 22° (opt_ax={:?}, tilt={:.1}°)",
            opt_ax, tilt_deg
        );
        let aud_opt = CrownDesigner::audit_margin_nodes(
            &incisor,
            None,
            &ctrl_12,
            &curve_96,
            opt_ax,
            &params,
            false,
        );
        assert!(
            aud_opt.valid_for_cement && aud_opt.bad_node_indices.is_empty(),
            "Con Auto-Eje aplicado, todos los nodos deben quedar limpios (bad={:?}, msg={})",
            aud_opt.bad_node_indices,
            aud_opt.message
        );

        // Generar cemento + corona directamente desde el eje vertical (0,0,-1): debe auto-ajustarse y pasar!
        let (_, _, aux_aud) =
            CrownDesigner::generate_auxiliary_shells(&incisor, &curve_96, Vec3::NEG_Z, &params)
                .expect("aux shells incisivo proclivado");
        assert!(
            aux_aud.valid_for_production && aux_aud.exposed_die_points == 0,
            "El espacio de cementado debe auto-optimizar el eje y cubrir 100% el incisivo (exp={}, msg={})",
            aux_aud.exposed_die_points,
            aux_aud.warning_message
        );
        let lib = ToothLibraryManager::new();
        let tooth_lib = lib.load_tooth_mesh("BodyParts3D_Humana_FDI", 11);
        let placed = CrownDesigner::place_library_tooth_world(
            &tooth_lib,
            &curve_96,
            aux_aud.effective_axis,
            &params,
            Some(&incisor),
            None,
        );
        let (crown, c_aud) = CrownDesigner::generate_crown_from_positioned_tooth(
            &incisor,
            &curve_96,
            &placed,
            Vec3::NEG_Z,
            &params,
            None,
        )
        .expect("corona incisivo proclivado debe generarse sin bloqueos");
        assert!(c_aud.valid_for_production && crown.inspect_cam().watertight);

        // 2) Muñón con socavado BILATERAL (forma de hongo 360° donde ningún eje lo quita por sí solo):
        //    El Bloqueo Virtual de Socavados (`remove_undercuts = true`) debe cubrir 100% sin bloquear!
        let mush_prof: &[(f32, f32)] = &[
            (0.0, 4.6),
            (2.2, 4.5),
            (3.45, 2.5), // barriga 360° más ancha que el cuello 2.95 mm
            (2.95, 0.0), // cuello cervical
            (4.8, -0.3),
        ];
        let mut mv = vec![Vec3::new(0.0, 0.0, 4.65)];
        let mut mt = Vec::new();
        for &(r, z) in &mush_prof[1..] {
            for k in 0..nt {
                let th = 2.0 * std::f32::consts::PI * k as f32 / nt as f32;
                mv.push(Vec3::new(r * th.cos(), r * th.sin(), z));
            }
        }
        for k in 0..nt {
            let nk = (k + 1) % nt;
            mt.push([0, 1 + k as u32, 1 + nk as u32]);
        }
        for ring in 0..(mush_prof.len() - 2) {
            let b0 = 1 + ring * nt;
            let b1 = 1 + (ring + 1) * nt;
            for k in 0..nt {
                let nk = (k + 1) % nt;
                mt.push([(b0 + k) as u32, (b1 + k) as u32, (b1 + nk) as u32]);
                mt.push([(b0 + k) as u32, (b1 + nk) as u32, (b0 + nk) as u32]);
            }
        }
        let mush_die = TriMeshData::new(mv, mt);
        let mush_margin: Vec<Vec3> = (0..96)
            .map(|k| {
                let th = 2.0 * std::f32::consts::PI * k as f32 / 96.0;
                Vec3::new(2.98 * th.cos(), 2.98 * th.sin(), 0.02)
            })
            .collect();
        let (_, _, m_aud) =
            CrownDesigner::generate_auxiliary_shells(&mush_die, &mush_margin, Vec3::Z, &params)
                .expect("bloqueo virtual sobre socavado bilateral");
        assert!(
            m_aud.valid_for_production && m_aud.exposed_die_points == 0 && m_aud.blockout_applied,
            "El bloqueo virtual debe cubrir 100% incluso un socavado bilateral 360° (valid={}, exp={}, bo={}, msg={})",
            m_aud.valid_for_production,
            m_aud.exposed_die_points,
            m_aud.blockout_applied,
            m_aud.warning_message
        );

        // 3) Curva en forma de 8 (auto-intersección) SÍ debe marcar `blocks_design = true`!
        let mut crossed_ctrl = ctrl_12.clone();
        crossed_ctrl.swap(2, 8);
        let cross_aud = CrownDesigner::audit_margin_nodes(
            &incisor,
            None,
            &crossed_ctrl,
            &curve_96,
            opt_ax,
            &params,
            false,
        );
        assert!(
            cross_aud.blocks_design && cross_aud.issues.contains(&MarginNodeIssue::SelfIntersection),
            "Una curva cruzada en 8 SÍ debe bloquear el diseño hasta ordenarla"
        );
    }

    #[test]
    fn test_razor_concave_incisor_zero_min_thickness_folds_and_zero_stalactites() {
        use apex_dentalcraft_lib::core::case_manager::DesignParameters;
        use apex_dentalcraft_lib::core::library_manager::ToothLibraryManager;
        use apex_dentalcraft_lib::geometry::crown::CrownDesigner;
        use apex_dentalcraft_lib::geometry::mesh::{MeshCollider, TriMeshData};
        use glam::Vec3;

        // Muñón incisivo en cincel extremo con concavidad palatina pronunciada (radio cóncavo ~0.3 mm,
        // mucho menor que min_thickness = 0.80 mm) que históricamente causaba pliegues "swallowtail"
        // y estalactitas naranjas penetrando el espacio de cementado.
        let nt = 72usize;
        let nr = 20usize;
        let mut verts = Vec::new();
        let mut tris = Vec::new();
        let surface_z = |x: f32, y: f32| -> f32 {
            // Borde incisal desplazado hacia vestibular (x = +1.35) + fosa cóncava palatina en x < 0.8
            let r_norm = ((x / 3.1).powi(2) + (y / 2.6).powi(2)).sqrt();
            if r_norm >= 1.0 {
                return -0.25 * (r_norm - 1.0);
            }
            let dome = 5.4 * (1.0 - r_norm.powi(2)).max(0.0).powf(0.55);
            // Esculpir una fosa cóncava palatina aguda en (-0.55, 0.0) con radio 0.65 mm
            let d_pal = ((x + 0.55).powi(2) + (y * 0.75).powi(2)).sqrt();
            let concavity = 1.95 * (-(d_pal / 0.85).powi(2)).exp() * (1.0 - r_norm).clamp(0.0, 1.0);
            // Cresta incisal fina en x = +1.15
            let d_inc = ((x - 1.15) / 0.55).powi(2) + (y / 2.1).powi(2);
            let ridge = 0.95 * (-d_inc).exp();
            (dome - concavity + ridge).max(0.0)
        };

        verts.push(Vec3::new(0.0, 0.0, surface_z(0.0, 0.0)));
        for ri in 1..=nr {
            let s = ri as f32 / nr as f32 * 1.45;
            for k in 0..nt {
                let th = 2.0 * std::f32::consts::PI * k as f32 / nt as f32;
                let x = 3.1 * s * th.cos();
                let y = 2.6 * s * th.sin();
                verts.push(Vec3::new(x, y, surface_z(x, y)));
            }
        }
        for k in 0..nt {
            let nk = (k + 1) % nt;
            tris.push([0, 1 + k as u32, 1 + nk as u32]);
        }
        for ri in 0..(nr - 1) {
            let b0 = 1 + ri * nt;
            let b1 = 1 + (ri + 1) * nt;
            for k in 0..nt {
                let nk = (k + 1) % nt;
                tris.push([(b0 + k) as u32, (b1 + k) as u32, (b1 + nk) as u32]);
                tris.push([(b0 + k) as u32, (b1 + nk) as u32, (b0 + nk) as u32]);
            }
        }
        let concave_incisor = TriMeshData::new(verts, tris);
        let margin: Vec<Vec3> = (0..96)
            .map(|k| {
                let th = 2.0 * std::f32::consts::PI * k as f32 / 96.0;
                let x = 3.12 * th.cos();
                let y = 2.62 * th.sin();
                Vec3::new(x, y, surface_z(x, y))
            })
            .collect();

        let mut params = DesignParameters::default();
        params.min_thickness_mm = 0.80; // Espesor alto (4Y/5Y) para máximo estrés de pliegue cóncavo

        let (cem, min_t, aud) =
            CrownDesigner::generate_auxiliary_shells(&concave_incisor, &margin, Vec3::Z, &params)
                .expect("aux shells sobre incisivo cóncavo extremo");
        assert!(
            aud.valid_for_production && aud.exposed_die_points == 0,
            "Cobertura 100% obligatoria (exp={}, msg={})",
            aud.exposed_die_points,
            aud.warning_message
        );

        let count_folds = |m: &TriMeshData| -> usize {
            let mut emap: std::collections::HashMap<(u32, u32), Vec<Vec3>> =
                std::collections::HashMap::new();
            for &[a, b, c] in &m.indices {
                let fnrm = (m.positions[b as usize] - m.positions[a as usize])
                    .cross(m.positions[c as usize] - m.positions[a as usize])
                    .normalize_or_zero();
                for (u, v) in [(a, b), (b, c), (c, a)] {
                    let key = if u < v { (u, v) } else { (v, u) };
                    emap.entry(key).or_default().push(fnrm);
                }
            }
            emap.values()
                .filter(|ns| ns.len() == 2 && ns[0].dot(ns[1]) < -0.15)
                .count()
        };
        assert_eq!(count_folds(&cem), 0, "Cero pliegues permitidos en espacio de cementado");
        assert_eq!(
            count_folds(&min_t),
            0,
            "Cero pliegues swallowtail permitidos en grosor mínimo sobre incisivo cóncavo"
        );

        // Ningún vértice del grosor mínimo puede quedar por dentro del cementado ni formar estalactitas!
        let n_v = cem.positions.len();
        let margin_start = n_v - 144;
        let mut cem_solid_v = cem.positions.clone();
        let mut cem_solid_f = cem.indices.clone();
        let bot_start = cem_solid_v.len() as u32;
        for &mp in &cem.positions[margin_start..] {
            cem_solid_v.push(Vec3::new(mp.x, mp.y, -4.0));
        }
        let bot_c = cem_solid_v.len() as u32;
        cem_solid_v.push(Vec3::new(0.0, 0.0, -4.0));
        for j in 0..144usize {
            let nj = (j + 1) % 144;
            let t0 = (margin_start + j) as u32;
            let t1 = (margin_start + nj) as u32;
            let b0 = bot_start + j as u32;
            let b1 = bot_start + nj as u32;
            cem_solid_f.push([t0, b0, b1]);
            cem_solid_f.push([t0, b1, t1]);
            cem_solid_f.push([bot_c, b1, b0]);
        }
        let mut cem_solid = TriMeshData::new(cem_solid_v, cem_solid_f);
        cem_solid.ensure_outward_winding_if_closed();
        let cem_solid_col = MeshCollider::from_mesh(&cem_solid).unwrap();
        let cem_col = MeshCollider::from_mesh(&cem).unwrap();

        let mut min_inside_cem = 0usize;
        let mut max_min_disp = 0.0_f32;
        for vi in 0..margin_start {
            let p = min_t.positions[vi];
            if cem_solid_col.contains_point_closed(p) {
                min_inside_cem += 1;
            }
            let d = p.distance(cem_col.project_point(p));
            max_min_disp = max_min_disp.max(d);
        }
        assert_eq!(
            min_inside_cem, 0,
            "CERO vértices del grosor mínimo pueden atravesar hacia adentro del espacio de cementado"
        );
        assert!(
            max_min_disp <= params.min_thickness_mm + 0.35,
            "Grosor mínimo sin picos ni estalactitas (max_disp={:.3} mm)",
            max_min_disp
        );

        // Corona final 100% watertight sobre el incisivo cóncavo
        let lib = ToothLibraryManager::new();
        let tooth_lib = lib.load_tooth_mesh("BodyParts3D_Humana_FDI", 11);
        let placed = CrownDesigner::place_library_tooth_world(
            &tooth_lib,
            &margin,
            aud.effective_axis,
            &params,
            Some(&concave_incisor),
            None,
        );
        let (crown, c_aud) = CrownDesigner::generate_crown_from_positioned_tooth(
            &concave_incisor,
            &margin,
            &placed,
            Vec3::Z,
            &params,
            None,
        )
        .expect("corona sobre incisivo cóncavo extremo");
        let cam = crown.inspect_cam();
        assert!(c_aud.valid_for_production && cam.watertight && cam.open_edges == 0 && cam.non_manifold_edges == 0);
    }

    #[test]
    fn test_solid_and_hollow_base_conflict_free_watertight_both_arches() {
        use apex_dentalcraft_lib::geometry::mesh_ops::{clip_by_plane, create_hollow_base_checked};

        let mandible = generate_parametric_demo_arch(false);
        let maxilla = generate_parametric_demo_arch(true);

        // Cortes múltiples consecutivos (plano centrado + segundo corte lateral)
        let mid_mand = mandible.center();
        let cut1 = clip_by_plane(&mandible, mid_mand - Vec3::Z * 4.0, Vec3::Z, false);
        assert!(!cut1.is_empty(), "Primer recorte del maxilar inferior debe conservar geometría");
        let cut2 = clip_by_plane(&cut1, mid_mand + Vec3::X * 26.0, -Vec3::X, false);
        assert!(!cut2.is_empty(), "Segundo recorte consecutivo debe permitir seguir recortando");

        // Zócalos sólidos sin triángulos conflictivos en maxilar inferior (-Z) y superior (+Z)
        for (label, arch, down) in [("mandible", &cut2, true), ("maxilla", &maxilla, false)] {
            let solid = create_solid_base(arch, 6.0, down);
            let au = solid.inspect_cam();
            assert!(
                au.valid && au.watertight && au.open_edges == 0 && au.non_manifold_edges == 0 && au.flipped_winding_edges == 0,
                "Zócalo sólido en {} debe ser 100% watertight sin aristas conflictivas (wt={}, open={}, nm={}, flip={})",
                label, au.watertight, au.open_edges, au.non_manifold_edges, au.flipped_winding_edges
            );
            assert!(au.volume_mm3 > 50.0, "Volumen positivo esperado en {}", label);
        }

        // Modelos Hollow imprimibles (paredes 1.0 mm, 1.5 mm, 2.0 mm, 3.0 mm) en ambas arcadas
        for (label, arch, wall, down) in [
            ("mandible_1mm", &mandible, 1.0_f32, true),
            ("mandible_2mm", &mandible, 2.0_f32, true),
            ("mandible_3mm", &mandible, 3.0_f32, true),
            ("maxilla_1_5mm", &maxilla, 1.5_f32, false),
        ] {
            let (hollow, ok, msg) = create_hollow_base_checked(arch, 6.0, wall, down);
            let au = hollow.inspect_cam();
            assert!(
                ok && au.valid && au.watertight && au.open_edges == 0 && au.non_manifold_edges == 0 && au.flipped_winding_edges == 0,
                "Hollow {} debe ser watertight y libre de conflictos ({}): wt={}, open={}, nm={}, flip={}",
                label, msg, au.watertight, au.open_edges, au.non_manifold_edges, au.flipped_winding_edges
            );
            assert!(au.volume_mm3 > 10.0, "Hollow {} debe tener volumen neto > 10 mm³ ({:.1})", label, au.volume_mm3);
        }
    }

    #[test]
    fn test_bridge_connectors_area_and_single_watertight_fused_mesh() {
        use apex_dentalcraft_lib::core::case_manager::{DentalCase, ToothRestorationUnit};
        use apex_dentalcraft_lib::geometry::connector::{
            build_connector_mesh, compute_connector_frame, compute_cross_section_area_mm2,
            compute_profile_dimensions_mm, fuse_bridge_watertight,
        };
        use std::collections::VecDeque;

        let mut case = DentalCase::default();
        case.prep_arch = "maxilla".to_string();
        case.insertion_axis = [0.0, 0.0, -1.0];
        case.tooth_number = 24;
        case.restorations.clear();
        case.restorations.insert(
            24,
            ToothRestorationUnit::from_material_preset(
                24,
                "Pilar de Puente",
                APEX_MATERIAL_PRESETS[0].name,
                "A2",
                vec![25],
            ),
        );
        case.restorations.insert(
            25,
            ToothRestorationUnit::from_material_preset(
                25,
                "Póntico de Puente",
                APEX_MATERIAL_PRESETS[0].name,
                "A2",
                vec![24, 26],
            ),
        );
        case.restorations.insert(
            26,
            ToothRestorationUnit::from_material_preset(
                26,
                "Pilar de Puente",
                APEX_MATERIAL_PRESETS[0].name,
                "A2",
                vec![25],
            ),
        );
        case.sync_bridge_connectors();

        assert_eq!(case.connected_pairs(), vec![(24, 25), (25, 26)]);
        assert_eq!(case.connected_bridge_groups(), vec![vec![24, 25, 26]]);
        assert_eq!(case.bridge_connectors.len(), 2);

        let lib = ToothLibraryManager::new();
        let center_mesh = |mut m: apex_dentalcraft_lib::geometry::mesh::TriMeshData, target_x: f32| {
            let c = m.center();
            let shift = Vec3::new(target_x - c.x, -c.y, -c.z);
            for p in &mut m.positions {
                *p += shift;
            }
            m
        };

        let m24_raw = lib.load_tooth_mesh("BodyParts3D_Humana_FDI", 24);
        let m25_raw = lib.load_tooth_mesh("BodyParts3D_Humana_FDI", 25);
        let m26_raw = lib.load_tooth_mesh("BodyParts3D_Humana_FDI", 26);

        let half_24 = {
            let (mn, mx) = m24_raw.bounds();
            0.5 * (mx.x - mn.x)
        };
        let half_25 = {
            let (mn, mx) = m25_raw.bounds();
            0.5 * (mx.x - mn.x)
        };
        let half_26 = {
            let (mn, mx) = m26_raw.bounds();
            0.5 * (mx.x - mn.x)
        };

        let m24 = center_mesh(m24_raw, -(half_25 + half_24 + 0.9));
        let m25 = center_mesh(m25_raw, 0.0);
        let m26 = center_mesh(m26_raw, half_25 + half_26 + 0.9);

        let ax = Vec3::from_array(case.insertion_axis);
        for cfg in &case.bridge_connectors {
            let area = compute_cross_section_area_mm2(cfg);
            let (h_og, w_vl) = compute_profile_dimensions_mm(cfg);
            assert!(area > 5.0 && h_og > 1.0 && w_vl > 1.0, "Conector con dimensiones válidas (area={:.2})", area);
            let (ma, mb) = if cfg.tooth_a == 24 { (&m24, &m25) } else { (&m25, &m26) };
            let fr = compute_connector_frame(ma, mb, ax, cfg);
            let tube = build_connector_mesh(&fr, cfg);
            assert!(!tube.is_empty(), "Malla de conector no puede estar vacía");
        }

        let conn_refs: Vec<_> = case.bridge_connectors.iter().map(|c| (c, ax)).collect();
        let units = [(24, m24, false), (25, m25, true), (26, m26, false)];
        let fused = fuse_bridge_watertight(&units, &conn_refs);
        let au = fused.inspect_cam();
        assert!(
            au.valid && au.watertight && au.open_edges == 0 && au.non_manifold_edges == 0,
            "Puente fusionado debe ser una malla 100% cerrada watertight (wt={}, open={}, nm={})",
            au.watertight, au.open_edges, au.non_manifold_edges
        );
        assert!(au.volume_mm3 > 80.0, "Volumen del puente fusionado debe superar 80 mm³ ({:.1})", au.volume_mm3);

        // Verificar que todas las piezas y conectores quedaron en UN SOLO componente conexo
        let mut adj: Vec<Vec<u32>> = vec![Vec::new(); fused.positions.len()];
        for &[a, b, c] in &fused.indices {
            adj[a as usize].extend_from_slice(&[b, c]);
            adj[b as usize].extend_from_slice(&[a, c]);
            adj[c as usize].extend_from_slice(&[a, b]);
        }
        let mut seen = vec![false; fused.positions.len()];
        let mut q = VecDeque::new();
        if !seen.is_empty() {
            seen[0] = true;
            q.push_back(0u32);
            while let Some(v) = q.pop_front() {
                for &nb in &adj[v as usize] {
                    if !seen[nb as usize] {
                        seen[nb as usize] = true;
                        q.push_back(nb);
                    }
                }
            }
        }
        let visited_count = seen.iter().filter(|&&s| s).count();
        assert_eq!(
            visited_count,
            fused.positions.len(),
            "El puente entero debe formar UNA SOLA malla conexa cerrada ({}/{})",
            visited_count,
            fused.positions.len()
        );
    }

    #[test]
    fn test_pontic_no_margin_required_and_clip_plane_visuals() {
        use apex_dentalcraft_lib::core::case_manager::{CaseManager, DentalCase, ToothRestorationUnit};
        use apex_dentalcraft_lib::geometry::mesh_ops::{build_clip_box_wireframe, build_clip_plane_visuals};
        use apex_dentalcraft_lib::ui::margin_panel::MarginPanelState;

        let u_pontic = ToothRestorationUnit::from_material_preset(
            25,
            "Póntico de Puente",
            APEX_MATERIAL_PRESETS[0].name,
            "A2",
            vec![24, 26],
        );
        let u_crown = ToothRestorationUnit::from_material_preset(
            24,
            "Pilar de Puente",
            APEX_MATERIAL_PRESETS[0].name,
            "A2",
            vec![25],
        );
        assert!(u_pontic.is_pontic(), "Póntico debe identificarse con is_pontic() = true");
        assert!(!u_crown.is_pontic(), "Pilar no es póntico");

        let tmp_root = std::env::temp_dir().join(format!("apex_test_pontic_margin_{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&tmp_root);
        let cm = CaseManager::new(Some(tmp_root.clone()));
        let mut case = DentalCase::default();
        case.case_id = "TEST_PONTIC_MARGIN".to_string();
        case.tooth_number = 25; // Seleccionado inicialmente en el póntico
        case.restorations.clear();
        case.restorations.insert(24, u_crown);
        case.restorations.insert(25, u_pontic);
        let _ = cm.save_case(&mut case);

        let mut margin_panel = MarginPanelState::default();
        margin_panel.load_case(&mut case, &cm);
        assert_eq!(
            case.tooth_number, 24,
            "Al abrir Módulo 3 sobre un póntico (#25), debe activar automáticamente el primer pilar (#24)"
        );
        let _ = std::fs::remove_dir_all(&tmp_root);

        let (plane_quad, arrow) = build_clip_plane_visuals(Vec3::new(0.0, 0.0, 2.0), Vec3::Z, Vec3::Z, 30.0);
        assert!(!plane_quad.is_empty() && !arrow.is_empty(), "Plano visual de corte y flecha deben generarse");
        let box_wire = build_clip_box_wireframe(Vec3::splat(-12.0), Vec3::splat(12.0));
        assert!(!box_wire.is_empty(), "Wireframe de caja de corte debe generarse");
    }

    #[test]
    fn test_hollow_on_existing_solid_base_and_visible_cavity() {
        use apex_dentalcraft_lib::geometry::mesh::TriMeshData;
        use apex_dentalcraft_lib::geometry::mesh_ops::create_hollow_base_checked;

        let flat_base_xy_area = |m: &TriMeshData| -> f32 {
            let (mn, _) = m.bounds();
            let mut area = 0.0_f32;
            for &[a, b, c] in &m.indices {
                let pa = m.positions[a as usize];
                let pb = m.positions[b as usize];
                let pc = m.positions[c as usize];
                if (pa.z - mn.z).abs() < 0.12 && (pb.z - mn.z).abs() < 0.12 && (pc.z - mn.z).abs() < 0.12 {
                    area += 0.5 * ((pb.x - pa.x) * (pc.y - pa.y) - (pb.y - pa.y) * (pc.x - pa.x)).abs();
                }
            }
            area
        };

        let mand = generate_parametric_demo_arch(false);
        let solid = create_solid_base(&mand, 6.0, true);
        let solid_au = solid.inspect_cam();
        let solid_area = flat_base_xy_area(&solid);
        assert!(solid_au.watertight && solid_area > 50.0);

        // Ahuecar un modelo que YA tiene zócalo sólido cerrado (debe retirar tapa inferior y abrir cavidad)
        let (hollow, ok, msg) = create_hollow_base_checked(&solid, 6.0, 2.0, true);
        let h_au = hollow.inspect_cam();
        let hollow_area = flat_base_xy_area(&hollow);
        assert!(
            ok && h_au.watertight && h_au.open_edges == 0 && h_au.non_manifold_edges == 0,
            "Hollow sobre zócalo sólido falló ({}): wt={}, open={}, nm={}",
            msg, h_au.watertight, h_au.open_edges, h_au.non_manifold_edges
        );
        assert!(
            h_au.volume_mm3 < solid_au.volume_mm3 * 0.85,
            "Hollow debe reducir el volumen neto (solid={:.1}, hollow={:.1})",
            solid_au.volume_mm3, h_au.volume_mm3
        );
        assert!(
            hollow_area < solid_area * 0.65,
            "Hollow debe abrir el área inferior para mostrar el ahuecado (solid_area={:.1}, hollow_area={:.1})",
            solid_area, hollow_area
        );

        let lib = ToothLibraryManager::new();
        if let Some(real_path) = lib.find_demo_asset("demo_mandible_prep36.stl") {
            if let Ok(real_mand) = TriMeshData::load_from_path(&real_path) {
                let (real_h, real_ok, real_msg) = create_hollow_base_checked(&real_mand, 6.0, 2.0, true);
                assert!(
                    real_ok && real_h.inspect_cam().volume_mm3 > 10.0,
                    "Hollow sobre escaneo clínico real debe aplicarse limpiamente: {}",
                    real_msg
                );
            }
        }
    }

    #[test]
    fn test_isotropic_library_insertion_and_default_apex_detailed() {
        let params = DesignParameters::default();
        assert_eq!(params.library_id, "LIBRERIA APEX DETAILED");

        let lib = ToothLibraryManager::new();
        let raw = lib.load_tooth_mesh(&params.library_id, 23);
        let (raw_mn, raw_mx) = raw.bounds();
        let raw_d = raw_mx - raw_mn;

        // Margen deliberadamente ovalado: la inserción inicial DEBE mantener escala pareja (isotrópica) sin aplastar
        let oval_margin: Vec<Vec3> = (0..32)
            .map(|i| {
                let th = 2.0 * std::f32::consts::PI * (i as f32) / 32.0;
                Vec3::new(4.8 * th.cos(), 3.0 * th.sin(), 0.0)
            })
            .collect();
        let placed = CrownDesigner::place_library_tooth_world(&raw, &oval_margin, Vec3::Z, &params, None, None);
        let (pl_mn, pl_mx) = placed.bounds();
        let pl_d = pl_mx - pl_mn;

        let raw_xy = raw_d.x / raw_d.y.max(1e-3);
        let pl_xy = pl_d.x / pl_d.y.max(1e-3);
        let raw_xz = raw_d.x / raw_d.z.max(1e-3);
        let pl_xz = pl_d.x / pl_d.z.max(1e-3);
        assert!(
            ((pl_xy - raw_xy).abs() / raw_xy) < 0.03,
            "Relación X/Y alterada al insertar librería (raw={:.3}, placed={:.3})",
            raw_xy, pl_xy
        );
        assert!(
            ((pl_xz - raw_xz).abs() / raw_xz) < 0.03,
            "Relación X/Z alterada al insertar librería (raw={:.3}, placed={:.3})",
            raw_xz, pl_xz
        );
    }

    #[test]
    fn test_3shape_3ring_connector_curved_bridge_zero_folds_and_sculpt_stage() {
        use apex_dentalcraft_lib::core::case_manager::{CaseManager, DentalCase, ToothRestorationUnit};
        use apex_dentalcraft_lib::geometry::connector::{
            build_connector_mesh, compute_connector_frame, fuse_bridge_watertight,
        };
        use apex_dentalcraft_lib::geometry::mesh::{MeshCollider, TriMeshData};
        use apex_dentalcraft_lib::ui::bridge_sculpt_panel::BridgeSculptPanelState;
        use apex_dentalcraft_lib::ui::export_panel::ExportPanelState;
        use std::collections::VecDeque;

        let tmp_root = std::env::temp_dir().join(format!("apex_test_bridge_sculpt_{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&tmp_root);
        let cm = CaseManager::new(Some(tmp_root.clone()));

        let mut case = DentalCase::default();
        case.case_id = "TEST_CURVED_BRIDGE".to_string();
        case.prep_arch = "maxilla".to_string();
        case.insertion_axis = [0.0, 0.0, -1.0];
        case.tooth_number = 23;
        case.restorations.clear();
        case.restorations.insert(
            23,
            ToothRestorationUnit::from_material_preset(23, "Pilar de Puente", APEX_MATERIAL_PRESETS[0].name, "A2", vec![24]),
        );
        case.restorations.insert(
            24,
            ToothRestorationUnit::from_material_preset(24, "Póntico de Puente", APEX_MATERIAL_PRESETS[0].name, "A2", vec![23, 25]),
        );
        case.restorations.insert(
            25,
            ToothRestorationUnit::from_material_preset(25, "Pilar de Puente", APEX_MATERIAL_PRESETS[0].name, "A2", vec![24]),
        );
        case.sync_bridge_connectors();
        assert!(case.has_bridge());

        let lib = ToothLibraryManager::new();
        let shift_mesh = |mut m: TriMeshData, target: Vec3| {
            let c = m.center();
            let d = target - c;
            for p in &mut m.positions {
                *p += d;
            }
            m
        };
        let m23_raw = lib.load_tooth_mesh(&case.design_params.library_id, 23);
        let m24_raw = lib.load_tooth_mesh(&case.design_params.library_id, 24);
        let m25_raw = lib.load_tooth_mesh(&case.design_params.library_id, 25);

        let w23 = { let (a, b) = m23_raw.bounds(); 0.5 * (b.x - a.x) };
        let w24 = { let (a, b) = m24_raw.bounds(); 0.5 * (b.x - a.x) };
        let w25 = { let (a, b) = m25_raw.bounds(); 0.5 * (b.x - a.x) };

        // Puente maxilar curvo con escalón de 5.0 mm en Z entre #23 y #25
        let m23 = shift_mesh(m23_raw, Vec3::new(-(w24 + w23 + 0.8), 0.0, 2.5));
        let m24 = shift_mesh(m24_raw, Vec3::new(0.0, 1.5, 0.0));
        let m25 = shift_mesh(m25_raw, Vec3::new(w24 + w25 + 0.8, 4.0, -2.5));

        let ax = Vec3::from_array(case.insertion_axis);
        let col23 = MeshCollider::from_mesh(&m23).unwrap();
        let fr0 = compute_connector_frame(&m23, &m24, ax, &case.bridge_connectors[0]);
        assert_eq!(fr0.ring_a_3d.len(), 48);
        assert_eq!(fr0.ring_c_3d.len(), 48);
        assert_eq!(fr0.ring_b_3d.len(), 48);
        let max_snap_err = fr0
            .ring_a_3d
            .iter()
            .map(|&p| col23.closest_point_and_dist(p).1)
            .fold(0.0_f32, f32::max);
        assert!(max_snap_err < 0.35, "Anillo A debe adherirse al diente #23 (err={:.3})", max_snap_err);

        // Editar nodos de Diente A y escala debe reconstruir limpiamente el conector
        case.bridge_connectors[0].attach_a_radii_mm[0] = 2.6;
        case.bridge_connectors[0].attach_a_scale = 1.15;
        let fr_edit = compute_connector_frame(&m23, &m24, ax, &case.bridge_connectors[0]);
        assert!(!build_connector_mesh(&fr_edit, &case.bridge_connectors[0]).is_empty());

        let conn_refs: Vec<_> = case.bridge_connectors.iter().map(|c| (c, ax)).collect();
        let units = [(23, m23.clone(), false), (24, m24.clone(), true), (25, m25.clone(), false)];
        let fused = fuse_bridge_watertight(&units, &conn_refs);
        let au = fused.inspect_cam();
        assert!(
            au.valid
                && au.watertight
                && au.open_edges == 0
                && au.non_manifold_edges == 0
                && au.flipped_winding_edges == 0
                && au.folded_edges <= 12,
            "Puente curvo debe fusionarse limpio sin errores (wt={}, open={}, nm={}, flip={}, fold={})",
            au.watertight, au.open_edges, au.non_manifold_edges, au.flipped_winding_edges, au.folded_edges
        );

        let mut adj: Vec<Vec<u32>> = vec![Vec::new(); fused.positions.len()];
        for &[a, b, c] in &fused.indices {
            adj[a as usize].extend_from_slice(&[b, c]);
            adj[b as usize].extend_from_slice(&[a, c]);
            adj[c as usize].extend_from_slice(&[a, b]);
        }
        let mut seen = vec![false; fused.positions.len()];
        let mut q = VecDeque::new();
        seen[0] = true;
        q.push_back(0u32);
        while let Some(v) = q.pop_front() {
            for &nb in &adj[v as usize] {
                if !seen[nb as usize] {
                    seen[nb as usize] = true;
                    q.push_back(nb);
                }
            }
        }
        assert_eq!(seen.iter().filter(|&&s| s).count(), fused.positions.len());

        // Probar módulo Puente Final -> Esculpido libre -> Exportación
        let design_dir = cm.get_case_dir(&case.case_id).join("design");
        let _ = std::fs::create_dir_all(&design_dir);
        let _ = m23.save_stl_binary(&design_dir.join("crown_23_designed.stl"));
        let _ = m24.save_stl_binary(&design_dir.join("crown_24_designed.stl"));
        let _ = m25.save_stl_binary(&design_dir.join("crown_25_designed.stl"));
        case.restorations.get_mut(&23).unwrap().crown_design_path = Some("design/crown_23_designed.stl".to_string());
        case.restorations.get_mut(&24).unwrap().crown_design_path = Some("design/crown_24_designed.stl".to_string());
        case.restorations.get_mut(&25).unwrap().crown_design_path = Some("design/crown_25_designed.stl".to_string());
        let _ = cm.save_case(&mut case);

        let mut bs = BridgeSculptPanelState::default();
        bs.load_case(&case, &cm, false);
        assert_eq!(bs.bridges.len(), 1);
        let orig_vol = bs.bridges[0].audit.volume_mm3;

        let pontic_pt = bs.bridges[0]
            .mesh
            .positions
            .iter()
            .zip(bs.bridges[0].mesh.is_external.iter())
            .find(|(p, ext)| **ext && p.x.abs() < 2.0)
            .map(|(&p, _)| p)
            .unwrap_or_else(|| bs.bridges[0].mesh.center());
        CrownSculptor::apply_brush_stroke(
            &mut bs.bridges[0].mesh,
            pontic_pt,
            SculptMode::Add,
            2.2,
            0.45,
            0.6,
            None,
        );
        bs.save_all_bridges(&case, &cm);

        let mut exp = ExportPanelState::default();
        exp.load_case(&case, &cm);
        assert_eq!(exp.export_pieces.len(), 1);
        assert!(exp.export_pieces[0].is_bridge);
        assert!(exp.export_pieces[0].audit.watertight);
        assert!(
            exp.export_pieces[0].audit.volume_mm3 > orig_vol + 0.2,
            "Exportar debe conservar el esculpido final del puente (orig={:.2}, exp={:.2})",
            orig_vol, exp.export_pieces[0].audit.volume_mm3
        );
        let _ = std::fs::remove_dir_all(&tmp_root);
    }

    #[test]
    fn test_free_connector_nodes_and_interproximal_contact_bridge_union() {
        use apex_dentalcraft_lib::core::case_manager::BridgeConnectorConfig;
        use apex_dentalcraft_lib::core::library_manager::ToothLibraryManager;
        use apex_dentalcraft_lib::geometry::connector::{
            build_connector_mesh, compute_connector_frame, fuse_bridge_watertight,
        };
        use apex_dentalcraft_lib::geometry::mesh::MeshCollider;
        use std::collections::VecDeque;

        let lib = ToothLibraryManager::new();
        let center_mesh = |mut m: apex_dentalcraft_lib::geometry::mesh::TriMeshData, target_x: f32| {
            let c = m.center();
            let shift = Vec3::new(target_x - c.x, -c.y, -c.z);
            for p in &mut m.positions {
                *p += shift;
            }
            m
        };

        let m23_raw = lib.load_tooth_mesh("BodyParts3D_Humana_FDI", 23);
        let m24_raw = lib.load_tooth_mesh("BodyParts3D_Humana_FDI", 24);
        let m25_raw = lib.load_tooth_mesh("BodyParts3D_Humana_FDI", 25);

        let w23 = {
            let (mn, mx) = m23_raw.bounds();
            mx.x - mn.x
        };
        let w24 = {
            let (mn, mx) = m24_raw.bounds();
            mx.x - mn.x
        };
        let half_25 = {
            let (mn, mx) = m25_raw.bounds();
            0.5 * (mx.x - mn.x)
        };

        // #24 centrado en X=0; #23 haciendo contacto interproximal con #24 (solape 0.35 mm, SIN conector);
        // #25 separado +0.9 mm para ser unido por conector 24-25 con nodos libres.
        let mut m24 = center_mesh(m24_raw, 0.0);
        let (mn24, mx24) = m24.bounds();
        let mut m23 = center_mesh(m23_raw, mn24.x - 0.5 * w23 + 0.35);
        let (mn23, mx23) = m23.bounds();
        assert!(mx23.x >= mn24.x + 0.15, "23 y 24 deben tocarse en contacto interproximal");

        let m25 = center_mesh(m25_raw, mx24.x + half_25 + 0.9);
        assert!(w24 > 1.0);

        // Marcar zona basal de #23 y #24 como intaglio/margen protegido (!is_external)
        let mut intaglio_samples = Vec::new();
        let cut_z23 = mn23.z + 1.2;
        for (i, p) in m23.positions.iter().enumerate() {
            if p.z < cut_z23 {
                m23.is_external[i] = false;
                if p.z < mn23.z + 0.8 && intaglio_samples.len() < 8 {
                    intaglio_samples.push(*p);
                }
            }
        }
        assert!(!intaglio_samples.is_empty());
        let cut_z24 = mn24.z + 1.2;
        for (i, p) in m24.positions.iter().enumerate() {
            if p.z < cut_z24 {
                m24.is_external[i] = false;
            }
        }

        // 1) Probar movimiento libre de anclaje y nodos 3D/2D del conector 24-25 sobre superficies curvas
        let ax = Vec3::Z;
        let mut cfg = BridgeConnectorConfig::for_pair_and_area(24, 25, 9.0);
        cfg.ensure_free_profile_nodes_initialized();
        cfg.ensure_free_attach_nodes_initialized();
        let fr_base = compute_connector_frame(&m24, &m25, ax, &cfg);

        cfg.attach_a_offset = [0.25, 0.65, 0.85];
        cfg.attach_a_nodes_local[0][1] += 0.9;
        cfg.attach_a_nodes_local[0][2] += 0.55;
        cfg.profile_nodes_2d[2][0] += 0.45;
        let fr_moved = compute_connector_frame(&m24, &m25, ax, &cfg);

        assert!(
            (fr_moved.anchor_a - fr_base.anchor_a).length() > 0.35,
            "El anclaje libre del conector debe desplazarse sobre el diente sin regresar al punto fijo (delta={:.3})",
            (fr_moved.anchor_a - fr_base.anchor_a).length()
        );
        assert!(
            (fr_moved.ring_a_nodes[0] - fr_base.ring_a_nodes[0]).length() > 0.40,
            "Los nodos del anillo sobre el diente deben moverse libremente (delta={:.3})",
            (fr_moved.ring_a_nodes[0] - fr_base.ring_a_nodes[0]).length()
        );
        let col24 = MeshCollider::from_mesh(&m24).expect("collider m24");
        let (_, dist_on_surf, _) = col24.closest_point_dist_and_tri_normal(fr_moved.ring_a_nodes[0]);
        assert!(
            dist_on_surf < 0.25,
            "El nodo movido debe proyectarse sobre la superficie del diente (dist={:.3})",
            dist_on_surf
        );
        let tube = build_connector_mesh(&fr_moved, &cfg);
        assert!(!tube.is_empty());

        // 2) Fusionar #23-#24 (contacto interproximal) + #24-#25 (conector) en UNA sola malla cerrada watertight
        let units = [(23, m23, false), (24, m24, false), (25, m25, true)];
        let fused = fuse_bridge_watertight(&units, &[(&cfg, ax)]);
        let au = fused.inspect_cam();
        assert!(
            au.valid && au.watertight && au.open_edges == 0 && au.non_manifold_edges == 0,
            "Puente unido por conector + contacto interproximal debe ser watertight (wt={}, open={}, nm={})",
            au.watertight,
            au.open_edges,
            au.non_manifold_edges
        );

        let mut adj: Vec<Vec<u32>> = vec![Vec::new(); fused.positions.len()];
        for &[a, b, c] in &fused.indices {
            adj[a as usize].extend_from_slice(&[b, c]);
            adj[b as usize].extend_from_slice(&[a, c]);
            adj[c as usize].extend_from_slice(&[a, b]);
        }
        let mut seen = vec![false; fused.positions.len()];
        let mut q = VecDeque::new();
        seen[0] = true;
        q.push_back(0);
        while let Some(v) = q.pop_front() {
            for &nb in &adj[v as usize] {
                if !seen[nb as usize] {
                    seen[nb as usize] = true;
                    q.push_back(nb);
                }
            }
        }
        assert_eq!(
            seen.iter().filter(|&&s| s).count(),
            fused.positions.len(),
            "Las 3 piezas (contacto interproximal + conector) deben quedar en 1 sola pieza conexa"
        );

        assert!(
            fused.is_external.iter().any(|&e| !e),
            "La malla fusionada debe conservar vértices internos/margen marcados como !is_external"
        );
        let fused_col = MeshCollider::from_mesh(&fused).expect("fused collider");
        for sample in intaglio_samples {
            let surf_d = fused_col.closest_point_and_dist(sample).1;
            assert!(
                surf_d < 0.035,
                "El intaglio/margen protegido del pilar debe conservarse intacto tras unir el puente (surf_d={:.4})",
                surf_d
            );
        }
    }

    #[test]
    fn test_smart_adapt_pontic_to_gingiva_reduction_and_seating() {
        use apex_dentalcraft_lib::geometry::margin::build_uv_sphere;
        use apex_dentalcraft_lib::geometry::smart_tools::SmartToolsEngine;

        // Encía convexa simulada cuyo punto crestal superior está exactamente en Z = 0.0 mm
        let gingiva = build_uv_sphere(Vec3::new(0.0, 0.0, -20.0), 20.0, 28, 36);

        // Caso A: Póntico penetrando la encía hasta Z = -1.2 mm -> debe reducirse hacia Z ≈ 0
        let pontic_penetrating = build_uv_sphere(Vec3::new(0.0, 0.0, 1.0), 2.2, 24, 32);
        let (mn_pen_before, mx_pen_before) = pontic_penetrating.bounds();
        assert!(mn_pen_before.z < -1.0);
        let reduced = SmartToolsEngine::smart_adapt_pontic_to_gingiva(
            &pontic_penetrating,
            &gingiva,
            Vec3::Z,
            0.0,
            None,
        );
        let (mn_pen_after, mx_pen_after) = reduced.bounds();
        assert!(
            mn_pen_after.z > -0.25,
            "La reducción gingival debe eliminar la penetración del póntico en la encía (antes={:.3}, después={:.3})",
            mn_pen_before.z,
            mn_pen_after.z
        );
        assert!(
            (mx_pen_after.z - mx_pen_before.z).abs() < 0.10,
            "La cara oclusal superior del póntico no debe deformarse al adaptar la base"
        );

        // Caso B: Póntico flotando +1.0 mm sobre la encía -> debe bajar y adaptarse sobre la cresta
        let pontic_floating = build_uv_sphere(Vec3::new(0.0, 0.0, 3.2), 2.2, 24, 32);
        let (mn_flt_before, _) = pontic_floating.bounds();
        assert!(mn_flt_before.z > 0.9);
        let seated = SmartToolsEngine::smart_adapt_pontic_to_gingiva(
            &pontic_floating,
            &gingiva,
            Vec3::Z,
            0.0,
            None,
        );
        let (mn_flt_after, _) = seated.bounds();
        assert!(
            mn_flt_after.z < 0.28,
            "La adaptación gingival debe descender la base flotante hasta contactar la encía (antes={:.3}, después={:.3})",
            mn_flt_before.z,
            mn_flt_after.z
        );

        // Caso C: Zona de pilar protegida fuera de `pontic_zones` debe permanecer inmóvil
        let mut combined = pontic_penetrating.clone();
        let abutment_start = combined.positions.len();
        let abutment = build_uv_sphere(Vec3::new(9.5, 0.0, 0.8), 2.0, 16, 20);
        combined.merge_in_place(&abutment);
        let zones = [(Vec3::ZERO, 3.8)];
        let adapted_zone = SmartToolsEngine::smart_adapt_pontic_to_gingiva(
            &combined,
            &gingiva,
            Vec3::Z,
            0.0,
            Some(&zones),
        );
        let max_abutment_shift = adapted_zone.positions[abutment_start..]
            .iter()
            .zip(combined.positions[abutment_start..].iter())
            .map(|(&a, &b)| a.distance(b))
            .fold(0.0f32, f32::max);
        assert!(
            max_abutment_shift < 1e-5,
            "Los pilares fuera de pontic_zones no deben modificarse al adaptar el póntico (shift={:.6})",
            max_abutment_shift
        );
    }

    #[test]
    fn test_multilingual_settings_es_en_pt_roundtrip() {
        use apex_dentalcraft_lib::core::i18n::{
            tr_fdi_tooth_name_for, tr_for, tr_material_name_for, tr_process_type_for, AppLanguage,
        };
        use apex_dentalcraft_lib::core::settings::AppSettings;

        // Backward compatibility: legacy settings JSON without a `language` field defaults to English
        let legacy: AppSettings = serde_json::from_str("{}").expect("deserialize default settings");
        assert_eq!(legacy.language, AppLanguage::En);

        for lang in [AppLanguage::Es, AppLanguage::En, AppLanguage::Pt] {
            let mut s = AppSettings::default();
            s.language = lang;
            let json = serde_json::to_string(&s).expect("serialize settings");
            let back: AppSettings = serde_json::from_str(&json).expect("deserialize settings");
            assert_eq!(back.language, lang);
        }

        // Terminología clínica distinta y precisa en ES / EN / PT sin mutar el idioma global
        assert_eq!(tr_for(AppLanguage::Es, "Corona", "Crown", "Coroa"), "Corona");
        assert_eq!(tr_for(AppLanguage::En, "Corona", "Crown", "Coroa"), "Crown");
        assert_eq!(tr_for(AppLanguage::Pt, "Corona", "Crown", "Coroa"), "Coroa");

        assert_eq!(tr_process_type_for(AppLanguage::En, "Corona Individual"), "Single Crown");
        assert_eq!(tr_process_type_for(AppLanguage::Pt, "Corona Individual"), "Coroa Unitária");
        assert_eq!(tr_process_type_for(AppLanguage::En, "Póntico de Puente"), "Bridge Pontic");
        assert_eq!(tr_process_type_for(AppLanguage::Pt, "Póntico de Puente"), "Pôntico de Ponte");

        assert_ne!(
            tr_material_name_for(
                AppLanguage::Es,
                "Zirconia Translúcida Multicapa (4Y/5Y-PSZ Estética)"
            ),
            tr_material_name_for(
                AppLanguage::En,
                "Zirconia Translúcida Multicapa (4Y/5Y-PSZ Estética)"
            )
        );
        assert_ne!(
            tr_material_name_for(
                AppLanguage::Pt,
                "Zirconia Translúcida Multicapa (4Y/5Y-PSZ Estética)"
            ),
            tr_material_name_for(
                AppLanguage::En,
                "Zirconia Translúcida Multicapa (4Y/5Y-PSZ Estética)"
            )
        );

        assert_eq!(tr_fdi_tooth_name_for(AppLanguage::En, 11), "Central Incisor");
        assert_eq!(tr_fdi_tooth_name_for(AppLanguage::Pt, 24), "1º Pré-Molar");
    }

    #[test]
    fn test_crisp_topological_contact_bridge_and_fast_transform_sculpt() {
        use apex_dentalcraft_lib::core::case_manager::BridgeConnectorConfig;
        use apex_dentalcraft_lib::core::library_manager::ToothLibraryManager;
        use apex_dentalcraft_lib::geometry::connector::fuse_bridge_watertight;
        use apex_dentalcraft_lib::geometry::mesh::MeshCollider;
        use apex_dentalcraft_lib::geometry::sculpt::{
            nearest_intaglio_dist_and_pt, CrownSculptor, PointSpatialGrid, SculptMode,
        };
        use std::collections::VecDeque;

        let lib = ToothLibraryManager::new();
        let center_mesh = |mut m: apex_dentalcraft_lib::geometry::mesh::TriMeshData, target_x: f32| {
            let c = m.center();
            let shift = Vec3::new(target_x - c.x, -c.y, -c.z);
            for p in &mut m.positions {
                *p += shift;
            }
            m
        };

        let m23_raw = lib.load_tooth_mesh("BodyParts3D_Humana_FDI", 23);
        let m24_raw = lib.load_tooth_mesh("BodyParts3D_Humana_FDI", 24);
        let m25_raw = lib.load_tooth_mesh("BodyParts3D_Humana_FDI", 25);
        let w23 = {
            let (mn, mx) = m23_raw.bounds();
            mx.x - mn.x
        };
        let half_25 = {
            let (mn, mx) = m25_raw.bounds();
            0.5 * (mx.x - mn.x)
        };

        let mut m24 = center_mesh(m24_raw, 0.0);
        let (mn24, mx24) = m24.bounds();
        let mut m23 = center_mesh(m23_raw, mn24.x - 0.5 * w23 + 0.35);
        let (mn23, mx23) = m23.bounds();
        let m25 = center_mesh(m25_raw, mx24.x + half_25 + 0.9);
        let (mn25, mx25) = m25.bounds();

        // 1) Exactitud de PointSpatialGrid frente a búsqueda fuerza bruta
        let grid = PointSpatialGrid::from_points(&m24.positions, 0.65);
        let probes = [
            m24.center(),
            m24.center() + Vec3::new(1.1, -0.7, 2.3),
            m24.center() + Vec3::new(18.0, -11.0, 9.0),
        ];
        for &q in &probes {
            let (dg, pg) = grid.nearest(q);
            let (db, pb) = nearest_intaglio_dist_and_pt(&m24.positions, q);
            assert!(
                (dg - db).abs() < 1e-4 && pg.distance(pb) < 1e-4,
                "PointSpatialGrid::nearest debe coincidir con búsqueda exacta (dg={:.5}, db={:.5})",
                dg,
                db
            );
        }
        assert!(grid.nearest_within(m24.positions[0] + Vec3::new(0.1, 0.0, 0.0), 0.5).is_some());
        assert!(grid.nearest_within(m24.center() + Vec3::new(25.0, 0.0, 0.0), 0.5).is_none());

        // Marcar banda basal de #23 y #24 como intaglio/margen bloqueado
        let mut intaglio_samples = Vec::new();
        for (i, p) in m23.positions.iter().enumerate() {
            if p.z < mn23.z + 1.2 {
                m23.is_external[i] = false;
                if p.z < mn23.z + 0.8 && intaglio_samples.len() < 8 {
                    intaglio_samples.push(*p);
                }
            }
        }
        for (i, p) in m24.positions.iter().enumerate() {
            if p.z < mn24.z + 1.2 {
                m24.is_external[i] = false;
            }
        }

        // 2) Sesiones rápidas TransformSession, MorphSession y SculptStrokeCache con intaglio inmóvil
        let xform_sess = CrownSculptor::prepare_transform_session(
            &m24,
            Vec3::Z,
            true,
            Vec3::X,
            Vec3::Y,
            Vec3::NEG_Z,
        );
        let xformed = CrownSculptor::apply_transform_session(
            &xform_sess,
            Vec3::new(0.25, -0.15, 0.35),
            None,
            Vec3::splat(1.04),
            None,
            0.60,
        );
        assert!(xformed.inspect_cam().watertight);
        let top_24 = m24
            .positions
            .iter()
            .copied()
            .max_by(|a, b| a.z.partial_cmp(&b.z).unwrap_or(std::cmp::Ordering::Equal))
            .unwrap();
        let morph_sess = CrownSculptor::prepare_morph_session(&xformed, top_24, 2.4);
        let morphed = CrownSculptor::apply_morph_drag(&morph_sess, Vec3::new(0.0, 0.0, 0.32), 0.60, false);
        assert!(morphed.inspect_cam().watertight);
        let mut sculpted = morphed.clone();
        let sc_cache = CrownSculptor::prepare_sculpt_cache(&sculpted);
        CrownSculptor::apply_brush_stroke_cached(&mut sculpted, top_24, SculptMode::Add, 1.8, 0.18, 0.60, &sc_cache);
        CrownSculptor::apply_brush_stroke_cached(&mut sculpted, top_24, SculptMode::Smooth, 1.8, 0.15, 0.60, &sc_cache);
        assert!(sculpted.inspect_cam().watertight);
        for (i, &ext) in m24.is_external.iter().enumerate() {
            if !ext {
                assert!(
                    sculpted.positions[i].distance(m24.positions[i]) < 1e-5,
                    "El intaglio bloqueado no debe moverse durante Transform/Morph/Sculpt"
                );
            }
        }

        // 3) Fusión topológica nítida (0 voxelización) con contacto #23-#24 + conector #24-#25
        let cfg = BridgeConnectorConfig::for_pair_and_area(24, 25, 9.0);
        let input_tris = m23.indices.len() + m24.indices.len() + m25.indices.len();
        let crown_wall_samples: Vec<Vec3> = m23
            .positions
            .iter()
            .filter(|&&p| p.z > mx23.z - 0.8 && p.x < mx23.x - 1.5)
            .take(5)
            .chain(
                m25.positions
                    .iter()
                    .filter(|&&p| p.z > mx25.z - 0.8 && p.x > mn25.x + 1.5)
                    .take(5),
            )
            .copied()
            .collect();
        assert!(!crown_wall_samples.is_empty());

        let fused = fuse_bridge_watertight(&[(23, m23, false), (24, m24, false), (25, m25, true)], &[(&cfg, Vec3::Z)]);
        let au = fused.inspect_cam();
        assert!(
            au.valid && au.watertight && au.open_edges == 0 && au.non_manifold_edges == 0,
            "Puente topológico nítido debe ser watertight 2-manifold (wt={}, open={}, nm={})",
            au.watertight,
            au.open_edges,
            au.non_manifold_edges
        );
        assert!(
            fused.indices.len() <= (input_tris * 5) / 2,
            "La unión topológica nítida no debe multiplicar triángulos por voxelización ({} vs input {})",
            fused.indices.len(),
            input_tris
        );

        // Un solo componente conexo
        let mut adj: Vec<Vec<u32>> = vec![Vec::new(); fused.positions.len()];
        for &[a, b, c] in &fused.indices {
            adj[a as usize].extend_from_slice(&[b, c]);
            adj[b as usize].extend_from_slice(&[a, c]);
            adj[c as usize].extend_from_slice(&[a, b]);
        }
        let mut seen = vec![false; fused.positions.len()];
        let mut q = VecDeque::new();
        seen[0] = true;
        q.push_back(0);
        while let Some(v) = q.pop_front() {
            for &nb in &adj[v as usize] {
                if !seen[nb as usize] {
                    seen[nb as usize] = true;
                    q.push_back(nb);
                }
            }
        }
        assert_eq!(seen.iter().filter(|&&s| s).count(), fused.positions.len());

        let fused_col = MeshCollider::from_mesh(&fused).expect("fused collider");
        for p in crown_wall_samples {
            let d = fused_col.closest_point_and_dist(p).1;
            assert!(
                d < 0.01,
                "Las paredes anatómicas originales fuera de costuras deben permanecer intactas sin pixelación (d={:.5})",
                d
            );
        }
        for p in intaglio_samples {
            let d = fused_col.closest_point_and_dist(p).1;
            assert!(
                d < 0.005,
                "El intaglio/margen protegido debe conservar fidelidad sub-5 micras (d={:.5})",
                d
            );
        }
    }

    #[test]
    fn test_export_casos_scene_and_reopen_roundtrip() {
        use apex_dentalcraft_lib::core::case_manager::{
            CaseManager, ToothRestorationUnit, APEX_MATERIAL_PRESETS,
        };
        use apex_dentalcraft_lib::core::scene_file::{
            self, APEX_SCENE_EXTENSION, APEX_SCENE_FORMAT_ID, APEX_SCENE_FORMAT_VERSION,
        };
        use apex_dentalcraft_lib::geometry::mesh::TriMeshData;
        use apex_dentalcraft_lib::ui::export_panel::ExportPanelState;
        use apex_dentalcraft_lib::ui::stl_viewer_panel::{SceneRole, StlViewerPanelState};

        let tmp_root = std::env::temp_dir().join(format!(
            "apex_scene_export_roundtrip_{}",
            std::process::id()
        ));
        let _ = std::fs::remove_dir_all(&tmp_root);
        let ws1 = tmp_root.join("ws1");
        let ws2 = tmp_root.join("ws2");
        let program_casos = tmp_root.join("Programa").join("Casos");

        let cm1 = CaseManager::new(Some(ws1.clone()));
        let mut units = BTreeMap::new();
        units.insert(
            36,
            ToothRestorationUnit::from_material_preset(
                36,
                "Corona Individual",
                APEX_MATERIAL_PRESETS[0].name,
                "A2",
                Vec::new(),
            ),
        );
        let mut case = cm1
            .create_case(
                "Paciente Escena Export",
                "Dr. Prueba",
                36,
                APEX_MATERIAL_PRESETS[0].name,
                "A2",
                "Prueba exportación Casos + .apexscene + STLs",
                units,
            )
            .expect("crear caso test");

        // Importar mandíbula (muñón/trabajo) y maxilar (antagonista)
        let mand = generate_parametric_demo_arch(false);
        let max = generate_parametric_demo_arch(true);
        let src_dir = tmp_root.join("inputs");
        std::fs::create_dir_all(&src_dir).expect("crear inputs");
        let mand_in = src_dir.join("mand.stl");
        let max_in = src_dir.join("max.stl");
        mand.save_stl_binary(&mand_in).expect("save mand");
        max.save_stl_binary(&max_in).expect("save max");
        cm1.import_scan(&mut case, "mandible", &mand_in)
            .expect("import mand");
        cm1.import_scan(&mut case, "maxilla", &max_in)
            .expect("import max");

        // Guardar margen y corona diseñada
        let margin: Vec<[f32; 3]> = (0..24)
            .map(|k| {
                let th = 2.0 * std::f32::consts::PI * (k as f32) / 24.0;
                [3.5 * th.cos(), 3.5 * th.sin(), 1.2]
            })
            .collect();
        case.margin_control_points = margin.iter().step_by(2).copied().collect();
        case.margin_points = margin;

        let crown_mesh = ToothLibraryManager::generate_procedural_tooth_crown(36);
        let crown_rel = "design/crown_fdi_36.stl".to_string();
        let crown_abs = cm1.get_case_dir(&case.case_id).join(&crown_rel);
        crown_mesh
            .save_stl_binary(&crown_abs)
            .expect("save crown stl");
        case.crown_design_path = Some(crown_rel.clone());
        if let Some(u) = case.restorations.get_mut(&36) {
            u.crown_design_path = Some(crown_rel);
        }
        cm1.save_case(&mut case).expect("save case");

        // Cargar en Módulo 6 y exportar en Carpeta Casos
        let mut exp = ExportPanelState::default();
        exp.load_case(&case, &cm1);
        assert_eq!(exp.export_pieces.len(), 1);
        let target_dir = scene_file::case_export_dir_in(&program_casos, &case);
        let exported_primary = exp
            .export_bundle_to_dir(&mut case, &cm1, &target_dir)
            .expect("exportar bundle en Casos");

        assert_eq!(
            exported_primary.extension().and_then(|s| s.to_str()),
            Some(APEX_SCENE_EXTENSION),
            "La exportación primaria debe generar .apexscene"
        );
        assert!(exported_primary.is_file());

        let scene = scene_file::load_apex_scene(&exported_primary).expect("cargar .apexscene");
        assert_eq!(scene.format, APEX_SCENE_FORMAT_ID);
        assert_eq!(scene.version, APEX_SCENE_FORMAT_VERSION);
        assert_eq!(scene.restorations.len(), 1);

        let rest_stl = target_dir.join(&scene.restorations[0].file_name);
        let die_stl = target_dir.join(scene.models.die_model.as_deref().expect("die_model"));
        let ant_stl = target_dir.join(
            scene
                .models
                .antagonist_model
                .as_deref()
                .expect("antagonist_model"),
        );
        assert!(rest_stl.is_file(), "Debe existir STL de restauración");
        assert!(die_stl.is_file(), "Debe existir STL de muñón/modelo de trabajo");
        assert!(ant_stl.is_file(), "Debe existir STL de antagonista");
        assert!(!TriMeshData::load_from_path(&rest_stl).unwrap().is_empty());
        assert!(!TriMeshData::load_from_path(&die_stl).unwrap().is_empty());
        assert!(!TriMeshData::load_from_path(&ant_stl).unwrap().is_empty());

        let report_txt = std::fs::read_dir(&target_dir)
            .unwrap()
            .flatten()
            .map(|e| e.path())
            .find(|p| p.extension().and_then(|s| s.to_str()) == Some("txt"))
            .expect("Debe existir ficha .txt");
        let report_body = std::fs::read_to_string(&report_txt).unwrap();
        assert!(!report_body.contains("100% Nativo Rust"));

        // Reabrir en un workspace limpio (simulando otro equipo/sesión)
        let cm2 = CaseManager::new(Some(ws2));
        let (reopened_case, _) =
            scene_file::import_scene_into_case_manager(&exported_primary, &cm2)
                .expect("reabrir escena en CaseManager");
        assert_eq!(reopened_case.patient_name, "Paciente Escena Export");
        assert_eq!(reopened_case.tooth_number, 36);
        assert_eq!(reopened_case.margin_points.len(), 24);
        assert!(cm2
            .get_active_scan_path(&reopened_case, "mandible")
            .is_some());
        assert!(cm2
            .get_active_scan_path(&reopened_case, "maxilla")
            .is_some());
        assert!(cm2.get_crown_design_path(&reopened_case, Some(36)).is_file());

        // Reabrir directamente en el Visor 3D
        let mut viewer = StlViewerPanelState::default();
        let _msg = viewer.open_apex_scene(&exported_primary);
        assert_eq!(viewer.objects.len(), 3);
        assert!(viewer
            .objects
            .iter()
            .any(|o| o.role == SceneRole::Restoration));
        assert!(viewer.objects.iter().any(|o| o.role == SceneRole::ModelDie));
        assert!(viewer
            .objects
            .iter()
            .any(|o| o.role == SceneRole::Antagonist));
        assert_eq!(viewer.margin_curve.len(), 24);

        let _ = std::fs::remove_dir_all(&tmp_root);
    }
}
