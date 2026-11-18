//! `Engine::Solver3`: the facade's route to the 3D solver on the base-wake grid. Structure and plumbing on the CPU with tiny grids and iteration caps (the
//! numbers are the gate harness's: `wedge_gates.rs` through the facade, and the ignored tests at the end).

use bowshock_core::{
    AftSpec, Analysis, Atmosphere, Backend, BackendUsed, BodySpec, Condition, Engine, Error, FinProfile, FinSpec, FlightConditions, NoseShape, NoseSpec,
    Quality, Resolution, StopReason, Symmetry,
};
use std::ops::ControlFlow;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::Arc;

const D: f64 = 0.04;

fn body() -> BodySpec {
    let nose_len = 0.5 * D / 10f64.to_radians().tan();
    BodySpec { nose: NoseSpec { shape: NoseShape::Cone, length: nose_len, diameter: D }, aft: vec![AftSpec::Tube { length: 10.0 * D - nose_len }] }
}

fn fins() -> FinSpec {
    FinSpec {
        count: 4,
        root_chord: D,
        tip_chord: D,
        span: D,
        sweep: 0.0,
        thickness: 0.08 * D,
        roughness: 0.0,
        root_te_to_base: 0.0,
        profile: FinProfile::Square,
    }
}

/// The whole base-wake grid shrunk to a handful of cells, `nk = 8` where there is an azimuth.
fn tiny(c: &mut bowshock_core::engine::case3::Case3) {
    if let Some(b) = c.grid.base.as_mut() {
        b.n_up = 6;
        b.n_nose = 10;
        b.n_seg = b.n_seg.iter().map(|&n| (n / 4).max(4)).collect();
        b.n_wake = 8;
        b.n_gap = 6;
        b.nj_out = 10;
    }
    c.grid.nk = 8;
    c.grid.fin_chord_cells = Some(8);
}

/// [`tiny`] for the wall-resolved fin grid: the radial extent (`nj_out`) and the azimuthal stacks stay (the fin tip needs room, the stack sets its own cells).
fn tiny_stacked(c: &mut bowshock_core::engine::case3::Case3) {
    if let Some(b) = c.grid.base.as_mut() {
        b.n_up = 6;
        b.n_nose = 10;
        b.n_seg = b.n_seg.iter().map(|&n| (n / 4).max(4)).collect();
        b.n_wake = 8;
        b.n_gap = 6;
    }
    c.grid.fin_chord_cells = Some(8);
}

fn solver3() -> Analysis {
    Analysis::new(&body()).unwrap().engine(Engine::Solver3).backend(Backend::Cpu).resolution(Resolution::Coarse).case3_hook(tiny).max_iterations(60)
}

#[test]
fn the_engine_is_legacy_unless_asked_and_travels_in_the_spec() {
    let a = Analysis::new(&body()).unwrap();
    // BOWSHOCK_ENGINE decides the default of a new analysis; the tests run without it
    if std::env::var("BOWSHOCK_ENGINE").is_err() {
        assert_eq!(a.spec().unwrap().engine, Engine::Legacy);
    }
    let spec = a.engine(Engine::Solver3).spec().unwrap();
    assert_eq!(spec.engine, Engine::Solver3);
    assert_eq!(Analysis::from_spec(&spec).unwrap().spec().unwrap().engine, Engine::Solver3);
}

#[test]
fn a_motor_request_and_bad_viscous_settings_fail_loudly() {
    let motor = bowshock_core::MotorSpec { exit_diameter: 0.02, exit_mach: 2.5, exit_pressure_pa: 101_325.0, total_temperature_k: 1500.0 };
    let e = solver3().viscous(flight()).motor(motor).run(&[1.5]).unwrap_err();
    assert!(matches!(&e, Error::Unsupported(m) if m.contains("motor")), "{e}");
    // roughness and a drag tolerance are viscous knobs too
    assert!(matches!(solver3().roughness(1e-6).run(&[1.5]), Err(Error::InvalidInput(_))));
    assert!(matches!(solver3().drag_tolerance(0.01).run(&[1.5]), Err(Error::InvalidInput(_))));
    // and the checks of the conditions are the facade's
    assert!(matches!(solver3().run(&[5.0]), Err(Error::InvalidInput(_))));
    assert!(matches!(solver3().run(&[Condition::new(2.0, 40.0)]), Err(Error::InvalidInput(_))));
    assert!(matches!(solver3().run(&[Condition::new(0.2, 4.0)]), Err(Error::InvalidInput(_))));
}

#[test]
fn a_zero_incidence_point_is_the_wedge_with_its_parts_its_wall_and_an_axisymmetric_field() {
    let polar = solver3().keep_fields(true).run(&[2.0]).unwrap();
    assert_eq!(polar.backend, BackendUsed::Cpu);
    let p = &polar.points[0];
    assert!(p.forces.is_none() && p.field.is_none());
    let d = p.drag;
    assert!(!d.base_modelled && d.friction.is_none() && d.fins.is_none());
    assert!(d.forebody_pressure.is_finite() && d.base.is_finite() && d.total.is_finite());
    assert!((d.total - (d.forebody_pressure + d.base)).abs() < 1e-12, "the total is the parts");
    assert_eq!(p.detail.cd_aft.len(), 1, "one aft segment");
    assert!((p.detail.cd_nose + p.detail.cd_aft[0] - d.forebody_pressure).abs() < 1e-12);
    assert_eq!(p.detail.cd_base_solver, d.base);
    assert!(p.detail.cp_base.is_finite() && p.detail.y_plus_max.is_nan() && p.detail.cd_friction == 0.0);
    assert!(p.detail.wall.len() > 10 && p.detail.wall.windows(2).all(|w| w[0].x <= w[1].x), "the wall Cp in axial order");
    assert!(p.detail.wall.iter().all(|w| w.x < 10.0 * D && w.r > 0.0));
    assert_eq!(p.detail.stop_reason, StopReason::MaxIterations);
    assert_eq!(p.quality, Quality::Stalled);
    let f = p.field3.as_ref().expect("keep_fields keeps the final state");
    assert!(matches!(f.symmetry, Symmetry::Axisymmetric { .. }), "the viewer's 2D mode: {:?}", f.symmetry);
    assert_eq!((f.mach, f.alpha_deg), (2.0, 0.0));
}

#[test]
fn a_point_at_incidence_is_the_half_domain_with_forces_and_the_axial_force_by_surface() {
    let polar = solver3().run(&[Condition::new(2.0, 4.0)]).unwrap();
    let p = &polar.points[0];
    let f = p.forces.expect("forces at an angle of attack");
    assert!(f.cn.is_finite() && f.ca.is_finite());
    assert!((p.drag.total - f.ca).abs() < 1e-9 * (1.0 + f.ca.abs()), "the parts of the axial force add up to CA: {} {}", p.drag.total, f.ca);
    assert!(p.detail.wall.is_empty(), "a half domain has a ring of faces at every station");
    assert!(p.field.is_none() && p.field3.is_none());
}

#[test]
fn fins_at_zero_incidence_are_the_sector_with_a_direct_fin_drag() {
    let polar = solver3().fins(fins()).run(&[2.0]).unwrap();
    let p = &polar.points[0];
    let fd = p.drag.fins.expect("fins were given");
    assert_eq!(fd.friction, 0.0, "no friction while the engine is inviscid");
    assert!(fd.pressure.is_finite());
    assert!((p.drag.total - (p.drag.forebody_pressure + p.drag.base + fd.pressure)).abs() < 1e-12);
}

#[test]
fn conditions_come_back_in_the_order_asked_for_and_a_sweep_is_ascending() {
    let asked = [Condition::new(2.0, 4.0), Condition::from(1.5), Condition::new(2.0, 2.0), Condition::from(1.2)];
    let polar = solver3().run(&asked).unwrap();
    let got: Vec<(f64, f64)> = polar.points.iter().map(|p| (p.mach, p.alpha_deg)).collect();
    assert_eq!(got, vec![(2.0, 4.0), (1.5, 0.0), (2.0, 2.0), (1.2, 0.0)]);
}

#[test]
fn progress_is_reported_and_a_break_cancels() {
    let seen = Arc::new(AtomicUsize::new(0));
    let s = seen.clone();
    let polar = solver3().on_progress(move |_| {
        s.fetch_add(1, Ordering::Relaxed);
        ControlFlow::Continue(())
    });
    polar.run(&[2.0]).unwrap();
    assert!(seen.load(Ordering::Relaxed) >= 1);
    let e = solver3().on_progress(|_| ControlFlow::Break(())).run(&[2.0]).unwrap_err();
    assert!(matches!(e, Error::Cancelled), "{e}");
}

#[test]
fn the_uncertainty_estimate_runs_the_neighbouring_level() {
    let polar = solver3().estimate_uncertainty(true).run(&[2.0]).unwrap();
    assert!(polar.points[0].cd_uncertainty.is_some_and(|u| u.is_finite()));
}

#[test]
fn the_3d_engine_takes_one_to_sixteen_fins_the_legacy_engine_three_to_eight() {
    for (count, legacy, solver3) in
        [(1, false, true), (2, false, true), (3, true, true), (8, true, true), (9, false, true), (16, false, true), (17, false, false)]
    {
        let f = FinSpec { count, ..fins() };
        assert_eq!(f.validate_for(Engine::Legacy).is_ok(), legacy, "{count} fins, legacy");
        assert_eq!(f.validate_for(Engine::Solver3).is_ok(), solver3, "{count} fins, 3D engine");
        assert_eq!(f.validate().is_ok(), legacy, "validate() is the legacy rule");
    }
    // and the route builds a 12-fin sector
    let polar = solver3().fins(FinSpec { count: 12, ..fins() }).run(&[2.0]).unwrap();
    assert!(polar.points[0].drag.fins.is_some());
}

/// The GPU solver of a point is kept for the next cold point on the same grid (`analysis/solver3.rs`, `POOL`): a point solved on the kept solver is the point a fresh
/// solver gives, bit for bit. Through `tools/gpuq/gpuq`.
#[test]
#[ignore = "GPU: through tools/gpuq"]
fn a_kept_gpu_solver_gives_the_point_a_fresh_one_gives() {
    // each case on its own thread (the pool is per thread): the last point of `conditions` after the earlier ones, which leave their solver for it
    let last = |conditions: &'static [Condition]| {
        std::thread::spawn(move || {
            let a = solver3().backend(Backend::Gpu).max_iterations(200);
            let mut last = None;
            for &c in conditions {
                last = Some(a.run(&[c]).unwrap().points.remove(0));
            }
            last.unwrap()
        })
        .join()
        .unwrap()
    };
    // the wedge, then the half domain (a grid the kept wedge solver was not built for, then one it was)
    let _ = bowshock_core::take_facade_timings();
    let fresh = last(&[Condition { mach: 2.0, alpha_deg: 0.0 }]);
    let t0 = bowshock_core::take_facade_timings();
    assert!(t0.iter().filter(|t| t.what == "point").all(|t| !t.reused), "a fresh thread has no kept solver");
    let kept = last(&[Condition { mach: 1.5, alpha_deg: 0.0 }, Condition { mach: 2.0, alpha_deg: 0.0 }]);
    let t1 = bowshock_core::take_facade_timings();
    let reused: Vec<bool> = t1.iter().filter(|t| t.what == "point").map(|t| t.reused).collect();
    assert_eq!(reused, [false, true], "the second wedge point is solved on the kept solver");
    assert_eq!(fresh.drag.total.to_bits(), kept.drag.total.to_bits(), "wedge: {} {}", fresh.drag.total, kept.drag.total);
    assert_eq!((fresh.detail.iterations, fresh.detail.cp_base.to_bits()), (kept.detail.iterations, kept.detail.cp_base.to_bits()));
    let fresh = last(&[Condition { mach: 2.0, alpha_deg: 4.0 }]);
    let kept = last(&[Condition { mach: 2.0, alpha_deg: 0.0 }, Condition { mach: 1.5, alpha_deg: 4.0 }, Condition { mach: 2.0, alpha_deg: 4.0 }]);
    let (f, k) = (fresh.forces.unwrap(), kept.forces.unwrap());
    assert_eq!((f.cn.to_bits(), f.x_cp.to_bits()), (k.cn.to_bits(), k.x_cp.to_bits()), "half: {f:?} {k:?}");
    assert_eq!(fresh.detail.iterations, kept.detail.iterations);
}

fn flight() -> FlightConditions {
    FlightConditions { atmosphere: Atmosphere::Isa { altitude_m: 0.0 } }
}

/// A viscous request is a RANS run on the CPU (Backend::Cpu), tiny grid and a short run: every field of the drag is a finite number, the friction is there, the field has its eddy
/// ratio and the walls their skin friction.
fn viscous_point_is_complete(a: Analysis, c: Condition) {
    let polar = a.viscous(flight()).keep_fields(true).max_iterations(40).run(&[c]).unwrap();
    assert_eq!(polar.backend, BackendUsed::Cpu);
    let p = &polar.points[0];
    let d = p.drag;
    let friction = d.friction.expect("a viscous point has its skin friction");
    for (name, v) in [
        ("forebody", d.forebody_pressure),
        ("friction", friction),
        ("base", d.base),
        ("total", d.total),
        ("cd_friction", p.detail.cd_friction),
        ("y+", p.detail.y_plus_max),
    ] {
        assert!(v.is_finite(), "{name} {v} at {c:?}: {d:?} {:?}", p.detail);
    }
    assert!(friction > 0.0, "the shear of the walls: {friction}");
    assert!(p.detail.cp_base.is_finite() && p.detail.cd_nose.is_finite() && p.detail.cd_aft.iter().all(|x| x.is_finite()));
    let f = p.field3.as_ref().expect("keep_fields");
    assert_eq!(f.eddy_ratio.len(), f.cells(), "an eddy ratio per cell");
    assert!(f.walls.iter().any(|w| w.cf.iter().any(|v| *v != 0.0)), "the skin friction of the walls");
    if let Some(fd) = d.fins {
        assert!(fd.friction.is_finite() && fd.pressure.is_finite() && fd.friction > 0.0, "{fd:?}");
    }
    assert!((d.total - (d.forebody_pressure + friction + d.base + d.fins.map_or(0.0, |f| f.pressure + f.friction))).abs() < 1e-12);
}

#[test]
fn a_viscous_wedge_point_fills_every_drag_field_on_the_cpu() {
    viscous_point_is_complete(solver3(), Condition::from(0.5));
}

#[test]
fn a_viscous_finned_sector_point_fills_every_drag_field_on_the_cpu() {
    viscous_point_is_complete(solver3().fins(fins()).case3_hook(tiny_stacked), Condition::from(0.5));
}

#[test]
fn a_viscous_half_domain_point_fills_every_drag_field_on_the_cpu() {
    viscous_point_is_complete(solver3().fins(fins()), Condition::new(0.5, 4.0));
}

#[test]
fn an_elliptical_and_a_freeform_outline_are_meshed_and_a_notched_one_is_refused_with_its_cause() {
    use bowshock_core::FinOutline;
    let ellipse = solver3().fins(FinSpec { tip_chord: 0.0, sweep: 0.5 * D, ..fins() }).fin_outline(FinOutline::Ellipse).run(&[2.0]).unwrap();
    assert!(ellipse.points[0].drag.fins.is_some_and(|f| f.pressure.is_finite()));
    // a quadrilateral outline: root 1 d from 0 to 1 d, tip 0.4 d at 0.5 d span, swept
    let d = D;
    let polygon = vec![(0.0, 0.0), (0.6 * d, d), (d, d), (d, 0.0)];
    let free = solver3().fins(FinSpec { tip_chord: 0.4 * d, sweep: 0.6 * d, ..fins() }).fin_outline(FinOutline::Polygon(polygon)).run(&[2.0]).unwrap();
    assert!(free.points[0].drag.fins.is_some_and(|f| f.pressure.is_finite()));
    // an outline with two chordwise intervals at one height (a notch) is refused by the check, with the engine's words
    let notched = vec![(0.0, 0.0), (0.0, d), (0.3 * d, d), (0.3 * d, 0.4 * d), (0.6 * d, 0.4 * d), (0.6 * d, d), (d, d), (d, 0.0)];
    let e = solver3().fins(fins()).fin_outline(FinOutline::Polygon(notched)).mesh_check().unwrap_err();
    assert!(e.to_string().contains("chordwise"), "{e}");
}

#[test]
fn the_draft_resolution_is_a_viscous_wedge_with_wall_functions_and_fewer_cells() {
    use bowshock_core::Resolution;
    let run = |r: Resolution| {
        solver3()
            .resolution(r)
            .backend(Backend::Cpu)
            .viscous(flight())
            .keep_fields(true)
            .max_iterations(20)
            .case3_hook(|_| ())
            .run(&[2.0])
            .unwrap()
            .points
            .remove(0)
    };
    let (draft, coarse) = (run(Resolution::Draft), run(Resolution::Coarse));
    assert!(draft.drag.total.is_finite() && draft.drag.friction.is_some());
    assert!(draft.field3.unwrap().cells() < coarse.field3.unwrap().cells(), "a draft grid has fewer cells");
}

/// A run the solver's CFL controller took back is a point obtained at other numerics, and the point says so: the rollbacks, the CFL limit of the run that gave the result and
/// the warning. A run that goes through carries none of them.
#[test]
fn a_run_taken_back_by_the_cfl_controller_says_so() {
    use bowshock_core::Warning;
    let run = |cfl: Option<f64>| {
        let a = solver3().backend(Backend::Cpu).max_iterations(400);
        let a = match cfl {
            Some(_) => a.case3_hook(|c| c.numerics.implicit.cfl_max = 400.0),
            None => a,
        };
        a.run(&[2.0]).unwrap().points.remove(0)
    };
    let clean = run(None);
    assert!(clean.detail.rollbacks.is_empty() && clean.detail.cfl_limit.is_some());
    assert_eq!(clean.detail.attempts(), 1);
    assert!(!clean.warnings.iter().any(|w| matches!(w, Warning::RestartedAtLowerCfl { .. })));
    let taken = run(Some(400.0));
    let d = &taken.detail;
    assert!(!d.rollbacks.is_empty(), "no failure at cfl_max 400: raise it");
    assert_eq!(d.attempts(), 1 + d.rollbacks.len());
    assert!(d.cfl_limit.unwrap() < 400.0 && d.cfl_limit == d.rollbacks.last().map(|r| r.cfl_after));
    let w = taken.warnings.iter().find_map(|w| match w {
        Warning::RestartedAtLowerCfl { rollbacks, cfl_from, cfl_to } => Some((*rollbacks, *cfl_from, *cfl_to)),
        _ => None,
    });
    assert_eq!(w, Some((d.rollbacks.len(), 400.0, d.cfl_limit.unwrap())));
}
