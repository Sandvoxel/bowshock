//! Viscous (RANS, k-omega SST) analysis gate G3: skin friction, wall resolution and base pressure of a body at subsonic speed.

use bowshock_core::{
    AftSpec, Analysis, Atmosphere, Backend, BodySpec, FinProfile, FinSpec, FlightConditions, MotorSpec, NoseShape, NoseSpec, Quality, Resolution, Wake,
};
use std::time::Instant;

/// Tangent ogive 0.30 x 0.10 m and a 0.60 m tube at Mach 0.5 in the ISA at sea level: the friction drag is within 15 % of the
/// flat-plate estimate over the wetted area, the first cell has `y+ < 3`, the base pressure is a plausible subsonic suction and
/// the run ends converged or stationary.
#[test]
#[ignore]
fn g3_ogive_cylinder_friction_and_base_pressure() {
    let (d, nose_len, tube_len, mach) = (0.10, 0.30, 0.60, 0.5);
    let spec = BodySpec { nose: NoseSpec { shape: NoseShape::TangentOgive, length: nose_len, diameter: d }, aft: vec![AftSpec::Tube { length: tube_len }] };
    let atmosphere = Atmosphere::Isa { altitude_m: 0.0 };
    let air = atmosphere.state().unwrap();
    let re_c = air.density * air.sound_speed * d / air.viscosity;

    let t0 = Instant::now();
    let polar =
        Analysis::legacy(&spec).unwrap().backend(Backend::Cpu).resolution(Resolution::Coarse).viscous(FlightConditions { atmosphere }).run(&[mach]).unwrap();
    let wall_seconds = t0.elapsed().as_secs_f64();
    let p = &polar.points[0];
    let r = &p.detail;

    // flat-plate friction over the wetted area, in units of the nose base area
    let re_l = re_c * mach * (nose_len + tube_len) / d;
    let cf = 0.455 / re_l.log10().powf(2.58) / (1.0 + 0.144 * mach * mach).powf(0.65);
    let radius = 0.5 * d;
    let wetted: f64 = r.wall.windows(2).map(|w| 2.0 * 0.5 * (w[0].r + w[1].r) * (w[1].x - w[0].x).hypot(w[1].r - w[0].r) / (radius * radius)).sum();
    let cd_f_est = cf * wetted;

    println!(
        "G3: {} after {} iterations ({:.0} s), quality {:?}, drop {:.1} orders",
        r.stop_reason, r.iterations, wall_seconds, p.quality, r.residual_drop_orders
    );
    println!("G3: re_c {re_c:.4e}  Re_L {re_l:.4e}  Cf {cf:.5}  S_wet/A_ref {wetted:.3}");
    println!("G3: cd_friction {:.5}  flat-plate estimate {cd_f_est:.5}  ratio {:.3}", r.cd_friction, r.cd_friction / cd_f_est);
    println!(
        "G3: y_plus_max {:.3}  cp_base {:.4}  cd_base {:.5}  cd_pressure {:.5}  cd_total {:.5}",
        r.y_plus_max, r.cp_base, r.cd_base_solver, p.drag.forebody_pressure, p.drag.total
    );
    println!("G3: warnings {:?}  backend {:?}  fallback {:?}", p.warnings, polar.backend, polar.fallback);

    assert!(matches!(p.quality, Quality::Converged | Quality::Stationary), "quality {:?}, stop reason {}", p.quality, r.stop_reason);
    assert!(r.y_plus_max < 3.0, "y+ max {}", r.y_plus_max);
    assert!((-0.25..=-0.05).contains(&r.cp_base), "base Cp {}", r.cp_base);
    assert!((r.cd_friction / cd_f_est - 1.0).abs() < 0.15, "cd_friction {} vs flat-plate estimate {cd_f_est}", r.cd_friction);
}

fn spec() -> BodySpec {
    BodySpec {
        nose: NoseSpec { shape: NoseShape::TangentOgive, length: 0.30, diameter: 0.10 },
        aft: vec![AftSpec::Tube { length: 0.60 }, AftSpec::Transition { length: 0.10, aft_diameter: 0.08 }],
    }
}

fn flight() -> FlightConditions {
    FlightConditions { atmosphere: Atmosphere::Isa { altitude_m: 0.0 } }
}

#[test]
fn viscous_analysis_needs_a_dimensional_spec() {
    let body = bowshock_core::engine::body_grid::BaseBody::cylinder(
        bowshock_core::engine::body_grid::Body { nose: bowshock_core::engine::body_grid::Nose::TangentOgive { fineness: 3.0 } },
        6.0,
    );
    let err = Analysis::from_internal(body).backend(Backend::Cpu).viscous(flight()).run(&[0.5]).unwrap_err();
    assert!(matches!(err, bowshock_core::Error::InvalidInput(m) if m.contains("dimensional BodySpec")));
}

#[test]
fn viscous_grids_pass_the_quality_check_and_report_friction() {
    use bowshock_core::Wake;
    for (wake, base_modelled) in [(None, false), (Some(Wake::Sting), true)] {
        let mut a = Analysis::legacy(&spec()).unwrap().backend(Backend::Cpu).resolution(Resolution::Coarse).viscous(flight()).max_iterations(5);
        if let Some(w) = wake {
            a = a.wake(w);
        }
        let polar = a.run(&[0.5]).unwrap();
        let p = &polar.points[0];
        let friction = p.drag.friction.expect("viscous analysis reports friction");
        assert!(friction.is_finite() && friction > 0.0, "friction {friction}");
        assert_eq!(p.drag.base_modelled, base_modelled);
        assert!(p.detail.y_plus_max.is_finite());
        assert!((p.drag.total - (p.drag.forebody_pressure + friction + p.drag.base)).abs() < 1e-12);
    }
}

/// The G3 body at Mach 0.5 on the CPU and on the GPU: total CD within 1 %, friction within 2 %, base Cp within 0.01.
#[cfg(feature = "gpu")]
#[test]
#[ignore = "needs a GPU (CI has none)"]
fn g3_gpu_matches_cpu() {
    let spec = BodySpec { nose: NoseSpec { shape: NoseShape::TangentOgive, length: 0.30, diameter: 0.10 }, aft: vec![AftSpec::Tube { length: 0.60 }] };
    let run = |backend| {
        let t0 = Instant::now();
        let polar = Analysis::legacy(&spec).unwrap().backend(backend).resolution(Resolution::Coarse).viscous(flight()).run(&[0.5]).unwrap();
        println!("G3 {backend:?}: {:.1} s, backend used {:?}, fallback {:?}", t0.elapsed().as_secs_f64(), polar.backend, polar.fallback);
        polar.points.into_iter().next().unwrap()
    };
    let (cpu, gpu) = (run(Backend::Cpu), run(Backend::Gpu));
    let (c, g) = (&cpu.detail, &gpu.detail);
    println!("G3: cd_total    cpu {:.5}  gpu {:.5}  rel {:.4}", cpu.drag.total, gpu.drag.total, gpu.drag.total / cpu.drag.total - 1.0);
    println!("G3: cd_friction cpu {:.5}  gpu {:.5}  rel {:.4}", c.cd_friction, g.cd_friction, g.cd_friction / c.cd_friction - 1.0);
    println!("G3: cp_base     cpu {:.4}  gpu {:.4}  diff {:.4}", c.cp_base, g.cp_base, g.cp_base - c.cp_base);
    println!("G3: iters cpu {} gpu {}; quality cpu {:?} gpu {:?}", c.iterations, g.iterations, cpu.quality, gpu.quality);
    assert!((gpu.drag.total / cpu.drag.total - 1.0).abs() < 0.01);
    assert!((g.cd_friction / c.cd_friction - 1.0).abs() < 0.02);
    assert!((g.cp_base - c.cp_base).abs() < 0.01);
}

fn fin_set() -> FinSpec {
    FinSpec {
        count: 4,
        root_chord: 0.12,
        tip_chord: 0.06,
        span: 0.08,
        sweep: 0.06,
        thickness: 0.003,
        roughness: 0.0,
        root_te_to_base: 0.0,
        profile: FinProfile::Rounded,
    }
}

fn g3_spec() -> BodySpec {
    BodySpec { nose: NoseSpec { shape: NoseShape::TangentOgive, length: 0.30, diameter: 0.10 }, aft: vec![AftSpec::Tube { length: 0.60 }] }
}

/// The G3 body with fins: both fin parts are positive and the total is the sum of all parts.
#[test]
fn fins_add_friction_and_pressure_drag() {
    let polar = Analysis::legacy(&g3_spec())
        .unwrap()
        .fins(fin_set())
        .backend(Backend::Cpu)
        .resolution(Resolution::Coarse)
        .viscous(FlightConditions { atmosphere: Atmosphere::Isa { altitude_m: 0.0 } })
        .max_iterations(50)
        .run(&[0.5])
        .unwrap();
    let d = polar.points[0].drag;
    let f = d.fins.expect("fin drag");
    assert!(f.friction > 0.0 && f.pressure > 0.0, "{f:?}");
    let sum = d.forebody_pressure + d.friction.unwrap() + d.base + f.friction + f.pressure;
    assert!((d.total - sum).abs() < 1e-12, "{} vs {sum}", d.total);
}

#[test]
fn fins_without_flight_conditions_are_rejected() {
    let err = Analysis::legacy(&g3_spec()).unwrap().fins(fin_set()).run(&[0.5]).unwrap_err();
    assert_eq!(err.to_string(), "fins need flight conditions: call Analysis::viscous");
}

fn motor() -> MotorSpec {
    MotorSpec { exit_diameter: 0.06, exit_mach: 2.5, exit_pressure_pa: 101325.0, total_temperature_k: 1500.0 }
}

/// The G3 body with a Mach 2.5 jet from a 60 mm exit in a 100 mm base: the point is flagged motor-on, the base drag is the solver's
/// (annulus only) and the base pressure is finite.
#[test]
fn motor_on_analysis_reports_annulus_base_drag() {
    let t0 = Instant::now();
    let polar = Analysis::legacy(&g3_spec())
        .unwrap()
        .motor(motor())
        .backend(Backend::Cpu)
        .resolution(Resolution::Coarse)
        .viscous(FlightConditions { atmosphere: Atmosphere::Isa { altitude_m: 0.0 } })
        .max_iterations(100)
        .run(&[0.8])
        .unwrap();
    let p = &polar.points[0];
    let (d, r) = (&p.drag, &p.detail);
    println!("motor: forebody {:.5}  friction {:.5}  base {:.5}  total {:.5}", d.forebody_pressure, d.friction.unwrap(), d.base, d.total);
    println!(
        "motor: cp_base {:.4}  quality {:?}  iterations {}  stop {}  wall {:.1} s",
        r.cp_base,
        p.quality,
        r.iterations,
        r.stop_reason,
        t0.elapsed().as_secs_f64()
    );
    assert!(polar.motor_on);
    assert!(!d.base_modelled);
    assert_ne!(p.quality, Quality::Failed);
    assert!(r.cp_base.is_finite(), "cp_base {}", r.cp_base);
}

/// The motor case at Mach 0.8 on the GPU and on the CPU: total drag within 1 %, base pressure within 0.01.
#[test]
#[ignore = "needs a GPU"]
fn motor_gpu_matches_cpu() {
    let run = |backend| {
        let t0 = Instant::now();
        let polar = Analysis::legacy(&g3_spec())
            .unwrap()
            .motor(motor())
            .backend(backend)
            .resolution(Resolution::Coarse)
            .viscous(FlightConditions { atmosphere: Atmosphere::Isa { altitude_m: 0.0 } })
            .run(&[0.8])
            .unwrap();
        println!("motor {backend:?}: {:.1} s, backend used {:?}, fallback {:?}", t0.elapsed().as_secs_f64(), polar.backend, polar.fallback);
        polar.points.into_iter().next().unwrap()
    };
    let (cpu, gpu) = (run(Backend::Cpu), run(Backend::Gpu));
    let (c, g) = (&cpu.detail, &gpu.detail);
    println!("motor: cd_total    cpu {:.5}  gpu {:.5}  rel {:.4}", cpu.drag.total, gpu.drag.total, gpu.drag.total / cpu.drag.total - 1.0);
    println!("motor: cd_friction cpu {:.5}  gpu {:.5}  rel {:.4}", c.cd_friction, g.cd_friction, g.cd_friction / c.cd_friction - 1.0);
    println!("motor: cd_base     cpu {:.5}  gpu {:.5}", cpu.drag.base, gpu.drag.base);
    println!("motor: cp_base     cpu {:.4}  gpu {:.4}  diff {:.4}", c.cp_base, g.cp_base, g.cp_base - c.cp_base);
    println!("motor: iters cpu {} gpu {}; quality cpu {:?} gpu {:?}", c.iterations, g.iterations, cpu.quality, gpu.quality);
    assert!((gpu.drag.total / cpu.drag.total - 1.0).abs() < 0.01);
    assert!((g.cp_base - c.cp_base).abs() < 0.01);
}

#[test]
fn motor_run_errors() {
    let flight = FlightConditions { atmosphere: Atmosphere::Isa { altitude_m: 0.0 } };
    let err = Analysis::legacy(&g3_spec()).unwrap().motor(motor()).run(&[0.5]).unwrap_err();
    assert_eq!(err.to_string(), "motor-on analysis needs Analysis::viscous");
    let err = Analysis::legacy(&g3_spec()).unwrap().motor(motor()).viscous(flight).wake(Wake::Sting).run(&[0.5]).unwrap_err();
    assert_eq!(err.to_string(), "motor-on analysis needs the base wake");
    let wide = MotorSpec { exit_diameter: 0.0951, ..motor() };
    let err = Analysis::legacy(&g3_spec()).unwrap().motor(wide).viscous(flight).run(&[0.5]).unwrap_err();
    assert!(err.to_string().contains("motor.exit_diameter"), "{err}");
}

#[test]
fn drag_tolerance_needs_viscous_and_a_sane_value() {
    let base = || Analysis::legacy(&spec()).unwrap().backend(Backend::Cpu).resolution(Resolution::Coarse);
    let bad = |a: Analysis| matches!(a.run(&[0.5]), Err(bowshock_core::Error::InvalidInput(m)) if m.contains("drag"));
    assert!(bad(base().drag_tolerance(0.002)));
    for rel in [0.0, f64::NAN, -1e-3, 0.5] {
        assert!(bad(base().viscous(flight()).drag_tolerance(rel)), "{rel}");
    }
}

fn roughness_spec() -> BodySpec {
    BodySpec { nose: NoseSpec { shape: NoseShape::TangentOgive, length: 0.30, diameter: 0.10 }, aft: vec![AftSpec::Tube { length: 0.60 }] }
}

#[test]
fn roughness_is_validated() {
    let invalid = |a: Analysis| match a.backend(Backend::Cpu).run(&[0.5]) {
        Err(bowshock_core::Error::InvalidInput(m)) => m,
        other => panic!("expected InvalidInput, got {:?}", other.map(|p| p.points.len())),
    };
    let viscous = || Analysis::legacy(&roughness_spec()).unwrap().viscous(flight());
    for bad in [-1e-6, f64::NAN, f64::INFINITY, 1e-3, 0.01 * 0.10] {
        assert!(invalid(viscous().roughness(bad)).contains("roughness"), "{bad}");
    }
    let m = invalid(Analysis::legacy(&roughness_spec()).unwrap().roughness(60e-6));
    assert!(m.contains("needs Analysis::viscous"), "{m}");
}

#[test]
fn roughness_reaches_the_solver_case_in_nose_diameters() {
    let a = Analysis::legacy(&roughness_spec()).unwrap().viscous(flight()).roughness(60e-6);
    let v = a.engine_cases(&[0.5]).unwrap()[0].viscous.unwrap();
    assert!((v.roughness - 60e-6 / 0.10).abs() < 1e-15, "{}", v.roughness);
    let smooth = Analysis::legacy(&roughness_spec()).unwrap().viscous(flight()).engine_cases(&[0.5]).unwrap();
    assert_eq!(smooth[0].viscous.unwrap().roughness, 0.0);
}
