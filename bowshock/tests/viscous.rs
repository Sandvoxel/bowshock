//! Viscous (RANS, k-omega SST) analysis gate G3: skin friction, wall resolution and base pressure of a body at subsonic speed.

use bowshock::analysis::{Analysis, Backend, Quality, Resolution};
use bowshock::atmosphere::{Atmosphere, FlightConditions};
use bowshock::spec::{AftSpec, BodySpec, NoseShape, NoseSpec};
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
        Analysis::new(&spec).unwrap().backend(Backend::Cpu).resolution(Resolution::Coarse).viscous(FlightConditions { atmosphere }).run(&[mach]).unwrap();
    let wall_seconds = t0.elapsed().as_secs_f64();
    let p = &polar.points[0];
    let r = &p.detail;

    // flat-plate friction over the wetted area, in units of the nose base area
    let re_l = re_c * mach * (nose_len + tube_len) / d;
    let cf = 0.455 / re_l.log10().powf(2.58) / (1.0 + 0.144 * mach * mach).powf(0.65);
    let radius = 0.5;
    let wetted: f64 = r.wall.windows(2).map(|w| 2.0 * 0.5 * (w[0].r + w[1].r) * (w[1].x - w[0].x).hypot(w[1].r - w[0].r) / (radius * radius)).sum();
    let cd_f_est = cf * wetted;

    println!("G3: {} after {} iterations ({:.0} s), quality {:?}, drop {:.1} orders", r.stop_reason, r.iters, wall_seconds, p.quality, r.drop_orders);
    println!("G3: re_c {re_c:.4e}  Re_L {re_l:.4e}  Cf {cf:.5}  S_wet/A_ref {wetted:.3}");
    println!("G3: cd_friction {:.5}  flat-plate estimate {cd_f_est:.5}  ratio {:.3}", r.cd_friction, r.cd_friction / cd_f_est);
    println!(
        "G3: y_plus_max {:.3}  cp_base {:.4}  cd_base {:.5}  cd_pressure {:.5}  cd_total {:.5}",
        r.y_plus_max, r.cp_base, r.cd_base, p.drag.forebody_pressure, p.drag.total
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
    let body = bowshock::body_grid::BaseBody::cylinder(bowshock::body_grid::Body { nose: bowshock::body_grid::Nose::TangentOgive { fineness: 3.0 } }, 6.0);
    let err = Analysis::from_internal(body).backend(Backend::Cpu).viscous(flight()).run(&[0.5]).unwrap_err();
    assert!(matches!(err, bowshock::error::Error::InvalidInput(m) if m.contains("dimensional BodySpec")));
}

#[test]
fn viscous_grids_pass_the_quality_check_and_report_friction() {
    use bowshock::case::Wake;
    for (wake, base_modelled) in [(None, false), (Some(Wake::Sting), true)] {
        let mut a = Analysis::new(&spec()).unwrap().backend(Backend::Cpu).resolution(Resolution::Coarse).viscous(flight()).max_iterations(5);
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
        let polar = Analysis::new(&spec).unwrap().backend(backend).resolution(Resolution::Coarse).viscous(flight()).run(&[0.5]).unwrap();
        println!("G3 {backend:?}: {:.1} s, backend used {:?}, fallback {:?}", t0.elapsed().as_secs_f64(), polar.backend, polar.fallback);
        polar.points.into_iter().next().unwrap()
    };
    let (cpu, gpu) = (run(Backend::Cpu), run(Backend::Gpu));
    let (c, g) = (&cpu.detail, &gpu.detail);
    println!("G3: cd_total    cpu {:.5}  gpu {:.5}  rel {:.4}", cpu.drag.total, gpu.drag.total, gpu.drag.total / cpu.drag.total - 1.0);
    println!("G3: cd_friction cpu {:.5}  gpu {:.5}  rel {:.4}", c.cd_friction, g.cd_friction, g.cd_friction / c.cd_friction - 1.0);
    println!("G3: cp_base     cpu {:.4}  gpu {:.4}  diff {:.4}", c.cp_base, g.cp_base, g.cp_base - c.cp_base);
    println!("G3: iters cpu {} gpu {}; quality cpu {:?} gpu {:?}", c.iters, g.iters, cpu.quality, gpu.quality);
    assert!((gpu.drag.total / cpu.drag.total - 1.0).abs() < 0.01);
    assert!((g.cd_friction / c.cd_friction - 1.0).abs() < 0.02);
    assert!((g.cp_base - c.cp_base).abs() < 0.01);
}
