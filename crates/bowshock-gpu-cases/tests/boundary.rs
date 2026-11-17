//! `GpuSolver::read_boundary_into` and `GpuSolver3::read_boundary_into` against a full state download: the host state outside the cells they read is NaN,
//! so a cell the post-processing needs but the partial read missed turns its result into NaN and fails the exact comparison with the results from the full
//! download (`wall_distribution`, `friction`, `base_state` in 2D; `forces_on`, `breakdown_on` in 3D). The state is the free stream after a few iterations.

use bowshock_cfd::gpu::{GpuSolver, GpuSolver3, Precision};
use bowshock_core::atmosphere::{Atmosphere, FlightConditions};
use bowshock_core::engine::body_grid::BaseBody;
use bowshock_core::engine::case::{Case, Wake};
use bowshock_core::engine::case3::Case3;
use bowshock_core::engine::run::{base_state, friction, wall_distribution};
use bowshock_core::engine::run3::{breakdown_on, forces_on};
use bowshock_core::spec::{AftSpec, BodySpec, MotorSpec, NoseShape, NoseSpec};
use bowshock_core::{Analysis, Resolution};

const PRECISIONS: [Precision; 2] = [Precision::F32, Precision::Extended];

/// The G3 body (tangent ogive 0.30 x 0.10 m + 0.60 m tube, ISA at sea level, Mach 0.5), coarse grid with the flat base and wake block; with a sting
/// behind the base (`j_sting > 0`) if `sting`.
fn body_case(sting: bool) -> Case {
    let (d, mach) = (0.10, 0.5);
    let spec = BodySpec { nose: NoseSpec { shape: NoseShape::TangentOgive, length: 0.30, diameter: d }, aft: vec![AftSpec::Tube { length: 0.60 }] };
    let (mut body, _): (BaseBody, _) = spec.to_internal().unwrap();
    if sting {
        body.sting_radius = 0.4 * body.base_radius();
    }
    let air = Atmosphere::Isa { altitude_m: 0.0 }.state().unwrap();
    let re_c = air.density * air.sound_speed * d / air.viscosity;
    let mut case = Case::viscous(mach, body, 0, Wake::Base, re_c, 110.4 / air.temperature_k, mach);
    case.numerics.first_order_iters = 5;
    case
}

/// The C2 motor case on the G3 body (Mach 0.8, coarse grid): a Mach 2.5 jet from a 60 mm exit in the base (nozzle rows on the base face).
fn motor_case() -> Case {
    let spec = BodySpec { nose: NoseSpec { shape: NoseShape::TangentOgive, length: 0.30, diameter: 0.10 }, aft: vec![AftSpec::Tube { length: 0.60 }] };
    let motor = MotorSpec { exit_diameter: 0.06, exit_mach: 2.5, exit_pressure_pa: 101325.0, total_temperature_k: 1500.0 };
    let flight = FlightConditions { atmosphere: Atmosphere::Isa { altitude_m: 0.0 } };
    let analysis = Analysis::new(&spec).unwrap().resolution(Resolution::Coarse).viscous(flight).motor(motor);
    let mut case = analysis.engine_cases(&[0.8]).unwrap().remove(0);
    case.numerics.first_order_iters = 5;
    case
}

/// 10 iterations from the free stream, then the post-processing of the full state against that of the boundary read into a NaN-filled state.
fn check_2d(label: &str, case: &Case, prec: Precision) {
    let ctx = luma::default_context().unwrap();
    let (mut full_host, mut part_host) = (case.build_solver(), case.build_solver());
    let mut lazy = GpuSolver::from_host(&ctx, &full_host, prec).unwrap();
    lazy.run(10).unwrap();
    let full = lazy.state_f64().unwrap();
    let mut part = vec![[f64::NAN; 4]; full.len()];
    lazy.read_boundary_into(&mut part).unwrap();
    let read = part.iter().filter(|q| !q[0].is_nan()).count();
    assert!(read > 0 && 4 * read < full.len(), "{label} {prec:?}: {read} of {} cells read", full.len());
    for (p, f) in part.iter().zip(&full) {
        assert!(p[0].is_nan() || p == f, "{label} {prec:?}: a cell read differs from the full download");
    }
    full_host.u = full;
    part_host.u = part;
    let rb2 = case.body.body.radius().powi(2);
    let (wall_a, wall_b) = (wall_distribution(&case.body, &full_host), wall_distribution(&case.body, &part_host));
    assert_eq!(wall_a.0.len(), wall_b.0.len(), "{label} {prec:?}: wall points");
    for (a, b) in wall_a.0.iter().zip(&wall_b.0) {
        same(&[a.x, a.r, a.s0, a.s1, a.cp, a.cp_cell], &[b.x, b.r, b.s0, b.s1, b.cp, b.cp_cell], label, "wall_distribution");
    }
    same(&[wall_a.1, wall_a.2], &[wall_b.1, wall_b.2], label, "wall_distribution nose");
    let (fa, fb) = (friction(case, &full_host), friction(case, &part_host));
    same(&[fa.0, fa.1], &[fb.0, fb.1], label, "friction");
    let (base_a, base_b) = (base_state(&full_host, rb2), base_state(&part_host, rb2));
    assert_eq!(base_a.0.len(), base_b.0.len(), "{label} {prec:?}: base points");
    for (a, b) in base_a.0.iter().zip(&base_b.0) {
        same(&[a.r0, a.r1, a.cp, a.cp_cell], &[b.r0, b.r1, b.cp, b.cp_cell], label, "base_state");
    }
    same(&[base_a.1, base_a.2, base_a.3], &[base_b.1, base_b.2, base_b.3], label, "base_state integrals");
}

/// The values of the full download are finite and the boundary read reproduces them exactly.
fn same(full: &[f64], part: &[f64], label: &str, what: &str) {
    assert!(full.iter().all(|x| x.is_finite()), "{label}: {what} of the full download is not finite: {full:?}");
    assert_eq!(full, part, "{label}: {what} from the boundary read");
}

#[test]
fn boundary_read_matches_full_download_with_base_block() {
    let case = body_case(false);
    assert_eq!(case.build_solver().block().map(|b| b.j_sting), Some(0));
    for prec in PRECISIONS {
        check_2d("base block", &case, prec);
    }
}

#[test]
fn boundary_read_matches_full_download_with_sting_offset() {
    let case = body_case(true);
    assert!(case.build_solver().block().is_some_and(|b| b.j_sting > 0));
    for prec in PRECISIONS {
        check_2d("sting block", &case, prec);
    }
}

#[test]
fn boundary_read_matches_full_download_with_nozzle() {
    let case = motor_case();
    assert!(case.build_solver().nozzle().is_some());
    for prec in PRECISIONS {
        check_2d("nozzle", &case, prec);
    }
}

/// The tiny ANF-like case with four fins at 2 degrees: 5 iterations, then `forces_on` and `breakdown_on` of the full state against the boundary read.
#[test]
fn boundary_read_matches_full_download_3d() {
    let case = Case3::tiny_anf(0.5, 2f64.to_radians());
    for prec in PRECISIONS {
        let ctx = luma::default_context().unwrap();
        let host = case.build_solver().unwrap();
        let mut lazy = GpuSolver3::from_host(&ctx, &host, prec).unwrap();
        lazy.run(5).unwrap();
        let full = lazy.state_f64().unwrap();
        let mut part = vec![[f64::NAN; 5]; full.len()];
        lazy.read_boundary_into(&host.grid, &mut part).unwrap();
        let read = part.iter().filter(|q| !q[0].is_nan()).count();
        assert!(read > 0 && 2 * read < full.len(), "3D {prec:?}: {read} of {} cells read", full.len());
        for (p, f) in part.iter().zip(&full) {
            assert!(p[0].is_nan() || p == f, "3D {prec:?}: a cell read differs from the full download");
        }
        let (g, p_inf, x_base) = (&host.grid, host.cfg.freestream[4], case.body.x_base());
        let (fa, fb) = (forces_on(&case, g, &full, p_inf, x_base), forces_on(&case, g, &part, p_inf, x_base));
        same(&[fa.cn, fa.ca, fa.x_cp], &[fb.cn, fb.ca, fb.x_cp], "3D", "forces_on");
        let (ba, bb) = (breakdown_on(&case, g, &full, p_inf), breakdown_on(&case, g, &part, p_inf));
        for (a, b) in [(ba.nose, bb.nose), (ba.afterbody, bb.afterbody), (ba.fin_sides, bb.fin_sides), (ba.fin_edges, bb.fin_edges), (ba.fin_tips, bb.fin_tips)]
        {
            // `x_cp` of a part is NaN when its normal force vanishes
            same(&[a.cn, a.ca], &[b.cn, b.ca], "3D", "breakdown_on");
            assert_eq!(a.x_cp.to_bits(), b.x_cp.to_bits(), "3D {prec:?}: breakdown_on x_cp");
        }
    }
}
