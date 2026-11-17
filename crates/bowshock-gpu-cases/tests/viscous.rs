//! The viscous terms and the SST model in array ops (`GpuSolver`, f32 and double-float) against the f64 CPU `Solver`: the flat plates of the CPU gates G1 (laminar) and G2 (SST) in the annulus formulation, Mach 0.2, `Scheme::Implicit(ImplicitParams::VISCOUS)`,
//! van Albada with the low-Mach correction and only 5 first-order iterations, and the G3 body (tangent ogive 0.30 x 0.10 m + 0.60 m tube,
//! ISA 0 m, Mach 0.5, coarse grid with the flat base and wake block, SST).
//!
//! CPU and GPU solver start from the same state (`GpuSolver::from_host`) and run the same number of iterations; the states are compared by
//! `max |dU| / (1 + |U|)`, the residual histories by `max |r - r_cpu| / r_cpu` (from the second iteration: the first is round-off of the
//! uniform start on both sides).

use bowshock_cfd::gas::freestream;
use bowshock_cfd::gpu::{GpuSolver, Precision};
use bowshock_cfd::grid::{plate_grid, PlateGridParams};
use bowshock_cfd::implicit_params::ImplicitParams;
use bowshock_cfd::limiter::Limiter;
use bowshock_cfd::solver::{Bc, Config, Scheme, Solver};
use bowshock_cfd::sst::K_INF;
use bowshock_cfd::sst::{Turbulence, Viscous};
use bowshock_core::atmosphere::{Atmosphere, FlightConditions};
use bowshock_core::engine::body_grid::BaseBody;
use bowshock_core::engine::case::{Case, Wake};
use bowshock_core::engine::result::StopReason;
use bowshock_core::engine::run::{finish, run_on};
use bowshock_core::spec::{AftSpec, BodySpec, MotorSpec, NoseShape, NoseSpec};
use bowshock_core::{Analysis, Resolution};

/// The flat plates of the CPU gates, with only 5 first-order iterations: G1 (laminar, `re_c = 1e6`) or G2 (SST, `re_c = 2.5e7`).
fn plate_solver(sst: bool, scheme: Scheme) -> Solver {
    plate_solver_ks(sst, scheme, 0.0)
}

/// [`plate_solver`] with the wall roughness `ks` (nose diameters) on the plate.
fn plate_solver_ks(sst: bool, scheme: Scheme, ks: f64) -> Solver {
    let (grid, re_c, turbulence, wall_i_start) = if sst {
        (
            plate_grid(&PlateGridParams { r0: 20.0, x_up: 0.33, length: 2.0, height: 1.0, n_up: 24, n_plate: 112, nj: 72, dx_le: 0.002, dr_wall: 4e-6 }),
            2.5e7,
            Turbulence::Sst,
            24,
        )
    } else {
        (
            plate_grid(&PlateGridParams { r0: 20.0, x_up: 0.5, length: 1.0, height: 0.6, n_up: 20, n_plate: 80, nj: 64, dx_le: 0.005, dr_wall: 2e-4 }),
            1e6,
            Turbulence::Laminar,
            20,
        )
    };
    let cfg = Config {
        cfl: 0.5,
        limiter: Limiter::VanAlbada,
        freestream: freestream(0.2),
        bc: [Bc::Characteristic, Bc::PressureOutflow, Bc::SlipWall, Bc::Characteristic],
        first_order_iters: 5,
    };
    let mut s = Solver::new(grid, cfg);
    s.low_mach = true;
    s.set_scheme(scheme);
    s.set_viscous(Some(Viscous { re_c, sutherland: 0.368, turbulence, wall_i_start, roughness: ks }));
    s
}

/// Maximum and mean of `|U - U_cpu| / (1 + |U_cpu|)` over the cells and the four components.
fn state_diff(a: &[[f64; 4]], cpu: &[[f64; 4]]) -> (f64, f64) {
    let d: Vec<f64> = a.iter().zip(cpu).flat_map(|(a, b)| (0..4).map(move |k| (a[k] - b[k]).abs() / (1.0 + b[k].abs()))).collect();
    (d.iter().fold(0.0f64, |m, &x| m.max(x)), d.iter().sum::<f64>() / d.len() as f64)
}

fn residual_diff(a: &[f64], cpu: &[f64]) -> f64 {
    a.iter().zip(cpu).skip(1).fold(0.0f64, |m, (a, b)| m.max((a - b).abs() / b))
}

/// Laminar flat plate (G1): the viscous terms (cell gradients, viscous face fluxes, hoop stress, diffusive pseudo-time step), the no-slip wall
/// ghost with its Jacobian and the diffusive spectral radius of the implicit operator, 30 implicit iterations from the free stream.
macro_rules! laminar_plate {
    ($prec:expr, $tol:expr) => {{
        let ctx = luma::default_context().unwrap();
        let mut cpu = plate_solver(false, Scheme::Implicit(ImplicitParams::VISCOUS));
        let mut lazy = GpuSolver::from_host(&ctx, &cpu, $prec).unwrap();
        let res_cpu: Vec<f64> = (0..30).map(|_| cpu.step()).collect();
        let res = lazy.run(30).unwrap();
        let (max, mean) = state_diff(&lazy.state_f64().unwrap(), &cpu.u);
        let worst_res = residual_diff(&res, &res_cpu);
        println!(
            "laminar plate {}: max |dU|/(1+|U|) = {max:.2e}, mean {mean:.2e}, max rel residual diff {worst_res:.2e}, residual {:.3e} -> {:.3e}",
            format!("{:?}", $prec),
            res_cpu[1],
            res_cpu[29]
        );
        assert!(max <= $tol && res.iter().all(|r| r.is_finite()), "laminar plate {}: state difference {max:e}", format!("{:?}", $prec));
    }};
}

#[test]
fn laminar_plate_state_matches_cpu_f32() {
    laminar_plate!(Precision::F32, 1e-3);
}

#[test]
fn laminar_plate_state_matches_cpu_df32() {
    laminar_plate!(Precision::Extended, 1e-3);
}

/// The explicit scheme (SSP-RK3 with the diffusive time step) of the laminar plate, 30 iterations.
#[test]
fn laminar_plate_explicit_state_matches_cpu() {
    let ctx = luma::default_context().unwrap();
    let mut cpu = plate_solver(false, Scheme::Explicit);
    let mut lazy = GpuSolver::from_host(&ctx, &cpu, Precision::Extended).unwrap();
    let res_cpu: Vec<f64> = (0..30).map(|_| cpu.step()).collect();
    let res = lazy.run(30).unwrap();
    let (max, mean) = state_diff(&lazy.state_f64().unwrap(), &cpu.u);
    println!("laminar plate explicit df32: max |dU|/(1+|U|) = {max:.2e}, mean {mean:.2e}, max rel residual diff {:.2e}", residual_diff(&res, &res_cpu));
    assert!(max <= 1e-6, "state difference {max:e}");
}

/// SST flat plate (G2 grid), 50 iterations from the free stream: the mean state and the `k`, `omega` fields on the cells where the turbulence has
/// grown (`k > 10 k_inf`).
macro_rules! sst_plate {
    ($prec:expr, $scheme:expr, $grown:expr) => {
        sst_plate!($prec, $scheme, $grown, 0.0)
    };
    ($prec:expr, $scheme:expr, $grown:expr, $ks:expr) => {{
        let ctx = luma::default_context().unwrap();
        let mut cpu = plate_solver_ks(true, $scheme, $ks);
        let mut lazy = GpuSolver::from_host(&ctx, &cpu, $prec).unwrap();
        let res_cpu: Vec<f64> = (0..50).map(|_| cpu.step()).collect();
        let res = lazy.run(50).unwrap();
        let (max, mean) = state_diff(&lazy.state_f64().unwrap(), &cpu.u);
        let (turb, mu_t) = lazy.turb_state().unwrap();
        let (mut dk, mut dw, mut dm, mut n) = (0.0f64, 0.0f64, 0.0f64, 0);
        for ((t, m), (c, cm)) in turb.iter().zip(&mu_t).zip(cpu.turb.iter().zip(&cpu.mu_t)) {
            if c[0] > $grown * K_INF {
                dk = dk.max((t[0] - c[0]).abs() / c[0]);
                dw = dw.max((t[1] - c[1]).abs() / c[1]);
                dm = dm.max((m - cm).abs() / cm.abs().max(1e-12));
                n += 1;
            }
        }
        let rn = lazy.turb_residual().unwrap();
        println!(
            "SST plate {}: state max {max:.2e} mean {mean:.2e}; k, omega, mu_t on {n} cells with k > {} k_inf: max rel diff {dk:.2e}, {dw:.2e}, {dm:.2e}; max rel residual diff {:.2e}, residual {:.3e} -> {:.3e}; k residual lazy {rn:.3e} cpu {:.3e}",
            format!("{:?}", $prec),
            $grown,
            residual_diff(&res, &res_cpu),
            res_cpu[1],
            res_cpu[49],
            cpu.turb_residual()
        );
        assert!(n > 100, "the turbulence has not grown: {n} cells with k > {} k_inf", $grown);
        assert!(mean <= 1e-3 && dk <= 1e-2 && dw <= 1e-2, "SST plate {}: mean state {mean:e}, k {dk:e}, omega {dw:e}", format!("{:?}", $prec));
        (mean, dk, dw)
    }};
}

#[test]
fn sst_plate_state_matches_cpu_f32() {
    sst_plate!(Precision::F32, Scheme::Implicit(ImplicitParams::VISCOUS), 10.0);
}

#[test]
fn sst_plate_state_matches_cpu_df32() {
    sst_plate!(Precision::Extended, Scheme::Implicit(ImplicitParams::VISCOUS), 10.0);
}

/// The rough-wall SST plate: `ks = 1e-3 x` at the G2 station `x = 0.97` (fully rough there), the rough `omega_w` of the wall ghost and the `F3` factor
/// of the eddy viscosity, same state tolerances as the smooth plate.
#[test]
fn sst_rough_plate_state_matches_cpu_f32() {
    sst_plate!(Precision::F32, Scheme::Implicit(ImplicitParams::VISCOUS), 10.0, 1e-3 * 0.97);
}

#[test]
fn sst_rough_plate_state_matches_cpu_df32() {
    sst_plate!(Precision::Extended, Scheme::Implicit(ImplicitParams::VISCOUS), 10.0, 1e-3 * 0.97);
}

/// The explicit scheme with the SST update after the Runge-Kutta step (the last-stage quantities and the first-stage time step); the explicit
/// time step is small, so the turbulence hardly grows in 50 iterations: all cells are compared (and the `k` residual printed).
#[test]
fn sst_plate_explicit_state_matches_cpu() {
    sst_plate!(Precision::Extended, Scheme::Explicit, 0.0);
}

/// The G3 body (tangent ogive 0.30 x 0.10 m + 0.60 m tube, ISA at sea level, Mach 0.5), coarse grid with the flat base and wake block.
fn g3_case() -> Case {
    let (d, mach) = (0.10, 0.5);
    let spec = BodySpec { nose: NoseSpec { shape: NoseShape::TangentOgive, length: 0.30, diameter: d }, aft: vec![AftSpec::Tube { length: 0.60 }] };
    let (body, _): (BaseBody, _) = spec.to_internal().unwrap();
    let air = Atmosphere::Isa { altitude_m: 0.0 }.state().unwrap();
    let re_c = air.density * air.sound_speed * d / air.viscosity;
    Case::viscous(mach, body, 0, Wake::Base, re_c, 110.4 / air.temperature_k, mach)
}

/// The C2 motor case on the G3 body (Mach 0.8, coarse grid): a Mach 2.5 jet from a 60 mm exit in the 100 mm base, built by the facade
/// (`Analysis::engine_cases`) so that the jet state is derived in one place.
fn motor_case() -> Case {
    let spec = BodySpec { nose: NoseSpec { shape: NoseShape::TangentOgive, length: 0.30, diameter: 0.10 }, aft: vec![AftSpec::Tube { length: 0.60 }] };
    let motor = MotorSpec { exit_diameter: 0.06, exit_mach: 2.5, exit_pressure_pa: 101325.0, total_temperature_k: 1500.0 };
    let flight = FlightConditions { atmosphere: Atmosphere::Isa { altitude_m: 0.0 } };
    let analysis = Analysis::new(&spec).unwrap().resolution(Resolution::Coarse).viscous(flight).motor(motor);
    analysis.engine_cases(&[0.8]).unwrap().remove(0)
}

/// The motor case: 30 iterations from the free stream (5 first order) vs CPU; the jet rows of the base face (Euler flux, viscous ghost, implicit
/// left-hand side, turbulence ghost) are all exercised.
macro_rules! motor_check {
    ($prec:expr, $scheme:expr, $tol:expr) => {{
        let ctx = luma::default_context().unwrap();
        let mut case = motor_case();
        case.numerics.first_order_iters = 5;
        if let Some(scheme) = $scheme {
            case.numerics.scheme = scheme;
        }
        let mut cpu = case.build_solver();
        let nozzle = cpu.nozzle().expect("a motor case has a nozzle");
        let mut lazy = GpuSolver::from_host(&ctx, &cpu, $prec).unwrap();
        assert_eq!(lazy.nozzle(), Some(nozzle));
        let res_cpu: Vec<f64> = (0..30).map(|_| cpu.step()).collect();
        let res = lazy.run(30).unwrap();
        let (max, mean) = state_diff(&lazy.state_f64().unwrap(), &cpu.u);
        println!(
            "motor {} {}: jet rows {}, max |dU|/(1+|U|) = {max:.2e}, mean {mean:.2e}, max rel residual diff {:.2e}, residual {:.3e} -> {:.3e}",
            format!("{:?}", $prec),
            stringify!($scheme),
            nozzle.j_end,
            residual_diff(&res, &res_cpu),
            res_cpu[1],
            res_cpu[29]
        );
        assert!(max <= $tol && res.iter().all(|r| r.is_finite()), "motor {}: state difference {max:e}", format!("{:?}", $prec));
    }};
}

#[test]
fn motor_state_matches_cpu_f32() {
    motor_check!(Precision::F32, None::<Scheme>, 1e-3);
}

#[test]
fn motor_state_matches_cpu_df32() {
    motor_check!(Precision::Extended, None::<Scheme>, 1e-4);
}

/// The explicit scheme (SSP-RK3, SST update after the step).
#[test]
fn motor_explicit_state_matches_cpu_df32() {
    motor_check!(Precision::Extended, Some(Scheme::Explicit), 1e-6);
}

/// `set_nozzle` outside the order block, nozzle, viscous is refused.
#[test]
fn set_nozzle_order_is_checked() {
    let ctx = luma::default_context().unwrap();
    let case = motor_case();
    let host = case.build_solver();
    let nozzle = host.nozzle().unwrap();
    let mut s = GpuSolver::new(&ctx, &host.grid, &host.cfg, Precision::F32).unwrap();
    let err = s.set_nozzle(Some(nozzle)).unwrap_err();
    assert_eq!(err.to_string(), "set_nozzle must follow set_block and precede set_viscous");
    s.set_block(host.block()).unwrap();
    s.set_nozzle(Some(nozzle)).unwrap();
    s.set_viscous(host.viscous(), &host.grid).unwrap();
    assert!(s.set_nozzle(None).is_err());
}

/// Viscous body with the solid block (no-slip body surface, base wall and the axis behind the base as a slip wall): 30 iterations vs CPU.
macro_rules! body_check {
    ($prec:expr, $tol:expr) => {{
        let ctx = luma::default_context().unwrap();
        let mut case = g3_case();
        case.numerics.first_order_iters = 5;
        let mut cpu = case.build_solver();
        assert!(cpu.block().is_some() && cpu.viscous().is_some());
        let mut lazy = GpuSolver::from_host(&ctx, &cpu, $prec).unwrap();
        let res_cpu: Vec<f64> = (0..30).map(|_| cpu.step()).collect();
        let res = lazy.run(30).unwrap();
        let (max, mean) = state_diff(&lazy.state_f64().unwrap(), &cpu.u);
        let (turb, _) = lazy.turb_state().unwrap();
        // the fluid cells where `k` has moved off its free-stream value
        let (mut dk, mut dw, mut n) = (0.0f64, 0.0f64, 0);
        for (t, c) in turb.iter().zip(&cpu.turb) {
            if c[0] > 0.0 && (c[0] / K_INF - 1.0).abs() > 1e-3 {
                dk = dk.max((t[0] - c[0]).abs() / c[0]);
                dw = dw.max((t[1] - c[1]).abs() / c[1]);
                n += 1;
            }
        }
        println!(
            "G3 body {}: max |dU|/(1+|U|) = {max:.2e}, mean {mean:.2e}, k, omega on {n} changed cells {dk:.2e}, {dw:.2e}, max rel residual diff {:.2e}, residual {:.3e} -> {:.3e}",
            format!("{:?}", $prec),
            residual_diff(&res, &res_cpu),
            res_cpu[1],
            res_cpu[29]
        );
        assert!(n > 0 && max <= $tol && res.iter().all(|r| r.is_finite()), "G3 body {}: state difference {max:e}", format!("{:?}", $prec));
    }};
}

#[test]
fn body_state_matches_cpu_f32() {
    body_check!(Precision::F32, 1e-3);
}

#[test]
fn body_state_matches_cpu_df32() {
    body_check!(Precision::Extended, 1e-3);
}

/// The same body with the explicit scheme (SSP-RK3, the SST update from the last stage).
#[test]
fn body_explicit_state_matches_cpu_df32() {
    let ctx = luma::default_context().unwrap();
    let mut case = g3_case();
    case.numerics.first_order_iters = 5;
    case.numerics.scheme = Scheme::Explicit;
    let mut cpu = case.build_solver();
    let mut lazy = GpuSolver::from_host(&ctx, &cpu, Precision::Extended).unwrap();
    let res_cpu: Vec<f64> = (0..30).map(|_| cpu.step()).collect();
    let res = lazy.run(30).unwrap();
    let (max, mean) = state_diff(&lazy.state_f64().unwrap(), &cpu.u);
    println!("G3 body explicit df32s: max |dU|/(1+|U|) = {max:.2e}, mean {mean:.2e}, max rel residual diff {:.2e}", residual_diff(&res, &res_cpu));
    assert!(max <= 1e-6, "state difference {max:e}");
}

/// The G3 body iterated to the stop rule of the CPU run (`bowshock_core::engine::run::run_on`), the lazy solver for the same number of iterations from the free
/// stream: the total drag within 1 %. (The CPU run takes about half a minute.)
#[test]
#[ignore = "runs the CPU solver to convergence (about half a minute)"]
fn converged_body_total_drag_matches_cpu() {
    let ctx = luma::default_context().unwrap();
    let case = g3_case();
    let mut s = case.build_solver();
    let cpu = run_on(&case, &mut s, false, None);
    println!("CPU: {} after {} iterations ({:.1} orders)", cpu.stop_reason, cpu.iters, cpu.drop_orders);
    macro_rules! lazy_run {
        ($prec:expr) => {{
            let host = case.build_solver();
            let mut lazy = GpuSolver::from_host(&ctx, &host, $prec).unwrap();
            let t0 = std::time::Instant::now();
            let mut left = cpu.iters;
            while left > 0 {
                let n = left.min(500);
                lazy.run(n).unwrap();
                left -= n;
            }
            let seconds = t0.elapsed().as_secs_f64();
            let mut post = case.build_solver();
            post.u = lazy.state_f64().unwrap();
            let r = finish(&case, &post, Vec::new(), 0.0, Vec::new(), StopReason::MaxIterations);
            println!(
                "{} ({seconds:.1} s): total CD {:.5} cpu {:.5} ({:+.3} %); cd_friction {:.5} cpu {:.5}; cd_base {:.5} cpu {:.5}; cp_base {:.4} cpu {:.4}; y+ max {:.3} cpu {:.3}",
                format!("{:?}", $prec),
                r.cd_total,
                cpu.cd_total,
                100.0 * (r.cd_total / cpu.cd_total - 1.0),
                r.cd_friction,
                cpu.cd_friction,
                r.cd_base,
                cpu.cd_base,
                r.cp_base,
                cpu.cp_base,
                r.y_plus_max,
                cpu.y_plus_max
            );
            r.cd_total
        }};
    }
    let (cd32, cd_df) = (lazy_run!(Precision::F32), lazy_run!(Precision::Extended));
    for (name, cd) in [("f32", cd32), ("df32", cd_df)] {
        assert!((cd / cpu.cd_total - 1.0).abs() < 0.01, "{name}: total CD {cd} vs CPU {}", cpu.cd_total);
    }
}

/// Every `Staging` computes the same viscous iteration (the stagings only decide which expressions are evaluated on their own): the SST plate,
/// 10 iterations in f32.
#[test]
fn stagings_agree_on_the_sst_plate() {
    use bowshock_cfd::gpu::Staging;
    let ctx = luma::default_context().unwrap();
    let mut cpu = plate_solver(true, Scheme::Implicit(ImplicitParams::VISCOUS));
    let hosts: Vec<_> = [Staging::Fusion, Staging::Lines, Staging::Fluxes, Staging::Split]
        .into_iter()
        .map(|staging| {
            let mut lazy = bowshock_cfd::gpu::f32s::LazySolver::from_host(&ctx, &cpu).unwrap();
            lazy.staging = staging;
            (staging, lazy)
        })
        .collect();
    for _ in 0..10 {
        cpu.step();
    }
    for (staging, mut lazy) in hosts {
        lazy.run(10, 10).unwrap();
        let (max, mean) = state_diff(&lazy.state_f64().unwrap(), &cpu.u);
        println!("SST plate f32s {staging:?}: max |dU|/(1+|U|) = {max:.2e}, mean {mean:.2e}");
        assert!(max <= 1e-3, "{staging:?}: state difference {max:e}");
    }
}
