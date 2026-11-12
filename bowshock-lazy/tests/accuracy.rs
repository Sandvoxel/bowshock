//! Accuracy of the array-op solver against the f64 CPU `Solver`, on the set-ups of
//! `bowshock-gpu-cases/tests/gpu_vs_cpu.rs` (`state_after_20_iterations_matches_cpu`,
//! `characteristic_far_field_matches_cpu_on_a_perturbed_state`, `transonic_state_after_20_iterations_matches_cpu`):
//! the CPU and the lazy solver start from the same state, run the same iterations, and the conserved state and the density
//! residual history are compared,
//!
//! ```text
//! state:    max over cells and components of |dU| / (1 + |U_cpu|)
//! residual: max over iterations of |r_lazy - r_cpu| / r_cpu
//! ```
//!
//! Targets are the tolerances those tests use for the hand-written kernels of the same precision:
//! f32 (`Precision::F32`): state < 1e-5, residual < 1e-4; double-float (`Precision::Extended`): state < 3e-7 and residual
//! < 2e-7 (5e-7 on the transonic grid). The numbers are printed (`cargo test --release -- --nocapture`) whether or not
//! they meet the targets; every test checks all its rows and fails at the end if one misses.

use bowshock::body_grid::{Body, Nose};
use bowshock_cases::cone::{build_solver, ConeCase};
use bowshock_cases::transonic::transonic_case;
use bowshock_core::flux::Limiter;
use bowshock_core::gas::{cons_to_prim, freestream, prim_to_cons};
use bowshock_lazy::Staging;
use bowshock_core::grid::Grid;
use bowshock_core::solver::{Bc, Config, Solver};
use bowshock_reference::taylor_maccoll::solve_cone;

/// State and residual-history differences of one run.
struct Diff {
    state: f64,
    pressure: f64,
    residual: f64,
}

fn diff(u_lazy: &[[f64; 4]], res_lazy: &[f64], cpu: &Solver, res_cpu: &[f64]) -> Diff {
    let (mut state, mut pressure) = (0.0f64, 0.0f64);
    for (a, b) in u_lazy.iter().zip(&cpu.u) {
        for k in 0..4 {
            state = state.max((a[k] - b[k]).abs() / (1.0 + b[k].abs()));
        }
        pressure = pressure.max((cons_to_prim(a)[3] - cons_to_prim(b)[3]).abs() / cons_to_prim(b)[3]);
    }
    let residual = res_lazy.iter().zip(res_cpu).fold(0.0f64, |m, (a, b)| m.max((a - b).abs() / b));
    Diff { state, pressure, residual }
}

/// Targets of the hand-written kernels: `(state, residual)`.
fn target(precision: &str, res_ext: f64) -> (f64, f64) {
    if precision == "f32" {
        (1e-5, 1e-4)
    } else {
        (3e-7, res_ext)
    }
}

/// Runs the lazy solver of one precision from `u0` for `n` iterations and compares with the CPU solver, which has already
/// run them (`res_cpu`). Every test uses a context of its own: tests run in parallel threads, and the global default context
/// of Luma gives wrong results when several threads trace and replay on it at once (README, Luma issues).
macro_rules! run_lazy {
    ($module:ident, $ctx:expr, $staging:expr, $grid:expr, $cfg:expr, $low_mach:expr, $u0:expr, $n:expr) => {{
        let mut lazy = bowshock_lazy::$module::LazySolver::new($ctx, $grid, $cfg).unwrap();
        lazy.staging = $staging;
        lazy.set_low_mach($low_mach);
        lazy.set_state($u0).unwrap();
        let res = lazy.run($n, 16).unwrap();
        (lazy.state_f64().unwrap(), res)
    }};
}

/// Both precisions and every staging on one set-up; adds the rows `(label, precision, diff)`.
fn both(label: &str, cpu: &Solver, u0: &[[f64; 4]], res_cpu: &[f64], rows: &mut Vec<(String, &'static str, Diff, f64)>, res_ext: f64) {
    let n = res_cpu.len();
    let ctx = luma::Context::new().unwrap();
    for staging in [Staging::Fusion, Staging::Lines, Staging::Split] {
        let (u, r) = run_lazy!(f32s, &ctx, staging, &cpu.grid, &cpu.cfg, cpu.low_mach, u0, n);
        rows.push((format!("{label} [{staging:?}]"), "f32", diff(&u, &r, cpu, res_cpu), res_ext));
        let (u, r) = run_lazy!(df32s, &ctx, staging, &cpu.grid, &cpu.cfg, cpu.low_mach, u0, n);
        rows.push((format!("{label} [{staging:?}]"), "df32", diff(&u, &r, cpu, res_cpu), res_ext));
    }
}

fn report(test: &str, rows: &[(String, &'static str, Diff, f64)]) {
    println!("\n{test}");
    println!("| case | precision | max abs dU/(1+abs U) | max rel dp | max rel residual diff | target state / residual |");
    println!("|---|---|---|---|---|---|");
    let mut failed = Vec::new();
    for (label, prec, d, res_ext) in rows {
        let (ts, tr) = target(if *prec == "f32" { "f32" } else { "ext" }, *res_ext);
        println!("| {label} | {prec} | {:.2e} | {:.2e} | {:.2e} | {ts:.0e} / {tr:.0e} |", d.state, d.pressure, d.residual);
        if !(d.state < ts && d.residual < tr) {
            failed.push(format!("{label} {prec}: state {:e} (target {ts:e}), residual {:e} (target {tr:e})", d.state, d.residual));
        }
    }
    assert!(failed.is_empty(), "targets missed: {failed:#?}");
}

/// 40x20 cone grid, M = 2, 15 degree cone, both limiters, 20 iterations from the free stream (the first 5 first order).
#[test]
fn cone_state_after_20_iterations_matches_cpu() {
    let tm = solve_cone(2.0, 15f64.to_radians()).unwrap();
    let mut rows = Vec::new();
    for limiter in [Limiter::Minmod, Limiter::VanAlbada] {
        let mut case = ConeCase::new(2.0, 15.0, 40, 20);
        case.limiter = limiter;
        case.first_order_iters = 5;
        let mut cpu = build_solver(&case, tm.beta);
        let u0 = cpu.u.clone();
        let res_cpu: Vec<f64> = (0..20).map(|_| cpu.step()).collect();
        both(&format!("cone 40x20 {limiter:?}"), &cpu, &u0, &res_cpu, &mut rows, 2e-7);
    }
    report("cone_state_after_20_iterations_matches_cpu", &rows);
}

/// Rectangular axisymmetric box with the characteristic far field on the three outer boundaries (as in `gpu_vs_cpu.rs`).
fn far_field_box(mach: f64, outflow: Bc) -> Solver {
    let (ni, nj) = (40usize, 20usize);
    let mut x = vec![0.0; (ni + 1) * (nj + 1)];
    let mut r = x.clone();
    for j in 0..=nj {
        for i in 0..=ni {
            x[j * (ni + 1) + i] = 4.0 * (i as f64 / ni as f64).powf(1.15);
            r[j * (ni + 1) + i] = 2.0 * (j as f64 / nj as f64).powf(1.1);
        }
    }
    let cfg = Config {
        cfl: 0.5,
        limiter: Limiter::Minmod,
        freestream: freestream(mach),
        bc: [Bc::Characteristic, outflow, Bc::SlipWall, Bc::Characteristic],
        first_order_iters: 5,
    };
    Solver::new(Grid::from_nodes(ni, nj, x, r), cfg)
}

/// Smooth density / pressure / transverse-velocity perturbation of the free stream.
fn perturbed_state(s: &Solver) -> Vec<[f64; 4]> {
    let fs = s.cfg.freestream;
    (0..s.grid.ni * s.grid.nj)
        .map(|c| {
            let (xc, rc) = (s.grid.xc[c], s.grid.rc[c]);
            let g = (-((xc - 3.0).powi(2) + (rc - 1.4).powi(2)) / 0.8).exp() + 0.5 * (-((xc - 0.6).powi(2) + (rc - 1.6).powi(2)) / 0.5).exp();
            prim_to_cons(&[fs[0] * (1.0 + 0.25 * g), fs[1] + 0.2 * g, 0.3 * g * (0.5 * xc).sin(), fs[3] * (1.0 + 0.4 * g)])
        })
        .collect()
}

/// Characteristic far field and static-pressure outflow on a perturbed state, 20 iterations (5 first order), M = 0.9 and 1.3.
#[test]
fn characteristic_far_field_matches_cpu_on_a_perturbed_state() {
    let mut rows = Vec::new();
    for (mach, outflow) in [(0.9, Bc::Characteristic), (0.9, Bc::PressureOutflow), (1.3, Bc::Characteristic), (1.3, Bc::PressureOutflow)] {
        let mut cpu = far_field_box(mach, outflow);
        let u0 = perturbed_state(&cpu);
        cpu.u = u0.clone();
        let res_cpu: Vec<f64> = (0..20).map(|_| cpu.step()).collect();
        both(&format!("box M={mach} {outflow:?}"), &cpu, &u0, &res_cpu, &mut rows, 2e-7);
    }
    report("characteristic_far_field_matches_cpu_on_a_perturbed_state", &rows);
}

/// Transonic case: ogive-cylinder in the large domain (characteristic far field, level 0: 144x70), M = 0.9, 20 iterations
/// (5 first order), explicit scheme, minmod, no low-Mach correction.
#[test]
fn transonic_state_after_20_iterations_matches_cpu() {
    let mut case = transonic_case(0.9, 0, Body { nose: Nose::TangentOgive { fineness: 3.0 } }, 6.0);
    case.numerics.first_order_iters = 5;
    let mut cpu = case.build_solver();
    let u0 = cpu.u.clone();
    let res_cpu: Vec<f64> = (0..20).map(|_| cpu.step()).collect();
    let mut rows = Vec::new();
    let (ni, nj) = (cpu.grid.ni, cpu.grid.nj);
    both(&format!("transonic M=0.9 {ni}x{nj}"), &cpu, &u0, &res_cpu, &mut rows, 5e-7);
    report("transonic_state_after_20_iterations_matches_cpu", &rows);
}

/// The low-Mach correction (`Solver::low_mach`) on the subsonic far-field box.
#[test]
fn low_mach_correction_matches_cpu() {
    let mut cpu = far_field_box(0.3, Bc::Characteristic);
    cpu.low_mach = true;
    cpu.cfg.limiter = Limiter::VanAlbada;
    let u0 = perturbed_state(&cpu);
    cpu.u = u0.clone();
    let res_cpu: Vec<f64> = (0..20).map(|_| cpu.step()).collect();
    let mut rows = Vec::new();
    both("box M=0.3 low-Mach van Albada", &cpu, &u0, &res_cpu, &mut rows, 2e-7);
    report("low_mach_correction_matches_cpu", &rows);
}
