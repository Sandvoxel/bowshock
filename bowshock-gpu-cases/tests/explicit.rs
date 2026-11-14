//! Accuracy of the explicit scheme of the array-op solver (`bowshock_gpu::{f32s, df32s}::LazySolver`, every `Staging`) against the f64 CPU
//! `Solver`, and of the `GpuSolver` handle on the same set-ups: the CPU and the GPU solver start from the same state, run the same
//! iterations, and the conserved state and the density residual history are compared,
//!
//! ```text
//! state:    max over cells and components of |dU| / (1 + |U_cpu|)
//! residual: max over iterations of |r_gpu - r_cpu| / r_cpu
//! ```
//!
//! Targets: f32 (`Precision::F32`): state < 1e-5, residual < 1e-4; double-float (`Precision::Extended`): state < 3e-7 and residual
//! < 2e-7 (5e-7 on the transonic grid). The numbers are printed (`cargo test --release -- --nocapture`) whether or not
//! they meet the targets; every `both`-based test checks all its rows and fails at the end if one misses.

mod common;

use bowshock::body_grid::{Body, Nose};
use bowshock_cases::cone::{build_solver, ConeCase};
use bowshock_cases::ogive::OgiveCase;
use bowshock_cases::subsonic::{fox_case, subsonic_case};
use bowshock_cases::transonic::transonic_case;
use bowshock_core::flux::Limiter;
use bowshock_core::gas::cons_to_prim;
use bowshock_core::solver::Scheme;
use bowshock_core::solver::{Bc, Solver};
use bowshock_gpu::{GpuSolver, Precision, Staging};
use bowshock_reference::taylor_maccoll::solve_cone;
use common::{far_field_box, perturbed_state};

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
/// run them (`res_cpu`). Every test uses a context of its own for isolation (the shared default context works too: see
/// `threads_on_the_default_context_agree` in `loops.rs`).
macro_rules! run_lazy {
    ($module:ident, $ctx:expr, $staging:expr, $grid:expr, $cfg:expr, $low_mach:expr, $block:expr, $u0:expr, $n:expr) => {{
        let mut lazy = bowshock_gpu::$module::LazySolver::new($ctx, $grid, $cfg).unwrap();
        lazy.set_block($block).unwrap();
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
    // `LAZY_SHARED_CONTEXT=1` runs every test on `luma::default_context()` (the tests run in parallel threads), otherwise each
    // call builds a context of its own.
    let ctx = if std::env::var_os("LAZY_SHARED_CONTEXT").is_some() { luma::default_context().unwrap() } else { luma::Context::new().unwrap() };
    for staging in [Staging::Fusion, Staging::Lines, Staging::Fluxes, Staging::Split] {
        let (u, r) = run_lazy!(f32s, &ctx, staging, &cpu.grid, &cpu.cfg, cpu.low_mach, cpu.block(), u0, n);
        rows.push((format!("{label} [{staging:?}]"), "f32", diff(&u, &r, cpu, res_cpu), res_ext));
        let (u, r) = run_lazy!(df32s, &ctx, staging, &cpu.grid, &cpu.cfg, cpu.low_mach, cpu.block(), u0, n);
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

/// Base / wake grids with a solid block, explicit scheme, 20 iterations (5 first order): (a) ogive-cylinder with a base and no
/// sting (`j_sting = 0`), (b) Fox TM X-2439 cone-cylinder-boattail on a sting (`j_sting > 0`).
#[test]
fn base_block_explicit_state_matches_cpu() {
    let mut rows = Vec::new();
    let cases = [("ogive base M=0.5", subsonic_case(0.5, Nose::TangentOgive { fineness: 3.0 }, 0, true)), ("fox sting M=0.6", fox_case(false, 0.6, 0))];
    for (label, mut case) in cases {
        case.numerics.first_order_iters = 5;
        case.numerics.scheme = Scheme::Explicit;
        let mut cpu = case.build_solver();
        if cpu.scheme() != Scheme::Explicit {
            cpu.set_scheme(Scheme::Explicit);
        }
        let u0 = cpu.u.clone();
        let res_cpu: Vec<f64> = (0..20).map(|_| cpu.step()).collect();
        let (ni, nj, b) = (cpu.grid.ni, cpu.grid.nj, cpu.block().expect("block"));
        both(&format!("{label} {ni}x{nj} block {b:?}"), &cpu, &u0, &res_cpu, &mut rows, 2e-7);
    }
    report("base_block_explicit_state_matches_cpu", &rows);
}

/// Free-stream preservation: the characteristic far field at M = 0.9 (all far-field faces subsonic) and 1.3 keeps the uniform free stream
/// to round-off on the CPU and to the geometry rounding on the GPU (f32 and double-float): no spurious waves from the BC.
#[test]
fn characteristic_far_field_preserves_the_free_stream() {
    let ctx = luma::default_context().unwrap();
    for mach in [0.9, 1.3] {
        let mut cpu = far_field_box(mach, Bc::Characteristic);
        let fs = bowshock_core::gas::prim_to_cons(&cpu.cfg.freestream);
        let dev = |u: &[[f64; 4]]| u.iter().fold(0.0f64, |m, q| (0..4).fold(m, |m, k| m.max((q[k] - fs[k]).abs() / (1.0 + fs[k].abs()))));
        for _ in 0..30 {
            cpu.step();
        }
        let dev_cpu = dev(&cpu.u);
        println!("M = {mach}: CPU max deviation from the free stream after 30 iterations: {dev_cpu:.2e}");
        assert!(dev_cpu < 1e-12, "CPU: free stream not preserved at M = {mach}: {dev_cpu:e}");
        for prec in [Precision::F32, Precision::Extended] {
            let mut gpu = GpuSolver::new(&ctx, &cpu.grid, &cpu.cfg, prec).unwrap();
            gpu.run(30).unwrap();
            let d = dev(&gpu.state_f64().unwrap());
            println!("M = {mach} {prec:?}: max deviation from the free stream after 30 iterations: {d:.2e}");
            // f32 geometry (and the f32 operations of the left-hand side) bound the closure of a uniform flow; no growth in 30 iterations
            assert!(d < 1e-5 && (prec == Precision::F32 || d < 2e-6), "{prec:?} M = {mach}: free stream deviates by {d:e}");
        }
    }
}

/// 400x200 (3 iterations, no start-up phase): residual history and state of `GpuSolver` vs CPU on a grid with many workgroups.
#[test]
fn large_grid_residual_history_matches_cpu() {
    let ctx = luma::default_context().unwrap();
    let tm = solve_cone(2.0, 10f64.to_radians()).unwrap();
    let mut case = ConeCase::new(2.0, 10.0, 400, 200);
    case.first_order_iters = 0;
    let mut cpu = build_solver(&case, tm.beta);
    let res_cpu: Vec<f64> = (0..3).map(|_| cpu.step()).collect();
    for prec in [Precision::F32, Precision::Extended] {
        let mut gpu = GpuSolver::new(&ctx, &cpu.grid, &cpu.cfg, prec).unwrap();
        let res_gpu = gpu.run(3).unwrap();
        let worst_res = common::residual_diff(&res_gpu, &res_cpu);
        let worst = common::state_diff(&gpu.state_f64().unwrap(), &cpu.u);
        println!("{prec:?} 400x200: residual diff {worst_res:.2e}, state diff {worst:.2e}");
        assert!(worst_res < 1e-4 && worst < 1e-5, "{prec:?}: residual diff {worst_res:e}, state diff {worst:e}");
    }
}

/// A small cone grid (60x30, 15 deg, minmod) converged by the double-float solver: it reaches the CPU's 6-orders criterion (and keeps
/// dropping), and its state agrees with the f64 CPU solver after the same number of iterations. The f32 solver's orders are printed.
#[test]
fn extended_precision_reaches_six_orders_on_a_small_grid() {
    let ctx = luma::default_context().unwrap();
    let tm = solve_cone(2.0, 15f64.to_radians()).unwrap();
    let case = ConeCase::new(2.0, 15.0, 60, 30);
    let host = build_solver(&case, tm.beta);
    for prec in [Precision::F32, Precision::Extended] {
        let mut gpu = GpuSolver::new(&ctx, &host.grid, &host.cfg, prec).unwrap();
        let mut res: Vec<f64> = Vec::new();
        let mut peak = 0.0f64;
        while res.len() < 4000 {
            for r in gpu.run(50).unwrap() {
                if res.len() >= 20 {
                    peak = peak.max(r);
                }
                res.push(r);
            }
            if prec == Precision::Extended && (peak / res[res.len() - 1]).log10() >= 6.0 {
                break;
            }
        }
        let best = res.iter().skip(20).cloned().fold(f64::INFINITY, f64::min);
        let orders = (peak / best).log10();
        println!("{prec:?}: {} iterations, peak {peak:.2e}, min {best:.2e}, {orders:.2} orders", res.len());
        if prec == Precision::Extended {
            assert!(orders >= 6.0, "extended solver only reached {orders} orders");
            let mut cpu = build_solver(&case, tm.beta);
            for _ in 0..res.len() {
                cpu.step();
            }
            let worst = common::state_diff(&gpu.state_f64().unwrap(), &cpu.u);
            println!("Extended: max |dU|/(1+|U|) vs CPU after the same number of iterations = {worst:.2e}");
            assert!(worst < 1e-5, "extended precision state differs from the CPU by {worst:e}");
        }
    }
}

/// Ogive-cylinder (M = 1.98, coarse grid 114x50: free-stream inflow, extrapolation outflow, slip wall/axis, free-stream outer boundary):
/// state after 20 iterations (5 first order) vs CPU in both precisions, then the wall Cp and forebody CD after 500 iterations vs CPU.
#[test]
fn ogive_state_and_wall_cp_match_cpu() {
    let ctx = luma::default_context().unwrap();
    let mut case = OgiveCase::level(1.98, 0);
    case.first_order_iters = 5;
    let mut cpu = case.build_solver();
    let res_cpu: Vec<f64> = (0..20).map(|_| cpu.step()).collect();
    for prec in [Precision::F32, Precision::Extended] {
        let mut gpu = GpuSolver::new(&ctx, &case.build_solver().grid, &cpu.cfg, prec).expect("GPU solver");
        let res_gpu = gpu.run(20).expect("GPU run");
        let worst = common::state_diff(&gpu.state_f64().unwrap(), &cpu.u);
        let worst_res = common::residual_diff(&res_gpu, &res_cpu);
        println!("ogive {prec:?}: max |dU|/(1+|U|) = {worst:.2e}, max rel residual diff = {worst_res:.2e}");
        assert!(worst < 1e-5 && worst_res < 1e-4, "{prec:?}: state {worst:e}, residual {worst_res:e}");
        if prec == Precision::Extended {
            assert!(worst < 3e-7 && worst_res < 2e-7, "extended precision state {worst:e}, residual {worst_res:e}");
        }
    }

    // 500 iterations (MUSCL from iteration 5): wall Cp and forebody CD
    let n = 500;
    let mut cpu = case.build_solver();
    for _ in 0..n {
        cpu.step();
    }
    let post_cpu = bowshock_gpu_cases::ogive::postprocess(&case, &cpu.grid, &cpu.cfg.freestream, &cpu.u);
    let mut gpu = GpuSolver::new(&ctx, &cpu.grid, &cpu.cfg, Precision::Extended).unwrap();
    for _ in 0..n / 50 {
        gpu.run(50).unwrap();
    }
    let post_gpu = bowshock_gpu_cases::ogive::postprocess(&case, &cpu.grid, &cpu.cfg.freestream, &gpu.state_f64().unwrap());
    let max_dcp = post_gpu.wall.iter().zip(&post_cpu.wall).fold(0.0f64, |m, (a, b)| m.max((a.cp - b.cp).abs()));
    let cd_rel = (post_gpu.cd_nose - post_cpu.cd_nose) / post_cpu.cd_nose;
    println!(
        "ogive after {n} iterations: CD gpu {:.6} cpu {:.6} (rel {cd_rel:+.2e}), max |dCp| over the wall {max_dcp:.2e}",
        post_gpu.cd_nose, post_cpu.cd_nose
    );
    assert!(cd_rel.abs() < 1e-5, "forebody CD differs by {cd_rel:e}");
    assert!(max_dcp < 1e-5, "wall Cp differs by {max_dcp:e}");
}
