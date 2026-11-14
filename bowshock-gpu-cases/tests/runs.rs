//! GPU runs of whole cases through the facade runner (`bowshock::gpu::GpuRunner`, the GPU counterpart of `bowshock::run::run_on`) against
//! the CPU runner with the same stopping rules: transonic runs and warm starts, subsonic sting / base runs, the base-pressure sample, a
//! cold restart on a reused solver, converged implicit states, and `GpuSolver::set_config` (a Mach sweep point).

mod common;

use bowshock::body_grid::{Body, Nose};
use bowshock::case::Case;
use bowshock::gpu::{BaseSampler, GpuRun, GpuRunner};
use bowshock::result::{PointResult, StopReason};
use bowshock::run::{base_state, run_on};
use bowshock_cases::subsonic::subsonic_case;
use bowshock_cases::transonic::transonic_case;
use bowshock_core::flux::Limiter;
use bowshock_core::solver::{Bc, Scheme};
use bowshock_gpu::{GpuSolver, Precision};
use common::{far_field_box, perturbed_state, residual_diff, state_diff};

/// Default transonic case: tangent ogive of fineness 3 plus a 6 d cylinder, grid level 0.
fn transonic_level(mach: f64) -> Case {
    transonic_case(mach, 0, Body { nose: bowshock::body_grid::Nose::TangentOgive { fineness: 3.0 } }, 6.0)
}

/// Short cold-start run at M = 0.9 followed by a warm-started run at M = 0.95 (`run_on` of both solvers with the same
/// arguments, 200 iterations each): residual history, forebody CD and wall Cp vs CPU.
#[test]
fn transonic_run_and_warm_start_match_cpu() {
    let mut case = transonic_level(0.9);
    case.numerics.max_iter = 200;
    case.numerics.first_order_iters = 20;
    let mut s = case.build_solver();
    let cpu0 = run_on(&case, &mut s, false, None);
    let mut gt = GpuRunner::new(&case, Precision::Extended).unwrap();
    let gpu0 = gt.run_on(&case, false, None).unwrap();
    let check = |tag: &str, cpu: &PointResult, gpu: &GpuRun| {
        let g = &gpu.result;
        assert_eq!((g.iters, gpu.iters_exec), (cpu.iters, cpu.iters), "{tag}: iteration counts");
        let res = g.residual.iter().zip(&cpu.residual).fold(0.0f64, |m, (a, b)| m.max((a - b).abs() / b));
        let dcp = g.wall.iter().zip(&cpu.wall).fold(0.0f64, |m, (a, b)| m.max((a.cp - b.cp).abs()));
        let cd_rel = (g.cd_nose - cpu.cd_nose) / cpu.cd_nose;
        let dcd_hist = g.monitor_history.iter().zip(&cpu.monitor_history).fold(0.0f64, |m, (a, b)| m.max((a.1 - b.1).abs()));
        println!("{tag}: CD gpu {:.6} cpu {:.6} (rel {cd_rel:+.2e}), max |dCp| {dcp:.2e}, max rel residual diff {res:.2e}, max |dCD history| {dcd_hist:.2e}, farfield dp {:.2e} / {:.2e}", g.cd_nose, cpu.cd_nose, g.farfield_dp, cpu.farfield_dp);
        assert!(res < 1e-4 && dcp < 1e-5 && cd_rel.abs() < 1e-5 && dcd_hist < 1e-6, "{tag}: GPU differs from CPU");
    };
    check("M = 0.9 cold start", &cpu0, &gpu0);
    case.mach = 0.95;
    let cpu1 = run_on(&case, &mut s, true, Some(cpu0.residual_peak));
    let gpu1 = gt.run_on(&case, true, Some(gpu0.result.residual_peak)).unwrap();
    check("M = 0.95 warm start", &cpu1, &gpu1);
}

/// Short cold start of the sting and base cases through `run_on` of both solvers (300 iterations, van Albada, low Mach, implicit): the
/// residual history, wall Cp, drag breakdown and base pressure of the GPU vs CPU.
#[test]
fn subsonic_run_matches_cpu() {
    for base in [false, true] {
        let mut case = subsonic_case(0.5, Nose::TangentOgive { fineness: 3.0 }, 0, base);
        case.numerics.max_iter = 300;
        let mut s = case.build_solver();
        let cpu = run_on(&case, &mut s, false, None);
        let mut gs = GpuRunner::new(&case, Precision::Extended).unwrap();
        let gpu = gs.run_on(&case, false, None).unwrap();
        let g = &gpu.result;
        assert_eq!((g.iters, gpu.iters_exec), (cpu.iters, cpu.iters));
        let res = g.residual.iter().zip(&cpu.residual).fold(0.0f64, |m, (a, b)| m.max((a - b).abs() / b));
        let dcp = g.wall.iter().zip(&cpu.wall).fold(0.0f64, |m, (a, b)| m.max((a.cp - b.cp).abs()));
        println!(
            "subsonic M = 0.5 base = {base}: residual {:.2e} -> {:.2e} (gpu {:.2e}), max rel residual diff {res:.2e}, max |dCp| {dcp:.2e}, CD_nose gpu {:.5} cpu {:.5}, Cp_base gpu {:.5} cpu {:.5}",
            cpu.residual[0], cpu.residual.last().unwrap(), g.residual.last().unwrap(), g.cd_nose, cpu.cd_nose, g.cp_base, cpu.cp_base
        );
        assert!(dcp < 2e-4 && (g.cd_nose - cpu.cd_nose).abs() < 2e-4, "base = {base}: wall Cp differs from the CPU");
        if base {
            assert!((g.cp_base - cpu.cp_base).abs() < 2e-4, "base pressure differs from the CPU");
        }
    }
}

/// GPU base-pressure sample (kernel + host weights) vs `base_state` of the downloaded state, both precisions, at M = 0.1 (low dynamic
/// pressure: the most sensitive case) and 0.5.
#[test]
fn gpu_base_cp_matches_base_state() {
    for mach in [0.1, 0.5] {
        let mut case = subsonic_case(mach, Nose::TangentOgive { fineness: 3.0 }, 0, true);
        case.numerics.first_order_iters = 5;
        for prec in [Precision::F32, Precision::Extended] {
            let mut gs = GpuRunner::new(&case, prec).unwrap();
            gs.gpu.run(100).unwrap();
            let rb2 = case.body.body.radius().powi(2);
            let sampler = BaseSampler::new(&gs.host, rb2);
            let cp_gpu = sampler.cp_base(&gs.gpu.base_cells().unwrap());
            gs.download().unwrap();
            let cp_cpu = base_state(&gs.host, rb2).2;
            println!("base Cp sample M = {mach} {prec:?}: gpu {cp_gpu:.10} host {cp_cpu:.10} diff {:.2e}", (cp_gpu - cp_cpu).abs());
            assert!((cp_gpu - cp_cpu).abs() < 1e-9, "M = {mach} {prec:?}: {cp_gpu} vs {cp_cpu}");
        }
    }
}

/// Converged-state agreement of the line-implicit scheme: the sting case (no base) at M = 0.3 and 0.6 (grid level 0, van Albada, low-Mach
/// correction) is iterated by the CPU and by the GPU (extended precision) to 5 orders of density-residual drop with the same stopping rule;
/// iteration counts, wall Cp, forebody CD and the field agree, and the fixed point is that of the explicit scheme (steady-state independence of the
/// acceleration: the explicit GPU solver evaluated on the converged implicit state has the converged residual level, not the start-up level).
#[test]
fn implicit_scheme_converged_state_matches_cpu() {
    for mach in [0.3, 0.6] {
        let mut case = subsonic_case(mach, Nose::TangentOgive { fineness: 3.0 }, 0, false);
        case.numerics.drop_orders = 5.0;
        case.numerics.max_iter = 4000;
        let mut s = case.build_solver();
        let cpu = run_on(&case, &mut s, false, None);
        let mut gs = GpuRunner::new(&case, Precision::Extended).unwrap();
        let gpu = gs.run_on(&case, false, None).unwrap();
        let g = &gpu.result;
        let dcp = g.wall.iter().zip(&cpu.wall).fold(0.0f64, |m, (a, b)| m.max((a.cp - b.cp).abs()));
        let diff = bowshock_gpu_cases::run::field_diff(&gs.gpu.state_f64().unwrap(), &s.u, s.cfg.freestream[3]);
        println!(
            "M = {mach}: CPU {} it ({}, {:.1} orders), GPU {} it ({}, {:.1} orders), max |dCp| {dcp:.2e}, CD_nose gpu {:+.6} cpu {:+.6}, field: max dp/p_inf {:.2e}, rms {:.2e}",
            cpu.iters, cpu.stop_reason, cpu.drop_orders, gpu.iters_stop, g.stop_reason, g.drop_orders, g.cd_nose, cpu.cd_nose, diff.max_dp_over_pinf, diff.rms_dp_over_pinf
        );
        // steady-state independence: one explicit SSP-RK3 iteration on the implicit solution (same operator, no first-order phase)
        let mut ex = GpuSolver::new(&luma::default_context().unwrap(), &s.grid, &s.cfg, Precision::Extended).unwrap();
        ex.set_low_mach(case.numerics.low_mach);
        ex.set_state(&gs.gpu.state_f64().unwrap()).unwrap();
        ex.set_iter(case.numerics.first_order_iters);
        let r_exp = ex.run(1).unwrap()[0];
        println!("   explicit residual of the implicit solution {r_exp:.3e} (implicit final {:.3e}, peak {:.3e})", g.residual.last().unwrap(), g.residual_peak);
        assert!(r_exp < 3.0 * g.residual.last().unwrap() + 1e-3 * g.residual_peak, "explicit residual {r_exp:e} of the implicit solution");
        assert_eq!(cpu.stop_reason, StopReason::ResidualDrop);
        assert_eq!(g.stop_reason, StopReason::ResidualDrop, "GPU did not reach {} orders", case.numerics.drop_orders);
        assert!((gpu.iters_stop as f64 - cpu.iters as f64).abs() <= 0.05 * cpu.iters as f64 + 5.0, "iteration counts {} vs {}", gpu.iters_stop, cpu.iters);
        assert!(dcp < 1e-4 && (g.cd_nose - cpu.cd_nose).abs() < 1e-4 && diff.max_dp_over_pinf < 1e-4, "converged fields differ");
    }
}

/// Warm-started implicit transonic sweep step on the GPU (`transonic` case, `Scheme::Implicit`, van Albada, 150 iterations at M = 0.8, then
/// M = 0.81 warm): residual history and forebody CD vs CPU; the CFL ramp restarts at the warm start on both sides. (Beyond ~190 iterations the
/// nonlinear iteration of the two solvers, which differ by round-off and by the f32 left-hand side, drifts apart at the 1e-2 level in a
/// residual that has dropped 4 orders: the converged states, not the transients, are compared in `implicit_scheme_converged_state_matches_cpu`.)
#[test]
fn implicit_transonic_warm_start_matches_cpu() {
    let mut case = transonic_level(0.8);
    case.numerics.limiter = Limiter::VanAlbada;
    case.numerics.scheme = Scheme::Implicit(bowshock_core::implicit::ImplicitParams::DEFAULT);
    case.numerics.first_order_iters = 100;
    case.numerics.max_iter = 150;
    let mut s = case.build_solver();
    let cpu0 = run_on(&case, &mut s, false, None);
    let mut gt = GpuRunner::new(&case, Precision::Extended).unwrap();
    let gpu0 = gt.run_on(&case, false, None).unwrap();
    case.mach = 0.81;
    case.numerics.max_iter = 40; // the warm start from the unconverged field is a violent transient: compare its first iterations only
    let cpu1 = run_on(&case, &mut s, true, Some(cpu0.residual_peak));
    let gpu1 = gt.run_on(&case, true, Some(gpu0.result.residual_peak)).unwrap();
    for (tag, cpu, gpu, tol_res) in [("M = 0.8 cold", &cpu0, &gpu0, 1e-3), ("M = 0.81 warm", &cpu1, &gpu1, 5e-2)] {
        let g = &gpu.result;
        assert_eq!((g.iters, gpu.iters_exec), (cpu.iters, cpu.iters), "{tag}: iteration counts");
        let res = g.residual.iter().zip(&cpu.residual).fold(0.0f64, |m, (a, b)| m.max((a - b).abs() / b));
        let dcp = g.wall.iter().zip(&cpu.wall).fold(0.0f64, |m, (a, b)| m.max((a.cp - b.cp).abs()));
        println!(
            "{tag}: CD gpu {:.6} cpu {:.6}, max |dCp| {dcp:.2e}, max rel residual diff {res:.2e}, residual {:.2e} -> {:.2e}",
            g.cd_nose,
            cpu.cd_nose,
            cpu.residual[0],
            cpu.residual.last().unwrap()
        );
        assert!(res < tol_res && dcp < 1e-5 && (g.cd_nose - cpu.cd_nose).abs() < 1e-5, "{tag}: GPU differs from CPU");
    }
}

/// A cold start on a reused GPU solver (Mach sweeps) is a fresh start: state, iteration counter and the implicit CFL ramp are reset, so a
/// second `run_on` at the same Mach number reproduces the residual history of the first (the ramp was once left at its maximum).
#[test]
fn cold_restart_on_a_reused_solver_reproduces_the_run() {
    let mut case = subsonic_case(0.4, Nose::TangentOgive { fineness: 3.0 }, 0, false);
    case.numerics.max_iter = 200;
    let mut gs = GpuRunner::new(&case, Precision::Extended).unwrap();
    let first = gs.run_on(&case, false, None).unwrap();
    case.mach = 0.7; // a different run in between
    gs.run_on(&case, false, None).unwrap();
    case.mach = 0.4;
    let again = gs.run_on(&case, false, None).unwrap();
    let d = first.result.residual.iter().zip(&again.result.residual).fold(0.0f64, |m, (a, b)| m.max((a - b).abs() / a));
    println!("cold restart: max relative residual difference {d:.2e}");
    assert!(d < 1e-6, "restart differs from the first run: {d:e}");
}

/// A sweep point on a reused `GpuSolver`: `set_config` with the free stream of another Mach number and `set_state` reproduce a CPU run of the new
/// configuration from a perturbed state (the fixed ghost states of the far field are uploaded again, the free stream constants of the step are
/// re-traced); `reset` then returns to the new free stream.
#[test]
fn set_config_moves_the_free_stream() {
    let ctx = luma::default_context().unwrap();
    let mut first = far_field_box(0.9, Bc::Characteristic);
    first.cfg.bc = [Bc::Freestream, Bc::PressureOutflow, Bc::SlipWall, Bc::Characteristic];
    let mut second = far_field_box(1.3, Bc::Characteristic);
    second.cfg.bc = first.cfg.bc;
    second.cfg.limiter = Limiter::VanAlbada;
    let u0 = perturbed_state(&second);
    second.u = u0.clone();
    let res_cpu: Vec<f64> = (0..20).map(|_| second.step()).collect();
    for (prec, tol) in [(Precision::F32, 1e-5), (Precision::Extended, 3e-7)] {
        let mut gpu = GpuSolver::new(&ctx, &first.grid, &first.cfg, prec).unwrap();
        gpu.run(10).unwrap(); // a different run in between
        gpu.set_config(&second.cfg).unwrap();
        assert_eq!(gpu.config().freestream, second.cfg.freestream);
        gpu.set_state(&u0).unwrap();
        assert_eq!(gpu.iter(), 0);
        let res = gpu.run(20).unwrap();
        let (worst, worst_res) = (state_diff(&gpu.state_f64().unwrap(), &second.u), residual_diff(&res, &res_cpu));
        println!("set_config {prec:?}: state {worst:.2e}, residual {worst_res:.2e}");
        assert!(worst < tol && worst_res < 1e-4, "{prec:?}: state {worst:e}, residual {worst_res:e}");
        gpu.reset().unwrap();
        let fs = bowshock_core::gas::prim_to_cons(&second.cfg.freestream);
        let d = state_diff(&gpu.state_f64().unwrap(), &vec![fs; gpu.ni * gpu.nj]);
        assert!(d < 1e-6 && gpu.iter() == 0, "reset: state differs from the free stream by {d:e}");
    }
}
