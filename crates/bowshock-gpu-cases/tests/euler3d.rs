//! Accuracy of the explicit scheme of the 3D array-op solver (`bowshock_cfd::gpu::{f32s, df32s}::LazySolver3`, every `Staging`) against the f64 CPU
//! `Solver3`, and of the `GpuSolver3` handle: the CPU and the GPU solver start from the same state, run the same iterations, and the conserved state
//! and the density residual history are compared,
//!
//! ```text
//! state:    max over cells and components of |dU| / (1 + |U_cpu|)
//! residual: max over iterations of |r_gpu - r_cpu| / r_cpu
//! ```
//!
//! The set-up is the tiny ANF-like case of the 3D plan (`body_grid3`: cone-cylinder with four square fins of chord 1, span 1, thickness 0.08 on the
//! sting grid with `n_up 10, n_nose 16, n_seg [12], n_ext 10, nj 24`, `nk` 8, flush fins), M = 0.5 (van Albada, low-Mach correction) and M = 2
//! (minmod) at 2 degrees, 30 iterations (the first 5 first order) from the free stream and from a CPU state advanced 150 iterations.
//! Targets: f32 state < 1e-5, double-float state < 3e-7 (the 2D `explicit.rs` tolerances). The numbers are printed
//! (`cargo test --release -- --nocapture`) whether or not they meet the targets.

use bowshock_cfd::flux::Limiter;
use bowshock_cfd::gas3::{freestream3, prim_to_cons3, Cons3};
use bowshock_cfd::gpu::{GpuSolver3, Precision, Staging};
use bowshock_cfd::grid::Grid;
use bowshock_cfd::grid3::{revolve, Grid3};
use bowshock_cfd::implicit::{ImplicitParams, LineSolve3};
use bowshock_cfd::solver3::{Config3, Scheme3, Solver3};
use bowshock_core::engine::body_grid::{BaseBody, Body, LargeGridParams, Nose};
use bowshock_core::engine::grid3::{body_grid3, Fins3, Grid3Params};
use bowshock_core::FinProfile;

/// The tiny ANF-like grid of the plan: 10 degree cone nose, tube to 10 diameters, four square fins (chord 1, span 1, thickness 0.08) flush with
/// the base, sting-grid counts `n_up 10, n_nose 16, n_seg [12], n_ext 10, nj 24`, `nk` 8 (level 0 spacings of `Case::new`).
fn tiny_anf_grid() -> Grid3 {
    let nose = Body { nose: Nose::Cone { half_angle: 10f64.to_radians() } };
    let l_nose = nose.nose_length();
    let body = BaseBody::cylinder(nose, 10.0 - l_nose);
    let l = body.x_base();
    let sting = LargeGridParams {
        body: body.clone(),
        x_up: 10.0 * l,
        x_down: 5.0 * l,
        r_far: 10.0 * l,
        n_up: 10,
        n_nose: 16,
        n_seg: vec![12],
        n_ext: 10,
        nj: 24,
        dx_tip: 0.03,
        dr_wall: 0.03,
    };
    let fins = Fins3 { count: 4, root_chord: 1.0, tip_chord: 1.0, span: 1.0, sweep: 0.0, thickness: 0.08, te_to_base: 0.0, profile: FinProfile::Square };
    body_grid3(&Grid3Params::half(sting, Some(fins), 8, 0)).expect("the tiny ANF grid builds")
}

fn config(mach: f64, alpha_deg: f64, limiter: Limiter, low_mach: bool) -> Config3 {
    Config3 { freestream: freestream3(mach, alpha_deg.to_radians()), cfl: 0.5, limiter, first_order_iters: 5, low_mach, scheme: Scheme3::Explicit }
}

/// Context of its own per call (isolation); `LAZY_SHARED_CONTEXT=1` uses the shared default context instead.
fn context() -> luma::Context {
    if std::env::var_os("LAZY_SHARED_CONTEXT").is_some() {
        luma::default_context().unwrap()
    } else {
        luma::Context::new().unwrap()
    }
}

struct Diff {
    state: f64,
    residual: f64,
}

fn diff(u_gpu: &[Cons3], res_gpu: &[f64], u_cpu: &[Cons3], res_cpu: &[f64]) -> Diff {
    let state = u_gpu.iter().zip(u_cpu).fold(0.0f64, |m, (a, b)| (0..5).fold(m, |m, k| m.max((a[k] - b[k]).abs() / (1.0 + b[k].abs()))));
    let residual = res_gpu.iter().zip(res_cpu).fold(0.0f64, |m, (a, b)| m.max((a - b).abs() / b));
    Diff { state, residual }
}

/// State targets `(f32, df32)`: the explicit tolerances of the 2D `explicit.rs`, and the implicit ones of the 2D `implicit.rs` (the f32 left-hand side
/// steers the iteration, so the GPU and the CPU iterates differ more).
type Targets = (f64, f64);
const EXPLICIT: Targets = (1e-5, 3e-7);
const IMPLICIT: Targets = (1e-3, 1e-4);

type Row = (String, &'static str, Diff, f64);

/// Runs the lazy solver of one precision from `u0` (iteration count `it0`) for `n` iterations.
macro_rules! run_lazy {
    ($module:ident, $ctx:expr, $staging:expr, $grid:expr, $cfg:expr, $u0:expr, $it0:expr, $n:expr) => {{
        let mut lazy = bowshock_cfd::gpu::$module::LazySolver3::new($ctx, $grid, $cfg).unwrap();
        lazy.staging = $staging;
        lazy.set_state($u0).unwrap();
        lazy.iter = $it0;
        let res = lazy.run($n, 16).unwrap();
        (lazy.state_f64().unwrap(), res)
    }};
}

/// One set-up: the CPU advances `warm` iterations from the free stream, then both solvers run `n` more from that state; both precisions and the given
/// stagings.
fn compare(label: &str, grid: &Grid3, cfg: &Config3, warm: usize, n: usize, stagings: &[Staging], targets: Targets, rows: &mut Vec<Row>) {
    let mut cpu = Solver3::new(grid.clone(), cfg.clone());
    for _ in 0..warm {
        cpu.step();
    }
    let (u0, it0) = (cpu.u.clone(), cpu.iter);
    let res_cpu: Vec<f64> = (0..n).map(|_| cpu.step()).collect();
    assert!(cpu.u.iter().all(|q| q.iter().all(|x| x.is_finite())), "the CPU run is finite");
    let ctx = context();
    for &staging in stagings {
        let (u, r) = run_lazy!(f32s, &ctx, staging, grid, cfg, &u0, it0, n);
        rows.push((format!("{label} [{staging:?}]"), "f32", diff(&u, &r, &cpu.u, &res_cpu), targets.0));
        let (u, r) = run_lazy!(df32s, &ctx, staging, grid, cfg, &u0, it0, n);
        rows.push((format!("{label} [{staging:?}]"), "df32", diff(&u, &r, &cpu.u, &res_cpu), targets.1));
    }
}

fn report(test: &str, rows: &[Row]) {
    println!("\n{test}");
    println!("| case | precision | max abs dU/(1+abs U) | max rel residual diff | target state |");
    println!("|---|---|---|---|---|");
    let mut failed = Vec::new();
    for (label, prec, d, ts) in rows {
        println!("| {label} | {prec} | {:.2e} | {:.2e} | {ts:.0e} |", d.state, d.residual);
        if d.state.is_nan() || d.state >= *ts {
            failed.push(format!("{label} {prec}: state {:e} (target {ts:e}), residual {:e}", d.state, d.residual));
        }
    }
    assert!(failed.is_empty(), "targets missed: {failed:#?}");
}

const ALL_STAGINGS: [Staging; 4] = [Staging::Fusion, Staging::Lines, Staging::Fluxes, Staging::Split];

/// The implicit parameters of `Numerics3::auto` (`cfl_max` 10), and a steeper ramp that reaches the CFL limit within the 30 iterations.
fn implicit_params(steep: bool) -> ImplicitParams {
    if steep {
        ImplicitParams { cfl0: 3.0, growth: 1.25, cfl_max: 10.0, ..ImplicitParams::DEFAULT }
    } else {
        ImplicitParams { cfl_max: 10.0, ..ImplicitParams::DEFAULT }
    }
}

/// Tiny ANF-like case with fins at M = 0.5 (van Albada, low-Mach correction) and M = 2 (minmod), 2 degrees, 30 iterations from the free stream
/// (the first 5 first order) and 30 iterations after 150 CPU iterations.
#[test]
fn tiny_anf_state_after_30_iterations_matches_cpu() {
    let grid = tiny_anf_grid();
    println!("tiny ANF grid: {} x {} x {} cells, {} solid boxes, axis {:?}", grid.ni, grid.nj, grid.nk, grid.solid.len(), grid.axis);
    let mut rows = Vec::new();
    for (mach, limiter, low_mach) in [(0.5, Limiter::VanAlbada, true), (2.0, Limiter::Minmod, false), (2.0, Limiter::Venkatakrishnan, false)] {
        let cfg = config(mach, 2.0, limiter, low_mach);
        compare(&format!("M {mach} cold"), &grid, &cfg, 0, 30, &ALL_STAGINGS, EXPLICIT, &mut rows);
        compare(&format!("M {mach} warm"), &grid, &cfg, 150, 30, &ALL_STAGINGS, EXPLICIT, &mut rows);
    }
    report("tiny ANF, explicit, 30 iterations", &rows);
}

/// The implicit scheme on the tiny ANF case, M = 0.5 (van Albada, low-Mach correction) and M = 2 (minmod), 2 degrees, 30 iterations with the first 5
/// first order from the free stream, the `Numerics3::auto` ramp and a steeper one (`implicit_params`), both precisions, `Lines` and `Fluxes` staging.
#[test]
fn tiny_anf_implicit_state_after_30_iterations_matches_cpu() {
    let grid = tiny_anf_grid();
    let mut rows = Vec::new();
    for (mach, limiter, low_mach) in [(0.5, Limiter::VanAlbada, true), (2.0, Limiter::Minmod, false), (2.0, Limiter::Venkatakrishnan, false)] {
        for steep in [false, true] {
            let cfg = Config3 { scheme: Scheme3::Implicit(implicit_params(steep)), ..config(mach, 2.0, limiter, low_mach) };
            let tag = if steep { "steep" } else { "auto" };
            compare(&format!("M {mach} implicit {tag} cold"), &grid, &cfg, 0, 30, &[Staging::Lines, Staging::Fluxes], IMPLICIT, &mut rows);
        }
    }
    report("tiny ANF, implicit, 30 iterations", &rows);
}

/// The diagonalised scheme with one defect-correction pass (`LineSolve3::DiagonalCorrected`, what `Numerics3::auto` selects) on the tiny ANF case against
/// the CPU solver with the same parameters, 20 iterations from the free stream (the first 5 first order), M = 0.5 and M = 2, the `Numerics3::auto` ramp
/// and the steep one, `Fluxes` staging, both precisions. The f32 target is 1e-2, not the 1e-3 of the block scheme: the correction pass amplifies the f32
/// rounding of the left-hand side on the steep ramp (CFL 3 -> 10 in 20 iterations; measured 6.0e-3 at M 0.5 and 1.1e-3 at M 2; the other f32 rows and
/// all df32 rows are within 1e-3 / 1e-4).
#[test]
fn tiny_anf_diag_state_after_20_iterations_matches_cpu() {
    const DIAG: Targets = (1e-2, 1e-4);
    let grid = tiny_anf_grid();
    let mut rows = Vec::new();
    for (mach, limiter, low_mach) in [(0.5, Limiter::VanAlbada, true), (2.0, Limiter::Minmod, false)] {
        for steep in [false, true] {
            let params = ImplicitParams { line_solve: LineSolve3::DiagonalCorrected, ..implicit_params(steep) };
            let cfg = Config3 { scheme: Scheme3::Implicit(params), ..config(mach, 2.0, limiter, low_mach) };
            let tag = if steep { "steep" } else { "auto" };
            compare(&format!("M {mach} diag {tag} cold"), &grid, &cfg, 0, 20, &[Staging::Fluxes], DIAG, &mut rows);
        }
    }
    report("tiny ANF, diagonalised implicit, 20 iterations", &rows);
}

/// The implicit scheme on every solid-face pattern of `solid_box_patterns_match_cpu` (lines cut by the solid cells in all three directions, the first
/// and last cell of a line next to a solid cell, a half layer against the symmetry plane), M = 2. Two runs on the three boxes, each box alone and
/// together:
///
/// * the steep ramp, 30 iterations, with the ghost Jacobians frozen (`implicit_bc = false`): the solid-wall folds and the identity rows are checked
///   without any ghost Jacobian (the characteristic far field is always frozen, `T = 0`, on both sides; only the wall and symmetry reflections
///   are switched by `implicit_bc`);
/// * the `Numerics3::auto` ramp for 3 iterations with the wall / symmetry ghost Jacobians on.
#[test]
fn solid_box_patterns_implicit_match_cpu() {
    let mut rows = Vec::new();
    for boxes in [&[0][..], &[1][..], &[2][..], &[0, 1, 2][..]] {
        let grid = solid_pattern_grid(boxes);
        let frozen = ImplicitParams { implicit_bc: false, ..implicit_params(true) };
        let cfg = Config3 { scheme: Scheme3::Implicit(frozen), ..config(2.0, 2.0, Limiter::Minmod, false) };
        compare(
            &format!("solid patterns implicit {boxes:?}, bc frozen, steep, n=30"),
            &grid,
            &cfg,
            0,
            30,
            &[Staging::Lines, Staging::Fluxes],
            IMPLICIT,
            &mut rows,
        );
    }
    let grid = solid_pattern_grid(&[0, 1, 2]);
    let cfg = Config3 { scheme: Scheme3::Implicit(implicit_params(false)), ..config(2.0, 2.0, Limiter::Minmod, false) };
    compare("solid patterns [0, 1, 2], bc on, auto, n=3", &grid, &cfg, 0, 3, &[Staging::Fluxes], IMPLICIT, &mut rows);
    report("solid box patterns, implicit", &rows);
}

/// A rectangular axisymmetric box (`x, r` in the unit square, interior nodes perturbed by 10 % of the spacing), revolved into `nk` non-uniform azimuthal
/// cells, `axis_cells = ni`: no body, the characteristic far field on every outer boundary.
fn far_field_box_grid(ni: usize, nj: usize, nk: usize) -> Grid3 {
    let mut state = 0x1234_5678u64;
    let mut next = move || {
        state = state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        ((state >> 11) as f64) / ((1u64 << 53) as f64)
    };
    let (mut x, mut r) = (vec![0.0; (ni + 1) * (nj + 1)], vec![0.0; (ni + 1) * (nj + 1)]);
    for j in 0..=nj {
        for i in 0..=ni {
            let n = j * (ni + 1) + i;
            let (mut xx, mut rr) = (i as f64 / ni as f64, j as f64 / nj as f64);
            if i > 0 && i < ni && j > 0 && j < nj {
                xx += 0.1 * (2.0 * next() - 1.0) / ni as f64;
                rr += 0.1 * (2.0 * next() - 1.0) / nj as f64;
            }
            x[n] = xx;
            r[n] = rr;
        }
    }
    let g2 = Grid::try_from_nodes(ni, nj, x, r).unwrap();
    let theta: Vec<f64> = (0..=nk)
        .map(|k| {
            let s = k as f64 / nk as f64;
            std::f64::consts::PI * (s - 0.06 * (2.0 * std::f64::consts::PI * s).sin())
        })
        .collect();
    let mut g3 = revolve(&g2, nk, &theta).unwrap();
    g3.axis = vec![bowshock_cfd::grid3::AxisRun { j: 0, i: 0..ni }];
    g3
}

/// Free-stream preservation on the GPU: M = 0.8 at 6 degrees on the far-field box (CPU: 1e-12 after one step), 5 explicit and 5 implicit iterations; the
/// GPU keeps the uniform flow to round-off of its precision.
#[test]
fn far_field_box_preserves_the_free_stream() {
    let grid = far_field_box_grid(8, 8, 8);
    let explicit = Config3 { first_order_iters: 0, ..config(0.8, 6.0, Limiter::VanAlbada, false) };
    let implicit = Config3 { scheme: Scheme3::Implicit(implicit_params(true)), ..explicit.clone() };
    let q0 = prim_to_cons3(&explicit.freestream);
    let ctx = context();
    let mut failed = Vec::new();
    for (scheme, cfg) in [("explicit", &explicit), ("implicit", &implicit)] {
        for staging in [Staging::Fusion, Staging::Lines, Staging::Fluxes, Staging::Split] {
            for (prec, target, run) in [("f32", 1e-6, 0), ("df32", 1e-11, 1)] {
                let u0 = vec![q0; grid.ni * grid.nj * grid.nk];
                let (u, _) =
                    if run == 0 { run_lazy!(f32s, &ctx, staging, &grid, cfg, &u0, 0, 5) } else { run_lazy!(df32s, &ctx, staging, &grid, cfg, &u0, 0, 5) };
                let worst = u.iter().fold(0.0f64, |m, q| (0..5).fold(m, |m, k| m.max((q[k] - q0[k]).abs() / (1.0 + q0[k].abs()))));
                println!("free stream {scheme} {prec} [{staging:?}]: max |dU|/(1+|U|) = {worst:.2e} (target {target:.0e})");
                if worst.is_nan() || worst >= target {
                    failed.push(format!("{scheme} {prec} [{staging:?}]: {worst:e} (target {target:e})"));
                }
            }
        }
    }
    assert!(failed.is_empty(), "free stream not preserved: {failed:#?}");
}

/// The `GpuSolver3` handle: `from_host` + `run` + `state_f64` + `reset` + `set_config` on the tiny ANF case, both precisions, the default staging,
/// then the implicit scheme; kernel launches and GPU time per iteration are printed.
#[test]
fn gpu_solver3_handle_and_timing() {
    let grid = tiny_anf_grid();
    let cfg = config(2.0, 2.0, Limiter::Minmod, false);
    let mut cpu = Solver3::new(grid.clone(), cfg.clone());
    for _ in 0..40 {
        cpu.step();
    }
    let ctx = context();
    for (prec, name) in [(Precision::F32, "f32"), (Precision::Extended, "df32")] {
        let mut gpu = GpuSolver3::from_host(&ctx, &cpu, prec).unwrap();
        assert_eq!(gpu.iter(), cpu.iter);
        let res = gpu.run(10).unwrap();
        assert_eq!(res.len(), 10);
        assert_eq!(gpu.iter(), cpu.iter + 10);
        let u = gpu.state_f64().unwrap();
        assert_eq!(u.len(), grid.ni * grid.nj * grid.nk);
        assert!(u.iter().all(|q| q.iter().all(|x| x.is_finite())));

        // timing: the `iterate_all` replay of 100 iterations
        let n = 100;
        let t0 = std::time::Instant::now();
        let p = gpu.profile_replay(n).unwrap();
        let wall = t0.elapsed().as_secs_f64();
        println!(
            "tiny ANF explicit {name}: {} cells, {:.1} kernel launches / iteration, {:.1} us GPU time / iteration (replay of {n}, wall incl. compile {wall:.2} s)",
            grid.ni * grid.nj * grid.nk,
            f64::from(p.launches()) / n as f64,
            p.total().as_secs_f64() * 1e6 / n as f64,
        );
        let t0 = std::time::Instant::now();
        gpu.run(n).unwrap();
        println!("tiny ANF explicit {name}: {:.1} us / iteration wall (run of {n} with the residual recorded)", t0.elapsed().as_secs_f64() * 1e6 / n as f64);

        // reset to the free stream of the configuration, then a new configuration
        gpu.reset().unwrap();
        assert_eq!(gpu.iter(), 0);
        let q0 = prim_to_cons3(&gpu.config().freestream);
        assert!(gpu.state_f64().unwrap().iter().all(|q| (0..5).all(|k| (q[k] - q0[k]).abs() < 1e-5 * (1.0 + q0[k].abs()))));
        gpu.set_config(&config(2.5, 4.0, Limiter::VanAlbada, false)).unwrap();
        assert!((gpu.config().freestream[1] - 2.5 * 4f64.to_radians().cos()).abs() < 1e-12);
        gpu.run(5).unwrap();

        // the implicit scheme: the ramp restarts with the scheme, the state stays finite
        let implicit = Config3 { scheme: Scheme3::Implicit(implicit_params(false)), ..cfg.clone() };
        gpu.set_config(&implicit).unwrap();
        gpu.reset().unwrap();
        gpu.run(10).unwrap();
        assert!(gpu.state_f64().unwrap().iter().all(|q| q.iter().all(|x| x.is_finite())));
        assert!(GpuSolver3::new(&ctx, &grid, &implicit, prec).is_ok());
    }
}

/// One SSP-RK3 step of the tiny ANF case as the `eval` loop launches it, kernel by kernel (labels cut to 90 characters): where the launches of a step go.
/// Informational; nothing is asserted beyond the step running.
#[test]
fn tiny_anf_step_profile() {
    let grid = tiny_anf_grid();
    let cfg = config(2.0, 2.0, Limiter::Minmod, false);
    let cpu = Solver3::new(grid, cfg);
    let ctx = context();
    for (prec, name) in [(Precision::F32, "f32"), (Precision::Extended, "df32")] {
        let mut gpu = GpuSolver3::from_host(&ctx, &cpu, prec).unwrap();
        gpu.run(10).unwrap();
        let p = gpu.profile_step().unwrap();
        println!(
            "\ntiny ANF {name}: one step as the eval loop launches it: {} launches, {:.1} us GPU time, {} fusion cuts",
            p.launches(),
            p.total().as_secs_f64() * 1e6,
            p.splits.len()
        );
        println!("{:<92} {:>8} {:>10} {:>10}", "kernel", "launches", "total us", "mean us");
        for k in &p.kernels {
            let label: String = k.label.chars().take(90).collect();
            println!("{label:<92} {:>8} {:>10.1} {:>10.1}", k.launches, k.total.as_secs_f64() * 1e6, k.mean().as_secs_f64() * 1e6);
        }
        assert!(p.launches() > 0);
    }
}

/// The tiny grid with a revolved far-field box and three solid boxes (every solid-face pattern of `Family::fluxes`): a one-cell-wide box (`i` 4..5, so
/// the face before and the face after it see the solid cell as `m1`/`p0` and as the outer stencil cell on the same face line), two one-cell layers in
/// `k` with one fluid cell between them (the faces next to the gap see a solid cell on both sides of the stencil), a wider box, and a half layer at
/// `k = nk - 1` against the symmetry plane.
fn solid_pattern_grid(boxes: &[usize]) -> Grid3 {
    let (ni, nj, nk) = (12, 10, 8);
    let mut grid = far_field_box_grid(ni, nj, nk);
    // no axis run: the whole `j = 0` plane of the revolved box (zero-area faces on the axis) is the slip-wall branch; the solid boxes stay off it
    grid.axis.clear();
    let all = [
        bowshock_cfd::grid3::SolidBox { i: 4..5, j: 2..6, k: 2..3 },
        bowshock_cfd::grid3::SolidBox { i: 4..7, j: 2..6, k: 4..5 },
        bowshock_cfd::grid3::SolidBox { i: 8..10, j: 3..7, k: nk - 1..nk },
    ];
    for &b in boxes {
        grid.mark_solid(all[b].clone(), Some(bowshock_cfd::grid3::Part::Fin(0)));
    }
    grid
}

/// Every solid-face pattern of `Family::fluxes` on a revolved box (no body, free-stream start at M = 2, 2 degrees): 30 iterations (the first 5 first
/// order) against the CPU, both precisions, `Fusion` (lazy region writes) and `Fluxes` (cached fluxes).
#[test]
fn solid_box_patterns_match_cpu() {
    let grid = solid_pattern_grid(&[0, 1, 2]);
    let cfg = config(2.0, 2.0, Limiter::Minmod, false);
    let mut rows = Vec::new();
    compare("solid patterns", &grid, &cfg, 0, 30, &[Staging::Fusion, Staging::Fluxes], EXPLICIT, &mut rows);
    report("solid box patterns, explicit, 30 iterations", &rows);
}
