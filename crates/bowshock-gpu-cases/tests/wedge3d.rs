//! The 3D array-op solver on a one-cell wedge and on thin sectors (`nk` = 1, 2, 4, 12 cells over a 6 degree opening angle) against the f64 CPU `Solver3`:
//! the pattern of `euler3d.rs` (same state after 30 iterations from the free stream, `max |dU| / (1 + |U|)`) on the base-wake grid of the G3 body
//! revolved by `bowshock_gpu_cases::wedge3` (body block solid over every `k`, `axis_cells = 0`, the k planes the reflecting planes of today's solver).
//! Whether the physics of that wedge is right does not matter here; what is checked is that the GPU code and Luma handle a singleton or very short `k`
//! axis: the block-tridiagonal and scalar tridiagonal parts solves along `k` of length 1, 2 and 4 (and the diagonalised scheme), the pad / slice
//! / region assigns of the ghost layers at `k = 0` and `k = nk`, the solid-face masks, the reductions.
//!
//! Every case runs in its own GPU context; a panic or an error of one case is caught and reported in the matrix, the others still run.
//! `cargo test --release -p bowshock-gpu-cases --test wedge3d -- --nocapture`

use bowshock_cfd::flux::Limiter;
use bowshock_cfd::gas3::{freestream3, Cons3};
use bowshock_cfd::gpu::{GpuSolver3, Precision, Staging};
use bowshock_cfd::grid3::Grid3;
use bowshock_cfd::implicit::{ImplicitParams, LineSolve3};
use bowshock_cfd::solver3::{Config3, Scheme3, Solver3};
use bowshock_core::{AftSpec, BodySpec, NoseShape, NoseSpec, Resolution};
use bowshock_gpu_cases::wedge3::{base_grid_params, small, wedge_grid};
use std::panic::{catch_unwind, AssertUnwindSafe};

const NK: [usize; 4] = [1, 2, 4, 12];

/// The `nk` of the matrix; `WEDGE3D_NK=1,2` runs a subset.
fn nk_list() -> Vec<usize> {
    match std::env::var("WEDGE3D_NK") {
        Ok(v) => v.split(',').map(|x| x.trim().parse().expect("WEDGE3D_NK is a comma list of integers")).collect(),
        Err(_) => NK.to_vec(),
    }
}
const OPENING_DEG: f64 = 6.0;

fn g3_body() -> BodySpec {
    BodySpec { nose: NoseSpec { shape: NoseShape::TangentOgive, length: 0.30, diameter: 0.10 }, aft: vec![AftSpec::Tube { length: 0.60 }] }
}

fn grid(nk: usize) -> Grid3 {
    wedge_grid(&small(&base_grid_params(&g3_body(), 0.5, false, Resolution::Coarse)), nk, OPENING_DEG)
}

fn config(scheme: Scheme3) -> Config3 {
    Config3 { freestream: freestream3(0.5, 0.0), cfl: 0.5, limiter: Limiter::VanAlbada, first_order_iters: 5, low_mach: true, scheme }
}

/// One row of the matrix: a scheme, the stagings it runs with, the number of iterations and the state targets `(f32, df32)`.
struct Row {
    label: &'static str,
    scheme: Scheme3,
    stagings: &'static [Staging],
    n: usize,
    targets: (f64, f64),
}

fn rows() -> Vec<Row> {
    let steep = ImplicitParams { cfl0: 3.0, growth: 1.25, cfl_max: 10.0, ..ImplicitParams::DEFAULT };
    vec![
        Row { label: "explicit", scheme: Scheme3::Explicit, stagings: &[Staging::Fusion, Staging::Fluxes], n: 30, targets: (1e-5, 3e-7) },
        Row { label: "implicit block", scheme: Scheme3::Implicit(steep), stagings: &[Staging::Fluxes], n: 30, targets: (1e-3, 1e-4) },
        Row {
            label: "implicit diag",
            scheme: Scheme3::Implicit(ImplicitParams { line_solve: LineSolve3::DiagonalCorrected, ..steep }),
            stagings: &[Staging::Fluxes],
            n: 20,
            targets: (1e-2, 1e-4),
        },
    ]
}

fn panic_text(e: Box<dyn std::any::Any + Send>) -> String {
    let s = e.downcast_ref::<String>().cloned().or_else(|| e.downcast_ref::<&str>().map(|s| s.to_string())).unwrap_or_else(|| "panic".into());
    s.lines().next().unwrap_or("").chars().take(160).collect()
}

fn state_diff(a: &[Cons3], b: &[Cons3]) -> f64 {
    a.iter().zip(b).fold(0.0f64, |m, (x, y)| (0..5).fold(m, |m, k| m.max((x[k] - y[k]).abs() / (1.0 + y[k].abs()))))
}

/// The lazy solver of one precision for `n` iterations from `u0`.
macro_rules! run_lazy {
    ($module:ident, $ctx:expr, $staging:expr, $grid:expr, $cfg:expr, $u0:expr, $n:expr) => {{
        let mut lazy = bowshock_cfd::gpu::$module::LazySolver3::new($ctx, $grid, $cfg).map_err(|e| format!("new: {e}"))?;
        lazy.staging = $staging;
        lazy.set_state($u0).map_err(|e| format!("set_state: {e}"))?;
        lazy.run($n, 16).map_err(|e| format!("run: {e}"))?;
        lazy.state_f64().map_err(|e| format!("state: {e}"))?
    }};
}

enum Cell {
    Pass(f64),
    Fail(f64),
    Error(String),
}

fn run_gpu(f32_precision: bool, staging: Staging, grid: &Grid3, cfg: &Config3, u0: &[Cons3], n: usize) -> Result<Vec<Cons3>, String> {
    catch_unwind(AssertUnwindSafe(|| -> Result<Vec<Cons3>, String> {
        let ctx = luma::Context::new().map_err(|e| format!("context: {e}"))?;
        Ok(if f32_precision { run_lazy!(f32s, &ctx, staging, grid, cfg, u0, n) } else { run_lazy!(df32s, &ctx, staging, grid, cfg, u0, n) })
    }))
    .unwrap_or_else(|e| Err(format!("panic: {}", panic_text(e))))
}

#[test]
fn wedge_and_thin_sectors_match_cpu() {
    let mut table: Vec<(String, usize, &'static str, Cell)> = Vec::new();
    for nk in nk_list() {
        let g = grid(nk);
        println!("nk {nk}: grid {} x {} x {} = {} cells, {} solid box, opening {OPENING_DEG} deg", g.ni, g.nj, g.nk, g.ni * g.nj * g.nk, g.solid.len());
        for row in rows() {
            let cfg = config(row.scheme);
            let cpu = catch_unwind(AssertUnwindSafe(|| {
                let mut s = Solver3::new(g.clone(), cfg.clone());
                let u0 = s.u.clone();
                for _ in 0..row.n {
                    s.step();
                }
                (u0, s.u)
            }));
            let (u0, u_cpu) = match cpu {
                Ok(x) if x.1.iter().all(|q| q.iter().all(|v| v.is_finite())) => x,
                Ok(_) => {
                    for prec in ["f32", "df32"] {
                        table.push((row.label.into(), nk, prec, Cell::Error("CPU solver went non-finite".into())));
                    }
                    continue;
                }
                Err(e) => {
                    let text = format!("CPU solver panic: {}", panic_text(e));
                    for prec in ["f32", "df32"] {
                        table.push((row.label.into(), nk, prec, Cell::Error(text.clone())));
                    }
                    break;
                }
            };
            for &staging in row.stagings {
                for (f32_precision, prec, target) in [(true, "f32", row.targets.0), (false, "df32", row.targets.1)] {
                    let label = if row.stagings.len() > 1 { format!("{} [{staging:?}]", row.label) } else { row.label.to_string() };
                    let cell = match run_gpu(f32_precision, staging, &g, &cfg, &u0, row.n) {
                        Ok(u) => {
                            let d = state_diff(&u, &u_cpu);
                            if d.is_nan() || d >= target {
                                Cell::Fail(d)
                            } else {
                                Cell::Pass(d)
                            }
                        }
                        Err(e) => Cell::Error(e),
                    };
                    table.push((label, nk, prec, cell));
                }
            }
        }
    }
    println!("\n| case | precision | nk | result |\n|---|---|---|---|");
    let mut bad = Vec::new();
    for (label, nk, prec, cell) in &table {
        let text = match cell {
            Cell::Pass(d) => format!("pass {d:.2e}"),
            Cell::Fail(d) => format!("FAIL {d:.2e}"),
            Cell::Error(e) => format!("ERROR {e}"),
        };
        println!("| {label} | {prec} | {nk} | {text} |");
        if !matches!(cell, Cell::Pass(_)) {
            bad.push(format!("{label} {prec} nk {nk}: {text}"));
        }
    }
    assert!(bad.is_empty(), "{} cells of the matrix are not green:\n{}", bad.len(), bad.join("\n"));
}

/// Wedge mode (`GpuSolver3::set_wedge_mode`: no k factor, no `L_k` in the time step, the pressure of the cell as the flux through the two k planes) on the
/// one-cell wedge, M 0.5 Euler, the small base grid, df32, 400 iterations. Measured (`DESIGN3D` section A2 proposes dropping the k factor; scan of
/// 1500 iterations, `cfl_max` 0.5, 1, 2, 5, 10): with the pressure flux through the planes the iteration without the k factor converges at `cfl_max` 1 and
/// below, rises at 2, is O(1) at 5 (`Block`) / NaN (`DiagonalCorrected`) and NaN at 10; with the Riemann flux it was NaN at 10 (only that was run), with the
/// k factor kept the pressure flux is stable at 10 (`Block`). So the wedge runs at `cfl_max` 1. Three runs per line solve:
///
/// * the reference, the product's operator: k factor, `L_k` in the time step, Riemann flux through the planes, `cfl_max` 10 (`Block`) / 5 (`DiagonalCorrected`,
///   which diverges at 10 on this grid: ISSUES #20);
/// * wedge mode at `cfl_max` 1;
/// * the same time step and residual as wedge mode with the k factor kept in the left-hand side (`set_wedge_parts(false, true, true)`): the two differ only
///   by the k factor, so they follow the same trajectory (`Block` only: the diagonal scheme with the k factor but without `L_k` in the time step diverges).
///
/// Asserts finite states and a residual below 1e-3 at iteration 400 for wedge mode (both line solves), for the reference and the twin (`Block`), and the
/// states of wedge mode and its k-factor twin within 2e-2 of each other (`Block`, measured 8e-3; the Euler wake of M 0.5 is unsteady, so runs with
/// different time steps are not compared state against state: wedge mode against the reference differs by 0.15).
#[test]
fn wedge_mode_converges_like_the_k_factor() {
    let g = grid(1);
    let ctx = luma::Context::new().unwrap();
    for (label, line_solve, cfl_ref) in [("block", LineSolve3::Block, 10.0), ("diag", LineSolve3::DiagonalCorrected, 5.0)] {
        let run = |parts: (bool, bool, bool), cfl_max: f64| {
            let params = ImplicitParams { cfl_max, line_solve, ..ImplicitParams::DEFAULT };
            let cfg = Config3 { first_order_iters: 50, ..config(Scheme3::Implicit(params)) };
            let mut s = GpuSolver3::new(&ctx, &g, &cfg, Precision::Extended).unwrap();
            s.set_wedge_parts(parts.0, parts.1, parts.2);
            assert_eq!(s.wedge_mode(), parts == (true, true, true));
            let res = s.run(400).unwrap();
            (res, s.state_f64().unwrap())
        };
        let finite = |u: &[Cons3]| u.iter().all(|q| q.iter().all(|v| v.is_finite()));
        let (r_ref, u_ref) = run((false, false, false), cfl_ref);
        let (r_wedge, u_wedge) = run((true, true, true), 1.0);
        let (r_twin, u_twin) = run((false, true, true), 1.0);
        println!(
            "wedge mode {label}: residual at 400: reference (cfl_max {cfl_ref}) {:.2e}, wedge mode {:.2e}, wedge mode with the k factor {:.2e}; max |dU|/(1+|U|): wedge mode - its k-factor twin {:.2e}, wedge mode - reference {:.2e}",
            r_ref[399],
            r_wedge[399],
            r_twin[399],
            state_diff(&u_wedge, &u_twin),
            state_diff(&u_wedge, &u_ref)
        );
        assert!(finite(&u_wedge) && r_wedge[399] < 1e-3, "{label}: wedge mode did not settle");
        if label == "block" {
            assert!(finite(&u_ref) && finite(&u_twin) && r_ref[399] < 1e-3 && r_twin[399] < 1e-3, "{label}: the references did not settle");
            let d = state_diff(&u_wedge, &u_twin);
            assert!(d < 2e-2, "{label}: wedge mode and its k-factor twin differ by {d:e}");
        }
    }
    assert!(GpuSolver3::new(&ctx, &grid(2), &config(Scheme3::Explicit), Precision::F32).unwrap().set_wedge_mode(true).is_err(), "wedge mode needs nk == 1");
}
