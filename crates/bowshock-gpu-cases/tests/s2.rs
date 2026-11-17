//! Stage S2: the 3D array-op solver on the S1 topologies (`Case3::{wedge, base_half, sector}`: the revolved base-wake grid with dead cells, two axis runs
//! and the generalised axis ghost; the single-cell wedge with its pressure flux and source block; fin sectors of even and odd `N`) against the f64 CPU
//! `Solver3` on the same grid: same state, same iterations from the free stream, `max |dU| / (1 + |U|)` and the density residual history (the pattern and
//! the targets of `euler3d.rs`). Tiny grids; each row is its own GPU context. Every precision, the explicit scheme (`Fusion`, `Fluxes`), the implicit block
//! scheme and the diagonalised one.
//! `cargo test --release -p bowshock-gpu-cases --test s2 -- --nocapture`

use bowshock_cfd::gas3::Cons3;
use bowshock_cfd::gpu::Staging;
use bowshock_cfd::grid3::Grid3;
use bowshock_cfd::implicit::{ImplicitParams, LineSolve3};
use bowshock_cfd::solver3::{Config3, Scheme3, Solver3};
use bowshock_core::engine::body_grid::{BaseBody, Body, Nose};
use bowshock_core::engine::case3::Case3;
use bowshock_core::engine::grid3::{body_grid3, Fins3};
use bowshock_core::FinProfile;
use std::panic::{catch_unwind, AssertUnwindSafe};

/// Tangent ogive (fineness 3) and a 6 d cylinder: the body of `bowshock-cases`.
fn body() -> BaseBody {
    BaseBody::cylinder(Body { nose: Nose::TangentOgive { fineness: 3.0 } }, 6.0)
}

fn tiny_base(c: &mut Case3) {
    let p = c.grid.base.as_mut().expect("base-wake grid");
    p.n_up = 6;
    p.n_nose = 10;
    p.n_seg = p.n_seg.iter().map(|&n| (n / 4).max(4)).collect();
    p.n_wake = 8;
    p.n_gap = 6;
    p.nj_out = 10;
}

/// The one-cell wedge of the 2 degree default on the tiny base-wake grid.
fn wedge(mach: f64) -> Case3 {
    let mut c = Case3::wedge(mach, body(), 0);
    tiny_base(&mut c);
    c
}

/// The half domain (pitch-plane symmetry) on the tiny base-wake grid, 2 degrees incidence.
fn half(mach: f64) -> Case3 {
    let mut c = Case3::base_half(mach, 2f64.to_radians(), body(), 0);
    tiny_base(&mut c);
    c.grid.nk = 8;
    c
}

/// `n` fins of chord 1, span 1, thickness 0.08 on the cone-cylinder of the tiny ANF case, as the sector `[0, pi/n]`.
fn sector(mach: f64, n: u32, nk: usize) -> Case3 {
    let nose = Body { nose: Nose::Cone { half_angle: 10f64.to_radians() } };
    let body = BaseBody::cylinder(nose, 10.0 - nose.nose_length());
    let fins = Fins3 { count: n, root_chord: 1.0, tip_chord: 1.0, span: 1.0, sweep: 0.0, thickness: 0.08, te_to_base: 0.0, profile: FinProfile::Square };
    let mut c = Case3::sector(mach, body, fins, nk, 0);
    let s = &mut c.grid.sting;
    s.n_up = 10;
    s.n_nose = 16;
    s.n_seg = vec![12];
    s.n_ext = 10;
    s.nj = 24;
    c.grid.fin_chord_cells = Some(16);
    c
}

fn config(c: &Case3, scheme: Scheme3) -> Config3 {
    let n = &c.numerics;
    Config3 { freestream: c.freestream(), cfl: 0.5, limiter: n.limiter, first_order_iters: 5, low_mach: n.low_mach, scheme }
}

fn panic_text(e: Box<dyn std::any::Any + Send>) -> String {
    let s = e.downcast_ref::<String>().cloned().or_else(|| e.downcast_ref::<&str>().map(|s| s.to_string())).unwrap_or_else(|| "panic".into());
    s.lines().next().unwrap_or("").chars().take(200).collect()
}

fn state_diff(a: &[Cons3], b: &[Cons3]) -> f64 {
    a.iter().zip(b).fold(0.0f64, |m, (x, y)| (0..5).fold(m, |m, k| m.max((x[k] - y[k]).abs() / (1.0 + y[k].abs()))))
}

macro_rules! run_lazy {
    ($module:ident, $ctx:expr, $staging:expr, $grid:expr, $cfg:expr, $u0:expr, $n:expr) => {{
        let mut lazy = bowshock_cfd::gpu::$module::LazySolver3::new($ctx, $grid, $cfg).map_err(|e| format!("new: {e}"))?;
        lazy.staging = $staging;
        lazy.set_state($u0).map_err(|e| format!("set_state: {e}"))?;
        lazy.run($n, 16).map_err(|e| format!("run: {e}"))?;
        lazy.state_f64().map_err(|e| format!("state: {e}"))?
    }};
}

fn run_gpu(f32_precision: bool, staging: Staging, grid: &Grid3, cfg: &Config3, u0: &[Cons3], n: usize) -> Result<Vec<Cons3>, String> {
    catch_unwind(AssertUnwindSafe(|| -> Result<Vec<Cons3>, String> {
        let ctx = luma::Context::new().map_err(|e| format!("context: {e}"))?;
        Ok(if f32_precision { run_lazy!(f32s, &ctx, staging, grid, cfg, u0, n) } else { run_lazy!(df32s, &ctx, staging, grid, cfg, u0, n) })
    }))
    .unwrap_or_else(|e| Err(format!("panic: {}", panic_text(e))))
}

type Targets = (f64, f64);
const EXPLICIT: Targets = (1e-5, 3e-7);
/// The one-cell wedge amplifies the f32 round-off of the explicit scheme (df32 agrees to 6e-13, so the scheme is the same): 5e-5 after 30 iterations.
const EXPLICIT_WEDGE: Targets = (5e-5, 3e-7);
const IMPLICIT: Targets = (1e-3, 1e-4);
const DIAG: Targets = (1e-2, 1e-4);

fn implicit(line_solve: LineSolve3) -> Scheme3 {
    Scheme3::Implicit(ImplicitParams { cfl_max: 10.0, line_solve, ..ImplicitParams::DEFAULT })
}

struct Row {
    label: String,
    prec: &'static str,
    result: Result<f64, String>,
    target: f64,
}

/// One set-up: the CPU runs `n` iterations from the free stream, both precisions run the same from the same state.
fn compare(label: &str, grid: &Grid3, cfg: &Config3, n: usize, stagings: &[Staging], targets: Targets, rows: &mut Vec<Row>) {
    let cpu = catch_unwind(AssertUnwindSafe(|| {
        let mut s = Solver3::new(grid.clone(), cfg.clone());
        let u0 = s.u.clone();
        for _ in 0..n {
            s.step();
        }
        (u0, s.u)
    }));
    let (u0, u_cpu) = match cpu {
        Ok(x) if x.1.iter().all(|q| q.iter().all(|v| v.is_finite())) => x,
        Ok(_) => {
            rows.push(Row { label: label.into(), prec: "-", result: Err("the CPU run went non-finite".into()), target: 0.0 });
            return;
        }
        Err(e) => {
            rows.push(Row { label: label.into(), prec: "-", result: Err(format!("CPU panic: {}", panic_text(e))), target: 0.0 });
            return;
        }
    };
    for &staging in stagings {
        for (f32_precision, prec, target) in [(true, "f32", targets.0), (false, "df32", targets.1)] {
            let label = if stagings.len() > 1 { format!("{label} [{staging:?}]") } else { label.to_string() };
            let result = run_gpu(f32_precision, staging, grid, cfg, &u0, n).map(|u| state_diff(&u, &u_cpu));
            rows.push(Row { label, prec, result, target });
        }
    }
}

fn report(title: &str, rows: &[Row]) {
    println!("\n{title}\n| case | precision | max abs dU/(1+abs U) | target |\n|---|---|---|---|");
    let mut bad = Vec::new();
    for r in rows {
        let text = match &r.result {
            Ok(d) if !d.is_nan() && *d < r.target => format!("{d:.2e}"),
            Ok(d) => format!("FAIL {d:.2e}"),
            Err(e) => format!("ERROR {e}"),
        };
        println!("| {} | {} | {text} | {:.0e} |", r.label, r.prec, r.target);
        if !matches!(&r.result, Ok(d) if !d.is_nan() && *d < r.target) {
            bad.push(format!("{} {}: {text}", r.label, r.prec));
        }
    }
    assert!(bad.is_empty(), "{} rows are not green:\n{}", bad.len(), bad.join("\n"));
}

fn print_grid(name: &str, g: &Grid3) {
    println!(
        "{name}: {} x {} x {} cells, sym {:?}, {} axis run(s) {:?}, {} dead / {} wall cells",
        g.ni,
        g.nj,
        g.nk,
        g.sym,
        g.axis.len(),
        g.axis,
        g.part.iter().filter(|&&p| p == bowshock_cfd::grid3::PART_DEAD).count(),
        g.part.iter().filter(|&&p| p != bowshock_cfd::grid3::PART_DEAD && p != bowshock_cfd::grid3::PART_FLUID).count()
    );
}

/// The wedge: pressure-only k flux, no `L_k`, no k line, the source block on the diagonal of the j factor; two Mach numbers, three schemes.
#[test]
fn wedge_matches_cpu() {
    let mut rows = Vec::new();
    for mach in [0.5, 2.0] {
        let c = wedge(mach);
        let g = body_grid3(&c.grid).expect("wedge grid");
        print_grid(&format!("wedge M {mach}"), &g);
        compare(&format!("M {mach} explicit"), &g, &config(&c, Scheme3::Explicit), 30, &[Staging::Fusion, Staging::Fluxes], EXPLICIT_WEDGE, &mut rows);
        compare(&format!("M {mach} implicit block"), &g, &config(&c, implicit(LineSolve3::Block)), 30, &[Staging::Fluxes], IMPLICIT, &mut rows);
        compare(&format!("M {mach} implicit diag"), &g, &config(&c, implicit(LineSolve3::DiagonalCorrected)), 20, &[Staging::Fluxes], DIAG, &mut rows);
    }
    report("S1 wedge (tiny), 30 / 20 iterations from the free stream", &rows);
}

/// The base-wake half domain at 2 degrees (two axis runs, dead cells, mirror axis ghost).
#[test]
fn base_half_domain_matches_cpu() {
    let mut rows = Vec::new();
    for mach in [0.5, 2.0] {
        let c = half(mach);
        let g = body_grid3(&c.grid).expect("half-domain grid");
        print_grid(&format!("base half M {mach}"), &g);
        compare(&format!("M {mach} explicit"), &g, &config(&c, Scheme3::Explicit), 30, &[Staging::Fluxes], EXPLICIT, &mut rows);
        compare(&format!("M {mach} implicit block"), &g, &config(&c, implicit(LineSolve3::Block)), 30, &[Staging::Fluxes], IMPLICIT, &mut rows);
        compare(&format!("M {mach} implicit diag"), &g, &config(&c, implicit(LineSolve3::DiagonalCorrected)), 20, &[Staging::Fluxes], DIAG, &mut rows);
    }
    report("S1 base-wake half domain (tiny, alpha 2 deg)", &rows);
}

/// Fin sectors: even `N` (same-cell axis ghost, `-I`) and odd `N` (mirrored cell, interpolation) at zero incidence.
#[test]
fn fin_sectors_match_cpu() {
    let mut rows = Vec::new();
    for (n, nk) in [(4u32, 6usize), (3, 6), (3, 7)] {
        let c = sector(2.0, n, nk);
        let g = body_grid3(&c.grid).expect("sector grid");
        print_grid(&format!("sector N {n}"), &g);
        compare(&format!("N {n} nk {nk} explicit"), &g, &config(&c, Scheme3::Explicit), 30, &[Staging::Fluxes], EXPLICIT, &mut rows);
        compare(&format!("N {n} nk {nk} implicit block"), &g, &config(&c, implicit(LineSolve3::Block)), 30, &[Staging::Fluxes], IMPLICIT, &mut rows);
        compare(&format!("N {n} nk {nk} implicit diag"), &g, &config(&c, implicit(LineSolve3::DiagonalCorrected)), 20, &[Staging::Fluxes], DIAG, &mut rows);
    }
    report("S1 fin sectors (tiny), M 2", &rows);
}

/// The forces reduced on the device (`GpuRunner3::wall_forces`: class masks and the wall-pressure extrapolation as array ops) against the host integration
/// `run3::wall_forces` of the downloaded state: every class, the whole wall and the part up to the base station, on all three topologies.
#[test]
fn device_forces_match_host_integration() {
    use bowshock_cfd::gpu::Precision;
    use bowshock_core::engine::gpu3::GpuRunner3;
    use bowshock_core::engine::run3::wall_forces;
    let cases = [("wedge", wedge(2.0)), ("base half", half(2.0)), ("sector N 4", sector(2.0, 4, 6)), ("sector N 3", sector(2.0, 3, 6))];
    let mut bad = Vec::new();
    println!("\n| case | precision | classes | max abs diff / scale (all faces, cut) |\n|---|---|---|---|");
    for (name, case) in &cases {
        for (prec, target) in [(Precision::F32, 2e-4), (Precision::Extended, 1e-8)] {
            let mut r = GpuRunner3::new(case, prec).expect("runner");
            r.gpu.run(30).expect("run");
            let u = r.gpu.state_f64().expect("state");
            let p_inf = r.host.cfg.freestream[4];
            let (all, cut) = r.wall_forces().expect("device forces");
            let g = &r.host.grid;
            let mut worst = 0.0f64;
            for (dev, x_max) in [(&all, f64::INFINITY), (&cut, case.body.x_base())] {
                let host = wall_forces(g, &u, p_inf, x_max);
                let scale = 1e-3 + host.by.iter().flat_map(|(_, a)| a.iter().map(|v| v.abs())).fold(0.0, f64::max);
                for (c, a) in &host.by {
                    let d = dev.get(*c);
                    for q in 0..3 {
                        worst = worst.max((d[q] - a[q]).abs() / scale);
                    }
                }
                // a class the host has no cut face for must be zero on the device
                for (c, d) in &dev.by {
                    if !host.by.iter().any(|(h, _)| h == c) {
                        worst = worst.max(d.iter().fold(0.0f64, |m, v| m.max(v.abs())) / scale);
                    }
                }
            }
            println!("| {name} | {prec:?} | {} | {worst:.2e} |", all.by.len());
            if worst.is_nan() || worst >= target {
                bad.push(format!("{name} {prec:?}: {worst:.2e} >= {target:.0e}"));
            }
        }
    }
    assert!(bad.is_empty(), "device forces differ from the host integration:\n{}", bad.join("\n"));
}
