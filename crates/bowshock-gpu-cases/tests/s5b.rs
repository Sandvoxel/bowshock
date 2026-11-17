//! Stage S5b: the viscous (and SST) 3D step on the GPU against the CPU `Solver3` with `set_viscous`, in the pattern of `euler3d.rs`: the same grid and state,
//! `n` iterations from the free stream, `max |dU| / (1 + |U|)` of the conserved state (and, for SST, of `k`, `omega`, `mu_t`) against the f64 CPU.
//! Tiny grids, f32 and df32, explicit scheme and the implicit block scheme. The diagonal line solve of a viscous run is not ported (a wedge runs on `Block`).
//! `cargo test --release -p bowshock-gpu-cases --test s5b -- --nocapture` (through gpuq).

use bowshock_cfd::flux::Limiter;
use bowshock_cfd::gas3::{freestream3, Cons3};
use bowshock_cfd::gpu::Staging;
use bowshock_cfd::grid3::{Grid3, Part, SolidBox};
use bowshock_cfd::implicit::{ImplicitParams, LineSolve3};
use bowshock_cfd::solver::{Turbulence, Viscous};
use bowshock_cfd::solver3::{Config3, Scheme3, Solver3};
use std::panic::{catch_unwind, AssertUnwindSafe};

/// A `10 x 8 x 6` box of cubes (a slightly sheared grid) with a no-slip block on the floor and a thin dead block: `j = 0` is the slip floor of the domain, `k`
/// the symmetry planes.
fn box_grid() -> Grid3 {
    let (n, h) = ([10usize, 8, 6], [0.1, 0.1, 0.1]);
    let (mut x, mut y, mut z) = (vec![], vec![], vec![]);
    for k in 0..=n[2] {
        for j in 0..=n[1] {
            for i in 0..=n[0] {
                let (fi, fj, fk) = (i as f64, j as f64, k as f64);
                x.push(fi * h[0] + 0.01 * fj);
                y.push(fj * h[1] * (1.0 + 0.03 * fi / 10.0));
                z.push(fk * h[2]);
            }
        }
    }
    let mut g = Grid3::try_from_nodes(n[0], n[1], n[2], x, y, z).unwrap();
    g.mark_solid(SolidBox { i: 3..5, j: 0..3, k: 2..4 }, Some(Part::Body(0)));
    g.mark_solid(SolidBox { i: 7..8, j: 0..1, k: 0..1 }, None);
    g
}

fn viscous(turbulence: Turbulence, re_c: f64) -> Viscous {
    Viscous { re_c, sutherland: 0.368, turbulence, wall_i_start: 0, roughness: 0.0 }
}

fn host(grid: &Grid3, mach: f64, scheme: Scheme3, v: Viscous) -> Solver3 {
    let cfg = Config3 { freestream: freestream3(mach, 0.0), cfl: 0.5, limiter: Limiter::VanAlbada, first_order_iters: 5, low_mach: false, scheme };
    let mut s = Solver3::new(grid.clone(), cfg);
    s.set_viscous(Some(v));
    s
}

fn panic_text(e: Box<dyn std::any::Any + Send>) -> String {
    let s = e.downcast_ref::<String>().cloned().or_else(|| e.downcast_ref::<&str>().map(|s| s.to_string())).unwrap_or_else(|| "panic".into());
    s.lines().next().unwrap_or("").chars().take(200).collect()
}

macro_rules! run_lazy {
    ($module:ident, $ctx:expr, $host:expr, $n:expr) => {{
        let mut lazy = bowshock_cfd::gpu::$module::LazySolver3::from_host($ctx, $host).map_err(|e| format!("from_host: {e}"))?;
        lazy.staging = Staging::Fluxes;
        lazy.run($n, 16).map_err(|e| format!("run: {e}"))?;
        (lazy.state_f64().map_err(|e| format!("state: {e}"))?, lazy.turb_f64().map_err(|e| format!("turb: {e}"))?)
    }};
}

type Gpu = (Vec<Cons3>, Option<(Vec<[f64; 2]>, Vec<f64>)>);

fn run_gpu(f32_precision: bool, host: &Solver3, n: usize) -> Result<Gpu, String> {
    catch_unwind(AssertUnwindSafe(|| -> Result<Gpu, String> {
        let ctx = luma::Context::new().map_err(|e| format!("context: {e}"))?;
        Ok(if f32_precision { run_lazy!(f32s, &ctx, host, n) } else { run_lazy!(df32s, &ctx, host, n) })
    }))
    .unwrap_or_else(|e| Err(format!("panic: {}", panic_text(e))))
}

struct Row {
    label: String,
    prec: &'static str,
    state: Result<f64, String>,
    turb: Option<f64>,
    target: f64,
}

fn rel(a: &[f64], b: &[f64]) -> f64 {
    a.iter().zip(b).fold(0.0f64, |m, (x, y)| m.max((x - y).abs() / (1.0 + y.abs())))
}

/// One set-up: the CPU runs `n` iterations from the free stream, both GPU precisions run the same.
fn compare(label: &str, grid: &Grid3, mach: f64, scheme: Scheme3, v: Viscous, n: usize, targets: (f64, f64), rows: &mut Vec<Row>) {
    let mut cpu = host(grid, mach, scheme, v);
    let fresh = host(grid, mach, scheme, v);
    let cpu_ok = catch_unwind(AssertUnwindSafe(|| {
        for _ in 0..n {
            cpu.step();
        }
    }));
    if let Err(e) = cpu_ok {
        rows.push(Row { label: label.into(), prec: "-", state: Err(format!("CPU panic: {}", panic_text(e))), turb: None, target: 0.0 });
        return;
    }
    if !cpu.u.iter().all(|q| q.iter().all(|x| x.is_finite())) {
        rows.push(Row { label: label.into(), prec: "-", state: Err("the CPU run went non-finite".into()), turb: None, target: 0.0 });
        return;
    }
    let flat = |u: &[Cons3]| -> Vec<f64> { u.iter().flatten().copied().collect() };
    for (f32_precision, prec, target) in [(true, "f32", targets.0), (false, "df32", targets.1)] {
        let r = run_gpu(f32_precision, &fresh, n);
        let (state, turb) = match r {
            Ok((u, t)) => {
                let st = rel(&flat(&u), &flat(&cpu.u));
                let tb = match (t, cpu.rans()) {
                    (Some((k, mu_t)), Some(r)) if !r.turb.is_empty() => {
                        let kk: Vec<f64> = k.iter().flatten().copied().collect();
                        let cc: Vec<f64> = r.turb.iter().flatten().copied().collect();
                        Some(rel(&kk, &cc).max(rel(&mu_t, &r.mu_t)))
                    }
                    _ => None,
                };
                (Ok(st), tb)
            }
            Err(e) => (Err(e), None),
        };
        rows.push(Row { label: label.into(), prec, state, turb, target });
    }
}

fn report(title: &str, rows: &[Row]) {
    println!("\n{title}\n| case | precision | max abs dU/(1+abs U) | k, omega, mu_t | target |\n|---|---|---|---|---|");
    let mut bad = Vec::new();
    for r in rows {
        let ok = |d: f64| !d.is_nan() && d < r.target;
        let state = match &r.state {
            Ok(d) if ok(*d) => format!("{d:.2e}"),
            Ok(d) => format!("FAIL {d:.2e}"),
            Err(e) => format!("ERROR {e}"),
        };
        let turb = r.turb.map_or("-".to_string(), |t| format!("{t:.2e}"));
        println!("| {} | {} | {state} | {turb} | {:.0e} |", r.label, r.prec, r.target);
        if !matches!(&r.state, Ok(d) if ok(*d)) || r.turb.is_some_and(|t| !ok(t)) {
            bad.push(format!("{} {}: {state} / {turb}", r.label, r.prec));
        }
    }
    assert!(bad.is_empty(), "{} rows are not green:\n{}", bad.len(), bad.join("\n"));
}

const EXPLICIT: (f64, f64) = (1e-5, 3e-7);
const IMPLICIT: (f64, f64) = (1e-3, 1e-4);

fn block() -> Scheme3 {
    Scheme3::Implicit(ImplicitParams { line_solve: LineSolve3::Block, ..ImplicitParams::VISCOUS })
}

/// Laminar viscous runs: no-slip block, dead block, static-pressure outflow at `i = ni`, symmetry planes in `k` and the slip floor.
#[test]
fn laminar_box_matches_cpu() {
    let g = box_grid();
    let mut rows = Vec::new();
    for (mach, re) in [(0.3, 500.0), (2.0, 2e3)] {
        let v = viscous(Turbulence::Laminar, re);
        compare(&format!("M {mach} Re {re} explicit"), &g, mach, Scheme3::Explicit, v, 30, EXPLICIT, &mut rows);
        compare(&format!("M {mach} Re {re} implicit block"), &g, mach, block(), v, 30, IMPLICIT, &mut rows);
    }
    report("S5b laminar viscous box (10 x 8 x 6, no-slip block, dead block), 30 iterations from the free stream", &rows);
}
