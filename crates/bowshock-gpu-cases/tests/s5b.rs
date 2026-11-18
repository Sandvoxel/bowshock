//! Stage S5b: the viscous (and SST) 3D step on the GPU against the CPU `Solver3` with `set_viscous`, in the pattern of `euler3d.rs`: the same grid and state,
//! `n` iterations from the free stream, `max |dU| / (1 + |U|)` of the conserved state (and, for SST, of `k`, `omega`, `mu_t`) against the f64 CPU.
//! Tiny grids, f32 and df32, explicit scheme and the implicit block scheme. The diagonal line solve of a viscous run is not ported (a wedge runs on `Block`).
//! `cargo test --release -p bowshock-gpu-cases --test s5b -- --nocapture` (through gpuq).

use bowshock_cfd::gas3::{freestream3, Cons3};
use bowshock_cfd::gpu::Staging;
use bowshock_cfd::grid3::{Grid3, Part, SolidBox};
use bowshock_cfd::implicit_params::{ImplicitParams, LineSolve3};
use bowshock_cfd::limiter::Limiter;
use bowshock_cfd::solver3::{Config3, Scheme3, Solver3};
use bowshock_cfd::sst::{Turbulence, Viscous};
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
    let low_mach = matches!(scheme, Scheme3::Implicit(p) if p.low_mach_precond);
    let cfg = Config3 { freestream: freestream3(mach, 0.0), cfl: 0.5, limiter: Limiter::VanAlbada, first_order_iters: 5, low_mach, scheme };
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
    compare_with(label, &|| host(grid, mach, scheme, v), n, targets, rows)
}

/// [`compare`] for the host solver `make` builds (called twice: the CPU run and the state the GPU starts from).
fn compare_with(label: &str, make: &dyn Fn() -> Solver3, n: usize, targets: (f64, f64), rows: &mut Vec<Row>) {
    let mut cpu = make();
    let fresh = make();
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
                        // the CPU update zeroes the entries of the solid cells, the motor-jet cells included (their jet state is the ghost the exit faces read, re-stored when
                        // it is read); the GPU keeps the jet state in the cell: compare the cells the CPU holds
                        let keep: Vec<usize> = (0..r.turb.len()).filter(|&c| r.turb[c] != [0.0, 0.0]).collect();
                        let kk: Vec<f64> = keep.iter().flat_map(|&c| k[c]).collect();
                        let cc: Vec<f64> = keep.iter().flat_map(|&c| r.turb[c]).collect();
                        let (mg, mc): (Vec<f64>, Vec<f64>) = keep.iter().map(|&c| (mu_t[c], r.mu_t[c])).unzip();
                        Some(rel(&kk, &cc).max(rel(&mg, &mc)))
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

/// SST with the automatic wall treatment (law-of-the-wall strain and `k` cap in the wall cells): the state and `k`, `omega`, `mu_t` after 30 iterations; two
/// Reynolds numbers put the first cell of the block walls at a small and at a large `y+`.
#[test]
fn sst_box_matches_cpu() {
    let g = box_grid();
    let mut rows = Vec::new();
    for (mach, re) in [(0.3, 2e3), (0.3, 2e5)] {
        let v = viscous(Turbulence::Sst, re);
        compare(&format!("SST M {mach} Re {re:.0e} explicit"), &g, mach, Scheme3::Explicit, v, 30, EXPLICIT, &mut rows);
        compare(&format!("SST M {mach} Re {re:.0e} implicit block"), &g, mach, block(), v, 30, IMPLICIT, &mut rows);
    }
    report("S5b SST box (10 x 8 x 6, no-slip block, dead block), 30 iterations from the free stream", &rows);
}

/// The low-Mach preconditioning of the block scheme (`P` in the factors, between the sweeps and in the defect, `P |P^-1 A|` and `|u'| + c'` in the left-hand side,
/// the low-Mach flux correction on the right) at Mach 0.1 and 0.3: laminar and SST; 30 iterations from the free stream.
#[test]
fn low_mach_block_matches_cpu() {
    let g = box_grid();
    let mut rows = Vec::new();
    let pre = Scheme3::Implicit(ImplicitParams { line_solve: LineSolve3::Block, low_mach_precond: true, ..ImplicitParams::VISCOUS });
    for (mach, re) in [(0.1, 500.0), (0.3, 2e3)] {
        compare(&format!("laminar M {mach} Re {re} block + P"), &g, mach, pre, viscous(Turbulence::Laminar, re), 30, IMPLICIT, &mut rows);
    }
    compare("SST M 0.1 Re 2e3 block + P", &g, 0.1, pre, viscous(Turbulence::Sst, 2e3), 30, IMPLICIT, &mut rows);
    report("S5b low-Mach preconditioning (10 x 8 x 6 box, no-slip block, dead block), 30 iterations from the free stream", &rows);
}

/// DDADI and the dissipation boost on the viscous block scheme (`ImplicitParams::ddadi`, `lhs_dis_boost`): the laminar box (three directions, no-slip block and dead block cut the
/// lines, static-pressure outflow) at Mach 0.3 and 2, `omega` 1 and 0.5 with the boost 0.35 and the boost alone, a steep ramp (CFL 3 -> 20; the boost-only rows, whose plain factors cannot take it, CFL 1 -> 4.3) so that the factorisation error is large;
/// and Mach 0.1 with the low-Mach preconditioning (`D` between the factors where `P` stood). 30 iterations from the free stream, the targets of the plain block scheme.
#[test]
fn ddadi_block_matches_cpu() {
    let g = box_grid();
    let mut rows = Vec::new();
    let scheme = |ddadi: Option<f64>, boost: f64, precond: bool| {
        Scheme3::Implicit(ImplicitParams {
            line_solve: LineSolve3::Block,
            // the plain factors (no DDADI) diverge on both sides on the steep ramp at Mach 2: the boost-only rows use a gentle one (CFL 1 -> 4.3)
            cfl0: if ddadi.is_some() { 3.0 } else { 1.0 },
            growth: if ddadi.is_some() { 1.25 } else { 1.05 },
            cfl_max: 20.0,
            low_mach_precond: precond,
            ddadi,
            lhs_dis_boost: boost,
            ..ImplicitParams::VISCOUS
        })
    };
    for (mach, re, precond) in [(0.3, 500.0, false), (2.0, 2e3, false), (0.1, 500.0, true)] {
        let v = viscous(Turbulence::Laminar, re);
        let pre = if precond { " + P" } else { "" };
        for (tag, ddadi, boost) in
            [("ddadi 1 + boost 0.35", Some(1.0), 0.35), ("ddadi 0.5 + boost 0.35", Some(0.5), 0.35), ("ddadi 0.5", Some(0.5), 0.0), ("boost 0.35", None, 0.35)]
        {
            compare(&format!("laminar M {mach} Re {re} {tag}{pre}"), &g, mach, scheme(ddadi, boost, precond), v, 30, IMPLICIT, &mut rows);
        }
    }
    compare("SST M 0.3 Re 2e3 ddadi 1 + boost 0.35", &g, 0.3, scheme(Some(1.0), 0.35, false), viscous(Turbulence::Sst, 2e3), 30, IMPLICIT, &mut rows);
    report("S5b DDADI and dissipation boost (10 x 8 x 6 box, no-slip block, dead block), 30 iterations from the free stream", &rows);
}

/// The viscous single-cell wedge of an axisymmetric body (`Case3::wedge_of`, the G3 ogive-cylinder at Mach 0.5, level 0, SST, the 2D case's numerics: block lines,
/// the low-Mach correction and preconditioning, characteristic outflow; the body surface is `Grid3::wall_j0`, a no-slip wall of a viscous run) after 20 iterations:
/// the swirl projection, the tangential wall speed, the no-slip `j = 0` ghost and its Jacobian, `P` with two factors.
#[test]
fn viscous_wedge_matches_cpu() {
    use bowshock_cfd::solver::Scheme;
    use bowshock_cfd::solver3::RansOpts3;
    use bowshock_core::engine::body_grid::{BaseBody, Body, Nose};
    use bowshock_core::engine::case::Case;
    use bowshock_core::engine::case3::Case3;
    use bowshock_core::{Atmosphere, Wake};

    let air = Atmosphere::Isa { altitude_m: 0.0 }.state().unwrap();
    let (re_c, sutherland) = (air.density * air.sound_speed * 0.1 / air.viscosity, 110.4 / air.temperature_k);
    let body = BaseBody::cylinder(Body { nose: Nose::TangentOgive { fineness: 3.0 } }, 6.0);
    let case = Case::viscous(0.5, body, 0, Wake::Base, re_c, sutherland, 0.5);
    // `dd`: DDADI (`omega`, `lhs_dis_boost`) on a steep ramp (CFL 3 -> its cap in 10 iterations), `None`: the case's own numerics
    let make = |turbulence: Turbulence, dd: Option<(f64, f64)>| -> Solver3 {
        let c3 = Case3::wedge_of(&case, 0);
        let mut s = c3.build_solver().unwrap();
        let n = &case.numerics;
        let Scheme::Implicit(p) = n.scheme else { panic!("the viscous numerics are implicit") };
        let p = match dd {
            Some((omega, boost)) => ImplicitParams { cfl0: 3.0, growth: 1.25, ddadi: Some(omega), lhs_dis_boost: boost, ..p },
            None => p,
        };
        s.cfg.scheme = Scheme3::Implicit(ImplicitParams { line_solve: LineSolve3::Block, ..p });
        s.cfg.limiter = n.limiter;
        s.cfg.low_mach = n.low_mach;
        s.cfg.first_order_iters = n.first_order_iters;
        let visc = case.viscous.map(|v| Viscous { turbulence, ..v });
        s.set_viscous_with(visc, RansOpts3 { pressure_outflow: n.outflow == bowshock_cfd::solver::Bc::PressureOutflow, ..RansOpts3::default() });
        s.restart_ramp();
        s
    };
    let mut rows = Vec::new();
    compare_with("G3 wedge laminar M 0.5 block + P", &|| make(Turbulence::Laminar, None), 20, IMPLICIT, &mut rows);
    compare_with("G3 wedge SST M 0.5 block + P", &|| make(Turbulence::Sst, None), 20, IMPLICIT, &mut rows);
    // DDADI on the wedge: two factors, `D = d_i + d_j - P` (the source block on the j diagonal), `D` between them
    compare_with("G3 wedge laminar M 0.5 ddadi 1 + boost 0.35 + P", &|| make(Turbulence::Laminar, Some((1.0, 0.35))), 20, IMPLICIT, &mut rows);
    compare_with("G3 wedge SST M 0.5 ddadi 0.5 + boost 0.35 + P", &|| make(Turbulence::Sst, Some((0.5, 0.35))), 20, IMPLICIT, &mut rows);
    report("S5b viscous wedge (G3 ogive-cylinder, level 0), 20 iterations from the free stream", &rows);
}

/// The motor jet on the wedge (`Case3::wedge` with `with_motor`: the ANF body, the nozzle rows of the base-wake grid, two columns of `Part::Jet` cells ahead of the base whose
/// `+i` faces are an inflow of the jet state): the exit-face stencil `(jet, jet, p0, p1)`, the jet state kept in the jet cells, the implicit factor without wall Jacobian and
/// sub-diagonal at the exit, and the viscous / scalar ghost of the jet (SST: `mu(T_jet)`, `rho_jet k / omega`, the gradient of the fluid cell behind it). 30 iterations from the free
/// stream against the f64 CPU, Euler and SST, explicit and block.
#[test]
fn motor_jet_wedge_matches_cpu() {
    use bowshock_core::engine::case3::Case3;
    use bowshock_core::engine::meshing::Jet;

    let make = |turbulence: Option<Turbulence>, explicit: bool| -> Solver3 {
        let jet = Jet { r_exit: 0.25, state: [0.5, 2.0, 0.0, 0.5], turb: [1e-4, 10.0] };
        let mut c = Case3::wedge(2.0, Case3::anf_body(), 0).with_motor(jet).expect("motor");
        c.viscous = turbulence.map(|turbulence| Viscous { re_c: 2.0e5, sutherland: 0.368, turbulence, wall_i_start: 0, roughness: 0.0 });
        let mut s = c.build_solver().expect("solver");
        assert!(s.jet().is_some(), "the case carries the jet");
        if explicit {
            s.cfg.scheme = Scheme3::Explicit;
        }
        s
    };
    let mut rows = Vec::new();
    // the Euler jet exit is a strong start-up transient (jet p 0.5 against the free stream): single precision rounding grows to 2.3e-5 over the 30 explicit steps (the same
    // exit with the SST terms: 1.5e-6, and df32 agrees to 7e-12: the logic is the CPU's), so the f32 target of this row is 5e-5
    compare_with("jet Euler explicit", &|| make(None, true), 30, (5e-5, EXPLICIT.1), &mut rows);
    compare_with("jet Euler block", &|| make(None, false), 30, IMPLICIT, &mut rows);
    compare_with("jet SST explicit", &|| make(Some(Turbulence::Sst), true), 30, EXPLICIT, &mut rows);
    compare_with("jet SST block", &|| make(Some(Turbulence::Sst), false), 30, IMPLICIT, &mut rows);
    report("S5b motor jet on the wedge (ANF body, jet 0.25 d, Mach 2), 30 iterations from the free stream", &rows);
}

/// [`host`] with isothermal no-slip walls at `T_w / T_inf = t_wall` (`RansOpts3::wall_thermal`).
fn host_isothermal(grid: &Grid3, mach: f64, scheme: Scheme3, v: Viscous, t_wall: f64) -> Solver3 {
    use bowshock_cfd::solver3::RansOpts3;
    use bowshock_cfd::sst::WallThermal;
    let low_mach = matches!(scheme, Scheme3::Implicit(p) if p.low_mach_precond);
    let cfg = Config3 { freestream: freestream3(mach, 0.0), cfl: 0.5, limiter: Limiter::VanAlbada, first_order_iters: 5, low_mach, scheme };
    let mut s = Solver3::new(grid.clone(), cfg);
    s.set_viscous_with(Some(v), RansOpts3 { wall_thermal: WallThermal::Isothermal(t_wall), ..RansOpts3::default() });
    s
}

/// Isothermal walls (`WallThermal::Isothermal`: the viscous ghost `2 T_w - T`, the wall viscosity `mu(T_w)`, the law of the wall at the wall temperature) on the box of the
/// laminar and SST runs: the wall at 0.8 `T_inf` under the Mach 0.3 flow (heat to the wall) and at 1.2 under the Mach 2 flow (both signs, the stagnation temperature 1.8), explicit
/// and implicit block, 30 iterations from the free stream, the targets of the adiabatic rows.
#[test]
fn isothermal_box_matches_cpu() {
    let g = box_grid();
    let mut rows = Vec::new();
    for (turbulence, mach, re, t_wall) in [
        (Turbulence::Laminar, 0.3, 500.0, 0.8),
        (Turbulence::Laminar, 2.0, 2e3, 1.2),
        (Turbulence::Sst, 0.3, 2e3, 0.8),
        (Turbulence::Sst, 0.3, 2e5, 0.8),
        (Turbulence::Sst, 2.0, 2e5, 1.2),
    ] {
        let v = viscous(turbulence, re);
        let name = format!("{turbulence:?} M {mach} Re {re:.0e} Tw {t_wall}");
        compare_with(&format!("{name} explicit"), &|| host_isothermal(&g, mach, Scheme3::Explicit, v, t_wall), 30, EXPLICIT, &mut rows);
        compare_with(&format!("{name} implicit block"), &|| host_isothermal(&g, mach, block(), v, t_wall), 30, IMPLICIT, &mut rows);
    }
    report("S5b isothermal viscous box (10 x 8 x 6, no-slip block, dead block), 30 iterations from the free stream", &rows);
}
