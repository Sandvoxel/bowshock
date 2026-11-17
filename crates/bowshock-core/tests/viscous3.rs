//! The 3D viscous (RANS) solver on the single-cell wedge of an axisymmetric body against the 2D solver: the CPU gates of `DESIGN3D.md` section L (G-V1
//! second half: the laminar wedge body; G-V3: the G3 ogive-cylinder at Mach 0.5, `tests/viscous.rs`). All `#[ignore]`: minutes of CPU each.
//!
//! The wedge is `Case3::wedge_of(&case2d)`: the 2D case's own grid parameters, so the 3D grid has the 2D grid's cells (`ni`, `nj`, centroids in `(x, r)` to the
//! chord effect of the flat faces). The numerics are the 2D case's (`Numerics::viscous`: van Albada, the low-Mach correction and preconditioning below
//! Mach 1, 200 first-order iterations, `cfl_max` 50 below Mach 1 and 20 above, `DtMode::Sum`) with the block line solve. Pressure forces are those of
//! `run3::drag_parts` (the engine's own wall integration); the friction is the viscous traction of the wall faces `Solver3::wall_tractions`.
//!
//! Environment knobs: `RE_C` (laminar test), `ITERS`, `CFL`, `LEVEL`, `DTHETA` (degrees), `LINES` (`block` | `diag`), `PRECOND` (`0` | `1`).

use bowshock_cfd::gas3::Cons3;
use bowshock_cfd::grid3::Grid3;
use bowshock_cfd::implicit::{ImplicitParams, LineSolve3};
use bowshock_cfd::solver::{Scheme, Solver, Turbulence};
use bowshock_cfd::solver3::{RansOpts3, Scheme3, Solver3};
use bowshock_core::engine::body_grid::{BaseBody, Body, Nose};
use bowshock_core::engine::case::Case;
use bowshock_core::engine::case3::Case3;
use bowshock_core::engine::grid3::Domain3;
use bowshock_core::engine::result::PointResult;
use bowshock_core::engine::{run, run3};
use bowshock_core::{Atmosphere, StopReason, Wake};
use std::time::Instant;

fn env<T: std::str::FromStr>(key: &str, default: T) -> T {
    std::env::var(key).ok().and_then(|v| v.parse().ok()).unwrap_or(default)
}

/// The G3 body of `tests/viscous.rs`: tangent ogive of fineness 3 and a tube of 6 calibres.
fn g3_body() -> BaseBody {
    BaseBody::cylinder(Body { nose: Nose::TangentOgive { fineness: 3.0 } }, 6.0)
}

/// Re_c and the Sutherland ratio of the ISA at sea level for a nose diameter of 0.1 m, as `Analysis::viscous` takes them.
fn flight() -> (f64, f64) {
    let air = Atmosphere::Isa { altitude_m: 0.0 }.state().unwrap();
    (air.density * air.sound_speed * 0.1 / air.viscosity, 110.4 / air.temperature_k)
}

fn g3_case(mach: f64, level: u32) -> Case {
    let (re_c, sutherland) = flight();
    Case::viscous(mach, g3_body(), level, Wake::Base, re_c, sutherland, mach)
}

/// The 3D solver on the wedge of `case` with the numerics of the 2D case and its viscous parameters.
fn build3(case: &Case, dtheta: f64, level: u32) -> (Case3, Solver3) {
    let mut c3 = Case3::wedge_of(case, level);
    c3.grid.domain = Domain3::Wedge { dtheta };
    let mut s = c3.build_solver().unwrap();
    let n = &case.numerics;
    let Scheme::Implicit(p) = n.scheme else { panic!("the viscous numerics are implicit") };
    let line_solve = match std::env::var("LINES").as_deref() {
        Ok("diag") => LineSolve3::DiagonalCorrected,
        _ => LineSolve3::Block,
    };
    let p = ImplicitParams {
        line_solve,
        cfl_max: env("CFL", p.cfl_max),
        roe: env("ROE", u8::from(p.roe)) != 0,
        sub_iters: env("SUB", p.sub_iters),
        implicit_bc: env("IBC", u8::from(p.implicit_bc)) != 0,
        low_mach_precond: env("PRECOND", u8::from(p.low_mach_precond)) != 0,
        ..p
    };
    s.cfg.scheme = Scheme3::Implicit(p);
    s.cfg.limiter = n.limiter;
    s.cfg.low_mach = n.low_mach;
    s.cfg.first_order_iters = n.first_order_iters;
    let visc = match std::env::var("MODEL").as_deref() {
        Ok("euler") => None,
        Ok("laminar") => case.viscous.map(|v| bowshock_cfd::solver::Viscous { turbulence: Turbulence::Laminar, ..v }),
        _ => case.viscous,
    };
    // the outflow plane of the 2D case: characteristic (the wall-resolved body cases) or static pressure (the plates)
    let opts = RansOpts3 { pressure_outflow: n.outflow == bowshock_cfd::solver::Bc::PressureOutflow, ..RansOpts3::default() };
    s.set_viscous_with(visc, opts);
    s.restart_ramp();
    (c3, s)
}

/// The forces of the wedge state in the 2D solver's units (drag coefficients on `pi/4` of the nose diameter 1).
#[derive(Clone, Copy, Debug, Default)]
struct Forces3 {
    cd_pressure: f64,
    cd_base: f64,
    cd_friction: f64,
    y_plus_max: f64,
}

impl Forces3 {
    fn total(&self) -> f64 {
        self.cd_pressure + self.cd_friction + self.cd_base
    }
}

fn forces3(c3: &Case3, s: &Solver3) -> Forces3 {
    let g = &s.grid;
    let fs = s.cfg.freestream;
    let parts = run3::drag_parts(c3, g, &s.u, fs[4]);
    let q = 0.5 * fs[0] * fs[1] * fs[1];
    let norm = (2.0 * std::f64::consts::PI / g.theta_span) / (q * std::f64::consts::FRAC_PI_4);
    let x_base = c3.body.x_base();
    let Some(rans) = s.rans() else {
        return Forces3 { cd_pressure: parts.nose + parts.aft.iter().sum::<f64>(), cd_base: parts.base, ..Forces3::default() };
    };
    let (mut cd_friction, mut y_plus_max) = (0.0, 0.0f64);
    for t in s.wall_tractions() {
        // the surface of the body ahead of the base plane: j faces (the base is an i face)
        if t.dir != 1 {
            continue;
        }
        let [i, j, k] = t.idx;
        let f = &g.jface[(k * (g.nj + 1) + j) * g.ni + i];
        if f.c[0] >= x_base {
            continue;
        }
        cd_friction += t.tau[0] * t.area;
        let cell = if g.part[g.cell(i, j, k)] == 0 { g.cell(i, j, k) } else { g.cell(i, j - 1, k) };
        let x = g.xc[cell];
        let d_n = ((x[0] - f.c[0]) * t.n[0] + (x[1] - f.c[1]) * t.n[1] + (x[2] - f.c[2]) * t.n[2]).abs();
        let tau = (t.tau[0] * t.tau[0] + t.tau[1] * t.tau[1] + t.tau[2] * t.tau[2]).sqrt();
        y_plus_max = y_plus_max.max(d_n * (s.u[cell][0] * tau).sqrt() / rans.mu[cell]);
    }
    Forces3 { cd_pressure: parts.nose + parts.aft.iter().sum::<f64>(), cd_base: parts.base, cd_friction: norm * cd_friction, y_plus_max }
}

/// The `i` faces that are no-slip walls (the base, the nose tip): row, wall distance of the first cell and its `y+`.
fn i_wall_report(s: &Solver3) {
    let g = &s.grid;
    let rans = s.rans().expect("viscous");
    for t in s.wall_tractions().iter().filter(|t| t.dir == 0) {
        let [i, j, k] = t.idx;
        let f = &g.iface[(k * g.nj + j) * (g.ni + 1) + i];
        let cell = if g.part[g.cell(i, j, k)] == 0 { g.cell(i, j, k) } else { g.cell(i - 1, j, k) };
        let x = g.xc[cell];
        let d_n = ((x[0] - f.c[0]) * t.n[0] + (x[1] - f.c[1]) * t.n[1] + (x[2] - f.c[2]) * t.n[2]).abs();
        let tau = (t.tau[0] * t.tau[0] + t.tau[1] * t.tau[1] + t.tau[2] * t.tau[2]).sqrt();
        let w = &s.u[cell];
        let yp = d_n * (w[0] * tau).sqrt() / rans.mu[cell];
        if j % 4 == 0 || j > 40 {
            println!(
                "  i-wall (i {i}, j {j}) r {:.4} d {d_n:.2e} y+ {yp:.2} tau {tau:.3e}  u_t {:.4}",
                f.c[1].hypot(f.c[2]),
                (w[2] * w[2] + w[3] * w[3]).sqrt() / w[0]
            );
        }
    }
}

/// `n` iterations of the 3D solver; the residual history. Prints a line every `every` iterations.
fn iterate3(c3: &Case3, s: &mut Solver3, n: usize, every: usize) -> Vec<f64> {
    let t0 = Instant::now();
    let mut hist = Vec::with_capacity(n);
    let trace = std::env::var_os("TRACE").is_some();
    let mut prev = s.u.clone();
    for it in 0..n {
        if trace {
            prev.clone_from(&s.u);
        }
        let r = s.step();
        hist.push(r);
        if trace && (it < 40 || it % 10 == 9) && it < 200 {
            let (sn, cs) = (0.5 * s.grid.theta_span).sin_cos();
            let (mut sw, mut at, mut rad) = (0.0f64, 0, 0.0f64);
            for (c, q) in s.u.iter().enumerate() {
                if s.grid.part[c] == 0 {
                    let t = (q[2] * cs - q[3] * sn).abs();
                    if t > sw {
                        (sw, at) = (t, c);
                    }
                    rad = rad.max((q[2] * sn + q[3] * cs).abs());
                }
            }
            println!("  3D it {:3}  r {r:.4e}  max swirl momentum {sw:.3e} at ({}, {})  max radial {rad:.3e}", it + 1, at % s.grid.ni, at / s.grid.ni);
        }
        if !r.is_finite() {
            println!("  3D: non-finite residual at iteration {}", it + 1);
            if trace {
                // the state before the failing step: the cell of the lowest pressure and of the largest speed
                let g = &s.grid;
                let (mut pmin, mut at_p, mut vmax, mut at_v) = (f64::INFINITY, 0, 0.0f64, 0);
                for (c, q) in prev.iter().enumerate() {
                    if g.part[c] != 0 {
                        continue;
                    }
                    let w = bowshock_cfd::gas3::cons_to_prim3(q);
                    let v = (w[1] * w[1] + w[2] * w[2] + w[3] * w[3]).sqrt();
                    if w[4] < pmin {
                        (pmin, at_p) = (w[4], c);
                    }
                    if v > vmax {
                        (vmax, at_v) = (v, c);
                    }
                }
                println!("  3D: before: p min {pmin:.4e} at ({}, {}), |v| max {vmax:.3} at ({}, {})", at_p % g.ni, at_p / g.ni, at_v % g.ni, at_v / g.ni);
            }
            break;
        }
        if every > 0 && (it + 1) % every == 0 {
            let f = forces3(c3, s);
            println!(
                "  3D it {:5}  r {:.3e}  cd {:.5} (p {:.5} f {:.5} base {:.5})  y+ {:.3}  {:.1} s",
                it + 1,
                r,
                f.total(),
                f.cd_pressure,
                f.cd_friction,
                f.cd_base,
                f.y_plus_max,
                t0.elapsed().as_secs_f64()
            );
        }
    }
    hist
}

/// The 2D solver of `case` for `n` iterations and its result record at the end.
fn run2(case: &Case, n: usize, every: usize) -> (Solver, PointResult, f64) {
    let mut s = case.build_solver();
    let t0 = Instant::now();
    let mut hist = Vec::with_capacity(n);
    for it in 0..n {
        let r = s.step();
        hist.push(r);
        if !r.is_finite() {
            println!("  2D: non-finite residual at iteration {}", it + 1);
            break;
        }
        if every > 0 && (it + 1) % every == 0 {
            println!("  2D it {:5}  r {:.3e}  {:.1} s", it + 1, r, t0.elapsed().as_secs_f64());
        }
    }
    let secs = t0.elapsed().as_secs_f64();
    let rmax = hist.iter().skip(20).fold(0.0f64, |m, &r| m.max(r));
    let res = run::finish(case, &s, hist, rmax, Vec::new(), StopReason::MaxIterations);
    (s, res, secs)
}

/// The `n` fluid cells where the 2D and the wedge state differ most (`|dU| / (1 + |U|)` of the component of the largest difference), with their position.
fn print_worst(s2: &Solver, s3: &Solver3, n: usize) {
    let g = &s3.grid;
    let (sn, cs) = (0.5 * g.theta_span).sin_cos();
    let mut diffs: Vec<(f64, usize)> = (0..g.ni * g.nj)
        .filter(|&c| g.part[c] == 0)
        .map(|c| {
            let (a, b) = (&s2.u[c], &s3.u[c]);
            let rad = b[2] * sn + b[3] * cs;
            let d = [
                (a[0] - b[0]).abs() / (1.0 + a[0].abs()),
                (a[1] - b[1]).abs() / (1.0 + a[1].abs()),
                (a[2] - rad).abs() / (1.0 + a[2].abs()),
                (a[3] - b[4]).abs() / (1.0 + a[3].abs()),
            ];
            (d.iter().copied().fold(0.0, f64::max), c)
        })
        .collect();
    diffs.sort_by(|a, b| b.0.total_cmp(&a.0));
    for (d, c) in diffs.iter().take(n) {
        println!("  |dU|/(1+|U|) {d:.3e} at ({}, {})  x {:.3} r {:.4}", c % g.ni, c / g.ni, g.xc[*c][0], g.xc[*c][1].hypot(g.xc[*c][2]));
    }
}

/// The largest `|dU| / (1 + |U|)` between the 2D state and the wedge's, over the fluid cells of centroid radius `r >= r_min` (the cells close to the axis have a
/// different discretisation in the two solvers: the 3D prism has its centroid at 2/3 of the row's radius where the axisymmetric 2D cell has 1/2, and the 3D Green-Gauss
/// gradient weighs the faces with their true areas where the 2D one is planar in the meridian plane; both are `O(h / r)`, not `O(dtheta^2)`), per conserved component
/// and where: the 3D momentum is rotated into the radial direction at the mid-plane of the wedge. The 3D swirl momentum is returned as the last entry of the first array.
fn state_difference(s2: &Solver, s3: &Solver3, r_min: f64) -> ([f64; 5], (usize, usize)) {
    let g3: &Grid3 = &s3.grid;
    assert_eq!((s2.grid.ni, s2.grid.nj), (g3.ni, g3.nj), "same cells");
    let th = 0.5 * g3.theta_span;
    let (sn, cs) = th.sin_cos();
    let (mut worst, mut at) = ([0.0f64; 5], (0, 0));
    let mut top = 0.0;
    for j in 0..g3.nj {
        for i in 0..g3.ni {
            let c = j * g3.ni + i;
            if g3.part[c] != 0 || g3.xc[c][1].hypot(g3.xc[c][2]) < r_min {
                continue;
            }
            let (a, b): (&[f64; 4], &Cons3) = (&s2.u[c], &s3.u[c]);
            let rad = b[2] * sn + b[3] * cs;
            let swirl = b[2] * cs - b[3] * sn;
            let d = [
                (a[0] - b[0]).abs() / (1.0 + a[0].abs()),
                (a[1] - b[1]).abs() / (1.0 + a[1].abs()),
                (a[2] - rad).abs() / (1.0 + a[2].abs()),
                (a[3] - b[4]).abs() / (1.0 + a[3].abs()),
                swirl.abs(),
            ];
            for q in 0..5 {
                worst[q] = worst[q].max(d[q]);
            }
            let m = d[..4].iter().copied().fold(0.0, f64::max);
            if m > top {
                top = m;
                at = (i, j);
            }
        }
    }
    (worst, at)
}

/// The wedge state as the 2D state of its cells: `(rho, rho u, rho v_r, E)`, the radial momentum from the Cartesian components at the mid-plane of the wedge.
fn as_2d(s3: &Solver3) -> Vec<[f64; 4]> {
    let (sn, cs) = (0.5 * s3.grid.theta_span).sin_cos();
    s3.u.iter().map(|b| [b[0], b[1], b[2] * sn + b[3] * cs, b[4]]).collect()
}

/// The largest `|dU| / (1 + |U|)` between two 2D-form states `a` (the reference) and `b` over the fluid cells of `g` with `r >= r_min`.
fn array_difference(a: &[[f64; 4]], b: &[[f64; 4]], g: &Grid3, r_min: f64) -> f64 {
    (0..g.ni * g.nj)
        .filter(|&c| g.part[c] == 0 && g.xc[c][1].hypot(g.xc[c][2]) >= r_min)
        .map(|c| (0..4).map(|q| (a[c][q] - b[c][q]).abs() / (1.0 + a[c][q].abs())).fold(0.0, f64::max))
        .fold(0.0, f64::max)
}

/// What the grid of the wedge looks like: dead cells, walls, axis runs and the fluid / dead faces.
#[test]
#[ignore]
fn wedge_grid_report() {
    let case = g3_case(0.5, 0);
    let c3 = Case3::wedge_of(&case, 0);
    let s = c3.build_solver().unwrap();
    let g = &s.grid;
    let dead = g.part.iter().filter(|&&p| p == bowshock_cfd::grid3::PART_DEAD).count();
    let walls = g.part.iter().filter(|&&p| p != 0 && p != bowshock_cfd::grid3::PART_DEAD).count();
    println!("wedge grid {} x {} x {}: {dead} dead cells, {walls} wall cells", g.ni, g.nj, g.nk);
    println!("axis runs {:?}  wall_j0 {:?}  parts {:?}  sym {:?}  theta_span {}", g.axis, g.wall_j0, g.parts, g.sym, g.theta_span);
    println!("viscous {:?}", case.viscous.map(|v| (v.re_c, v.sutherland, v.wall_i_start)));
    let s2 = case.build_solver();
    println!("2D grid {} x {}", s2.grid.ni, s2.grid.nj);
}

/// G-V1, second half: the laminar wedge body against the 2D CPU state. A laminar low-Reynolds-number case (`RE_C`, Re_d = 0.5 Re_c) on the G3 body's grid
/// of Mach 0.5: both solvers iterate `ITERS` times; `max |dU| / (1 + |U|)` of the converged states at 2 degrees (<= 5e-4) and at 1 degree (a quarter of it).
#[test]
#[ignore]
fn laminar_wedge_matches_the_2d_state() {
    // minimum radii of the cells of the comparison: all cells, and those away from the axis
    const R_MINS: [f64; 3] = [0.0, 0.1, 0.25];
    let re_c: f64 = env("RE_C", 1000.0);
    let iters: usize = env("ITERS", 3000);
    let level: u32 = env("LEVEL", 0);
    let mut case = Case::viscous(0.5, g3_body(), level, Wake::Base, re_c, flight().1, 0.5);
    if let Some(v) = case.viscous.as_mut() {
        v.turbulence = Turbulence::Laminar;
    }
    if std::env::var_os("EULER").is_some() {
        // the Euler equations on the same grid: the geometry and axis of the wedge without the viscous terms
        case.viscous = None;
    }
    let (s2, r2, secs2) = run2(&case, iters, 500);
    println!(
        "G-V1 wedge: 2D after {iters} iterations ({:.1} s): cd p {:.5} f {:.5} base {:.5}  cp_base {:.4}  y+ {:.3}",
        secs2,
        r2.cd_nose + r2.cd_aft.iter().sum::<f64>(),
        r2.cd_friction,
        r2.cd_base,
        r2.cp_base,
        r2.y_plus_max
    );
    let mut worst = Vec::new();
    let mut states: Vec<(f64, Vec<[f64; 4]>)> = Vec::new();
    // the last angle is the reference of the angle convergence
    let degs: Vec<f64> = std::env::var("DEGS").unwrap_or_else(|_| "2,1,0.5".into()).split(',').filter_map(|d| d.parse().ok()).collect();
    let mut grid = None;
    for deg in degs {
        let (c3, mut s3) = build3(&case, deg.to_radians(), level);
        let t0 = Instant::now();
        iterate3(&c3, &mut s3, iters, 500);
        let secs = t0.elapsed().as_secs_f64();
        let f = forces3(&c3, &s3);
        print_worst(&s2, &s3, 4);
        let mut row = Vec::new();
        for r_min in R_MINS {
            let (d, at) = state_difference(&s2, &s3, r_min);
            let m = d[..4].iter().copied().fold(0.0, f64::max);
            println!(
                "G-V1 wedge {deg} deg vs 2D, cells with r >= {r_min}: max |dU|/(1+|U|) {m:.3e} at {at:?} per component {:.2e} {:.2e} {:.2e} {:.2e}  swirl {:.1e}",
                d[0], d[1], d[2], d[3], d[4]
            );
            row.push(m);
        }
        println!("G-V1 wedge {deg} deg ({secs:.1} s): cd p {:.5} f {:.5} base {:.5}  y+ {:.3}", f.cd_pressure, f.cd_friction, f.cd_base, f.y_plus_max);
        worst.push(row);
        states.push((deg, as_2d(&s3)));
        grid = Some(s3.grid.clone());
    }
    // the wedge-angle error: each wedge against the smallest one (its own `O(dtheta^2)` error is in the reference, so the ratio of 2 and 1 degrees is `(4 - 1/4) / (1 - 1/4)` = 5)
    let g = grid.expect("at least one angle");
    let mut angle_err = Vec::new();
    if let Some((ref_deg, reference)) = states.last() {
        for (deg, s) in &states[..states.len() - 1] {
            let e: Vec<f64> = R_MINS.iter().map(|&r| array_difference(reference, s, &g, r)).collect();
            println!("G-V1 wedge {deg} deg against {ref_deg} deg: max |dU|/(1+|U|) {:.3e} (r >= 0), {:.3e} (r >= 0.1), {:.3e} (r >= 0.25)", e[0], e[1], e[2]);
            angle_err.push(e);
        }
    }
    if angle_err.len() >= 2 {
        for (q, r_min) in R_MINS.iter().enumerate() {
            println!("G-V1 wedge: r >= {r_min}: angle error ratio {:.2} (first / second angle)", angle_err[0][q] / angle_err[1][q]);
        }
    }
    assert!(angle_err[0][0] <= 5e-4, "wedge at {} degrees against the smallest wedge: {}", states[0].0, angle_err[0][0]);
}

/// G-V3 on the CPU: the G3 ogive-cylinder RANS case of `tests/viscous.rs` (Mach 0.5, ISA sea level, d = 0.1 m, `Resolution::Coarse`) on the wedge against
/// the 2D solver's CD 0.21300, friction 0.09003, base 0.12257 (`cp_base` -0.1226, y+ max 0.648, 4000 iterations): CD within 2 %, friction within 3 %,
/// base within 4 %, y+ max < 3.
#[test]
#[ignore]
fn g3_ogive_cylinder_rans_on_the_wedge() {
    let iters: usize = env("ITERS", 4000);
    let level: u32 = env("LEVEL", 0);
    let case = g3_case(0.5, level);
    let (c3, mut s3) = build3(&case, bowshock_core::engine::grid3::WEDGE_ANGLE, level);
    println!("G-V3: {} x {} cells, Re_c {:.4e}, 2D numerics {:?}", s3.grid.ni, s3.grid.nj, case.viscous.unwrap().re_c, case.numerics.scheme);
    let t0 = Instant::now();
    iterate3(&c3, &mut s3, iters, 250);
    let secs = t0.elapsed().as_secs_f64();
    let f = forces3(&c3, &s3);
    println!(
        "G-V3 3D {iters} iterations ({secs:.1} s, {:.1} ms each): CD {:.5}  friction {:.5}  base {:.5}  pressure {:.5}  cp_base {:.4}  y+ max {:.3}",
        1e3 * secs / iters as f64,
        f.total(),
        f.cd_friction,
        f.cd_base,
        f.cd_pressure,
        -f.cd_base,
        f.y_plus_max
    );
    let (r_cd, r_f, r_b) = (f.total() / 0.21300 - 1.0, f.cd_friction / 0.09003 - 1.0, f.cd_base / 0.12257 - 1.0);
    println!("G-V3 vs 2D reference: CD {:+.2} %  friction {:+.2} %  base {:+.2} %", 100.0 * r_cd, 100.0 * r_f, 100.0 * r_b);
    assert!(r_cd.abs() < 0.02 && r_f.abs() < 0.03 && r_b.abs() < 0.04 && f.y_plus_max < 3.0);
}

/// The G3 wedge against the 2D solver on the same grid and the same number of iterations (`ITERS`, 4000): the forces of both, the largest state differences and
/// where they are.
#[test]
#[ignore]
fn g3_wedge_against_the_2d_state() {
    let iters: usize = env("ITERS", 4000);
    let level: u32 = env("LEVEL", 0);
    let case = g3_case(env("MACH", 0.5), level);
    let (s2, r2, secs2) = run2(&case, iters, 1000);
    println!(
        "G-V3 2D {iters} iterations ({secs2:.1} s, {:.1} ms each): CD {:.5}  p {:.5}  friction {:.5}  base {:.5}  cp_base {:.4}  y+ {:.3}",
        1e3 * secs2 / iters as f64,
        r2.cd_total,
        r2.cd_nose + r2.cd_aft.iter().sum::<f64>(),
        r2.cd_friction,
        r2.cd_base,
        r2.cp_base,
        r2.y_plus_max
    );
    let (c3, mut s3) = build3(&case, bowshock_core::engine::grid3::WEDGE_ANGLE, level);
    let t3 = Instant::now();
    iterate3(&c3, &mut s3, iters, 1000);
    println!("G-V3 3D cost: {:.1} ms per iteration (the 2D solver {:.1})", 1e3 * t3.elapsed().as_secs_f64() / iters as f64, 1e3 * secs2 / iters as f64);
    let f = forces3(&c3, &s3);
    println!(
        "G-V3 3D {iters} iterations: CD {:.5}  p {:.5}  friction {:.5}  base {:.5}  y+ {:.3}",
        f.total(),
        f.cd_pressure,
        f.cd_friction,
        f.cd_base,
        f.y_plus_max
    );
    println!(
        "G-V3 3D vs 2D: CD {:+.2} %  friction {:+.2} %  base {:+.2} %",
        100.0 * (f.total() / r2.cd_total - 1.0),
        100.0 * (f.cd_friction / r2.cd_friction - 1.0),
        100.0 * (f.cd_base / r2.cd_base - 1.0)
    );
    print_worst(&s2, &s3, 8);
    let (d, at) = state_difference(&s2, &s3, 0.0);
    println!("  max per component {:.2e} {:.2e} {:.2e} {:.2e} at {at:?}", d[0], d[1], d[2], d[3]);
    i_wall_report(&s3);
}

/// The residual of the wedge operator on a converged 2D state against the 2D residual of the same state (both `dU/dt` per volume; the 2D one is ~0 after
/// convergence): where and how much the two operators differ, without the convergence of the two solvers in the way. `RE_C`, `ITERS`, `EULER` as above.
#[test]
#[ignore]
fn wedge_residual_of_the_2d_state() {
    let re_c: f64 = env("RE_C", 2000.0);
    let iters: usize = env("ITERS", 2500);
    let level: u32 = env("LEVEL", 0);
    let mut case = Case::viscous(0.5, g3_body(), level, Wake::Base, re_c, flight().1, 0.5);
    if let Some(v) = case.viscous.as_mut() {
        v.turbulence = Turbulence::Laminar;
    }
    if std::env::var_os("EULER").is_some() {
        case.viscous = None;
    }
    let (mut s2, _, _) = run2(&case, iters, 0);
    let (_, mut s3) = build3(&case, env("DEG", 2.0f64).to_radians(), level);
    let g = &s3.grid;
    let (sn, cs) = (0.5 * g.theta_span).sin_cos();
    for c in 0..g.ni * g.nj {
        if g.part[c] != 0 {
            continue;
        }
        let a = s2.u[c];
        s3.u[c] = [a[0], a[1], a[2] * sn, a[2] * cs, a[3]];
    }
    // the first-order phase of both solvers is over (the 2D solver ran `iters` iterations; the wedge one has to be told)
    s2.eval_residual(false);
    s3.iter = iters.max(s3.cfg.first_order_iters);
    s3.eval_residual();
    let (full2, full3) = (s2.res.clone(), s3.res.clone());
    report_residuals("whole residual", &full2, &full3, &s3.grid);
    if case.viscous.is_some() {
        // the viscous terms alone: the residual with them minus the one without
        s2.set_viscous(None);
        s3.set_viscous_with(None, RansOpts3::default());
        s2.eval_residual(false);
        s3.eval_residual();
        let v2: Vec<[f64; 4]> = full2.iter().zip(&s2.res).map(|(a, b)| [a[0] - b[0], a[1] - b[1], a[2] - b[2], a[3] - b[3]]).collect();
        let v3: Vec<Cons3> = full3.iter().zip(&s3.res).map(|(a, b)| [a[0] - b[0], a[1] - b[1], a[2] - b[2], a[3] - b[3], a[4] - b[4]]).collect();
        report_residuals("viscous part", &v2, &v3, &s3.grid);
    }
}

/// Where the residual of the wedge `r3` differs from the 2D one `r2` of the same state: rms and the worst cells.
fn report_residuals(label: &str, r2: &[[f64; 4]], r3: &[Cons3], g: &Grid3) {
    let (sn, cs) = (0.5 * g.theta_span).sin_cos();
    let mut rows: Vec<(f64, usize, [f64; 4], [f64; 4])> = Vec::new();
    for c in 0..g.ni * g.nj {
        if g.part[c] != 0 {
            continue;
        }
        let (a, b) = (r2[c], r3[c]);
        let b = [b[0], b[1], b[2] * sn + b[3] * cs, b[4]];
        let d = [b[0] - a[0], b[1] - a[1], b[2] - a[2], b[3] - a[3]];
        rows.push((d.iter().fold(0.0f64, |m, x| m.max(x.abs())), c, a, b));
    }
    rows.sort_by(|a, b| b.0.total_cmp(&a.0));
    let rms = |f: &dyn Fn(&(f64, usize, [f64; 4], [f64; 4])) -> f64| (rows.iter().map(|r| f(r).powi(2)).sum::<f64>() / rows.len() as f64).sqrt();
    println!("{label}: rms 2D {:.3e}  rms difference {:.3e}", rms(&|r| r.2.iter().fold(0.0f64, |m, x| m.max(x.abs()))), rms(&|r| r.0));
    for (m, c, a, b) in rows.iter().take(10) {
        println!(
            "  |res3 - res2| {m:.3e} at ({}, {}) x {:.3} r {:.4}  res2 {:?}  res3 {:?}",
            c % g.ni,
            c / g.ni,
            g.xc[*c][0],
            g.xc[*c][1].hypot(g.xc[*c][2]),
            a.map(|x| (x * 1e6).round() / 1e6),
            b.map(|x| (x * 1e6).round() / 1e6)
        );
        // the skew of the cell over its lower j face: the angle between the face normal and the line from the face centroid to the cell centroid (the wall normal and the
        // direction of the 2D solver's ghost, a point image in the face centre, apart)
        let f = &g.jface[(c / g.ni) * g.ni + c % g.ni];
        let v = [g.xc[*c][0] - f.c[0], g.xc[*c][1] - f.c[1], g.xc[*c][2] - f.c[2]];
        let skew = ((v[0] * f.n[0] + v[1] * f.n[1] + v[2] * f.n[2]).abs() / (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt()).acos().to_degrees();
        println!("      skew of the lower j face: {skew:.1} deg");
    }
}
