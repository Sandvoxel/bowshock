//! Diagnostic of the slow points of the flight-validation rockets (DESIGN3D item 56): one drag point through the facade path the app uses, with the grid's quality
//! numbers and the state at every snapshot. `DIAG_ROCKET=rabia-short-fin-can` (a flight fixture; `example` the app's finned example), `DIAG_MACHS=0.8`,
//! `DIAG_RES=draft|coarse`, `DIAG_ALPHA=2` (the app's stability node: half domain at that angle), `DIAG_COLLAPSE=1` (the cells of the final state under the positivity guard, with the part they touch), `DIAG_OWNGRID=1`, `DIAG_CPU=1`, `DIAG_GRID=1` (grid statistics before the run), `DIAG_MAXIT`, `DIAG_SNAP=<from>` (state every 50 iterations from there);
//! the fixture probes of `flights.rs` (`FLIGHTVAL_BLUNT_IN`, `FLIGHTVAL_TIP_IN`, ...) apply.

use std::sync::{Arc, Mutex};
use std::time::Instant;

use super::*;
use crate::units::UnitSystem;

/// The grid of the last case built by the hook under `DIAG_GRID`/`DIAG_COLLAPSE`: its part table, to name what a collapsed cell sits against.
static GRID_PARTS: Mutex<Option<(usize, usize, usize, Vec<String>)>> = Mutex::new(None);

fn env_f(name: &str) -> Option<f64> {
    std::env::var(name).ok().map(|v| v.parse().unwrap())
}

fn hook(c: &mut solver::engine::case3::Case3) {
    if let Some(x) = env_f("DIAG_MAXIT") {
        c.numerics.max_iter = x as usize;
    }
    if std::env::var_os("DIAG_PLAIN").is_some() {
        c.numerics.implicit.ddadi = None;
        c.numerics.implicit.lhs_dis_boost = 0.0;
        c.numerics.implicit.cfl_max = 20.0;
    }
    if let Some(x) = env_f("DIAG_CFLMAX") {
        c.numerics.implicit.cfl_max = x;
    }
    if let Some(x) = env_f("DIAG_FIRST") {
        c.numerics.first_order_iters = x as usize;
    }
    if std::env::var_os("DIAG_LE_ROWS").is_some() {
        c.grid.fin_le_rows = true;
    }
    if let Some(x) = env_f("DIAG_ACROSS") {
        c.grid.fin_cells_across = x as usize;
    }
    if let Some(x) = env_f("DIAG_CHORD") {
        c.grid.fin_chord_cells = Some(x as usize);
    }
    if env_f("DIAG_LOWMACH") == Some(0.0) {
        c.numerics.low_mach = false;
    }
    if std::env::var_os("DIAG_GRID").is_some() || std::env::var_os("DIAG_COLLAPSE").is_some() {
        let g = solver::engine::grid3::body_grid3(&c.grid).expect("the grid builds");
        let mut parts = Vec::with_capacity(g.ni * g.nj * g.nk);
        for k in 0..g.nk {
            for j in 0..g.nj {
                for i in 0..g.ni {
                    parts.push(
                        g.part_of(i, j, k).map_or_else(|| if g.part[g.cell(i, j, k)] == 0 { "fluid".into() } else { "dead".into() }, |p| format!("{p:?}")),
                    );
                }
            }
        }
        *GRID_PARTS.lock().unwrap() = Some((g.ni, g.nj, g.nk, parts));
    }
    if std::env::var_os("DIAG_GRID").is_some() {
        let g = solver::engine::grid3::body_grid3(&c.grid).expect("the grid builds");
        let label = std::env::var("DIAG_ROCKET").unwrap_or_default();
        solver::engine::grid3::print_quality(&label, &g);
        solver::engine::grid3::print_le_quality(&label, &g);
        if let Ok(v) = std::env::var("DIAG_CELL") {
            let v: Vec<usize> = v.split(',').map(|x| x.parse().unwrap()).collect();
            let (ci, cj, ck, span) = (v[0], v[1], v[2], v[3]);
            let node = |i: usize, j: usize, k: usize| {
                let n = (k * (g.nj + 1) + j) * (g.ni + 1) + i;
                (g.x[n], g.y[n].hypot(g.z[n]), g.z[n].atan2(g.y[n]).to_degrees())
            };
            let show = |i: usize, j: usize, k: usize| {
                let (a, b) = (node(i, j, k), node(i + 1, j + 1, k + 1));
                let c = node(i + 1, j, k);
                let d = node(i, j + 1, k);
                format!(
                    "({i},{j},{k}) x {:.4}..{:.4} (top row {:.4}..{:.4}) r {:.4}..{:.4} th {:.3}..{:.3} solid {} vol {:.3e}",
                    a.0,
                    c.0,
                    d.0,
                    node(i + 1, j + 1, k).0,
                    a.1,
                    d.1,
                    a.2,
                    b.2,
                    g.part[g.cell(i, j, k)],
                    g.vol[g.cell(i, j, k)]
                )
            };
            for i in ci.saturating_sub(span)..=(ci + span).min(g.ni - 1) {
                println!("CELL i-scan {}", show(i, cj, ck));
            }
            for k in 0..=(ck + span).min(g.nk - 1) {
                println!("CELL k-scan {}", show(ci, cj, k));
            }
        }
        if let (Some(j), Some(k)) = (env_f("DIAG_ROW_J"), env_f("DIAG_ROW_K")) {
            let (j, k) = (j as usize, k as usize);
            let xs: Vec<String> = (0..=g.ni).map(|i| format!("{:.4}", g.x[(k * (g.nj + 1) + j) * (g.ni + 1) + i])).collect();
            println!("ROW j {j} k {k} x nodes: {}", xs.join(" "));
        }
    }
    if let Some(from) = env_f("DIAG_SNAP") {
        let every = env_f("DIAG_SNAP_EVERY").unwrap_or(50.0) as usize;
        let prev: Arc<Mutex<Vec<f32>>> = Arc::new(Mutex::new(Vec::new()));
        c.snapshots = Some(solver::engine::field3::Snapshots3::new(every, 1.0, move |f| {
            if (f.iteration as f64) < from {
                return;
            }
            let fs = f.freestream;
            let (mut rho_lo, mut p_lo, mut rho_hi, mut p_hi) = ((f64::INFINITY, 0usize), (f64::INFINITY, 0usize), (0.0f64, 0usize), (0.0f64, 0usize));
            let mut d_rho = (0.0f64, 0usize);
            let mut d_p = (0.0f64, 0usize);
            let mut prev = prev.lock().unwrap();
            let have = prev.len() == 2 * f.cells();
            let mut now = vec![0f32; 2 * f.cells()];
            for c in 0..f.cells() {
                if f.solid[c] {
                    continue;
                }
                let [rho, _, _, _, p] = f.prim[c].map(|v| v as f64);
                now[2 * c] = rho as f32;
                now[2 * c + 1] = p as f32;
                if rho < rho_lo.0 {
                    rho_lo = (rho, c);
                }
                if p < p_lo.0 {
                    p_lo = (p, c);
                }
                if rho > rho_hi.0 {
                    rho_hi = (rho, c);
                }
                if p > p_hi.0 {
                    p_hi = (p, c);
                }
                if have {
                    let dr = (rho - prev[2 * c] as f64).abs() / fs[0];
                    let dp = (p - prev[2 * c + 1] as f64).abs() / fs[4];
                    if dr > d_rho.0 {
                        d_rho = (dr, c);
                    }
                    if dp > d_p.0 {
                        d_p = (dp, c);
                    }
                }
            }
            *prev = now;
            let at = |c: usize| {
                let (i, j, k) = (c % f.ni, (c / f.ni) % f.nj, c / (f.ni * f.nj));
                let x = f.centre(i, j, k);
                format!("({i},{j},{k}) x {:.3} r {:.3} th {:.0}", x[0], x[1].hypot(x[2]), x[2].atan2(x[1]).to_degrees())
            };
            println!(
                "SNAP it {} rho_min {:.3} {} | p_min {:.3} {} | rho_max {:.3} {} | p_max {:.3} {} | d_rho {:.2e} {} | d_p {:.2e} {}",
                f.iteration,
                rho_lo.0 / fs[0],
                at(rho_lo.1),
                p_lo.0 / fs[4],
                at(p_lo.1),
                rho_hi.0 / fs[0],
                at(rho_hi.1),
                p_hi.0 / fs[4],
                at(p_hi.1),
                d_rho.0,
                at(d_rho.1),
                d_p.0,
                at(d_p.1)
            );
        }));
    }
}

#[test]
#[ignore = "GPU: one drag point of a flight-validation rocket through the facade, with grid statistics and state history"]
fn diag56_point() {
    let rocket = std::env::var("DIAG_ROCKET").unwrap_or_else(|_| "rabia-short-fin-can".into());
    let (plan, altitude) = if rocket == "example" {
        let (r, v) = example();
        (plan(&r, &v, UnitSystem::default()).unwrap(), 120.0)
    } else {
        flights::plan_and_altitude(&rocket)
    };
    assert!(plan.fins.is_some(), "{:?}", plan.notes);
    println!("PLAN fins {:?} outline {:?} body {:?}", plan.fins, plan.outline, plan.body);
    let machs: Vec<f64> = std::env::var("DIAG_MACHS").map(|v| v.split(',').map(|m| m.trim().parse().unwrap()).collect()).unwrap_or_else(|_| vec![0.8]);
    for mach in machs {
        // `DIAG_ALPHA=2` solves the stability node of the app (the half domain at that angle: Coarse, no drag stop rule, grid sized for its own Mach number)
        let alpha = env_f("DIAG_ALPHA").unwrap_or(0.0);
        let res = match std::env::var("DIAG_RES").as_deref() {
            _ if alpha != 0.0 => solver::Resolution::Coarse,
            Ok("draft") => solver::Resolution::Draft,
            _ => solver::Resolution::Coarse,
        };
        let backend = if std::env::var_os("DIAG_CPU").is_some() { solver::Backend::Cpu } else { solver::Backend::Gpu };
        let mut a = solver::Analysis::new(&plan.body)
            .unwrap()
            .engine(solver::Engine::Solver3)
            .resolution(res)
            .backend(backend)
            .keep_fields(true)
            .viscous(solver::FlightConditions { atmosphere: solver::Atmosphere::Isa { altitude_m: altitude } })
            .roughness(plan.body_roughness)
            .case3_hook(hook);
        if alpha == 0.0 {
            a = a.drag_tolerance(DRAG_TOLERANCE);
        }
        // the app's shared grid: a drag node's grid is sized for max(its Mach, 1), a stability node's for its own (`DIAG_OWNGRID=1`: a drag node's for its own Mach number too)
        if alpha != 0.0 {
            a = a.grid_mach(mach.min(3.0));
        } else if std::env::var_os("DIAG_OWNGRID").is_none() {
            a = a.grid_mach(mach.clamp(1.0, 3.0));
        }
        if let Some(f) = plan.fins.as_ref().filter(|_| std::env::var_os("DIAG_NOFINS").is_none()) {
            a = a.fins(f.clone()).fin_outline(plan.outline.clone());
        }
        let hist = Arc::new(Mutex::new(Vec::new()));
        let sink = hist.clone();
        a = a.on_progress(move |p| {
            sink.lock().unwrap().push((p.iteration, p.residual_drop_orders, p.monitor));
            std::ops::ControlFlow::Continue(())
        });
        let t = Instant::now();
        let polar = a.run(&[solver::Condition::new(mach, alpha)]).expect("solves");
        let wall = t.elapsed().as_secs_f64();
        let p = &polar.points[0];
        let d = &p.detail;
        println!(
            "POINT {rocket} M {mach} alpha {alpha} wall {wall:.1} s quality {:?} stop {:?} its {} cd {:.5} cp_base {:.4} cn {:?}",
            p.quality,
            d.stop_reason,
            d.iterations,
            p.drag.total,
            d.cp_base,
            p.forces.map(|f| f.cn)
        );
        for r in &d.rollbacks {
            println!("ROLLBACK {r:?}");
        }
        if let Some(f) = &p.field3 {
            report_collapsed(&rocket, mach, f);
        }
        for (it, drop, mon) in hist.lock().unwrap().iter().filter(|h| h.0 % 250 == 0) {
            println!("HIST it {it} drop {drop:.2} CA {mon:.6}");
        }
    }
}

/// The cells of the final state whose density or pressure is under 1.5e-2 of the free stream's (the `clipped_state` guard of the facade): their count, the lowest few
/// with the cell index, position (x, r, theta in degrees), the ratios, and the solid parts they touch (`DIAG_GRID` or `DIAG_COLLAPSE` names the parts).
fn report_collapsed(label: &str, mach: f64, f: &solver::Field3) {
    let fs = f.freestream;
    let mut bad: Vec<(f64, usize)> = (0..f.cells())
        .filter(|&c| !f.solid[c])
        .map(|c| (((f.prim[c][0] as f64) / fs[0]).min((f.prim[c][4] as f64) / fs[4]), c))
        .filter(|(m, _)| *m < 1.5e-2)
        .collect();
    bad.sort_by(|a, b| a.0.total_cmp(&b.0));
    println!(
        "COLLAPSED {label} M {mach}: {} cells under 1.5e-2 of the free stream (grid {}x{}x{}, fins at theta {:?})",
        bad.len(),
        f.ni,
        f.nj,
        f.nk,
        f.fin_theta
    );
    let parts = GRID_PARTS.lock().unwrap();
    for (m, c) in bad.iter().take(12) {
        let (i, j, k) = (c % f.ni, (c / f.ni) % f.nj, c / (f.ni * f.nj));
        let x = f.centre(i, j, k);
        let mut touching = Vec::new();
        for (di, dj, dk) in [(-1i64, 0i64, 0i64), (1, 0, 0), (0, -1, 0), (0, 1, 0), (0, 0, -1), (0, 0, 1)] {
            let (a, b, d) = (i as i64 + di, j as i64 + dj, k as i64 + dk);
            if a < 0 || b < 0 || d < 0 || a >= f.ni as i64 || b >= f.nj as i64 || d >= f.nk as i64 {
                continue;
            }
            let n = f.cell_index(a as usize, b as usize, d as usize);
            if f.solid[n] {
                let name = parts.as_ref().map_or("solid".to_string(), |(_, _, _, p)| p[n].clone());
                touching.push(format!("{name}@{di:+},{dj:+},{dk:+}"));
            }
        }
        println!(
            "COLLAPSED cell ({i},{j},{k}) x {:.4} r {:.4} th {:.2} rho/rho_inf {:.4} p/p_inf {:.4} touches {touching:?}",
            x[0],
            x[1].hypot(x[2]),
            x[2].atan2(x[1]).to_degrees(),
            f.prim[*c][0] as f64 / fs[0],
            f.prim[*c][4] as f64 / fs[4]
        );
        let _ = m;
    }
}
