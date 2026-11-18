//! Diagnostic: single sector points of the finned example's drag table, through the facade path the app uses (Coarse, altitude 120 m, the parts' roughness,
//! the drag tolerance), with the history of the monitor. `DIAG_MACHS=1.3,1.1`; `DIAG_PLAIN=1` the plain factors at `cfl_max` 20 (no DDADI, no LHS boost);
//! `DIAG_CFLMAX=<x>`, `DIAG_BOOST=<x>`, `DIAG_OMEGA=<x>`, `DIAG_DRIFT=0` override single numerics.

use std::sync::{Arc, Mutex};
use std::time::Instant;

use super::*;
use crate::units::UnitSystem;

fn env_f(name: &str) -> Option<f64> {
    std::env::var(name).ok().map(|v| v.parse().unwrap())
}

fn hook(c: &mut solver::engine::case3::Case3) {
    if std::env::var_os("DIAG_PLAIN").is_some() {
        c.numerics.implicit.ddadi = None;
        c.numerics.implicit.lhs_dis_boost = 0.0;
        c.numerics.implicit.cfl_max = 20.0;
    }
    if let Some(x) = env_f("DIAG_CFLMAX") {
        c.numerics.implicit.cfl_max = x;
    }
    if let Some(x) = env_f("DIAG_BOOST") {
        c.numerics.implicit.lhs_dis_boost = x;
    }
    if let Some(x) = env_f("DIAG_OMEGA") {
        c.numerics.implicit.ddadi = Some(x);
    }
    if let Some(x) = env_f("DIAG_DRIFT") {
        c.numerics.drift_block = x;
    }
    if let Some(x) = env_f("DIAG_MAXIT") {
        c.numerics.max_iter = x as usize;
    }
    if let Some(from) = env_f("DIAG_SNAP") {
        let to = env_f("DIAG_SNAP_TO").unwrap_or(f64::INFINITY);
        let every = env_f("DIAG_SNAP_EVERY").unwrap_or(50.0) as usize;
        c.snapshots = Some(solver::engine::field3::Snapshots3::new(every, 1.0, move |f| {
            if (f.iteration as f64) < from || f.iteration as f64 > to {
                return;
            }
            let fs = f.freestream;
            let (mut rho_lo, mut p_lo, mut rho_hi, mut p_hi) = ((f64::INFINITY, 0usize), (f64::INFINITY, 0usize), (0.0f64, 0usize), (0.0f64, 0usize));
            let mut dp_far = (0.0f64, 0usize);
            for c in 0..f.cells() {
                if f.solid[c] {
                    continue;
                }
                let [rho, _, _, _, p] = f.prim[c].map(|v| v as f64);
                if rho < rho_lo.0 { rho_lo = (rho, c); }
                if p < p_lo.0 { p_lo = (p, c); }
                if rho > rho_hi.0 { rho_hi = (rho, c); }
                if p > p_hi.0 { p_hi = (p, c); }
                let (i, j) = (c % f.ni, (c / f.ni) % f.nj);
                if i == 0 || i == f.ni - 1 || j == f.nj - 1 {
                    let d = (p / fs[4] - 1.0).abs();
                    if d > dp_far.0 { dp_far = (d, c); }
                }
            }
            let at = |c: usize| {
                let (i, j, k) = (c % f.ni, (c / f.ni) % f.nj, c / (f.ni * f.nj));
                let x = f.centre(i, j, k);
                format!("({i},{j},{k}) x {:.2} r {:.3} th {:.1}", x[0], x[1].hypot(x[2]), x[2].atan2(x[1]).to_degrees())
            };
            println!(
                "SNAP it {} rho_min {:.3e} {} | p_min {:.3e} {} | rho_max {:.3} {} | p_max {:.3} {} | far dp {:.3e} {}",
                f.iteration, rho_lo.0, at(rho_lo.1), p_lo.0, at(p_lo.1), rho_hi.0, at(rho_hi.1), p_hi.0, at(p_hi.1), dp_far.0, at(dp_far.1)
            );
        }));
    }
    println!("CASE mach {} implicit {:?} numerics {:?}", c.mach, c.numerics.implicit, c.numerics);
}

#[test]
#[ignore = "GPU: single sector points of the finned example through the facade"]
fn diag13_sector_points() {
    let (rocket, vehicle) = example();
    let plan = plan(&rocket, &vehicle, UnitSystem::default()).unwrap();
    assert!(plan.fins.is_some());
    let machs: Vec<f64> = std::env::var("DIAG_MACHS").map(|v| v.split(',').map(|m| m.trim().parse().unwrap()).collect()).unwrap_or_else(|_| vec![1.3]);
    for mach in machs {
        let mut a = solver::Analysis::new(&plan.body)
            .unwrap()
            .engine(solver::Engine::Solver3)
            .resolution(solver::Resolution::Coarse)
            .backend(solver::Backend::Gpu)
            .keep_fields(true)
            .viscous(solver::FlightConditions { atmosphere: solver::Atmosphere::Isa { altitude_m: 120.0 } })
            .roughness(plan.body_roughness)
            .drag_tolerance(DRAG_TOLERANCE)
            .case3_hook(hook);
        if let Some(f) = &plan.fins {
            a = a.fins(f.clone()).fin_outline(plan.outline.clone());
        }
        let hist = Arc::new(Mutex::new(Vec::new()));
        let sink = hist.clone();
        a = a.on_progress(move |p| {
            sink.lock().unwrap().push((p.iteration, p.residual_drop_orders, p.monitor, p.spread));
            std::ops::ControlFlow::Continue(())
        });
        let t = Instant::now();
        let polar = a.run(&[solver::Condition::new(mach, 0.0)]).expect("solves");
        let wall = t.elapsed().as_secs_f64();
        let p = &polar.points[0];
        let d = &p.detail;
        println!(
            "POINT M {mach} wall {wall:.1} s quality {:?} stop {:?} its {} farfield_dp {:.4e} cp_base {:.4} rollbacks {:?} cfl_limit {:?} cd_total(solver) {:.5} parts: forebody {:.5} friction {:?} base {:.5} fins {:?} warnings {:?}",
            p.quality,
            d.stop_reason,
            d.iterations,
            d.farfield_dp,
            d.cp_base,
            d.rollbacks,
            d.cfl_limit,
            p.drag.total,
            p.drag.forebody_pressure,
            p.drag.friction,
            p.drag.base,
            p.drag.fins,
            p.warnings
        );
        for (it, drop, mon, spread) in hist.lock().unwrap().iter().filter(|h| h.0 % 100 == 0) {
            println!("HIST M {mach} it {it} drop {drop:.2} CA {mon:.6} spread {spread:?}");
        }
    }
}
