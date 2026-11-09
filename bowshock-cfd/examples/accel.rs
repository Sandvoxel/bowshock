//! Convergence-scheme measurements: one case, one scheme, residual history and timing.
//!
//! `cargo run --release --example accel -- <body> <level> <mach> <scheme> [key=value ...]`
//! scheme = `explicit` | `irs` | `mg` | `imp` (line-implicit; `cfl0`, `cfl`=max, `growth`, `min`=1 weaker-direction dt, `ibc`) | `pt` (point-implicit); keys: `cfl`, `limiter` (minmod|vanalbada), `lm` (0|1), `cstar` (IRS per-direction CFL limit, 0 = off), `levels`, `pre`, `post`,
//! `gamma`, `coarsest`, `fo` (first-order iterations), `max`, `drop` (orders), `every` (print interval), `cdtol`.
use bowshock_cfd::flux::Limiter;
use bowshock_cfd::implicit::{DtMode, ImplicitParams};
use bowshock_cfd::solver::{Irs, MgParams, Scheme};
use bowshock_cfd::transonic::{named_body, run_transonic, TransonicCase};
use std::time::Instant;

fn main() {
    let a: Vec<String> = std::env::args().skip(1).collect();
    let (body, _) = named_body(&a[0]).expect("body");
    let level: u32 = a[1].parse().unwrap();
    let mach: f64 = a[2].parse().unwrap();
    let kind = a[3].as_str();
    let kv = |k: &str| a.iter().skip(4).find_map(|s| s.strip_prefix(&format!("{k}=")).map(String::from));
    let num = |k: &str, d: f64| kv(k).map_or(d, |v| v.parse().unwrap());
    let irs = Irs { cfl_star: num("cstar", 0.0) };
    let mut case = TransonicCase::with_body(mach, level, body, 6.0);
    case.scheme = match kind {
        "explicit" => Scheme::Explicit,
        "irs" => Scheme::Smoothed(irs),
        "mg" => Scheme::Multigrid(MgParams {
            levels: num("levels", 6.0) as usize,
            pre: num("pre", 1.0) as usize,
            post: num("post", 1.0) as usize,
            gamma: num("gamma", 1.0) as usize,
            coarsest: num("coarsest", 4.0) as usize,
            irs,
        }),
        "imp" | "pt" => Scheme::Implicit(ImplicitParams {
            cfl0: num("cfl0", 1.0),
            cfl_max: num("cfl", 30.0),
            growth: num("growth", 1.02),
            dt_mode: if num("min", 0.0) > 0.0 { DtMode::MinDirection } else { DtMode::Sum },
            lines: kind == "imp",
            implicit_bc: num("ibc", 1.0) > 0.0,
            sub_iters: num("sub", 2.0) as usize,
            roe: num("roe", 1.0) > 0.0,
            eps_fix: num("epsfix", 0.1),
        }),
        _ => panic!("scheme"),
    };
    if let Some(d) = kv("dom") {
        let d: f64 = d.parse().unwrap();
        let lb = body.nose_length() + case.grid.cyl_length;
        case.grid.x_up = d * lb;
        case.grid.r_far = d * lb;
        case.grid.x_down = 0.5 * d * lb;
    }
    case.grid.dx_tip = num("dxtip", case.grid.dx_tip);
    case.grid.dr_wall = num("drwall", case.grid.dr_wall);
    if kv("out").as_deref() == Some("pressure") {
        case.outflow = bowshock_cfd::solver::Bc::PressureOutflow;
    }
    case.cfl = num("cfl", 0.5);
    case.low_mach = num("lm", 0.0) > 0.0;
    case.limiter = match kv("limiter").as_deref() {
        Some("vanalbada") => Limiter::VanAlbada,
        Some("first") => Limiter::FirstOrder,
        _ => Limiter::Minmod,
    };
    case.first_order_iters = num("fo", 300.0) as usize;
    case.max_iter = num("max", 30000.0) as usize;
    case.drop_orders = num("drop", 6.0);
    case.cd_tol = num("cdtol", 0.0);
    let t0 = Instant::now();
    let r = run_transonic(&case);
    let dt = t0.elapsed().as_secs_f64();
    let every = num("every", 500.0) as usize;
    for (i, x) in r.residual.iter().enumerate() {
        if (i + 1) % every == 0 {
            println!("  it {:>6}  res {:.3e}", i + 1, x);
        }
    }
    println!(
        "{kind} L{level} M={mach} cfl={} lim={:?} lm={}: {} iters, {dt:.1} s ({:.2} ms/it), stop {}, drop {:.2} orders, peak {:.2e} final {:.2e}, CD {:.5}, Cp_min {:.4}, cd_range {:.1e}",
        case.cfl, case.limiter, case.low_mach, r.iters, 1e3 * dt / r.iters as f64, r.stop_reason, r.drop_orders, r.residual_peak, r.residual.last().unwrap(), r.cd_nose, r.cp_min, r.cd_range
    );
}
