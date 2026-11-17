//! `half_probe`: the example rocket's half domain at incidence on the GPU, residual history, to find where a cold run goes non-finite.
//!
//! ```text
//! cargo run --release -p bowshock-validate --example half_probe -- [--mach 3] [--alpha 6] [--solve diag|block|cpu] [--precision f32|df32] [--cfl 10] [--iters 300] [--every 10]
//!                                                                  [--first-order N] [--fins 3|4]
//! ```
//!
//! The case of the repro (`solver3d` / `app-integrate`): ogive 3 d nose + 12 d tube, `--fins` square fins (3: root 0.12 / 0.0762 d, tip 0.05 / 0.0762 d, span 1 d, sweep
//! 0.06 / 0.0762 d, thickness 0.00318 / 0.0762 d; 4: the example rocket's 1.575 / 0.656 / 0.787 d, t 0.0417 d), flush trailing edge, `Case3::base_half(mach, alpha, ..,
//! Tier::Euler, 0)` with the fins between the pitch plane and each other. One line per `--every` iterations: residual, lowest pressure and its cell (device state download).

use bowshock_cfd::gpu::Precision;
use bowshock_cfd::implicit_params::LineSolve3;
use bowshock_core::engine::body_grid::{BaseBody, Body, Nose};
use bowshock_core::engine::case3::{Case3, Tier};
use bowshock_core::engine::gpu3::GpuRunner3;
use bowshock_core::engine::grid3::{Domain3, FinPhase, Fins3};
use bowshock_core::FinProfile;

fn main() -> Result<(), String> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let value = |name: &str| args.iter().position(|a| a == name).and_then(|i| args.get(i + 1)).cloned();
    let num = |name: &str, d: f64| value(name).map_or(Ok(d), |v| v.parse::<f64>().map_err(|e| format!("{name}: {e}")));
    let (mach, alpha) = (num("--mach", 3.0)?, num("--alpha", 6.0)?);
    let iters = num("--iters", 300.0)? as usize;
    let every = num("--every", 10.0)? as usize;
    let nfins = num("--fins", 3.0)? as u32;
    let d = 0.0762;
    let nose = Body { nose: Nose::TangentOgive { fineness: 0.2286 / d } };
    let body = BaseBody::cylinder(nose, 0.9134 / d);
    let fins = if nfins == 3 {
        Fins3 {
            count: 3,
            root_chord: 0.12 / d,
            tip_chord: 0.05 / d,
            span: 1.0,
            sweep: 0.06 / d,
            thickness: 0.00318 / d,
            te_to_base: 0.0,
            profile: FinProfile::Square,
            planform: None,
        }
    } else {
        Fins3 {
            count: 4,
            root_chord: 1.575,
            tip_chord: 0.656,
            span: 1.0,
            sweep: 0.787,
            thickness: 0.0417,
            te_to_base: 0.0,
            profile: FinProfile::Square,
            planform: None,
        }
    };
    let mut case = Case3::base_half(mach, alpha.to_radians(), body, Some(fins), Tier::Euler, 0);
    case.grid.domain = Domain3::Half { phase: FinPhase::Between };
    if let Some(c) = value("--cfl") {
        case.numerics.implicit.cfl_max = c.parse().map_err(|e| format!("--cfl: {e}"))?;
    }
    if let Some(n) = value("--first-order") {
        case.numerics.first_order_iters = n.parse().map_err(|e| format!("--first-order: {e}"))?;
    }
    let solve = value("--solve").unwrap_or_else(|| "diag".into());
    case.numerics.line_solve = match solve.as_str() {
        "block" | "cpu" => Some(LineSolve3::Block),
        _ => Some(LineSolve3::DiagonalCorrected), // `diag` (GPU), `cpudiag` (CPU f64)
    };
    let prec = if value("--precision").as_deref() == Some("df32") { Precision::Extended } else { Precision::F32 };
    if solve == "cpu" || solve == "cpudiag" {
        let mut s = case.build_solver().map_err(|e| e.to_string())?;
        println!("CPU {} x {} x {}", s.grid.ni, s.grid.nj, s.grid.nk);
        for it in 0..iters {
            let r = s.step();
            if (it + 1) % every == 0 {
                let (pmin, at) =
                    s.u.iter()
                        .enumerate()
                        .map(|(c, q)| (bowshock_cfd::gas3::cons_to_prim3(q)[4], c))
                        .fold((f64::INFINITY, 0), |a, b| if b.0 < a.0 { b } else { a });
                println!("it {:5}  residual {r:.4e}  pmin {pmin:.4} at cell {at}", it + 1);
            }
            if !r.is_finite() {
                break;
            }
        }
        return Ok(());
    }
    let mut run = GpuRunner3::new(&case, prec).map_err(|e| e.to_string())?;
    let g = &run.host.grid;
    println!("{:?} {solve} {} x {} x {}, cfl_max {}", prec, g.ni, g.nj, g.nk, case.numerics.implicit.cfl_max);
    let cfg = bowshock_cfd::solver3::Config3 {
        freestream: case.freestream(),
        cfl: 0.5,
        limiter: case.numerics.limiter,
        first_order_iters: case.numerics.first_order_iters,
        low_mach: case.numerics.low_mach,
        scheme: bowshock_cfd::solver3::Scheme3::Implicit(case.numerics.implicit_for(true)),
    };
    run.gpu.set_config(&cfg).map_err(|e| e.to_string())?;
    let mut done = 0;
    while done < iters {
        let n = every.min(iters - done);
        let h = run.gpu.run(n).map_err(|e| e.to_string())?;
        done += n;
        let u = run.gpu.state_f64().map_err(|e| e.to_string())?;
        let (pmin, at) =
            u.iter().enumerate().map(|(c, q)| (bowshock_cfd::gas3::cons_to_prim3(q)[4], c)).fold((f64::INFINITY, 0), |a, b| if b.0 < a.0 { b } else { a });
        let g = &run.host.grid;
        let (i, j, k) = (at % g.ni, (at / g.ni) % g.nj, at / (g.ni * g.nj));
        println!("it {done:5}  residual {:.4e}  pmin {pmin:.4} at ({i}, {j}, {k}) x {:.3}", h.last().copied().unwrap_or(f64::NAN), g.xc[at][0]);
        if !h.last().is_some_and(|r| r.is_finite()) {
            break;
        }
    }
    Ok(())
}
