//! `wedge_rans`: one RANS point of the example table on the single-cell wedge, on the GPU or the CPU, with the history of the total drag.
//!
//! ```text
//! cargo run --release -p bowshock-validate --example wedge_rans -- --mach 0.5 [--precision f32|df32|cpu] [--iters N] [--every 250]
//! ```
//!
//! The case is the facade's: the example body (`bowshock-gpu-cases::wedge3::engine_case`, coarse resolution, ISA 120 m) at `mach`, `Case3::wedge_of` (the 2D case's
//! numerics, block lines, the viscous parameters), `GpuRunner3` (`--precision f32|df32`) or the CPU `Solver3` (`cpu`). Without `--iters` the run goes to the stop rule of the
//! case; with it, exactly that many iterations (the monitor is sampled every `--every`: the total CD of the 3D forces and, on the GPU, the density residual).
//! One line per sample, then the final numbers: CD, friction, base, y+ max, base Cp, iterations, stop reason, wall time.

use bowshock_cfd::gpu::Precision;
use bowshock_core::engine::case3::Case3;
use bowshock_core::engine::gpu3::GpuRunner3;
use bowshock_core::engine::run3::{drag_parts_on, run3_on};
use bowshock_core::{AftSpec, BodySpec, NoseShape, NoseSpec, Resolution};
use bowshock_gpu_cases::wedge3::engine_case;
use std::time::Instant;

fn example_body() -> BodySpec {
    BodySpec { nose: NoseSpec { shape: NoseShape::TangentOgive, length: 0.2286, diameter: 0.0762 }, aft: vec![AftSpec::Tube { length: 0.9134 }] }
}

fn main() -> Result<(), String> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let value = |name: &str| args.iter().position(|a| a == name).and_then(|i| args.get(i + 1)).cloned();
    let mach: f64 = value("--mach").ok_or("--mach <M>")?.parse().map_err(|e| format!("--mach: {e}"))?;
    let which = value("--precision").unwrap_or_else(|| "df32".into());
    let iters: Option<usize> = value("--iters").map(|v| v.parse().map_err(|e| format!("--iters: {e}"))).transpose()?;
    let every: usize = value("--every").map_or(Ok(250), |v| v.parse().map_err(|e| format!("--every: {e}")))?;
    let c2 = engine_case(&example_body(), mach, true, Resolution::Coarse);
    let mut case = Case3::wedge_of(&c2, 0);
    if let Some(s) = value("--sub") {
        case.numerics.implicit.sub_iters = s.parse().map_err(|e| format!("--sub: {e}"))?;
    }
    if let Some(c) = value("--cfl") {
        case.numerics.implicit.cfl_max = c.parse().map_err(|e| format!("--cfl: {e}"))?;
    }
    if let Some(n) = iters {
        case.numerics.max_iter = n;
        // no stop rule but the count
        case.numerics.stationary_min_iters = usize::MAX / 2;
        case.numerics.warm_min_iters = usize::MAX / 2;
    }
    println!("M {mach}: {which}");
    let t = Instant::now();
    if which == "cpu" {
        let mut s = case.build_solver().map_err(|e| e.to_string())?;
        let r = run3_on(&case, &mut s, false);
        let d = drag_parts_on(&case, &s);
        println!(
            "CPU: {} iterations, stop {:?}, {:.2} orders, CD {:.5} (pressure {:.5} friction {:.5} base {:.5}), y+ max {:.3}, cp_base {:.4}, {:.1} s",
            r.iterations,
            r.stop_reason,
            r.drop_orders,
            d.total,
            d.pressure_total(),
            d.friction.total,
            d.base,
            r.y_plus_max,
            r.cp_base,
            t.elapsed().as_secs_f64()
        );
        return Ok(());
    }
    let prec = if which == "f32" { Precision::F32 } else { Precision::Extended };
    let mut run = GpuRunner3::new(&case, prec).map_err(|e| e.to_string())?;
    println!("set-up {:.1} s, {} cells", t.elapsed().as_secs_f64(), run.host.grid.ni * run.host.grid.nj * run.host.grid.nk);
    let t = Instant::now();
    if iters.is_some() {
        let mut done = 0usize;
        while done < iters.unwrap_or(0) {
            let n = every.min(iters.unwrap_or(0) - done);
            let h = run.gpu.run(n).map_err(|e| e.to_string())?;
            done += n;
            let d = run.drag_parts(&case).map_err(|e| e.to_string())?;
            println!(
                "it {done:6}  residual {:.3e}  CD {:.5} (friction {:.5}, base {:.5})",
                h.last().copied().unwrap_or(f64::NAN),
                d.total,
                d.friction.total,
                d.base
            );
            if !h.last().is_some_and(|r| r.is_finite()) {
                println!("non-finite residual");
                break;
            }
        }
        println!("{} iterations, {:.1} s, {:.2} ms per iteration", done, t.elapsed().as_secs_f64(), 1e3 * t.elapsed().as_secs_f64() / done.max(1) as f64);
        return Ok(());
    }
    let r = run.run_on(&case, false).map_err(|e| e.to_string())?;
    let d = run.drag_parts(&case).map_err(|e| e.to_string())?;
    let wall = t.elapsed().as_secs_f64();
    println!(
        "GPU {which}: {} iterations, stop {:?}, {:.2} orders, CD {:.5} (pressure {:.5} friction {:.5} base {:.5}), y+ max {:.3}, cp_base {:.4}, {wall:.1} s ({:.2} ms per iteration)",
        r.iterations,
        r.stop_reason,
        r.drop_orders,
        d.total,
        d.pressure_total(),
        d.friction.total,
        d.base,
        r.y_plus_max,
        r.cp_base,
        1e3 * wall / r.iterations.max(1) as f64
    );
    for (it, cd) in r.cn_history.iter().step_by(((r.cn_history.len() / 20).max(1)) as usize) {
        println!("  monitor it {it}: {cd:.5}");
    }
    Ok(())
}
