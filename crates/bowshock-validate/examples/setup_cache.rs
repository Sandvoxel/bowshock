//! `setup_cache`: the per-point set-up of a GPU runner when one call follows another (a Mach table: one `Analysis::run` per node), on the tiny ANF sector grid.
//!
//! ```text
//! cargo run --release -p bowshock-validate --example setup_cache -- [--calls 4] [--iters 100] [--precision f32|df32]
//! ```
//!
//! One line per call: the host grid + solver build of `Case3::build_solver` (what `GpuRunner3::new` did) next to `Case3::build_solver_cached` (what it does now:
//! the grid of `cache3::shared()`), the runner's build / context / upload seconds, and the wall time of `new` + `iters` iterations (the first call compiles the kernels).

use bowshock_cfd::gpu::Precision;
use bowshock_core::engine::case3::Case3;
use bowshock_core::engine::gpu3::GpuRunner3;
use std::time::Instant;

fn main() -> Result<(), String> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let value = |name: &str| args.iter().position(|a| a == name).and_then(|i| args.get(i + 1)).cloned();
    let calls: usize = value("--calls").map_or(Ok(4), |v| v.parse().map_err(|e| format!("--calls: {e}")))?;
    let iters: usize = value("--iters").map_or(Ok(100), |v| v.parse().map_err(|e| format!("--iters: {e}")))?;
    let prec = if value("--precision").as_deref() == Some("df32") { Precision::Extended } else { Precision::F32 };
    println!("| call | Mach | build_solver s | build_solver_cached s | runner build s | ctx s | upload s | new + {iters} iterations s |\n|---|---|---|---|---|---|---|---|");
    for call in 0..calls {
        let mach = 2.0 + 0.1 * call as f64;
        let mut case = Case3::tiny_anf(mach, 0.0);
        case.numerics.max_iter = iters;
        let t = Instant::now();
        let _plain = case.build_solver().map_err(|e| e.to_string())?;
        let plain = t.elapsed().as_secs_f64();
        let t = Instant::now();
        let _cached = case.build_solver_cached().map_err(|e| e.to_string())?;
        let cached = t.elapsed().as_secs_f64();
        let t = Instant::now();
        let mut runner = GpuRunner3::new(&case, prec).map_err(|e| e.to_string())?;
        runner.run_on(&case, false).map_err(|e| e.to_string())?;
        let total = t.elapsed().as_secs_f64();
        println!(
            "| {call} | {mach:.1} | {plain:.3} | {cached:.3} | {:.3} | {:.3} | {:.3} | {total:.3} |",
            runner.build_seconds, runner.ctx_seconds, runner.upload_seconds
        );
    }
    Ok(())
}
