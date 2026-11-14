//! Where the time of the `eval` loop form goes (`LazySolver::run_eval`: the step is built in Rust and `eval_all`ed every
//! iteration, no replay): host time to build one step's expression, host time per iteration of the whole loop, GPU time and
//! kernel count per iteration, and whether any kernel is compiled after the first iteration.
//!
//! `cargo run --release --example eval_loop [-- --n 300]`

use bowshock_cases::cone::{build_solver, ConeCase};
use bowshock_gpu::Staging;
use bowshock_reference::taylor_maccoll::solve_cone;
use std::time::Instant;

macro_rules! report {
    ($module:ident, $name:expr, $staging:expr, $ctx:expr, $host:expr, $n:expr) => {{
        let mut s = bowshock_gpu::$module::LazySolver::new($ctx, &$host.grid, &$host.cfg).unwrap();
        s.staging = $staging;
        // first iteration: construction, tracing, compilation
        let t0 = Instant::now();
        s.run_eval(1).unwrap();
        s.sync().unwrap();
        let first = t0.elapsed().as_secs_f64() * 1e3;
        s.run_eval(20).unwrap();
        s.sync().unwrap();
        // the loop, host time per iteration (the GPU keeps up: nothing is waited for inside it)
        let t0 = Instant::now();
        let h = s.run_eval($n).unwrap();
        let queued = t0.elapsed().as_secs_f64();
        s.sync().unwrap();
        let total = t0.elapsed().as_secs_f64();
        drop(h);
        // building the expression only
        let trace = s.trace_seconds($n);
        // one profiled iteration
        let p = s.profile_step().unwrap();
        println!(
            "| {} | {:?} | {first:.1} | {:.0} | {:.0} | {:.0} | {} | {} |",
            $name,
            $staging,
            1e6 * total / $n as f64,
            1e6 * trace / $n as f64,
            1e6 * (queued - trace).max(0.0) / $n as f64,
            p.launches(),
            p.total().as_micros()
        );
    }};
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let n = args.iter().position(|a| a == "--n").and_then(|i| args.get(i + 1)).and_then(|s| s.parse::<usize>().ok()).unwrap_or(300);
    let tm = solve_cone(2.0, 15f64.to_radians()).unwrap();
    let mut case = ConeCase::new(2.0, 15.0, 200, 100);
    case.first_order_iters = 0;
    let host = build_solver(&case, tm.beta);
    let ctx = luma::default_context().unwrap();
    if args.iter().any(|a| a == "--trace-only") {
        // for `perf record`: only the Rust side of the loop, no GPU work
        let mut s = bowshock_gpu::f32s::LazySolver::new(&ctx, &host.grid, &host.cfg).unwrap();
        s.staging = Staging::Fluxes;
        println!("{:.0} us per step", 1e6 * s.trace_seconds(100) / 100.0);
        return;
    }
    println!("eval loop on the 200x100 cone grid, {n} iterations; first = the first iteration (ms, with compilation), the other times in us per iteration");
    println!("| precision | staging | first (ms) | loop total | build the step (Rust only) | rest of the loop (eval_all) | launches / step | GPU us / step |");
    println!("|---|---|---|---|---|---|---|---|");
    for staging in [Staging::Fusion, Staging::Fluxes] {
        report!(f32s, "f32", staging, &ctx, host, n);
        report!(df32s, "df32", staging, &ctx, host, n);
    }
}
