//! End-to-end set-up and iteration-cost breakdown of one cold 3D point (the ANF at level 0, M 2, alpha 2 degrees) as the app runs it
//! (`GpuRunner3` in extended precision), plus diagnostics for the per-chunk overhead and for what a changed Mach number costs.
//!
//! ```text
//! BOWSHOCK_TIMING=1 cargo run --release -p bowshock-validate --example perf_e2e
//! ```
//!
//! Prints one `PERF {"case":"3d-anf-L0-point-M2",..}` line (the cold point: host build, context, upload, run) and the diagnostics as
//! plain text. Instrumentation only: nothing here changes a result.

use bowshock_cfd::gpu::{GpuSolver3, Precision};
use bowshock_core::engine::case3::Case3;
use bowshock_core::engine::gpu3::GpuRunner3;
use bowshock_core::engine::grid3::body_grid3;
use bowshock_core::engine::run3::forces_on;
use bowshock_core::engine::timing::{take_timings, PointTiming};
use std::time::Instant;

fn anf(mach: f64, alpha_deg: f64) -> Case3 {
    Case3::new(mach, alpha_deg.to_radians(), Case3::anf_body(), Some(Case3::anf_fins()), 0)
}

fn row(t: &PointTiming) {
    println!(
        "  M {:.2} a {:.0} {} cells {} | build {:.3} ctx {:.3} upload {:.3} reset {:.3} | first chunk {:.3} | {} more chunks {:.3} ({:.1} ms each) | {} downloads {:.3} forces {:.3} | final {:.3} | run {:.3} | {}/{} {}",
        t.mach,
        t.alpha_deg,
        if t.warm { "warm" } else { "cold" },
        t.cells,
        t.build_s,
        t.ctx_s,
        t.upload_s,
        t.reset_s,
        t.first_chunk_s,
        t.chunks_rest,
        t.rest_chunks_s,
        1e3 * t.rest_chunks_s / t.chunks_rest.max(1) as f64,
        t.downloads,
        t.download_s,
        t.forces_s,
        t.final_s,
        t.run_s,
        t.iters_stop,
        t.iters_exec,
        t.stop
    );
}

fn main() {
    let prec = Precision::Extended;
    // 1. the host side of the set-up, separated (grid alone, then grid + solver)
    let case = anf(2.0, 2.0);
    let t = Instant::now();
    let g = body_grid3(&case.grid).unwrap();
    let grid_s = t.elapsed().as_secs_f64();
    let t = Instant::now();
    let _ = case.build_solver().unwrap();
    let build_s = t.elapsed().as_secs_f64();
    println!("grid {}x{}x{} = {} cells: body_grid3 {:.3} s, build_solver (grid + Solver3::new) {:.3} s", g.ni, g.nj, g.nk, g.ni * g.nj * g.nk, grid_s, build_s);
    let _ = take_timings();

    // 2. the cold point, process-cold (first use of the GPU context and of the kernel cache)
    let t0 = Instant::now();
    let mut r = GpuRunner3::new(&case, prec).expect("GPU");
    let new_s = t0.elapsed().as_secs_f64();
    let res = r.run_on(&case, false).unwrap();
    let wall = t0.elapsed().as_secs_f64();
    println!(
        "cold point (new {new_s:.3} s + run {:.3} s) = {wall:.3} s: CN {:.4} x_cp {:.3} {:?} after {} iterations",
        wall - new_s,
        res.cn,
        res.x_cp,
        res.stop_reason,
        res.iterations
    );
    let iters = take_timings();
    iters.iter().for_each(row);
    let it = iters[0].iters_exec.max(1);
    println!(
        "PERF {{\"case\":\"3d-anf-L0-point-M2\",\"precision\":\"df32\",\"cells\":{},\"iters\":{},\"us_per_iter\":{:.3},\"launches_per_iter\":null,\"gpu_us_per_iter\":null,\"wall_s\":{:.3}}}",
        iters[0].cells,
        it,
        1e6 * wall / it as f64,
        wall
    );

    // 3. the same solver: warm alpha 6, then a different Mach (limiter, low-Mach flag, CFL cap, free stream all change)
    for (m, a, warm) in [(2.0, 6.0, true), (0.8, 2.0, true), (0.5, 2.0, true), (2.0, 2.0, true)] {
        let c = anf(m, a);
        let t = Instant::now();
        let _ = r.run_on(&c, warm).unwrap();
        println!("same solver, M {m} alpha {a}: {:.3} s", t.elapsed().as_secs_f64());
    }
    take_timings().iter().for_each(row);

    // 4. a second cold solver of the same case in this (now warm) process: is the compile cached?
    let t = Instant::now();
    let mut r2 = GpuRunner3::new(&case, prec).unwrap();
    let new2 = t.elapsed().as_secs_f64();
    let _ = r2.run_on(&case, false).unwrap();
    println!("second cold solver, same case: new {new2:.3} s, total {:.3} s", t.elapsed().as_secs_f64());
    take_timings().iter().for_each(row);
    // and one at another Mach
    let c = anf(0.8, 2.0);
    let t = Instant::now();
    let mut r3 = GpuRunner3::new(&c, prec).unwrap();
    let _ = r3.run_on(&c, false).unwrap();
    println!("cold solver, M 0.8: total {:.3} s", t.elapsed().as_secs_f64());
    take_timings().iter().for_each(row);

    // 5. per-chunk costs on a fresh solver: run(50) (trace + residual history) against run_replay(50), then downloads and forces
    let host = case.build_solver().unwrap();
    let ctx = luma::default_context().unwrap();
    let t = Instant::now();
    let mut gpu = GpuSolver3::from_host(&ctx, &host, prec).unwrap();
    println!("from_host {:.3} s", t.elapsed().as_secs_f64());
    let mut times = Vec::new();
    for _ in 0..14 {
        let t = Instant::now();
        gpu.run(50).unwrap();
        times.push(1e3 * t.elapsed().as_secs_f64());
    }
    println!("run(50) chunk times, ms (first 100 its first-order): {}", times.iter().map(|x| format!("{x:.1}")).collect::<Vec<_>>().join(" "));
    let mut times = Vec::new();
    for _ in 0..6 {
        let t = Instant::now();
        gpu.run_replay(50).unwrap();
        gpu.sync().unwrap();
        times.push(1e3 * t.elapsed().as_secs_f64());
    }
    println!("run_replay(50)+sync chunk times, ms: {}", times.iter().map(|x| format!("{x:.1}")).collect::<Vec<_>>().join(" "));
    let mut times = Vec::new();
    for _ in 0..6 {
        let t = Instant::now();
        gpu.run(50).unwrap();
        times.push(1e3 * t.elapsed().as_secs_f64());
    }
    println!("run(50) chunk times again, ms: {}", times.iter().map(|x| format!("{x:.1}")).collect::<Vec<_>>().join(" "));
    let t = Instant::now();
    let u = gpu.state_f64().unwrap();
    println!("state_f64 download {:.1} ms ({} cells)", 1e3 * t.elapsed().as_secs_f64(), u.len());
    let t = Instant::now();
    let _ = gpu.state_f64().unwrap();
    println!("state_f64 download (again) {:.1} ms", 1e3 * t.elapsed().as_secs_f64());
    let x_base = case.body.x_base();
    let t = Instant::now();
    let f = forces_on(&case, &host.grid, &u, host.cfg.freestream[4], x_base);
    println!("forces_on {:.1} ms (CN {:.4})", 1e3 * t.elapsed().as_secs_f64(), f.cn);
}
