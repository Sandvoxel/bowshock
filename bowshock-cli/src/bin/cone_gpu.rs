//! GPU vs CPU cone validation: M=2 flow over 10 and 15 degree cones, three grids, both limiters. The GPU solver runs in
//! extended precision (default; guarded and direct formulation) and, for the before/after comparison of the f32 residual
//! floor, in the original f32 precision and with the compensated state only.
//!
//! Usage: `cargo run --release --bin cone_gpu [-- --quick] [--limiter minmod|vanalbada|both]
//!         [--mach M] [--chunk K] [--bench [--no-cpu]]`
//! `--quick` skips the 400x200 grid, `--chunk` is the number of iterations recorded between two
//! synchronisations (default 50, max 256), `--bench` runs only the timing studies (steady
//! per-iteration benchmark, launch overhead, sync-interval sweep) and skips the validation runs.
//! CSV output: `out/gpu/`.

use bowshock_gpu_cases::cone::{compare, residual_on_state, run_cpu, run_gpu, Compare, RunResult};
use bowshock_gpu::{GpuSolver, Mode, Precision};
use bowshock_cases::cone::{build_solver, ConeCase};
use bowshock_core::flux::Limiter;
use bowshock_reference::taylor_maccoll::{solve_cone, ConeSolution};
use std::fs;
use std::io::Write;
use std::path::PathBuf;
use std::time::Instant;

struct Row {
    theta: f64,
    ni: usize,
    nj: usize,
    lim: &'static str,
    cp_gpu: f64,
    cp_cpu: f64,
    cp_tm: f64,
    beta_gpu: f64,
    beta_cpu: f64,
    /// Guarded-mode GPU (extended precision) vs CPU.
    cmp: Compare,
    /// Direct-mode GPU (extended precision) vs CPU.
    cmp_direct: Compare,
    /// Guarded GPU in the original f32 precision / with compensated state, vs CPU.
    cmp_f32: Compare,
    cmp_comp: Compare,
    /// Guarded (optimised) GPU run, direct (bowshock-core mirror) GPU run, both in the default (extended) precision; CPU run;
    /// guarded runs in the original f32 precision and with the compensated state only.
    gpu: RunResult,
    direct: RunResult,
    cpu: RunResult,
    f32run: RunResult,
    comp: RunResult,
    /// Residual (first iteration, geo-mean of 20) of the f32 / extended solver evaluated on the converged CPU f64 state.
    on_cpu_state_f32: (f64, f64),
    on_cpu_state_ext: (f64, f64),
}

fn write_csvs(out: &PathBuf, tag: &str, row: &Row, tm: &ConeSolution) {
    let mut f = fs::File::create(out.join(format!("cp_{tag}.csv"))).unwrap();
    writeln!(f, "x_over_L,p_over_pinf_gpu,cp_gpu,cp_cpu,cp_firstcell_gpu,cp_firstcell_cpu,rel_diff_cp,cp_taylor_maccoll").unwrap();
    for (g, c) in row.gpu.post.surface.iter().zip(&row.cpu.post.surface) {
        writeln!(
            f,
            "{:.8},{:.8},{:.8},{:.8},{:.8},{:.8},{:.4e},{:.8}",
            g.0, g.1, g.2, c.2, g.3, c.3, (g.2 - c.2) / c.2, tm.cp_surface
        )
        .unwrap();
    }
    let mut f = fs::File::create(out.join(format!("residual_{tag}.csv"))).unwrap();
    writeln!(f, "iter,density_residual_gpu,density_residual_cpu").unwrap();
    let n = row.gpu.residual.len().max(row.cpu.residual.len());
    for i in 0..n {
        let g = row.gpu.residual.get(i).map_or(String::new(), |v| format!("{v:e}"));
        let c = row.cpu.residual.get(i).map_or(String::new(), |v| format!("{v:e}"));
        writeln!(f, "{},{g},{c}", i + 1).unwrap();
    }
}

/// Minimum and the mean over the last 1000 iterations of the residual: the noise floor.
fn floor(res: &[f64]) -> (f64, f64) {
    let min = res.iter().cloned().fold(f64::INFINITY, f64::min);
    let tail = &res[res.len().saturating_sub(1000)..];
    (min, 10f64.powf(tail.iter().map(|x| x.log10()).sum::<f64>() / tail.len() as f64))
}

/// Cost of a trivial dependent Luma launch (read-modify-write of one buffer, no uniform of its own).
fn launch_overhead() {
    println!("\n## Luma launch overhead: 4000 dependent launches of `a[i] += 1` (one storage binding, luma grid uniform only)");
    println!("| invocations | host record us/launch | record+sync us/launch |");
    println!("|---|---|---|");
    let ctx = luma::default_context().unwrap();
    let k = ctx
        .kernel("@group(1) @binding(0) var<storage, read_write> a: array<f32>;\n@compute @workgroup_size(64)\nfn main(@builtin(global_invocation_id) gid: vec3<u32>) { if (gid.x < luma.size.x) { a[gid.x] = a[gid.x] + 1.0; } }")
        .build()
        .unwrap();
    for &n in &[64u32, 5_000, 80_000, 1_000_000] {
        let mut a = luma::Array::<f32>::zeros(n as usize).unwrap();
        k.launch([n], luma::args![&mut a]).unwrap();
        a.eval().unwrap();
        let reps = 4000;
        let t0 = Instant::now();
        for _ in 0..reps {
            k.launch([n], luma::args![&mut a]).unwrap();
        }
        let rec = t0.elapsed().as_secs_f64();
        a.eval().unwrap();
        let all = t0.elapsed().as_secs_f64();
        println!("| {n} | {:.2} | {:.2} |", 1e6 * rec / reps as f64, 1e6 * all / reps as f64);
    }
}

fn bench() {
    launch_overhead();
    println!("\n## GPU per-iteration timing study (15 deg, minmod, from free stream, 1000 iterations, f32; first-order phase off)");
    println!("Residual history = per-iteration density residual (history kernels), read back every K iterations.");
    println!("| mode | grid | hist, sync K=1 | K=10 | K=50 | K=250 | Luma norm() every 50 (no hist kernels) | no residual at all, record+sync | host recording time only |");
    println!("|---|---|---|---|---|---|---|---|---|");
    let tm = solve_cone(2.0, 15f64.to_radians()).unwrap();
    let ctx = luma::default_context().unwrap();
    for mode in [Mode::Direct, Mode::Guarded] {
        for &(ni, nj) in &[(100usize, 50usize), (200, 100), (400, 200)] {
            let mut case = ConeCase::new(2.0, 15.0, ni, nj);
            case.first_order_iters = 0;
            let host = build_solver(&case, tm.beta);
            let mut g = GpuSolver::new_with(&ctx, &host.grid, &host.cfg, mode, Precision::F32).unwrap();
            g.run(1).unwrap();
            let total = 1000usize;
            let mut cells = Vec::new();
            for &k in &[1usize, 10, 50, 250] {
                g.reset().unwrap();
                g.context().sync().unwrap();
                let t0 = Instant::now();
                for _ in 0..total / k {
                    g.run(k).unwrap();
                }
                cells.push(1e6 * t0.elapsed().as_secs_f64() / ((total / k) * k) as f64);
            }
            g.reset().unwrap();
            g.context().sync().unwrap();
            let t0 = Instant::now();
            g.record(total).unwrap();
            let t_rec = t0.elapsed().as_secs_f64();
            g.context().sync().unwrap();
            let t_all = t0.elapsed().as_secs_f64();
            g.reset().unwrap();
            g.context().sync().unwrap();
            let t0 = Instant::now();
            for _ in 0..total / 50 {
                g.run_luma_norm(50).unwrap();
            }
            let t_norm = 1e6 * t0.elapsed().as_secs_f64() / total as f64;
            println!(
                "| {mode:?} | {ni}x{nj} | {:.1} us | {:.1} us | {:.1} us | {:.1} us | {:.1} us | {:.1} us | {:.1} us |",
                cells[0], cells[1], cells[2], cells[3], t_norm, 1e6 * t_all / total as f64, 1e6 * t_rec / total as f64
            );
        }
    }
}

/// Steady per-iteration cost: 2000 iterations of the 15 deg / minmod case (MUSCL from the first
/// iteration), best of 3 repeats (the machine is shared; the minimum is the least disturbed run).
/// CPU: all rayon threads. GPU: history kernels on, sync every `chunk` iterations.
struct Perf {
    cpu_us: f64,
    /// Guarded mode: extended (default), compensated, original f32; direct mode: extended, original f32.
    guarded_us: f64,
    guarded_comp_us: f64,
    guarded_f32_us: f64,
    direct_us: f64,
    direct_f32_us: f64,
}

fn perf_study(grids: &[(usize, usize)], chunk: usize, with_cpu: bool) -> Vec<((usize, usize), Perf)> {
    let n = 2000usize;
    let tm = solve_cone(2.0, 15f64.to_radians()).unwrap();
    let ctx = luma::default_context().unwrap();
    println!("\n### Steady per-iteration cost (15 deg, minmod, {n} iterations from the free stream, best of 3, sync every K={chunk})");
    println!("| grid | cells | CPU f64 rayon us/iter | GPU-guarded extended us/iter | guarded compensated | guarded f32 | GPU-direct extended | direct f32 | guarded extended speed-up vs CPU | direct extended speed-up | extended / f32 cost (guarded) |");
    println!("|---|---|---|---|---|---|---|---|---|---|---|");
    let mut out = Vec::new();
    for &(ni, nj) in grids {
        let mut case = ConeCase::new(2.0, 15.0, ni, nj);
        case.first_order_iters = 0;
        let mut best_cpu = f64::INFINITY;
        for _ in 0..(if with_cpu { 3 } else { 0 }) {
            let mut s = build_solver(&case, tm.beta);
            for _ in 0..20 {
                s.step();
            }
            let t0 = Instant::now();
            for _ in 0..n {
                s.step();
            }
            best_cpu = best_cpu.min(1e6 * t0.elapsed().as_secs_f64() / n as f64);
        }
        let configs = [
            (Mode::Guarded, Precision::Extended),
            (Mode::Guarded, Precision::Compensated),
            (Mode::Guarded, Precision::F32),
            (Mode::Direct, Precision::Extended),
            (Mode::Direct, Precision::F32),
        ];
        let mut gpu = [0.0f64; 5];
        for (m, (mode, prec)) in configs.into_iter().enumerate() {
            let host = build_solver(&case, tm.beta);
            let mut g = GpuSolver::new_with(&ctx, &host.grid, &host.cfg, mode, prec).unwrap();
            g.run(1).unwrap();
            let mut best = f64::INFINITY;
            for _ in 0..3 {
                g.reset().unwrap();
                g.context().sync().unwrap();
                let t0 = Instant::now();
                for _ in 0..n / chunk {
                    g.run(chunk).unwrap();
                }
                best = best.min(1e6 * t0.elapsed().as_secs_f64() / ((n / chunk) * chunk) as f64);
            }
            gpu[m] = best;
        }
        println!(
            "| {ni}x{nj} | {} | {:.0} | {:.1} | {:.1} | {:.1} | {:.1} | {:.1} | {:.1}x | {:.1}x | {:.2}x |",
            ni * nj, best_cpu, gpu[0], gpu[1], gpu[2], gpu[3], gpu[4], best_cpu / gpu[0], best_cpu / gpu[3], gpu[0] / gpu[2]
        );
        out.push((
            (ni, nj),
            Perf {
                cpu_us: best_cpu,
                guarded_us: gpu[0],
                guarded_comp_us: gpu[1],
                guarded_f32_us: gpu[2],
                direct_us: gpu[3],
                direct_f32_us: gpu[4],
            },
        ));
    }
    out
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let quick = args.iter().any(|a| a == "--quick");
    let arg = |name: &str| args.iter().position(|a| a == name).and_then(|i| args.get(i + 1)).cloned();
    let limiters: Vec<(Limiter, &'static str)> = match arg("--limiter").as_deref() {
        Some("minmod") => vec![(Limiter::Minmod, "minmod")],
        Some("vanalbada") => vec![(Limiter::VanAlbada, "vanalbada")],
        _ => vec![(Limiter::Minmod, "minmod"), (Limiter::VanAlbada, "vanalbada")],
    };
    let mach: f64 = arg("--mach").and_then(|s| s.parse().ok()).unwrap_or(2.0);
    let chunk: usize = arg("--chunk").and_then(|s| s.parse().ok()).unwrap_or(50);
    let out_dir: PathBuf = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("..").join("out").join("gpu");
    fs::create_dir_all(&out_dir).expect("create out dir");

    let t0 = Instant::now();
    let ctx = luma::default_context().expect("open GPU");
    let ctx_seconds = t0.elapsed().as_secs_f64();
    println!("bowshock-gpu cone validation: M = {mach}, gamma = 1.4, CFL = 0.5, MUSCL + HLLC/Rusanov blend + SSP-RK3, f32 on GPU (Luma) vs f64 CPU (bowshock-core)");
    println!("sync interval K = {chunk} iterations; CPU rayon threads = {}", std::thread::available_parallelism().map_or(0, |n| n.get()));
    println!("GPU context creation: {:.2} s", ctx_seconds);
    let _ = ctx;

    let mut grids = vec![(100usize, 50usize), (200, 100)];
    if !quick {
        grids.push((400, 200));
    }

    if args.iter().any(|a| a == "--bench") {
        perf_study(&grids, chunk, !args.iter().any(|a| a == "--no-cpu"));
        bench();
        return;
    }

    let mut rows: Vec<Row> = Vec::new();
    for &(limiter, lname) in &limiters {
        for &theta in &[10.0f64, 15.0] {
            let tm = solve_cone(mach, theta.to_radians()).expect("attached conical shock");
            println!(
                "\n=== {theta} deg cone, {lname}: Taylor-Maccoll beta = {:.4} deg, Cp = {:.5}",
                tm.beta.to_degrees(),
                tm.cp_surface
            );
            for &(ni, nj) in &grids {
                let mut case = ConeCase::new(mach, theta, ni, nj);
                case.limiter = limiter;
                let cpu = run_cpu(&case, &tm);
                let gpu = run_gpu(&case, &tm, chunk, Mode::Guarded, Precision::Extended).expect("GPU run (guarded, extended)");
                let direct = run_gpu(&case, &tm, chunk, Mode::Direct, Precision::Extended).expect("GPU run (direct, extended)");
                let f32run = run_gpu(&case, &tm, chunk, Mode::Guarded, Precision::F32).expect("GPU run (guarded, f32)");
                let comp = run_gpu(&case, &tm, chunk, Mode::Guarded, Precision::Compensated).expect("GPU run (guarded, compensated)");
                let fs = bowshock_core::gas::freestream(mach);
                let cmp = compare(&case, &fs, &gpu, &cpu);
                let cmp_direct = compare(&case, &fs, &direct, &cpu);
                let cmp_f32 = compare(&case, &fs, &f32run, &cpu);
                let cmp_comp = compare(&case, &fs, &comp, &cpu);
                let on_state = |prec| {
                    let on = residual_on_state(&case, &tm, &cpu.u, 20, Mode::Guarded, prec).expect("residual on CPU state");
                    (on[0], 10f64.powf(on.iter().map(|x| x.log10()).sum::<f64>() / on.len() as f64))
                };
                let (on_cpu_state_f32, on_cpu_state_ext) = (on_state(Precision::F32), on_state(Precision::Extended));
                let gmin = floor(&gpu.residual).0;
                println!(
                    "  {ni}x{nj}: CPU {} it ({:.1} s, {}, {:.1} orders, final res {:.2e}) | GPU {} it stop / {} exec ({:.2} s, {}, {:.1} orders; min res {:.1e}) | f32-solver residual on the CPU state: {:.1e} (first it.), {:.1e} (mean of 20); extended: {:.1e}, {:.1e} | f32 solver {:.1} orders, compensated {:.1} orders",
                    cpu.iters_stop, cpu.loop_seconds, cpu.reason, cpu.drop_orders, cpu.residual.last().unwrap(),
                    gpu.iters_stop, gpu.iters_exec, gpu.loop_seconds, gpu.reason, gpu.drop_orders, gmin,
                    on_cpu_state_f32.0, on_cpu_state_f32.1, on_cpu_state_ext.0, on_cpu_state_ext.1,
                    f32run.drop_orders, comp.drop_orders
                );
                println!(
                    "         Cp GPU {:.5} CPU {:.5} TM {:.5} (avg diff {:+.1e}%), max rel Cp diff {:.1e}% (window) / {:.1e}% (all), field: max dp/pinf {:.2e}, rms {:.2e}, max drho/rho {:.2e}; min rho {:.4} min p {:.4}; direct-mode Cp {:.5} (avg diff vs CPU {:+.1e}%)",
                    gpu.post.cp_avg, cpu.post.cp_avg, tm.cp_surface, 100.0 * cmp.avg_cp_rel,
                    100.0 * cmp.max_rel_cp_window, 100.0 * cmp.max_rel_cp_all, cmp.max_dp_over_pinf, cmp.rms_dp_over_pinf, cmp.max_rel_rho,
                    gpu.post.min_rho, gpu.post.min_p, direct.post.cp_avg, 100.0 * cmp_direct.avg_cp_rel
                );
                let row = Row {
                    theta,
                    ni,
                    nj,
                    lim: lname,
                    cp_gpu: gpu.post.cp_avg,
                    cp_cpu: cpu.post.cp_avg,
                    cp_tm: tm.cp_surface,
                    beta_gpu: gpu.post.beta_num.to_degrees(),
                    beta_cpu: cpu.post.beta_num.to_degrees(),
                    cmp,
                    cmp_direct,
                    cmp_f32,
                    cmp_comp,
                    gpu,
                    direct,
                    cpu,
                    f32run,
                    comp,
                    on_cpu_state_f32,
                    on_cpu_state_ext,
                };
                write_csvs(&out_dir, &format!("{}deg_{}x{}_{}", theta as i32, ni, nj, lname), &row, &tm);
                rows.push(row);
            }
        }
    }

    let perf = perf_study(&grids, chunk, true);
    let perf_of = |r: &Row| perf.iter().find(|(g, _)| *g == (r.ni, r.nj)).map(|(_, p)| p).unwrap();

    // ---- tables ----
    println!("\n### Accuracy: GPU (extended precision, default) vs CPU (f64) and vs Taylor-Maccoll; before = original f32 solver, compensated = compensated state only");
    println!("| cone | grid | limiter | Cp GPU | Cp CPU | Cp TM | GPU err vs TM % | CPU err vs TM % | avg Cp diff GPU-CPU % | max rel Cp diff (0.4<=x/L<=0.9) % | max rel Cp diff (all stations) % | beta GPU [deg] | beta CPU [deg] | direct-mode avg Cp diff vs CPU % | direct-mode max rel Cp diff (window) % | f32 (before) avg Cp diff % | f32 (before) max rel Cp diff (window) % | compensated avg Cp diff % |");
    println!("|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|");
    for r in &rows {
        println!(
            "| {:.0} deg | {}x{} | {} | {:.5} | {:.5} | {:.5} | {:+.3} | {:+.3} | {:+.1e} | {:.1e} | {:.1e} | {:.3} | {:.3} | {:+.1e} | {:.1e} | {:+.1e} | {:.1e} | {:+.1e} |",
            r.theta, r.ni, r.nj, r.lim, r.cp_gpu, r.cp_cpu, r.cp_tm,
            100.0 * (r.cp_gpu - r.cp_tm) / r.cp_tm, 100.0 * (r.cp_cpu - r.cp_tm) / r.cp_tm,
            100.0 * r.cmp.avg_cp_rel, 100.0 * r.cmp.max_rel_cp_window, 100.0 * r.cmp.max_rel_cp_all,
            r.beta_gpu, r.beta_cpu, 100.0 * r.cmp_direct.avg_cp_rel, 100.0 * r.cmp_direct.max_rel_cp_window,
            100.0 * r.cmp_f32.avg_cp_rel, 100.0 * r.cmp_f32.max_rel_cp_window, 100.0 * r.cmp_comp.avg_cp_rel
        );
    }
    println!("\n### Field difference GPU (guarded, extended) - CPU at the end of the runs (f32 = original f32 solver)");
    println!("| cone | grid | limiter | max abs dp/p_inf | rms dp/p_inf | max rel drho/rho | f32: max abs dp/p_inf | f32: rms dp/p_inf | f32: max rel drho/rho |");
    println!("|---|---|---|---|---|---|---|---|---|");
    for r in &rows {
        println!(
            "| {:.0} deg | {}x{} | {} | {:.2e} | {:.2e} | {:.2e} | {:.2e} | {:.2e} | {:.2e} |",
            r.theta, r.ni, r.nj, r.lim, r.cmp.max_dp_over_pinf, r.cmp.rms_dp_over_pinf, r.cmp.max_rel_rho,
            r.cmp_f32.max_dp_over_pinf, r.cmp_f32.rms_dp_over_pinf, r.cmp_f32.max_rel_rho
        );
    }
    println!("\n### Convergence (density residual drop from its peak after iteration 20; stop = criterion of `run_cone`)");
    println!("| cone | grid | limiter | CPU iters | CPU stop | CPU drop (orders) | CPU final res | GPU iters (stop/exec) | GPU stop | GPU drop (orders) | GPU final res | GPU-direct iters (stop/exec) | GPU-direct stop | GPU-direct drop | f32 (before): stop | f32 drop | f32 tail geo-mean res | compensated: stop | compensated drop | compensated tail geo-mean res |");
    println!("|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|");
    for r in &rows {
        println!(
            "| {:.0} deg | {}x{} | {} | {} | {} | {:.1} | {:.2e} | {}/{} | {} | {:.1} | {:.2e} | {}/{} | {} | {:.1} | {} | {:.1} | {:.2e} | {} | {:.1} | {:.2e} |",
            r.theta, r.ni, r.nj, r.lim, r.cpu.iters_stop, r.cpu.reason, r.cpu.drop_orders,
            r.cpu.residual.last().unwrap(), r.gpu.iters_stop, r.gpu.iters_exec, r.gpu.reason, r.gpu.drop_orders,
            r.gpu.residual[r.gpu.iters_stop - 1], r.direct.iters_stop, r.direct.iters_exec, r.direct.reason, r.direct.drop_orders,
            r.f32run.reason, r.f32run.drop_orders, floor(&r.f32run.residual).1,
            r.comp.reason, r.comp.drop_orders, floor(&r.comp.residual).1
        );
    }
    println!("\n### Residual of the converged CPU f64 state evaluated by the GPU solver in f32 (original) and in extended precision");
    println!("(f32: 20 iterations from the CPU state; extended: the GPU operator has f32-rounded geometry (normals, face weights, volumes), so it differs from the CPU operator by a fixed small amount, which is what this measures)");
    println!("| cone | grid | limiter | CPU final residual (f64) | f32 residual, iteration 1 | f32 residual, geo-mean of 20 | extended residual, iteration 1 | extended residual, geo-mean of 20 |");
    println!("|---|---|---|---|---|---|---|---|");
    for r in &rows {
        println!(
            "| {:.0} deg | {}x{} | {} | {:.2e} | {:.2e} | {:.2e} | {:.2e} | {:.2e} |",
            r.theta, r.ni, r.nj, r.lim, r.cpu.residual.last().unwrap(), r.on_cpu_state_f32.0, r.on_cpu_state_f32.1,
            r.on_cpu_state_ext.0, r.on_cpu_state_ext.1
        );
    }

    println!("\n### Wall time of the convergence runs themselves (iteration loop incl. history readback every K={chunk}; setup excluded; shared desktop, noisy: see the steady benchmark above)");
    println!("| cone | grid | limiter | CPU iters | CPU us/iter | GPU-guarded ext us/iter | GPU-direct ext us/iter | GPU iters exec (stop) | CPU total s | GPU-guarded ext total s | GPU-direct ext total s | f32 guarded total s (8000+ its) |");
    println!("|---|---|---|---|---|---|---|---|---|---|---|---|");
    for r in &rows {
        println!(
            "| {:.0} deg | {}x{} | {} | {} | {:.0} | {:.0} | {:.0} | {} ({}) | {:.2} | {:.2} | {:.2} | {:.2} |",
            r.theta, r.ni, r.nj, r.lim, r.cpu.iters_exec, r.cpu.us_per_iter(), r.gpu.us_per_iter(), r.direct.us_per_iter(),
            r.gpu.iters_exec, r.gpu.iters_stop, r.cpu.loop_seconds, r.gpu.loop_seconds, r.direct.loop_seconds, r.f32run.loop_seconds
        );
    }
    println!("\n### Total time to the stopping criterion of each solver (iterations to stop x steady us/iter; GPU extended and CPU both stop at 6 orders; the f32 solver (before) ends at its plateau test)");
    println!("| cone | grid | limiter | CPU iters | CPU s | GPU-guarded ext iters (stop) | GPU-guarded ext s | GPU-direct ext s | guarded speed-up | direct speed-up | f32 guarded iters (stop) | f32 guarded s | f32 orders reached |");
    println!("|---|---|---|---|---|---|---|---|---|---|---|---|---|");
    for r in &rows {
        let pf = perf_of(r);
        let (tc, tg, td, tf) = (
            r.cpu.iters_stop as f64 * pf.cpu_us * 1e-6,
            r.gpu.iters_stop as f64 * pf.guarded_us * 1e-6,
            r.direct.iters_stop as f64 * pf.direct_us * 1e-6,
            r.f32run.iters_stop as f64 * pf.guarded_f32_us * 1e-6,
        );
        println!(
            "| {:.0} deg | {}x{} | {} | {} | {:.2} | {} | {:.2} | {:.2} | {:.1}x | {:.1}x | {} | {:.2} | {:.1} |",
            r.theta, r.ni, r.nj, r.lim, r.cpu.iters_stop, tc, r.gpu.iters_stop, tg, td, tc / tg, tc / td,
            r.f32run.iters_stop, tf, r.f32run.drop_orders
        );
    }
    println!("\n### One-time GPU setup (kernel build + geometry/state upload) and first-launch warm-up, seconds (context creation {ctx_seconds:.2} s); extended precision");
    println!("| cone | grid | limiter | guarded setup | guarded warm-up | direct setup | direct warm-up |");
    println!("|---|---|---|---|---|---|---|");
    for r in &rows {
        println!(
            "| {:.0} deg | {}x{} | {} | {:.3} | {:.3} | {:.3} | {:.3} |",
            r.theta, r.ni, r.nj, r.lim, r.gpu.setup_seconds, r.gpu.warmup_seconds, r.direct.setup_seconds, r.direct.warmup_seconds
        );
    }

    let mut f = fs::File::create(out_dir.join("summary.csv")).unwrap();
    writeln!(f, "theta_deg,ni,nj,limiter,cp_gpu,cp_cpu,cp_tm,avg_cp_rel_diff,max_rel_cp_diff_window,max_rel_cp_diff_all,max_dp_over_pinf,rms_dp_over_pinf,max_rel_rho,beta_gpu_deg,beta_cpu_deg,cpu_iters,cpu_reason,cpu_drop_orders,cpu_final_residual,gpu_iters_stop,gpu_iters_exec,gpu_reason,gpu_drop_orders,gpu_final_residual,gpu_min_residual,direct_iters_stop,direct_drop_orders,direct_avg_cp_rel_diff,direct_max_rel_cp_diff_window,f32_drop_orders,f32_tail_residual,f32_avg_cp_rel_diff,comp_drop_orders,comp_tail_residual,comp_avg_cp_rel_diff,f32_residual_on_cpu_state,ext_residual_on_cpu_state,cpu_steady_us_per_iter,gpu_guarded_ext_steady_us_per_iter,gpu_guarded_comp_steady_us_per_iter,gpu_guarded_f32_steady_us_per_iter,gpu_direct_ext_steady_us_per_iter,gpu_direct_f32_steady_us_per_iter,gpu_setup_s,gpu_warmup_s").unwrap();
    for r in &rows {
        let pf = perf_of(r);
        writeln!(
            f,
            "{},{},{},{},{:.8},{:.8},{:.8},{:.4e},{:.4e},{:.4e},{:.4e},{:.4e},{:.4e},{:.4},{:.4},{},{},{:.2},{:.4e},{},{},{},{:.2},{:.4e},{:.4e},{},{:.2},{:.4e},{:.4e},{:.2},{:.4e},{:.4e},{:.2},{:.4e},{:.4e},{:.4e},{:.4e},{:.1},{:.1},{:.1},{:.1},{:.1},{:.1},{:.4},{:.4}",
            r.theta, r.ni, r.nj, r.lim, r.cp_gpu, r.cp_cpu, r.cp_tm, r.cmp.avg_cp_rel, r.cmp.max_rel_cp_window, r.cmp.max_rel_cp_all,
            r.cmp.max_dp_over_pinf, r.cmp.rms_dp_over_pinf, r.cmp.max_rel_rho, r.beta_gpu, r.beta_cpu,
            r.cpu.iters_stop, r.cpu.reason, r.cpu.drop_orders, r.cpu.residual.last().unwrap(),
            r.gpu.iters_stop, r.gpu.iters_exec, r.gpu.reason, r.gpu.drop_orders, r.gpu.residual[r.gpu.iters_stop - 1], floor(&r.gpu.residual).0,
            r.direct.iters_stop, r.direct.drop_orders, r.cmp_direct.avg_cp_rel, r.cmp_direct.max_rel_cp_window,
            r.f32run.drop_orders, floor(&r.f32run.residual).1, r.cmp_f32.avg_cp_rel,
            r.comp.drop_orders, floor(&r.comp.residual).1, r.cmp_comp.avg_cp_rel,
            r.on_cpu_state_f32.0, r.on_cpu_state_ext.0,
            pf.cpu_us, pf.guarded_us, pf.guarded_comp_us, pf.guarded_f32_us, pf.direct_us, pf.direct_f32_us,
            r.gpu.setup_seconds, r.gpu.warmup_seconds
        )
        .unwrap();
    }

    bench();
    println!("\nCSV output written to {}", out_dir.canonicalize().unwrap_or(out_dir).display());
}
