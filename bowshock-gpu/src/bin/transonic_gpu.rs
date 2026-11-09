//! GPU vs CPU transonic Mach sweep, mirroring `bowshock-cfd`'s `transonic` bin: nose-cylinder body on the large-domain grid with
//! characteristic far-field BCs, forebody pressure drag CD vs Mach compared between the f64 CPU solver and the GPU solver
//! (extended precision, guarded formulation), plus the Stoney free-flight data (NACA RM L53K17, digitised) for reference.
//!
//! Usage: `cargo run --release --bin transonic_gpu -- [--body cone|karman|lvhaack|sqrt|ogive] [--level 0|1|2] [--quick]
//!   [--mach 0.8,0.9,...] [--warm] [--domain <body lengths>] [--max-iter N] [--drop <orders>] [--outflow pressure]
//!   [--limiter minmod|vanalbada] [--scheme explicit|implicit] [--lowmach on|off] [--tag name] [--no-bench] [--bench-only]`
//! `--scheme implicit` = the line-implicit convergence scheme of `bowshock-cfd` (`bowshock_gpu::shaders_impl`; default limiter then van Albada).
//! The options and defaults are those of the CPU bin (`--quick` = grid level 0; default level 1; 12 Mach numbers 0.7 ... 1.5;
//! cold start for every Mach number, `--warm` = start from the previous converged field). Every Mach number is run by the CPU and by
//! the GPU with the same procedure and stopping rules. The GPU synchronises every 50 iterations (the CD sampling interval).
//! Output: `out/gpu_transonic/`.

use bowshock_cfd::flux::Limiter;
use bowshock_cfd::implicit::ImplicitParams;
use bowshock_cfd::solver::{Bc, Scheme};
use bowshock_cfd::transonic::{named_body, run_on, stoney, tangent_cone_cd, TransonicCase, TransonicResult};
use bowshock_gpu::transonic::{GpuTransonic, GpuTransonicRun, SYNC};
use bowshock_gpu::{Mode, Precision};
use std::fs;
use std::io::Write;
use std::path::{Path, PathBuf};
use std::time::Instant;

const DEFAULT_MACH: [f64; 12] = [0.7, 0.8, 0.85, 0.9, 0.95, 0.98, 1.0, 1.02, 1.05, 1.1, 1.2, 1.5];

fn arg<'a>(args: &'a [String], key: &str) -> Option<&'a str> {
    args.iter().position(|a| a == key).and_then(|i| args.get(i + 1)).map(String::as_str)
}

struct Row {
    mach: f64,
    cpu: TransonicResult,
    cpu_s: f64,
    gpu: GpuTransonicRun,
    /// max over wall faces of |Cp_gpu - Cp_cpu|.
    max_dcp: f64,
    stoney: Option<f64>,
    theory: f64,
}

fn write_case_files(dir: &Path, tag: &str, r: &Row) {
    let mut f = fs::File::create(dir.join(format!("cp_{tag}_M{}.csv", r.mach))).unwrap();
    writeln!(f, "x_over_d,r_over_d,cp_gpu,cp_cpu,cp_firstcell_gpu,cp_firstcell_cpu,diff_cp").unwrap();
    for (g, c) in r.gpu.result.wall.iter().zip(&r.cpu.wall) {
        writeln!(f, "{:.8},{:.8},{:.8},{:.8},{:.8},{:.8},{:.4e}", g.x, g.r, g.cp, c.cp, g.cp_cell, c.cp_cell, g.cp - c.cp).unwrap();
    }
    let mut f = fs::File::create(dir.join(format!("residual_{tag}_M{}.csv", r.mach))).unwrap();
    writeln!(f, "iter,density_residual_gpu,density_residual_cpu").unwrap();
    let n = r.gpu.result.residual.len().max(r.cpu.residual.len());
    for i in 0..n {
        let g = r.gpu.result.residual.get(i).map_or(String::new(), |v| format!("{v:e}"));
        let c = r.cpu.residual.get(i).map_or(String::new(), |v| format!("{v:e}"));
        writeln!(f, "{},{g},{c}", i + 1).unwrap();
    }
    let mut f = fs::File::create(dir.join(format!("cd_history_{tag}_M{}.csv", r.mach))).unwrap();
    writeln!(f, "iter,cd_gpu,cd_cpu").unwrap();
    for i in 0..r.gpu.result.cd_history.len().max(r.cpu.cd_history.len()) {
        let g = r.gpu.result.cd_history.get(i).map_or(String::new(), |v| format!("{:.8}", v.1));
        let c = r.cpu.cd_history.get(i).map_or(String::new(), |v| format!("{:.8}", v.1));
        writeln!(f, "{},{g},{c}", 50 * (i + 1)).unwrap();
    }
}

/// Steady per-iteration cost on the case grid at `case.mach`: 1000 iterations from the free stream (MUSCL from the first
/// iteration), best of 3, GPU with history kernels and a sync every `SYNC` iterations.
/// Returns `(cpu_us, guarded_ext_us, direct_ext_us, guarded_f32_us)`.
fn perf(case: &TransonicCase) -> (f64, f64, f64, f64) {
    let n = 1000usize;
    let implicit = matches!(case.scheme, Scheme::Implicit(_));
    let mut case = case.clone();
    case.first_order_iters = 0;
    let mut best_cpu = f64::INFINITY;
    for _ in 0..3 {
        let mut s = case.build_solver();
        for _ in 0..20 {
            s.step();
        }
        let t0 = Instant::now();
        for _ in 0..n {
            s.step();
        }
        best_cpu = best_cpu.min(1e6 * t0.elapsed().as_secs_f64() / n as f64);
    }
    let mut gpu = [f64::NAN; 3];
    for (m, (mode, prec)) in [(Mode::Guarded, Precision::Extended), (Mode::Direct, Precision::Extended), (Mode::Guarded, Precision::F32)].into_iter().enumerate() {
        if implicit && mode == Mode::Direct {
            continue; // the line-implicit scheme exists in the guarded formulation only
        }
        let mut gt = GpuTransonic::new(&case, mode, prec).unwrap();
        gt.gpu.run(1).unwrap();
        let mut best = f64::INFINITY;
        for _ in 0..3 {
            gt.gpu.reset().unwrap();
            gt.gpu.context().sync().unwrap();
            let t0 = Instant::now();
            for _ in 0..n / SYNC {
                gt.gpu.run(SYNC).unwrap();
            }
            best = best.min(1e6 * t0.elapsed().as_secs_f64() / ((n / SYNC) * SYNC) as f64);
        }
        gpu[m] = best;
    }
    (best_cpu, gpu[0], gpu[1], gpu[2])
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let quick = args.iter().any(|a| a == "--quick");
    let warm = args.iter().any(|a| a == "--warm");
    let with_bench = !args.iter().any(|a| a == "--no-bench");
    let level: u32 = arg(&args, "--level").and_then(|v| v.parse().ok()).unwrap_or(if quick { 0 } else { 1 });
    let body_name = arg(&args, "--body").unwrap_or("cone");
    let (body, data) = named_body(body_name).unwrap_or_else(|| panic!("unknown body '{body_name}' (cone|karman|lvhaack|sqrt|ogive)"));
    let machs: Vec<f64> = match arg(&args, "--mach") {
        Some(l) => l.split(',').map(|v| v.parse().expect("Mach list")).collect(),
        None => DEFAULT_MACH.to_vec(),
    };
    let implicit = arg(&args, "--scheme") == Some("implicit");
    let limiter = match arg(&args, "--limiter") {
        Some("vanalbada") => Limiter::VanAlbada,
        Some("minmod") => Limiter::Minmod,
        _ if implicit => Limiter::VanAlbada,
        _ => Limiter::Minmod,
    };
    let low_mach = arg(&args, "--lowmach") == Some("on");
    let tag = arg(&args, "--tag").map(String::from).unwrap_or_else(|| format!("{body_name}_L{level}{}{}", if implicit { "_imp" } else { "" }, if warm { "_warm" } else { "" }));
    let out_dir: PathBuf = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("..").join("out").join("gpu_transonic");
    fs::create_dir_all(&out_dir).expect("create out dir");

    let make_case = |m: f64| {
        let cyl = 6.0;
        let mut c = TransonicCase::with_body(m, level, body, cyl);
        c.limiter = limiter;
        c.low_mach = low_mach;
        if implicit {
            c.scheme = Scheme::Implicit(ImplicitParams::DEFAULT);
            c.first_order_iters = 100;
        }
        if let Some(v) = arg(&args, "--max-iter").and_then(|v| v.parse().ok()) {
            c.max_iter = v;
        }
        if let Some(v) = arg(&args, "--drop").and_then(|v| v.parse().ok()) {
            c.drop_orders = v;
        }
        if arg(&args, "--outflow") == Some("pressure") {
            c.outflow = Bc::PressureOutflow;
        }
        if let Some(d) = arg(&args, "--domain").and_then(|v| v.parse::<f64>().ok()) {
            let lb = body.nose_length() + cyl;
            c.grid.x_up = d * lb;
            c.grid.r_far = d * lb;
        }
        c
    };

    let c0 = make_case(machs[0]);
    let (ni, nj) = c0.cells();
    let t0 = Instant::now();
    let _ctx = luma::default_context().expect("open GPU");
    println!(
        "bowshock-gpu transonic: body {body_name} (nose l/d {:.3}), grid L{level} {ni}x{nj} ({} cells), limiter {limiter:?}, outflow {:?}, {}; f32 on GPU (Luma, extended precision) vs f64 CPU (bowshock-cfd)",
        body.nose_length(),
        ni * nj,
        c0.outflow,
        if warm { "warm-start sweep" } else { "cold start for every Mach" }
    );
    println!("GPU sync interval = {SYNC} iterations; CPU rayon threads = {}; GPU context creation {:.2} s", std::thread::available_parallelism().map_or(0, |n| n.get()), t0.elapsed().as_secs_f64());

    if args.iter().any(|a| a == "--bench-only") {
        let (c, g, d, f) = perf(&make_case(0.9));
        println!("\n### Steady per-iteration cost (M = 0.9, 1000 iterations from the free stream, best of 3, sync every {SYNC}; history kernels on)");
        println!("| grid | cells | CPU f64 rayon us/iter | GPU-guarded extended us/iter | GPU-direct extended | guarded f32 | guarded extended speed-up vs CPU | direct extended speed-up | extended / f32 cost |");
        println!("|---|---|---|---|---|---|---|---|---|");
        println!("| L{level} {ni}x{nj} | {} | {c:.0} | {g:.1} | {d:.1} | {f:.1} | {:.1}x | {:.1}x | {:.2}x |", ni * nj, c / g, c / d, g / f);
        return;
    }

    let mut rows: Vec<Row> = Vec::new();
    let mut cpu_solver = c0.build_solver();
    let mut gt = GpuTransonic::new(&c0, Mode::Guarded, Precision::Extended).expect("GPU solver");
    println!("GPU setup (kernel build + upload): {:.3} s", gt.setup_seconds);
    let (mut r_ref_cpu, mut r_ref_gpu): (Option<f64>, Option<f64>) = (None, None);
    for (k, &m) in machs.iter().enumerate() {
        let case = make_case(m);
        if !warm || k == 0 {
            cpu_solver = case.build_solver();
        }
        let t0 = Instant::now();
        let cpu = run_on(&case, &mut cpu_solver, warm && k > 0, if warm { r_ref_cpu } else { None });
        let cpu_s = t0.elapsed().as_secs_f64();
        let gpu = gt.run_on(&case, warm && k > 0, if warm { r_ref_gpu } else { None }).expect("GPU run");
        if warm && k == 0 {
            r_ref_cpu = Some(cpu.residual_peak);
            r_ref_gpu = Some(gpu.result.residual_peak);
        }
        let max_dcp = gpu.result.wall.iter().zip(&cpu.wall).fold(0.0f64, |mx, (a, b)| mx.max((a.cp - b.cp).abs()));
        let row = Row {
            mach: m,
            cpu_s,
            max_dcp,
            stoney: data.and_then(|d| stoney::interp(d, m)),
            theory: tangent_cone_cd(&body, m),
            cpu,
            gpu,
        };
        println!(
            "M = {m:<5} CPU {:>6} it ({:6.1} s, {}, {:.1} orders) | GPU {:>6} it stop / {} exec ({:6.2} s, {}, {:.1} orders) | CD_nose GPU {:.5} CPU {:.5} (diff {:+.2e}), max |dCp| {:.1e}, Cp_min GPU {:.4} CPU {:.4}, far-field |dp/p| GPU {:.1e} CPU {:.1e}",
            row.cpu.iters, row.cpu_s, row.cpu.stop_reason, row.cpu.drop_orders, row.gpu.iters_stop, row.gpu.iters_exec, row.gpu.loop_seconds,
            row.gpu.result.stop_reason, row.gpu.result.drop_orders, row.gpu.result.cd_nose, row.cpu.cd_nose, row.gpu.result.cd_nose - row.cpu.cd_nose,
            row.max_dcp, row.gpu.result.cp_min, row.cpu.cp_min, row.gpu.result.farfield_dp, row.cpu.farfield_dp
        );
        write_case_files(&out_dir, &tag, &row);
        rows.push(row);
    }

    // ---- tables ----
    println!("\n### {body_name}, grid L{level} {ni}x{nj}, {}: forebody CD, GPU (guarded, extended) vs CPU (f64)", if warm { "warm-start sweep" } else { "cold starts" });
    println!("| M | CD GPU | CD CPU | GPU - CPU | (GPU - CPU)/CPU % | first-cell CD diff | Cp_min GPU | Cp_min CPU | max abs dCp (wall) | M_wall,max GPU | M_wall,max CPU | Stoney (digitised) | tangent-cone / T-M |");
    println!("|---|---|---|---|---|---|---|---|---|---|---|---|---|");
    for r in &rows {
        let (g, c) = (&r.gpu.result, &r.cpu);
        println!(
            "| {} | {:.5} | {:.5} | {:+.2e} | {:+.2e} | {:+.2e} | {:.4} | {:.4} | {:.1e} | {:.3} | {:.3} | {} | {} |",
            r.mach, g.cd_nose, c.cd_nose, g.cd_nose - c.cd_nose, 100.0 * (g.cd_nose - c.cd_nose) / c.cd_nose, g.cd_nose_cell - c.cd_nose_cell, g.cp_min, c.cp_min,
            r.max_dcp, g.m_wall_max, c.m_wall_max, r.stoney.map_or("-".to_string(), |v| format!("{v:.3}")),
            if r.theory.is_nan() { "-".to_string() } else { format!("{:.5}", r.theory) }
        );
    }
    println!("\n### Convergence");
    println!("| M | CPU iters | CPU stop | CPU drop | CPU final res | GPU iters (stop/exec) | GPU stop | GPU drop | GPU final res | residual peak GPU / CPU |");
    println!("|---|---|---|---|---|---|---|---|---|---|");
    for r in &rows {
        println!(
            "| {} | {} | {} | {:.1} | {:.2e} | {}/{} | {} | {:.1} | {:.2e} | {:.2e} / {:.2e} |",
            r.mach, r.cpu.iters, r.cpu.stop_reason, r.cpu.drop_orders, r.cpu.residual.last().unwrap(), r.gpu.iters_stop, r.gpu.iters_exec, r.gpu.result.stop_reason,
            r.gpu.result.drop_orders, r.gpu.result.residual.last().unwrap(), r.gpu.result.residual_peak, r.cpu.residual_peak
        );
    }
    println!("\n### Timing (wall time of the runs to their stopping criterion: iteration loop incl. residual/state read-back and CD sampling every {SYNC} iterations, setup excluded)");
    println!("| M | CPU iters | GPU iters (exec) | CPU s | GPU s | CPU us/iter | GPU us/iter | speed-up (time to convergence) | speed-up (per iteration) |");
    println!("|---|---|---|---|---|---|---|---|---|");
    let (mut tc, mut tg) = (0.0, 0.0);
    for r in &rows {
        let (cu, gu) = (1e6 * r.cpu_s / r.cpu.iters as f64, r.gpu.us_per_iter());
        tc += r.cpu_s;
        tg += r.gpu.loop_seconds;
        println!("| {} | {} | {} | {:.1} | {:.2} | {:.0} | {:.0} | {:.1}x | {:.1}x |", r.mach, r.cpu.iters, r.gpu.iters_exec, r.cpu_s, r.gpu.loop_seconds, cu, gu, r.cpu_s / r.gpu.loop_seconds, cu / gu);
    }
    println!("| total | | | {tc:.1} | {tg:.2} | | | {:.1}x | |", tc / tg);
    let steady = with_bench.then(|| perf(&make_case(0.9)));
    if let Some((c, g, d, f)) = steady {
        println!("\n### Steady per-iteration cost (M = 0.9, 1000 iterations from the free stream, best of 3, sync every {SYNC}; history kernels on)");
        println!("| grid | cells | CPU f64 rayon us/iter | GPU-guarded extended us/iter | GPU-direct extended | guarded f32 | guarded extended speed-up vs CPU | direct extended speed-up | extended / f32 cost |");
        println!("|---|---|---|---|---|---|---|---|---|");
        println!("| L{level} {ni}x{nj} | {} | {c:.0} | {g:.1} | {d:.1} | {f:.1} | {:.1}x | {:.1}x | {:.2}x |", ni * nj, c / g, c / d, g / f);
        println!("\n### Time to the stopping criterion from the steady cost above (iterations x steady us/iter)");
        println!("| M | CPU iters | CPU s | GPU iters (exec) | GPU-guarded s | speed-up |");
        println!("|---|---|---|---|---|---|");
        let (mut tc, mut tg) = (0.0, 0.0);
        for r in &rows {
            let (a, b) = (r.cpu.iters as f64 * c * 1e-6, r.gpu.iters_exec as f64 * g * 1e-6);
            tc += a;
            tg += b;
            println!("| {} | {} | {a:.1} | {} | {b:.2} | {:.1}x |", r.mach, r.cpu.iters, r.gpu.iters_exec, a / b);
        }
        println!("| total | | {tc:.1} | | {tg:.2} | {:.1}x |", tc / tg);
    }

    let mut f = fs::File::create(out_dir.join(format!("sweep_{tag}.csv"))).unwrap();
    writeln!(f, "mach,cd_gpu,cd_cpu,cd_diff,cd_cell_gpu,cd_cell_cpu,cp_min_gpu,cp_min_cpu,max_abs_dcp_wall,cpu_iters,cpu_stop,cpu_drop_orders,gpu_iters_stop,gpu_iters_exec,gpu_stop,gpu_drop_orders,cpu_seconds,gpu_seconds,stoney_cd,tangent_cone_cd").unwrap();
    for r in &rows {
        let (g, c) = (&r.gpu.result, &r.cpu);
        writeln!(
            f,
            "{},{:.8},{:.8},{:.4e},{:.8},{:.8},{:.6},{:.6},{:.4e},{},{},{:.2},{},{},{},{:.2},{:.3},{:.3},{},{}",
            r.mach, g.cd_nose, c.cd_nose, g.cd_nose - c.cd_nose, g.cd_nose_cell, c.cd_nose_cell, g.cp_min, c.cp_min, r.max_dcp, c.iters, c.stop_reason, c.drop_orders,
            r.gpu.iters_stop, r.gpu.iters_exec, g.stop_reason, g.drop_orders, r.cpu_s, r.gpu.loop_seconds, r.stoney.map_or(String::new(), |v| format!("{v}")),
            if r.theory.is_nan() { String::new() } else { format!("{:.6}", r.theory) }
        )
        .unwrap();
    }
    if let Some((c, g, d, f32us)) = steady {
        let mut f = fs::File::create(out_dir.join(format!("steady_{tag}.csv"))).unwrap();
        writeln!(f, "ni,nj,cpu_us,gpu_guarded_ext_us,gpu_direct_ext_us,gpu_guarded_f32_us").unwrap();
        writeln!(f, "{ni},{nj},{c:.1},{g:.1},{d:.1},{f32us:.1}").unwrap();
    }
    println!("\nCSV output written to {}", out_dir.canonicalize().unwrap_or(out_dir).display());
}
