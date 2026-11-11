//! GPU vs CPU subsonic Mach sweep, mirroring the CPU `subsonic` bin: nose-cylinder body with and without a flat blunt base on the
//! large-domain grid with characteristic far field; line-implicit convergence scheme, low-Mach flux correction, base block / wake. Each
//! case is run by the f64 CPU solver and by the GPU solver (guarded formulation, extended precision) with the same procedure and stopping
//! rules; drag breakdown, base pressure, wall Cp and the cost of both are compared.
//!
//! Usage: `cargo run --release --bin subsonic_gpu -- [--body ogive|cone|karman|lvhaack|sqrt|parabola] [--level 0|1|2] [--quick]
//!   [--mach 0.1,0.2,...] [--base on|off|both] [--warm] [--lowmach on|off] [--limiter vanalbada|minmod] [--scheme implicit|explicit]
//!   [--max-iter N] [--drop <orders>] [--cyl <diameters>] [--domain <body lengths>] [--tag name] [--no-cpu] [--no-bench] [--bench-only]
//!   [--validate]`
//! Defaults are those of the CPU bin (`--quick` = grid level 0, default level 1; M = 0.1 ... 0.8; both base settings; cold start for every
//! Mach number; implicit scheme, van Albada, low-Mach correction on). `--no-cpu` skips the CPU runs (GPU results only); `--validate` runs the
//! NASA TM X-2439 (Fox 1971) configurations on the GPU and prints the statistics of the CPU bin. Output: `out/gpu_subsonic/`.

use bowshock_cases::body_grid::Body;
use bowshock_cases::subsonic::{fox_case, fox_tm_x2439 as fox, run_on, wall_cp_at, SubsonicCase, SubsonicResult};
use bowshock_cases::transonic::named_body;
use bowshock_core::flux::Limiter;
use bowshock_core::implicit::ImplicitParams;
use bowshock_core::solver::Scheme;
use bowshock_gpu::{Mode, Precision};
use bowshock_gpu_cases::subsonic::{GpuSubsonic, GpuSubsonicRun, SYNC};
use std::fmt::Write as _;
use std::fs;
use std::io::Write;
use std::path::{Path, PathBuf};
use std::time::Instant;

fn arg<'a>(args: &'a [String], key: &str) -> Option<&'a str> {
    args.iter().position(|a| a == key).and_then(|i| args.get(i + 1)).map(String::as_str)
}

struct Opts {
    level: u32,
    limiter: Limiter,
    low_mach: bool,
    scheme: Scheme,
    max_iter: Option<usize>,
    drop: Option<f64>,
    domain: Option<f64>,
}

impl Opts {
    fn parse(args: &[String]) -> Opts {
        let quick = args.iter().any(|a| a == "--quick");
        let explicit = arg(args, "--scheme") == Some("explicit");
        Opts {
            level: arg(args, "--level").and_then(|v| v.parse().ok()).unwrap_or(if quick { 0 } else { 1 }),
            limiter: match arg(args, "--limiter") {
                Some("minmod") => Limiter::Minmod,
                Some("vanalbada") => Limiter::VanAlbada,
                _ if explicit => Limiter::Minmod,
                _ => Limiter::VanAlbada,
            },
            low_mach: arg(args, "--lowmach") != Some("off"),
            scheme: if explicit { Scheme::Explicit } else { Scheme::Implicit(ImplicitParams::DEFAULT) },
            max_iter: arg(args, "--max-iter").and_then(|v| v.parse().ok()),
            drop: arg(args, "--drop").and_then(|v| v.parse().ok()),
            domain: arg(args, "--domain").and_then(|v| v.parse().ok()),
        }
    }
    fn apply(&self, mut c: SubsonicCase) -> SubsonicCase {
        c.limiter = self.limiter;
        c.low_mach = self.low_mach;
        c.scheme = self.scheme;
        if let Some(n) = self.max_iter {
            c.max_iter = n;
        } else if self.scheme == Scheme::Explicit {
            c.max_iter = 60_000;
            c.first_order_iters = 300;
        }
        if let Some(d) = self.drop {
            c.drop_orders = d;
        }
        if let Some(d) = self.domain {
            c = c.with_domain(d, 0.5 * d);
        }
        c
    }
}

struct Row {
    mach: f64,
    base: bool,
    cpu: Option<SubsonicResult>,
    cpu_s: f64,
    gpu: GpuSubsonicRun,
    /// max over wall faces of |Cp_gpu - Cp_cpu|.
    max_dcp: f64,
}

const DCP_TOL: f64 = 1e-4;
const CPB_MEAN_RTOL: f64 = 1e-2;

/// GPU-vs-CPU pass/fail: mean base Cp for an unsettled wake (CPU hit `max_iter`), wall Cp max difference otherwise.
fn check(base: bool, gpu: &SubsonicResult, cpu: Option<&SubsonicResult>, max_dcp: f64) -> String {
    let Some(c) = cpu else {
        return "-".to_string();
    };
    if base && c.stop_reason == "max iterations" {
        let d = (gpu.cp_base_mean - c.cp_base_mean).abs() / c.cp_base_mean.abs();
        let verdict = if d <= CPB_MEAN_RTOL { "ok" } else { "FAIL" };
        format!("{verdict} (Cp_b mean, d={d:.1e})")
    } else if max_dcp <= DCP_TOL {
        "ok (dCp)".to_string()
    } else {
        "FAIL (dCp)".to_string()
    }
}

fn write_case_files(dir: &Path, name: &str, r: &Row) {
    let g = &r.gpu.result;
    let mut f = fs::File::create(dir.join(format!("cp_{name}.csv"))).unwrap();
    writeln!(f, "x_over_d,r_over_d,cp_gpu,cp_cpu,cp_firstcell_gpu,cp_firstcell_cpu,diff_cp").unwrap();
    for (k, w) in g.wall.iter().enumerate() {
        let c = r.cpu.as_ref().map(|c| &c.wall[k]);
        writeln!(
            f,
            "{:.8},{:.8},{:.8},{},{:.8},{},{}",
            w.x,
            w.r,
            w.cp,
            c.map_or(String::new(), |c| format!("{:.8}", c.cp)),
            w.cp_cell,
            c.map_or(String::new(), |c| format!("{:.8}", c.cp_cell)),
            c.map_or(String::new(), |c| format!("{:.4e}", w.cp - c.cp))
        )
        .unwrap();
    }
    if !g.base_wall.is_empty() {
        let mut f = fs::File::create(dir.join(format!("base_{name}.csv"))).unwrap();
        writeln!(f, "r0,r1,cp_gpu,cp_cpu").unwrap();
        for (k, b) in g.base_wall.iter().enumerate() {
            writeln!(f, "{:.6},{:.6},{:.6},{}", b.r0, b.r1, b.cp, r.cpu.as_ref().map_or(String::new(), |c| format!("{:.6}", c.base_wall[k].cp))).unwrap();
        }
    }
    let mut f = fs::File::create(dir.join(format!("residual_{name}.csv"))).unwrap();
    writeln!(f, "iter,density_residual_gpu,density_residual_cpu").unwrap();
    let n = g.residual.len().max(r.cpu.as_ref().map_or(0, |c| c.residual.len()));
    for i in 0..n {
        let a = g.residual.get(i).map_or(String::new(), |v| format!("{v:e}"));
        let b = r.cpu.as_ref().and_then(|c| c.residual.get(i)).map_or(String::new(), |v| format!("{v:e}"));
        writeln!(f, "{},{a},{b}", i + 1).unwrap();
    }
}

/// Steady per-iteration cost on the case grid at `case.mach`: 1000 iterations from the free stream (MUSCL from the first iteration),
/// best of 3, GPU with history kernels and a sync every `SYNC` iterations. Returns `(cpu_us, gpu_guarded_ext_us, gpu_guarded_f32_us)`
/// (CPU omitted, NaN, with `cpu = false`). The CPU ramps its CFL during the warm-up as the implicit scheme does at the start of a run.
fn perf(case: &SubsonicCase, cpu: bool) -> (f64, f64, f64) {
    let n = 1000usize;
    let mut case = case.clone();
    case.first_order_iters = 0;
    let mut best_cpu = f64::NAN;
    if cpu {
        best_cpu = f64::INFINITY;
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
    }
    let mut gpu = [0.0f64; 2];
    for (m, prec) in [Precision::Extended, Precision::F32].into_iter().enumerate() {
        let mut gs = GpuSubsonic::new(&case, Mode::Guarded, prec).unwrap();
        gs.gpu.run(1).unwrap();
        let mut best = f64::INFINITY;
        for _ in 0..3 {
            gs.gpu.reset().unwrap();
            gs.gpu.context().sync().unwrap();
            let t0 = Instant::now();
            for _ in 0..n / SYNC {
                gs.gpu.run(SYNC).unwrap();
            }
            best = best.min(1e6 * t0.elapsed().as_secs_f64() / ((n / SYNC) * SYNC) as f64);
        }
        gpu[m] = best;
    }
    (best_cpu, gpu[0], gpu[1])
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let out_dir: PathBuf = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("..").join("out").join("gpu_subsonic");
    fs::create_dir_all(&out_dir).expect("create out dir");
    let opts = Opts::parse(&args);
    let t0 = Instant::now();
    let _ctx = luma::default_context().expect("open GPU");
    println!(
        "bowshock-gpu subsonic: grid level {}, limiter {:?}, low-Mach {}, scheme {}; GPU guarded, extended precision (Luma, f32 storage) vs f64 CPU (bowshock-core)",
        opts.level,
        opts.limiter,
        opts.low_mach,
        if matches!(opts.scheme, Scheme::Explicit) { "explicit SSP-RK3" } else { "line-implicit" }
    );
    println!(
        "GPU sync interval = {SYNC} iterations; CPU rayon threads = {}; GPU context creation {:.2} s",
        std::thread::available_parallelism().map_or(0, |n| n.get()),
        t0.elapsed().as_secs_f64()
    );
    if args.iter().any(|a| a == "--validate") {
        validate(&out_dir, &opts, &args);
    } else {
        sweep(&out_dir, &opts, &args);
    }
}

fn sweep(out_dir: &Path, opts: &Opts, args: &[String]) {
    let with_cpu = !args.iter().any(|a| a == "--no-cpu");
    let with_bench = !args.iter().any(|a| a == "--no-bench");
    let body_name = arg(args, "--body").unwrap_or("ogive");
    let (body, _): (Body, _) = named_body(body_name).unwrap_or_else(|| panic!("unknown body '{body_name}'"));
    let machs: Vec<f64> = match arg(args, "--mach") {
        Some(l) => l.split(',').map(|v| v.parse().expect("Mach list")).collect(),
        None => vec![0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8],
    };
    let bases: Vec<bool> = match arg(args, "--base") {
        Some("on") => vec![true],
        Some("off") => vec![false],
        _ => vec![false, true],
    };
    let warm = args.iter().any(|a| a == "--warm");
    let cyl: f64 = arg(args, "--cyl").and_then(|v| v.parse().ok()).unwrap_or(6.0);
    let tag = arg(args, "--tag").map(String::from).unwrap_or_else(|| {
        format!("{body_name}_L{}{}{}", opts.level, if matches!(opts.scheme, Scheme::Explicit) { "_exp" } else { "" }, if warm { "_warm" } else { "" })
    });
    let make = |m: f64, base: bool| {
        let mut bb = SubsonicCase::new(m, body.nose, opts.level, base);
        bb.body.cyl_length = cyl;
        bb.grid_base.body.cyl_length = cyl;
        bb.grid_sting.cyl_length = cyl;
        opts.apply(bb)
    };
    if args.iter().any(|a| a == "--bench-only") {
        for &base in &bases {
            let c = make(0.5, base);
            let (ni, nj) = c.cells();
            let (cu, g, f) = perf(&c, with_cpu);
            println!("base {base}: {ni}x{nj} ({} cells): CPU {cu:.0} us/iter, GPU extended {g:.1} us/iter ({:.1}x), GPU f32 {f:.1} us/iter", ni * nj, cu / g);
        }
        return;
    }
    println!("body {body_name} (nose l/d {:.2}), cylinder {cyl} d, {} sweep", body.nose_length(), if warm { "warm-start" } else { "cold-start" });
    let mut rows: Vec<Row> = Vec::new();
    for &base in &bases {
        let c0 = make(machs[0], base);
        let (ni, nj) = c0.cells();
        let mut gs = GpuSubsonic::new(&c0, Mode::Guarded, Precision::Extended).expect("GPU solver");
        println!("\nbase = {base}: grid {ni}x{nj} ({} cells), GPU setup (kernel build + upload) {:.3} s", ni * nj, gs.setup_seconds);
        let mut cpu_solver = c0.build_solver();
        let (mut r_ref_cpu, mut r_ref_gpu): (Option<f64>, Option<f64>) = (None, None);
        for (k, &m) in machs.iter().enumerate() {
            let case = make(m, base);
            let (mut cpu, mut cpu_s) = (None, 0.0);
            if with_cpu {
                if !warm || k == 0 {
                    cpu_solver = case.build_solver();
                }
                let t0 = Instant::now();
                cpu = Some(run_on(&case, &mut cpu_solver, warm && k > 0, if warm { r_ref_cpu } else { None }));
                cpu_s = t0.elapsed().as_secs_f64();
            }
            let mut gcase = case.clone();
            if let Some(c) = &cpu {
                gcase.min_iter = c.iters;
                gcase.max_iter = c.iters;
            }
            let gpu = gs.run_on(&gcase, warm && k > 0, if warm { r_ref_gpu } else { None }).expect("GPU run");
            if warm && k == 0 {
                r_ref_cpu = cpu.as_ref().map(|c| c.residual_peak);
                r_ref_gpu = Some(gpu.result.residual_peak);
            }
            let max_dcp = cpu.as_ref().map_or(f64::NAN, |c| gpu.result.wall.iter().zip(&c.wall).fold(0.0f64, |mx, (a, b)| mx.max((a.cp - b.cp).abs())));
            let row = Row { mach: m, base, cpu, cpu_s, gpu, max_dcp };
            let g = &row.gpu.result;
            let cpu_txt = row
                .cpu
                .as_ref()
                .map_or(String::new(), |c| format!("CPU {:>6} it ({:6.1} s, {}, {:.1} orders) | ", c.iters, row.cpu_s, c.stop_reason, c.drop_orders));
            println!(
                "M = {m:<4} {cpu_txt}GPU {:>6} it stop / {} exec ({:6.2} s, {}, {:.1} orders) | CD nose {:+.5} cyl {:+.5} base {:+.5} total {:+.5}, Cp_b {:+.4}; vs CPU: dCD_total {:+.1e}, dCp_b {:+.1e}, max |dCp| {:.1e}",
                row.gpu.iters_stop, row.gpu.iters_exec, row.gpu.loop_seconds, g.stop_reason, g.drop_orders, g.cd_nose, g.cd_cyl, g.cd_base, g.cd_total, g.cp_base_mean,
                row.cpu.as_ref().map_or(f64::NAN, |c| g.cd_total - c.cd_total), row.cpu.as_ref().map_or(f64::NAN, |c| g.cp_base_mean - c.cp_base_mean), row.max_dcp
            );
            write_case_files(out_dir, &format!("{tag}_M{m}_{}", if base { "base" } else { "nobase" }), &row);
            rows.push(row);
        }
    }

    // ---- tables ----
    println!("\n### {body_name}, grid level {}, {}: GPU (guarded, extended) vs CPU (f64)", opts.level, if warm { "warm starts" } else { "cold starts" });
    println!("| M | base | CD nose GPU | CD nose CPU | CD base GPU | CD base CPU | CD total GPU | CD total CPU | Cp_b GPU | Cp_b CPU | Cp_min GPU | Cp_min CPU | max abs dCp (wall) | CD base corr. | CD total corr. | check |");
    println!("|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|");
    let f = |v: Option<f64>, p: usize| v.map_or("-".to_string(), |v| format!("{v:+.p$}"));
    for r in &rows {
        let (g, c) = (&r.gpu.result, r.cpu.as_ref());
        println!(
            "| {} | {} | {:+.5} | {} | {:+.5} | {} | {:+.5} | {} | {} | {} | {:.4} | {} | {:.1e} | {} | {} | {} |",
            r.mach,
            if r.base { "on" } else { "off" },
            g.cd_nose,
            f(c.map(|c| c.cd_nose), 5),
            g.cd_base,
            f(c.map(|c| c.cd_base), 5),
            g.cd_total,
            f(c.map(|c| c.cd_total), 5),
            if r.base { format!("{:+.4}", g.cp_base_mean) } else { "-".into() },
            if r.base { f(c.map(|c| c.cp_base_mean), 4) } else { "-".into() },
            g.cp_min,
            c.map_or("-".to_string(), |c| format!("{:.4}", c.cp_min)),
            r.max_dcp,
            f(Some(g.cd_base_model).filter(|v| !v.is_nan()), 5),
            f(Some(g.cd_total_model).filter(|v| !v.is_nan()), 5),
            check(r.base, g, c, r.max_dcp)
        );
    }
    println!("\n### Convergence");
    println!("| M | base | CPU iters | CPU stop | CPU drop | GPU iters (stop/exec) | GPU stop | GPU drop | GPU final res | residual peak GPU / CPU |");
    println!("|---|---|---|---|---|---|---|---|---|---|");
    for r in &rows {
        let g = &r.gpu.result;
        println!(
            "| {} | {} | {} | {} | {} | {}/{} | {} | {:.1} | {:.2e} | {:.2e} / {} |",
            r.mach,
            if r.base { "on" } else { "off" },
            r.cpu.as_ref().map_or("-".into(), |c| c.iters.to_string()),
            r.cpu.as_ref().map_or("-", |c| c.stop_reason),
            r.cpu.as_ref().map_or("-".into(), |c| format!("{:.1}", c.drop_orders)),
            r.gpu.iters_stop,
            r.gpu.iters_exec,
            g.stop_reason,
            g.drop_orders,
            g.residual.last().unwrap(),
            g.residual_peak,
            r.cpu.as_ref().map_or("-".into(), |c| format!("{:.2e}", c.residual_peak))
        );
    }
    println!(
        "\n### Timing (wall time of the runs to their stopping criterion: iteration loop incl. residual read-back{}, setup excluded)",
        if bases.contains(&true) { " and base-pressure sampling every 50 iterations" } else { "" }
    );
    println!(
        "| M | base | CPU iters | GPU iters (exec) | CPU s | GPU s | CPU us/iter | GPU us/iter | speed-up (time to convergence) | speed-up (per iteration) |"
    );
    println!("|---|---|---|---|---|---|---|---|---|---|");
    let (mut tc, mut tg) = (0.0, 0.0);
    for r in &rows {
        let gu = r.gpu.us_per_iter();
        tg += r.gpu.loop_seconds;
        match &r.cpu {
            Some(c) => {
                let cu = 1e6 * r.cpu_s / c.iters as f64;
                tc += r.cpu_s;
                println!(
                    "| {} | {} | {} | {} | {:.1} | {:.2} | {:.0} | {:.0} | {:.1}x | {:.1}x |",
                    r.mach,
                    if r.base { "on" } else { "off" },
                    c.iters,
                    r.gpu.iters_exec,
                    r.cpu_s,
                    r.gpu.loop_seconds,
                    cu,
                    gu,
                    r.cpu_s / r.gpu.loop_seconds,
                    cu / gu
                );
            }
            None => println!(
                "| {} | {} | - | {} | - | {:.2} | - | {:.0} | - | - |",
                r.mach,
                if r.base { "on" } else { "off" },
                r.gpu.iters_exec,
                r.gpu.loop_seconds,
                gu
            ),
        }
    }
    println!("| total | | | | {tc:.1} | {tg:.2} | | | {:.1}x | |", tc / tg);
    let mut steady = Vec::new();
    if with_bench {
        println!("\n### Steady per-iteration cost (M = 0.5, 1000 iterations from the free stream, best of 3, sync every {SYNC}; history kernels on)");
        println!(
            "| base | grid | cells | CPU f64 rayon us/iter | GPU-guarded extended us/iter | guarded f32 | extended speed-up vs CPU | extended / f32 cost |"
        );
        println!("|---|---|---|---|---|---|---|---|");
        for &base in &bases {
            let c = make(0.5, base);
            let (ni, nj) = c.cells();
            let (cu, g, f32us) = perf(&c, with_cpu);
            println!("| {} | {ni}x{nj} | {} | {cu:.0} | {g:.1} | {f32us:.1} | {:.1}x | {:.2}x |", if base { "on" } else { "off" }, ni * nj, cu / g, g / f32us);
            steady.push((base, ni, nj, cu, g, f32us));
        }
    }

    let mut f = fs::File::create(out_dir.join(format!("sweep_{tag}.csv"))).unwrap();
    writeln!(f, "mach,base,gpu_iters_stop,gpu_iters_exec,gpu_stop,gpu_drop_orders,gpu_seconds,cpu_iters,cpu_stop,cpu_drop_orders,cpu_seconds,cd_nose_gpu,cd_nose_cpu,cd_cyl_gpu,cd_cyl_cpu,cd_base_gpu,cd_base_cpu,cd_total_gpu,cd_total_cpu,cp_base_gpu,cp_base_cpu,cp_min_gpu,cp_min_cpu,max_abs_dcp_wall,cd_base_model,cd_total_model,check").unwrap();
    for r in &rows {
        let (g, c) = (&r.gpu.result, r.cpu.as_ref());
        let o = |v: Option<f64>| v.map_or(String::new(), |v| format!("{v:.8}"));
        let ck = check(r.base, g, c, r.max_dcp);
        let ck = if ck == "-" { "" } else { ck.split(' ').next().unwrap() };
        writeln!(
            f,
            "{},{},{},{},{},{:.2},{:.3},{},{},{},{:.3},{:.8},{},{:.8},{},{:.8},{},{:.8},{},{:.8},{},{:.6},{},{:.4e},{},{},{}",
            r.mach,
            r.base,
            r.gpu.iters_stop,
            r.gpu.iters_exec,
            g.stop_reason,
            g.drop_orders,
            r.gpu.loop_seconds,
            c.map_or(String::new(), |c| c.iters.to_string()),
            c.map_or("", |c| c.stop_reason),
            c.map_or(String::new(), |c| format!("{:.2}", c.drop_orders)),
            r.cpu_s,
            g.cd_nose,
            o(c.map(|c| c.cd_nose)),
            g.cd_cyl,
            o(c.map(|c| c.cd_cyl)),
            g.cd_base,
            o(c.map(|c| c.cd_base)),
            g.cd_total,
            o(c.map(|c| c.cd_total)),
            g.cp_base_mean,
            o(c.map(|c| c.cp_base_mean)),
            g.cp_min,
            o(c.map(|c| c.cp_min)),
            r.max_dcp,
            o(Some(g.cd_base_model).filter(|v| !v.is_nan())),
            o(Some(g.cd_total_model).filter(|v| !v.is_nan())),
            ck
        )
        .unwrap();
    }
    if !steady.is_empty() {
        let mut f = fs::File::create(out_dir.join(format!("steady_{tag}.csv"))).unwrap();
        writeln!(f, "base,ni,nj,cpu_us,gpu_guarded_ext_us,gpu_guarded_f32_us").unwrap();
        for (b, ni, nj, c, g, f32us) in steady {
            writeln!(f, "{b},{ni},{nj},{c:.1},{g:.1},{f32us:.1}").unwrap();
        }
    }
    println!("\nCSV output written to {}", out_dir.canonicalize().unwrap_or(out_dir.to_path_buf()).display());
}

/// NASA TM X-2439 configurations on the GPU (the CPU bin's `--validate`): wall Cp statistics against the digitised data.
fn validate(out_dir: &Path, opts: &Opts, args: &[String]) {
    let with_cpu = !args.iter().any(|a| a == "--no-cpu");
    let mut md = String::new();
    writeln!(md, "### NASA TM X-2439 (Fox 1971), sting-mounted nose-cylinder-boattail bodies, grid level {}, GPU (extended) vs CPU\n", opts.level).unwrap();
    writeln!(md, "| config | M | GPU iters | GPU time [s] | CPU iters | CPU time [s] | RMS fore+cyl (GPU) | RMS fore+cyl (CPU) | base Cp GPU | base Cp CPU | data base Cp | CD total GPU | CD total CPU | max abs dCp GPU-CPU | CD base corr. | CD total corr. | check |").unwrap();
    writeln!(md, "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|").unwrap();
    for cone in [false, true] {
        for (k, &m) in fox::MACH.iter().enumerate() {
            let case = opts.apply(fox_case(cone, m, opts.level));
            let (cpu, cpu_s) = if with_cpu {
                let mut s = case.build_solver();
                let t0 = Instant::now();
                let r = run_on(&case, &mut s, false, None);
                (Some(r), t0.elapsed().as_secs_f64())
            } else {
                (None, 0.0)
            };
            let mut gs = GpuSubsonic::new(&case, Mode::Guarded, Precision::Extended).unwrap();
            let mut gcase = case.clone();
            if let Some(c) = &cpu {
                gcase.min_iter = c.iters;
                gcase.max_iter = c.iters;
            }
            let gpu = gs.run_on(&gcase, false, None).unwrap();
            let (xs, cps): (&[f64], &[f64]) = match (cone, k) {
                (false, 0) => (&fox::OGIVE_CYL_X_D, &fox::OGIVE_CYL_CP_M04_MEAN),
                (false, 1) => (&fox::OGIVE_CYL_X_D, &fox::OGIVE_CYL_CP_M06_MEAN),
                (false, _) => (&fox::OGIVE_CYL_X_D, &fox::OGIVE_CYL_CP_M08_MEAN),
                (true, 0) => (&fox::CONE_CYL_X_D, &fox::CONE_CYL_CP_M04_MEAN),
                (true, 1) => (&fox::CONE_CYL_X_D, &fox::CONE_CYL_CP_M06_MEAN),
                (true, _) => (&fox::CONE_CYL_X_D, &fox::CONE_CYL_CP_M08_MEAN),
            };
            let x_fore = if cone { 4.2 } else { 3.4 };
            let rms = |r: &SubsonicResult| {
                let (mut s, mut n) = (0.0f64, 0usize);
                for (x, d) in xs.iter().zip(cps.iter()) {
                    if *x <= x_fore {
                        if let Some(c) = wall_cp_at(&r.wall, *x) {
                            s += (c - d).powi(2);
                            n += 1;
                        }
                    }
                }
                (s / n.max(1) as f64).sqrt()
            };
            let g = &gpu.result;
            let data_base = if cone { fox::BASE_CP_CONE[k] } else { fox::BASE_CP_OGIVE[k] };
            let cfg = if cone { "2 (cone)" } else { "4 (ogive)" };
            let dcp = cpu.as_ref().map_or(f64::NAN, |c| g.wall.iter().zip(&c.wall).fold(0.0f64, |mx, (a, b)| mx.max((a.cp - b.cp).abs())));
            let chk = check(case.base, g, cpu.as_ref(), dcp);
            let corr = |v: f64| if v.is_nan() { "-".to_string() } else { format!("{v:+.4}") };
            println!(
                "config {cfg} M={m}: GPU {} it ({:.1} s, {}, {:.1} orders), Cp RMS (x <= {x_fore}) {:.4}, base Cp {:+.4} (data {:+.4}), CD total {:+.4}{}; check {chk}",
                gpu.iters_stop, gpu.loop_seconds, g.stop_reason, g.drop_orders, rms(g), g.cp_base_mean, data_base, g.cd_total,
                cpu.as_ref().map_or(String::new(), |c| format!(" | CPU {} it ({cpu_s:.1} s): Cp RMS {:.4}, base Cp {:+.4}, CD total {:+.4}; max |dCp| {dcp:.1e}", c.iters, rms(c), c.cp_base_mean, c.cd_total))
            );
            writeln!(
                md,
                "| {cfg} | {m} | {} | {:.1} | {} | {:.1} | {:.4} | {} | {:+.4} | {} | {:+.4} | {:+.4} | {} | {:.1e} | {} | {} | {} |",
                gpu.iters_stop,
                gpu.loop_seconds,
                cpu.as_ref().map_or("-".into(), |c| c.iters.to_string()),
                cpu_s,
                rms(g),
                cpu.as_ref().map_or("-".into(), |c| format!("{:.4}", rms(c))),
                g.cp_base_mean,
                cpu.as_ref().map_or("-".into(), |c| format!("{:+.4}", c.cp_base_mean)),
                data_base,
                g.cd_total,
                cpu.as_ref().map_or("-".into(), |c| format!("{:+.4}", c.cd_total)),
                dcp,
                corr(g.cd_base_model),
                corr(g.cd_total_model),
                chk
            )
            .unwrap();
        }
    }
    fs::write(out_dir.join("tables_validate.md"), &md).unwrap();
    println!("\n{md}");
}
