//! GPU vs CPU tangent-ogive-cylinder validation, mirroring the CPU `ogive` bin: M = 1.98, fineness-ratio-3
//! tangent ogive + cylinder (Perkins & Jorgensen, NACA RM A54H23, Table I(a)), coarse/medium/fine grids. The GPU solver
//! runs in extended precision (default, double-float).
//!
//! Usage: `cargo run --release --bin ogive_gpu [-- --quick] [--limiter minmod|vanalbada] [--chunk K] [--no-bench]
//!         [--mach M]`
//! `--quick` runs only the coarse and medium grids; `--chunk` is the number of iterations recorded between two
//! synchronisations (default 50, max 256). Output: `out/gpu_ogive/`.

use bowshock_cases::ogive::{perkins_jorgensen as pj, wall_cp_at, OgiveCase};
use bowshock_core::flux::Limiter;
use bowshock_gpu::{GpuSolver, Precision};
use bowshock_gpu_cases::ogive::{compare, run_cpu, run_gpu, Compare, OgiveRun};
use bowshock_reference::shock_expansion::{Ogive, SurfaceTheory};
use std::fs;
use std::io::Write;
use std::path::Path;
use std::time::Instant;

const GRID_NAMES: [&str; 3] = ["coarse", "medium", "fine"];

fn write_csvs(dir: &Path, tag: &str, gpu: &OgiveRun, cpu: &OgiveRun, th: &SurfaceTheory) {
    let mut f = fs::File::create(dir.join(format!("cp_{tag}.csv"))).unwrap();
    writeln!(f, "x_over_d,r_over_d,cp_gpu,cp_cpu,cp_firstcell_gpu,cp_firstcell_cpu,diff_cp,cp_shock_expansion").unwrap();
    for (g, c) in gpu.post.wall.iter().zip(&cpu.post.wall) {
        writeln!(f, "{:.8},{:.8},{:.8},{:.8},{:.8},{:.8},{:.4e},{:.8}", g.x, g.r, g.cp, c.cp, g.cp_cell, c.cp_cell, g.cp - c.cp, th.cp_shock_expansion(g.x))
            .unwrap();
    }
    let mut f = fs::File::create(dir.join(format!("residual_{tag}.csv"))).unwrap();
    writeln!(f, "iter,density_residual_gpu,density_residual_cpu").unwrap();
    let n = gpu.run.residual.len().max(cpu.run.residual.len());
    for i in 0..n {
        let g = gpu.run.residual.get(i).map_or(String::new(), |v| format!("{v:e}"));
        let c = cpu.run.residual.get(i).map_or(String::new(), |v| format!("{v:e}"));
        writeln!(f, "{},{g},{c}", i + 1).unwrap();
    }
}

/// Steady per-iteration cost on the case grid: 1000 iterations from the free stream (MUSCL from the first iteration),
/// best of 3, GPU with history kernels and a sync every `chunk` iterations. Returns `(cpu_us, ext_us, f32_us)`.
fn perf(case: &OgiveCase, chunk: usize) -> (f64, f64, f64) {
    let n = 1000usize;
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
    let ctx = luma::default_context().unwrap();
    let mut gpu = [0.0f64; 2];
    for (m, prec) in [Precision::Extended, Precision::F32].into_iter().enumerate() {
        let host = case.build_solver();
        let mut g = GpuSolver::new(&ctx, &host.grid, &host.cfg, prec).unwrap();
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
    (best_cpu, gpu[0], gpu[1])
}

struct Row {
    name: &'static str,
    ni: usize,
    nj: usize,
    cmp: Compare,
    gpu: OgiveRun,
    cpu: OgiveRun,
    perf: Option<(f64, f64, f64)>,
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let arg = |name: &str| args.iter().position(|a| a == name).and_then(|i| args.get(i + 1)).cloned();
    let quick = args.iter().any(|a| a == "--quick");
    let with_bench = !args.iter().any(|a| a == "--no-bench");
    let limiter = match arg("--limiter").as_deref() {
        Some("vanalbada") => Limiter::VanAlbada,
        _ => Limiter::Minmod,
    };
    let chunk: usize = arg("--chunk").and_then(|s| s.parse().ok()).unwrap_or(50);
    let mach: f64 = arg("--mach").and_then(|s| s.parse().ok()).unwrap_or(pj::MACH);
    let out_dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("..").join("out").join("gpu_ogive");
    fs::create_dir_all(&out_dir).expect("create out dir");
    let n_levels: u32 = if quick { 2 } else { 3 };

    let t0 = Instant::now();
    let _ctx = luma::default_context().expect("open GPU");
    println!("bowshock-gpu ogive validation: tangent ogive L/d = {}, M = {mach}, limiter = {limiter:?}, CFL = 0.5, f32 on GPU (Luma, extended precision) vs f64 CPU (bowshock-core)", pj::FINENESS);
    println!(
        "sync interval K = {chunk} iterations; CPU rayon threads = {}; GPU context creation {:.2} s",
        std::thread::available_parallelism().map_or(0, |n| n.get()),
        t0.elapsed().as_secs_f64()
    );

    if args.iter().any(|a| a == "--bench-only") {
        println!("\n### Steady per-iteration cost (1000 iterations from the free stream, best of 3, sync every K={chunk}; history kernels on)");
        println!("| grid | cells | CPU f64 rayon us/iter | GPU extended us/iter | GPU f32 | extended speed-up vs CPU | extended / f32 cost |");
        println!("|---|---|---|---|---|---|---|");
        for lv in 0..n_levels {
            let mut case = OgiveCase::level(mach, lv);
            case.limiter = limiter;
            let (ni, nj) = case.cells();
            let (c, g, f) = perf(&case, chunk);
            println!("| {} {ni}x{nj} | {} | {c:.0} | {g:.1} | {f:.1} | {:.1}x | {:.2}x |", GRID_NAMES[lv as usize], ni * nj, c / g, g / f);
        }
        return;
    }

    let body = Ogive { fineness: pj::FINENESS };
    let th = SurfaceTheory::new(mach, body);
    let (x0, x1) = (pj::X_OVER_D[0], pj::X_OVER_D[1]);
    let tip_extrap = pj::CP[0] + (pj::CP[0] - pj::CP[1]) / (x1 - x0) * x0;
    let cd_exp = pj::forebody_cd(&body, tip_extrap, pj::CP[7]);
    let cd_se = th.forebody_cd(SurfaceTheory::cp_shock_expansion, 4000);
    let cd_tc = th.forebody_cd(SurfaceTheory::cp_tangent_cone, 400);
    println!("reference: CD_forebody experiment {cd_exp:.4} (M = 1.98 only), shock-expansion {cd_se:.4}, tangent-cone {cd_tc:.4}");

    let mut rows: Vec<Row> = Vec::new();
    for lv in 0..n_levels {
        let mut case = OgiveCase::level(mach, lv);
        case.limiter = limiter;
        let (ni, nj) = case.cells();
        let name = GRID_NAMES[lv as usize];
        let cpu = run_cpu(&case);
        let gpu = run_gpu(&case, chunk, Precision::Extended).expect("GPU run (extended)");
        let fs = bowshock_core::gas::freestream(mach);
        let cmp = compare(&case, &fs, &gpu, &cpu);
        println!(
            "  {name:6} {ni}x{nj}: CPU {} it ({:.1} s, {}, {:.1} orders) | GPU {} it stop / {} exec ({:.2} s, {}, {:.1} orders)",
            cpu.run.iters_stop,
            cpu.run.loop_seconds,
            cpu.run.reason,
            cpu.run.drop_orders,
            gpu.run.iters_stop,
            gpu.run.iters_exec,
            gpu.run.loop_seconds,
            gpu.run.reason,
            gpu.run.drop_orders
        );
        println!(
            "         CD_nose GPU {:.5} CPU {:.5} (diff {:+.1e}%; first-cell {:+.1e}%); nose-mean Cp diff {:+.1e}%, max rel Cp diff (nose, |Cp| >= 0.05) {:.1e}%, max |dCp| (all wall) {:.1e}; field: max dp/pinf {:.2e}, rms {:.2e}, max drho/rho {:.2e}; min rho {:.4} min p {:.4}",
            gpu.post.cd_nose, cpu.post.cd_nose, 100.0 * cmp.cd_rel, 100.0 * cmp.cd_cell_rel, 100.0 * cmp.avg_cp_nose_rel, 100.0 * cmp.max_rel_cp_nose,
            cmp.max_abs_dcp_wall, cmp.max_dp_over_pinf, cmp.rms_dp_over_pinf, cmp.max_rel_rho, gpu.post.min_rho, gpu.post.min_p
        );
        write_csvs(&out_dir, &format!("M{mach}_{name}"), &gpu, &cpu, &th);
        let pf = with_bench.then(|| perf(&case, chunk));
        rows.push(Row { name, ni, nj, cmp, gpu, cpu, perf: pf });
    }

    // ---- tables ----
    println!("\n### Forebody pressure drag (base-area reference, nose 0 <= x <= 3d), M = {mach}");
    println!("| source | CD_forebody | vs CPU % |\n|---|---|---|");
    if mach == pj::MACH {
        println!("| experiment (Table I(a), tip extrapolated) | {cd_exp:.4} | - |");
    }
    println!("| shock-expansion | {cd_se:.4} | - |\n| tangent-cone | {cd_tc:.4} | - |");
    for r in &rows {
        println!("| CPU {} {}x{} | {:.5} | - |", r.name, r.ni, r.nj, r.cpu.post.cd_nose);
        println!("| GPU {} {}x{} | {:.5} | {:+.2e} |", r.name, r.ni, r.nj, r.gpu.post.cd_nose, 100.0 * r.cmp.cd_rel);
    }

    println!("\n### Accuracy: GPU (extended) vs CPU (f64)");
    println!("| grid | cells | CD GPU | CD CPU | CD diff % | CD first-cell diff % | nose mean Cp diff % | max rel Cp diff nose (abs(Cp) >= 0.05) % | max abs dCp (wall) | max dp/p_inf | rms dp/p_inf | max drho/rho |");
    println!("|---|---|---|---|---|---|---|---|---|---|---|---|");
    for r in &rows {
        println!(
            "| {} {}x{} | {} | {:.5} | {:.5} | {:+.1e} | {:+.1e} | {:+.1e} | {:.1e} | {:.1e} | {:.2e} | {:.2e} | {:.2e} |",
            r.name,
            r.ni,
            r.nj,
            r.ni * r.nj,
            r.gpu.post.cd_nose,
            r.cpu.post.cd_nose,
            100.0 * r.cmp.cd_rel,
            100.0 * r.cmp.cd_cell_rel,
            100.0 * r.cmp.avg_cp_nose_rel,
            100.0 * r.cmp.max_rel_cp_nose,
            r.cmp.max_abs_dcp_wall,
            r.cmp.max_dp_over_pinf,
            r.cmp.rms_dp_over_pinf,
            r.cmp.max_rel_rho
        );
    }

    if mach == pj::MACH {
        println!("\n### Cp at the 15 orifice stations of Perkins & Jorgensen (Table I(a)): experiment, shock-expansion, CPU, GPU");
        let mut hdr = String::from("| x/d | Cp exp | shock-exp. |");
        let mut sep = String::from("|---|---|---|");
        for r in &rows {
            hdr += &format!(" CPU {} | GPU {} |", r.name, r.name);
            sep += "---|---|";
        }
        println!("{hdr}\n{sep}");
        for (k, &x) in pj::X_OVER_D.iter().enumerate() {
            let mut line = format!("| {x:.2} | {:.3} | {:.3} |", pj::CP[k], th.cp_shock_expansion(x));
            for r in &rows {
                line += &format!(" {:.4} | {:.4} |", wall_cp_at(&r.cpu.post.wall, x), wall_cp_at(&r.gpu.post.wall, x));
            }
            println!("{line}");
        }
        let mut f = fs::File::create(out_dir.join("cp_compare_M1.98.csv")).unwrap();
        let mut header = String::from("x_over_d,cp_experiment,cp_shock_expansion");
        for r in &rows {
            header += &format!(",cp_cpu_{},cp_gpu_{}", r.name, r.name);
        }
        writeln!(f, "{header}").unwrap();
        for (k, &x) in pj::X_OVER_D.iter().enumerate() {
            let mut line = format!("{x},{},{:.5}", pj::CP[k], th.cp_shock_expansion(x));
            for r in &rows {
                line += &format!(",{:.5},{:.5}", wall_cp_at(&r.cpu.post.wall, x), wall_cp_at(&r.gpu.post.wall, x));
            }
            writeln!(f, "{line}").unwrap();
        }
    }

    println!("\n### Convergence (density residual drop from its peak after iteration 20; stop = criterion of `run_ogive`)");
    println!("| grid | CPU iters | CPU stop | CPU drop | CPU final res | GPU iters (stop/exec) | GPU stop | GPU drop | GPU final res |");
    println!("|---|---|---|---|---|---|---|---|---|");
    for r in &rows {
        println!(
            "| {} {}x{} | {} | {} | {:.1} | {:.2e} | {}/{} | {} | {:.1} | {:.2e} |",
            r.name,
            r.ni,
            r.nj,
            r.cpu.run.iters_stop,
            r.cpu.run.reason,
            r.cpu.run.drop_orders,
            r.cpu.run.residual.last().unwrap(),
            r.gpu.run.iters_stop,
            r.gpu.run.iters_exec,
            r.gpu.run.reason,
            r.gpu.run.drop_orders,
            r.gpu.run.residual[r.gpu.run.iters_stop - 1]
        );
    }

    println!("\n### Timing (wall time of the convergence runs: iteration loop incl. history readback every K={chunk}, setup excluded)");
    println!("| grid | CPU iters | CPU us/iter | GPU us/iter | CPU total s | GPU total s | speed-up | GPU setup s | GPU warm-up s |");
    println!("|---|---|---|---|---|---|---|---|---|");
    for r in &rows {
        println!(
            "| {} {}x{} | {} | {:.0} | {:.0} | {:.2} | {:.2} | {:.1}x | {:.3} | {:.3} |",
            r.name,
            r.ni,
            r.nj,
            r.cpu.run.iters_exec,
            r.cpu.run.us_per_iter(),
            r.gpu.run.us_per_iter(),
            r.cpu.run.loop_seconds,
            r.gpu.run.loop_seconds,
            r.cpu.run.loop_seconds / r.gpu.run.loop_seconds,
            r.gpu.run.setup_seconds,
            r.gpu.run.warmup_seconds
        );
    }
    if with_bench {
        println!("\n### Steady per-iteration cost (1000 iterations from the free stream, best of 3, sync every K={chunk}; history kernels on)");
        println!("| grid | cells | CPU f64 rayon us/iter | GPU extended us/iter | GPU f32 | extended speed-up vs CPU | extended / f32 cost |");
        println!("|---|---|---|---|---|---|---|");
        for r in &rows {
            let (c, g, f) = r.perf.unwrap();
            println!("| {} {}x{} | {} | {:.0} | {:.1} | {:.1} | {:.1}x | {:.2}x |", r.name, r.ni, r.nj, r.ni * r.nj, c, g, f, c / g, g / f);
        }
        println!("\n### Time to the stopping criterion (iterations to stop x steady us/iter)");
        println!("| grid | CPU iters | CPU s | GPU iters (stop) | GPU s | speed-up |");
        println!("|---|---|---|---|---|---|");
        for r in &rows {
            let (c, g, _) = r.perf.unwrap();
            let (tc, tg) = (r.cpu.run.iters_stop as f64 * c * 1e-6, r.gpu.run.iters_stop as f64 * g * 1e-6);
            println!("| {} {}x{} | {} | {:.2} | {} | {:.2} | {:.1}x |", r.name, r.ni, r.nj, r.cpu.run.iters_stop, tc, r.gpu.run.iters_stop, tg, tc / tg);
        }
    }

    let mut f = fs::File::create(out_dir.join("summary.csv")).unwrap();
    writeln!(
        f,
        "mach,grid,ni,nj,cd_gpu,cd_cpu,cd_rel_diff,cd_cell_rel_diff,nose_mean_cp_rel_diff,max_rel_cp_diff_nose,max_abs_dcp_wall,max_dp_over_pinf,rms_dp_over_pinf,max_rel_rho,cpu_iters,cpu_reason,cpu_drop_orders,gpu_iters_stop,gpu_iters_exec,gpu_reason,gpu_drop_orders,cpu_loop_s,gpu_loop_s,cpu_steady_us,gpu_ext_steady_us,gpu_f32_steady_us"
    )
    .unwrap();
    for r in &rows {
        let (c, g, f32us) = r.perf.unwrap_or((f64::NAN, f64::NAN, f64::NAN));
        writeln!(
            f,
            "{mach},{},{},{},{:.8},{:.8},{:.4e},{:.4e},{:.4e},{:.4e},{:.4e},{:.4e},{:.4e},{:.4e},{},{},{:.2},{},{},{},{:.2},{:.3},{:.3},{:.1},{:.1},{:.1}",
            r.name,
            r.ni,
            r.nj,
            r.gpu.post.cd_nose,
            r.cpu.post.cd_nose,
            r.cmp.cd_rel,
            r.cmp.cd_cell_rel,
            r.cmp.avg_cp_nose_rel,
            r.cmp.max_rel_cp_nose,
            r.cmp.max_abs_dcp_wall,
            r.cmp.max_dp_over_pinf,
            r.cmp.rms_dp_over_pinf,
            r.cmp.max_rel_rho,
            r.cpu.run.iters_stop,
            r.cpu.run.reason,
            r.cpu.run.drop_orders,
            r.gpu.run.iters_stop,
            r.gpu.run.iters_exec,
            r.gpu.run.reason,
            r.gpu.run.drop_orders,
            r.cpu.run.loop_seconds,
            r.gpu.run.loop_seconds,
            c,
            g,
            f32us
        )
        .unwrap();
    }
    println!("\nCSV output written to {}", out_dir.canonicalize().unwrap_or(out_dir).display());
}
