//! Cone validation driver: M=2 flow over 10 and 15 degree cones vs Taylor-Maccoll.
//!
//! Usage: `cargo run --release --bin cone [-- --quick] [--limiter minmod|vanalbada] [--mach M]`
//! `--quick` runs only the coarse and medium grids.

use bowshock_cases::cone::{run_cone, ConeCase};
use bowshock_core::flux::Limiter;
use bowshock_reference::taylor_maccoll::solve_cone;
use std::fs;
use std::io::Write;
use std::path::PathBuf;
use std::time::Instant;

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let quick = args.iter().any(|a| a == "--quick");
    let arg = |name: &str| args.iter().position(|a| a == name).and_then(|i| args.get(i + 1)).cloned();
    let limiter = match arg("--limiter").as_deref() {
        Some("vanalbada") => Limiter::VanAlbada,
        _ => Limiter::Minmod,
    };
    let mach: f64 = arg("--mach").and_then(|s| s.parse().ok()).unwrap_or(2.0);
    let out_dir: PathBuf = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("..").join("out");
    fs::create_dir_all(&out_dir).expect("create out dir");

    let mut grids = vec![("coarse", 100, 50), ("medium", 200, 100)];
    if !quick {
        grids.push(("fine", 400, 200));
    }

    println!("bowshock-core cone validation: M = {mach}, gamma = 1.4, limiter = {limiter:?}, CFL = 0.5, SSP-RK3 + HLLC/Rusanov shock blend");
    println!("threads (rayon): {}", rayon::current_num_threads());
    let mut rows = Vec::new();
    for &theta in &[10.0f64, 15.0] {
        let tm = solve_cone(mach, theta.to_radians()).expect("attached conical shock");
        println!(
            "\n=== {theta} deg cone, Taylor-Maccoll: beta = {:.4} deg, p_c/p_inf = {:.5}, Cp = {:.5}, M_c = {:.4}",
            tm.beta.to_degrees(),
            tm.p_ratio_surface,
            tm.cp_surface,
            tm.mach_surface
        );
        for &(name, ni, nj) in &grids {
            let mut case = ConeCase::new(mach, theta, ni, nj);
            case.limiter = limiter;
            let t0 = Instant::now();
            let res = run_cone(&case, &tm);
            let dt = t0.elapsed().as_secs_f64();
            let lim = match limiter { Limiter::VanAlbada => "vanalbada", _ => "minmod" };
            let tag = format!("{}deg_{}x{}_{}", theta as i32, ni, nj, lim);

            let mut f = fs::File::create(out_dir.join(format!("cp_{tag}.csv"))).unwrap();
            writeln!(f, "x_over_L,p_over_pinf,cp_wall_extrapolated,cp_first_cell,cp_taylor_maccoll").unwrap();
            for r in &res.surface {
                writeln!(f, "{:.8},{:.8},{:.8},{:.8},{:.8}", r.0, r.1, r.2, r.3, tm.cp_surface).unwrap();
            }
            let mut f = fs::File::create(out_dir.join(format!("residual_{tag}.csv"))).unwrap();
            writeln!(f, "iter,density_residual_l2").unwrap();
            for (i, r) in res.residual.iter().enumerate() {
                writeln!(f, "{},{:e}", i + 1, r).unwrap();
            }

            let hist = |k: usize| res.residual.get(k.min(res.residual.len() - 1)).copied().unwrap_or(f64::NAN);
            println!(
                "  {name:6} {ni:3}x{nj:3}: {} iters ({:.1}s), stop: {}, residual {:.2e} -> {:.2e} (peak/final = {:.1} orders)",
                res.iters, dt, res.stop_reason, hist(20), hist(res.iters - 1), res.drop_orders
            );
            println!(
                "          min rho = {:.4}, min p = {:.4}   Cp(num) = {:.5} (first cell {:.5})   beta(num) = {:.3} deg",
                res.min_rho, res.min_p, res.cp_avg, res.cp_cell_avg, res.beta_num.to_degrees()
            );
            rows.push((theta, name, ni, nj, res.cp_avg, tm.cp_surface, res.beta_num.to_degrees(), tm.beta.to_degrees(), res.drop_orders, res.iters));
        }
    }

    println!("\n| cone | grid | Cp (CFD) | Cp (TM) | Cp err % | beta CFD [deg] | beta TM [deg] | beta err [deg] | residual drop | iters |");
    println!("|---|---|---|---|---|---|---|---|---|---|");
    for (theta, name, ni, nj, cp, cp_tm, b, b_tm, drop, iters) in rows {
        println!(
            "| {theta:.0} deg | {name} {ni}x{nj} | {cp:.5} | {cp_tm:.5} | {:+.2} | {b:.3} | {b_tm:.3} | {:+.3} | {drop:.1} orders | {iters} |",
            100.0 * (cp - cp_tm) / cp_tm,
            b - b_tm
        );
    }
    println!("\nCSV output written to {}", out_dir.canonicalize().unwrap_or(out_dir).display());
}
