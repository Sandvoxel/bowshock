//! Speed of the array-op solver (f32, double-float) against the hand-written WGSL kernels (`bowshock_gpu::GpuSolver`,
//! `Mode::Guarded`, `Precision::F32` / `Extended`) and the f64 CPU `Solver` on the cone grids of `cone_gpu --bench`
//! (M = 2, 15 degree cone, minmod, MUSCL from the first iteration, from the free stream, best of 3).
//!
//! `cargo run --release --example bench [-- --quick] [--n 2000] [--sync 50] [--no-cpu] [--no-profile] [--profile-only]`
//!
//! `--quick` skips 400x200. Steady-state us/iteration excludes compilation: the first call of every loop form is run (and
//! timed, reported separately as "first call") before the steady measurement starts.

use bowshock_cases::cone::{build_solver, ConeCase};
use bowshock_gpu::{GpuSolver, Mode, Precision};
use bowshock_lazy::Staging;
use bowshock_reference::taylor_maccoll::solve_cone;
use std::time::Instant;

fn best_of<F: FnMut() -> f64>(reps: usize, mut f: F) -> f64 {
    (0..reps).map(|_| f()).fold(f64::INFINITY, f64::min)
}

/// `Profile` with the kernel labels cut to 90 characters (fused kernels are labelled by every operation they contain).
fn compact(p: &luma::Profile) -> String {
    let mut out = format!("{:<92} {:>8} {:>10} {:>10}\n", "kernel", "launches", "total us", "mean us");
    for k in &p.kernels {
        let label: String = k.label.chars().take(90).collect();
        out += &format!("{label:<92} {:>8} {:>10.1} {:>10.1}\n", k.launches, k.total.as_secs_f64() * 1e6, k.mean().as_secs_f64() * 1e6);
    }
    out
}

/// Distinct fusion split reasons of a profile with their counts.
fn splits(p: &luma::Profile) -> String {
    let mut seen: Vec<(String, usize)> = Vec::new();
    for s in &p.splits {
        let text = s.to_string();
        match seen.iter_mut().find(|(t, _)| *t == text) {
            Some((_, n)) => *n += 1,
            None => seen.push((text, 1)),
        }
    }
    seen.iter().map(|(t, n)| format!("{t}  x{n}\n")).collect()
}

/// One lazy solver of one precision: steady us/iteration of the three loop forms and the first-call time, for every staging.
macro_rules! bench_lazy {
    ($module:ident, $staging:expr, $ctx:expr, $host:expr, $u0:expr, $n:expr, $n_eval:expr, $sync:expr) => {{
        use bowshock_lazy::$module::LazySolver;
        let (n, n_eval, sync) = ($n, $n_eval, $sync);
        let t0 = Instant::now();
        let mut s = LazySolver::new($ctx, &$host.grid, &$host.cfg).unwrap();
        s.staging = $staging;
        // first call: compile + trace
        s.run(sync, sync).unwrap();
        let first = t0.elapsed().as_secs_f64();
        let mut forms = Vec::new();
        // iterate_until_all with the residual recorded every iteration
        forms.push(best_of(3, || {
            s.set_state($u0).unwrap();
            s.sync().unwrap();
            let t0 = Instant::now();
            s.run(n, sync).unwrap();
            1e6 * t0.elapsed().as_secs_f64() / n as f64
        }));
        // iterate_all replay, no residual history
        s.run_replay(sync).unwrap();
        s.sync().unwrap();
        forms.push(best_of(3, || {
            s.set_state($u0).unwrap();
            s.sync().unwrap();
            let t0 = Instant::now();
            s.run_replay(n).unwrap();
            s.sync().unwrap();
            1e6 * t0.elapsed().as_secs_f64() / n as f64
        }));
        // eval_all every iteration, no replay
        s.run_eval(sync).unwrap();
        s.sync().unwrap();
        forms.push(best_of(3, || {
            s.set_state($u0).unwrap();
            s.sync().unwrap();
            let t0 = Instant::now();
            let h = s.run_eval(n_eval).unwrap();
            s.sync().unwrap();
            let us = 1e6 * t0.elapsed().as_secs_f64() / n_eval as f64;
            drop(h);
            us
        }));
        (first, forms)
    }};
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let flag = |name: &str| args.iter().any(|a| a == name);
    let arg = |name: &str| args.iter().position(|a| a == name).and_then(|i| args.get(i + 1)).and_then(|s| s.parse::<usize>().ok());
    let n = arg("--n").unwrap_or(2000);
    let n_eval = n.min(300);
    let sync = arg("--sync").unwrap_or(50);
    let mut grids = vec![(100usize, 50usize), (200, 100)];
    if !flag("--quick") {
        grids.push((400, 200));
    }
    if flag("--profile-only") {
        grids.clear();
    }

    let ctx = luma::default_context().expect("open GPU (lazy Luma)");
    let ctx_old = luma_old::default_context().expect("open GPU (Luma 0.2.4 of bowshock-gpu)");
    println!("RTX 3090 via Vulkan, both Luma versions on the same adapter; {n} iterations (eval loop {n_eval}), sync every {sync}");
    let tm = solve_cone(2.0, 15f64.to_radians()).unwrap();
    // `--staging Lines` runs one staging only (a fresh process shows its cold compile time)
    let only = args.iter().position(|a| a == "--staging").and_then(|i| args.get(i + 1)).cloned();
    let stagings: Vec<(Staging, &str)> = [(Staging::Fusion, "Fusion"), (Staging::Lines, "Lines"), (Staging::Fluxes, "Fluxes"), (Staging::Split, "Split")]
        .into_iter()
        .filter(|(_, name)| only.as_deref().is_none_or(|o| o == *name))
        .collect();

    let mut details = Vec::new();
    let mut firsts = Vec::new();
    println!("\n### Steady per-iteration cost, us/iteration (15 deg cone, M = 2, minmod, {n} iterations from the free stream, best of 3)");
    println!("Lazy columns: the fastest of staging x loop form (see the detail table).");
    println!(
        "| grid | CPU f64 (rayon, {} threads) | custom F32 | custom Extended | lazy f32 | lazy df32 | lazy f32 / custom F32 | lazy df32 / custom Extended |",
        std::thread::available_parallelism().map_or(0, |n| n.get())
    );
    println!("|---|---|---|---|---|---|---|---|");
    for &(ni, nj) in &grids {
        let mut case = ConeCase::new(2.0, 15.0, ni, nj);
        case.first_order_iters = 0;
        let host = build_solver(&case, tm.beta);
        let u0 = host.u.clone();

        let cpu = if flag("--no-cpu") {
            f64::NAN
        } else {
            let cpu_n = n.min(500);
            best_of(2, || {
                let mut s = build_solver(&case, tm.beta);
                for _ in 0..20 {
                    s.step();
                }
                let t0 = Instant::now();
                for _ in 0..cpu_n {
                    s.step();
                }
                1e6 * t0.elapsed().as_secs_f64() / cpu_n as f64
            })
        };

        let mut custom = [0.0f64; 2];
        let mut custom_first = [0.0f64; 2];
        for (m, prec) in [Precision::F32, Precision::Extended].into_iter().enumerate() {
            let t0 = Instant::now();
            let mut g = GpuSolver::new_with(&ctx_old, &host.grid, &host.cfg, Mode::Guarded, prec).unwrap();
            g.run(1).unwrap();
            custom_first[m] = t0.elapsed().as_secs_f64();
            g.run(sync).unwrap();
            custom[m] = best_of(3, || {
                g.reset().unwrap();
                g.context().sync().unwrap();
                let t0 = Instant::now();
                for _ in 0..n / sync {
                    g.run(sync).unwrap();
                }
                1e6 * t0.elapsed().as_secs_f64() / ((n / sync) * sync) as f64
            });
        }

        let mut best = [f64::INFINITY; 2];
        for &(staging, name) in &stagings {
            let (f_first, f_forms) = bench_lazy!(f32s, staging, &ctx, host, &u0, n, n_eval, sync);
            let (d_first, d_forms) = bench_lazy!(df32s, staging, &ctx, host, &u0, n, n_eval, sync);
            best[0] = f_forms.iter().fold(best[0], |a, &b| a.min(b));
            best[1] = d_forms.iter().fold(best[1], |a, &b| a.min(b));
            details.push((ni, nj, name, f_forms, d_forms));
            firsts.push((ni, nj, name, f_first, d_first, custom_first));
        }
        println!(
            "| {ni}x{nj} | {cpu:.0} | {:.1} | {:.1} | {:.1} | {:.1} | {:.0}x | {:.0}x |",
            custom[0],
            custom[1],
            best[0],
            best[1],
            best[0] / custom[0],
            best[1] / custom[1]
        );
    }

    if !details.is_empty() {
        println!("\n### Lazy solver detail, us/iteration by staging and loop form");
        println!("`until` = `iterate_until_all` with the density residual recorded every iteration, `replay` = `iterate_all`, `eval` = `eval_all` every iteration");
        println!("| grid | staging | f32 until | f32 replay | f32 eval | df32 until | df32 replay | df32 eval |");
        println!("|---|---|---|---|---|---|---|---|");
        for (ni, nj, name, f, d) in &details {
            println!("| {ni}x{nj} | {name} | {:.0} | {:.0} | {:.0} | {:.0} | {:.0} | {:.0} |", f[0], f[1], f[2], d[0], d[1], d[2]);
        }
        println!("\n### First call (construction + compile + trace + first {sync} iterations), seconds");
        println!("| grid | staging | lazy f32 | lazy df32 | custom F32 | custom Extended |");
        println!("|---|---|---|---|---|---|");
        for (ni, nj, name, f, d, c) in &firsts {
            println!("| {ni}x{nj} | {name} | {f:.2} | {d:.2} | {:.2} | {:.2} |", c[0], c[1]);
        }
    }

    if !flag("--no-profile") {
        let case = {
            let mut c = ConeCase::new(2.0, 15.0, 200, 100);
            c.first_order_iters = 0;
            c
        };
        let host = build_solver(&case, tm.beta);
        macro_rules! profile {
            ($module:ident, $name:expr, $staging:expr, $sname:expr) => {{
                let mut s = bowshock_lazy::$module::LazySolver::new(&ctx, &host.grid, &host.cfg).unwrap();
                s.staging = $staging;
                s.set_state(&host.u).unwrap();
                let p = s.profile_residual().unwrap();
                println!(
                    "\n### {} / {} - one residual evaluation (200x100): {} kernel launches, {} us GPU time, {} fusion cuts\n```\n{}{}```",
                    $name,
                    $sname,
                    p.launches(),
                    p.total().as_micros(),
                    p.splits.len(),
                    splits(&p),
                    compact(&p)
                );
                let p = s.profile_step().unwrap();
                println!(
                    "### {} / {} - one SSP-RK3 step (200x100): {} kernel launches, {} us GPU time, {} fusion cuts\n```\n{}{}```",
                    $name,
                    $sname,
                    p.launches(),
                    p.total().as_micros(),
                    p.splits.len(),
                    splits(&p),
                    compact(&p)
                );
            }};
        }
        for &(staging, sname) in &stagings {
            profile!(f32s, "lazy f32", staging, sname);
            profile!(df32s, "lazy df32", staging, sname);
        }
    }
}
