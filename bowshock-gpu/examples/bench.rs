//! Speed of the array-op solver (f32, double-float) against the f64 CPU `Solver` on the cone grids of `cone_gpu --bench`
//! (M = 2, 15 degree cone, minmod, MUSCL from the first iteration, from the free stream, best of 3).
//!
//! `cargo run --release -p bowshock-gpu --example bench [-- --quick] [--n 2000] [--sync 50] [--no-cpu] [--no-profile] [--profile-only]
//! [--implicit] [--no-implicit]`
//!
//! `--implicit` runs only the line-implicit section: us/iteration of `Scheme::Implicit` on the sting case and the base case (solid
//! block) of `subsonic_case` (M = 0.5, tangent ogive, grid levels 0 and 1; 200 iterations from the free stream, the first 100 first order) for the CPU
//! `Solver::step` and the lazy solver (`iterate_until_all` and `iterate_all` replay), and the per-kernel profile of one lazy implicit step at
//! level 0. `LUMA_TRIDIAGONAL=thomas|cyclic` forces the algorithm of the line solves. `--no-implicit` skips the section.
//!
//! `--quick` skips 400x200. Steady-state us/iteration excludes compilation: the first call of every loop form is run (and
//! timed, reported separately as "first call") before the steady measurement starts.

use bowshock::body_grid::Nose;
use bowshock_cases::cone::{build_solver, ConeCase};
use bowshock_cases::subsonic::subsonic_case;
use bowshock_gpu::Staging;
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
        use bowshock_gpu::$module::LazySolver;
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

/// The line-implicit section: see the module comment.
fn implicit_section(ctx: &luma::Context, n: usize, sync: usize, no_cpu: bool, profile: bool) {
    println!("\n### Line-implicit scheme (`Scheme::Implicit`), us/iteration: M = 0.5, {n} iterations from the free stream (100 first order), best of 3");
    println!(
        "`lazy until` = `iterate_until_all` with the residual recorded, `lazy replay` = `iterate_all`; `LUMA_TRIDIAGONAL` = {}",
        std::env::var("LUMA_TRIDIAGONAL").unwrap_or_else(|_| "(unset: Luma chooses)".into())
    );
    println!("| case | level | grid | CPU f64 | lazy f32 until | lazy f32 replay | lazy df32 until | lazy df32 replay |");
    println!("|---|---|---|---|---|---|---|---|");
    for (base, level) in [(false, 0u32), (false, 1), (true, 0), (true, 1)] {
        let case = subsonic_case(0.5, Nose::TangentOgive { fineness: 3.0 }, level, base);
        let (ni, nj) = case.cells();
        let host = case.build_solver();
        let cpu = if no_cpu {
            f64::NAN
        } else {
            best_of(2, || {
                let mut s = case.build_solver();
                for _ in 0..10 {
                    s.step();
                }
                let t0 = Instant::now();
                for _ in 0..n {
                    s.step();
                }
                1e6 * t0.elapsed().as_secs_f64() / n as f64
            })
        };
        macro_rules! lazy {
            ($module:ident) => {{
                let mut s = bowshock_gpu::$module::LazySolver::from_host(ctx, &host).unwrap();
                let t0 = Instant::now();
                s.run(n, sync).unwrap();
                let first = t0.elapsed().as_secs_f64();
                s.run_replay(n).unwrap();
                s.sync().unwrap();
                let until = best_of(3, || {
                    s.set_state(&host.u).unwrap();
                    s.sync().unwrap();
                    let t0 = Instant::now();
                    s.run(n, sync).unwrap();
                    1e6 * t0.elapsed().as_secs_f64() / n as f64
                });
                let replay = best_of(3, || {
                    s.set_state(&host.u).unwrap();
                    s.sync().unwrap();
                    let t0 = Instant::now();
                    s.run_replay(n).unwrap();
                    s.sync().unwrap();
                    1e6 * t0.elapsed().as_secs_f64() / n as f64
                });
                eprintln!("level {level} {}: first call (compile + trace + {n} iterations) {first:.2} s", stringify!($module));
                (until, replay)
            }};
        }
        let (f, d) = (lazy!(f32s), lazy!(df32s));
        println!(
            "| {} | {level} | {ni}x{nj} | {cpu:.0} | {:.1} | {:.1} | {:.1} | {:.1} |",
            if base { "base (solid block)" } else { "sting" },
            f.0,
            f.1,
            d.0,
            d.1
        );
    }

    for base in [false, true].into_iter().filter(|_| profile) {
        let case = subsonic_case(0.5, Nose::TangentOgive { fineness: 3.0 }, 0, base);
        let host = case.build_solver();
        macro_rules! profile {
            ($module:ident, $name:expr) => {{
                let mut s = bowshock_gpu::$module::LazySolver::from_host(ctx, &host).unwrap();
                s.run(n, sync).unwrap(); // past the first-order phase: the profiled step is the MUSCL one
                let p = s.profile_step().unwrap();
                println!(
                    "\n### {} {} - one implicit step as the eval loop launches it (level 0): {} kernel launches, {} us GPU time, {} fusion cuts\n```\n{}{}```",
                    $name,
                    if base { "base" } else { "sting" },
                    p.launches(),
                    p.total().as_micros(),
                    p.splits.len(),
                    splits(&p),
                    compact(&p)
                );
                let n_replay = 20u64;
                let p = s.profile_replay(n_replay as usize).unwrap();
                println!(
                    "{} {} - one implicit step of the `iterate_all` replay: {} kernel launches, {} us GPU time\n",
                    $name,
                    if base { "base" } else { "sting" },
                    p.launches() as u64 / n_replay,
                    p.total().as_micros() as u64 / n_replay
                );
            }};
        }
        profile!(f32s, "lazy f32");
        profile!(df32s, "lazy df32");
    }
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
    println!("{n} iterations (eval loop {n_eval}), sync every {sync}");
    if flag("--implicit") {
        implicit_section(&ctx, arg("--n").unwrap_or(200), sync, flag("--no-cpu"), !flag("--no-profile"));
        return;
    }
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
        "| grid | CPU f64 (rayon, {} threads) | lazy f32 | lazy df32 | lazy f32 vs CPU | lazy df32 vs CPU |",
        std::thread::available_parallelism().map_or(0, |n| n.get())
    );
    println!("|---|---|---|---|---|---|");
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

        let mut best = [f64::INFINITY; 2];
        for &(staging, name) in &stagings {
            let (f_first, f_forms) = bench_lazy!(f32s, staging, &ctx, host, &u0, n, n_eval, sync);
            let (d_first, d_forms) = bench_lazy!(df32s, staging, &ctx, host, &u0, n, n_eval, sync);
            best[0] = f_forms.iter().fold(best[0], |a, &b| a.min(b));
            best[1] = d_forms.iter().fold(best[1], |a, &b| a.min(b));
            details.push((ni, nj, name, f_forms, d_forms));
            firsts.push((ni, nj, name, f_first, d_first));
        }
        println!("| {ni}x{nj} | {cpu:.0} | {:.1} | {:.1} | {:.0}x | {:.0}x |", best[0], best[1], cpu / best[0], cpu / best[1]);
    }

    if !details.is_empty() {
        println!("\n### Lazy solver detail, us/iteration by staging and loop form");
        println!(
            "`until` = `iterate_until_all` with the density residual recorded every iteration, `replay` = `iterate_all`, `eval` = `eval_all` every iteration"
        );
        println!("| grid | staging | f32 until | f32 replay | f32 eval | df32 until | df32 replay | df32 eval |");
        println!("|---|---|---|---|---|---|---|---|");
        for (ni, nj, name, f, d) in &details {
            println!("| {ni}x{nj} | {name} | {:.0} | {:.0} | {:.0} | {:.0} | {:.0} | {:.0} |", f[0], f[1], f[2], d[0], d[1], d[2]);
        }
        println!("\n### First call (construction + compile + trace + first {sync} iterations), seconds");
        println!("| grid | staging | lazy f32 | lazy df32 |");
        println!("|---|---|---|---|");
        for (ni, nj, name, f, d) in &firsts {
            println!("| {ni}x{nj} | {name} | {f:.2} | {d:.2} |");
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
                let mut s = bowshock_gpu::$module::LazySolver::new(&ctx, &host.grid, &host.cfg).unwrap();
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
                // the `iterate_all` replay the steady table measures: 20 iterations, per-iteration numbers
                let n_replay = 20u64;
                let p = s.profile_replay(n_replay as usize).unwrap();
                println!(
                    "### {} / {} - one SSP-RK3 step of the replay (200x100): {} kernel launches, {} us GPU time\n",
                    $name,
                    $sname,
                    p.launches() as u64 / n_replay,
                    p.total().as_micros() as u64 / n_replay,
                );
            }};
        }
        for &(staging, sname) in &stagings {
            profile!(f32s, "lazy f32", staging, sname);
            profile!(df32s, "lazy df32", staging, sname);
        }
    }

    if !flag("--no-implicit") && !flag("--profile-only") {
        implicit_section(&ctx, arg("--n").map_or(200, |n| n.min(200)), sync, flag("--no-cpu"), !flag("--no-profile"));
    }
}
