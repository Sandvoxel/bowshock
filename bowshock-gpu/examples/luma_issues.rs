//! What still applies of the Luma problems met while writing the solver (see README, "Luma issues"). The four bugs of the first
//! version of this file (Df32 comparison kernels, the Df32 -> f32 cast, the input-buffer count after an evaluation, threads on
//! `default_context()`) are fixed in Luma and have their regression tests there. Two observations remain:
//!
//! 1. ghost layers written as region assigns into a lazy padded array are computed at every cell that reads them, so the
//!    operation count of the kernel that reads the array grows with the size of the ghost expressions and the fusion limits cut it;
//! 2. building a lazy expression costs about half a microsecond per operation, which is what the `eval` loop form spends
//!    (1.5 of 1.7 ms per iteration) on;
//! 3. region assigns of the solid block into the lazy residual, flux and ghost expressions of the explicit solver (case 3
//!    below): launches, fusion cuts and GPU time of one SSP-RK3 step with and without the block on the same grid.
//!
//! `cargo run --release --example luma_issues [-- 3]` (`3`: only case 3; `LUMA_ISSUES_ONLY=<grid>:<staging>:<prec>:<yes|no>` limits it to
//! the rows starting so, `LUMA_ISSUES_KERNELS=1` lists the kernels of the `eval` step)

use bowshock::body_grid::Nose;
use bowshock_cases::cone::{build_solver, ConeCase};
use bowshock_cases::subsonic::{fox_case, subsonic_case};
use bowshock_core::solver::Scheme;
use bowshock_gpu::Staging;
use bowshock_reference::taylor_maccoll::solve_cone;
use luma::{Array, Fields, PadMode};
use std::time::Instant;

type A = Array<f32>;

/// A chain of `ops` mixed operations over the arrays `xs`, standing in for a ghost-state expression (a Riemann invariant,
/// a few square roots and selects).
fn chain(xs: &[&A], seed: usize, ops: usize) -> A {
    let mut acc = xs[seed % xs.len()] * 1.0;
    for k in 0..ops {
        let y = xs[(seed + k + 1) % xs.len()];
        acc = match k % 4 {
            0 => &acc * y + 0.1,
            1 => (&acc - y).abs() + 0.2,
            2 => &acc / (y + 1.5),
            _ => &acc + y * 0.5,
        };
    }
    acc
}

/// Distinct split reasons of a profile (location left out) with their counts.
fn cuts(p: &luma::Profile) -> String {
    let mut seen: Vec<(String, usize)> = Vec::new();
    for s in &p.splits {
        // "split at <path>/src/body.rs:453:20 (`assign`): ..." without the directory
        let full = s.to_string().replace("split at ", "");
        let text = full.find("src/").map_or(full.clone(), |at| full[at..].to_string());
        match seen.iter_mut().find(|(t, _)| *t == text) {
            Some((_, n)) => *n += 1,
            None => seen.push((text, 1)),
        }
    }
    seen.iter().map(|(t, n)| format!("      cut x{n}: {t}\n")).collect()
}

/// One SSP-RK3 step of the explicit solver, per grid (with the solid block and the same grid without it), staging and
/// precision. `replay`: launches and GPU time per step of the `iterate_all` replay; `eval`: launches, cuts and GPU time of one
/// step in the `eval` loop form (the cuts are made while the step is built, which the replay does once).
fn case3(ctx: &luma::Context) {
    println!("== 3. the explicit solver with a solid block (region assigns for ghosts, fluxes, zero residual / time step) against the same grid without it");
    println!("   cost model: {:?}", ctx.cost_model());
    println!(
        "   {:<16} {:<9} {:<6} {:<6} | {:>8} {:>8} | {:>8} {:>8} {:>5}",
        "grid", "staging", "prec", "block", "replay L", "replay us", "eval L", "eval us", "cuts"
    );
    // the two block grids, and a block-free cone grid of the base grid's size (the reference of the issue list)
    let solid = |label: &'static str, mut case: bowshock::case::Case| {
        case.numerics.scheme = Scheme::Explicit;
        let mut host = case.build_solver();
        if host.scheme() != Scheme::Explicit {
            host.set_scheme(Scheme::Explicit);
        }
        (label, host)
    };
    let cone = {
        let mut c = ConeCase::new(2.0, 15.0, 156, 76);
        c.first_order_iters = 0;
        build_solver(&c, solve_cone(2.0, 15f64.to_radians()).unwrap().beta)
    };
    let cases =
        [solid("ogive base", subsonic_case(0.5, Nose::TangentOgive { fineness: 3.0 }, 0, true)), solid("fox sting", fox_case(false, 0.6, 0)), ("cone", cone)];
    let verbose = std::env::var("LUMA_ISSUES_KERNELS").is_ok();
    for (label, host) in cases {
        let grid = format!("{label} {}x{}", host.grid.ni, host.grid.nj);
        macro_rules! run {
            ($module:ident, $prec:expr) => {
                for staging in [Staging::Fluxes, Staging::Fusion] {
                    for with_block in if host.block().is_some() { vec![true, false] } else { vec![false] } {
                        if let Ok(only) = std::env::var("LUMA_ISSUES_ONLY") {
                            let name = format!("{label}:{staging:?}:{}:{}", $prec, if with_block { "yes" } else { "no" });
                            if !name.starts_with(&only) {
                                continue;
                            }
                        }
                        let mut s = bowshock_gpu::$module::LazySolver::from_host(ctx, &host).unwrap();
                        if !with_block {
                            s.set_block(None).unwrap();
                        }
                        s.staging = staging;
                        let n = 20;
                        // keep the GPU at its working clock (the first profile after a pause runs on lowered clocks)
                        let burn = Instant::now();
                        while burn.elapsed().as_secs_f64() < 0.3 {
                            s.run_replay(100).unwrap();
                            s.sync().unwrap();
                        }
                        s.set_state(&host.u).unwrap();
                        // GPU clocks and the first replay's warm-up make single profiles noisy: the fastest of five
                        let replay = (0..5).map(|_| s.profile_replay(n).unwrap()).min_by_key(|p| p.total()).unwrap();
                        let eval = s.profile_step().unwrap();
                        println!(
                            "   {grid:<16} {:<9} {:<6} {:<6} | {:>8.1} {:>8.1} | {:>8} {:>8} {:>5}",
                            format!("{staging:?}"),
                            $prec,
                            if with_block { "yes" } else { "no" },
                            replay.launches() as f64 / n as f64,
                            replay.total().as_secs_f64() * 1e6 / n as f64,
                            eval.launches(),
                            eval.total().as_micros(),
                            eval.splits.len()
                        );
                        print!("{}", cuts(&eval));
                        if verbose {
                            for k in &eval.kernels {
                                let chars: Vec<char> = k.label.chars().collect();
                                let label: String = if chars.len() > 130 {
                                    chars[..70].iter().chain(['.', '.', '.'].iter()).chain(chars[chars.len() - 55..].iter()).collect()
                                } else {
                                    k.label.clone()
                                };
                                println!("      {label} x{} ({:.1} us)", k.launches, k.total.as_secs_f64() * 1e6);
                            }
                        }
                    }
                }
            };
        }
        run!(f32s, "f32");
        run!(df32s, "df32");
    }
}

fn main() {
    let ctx = luma::default_context().unwrap();
    case3(&ctx);
    if std::env::args().any(|a| a == "3") {
        return;
    }
    let n = 200usize;
    let data = |k: usize| Array::from_slice_on(&ctx, &(0..n * n).map(|i| 1.0 + 0.1 * ((i + k) as f32 * 0.37).sin()).collect::<Vec<_>>(), [n, n]).unwrap();
    let u: [A; 4] = std::array::from_fn(data);

    println!("== 1. ghost layers as region assigns into a lazy padded record, read by a four-point stencil");
    println!("   (a 4-field record on {n}x{n}, `pad` + one `assign` per side whose right-hand side is `ghost_ops` operations of the");
    println!("   edge cells, then one fused stencil expression reading the padded record at four offsets in each direction)");
    println!("   expected: one kernel whatever the ghost expressions are (the region is 2 cells wide). `dropped`: the padded record is");
    println!("   dropped before the evaluation (as in the solver); `kept`: the program still holds it, so `eval_all` evaluates it first as");
    println!("   a kernel of its own (an array that is alive is work the program asked for)");
    println!("   {:>9} | {:>16} {:>11} | {:>16} {:>11} | {:>16}", "ghost_ops", "launches dropped", "cuts", "launches kept", "cuts", "first cut");
    let sizes: Vec<usize> =
        std::env::var("LUMA_ISSUES_GHOST").map_or_else(|_| vec![0, 10, 40, 100], |list| list.split(',').filter_map(|s| s.parse().ok()).collect());
    for ghost_ops in sizes {
        let build = |keep: bool| {
            let (out, held): ([A; 4], ([A; 4], [A; 4], [A; 4])) = {
                let mut w = u.clone().pad([(2, 2), (2, 2)], PadMode::Edge);
                for side in 0..4 {
                    let edge = match side {
                        0 => u.clone().pad([(2, 0)], PadMode::Edge).slice(0, 0..2),
                        1 => u.clone().pad([(0, 2)], PadMode::Edge).slice(0, n..n + 2),
                        2 => u.clone().pad([(0, 0), (2, 0)], PadMode::Edge).slice(1, 0..2),
                        _ => u.clone().pad([(0, 0), (0, 2)], PadMode::Edge).slice(1, n..n + 2),
                    };
                    if ghost_ops > 0 {
                        let rhs: [A; 4] = std::array::from_fn(|k| chain(&[&edge[0], &edge[1], &edge[2], &edge[3]], k, ghost_ops));
                        let mut region = match side {
                            0 => w.slice_mut(0, ..2).slice(1, 2..n + 2),
                            1 => w.slice_mut(0, n + 2..).slice(1, 2..n + 2),
                            2 => w.slice_mut(1, ..2).slice(0, 2..n + 2),
                            _ => w.slice_mut(1, n + 2..).slice(0, 2..n + 2),
                        };
                        region.assign(rhs).unwrap();
                    }
                }
                // a stencil that reads the padded record at the cells -1 .. +2 of each direction, as a MUSCL face does
                let wi = w.slice(1, 2..n + 2);
                let wj = w.slice(0, 2..n + 2);
                let at = |r: &[A; 4], axis: usize, k: usize, len: usize| r.slice(axis, k..k + len);
                let out: [A; 4] = std::array::from_fn(|f| {
                    let (a, b, c, d) = (at(&wi, 0, 0, n + 1), at(&wi, 0, 1, n + 1), at(&wi, 0, 2, n + 1), at(&wi, 0, 3, n + 1));
                    let fi = (&a[f] - &b[f]).abs() * &c[f] + (&d[f] - &c[f]) * 0.5;
                    let (a, b, c, d) = (at(&wj, 1, 0, n + 1), at(&wj, 1, 1, n + 1), at(&wj, 1, 2, n + 1), at(&wj, 1, 3, n + 1));
                    let fj = (&a[f] - &b[f]).abs() * &c[f] + (&d[f] - &c[f]) * 0.5;
                    fi.slice(0, 1..n + 1).slice(1, 0..n) + fj.slice(1, 1..n + 1).slice(0, 0..n)
                });
                (out, (w, wi, wj))
            };
            let kept = keep.then_some(held);
            luma::eval_all(&out.iter().collect::<Vec<_>>()).unwrap();
            drop(kept);
        };
        let mut row = Vec::new();
        for keep in [false, true] {
            build(keep);
            ctx.sync().unwrap();
            let (_, profile) = ctx
                .profile(|| {
                    build(keep);
                    ctx.sync().unwrap()
                })
                .unwrap();
            if std::env::var("LUMA_ISSUES_KERNELS").is_ok() {
                println!("   ghost_ops {ghost_ops}, w {}:", if keep { "kept" } else { "dropped" });
                for k in &profile.kernels {
                    let label: String = k.label.chars().take(100).collect();
                    println!("             {label} x{} ({:.0} us)", k.launches, k.total.as_secs_f64() * 1e6);
                }
            }
            row.push((profile.launches(), profile.splits.len(), profile.splits.first().map_or(String::new(), |s| s.to_string())));
        }
        println!("   {ghost_ops:>9} | {:>16} {:>11} | {:>16} {:>11} | {:>16}", row[0].0, row[0].1, row[1].0, row[1].1, row[1].2);
    }

    println!("== 2. host cost of building a lazy expression (no evaluation): microseconds per operation, 4 inputs");
    println!("   {:>6} {:>12} {:>14}", "ops", "us per build", "ns per op");
    let xs: Vec<A> = (0..4).map(|k| data(k).slice(0, ..64).slice(1, ..64)).collect();
    let refs: Vec<&A> = xs.iter().collect();
    for ops in [100usize, 400, 900] {
        let reps = 200;
        let t0 = Instant::now();
        for r in 0..reps {
            std::hint::black_box(chain(&refs, r, ops));
        }
        let us = 1e6 * t0.elapsed().as_secs_f64() / reps as f64;
        println!("   {ops:>6} {us:>12.1} {:>14.0}", 1e3 * us / ops as f64);
    }
}
