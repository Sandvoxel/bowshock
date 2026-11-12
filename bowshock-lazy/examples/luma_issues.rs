//! What still applies of the Luma problems met while writing the solver (see README, "Luma issues"). The four bugs of the first
//! version of this file (Df32 comparison kernels, the Df32 -> f32 cast, the input-buffer count after an evaluation, threads on
//! `default_context()`) are fixed in Luma and have their regression tests there. Two observations remain:
//!
//! 1. ghost layers written as region assigns into a lazy padded array are computed at every cell that reads them, so the
//!    operation count of the kernel that reads the array grows with the size of the ghost expressions and the fusion limits cut it;
//! 2. building a lazy expression costs about half a microsecond per operation, which is what the `eval` loop form spends
//!    (1.5 of 1.7 ms per iteration) on.
//!
//! `cargo run --release --example luma_issues`

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

fn main() {
    let ctx = luma::default_context().unwrap();
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
    let sizes: Vec<usize> = std::env::var("LUMA_ISSUES_GHOST").map_or_else(|_| vec![0, 10, 40, 100], |list| list.split(',').filter_map(|s| s.parse().ok()).collect());
    for ghost_ops in sizes {
        let build = |keep: bool| {
            let (out, held): ([A; 4], ([A; 4], [A; 4], [A; 4])) = {
                let w = u.clone().pad([(2, 2), (2, 2)], PadMode::Edge);
                for side in 0..4 {
                    let (region, edge) = match side {
                        0 => (w.slice(0, ..2).slice(1, 2..n + 2), u.clone().pad([(2, 0)], PadMode::Edge).slice(0, 0..2)),
                        1 => (w.slice(0, n + 2..).slice(1, 2..n + 2), u.clone().pad([(0, 2)], PadMode::Edge).slice(0, n..n + 2)),
                        2 => (w.slice(1, ..2).slice(0, 2..n + 2), u.clone().pad([(0, 0), (2, 0)], PadMode::Edge).slice(1, 0..2)),
                        _ => (w.slice(1, n + 2..).slice(0, 2..n + 2), u.clone().pad([(0, 0), (0, 2)], PadMode::Edge).slice(1, n..n + 2)),
                    };
                    if ghost_ops > 0 {
                        let rhs: [A; 4] = std::array::from_fn(|k| chain(&[&edge[0], &edge[1], &edge[2], &edge[3]], k, ghost_ops));
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
