//! Speed of the viscous array-op solver (laminar and SST, `Scheme::Implicit(ImplicitParams::VISCOUS)`) on the G3 body grids (tangent ogive
//! 0.30 x 0.10 m + 0.60 m tube, ISA 0 m, Mach 0.5, flat base and wake block; `Case::viscous` at the grid levels 0 = coarse and 1 = medium)
//! against the f64 CPU `Solver`, and the hand-written WGSL kernels (`bowshock_gpu::GpuSolver`, `Mode::Guarded`, `Precision::F32` /
//! `Extended`, viscous terms set from the host solver).
//!
//! `cargo run --release --example bench_viscous [-- --n 200] [--sync 50] [--no-cpu] [--no-profile] [--levels 0,1] [--laminar]
//! [--staging Fusion|Lines|Fluxes|Split]`
//!
//! us/iteration of `n` iterations from the free stream (the first half first order, so both limiter phases are compiled before the clock),
//! best of 3; CPU = `Solver::step` on all rayon threads, custom = `run(sync)` chunks, lazy `until` = `iterate_until_all` with the residual recorded,
//! `replay` = `iterate_all`. `--laminar` switches the SST model off. The profile section prints launches, fusion cuts and the biggest kernels of
//! one lazy step.

use bowshock::atmosphere::Atmosphere;
use bowshock::body_grid::BaseBody;
use bowshock::case::{Case, Wake};
use bowshock::spec::{AftSpec, BodySpec, NoseShape, NoseSpec};
use bowshock_core::solver::Turbulence;
use bowshock_gpu::{GpuSolver, Mode, Precision};
use bowshock_lazy::Staging;
use std::time::Instant;

fn best_of<F: FnMut() -> f64>(reps: usize, mut f: F) -> f64 {
    (0..reps).map(|_| f()).fold(f64::INFINITY, f64::min)
}

/// The G3 case at grid level `level`.
fn g3_case(level: u32, laminar: bool, n: usize) -> Case {
    let (d, mach) = (0.10, 0.5);
    let spec = BodySpec { nose: NoseSpec { shape: NoseShape::TangentOgive, length: 0.30, diameter: d }, aft: vec![AftSpec::Tube { length: 0.60 }] };
    let (body, _): (BaseBody, _) = spec.to_internal().unwrap();
    let air = Atmosphere::Isa { altitude_m: 0.0 }.state().unwrap();
    let re_c = air.density * air.sound_speed * d / air.viscosity;
    let mut case = Case::viscous(mach, body, level, Wake::Base, re_c, 110.4 / air.temperature_k, mach);
    case.numerics.first_order_iters = n / 2;
    if laminar {
        case.viscous.as_mut().unwrap().turbulence = Turbulence::Laminar;
    }
    case
}

/// `Profile` with the kernel labels cut to 90 characters (fused kernels are labelled by every operation they contain), the `top` biggest first.
fn compact(p: &luma::Profile, top: usize) -> String {
    let mut out = format!("{:<92} {:>8} {:>10} {:>10}\n", "kernel", "launches", "total us", "mean us");
    let mut kernels: Vec<_> = p.kernels.iter().collect();
    kernels.sort_by_key(|k| std::cmp::Reverse(k.total));
    for k in kernels.into_iter().take(top) {
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

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let flag = |name: &str| args.iter().any(|a| a == name);
    let arg = |name: &str| args.iter().position(|a| a == name).and_then(|i| args.get(i + 1)).cloned();
    let n: usize = arg("--n").and_then(|s| s.parse().ok()).unwrap_or(200);
    let sync: usize = arg("--sync").and_then(|s| s.parse().ok()).unwrap_or(50);
    let levels: Vec<u32> = arg("--levels").map_or(vec![0, 1], |s| s.split(',').filter_map(|l| l.parse().ok()).collect());
    let laminar = flag("--laminar");
    let staging = match arg("--staging").as_deref() {
        Some("Fusion") => Staging::Fusion,
        Some("Lines") => Staging::Lines,
        Some("Split") => Staging::Split,
        _ => Staging::Fluxes,
    };
    let ctx = luma::default_context().expect("open GPU (lazy Luma)");
    let ctx_old = luma_old::default_context().expect("open GPU (Luma 0.2.4 of bowshock-gpu)");

    println!("\n### Viscous implicit scheme ({}), us/iteration: G3 body, M = 0.5, {n} iterations from the free stream (first {} first order), best of 3, staging {staging:?}", if laminar { "laminar" } else { "SST" }, n / 2);
    println!("| level | grid | CPU f64 | custom F32 | custom Extended | lazy f32 until | lazy f32 replay | lazy df32 until | lazy df32 replay |");
    println!("|---|---|---|---|---|---|---|---|---|");
    for &level in &levels {
        let case = g3_case(level, laminar, n);
        let (ni, nj) = case.cells();
        let host = case.build_solver();
        let cpu = if flag("--no-cpu") {
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
        let mut custom = [f64::NAN; 2];
        for (m, prec) in [Precision::F32, Precision::Extended].into_iter().enumerate() {
            let mut g = GpuSolver::from_host(&ctx_old, &host, Mode::Guarded, prec).unwrap();
            for _ in 0..n / sync {
                g.run(sync).unwrap();
            }
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
        macro_rules! lazy {
            ($module:ident) => {{
                let t0 = Instant::now();
                let mut s = bowshock_lazy::$module::LazySolver::from_host(&ctx, &host).unwrap();
                s.staging = staging;
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
                eprintln!("level {level} {}: first call (construction + compile + trace + {n} iterations) {first:.2} s", stringify!($module));
                (until, replay)
            }};
        }
        let (f, d) = (lazy!(f32s), lazy!(df32s));
        println!("| {level} | {ni}x{nj} | {cpu:.0} | {:.1} | {:.1} | {:.1} | {:.1} | {:.1} | {:.1} |", custom[0], custom[1], f.0, f.1, d.0, d.1);
    }

    if flag("--no-profile") {
        return;
    }
    for &level in &levels {
        let case = g3_case(level, laminar, n);
        let host = case.build_solver();
        macro_rules! profile {
            ($module:ident, $name:expr) => {{
                let mut s = bowshock_lazy::$module::LazySolver::from_host(&ctx, &host).unwrap();
                s.staging = staging;
                s.run(n, sync).unwrap(); // past the first-order phase: the profiled step is the MUSCL one
                let p = s.profile_step().unwrap();
                println!(
                    "\n### {} level {level} - one viscous implicit step as the eval loop launches it: {} kernel launches, {} us GPU time, {} fusion cuts\n```\n{}{}```",
                    $name,
                    p.launches(),
                    p.total().as_micros(),
                    p.splits.len(),
                    splits(&p),
                    compact(&p, 14)
                );
                let n_replay = 20u64;
                let p = s.profile_replay(n_replay as usize).unwrap();
                println!("{} level {level} - one viscous implicit step of the `iterate_all` replay: {} kernel launches, {} us GPU time\n", $name, p.launches() as u64 / n_replay, p.total().as_micros() as u64 / n_replay);
            }};
        }
        profile!(f32s, "lazy f32");
        profile!(df32s, "lazy df32");
    }
}
