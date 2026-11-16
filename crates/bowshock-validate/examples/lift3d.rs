//! The 3D Euler stability solver on the GPU: benchmark (and, in the G2 package, the Mach / incidence sweep).
//!
//! ```text
//! cargo run --release -p bowshock-validate --example lift3d -- --case anf|ogive --level 0|1 --precision f32|df32 --bench N
//! ```
//!
//! `--bench N`: the `Case3::new(2.0, 2 degrees, ..)` case on the GPU (`GpuSolver3` from the host `Solver3` of `Case3::build_solver`, implicit
//! scheme), `--case anf`: the Army-Navy Basic Finner (10 d cone-cylinder, four square fins, [`Case3::anf_body`] / [`Case3::anf_fins`]),
//! `--case ogive`: a fin-less tangent ogive (fineness 3) cylinder of the same length. 20 warm-up iterations (compile, trace), then `N` timed
//! iterations of the `iterate_all` replay from the free stream (best of 3), all with second-order reconstruction (`first_order_iters` = 0:
//! the timed step is the MUSCL one). Then the per-kernel profile of one step as the `eval` loop launches it, and one line
//!
//! ```text
//! PERF {"case":"3d-anf-L0-implicit","precision":"df32","cells":..,"iters":N,"us_per_iter":..,"launches_per_iter":..,"gpu_us_per_iter":..,"wall_s":null}
//! ```
//!
//! for `gpuq` (`perf/ledger.jsonl`) and `perfcheck`. `--precision` may be repeated as a comma list (`f32,df32`; default `df32`).
//!
//! `--no-perf` suppresses the PERF line (experiments with `LUMA_TRIDIAGONAL` set must not enter the ledger).
//!
//! `--sweep --mach <list> --alpha 2,6 [--level L]`: one `RESULT {json}` line per (Mach, alpha) point (cold first alpha, warm later ones).

use bowshock_cfd::gpu::{GpuSolver3, Precision};
use bowshock_core::engine::body_grid::{BaseBody, Body, Nose};
use bowshock_core::engine::case3::Case3;
use std::time::Instant;

/// The body of `--case`: `(body, fins)` of `Case3::new`.
fn body_of(case: &str) -> Result<(BaseBody, Option<bowshock_core::engine::grid3::Fins3>), String> {
    match case {
        "anf" => Ok((Case3::anf_body(), Some(Case3::anf_fins()))),
        "ogive" => {
            let nose = Body { nose: Nose::TangentOgive { fineness: 3.0 } };
            let l_nose = nose.nose_length();
            Ok((BaseBody::cylinder(nose, 10.0 - l_nose), None))
        }
        other => Err(format!("--case anf|ogive, got `{other}`")),
    }
}

/// What the command line asks for.
struct Args {
    case: String,
    level: u32,
    precisions: Vec<(Precision, &'static str)>,
    mode: Mode,
    /// Print the `PERF` line (`--no-perf` for experiments that must not enter the ledger, e.g. with `LUMA_TRIDIAGONAL` set).
    perf: bool,
}

enum Mode {
    /// `--bench N`: iterations to time.
    Bench(usize),
    /// `--sweep --mach <list> --alpha <list>` (G2).
    Sweep { machs: Vec<f64>, alphas_deg: Vec<f64> },
}

fn parse_list(s: &str) -> Result<Vec<f64>, String> {
    s.split(',').map(|x| x.trim().parse::<f64>().map_err(|e| format!("`{x}`: {e}"))).collect()
}

fn parse(args: &[String]) -> Result<Args, String> {
    let flag = |name: &str| args.iter().any(|a| a == name);
    let value = |name: &str| args.iter().position(|a| a == name).map(|i| args.get(i + 1).cloned().ok_or_else(|| format!("{name} needs a value")));
    let case = value("--case").transpose()?.unwrap_or_else(|| "anf".into());
    body_of(&case)?;
    let level = value("--level").transpose()?.map_or(Ok(0), |l| l.parse::<u32>().map_err(|e| format!("--level: {e}")))?;
    if level > 1 {
        return Err("--level 0|1".into());
    }
    let mut precisions = Vec::new();
    for p in value("--precision").transpose()?.unwrap_or_else(|| "df32".into()).split(',') {
        precisions.push(match p.trim() {
            "f32" => (Precision::F32, "f32"),
            "df32" => (Precision::Extended, "df32"),
            other => return Err(format!("--precision f32|df32, got `{other}`")),
        });
    }
    let mode = if flag("--sweep") {
        let machs = parse_list(&value("--mach").transpose()?.ok_or("--sweep needs --mach <list>")?)?;
        let alphas_deg = parse_list(&value("--alpha").transpose()?.unwrap_or_else(|| "2,6".into()))?;
        Mode::Sweep { machs, alphas_deg }
    } else if let Some(n) = value("--bench") {
        Mode::Bench(n?.parse::<usize>().map_err(|e| format!("--bench: {e}"))?)
    } else {
        return Err("one of --bench N or --sweep".into());
    };
    Ok(Args { case, level, precisions, mode, perf: !flag("--no-perf") })
}

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

fn bench(args: &Args, n: usize) -> Result<(), String> {
    let (body, fins) = body_of(&args.case)?;
    let case = Case3::new(2.0, 2f64.to_radians(), body, fins, args.level);
    let t0 = Instant::now();
    let mut host = case.build_solver().map_err(|e| format!("grid: {e}"))?;
    let g = &host.grid;
    let cells = g.ni * g.nj * g.nk;
    eprintln!(
        "{} level {}: grid {} x {} x {} = {cells} cells, {} solid boxes, built in {:.2} s",
        args.case,
        args.level,
        g.ni,
        g.nj,
        g.nk,
        g.solid.len(),
        t0.elapsed().as_secs_f64()
    );
    // second-order reconstruction from the first timed iteration
    host.cfg.first_order_iters = 0;
    let ctx = luma::Context::new().map_err(|e| format!("no GPU context: {e}"))?;
    for &(precision, name) in &args.precisions {
        let mut s = GpuSolver3::from_host(&ctx, &host, precision).map_err(|e| format!("{name}: {e}"))?;
        // warm-up: compile, trace, 20 iterations (the CFL ramp restarts with every `set_state`)
        let t0 = Instant::now();
        s.run_replay(20).map_err(|e| format!("{name}: {e}"))?;
        s.sync().map_err(|e| format!("{name}: {e}"))?;
        eprintln!("{} level {} {name}: first call (compile + trace + 20 iterations) {:.2} s", args.case, args.level, t0.elapsed().as_secs_f64());
        let us = best_of(3, || {
            s.set_state(&host.u).expect("state upload");
            s.run_replay(20).expect("warm-up");
            s.sync().expect("sync");
            let t0 = Instant::now();
            s.run_replay(n).expect("replay");
            s.sync().expect("sync");
            1e6 * t0.elapsed().as_secs_f64() / n as f64
        });
        // the profiled step: one iteration as the `eval` loop launches it, after the warm-up
        s.set_state(&host.u).map_err(|e| format!("{name}: {e}"))?;
        s.run_replay(20).map_err(|e| format!("{name}: {e}"))?;
        let p = s.profile_step().map_err(|e| format!("{name}: {e}"))?;
        println!(
            "\n### {} L{} {name} - one implicit step as the eval loop launches it: {} kernel launches, {:.0} us GPU time, {} fusion cuts\n```\n{}```",
            args.case,
            args.level,
            p.launches(),
            p.total().as_secs_f64() * 1e6,
            p.splits.len(),
            compact(&p)
        );
        println!("{} L{} {name}: {us:.1} us / iteration (replay of {n} after 20 warm-up iterations, best of 3), {cells} cells", args.case, args.level);
        if args.perf {
            println!(
                "PERF {{\"case\":\"3d-{}-L{}-implicit\",\"precision\":\"{name}\",\"cells\":{cells},\"iters\":{n},\"us_per_iter\":{us:.3},\"launches_per_iter\":{},\"gpu_us_per_iter\":{:.3},\"wall_s\":null}}",
                args.case,
                args.level,
                p.launches(),
                p.total().as_secs_f64() * 1e6
            );
        }
    }
    Ok(())
}

/// G2: per Mach a cold first incidence, warm later ones (a warm point ending Failed/Stalled is re-run cold, a diverged cold run
/// once more with `cfl_max` halved, as `Analysis::run_lift`); one `RESULT {json}` line per point.
fn sweep(args: &Args, machs: &[f64], alphas_deg: &[f64]) -> Result<(), String> {
    use bowshock_core::engine::gpu3::GpuRunner3;
    use bowshock_core::engine::result::StopReason;
    let (body, fins) = body_of(&args.case)?;
    let (prec, pname) = args.precisions[0];
    let ok = |r: &bowshock_core::engine::run3::PointResult3| {
        r.stop_reason != StopReason::NonFinite && r.cn.is_finite() && r.ca.is_finite() && matches!(r.stop_reason, StopReason::ResidualDrop | StopReason::CdStationary | StopReason::MeanStationary | StopReason::BasePressureStationary | StopReason::ResidualPlateau)
    };
    for &m in machs {
        let mut runner: Option<GpuRunner3> = None;
        let mut warm = false;
        for &a in alphas_deg {
            let t0 = Instant::now();
            let case = Case3::new(m, a.to_radians(), body.clone(), fins.clone(), args.level);
            if runner.is_none() {
                runner = Some(GpuRunner3::new(&case, prec).map_err(|e| e.to_string())?);
            }
            let rn = runner.as_mut().unwrap();
            let mut r = None;
            if warm {
                let w = rn.run_on(&case, true).map_err(|e| e.to_string())?;
                if ok(&w) {
                    r = Some(w);
                }
            }
            let r = match r {
                Some(r) => r,
                None => {
                    let mut c = rn.run_on(&case, false).map_err(|e| e.to_string())?;
                    if c.stop_reason == StopReason::NonFinite {
                        let mut half = case.clone();
                        half.numerics.implicit.cfl_max *= 0.5;
                        c = rn.run_on(&half, false).map_err(|e| e.to_string())?;
                    }
                    c
                }
            };
            warm = r.cn.is_finite();
            let quality = if !r.cn.is_finite() || !r.ca.is_finite() || r.stop_reason == StopReason::NonFinite {
                "Failed"
            } else if r.stop_reason == StopReason::ResidualDrop {
                "Converged"
            } else if r.stop_reason == StopReason::MaxIterations {
                "Stalled"
            } else {
                "Stationary"
            };
            let f = |v: f64| if v.is_finite() { format!("{v}") } else { "null".into() };
            println!(
                "RESULT {{\"case\":\"{}\",\"level\":{},\"precision\":\"{pname}\",\"mach\":{m},\"alpha_deg\":{a},\"cn\":{},\"cn_alpha\":{},\"x_cp_cal\":{},\"ca\":{},\"quality\":\"{quality}\",\"stop\":\"{:?}\",\"drop_orders\":{},\"iterations\":{},\"wall_s\":{:.2}}}",
                args.case, args.level, f(r.cn), f(r.cn / a.to_radians()), f(r.x_cp), f(r.ca), r.stop_reason, f(r.drop_orders), r.iterations, t0.elapsed().as_secs_f64()
            );
        }
    }
    Ok(())
}

fn main() {
    let argv: Vec<String> = std::env::args().skip(1).collect();
    let result = parse(&argv).and_then(|args| match &args.mode {
        Mode::Bench(n) => bench(&args, *n),
        Mode::Sweep { machs, alphas_deg } => sweep(&args, machs, alphas_deg),
    });
    if let Err(e) = result {
        eprintln!("lift3d: {e}");
        std::process::exit(2);
    }
}
