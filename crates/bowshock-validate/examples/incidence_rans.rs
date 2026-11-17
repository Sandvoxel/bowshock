//! The viscous finned half domain at incidence on the GPU to the engine's stop rule (`GpuRunner3::run_on`: residual drop, CN stationary, or the mean-stationary rule): CN,
//! `x_cp`, CA, iterations and the oscillation of the last averaging window, for the Lift tier (wall functions, `y+` 60) or the wall-resolved tiers.
//!
//! ```text
//! cargo run --release -p bowshock-validate --example incidence_rans -- [--mach 2.0] [--alpha 2.0] [--body short|anf] [--tier lift|coarse|fine] [--fins 1|0]
//!     [--chord-cells N] [--span-cells N] [--cfl-max X] [--sub-iters N] [--max-iter N] [--precision f32|df32] [--phase between|inplane] [--first-order N] [--fin-yp Y]
//!     [--domain half|sector] [--sector-fins gf1|anf] [--history-every N]
//! ```
//!
//! The grid is `Case3::base_half` as the G-L1 gate builds it (ANF fins, the sea-level `Re_c` of the 0.03 m ANF, the grid sized for Mach 3.147); `short` is the ANF nose on a
//! tube of 1.5 calibres. Prints one `INCIDENCE {json}` line; `--history` also prints the CN samples (every 50 iterations). `--domain sector` is the finned sector at zero incidence
//! as the G-F1 gate builds it (four thin fins, t/c 2 %, chord and span 0.5 d, `Re_c` 1e6, the grid sized for Mach 2; `--sector-fins anf` takes the ANF fins and flow).
//! `--history-every N` (a multiple of 50) replaces the stop rule: a cold run to `--max-iter` printing, every N iterations, `H it residual CA CN nose afterbody fin_sides fin_edges
//! fin_tips base base_friction_nose+aft fin_friction` (the parts with pressure and shear as `LiftBreakdown`, base and the friction of `DragParts`).

use bowshock_cfd::gpu::Precision;
use bowshock_cfd::implicit_params::{ImplicitParams, LineSolve3};
use bowshock_cfd::solver3::{Config3, Scheme3};
use bowshock_core::engine::body_grid::{BaseBody, Body, Nose};
use bowshock_core::engine::case3::{Case3, Tier, WallFlow, SUTHERLAND_SEA_LEVEL};
use bowshock_core::engine::gpu3::{GpuRunner3, SYNC};
use bowshock_core::engine::grid3::{Domain3, FinPhase, Fins3};
use bowshock_core::FinProfile;
use std::time::Instant;

fn main() -> Result<(), String> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let value = |name: &str| args.iter().position(|a| a == name).and_then(|i| args.get(i + 1)).cloned();
    let num = |name: &str, default: f64| value(name).map_or(Ok(default), |v| v.parse::<f64>().map_err(|e| format!("{name}: {e}")));
    let (mach, alpha) = (num("--mach", 2.0)?, num("--alpha", 2.0)?);
    let body = match value("--body").as_deref().unwrap_or("short") {
        "short" => BaseBody::cylinder(Body { nose: Nose::Cone { half_angle: 10f64.to_radians() } }, 1.5),
        "anf" => Case3::anf_body(),
        other => return Err(format!("--body short|anf, got `{other}`")),
    };
    let sector = value("--domain").as_deref() == Some("sector");
    let thin = sector && value("--sector-fins").as_deref() != Some("anf");
    let flow = if thin {
        WallFlow { re_c: 1.0e6, mach_max: 2.0, ks_body: 0.0, ks_fin: 0.0, sutherland: 0.368 }
    } else {
        WallFlow { re_c: 1.225 * 340.294 * 0.03 / 1.7894e-5, mach_max: 3.147, ks_body: 0.0, ks_fin: 0.0, sutherland: SUTHERLAND_SEA_LEVEL }
    };
    let tier = match value("--tier").as_deref().unwrap_or("lift") {
        "lift" => Tier::Lift(flow),
        "coarse" => Tier::Coarse(flow),
        "fine" => Tier::Fine(flow),
        other => return Err(format!("--tier lift|coarse|fine, got `{other}`")),
    };
    let fins = if thin {
        let (chord, tc) = (0.5, 0.02);
        Some(Fins3 {
            count: 4,
            root_chord: chord,
            tip_chord: chord,
            span: 0.5,
            sweep: 0.0,
            thickness: tc * chord,
            te_to_base: 0.0,
            profile: FinProfile::DoubleWedge,
            planform: None,
        })
    } else {
        (num("--fins", 1.0)? != 0.0).then(Case3::anf_fins)
    };
    let (prec, pname) = match value("--precision").as_deref().unwrap_or("f32") {
        "f32" => (Precision::F32, "f32"),
        "df32" => (Precision::Extended, "df32"),
        other => return Err(format!("--precision f32|df32, got `{other}`")),
    };
    let mut case = if sector {
        Case3::sector(mach, body, fins.ok_or("--domain sector needs fins")?, tier, 0)
    } else {
        let mut c = Case3::base_half(mach, alpha.to_radians(), body, fins, tier, 0);
        let phase = if value("--phase").as_deref() == Some("inplane") { FinPhase::InPlane } else { FinPhase::Between };
        c.grid.domain = Domain3::Half { phase };
        c
    };
    if let Some(n) = value("--chord-cells") {
        case.grid.fin_chord_cells = Some(n.parse().map_err(|e| format!("--chord-cells: {e}"))?);
    }
    if let Some(n) = value("--span-cells") {
        case.grid.span_cells = Some(n.parse().map_err(|e| format!("--span-cells: {e}"))?);
    }
    // the fin stack at another `y+` than the tier's (the tier sets the body `y+` and the stack: Coarse 1 / 15, Lift 60 / 60, Fine 1 / 1)
    if let (Some(v), Some(f)) = (value("--fin-yp"), case.grid.fins.as_ref()) {
        let yp: f64 = v.parse().map_err(|e| format!("--fin-yp: {e}"))?;
        case.grid.fin_stack = Some(bowshock_core::engine::grid3::FinStack::from_flow(flow.re_c, flow.mach_max, f.root_chord, yp));
    }
    if let Some(v) = value("--cfl-max") {
        case.numerics.implicit.cfl_max = v.parse().map_err(|e| format!("--cfl-max: {e}"))?;
    }
    if let Some(v) = value("--sub-iters") {
        case.numerics.implicit.sub_iters = v.parse().map_err(|e| format!("--sub-iters: {e}"))?;
    }
    if let Some(v) = value("--max-iter") {
        case.numerics.max_iter = v.parse().map_err(|e| format!("--max-iter: {e}"))?;
    }
    if let Some(v) = value("--first-order") {
        case.numerics.first_order_iters = v.parse().map_err(|e| format!("--first-order: {e}"))?;
    }
    let mut runner = GpuRunner3::new(&case, prec).map_err(|e| format!("{e}"))?;
    if let Some(v) = value("--history-every") {
        return history(&mut runner, &case, v.parse().map_err(|e| format!("--history-every: {e}"))?);
    }
    let t0 = Instant::now();
    let r = runner.run_on(&case, false).map_err(|e| format!("{e}"))?;
    let wall = t0.elapsed().as_secs_f64();
    // CN over the last 1000 iterations of the 50-iteration samples: the oscillation of the monitor
    let tail: Vec<f64> = r.cn_history.iter().filter(|(it, _)| *it + 1000 > r.iterations).map(|&(_, c)| c).collect();
    let (lo, hi) = tail.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |(l, h), &c| (l.min(c), h.max(c)));
    let w = r.window;
    println!(
        "INCIDENCE {{\"mach\":{mach},\"alpha_deg\":{alpha},\"tier\":\"{}\",\"cfl_max\":{},\"sub_iters\":{},\"precision\":\"{pname}\",\"iterations\":{},\"stop\":\"{:?}\",\"drop_orders\":{:.2},\"cn\":{:.5},\"ca\":{:.5},\"x_cp\":{:.4},\"window\":{},\"cn_last1000_half_range_rel\":{:.4},\"y_plus_max\":{:.1},\"wall_s\":{wall:.1},\"ms_per_iter\":{:.2}}}",
        value("--tier").unwrap_or_else(|| "lift".into()),
        case.numerics.implicit.cfl_max,
        case.numerics.implicit.sub_iters,
        r.iterations,
        r.stop_reason,
        r.drop_orders,
        r.cn,
        r.ca,
        r.x_cp,
        w.map_or("null".to_string(), |w| format!(
            "{{\"end\":{},\"length\":{},\"ca\":{:.5},\"cn\":{:.5},\"x_cp\":{:.4},\"oscillation\":{:.4}}}",
            w.end, w.length, w.ca, w.cn, w.x_cp, w.oscillation
        )),
        if tail.is_empty() { f64::NAN } else { 0.5 * (hi - lo) / (0.5 * (hi + lo)).abs().max(1e-300) },
        r.y_plus_max,
        1e3 * wall / r.iterations.max(1) as f64
    );
    if args.iter().any(|a| a == "--history") {
        for (it, c) in &r.cn_history {
            println!("CN {it} {c:.6}");
        }
    }
    Ok(())
}

/// A cold run without a stop rule to `max_iter`, set up as `GpuRunner3::run_on` sets it up (config, reset), printing the residual and the force parts every `every` iterations.
fn history(runner: &mut GpuRunner3, case: &Case3, every: usize) -> Result<(), String> {
    let n = &case.numerics;
    let g = &runner.host.grid;
    println!("CELLS {} ({} x {} x {})", g.ni * g.nj * g.nk, g.ni, g.nj, g.nk);
    let cfg = Config3 {
        freestream: case.freestream(),
        cfl: runner.host.cfg.cfl,
        limiter: n.limiter,
        first_order_iters: n.first_order_iters,
        low_mach: n.low_mach,
        scheme: Scheme3::Implicit(ImplicitParams { line_solve: LineSolve3::Block, ..n.implicit_for(true) }),
    };
    runner.gpu.set_config(&cfg).map_err(|e| format!("{e}"))?;
    runner.host.cfg = cfg;
    runner.gpu.reset().map_err(|e| format!("{e}"))?;
    runner.gpu.context().sync().map_err(|e| format!("{e}"))?;
    let t0 = Instant::now();
    let mut done = 0;
    while done < n.max_iter {
        let len = SYNC.min(n.max_iter - done);
        let h = runner.gpu.run(len).map_err(|e| format!("{e}"))?;
        done += len;
        if h.iter().any(|r| !r.is_finite()) {
            println!("NONFINITE {done}");
            break;
        }
        if done.is_multiple_of(every) {
            let f = runner.lift_forces(case).map_err(|e| format!("{e}"))?;
            let b = runner.breakdown(case).map_err(|e| format!("{e}"))?;
            let d = runner.drag_parts(case).map_err(|e| format!("{e}"))?;
            println!(
                "H {done} {:.4e} {:.5} {:.5} {:.5} {:.5} {:.5} {:.5} {:.5} {:.5} {:.5} {:.5} ({:.1} s)",
                h.last().copied().unwrap_or(f64::NAN),
                f.ca,
                f.cn,
                b.nose.ca,
                b.afterbody.ca,
                b.fin_sides.ca,
                b.fin_edges.ca,
                b.fin_tips.ca,
                d.base,
                d.body_friction(),
                d.fin_friction(),
                t0.elapsed().as_secs_f64()
            );
        }
    }
    Ok(())
}
