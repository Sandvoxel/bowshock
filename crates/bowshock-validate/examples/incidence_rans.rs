//! The viscous finned half domain at incidence on the GPU to the engine's stop rule (`GpuRunner3::run_on`: residual drop, CN stationary, or the mean-stationary rule): CN,
//! `x_cp`, CA, iterations and the oscillation of the last averaging window, for the Lift tier (wall functions, `y+` 60) or the wall-resolved tiers.
//!
//! ```text
//! cargo run --release -p bowshock-validate --example incidence_rans -- [--mach 2.0] [--alpha 2.0] [--body short|anf] [--tier lift|coarse|fine] [--fins 1|0]
//!     [--chord-cells N] [--span-cells N] [--cfl-max X] [--sub-iters N] [--max-iter N] [--precision f32|df32] [--phase between|inplane] [--first-order N] [--fin-yp Y]
//! ```
//!
//! The grid is `Case3::base_half` as the G-L1 gate builds it (ANF fins, the sea-level `Re_c` of the 0.03 m ANF, the grid sized for Mach 3.147); `short` is the ANF nose on a
//! tube of 1.5 calibres. Prints one `INCIDENCE {json}` line; `--history` also prints the CN samples (every 50 iterations).

use bowshock_cfd::gpu::Precision;
use bowshock_core::engine::body_grid::{BaseBody, Body, Nose};
use bowshock_core::engine::case3::{Case3, Tier, WallFlow, SUTHERLAND_SEA_LEVEL};
use bowshock_core::engine::gpu3::GpuRunner3;
use bowshock_core::engine::grid3::{Domain3, FinPhase};
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
    let flow = WallFlow { re_c: 1.225 * 340.294 * 0.03 / 1.7894e-5, mach_max: 3.147, ks_body: 0.0, ks_fin: 0.0, sutherland: SUTHERLAND_SEA_LEVEL };
    let tier = match value("--tier").as_deref().unwrap_or("lift") {
        "lift" => Tier::Lift(flow),
        "coarse" => Tier::Coarse(flow),
        "fine" => Tier::Fine(flow),
        other => return Err(format!("--tier lift|coarse|fine, got `{other}`")),
    };
    let fins = (num("--fins", 1.0)? != 0.0).then(Case3::anf_fins);
    let (prec, pname) = match value("--precision").as_deref().unwrap_or("f32") {
        "f32" => (Precision::F32, "f32"),
        "df32" => (Precision::Extended, "df32"),
        other => return Err(format!("--precision f32|df32, got `{other}`")),
    };
    let mut case = Case3::base_half(mach, alpha.to_radians(), body, fins, tier, 0);
    let phase = if value("--phase").as_deref() == Some("inplane") { FinPhase::InPlane } else { FinPhase::Between };
    case.grid.domain = Domain3::Half { phase };
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
