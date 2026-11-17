//! `finned_table`: one point of the finned example rocket's zero-incidence table on the GPU: the four-fin sector (`Case3::sector`) of the example body at `mach`, on the
//! Coarse tier, the Draft tier (`--tier draft`) or the Draft tier's safe subsonic variant (`--tier draft-safe`: `Case3::draft_subsonic`), `GpuRunner3` in f32, to the case's stop rule.
//!
//! ```text
//! cargo run --release -p bowshock-validate --example finned_table -- --tier coarse|draft|draft-safe --mach 2.0 [--sub N] [--max-iter N]
//! ```
//!
//! One `TABLE` line: tier, Mach number, grid, cells, iterations, stop reason, wall seconds (the whole run, set-up included), ms per iteration, CD total and its parts (coefficients on pi/4
//! of the sector, the 4-fin rocket): body friction (nose + afterbody), fin friction, forebody pressure (nose), afterbody pressure, base, fin pressure (sides + edges + tips). The device
//! memory peak is `tools/gpuq/gpumem`'s (`--label`), not printed here.

use bowshock_cfd::gpu::Precision;
use bowshock_core::engine::body_grid::{BaseBody, Body, Nose};
use bowshock_core::engine::case3::{Case3, Tier, WallFlow, SUTHERLAND_SEA_LEVEL};
use bowshock_core::engine::gpu3::GpuRunner3;
use bowshock_core::engine::grid3::Fins3;
use bowshock_core::FinProfile;
use std::time::Instant;

fn example_body() -> BaseBody {
    BaseBody::cylinder(Body { nose: Nose::TangentOgive { fineness: 0.2286 / 0.0762 } }, 0.9134 / 0.0762)
}

/// The finned example rocket's fins (`tests/grid_census.rs`).
fn example_fins() -> Fins3 {
    let d = 0.0762;
    Fins3 {
        count: 4,
        root_chord: 0.12 / d,
        tip_chord: 0.05 / d,
        span: 0.0762 / d,
        sweep: 0.06 / d,
        thickness: 0.00318 / d,
        te_to_base: 0.0,
        profile: FinProfile::Square,
        planform: None,
    }
}

fn main() -> Result<(), String> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let value = |name: &str| args.iter().position(|a| a == name).and_then(|i| args.get(i + 1)).cloned();
    let mach: f64 = value("--mach").ok_or("--mach <M>")?.parse().map_err(|e| format!("--mach: {e}"))?;
    let tier_name = value("--tier").unwrap_or_else(|| "coarse".into());
    // the table's grid is sized for its highest Mach number (3) at the example rocket's Reynolds number (120 m)
    let flow = WallFlow { re_c: 1.7e6, mach_max: 3.0, ks_body: 0.0, ks_fin: 0.0, sutherland: SUTHERLAND_SEA_LEVEL };
    let tier = if tier_name == "coarse" { Tier::Coarse(flow) } else { Tier::draft(flow) };
    let mut case = Case3::sector(mach, example_body(), example_fins(), tier, 0);
    if args.iter().any(|a| a == "--gf1") {
        // the G-F1 case (`bench3 --case sector-rans`): the Army-Navy body, four thin double-wedge fins, Re_c 1e6, Coarse tier sized for Mach 2
        let fins = Fins3 {
            count: 4,
            root_chord: 0.5,
            tip_chord: 0.5,
            span: 0.5,
            sweep: 0.0,
            thickness: 0.01,
            te_to_base: 0.0,
            profile: FinProfile::DoubleWedge,
            planform: None,
        };
        let flow = WallFlow { re_c: 1.0e6, mach_max: 2.0, ks_body: 0.0, ks_fin: 0.0, sutherland: 0.368 };
        case = Case3::sector(mach, Case3::anf_body(), fins, Tier::Coarse(flow), 0);
    }
    if tier_name == "draft-safe" {
        case = case.draft_subsonic();
    }
    if let Some(s) = value("--sub") {
        case.numerics.implicit.sub_iters = s.parse().map_err(|e| format!("--sub: {e}"))?;
    }
    if let Some(c) = value("--cfl-max") {
        case.numerics.implicit.cfl_max = c.parse().map_err(|e| format!("--cfl-max: {e}"))?;
    }
    if let Some(n) = value("--first-order") {
        case.numerics.first_order_iters = n.parse().map_err(|e| format!("--first-order: {e}"))?;
    }
    if let Some(n) = value("--max-iter") {
        case.numerics.max_iter = n.parse().map_err(|e| format!("--max-iter: {e}"))?;
    }
    if args.iter().any(|a| a == "--cpu") {
        // the f64 CPU solver on the same case (a cross-check of a GPU failure: slow, use `--max-iter`)
        let t = Instant::now();
        let mut s = case.build_solver().map_err(|e| e.to_string())?;
        let (ni, nj, nk) = (s.grid.ni, s.grid.nj, s.grid.nk);
        let p = bowshock_core::engine::run3::run3_on(&case, &mut s, false);
        let d = bowshock_core::engine::run3::drag_parts_on(&case, &s);
        println!(
            "TABLE-CPU {tier_name} M{mach}: {ni}x{nj}x{nk} cells, {} iterations ({:?}), {:.1} s; CD {:.5} (friction {:.5}, base {:.5}, nose {:.5})",
            p.iterations,
            p.stop_reason,
            t.elapsed().as_secs_f64(),
            d.total,
            d.friction.total,
            d.base,
            d.nose
        );
        return Ok(());
    }
    let t = Instant::now();
    let mut run = GpuRunner3::new(&case, Precision::F32).map_err(|e| e.to_string())?;
    let g = &run.host.grid;
    let (ni, nj, nk) = (g.ni, g.nj, g.nk);
    let setup = t.elapsed().as_secs_f64();
    let p = run.run_on(&case, false).map_err(|e| e.to_string())?;
    let d = run.drag_parts(&case).map_err(|e| e.to_string())?;
    let wall = t.elapsed().as_secs_f64();
    println!(
        "TABLE {tier_name} M{mach}: {ni}x{nj}x{nk} = {} cells, {} iterations ({:?}), {wall:.1} s (set-up {setup:.1} s), {:.2} ms/it; CD {:.5} = body friction {:.5} + fin friction {:.5} + forebody pressure {:.5} + afterbody pressure {:.5} + base {:.5} + fin pressure {:.5}; y+ max {:.2}",
        ni * nj * nk,
        p.iterations,
        p.stop_reason,
        1e3 * (wall - setup) / p.iterations.max(1) as f64,
        d.total,
        d.body_friction(),
        d.fin_friction(),
        d.nose,
        d.aft.iter().sum::<f64>(),
        d.base,
        d.fin_sides + d.fin_edges + d.fin_tips,
        p.y_plus_max
    );
    Ok(())
}
