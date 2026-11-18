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

/// The finned example rocket's fins: THREE square fins (the example document and the frozen 2D fixture; `tests/grid_census.rs` has four, the first version of this helper copied it).
fn example_fins() -> Fins3 {
    let d = 0.0762;
    Fins3 {
        count: 3,
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
    // the roughness heights of the body and the fins (nose diameters, `WallFlow::ks_*`; the app passes roughness_m / d)
    let ks_body: f64 = value("--ks-body").map_or(Ok(0.0), |v| v.parse()).map_err(|e| format!("--ks-body: {e}"))?;
    let ks_fin: f64 = value("--ks-fin").map_or(Ok(0.0), |v| v.parse()).map_err(|e| format!("--ks-fin: {e}"))?;
    // the table's grid is sized for its highest Mach number (3) at the example rocket's Reynolds number (120 m)
    let flow = WallFlow { re_c: 1.7e6, mach_max: 3.0, ks_body, ks_fin, sutherland: SUTHERLAND_SEA_LEVEL };
    let tier = match tier_name.as_str() {
        "coarse" => Tier::Coarse(flow),
        "fine" => Tier::Fine(flow),
        _ => Tier::draft(flow),
    };
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
        let flow = WallFlow { re_c: 1.0e6, mach_max: 2.0, ks_body, ks_fin, sutherland: 0.368 };
        let tier = if tier_name == "fine" { Tier::Fine(flow) } else { Tier::Coarse(flow) };
        case = Case3::sector(mach, Case3::anf_body(), fins, tier, 0);
    }
    if tier_name == "draft-safe" {
        case = case.draft_subsonic();
    }
    if let Some(s) = value("--sub") {
        case.numerics.implicit.sub_iters = s.parse().map_err(|e| format!("--sub: {e}"))?;
    }
    if let Some(n) = value("--chord") {
        case.grid.fin_chord_cells = Some(n.parse().map_err(|e| format!("--chord: {e}"))?);
    }
    if let Some(c) = value("--cfl-max") {
        case.numerics.implicit.cfl_max = c.parse().map_err(|e| format!("--cfl-max: {e}"))?;
    }
    if let Some(c) = value("--cfl-ref") {
        case.numerics.cfl_ref = c.parse().map_err(|e| format!("--cfl-ref: {e}"))?;
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
            "TABLE-CPU {tier_name} M{mach}: {ni}x{nj}x{nk} cells, {} iterations ({:?}), {:.1} s; CD {:.5} (body friction {:.5}, fin friction {:.5}, base {:.5}, forebody pressure {:.5}, fin pressure {:.5})",
            p.iterations,
            p.stop_reason,
            t.elapsed().as_secs_f64(),
            d.total,
            d.body_friction(),
            d.fin_friction(),
            d.base,
            d.nose,
            d.fin_sides + d.fin_edges + d.fin_tips
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
    if args.iter().any(|a| a == "--gf1") {
        // the G-F1 reference (flat plate from the leading edge, fully turbulent, Re_c 1e6): the gate's fin friction of Mach 0.5 and 2
        let reference = if mach < 1.0 { 0.014593 } else { 0.008477 };
        println!("GF1-RATIO {tier_name} M{mach}: fin friction {:.6} / reference {reference} = {:.4}", d.fin_friction(), d.fin_friction() / reference);
    }
    println!(
        "TABLE-RUN {tier_name} M{mach}: cfl_max_start {} cfl_max_end {} cfl_ref {} sub_iters {} accepted {} wasted {} wall_s {wall:.1} stop {:?} ca {:.5} window_osc {}",
        case.numerics.implicit.cfl_max,
        p.final_cfl_cap().unwrap_or(case.numerics.implicit.cfl_max),
        case.numerics.cfl_ref,
        case.numerics.implicit.sub_iters,
        p.iterations,
        p.wasted_iterations(),
        p.stop_reason,
        p.ca,
        p.window.map_or("null".to_string(), |w| format!("{:.4}", w.oscillation))
    );
    if !p.cfl_events.is_empty() {
        let list: Vec<String> = p
            .cfl_events
            .iter()
            .map(|e| format!("{}: cfl {:.1} -> cap {:.1} -> {:.1}, back to {}", e.iteration, e.cfl_reached, e.cfl_before, e.cfl_after, e.restored_to))
            .collect();
        println!(
            "TABLE-ROLLBACKS {tier_name} M{mach}: {} (iteration: CFL reached, cfl_max before -> after, restored to; {} iterations wasted): {}",
            p.cfl_events.len(),
            p.wasted_iterations(),
            list.join("; ")
        );
    }
    if args.iter().any(|a| a == "--worst") {
        // where the field is worst (run to a `--max-iter` before the blow-up): the fluid cells of lowest pressure and density, with their indices and centroids
        let u = run.gpu.state_f64().map_err(|e| e.to_string())?;
        let g = &run.host.grid;
        let fs = run.host.cfg.freestream;
        let mut cells: Vec<(f64, f64, usize)> = (0..u.len())
            .filter(|&c| g.part[c] == bowshock_cfd::grid3::PART_FLUID)
            .map(|c| {
                let q = u[c];
                let p = (bowshock_cfd::gas3::GAMMA - 1.0) * (q[4] - 0.5 * (q[1] * q[1] + q[2] * q[2] + q[3] * q[3]) / q[0]);
                (p / fs[4], q[0] / fs[0], c)
            })
            .collect();
        for (name, key) in [("p/p_inf", 0usize), ("rho/rho_inf", 1)] {
            cells.sort_by(|a, b| (if key == 0 { a.0 } else { a.1 }).total_cmp(&(if key == 0 { b.0 } else { b.1 })));
            for &(p, r, c) in cells.iter().take(8) {
                let (i, j, k) = (c % g.ni, (c / g.ni) % g.nj, c / (g.ni * g.nj));
                let x = g.xc[c];
                println!("WORST lowest {name}: cell ({i},{j},{k}) x = ({:.4}, {:.4}, {:.4}) p/p_inf {p:.4} rho/rho_inf {r:.4}", x[0], x[1], x[2]);
            }
        }
    }
    Ok(())
}
