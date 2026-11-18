//! `finned_table`: one point of the finned example rocket's zero-incidence table on the GPU: the four-fin sector (`Case3::sector`) of the example body at `mach`, on the
//! Coarse tier, the Draft tier (`--tier draft`) or the Draft tier's safe subsonic variant (`--tier draft-safe`: `Case3::draft_subsonic`), `GpuRunner3` in f32, to the case's stop rule.
//!
//! ```text
//! cargo run --release -p bowshock-validate --example finned_table -- --tier coarse|draft|draft-safe --mach 2.0 [--anf] [--gf1] [--fin-cells N] [--chord N] [--te-to-base X (with --anf)] [--sub N] [--max-iter N] [--ddadi OMEGA] [--dis-boost DELTA] [--history-every N] [--dt-mode sum|min]
//! ```
//!
//! One `TABLE` line: tier, Mach number, grid, cells, iterations, stop reason, wall seconds (the whole run, set-up included), ms per iteration, CD total and its parts (coefficients on pi/4
//! of the sector, the 4-fin rocket): body friction (nose + afterbody), fin friction, forebody pressure (nose), afterbody pressure, base, fin pressure (sides + edges + tips). The device
//! memory peak is `tools/gpuq/gpumem`'s (`--label`), not printed here.

use bowshock_cfd::gpu::Precision;
use bowshock_cfd::implicit_params::DtMode;
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
    if args.iter().any(|a| a == "--anf") {
        // the G-F5 case on the sector: the Army-Navy Basic Finner with the sourced fin section, sea-level Reynolds number of the 30 mm model, grid sized for Mach 3.147
        let flow = WallFlow { re_c: 1.225 * 340.294 * 0.03 / 1.7894e-5, mach_max: 3.147, ks_body, ks_fin, sutherland: 0.368 };
        let tier = if tier_name == "fine" { Tier::Fine(flow) } else { Tier::Coarse(flow) };
        // `--te-to-base X`: the fin's root trailing edge X calibers ahead of the base plane (0 is flush, the sourced Finner), to see the trailing-edge face's pressure with the fin off the base
        let mut fins = Case3::anf_fins();
        if let Some(x) = value("--te-to-base") {
            fins.te_to_base = x.parse().map_err(|e| format!("--te-to-base: {e}"))?;
        }
        case = Case3::sector(mach, Case3::anf_body(), fins, tier, 0);
    }
    if tier_name == "draft-safe" {
        case = case.draft_subsonic();
    }
    // `--fin-cells N`: N cells across the fin thickness (`Grid3Params::fin_cells_across`; a half fin in the sector owns N / 2, at least one)
    if let Some(n) = value("--fin-cells") {
        case = case.with_fin_cells_across(n.parse().map_err(|e| format!("--fin-cells: {e}"))?);
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
    // DDADI (`ImplicitParams::ddadi`, the diagonally dominant block factors, correction x omega) and the boost of the left-hand side dissipation (`lhs_dis_boost`)
    if let Some(w) = value("--ddadi") {
        case.numerics.implicit.ddadi = Some(w.parse().map_err(|e| format!("--ddadi: {e}"))?);
    }
    if let Some(d) = value("--dis-boost") {
        case.numerics.implicit.lhs_dis_boost = d.parse().map_err(|e| format!("--dis-boost: {e}"))?;
    }
    // `--dt-mode sum|min`: `DtMode::Sum` (`CFL V / (L_i + L_j + L_k)`) or `DtMode::MinDirection` (`CFL V / (3 min L_d)`, the weakest direction sizes the step)
    match value("--dt-mode").as_deref() {
        None => {}
        Some("sum") => case.numerics.implicit.dt_mode = DtMode::Sum,
        Some("min") => case.numerics.implicit.dt_mode = DtMode::MinDirection,
        Some(other) => return Err(format!("--dt-mode sum|min, not {other}")),
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
    if args.iter().any(|a| a == "--no-control") {
        // the fixed cap, no rollback (the controller's failures show as `NonFinite`)
        case.numerics.cfl_control = false;
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
    let precision = match value("--precision").as_deref() {
        None | Some("f32") => Precision::F32,
        Some("df32") => Precision::Extended,
        Some(other) => return Err(format!("--precision f32|df32, not {other}")),
    };
    let mut run = GpuRunner3::new(&case, precision).map_err(|e| e.to_string())?;
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
    {
        // the fin's leading / trailing edge faces: their projected area (sector units) and the area-mean pressure coefficient, cells across the fin
        let g = &run.host.grid;
        let (mut a_le, mut a_te, mut cells, mut across) = (0.0, 0.0, 0usize, 0usize);
        for b in g.solid.iter().filter(|b| matches!(g.part_of(b.i.start, b.j.start, b.k.start), Some(bowshock_cfd::grid3::Part::Fin(_)))) {
            across += b.k.len();
            for k in b.k.clone() {
                for j in b.j.clone() {
                    a_le += g.iface[(k * g.nj + j) * (g.ni + 1) + b.i.start].n[0].abs() * g.iface[(k * g.nj + j) * (g.ni + 1) + b.i.start].area;
                    a_te += g.iface[(k * g.nj + j) * (g.ni + 1) + b.i.end].n[0].abs() * g.iface[(k * g.nj + j) * (g.ni + 1) + b.i.end].area;
                    cells += b.i.len();
                }
            }
        }
        // coefficients are on pi/4 of the whole body, the faces' area is of this domain: cd = (2 pi / span) F / (q pi/4), F = cd q pi\/4 span\/(2 pi), F = -Cp q A on a trailing face (the outward normal is +x), +Cp q A on a leading one
        let f_unit = std::f64::consts::FRAC_PI_4 * g.theta_span / (2.0 * std::f64::consts::PI);
        println!(
            "FINFACES {tier_name} M{mach}: fin cells across (domain) {across} (full thickness {}), solid fin cells {cells}; LE face cd {:.5} area {a_le:.6} mean Cp {:.4}; TE face cd {:.5} area {a_te:.6} mean Cp {:.4}; sides {:.5} tips {:.5} fin friction {:.5}",
            if g.theta_span < 3.0 { 2 * across } else { across },
            d.fin_le,
            d.fin_le * f_unit / a_le,
            d.fin_te,
            -d.fin_te * f_unit / a_te,
            d.fin_sides,
            d.fin_tips,
            d.fin_friction()
        );
    }
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
    // `--history-every N`: the sampled CA (CN with incidence) every N iterations (a multiple of the 50-iteration sampling) of the accepted path, one line each
    if let Some(n) = value("--history-every") {
        let n: usize = n.parse().map_err(|e| format!("--history-every: {e}"))?;
        for (it, ca) in p.cn_history.iter().filter(|(it, _)| it % n.max(1) == 0) {
            println!("HIST {tier_name} M{mach}: {it} {ca:.6}");
        }
    }
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
