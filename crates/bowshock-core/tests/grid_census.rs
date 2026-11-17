//! Cell budget of the product grids: ni x nj x nk, the cells that are never updated (the solid body block, the fin boxes), and where the fluid cells are (upstream, along the body,
//! in the wake; inside 2 body lengths or beyond). Mesher only, seconds. `cargo test -p bowshock-core --test grid_census -- --ignored --nocapture`.

use bowshock_core::engine::body_grid::{BaseBody, Body, Nose};
use bowshock_core::engine::case3::{Case3, Tier, WallFlow, SUTHERLAND_SEA_LEVEL};
use bowshock_core::engine::grid3::{Domain3, FinPhase, Fins3};
use bowshock_core::FinProfile;

fn census(name: &str, case: &Case3) {
    let s = case.build_solver().unwrap_or_else(|e| panic!("{name}: {e}"));
    let g = &s.grid;
    let n = g.ni * g.nj * g.nk;
    let xb = case.body.x_base();
    let l = xb;
    let (mut fluid, mut solid_body, mut solid_other, mut dead) = (0usize, 0usize, 0usize, 0usize);
    let (mut up, mut along, mut wake, mut far) = (0usize, 0usize, 0usize, 0usize);
    let mut axis_cols = 0usize;
    for c in 0..n {
        if g.part[c] != bowshock_cfd::grid3::PART_FLUID {
            if g.part[c] == bowshock_cfd::grid3::PART_DEAD {
                dead += 1;
                continue;
            }
            let p = format!("{:?}", g.parts.get((g.part[c] as usize).wrapping_sub(1)));
            if p.contains("Body") || p.contains("Sting") {
                solid_body += 1;
            } else {
                solid_other += 1;
            }
            continue;
        }
        fluid += 1;
        let x = g.xc[c];
        let r = x[1].hypot(x[2]);
        if x[0] < -2.0 * l || x[0] > xb + 2.0 * l || r > 2.0 * l {
            far += 1;
        } else if x[0] < 0.0 {
            up += 1;
        } else if x[0] <= xb {
            along += 1;
        } else {
            wake += 1;
        }
        if r < 0.05 {
            axis_cols += 1;
        }
    }
    let col_far = (0..g.ni).filter(|&i| g.xc[g.cell(i, 0, 0)][0] < -2.0 * l || g.xc[g.cell(i, 0, 0)][0] > xb + 2.0 * l).count();
    let row_far = (0..g.nj)
        .filter(|&j| {
            let x = g.xc[g.cell(g.ni / 2, j, 0)];
            x[1].hypot(x[2]) > 2.0 * l
        })
        .count();
    println!("CENSUS   {name}: of {} columns {col_far} lie beyond 2 L upstream / downstream, of {} rows {row_far} beyond 2 L radially", g.ni, g.nj);
    let pct = |a: usize| 100.0 * a as f64 / n as f64;
    println!(
        "CENSUS {name}: {} x {} x {} = {n} cells; fluid {fluid} ({:.1} %); solid body block {solid_body} ({:.1} %), dead (upstream of the tip) {dead} ({:.1} %), fin boxes {solid_other} ({:.1} %); fluid cells: upstream of the tip within 2 L {up} ({:.1} %), along the body {along} ({:.1} %), in the wake within 2 L {wake} ({:.1} %), beyond 2 L (x < -2 L, x > x_base + 2 L, r > 2 L) {far} ({:.1} %); axis-adjacent {axis_cols}",
        g.ni, g.nj, g.nk, pct(fluid), pct(solid_body), pct(dead), pct(solid_other), pct(up), pct(along), pct(wake), pct(far)
    );
}

fn example_body() -> BaseBody {
    BaseBody::cylinder(Body { nose: Nose::TangentOgive { fineness: 0.2286 / 0.0762 } }, 0.9134 / 0.0762)
}

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

fn gf1_fins() -> Fins3 {
    Fins3 {
        count: 4,
        root_chord: 0.5,
        tip_chord: 0.5,
        span: 0.5,
        sweep: 0.0,
        thickness: 0.01,
        te_to_base: 0.0,
        profile: FinProfile::DoubleWedge,
        planform: None,
    }
}

fn flow(re_c: f64, mach_max: f64) -> WallFlow {
    WallFlow { re_c, mach_max, ks_body: 0.0, ks_fin: 0.0, sutherland: SUTHERLAND_SEA_LEVEL }
}

#[test]
#[ignore = "mesher only; prints"]
fn the_cell_budget_of_the_product_grids() {
    let ex = flow(1.7e6, 3.0);
    census("example body, finless wedge, Coarse tier (the RANS table)", &Case3::wedge(2.0, example_body(), 0).with_tier(Tier::Coarse(ex)));
    census("example rocket 4 fins, sector, Coarse tier", &Case3::sector(2.0, example_body(), example_fins(), Tier::Coarse(ex), 0));
    let mut h = Case3::base_half(2.0, 2f64.to_radians(), example_body(), Some(example_fins()), Tier::Lift(ex), 0);
    h.grid.domain = Domain3::Half { phase: FinPhase::Between };
    census("example rocket 4 fins, half domain, Lift tier", &h);
    let f = flow(1.0e6, 2.0);
    census("G-F1 (ANF body, 4 thin fins), sector, Coarse tier", &Case3::sector(2.0, Case3::anf_body(), gf1_fins(), Tier::Coarse(f), 0));
    let mut h = Case3::base_half(2.0, 2f64.to_radians(), Case3::anf_body(), Some(gf1_fins()), Tier::Lift(f), 0);
    h.grid.domain = Domain3::Half { phase: FinPhase::Between };
    census("G-F1 (ANF body, 4 thin fins), half domain, Lift tier", &h);
    census("ANF default fins, sector, Coarse tier", &Case3::sector(2.0, Case3::anf_body(), Case3::anf_fins(), Tier::Coarse(f), 0));
    census("ANF finless wedge, Coarse tier", &Case3::wedge(2.0, Case3::anf_body(), 0).with_tier(Tier::Coarse(f)));
}

#[test]
#[ignore = "mesher only; prints"]
fn the_parameters_of_the_grids() {
    let ex = flow(1.7e6, 3.0);
    let c = Case3::wedge(2.0, example_body(), 0).with_tier(Tier::Coarse(ex));
    let b = c.grid.base.as_ref().unwrap();
    println!("PARAMS example finless Coarse base grid: n_up {} n_nose {} n_seg {:?} n_wake {} n_sting {} n_gap {} nj_out {} dx_tip {} dr_wall {:.3e} x_up {} x_down {} r_far {}", b.n_up, b.n_nose, b.n_seg, b.n_wake, b.n_sting, b.n_gap, b.nj_out, b.dx_tip, b.dr_wall, b.x_up, b.x_down, b.r_far);
    let e = Case3::wedge(2.0, example_body(), 0);
    let b = e.grid.base.as_ref().unwrap();
    println!(
        "PARAMS example finless Euler base grid: n_up {} n_nose {} n_seg {:?} n_wake {} n_sting {} n_gap {} nj_out {} dx_tip {} dr_wall {:.3e}",
        b.n_up, b.n_nose, b.n_seg, b.n_wake, b.n_sting, b.n_gap, b.nj_out, b.dx_tip, b.dr_wall
    );
    let s = Case3::sector(2.0, example_body(), example_fins(), Tier::Coarse(ex), 0);
    let g = bowshock_core::engine::grid3::body_grid3(&s.grid).unwrap();
    println!("PARAMS example sector Coarse: nk {} (stack {:?})", g.nk, s.grid.fin_stack);
}
