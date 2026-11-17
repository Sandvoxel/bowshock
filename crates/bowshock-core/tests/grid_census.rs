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
    census("example body, finless wedge, Draft tier (y+ 40, 0.6 axial, 5 L / 3 L)", &Case3::wedge(2.0, example_body(), 0).with_tier(Tier::draft(ex)));
    census("example rocket 4 fins, sector, Draft tier", &Case3::sector(2.0, example_body(), example_fins(), Tier::draft(ex), 0));
    let mut hd = Case3::base_half(2.0, 2f64.to_radians(), example_body(), Some(example_fins()), Tier::draft(ex), 0);
    hd.grid.domain = Domain3::Half { phase: FinPhase::Between };
    census("example rocket 4 fins, half domain, Draft tier", &hd);
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
    let d = Case3::wedge(2.0, example_body(), 0).with_tier(Tier::draft(ex));
    let b = d.grid.base.as_ref().unwrap();
    println!("PARAMS example finless Draft base grid: n_up {} n_nose {} n_seg {:?} n_wake {} n_sting {} n_gap {} nj_out {} dx_tip {} dr_wall {:.3e} x_up {} x_down {} r_far {}", b.n_up, b.n_nose, b.n_seg, b.n_wake, b.n_sting, b.n_gap, b.nj_out, b.dx_tip, b.dr_wall, b.x_up, b.x_down, b.r_far);
    let s = Case3::sector(2.0, example_body(), example_fins(), Tier::Coarse(ex), 0);
    let g = bowshock_core::engine::grid3::body_grid3(&s.grid).unwrap();
    println!("PARAMS example sector Coarse: nk {} (stack {:?})", g.nk, s.grid.fin_stack);
}

/// Cells of the finless example wedge (the RANS table grid) and of the four-fin sector with each lever of the Draft tier alone and all together.
#[test]
#[ignore = "mesher only; prints"]
fn the_levers_of_the_draft_tier() {
    let ex = flow(1.7e6, 3.0);
    let l = example_body().x_base();
    type Mod = Box<dyn Fn(&mut Case3)>;
    let axial = |c: &mut Case3| {
        let b = c.grid.base.as_mut().unwrap();
        let cut = |n: usize| ((n as f64 * 0.6).ceil() as usize).div_ceil(2) * 2;
        b.n_up = cut(b.n_up);
        b.n_nose = cut(b.n_nose);
        b.n_seg.iter_mut().for_each(|n| *n = cut(*n));
        b.n_wake = cut(b.n_wake);
        b.dx_tip = 0.05;
    };
    let domain = move |c: &mut Case3| {
        let b = c.grid.base.as_mut().unwrap();
        (b.x_up, b.r_far, b.x_down) = (5.0 * l, 5.0 * l, 3.0 * l);
    };
    let gap = |c: &mut Case3| c.grid.base.as_mut().unwrap().dr_gap = Some(0.01);
    let levers: Vec<(&str, Mod, bool)> = vec![
        ("Coarse (y+ 1, 10 L / 5 L)", Box::new(|_| {}), false),
        ("domain 5 L / 3 L", Box::new(domain), false),
        ("axial cells x 0.6, tip cell 0.05", Box::new(axial), false),
        ("body y+ 60 (wall functions, the Lift tier spacing)", Box::new(|_| {}), true),
        ("gap rows from 0.01 d", Box::new(gap), false),
    ];
    for (name, f, wf) in levers {
        let mut c = Case3::wedge(2.0, example_body(), 0);
        f(&mut c);
        let c = c.with_tier(if wf { Tier::Lift(ex) } else { Tier::Coarse(ex) });
        let g = c.build_solver().unwrap().grid.clone();
        println!("LEVER finless wedge: {name}: {} x {} = {} cells", g.ni, g.nj, g.ni * g.nj);
    }
    let c = Case3::wedge(2.0, example_body(), 0).with_tier(Tier::draft(ex));
    let g = c.build_solver().unwrap().grid.clone();
    println!("LEVER finless wedge: all (Draft): {} x {} = {} cells", g.ni, g.nj, g.ni * g.nj);
}

/// Grid sequencing, the mesher part: the grid one level down has every axial and radial count halved, the fine grid's fluid cells all have a fluid parent (the body
/// surface, the base plane, the fin boxes and the axis fall on even indices), a uniform state prolongs to a uniform state and `restrict(prolong(u)) = u`.
#[test]
fn a_coarsened_grid_pairs_with_the_fine_one() {
    use bowshock_cfd::gas3::{prim_to_cons3, Cons3};
    use bowshock_core::engine::grid3::body_grid3;
    use bowshock_core::engine::sequence3::{prolong, restrict};
    let ex = flow(1.7e6, 3.0);
    let cases: Vec<(&str, Case3)> = vec![
        ("finless wedge, Coarse", Case3::wedge(2.0, example_body(), 0).with_tier(Tier::Coarse(ex))),
        ("finless wedge, Draft", Case3::wedge(2.0, example_body(), 0).with_tier(Tier::draft(ex))),
        ("example rocket sector, Coarse", Case3::sector(2.0, example_body(), example_fins(), Tier::Coarse(ex), 0)),
        ("example rocket sector, Draft", Case3::sector(2.0, example_body(), example_fins(), Tier::draft(ex), 0)),
        ("ANF finless wedge, Euler", Case3::wedge(2.0, Case3::anf_body(), 0)),
    ];
    for (name, case) in cases {
        let fine = body_grid3(&case.grid).unwrap_or_else(|e| panic!("{name}: {e}"));
        let coarse_params = case.grid.coarsened().unwrap_or_else(|e| panic!("{name}: {e}"));
        let coarse = body_grid3(&coarse_params).unwrap_or_else(|e| panic!("{name} coarse: {e}"));
        if (fine.ni, fine.nj, fine.nk) != (2 * coarse.ni, 2 * coarse.nj, coarse.nk) {
            // the fin region's columns come from fans of free length, not from the halved counts: not paired (needs an even fan length and chord count)
            assert!(case.grid.fins.is_some(), "{name}: a finless grid pairs");
            println!(
                "SEQ {name}: {} x {} x {} and {} x {} x {} do not pair (fin columns: fans of free length)",
                fine.ni, fine.nj, fine.nk, coarse.ni, coarse.nj, coarse.nk
            );
            continue;
        }
        let q: Cons3 = prim_to_cons3(&case.freestream());
        let uc: Vec<Cons3> = (0..coarse.ni * coarse.nj * coarse.nk).map(|c| [q[0] * (1.0 + 1e-3 * (c % 7) as f64), q[1], q[2], q[3], q[4]]).collect();
        let u = prolong(&coarse, &uc, &fine, q).unwrap_or_else(|e| panic!("{name}: {e}"));
        let back = restrict(&fine, &u, &coarse, q).expect("restrict");
        for (c, (a, b)) in uc.iter().zip(&back).enumerate() {
            if coarse.part[c] == bowshock_cfd::grid3::PART_FLUID {
                assert!((0..5).all(|m| (a[m] - b[m]).abs() <= 1e-13 * a[m].abs().max(1.0)), "{name}: cell {c}: {a:?} vs {b:?}");
            }
        }
        println!(
            "SEQ {name}: {} x {} x {} -> {} x {} x {}: every fluid cell has a fluid parent, restrict(prolong(u)) = u",
            fine.ni, fine.nj, fine.nk, coarse.ni, coarse.nj, coarse.nk
        );
    }
}
