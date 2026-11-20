//! Subsonic drag diagnosis (DESIGN3D item 62): the Arcas long model without fins at Mach 0.6 / 0.8 (NASA TN D-4013 Fig. 12, `C_A,corr` 0.250 / 0.2525) on the axisymmetric wedge of the
//! Coarse tier, the parts, the local skin friction and pressure along the body, and the variants of the setup that could explain the +16 % of `gate_x4`.
//!
//! `SD_MACHS` (comma list, default `0.6,0.8`), `SD_VARIANTS` (comma list, default `base`):
//! - `base`: the gate's case (fully turbulent, no sting, tangent ogive nose);
//! - `trip`: laminar upstream of the carborundum band of the test, 1.25 in aft of the nose tip;
//! - `sting`: the 1 in sting of the test behind the 1.47 in base (the sting is part of the wall; its wetted area is not counted in the force);
//! - `ref15`, `ref20`: the grid refined by 1.5 / 2 (cell counts, wall spacing);
//! - `xtr<N>`: laminar to N calibres from the tip (a trip that does not work: natural transition), turbulent behind;
//! - `short`: the short model (data `arcas::TRANSONIC`); `euler`: the Euler tier (pressure only);
//! - `lip`: the base as drawn in D-4014 Fig. 1(a) detail A: a 15 degree tail cone to a 1.308 in diameter (1.757 in long) and the reflexed lip, a 0.053 in step back out to the 1.470 in base
//!   (the gate's boattail is the straight 12.2 degree cone to the 1.470 in base), the lip a one-cell `BaseBody::with_step`;
//! - tokens combine with `+` (`ref15+trip+sting`).
//!
//! Prints `SD <variant> M<mach> ...` lines (`parts`, `y+`, `cfx` / `cpx` binned along the body) for `tools/subdrag_post.py`. Needs `BOWSHOCK_WEDGE_BACKEND=gpu`, `BOWSHOCK_FIN_TIER=coarse`.
#[path = "fin_gates/common.rs"]
mod common;
#[path = "fin_gates/rockets.rs"]
mod rockets;

use bowshock_cfd::viscous3::WallTraction;
use bowshock_core::engine::case3::Case3;
use bowshock_core::engine::run3::{wall_samples, Wall};
use bowshock_core::AftSpec;
use bowshock_reference::{arcas, arcas_stab};
use common::{machs, tier, tier_kind, Session, TierKind};
use rockets::{arcas_rocket, Rocket};

/// The long model of the transonic tunnel (`rocket_gates::arcas_for(m, true)`), or the short one.
fn model(short: bool) -> Rocket {
    let extra = if short { 0.0 } else { arcas_stab::LONG_TUBE_LENGTH - arcas::TUBE_LENGTH };
    arcas_rocket(extra, arcas::TRANSONIC_RE_PER_M, arcas::TRANSONIC_T0_K)
}

const IN: f64 = 0.0254;
/// Sting radius of the test over the nose diameter (1 in diameter sting, 2.25 in nose).
const STING_RADIUS: f64 = 0.5 / 2.25;
/// The transition band of the test, nose diameters from the tip (1.25 in aft of the nose tip).
const TRIP_X: f64 = 1.25 / 2.25;

fn case_of(r: &Rocket, mach: f64, variant: &str) -> Case3 {
    let parts: Vec<&str> = variant.split('+').collect();
    let mut spec = r.body.clone();
    if parts.contains(&"lip") {
        let tube = arcas_stab::LONG_TUBE_LENGTH - if parts.contains(&"short") { arcas_stab::LONG_TUBE_LENGTH - arcas::TUBE_LENGTH } else { 0.0 };
        spec.aft = vec![AftSpec::Tube { length: tube }, AftSpec::Transition { length: 1.757 * IN, aft_diameter: 1.308 * IN }];
    }
    let (mut body, _) = spec.to_internal().expect("body");
    if parts.contains(&"lip") {
        // the lip: a step of 0.081 in over 0.053 in (the spec refuses a cone steeper than 30 degrees)
        body = body.with_step(0.5 * (arcas::BASE_DIAMETER - 1.308 * IN) / r.d, 0.053 * IN / r.d);
    }
    if parts.contains(&"sting") {
        body = body.with_sting(STING_RADIUS);
    }
    let kind = if parts.contains(&"euler") { TierKind::Euler } else { tier_kind() };
    let mut c = Case3::wedge(mach, body, 0).with_tier(tier(kind, r.flow(mach, 1.2)));
    for p in &parts {
        if let Some(s) = p.strip_prefix("ref") {
            c = c.with_refinement(s.parse::<f64>().expect("refN") / 10.0);
        }
    }
    if parts.contains(&"trip") {
        c = c.with_transition(TRIP_X, 0.0);
    }
    for p in &parts {
        if let Some(x) = p.strip_prefix("xtr") {
            c = c.with_transition(x.parse::<f64>().expect("xtrN"), 0.0);
        }
    }
    c
}

/// Face centre of a wall traction's face.
fn face_x(g: &bowshock_cfd::grid3::Grid3, t: &WallTraction) -> f64 {
    let [i, j, k] = t.idx;
    match t.dir {
        0 => g.iface[(k * g.nj + j) * (g.ni + 1) + i].c[0],
        1 => g.jface[(k * (g.nj + 1) + j) * g.ni + i].c[0],
        _ => g.kface[(k * g.nj + j) * g.ni + i].c[0],
    }
}

fn tractions(s: &Session) -> Vec<WallTraction> {
    match s {
        Session::Cpu(sol) => sol.wall_tractions(),
        Session::Gpu(r) => {
            let g = &r.host.grid;
            let Some(f) = r.gpu.viscous_flux().expect("flux") else { return Vec::new() };
            let solid: Vec<bool> = g.part.iter().map(|&p| p != bowshock_cfd::grid3::PART_FLUID).collect();
            bowshock_cfd::viscous3::wall_tractions(g, &solid, &r.host.rans().expect("rans").dirs, &f[0], &f[1], &f[2])
        }
    }
}

fn host(s: &Session) -> &bowshock_cfd::solver3::Solver3 {
    match s {
        Session::Cpu(sol) => sol,
        Session::Gpu(r) => &r.host,
    }
}

/// Bin edges along the body (nose diameters): 0.25 over the first 2, 0.5 to the end of the nose, 1 over the cylinder, 0.1 over the boattail.
fn edges(x_bt: f64, x_end: f64) -> Vec<f64> {
    let mut e = vec![0.0];
    let mut x = 0.0;
    while x < x_bt - 1e-9 {
        x += if x < 2.0 {
            0.25
        } else if x < 4.71 {
            0.5
        } else {
            1.0
        };
        e.push(x.min(x_bt));
    }
    while *e.last().unwrap() < x_end - 1e-9 {
        let n = (e.last().unwrap() + 0.1).min(x_end);
        e.push(n);
    }
    e
}

#[test]
#[ignore = "diagnosis: Coarse wedge on the GPU"]
fn sd1_arcas_long_finless() {
    let long_rows: Vec<(f64, f64)> = arcas_stab::LONG_TRANSONIC.iter().map(|r| (r.0, r.2)).collect();
    let short_rows: Vec<(f64, f64)> = arcas::TRANSONIC.iter().map(|r| (r.0, r.1)).collect();
    let want = machs(&[0.6, 0.8]);
    let variants: Vec<String> = std::env::var("SD_VARIANTS").unwrap_or_else(|_| "base".into()).split(',').map(|s| s.trim().to_string()).collect();
    let mut machs_run: Vec<f64> = std::env::var("SD_MACHS").map(|v| v.split(',').map(|x| x.parse().unwrap()).collect()).unwrap_or(want);
    machs_run.retain(|m| long_rows.iter().any(|r| (r.0 - m).abs() < 1e-9));
    for variant in &variants {
        let short = variant.split('+').any(|t| t == "short");
        let (r, rows) = (model(short), if short { &short_rows } else { &long_rows });
        for &m in &machs_run {
            let Some(data) = rows.iter().find(|x| (x.0 - m).abs() < 1e-9).map(|x| x.1) else { continue };
            let case = case_of(&r, m, variant);
            let tag = format!("SD {variant} M{m:.2}");
            let mut s = match Session::new(&case) {
                Ok(s) => s,
                Err(e) => {
                    println!("{tag} FAILED build: {e}");
                    continue;
                }
            };
            let sol = match s.run(&case, false) {
                Ok(x) => x,
                Err(e) => {
                    println!("{tag} FAILED run: {e}");
                    continue;
                }
            };
            let af = r.area_factor();
            let p = &sol.parts;
            let v = af * (p.total - p.base - p.friction.base);
            let g = &host(&s).grid;
            let ni = g.ni;
            let nj = g.nj;
            let nk = g.nk;
            println!(
                "{tag} grid {ni}x{nj}x{nk} area_factor {af:.5} iters {} stop {:?} y+max {:.3} cp_base {:+.4} drop {:.2}",
                sol.point.iterations, sol.point.stop_reason, sol.point.y_plus_max, sol.point.cp_base, sol.point.drop_orders
            );
            let aft: Vec<String> = p.aft.iter().map(|a| format!("{a:+.4}")).collect();
            let faft: Vec<String> = p.friction.aft.iter().map(|a| format!("{a:.4}")).collect();
            println!(
                "{tag} parts v {v:.4} data {data:.4} err {:+.1}% total {:.4} | pressure nose {:+.4} aft [{}] base {:+.4} | friction nose {:.4} aft [{}] base {:.5}",
                100.0 * (v / data - 1.0),
                p.total,
                p.nose,
                aft.join(" "),
                p.base,
                p.friction.nose,
                faft.join(" "),
                p.friction.base
            );
            // local Cf (axial traction / q) and Cp of the body surface
            let body = case.body.clone();
            let x_bt = body.station(body.segments.len() - 1).0;
            let x_end = body.x_base();
            let e = edges(x_bt, x_end);
            let nb = e.len() - 1;
            let bin = |x: f64| (0..nb).find(|&b| x >= e[b] && x < e[b + 1]);
            let q = 0.5 * m * m;
            let (mut cf, mut cfn, mut cp, mut cpn) = (vec![0.0; nb], vec![0usize; nb], vec![0.0; nb], vec![0usize; nb]);
            for t in tractions(&s) {
                if t.dir == 0 {
                    continue;
                }
                let x = face_x(g, &t);
                if let Some(b) = bin(x) {
                    cf[b] += t.tau[0].abs() / q;
                    cfn[b] += 1;
                }
            }
            let h = host(&s);
            let p_inf = h.cfg.freestream[4];
            for w in wall_samples(&h.grid, &h.u) {
                if matches!(w.class, Wall::Body(_)) {
                    if let Some(b) = bin(w.face.c[0]) {
                        cp[b] += (w.pw - p_inf) / q;
                        cpn[b] += 1;
                    }
                }
            }
            for b in 0..nb {
                let n1 = cfn[b].max(1) as f64;
                let n2 = cpn[b].max(1) as f64;
                println!("{tag} x {:.3} {:.3} cf {:.6} cp {:+.5} n {} {}", e[b], e[b + 1], cf[b] / n1, cp[b] / n2, cfn[b], cpn[b]);
            }
            let _ = TierKind::Coarse;
        }
    }
}

/// Every variant builds a grid (CPU, no solve).
#[test]
fn variants_build() {
    for v in ["base", "lip", "lip+sting+trip", "short+lip", "euler", "xtr4"] {
        let r = model(v.contains("short"));
        let case = case_of(&r, 0.6, v);
        let s = case.build_solver_cached().unwrap_or_else(|e| panic!("{v}: {e}"));
        println!("{v}: {}x{}", s.grid.ni, s.grid.nj);
    }
}

/// The solver's boattail pressure drag against the data of NASA TM X-2439 (`bowshock_cases::subsonic::fox_tm_x2439`, tangent ogive configuration 4, a 10 degree boattail of 0.647 d base
/// diameter on a 0.13 d sting, Mach 0.404 / 0.608 / 0.831; the Arcas has a 0.65 d base): Euler wedge at levels 0 and 1, the boattail drag `-int Cp dA/S` and the base annulus
/// `Cp` printed for `tools/subdrag_post.py fox` to put beside the integral of the tabulated Cp.
#[test]
#[ignore = "diagnosis: Euler wedge on the GPU"]
fn sd2_fox_boattail() {
    for level in [0u32, 1] {
        for m in bowshock_cases::subsonic::fox_tm_x2439::MACH {
            let case = bowshock_cases::subsonic::fox_wedge_case(false, m, level);
            let mut s = Session::new(&case).expect("session");
            let sol = s.run(&case, false).expect("run");
            let p = &sol.parts;
            println!(
                "SDFOX level {level} M{m:.3} iters {} stop {:?} cd_nose {:+.4} cd_aft {:?} cd_base {:+.4} cp_base {:+.4}",
                sol.point.iterations,
                sol.point.stop_reason,
                p.nose,
                p.aft.iter().map(|a| (a * 1e4).round() / 1e4).collect::<Vec<_>>(),
                p.base,
                sol.point.cp_base
            );
            let h = host(&s);
            let p_inf = h.cfg.freestream[4];
            let q = 0.5 * m * m;
            for w in wall_samples(&h.grid, &h.u) {
                if matches!(w.class, Wall::Body(_)) && w.face.c[0] > 3.9 {
                    println!("SDFOX level {level} M{m:.3} cp x {:.4} {:+.5}", w.face.c[0], (w.pw - p_inf) / q);
                }
            }
        }
    }
}
