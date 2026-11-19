//! Cross-validation gates (DESIGN3D item 55): the 3D engine on the public wind-tunnel data of rockets other than the Basic Finner, so that the Finner's error pattern (CD0 +3-7 % fully
//! turbulent, subsonic `CN_alpha` +11 %) is not mistaken for the solver's. The articles are in `fin_gates/rockets.rs`, the data in `bowshock_reference::{arcas_stab, yuska, grm}`.
//!
//! - G-X1 `arcas`: the Arcas short model (NASA TN D-4013 / D-4014): `CN_alpha` (secant 0-4 degrees, per rad) and `x_cp` on the Lift half domain, Mach 0.6 to 2.96.
//! - G-X2 `yuska`: the Lewis 1/5-scale sounding-rocket second stage (NASA TN D-3182, thick swept wedge fins): `CN_alpha` and `x_cp`, Mach 1.79-3.47, models A (roll 0 and 45), B, C.
//! - G-X3 `grm`: the Martin generalized research model B4 + F13 (AEDC-TR-75-125 Test 1, tabulated): `CN_alpha` and `x_cp`, Mach 0.8-1.3.
//! - G-X4 `arcas_long`, G-X5 `yuska_drag`, G-X6 `grm_drag`: the axial force without the base on the four-fin sector of the Coarse tier (`BOWSHOCK_FIN_TIER=coarse`, the product's table path).
//!
//! Backend, tier and Mach list as `fin_lift_gates` (`BOWSHOCK_WEDGE_BACKEND=gpu`, `BOWSHOCK_FIN_TIER=lift` for G-X1..3 and `coarse` for G-X4..6, `BOWSHOCK_GATE_MACHS=a,b`).
//! All `#[ignore]`d: `cargo test --release -p bowshock-core --test rocket_gates -- --ignored --nocapture <name>`.

#[path = "fin_gates/common.rs"]
mod common;
#[path = "fin_gates/rockets.rs"]
mod rockets;

use bowshock_core::engine::grid3::FinPhase;
use bowshock_reference::{arcas, arcas_stab, grm, yuska};
use common::{is_gate, machs, par_map, tier_kind, Band, Lines, Session, Solved, TierKind};
use rockets::{arcas_rocket, grm_rocket, yuska_rocket, Rocket};

fn yuska_re_per_ft(mach: f64) -> f64 {
    yuska::TEST_CONDITIONS.iter().find(|c| (c.0 - mach).abs() < 1e-9).unwrap_or_else(|| panic!("no Table II row for Mach {mach}")).3
}

fn arcas_for(mach: f64, long: bool) -> Rocket {
    let extra = if long { arcas_stab::LONG_TUBE_LENGTH - arcas::TUBE_LENGTH } else { 0.0 };
    if mach < 1.3 {
        arcas_rocket(extra, arcas::TRANSONIC_RE_PER_M, arcas::TRANSONIC_T0_K)
    } else {
        arcas_rocket(extra, arcas::SUPERSONIC_RE_PER_M, arcas::SUPERSONIC_T0_K)
    }
}

// ---------------------------------------------------------------------------------------------------------------------------------
// Stability on the Lift half domain

/// One reference point of a stability gate: Mach, `CN_alpha` (per rad, reference area of the data) and `x_cp` (calibers from the nose tip); NaN when the data has none.
type StabPoint = (f64, f64, f64);

/// What one gate compares: the rocket of each Mach number, the fin roll phase, the incidences run (warm-started in this order), the one the data's slope definition is taken at,
/// and the bands.
struct StabGate<'a> {
    id: &'static str,
    tag: &'static str,
    make: &'a (dyn Fn(f64) -> Rocket + Sync),
    points: Vec<StabPoint>,
    phase: FinPhase,
    alphas: &'static [f64],
    alpha_ref: f64,
    mach_max: f64,
    cn_band: Band,
    xcp_abs: f64,
    /// INFO only (a documented offset of the data, not a solver gate).
    info: bool,
}

fn run_stability(g: &StabGate) -> Lines {
    let keep = machs(&g.points.iter().map(|p| p.0).collect::<Vec<_>>());
    let items: Vec<StabPoint> = g.points.iter().copied().filter(|p| keep.contains(&p.0)).collect();
    let runs = par_map(&items, |p| -> Result<(Vec<Solved>, f64, bool), String> {
        let r = (g.make)(p.0);
        let cases: Vec<_> = g.alphas.iter().map(|&a| r.half(p.0, g.mach_max, a, g.phase, tier_kind())).collect();
        let mut s = Session::new(&cases[0])?;
        let mut out = Vec::new();
        for (k, c) in cases.iter().enumerate() {
            out.push(s.run(c, k > 0)?);
        }
        Ok((out, r.area_factor(), is_gate(tier_kind(), &cases[0])))
    });
    let mut lines = Lines::default();
    for (p, run) in items.iter().zip(runs) {
        let case = format!("{}-M{:.3}", g.tag, p.0);
        match run {
            Ok((sol, q, viscous)) => {
                let gate = viscous && !g.info;
                let k = g.alphas.iter().position(|&a| a == g.alpha_ref).expect("alpha_ref is one of the incidences");
                let cna = |i: usize| q * sol[i].lift.cn / g.alphas[i].to_radians();
                if p.1.is_finite() {
                    lines.check(gate, g.id, &format!("{case}:cn_alpha"), cna(k), p.1, g.cn_band);
                }
                if p.2.is_finite() {
                    lines.check(gate, g.id, &format!("{case}:x_cp"), sol[k].lift.x_cp, p.2, Band::RelAbs(0.0, g.xcp_abs));
                }
                let all: Vec<String> = g.alphas.iter().enumerate().map(|(i, a)| format!("CNa({a}) {:.2} x_cp {:.3} [{}]", cna(i), sol[i].lift.x_cp, sol[i].stop())).collect();
                lines.info(g.id, &format!("{case}:slopes"), &format!("{}; data CNa {:.2} x_cp {:.3}; error {:+.1} % / {:+.3} cal", all.join(", "), p.1, p.2, 100.0 * (cna(k) / p.1 - 1.0), sol[k].lift.x_cp - p.2));
            }
            Err(e) => {
                println!("GATE {} {case} n/a n/a n/a FAIL ({e})", g.id);
                lines.failed += 1;
            }
        }
    }
    lines
}

fn arcas_points() -> Vec<StabPoint> {
    let mut v: Vec<StabPoint> = arcas_stab::SHORT_TRANSONIC.iter().filter(|r| r.1.is_finite()).map(|r| (r.0, r.1, r.3)).collect();
    v.extend(arcas_stab::SHORT_SUPERSONIC.iter().filter(|r| r.0 <= 3.0).map(|r| (r.0, r.1, r.3)));
    v
}

/// G-X1: the Arcas short model (l/d 18.2, tangent ogive, boattail with the reflex lip modelled as a plain boattail, double-wedge fins as G-F4) `CN_alpha` (secant 0 to 4 degrees, fins
/// on, roll 0, `arcas_stab::SHORT_*`: figure readings, +-0.9 per rad) and `x_cp`, Lift half domain, fins in the pitch plane, free wake behind the base (the 1 in sting of the transonic tests is
/// not modelled). Bands: `CN_alpha` 15 %, `x_cp` 0.35 cal (2 % of the length; the reading uncertainty is 0.09 to 0.27 cal).
#[test]
#[ignore = "validation: lift tier on the GPU"]
fn gate_x1_arcas_cn_alpha_and_xcp() {
    let make = |m: f64| arcas_for(m, false);
    let g = StabGate { id: "G-X1", tag: "arcas", make: &make, points: arcas_points(), phase: FinPhase::InPlane, alphas: &[2.0, 4.0], alpha_ref: 4.0, mach_max: 2.96, cn_band: Band::Rel(0.15), xcp_abs: 0.35, info: false };
    run_stability(&g).finish();
}

/// The Yuska points of one table as `StabPoint`s.
fn yuska_points(t: &[(f64, f64, f64, f64)]) -> Vec<StabPoint> {
    t.iter().filter(|r| r.1.is_finite()).map(|r| (r.0, r.1, r.2)).collect()
}

/// G-X2: the Lewis 1/5-scale sounding-rocket second stage (NASA TN D-3182; `yuska::*`), `CN_alpha` at zero normal force (2 degree secant) and `x_cp/d` at Mach 2.00-3.47 of the 10 x 10 ft
/// tunnel: model A roll 0 (fins in the pitch plane) and roll 45 (between), the shortened model B and the trimmed-fin model C. The 8 x 6 ft tunnel's A points (Mach 1.79-2.08) are INFO: the
/// report's own Fig. 4 puts them 10 % above the 10 x 10 ft ones and blames flow angularity or the fin cant. Bands: `CN_alpha` 13 %, `x_cp` 0.25 cal (the report quotes +9 / -7 % and 1.5 % of
/// the body length, 0.16 cal, for its own theory against these data).
#[test]
#[ignore = "validation: lift tier on the GPU"]
fn gate_x2_yuska_cn_alpha_and_xcp() {
    let a = |m: f64| yuska_rocket(false, false, yuska_re_per_ft(m));
    let b = |m: f64| yuska_rocket(true, false, yuska_re_per_ft(m));
    let c = |m: f64| yuska_rocket(false, true, yuska_re_per_ft(m));
    let mk = |id, tag, make: &'static (dyn Fn(f64) -> Rocket + Sync), pts, phase, info| StabGate {
        id,
        tag,
        make,
        points: pts,
        phase,
        alphas: &[2.0],
        alpha_ref: 2.0,
        mach_max: 3.47,
        cn_band: Band::Rel(0.13),
        xcp_abs: 0.25,
        info,
    };
    // leaked so the closures live for the gates' `'static` borrow
    let (a, b, c): (&'static (dyn Fn(f64) -> Rocket + Sync), &'static (dyn Fn(f64) -> Rocket + Sync), &'static (dyn Fn(f64) -> Rocket + Sync)) =
        (Box::leak(Box::new(a)), Box::leak(Box::new(b)), Box::leak(Box::new(c)));
    let mut lines = Lines::default();
    for g in [
        mk("G-X2", "yuska-A-roll0", a, yuska_points(&yuska::MODEL_A_ROLL0_10X10), FinPhase::InPlane, false),
        mk("G-X2", "yuska-A-roll45", a, yuska_points(&yuska::MODEL_A_ROLL45_10X10), FinPhase::Between, false),
        mk("G-X2", "yuska-B-roll0", b, yuska_points(&yuska::MODEL_B_ROLL0_10X10), FinPhase::InPlane, false),
        mk("G-X2", "yuska-C-roll0", c, yuska_points(&yuska::MODEL_C_ROLL0_10X10), FinPhase::InPlane, false),
        mk("G-X2", "yuska-A-8x6", a, yuska_points(&yuska::MODEL_A_ROLL0_8X6), FinPhase::InPlane, true),
    ] {
        let l = run_stability(&g);
        lines.failed += l.failed;
        lines.known += l.known;
        lines.gates += l.gates;
        lines.infos += l.infos;
    }
    lines.finish();
}

/// G-X3: the Martin generalized research model B4 + F13 (AEDC-TR-75-125 Test 1, 4T tunnel; `grm::FINS_ON`, tabulated, +-0.3 per rad and +-0.15 cal): `CN_alpha` (secant 0 to 4 degrees) and `x_cp`
/// at Mach 0.80-1.30, Lift half domain, fins in the pitch plane, fully turbulent (the report does not say how transition was fixed), no sting (not dimensioned). `CN_alpha` 13 %, `x_cp` 0.25 cal.
#[test]
#[ignore = "validation: lift tier on the GPU"]
fn gate_x3_grm_cn_alpha_and_xcp() {
    let make = |_m: f64| grm_rocket();
    let points: Vec<StabPoint> = grm::FINS_ON.iter().map(|r| (r.0, r.1, r.2)).collect();
    let g = StabGate { id: "G-X3", tag: "grm", make: &make, points, phase: FinPhase::InPlane, alphas: &[2.0, 4.0], alpha_ref: 4.0, mach_max: 1.3, cn_band: Band::Rel(0.13), xcp_abs: 0.25, info: false };
    run_stability(&g).finish();
}

// ---------------------------------------------------------------------------------------------------------------------------------
// Axial force on the Coarse sector

/// The axial force without the base of the four-fin sector, `C_A,corr` of the data: pressure and friction of the body and the fins on the data's reference area.
fn sector_axial(r: &Rocket, mach: f64, mach_max: f64, fins: bool) -> Result<(Solved, f64, bool), String> {
    let kind = tier_kind();
    let case = if fins { r.sector(mach, mach_max, kind) } else { r.finless(mach, mach_max, kind) };
    let s = Session::new(&case).and_then(|mut s| s.run(&case, false))?;
    let v = r.area_factor() * (s.parts.total - s.parts.base - s.parts.friction.base);
    Ok((s, v, is_gate(kind, &case)))
}

fn print_axial(lines: &mut Lines, id: &str, case: &str, gate: bool, v: f64, data: f64, band: Band, s: &Solved) {
    lines.info(
        id,
        &format!("{case}:parts"),
        &format!(
            "{v:.4} without base (total {:.4}; body pressure {:+.4}, base {:+.4}, fin pressure {:+.4}, friction body {:.4} fins {:.4}; {}); data {data:.4}: {:+.1} %",
            s.parts.total,
            s.body_pressure(),
            s.parts.base,
            s.fin_pressure(),
            s.parts.body_friction(),
            s.fin_friction.unwrap_or(f64::NAN),
            s.stop(),
            100.0 * (v / data - 1.0)
        ),
    );
    lines.check(gate, id, case, v, data, band);
}

/// G-X4: the Arcas **long** model (l/d 23.77: the short model's cylinder 12.55 in longer, NASA TN D-4013 Fig. 12) `C_A,corr` fins on and off, zero incidence, Mach 0.6 / 0.8 gated (fins on 15 %,
/// fins off 12 % or 0.03) and Mach 0.9, 0.95, 1.0, 1.2 INFO (the report's reading uncertainty is 0.02 at Mach 0.9-1.0 and the transonic drag rise is where the data of the short model
/// disagree with the solver most), as G-F4 for the short model.
#[test]
#[ignore = "validation: Coarse sector on the GPU"]
fn gate_x4_arcas_long_axial_force() {
    let rows: Vec<(f64, f64, f64)> = arcas_stab::LONG_TRANSONIC.iter().map(|r| (r.0, r.2, r.3)).collect();
    let keep = machs(&rows.iter().map(|r| r.0).collect::<Vec<_>>());
    let rows: Vec<_> = rows.into_iter().filter(|r| keep.contains(&r.0)).collect();
    let runs = par_map(&rows, |&(m, _, _)| {
        let r = arcas_for(m, true);
        (sector_axial(&r, m, 1.2, false), sector_axial(&r, m, 1.2, true))
    });
    let mut lines = Lines::default();
    for (&(m, off, on), (a, b)) in rows.iter().zip(runs) {
        match (a, b) {
            (Ok((so, vo, go)), Ok((sn, vn, gn))) => {
                let gated = [0.6, 0.8].contains(&m) && go && gn;
                print_axial(&mut lines, "G-X4", &format!("arcas-long-M{m:.2}:ca_fins_off"), gated, vo, off, Band::RelAbs(0.12, 0.03), &so);
                print_axial(&mut lines, "G-X4", &format!("arcas-long-M{m:.2}:ca_fins_on"), gated, vn, on, Band::Rel(0.15), &sn);
                lines.check(gated, "G-X4", &format!("arcas-long-M{m:.2}:increment"), vn - vo, on - off, Band::RelAbs(0.30, 0.03));
            }
            (a, b) => {
                println!("GATE G-X4 arcas-long-M{m:.2} n/a n/a n/a FAIL ({:?} / {:?})", a.err(), b.err());
                lines.failed += 1;
            }
        }
    }
    lines.finish();
}

/// G-X5: the Lewis sounding-rocket stage's axial force at zero incidence without the base (`yuska` Fig. 5, "adjusted to free-stream static pressure at the base"), the four-fin sector of the
/// Coarse tier, models A, B and C at the 10 x 10 ft Mach numbers (the 8 x 6 ft ones of model A INFO): 12 %.
#[test]
#[ignore = "validation: Coarse sector on the GPU"]
fn gate_x5_yuska_axial_force() {
    let mut rows: Vec<(&str, bool, bool, f64, f64, bool)> = Vec::new(); // (model, shortened, trimmed, mach, ca0, gated)
    let mut add = |name: &'static str, short: bool, trim: bool, t: &[(f64, f64, f64, f64)], gated: bool| {
        for r in t.iter().filter(|r| r.3.is_finite()) {
            rows.push((name, short, trim, r.0, r.3, gated));
        }
    };
    add("A", false, false, &yuska::MODEL_A_ROLL0_10X10, true);
    add("B", true, false, &yuska::MODEL_B_ROLL0_10X10, true);
    add("C", false, true, &yuska::MODEL_C_ROLL0_10X10, true);
    add("A-8x6", false, false, &yuska::MODEL_A_ROLL0_8X6, false);
    let keep = machs(&rows.iter().map(|r| r.3).collect::<Vec<_>>());
    rows.retain(|r| keep.contains(&r.3));
    let runs = par_map(&rows, |&(_, short, trim, m, _, _)| sector_axial(&yuska_rocket(short, trim, yuska_re_per_ft(m)), m, 3.47, true));
    let mut lines = Lines::default();
    for (&(name, _, _, m, ca, gated), r) in rows.iter().zip(runs) {
        match r {
            Ok((s, v, viscous)) => print_axial(&mut lines, "G-X5", &format!("yuska-{name}-M{m:.2}:ca0"), gated && viscous, v, ca, Band::Rel(0.12), &s),
            Err(e) => {
                println!("GATE G-X5 yuska-{name}-M{m:.2} n/a n/a n/a FAIL ({e})");
                lines.failed += 1;
            }
        }
    }
    lines.finish();
}

/// G-X6: the generalized research model B4 + F13's forebody axial force at zero incidence (`grm::FINS_ON` `CAF`: the total with the base pressure force removed; `CA - CAB`), the sector of
/// the Coarse tier, Mach 0.80-1.30. 12 % (the report's `dCA` is 0.014, the repeats of the transonic points scatter by 0.02-0.05, and the transition state is unknown): Mach 0.92-1.10 INFO.
#[test]
#[ignore = "validation: Coarse sector on the GPU"]
fn gate_x6_grm_axial_force() {
    let rows: Vec<(f64, f64)> = grm::FINS_ON.iter().map(|r| (r.0, r.3)).collect();
    let keep = machs(&rows.iter().map(|r| r.0).collect::<Vec<_>>());
    let rows: Vec<_> = rows.into_iter().filter(|r| keep.contains(&r.0)).collect();
    let runs = par_map(&rows, |&(m, _)| sector_axial(&grm_rocket(), m, 1.3, true));
    let mut lines = Lines::default();
    for (&(m, caf), r) in rows.iter().zip(runs) {
        match r {
            Ok((s, v, viscous)) => print_axial(&mut lines, "G-X6", &format!("grm-M{m:.2}:caf"), viscous && !(0.92..=1.10).contains(&m), v, caf, Band::Rel(0.12), &s),
            Err(e) => {
                println!("GATE G-X6 grm-M{m:.2} n/a n/a n/a FAIL ({e})");
                lines.failed += 1;
            }
        }
    }
    lines.finish();
}

/// The mesher takes every test article: the sector of the Coarse tier builds (one cell count line each).
#[test]
fn the_rocket_grids_build() {
    for (name, r) in [
        ("arcas-short", arcas_for(0.8, false)),
        ("arcas-long", arcas_for(0.8, true)),
        ("yuska-A", yuska_rocket(false, false, 2.4e6)),
        ("yuska-B", yuska_rocket(true, false, 2.4e6)),
        ("yuska-C", yuska_rocket(false, true, 2.4e6)),
        ("grm", grm_rocket()),
    ] {
        let case = r.sector(2.0, 3.5, TierKind::Coarse);
        let s = case.build_solver().unwrap_or_else(|e| panic!("{name}: {e}"));
        println!("GRID {name}: {} x {} x {} cells", s.grid.ni, s.grid.nj, s.grid.nk);
        let half = r.half(2.0, 3.5, 2.0, FinPhase::InPlane, TierKind::Lift);
        let s = half.build_solver().unwrap_or_else(|e| panic!("{name} half domain: {e}"));
        println!("GRID {name} half domain: {} x {} x {} cells", s.grid.ni, s.grid.nj, s.grid.nk);
    }
}
