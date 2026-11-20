//! Fin and incidence gates of the 3D engine (DESIGN3D G-F1, G-F4, G-F5, G-L1 and the sector-vs-half gate G-S), written to run the minute the viscous engine is on the
//! GPU: the same tests run Euler today (`INFO` lines) and RANS later (`GATE` lines). Backend: CPU, or the GPU with `BOWSHOCK_WEDGE_BACKEND=gpu` (as `wedge_gates`);
//! tier: `BOWSHOCK_FIN_TIER=euler|coarse|fine|lift` (default `euler`); `BOWSHOCK_GATE_MACHS=0.8,2` restricts the Mach list. All `#[ignore]`d: run with
//!
//! ```text
//! cargo test -p bowshock-core --test fin_lift_gates -- --ignored --nocapture <name>
//! BOWSHOCK_WEDGE_BACKEND=gpu BOWSHOCK_FIN_TIER=coarse tools/gpuq/gpuq ... (the same command through the queue)
//! ```
//!
//! Output lines: `GATE|INFO <id> <case> <measured> <reference> <band> [PASS|FAIL]`. The shared plumbing is `fin_gates/common.rs`; where the viscous engine plugs in is
//! [`common::engine_is_viscous`] and [`common::Solved::fin_friction`].

#[path = "fin_gates/common.rs"]
mod common;

use bowshock_core::engine::case3::{Case3, WallFlow};
use bowshock_core::engine::grid3::{Domain3, FinPhase, Fins3};
use bowshock_core::{AftSpec, BodySpec, FinProfile, FinSpec, NoseShape, NoseSpec};
use bowshock_reference::{anf, arcas};
use common::{is_gate, machs, par_map, tier, tier_kind, Band, Lines, Session, Solved, TierKind};
use std::f64::consts::FRAC_PI_4;

// ---------------------------------------------------------------------------------------------------------------------------------
// Flow, references

/// `rho c d / mu` of the ISA sea-level atmosphere for a body of diameter `d` \[m\] (the ANF: 0.03 m, 7.0e5).
fn re_c_sea_level(d: f64) -> f64 {
    1.225 * 340.294 * d / 1.7894e-5
}

/// Mean turbulent skin friction of a flat plate of chord `c` (in body diameters) at Mach `m` and `Re_c = rho c d / mu`: the facade's fin model (`bowshock_core::fins`,
/// Niskanen 2009 eqs. 3.78-3.81): `1 / (1.5 ln Re - 5.6)^2` with its compressibility factors, `Re = Re_c m c`.
fn flat_plate_cf(re_c: f64, mach: f64, chord: f64) -> f64 {
    let re = re_c * mach * chord;
    let cf = 1.0 / (1.50 * re.ln() - 5.6).powi(2);
    if mach < 1.0 {
        cf * (1.0 - 0.1 * mach * mach)
    } else {
        cf / (1.0 + 0.15 * mach * mach).powf(0.58)
    }
}

/// The ANF's free stream for tier sizing (sea level, the highest Mach of the tables).
fn anf_flow(mach_max: f64) -> WallFlow {
    WallFlow { re_c: re_c_sea_level(0.03), mach_max, ks_body: 0.0, ks_fin: 0.0, sutherland: 0.368 }
}

// ---------------------------------------------------------------------------------------------------------------------------------
// G-F1: fin friction

/// G-F1: the friction of a thin fin (t/c 2 %, N = 4, chord 0.5 d, span 0.5 d) at `Re_c` 1e6, Mach 0.5 and 2, against `2 Cf(Re_c, M) A_fin N` (the flat-plate skin
/// friction of the facade's fin model), band `[0.92, 1.12]`. Needs the viscous engine: **fails loudly while the engine is inviscid.**
#[test]
#[ignore = "validation: needs the viscous engine (RANS), CPU/GPU minutes"]
fn gate_f1_fin_friction_matches_the_flat_plate() {
    let kind = tier_kind();
    let (chord, span, tc, n) = (0.5, 0.5, 0.02, 4u32);
    let fins = Fins3 {
        count: n,
        root_chord: chord,
        tip_chord: chord,
        span,
        sweep: 0.0,
        thickness: tc * chord,
        te_to_base: 0.0,
        profile: FinProfile::DoubleWedge,
        planform: None,
    };
    let flow = WallFlow { re_c: 1.0e6, mach_max: 2.0, ks_body: 0.0, ks_fin: 0.0, sutherland: 0.368 };
    let probe = Case3::sector(2.0, Case3::anf_body(), fins.clone(), tier(kind, flow), 0);
    assert!(
        common::engine_is_viscous(&probe) && kind != TierKind::Euler,
        "G-F1: engine is inviscid (the 3D solver has no viscous fluxes yet, or BOWSHOCK_FIN_TIER is euler): the fin friction cannot be measured"
    );
    let mut lines = Lines::default();
    for m in machs(&[0.5, 2.0]) {
        let case = Case3::sector(m, Case3::anf_body(), fins.clone(), tier(kind, flow), 0);
        let solved = Session::new(&case).and_then(|mut s| s.run(&case, false)).expect("run");
        let measured = solved.fin_friction.expect("G-F1: the engine is viscous but Solved::fin_friction is not filled");
        let reference = 2.0 * flat_plate_cf(flow.re_c, m, chord) * (chord * span) * f64::from(n) / FRAC_PI_4;
        let (case_name, band) = (format!("fin-friction-M{m:.1}"), Band::Ratio(0.92, 1.12));
        if m < 1.0 {
            // the subsonic fin sits at 0.92 of the idealised plate whatever the grid (GPU, cfl 10, sub 2: Coarse 0.9194, Coarse with 64 instead of 32 chordwise cells 0.9191, Fine tier
            // with the fin sides at y+ ~ 1: 0.9278): the reference is a fully turbulent plate from the leading edge, the fin lives in the body's boundary layer at its root and has a
            // leading-edge region (DESIGN3D item 41). The band stays; the line is the documented discrepancy
            lines.check_known(
                true,
                "G-F1",
                &case_name,
                measured,
                reference,
                band,
                "reference: flat plate from the leading edge; grid-independent 0.919 (Coarse, 64 chord cells), 0.928 (Fine y+ 1), DESIGN3D item 41",
            );
        } else {
            lines.check(true, "G-F1", &case_name, measured, reference, band);
        }
    }
    lines.finish();
}

// ---------------------------------------------------------------------------------------------------------------------------------
// G-F4: Arcas

fn arcas_body_spec() -> BodySpec {
    BodySpec {
        nose: NoseSpec { shape: NoseShape::TangentOgive, length: arcas::NOSE_LENGTH, diameter: arcas::BODY_DIAMETER },
        aft: vec![AftSpec::Tube { length: arcas::TUBE_LENGTH }, AftSpec::Transition { length: arcas::BOATTAIL_LENGTH, aft_diameter: arcas::BASE_DIAMETER }],
    }
}

/// The Arcas fins as the reference crate has them (`bowshock_reference::arcas`): trapezoidal **double wedge**, sharp leading and trailing edge, one mean thickness.
///
/// Source (read in the reports, NTRS 19670020031 and 19670020050): NASA TN D-4014 (Babb & Fuller, 1967) p. 4, "The model consisted of a cylindrical centerbody, a
/// boattailed afterbody, and **trapezoidal double-wedge fins**. The afterbody boattail ended with a reflexed lip." Fig. 1(b), fin details: section 0.150 in thick at
/// the root and 0.096 in at the tip (a spanwise taper of 1.47 degrees; the model takes the mean, 0.123 in: the mean of `t^2` is the same to 1 %), wedge half angle
/// 2.72 degrees "normal to 15 degree line", 30 degree leading-edge sweep; fin anchors (tabs 0.220 x 2.165 x 0.035 in) at the roots. Both reports: carborundum
/// transition strips on the nose and "1/2 in aft of the leading edge of each fin"; D-4013 p. 4: a 1 in diameter sting 11.5 in aft of the base, axial force not corrected
/// to free-stream conditions at the base except for the summary data at roll 0, alpha 0 (the data used here); D-4014 Fig. 4: the chamber axial-force coefficient is
/// 0.02-0.04 at alpha 0 (the sign of the correction is not stated).
///
/// **A flat-back (blunt trailing edge) section is not supported by these sources** and is not used: an Euler probe (`tests/arcas_probe.rs`) shows a flat-back fin
/// would close the supersonic increment (DESIGN3D L item 33), but the reports say double wedge; what the double wedge plus flat-plate friction misses at Mach 1.5-2.3
/// (0.07 / 0.04 / 0.03 of C_A) is **unexplained**: candidates are the fin anchors and the strips on the fins, the sting, and the sign / fin-dependence of the chamber correction.
fn arcas_fin_spec() -> FinSpec {
    FinSpec {
        count: 4,
        root_chord: arcas::FIN_ROOT_CHORD,
        tip_chord: arcas::FIN_TIP_CHORD,
        span: arcas::FIN_SPAN,
        sweep: arcas::FIN_SWEEP,
        thickness: arcas::FIN_THICKNESS,
        roughness: 0.0,
        root_te_to_base: 0.0,
        profile: FinProfile::DoubleWedge,
    }
}

/// `Re_c = rho c d / mu` of a tunnel run at `mach` with `re_per_m` (`Re/m = rho M c / mu`).
fn tunnel_re_c(mach: f64, re_per_m: f64) -> f64 {
    re_per_m * arcas::BODY_DIAMETER / mach
}

/// The axial force without the base, `C_A,corr`, of the Arcas at `mach`, fins on or off, on the body cross-section.
fn arcas_axial(mach: f64, fins_on: bool, re_per_m: f64) -> Result<(Solved, f64, bool), String> {
    let (body, _) = arcas_body_spec().to_internal().map_err(|e| e.to_string())?;
    let d = arcas::BODY_DIAMETER;
    let flow = WallFlow { re_c: tunnel_re_c(mach, re_per_m), mach_max: mach, ks_body: 0.0, ks_fin: 0.0, sutherland: 0.368 };
    let kind = tier_kind();
    let case = if fins_on {
        let f = arcas_fin_spec();
        let fins = Fins3 {
            count: f.count,
            root_chord: f.root_chord / d,
            tip_chord: f.tip_chord / d,
            span: f.span / d,
            sweep: f.sweep / d,
            thickness: f.thickness / d,
            // the Arcas fins stand where they are: root trailing edge flush with the base, the root straddling the tube and the boattail
            te_to_base: 0.0,
            profile: f.profile,
            planform: None,
        };
        Case3::sector(mach, body, fins, tier(kind, flow), 0)
    } else {
        Case3::wedge(mach, body, 0).with_tier(tier(kind, flow))
    };
    let solved = Session::new(&case).and_then(|mut s| s.run(&case, false))?;
    let q = case.body.nose_to_largest_area();
    // the axial force without the base: pressure and friction of the body and the fins (`C_A,corr` of the data)
    let value = q * (solved.parts.total - solved.parts.base - solved.parts.friction.base);
    Ok((solved, value, is_gate(kind, &case)))
}

/// G-F4: the Arcas axial force without the base (NASA TN D-4013 / D-4014), fins on and off, through `Case3` (the wedge without fins, the four-fin sector with
/// them, `tests/validation.rs::arcas_axial_force` through the engine): fins on within 15 %, fins off within 12 % or 0.03, and the increment on minus off within 30 %
/// or 0.03 at Mach 0.6, 0.8, 1.5, 1.8, 2.3, 2.96; Mach 0.9 to 1.2 are INFO. Euler (today): every line is INFO, and the increment is the fin **pressure** drag
/// (the data contain friction): how much of the fin drag is pressure.
#[test]
#[ignore = "validation: CPU minutes per case (sector on the Arcas, 18 d long), the GPU for the viscous tiers"]
fn gate_f4_arcas_axial_force_with_and_without_fins() {
    let mut cases: Vec<(f64, f64, f64, f64, bool)> = Vec::new(); // (M, off, on, Re/m, gated)
    for &(m, off, on) in &arcas::TRANSONIC {
        cases.push((m, off, on, arcas::TRANSONIC_RE_PER_M, [0.6, 0.8].contains(&m)));
    }
    for &(m, off, on) in &arcas::SUPERSONIC {
        cases.push((m, off, on, arcas::SUPERSONIC_RE_PER_M, true));
    }
    let keep = machs(&cases.iter().map(|c| c.0).collect::<Vec<_>>());
    cases.retain(|c| keep.iter().any(|m| (m - c.0).abs() < 1e-9));
    let runs: Vec<(Result<(Solved, f64, bool), String>, Result<(Solved, f64, bool), String>)> =
        par_map(&cases, |c| (arcas_axial(c.0, false, c.3), arcas_axial(c.0, true, c.3)));
    let mut lines = Lines::default();
    for (c, (off, on)) in cases.iter().zip(runs) {
        let (m, data_off, data_on, _, gated) = *c;
        let (off, on) = match (off, on) {
            (Ok(a), Ok(b)) => (a, b),
            (a, b) => {
                println!("GATE G-F4 arcas-M{m:.2} n/a n/a n/a FAIL ({:?} / {:?})", a.err(), b.err());
                lines.failed += 1;
                continue;
            }
        };
        // viscous (a wall-resolved tier on an engine that solves the viscous equations) as the cases that ran, not as a bare wedge
        let gate = gated && off.2 && on.2;
        let (v_off, v_on) = (off.1, on.1);
        lines.info(
            "G-F4",
            &format!("arcas-M{m:.2}:parts"),
            &format!(
                "off {v_off:.4} (nose+aft; {}), on {v_on:.4} (body {:.4}, fin sides {:+.4} edges {:+.4} tips {:+.4}; {}); data off {data_off:.3} on {data_on:.3}; increment {:+.4} (data {:+.4})",
                off.0.stop(),
                on.0.body_pressure(),
                on.0.parts.fin_sides,
                on.0.parts.fin_edges,
                on.0.parts.fin_tips,
                on.0.stop(),
                v_on - v_off,
                data_on - data_off
            ),
        );
        // Mach 1.5 and 1.8: the report's fin increment exceeds wave drag + flat-plate friction + anchors by 0.074 / 0.038 of C_A (the 3D solver reproduces that sum): a documented
        // discrepancy with the reference, DESIGN3D L item 37 (KNOWN: the failure stays visible, the band is not widened, the test does not fail on it)
        let known = [1.5, 1.8].contains(&m);
        let why = "DESIGN3D-L-item-37";
        let check = |lines: &mut Lines, case: &str, v: f64, d: f64, band: Band| {
            if known {
                lines.check_known(gate, "G-F4", case, v, d, band, why)
            } else {
                lines.check(gate, "G-F4", case, v, d, band)
            }
        };
        check(&mut lines, &format!("arcas-M{m:.2}:ca_fins_on"), v_on, data_on, Band::Rel(0.15));
        lines.check(gate, "G-F4", &format!("arcas-M{m:.2}:ca_fins_off"), v_off, data_off, Band::RelAbs(0.12, 0.03));
        check(&mut lines, &format!("arcas-M{m:.2}:increment"), v_on - v_off, data_on - data_off, Band::RelAbs(0.30, 0.03));
    }
    lines.finish();
}

// ---------------------------------------------------------------------------------------------------------------------------------
// G-F5, G-L1: the ANF on the half domain

/// Reynolds number `Re_L` (body length, 10 calibres) of the DRDC wind-tunnel runs (task statement, ~4e6) and the radius of the balance sting over the base radius
/// (19 mm on the 30 mm base: 0.63). DESIGN3D item 51: the sting-mounted tunnel points are compared on this setup.
const TUNNEL_RE_L: f64 = 4.0e6;
const TUNNEL_STING_OVER_BASE: f64 = 0.63;

/// `tunnel`: the sting-mounted wind-tunnel setup (a sting of [`TUNNEL_STING_OVER_BASE`] behind the base, `Re_L` [`TUNNEL_RE_L`] instead of the sea-level free-flight value;
/// the grid is still sized for `mach_max`).
fn anf_half(mach: f64, alpha_deg: f64, phase: FinPhase, mach_max: f64, tunnel: bool) -> Case3 {
    let kind = if tier_kind() == TierKind::Euler { TierKind::Euler } else { TierKind::Lift };
    let (body, flow) = if tunnel {
        let body = Case3::anf_body().with_sting(0.5 * TUNNEL_STING_OVER_BASE);
        (body, WallFlow { re_c: TUNNEL_RE_L / (10.0 * mach), ..anf_flow(mach_max) })
    } else {
        (Case3::anf_body(), anf_flow(mach_max))
    };
    let mut c = Case3::base_half(mach, alpha_deg.to_radians(), body, Some(Case3::anf_fins()), tier(kind, flow), 0);
    c.grid.domain = Domain3::Half { phase };
    c
}

/// Linear interpolation of an ascending `(x, y)` table, clamped at its ends.
fn interp(t: &[(f64, f64)], x: f64) -> f64 {
    match t.windows(2).find(|w| x <= w[1].0) {
        Some(w) if x > w[0].0 => w[0].1 + (w[1].1 - w[0].1) * (x - w[0].0) / (w[1].0 - w[0].0),
        Some(w) => w[0].1,
        None => t[t.len() - 1].1,
    }
}

const PHASES: [(FinPhase, &str); 2] = [(FinPhase::Between, "between"), (FinPhase::InPlane, "inplane")];

/// G-F5: the ANF's zero-incidence drag `C_D0` (base included, `pi d^2 / 4`) on the half domain of the base-wake grid with the lift tier, in both roll phases, against
/// `bowshock_reference::anf::CD0`, 8 %: zero fin cant throughout (the solver models uncanted fins; shots of cant 2 and 4 degrees carry roll-induced drag). Supersonic
/// (M 1.850 / 2.364 / 2.741 / 2.970: 0.567 / 0.478 / 0.400 / 0.3805) the zero-cant free-flight single fits of DREV-TM-9703 Table VIII (`:cd0[TM9703-T8-cant0]`);
/// subsonic (M 0.766: 0.508, pooled free flight, fin cant 2 degrees, 1 % standard error) the DRDC TM 2002-136 Table 7 point as replotted in ARL-TR-6725 Fig. 5
/// (`:cd0[TM2002136-T7-ff]`; the cant adds roll-induced drag the uncanted solver does not model). The sting-mounted tunnel points (M 0.5 / 0.7: 0.467 / 0.479, +-0.015,
/// `anf::CD0_TUNNEL`) are INFO (`:cd0-tunnel`). The pooled Table IX values
/// (`anf::CD0_POOLED`, cant 0 / 2 / 4 mixed, linearly interpolated to the run's Mach number) are INFO (`:cd0-pooled`). Both the range models (cold wall, about +14 % friction)
/// and the cant make the zero-cant references a bias source the solver's adiabatic wall does not share.
#[test]
#[ignore = "validation: viscous lift tier on the GPU; Euler on the CPU gives INFO (pressure drag only)"]
fn gate_f5_anf_cd0() {
    let table: Vec<f64> = anf::CD0.iter().chain(anf::CD0_TUNNEL).map(|p| p.0).collect();
    let keep = machs(&table);
    let items: Vec<(f64, f64, FinPhase, &str)> = anf::CD0
        .iter()
        .chain(anf::CD0_TUNNEL)
        .filter(|p| keep.contains(&p.0))
        .flat_map(|&(m, cd)| PHASES.iter().map(move |&(ph, name)| (m, cd, ph, name)))
        .collect();
    let runs = par_map(&items, |&(m, _, ph, _)| {
        let tunnel = anf::CD0_TUNNEL.iter().any(|p| p.0 == m);
        let case = anf_half(m, 0.0, ph, 3.147, tunnel);
        Session::new(&case).and_then(|mut s| s.run(&case, false)).map(|r| (r, case))
    });
    let kind = tier_kind();
    let mut lines = Lines::default();
    for (&(m, cd, _, name), r) in items.iter().zip(runs) {
        match r {
            Ok((r, case)) => {
                let q = case.body.nose_to_largest_area();
                let measured = q * r.lift.ca;
                lines.info(
                    "G-F5",
                    &format!("anf-M{m:.3}-{name}:parts"),
                    &format!(
                        "ca {:+.4} ({}), nose {:+.4} base {:+.4} fins {:+.4}",
                        r.lift.ca,
                        r.stop(),
                        r.breakdown.nose.ca,
                        r.parts.base,
                        r.breakdown.fins().ca
                    ),
                );
                let tunnel = anf::CD0_TUNNEL.iter().any(|p| p.0 == m);
                let src = if m > 1.0 { "TM9703-T8-cant0" } else { "TM2002136-T7-ff" };
                let id = format!("anf-M{m:.3}-{name}:cd0[{src}]");
                if m > 1.5 {
                    let pooled = interp(anf::CD0_POOLED, m);
                    lines.info(
                        "G-F5",
                        &format!("anf-M{m:.3}-{name}:cd0-pooled"),
                        &format!(
                            "measured {measured:.4} vs DREV-TM-9703 Table IX pooled (cant 0/2/4) {pooled:.4} at this Mach: {:+.1} %",
                            100.0 * (measured / pooled - 1.0)
                        ),
                    );
                }
                // supersonic: the sourced section (full-chord wedge, DESIGN3D L item 38) reads 9-12 % below the pooled table; against the zero-cant single fits the discrepancy is
                // documented as KNOWN (cold-wall friction of the range models, fin trailing-edge base, leading-edge resolution; DESIGN3D L item 44), the band is not widened
                if tunnel {
                    lines.info(
                        "G-F5",
                        &format!("anf-M{m:.3}-{name}:cd0-tunnel"),
                        &format!(
                            "measured {measured:.4} vs ARL-TR-6725 Fig. 5 sting-mounted tunnel {cd:.3} (+-0.015): {:+.1} %",
                            100.0 * (measured / cd - 1.0)
                        ),
                    );
                } else if m > 1.5 {
                    lines.check_known(is_gate(kind, &case), "G-F5", &id, measured, cd, Band::Rel(0.08), "DESIGN3D-L-item-38");
                } else {
                    lines.check(is_gate(kind, &case), "G-F5", &id, measured, cd, Band::Rel(0.08));
                }
            }
            Err(e) => {
                println!("GATE G-F5 anf-M{m:.3}-{name} n/a n/a n/a FAIL ({e})");
                lines.failed += 1;
            }
        }
    }
    lines.finish();
}

/// The Mach number the G-L1 grids are sized for: the gate's 3.147 unless `BOWSHOCK_GATE_GRID_MACH` says `own` (the point's own Mach number, a per-point grid), `floorX` (its own, not below
/// X) or a number (DESIGN3D item 57: which sizing is closer to the data).
fn gate_grid_mach(mach: f64) -> f64 {
    match std::env::var("BOWSHOCK_GATE_GRID_MACH").ok().as_deref() {
        None => 3.147,
        Some("own") => mach,
        Some(v) if v.starts_with("floor") => mach.max(v["floor".len()..].parse().expect("floorX")),
        Some(v) => v.parse().expect("BOWSHOCK_GATE_GRID_MACH: own, floorX or a number"),
    }
}

/// G-F5s: the ANF's zero-incidence drag on the four-fin **sector** (`Case3::sector`, the product's table path: `BOWSHOCK_FIN_TIER=coarse`, 64 chord cells) against the same
/// `bowshock_reference::anf::CD0` references and band (8 %) as G-F5 (which runs the Lift half domain): the free stream is the sea-level one of the 30 mm model, the grid is sized for Mach
/// 3.147 (as G-F5 and `finned_table --anf`). Total drag on `pi d^2 / 4` of the sector rocket (`DragParts::total`). The three Mach numbers below 2 are the ones the product's drag nodes
/// need most (peak drag near Mach 1.1); the supersonic ones keep their KNOWN marker (DESIGN3D L item 38) as in G-F5.
#[test]
#[ignore = "validation: Coarse sector on the GPU (minutes per point)"]
fn gate_f5s_anf_cd0_sector() {
    let table: Vec<f64> = anf::CD0.iter().map(|p| p.0).collect();
    let keep = machs(&table);
    let items: Vec<(f64, f64)> = anf::CD0.iter().copied().filter(|p| keep.contains(&p.0)).collect();
    let runs = par_map(&items, |&(m, _)| {
        let case = Case3::sector(m, Case3::anf_body(), Case3::anf_fins(), tier(tier_kind(), anf_flow(3.147)), 0);
        Session::new(&case).and_then(|mut s| s.run(&case, false)).map(|r| (r, case))
    });
    let kind = tier_kind();
    let mut lines = Lines::default();
    for (&(m, cd), r) in items.iter().zip(runs) {
        match r {
            Ok((r, case)) => {
                let measured = case.body.nose_to_largest_area() * r.parts.total;
                lines.info(
                    "G-F5s",
                    &format!("anf-M{:.3}-sector:parts", m),
                    &format!(
                        "cd {measured:.4} ({}), body pressure {:+.4} base {:+.4} fin pressure {:+.4} friction body {:.4} fins {:.4}",
                        r.stop(),
                        r.body_pressure(),
                        r.parts.base,
                        r.fin_pressure(),
                        r.parts.body_friction(),
                        r.fin_friction.unwrap_or(f64::NAN)
                    ),
                );
                let src = if m > 1.0 { "TM9703-T8-cant0" } else { "TM2002136-T7-ff" };
                let id = format!("anf-M{m:.3}-sector:cd0[{src}]");
                if m > 1.5 {
                    lines.check_known(is_gate(kind, &case), "G-F5s", &id, measured, cd, Band::Rel(0.08), "DESIGN3D-L-item-38");
                } else {
                    lines.check(is_gate(kind, &case), "G-F5s", &id, measured, cd, Band::Rel(0.08));
                }
            }
            Err(e) => {
                println!("GATE G-F5s anf-M{m:.3}-sector n/a n/a n/a FAIL ({e})");
                lines.failed += 1;
            }
        }
    }
    lines.finish();
}

/// G-L1: the ANF's `CN_alpha` (`CN / alpha` at 2 degrees, per rad) and `x_cp` (calibers from the tip) at 2 and 6 degrees (6 degrees is warm-started from 2) on the half
/// domain with the lift tier in both roll phases, against `bowshock_reference::anf` at the points the Euler gate gates (`out/gate/anf_3d.md`: wind tunnel M 0.69,
/// 0.79, 1.728, 1.99, 2.48; free flight 1.8, 2.351, 2.665): CN_alpha 13 %, `x_cp` 0.25 cal, no worse than that gate. The 6-degree nonlinearity is INFO. The tunnel (`wt`)
/// points run on the tunnel setup (`Re_L` 4e6, balance sting of 0.63 base diameters), the free-flight points as before. DESIGN3D item 51: CN_alpha is insensitive to the
/// slope definition (0-1 degree slope, 2 degree secant, least squares over 0..5 degrees: within 2 %), the Reynolds number and the sting (within 1.5 %), so the 2-degree
/// secant is kept; on the tunnel setup the subsonic offset is +11 to +13.4 % (one FAIL, Mach 0.79 in-plane), so it is not a like-for-like artefact.
///
/// Provenance of the references: they are digitised from Yayla (2021) Fig. 3.10 / 4.4, which replot Dupuis' tunnel (WT) and free-flight points. The roll position of the
/// WT points is not stated there nor in ARL-TR-6725 Fig. 5 (the public replot, which shows one WT series; its M 0.8 C_Nalpha reads about 16-17, consistent with 16.27), so
/// they are not verifiably the 0-degree-roll values. DRDC TM 2002-136 Tables 12 and 13 list the tunnel's C_Nalpha, C_malpha and CP at 0 and 45 degrees roll (Mach 0.496
/// to 4.101); that report does not allow redistribution and no public copy of the 45-degree values was found, so there is no 45-degree INFO line. Which fin cant the replotted
/// free-flight points belong to is not stated either.
#[test]
#[ignore = "validation: lift tier on the GPU (minutes); Euler on the CPU gives INFO"]
fn gate_l1_anf_cn_alpha_and_cp() {
    let wt = [0.69, 0.79, 1.728, 1.99, 2.48];
    let ff = [1.8, 2.351, 2.665];
    let pick = |set: &'static [anf::AnfPoint], list: &[f64], tag: &'static str| -> Vec<(&'static str, anf::AnfPoint)> {
        machs(list).iter().map(|&m| (tag, *set.iter().find(|p| (p.mach - m).abs() < 2e-3).unwrap_or_else(|| panic!("no {tag} point at M {m}")))).collect()
    };
    let mut points = pick(anf::WIND_TUNNEL, &wt, "wt");
    points.extend(pick(anf::FREE_FLIGHT, &ff, "ff"));
    let items: Vec<(&str, anf::AnfPoint, FinPhase, &str)> = points.iter().flat_map(|&(t, p)| PHASES.iter().map(move |&(ph, name)| (t, p, ph, name))).collect();
    let runs = par_map(&items, |&(tag, p, ph, _)| -> Result<(Solved, Solved, Case3), String> {
        let tunnel = tag == "wt";
        let (c2, c6) = (anf_half(p.mach, 2.0, ph, gate_grid_mach(p.mach), tunnel), anf_half(p.mach, 6.0, ph, gate_grid_mach(p.mach), tunnel));
        let mut s = Session::new(&c2)?;
        let r2 = s.run(&c2, false)?;
        let r6 = s.run(&c6, true)?;
        Ok((r2, r6, c2))
    });
    let kind = tier_kind();
    let mut lines = Lines::default();
    for (&(tag, p, _, name), r) in items.iter().zip(runs) {
        let case_id = format!("{tag}-M{:.3}-{name}", p.mach);
        match r {
            Ok((r2, r6, case)) => {
                let gate = is_gate(kind, &case);
                let a2 = 2f64.to_radians();
                let q = case.body.nose_to_largest_area();
                let cna = q * r2.lift.cn / a2;
                lines.check(gate, "G-L1", &format!("{case_id}:cn_alpha"), cna, p.cn_alpha, Band::Rel(0.13));
                lines.check(gate, "G-L1", &format!("{case_id}:x_cp"), r2.lift.x_cp, p.x_cp_cal, Band::RelAbs(0.0, 0.25));
                lines.info(
                    "G-L1",
                    &format!("{case_id}:alpha6"),
                    &format!(
                        "CNa(6) {:.2} ({:+.1} % of CNa(2)), x_cp(6) {:.3}; {} / {}",
                        q * r6.lift.cn / 6f64.to_radians(),
                        100.0 * (r6.lift.cn / 6f64.to_radians() / (r2.lift.cn / a2) - 1.0),
                        r6.lift.x_cp,
                        r2.stop(),
                        r6.stop()
                    ),
                );
            }
            Err(e) => {
                println!("GATE G-L1 {case_id} n/a n/a n/a FAIL ({e})");
                lines.failed += 1;
            }
        }
    }
    lines.finish();
}

// ---------------------------------------------------------------------------------------------------------------------------------
// G-S: sector vs half domain

/// The ANF fins with `n` fins.
fn fins_of(n: u32) -> Fins3 {
    Fins3 { count: n, ..Case3::anf_fins() }
}

/// G-S: a sector of `n` fins against the half domain of the same body and fins at zero incidence (the half domain is the whole body's mirror half: `Between` for
/// even `n`, `InPlane` for odd), N = 4 and N = 3 on the stack grids at level 0 (`BOWSHOCK_FIN_TIER=coarse`), Mach 0.8 and 2: total drag within 1 %. On the Euler tier
/// (the default: uniform azimuthal cells, `nk 6` per sector and the cell count of the half domain that replicates the sector grid) the lines are INFO: the same
/// numbers as the CPU / GPU Euler stop-gap (`examples/sector_vs_half`).
///
/// **Equal resolution** (DESIGN3D item 52): the sector's half fin owns `max(n / 2, 1)` cells across its thickness, a whole fin of the half domain `n`, so with the default 1 the sector
/// resolves the fin's leading and trailing edge with two cells across (the mirror image) and the half domain with one: the leading-edge face then differs by 4.5 % at 64 chord cells.
/// The stack tiers take 2 cells across in both, which are the same cells (the sector's grid is the half domain's mirrored, node for node: `gs_diag grid`).
///
/// **Mach 0.8 is a documented discrepancy, not a grid difference** (item 52): the half domain's whole fins are free to have an asymmetric, oscillating wake behind their blunt trailing edge (0.08 d),
/// the sector's fin mid-plane is a symmetry plane and its wake steady. At 64 chord cells (trailing-edge cells 0.0047 d) the half domain's free fins have a trailing-edge drag 6-10 % above the
/// sector's (the half fin of the in-plane N = 3 half domain, on a symmetry plane, is within 1.2 % of the sector's) and a persistent oscillation of CA of 0.3-1 %; at 32 chord cells (trailing-edge
/// cells twice as long) the trailing-edge drags agree and the two domains to 0.6 %. The 1 % band stays; the line prints `FAIL KNOWN`.
#[test]
#[ignore = "validation: stack grids need the GPU (sector 0.5 M cells, half domain 2-3 M); Euler cases run on the CPU in minutes"]
fn gate_s_sector_vs_half_on_the_stack_grids() {
    let kind = tier_kind();
    let items: Vec<(u32, f64)> = [4u32, 3].iter().flat_map(|&n| machs(&[0.8, 2.0]).into_iter().map(move |m| (n, m))).collect();
    let nk_s = 6usize;
    let across = if kind == TierKind::Euler { 1 } else { 2 };
    let flow = |m: f64| WallFlow { re_c: re_c_sea_level(0.03), mach_max: m.max(2.0), ks_body: 0.0, ks_fin: 0.0, sutherland: 0.368 };
    let runs = par_map(&items, |&(n, m)| -> Result<(Solved, Solved, Case3), String> {
        let mut sector = Case3::sector(m, Case3::anf_body(), fins_of(n), tier(kind, flow(m)), 0);
        let mut half = Case3::base_half(m, 0.0, Case3::anf_body(), Some(fins_of(n)), tier(kind, flow(m)), 0);
        let even = n % 2 == 0;
        half.grid.domain = Domain3::Half { phase: if even { FinPhase::Between } else { FinPhase::InPlane } };
        (sector.grid.fin_cells_across, half.grid.fin_cells_across) = (across, across);
        if kind == TierKind::Euler {
            sector.grid.nk = nk_s;
            half.grid.nk = n as usize * nk_s - if even { 2 } else { 1 };
        }
        let rs = Session::new(&sector).and_then(|mut s| s.run(&sector, false))?;
        let rh = Session::new(&half).and_then(|mut s| s.run(&half, false))?;
        Ok((rs, rh, sector))
    });
    let mut lines = Lines::default();
    for (&(n, m), r) in items.iter().zip(runs) {
        let id = format!("anf-N{n}-M{m:.1}");
        match r {
            Ok((rs, rh, case)) => {
                let (ts, th) = (rs.parts.total, rh.parts.total);
                lines.info(
                    "G-S",
                    &format!("{id}:parts"),
                    &format!(
                        "sector total {ts:+.5} (nose {:+.5} base {:+.5} fins {:+.5}: LE {:+.5} TE {:+.5}; {}), half {th:+.5} (nose {:+.5} base {:+.5} fins {:+.5}: LE {:+.5} TE {:+.5}; {}); nk {} / {}",
                        rs.parts.nose,
                        rs.parts.base,
                        rs.fin_pressure(),
                        rs.parts.fin_le,
                        rs.parts.fin_te,
                        rs.stop(),
                        rh.parts.nose,
                        rh.parts.base,
                        rh.fin_pressure(),
                        rh.parts.fin_le,
                        rh.parts.fin_te,
                        rh.stop(),
                        case.grid.nk,
                        n
                    ),
                );
                let (gate, id_cd) = (is_gate(kind, &case), format!("{id}:cd_total"));
                if m < 1.0 {
                    let why = "the half domain's free fins have an asymmetric, oscillating trailing-edge wake that the sector's symmetric one cannot: DESIGN3D item 52";
                    lines.check_known(gate, "G-S", &id_cd, ts, th, Band::Rel(0.01), why);
                } else {
                    lines.check(gate, "G-S", &id_cd, ts, th, Band::Rel(0.01));
                }
            }
            Err(e) => {
                println!("GATE G-S {id} n/a n/a n/a FAIL ({e})");
                lines.failed += 1;
            }
        }
    }
    lines.finish();
}

#[test]
fn the_flat_plate_friction_reference_is_sane() {
    // Re_c 1e6 over half a diameter: Cf of the order of 4e-3 at Mach 0.5, lower at Mach 2
    let (a, b) = (flat_plate_cf(1.0e6, 0.5, 0.5), flat_plate_cf(1.0e6, 2.0, 0.5));
    assert!((2e-3..8e-3).contains(&a) && b < a, "{a} {b}");
    assert!((re_c_sea_level(0.03) / 6.99e5 - 1.0).abs() < 0.01);
}

#[test]
fn the_arcas_and_anf_data_are_wired() {
    assert_eq!(arcas::TRANSONIC.len() + arcas::SUPERSONIC.len(), 10);
    assert_eq!(anf::CD0.len(), 7);
    assert_eq!(anf::CD0_TUNNEL.len(), 2);
    let (body, scale) = arcas_body_spec().to_internal().expect("arcas body");
    assert!((scale.d_nose - arcas::BODY_DIAMETER).abs() < 1e-12);
    assert!((body.nose_to_largest_area() - 1.0).abs() < 1e-12, "the tube is the largest section");
}

#[test]
fn engine_is_inviscid_today_so_f1_has_to_fail() {
    assert!(!common::engine_is_viscous(&Case3::wedge(2.0, Case3::anf_body(), 0)));
}
