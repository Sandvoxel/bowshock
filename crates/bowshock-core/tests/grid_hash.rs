//! The grids of the example rocket and of the validation rockets are untouched by the swept flat-face rule (DESIGN3D item 72, `Grid3Params::fin_across_swept`): the FNV hash of
//! the node coordinates with the rule on equals the hash with it off (which is the mesher before the rule) for the Coarse and Fine sector, the Coarse and Fine wedge and the
//! Lift half domain of the example rocket, the Basic Finner, Arcas, Yuska and the AEDC GRM, and the rule does change Byrum's. Mesher only, seconds.
//! `cargo test --release -p bowshock-core --test grid_hash -- --ignored --nocapture`.

#[path = "fin_gates/common.rs"]
mod common;
#[path = "fin_gates/rockets.rs"]
mod rockets;

use bowshock_core::engine::body_grid::{BaseBody, Body, Nose};
use bowshock_core::engine::case3::{Case3, Tier, WallFlow, SUTHERLAND_SEA_LEVEL};
use bowshock_core::engine::grid3::{body_grid3, Domain3, FinPhase, Fins3};
use bowshock_core::FinProfile;
use bowshock_reference::{arcas, yuska};
use common::TierKind;
use rockets::{arcas_rocket, grm_rocket, yuska_rocket};

/// The hash of the grid, or the error that refuses it (a combination that does not build is compared by its message).
fn fnv(c: &Case3) -> Result<u64, String> {
    let g = body_grid3(&c.grid).map_err(|e| e.to_string())?;
    let mut h = 0xcbf2_9ce4_8422_2325u64;
    for v in g.x.iter().chain(&g.y).chain(&g.z) {
        for b in v.to_bits().to_le_bytes() {
            h = (h ^ u64::from(b)).wrapping_mul(0x0100_0000_01b3);
        }
    }
    Ok(h)
}

/// The hash of the case's grid with the rule on and off; asserted equal unless `differs`.
fn check(name: &str, c: Case3, differs: bool) {
    let on = fnv(&c);
    let mut off = c;
    off.grid.fin_across_swept = false;
    let off = fnv(&off);
    println!("HASH {name}: rule on {on:x?} off {off:x?}{}", if on == off { "" } else { "  DIFFERENT" });
    assert_eq!(on != off, differs, "{name}");
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

fn byrum_fins() -> Fins3 {
    let d = 3.0;
    Fins3 {
        count: 3,
        root_chord: 11.75 / d,
        tip_chord: 1.875 / d,
        span: 3.1875 / d,
        sweep: 7.875 / d,
        thickness: 0.25 / d,
        te_to_base: 1.25 / d,
        profile: FinProfile::Square,
        planform: None,
    }
}

fn half(c: Case3, phase: FinPhase) -> Case3 {
    let mut c = c;
    c.grid.domain = Domain3::Half { phase };
    c
}

#[test]
#[ignore = "mesher only; prints"]
fn the_swept_flat_face_rule_leaves_the_validation_grids_alone() {
    let flow = |re_c: f64, mach_max: f64| WallFlow { re_c, mach_max, ks_body: 0.0, ks_fin: 0.0, sutherland: SUTHERLAND_SEA_LEVEL };
    let ex_body = || BaseBody::cylinder(Body { nose: Nose::TangentOgive { fineness: 0.2286 / 0.0762 } }, 0.9134 / 0.0762);
    let ex = flow(1.7e6, 3.0);
    let f1 = flow(1.0e6, 2.0);
    // the example rocket
    for (name, tier) in [("coarse", Tier::Coarse(ex)), ("fine", Tier::Fine(ex)), ("draft", Tier::draft(ex))] {
        check(&format!("example sector {name}"), Case3::sector(2.0, ex_body(), example_fins(), tier, 0), false);
    }
    for (name, tier) in [("coarse", Tier::Coarse(ex)), ("fine", Tier::Fine(ex))] {
        check(&format!("example wedge {name}"), Case3::wedge(2.0, ex_body(), 0).with_tier(tier), false);
    }
    for phase in [FinPhase::Between, FinPhase::InPlane] {
        check(
            &format!("example half Lift {phase:?}"),
            half(Case3::base_half(2.0, 2f64.to_radians(), ex_body(), Some(example_fins()), Tier::Lift(ex), 0), phase),
            false,
        );
    }
    // the Basic Finner (ANF)
    for (name, tier) in [("coarse", Tier::Coarse(f1)), ("fine", Tier::Fine(f1))] {
        check(&format!("finner sector {name}"), Case3::sector(2.0, Case3::anf_body(), Case3::anf_fins(), tier, 0), false);
    }
    check("finner wedge coarse", Case3::wedge(2.0, Case3::anf_body(), 0).with_tier(Tier::Coarse(f1)), false);
    check(
        "finner half Lift",
        half(Case3::base_half(2.0, 2f64.to_radians(), Case3::anf_body(), Some(Case3::anf_fins()), Tier::Lift(f1), 0), FinPhase::Between),
        false,
    );
    // Arcas, Yuska, AEDC GRM
    let rockets = [arcas_rocket(0.0, arcas::TRANSONIC_RE_PER_M, arcas::TRANSONIC_T0_K), yuska_rocket(false, false, yuska::TEST_CONDITIONS[0].3), grm_rocket()];
    for r in &rockets {
        for kind in [TierKind::Coarse, TierKind::Fine] {
            check(&format!("{} sector {kind:?}", r.name), r.sector(2.0, 3.0, kind), false);
            check(&format!("{} wedge {kind:?}", r.name), r.finless(2.0, 3.0, kind), false);
        }
        check(&format!("{} half Lift", r.name), r.half(2.0, 3.0, 2.0, FinPhase::Between, TierKind::Lift), false);
    }
    // Byrum's fins on the example body's tube: the rule does apply
    let by = flow(1.0e6, 3.0);
    let body = BaseBody::cylinder(Body { nose: Nose::TangentOgive { fineness: 13.0 / 3.0 } }, 71.75 / 3.0);
    check("byrum sector coarse", Case3::sector(0.8, body.clone(), byrum_fins(), Tier::Coarse(by), 1), true);
    check("byrum sector draft", Case3::sector(0.8, body, byrum_fins(), Tier::draft(by), 0), false);
}
