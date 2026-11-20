//! Study of the incidence response of the gate articles (DESIGN3D item 67): CN, the pitching moment and `x_cp` at a list of incidences in the order given (the first cold, the others warm
//! from the previous), both roll phases, fins on or off, with the nose / afterbody / fin split of CN. Not a gate: prints `STAB` lines.
//!
//! ```text
//! BOWSHOCK_WEDGE_BACKEND=gpu STUDY_ROCKET=arcas STUDY_MACH=0.8 STUDY_ALPHAS=4,2,1,0 [STUDY_PHASE=inplane|between] [STUDY_FINS=1|0] [STUDY_TIER=lift|coarse] [STUDY_MACH_MAX=2.96]
//!   cargo test --release -p bowshock-core --test stab_study -- --ignored --nocapture
//! ```
//! `STUDY_ROCKET`: `arcas`, `yuskaA`, `yuskaB`, `yuskaC`, `grm`. `STUDY_RE_FT` overrides the Yuska Reynolds number per foot (default: the Table II row of the Mach number).

#[path = "fin_gates/common.rs"]
mod common;
#[path = "fin_gates/rockets.rs"]
mod rockets;

use bowshock_core::engine::case3::Case3;
use bowshock_core::engine::grid3::{Domain3, FinPhase};
use bowshock_reference::{arcas, yuska};
use common::{Session, TierKind};
use rockets::{arcas_rocket, grm_rocket, yuska_rocket, Rocket};

fn env_f(name: &str, default: f64) -> f64 {
    std::env::var(name).ok().map_or(default, |v| v.parse().unwrap_or_else(|_| panic!("{name}: number")))
}

#[test]
#[ignore = "study: GPU"]
fn stab_study() {
    let mach = env_f("STUDY_MACH", 0.8);
    let name = std::env::var("STUDY_ROCKET").unwrap_or_else(|_| "arcas".into());
    let alphas: Vec<f64> = std::env::var("STUDY_ALPHAS").unwrap_or_else(|_| "4,2".into()).split(',').map(|a| a.trim().parse().expect("STUDY_ALPHAS: numbers")).collect();
    let phase = match std::env::var("STUDY_PHASE").as_deref() {
        Ok("between") => FinPhase::Between,
        _ => FinPhase::InPlane,
    };
    let fins_on = std::env::var("STUDY_FINS").as_deref() != Ok("0");
    let kind = match std::env::var("STUDY_TIER").as_deref() {
        Ok("coarse") => TierKind::Coarse,
        _ => TierKind::Lift,
    };
    let re_ft = |m: f64| env_f("STUDY_RE_FT", yuska::TEST_CONDITIONS.iter().find(|c| (c.0 - m).abs() < 1e-9).map_or(2.4e6, |c| c.3));
    let r: Rocket = match name.as_str() {
        "arcas" if mach < 1.3 => arcas_rocket(0.0, arcas::TRANSONIC_RE_PER_M, arcas::TRANSONIC_T0_K),
        "arcas" => arcas_rocket(0.0, arcas::SUPERSONIC_RE_PER_M, arcas::SUPERSONIC_T0_K),
        "yuskaA" => yuska_rocket(false, false, re_ft(mach)),
        "yuskaB" => yuska_rocket(true, false, re_ft(mach)),
        "yuskaC" => yuska_rocket(false, true, re_ft(mach)),
        "grm" => grm_rocket(),
        other => panic!("STUDY_ROCKET {other}"),
    };
    let mach_max = env_f("STUDY_MACH_MAX", if name == "arcas" { 2.96 } else if name == "grm" { 1.3 } else { 3.47 });
    let q = r.area_factor();
    let build = |alpha: f64| -> Case3 {
        let mut c = r.half(mach, mach_max, alpha, phase, kind);
        if !fins_on {
            let (body, _) = r.body.to_internal().expect("body");
            c = Case3::base_half(mach, alpha.to_radians(), body, None, common::tier(kind, r.flow(mach, mach_max)), 0);
            c.grid.domain = Domain3::Half { phase };
        }
        c
    };
    let cases: Vec<Case3> = alphas.iter().map(|&a| build(a)).collect();
    let mut s = Session::new(&cases[0]).expect("session");
    for (k, (c, &a)) in cases.iter().zip(&alphas).enumerate() {
        let t = std::time::Instant::now();
        let o = s.run(c, k > 0).expect("run");
        let cn = q * o.lift.cn;
        let x = o.lift.x_cp; // nose diameters from the tip
        let part = |p: &bowshock_core::engine::run3::PartForces| q * p.cn;
        let b = &o.breakdown;
        println!(
            "STAB {name} M{mach:.3} {:?} fins {} alpha {a:.2} CN {cn:.4} CNa {:.3} /rad x_cp {x:.3} cal CM(nose) {:.4} | nose {:.4} aft {:.4} fins {:.4} | CA {:.4} | {} | {:.0} s",
            phase,
            fins_on as u8,
            if a != 0.0 { cn / a.to_radians() } else { f64::NAN },
            -cn * x,
            part(&b.nose),
            part(&b.afterbody),
            part(&b.fins()),
            o.lift.ca * q,
            o.stop(),
            t.elapsed().as_secs_f64()
        );
    }
}
