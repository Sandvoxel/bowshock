//! G-F4 probe: the Arcas axial force without the base on the CPU Euler sector with different fin sections, to see what the missing supersonic fin increment is made of.
//! `PROFILE=dw|blunt|hex|square MACH=1.5 FINS=on|off cargo test -p bowshock-core --test arcas_probe -- --ignored --nocapture`.

use bowshock_core::engine::case3::Case3;
use bowshock_core::engine::grid3::Fins3;
use bowshock_core::engine::run3::{drag_parts_on, run3_on};
use bowshock_core::{AftSpec, BodySpec, FinProfile, NoseShape, NoseSpec};
use bowshock_reference::arcas;

fn env<T: std::str::FromStr>(key: &str, default: T) -> T {
    std::env::var(key).ok().and_then(|v| v.parse().ok()).unwrap_or(default)
}

#[test]
#[ignore = "Euler on the CPU: tens of minutes"]
fn the_arcas_axial_force_with_a_fin_section() {
    let spec = BodySpec {
        nose: NoseSpec { shape: NoseShape::TangentOgive, length: arcas::NOSE_LENGTH, diameter: arcas::BODY_DIAMETER },
        aft: vec![AftSpec::Tube { length: arcas::TUBE_LENGTH }, AftSpec::Transition { length: arcas::BOATTAIL_LENGTH, aft_diameter: arcas::BASE_DIAMETER }],
    };
    let (body, _) = spec.to_internal().expect("body");
    let d = arcas::BODY_DIAMETER;
    let mach: f64 = env("MACH", 1.5);
    let profile_name: String = env("PROFILE", "dw".to_string());
    let profile = match profile_name.as_str() {
        "blunt" => FinProfile::Custom { section: vec![[0.0, 0.0], [0.4, 1.0], [1.0, 1.0]] },
        "hex" => FinProfile::Hexagonal { wedge_fraction: 0.25 },
        "square" => FinProfile::Square,
        _ => FinProfile::DoubleWedge,
    };
    let fins_on = env("FINS", "on".to_string()) == "on";
    let fins = Fins3 {
        count: 4,
        root_chord: arcas::FIN_ROOT_CHORD / d,
        tip_chord: arcas::FIN_TIP_CHORD / d,
        span: arcas::FIN_SPAN / d,
        sweep: arcas::FIN_SWEEP / d,
        thickness: arcas::FIN_THICKNESS / d,
        te_to_base: 0.0,
        profile,
        planform: None,
    };
    let case = if fins_on { Case3::sector(mach, body, fins, bowshock_core::engine::case3::Tier::Euler, 0) } else { Case3::wedge(mach, body, 0) };
    let mut s = case.build_solver().expect("grid");
    let t0 = std::time::Instant::now();
    let r = run3_on(&case, &mut s, false);
    let p = drag_parts_on(&case, &s);
    let q = case.body.nose_to_largest_area();
    println!(
        "ARCAS {profile_name} fins {} M {mach}: {} iterations {:?} {:.0} s; C_A without base {:.4} (nose {:.4} aft {:.4} fin sides {:+.4} edges {:+.4} tips {:+.4}) base {:.4}",
        if fins_on { "on" } else { "off" },
        r.iterations,
        r.stop_reason,
        t0.elapsed().as_secs_f64(),
        q * (p.total - p.base),
        q * p.nose,
        q * p.aft.iter().sum::<f64>(),
        q * p.fin_sides,
        q * p.fin_edges,
        q * p.fin_tips,
        q * p.base
    );
}
