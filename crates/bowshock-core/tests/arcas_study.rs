//! The Arcas supersonic fin increment, case by case (DESIGN3D L items 33-37): one line per run with every part of the axial force, for the grid variants of the
//! Arcas sector (chordwise cells, tier, level, tip rows) and the fin section. `#[ignore]`d; the CPU is minutes per case, the GPU
//! (`BOWSHOCK_WEDGE_BACKEND=gpu`) seconds.
//!
//! `STUDY_MACH=1.5 STUDY_TIER=coarse|fine|euler STUDY_LEVEL=0 STUDY_CHORD=32 STUDY_SPAN=0 STUDY_FINS=on|off STUDY_PROFILE=dw|blunt cargo test --release -p bowshock-core --test arcas_study -- --ignored --nocapture`

#[path = "fin_gates/common.rs"]
mod common;

use bowshock_core::engine::case3::{Case3, Tier, WallFlow};
use bowshock_core::engine::grid3::Fins3;
use bowshock_core::{AftSpec, BodySpec, FinProfile, NoseShape, NoseSpec};
use bowshock_reference::arcas;
use common::Session;

fn env<T: std::str::FromStr>(key: &str, default: T) -> T {
    std::env::var(key).ok().and_then(|v| v.parse().ok()).unwrap_or(default)
}

#[test]
#[ignore = "study: CPU minutes, GPU seconds"]
fn the_arcas_axial_force_by_part() {
    let spec = BodySpec {
        nose: NoseSpec { shape: NoseShape::TangentOgive, length: arcas::NOSE_LENGTH, diameter: arcas::BODY_DIAMETER },
        aft: vec![AftSpec::Tube { length: arcas::TUBE_LENGTH }, AftSpec::Transition { length: arcas::BOATTAIL_LENGTH, aft_diameter: arcas::BASE_DIAMETER }],
    };
    let (body, _) = spec.to_internal().expect("body");
    let d = arcas::BODY_DIAMETER;
    let mach: f64 = env("STUDY_MACH", 1.5);
    let re_per_m = if mach < 1.3 { arcas::TRANSONIC_RE_PER_M } else { arcas::SUPERSONIC_RE_PER_M };
    let flow = WallFlow { re_c: re_per_m * d / mach, mach_max: mach, ks_body: 0.0, ks_fin: 0.0, sutherland: 0.368 };
    let tier_name: String = env("STUDY_TIER", "coarse".to_string());
    let level: u32 = env("STUDY_LEVEL", 0);
    let chord: usize = env("STUDY_CHORD", 0);
    let span: usize = env("STUDY_SPAN", 0);
    let fins_on = env("STUDY_FINS", "on".to_string()) == "on";
    let profile_name: String = env("STUDY_PROFILE", "dw".to_string());
    let profile = match profile_name.as_str() {
        "blunt" => FinProfile::Custom { section: vec![[0.0, 0.0], [0.4, 1.0], [1.0, 1.0]] },
        _ => FinProfile::DoubleWedge,
    };
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
    let anf = env("STUDY_BODY", "arcas".to_string()) == "anf";
    let (body, fins, flow) = if anf {
        // the Army-Navy Finner (0.03 m) at the sea-level atmosphere, the grid sized for Mach 3.147 as in G-F5
        let flow = WallFlow { re_c: 1.225 * 340.294 * 0.03 / 1.7894e-5, mach_max: env("STUDY_GRID_MACH", 3.147), ks_body: 0.0, ks_fin: 0.0, sutherland: 0.368 };
        let mut f = Case3::anf_fins();
        // STUDY_LE=<f>: the leading-edge wedge of the flat-back section over the fraction f of the chord (the code's default is 0.1: a 21.8 degree half angle)
        if let Some(le) = std::env::var("STUDY_LE").ok().and_then(|v| v.parse::<f64>().ok()) {
            // STUDY_Z0=<z>: the leading-edge thickness as a fraction of the maximum (a blunt edge: DRDC gives a leading-edge radius of 0.004 cal on a 0.08 cal trailing edge, z = 0.1)
            let z0: f64 = env("STUDY_Z0", 0.0);
            f.profile = FinProfile::Custom { section: vec![[0.0, z0], [le, 1.0], [1.0, 1.0]] };
        }
        (Case3::anf_body(), f, flow)
    } else {
        (body, fins, flow)
    };
    let tier = match tier_name.as_str() {
        "fine" => Tier::Fine(flow),
        "euler" => Tier::Euler,
        "draft" => Tier::draft(flow),
        _ => Tier::Coarse(flow),
    };
    let mut case = if fins_on { Case3::sector(mach, body, fins, tier, level) } else { Case3::wedge(mach, body, level).with_tier(tier) };
    if chord > 0 {
        case.grid.fin_chord_cells = Some(chord);
    }
    if span > 0 {
        case.grid.span_cells = Some(span);
    }
    let g = bowshock_core::engine::grid3::body_grid3(&case.grid).expect("grid");
    let t0 = std::time::Instant::now();
    let r = Session::new(&case).and_then(|mut s| s.run(&case, false)).expect("run");
    let q = case.body.nose_to_largest_area();
    let p = &r.parts;
    let ca_nobase = q * (p.total - p.base - p.friction.base);
    println!(
        "STUDY {} {tier_name} L{level} chord {} span {} {profile_name} fins {} M {mach}: {} x {} x {} = {} cells, {} ({:.0} s); C_A without base {:.4}, total {:.4} | pressure nose {:.4} aft {:.4} fin sides {:+.4} edges {:+.4} tips {:+.4} | friction body {:.4} fin {:.4} | base {:.4}",
        if anf { "anf" } else { "arcas" },
        if chord > 0 { chord.to_string() } else { "default".to_string() },
        if span > 0 { span.to_string() } else { "default".to_string() },
        if fins_on { "on" } else { "off" },
        g.ni,
        g.nj,
        g.nk,
        g.ni * g.nj * g.nk,
        r.stop(),
        t0.elapsed().as_secs_f64(),
        ca_nobase,
        q * p.total,
        q * p.nose,
        q * p.aft.iter().sum::<f64>(),
        q * p.fin_sides,
        q * p.fin_edges,
        q * p.fin_tips,
        q * (p.friction.nose + p.friction.aft.iter().sum::<f64>()),
        q * (p.friction.fin_sides + p.friction.fin_edges + p.friction.fin_tips),
        q * p.base
    );
    println!(
        "STUDY-BASE {} {} M {mach}: body base Cp {:+.3} (base radius {:.4})",
        if anf { "anf" } else { "arcas" },
        if fins_on { "fins on" } else { "fins off" },
        -p.base / (case.body.base_radius() / 0.5).powi(2),
        case.body.base_radius()
    );
    if let Some(f) = case.grid.fins.as_ref().filter(|_| fins_on) {
        // the leading and trailing edge faces apart, as pressure coefficients on their own area (units of the nose reference area pi/4; positive = overpressure on the
        // leading edge, suction behind the trailing edge); the leading-edge half thickness after the mesher's floors (2 % of t/2 at a sharp edge, 10 % at a blunt one)
        let z0 = match &f.profile {
            FinProfile::Custom { section } => section[0][1] / section.iter().fold(0.0f64, |m, p| m.max(p[1])),
            _ => 0.0,
        };
        let le_frac = if z0 < 0.05 { 0.02 } else { z0.max(0.1) };
        let area = |h: f64| f64::from(f.count) * 2.0 * h * f.span / std::f64::consts::FRAC_PI_4;
        let (a_te, a_le) = (area(0.5 * f.thickness), area(0.5 * f.thickness * le_frac));
        let a_base = (case.body.base_radius() / 0.5).powi(2);
        println!(
            "STUDY-EDGES {} M {mach}: leading edge {:+.4} (nose-reference units) on {:.4} areas = Cp {:+.3}; trailing edge {:+.4} on {:.4} = Cp {:+.3}; body base {:+.4} on {:.3} = Cp {:+.3}",
            if anf { "anf" } else { "arcas" },
            p.fin_le,
            a_le,
            p.fin_le / a_le,
            p.fin_te,
            a_te,
            -p.fin_te / a_te,
            p.base,
            a_base,
            -p.base / a_base
        );
    }
}
