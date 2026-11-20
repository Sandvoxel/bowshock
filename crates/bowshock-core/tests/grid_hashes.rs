//! Fingerprints (FNV-1a of the node coordinates, the part flags and the solid boxes) of the grids of the gate rockets and the Basic Finner: the Coarse and Fine sector, the Lift half
//! domain and the finless wedge, at a few Mach numbers. A mesher change that must leave those rockets' grids as they are is checked by running this before and after and comparing the
//! `HASH` lines (DESIGN3D items 66 and 73). Mesher only, seconds: `cargo test --release -p bowshock-core --test grid_hashes -- --ignored --nocapture`.

#[path = "fin_gates/common.rs"]
mod common;
#[path = "fin_gates/rockets.rs"]
mod rockets;

use bowshock_cfd::grid3::Grid3;
use bowshock_core::engine::case3::{Case3, Tier, WallFlow};
use bowshock_core::engine::grid3::{body_grid3, Domain3, FinPhase};
use bowshock_reference::{arcas, arcas_stab};
use common::TierKind;
use rockets::{arcas_rocket, grm_rocket, yuska_rocket, Rocket};

fn fingerprint(g: &Grid3) -> u64 {
    let mut h = 0xcbf2_9ce4_8422_2325u64;
    let mut eat = |v: u64| {
        for b in v.to_le_bytes() {
            h = (h ^ u64::from(b)).wrapping_mul(0x0100_0000_01b3);
        }
    };
    [g.ni, g.nj, g.nk].iter().for_each(|&n| eat(n as u64));
    g.x.iter().chain(&g.y).chain(&g.z).for_each(|v| eat(v.to_bits()));
    g.part.iter().for_each(|&p| eat(p as u64));
    g.solid.iter().for_each(|b| [b.i.start, b.i.end, b.j.start, b.j.end, b.k.start, b.k.end].iter().for_each(|&n| eat(n as u64)));
    h
}

fn show(name: &str, case: &Case3) {
    match body_grid3(&case.grid) {
        Ok(g) => println!("HASH {name}: {} x {} x {} {:016x}", g.ni, g.nj, g.nk, fingerprint(&g)),
        Err(e) => println!("HASH {name}: error {e}"),
    }
}

fn rocket_hashes(r: &Rocket) {
    for (kind, label) in [(TierKind::Coarse, "coarse"), (TierKind::Fine, "fine")] {
        for mach in [0.8, 2.0] {
            show(&format!("{} sector {label} M{mach}", r.name), &r.sector(mach, 3.0, kind));
        }
        show(&format!("{} wedge {label}", r.name), &r.finless(2.0, 3.0, kind));
    }
    for phase in [FinPhase::Between, FinPhase::InPlane] {
        show(&format!("{} half lift {phase:?}", r.name), &r.half(0.8, 3.0, 2.0, phase, TierKind::Lift));
    }
}

#[test]
#[ignore = "prints the fingerprints; compare two builds"]
fn grid_hashes() {
    for r in [
        arcas_rocket(0.0, arcas::SUPERSONIC_RE_PER_M, arcas::SUPERSONIC_T0_K),
        arcas_rocket(arcas_stab::LONG_TUBE_LENGTH - arcas::TUBE_LENGTH, arcas::SUPERSONIC_RE_PER_M, arcas::SUPERSONIC_T0_K),
        yuska_rocket(false, false, 2.0e6),
        yuska_rocket(true, true, 2.0e6),
        grm_rocket(),
    ] {
        rocket_hashes(&r);
    }
    // the Basic Finner (ANF): the case the fin gates use
    let flow = WallFlow { re_c: 1.0e6, mach_max: 3.0, ks_body: 0.0, ks_fin: 0.0, sutherland: 0.4 };
    for (tier, label) in [(Tier::Coarse(flow), "coarse"), (Tier::Fine(flow), "fine")] {
        show(&format!("anf sector {label}"), &Case3::sector(2.0, Case3::anf_body(), Case3::anf_fins(), tier, 0));
        show(&format!("anf wedge {label}"), &Case3::wedge(2.0, Case3::anf_body(), 0).with_tier(tier));
    }
    let lift = Tier::Lift(flow);
    for phase in [FinPhase::Between, FinPhase::InPlane] {
        let mut c = Case3::base_half(0.8, 2f64.to_radians(), Case3::anf_body(), Some(Case3::anf_fins()), lift, 0);
        c.grid.domain = Domain3::Half { phase };
        show(&format!("anf half lift {phase:?}"), &c);
    }
}

/// The Cancer Descending fixture's sector (DESIGN3D item 73) as the app builds it: three 61 degree rounded fins whose tip trailing edge is 0.42 d behind the root's (the root's is 0.84 d
/// ahead of the base). Prints the quality numbers and the hash.
#[test]
#[ignore = "prints the grid quality of the Cancer Descending sector"]
fn cancer_sector_quality() {
    use bowshock_core::engine::body_grid::{BaseBody, Body, Nose};
    use bowshock_core::engine::grid3::{print_quality, Fins3};
    use bowshock_core::FinProfile;
    let d = 6.0;
    let nose = Body { nose: Nose::TangentOgive { fineness: 4.0 } };
    let body = BaseBody::cylinder(nose, 85.0 / d);
    let fins = Fins3 {
        count: 3,
        root_chord: 19.0 / d,
        tip_chord: 5.5 / d,
        span: 9.0 / d,
        sweep: 16.0 / d,
        thickness: 0.375 / d,
        te_to_base: 5.05 / d,
        profile: FinProfile::Rounded,
        planform: None,
    };
    let flow = WallFlow { re_c: 3.2e6, mach_max: 1.0, ks_body: 0.0, ks_fin: 0.0, sutherland: 0.4 };
    let case = Case3::sector(0.5, body, fins, Tier::Coarse(flow), 0);
    let g = body_grid3(&case.grid).expect("builds");
    print_quality("cancer", &g);
    if let Ok(w) = std::env::var("QUAD") {
        // QUAD=i0,i1,j0,j1,k: the (x, r) of a window of nodes
        let v: Vec<usize> = w.split(',').map(|x| x.parse().unwrap()).collect();
        for j in (v[2]..=v[3]).rev() {
            let row: Vec<String> = (v[0]..=v[1])
                .map(|i| {
                    let n = (v[4] * (g.nj + 1) + j) * (g.ni + 1) + i;
                    format!("{:.4},{:.4}", g.x[n], g.y[n].hypot(g.z[n]))
                })
                .collect();
            println!("QUAD j {j}: {}", row.join(" | "));
        }
    }
    println!("HASH cancer sector coarse: {} x {} x {} {:016x}", g.ni, g.nj, g.nk, fingerprint(&g));
}
