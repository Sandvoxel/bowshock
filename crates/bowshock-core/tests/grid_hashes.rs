//! Fingerprints (FNV-1a of the node coordinates, the part flags and the solid boxes) of the grids of the gate rockets and the Basic Finner: the Coarse and Fine sector, the Lift half
//! domain and the finless wedge, at a few Mach numbers. A mesher change that must leave those rockets' grids as they are is checked by running this before and after and comparing the
//! `HASH` lines (DESIGN3D items 66 and 73). Mesher only, seconds: `cargo test --release -p bowshock-core --test grid_hashes -- --ignored --nocapture`.

#[path = "fin_gates/common.rs"]
mod common;
#[path = "fin_gates/rockets.rs"]
mod rockets;

use bowshock_cfd::grid3::{Grid3, Part};
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
/// ahead of the base).
fn cancer_sector() -> Grid3 {
    use bowshock_core::engine::body_grid::{BaseBody, Body, Nose};
    use bowshock_core::engine::grid3::Fins3;
    use bowshock_core::FinProfile;
    let d = 6.0;
    let body = BaseBody::cylinder(Body { nose: Nose::TangentOgive { fineness: 4.0 } }, 85.0 / d);
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
    body_grid3(&Case3::sector(0.5, body, fins, Tier::Coarse(flow), 0).grid).expect("builds")
}

/// The smallest angle (degrees, between the lines) of the mean `i` and `j` edges of the cells of the columns `i_lo..i_hi` and rows `j_lo..j_hi`, over the fluid cells.
fn min_ij_angle(g: &Grid3, i_lo: usize, i_hi: usize, j_lo: usize, j_hi: usize) -> (f64, usize, usize, usize) {
    let node = |i: usize, j: usize, k: usize| {
        let n = (k * (g.nj + 1) + j) * (g.ni + 1) + i;
        [g.x[n], g.y[n], g.z[n]]
    };
    let mut worst = (90.0f64, 0, 0, 0);
    for k in 0..g.nk {
        for j in j_lo..j_hi {
            for i in i_lo..i_hi {
                if g.part[g.cell(i, j, k)] != bowshock_cfd::grid3::PART_FLUID {
                    continue;
                }
                let (mut ei, mut ej) = ([0.0; 3], [0.0; 3]);
                for a in 0..2 {
                    for b in 0..2 {
                        for q in 0..3 {
                            ei[q] += 0.25 * (node(i + 1, j + a, k + b)[q] - node(i, j + a, k + b)[q]);
                            ej[q] += 0.25 * (node(i + b, j + 1, k + a)[q] - node(i + b, j, k + a)[q]);
                        }
                    }
                }
                let dot = ei.iter().zip(&ej).map(|(a, b)| a * b).sum::<f64>();
                let norm = |v: [f64; 3]| v.iter().map(|a| a * a).sum::<f64>().sqrt();
                let ang = (dot / (norm(ei) * norm(ej))).abs().min(1.0).acos().to_degrees();
                if ang < worst.0 {
                    worst = (ang, i, j, k);
                }
            }
        }
    }
    worst
}

/// A fin whose tip trailing edge is behind the root's (Cancer Descending, DESIGN3D item 73) leans its trailing-edge columns forward above the tip while the rows beside the edge fall
/// to the nominal radius: with the fade of the leading-edge rules (4 + 4 columns) the cells above the tip were slivers of 6 degrees between the `i` and `j` edges (the app's Draft
/// and Coarse points of this rocket were unsteady, C_D 1.47 at Mach 0.5 against 0.55 with the tip edge flush). The fade now keeps the angle above `OVERHANG_EDGE_ANGLE_DEG`.
#[test]
fn an_overhanging_trailing_edge_has_no_sliver_cells() {
    let g = cancer_sector();
    // the fin's cells: the last column, the first and the last row
    let (mut i_te, mut j0, mut j1) = (0, usize::MAX, 0);
    for k in 0..g.nk {
        for j in 0..g.nj {
            for i in 0..g.ni {
                if matches!(g.part_of(i, j, k), Some(Part::Fin(_))) {
                    (i_te, j0, j1) = (i_te.max(i + 1), j0.min(j), j1.max(j + 1));
                }
            }
        }
    }
    assert!(i_te > 0, "no fin cells");
    let (angle, i, j, k) = min_ij_angle(&g, i_te, i_te + 14, j0, j1 + 16);
    println!("cancer: smallest i-j angle behind the trailing edge {angle:.1} degrees at ({i},{j},{k})");
    assert!(angle > 20.0, "a sliver of {angle:.1} degrees behind the overhanging trailing edge at ({i},{j},{k})");
}

#[test]
#[ignore = "prints the grid quality of the Cancer Descending sector"]
fn cancer_sector_quality() {
    use bowshock_core::engine::grid3::print_quality;
    let g = cancer_sector();
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
