//! Slender bodies of 40 to 100 nose diameters: a two-stage high-power rocket (38 mm sustainer on a 54 mm booster, 46 diameters) validates,
//! its grids keep the axial resolution of a shorter body, and (slow test) its CFD drag agrees with the same body cut at 40 diameters.

use bowshock_core::engine::case::Case;
use bowshock_core::{AftSpec, Analysis, Atmosphere, Backend, BodySpec, Error, FlightConditions, NoseShape, NoseSpec, Quality, Resolution, Wake};
use std::time::Instant;

/// Sustainer diameter, m.
const D: f64 = 0.0381;

/// Tangent ogive of fineness 5, a 38.1 mm tube to 19 d, a conical transition (1.3 d long, half-angle 9 deg) up to 54 mm, a 54 mm tube
/// to `total` nose diameters.
fn two_stage(total: f64) -> BodySpec {
    let (nose, tube, transition) = (5.0, 14.0, 1.3);
    BodySpec {
        nose: NoseSpec { shape: NoseShape::TangentOgive, length: nose * D, diameter: D },
        aft: vec![
            AftSpec::Tube { length: tube * D },
            AftSpec::Transition { length: transition * D, aft_diameter: 0.054 },
            AftSpec::Tube { length: (total - nose - tube - transition) * D },
        ],
    }
}

fn flight() -> FlightConditions {
    FlightConditions { atmosphere: Atmosphere::Isa { altitude_m: 0.0 } }
}

/// Coarsest axial cell, in nose diameters, of the body surface (nose end to base) of the grid of `case`.
fn coarsest_body_cell(case: &bowshock_core::engine::case::Case) -> f64 {
    let solver = case.try_build_solver().unwrap();
    let j = solver.block().map_or(0, |b| b.j_end);
    let ln = case.body.body.nose_length();
    let x_base = case.body.x_base();
    (0..solver.grid.ni)
        .map(|i| (solver.grid.node(i, j).0, solver.grid.node(i + 1, j).0))
        .filter(|&(a, b)| a >= ln - 1e-9 && b <= x_base + 1e-9)
        .map(|(a, b)| b - a)
        .fold(0.0, f64::max)
}

/// RANS analysis with the base wake (`rans`) or the Euler sting grid.
fn analysis(spec: &BodySpec, rans: bool, resolution: Resolution) -> Analysis {
    let a = Analysis::legacy(spec).unwrap().resolution(resolution);
    if rans {
        a.viscous(flight())
    } else {
        a.wake(Wake::Sting)
    }
}

/// The cases of `spec` at Mach 0.5.
fn cases(spec: &BodySpec, rans: bool, resolution: Resolution) -> Vec<Case> {
    analysis(spec, rans, resolution).engine_cases(&[0.5]).unwrap_or_else(|e| panic!("{resolution:?}: {e}"))
}

/// Coarsest cylinder cell of the longest body the solver took when the limit was 40 d: an ogive of fineness 1.5 and one 38.5 d tube with
/// the cell counts of that time (40 cells per tube at level 0, 36 for the last tube of the sting grid; the largest cell of the clustered
/// distribution is about 3 d, three times the mean, in the sting grid about 5 d, where the tube grows geometrically).
fn coarsest_cell_at_the_old_limit(rans: bool, resolution: Resolution) -> f64 {
    let spec = BodySpec { nose: NoseSpec { shape: NoseShape::TangentOgive, length: 1.5 * D, diameter: D }, aft: vec![AftSpec::Tube { length: 38.5 * D }] };
    let mut case = cases(&spec, rans, resolution).remove(0);
    if !rans {
        let f = match resolution {
            Resolution::Draft | Resolution::Coarse => 1,
            Resolution::Medium => 2,
            Resolution::Fine => 4,
        };
        case.grid_sting.n_seg = vec![36 * f];
    }
    coarsest_body_cell(&case)
}

/// The 46 d rocket passes the validation and both of its grids (base wake with RANS, sting without) pass the stretching check, and no
/// body cell is coarser than the coarsest one of a 40 d body at the same level.
#[test]
fn a_46_diameter_two_stage_rocket_validates_and_keeps_the_axial_resolution() {
    for rans in [true, false] {
        for resolution in [Resolution::Coarse, Resolution::Medium] {
            let c = cases(&two_stage(46.0), rans, resolution);
            let (cell, old) = (coarsest_body_cell(&c[0]), coarsest_cell_at_the_old_limit(rans, resolution));
            println!("rans {rans}, {resolution:?}: cells {:?}, coarsest body cell {cell:.3} d, at the old 40 d limit {old:.3} d", c[0].cells());
            assert!(cell <= old, "rans {rans}, {resolution:?}: coarsest body cell {cell:.3} d > {old:.3} d of a 40 d body");
        }
    }
}

/// A body past 40 d takes more streamwise cells (the cell count follows the length), a body of up to 40 d keeps its count, no body cell is
/// coarser than at the old limit, and every length up to the limit builds at every level.
#[test]
fn the_streamwise_cell_count_follows_the_length_beyond_40_diameters() {
    let old = coarsest_cell_at_the_old_limit(true, Resolution::Coarse);
    let ni_dx = |total: f64| {
        let c = cases(&two_stage(total), true, Resolution::Coarse);
        (c[0].cells().0, coarsest_body_cell(&c[0]))
    };
    let ((ni40, dx40), (ni46, dx46), (ni80, dx80), (ni100, dx100)) = (ni_dx(40.0), ni_dx(46.0), ni_dx(80.0), ni_dx(100.0));
    println!("ni 40 d {ni40}, 46 d {ni46}, 80 d {ni80}, 100 d {ni100}; coarsest cells {dx40:.2} {dx46:.2} {dx80:.2} {dx100:.2}, old limit {old:.2}");
    // the 19.7 d and 25.7 d tubes keep their 40 cells of level 0; the last tube of the 80 d and 100 d rockets (59.7 d, 79.7 d) takes 60 and 80
    assert_eq!(ni40, ni46);
    assert_eq!(ni80, ni40 + 20);
    assert_eq!(ni100, ni40 + 40);
    assert!(dx40 < dx46 && dx46 < dx80, "{dx40} {dx46} {dx80}");
    assert!(dx80 <= old * 1.05 && dx100 <= old * 1.05, "{dx80} {dx100} vs {old}");
    // the Euler sting grid (its last tube grows geometrically from the transition) too
    let old_sting = coarsest_cell_at_the_old_limit(false, Resolution::Coarse);
    for total in [46.0, 80.0, 100.0] {
        let dx = coarsest_body_cell(&cases(&two_stage(total), false, Resolution::Coarse)[0]);
        println!("sting {total} d: coarsest cell {dx:.2}, old limit {old_sting:.2}");
        assert!(dx <= old_sting * 1.05, "sting {total} d: {dx} vs {old_sting}");
    }
    for resolution in [Resolution::Coarse, Resolution::Medium, Resolution::Fine] {
        for total in [40.0, 46.0, 80.0, 100.0] {
            cases(&two_stage(total), true, resolution);
            cases(&two_stage(total), false, resolution);
        }
    }
}

/// The limit is explicit and the message names the field and the length.
#[test]
fn a_body_past_the_length_limit_is_refused_with_the_length_in_the_message() {
    let Err(Error::InvalidInput(msg)) = Analysis::legacy(&two_stage(101.0)).map(|_| ()) else { panic!("101 d accepted") };
    assert_eq!(msg, "aft: total body length 101.0 nose diameters exceeds 100");
    assert!(Analysis::legacy(&two_stage(100.0)).is_ok());
}

/// The 46 d rocket and the same rocket cut at 40 d at Mach 0.5 and 2 (RANS, base wake, coarse grid, CPU): both converge, the forebody
/// pressure drag (nose, transition) is the same to 1 %, and the extra 6 d of 54 mm tube add friction only, the total changes by the friction
/// change within 0.02 (the base drag shifts by about 0.005 of the 0.2 to 0.3 the base contributes). Friction stays within 15 % of the
/// flat-plate estimate over the wetted area, the same band as the G3 gate. About 100 s per Mach number.
#[test]
#[ignore = "slow: two RANS solves per Mach number, about 100 s"]
fn a_46_diameter_body_has_the_forebody_drag_of_the_40_diameter_one_plus_friction() {
    let air = Atmosphere::Isa { altitude_m: 0.0 }.state().unwrap();
    let re_c = air.density * air.sound_speed * D / air.viscosity;
    for mach in [0.5, 2.0] {
        let solve = |total: f64| {
            let t0 = Instant::now();
            let polar = Analysis::legacy(&two_stage(total))
                .unwrap()
                .backend(Backend::Cpu)
                .resolution(Resolution::Coarse)
                .reference_diameter(D)
                .viscous(flight())
                .run(&[mach])
                .unwrap();
            let p = polar.points.into_iter().next().unwrap();
            println!(
                "{total} d, M {mach}: {} after {} iterations ({:.0} s), cd {:.5} = pressure {:.5} + friction {:.5} + base {:.5}",
                p.detail.stop_reason,
                p.detail.iterations,
                t0.elapsed().as_secs_f64(),
                p.drag.total,
                p.drag.forebody_pressure,
                p.drag.friction.unwrap(),
                p.drag.base
            );
            p
        };
        let (short, long) = (solve(40.0), solve(46.0));
        for p in [&short, &long] {
            assert!(matches!(p.quality, Quality::Converged | Quality::Stationary), "M {mach}: {:?}, {}", p.quality, p.detail.stop_reason);
            assert!(p.warnings.is_empty(), "M {mach}: {:?}", p.warnings);
            assert!(p.detail.y_plus_max < 3.0, "M {mach}: y+ {}", p.detail.y_plus_max);
        }
        let (p40, p46) = (short.drag.forebody_pressure, long.drag.forebody_pressure);
        assert!((p46 / p40 - 1.0).abs() < 0.01, "M {mach}: forebody pressure {p40} (40 d) vs {p46} (46 d)");
        let (f40, f46) = (short.drag.friction.unwrap(), long.drag.friction.unwrap());
        assert!(f46 > f40);
        assert!(
            ((long.drag.total - short.drag.total) - (f46 - f40)).abs() < 0.02,
            "M {mach}: total {} -> {}, friction {f40} -> {f46}",
            short.drag.total,
            long.drag.total
        );
        let cf = 0.455 / (re_c * mach * 46.0).log10().powf(2.58) / (1.0 + 0.144 * mach * mach).powf(0.65);
        let wetted: f64 = long.detail.wall.windows(2).map(|w| 2.0 * 0.5 * (w[0].r + w[1].r) * (w[1].x - w[0].x).hypot(w[1].r - w[0].r) / (0.25 * D * D)).sum();
        assert!((f46 / (cf * wetted) - 1.0).abs() < 0.15, "M {mach}: friction {f46} vs flat plate {}", cf * wetted);
    }
}
