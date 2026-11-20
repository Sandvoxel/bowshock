use bowshock_model::CfdGrid;
use bowshock_sim::BodyKind;

use super::*;
use crate::units::UnitSystem;

#[test]
fn lift_nodes_are_the_interior_nodes_inside_the_range() {
    assert_eq!(lift_nodes((0.1, 3.0)).unwrap(), LIFT_INTERIOR.to_vec());
    assert_eq!(lift_nodes((0.8, 1.2)).unwrap(), vec![0.8, 0.95, 1.1]);
    assert_eq!(lift_nodes((3.0, 0.8)).unwrap().last(), Some(&3.0));
    assert_eq!(lift_nodes((0.1, 0.3)).unwrap(), vec![0.5]);
    assert_eq!(lift_nodes((0.5, 0.5)).unwrap(), vec![0.5]);
    assert_eq!(lift_nodes((3.2, 4.0)), None);
    // between two nodes: the nearest one stands in
    assert_eq!(lift_nodes((1.15, 1.2)).unwrap(), vec![1.1]);
}

#[test]
fn the_lift_key_ignores_the_altitude_and_follows_geometry_and_nodes() {
    let (rocket, vehicle) = example();
    let plan = plan(&rocket, &vehicle, UnitSystem::default()).unwrap();
    let nodes = lift_nodes((0.1, 3.0)).unwrap();
    assert_ne!(table_key(plan.geometry, 0.0, &nodes), table_key(plan.geometry, 3000.0, &nodes));
    let key = lift_key(plan.geometry, &nodes);
    let fresh = lift_table(key);
    let ok: Result<Plan, String> = Ok(plan.clone());
    assert_eq!(lift_status(&ok, Some(&fresh), (0.1, 3.0)), Status::Fresh);
    assert_eq!(lift_status(&ok, None, (0.1, 3.0)), Status::Missing);
    assert_eq!(lift_status(&ok, Some(&fresh), (0.8, 1.2)), Status::Stale, "other nodes");
    assert_eq!(lift_status(&ok, Some(&lift_table(key ^ 1)), (0.1, 3.0)), Status::Stale);
    assert!(matches!(lift_status(&ok, Some(&fresh), (3.5, 4.0)), Status::Unsupported(_)));
    assert_ne!(key, lift_key(plan.geometry ^ 1, &nodes));
}

#[test]
fn the_3d_solve_is_kept_across_a_drag_re_solve() {
    let mut rocket = Rocket::default();
    rocket.store_cfd_table(CfdTable { lift: Some(lift_table(5)), ..table() });
    rocket.store_cfd_table(CfdTable { key: table().key + 1, ..table() });
    assert_eq!(rocket.cfd_table(table().stage).unwrap().lift.as_ref().map(|l| l.key), Some(5));
}

#[test]
fn fin_position_is_the_root_trailing_edge_distance_to_the_base() {
    let (_, vehicle) = example();
    let x_base = vehicle.body.last().unwrap().aft_station();
    let mut set = vehicle.fin_sets[0].clone();
    set.station = x_base - set.root_chord - 0.05;
    assert!((fin_spec(&set, x_base).unwrap().root_te_to_base - 0.05).abs() < 1e-12);
    set.station = x_base - set.root_chord;
    assert_eq!(fin_spec(&set, x_base).unwrap().root_te_to_base, 0.0);
    set.station = x_base - set.root_chord + 0.01; // overhanging the base clamps to flush
    assert_eq!(fin_spec(&set, x_base).unwrap().root_te_to_base, 0.0);
}

#[test]
fn the_example_rocket_maps_to_a_body_and_four_fins() {
    let (rocket, vehicle) = example();
    let plan = plan(&rocket, &vehicle, UnitSystem::default()).expect("the example is an axisymmetric stack");
    let d = vehicle.reference_diameter;
    assert!((plan.body.reference_diameter() - d).abs() < 1e-9, "reference diameter {d}");
    let len = plan.body.nose.length
        + plan
            .body
            .aft
            .iter()
            .map(|a| match a {
                solver::AftSpec::Tube { length } | solver::AftSpec::Transition { length, .. } => *length,
                solver::AftSpec::Step { .. } => 0.0,
            })
            .sum::<f64>();
    assert!((len - vehicle.length()).abs() < 1e-9, "the body is as long as the rocket: {len} vs {}", vehicle.length());
    let fins = plan.fins.as_ref().expect("the example has fins");
    let set = &vehicle.fin_sets[0];
    assert_eq!((fins.count, fins.root_chord, fins.span), (set.count, set.root_chord, set.span));
    fins.validate().unwrap();
}

#[test]
fn the_wall_roughness_is_the_wetted_area_mean_of_the_finishes() {
    // independent areas: fine trapezoid integration of the profile
    let area = |s: &bowshock_sim::BodySegment| {
        let n = 4000;
        (0..n)
            .map(|i| {
                let (x0, x1) = (s.length * i as f64 / n as f64, s.length * (i + 1) as f64 / n as f64);
                let (r0, r1) = (s.radius_at(x0), s.radius_at(x1));
                std::f64::consts::PI * (r0 + r1) * (r1 - r0).hypot(x1 - x0)
            })
            .sum::<f64>()
    };
    let (rocket, mut vehicle) = example();
    let all = plan(&rocket, &vehicle, UnitSystem::default()).unwrap();
    let expected = {
        let (sum, a) = vehicle.body.iter().fold((0.0, 0.0), |(s, a), seg| (s + seg.roughness * area(seg), a + area(seg)));
        sum / a
    };
    assert!(expected > 0.0);
    assert!((all.body_roughness - expected).abs() < 1e-3 * expected, "{} vs {expected}", all.body_roughness);
    let um = format!("{:.0} um", expected * 1e6);
    assert!(all.notes.iter().any(|n| n.contains(&um) && n.contains("roughness")), "{:?}", all.notes);
    assert!(!all.notes.iter().any(|n| n.contains("solved smooth;")), "{:?}", all.notes);
    // a different finish on part of the body moves the mean and the geometry hash
    vehicle.body[0].roughness = 0.0;
    for s in &mut vehicle.body[1..] {
        s.roughness = 100e-6;
    }
    let split = plan(&rocket, &vehicle, UnitSystem::default()).unwrap();
    let (nose, rest) = (area(&vehicle.body[0]), vehicle.body[1..].iter().map(area).sum::<f64>());
    let expected = 100e-6 * rest / (nose + rest);
    assert!((split.body_roughness - expected).abs() < 1e-3 * expected, "{} vs {expected}", split.body_roughness);
    assert_ne!(split.geometry, all.geometry);
    // fins carry their own finish
    assert_eq!(all.fins.as_ref().unwrap().roughness, vehicle.fin_sets[0].roughness);
    // a polished body is solved smooth and says so
    for s in &mut vehicle.body {
        s.roughness = 0.0;
    }
    let smooth = plan(&rocket, &vehicle, UnitSystem::default()).unwrap();
    assert_eq!(smooth.body_roughness, 0.0);
    assert!(smooth.notes.iter().any(|n| n.contains("solved smooth")), "{:?}", smooth.notes);
}

#[test]
fn what_is_not_solved_is_said() {
    let (rocket, mut vehicle) = example();
    vehicle.protuberances.push(bowshock_sim::Protuberance::RailButton { station: 0.5, height: 0.01, diameter: 0.01 });
    let mapped = plan(&rocket, &vehicle, UnitSystem::default()).unwrap();
    assert!(mapped.notes.iter().any(|n| n.contains("rail buttons")), "{:?}", mapped.notes);
    // A second fin set is left to the build-up and said so.
    let (rocket, mut vehicle) = example();
    let again = vehicle.fin_sets[0].clone();
    vehicle.fin_sets.push(again);
    let mapped = plan(&rocket, &vehicle, UnitSystem::default()).unwrap();
    assert!(mapped.notes.iter().any(|n| n.contains("fin set")), "{:?}", mapped.notes);
    assert_eq!(mapped.rest.fin_sets.len(), 1);
}

/// The outline the 3D engine meshes comes from the document: the half ellipse of an elliptical fin set, the polygon of a freeform one, the trapezoid otherwise; the legacy
/// engine (the test's default) says it flies the equivalent trapezoid and keeps the trapezoid outline.
#[test]
fn the_fins_outline_follows_the_document_for_the_3d_engine() {
    let (rocket, mut vehicle) = example();
    assert_eq!(fin_outline(&rocket, &vehicle.fin_sets[0]), solver::FinOutline::Trapezoid);
    vehicle.fin_sets[0].planform = bowshock_sim::FinPlanform::Elliptical;
    assert_eq!(fin_outline(&rocket, &vehicle.fin_sets[0]), solver::FinOutline::Ellipse);
    let mapped = plan(&rocket, &vehicle, UnitSystem::default()).unwrap();
    // the engine of the run (`BOWSHOCK_ENGINE`) decides: the 3D engine meshes the ellipse, the legacy one flies the equivalent trapezoid and says so
    let three_d = solver::Engine::from_env() == solver::Engine::Solver3;
    assert_eq!(mapped.outline, if three_d { solver::FinOutline::Ellipse } else { solver::FinOutline::Trapezoid });
    assert_eq!(mapped.notes.iter().any(|n| n.contains("Elliptical fins are solved as equivalent trapezoids")), !three_d, "{:?}", mapped.notes);

    // a freeform fin: the example's fin set replaced by a quadrilateral of the same span
    let (mut rocket, _) = example();
    let fins = rocket.radials().iter().find(|r| matches!(r.kind, bowshock_model::RadialKind::FinSet { .. })).cloned().unwrap();
    let bowshock_model::RadialKind::FinSet { span, thickness, .. } = fins.kind else { unreachable!() };
    rocket.remove_radial(fins.id).unwrap();
    let points = vec![(0.0, 0.0), (0.03, span), (0.06, span), (0.1, 0.0)];
    rocket
        .attach_radial(fins.parent, fins.offset, fins.angle, fins.count, bowshock_model::RadialKind::FreeformFin { points: points.clone(), thickness })
        .unwrap();
    let vehicle = from_model::vehicle(&rocket, &MotorDb::bundled(), Phase::Full).unwrap();
    assert!(vehicle.fin_sets[0].root_extent.is_some(), "a freeform fin carries its real root");
    assert_eq!(fin_outline(&rocket, &vehicle.fin_sets[0]), solver::FinOutline::Polygon(points));
    let mapped = plan(&rocket, &vehicle, UnitSystem::default()).unwrap();
    assert_eq!(mapped.notes.iter().any(|n| n.contains("Freeform fins are solved as equivalent trapezoids")), !three_d, "{:?}", mapped.notes);
    if three_d {
        assert_eq!(mapped.outline, solver::FinOutline::Polygon(vec![(0.0, 0.0), (0.03, span), (0.06, span), (0.1, 0.0)]));
    }
}

#[test]
fn geometry_the_solver_cannot_take_is_refused_with_a_reason() {
    let (rocket, mut vehicle) = example();
    // A step in diameter: the legacy engine's body is continuous (the 3D engine's steps are in `a_step_in_diameter_is_solved_by_the_3d_engine`).
    if !three_d() {
        vehicle.body[1].fore_radius += 0.01;
        let why = plan(&rocket, &vehicle, UnitSystem::default()).unwrap_err();
        assert!(why.contains("step in diameter"), "{why}");
    }
    // A pod: a zero-size separator ahead of the core.
    let (rocket, mut vehicle) = example();
    vehicle.body.insert(0, separator());
    let why = plan(&rocket, &vehicle, UnitSystem::default()).unwrap_err();
    assert!(why.contains("pods"), "{why}");
    // No nose.
    let (rocket, mut vehicle) = example();
    vehicle.body[0].kind = BodyKind::Tube;
    assert!(plan(&rocket, &vehicle, UnitSystem::default()).unwrap_err().contains("nose"));
    // Tube fins (open pod tubes) are bodies beside the axis too.
    let (rocket, mut vehicle) = example();
    vehicle.tube_fins.push(bowshock_sim::TubeFinSet {
        count: 2,
        station: 0.5,
        length: 0.2,
        outer_radius: 0.02,
        inner_radius: 0.019,
        body_radius: 0.038,
        gap: 0.0,
        roughness: 0.0,
    });
    assert!(plan(&rocket, &vehicle, UnitSystem::default()).unwrap_err().contains("tube fins"));
}

fn separator() -> bowshock_sim::BodySegment {
    bowshock_sim::BodySegment { station: 0.0, length: 0.0, fore_radius: 0.0, aft_radius: 0.0, kind: BodyKind::Tube, tip_radius: 0.0, roughness: 0.0 }
}

#[test]
fn keys_are_stable_and_follow_geometry_and_conditions() {
    let (rocket, vehicle) = example();
    let a = plan(&rocket, &vehicle, UnitSystem::default()).unwrap();
    let b = plan(&rocket, &vehicle, UnitSystem::default()).unwrap();
    assert_eq!(a.geometry, b.geometry);
    // The hash must not change from run to run, platform to platform or toolchain to toolchain, or every stored table would
    // read as stale: FNV-1a is pinned by its published test vector ("a" hashes to 0xaf63dc4c8601ec8c).
    assert_eq!(Fnv::new().bytes(b"a").0, 0xaf63_dc4c_8601_ec8c);
    assert_eq!(Fnv::new().bytes(b"").0, 0xcbf2_9ce4_8422_2325);
    let nodes = nodes((0.1, 3.0)).unwrap();
    let key = table_key(a.geometry, 120.0, &nodes);
    assert_eq!(key, table_key(b.geometry, 120.0, &nodes));
    assert_ne!(key, table_key(a.geometry, 500.0, &nodes), "altitude changes the Reynolds number");
    assert_ne!(key, table_key(a.geometry, 120.0, &self::nodes((0.5, 2.0)).unwrap()), "the Mach nodes follow the range");
    // A recomputed length that differs in the last bits is the same geometry; a millimetre is not.
    let mut v = vehicle.clone();
    v.body[1].length += 1e-12;
    assert_eq!(plan(&rocket, &v, UnitSystem::default()).unwrap().geometry, a.geometry);
    v.body[1].length += 1e-3;
    assert_ne!(plan(&rocket, &v, UnitSystem::default()).unwrap().geometry, a.geometry);
}

#[test]
fn nodes_cover_the_range_and_follow_it() {
    let all = nodes((0.1, 3.0)).unwrap();
    assert_eq!((all[0], *all.last().unwrap()), (0.1, 3.0));
    assert!(all.windows(2).all(|w| w[0] < w[1]) && all.len() <= 12, "{all:?}");
    assert!(all.contains(&1.0) && all.contains(&0.9) && all.contains(&1.1), "transonic detail: {all:?}");
    assert_eq!(nodes((3.0, 0.1)).unwrap(), all, "the range may be given either way round");
    let narrow = nodes((0.8, 1.2)).unwrap();
    assert_eq!((narrow[0], *narrow.last().unwrap()), (0.8, 1.2));
    assert!(narrow.iter().all(|m| (0.8..=1.2).contains(m)));
    assert_eq!(nodes((2.0, 6.0)).unwrap().last(), Some(&3.0), "clipped to the solver's range");
    assert_eq!(nodes((4.0, 6.0)), None);
}

#[test]
fn a_stored_table_is_fresh_until_the_geometry_or_the_conditions_change() {
    let (rocket, vehicle) = example();
    let mapped = plan(&rocket, &vehicle, UnitSystem::default());
    let full = (0.1, 3.0);
    let mut t = table();
    t.geometry = mapped.as_ref().unwrap().geometry;
    t.key = table_key(t.geometry, 120.0, &nodes(full).unwrap());
    assert_eq!(status(&mapped, None, 120.0, full), Status::Missing);
    assert_eq!(status(&mapped, Some(&t), 120.0, full), Status::Fresh);
    assert_eq!(status(&mapped, Some(&t), 500.0, full), Status::Stale, "the altitude sets the Reynolds number");
    assert_eq!(status(&mapped, Some(&t), 120.0, (0.5, 2.0)), Status::Stale, "the Mach range sets the nodes");
    // Editing the geometry makes the table stale: Re-solve.
    let mut longer = vehicle.clone();
    // A longer tube pushes everything behind it aft, so the stack stays contiguous.
    longer.body[1].length += 0.01;
    longer.body[2..].iter_mut().for_each(|s| s.station += 0.01);
    assert_eq!(status(&plan(&rocket, &longer, UnitSystem::default()), Some(&t), 120.0, full), Status::Stale);
    // Not solvable at all, and a Mach range the solver does not cover.
    assert!(matches!(status(&Err("pods".to_string()), Some(&t), 120.0, full), Status::Unsupported(_)));
    assert!(matches!(status(&mapped, None, 120.0, (4.0, 6.0)), Status::Unsupported(_)));
}

#[test]
fn a_medium_table_of_the_same_geometry_and_conditions_is_fresh() {
    let (rocket, vehicle) = example();
    let mapped = plan(&rocket, &vehicle, UnitSystem::default());
    let full = (0.1, 3.0);
    let mut t = table();
    t.geometry = mapped.as_ref().unwrap().geometry;
    t.key = table_key(t.geometry, 120.0, &nodes(full).unwrap());
    t.grid = CfdGrid::Medium;
    assert_eq!(status(&mapped, Some(&t), 120.0, full), Status::Fresh);
}

/// A blunted nose goes to the solver as the blunted profile: the solver's body is the same outline the model draws (the tangent
/// sphere on the sharp ogive, the length as built), and the geometry hash follows the tip.
#[test]
fn a_blunted_nose_is_solved_as_the_blunted_profile() {
    let (mut rocket, pointed_vehicle) = example();
    let pointed = plan(&rocket, &pointed_vehicle, UnitSystem::default()).unwrap();
    assert!(!matches!(pointed.body.nose.shape, solver::NoseShape::Blunted { .. }));
    let nose = rocket.parts()[0].id;
    assert!(rocket.set_dim(nose, bowshock_model::Dim::TipRadius, 0.01).is_some());
    let vehicle = from_model::vehicle(&rocket, &MotorDb::bundled(), Phase::Full).unwrap();
    let mapped = plan(&rocket, &vehicle, UnitSystem::default()).unwrap();
    let first = &vehicle.body[0];
    assert!(matches!(mapped.body.nose.shape, solver::NoseShape::Blunted { tip_radius, .. } if tip_radius == 0.01), "{:?}", mapped.body.nose.shape);
    assert_eq!(mapped.body.nose.length, first.length, "the nose length is the length as built");
    assert_eq!(mapped.body.nose.length, rocket.parts()[0].kind.length());
    assert_ne!(mapped.geometry, pointed.geometry);
    // the solver's wall is the model's outline
    let (internal, scale) = mapped.body.to_internal().unwrap();
    let kind = &rocket.parts()[0].kind;
    for i in 0..=40 {
        let z = first.length * f64::from(i) / 40.0;
        let solved = internal.body.r(z / scale.d_nose) * scale.d_nose;
        assert!((solved - kind.radius_at(z)).abs() < 1e-9, "z {z}: solver {solved} vs model {}", kind.radius_at(z));
        assert!((solved - first.radius_at(z)).abs() < 1e-9, "z {z}: solver {solved} vs sim {}", first.radius_at(z));
    }
}

/// The 3D engine meshes a diameter step as a chamfer of one cell (DESIGN3D item 29): a shoulder behind the nose and a booster stepping down map to
/// `AftSpec::Step`, keep the rocket's length and reference diameter; one too tall for the mesher, and a fin root across a step, are refused with a reason.
/// The legacy engine (the test's default) refuses every step.
#[test]
fn a_step_in_diameter_is_solved_by_the_3d_engine() {
    let (rocket, vehicle) = example();
    let n = vehicle.body.len();
    // the radius jumps by 4 mm at the start of the last body segment
    let mut stepped = vehicle.clone();
    stepped.body[n - 1].fore_radius += 0.004;
    stepped.body[n - 1].aft_radius = stepped.body[n - 1].fore_radius;
    if !three_d() {
        let why = plan(&rocket, &stepped, UnitSystem::default()).unwrap_err();
        assert!(why.contains("step in diameter") && why.contains("3D engine"), "{why}");
        return;
    }
    let mapped = plan(&rocket, &stepped, UnitSystem::default()).expect("the 3D engine takes a step");
    let steps = mapped.body.aft.iter().filter(|a| matches!(a, solver::AftSpec::Step { .. })).count();
    assert_eq!(steps, 1, "{:?}", mapped.body.aft);
    let len = mapped.body.nose.length
        + mapped
            .body
            .aft
            .iter()
            .map(|a| match a {
                solver::AftSpec::Tube { length } | solver::AftSpec::Transition { length, .. } => *length,
                solver::AftSpec::Step { .. } => 0.0,
            })
            .sum::<f64>();
    // the chamfer is taken from the section after it: the body has the rocket's length (to the chamfer, when it is the last)
    let to_internal = mapped.body.to_internal().unwrap().0;
    assert!((to_internal.x_base() * mapped.body.nose.diameter - stepped.length()).abs() < 1e-9, "{len}");
    assert!((mapped.body.reference_diameter() - 2.0 * stepped.body[n - 1].fore_radius.max(vehicle.body[1].fore_radius)).abs() < 1e-9);
    // a step taller than the mesher takes is the engine's error, naming the step
    let mut tall = vehicle.clone();
    tall.body[n - 1].fore_radius += 0.7 * vehicle.reference_diameter;
    tall.body[n - 1].aft_radius = tall.body[n - 1].fore_radius;
    let why = plan(&rocket, &tall, UnitSystem::default()).unwrap_err();
    assert!(why.contains("step of"), "{why}");
}
