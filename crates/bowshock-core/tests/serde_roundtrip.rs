//! With the `serde` feature, inputs and results round-trip through JSON.
#![cfg(feature = "serde")]

use bowshock_core::{
    AftSpec, Analysis, Atmosphere, Backend, BodySpec, Condition, FinProfile, FinSpec, FlightConditions, MotorSpec, NoseShape, NoseSpec, Polar, Resolution,
};

fn spec() -> BodySpec {
    BodySpec {
        nose: NoseSpec { shape: NoseShape::TangentOgive, length: 0.30, diameter: 0.10 },
        aft: vec![AftSpec::Tube { length: 0.60 }, AftSpec::Transition { length: 0.10, aft_diameter: 0.08 }],
    }
}

fn roundtrip<T: serde::Serialize + serde::de::DeserializeOwned>(x: &T) -> T {
    serde_json::from_str(&serde_json::to_string(x).expect("serialize")).expect("deserialize")
}

#[test]
fn inputs_round_trip() {
    let s = spec();
    assert_eq!(roundtrip(&s), s);
    let f = FlightConditions { atmosphere: Atmosphere::Explicit { temperature_k: 250.0, pressure_pa: 80_000.0 } };
    assert_eq!(roundtrip(&f), f);
    let fins = FinSpec {
        count: 4,
        root_chord: 0.12,
        tip_chord: 0.06,
        span: 0.08,
        sweep: 0.06,
        thickness: 0.003,
        roughness: 0.0,
        root_te_to_base: 0.0,
        profile: FinProfile::DoubleWedge,
    };
    assert_eq!(roundtrip(&fins), fins);
    let custom = FinSpec { profile: FinProfile::Custom { section: vec![[0.0, 0.0], [0.1, 0.8], [1.0, 0.05]] }, ..fins.clone() };
    assert_eq!(roundtrip(&custom), custom);
    let motor = MotorSpec { exit_diameter: 0.06, exit_mach: 2.5, exit_pressure_pa: 101_325.0, total_temperature_k: 1500.0 };
    assert_eq!(roundtrip(&motor), motor);
}

#[test]
fn viscous_polar_round_trips_equal() {
    let polar = Analysis::legacy(&spec())
        .unwrap()
        .backend(Backend::Cpu)
        .resolution(Resolution::Coarse)
        .viscous(FlightConditions { atmosphere: Atmosphere::Isa { altitude_m: 0.0 } })
        .max_iterations(20)
        .run(&[0.5])
        .unwrap();
    assert_eq!(roundtrip(&polar), polar);
}

/// JSON has no NaN: serde_json writes it as `null`, and the fields documented as NaN read it back as NaN.
#[test]
fn nan_fields_of_an_euler_point_come_back_as_nan() {
    let polar = Analysis::legacy(&spec()).unwrap().backend(Backend::Cpu).resolution(Resolution::Coarse).max_iterations(20).run(&[0.5]).unwrap();
    let back: Polar = roundtrip(&polar);
    let (a, b) = (&polar.points[0], &back.points[0]);
    for (x, y) in [(a.detail.cd_base_solver, b.detail.cd_base_solver), (a.detail.cp_base, b.detail.cp_base), (a.detail.y_plus_max, b.detail.y_plus_max)] {
        assert!(x.is_nan() && y.is_nan(), "{x} -> {y}");
    }
    assert_eq!(a.drag, b.drag);
    assert_eq!(a.detail.wall, b.detail.wall);
}

fn full_analysis() -> Analysis {
    let fins = FinSpec {
        count: 4,
        root_chord: 0.12,
        tip_chord: 0.06,
        span: 0.08,
        sweep: 0.06,
        thickness: 0.003,
        roughness: 0.0,
        root_te_to_base: 0.0,
        profile: FinProfile::Rounded,
    };
    let motor = MotorSpec { exit_diameter: 0.05, exit_mach: 2.5, exit_pressure_pa: 101_325.0, total_temperature_k: 1500.0 };
    Analysis::legacy(&spec())
        .unwrap()
        .reference_diameter(0.09)
        .viscous(FlightConditions { atmosphere: Atmosphere::Isa { altitude_m: 1000.0 } })
        .fins(fins)
        .motor(motor)
        .roughness(1e-5)
        .backend(Backend::Cpu)
        .resolution(Resolution::Coarse)
        .max_iterations(25)
        .drag_tolerance(0.002)
        .warm_start(true)
        .estimate_uncertainty(true)
        .grid_mach(3.0)
}

#[test]
fn an_analysis_spec_round_trips_through_json_and_rebuilds_the_analysis() {
    let s = full_analysis().spec().unwrap();
    let back: bowshock_core::AnalysisSpec = roundtrip(&s);
    assert_eq!(back, s);
    assert_eq!(Analysis::from_spec(&back).unwrap().spec().unwrap(), s);
    assert!(Analysis::from_internal(bowshock_core::engine::body_grid::BaseBody::cylinder(
        bowshock_core::engine::body_grid::Body { nose: bowshock_core::engine::body_grid::Nose::Cone { half_angle: 0.2 } },
        3.0
    ))
    .spec()
    .is_err());
}

/// The engine is additive on the wire: a spec written before it (no `engine` key) is the default engine's (the 3D one), and the 3D engine survives the round trip.
#[test]
fn the_engine_of_a_spec_defaults_to_the_default_engine_and_round_trips() {
    let s = full_analysis().engine(bowshock_core::Engine::Solver3).spec().unwrap();
    let back: bowshock_core::AnalysisSpec = roundtrip(&s);
    assert_eq!(back.engine, bowshock_core::Engine::Solver3);
    let mut v = serde_json::to_value(&s).unwrap();
    v.as_object_mut().unwrap().remove("engine");
    let old: bowshock_core::AnalysisSpec = serde_json::from_value(v).unwrap();
    assert_eq!(old.engine, bowshock_core::Engine::DEFAULT);
    assert_eq!(old.engine, bowshock_core::Engine::Solver3);
}

#[test]
fn from_spec_of_spec_gives_the_same_run() {
    let a = Analysis::legacy(&spec()).unwrap().backend(Backend::Cpu).resolution(Resolution::Coarse).max_iterations(40);
    let b = Analysis::from_spec(&a.spec().unwrap()).unwrap();
    let (pa, pb) = (a.run(&[0.8, 2.0]).unwrap(), b.run(&[0.8, 2.0]).unwrap());
    // JSON writes the NaN of a field without a base wake as null on both sides
    // (the residual is a parallel sum: its last bits are not reproducible, so it is compared to nine digits)
    let json = |p: &bowshock_core::Polar| {
        let mut v = serde_json::to_value(p).unwrap();
        for pt in v["points"].as_array_mut().unwrap() {
            let d = &mut pt["detail"]["residual_drop_orders"];
            *d = serde_json::json!((d.as_f64().unwrap() * 1e9).round() / 1e9);
        }
        v.to_string()
    };
    assert_eq!(json(&pa), json(&pb));
}

/// A finned point at an angle of attack, with its field: the NaN of what the 3D Euler solver does not give (the base and the total
/// drag, the nose pressure drag, the base pressure and `y+`) comes back as NaN, everything else equal; and the condition itself.
#[test]
fn a_finned_angle_of_attack_point_round_trips_with_its_nans_and_its_field() {
    let d = 0.04;
    let nose_len = 0.5 * d / 10f64.to_radians().tan();
    let body = BodySpec { nose: NoseSpec { shape: NoseShape::Cone, length: nose_len, diameter: d }, aft: vec![AftSpec::Tube { length: 10.0 * d - nose_len }] };
    let fins = FinSpec {
        count: 4,
        root_chord: d,
        tip_chord: d,
        span: d,
        sweep: 0.0,
        thickness: 0.08 * d,
        roughness: 0.0,
        root_te_to_base: 0.0,
        profile: FinProfile::Square,
    };
    let polar = Analysis::legacy(&body)
        .unwrap()
        .fins(fins)
        .backend(Backend::Cpu)
        .resolution(Resolution::Coarse)
        .case3_hook(|c| {
            let s = &mut c.grid.sting;
            s.n_up = 10;
            s.n_nose = 16;
            s.n_seg = vec![12];
            s.n_ext = 10;
            s.nj = 24;
            c.grid.nk = 8;
        })
        .max_iterations(50)
        .keep_fields(true)
        .run(&[Condition::new(2.0, 4.0)])
        .unwrap();
    let back: Polar = roundtrip(&polar);
    let (a, b) = (&polar.points[0], &back.points[0]);
    assert_eq!((a.alpha_deg, a.forces, &a.field3), (b.alpha_deg, b.forces, &b.field3));
    assert!(a.forces.is_some() && a.field3.is_some());
    for (x, y) in [(a.drag.base, b.drag.base), (a.drag.total, b.drag.total), (a.detail.cd_nose, b.detail.cd_nose), (a.detail.cp_base, b.detail.cp_base)] {
        assert!(x.is_nan() && y.is_nan(), "{x} -> {y}");
    }
    assert_eq!(
        (a.drag.forebody_pressure, a.drag.friction, a.quality, &a.detail.stop_reason),
        (b.drag.forebody_pressure, b.drag.friction, b.quality, &b.detail.stop_reason)
    );
    // a point at zero incidence has none of it
    assert!(serde_json::to_string(
        &Analysis::legacy(&spec()).unwrap().backend(Backend::Cpu).resolution(Resolution::Coarse).max_iterations(20).run(&[0.5]).unwrap()
    )
    .unwrap()
    .find("forces")
    .is_none());
    let c = Condition::new(2.0, 4.0);
    assert_eq!(roundtrip(&c), c);
    assert_eq!(serde_json::to_string(&c).unwrap(), r#"{"mach":2.0,"alpha_deg":4.0}"#);
}
