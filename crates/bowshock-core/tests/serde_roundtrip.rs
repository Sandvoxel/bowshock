//! With the `serde` feature, inputs and results round-trip through JSON.
#![cfg(feature = "serde")]

use bowshock_core::{
    AftSpec, Analysis, Atmosphere, Backend, BodySpec, FinProfile, FinSpec, FlightConditions, MotorSpec, NoseShape, NoseSpec, Polar, Resolution,
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
    let fins =
        FinSpec { count: 4, root_chord: 0.12, tip_chord: 0.06, span: 0.08, sweep: 0.06, thickness: 0.003, roughness: 0.0, profile: FinProfile::DoubleWedge };
    assert_eq!(roundtrip(&fins), fins);
    let custom = FinSpec { profile: FinProfile::Custom { section: vec![[0.0, 0.0], [0.1, 0.8], [1.0, 0.05]] }, ..fins.clone() };
    assert_eq!(roundtrip(&custom), custom);
    let motor = MotorSpec { exit_diameter: 0.06, exit_mach: 2.5, exit_pressure_pa: 101_325.0, total_temperature_k: 1500.0 };
    assert_eq!(roundtrip(&motor), motor);
}

#[test]
fn viscous_polar_round_trips_equal() {
    let polar = Analysis::new(&spec())
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
    let polar = Analysis::new(&spec()).unwrap().backend(Backend::Cpu).resolution(Resolution::Coarse).max_iterations(20).run(&[0.5]).unwrap();
    let back: Polar = roundtrip(&polar);
    let (a, b) = (&polar.points[0], &back.points[0]);
    for (x, y) in [(a.detail.cd_base_solver, b.detail.cd_base_solver), (a.detail.cp_base, b.detail.cp_base), (a.detail.y_plus_max, b.detail.y_plus_max)] {
        assert!(x.is_nan() && y.is_nan(), "{x} -> {y}");
    }
    assert_eq!(a.drag, b.drag);
    assert_eq!(a.detail.wall, b.detail.wall);
}
