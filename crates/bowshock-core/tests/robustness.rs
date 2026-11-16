//! The public API never panics: awkward but valid bodies return `Ok` with every point finite or `Failed`.

use bowshock_core::{
    AftSpec, Analysis, Atmosphere, Backend, BodySpec, Error, FinProfile, FinSpec, FlightConditions, MotorSpec, NoseShape, NoseSpec, Quality, Resolution,
};

const D: f64 = 0.1;

fn spec(shape: NoseShape, nose_calibers: f64, aft: Vec<AftSpec>) -> BodySpec {
    BodySpec { nose: NoseSpec { shape, length: nose_calibers * D, diameter: D }, aft }
}

fn tube(calibers: f64) -> AftSpec {
    AftSpec::Tube { length: calibers * D }
}

/// Transition of half-angle just under 30 degrees between diameters `from` and `to` (in calibers).
fn transition(from: f64, to: f64) -> AftSpec {
    let length = 0.5 * (to - from).abs() * D / 29.9f64.to_radians().tan();
    AftSpec::Transition { length, aft_diameter: to * D }
}

#[test]
fn awkward_bodies_return_ok_with_finite_or_failed_points() {
    let machs = [0.1, 0.95, 3.0];
    let bodies = [
        ("short ogive 0.5 d", spec(NoseShape::TangentOgive, 0.5, vec![tube(1.0)])),
        ("long Haack 12 d", spec(NoseShape::Haack { c: 1.0 / 3.0 }, 12.0, vec![tube(1.0)])),
        ("power 0.5 with 0.01 d tube", spec(NoseShape::Power { n: 0.5 }, 3.0, vec![tube(0.01)])),
        ("cone 29.9 deg", spec(NoseShape::Cone, 0.87, vec![tube(1.0)])),
        ("flare and boattail 30 deg", spec(NoseShape::TangentOgive, 3.0, vec![tube(2.0), transition(1.0, 1.5), tube(1.0), transition(1.5, 1.0)])),
        ("40 d long", spec(NoseShape::TangentOgive, 3.0, vec![tube(37.0)])),
    ];
    for (name, body) in &bodies {
        let result = Analysis::new(body)
            .unwrap_or_else(|e| panic!("{name}: Analysis::new failed: {e}"))
            .backend(Backend::Cpu)
            .resolution(Resolution::Coarse)
            .max_iterations(30)
            .run(&machs);
        // A 0.01 d tube cannot be gridded within the stretching limit: the grid check rejects it with an error instead of a panic.
        let polar = match result {
            Err(Error::Grid(why)) if name.contains("0.01 d tube") => {
                println!("{name}: rejected by the grid check: {why}");
                continue;
            }
            other => other.unwrap_or_else(|e| panic!("{name}: run failed: {e}")),
        };
        assert_eq!(polar.points.len(), machs.len(), "{name}");
        for p in &polar.points {
            println!("{name}: M = {:.2}, quality = {:?}, total = {:.5}", p.mach, p.quality, p.drag.total);
            assert!(p.quality == Quality::Failed || p.drag.total.is_finite(), "{name} M = {}: {:?} with total {}", p.mach, p.quality, p.drag.total);
        }
    }
}

/// The ogive body with fins in a viscous analysis, power-off and with a motor: `Ok`, every point finite or `Failed`.
#[test]
fn fins_and_motor_return_ok_with_finite_or_failed_points() {
    let body = spec(NoseShape::TangentOgive, 3.0, vec![tube(5.0)]);
    let fins = FinSpec { count: 4, root_chord: 0.12, tip_chord: 0.06, span: 0.08, sweep: 0.06, thickness: 0.003, roughness: 0.0, profile: FinProfile::Rounded };
    let motor = MotorSpec { exit_diameter: 0.06, exit_mach: 2.5, exit_pressure_pa: 101_325.0, total_temperature_k: 1500.0 };
    let analysis = || {
        Analysis::new(&body)
            .unwrap()
            .fins(fins.clone())
            .viscous(FlightConditions { atmosphere: Atmosphere::Isa { altitude_m: 0.0 } })
            .backend(Backend::Cpu)
            .resolution(Resolution::Coarse)
            .max_iterations(30)
    };
    for (name, a) in [("fins, viscous", analysis()), ("fins, viscous, motor", analysis().motor(motor))] {
        let polar = a.run(&[0.5, 2.0]).unwrap_or_else(|e| panic!("{name}: run failed: {e}"));
        assert_eq!(polar.points.len(), 2, "{name}");
        for p in &polar.points {
            println!("{name}: M = {:.2}, quality = {:?}, total = {:.5}", p.mach, p.quality, p.drag.total);
            assert!(p.quality == Quality::Failed || p.drag.total.is_finite(), "{name} M = {}: {:?} with total {}", p.mach, p.quality, p.drag.total);
        }
    }
}
