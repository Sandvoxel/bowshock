//! Rigid-body flight: weathercocking, instability, damping, roll from fin cant and the return to a point mass under a parachute.

use bowshock_sim::*;

/// Approximate Aerotech H128W-class motor: 176 N s, 1.45 s burn, 0.105 kg of propellant.
fn h128() -> ThrustCurve {
    ThrustCurve {
        points: vec![(0.0, 0.0), (0.04, 190.0), (0.1, 170.0), (0.3, 140.0), (0.7, 125.0), (1.1, 105.0), (1.35, 60.0), (1.45, 0.0)],
        propellant_mass: 0.105,
        total_mass: 0.230,
        length: 0.205,
        diameter: 0.038,
    }
}

/// A 1.1 m, 76 mm rocket whose dry CG sits at `dry_cg` (m from the tip): 0.62 is stable (about 2.9 calibers), 0.95 is not.
fn rocket(dry_cg: f64, cant: f64) -> Vehicle {
    let r = 0.038;
    Vehicle {
        reference_diameter: 2.0 * r,
        body: vec![
            BodySegment { station: 0.0, length: 0.30, fore_radius: 0.0, aft_radius: r, kind: BodyKind::Nose(NoseShape::TangentOgive), roughness: 30e-6 },
            BodySegment { station: 0.30, length: 0.80, fore_radius: r, aft_radius: r, kind: BodyKind::Tube, roughness: 30e-6 },
        ],
        tube_fins: vec![],
        fin_sets: vec![FinSet {
            count: 4,
            station: 0.93,
            body_radius: r,
            root_chord: 0.17,
            tip_chord: 0.07,
            span: 0.10,
            sweep: 0.09,
            thickness: 0.003,
            cross_section: FinCrossSection::Rounded,
            cant,
            roughness: 30e-6,
            planform: FinPlanform::Trapezoidal,
            angles: Vec::new(),
            parent: 0,
            bodies: 1,
            mid_chord_cos: None,
            root_extent: None,
        }],
        protuberances: vec![],
        stages: vec![StageMass {
            dry: MassProps { mass: 1.07, cg: dry_cg, i_long: 0.1, i_rot: 0.0008 },
            fore_station: 0.0,
            aft_station: 1.10,
            motor: Some(StageMotor { curve: h128(), mount_aft_station: 1.10, ignition: Timing::LAUNCH, ejection_delay: Some(9.0), tilt_lockout: None }),
            separation: Timing::BURNOUT,
        }],
        labels: Default::default(),
    }
}

/// Flight with a 1.5 m rail tilted `tilt` rad from vertical, a `wind` m/s wind from the east, a main parachute at apogee.
fn fly(vehicle: Vehicle, wind: f64, tilt: f64, model: FlightModel) -> FlightResult {
    let launch = Launch { rail_length: 1.5, angle_from_vertical: tilt, wind_speed: wind, wind_from: std::f64::consts::FRAC_PI_2, ..Launch::default() };
    let recovery = vec![RecoveryDevice { name: "Main".into(), stage: 0, cd_area: 0.9, deploy: Deploy::Apogee, delay: 0.0 }];
    let mut input = SimInput::new(vehicle, launch, recovery);
    input.model = model;
    simulate(&input)
}

fn event_time(r: &FlightResult, kind: EventKind) -> f64 {
    r.trajectory.events.iter().find(|e| e.kind == kind).unwrap_or_else(|| panic!("no {kind:?}")).t
}

fn deg(x: f64) -> f64 {
    x.to_degrees()
}

#[test]
fn rigid_body_is_the_default_model() {
    assert_eq!(FlightModel::default(), FlightModel::RigidBody);
    assert_eq!(SimInput::new(rocket(0.62, 0.0), Launch::default(), vec![]).model, FlightModel::RigidBody);
}

#[test]
fn active_roll_control_holds_the_spin_of_canted_fins_near_zero() {
    let cant = 3f64.to_radians();
    let uncontrolled = fly(rocket(0.62, cant), 0.0, 0.0, FlightModel::RigidBody);
    let mut v = rocket(0.62, cant);
    let rc = RollControl { target_rate: 0.0, start: 0.5, kp: 0.007, ki: 0.2, max_cant: 15f64.to_radians(), max_rate: 10f64.to_radians() };
    v.labels.fin_sets = vec![Label { name: "Fins".into(), part: Some(7), roll_control: Some(rc), ..Label::default() }];
    let controlled = fly(v, 0.0, 0.0, FlightModel::RigidBody);
    let apogee = event_time(&controlled, EventKind::Apogee);
    let up: Vec<&Sample> = controlled.trajectory.samples.iter().filter(|s| s.t <= apogee).collect();
    let worst = uncontrolled.summary.max_roll_rate.abs();
    let last = up.last().unwrap().roll_rate.abs();
    assert!(worst > 1.0, "the canted fins spin the rocket: {worst} rad/s");
    assert!(last < 0.1 * worst, "held: {last} rad/s at apogee against {worst} rad/s uncontrolled");
    assert!(up.iter().all(|s| s.control_cant.abs() <= 15f64.to_radians() + 1e-12));
    assert!(up.iter().any(|s| s.control_cant < cant - 1e-6), "the controller turned the cant against the spin");
}

#[test]
fn a_stable_rocket_weathercocks_into_a_crosswind_and_the_angle_of_attack_decays_oscillating() {
    let r = fly(rocket(0.62, 0.0), 5.0, 0.0, FlightModel::RigidBody);
    assert!(r.summary.stability_off_rail_cal > 2.0);
    let (exit, burnout) = (event_time(&r, EventKind::RailExit), event_time(&r, EventKind::Burnout));
    let aoa = |from: f64, to: f64| -> Vec<(f64, f64)> {
        r.trajectory.samples.iter().filter(|s| s.t >= from && s.t <= to).map(|s| (s.t, deg(s.angle_of_attack))).collect()
    };
    let peak = |v: &[(f64, f64)]| v.iter().map(|p| p.1).fold(0.0, f64::max);

    // Leaving the rail in a 5 m/s crosswind at ~20 m/s the air meets the rocket 10-15 degrees off its axis.
    let early = aoa(exit, exit + 0.3);
    assert!(peak(&early) > 8.0, "peak {}", peak(&early));
    // It weathercocks: the angle comes back through (near) zero, overshoots, and the swings shrink.
    let window = aoa(exit, exit + 1.2);
    let minima: Vec<&(f64, f64)> = window.windows(3).filter(|w| w[1].1 < w[0].1 && w[1].1 < w[2].1).map(|w| &w[1]).collect();
    assert!(minima.len() >= 2, "AoA should swing about the wind at least twice: {minima:?}");
    let maxima: Vec<f64> = window.windows(3).filter(|w| w[1].1 > w[0].1 && w[1].1 > w[2].1).map(|w| w[1].1).collect();
    assert!(maxima.windows(2).all(|m| m[1] < m[0]), "each swing is smaller than the last: {maxima:?}");
    // Settled well within the boost: a fraction of a degree, and the pitch rate is small too.
    let settled = aoa(exit + 1.0, burnout);
    assert!(peak(&settled) < 0.5, "AoA after {} s: {}", exit + 1.0, peak(&settled));
    assert!(peak(&settled) < 0.05 * peak(&early));
    // The rocket leaned over: its axis is no longer vertical while the point-mass model's thrust is never off the velocity.
    let tilt = r.trajectory.samples.iter().filter(|s| s.t > exit && s.t < burnout).map(|s| 90.0 - deg(s.attitude_angle)).fold(0.0, f64::max);
    assert!(tilt > 5.0, "tilt {tilt}");
    // Pitch rate is real while it swings.
    assert!(r.trajectory.samples.iter().any(|s| s.pitch_rate > 0.5));
    // No roll without cant.
    assert!(r.trajectory.samples.iter().all(|s| s.roll_rate.abs() < 1e-9));
}

#[test]
fn the_nose_leans_into_the_wind() {
    // Wind from the east: the rocket's ground velocity at burnout points east (it flies into the wind's source).
    let r = fly(rocket(0.62, 0.0), 5.0, 0.0, FlightModel::RigidBody);
    let burnout = event_time(&r, EventKind::Burnout);
    let s = r.trajectory.samples.iter().find(|s| s.t >= burnout).unwrap();
    assert!(s.velocity[0] > 0.0 && s.velocity[1].abs() < 0.5 * s.velocity[0].abs(), "{:?}", s.velocity);
}

#[test]
fn an_unstable_rocket_diverges_where_the_stable_one_settles() {
    let wind = 3.0;
    let stable = fly(rocket(0.62, 0.0), wind, 0.0, FlightModel::RigidBody);
    let unstable = fly(rocket(0.95, 0.0), wind, 0.0, FlightModel::RigidBody);
    assert!(stable.summary.stability_off_rail_cal > 2.0 && unstable.summary.stability_off_rail_cal < 0.0);
    let exit = event_time(&unstable, EventKind::RailExit);
    let max_aoa =
        |r: &FlightResult, to: f64| r.trajectory.samples.iter().filter(|s| s.t > exit && s.t < to).map(|s| deg(s.angle_of_attack)).fold(0.0, f64::max);
    assert!(max_aoa(&stable, exit + 3.0) < 20.0, "stable {}", max_aoa(&stable, exit + 3.0));
    // The unstable rocket's angle of attack grows without bound until it tumbles through more than 90 degrees.
    assert!(max_aoa(&unstable, exit + 3.0) > 90.0, "unstable {}", max_aoa(&unstable, exit + 3.0));
    // ... and it loses most of its altitude (a point mass flies it as if stable).
    let point = fly(rocket(0.95, 0.0), wind, 0.0, FlightModel::PointMass);
    assert!(unstable.summary.apogee < 0.5 * point.summary.apogee, "{} vs {}", unstable.summary.apogee, point.summary.apogee);
    // Every sample stays finite while it tumbles.
    assert!(unstable.trajectory.samples.iter().all(|s| s.position.iter().chain(&s.velocity).all(|v| v.is_finite()) && s.pitch_rate.is_finite()));
}

#[test]
fn without_wind_the_rigid_body_reaches_the_point_mass_apogee_within_one_percent() {
    for tilt in [0.0, 5.0_f64.to_radians()] {
        let point = fly(rocket(0.62, 0.0), 0.0, tilt, FlightModel::PointMass).summary;
        let rigid = fly(rocket(0.62, 0.0), 0.0, tilt, FlightModel::RigidBody).summary;
        assert!((rigid.apogee / point.apogee - 1.0).abs() < 0.01, "tilt {tilt}: {} vs {}", rigid.apogee, point.apogee);
        assert!((rigid.max_velocity / point.max_velocity - 1.0).abs() < 0.01);
        assert!((rigid.flight_time / point.flight_time - 1.0).abs() < 0.01);
    }
}

#[test]
fn a_calm_vertical_flight_has_no_angle_of_attack_and_no_rotation() {
    let r = fly(rocket(0.62, 0.0), 0.0, 0.0, FlightModel::RigidBody);
    let apogee = event_time(&r, EventKind::Apogee);
    assert!(r.trajectory.samples.iter().all(|s| s.pitch_rate < 1e-9 && s.roll_rate.abs() < 1e-9));
    // Up to apogee the axis stays vertical and the air meets it head on.
    assert!(r
        .trajectory
        .samples
        .iter()
        .filter(|s| s.t < apogee)
        .all(|s| s.angle_of_attack < 1e-9 && (s.attitude_angle - std::f64::consts::FRAC_PI_2).abs() < 1e-9));
}

#[test]
fn point_mass_still_weathercocks_perfectly() {
    let r = fly(rocket(0.62, 0.0), 5.0, 0.0, FlightModel::PointMass);
    let exit = event_time(&r, EventKind::RailExit);
    assert!(r.trajectory.samples.iter().filter(|s| s.t > exit + 0.1 && s.air_speed > 1.0).all(|s| s.angle_of_attack < 1e-6 && s.pitch_rate == 0.0));
}

#[test]
fn the_angle_of_attack_swings_are_damped_by_the_design() {
    // Same rocket, but the swings after rail exit decay faster with the fins' damping than they would undamped: the ratio of the
    // second to the first AoA peak is well below 1. A rocket with nearly no body to damp (CG far forward, short lever arm
    // changes the frequency) still decays, so compare against the undamped limit exp(0) = 1.
    let r = fly(rocket(0.62, 0.0), 5.0, 0.0, FlightModel::RigidBody);
    let exit = event_time(&r, EventKind::RailExit);
    let v: Vec<f64> = r.trajectory.samples.iter().filter(|s| s.t >= exit && s.t <= exit + 1.2).map(|s| s.angle_of_attack).collect();
    let peaks: Vec<f64> = v.windows(3).filter(|w| w[1] > w[0] && w[1] > w[2]).map(|w| w[1]).collect();
    assert!(peaks.len() >= 2, "{peaks:?}");
    assert!(peaks[1] / peaks[0] < 0.7, "second swing is {} of the first", peaks[1] / peaks[0]);
}

#[test]
fn canted_fins_spin_the_rocket_up_to_the_roll_rate_of_the_fin_helix() {
    let cant = 1.0_f64.to_radians();
    let r = fly(rocket(0.62, cant), 0.0, 0.0, FlightModel::RigidBody);
    let fast = r.trajectory.samples.iter().max_by(|a, b| a.air_speed.total_cmp(&b.air_speed)).unwrap();
    // Steady roll: each fin section at radius r meets the flow at the cant angle when w r / V = cant, so w = V cant / r_eff with
    // r_eff the chord-weighted fin radius, between the root (0.038 m) and the tip (0.138 m).
    let (lo, hi) = (fast.air_speed * cant / 0.138, fast.air_speed * cant / 0.038);
    assert!(fast.roll_rate > lo && fast.roll_rate < hi, "roll rate {} not in {lo}..{hi}", fast.roll_rate);
    // Positive cant gives positive roll, negative gives negative, and flight stays unaffected otherwise.
    let back = fly(rocket(0.62, -cant), 0.0, 0.0, FlightModel::RigidBody);
    let fast_back = back.trajectory.samples.iter().max_by(|a, b| a.air_speed.total_cmp(&b.air_speed)).unwrap();
    assert!((fast_back.roll_rate + fast.roll_rate).abs() < 1e-6 * fast.roll_rate.abs());
    assert!((back.summary.apogee / r.summary.apogee - 1.0).abs() < 1e-3);
    // It spins up from rest: no roll at the pad.
    assert!(r.trajectory.samples[0].roll_rate.abs() < 1e-9);
}

#[test]
fn under_a_parachute_the_descent_reverts_to_a_point_mass() {
    let r = fly(rocket(0.62, 0.0), 5.0, 0.0, FlightModel::RigidBody);
    let deploy = r.trajectory.events.iter().find(|e| matches!(e.kind, EventKind::Deploy(_))).unwrap().t;
    let under: Vec<&Sample> = r.trajectory.samples.iter().filter(|s| s.t > deploy + 0.01 && s.air_speed > 1.0).collect();
    assert!(under.len() > 100);
    assert!(under.iter().all(|s| s.pitch_rate == 0.0 && s.roll_rate == 0.0 && s.angle_of_attack < 1e-6));
    // It hangs nose first into the descent.
    assert!(under.iter().all(|s| s.attitude_angle < 0.0));
    // And the whole flight lands with the speed of the canopy.
    assert!(r.summary.ground_hit_velocity < 10.0 && r.summary.ground_hit_velocity > 2.0, "{}", r.summary.ground_hit_velocity);
}

#[test]
fn weathercocking_lifts_the_apogee_above_the_point_mass_in_wind() {
    // The point-mass model turns the thrust into the relative wind the instant the rail is left; the rigid body takes time to
    // turn, so more of the impulse goes up: the windy apogee is higher (and drift smaller on the way up).
    let point = fly(rocket(0.62, 0.0), 8.0, 5.0_f64.to_radians(), FlightModel::PointMass).summary;
    let rigid = fly(rocket(0.62, 0.0), 8.0, 5.0_f64.to_radians(), FlightModel::RigidBody).summary;
    assert!(rigid.apogee > point.apogee, "{} vs {}", rigid.apogee, point.apogee);
    assert!(rigid.apogee < 1.3 * point.apogee);
}

#[test]
fn rigid_body_flights_run_fast() {
    let t0 = std::time::Instant::now();
    let _ = fly(rocket(0.62, 0.0), 5.0, 0.0, FlightModel::RigidBody);
    let limit = if cfg!(debug_assertions) { 3000 } else { 100 };
    assert!(t0.elapsed().as_millis() < limit, "took {:?}", t0.elapsed());
}

#[test]
fn the_summary_reports_the_peak_burnout_and_apogee_spin_of_canted_fins() {
    let cant = 1.0_f64.to_radians();
    let r = fly(rocket(0.62, cant), 0.0, 0.0, FlightModel::RigidBody);
    let s = &r.summary;
    let peak = r.trajectory.samples.iter().map(|s| s.roll_rate).fold(f64::MIN, f64::max);
    // The summary reads the undecimated flight: the samples' peak is within a decimation step of it.
    assert!(s.max_roll_rate > 0.0 && (s.max_roll_rate - peak).abs() < 0.02 * peak, "{} vs {peak}", s.max_roll_rate);
    // The fin helix spins the rocket up with the airspeed, which peaks at burnout; it coasts on, a little slower, to apogee, where
    // the deployment stops it: the number is the rate just before.
    assert!(s.roll_rate_at_burnout > 0.95 * s.max_roll_rate, "{} vs {}", s.roll_rate_at_burnout, s.max_roll_rate);
    assert!(s.roll_rate_at_apogee > 0.0 && s.roll_rate_at_apogee < s.roll_rate_at_burnout, "{}", s.roll_rate_at_apogee);
    // A rocket without canted fins does not spin, and has nothing to resonate with.
    let plain = fly(rocket(0.62, 0.0), 0.0, 0.0, FlightModel::RigidBody).summary;
    assert_eq!((plain.max_roll_rate, plain.roll_rate_at_burnout, plain.roll_rate_at_apogee, plain.roll_resonance), (0.0, 0.0, 0.0, None));
    // The other way round is the mirror image.
    let back = fly(rocket(0.62, -cant), 0.0, 0.0, FlightModel::RigidBody).summary;
    assert!((back.max_roll_rate + s.max_roll_rate).abs() < 1e-6 * s.max_roll_rate && back.roll_rate_at_burnout < 0.0);
}

#[test]
fn a_spin_that_outruns_the_pitch_frequency_crosses_it_while_spinning_up() {
    // The pitch frequency of this rocket peaks near 36 rad/s. One degree of cant spins it at about 0.6 of the pitch frequency all
    // the way (both follow the airspeed): no meeting. Three degrees spins it at about 1.8 times: it passes through while
    // the rocket accelerates off the rail.
    let gentle = fly(rocket(0.62, 1.0_f64.to_radians()), 0.0, 0.0, FlightModel::RigidBody);
    assert_eq!(gentle.summary.roll_resonance, None);
    let hard = fly(rocket(0.62, 3.0_f64.to_radians()), 0.0, 0.0, FlightModel::RigidBody);
    let res = hard.summary.roll_resonance.expect("three degrees outrun the pitch frequency");
    assert!(res.crossed && res.time > 0.17 && res.time < 1.45, "{res:?}");
    assert!((res.roll_rate - res.pitch_frequency).abs() < 0.05 * res.pitch_frequency, "{res:?}");
    // The recorded pitch frequency is there to compare: none on the rail, none under the parachute, tens of rad/s in between.
    let samples = &hard.trajectory.samples;
    assert_eq!(samples[0].pitch_frequency, 0.0);
    let peak = samples.iter().map(|s| s.pitch_frequency).fold(0.0, f64::max);
    assert!((20.0..60.0).contains(&peak), "{peak}");
    let deploy = event_time(&hard, EventKind::Deploy("Main".into()));
    assert!(samples.iter().filter(|s| s.t > deploy + 0.01).all(|s| s.pitch_frequency == 0.0));
}
