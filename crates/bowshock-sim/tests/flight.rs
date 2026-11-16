//! Integration tests: full flights of an H128W-class rocket.

use bowshock_sim::*;

/// Approximate Aerotech H128W: 176 N s, ~1.4 s burn (hand table), 38 mm x 0.2 m, 0.115 kg propellant.
fn h128() -> ThrustCurve {
    ThrustCurve {
        points: vec![(0.0, 0.0), (0.04, 190.0), (0.1, 170.0), (0.3, 140.0), (0.7, 125.0), (1.1, 105.0), (1.35, 60.0), (1.45, 0.0)],
        propellant_mass: 0.105,
        total_mass: 0.230,
        length: 0.205,
        diameter: 0.038,
    }
}

fn rocket() -> Vehicle {
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
            cant: 0.0,
            roughness: 30e-6,
            planform: FinPlanform::Trapezoidal,
            angles: Vec::new(),
            parent: 0,
            bodies: 1,
            mid_chord_cos: None,
            root_extent: None,
        }],
        protuberances: vec![Protuberance::RailButton { station: 0.8, height: 0.008, diameter: 0.009 }],
        stages: vec![StageMass {
            // Airframe without motor: 1.3 kg total at pad with the motor (0.23 kg).
            dry: MassProps { mass: 1.07, cg: 0.62, i_long: 0.1, i_rot: 0.0008 },
            fore_station: 0.0,
            aft_station: 1.10,
            motor: Some(StageMotor { curve: h128(), mount_aft_station: 1.10, ignition: Timing::LAUNCH, ejection_delay: Some(9.0) }),
            separation: Timing::BURNOUT,
        }],
        labels: Default::default(),
    }
}

fn input() -> SimInput {
    let launch = Launch { rail_length: 1.5, wind_speed: 2.0, wind_from: 1.0, ..Launch::default() };
    let recovery = vec![RecoveryDevice { name: "Main".into(), stage: 0, cd_area: 0.9, deploy: Deploy::Ejection, delay: 0.0 }];
    SimInput::new(rocket(), launch, recovery)
}

#[test]
fn the_wind_profile_interpolates_between_levels_and_holds_above() {
    let west = 270f64.to_radians();
    let launch = Launch { wind_speed: 2.0, wind_from: west, wind_levels: vec![WindPoint { altitude: 1000.0, speed: 20.0, from: west }], ..Launch::default() };
    // A wind from the west blows east: +x.
    let at = |h: f64| launch.wind_at(h);
    assert!((at(500.0)[0] - 11.0).abs() < 1e-9 && at(500.0)[1].abs() < 1e-9, "{:?}", at(500.0));
    assert!((at(2000.0)[0] - 20.0).abs() < 1e-9);
    assert!((at(0.0)[0] - 2.0).abs() < 1e-9);
    // No levels: the same wind everywhere.
    let flat = Launch { wind_speed: 2.0, wind_from: west, ..Launch::default() };
    assert_eq!(flat.wind_at(0.0), flat.wind_at(5000.0));
}

#[test]
fn the_rotating_earth_moves_the_landing_and_gravity_follows_latitude() {
    assert!(normal_gravity(std::f64::consts::FRAC_PI_2) > normal_gravity(0.0));
    assert!((normal_gravity(0.0) - 9.7803253359).abs() < 1e-9);
    let fly = |latitude: Option<f64>| {
        let launch = Launch { rail_length: 1.5, latitude, ..Launch::default() };
        let rec = vec![RecoveryDevice { name: "Main".into(), stage: 0, cd_area: 0.9, deploy: Deploy::Ejection, delay: 0.0 }];
        let mut input = SimInput::new(rocket(), launch, rec);
        input.model = FlightModel::PointMass;
        simulate(&input).summary.landing_position
    };
    let (flat, turning) = (fly(None), fly(Some(45f64.to_radians())));
    assert!(turning.iter().all(|x| x.is_finite()));
    let moved = (turning[0] - flat[0]).hypot(turning[1] - flat[1]);
    assert!(moved > 0.01, "the landing moved {moved} m");
}

#[test]
fn an_air_start_begins_at_its_altitude_and_speed() {
    let launch = Launch { air_start: Some((1000.0, 100.0)), ..Launch::default() };
    let r = simulate(&SimInput::new(rocket(), launch, vec![]));
    let first = &r.trajectory.samples[0];
    assert!((first.altitude - 1000.0).abs() < 1e-9 && (first.speed - 100.0).abs() < 1e-9, "{} m at {} m/s", first.altitude, first.speed);
    assert!(r.summary.apogee > 1000.0);
    assert_eq!(r.summary.rail_exit_velocity, 100.0);
    let rail_exit = r.trajectory.events.iter().find(|e| e.kind == EventKind::RailExit).unwrap();
    assert_eq!((rail_exit.t, rail_exit.altitude), (0.0, 1000.0));
}

#[test]
fn h128_flight_is_plausible_and_ordered() {
    let h = h128();
    assert!((h.total_impulse() - 176.0).abs() < 12.0, "impulse {}", h.total_impulse());
    let r = simulate(&input());
    let s = &r.summary;
    assert!((400.0..800.0).contains(&s.apogee), "apogee {}", s.apogee);
    assert!(s.max_velocity > 60.0 && s.max_velocity < 250.0, "vmax {}", s.max_velocity);
    assert!(s.rail_exit_velocity > 15.0, "rail exit {}", s.rail_exit_velocity);
    assert!(s.stability_off_rail_cal > 1.0 && s.stability_off_rail_cal < 5.0, "margin {}", s.stability_off_rail_cal);
    assert!(s.optimum_delay > 4.0 && s.optimum_delay < 12.0, "delay {}", s.optimum_delay);
    assert!(s.ground_hit_velocity < 10.0 && s.ground_hit_velocity > 2.0, "hit {}", s.ground_hit_velocity);
    assert_eq!(s.descent_rates.len(), 1);

    let ev = &r.trajectory.events;
    let order: Vec<&EventKind> = ev.iter().map(|e| &e.kind).collect();
    let pos = |k: &EventKind| order.iter().position(|x| *x == k).unwrap_or_else(|| panic!("missing {k:?} in {order:?}"));
    let (i_ign, i_rail, i_bo, i_ap, i_land) =
        (pos(&EventKind::Ignition), pos(&EventKind::RailExit), pos(&EventKind::Burnout), pos(&EventKind::Apogee), pos(&EventKind::Landing));
    let i_dep = pos(&EventKind::Deploy("Main".into()));
    assert!(i_ign < i_rail && i_rail < i_bo && i_bo < i_ap && i_ap < i_dep && i_dep < i_land, "{order:?}");
    assert!(ev.windows(2).all(|w| w[0].t <= w[1].t + 1e-9));
    // Ejection at burnout + 9 s, after the ballistic apogee (~8 s after burnout).
    let burnout = ev[i_bo].t;
    assert!((ev[i_dep].t - (burnout + 9.0)).abs() < 1e-6);
    // Flight really ends at the ground.
    assert!(ev[i_land].altitude.abs() < 1e-9);

    let n = r.trajectory.samples.len();
    assert!(n > 200 && n <= 2300, "{n} samples");
    assert!(r.trajectory.samples.windows(2).all(|w| w[0].t < w[1].t));
    // Wind from direction 1 rad (NNE-ish) pushes the rocket downwind (south-west).
    assert!(s.landing.0 > 5.0);
}

#[test]
fn runs_fast() {
    let inp = input();
    let t0 = std::time::Instant::now();
    let _ = simulate(&inp);
    let dt = t0.elapsed();
    // 50 ms in release; generous in debug builds.
    let limit = if cfg!(debug_assertions) { 2000 } else { 50 };
    assert!(dt.as_millis() < limit, "took {dt:?}");
}

#[test]
fn no_wind_vertical_flight_lands_near_pad() {
    let mut inp = input();
    inp.launch.wind_speed = 0.0;
    let r = simulate(&inp);
    assert!(r.summary.landing.0 < 1.0, "{:?}", r.summary.landing);
}

#[test]
fn heavier_rocket_flies_lower() {
    let mut inp = input();
    inp.launch.wind_speed = 0.0;
    let base = simulate(&inp).summary.apogee;
    inp.vehicle.stages[0].dry.mass += 0.5;
    let heavy = simulate(&inp).summary.apogee;
    assert!(heavy < base, "{heavy} vs {base}");
}

#[test]
fn drogue_then_main() {
    let mut inp = input();
    inp.recovery = vec![
        RecoveryDevice { name: "Drogue".into(), stage: 0, cd_area: 0.1, deploy: Deploy::Apogee, delay: 1.0 },
        RecoveryDevice { name: "Main".into(), stage: 0, cd_area: 1.2, deploy: Deploy::Altitude(150.0), delay: 0.0 },
    ];
    let r = simulate(&inp);
    let s = &r.summary;
    assert_eq!(s.descent_rates.len(), 2);
    assert!(s.descent_rates[0].1 > s.descent_rates[1].1);
    let names: Vec<_> =
        r.trajectory.events.iter().filter_map(|e| if let EventKind::Deploy(n) = &e.kind { Some((n.clone(), e.altitude)) } else { None }).collect();
    assert_eq!(names[0].0, "Drogue");
    assert_eq!(names[1].0, "Main");
    assert!(names[1].1 <= 150.0 && names[1].1 > 140.0, "{names:?}");
}

/// The two-stage example: the H128 booster below the H128 sustainer; `ignition` and `separation` are the sustainer's ignition and
/// the booster's separation.
fn two_stage(ignition: Timing, separation: Timing) -> Vehicle {
    let mut v = rocket();
    let booster = StageMass {
        dry: MassProps { mass: 0.4, cg: 1.3, i_long: 0.01, i_rot: 0.0003 },
        fore_station: 1.10,
        aft_station: 1.50,
        motor: Some(StageMotor { curve: h128(), mount_aft_station: 1.50, ignition: Timing::LAUNCH, ejection_delay: None }),
        separation,
    };
    let mut sustainer = v.stages[0].clone();
    sustainer.motor.as_mut().unwrap().ignition = ignition;
    sustainer.motor.as_mut().unwrap().ejection_delay = Some(6.0);
    v.stages = vec![booster, sustainer];
    v.body.push(BodySegment { station: 1.10, length: 0.4, fore_radius: 0.038, aft_radius: 0.038, kind: BodyKind::Tube, roughness: 30e-6 });
    v
}

fn fly_two_stage(ignition: Timing, separation: Timing) -> FlightResult {
    let launch = Launch { rail_length: 2.0, ..Launch::default() };
    let rec = vec![RecoveryDevice { name: "Main".into(), stage: 1, cd_area: 0.9, deploy: Deploy::Ejection, delay: 0.0 }];
    simulate(&SimInput::new(two_stage(ignition, separation), launch, rec))
}

/// `(time, altitude)` of the event of `kind` for `stage`.
fn event_of(r: &FlightResult, kind: EventKind, stage: usize) -> (f64, f64) {
    let e = r.trajectory.events.iter().find(|e| e.kind == kind && e.stage == Some(stage)).unwrap_or_else(|| panic!("no {kind:?} of stage {stage}"));
    (e.t, e.altitude)
}

#[test]
fn two_stage_separates_and_sustainer_flies_higher_than_booster_alone() {
    // Booster at launch, sustainer after a 1 s coast.
    let r = fly_two_stage(Timing::new(Trigger::Burnout, 1.0), Timing::BURNOUT);
    let kinds: Vec<_> = r.trajectory.events.iter().map(|e| e.kind.clone()).collect();
    assert!(kinds.contains(&EventKind::Separation), "{kinds:?}");
    assert_eq!(kinds.iter().filter(|k| **k == EventKind::Ignition).count(), 2);
    assert!(r.summary.apogee > 600.0, "{}", r.summary.apogee);
}

#[test]
fn burnout_triggers_separate_at_burnout_and_ignite_a_delay_later() {
    let r = fly_two_stage(Timing::new(Trigger::Burnout, 1.0), Timing::BURNOUT);
    let burnout = event_of(&r, EventKind::Burnout, 0).0;
    assert!((event_of(&r, EventKind::Separation, 0).0 - burnout).abs() < 0.02, "separation at burnout");
    assert!((event_of(&r, EventKind::Ignition, 1).0 - (burnout + 1.0)).abs() < 0.02, "sustainer lights 1 s after the booster burns out");
}

#[test]
fn separation_trigger_lights_the_upper_stage_after_the_separation_not_the_burnout() {
    // Separation 0.25 s after burnout, then ignition 0.5 s after that: separation and ignition are different instants.
    let r = fly_two_stage(Timing::new(Trigger::Separation, 0.5), Timing::new(Trigger::Burnout, 0.25));
    let burnout = event_of(&r, EventKind::Burnout, 0).0;
    let (sep, ign) = (event_of(&r, EventKind::Separation, 0).0, event_of(&r, EventKind::Ignition, 1).0);
    assert!((sep - (burnout + 0.25)).abs() < 0.02, "sep {sep} burnout {burnout}");
    assert!((ign - (sep + 0.5)).abs() < 0.02, "ign {ign} sep {sep}");
}

#[test]
fn time_from_launch_triggers_fire_at_those_times() {
    let r = fly_two_stage(Timing::new(Trigger::Launch, 5.0), Timing::new(Trigger::Launch, 4.0));
    assert!((event_of(&r, EventKind::Separation, 0).0 - 4.0).abs() < 0.02);
    assert!((event_of(&r, EventKind::Ignition, 1).0 - 5.0).abs() < 0.02);
    // The booster burned out before it was separated.
    assert!(event_of(&r, EventKind::Burnout, 0).0 < 4.0);
}

#[test]
fn a_stage_dropped_before_its_burnout_never_burns_out() {
    let r = fly_two_stage(Timing::new(Trigger::Separation, 0.0), Timing::new(Trigger::Launch, 0.5));
    assert!(!r.trajectory.events.iter().any(|e| e.kind == EventKind::Burnout && e.stage == Some(0)));
    assert!((event_of(&r, EventKind::Separation, 0).0 - 0.5).abs() < 0.02);
}

#[test]
fn altitude_triggers_fire_climbing_through_the_altitude() {
    // Separate on the way up through 150 m, light the sustainer 0.5 s after climbing through 200 m.
    let r = fly_two_stage(Timing::new(Trigger::Altitude(200.0), 0.5), Timing::new(Trigger::Altitude(150.0), 0.0));
    let (t_sep, h_sep) = event_of(&r, EventKind::Separation, 0);
    assert!((150.0..160.0).contains(&h_sep), "separated at {h_sep} m");
    let (t_ign, _) = event_of(&r, EventKind::Ignition, 1);
    // The ignition is 0.5 s after the rocket crossed 200 m: find that crossing in the samples.
    let crossing = r.trajectory.samples.iter().find(|s| s.altitude >= 200.0 && s.t > t_sep).expect("the rocket climbs through 200 m").t;
    assert!(t_ign > crossing && t_ign - crossing < 0.6, "ignition {t_ign}, crossing {crossing}");
    assert!(r.summary.apogee > 600.0);
}

#[test]
fn an_altitude_never_reached_never_fires() {
    let r = fly_two_stage(Timing::new(Trigger::Separation, 0.0), Timing::new(Trigger::Altitude(50_000.0), 0.0));
    assert!(!r.trajectory.events.iter().any(|e| matches!(e.kind, EventKind::Separation)));
    // The sustainer never lights either: it waits for a separation that never comes.
    assert!(!r.trajectory.events.iter().any(|e| e.kind == EventKind::Ignition && e.stage == Some(1)));
}

#[test]
fn the_static_timeline_and_the_flight_agree() {
    let tl = Timeline::of(&two_stage(Timing::new(Trigger::Separation, 0.5), Timing::new(Trigger::Burnout, 0.25)));
    let r = fly_two_stage(Timing::new(Trigger::Separation, 0.5), Timing::new(Trigger::Burnout, 0.25));
    for (kind, stage, time) in [
        (EventKind::Burnout, 0, tl.burnout[0]),
        (EventKind::Separation, 0, tl.separation[0]),
        (EventKind::Ignition, 1, tl.ignition[1]),
        (EventKind::Burnout, 1, tl.burnout[1]),
    ] {
        let flown = event_of(&r, kind.clone(), stage).0;
        assert!((flown - time.unwrap()).abs() < 0.02, "{kind:?} {stage}: flown {flown}, timeline {time:?}");
    }
    // Altitude triggers are unknown to the static timeline and resolved from a flight by `staging_times`.
    let alt = two_stage(Timing::new(Trigger::Separation, 0.0), Timing::new(Trigger::Altitude(150.0), 0.0));
    assert_eq!(Timeline::of(&alt).separation[0], None);
    let resolved = staging_times(&SimInput::new(alt, Launch { rail_length: 2.0, ..Launch::default() }, vec![]));
    let t_sep = resolved.separation[0].expect("resolved by a flight");
    assert!(resolved.ignition[1].is_some_and(|t| (t - t_sep).abs() < 1e-9));
}

#[test]
fn deployment_after_launch_fires_that_many_seconds_in() {
    let rec = vec![RecoveryDevice { name: "Drogue".into(), stage: 0, cd_area: 0.1, deploy: Deploy::Launch, delay: 3.0 }];
    let r = simulate(&SimInput::new(rocket(), Launch { rail_length: 1.5, ..Launch::default() }, rec));
    let e = r.trajectory.events.iter().find(|e| matches!(e.kind, EventKind::Deploy(_))).expect("deploys");
    assert!((e.t - 3.0).abs() < 0.02, "deployed at {}", e.t);
}

#[test]
fn monte_carlo_is_reproducible_and_spreads() {
    let inp = input();
    let sigma = Dispersion { wind_speed: 1.0, wind_dir: 0.3, launch_angle: 0.02, thrust_scale: 0.03, cd_scale: 0.05, mass: 0.02 };
    let a = monte_carlo(&inp, 24, &sigma);
    let b = monte_carlo(&inp, 24, &sigma);
    assert_eq!(a, b);
    assert_eq!(a.runs.len(), 24);
    assert!(a.stats.apogee_std > 1.0 && a.stats.ellipse_major > 1.0, "{:?}", a.stats);
}

#[test]
fn canopy_inflation_limits_opening_shock() {
    let mut inp = input();
    inp.launch.wind_speed = 0.0;
    inp.recovery = vec![
        RecoveryDevice { name: "Drogue".into(), stage: 0, cd_area: 0.1, deploy: Deploy::Apogee, delay: 0.0 },
        RecoveryDevice { name: "Main".into(), stage: 0, cd_area: 1.2, deploy: Deploy::Altitude(150.0), delay: 0.0 },
    ];
    let r = simulate(&inp);
    let t_first = r.trajectory.events.iter().find(|e| matches!(e.kind, EventKind::Deploy(_))).unwrap().t;
    let peak_g = r.trajectory.samples.iter().filter(|s| s.t >= t_first).map(|s| s.acceleration).fold(0.0, f64::max) / G0;
    assert!(peak_g < 20.0, "opening shock {peak_g} g");
    // The canopy takes a finite time to fill: speed right after the main deploys is still near the drogue rate.
    let t_main = r.trajectory.events.iter().find(|e| e.kind == EventKind::Deploy("Main".into())).unwrap().t;
    let v = |t: f64| r.trajectory.samples.iter().min_by(|a, b| (a.t - t).abs().total_cmp(&(b.t - t).abs())).unwrap().vertical_velocity.abs();
    assert!(v(t_main + 0.1) > 0.9 * v(t_main), "no inflation delay");
    println!("peak opening shock {peak_g:.2} g");
}

/// A calm flight (no wind, no gusts) of the H128W-class rocket, so air-relative and ground-relative values coincide.
fn calm() -> FlightResult {
    let mut inp = input();
    inp.launch.wind_speed = 0.0;
    inp.launch.turbulence = Turbulence::Off;
    simulate(&inp)
}

#[test]
fn samples_carry_consistent_kinematic_and_atmosphere_channels() {
    let inp = input();
    let site = inp.launch.site_altitude;
    let r = calm();
    let samples = &r.trajectory.samples;
    for s in samples {
        assert!((s.altitude_msl - (site + s.altitude)).abs() < 1e-9);
        assert!((s.ground_distance - s.position[0].hypot(s.position[1])).abs() < 1e-9);
        assert!((s.horizontal_velocity - s.velocity[0].hypot(s.velocity[1])).abs() < 1e-9);
        // Calm air: the air-relative speed is the ground speed and the wind is zero.
        assert!((s.air_speed - s.speed).abs() < 1e-9 && s.wind_speed == 0.0);
        assert!((s.mach - s.air_speed / s.speed_of_sound).abs() < 1e-9);
        assert!((s.dynamic_pressure - 0.5 * s.air_density * s.air_speed * s.air_speed).abs() < 1e-6 * s.dynamic_pressure.max(1.0));
        // Ideal gas: p = rho R T for dry air.
        assert!((s.air_pressure - s.air_density * 287.05 * s.air_temperature).abs() < 0.01 * s.air_pressure);
        assert!((-std::f64::consts::FRAC_PI_2 - 1e-9..=std::f64::consts::FRAC_PI_2 + 1e-9).contains(&s.flight_path_angle));
    }
    // ISA at a 120 m site on a 15 degC day: a = sqrt(1.4 R T) ~ 340 m/s, rho ~ 1.2 kg/m^3.
    let first = samples.first().unwrap();
    assert!((first.speed_of_sound - 340.3).abs() < 1.0, "{}", first.speed_of_sound);
    assert!((first.air_density - 1.21).abs() < 0.03, "{}", first.air_density);
    // Climbing at the start, descending at the end, and the angle always has the sign of the vertical velocity.
    assert!(samples[20].flight_path_angle > 1.4 && samples.last().unwrap().flight_path_angle < -1.4);
    assert!(samples.iter().filter(|s| s.vertical_velocity.abs() > 1e-6).all(|s| s.flight_path_angle.signum() == s.vertical_velocity.signum()));
}

#[test]
fn samples_carry_consistent_drag_mass_and_balance_channels() {
    let inp = input();
    let r = calm();
    let samples = &r.trajectory.samples;
    let d_ref = inp.vehicle.reference_diameter;
    for s in samples {
        let parts = s.cd_friction + s.cd_pressure + s.cd_base + s.cd_fins + s.cd_protuberances;
        assert!((s.cd - parts).abs() < 1e-9, "Cd build-up does not add up at t = {}", s.t);
        assert!((s.stability_margin - (s.cp - s.cg) / d_ref).abs() < 1e-9);
        assert!((s.cd_recovery - s.recovery_cd_area / inp.vehicle.reference_area()).abs() < 1e-12);
        // F = q Cd A (the deployed devices and the airframe share one reference area in this model).
        if s.dynamic_pressure > 1.0 {
            assert!((s.drag - s.dynamic_pressure * (s.cd + s.cd_recovery) * inp.vehicle.reference_area()).abs() < 1e-6 * s.drag.max(1.0), "t = {}", s.t);
        }
    }
    // A slender rocket at low Mach: Cd of order 0.3 to 0.9 with the motor lit.
    let cruise = samples.iter().find(|s| s.t > 1.0 && s.mach > 0.2).unwrap();
    assert!((0.3..1.0).contains(&cruise.cd), "Cd {}", cruise.cd);
    // The propellant burns out and the mass drops by exactly that much; nothing else leaves the rocket.
    let (start, end) = (samples.first().unwrap(), samples.last().unwrap());
    assert!((start.propellant_mass - 0.105).abs() < 1e-9 && end.propellant_mass < 1e-9);
    assert!(((start.mass - end.mass) - 0.105).abs() < 1e-6, "{} -> {}", start.mass, end.mass);
    // Stable at the pad (CP aft of CG), and the CG moves forward as the propellant burns.
    assert!(start.stability_margin > 1.0 && end.cg < start.cg);
    // Thrust only while burning; the recovery drag area is zero until the canopy opens and full at the end.
    assert!(start.thrust < 1.0 && end.thrust == 0.0 && samples.iter().any(|s| s.thrust > 100.0));
    assert_eq!(start.recovery_cd_area, 0.0);
    assert!((end.recovery_cd_area - 0.9).abs() < 1e-9, "{}", end.recovery_cd_area);
    // Reynolds number of a 1.1 m rocket at ~100 m/s: rho v L / mu ~ 7e6 (sanity range).
    let fast = samples.iter().max_by(|a, b| a.air_speed.total_cmp(&b.air_speed)).unwrap();
    assert!((1e6..2e7).contains(&fast.reynolds), "Re {}", fast.reynolds);
}

#[test]
fn wind_and_angle_of_attack_channels_follow_the_launch_conditions() {
    let r = simulate(&input());
    // Mean wind 2 m/s plus Low-turbulence gusts (5 %): always near 2.
    assert!(r.trajectory.samples.iter().all(|s| (1.0..3.0).contains(&s.wind_speed)));
    // The rigid-body model flies a real angle of attack: the crosswind and the gusts push it off the relative wind, but the
    // rocket is stable, so once it has weathercocked it stays within a few degrees under thrust (the point-mass model weathercocks
    // perfectly and has none).
    let exit = r.trajectory.events.iter().find(|e| e.kind == EventKind::RailExit).unwrap().t;
    let boost: Vec<&Sample> = r.trajectory.samples.iter().filter(|s| s.t > exit + 1.0 && s.t < 1.4 && s.air_speed > 1.0).collect();
    assert!(!boost.is_empty() && boost.iter().all(|s| s.angle_of_attack < 3f64.to_radians()));
    let mut point = input();
    point.model = FlightModel::PointMass;
    let p = simulate(&point);
    assert!(p.trajectory.samples.iter().filter(|s| s.t > exit + 0.1 && s.air_speed > 1.0).all(|s| s.angle_of_attack < 1e-6));
}

// ---- CFD drag table ----

const TABLE_ALT: f64 = 1000.0;

fn re_tab(mach: f64) -> f64 {
    let a = Atmosphere::standard().at_msl(TABLE_ALT);
    mach * a.speed_of_sound * a.density / a.viscosity
}

/// A table that is the build-up scaled by `k`, at the nodes, with distinct parts.
fn table_from_buildup(aero: &Aero, k: f64) -> DragTable {
    let points = [0.3, 0.6, 0.9, 1.2, 2.0]
        .iter()
        .map(|&m| {
            let d = aero.drag(m, re_tab(m), false);
            DragTablePoint { mach: m, friction: k * d.friction, pressure: k * d.pressure, base: k * d.base, fins: k * d.fins, other: k * d.protuberances }
        })
        .collect();
    DragTable { points, altitude: TABLE_ALT }
}

fn close(a: f64, b: f64) {
    assert!((a - b).abs() <= 1e-12, "{a} vs {b}");
}

#[test]
fn a_table_node_at_its_own_reynolds_number_is_returned_as_is() {
    let plain = Aero::new(&input().vehicle);
    let mut table = table_from_buildup(&plain, 1.0);
    for (i, p) in table.points.iter_mut().enumerate() {
        let f = 1.0 + 0.1 * i as f64;
        p.friction *= f;
        p.pressure *= f + 0.05;
        p.base *= f + 0.1;
        p.fins += 0.01;
        p.other += 0.02;
    }
    let aero = plain.clone().with_table(table.clone());
    for p in &table.points {
        let d = aero.drag(p.mach, re_tab(p.mach), false);
        close(d.friction, p.friction);
        close(d.pressure, p.pressure);
        close(d.base, p.base);
        close(d.fins, p.fins);
        close(d.protuberances, p.other);
        close(d.total, p.friction + p.pressure + p.base + p.fins + p.other);
    }
}

#[test]
fn motor_on_scales_only_the_table_base_by_the_buildup_ratio() {
    let plain = Aero::new(&input().vehicle);
    let table = table_from_buildup(&plain, 1.2);
    let aero = plain.clone().with_table(table.clone());
    let m = 0.6;
    let (off, on) = (aero.drag(m, re_tab(m), false), aero.drag(m, re_tab(m), true));
    let (b_off, b_on) = (plain.drag(m, re_tab(m), false), plain.drag(m, re_tab(m), true));
    assert!(b_on.base < b_off.base);
    close(on.base, off.base * b_on.base / b_off.base);
    close(on.friction, off.friction);
    close(on.pressure, off.pressure);
    close(on.fins, off.fins);
    close(on.protuberances, off.protuberances);
}

#[test]
fn reynolds_number_changes_friction_exactly_as_the_buildup() {
    let plain = Aero::new(&input().vehicle);
    let aero = plain.clone().with_table(table_from_buildup(&plain, 1.2));
    let m = 0.9;
    let hi = 3.0 * re_tab(m);
    let (a, b) = (aero.drag(m, re_tab(m), false), aero.drag(m, hi, false));
    close(b.friction - a.friction, plain.drag(m, hi, false).friction - plain.drag(m, re_tab(m), false).friction);
}

#[test]
fn below_the_first_node_the_end_follows_the_buildup_trend() {
    let plain = Aero::new(&input().vehicle);
    let table = table_from_buildup(&plain, 1.3);
    let aero = plain.clone().with_table(table.clone());
    let (m, first) = (0.1, &table.points[0]);
    let d = aero.drag(m, re_tab(m), false);
    let (here, at) = (plain.drag(m, re_tab(m), false), plain.drag(first.mach, re_tab(first.mach), false));
    close(d.friction, first.friction * here.friction / at.friction);
    close(d.pressure, first.pressure * here.pressure / at.pressure);
    close(d.base, first.base * here.base / at.base);
    close(d.fins, first.fins * here.fins / at.fins);
    let last = table.points.last().unwrap();
    let hi = aero.drag(3.0, re_tab(3.0), false);
    let (here, at) = (plain.drag(3.0, re_tab(3.0), false), plain.drag(last.mach, re_tab(last.mach), false));
    close(hi.friction, last.friction * here.friction / at.friction);
}

#[test]
fn without_a_table_the_drag_is_the_buildup() {
    let plain = Aero::new(&input().vehicle);
    let none = plain.clone();
    for m in [0.05, 0.4, 1.1, 2.5] {
        assert_eq!(none.drag(m, 2.0e6, true), plain.drag(m, 2.0e6, true));
    }
    let mut i = input();
    i.drag_tables = vec![None];
    assert_eq!(simulate(&i).summary.apogee, simulate(&input()).summary.apogee);
}

#[test]
fn a_heavier_drag_table_flies_lower() {
    let plain = Aero::new(&input().vehicle);
    let mut i = input();
    i.drag_tables = vec![Some(table_from_buildup(&plain, 1.3))];
    assert!(simulate(&i).summary.apogee < simulate(&input()).summary.apogee);
}
