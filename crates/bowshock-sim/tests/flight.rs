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
        }],
        protuberances: vec![Protuberance::RailButton { station: 0.8, height: 0.008, diameter: 0.009 }],
        stages: vec![StageMass {
            // Airframe without motor: 1.3 kg total at pad with the motor (0.23 kg).
            dry: MassProps { mass: 1.07, cg: 0.62, i_long: 0.05 },
            fore_station: 0.0,
            aft_station: 1.10,
            motor: Some(StageMotor { curve: h128(), mount_aft_station: 1.10, ignition: Ignition::Launch, ejection_delay: Some(9.0) }),
        }],
    }
}

fn input() -> SimInput {
    let launch = Launch { rail_length: 1.5, wind_speed: 2.0, wind_from: 1.0, ..Launch::default() };
    let recovery = vec![RecoveryDevice { name: "Main".into(), stage: 0, cd_area: 0.9, deploy: Deploy::Ejection, delay: 0.0 }];
    SimInput::new(rocket(), launch, recovery)
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

#[test]
fn two_stage_separates_and_sustainer_flies_higher_than_booster_alone() {
    let mut v = rocket();
    // Booster below the sustainer: stage 0 fires at launch, stage 1 (sustainer) after a 1 s coast.
    let booster = StageMass {
        dry: MassProps { mass: 0.4, cg: 1.3, i_long: 0.01 },
        fore_station: 1.10,
        aft_station: 1.50,
        motor: Some(StageMotor { curve: h128(), mount_aft_station: 1.50, ignition: Ignition::Launch, ejection_delay: None }),
    };
    let mut sustainer = v.stages[0].clone();
    sustainer.motor.as_mut().unwrap().ignition = Ignition::AfterPreviousBurnout { delay: 1.0 };
    sustainer.motor.as_mut().unwrap().ejection_delay = Some(6.0);
    v.stages = vec![booster, sustainer];
    v.body.push(BodySegment { station: 1.10, length: 0.4, fore_radius: 0.038, aft_radius: 0.038, kind: BodyKind::Tube, roughness: 30e-6 });
    let launch = Launch { rail_length: 2.0, ..Launch::default() };
    let rec = vec![RecoveryDevice { name: "Main".into(), stage: 1, cd_area: 0.9, deploy: Deploy::Ejection, delay: 0.0 }];
    let r = simulate(&SimInput::new(v, launch, rec));
    let kinds: Vec<_> = r.trajectory.events.iter().map(|e| e.kind.clone()).collect();
    assert!(kinds.contains(&EventKind::Separation), "{kinds:?}");
    assert_eq!(kinds.iter().filter(|k| **k == EventKind::Ignition).count(), 2);
    assert!(r.summary.apogee > 600.0, "{}", r.summary.apogee);
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
