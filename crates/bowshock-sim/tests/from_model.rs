//! The model -> simulation bridge: a realistic single-stage rocket (H128W on a 3" airframe) and a two-stage one, built through
//! the `bowshock-model` API.

use bowshock_model::{
    Ballast, DeployEvent, Deployment, EventKind as ModelEvent, InternalKind, MassRole, MotorRef, NoseShape, PartId, PartKind, RadialKind, Rocket, StageEvent,
    StageTrigger,
};
use bowshock_motors::MotorDb;
use bowshock_sim::from_model::{balance, recovery, sim_input, vehicle, warnings, BridgeError, Level, Phase};
use bowshock_sim::{simulate, BodyKind, Deploy, EventKind, Launch, NoseShape as SimNose, Protuberance, Timing, Trigger};

const BODY: f64 = 0.0762;
const NOSE: f64 = 0.2286;

fn mount(motor_od: f64, length: f64) -> InternalKind {
    InternalKind::InnerTube { outer_diameter: motor_od, thickness: 0.0015, length, motor_mount: true, overhang: 0.0 }
}

fn parachute(diameter: f64, event: DeployEvent, delay: f64) -> InternalKind {
    InternalKind::Parachute {
        diameter,
        cd: 0.75,
        line_count: 8,
        line_length: diameter,
        line_material: bowshock_model::MaterialId::BRAIDED_NYLON_2MM,
        packed_length: 0.15,
        packed_diameter: None,
        deploy: Deployment { event, delay },
    }
}

/// Nose 228.6 mm, 600 mm payload/recovery tube, 300 mm motor tube with a 38 mm mount, three fins, a 36" parachute on apogee
/// plus 1 s and an altimeter, H128W loaded.
fn single_stage() -> (Rocket, PartId) {
    let mut r = Rocket::default();
    r.insert(0, PartKind::NoseCone { shape: NoseShape::TangentOgive, length: NOSE, diameter: BODY }).unwrap();
    let upper = r.insert(1, PartKind::BodyTube { length: 0.6, diameter: BODY }).unwrap();
    let lower = r.insert(2, PartKind::BodyTube { length: 0.3, diameter: BODY }).unwrap();
    let motor_mount = r.attach_internal(lower, 0.05, mount(0.041, 0.25)).unwrap();
    let ring = InternalKind::CenteringRing { outer_diameter: None, inner_diameter: None, thickness: 0.006 };
    r.attach_internal(lower, 0.05, ring.clone()).unwrap();
    r.attach_internal(lower, 0.294, ring).unwrap();
    r.attach_internal(upper, 0.12, parachute(0.9144, DeployEvent::Apogee, 1.0)).unwrap();
    r.attach_internal(upper, 0.35, InternalKind::MassComponent { mass: 0.06, length: 0.08, diameter: None, role: MassRole::Altimeter }).unwrap();
    r.attach_internal(upper, 0.25, InternalKind::MassComponent { mass: 0.15, length: 0.06, diameter: None, role: MassRole::Battery }).unwrap();
    r.attach_internal(upper, 0.45, InternalKind::MassComponent { mass: 0.10, length: 0.06, diameter: None, role: MassRole::Tracker }).unwrap();
    r.attach_internal(upper, 0.52, InternalKind::MassComponent { mass: 0.15, length: 0.04, diameter: None, role: MassRole::Ballast }).unwrap();
    r.attach_radial(lower, 0.12, 0.0, 3, RadialKind::FinSet { root_chord: 0.15, tip_chord: 0.07, span: 0.09, sweep: 0.07, thickness: 0.003 }).unwrap();
    let cfg = r.active_config_id();
    r.set_motor(cfg, motor_mount, Some(MotorRef::new("H128W", "AeroTech"))).unwrap();
    r.set_motor_delay(cfg, motor_mount, 6.0).unwrap();
    (r, motor_mount)
}

#[test]
fn single_stage_balance_is_stable_and_light() {
    let (r, _) = single_stage();
    let db = MotorDb::bundled();
    let lift = balance(&r, &db, Phase::Full, 0.3, 0.0, true).unwrap();
    let burn = balance(&r, &db, Phase::Full, 0.3, 0.0, false).unwrap();
    assert!((1.0..1.5).contains(&lift.mass), "mass {}", lift.mass);
    assert!(lift.margin_cal > 1.0 && lift.margin_cal < 6.0, "margin {}", lift.margin_cal);
    assert!(lift.cn_alpha > 5.0 && lift.cn_alpha < 20.0, "CNa {}", lift.cn_alpha);
    assert!((lift.length - (NOSE + 0.9)).abs() < 1e-9 && (lift.reference_diameter - BODY).abs() < 1e-12);
    // H128W: 201.6 g loaded, 94.1 g of propellant (the catalogue stores f32: about 7 significant digits).
    assert!((lift.motor_mass - 0.2016).abs() < 1e-6);
    assert!((burn.motor_mass - (0.2016 - 0.09408)).abs() < 1e-6);
    assert!(lift.cg > burn.cg, "burning propellant moves the CG forward: {} vs {}", lift.cg, burn.cg);
    assert!((lift.mass - burn.mass - 0.09408).abs() < 1e-6);
    // The CP is a property of the airframe alone.
    assert_eq!(lift.cp, burn.cp);
    assert!(lift.cp > lift.cg && lift.cg > 0.4 && lift.cp < lift.length);
    // The dry mass of the model is what the stage carries.
    let v = vehicle(&r, &db, Phase::Full).unwrap();
    assert_eq!(v.stages.len(), 1);
    assert!((v.stages[0].dry.mass - r.dry_mass().mass).abs() < 1e-9);
    assert!((v.stages[0].dry.cg - r.dry_mass().cg).abs() < 1e-9);
}

#[test]
fn single_stage_vehicle_maps_geometry() {
    let (r, mount_id) = single_stage();
    let db = MotorDb::bundled();
    let v = vehicle(&r, &db, Phase::Full).unwrap();
    assert_eq!(v.body.len(), 3);
    assert_eq!(v.body[0].kind, BodyKind::Nose(SimNose::TangentOgive));
    assert_eq!((v.body[0].station, v.body[0].length), (0.0, NOSE));
    assert!((v.body[2].station - (NOSE + 0.6)).abs() < 1e-12 && v.body[2].kind == BodyKind::Tube);
    assert_eq!(v.body[1].roughness, 60e-6, "regular paint");
    let f = &v.fin_sets[0];
    assert_eq!(f.count, 3);
    assert!((f.station - (NOSE + 0.6 + 0.12)).abs() < 1e-12 && (f.body_radius - BODY / 2.0).abs() < 1e-12);
    assert_eq!((f.root_chord, f.tip_chord, f.span, f.sweep), (0.15, 0.07, 0.09, 0.07));
    // Motor: the mount's aft end is at the aft end of the airframe, the slot's delay is the ejection delay.
    let m = v.stages[0].motor.as_ref().unwrap();
    assert!((m.mount_aft_station - (NOSE + 0.9)).abs() < 1e-9);
    assert_eq!((m.ignition, m.ejection_delay), (Timing::LAUNCH, Some(6.0)));
    assert!((m.curve.diameter - 0.029).abs() < 1e-9 && m.curve.points[0] == (0.0, 0.0));
    assert!((m.curve.total_impulse() - 172.9).abs() < 8.0, "{}", m.curve.total_impulse());

    // Recovery: 36" at Cd 0.75, apogee + 1 s, named after the part.
    let rec = recovery(&r, Phase::Full);
    assert_eq!(rec.len(), 1);
    assert_eq!((rec[0].name.as_str(), rec[0].stage, rec[0].deploy, rec[0].delay), ("Parachute", 0, Deploy::Apogee, 1.0));
    assert!((rec[0].cd_area - 0.75 * std::f64::consts::PI * 0.9144 * 0.9144 / 4.0).abs() < 1e-12);

    // Plugged motor: no ejection charge.
    let mut plugged = r.clone();
    let cfg = plugged.active_config_id();
    plugged.set_motor_delay(cfg, mount_id, f64::INFINITY).unwrap();
    assert_eq!(vehicle(&plugged, &db, Phase::Full).unwrap().stages[0].motor.as_ref().unwrap().ejection_delay, None);
}

#[test]
fn single_stage_flies_to_a_plausible_apogee() {
    let (r, _) = single_stage();
    let db = MotorDb::bundled();
    let input = sim_input(&r, &db, Phase::Full, Launch::default()).unwrap();
    let result = simulate(&input);
    let s = &result.summary;
    // ~1.2 kg on an H128W (173 N s) reaches roughly 300-700 m.
    assert!((250.0..800.0).contains(&s.apogee), "apogee {}", s.apogee);
    assert!(s.rail_exit_velocity > 12.0, "rail exit {}", s.rail_exit_velocity);
    assert!(s.stability_off_rail_cal > 1.0, "margin {}", s.stability_off_rail_cal);
    assert!(s.ground_hit_velocity < 8.0, "hit {}", s.ground_hit_velocity);
    let kinds: Vec<&EventKind> = result.trajectory.events.iter().map(|e| &e.kind).collect();
    assert!(kinds.contains(&&EventKind::Deploy("Parachute".into())) && kinds.contains(&&EventKind::Landing));
}

#[test]
fn single_stage_has_no_blocking_warnings() {
    let (r, _) = single_stage();
    let db = MotorDb::bundled();
    let b = balance(&r, &db, Phase::Full, 0.3, 0.0, true).unwrap();
    let w = warnings(&r, &db, &b);
    assert!(w.iter().all(|w| w.level != Level::Err), "{w:?}");
    assert!(w.iter().any(|w| w.level == Level::Ok && w.message.starts_with("Stable · ")), "{w:?}");
}

#[test]
fn warnings_report_what_is_wrong() {
    let (mut r, mount_id) = single_stage();
    let db = MotorDb::bundled();
    let cfg = r.active_config_id();
    let b = balance(&r, &db, Phase::Full, 0.3, 0.0, true).unwrap();

    // Unstable and under-stable readings use the design voice.
    let low = bowshock_sim::from_model::Balance { margin_cal: 0.74, ..b };
    assert!(warnings(&r, &db, &low).iter().any(|w| w.level == Level::Warn && w.message == "Marginal · 0.74 cal (min 1.0)"));
    let neg = bowshock_sim::from_model::Balance { margin_cal: -0.3, ..b };
    assert!(warnings(&r, &db, &neg).iter().any(|w| w.level == Level::Err && w.message == "Unstable · \u{2212}0.30 cal (min 1.0)"));

    // A motor longer than its mount, then wider than it, then an unknown one.
    r.set_motor(cfg, mount_id, Some(MotorRef::new("J420R", "AeroTech"))).unwrap();
    let w = warnings(&r, &db, &b);
    let long = w.iter().find(|w| w.message.contains("long")).unwrap_or_else(|| panic!("{w:?}"));
    assert_eq!((long.level, long.part, long.message.as_str()), (Level::Warn, Some(mount_id), "Motor too long · 337.0 mm (mount 250.0)"));
    r.set_internal_kind(mount_id, mount(0.0301, 0.25));
    let w = warnings(&r, &db, &b);
    let wide = w.iter().find(|w| w.message.contains("wide")).unwrap_or_else(|| panic!("{w:?}"));
    assert_eq!((wide.level, wide.part), (Level::Err, Some(mount_id)));
    r.set_motor(cfg, mount_id, Some(MotorRef::new("Z999", "Nobody"))).unwrap();
    assert!(warnings(&r, &db, &b).iter().any(|w| w.level == Level::Err && w.message.contains("Z999")));
    assert_eq!(vehicle(&r, &db, Phase::Full), Err(BridgeError::UnknownMotor("Z999".into())));

    // No motor at all.
    r.set_motor(cfg, mount_id, None).unwrap();
    let w = warnings(&r, &db, &b);
    assert!(w.iter().any(|w| w.level == Level::Warn && w.message == "No motor in the current configuration"));
    assert_eq!(vehicle(&r, &db, Phase::Full).unwrap().stages[0].motor, None);

    // A parachute that waits for an ejection charge the plugged motor lacks.
    r.set_motor(cfg, mount_id, Some(MotorRef::new("H128W", "AeroTech"))).unwrap();
    let upper = r.parts()[1].id;
    let chute = r.attach_internal(upper, 0.5, parachute(0.3, DeployEvent::Ejection, 0.0)).unwrap();
    r.set_motor_delay(cfg, mount_id, f64::INFINITY).unwrap();
    let w = warnings(&r, &db, &b);
    assert!(w.iter().any(|w| w.part == Some(chute) && w.message.contains("ejection")), "{w:?}");
}

/// Two curves with the same designation and maker (the catalogue's H128W and a user's file of a hotter H128W) are two motors:
/// a configuration names one by id, flies that one, and keeps naming it through a save and load.
#[test]
fn same_designation_curves_are_chosen_and_flown_distinctly() {
    use bowshock_sim::from_model::{find_motor, motor_ref};
    let (mut r, mount_id) = single_stage();
    let mut db = MotorDb::bundled();
    let catalogue = db.find("H128W").unwrap().clone();
    let mut hotter = catalogue.clone();
    hotter.thrustcurve = None;
    hotter.points.iter_mut().for_each(|p| p.1 *= 1.1);
    hotter.id = hotter.file_id();
    db.upsert(hotter.clone());
    assert_eq!(db.motors.iter().filter(|m| m.designation == "H128W" && m.manufacturer == "AeroTech").count(), 2);

    let cfg = r.active_config_id();
    let mut fly = |motor: &bowshock_motors::Motor| {
        r.set_motor(cfg, mount_id, Some(motor_ref(motor))).unwrap();
        let back = Rocket::from_bytes(&r.to_bytes()).unwrap();
        let reference = back.active_configuration().slot(mount_id).unwrap().motor.clone().unwrap();
        assert_eq!(find_motor(&db, &reference).unwrap().id, motor.id, "the saved reference names the same curve");
        let input = sim_input(&back, &db, Phase::Full, Launch::default()).unwrap();
        (input.vehicle.stages[0].motor.as_ref().unwrap().curve.points.clone(), simulate(&input).summary.apogee)
    };
    let (curve_a, apogee_a) = fly(&catalogue);
    let (curve_b, apogee_b) = fly(&hotter);
    assert_ne!(curve_a, curve_b);
    assert!(apogee_b > apogee_a * 1.05, "the hotter curve flies higher: {apogee_b} vs {apogee_a}");
    // A reference by name alone (an imported document) still finds a motor: the first, the catalogue's.
    assert_eq!(find_motor(&db, &MotorRef::new("H128W", "AeroTech")).unwrap().id, catalogue.id);
}

#[test]
fn json_round_trip_gives_identical_balance() {
    let (r, _) = single_stage();
    let db = MotorDb::bundled();
    let back = Rocket::from_bytes(&r.to_bytes()).unwrap();
    for with_motor in [true, false] {
        let a = balance(&r, &db, Phase::Full, 0.3, 0.0, with_motor).unwrap();
        let b = balance(&back, &db, Phase::Full, 0.3, 0.0, with_motor).unwrap();
        assert_eq!(a, b);
    }
    assert_eq!(vehicle(&r, &db, Phase::Full), vehicle(&back, &db, Phase::Full));
    assert_eq!(recovery(&r, Phase::Full), recovery(&back, Phase::Full));
}

struct TwoStage {
    rocket: Rocket,
    booster_mount: PartId,
    sustainer_mount: PartId,
    booster_tube: PartId,
}

/// Nose + 600 mm sustainer tube (G40W) + 500 mm booster (H128W, fins), 29 mm mounts; the sustainer lights 0.5 s after the
/// booster separates, which itself happens 0.25 s after the booster's burnout. A streamer rides in the booster, a parachute in
/// the sustainer.
fn two_stage() -> TwoStage {
    let mut r = Rocket::default();
    r.insert(0, PartKind::NoseCone { shape: NoseShape::VonKarman, length: NOSE, diameter: BODY }).unwrap();
    let sustainer = r.insert(1, PartKind::BodyTube { length: 0.6, diameter: BODY }).unwrap();
    let booster = r.insert(2, PartKind::BodyTube { length: 0.5, diameter: BODY }).unwrap();
    let sustainer_mount = r.attach_internal(sustainer, 0.3, mount(0.032, 0.25)).unwrap();
    let booster_mount = r.attach_internal(booster, 0.2, mount(0.032, 0.25)).unwrap();
    r.attach_internal(sustainer, 0.05, parachute(0.6, DeployEvent::Ejection, 0.0)).unwrap();
    r.attach_internal(
        booster,
        0.02,
        InternalKind::Streamer { length: 0.6, width: 0.05, cd: 0.6, deploy: Deployment { event: DeployEvent::Altitude(100.0), delay: 0.5 } },
    )
    .unwrap();
    r.attach_radial(booster, 0.3, 0.0, 4, RadialKind::FinSet { root_chord: 0.15, tip_chord: 0.06, span: 0.08, sweep: 0.08, thickness: 0.003 }).unwrap();
    r.attach_radial(booster, 0.1, 0.5, 1, RadialKind::RailButton { height: 0.01, diameter: 0.01 }).unwrap();
    let cfg = r.active_config_id();
    r.set_motor(cfg, booster_mount, Some(MotorRef::new("H128W", "AeroTech"))).unwrap();
    r.set_motor(cfg, sustainer_mount, Some(MotorRef::new("G40W", "AeroTech"))).unwrap();
    r.set_motor_delay(cfg, booster_mount, f64::INFINITY).unwrap();
    r.set_motor_delay(cfg, sustainer_mount, 9.0).unwrap();
    let event = |stage, kind, trigger, delay| StageEvent { stage, kind, trigger, delay };
    r.set_stage_event(cfg, event(2, ModelEvent::Ignition, StageTrigger::Separation, 0.5)).unwrap();
    r.set_stage_event(cfg, event(1, ModelEvent::Separation, StageTrigger::Burnout, 0.25)).unwrap();
    TwoStage { rocket: r, booster_mount, sustainer_mount, booster_tube: booster }
}

#[test]
fn two_stage_maps_to_two_sim_stages_with_ignition_and_phases() {
    let TwoStage { rocket: r, .. } = two_stage();
    let db = MotorDb::bundled();
    assert!(r.stage_issues().is_empty());
    let v = vehicle(&r, &db, Phase::Full).unwrap();
    assert_eq!(v.stages.len(), 2);
    let (booster, sustainer) = (&v.stages[0], &v.stages[1]);
    let joint = NOSE + 0.6;
    assert!((booster.fore_station - joint).abs() < 1e-9 && (booster.aft_station - (joint + 0.5)).abs() < 1e-9);
    assert!((sustainer.fore_station, sustainer.aft_station) == (0.0, booster.fore_station));
    let (bm, sm) = (booster.motor.as_ref().unwrap(), sustainer.motor.as_ref().unwrap());
    assert_eq!((bm.ignition, bm.ejection_delay), (Timing::LAUNCH, None));
    assert_eq!(sm.ignition, Timing::new(Trigger::Separation, 0.5));
    assert_eq!((booster.separation, sustainer.separation.trigger), (Timing::new(Trigger::Burnout, 0.25), Trigger::Burnout));
    assert_eq!(sm.ejection_delay, Some(9.0));
    assert!((bm.curve.total_mass - 0.2016).abs() < 1e-6 && (sm.curve.total_mass - 0.123).abs() < 1e-6);
    assert!((booster.dry.mass + sustainer.dry.mass - r.dry_mass().mass).abs() < 1e-9);
    assert!(booster.dry.cg > joint && sustainer.dry.cg < joint);
    assert_eq!(v.fin_sets.len(), 1);
    assert_eq!(v.protuberances.len(), 1);
    assert!(matches!(v.protuberances[0], Protuberance::RailButton { .. }));

    // Recovery stage indices: the streamer is in the booster (0), the parachute in the sustainer (1).
    let rec = recovery(&r, Phase::Full);
    let by_name = |n: &str| rec.iter().find(|d| d.name == n).unwrap_or_else(|| panic!("{n} in {rec:?}"));
    assert_eq!((by_name("Streamer").stage, by_name("Streamer").deploy, by_name("Streamer").delay), (0, Deploy::Altitude(100.0), 0.5));
    assert_eq!((by_name("Parachute").stage, by_name("Parachute").deploy), (1, Deploy::Ejection));
    assert!((by_name("Streamer").cd_area - 0.6 * 0.6 * 0.05).abs() < 1e-12);

    // Sustainer alone: the lower stage and everything aft of the joint is gone, the motor lights on the pad.
    assert_eq!(Phase::UpperFrom(1).first_stage(), 1);
    assert_eq!(vehicle(&r, &db, Phase::UpperFrom(1)), Ok(v.clone()));
    let up = vehicle(&r, &db, Phase::UpperFrom(2)).unwrap();
    assert_eq!(up.stages.len(), 1);
    assert_eq!(up.stages[0].motor.as_ref().unwrap().ignition, Timing::LAUNCH);
    assert!((up.length() - joint).abs() < 1e-9);
    assert!(up.fin_sets.is_empty() && up.protuberances.is_empty());
    assert_eq!(up.body.len(), 2);
    assert_eq!(up.stages[0].dry, sustainer.dry);
    let rec_up = recovery(&r, Phase::UpperFrom(2));
    assert_eq!(rec_up.len(), 1);
    assert_eq!((rec_up[0].name.as_str(), rec_up[0].stage), ("Parachute", 0));
    assert_eq!(vehicle(&r, &db, Phase::UpperFrom(3)), Err(BridgeError::NoSuchStage(3)));
    assert!(recovery(&r, Phase::UpperFrom(3)).is_empty());

    // Balance of the upper stack is lighter and shorter than the whole.
    let full = balance(&r, &db, Phase::Full, 0.3, 0.0, true).unwrap();
    let upper = balance(&r, &db, Phase::UpperFrom(2), 0.3, 0.0, true).unwrap();
    assert!(upper.mass < full.mass && upper.length < full.length);
    assert!((upper.motor_mass - 0.123).abs() < 1e-6 && (full.motor_mass - (0.123 + 0.2016)).abs() < 1e-6);
    assert!(full.margin_cal > 1.0, "full margin {}", full.margin_cal);

    // Flights: the sim stages the rocket (booster, then sustainer, two ignitions and a separation in order).
    let flight = simulate(&sim_input(&r, &db, Phase::Full, Launch::default()).unwrap());
    let ev = &flight.trajectory.events;
    let t_of =
        |kind: EventKind, stage: usize| ev.iter().find(|e| e.kind == kind && e.stage == Some(stage)).unwrap_or_else(|| panic!("{kind:?} {stage}: {ev:?}")).t;
    let (burnout0, sep0, ignite1) = (t_of(EventKind::Burnout, 0), t_of(EventKind::Separation, 0), t_of(EventKind::Ignition, 1));
    // The separation is 0.25 s after the burnout; the sustainer lights 0.5 s after the separation (not at it).
    assert!((sep0 - burnout0 - 0.25).abs() < 1e-6 && (ignite1 - sep0 - 0.5).abs() < 1e-6, "{burnout0} {sep0} {ignite1}");
    let single = simulate(&sim_input(&r, &db, Phase::UpperFrom(2), Launch::default()).unwrap());
    assert!(flight.summary.apogee > single.summary.apogee, "{} vs {}", flight.summary.apogee, single.summary.apogee);
    assert!(single.trajectory.events.iter().any(|e| e.kind == EventKind::Landing));
}

#[test]
fn every_trigger_kind_maps_to_the_sim_and_flies() {
    let TwoStage { rocket: mut r, .. } = two_stage();
    let db = MotorDb::bundled();
    let cfg = r.active_config_id();
    let event = |stage, kind, trigger, delay| StageEvent { stage, kind, trigger, delay };
    let cases =
        [(StageTrigger::Burnout, Trigger::Burnout), (StageTrigger::TimeFromLaunch, Trigger::Launch), (StageTrigger::Altitude(120.0), Trigger::Altitude(120.0))];
    for (model, sim) in cases {
        r.set_stage_event(cfg, event(1, ModelEvent::Separation, model, 0.3)).unwrap();
        r.set_stage_event(cfg, event(2, ModelEvent::Ignition, model, 0.4)).unwrap();
        let v = vehicle(&r, &db, Phase::Full).unwrap();
        assert_eq!(v.stages[0].separation, Timing::new(sim, 0.3), "{model:?}");
        assert_eq!(v.stages[1].motor.as_ref().unwrap().ignition, Timing::new(sim, 0.4), "{model:?}");
        let flight = simulate(&sim_input(&r, &db, Phase::Full, Launch::default()).unwrap());
        let kinds: Vec<_> = flight.trajectory.events.iter().map(|e| e.kind.clone()).collect();
        assert!(kinds.contains(&EventKind::Separation) && kinds.iter().filter(|k| **k == EventKind::Ignition).count() == 2, "{model:?}: {kinds:?}");
    }
}

#[test]
fn recovery_timing_and_ballast_come_from_the_active_configuration() {
    let TwoStage { rocket: mut r, .. } = two_stage();
    let db = MotorDb::bundled();
    let chute = r.internals().iter().find(|p| matches!(p.kind, InternalKind::Parachute { .. })).unwrap().id;
    let first = r.active_config_id();
    let second = r.duplicate_configuration(first).unwrap();
    r.set_deployment(second, chute, Deployment { event: DeployEvent::TimeFromLaunch, delay: 7.0 }).unwrap();
    let deploy_of = |r: &Rocket| recovery(r, Phase::Full).into_iter().find(|d| d.name == "Parachute").map(|d| (d.deploy, d.delay)).unwrap();
    assert_eq!(deploy_of(&r), (Deploy::Ejection, 0.0), "the part's own deployment in the first configuration");
    r.select_configuration(second);
    assert_eq!(deploy_of(&r), (Deploy::Launch, 7.0));

    // Ballast is dry mass of the stage holding its station: it moves the CG toward it and counts in the mass.
    let before = balance(&r, &db, Phase::Full, 0.3, 0.0, true).unwrap();
    let tail = r.length() - 0.05;
    r.set_ballast(second, Some(Ballast { mass: 0.1, station: tail })).unwrap();
    let after = balance(&r, &db, Phase::Full, 0.3, 0.0, true).unwrap();
    assert!((after.mass - before.mass - 0.1).abs() < 1e-9);
    assert!(after.cg > before.cg, "ballast at the tail moves the CG aft: {} -> {}", before.cg, after.cg);
    r.select_configuration(first);
    let plain = balance(&r, &db, Phase::Full, 0.3, 0.0, true).unwrap();
    assert!((plain.mass - before.mass).abs() < 1e-9, "the first configuration carries no ballast");
    assert!((r.dry_mass().mass - plain.mass + plain.motor_mass).abs() < 1e-9);
}

#[test]
fn staging_problems_and_clusters_are_warned_about() {
    let TwoStage { rocket: mut r, booster_tube, sustainer_mount, booster_mount } = two_stage();
    let db = MotorDb::bundled();
    let b = balance(&r, &db, Phase::Full, 0.3, 0.0, true).unwrap();
    assert!(warnings(&r, &db, &b).iter().all(|w| w.level != Level::Err));

    // The upper motor sticks out past the joint: no joint aft of it.
    r.set_internal_kind(sustainer_mount, InternalKind::InnerTube { outer_diameter: 0.032, thickness: 0.0015, length: 0.25, motor_mount: true, overhang: 0.1 });
    let issues = r.stage_issues();
    assert_eq!(issues.len(), 1);
    let w = warnings(&r, &db, &b);
    let staging = w.iter().find(|w| w.message.starts_with("No joint aft of the upper motor: split Body tube at ")).unwrap_or_else(|| panic!("{w:?}"));
    assert_eq!((staging.level, staging.part), (Level::Err, Some(booster_tube)));
    assert!(staging.message.ends_with(" mm"));
    // The sim still builds (the part boundary is wrong, the maths is fine).
    assert!(vehicle(&r, &db, Phase::Full).is_ok());

    // A stage with no motor.
    let cfg = r.active_config_id();
    r.set_motor(cfg, booster_mount, None).unwrap();
    assert!(warnings(&r, &db, &b).iter().any(|w| w.message == "No motor in Stage 1" && w.part == Some(booster_mount)));
}

#[test]
fn freeform_fin_becomes_the_equivalent_trapezoid() {
    let mut r = Rocket::default();
    r.insert(0, PartKind::NoseCone { shape: NoseShape::Haack(0.0), length: NOSE, diameter: BODY }).unwrap();
    let tube = r.insert(1, PartKind::BodyTube { length: 0.5, diameter: BODY }).unwrap();
    // A trapezoid written as a polygon: root 0.15, tip 0.07 swept 0.06, span 0.09.
    let outline = vec![(0.0, 0.0), (0.06, 0.09), (0.13, 0.09), (0.15, 0.0)];
    r.attach_radial(tube, 0.3, 0.0, 3, RadialKind::FreeformFin { points: outline, thickness: 0.003 }).unwrap();
    r.attach_radial(tube, 0.3, 0.0, 3, RadialKind::EllipticalFin { root_chord: 0.15, span: 0.09, thickness: 0.003 }).unwrap();
    r.attach_radial(tube, 0.05, 0.0, 2, RadialKind::LaunchLug { length: 0.05, diameter: 0.008 }).unwrap();
    let db = MotorDb::bundled();
    let v = vehicle(&r, &db, Phase::Full).unwrap();
    // Haack(0) is the Von Karman shape.
    assert_eq!(v.body[0].kind, BodyKind::Nose(SimNose::VonKarman));
    assert_eq!(v.fin_sets.len(), 2);
    let f = &v.fin_sets[0];
    assert!((f.root_chord - 0.15).abs() < 1e-9 && (f.tip_chord - 0.07).abs() < 1e-9 && (f.span - 0.09).abs() < 1e-9 && (f.sweep - 0.06).abs() < 1e-9, "{f:?}");
    assert!((f.station - (NOSE + 0.3)).abs() < 1e-12);
    assert_eq!(v.fin_sets[1].planform, bowshock_sim::FinPlanform::Elliptical);
    assert_eq!(v.protuberances.len(), 2);
    // A polygon that cannot be a trapezoid on its root (a big triangle on a small root) keeps area and span.
    let mut q = Rocket::default();
    let t = q.insert(0, PartKind::BodyTube { length: 0.5, diameter: BODY }).unwrap();
    let kite = vec![(0.05, 0.0), (0.0, 0.06), (0.08, 0.12), (0.1, 0.06), (0.1, 0.0)];
    let kind = RadialKind::FreeformFin { points: kite, thickness: 0.003 };
    let area = kind.fin_area().unwrap();
    q.attach_radial(t, 0.1, 0.0, 3, kind).unwrap();
    let f = &vehicle(&q, &db, Phase::Full).unwrap().fin_sets[0];
    assert!((f.fin_area() - area).abs() < 1e-9 && (f.span - 0.12).abs() < 1e-9, "{f:?}");
    // Parametric noses fall back to the closest of the sim's shapes.
    let mut n = Rocket::default();
    n.insert(0, PartKind::NoseCone { shape: NoseShape::PowerSeries(1.0), length: NOSE, diameter: BODY }).unwrap();
    assert_eq!(vehicle(&n, &db, Phase::Full).unwrap().body[0].kind, BodyKind::Nose(SimNose::Conical));
    assert_eq!(vehicle(&Rocket::default(), &db, Phase::Full), Err(BridgeError::EmptyRocket));
}

#[test]
fn a_canted_fin_of_a_group_flies_as_its_own_fin_set() {
    use bowshock_model::{Dim, RadialField};
    let (mut r, _) = single_stage();
    let fins = r.radials()[0].id;
    let db = MotorDb::bundled();
    let before = vehicle(&r, &db, Phase::Full).unwrap();
    assert_eq!((before.fin_sets.len(), before.fin_sets[0].count), (1, 3));
    // Unlinking changes nothing until the copy's value differs: the three copies still fly as one set.
    assert!(r.unlink_field(fins, 1, RadialField::Cant));
    assert_eq!(vehicle(&r, &db, Phase::Full).unwrap().fin_sets, before.fin_sets);
    r.set_instance_dim(fins, 1, Dim::Cant, 0.035).unwrap();
    let split = vehicle(&r, &db, Phase::Full).unwrap();
    let mut sets: Vec<(u32, f64)> = split.fin_sets.iter().map(|f| (f.count, f.cant)).collect();
    sets.sort_by_key(|s| std::cmp::Reverse(s.0));
    assert_eq!(sets, [(2, 0.0), (1, 0.035)]);
    // Geometry and station are the group's: the sets differ only in cant, and the three fins are all still there.
    assert!(split.fin_sets.iter().all(|f| f.root_chord == before.fin_sets[0].root_chord && f.station == before.fin_sets[0].station));
    assert_eq!(split.fin_sets.iter().map(|f| f.count).sum::<u32>(), 3);
    // A taller fin C is its own set too, with its own span.
    r.unlink_field(fins, 2, RadialField::Span);
    r.set_instance_dim(fins, 2, Dim::Span, 0.12).unwrap();
    let spans: Vec<f64> = vehicle(&r, &db, Phase::Full).unwrap().fin_sets.iter().map(|f| f.span).collect();
    assert_eq!(spans.len(), 3);
    assert!(spans.contains(&0.12));
}

#[test]
fn stability_warnings_follow_the_documents_thresholds() {
    let (mut r, mount_id) = single_stage();
    let db = MotorDb::bundled();
    let b = balance(&r, &db, Phase::Full, 0.3, 0.0, true).unwrap();
    let at = |margin_cal: f64, r: &bowshock_model::Rocket| {
        warnings(r, &db, &bowshock_sim::from_model::Balance { margin_cal, ..b }).into_iter().find(|w| w.message.contains("cal ("))
    };

    // Defaults: unstable below 0.5, marginal below 1.0, over-stable above 3.0 (no longer the old 6.0).
    assert_eq!(at(0.4, &r).unwrap().level, Level::Err);
    assert_eq!(at(0.74, &r).unwrap().level, Level::Warn);
    assert_eq!(at(2.0, &r).unwrap().level, Level::Ok);
    let over = at(3.5, &r).unwrap();
    assert_eq!((over.level, over.message.as_str()), (Level::Warn, "Over-stable · 3.5 cal (max 3.0)"));

    // A stricter document: marginal until 1.5 calibers, over-stable above 2.0.
    r.set_stability_bands(bowshock_model::StabilityBands { marginal: 0.8, stable: 1.5, over: 2.0 });
    assert_eq!(at(0.7, &r).unwrap().level, Level::Err);
    assert_eq!(at(1.2, &r).unwrap().message, "Marginal · 1.20 cal (min 1.5)");
    assert_eq!(at(2.5, &r).unwrap().message, "Over-stable · 2.5 cal (max 2.0)");

    // Unstable and marginal readings point at the fin span; motor misfits at the mount's dimension.
    let fins = at(1.2, &r).unwrap();
    assert!(fins.part.is_some() && fins.field == Some(bowshock_model::Dim::Span), "{fins:?}");
    let cfg = r.active_config_id();
    r.set_motor(cfg, mount_id, Some(MotorRef::new("J420R", "AeroTech"))).unwrap();
    let w = warnings(&r, &db, &b);
    let long = w.iter().find(|w| w.message.contains("too long")).unwrap();
    assert_eq!((long.part, long.field), (Some(mount_id), Some(bowshock_model::Dim::Length)));
}

/// Fin roll angles and the free-form fin's mid-chord sweep and root reach the simulation: a 2-fin free-form set turned
/// 90 deg from +Y carries its two angles, its body, its span-averaged mid-chord cosine and its real root chord.
#[test]
fn fin_angles_and_free_form_mid_chord_reach_the_vehicle() {
    let (mut r, _) = single_stage();
    let lower = r.parts()[2].id;
    // Leading edge 0 -> 0.10 and trailing edge 0.16 -> 0.14 over span 0.05: the mid-chord line x = 0.08 + 0.8 y is
    // straight, so its cosine is 1/sqrt(1 + 0.8^2) everywhere.
    let points = vec![(0.0, 0.0), (0.10, 0.05), (0.14, 0.05), (0.16, 0.0)];
    r.attach_radial(lower, 0.0, std::f64::consts::FRAC_PI_2, 2, RadialKind::FreeformFin { points, thickness: 0.003 }).unwrap();
    let v = vehicle(&r, &MotorDb::bundled(), Phase::Full).unwrap();
    let free = v.fin_sets.iter().find(|f| f.count == 2).unwrap();
    assert_eq!(free.angles.len(), 2);
    assert!((free.angles[0] - std::f64::consts::FRAC_PI_2).abs() < 1e-12 && (free.angles[1] - 1.5 * std::f64::consts::PI).abs() < 1e-12);
    assert!((free.mid_chord_cos.unwrap() - 1.0 / 1.64f64.sqrt()).abs() < 1e-9);
    let (a, b) = free.root_extent.unwrap();
    assert!((b - a - 0.16).abs() < 1e-12 && a <= free.station + 1e-12);
    assert_eq!(free.bodies, 1);
    let three = v.fin_sets.iter().find(|f| f.count == 3).unwrap();
    assert_eq!((three.parent, three.mid_chord_cos, three.root_extent), (free.parent, None, None));
}
