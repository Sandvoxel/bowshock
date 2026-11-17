//! Pods (radial strap-on bodies) in the model -> simulation bridge: each pod copy is an off-axis body whose normal force and drag
//! are added to the rocket's (Barrowman 1967: component contributions add; Niskanen 2009 section 3.2).

use bowshock_model::{DeployEvent, Deployment, InternalKind, MotorRef, NoseShape, PartId, PartKind, PodBody, RadialKind, Rocket};
use bowshock_motors::MotorDb;
use bowshock_sim::from_model::{balance, fin_set_sources, recovery, sim_input, vehicle, warnings, Phase};
use bowshock_sim::{simulate, Aero, Launch, Vehicle};
use std::f64::consts::PI;

const BODY: f64 = 0.0762;
const NOSE: f64 = 0.2286;
/// Reynolds number per metre at Mach 0.3 at sea level (102 m/s over 1.46e-5 m^2/s) and at Mach 1.5.
const RE_SUBSONIC: f64 = 7.0e6;
const RE_SUPERSONIC: f64 = 3.0e7;

/// A 3" rocket: nose, two body tubes, three fins on the lower one. Returns the rocket and the upper tube.
fn core() -> (Rocket, PartId) {
    let mut r = Rocket::default();
    r.insert(0, PartKind::NoseCone { shape: NoseShape::TangentOgive, length: NOSE, diameter: BODY, tip_radius: 0.0 }).unwrap();
    let upper = r.insert(1, PartKind::BodyTube { length: 0.6, diameter: BODY }).unwrap();
    let lower = r.insert(2, PartKind::BodyTube { length: 0.3, diameter: BODY }).unwrap();
    r.attach_radial(lower, 0.12, 0.0, 3, RadialKind::FinSet { root_chord: 0.15, tip_chord: 0.07, span: 0.09, sweep: 0.07, thickness: 0.003 }).unwrap();
    (r, upper)
}

/// `core()` with two standard pods (tangent-ogive nose and a tube) on the upper tube, at the front of it. Returns the rocket and
/// the pod set's id.
fn with_pods() -> (Rocket, PartId) {
    let (mut r, upper) = core();
    let pod = r.attach_radial(upper, 0.05, 0.0, 2, RadialKind::Pod { gap: 0.0, body: PodBody::standard(BODY, 0.6) }).unwrap();
    (r, pod)
}

fn fins() -> RadialKind {
    RadialKind::FinSet { root_chord: 0.06, tip_chord: 0.03, span: 0.03, sweep: 0.02, thickness: 0.002 }
}

fn fly(r: &Rocket) -> Vehicle {
    vehicle(r, &MotorDb::bundled(), Phase::Full).unwrap()
}

/// Sum of the masses of every part of every pod (all copies).
fn pod_parts_mass(r: &Rocket) -> f64 {
    r.ids().into_iter().filter(|&id| r.pod_of(id).is_some()).map(|id| r.component_mass(id).unwrap().mass).sum()
}

#[test]
fn pods_add_drag() {
    let (plain, _) = core();
    let (podded, _) = with_pods();
    let (a, b) = (fly(&plain), fly(&podded));
    // The same reference area: the core's.
    assert!((a.reference_diameter - b.reference_diameter).abs() < 1e-12);
    assert!((a.reference_diameter - BODY).abs() < 1e-9);
    for (mach, re) in [(0.3, RE_SUBSONIC), (1.5, RE_SUPERSONIC)] {
        let (cd0, cd1) = (Aero::new(&a).drag(mach, re, false), Aero::new(&b).drag(mach, re, false));
        assert!(cd1.total > cd0.total * 1.05, "M {mach}: {} vs {}", cd1.total, cd0.total);
        // Each of the contributions a pod has grows: skin friction, nose pressure drag and the base of its own tube.
        assert!(cd1.friction > cd0.friction, "friction M {mach}");
        assert!(cd1.pressure > cd0.pressure, "pressure M {mach}");
        assert!(cd1.base > cd0.base, "base M {mach}");
        // Nothing of the core changes.
        assert!((cd1.fins - cd0.fins).abs() < 1e-12 && (cd1.protuberances - cd0.protuberances).abs() < 1e-12);
    }
}

#[test]
fn pod_drag_scales_with_copies_and_does_not_touch_the_core_base() {
    let (mut r, pod) = with_pods();
    let two = Aero::new(&fly(&r)).drag(0.3, RE_SUBSONIC, false);
    let plain = Aero::new(&fly(&core().0)).drag(0.3, RE_SUBSONIC, false);
    let kind = r.radial(pod).unwrap().kind.clone();
    assert!(r.set_radial(pod, kind, 4));
    let four = Aero::new(&fly(&r)).drag(0.3, RE_SUBSONIC, false);
    // The base drag is the core's plus one tube base per copy: linear in the number of copies, and the pod ends ahead of the core's
    // end, so the core's own base is counted once.
    let per_copy = (two.base - plain.base) / 2.0;
    assert!(per_copy > 0.0);
    assert!((four.base - plain.base - 4.0 * per_copy).abs() < 1e-9 * four.base, "{} {} {}", four.base, plain.base, per_copy);
    // With the motor on, the exhaust takes nothing from a pod's base (the core's last base only).
    let burning = Aero::new(&fly(&r)).drag(0.3, RE_SUBSONIC, true);
    assert!((burning.base - four.base).abs() < 1e-12, "no motor in the vehicle, so no exhaust to subtract");
}

#[test]
fn pod_nose_adds_barrowman_normal_force_at_its_cp() {
    let (plain, _) = core();
    let (podded, pod) = with_pods();
    let (a, b) = (fly(&plain), fly(&podded));
    let (before, after) = (Aero::new(&a).stability(0.3, 0.0), Aero::new(&b).stability(0.3, 0.0));

    // Hand calculation. A pod nose of diameter d_p adds CN_alpha = 2 A_p / A_ref (Barrowman 1967, eq. for a nose cone) at
    // x = x_pod + (L A_p - V) / A_p, V the volume of revolution of the profile (integrated here from the model's radius); the
    // tube behind it has no lift at zero angle of attack. Two copies.
    let frame = podded.pod_frame(podded.radial(pod).unwrap()).unwrap();
    let body = podded.radial(pod).unwrap().kind.pod_body().unwrap();
    let nose = body.parts()[0].kind.clone();
    let (len, dia) = (nose.length(), nose.aft_diameter());
    let area = PI * dia * dia / 4.0;
    let n = 20_000;
    let volume: f64 = (0..n)
        .map(|i| {
            let r = nose.radius_at((f64::from(i) + 0.5) * len / f64::from(n));
            PI * r * r * len / f64::from(n)
        })
        .sum();
    let nose_cp = frame.station + (len * area - volume) / area;
    let a_ref = PI * BODY * BODY / 4.0;
    let nose_cna = 2.0 * area / a_ref;

    let cna = before.cn_alpha + 2.0 * nose_cna;
    let expected_cp = (before.cn_alpha * before.cp + 2.0 * nose_cna * nose_cp) / cna;
    assert!((after.cn_alpha - cna).abs() < 0.01 * cna, "CN_alpha {} vs {cna}", after.cn_alpha);
    assert!((after.cp - expected_cp).abs() < 0.01 * expected_cp, "CP {} vs {expected_cp}", after.cp);
    // And the pod's nose is well ahead of the core's CP, so the CP moves forward.
    assert!(after.cp < before.cp);
    // The fins of the core are untouched.
    let fin_cna = |s: &bowshock_sim::Stability| s.components.iter().filter(|c| c.0.starts_with("Fin")).map(|c| c.1).sum::<f64>();
    assert!((fin_cna(&before) - fin_cna(&after)).abs() < 1e-12);
}

#[test]
fn pod_tubes_have_no_lift_at_zero_angle_but_cross_flow_lift_at_angle() {
    let (podded, _) = with_pods();
    let plain = core().0;
    let (a, b) = (Aero::new(&fly(&plain)), Aero::new(&fly(&podded)));
    // At zero angle the only difference is the two noses (a tube has no Barrowman lift): 2 x 2 A_p/A_ref x 2 copies counted above.
    let zero = b.stability(0.3, 0.0).cn_alpha - a.stability(0.3, 0.0).cn_alpha;
    let a_ref = PI * BODY * BODY / 4.0;
    let area = PI * (BODY / 4.0).powi(2);
    assert!((zero - 2.0 * 2.0 * area / a_ref).abs() < 1e-6, "{zero}");
    // At 10 degrees the pod tubes add the sim's cross-flow body lift.
    let alpha = 10f64.to_radians();
    assert!(b.stability(0.3, alpha).cn_alpha - a.stability(0.3, alpha).cn_alpha > zero + 1e-3);
}

#[test]
fn balance_includes_the_pods() {
    let db = MotorDb::bundled();
    let (plain, _) = core();
    let (podded, pod) = with_pods();
    let (b0, b1) = (balance(&plain, &db, Phase::Full, 0.3, 0.0, true).unwrap(), balance(&podded, &db, Phase::Full, 0.3, 0.0, true).unwrap());

    let pods = pod_parts_mass(&podded);
    assert!(pods > 0.02, "a pod weighs something: {pods}");
    // The pod set's own mass is its parts'.
    let total = podded.component_mass(pod).unwrap();
    assert!((total.mass - pods).abs() < 1e-9 * pods);
    assert!((b1.mass - b0.mass - pods).abs() < 1e-9, "dry mass {} -> {} but the pods weigh {pods}", b0.mass, b1.mass);

    // The CG is the mass-weighted mean of the core's and the pods' (a pod CG station is in rocket stations).
    let expected = (b0.mass * b0.cg + pods * total.cg) / (b0.mass + pods);
    assert!((b1.cg - expected).abs() < 1e-9, "{} vs {expected}", b1.cg);
    assert!((b1.cg - b0.cg) * (total.cg - b0.cg) > 0.0, "the CG moves towards the pods");

    // The same through the stage's dry mass of the vehicle.
    let v = fly(&podded);
    assert!((v.stages.iter().map(|s| s.dry.mass).sum::<f64>() - podded.dry_mass().mass).abs() < 1e-9);
}

#[test]
fn pod_parts_belong_to_the_stage_of_their_parent() {
    let (r, pod) = with_pods();
    let stage = r.stage_of(r.radial(pod).unwrap().parent).unwrap();
    let v = fly(&r);
    // One stage here: its dry mass is the whole rocket's, pods included.
    assert_eq!(v.stages.len(), 1);
    assert_eq!(stage, 1);
    assert!((v.stages[0].dry.mass - r.dry_mass().mass).abs() < 1e-9);
}

#[test]
fn pod_fins_add_normal_force_and_stability() {
    let db = MotorDb::bundled();
    // Two pods on the aft tube, so that their fins (like the core's) stand behind the CP.
    let (mut bare, _) = core();
    let lower = bare.parts()[2].id;
    let pod = bare.attach_radial(lower, 0.0, 0.0, 2, RadialKind::Pod { gap: 0.0, body: PodBody::standard(BODY, 0.3) }).unwrap();
    let mut finned = bare.clone();
    let tube = finned.radial(pod).unwrap().kind.pod_body().unwrap().parts()[1].id;
    finned.attach_radial(tube, 0.12, 0.0, 3, fins()).unwrap();

    let (a, b) = (fly(&bare), fly(&finned));
    assert_eq!(b.fin_sets.len(), a.fin_sets.len() + 1);
    let set = &b.fin_sets[b.fin_sets.len() - 1];
    let frame = finned.pod_frame(finned.radial(pod).unwrap()).unwrap();
    // 3 fins on each of 2 pod copies, on the pod's radius, at the pod's station plus the fin's station in the pod.
    assert_eq!(set.count, 6);
    assert!((set.body_radius - BODY / 4.0).abs() < 1e-9);
    assert!((set.station - (finned.part_station(tube).unwrap() + 0.12)).abs() < 1e-9);
    assert!(set.station > frame.station);
    // The core's own fin set is first and unchanged.
    assert_eq!(a.fin_sets[0], b.fin_sets[0]);
    // The sources list matches the fin sets one to one.
    assert_eq!(fin_set_sources(&finned).len(), b.fin_sets.len());

    for mach in [0.3, 1.5] {
        let (sa, sb) = (Aero::new(&a).stability(mach, 0.0), Aero::new(&b).stability(mach, 0.0));
        assert!(sb.cn_alpha > sa.cn_alpha + 0.05, "M {mach}: {} vs {}", sb.cn_alpha, sa.cn_alpha);
        assert!(sb.cp > sa.cp, "the fins at the aft end of the pod move the CP aft");
    }
    // Static margin at Mach 0.3 from the balance, whose CG includes the fins' own small mass.
    let (m0, m1) = (balance(&bare, &db, Phase::Full, 0.3, 0.0, true).unwrap(), balance(&finned, &db, Phase::Full, 0.3, 0.0, true).unwrap());
    assert!(m1.cn_alpha > m0.cn_alpha);
    assert!(m1.margin_cal > m0.margin_cal, "{} vs {}", m1.margin_cal, m0.margin_cal);
    // Fin drag comes with it.
    assert!(Aero::new(&b).drag(0.3, RE_SUBSONIC, false).fins > Aero::new(&a).drag(0.3, RE_SUBSONIC, false).fins);
}

#[test]
fn pod_launch_lugs_fly_once_per_copy() {
    let (mut r, pod) = with_pods();
    let tube = r.radial(pod).unwrap().kind.pod_body().unwrap().parts()[1].id;
    let before = fly(&r).protuberances.len();
    r.attach_radial(tube, 0.1, 0.0, 1, RadialKind::LaunchLug { length: 0.05, diameter: 0.006 }).unwrap();
    assert_eq!(fly(&r).protuberances.len(), before + 2);
}

#[test]
fn upper_stack_flight_drops_pods_of_the_stages_below() {
    // Two stages: the pod stands on the lower tube, so the upper stack alone flies without it.
    let mut r = Rocket::default();
    r.insert(0, PartKind::NoseCone { shape: NoseShape::TangentOgive, length: NOSE, diameter: BODY, tip_radius: 0.0 }).unwrap();
    let upper = r.insert(1, PartKind::BodyTube { length: 0.4, diameter: BODY }).unwrap();
    let lower = r.insert(2, PartKind::BodyTube { length: 0.5, diameter: BODY }).unwrap();
    let mount = InternalKind::InnerTube { outer_diameter: 0.041, thickness: 0.0015, length: 0.2, motor_mount: true, overhang: 0.0 };
    r.attach_internal(upper, 0.15, mount.clone()).unwrap();
    r.attach_internal(lower, 0.25, mount).unwrap();
    let pod = r.attach_radial(lower, 0.05, 0.0, 2, RadialKind::Pod { gap: 0.0, body: PodBody::standard(BODY, 0.5) }).unwrap();
    let nose_segments = |v: &Vehicle| v.body.iter().filter(|b| matches!(b.kind, bowshock_sim::BodyKind::Nose(_))).count();
    assert_eq!(r.stages().len(), 2, "two motor mounts make two stages");
    assert_eq!(r.stage_of(r.radial(pod).unwrap().parent), Some(1));
    let db = MotorDb::bundled();
    let full = vehicle(&r, &db, Phase::Full).unwrap();
    let top = vehicle(&r, &db, Phase::UpperFrom(2)).unwrap();
    assert_eq!(nose_segments(&full), 3, "the core's nose and one per pod copy");
    assert_eq!(nose_segments(&top), 1, "the pods of the first stage are gone");
}

#[test]
fn a_parachute_in_a_pod_flies_once_per_copy() {
    let (mut r, pod) = with_pods();
    let tube = r.radial(pod).unwrap().kind.pod_body().unwrap().parts()[1].id;
    let chute = InternalKind::Parachute {
        diameter: 0.3,
        cd: 0.8,
        line_count: 6,
        line_length: 0.3,
        line_material: bowshock_model::MaterialId::BRAIDED_NYLON_2MM,
        packed_length: 0.1,
        packed_diameter: None,
        deploy: Deployment { event: DeployEvent::Apogee, delay: 0.0 },
    };
    r.attach_internal(tube, 0.05, chute).unwrap();
    let devices = recovery(&r, Phase::Full);
    assert_eq!(devices.len(), 1);
    assert!((devices[0].cd_area - 2.0 * 0.8 * PI * 0.3 * 0.3 / 4.0).abs() < 1e-12);
    assert_eq!(devices[0].stage, 0);
}

#[test]
fn json_round_trip_gives_the_same_vehicle() {
    let (mut r, pod) = with_pods();
    let tube = r.radial(pod).unwrap().kind.pod_body().unwrap().parts()[1].id;
    r.attach_radial(tube, 0.3, 0.0, 3, fins()).unwrap();
    let back = Rocket::from_bytes(&r.to_bytes()).unwrap();
    assert_eq!(back.pods().count(), 1);
    let (a, b) = (fly(&r), fly(&back));
    assert_eq!(a, b);
    assert_eq!(
        balance(&r, &MotorDb::bundled(), Phase::Full, 0.3, 0.0, true).unwrap(),
        balance(&back, &MotorDb::bundled(), Phase::Full, 0.3, 0.0, true).unwrap()
    );
}

// ---- strap-on boosters: motors in pod mounts -------------------------------------------------------------------------------

fn mount(outer: f64, thickness: f64, length: f64) -> InternalKind {
    InternalKind::InnerTube { outer_diameter: outer, thickness, length, motor_mount: true, overhang: 0.0 }
}

/// The core of `core()` with a 38 mm mount (I218R) in its lower tube and two pods on its upper tube whose tubes hold a 29 mm mount
/// each (G80T per pod copy when `loaded`). Returns the rocket, the core mount and the pod mount.
fn boosted(loaded: bool) -> (Rocket, PartId, PartId) {
    let (mut r, upper) = core();
    let lower = r.parts()[2].id;
    let core_mount = r.attach_internal(lower, 0.03, mount(0.041, 0.0015, 0.25)).unwrap();
    let pod = r.attach_radial(upper, 0.05, 0.0, 2, RadialKind::Pod { gap: 0.0, body: PodBody::standard(BODY, 0.6) }).unwrap();
    let tube = r.radial(pod).unwrap().kind.pod_body().unwrap().parts()[1].id;
    let pod_mount = r.attach_internal(tube, 0.1, mount(0.031, 0.001, 0.14)).unwrap();
    let cfg = r.active_config_id();
    r.set_motor(cfg, core_mount, Some(MotorRef::new("I218R", "AeroTech"))).unwrap();
    r.set_motor_delay(cfg, core_mount, 8.0).unwrap();
    if loaded {
        r.set_motor(cfg, pod_mount, Some(MotorRef::new("G80T", "AeroTech"))).unwrap();
        r.set_motor_delay(cfg, pod_mount, 5.0).unwrap();
    }
    (r, core_mount, pod_mount)
}

fn liftoff_mass(r: &Rocket) -> f64 {
    balance(r, &MotorDb::bundled(), Phase::Full, 0.3, 0.0, true).unwrap().mass
}

/// Thrust at 0.1 s over the liftoff mass: the liftoff acceleration before gravity and drag (m/s^2).
fn liftoff_acceleration(r: &Rocket) -> f64 {
    let thrust: f64 = fly(r).stages.iter().filter_map(|s| s.motor.as_ref()).map(|m| m.curve.thrust(0.1)).sum();
    thrust / liftoff_mass(r)
}

fn apogee(r: &Rocket) -> f64 {
    let launch = Launch { wind_speed: 0.0, ..Launch::default() };
    simulate(&sim_input(r, &MotorDb::bundled(), Phase::Full, launch).unwrap()).summary.apogee
}

#[test]
fn strap_on_motors_add_thrust_mass_and_apogee() {
    let db = MotorDb::bundled();
    let (empty, ..) = boosted(false);
    let (full, ..) = boosted(true);
    let (core_motor, pod_motor) = (db.find("I218R").unwrap(), db.find("G80T").unwrap());

    // The stage's one equivalent motor: thrust summed (two copies of the pod's motor), masses summed, burn as long as the longest.
    let v = fly(&full);
    assert_eq!(v.stages.len(), 1);
    let m = v.stages[0].motor.as_ref().unwrap();
    for t in [0.05, 0.3, 0.8, 1.5] {
        let expected = core_motor.thrust(t) + 2.0 * pod_motor.thrust(t);
        assert!((m.curve.thrust(t) - expected).abs() < 1e-6 * expected.max(1.0) + 0.5, "t {t}: {} vs {expected}", m.curve.thrust(t));
    }
    assert!((m.curve.total_mass - (core_motor.total_mass + 2.0 * pod_motor.total_mass)).abs() < 1e-12);
    assert!((m.curve.propellant_mass - (core_motor.propellant_mass + 2.0 * pod_motor.propellant_mass)).abs() < 1e-12);
    assert!((m.curve.burn_time() - core_motor.burn_time().max(pod_motor.burn_time())).abs() < 1e-12);
    assert!((m.curve.total_impulse() - (core_motor.total_impulse() + 2.0 * pod_motor.total_impulse())).abs() < 0.01 * m.curve.total_impulse());
    // The core motor's size and ejection charge stay.
    assert_eq!((m.curve.length, m.curve.diameter), (core_motor.length, core_motor.diameter));
    assert_eq!(m.ejection_delay, Some(8.0));

    // Liftoff mass includes the pod propellant (and the casings): the loaded motors of both pod copies.
    let extra = liftoff_mass(&full) - liftoff_mass(&empty);
    assert!((extra - 2.0 * pod_motor.total_mass).abs() < 1e-9, "{extra}");
    let burnout = |r: &Rocket| balance(r, &db, Phase::Full, 0.3, 0.0, false).unwrap().mass;
    assert!((burnout(&full) - burnout(&empty) - 2.0 * (pod_motor.total_mass - pod_motor.propellant_mass)).abs() < 1e-9);
    let b = balance(&full, &db, Phase::Full, 0.3, 0.0, true).unwrap();
    assert!((b.motor_mass - (core_motor.total_mass + 2.0 * pod_motor.total_mass)).abs() < 1e-9);

    // The pod motors sit at their mount: ahead of the core motor, so the loaded CG moves forward.
    assert!(m.cg_station() < fly(&empty).stages[0].motor.as_ref().unwrap().cg_station());

    // More thrust per kilogram, and a higher flight.
    assert!(liftoff_acceleration(&full) > liftoff_acceleration(&empty), "{} vs {}", liftoff_acceleration(&full), liftoff_acceleration(&empty));
    let (low, high) = (apogee(&empty), apogee(&full));
    assert!(high > low * 1.1, "apogee {high} vs {low}");
}

#[test]
fn pod_motor_fit_and_thrust_to_weight_are_judged_in_their_stage() {
    let db = MotorDb::bundled();
    let (mut r, core_mount, pod_mount) = boosted(true);
    let cfg = r.active_config_id();
    let bal = balance(&r, &db, Phase::Full, 0.3, 0.0, true).unwrap();
    let notes = |r: &Rocket| warnings(r, &db, &balance(r, &db, Phase::Full, 0.3, 0.0, true).unwrap());
    let w = notes(&r);
    assert!(!w.iter().any(|w| w.message.contains("unpowered")), "{w:?}");
    assert!(!w.iter().any(|w| w.message.starts_with("Motor too")), "{w:?}");
    assert!(!w.iter().any(|w| w.message.starts_with("No motor")), "{w:?}");
    assert!(bal.mass > 0.0);

    // A 38 mm motor in the 29 mm pod mount is too wide, and the warning points at the pod's mount.
    r.set_motor(cfg, pod_mount, Some(MotorRef::new("I218R", "AeroTech"))).unwrap();
    assert!(notes(&r).iter().any(|w| w.message.starts_with("Motor too wide") && w.part == Some(pod_mount)));
    // An empty core mount is still reported, and the pod's motors do not hide it.
    r.set_motor(cfg, pod_mount, Some(MotorRef::new("G80T", "AeroTech"))).unwrap();
    r.set_motor(cfg, core_mount, None).unwrap();
    assert!(notes(&r).iter().any(|w| w.message.starts_with("No motor in") && w.part == Some(core_mount)));
}

#[test]
fn thrust_to_weight_counts_the_pod_motors() {
    let db = MotorDb::bundled();
    let flagged = |r: &Rocket| {
        let b = balance(r, &db, Phase::Full, 0.3, 0.0, true).unwrap();
        warnings(r, &db, &b).iter().any(|w| w.message.starts_with("Thrust/weight"))
    };
    // Ballast the unpowered-pod rocket to a thrust-to-weight of 4 on its core motor alone (flagged, the minimum is 5); the two
    // G80T strap-ons then lift it to above 5.
    let (mut empty, ..) = boosted(false);
    let (mut full, ..) = boosted(true);
    let core_thrust = db.find("I218R").unwrap().average_thrust();
    let ballast = core_thrust / (4.0 * bowshock_sim::G0) - liftoff_mass(&empty);
    assert!(ballast > 0.0, "{ballast}");
    for r in [&mut empty, &mut full] {
        let upper = r.parts()[1].id;
        let kind = InternalKind::MassComponent { mass: ballast, length: 0.05, diameter: None, role: bowshock_model::MassRole::Ballast };
        r.attach_internal(upper, 0.3, kind).unwrap();
    }
    assert!(flagged(&empty));
    assert!(!flagged(&full), "{} N of strap-on thrust on {} kg", 2.0 * db.find("G80T").unwrap().average_thrust(), liftoff_mass(&full));
}

#[test]
fn strap_ons_leave_with_their_stage_and_fire_alone_without_a_core_motor() {
    let db = MotorDb::bundled();
    // Two stages; the pods stand on the lower (booster) one and carry the only motors of it.
    let mut r = Rocket::default();
    r.insert(0, PartKind::NoseCone { shape: NoseShape::TangentOgive, length: NOSE, diameter: BODY, tip_radius: 0.0 }).unwrap();
    let upper = r.insert(1, PartKind::BodyTube { length: 0.4, diameter: BODY }).unwrap();
    let lower = r.insert(2, PartKind::BodyTube { length: 0.5, diameter: BODY }).unwrap();
    let sustainer = r.attach_internal(upper, 0.15, mount(0.041, 0.0015, 0.2)).unwrap();
    r.attach_internal(lower, 0.25, mount(0.041, 0.0015, 0.2)).unwrap();
    let pod = r.attach_radial(lower, 0.05, 0.0, 2, RadialKind::Pod { gap: 0.0, body: PodBody::standard(BODY, 0.5) }).unwrap();
    let tube = r.radial(pod).unwrap().kind.pod_body().unwrap().parts()[1].id;
    let strap = r.attach_internal(tube, 0.05, mount(0.031, 0.001, 0.14)).unwrap();
    let cfg = r.active_config_id();
    r.set_motor(cfg, sustainer, Some(MotorRef::new("G40W", "AeroTech"))).unwrap();
    r.set_motor(cfg, strap, Some(MotorRef::new("G80T", "AeroTech"))).unwrap();
    r.set_motor_delay(cfg, strap, 4.0).unwrap();

    let g80 = db.find("G80T").unwrap();
    let full = vehicle(&r, &db, Phase::Full).unwrap();
    assert_eq!(full.stages.len(), 2);
    // The booster stage has no core motor: its motor is the two pod motors, with the diameter of 0 that takes nothing from the
    // core's base, their ejection delay and the stage's own ignition (at launch).
    let m = full.stages[0].motor.as_ref().unwrap();
    assert!((m.curve.thrust(0.3) - 2.0 * g80.thrust(0.3)).abs() < 1e-6 * g80.thrust(0.3).max(1.0) + 0.5);
    assert!((m.curve.total_mass - 2.0 * g80.total_mass).abs() < 1e-12);
    assert_eq!((m.curve.diameter, m.ejection_delay), (0.0, Some(4.0)));
    assert_eq!(m.ignition, bowshock_sim::Timing::LAUNCH);

    // The upper stack alone leaves the pods, and their motors, behind.
    let b = balance(&r, &db, Phase::UpperFrom(2), 0.3, 0.0, true).unwrap();
    let top = vehicle(&r, &db, Phase::UpperFrom(2)).unwrap();
    assert_eq!(top.stages.len(), 1);
    assert!((b.motor_mass - db.find("G40W").unwrap().total_mass).abs() < 1e-12, "{}", b.motor_mass);
    let all = balance(&r, &db, Phase::Full, 0.3, 0.0, true).unwrap();
    assert!((all.motor_mass - (db.find("G40W").unwrap().total_mass + 2.0 * g80.total_mass)).abs() < 1e-12);
}

#[test]
fn json_round_trip_keeps_the_strap_on_flight() {
    let (r, ..) = boosted(true);
    let back = Rocket::from_bytes(&r.to_bytes()).unwrap();
    assert_eq!(fly(&r), fly(&back));
    assert_eq!(liftoff_mass(&r), liftoff_mass(&back));
    assert_eq!(apogee(&r), apogee(&back));
}
