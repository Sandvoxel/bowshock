//! Open pods flown as tube fins (Ribner 1947; Niskanen 2009 section 3.1.1): the model
//! -> simulation classification and the numbers against a hand calculation of the tube fin formulas.

use bowshock_model::{InternalKind, NoseShape, Part, PartId, PartKind, PodBody, RadialKind, Rocket};
use bowshock_motors::MotorDb;
use bowshock_sim::from_model::{balance, vehicle, warnings, Phase};
use bowshock_sim::{Aero, Vehicle};
use std::f64::consts::PI;

const BODY: f64 = 0.0762;
const NOSE: f64 = 0.2286;
/// Tube fin: 25.4 mm outer diameter, 1 mm wall, 0.15 m long.
const TUBE_D: f64 = 0.0254;
const TUBE_L: f64 = 0.15;
/// Mach 0.3 at sea level: V = 102 m/s, nu = 1.4607e-5 m^2/s.
const RE_PER_M: f64 = 102.0 / 1.4607e-5;

/// A 3" rocket (nose, two tubes, three fins on the lower one), the lower tube's id.
fn core() -> (Rocket, PartId) {
    let mut r = Rocket::default();
    r.insert(0, PartKind::NoseCone { shape: NoseShape::TangentOgive, length: NOSE, diameter: BODY, tip_radius: 0.0 }).unwrap();
    r.insert(1, PartKind::BodyTube { length: 0.6, diameter: BODY }).unwrap();
    let lower = r.insert(2, PartKind::BodyTube { length: 0.3, diameter: BODY }).unwrap();
    r.attach_radial(lower, 0.12, 0.0, 3, RadialKind::FinSet { root_chord: 0.15, tip_chord: 0.07, span: 0.09, sweep: 0.07, thickness: 0.003 }).unwrap();
    (r, lower)
}

fn tube() -> PartKind {
    PartKind::BodyTube { length: TUBE_L, diameter: TUBE_D }
}

/// `core()` with `count` empty tubes standing on the lower tube, 0.12 m from its fore end, wall 1 mm. Returns the pod's id.
fn with_tubes(count: u8) -> (Rocket, PartId) {
    let (mut r, lower) = core();
    let body = PodBody::single(Part::new(0, tube()), vec![], vec![]);
    let pod = r.attach_radial(lower, 0.12, 0.0, count, RadialKind::Pod { gap: 0.0, body }).unwrap();
    let part = r.radial(pod).unwrap().kind.pod_body().unwrap().parts()[0].id;
    assert!(r.set_thickness(part, 0.001));
    (r, pod)
}

fn fly(r: &Rocket) -> Vehicle {
    vehicle(r, &MotorDb::bundled(), Phase::Full).unwrap()
}

fn a_ref() -> f64 {
    PI * BODY * BODY / 4.0
}

#[test]
fn an_empty_pod_tube_is_a_tube_fin_set_and_leaves_the_body_alone() {
    let (plain, _) = core();
    let (r, _) = with_tubes(3);
    let (a, b) = (fly(&plain), fly(&r));
    assert_eq!(b.body, a.body, "an open tube adds no body segment");
    assert_eq!(b.tube_fins.len(), 1);
    let t = &b.tube_fins[0];
    assert_eq!(t.count, 3);
    assert!((t.station - (NOSE + 0.6 + 0.12)).abs() < 1e-9 && (t.length - TUBE_L).abs() < 1e-12);
    assert!((t.outer_radius - TUBE_D / 2.0).abs() < 1e-12 && (t.inner_radius - (TUBE_D / 2.0 - 0.001)).abs() < 1e-12);
    assert!((t.body_radius - BODY / 2.0).abs() < 1e-12 && t.gap == 0.0);
    assert!((b.reference_diameter - BODY).abs() < 1e-12);
}

#[test]
fn a_pod_with_a_nose_a_bulkhead_or_a_transition_is_a_body() {
    let db = MotorDb::bundled();
    let (mut r, lower) = core();
    // Nose + open tube: still a body, whatever the aft end.
    let mut nosed = PodBody::standard(0.0254, 0.15);
    assert!(!nosed.is_open_tube());
    r.attach_radial(lower, 0.12, 0.0, 2, RadialKind::Pod { gap: 0.0, body: nosed.clone() }).unwrap();
    let v = vehicle(&r, &db, Phase::Full).unwrap();
    assert!(v.tube_fins.is_empty() && v.body.len() > fly(&core().0).body.len());
    // The same pod without the nose is open; a bulkhead in the tube closes it again.
    let open = PodBody::single(Part::new(0, tube()), vec![], vec![]);
    assert!(open.is_open_tube());
    let (mut r, pod) = with_tubes(2);
    assert_eq!(fly(&r).tube_fins.len(), 1);
    let part = r.radial(pod).unwrap().kind.pod_body().unwrap().parts()[0].id;
    r.attach_internal(part, 0.0, InternalKind::Bulkhead { diameter: None, thickness: 0.003 }).unwrap();
    let v = fly(&r);
    assert!(v.tube_fins.is_empty() && v.body.len() > fly(&core().0).body.len());
    nosed = PodBody::default();
    assert!(!nosed.is_open_tube(), "an empty body is not a tube");
}

/// Hand calculation of the tube fin normal force for three tubes of 0.15 m, r_out 12.7 mm, r_in 11.7 mm: AR = 0.156,
/// AR' = 0.0993127, per tube `CN_alpha A_ref` = 2 (AR'/(1+AR')) pi^2 r_in L = 3.12961e-3 m^2; A_ref = pi 0.0381^2 = 4.5604e-3,
/// so 0.686263 per tube and 2.058789 for three (no Mach dependence, no body interference factor). CP at the quarter chord.
#[test]
fn normal_force_and_cp_match_hand_calc() {
    let (r, _) = with_tubes(3);
    let aero = Aero::new(&fly(&r));
    let s = aero.stability(0.3, 0.0);
    let tubes = s.components.iter().find(|c| c.0 == "Tube fins").expect("a tube fin component");
    assert!((tubes.1 - 2.058_789).abs() < 1e-5, "CN_alpha {}", tubes.1);
    assert!((tubes.2 - (NOSE + 0.6 + 0.12 + 0.25 * TUBE_L)).abs() < 1e-9, "CP {}", tubes.2);
    // The same slope at Mach 0.45 (still subsonic and below the 0.5 where the CP leaves the quarter chord).
    assert!((aero.stability(0.45, 0.0).components.iter().find(|c| c.0 == "Tube fins").unwrap().1 - tubes.1).abs() < 1e-12);
    // Past the 20 degree stall angle the normal force stops growing: CN(alpha) = CN_alpha * min(alpha, 20 deg).
    let late = aero.stability(0.3, 0.6).components.iter().find(|c| c.0 == "Tube fins").unwrap().1 * 0.6;
    assert!((late - 2.058_789 * 20.0 * PI / 180.0).abs() < 1e-5);
}

#[test]
fn open_tubes_move_the_cp_aft() {
    let (plain, _) = core();
    let (tubes, _) = with_tubes(3);
    let (before, after) = (Aero::new(&fly(&plain)).stability(0.3, 0.0), Aero::new(&fly(&tubes)).stability(0.3, 0.0));
    assert!(after.cn_alpha > before.cn_alpha + 2.0, "{} vs {}", after.cn_alpha, before.cn_alpha);
    // Tube fins at 0.99 m, behind the core's CP.
    assert!(after.cp > before.cp, "CP {} vs {}", after.cp, before.cp);
    let expected = (before.cn_alpha * before.cp + 2.058_789 * (NOSE + 0.6 + 0.12 + 0.25 * TUBE_L)) / (before.cn_alpha + 2.058_789);
    assert!((after.cp - expected).abs() < 1e-4, "CP {} vs {expected}", after.cp);
}

/// Pressure drag of the three tubes by `TubeCalc`/`TubeFinSetCalc` at Mach 0.3 (stagnation 0.869297, base 0.1317): bore
/// f L/D = 0.026155 (0.15/0.0234) = 0.167658 on 4.2999e-4 m^2, annular face 7.6655e-5 m^2 at 0.7 (stag + base), interstice
/// 4.5281e-5 m^2 at (stag + base): 0.037528 per tube, 0.112583 for three, over A_ref. Friction: the roughness-limited C_f of the
/// rocket (0.032 (eps/L)^0.2 (1 - 0.1 M^2), eps = 60 um, L = rocket length) times the outer wetted 4.0593e-3 m^2 per tube.
#[test]
fn drag_matches_hand_calc_and_an_open_tube_has_no_base() {
    let (plain, _) = core();
    let (tubes, _) = with_tubes(3);
    let (a, b) = (Aero::new(&fly(&plain)).drag(0.3, RE_PER_M, false), Aero::new(&fly(&tubes)).drag(0.3, RE_PER_M, false));
    let length = NOSE + 0.9;
    let cf = 0.032 * (60e-6f64 / length).powf(0.2) * (1.0 - 0.1 * 0.09);
    let expected = 0.112_583 + 3.0 * cf * 4.059_310e-3 / a_ref();
    assert!((b.fins - a.fins - expected).abs() < 5e-5, "tube drag {} vs {expected}", b.fins - a.fins);
    // Body friction, pressure and base are the core's: no base drag from the open aft end.
    assert!((b.base - a.base).abs() < 1e-12 && (b.pressure - a.pressure).abs() < 1e-12 && (b.friction - a.friction).abs() < 1e-12);
    // A closed pod of the same size (a nose and a tube) adds a base.
    let (mut closed, lower) = core();
    closed.attach_radial(lower, 0.12, 0.0, 3, RadialKind::Pod { gap: 0.0, body: PodBody::standard(TUBE_D, 0.15) }).unwrap();
    let c = Aero::new(&fly(&closed)).drag(0.3, RE_PER_M, false);
    assert!(c.base > a.base + 1e-4);
}

#[test]
fn open_tubes_raise_the_drag_with_their_count() {
    let drag = |n: u8| Aero::new(&fly(&with_tubes(n).0)).drag(0.3, RE_PER_M, false).total;
    let plain = Aero::new(&fly(&core().0)).drag(0.3, RE_PER_M, false).total;
    assert!(plain < drag(1) && drag(1) < drag(3));
    assert!(((drag(3) - plain) / (drag(1) - plain) - 3.0).abs() < 1e-9, "linear in the tube count");
}

#[test]
fn health_list_has_no_blunt_front_line_for_open_tubes() {
    let db = MotorDb::bundled();
    let (r, _) = with_tubes(3);
    let b = balance(&r, &db, Phase::Full, 0.3, 0.0, true).unwrap();
    let lines = warnings(&r, &db, &b);
    assert!(lines.iter().all(|w| !w.message.contains("blunt")), "{lines:?}");
}
