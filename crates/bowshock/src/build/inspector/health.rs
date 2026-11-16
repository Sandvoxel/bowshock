//! The inspector's inline health chips: the checks specific to the selected part (design/components/WarningChip). Fin flutter
//! (Martin, NACA TN 4197, through `analyze::flutter`: the same margin the Analyze screen shows), parachute descent rate, whether
//! a recovery device packs in its tube, and whether the motor fits its mount. A check is plain data ([`Check`]) so it can be
//! tested; the view turns it into a chip.
//!
//! The flutter and descent chips need a flight (the margin along it, the descent rate). It is flown with the default launch,
//! cached for the document revision it was flown for and reflown only when the document changes, so a field that is scrubbed
//! does not fly the rocket on every frame ([`FlightCache::needs_flight`] says when it is stale).

use std::cell::RefCell;
use std::rc::Rc;

use bowshock_model::{InternalKind, MotorMount, PartId, PartRef, Rocket};
use bowshock_motors::MotorDb;
use bowshock_sim::flutter::FlutterWarning;
use bowshock_sim::from_model::{self, Level, Phase, Warning};
use bowshock_sim::{simulate, Launch};

use crate::analyze::compute::FLUTTER_MIN_MARGIN;
use crate::analyze::flutter::{stocks, FlutterAnalysis};
use crate::app::{ui, Ctx};
use crate::units::Quantity;
use crate::widgets::chip::Severity;

/// A descent rate up to this is a comfortable landing, m/s (20 ft/s; the 15 to 20 ft/s range most club rules ask for).
pub const DESCENT_OK: f64 = 6.1;
/// Up to this a landing is hard but survivable, m/s (30 ft/s); beyond it the rocket is likely to break.
pub const DESCENT_WARN: f64 = 9.1;

/// One chip: a severity and the words (what, by how much, the limit).
#[derive(Clone, Debug, PartialEq)]
pub struct Check {
    /// `ok`, `warn` or `err`.
    pub severity: Severity,
    /// The label.
    pub text: String,
    /// The row label it sits on in the inspector ("Flutter").
    pub row: &'static str,
}

/// What the flight told us: the flutter margin along it and each device's descent rate.
#[derive(Clone, Debug, Default)]
pub struct Flight {
    /// The fin flutter estimate along the flight up to apogee, for every fin set; `None` for a rocket without fins.
    pub flutter: Option<FlutterAnalysis>,
    /// `(device name, descent rate m/s)`.
    pub descent_rates: Vec<(String, f64)>,
}

/// Flies `rocket` with its active configuration (whole stack, default launch). `None` if it cannot fly (no parts, no motor).
pub fn fly(rocket: &Rocket, motors: &MotorDb) -> Option<Flight> {
    let input = from_model::sim_input(rocket, motors, Phase::Full, Launch::default()).ok()?;
    let flight = simulate(&input);
    let flutter = FlutterAnalysis::of_flight(stocks(rocket, &input.vehicle), &input.launch, &flight);
    Some(Flight { flutter, descent_rates: flight.summary.descent_rates.clone() })
}

/// The flight of the document at a revision, flown on demand.
#[derive(Default)]
pub struct FlightCache {
    slot: RefCell<Option<(u64, Option<Rc<Flight>>)>>,
}

impl FlightCache {
    /// The last flight flown, whatever revision it was for (what a scrubbed field keeps showing until it is released).
    pub fn latest(&self) -> Option<Rc<Flight>> {
        self.slot.borrow().as_ref().and_then(|(_, f)| f.clone())
    }

    /// Whether the cache holds a flight for `revision`.
    pub fn needs_flight(&self, revision: u64) -> bool {
        self.slot.borrow().as_ref().is_none_or(|(rev, _)| *rev != revision)
    }

    /// The flight for `revision`, flown now if the cache is for another revision.
    pub fn get(&self, rocket: &Rocket, motors: &MotorDb, revision: u64) -> Option<Rc<Flight>> {
        if self.needs_flight(revision) {
            *self.slot.borrow_mut() = Some((revision, fly(rocket, motors).map(Rc::new)));
        }
        self.slot.borrow().as_ref().and_then(|(_, f)| f.clone())
    }
}

/// The flutter check of fin group `id`: `None` for anything that is not a fin or when there is no flight to judge against. The
/// margin is the lowest of `V_f / V` over the whole flight up to apogee for the group's fins (each copy with its own overrides
/// counts), the same number the Analyze screen shows for the rocket when this group governs. The message names the cause and
/// the cheapest fix: the thickness that reaches [`FLUTTER_MIN_MARGIN`] (`V_f` grows with `t^1.5`).
pub fn flutter(rocket: &Rocket, id: PartId, flight: &Flight, ctx: &Ctx) -> Option<Check> {
    rocket.radial(id).filter(|r| r.kind.is_fin())?;
    let group = flight.flutter.as_ref()?.for_group(id)?;
    let worst = group.trace.worst();
    let margin = group.min_margin();
    let inputs = group.inputs()?;
    let beyond = group.warnings().into_iter().find_map(|w| if let FlutterWarning::BeyondMach(m) = w { Some(m) } else { None });
    let needed = ui::with_unit(inputs.thickness * (FLUTTER_MIN_MARGIN / margin).powf(2.0 / 3.0), Quantity::Length, ctx.units);
    let (severity, text) = if !margin.is_finite() {
        (Severity::Ok, "No flutter".to_string())
    } else if margin < 1.0 {
        let at = ui::with_unit(worst.speed, Quantity::Velocity, ctx.units);
        (Severity::Err, format!("Flutters at {at} (M {:.2}) · fins too thin or soft: {needed} thick for {FLUTTER_MIN_MARGIN:.2}×", worst.mach))
    } else if margin < FLUTTER_MIN_MARGIN {
        (Severity::Warn, format!("{margin:.2}× (min {FLUTTER_MIN_MARGIN:.2}) · fins too thin or soft: {needed} thick for {FLUTTER_MIN_MARGIN:.2}×"))
    } else if let Some(mach) = beyond {
        (Severity::Warn, format!("{margin:.1}× margin · reaches M {mach:.1}, beyond Martin's Mach 1.5"))
    } else {
        (Severity::Ok, format!("{margin:.1}× margin"))
    };
    Some(Check { severity, text, row: "Flutter" })
}

/// The descent-rate check of recovery device `id`.
pub fn descent(rocket: &Rocket, id: PartId, flight: &Flight, ctx: &Ctx) -> Option<Check> {
    let part = rocket.internal(id).filter(|p| matches!(p.kind, InternalKind::Parachute { .. } | InternalKind::Streamer { .. }))?;
    let name = part.display_name();
    let (_, rate) = flight.descent_rates.iter().find(|(n, _)| *n == name)?;
    let rate_text = ui::with_unit(*rate, Quantity::Velocity, ctx.units);
    let limit = ui::bare(DESCENT_OK, Quantity::Velocity, ctx.units);
    let text = format!("{rate_text} (max {limit})");
    let severity = if *rate <= DESCENT_OK {
        Severity::Ok
    } else if *rate <= DESCENT_WARN {
        Severity::Warn
    } else {
        Severity::Err
    };
    Some(Check { severity, text, row: "Descent" })
}

/// Whether recovery device `id` packs in its tube: its packed bundle must be no wider than the bore and lie within the tube's
/// length.
pub fn packing(rocket: &Rocket, id: PartId, ctx: &Ctx) -> Option<Check> {
    let part =
        rocket.internal(id).filter(|p| matches!(p.kind, InternalKind::Parachute { .. } | InternalKind::Streamer { .. } | InternalKind::ShockCord { .. }))?;
    let parent = rocket.part(part.parent)?;
    let resolved = rocket.resolved(id)?;
    let bore = rocket.inner_diameter_at(part.parent, part.offset + part.kind.extent() / 2.0)?;
    let past_end = part.offset + part.kind.extent() - parent.kind.length();
    let wide = resolved.outer_diameter - bore;
    let mm = |v: f64| ui::with_unit(v, Quantity::Length, ctx.units);
    let (severity, text) = if wide > 1e-6 {
        (Severity::Warn, format!("Too wide · {} over", mm(wide)))
    } else if past_end > 1e-6 {
        (Severity::Warn, format!("Past the end · {} over", mm(past_end)))
    } else {
        (Severity::Ok, format!("Fits · {} of {}", ui::bare(part.kind.extent(), Quantity::Length, ctx.units), mm(parent.kind.length())))
    };
    Some(Check { severity, text, row: "Packing" })
}

/// The motor of the active configuration in mount `id`, with the fit warnings of `warnings` for the mount (or an `ok` chip
/// saying it fits). Empty for a mount without a motor in this configuration.
pub fn motor(rocket: &Rocket, id: PartId, motors: &MotorDb, warnings: &[Warning], ctx: &Ctx) -> Vec<Check> {
    let Some(mount) = rocket.motor_mounts().into_iter().find(|m| m.id == id) else {
        return Vec::new();
    };
    let Some(designation) = rocket.active_configuration().slot(id).and_then(|s| s.motor.as_ref()).map(|m| m.designation.clone()) else {
        return vec![Check { severity: Severity::Warn, text: "No motor in this configuration".into(), row: "Motor" }];
    };
    let mut out: Vec<Check> = warnings
        .iter()
        .filter(|w| w.part == Some(id) && w.level != Level::Ok)
        .map(|w| Check { severity: if w.level == Level::Err { Severity::Err } else { Severity::Warn }, text: w.message.clone(), row: "Motor" })
        .collect();
    if out.is_empty() {
        out.push(Check { severity: Severity::Ok, text: fits(&designation, &mount, motors, ctx), row: "Motor" });
    }
    out
}

fn fits(designation: &str, mount: &MotorMount, motors: &MotorDb, ctx: &Ctx) -> String {
    let mm = |v: f64| ui::with_unit(v, Quantity::Length, ctx.units);
    match motors.find(designation) {
        Some(m) => format!("{designation} fits · {} mm", crate::app::ui::mm(m.length)),
        None => format!("{designation} fits {}", mm(mount.inner_diameter)),
    }
}

/// Every check for part `id`, in the order the inspector shows them.
pub fn checks(rocket: &Rocket, id: PartId, flight: Option<&Flight>, ctx: &Ctx) -> Vec<Check> {
    let mut out = Vec::new();
    match rocket.kind_of(id) {
        Some(PartRef::Radial(_)) => out.extend(flight.and_then(|f| flutter(rocket, id, f, ctx))),
        Some(PartRef::Internal(p)) => {
            out.extend(packing(rocket, id, ctx));
            out.extend(flight.and_then(|f| descent(rocket, id, f, ctx)));
            if p.kind.is_motor_mount() {
                out.extend(motor(rocket, id, ctx.physics.motors(), ctx.physics.warnings(), ctx));
            }
        }
        _ => {}
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::app::App;
    use bowshock_model::{MassRole, MaterialId};

    fn app() -> App {
        App::with_example()
    }

    fn id_of(rocket: &Rocket, f: impl Fn(&bowshock_model::InternalPart) -> bool) -> PartId {
        rocket.internals().iter().find(|p| f(p)).map(|p| p.id).unwrap()
    }

    #[test]
    fn the_example_flies_and_its_fins_and_chute_are_judged() {
        let app = app();
        let ctx = app.ctx();
        let rocket = ctx.doc.rocket();
        let flight = fly(rocket, ctx.physics.motors()).expect("the example flies");
        let analysis = flight.flutter.as_ref().expect("the example has fins");
        let top = analysis.trace.points.iter().map(|p| p.speed).fold(0.0, f64::max);
        assert!(top > 50.0 && top < 400.0, "{top}");
        let fins = rocket.radials().iter().find(|r| r.kind.is_fin()).unwrap().id;
        let check = flutter(rocket, fins, &flight, ctx).expect("flutter check");
        // 120 mm root, 3.18 mm birch plywood (G 0.85 GPa): V_f is about 169 m/s at Mach 0.53, 181 m/s is the rocket's speed there.
        assert_eq!(check.severity, Severity::Err, "{}", check.text);
        assert!(check.text.starts_with("Flutters at") && check.text.contains("thick for 1.25×"), "{}", check.text);
        // The Analyze screen's number is the same: one fin group, one margin.
        let (jobs, _) = crate::analyze::compute::jobs(rocket, ctx.physics.motors(), bowshock_sim::from_model::Phase::Full).unwrap();
        let out = crate::analyze::compute::compute(&crate::analyze::compute::Request {
            analysis: crate::analyze::Analysis::Flutter,
            method: crate::analyze::Method::Extended,
            conditions: Default::default(),
            jobs,
            bands: Default::default(),
            cfd: None,
        });
        let crate::analyze::compute::Data::Flutter(Some(on_screen)) = &out.runs[0].data else { panic!("the example has fins") };
        assert_eq!(on_screen.min_margin(), flight.flutter.as_ref().unwrap().min_margin());
        // The fix the message names works: fins 5 mm thick.
        let mut thicker = rocket.clone();
        thicker.set_dim(fins, bowshock_model::Dim::Thickness, 0.005);
        let fixed = flutter(&thicker, fins, &fly(&thicker, ctx.physics.motors()).unwrap(), ctx).unwrap();
        assert_eq!(fixed.severity, Severity::Ok, "{}", fixed.text);
        assert!(descent(rocket, id_of(rocket, |p| matches!(p.kind, InternalKind::Parachute { .. })), &flight, ctx).is_some());
        // Nothing to say about a body tube.
        assert!(flutter(rocket, rocket.parts()[0].id, &flight, ctx).is_none());
    }

    #[test]
    fn one_long_override_fin_decides_the_groups_flutter_margin() {
        use bowshock_model::{Dim, RadialField};
        let app = app();
        let ctx = app.ctx();
        let mut rocket = ctx.doc.rocket().clone();
        let fins = rocket.radials().iter().find(|r| r.kind.is_fin() && r.count > 1).unwrap().id;
        let motors = ctx.physics.motors();
        let base = flutter(&rocket, fins, &fly(&rocket, motors).unwrap(), ctx).unwrap();
        assert!(rocket.unlink_field(fins, 1, RadialField::Span));
        let longer = rocket.instance_dim(fins, 1, Dim::Span).unwrap() * 3.0;
        rocket.set_instance_dim(fins, 1, Dim::Span, longer).unwrap();
        let flight = fly(&rocket, motors).unwrap();
        assert_eq!(flight.flutter.as_ref().unwrap().stocks.len(), 2, "the loose fin flies as its own set");
        let margin = |c: &Check| c.text.split('×').next().unwrap().parse::<f64>().unwrap_or(0.0);
        let worst = flutter(&rocket, fins, &flight, ctx).unwrap();
        assert!(margin(&worst) < margin(&base) || worst.severity != Severity::Ok, "{} vs {}", worst.text, base.text);
    }

    #[test]
    fn thin_soft_fins_flutter() {
        let app = app();
        let ctx = app.ctx();
        let mut rocket = ctx.doc.rocket().clone();
        let fins = rocket.radials().iter().find(|r| r.kind.is_fin()).unwrap().id;
        rocket.set_material(fins, MaterialId::BALSA);
        rocket.set_dim(fins, bowshock_model::Dim::Thickness, 0.0004);
        rocket.set_dim(fins, bowshock_model::Dim::Span, 0.2);
        let flight = fly(&rocket, ctx.physics.motors()).unwrap();
        let check = flutter(&rocket, fins, &flight, ctx).unwrap();
        assert_eq!(check.severity, Severity::Err, "{}", check.text);
        // The message names the cause and the fix.
        assert!(check.text.starts_with("Flutters at") && check.text.contains("thick for"), "{}", check.text);
        // Stock 100 times stiffer (G 20 GPa against balsa's 0.2) is 10 times the margin.
        let before = flight.flutter.as_ref().unwrap().min_margin();
        assert!(rocket.set_fin_shear_modulus(fins, Some(20.0e9)));
        let after = fly(&rocket, ctx.physics.motors()).unwrap().flutter.unwrap().min_margin();
        assert!((after / before - 10.0).abs() < 0.2, "{before} -> {after}");
    }

    #[test]
    fn packing_checks_the_bore_and_the_tube_length() {
        let app = app();
        let ctx = app.ctx();
        let mut rocket = ctx.doc.rocket().clone();
        let chute = id_of(&rocket, |p| matches!(p.kind, InternalKind::Parachute { .. }));
        assert_eq!(packing(&rocket, chute, ctx).unwrap().severity, Severity::Ok);
        // A packed bundle wider than the bore.
        assert!(super::super::edit::set_auto_diameter(&mut rocket, chute, Some(0.2)));
        let wide = packing(&rocket, chute, ctx).unwrap();
        assert_eq!(wide.severity, Severity::Warn);
        assert!(wide.text.starts_with("Too wide ·"), "{}", wide.text);
        assert!(packing(&rocket, rocket.parts()[0].id, ctx).is_none());
        let ballast = id_of(&rocket, |p| matches!(p.kind, InternalKind::MassComponent { role: MassRole::Ballast, .. }));
        assert!(packing(&rocket, ballast, ctx).is_none());
    }

    #[test]
    fn the_motor_mount_says_whether_the_motor_fits() {
        let app = app();
        let ctx = app.ctx();
        let rocket = ctx.doc.rocket();
        let mount = id_of(rocket, |p| p.kind.is_motor_mount());
        let out = motor(rocket, mount, ctx.physics.motors(), ctx.physics.warnings(), ctx);
        assert_eq!(out.len(), 1);
        assert_eq!(out[0].severity, Severity::Ok);
        assert!(out[0].text.starts_with("H128W fits"), "{}", out[0].text);
        // A ring is not a mount.
        let ring = id_of(rocket, |p| matches!(p.kind, InternalKind::CenteringRing { .. }));
        assert!(motor(rocket, ring, ctx.physics.motors(), ctx.physics.warnings(), ctx).is_empty());
    }

    #[test]
    fn the_cache_flies_once_per_revision() {
        let app = app();
        let ctx = app.ctx();
        let cache = FlightCache::default();
        assert!(cache.needs_flight(7));
        let a = cache.get(ctx.doc.rocket(), ctx.physics.motors(), 7).unwrap();
        assert!(!cache.needs_flight(7) && cache.needs_flight(8));
        let b = cache.get(ctx.doc.rocket(), ctx.physics.motors(), 7).unwrap();
        assert!(Rc::ptr_eq(&a, &b));
    }
}
