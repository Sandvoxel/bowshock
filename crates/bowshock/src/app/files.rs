//! File commands: New, Open, Save, Save As and the built-in example, with the unsaved-changes guard.

use std::path::Path;

use bowshock_model::Rocket;
use iced::Task;

use super::document::EXTENSION;
use super::{Armed, Ctx, Message};

/// The built-in example, a 38 mm high-power rocket (the kit's `hpr_l1_cert`). `tests::example_is_current` keeps it loadable and
/// in step with the model.
const EXAMPLE: &str = include_str!("../../assets/examples/hpr_l1_cert.bowshock");

/// A two-stage example, selected by `BOWSHOCK_EXAMPLE=two_stage` (for screenshots of staging).
const TWO_STAGE: &str = include_str!("../../assets/examples/two_stage.bowshock");

/// Whether the command `what` may replace the document now. With unsaved changes the first press only warns (and arms the
/// command); pressing the same command again proceeds.
pub(super) fn guard(ctx: &mut Ctx, what: Armed) -> bool {
    if !ctx.doc.is_dirty() || ctx.armed == Some(what) {
        ctx.armed = None;
        return true;
    }
    ctx.armed = Some(what);
    ctx.notify("Unsaved changes: press again to discard them, or save first");
    false
}

/// Starts an empty document.
pub(super) fn new(ctx: &mut Ctx) {
    ctx.doc.new_document();
    ctx.loaded();
}

/// Loads the built-in example as an unsaved document named after it.
pub(super) fn example(ctx: &mut Ctx) {
    let source = if std::env::var("BOWSHOCK_EXAMPLE").is_ok_and(|v| v == "two_stage") { TWO_STAGE } else { EXAMPLE };
    match Rocket::from_json(source) {
        Ok(rocket) => {
            ctx.doc.replace(rocket, None);
            ctx.loaded();
        }
        Err(e) => ctx.notify(format!("The example rocket is not loadable: {e}")),
    }
}

/// Opens the file at `path`, reporting failure in the status strip.
pub(super) fn open(ctx: &mut Ctx, path: &Path) {
    match ctx.doc.load(path) {
        Ok(()) => ctx.loaded(),
        Err(e) => ctx.notify(format!("Could not open {}: {e}", path.display())),
    }
}

/// Saves to the document's path, or asks for one.
pub(super) fn save(ctx: &mut Ctx) -> Task<Message> {
    if ctx.doc.path().is_none() {
        return pick_save(ctx);
    }
    match ctx.doc.save() {
        Ok(()) => ctx.notify(format!("Saved {}.{EXTENSION}", ctx.doc.title())),
        Err(e) => ctx.notify(format!("Could not save: {e}")),
    }
    Task::none()
}

/// Saves to `path` and makes it the document's path.
pub(super) fn save_to(ctx: &mut Ctx, path: &Path) {
    match ctx.doc.save_as(path) {
        Ok(()) => ctx.notify(format!("Saved {}.{EXTENSION}", ctx.doc.title())),
        Err(e) => ctx.notify(format!("Could not save: {e}")),
    }
}

/// The open dialog; the answer arrives as [`Message::OpenPath`].
pub(super) fn pick_open() -> Task<Message> {
    Task::perform(
        async {
            let file = rfd::AsyncFileDialog::new().set_title("Open rocket").add_filter("Bowshock document", &[EXTENSION]).pick_file().await;
            file.map(|f| f.path().to_path_buf())
        },
        Message::OpenPath,
    )
}

/// The save dialog; the answer arrives as [`Message::SavePath`].
pub(super) fn pick_save(ctx: &Ctx) -> Task<Message> {
    let name = format!("{}.{EXTENSION}", ctx.doc.title());
    Task::perform(
        async move {
            let file = rfd::AsyncFileDialog::new().set_title("Save rocket").set_file_name(name).add_filter("Bowshock document", &[EXTENSION]).save_file().await;
            file.map(|f| f.path().to_path_buf())
        },
        Message::SavePath,
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use bowshock_model::{DeployEvent, Deployment, InternalKind, MassRole, MaterialId, NoseShape, PartKind, RadialKind};
    use bowshock_motors::MotorDb;
    use bowshock_sim::from_model::{self, Level, Phase};

    /// The example rocket, built through the model API: the kit's `hpr_l1_cert`, a 76.2 mm high-power rocket on an H128W.
    fn build_example() -> Rocket {
        let mut r = Rocket::default();
        r.name = "hpr_l1_cert".to_string();
        r.designer = "Bowshock".to_string();
        r.insert(0, PartKind::NoseCone { shape: NoseShape::TangentOgive, length: 0.2286, diameter: 0.0762 }).unwrap();
        let upper = r.insert(1, PartKind::BodyTube { length: 0.520, diameter: 0.0762 }).unwrap();
        let lower = r.insert(2, PartKind::BodyTube { length: 0.3934, diameter: 0.0762 }).unwrap();
        r.set_name(upper, Some("Recovery bay".to_string()));
        r.set_name(lower, Some("Booster".to_string()));

        // Three trapezoidal fins flush with the aft end of the booster, a pair of rail buttons, a second pair further aft.
        let fins = RadialKind::FinSet { root_chord: 0.120, tip_chord: 0.050, span: 0.0762, sweep: 0.060, thickness: 0.00318 };
        r.attach_radial(lower, 0.3934 - 0.120, 0.0, 3, fins).unwrap();
        let button = RadialKind::RailButton { height: 0.008, diameter: 0.011 };
        r.attach_radial(upper, 0.380, 0.0, 1, button.clone()).unwrap();
        r.attach_radial(lower, 0.100, 0.0, 1, button).unwrap();

        // The 38 mm motor mount with a centering ring at each end.
        let mount = r
            .attach_internal(
                lower,
                0.0534,
                InternalKind::InnerTube { outer_diameter: 0.042, thickness: 0.002, length: 0.340, motor_mount: true, overhang: 0.0 },
            )
            .unwrap();
        let ring = InternalKind::CenteringRing { outer_diameter: None, inner_diameter: None, thickness: 0.006 };
        r.attach_internal(lower, 0.0534, ring.clone()).unwrap();
        r.attach_internal(lower, 0.3934 - 0.006, ring).unwrap();

        // Recovery: a 36 in chute on the ejection charge, shock cord, an altimeter and ballast in the bay.
        let chute = InternalKind::Parachute {
            diameter: 0.9144,
            cd: 0.8,
            line_count: 6,
            line_length: 0.9144,
            line_material: MaterialId::BRAIDED_NYLON_2MM,
            packed_length: 0.200,
            packed_diameter: None,
            deploy: Deployment { event: DeployEvent::Ejection, delay: 0.0 },
        };
        r.attach_internal(upper, 0.250, chute).unwrap();
        r.attach_internal(upper, 0.150, InternalKind::ShockCord { length: 2.0, packed_length: 0.080 }).unwrap();
        r.attach_internal(upper, 0.470, InternalKind::MassComponent { mass: 0.045, length: 0.060, diameter: Some(0.030), role: MassRole::Altimeter }).unwrap();
        r.attach_internal(upper, 0.020, InternalKind::MassComponent { mass: 0.120, length: 0.040, diameter: None, role: MassRole::Ballast }).unwrap();

        // Configuration H128W (named after its motor).
        let db = MotorDb::bundled();
        let motor = db.find("H128W").expect("H128W is bundled");
        let id = r.active_config_id();
        r.set_motor(id, mount, Some(bowshock_model::MotorRef::new(motor.designation.clone(), motor.manufacturer.clone()))).unwrap();
        r.set_motor_delay(id, mount, 14.0).unwrap();
        r
    }

    /// Keeps `assets/examples/hpr_l1_cert.bowshock` in step with [`build_example`] and the model's save format. Regenerate it
    /// with `BOWSHOCK_WRITE_EXAMPLE=1 cargo test -p bowshock example_is_current`.
    #[test]
    fn example_is_current() {
        let built = build_example();
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("assets/examples/hpr_l1_cert.bowshock");
        if std::env::var_os("BOWSHOCK_WRITE_EXAMPLE").is_some() {
            std::fs::create_dir_all(path.parent().unwrap()).unwrap();
            std::fs::write(&path, built.to_json()).unwrap();
        }
        let shipped = Rocket::from_json(EXAMPLE).expect("the shipped example loads");
        assert_eq!(shipped, built, "assets/examples/hpr_l1_cert.bowshock is stale: regenerate it (see this test's docs)");
    }

    #[test]
    fn the_example_is_the_kits_rocket_and_flies_stable() {
        let rocket = Rocket::from_json(EXAMPLE).unwrap();
        assert!((rocket.length() - 1.142).abs() < 1e-9 && rocket.reference_diameter() == Some(0.0762));
        let db = MotorDb::bundled();
        let balance = from_model::balance(&rocket, &db, Phase::Full, 0.3, 0.0, true).unwrap();
        // A well-built 3-fin rocket on this motor: stable between 1 and 6 calibers, no errors.
        assert!(balance.margin_cal > 1.0 && balance.margin_cal < 6.0, "margin {}", balance.margin_cal);
        let warnings = from_model::warnings(&rocket, &db, &balance);
        assert!(warnings.iter().all(|w| w.level != Level::Err), "{warnings:?}");
    }

    /// The two-stage example (the kit's `two_stage`): an I284W booster under an H128W sustainer, separating at the joint behind
    /// the sustainer's motor.
    fn build_two_stage() -> Rocket {
        let mut r = Rocket::default();
        r.name = "two_stage".to_string();
        r.designer = "Bowshock".to_string();
        r.insert(0, PartKind::NoseCone { shape: NoseShape::TangentOgive, length: 0.2286, diameter: 0.0762 }).unwrap();
        let sustainer = r.insert(1, PartKind::BodyTube { length: 0.642, diameter: 0.0762 }).unwrap();
        let booster = r.insert(2, PartKind::BodyTube { length: 0.530, diameter: 0.0762 }).unwrap();
        r.set_name(sustainer, Some("Sustainer tube".to_string()));
        r.set_name(booster, Some("Booster tube".to_string()));
        r.attach_radial(
            sustainer,
            0.642 - 0.120,
            0.0,
            3,
            RadialKind::FinSet { root_chord: 0.120, tip_chord: 0.050, span: 0.0762, sweep: 0.060, thickness: 0.00318 },
        )
        .unwrap();
        r.attach_radial(
            booster,
            0.530 - 0.120,
            0.0,
            4,
            RadialKind::FinSet { root_chord: 0.120, tip_chord: 0.050, span: 0.0762, sweep: 0.060, thickness: 0.00318 },
        )
        .unwrap();
        let mount_kind = InternalKind::InnerTube { outer_diameter: 0.042, thickness: 0.002, length: 0.340, motor_mount: true, overhang: 0.0 };
        let upper = r.attach_internal(sustainer, 0.302, mount_kind.clone()).unwrap();
        let lower = r.attach_internal(booster, 0.190, mount_kind).unwrap();
        let ring = InternalKind::CenteringRing { outer_diameter: None, inner_diameter: None, thickness: 0.006 };
        for (tube, offsets) in [(sustainer, [0.302, 0.636]), (booster, [0.190, 0.524])] {
            for offset in offsets {
                r.attach_internal(tube, offset, ring.clone()).unwrap();
            }
        }
        r.attach_internal(
            sustainer,
            0.080,
            InternalKind::Parachute {
                diameter: 0.762,
                cd: 0.8,
                line_count: 6,
                line_length: 0.762,
                line_material: MaterialId::BRAIDED_NYLON_2MM,
                packed_length: 0.180,
                packed_diameter: None,
                deploy: Deployment { event: DeployEvent::Ejection, delay: 0.0 },
            },
        )
        .unwrap();
        r.attach_internal(sustainer, 0.020, InternalKind::MassComponent { mass: 0.045, length: 0.040, diameter: Some(0.030), role: MassRole::Altimeter })
            .unwrap();
        let db = MotorDb::bundled();
        let id = r.active_config_id();
        for (mount, name) in [(upper, "H128W"), (lower, "I284W")] {
            let motor = db.find(name).expect("bundled");
            r.set_motor(id, mount, Some(bowshock_model::MotorRef::new(motor.designation.clone(), motor.manufacturer.clone()))).unwrap();
        }
        r
    }

    /// Keeps `assets/examples/two_stage.bowshock` in step with [`build_two_stage`] (regenerate like the first example) and checks
    /// the model finds the separation behind the sustainer's motor.
    #[test]
    fn two_stage_example_is_current() {
        let built = build_two_stage();
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("assets/examples/two_stage.bowshock");
        if std::env::var_os("BOWSHOCK_WRITE_EXAMPLE").is_some() {
            std::fs::write(&path, built.to_json()).unwrap();
        }
        let shipped = Rocket::from_json(TWO_STAGE).expect("the shipped example loads");
        assert_eq!(shipped, built, "assets/examples/two_stage.bowshock is stale: regenerate it (see example_is_current)");
        let stages = shipped.stages();
        assert_eq!(stages.len(), 2);
        let separation = stages[0].separation.expect("a joint behind the sustainer's motor");
        assert!((separation.station - 0.8706).abs() < 1e-9);
    }
}
