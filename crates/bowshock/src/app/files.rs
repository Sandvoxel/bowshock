//! File commands: New, Open, Save, Save As and the built-in example, with the unsaved-changes guard.

use std::path::{Path, PathBuf};

use bowshock_model::ork::OrkImport;
use bowshock_model::{FlightMotor, MotorRef, Rocket};
use bowshock_motors::{same_motor_name, Motor};
use bowshock_sim::from_model::{find_motor, motor_ref};
use iced::Task;

use super::dialogs;
use super::document::EXTENSION;
use super::{Ctx, Message, Pending};

/// The built-in example, a 38 mm high-power rocket (the kit's `hpr_l1_cert`). `tests::example_is_current` keeps it loadable and
/// in step with the model.
const EXAMPLE: &[u8] = include_bytes!("../../assets/examples/hpr_l1_cert.bowshock");

/// A two-stage example, selected by `BOWSHOCK_EXAMPLE=two_stage` (for screenshots of staging).
const TWO_STAGE: &[u8] = include_bytes!("../../assets/examples/two_stage.bowshock");

/// Whether the command `what` may replace the document now. With unsaved changes it does not: the command is parked in
/// [`Ctx::unsaved`] and the "Save changes?" modal opens; its three answers are [`proceed`] (after saving, or without).
pub(super) fn guard(ctx: &mut Ctx, what: Pending) -> bool {
    if !ctx.doc.is_dirty() {
        return true;
    }
    ctx.unsaved = Some(what);
    false
}

/// Runs a command the unsaved-changes modal was holding back.
pub(super) fn proceed(ctx: &mut Ctx, what: Pending) -> Task<Message> {
    match what {
        Pending::New => new(ctx),
        Pending::Open => return pick_open(),
        Pending::Example => example(ctx),
        Pending::Import => return pick_import(),
        Pending::Recent(path) => open(ctx, &path),
        Pending::Exit => return iced::exit(),
    }
    Task::none()
}

/// The modal's *Save*: saves to the document's path (or opens Save As) and then runs the held command. A failed save, or a
/// Save As that is cancelled, abandons the command and keeps the document.
pub(super) fn save_then(ctx: &mut Ctx, what: Pending) -> Task<Message> {
    if ctx.doc.path().is_none() {
        ctx.saving_for = Some(what);
        return pick_save(ctx);
    }
    let task = save(ctx);
    if ctx.doc.is_dirty() {
        return task;
    }
    proceed(ctx, what)
}

/// The save dialog answered while the modal's *Save* waited for it.
pub(super) fn saved_as(ctx: &mut Ctx, path: Option<PathBuf>) -> Task<Message> {
    let pending = ctx.saving_for.take();
    let Some(path) = path else { return Task::none() };
    save_to(ctx, &path);
    match pending {
        Some(what) if !ctx.doc.is_dirty() => proceed(ctx, what),
        _ => Task::none(),
    }
}

/// Starts an empty document.
pub(super) fn new(ctx: &mut Ctx) {
    ctx.doc.new_document();
    ctx.loaded();
}

/// Loads the built-in example as an unsaved document named after it.
pub(super) fn example(ctx: &mut Ctx) {
    let source = if ctx.env.example_two_stage { TWO_STAGE } else { EXAMPLE };
    match Rocket::from_bytes(source) {
        Ok(rocket) => {
            ctx.doc.replace(rocket, None);
            ctx.loaded();
            ctx.fetch_wanted = true;
        }
        Err(e) => ctx.notify(format!("The example rocket is not loadable: {e}")),
    }
}

/// Opens the file at `path`, reporting failure in the status strip. An OpenRocket `.ork` is imported as a new untitled document
/// (see `bowshock_model::ork`); the status strip says how many things could not be carried over.
pub(super) fn open(ctx: &mut Ctx, path: &Path) {
    if path.extension().is_some_and(|x| x.eq_ignore_ascii_case("ork")) {
        match bowshock_model::ork::import_file(path) {
            Ok(imp) => {
                let mut rocket = imp.rocket.clone();
                let mut notes = imp.warnings.clone();
                notes.extend(use_embedded_curves(ctx, &imp, &mut rocket));
                if !rocket.name.ends_with(IMPORTED) {
                    rocket.name = if rocket.name.is_empty() { format!("untitled{IMPORTED}") } else { format!("{}{IMPORTED}", rocket.name) };
                }
                ctx.doc.replace(rocket, None);
                ctx.loaded();
                ctx.fetch_wanted = true;
                ctx.notify(match notes.len() {
                    0 => "Imported from OpenRocket".to_string(),
                    1 => "Imported from OpenRocket · 1 note".to_string(),
                    n => format!("Imported from OpenRocket · {n} notes"),
                });
                ctx.import_notes = notes;
            }
            Err(e) => ctx.notify(format!("Could not import {}: {e}", path.display())),
        }
        return;
    }
    match ctx.doc.load(path) {
        Ok(()) => {
            ctx.loaded();
            ctx.fetch_wanted = true;
            ctx.recent.push(path);
        }
        Err(e) => ctx.notify(format!("Could not open {}: {e}", path.display())),
    }
}

/// Registers the thrust curves an `.ork` stores in its ZIP (`thrustcurves/<digest>.rse`) for the motors the design names that
/// the motor database cannot resolve, so they fly and the ThrustCurve.org lookup is not needed for them.
///
/// Each distinct `(designation, manufacturer, digest)` of `imp.motor_digests` whose digest has an embedded curve becomes a
/// motor (the curve's first engine under the *design's* designation and manufacturer, so one digest carrying `J33N`, `I32N` and
/// `J34N` gives three motors; see [`Motor::renamed`] for its `eng:` id), added through [`Physics::add_embedded`](super::physics::Physics::add_embedded)
/// (a user motor, kept in the motors directory). The references of `rocket` that name them get the new ids. Returns the notes for
/// the import list: what was taken from the file, what could not be read and what could not be kept. A motor of the catalogue
/// stays the catalogue's.
fn use_embedded_curves(ctx: &mut Ctx, imp: &OrkImport, rocket: &mut Rocket) -> Vec<String> {
    let mut notes = Vec::new();
    let mut parsed: Vec<(&str, Option<Motor>)> = Vec::new();
    let mut registered: Vec<MotorRef> = Vec::new();
    let mut names: Vec<&str> = Vec::new();
    for (reference, digest) in &imp.motor_digests {
        if find_motor(ctx.physics.motors(), reference).is_some() {
            continue;
        }
        let Some(curve) = imp.embedded_curves.iter().find(|c| &c.digest == digest) else { continue };
        if !parsed.iter().any(|(d, _)| *d == digest.as_str()) {
            let motor = match bowshock_motors::parse_rse(&curve.rse) {
                Ok(engines) => engines.into_iter().next(),
                Err(e) => {
                    let prefix: String = digest.chars().take(8).collect();
                    notes.push(format!("thrust curve {prefix}… in the file could not be read: {e}"));
                    None
                }
            };
            parsed.push((digest.as_str(), motor));
        }
        let Some((_, Some(curve))) = parsed.iter().find(|(d, _)| *d == digest.as_str()) else { continue };
        let motor = curve.renamed(&reference.designation, &reference.manufacturer);
        registered.push(motor_ref(&motor));
        names.push(&reference.designation);
        notes.extend(ctx.physics.add_embedded(motor));
    }
    if !registered.is_empty() {
        let n = registered.len();
        notes.push(if n == 1 {
            format!("1 motor uses a thrust curve stored in the file ({})", names[0])
        } else {
            format!("{n} motors use thrust curves stored in the file ({})", names.join(", "))
        });
        carry_ids(rocket, &registered);
    }
    notes
}

/// Makes the references of `rocket` (every configuration's motors and every flight's own motor) that name one of `motors` and
/// carry no id name it exactly.
fn carry_ids(rocket: &mut Rocket, motors: &[MotorRef]) {
    let exact = |m: &MotorRef| -> Option<MotorRef> {
        if !m.id.is_empty() {
            return None;
        }
        motors.iter().find(|r| same_motor_name(&r.designation, &r.manufacturer, &m.designation, &m.manufacturer)).cloned()
    };
    let mut slots = Vec::new();
    for config in rocket.configurations() {
        for slot in &config.motors {
            if let Some(motor) = slot.motor.as_ref().and_then(exact) {
                slots.push((config.id, slot.mount, motor));
            }
        }
    }
    for (config, mount, motor) in slots {
        let _ = rocket.set_motor(config, mount, Some(motor));
    }
    let mut flights = rocket.flights().to_vec();
    for flight in &mut flights {
        if let FlightMotor::Motor(m) = &flight.motor {
            if let Some(motor) = exact(m) {
                flight.motor = FlightMotor::Motor(motor);
            }
        }
    }
    rocket.set_flights(flights);
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
        Ok(()) => {
            ctx.notify(format!("Saved {}.{EXTENSION}", ctx.doc.title()));
            ctx.recent.push(path);
        }
        Err(e) => ctx.notify(format!("Could not save: {e}")),
    }
}

/// The import dialog (OpenRocket `.ork` only); the answer arrives as [`Message::OpenPath`], which imports by extension.
pub(super) fn pick_import() -> Task<Message> {
    dialogs::pick_file("Import OpenRocket design", &[("OpenRocket design", &["ork"])], Message::OpenPath)
}

/// The suffix an imported design's name gets, so the top bar says where the untitled document came from.
const IMPORTED: &str = " (imported)";

/// How many recent files the File menu keeps.
pub const RECENT_MAX: usize = 5;

/// The most recently opened or saved documents, newest first, persisted one path per line.
#[derive(Debug, Clone, Default)]
pub struct Recent {
    list: Vec<PathBuf>,
    file: Option<PathBuf>,
}

impl Recent {
    /// The list stored at `file` (empty when it is missing); later changes are written back to it.
    pub fn load(file: Option<PathBuf>) -> Self {
        let text = file.as_ref().and_then(|f| std::fs::read_to_string(f).ok()).unwrap_or_default();
        let list = text.lines().filter(|l| !l.is_empty()).map(PathBuf::from).take(RECENT_MAX).collect();
        Recent { list, file }
    }

    /// The paths, newest first.
    pub fn paths(&self) -> &[PathBuf] {
        &self.list
    }

    /// Moves `path` to the front (dropping an earlier entry for it), keeps [`RECENT_MAX`] and persists.
    pub fn push(&mut self, path: &Path) {
        let path = std::path::absolute(path).unwrap_or_else(|_| path.to_path_buf());
        self.list.retain(|p| *p != path);
        self.list.insert(0, path);
        self.list.truncate(RECENT_MAX);
        if let Some(file) = &self.file {
            if let Some(dir) = file.parent() {
                let _ = std::fs::create_dir_all(dir);
            }
            let text: String = self.list.iter().map(|p| format!("{}\n", p.display())).collect();
            let _ = std::fs::write(file, text);
        }
    }
}

/// The open dialog; the answer arrives as [`Message::OpenPath`].
pub(super) fn pick_open() -> Task<Message> {
    dialogs::pick_file(
        "Open rocket",
        &[("Bowshock or OpenRocket", &[EXTENSION, "ork"]), ("Bowshock document", &[EXTENSION]), ("OpenRocket design", &["ork"])],
        Message::OpenPath,
    )
}

/// The save dialog; the answer arrives as [`Message::SavePath`].
pub(super) fn pick_save(ctx: &Ctx) -> Task<Message> {
    let name = format!("{}.{EXTENSION}", ctx.doc.title());
    dialogs::save_file(None, "Save rocket", name, ("Bowshock document", &[EXTENSION]), Message::SavePath)
}

/// The "Save changes to <name>?" modal, centred over a dimmed backdrop that takes every click; `None` while no command waits on
/// an answer. Save (Enter) is the one primary action, Don't save is destructive, Cancel (Esc) goes back to the document.
pub(super) fn unsaved_modal(ctx: &Ctx) -> Option<iced::Element<'_, Message>> {
    use iced::widget::{row, text, Space};
    use iced::{Fill, Widget};

    use crate::theme;
    use crate::tokens::{SPACE_2, TEXT_BODY};
    use crate::widgets::button::{button, Weight};

    ctx.unsaved.as_ref()?;
    let nothing = || Message::Chrome(crate::chrome::Message::Nothing);
    let body = text("Your changes will be lost if you don't save them.").size(TEXT_BODY).style(theme::muted);
    let discard: iced::Element<'_, Message> = button("Don't save", Weight::Destructive).on_press(Message::UnsavedDiscard).into();
    let cancel: iced::Element<'_, Message> = button("Cancel", Weight::Secondary).key("Esc").on_press(Message::UnsavedCancel).into();
    let save: iced::Element<'_, Message> = button("Save", Weight::Primary).key("↵").on_press(Message::UnsavedSave).into();
    let buttons = row![discard, Space::new().width(Fill), cancel, save].spacing(SPACE_2).align_y(iced::Center);
    Some(crate::widgets::modal::modal(format!("Save changes to {}?", ctx.doc.title()), body.boxed(), buttons.boxed(), 420.0, nothing(), nothing()))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Opening an `.ork` imports it as a new untitled document and says so in the status strip.
    #[test]
    fn opening_an_ork_imports_it_as_an_untitled_document() {
        let ork = Path::new(env!("CARGO_MANIFEST_DIR")).join("../bowshock-model/tests/data/mini_two_stage.ork");
        let mut ctx = Ctx::new();
        open(&mut ctx, &ork);
        assert_eq!(ctx.doc.rocket().name, "Mini two-stage (imported)");
        assert_eq!(ctx.doc.title(), "Mini two-stage (imported)");
        assert!(ctx.doc.path().is_none(), "an import has no .bowshock path until it is saved");
        assert!(!ctx.doc.is_dirty());
        assert_eq!(ctx.doc.rocket().configurations().len(), 2);
        let notice = ctx.notice.as_ref().map(|n| n.0.clone()).unwrap_or_default();
        assert!(notice.starts_with("Imported from OpenRocket"), "{notice}");
        // A file that is not an OpenRocket design is reported, and the document is kept.
        let bad = std::env::temp_dir().join(format!("bowshock-bad-{}.ork", std::process::id()));
        std::fs::write(&bad, "nope").unwrap();
        open(&mut ctx, &bad);
        std::fs::remove_file(&bad).ok();
        assert_eq!(ctx.doc.rocket().name, "Mini two-stage (imported)");
        assert!(ctx.notice.as_ref().is_some_and(|n| n.0.starts_with("Could not import")));
    }
    use bowshock_model::{DeployEvent, Deployment, InternalKind, MassRole, MaterialId, NoseShape, PartKind, RadialKind};
    use bowshock_motors::MotorDb;
    use bowshock_sim::from_model::{self, Level, Phase};

    /// The example rocket, built through the model API: the kit's `hpr_l1_cert`, a 76.2 mm high-power rocket on an H128W.
    fn build_example() -> Rocket {
        let mut r = Rocket::default();
        r.name = "hpr_l1_cert".to_string();
        r.designer = "Bowshock".to_string();
        r.insert(0, PartKind::NoseCone { shape: NoseShape::TangentOgive, length: 0.2286, diameter: 0.0762, tip_radius: 0.0 }).unwrap();
        let tube = r.insert(1, PartKind::BodyTube { length: 0.9134, diameter: 0.0762 }).unwrap();
        r.set_name(tube, Some("Body tube".to_string()));

        // Three trapezoidal fins flush with the aft end of the airframe, a pair of rail buttons.
        let fins = RadialKind::FinSet { root_chord: 0.120, tip_chord: 0.050, span: 0.0762, sweep: 0.060, thickness: 0.00318 };
        r.attach_radial(tube, 0.9134 - 0.120, 0.0, 3, fins).unwrap();
        let button = RadialKind::RailButton { height: 0.008, diameter: 0.011 };
        r.attach_radial(tube, 0.380, 0.0, 1, button.clone()).unwrap();
        r.attach_radial(tube, 0.620, 0.0, 1, button).unwrap();

        // The 38 mm motor mount with a centering ring at each end.
        let mount = r
            .attach_internal(tube, 0.5734, InternalKind::InnerTube { outer_diameter: 0.042, thickness: 0.002, length: 0.340, motor_mount: true, overhang: 0.0 })
            .unwrap();
        let ring = InternalKind::CenteringRing { outer_diameter: None, inner_diameter: None, thickness: 0.006 };
        r.attach_internal(tube, 0.5734, ring.clone()).unwrap();
        r.attach_internal(tube, 0.9134 - 0.006, ring).unwrap();

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
        r.attach_internal(tube, 0.250, chute).unwrap();
        r.attach_internal(tube, 0.150, InternalKind::ShockCord { length: 2.0, packed_length: 0.080 }).unwrap();
        r.attach_internal(tube, 0.470, InternalKind::MassComponent { mass: 0.045, length: 0.060, diameter: Some(0.030), role: MassRole::Altimeter }).unwrap();
        r.attach_internal(tube, 0.020, InternalKind::MassComponent { mass: 0.120, length: 0.040, diameter: None, role: MassRole::Ballast }).unwrap();

        // Configuration H128W (named after its motor).
        let db = MotorDb::bundled();
        let motor = db.find("H128W").expect("H128W is bundled");
        let id = r.active_config_id();
        r.set_motor(id, mount, Some(bowshock_model::MotorRef::new(motor.designation.clone(), motor.manufacturer.clone()))).unwrap();
        r.set_motor_delay(id, mount, 14.0).unwrap();
        // The kit's four flights (Nominal, Windy, Hot day, Bigger motor) ship with the example; a new document has only Nominal.
        r.set_flights(bowshock_model::example_flights());
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
            std::fs::write(&path, built.to_bytes()).unwrap();
        }
        let shipped = Rocket::from_bytes(EXAMPLE).expect("the shipped example loads");
        assert_eq!(shipped, built, "assets/examples/hpr_l1_cert.bowshock is stale: regenerate it (see this test's docs)");
    }

    /// The committed version-1 golden file (`bowshock-model/tests/data/v1_example.bowshock`) is the example as first written
    /// (nose + two tubes); it is frozen and never regenerated (see the model's `format` module docs), so it loads and
    /// round-trips but no longer equals the current example.
    #[test]
    fn the_v1_golden_file_loads_and_round_trips() {
        let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("../bowshock-model/tests/data/v1_example.bowshock");
        let golden = Rocket::from_bytes(&std::fs::read(&path).unwrap()).expect("the golden file loads");
        assert_eq!(golden.name, "hpr_l1_cert");
        assert_eq!(golden.parts().len(), 3);
        assert_eq!(Rocket::from_bytes(&golden.to_bytes()).unwrap(), golden);
    }

    #[test]
    fn the_example_is_the_kits_rocket_and_flies_stable() {
        let rocket = Rocket::from_bytes(EXAMPLE).unwrap();
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
        r.insert(0, PartKind::NoseCone { shape: NoseShape::TangentOgive, length: 0.2286, diameter: 0.0762, tip_radius: 0.0 }).unwrap();
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
        r.set_flights(bowshock_model::example_flights().into_iter().take(3).collect());
        r
    }

    /// Keeps `assets/examples/two_stage.bowshock` in step with [`build_two_stage`] (regenerate like the first example) and checks
    /// the model finds the separation behind the sustainer's motor.
    #[test]
    fn two_stage_example_is_current() {
        let built = build_two_stage();
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("assets/examples/two_stage.bowshock");
        if std::env::var_os("BOWSHOCK_WRITE_EXAMPLE").is_some() {
            std::fs::write(&path, built.to_bytes()).unwrap();
        }
        let shipped = Rocket::from_bytes(TWO_STAGE).expect("the shipped example loads");
        assert_eq!(shipped, built, "assets/examples/two_stage.bowshock is stale: regenerate it (see example_is_current)");
        let stages = shipped.stages();
        assert_eq!(stages.len(), 2);
        let separation = stages[0].separation.expect("a joint behind the sustainer's motor");
        assert!((separation.station - 0.8706).abs() < 1e-9);
    }
}

#[cfg(test)]
mod menu_tests {
    use super::*;

    fn temp(name: &str) -> PathBuf {
        std::env::temp_dir().join(format!("bowshock-{}-{name}", std::process::id()))
    }

    /// Newest first, no duplicates, at most five, and it survives a reload.
    #[test]
    fn recent_files_order_dedupe_and_persist() {
        let file = temp("recent.txt");
        std::fs::remove_file(&file).ok();
        let mut recent = Recent::load(Some(file.clone()));
        assert!(recent.paths().is_empty());
        for n in 0..7 {
            recent.push(Path::new(&format!("/tmp/r{n}.bowshock")));
        }
        recent.push(Path::new("/tmp/r4.bowshock"));
        let names: Vec<String> = recent.paths().iter().map(|p| p.file_name().unwrap().to_string_lossy().into_owned()).collect();
        assert_eq!(names, ["r4.bowshock", "r6.bowshock", "r5.bowshock", "r3.bowshock", "r2.bowshock"]);
        let again = Recent::load(Some(file.clone()));
        std::fs::remove_file(&file).ok();
        assert_eq!(again.paths(), recent.paths());
    }

    /// Saving clears the dirty dot and says what was saved; the saved file enters the recent list.
    #[test]
    fn save_clears_dirty_and_reports() {
        let path = temp("hpr_l1_cert.bowshock");
        let mut ctx = Ctx::new();
        example(&mut ctx);
        ctx.doc.edit("Rename", |r| r.name = "x".to_string());
        assert!(ctx.doc.is_dirty());
        save_to(&mut ctx, &path);
        std::fs::remove_file(&path).ok();
        assert!(!ctx.doc.is_dirty());
        assert_eq!(ctx.notice.as_ref().map(|n| n.0.clone()), Some(format!("Saved {}.{EXTENSION}", ctx.doc.title())));
        assert_eq!(ctx.recent.paths().len(), 1);
    }

    /// The export and print dialogs hold the app: other commands are swallowed, Escape closes them, and an empty rocket opens none.
    #[test]
    fn the_export_and_print_dialogs_are_modal() {
        use crate::app::App;
        use crate::{export, print};
        let mut app = App::with_example();
        let _ = app.update(Message::Export(export::Message::Open(export::Kind::Image)));
        assert!(app.export.is_open());
        let _ = app.update(Message::New);
        assert!(!app.ctx().doc.rocket().parts().is_empty(), "New is swallowed while the dialog is up");
        let _ = app.update(Message::Cancel);
        assert!(!app.export.is_open());
        let _ = app.update(Message::Print(print::Message::Open));
        assert!(app.print.is_open());
        let _ = app.update(Message::Cancel);
        assert!(!app.print.is_open());
        let _ = app.update(Message::New);
        let _ = app.update(Message::Export(export::Message::Open(export::Kind::Model)));
        assert!(!app.export.is_open(), "nothing to export from an empty rocket");
    }

    /// A ZIP of stored entries (the importer does not check CRCs).
    fn stored_zip(entries: &[(&str, &[u8])]) -> Vec<u8> {
        let (mut zip, mut central) = (Vec::new(), Vec::new());
        for (name, data) in entries {
            let offset = zip.len() as u32;
            for (bytes, header) in [(&mut zip, true), (&mut central, false)] {
                bytes.extend_from_slice(if header { &[0x50, 0x4b, 3, 4] } else { &[0x50, 0x4b, 1, 2, 20, 0] });
                bytes.extend_from_slice(&[20, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]);
                bytes.extend_from_slice(&(data.len() as u32).to_le_bytes());
                bytes.extend_from_slice(&(data.len() as u32).to_le_bytes());
                bytes.extend_from_slice(&(name.len() as u16).to_le_bytes());
                bytes.extend_from_slice(&[0, 0]);
                if !header {
                    bytes.extend_from_slice(&[0; 10]);
                    bytes.extend_from_slice(&offset.to_le_bytes());
                }
                bytes.extend_from_slice(name.as_bytes());
                if header {
                    bytes.extend_from_slice(data);
                }
            }
        }
        let start = zip.len() as u32;
        zip.extend_from_slice(&central);
        zip.extend_from_slice(&[0x50, 0x4b, 5, 6, 0, 0, 0, 0]);
        zip.extend_from_slice(&(entries.len() as u16).to_le_bytes());
        zip.extend_from_slice(&(entries.len() as u16).to_le_bytes());
        zip.extend_from_slice(&(central.len() as u32).to_le_bytes());
        zip.extend_from_slice(&start.to_le_bytes());
        zip.extend_from_slice(&[0, 0]);
        zip
    }

    /// A design whose configurations fly the given `(designation, manufacturer, digest)` motors from one mount.
    fn ork_with(motors: &[(&str, &str, &str)]) -> String {
        let ids = ["c1", "c2", "c3", "c4"];
        let configs: String = ids
            .iter()
            .take(motors.len())
            .enumerate()
            .map(|(i, id)| format!("<motorconfiguration configid=\"{id}\"{}/>", if i == 0 { " default=\"true\"" } else { "" }))
            .collect();
        let motors: String = motors
            .iter()
            .zip(ids)
            .map(|((d, m, g), id)| {
                format!("<motor configid=\"{id}\"><manufacturer>{m}</manufacturer><digest>{g}</digest><designation>{d}</designation><diameter>0.038</diameter><length>0.356</length><delay>none</delay></motor>")
            })
            .collect();
        format!(
            "<openrocket version=\"1.11\"><rocket><name>t</name>{configs}<subcomponents><stage><name>s</name><subcomponents>\
             <nosecone><name>n</name><material type=\"bulk\" density=\"1050\">p</material><length>0.2</length><thickness>0.002</thickness><shape>ogive</shape><shapeparameter>1.0</shapeparameter><aftradius>0.025</aftradius></nosecone>\
             <bodytube><name>body</name><material type=\"bulk\" density=\"680.0\">Cardboard</material><length>0.86</length><radius>0.0508</radius><thickness>0.0013</thickness><subcomponents>\
             <innertube><name>mount</name><axialoffset method=\"bottom\">0.0</axialoffset><length>0.6</length><outerradius>0.0205</outerradius><thickness>0.0015</thickness>\
             <motormount><ignitionevent>automatic</ignitionevent><overhang>0.0</overhang>{motors}</motormount></innertube>\
             </subcomponents></bodytube></subcomponents></stage></subcomponents></rocket></openrocket>"
        )
    }

    const RSE: &str = "<engine-database><engine-list><engine mfg=\"AeroTech\" code=\"Q32N\" dia=\"38\" len=\"356\" initWt=\"647\" propWt=\"380.5\" delays=\"1000\">\
        <data><eng-data t=\"0\" f=\"0\" m=\"647\"/><eng-data t=\"1\" f=\"30\" m=\"500\"/><eng-data t=\"20\" f=\"0\" m=\"266.5\"/></data></engine></engine-list></engine-database>";

    fn open_zip(ctx: &mut Ctx, tag: &str, ork: &str, curves: &[(&str, &str)]) {
        let names: Vec<String> = curves.iter().map(|(d, _)| format!("thrustcurves/{d}.rse")).collect();
        let mut entries: Vec<(&str, &[u8])> = vec![("rocket.ork", ork.as_bytes())];
        entries.extend(names.iter().zip(curves).map(|(n, (_, text))| (n.as_str(), text.as_bytes())));
        let path = std::env::temp_dir().join(format!("bowshock-embedded-{tag}-{}.ork", std::process::id()));
        std::fs::write(&path, stored_zip(&entries)).unwrap();
        open(ctx, &path);
        std::fs::remove_file(&path).ok();
    }

    /// The thrust curves stored in an `.ork` fly the motors that carry their digest: every name of the digest is a user motor with
    /// the curve's data, the document's references name them by id, and nothing is fetched for them.
    #[test]
    fn an_ork_with_stored_thrust_curves_flies_them_without_a_lookup() {
        let mut ctx = Ctx::new();
        ctx.env.fetch_motors = true;
        let ork = ork_with(&[("Q33N", "AeroTech", "d1"), ("Q32N", "AeroTech", "d1"), ("Q34N", "Kuhns", "d1")]);
        open_zip(&mut ctx, "all", &ork, &[("d1", RSE)]);
        let rocket = ctx.doc.rocket();
        assert_eq!(rocket.configurations().len(), 3);
        let db = ctx.physics.motors();
        let curve = bowshock_motors::parse_rse(RSE).unwrap().remove(0);
        for (designation, manufacturer) in [("Q33N", "AeroTech"), ("Q32N", "AeroTech"), ("Q34N", "Kuhns")] {
            let reference = rocket
                .configurations()
                .iter()
                .flat_map(|c| c.motors.iter().filter_map(|s| s.motor.clone()))
                .find(|m| m.designation == designation)
                .unwrap_or_else(|| panic!("{designation} is not in a configuration"));
            assert!(reference.id.starts_with("eng:") && reference.manufacturer == manufacturer, "{reference:?}");
            let motor = find_motor(db, &reference).unwrap_or_else(|| panic!("{designation} does not resolve"));
            assert_eq!(motor.id, reference.id);
            assert_eq!((motor.designation.as_str(), motor.manufacturer.as_str()), (designation, manufacturer));
            assert_eq!(motor.points, curve.points);
            assert!((motor.total_impulse() - curve.total_impulse()).abs() < 1e-9 && motor.total_impulse() > 100.0);
            assert!((motor.total_mass - 0.647).abs() < 1e-9 && (motor.propellant_mass - 0.3805).abs() < 1e-9);
            let index = db.position(&motor.id).unwrap();
            assert_eq!(ctx.physics.motor_origin(index), super::super::physics::MotorOrigin::File);
        }
        assert!(super::super::motor_fetch::missing(rocket, db).is_empty());
        ctx.physics.sync(ctx.doc.rocket(), ctx.doc.revision(), ctx.units);
        assert!(!ctx.physics.warnings().iter().any(|w| w.message.contains("not in the motor database")), "{:?}", ctx.physics.warnings());
        assert!(ctx.import_notes.iter().any(|n| n == "3 motors use thrust curves stored in the file (Q33N, Q32N, Q34N)"), "{:?}", ctx.import_notes);
        let _ = super::super::motor_fetch::start(&mut ctx);
        assert!(!ctx.notice.as_ref().is_some_and(|n| n.0.starts_with("Fetching")), "no lookup for motors the file carries");
    }

    /// A catalogue motor keeps the catalogue entry, and a digest without a stored curve stays missing (and is looked up).
    #[test]
    fn a_catalogue_motor_or_an_unstored_digest_is_not_taken_from_the_file() {
        let mut ctx = Ctx::new();
        let ork = ork_with(&[("H128W", "AeroTech", "d1"), ("X999Q", "Nobody", "d9"), ("Q32N", "AeroTech", "d1")]);
        open_zip(&mut ctx, "mixed", &ork, &[("d1", RSE)]);
        let rocket = ctx.doc.rocket();
        let named =
            |d: &str| rocket.configurations().iter().flat_map(|c| c.motors.iter().filter_map(|s| s.motor.clone())).find(|m| m.designation == d).unwrap();
        assert!(named("H128W").id.is_empty(), "the catalogue's H128W stays a name");
        assert!(named("X999Q").id.is_empty());
        assert!(named("Q32N").id.starts_with("eng:"));
        assert_eq!(super::super::motor_fetch::missing(rocket, ctx.physics.motors()), vec![MotorRef::new("X999Q", "Nobody")]);
        assert!(ctx.import_notes.iter().any(|n| n == "1 motor uses a thrust curve stored in the file (Q32N)"), "{:?}", ctx.import_notes);
        // A curve that cannot be read is noted, and its motor is left to the lookup.
        let ork = ork_with(&[("Q35N", "AeroTech", "0123456789abcdef")]);
        open_zip(&mut ctx, "broken", &ork, &[("0123456789abcdef", "<engine-database/>")]);
        assert!(ctx.import_notes.iter().any(|n| n.starts_with("thrust curve 01234567… in the file could not be read: ")), "{:?}", ctx.import_notes);
        assert_eq!(super::super::motor_fetch::missing(ctx.doc.rocket(), ctx.physics.motors()), vec![MotorRef::new("Q35N", "AeroTech")]);
    }
}
