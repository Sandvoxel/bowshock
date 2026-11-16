//! File commands: New, Open, Save, Save As and the built-in example, with the unsaved-changes guard.

use std::path::{Path, PathBuf};

use bowshock_model::Rocket;
use iced::Task;

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
    let source = if std::env::var("BOWSHOCK_EXAMPLE").is_ok_and(|v| v == "two_stage") { TWO_STAGE } else { EXAMPLE };
    match Rocket::from_bytes(source) {
        Ok(rocket) => {
            ctx.doc.replace(rocket, None);
            ctx.loaded();
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
                let mut rocket = imp.rocket;
                if !rocket.name.ends_with(IMPORTED) {
                    rocket.name = if rocket.name.is_empty() { format!("untitled{IMPORTED}") } else { format!("{}{IMPORTED}", rocket.name) };
                }
                ctx.doc.replace(rocket, None);
                ctx.loaded();
                ctx.notify(match imp.warnings.len() {
                    0 => "Imported from OpenRocket".to_string(),
                    1 => "Imported from OpenRocket · 1 note".to_string(),
                    n => format!("Imported from OpenRocket · {n} notes"),
                });
                ctx.import_notes = imp.warnings.iter().map(ToString::to_string).collect();
            }
            Err(e) => ctx.notify(format!("Could not import {}: {e}", path.display())),
        }
        return;
    }
    match ctx.doc.load(path) {
        Ok(()) => {
            ctx.loaded();
            ctx.recent.push(path);
        }
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
        Ok(()) => {
            ctx.notify(format!("Saved {}.{EXTENSION}", ctx.doc.title()));
            ctx.recent.push(path);
        }
        Err(e) => ctx.notify(format!("Could not save: {e}")),
    }
}

/// The import dialog (OpenRocket `.ork` only); the answer arrives as [`Message::OpenPath`], which imports by extension.
pub(super) fn pick_import() -> Task<Message> {
    Task::perform(
        async {
            let file = rfd::AsyncFileDialog::new().set_title("Import OpenRocket design").add_filter("OpenRocket design", &["ork"]).pick_file().await;
            file.map(|f| f.path().to_path_buf())
        },
        Message::OpenPath,
    )
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
    /// Where the list lives: `$BOWSHOCK_RECENT_FILE`, else `~/.config/bowshock/recent.txt` (next to the user motors directory).
    pub fn default_file() -> Option<PathBuf> {
        let over = std::env::var_os("BOWSHOCK_RECENT_FILE").filter(|s| !s.is_empty()).map(PathBuf::from);
        over.or_else(|| std::env::var_os("HOME").map(|h| PathBuf::from(h).join(".config").join("bowshock").join("recent.txt")))
    }

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
    Task::perform(
        async {
            let file = rfd::AsyncFileDialog::new()
                .set_title("Open rocket")
                .add_filter("Bowshock or OpenRocket", &[EXTENSION, "ork"])
                .add_filter("Bowshock document", &[EXTENSION])
                .add_filter("OpenRocket design", &["ork"])
                .pick_file()
                .await;
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

/// Width of the File menu panel.
const MENU_W: f32 = 300.0;
/// Where the panel's left edge sits: under the file name in the top bar.
const MENU_X: f32 = 148.0;

/// The modifier a shortcut hint is written with.
const MOD: &str = if cfg!(target_os = "macos") { "⌘" } else { "Ctrl+" };
/// Shift's glyph in a hint.
const SHIFT: &str = if cfg!(target_os = "macos") { "⇧" } else { "Shift+" };

/// One File menu row, as `(label, shortcut hint, what it sends)`; `None` is a greyed row.
type Row = (String, String, Option<Message>);

/// The rows of the File menu: the commands, then the recent files.
fn menu_rows(recent: &Recent) -> (Vec<Row>, Vec<Row>) {
    let m = |label: &str, hint: String, message: Message| (label.to_string(), hint, Some(message));
    let commands = vec![
        m("New", format!("{MOD}N"), Message::New),
        m("Open…", format!("{MOD}O"), Message::Open),
        m("Import OpenRocket (.ork)…", String::new(), Message::ImportOrk),
        m("Save", format!("{MOD}S"), Message::Save),
        m("Save as…", format!("{SHIFT}{MOD}S"), Message::SaveAs),
        m("Print / export PDF…", format!("{MOD}P"), Message::Print(crate::print::Message::Open)),
        m("Export 3D model…", String::new(), Message::Export(crate::export::Message::Open(crate::export::Kind::Model))),
        m("Export image…", String::new(), Message::Export(crate::export::Message::Open(crate::export::Kind::Image))),
        m("Open example", format!("{MOD}E"), Message::Example),
    ];
    let recents = recent
        .paths()
        .iter()
        .map(|p| {
            let name = p.file_name().map(|n| n.to_string_lossy().into_owned()).unwrap_or_else(|| p.display().to_string());
            let dir = p.parent().and_then(|d| d.file_name()).map(|n| n.to_string_lossy().into_owned()).unwrap_or_default();
            (name, dir, p.exists().then(|| Message::OpenRecent(p.clone())))
        })
        .collect();
    (commands, recents)
}

/// The File menu, hung from the file name in the top bar; `None` while it is closed. A click anywhere else closes it.
pub(super) fn menu(ctx: &Ctx) -> Option<iced::Element<'_, Message>> {
    use iced::widget::{button, column, container, mouse_area, row, text, Space, Stack};
    use iced::{Background, Center, Element, Fill, Padding, Theme, Widget};

    use crate::theme::{self, MONO};
    use crate::tokens::TOPBAR_H;

    if !ctx.file_menu {
        return None;
    }
    let item_style = |theme: &Theme, status: button::Status| {
        let t = theme::tokens(theme);
        let hovered = matches!(status, button::Status::Hovered | button::Status::Pressed);
        let background = hovered.then_some(Background::Color(t.bg_400));
        let text_color = if matches!(status, button::Status::Disabled) { t.ink_faint } else { t.ink };
        button::Style { background, text_color, border: iced::Border::default(), ..button::Style::default() }
    };
    let rule = || container(Space::new()).width(Fill).height(1).style(theme::hairline);
    let row_view = |(label, hint, message): Row| -> Element<'_, Message> {
        let line = row![text(label).size(12.5), Space::new().width(Fill), text(hint).font(MONO).size(11.5).style(theme::muted)].align_y(Center);
        let mut b = button(line).width(Fill).height(26).padding([0.0, 10.0]).style(item_style);
        if let Some(message) = message {
            b = b.on_press(message);
        }
        b.boxed()
    };
    let (commands, recents) = menu_rows(&ctx.recent);
    let mut col = column![];
    for r in commands {
        col = col.push(row_view(r));
    }
    col = col.push(rule().boxed());
    col = col.push(container(super::ui::label("RECENT")).padding([6.0, 10.0]).boxed());
    if recents.is_empty() {
        col = col.push(container(text("No recent files").size(12.5).style(theme::faint)).padding([4.0, 10.0]).boxed());
    }
    for r in recents {
        col = col.push(row_view(r));
    }
    let panel = container(mouse_area(col).on_press(Message::Chrome(crate::chrome::Message::Nothing))).width(MENU_W).style(theme::overlay);
    // `Idle` (not the default `None`) makes the stack hide the cursor from the page beneath, so hovering the parts bin or the
    // viewport while the menu is open does not reach their hover handlers (which would close the menu).
    let scrim = mouse_area(Space::new().width(Fill).height(Fill)).interaction(iced::mouse::Interaction::Idle).on_press(Message::ToggleFileMenu);
    let placed = container(panel).width(Fill).height(Fill).padding(Padding { top: TOPBAR_H, left: MENU_X, ..Padding::ZERO });
    Some(Stack::with_children([scrim.boxed(), placed.boxed()]).boxed())
}

/// The "Save changes to <name>?" modal, centred over a dimmed backdrop that takes every click; `None` while no command waits on
/// an answer. Save (Enter) is the one primary action, Don't save is destructive, Cancel (Esc) goes back to the document.
pub(super) fn unsaved_modal(ctx: &Ctx) -> Option<iced::Element<'_, Message>> {
    use iced::widget::{column, container, mouse_area, row, text, Space, Stack};
    use iced::{Background, Color, Fill, Widget};

    use crate::theme::{self, SANS_MEDIUM};
    use crate::tokens::{SPACE_2, SPACE_3, TEXT_BODY};
    use crate::widgets::button::{button, Weight};

    ctx.unsaved.as_ref()?;
    let nothing = || Message::Chrome(crate::chrome::Message::Nothing);
    let backdrop = container(Space::new().width(Fill).height(Fill))
        .style(|_| container::Style { background: Some(Background::Color(Color::from_rgba(0.0, 0.0, 0.0, 0.55))), ..container::Style::default() });
    let title = text(format!("Save changes to {}?", ctx.doc.title())).font(SANS_MEDIUM).size(15);
    let body = text("Your changes will be lost if you don't save them.").size(TEXT_BODY).style(theme::muted);
    let discard: iced::Element<'_, Message> = button("Don't save", Weight::Destructive).on_press(Message::UnsavedDiscard).into();
    let cancel: iced::Element<'_, Message> = button("Cancel", Weight::Secondary).key("Esc").on_press(Message::UnsavedCancel).into();
    let save: iced::Element<'_, Message> = button("Save", Weight::Primary).key("↵").on_press(Message::UnsavedSave).into();
    let buttons = row![discard, Space::new().width(Fill), cancel, save].spacing(SPACE_2).align_y(iced::Center);
    let card =
        container(mouse_area(column![title, body, buttons].spacing(SPACE_3 + 2.0)).on_press(nothing())).width(420).padding(SPACE_3 + 8.0).style(theme::overlay);
    let placed = container(card).center(Fill);
    Some(Stack::with_children([mouse_area(backdrop).on_press(nothing()).boxed(), placed.boxed()]).boxed())
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
        r.insert(0, PartKind::NoseCone { shape: NoseShape::TangentOgive, length: 0.2286, diameter: 0.0762 }).unwrap();
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

    /// The File menu's rows route to the right commands, in order.
    #[test]
    fn menu_rows_route_to_commands() {
        let (commands, _) = menu_rows(&Recent::default());
        let labels: Vec<&str> = commands.iter().map(|r| r.0.as_str()).collect();
        assert_eq!(
            labels,
            ["New", "Open…", "Import OpenRocket (.ork)…", "Save", "Save as…", "Print / export PDF…", "Export 3D model…", "Export image…", "Open example"]
        );
        let routes: Vec<bool> = vec![
            matches!(commands[0].2, Some(Message::New)),
            matches!(commands[1].2, Some(Message::Open)),
            matches!(commands[2].2, Some(Message::ImportOrk)),
            matches!(commands[3].2, Some(Message::Save)),
            matches!(commands[4].2, Some(Message::SaveAs)),
            matches!(commands[5].2, Some(Message::Print(crate::print::Message::Open))),
            matches!(commands[6].2, Some(Message::Export(crate::export::Message::Open(crate::export::Kind::Model)))),
            matches!(commands[7].2, Some(Message::Export(crate::export::Message::Open(crate::export::Kind::Image)))),
            matches!(commands[8].2, Some(Message::Example)),
        ];
        assert!(routes.iter().all(|&ok| ok), "{routes:?}");
        assert_eq!(commands[3].1, format!("{MOD}S"));
    }

    /// A recent file that is gone is greyed (no message); one that exists opens.
    #[test]
    fn missing_recent_files_are_greyed() {
        let real = temp("real.bowshock");
        std::fs::write(&real, "{}").unwrap();
        let mut recent = Recent::default();
        recent.push(&temp("gone.bowshock"));
        recent.push(&real);
        let (_, rows) = menu_rows(&recent);
        std::fs::remove_file(&real).ok();
        assert!(matches!(rows[0].2, Some(Message::OpenRecent(_))));
        assert!(rows[1].2.is_none(), "a missing file is greyed");
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
}
