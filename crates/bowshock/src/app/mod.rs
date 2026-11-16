//! The application: one [`App`] holding the shared context ([`Ctx`]) and one state per area, one top-level [`Message`] that wraps
//! each area's own, and the routing between them. See `ARCHITECTURE.md` next to `main.rs` for the contract every module follows.

pub mod document;
pub mod drag;
mod files;
pub mod live;
mod multi;
pub mod part_menu;
pub mod physics;
pub mod refs;
pub mod scale;
pub mod settings;
pub mod symmetry;
pub mod ui;

use std::path::PathBuf;
use std::time::{Duration, Instant};

use bowshock_model::PartId;
use iced::widget::{column, mouse_area};
use iced::{keyboard, mouse, Element, Point, Task, Theme, Widget};

use crate::build::{self, inspector, lanes, parts_bin};
use crate::chrome;
use crate::theme;
use crate::tokens::{self, Tokens};
use crate::units::UnitSystem;
use crate::viewport;
use crate::{analyze, export, print, simulate};
use document::Document;
use drag::Drags;
use physics::Physics;

/// How long a status message stays up.
const NOTICE_TIME: Duration = Duration::from_secs(3);

/// The three modes of the top bar.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mode {
    /// Build the rocket.
    Build,
    /// Define and run flights.
    Simulate,
    /// Aerodynamics across the speed range.
    Analyze,
}

/// What is selected, app-wide: the viewport, the lanes and the inspector all read and set it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Selection {
    /// Nothing: the inspector shows the rocket.
    #[default]
    None,
    /// A part of any class (axial, radial group or internal), by id.
    Part(PartId),
    /// The separation of stage `n` from the stage ahead of it (`Stage::index`, from 2).
    #[allow(dead_code)] // Selected by the separation diamond in the viewport and the lane line.
    Separation(usize),
}

impl Selection {
    /// The selected part, if a part is selected.
    pub fn part(self) -> Option<PartId> {
        match self {
            Selection::Part(id) => Some(id),
            _ => None,
        }
    }
}

/// A file command held back by the "Save changes?" modal until the user answers.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) enum Pending {
    /// File > New.
    New,
    /// Open (the file dialog comes after the answer).
    Open,
    /// Open the built-in example.
    Example,
    /// Import an OpenRocket design (the file dialog comes after the answer).
    Import,
    /// Open recent file (the path is already known).
    Recent(std::path::PathBuf),
    /// Quit the application (the window was asked to close).
    Exit,
}

/// Everything the areas share: the document and its physics, the selection, drags in progress, the unit system and the
/// handful of app-wide settings. Modules receive it in `update(.., ctx: &mut Ctx)` and `view(.., ctx: &Ctx)`.
pub struct Ctx {
    /// The open document. Every change to the rocket goes through its `edit*` methods.
    pub doc: Document,
    /// What is selected.
    pub selection: Selection,
    /// Drags in progress, and the radial placement settings.
    pub drag: Drags,
    /// Balance, warnings and motor queries for the document, kept current by the app after every message.
    pub physics: Physics,
    /// The display unit system (top bar).
    pub units: UnitSystem,
    /// Where the user's settings (the unit set) are written; `None` keeps them for the session only.
    pub settings_file: Option<PathBuf>,
    /// The cursor in window coordinates, tracked app-wide so a drag can leave the parts bin.
    pub cursor: Option<Point>,
    /// The mode on show.
    pub mode: Mode,
    /// Dark theme (else light).
    pub dark: bool,
    /// "Compare all in lanes": the lanes draw every configuration's motor and CG.
    pub compare_configs: bool,
    /// Whether the viewport and the lanes draw the CG marker (the status strip's CG cell toggles it).
    pub show_cg: bool,
    /// Whether the viewport and the lanes draw the CP marker (the status strip's CP cell toggles it).
    pub show_cp: bool,
    /// A transient status message and when it expires.
    pub notice: Option<(String, Instant)>,
    /// What an OpenRocket import could not carry over exactly, one line each (the warnings chip lists them); cleared on every load.
    pub import_notes: Vec<String>,
    /// The File menu is open (the top bar's file name toggles it).
    pub file_menu: bool,
    /// The recently opened or saved documents (File menu).
    pub recent: files::Recent,
    /// Ask the viewport to frame the rocket once its mesh is current (set after adding a part or loading a document).
    pub camera_fit: bool,
    /// The document was just replaced: select nothing and let the gauge jump rather than animate.
    reloaded: bool,
    /// A file command that would discard unsaved changes, waiting on the "Save changes?" modal (which is open while this is set).
    unsaved: Option<Pending>,
    /// The same command while the Save As dialog the modal's Save opened is up; cancelling the dialog abandons it.
    saving_for: Option<Pending>,
    /// Which copy of the selected symmetry group is the clicked one, and a pending rejoin (see [`symmetry`]).
    pub symmetry: symmetry::SymmetryUi,
    /// Pick mode and the formula UI's transient state (see [`refs`]).
    pub refs: refs::Ui,
    /// The part just dropped or edited and when: the viewport outlines it and the lanes flash its bar (see
    /// [`crate::viewport::flash`]).
    pub flash: Option<(PartId, Instant)>,
    /// The parts selected along with `selection`'s primary one (see [`multi`]).
    multi: multi::Extra,
    /// The baseline of the drag in progress, for the status strip's semibold readouts (see [`live`]).
    live: live::Live,
    /// The balance of the part being carried as it would land (see [`live::Preview`]).
    pub preview: live::Preview,
    /// The part context menu (right-click on a part), if open (see [`part_menu`]).
    pub part_menu: Option<part_menu::Open>,
    /// The Scale overlay, if open (see [`scale`]).
    pub scale: Option<scale::Dialog>,
    /// The flight selected on the Simulate screen (the Optimize analysis starts from it).
    pub selected_flight: Option<bowshock_model::FlightId>,
}

impl Ctx {
    fn new() -> Self {
        Ctx {
            doc: Document::default(),
            selection: Selection::None,
            drag: Drags::default(),
            physics: Physics::new(),
            units: UnitSystem::default(),
            settings_file: None,
            cursor: None,
            mode: Mode::Build,
            dark: true,
            compare_configs: false,
            show_cg: true,
            show_cp: true,
            notice: None,
            import_notes: Vec::new(),
            recent: files::Recent::default(),
            file_menu: false,
            camera_fit: false,
            reloaded: false,
            unsaved: None,
            saving_for: None,
            symmetry: symmetry::SymmetryUi::default(),
            refs: refs::Ui::default(),
            flash: None,
            multi: multi::Extra::default(),
            live: live::Live::default(),
            preview: live::Preview::default(),
            part_menu: None,
            scale: None,
            selected_flight: None,
        }
    }

    /// The design tokens of the current theme.
    pub fn tokens(&self) -> &'static Tokens {
        if self.dark {
            &tokens::DARK
        } else {
            &tokens::LIGHT
        }
    }

    /// Shows `message` in the status strip for a few seconds.
    pub fn notify(&mut self, message: impl Into<String>) {
        self.notice = Some((message.into(), Instant::now() + NOTICE_TIME));
    }

    /// Whether anything needs frame ticks because of the context itself.
    fn animating(&self) -> bool {
        self.notice.is_some() || self.physics.animating() || (self.flash.is_some() && crate::viewport::animation_on())
    }

    /// Starts the one-time flash and settle of part `id` (it was just dropped or its edit was committed).
    pub fn flash_part(&mut self, id: PartId) {
        if crate::viewport::flash::enabled() {
            self.flash = Some((id, Instant::now()));
        }
    }

    /// The flash as the views draw it now: `(part, outline strength 0..1, height above its place in metres)`.
    pub fn flash_levels(&self) -> Option<(PartId, f32, f32)> {
        let (id, since) = self.flash?;
        let age = crate::viewport::flash::age(since, Instant::now());
        let level = crate::viewport::flash::level(age);
        (level > 0.0).then(|| (id, level, crate::viewport::flash::lift(age)))
    }

    /// Call after replacing the document wholesale (New, Open, the example).
    pub(crate) fn loaded(&mut self) {
        self.selection = Selection::None;
        self.flash = None;
        self.multi.clear();
        self.import_notes.clear();
        self.symmetry = symmetry::SymmetryUi::default();
        self.part_menu = None;
        self.scale = None;
        self.refs = refs::Ui::default();
        self.drag = Drags { symmetry: self.drag.symmetry, angle_snap: self.drag.angle_snap, ..Drags::default() };
        self.camera_fit = true;
        self.reloaded = true;
    }

    /// Drops a selection that no longer exists (after a delete, an undo, a load).
    fn sanitize_selection(&mut self) {
        // A part being carried is out of the rocket but still selected.
        if self.drag.active() {
            return;
        }
        let rocket = self.doc.rocket();
        let exists = match self.selection {
            Selection::None => true,
            Selection::Part(id) => rocket.kind_of(id).is_some(),
            Selection::Separation(stage) => self.physics.stages().iter().any(|s| s.index == stage && s.separation.is_some()),
        };
        if !exists {
            self.selection = Selection::None;
        }
        self.multi.validate(self.selection, self.doc.rocket());
        self.symmetry.validate(self.doc.rocket());
        if self.symmetry.instance.is_some_and(|(id, _)| self.doc.rocket().radial(id).is_none()) {
            self.symmetry.instance = None;
        }
    }
}

/// A key press, as the subscription reports it.
#[derive(Debug, Clone)]
pub struct KeyPress {
    /// The key.
    pub key: keyboard::Key,
    /// Modifier keys held.
    pub modifiers: keyboard::Modifiers,
    /// A widget (a focused text field, say) already handled it: plain-character shortcuts must not fire.
    pub captured: bool,
}

/// The top-level message: shared actions plus one variant per area, each wrapping that area's own `Message`.
#[derive(Debug, Clone)]
pub enum Message {
    /// A pane divider of the Build screen moved.
    Resized(iced::widget::pane_grid::ResizeEvent),
    /// The top bar's Build / Simulate / Analyze switch.
    Mode(Mode),
    /// Light / dark.
    ToggleTheme,
    /// The unit system select.
    Units(UnitSystem),
    /// Animation / notice clock tick.
    Frame(Instant),
    /// The cursor moved (window coordinates).
    Cursor(Point),
    /// Left button released anywhere nothing took it.
    Released,
    /// Escape: abandon the drag or press in progress and close menus.
    Cancel,
    /// F2: rename the selected part in the Inspector, else the active configuration.
    Rename,
    /// Right-click on a part (the viewport and the lanes have selected it): open its context menu at the cursor.
    OpenPartMenu(PartId),
    /// An entry of the part context menu (or a click outside it).
    PartMenu(part_menu::Action),
    /// The Scale overlay: open it, edit it, apply it.
    Scale(scale::Message),
    /// Any key press (the app maps global shortcuts and forwards the rest to the active area).
    Key(KeyPress),
    /// Delete / Backspace: remove the selected part.
    Delete,
    /// Undo the last document step.
    Undo,
    /// Redo the last undone step.
    Redo,
    /// New document.
    New,
    /// Open a document (shows the file dialog).
    Open,
    /// The open dialog finished.
    OpenPath(Option<PathBuf>),
    /// Save (to the document's path, or Save As when it has none).
    Save,
    /// Save As (shows the file dialog).
    SaveAs,
    /// The save dialog finished.
    SavePath(Option<PathBuf>),
    /// Load the built-in example rocket.
    Example,
    /// The top bar's file name: open or close the File menu.
    ToggleFileMenu,
    /// File menu > Import OpenRocket (.ork)…: shows the import dialog.
    ImportOrk,
    /// File menu > a recent file.
    OpenRecent(PathBuf),
    /// The "Save changes?" modal's *Save* (Enter): save, then run the held command.
    UnsavedSave,
    /// The modal's *Don't save*: run the held command and lose the changes.
    UnsavedDiscard,
    /// The modal's *Cancel* (Escape): keep the document.
    UnsavedCancel,
    /// The Simulate button (F5): switch to Simulate and run the current configuration's flight.
    Run,
    /// The Simulate button's menu: switch to Simulate and fly every configuration (one flight each).
    RunAll,
    /// The parts bin.
    PartsBin(parts_bin::Message),
    /// The viewport.
    Viewport(viewport::Message),
    /// Station Lanes.
    Lanes(lanes::Message),
    /// The inspector.
    Inspector(inspector::Message),
    /// The Simulate screen.
    Simulate(simulate::Message),
    /// The Analyze screen.
    Analyze(analyze::Message),
    /// The top bar and status strip: configuration menu, phase, issues, motor picker.
    Chrome(chrome::Message),
    /// The window's close button, Cmd+Q or Alt+F4: exits at once when nothing is unsaved, else asks first.
    CloseRequested,
    /// The export 3D model and export image dialogs.
    Export(export::Message),
    /// The print / PDF dialog.
    Print(print::Message),
}

/// The application state.
pub struct App {
    ctx: Ctx,
    layout: build::Layout,
    /// The ids in the rocket while the drag in progress carries its part, and the rocket as it was when the drag began (see
    /// [`App::track_drop`]).
    drag_start: Option<(Vec<PartId>, bowshock_model::Rocket)>,
    parts_bin: parts_bin::State,
    viewport: viewport::View,
    lanes: lanes::State,
    inspector: inspector::State,
    simulate: simulate::State,
    analyze: analyze::State,
    chrome: chrome::State,
    /// The 3D model and image export dialogs.
    export: export::State,
    /// The print / PDF dialog.
    print: print::State,
}

impl App {
    /// An empty document, Build mode.
    pub fn new() -> Self {
        App {
            ctx: Ctx::new(),
            layout: build::Layout::new(),
            drag_start: None,
            parts_bin: parts_bin::State::new(),
            viewport: viewport::View::new(),
            lanes: lanes::State::new(),
            inspector: inspector::State::new(),
            simulate: simulate::State::new(),
            analyze: analyze::State::new(),
            chrome: chrome::State::new(),
            export: export::State::new(),
            print: print::State::new(),
        }
    }

    /// The app with the built-in example rocket loaded (tests; the binary starts through [`App::launch`]).
    #[cfg(test)]
    pub fn with_example() -> Self {
        App::launch(true)
    }

    /// The app as the binary starts it: empty, or with the built-in example when `example`. `BOWSHOCK_OPEN=<file>` opens that
    /// document (a `.bowshock`, or an OpenRocket `.ork` that is imported) instead of either; a drive script cannot press Ctrl+O,
    /// so this is how one starts from a given rocket (from a rocket whose CFD table is already solved and stored, say).
    pub fn launch(example: bool) -> Self {
        let mut app = App::new();
        if example {
            files::example(&mut app.ctx);
        }
        if let Some(path) = std::env::var_os("BOWSHOCK_OPEN").filter(|p| !p.is_empty()) {
            files::open(&mut app.ctx, std::path::Path::new(&path));
        }
        app.ctx.recent = files::Recent::load(files::Recent::default_file());
        // Unit choices are the user's, kept across documents and runs. Unit tests start from the defaults, never the real file.
        if !cfg!(test) {
            app.ctx.settings_file = settings::Settings::default_file();
            app.ctx.units = settings::Settings::load(app.ctx.settings_file.as_deref()).units;
        }
        app.after_update();
        // A headless `drive` run delivers no key subscription events, so `BOWSHOCK_DRIVE_TAB=1` presses Tab once at start-up.
        if std::env::var_os("BOWSHOCK_DRIVE_TAB").is_some() {
            app.layout.toggle_sides();
        }
        app
    }

    /// The shared context (tests and drive hooks).
    #[cfg(test)]
    pub fn ctx(&self) -> &Ctx {
        &self.ctx
    }

    /// The shared context, mutably (tests).
    #[cfg(test)]
    pub fn ctx_mut(&mut self) -> &mut Ctx {
        &mut self.ctx
    }

    /// The Analyze screen's state (tests).
    #[cfg(test)]
    pub fn analyze(&self) -> &analyze::State {
        &self.analyze
    }

    /// The Analyze screen's state, mutably (tests).
    #[cfg(test)]
    pub fn analyze_mut(&mut self) -> &mut analyze::State {
        &mut self.analyze
    }

    /// Brings derived state up to date after a test changed the context directly.
    #[cfg(test)]
    pub fn sync(&mut self) {
        self.after_update();
    }

    /// Handles `message`, then brings everything derived from the document up to date.
    pub fn update(&mut self, message: Message) -> Task<Message> {
        let before = self.ctx.physics.balance().copied();
        let task = self.route(message);
        self.track_drop();
        self.after_update();
        self.ctx.track_live(before);
        self.ctx.sync_shown();
        // The Analyze screen starts its off-thread computations (and rebuilds its plots) after every message while it shows.
        if self.ctx.mode == Mode::Analyze {
            return Task::batch([task, self.analyze.refresh(&self.ctx)]);
        }
        task
    }

    /// Notices a drag ending in a drop: the part that is in the rocket now but was not while it was carried (a new part from
    /// the bin, or one that was picked up and put down) is the one to flash and settle.
    fn track_drop(&mut self) {
        let ctx = &mut self.ctx;
        if ctx.drag.active() {
            if self.drag_start.is_none() {
                self.drag_start = Some((part_ids(ctx.doc.rocket()), ctx.doc.drag_base().cloned().unwrap_or_else(|| ctx.doc.rocket().clone())));
            }
        } else if let Some((carried, base)) = self.drag_start.take() {
            // A cancelled drag puts the document back exactly as it was: nothing landed, nothing to flash.
            if ctx.doc.rocket() == &base {
                return;
            }
            let new: Vec<PartId> = part_ids(ctx.doc.rocket()).into_iter().filter(|id| !carried.contains(id)).collect();
            let pick = ctx.selection.part().filter(|id| new.contains(id)).or_else(|| new.first().copied());
            if let Some(id) = pick {
                ctx.flash_part(id);
            }
        }
    }

    /// Recomputes what depends on the document (physics, the viewport mesh) and applies requests made during the update.
    fn after_update(&mut self) {
        let ctx = &mut self.ctx;
        ctx.physics.sync(ctx.doc.rocket(), ctx.doc.revision());
        let reloaded = std::mem::take(&mut ctx.reloaded);
        if reloaded {
            ctx.physics.snap_gauge();
        }
        ctx.sanitize_selection();
        self.simulate.sync(&self.ctx, reloaded);
        self.ctx.selected_flight = Some(self.simulate.selected_id());
        self.viewport.sync(&self.ctx);
        if std::mem::take(&mut self.ctx.camera_fit) {
            self.viewport.fit();
        }
    }

    fn route(&mut self, mut message: Message) -> Task<Message> {
        // The "Save changes?" modal takes everything: Escape cancels, Enter saves, and only the clock, the cursor, the layout and
        // the file dialogs' answers get through.
        if self.ctx.unsaved.is_some() {
            use keyboard::key::Named;
            match &message {
                Message::Cancel => message = Message::UnsavedCancel,
                Message::Key(KeyPress { key: keyboard::Key::Named(Named::Enter), modifiers, .. }) if modifiers.is_empty() => message = Message::UnsavedSave,
                Message::Frame(_)
                | Message::Cursor(_)
                | Message::Resized(_)
                | Message::Released
                | Message::Key(_)
                | Message::UnsavedSave
                | Message::UnsavedDiscard
                | Message::UnsavedCancel
                | Message::SavePath(_)
                | Message::OpenPath(_)
                | Message::Chrome(chrome::Message::Nothing) => {}
                _ => return Task::none(),
            }
        }
        // The export and print dialogs are modal too: Escape closes, Enter saves, and only the clock, the cursor, the layout, the
        // viewport's size reports and the dialogs' own answers get through.
        if self.export.is_open() || self.print.is_open() {
            use keyboard::key::Named;
            let printing = self.print.is_open();
            match &message {
                Message::Cancel => message = if printing { Message::Print(print::Message::Close) } else { Message::Export(export::Message::Close) },
                Message::Key(KeyPress { key: keyboard::Key::Named(Named::Enter), modifiers, .. }) if modifiers.is_empty() => {
                    message = if printing { Message::Print(print::Message::Save) } else { Message::Export(export::Message::Save) }
                }
                Message::Frame(_)
                | Message::Cursor(_)
                | Message::Resized(_)
                | Message::Released
                | Message::Key(_)
                | Message::Export(_)
                | Message::Print(_)
                | Message::Viewport(viewport::Message::Fit(_) | viewport::Message::Resized(_))
                | Message::Chrome(chrome::Message::Nothing) => {}
                _ => return Task::none(),
            }
        }
        // While a field is in pick mode a click on a part opens that part's value menu instead of selecting or grabbing it.
        if self.ctx.refs.is_picking() {
            if let Some(hit) = pick_intercept(&message) {
                if let (Some(part), Some(at)) = (hit, self.ctx.cursor) {
                    self.ctx.refs.open_menu(part, at);
                }
                return Task::none();
            }
        }
        let was_open = self.ctx.file_menu;
        // Pointer movement is not a decision: a hover highlight (the parts bin's rows, its keyboard-focus flag) must never close
        // a menu. Only clicks, Esc and choosing an item do.
        let passive = matches!(
            message,
            Message::Frame(_) | Message::Cursor(_) | Message::Released | Message::PartsBin(parts_bin::Message::Hover(_) | parts_bin::Message::RowHover(_))
        );
        if !passive && !matches!(message, Message::OpenPartMenu(_) | Message::PartMenu(_) | Message::Chrome(chrome::Message::Nothing)) {
            self.ctx.part_menu = None;
        }
        if !passive && !matches!(message, Message::ToggleFileMenu | Message::Chrome(chrome::Message::Nothing)) {
            self.ctx.file_menu = false;
        }
        let ctx = &mut self.ctx;
        match message {
            Message::Resized(e) => self.layout.resize(e),
            Message::Mode(mode) => {
                ctx.mode = mode;
                self.chrome.dismiss();
            }
            Message::ToggleTheme => ctx.dark = !ctx.dark,
            Message::Units(units) => {
                crate::units::clear_unit_overrides();
                ctx.units = units;
                let saved = settings::Settings { units }.save(ctx.settings_file.as_deref());
                if let Err(e) = saved {
                    ctx.notify(format!("Could not save settings: {e}"));
                }
            }
            Message::Frame(now) => {
                if let Some((_, since)) = ctx.flash {
                    if crate::viewport::flash::done(crate::viewport::flash::age(since, now)) {
                        ctx.flash = None;
                    }
                }
                self.viewport.tick(now);
                ctx.physics.tick(now);
                self.simulate.tick(now);
                self.analyze.tick(now);
                if ctx.notice.as_ref().is_some_and(|(_, until)| now >= *until) {
                    ctx.notice = None;
                }
            }
            Message::Cursor(at) => {
                ctx.cursor = Some(at);
                return self.parts_bin.update(parts_bin::Message::CursorMoved, ctx);
            }
            Message::Released => {
                let task = self.parts_bin.update(parts_bin::Message::Released, ctx);
                drag::cancel(ctx);
                ctx.doc.seal();
                return task;
            }
            Message::Cancel => {
                ctx.scale = None;
                if !ctx.drag.active() {
                    ctx.clear_multi();
                }
                let task = self.parts_bin.update(parts_bin::Message::Cancel, ctx);
                drag::cancel(ctx);
                self.chrome.dismiss();
                self.viewport.cancel();
                self.simulate.dismiss_picker();
                return task;
            }
            Message::Key(press) => return self.key(press),
            // Parts are only deleted where they are shown and selected for editing: the Build screen.
            Message::Delete if ctx.mode == Mode::Build => drag::delete_selection(ctx),
            Message::Delete => {}
            Message::Undo => {
                let shift = ctx.doc.undo_shift();
                if let Some(label) = ctx.doc.undo() {
                    self.viewport.shift_axis(-shift);
                    ctx.notify(format!("Undid {}", label.to_lowercase()));
                }
            }
            Message::Redo => {
                let shift = ctx.doc.redo_shift();
                if let Some(label) = ctx.doc.redo() {
                    self.viewport.shift_axis(shift);
                    ctx.notify(format!("Redid {}", label.to_lowercase()));
                }
            }
            Message::New => {
                if files::guard(ctx, Pending::New) {
                    files::new(ctx);
                }
            }
            Message::Open => {
                if files::guard(ctx, Pending::Open) {
                    return files::pick_open();
                }
            }
            Message::OpenPath(path) => {
                if let Some(path) = path {
                    files::open(ctx, &path);
                }
            }
            Message::Save => return files::save(ctx),
            Message::SaveAs => return files::pick_save(ctx),
            Message::SavePath(path) => return files::saved_as(ctx, path),
            Message::Example => {
                if files::guard(ctx, Pending::Example) {
                    files::example(ctx);
                }
            }
            Message::ToggleFileMenu => {
                self.chrome.dismiss();
                ctx.file_menu = !was_open;
            }
            Message::ImportOrk => {
                if files::guard(ctx, Pending::Import) {
                    return files::pick_import();
                }
            }
            Message::OpenRecent(path) => {
                if files::guard(ctx, Pending::Recent(path.clone())) {
                    files::open(ctx, &path);
                }
            }
            Message::UnsavedSave => {
                if let Some(what) = ctx.unsaved.take() {
                    return files::save_then(ctx, what);
                }
            }
            Message::UnsavedDiscard => {
                if let Some(what) = ctx.unsaved.take() {
                    return files::proceed(ctx, what);
                }
            }
            Message::UnsavedCancel => ctx.unsaved = None,
            Message::CloseRequested => {
                if files::guard(ctx, Pending::Exit) {
                    return iced::exit();
                }
            }
            Message::OpenPartMenu(part) => {
                if ctx.mode == Mode::Build {
                    ctx.part_menu = ctx.cursor.map(|at| part_menu::Open { part, at });
                }
            }
            Message::Scale(m) => return scale::update(ctx, m),
            Message::PartMenu(action) => {
                let Some(open) = ctx.part_menu.take() else { return Task::none() };
                let tasks: Vec<Task<Message>> = part_menu::messages(ctx, open.part, action).into_iter().map(|m| self.update(m)).collect();
                return Task::batch(tasks);
            }
            Message::Rename => {
                let part_selected = ctx.mode == Mode::Build && matches!(ctx.selection, Selection::Part(_));
                let message = if part_selected { Message::Inspector(inspector::Message::StartRename) } else { Message::Chrome(chrome::Message::BeginRename) };
                return self.update(message);
            }
            Message::Run => {
                ctx.mode = Mode::Simulate;
                self.chrome.dismiss();
                return self.simulate.update(simulate::Message::Run, ctx);
            }
            Message::RunAll => {
                ctx.mode = Mode::Simulate;
                self.chrome.dismiss();
                return self.simulate.update(simulate::Message::PerConfiguration, ctx);
            }
            Message::PartsBin(m) => return self.parts_bin.update(m, ctx),
            Message::Viewport(m) => return self.viewport.update(m, ctx),
            Message::Lanes(m) => return self.lanes.update(m, ctx),
            Message::Inspector(m) => {
                // A committed edit (not every tick of a scrub) flashes the part it changed.
                let (scrub, commit) = (matches!(m, inspector::Message::Edit(_, _, true)), matches!(m, inspector::Message::Edit(_, _, false)));
                let revision = ctx.doc.revision();
                let task = self.inspector.update(m, ctx);
                if commit || (!scrub && ctx.doc.revision() != revision) {
                    if let Some(id) = ctx.selection.part() {
                        ctx.flash_part(id);
                    }
                }
                return task;
            }
            Message::Simulate(m) => return self.simulate.update(m, ctx),
            Message::Analyze(m) => return self.analyze.update(m, ctx),
            Message::Chrome(m) => return self.chrome.update(m, ctx),
            Message::Export(m) => return self.export.update(m, ctx, &mut self.viewport),
            Message::Print(m) => return self.print.update(m, ctx),
        }
        Task::none()
    }

    /// Global shortcuts first (they work whatever has focus), then the active area's own.
    fn key(&mut self, press: KeyPress) -> Task<Message> {
        if let Some(message) = shortcut(&press) {
            return self.route(message);
        }
        if press.captured {
            return Task::none();
        }
        // Tab folds both side columns away for a full-screen viewport (and brings them back).
        if press.key == keyboard::Key::Named(keyboard::key::Named::Tab) && press.modifiers.is_empty() && self.ctx.mode == Mode::Build {
            self.layout.toggle_sides();
            return Task::none();
        }
        let (key, modifiers) = (&press.key, press.modifiers);
        let ctx = &self.ctx;
        let message = match ctx.mode {
            Mode::Build => self
                .parts_bin
                .on_key(key, modifiers, ctx)
                .map(Message::PartsBin)
                .or_else(|| self.viewport.on_key(key, modifiers, ctx).map(Message::Viewport))
                .or_else(|| self.lanes.on_key(key, modifiers, ctx).map(Message::Lanes))
                .or_else(|| self.inspector.on_key(key, modifiers, ctx).map(Message::Inspector)),
            Mode::Simulate => self.simulate.on_key(key, modifiers, ctx).map(Message::Simulate),
            Mode::Analyze => self.analyze.on_key(key, modifiers, ctx).map(Message::Analyze),
        };
        match message {
            Some(message) => self.route(message),
            None => Task::none(),
        }
    }

    /// The theme for the window.
    pub fn theme(&self) -> Theme {
        if self.ctx.dark {
            theme::dark()
        } else {
            theme::light()
        }
    }

    /// Global events (keys, the button release that ends a drag or click wherever it lands) and the frame clock.
    pub fn subscription(&self) -> iced::Subscription<Message> {
        // The window's root widget also sees releases the viewport and bin did not take, but not ones outside the window.
        let events = iced::event::listen_with(|event, status, _| {
            let ignored = status == iced::event::Status::Ignored;
            match event {
                iced::Event::Keyboard(keyboard::Event::KeyPressed { key, modifiers, .. }) => {
                    Some(Message::Key(KeyPress { key, modifiers, captured: !ignored }))
                }
                iced::Event::Mouse(mouse::Event::ButtonReleased(mouse::Button::Left)) if ignored => Some(Message::Released),
                // A right-click on a unit chip changes the units of every field; any message makes the view rebuild.
                iced::Event::Mouse(mouse::Event::ButtonPressed(mouse::Button::Right)) => Some(Message::Chrome(chrome::Message::Nothing)),
                _ => None,
            }
        });
        let animating = self.viewport.animating()
            || self.ctx.animating()
            || self.simulate.animating()
            || self.analyze.animating()
            || self.chrome.animating()
            || self.print.animating();
        let frames = animating.then(|| iced::window::frames().map(Message::Frame));
        let closes = iced::window::close_requests().map(|_| Message::CloseRequested);
        iced::Subscription::batch([events, closes].into_iter().chain(frames))
    }

    /// The whole window: top bar, the screen of the current mode, status strip, and the chrome's overlays.
    pub fn view(&self) -> Element<'_, Message> {
        let ctx = &self.ctx;
        let screen: Element<'_, Message> = match ctx.mode {
            Mode::Build => build::view(&self.layout, &self.parts_bin, &self.viewport, &self.lanes, &self.inspector, ctx),
            Mode::Simulate => self.simulate.view(ctx).map(Message::Simulate).boxed(),
            Mode::Analyze => self.analyze.view(ctx).map(Message::Analyze).boxed(),
        };
        let page = column![chrome::top_bar(ctx), screen, chrome::status_strip(&self.chrome, ctx)];
        // The page is always the first layer of one stack, so its widgets (a field being edited, say) keep their state while
        // a menu or the part menu of a pick comes and goes above it.
        let mut layers: Vec<Element<'_, Message>> = vec![page.boxed()];
        layers.extend(self.chrome.overlay(ctx));
        layers.extend(files::menu(ctx));
        layers.extend(files::unsaved_modal(ctx));
        layers.extend(self.export.view());
        layers.extend(self.print.view());
        layers.extend(refs::menu_view(ctx));
        layers.extend(part_menu::view(ctx));
        layers.extend(scale::view(ctx));
        let root: Element<'_, Message> = iced::widget::Stack::with_children(layers).boxed();
        // The root sees every cursor move and any release the widgets below leave alone: that is what lets a drag begun in the
        // parts bin follow the cursor anywhere, and end (or be cancelled) wherever it is released.
        mouse_area(root).on_move(Message::Cursor).on_release(Message::Released).boxed()
    }
}

/// Every part id in `rocket`: axial parts first, then radial, then internal.
fn part_ids(rocket: &bowshock_model::Rocket) -> Vec<PartId> {
    rocket.parts().iter().map(|p| p.id).chain(rocket.radials().iter().map(|p| p.id)).chain(rocket.internals().iter().map(|p| p.id)).collect()
}

/// What a click means while a field is in pick mode (the viewport and the lanes are pickable): `Some(Some(part))` opens the part's
/// value menu, `Some(None)` is swallowed (a click on nothing, a grab, a drag of a bar), `None` is not a pick at all.
fn pick_intercept(message: &Message) -> Option<Option<PartId>> {
    use crate::build::lanes::Message as Lanes;
    use crate::viewport::Message as Viewport;
    match message {
        Message::Viewport(Viewport::Select(Some(pick)) | Viewport::Toggle(pick)) => Some(Some(pick.id())),
        Message::Viewport(Viewport::SelectPart(Some(id))) => Some(Some(*id)),
        Message::Viewport(Viewport::Select(None) | Viewport::SelectPart(None) | Viewport::SelectSeparation(_) | Viewport::Grab(_) | Viewport::Focus(_)) => {
            Some(None)
        }
        Message::Lanes(Lanes::Select(Some(key)) | Lanes::Toggle(key)) => Some(Some(key.part())),
        Message::Viewport(Viewport::PartMenu(_)) | Message::Lanes(Lanes::PartMenu(_)) => Some(None),
        Message::Lanes(
            Lanes::Select(None) | Lanes::SelectSeparation(_) | Lanes::Focus(_) | Lanes::GrabAxial(_) | Lanes::MoveTo { .. } | Lanes::ResizeTo { .. },
        ) => Some(None),
        _ => None,
    }
}

/// The global keyboard map: shortcuts that work in every mode, whatever has focus.
fn shortcut(press: &KeyPress) -> Option<Message> {
    use keyboard::key::Named;
    let mods = press.modifiers;
    match &press.key {
        keyboard::Key::Named(Named::Escape) => Some(Message::Cancel),
        keyboard::Key::Named(Named::F5) => Some(Message::Run),
        keyboard::Key::Named(Named::F2) => Some(Message::Rename),
        keyboard::Key::Named(Named::Delete | Named::Backspace) if !press.captured && !mods.command() => Some(Message::Delete),
        keyboard::Key::Character(c) if mods.command() => {
            let c = c.as_str().to_lowercase();
            match (c.as_str(), mods.shift()) {
                ("z", false) => Some(Message::Undo),
                ("z", true) | ("y", false) => Some(Message::Redo),
                ("n", false) => Some(Message::New),
                ("o", false) => Some(Message::Open),
                ("s", false) => Some(Message::Save),
                ("s", true) => Some(Message::SaveAs),
                ("e", false) => Some(Message::Example),
                ("l", false) => Some(Message::Chrome(chrome::Message::NewConfiguration)),
                ("p", false) => Some(Message::Print(print::Message::Open)),
                _ => None,
            }
        }
        _ => None,
    }
}

#[cfg(test)]
#[allow(unused_must_use)] // The tests drive `update` directly and have no runtime to hand its tasks to.
mod tests {
    use super::*;
    use crate::viewport::manipulate::Edit;
    use crate::viewport::pick::{Pick, SurfaceTarget};
    use crate::viewport::Message as Vp;
    use bowshock_model::{Dim, NoseShape, PartKind, RadialKind};

    /// An app holding a nose, a tube and a transition (fore to aft).
    fn app() -> (App, [PartId; 3]) {
        let mut app = App::new();
        let ctx = &mut app.ctx;
        drag::place(ctx, 0, PartKind::NoseCone { shape: NoseShape::TangentOgive, length: 0.2, diameter: 0.1 });
        drag::place(ctx, 1, PartKind::BodyTube { length: 0.5, diameter: 0.1 });
        drag::place(ctx, 2, PartKind::Transition { length: 0.1, fore_diameter: 0.1, aft_diameter: 0.06 });
        app.after_update();
        let ids = [0, 1, 2].map(|i| app.ctx.doc.rocket().parts()[i].id);
        (app, ids)
    }

    fn vp(app: &mut App, message: Vp) {
        app.update(Message::Viewport(message));
    }

    /// The app with the viewport fitted to 900 × 560 and its animation landed.
    fn fitted() -> (App, [PartId; 3]) {
        let (mut app, ids) = app();
        land(&mut app, Vp::Fit(iced::Size::new(900.0, 560.0)));
        (app, ids)
    }

    /// Sends `message` to the viewport and runs any camera animation it started to its end.
    fn land(app: &mut App, message: Vp) {
        vp(app, message);
        app.viewport.tick(std::time::Instant::now() + std::time::Duration::from_secs(5));
    }

    /// Lengthens the nose cone from 0.2 m to 1.5 m, which pushes the rocket's far end out of a view fitted to the short one
    /// (stations run from the tip, so a longer nose moves everything behind it aft).
    fn lengthen_nose(app: &mut App, nose: PartId) {
        app.ctx.doc.edit("Lengthen nose", |r| r.set_dim(nose, Dim::Length, 1.5));
        app.after_update();
    }

    #[test]
    fn an_edit_that_pushes_the_rocket_out_of_an_untouched_view_refits_it() {
        let (mut app, [nose, ..]) = fitted();
        assert!(!app.viewport.out_of_view());
        let was = app.viewport.camera().scale;
        lengthen_nose(&mut app, nose);
        // The refit is the usual animated one: it lands with the whole rocket in the view, zoomed out from before.
        app.viewport.tick(std::time::Instant::now() + std::time::Duration::from_secs(5));
        assert!(!app.viewport.out_of_view(), "the longer rocket is framed");
        assert!(app.viewport.camera().scale < was, "zoomed out to take it in");
        assert!(app.viewport.hints(&app.ctx).is_empty(), "nothing to hint: it was refitted for the user");
    }

    #[test]
    fn after_the_user_zoomed_an_edit_leaves_the_view_alone_offers_fit_and_zooming_out_still_works() {
        let (mut app, [nose, ..]) = fitted();
        // Zoom out a little by hand (all of the rocket still shows), then grow the rocket beyond the edge.
        land(&mut app, Vp::Zoom { factor: 0.9, offset: iced::Vector::new(-100.0, 0.0) });
        assert!(!app.viewport.out_of_view());
        let hand = app.viewport.camera();
        lengthen_nose(&mut app, nose);
        app.viewport.tick(std::time::Instant::now() + std::time::Duration::from_secs(5));
        assert_eq!(app.viewport.camera(), hand, "the view is the user's, not refitted under them");
        assert!(app.viewport.out_of_view());
        assert!(app.viewport.hints(&app.ctx).contains(&("F", "fit")), "the hint bar offers fit");

        // Zoom out about the nose end, as the wheel does: it is not limited by the old size of the rocket.
        let mut notches = 0;
        while app.viewport.out_of_view() {
            let before = app.viewport.camera().scale;
            land(&mut app, Vp::Zoom { factor: 1.0 / 1.2, offset: iced::Vector::new(-300.0, 0.0) });
            assert!(app.viewport.camera().scale < before, "each notch zooms out");
            notches += 1;
            assert!(notches < 30, "never came into view");
        }
        assert!(app.viewport.hints(&app.ctx).is_empty(), "the hint goes once the whole rocket is in view");

        // Fit (the `F` key) frames it again whatever was done.
        land(&mut app, Vp::Fit(iced::Size::new(900.0, 560.0)));
        assert!(!app.viewport.out_of_view());
    }

    #[test]
    fn a_pinch_over_the_viewport_zooms_about_the_cursor_and_two_finger_scroll_does_not() {
        use crate::widgets::pinch::Phase as GesturePhase;
        let (app, _) = app();
        let mut sim = iced_test::Simulator::with_size(iced::Settings::default(), iced::Size::new(1280.0, 800.0), app.view());
        sim.point_at(Point::new(640.0, 400.0));
        // The viewport's first event only reports its size.
        let _ = sim.simulate([iced::Event::Mouse(mouse::Event::CursorMoved { position: Point::new(640.0, 400.0) })]);
        let _ = sim.drain().count();
        let pinch = |delta| iced::Event::Mouse(mouse::Event::PinchGesture { delta, phase: GesturePhase::Moved });
        let _ = sim.simulate([pinch(0.25), pinch(-0.25)]);
        let zooms: Vec<f32> = sim
            .drain()
            .filter_map(|m| match m {
                Message::Viewport(Vp::Zoom { factor, .. }) => Some(factor),
                _ => None,
            })
            .collect();
        assert_eq!(zooms.len(), 2, "one zoom per pinch event: {zooms:?}");
        assert!(zooms[0] > 1.0 && zooms[1] < 1.0, "spreading zooms in, closing zooms out: {zooms:?}");
        assert!((zooms[0] * zooms[1] - 1.0).abs() < 1e-6);

        // Two-finger scroll (pixel deltas, no modifier) still orbits.
        let _ = sim.scroll(mouse::ScrollDelta::Pixels { x: 6.0, y: 3.0 });
        let moves: Vec<Message> = sim.drain().collect();
        assert!(moves.iter().any(|m| matches!(m, Message::Viewport(Vp::Orbit(_)))), "{moves:?}");
        assert!(!moves.iter().any(|m| matches!(m, Message::Viewport(Vp::Zoom { .. }))));
    }

    fn parts(app: &App) -> Vec<bowshock_model::Part> {
        app.ctx.doc.rocket().parts().to_vec()
    }

    #[test]
    fn select_and_delete() {
        let (mut app, [_, tube, _]) = app();
        vp(&mut app, Vp::Select(Some(Pick::Axial(tube))));
        assert_eq!(app.ctx.selection, Selection::Part(tube));
        app.update(Message::Delete);
        assert!(app.ctx.doc.rocket().part(tube).is_none() && app.ctx.selection == Selection::None);
        assert_eq!(app.ctx.doc.rocket().parts().len(), 2);
        // Clicking empty space deselects.
        vp(&mut app, Vp::Select(None));
        app.update(Message::Delete);
        assert_eq!(app.ctx.doc.rocket().parts().len(), 2);
        // The delete is one undo step, and the selection is dropped when undo removes what it pointed at.
        app.update(Message::Undo);
        assert_eq!(app.ctx.doc.rocket().parts().len(), 3);
    }

    #[test]
    fn grabbing_a_part_moves_it_and_cancelling_puts_it_back() {
        let (mut app, [nose, tube, transition]) = app();
        let before = parts(&app);
        vp(&mut app, Vp::Grab(Pick::Axial(tube)));
        assert!(app.ctx.doc.rocket().part(tube).is_none() && app.ctx.drag.axial.is_some() && app.ctx.selection == Selection::Part(tube));
        app.update(Message::Cancel);
        assert_eq!(parts(&app), before);

        // No snap target on release: back where it was.
        vp(&mut app, Vp::Grab(Pick::Axial(transition)));
        vp(&mut app, Vp::DropAxial);
        assert_eq!(parts(&app), before);

        // Dropped on a node: moved, keeping its id, and matched to its new neighbours.
        vp(&mut app, Vp::Grab(Pick::Axial(transition)));
        vp(&mut app, Vp::SnapNode(Some(1)));
        assert!(app.ctx.drag.axial.as_ref().is_some_and(|d| d.snap == Some(1)));
        vp(&mut app, Vp::DropAxial);
        let rocket = app.ctx.doc.rocket();
        assert_eq!(rocket.index_of(transition), Some(1));
        assert_eq!(rocket.index_of(nose), Some(0));
        assert_eq!(rocket.part(transition).unwrap().kind.fore_diameter(), 0.1);
        assert_eq!(rocket.part(transition).unwrap().kind.aft_diameter(), 0.1);
        assert!(app.ctx.drag.axial.is_none());
    }

    #[test]
    fn a_drop_flashes_the_part_once_and_a_cancel_does_not() {
        let (mut app, [_, tube, transition]) = app();
        assert!(app.ctx.flash.is_none());
        vp(&mut app, Vp::Grab(Pick::Axial(tube)));
        app.update(Message::Cancel);
        assert!(app.ctx.flash.is_none());
        vp(&mut app, Vp::Grab(Pick::Axial(transition)));
        assert!(app.ctx.flash.is_none(), "nothing flashes while the part is carried");
        vp(&mut app, Vp::SnapNode(Some(1)));
        vp(&mut app, Vp::DropAxial);
        let (id, since) = app.ctx.flash.expect("the dropped part flashes");
        assert_eq!(id, transition);
        // The flash outlasts the 120 ms settle, runs out after 400 ms, and is cleared by the frame tick that sees it end.
        let levels = app.ctx.flash_levels().expect("flash running");
        assert_eq!(levels.0, transition);
        app.update(Message::Frame(since + crate::viewport::flash::SETTLE_TIME));
        assert!(app.ctx.flash.is_some());
        app.update(Message::Frame(since + crate::viewport::flash::FLASH_TIME));
        assert!(app.ctx.flash.is_none());
    }

    #[test]
    fn only_a_committed_inspector_edit_flashes_the_part() {
        use crate::build::inspector::{Field, Message as Insp};
        let (mut app, [_, tube, _]) = app();
        vp(&mut app, Vp::Select(Some(Pick::Axial(tube))));
        let field = Field::Dim(bowshock_model::Dim::Length);
        app.update(Message::Inspector(Insp::Edit(field, 0.4, true)));
        assert!(app.ctx.flash.is_none(), "a scrub tick does not flash");
        app.update(Message::Inspector(Insp::Edit(field, 0.42, false)));
        assert_eq!(app.ctx.flash.map(|(id, _)| id), Some(tube));
    }

    #[test]
    fn tab_folds_the_side_columns_in_build_but_not_while_a_field_captures_it() {
        let (mut app, _) = app();
        let tab = |captured| KeyPress { key: keyboard::Key::Named(keyboard::key::Named::Tab), modifiers: keyboard::Modifiers::empty(), captured };
        app.update(Message::Key(tab(true)));
        assert!(!app.layout.sides_hidden());
        app.update(Message::Key(tab(false)));
        assert!(app.layout.sides_hidden());
        app.update(Message::Key(tab(false)));
        assert!(!app.layout.sides_hidden());
    }

    #[test]
    fn moving_a_part_keeps_its_material_and_undo_is_one_step() {
        let (mut app, [_, tube, transition]) = app();
        app.ctx.doc.edit("Name", |r| r.set_name(tube, Some("Payload".to_string())));
        app.ctx.doc.edit("Wall", |r| r.set_thickness(tube, 0.004));
        let before = parts(&app);
        vp(&mut app, Vp::Grab(Pick::Axial(tube)));
        vp(&mut app, Vp::SnapNode(Some(2)));
        vp(&mut app, Vp::DropAxial);
        let rocket = app.ctx.doc.rocket();
        assert_eq!(rocket.index_of(tube), Some(2));
        let moved = rocket.part(tube).unwrap();
        assert_eq!((moved.name.as_deref(), moved.thickness), (Some("Payload"), 0.004));
        assert_eq!(rocket.index_of(transition), Some(1));
        // The whole move is one undo step.
        app.update(Message::Undo);
        assert_eq!(parts(&app), before);
    }

    #[test]
    fn resizing_changes_only_that_part_and_a_resize_drag_is_one_undo_step() {
        let (mut app, [_, tube, transition]) = app();
        let before = parts(&app);
        vp(&mut app, Vp::Edit(Edit::Begin(tube, "Drag face")));
        for length in [0.55, 0.6, 0.7] {
            vp(&mut app, Vp::Edit(Edit::Dim(tube, Dim::Length, length)));
        }
        vp(&mut app, Vp::Edit(Edit::Dim(tube, Dim::Diameter, 0.12)));
        vp(&mut app, Vp::Edit(Edit::End));
        let kind = PartKind::BodyTube { length: 0.7, diameter: 0.12 };
        assert_eq!(app.ctx.doc.rocket().part(tube).unwrap().kind, kind);
        // The transition behind it follows the tube's new diameter at their joint.
        assert_eq!(app.ctx.doc.rocket().part(transition).unwrap().kind.fore_diameter(), 0.12);
        // Cancelled, a drag puts everything back.
        vp(&mut app, Vp::Edit(Edit::Begin(tube, "Drag face")));
        vp(&mut app, Vp::Edit(Edit::Dim(tube, Dim::Length, 0.3)));
        vp(&mut app, Vp::Edit(Edit::Cancel));
        assert_eq!(app.ctx.doc.rocket().part(tube).unwrap().kind, kind);
        app.update(Message::Undo);
        assert_eq!(parts(&app), before);
    }

    #[test]
    fn undo_and_redo_of_a_fore_end_drag_keep_the_aft_end_still_on_screen() {
        let (mut app, [_, tube, _]) = app();
        let start = app.viewport.camera().target.z;
        let before = app.ctx.doc.rocket().dim(tube, Dim::Length).unwrap();
        vp(&mut app, Vp::Edit(Edit::Begin(tube, "Drag fore face")));
        for length in [before + 0.05, before + 0.2] {
            vp(&mut app, Vp::Edit(Edit::Dim(tube, Dim::ForeLength, length)));
        }
        vp(&mut app, Vp::Edit(Edit::End));
        let grown = app.viewport.camera().target.z;
        let delta = (app.ctx.doc.rocket().dim(tube, Dim::Length).unwrap() - before) as f32;
        assert!(delta > 0.1 && (grown - start - delta).abs() < 1e-5, "the camera follows the aft end");
        app.update(Message::Undo);
        assert!((app.viewport.camera().target.z - start).abs() < 1e-5, "undo moves the camera back");
        app.update(Message::Redo);
        assert!((app.viewport.camera().target.z - grown).abs() < 1e-5, "redo moves it forward again");
    }

    fn target(parent: PartId, offset: f64, angle: f64) -> SurfaceTarget {
        SurfaceTarget { parent, offset, angle, copy: 0 }
    }

    /// Hovering the parts bin (rows, focus flag) while the File menu is open must not close it; a click elsewhere still does.
    #[test]
    fn hovering_the_parts_bin_keeps_the_file_menu_open() {
        let (mut app, _) = app();
        app.update(Message::ToggleFileMenu);
        assert!(app.ctx.file_menu);
        let row = parts_bin::RowId::new(parts_bin::Category::Fins, 0);
        app.update(Message::PartsBin(parts_bin::Message::Hover(true)));
        app.update(Message::PartsBin(parts_bin::Message::RowHover(Some(row))));
        app.update(Message::PartsBin(parts_bin::Message::RowHover(None)));
        app.update(Message::PartsBin(parts_bin::Message::Hover(false)));
        assert!(app.ctx.file_menu, "hover must not dismiss the menu");
        app.update(Message::PartsBin(parts_bin::Message::Category(parts_bin::Category::Fins)));
        assert!(!app.ctx.file_menu, "a real action closes it");
    }

    #[test]
    fn a_click_attaches_radial_parts_to_the_aftmost_tube_with_the_symmetry_count() {
        let (mut app, [_, tube, _]) = app();
        app.update(Message::PartsBin(parts_bin::Message::Category(parts_bin::Category::Fins)));
        app.update(Message::PartsBin(parts_bin::Message::Symmetry(4)));
        app.update(Message::PartsBin(parts_bin::Message::RowPress(parts_bin::RowId::new(parts_bin::Category::Fins, 0))));
        app.update(Message::Released);
        let [fins] = app.ctx.doc.rocket().radials() else { panic!("one radial part") };
        assert_eq!((fins.parent, fins.count, fins.angle), (tube, 4, 0.0));
        // Flush with the aft end of the tube (0.5 long, 0.12 root chord).
        assert!((fins.offset - (0.5 - 0.12)).abs() < 1e-9);
        // With nothing to attach to, a click does nothing.
        let mut empty = App::new();
        empty.update(Message::PartsBin(parts_bin::Message::Category(parts_bin::Category::Fins)));
        empty.update(Message::PartsBin(parts_bin::Message::RowPress(parts_bin::RowId::new(parts_bin::Category::Fins, 1))));
        empty.update(Message::Released);
        assert!(empty.ctx.doc.rocket().radials().is_empty());
    }

    #[test]
    fn dragging_a_radial_part_onto_the_surface_creates_a_group_with_the_live_count() {
        let (mut app, [_, tube, _]) = app();
        app.update(Message::PartsBin(parts_bin::Message::Category(parts_bin::Category::Fins)));
        app.update(Message::PartsBin(parts_bin::Message::Symmetry(3)));
        // Press the lug row and move beyond the slop: the drag starts with the footer's count.
        app.update(Message::Cursor(Point::new(100.0, 100.0)));
        app.update(Message::PartsBin(parts_bin::Message::RowPress(parts_bin::RowId::new(parts_bin::Category::Fins, 3))));
        app.update(Message::Cursor(Point::new(160.0, 100.0)));
        assert!(app.ctx.drag.radial.as_ref().is_some_and(|d| d.count == 3 && d.row == Some(3)));
        // X cycles the count (and the footer's, for the next one); Shift+X goes back.
        app.update(Message::PartsBin(parts_bin::Message::CycleSymmetry(true)));
        assert_eq!((app.ctx.drag.radial.as_ref().unwrap().count, app.ctx.drag.symmetry), (4, 4));
        app.update(Message::PartsBin(parts_bin::Message::CycleSymmetry(false)));
        app.update(Message::PartsBin(parts_bin::Message::CycleSymmetry(false)));
        assert_eq!(app.ctx.drag.radial.as_ref().unwrap().count, 2);
        vp(&mut app, Vp::Surface(Some(target(tube, 0.1, 1.0)), false));
        assert!(app.ctx.drag.radial.as_ref().is_some_and(|d| d.target == Some(target(tube, 0.1, 1.0))));
        vp(&mut app, Vp::DropRadial);
        let [lug] = app.ctx.doc.rocket().radials() else { panic!("one radial part") };
        assert_eq!((lug.parent, lug.count, lug.offset, lug.angle), (tube, 2, 0.1, 1.0));
        assert!(app.ctx.drag.radial.is_none());

        // No target: cancelled. Escape: cancelled.
        let start = |app: &mut App| {
            app.update(Message::PartsBin(parts_bin::Message::RowPress(parts_bin::RowId::new(parts_bin::Category::Fins, 4))));
            app.update(Message::Cursor(Point::new(220.0, 100.0)));
        };
        start(&mut app);
        vp(&mut app, Vp::DropRadial);
        start(&mut app);
        vp(&mut app, Vp::Surface(Some(target(tube, 0.1, 1.0)), false));
        app.update(Message::Cancel);
        assert_eq!(app.ctx.doc.rocket().radials().len(), 1);
    }

    #[test]
    fn a_radial_group_can_be_picked_up_moved_and_put_back_and_deleted() {
        let (mut app, [_, tube, transition]) = app();
        let id = app.ctx.doc.edit("Lug", |r| r.attach_radial(tube, 0.0, 0.0, 3, RadialKind::LaunchLug { length: 0.05, diameter: 0.01 }).unwrap());
        let before = app.ctx.doc.rocket().radials().to_vec();
        vp(&mut app, Vp::Grab(Pick::Radial { id, instance: 1 }));
        assert!(app.ctx.doc.rocket().radials().is_empty() && app.ctx.selection == Selection::Part(id));
        // Cancelled, or dropped with no target: back as it was.
        app.update(Message::Cancel);
        assert_eq!(app.ctx.doc.rocket().radials(), before.as_slice());
        vp(&mut app, Vp::Grab(Pick::Radial { id, instance: 1 }));
        vp(&mut app, Vp::DropRadial);
        assert_eq!(app.ctx.doc.rocket().radials(), before.as_slice());
        // Dropped on another part: moved there, same id, same count.
        vp(&mut app, Vp::Grab(Pick::Radial { id, instance: 1 }));
        vp(&mut app, Vp::Surface(Some(target(transition, 0.0, 2.0)), false));
        vp(&mut app, Vp::DropRadial);
        let moved = app.ctx.doc.rocket().radial(id).unwrap();
        assert_eq!((moved.parent, moved.count, moved.angle), (transition, 3, 2.0));
        // Deleting the selected group removes all of it; its ids are not reused.
        app.update(Message::Delete);
        assert!(app.ctx.doc.rocket().radials().is_empty() && app.ctx.selection == Selection::None);
        assert_eq!(app.ctx.doc.rocket().parts().len(), 3);
    }

    #[test]
    fn an_ordinary_tube_dragged_onto_a_tubes_side_becomes_a_pod_and_a_node_still_stacks() {
        let (mut app, [_, tube, _]) = app();
        let before = app.ctx.doc.rocket().clone();
        let row = parts_bin::axial_catalogue().iter().position(|(n, _)| *n == "Body tube").unwrap();
        let start = |app: &mut App| {
            app.update(Message::PartsBin(parts_bin::Message::Category(parts_bin::Category::Airframe)));
            app.update(Message::Cursor(Point::new(100.0, 100.0)));
            app.update(Message::PartsBin(parts_bin::Message::RowPress(parts_bin::RowId::new(parts_bin::Category::Airframe, row))));
            app.update(Message::Cursor(Point::new(160.0, 100.0)));
        };
        // On a tube's side: a pod set of the bin's symmetry count whose body is exactly the part dropped.
        app.update(Message::PartsBin(parts_bin::Message::Symmetry(3)));
        start(&mut app);
        assert!(app.ctx.drag.axial.as_ref().is_some_and(|d| d.count == 3 && d.surface.is_none()));
        vp(&mut app, Vp::AxialSurface(Some(target(tube, 0.1, 1.0))));
        assert!(app.ctx.drag.axial.as_ref().is_some_and(|d| d.surface.is_some() && d.snap.is_none()));
        // X cycles the copies while it hovers there.
        app.update(Message::PartsBin(parts_bin::Message::CycleSymmetry(false)));
        assert_eq!(app.ctx.drag.axial.as_ref().unwrap().count, 2);
        vp(&mut app, Vp::DropAxial);
        let pod = app.ctx.doc.rocket().pods().next().expect("a pod set").clone();
        let body = pod.kind.pod_body().unwrap();
        assert_eq!((pod.count, pod.parent, pod.offset, pod.angle), (2, tube, 0.1, 1.0));
        assert_eq!(body.parts().len(), 1);
        assert!(matches!(body.parts()[0].kind, bowshock_model::PartKind::BodyTube { .. }));
        assert_eq!(app.ctx.doc.rocket().parts().len(), 3, "the core is unchanged");
        assert_eq!(app.ctx.selection, Selection::Part(body.parts()[0].id));
        // Undo is one step.
        app.update(Message::Undo);
        assert_eq!(app.ctx.doc.rocket().ids(), before.ids());
        // A node still stacks, and moving to a node leaves the surface mode.
        start(&mut app);
        vp(&mut app, Vp::AxialSurface(Some(target(tube, 0.1, 1.0))));
        vp(&mut app, Vp::SnapNode(Some(1)));
        assert!(app.ctx.drag.axial.as_ref().is_some_and(|d| d.surface.is_none() && d.snap == Some(1)));
        vp(&mut app, Vp::DropAxial);
        assert_eq!((app.ctx.doc.rocket().parts().len(), app.ctx.doc.rocket().pods().count()), (4, 0));
    }

    #[test]
    fn moving_an_existing_tube_onto_a_tubes_side_makes_a_pod_and_back_onto_a_node_stacks_it() {
        let (mut app, [_, tube, transition]) = app();
        let fins = app.ctx.doc.edit("Fins", |r| {
            r.attach_radial(transition, 0.0, 0.0, 3, RadialKind::FinSet { root_chord: 0.03, tip_chord: 0.01, span: 0.02, sweep: 0.01, thickness: 0.002 })
                .unwrap()
        });
        let before = app.ctx.doc.rocket().clone();
        // Esc cancels exactly.
        vp(&mut app, Vp::Grab(Pick::Axial(transition)));
        vp(&mut app, Vp::AxialSurface(Some(target(tube, 0.1, 2.0))));
        app.update(Message::Cancel);
        assert_eq!(app.ctx.doc.rocket(), &before);
        // Dropped on a tube's side: a pod of that one part (ids and the fins on it kept), as one undo step.
        vp(&mut app, Vp::Grab(Pick::Axial(transition)));
        vp(&mut app, Vp::AxialSurface(Some(target(tube, 0.1, 2.0))));
        vp(&mut app, Vp::DropAxial);
        let rocket = app.ctx.doc.rocket();
        let pod = rocket.pod_of(transition).expect("the transition is a pod's part now");
        assert_eq!((rocket.parts().len(), pod.kind.pod_body().unwrap().parts().len(), pod.angle), (2, 1, 2.0));
        assert_eq!(rocket.parent_of(fins), Some(transition));
        app.update(Message::Undo);
        assert_eq!(app.ctx.doc.rocket(), &before);
        app.update(Message::Redo);
        // Back onto a node: the part stacks into the core again and the pod is gone.
        vp(&mut app, Vp::Grab(Pick::Axial(transition)));
        assert!(app.ctx.drag.axial.as_ref().is_some_and(|d| d.origin.is_some()) && app.ctx.doc.rocket().pods().next().is_none());
        vp(&mut app, Vp::SnapNode(Some(2)));
        vp(&mut app, Vp::DropAxial);
        let rocket = app.ctx.doc.rocket();
        assert_eq!((rocket.parts().len(), rocket.pods().count(), rocket.index_of(transition)), (3, 0, Some(2)));
        assert_eq!(rocket.parent_of(fins), Some(transition));
    }

    #[test]
    fn a_click_add_lands_on_the_selected_pod_tube_and_a_pod_mount_takes_a_motor() {
        let (mut app, [_, tube, _]) = app();
        let kind = app.ctx.doc.rocket().default_pod();
        let pod = app.ctx.doc.edit("Pod", |r| r.attach_radial(tube, 0.0, 0.0, 2, kind).unwrap());
        let pod_tube = app.ctx.doc.rocket().radial(pod).unwrap().kind.pod_body().unwrap().parts()[1].id;
        app.ctx.selection = Selection::Part(pod_tube);
        app.update(Message::PartsBin(parts_bin::Message::Category(parts_bin::Category::Fins)));
        app.update(Message::PartsBin(parts_bin::Message::RowPress(parts_bin::RowId::new(parts_bin::Category::Fins, 0))));
        app.update(Message::Released);
        let fins = app.ctx.doc.rocket().pod_of(pod_tube).and_then(|p| p.kind.pod_body()).unwrap().radials()[0].id;
        assert_eq!(app.ctx.doc.rocket().parent_of(fins), Some(pod_tube));
        // A motor mount in the pod is a mount of the rocket (one motor per copy) and takes a motor from the bin.
        let mount = app.ctx.doc.edit("Mount", |r| {
            r.attach_internal(
                pod_tube,
                0.05,
                bowshock_model::InternalKind::InnerTube { outer_diameter: 0.0229, thickness: 0.0005, length: 0.12, motor_mount: true, overhang: 0.0 },
            )
            .unwrap()
        });
        let m = app.ctx.doc.rocket().motor_mounts().into_iter().find(|m| m.id == mount).expect("the pod's mount is a mount of the rocket");
        assert_eq!((m.pod, m.copies), (Some(pod), 2));
        let config = app.ctx.doc.rocket().active_config_id();
        app.ctx.doc.edit("Motor", |r| {
            r.set_motor(config, mount, Some(bowshock_model::MotorRef::new("x", "y"))).unwrap();
        });
        assert!(app.ctx.doc.rocket().active_configuration().slot(mount).is_some_and(|s| s.motor.is_some()));
        // Pod mounts are not stages.
        assert!(app.ctx.doc.rocket().stages().iter().all(|s| !s.mounts.contains(&mount)));
    }

    /// A pod of one bare tube (Ø30 × 300 mm), three copies, on core part `tube`. Returns the pod set and its tube.
    fn bare_pod(app: &mut App, tube: PartId) -> (PartId, PartId) {
        let body = bowshock_model::PodBody::single(bowshock_model::Part::new(0, PartKind::BodyTube { length: 0.3, diameter: 0.03 }), vec![], vec![]);
        let pod = app.ctx.doc.edit("Pod", |r| r.attach_radial(tube, 0.1, 0.0, 3, RadialKind::Pod { gap: 0.0, body }).unwrap());
        let part = app.ctx.doc.rocket().radial(pod).unwrap().kind.pod_body().unwrap().parts()[0].id;
        (pod, part)
    }

    fn pod_kinds(app: &App, pod: PartId) -> Vec<PartKind> {
        app.ctx.doc.rocket().radial(pod).unwrap().kind.pod_body().unwrap().parts().iter().map(|p| p.kind.clone()).collect()
    }

    /// Why each of the bin's nose rows (every nose variant) is unavailable, `None` for an available one.
    fn nose_reasons(app: &App) -> Vec<Option<String>> {
        (0..5).map(|i| app.parts_bin.row_unavailable(parts_bin::RowId::new(parts_bin::Category::Airframe, i), &app.ctx)).collect()
    }

    #[test]
    fn the_nose_rows_stay_enabled_while_any_body_has_a_free_fore_end() {
        let (mut app, [_, tube, _]) = app();
        // The core has its nose and there is no other body: every nose row says why it is dim.
        assert!(nose_reasons(&app).iter().all(|r| r.as_deref().is_some_and(|r| r.starts_with("Every body has a nose"))), "{:?}", nose_reasons(&app));
        // A bare tube on the outside has a free fore end: every nose variant is available again.
        let (pod, _) = bare_pod(&mut app, tube);
        assert!(nose_reasons(&app).iter().all(Option::is_none), "{:?}", nose_reasons(&app));
        // Once that pod has a nose too, no body has a free fore end.
        assert!(app.ctx.doc.edit("Nose", |r| r.insert_in_pod(pod, 0, PartKind::NoseCone { shape: NoseShape::Conical, length: 0.05, diameter: 0.03 }).is_ok()));
        assert!(nose_reasons(&app).iter().all(Option::is_some));
        // The tail cone goes to the aft node of any body, the pod's included.
        let boattail = parts_bin::RowId::new(parts_bin::Category::Airframe, 7);
        assert!(app.parts_bin.row_unavailable(boattail, &app.ctx).is_none());
    }

    #[test]
    fn a_nose_dragged_onto_a_pod_tubes_fore_node_goes_on_it_and_the_core_keeps_its_own() {
        let (mut app, [_, tube, _]) = app();
        let (pod, pod_tube) = bare_pod(&mut app, tube);
        let before = app.ctx.doc.rocket().clone();
        app.update(Message::Cursor(Point::new(100.0, 100.0)));
        app.update(Message::PartsBin(parts_bin::Message::RowPress(parts_bin::RowId::new(parts_bin::Category::Airframe, 0))));
        app.update(Message::Cursor(Point::new(160.0, 100.0)));
        assert!(app.ctx.drag.axial.as_ref().is_some_and(|d| matches!(d.kind, PartKind::NoseCone { shape: NoseShape::TangentOgive, .. })), "the nose row drags");
        vp(&mut app, Vp::SnapPodNode(Some((pod, 0))));
        vp(&mut app, Vp::DropAxial);
        let kinds = pod_kinds(&app, pod);
        assert_eq!(kinds.len(), 2);
        // It is the pod's first part, sized to the tube behind it (Ø30), and the core is unchanged.
        assert!(matches!(kinds[0], PartKind::NoseCone { shape: NoseShape::TangentOgive, diameter, .. } if (diameter - 0.03).abs() < 1e-9), "{kinds:?}");
        assert_eq!(app.ctx.doc.rocket().parts(), before.parts());
        assert_eq!(app.ctx.doc.rocket().pod_part_index(pod_tube), Some((pod, 1)));
        // One undo step takes it off again.
        app.update(Message::Undo);
        assert_eq!(app.ctx.doc.rocket(), &before);
    }

    #[test]
    fn a_clicked_nose_goes_on_the_selected_pod_and_else_on_the_first_pod_with_a_free_fore_end() {
        let (mut app, [_, tube, _]) = app();
        let (pod, pod_tube) = bare_pod(&mut app, tube);
        // (Different rows, so the second click is not taken for a double click on the first.)
        let click = |app: &mut App, row: usize| {
            app.update(Message::PartsBin(parts_bin::Message::RowPress(parts_bin::RowId::new(parts_bin::Category::Airframe, row))));
            app.update(Message::Released);
        };
        // A pod's tube selected: the nose goes on that pod.
        app.ctx.selection = Selection::Part(pod_tube);
        click(&mut app, 0);
        assert!(matches!(pod_kinds(&app, pod)[0], PartKind::NoseCone { .. }));
        assert_eq!(app.ctx.doc.rocket().parts().len(), 3, "the core is unchanged");
        // A second pod, with the core's tube selected: the core has its nose, so it goes to the pod.
        let (pod2, _) = bare_pod(&mut app, tube);
        app.ctx.selection = Selection::Part(tube);
        click(&mut app, 1);
        assert!(matches!(pod_kinds(&app, pod2)[0], PartKind::NoseCone { shape: NoseShape::Conical, .. }), "{:?}", pod_kinds(&app, pod2));
        assert_eq!(app.ctx.doc.rocket().parts().len(), 3);
    }

    #[test]
    fn a_pod_is_dragged_in_filled_with_fins_extended_moved_and_undone_as_one_body() {
        let (mut app, [_, tube, _]) = app();
        let before = app.ctx.doc.rocket().clone();
        app.update(Message::PartsBin(parts_bin::Message::Symmetry(2)));
        app.update(Message::Cursor(Point::new(100.0, 100.0)));
        // The pod row is the last of the Airframe list.
        let row = parts_bin::airframe_items().len() - 1;
        app.update(Message::PartsBin(parts_bin::Message::RowPress(parts_bin::RowId::new(parts_bin::Category::Airframe, row))));
        app.update(Message::Cursor(Point::new(160.0, 100.0)));
        assert!(app.ctx.drag.radial.as_ref().is_some_and(|d| d.kind.is_pod() && d.count == 2));
        vp(&mut app, Vp::Surface(Some(target(tube, 0.05, 0.0)), false));
        vp(&mut app, Vp::DropRadial);
        let pod = app.ctx.doc.rocket().pods().next().expect("a pod was placed").id;
        let body = app.ctx.doc.rocket().radial(pod).unwrap().kind.pod_body().unwrap().clone();
        assert_eq!((app.ctx.doc.rocket().radial(pod).unwrap().count, body.parts().len()), (2, 2));
        let pod_tube = body.parts()[1].id;
        // Fins land on the pod's tube; a part of the pod joins at a node of its stack.
        app.ctx.doc.begin_drag("Add fins");
        app.ctx.drag.radial = Some(crate::viewport::surface::RadialDrag {
            row: None,
            kind: RadialKind::FinSet { root_chord: 0.05, tip_chord: 0.02, span: 0.03, sweep: 0.02, thickness: 0.002 },
            count: 3,
            origin: None,
            target: Some(target(pod_tube, 0.05, 0.0)),
            detached: false,
        });
        vp(&mut app, Vp::DropRadial);
        let fins = app.ctx.doc.rocket().pod_of(pod_tube).and_then(|p| p.kind.pod_body()).unwrap().radials()[0].id;
        assert_eq!(app.ctx.doc.rocket().parent_of(fins), Some(pod_tube));
        app.ctx.doc.begin_drag("Add tube");
        app.ctx.drag.axial = Some(crate::viewport::assembly::Drag {
            row: None,
            origin: None,
            name: "Body tube",
            kind: bowshock_model::PartKind::BodyTube { length: 0.1, diameter: 0.0381 },
            snap: None,
            pod_snap: Some((pod, 2)),
            surface: None,
            count: 2,
        });
        vp(&mut app, Vp::DropAxial);
        assert_eq!(app.ctx.doc.rocket().radial(pod).unwrap().kind.pod_body().unwrap().parts().len(), 3);
        // Picking up any part of the pod carries the whole body, fins and all; cancelled, everything is back.
        let snapshot = app.ctx.doc.rocket().clone();
        vp(&mut app, Vp::Grab(Pick::Axial(pod_tube)));
        assert!(app.ctx.doc.rocket().pods().next().is_none() && app.ctx.drag.radial.as_ref().is_some_and(|d| d.origin.is_some()));
        app.update(Message::Cancel);
        assert_eq!(app.ctx.doc.rocket(), &snapshot);
        // Dropped elsewhere it moves with every part, ids unchanged.
        vp(&mut app, Vp::Grab(Pick::Axial(pod_tube)));
        vp(&mut app, Vp::Surface(Some(target(tube, 0.2, 1.5)), false));
        vp(&mut app, Vp::DropRadial);
        let moved = app.ctx.doc.rocket().radial(pod).unwrap();
        assert_eq!((moved.offset, moved.angle), (0.2, 1.5));
        assert!(app.ctx.doc.rocket().radial(fins).is_some() && app.ctx.doc.rocket().part(pod_tube).is_some());
        // Four steps above (the pod, its fins, the tube, the move) undo back to the rocket we started with.
        for _ in 0..4 {
            app.update(Message::Undo);
        }
        assert_eq!(app.ctx.doc.rocket().ids(), before.ids());
    }

    #[test]
    fn moving_or_deleting_an_axial_part_takes_its_radial_parts_along() {
        let (mut app, [nose, tube, transition]) = app();
        let id = app.ctx.doc.edit("Lug", |r| r.attach_radial(tube, 0.3, 0.5, 2, RadialKind::LaunchLug { length: 0.05, diameter: 0.01 }).unwrap());
        // Grab the tube: its radial part rides along (out of the rocket) and returns with it, offset unchanged.
        vp(&mut app, Vp::Grab(Pick::Axial(tube)));
        assert!(app.ctx.doc.rocket().radials().is_empty());
        app.update(Message::Cancel);
        assert_eq!(app.ctx.doc.rocket().radial(id).map(|r| (r.parent, r.offset)), Some((tube, 0.3)));
        // Move the transition forward of the tube: the tube's radial part is unaffected.
        vp(&mut app, Vp::Grab(Pick::Axial(transition)));
        vp(&mut app, Vp::SnapNode(Some(1)));
        vp(&mut app, Vp::DropAxial);
        assert_eq!(app.ctx.doc.rocket().index_of(tube), Some(2));
        assert_eq!(app.ctx.doc.rocket().radial(id).map(|r| r.parent), Some(tube));
        // Resizing the parent keeps the radial part on it.
        vp(&mut app, Vp::Edit(Edit::Begin(tube, "Drag face")));
        vp(&mut app, Vp::Edit(Edit::Dim(tube, Dim::Length, 0.2)));
        vp(&mut app, Vp::Edit(Edit::End));
        assert!((app.ctx.doc.rocket().radial(id).unwrap().offset - 0.15).abs() < 1e-9);
        // Deleting the tube deletes its radial part.
        vp(&mut app, Vp::Select(Some(Pick::Axial(tube))));
        app.update(Message::Delete);
        assert!(app.ctx.doc.rocket().radials().is_empty() && app.ctx.doc.rocket().index_of(nose) == Some(0));
    }

    #[test]
    fn resizing_a_fin_set_changes_the_group() {
        let (mut app, [_, tube, _]) = app();
        let fins = RadialKind::FinSet { root_chord: 0.1, tip_chord: 0.05, span: 0.05, sweep: 0.03, thickness: 0.003 };
        let id = app.ctx.doc.edit("Fins", |r| r.attach_radial(tube, 0.0, 0.0, 4, fins).unwrap());
        vp(&mut app, Vp::Edit(Edit::Begin(id, "Drag face")));
        vp(&mut app, Vp::Edit(Edit::Dim(id, Dim::Span, 0.09)));
        vp(&mut app, Vp::Edit(Edit::End));
        let radial = app.ctx.doc.rocket().radial(id).unwrap();
        assert_eq!((radial.kind.fin_span(), radial.count), (Some(0.09), 4));
    }

    #[test]
    fn undo_redo_shortcuts_and_the_dirty_flag() {
        let (mut app, _) = app();
        assert!(app.ctx.doc.is_dirty());
        let press = |c: &str, shift: bool| {
            let modifiers = if shift { keyboard::Modifiers::COMMAND | keyboard::Modifiers::SHIFT } else { keyboard::Modifiers::COMMAND };
            Message::Key(KeyPress { key: keyboard::Key::Character(c.into()), modifiers, captured: false })
        };
        app.update(press("z", false));
        assert_eq!(app.ctx.doc.rocket().parts().len(), 2);
        app.update(press("z", true));
        assert_eq!(app.ctx.doc.rocket().parts().len(), 3);
        app.update(press("z", false));
        app.update(press("y", false));
        assert_eq!(app.ctx.doc.rocket().parts().len(), 3);
    }

    #[test]
    fn the_example_loads_and_a_clean_document_is_replaced_without_asking() {
        let mut app = App::new();
        app.update(Message::Example);
        assert!(!app.ctx.doc.rocket().parts().is_empty() && !app.ctx.doc.is_dirty());
        assert!(app.ctx.physics.balance().is_some());
        app.update(Message::New);
        assert!(app.ctx.unsaved.is_none(), "no modal when nothing is unsaved");
        assert!(app.ctx.doc.rocket().parts().is_empty());
    }

    #[test]
    fn f2_renames_the_selected_part_and_otherwise_the_configuration() {
        let mut app = App::new();
        app.update(Message::Example);
        let id = app.ctx.doc.rocket().parts()[0].id;
        app.ctx.selection = Selection::Part(id);
        let f2 = KeyPress { key: keyboard::Key::Named(keyboard::key::Named::F2), modifiers: keyboard::Modifiers::empty(), captured: false };
        assert!(matches!(shortcut(&f2), Some(Message::Rename)));
        app.update(Message::Rename);
        app.update(Message::Inspector(inspector::Message::RenameInput("Cone".to_string())));
        app.update(Message::Inspector(inspector::Message::RenameCommit));
        assert_eq!(app.ctx.doc.rocket().part(id).unwrap().display_name(), "Cone");
        // Nothing selected: the configuration's name is edited instead, the part keeps its own.
        app.ctx.selection = Selection::None;
        app.update(Message::Rename);
        app.update(Message::Chrome(chrome::Message::RenameInput("Windy".to_string())));
        app.update(Message::Chrome(chrome::Message::CommitRename));
        assert_eq!(app.ctx.doc.rocket().part(id).unwrap().display_name(), "Cone");
        let rocket = app.ctx.doc.rocket();
        assert_eq!(rocket.configuration_name(rocket.active_config_id()).as_deref(), Some("Windy"));
    }

    /// An example with an unsaved edit, New pressed: the modal is up holding New.
    fn dirty_app() -> App {
        let mut app = App::new();
        app.update(Message::Example);
        app.ctx.doc.edit("Rename", |r| r.name = "changed".to_string());
        app.update(Message::New);
        app
    }

    #[test]
    fn new_with_unsaved_edits_opens_the_modal_and_blocks_other_commands() {
        let mut app = dirty_app();
        assert_eq!(app.ctx.unsaved, Some(Pending::New));
        assert!(!app.ctx.doc.rocket().parts().is_empty(), "nothing is replaced yet");
        app.update(Message::Delete);
        app.update(Message::Example);
        assert_eq!(app.ctx.unsaved, Some(Pending::New), "other commands are swallowed while it is open");
        assert!(app.ctx.doc.is_dirty());
    }

    #[test]
    fn dont_save_proceeds_and_cancel_keeps_the_document() {
        let mut app = dirty_app();
        app.update(Message::UnsavedCancel);
        assert!(app.ctx.unsaved.is_none() && app.ctx.doc.is_dirty() && !app.ctx.doc.rocket().parts().is_empty());
        app.update(Message::New);
        app.update(Message::Cancel);
        assert!(app.ctx.unsaved.is_none() && app.ctx.doc.is_dirty(), "Escape cancels");
        app.update(Message::New);
        app.update(Message::UnsavedDiscard);
        assert!(app.ctx.unsaved.is_none() && !app.ctx.doc.is_dirty() && app.ctx.doc.rocket().parts().is_empty());
    }

    #[test]
    fn save_writes_the_file_then_proceeds() {
        let path = std::env::temp_dir().join(format!("bowshock-{}-unsaved-modal.bowshock", std::process::id()));
        let mut app = App::new();
        app.update(Message::Example);
        app.ctx.doc.save_as(&path).unwrap();
        app.ctx.doc.edit("Rename", |r| r.name = "changed".to_string());
        app.update(Message::New);
        assert!(app.ctx.unsaved.is_some());
        app.update(Message::Key(KeyPress { key: keyboard::Key::Named(keyboard::key::Named::Enter), modifiers: keyboard::Modifiers::empty(), captured: false }));
        let saved = bowshock_model::Rocket::from_bytes(&std::fs::read(&path).unwrap()).unwrap();
        std::fs::remove_file(&path).ok();
        assert_eq!(saved.name, "changed", "Enter saves");
        assert!(app.ctx.unsaved.is_none() && app.ctx.doc.rocket().parts().is_empty() && !app.ctx.doc.is_dirty());
    }

    #[test]
    fn closing_the_window_with_unsaved_edits_asks_first() {
        let mut app = App::new();
        app.update(Message::Example);
        app.update(Message::CloseRequested);
        assert!(app.ctx.unsaved.is_none(), "a clean document closes without asking");
        app.ctx.doc.edit("Rename", |r| r.name = "changed".to_string());
        app.update(Message::CloseRequested);
        assert_eq!(app.ctx.unsaved, Some(Pending::Exit));
        app.update(Message::UnsavedCancel);
        assert!(app.ctx.unsaved.is_none() && app.ctx.doc.is_dirty(), "Cancel stays");
        app.update(Message::CloseRequested);
        app.update(Message::UnsavedSave);
        assert_eq!(app.ctx.saving_for, Some(Pending::Exit), "an untitled document asks where to save, then exits");
        app.update(Message::SavePath(None));
        assert!(app.ctx.saving_for.is_none() && app.ctx.doc.is_dirty(), "a cancelled Save As keeps the window open");
    }

    #[test]
    fn save_as_from_the_modal_proceeds_when_answered_and_aborts_when_cancelled() {
        let path = std::env::temp_dir().join(format!("bowshock-{}-unsaved-as.bowshock", std::process::id()));
        let mut app = dirty_app();
        app.update(Message::UnsavedSave);
        assert!(app.ctx.unsaved.is_none() && app.ctx.saving_for == Some(Pending::New), "an untitled document asks where to save");
        app.update(Message::SavePath(None));
        assert!(app.ctx.saving_for.is_none() && app.ctx.doc.is_dirty() && !app.ctx.doc.rocket().parts().is_empty(), "cancelled: nothing happens");
        app.update(Message::New);
        app.update(Message::UnsavedSave);
        app.update(Message::SavePath(Some(path.clone())));
        let written = path.exists();
        std::fs::remove_file(&path).ok();
        assert!(written && app.ctx.doc.rocket().parts().is_empty() && !app.ctx.doc.is_dirty());
    }

    #[test]
    fn configuration_commands_undo_and_the_status_values_follow_the_document() {
        use crate::chrome::Message as Chrome;
        let mut app = App::new();
        app.update(Message::Example);
        let first = app.ctx.doc.rocket().active_config_id();
        let margin = app.ctx.physics.balance().unwrap().margin_cal;
        assert!(margin > 1.0 && app.ctx.physics.config_margin(first) == Some(margin));

        // Duplicate makes the copy current; Undo brings the first back.
        app.update(Message::Chrome(Chrome::DuplicateConfig));
        let copy = app.ctx.doc.rocket().active_config_id();
        assert_ne!(copy, first);
        assert_eq!(app.ctx.doc.rocket().configurations().len(), 2);
        app.update(Message::Undo);
        assert_eq!(app.ctx.doc.rocket().active_config_id(), first);

        // Rename through the menu's field.
        app.update(Message::Chrome(Chrome::DuplicateConfig));
        app.update(Message::Chrome(Chrome::BeginRename));
        app.update(Message::Chrome(Chrome::RenameInput("Windy day".to_string())));
        app.update(Message::Chrome(Chrome::CommitRename));
        let rocket = app.ctx.doc.rocket();
        assert_eq!(rocket.configuration_name(rocket.active_config_id()).as_deref(), Some("Windy day"));

        // Delete: one is removed, the last one stays (with a message).
        let active = rocket.active_config_id();
        app.update(Message::Chrome(Chrome::DeleteConfig(active)));
        assert_eq!(app.ctx.doc.rocket().configurations().len(), 1);
        let last = app.ctx.doc.rocket().active_config_id();
        app.update(Message::Chrome(Chrome::DeleteConfig(last)));
        assert_eq!(app.ctx.doc.rocket().configurations().len(), 1);
        assert!(app.ctx.notice.is_some());
    }

    #[test]
    fn the_wheel_mid_radial_drag_steps_the_count_and_does_not_zoom() {
        use iced::widget::canvas::Program;
        let (mut app, _) = app();
        app.update(Message::PartsBin(parts_bin::Message::Category(parts_bin::Category::Fins)));
        app.update(Message::PartsBin(parts_bin::Message::Symmetry(3)));
        app.update(Message::Cursor(Point::new(100.0, 100.0)));
        app.update(Message::PartsBin(parts_bin::Message::RowPress(parts_bin::RowId::new(parts_bin::Category::Fins, 3))));
        app.update(Message::Cursor(Point::new(160.0, 100.0)));
        assert_eq!(app.ctx.drag.radial.as_ref().map(|d| d.count), Some(3));

        // The overlay that owns the viewport during the drag turns a wheel notch into a count step and captures the event, so
        // the camera's own wheel handler (zoom) never sees it.
        let camera = app.viewport.camera();
        let drag = app.ctx.drag.radial.clone().unwrap();
        let overlay = crate::viewport::surface::Overlay {
            rocket: app.ctx.doc.rocket(),
            drag: &drag,
            camera,
            snap: 0,
            snap_step: 0.001,
            cursor: None,
            margin: None,
            units: Default::default(),
        };
        let bounds = iced::Rectangle::new(Point::ORIGIN, iced::Size::new(800.0, 600.0));
        let cursor = mouse::Cursor::Available(Point::new(400.0, 300.0));
        let wheel = |y: f32| iced::Event::Mouse(mouse::Event::WheelScrolled { delta: mouse::ScrollDelta::Lines { x: 0.0, y } });
        let mut state = crate::viewport::surface::State::default();
        let (up, _, status) = overlay.update(&mut state, &wheel(1.0), bounds, cursor).expect("the overlay handles the wheel").into_inner();
        assert!(matches!(up, Some(Vp::CopiesStep(true))), "a notch up is the next count, not a zoom: {up:?}");
        assert_eq!(status, iced::event::Status::Captured);
        let (down, _, _) = overlay.update(&mut state, &wheel(-1.0), bounds, cursor).unwrap().into_inner();
        assert!(matches!(down, Some(Vp::CopiesStep(false))));

        // The viewport applies the step to the group in the air (and to the bin's count) and leaves the camera alone.
        vp(&mut app, up.unwrap());
        assert_eq!((app.ctx.drag.radial.as_ref().unwrap().count, app.ctx.drag.symmetry), (4, 4));
        vp(&mut app, down.unwrap());
        vp(&mut app, Vp::CopiesStep(false));
        assert_eq!(app.ctx.drag.radial.as_ref().unwrap().count, 2);
        assert_eq!(app.viewport.camera(), camera, "the wheel did not zoom");
    }

    #[test]
    fn readouts_that_a_handle_drag_changes_are_marked_until_the_release() {
        let (mut app, [_, tube, _]) = app();
        assert_eq!(app.ctx.changing(), live::Changed::default());
        vp(&mut app, Vp::Edit(Edit::Begin(tube, "Drag face")));
        assert_eq!(app.ctx.changing(), live::Changed::default(), "nothing has changed yet");
        vp(&mut app, Vp::Edit(Edit::Dim(tube, Dim::Length, 0.7)));
        let c = app.ctx.changing();
        assert!(c.mass && c.length && c.cg && c.cp, "a longer tube moves mass, length, CG and CP: {c:?}");
        assert!(!c.diameter, "the diameter did not move");
        // Dragged back to the start, the readout is regular again even though the drag goes on.
        vp(&mut app, Vp::Edit(Edit::Dim(tube, Dim::Length, 0.5)));
        assert_eq!(app.ctx.changing(), live::Changed::default());
        vp(&mut app, Vp::Edit(Edit::Dim(tube, Dim::Length, 0.6)));
        assert!(app.ctx.changing().length);
        app.update(Message::Released);
        vp(&mut app, Vp::Edit(Edit::End));
        assert_eq!(app.ctx.changing(), live::Changed::default(), "back to regular on release");
    }

    /// The tube with a fin set on it at `offset`, and the fins' id.
    fn finned(app: &mut App, tube: PartId, offset: f64) -> PartId {
        let id = app.ctx.doc.edit("Fins", |r| {
            r.attach_radial(tube, offset, 0.0, 3, RadialKind::FinSet { root_chord: 0.08, tip_chord: 0.04, span: 0.06, sweep: 0.03, thickness: 0.003 }).unwrap()
        });
        app.after_update();
        id
    }

    fn committed(app: &App) -> bowshock_sim::from_model::Balance {
        *app.ctx.physics.balance().expect("the fixture has a balance")
    }

    #[test]
    fn moving_a_fin_set_previews_the_balance_before_the_release_and_the_release_commits_it() {
        let (mut app, [_, tube, _]) = app();
        let fins = finned(&mut app, tube, 0.1);
        let was = committed(&app);
        let health = app.ctx.physics.warnings().to_vec();
        vp(&mut app, Vp::Grab(Pick::Radial { id: fins, instance: 0 }));
        // Picked up with no target yet: the committed balance, nothing to compare, and the health of the rocket before the
        // drag (never that of the rocket with the fins out of it).
        assert_eq!(app.ctx.shown().now, Some(was));
        assert_eq!(app.ctx.shown().committed, None);
        assert_eq!(app.ctx.physics.warnings(), health.as_slice());
        vp(&mut app, Vp::Surface(Some(target(tube, 0.35, 0.0)), false));
        let aft = app.ctx.shown();
        assert_eq!(aft.committed, Some(was));
        let aft_now = aft.now.expect("a preview");
        assert!(aft_now.cp > was.cp, "fins moved aft move the CP aft: {} vs {}", aft_now.cp, was.cp);
        assert!(app.ctx.changing().cp, "the CP readout goes semibold");
        // Further forward than the start: the CP goes the other way.
        vp(&mut app, Vp::Surface(Some(target(tube, 0.0, 0.0)), false));
        assert!(app.ctx.shown().now.unwrap().cp < was.cp);
        // No valid target any more: the committed balance again.
        vp(&mut app, Vp::Surface(None, false));
        assert_eq!((app.ctx.shown().now, app.ctx.shown().committed), (Some(was), None));
        // The gauge follows what is shown.
        vp(&mut app, Vp::Surface(Some(target(tube, 0.35, 0.0)), false));
        assert!((f64::from(app.ctx.physics.gauge().target()) - aft_now.margin_cal).abs() < 1e-5);
        let previewed_health = app.ctx.physics.warnings().to_vec();
        // Release: the committed balance and health are the previewed ones.
        vp(&mut app, Vp::DropRadial);
        assert_eq!(app.ctx.physics.warnings(), previewed_health.as_slice());
        let after = committed(&app);
        assert!((after.cp - aft_now.cp).abs() < 1e-12 && (after.cg - aft_now.cg).abs() < 1e-12 && (after.mass - aft_now.mass).abs() < 1e-12);
        assert_eq!(app.ctx.shown(), live::Shown { now: Some(after), committed: None });
        assert_eq!(app.ctx.changing(), live::Changed::default());
        assert!((f64::from(app.ctx.physics.gauge().target()) - after.margin_cal).abs() < 1e-5);
    }

    #[test]
    fn cancelling_a_carry_restores_the_committed_balance_everywhere() {
        let (mut app, [_, tube, _]) = app();
        let fins = finned(&mut app, tube, 0.1);
        let was = committed(&app);
        vp(&mut app, Vp::Grab(Pick::Radial { id: fins, instance: 0 }));
        vp(&mut app, Vp::Surface(Some(target(tube, 0.35, 0.0)), false));
        assert_ne!(app.ctx.shown().now, Some(was));
        app.update(Message::Cancel);
        assert_eq!(committed(&app), was);
        assert_eq!(app.ctx.shown(), live::Shown { now: Some(was), committed: None });
        assert!((f64::from(app.ctx.physics.gauge().target()) - was.margin_cal).abs() < 1e-5);
    }

    #[test]
    fn a_tube_dragged_in_from_the_bin_previews_its_node_and_shifts_the_cg() {
        let (mut app, [_, _, _]) = app();
        let was = committed(&app);
        let row = parts_bin::axial_catalogue().iter().position(|(n, _)| *n == "Body tube").unwrap();
        app.update(Message::PartsBin(parts_bin::Message::Category(parts_bin::Category::Airframe)));
        app.update(Message::Cursor(Point::new(100.0, 100.0)));
        app.update(Message::PartsBin(parts_bin::Message::RowPress(parts_bin::RowId::new(parts_bin::Category::Airframe, row))));
        app.update(Message::Cursor(Point::new(160.0, 100.0)));
        assert!(app.ctx.drag.axial.is_some());
        // No node: the committed balance.
        assert_eq!(app.ctx.shown().now, Some(was));
        // Between the nose and the tube: heavier up front, longer, the CG moves.
        vp(&mut app, Vp::SnapNode(Some(1)));
        let fore = app.ctx.shown();
        let fore_now = fore.now.expect("a preview");
        assert_eq!(fore.committed, Some(was));
        assert!(fore_now.mass > was.mass && fore_now.length > was.length && (fore_now.cg - was.cg).abs() > 1e-6);
        assert!(app.ctx.changing().mass && app.ctx.changing().cg);
        // The aft node gives a different CG than the fore node.
        vp(&mut app, Vp::SnapNode(Some(3)));
        let aft_now = app.ctx.shown().now.unwrap();
        assert!((aft_now.cg - fore_now.cg).abs() > 1e-6);
        // A pod on the tube's side previews too.
        let tube = app.ctx.doc.rocket().parts()[1].id;
        vp(&mut app, Vp::AxialSurface(Some(target(tube, 0.1, 0.0))));
        assert!(app.ctx.shown().now.is_some_and(|b| b.mass > was.mass));
        // Release at the fore node: committed equals the preview.
        vp(&mut app, Vp::SnapNode(Some(1)));
        vp(&mut app, Vp::DropAxial);
        let after = committed(&app);
        assert!((after.cg - fore_now.cg).abs() < 1e-12 && (after.cp - fore_now.cp).abs() < 1e-12 && (after.mass - fore_now.mass).abs() < 1e-12);
    }

    #[test]
    fn a_picked_up_part_and_a_radial_pod_preview_where_they_would_land() {
        let (mut app, [_, tube, transition]) = app();
        let was = committed(&app);
        // An axial part moved to another node keeps its mass: only the CG and CP move.
        vp(&mut app, Vp::Grab(Pick::Axial(transition)));
        vp(&mut app, Vp::SnapNode(Some(1)));
        let moved = app.ctx.shown().now.expect("a preview");
        assert!((moved.cg - was.cg).abs() > 1e-6, "the CG moves when the transition does");
        app.update(Message::Cancel);
        assert_eq!(committed(&app), was);
        vp(&mut app, Vp::Grab(Pick::Axial(transition)));
        vp(&mut app, Vp::SnapNode(Some(1)));
        vp(&mut app, Vp::DropAxial);
        assert!((committed(&app).cg - moved.cg).abs() < 1e-12 && (committed(&app).mass - moved.mass).abs() < 1e-12);
        app.update(Message::Undo);
        assert_eq!(committed(&app), was);
        // A pod's parts are carried with the pod: dragging it along the tube previews its CG.
        let pod = app.ctx.doc.edit("Pod", |r| {
            let body =
                bowshock_model::PodBody::single(bowshock_model::Part::new(0, PartKind::BodyTube { length: 0.2, diameter: 0.03 }), Vec::new(), Vec::new());
            r.attach_radial(tube, 0.0, 0.0, 2, RadialKind::Pod { gap: 0.0, body }).unwrap()
        });
        app.after_update();
        let with_pod = committed(&app);
        vp(&mut app, Vp::Grab(Pick::Radial { id: pod, instance: 0 }));
        vp(&mut app, Vp::Surface(Some(target(tube, 0.3, 0.0)), false));
        let shifted = app.ctx.shown().now.expect("a preview");
        assert!((shifted.mass - with_pod.mass).abs() < 1e-9 && shifted.cg > with_pod.cg);
        vp(&mut app, Vp::DropRadial);
        assert!((committed(&app).cg - shifted.cg).abs() < 1e-12);
    }

    /// The per-message cost of a preview on the two-stage example, release build: `cargo test --release -p bowshock
    /// the_preview_costs -- --nocapture`.
    #[test]
    fn the_preview_costs_a_few_milliseconds_at_most() {
        let mut app = App::new();
        let rocket = bowshock_model::Rocket::from_bytes(include_bytes!("../../assets/examples/two_stage.bowshock")).unwrap();
        app.ctx.doc.replace(rocket, None);
        app.ctx.loaded();
        app.after_update();
        let fins = app.ctx.doc.rocket().radials().iter().find(|r| matches!(r.kind, RadialKind::FinSet { .. })).expect("the example has fins").clone();
        vp(&mut app, Vp::Grab(Pick::Radial { id: fins.id, instance: 0 }));
        let mut total = std::time::Duration::ZERO;
        let steps = 40;
        for i in 0..steps {
            vp(&mut app, Vp::Surface(Some(target(fins.parent, 0.002 * f64::from(i), 0.0)), false));
            total += app.ctx.preview.cost;
            assert!(app.ctx.shown().committed.is_some(), "step {i} has a preview that differs from the committed balance");
        }
        let each = total / steps;
        eprintln!("preview per update (two_stage, {}): {each:?}", if cfg!(debug_assertions) { "debug" } else { "release" });
        assert!(app.ctx.shown().now.is_some());
        assert!(each < std::time::Duration::from_millis(200), "{each:?}");
        app.update(Message::Cancel);
    }

    #[test]
    fn a_fin_set_pulled_off_the_body_is_removed_on_empty_space_or_lands_on_another_tube() {
        let (mut app, [_, tube, transition]) = app();
        let fins = finned(&mut app, tube, 0.1);
        let (before, was) = (app.ctx.doc.rocket().clone(), committed(&app));
        // Pulled free of the body over empty space: the preview is the rocket without them, released they are removed (one
        // undo step, with a notice); the status strip gets the CP of the finless rocket while it is carried.
        vp(&mut app, Vp::Grab(Pick::Radial { id: fins, instance: 0 }));
        assert_eq!(app.ctx.drag.landing(), None);
        vp(&mut app, Vp::Surface(None, true));
        assert_eq!(app.ctx.drag.landing(), Some(drag::Landing::Removed));
        assert!(app.ctx.shown().now.is_some_and(|b| (b.cp - was.cp).abs() > 1e-6), "the preview is the rocket without its fins");
        vp(&mut app, Vp::DropRadial);
        assert!(app.ctx.doc.rocket().radial(fins).is_none() && app.ctx.selection == Selection::None);
        assert!(app.ctx.notice.as_ref().is_some_and(|(text, _)| text.starts_with("Removed Fin set")), "{:?}", app.ctx.notice);
        assert_eq!(app.ctx.doc.undo_label(), Some("Remove fin set"));
        app.update(Message::Undo);
        assert_eq!(app.ctx.doc.rocket(), &before);

        // Esc after pulling free puts them back exactly, with no notice.
        app.ctx.notice = None;
        vp(&mut app, Vp::Grab(Pick::Radial { id: fins, instance: 0 }));
        vp(&mut app, Vp::Surface(None, true));
        app.update(Message::Cancel);
        assert_eq!(app.ctx.doc.rocket(), &before);
        assert!(app.ctx.notice.is_none());

        // Back within reach of the body (not detached any more) with no target: it goes back as it was, not removed.
        vp(&mut app, Vp::Grab(Pick::Radial { id: fins, instance: 0 }));
        vp(&mut app, Vp::Surface(None, true));
        vp(&mut app, Vp::Surface(Some(target(tube, 0.2, 0.0)), false));
        vp(&mut app, Vp::DropRadial);
        assert_eq!(app.ctx.doc.rocket().radial(fins).map(|r| (r.parent, r.offset)), Some((tube, 0.2)));
        app.update(Message::Undo);
        assert_eq!(app.ctx.doc.rocket(), &before);

        // Pulled free and dropped on another tube (a part of the core, or of a pod): it lands there, one undo step.
        vp(&mut app, Vp::Grab(Pick::Radial { id: fins, instance: 0 }));
        vp(&mut app, Vp::Surface(None, true));
        vp(&mut app, Vp::Surface(Some(target(transition, 0.0, 2.0)), true));
        vp(&mut app, Vp::DropRadial);
        assert_eq!(app.ctx.doc.rocket().radial(fins).map(|r| (r.parent, r.angle, r.count)), Some((transition, 2.0, 3)));
        app.update(Message::Undo);
        assert_eq!(app.ctx.doc.rocket(), &before);
        let (_, pod_tube) = bare_pod(&mut app, tube);
        let with_pod = app.ctx.doc.rocket().clone();
        vp(&mut app, Vp::Grab(Pick::Radial { id: fins, instance: 0 }));
        vp(&mut app, Vp::Surface(None, true));
        vp(&mut app, Vp::Surface(Some(target(pod_tube, 0.05, 1.0)), true));
        vp(&mut app, Vp::DropRadial);
        assert_eq!(app.ctx.doc.rocket().parent_of(fins), Some(pod_tube));
        app.update(Message::Undo);
        assert_eq!(app.ctx.doc.rocket(), &with_pod);
    }

    /// A motor mount that was put in the nose cone by mistake (placed there by the model; the nose's bore is too narrow for it).
    fn mount_in_the_nose(app: &mut App, [nose, tube, _]: [PartId; 3]) -> PartId {
        let kind = bowshock_model::InternalKind::InnerTube { outer_diameter: 0.04, thickness: 0.002, length: 0.15, motor_mount: true, overhang: 0.0 };
        let mount = app.ctx.doc.edit("Mount", |r| {
            let id = r.attach_internal(tube, 0.0, kind).unwrap();
            r.move_internal(id, nose, 0.03).unwrap();
            id
        });
        app.after_update();
        mount
    }

    #[test]
    fn a_motor_mount_carried_from_the_nose_into_the_body_keeps_its_station_and_is_one_undo_step() {
        let (mut app, ids) = app();
        let [nose, tube, _] = ids;
        let mount = mount_in_the_nose(&mut app, ids);
        let (before, was) = (app.ctx.doc.rocket().clone(), committed(&app));
        assert_eq!(before.internal(mount).map(|p| p.parent), Some(nose));
        // Pressed on and dragged: the part is selected and one undo step opens; the tube around it is not touched.
        vp(&mut app, Vp::Edit(Edit::Carry(mount)));
        assert_eq!(app.ctx.selection, Selection::Part(mount));
        assert!(app.ctx.drag.axial.is_none() && app.ctx.drag.radial.is_none() && app.ctx.doc.dragging());
        // Over the tube (station 0.2 to 0.7), 0.35 further aft: the station is kept, the offset is from the tube's fore face.
        vp(&mut app, Vp::Edit(Edit::Place(mount, tube, 0.18)));
        let moved = app.ctx.doc.rocket().internal(mount).unwrap().clone();
        assert_eq!((moved.parent, moved.offset), (tube, 0.18));
        assert!((app.ctx.doc.rocket().part_station(mount).unwrap() - 0.38).abs() < 1e-12);
        // A motor mount is placed from the bottom of its body part, wherever it goes.
        assert_eq!(moved.anchor, Some(bowshock_model::Anchor::Bottom));
        // The CG moves live, before the release.
        assert!((committed(&app).cg - was.cg).abs() > 1e-6 && app.ctx.changing().cg);
        vp(&mut app, Vp::Edit(Edit::End));
        assert_eq!(app.ctx.doc.undo_label(), Some("Move motor mount"));
        assert_eq!(app.ctx.doc.rocket().internal(mount).map(|p| p.parent), Some(tube));
        app.update(Message::Undo);
        assert_eq!(app.ctx.doc.rocket(), &before);
        assert!((committed(&app).cg - was.cg).abs() < 1e-12);
        // Escape puts it back exactly.
        vp(&mut app, Vp::Edit(Edit::Carry(mount)));
        vp(&mut app, Vp::Edit(Edit::Place(mount, tube, 0.18)));
        vp(&mut app, Vp::Edit(Edit::Cancel));
        assert_eq!(app.ctx.doc.rocket(), &before);
    }

    #[test]
    fn an_internal_part_dragged_from_the_bin_over_the_viewport_opens_the_airframe_and_lands_inside_the_body_part() {
        use crate::viewport::InsideView;
        let (mut app, [_, tube, _]) = app();
        let (before, was) = (app.ctx.doc.rocket().clone(), committed(&app));
        let ring = bowshock_model::InternalKind::CenteringRing { outer_diameter: None, inner_diameter: None, thickness: 0.006 };
        let mount = bowshock_model::InternalKind::InnerTube { outer_diameter: 0.04, thickness: 0.002, length: 0.15, motor_mount: true, overhang: 0.0 };
        let target = |offset| crate::app::drag::InternalTarget { parent: tube, offset };
        assert_eq!(app.viewport.inside(), InsideView::Solid);
        // A centering ring: the view goes to X-ray, the preview moves the CG, the drop inserts it (anchored to the nearer end).
        drag::start_internal(&mut app.ctx, Some(0), "Centering ring", ring);
        app.after_update();
        assert_eq!(app.viewport.inside(), InsideView::XRay);
        vp(&mut app, Vp::InternalTarget(Some(target(0.05))));
        assert!(app.ctx.shown().now.is_some_and(|b| (b.cg - was.cg).abs() > 1e-9 && b.mass > was.mass));
        vp(&mut app, Vp::DropInternal);
        let [placed] = app.ctx.doc.rocket().internals() else { panic!("one internal part") };
        assert_eq!((placed.parent, placed.offset, placed.anchor), (tube, 0.05, Some(bowshock_model::Anchor::Top)));
        assert_eq!(app.viewport.inside(), InsideView::Solid, "the view it was before");
        assert_eq!(app.ctx.selection, Selection::Part(placed.id));
        app.update(Message::Undo);
        assert_eq!(app.ctx.doc.rocket(), &before);
        // A motor mount is anchored to the bottom.
        drag::start_internal(&mut app.ctx, Some(1), "Motor mount", mount);
        app.after_update();
        vp(&mut app, Vp::InternalTarget(Some(target(0.3))));
        vp(&mut app, Vp::DropInternal);
        assert_eq!(app.ctx.doc.rocket().internals()[0].anchor, Some(bowshock_model::Anchor::Bottom));
        app.update(Message::Undo);
        // Esc adds nothing and puts the view back to what it was (here the cutaway).
        vp(&mut app, Vp::InsideView(InsideView::Cutaway));
        drag::start_internal(&mut app.ctx, Some(0), "Bulkhead", bowshock_model::InternalKind::Bulkhead { diameter: None, thickness: 0.006 });
        app.after_update();
        vp(&mut app, Vp::InternalTarget(Some(target(0.1))));
        app.update(Message::Cancel);
        assert!(app.ctx.doc.rocket().internals().is_empty() && app.viewport.inside() == InsideView::Cutaway);
    }

    #[test]
    fn a_fin_set_pulled_free_takes_the_symmetry_selection_as_its_count_when_it_lands_again() {
        let (mut app, [_, tube, _]) = app();
        let fins = finned(&mut app, tube, 0.1);
        let before = app.ctx.doc.rocket().clone();
        assert_eq!(before.radial(fins).unwrap().count, 3);
        vp(&mut app, Vp::Grab(Pick::Radial { id: fins, instance: 0 }));
        vp(&mut app, Vp::Surface(None, true));
        // X (or the wheel) steps the count and makes it the Symmetry selection; so does the footer.
        app.update(Message::PartsBin(parts_bin::Message::CycleSymmetry(true)));
        assert_eq!((app.ctx.drag.radial.as_ref().unwrap().count, app.ctx.drag.symmetry), (4, 4));
        app.update(Message::PartsBin(parts_bin::Message::Symmetry(6)));
        assert_eq!(app.ctx.drag.radial.as_ref().unwrap().count, 6);
        app.update(Message::PartsBin(parts_bin::Message::Symmetry(4)));
        // Back onto the tube: the group lands with ×4, in one undo step.
        vp(&mut app, Vp::Surface(Some(target(tube, 0.2, 0.0)), true));
        vp(&mut app, Vp::DropRadial);
        let landed = app.ctx.doc.rocket().radial(fins).unwrap();
        assert_eq!((landed.count, landed.offset, landed.parent), (4, 0.2, tube));
        app.update(Message::Undo);
        assert_eq!(app.ctx.doc.rocket(), &before);
        assert_eq!(app.ctx.doc.rocket().radial(fins).unwrap().count, 3);
        // Still stuck (sliding): the footer does not recount a picked-up group.
        vp(&mut app, Vp::Grab(Pick::Radial { id: fins, instance: 0 }));
        app.update(Message::PartsBin(parts_bin::Message::Symmetry(8)));
        assert_eq!(app.ctx.drag.radial.as_ref().unwrap().count, 3);
        app.update(Message::Cancel);
    }

    #[test]
    fn pulling_a_fin_set_free_hands_it_the_symmetry_selection_without_pressing_anything() {
        let (mut app, [_, tube, _]) = app();
        let fins = finned(&mut app, tube, 0.1);
        app.update(Message::PartsBin(parts_bin::Message::Symmetry(4)));
        vp(&mut app, Vp::Grab(Pick::Radial { id: fins, instance: 0 }));
        // Still stuck: the group keeps its own count.
        assert_eq!(app.ctx.drag.radial.as_ref().unwrap().count, 3);
        // Fully pulled free: it takes the selection at once, and lands with it.
        vp(&mut app, Vp::Surface(None, true));
        assert_eq!(app.ctx.drag.radial.as_ref().unwrap().count, 4);
        vp(&mut app, Vp::Surface(Some(target(tube, 0.2, 0.0)), true));
        vp(&mut app, Vp::DropRadial);
        assert_eq!(app.ctx.doc.rocket().radial(fins).unwrap().count, 4);
    }

    #[test]
    fn the_symmetry_footer_and_x_recount_a_selected_fin_set_in_place() {
        let (mut app, [_, tube, _]) = app();
        let fins = finned(&mut app, tube, 0.1);
        let before = app.ctx.doc.rocket().radial(fins).unwrap().clone();
        app.ctx.selection = Selection::Part(fins);
        app.update(Message::PartsBin(parts_bin::Message::Symmetry(4)));
        let after = app.ctx.doc.rocket().radial(fins).unwrap().clone();
        assert_eq!((after.count, after.offset, after.parent), (4, before.offset, before.parent));
        app.update(Message::PartsBin(parts_bin::Message::CycleSymmetry(true)));
        assert_eq!(app.ctx.doc.rocket().radial(fins).unwrap().count, 6);
        app.update(Message::Undo);
        app.update(Message::Undo);
        assert_eq!(app.ctx.doc.rocket().radial(fins).unwrap().count, 3);
    }
    #[test]
    fn the_part_menu_opens_on_a_right_click_renames_and_deletes_in_one_undo_step() {
        use part_menu::Action;
        let (mut app, [_, tube, transition]) = app();
        app.ctx.cursor = Some(Point::new(300.0, 200.0));
        // The viewport: the right-click selects the part under it, then the menu opens at the cursor.
        app.update(Message::Viewport(Vp::PartMenu(Pick::Axial(tube))));
        assert_eq!(app.ctx.selection, Selection::Part(tube));
        app.update(Message::OpenPartMenu(tube));
        assert_eq!(app.ctx.part_menu, Some(part_menu::Open { part: tube, at: Point::new(300.0, 200.0) }));
        // Escape closes it.
        app.update(Message::Cancel);
        assert!(app.ctx.part_menu.is_none());
        // The viewport: a right-click on another part moves the selection to it.
        app.update(Message::Viewport(Vp::PartMenu(Pick::Axial(transition))));
        assert_eq!(app.ctx.selection, Selection::Part(transition));
        app.update(Message::OpenPartMenu(transition));
        assert_eq!(app.ctx.part_menu.map(|m| m.part), Some(transition));
        // Rename starts the inspector's inline rename (the F2 path): the commit renames the part.
        app.update(Message::PartMenu(Action::Rename));
        assert!(app.ctx.part_menu.is_none());
        app.update(Message::Inspector(inspector::Message::RenameInput("Adapter".to_string())));
        app.update(Message::Inspector(inspector::Message::RenameCommit));
        assert_eq!(app.ctx.doc.rocket().kind_of(transition).unwrap().name(), "Adapter");
        // Delete removes the part as one undo step.
        let before = app.ctx.doc.rocket().clone();
        app.update(Message::OpenPartMenu(transition));
        app.update(Message::PartMenu(Action::Delete));
        assert!(app.ctx.doc.rocket().kind_of(transition).is_none());
        app.update(Message::Undo);
        assert_eq!(app.ctx.doc.rocket(), &before);
    }

    #[test]
    fn the_scale_overlay_applies_one_undo_step() {
        let (mut app, [_, tube, _]) = app();
        let before = app.ctx.doc.rocket().clone();
        let length = before.length();
        app.update(Message::Scale(scale::Message::OpenWhole));
        assert!(app.ctx.scale.is_some());
        app.update(Message::Scale(scale::Message::Factor("2".into())));
        app.update(Message::Scale(scale::Message::Apply));
        assert!(app.ctx.scale.is_none());
        assert!((app.ctx.doc.rocket().length() - 2.0 * length).abs() < 1e-9);
        assert_eq!(app.ctx.doc.undo_label(), Some("Scale ×2.000"));
        app.update(Message::Undo);
        assert_eq!(app.ctx.doc.rocket(), &before, "one undo step restores the rocket");
        // An out-of-range factor cannot be applied; the parts form scales only the selection.
        app.update(Message::Scale(scale::Message::OpenWhole));
        app.update(Message::Scale(scale::Message::Factor("500".into())));
        app.update(Message::Scale(scale::Message::Apply));
        assert!(app.ctx.scale.is_some());
        app.update(Message::Cancel);
        assert!(app.ctx.scale.is_none());
        app.ctx.selection = Selection::Part(tube);
        app.update(Message::Scale(scale::Message::OpenParts));
        app.update(Message::Scale(scale::Message::Factor("2".into())));
        app.update(Message::Scale(scale::Message::Apply));
        assert!((app.ctx.doc.rocket().length() - (length + 0.5)).abs() < 1e-9);
    }

    #[test]
    fn a_chosen_unit_set_is_kept_in_the_settings_file_and_not_in_the_rocket() {
        use crate::units::{Family, Quantity};
        let file = std::env::temp_dir().join(format!("bowshock_app_units_{}.json", std::process::id()));
        let _ = std::fs::remove_file(&file);
        let (mut app, _) = app();
        app.ctx.settings_file = Some(file.clone());
        let rocket_before = app.ctx.doc.rocket().clone();

        let mixed = UnitSystem::MM_G_S.with(Family::Length, "in").with(Family::Mass, "kg");
        app.update(Message::Units(mixed));
        assert_eq!(app.ctx.units.label(), "in · kg · s");
        // Lengths read in inches and masses in kilograms through the same set.
        assert_eq!(ui::with_unit(0.0762, Quantity::Length, app.ctx.units), "3.00 in");
        assert_eq!(ui::with_unit(1.284, Quantity::Mass, app.ctx.units), "1.284 kg");
        // The next launch reads the choice back; the document is untouched (not dirty from a display preference).
        assert_eq!(settings::Settings::load(Some(&file)).units, mixed);
        assert_eq!(app.ctx.doc.rocket(), &rocket_before);

        // A preset replaces it, and so does a project-wide chip choice (cleared when the set changes).
        crate::units::set_unit_override(Quantity::Length, &Quantity::Length.units()[4]);
        app.update(Message::Units(UnitSystem::M_KG_S));
        assert_eq!(Quantity::Length.unit_for(app.ctx.units).symbol, "m");
        assert_eq!(settings::Settings::load(Some(&file)).units, UnitSystem::M_KG_S);
        let _ = std::fs::remove_file(&file);
    }
}
