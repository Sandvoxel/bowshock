//! The application: one [`App`] holding the shared context ([`Ctx`]) and one state per area, one top-level [`Message`] that wraps
//! each area's own, and the routing between them. See `ARCHITECTURE.md` next to `main.rs` for the contract every module follows.

pub mod document;
pub mod drag;
mod files;
pub mod physics;
pub mod refs;
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
use crate::{analyze, simulate};
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

/// An unsaved-changes guard: the first press of a destructive file command only warns, the second proceeds.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Armed {
    New,
    Open,
    Example,
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
    /// The cursor in window coordinates, tracked app-wide so a drag can leave the parts bin.
    pub cursor: Option<Point>,
    /// The mode on show.
    pub mode: Mode,
    /// Dark theme (else light).
    pub dark: bool,
    /// "Compare all in lanes": the lanes draw every configuration's motor and CG.
    pub compare_configs: bool,
    /// A transient status message and when it expires.
    pub notice: Option<(String, Instant)>,
    /// Ask the viewport to frame the rocket once its mesh is current (set after adding a part or loading a document).
    pub camera_fit: bool,
    /// The document was just replaced: select nothing and let the gauge jump rather than animate.
    reloaded: bool,
    armed: Option<Armed>,
    /// Which copy of the selected symmetry group is the clicked one, and a pending rejoin (see [`symmetry`]).
    pub symmetry: symmetry::SymmetryUi,
    /// Pick mode and the formula UI's transient state (see [`refs`]).
    pub refs: refs::Ui,
}

impl Ctx {
    fn new() -> Self {
        Ctx {
            doc: Document::default(),
            selection: Selection::None,
            drag: Drags::default(),
            physics: Physics::new(),
            units: UnitSystem::default(),
            cursor: None,
            mode: Mode::Build,
            dark: true,
            compare_configs: false,
            notice: None,
            camera_fit: false,
            reloaded: false,
            armed: None,
            symmetry: symmetry::SymmetryUi::default(),
            refs: refs::Ui::default(),
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
        self.notice.is_some() || self.physics.animating()
    }

    /// Call after replacing the document wholesale (New, Open, the example).
    pub(crate) fn loaded(&mut self) {
        self.selection = Selection::None;
        self.symmetry = symmetry::SymmetryUi::default();
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
    /// The Simulate button (F5): switch to Simulate and run the current configuration's flight.
    Run,
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
}

/// The application state.
pub struct App {
    ctx: Ctx,
    layout: build::Layout,
    parts_bin: parts_bin::State,
    viewport: viewport::View,
    lanes: lanes::State,
    inspector: inspector::State,
    simulate: simulate::State,
    analyze: analyze::State,
    chrome: chrome::State,
}

impl App {
    /// An empty document, Build mode.
    pub fn new() -> Self {
        App {
            ctx: Ctx::new(),
            layout: build::Layout::new(),
            parts_bin: parts_bin::State::new(),
            viewport: viewport::View::new(),
            lanes: lanes::State::new(),
            inspector: inspector::State::new(),
            simulate: simulate::State::new(),
            analyze: analyze::State::new(),
            chrome: chrome::State::new(),
        }
    }

    /// The app with the built-in example rocket loaded.
    pub fn with_example() -> Self {
        let mut app = App::new();
        files::example(&mut app.ctx);
        app.after_update();
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

    /// Brings derived state up to date after a test changed the context directly.
    #[cfg(test)]
    pub fn sync(&mut self) {
        self.after_update();
    }

    /// Handles `message`, then brings everything derived from the document up to date.
    pub fn update(&mut self, message: Message) -> Task<Message> {
        let task = self.route(message);
        self.after_update();
        // The Analyze screen starts its off-thread computations (and rebuilds its plots) after every message while it shows.
        if self.ctx.mode == Mode::Analyze {
            return Task::batch([task, self.analyze.refresh(&self.ctx)]);
        }
        task
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
        self.viewport.sync(&self.ctx);
        if std::mem::take(&mut self.ctx.camera_fit) {
            self.viewport.fit();
        }
    }

    fn route(&mut self, message: Message) -> Task<Message> {
        // While a field is in pick mode a click on a part opens that part's value menu instead of selecting or grabbing it.
        if self.ctx.refs.is_picking() {
            if let Some(hit) = pick_intercept(&message) {
                if let (Some(part), Some(at)) = (hit, self.ctx.cursor) {
                    self.ctx.refs.open_menu(part, at);
                }
                return Task::none();
            }
        }
        let ctx = &mut self.ctx;
        match message {
            Message::Resized(e) => self.layout.resize(e),
            Message::Mode(mode) => {
                ctx.mode = mode;
                self.chrome.dismiss();
            }
            Message::ToggleTheme => ctx.dark = !ctx.dark,
            Message::Units(units) => ctx.units = units,
            Message::Frame(now) => {
                self.viewport.tick(now);
                ctx.physics.tick(now);
                self.simulate.tick(now);
                self.analyze.tick(now);
                if ctx.notice.as_ref().is_some_and(|(_, until)| now >= *until) {
                    ctx.notice = None;
                    ctx.armed = None;
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
                let task = self.parts_bin.update(parts_bin::Message::Cancel, ctx);
                drag::cancel(ctx);
                self.chrome.dismiss();
                self.viewport.cancel();
                return task;
            }
            Message::Key(press) => return self.key(press),
            // Parts are only deleted where they are shown and selected for editing: the Build screen.
            Message::Delete if ctx.mode == Mode::Build => drag::delete_selection(ctx),
            Message::Delete => {}
            Message::Undo => {
                if let Some(label) = ctx.doc.undo() {
                    ctx.notify(format!("Undid {}", label.to_lowercase()));
                }
            }
            Message::Redo => {
                if let Some(label) = ctx.doc.redo() {
                    ctx.notify(format!("Redid {}", label.to_lowercase()));
                }
            }
            Message::New => {
                if files::guard(ctx, Armed::New) {
                    files::new(ctx);
                }
            }
            Message::Open => {
                if files::guard(ctx, Armed::Open) {
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
            Message::SavePath(path) => {
                if let Some(path) = path {
                    files::save_to(ctx, &path);
                }
            }
            Message::Example => {
                if files::guard(ctx, Armed::Example) {
                    files::example(ctx);
                }
            }
            Message::Run => {
                ctx.mode = Mode::Simulate;
                self.chrome.dismiss();
                return self.simulate.update(simulate::Message::Run, ctx);
            }
            Message::PartsBin(m) => return self.parts_bin.update(m, ctx),
            Message::Viewport(m) => return self.viewport.update(m, ctx),
            Message::Lanes(m) => return self.lanes.update(m, ctx),
            Message::Inspector(m) => return self.inspector.update(m, ctx),
            Message::Simulate(m) => return self.simulate.update(m, ctx),
            Message::Analyze(m) => return self.analyze.update(m, ctx),
            Message::Chrome(m) => return self.chrome.update(m, ctx),
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
                _ => None,
            }
        });
        let animating = self.viewport.animating() || self.ctx.animating() || self.simulate.animating() || self.analyze.animating();
        let frames = animating.then(|| iced::window::frames().map(Message::Frame));
        iced::Subscription::batch([events].into_iter().chain(frames))
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
        layers.extend(refs::menu_view(ctx));
        let root: Element<'_, Message> = iced::widget::Stack::with_children(layers).boxed();
        // The root sees every cursor move and any release the widgets below leave alone: that is what lets a drag begun in the
        // parts bin follow the cursor anywhere, and end (or be cancelled) wherever it is released.
        mouse_area(root).on_move(Message::Cursor).on_release(Message::Released).boxed()
    }
}

/// What a click means while a field is in pick mode (the viewport and the lanes are pickable): `Some(Some(part))` opens the part's
/// value menu, `Some(None)` is swallowed (a click on nothing, a grab, a drag of a bar), `None` is not a pick at all.
fn pick_intercept(message: &Message) -> Option<Option<PartId>> {
    use crate::build::lanes::Message as Lanes;
    use crate::viewport::Message as Viewport;
    match message {
        Message::Viewport(Viewport::Select(Some(pick))) => Some(Some(pick.id())),
        Message::Viewport(Viewport::SelectPart(Some(id))) => Some(Some(*id)),
        Message::Viewport(Viewport::Select(None) | Viewport::SelectPart(None) | Viewport::SelectSeparation(_) | Viewport::Grab(_) | Viewport::Focus(_)) => {
            Some(None)
        }
        Message::Lanes(Lanes::Select(Some(key))) => Some(Some(key.part())),
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
        keyboard::Key::Named(Named::F2) => Some(Message::Chrome(chrome::Message::BeginRename)),
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
        vp(&mut app, Vp::Edit(Edit::Begin(tube)));
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
        vp(&mut app, Vp::Edit(Edit::Begin(tube)));
        vp(&mut app, Vp::Edit(Edit::Dim(tube, Dim::Length, 0.3)));
        vp(&mut app, Vp::Edit(Edit::Cancel));
        assert_eq!(app.ctx.doc.rocket().part(tube).unwrap().kind, kind);
        app.update(Message::Undo);
        assert_eq!(parts(&app), before);
    }

    fn target(parent: PartId, offset: f64, angle: f64) -> SurfaceTarget {
        SurfaceTarget { parent, offset, angle }
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
        vp(&mut app, Vp::Surface(Some(target(tube, 0.1, 1.0))));
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
        vp(&mut app, Vp::Surface(Some(target(tube, 0.1, 1.0))));
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
        vp(&mut app, Vp::Surface(Some(target(transition, 0.0, 2.0))));
        vp(&mut app, Vp::DropRadial);
        let moved = app.ctx.doc.rocket().radial(id).unwrap();
        assert_eq!((moved.parent, moved.count, moved.angle), (transition, 3, 2.0));
        // Deleting the selected group removes all of it; its ids are not reused.
        app.update(Message::Delete);
        assert!(app.ctx.doc.rocket().radials().is_empty() && app.ctx.selection == Selection::None);
        assert_eq!(app.ctx.doc.rocket().parts().len(), 3);
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
        vp(&mut app, Vp::Edit(Edit::Begin(tube)));
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
        vp(&mut app, Vp::Edit(Edit::Begin(id)));
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
    fn the_example_loads_and_unsaved_work_is_guarded() {
        let mut app = App::new();
        app.update(Message::Example);
        assert!(!app.ctx.doc.rocket().parts().is_empty() && !app.ctx.doc.is_dirty());
        assert!(app.ctx.physics.balance().is_some());
        // A change makes New ask twice.
        app.update(Message::Delete);
        app.ctx.doc.edit("Rename", |r| r.name = "changed".to_string());
        app.update(Message::New);
        assert!(!app.ctx.doc.rocket().parts().is_empty());
        app.update(Message::New);
        assert!(app.ctx.doc.rocket().parts().is_empty() && !app.ctx.doc.is_dirty());
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
}
