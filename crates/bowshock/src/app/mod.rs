//! The application: one [`App`] holding the shared context ([`Ctx`]) and one state per area, one top-level [`Message`] that wraps
//! each area's own, and the routing between them. See `ARCHITECTURE.md` next to `main.rs` for the contract every module follows.

mod ctx;
pub mod dialogs;
pub mod document;
pub mod drag;
pub mod env;
mod files;
pub mod live;
mod message;
mod multi;
pub mod part_menu;
pub mod physics;
pub mod refs;
mod route;
pub mod scale;
pub mod selection_set;
pub mod settings;
pub mod symmetry;
#[cfg(test)]
mod tests;
pub mod ui;
pub mod worker;

pub(crate) use ctx::Pending;
pub use ctx::{Ctx, Mode, Selection};
pub use message::{KeyPress, Message};

use bowshock_model::PartId;
use iced::widget::{column, mouse_area};
use iced::{keyboard, mouse, Element, Task, Theme, Widget};

use crate::build::{self, inspector, lanes, parts_bin};
use crate::chrome;
use crate::theme;
use crate::viewport;
use crate::{analyze, export, prefs, print, simulate};
use route::part_ids;

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
    /// The Preferences window.
    prefs: prefs::State,
}

impl App {
    /// An empty document, Build mode.
    #[cfg(test)]
    pub fn new() -> Self {
        App::with_env(env::Env::for_tests())
    }

    /// The app over `env`, with nothing loaded.
    pub fn with_env(env: env::Env) -> Self {
        App {
            parts_bin: parts_bin::State::from_env(&env),
            analyze: analyze::State::from_env(&env),
            ctx: Ctx::with_env(env),
            layout: build::Layout::new(),
            drag_start: None,
            viewport: viewport::View::new(),
            lanes: lanes::State::new(),
            inspector: inspector::State::new(),
            simulate: simulate::State::new(),
            chrome: chrome::State::new(),
            export: export::State::new(),
            print: print::State::new(),
            prefs: prefs::State::new(),
        }
    }

    /// The app with the built-in example rocket loaded (tests; the binary starts through [`App::launch`]).
    #[cfg(test)]
    pub fn with_example() -> Self {
        App::launch(true, true)
    }

    /// The app as the binary starts it: empty, or with the built-in example when `example`. `BOWSHOCK_OPEN=<file>` opens that
    /// document (a `.bowshock`, or an OpenRocket `.ork` that is imported) instead of either; a drive script cannot press Ctrl+O,
    /// so this is how one starts from a given rocket (from a rocket whose CFD table is already solved and stored, say).
    pub fn launch(example: bool, animate: bool) -> Self {
        let env = if cfg!(test) { env::Env { animate, ..env::Env::for_tests() } } else { env::Env::from_process(animate) };
        let mut app = App::with_env(env);
        if example {
            files::example(&mut app.ctx);
        }
        if let Some(path) = app.ctx.env.open_path.clone() {
            files::open(&mut app.ctx, &path);
        }
        app.ctx.recent = files::Recent::load(app.ctx.env.recent_file.clone());
        // Unit choices are the user's, kept across documents and runs. Unit tests start from the defaults, never the real file.
        app.ctx.settings_file = app.ctx.env.settings_file.clone();
        let settings = settings::Settings::load(app.ctx.settings_file.as_deref());
        app.ctx.apply_settings(settings);
        app.after_update();
        // A headless `drive` run delivers no key subscription events, so `BOWSHOCK_DRIVE_TAB=1` presses Tab once at start-up.
        if app.ctx.env.drive_tab {
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
    /// The Preferences window's state (tests).
    #[cfg(test)]
    pub fn prefs(&self) -> &prefs::State {
        &self.prefs
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
        ctx.physics.sync(ctx.doc.rocket(), ctx.doc.revision(), ctx.units);
        let reloaded = std::mem::take(&mut ctx.reloaded);
        if reloaded {
            ctx.physics.snap_gauge();
        }
        ctx.sanitize_selection();
        self.simulate.sync(&self.ctx, reloaded);
        self.ctx.selected_flight = Some(self.simulate.selected_id());
        self.viewport.sync(&self.ctx);
        if std::mem::take(&mut self.ctx.camera_fit) {
            self.viewport.fit(self.ctx.env.animate);
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
        layers.extend(self.prefs.view(ctx));
        layers.extend(refs::menu_view(ctx));
        layers.extend(part_menu::view(ctx));
        layers.extend(scale::view(ctx));
        let root: Element<'_, Message> = iced::widget::Stack::with_children(layers).boxed();
        // The root sees every cursor move and any release the widgets below leave alone: that is what lets a drag begun in the
        // parts bin follow the cursor anywhere, and end (or be cancelled) wherever it is released.
        mouse_area(root).on_move(Message::Cursor).on_release(Message::Released).boxed()
    }
}
