//! Message routing: the modal overlays' interception, the dispatch of every [`Message`] to its area, and the key map.

use bowshock_model::PartId;
use iced::{keyboard, Task};

use super::message::{pick_intercept, shortcut};
use super::{drag, files, part_menu, scale, App, KeyPress, Message, Mode, Pending, Selection};
use crate::build::{inspector, parts_bin};
use crate::viewport;
use crate::{chrome, export, prefs, print, simulate};

impl App {
    /// The modal overlays (unsaved changes, export, print, scale) take everything: Escape cancels, Enter confirms, and only the
    /// clock, the cursor, the layout, the viewport's size reports and the dialogs' own answers get through. `true` when `message`
    /// is blocked.
    pub(super) fn intercept_modals(&self, message: &mut Message) -> bool {
        use keyboard::key::Named;
        let enter = matches!(message, Message::Key(KeyPress { key: keyboard::Key::Named(Named::Enter), modifiers, .. }) if modifiers.is_empty());
        if self.ctx.unsaved.is_some() {
            match message {
                Message::Cancel => *message = Message::UnsavedCancel,
                _ if enter => *message = Message::UnsavedSave,
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
                _ => return true,
            }
        }
        if self.export.is_open() || self.print.is_open() {
            let printing = self.print.is_open();
            match message {
                Message::Cancel => *message = if printing { Message::Print(print::Message::Close) } else { Message::Export(export::Message::Close) },
                _ if enter => *message = if printing { Message::Print(print::Message::Save) } else { Message::Export(export::Message::Save) },
                Message::Frame(_)
                | Message::Cursor(_)
                | Message::Resized(_)
                | Message::Released
                | Message::Key(_)
                | Message::Export(_)
                | Message::Print(_)
                | Message::Viewport(viewport::Message::Fit(_) | viewport::Message::Resized(_))
                | Message::Chrome(chrome::Message::Nothing) => {}
                _ => return true,
            }
        }
        if self.prefs.is_open() {
            match message {
                Message::Cancel => *message = Message::Prefs(prefs::Message::Close),
                Message::Frame(_)
                | Message::Cursor(_)
                | Message::Resized(_)
                | Message::Released
                | Message::Key(_)
                | Message::Prefs(_)
                // Results of work that was already running (a solve, a flight) still arrive; the scrim keeps clicks from the page.
                | Message::Analyze(_)
                | Message::Simulate(_)
                | Message::Viewport(viewport::Message::Fit(_) | viewport::Message::Resized(_))
                | Message::Chrome(chrome::Message::Nothing) => {}
                _ => return true,
            }
        }
        if self.ctx.scale.is_some() {
            match message {
                Message::Cancel => *message = Message::Scale(scale::Message::Close),
                _ if enter => *message = Message::Scale(scale::Message::Apply),
                Message::Frame(_)
                | Message::Cursor(_)
                | Message::Resized(_)
                | Message::Released
                | Message::Key(_)
                | Message::Scale(_)
                | Message::Viewport(viewport::Message::Fit(_) | viewport::Message::Resized(_))
                | Message::Chrome(chrome::Message::Nothing) => {}
                _ => return true,
            }
        }
        false
    }

    pub(super) fn route(&mut self, mut message: Message) -> Task<Message> {
        if self.intercept_modals(&mut message) {
            return Task::none();
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
            Message::ToggleTheme => ctx.set_dark(!ctx.dark),
            Message::Units(units) => ctx.set_units(units),
            Message::Frame(now) => {
                if let Some((_, since)) = ctx.flash {
                    if crate::viewport::flash::done(crate::viewport::flash::age(since, now, ctx.env.freeze_flash)) {
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
                self.viewport.cancel(ctx.env.animate);
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
            Message::Prefs(m) => return self.prefs.update(m, ctx),
        }
        Task::none()
    }

    /// Global shortcuts first (they work whatever has focus), then the active area's own.
    pub(super) fn key(&mut self, press: KeyPress) -> Task<Message> {
        if let Some(message) = shortcut(&press) {
            return self.route(message);
        }
        // Typing in Preferences is not a shortcut of the screen behind it.
        if self.prefs.is_open() {
            return Task::none();
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
}

/// Every part id in `rocket`: axial parts first, then radial, then internal.
pub(super) fn part_ids(rocket: &bowshock_model::Rocket) -> Vec<PartId> {
    rocket.parts().iter().map(|p| p.id).chain(rocket.radials().iter().map(|p| p.id)).chain(rocket.internals().iter().map(|p| p.id)).collect()
}
