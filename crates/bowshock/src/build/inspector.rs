//! The inspector: sections 01..05 for the selected part, the rocket's own inspector when nothing is selected, and the separation
//! inspector (design/components/Inspector, Staging). Minimal for now: a title and a note.
//!
//! Module contract (see `ARCHITECTURE.md`): [`State`] is this module's state, [`Message`] its messages; the app calls
//! [`State::update`], [`State::view`] and [`State::on_key`] with the shared [`Ctx`].

use iced::widget::{column, container, text};
use iced::{keyboard, Element, Fill, Task, Widget};

use crate::app::{self, ui, Ctx, Selection};
use crate::theme::{self, SANS_SEMIBOLD};
use crate::tokens::{SPACE_2, TEXT_BODY};

/// The inspector's messages.
#[derive(Debug, Clone)]
pub enum Message {}

/// The inspector's state.
#[derive(Debug, Default)]
pub struct State {}

impl State {
    /// An empty inspector state.
    pub fn new() -> Self {
        State {}
    }

    /// Handles one of the inspector's messages. Edits go through `ctx.doc` (use `edit_merging` for scrubbed fields).
    pub fn update(&mut self, message: Message, _ctx: &mut Ctx) -> Task<app::Message> {
        match message {}
    }

    /// The shortcut `key` maps to while the inspector is on screen, if any. Only called for keys no widget handled.
    pub fn on_key(&self, _key: &keyboard::Key, _modifiers: keyboard::Modifiers, _ctx: &Ctx) -> Option<Message> {
        None
    }

    /// The inspector pane.
    pub fn view<'a>(&'a self, ctx: &'a Ctx) -> Element<'a, Message> {
        let rocket = ctx.doc.rocket();
        let (title, note) = match ctx.selection {
            Selection::Part(id) => match (rocket.part(id), rocket.radial(id), rocket.internal(id)) {
                (Some(part), _, _) => (part.display_name(), "SELECTED".to_string()),
                (_, Some(radial), _) if radial.count > 1 => (format!("{} · LINKED ×{}", radial.kind.name(), radial.count), "SELECTED".to_string()),
                (_, Some(radial), _) => (radial.kind.name().to_string(), "SELECTED".to_string()),
                (_, _, Some(internal)) => (internal.kind.name().to_string(), "SELECTED".to_string()),
                _ => ("Rocket".to_string(), "NOTHING SELECTED".to_string()),
            },
            Selection::Separation(stage) => (format!("Separation · stage {stage}"), "SELECTED".to_string()),
            Selection::None => ("Rocket".to_string(), "NOTHING SELECTED".to_string()),
        };
        let body = column![text(title).font(SANS_SEMIBOLD).size(TEXT_BODY + 2.0), ui::label(note)].spacing(SPACE_2).padding(theme::PAD_PANEL);
        container(body).width(Fill).height(Fill).style(theme::panel).boxed()
    }
}
