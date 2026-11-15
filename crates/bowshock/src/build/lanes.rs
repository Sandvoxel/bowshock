//! Station Lanes: the canvas under the viewport that lines every part up with the model by station (design/components/
//! StationLanes). Placeholder: the ruler label only, until the lanes are built.
//!
//! Module contract (see `ARCHITECTURE.md`): [`State`] is this module's state, [`Message`] its messages; the app calls
//! [`State::update`], [`State::view`] and [`State::on_key`] with the shared [`Ctx`].

use iced::widget::{column, container};
use iced::{keyboard, Element, Fill, Task, Widget};

use crate::app::{self, ui, Ctx};
use crate::theme;

/// Station Lanes' messages.
#[derive(Debug, Clone)]
pub enum Message {}

/// Station Lanes' state.
#[derive(Debug, Default)]
pub struct State {}

impl State {
    /// An empty lanes state.
    pub fn new() -> Self {
        State {}
    }

    /// Handles one of the lanes' messages. Edits go through `ctx.doc`; selection through `ctx.selection`.
    pub fn update(&mut self, message: Message, _ctx: &mut Ctx) -> Task<app::Message> {
        match message {}
    }

    /// The shortcut `key` maps to while the lanes are on screen, if any. Only called for keys no widget handled.
    pub fn on_key(&self, _key: &keyboard::Key, _modifiers: keyboard::Modifiers, _ctx: &Ctx) -> Option<Message> {
        None
    }

    /// The lanes pane.
    pub fn view<'a>(&'a self, _ctx: &'a Ctx) -> Element<'a, Message> {
        container(column![ui::label("MM FROM TIP")].padding(theme::PAD_PANEL)).width(Fill).height(Fill).style(theme::ground).boxed()
    }
}
