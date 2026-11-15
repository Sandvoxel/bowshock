//! The Analyze screen (design/components/AnalyzeScreen): analyses list, method and conditions on the left, linked plots vs Mach
//! in the centre, values at the cursor on the right. Placeholder: the three-column frame.
//!
//! Module contract (see `ARCHITECTURE.md`): [`State`] is this module's state, [`Message`] its messages; the app calls
//! [`State::update`], [`State::view`], [`State::on_key`], [`State::tick`] and [`State::animating`] with the shared [`Ctx`].

use iced::widget::column;
use iced::{keyboard, Element, Task, Widget};
use std::time::Instant;

use crate::app::{self, ui, Ctx};
use crate::tokens::SPACE_2;

/// The Analyze screen's messages.
#[derive(Debug, Clone)]
pub enum Message {}

/// The Analyze screen's state.
#[derive(Debug, Default)]
pub struct State {}

impl State {
    /// An empty Analyze state.
    pub fn new() -> Self {
        State::default()
    }

    /// Handles one of the screen's messages.
    pub fn update(&mut self, message: Message, _ctx: &mut Ctx) -> Task<app::Message> {
        match message {}
    }

    /// The shortcut `key` maps to on this screen, if any. Only called for keys no widget handled.
    pub fn on_key(&self, _key: &keyboard::Key, _modifiers: keyboard::Modifiers, _ctx: &Ctx) -> Option<Message> {
        None
    }

    /// Advances animations (called on frame ticks while [`animating`](Self::animating)).
    pub fn tick(&mut self, _now: Instant) {}

    /// Whether the screen needs frame ticks.
    pub fn animating(&self) -> bool {
        false
    }

    /// The screen: three columns under the shared top bar and status strip.
    pub fn view<'a>(&'a self, _ctx: &'a Ctx) -> Element<'a, Message> {
        let left = column![ui::label("ANALYSES"), ui::label("METHOD"), ui::label("CONDITIONS")].spacing(SPACE_2);
        let centre = column![ui::label("STATIC MARGIN · CP AND CG · DRAG VS MACH")].spacing(SPACE_2);
        let right = column![ui::label("VALUES AT CURSOR")].spacing(SPACE_2);
        ui::three_columns(left.boxed(), centre.boxed(), right.boxed())
    }
}
