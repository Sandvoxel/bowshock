//! The Simulate screen (design/components/SimulateScreen): flights list and conditions on the left, the events strip and linked
//! plots in the centre, results and the ground track on the right. Placeholder: the three-column frame and the Run wiring.
//!
//! Module contract (see `ARCHITECTURE.md`): [`State`] is this module's state, [`Message`] its messages; the app calls
//! [`State::update`], [`State::view`], [`State::on_key`], [`State::tick`] and [`State::animating`] with the shared [`Ctx`].
//! [`Message::Run`] is what the top bar's Simulate button (`F5`) sends after switching to this screen: it must run the
//! document's active configuration (`ctx.doc.rocket()`, `ctx.physics.motors()`, `bowshock_sim::from_model::sim_input`).

use iced::widget::{column, text};
use iced::{keyboard, Element, Task, Widget};
use std::time::Instant;

use crate::app::{self, ui, Ctx};
use crate::tokens::{SPACE_2, TEXT_BODY};

/// The Simulate screen's messages.
#[derive(Debug, Clone)]
pub enum Message {
    /// Run the current configuration's flight (top bar button, `F5`).
    Run,
}

/// The Simulate screen's state.
#[derive(Debug, Default)]
pub struct State {
    /// How many times Run was pressed (until flights exist).
    runs: u32,
}

impl State {
    /// An empty Simulate state.
    pub fn new() -> Self {
        State::default()
    }

    /// Handles one of the screen's messages.
    pub fn update(&mut self, message: Message, ctx: &mut Ctx) -> Task<app::Message> {
        match message {
            Message::Run => {
                self.runs += 1;
                ctx.notify(format!("Run {}: {}", self.runs, ctx.doc.rocket().active_configuration().display_name()));
            }
        }
        Task::none()
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
    pub fn view<'a>(&'a self, ctx: &'a Ctx) -> Element<'a, Message> {
        let config = ctx.doc.rocket().active_configuration().display_name();
        let left = column![ui::label("FLIGHTS"), text(config).size(TEXT_BODY), ui::label("CONDITIONS")].spacing(SPACE_2);
        let centre = column![ui::label("EVENTS"), ui::label("ALTITUDE · VELOCITY · ACCELERATION")].spacing(SPACE_2);
        let right = column![ui::label("RESULTS"), ui::label("GROUND TRACK")].spacing(SPACE_2);
        ui::three_columns(left.boxed(), centre.boxed(), right.boxed())
    }
}
