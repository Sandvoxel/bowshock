//! UnitInput (design/components/UnitInput): a numeric field that knows its physical quantity. Label column, scrub grip,
//! mono value, optional badge and a unit chip, with the kit's states: default, focus, drag, pick, formula, auto, override
//! and error.
//!
//! * **Typing**: a number, a unit-suffixed value (`3in`, `76.2mm`, `1' 2"`) or arithmetic (`(120-50)/2`, `2*38`), parsed by
//!   [`crate::units`]. Enter or clicking away commits, Esc reverts. A value that does not parse keeps the last valid one
//!   and explains itself in a hint line under the field.
//! * **Scrub**: drag the grip (or Alt-drag the field) sideways; one pixel is one display step, `Shift` ×0.1, `Ctrl`/`⌘`
//!   ×10. [`UnitInput::on_scrub`] is published live, [`UnitInput::on_commit`] when the mouse is released.
//! * **Unit chip**: a click cycles the display unit within the quantity (mm → cm → m → in → ft). The choice lives in the
//!   widget; [`UnitInput::on_unit`] reports it so the app can make it project-wide.
//! * **References**: text containing `{id}` is a formula. With [`UnitInput::resolver`] it is evaluated, and the raw text is
//!   published through [`UnitInput::on_formula`] so the app can store the formula and show the `ƒ` badge
//!   ([`State::Formula`]). Typing `{` enters pick mode ([`UnitInput::on_pick_mode`]): the field edits as a row of characters
//!   and reference **chips** (`[■ Body tube · Ø]`, [`UnitInput::chips`]) instead of a text input, the app marks other fields
//!   [`State::PickTarget`] and answers a pick with [`UnitInput::insert`], which turns the open `{` into a chip. Typing `}`,
//!   Esc or a pick leaves pick mode; clicking a chip re-picks it, Backspace removes it.
//!
//! The edit buffer, focus, drag and unit choice are widget state, so a screen only keeps the SI value:
//!
//! ```ignore
//! unit_input("Root chord", self.root_chord, Quantity::Length, &self.units)
//!     .on_scrub(Message::RootChord)
//!     .on_commit(Message::RootChord)
//! ```

mod builder;
mod cells;
mod logic;
#[cfg(test)]
mod tests;
mod widget;

use cells::Cell;

use iced::advanced::layout::{self, Layout};
use iced::advanced::renderer::{self, Quad};
use iced::advanced::text::{Alignment, Ellipsis, LineHeight, Paragraph, Shaping, Text, Wrapping};
use iced::advanced::widget::operation::{Focusable, TextInput as TextInputState};
use iced::advanced::widget::{tree, Id, Meta, Operation, Tree};
use iced::advanced::{shell, Renderer as _, Shell, Widget};
use iced::widget::text_input;
use iced::{alignment, keyboard, mouse, Background, Border, Color, Element, Event, Font, Length, Pixels, Point, Rectangle, Renderer, Size, Theme};

use super::dashed;
use crate::theme::{tokens, MONO, SANS};
use crate::tokens::{text as type_scale, CONTROL_H, SPACE_2};
use crate::units::{self, Quantity, Unit, UnitSystem};

type TextRenderer = Renderer;

/// Width of the scrub grip at the field's left edge.
const GRIP_W: f32 = 16.0;
/// Height of a badge.
const BADGE_H: f32 = 16.0;
/// Default width of the label column.
pub const LABEL_W: f32 = super::field::LABEL_W;

/// How the field looks and behaves beyond the default. Focus and drag are tracked by the widget itself.
#[derive(Debug, Clone, PartialEq, Default)]
pub enum State {
    /// A plain value.
    #[default]
    Normal,
    /// The value is the result of a formula: `ƒ` badge. Editing shows the text given to [`UnitInput::formula`].
    Formula,
    /// A derived value: dashed border, no fill, `ink-muted` value, `AUTO` badge. Typing overrides it.
    Auto,
    /// An overridden value (mass): the value is `warn`, `OVR` badge.
    Override,
    /// `err` border and this message under the field; the last valid value is kept.
    Error(String),
    /// This field is the one being edited in pick mode: `accent` border plus an inset ring.
    Picking,
    /// Another field is in pick mode and this one can be clicked to insert a reference: dashed `accent` outline.
    PickTarget,
}

/// The small tag before the unit chip.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Badge<'a> {
    /// `AUTO`: derived from other values.
    Auto,
    /// `OVR`: the computed value is overridden.
    Override,
    /// `ƒ`: the value is a formula.
    Formula,
    /// `→ N`: this value is referenced by N other fields (the field also gets an `accent` bar on its left edge).
    Refs(usize),
    /// `PER CONFIG`: stored per motor configuration.
    PerConfig,
    /// Any other short tag.
    Text(&'a str),
}

impl Badge<'_> {
    /// The text of the badge.
    pub fn label(&self) -> String {
        match self {
            Badge::Auto => "AUTO".into(),
            Badge::Override => "OVR".into(),
            Badge::Formula => "ƒ".into(),
            Badge::Refs(n) => format!("→ {n}"),
            Badge::PerConfig => "PER CONFIG".into(),
            Badge::Text(s) => (*s).into(),
        }
    }
}

/// Which colour the value text takes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ValueTone {
    Ink,
    Muted,
    Warn,
    Accent,
}

#[derive(Debug, Clone)]
enum Inner {
    Input(String),
    Submit,
}

/// How a reference shows as a chip while a formula is edited.
#[derive(Debug, Clone, PartialEq)]
pub struct Chip {
    /// `Body tube · Ø`.
    pub label: String,
    /// The part's system colour (the square before the label).
    pub color: Color,
    /// The part is gone: the chip is red and struck through.
    pub broken: bool,
}

/// A numeric field with a physical quantity. Create it with [`unit_input`] or [`UnitInput::new`] and convert it to an
/// element with `.into()`.
pub struct UnitInput<'a, M> {
    label: &'a str,
    label_width: f32,
    value_si: f64,
    quantity: Quantity,
    system: UnitSystem,
    state: State,
    badge: Option<Badge<'a>>,
    hint: Option<&'a str>,
    formula: Option<&'a str>,
    insert: Option<(u64, &'a str)>,
    id: Option<Id>,
    on_commit: Option<Box<dyn Fn(f64) -> M + 'a>>,
    on_scrub: Option<Box<dyn Fn(f64) -> M + 'a>>,
    on_formula: Option<Box<dyn Fn(String, &'static Unit) -> M + 'a>>,
    on_unit: Option<Box<dyn Fn(&'static Unit) -> M + 'a>>,
    on_pick_mode: Option<Box<dyn Fn(bool) -> M + 'a>>,
    on_pick: Option<M>,
    on_badge: Option<M>,
    scrub_asks: Option<M>,
    resolve: Option<Box<dyn Fn(&str) -> Option<f64> + 'a>>,
    chips: Option<Box<dyn Fn(&str) -> Option<Chip> + 'a>>,
    formula_in: Option<Box<dyn Fn(&'static Unit) -> String + 'a>>,
    mixed: bool,
    input: Element<'a, Inner>,
}

/// A [`UnitInput`] showing `value_si` (SI) as `quantity` in the unit `units` selects for it.
pub fn unit_input<'a, M: Clone + 'a>(label: &'a str, value_si: f64, quantity: Quantity, units: &UnitSystem) -> UnitInput<'a, M> {
    UnitInput::new(label, value_si, quantity, units)
}

/// What the widget remembers between frames.
struct Memory {
    focused: bool,
    editing: bool,
    buffer: String,
    initial: String,
    unit: Option<&'static Unit>,
    drag: Option<Drag>,
    mods: keyboard::Modifiers,
    error: Option<String>,
    picking: bool,
    /// What the app was last told about pick mode.
    told_picking: bool,
    insert_seen: Option<u64>,
    unit_w: f32,
    badge_w: f32,
    hint_h: f32,
    /// The field is edited as cells (a formula with references) instead of by the text input.
    chip: bool,
    cells: Vec<Cell>,
    caret: usize,
}

#[derive(Clone, Copy)]
struct Drag {
    last_x: f32,
    /// The value in display units, unquantised.
    shown: f64,
}

impl Default for Memory {
    fn default() -> Self {
        Self {
            focused: false,
            editing: false,
            buffer: String::new(),
            initial: String::new(),
            unit: None,
            drag: None,
            mods: keyboard::Modifiers::default(),
            error: None,
            picking: false,
            told_picking: false,
            insert_seen: None,
            unit_w: 0.0,
            badge_w: 0.0,
            hint_h: 0.0,
            chip: false,
            cells: Vec::new(),
            caret: 0,
        }
    }
}

/// The regions of a field, in absolute coordinates.
struct Geo {
    label: Rectangle,
    field: Rectangle,
    grip: Rectangle,
    text: Rectangle,
    badge: Option<Rectangle>,
    formula: Option<Rectangle>,
    unit: Rectangle,
    hint: Rectangle,
}

impl<'a, M: Clone + 'a> From<UnitInput<'a, M>> for Element<'a, M> {
    fn from(input: UnitInput<'a, M>) -> Self {
        iced::Widget::boxed(input)
    }
}
