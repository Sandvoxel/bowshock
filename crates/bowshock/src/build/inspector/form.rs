//! Field builders of the inspector: a [`Form`] knows the unit system and the limit message to show, and turns a [`Field`] and
//! its value into a `UnitInput` wired to [`Message::Edit`] (live while scrubbing, final on commit). Selects, read-only rows and
//! the value rows of the mass section live here too.

use std::fmt;

use super::edit::Field;
use super::Message;
use crate::app::Ctx;
use crate::build::parts_bin::INSPECTOR_FIRST_FIELD;
use crate::theme::{self, MONO};
use crate::tokens::{CONTROL_H, SPACE_2, TEXT_BODY};
use crate::units::Quantity;
use crate::widgets::field::{labeled, select};
use crate::widgets::unit_input::{unit_input, Badge, State};
use bowshock_model::Dim;
use iced::widget::Id;
use iced::widget::{container, mouse_area, row, text, Space};
use iced::{Element, Fill, Widget};

/// An option of a select: the value and what the menu shows.
#[derive(Clone, Debug, PartialEq)]
pub struct Choice<T> {
    /// What choosing it means.
    pub value: T,
    /// The menu text.
    pub label: String,
}

impl<T> fmt::Display for Choice<T> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.label)
    }
}

/// A select over `(value, label)` options with `selected` chosen (the option equal to it, if any), publishing `on` of the value.
pub fn choose<'a, T>(options: Vec<(T, String)>, selected: &T, on: impl Fn(T) -> Message + 'a) -> Element<'a, Message>
where
    T: Clone + PartialEq + 'a,
{
    let choices: Vec<Choice<T>> = options.into_iter().map(|(value, label)| Choice { value, label }).collect();
    let current = choices.iter().find(|c| c.value == *selected).cloned();
    select(choices, current, move |c: Choice<T>| on(c.value))
}

/// The unit system and limit hint the fields of one inspector view share.
pub struct Form<'a> {
    /// The shared context (unit system).
    pub ctx: &'a Ctx,
    /// The dimension that was last limited, and why ("MINIMUM LENGTH"), shown as a hint under its field.
    pub limit: Option<(Dim, &'static str)>,
    /// The part's first editable field, which takes the id the parts bin focuses on a double click.
    pub first: Option<Field>,
}

impl<'a> Form<'a> {
    fn base(&self, label: &'static str, value: f64, quantity: Quantity, field: Field) -> crate::widgets::unit_input::UnitInput<'a, Message> {
        let id = if Some(field) == self.first { Id::from(INSPECTOR_FIRST_FIELD) } else { Id::from(field.key()) };
        let mut input = unit_input(label, value, quantity, &self.ctx.units)
            .id(id)
            .on_scrub(move |v| Message::Edit(field, v, true))
            .on_commit(move |v| Message::Edit(field, v, false));
        if let (Field::Dim(dim), Some((limited, reason))) = (field, self.limit) {
            if dim == limited {
                input = input.hint(reason);
            }
        }
        input
    }

    /// A plain numeric field: scrubbed live, committed on Enter.
    pub fn num(&self, label: &'static str, value: f64, quantity: Quantity, field: Field) -> Element<'a, Message> {
        self.base(label, value, quantity, field).into()
    }

    /// A derived value (`AUTO`, dashed): typing into it flips which of the pair is driven.
    pub fn derived(&self, label: &'static str, value: f64, quantity: Quantity, field: Field) -> Element<'a, Message> {
        self.base(label, value, quantity, field).state(State::Auto).into()
    }

    /// A derived value that cannot be typed into (`AUTO`, dashed, no edit).
    pub fn fixed(&self, label: &'static str, value: f64, quantity: Quantity) -> Element<'a, Message> {
        unit_input(label, value, quantity, &self.ctx.units).state(State::Auto).into()
    }

    /// A value that follows its parent until typed: `AUTO` while it does, a plain field otherwise. Right-click on an explicit
    /// value puts it back to automatic (`reset`).
    pub fn auto_or(&self, label: &'static str, value: f64, quantity: Quantity, field: Field, auto: bool, reset: Message) -> Element<'a, Message> {
        let input = self.base(label, value, quantity, field);
        if auto {
            input.state(State::Auto).into()
        } else {
            let element: Element<'a, Message> = input.hint("Right-click for auto").into();
            mouse_area(element).on_right_press(reset).boxed()
        }
    }

    /// An overridden value: `warn` with `OVR`.
    pub fn overridden(&self, label: &'static str, value: f64, quantity: Quantity, field: Field) -> Element<'a, Message> {
        self.base(label, value, quantity, field).state(State::Override).into()
    }

    /// A field stored per configuration: the `PER CONFIG` badge.
    pub fn per_config(&self, label: &'static str, value: f64, quantity: Quantity, field: Field) -> Element<'a, Message> {
        self.base(label, value, quantity, field).badge(Badge::PerConfig).into()
    }
}

/// A label with a mono value: `Computed  42.6 g`.
pub fn value_row<'a>(name: &'a str, value: String, action: Option<Element<'a, Message>>) -> Element<'a, Message> {
    let value = container(text(value).font(MONO).size(12.5).style(theme::ink)).height(CONTROL_H).center_y(CONTROL_H);
    let mut content = row![value, Space::new().width(Fill)].spacing(SPACE_2).align_y(iced::Center);
    if let Some(action) = action {
        content = content.push(action);
    }
    labeled(name, content.boxed())
}

/// A label with plain text (a name, a read-only description).
pub fn text_row<'a>(name: &'a str, value: String) -> Element<'a, Message> {
    let value = container(text(value).size(TEXT_BODY).style(theme::ink)).height(CONTROL_H).center_y(CONTROL_H);
    labeled(name, value.boxed())
}
