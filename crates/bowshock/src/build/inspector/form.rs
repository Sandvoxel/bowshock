//! Field builders of the inspector: a [`Form`] knows the unit system and the limit message to show, and turns a [`Field`] and
//! its value into a `UnitInput` wired to [`Message::Edit`] (live while scrubbing, final on commit). Selects, read-only rows and
//! the value rows of the mass section live here too.

use std::fmt;

use super::edit::Field;
use super::Message;
use crate::app::refs;
use crate::app::{Ctx, Selection};
use crate::build::parts_bin::INSPECTOR_FIRST_FIELD;
use crate::theme::{self, MONO};
use crate::tokens::{CONTROL_H, SPACE_2, TEXT_BODY};
use crate::units::{self, Quantity};
use crate::widgets::button::{button, Weight};
use crate::widgets::field::{labeled, select, LABEL_W};
use crate::widgets::unit_input::{unit_input, Badge, State, UnitInput};
use bowshock_model::{Dim, FieldRef};
use iced::widget::Id;
use iced::widget::{column, container, mouse_area, row, text, tooltip, Space};
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
    pub(super) fn base(&self, label: &'static str, value: f64, quantity: Quantity, field: Field) -> UnitInput<'a, Message> {
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
        self.wire(input, field)
    }

    /// The reference a field of the selected part is: its part and dimension (`None` for a field that is not one).
    fn reference(&self, field: Field) -> Option<FieldRef> {
        let (Selection::Part(id), Field::Dim(dim)) = (self.ctx.selection, field) else { return None };
        self.ctx.doc.rocket().dim(id, dim)?;
        Some(FieldRef::new(id, dim))
    }

    /// Whether a field is another field's pick target right now (the dashed accent outline).
    fn is_pick_target(&self, field: Field) -> bool {
        self.reference(field).is_some_and(|r| self.ctx.refs.is_picking() && !self.ctx.refs.is_target(r))
    }

    /// The reference states of a field (design/components/UnitInput, *References*): typing `{` picks, other fields are pick
    /// targets meanwhile, a committed formula shows `ƒ` and its result, a source shows `→ N`, a refused formula shows why.
    fn wire(&self, mut input: UnitInput<'a, Message>, field: Field) -> UnitInput<'a, Message> {
        let Some(r) = self.reference(field) else { return input };
        let ctx = self.ctx;
        let (rocket, refs, tokens) = (ctx.doc.rocket(), &ctx.refs, ctx.tokens());
        input = input.resolver(move |token| refs::resolve(rocket, token)).chips(move |token| refs::chip(rocket, tokens, token));
        // A position is edited relative to an anchor the formula would not know about: it can be referenced, not computed.
        let computable = !matches!(field, Field::Dim(Dim::Position)) && rocket.dim(r.part, r.dim).is_some_and(|v| rocket.clamp_dim(r.part, r.dim, v).is_some());
        let formula = rocket.formula(r.part, r.dim).filter(|_| computable);
        if computable {
            input = input.on_pick_mode(move |on| Message::PickMode(on.then_some(r))).on_formula(move |text, unit| Message::Formula(r, text, unit));
            if let Some(insert) = refs.insert.as_ref().filter(|i| i.target == r) {
                input = input.insert(insert.token, &insert.text);
            }
        }
        let dependants = rocket.dependants(r.part, r.dim).len();
        if dependants > 0 {
            input = input.badge(Badge::Refs(dependants)).on_badge(Message::Dependants(r));
        }
        if let Some(formula) = formula {
            let quantity = refs::dim_quantity(r.dim);
            input = input
                .state(State::Formula)
                .formula_in(move |unit| units::formula_text(&formula.expr, quantity, unit, &|p| rocket.formula_name(formula, p)).unwrap_or_default())
                .on_scrub_attempt(Message::ScrubFormula(r));
            if let Some(name) = refs::broken_name(rocket, r.part, r.dim) {
                input = input.state(State::Error(format!("Broken reference: {name} was deleted. Re-pick or convert to a value."))).badge(Badge::Formula);
            }
        }
        if let Some((target, message)) = &refs.error {
            if *target == r {
                input = input.state(State::Error(message.clone()));
                if formula.is_some() && dependants == 0 {
                    input = input.badge(Badge::Formula);
                }
            }
        }
        if self.is_pick_target(field) {
            input = input.state(State::PickTarget).on_pick(Message::PickField(r));
        }
        input
    }

    /// A field as an element with what hangs under it: the formula in a tooltip, the *Replace formula with a value?* question,
    /// the *Re-pick* / *Convert to value* choices of a broken reference, the list of dependants of a source.
    pub(super) fn finish(&self, input: UnitInput<'a, Message>, field: Field) -> Element<'a, Message> {
        let element: Element<'a, Message> = input.into();
        let Some(r) = self.reference(field) else { return element };
        let ctx = self.ctx;
        let (rocket, refs) = (ctx.doc.rocket(), &ctx.refs);
        let indent = || Space::new().width(LABEL_W + SPACE_2).boxed();
        let note = |what: String, actions: Vec<Element<'a, Message>>| -> Element<'a, Message> {
            let body = column![text(what).size(12).style(theme::colored(|t| t.warn)), row(actions).spacing(SPACE_2)].spacing(SPACE_2);
            row![indent(), body.boxed()].boxed()
        };
        let mut cells: Vec<Element<'a, Message>> = Vec::new();
        match rocket.formula(r.part, r.dim) {
            Some(formula) => {
                let quantity = refs::dim_quantity(r.dim);
                let unit = quantity.unit_for(ctx.units);
                let shown = units::formula_text(&formula.expr, quantity, unit, &|p| rocket.formula_name(formula, p)).unwrap_or_default();
                let tip = container(text(format!("= {}", refs::pretty(rocket, ctx.tokens(), &shown))).font(MONO).size(12).style(theme::ink))
                    .padding([4.0, SPACE_2])
                    .style(theme::tooltip);
                cells.push(tooltip(element, tip).position(tooltip::Position::Top).gap(4).boxed());
            }
            None => cells.push(element),
        }
        if refs.confirm == Some(r) {
            cells.push(note(
                "Replace formula with a value?".into(),
                vec![
                    button("Replace", Weight::Secondary).on_press(Message::ReplaceFormula(r, true)).into(),
                    button("Cancel", Weight::Ghost).on_press(Message::ReplaceFormula(r, false)).into(),
                ],
            ));
        }
        if refs::broken_name(rocket, r.part, r.dim).is_some() {
            let actions: Vec<Element<'a, Message>> = vec![
                indent(),
                button("Re-pick", Weight::Secondary).on_press(Message::Focus(r.dim)).into(),
                button("Convert", Weight::Ghost).on_press(Message::ConvertFormula(r)).into(),
            ];
            cells.push(row(actions).spacing(SPACE_2).boxed());
        }
        if refs.dependants == Some(r) {
            let mut jumps: Vec<Element<'a, Message>> = vec![text("USED BY").font(MONO).size(10.5).style(theme::faint).boxed()];
            for dep in rocket.dependants(r.part, r.dim) {
                let name = rocket.kind_of(dep.part).map(|p| p.name()).unwrap_or_default();
                let label = format!("{name} · {}", refs::dim_name(rocket, dep.part, dep.dim));
                jumps.push(iced::widget::button(text(label).size(12)).padding([3.0, SPACE_2]).style(theme::secondary).on_press(Message::Jump(dep)).boxed());
            }
            cells.push(row![indent(), column(jumps).spacing(4).boxed()].boxed());
        }
        if cells.len() == 1 {
            return cells.remove(0);
        }
        column(cells).spacing(SPACE_2).boxed()
    }

    /// A plain numeric field: scrubbed live, committed on Enter.
    pub fn num(&self, label: &'static str, value: f64, quantity: Quantity, field: Field) -> Element<'a, Message> {
        self.finish(self.base(label, value, quantity, field), field)
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
            // An automatic field that is another's pick target keeps the target look and click.
            let input = if self.is_pick_target(field) { input } else { input.state(State::Auto) };
            self.finish(input, field)
        } else {
            let element: Element<'a, Message> = self.finish(input.hint("Right-click for auto"), field);
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
