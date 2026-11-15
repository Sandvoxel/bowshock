//! The inspector's symmetry surfaces (design/components/Symmetry, Inspector): the bar under the header with the `LINKED ×N`
//! tag and *Detach* / *Split all* / *Relink* / *Rejoin group*, the confirmation before a count change would drop overrides,
//! the choice of whose settings win when a free part rejoins a group, and the per-field *Unlink for this fin* / *Relink field*
//! menu with the link icon after each field of a group that has overrides.
//!
//! The clicked copy (A, B, C) is `ctx.symmetry`'s: a right-click on a field unlinks it for that copy only, and the field then
//! shows that copy's own value in the override style. Every action is one document edit (one undo step); the pending state
//! (open menu, count confirmation, rejoin choice) is view state and never reaches the document.

use bowshock_model::{Dim, PartId, RadialField, RadialPart, Rocket, Winner};
use iced::widget::{button as plain_button, column, container, mouse_area, row, text, Space};
use iced::{mouse, Element, Fill, Widget};

use super::edit::Field;
use super::part::Params;
use super::silhouette;
use super::{Message, State};
use crate::app::Ctx;
use crate::theme::{self, MONO};
use crate::tokens::{text as type_scale, SPACE_1, SPACE_2, SPACE_3, TEXT_BODY, TEXT_LABEL};
use crate::units::Quantity;
use crate::widgets::button::{button, Weight};
use crate::widgets::unit_input::State as InputState;

/// Width of the link icon after a field.
const ICON_W: f32 = 14.0;

/// The selected radial group and the copy on show, for the field builders.
pub struct Link<'a> {
    /// The group.
    pub group: &'a RadialPart,
    /// The copy on show (0 is A).
    pub copy: usize,
    /// The field whose *Unlink / Relink* menu is open.
    pub menu: Option<Field>,
}

impl<'a> Link<'a> {
    /// The link context of part `id` (a radial group of two or more copies), if it has one.
    pub fn of(state: &State, ctx: &Ctx, rocket: &'a Rocket, id: PartId) -> Option<Link<'a>> {
        let group = rocket.radial(id).filter(|g| g.count > 1)?;
        let copy = ctx.symmetry.copy_in(rocket, id);
        let menu = state.field_menu.filter(|(part, c, _)| *part == id && *c == copy).map(|(.., field)| field);
        Some(Link { group, copy, menu })
    }
}

/// A numeric field of a radial part with the symmetry affordances: right-click opens *Unlink for this fin* (or *Relink field*
/// when this copy already has its own value), an unlinked field shows in the override style, and once the group has any
/// override every field of it carries a link icon (broken `warn` for an unlinked one, `accent` for a linked one).
/// Without a group (or for a field a copy cannot hold alone) it is a plain field.
pub fn linked<'a>(p: &Params<'a>, label: &'static str, value: f64, quantity: Quantity, dim: Dim) -> Element<'a, Message> {
    let f = &p.form;
    let field = Field::Dim(dim);
    let Some(link) = p.link.as_ref() else {
        return f.num(label, value, quantity, field);
    };
    let Some(rf) = RadialField::of_dim(dim).filter(|rf| rf.applies_to(&link.group.kind)) else {
        return f.num(label, value, quantity, field);
    };
    let overridden = link.group.is_overridden(link.copy, rf);
    let mut input = f.base(label, value, quantity, field);
    if overridden {
        input = input.state(InputState::Override);
    }
    let input = f.finish(input, field);
    let mut cells: Vec<Element<'a, Message>> = vec![mouse_area(input).on_right_press(Message::FieldMenu(field)).boxed()];
    if link.group.has_overrides() {
        let icon: Element<'a, Message> = if overridden {
            let glyph: Element<'a, Message> = container(silhouette::unlink(ICON_W, |t| t.warn)).width(ICON_W).boxed();
            mouse_area(glyph).on_press(Message::Relink(field)).interaction(mouse::Interaction::Pointer).boxed()
        } else {
            container(silhouette::link(ICON_W, |t| t.accent)).width(ICON_W).boxed()
        };
        cells.push(icon);
    }
    let line: Element<'a, Message> = row(cells).spacing(SPACE_2).align_y(iced::Center).boxed();
    if link.menu != Some(field) {
        return line;
    }
    let (label, message) = if overridden { ("Relink field", Message::Relink(field)) } else { ("Unlink for this fin", Message::Unlink(field)) };
    let action: Element<'a, Message> = button(label, Weight::Ghost).on_press(message).into();
    let item: Element<'a, Message> = container(action).style(theme::overlay).boxed();
    let menu: Element<'a, Message> = container(item).width(Fill).align_right(Fill).boxed();
    column(vec![line, menu]).spacing(SPACE_2).boxed()
}

/// A text button whose label is built at view time (the library button takes `&str`s that outlive the view).
fn text_button<'a>(label: String, message: Message) -> Element<'a, Message> {
    plain_button(text(label).size(TEXT_BODY)).padding([4.0, SPACE_2]).style(theme::secondary).on_press(message).boxed()
}

/// The tag of a group or of a free part: an outlined mono label with an optional link glyph.
fn tag<'a>(label: String, glyph: Option<Element<'a, Message>>, color: fn(&crate::tokens::Tokens) -> iced::Color) -> Element<'a, Message> {
    let label = text(label).font(MONO).size(type_scale::NUM_LABEL.size).style(theme::colored(color));
    let mut cells: Vec<Element<'a, Message>> = Vec::new();
    cells.extend(glyph);
    cells.push(label.boxed());
    container(row(cells).spacing(6).align_y(iced::Center)).padding([2.0, SPACE_2]).style(theme::outlined(color)).boxed()
}

/// The name a part goes by in a sentence: `Fins ×3`, `Fin B`.
fn name_of(r: &RadialPart) -> String {
    if r.count > 1 {
        format!("{} ×{}", r.display_name(), r.count)
    } else {
        r.display_name()
    }
}

/// The symmetry bar of radial part `id`: the `LINKED ×N` tag (with `· K OVR` when copies have unlinked fields) and its actions
/// for a group, `FREE` and *Rejoin group* for a part of one copy, plus the confirmations and choices in progress. `None` for
/// a part with nothing to say (a lone lug).
pub fn bar<'a>(state: &'a State, ctx: &'a Ctx, id: PartId) -> Option<Element<'a, Message>> {
    let rocket = ctx.doc.rocket();
    let r = rocket.radial(id)?;
    let mut rows: Vec<Element<'a, Message>> = Vec::new();
    let candidates = rocket.rejoin_candidates(id);
    if r.count > 1 {
        let copy = ctx.symmetry.copy_in(rocket, id);
        let overrides = r.overrides.len();
        let (label, color): (String, fn(&crate::tokens::Tokens) -> iced::Color) =
            if overrides > 0 { (format!("LINKED ×{} · {overrides} OVR", r.count), |t| t.warn) } else { (format!("LINKED ×{}", r.count), |t| t.accent) };
        let glyph = silhouette::link(11.0, color);
        let mut buttons: Vec<Element<'a, Message>> = Vec::new();
        if r.overridden_fields(copy).is_empty() {
            buttons.push(button("Detach", Weight::Secondary).small().on_press(Message::Detach).into());
        } else {
            // The kit swaps Detach for Relink while this copy has unlinked fields.
            buttons.push(button("Relink", Weight::Secondary).small().on_press(Message::RelinkAll).into());
        }
        buttons.push(button("Split all", Weight::Secondary).small().on_press(Message::SplitAll).into());
        let buttons = row(buttons).spacing(SPACE_1 + 2.0);
        if overrides > 0 {
            // The longer tag leaves no room beside the actions: they drop to their own line.
            rows.push(tag(label, Some(glyph), color));
            rows.push(buttons.boxed());
        } else {
            rows.push(
                row(vec![tag(label, Some(glyph), color), Space::new().width(Fill).boxed(), buttons.boxed()]).spacing(SPACE_2).align_y(iced::Center).boxed(),
            );
        }
    } else if r.kind.is_fin() || !candidates.is_empty() {
        let mut rejoin = button("Rejoin group", Weight::Secondary).small();
        rejoin = if candidates.is_empty() { rejoin.disabled("No group of the same kind to rejoin") } else { rejoin.on_press(Message::RejoinStart) };
        rows.push(
            row(vec![rejoin.into(), Space::new().width(Fill).boxed(), tag("FREE".into(), None, |t| t.ink_muted)])
                .spacing(SPACE_2)
                .align_y(iced::Center)
                .boxed(),
        );
    }

    if let Some((part, count)) = state.confirm_count.filter(|(part, _)| *part == id) {
        let _ = part;
        let n = r.overrides.len();
        let note = format!("Changing the count re-spaces the copies and drops {n} per-copy override{}.", if n == 1 { "" } else { "s" });
        rows.push(text(note).size(TEXT_LABEL).style(theme::muted).boxed());
        rows.push(
            row(vec![text_button(format!("Change to {count}"), Message::ConfirmCount), button("Cancel", Weight::Ghost).on_press(Message::CancelCount).into()])
                .spacing(SPACE_2)
                .boxed(),
        );
    }

    if state.rejoin_picking == Some(id) && ctx.symmetry.rejoin.is_none_or(|j| j.part != id) {
        rows.push(text("Rejoin which group?").size(TEXT_LABEL).style(theme::muted).boxed());
        let mut choices: Vec<Element<'a, Message>> =
            candidates.iter().filter_map(|g| rocket.radial(*g)).map(|g| text_button(name_of(g), Message::RejoinWith(g.id))).collect();
        choices.push(button("Cancel", Weight::Ghost).on_press(Message::RejoinCancel).into());
        rows.push(row(choices).spacing(SPACE_2).wrap().boxed());
    }

    if let Some(join) = ctx.symmetry.rejoin.filter(|j| j.part == id) {
        if let Some(group) = rocket.radial(join.group) {
            rows.push(text(format!("Rejoin {} to {}: whose settings win?", name_of(r), name_of(group))).size(TEXT_LABEL).style(theme::muted).boxed());
            rows.push(
                row(vec![
                    text_button("Group's".into(), Message::RejoinChoose(Winner::Group)),
                    text_button(format!("{}'s", r.display_name()), Message::RejoinChoose(Winner::Part)),
                    button("Cancel", Weight::Ghost).on_press(Message::RejoinCancel).into(),
                ])
                .spacing(SPACE_2)
                .boxed(),
            );
        }
    }

    if rows.is_empty() {
        return None;
    }
    Some(container(column(rows).spacing(SPACE_2)).padding([SPACE_2 - 2.0, SPACE_3]).width(Fill).boxed())
}

#[cfg(test)]
mod tests {
    use bowshock_model::{Dim, PartId, RadialField, Rocket, Winner};

    use super::super::edit::Field;
    use super::super::{Message, State};
    use crate::app::{App, Message as AppMessage, Selection};

    /// The example rocket (three fins A, B, C) with the fins selected and an inspector state of its own.
    struct Fx {
        app: App,
        state: State,
        fins: PartId,
    }

    impl Fx {
        fn new() -> Fx {
            let mut app = App::with_example();
            let fins = app.ctx().doc.rocket().radials().iter().find(|r| r.kind.is_fin()).map(|r| r.id).unwrap();
            app.ctx_mut().selection = Selection::Part(fins);
            Fx { app, state: State::new(), fins }
        }

        fn send(&mut self, message: Message) {
            let _ = self.state.update(message, self.app.ctx_mut());
            self.app.sync();
        }

        fn rocket(&self) -> &Rocket {
            self.app.ctx().doc.rocket()
        }

        /// Selects copy `index` (A is 0) the way the header letter does.
        fn copy(&mut self, index: usize) {
            self.app.ctx_mut().symmetry.instance = Some((self.fins, index));
        }

        fn cant(&self, index: usize) -> f64 {
            self.rocket().radial(self.fins).unwrap().effective(index).cant
        }

        /// Views of every part build with the state as it is (a panic here is a bug in a field or the bar).
        fn views(&mut self) {
            let _ = self.state.view(self.app.ctx());
        }
    }

    const CANT: Field = Field::Dim(Dim::Cant);

    #[test]
    fn a_right_clicked_field_is_unlinked_for_the_clicked_copy_only_and_relinks() {
        let mut fx = Fx::new();
        fx.copy(1);
        fx.send(Message::FieldMenu(CANT));
        assert_eq!(fx.state.field_menu, Some((fx.fins, 1, CANT)));
        fx.views();
        fx.send(Message::Unlink(CANT));
        assert_eq!(fx.state.field_menu, None);
        assert!(fx.rocket().radial(fx.fins).unwrap().is_overridden(1, RadialField::Cant));
        // Editing the field now edits copy B alone, in one scrubbed undo step.
        for cant in [0.01, 0.02, 0.035] {
            fx.send(Message::Edit(CANT, cant, true));
        }
        fx.send(Message::Edit(CANT, 0.035, false));
        assert_eq!((fx.cant(0), fx.cant(1), fx.cant(2)), (0.0, 0.035, 0.0));
        assert_eq!(fx.rocket().radial(fx.fins).unwrap().cant, 0.0);
        // Copy A's field is still the group's: it moves A and C but not the unlinked B.
        fx.copy(0);
        fx.send(Message::Edit(CANT, 0.02, false));
        assert_eq!((fx.cant(0), fx.cant(1), fx.cant(2)), (0.02, 0.035, 0.02));
        fx.views();
        // Each step undoes alone: the group edit, then the scrub, then the unlink.
        let _ = fx.app.update(AppMessage::Undo);
        assert_eq!(fx.cant(0), 0.0);
        let _ = fx.app.update(AppMessage::Undo);
        assert_eq!(fx.cant(1), 0.0);
        assert!(fx.rocket().radial(fx.fins).unwrap().is_overridden(1, RadialField::Cant));
        let _ = fx.app.update(AppMessage::Undo);
        assert!(!fx.rocket().radial(fx.fins).unwrap().has_overrides());
        let _ = fx.app.update(AppMessage::Redo);
        let _ = fx.app.update(AppMessage::Redo);
        let _ = fx.app.update(AppMessage::Redo);
        assert_eq!((fx.cant(0), fx.cant(1)), (0.02, 0.035));
        // Relink field and Relink (all of the copy's fields) both put it back, each one step.
        fx.copy(1);
        fx.send(Message::Relink(CANT));
        assert_eq!(fx.cant(1), 0.02);
        let _ = fx.app.update(AppMessage::Undo);
        assert_eq!(fx.cant(1), 0.035);
        fx.send(Message::RelinkAll);
        assert!(!fx.rocket().radial(fx.fins).unwrap().has_overrides());
    }

    #[test]
    fn the_unlink_menu_closes_on_anything_else_and_a_single_fin_has_nothing_to_unlink() {
        let mut fx = Fx::new();
        fx.send(Message::FieldMenu(CANT));
        assert!(fx.state.field_menu.is_some());
        fx.send(Message::FieldMenu(CANT));
        assert!(fx.state.field_menu.is_none(), "a second right-click closes it");
        fx.send(Message::FieldMenu(CANT));
        fx.send(Message::NextInstance);
        assert!(fx.state.field_menu.is_none());
        // A free fin has no group: unlinking does nothing.
        fx.send(Message::Detach);
        let free = fx.app.ctx().selection.part().unwrap();
        assert_ne!(free, fx.fins);
        fx.send(Message::Unlink(CANT));
        assert!(!fx.rocket().radial(free).unwrap().has_overrides());
    }

    #[test]
    fn changing_the_count_with_overrides_asks_first() {
        let mut fx = Fx::new();
        fx.copy(2);
        fx.send(Message::Unlink(Field::Dim(Dim::Span)));
        fx.views();
        fx.send(Message::Count(4));
        // Nothing changed yet: the bar asks.
        assert_eq!(fx.rocket().radial(fx.fins).unwrap().count, 3);
        assert_eq!(fx.state.confirm_count, Some((fx.fins, 4)));
        fx.views();
        fx.send(Message::CancelCount);
        assert_eq!((fx.state.confirm_count, fx.rocket().radial(fx.fins).unwrap().count), (None, 3));
        assert!(fx.rocket().radial(fx.fins).unwrap().has_overrides());
        fx.send(Message::Count(4));
        fx.send(Message::ConfirmCount);
        let group = fx.rocket().radial(fx.fins).unwrap();
        assert_eq!((group.count, group.has_overrides()), (4, false));
        // The same count again is not a change: no question. And a group with no overrides never asks.
        fx.send(Message::Count(4));
        assert_eq!(fx.state.confirm_count, None);
        fx.send(Message::Count(3));
        assert_eq!((fx.state.confirm_count, fx.rocket().radial(fx.fins).unwrap().count), (None, 3));
        // The count change is one undo step.
        let _ = fx.app.update(AppMessage::Undo);
        assert_eq!(fx.rocket().radial(fx.fins).unwrap().count, 4);
    }

    #[test]
    fn detach_keeps_the_effective_values_and_the_rest_stay_linked() {
        let mut fx = Fx::new();
        fx.copy(1);
        fx.send(Message::Unlink(CANT));
        fx.send(Message::Edit(CANT, 0.03, false));
        fx.send(Message::Detach);
        let free = fx.app.ctx().selection.part().unwrap();
        let r = fx.rocket();
        assert_eq!((r.radial(free).unwrap().cant, r.radial(free).unwrap().count), (0.03, 1));
        let group = r.radial(fx.fins).unwrap();
        assert_eq!((group.count, group.has_overrides()), (2, false));
        // The bar of the free fin offers to rejoin; with the only candidate the choice opens straight away.
        fx.views();
        fx.send(Message::RejoinStart);
        assert_eq!(fx.app.ctx().symmetry.rejoin.map(|j| (j.part, j.group)), Some((free, fx.fins)));
        fx.views();
        // The part's settings win: the group takes its cant and is three fins again; one undo step splits it back.
        fx.send(Message::RejoinChoose(Winner::Part));
        let r = fx.rocket();
        assert!(r.radial(free).is_none());
        assert_eq!((r.radial(fx.fins).unwrap().count, r.radial(fx.fins).unwrap().cant), (3, 0.03));
        assert_eq!(fx.app.ctx().selection, Selection::Part(fx.fins));
        assert_eq!(fx.app.ctx().symmetry.rejoin, None);
        let _ = fx.app.update(AppMessage::Undo);
        assert_eq!(fx.rocket().radial(free).unwrap().count, 1);
        assert_eq!(fx.rocket().radial(fx.fins).unwrap().count, 2);
    }

    #[test]
    fn rejoin_can_be_cancelled_or_the_group_can_win() {
        let mut fx = Fx::new();
        fx.send(Message::Detach);
        let free = fx.app.ctx().selection.part().unwrap();
        fx.send(Message::RejoinStart);
        fx.send(Message::RejoinCancel);
        assert_eq!(fx.app.ctx().symmetry.rejoin, None);
        // Dropping on a group's bar in the lanes sets the same pending choice.
        fx.app.ctx_mut().symmetry.rejoin = Some(crate::app::symmetry::Rejoin { part: free, group: fx.fins });
        fx.views();
        fx.send(Message::RejoinChoose(Winner::Group));
        assert_eq!((fx.rocket().radial(fx.fins).unwrap().count, fx.rocket().radial(free).is_none()), (3, true));
        // Rejoining something that is not a free fin of the same family is refused and leaves no choice pending.
        fx.send(Message::RejoinChoose(Winner::Group));
        assert_eq!(fx.app.ctx().symmetry.rejoin, None);
    }

    #[test]
    fn a_pending_rejoin_is_forgotten_when_its_parts_go_away() {
        let mut fx = Fx::new();
        fx.send(Message::Detach);
        let free = fx.app.ctx().selection.part().unwrap();
        fx.send(Message::RejoinStart);
        assert!(fx.app.ctx().symmetry.rejoin.is_some());
        // Undo the detach: the free part is gone, so the pending rejoin is dropped by the app's housekeeping.
        let _ = fx.app.update(AppMessage::Undo);
        assert!(fx.rocket().radial(free).is_none());
        assert_eq!(fx.app.ctx().symmetry.rejoin, None);
    }

    #[test]
    fn every_symmetry_state_builds_a_view() {
        let mut fx = Fx::new();
        fx.views();
        fx.copy(1);
        fx.send(Message::Unlink(CANT));
        fx.send(Message::Unlink(Field::Dim(Dim::Span)));
        fx.send(Message::Unlink(Field::Dim(Dim::TipChord)));
        fx.send(Message::FieldMenu(Field::Dim(Dim::Sweep)));
        fx.views();
        fx.send(Message::Count(2));
        fx.views();
        fx.send(Message::SplitAll);
        let ids: Vec<PartId> = fx.rocket().radials().iter().map(|r| r.id).collect();
        for id in ids {
            fx.app.ctx_mut().selection = Selection::Part(id);
            fx.views();
        }
    }
}
