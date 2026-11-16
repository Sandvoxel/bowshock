//! What the inspector does with references between fields (design/components/UnitInput, *References*): pick mode, committing a
//! formula, the *Replace formula with a value?* question, converting a broken formula, and the `→ N` dependants list. The
//! formulas themselves are the document's (`bowshock_model::Formula`); the transient state (pick mode, the pending insert, the
//! error under a field) is `ctx.refs`.

use bowshock_model::{FieldRef, FormulaError};
use iced::Task;

use super::Message;
use crate::app::refs::{self, dim_quantity};
use crate::app::{self, Ctx, Selection};
use crate::units::{self, Unit};

/// Whether `message` is one of the reference messages this module handles.
pub fn handles(message: &Message) -> bool {
    matches!(
        message,
        Message::PickMode(_)
            | Message::PickField(_)
            | Message::PickMenuClose
            | Message::Formula(..)
            | Message::ScrubFormula(_)
            | Message::ReplaceFormula(..)
            | Message::ConvertFormula(_)
            | Message::Dependants(_)
            | Message::Jump(_)
    )
}

/// Handles a reference message.
pub fn update(message: Message, ctx: &mut Ctx) -> Task<app::Message> {
    // Nothing edits the rocket while a drag has it open.
    let editing = !ctx.doc.dragging() && !ctx.drag.active();
    match message {
        Message::PickMode(Some(target)) => ctx.refs.start(target),
        Message::PickMode(None) => ctx.refs.end(),
        Message::PickField(reference) => ctx.refs.pick(ctx.doc.rocket(), reference),
        Message::PickMenuClose => ctx.refs.close_menu(),
        Message::Formula(target, text, unit) if editing => commit(ctx, target, &text, unit),
        Message::ScrubFormula(target) => ctx.refs.confirm = Some(target),
        Message::ReplaceFormula(target, replace) => {
            ctx.refs.confirm = None;
            if replace && editing {
                ctx.doc.edit("Replace formula with value", |r| r.clear_formula(target.part, target.dim));
            }
        }
        Message::ConvertFormula(target) if editing => {
            ctx.refs.error = None;
            ctx.doc.edit("Convert formula to value", |r| r.clear_formula(target.part, target.dim));
        }
        Message::Dependants(source) => ctx.refs.dependants = (ctx.refs.dependants != Some(source)).then_some(source),
        Message::Jump(target) => {
            ctx.refs.dependants = None;
            ctx.selection = Selection::Part(target.part);
            return Task::done(app::Message::Inspector(Message::Focus(target.dim)));
        }
        _ => {}
    }
    Task::none()
}

/// A formula was typed into `target`: store it, or say why not and keep the last valid value.
fn commit(ctx: &mut Ctx, target: FieldRef, text: &str, unit: &'static Unit) {
    ctx.refs.error = None;
    ctx.refs.confirm = None;
    let quantity = dim_quantity(target.dim);
    let expr = match units::parse_formula(text, quantity, unit) {
        Ok(expr) => expr,
        Err(message) => {
            ctx.refs.error = Some((target, message));
            return;
        }
    };
    let result = ctx.doc.edit("Edit formula", |r| r.set_formula(target.part, target.dim, expr));
    if let Err(error) = result {
        let rocket = ctx.doc.rocket();
        let kept = rocket.dim(target.part, target.dim).map(|v| units::format(v, quantity, &ctx.units).0);
        let kept = kept.map(|v| format!(" The last valid value ({v}) is kept.")).unwrap_or_default();
        let message = match error {
            FormulaError::Circular { target, referenced } => format!("Circular: {}{kept}", refs::circular_message(rocket, target, referenced)),
            FormulaError::NoSuchField(r) => {
                let name = rocket.kind_of(r.part).map(|p| p.name()).unwrap_or_else(|| "That part".to_string());
                format!("Broken reference: {name} has no {}.{kept}", refs::dim_name(rocket, r.part, r.dim).to_lowercase())
            }
            other => format!("{other}.{kept}"),
        };
        ctx.refs.error = Some((target, message));
    }
}

#[cfg(test)]
mod tests {
    use bowshock_model::{Dim, FieldRef, FormulaStatus};

    use super::super::edit::Field;
    use super::super::{Message, State};
    use crate::app::{App, Message as AppMessage, Selection};
    use crate::units::{Quantity, UnitSystem};
    use crate::viewport::pick::Pick;
    use crate::viewport::Message as Viewport;

    /// An app with the example rocket, an inspector state of its own, and the example's fin set selected.
    struct Fx {
        app: App,
        state: State,
        fins: u64,
        tube: u64,
    }

    impl Fx {
        fn new() -> Fx {
            let mut app = App::with_example();
            let fins = app.ctx().doc.rocket().radials().iter().find(|r| r.kind.is_fin()).map(|r| r.id).unwrap();
            let tube = app.ctx().doc.rocket().radial(fins).map(|r| r.parent).unwrap();
            app.ctx_mut().selection = Selection::Part(fins);
            Fx { app, state: State::new(), fins, tube }
        }

        fn send(&mut self, message: Message) {
            let _ = self.state.update(message, self.app.ctx_mut());
            self.app.sync();
        }

        /// Types `text` (millimetres) into `dim` of the fin set, as the field would publish it.
        fn type_formula(&mut self, dim: Dim, text: &str) {
            let mm = Quantity::Length.unit_for(UnitSystem::MM_G_S);
            self.send(Message::Formula(FieldRef::new(self.fins, dim), text.to_string(), mm));
        }

        fn rocket(&self) -> &bowshock_model::Rocket {
            self.app.ctx().doc.rocket()
        }

        fn dim(&self, part: u64, dim: Dim) -> f64 {
            self.rocket().dim(part, dim).unwrap()
        }

        fn error(&self) -> Option<String> {
            self.app.ctx().refs.error.as_ref().map(|(_, m)| m.clone())
        }
    }

    #[test]
    fn a_typed_formula_follows_its_source_and_is_one_undo_step() {
        let mut fx = Fx::new();
        let (fins, tube) = (fx.fins, fx.tube);
        let before = fx.dim(fins, Dim::RootChord);
        fx.type_formula(Dim::RootChord, &format!("2 * {{body_tube#{tube}.od}}"));
        assert_eq!(fx.error(), None);
        assert!((fx.dim(fins, Dim::RootChord) - 2.0 * fx.dim(tube, Dim::Diameter)).abs() < 1e-12);
        // The source side counts its dependants.
        assert_eq!(fx.rocket().dependants(tube, Dim::Diameter), vec![FieldRef::new(fins, Dim::RootChord)]);
        // Change the source through the field the inspector shows: the dependant follows in the same step.
        fx.select(tube);
        fx.send(Message::Edit(Field::Dim(Dim::Diameter), 0.1, false));
        assert!((fx.dim(fins, Dim::RootChord) - 0.2).abs() < 1e-12);
        let _ = fx.app.update(AppMessage::Undo);
        assert!((fx.dim(fins, Dim::RootChord) - 2.0 * 0.0762).abs() < 1e-12, "undo takes the source and the dependant back together");
        let _ = fx.app.update(AppMessage::Undo);
        assert_eq!(fx.dim(fins, Dim::RootChord), before, "undo covers the formula itself");
        assert!(fx.rocket().formulas().is_empty());
        let _ = fx.app.update(AppMessage::Redo);
        assert!(fx.rocket().formula(fins, Dim::RootChord).is_some());
    }

    impl Fx {
        fn select(&mut self, id: u64) {
            self.app.ctx_mut().selection = Selection::Part(id);
        }
    }

    #[test]
    fn a_circular_formula_is_refused_with_the_kits_sentence() {
        let mut fx = Fx::new();
        let fins = fx.fins;
        fx.type_formula(Dim::RootChord, &format!("2 * {{fin_set#{fins}.tip_chord}}"));
        let tip_before = fx.dim(fins, Dim::TipChord);
        fx.type_formula(Dim::TipChord, &format!("0.4 * {{fin_set#{fins}.root_chord}}"));
        let message = fx.error().expect("the cycle is refused");
        assert!(message.starts_with("Circular: Root chord already uses Tip chord."), "{message}");
        assert!(message.contains("The last valid value (50.00) is kept."), "{message}");
        assert_eq!(fx.dim(fins, Dim::TipChord), tip_before);
        assert!(fx.rocket().formula(fins, Dim::TipChord).is_none());
    }

    #[test]
    fn picking_ends_pick_mode_and_hands_the_reference_to_the_target() {
        let mut fx = Fx::new();
        let (fins, tube) = (fx.fins, fx.tube);
        let target = FieldRef::new(fins, Dim::Span);
        fx.send(Message::PickMode(Some(target)));
        assert!(fx.app.ctx().refs.is_target(target));
        fx.send(Message::PickField(FieldRef::new(tube, Dim::Diameter)));
        let ctx = fx.app.ctx();
        assert!(!ctx.refs.is_picking());
        let insert = ctx.refs.insert.as_ref().expect("an insert for the target");
        assert_eq!(insert.target, target);
        assert_eq!(insert.text, format!("{{body_tube#{tube}.od}}"));
        // A second pick is a new insert.
        let first = insert.token;
        fx.send(Message::PickMode(Some(target)));
        fx.send(Message::PickField(FieldRef::new(tube, Dim::Length)));
        assert_ne!(fx.app.ctx().refs.insert.as_ref().unwrap().token, first);
    }

    #[test]
    fn clicking_a_part_while_picking_opens_its_menu_instead_of_selecting_it() {
        let mut fx = Fx::new();
        let (fins, tube) = (fx.fins, fx.tube);
        fx.send(Message::PickMode(Some(FieldRef::new(fins, Dim::Span))));
        fx.app.ctx_mut().cursor = Some(iced::Point::new(300.0, 400.0));
        let _ = fx.app.update(AppMessage::Viewport(Viewport::Select(Some(Pick::Axial(tube)))));
        assert_eq!(fx.app.ctx().selection, Selection::Part(fins), "the pick must not change what the inspector shows");
        let menu = fx.app.ctx().refs.picking.as_ref().and_then(|p| p.menu).expect("the part menu is open");
        assert_eq!(menu.part, tube);
        // Without a pick in progress a click selects as before.
        fx.send(Message::PickMenuClose);
        fx.send(Message::PickMode(None));
        let _ = fx.app.update(AppMessage::Viewport(Viewport::Select(Some(Pick::Axial(tube)))));
        assert_eq!(fx.app.ctx().selection, Selection::Part(tube));
    }

    #[test]
    fn replacing_a_formula_keeps_the_value_and_typing_a_number_replaces_it_in_one_step() {
        let mut fx = Fx::new();
        let (fins, tube) = (fx.fins, fx.tube);
        fx.type_formula(Dim::RootChord, &format!("2 * {{body_tube#{tube}.od}}"));
        let value = fx.dim(fins, Dim::RootChord);
        // Scrubbing asks first; nothing changed yet.
        let root = FieldRef::new(fins, Dim::RootChord);
        fx.send(Message::ScrubFormula(root));
        assert_eq!(fx.app.ctx().refs.confirm, Some(root));
        assert!(fx.rocket().formula(fins, Dim::RootChord).is_some());
        fx.send(Message::ReplaceFormula(root, false));
        assert!(fx.rocket().formula(fins, Dim::RootChord).is_some());
        fx.send(Message::ReplaceFormula(root, true));
        assert!(fx.rocket().formula(fins, Dim::RootChord).is_none());
        assert_eq!(fx.dim(fins, Dim::RootChord), value, "the value stays");
        let _ = fx.app.update(AppMessage::Undo);
        assert!(fx.rocket().formula(fins, Dim::RootChord).is_some(), "undo brings the formula back");
        // Typing a plain number replaces the formula, and one undo brings both back.
        fx.send(Message::Edit(Field::Dim(Dim::RootChord), 0.1, false));
        assert!(fx.rocket().formula(fins, Dim::RootChord).is_none());
        assert!((fx.dim(fins, Dim::RootChord) - 0.1).abs() < 1e-12);
        let _ = fx.app.update(AppMessage::Undo);
        assert!(fx.rocket().formula(fins, Dim::RootChord).is_some());
        assert!((fx.dim(fins, Dim::RootChord) - value).abs() < 1e-12);
    }

    #[test]
    fn a_deleted_source_breaks_the_reference_and_converting_keeps_the_value() {
        let mut fx = Fx::new();
        let fins = fx.fins;
        // The ballast mass component in the example: its length drives the fin height.
        let ballast = fx.rocket().internals().iter().find(|p| matches!(p.kind, bowshock_model::InternalKind::MassComponent { .. })).map(|p| p.id).unwrap();
        let token = format!("ballast#{ballast}.length");
        fx.type_formula(Dim::Span, &format!("0.5 * {{{token}}}"));
        assert_eq!(fx.error(), None);
        let value = fx.dim(fins, Dim::Span);
        fx.app.ctx_mut().selection = Selection::Part(ballast);
        let _ = fx.app.update(AppMessage::Delete);
        assert!(fx.rocket().internal(ballast).is_none());
        assert_eq!(fx.rocket().formula_status(fins, Dim::Span), FormulaStatus::Broken(vec![FieldRef::new(ballast, Dim::Length)]));
        assert_eq!(fx.dim(fins, Dim::Span), value, "the last valid value is kept");
        fx.send(Message::ConvertFormula(FieldRef::new(fins, Dim::Span)));
        assert!(fx.rocket().formula(fins, Dim::Span).is_none());
        assert_eq!(fx.dim(fins, Dim::Span), value);
    }
}
