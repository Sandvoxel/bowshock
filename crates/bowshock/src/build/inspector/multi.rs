//! The inspector of a multi-selection (design/components/Inspector: *Multi-select shows only the fields the selected parts share.
//! Mixed values display as `—` and typing sets all of them*).
//!
//! The header is `3 parts` with the parts' silhouettes. Only fields every selected part has are shown: a [`Dim`] (the same
//! dimension of the model, the one `Rocket::dim` answers for each part), the material (when the parts take the same kind of
//! material), the finish, the mass override. A field whose parts agree shows the value; one where they differ is empty with the
//! faint `—` placeholder (`UnitInput::mixed`, a select with nothing chosen). Typing or choosing sets the value on every part as
//! one undo step. Name, symmetry, deployment and the other single-part sections are left out.

use bowshock_model::{materials_of_kind, Dim, Finish, MaterialId, MaterialKind, PartId, PartRef, Rocket};
use iced::widget::Id;
use iced::widget::{container, row, text, Space};
use iced::{Element, Fill, Widget};

use super::edit::Field;
use super::form::{choose, value_row};
use super::part::{finish_label, material_label};
use super::silhouette;
use super::{Message, HEADER_H};
use crate::app::{refs, ui, Ctx};
use crate::theme::{self, MONO, SANS_SEMIBOLD};
use crate::tokens::{text as type_scale, SPACE_2};
use crate::units::{Quantity, UnitSystem};
use crate::widgets::button::{button, icon_button, Weight};
use crate::widgets::field::{labeled, section, select};
use crate::widgets::glyph::Mark;
use crate::widgets::unit_input::{unit_input, State as InputState};

/// How many silhouettes the header draws before `+N`.
const MAX_SILHOUETTES: usize = 4;

/// The geometry dimensions a multi-selection can edit, in the order the sections list them, with the label the field shows.
/// Position-like ones are listed apart ([`POSITION_DIMS`]).
const GEOMETRY_DIMS: [(Dim, &str); 13] = [
    (Dim::Length, "Length"),
    (Dim::Diameter, "Diameter"),
    (Dim::ForeDiameter, "Fore Ø"),
    (Dim::Wall, "Wall"),
    (Dim::Shape, "Shape"),
    (Dim::RootChord, "Root chord"),
    (Dim::TipChord, "Tip chord"),
    (Dim::Span, "Height"),
    (Dim::Height, "Height"),
    (Dim::Sweep, "Sweep length"),
    (Dim::Thickness, "Thickness"),
    (Dim::Overhang, "Overhang"),
    (Dim::PackedLength, "Packed length"),
];

/// The placement dimensions of radial and internal parts (an axial part has none, so they drop out of a mixed selection).
const POSITION_DIMS: [(Dim, &str); 4] = [(Dim::Position, "Offset"), (Dim::Angle, "Rotation"), (Dim::Cant, "Cant"), (Dim::FilletRadius, "Fillet radius")];

/// A value over several parts: the one they all have, or that they differ.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Shared<T> {
    /// Every part has this value.
    Same(T),
    /// The parts disagree.
    Mixed,
}

impl<T: Copy> Shared<T> {
    /// The common value, if there is one.
    pub fn value(self) -> Option<T> {
        match self {
            Shared::Same(v) => Some(v),
            Shared::Mixed => None,
        }
    }
}

/// Folds `values` into a [`Shared`] (`None` for no values at all), comparing with `same`.
fn fold<T: Copy>(values: impl IntoIterator<Item = T>, same: impl Fn(T, T) -> bool) -> Option<Shared<T>> {
    let mut it = values.into_iter();
    let first = it.next()?;
    Some(if it.all(|v| same(first, v)) { Shared::Same(first) } else { Shared::Mixed })
}

/// Two SI values that differ by less than a nanometre-ish relative tolerance are the same number.
fn close(a: f64, b: f64) -> bool {
    (a - b).abs() <= 1e-9 * a.abs().max(b.abs()).max(1.0)
}

/// One numeric dimension shared by every selected part.
#[derive(Debug, Clone, PartialEq)]
pub struct SharedDim {
    /// The dimension, edited with `Rocket::set_dim` on each part.
    pub dim: Dim,
    /// The label of the field.
    pub label: &'static str,
    /// What the value measures.
    pub quantity: Quantity,
    /// The value they share, or mixed.
    pub value: Shared<f64>,
}

/// The dimensions every part of `ids` has (the same [`Dim`], so `Rocket::dim` answers for all of them), among `candidates`,
/// with the value they share or [`Shared::Mixed`].
pub fn shared_dims(rocket: &Rocket, ids: &[PartId], candidates: &[(Dim, &'static str)]) -> Vec<SharedDim> {
    let mut out: Vec<SharedDim> = Vec::new();
    for &(dim, label) in candidates {
        let values: Option<Vec<f64>> = ids.iter().map(|id| rocket.dim(*id, dim)).collect();
        let Some(value) = values.and_then(|v| fold(v, close)) else { continue };
        // Height is two dimensions (a fin's span, a rail button's height) with one name: the first that applies is the field.
        if out.iter().any(|f| f.label == label) {
            continue;
        }
        out.push(SharedDim { dim, label, quantity: refs::dim_quantity(dim), value });
    }
    out
}

/// The material every part takes from the same list (bulk stock, fabric, cord), with the one they share, or `None` when the
/// parts take different kinds of material (or one has none).
pub fn shared_material(rocket: &Rocket, ids: &[PartId]) -> Option<(MaterialKind, Shared<MaterialId>)> {
    let each: Option<Vec<(MaterialKind, MaterialId)>> = ids
        .iter()
        .map(|id| match rocket.kind_of(*id)? {
            PartRef::Axial(p) => Some((MaterialKind::Bulk, p.material)),
            PartRef::Radial(r) => Some((MaterialKind::Bulk, r.material)),
            PartRef::Internal(i) => Some((i.kind.material_kind()?, i.material)),
        })
        .collect();
    let each = each?;
    let kind = each.first()?.0;
    if each.iter().any(|(k, _)| *k != kind) {
        return None;
    }
    Some((kind, fold(each.iter().map(|(_, m)| *m), |a, b| a == b)?))
}

/// The finish every part has (`None` when one has no finish: internal parts are inside), with the one they share.
pub fn shared_finish(rocket: &Rocket, ids: &[PartId]) -> Option<Shared<Finish>> {
    let each: Option<Vec<Finish>> = ids
        .iter()
        .map(|id| match rocket.kind_of(*id)? {
            PartRef::Axial(p) => Some(p.finish),
            PartRef::Radial(r) => Some(r.finish),
            PartRef::Internal(_) => None,
        })
        .collect();
    fold(each?, |a, b| a == b)
}

/// The mass of every selected part and whether each one's is an override.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MassRow {
    /// The mass they share (a radial group counts all its copies), or mixed.
    pub mass: Shared<f64>,
    /// Every part has an explicit mass.
    pub all_overridden: bool,
    /// At least one part has an explicit mass.
    pub any_overridden: bool,
    /// The sum over the parts.
    pub total: f64,
}

/// The mass section's values for `ids`.
pub fn mass_row(rocket: &Rocket, ids: &[PartId]) -> Option<MassRow> {
    let masses: Vec<f64> = ids.iter().filter_map(|id| rocket.component_mass(*id)).map(|m| m.mass).collect();
    let overridden = |id: &PartId| match rocket.kind_of(*id) {
        Some(PartRef::Axial(p)) => p.mass_override.is_some(),
        Some(PartRef::Radial(r)) => r.mass_override.is_some(),
        Some(PartRef::Internal(i)) => i.mass_override.is_some(),
        None => false,
    };
    Some(MassRow {
        mass: fold(masses.iter().copied(), close)?,
        all_overridden: ids.iter().all(overridden),
        any_overridden: ids.iter().any(overridden),
        total: masses.iter().sum(),
    })
}

/// Whether a message edits the whole multi-selection (see [`update`]).
pub fn handles(message: &Message) -> bool {
    matches!(
        message,
        Message::Edit(Field::Dim(_) | Field::MassOverride, ..)
            | Message::Material(_)
            | Message::Finish(_)
            | Message::ClearOverride(Field::MassOverride)
            | Message::StartOverride(Field::MassOverride)
    )
}

/// Applies an edit of a multi-selection to every selected part: one undo step (a scrub is one merged step).
pub fn update(ctx: &mut Ctx, message: Message) {
    let ids = ctx.selected_parts();
    let n = ids.len();
    match message {
        Message::Edit(Field::Dim(dim), value, live) => {
            let key = format!("multi:{dim:?}");
            let label = format!("Set {} of {n} parts", dim.label().to_lowercase());
            let apply = |r: &mut Rocket| {
                for id in &ids {
                    r.clear_formula(*id, dim);
                    r.set_dim(*id, dim, value);
                }
            };
            if live {
                ctx.doc.edit_merging(&label, &key, apply);
            } else {
                ctx.doc.edit(&label, apply);
            }
        }
        Message::Edit(Field::MassOverride, value, live) => {
            let apply = |r: &mut Rocket| {
                ids.iter().for_each(|id| {
                    r.set_mass_override(*id, Some(value));
                })
            };
            if live {
                ctx.doc.edit_merging(&format!("Override mass of {n} parts"), "multi:mass", apply);
            } else {
                ctx.doc.edit(&format!("Override mass of {n} parts"), apply);
            }
        }
        Message::Material(material) => {
            ctx.doc.edit(&format!("Set material of {n} parts"), |r| {
                ids.iter().for_each(|id| {
                    r.set_material(*id, material);
                })
            });
        }
        Message::Finish(finish) => {
            ctx.doc.edit(&format!("Set finish of {n} parts"), |r| {
                ids.iter().for_each(|id| {
                    r.set_finish(*id, finish);
                })
            });
        }
        Message::StartOverride(_) => {
            // Each part starts from its own computed mass.
            let masses: Vec<(PartId, f64)> = ids.iter().filter_map(|id| Some((*id, ctx.doc.rocket().component_mass(*id)?.mass))).collect();
            ctx.doc.edit(&format!("Override mass of {n} parts"), |r| {
                masses.iter().for_each(|(id, m)| {
                    r.set_mass_override(*id, Some(*m));
                })
            });
        }
        Message::ClearOverride(_) => {
            ctx.doc.edit(&format!("Clear mass override of {n} parts"), |r| {
                ids.iter().for_each(|id| {
                    r.set_mass_override(*id, None);
                })
            });
        }
        _ => {}
    }
}

/// A numeric field over the selection: the value (or the `—` placeholder), edited live while scrubbing and committed on Enter.
fn field<'a>(units: &UnitSystem, label: &'a str, value: Shared<f64>, quantity: Quantity, f: Field) -> Element<'a, Message> {
    let v = value.value();
    unit_input(label, v.unwrap_or(0.0), quantity, units)
        .id(Id::from(format!("multi:{}", f.key())))
        .mixed(v.is_none())
        .on_scrub(move |x| Message::Edit(f, x, true))
        .on_commit(move |x| Message::Edit(f, x, false))
        .into()
}

/// The header, and the sections of the shared fields.
pub fn view<'a>(ctx: &'a Ctx, menu_open: bool) -> (Element<'a, Message>, Vec<Element<'a, Message>>) {
    let ids = ctx.selected_parts();
    (header(ctx, &ids, menu_open), sections(ctx, &ids))
}

fn header<'a>(ctx: &'a Ctx, ids: &[PartId], menu_open: bool) -> Element<'a, Message> {
    let rocket = ctx.doc.rocket();
    let mut cells: Vec<Element<'a, Message>> = ids.iter().take(MAX_SILHOUETTES).map(|id| silhouette::silhouette(rocket, *id)).collect();
    if ids.len() > MAX_SILHOUETTES {
        cells.push(text(format!("+{}", ids.len() - MAX_SILHOUETTES)).font(MONO).size(10.5).style(theme::faint).boxed());
    }
    let silhouettes = row(cells).spacing(2).align_y(iced::Center);
    let title = text(format!("{} parts", ids.len())).font(SANS_SEMIBOLD).size(type_scale::UI_TITLE.size).style(theme::ink);
    let more: Element<'a, Message> = icon_button(Mark::Ellipsis, "More").on_press(Message::MenuOpen(!menu_open)).into();
    container(row![silhouettes, title, Space::new().width(Fill), more].spacing(SPACE_2).align_y(iced::Center))
        .padding([0.0, SPACE_2 + 2.0])
        .height(HEADER_H)
        .center_y(HEADER_H)
        .boxed()
}

fn sections<'a>(ctx: &'a Ctx, ids: &[PartId]) -> Vec<Element<'a, Message>> {
    let rocket = ctx.doc.rocket();
    let units = &ctx.units;
    let mut out: Vec<Element<'a, Message>> = Vec::new();

    let geometry: Vec<Element<'a, Message>> =
        shared_dims(rocket, ids, &GEOMETRY_DIMS).into_iter().map(|f| field(units, f.label, f.value, f.quantity, Field::Dim(f.dim))).collect();
    out.push(section(1, "Geometry", false, if geometry.is_empty() { vec![none_shared()] } else { geometry }));

    let position: Vec<Element<'a, Message>> =
        shared_dims(rocket, ids, &POSITION_DIMS).into_iter().map(|f| field(units, f.label, f.value, f.quantity, Field::Dim(f.dim))).collect();
    if !position.is_empty() {
        out.push(section(2, "Position", true, position));
    }

    let mut material: Vec<Element<'a, Message>> = Vec::new();
    if let Some((kind, shared)) = shared_material(rocket, ids) {
        let options: Vec<(MaterialId, String)> = materials_of_kind(kind).map(|m| (m.id, material_label(m))).collect();
        material.push(labeled("Material", select_of(options, shared.value(), Message::Material)));
    }
    if let Some(shared) = shared_finish(rocket, ids) {
        let options: Vec<(Finish, String)> = Finish::ALL.iter().map(|f| (*f, finish_label(*f))).collect();
        material.push(labeled("Finish", select_of(options, shared.value(), Message::Finish)));
    }
    if !material.is_empty() {
        out.push(section(3, "Material & finish", true, material));
    }

    if let Some(row) = mass_row(rocket, ids) {
        let mut mass: Vec<Element<'a, Message>> = Vec::new();
        let total = ui::with_unit(row.total, Quantity::Mass, ctx.units);
        mass.push(value_row("Total", total, None));
        if row.any_overridden {
            let input = unit_input("Mass each", row.mass.value().unwrap_or(0.0), Quantity::Mass, units)
                .id(Id::from(format!("multi:{}", Field::MassOverride.key())))
                .mixed(row.mass.value().is_none())
                .on_scrub(|x| Message::Edit(Field::MassOverride, x, true))
                .on_commit(|x| Message::Edit(Field::MassOverride, x, false));
            let input = if row.all_overridden { input.state(InputState::Override) } else { input };
            mass.push(input.into());
            mass.push(labeled("", button("Clear overrides", Weight::Ghost).on_press(Message::ClearOverride(Field::MassOverride)).into()));
        } else {
            mass.push(labeled("Override", button("Override all…", Weight::Ghost).on_press(Message::StartOverride(Field::MassOverride)).into()));
        }
        out.push(section(4, "Mass", true, mass));
    }
    out
}

fn none_shared<'a>() -> Element<'a, Message> {
    container(text("The parts share no editable field.").size(12).style(theme::muted)).padding([SPACE_2, 0.0]).boxed()
}

/// A select with the shared choice (or nothing chosen, shown as `—`).
fn select_of<'a, T>(options: Vec<(T, String)>, shared: Option<T>, on: impl Fn(T) -> Message + 'a) -> Element<'a, Message>
where
    T: Clone + PartialEq + 'a,
{
    match shared {
        Some(selected) => choose(options, &selected, on),
        None => {
            let choices: Vec<super::form::Choice<T>> = options.into_iter().map(|(value, label)| super::form::Choice { value, label }).collect();
            select(choices, None, move |c| on(c.value))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::app::{App, Message as AppMessage, Selection};
    use crate::build::inspector::State;

    /// The example app with the nose (index 0) and the first body tube (index 1) selected, and their diameters equal.
    fn app() -> (App, State, Vec<PartId>) {
        let mut app = App::with_example();
        let ids: Vec<PartId> = vec![app.ctx().doc.rocket().parts()[0].id, app.ctx().doc.rocket().parts()[1].id];
        let d = app.ctx().doc.rocket().dim(ids[1], Dim::Diameter).unwrap();
        app.ctx_mut().doc.edit("Match", |r| {
            r.set_dim(ids[0], Dim::Diameter, d);
        });
        app.ctx_mut().select_parts(&ids);
        app.sync();
        (app, State::new(), ids)
    }

    fn send(app: &mut App, state: &mut State, message: Message) {
        let _ = state.update(message, app.ctx_mut());
        app.sync();
    }

    #[test]
    fn only_the_fields_every_part_has_are_shown() {
        let (app, _, ids) = app();
        let rocket = app.ctx().doc.rocket();
        let dims = shared_dims(rocket, &ids, &GEOMETRY_DIMS);
        let names: Vec<&str> = dims.iter().map(|f| f.label).collect();
        // A nose cone and a tube share length, diameter and wall; the nose's shape and a tube's missing shape do not.
        assert_eq!(names, ["Length", "Diameter", "Wall"]);
        // A fin set adds nothing in common with an axial part.
        let fins = rocket.radials().iter().find(|r| r.kind.is_fin()).map(|r| r.id).unwrap();
        assert!(shared_dims(rocket, &[ids[1], fins], &GEOMETRY_DIMS).is_empty());
        assert!(shared_dims(rocket, &[ids[1], fins], &POSITION_DIMS).is_empty());
    }

    #[test]
    fn equal_values_show_and_differing_values_are_mixed() {
        let (app, _, ids) = app();
        let rocket = app.ctx().doc.rocket();
        let dims = shared_dims(rocket, &ids, &GEOMETRY_DIMS);
        let by = |label: &str| dims.iter().find(|f| f.label == label).unwrap().value;
        assert!(matches!(by("Diameter"), Shared::Same(_)), "the diameters were made equal");
        assert_eq!(by("Length"), Shared::Mixed, "a nose and a tube differ in length");
        // The widget shows the `—` placeholder (an empty buffer) for a mixed value and the number otherwise.
        assert_eq!(by("Length").value(), None);
    }

    #[test]
    fn typing_a_value_sets_every_part_in_one_undo_step() {
        let (mut app, mut state, ids) = app();
        let before: Vec<f64> = ids.iter().map(|id| app.ctx().doc.rocket().dim(*id, Dim::Wall).unwrap()).collect();
        send(&mut app, &mut state, Message::Edit(Field::Dim(Dim::Wall), 0.0025, false));
        for id in &ids {
            assert_eq!(app.ctx().doc.rocket().dim(*id, Dim::Wall), Some(0.0025));
        }
        assert_eq!(app.ctx().selected_parts(), ids, "the selection survives the edit");
        let _ = app.update(AppMessage::Undo);
        let after: Vec<f64> = ids.iter().map(|id| app.ctx().doc.rocket().dim(*id, Dim::Wall).unwrap()).collect();
        assert_eq!(after, before, "one undo restores both");
        assert_eq!(app.ctx().selected_parts(), ids);
        let _ = app.update(AppMessage::Redo);
        assert_eq!(app.ctx().doc.rocket().dim(ids[0], Dim::Wall), Some(0.0025));
        assert_eq!(app.ctx().doc.rocket().dim(ids[1], Dim::Wall), Some(0.0025));
    }

    #[test]
    fn a_scrub_across_the_selection_is_one_step() {
        let (mut app, mut state, ids) = app();
        let before: Vec<f64> = ids.iter().map(|id| app.ctx().doc.rocket().dim(*id, Dim::Wall).unwrap()).collect();
        for w in [0.0021, 0.0022, 0.0024] {
            send(&mut app, &mut state, Message::Edit(Field::Dim(Dim::Wall), w, true));
        }
        send(&mut app, &mut state, Message::Edit(Field::Dim(Dim::Wall), 0.0024, false));
        let _ = app.update(AppMessage::Undo);
        let after: Vec<f64> = ids.iter().map(|id| app.ctx().doc.rocket().dim(*id, Dim::Wall).unwrap()).collect();
        assert_eq!(after, before);
    }

    #[test]
    fn material_and_finish_apply_to_all() {
        let (mut app, mut state, ids) = app();
        send(&mut app, &mut state, Message::Material(MaterialId::FIBERGLASS));
        send(&mut app, &mut state, Message::Finish(Finish::ALL[1]));
        let rocket = app.ctx().doc.rocket();
        assert_eq!(shared_material(rocket, &ids), Some((MaterialKind::Bulk, Shared::Same(MaterialId::FIBERGLASS))));
        assert_eq!(shared_finish(rocket, &ids), Some(Shared::Same(Finish::ALL[1])));
        let _ = app.update(AppMessage::Undo);
        let rocket = app.ctx().doc.rocket();
        assert_eq!(shared_finish(rocket, &ids).map(Shared::value), Some(Some(rocket.part(ids[0]).unwrap().finish)));
        assert_ne!(shared_finish(rocket, &ids), Some(Shared::Same(Finish::ALL[1])), "the finish step is undone on its own");
        // Different materials read as mixed.
        let mut r = rocket.clone();
        r.set_material(ids[0], MaterialId::BALSA);
        r.set_material(ids[1], MaterialId::CARDBOARD);
        assert_eq!(shared_material(&r, &ids), Some((MaterialKind::Bulk, Shared::Mixed)));
    }

    #[test]
    fn the_mass_override_applies_to_all_and_totals_sum() {
        let (mut app, mut state, ids) = app();
        let rocket = app.ctx().doc.rocket();
        let row = mass_row(rocket, &ids).unwrap();
        let sum: f64 = ids.iter().map(|id| rocket.component_mass(*id).unwrap().mass).sum();
        assert!((row.total - sum).abs() < 1e-12);
        assert!(!row.any_overridden);
        send(&mut app, &mut state, Message::Edit(Field::MassOverride, 0.05, false));
        let row = mass_row(app.ctx().doc.rocket(), &ids).unwrap();
        assert!(row.all_overridden);
        assert_eq!(row.mass, Shared::Same(0.05));
        send(&mut app, &mut state, Message::ClearOverride(Field::MassOverride));
        assert!(!mass_row(app.ctx().doc.rocket(), &ids).unwrap().any_overridden);
    }

    #[test]
    fn the_view_builds_for_a_multi_selection() {
        let (app, state, _) = app();
        assert!(app.ctx().is_multi() && matches!(app.ctx().selection, Selection::Part(_)));
        let _ = state.view(app.ctx());
    }
}
