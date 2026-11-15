//! The inspector: the right column that edits whatever is selected (design/components/Inspector, UnitInput, Symmetry, Staging).
//! A header (part silhouette, name with double-click rename, sub-type select, overflow menu), the symmetry bar of a linked
//! group, then the fixed sections 01 Geometry, 02 Position, 03 Material & finish, 04 Mass (with inline health chips) and
//! 05 Notes. With nothing selected it is the Rocket inspector; with a separation selected, the Staging sections.
//!
//! Module contract (see `ARCHITECTURE.md`): [`State`] is this module's state, [`Message`] its messages; the app calls
//! [`State::update`], [`State::view`] and [`State::on_key`] with the shared [`Ctx`]. Every edit goes through `ctx.doc`: a
//! scrubbed field is one merged undo step, a typed value one step. Geometry is edited with `Rocket::set_dim`, the same call
//! the viewport's handles make, so a field and a handle never disagree.
//!
//! **Focusing a field:** [`Message::Focus`] with a `bowshock_model::Dim` puts keyboard focus in the selected part's field for
//! that dimension (a handle's double click uses it). `Dim::Span` and `Dim::Height` are the one *Height* field.

mod edit;
mod form;
mod formulas;
mod health;
mod multi;
mod part;
mod reveal;
mod rocket_view;
mod separation;
mod silhouette;
mod symmetry;
#[cfg(test)]
mod tests;

use std::mem::discriminant;

use bowshock_model::{Dim, EventKind, FieldRef, FinPlanform, FinSection, Finish, MassRole, MaterialId, NoseShape, PartId, PartRef, RadialField, Rocket};
use iced::widget::{column, container, mouse_area, row, scrollable, stack, text, text_input, Space};
use iced::{keyboard, mouse, Element, Fill, Length, Task, Widget};

use self::edit::{Anchor, StageOn, Trigger};
use self::form::{choose, Form};
use self::part::Params;
use crate::app::symmetry::Rejoin;
use crate::app::{self, Ctx, Selection};
use crate::build::parts_bin::INSPECTOR_FIRST_FIELD;
use crate::theme::{self, MONO, SANS_SEMIBOLD};
use crate::tokens::{text as type_scale, SPACE_2, SPACE_3};
use crate::widgets::button::{button, icon_button, Weight};
use crate::widgets::field::rule;
use crate::widgets::glyph::Mark;

pub use self::edit::Field;

/// The widget id of the rename field.
const RENAME_ID: &str = "inspector-rename";
/// Height of the header.
const HEADER_H: f32 = 44.0;
/// Width of the sub-type select in the header.
const SUBTYPE_W: f32 = 118.0;

/// Which stability threshold a settings field edits.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Band {
    /// Below it the rocket is unstable.
    Marginal,
    /// Above it the rocket is stable.
    Stable,
    /// Above it the rocket is over-stable.
    Over,
}

/// The inspector's messages.
#[derive(Debug, Clone)]
pub enum Message {
    /// Nothing (a select with one option).
    Noop,
    /// A field's value changed, in SI; `true` while scrubbing (one merged undo step), `false` when committed.
    Edit(Field, f64, bool),
    /// Focus the selected part's field for this dimension (a handle's double click).
    Focus(Dim),
    /// A field entered (`Some`) or left (`None`) pick mode: it typed a `{`, or closed or cancelled the reference.
    PickMode(Option<FieldRef>),
    /// A field, or a value in the part menu, was picked: the field in pick mode takes it as a reference chip.
    PickField(FieldRef),
    /// The part menu of a pick was dismissed.
    PickMenuClose,
    /// A formula was committed in a field: the text typed and the unit it was typed in.
    Formula(FieldRef, String, &'static crate::units::Unit),
    /// A scrub was tried on a formula field: ask before it is replaced by a value.
    ScrubFormula(FieldRef),
    /// The answer to *Replace formula with a value?*
    ReplaceFormula(FieldRef, bool),
    /// A formula becomes the value it has (the fix for a broken reference).
    ConvertFormula(FieldRef),
    /// The `→ N` of a source field: list or hide its dependants.
    Dependants(FieldRef),
    /// Select a dependant and focus its field.
    Jump(FieldRef),
    /// Right-click on an explicit value: back to automatic.
    ResetAuto(Field),
    /// The number of copies of a radial group.
    Count(u8),
    /// A fin's plate section.
    CrossSection(FinSection),
    /// A fin's planform (the header's sub-type select).
    Planform(FinPlanform),
    /// A nose cone's shape family (the header's sub-type select).
    NoseShape(NoseShape),
    /// A mass component's role (the header's sub-type select).
    Role(MassRole),
    /// The part's material.
    Material(MaterialId),
    /// The material of a fin group's fillet.
    FilletMaterial(MaterialId),
    /// A parachute's shroud-line material.
    LineMaterial(MaterialId),
    /// The part's surface finish.
    Finish(Finish),
    /// Which edge the offset is measured from.
    Anchor(Anchor),
    /// A recovery device's trigger.
    Trigger(Trigger),
    /// Whether an inner tube holds a motor.
    MotorMount(bool),
    /// Plug the motor of this mount (no ejection charge) in the active configuration.
    Plug(bool),
    /// What a staging event waits for (`(stage, which event, choice)`), in the active configuration.
    StageOn(usize, EventKind, StageOn),
    /// *Override…*: give the mass or CG an explicit value, starting from the computed one.
    StartOverride(Field),
    /// *Clear*: compute the mass or CG again.
    ClearOverride(Field),
    /// Open or close the notes section.
    NotesOpen(bool),
    /// The part's note.
    Note(String),
    /// Double-click on the name.
    StartRename,
    /// The rename field's text.
    RenameInput(String),
    /// Enter in the rename field.
    RenameCommit,
    /// Escape in the rename field.
    RenameCancel,
    /// Open or close the overflow menu.
    MenuOpen(bool),
    /// Overflow menu: duplicate the part.
    Duplicate,
    /// Overflow menu: delete the part.
    Delete,
    /// Symmetry bar: the next copy of the group is the one the fields and *Detach* act on.
    NextInstance,
    /// Symmetry bar: *Detach*, the chosen copy leaves the group.
    Detach,
    /// Symmetry bar: *Split all*.
    SplitAll,
    /// Right-click on a field of a group: open its *Unlink for this fin* / *Relink field* menu.
    FieldMenu(Field),
    /// *Unlink for this fin*: the copy on show gets its own value of this field.
    Unlink(Field),
    /// *Relink field* (or a click on the broken-link icon): the copy follows the group's value of this field again.
    Relink(Field),
    /// Symmetry bar: *Relink*, every field the copy on show had unlinked follows the group again.
    RelinkAll,
    /// The user confirmed changing the count although it drops the group's per-copy overrides.
    ConfirmCount,
    /// The user backed out of that count change.
    CancelCount,
    /// *Rejoin group* on a free part: choose the group (or take the only one).
    RejoinStart,
    /// The group a free part should rejoin was chosen.
    RejoinWith(PartId),
    /// Whose settings win in the pending rejoin.
    RejoinChoose(bowshock_model::Winner),
    /// Back out of the pending rejoin.
    RejoinCancel,
    /// The rocket's name (Rocket inspector).
    RocketName(String),
    /// The rocket's designer (Rocket inspector).
    RocketDesigner(String),
    /// One of the stability thresholds (calibers), from the Rocket inspector's settings.
    Band(Band, f64),
    /// Select the separation of this stage (the stage list of the Rocket inspector).
    SelectSeparation(usize),
    /// Cut this tube at this station (the fix for a stage with no joint).
    SplitTube(PartId, f64),
}

/// The inspector's state: what is being typed or open, not the rocket.
#[derive(Default)]
pub struct State {
    /// The name being edited, and the selection it belongs to.
    renaming: Option<(Selection, String)>,
    /// The selection the overflow menu is open for.
    menu: Option<Selection>,
    /// Whether the notes section is open.
    notes_open: bool,
    /// The anchor the user chose for a part's offset.
    anchor: Option<(PartId, Anchor)>,
    /// The field whose *Unlink for this fin* / *Relink field* menu is open: the group, the copy and the field.
    field_menu: Option<(PartId, usize, Field)>,
    /// A count change the user must confirm because it would drop the group's per-copy overrides.
    confirm_count: Option<(PartId, u8)>,
    /// A free part choosing which group to rejoin.
    rejoin_picking: Option<PartId>,
    /// The dimension that was last limited, and why (shown under its field).
    limit: Option<(PartId, Dim, &'static str)>,
    /// A field is being scrubbed: the flight behind the health chips is not reflown until it ends.
    scrubbing: bool,
    /// The flight the health chips judge against.
    flight: health::FlightCache,
}

impl State {
    /// An empty inspector state.
    pub fn new() -> Self {
        State::default()
    }

    /// The edge the offset of part `id` is measured from: the user's choice, else the bottom for a fin (a fin flush with the end
    /// of its tube reads 0) and the top for everything else.
    fn anchor(&self, rocket: &Rocket, id: PartId) -> Anchor {
        match self.anchor {
            Some((part, anchor)) if part == id => anchor,
            _ if rocket.radial(id).is_some_and(|r| r.kind.is_fin()) => Anchor::Bottom,
            _ => Anchor::Top,
        }
    }

    /// The stage (from 1) the motor mount `mount` belongs to.
    fn stage_of_mount(&self, rocket: &Rocket, mount: PartId) -> usize {
        rocket.stages().iter().find(|s| s.mounts.contains(&mount)).map_or(1, |s| s.index)
    }

    /// Handles one of the inspector's messages. Edits go through `ctx.doc`.
    pub fn update(&mut self, message: Message, ctx: &mut Ctx) -> Task<app::Message> {
        if formulas::handles(&message) {
            return formulas::update(message, ctx);
        }
        // With several parts selected, edits of what they share go to every one; the single-part messages mean nothing.
        if ctx.is_multi() && multi::handles(&message) {
            if !ctx.doc.dragging() && !ctx.drag.active() {
                multi::update(ctx, message);
            }
            return Task::none();
        }
        if !matches!(message, Message::Edit(_, _, true)) {
            self.scrubbing = false;
        }
        // The unlink menu, a pending count change and the rejoin picker belong to the interaction that opened them.
        if !matches!(message, Message::FieldMenu(_)) {
            self.field_menu = None;
        }
        if !matches!(message, Message::Count(_) | Message::ConfirmCount | Message::NextInstance | Message::Noop) {
            self.confirm_count = None;
        }
        if !matches!(message, Message::RejoinStart | Message::RejoinWith(_) | Message::NextInstance | Message::Noop) {
            self.rejoin_picking = None;
        }
        // Nothing edits the rocket while a drag has it open.
        let editing = !ctx.doc.dragging() && !ctx.drag.active();
        match message {
            Message::Noop => {}
            Message::Focus(dim) => {
                let key = match ctx.selection {
                    Selection::Part(id) if Some(Field::Dim(dim)) == part::first_field(ctx.doc.rocket(), id) => INSPECTOR_FIRST_FIELD.to_string(),
                    Selection::Part(id) if dim == Dim::Diameter && part::first_field(ctx.doc.rocket(), id) == Some(Field::AutoDiameter) => {
                        INSPECTOR_FIRST_FIELD.to_string()
                    }
                    _ => edit::focus_key(dim),
                };
                return Task::batch([iced::widget::operation::focus(iced::widget::Id::from(key.clone())), reveal::reveal(key)]);
            }
            Message::StartRename => {
                let name = match ctx.selection {
                    Selection::Part(id) => ctx.doc.rocket().kind_of(id).map(|p| p.name()),
                    Selection::None => Some(ctx.doc.rocket().name.clone()),
                    Selection::Separation(_) => None,
                };
                if let Some(name) = name {
                    self.renaming = Some((ctx.selection, name));
                    return iced::widget::operation::focus(RENAME_ID);
                }
            }
            Message::RenameInput(text) => {
                if let Some((_, buffer)) = &mut self.renaming {
                    *buffer = text;
                }
            }
            Message::RenameCancel => self.renaming = None,
            Message::RenameCommit => {
                if let Some((selection, name)) = self.renaming.take() {
                    let name = name.trim().to_string();
                    match selection {
                        Selection::Part(id) if editing => {
                            let kind_name = ctx.doc.rocket().kind_of(id).map(|p| p.kind_name().to_string());
                            let name = (!name.is_empty() && Some(&name) != kind_name.as_ref()).then_some(name);
                            ctx.doc.edit("Rename part", |r| r.set_name(id, name));
                        }
                        Selection::None if editing => ctx.doc.edit("Rename rocket", |r| r.name = name),
                        _ => {}
                    }
                }
            }
            Message::MenuOpen(open) => self.menu = open.then_some(ctx.selection),
            Message::NotesOpen(open) => self.notes_open = open,
            Message::NextInstance => {
                if let Selection::Part(id) = ctx.selection {
                    let next = ctx.symmetry.copy_in(ctx.doc.rocket(), id) + 1;
                    ctx.symmetry.instance = Some((id, next));
                }
            }
            Message::Anchor(anchor) => {
                if let Selection::Part(id) = ctx.selection {
                    self.anchor = Some((id, anchor));
                }
            }
            Message::SelectSeparation(n) => ctx.selection = Selection::Separation(n),
            Message::Delete => {
                self.menu = None;
                return Task::done(app::Message::Delete);
            }
            _ if !editing => {}
            Message::Edit(field, value, live) => self.edit_field(ctx, field, value, live),
            Message::RocketName(name) => {
                ctx.doc.edit_merging("Rename rocket", "rocket:name", |r| r.name = name);
            }
            Message::RocketDesigner(designer) => {
                ctx.doc.edit_merging("Edit designer", "rocket:designer", |r| r.designer = designer);
            }
            Message::Band(band, value) => {
                ctx.doc.edit_merging("Edit stability limit", "rocket:bands", |r| {
                    let mut bands = r.stability_bands();
                    match band {
                        Band::Marginal => bands.marginal = value,
                        Band::Stable => bands.stable = value,
                        Band::Over => bands.over = value,
                    }
                    r.set_stability_bands(bands);
                });
            }
            Message::SplitTube(tube, station) => {
                if ctx.doc.edit("Split tube", |r| r.split_tube(tube, station)).is_err() {
                    ctx.notify("Could not split the tube there");
                }
            }
            Message::StageOn(stage, kind, on) => {
                ctx.doc.edit("Set staging trigger", |r| edit::set_stage_on(r, stage, kind, on));
            }
            Message::Note(note) => {
                if let Selection::Part(id) = ctx.selection {
                    ctx.doc.edit_merging("Edit note", &format!("note:{id}"), |r| r.set_note(id, &note));
                }
            }
            message => {
                if let Selection::Part(id) = ctx.selection {
                    self.edit_part(ctx, id, message);
                }
            }
        }
        Task::none()
    }

    /// A numeric field changed.
    fn edit_field(&mut self, ctx: &mut Ctx, field: Field, value: f64, live: bool) {
        self.scrubbing = live;
        let label = field.undo_label();
        match ctx.selection {
            Selection::Part(id) => {
                let mut value = value;
                if field == Field::Dim(Dim::Position) {
                    if let Some((_, parent_length, extent, _)) = part::placement(ctx.doc.rocket(), id) {
                        value = self.anchor(ctx.doc.rocket(), id).offset(value, extent, parent_length);
                    }
                }
                let copy = ctx.symmetry.copy_in(ctx.doc.rocket(), id);
                let key = format!("inspect:{id}:{copy}:{field:?}");
                // Typing a plain number into a formula field replaces the formula, in the same undo step.
                let apply = |r: &mut Rocket| {
                    if let Field::Dim(dim) = field {
                        r.clear_formula(id, dim);
                    }
                    edit::apply_copy(r, id, copy, field, value)
                };
                ctx.refs.error = None;
                let applied = if live { ctx.doc.edit_merging(&label, &key, apply) } else { ctx.doc.edit(&label, apply) };
                self.limit = match (field, applied.limited) {
                    (Field::Dim(dim), Some(why)) => Some((id, dim, why)),
                    _ => None,
                };
            }
            Selection::Separation(_) => {
                // Staging timing is stored per configuration; the field names its stage.
                let key = format!("inspect:separation:{field:?}");
                let apply = |r: &mut Rocket| edit::apply(r, 0, field, value);
                if live {
                    ctx.doc.edit_merging(&label, &key, apply);
                } else {
                    ctx.doc.edit(&label, apply);
                }
            }
            Selection::None => {}
        }
    }

    /// Everything that edits the selected part with a single call.
    fn edit_part(&mut self, ctx: &mut Ctx, id: PartId, message: Message) {
        let rocket = ctx.doc.rocket();
        let instance = ctx.symmetry.copy_in(rocket, id);
        match message {
            Message::ResetAuto(Field::Dim(Dim::InnerDiameter)) => {
                ctx.doc.edit("Auto inner diameter", |r| edit::reset_inner_diameter(r, id));
            }
            Message::ResetAuto(Field::Dim(d @ (Dim::ShoulderDiameter | Dim::ForeShoulderDiameter))) => {
                ctx.doc.edit("Auto shoulder diameter", |r| r.reset_shoulder_diameter(id, d == Dim::ShoulderDiameter));
            }
            Message::ResetAuto(_) => {
                ctx.doc.edit("Auto diameter", |r| edit::set_auto_diameter(r, id, None));
            }
            Message::Count(count) => {
                // Re-spacing a group loses its per-copy overrides: ask before throwing them away.
                if rocket.radial(id).is_some_and(|r| r.count != count && r.has_overrides()) {
                    self.confirm_count = Some((id, count));
                } else {
                    ctx.doc.edit("Set fin count", |r| edit::set_count(r, id, count));
                }
            }
            Message::CrossSection(section) => {
                ctx.doc.edit("Set cross-section", |r| r.set_fin_section(id, section));
            }
            Message::Planform(planform) => {
                ctx.doc.edit("Set planform", |r| edit::set_planform(r, id, planform));
            }
            Message::NoseShape(shape) => {
                ctx.doc.edit("Set nose shape", |r| edit::set_nose_shape(r, id, shape));
            }
            Message::Role(role) => {
                ctx.doc.edit("Set role", |r| edit::set_role(r, id, role));
            }
            Message::Material(material) => {
                ctx.doc.edit("Set material", |r| r.set_material(id, material));
            }
            Message::FilletMaterial(material) => {
                ctx.doc.edit("Set fillet material", |r| r.set_fillet_material(id, material));
            }
            Message::LineMaterial(material) => {
                ctx.doc.edit("Set line material", |r| edit::set_line_material(r, id, material));
            }
            Message::Finish(finish) => {
                ctx.doc.edit("Set finish", |r| r.set_finish(id, finish));
            }
            Message::Trigger(trigger) => {
                ctx.doc.edit("Set deployment", |r| edit::set_trigger(r, id, trigger));
            }
            Message::MotorMount(mount) => {
                ctx.doc.edit("Set motor mount", |r| edit::set_motor_mount(r, id, mount));
            }
            Message::Plug(plugged) => {
                ctx.doc.edit("Set ejection charge", |r| edit::set_plugged(r, id, plugged));
            }
            Message::StartOverride(Field::MassOverride) => {
                let computed = rocket.component_mass(id).map_or(0.0, |m| m.mass);
                ctx.doc.edit("Override mass", |r| r.set_mass_override(id, Some(computed)));
            }
            Message::StartOverride(_) => {
                let cg = rocket.component_mass(id).zip(rocket.part_station(id)).map_or(0.0, |(m, station)| (m.cg - station).max(0.0));
                ctx.doc.edit("Override CG", |r| r.set_cg_override(id, Some(cg)));
            }
            Message::ClearOverride(Field::MassOverride) => {
                ctx.doc.edit("Clear mass override", |r| r.set_mass_override(id, None));
            }
            Message::ClearOverride(_) => {
                ctx.doc.edit("Clear CG override", |r| r.set_cg_override(id, None));
            }
            Message::Duplicate => {
                self.menu = None;
                if let Some(new) = ctx.doc.edit("Duplicate part", |r| edit::duplicate(r, id)) {
                    ctx.selection = Selection::Part(new);
                }
            }
            Message::Detach => {
                if let Some(new) = ctx.doc.edit("Detach fin", |r| r.detach_radial(id, instance).ok()) {
                    ctx.selection = Selection::Part(new);
                    ctx.symmetry.instance = None;
                }
            }
            Message::SplitAll => {
                ctx.doc.edit("Split all fins", |r| r.split_all_radial(id).ok());
                ctx.symmetry.instance = None;
            }
            Message::FieldMenu(field) => {
                let open = (id, instance, field);
                self.field_menu = if self.field_menu == Some(open) { None } else { Some(open) };
            }
            Message::Unlink(Field::Dim(dim)) => {
                if let Some(field) = RadialField::of_dim(dim) {
                    let label = format!("Unlink {} of fin {}", field.name(), bowshock_model::instance_letter(instance));
                    ctx.doc.edit(&label, |r| r.unlink_field(id, instance, field));
                }
            }
            Message::Relink(Field::Dim(dim)) => {
                if let Some(field) = RadialField::of_dim(dim) {
                    let label = format!("Relink {} of fin {}", field.name(), bowshock_model::instance_letter(instance));
                    ctx.doc.edit(&label, |r| r.relink_field(id, instance, field));
                }
            }
            Message::RelinkAll => {
                let label = format!("Relink fin {}", bowshock_model::instance_letter(instance));
                ctx.doc.edit(&label, |r| r.relink_copy(id, instance));
            }
            Message::ConfirmCount => {
                if let Some((_, count)) = self.confirm_count.take().filter(|(part, _)| *part == id) {
                    ctx.doc.edit("Set fin count", |r| edit::set_count(r, id, count));
                }
            }
            Message::CancelCount => self.confirm_count = None,
            Message::RejoinStart => match rocket.rejoin_candidates(id).as_slice() {
                [] => ctx.notify("No group of the same kind to rejoin"),
                [group] => ctx.symmetry.rejoin = Some(Rejoin { part: id, group: *group }),
                _ => self.rejoin_picking = Some(id),
            },
            Message::RejoinWith(group) => {
                self.rejoin_picking = None;
                if rocket.can_rejoin(id, group) {
                    ctx.symmetry.rejoin = Some(Rejoin { part: id, group });
                }
            }
            Message::RejoinCancel => {
                self.rejoin_picking = None;
                ctx.symmetry.rejoin = None;
            }
            Message::RejoinChoose(winner) => {
                if let Some(join) = ctx.symmetry.rejoin.take().filter(|j| j.part == id) {
                    if ctx.doc.edit("Rejoin group", |r| r.rejoin_radial(join.part, join.group, winner).is_ok()) {
                        ctx.selection = Selection::Part(join.group);
                        ctx.symmetry.instance = None;
                    }
                }
            }
            _ => {}
        }
    }

    /// The shortcut `key` maps to while the inspector is on screen, if any. Only called for keys no widget handled.
    pub fn on_key(&self, key: &keyboard::Key, _modifiers: keyboard::Modifiers, ctx: &Ctx) -> Option<Message> {
        match key {
            keyboard::Key::Named(keyboard::key::Named::Escape) if self.renaming.is_some() => Some(Message::RenameCancel),
            keyboard::Key::Named(keyboard::key::Named::Escape) if self.menu.is_some() => Some(Message::MenuOpen(false)),
            keyboard::Key::Named(keyboard::key::Named::Escape) if self.field_menu.is_some() => Some(Message::Noop),
            keyboard::Key::Named(keyboard::key::Named::Escape) if self.confirm_count.is_some() => Some(Message::CancelCount),
            keyboard::Key::Named(keyboard::key::Named::Escape) if self.rejoin_picking.is_some() || ctx.symmetry.rejoin.is_some() => Some(Message::RejoinCancel),
            _ => None,
        }
    }

    /// The inspector pane.
    pub fn view<'a>(&'a self, ctx: &'a Ctx) -> Element<'a, Message> {
        let rocket = ctx.doc.rocket();
        let selection = match ctx.selection {
            Selection::Part(id) if rocket.kind_of(id).is_none() => Selection::None,
            other => other,
        };
        let (header, sections, bar) = match selection {
            Selection::Part(_) if ctx.is_multi() => {
                let (header, sections) = multi::view(ctx, self.menu == Some(selection));
                (header, sections, None)
            }
            Selection::Part(id) => self.part_view(ctx, id),
            Selection::Separation(n) => match separation::view(ctx, n) {
                Some((title, station, sections)) => (self.simple_header(title, Some(station)), sections, None),
                None => self.rocket_view(ctx),
            },
            Selection::None => self.rocket_view(ctx),
        };
        let mut cells: Vec<Element<'a, Message>> = vec![header, rule()];
        if let Some(bar) = bar {
            cells.push(bar);
            cells.push(rule());
        }
        cells.push(
            scrollable(column(sections))
                .id(reveal::SCROLL_ID)
                .direction(scrollable::Direction::Vertical(scrollable::Scrollbar::new().width(4).scroller_width(4).margin(1)))
                .style(theme::scroll)
                .width(Fill)
                .height(Fill)
                .boxed(),
        );
        let body = column(cells);
        let panel = container(body).width(Fill).height(Fill).style(theme::panel);

        match (&self.menu, selection) {
            (Some(open), now) if *open == now && matches!(now, Selection::Part(_)) => {
                let backdrop = mouse_area(Space::new().width(Fill).height(Fill)).on_press(Message::MenuOpen(false));
                let can_copy =
                    matches!(now, Selection::Part(id) if rocket.part(id).is_none_or(|p| !matches!(p.kind, bowshock_model::PartKind::NoseCone { .. })));
                let mut items: Vec<Element<'a, Message>> = Vec::new();
                if can_copy && !ctx.is_multi() {
                    items.push(button("Duplicate", Weight::Ghost).width(Fill).on_press(Message::Duplicate).into());
                }
                items.push(button("Delete", Weight::Ghost).width(Fill).on_press(Message::Delete).into());
                let menu = container(container(column(items).width(132)).style(theme::overlay))
                    .padding(iced::Padding { top: HEADER_H - 6.0, right: SPACE_2, bottom: 0.0, left: 0.0 })
                    .align_right(Fill);
                stack![panel, backdrop, menu].boxed()
            }
            _ => panel.boxed(),
        }
    }

    fn simple_header<'a>(&'a self, title: String, tag: Option<String>) -> Element<'a, Message> {
        let mut cells: Vec<Element<'a, Message>> =
            vec![text(title).font(SANS_SEMIBOLD).size(type_scale::UI_TITLE.size).style(theme::ink).boxed(), Space::new().width(Fill).boxed()];
        if let Some(tag) = tag {
            cells.push(text(tag).font(MONO).size(12.5).style(theme::muted).boxed());
        }
        container(row(cells).align_y(iced::Center)).padding([0.0, SPACE_3]).height(HEADER_H).center_y(HEADER_H).boxed()
    }

    /// The Rocket inspector: header and sections.
    fn rocket_view<'a>(&'a self, ctx: &'a Ctx) -> (Element<'a, Message>, Vec<Element<'a, Message>>, Option<Element<'a, Message>>) {
        let (title, sections) = rocket_view::view(ctx);
        let title: Element<'a, Message> = match &self.renaming {
            Some((Selection::None, buffer)) => self.rename_field(buffer),
            _ => {
                let name = text(title).font(SANS_SEMIBOLD).size(type_scale::UI_TITLE.size).style(theme::ink);
                mouse_area(name).on_double_click(Message::StartRename).boxed()
            }
        };
        let header = container(
            row![silhouette::rocket_silhouette(ctx.doc.rocket()), title, Space::new().width(Fill), text("ROCKET").font(MONO).size(10.5).style(theme::faint)]
                .spacing(SPACE_2)
                .align_y(iced::Center),
        )
        .padding([0.0, SPACE_3])
        .height(HEADER_H)
        .center_y(HEADER_H)
        .boxed();
        (header, sections, None)
    }

    fn rename_field<'a>(&'a self, buffer: &'a str) -> Element<'a, Message> {
        text_input("Name", buffer)
            .id(RENAME_ID)
            .on_input(Message::RenameInput)
            .on_submit(Message::RenameCommit)
            .size(type_scale::UI_TITLE.size)
            .font(SANS_SEMIBOLD)
            .padding([2.0, SPACE_2])
            .style(theme::input)
            .width(Length::Fill)
            .boxed()
    }

    /// The header, sections and symmetry bar of the selected part.
    fn part_view<'a>(&'a self, ctx: &'a Ctx, id: PartId) -> (Element<'a, Message>, Vec<Element<'a, Message>>, Option<Element<'a, Message>>) {
        let rocket = ctx.doc.rocket();
        let flight = match rocket.kind_of(id) {
            Some(PartRef::Radial(r)) if r.kind.is_fin() => self.flight_for(ctx),
            Some(PartRef::Internal(p)) if matches!(p.kind, bowshock_model::InternalKind::Parachute { .. } | bowshock_model::InternalKind::Streamer { .. }) => {
                self.flight_for(ctx)
            }
            _ => None,
        };
        let limit = self.limit.filter(|(part, ..)| *part == id).map(|(_, dim, why)| (dim, why));
        let link = symmetry::Link::of(self, ctx, rocket, id);
        let copy = ctx.symmetry.copy_in(rocket, id);
        let params = Params { form: Form { ctx, limit, first: part::first_field(rocket, id) }, state: self, rocket, id, flight, link, copy };
        let sections = part::view(&params);
        let bar = symmetry::bar(self, ctx, id);
        (self.part_header(ctx, id), sections, bar)
    }

    /// The flight the health chips judge against, flown now unless a field is being scrubbed.
    fn flight_for(&self, ctx: &Ctx) -> Option<std::rc::Rc<health::Flight>> {
        if self.scrubbing {
            if let Some(stale) = self.flight.latest() {
                return Some(stale);
            }
        }
        self.flight.get(ctx.doc.rocket(), ctx.physics.motors(), ctx.doc.revision())
    }

    fn part_header<'a>(&'a self, ctx: &'a Ctx, id: PartId) -> Element<'a, Message> {
        let rocket = ctx.doc.rocket();
        let Some(part) = rocket.kind_of(id) else {
            return self.simple_header("Rocket".into(), None);
        };
        let name: Element<'a, Message> = match &self.renaming {
            Some((Selection::Part(p), buffer)) if *p == id => self.rename_field(buffer),
            _ => {
                let mut cells: Vec<Element<'a, Message>> =
                    vec![text(part.name()).font(SANS_SEMIBOLD).size(type_scale::UI_TITLE.size).style(theme::ink).boxed()];
                if rocket.radial(id).is_some_and(|r| r.count > 1) {
                    let letter = bowshock_model::instance_letter(ctx.symmetry.copy_in(rocket, id));
                    let tag = text(format!(" · {letter}")).font(SANS_SEMIBOLD).size(type_scale::UI_TITLE.size).style(theme::ink);
                    cells.push(mouse_area(tag).on_press(Message::NextInstance).interaction(mouse::Interaction::Pointer).boxed());
                }
                mouse_area(row(cells).align_y(iced::Center)).on_double_click(Message::StartRename).boxed()
            }
        };
        let renaming = matches!(&self.renaming, Some((Selection::Part(p), _)) if *p == id);
        let mut cells: Vec<Element<'a, Message>> = vec![silhouette::silhouette(rocket, id), name];
        if !renaming {
            cells.push(Space::new().width(Fill).boxed());
        }
        if let Some(subtype) = subtype(rocket, id) {
            cells.push(container(subtype).width(SUBTYPE_W).boxed());
        }
        cells.push(icon_button(Mark::Ellipsis, "More").on_press(Message::MenuOpen(self.menu != Some(Selection::Part(id)))).into());
        container(row(cells).spacing(SPACE_2).align_y(iced::Center)).padding([0.0, SPACE_2 + 2.0]).height(HEADER_H).center_y(HEADER_H).boxed()
    }
}

/// The header's sub-type select: a nose's shape family, a fin's planform, a mass component's role.
fn subtype<'a>(rocket: &Rocket, id: PartId) -> Option<Element<'a, Message>> {
    match rocket.kind_of(id)? {
        PartRef::Axial(p) => match p.kind {
            bowshock_model::PartKind::NoseCone { shape, .. } => {
                let options: Vec<(NoseShape, String)> = NoseShape::LIST.iter().map(|s| (*s, s.name().to_string())).collect();
                let current = NoseShape::LIST.iter().find(|s| discriminant(*s) == discriminant(&shape)).copied().unwrap_or(shape);
                Some(choose(options, &current, Message::NoseShape))
            }
            _ => None,
        },
        PartRef::Radial(r) => {
            let planform = r.kind.fin_planform()?;
            let options: Vec<(FinPlanform, String)> = FinPlanform::ALL.iter().map(|p| (*p, p.name().to_string())).collect();
            Some(choose(options, &planform, Message::Planform))
        }
        PartRef::Internal(p) => match p.kind {
            bowshock_model::InternalKind::MassComponent { role, .. } => Some(choose(part::role_options(), &role, Message::Role)),
            _ => None,
        },
    }
}
