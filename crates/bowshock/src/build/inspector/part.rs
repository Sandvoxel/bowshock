//! The sections of a selected part's inspector, in the kit's fixed order: 01 Geometry, 02 Position, 03 Material & finish,
//! 04 Mass (with the part's health chips at its end) and 05 Notes. Each builder returns the section's rows; the view puts
//! them in `widgets::field::section`s.

use std::rc::Rc;

use bowshock_model::{
    materials_of_kind, DeployEvent, Dim, EventKind, FinSection, Finish, InternalKind, InternalPart, MassRole, Material, MaterialKind, Part, PartId, PartKind,
    PartRef, RadialKind, RadialPart, Rocket,
};
use iced::widget::{column, container, mouse_area, row, text, text_input, Space};
use iced::{mouse, Element, Fill, Widget};

use super::edit::{Anchor, Field, Trigger};
use super::form::{choose, text_row, value_row, Form};
use super::health::{self, Flight};
use super::separation;
use super::symmetry::{linked, Link};
use super::{Message, State};
use crate::app::ui;
use crate::theme::{self, MONO};
use crate::tokens::{SPACE_2, SPACE_3, TEXT_BODY, TEXT_LABEL};
use crate::units::Quantity;
use crate::widgets::button::{button, Weight};
use crate::widgets::chip::chip;
use crate::widgets::field::{labeled, rule, section, section_head, section_picking};
use crate::widgets::glyph::{self, Mark, Tone};
use crate::widgets::segmented::segmented_fill;

/// Everything the section builders read.
pub struct Params<'a> {
    /// The field builder (unit system, limit hint).
    pub form: Form<'a>,
    /// The inspector's state (anchor, notes open).
    pub state: &'a State,
    /// The rocket on show.
    pub rocket: &'a Rocket,
    /// The selected part.
    pub id: u64,
    /// The cached flight, if one has been flown for this revision.
    pub flight: Option<Rc<Flight>>,
    /// The selected radial group and the copy on show, when it has two or more copies.
    pub link: Option<Link<'a>>,
    /// The copy of a radial group on show (0 is A); 0 for every other part.
    pub copy: usize,
}

/// The fin cross-section options as the kit words them.
const SECTIONS: [(FinSection, &str); 3] = [(FinSection::Square, "Square"), (FinSection::Rounded, "Round"), (FinSection::Airfoil, "Airfoil")];
/// Symmetry counts offered for a radial group.
const COUNTS: [(u8, &str); 6] = [(1, "1"), (2, "2"), (3, "3"), (4, "4"), (6, "6"), (8, "8")];

/// All sections of part `p.id`, numbered as the kit does (a part without one skips its number).
pub fn view<'a>(p: &Params<'a>) -> Vec<Element<'a, Message>> {
    let picking = p.form.picking_dim();
    let in_position = matches!(picking, Some(Dim::Position | Dim::Angle | Dim::Cant));
    let mut out = vec![section_picking(1, "Geometry", false, picking.is_some() && !in_position, geometry(p))];
    out.push(section_picking(2, "Position", true, in_position, position(p)));
    if let Some(rows) = material_rows(p) {
        out.push(section(3, "Material & finish", true, rows));
    }
    out.push(section(4, "Mass", true, mass(p)));
    out.push(notes(p));
    out
}

fn geometry<'a>(p: &Params<'a>) -> Vec<Element<'a, Message>> {
    match p.rocket.kind_of(p.id) {
        Some(PartRef::Axial(part)) => axial(p, part),
        Some(PartRef::Radial(radial)) => radial_rows(p, radial),
        Some(PartRef::Internal(part)) => internal(p, part),
        None => Vec::new(),
    }
}

fn dim(d: Dim) -> Field {
    Field::Dim(d)
}

fn axial<'a>(p: &Params<'a>, part: &Part) -> Vec<Element<'a, Message>> {
    let f = &p.form;
    let mut rows = Vec::new();
    match &part.kind {
        PartKind::NoseCone { shape, length, diameter } => {
            rows.push(f.num("Length", *length, Quantity::Length, dim(Dim::Length)));
            rows.push(f.num("Base Ø", *diameter, Quantity::Length, dim(Dim::Diameter)));
            if let Some(parameter) = shape.parameter() {
                let label = if matches!(shape, bowshock_model::NoseShape::PowerSeries(_)) { "Exponent n" } else { "Shape C" };
                rows.push(f.num(label, parameter, Quantity::Scalar, dim(Dim::Shape)));
            }
        }
        PartKind::BodyTube { length, diameter } => {
            rows.push(f.num("Length", *length, Quantity::Length, dim(Dim::Length)));
            rows.push(f.num("Outer Ø", *diameter, Quantity::Length, dim(Dim::Diameter)));
        }
        PartKind::Transition { length, fore_diameter, aft_diameter } => {
            rows.push(f.num("Length", *length, Quantity::Length, dim(Dim::Length)));
            rows.push(f.num("Fore Ø", *fore_diameter, Quantity::Length, dim(Dim::ForeDiameter)));
            rows.push(f.num("Aft Ø", *aft_diameter, Quantity::Length, dim(Dim::Diameter)));
        }
    }
    rows.push(f.num("Wall", part.thickness, Quantity::Length, dim(Dim::Wall)));
    let outer = part.kind.fore_diameter().max(part.kind.aft_diameter());
    rows.push(f.derived("Inner Ø", (outer - 2.0 * part.thickness).max(0.0), Quantity::Length, Field::BoreDiameter));
    for aft in [false, true] {
        let Some(s) = p.rocket.shoulder(part.id, aft) else { continue };
        let (length, diameter, wall) =
            if aft { ("Shoulder length", "Shoulder Ø", "Shoulder wall") } else { ("Fore shoulder length", "Fore shoulder Ø", "Fore shoulder wall") };
        let dims = if aft {
            (Dim::ShoulderLength, Dim::ShoulderDiameter, Dim::ShoulderThickness)
        } else {
            (Dim::ForeShoulderLength, Dim::ForeShoulderDiameter, Dim::ForeShoulderThickness)
        };
        rows.push(f.num(length, s.length, Quantity::Length, dim(dims.0)));
        let resolved = p.rocket.shoulder_diameter(part.id, aft).unwrap_or(0.0);
        rows.push(f.auto_or(diameter, resolved, Quantity::Length, dim(dims.1), s.diameter <= 0.0, Message::ResetAuto(dim(dims.1))));
        rows.push(f.num(wall, s.thickness, Quantity::Length, dim(dims.2)));
    }
    rows
}

fn radial_rows<'a>(p: &Params<'a>, radial: &RadialPart) -> Vec<Element<'a, Message>> {
    let f = &p.form;
    let mut rows: Vec<Element<'a, Message>> = Vec::new();
    let count_label = if radial.kind.is_fin() { "Fin count" } else { "Count" };
    rows.push(labeled(count_label, segmented_fill(&COUNTS, radial.count, Message::Count)));
    // The fields show the copy on show: the group's values with that copy's own overrides on top.
    let own = radial.effective(p.copy);
    match &own.kind {
        RadialKind::FinSet { root_chord, tip_chord, span, sweep, thickness } => {
            rows.push(linked(p, "Root chord", *root_chord, Quantity::Length, Dim::RootChord));
            rows.push(linked(p, "Tip chord", *tip_chord, Quantity::Length, Dim::TipChord));
            rows.push(linked(p, "Height", *span, Quantity::Length, Dim::Span));
            rows.push(linked(p, "Sweep length", *sweep, Quantity::Length, Dim::Sweep));
            rows.push(f.derived("Sweep angle", sweep.atan2(*span), Quantity::Angle, Field::SweepAngle));
            rows.push(linked(p, "Thickness", *thickness, Quantity::Length, Dim::Thickness));
        }
        RadialKind::EllipticalFin { root_chord, span, thickness } => {
            rows.push(linked(p, "Root chord", *root_chord, Quantity::Length, Dim::RootChord));
            rows.push(linked(p, "Height", *span, Quantity::Length, Dim::Span));
            rows.push(linked(p, "Thickness", *thickness, Quantity::Length, Dim::Thickness));
        }
        RadialKind::FreeformFin { points, thickness } => {
            // The outline is edited by its vertices in the viewport; the extents follow from it.
            rows.push(f.fixed("Root chord", own.kind.extent(), Quantity::Length));
            rows.push(f.fixed("Height", own.kind.fin_span().unwrap_or(0.0), Quantity::Length));
            rows.push(text_row("Vertices", format!("{} (drag them in the viewport)", points.len())));
            rows.push(linked(p, "Thickness", *thickness, Quantity::Length, Dim::Thickness));
        }
        RadialKind::LaunchLug { length, diameter } => {
            rows.push(linked(p, "Length", *length, Quantity::Length, Dim::Length));
            rows.push(linked(p, "Outer Ø", *diameter, Quantity::Length, Dim::Diameter));
        }
        RadialKind::RailButton { height, diameter } => {
            rows.push(linked(p, "Height", *height, Quantity::Length, Dim::Height));
            rows.push(linked(p, "Head Ø", *diameter, Quantity::Length, Dim::Diameter));
        }
    }
    if radial.kind.is_fin() {
        rows.push(labeled("Cross-section", segmented_fill(&SECTIONS, radial.cross_section, Message::CrossSection)));
        rows.push(f.num("Fillet radius", radial.fillet_radius, Quantity::Length, dim(Dim::FilletRadius)));
    }
    rows
}

fn internal<'a>(p: &Params<'a>, part: &InternalPart) -> Vec<Element<'a, Message>> {
    let f = &p.form;
    let res = p.rocket.resolved(part.id);
    let (outer, outer_auto, inner, inner_auto) = res.map_or((0.0, false, 0.0, false), |r| (r.outer_diameter, r.outer_auto, r.inner_diameter, r.inner_auto));
    let auto_outer = |label: &'static str| f.auto_or(label, outer, Quantity::Length, Field::AutoDiameter, outer_auto, Message::ResetAuto(Field::AutoDiameter));
    let mut rows: Vec<Element<'a, Message>> = Vec::new();
    match &part.kind {
        InternalKind::InnerTube { outer_diameter, thickness, length, motor_mount, overhang } => {
            rows.push(f.num("Length", *length, Quantity::Length, dim(Dim::Length)));
            rows.push(f.num("Outer Ø", *outer_diameter, Quantity::Length, dim(Dim::Diameter)));
            rows.push(f.num("Wall", *thickness, Quantity::Length, dim(Dim::Wall)));
            rows.push(f.derived("Inner Ø", inner, Quantity::Length, Field::BoreDiameter));
            rows.push(labeled("Motor mount", crate::widgets::field::checkbox("Holds a motor", *motor_mount, Message::MotorMount)));
            if *motor_mount {
                rows.push(f.num("Overhang", *overhang, Quantity::Length, dim(Dim::Overhang)));
                rows.extend(motor_rows(p));
            }
        }
        InternalKind::CenteringRing { thickness, .. } => {
            rows.push(auto_outer("Outer Ø"));
            rows.push(f.auto_or("Inner Ø", inner, Quantity::Length, dim(Dim::InnerDiameter), inner_auto, Message::ResetAuto(dim(Dim::InnerDiameter))));
            rows.push(f.num("Thickness", *thickness, Quantity::Length, dim(Dim::Thickness)));
        }
        InternalKind::Bulkhead { thickness, .. } => {
            rows.push(auto_outer("Diameter"));
            rows.push(f.num("Thickness", *thickness, Quantity::Length, dim(Dim::Thickness)));
        }
        InternalKind::Coupler { thickness, length, .. } => {
            rows.push(f.num("Length", *length, Quantity::Length, dim(Dim::Length)));
            rows.push(auto_outer("Outer Ø"));
            rows.push(f.num("Wall", *thickness, Quantity::Length, dim(Dim::Wall)));
            rows.push(f.derived("Inner Ø", inner, Quantity::Length, Field::BoreDiameter));
        }
        InternalKind::EngineBlock { inner: hole, thickness, .. } => {
            rows.push(auto_outer("Outer Ø"));
            rows.push(f.num("Inner Ø", *hole, Quantity::Length, dim(Dim::InnerDiameter)));
            rows.push(f.num("Thickness", *thickness, Quantity::Length, dim(Dim::Thickness)));
        }
        InternalKind::Parachute { diameter, cd, line_count, line_length, packed_length, .. } => {
            rows.push(f.num("Canopy Ø", *diameter, Quantity::Length, Field::CanopyDiameter));
            rows.push(f.num("Drag coeff.", *cd, Quantity::Scalar, Field::Cd));
            rows.push(f.num("Lines", f64::from(*line_count), Quantity::Scalar, Field::LineCount));
            rows.push(f.num("Line length", *line_length, Quantity::Length, Field::LineLength));
            rows.push(f.num("Packed length", *packed_length, Quantity::Length, dim(Dim::PackedLength)));
            rows.push(auto_outer("Packed Ø"));
            rows.extend(deployment_rows(p, part.id));
        }
        InternalKind::Streamer { length, width, cd, .. } => {
            rows.push(f.num("Length", *length, Quantity::Length, Field::StreamerLength));
            rows.push(f.num("Width", *width, Quantity::Length, Field::StreamerWidth));
            rows.push(f.num("Drag coeff.", *cd, Quantity::Scalar, Field::Cd));
            rows.extend(deployment_rows(p, part.id));
        }
        InternalKind::ShockCord { length, packed_length } => {
            rows.push(f.num("Length", *length, Quantity::Length, Field::CordLength));
            rows.push(f.num("Packed length", *packed_length, Quantity::Length, dim(Dim::PackedLength)));
        }
        InternalKind::MassComponent { mass, length, .. } => {
            rows.push(f.num("Mass", *mass, Quantity::Mass, Field::MassValue));
            rows.push(f.num("Length", *length, Quantity::Length, dim(Dim::Length)));
            rows.push(auto_outer("Diameter"));
        }
    }
    rows
}

/// A parachute's or streamer's trigger, altitude and delay in the active configuration (the part's own value is what a
/// configuration without one of its own gets): the same event-plus-delay pattern as the staging sections, under a head that
/// carries the `PER CONFIG` badge.
fn deployment_rows<'a>(p: &Params<'a>, id: PartId) -> Vec<Element<'a, Message>> {
    let f = &p.form;
    let deploy = p.rocket.active_deployment(id).unwrap_or_default();
    let options: Vec<(Trigger, String)> = Trigger::ALL.iter().map(|t| (*t, t.name().to_string())).collect();
    let head = row![text("DEPLOYMENT").font(MONO).size(9.5).style(theme::muted), separation::badge("PER CONFIG")].spacing(SPACE_2).align_y(iced::Center);
    let mut rows = vec![container(head).padding([SPACE_2, 0.0]).boxed(), labeled("Deploys on", choose(options, &Trigger::of(deploy.event), Message::Trigger))];
    if let DeployEvent::Altitude(h) = deploy.event {
        rows.push(f.num("Altitude", h, Quantity::Altitude, Field::DeployAltitude));
    }
    rows.push(f.num("Delay", deploy.delay, Quantity::Time, Field::DeployDelay));
    rows
}

/// What the active configuration holds in a motor mount: the motor, its ejection delay and (above stage 1) its ignition.
fn motor_rows<'a>(p: &Params<'a>) -> Vec<Element<'a, Message>> {
    let f = &p.form;
    let config = p.rocket.active_configuration();
    let slot = config.slot(p.id);
    let designation = slot.and_then(|s| s.motor.as_ref()).map_or_else(|| "None".to_string(), |m| m.designation.clone());
    let mut rows = vec![text_row("Motor", designation.clone())];
    let Some(slot) = slot.filter(|s| s.motor.is_some()) else {
        return rows;
    };
    rows.push(labeled("Ejection", crate::widgets::field::checkbox("Plugged (no charge)", slot.is_plugged(), Message::Plug)));
    if !slot.is_plugged() {
        rows.push(f.per_config("Ejection delay", slot.delay, Quantity::Time, Field::EjectionDelay));
    }
    let stage = p.state.stage_of_mount(p.rocket, p.id);
    if stage > 1 {
        let lower = separation::stage_name(stage - 1, p.rocket.stages().len());
        rows.extend(separation::timing_rows(f, config, stage, EventKind::Ignition, &lower, true));
    }
    rows
}

/// The parent's name and length, the part's extent and its offset from the parent's fore face.
pub(super) fn placement(rocket: &Rocket, id: u64) -> Option<(String, f64, f64, f64)> {
    let (parent, extent, offset) = match rocket.kind_of(id)? {
        PartRef::Radial(r) => (r.parent, r.kind.extent(), r.offset),
        PartRef::Internal(i) => (i.parent, i.kind.extent(), i.offset),
        PartRef::Axial(_) => return None,
    };
    let parent = rocket.part(parent)?;
    Some((parent.display_name(), parent.kind.length(), extent, offset))
}

fn position<'a>(p: &Params<'a>) -> Vec<Element<'a, Message>> {
    let f = &p.form;
    let rocket = p.rocket;
    let station = rocket.part_station(p.id).unwrap_or(0.0);
    let Some((parent, parent_length, extent, offset)) = placement(rocket, p.id) else {
        // An axial part sits in the stack: its station follows from what is ahead of it.
        let after = rocket.index_of(p.id).and_then(|i| i.checked_sub(1)).and_then(|i| rocket.parts().get(i)).map(|part| part.display_name());
        return vec![
            f.fixed("Station", station, Quantity::Length),
            text_row("Order", after.map_or_else(|| "First (nose)".to_string(), |name| format!("After {name}"))),
        ];
    };
    let anchor = p.state.anchor(rocket, p.id);
    let options: Vec<(Anchor, String)> = Anchor::ALL.iter().map(|a| (*a, a.label(&parent))).collect();
    let mut rows = vec![
        labeled("Relative to", choose(options, &anchor, Message::Anchor)),
        f.num("Offset", anchor.shown(offset, extent, parent_length), Quantity::Length, dim(Dim::Position)),
    ];
    if let Some(radial) = rocket.radial(p.id) {
        rows.push(f.num("Rotation", radial.angle, Quantity::Angle, dim(Dim::Angle)));
        if radial.kind.is_fin() {
            rows.push(linked(p, "Cant", radial.effective(p.copy).cant, Quantity::Angle, Dim::Cant));
        }
    }
    rows
}

/// The material and finish selects: bulk stock for tubes, fins and discs, fabric for a canopy, cord for the lines. A mass
/// component has no material of its own (its mass is the number), so it has no section.
fn material_rows<'a>(p: &Params<'a>) -> Option<Vec<Element<'a, Message>>> {
    let (material, kind, finish, canopy) = match p.rocket.kind_of(p.id)? {
        PartRef::Axial(part) => (part.material, MaterialKind::Bulk, Some(part.finish), false),
        PartRef::Radial(r) => (r.material, MaterialKind::Bulk, Some(r.finish), false),
        PartRef::Internal(i) => (i.material, i.kind.material_kind()?, None, matches!(i.kind, InternalKind::Parachute { .. } | InternalKind::Streamer { .. })),
    };
    let options = |kind: MaterialKind| -> Vec<(bowshock_model::MaterialId, String)> { materials_of_kind(kind).map(|m| (m.id, material_label(m))).collect() };
    let mut rows = vec![labeled(if canopy { "Canopy" } else { "Material" }, choose(options(kind), &material, Message::Material))];
    if let Some(InternalPart { kind: InternalKind::Parachute { line_material, .. }, .. }) = p.rocket.internal(p.id) {
        rows.push(labeled("Lines", choose(options(MaterialKind::Line), line_material, Message::LineMaterial)));
    }
    if let Some(finish) = finish {
        if let Some(r) = p.rocket.radial(p.id).filter(|r| r.kind.is_fin()) {
            rows.push(labeled("Fillet", choose(options(MaterialKind::Bulk), &r.fillet_material, Message::FilletMaterial)));
        }
        let finishes: Vec<(Finish, String)> = Finish::ALL.iter().map(|f| (*f, finish_label(*f))).collect();
        rows.push(labeled("Finish", choose(finishes, &finish, Message::Finish)));
    }
    Some(rows)
}

/// `Basswood · 500 kg/m³`.
pub fn material_label(m: &Material) -> String {
    let density = if m.density < 1.0 { format!("{:.3}", m.density) } else { format!("{:.0}", m.density) };
    format!("{} · {density} {}", m.name, m.kind.density_unit())
}

/// `Regular paint · 60 µm`.
pub fn finish_label(f: Finish) -> String {
    format!("{} · {:.0} µm", f.name(), f.roughness() * 1e6)
}

fn mass<'a>(p: &Params<'a>) -> Vec<Element<'a, Message>> {
    let f = &p.form;
    let rocket = p.rocket;
    let units = p.form.ctx.units;
    let props = rocket.component_mass(p.id);
    let (mass_override, cg_override, is_mass_component) = match rocket.kind_of(p.id) {
        Some(PartRef::Axial(part)) => (part.mass_override, part.cg_override, false),
        Some(PartRef::Radial(r)) => (r.mass_override, r.cg_override, false),
        Some(PartRef::Internal(i)) => (i.mass_override, i.cg_override, matches!(i.kind, InternalKind::MassComponent { .. })),
        None => (None, None, false),
    };
    let ghost = |label: &'static str, message: Message| -> Element<'a, Message> { button(label, Weight::Ghost).on_press(message).into() };
    let mut rows: Vec<Element<'a, Message>> = Vec::new();

    let computed = props.map_or(0.0, |m| m.mass);
    let group = rocket.radial(p.id).filter(|r| r.count > 1).map(|r| r.count);
    let computed_text = match group {
        Some(n) => format!("{} ×{n}", ui::with_unit(computed, Quantity::Mass, units)),
        None => ui::with_unit(computed, Quantity::Mass, units),
    };
    match (is_mass_component, mass_override) {
        // A mass component's mass is its own number in 01 Geometry; here it is only read.
        (true, _) => rows.push(value_row("Mass", computed_text, None)),
        (false, None) => rows.push(value_row("Computed", computed_text, Some(ghost("Override…", Message::StartOverride(Field::MassOverride))))),
        (false, Some(m)) => {
            rows.push(f.overridden("Mass", m, Quantity::Mass, Field::MassOverride));
            rows.push(value_row(
                "Computed",
                ui::with_unit(computed_unset(p), Quantity::Mass, units),
                Some(ghost("Clear", Message::ClearOverride(Field::MassOverride))),
            ));
        }
    }

    let station = rocket.part_station(p.id).unwrap_or(0.0);
    let cg = props.map_or(station, |m| m.cg);
    match cg_override {
        None => rows.push(value_row("CG", ui::with_unit(cg, Quantity::Length, units), Some(ghost("Override…", Message::StartOverride(Field::CgOverride))))),
        Some(c) => {
            rows.push(f.overridden("CG from fore", c, Quantity::Length, Field::CgOverride));
            rows.push(value_row("CG", ui::with_unit(cg, Quantity::Length, units), Some(ghost("Clear", Message::ClearOverride(Field::CgOverride)))));
        }
    }

    for check in health::checks(rocket, p.id, p.flight.as_deref(), p.form.ctx) {
        // A chip that would not fit beside its label takes the whole row under it.
        let element: Element<'a, Message> = chip(check.severity, check.text.clone()).into();
        rows.push(if check.text.chars().count() <= 22 { labeled(check.row, element) } else { element });
    }
    rows
}

/// The mass the part would have with its override removed.
fn computed_unset(p: &Params<'_>) -> f64 {
    let mut copy = p.rocket.clone();
    copy.set_mass_override(p.id, None);
    copy.component_mass(p.id).map_or(0.0, |m| m.mass)
}

/// The first editable numeric field of part `id`: it takes the id the parts bin focuses on a double click.
pub fn first_field(rocket: &Rocket, id: u64) -> Option<Field> {
    Some(match rocket.kind_of(id)? {
        PartRef::Axial(_) => dim(Dim::Length),
        PartRef::Radial(r) => match &r.kind {
            RadialKind::FinSet { .. } | RadialKind::EllipticalFin { .. } => dim(Dim::RootChord),
            RadialKind::FreeformFin { .. } => dim(Dim::Thickness),
            RadialKind::LaunchLug { .. } => dim(Dim::Length),
            RadialKind::RailButton { .. } => dim(Dim::Height),
        },
        PartRef::Internal(i) => match &i.kind {
            InternalKind::InnerTube { .. } | InternalKind::Coupler { .. } => dim(Dim::Length),
            InternalKind::CenteringRing { .. } | InternalKind::Bulkhead { .. } | InternalKind::EngineBlock { .. } => Field::AutoDiameter,
            InternalKind::Parachute { .. } => Field::CanopyDiameter,
            InternalKind::Streamer { .. } => Field::StreamerLength,
            InternalKind::ShockCord { .. } => Field::CordLength,
            InternalKind::MassComponent { .. } => Field::MassValue,
        },
    })
}

/// `05 NOTES`, collapsed by default; open it to edit the part's note.
fn notes<'a>(p: &Params<'a>) -> Element<'a, Message> {
    let open = p.state.notes_open;
    let note = p.rocket.note(p.id);
    let caret = glyph::icon(if open { Mark::CaretDown } else { Mark::CaretUp }, Tone::Faint, 10.0);
    let summary: Element<'a, Message> = if open || note.is_empty() {
        Space::new().boxed()
    } else {
        let first = note.lines().next().unwrap_or("");
        text(first.chars().take(24).collect::<String>()).size(TEXT_LABEL).style(theme::muted).boxed()
    };
    let head = mouse_area(row![section_head(5, "Notes"), caret, Space::new().width(Fill).boxed(), summary].spacing(SPACE_2).align_y(iced::Center))
        .on_press(Message::NotesOpen(!open))
        .interaction(mouse::Interaction::Pointer);
    let mut cells: Vec<Element<'a, Message>> = vec![rule(), container(head).padding([SPACE_2, 0.0]).boxed()];
    if open {
        cells.push(
            text_input("Notes about this part", note).on_input(Message::Note).size(TEXT_BODY).font(MONO).padding([4.0, SPACE_2]).style(theme::input).boxed(),
        );
    }
    container(column(cells).spacing(SPACE_2)).padding([SPACE_3, SPACE_3]).width(Fill).boxed()
}

/// The material row labels, exposed for the header's sub-type select of a mass component.
pub fn role_options() -> Vec<(MassRole, String)> {
    MassRole::ALL.iter().map(|r| (*r, r.name().to_string())).collect()
}
