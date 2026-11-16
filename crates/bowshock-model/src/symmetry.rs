//! Symmetry groups: a radial part is a group of `count` linked copies spaced evenly around its parent (design/components/
//! Symmetry). This module holds the three states of a group and the moves between them:
//!
//! 1. **Linked**: one set of settings drives every copy.
//! 2. **Per-value override**: one field of one copy is loosened from the group ([`Rocket::unlink_field`], e.g. a single canted
//!    fin for a roll test). The group's `overrides` list names the copy, the field and its own value; every other field stays
//!    shared. [`Rocket::relink_field`] puts it back. Everything that needs a copy's values reads them through
//!    [`RadialPart::effective`] / [`Rocket::effective_params`] and never the group's own, so geometry, mass and physics agree.
//! 3. **Detached / split**: *Detach* lets one copy leave as a free part ([`Rocket::detach_radial`]), *Split all* frees every
//!    copy ([`Rocket::split_all_radial`]); a free part can *Rejoin* a group ([`Rocket::rejoin_radial`]) with the group's or its
//!    own settings winning.
//!
//! The model stores a group as one [`RadialPart`] (shared geometry, a start angle and a count). A fresh group's copies are evenly
//! spaced; the angular position is never shared, so once a copy leaves, the copies left behind keep their own angles
//! ([`RadialPart::instance_angles`], read through [`RadialPart::angles`]). Each free part keeps the angle its copy had. Changing
//! the count of a linked group re-spaces it evenly and drops its overrides (the Symmetry spec). Rejoining a copy to a group
//! that has its own angles adds the part's angle; to an evenly spaced group it re-spaces. Detach, split and
//! rejoin keep the total mass: a mass override of the whole group is shared out by copy.
//!
//! The spec's `SymmetryGroup { members, shared, overrides: HashMap<ComponentId, FieldMask> }` maps onto the group part itself:
//! the members are its copies (index `0..count`, lettered [`instance_letter`]), `shared` is the part's own kind and cant, and
//! `overrides` is [`RadialPart::overrides`] (a sorted list of copy, field, value; a map keyed by a float-bearing enum would not
//! serialise as JSON).

use serde::{Deserialize, Serialize};

use crate::geometry::{RadialKind, RadialPart};
use crate::handles::{Dim, DimEdit};
use crate::rocket::{RadialError, Rocket};
use crate::{PartId, MAX_COUNT};

/// The letter that labels copy `index` of a group (`A`, `B`, ... `Z`, then `AA`).
pub fn instance_letter(index: usize) -> String {
    let (n, letter) = (index / 26, char::from(b'A' + (index % 26) as u8));
    if n == 0 {
        letter.to_string()
    } else {
        format!("{}{letter}", char::from(b'A' + ((n - 1) % 26) as u8))
    }
}

/// A value of a radial part that one copy may hold on its own (*Unlink for this fin*). Position and angle are never
/// per-copy: the group stores a start angle and the copies are spaced evenly.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub enum RadialField {
    /// Fin root chord (trapezoidal and elliptical).
    RootChord,
    /// Fin tip chord (trapezoidal).
    TipChord,
    /// Fin height above the surface (trapezoidal and elliptical).
    Span,
    /// Fin sweep length (trapezoidal).
    Sweep,
    /// Fin plate thickness.
    Thickness,
    /// Fin cant, radians.
    Cant,
    /// Launch lug length.
    Length,
    /// Launch lug outer diameter or rail button head diameter.
    Diameter,
    /// Rail button height.
    Height,
}

impl RadialField {
    /// The field edited by `dim`, if copies may differ in it.
    pub fn of_dim(dim: Dim) -> Option<RadialField> {
        Some(match dim {
            Dim::RootChord => RadialField::RootChord,
            Dim::TipChord => RadialField::TipChord,
            Dim::Span => RadialField::Span,
            Dim::Sweep => RadialField::Sweep,
            Dim::Thickness => RadialField::Thickness,
            Dim::Cant => RadialField::Cant,
            Dim::Length => RadialField::Length,
            Dim::Diameter => RadialField::Diameter,
            Dim::Height => RadialField::Height,
            _ => return None,
        })
    }

    /// The dimension this field is edited as.
    pub fn dim(self) -> Dim {
        match self {
            RadialField::RootChord => Dim::RootChord,
            RadialField::TipChord => Dim::TipChord,
            RadialField::Span => Dim::Span,
            RadialField::Sweep => Dim::Sweep,
            RadialField::Thickness => Dim::Thickness,
            RadialField::Cant => Dim::Cant,
            RadialField::Length => Dim::Length,
            RadialField::Diameter => Dim::Diameter,
            RadialField::Height => Dim::Height,
        }
    }

    /// Lower-case name for the lanes' override list and the menus (`cant`, `root chord`).
    pub fn name(self) -> &'static str {
        match self {
            RadialField::RootChord => "root chord",
            RadialField::TipChord => "tip chord",
            RadialField::Span => "height",
            RadialField::Sweep => "sweep",
            RadialField::Thickness => "thickness",
            RadialField::Cant => "cant",
            RadialField::Length => "length",
            RadialField::Diameter => "diameter",
            RadialField::Height => "height",
        }
    }

    /// Whether a part of `kind` has this field (a trapezoid has a tip chord, an ellipse does not; a freeform fin's outline
    /// is its vertices, so only its thickness and cant can differ).
    pub fn applies_to(self, kind: &RadialKind) -> bool {
        self.read(kind, 0.0).is_some()
    }

    /// The field's value in `kind` (`cant` is the part's cant, passed separately because it is not in the kind).
    fn read(self, kind: &RadialKind, cant: f64) -> Option<f64> {
        match (self, kind) {
            (RadialField::RootChord, RadialKind::FinSet { root_chord, .. } | RadialKind::EllipticalFin { root_chord, .. }) => Some(*root_chord),
            (RadialField::TipChord, RadialKind::FinSet { tip_chord, .. }) => Some(*tip_chord),
            (RadialField::Span, RadialKind::FinSet { span, .. } | RadialKind::EllipticalFin { span, .. }) => Some(*span),
            (RadialField::Sweep, RadialKind::FinSet { sweep, .. }) => Some(*sweep),
            (RadialField::Thickness, k) => k.thickness(),
            (RadialField::Cant, k) if k.is_fin() => Some(cant),
            (RadialField::Length, RadialKind::LaunchLug { length, .. }) => Some(*length),
            (RadialField::Diameter, RadialKind::LaunchLug { diameter, .. } | RadialKind::RailButton { diameter, .. }) => Some(*diameter),
            (RadialField::Height, RadialKind::RailButton { height, .. }) => Some(*height),
            _ => None,
        }
    }

    /// Writes `value` into `kind` (or `cant`); false if the field does not apply.
    fn write(self, kind: &mut RadialKind, cant: &mut f64, value: f64) -> bool {
        match (self, kind) {
            (RadialField::RootChord, RadialKind::FinSet { root_chord, .. } | RadialKind::EllipticalFin { root_chord, .. }) => *root_chord = value,
            (RadialField::TipChord, RadialKind::FinSet { tip_chord, .. }) => *tip_chord = value,
            (RadialField::Span, RadialKind::FinSet { span, .. } | RadialKind::EllipticalFin { span, .. }) => *span = value,
            (RadialField::Sweep, RadialKind::FinSet { sweep, .. }) => *sweep = value,
            (
                RadialField::Thickness,
                RadialKind::FinSet { thickness, .. } | RadialKind::EllipticalFin { thickness, .. } | RadialKind::FreeformFin { thickness, .. },
            ) => *thickness = value,
            (RadialField::Cant, k) if k.is_fin() => *cant = value,
            (RadialField::Length, RadialKind::LaunchLug { length, .. }) => *length = value,
            (RadialField::Diameter, RadialKind::LaunchLug { diameter, .. } | RadialKind::RailButton { diameter, .. }) => *diameter = value,
            (RadialField::Height, RadialKind::RailButton { height, .. }) => *height = value,
            _ => return false,
        }
        true
    }
}

/// One loosened field: copy `index` of a group holds `value` for `field` instead of the group's own.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct FieldOverride {
    /// The copy (0 is A).
    pub index: u8,
    /// The field.
    pub field: RadialField,
    /// Its value for that copy, SI (metres, radians).
    pub value: f64,
}

/// What one copy of a radial group is: the geometry and cant it actually has, the group's values with the copy's overrides on
/// top. This is the only thing rendering, picking, mass and physics read for a copy (the spec's `effective_params`).
#[derive(Clone, Debug, PartialEq)]
pub struct InstanceParams {
    /// The copy's geometry.
    pub kind: RadialKind,
    /// The copy's cant, radians (0 for parts that cannot cant).
    pub cant: f64,
}

impl RadialPart {
    /// The values of copy `index` with its overrides applied. An index past the last copy gets the group's values.
    pub fn effective(&self, index: usize) -> InstanceParams {
        let mut params = InstanceParams { kind: self.kind.clone(), cant: self.cant };
        for o in self.overrides.iter().filter(|o| usize::from(o.index) == index) {
            o.field.write(&mut params.kind, &mut params.cant, o.value);
        }
        params
    }

    /// The group's own (shared) value of `field`, if the part has it.
    pub fn group_value(&self, field: RadialField) -> Option<f64> {
        field.read(&self.kind, self.cant)
    }

    /// The value of `field` for copy `index`: its override, else the group's.
    pub fn value(&self, index: usize, field: RadialField) -> Option<f64> {
        self.override_of(index, field).or_else(|| self.group_value(field))
    }

    /// Copy `index`'s own value of `field`, if it has been unlinked.
    pub fn override_of(&self, index: usize, field: RadialField) -> Option<f64> {
        self.overrides.iter().find(|o| usize::from(o.index) == index && o.field == field).map(|o| o.value)
    }

    /// Whether copy `index` has unlinked `field`.
    pub fn is_overridden(&self, index: usize, field: RadialField) -> bool {
        self.override_of(index, field).is_some()
    }

    /// Whether any copy has loosened any field.
    pub fn has_overrides(&self) -> bool {
        !self.overrides.is_empty()
    }

    /// The fields copy `index` has unlinked, in field order.
    pub fn overridden_fields(&self, index: usize) -> Vec<RadialField> {
        let mut fields: Vec<RadialField> = self.overrides.iter().filter(|o| usize::from(o.index) == index).map(|o| o.field).collect();
        fields.sort_unstable();
        fields
    }

    /// The copies with at least one override, in order.
    pub fn overridden_copies(&self) -> Vec<usize> {
        let mut copies: Vec<usize> = self.overrides.iter().map(|o| usize::from(o.index)).collect();
        copies.sort_unstable();
        copies.dedup();
        copies
    }

    /// The overrides as the Station Lanes list them on the group's bar: `B: cant`, `B: cant, height · C: sweep`.
    pub fn override_summary(&self) -> String {
        self.overridden_copies()
            .into_iter()
            .map(|i| {
                let names: Vec<&str> = self.overridden_fields(i).into_iter().map(RadialField::name).collect();
                format!("{}: {}", instance_letter(i), names.join(", "))
            })
            .collect::<Vec<_>>()
            .join(" · ")
    }

    /// Stores explicit per-copy angles (one per copy, the first becomes `angle`); evenly spaced lists are stored as `None`.
    pub(crate) fn set_angles(&mut self, list: Vec<f64>) {
        let tau = std::f64::consts::TAU;
        let n = list.len().max(1);
        self.angle = list.first().copied().unwrap_or(self.angle).rem_euclid(tau);
        // Evenly spaced as a set (the order a rejoined copy was appended in does not matter).
        let mut rel: Vec<f64> = list.iter().map(|a| (a - self.angle).rem_euclid(tau)).collect();
        rel.sort_by(f64::total_cmp);
        let even = rel.iter().enumerate().all(|(i, a)| {
            let d = (a - tau * i as f64 / n as f64).abs();
            d < 1e-9 || tau - d < 1e-9
        });
        self.instance_angles = if even { None } else { Some(list) };
    }

    /// Drops overrides that name a copy that no longer exists or a field the kind does not have.
    pub(crate) fn prune_overrides(&mut self) {
        let (count, kind) = (self.count, &self.kind);
        self.overrides.retain(|o| o.index < count && o.field.applies_to(kind));
        self.overrides.sort_by_key(|o| (o.index, o.field));
    }
}

/// Whose settings win when a free part rejoins a group.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Winner {
    /// The group keeps its settings (and its overrides); the part's own are dropped.
    Group,
    /// The group takes the part's settings (its overrides go).
    Part,
}

/// The name a copy takes when it leaves its group: `Fin B`, `Launch lug C`, or the group's own name with the letter.
fn free_name(group: &RadialPart, index: usize) -> String {
    let base = match (&group.name, &group.kind) {
        (Some(name), _) => name.clone(),
        (None, kind) if kind.is_fin() => "Fin".to_string(),
        (None, kind) => kind.name().to_string(),
    };
    format!("{base} {}", instance_letter(group.copy_index(index)))
}

/// Whether a part may join a group: fins with fins, lugs with lugs, buttons with buttons.
fn same_family(a: &RadialKind, b: &RadialKind) -> bool {
    !a.is_pod() && !b.is_pod() && (std::mem::discriminant(a) == std::mem::discriminant(b) || (a.is_fin() && b.is_fin()))
}

impl Rocket {
    fn fresh_part_id(&mut self) -> PartId {
        let id = self.next_id;
        self.next_id += 1;
        id
    }

    /// A free part made from copy `index` of `group`: one copy at that copy's own angle, with the copy's effective values.
    fn free_copy(&mut self, group: &RadialPart, index: usize) -> RadialPart {
        let count = f64::from(group.count.max(1));
        let own = group.effective(index);
        RadialPart {
            id: self.fresh_part_id(),
            kind: own.kind,
            cant: own.cant,
            angle: group.angles().get(index).copied().unwrap_or(group.angle),
            count: 1,
            name: Some(free_name(group, index)),
            mass_override: group.mass_override.map(|m| m / count),
            cg_override: group.cg_override,
            overrides: Vec::new(),
            instance_angles: None,
            letters: Vec::new(),
            ..group.clone()
        }
    }

    /// The effective values of copy `index` of radial group `id` (the spec's `effective_params`): the group's with the copy's
    /// overrides on top. `None` if `id` is not a radial part or `index` is past its last copy.
    pub fn effective_params(&self, id: PartId, index: usize) -> Option<InstanceParams> {
        let radial = self.radial(id)?;
        (index < usize::from(radial.count.max(1))).then(|| radial.effective(index))
    }

    /// The effective value of `dim` for copy `index` of part `id` (see [`Rocket::dim`] for the group's own): `None` if the part
    /// has no such dimension. Only a radial group's copies can differ; for every other part, and for dimensions that are never
    /// per-copy (position, angle), this is [`Rocket::dim`].
    pub fn instance_dim(&self, id: PartId, index: usize, dim: Dim) -> Option<f64> {
        let group = self.dim(id, dim)?;
        let own = self.radial(id).zip(RadialField::of_dim(dim)).and_then(|(radial, field)| radial.override_of(index, field));
        Some(own.unwrap_or(group))
    }

    /// Sets `dim` for copy `index` of radial group `id`: if the copy has unlinked that field its own value is set (limited as
    /// [`Rocket::set_dim`] limits it), otherwise the group's value is. `None` if the part has no such dimension.
    pub fn set_instance_dim(&mut self, id: PartId, index: usize, dim: Dim, value: f64) -> Option<DimEdit> {
        if self.pod_index(id).is_some() {
            return self.in_pod(id, |s| s.set_instance_dim(id, index, dim, value)).flatten();
        }
        let field = RadialField::of_dim(dim).filter(|f| self.radial(id).is_some_and(|r| r.is_overridden(index, *f)));
        let Some(field) = field else {
            return self.set_dim(id, dim, value);
        };
        let (applied, limited) = self.clamp_dim(id, dim, value)?;
        let radial = self.radials.iter_mut().find(|r| r.id == id)?;
        if let Some(o) = radial.overrides.iter_mut().find(|o| usize::from(o.index) == index && o.field == field) {
            o.value = applied;
        }
        Some(DimEdit { applied, limited })
    }

    /// *Unlink for this fin*: copy `index` of radial group `id` gets its own value of `field`, starting at the group's value, and
    /// edits to the group no longer reach it. Returns whether it was applied: the group has two or more copies, `index` is
    /// one of them, the field applies to the kind and is not already unlinked.
    pub fn unlink_field(&mut self, id: PartId, index: usize, field: RadialField) -> bool {
        if self.pod_index(id).is_some() {
            return self.in_pod(id, |s| s.unlink_field(id, index, field)).unwrap_or(false);
        }
        let Some(radial) = self.radials.iter_mut().find(|r| r.id == id) else {
            return false;
        };
        let Some(value) = radial.group_value(field) else {
            return false;
        };
        if radial.count < 2 || index >= usize::from(radial.count) || radial.is_overridden(index, field) {
            return false;
        }
        radial.overrides.push(FieldOverride { index: index as u8, field, value });
        radial.prune_overrides();
        true
    }

    /// *Relink field*: copy `index` follows the group's value of `field` again. Returns whether it had been unlinked.
    pub fn relink_field(&mut self, id: PartId, index: usize, field: RadialField) -> bool {
        if self.pod_index(id).is_some() {
            return self.in_pod(id, |s| s.relink_field(id, index, field)).unwrap_or(false);
        }
        let Some(radial) = self.radials.iter_mut().find(|r| r.id == id) else {
            return false;
        };
        let before = radial.overrides.len();
        radial.overrides.retain(|o| !(usize::from(o.index) == index && o.field == field));
        radial.overrides.len() != before
    }

    /// Relinks every field copy `index` of radial group `id` had unlinked. Returns whether it had any.
    pub fn relink_copy(&mut self, id: PartId, index: usize) -> bool {
        if self.pod_index(id).is_some() {
            return self.in_pod(id, |s| s.relink_copy(id, index)).unwrap_or(false);
        }
        let Some(radial) = self.radials.iter_mut().find(|r| r.id == id) else {
            return false;
        };
        let before = radial.overrides.len();
        radial.overrides.retain(|o| usize::from(o.index) != index);
        radial.overrides.len() != before
    }

    /// Gives radial part `id` the per-copy `overrides` (those that do not fit its count or kind are dropped), as when a part is
    /// duplicated. Returns whether `id` is a radial part.
    pub fn set_overrides(&mut self, id: PartId, overrides: Vec<FieldOverride>) -> bool {
        if self.pod_index(id).is_some() {
            return self.in_pod(id, |s| s.set_overrides(id, overrides)).unwrap_or(false);
        }
        let Some(radial) = self.radials.iter_mut().find(|r| r.id == id) else {
            return false;
        };
        radial.overrides = overrides;
        radial.prune_overrides();
        true
    }

    /// Detaches copy `index` of radial group `id` (see [`RadialPart::angles`]): it becomes a free part of one copy that keeps
    /// its current effective values (the group's with its own overrides) and its own angle, and is named `Fin B` (the letter of
    /// the copy). The rest stay linked as a group of `count - 1`, re-spaced evenly from the first remaining copy; their
    /// overrides stay with them. Returns the id of the new free part.
    ///
    /// Errors: [`RadialError::NoSuchRadial`] if `id` is not a radial group, `index` is out of range or the group has a single
    /// copy (there is nothing to detach from).
    pub fn detach_radial(&mut self, id: PartId, index: usize) -> Result<PartId, RadialError> {
        if self.pod_index(id).is_some() {
            return self.in_pod(id, |s| s.detach_radial(id, index)).ok_or(RadialError::NoSuchRadial)?;
        }
        // The copies of a pod share one body of parts with ids; they cannot leave the group.
        if self.is_pod(id) {
            return Err(RadialError::NoSuchRadial);
        }
        let group = self.radial(id).ok_or(RadialError::NoSuchRadial)?.clone();
        if group.count < 2 || index >= usize::from(group.count) {
            return Err(RadialError::NoSuchRadial);
        }
        let free = self.free_copy(&group, index);
        let free_id = free.id;
        let angles = group.angles();
        let count = f64::from(group.count);
        let first_left = if index == 0 { 1 } else { 0 };
        let position = self.radials.iter().position(|r| r.id == id).ok_or(RadialError::NoSuchRadial)?;
        let rest = &mut self.radials[position];
        rest.count -= 1;
        let left: Vec<f64> = angles.iter().enumerate().filter(|(i, _)| *i != index).map(|(_, a)| *a).collect();
        rest.set_angles(left);
        debug_assert_eq!(rest.angle, angles[first_left]);
        rest.mass_override = group.mass_override.map(|m| m * (count - 1.0) / count);
        // The copies keep the letters they had: a group that lost `A` reads `B, C`.
        let mut letters: Vec<u8> = (0..usize::from(group.count)).map(|i| group.copy_index(i) as u8).collect();
        letters.remove(index);
        rest.letters = letters;
        // The overrides' copy indices follow the copies' places in the group, so the ones after the one that left move up.
        rest.overrides.retain(|o| usize::from(o.index) != index);
        for o in rest.overrides.iter_mut().filter(|o| usize::from(o.index) > index) {
            o.index -= 1;
        }
        self.radials.insert(position + 1, free);
        Ok(free_id)
    }

    /// Splits radial group `id` into free parts, one per copy, each keeping the group's values (and its own overrides) at that
    /// moment and its own angle. The group's own id stays with copy A (the first); the others are new, named `Fin B`,
    /// `Fin C`, ... Returns every id in copy order (so `[id, ..]`); a group of one copy is returned unchanged as `[id]`.
    ///
    /// Errors: [`RadialError::NoSuchRadial`] if `id` is not a radial group.
    pub fn split_all_radial(&mut self, id: PartId) -> Result<Vec<PartId>, RadialError> {
        if self.pod_index(id).is_some() {
            return self.in_pod(id, |s| s.split_all_radial(id)).ok_or(RadialError::NoSuchRadial)?;
        }
        if self.is_pod(id) {
            return Err(RadialError::NoSuchRadial);
        }
        let group = self.radial(id).ok_or(RadialError::NoSuchRadial)?.clone();
        if group.count < 2 {
            return Ok(vec![id]);
        }
        let position = self.radials.iter().position(|r| r.id == id).ok_or(RadialError::NoSuchRadial)?;
        let mut ids = vec![id];
        let mut copies = Vec::new();
        for index in 1..usize::from(group.count) {
            let free = self.free_copy(&group, index);
            ids.push(free.id);
            copies.push(free);
        }
        let count = f64::from(group.count);
        let own = group.effective(0);
        let first = &mut self.radials[position];
        first.count = 1;
        first.kind = own.kind;
        first.cant = own.cant;
        first.overrides.clear();
        first.angle = group.angles()[0];
        first.instance_angles = None;
        first.letters.clear();
        first.name = Some(free_name(&group, 0));
        first.mass_override = group.mass_override.map(|m| m / count);
        for (offset, copy) in copies.into_iter().enumerate() {
            self.radials.insert(position + 1 + offset, copy);
        }
        Ok(ids)
    }

    /// Whether radial part `id` is a fin of a single copy (a free fin, whether detached from a group or placed alone).
    pub fn is_free_fin(&self, id: PartId) -> bool {
        self.radial(id).is_some_and(|r| r.count == 1 && r.kind.is_fin())
    }

    /// Whether free part `part` (a single copy) may rejoin radial `group`: both exist and are different, the part is one copy,
    /// they are the same family (fin with fin, lug with lug, button with button) and the group has room for another copy.
    pub fn can_rejoin(&self, part: PartId, group: PartId) -> bool {
        match (self.radial(part), self.radial(group)) {
            (Some(p), Some(g)) => {
                part != group && p.count == 1 && same_family(&p.kind, &g.kind) && g.count < MAX_COUNT && self.pod_index(part) == self.pod_index(group)
            }
            _ => false,
        }
    }

    /// The radial parts a free part could rejoin, groups of two or more first.
    pub fn rejoin_candidates(&self, part: PartId) -> Vec<PartId> {
        let mut ids: Vec<PartId> = self.ids().into_iter().filter(|g| self.radial(*g).is_some() && self.can_rejoin(part, *g)).collect();
        ids.sort_by_key(|g| std::cmp::Reverse(self.radial(*g).map_or(1, |r| r.count)));
        ids
    }

    /// Rejoins free part `part` to radial group `group`: the part leaves the document and the group gains a copy, re-spaced
    /// evenly from its start angle. With [`Winner::Group`] the group keeps its settings and overrides; with [`Winner::Part`] it
    /// takes the part's geometry, cant, position, material, finish, section and CG (its overrides go, and a mass override
    /// is shared out by copy, so the part's mass per copy wins). Returns the group's new count.
    ///
    /// Errors: [`RadialError::NoSuchRadial`] if either id is not a radial part, they are the same, the part has several copies
    /// or the two are not the same family; [`RadialError::BadCount`] if the group is full.
    pub fn rejoin_radial(&mut self, part: PartId, group: PartId, winner: Winner) -> Result<u8, RadialError> {
        if self.pod_index(part).is_some() || self.pod_index(group).is_some() {
            if self.pod_index(part) != self.pod_index(group) {
                return Err(RadialError::NoSuchRadial);
            }
            return self.in_pod(part, |s| s.rejoin_radial(part, group, winner)).ok_or(RadialError::NoSuchRadial)?;
        }
        let (p, g) = (self.radial(part).ok_or(RadialError::NoSuchRadial)?.clone(), self.radial(group).ok_or(RadialError::NoSuchRadial)?.clone());
        if part == group || p.count != 1 || !same_family(&p.kind, &g.kind) {
            return Err(RadialError::NoSuchRadial);
        }
        if g.count >= MAX_COUNT {
            return Err(RadialError::BadCount);
        }
        let count = g.count + 1;
        let per_copy = match winner {
            Winner::Group => g.mass_override.map(|m| m / f64::from(g.count)),
            Winner::Part => p.mass_override,
        };
        let moved = match winner {
            Winner::Part => Some(self.clamp_offset(p.parent, p.offset, &p.kind).unwrap_or(0.0)),
            Winner::Group => None,
        };
        let target = self.radials.iter_mut().find(|r| r.id == group).ok_or(RadialError::NoSuchRadial)?;
        if target.instance_angles.is_some() {
            let mut list = target.angles();
            list.push(p.angles()[0]);
            target.set_angles(list);
        }
        target.count = count;
        target.letters.clear();
        target.mass_override = per_copy.map(|m| m * f64::from(count));
        if winner == Winner::Part {
            let own = p.effective(0);
            target.kind = own.kind;
            target.cant = own.cant;
            target.parent = p.parent;
            target.offset = moved.unwrap_or(p.offset);
            target.material = p.material;
            target.finish = p.finish;
            target.cross_section = p.cross_section;
            target.cg_override = p.cg_override;
            target.overrides.clear();
        }
        self.radials.retain(|r| r.id != part);
        Ok(count)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::{NoseShape, PartKind};
    use std::f64::consts::TAU;

    fn finned(count: u8) -> (Rocket, PartId) {
        let mut r = Rocket::default();
        r.insert(0, PartKind::NoseCone { shape: NoseShape::TangentOgive, length: 0.2286, diameter: 0.0762 }).unwrap();
        let tube = r.insert(1, PartKind::BodyTube { length: 0.9, diameter: 0.0762 }).unwrap();
        let fins = r
            .attach_radial(tube, 0.78, 0.0, count, RadialKind::FinSet { root_chord: 0.12, tip_chord: 0.05, span: 0.0762, sweep: 0.06, thickness: 0.003 })
            .unwrap();
        (r, fins)
    }

    fn total_mass(r: &Rocket) -> f64 {
        r.radials().iter().map(|g| r.component_mass(g.id).unwrap().mass).sum()
    }

    #[test]
    fn letters_count_like_spreadsheet_columns() {
        assert_eq!(instance_letter(0), "A");
        assert_eq!(instance_letter(2), "C");
        assert_eq!(instance_letter(25), "Z");
        assert_eq!(instance_letter(26), "AA");
    }

    #[test]
    fn detach_frees_one_copy_at_its_own_angle_and_keeps_the_mass() {
        let (mut r, fins) = finned(3);
        let before = total_mass(&r);
        let free = r.detach_radial(fins, 1).unwrap();
        let (group, lone) = (r.radial(fins).unwrap(), r.radial(free).unwrap());
        assert_eq!((group.count, lone.count), (2, 1));
        // Copy B of three fins at 0, 120, 240 degrees leaves from 120.
        assert!((lone.angle - TAU / 3.0).abs() < 1e-12);
        assert_eq!(lone.name.as_deref(), Some("Fin B"));
        assert_eq!((lone.kind.clone(), lone.offset, lone.material), (group.kind.clone(), group.offset, group.material));
        assert!((total_mass(&r) - before).abs() < 1e-12, "{} vs {before}", total_mass(&r));
        // The rest is re-spaced from the first remaining copy.
        assert_eq!(group.angle, 0.0);
        // Detaching copy A starts the rest at B's angle.
        let (mut r, fins) = finned(4);
        r.detach_radial(fins, 0).unwrap();
        assert!((r.radial(fins).unwrap().angle - TAU / 4.0).abs() < 1e-12);
    }

    #[test]
    fn the_remaining_copies_keep_their_original_letters() {
        let (mut r, fins) = finned(3);
        let a = r.detach_radial(fins, 0).unwrap();
        assert_eq!(r.radial(a).unwrap().name.as_deref(), Some("Fin A"));
        let g = r.radial(fins).unwrap();
        assert_eq!((g.copy_index(0), g.copy_index(1)), (1, 2), "the group reads B, C");
        // The next copy to leave is named by its own letter, not its place in the shrunken group.
        let b = r.detach_radial(fins, 0).unwrap();
        assert_eq!(r.radial(b).unwrap().name.as_deref(), Some("Fin B"));
        // The lettering survives a save, and a new count starts over.
        let (mut r, fins) = finned(4);
        r.detach_radial(fins, 0).unwrap();
        assert_eq!(r.radial(fins).unwrap().letters, vec![1, 2, 3]);
        assert_eq!(Rocket::from_bytes(&r.to_bytes()).unwrap(), r);
        let kind = r.radial(fins).unwrap().kind.clone();
        assert!(r.set_radial(fins, kind, 2));
        assert!(r.radial(fins).unwrap().letters.is_empty());
    }

    #[test]
    fn detaching_keeps_the_remaining_copies_at_their_original_angles() {
        let (mut r, fins) = finned(3);
        let free = r.detach_radial(fins, 1).unwrap();
        let a = r.radial(fins).unwrap().angles();
        assert!((a[0] - 0.0).abs() < 1e-12 && (a[1] - 2.0 * TAU / 3.0).abs() < 1e-12, "{a:?}");
        // The overrides' copy indices and the JSON round trip survive.
        assert_eq!(Rocket::from_bytes(&r.to_bytes()).unwrap(), r);
        // Rotating the group rotates the remaining copies together.
        let g = r.radial(fins).unwrap().clone();
        r.move_radial(fins, g.parent, g.offset, 0.5).unwrap();
        let a = r.radial(fins).unwrap().angles();
        assert!((a[1] - a[0] - 2.0 * TAU / 3.0).abs() < 1e-9 && (a[0] - 0.5).abs() < 1e-9);
        r.move_radial(fins, g.parent, g.offset, 0.0).unwrap();
        // Rejoining the fin restores an evenly spaced group; a new count re-spaces.
        r.rejoin_radial(free, fins, Winner::Group).unwrap();
        let g = r.radial(fins).unwrap();
        assert!(g.instance_angles.is_none() && g.count == 3);
        let a = g.angles();
        assert!((a[1] - a[0] - TAU / 3.0).abs() < 1e-9 || (a[1] - a[0] + 2.0 * TAU / 3.0).abs() < 1e-9, "{a:?}");
        let (mut r, fins) = finned(4);
        r.detach_radial(fins, 3).unwrap();
        let kind = r.radial(fins).unwrap().kind.clone();
        assert!(r.set_radial(fins, kind, 2));
        assert!(r.radial(fins).unwrap().instance_angles.is_none());
    }

    #[test]
    fn detach_shares_out_a_group_mass_override() {
        let (mut r, fins) = finned(3);
        r.set_mass_override(fins, Some(0.09));
        let free = r.detach_radial(fins, 2).unwrap();
        assert!((r.radial(free).unwrap().mass_override.unwrap() - 0.03).abs() < 1e-12);
        assert!((r.radial(fins).unwrap().mass_override.unwrap() - 0.06).abs() < 1e-12);
        assert!((total_mass(&r) - 0.09).abs() < 1e-12);
    }

    #[test]
    fn split_all_makes_one_free_part_per_copy() {
        let (mut r, fins) = finned(4);
        let before = total_mass(&r);
        let ids = r.split_all_radial(fins).unwrap();
        assert_eq!(ids.len(), 4);
        assert_eq!(ids[0], fins);
        assert!(r.radials().iter().all(|g| g.count == 1));
        let angles: Vec<f64> = ids.iter().map(|id| r.radial(*id).unwrap().angle).collect();
        for (i, a) in angles.iter().enumerate() {
            assert!((a - TAU * i as f64 / 4.0).abs() < 1e-12);
        }
        let names: Vec<_> = ids.iter().map(|id| r.radial(*id).unwrap().name.clone().unwrap()).collect();
        assert_eq!(names, ["Fin A", "Fin B", "Fin C", "Fin D"]);
        assert!((total_mass(&r) - before).abs() < 1e-12);
        assert!(ids.iter().all(|id| r.is_free_fin(*id)));
    }

    #[test]
    fn single_copies_and_unknown_ids_are_refused_or_unchanged() {
        let (mut r, fins) = finned(1);
        assert_eq!(r.detach_radial(fins, 0), Err(RadialError::NoSuchRadial));
        assert_eq!(r.split_all_radial(fins), Ok(vec![fins]));
        assert_eq!(r.detach_radial(999, 0), Err(RadialError::NoSuchRadial));
        let (mut r, fins) = finned(3);
        assert_eq!(r.detach_radial(fins, 3), Err(RadialError::NoSuchRadial));
    }

    #[test]
    fn ids_stay_unique_and_the_document_round_trips() {
        let (mut r, fins) = finned(3);
        r.split_all_radial(fins).unwrap();
        let mut ids = r.ids();
        ids.sort_unstable();
        ids.dedup();
        assert_eq!(ids.len(), r.ids().len());
        let again = Rocket::from_bytes(&r.to_bytes()).unwrap();
        assert_eq!(again, r);
    }

    // ---- per-copy overrides ----------------------------------------------------------------------------------------------

    #[test]
    fn an_unlinked_field_belongs_to_one_copy_and_the_rest_stay_shared() {
        let (mut r, fins) = finned(3);
        assert!(r.unlink_field(fins, 1, RadialField::Cant));
        assert!(!r.unlink_field(fins, 1, RadialField::Cant), "already unlinked");
        assert!(!r.unlink_field(fins, 3, RadialField::Cant), "no copy D");
        assert!(!r.unlink_field(fins, 1, RadialField::Length), "a fin has no length");
        // It starts at the group's value; editing the copy leaves the group and the siblings alone.
        assert_eq!(r.radial(fins).unwrap().override_of(1, RadialField::Cant), Some(0.0));
        let edit = r.set_instance_dim(fins, 1, Dim::Cant, 0.7).unwrap();
        // The cant limit (15 degrees) applies to a copy exactly as to the group.
        assert!((edit.applied - 15f64.to_radians()).abs() < 1e-12 && edit.limited.is_some());
        let edit = r.set_instance_dim(fins, 1, Dim::Cant, 0.0175).unwrap();
        assert_eq!(edit.limited, None);
        let g = r.radial(fins).unwrap();
        assert_eq!((g.cant, g.effective(0).cant, g.effective(2).cant), (0.0, 0.0, 0.0));
        assert_eq!(g.effective(1).cant, 0.0175);
        assert_eq!(r.instance_dim(fins, 1, Dim::Cant), Some(0.0175));
        assert_eq!(r.instance_dim(fins, 0, Dim::Cant), Some(0.0));
        // A field that is not overridden edits the group, and so reaches every copy that follows it.
        r.set_instance_dim(fins, 1, Dim::Span, 0.09).unwrap();
        assert_eq!(r.radial(fins).unwrap().effective(1).kind.fin_span(), Some(0.09));
        assert_eq!(r.radial(fins).unwrap().effective(0).kind.fin_span(), Some(0.09));
        assert_eq!(r.radial(fins).unwrap().override_summary(), "B: cant");
        // Relinking returns it to the group's value.
        assert!(r.relink_field(fins, 1, RadialField::Cant) && !r.relink_field(fins, 1, RadialField::Cant));
        assert_eq!(r.radial(fins).unwrap().effective(1).cant, 0.0);
        assert!(!r.radial(fins).unwrap().has_overrides());
    }

    #[test]
    fn a_freeform_fin_or_a_single_copy_cannot_unlink_what_it_does_not_have() {
        let (mut r, fins) = finned(1);
        assert!(!r.unlink_field(fins, 0, RadialField::Cant), "nothing to unlink from");
        let (mut r2, fins2) = finned(3);
        r2.set_radial(fins2, RadialKind::FreeformFin { points: vec![(0.0, 0.0), (0.04, 0.05), (0.1, 0.0)], thickness: 0.003 }, 3);
        assert!(!r2.unlink_field(fins2, 0, RadialField::Span) && r2.unlink_field(fins2, 0, RadialField::Thickness));
        // Turning the fin into an ellipse drops the overrides of fields that no longer exist (a tip chord).
        r.set_radial(fins, RadialKind::FinSet { root_chord: 0.12, tip_chord: 0.05, span: 0.0762, sweep: 0.06, thickness: 0.003 }, 3);
        assert!(r.unlink_field(fins, 2, RadialField::TipChord) && r.unlink_field(fins, 2, RadialField::Cant));
        r.set_radial(fins, RadialKind::EllipticalFin { root_chord: 0.12, span: 0.0762, thickness: 0.003 }, 3);
        assert_eq!(r.radial(fins).unwrap().overridden_fields(2), [RadialField::Cant]);
    }

    #[test]
    fn mass_follows_the_overridden_copy() {
        let (mut r, fins) = finned(3);
        let before = total_mass(&r);
        r.unlink_field(fins, 2, RadialField::Span);
        // Nothing has changed yet: the copy holds the group's own value.
        assert!((total_mass(&r) - before).abs() < 1e-15);
        r.set_instance_dim(fins, 2, Dim::Span, 0.1524).unwrap();
        let one = |span: f64| {
            let mut t = r.clone();
            t.relink_field(fins, 2, RadialField::Span);
            t.set_dim(fins, Dim::Span, span).unwrap();
            t.component_mass(fins).unwrap().mass / 3.0
        };
        let want = 2.0 * one(0.0762) + one(0.1524);
        let got = r.component_mass(fins).unwrap().mass;
        assert!((got - want).abs() < 1e-12 * want, "{got} vs {want}");
        assert!(got > before);
    }

    #[test]
    fn detach_keeps_the_effective_values_and_reindexes_the_rest() {
        let (mut r, fins) = finned(4);
        r.unlink_field(fins, 1, RadialField::Cant);
        r.set_instance_dim(fins, 1, Dim::Cant, 0.05).unwrap();
        r.unlink_field(fins, 3, RadialField::Span);
        r.set_instance_dim(fins, 3, Dim::Span, 0.1).unwrap();
        let free = r.detach_radial(fins, 1).unwrap();
        let (lone, rest) = (r.radial(free).unwrap(), r.radial(fins).unwrap());
        assert_eq!(lone.cant, 0.05);
        assert!(!lone.has_overrides());
        // D becomes C of the three that are left.
        assert_eq!((rest.count, rest.overrides.len()), (3, 1));
        assert_eq!(rest.effective(2).kind.fin_span(), Some(0.1));
        assert_eq!(rest.override_summary(), "C: height");
        assert_eq!(rest.effective(0).cant, 0.0);
        // Split all freezes each copy's effective values.
        let ids = r.split_all_radial(fins).unwrap();
        let spans: Vec<_> = ids.iter().map(|id| r.radial(*id).unwrap().kind.fin_span().unwrap()).collect();
        assert_eq!(spans, [0.0762, 0.0762, 0.1]);
        assert!(r.radials().iter().all(|g| !g.has_overrides()));
    }

    #[test]
    fn rejoin_lets_the_group_or_the_part_win() {
        let (mut r, fins) = finned(3);
        let free = r.detach_radial(fins, 1).unwrap();
        r.set_dim(free, Dim::Span, 0.09).unwrap();
        r.set_dim(free, Dim::Cant, 0.02).unwrap();
        assert!(r.can_rejoin(free, fins) && !r.can_rejoin(fins, free) && !r.can_rejoin(free, free));
        assert_eq!(r.rejoin_candidates(free), [fins]);
        let mut by_group = r.clone();
        assert_eq!(by_group.rejoin_radial(free, fins, Winner::Group), Ok(3));
        assert!(by_group.radial(free).is_none());
        let g = by_group.radial(fins).unwrap();
        assert_eq!((g.count, g.kind.fin_span(), g.cant), (3, Some(0.0762), 0.0));
        let mut by_part = r.clone();
        assert_eq!(by_part.rejoin_radial(free, fins, Winner::Part), Ok(3));
        let g = by_part.radial(fins).unwrap();
        assert_eq!((g.count, g.kind.fin_span(), g.cant), (3, Some(0.09), 0.02));
        assert!(by_part.radial(free).is_none());
        // A lug is not a fin, and a full group takes no more.
        let tube = r.parts()[1].id;
        let lug = r.attach_radial(tube, 0.1, 1.0, 1, RadialKind::LaunchLug { length: 0.05, diameter: 0.01 }).unwrap();
        assert_eq!(r.rejoin_radial(lug, fins, Winner::Group), Err(RadialError::NoSuchRadial));
        let (mut full, big) = finned(MAX_COUNT);
        let other = full.attach_radial(full.parts()[1].id, 0.5, 0.0, 1, full.radial(big).unwrap().kind.clone()).unwrap();
        assert_eq!(full.rejoin_radial(other, big, Winner::Group), Err(RadialError::BadCount));
    }

    #[test]
    fn rejoin_keeps_the_total_mass_when_the_group_wins() {
        let (mut r, fins) = finned(3);
        r.set_mass_override(fins, Some(0.09));
        let free = r.detach_radial(fins, 2).unwrap();
        let before = total_mass(&r);
        r.rejoin_radial(free, fins, Winner::Group).unwrap();
        assert!((total_mass(&r) - before).abs() < 1e-12, "{} vs {before}", total_mass(&r));
    }

    #[test]
    fn overrides_survive_the_json_round_trip_and_old_documents_load() {
        let (mut r, fins) = finned(3);
        r.unlink_field(fins, 1, RadialField::Cant);
        r.set_instance_dim(fins, 1, Dim::Cant, 0.02).unwrap();
        assert_eq!(Rocket::from_bytes(&r.to_bytes()).unwrap(), r);
        // A document with no overrides never mentions them (the field defaults), so older files load unchanged.
        let (plain, _) = finned(3);
        assert!(!crate::format::cbor_text(&plain).contains("overrides"));
    }

    #[test]
    fn removing_the_count_drops_the_overrides() {
        let (mut r, fins) = finned(3);
        r.unlink_field(fins, 2, RadialField::Cant);
        let kind = r.radial(fins).unwrap().kind.clone();
        assert!(r.set_radial(fins, kind.clone(), 3) && r.radial(fins).unwrap().has_overrides());
        // Fewer copies drop the overrides of copies that no longer exist.
        assert!(r.set_radial(fins, kind.clone(), 2));
        assert!(!r.radial(fins).unwrap().has_overrides());
        r.unlink_field(fins, 1, RadialField::Cant);
        // A different count re-spaces the group, and its overrides would be lost.
        assert!(r.set_radial(fins, kind, 4));
        assert!(!r.radial(fins).unwrap().has_overrides());
    }
}
