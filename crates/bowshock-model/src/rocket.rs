//! The [`Rocket`] document: the axial stack, radial parts, internal components and configurations, and the rules that keep
//! them consistent as parts are inserted, moved, resized and removed.

use std::fmt;

use serde::{Deserialize, Serialize};

use crate::config::{ConfigId, Configuration};
use crate::geometry::{FinSection, Part, PartKind, RadialKind, RadialPart, MAX_COUNT};
use crate::internal::{InternalKind, InternalPart, PartRef, AUTO_FILL};
use crate::material::{Finish, MaterialId, MaterialKind};
use crate::PartId;

/// Why a radial part could not be placed.
#[derive(Debug, PartialEq, Eq)]
pub enum RadialError {
    /// The parent is not an axial part of the rocket.
    NoSuchParent,
    /// The count is zero or more than [`MAX_COUNT`].
    BadCount,
    /// No radial part has this id (or the id is in use).
    NoSuchRadial,
}

impl fmt::Display for RadialError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            RadialError::NoSuchParent => "the parent is not part of the rocket",
            RadialError::BadCount => "the symmetry count must be from 1 to 16",
            RadialError::NoSuchRadial => "no such radial part",
        })
    }
}

impl std::error::Error for RadialError {}

/// Why an internal part could not be placed.
#[derive(Debug, PartialEq, Eq)]
pub enum InternalError {
    /// The parent is not an axial part of the rocket.
    NoSuchParent,
    /// No internal part has this id (or the id is in use).
    NoSuchInternal,
}

impl fmt::Display for InternalError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            InternalError::NoSuchParent => "the parent is not part of the rocket",
            InternalError::NoSuchInternal => "no such internal part",
        })
    }
}

impl std::error::Error for InternalError {}

/// Why an insertion was refused.
#[derive(Debug, PartialEq, Eq)]
pub enum InsertError {
    /// A nose cone may only be placed first.
    NoseNotFirst,
    /// The rocket already has a nose cone (or something would go before it).
    SecondNose,
    /// Index beyond the number of parts.
    IndexOutOfRange,
    /// A part with this id is already in the rocket.
    DuplicateId,
}

impl fmt::Display for InsertError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            InsertError::NoseNotFirst => "a nose cone can only go at the fore end",
            InsertError::SecondNose => "the rocket already has a nose cone",
            InsertError::IndexOutOfRange => "attach index out of range",
            InsertError::DuplicateId => "a part with this id is already in the rocket",
        })
    }
}

impl std::error::Error for InsertError {}

/// The size of an internal part with its automatic dimensions resolved against the parent, from
/// [`Rocket::resolved`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Resolved {
    /// Length along the axis.
    pub length: f64,
    /// Outer diameter (of the packed bundle for recovery gear and mass).
    pub outer_diameter: f64,
    /// Inner diameter: the hole of a ring or the bore of a tube, 0 for solid parts.
    pub inner_diameter: f64,
    /// The outer diameter follows the parent (shown as `AUTO`).
    pub outer_auto: bool,
    /// The inner diameter follows the part it sits on (shown as `AUTO`).
    pub inner_auto: bool,
}

/// A rocket document: an airframe stack of parts ordered fore to aft with radial and internal parts attached to them, and
/// the motor configurations. All lengths are SI metres, masses kilograms, angles radians.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Rocket {
    /// Name of the rocket.
    #[serde(default)]
    pub name: String,
    /// Who designed it.
    #[serde(default)]
    pub designer: String,
    pub(crate) parts: Vec<Part>,
    pub(crate) radials: Vec<RadialPart>,
    #[serde(default)]
    pub(crate) internals: Vec<InternalPart>,
    pub(crate) next_id: PartId,
    pub(crate) configs: Vec<Configuration>,
    pub(crate) active_config: ConfigId,
    pub(crate) next_config_id: ConfigId,
    /// Free-text notes by part id (the inspector's Notes section). Empty notes are not stored.
    #[serde(default, skip_serializing_if = "std::collections::BTreeMap::is_empty")]
    pub(crate) notes: std::collections::BTreeMap<PartId, String>,
    /// The stored flights of the Simulate screen (name, motors, launch conditions). Never empty.
    #[serde(default = "crate::flights::default_flights")]
    pub(crate) flights: Vec<crate::flights::FlightDef>,
    /// Formulas that compute dimensions of parts from dimensions of other parts (see [`crate::Formula`]).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub(crate) formulas: Vec<crate::expr::Formula>,
    /// The stability thresholds (gauge, warnings, Analyze lines). Not written while they are the defaults.
    #[serde(default, skip_serializing_if = "crate::bands::is_default")]
    pub(crate) bands: crate::bands::StabilityBands,
    /// The Simulate screen's plots (which data sources each panel draws). Not written while they are the defaults.
    #[serde(default, skip_serializing_if = "crate::flights::plots_are_default")]
    pub(crate) plots: crate::flights::PlotLayout,
    /// Solved CFD drag tables of the Analyze screen (a cache: not part of the rocket's equality, see [`crate::cfd`]).
    #[serde(default, skip_serializing_if = "crate::cfd::CfdStore::is_empty")]
    pub(crate) cfd: crate::cfd::CfdStore,
}

impl Default for Rocket {
    fn default() -> Self {
        Rocket {
            name: String::new(),
            designer: String::new(),
            parts: Vec::new(),
            radials: Vec::new(),
            internals: Vec::new(),
            next_id: 0,
            configs: vec![Configuration::new(0, "")],
            active_config: 0,
            next_config_id: 1,
            notes: std::collections::BTreeMap::new(),
            flights: crate::flights::default_flights(),
            formulas: Vec::new(),
            bands: crate::bands::StabilityBands::default(),
            plots: crate::flights::PlotLayout::default(),
            cfd: crate::cfd::CfdStore::default(),
        }
    }
}

impl Rocket {
    /// Parts, fore to aft.
    pub fn parts(&self) -> &[Part] {
        &self.parts
    }

    /// Station of each part's fore face from the fore end, with the total length last (`len = parts + 1`).
    pub fn stations(&self) -> Vec<f64> {
        let mut out = Vec::with_capacity(self.parts.len() + 1);
        let mut z = 0.0;
        for p in &self.parts {
            out.push(z);
            z += p.kind.length();
        }
        out.push(z);
        out
    }

    /// Total length.
    pub fn length(&self) -> f64 {
        self.parts.iter().map(|p| p.kind.length()).sum()
    }

    /// Largest diameter, `None` when empty.
    pub fn reference_diameter(&self) -> Option<f64> {
        self.parts.iter().map(|p| p.kind.fore_diameter().max(p.kind.aft_diameter())).reduce(f64::max)
    }

    fn has_nose(&self) -> bool {
        matches!(self.parts.first(), Some(Part { kind: PartKind::NoseCone { .. }, .. }))
    }

    fn check(&self, index: usize, kind: &PartKind) -> Result<(), InsertError> {
        if index > self.parts.len() {
            return Err(InsertError::IndexOutOfRange);
        }
        let is_nose = matches!(kind, PartKind::NoseCone { .. });
        if is_nose {
            if index != 0 {
                return Err(InsertError::NoseNotFirst);
            }
            if self.has_nose() {
                return Err(InsertError::SecondNose);
            }
        } else if index == 0 && self.has_nose() {
            return Err(InsertError::SecondNose);
        }
        Ok(())
    }

    /// Attach node indices where `kind` may be inserted, each with its station. Node `i` is the fore
    /// face of part `i`; node `parts.len()` is the aft face of the last part.
    pub fn nodes_for(&self, kind: &PartKind) -> Vec<(usize, f64)> {
        let stations = self.stations();
        (0..=self.parts.len()).filter(|&i| self.check(i, kind).is_ok()).map(|i| (i, stations[i])).collect()
    }

    /// The kind as it would be inserted at `index`, with diameters matched to its neighbours.
    pub fn matched(&self, index: usize, kind: &PartKind) -> PartKind {
        let before = index.checked_sub(1).and_then(|i| self.parts.get(i));
        let after = self.parts.get(index);
        let mut out = kind.clone();
        match &mut out {
            PartKind::NoseCone { diameter, .. } | PartKind::BodyTube { diameter, .. } => {
                if let Some(b) = before {
                    *diameter = b.kind.aft_diameter();
                } else if let Some(a) = after {
                    let d = a.kind.fore_diameter();
                    if d > 0.0 {
                        *diameter = d;
                    }
                }
            }
            PartKind::Transition { fore_diameter, aft_diameter, .. } => {
                let ratio = if *fore_diameter > 0.0 { *aft_diameter / *fore_diameter } else { 1.0 };
                if let Some(b) = before {
                    *fore_diameter = b.kind.aft_diameter();
                }
                if let Some(a) = after {
                    *aft_diameter = a.kind.fore_diameter();
                } else if before.is_some() {
                    *aft_diameter = *fore_diameter * ratio;
                }
            }
        }
        out
    }

    fn fresh_id(&mut self) -> PartId {
        let id = self.next_id;
        self.next_id += 1;
        id
    }

    /// Inserts `matched(index, kind)` at `index` and returns its id. The part gets the default material, finish and wall of
    /// its kind ([`Part::new`]).
    pub fn insert(&mut self, index: usize, kind: PartKind) -> Result<PartId, InsertError> {
        self.check(index, &kind)?;
        let kind = self.matched(index, &kind);
        let id = self.fresh_id();
        self.parts.insert(index, Part::new(id, kind));
        Ok(id)
    }

    /// Radial parts, in the order they were placed.
    pub fn radials(&self) -> &[RadialPart] {
        &self.radials
    }

    /// The radial part with `id`.
    pub fn radial(&self, id: PartId) -> Option<&RadialPart> {
        self.radials.iter().find(|r| r.id == id).or_else(|| self.pod_radials().find(|r| r.id == id))
    }

    /// Where a radial part of `kind` may start on `parent`: `offset` limited so enough of it
    /// ([`min_attached`](RadialKind::min_attached)) lies on the parent's length, which lets a fin trail past the aft end.
    /// `None` if `parent` is not an axial part.
    pub fn clamp_offset(&self, parent: PartId, offset: f64, kind: &RadialKind) -> Option<f64> {
        let length = self.part(parent)?.kind.length();
        Some(offset.clamp(0.0, (length - kind.min_attached().min(length)).max(0.0)))
    }

    /// Radius of the outer surface of axial part `parent` at `offset` from its fore face (interpolating the profile).
    pub fn surface_radius(&self, parent: PartId, offset: f64) -> Option<f64> {
        let profile = self.part(parent)?.kind.profile();
        let z = offset.clamp(0.0, profile.last()?.0);
        profile.windows(2).find(|w| z <= w[1].0).map(|w| {
            let t = if w[1].0 > w[0].0 { (z - w[0].0) / (w[1].0 - w[0].0) } else { 0.0 };
            w[0].1 + (w[1].1 - w[0].1) * t
        })
    }

    /// Inner diameter of the shell of axial part `parent` at `offset` from its fore face: the outer diameter there less twice
    /// the wall thickness (what auto-sized internal parts follow), never negative.
    pub fn inner_diameter_at(&self, parent: PartId, offset: f64) -> Option<f64> {
        Some(self.part(parent)?.inner_diameter_at(offset))
    }

    /// Station of the fore edge of a radial part.
    pub fn radial_station(&self, radial: &RadialPart) -> Option<f64> {
        if let Some(index) = self.index_of(radial.parent) {
            return Some(self.stations()[index] + radial.offset);
        }
        // On a part of a pod: its station within the pod, on the pod's.
        let (z0, sub) = self.scratch_of(radial.id)?;
        Some(z0 + sub.radial_station(radial)?)
    }

    /// Surface radius a radial part stands on: the smallest over its footprint, so it meets the surface along its whole
    /// length on a slope.
    pub fn radial_base_radius(&self, radial: &RadialPart) -> Option<f64> {
        let (a, b) = (self.surface_radius(radial.parent, radial.offset)?, self.surface_radius(radial.parent, radial.offset + radial.kind.extent())?);
        Some(a.min(b))
    }

    fn check_radial(&self, parent: PartId, count: u8) -> Result<(), RadialError> {
        if self.part(parent).is_none() {
            return Err(RadialError::NoSuchParent);
        }
        if count == 0 || count > MAX_COUNT {
            return Err(RadialError::BadCount);
        }
        Ok(())
    }

    /// Places `count` copies of a radial part on axial part `parent`, the first at `angle`. The offset is limited so it lies
    /// within the parent (a fin may trail past its aft end). The group gets the default material of its kind
    /// ([`RadialPart::new`]).
    pub fn attach_radial(&mut self, parent: PartId, offset: f64, angle: f64, count: u8, mut kind: RadialKind) -> Result<PartId, RadialError> {
        if self.pod_index(parent).is_some() {
            // A part of a pod: the pod holds it. (Pods do not nest.)
            if kind.is_pod() {
                return Err(RadialError::NoSuchParent);
            }
            return self.in_pod(parent, |s| s.attach_radial(parent, offset, angle, count, kind)).ok_or(RadialError::NoSuchParent)?;
        }
        self.check_radial(parent, count)?;
        let offset = self.clamp_offset(parent, offset, &kind).ok_or(RadialError::NoSuchParent)?;
        let id = self.fresh_id();
        if let RadialKind::Pod { body, .. } = &mut kind {
            // The pod's parts get ids of this rocket.
            self.next_id = body.reid(self.next_id);
        }
        let kind = kind.limited_at(offset);
        self.radials.push(RadialPart::new(id, kind, parent, offset, angle.rem_euclid(std::f64::consts::TAU), count));
        Ok(id)
    }

    /// Moves radial part `id` onto `parent` at `offset` and `angle`.
    pub fn move_radial(&mut self, id: PartId, parent: PartId, offset: f64, angle: f64) -> Result<(), RadialError> {
        if self.pod_index(id).is_some() {
            // A part of a pod stays in its pod: it can move along the pod's parts only.
            if self.pod_index(parent) != self.pod_index(id) {
                return Err(RadialError::NoSuchParent);
            }
            return self.in_pod(id, |s| s.move_radial(id, parent, offset, angle)).ok_or(RadialError::NoSuchRadial)?;
        }
        if self.radial(id).is_some_and(|r| r.kind.is_pod()) && self.index_of(parent).is_none() {
            return Err(RadialError::NoSuchParent);
        }
        let existing = self.radial(id).ok_or(RadialError::NoSuchRadial)?;
        let count = existing.count;
        self.check_radial(parent, count)?;
        let offset = self.clamp_offset(parent, offset, &existing.kind).ok_or(RadialError::NoSuchParent)?;
        let radial = self.radials.iter_mut().find(|r| r.id == id).ok_or(RadialError::NoSuchRadial)?;
        let tau = std::f64::consts::TAU;
        let delta = angle - radial.angles().first().copied().unwrap_or(radial.angle);
        if let Some(list) = radial.instance_angles.as_mut() {
            list.iter_mut().for_each(|a| *a = (*a + delta).rem_euclid(tau));
        }
        (radial.parent, radial.offset, radial.angle) = (parent, offset, angle.rem_euclid(tau));
        radial.kind = radial.kind.limited_at(offset);
        Ok(())
    }

    /// Replaces the geometry and copy count of radial part `id`; the kind of part must stay the same, except that a fin may
    /// change planform (trapezoidal, elliptical, freeform). Its offset is limited to keep it within the parent and a forward
    /// sweep to keep the tip behind the parent's fore face (see [`RadialKind::limited_at`]). A freeform fin needs at least
    /// three points. Returns whether it was applied.
    pub fn set_radial(&mut self, id: PartId, kind: RadialKind, count: u8) -> bool {
        if self.pod_index(id).is_some() {
            return self.in_pod(id, |s| s.set_radial(id, kind, count)).unwrap_or(false);
        }
        let Some(radial) = self.radial(id) else {
            return false;
        };
        let same_family = std::mem::discriminant(&radial.kind) == std::mem::discriminant(&kind) || (radial.kind.is_fin() && kind.is_fin());
        let degenerate = matches!(&kind, RadialKind::FreeformFin { points, .. } if points.len() < 3);
        if !same_family || degenerate || count == 0 || count > MAX_COUNT {
            return false;
        }
        let offset = self.clamp_offset(radial.parent, radial.offset, &kind).unwrap_or(0.0);
        if let Some(radial) = self.radials.iter_mut().find(|r| r.id == id) {
            // Re-spacing a group (a new count) loses the overrides of its copies; a new planform loses the fields it lacks.
            if radial.count != count {
                radial.overrides.clear();
                radial.instance_angles = None;
                radial.letters.clear();
            }
            (radial.kind, radial.count, radial.offset) = (kind.limited_at(offset), count, offset);
            radial.prune_overrides();
        }
        true
    }

    /// Removes radial part `id`, returning it.
    pub fn remove_radial(&mut self, id: PartId) -> Option<RadialPart> {
        if self.pod_index(id).is_some() {
            return self.in_pod(id, |s| s.remove_radial(id)).flatten();
        }
        let i = self.radials.iter().position(|r| r.id == id)?;
        Some(self.radials.remove(i))
    }

    /// Puts a removed radial part back as it was, if its parent is still there (its offset is limited to the parent). Overrides
    /// that no longer fit its count or kind are dropped.
    pub fn restore_radial(&mut self, mut radial: RadialPart) -> Result<(), RadialError> {
        if self.pod_index(radial.parent).is_some() {
            if radial.kind.is_pod() {
                return Err(RadialError::NoSuchParent);
            }
            let parent = radial.parent;
            return self.in_pod(parent, |s| s.restore_radial(radial)).ok_or(RadialError::NoSuchParent)?;
        }
        if radial.kind.is_pod() && self.index_of(radial.parent).is_none() {
            return Err(RadialError::NoSuchParent);
        }
        if let Some(body) = radial.kind.pod_body() {
            if body.ids().any(|id| self.part(id).is_some() || self.radial(id).is_some() || self.internal(id).is_some()) {
                return Err(RadialError::NoSuchRadial);
            }
            self.next_id = self.next_id.max(body.ids().max().map_or(0, |m| m + 1));
        }
        self.check_radial(radial.parent, radial.count)?;
        if self.radial(radial.id).is_some() {
            return Err(RadialError::NoSuchRadial);
        }
        radial.offset = self.clamp_offset(radial.parent, radial.offset, &radial.kind).ok_or(RadialError::NoSuchParent)?;
        radial.kind = radial.kind.limited_at(radial.offset);
        radial.prune_overrides();
        self.next_id = self.next_id.max(radial.id + 1);
        self.radials.push(radial);
        Ok(())
    }

    /// Takes every radial part off axial part `parent` (to carry them while it is moved) and returns them.
    pub fn take_radials(&mut self, parent: PartId) -> Vec<RadialPart> {
        if self.pod_index(parent).is_some() {
            return self.in_pod(parent, |s| s.take_radials(parent)).unwrap_or_default();
        }
        let (taken, kept) = std::mem::take(&mut self.radials).into_iter().partition(|r| r.parent == parent);
        self.radials = kept;
        taken
    }

    /// Puts radial parts taken with [`take_radials`](Self::take_radials) back on their parent.
    pub fn restore_radials(&mut self, radials: Vec<RadialPart>) {
        for radial in radials {
            let _ = self.restore_radial(radial);
        }
    }

    /// Limits the offsets of radial parts on `parent` to its (new) length.
    pub(crate) fn clamp_radials(&mut self, parent: PartId) {
        for i in 0..self.radials.len() {
            if self.radials[i].parent == parent {
                let r = &self.radials[i];
                if let Some(offset) = self.clamp_offset(parent, r.offset, &r.kind) {
                    self.radials[i].offset = offset;
                    self.radials[i].kind = self.radials[i].kind.limited_at(offset);
                }
            }
        }
    }

    // ---- internal parts -------------------------------------------------------------------------------------------

    /// Internal parts, in the order they were placed.
    pub fn internals(&self) -> &[InternalPart] {
        &self.internals
    }

    /// The internal part with `id`.
    pub fn internal(&self, id: PartId) -> Option<&InternalPart> {
        self.internals.iter().find(|p| p.id == id).or_else(|| self.pod_internals().find(|p| p.id == id))
    }

    /// The internal parts inside axial part `parent`.
    pub fn internals_of(&self, parent: PartId) -> impl Iterator<Item = &InternalPart> {
        self.internals.iter().chain(self.pod_internals()).filter(move |p| p.parent == parent)
    }

    /// Where an internal part `extent` long may start in `parent`: `offset` limited so it lies wholly within the parent's
    /// length (the drag stops at the tube's walls). `None` if `parent` is not an axial part.
    pub fn clamp_internal_offset(&self, parent: PartId, offset: f64, extent: f64) -> Option<f64> {
        let length = self.part(parent)?.kind.length();
        Some(offset.clamp(0.0, (length - extent).max(0.0)))
    }

    /// Places an internal part in axial part `parent` with its fore edge `offset` behind the parent's fore face (limited so
    /// the part lies within the parent). It gets the default material of its kind ([`InternalPart::new`]).
    pub fn attach_internal(&mut self, parent: PartId, offset: f64, kind: InternalKind) -> Result<PartId, InternalError> {
        if self.pod_index(parent).is_some() {
            return self.in_pod(parent, |s| s.attach_internal(parent, offset, kind)).ok_or(InternalError::NoSuchParent)?;
        }
        let offset = self.clamp_internal_offset(parent, offset, kind.extent()).ok_or(InternalError::NoSuchParent)?;
        let id = self.fresh_id();
        self.internals.push(InternalPart::new(id, kind, parent, offset));
        Ok(id)
    }

    /// Moves internal part `id` into `parent` at `offset` (limited to keep it within the parent).
    pub fn move_internal(&mut self, id: PartId, parent: PartId, offset: f64) -> Result<(), InternalError> {
        if self.pod_index(id).is_some() {
            if self.pod_index(parent) != self.pod_index(id) {
                return Err(InternalError::NoSuchParent);
            }
            return self.in_pod(id, |s| s.move_internal(id, parent, offset)).ok_or(InternalError::NoSuchInternal)?;
        }
        if self.pod_index(parent).is_some() {
            return Err(InternalError::NoSuchParent);
        }
        let extent = self.internal(id).ok_or(InternalError::NoSuchInternal)?.kind.extent();
        let offset = self.clamp_internal_offset(parent, offset, extent).ok_or(InternalError::NoSuchParent)?;
        let part = self.internals.iter_mut().find(|p| p.id == id).ok_or(InternalError::NoSuchInternal)?;
        (part.parent, part.offset) = (parent, offset);
        Ok(())
    }

    /// Replaces the geometry of internal part `id`; it must stay the same kind of part. Its offset is limited to keep it
    /// within the parent. Turning a motor mount into a plain tube drops the motors configured for it. Returns whether it
    /// was applied.
    pub fn set_internal_kind(&mut self, id: PartId, kind: InternalKind) -> bool {
        if self.pod_index(id).is_some() {
            return self.in_pod(id, |s| s.set_internal_kind(id, kind)).unwrap_or(false);
        }
        let Some(part) = self.internal(id) else {
            return false;
        };
        if std::mem::discriminant(&part.kind) != std::mem::discriminant(&kind) {
            return false;
        }
        let offset = self.clamp_internal_offset(part.parent, part.offset, kind.extent()).unwrap_or(0.0);
        if let Some(part) = self.internals.iter_mut().find(|p| p.id == id) {
            (part.kind, part.offset) = (kind, offset);
        }
        self.prune_configs();
        true
    }

    /// Removes internal part `id`, returning it. Motors configured for it are dropped from every configuration.
    pub fn remove_internal(&mut self, id: PartId) -> Option<InternalPart> {
        if self.pod_index(id).is_some() {
            return self.in_pod(id, |s| s.remove_internal(id)).flatten();
        }
        let i = self.internals.iter().position(|p| p.id == id)?;
        let part = self.internals.remove(i);
        self.prune_configs();
        Some(part)
    }

    /// Puts a removed internal part back as it was, if its parent is still there (its offset is limited to the parent).
    pub fn restore_internal(&mut self, mut part: InternalPart) -> Result<(), InternalError> {
        if self.pod_index(part.parent).is_some() {
            let parent = part.parent;
            return self.in_pod(parent, |s| s.restore_internal(part)).ok_or(InternalError::NoSuchParent)?;
        }
        if self.internal(part.id).is_some() {
            return Err(InternalError::NoSuchInternal);
        }
        part.offset = self.clamp_internal_offset(part.parent, part.offset, part.kind.extent()).ok_or(InternalError::NoSuchParent)?;
        self.next_id = self.next_id.max(part.id + 1);
        self.internals.push(part);
        Ok(())
    }

    /// Takes every internal part out of axial part `parent` (to carry them while it is moved) and returns them. Configurations
    /// keep their motors, so [`restore_internals`](Self::restore_internals) brings everything back.
    pub fn take_internals(&mut self, parent: PartId) -> Vec<InternalPart> {
        if self.pod_index(parent).is_some() {
            return self.in_pod(parent, |s| s.take_internals(parent)).unwrap_or_default();
        }
        let (taken, kept) = std::mem::take(&mut self.internals).into_iter().partition(|p| p.parent == parent);
        self.internals = kept;
        taken
    }

    /// Puts internal parts taken with [`take_internals`](Self::take_internals) back on their parent.
    pub fn restore_internals(&mut self, parts: Vec<InternalPart>) {
        for part in parts {
            let _ = self.restore_internal(part);
        }
    }

    /// Limits the offsets of internal parts in `parent` to its (new) length.
    pub(crate) fn clamp_internals(&mut self, parent: PartId) {
        for i in 0..self.internals.len() {
            if self.internals[i].parent == parent {
                if let Some(offset) = self.clamp_internal_offset(parent, self.internals[i].offset, self.internals[i].kind.extent()) {
                    self.internals[i].offset = offset;
                }
            }
        }
    }

    /// The inner tube of `parent` that contains `z` (a position from the parent's fore face), other than `except`.
    fn inner_tube_at(&self, parent: PartId, z: f64, except: PartId) -> Option<&InternalPart> {
        self.internals_of(parent)
            .find(|p| p.id != except && matches!(p.kind, InternalKind::InnerTube { length, .. } if (p.offset..=p.offset + length).contains(&z)))
    }

    /// The size of internal part `id` with its auto dimensions resolved: against the parent's inner diameter where it sits,
    /// and for rings against the inner tube they sit on. Recovery gear, streamers and mass components with
    /// no diameter fill [`AUTO_FILL`] of the parent's bore. `None` if `id` is not an internal part.
    pub fn resolved(&self, id: PartId) -> Option<Resolved> {
        let part = self.internal(id)?;
        let extent = part.kind.extent();
        let mid = part.offset + extent / 2.0;
        let bore = self.inner_diameter_at(part.parent, mid).unwrap_or(0.0);
        let tube = self.inner_tube_at(part.parent, mid, id).and_then(|t| match t.kind {
            InternalKind::InnerTube { outer_diameter, thickness, .. } => Some((outer_diameter, (outer_diameter - 2.0 * thickness).max(0.0))),
            _ => None,
        });
        let pick = |given: Option<f64>, auto: f64| (given.unwrap_or(auto), given.is_none());
        let (outer, outer_auto, inner, inner_auto) = match part.kind {
            InternalKind::InnerTube { outer_diameter, thickness, .. } => (outer_diameter, false, (outer_diameter - 2.0 * thickness).max(0.0), false),
            InternalKind::CenteringRing { outer_diameter, inner_diameter, .. } => {
                let ((o, oa), (i, ia)) = (pick(outer_diameter, bore), pick(inner_diameter, tube.map_or(0.0, |t| t.0)));
                (o, oa, i, ia)
            }
            InternalKind::Bulkhead { diameter, .. } => {
                let (o, oa) = pick(diameter, bore);
                (o, oa, 0.0, false)
            }
            InternalKind::Coupler { outer_diameter, thickness, .. } => {
                let (o, oa) = pick(outer_diameter, bore);
                (o, oa, (o - 2.0 * thickness).max(0.0), false)
            }
            InternalKind::Parachute { packed_diameter, .. } => {
                let (o, oa) = pick(packed_diameter, AUTO_FILL * bore);
                (o, oa, 0.0, false)
            }
            InternalKind::MassComponent { diameter, .. } => {
                let (o, oa) = pick(diameter, AUTO_FILL * bore);
                (o, oa, 0.0, false)
            }
            InternalKind::Streamer { .. } | InternalKind::ShockCord { .. } => (AUTO_FILL * bore, true, 0.0, false),
        };
        Some(Resolved { length: extent, outer_diameter: outer, inner_diameter: inner, outer_auto, inner_auto })
    }

    /// Whether an internal part of `kind` fits in axial part `parent` with its fore edge `offset` behind the parent's fore face
    /// (limited to keep it within the parent): the parent is at least as long as the part, and the shell's bore is at least
    /// as wide as the part's outer diameter along its whole length. Parts whose diameter is automatic always fit the bore they
    /// follow. Used to decide whether an internal part dragged from one body part into another may land there.
    pub fn internal_fits(&self, kind: &InternalKind, parent: PartId, offset: f64) -> bool {
        const SLACK: f64 = 1e-6;
        let Some(part) = self.part(parent) else {
            return false;
        };
        let extent = kind.extent();
        if extent > part.kind.length() + SLACK {
            return false;
        }
        let need = kind.given_outer_diameter().unwrap_or(0.0);
        let Some(offset) = self.clamp_internal_offset(parent, offset, extent) else {
            return false;
        };
        [0.0, 0.5, 1.0].into_iter().all(|f| part.inner_diameter_at(offset + extent * f) + SLACK >= need)
    }

    // ---- axial parts ----------------------------------------------------------------------------------------------

    /// Inserts `matched(index, kind)` at `index` keeping `id` (a part moved within the rocket). Fails if `id` is already
    /// present. The part gets the default material of its kind; to keep a removed part's properties use
    /// [`restore`](Self::restore).
    pub fn insert_with_id(&mut self, index: usize, id: PartId, kind: PartKind) -> Result<(), InsertError> {
        self.restore(index, Part::new(id, self.matched(index, &kind)))
    }

    /// Puts `part` back at `index` exactly as it is, without matching diameters (undoing a removal).
    pub fn restore(&mut self, index: usize, part: Part) -> Result<(), InsertError> {
        self.check(index, &part.kind)?;
        if self.parts.iter().any(|p| p.id == part.id) {
            return Err(InsertError::DuplicateId);
        }
        self.next_id = self.next_id.max(part.id + 1);
        self.parts.insert(index, part);
        Ok(())
    }

    /// The axial part with `id`: of the core, or of a pod's stack.
    pub fn part(&self, id: PartId) -> Option<&Part> {
        self.parts.iter().find(|p| p.id == id).or_else(|| self.pod_parts().find(|p| p.id == id))
    }

    /// Position (fore to aft) of the part with `id`; it is the node index of that part's fore face.
    pub fn index_of(&self, id: PartId) -> Option<usize> {
        self.parts.iter().position(|p| p.id == id)
    }

    /// Replaces the geometry of part `id`. The kind of part must stay the same (a nose cone stays a nose cone, which keeps
    /// the stack's ordering rules intact); returns whether it was applied. Radial and internal parts on it are limited to
    /// its new length.
    pub fn set_kind(&mut self, id: PartId, kind: PartKind) -> bool {
        if self.pod_index(id).is_some() {
            return self.in_pod(id, |s| s.set_kind(id, kind)).unwrap_or(false);
        }
        match self.parts.iter_mut().find(|p| p.id == id) {
            Some(part) if std::mem::discriminant(&part.kind) == std::mem::discriminant(&kind) => {
                part.kind = kind;
                self.clamp_radials(id);
                self.clamp_internals(id);
                true
            }
            _ => false,
        }
    }

    /// Removes the axial part with `id` (and the radial and internal parts on it), returning it. Motors configured for
    /// removed motor mounts are dropped from every configuration.
    pub fn remove(&mut self, id: PartId) -> Option<Part> {
        if let Some(i) = self.pod_index(id) {
            let pod = self.radials[i].id;
            let part = self.in_pod(id, |s| s.remove(id)).flatten();
            // A pod with nothing left goes too.
            if self.radials[i].kind.pod_body().is_some_and(|b| b.parts.is_empty()) {
                self.remove_radial(pod);
            }
            return part;
        }
        let i = self.parts.iter().position(|p| p.id == id)?;
        self.radials.retain(|r| r.parent != id);
        self.internals.retain(|p| p.parent != id);
        self.prune_configs();
        Some(self.parts.remove(i))
    }

    // ---- any part -------------------------------------------------------------------------------------------------

    /// The part with `id`, whatever class it is of.
    pub fn kind_of(&self, id: PartId) -> Option<PartRef<'_>> {
        if let Some(p) = self.part(id) {
            Some(PartRef::Axial(p))
        } else if let Some(p) = self.radial(id) {
            Some(PartRef::Radial(p))
        } else {
            self.internal(id).map(PartRef::Internal)
        }
    }

    /// Every part id: the axial stack fore to aft, then radial parts, then internal parts.
    pub fn ids(&self) -> Vec<PartId> {
        let core = self.parts.iter().map(|p| p.id).chain(self.radials.iter().map(|r| r.id)).chain(self.internals.iter().map(|i| i.id));
        core.chain(self.radials.iter().filter_map(|r| r.kind.pod_body()).flat_map(|b| b.ids())).collect()
    }

    /// Station of the fore edge of any part, from the fore end of the rocket.
    pub fn part_station(&self, id: PartId) -> Option<f64> {
        if let Some((z0, sub)) = self.scratch_of(id) {
            return Some(z0 + sub.part_station(id)?);
        }
        match self.kind_of(id)? {
            PartRef::Axial(p) => Some(self.stations()[self.index_of(p.id)?]),
            PartRef::Radial(r) => self.radial_station(r),
            PartRef::Internal(i) => Some(self.stations()[self.index_of(i.parent)?] + i.offset),
        }
    }

    /// Stations of the fore and aft edges of any part (a fin group's root chord, a packed parachute's packed length).
    pub fn part_span(&self, id: PartId) -> Option<(f64, f64)> {
        let fore = self.part_station(id)?;
        let extent = match self.kind_of(id)? {
            PartRef::Axial(p) => p.kind.length(),
            PartRef::Radial(r) => r.kind.extent(),
            PartRef::Internal(i) => i.kind.extent(),
        };
        Some((fore, fore + extent))
    }

    /// The part `id` sits on or in; `None` for axial parts and unknown ids.
    pub fn parent_of(&self, id: PartId) -> Option<PartId> {
        self.kind_of(id)?.parent()
    }

    /// Sets the user's name of any part (`None` for the kind's name). Returns whether `id` exists.
    pub fn set_name(&mut self, id: PartId, name: Option<String>) -> bool {
        if self.pod_index(id).is_some() {
            return self.in_pod(id, |s| s.set_name(id, name)).unwrap_or(false);
        }
        if let Some(p) = self.parts.iter_mut().find(|p| p.id == id) {
            p.name = name;
        } else if let Some(p) = self.radials.iter_mut().find(|p| p.id == id) {
            p.name = name;
        } else if let Some(p) = self.internals.iter_mut().find(|p| p.id == id) {
            p.name = name;
        } else {
            return false;
        }
        true
    }

    /// Sets the material of any part. Refused (false) when the stock is of the wrong sort for the part: bulk for tubes, fins
    /// and discs, surface for a parachute canopy or streamer, line for shock cord. Mass components take any.
    pub fn set_material(&mut self, id: PartId, material: MaterialId) -> bool {
        if self.pod_index(id).is_some() {
            return self.in_pod(id, |s| s.set_material(id, material)).unwrap_or(false);
        }
        if let Some(p) = self.parts.iter_mut().find(|p| p.id == id) {
            let ok = material.kind() == MaterialKind::Bulk;
            ok && {
                p.material = material;
                true
            }
        } else if let Some(p) = self.radials.iter_mut().find(|p| p.id == id) {
            let ok = material.kind() == MaterialKind::Bulk;
            ok && {
                p.material = material;
                true
            }
        } else if let Some(p) = self.internals.iter_mut().find(|p| p.id == id) {
            let ok = p.kind.material_kind().is_none_or(|k| k == material.kind());
            ok && {
                p.material = material;
                true
            }
        } else {
            false
        }
    }

    /// Sets the outside finish of an axial or radial part. Returns whether it was applied (internal parts have no finish).
    pub fn set_finish(&mut self, id: PartId, finish: Finish) -> bool {
        if self.pod_index(id).is_some() {
            return self.in_pod(id, |s| s.set_finish(id, finish)).unwrap_or(false);
        }
        if let Some(p) = self.parts.iter_mut().find(|p| p.id == id) {
            p.finish = finish;
        } else if let Some(p) = self.radials.iter_mut().find(|p| p.id == id) {
            p.finish = finish;
        } else {
            return false;
        }
        true
    }

    /// Sets the shell wall thickness of an axial part, limited to `0..=` its largest radius (a thickness of the radius makes
    /// it solid). Returns whether it was applied.
    pub fn set_thickness(&mut self, id: PartId, thickness: f64) -> bool {
        if self.pod_index(id).is_some() {
            return self.in_pod(id, |s| s.set_thickness(id, thickness)).unwrap_or(false);
        }
        let Some(p) = self.parts.iter_mut().find(|p| p.id == id) else {
            return false;
        };
        let max_radius = p.kind.fore_diameter().max(p.kind.aft_diameter()) / 2.0;
        p.thickness = thickness.clamp(0.0, max_radius);
        true
    }

    /// Sets the mass that replaces the computed one for any part (`None` to compute it). A radial group's is for all copies.
    pub fn set_mass_override(&mut self, id: PartId, mass: Option<f64>) -> bool {
        if self.pod_index(id).is_some() {
            return self.in_pod(id, |s| s.set_mass_override(id, mass)).unwrap_or(false);
        }
        // A pod weighs what its parts weigh.
        if self.is_pod(id) {
            return false;
        }
        let mass = mass.map(|m| m.max(0.0));
        if let Some(p) = self.parts.iter_mut().find(|p| p.id == id) {
            p.mass_override = mass;
        } else if let Some(p) = self.radials.iter_mut().find(|p| p.id == id) {
            p.mass_override = mass;
        } else if let Some(p) = self.internals.iter_mut().find(|p| p.id == id) {
            p.mass_override = mass;
        } else {
            return false;
        }
        true
    }

    /// Sets the drag coefficient (on the rocket's reference area) that replaces the computed drag of an axial part or a radial
    /// group (all copies; on a pod, the whole pod), `None` to compute it. Limited to `0..=10`. Internal parts carry no
    /// aerodynamics and are refused. Returns whether it was applied.
    pub fn set_cd_override(&mut self, id: PartId, cd: Option<f64>) -> bool {
        if self.pod_index(id).is_some() {
            return self.in_pod(id, |s| s.set_cd_override(id, cd)).unwrap_or(false);
        }
        let cd = match cd {
            Some(c) if !c.is_finite() => return false,
            c => c.map(|c| c.clamp(0.0, 10.0)),
        };
        if let Some(p) = self.parts.iter_mut().find(|p| p.id == id) {
            p.cd_override = cd;
        } else if let Some(p) = self.radials.iter_mut().find(|p| p.id == id) {
            p.cd_override = cd;
        } else {
            return false;
        }
        true
    }

    /// Sets (or with `None` removes) the active roll control of fin group `id` (see [`crate::RollControl`]), its values brought
    /// into range. Refuses parts that are not fins and controllers with a value that is not a number. Returns whether it was
    /// applied.
    pub fn set_roll_control(&mut self, id: PartId, control: Option<crate::RollControl>) -> bool {
        if self.pod_index(id).is_some() {
            return self.in_pod(id, |s| s.set_roll_control(id, control)).unwrap_or(false);
        }
        let control = match control {
            Some(c) => match c.sanitized() {
                Some(c) => Some(c),
                None => return false,
            },
            None => None,
        };
        match self.radials.iter_mut().find(|r| r.id == id && r.kind.is_fin()) {
            Some(r) => {
                r.roll_control = control;
                true
            }
            None => false,
        }
    }

    /// Sets the CG that replaces the computed one for any part, metres behind the part's fore edge (`None` to compute it).
    pub fn set_cg_override(&mut self, id: PartId, cg: Option<f64>) -> bool {
        if self.pod_index(id).is_some() {
            return self.in_pod(id, |s| s.set_cg_override(id, cg)).unwrap_or(false);
        }
        if self.is_pod(id) {
            return false;
        }
        if let Some(p) = self.parts.iter_mut().find(|p| p.id == id) {
            p.cg_override = cg;
        } else if let Some(p) = self.radials.iter_mut().find(|p| p.id == id) {
            p.cg_override = cg;
        } else if let Some(p) = self.internals.iter_mut().find(|p| p.id == id) {
            p.cg_override = cg;
        } else {
            return false;
        }
        true
    }

    /// Sets the plate section of a fin group. Returns whether `id` is a radial part.
    pub fn set_fin_section(&mut self, id: PartId, section: FinSection) -> bool {
        if self.pod_index(id).is_some() {
            return self.in_pod(id, |s| s.set_fin_section(id, section)).unwrap_or(false);
        }
        match self.radials.iter_mut().find(|p| p.id == id) {
            Some(p) => {
                p.cross_section = section;
                true
            }
            None => false,
        }
    }

    /// Sets the cant of a fin group, radians. Returns whether `id` is a radial part.
    pub fn set_cant(&mut self, id: PartId, cant: f64) -> bool {
        if self.pod_index(id).is_some() {
            return self.in_pod(id, |s| s.set_cant(id, cant)).unwrap_or(false);
        }
        match self.radials.iter_mut().find(|p| p.id == id) {
            Some(p) => {
                p.cant = cant;
                true
            }
            None => false,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::{FinPlanform, NoseShape};
    use crate::internal::{Deployment, PartClass, System};

    const SHAPES: [NoseShape; 5] = [NoseShape::Conical, NoseShape::TangentOgive, NoseShape::VonKarman, NoseShape::Elliptical, NoseShape::Parabolic];

    fn nose(d: f64) -> PartKind {
        PartKind::NoseCone { shape: NoseShape::TangentOgive, length: 0.3, diameter: d }
    }
    fn tube(l: f64, d: f64) -> PartKind {
        PartKind::BodyTube { length: l, diameter: d }
    }
    fn trans(f: f64, a: f64) -> PartKind {
        PartKind::Transition { length: 0.1, fore_diameter: f, aft_diameter: a }
    }

    #[test]
    fn nose_profiles() {
        for shape in SHAPES {
            let k = PartKind::NoseCone { shape, length: 0.4, diameter: 0.1 };
            let p = k.profile();
            assert_eq!(p.len(), 33);
            assert_eq!(p[0], (0.0, 0.0));
            assert_eq!(p[32], (0.4, 0.05));
            for w in p.windows(2) {
                assert!(w[1].1 >= w[0].1 && w[1].0 > w[0].0, "{shape:?}");
            }
        }
    }

    #[test]
    fn names_and_diameters() {
        assert_eq!(nose(0.1).name(), "Nose cone");
        assert_eq!(nose(0.1).fore_diameter(), 0.0);
        assert_eq!(tube(1.0, 0.1).name(), "Body tube");
        assert_eq!(trans(0.1, 0.2).name(), "Transition");
        assert_eq!(trans(0.2, 0.1).name(), "Boattail");
    }

    fn stack() -> Rocket {
        let mut r = Rocket::default();
        r.insert(0, nose(0.1)).unwrap();
        r.insert(1, tube(1.0, 0.1)).unwrap();
        r
    }

    #[test]
    fn stations_length() {
        let r = stack();
        assert_eq!(r.stations(), vec![0.0, 0.3, 1.3]);
        assert_eq!(r.length(), 1.3);
        assert_eq!(r.reference_diameter(), Some(0.1));
        assert_eq!(Rocket::default().reference_diameter(), None);
        assert_eq!(Rocket::default().stations(), vec![0.0]);
    }

    #[test]
    fn nodes() {
        let e = Rocket::default();
        assert_eq!(e.nodes_for(&tube(1.0, 0.1)), vec![(0, 0.0)]);
        assert_eq!(e.nodes_for(&nose(0.1)), vec![(0, 0.0)]);
        let r = stack();
        assert!(r.nodes_for(&nose(0.1)).is_empty());
        assert_eq!(r.nodes_for(&tube(1.0, 0.1)), vec![(1, 0.3), (2, 1.3)]);
        let mut r2 = Rocket::default();
        r2.insert(0, tube(1.0, 0.1)).unwrap();
        assert_eq!(r2.nodes_for(&nose(0.1)), vec![(0, 0.0)]);
        assert_eq!(r2.nodes_for(&tube(1.0, 0.1)).len(), 2);
    }

    #[test]
    fn insert_errors() {
        let mut r = stack();
        assert_eq!(r.insert(0, nose(0.1)), Err(InsertError::SecondNose));
        assert_eq!(r.insert(0, tube(1.0, 0.1)), Err(InsertError::SecondNose));
        assert_eq!(r.insert(9, tube(1.0, 0.1)), Err(InsertError::IndexOutOfRange));
        let mut r2 = Rocket::default();
        r2.insert(0, tube(1.0, 0.1)).unwrap();
        assert_eq!(r2.insert(1, nose(0.1)), Err(InsertError::NoseNotFirst));
    }

    #[test]
    fn matching() {
        let r = stack();
        assert_eq!(r.matched(2, &tube(2.0, 0.5)), tube(2.0, 0.1));
        let mut r2 = Rocket::default();
        r2.insert(0, tube(1.0, 0.2)).unwrap();
        assert_eq!(r2.matched(0, &nose(0.9)), nose(0.2));
        assert_eq!(r2.matched(0, &tube(2.0, 0.5)), tube(2.0, 0.2));
        // between two parts
        let mut r3 = stack();
        r3.insert(2, tube(1.0, 0.3)).unwrap(); // matches 0.1 from before
        assert_eq!(r3.parts()[2].kind.aft_diameter(), 0.1);
        let r4 = {
            let mut r = stack();
            r.parts[1].kind = tube(1.0, 0.1);
            r.parts.push(Part::new(99, tube(1.0, 0.2)));
            r
        };
        assert_eq!(r4.matched(2, &trans(0.5, 0.7)), trans(0.1, 0.2));
        // appended keeps ratio
        assert_eq!(r.matched(2, &trans(0.2, 0.1)), trans(0.1, 0.05));
        // empty: unchanged
        assert_eq!(Rocket::default().matched(0, &trans(0.2, 0.1)), trans(0.2, 0.1));
    }

    #[test]
    fn ids_unique_stable() {
        let mut r = Rocket::default();
        let a = r.insert(0, tube(1.0, 0.1)).unwrap();
        let b = r.insert(1, tube(1.0, 0.1)).unwrap();
        assert_ne!(a, b);
        assert_eq!(r.remove(a).unwrap().id, a);
        assert!(r.remove(a).is_none());
        let c = r.insert(0, tube(1.0, 0.1)).unwrap();
        assert!(c != a && c != b);
        assert_eq!(r.parts()[1].id, b);
    }

    #[test]
    fn lookup_and_set_kind() {
        let mut r = stack();
        let tube_id = r.parts()[1].id;
        assert_eq!(r.index_of(tube_id), Some(1));
        assert_eq!(r.part(tube_id).unwrap().kind, tube(1.0, 0.1));
        assert!(r.index_of(77).is_none() && r.part(77).is_none());
        assert!(r.set_kind(tube_id, tube(2.0, 0.2)));
        assert_eq!(r.length(), 2.3);
        // Another kind of part is refused, as is an unknown id.
        assert!(!r.set_kind(tube_id, nose(0.1)));
        assert!(!r.set_kind(77, tube(1.0, 0.1)));
        assert_eq!(r.part(tube_id).unwrap().kind, tube(2.0, 0.2));
    }

    #[test]
    fn move_a_part_keeping_its_id() {
        let mut r = stack();
        let a = r.insert(2, trans(0.1, 0.05)).unwrap();
        let removed = r.remove(a).unwrap();
        assert_eq!(r.parts().len(), 2);
        // Dropping it between nose and tube re-matches its fore diameter to the nose's aft diameter.
        r.insert_with_id(1, a, trans(0.2, 0.08)).unwrap();
        assert_eq!(r.index_of(a), Some(1));
        assert_eq!(r.part(a).unwrap().kind, trans(0.1, 0.1));
        assert_eq!(r.insert_with_id(2, a, tube(1.0, 0.1)), Err(InsertError::DuplicateId));
        // Restoring puts the part back exactly, ids stay unique for later inserts.
        r.remove(a).unwrap();
        r.restore(2, removed.clone()).unwrap();
        assert_eq!(r.parts()[2], removed);
        let fresh = r.insert(3, tube(1.0, 0.1)).unwrap();
        assert!(fresh > a);
        // The ordering rules still hold when restoring.
        let mut nosed = stack();
        assert_eq!(nosed.restore(0, Part::new(50, tube(1.0, 0.1))), Err(InsertError::SecondNose));
    }

    #[test]
    fn kind_setters() {
        assert_eq!(tube(1.0, 0.1).with_length(2.0), tube(2.0, 0.1));
        assert_eq!(tube(1.0, 0.1).with_aft_diameter(0.2), tube(1.0, 0.2));
        assert_eq!(tube(1.0, 0.1).with_fore_diameter(0.2), tube(1.0, 0.2));
        assert_eq!(trans(0.1, 0.2).with_fore_diameter(0.3), trans(0.3, 0.2));
        assert_eq!(trans(0.1, 0.2).with_aft_diameter(0.3), trans(0.1, 0.3));
        assert_eq!(nose(0.1).with_fore_diameter(0.5), nose(0.1));
        assert_eq!(nose(0.1).with_aft_diameter(0.5), nose(0.5));
        assert_eq!(nose(0.1).with_length(0.4).length(), 0.4);
    }

    fn fins() -> RadialKind {
        RadialKind::FinSet { root_chord: 0.12, tip_chord: 0.05, span: 0.07, sweep: 0.06, thickness: 0.003 }
    }

    fn lug() -> RadialKind {
        RadialKind::LaunchLug { length: 0.05, diameter: 0.01 }
    }

    #[test]
    fn radial_names_angles_and_outline() {
        assert_eq!(fins().name(), "Fin set");
        assert_eq!(lug().name(), "Launch lug");
        assert_eq!(RadialKind::RailButton { height: 0.008, diameter: 0.011 }.name(), "Rail button");
        assert_eq!(fins().extent(), 0.12);
        assert_eq!(fins().fin_outline(), Some([(0.0, 0.0), (0.06, 0.07), (0.11, 0.07), (0.12, 0.0)]));
        assert_eq!(lug().fin_outline(), None);
        let r = RadialPart::new(1, fins(), 0, 0.0, 0.0, 4);
        let a = r.angles();
        assert_eq!(a.len(), 4);
        for (i, v) in a.iter().enumerate() {
            assert!((v - i as f64 * std::f64::consts::FRAC_PI_2).abs() < 1e-12);
        }
        // Copies wrap past a full turn.
        let r = RadialPart { angle: 5.0, count: 3, ..r };
        assert!(r.angles().iter().all(|a| (0.0..std::f64::consts::TAU).contains(a)));
    }

    #[test]
    fn attach_clamps_and_validates() {
        let mut r = stack();
        let tube_id = r.parts()[1].id;
        // A fin may trail past the aft end, with a quarter of its 0.12 root chord still on the tube (length 1.0)...
        let id = r.attach_radial(tube_id, 0.95, 0.5, 3, fins()).unwrap();
        assert_eq!(r.radial(id).unwrap().offset, 0.95);
        let far = r.attach_radial(tube_id, 5.0, 0.5, 3, fins()).unwrap();
        assert_eq!(r.radial(far).unwrap().offset, 1.0 - 0.03);
        // ...but a lug lies wholly on it.
        let lug_id = r.attach_radial(tube_id, 5.0, 0.5, 1, lug()).unwrap();
        assert_eq!(r.radial(lug_id).unwrap().offset, 1.0 - 0.05);
        assert_eq!(r.attach_radial(tube_id, -1.0, 0.0, 1, lug()).map(|id| r.radial(id).unwrap().offset), Ok(0.0));
        assert_eq!(r.attach_radial(99, 0.0, 0.0, 1, lug()), Err(RadialError::NoSuchParent));
        assert_eq!(r.attach_radial(tube_id, 0.0, 0.0, 0, lug()), Err(RadialError::BadCount));
        assert_eq!(r.attach_radial(tube_id, 0.0, 0.0, 17, lug()), Err(RadialError::BadCount));
        // A radial id is not an axial part.
        assert_eq!(r.attach_radial(id, 0.0, 0.0, 1, lug()), Err(RadialError::NoSuchParent));
        // Ids come from the same sequence as axial ids.
        let fresh = r.insert(2, tube(1.0, 0.1)).unwrap();
        assert!(r.parts().iter().all(|p| p.id != id) && fresh > id);
    }

    #[test]
    fn surface_radius_interpolates() {
        let mut r = stack();
        let t = r.insert(2, trans(0.1, 0.04)).unwrap();
        assert!((r.surface_radius(t, 0.0).unwrap() - 0.05).abs() < 1e-12);
        assert!((r.surface_radius(t, 0.05).unwrap() - 0.035).abs() < 1e-12);
        assert!((r.surface_radius(t, 0.1).unwrap() - 0.02).abs() < 1e-12);
        assert!((r.surface_radius(t, 5.0).unwrap() - 0.02).abs() < 1e-12);
        let n = r.parts()[0].id;
        assert_eq!(r.surface_radius(n, 0.0), Some(0.0));
        assert!((r.surface_radius(n, 0.3).unwrap() - 0.05).abs() < 1e-12);
        assert_eq!(r.surface_radius(99, 0.0), None);
        // A radial on the slope stands on the smaller radius.
        let f = r.attach_radial(t, 0.0, 0.0, 1, RadialKind::LaunchLug { length: 0.1, diameter: 0.01 }).unwrap();
        let radial = r.radial(f).unwrap().clone();
        assert!((r.radial_base_radius(&radial).unwrap() - 0.02).abs() < 1e-12);
        assert_eq!(r.radial_station(&radial), Some(1.3));
    }

    #[test]
    fn radials_follow_their_parent() {
        let mut r = stack();
        let tube_id = r.parts()[1].id;
        let f = r.attach_radial(tube_id, 0.5, 0.0, 3, fins()).unwrap();
        // Shortening the parent limits the offset.
        assert!(r.set_kind(tube_id, tube(0.4, 0.1)));
        assert!((r.radial(f).unwrap().offset - 0.37).abs() < 1e-12);
        // Moving the parent (take, remove, reinsert elsewhere) carries them with their relative offset.
        let carried = r.take_radials(tube_id);
        assert_eq!(carried.len(), 1);
        assert!(r.radials().is_empty());
        let part = r.remove(tube_id).unwrap();
        r.insert_with_id(1, part.id, part.kind).unwrap();
        r.restore_radials(carried);
        assert_eq!(r.radial(f).unwrap().parent, tube_id);
        // Removing the parent removes its radials.
        r.remove(tube_id).unwrap();
        assert!(r.radial(f).is_none());
    }

    #[test]
    fn edit_move_and_remove_radials() {
        let mut r = stack();
        let tube_id = r.parts()[1].id;
        let f = r.attach_radial(tube_id, 0.0, 0.0, 3, fins()).unwrap();
        let other = r.insert(2, tube(0.1, 0.1)).unwrap();
        // Moving onto a shorter part limits the offset; the angle wraps.
        r.move_radial(f, other, 0.5, -0.5).unwrap();
        let moved = r.radial(f).unwrap();
        assert_eq!(moved.parent, other);
        assert!((moved.offset - 0.07).abs() < 1e-12);
        assert!((moved.angle - (std::f64::consts::TAU - 0.5)).abs() < 1e-12);
        assert_eq!(r.move_radial(f, 99, 0.0, 0.0), Err(RadialError::NoSuchParent));
        assert_eq!(r.move_radial(77, other, 0.0, 0.0), Err(RadialError::NoSuchRadial));
        // set_radial keeps the part kind and refuses bad counts.
        let bigger = RadialKind::FinSet { root_chord: 0.2, tip_chord: 0.05, span: 0.07, sweep: 0.06, thickness: 0.003 };
        assert!(r.set_radial(f, bigger.clone(), 4));
        assert_eq!((r.radial(f).unwrap().kind.clone(), r.radial(f).unwrap().count), (bigger, 4));
        // A longer root chord limits the offset: 0.05 of it must stay on the 0.1 long tube.
        assert!((r.radial(f).unwrap().offset - 0.05).abs() < 1e-12);
        assert!(!r.set_radial(f, lug(), 4));
        assert!(!r.set_radial(f, fins(), 0));
        assert!(!r.set_radial(77, fins(), 3));
        // Remove and restore.
        let taken = r.remove_radial(f).unwrap();
        assert!(r.radial(f).is_none() && r.remove_radial(f).is_none());
        r.restore_radial(taken.clone()).unwrap();
        assert_eq!(r.radial(f), Some(&taken));
        assert_eq!(r.restore_radial(taken), Err(RadialError::NoSuchRadial));
    }

    #[test]
    fn fins_may_sweep_forward_but_not_past_the_parents_fore_face() {
        let mut r = stack();
        let tube_id = r.parts()[1].id;
        let fin = |sweep| RadialKind::FinSet { root_chord: 0.12, tip_chord: 0.05, span: 0.07, sweep, thickness: 0.003 };
        let sweep_of = |r: &Rocket, id| match r.radial(id).unwrap().kind {
            RadialKind::FinSet { sweep, .. } => sweep,
            _ => unreachable!(),
        };
        // 0.2 along the tube, the tip may lead the root by up to 0.2.
        let id = r.attach_radial(tube_id, 0.2, 0.0, 3, fin(-0.05)).unwrap();
        assert_eq!(sweep_of(&r, id), -0.05);
        assert!(r.set_radial(id, fin(-0.5), 3));
        assert_eq!(sweep_of(&r, id), -0.2);
        // The outline puts the tip's leading edge ahead of the root's.
        let outline = fin(-0.05).fin_outline().unwrap();
        assert_eq!((outline[1], outline[2]), ((-0.05, 0.07), (0.0, 0.07)));
        // Moving it fore tightens the limit; moving it aft leaves the sweep alone.
        r.move_radial(id, tube_id, 0.1, 0.0).unwrap();
        assert_eq!(sweep_of(&r, id), -0.1);
        r.move_radial(id, tube_id, 0.6, 0.0).unwrap();
        assert_eq!(sweep_of(&r, id), -0.1);
        // Putting it back keeps the limit, and a sweep past it on the way in is limited too.
        let taken = r.remove_radial(id).unwrap();
        r.restore_radial(RadialPart { kind: fin(-3.0), ..taken }).unwrap();
        assert_eq!(sweep_of(&r, id), -0.6);
        // Lugs have no sweep to limit.
        assert_eq!(lug().limited_at(0.0), lug());
    }

    #[test]
    fn parametric_nose_shapes() {
        let profile = |shape| PartKind::NoseCone { shape, length: 0.4, diameter: 0.1 }.profile();
        // A power series with n = 1 is the cone; n = 0.5 is fuller than the cone but monotonic; Haack C = 0 is the Von Karman.
        for (a, b) in profile(NoseShape::PowerSeries(1.0)).iter().zip(profile(NoseShape::Conical)) {
            assert!((a.1 - b.1).abs() < 1e-12);
        }
        let half = profile(NoseShape::PowerSeries(0.5));
        assert!(half.windows(2).all(|w| w[1].1 >= w[0].1));
        assert!(half[16].1 > profile(NoseShape::Conical)[16].1);
        for (a, b) in profile(NoseShape::Haack(0.0)).iter().zip(profile(NoseShape::VonKarman)) {
            assert!((a.1 - b.1).abs() < 1e-12);
        }
        for shape in NoseShape::LIST {
            let p = profile(shape);
            assert_eq!((p[0], p[32]), ((0.0, 0.0), (0.4, 0.05)), "{shape:?}");
            assert!(p.windows(2).all(|w| w[1].1 >= w[0].1 - 1e-12), "{shape:?}");
        }
        assert_eq!(NoseShape::Haack(0.25).parameter(), Some(0.25));
        assert_eq!(NoseShape::Conical.parameter(), None);
        // radius_at agrees with the sampled profile at its vertices.
        let k = PartKind::NoseCone { shape: NoseShape::TangentOgive, length: 0.4, diameter: 0.1 };
        let p = k.profile();
        assert!((k.radius_at(p[10].0) - p[10].1).abs() < 1e-12);
        assert!((k.radius_at(9.0) - 0.05).abs() < 1e-12);
    }

    #[test]
    fn new_parts_have_defaults() {
        let r = stack();
        let tube = &r.parts()[1];
        assert_eq!((tube.material, tube.finish, tube.thickness), (MaterialId::CARDBOARD, Finish::RegularPaint, 0.0016));
        assert_eq!((tube.name.as_deref(), tube.mass_override, tube.cg_override), (None, None, None));
        assert_eq!(tube.display_name(), "Body tube");
        assert_eq!(tube.inner_diameter_at(0.5), 0.1 - 0.0032);
        let f = RadialPart::new(1, fins(), 0, 0.0, 0.0, 3);
        assert_eq!((f.material, f.cross_section, f.cant), (MaterialId::BIRCH_PLYWOOD, FinSection::Square, 0.0));
        assert_eq!(RadialPart::new(1, lug(), 0, 0.0, 0.0, 1).material, MaterialId::CARDBOARD);
    }

    #[test]
    fn internal_parts_follow_their_parent() {
        let mut r = stack();
        let tube_id = r.parts()[1].id;
        let ring = InternalKind::CenteringRing { outer_diameter: None, inner_diameter: None, thickness: 0.006 };
        let a = r.attach_internal(tube_id, 0.5, ring.clone()).unwrap();
        // Limited to lie within the parent (length 1.0): a 6 mm ring starts at most 0.994.
        let b = r.attach_internal(tube_id, 5.0, ring.clone()).unwrap();
        assert!((r.internal(b).unwrap().offset - 0.994).abs() < 1e-12);
        assert_eq!(r.attach_internal(99, 0.0, ring.clone()), Err(InternalError::NoSuchParent));
        // A radial id is not an axial part, and ids come from the one sequence.
        let fin = r.attach_radial(tube_id, 0.0, 0.0, 3, fins()).unwrap();
        assert_eq!(r.attach_internal(fin, 0.0, ring.clone()), Err(InternalError::NoSuchParent));
        assert!(r.ids().len() == 5 && fin > b && b > a);
        // Shortening the parent limits the offsets.
        assert!(r.set_kind(tube_id, tube(0.4, 0.1)));
        assert!((r.internal(a).unwrap().offset - 0.394).abs() < 1e-12);
        // Carry them with the parent while it is moved.
        let carried = r.take_internals(tube_id);
        assert_eq!(carried.len(), 2);
        assert!(r.internals().is_empty());
        let part = r.remove(tube_id).unwrap();
        r.restore(1, part).unwrap();
        r.restore_internals(carried);
        assert_eq!(r.internals_of(tube_id).count(), 2);
        // Move to another tube; the offset is limited there.
        let other = r.insert(2, tube(0.1, 0.1)).unwrap();
        r.move_internal(a, other, 0.5).unwrap();
        assert_eq!((r.internal(a).unwrap().parent, r.internal(a).unwrap().offset), (other, 0.1 - 0.006));
        assert_eq!(r.move_internal(a, 99, 0.0), Err(InternalError::NoSuchParent));
        assert_eq!(r.move_internal(77, other, 0.0), Err(InternalError::NoSuchInternal));
        // set_internal_kind keeps the kind of part.
        assert!(r.set_internal_kind(a, InternalKind::CenteringRing { outer_diameter: Some(0.05), inner_diameter: Some(0.02), thickness: 0.003 }));
        assert!(!r.set_internal_kind(a, InternalKind::Bulkhead { diameter: None, thickness: 0.003 }));
        // Remove and restore.
        let taken = r.remove_internal(a).unwrap();
        assert!(r.internal(a).is_none() && r.remove_internal(a).is_none());
        r.restore_internal(taken.clone()).unwrap();
        assert_eq!(r.internal(a), Some(&taken));
        assert_eq!(r.restore_internal(taken), Err(InternalError::NoSuchInternal));
        // Removing the parent removes what is in it.
        r.remove(other).unwrap();
        assert!(r.internal(a).is_none());
    }

    #[test]
    fn any_part_lookup_and_properties() {
        let mut r = stack();
        let tube_id = r.parts()[1].id;
        let fin = r.attach_radial(tube_id, 0.5, 0.0, 3, fins()).unwrap();
        let ring = r.attach_internal(tube_id, 0.2, InternalKind::Bulkhead { diameter: None, thickness: 0.006 }).unwrap();
        let chute = r
            .attach_internal(
                tube_id,
                0.3,
                InternalKind::Parachute {
                    diameter: 0.4,
                    cd: 0.8,
                    line_count: 4,
                    line_length: 0.4,
                    line_material: MaterialId::BRAIDED_NYLON_2MM,
                    packed_length: 0.1,
                    packed_diameter: None,
                    deploy: Deployment::default(),
                },
            )
            .unwrap();
        assert!(matches!(r.kind_of(tube_id), Some(PartRef::Axial(_))));
        assert!(matches!(r.kind_of(fin), Some(PartRef::Radial(_))));
        assert!(matches!(r.kind_of(ring), Some(PartRef::Internal(_))));
        assert!(r.kind_of(99).is_none());
        let (axial, radial, internal) = (r.kind_of(tube_id).unwrap(), r.kind_of(fin).unwrap(), r.kind_of(ring).unwrap());
        assert_eq!((axial.system(), radial.system(), internal.system()), (System::Structure, System::Aero, System::Structure));
        assert_eq!((axial.class(), radial.class(), internal.class()), (PartClass::Axial, PartClass::Radial, PartClass::Internal));
        assert_eq!((axial.parent(), radial.parent(), internal.parent()), (None, Some(tube_id), Some(tube_id)));
        assert_eq!((radial.kind_name(), internal.kind_name()), ("Fin set", "Bulkhead"));
        assert_eq!(r.parent_of(fin), Some(tube_id));
        assert_eq!(r.ids(), vec![r.parts()[0].id, tube_id, fin, ring, chute]);
        // Stations: tube starts at 0.3.
        assert_eq!(r.part_station(tube_id), Some(0.3));
        assert!((r.part_station(fin).unwrap() - 0.8).abs() < 1e-12);
        let span = r.part_span(ring).unwrap();
        assert!((span.0 - 0.5).abs() < 1e-12 && (span.1 - 0.506).abs() < 1e-12);
        assert_eq!(r.part_span(chute).map(|s| (s.0 * 1e6).round()), Some(600_000.0));
        assert!(r.part_station(99).is_none());

        // Setters work on every class, and reject what does not apply.
        for id in [tube_id, fin, ring] {
            assert!(r.set_name(id, Some("Renamed".to_string())));
            assert_eq!(r.kind_of(id).unwrap().name(), "Renamed");
            assert!(r.set_name(id, None));
        }
        assert_eq!(r.kind_of(fin).unwrap().name(), "Fin set");
        assert!(!r.set_name(99, None));
        assert!(r.set_material(tube_id, MaterialId::CARBON_FIBER) && r.set_material(fin, MaterialId::FIBERGLASS) && r.set_material(ring, MaterialId::BALSA));
        // The wrong sort of stock is refused: a fabric tube, a bulk canopy, a line fin.
        assert!(!r.set_material(tube_id, MaterialId::MYLAR) && !r.set_material(chute, MaterialId::ALUMINUM) && !r.set_material(fin, MaterialId::KEVLAR_LINE));
        assert!(r.set_material(chute, MaterialId::MYLAR));
        assert!(r.set_finish(tube_id, Finish::Polished) && r.set_finish(fin, Finish::Rough) && !r.set_finish(ring, Finish::Rough));
        assert_eq!(r.part(tube_id).unwrap().finish.roughness(), 2e-6);
        assert!(r.set_thickness(tube_id, 0.003) && !r.set_thickness(fin, 0.003));
        assert_eq!(r.part(tube_id).unwrap().thickness, 0.003);
        r.set_thickness(tube_id, -1.0);
        assert_eq!(r.part(tube_id).unwrap().thickness, 0.0);
        assert!(r.set_cant(fin, 0.05) && !r.set_cant(tube_id, 0.05));
        assert!(r.set_fin_section(fin, FinSection::Rounded) && !r.set_fin_section(ring, FinSection::Rounded));
        assert_eq!((r.radial(fin).unwrap().cant, r.radial(fin).unwrap().cross_section), (0.05, FinSection::Rounded));
        // Mass overrides cannot go negative.
        r.set_mass_override(ring, Some(-1.0));
        assert_eq!(r.internal(ring).unwrap().mass_override, Some(0.0));
    }

    #[test]
    fn an_internal_part_fits_where_the_bore_is_wide_and_long_enough() {
        let mut r = stack();
        // `insert` would match the diameter to the neighbour; `restore` keeps the narrow tube as it is.
        r.restore(2, Part::new(50, tube(0.5, 0.05))).unwrap();
        let (narrow, wide) = (50, r.parts()[1].id);
        let mount = InternalKind::InnerTube { outer_diameter: 0.07, thickness: 0.001, length: 0.2, motor_mount: true, overhang: 0.0 };
        // The wide tube (Ø0.1) takes a Ø0.07 mount anywhere; the Ø0.05 one does not.
        assert!(r.internal_fits(&mount, wide, 0.3));
        assert!(!r.internal_fits(&mount, narrow, 0.1));
        // Too long for the parent; unknown parent; a part with an automatic diameter follows the bore, so it fits.
        let long = InternalKind::InnerTube { outer_diameter: 0.04, thickness: 0.001, length: 2.0, motor_mount: false, overhang: 0.0 };
        assert!(!r.internal_fits(&long, wide, 0.0) && !r.internal_fits(&mount, 99, 0.0));
        assert!(r.internal_fits(&InternalKind::Bulkhead { diameter: None, thickness: 0.006 }, narrow, 0.0));
        // The nose cone's bore narrows to nothing at the tip: a Ø0.07 tube does not fit there.
        let nose_id = r.parts()[0].id;
        assert!(!r.internal_fits(&mount, nose_id, 0.0));
    }

    #[test]
    fn fin_planforms_convert_and_stay_valid() {
        let mut r = stack();
        let tube_id = r.parts()[1].id;
        let trap = fins();
        let id = r.attach_radial(tube_id, 0.2, 0.0, 3, trap.clone()).unwrap();
        assert_eq!(trap.fin_planform(), Some(FinPlanform::Trapezoidal));
        assert_eq!(lug().fin_planform(), None);
        // The sub-type select swaps the planform in place, keeping root chord, span and thickness.
        let ell = trap.with_planform(FinPlanform::Elliptical);
        assert_eq!(ell, RadialKind::EllipticalFin { root_chord: 0.12, span: 0.07, thickness: 0.003 });
        assert!(r.set_radial(id, ell.clone(), 3));
        let free = ell.with_planform(FinPlanform::Freeform);
        let RadialKind::FreeformFin { points, thickness } = &free else {
            panic!("expected a freeform fin");
        };
        assert_eq!((points.len(), *thickness), (25, 0.003));
        assert!((free.fin_area().unwrap() - ell.fin_area().unwrap()).abs() / ell.fin_area().unwrap() < 0.01);
        assert!(r.set_radial(id, free.clone(), 3));
        let back = free.with_planform(FinPlanform::Trapezoidal);
        assert!(matches!(back, RadialKind::FinSet { root_chord, span, .. } if (root_chord - 0.12).abs() < 1e-12 && (span - 0.07).abs() < 1e-12));
        assert_eq!(ell.with_planform(FinPlanform::Elliptical), ell);
        assert_eq!(lug().with_planform(FinPlanform::Elliptical), lug());
        // A fin never becomes a lug, and a freeform fin needs three points.
        assert!(!r.set_radial(id, lug(), 3));
        assert!(!r.set_radial(id, RadialKind::FreeformFin { points: vec![(0.0, 0.0), (0.1, 0.0)], thickness: 0.003 }, 3));
        // Extent, area and centroid of a freeform right triangle (root 0.1, height 0.05).
        let tri = RadialKind::FreeformFin { points: vec![(0.0, 0.0), (0.0, 0.05), (0.1, 0.0)], thickness: 0.003 };
        assert!((tri.extent() - 0.1).abs() < 1e-12 && (tri.fin_span().unwrap() - 0.05).abs() < 1e-12);
        assert!((tri.fin_area().unwrap() - 0.0025).abs() < 1e-12);
        let (cz, ch) = tri.fin_centroid().unwrap();
        assert!((cz - 0.1 / 3.0).abs() < 1e-12 && (ch - 0.05 / 3.0).abs() < 1e-12);
        assert!((tri.min_attached() - 0.025).abs() < 1e-12);
        // A forward-swept freeform fin is limited like a trapezoid.
        let swept = RadialKind::FreeformFin { points: vec![(-0.5, 0.05), (0.0, 0.0), (0.1, 0.0)], thickness: 0.003 };
        let RadialKind::FreeformFin { points, .. } = swept.limited_at(0.2) else { unreachable!() };
        assert_eq!(points[0].0, -0.2);
    }
}
