//! Rocket airframe data model for the Bowshock builder. All lengths are SI metres.

use std::fmt;

/// Stable identifier of a part within a [`Rocket`].
pub type PartId = u64;

/// Nose cone profile family.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum NoseShape {
    /// Straight cone.
    Conical,
    /// Tangent ogive.
    TangentOgive,
    /// Von Karman (Haack LD) minimum-drag shape.
    VonKarman,
    /// Half ellipse.
    Elliptical,
    /// Parabolic with K = 1.
    Parabolic,
}

/// Geometry of one airframe part.
#[derive(Clone, Debug, PartialEq)]
pub enum PartKind {
    /// Nose cone ending at `diameter`.
    NoseCone {
        /// Profile family.
        shape: NoseShape,
        /// Length along the axis.
        length: f64,
        /// Aft (base) diameter.
        diameter: f64,
    },
    /// Cylindrical body tube.
    BodyTube {
        /// Length along the axis.
        length: f64,
        /// Outer diameter.
        diameter: f64,
    },
    /// Conical transition (a boattail when it contracts).
    Transition {
        /// Length along the axis.
        length: f64,
        /// Diameter at the fore face.
        fore_diameter: f64,
        /// Diameter at the aft face.
        aft_diameter: f64,
    },
}

const NOSE_POINTS: usize = 32;

impl PartKind {
    /// Display name: "Nose cone", "Body tube", "Transition", or "Boattail" when aft < fore.
    pub fn name(&self) -> &'static str {
        match self {
            PartKind::NoseCone { .. } => "Nose cone",
            PartKind::BodyTube { .. } => "Body tube",
            PartKind::Transition { fore_diameter, aft_diameter, .. } => {
                if aft_diameter < fore_diameter {
                    "Boattail"
                } else {
                    "Transition"
                }
            }
        }
    }

    /// Length along the axis.
    pub fn length(&self) -> f64 {
        match *self {
            PartKind::NoseCone { length, .. } | PartKind::BodyTube { length, .. } | PartKind::Transition { length, .. } => length,
        }
    }

    /// Diameter at the fore face (0 for a nose cone).
    pub fn fore_diameter(&self) -> f64 {
        match *self {
            PartKind::NoseCone { .. } => 0.0,
            PartKind::BodyTube { diameter, .. } => diameter,
            PartKind::Transition { fore_diameter, .. } => fore_diameter,
        }
    }

    /// Diameter at the aft face.
    pub fn aft_diameter(&self) -> f64 {
        match *self {
            PartKind::NoseCone { diameter, .. } | PartKind::BodyTube { diameter, .. } => diameter,
            PartKind::Transition { aft_diameter, .. } => aft_diameter,
        }
    }

    /// This kind with its length replaced.
    pub fn with_length(&self, length: f64) -> PartKind {
        let mut out = self.clone();
        match &mut out {
            PartKind::NoseCone { length: l, .. } | PartKind::BodyTube { length: l, .. } | PartKind::Transition { length: l, .. } => *l = length,
        }
        out
    }

    /// This kind with its aft diameter replaced (a nose cone's base diameter, a tube's only diameter).
    pub fn with_aft_diameter(&self, diameter: f64) -> PartKind {
        let mut out = self.clone();
        match &mut out {
            PartKind::NoseCone { diameter: d, .. } | PartKind::BodyTube { diameter: d, .. } | PartKind::Transition { aft_diameter: d, .. } => *d = diameter,
        }
        out
    }

    /// This kind with its fore diameter replaced. A nose cone has no fore diameter (always a point) and a body tube has one
    /// diameter, so for those this is the same as [`with_aft_diameter`](Self::with_aft_diameter) / unchanged respectively.
    pub fn with_fore_diameter(&self, diameter: f64) -> PartKind {
        match self {
            PartKind::NoseCone { .. } => self.clone(),
            PartKind::BodyTube { .. } => self.with_aft_diameter(diameter),
            PartKind::Transition { length, aft_diameter, .. } => PartKind::Transition { length: *length, fore_diameter: diameter, aft_diameter: *aft_diameter },
        }
    }

    /// Outer profile as `(z, radius)` with z measured from the part's fore face, fore to aft.
    /// Noses are sampled with 33 points (32 segments); tubes and transitions use two.
    pub fn profile(&self) -> Vec<(f64, f64)> {
        match *self {
            PartKind::NoseCone { shape, length, diameter } => {
                let r_max = diameter / 2.0;
                (0..=NOSE_POINTS)
                    .map(|i| {
                        if i == 0 {
                            return (0.0, 0.0);
                        }
                        if i == NOSE_POINTS {
                            return (length, r_max);
                        }
                        let x = length * i as f64 / NOSE_POINTS as f64;
                        (x, nose_radius(shape, length, r_max, x))
                    })
                    .collect()
            }
            PartKind::BodyTube { length, diameter } => {
                vec![(0.0, diameter / 2.0), (length, diameter / 2.0)]
            }
            PartKind::Transition { length, fore_diameter, aft_diameter } => vec![(0.0, fore_diameter / 2.0), (length, aft_diameter / 2.0)],
        }
    }
}

fn nose_radius(shape: NoseShape, l: f64, r: f64, x: f64) -> f64 {
    let t = x / l;
    let v = match shape {
        NoseShape::Conical => r * t,
        NoseShape::TangentOgive => {
            let rho = (r * r + l * l) / (2.0 * r);
            ((rho * rho - (l - x) * (l - x)).max(0.0)).sqrt() + r - rho
        }
        NoseShape::VonKarman => {
            let theta = (1.0 - 2.0 * t).clamp(-1.0, 1.0).acos();
            r * ((theta - (2.0 * theta).sin() / 2.0) / std::f64::consts::PI).max(0.0).sqrt()
        }
        NoseShape::Elliptical => r * (1.0 - (1.0 - t) * (1.0 - t)).max(0.0).sqrt(),
        NoseShape::Parabolic => r * (2.0 * t - t * t),
    };
    v.clamp(0.0, r)
}

/// One placed airframe part.
#[derive(Clone, Debug, PartialEq)]
pub struct Part {
    /// Unique, stable id.
    pub id: PartId,
    /// Geometry.
    pub kind: PartKind,
}

/// Geometry of a part attached to the side of an axial part (a "radial" part), repeated around the axis.
#[derive(Clone, Debug, PartialEq)]
pub enum RadialKind {
    /// A trapezoidal fin: root on the parent, tip `span` out, the tip's leading edge `sweep` aft of the root's.
    FinSet {
        /// Chord along the parent at the root.
        root_chord: f64,
        /// Chord at the tip.
        tip_chord: f64,
        /// Distance from the parent surface to the tip.
        span: f64,
        /// How far the tip's leading edge is behind the root's, along the axis.
        sweep: f64,
        /// Plate thickness.
        thickness: f64,
    },
    /// A tube parallel to the axis, standing off the surface.
    LaunchLug {
        /// Length along the axis.
        length: f64,
        /// Outer diameter.
        diameter: f64,
    },
    /// A rail button: a stem out from the surface with a wider head.
    RailButton {
        /// Height above the surface.
        height: f64,
        /// Head diameter.
        diameter: f64,
    },
}

impl RadialKind {
    /// Display name.
    pub fn name(&self) -> &'static str {
        match self {
            RadialKind::FinSet { .. } => "Fin set",
            RadialKind::LaunchLug { .. } => "Launch lug",
            RadialKind::RailButton { .. } => "Rail button",
        }
    }

    /// How much of the parent's length it occupies along the axis.
    pub fn extent(&self) -> f64 {
        match *self {
            RadialKind::FinSet { root_chord, .. } => root_chord,
            RadialKind::LaunchLug { length, .. } => length,
            RadialKind::RailButton { diameter, .. } => diameter,
        }
    }

    /// This kind placed with its fore edge at `offset` on its parent: a fin may sweep forward (negative sweep), but its tip's
    /// leading edge may not go ahead of the parent's fore face, so the sweep is limited to `-offset`.
    pub fn limited_at(&self, offset: f64) -> RadialKind {
        match *self {
            RadialKind::FinSet { root_chord, tip_chord, span, sweep, thickness } => {
                RadialKind::FinSet { root_chord, tip_chord, span, sweep: sweep.max(-offset), thickness }
            }
            _ => self.clone(),
        }
    }

    /// The least length of the part that must lie on its parent. A fin may trail past the parent's aft end as long as a
    /// quarter of its root chord stays on it; lugs and buttons must lie wholly on the parent.
    pub fn min_attached(&self) -> f64 {
        match self {
            RadialKind::FinSet { root_chord, .. } => root_chord / 4.0,
            _ => self.extent(),
        }
    }

    /// The fin outline as four `(z, height)` points, `z` along the axis from the fore end of the root and `height` above the
    /// parent surface: root leading edge, tip leading edge, tip trailing edge, root trailing edge. `None` for other kinds.
    pub fn fin_outline(&self) -> Option<[(f64, f64); 4]> {
        match *self {
            RadialKind::FinSet { root_chord, tip_chord, span, sweep, .. } => Some([(0.0, 0.0), (sweep, span), (sweep + tip_chord, span), (root_chord, 0.0)]),
            _ => None,
        }
    }
}

/// A radial part placed on an axial part: `count` linked copies spaced evenly around it.
#[derive(Clone, Debug, PartialEq)]
pub struct RadialPart {
    /// Unique id, from the same sequence as axial part ids.
    pub id: PartId,
    /// Geometry shared by every copy.
    pub kind: RadialKind,
    /// The axial part it sits on.
    pub parent: PartId,
    /// Distance from the parent's fore face to the fore edge of the part.
    pub offset: f64,
    /// Angle of the first copy, radians: 0 is +Y, increasing towards +X.
    pub angle: f64,
    /// Number of copies, evenly spaced.
    pub count: u8,
}

impl RadialPart {
    /// The angle of each copy.
    pub fn angles(&self) -> Vec<f64> {
        let n = f64::from(self.count.max(1));
        (0..self.count.max(1)).map(|i| (self.angle + std::f64::consts::TAU * f64::from(i) / n).rem_euclid(std::f64::consts::TAU)).collect()
    }
}

/// Most copies a radial part may have.
pub const MAX_COUNT: u8 = 16;

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

/// An airframe stack of parts ordered fore to aft.
#[derive(Clone, Debug, Default)]
pub struct Rocket {
    parts: Vec<Part>,
    radials: Vec<RadialPart>,
    next_id: PartId,
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

    /// Inserts `matched(index, kind)` at `index` and returns its id.
    pub fn insert(&mut self, index: usize, kind: PartKind) -> Result<PartId, InsertError> {
        self.check(index, &kind)?;
        let kind = self.matched(index, &kind);
        let id = self.next_id;
        self.next_id += 1;
        self.parts.insert(index, Part { id, kind });
        Ok(id)
    }

    /// Radial parts, in the order they were placed.
    pub fn radials(&self) -> &[RadialPart] {
        &self.radials
    }

    /// The radial part with `id`.
    pub fn radial(&self, id: PartId) -> Option<&RadialPart> {
        self.radials.iter().find(|r| r.id == id)
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

    /// Station of the fore edge of a radial part.
    pub fn radial_station(&self, radial: &RadialPart) -> Option<f64> {
        let index = self.index_of(radial.parent)?;
        Some(self.stations()[index] + radial.offset)
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
    /// within the parent (a fin may trail past its aft end).
    pub fn attach_radial(&mut self, parent: PartId, offset: f64, angle: f64, count: u8, kind: RadialKind) -> Result<PartId, RadialError> {
        self.check_radial(parent, count)?;
        let offset = self.clamp_offset(parent, offset, &kind).ok_or(RadialError::NoSuchParent)?;
        let kind = kind.limited_at(offset);
        let id = self.next_id;
        self.next_id += 1;
        self.radials.push(RadialPart { id, kind, parent, offset, angle: angle.rem_euclid(std::f64::consts::TAU), count });
        Ok(id)
    }

    /// Moves radial part `id` onto `parent` at `offset` and `angle`.
    pub fn move_radial(&mut self, id: PartId, parent: PartId, offset: f64, angle: f64) -> Result<(), RadialError> {
        let existing = self.radial(id).ok_or(RadialError::NoSuchRadial)?;
        let count = existing.count;
        self.check_radial(parent, count)?;
        let offset = self.clamp_offset(parent, offset, &existing.kind).ok_or(RadialError::NoSuchParent)?;
        let radial = self.radials.iter_mut().find(|r| r.id == id).ok_or(RadialError::NoSuchRadial)?;
        (radial.parent, radial.offset, radial.angle) = (parent, offset, angle.rem_euclid(std::f64::consts::TAU));
        radial.kind = radial.kind.limited_at(offset);
        Ok(())
    }

    /// Replaces the geometry and copy count of radial part `id`; the kind of part must stay the same. Its offset is limited
    /// to keep it within the parent and a forward sweep to keep the tip behind the parent's fore face (see
    /// [`RadialKind::limited_at`]). Returns whether it was applied.
    pub fn set_radial(&mut self, id: PartId, kind: RadialKind, count: u8) -> bool {
        let Some(radial) = self.radial(id) else {
            return false;
        };
        if std::mem::discriminant(&radial.kind) != std::mem::discriminant(&kind) || count == 0 || count > MAX_COUNT {
            return false;
        }
        let offset = self.clamp_offset(radial.parent, radial.offset, &kind).unwrap_or(0.0);
        if let Some(radial) = self.radials.iter_mut().find(|r| r.id == id) {
            (radial.kind, radial.count, radial.offset) = (kind.limited_at(offset), count, offset);
        }
        true
    }

    /// Removes radial part `id`, returning it.
    pub fn remove_radial(&mut self, id: PartId) -> Option<RadialPart> {
        let i = self.radials.iter().position(|r| r.id == id)?;
        Some(self.radials.remove(i))
    }

    /// Puts a removed radial part back as it was, if its parent is still there (its offset is limited to the parent).
    pub fn restore_radial(&mut self, mut radial: RadialPart) -> Result<(), RadialError> {
        self.check_radial(radial.parent, radial.count)?;
        if self.radial(radial.id).is_some() {
            return Err(RadialError::NoSuchRadial);
        }
        radial.offset = self.clamp_offset(radial.parent, radial.offset, &radial.kind).ok_or(RadialError::NoSuchParent)?;
        radial.kind = radial.kind.limited_at(radial.offset);
        self.next_id = self.next_id.max(radial.id + 1);
        self.radials.push(radial);
        Ok(())
    }

    /// Takes every radial part off axial part `parent` (to carry them while it is moved) and returns them.
    pub fn take_radials(&mut self, parent: PartId) -> Vec<RadialPart> {
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
    fn clamp_radials(&mut self, parent: PartId) {
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

    /// Inserts `matched(index, kind)` at `index` keeping `id` (a part moved within the rocket). Fails if `id` is already
    /// present.
    pub fn insert_with_id(&mut self, index: usize, id: PartId, kind: PartKind) -> Result<(), InsertError> {
        self.restore(index, Part { id, kind: self.matched(index, &kind) })
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

    /// The part with `id`.
    pub fn part(&self, id: PartId) -> Option<&Part> {
        self.parts.iter().find(|p| p.id == id)
    }

    /// Position (fore to aft) of the part with `id`; it is the node index of that part's fore face.
    pub fn index_of(&self, id: PartId) -> Option<usize> {
        self.parts.iter().position(|p| p.id == id)
    }

    /// Replaces the geometry of part `id`. The kind of part must stay the same (a nose cone stays a nose cone, which keeps
    /// the stack's ordering rules intact); returns whether it was applied.
    pub fn set_kind(&mut self, id: PartId, kind: PartKind) -> bool {
        match self.parts.iter_mut().find(|p| p.id == id) {
            Some(part) if std::mem::discriminant(&part.kind) == std::mem::discriminant(&kind) => {
                part.kind = kind;
                self.clamp_radials(id);
                true
            }
            _ => false,
        }
    }

    /// Removes the axial part with `id` (and the radial parts on it), returning it.
    pub fn remove(&mut self, id: PartId) -> Option<Part> {
        let i = self.parts.iter().position(|p| p.id == id)?;
        self.radials.retain(|r| r.parent != id);
        Some(self.parts.remove(i))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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
            r.parts.push(Part { id: 99, kind: tube(1.0, 0.2) });
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
        assert_eq!(nosed.restore(0, Part { id: 50, kind: tube(1.0, 0.1) }), Err(InsertError::SecondNose));
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
        let r = RadialPart { id: 1, kind: fins(), parent: 0, offset: 0.0, angle: 0.0, count: 4 };
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
}
