//! The content of an assembly definition: its own parts, radial parts and internal parts with definition-local ids.

use std::collections::BTreeSet;

use serde::{Deserialize, Serialize};

use super::AssemblyError;
use crate::expr::{Expr, FieldRef, Formula};
use crate::geometry::{Part, PartKind, RadialKind, RadialPart, MAX_COUNT};
use crate::internal::InternalPart;
use crate::PartId;

/// Which kind of assembly a [`Fragment`] is (see the [module docs](super)), derived from its contents.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum FragmentKind {
    /// A run of airframe parts with what they carry.
    Stack,
    /// Internal parts of one host tube.
    Internal,
    /// Radial parts on one host body.
    Surface,
}

/// The host a non-stack fragment was made in: what a stand-in host tube in the editor ([`AssemblyDef::to_rocket`](super::AssemblyDef::to_rocket))
/// is built from, and what members that touch the wall measure the new host against.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Host {
    /// Outer diameter of the host where the fragment sits, metres.
    pub outer_diameter: f64,
    /// Wall thickness of the host, metres.
    pub thickness: f64,
    /// Length of the host, metres.
    pub length: f64,
}

impl Host {
    /// Inner diameter of the host's shell.
    pub fn bore(&self) -> f64 {
        (self.outer_diameter - 2.0 * self.thickness).max(0.0)
    }
}

/// The contents of an assembly: parts, radial parts and internal parts with ids local to the definition, the formulas whose
/// references all lie inside, and for internal and surface assemblies the [`Host`] they were made in.
///
/// Positions are as they were in the host the fragment was made in; [`origin`](Self::origin) is where the fragment starts and
/// every placement works from the distance to it. A radial or internal part of a non-stack fragment has the parent
/// [`Fragment::HOST`].
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Fragment {
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub(crate) parts: Vec<Part>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub(crate) radials: Vec<RadialPart>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub(crate) internals: Vec<InternalPart>,
    /// Formulas over local ids.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub(crate) formulas: Vec<Formula>,
    /// The next unused local id: ids are never reused, so an id in a newer version means the same member.
    pub(crate) next_id: PartId,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub(crate) host: Option<Host>,
}

/// Every id of the given items in the one canonical order: airframe parts, radial parts, internal parts, then the ids inside
/// each pod's body (parts, radial parts, internal parts).
pub(crate) fn canonical_ids(parts: &[Part], radials: &[RadialPart], internals: &[InternalPart]) -> Vec<PartId> {
    let mut out: Vec<PartId> = parts.iter().map(|p| p.id).chain(radials.iter().map(|r| r.id)).chain(internals.iter().map(|i| i.id)).collect();
    for r in radials {
        if let Some(body) = r.kind.pod_body() {
            out.extend(body.ids());
        }
    }
    out
}

/// The kind of fragment the items make, or why they make none.
pub(crate) fn classify(parts: &[Part], radials: &[RadialPart], internals: &[InternalPart]) -> Result<FragmentKind, AssemblyError> {
    if !parts.is_empty() {
        Ok(FragmentKind::Stack)
    } else if !internals.is_empty() && !radials.is_empty() {
        Err(AssemblyError::MixedClasses)
    } else if !internals.is_empty() {
        Ok(FragmentKind::Internal)
    } else if !radials.is_empty() {
        Ok(FragmentKind::Surface)
    } else {
        Err(AssemblyError::Empty)
    }
}

impl Fragment {
    /// The parent of every radial and internal part of an internal or surface fragment: the host body it is placed on or in.
    pub const HOST: PartId = PartId::MAX;

    /// The airframe parts, fore to aft (a stack fragment's; empty for the other kinds).
    pub fn parts(&self) -> &[Part] {
        &self.parts
    }

    /// The radial parts, in the order they were made.
    pub fn radials(&self) -> &[RadialPart] {
        &self.radials
    }

    /// The internal parts, in the order they were made.
    pub fn internals(&self) -> &[InternalPart] {
        &self.internals
    }

    /// The formulas, over the fragment's local ids.
    pub fn formulas(&self) -> &[Formula] {
        &self.formulas
    }

    /// The next local id that has never been used.
    pub fn next_id(&self) -> PartId {
        self.next_id
    }

    /// The host an internal or surface fragment was made in; `None` for a stack.
    pub fn host(&self) -> Option<&Host> {
        self.host.as_ref()
    }

    /// The kind of assembly: a stack when there are airframe parts, else internal parts, else radial parts.
    pub fn kind(&self) -> FragmentKind {
        if !self.parts.is_empty() {
            FragmentKind::Stack
        } else if !self.internals.is_empty() {
            FragmentKind::Internal
        } else {
            FragmentKind::Surface
        }
    }

    /// Every local id, in the canonical order (airframe, radial, internal, then those inside pods).
    pub fn ids(&self) -> Vec<PartId> {
        canonical_ids(&self.parts, &self.radials, &self.internals)
    }

    /// Where an internal or surface fragment starts along its host: the fore edge of its fore-most member, from the host's fore
    /// face as it was made. 0 for a stack.
    pub fn origin(&self) -> f64 {
        if !self.parts.is_empty() {
            return 0.0;
        }
        let internal = self.internals.iter().map(|i| i.offset);
        let fore = internal.chain(self.radials.iter().map(|r| r.offset)).fold(f64::INFINITY, f64::min);
        if fore.is_finite() {
            fore
        } else {
            0.0
        }
    }

    /// How far along the axis the fragment reaches: a stack's length, else the distance from the origin to the aft edge of the
    /// aft-most member.
    pub fn extent(&self) -> f64 {
        if self.parts.is_empty() {
            let internal = self.internals.iter().map(|i| i.offset + i.kind.extent());
            let radial = self.radials.iter().map(|r| r.offset + r.kind.extent());
            internal.chain(radial).fold(0.0, f64::max) - self.origin()
        } else {
            self.parts.iter().map(|p| p.kind.length()).sum()
        }
    }

    /// Checks that the fragment is a well-formed definition: not empty, ids unique and below [`next_id`](Self::next_id), one
    /// kind, parents inside it (or the host), sizes finite, formulas inside, the stack rules for a nose cone.
    ///
    /// # Errors
    /// [`AssemblyError::Invalid`] with what is wrong, [`AssemblyError::Empty`] or [`AssemblyError::MixedClasses`].
    pub fn check(&self) -> Result<(), AssemblyError> {
        let kind = classify(&self.parts, &self.radials, &self.internals)?;
        let bad = |why: String| Err(AssemblyError::Invalid(why));
        let ids = self.ids();
        let set: BTreeSet<PartId> = ids.iter().copied().collect();
        if set.len() != ids.len() {
            return bad("an id is used twice".to_string());
        }
        if set.contains(&Fragment::HOST) || ids.iter().any(|&id| id >= self.next_id) {
            return bad("an id is not below the next unused id".to_string());
        }
        let finite = |v: f64| v.is_finite() && v >= 0.0;
        for (i, p) in self.parts.iter().enumerate() {
            let sizes = match p.kind {
                PartKind::NoseCone { length, diameter, tip_radius, .. } => [length, diameter, tip_radius],
                PartKind::BodyTube { length, diameter } => [length, diameter, 0.0],
                PartKind::Transition { length, fore_diameter, aft_diameter } => [length, fore_diameter, aft_diameter],
            };
            if !sizes.into_iter().all(finite) || !finite(p.thickness) {
                return bad(format!("part {} has a negative or non-finite size", p.id));
            }
            if matches!(p.kind, PartKind::NoseCone { .. }) && i != 0 {
                return bad(format!("nose cone {} is not first", p.id));
            }
        }
        let host_or = |parent: PartId, inside: &dyn Fn(PartId) -> bool| match kind {
            FragmentKind::Stack => inside(parent),
            _ => parent == Fragment::HOST,
        };
        let in_parts = |id: PartId| self.parts.iter().any(|p| p.id == id);
        for r in &self.radials {
            if r.count == 0 || r.count > MAX_COUNT || !finite(r.offset) || !finite(r.kind.extent()) {
                return bad(format!("radial part {} has a bad count or size", r.id));
            }
            if !host_or(r.parent, &in_parts) {
                return bad(format!("radial part {} is on a part outside the assembly", r.id));
            }
            if let RadialKind::Pod { body, .. } = &r.kind {
                let inner: BTreeSet<PartId> = body.parts.iter().map(|p| p.id).collect();
                if body.radials.iter().any(|b| b.kind.is_pod() || !inner.contains(&b.parent)) || body.internals.iter().any(|b| !inner.contains(&b.parent)) {
                    return bad(format!("pod {} is not well-formed", r.id));
                }
            }
        }
        for i in &self.internals {
            if !finite(i.offset) || !finite(i.kind.extent()) {
                return bad(format!("internal part {} has a negative or non-finite size", i.id));
            }
            if !host_or(i.parent, &in_parts) {
                return bad(format!("internal part {} is in a part outside the assembly", i.id));
            }
        }
        match (kind, &self.host) {
            (FragmentKind::Stack, None) => {}
            (FragmentKind::Stack, Some(_)) => return bad("a stack has no host".to_string()),
            (_, None) => return bad("an internal or surface assembly records the host it was made in".to_string()),
            (_, Some(h)) => {
                if !(h.outer_diameter.is_finite() && h.outer_diameter > 0.0 && finite(h.thickness) && h.length.is_finite() && h.length > 0.0) {
                    return bad("the recorded host has no size".to_string());
                }
            }
        }
        for f in &self.formulas {
            let target_ok = set.contains(&f.target.part);
            if !target_ok || f.expr.refs().iter().any(|r| !set.contains(&r.part)) {
                return bad("a formula refers outside the assembly".to_string());
            }
        }
        Ok(())
    }
}

/// `f` applied to every part id the radial part mentions: its own, its parent's and everything inside its pod body.
pub(crate) fn remap_radial(r: &mut RadialPart, f: &dyn Fn(PartId) -> PartId) {
    r.id = f(r.id);
    r.parent = f(r.parent);
    if let RadialKind::Pod { body, .. } = &mut r.kind {
        for p in &mut body.parts {
            p.id = f(p.id);
        }
        for b in &mut body.radials {
            b.id = f(b.id);
            b.parent = f(b.parent);
        }
        for b in &mut body.internals {
            b.id = f(b.id);
            b.parent = f(b.parent);
        }
    }
}

/// `f` applied to an internal part's id and parent.
pub(crate) fn remap_internal(i: &mut InternalPart, f: &dyn Fn(PartId) -> PartId) {
    i.id = f(i.id);
    i.parent = f(i.parent);
}

/// The expression with every referenced part id passed through `f`.
pub(crate) fn remap_expr(e: &Expr, f: &dyn Fn(PartId) -> PartId) -> Expr {
    match e {
        Expr::Num(v) => Expr::Num(*v),
        Expr::Ref(r) => Expr::Ref(FieldRef { part: f(r.part), dim: r.dim }),
        Expr::Neg(inner) => Expr::Neg(Box::new(remap_expr(inner, f))),
        Expr::Bin(l, op, r) => Expr::Bin(Box::new(remap_expr(l, f)), *op, Box::new(remap_expr(r, f))),
    }
}

/// The formula with its target, references and remembered names passed through `f`.
pub(crate) fn remap_formula(formula: &Formula, f: &dyn Fn(PartId) -> PartId) -> Formula {
    Formula {
        target: FieldRef { part: f(formula.target.part), dim: formula.target.dim },
        expr: remap_expr(&formula.expr, f),
        names: formula.names.iter().map(|(id, name)| (f(*id), name.clone())).collect(),
    }
}
