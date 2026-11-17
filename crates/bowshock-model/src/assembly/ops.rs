//! The operations on a [`Rocket`] that make, place, move, update and remove assemblies (see the [module docs](super)).
//!
//! Every operation that can fail part way (placing, moving, updating) works on a clone of the document and swaps it in when it
//! succeeds, so an error leaves the document as it was.

use std::collections::{BTreeMap, BTreeSet};
use std::f64::consts::TAU;

use super::fragment::{remap_formula, remap_internal, remap_radial};
use super::{AssemblyDef, AssemblyError, AssemblyId, AssemblyInstance, Fragment, FragmentKind, Host, InstanceId};
use crate::geometry::{Part, PartKind, RadialKind, RadialPart};
use crate::internal::{InternalKind, InternalPart};
use crate::rocket::Rocket;
use crate::PartId;

/// Lengths closer than this (a nanometre) are the same.
const EPS: f64 = 1e-9;
/// A member whose given outer diameter is within this (0.2 mm) of the host's bore, or beyond it, touched the wall.
const WALL_TOUCH: f64 = 2e-4;

/// Where an assembly goes, by kind.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Placement {
    /// A stack assembly: its first part goes at `index` of the stack (the rocket's, or of pod set `pod`), counting only the
    /// parts that are not the assembly's own (what an ordinary insert means; when moving an instance, the stack with the
    /// instance taken out). The assembly keeps its own diameters.
    Stack {
        /// The pod set whose stack it goes in; `None` for the rocket's own.
        pod: Option<PartId>,
        /// The node index: parts before it.
        index: usize,
    },
    /// An internal assembly: inside axial part `host`, its fore end `offset` behind the host's fore face (limited so it lies
    /// within the host).
    Internal {
        /// The host tube.
        host: PartId,
        /// Distance from the host's fore face to the fragment's origin.
        offset: f64,
    },
    /// A surface assembly: on axial part `host`, its origin `offset` behind the host's fore face (limited so every member
    /// stays on it) and the first radial part at `angle` (radians, 0 is +Y towards +X), the others keeping their angles
    /// relative to it.
    Surface {
        /// The host body.
        host: PartId,
        /// Distance from the host's fore face to the fragment's origin.
        offset: f64,
        /// Angle of the first radial part.
        angle: f64,
    },
}

impl Placement {
    /// The kind of assembly this placement is for.
    pub fn kind(&self) -> FragmentKind {
        match self {
            Placement::Stack { .. } => FragmentKind::Stack,
            Placement::Internal { .. } => FragmentKind::Internal,
            Placement::Surface { .. } => FragmentKind::Surface,
        }
    }
}

/// How the members of an internal assembly fit the host they were put in.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Fit {
    /// Members whose given outer diameter touched the wall of the host they were made in and now follow the new host's wall
    /// (they were resized; `AUTO` members follow by themselves and are not listed).
    pub resized: Vec<PartId>,
    /// Members that do not fit the host: wider than its bore, or longer than it. They are placed, with their own size.
    pub unfit: Vec<PartId>,
}

/// The result of [`Rocket::place`].
#[derive(Clone, Debug, PartialEq)]
pub struct Placed {
    /// The new instance.
    pub instance: InstanceId,
    /// How its members fit.
    pub fit: Fit,
}

/// What [`Rocket::update_assembly`] did to one instance.
#[derive(Clone, Debug, PartialEq)]
pub struct InstanceUpdate {
    /// The instance.
    pub instance: InstanceId,
    /// Members that are new (new ids).
    pub added: Vec<PartId>,
    /// Members the new definition no longer has (deleted).
    pub removed: Vec<PartId>,
    /// Members that kept their id (re-made from the new definition).
    pub kept: Vec<PartId>,
    /// How an internal assembly's members fit the host.
    pub fit: Fit,
}

/// What a selection resolved to.
struct Selection {
    kind: FragmentKind,
    host: Option<PartId>,
    parts: Vec<PartId>,
    radials: Vec<PartId>,
    internals: Vec<PartId>,
    /// Every id in the canonical order: the fragment's local id of each is its index.
    order: Vec<PartId>,
}

fn with_outer_diameter(kind: &InternalKind, d: f64) -> InternalKind {
    let mut out = kind.clone();
    match &mut out {
        InternalKind::InnerTube { outer_diameter, .. } => *outer_diameter = d,
        InternalKind::CenteringRing { outer_diameter, .. } | InternalKind::Coupler { outer_diameter, .. } => *outer_diameter = Some(d),
        InternalKind::Bulkhead { diameter, .. } | InternalKind::MassComponent { diameter, .. } => *diameter = Some(d),
        InternalKind::Parachute { packed_diameter, .. } => *packed_diameter = Some(d),
        _ => {}
    }
    out
}

/// The kind resized to follow a wall that moved from bore `old` to `new`, if it touched the old wall and has a given diameter.
fn follow_wall(kind: &InternalKind, old: f64, new: f64) -> Option<InternalKind> {
    let d = kind.given_outer_diameter()?;
    if old <= 0.0 || (new - old).abs() < 1e-12 || d + WALL_TOUCH < old {
        return None;
    }
    let resized = d - old + new;
    (resized > 0.0).then(|| with_outer_diameter(kind, resized))
}

impl Rocket {
    // ---- queries ----------------------------------------------------------------------------------------------------

    /// The assembly definitions of the document: those with instances and those on the shelf.
    pub fn assemblies(&self) -> &[AssemblyDef] {
        &self.assemblies
    }

    /// The definition with `identity`.
    pub fn assembly(&self, identity: &AssemblyId) -> Option<&AssemblyDef> {
        self.assemblies.iter().find(|d| &d.identity == identity)
    }

    /// Every instance, in the order they were made.
    pub fn instances(&self) -> &[AssemblyInstance] {
        &self.instances
    }

    /// The instance with `id`.
    pub fn instance(&self, id: InstanceId) -> Option<&AssemblyInstance> {
        self.instances.iter().find(|i| i.id == id)
    }

    /// The instances of one definition.
    pub fn instances_of<'a>(&'a self, identity: &'a AssemblyId) -> impl Iterator<Item = &'a AssemblyInstance> + 'a {
        self.instances.iter().filter(move |i| &i.assembly == identity)
    }

    /// The instance part `part` is a member of (any member, including the parts inside a member pod).
    pub fn instance_of(&self, part: PartId) -> Option<&AssemblyInstance> {
        self.instances.iter().find(|i| i.members.values().any(|&m| m == part))
    }

    /// Whether `part` is a member of an instance. A member is edited by editing its definition ([`AssemblyDef::to_rocket`]), so
    /// a UI refuses direct edits of members and asks this first; the setters themselves do not check.
    pub fn is_member(&self, part: PartId) -> bool {
        self.instance_of(part).is_some()
    }

    /// The ids of the parts that make up `instance`, in the order of the definition's local ids (those that exist).
    pub fn members(&self, instance: InstanceId) -> Vec<PartId> {
        self.instance(instance).map_or_else(Vec::new, |i| i.members.values().copied().filter(|&id| self.kind_of(id).is_some()).collect())
    }

    /// The stand-in host tube of a document made by [`AssemblyDef::to_rocket`] for an internal or surface assembly.
    pub fn stand_in(&self) -> Option<PartId> {
        self.stand_in
    }

    /// Where `instance` is now, in the terms [`move_instance`](Self::move_instance) takes. `None` for an unknown instance or one
    /// with no member left.
    pub fn placement(&self, instance: InstanceId) -> Option<Placement> {
        let inst = self.instance(instance)?;
        self.placement_of(inst, &self.assembly(&inst.assembly)?.fragment)
    }

    fn placement_of(&self, inst: &AssemblyInstance, frag: &Fragment) -> Option<Placement> {
        let real = |local: PartId| inst.members.get(&local).copied();
        match frag.kind() {
            FragmentKind::Stack => {
                let members: BTreeSet<PartId> = inst.members.values().copied().collect();
                let first = frag.parts.iter().filter_map(|p| real(p.id)).find(|&id| self.part(id).is_some())?;
                let pod = self.pod_of(first).map(|p| p.id);
                let index = self.stack_of(pod).iter().position(|p| members.contains(&p.id))?;
                Some(Placement::Stack { pod, index })
            }
            FragmentKind::Internal => {
                let held: Vec<&InternalPart> = frag.internals.iter().filter_map(|i| self.internal(real(i.id)?)).collect();
                let offset = held.iter().map(|i| i.offset).fold(f64::INFINITY, f64::min);
                Some(Placement::Internal { host: held.first()?.parent, offset })
            }
            FragmentKind::Surface => {
                let held: Vec<&RadialPart> = frag.radials.iter().filter_map(|r| self.radial(real(r.id)?)).collect();
                let offset = held.iter().map(|r| r.offset).fold(f64::INFINITY, f64::min);
                let first = held.first()?;
                Some(Placement::Surface { host: first.parent, offset, angle: first.angle })
            }
        }
    }

    /// The nodes of a stack assembly: the indices of the stack of `pod` (`None` for the rocket's) where assembly `identity` may
    /// go, each with its station along that stack from its fore face (a pod's own station is [`Rocket::pod_frame`]'s). With
    /// `except` an instance's parts are left out of the stack, as when moving it. Empty for other kinds.
    pub fn assembly_nodes(&self, identity: &AssemblyId, pod: Option<PartId>, except: Option<InstanceId>) -> Vec<(usize, f64)> {
        let Some(def) = self.assembly(identity).filter(|d| d.kind() == FragmentKind::Stack) else {
            return Vec::new();
        };
        let skip: BTreeSet<PartId> = except.and_then(|i| self.instance(i)).map(|i| i.members.values().copied().collect()).unwrap_or_default();
        let stack: Vec<&Part> = self.stack_of(pod).iter().filter(|p| !skip.contains(&p.id)).collect();
        let has_nose = stack.first().is_some_and(|p| matches!(p.kind, PartKind::NoseCone { .. }));
        let brings_nose = def.fragment.parts.first().is_some_and(|p| matches!(p.kind, PartKind::NoseCone { .. }));
        let mut out = Vec::new();
        let mut z = 0.0;
        for i in 0..=stack.len() {
            let ok = if brings_nose { i == 0 && !has_nose } else { !(i == 0 && has_nose) };
            if ok {
                out.push((i, z));
            }
            if let Some(p) = stack.get(i) {
                z += p.kind.length();
            }
        }
        out
    }

    // ---- making definitions and instances --------------------------------------------------------------------------

    /// Makes an assembly of the chosen parts and turns them, in place, into its first instance: nothing moves, no id changes.
    /// The selection is closed over what the airframe parts carry (their radial and internal parts, and a pod's body); the
    /// definition starts at version 1 with a new identity. Formulas whose target and references are all members go into the
    /// definition; a formula of a member that refers outside is dropped from the document (the field keeps the value it has).
    ///
    /// # Errors
    /// [`AssemblyError`]: [`NotContiguous`](AssemblyError::NotContiguous) for airframe parts that are not one run,
    /// [`AcrossStacks`](AssemblyError::AcrossStacks), [`NotCarried`](AssemblyError::NotCarried),
    /// [`MixedClasses`](AssemblyError::MixedClasses) and [`DifferentHosts`](AssemblyError::DifferentHosts) for parts that make no
    /// one kind, [`AlreadyMember`](AssemblyError::AlreadyMember), [`NoSuchPart`](AssemblyError::NoSuchPart),
    /// [`Empty`](AssemblyError::Empty).
    pub fn group(&mut self, ids: &[PartId], name: &str) -> Result<InstanceId, AssemblyError> {
        let selection = self.select(ids)?;
        let (fragment, members) = self.build_fragment(&selection);
        fragment.check()?;
        let real: BTreeSet<PartId> = members.values().copied().collect();
        // A formula of a member that refers outside is frozen: the field keeps its value, the formula goes.
        self.formulas.retain(|f| !real.contains(&f.target.part) || f.expr.refs().iter().all(|r| real.contains(&r.part)));
        let identity = AssemblyId::random();
        self.assemblies.push(AssemblyDef { identity: identity.clone(), version: 1, name: name.to_string(), fragment });
        let id = self.fresh_id();
        self.instances.push(AssemblyInstance { id, assembly: identity, version: 1, members });
        Ok(id)
    }

    /// Adds a definition (from the library, another document, [`AssemblyDef::from_rocket`]) to the document's shelf with no
    /// instance.
    ///
    /// # Errors
    /// [`Duplicate`](AssemblyError::Duplicate) if the identity is already here (use [`update_assembly`](Self::update_assembly)),
    /// [`Invalid`](AssemblyError::Invalid) for a fragment that is not well-formed.
    pub fn add_assembly(&mut self, def: AssemblyDef) -> Result<(), AssemblyError> {
        if self.assembly(&def.identity).is_some() {
            return Err(AssemblyError::Duplicate(def.identity));
        }
        def.fragment.check()?;
        self.assemblies.push(def);
        Ok(())
    }

    /// A copy of the definition under a new identity, at version 1, named `name` (a variant). Returns the new identity.
    ///
    /// # Errors
    /// [`NoSuchAssembly`](AssemblyError::NoSuchAssembly).
    pub fn duplicate_assembly(&mut self, identity: &AssemblyId, name: &str) -> Result<AssemblyId, AssemblyError> {
        let mut def = self.assembly(identity).ok_or_else(|| AssemblyError::NoSuchAssembly(identity.clone()))?.clone();
        def.identity = AssemblyId::random();
        def.version = 1;
        def.name = name.to_string();
        let fresh = def.identity.clone();
        self.assemblies.push(def);
        Ok(fresh)
    }

    /// Removes a definition that has no instance.
    ///
    /// # Errors
    /// [`InUse`](AssemblyError::InUse) while instances exist, [`NoSuchAssembly`](AssemblyError::NoSuchAssembly).
    pub fn remove_assembly(&mut self, identity: &AssemblyId) -> Result<(), AssemblyError> {
        self.assembly(identity).ok_or_else(|| AssemblyError::NoSuchAssembly(identity.clone()))?;
        let instances = self.instances_of(identity).count();
        if instances > 0 {
            return Err(AssemblyError::InUse { instances });
        }
        self.assemblies.retain(|d| &d.identity != identity);
        Ok(())
    }

    // ---- placing, moving, updating ----------------------------------------------------------------------------------

    /// Materialises a new instance of assembly `identity` at `at`, with new ids for every member and its formulas bound to them.
    /// Returns the instance and, for an internal assembly, how its members fit the host ([`Fit`]).
    ///
    /// # Errors
    /// [`WrongTarget`](AssemblyError::WrongTarget) if `at` is for another kind, [`NoSuchHost`](AssemblyError::NoSuchHost),
    /// [`TooLong`](AssemblyError::TooLong), [`PodInPod`](AssemblyError::PodInPod), or the stack rules
    /// ([`Insert`](AssemblyError::Insert): index out of range, a nose that is not first, a second nose).
    pub fn place(&mut self, identity: &AssemblyId, at: Placement) -> Result<Placed, AssemblyError> {
        let def = self.assembly(identity).ok_or_else(|| AssemblyError::NoSuchAssembly(identity.clone()))?.clone();
        let mut work = self.clone();
        let members = work.fresh_map(&def.fragment, &BTreeMap::new());
        let fit = work.materialise(&def.fragment, &members, at)?;
        work.bind_formulas(&def.fragment, &members);
        work.apply_formulas();
        let instance = work.fresh_id();
        work.instances.push(AssemblyInstance { id: instance, assembly: def.identity, version: def.version, members });
        *self = work;
        Ok(Placed { instance, fit })
    }

    /// Moves every member of `instance` as one, keeping their relative positions, with the rules of the single-part moves: a
    /// stack goes to another index of its own stack ([`Placement::Stack`] counts the stack without the instance; the stack rules
    /// for a nose cone apply, a stack does not change between the rocket and a pod), internal parts go to another host tube of
    /// the same body (`move_internal`), radial parts to another host body (`move_radial`; a pod set only onto the core). The
    /// members keep their ids, values, motors and formulas. One call, one consistent document; on an error nothing moved.
    ///
    /// # Errors
    /// As [`place`](Self::place), plus [`AcrossPods`](AssemblyError::AcrossPods) and
    /// [`BrokenInstance`](AssemblyError::BrokenInstance).
    pub fn move_instance(&mut self, instance: InstanceId, to: Placement) -> Result<Fit, AssemblyError> {
        let inst = self.instance(instance).ok_or(AssemblyError::NoSuchInstance(instance))?.clone();
        let def = self.assembly(&inst.assembly).ok_or_else(|| AssemblyError::NoSuchAssembly(inst.assembly.clone()))?;
        if to.kind() != def.kind() {
            return Err(AssemblyError::WrongTarget { expected: def.kind() });
        }
        if inst.members.values().any(|&id| self.kind_of(id).is_none()) {
            return Err(AssemblyError::BrokenInstance(instance));
        }
        let frag = def.fragment.clone();
        let mut work = self.clone();
        let fit = match to {
            Placement::Stack { pod, index } => work.move_stack(&inst, pod, index).map(|()| Fit::default())?,
            Placement::Internal { host, offset } => work.move_internals(&inst, &frag, host, offset)?,
            Placement::Surface { host, offset, angle } => work.move_radials(&inst, &frag, host, offset, angle).map(|()| Fit::default())?,
        };
        *self = work;
        Ok(fit)
    }

    /// Replaces the stored definition by the newer `def` of the same identity and re-materialises every instance in place: each
    /// keeps its placement (stack position; host and station; host and angle), members keep their ids where the definition-local id
    /// still exists, new members get new ids, members the new definition lacks are deleted, formulas are rebound. Motors stay on
    /// the mounts that survive. Nothing else in the document changes. Parts attached to a member directly (not through the
    /// definition) are removed with the member. One call, one consistent document.
    ///
    /// # Errors
    /// [`NoSuchAssembly`](AssemblyError::NoSuchAssembly), [`NotNewer`](AssemblyError::NotNewer) unless `def.version` is greater
    /// than the stored one, [`KindChanged`](AssemblyError::KindChanged), [`Invalid`](AssemblyError::Invalid), and whatever
    /// re-placing an instance runs into (the new content may not fit where an instance is; nothing changed then).
    pub fn update_assembly(&mut self, def: AssemblyDef) -> Result<Vec<InstanceUpdate>, AssemblyError> {
        let stored = self.assembly(&def.identity).ok_or_else(|| AssemblyError::NoSuchAssembly(def.identity.clone()))?;
        if def.version <= stored.version {
            return Err(AssemblyError::NotNewer { stored: stored.version, offered: def.version });
        }
        if def.kind() != stored.kind() {
            return Err(AssemblyError::KindChanged { from: stored.kind(), to: def.kind() });
        }
        def.fragment.check()?;
        let mut work = self.clone();
        let saved_configs = work.configs.clone();
        let ids: Vec<InstanceId> = work.instances.iter().filter(|i| i.assembly == def.identity).map(|i| i.id).collect();
        let mut report = Vec::with_capacity(ids.len());
        for id in ids {
            report.push(work.rematerialise(id, &def)?);
        }
        if let Some(slot) = work.assemblies.iter_mut().find(|d| d.identity == def.identity) {
            *slot = def;
        }
        // Re-making a member must not cost it its motor: only the mounts that are really gone lose theirs.
        work.configs = saved_configs;
        work.prune_configs();
        work.prune_formulas();
        work.apply_formulas();
        *self = work;
        Ok(report)
    }

    // ---- ending instances -------------------------------------------------------------------------------------------

    /// Detaches `instance`: its members become ordinary parts (nothing changes in the rocket) and the definition stays. Returns
    /// the members.
    ///
    /// # Errors
    /// [`NoSuchInstance`](AssemblyError::NoSuchInstance).
    pub fn detach(&mut self, instance: InstanceId) -> Result<Vec<PartId>, AssemblyError> {
        let at = self.instances.iter().position(|i| i.id == instance).ok_or(AssemblyError::NoSuchInstance(instance))?;
        let gone = self.instances.remove(at);
        Ok(gone.members.values().copied().filter(|&id| self.kind_of(id).is_some()).collect())
    }

    /// [`detach`](Self::detach), and drops the definition too unless `keep_definition` or another instance still uses it.
    ///
    /// # Errors
    /// [`NoSuchInstance`](AssemblyError::NoSuchInstance).
    pub fn ungroup(&mut self, instance: InstanceId, keep_definition: bool) -> Result<Vec<PartId>, AssemblyError> {
        let identity = self.instance(instance).ok_or(AssemblyError::NoSuchInstance(instance))?.assembly.clone();
        let members = self.detach(instance)?;
        if !keep_definition && self.instances_of(&identity).next().is_none() {
            self.assemblies.retain(|d| d.identity != identity);
        }
        Ok(members)
    }

    /// Deletes `instance` and its members, by the rules of deleting those parts one by one (an airframe part takes what stands on
    /// and in it; motors of removed mounts go). The definition stays.
    ///
    /// # Errors
    /// [`NoSuchInstance`](AssemblyError::NoSuchInstance).
    pub fn delete_instance(&mut self, instance: InstanceId) -> Result<(), AssemblyError> {
        let at = self.instances.iter().position(|i| i.id == instance).ok_or(AssemblyError::NoSuchInstance(instance))?;
        let gone = self.instances.remove(at);
        let ids: Vec<PartId> = gone.members.values().copied().collect();
        // Pod sets first (they take their bodies), then the other radial parts, the airframe parts and the internal parts.
        let mut radials: Vec<PartId> = ids.iter().copied().filter(|&id| self.radial(id).is_some()).collect();
        radials.sort_by_key(|&id| !self.is_pod(id));
        for id in radials {
            self.remove_radial(id);
        }
        for &id in &ids {
            if self.part(id).is_some() {
                self.remove(id);
            }
        }
        for &id in &ids {
            if self.internal(id).is_some() {
                self.remove_internal(id);
            }
        }
        Ok(())
    }

    /// Drops the records of instances that lost a member or their definition (a part taken out and never put back, a damaged
    /// document). The parts that remain are ordinary. Returns the instances dropped.
    pub fn reconcile_instances(&mut self) -> Vec<InstanceId> {
        let keep: Vec<bool> =
            self.instances.iter().map(|i| self.assembly(&i.assembly).is_some() && i.members.values().all(|&id| self.kind_of(id).is_some())).collect();
        let mut dropped = Vec::new();
        let mut flags = keep.into_iter();
        self.instances.retain(|i| {
            let keep = flags.next().unwrap_or(true);
            if !keep {
                dropped.push(i.id);
            }
            keep
        });
        dropped
    }

    /// Detaches every instance that has any of `ids` as a member: the document is about to remove or restructure those parts
    /// by an ordinary operation.
    pub(crate) fn release_touching(&mut self, ids: &[PartId]) {
        if !self.instances.is_empty() {
            self.instances.retain(|i| !i.members.values().any(|m| ids.contains(m)));
        }
    }

    /// `id` and everything that goes with it when it is removed: what stands on or in an airframe part, a pod's body.
    pub(crate) fn carried_ids(&self, id: PartId) -> Vec<PartId> {
        let mut out = vec![id];
        for r in self.radials.iter().chain(self.pod_radials()).filter(|r| r.parent == id) {
            out.push(r.id);
            out.extend(r.kind.pod_body().into_iter().flat_map(|b| b.ids()));
        }
        out.extend(self.internals_of(id).map(|i| i.id));
        out.extend(self.radial(id).and_then(|r| r.kind.pod_body()).into_iter().flat_map(|b| b.ids()));
        out
    }

    /// Checks the assemblies of a loaded document: a damaged definition, or an instance that is not whole, is dropped (its parts
    /// stay ordinary); counters are repaired.
    pub(crate) fn repair_assemblies(&mut self) {
        let mut seen = BTreeSet::new();
        self.assemblies.retain(|d| d.fragment.check().is_ok() && seen.insert(d.identity.clone()));
        let live: BTreeSet<PartId> = self.ids().into_iter().collect();
        let mut used: BTreeSet<PartId> = BTreeSet::new();
        let mut instance_ids: BTreeSet<InstanceId> = BTreeSet::new();
        let mut flags = Vec::with_capacity(self.instances.len());
        for i in &self.instances {
            let whole = self.assembly(&i.assembly).is_some_and(|d| {
                let locals: BTreeSet<PartId> = d.fragment.ids().into_iter().collect();
                locals == i.members.keys().copied().collect::<BTreeSet<_>>()
            }) && i.members.values().all(|id| live.contains(id) && !used.contains(id))
                && i.members.values().collect::<BTreeSet<_>>().len() == i.members.len()
                && instance_ids.insert(i.id);
            if whole {
                used.extend(i.members.values().copied());
            }
            flags.push(whole);
        }
        let mut flags = flags.into_iter();
        self.instances.retain(|_| flags.next().unwrap_or(false));
        if let Some(top) = self.instances.iter().map(|i| i.id).max() {
            self.next_id = self.next_id.max(top + 1);
        }
        if self.stand_in.is_some_and(|id| self.part(id).is_none()) {
            self.stand_in = None;
        }
    }

    // ---- selection and extraction -----------------------------------------------------------------------------------

    /// The airframe stack of the core (`None`) or of pod set `pod`.
    pub(crate) fn stack_of(&self, pod: Option<PartId>) -> &[Part] {
        match pod {
            None => &self.parts,
            Some(p) => self.radial(p).and_then(|r| r.kind.pod_body()).map_or(&[] as &[Part], |b| &b.parts[..]),
        }
    }

    fn all_radials(&self) -> impl Iterator<Item = &RadialPart> {
        self.radials.iter().chain(self.pod_radials())
    }

    fn all_internals(&self) -> impl Iterator<Item = &InternalPart> {
        self.internals.iter().chain(self.pod_internals())
    }

    /// Resolves chosen part ids to the parts of one assembly (see [`group`](Self::group) for the rules).
    fn select(&self, ids: &[PartId]) -> Result<Selection, AssemblyError> {
        if ids.is_empty() {
            return Err(AssemblyError::Empty);
        }
        let mut set: BTreeSet<PartId> = BTreeSet::new();
        for &id in ids {
            if self.kind_of(id).is_none() {
                return Err(AssemblyError::NoSuchPart(id));
            }
            set.insert(id);
        }
        // A part of a pod whose pod set is chosen too comes with the set.
        let covered: Vec<PartId> = set.iter().copied().filter(|&id| self.pod_of(id).is_some_and(|p| set.contains(&p.id))).collect();
        for id in covered {
            set.remove(&id);
        }
        let axial: Vec<PartId> = set.iter().copied().filter(|&id| self.part(id).is_some()).collect();
        let radial: Vec<PartId> = set.iter().copied().filter(|&id| self.radial(id).is_some()).collect();
        let internal: Vec<PartId> = set.iter().copied().filter(|&id| self.internal(id).is_some()).collect();

        let (kind, host, parts, radials, internals) = if let Some(&first) = axial.first() {
            let pod = self.pod_of(first).map(|p| p.id);
            if axial.iter().any(|&a| self.pod_of(a).map(|p| p.id) != pod) {
                return Err(AssemblyError::AcrossStacks);
            }
            let stack = self.stack_of(pod);
            let at: Vec<usize> = stack.iter().enumerate().filter(|(_, p)| set.contains(&p.id)).map(|(i, _)| i).collect();
            let (lo, hi) = (at[0], at[at.len() - 1]);
            if let Some(gap) = stack[lo..=hi].iter().find(|p| !set.contains(&p.id)) {
                return Err(AssemblyError::NotContiguous { gap: gap.id });
            }
            let parts: Vec<PartId> = stack[lo..=hi].iter().map(|p| p.id).collect();
            let held: BTreeSet<PartId> = parts.iter().copied().collect();
            for &id in radial.iter().chain(&internal) {
                if !self.parent_of(id).is_some_and(|p| held.contains(&p)) {
                    return Err(AssemblyError::NotCarried(id));
                }
            }
            let radials = self.all_radials().filter(|r| held.contains(&r.parent)).map(|r| r.id).collect();
            let internals = self.all_internals().filter(|i| held.contains(&i.parent)).map(|i| i.id).collect();
            (FragmentKind::Stack, None, parts, radials, internals)
        } else {
            if !radial.is_empty() && !internal.is_empty() {
                return Err(AssemblyError::MixedClasses);
            }
            let hosts: BTreeSet<PartId> = radial.iter().chain(&internal).filter_map(|&id| self.parent_of(id)).collect();
            let host = match (hosts.len(), hosts.first()) {
                (1, Some(&h)) => h,
                _ => return Err(AssemblyError::DifferentHosts),
            };
            let radials: Vec<PartId> = self.all_radials().filter(|r| set.contains(&r.id)).map(|r| r.id).collect();
            let internals: Vec<PartId> = self.all_internals().filter(|i| set.contains(&i.id)).map(|i| i.id).collect();
            let kind = if internals.is_empty() { FragmentKind::Surface } else { FragmentKind::Internal };
            (kind, Some(host), Vec::new(), radials, internals)
        };

        let mut order = parts.clone();
        order.extend(&radials);
        order.extend(&internals);
        for &r in &radials {
            order.extend(self.radial(r).and_then(|r| r.kind.pod_body()).into_iter().flat_map(|b| b.ids()));
        }
        for &id in &order {
            if let Some(i) = self.instance_of(id) {
                return Err(AssemblyError::AlreadyMember { part: id, instance: i.id });
            }
        }
        Ok(Selection { kind, host, parts, radials, internals, order })
    }

    /// The fragment of a selection, ids renumbered from 0 in the canonical order, with the map from local ids to the real ones.
    fn build_fragment(&self, sel: &Selection) -> (Fragment, BTreeMap<PartId, PartId>) {
        let to_local: BTreeMap<PartId, PartId> = sel.order.iter().enumerate().map(|(i, &real)| (real, i as PartId)).collect();
        let f = |x: PartId| to_local.get(&x).copied().unwrap_or(Fragment::HOST);
        let parts = sel.parts.iter().filter_map(|&id| self.part(id)).map(|p| Part { id: f(p.id), ..p.clone() }).collect();
        let radials = sel
            .radials
            .iter()
            .filter_map(|&id| self.radial(id))
            .map(|r| {
                let mut r = r.clone();
                remap_radial(&mut r, &f);
                r
            })
            .collect();
        let internals = sel
            .internals
            .iter()
            .filter_map(|&id| self.internal(id))
            .map(|i| {
                let mut i = i.clone();
                remap_internal(&mut i, &f);
                i
            })
            .collect();
        let mut fragment = Fragment { parts, radials, internals, formulas: Vec::new(), next_id: sel.order.len() as PartId, host: None };
        debug_assert_eq!(fragment.kind(), sel.kind);
        if let Some(host) = sel.host.and_then(|h| self.part(h)) {
            let mid = fragment.origin() + fragment.extent() / 2.0;
            fragment.host = Some(Host { outer_diameter: 2.0 * host.kind.radius_at(mid), thickness: host.thickness, length: host.kind.length() });
        }
        let real: BTreeSet<PartId> = sel.order.iter().copied().collect();
        fragment.formulas = self
            .formulas
            .iter()
            .filter(|fm| real.contains(&fm.target.part) && fm.expr.refs().iter().all(|r| real.contains(&r.part)))
            .map(|fm| remap_formula(fm, &f))
            .collect();
        let members = sel.order.iter().enumerate().map(|(i, &real)| (i as PartId, real)).collect();
        (fragment, members)
    }

    // ---- materialising ----------------------------------------------------------------------------------------------

    /// The real id of every local id of `frag`: `preferred` where it has one, else a new id.
    fn fresh_map(&mut self, frag: &Fragment, preferred: &BTreeMap<PartId, PartId>) -> BTreeMap<PartId, PartId> {
        frag.ids().into_iter().map(|local| (local, preferred.get(&local).copied().unwrap_or_else(|| self.fresh_id()))).collect()
    }

    /// Takes the members out of the document without any of the bookkeeping of a delete (no motors dropped, no records
    /// touched), together with whatever stands on or in an airframe part that goes.
    fn strip(&mut self, ids: &BTreeSet<PartId>) {
        self.parts.retain(|p| !ids.contains(&p.id));
        self.radials.retain(|r| !ids.contains(&r.id));
        self.internals.retain(|i| !ids.contains(&i.id));
        for r in &mut self.radials {
            if let RadialKind::Pod { body, .. } = &mut r.kind {
                body.parts.retain(|p| !ids.contains(&p.id));
                body.radials.retain(|b| !ids.contains(&b.id));
                body.internals.retain(|b| !ids.contains(&b.id));
            }
        }
        let alive: BTreeSet<PartId> = self.parts.iter().map(|p| p.id).collect();
        self.radials.retain(|r| alive.contains(&r.parent));
        self.internals.retain(|i| alive.contains(&i.parent));
        for r in &mut self.radials {
            if let RadialKind::Pod { body, .. } = &mut r.kind {
                let inner: BTreeSet<PartId> = body.parts.iter().map(|p| p.id).collect();
                body.radials.retain(|b| inner.contains(&b.parent));
                body.internals.retain(|b| inner.contains(&b.parent));
            }
        }
    }

    /// Puts the members of `frag` into the document with the real ids of `map`, at `at`. The ids must be free.
    fn materialise(&mut self, frag: &Fragment, map: &BTreeMap<PartId, PartId>, at: Placement) -> Result<Fit, AssemblyError> {
        if at.kind() != frag.kind() {
            return Err(AssemblyError::WrongTarget { expected: frag.kind() });
        }
        let id_of = |x: PartId| map.get(&x).copied().unwrap_or(x);
        match at {
            Placement::Stack { pod, index } => {
                if pod.is_some() && frag.radials.iter().any(|r| r.kind.is_pod()) {
                    return Err(AssemblyError::PodInPod);
                }
                for (k, p) in frag.parts.iter().enumerate() {
                    let part = Part { id: id_of(p.id), ..p.clone() };
                    match pod {
                        None => self.restore(index + k, part),
                        Some(pod) => self.restore_in_pod(pod, index + k, part),
                    }
                    .map_err(AssemblyError::Insert)?;
                }
                for r in &frag.radials {
                    let mut r = r.clone();
                    remap_radial(&mut r, &id_of);
                    self.restore_radial(r).map_err(AssemblyError::Radial)?;
                }
                for i in &frag.internals {
                    let mut i = i.clone();
                    remap_internal(&mut i, &id_of);
                    self.restore_internal(i).map_err(AssemblyError::Internal)?;
                }
                Ok(Fit::default())
            }
            Placement::Internal { host, offset } => {
                let host_part = self.part(host).ok_or(AssemblyError::NoSuchHost(host))?;
                let (source, span, length) = (frag.origin(), frag.extent(), host_part.kind.length());
                if span > length + EPS {
                    return Err(AssemblyError::TooLong { needs: span, available: length });
                }
                let origin = offset.clamp(0.0, (length - span).max(0.0));
                let old_bore = frag.host.map_or(0.0, |h| h.bore());
                let new_bore = host_part.inner_diameter_at(origin + span / 2.0);
                let f = |x: PartId| if x == Fragment::HOST { host } else { id_of(x) };
                let mut fit = Fit::default();
                let mut placed = Vec::with_capacity(frag.internals.len());
                for i in &frag.internals {
                    let mut part = i.clone();
                    remap_internal(&mut part, &f);
                    part.offset = origin + (i.offset - source);
                    if let Some(kind) = follow_wall(&part.kind, old_bore, new_bore) {
                        part.kind = kind;
                        fit.resized.push(part.id);
                    }
                    placed.push(part.id);
                    self.restore_internal(part).map_err(AssemblyError::Internal)?;
                }
                fit.unfit = self.unfit_among(&placed, host);
                Ok(fit)
            }
            Placement::Surface { host, offset, angle } => {
                let host_part = self.part(host).ok_or(AssemblyError::NoSuchHost(host))?;
                let length = host_part.kind.length();
                let source = frag.origin();
                let room = frag.radials.iter().map(|r| length - r.kind.min_attached() - (r.offset - source)).fold(f64::INFINITY, f64::min);
                if room < -EPS {
                    return Err(AssemblyError::TooLong { needs: length - room, available: length });
                }
                if self.pod_index(host).is_some() && frag.radials.iter().any(|r| r.kind.is_pod()) {
                    return Err(AssemblyError::PodInPod);
                }
                let origin = offset.clamp(0.0, room.max(0.0));
                let first = frag.radials.first().map_or(0.0, |r| r.angle);
                let f = |x: PartId| if x == Fragment::HOST { host } else { id_of(x) };
                for r in &frag.radials {
                    let mut radial = r.clone();
                    remap_radial(&mut radial, &f);
                    radial.offset = origin + (r.offset - source);
                    radial.angle = (r.angle - first + angle).rem_euclid(TAU);
                    if let Some(list) = radial.instance_angles.as_mut() {
                        list.iter_mut().for_each(|a| *a = (*a - first + angle).rem_euclid(TAU));
                    }
                    self.restore_radial(radial).map_err(AssemblyError::Radial)?;
                }
                Ok(Fit::default())
            }
        }
    }

    /// The internal parts among `ids` that do not fit the bore and length of `host` where they are.
    fn unfit_among(&self, ids: &[PartId], host: PartId) -> Vec<PartId> {
        ids.iter().copied().filter(|&id| self.internal(id).is_some_and(|p| !self.internal_fits(&p.kind, host, p.offset))).collect()
    }

    /// Adds the formulas of `frag` to the document with the real ids of `map`.
    fn bind_formulas(&mut self, frag: &Fragment, map: &BTreeMap<PartId, PartId>) {
        let f = |x: PartId| map.get(&x).copied().unwrap_or(x);
        for formula in &frag.formulas {
            let formula = remap_formula(formula, &f);
            self.formulas.retain(|x| x.target != formula.target);
            self.formulas.push(formula);
        }
    }

    /// Re-makes the members of one instance from `def` where they are.
    fn rematerialise(&mut self, instance: InstanceId, def: &AssemblyDef) -> Result<InstanceUpdate, AssemblyError> {
        let inst = self.instance(instance).ok_or(AssemblyError::NoSuchInstance(instance))?.clone();
        let old = self.assembly(&inst.assembly).ok_or_else(|| AssemblyError::NoSuchAssembly(inst.assembly.clone()))?;
        let at = self.placement_of(&inst, &old.fragment).ok_or(AssemblyError::BrokenInstance(instance))?;
        let old_real: BTreeSet<PartId> = inst.members.values().copied().collect();
        let existing: Vec<PartId> = old_real.iter().copied().filter(|&id| self.kind_of(id).is_some()).collect();
        let members = self.fresh_map(&def.fragment, &inst.members);
        self.formulas.retain(|f| !old_real.contains(&f.target.part));
        self.strip(&old_real);
        let fit = self.materialise(&def.fragment, &members, at)?;
        self.bind_formulas(&def.fragment, &members);
        let new_real: BTreeSet<PartId> = members.values().copied().collect();
        if let Some(record) = self.instances.iter_mut().find(|i| i.id == instance) {
            record.members = members;
            record.version = def.version;
        }
        Ok(InstanceUpdate {
            instance,
            added: new_real.iter().copied().filter(|id| !old_real.contains(id)).collect(),
            removed: existing.iter().copied().filter(|id| !new_real.contains(id)).collect(),
            kept: existing.iter().copied().filter(|id| new_real.contains(id)).collect(),
            fit,
        })
    }

    // ---- moving -----------------------------------------------------------------------------------------------------

    fn move_stack(&mut self, inst: &AssemblyInstance, pod: Option<PartId>, index: usize) -> Result<(), AssemblyError> {
        let parts: BTreeSet<PartId> = inst.members.values().copied().filter(|&id| self.part(id).is_some()).collect();
        let first = *parts.iter().next().ok_or(AssemblyError::BrokenInstance(inst.id))?;
        let from = self.pod_of(first).map(|p| p.id);
        if parts.iter().any(|&id| self.pod_of(id).map(|p| p.id) != from) {
            return Err(AssemblyError::AcrossStacks);
        }
        if from != pod {
            return Err(AssemblyError::AcrossPods);
        }
        match pod {
            None => move_run(self, &parts, index),
            Some(p) => self.with_pod_set(p, |sub| move_run(sub, &parts, index)).ok_or(AssemblyError::NoSuchHost(p))?,
        }
    }

    fn move_internals(&mut self, inst: &AssemblyInstance, frag: &Fragment, host: PartId, offset: f64) -> Result<Fit, AssemblyError> {
        let parts: Vec<InternalPart> = frag.internals.iter().filter_map(|i| inst.members.get(&i.id)).filter_map(|&id| self.internal(id).cloned()).collect();
        let from = parts.first().ok_or(AssemblyError::BrokenInstance(inst.id))?.parent;
        let to = self.part(host).ok_or(AssemblyError::NoSuchHost(host))?;
        if self.pod_index(from) != self.pod_index(host) {
            return Err(AssemblyError::AcrossPods);
        }
        let source = parts.iter().map(|p| p.offset).fold(f64::INFINITY, f64::min);
        let span = parts.iter().map(|p| p.offset + p.kind.extent()).fold(0.0, f64::max) - source;
        let length = to.kind.length();
        if span > length + EPS {
            return Err(AssemblyError::TooLong { needs: span, available: length });
        }
        let origin = offset.clamp(0.0, (length - span).max(0.0));
        let new_bore = to.inner_diameter_at(origin + span / 2.0);
        let old_bore = self.part(from).map_or(0.0, |h| h.inner_diameter_at(source + span / 2.0));
        let mut fit = Fit::default();
        for p in &parts {
            self.move_internal(p.id, host, origin + (p.offset - source)).map_err(AssemblyError::Internal)?;
            if let Some(kind) = follow_wall(&p.kind, old_bore, new_bore) {
                if self.set_internal_kind(p.id, kind) {
                    fit.resized.push(p.id);
                }
            }
        }
        let ids: Vec<PartId> = parts.iter().map(|p| p.id).collect();
        fit.unfit = self.unfit_among(&ids, host);
        Ok(fit)
    }

    fn move_radials(&mut self, inst: &AssemblyInstance, frag: &Fragment, host: PartId, offset: f64, angle: f64) -> Result<(), AssemblyError> {
        let radials: Vec<RadialPart> = frag.radials.iter().filter_map(|r| inst.members.get(&r.id)).filter_map(|&id| self.radial(id).cloned()).collect();
        let first = radials.first().ok_or(AssemblyError::BrokenInstance(inst.id))?;
        let to = self.part(host).ok_or(AssemblyError::NoSuchHost(host))?;
        if self.pod_index(first.parent) != self.pod_index(host) {
            return Err(AssemblyError::AcrossPods);
        }
        let length = to.kind.length();
        let source = radials.iter().map(|r| r.offset).fold(f64::INFINITY, f64::min);
        let room = radials.iter().map(|r| length - r.kind.min_attached() - (r.offset - source)).fold(f64::INFINITY, f64::min);
        if room < -EPS {
            return Err(AssemblyError::TooLong { needs: length - room, available: length });
        }
        let origin = offset.clamp(0.0, room.max(0.0));
        let turn = angle - first.angle;
        for r in &radials {
            self.move_radial(r.id, host, origin + (r.offset - source), (r.angle + turn).rem_euclid(TAU)).map_err(AssemblyError::Radial)?;
        }
        Ok(())
    }
}

/// Moves the run of airframe parts `ids` of the stack of `rocket` (the core, or a pod's body as a rocket) to `index` of the
/// stack without them, carrying what stands on and in them, by the rules of the stack (a nose cone only first, one only).
fn move_run(rocket: &mut Rocket, ids: &BTreeSet<PartId>, index: usize) -> Result<(), AssemblyError> {
    let at: Vec<usize> = rocket.parts.iter().enumerate().filter(|(_, p)| ids.contains(&p.id)).map(|(i, _)| i).collect();
    let (Some(&lo), Some(&hi)) = (at.first(), at.last()) else {
        return Err(AssemblyError::Empty);
    };
    if let Some(gap) = rocket.parts[lo..=hi].iter().find(|p| !ids.contains(&p.id)) {
        return Err(AssemblyError::NotContiguous { gap: gap.id });
    }
    let run: Vec<Part> = rocket.parts.drain(lo..=hi).collect();
    let (mut radials, mut internals) = (Vec::new(), Vec::new());
    for p in &run {
        radials.extend(rocket.take_radials(p.id));
        internals.extend(rocket.take_internals(p.id));
    }
    for (k, part) in run.into_iter().enumerate() {
        rocket.restore(index + k, part).map_err(AssemblyError::Insert)?;
    }
    for r in radials {
        rocket.restore_radial(r).map_err(AssemblyError::Radial)?;
    }
    for i in internals {
        rocket.restore_internal(i).map_err(AssemblyError::Internal)?;
    }
    Ok(())
}
