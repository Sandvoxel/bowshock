//! Reusable assemblies: build a bay, a stage or a set of fins once, use it in any rocket.
//!
//! # The model: materialised instances
//!
//! The rocket keeps its flat `parts` / `radials` / `internals` exactly as without assemblies, so mass, aerodynamics, stages, the
//! 3D view and every other reader see ordinary parts. On top of them the document holds
//!
//! * [`AssemblyDef`]s ([`Rocket::assemblies`]): a permanent [`AssemblyId`], a `version`, a name and a [`Fragment`], the
//!   definition's own parts, radial parts and internal parts with ids local to the definition (see below), and
//! * [`AssemblyInstance`]s ([`Rocket::instances`]): one use of a definition, with the map from each definition-local id to the
//!   ordinary [`PartId`] of the member in the rocket. Instance ids are drawn from the part id space, so they never equal a part id.
//!
//! A definition is the truth about what its members are. Editing the members is done by editing the definition
//! ([`AssemblyDef::to_rocket`] / [`AssemblyDef::from_rocket`] / [`Rocket::update_assembly`], which re-materialises every instance
//! in place); nothing in the model refuses a direct edit of a member (the setters stay as they are), the UI asks
//! [`Rocket::is_member`] first. A direct edit lasts only until the next update or move of its instance.
//!
//! # Fragments and their three kinds
//!
//! The kind is derived from what the fragment holds ([`FragmentKind`]):
//!
//! * **Stack**: a contiguous run of airframe parts of one stack (the core's, or one pod's) with every radial part and internal
//!   part they carry. Local ids; radial and internal parts point at their local airframe part; the origin is the fore face of
//!   the first part. Goes at a stack index ([`Placement::Stack`]), the same index rules as an ordinary insert (a nose cone only
//!   first, one per stack), but **without** matching diameters to the neighbours: an assembly keeps its own geometry. Internal parts
//!   are always inside a single airframe part in this model, so a part that "reaches into the neighbours" does not exist.
//! * **Internal**: internal parts sharing one host tube and no airframe part. Their parent is the stand-in [`Fragment::HOST`]; the
//!   origin is the fore end of the fore-most member and positions keep their distances from it. Goes into a host tube at a station
//!   ([`Placement::Internal`]). A member whose diameter is `AUTO` follows the host like any internal part; a member whose given outer
//!   diameter touched the wall of the host it was made in follows the new host's wall (keeping its clearance); every other member keeps
//!   its size, and one that does not fit is reported ([`Fit`]).
//! * **Surface**: radial parts (fins, lugs, buttons, pods) on one host body. Origin: the fore-most member's fore edge, and the angle
//!   of the first radial part. Goes on a host body at a station and an angle ([`Placement::Surface`]).
//!
//! Positions are stored as they were in the host the fragment was made in (so extracting and putting back is bit-exact); the
//! origin is derived ([`Fragment::origin`]) and everything is placed by its distance from it.
//!
//! Anything else is refused with an [`AssemblyError`] that says why. Pods: a pod set is a radial part, so it can be a member of a
//! surface fragment or be carried by a stack's tubes, and its body (parts, fins, internals, all with their ids) travels with it; a stack
//! of a pod's own tubes can be grouped and placed into a pod's stack. Pods do not nest, so a fragment with a pod does not go into a pod.
//! Not supported: an assembly inside an assembly, a member shared by two instances, and moving an instance from the core into a pod
//! or from one pod to another (the single-part moves refuse it too).
//!
//! # What the rocket keeps consistent
//!
//! Motor assignments are not part of a fragment (a motor mount is): they stay on the rocket by mount id, so they survive an update
//! that keeps the mount. A formula is part of a fragment when its target and every reference are members; a formula that refers outside
//! is dropped when the assembly is made and its target keeps the value it had (frozen). Formulas bind to the instance's own ids on
//! every placement.
//!
//! Ordinary operations that remove or restructure members **detach the instance first** (so no instance is ever half there):
//! [`Rocket::remove`], [`Rocket::remove_radial`], [`Rocket::remove_internal`] (also through the removal of the part they stand on
//! or in), [`Rocket::split_tube`], [`Rocket::detach_radial`], [`Rocket::split_all_radial`] and [`Rocket::rejoin_radial`].
//! Taking parts out and putting them back with the same ids (`take_*` / `restore_*`, how a drag carries parts) keeps the record;
//! [`Rocket::reconcile_instances`] drops the records of instances that lost a member some other way. Loading a document repairs
//! the same way (a damaged definition or instance is dropped and its parts stay ordinary). Undo is the document clone, which carries
//! all of this along.

mod edit;
mod fragment;
mod ops;
#[cfg(test)]
mod tests;

use std::collections::BTreeMap;
use std::fmt;

use serde::{Deserialize, Serialize};

use crate::format::{pack_as, unpack_as, FormatError};
use crate::rocket::{InsertError, InternalError, RadialError};
use crate::PartId;

pub use fragment::{Fragment, FragmentKind, Host};
pub use ops::{Fit, InstanceUpdate, Placed, Placement};

/// Identifies one use of an assembly in a document. Drawn from the part id space (never equal to a part id).
pub type InstanceId = u64;

/// The first eight bytes of a standalone assembly file ([`AssemblyDef::to_bytes`]).
pub const ASSEMBLY_MAGIC: &[u8; 8] = b"BOWSHASM";

/// The version [`AssemblyDef::to_bytes`] writes and [`AssemblyDef::from_bytes`] reads. The layout rules of [`crate::format`]
/// apply (a new field with a serde default needs no bump).
pub const ASSEMBLY_FORMAT_VERSION: u32 = 1;

/// The permanent identity of an assembly: 128 random bits as 32 lower-case hex digits. Every copy of the definition (in a
/// document, in the library, in another document) carries it, so versions of one assembly can be told from different ones.
#[derive(Clone, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(try_from = "String", into = "String")]
pub struct AssemblyId(String);

impl AssemblyId {
    /// A new identity from the operating system's random source (`getrandom`). If the system gives none (it never does in
    /// practice) the bytes come from the clock, a counter and the std hasher's per-process random keys.
    pub fn random() -> AssemblyId {
        let mut bytes = [0u8; 16];
        if getrandom::fill(&mut bytes).is_err() {
            bytes = fallback_bytes();
        }
        AssemblyId(bytes.iter().map(|b| format!("{b:02x}")).collect())
    }

    /// Reads an identity written by [`as_str`](Self::as_str): 32 hex digits (any case).
    pub fn parse(text: &str) -> Option<AssemblyId> {
        (text.len() == 32 && text.bytes().all(|b| b.is_ascii_hexdigit())).then(|| AssemblyId(text.to_ascii_lowercase()))
    }

    /// The 32 hex digits.
    pub fn as_str(&self) -> &str {
        &self.0
    }
}

impl fmt::Display for AssemblyId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0)
    }
}

impl TryFrom<String> for AssemblyId {
    type Error = String;

    fn try_from(text: String) -> Result<Self, Self::Error> {
        AssemblyId::parse(&text).ok_or_else(|| format!("'{text}' is not an assembly identity (32 hex digits)"))
    }
}

impl From<AssemblyId> for String {
    fn from(id: AssemblyId) -> String {
        id.0
    }
}

fn fallback_bytes() -> [u8; 16] {
    use std::collections::hash_map::RandomState;
    use std::hash::{BuildHasher, Hasher};
    use std::sync::atomic::{AtomicU64, Ordering};
    static COUNTER: AtomicU64 = AtomicU64::new(0);
    let nanos = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map_or(0, |d| d.as_nanos());
    let count = COUNTER.fetch_add(1, Ordering::Relaxed);
    let mut out = [0u8; 16];
    for half in out.chunks_mut(8) {
        let mut hasher = RandomState::new().build_hasher();
        hasher.write_u128(nanos);
        hasher.write_u64(count);
        hasher.write_u32(std::process::id());
        half.copy_from_slice(&hasher.finish().to_le_bytes());
    }
    out
}

/// A reusable definition: identity, version, name and the fragment that is its content.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct AssemblyDef {
    /// Permanent identity, shared by every copy and every version.
    pub identity: AssemblyId,
    /// Bumped on every committed change of the definition; starts at 1.
    pub version: u32,
    /// The name shown on the shelf.
    pub name: String,
    /// What the assembly is made of.
    pub fragment: Fragment,
}

impl AssemblyDef {
    /// Which kind of assembly this is (see [`FragmentKind`]).
    pub fn kind(&self) -> FragmentKind {
        self.fragment.kind()
    }

    /// The definition as a standalone file: the magic `BOWSHASM`, a `u16` version, then the definition as deflated CBOR (the
    /// container of [`crate::format`]); one file per assembly in the library.
    pub fn to_bytes(&self) -> Vec<u8> {
        pack_as(ASSEMBLY_MAGIC, ASSEMBLY_FORMAT_VERSION, self)
    }

    /// Loads a definition written by [`to_bytes`](Self::to_bytes) and checks it ([`Fragment::check`]).
    ///
    /// # Errors
    /// [`FormatError::NotBowshock`] for anything without the assembly magic, [`FormatError::UnsupportedVersion`],
    /// [`FormatError::Corrupt`] for a damaged file and [`FormatError::Invalid`] for an inconsistent definition.
    pub fn from_bytes(bytes: &[u8]) -> Result<AssemblyDef, FormatError> {
        let cbor = unpack_as(bytes, ASSEMBLY_MAGIC, ASSEMBLY_FORMAT_VERSION)?;
        let def: AssemblyDef = ciborium::from_reader(cbor.as_slice()).map_err(|e| FormatError::Corrupt(e.to_string()))?;
        def.fragment.check().map_err(|e| FormatError::Invalid(e.to_string()))?;
        Ok(def)
    }
}

/// One use of a definition in a document.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct AssemblyInstance {
    /// This use's id.
    pub id: InstanceId,
    /// The definition it is a use of.
    pub assembly: AssemblyId,
    /// The version of the definition the members were materialised from.
    pub version: u32,
    /// Each definition-local id of the fragment and the real id of the member in the rocket.
    pub members: BTreeMap<PartId, PartId>,
}

/// Why an assembly operation was refused. The document is unchanged when one is returned.
#[derive(Debug, PartialEq)]
pub enum AssemblyError {
    /// Nothing was selected.
    Empty,
    /// No part has this id.
    NoSuchPart(PartId),
    /// No airframe part has this id (a host).
    NoSuchHost(PartId),
    /// The part already belongs to an instance (assemblies do not nest, and a part is in one instance only).
    AlreadyMember {
        /// The part.
        part: PartId,
        /// The instance it belongs to.
        instance: InstanceId,
    },
    /// The chosen airframe parts are not one unbroken run: `gap` lies between them and was not chosen.
    NotContiguous {
        /// An airframe part between the chosen ones that is not chosen.
        gap: PartId,
    },
    /// The chosen airframe parts are not in one stack (the core's, or the same pod's).
    AcrossStacks,
    /// A radial or internal part was chosen with airframe parts, but the part it is on or in was not.
    NotCarried(PartId),
    /// Radial parts and internal parts were chosen with no airframe part to hold them: an assembly is a stack, internal parts
    /// of one tube, or radial parts of one body.
    MixedClasses,
    /// The chosen internal (or radial) parts are on or in different host parts and no airframe part was chosen: choose the host
    /// tubes too, or one host's parts.
    DifferentHosts,
    /// No assembly has this identity.
    NoSuchAssembly(AssemblyId),
    /// No instance has this id.
    NoSuchInstance(InstanceId),
    /// An assembly with this identity is already in the document.
    Duplicate(AssemblyId),
    /// The definition cannot be removed while it has instances.
    InUse {
        /// How many instances use it.
        instances: usize,
    },
    /// The definition is not newer than the stored one.
    NotNewer {
        /// The stored version.
        stored: u32,
        /// The version offered.
        offered: u32,
    },
    /// The new definition is of another kind than the stored one (a stack cannot become an internal assembly in place).
    KindChanged {
        /// The stored kind.
        from: FragmentKind,
        /// The offered kind.
        to: FragmentKind,
    },
    /// The placement is for another kind of assembly.
    WrongTarget {
        /// The kind the assembly is of.
        expected: FragmentKind,
    },
    /// The assembly is longer than the host's length (or the room left on it).
    TooLong {
        /// The length the assembly needs.
        needs: f64,
        /// The length the host has.
        available: f64,
    },
    /// A pod does not nest: it cannot go on a pod's part or into a pod's stack.
    PodInPod,
    /// Moving between the core and a pod (or between pods) is refused, as it is for single parts.
    AcrossPods,
    /// A member of the instance is missing from the rocket (a damaged instance): detach it.
    BrokenInstance(InstanceId),
    /// The stack rules refused the insertion.
    Insert(InsertError),
    /// The radial part could not be placed.
    Radial(RadialError),
    /// The internal part could not be placed.
    Internal(InternalError),
    /// The fragment is not a well-formed definition.
    Invalid(String),
}

impl fmt::Display for AssemblyError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            AssemblyError::Empty => f.write_str("nothing is selected"),
            AssemblyError::NoSuchPart(id) => write!(f, "part {id} is not in the rocket"),
            AssemblyError::NoSuchHost(id) => write!(f, "part {id} is not an airframe part to put the assembly on or in"),
            AssemblyError::AlreadyMember { part, instance } => write!(f, "part {part} already belongs to assembly use {instance}"),
            AssemblyError::NotContiguous { gap } => {
                write!(f, "the airframe parts are not next to each other: part {gap} lies between them and is not selected")
            }
            AssemblyError::AcrossStacks => f.write_str("the airframe parts are not in one stack (the rocket's or one pod's)"),
            AssemblyError::NotCarried(id) => write!(f, "part {id} is on or in an airframe part that is not selected"),
            AssemblyError::MixedClasses => f.write_str("radial and internal parts need an airframe part selected with them, or must be grouped separately"),
            AssemblyError::DifferentHosts => f.write_str("the parts are on or in different host parts and no airframe part is selected"),
            AssemblyError::NoSuchAssembly(id) => write!(f, "no assembly {id} in this document"),
            AssemblyError::NoSuchInstance(id) => write!(f, "no assembly use {id}"),
            AssemblyError::Duplicate(id) => write!(f, "assembly {id} is already in this document"),
            AssemblyError::InUse { instances } => write!(f, "the assembly is used {instances} time(s) in this rocket"),
            AssemblyError::NotNewer { stored, offered } => write!(f, "version {offered} is not newer than the stored version {stored}"),
            AssemblyError::KindChanged { from, to } => write!(f, "the assembly cannot change from {from:?} to {to:?} in place"),
            AssemblyError::WrongTarget { expected } => write!(f, "this assembly is of kind {expected:?} and goes elsewhere"),
            AssemblyError::TooLong { needs, available } => write!(f, "the assembly needs {needs:.4} m and the host has {available:.4} m"),
            AssemblyError::PodInPod => f.write_str("a pod cannot go on or in another pod"),
            AssemblyError::AcrossPods => f.write_str("an assembly cannot move between the rocket and a pod, or between pods"),
            AssemblyError::BrokenInstance(id) => write!(f, "assembly use {id} has lost a member"),
            AssemblyError::Insert(e) => e.fmt(f),
            AssemblyError::Radial(e) => e.fmt(f),
            AssemblyError::Internal(e) => e.fmt(f),
            AssemblyError::Invalid(why) => write!(f, "not a well-formed assembly: {why}"),
        }
    }
}

impl std::error::Error for AssemblyError {}
