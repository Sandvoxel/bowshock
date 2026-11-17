//! An assembly as a document: [`AssemblyDef::to_rocket`] opens a definition in the ordinary Build screen, and
//! [`AssemblyDef::from_rocket`] turns the edited document back into a definition.

use std::collections::BTreeSet;

use super::fragment::classify;
use super::{AssemblyDef, AssemblyError, AssemblyId, Fragment, FragmentKind, Host};
use crate::geometry::{Part, PartKind};
use crate::rocket::Rocket;
use crate::PartId;

impl AssemblyDef {
    /// The definition as a standalone document containing just the fragment, ids as in the definition (new parts get ids that were
    /// never used by it). A stack is the document's stack. An internal or surface assembly needs a host: the document gets a stand-in
    /// body tube of the recorded [`Host`] (outer diameter, wall, length) with the members in it or on it at the positions they were
    /// made at, flagged ([`Rocket::stand_in`]) so [`from_rocket`](Self::from_rocket) strips it again. The document has no motors and
    /// no flights of its own to keep: only the fragment is read back.
    pub fn to_rocket(&self) -> Rocket {
        let frag = &self.fragment;
        let mut rocket = Rocket {
            name: self.name.clone(),
            parts: frag.parts.clone(),
            radials: frag.radials.clone(),
            internals: frag.internals.clone(),
            formulas: frag.formulas.clone(),
            next_id: frag.next_id,
            ..Rocket::default()
        };
        if let Some(host) = frag.host {
            let id = frag.next_id;
            let mut tube = Part::new(id, PartKind::BodyTube { length: host.length, diameter: host.outer_diameter });
            tube.thickness = host.thickness;
            tube.name = Some("Host (stand-in)".to_string());
            rocket.parts.push(tube);
            rocket.radials.iter_mut().filter(|r| r.parent == Fragment::HOST).for_each(|r| r.parent = id);
            rocket.internals.iter_mut().filter(|i| i.parent == Fragment::HOST).for_each(|i| i.parent = id);
            rocket.next_id = id + 1;
            rocket.stand_in = Some(id);
        }
        rocket
    }

    /// The definition an edited document holds: its parts, radial parts and internal parts with their ids as they are (a part the
    /// editor added has a new id), minus the stand-in host, which becomes the recorded [`Host`] of an internal or surface assembly.
    /// Formulas whose target and references are all members are kept; the rest are dropped (the fields keep their values). Motor
    /// assignments are not read. `identity`, `version` and `name` are stored as given (the caller bumps the version for a changed
    /// definition). `from_rocket(.., &def.to_rocket())` is `def` again.
    ///
    /// # Errors
    /// [`AssemblyError::Empty`], [`AssemblyError::MixedClasses`] (radial and internal parts with no airframe part), or
    /// [`AssemblyError::Invalid`] for anything else that is not an assembly (an internal part left in a part that is not there).
    pub fn from_rocket(identity: AssemblyId, version: u32, name: &str, rocket: &Rocket) -> Result<AssemblyDef, AssemblyError> {
        let stand_in = rocket.stand_in.and_then(|id| rocket.parts.iter().find(|p| p.id == id));
        let host_id = stand_in.map(|p| p.id);
        let parts: Vec<Part> = rocket.parts.iter().filter(|p| Some(p.id) != host_id).cloned().collect();
        let mut radials = rocket.radials.clone();
        let mut internals = rocket.internals.clone();
        let to_host = |parent: PartId| if Some(parent) == host_id { Fragment::HOST } else { parent };
        radials.iter_mut().for_each(|r| r.parent = to_host(r.parent));
        internals.iter_mut().for_each(|i| i.parent = to_host(i.parent));
        let kind = classify(&parts, &radials, &internals)?;

        let mut fragment = Fragment { parts, radials, internals, formulas: Vec::new(), next_id: 0, host: None };
        let ids: BTreeSet<PartId> = fragment.ids().into_iter().collect();
        fragment.formulas =
            rocket.formulas.iter().filter(|f| ids.contains(&f.target.part) && f.expr.refs().iter().all(|r| ids.contains(&r.part))).cloned().collect();
        // The stand-in took the next id when the document was made: give it back if nothing else took one since.
        let next = match stand_in {
            Some(p) if p.id + 1 == rocket.next_id => p.id,
            _ => rocket.next_id,
        };
        fragment.next_id = next.max(ids.iter().max().map_or(0, |m| m + 1));
        if kind != FragmentKind::Stack {
            let tube = stand_in.ok_or_else(|| AssemblyError::Invalid("an internal or surface assembly is edited with its stand-in host tube".to_string()))?;
            let mid = fragment.origin() + fragment.extent() / 2.0;
            fragment.host = Some(Host { outer_diameter: 2.0 * tube.kind.radius_at(mid), thickness: tube.thickness, length: tube.kind.length() });
        }
        fragment.check()?;
        Ok(AssemblyDef { identity, version, name: name.to_string(), fragment })
    }
}
