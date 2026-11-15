//! Pods: radial bodies standing parallel to the core, like the boosters, pods and strap-ons of a multi-body rocket (OpenRocket's
//! *pod set*, Niskanen 2013 section 2.1; KSP's radially attached parts).
//!
//! A pod set is a [`RadialPart`] of kind [`RadialKind::Pod`]: it has the symmetry group of every radial part (count, start
//! angle, per-copy angles), sits on a core part at an offset along it and stands `gap` clear of the parent's surface. Its own
//! parts live in its [`PodBody`] — an axial stack plus the radial (fins) and internal parts on and in it — in the rocket's one
//! id space. Each copy is the same body rotated about the core axis: a copy at angle `a` has its axis `axis_radius` from the core
//! axis in direction `a` (0 is +Y, increasing towards +X, as for every radial part), and the body's own angle 0 is the direction
//! away from the core.
//!
//! Everything that edits or reads a part in a pod goes through the same [`Rocket`] methods as a part of the core: the rocket
//! finds the pod holding the id and runs the call on the body as a small rocket of its own ([`Rocket::pod_rocket`]), so a pod's
//! nose, tubes, transitions, fins and internals follow exactly the rules of the core's. A body's stations start at its own fore
//! face; the pod's [`station`](PodFrame::station) puts them on the rocket's.
//!
//! Limits of this version: pods do not nest (a pod stands on a part of the core), the copies of a pod cannot be detached from the
//! group (their parts would need new ids), and a motor mount inside a pod holds no motor (configurations and stages see the core).
//! Mass, CG and inertias include every copy ([`Rocket::component_mass`]), taking each copy's mass about the core axis by the
//! parallel-axis theorem; a pod part's mass and CG overrides are per copy.

use crate::config::Configuration;
use crate::geometry::{NoseShape, Part, PartKind, PodBody, RadialKind, RadialPart};
use crate::internal::{InternalKind, InternalPart};
use crate::rocket::{InsertError, Rocket};
use crate::PartId;

/// Where a pod stands in the rocket: the numbers a mesh or a flight model needs to place its copies.
#[derive(Clone, Debug, PartialEq)]
pub struct PodFrame {
    /// Station of the pod's fore face from the rocket's nose tip, metres.
    pub station: f64,
    /// Distance from the core axis to the pod's axis, metres: the parent's surface under the pod, plus the gap, plus the pod's
    /// body radius.
    pub axis_radius: f64,
    /// The angle of each copy around the core, radians (see [`RadialPart::angles`]).
    pub angles: Vec<f64>,
}

impl PodBody {
    /// A pod body for a core whose booster tube is `parent_diameter` wide and `parent_length` long: a tangent-ogive nose and a
    /// body tube, half the diameter and 0.6 of the length of the parent, with placeholder ids that the rocket replaces when the
    /// pod is placed ([`Rocket::attach_radial`]).
    pub fn standard(parent_diameter: f64, parent_length: f64) -> PodBody {
        let diameter = (0.5 * parent_diameter).max(0.01);
        let nose = Part::new(0, PartKind::NoseCone { shape: NoseShape::TangentOgive, length: 3.0 * diameter, diameter });
        let tube = Part::new(1, PartKind::BodyTube { length: (0.6 * parent_length).max(4.0 * diameter), diameter });
        PodBody { parts: vec![nose, tube], radials: Vec::new(), internals: Vec::new() }
    }

    /// Gives every part a new id from `next` (and points radial and internal parts at their parents' new ids). Returns the next
    /// unused id.
    pub(crate) fn reid(&mut self, mut next: PartId) -> PartId {
        let mut map: Vec<(PartId, PartId)> = Vec::new();
        let mut fresh = |old: PartId, next: &mut PartId| {
            let id = *next;
            *next += 1;
            map.push((old, id));
            id
        };
        for p in &mut self.parts {
            p.id = fresh(p.id, &mut next);
        }
        let lookup = |map: &[(PartId, PartId)], old: PartId| map.iter().find(|m| m.0 == old).map_or(old, |m| m.1);
        for r in &mut self.radials {
            r.id = fresh(r.id, &mut next);
        }
        for i in &mut self.internals {
            i.id = fresh(i.id, &mut next);
        }
        for r in &mut self.radials {
            r.parent = lookup(&map, r.parent);
        }
        for i in &mut self.internals {
            i.parent = lookup(&map, i.parent);
        }
        next
    }

    /// The body as a rocket of its own, ids kept (see [`Rocket::pod_rocket`] for a pod of a document). Its next id is past the
    /// body's.
    pub fn to_rocket(&self) -> Rocket {
        Rocket::from_body(self.clone(), self.ids().max().map_or(0, |m| m + 1))
    }
}

impl Rocket {
    /// A small rocket made of a pod body, ids kept, for running the ordinary rules on it.
    pub(crate) fn from_body(body: PodBody, next_id: PartId) -> Rocket {
        Rocket {
            name: String::new(),
            designer: String::new(),
            parts: body.parts,
            radials: body.radials,
            internals: body.internals,
            next_id,
            configs: vec![Configuration::new(0, "")],
            active_config: 0,
            next_config_id: 1,
            notes: std::collections::BTreeMap::new(),
            flights: Vec::new(),
            formulas: Vec::new(),
            bands: crate::bands::StabilityBands::default(),
            plots: crate::flights::PlotLayout::default(),
        }
    }

    fn into_body(self) -> PodBody {
        PodBody { parts: self.parts, radials: self.radials, internals: self.internals }
    }

    /// Every pod body, with its set.
    fn bodies(&self) -> impl Iterator<Item = &PodBody> {
        self.radials.iter().filter_map(|r| r.kind.pod_body())
    }

    pub(crate) fn pod_parts(&self) -> impl Iterator<Item = &Part> {
        self.bodies().flat_map(|b| b.parts.iter())
    }

    pub(crate) fn pod_radials(&self) -> impl Iterator<Item = &RadialPart> {
        self.bodies().flat_map(|b| b.radials.iter())
    }

    pub(crate) fn pod_internals(&self) -> impl Iterator<Item = &InternalPart> {
        self.bodies().flat_map(|b| b.internals.iter())
    }

    /// Index in `self.radials` of the pod whose body holds `id` (the pod's own id is not in its body).
    pub(crate) fn pod_index(&self, id: PartId) -> Option<usize> {
        self.radials.iter().position(|r| r.kind.pod_body().is_some_and(|b| b.contains(id)))
    }

    /// The pod set that holds part `id` in its body; `None` for parts of the core and for unknown ids.
    pub fn pod_of(&self, id: PartId) -> Option<&RadialPart> {
        self.pod_index(id).map(|i| &self.radials[i])
    }

    /// The pod sets, in the order they were placed.
    pub fn pods(&self) -> impl Iterator<Item = &RadialPart> {
        self.radials.iter().filter(|r| r.kind.is_pod())
    }

    /// A copy of the pod `pod` as a rocket of its own: its parts, radial parts and internal parts with their ids, stations from
    /// the pod's fore face. Reading it is the way to apply the usual rocket queries (stations, [`Rocket::part_span`],
    /// [`Rocket::component_mass`], ...) to a pod; edit through the outer rocket instead, which keeps ids and offsets consistent.
    /// `None` if `pod` is not a pod set.
    pub fn pod_rocket(&self, pod: PartId) -> Option<Rocket> {
        let body = self.radial(pod)?.kind.pod_body()?;
        Some(Rocket::from_body(body.clone(), self.next_id))
    }

    /// The pod holding `id` as a rocket of its own, with the pod's station.
    pub(crate) fn scratch_of(&self, id: PartId) -> Option<(f64, Rocket)> {
        let pod = &self.radials[self.pod_index(id)?];
        let station = self.radial_station(pod)?;
        Some((station, Rocket::from_body(pod.kind.pod_body()?.clone(), self.next_id)))
    }

    /// Runs `f` on the pod holding `id`, as a rocket of its own, and puts the result back. The pod is clamped to its parent
    /// afterwards (its length may have changed). `None` if `id` is not in a pod.
    pub(crate) fn in_pod<R>(&mut self, id: PartId, f: impl FnOnce(&mut Rocket) -> R) -> Option<R> {
        let i = self.pod_index(id)?;
        Some(self.run_on_pod(i, f))
    }

    /// Like [`in_pod`](Self::in_pod) but names the pod set itself, which may have no parts yet.
    pub(crate) fn with_pod_set<R>(&mut self, pod: PartId, f: impl FnOnce(&mut Rocket) -> R) -> Option<R> {
        let i = self.radials.iter().position(|r| r.id == pod && r.kind.is_pod())?;
        Some(self.run_on_pod(i, f))
    }

    fn run_on_pod<R>(&mut self, i: usize, f: impl FnOnce(&mut Rocket) -> R) -> R {
        let body = match &mut self.radials[i].kind {
            RadialKind::Pod { body, .. } => std::mem::take(body),
            _ => PodBody::default(),
        };
        let mut sub = Rocket::from_body(body, self.next_id);
        let out = f(&mut sub);
        self.next_id = self.next_id.max(sub.next_id);
        if let RadialKind::Pod { body, .. } = &mut self.radials[i].kind {
            *body = sub.into_body();
        }
        let parent = self.radials[i].parent;
        self.clamp_radials(parent);
        self.prune_configs();
        out
    }

    /// Where pod `pod` stands: its station, the distance of its axis from the core's and the angle of each copy. `None` if it is
    /// not a pod set of the rocket.
    pub fn pod_frame(&self, pod: &RadialPart) -> Option<PodFrame> {
        let RadialKind::Pod { gap, body } = &pod.kind else {
            return None;
        };
        let axis_radius = self.radial_base_radius(pod)? + gap + body.radius();
        Some(PodFrame { station: self.radial_station(pod)?, axis_radius, angles: pod.angles() })
    }

    /// The pod kind a new pod of the drag-in gets: the standard body for the rocket's aft-most body tube, the booster a strap-on
    /// stands against (see [`PodBody::standard`]), against the surface.
    pub fn default_pod(&self) -> RadialKind {
        let tube = self.parts.iter().rev().find(|p| matches!(p.kind, PartKind::BodyTube { .. })).map(|p| (p.kind.aft_diameter(), p.kind.length()));
        let (diameter, length) = tube.unwrap_or((self.reference_diameter().unwrap_or(0.0762), self.length().max(0.3)));
        RadialKind::Pod { gap: 0.0, body: PodBody::standard(diameter, length) }
    }

    /// The pod holding axial part `id` and the part's index in the pod's stack. `None` for parts of the core.
    pub fn pod_part_index(&self, id: PartId) -> Option<(PartId, usize)> {
        let pod = self.pod_of(id)?;
        let index = pod.kind.pod_body()?.parts.iter().position(|p| p.id == id)?;
        Some((pod.id, index))
    }

    /// Attach nodes of pod `pod` where `kind` may be inserted, each with its station from the rocket's nose tip (see
    /// [`Rocket::nodes_for`]).
    pub fn nodes_for_pod(&self, pod: PartId, kind: &PartKind) -> Vec<(usize, f64)> {
        let (Some(set), Some(sub)) = (self.radial(pod), self.pod_rocket(pod)) else {
            return Vec::new();
        };
        let z0 = self.radial_station(set).unwrap_or(0.0);
        sub.nodes_for(kind).into_iter().map(|(i, z)| (i, z + z0)).collect()
    }

    /// `kind` as it would be inserted at `index` of pod `pod`'s stack, diameters matched to its neighbours.
    pub fn matched_in_pod(&self, pod: PartId, index: usize, kind: &PartKind) -> PartKind {
        self.pod_rocket(pod).map_or_else(|| kind.clone(), |sub| sub.matched(index, kind))
    }

    /// Inserts `kind` (matched to its neighbours) at `index` of pod `pod`'s stack and returns the new part's id.
    pub fn insert_in_pod(&mut self, pod: PartId, index: usize, kind: PartKind) -> Result<PartId, InsertError> {
        self.with_pod_set(pod, |sub| sub.insert(index, kind)).ok_or(InsertError::IndexOutOfRange)?
    }

    /// Puts `part` at `index` of pod `pod`'s stack exactly as it is (see [`Rocket::restore`]).
    pub fn restore_in_pod(&mut self, pod: PartId, index: usize, part: Part) -> Result<(), InsertError> {
        self.with_pod_set(pod, |sub| sub.restore(index, part)).ok_or(InsertError::IndexOutOfRange)?
    }

    /// Checks the pods of a loaded document: each stands on a part of the core, holds a well-formed stack and parents that exist
    /// within it. Ids are checked with the rest of the document.
    pub(crate) fn check_pods(&self) -> Result<(), String> {
        for pod in self.pods() {
            if self.index_of(pod.parent).is_none() {
                return Err(format!("pod {} is on {}, which is not a part of the core", pod.id, pod.parent));
            }
            let Some(body) = pod.kind.pod_body() else { continue };
            if body.radials.iter().any(|r| r.kind.is_pod()) {
                return Err(format!("pod {} holds another pod", pod.id));
            }
            if body.parts.iter().enumerate().any(|(i, p)| matches!(p.kind, PartKind::NoseCone { .. }) && i != 0) {
                return Err(format!("pod {} has a nose cone that is not at its fore end", pod.id));
            }
            if body.parts.iter().any(|p| !p.kind.length().is_finite() || p.kind.length() < 0.0 || !p.kind.aft_diameter().is_finite()) {
                return Err(format!("pod {} has a negative or non-finite size", pod.id));
            }
            for r in &body.radials {
                if !body.parts.iter().any(|p| p.id == r.parent) || r.count == 0 || r.count > crate::MAX_COUNT {
                    return Err(format!("radial part {} in pod {} is on a part that is not in the pod", r.id, pod.id));
                }
            }
            for i in &body.internals {
                if !body.parts.iter().any(|p| p.id == i.parent) {
                    return Err(format!("internal part {} in pod {} is in a part that is not in the pod", i.id, pod.id));
                }
            }
        }
        Ok(())
    }

    /// Whether `id` is a pod set.
    pub fn is_pod(&self, id: PartId) -> bool {
        self.radial(id).is_some_and(|r| r.kind.is_pod())
    }

    /// The motor-mount flag of an internal tube inside a pod does nothing in this version; this lists those mounts so a UI can
    /// say so.
    pub fn pod_motor_mounts(&self) -> Vec<PartId> {
        self.pod_internals().filter(|p| matches!(p.kind, InternalKind::InnerTube { motor_mount: true, .. })).map(|p| p.id).collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::{FinSection, RadialKind};
    use crate::handles::Dim;
    use crate::rocket::RadialError;
    use std::f64::consts::PI;

    fn core() -> (Rocket, PartId, PartId) {
        let mut r = Rocket::default();
        r.insert(0, PartKind::NoseCone { shape: NoseShape::TangentOgive, length: 0.3, diameter: 0.1 }).unwrap();
        let tube = r.insert(1, PartKind::BodyTube { length: 1.0, diameter: 0.1 }).unwrap();
        (r, tube, 0)
    }

    fn with_pods(count: u8) -> (Rocket, PartId, PartId) {
        let (mut r, tube, _) = core();
        let kind = RadialKind::Pod { gap: 0.0, body: PodBody::standard(0.1, 1.0) };
        let pod = r.attach_radial(tube, 0.3, 0.0, count, kind).unwrap();
        (r, tube, pod)
    }

    #[test]
    fn a_pod_gets_fresh_ids_and_stands_against_the_surface() {
        let (r, tube, pod) = with_pods(2);
        let ids = r.ids();
        let mut sorted = ids.clone();
        sorted.sort_unstable();
        sorted.dedup();
        assert_eq!(sorted.len(), ids.len(), "unique ids");
        // The pod's stack: nose then tube, half the booster's diameter, 0.6 of its length.
        let body = r.radial(pod).unwrap().kind.pod_body().unwrap();
        assert_eq!(body.parts().len(), 2);
        assert!((body.radius() - 0.025).abs() < 1e-12);
        assert!((body.length() - (0.15 + 0.6)).abs() < 1e-12);
        // Touching: its axis is the surface radius plus its own radius out; the station is the parent's plus the offset.
        let frame = r.pod_frame(r.radial(pod).unwrap()).unwrap();
        assert!((frame.axis_radius - 0.075).abs() < 1e-12, "{}", frame.axis_radius);
        assert!((frame.station - 0.6).abs() < 1e-12);
        assert_eq!(frame.angles.len(), 2);
        assert!((frame.angles[1] - PI).abs() < 1e-12);
        // The core is untouched and pods do not count in its length.
        assert!((r.length() - 1.3).abs() < 1e-12 && r.parts().len() == 2);
        assert_eq!(r.pod_of(body.parts()[0].id).map(|p| p.id), Some(pod));
        assert_eq!(r.parent_of(pod), Some(tube));
    }

    #[test]
    fn parts_in_a_pod_are_edited_like_parts_of_the_core() {
        let (mut r, _, pod) = with_pods(2);
        let body = r.radial(pod).unwrap().kind.pod_body().unwrap().clone();
        let (nose, tube) = (body.parts()[0].id, body.parts()[1].id);
        // A dimension, its handles and its station.
        assert_eq!(r.dim(tube, Dim::Length), Some(0.6));
        let edit = r.set_dim(tube, Dim::Length, 0.5).unwrap();
        assert_eq!(edit.limited, None);
        assert_eq!(r.dim(tube, Dim::Length), Some(0.5));
        assert!(r.handle_specs(tube).iter().any(|h| h.dim == Dim::Diameter));
        assert!((r.part_station(tube).unwrap() - (0.6 + 0.15)).abs() < 1e-12);
        // The nose base follows the tube's diameter, as in the core.
        r.set_dim(tube, Dim::Diameter, 0.04).unwrap();
        assert_eq!(r.dim(nose, Dim::Diameter), Some(0.04));
        // Fins on the pod's tube, with the pod's surface under them.
        let fins =
            r.attach_radial(tube, 0.4, 0.0, 3, RadialKind::FinSet { root_chord: 0.08, tip_chord: 0.03, span: 0.04, sweep: 0.03, thickness: 0.002 }).unwrap();
        assert_eq!(r.parent_of(fins), Some(tube));
        assert!((r.part_station(fins).unwrap() - (0.6 + 0.15 + 0.4)).abs() < 1e-12);
        assert!(r.set_fin_section(fins, FinSection::Airfoil) && r.set_cant(fins, 0.01) && r.set_name(fins, Some("Pod fins".into())));
        assert_eq!(r.radial(fins).unwrap().display_name(), "Pod fins");
        assert_eq!(r.dim(fins, Dim::Span), Some(0.04));
        // Id spaces stay shared: the next part of the core does not reuse an id of the pod.
        let extra = r.insert(2, PartKind::BodyTube { length: 0.2, diameter: 0.1 }).unwrap();
        assert!(!r.pod_of(fins).is_none() && r.pod_of(extra).is_none());
        let mut ids = r.ids();
        ids.sort_unstable();
        ids.dedup();
        assert_eq!(ids.len(), r.ids().len());
        // An internal part inside the pod's tube.
        let ring = r.attach_internal(tube, 0.1, InternalKind::Bulkhead { diameter: None, thickness: 0.003 }).unwrap();
        assert!(r.resolved(ring).is_some_and(|s| (s.outer_diameter - (0.04 - 2.0 * r.part(tube).unwrap().thickness)).abs() < 1e-9));
        // Removing the pod's last stack parts takes the pod with it.
        r.remove(tube);
        r.remove(nose);
        assert!(r.radial(pod).is_none() && r.radial(fins).is_none() && r.internal(ring).is_none());
    }

    #[test]
    fn a_pod_adds_its_mass_about_the_core_axis() {
        let (core_only, _, _) = core();
        let (r, _, pod) = with_pods(2);
        let body = r.radial(pod).unwrap().kind.pod_body().unwrap();
        let (nose, tube) = (body.parts()[0].id, body.parts()[1].id);
        let (m_nose, m_tube) = (r.component_mass(nose).unwrap(), r.component_mass(tube).unwrap());
        // One copy of the tube on its own: m = rho pi/4 (D^2 - d^2) L, and two copies weigh twice that.
        let t = r.part(tube).unwrap();
        let (d_out, d_in) = (0.05_f64, 0.05 - 2.0 * t.thickness);
        let one = t.material.density() * PI / 4.0 * (d_out * d_out - d_in * d_in) * 0.6;
        assert!((m_tube.mass - 2.0 * one).abs() / one < 1e-3, "{} vs {}", m_tube.mass, 2.0 * one);
        // Its CG is the middle of the tube, in rocket stations: pod fore + nose + half the tube.
        assert!((m_tube.cg - (0.6 + 0.15 + 0.3)).abs() < 1e-9);
        // About the long axis the two tubes add m r^2 with r the axis distance (0.075) on top of their own.
        assert!(m_tube.i_rot > 2.0 * one * 0.075 * 0.075);
        let dry = r.dry_mass();
        let core_mass = core_only.dry_mass();
        assert!((dry.mass - (core_mass.mass + m_nose.mass + m_tube.mass)).abs() < 1e-9, "the set is its parts, once");
        // The set reports the same total, and the CG moves aft of the core's.
        let total = r.component_mass(pod).unwrap();
        assert!((total.mass - (m_nose.mass + m_tube.mass)).abs() < 1e-9);
        assert!(dry.cg > core_mass.cg);
        assert_eq!(r.component_masses().iter().filter(|(id, _)| *id == pod).count(), 0);
    }

    #[test]
    fn the_gap_moves_the_axis_out_and_the_set_moves_with_its_parent() {
        let (mut r, tube, pod) = with_pods(3);
        assert!(r.set_dim(pod, Dim::Gap, 0.01).is_some());
        assert!((r.pod_frame(r.radial(pod).unwrap()).unwrap().axis_radius - 0.085).abs() < 1e-12);
        assert_eq!(r.set_dim(pod, Dim::Gap, -1.0).unwrap().limited, Some("TOUCHING THE SURFACE"));
        assert_eq!(r.dim(pod, Dim::Gap), Some(0.0));
        // Shortening the parent limits the pod's offset to what keeps half of it on the tube.
        assert!(r.set_dim(tube, Dim::Length, 0.4).is_some());
        let offset = r.radial(pod).unwrap().offset;
        assert!((offset - (0.4 - 0.75 / 2.0)).abs() < 1e-12, "{offset}");
        // A pod does not stand on a part of a pod.
        let body = r.radial(pod).unwrap().kind.pod_body().unwrap().clone();
        let inner_tube = body.parts()[1].id;
        assert_eq!(r.attach_radial(inner_tube, 0.0, 0.0, 2, r.radial(pod).unwrap().kind.clone()), Err(RadialError::NoSuchParent));
        // Its copies cannot leave the group.
        assert!(r.detach_radial(pod, 0).is_err() && r.split_all_radial(pod).is_err());
        // Removing the parent takes the pod and everything in it.
        let ids_before = r.ids().len();
        r.remove(tube);
        assert!(r.radial(pod).is_none() && r.ids().len() < ids_before && r.pods().count() == 0);
    }

    #[test]
    fn a_pod_is_carried_out_and_put_back_whole() {
        let (mut r, tube, pod) = with_pods(2);
        let body = r.radial(pod).unwrap().kind.pod_body().unwrap().clone();
        let pod_tube = body.parts()[1].id;
        let fins = r
            .attach_radial(pod_tube, 0.4, 0.0, 3, RadialKind::FinSet { root_chord: 0.08, tip_chord: 0.03, span: 0.04, sweep: 0.03, thickness: 0.002 })
            .unwrap();
        let before = r.clone();
        let taken = r.remove_radial(pod).unwrap();
        assert!(r.part(pod_tube).is_none() && r.radial(fins).is_none(), "the body leaves with the set");
        let moved = RadialPart { parent: tube, offset: 0.1, angle: 1.0, ..taken };
        r.restore_radial(moved).unwrap();
        assert!(r.radial(fins).is_some() && r.part(pod_tube).is_some());
        assert_eq!(r.radial(pod).unwrap().offset, 0.1);
        assert_eq!(r.ids().len(), before.ids().len());
    }

    #[test]
    fn pods_survive_json_and_the_pod_rocket_matches() {
        let (mut r, _, pod) = with_pods(2);
        let tube = r.radial(pod).unwrap().kind.pod_body().unwrap().parts()[1].id;
        r.attach_radial(tube, 0.3, 0.5, 2, RadialKind::LaunchLug { length: 0.05, diameter: 0.01 }).unwrap();
        r.attach_internal(tube, 0.1, InternalKind::Bulkhead { diameter: None, thickness: 0.003 }).unwrap();
        let back = Rocket::from_json(&r.to_json()).unwrap();
        assert_eq!(back, r);
        assert_eq!(back.dry_mass(), r.dry_mass());
        // The pod as a rocket of its own: stations from its fore face.
        let sub = r.pod_rocket(pod).unwrap();
        assert_eq!(sub.parts().len(), 2);
        assert!((sub.stations()[1] - 0.15).abs() < 1e-12);
        assert_eq!(sub.radials().len(), 1);
        // A hand-edited document that nests a pod, or hangs one off a pod's part, is refused.
        let mut bad = r.clone();
        let inner = bad.radials[0].kind.pod_body().unwrap().parts[0].id;
        bad.radials[0].parent = inner;
        assert!(Rocket::from_json(&bad.to_json()).is_err());
    }

    #[test]
    fn nodes_and_insertions_work_on_a_pods_stack() {
        let (mut r, _, pod) = with_pods(2);
        let tube = PartKind::BodyTube { length: 0.1, diameter: 0.05 };
        // Nose is first, so a tube may go at nodes 1 and 2 only; stations are the rocket's.
        let nodes = r.nodes_for_pod(pod, &tube);
        assert_eq!(nodes.iter().map(|n| n.0).collect::<Vec<_>>(), vec![1, 2]);
        assert!((nodes[0].1 - (0.6 + 0.15)).abs() < 1e-12 && (nodes[1].1 - (0.6 + 0.75)).abs() < 1e-12);
        let id = r.insert_in_pod(pod, 2, tube).unwrap();
        assert_eq!(r.pod_part_index(id), Some((pod, 2)));
        assert!((r.part_station(id).unwrap() - (0.6 + 0.75)).abs() < 1e-12);
        // A second nose is refused, as in the core.
        assert_eq!(r.insert_in_pod(pod, 0, PartKind::NoseCone { shape: NoseShape::Conical, length: 0.1, diameter: 0.05 }), Err(InsertError::SecondNose));
    }

    #[test]
    fn a_pods_motor_mount_is_a_mount_with_copies_but_not_a_stage() {
        let (mut r, _, pod) = with_pods(2);
        let tube = r.radial(pod).unwrap().kind.pod_body().unwrap().parts()[1].id;
        let mount = r
            .attach_internal(tube, 0.1, InternalKind::InnerTube { outer_diameter: 0.0229, thickness: 0.0005, length: 0.2, motor_mount: true, overhang: 0.0 })
            .unwrap();
        let m = r.motor_mounts().into_iter().find(|m| m.id == mount).unwrap();
        assert_eq!((m.pod, m.copies), (Some(pod), 2));
        assert!((m.fore_station - (0.6 + 0.15 + 0.1)).abs() < 1e-12);
        // It takes a motor in the active configuration and keeps it through edits of the pod; the stages see only the core.
        let config = r.active_config_id();
        r.set_motor(config, mount, Some(crate::MotorRef::new("G80T", "AeroTech"))).unwrap();
        r.set_name(tube, Some("Booster".into()));
        assert!(r.active_configuration().slot(mount).is_some_and(|s| s.motor.is_some()));
        assert!(r.stages().iter().all(|s| !s.mounts.contains(&mount)));
        assert_eq!(r.stage_of(mount), Some(1));
        // Removing the mount drops its slot.
        r.remove_internal(mount);
        assert!(r.active_configuration().slot(mount).is_none());
    }
}
