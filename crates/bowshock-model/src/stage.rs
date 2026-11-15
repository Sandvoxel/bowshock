//! Stages, derived (never stored) from the motor mounts: one stage per in-line group of mounts, split at the first joint between
//! airframe parts at or aft of the upper motor's aft end.

use std::fmt;

use crate::config::MotorMount;
use crate::geometry::{Part, PartKind};
use crate::internal::PartRef;
use crate::rocket::Rocket;
use crate::PartId;

/// Stations closer than this (1 µm) are the same station.
const TOL: f64 = 1e-6;

/// Where a stage separates from the stage ahead of it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Separation {
    /// Station of the joint, from the rocket's tip.
    pub station: f64,
    /// The airframe parts either side of the joint, fore then aft.
    pub joint_between: (PartId, PartId),
    /// The motor mount of the upper stage whose aft end the joint was found from.
    pub found_from: PartId,
}

/// A problem with a stage's separation.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum StageIssue {
    /// No joint lies between the upper motor's aft end and the lower stage's mounts (both mounts are in one tube, or the
    /// motor is recessed past the joint). Splitting `tube` at `station` (the upper motor's aft end) fixes it, see
    /// [`Rocket::split_tube`].
    NoJoint {
        /// The airframe part the upper motor's aft end is in.
        tube: PartId,
        /// Station to split at.
        station: f64,
    },
}

/// One stage of the rocket.
#[derive(Clone, Debug, PartialEq)]
pub struct Stage {
    /// Firing order, from 1: the aft-most stage fires first.
    pub index: usize,
    /// `"Stage 1"`, ...
    pub name: String,
    /// Station of the stage's fore end: its separation, or where the separation should be if [`issue`](Self::issue) is set,
    /// or 0 for the top stage.
    pub fore_station: f64,
    /// Station of the stage's aft end: the next stage's fore end, or the rocket's length for stage 1.
    pub aft_station: f64,
    /// The joint this stage separates at from the stage ahead of it; `None` for the top stage (and when `issue` is set).
    pub separation: Option<Separation>,
    /// Why the separation could not be found.
    pub issue: Option<StageIssue>,
    /// The motor mounts of the stage, fore to aft. A cluster has several.
    pub mounts: Vec<PartId>,
}

/// Why a tube could not be split.
#[derive(Debug, PartialEq, Eq)]
pub enum SplitError {
    /// No axial part has this id.
    NoSuchPart,
    /// A nose cone cannot be split.
    NotSplittable,
    /// The station is not inside the part.
    OutOfRange,
}

impl fmt::Display for SplitError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            SplitError::NoSuchPart => "no such airframe part",
            SplitError::NotSplittable => "a nose cone cannot be split",
            SplitError::OutOfRange => "the station is not inside the part",
        })
    }
}

impl std::error::Error for SplitError {}

impl Rocket {
    /// The stages, in firing order (stage 1, the aft-most, first). A rocket with fewer than two in-line motor mounts (a
    /// cluster is one stage) has the single stage 1 and no separation. See the Staging spec: the separation of a stage is the
    /// first joint between airframe parts at or aft of the upper motor's aft end (mount aft end plus overhang) and at or
    /// fore of the stage's own mounts; if there is none the stage carries a [`StageIssue::NoJoint`].
    pub fn stages(&self) -> Vec<Stage> {
        let clusters = self.mount_clusters();
        let length = self.length();
        let count = clusters.len().max(1);
        let stations = self.stations();
        let joints: Vec<(f64, (PartId, PartId))> = (1..self.parts.len()).map(|j| (stations[j], (self.parts[j - 1].id, self.parts[j].id))).collect();

        // For cluster `p` (0 is the fore-most): the separation from the cluster ahead of it.
        let find = |p: usize| -> (Option<Separation>, Option<StageIssue>) {
            if p == 0 {
                return (None, None);
            }
            let (upper, lower) = (&clusters[p - 1], &clusters[p]);
            let source = upper.iter().max_by(|a, b| a.motor_aft_station.total_cmp(&b.motor_aft_station)).copied();
            let (Some(source), Some(lower_fore)) = (source, lower.iter().map(|m| m.fore_station).reduce(f64::min)) else {
                return (None, None);
            };
            let upper_aft = source.motor_aft_station;
            match joints.iter().find(|(s, _)| *s >= upper_aft - TOL && *s <= lower_fore + TOL) {
                Some(&(station, joint_between)) => (Some(Separation { station, joint_between, found_from: source.id }), None),
                None => {
                    let tube = (0..self.parts.len())
                        .find(|&j| upper_aft >= stations[j] - TOL && upper_aft < stations[j + 1] - TOL)
                        .or(self.parts.len().checked_sub(1));
                    (None, tube.map(|j| StageIssue::NoJoint { tube: self.parts[j].id, station: upper_aft }))
                }
            }
        };
        let found: Vec<_> = (0..count).map(find).collect();
        let fore_of = |p: usize| match &found[p] {
            (Some(s), _) => s.station,
            (None, Some(StageIssue::NoJoint { station, .. })) => *station,
            _ => 0.0,
        };

        (0..count)
            .rev()
            .map(|p| {
                let index = count - p;
                Stage {
                    index,
                    name: format!("Stage {index}"),
                    fore_station: fore_of(p),
                    aft_station: if p + 1 < count { fore_of(p + 1) } else { length },
                    separation: found[p].0,
                    issue: found[p].1,
                    mounts: clusters.get(p).map(|c| c.iter().map(|m| m.id).collect()).unwrap_or_default(),
                }
            })
            .collect()
    }

    /// Every stage's issue, in firing order.
    pub fn stage_issues(&self) -> Vec<(usize, StageIssue)> {
        self.stages().into_iter().filter_map(|s| s.issue.map(|i| (s.index, i))).collect()
    }

    /// The motor mounts grouped into clusters (mounts whose tubes overlap along the axis), fore-most cluster first.
    fn mount_clusters(&self) -> Vec<Vec<MotorMount>> {
        let mut clusters: Vec<Vec<MotorMount>> = Vec::new();
        let mut reach = f64::NEG_INFINITY;
        for mount in self.motor_mounts() {
            match clusters.last_mut() {
                Some(last) if mount.fore_station < reach - TOL => {
                    reach = reach.max(mount.aft_station);
                    last.push(mount);
                }
                _ => {
                    reach = mount.aft_station;
                    clusters.push(vec![mount]);
                }
            }
        }
        clusters
    }

    /// The stage (by [`Stage::index`]) part `id` belongs to, by where its middle lies; radial and internal parts (a coupler
    /// at a joint, fins on a tube) stay with the stage of the part they are attached to. `None` for an unknown id.
    pub fn stage_of(&self, id: PartId) -> Option<usize> {
        let anchor = match self.kind_of(id)? {
            PartRef::Axial(p) => p.id,
            PartRef::Radial(r) => r.parent,
            PartRef::Internal(i) => i.parent,
        };
        let (fore, aft) = self.part_span(anchor)?;
        let mid = (fore + aft) / 2.0;
        let stages = self.stages();
        stages.iter().find(|s| mid >= s.fore_station - TOL && mid < s.aft_station + TOL).or(stages.first()).map(|s| s.index)
    }

    /// Cuts body tube or transition `id` in two at `station` (from the rocket's tip) and returns `(fore piece, aft piece)`:
    /// the fore piece keeps `id`, the aft piece is new and has the same material, finish and wall. Radial and internal parts
    /// go with the piece their middle lies in (limited to it), so a motor mount stays with its stage. A mass or CG override
    /// is shared out by length (the CG override is dropped). This is the fix the Staging warning offers.
    pub fn split_tube(&mut self, id: PartId, station: f64) -> Result<(PartId, PartId), SplitError> {
        let index = self.index_of(id).ok_or(SplitError::NoSuchPart)?;
        let part = self.parts[index].clone();
        if matches!(part.kind, PartKind::NoseCone { .. }) {
            return Err(SplitError::NotSplittable);
        }
        let (fore, length) = (self.stations()[index], part.kind.length());
        let a = station - fore;
        if !(a > TOL && a < length - TOL) {
            return Err(SplitError::OutOfRange);
        }
        let b = length - a;
        let (fore_kind, aft_kind) = match part.kind {
            PartKind::Transition { fore_diameter, aft_diameter, .. } => {
                let middle = part.kind.radius_at(a) * 2.0;
                (
                    PartKind::Transition { length: a, fore_diameter, aft_diameter: middle },
                    PartKind::Transition { length: b, fore_diameter: middle, aft_diameter },
                )
            }
            _ => (part.kind.with_length(a), part.kind.with_length(b)),
        };
        let new_id = self.next_id;
        self.next_id += 1;
        let share = |fraction: f64| part.mass_override.map(|m| m * fraction);
        let aft_part = Part { id: new_id, kind: aft_kind, mass_override: share(b / length), cg_override: None, ..part.clone() };
        self.parts[index].kind = fore_kind;
        self.parts[index].mass_override = share(a / length);
        self.parts[index].cg_override = None;
        self.parts.insert(index + 1, aft_part);

        for r in self.radials.iter_mut().filter(|r| r.parent == id) {
            if r.offset + r.kind.extent() / 2.0 >= a {
                (r.parent, r.offset) = (new_id, r.offset - a);
            }
        }
        for p in self.internals.iter_mut().filter(|p| p.parent == id) {
            if p.offset + p.kind.extent() / 2.0 >= a {
                (p.parent, p.offset) = (new_id, p.offset - a);
            }
        }
        for parent in [id, new_id] {
            self.clamp_radials(parent);
            self.clamp_internals(parent);
        }
        Ok((id, new_id))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::{NoseShape, RadialKind};
    use crate::internal::InternalKind;

    fn mount(overhang: f64) -> InternalKind {
        InternalKind::InnerTube { outer_diameter: 0.054, thickness: 0.0016, length: 0.2, motor_mount: true, overhang }
    }

    fn nose_and_tube(tube_len: f64) -> (Rocket, PartId) {
        let mut r = Rocket::default();
        r.insert(0, PartKind::NoseCone { shape: NoseShape::TangentOgive, length: 0.3, diameter: 0.0762 }).unwrap();
        let t = r.insert(1, PartKind::BodyTube { length: tube_len, diameter: 0.0762 }).unwrap();
        (r, t)
    }

    #[test]
    fn one_motor_mount_is_one_stage() {
        let (mut r, tube) = nose_and_tube(0.8);
        let m = r.attach_internal(tube, 0.6, mount(0.0)).unwrap();
        let stages = r.stages();
        assert_eq!(stages.len(), 1);
        assert_eq!((stages[0].index, stages[0].name.as_str()), (1, "Stage 1"));
        assert_eq!((stages[0].fore_station, stages[0].aft_station), (0.0, 1.1));
        assert_eq!(stages[0].mounts, vec![m]);
        assert!(stages[0].separation.is_none() && stages[0].issue.is_none());
        // No mounts at all: still one stage covering the rocket. An empty rocket has an empty stage.
        let (bare, _) = nose_and_tube(0.8);
        assert_eq!(bare.stages().len(), 1);
        assert_eq!(Rocket::default().stages()[0].aft_station, 0.0);
    }

    #[test]
    fn two_stages_split_at_the_joint_behind_the_upper_motor() {
        let mut r = Rocket::default();
        r.insert(0, PartKind::NoseCone { shape: NoseShape::TangentOgive, length: 0.3, diameter: 0.0762 }).unwrap();
        let upper = r.insert(1, PartKind::BodyTube { length: 0.5, diameter: 0.0762 }).unwrap();
        let booster = r.insert(2, PartKind::BodyTube { length: 0.5, diameter: 0.0762 }).unwrap();
        let m_up = r.attach_internal(upper, 0.3, mount(0.0)).unwrap();
        let m_low = r.attach_internal(booster, 0.3, mount(0.0)).unwrap();

        let stages = r.stages();
        assert_eq!(stages.len(), 2);
        let (s1, s2) = (&stages[0], &stages[1]);
        assert_eq!((s1.index, s2.index), (1, 2));
        assert_eq!(s1.mounts, vec![m_low]);
        assert_eq!(s2.mounts, vec![m_up]);
        // Stage 1 separates at the joint between the two tubes (station 0.8), where the upper motor's aft end is.
        let sep = s1.separation.unwrap();
        assert!((sep.station - 0.8).abs() < 1e-12);
        assert_eq!((sep.joint_between, sep.found_from), ((upper, booster), m_up));
        assert!(s1.issue.is_none() && s2.separation.is_none());
        assert!((s1.fore_station - 0.8).abs() < 1e-12 && (s1.aft_station - 1.3).abs() < 1e-12);
        assert_eq!((s2.fore_station, s2.aft_station), (0.0, s1.fore_station));

        // Parts belong to the stage they lie in, children with their parent.
        let fin = r
            .attach_radial(booster, 0.35, 0.0, 3, RadialKind::FinSet { root_chord: 0.12, tip_chord: 0.05, span: 0.07, sweep: 0.06, thickness: 0.003 })
            .unwrap();
        assert_eq!(
            (r.stage_of(upper), r.stage_of(booster), r.stage_of(fin), r.stage_of(m_up), r.stage_of(m_low)),
            (Some(2), Some(1), Some(1), Some(2), Some(1))
        );
        assert_eq!(r.stage_of(999), None);
        assert!(r.stage_issues().is_empty());

        // A motor that sticks out past the joint (overhang 5 cm) needs a joint further aft: warn, and splitting fixes it.
        r.set_internal_kind(m_up, mount(0.05));
        let stages = r.stages();
        let Some(StageIssue::NoJoint { tube, station }) = stages[0].issue else {
            panic!("expected a warning: {:?}", stages[0]);
        };
        assert_eq!(tube, booster);
        assert!((station - 0.85).abs() < 1e-12);
        assert!(stages[0].separation.is_none());
        assert!((stages[0].fore_station - 0.85).abs() < 1e-12);
        let (fore, aft) = r.split_tube(tube, station).unwrap();
        assert_eq!(fore, booster);
        assert!((r.part(fore).unwrap().kind.length() - 0.05).abs() < 1e-12 && (r.part(aft).unwrap().kind.length() - 0.45).abs() < 1e-12);
        let stages = r.stages();
        let sep = stages[0].separation.unwrap();
        assert!((sep.station - 0.85).abs() < 1e-12 && sep.joint_between == (fore, aft));
        assert!(r.stage_issues().is_empty());
        // The booster's mount and fins moved to the aft piece with their offsets reduced.
        assert_eq!(r.internal(m_low).unwrap().parent, aft);
        assert!((r.internal(m_low).unwrap().offset - 0.25).abs() < 1e-12);
        assert_eq!(r.radial(fin).unwrap().parent, aft);
        assert_eq!(r.motor_mounts().iter().map(|m| m.id).collect::<Vec<_>>(), vec![m_up, m_low]);
    }

    #[test]
    fn both_mounts_in_one_tube_warn_and_split() {
        let (mut r, tube) = nose_and_tube(1.2);
        let upper = r.attach_internal(tube, 0.1, mount(0.0)).unwrap();
        let lower = r.attach_internal(tube, 0.8, mount(0.0)).unwrap();
        let stages = r.stages();
        assert_eq!(stages.len(), 2);
        let Some(StageIssue::NoJoint { tube: t, station }) = stages[0].issue else {
            panic!("expected a warning");
        };
        assert_eq!(t, tube);
        assert!((station - 0.6).abs() < 1e-12, "the upper motor ends at 0.3 + 0.3 = 0.6, got {station}");
        let (_, aft) = r.split_tube(t, station).unwrap();
        let stages = r.stages();
        assert!(stages[0].issue.is_none());
        assert!((stages[0].separation.unwrap().station - 0.6).abs() < 1e-12);
        assert_eq!(r.internal(upper).unwrap().parent, tube);
        assert_eq!(r.internal(lower).unwrap().parent, aft);
        // Three stages work the same way.
        let third = r.insert(r.parts().len(), PartKind::BodyTube { length: 0.5, diameter: 0.0762 }).unwrap();
        r.attach_internal(third, 0.2, mount(0.0)).unwrap();
        assert_eq!(r.stages().len(), 3);
        assert_eq!(r.stages().iter().map(|s| s.index).collect::<Vec<_>>(), vec![1, 2, 3]);
    }

    #[test]
    fn a_cluster_is_one_stage() {
        let (mut r, tube) = nose_and_tube(0.8);
        r.attach_internal(tube, 0.5, mount(0.0)).unwrap();
        r.attach_internal(tube, 0.55, mount(0.0)).unwrap();
        let stages = r.stages();
        assert_eq!(stages.len(), 1);
        assert_eq!(stages[0].mounts.len(), 2);
    }

    #[test]
    fn split_rules_and_properties() {
        let (mut r, tube) = nose_and_tube(0.8);
        r.set_mass_override(tube, Some(0.4));
        r.set_thickness(tube, 0.002);
        let nose = r.parts()[0].id;
        assert_eq!(r.split_tube(nose, 0.1), Err(SplitError::NotSplittable));
        assert_eq!(r.split_tube(999, 0.1), Err(SplitError::NoSuchPart));
        assert_eq!(r.split_tube(tube, 0.3), Err(SplitError::OutOfRange));
        assert_eq!(r.split_tube(tube, 1.1), Err(SplitError::OutOfRange));
        assert_eq!(r.split_tube(tube, 2.0), Err(SplitError::OutOfRange));
        let (a, b) = r.split_tube(tube, 0.5).unwrap();
        let (pa, pb) = (r.part(a).unwrap(), r.part(b).unwrap());
        assert_eq!((pa.thickness, pb.thickness), (0.002, 0.002));
        assert!((pa.kind.length() - 0.2).abs() < 1e-12 && (pb.kind.length() - 0.6).abs() < 1e-12);
        assert!((pa.mass_override.unwrap() - 0.1).abs() < 1e-12 && (pb.mass_override.unwrap() - 0.3).abs() < 1e-12);
        assert!((r.length() - 1.1).abs() < 1e-12);
        assert_eq!(r.index_of(b), Some(2));
        // A transition keeps its profile across the cut.
        let mut t = Rocket::default();
        let tr = t.insert(0, PartKind::Transition { length: 0.1, fore_diameter: 0.1, aft_diameter: 0.05 }).unwrap();
        let (x, y) = t.split_tube(tr, 0.04).unwrap();
        assert!((t.part(x).unwrap().kind.aft_diameter() - 0.08).abs() < 1e-12);
        assert!((t.part(y).unwrap().kind.fore_diameter() - 0.08).abs() < 1e-12);
    }
}
