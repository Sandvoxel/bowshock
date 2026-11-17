//! Stages, derived (never stored) from the motor mounts: one stage per in-line group of mounts, split at a joint between
//! airframe parts: by default the first one at or aft of the upper motor's aft end, or the joint the user chose (the one stored
//! per-rocket choice, see [`Rocket::set_separation_joint`]).

use std::fmt;

use crate::config::MotorMount;
use crate::geometry::{Part, PartKind, Shoulder};
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
    /// The motor mount of the upper stage whose aft end the default joint is found from.
    pub found_from: PartId,
    /// The joint was chosen by the user (see [`Rocket::set_separation_joint`]) rather than found by the default rule.
    pub chosen: bool,
}

/// A joint a separation can be moved to (see [`Rocket::separation_joints`]).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct JointOption {
    /// Station of the joint, from the rocket's tip.
    pub station: f64,
    /// The airframe parts either side of the joint, fore then aft. The aft part is what the choice is stored as.
    pub between: (PartId, PartId),
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

/// Everything about one separation (the boundary between cluster `p - 1` and cluster `p`) that the default rule and the
/// choice share.
struct Gap {
    /// Key of the stored choice: the upper cluster's fore-most mount.
    key: PartId,
    /// The upper cluster's mount with the aft-most motor end.
    source: MotorMount,
    /// The joints a separation here can take, fore to aft.
    eligible: Vec<JointOption>,
    /// The first joint at or aft of the upper motor's aft end (the default rule).
    default: Option<JointOption>,
}

impl Rocket {
    /// The separation between cluster `p - 1` (above) and cluster `p`, or `None` when either has no motors.
    fn gap(&self, clusters: &[Vec<MotorMount>], stations: &[f64], p: usize) -> Option<Gap> {
        let (upper, lower) = (clusters.get(p.checked_sub(1)?)?, clusters.get(p)?);
        let source = upper.iter().max_by(|a, b| a.motor_aft_station.total_cmp(&b.motor_aft_station)).copied()?;
        let lower_fore = lower.iter().map(|m| m.fore_station).reduce(f64::min)?;
        let tube_aft = upper.iter().map(|m| m.aft_station).fold(f64::NEG_INFINITY, f64::max);
        let reach = source.motor_aft_station.min(tube_aft);
        let joints: Vec<JointOption> =
            (1..self.parts.len()).map(|j| JointOption { station: stations[j], between: (self.parts[j - 1].id, self.parts[j].id) }).collect();
        let default = joints.iter().find(|j| j.station >= source.motor_aft_station - TOL && j.station <= lower_fore + TOL).copied();
        let eligible = joints.into_iter().filter(|j| j.station >= reach - TOL && j.station <= lower_fore + TOL).collect();
        Some(Gap { key: upper[0].id, source, eligible, default })
    }

    /// The stages, in firing order (stage 1, the aft-most, first). A rocket with fewer than two in-line motor mounts (a
    /// cluster is one stage) has the single stage 1 and no separation. See the Staging spec: the separation of a stage is the
    /// joint the user chose for it ([`Rocket::set_separation_joint`]) while that choice is eligible, otherwise the first joint
    /// between airframe parts at or aft of the upper motor's aft end (mount aft end plus overhang) and at or fore of the
    /// stage's own mounts; if there is none the stage carries a [`StageIssue::NoJoint`].
    pub fn stages(&self) -> Vec<Stage> {
        let clusters = self.mount_clusters();
        let length = self.length();
        let count = clusters.len().max(1);
        let stations = self.stations();

        // For cluster `p` (0 is the fore-most): the separation from the cluster ahead of it.
        let find = |p: usize| -> (Option<Separation>, Option<StageIssue>) {
            let Some(gap) = self.gap(&clusters, &stations, p) else {
                return (None, None);
            };
            let chosen = self.separation_joints.get(&gap.key).and_then(|&aft| gap.eligible.iter().find(|j| j.between.1 == aft)).copied();
            match chosen.or(gap.default) {
                Some(j) => (Some(Separation { station: j.station, joint_between: j.between, found_from: gap.source.id, chosen: chosen.is_some() }), None),
                None => {
                    let upper_aft = gap.source.motor_aft_station;
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

    /// The joints the separation of stage `stage` (the [`Stage::index`] of the stage below the joint: 1 for a two-stage
    /// rocket) can be moved to, fore to aft; empty when that stage has no separation. A joint between two airframe parts is
    /// eligible when it lies at or aft of the upper stage's
    /// motor-mount reach, `min(aft end of the upper motor, aft end of the upper motor-mount tube)` (so the upper stage
    /// contains its mount tube, and its motor too unless the motor overhangs the tube), and at or fore of the fore end of
    /// the lower stage's foremost motor-mount tube (so the lower stage contains its mounts). The default joint (see
    /// [`stages`](Self::stages)) is always among them when it exists. A motor overhanging the chosen joint is not warned about.
    pub fn separation_joints(&self, stage: usize) -> Vec<JointOption> {
        let clusters = self.mount_clusters();
        let count = clusters.len().max(1);
        if stage < 1 || stage >= count {
            return Vec::new();
        }
        self.gap(&clusters, &self.stations(), count - stage).map(|g| g.eligible).unwrap_or_default()
    }

    /// The joint the user chose for stage `stage`'s separation, as the first airframe part behind it, if that choice is stored
    /// and still eligible; `None` when the separation is automatic.
    pub fn separation_choice(&self, stage: usize) -> Option<PartId> {
        let sep = self.stages().into_iter().find(|s| s.index == stage)?.separation?;
        sep.chosen.then_some(sep.joint_between.1)
    }

    /// Chooses the joint stage `stage` separates at: `Some(part)` stores the airframe part just aft of the joint (it must be
    /// one of [`separation_joints`](Self::separation_joints)), `None` returns to the automatic joint. Returns whether
    /// anything changed; an ineligible part changes nothing. The choice is keyed by the upper stage's foremost motor mount,
    /// not by the stage number, so adding or removing other stages does not move it, and it is kept (but ignored while
    /// ineligible) when parts come and go. Choosing the joint the automatic rule would pick stores no choice.
    pub fn set_separation_joint(&mut self, stage: usize, joint: Option<PartId>) -> bool {
        let clusters = self.mount_clusters();
        let count = clusters.len().max(1);
        if stage < 1 || stage >= count {
            return false;
        }
        let Some(gap) = self.gap(&clusters, &self.stations(), count - stage) else {
            return false;
        };
        let wanted = match joint {
            None => None,
            Some(aft) if gap.default.is_some_and(|d| d.between.1 == aft) => None,
            Some(aft) if gap.eligible.iter().any(|j| j.between.1 == aft) => Some(aft),
            Some(_) => return false,
        };
        match wanted {
            Some(aft) => self.separation_joints.insert(gap.key, aft) != Some(aft),
            None => self.separation_joints.remove(&gap.key).is_some(),
        }
    }

    /// Every stage's issue, in firing order.
    pub fn stage_issues(&self) -> Vec<(usize, StageIssue)> {
        self.stages().into_iter().filter_map(|s| s.issue.map(|i| (s.index, i))).collect()
    }

    /// The motor mounts grouped into clusters (mounts whose tubes overlap along the axis), fore-most cluster first.
    fn mount_clusters(&self) -> Vec<Vec<MotorMount>> {
        let mut clusters: Vec<Vec<MotorMount>> = Vec::new();
        let mut reach = f64::NEG_INFINITY;
        for mount in self.motor_mounts().into_iter().filter(|m| m.pod.is_none()) {
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
        let carried = self.carried_ids(id);
        self.release_touching(&carried);
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
        let share_cd = |fraction: f64| part.cd_override.map(|c| c * fraction);
        let aft_part = Part {
            id: new_id,
            kind: aft_kind,
            mass_override: share(b / length),
            cg_override: None,
            cd_override: share_cd(b / length),
            shoulder_fore: Shoulder::NONE,
            ..part.clone()
        };
        self.parts[index].kind = fore_kind;
        self.parts[index].shoulder_aft = Shoulder::NONE;
        self.parts[index].mass_override = share(a / length);
        self.parts[index].cd_override = share_cd(a / length);
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
        r.insert(0, PartKind::NoseCone { shape: NoseShape::TangentOgive, length: 0.3, diameter: 0.0762, tip_radius: 0.0 }).unwrap();
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
        r.insert(0, PartKind::NoseCone { shape: NoseShape::TangentOgive, length: 0.3, diameter: 0.0762, tip_radius: 0.0 }).unwrap();
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

    /// Nose, upper tube, two interstage tubes and a booster; one motor mount in the upper tube and one in the booster.
    /// Joints at 0.8 (upper | inter1), 1.0 (inter1 | inter2), 1.2 (inter2 | booster).
    fn three_joint_rocket() -> (Rocket, [PartId; 4], [PartId; 2]) {
        let mut r = Rocket::default();
        r.insert(0, PartKind::NoseCone { shape: NoseShape::TangentOgive, length: 0.3, diameter: 0.0762, tip_radius: 0.0 }).unwrap();
        let upper = r.insert(1, PartKind::BodyTube { length: 0.5, diameter: 0.0762 }).unwrap();
        let inter1 = r.insert(2, PartKind::BodyTube { length: 0.2, diameter: 0.0762 }).unwrap();
        let inter2 = r.insert(3, PartKind::BodyTube { length: 0.2, diameter: 0.0762 }).unwrap();
        let booster = r.insert(4, PartKind::BodyTube { length: 0.5, diameter: 0.0762 }).unwrap();
        let m_up = r.attach_internal(upper, 0.3, mount(0.0)).unwrap();
        let m_low = r.attach_internal(booster, 0.3, mount(0.0)).unwrap();
        (r, [upper, inter1, inter2, booster], [m_up, m_low])
    }

    #[test]
    fn a_chosen_joint_moves_the_stage_spans_and_the_default_stays_without_one() {
        let (mut r, [upper, inter1, inter2, booster], _) = three_joint_rocket();
        let auto = r.stages();
        let sep = auto[0].separation.unwrap();
        assert!((sep.station - 0.8).abs() < 1e-12 && sep.joint_between == (upper, inter1) && !sep.chosen);
        assert_eq!(r.separation_choice(1), None);
        let joints = r.separation_joints(1);
        assert_eq!(joints.iter().map(|j| j.between).collect::<Vec<_>>(), vec![(upper, inter1), (inter1, inter2), (inter2, booster)]);
        assert!(r.separation_joints(2).is_empty() && r.separation_joints(0).is_empty());

        assert!(r.set_separation_joint(1, Some(inter2)));
        assert!(!r.set_separation_joint(1, Some(inter2)), "no change the second time");
        let stages = r.stages();
        let sep = stages[0].separation.unwrap();
        assert!(sep.chosen && (sep.station - 1.0).abs() < 1e-12 && sep.joint_between == (inter1, inter2));
        assert!((stages[0].fore_station - 1.0).abs() < 1e-12 && (stages[1].aft_station - 1.0).abs() < 1e-12);
        assert_eq!((r.stage_of(inter1), r.stage_of(inter2)), (Some(2), Some(1)));
        assert_eq!(r.separation_choice(1), Some(inter2));

        // The default joint, ineligible parts and unknown stages are not stored.
        assert!(r.set_separation_joint(1, Some(inter1)), "back to the default joint clears the choice");
        assert_eq!(r.separation_choice(1), None);
        assert!(!r.set_separation_joint(1, Some(upper)) && !r.set_separation_joint(1, Some(999)) && !r.set_separation_joint(2, Some(inter2)));
        assert!(r.set_separation_joint(1, Some(inter2)));
        assert!(r.set_separation_joint(1, None));
        assert_eq!(r.stages(), auto);
    }

    #[test]
    fn a_stale_choice_falls_back_to_automatic_and_comes_back() {
        let (mut r, [upper, inter1, inter2, booster], [_, m_low]) = three_joint_rocket();
        // Ineligible: the lower mount moves up to station 1.0, so the joint at 1.2 is aft of it.
        assert!(r.set_separation_joint(1, Some(booster)));
        let chosen = r.clone();
        r.move_internal(m_low, inter2, 0.0).unwrap();
        let sep = r.stages()[0].separation.unwrap();
        assert!(!sep.chosen && sep.joint_between == (upper, inter1), "{sep:?}");
        assert_eq!(r.separation_choice(1), None);
        assert!(!r.separation_joints(1).iter().any(|j| j.between.1 == booster));
        // Not deleted: moving the mount back makes it count again.
        r.move_internal(m_low, booster, 0.3).unwrap();
        assert_eq!(r.stages(), chosen.stages());
        // A deleted part: the choice is ignored.
        let (mut r, [upper, inter1, inter2, _], _) = three_joint_rocket();
        assert!(r.set_separation_joint(1, Some(inter2)));
        r.remove(inter2);
        let sep = r.stages()[0].separation.unwrap();
        assert!(!sep.chosen && sep.joint_between == (upper, inter1));
    }

    #[test]
    fn a_choice_survives_a_save_and_old_documents_have_none() {
        let (mut r, [_, _, inter2, _], _) = three_joint_rocket();
        assert!(r.set_separation_joint(1, Some(inter2)));
        let back = Rocket::from_bytes(&r.to_bytes()).unwrap();
        assert_eq!(back, r);
        assert_eq!(back.separation_choice(1), Some(inter2));
        let (plain, _, _) = three_joint_rocket();
        assert!(Rocket::from_bytes(&plain.to_bytes()).unwrap().separation_joints.is_empty());
    }
}
