//! Symmetry groups: a radial part is a group of `count` linked copies spaced evenly around its parent. This module takes a
//! group apart again (design/components/Symmetry): *Detach* lets one copy leave as a free part, *Split all* frees every copy.
//!
//! The model stores a group as one [`RadialPart`] (shared geometry, a start angle and a count), so a group's copies are always
//! evenly spaced. Each free part keeps the angle its copy had; the copies left behind are re-spaced as a smaller group
//! starting at the first of them (changing the count of a linked group re-spaces it, as the Symmetry spec says). Both
//! operations keep the total mass: a mass override of the whole group is shared out by copy.

use crate::geometry::{RadialKind, RadialPart};
use crate::rocket::{RadialError, Rocket};
use crate::PartId;

/// The letter that labels copy `index` of a group (`A`, `B`, ... `Z`, then `AA`).
pub fn instance_letter(index: usize) -> String {
    let (n, letter) = (index / 26, char::from(b'A' + (index % 26) as u8));
    if n == 0 {
        letter.to_string()
    } else {
        format!("{}{letter}", char::from(b'A' + ((n - 1) % 26) as u8))
    }
}

/// The name a copy takes when it leaves its group: `Fin B`, `Launch lug C`, or the group's own name with the letter.
fn free_name(group: &RadialPart, index: usize) -> String {
    let base = match (&group.name, &group.kind) {
        (Some(name), _) => name.clone(),
        (None, kind) if kind.is_fin() => "Fin".to_string(),
        (None, kind) => kind.name().to_string(),
    };
    format!("{base} {}", instance_letter(index))
}

impl Rocket {
    fn fresh_part_id(&mut self) -> PartId {
        let id = self.next_id;
        self.next_id += 1;
        id
    }

    /// A free part made from copy `index` of `group`: one copy at that copy's own angle, with the group's values.
    fn free_copy(&mut self, group: &RadialPart, index: usize) -> RadialPart {
        let count = f64::from(group.count.max(1));
        RadialPart {
            id: self.fresh_part_id(),
            kind: group.kind.clone(),
            angle: group.angles().get(index).copied().unwrap_or(group.angle),
            count: 1,
            name: Some(free_name(group, index)),
            mass_override: group.mass_override.map(|m| m / count),
            cg_override: group.cg_override,
            ..group.clone()
        }
    }

    /// Detaches copy `index` of radial group `id` (see [`RadialPart::angles`]): it becomes a free part of one copy that keeps
    /// the group's current values and its own angle, and is named `Fin B` (the letter of the copy). The rest stay linked as a
    /// group of `count - 1`, re-spaced evenly from the first remaining copy. Returns the id of the new free part.
    ///
    /// Errors: [`RadialError::NoSuchRadial`] if `id` is not a radial group, `index` is out of range or the group has a single
    /// copy (there is nothing to detach from).
    pub fn detach_radial(&mut self, id: PartId, index: usize) -> Result<PartId, RadialError> {
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
        rest.angle = angles[first_left];
        rest.mass_override = group.mass_override.map(|m| m * (count - 1.0) / count);
        self.radials.insert(position + 1, free);
        Ok(free_id)
    }

    /// Splits radial group `id` into free parts, one per copy, each keeping the group's values and its own angle. The group's
    /// own id stays with copy A (the first); the others are new, named `Fin B`, `Fin C`, ... Returns every id in copy order
    /// (so `[id, ..]`); a group of one copy is returned unchanged as `[id]`.
    ///
    /// Errors: [`RadialError::NoSuchRadial`] if `id` is not a radial group.
    pub fn split_all_radial(&mut self, id: PartId) -> Result<Vec<PartId>, RadialError> {
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
        let first = &mut self.radials[position];
        first.count = 1;
        first.name = Some(free_name(&group, 0));
        first.mass_override = group.mass_override.map(|m| m / count);
        for (offset, copy) in copies.into_iter().enumerate() {
            self.radials.insert(position + 1 + offset, copy);
        }
        Ok(ids)
    }

    /// Whether radial part `id` is a fin of a single copy (a free fin, whether detached from a group or placed alone).
    pub fn is_free_fin(&self, id: PartId) -> bool {
        self.radial(id)
            .is_some_and(|r| r.count == 1 && matches!(r.kind, RadialKind::FinSet { .. } | RadialKind::EllipticalFin { .. } | RadialKind::FreeformFin { .. }))
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
        let again = Rocket::from_json(&r.to_json()).unwrap();
        assert_eq!(again, r);
    }
}
