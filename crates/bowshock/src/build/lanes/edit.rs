//! The document edits the lanes make: moving a part to a station, changing a length by one of its ends, the targets a carried
//! part would land on, snapping to faces, and adding a part at a station. Pure functions on [`Rocket`], applied by the lanes'
//! `update` through `ctx.doc` (so they undo like every other edit), and unit-tested here.

use bowshock_model::{InternalKind, PartId, PartKind, RadialKind, Rocket};

/// Shortest length a drag may give a part, metres.
pub const MIN_EXTENT: f64 = 0.002;
/// Shift snaps to a face within this many pixels.
pub const SNAP_PX: f32 = 8.0;

/// Which end of a part a resize drags.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum End {
    /// The fore end.
    Fore,
    /// The aft end.
    Aft,
}

/// The axial part whose span holds `station`: the first part for a station ahead of the nose, the last for one behind the tail.
pub fn tube_at(rocket: &Rocket, station: f64) -> Option<PartId> {
    let stations = rocket.stations();
    let parts = rocket.parts();
    if parts.is_empty() {
        return None;
    }
    let i = (0..parts.len()).find(|&i| station < stations[i + 1]).unwrap_or(parts.len() - 1);
    Some(parts[i].id)
}

/// Station of the fore face of axial part `id`.
fn fore_of(rocket: &Rocket, id: PartId) -> Option<f64> {
    rocket.part_station(id)
}

/// Moves part `id` (radial or internal; axial parts reorder through the viewport's drag path) so its fore edge is at `start`.
/// An internal part stops at the walls of the tube it is in; a radial one may go on to the next tube. Returns whether
/// anything was applied.
pub fn move_part(rocket: &mut Rocket, id: PartId, start: f64) -> bool {
    if let Some(radial) = rocket.radial(id) {
        let (parent_now, angle) = (radial.parent, radial.angle);
        let stations = rocket.stations();
        let within = |p: PartId| rocket.index_of(p).is_some_and(|i| start >= stations[i] - 1e-9 && start < stations[i + 1]);
        let Some(parent) = (if within(parent_now) { Some(parent_now) } else { tube_at(rocket, start) }) else {
            return false;
        };
        let Some(fore) = fore_of(rocket, parent) else { return false };
        return rocket.move_radial(id, parent, start - fore, angle).is_ok();
    }
    if let Some(part) = rocket.internal(id) {
        let parent = part.parent;
        let Some(fore) = fore_of(rocket, parent) else { return false };
        return rocket.move_internal(id, parent, start - fore).is_ok();
    }
    false
}

/// `kind` with its length replaced, for the kinds that have one to drag.
fn internal_with_extent(kind: &InternalKind, extent: f64) -> Option<InternalKind> {
    let mut out = kind.clone();
    match &mut out {
        InternalKind::InnerTube { length, .. } | InternalKind::Coupler { length, .. } | InternalKind::MassComponent { length, .. } => *length = extent,
        InternalKind::Parachute { packed_length, .. } | InternalKind::ShockCord { packed_length, .. } => *packed_length = extent,
        InternalKind::Streamer { width, .. } => *width = extent,
        InternalKind::CenteringRing { .. } | InternalKind::Bulkhead { .. } | InternalKind::EngineBlock { .. } => return None,
    }
    Some(out)
}

/// `kind` with its extent along the axis replaced, for the radial kinds that have one to drag.
fn radial_with_extent(kind: &RadialKind, extent: f64) -> Option<RadialKind> {
    let mut out = kind.clone();
    match &mut out {
        RadialKind::FinSet { root_chord, .. } | RadialKind::EllipticalFin { root_chord, .. } => *root_chord = extent,
        RadialKind::LaunchLug { length, .. } => *length = extent,
        RadialKind::FreeformFin { .. } | RadialKind::RailButton { .. } => return None,
    }
    Some(out)
}

/// Drags `end` of part `id` to `station`, changing its length: the aft end of an airframe part (the parts behind it follow),
/// the aft end of a fin or lug, either end of an internal part (kept inside its tube). Returns whether anything was applied.
pub fn resize_part(rocket: &mut Rocket, id: PartId, end: End, station: f64) -> bool {
    if let Some(part) = rocket.part(id) {
        let Some(fore) = fore_of(rocket, id) else { return false };
        if end == End::Fore {
            return false;
        }
        let kind = part.kind.with_length((station - fore).max(MIN_EXTENT));
        return rocket.set_kind(id, kind);
    }
    if let Some(radial) = rocket.radial(id) {
        let Some(fore) = fore_of(rocket, id) else { return false };
        if end == End::Fore {
            return false;
        }
        let Some(kind) = radial_with_extent(&radial.kind, (station - fore).max(MIN_EXTENT)) else { return false };
        let count = radial.count;
        return rocket.set_radial(id, kind, count);
    }
    if let Some(part) = rocket.internal(id) {
        let (parent, kind) = (part.parent, part.kind.clone());
        let (Some(fore), Some((tube_fore, tube_aft))) = (fore_of(rocket, id), rocket.part_span(parent)) else { return false };
        let aft = fore + kind.extent();
        let (new_fore, new_aft) = match end {
            End::Aft => (fore, station.clamp(fore + MIN_EXTENT, tube_aft.max(fore + MIN_EXTENT))),
            End::Fore => (station.clamp(tube_fore, aft - MIN_EXTENT), aft),
        };
        let Some(resized) = internal_with_extent(&kind, new_aft - new_fore) else { return false };
        let changed = rocket.set_internal_kind(id, resized);
        changed && rocket.move_internal(id, parent, new_fore - tube_fore).is_ok()
    } else {
        false
    }
}

/// Where Shift snaps a part spanning `[start, start + extent]`: the shift that brings either of its faces onto the nearest
/// face in `faces` that is within `tolerance` metres, else `start` unchanged.
pub fn snap_span(start: f64, extent: f64, faces: &[f64], tolerance: f64) -> f64 {
    let mut best: Option<f64> = None;
    for &face in faces {
        for edge in [start, start + extent] {
            let d = face - edge;
            if d.abs() <= tolerance && best.is_none_or(|b| d.abs() < b.abs()) {
                best = Some(d);
            }
        }
    }
    start + best.unwrap_or(0.0)
}

/// Snaps one dragged edge at `station` to the nearest of `faces` within `tolerance`.
pub fn snap_edge(station: f64, faces: &[f64], tolerance: f64) -> f64 {
    faces.iter().copied().filter(|f| (f - station).abs() <= tolerance).min_by(|a, b| (a - station).abs().total_cmp(&(b - station).abs())).unwrap_or(station)
}

/// The attach node `kind` would land on with its fore edge near `start`: the nearest valid node. `None` if there is none.
pub fn nearest_node(rocket: &Rocket, kind: &PartKind, start: f64) -> Option<(usize, f64)> {
    rocket.nodes_for(kind).into_iter().min_by(|a, b| (a.1 - start).abs().total_cmp(&(b.1 - start).abs()))
}

/// Where a radial part of `kind` lands with its fore edge at `start`: the tube it is over and the offset in it (limited to the
/// tube). `None` for an empty rocket.
pub fn radial_landing(rocket: &Rocket, kind: &RadialKind, start: f64) -> Option<(PartId, f64)> {
    let parent = tube_at(rocket, start)?;
    let offset = rocket.clamp_offset(parent, start - fore_of(rocket, parent)?, kind)?;
    Some((parent, offset))
}

/// Where an internal part `extent` long lands with its fore edge at `start`: the tube it is over (its middle decides) and the
/// offset in it, limited to the tube's walls.
pub fn internal_landing(rocket: &Rocket, extent: f64, start: f64) -> Option<(PartId, f64)> {
    let parent = tube_at(rocket, start + extent / 2.0)?;
    let offset = rocket.clamp_internal_offset(parent, start - fore_of(rocket, parent)?, extent)?;
    Some((parent, offset))
}

#[cfg(test)]
mod tests {
    use super::*;
    use bowshock_model::NoseShape;

    /// A 0.2 m nose, a 0.5 m tube and a 0.3 m tube (stations 0 / 0.2 / 0.7 / 1.0).
    fn rocket() -> (Rocket, PartId, PartId, PartId) {
        let mut r = Rocket::default();
        let nose = r.insert(0, PartKind::NoseCone { shape: NoseShape::TangentOgive, length: 0.2, diameter: 0.1 }).unwrap();
        let a = r.insert(1, PartKind::BodyTube { length: 0.5, diameter: 0.1 }).unwrap();
        let b = r.insert(2, PartKind::BodyTube { length: 0.3, diameter: 0.1 }).unwrap();
        (r, nose, a, b)
    }

    fn ring() -> InternalKind {
        InternalKind::CenteringRing { outer_diameter: None, inner_diameter: None, thickness: 0.006 }
    }

    #[test]
    fn an_internal_part_moves_to_a_station_and_stops_at_the_tube_walls() {
        let (mut r, _, a, b) = rocket();
        let id = r
            .attach_internal(a, 0.1, InternalKind::MassComponent { mass: 0.1, length: 0.08, diameter: None, role: bowshock_model::MassRole::Ballast })
            .unwrap();
        assert!(move_part(&mut r, id, 0.45));
        assert_eq!(r.part_station(id), Some(0.45));
        // Far behind the tube: stops at its aft wall, never in the next tube.
        assert!(move_part(&mut r, id, 0.95));
        let (fore, aft) = r.part_span(id).unwrap();
        assert!((aft - 0.7).abs() < 1e-9 && (fore - 0.62).abs() < 1e-9);
        assert_eq!(r.internal(id).unwrap().parent, a);
        // And ahead of the front wall.
        assert!(move_part(&mut r, id, -1.0));
        assert!((r.part_station(id).unwrap() - 0.2).abs() < 1e-9);
        let _ = b;
    }

    #[test]
    fn a_radial_part_follows_the_station_onto_the_next_tube() {
        let (mut r, _, a, b) = rocket();
        let id = r.attach_radial(a, 0.3, 0.0, 3, RadialKind::FinSet { root_chord: 0.1, tip_chord: 0.05, span: 0.05, sweep: 0.03, thickness: 0.003 }).unwrap();
        assert!(move_part(&mut r, id, 0.5));
        assert_eq!((r.radial(id).unwrap().parent, r.part_station(id)), (a, Some(0.5)));
        // Across the joint at 0.7: now on the second tube.
        assert!(move_part(&mut r, id, 0.75));
        assert_eq!(r.radial(id).unwrap().parent, b);
        assert!((r.part_station(id).unwrap() - 0.75).abs() < 1e-9);
        assert_eq!(r.radial(id).unwrap().count, 3);
    }

    #[test]
    fn resizing_an_airframe_part_pushes_the_ones_behind_it() {
        let (mut r, _, a, b) = rocket();
        assert!(resize_part(&mut r, a, End::Aft, 0.9));
        assert!((r.part(a).unwrap().kind.length() - 0.7).abs() < 1e-9);
        assert!((r.part_station(b).unwrap() - 0.9).abs() < 1e-9);
        // The length never goes below the minimum and the fore end of an airframe part cannot be dragged.
        assert!(resize_part(&mut r, a, End::Aft, 0.0));
        assert_eq!(r.part(a).unwrap().kind.length(), MIN_EXTENT);
        assert!(!resize_part(&mut r, a, End::Fore, 0.3));
    }

    #[test]
    fn resizing_an_internal_part_keeps_it_in_its_tube_and_either_end_works() {
        let (mut r, _, a, _) = rocket();
        let id = r.attach_internal(a, 0.1, InternalKind::Coupler { outer_diameter: None, thickness: 0.002, length: 0.1 }).unwrap();
        // Aft end out past the tube's end stops at the wall (station 0.7).
        assert!(resize_part(&mut r, id, End::Aft, 5.0));
        assert!((r.part_span(id).unwrap().1 - 0.7).abs() < 1e-9 && (r.part_station(id).unwrap() - 0.3).abs() < 1e-9);
        // Fore end: the aft end stays, the offset follows.
        assert!(resize_part(&mut r, id, End::Fore, 0.5));
        let (fore, aft) = r.part_span(id).unwrap();
        assert!((fore - 0.5).abs() < 1e-9 && (aft - 0.7).abs() < 1e-9, "{fore} {aft}");
        // A disc has no length to drag.
        let disc = r.attach_internal(a, 0.2, ring()).unwrap();
        assert!(!resize_part(&mut r, disc, End::Aft, 0.6));
    }

    #[test]
    fn a_fin_set_changes_its_root_chord_and_keeps_its_count() {
        let (mut r, _, a, _) = rocket();
        let id = r.attach_radial(a, 0.3, 0.0, 4, RadialKind::FinSet { root_chord: 0.1, tip_chord: 0.05, span: 0.05, sweep: 0.03, thickness: 0.003 }).unwrap();
        assert!(resize_part(&mut r, id, End::Aft, 0.65));
        let fin = r.radial(id).unwrap();
        assert_eq!(fin.count, 4);
        assert!((fin.kind.extent() - 0.15).abs() < 1e-9);
        let button = r.attach_radial(a, 0.1, 0.0, 2, RadialKind::RailButton { height: 0.008, diameter: 0.011 }).unwrap();
        assert!(!resize_part(&mut r, button, End::Aft, 0.5));
    }

    #[test]
    fn shift_snaps_either_face_to_a_neighbouring_face() {
        let faces = [0.2, 0.7];
        // Fore edge 3 mm ahead of the face at 0.2.
        assert!((snap_span(0.197, 0.1, &faces, 0.005) - 0.2).abs() < 1e-12);
        // The aft edge (0.69 + 0.0104) lands on 0.7.
        assert!((snap_span(0.6, 0.1004, &faces, 0.005) - 0.5996).abs() < 1e-9);
        // Nothing near: unchanged.
        assert_eq!(snap_span(0.4, 0.05, &faces, 0.005), 0.4);
        assert_eq!(snap_edge(0.698, &faces, 0.005), 0.7);
        assert_eq!(snap_edge(0.5, &faces, 0.005), 0.5);
    }

    #[test]
    fn landings_pick_the_tube_under_the_station() {
        let (r, nose, a, b) = rocket();
        assert_eq!(tube_at(&r, -0.1), Some(nose));
        assert_eq!(tube_at(&r, 0.3), Some(a));
        assert_eq!(tube_at(&r, 5.0), Some(b));
        let fins = RadialKind::FinSet { root_chord: 0.1, tip_chord: 0.05, span: 0.05, sweep: 0.03, thickness: 0.003 };
        assert_eq!(radial_landing(&r, &fins, 0.75).map(|(p, o)| (p, (o * 1000.0).round())), Some((b, 50.0)));
        // An internal part's middle decides the tube, and its offset is limited to the walls.
        let (p, offset) = internal_landing(&r, 0.1, 0.66).unwrap();
        assert_eq!(p, b);
        assert!((offset - 0.0).abs() < 1e-9, "{offset}");
        let (p, offset) = internal_landing(&r, 0.1, 0.25).unwrap();
        assert_eq!((p, (offset * 1000.0).round()), (a, 50.0));
        assert_eq!(nearest_node(&r, &PartKind::BodyTube { length: 0.1, diameter: 0.1 }, 0.68).map(|n| n.0), Some(2));
    }
}
