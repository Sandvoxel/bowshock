//! Carrying an internal part along the rocket: where it lands for the cursor's place on the axis.
//!
//! The part keeps its station relative to where it was grabbed (`fore + cursor - grab`), so it follows the cursor like a part
//! held by the point under the press. Whichever body part the cursor is over is the part's parent: dragging past the end of one
//! body part carries the internal part into the next (the nearest end part beyond the ends of the rocket). The offset is
//! always from the new parent's fore face, limited so the part lies wholly within it; a body part whose bore the part does not
//! fit refuses it ([`Landing::fits`] is false) and the part stays where it last was.

use bowshock_model::{InternalKind, PartId, Rocket};

/// An internal part being carried by a press on it.
#[derive(Debug, Clone)]
pub struct Carry {
    /// The part.
    pub id: PartId,
    /// Its geometry, for the fit test.
    pub kind: InternalKind,
    /// The station on the axis under the cursor when it was grabbed.
    grab: f64,
    /// The station of the part's fore edge when it was grabbed.
    fore: f64,
}

/// Where the carried part would be for the cursor's place on the axis.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Landing {
    /// The body part under the cursor.
    pub parent: PartId,
    /// Distance from its fore face to the carried part's fore edge (already limited to lie within it).
    pub offset: f64,
    /// Whether the carried part fits the bore there.
    pub fits: bool,
}

impl Carry {
    /// Picks internal part `id` of `rocket` up at station `grab` of the axis; `None` if it is not an internal part of the core.
    pub fn new(rocket: &Rocket, id: PartId, grab: f64) -> Option<Carry> {
        let part = rocket.internals().iter().find(|p| p.id == id)?;
        Some(Carry { id, kind: part.kind.clone(), grab, fore: rocket.part_station(id)? })
    }

    /// Where the part lands with the cursor at station `z` of the axis, its fore edge snapped to a multiple of `step` metres
    /// (any if `None`). `None` when the rocket has no body parts.
    pub fn landing(&self, rocket: &Rocket, z: f64, step: Option<f64>) -> Option<Landing> {
        let stations = rocket.stations();
        let index = stations.get(..stations.len().checked_sub(1)?)?.iter().rposition(|&s| s <= z).unwrap_or(0);
        let parent = rocket.parts().get(index)?.id;
        let mut fore = self.fore + (z - self.grab);
        if let Some(step) = step.filter(|s| *s > 0.0) {
            fore = (fore / step).round() * step;
        }
        let offset = rocket.clamp_internal_offset(parent, fore - stations[index], self.kind.extent())?;
        Some(Landing { parent, offset, fits: rocket.internal_fits(&self.kind, parent, offset) })
    }
}

/// Where a new internal part of `kind` (dragged in from the parts bin) lands with the cursor at station `z` of the axis: in the
/// body part under the cursor, centred on the cursor, its fore edge snapped to a multiple of `step` metres (any if `None`).
pub fn centred_landing(rocket: &Rocket, kind: &InternalKind, z: f64, step: Option<f64>) -> Option<Landing> {
    let stations = rocket.stations();
    let index = stations.get(..stations.len().checked_sub(1)?)?.iter().rposition(|&s| s <= z).unwrap_or(0);
    let parent = rocket.parts().get(index)?.id;
    let mut fore = z - kind.extent() / 2.0;
    if let Some(step) = step.filter(|s| *s > 0.0) {
        fore = (fore / step).round() * step;
    }
    let offset = rocket.clamp_internal_offset(parent, fore - stations[index], kind.extent())?;
    Some(Landing { parent, offset, fits: rocket.internal_fits(kind, parent, offset) })
}

#[cfg(test)]
mod tests {
    use super::*;
    use bowshock_model::{NoseShape, Part, PartKind};

    /// Nose (0 to 0.3), a Ø0.1 tube (to 0.8) and a Ø0.05 tube (to 1.1), with a Ø0.07 motor mount at the fore end of the first
    /// tube.
    fn rocket() -> (Rocket, [PartId; 3], PartId) {
        let mut r = Rocket::default();
        let nose = r.insert(0, PartKind::NoseCone { shape: NoseShape::TangentOgive, length: 0.3, diameter: 0.1 }).unwrap();
        let wide = r.insert(1, PartKind::BodyTube { length: 0.5, diameter: 0.1 }).unwrap();
        r.restore(2, Part::new(60, PartKind::BodyTube { length: 0.3, diameter: 0.05 })).unwrap();
        let mount = r
            .attach_internal(wide, 0.0, InternalKind::InnerTube { outer_diameter: 0.07, thickness: 0.001, length: 0.2, motor_mount: true, overhang: 0.0 })
            .unwrap();
        (r, [nose, wide, 60], mount)
    }

    #[test]
    fn the_part_keeps_its_place_under_the_grab_and_follows_the_cursor() {
        let (r, [_, wide, _], mount) = rocket();
        // Grabbed 0.1 along the part (station 0.4), carried 0.15 aft: still in the same tube, 0.15 further back.
        let carry = Carry::new(&r, mount, 0.4).unwrap();
        let landing = carry.landing(&r, 0.55, None).unwrap();
        assert_eq!(landing.parent, wide);
        assert!((landing.offset - 0.15).abs() < 1e-12 && landing.fits);
        // The offset is limited to keep the whole part in the tube (0.5 long, 0.2 part).
        assert!((carry.landing(&r, 0.75, None).unwrap().offset - 0.3).abs() < 1e-12);
        // Snapped to a 10 mm grid.
        assert!((carry.landing(&r, 0.537, Some(0.01)).unwrap().offset - 0.14).abs() < 1e-9);
    }

    #[test]
    fn past_the_end_of_its_parent_the_part_goes_into_the_next_body_part_if_it_fits() {
        let (r, [nose, _, narrow], mount) = rocket();
        let carry = Carry::new(&r, mount, 0.4).unwrap();
        // Into the nose cone: the cursor is on it, and the Ø0.07 tube does not fit near the tip.
        let fore = carry.landing(&r, 0.1, None).unwrap();
        assert_eq!((fore.parent, fore.offset, fore.fits), (nose, 0.0, false));
        // Into the Ø0.05 tube: over it, but too wide for the bore.
        let aft = carry.landing(&r, 0.95, None).unwrap();
        assert_eq!((aft.parent, aft.fits), (narrow, false));
        // Beyond the aft end it is still the last body part; before the fore end the first.
        assert_eq!(carry.landing(&r, 3.0, None).unwrap().parent, narrow);
        assert_eq!(carry.landing(&r, -1.0, None).unwrap().parent, nose);
    }

    #[test]
    fn a_part_from_the_bin_lands_centred_on_the_cursor_in_the_body_part_under_it() {
        let (r, [_, wide, narrow], mount) = rocket();
        let ring = InternalKind::CenteringRing { outer_diameter: None, inner_diameter: None, thickness: 0.006 };
        let l = centred_landing(&r, &ring, 0.5, None).unwrap();
        assert_eq!((l.parent, l.fits), (wide, true));
        assert!((l.offset - 0.197).abs() < 1e-9, "{}", l.offset);
        // The Ø0.07 mount fits the Ø0.1 tube but not the Ø0.05 one.
        let kind = r.internal(mount).unwrap().kind.clone();
        assert!(centred_landing(&r, &kind, 0.5, None).unwrap().fits);
        assert_eq!(centred_landing(&r, &kind, 0.95, None).map(|l| (l.parent, l.fits)), Some((narrow, false)));
    }

    #[test]
    fn only_core_internal_parts_can_be_carried() {
        let (r, [nose, ..], mount) = rocket();
        assert!(Carry::new(&r, mount, 0.0).is_some() && Carry::new(&r, nose, 0.0).is_none() && Carry::new(&r, 999, 0.0).is_none());
    }
}
