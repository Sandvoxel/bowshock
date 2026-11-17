//! What the viewport, Station Lanes and inspector share about symmetry groups beyond the document itself: which copy of the
//! selected group is the clicked one (its letter A/B/C, the one *Unlink for this fin* and *Detach* act on) and a pending
//! *Rejoin group* waiting for the user to choose whose settings win. Both are view state, not document state: they are not
//! undone and not saved.

use bowshock_model::{PartId, Rocket};

/// A free part about to rejoin a group, waiting for the choice of whose settings win.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Rejoin {
    /// The free part that joins.
    pub part: PartId,
    /// The group it joins.
    pub group: PartId,
}

/// Shared symmetry state, in `Ctx::symmetry`.
#[derive(Debug, Clone, Default)]
pub struct SymmetryUi {
    /// The clicked copy of a radial group: the group and the copy's index (0 is A). Only meaningful while that group is selected.
    pub instance: Option<(PartId, usize)>,
    /// A rejoin waiting for the user's choice (the inspector shows it).
    pub rejoin: Option<Rejoin>,
}

impl SymmetryUi {
    /// The copy of group `id` the user is on: the clicked one, `0` (A) when none was or it is another group's.
    pub fn copy_of(&self, id: PartId) -> usize {
        match self.instance {
            Some((part, index)) if part == id => index,
            _ => 0,
        }
    }

    /// Like [`copy_of`](Self::copy_of) but wrapped into the group's copies (an index past the last copy wraps to A).
    pub fn copy_in(&self, rocket: &Rocket, id: PartId) -> usize {
        let count = rocket.radial(id).map_or(1, |r| usize::from(r.count.max(1)));
        self.copy_of(id) % count
    }

    /// Forgets a pending rejoin whose parts no longer exist or can no longer join (called after every message).
    pub fn validate(&mut self, rocket: &Rocket) {
        if self.rejoin.is_some_and(|r| !rocket.can_rejoin(r.part, r.group)) {
            self.rejoin = None;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use bowshock_model::{NoseShape, PartKind, RadialKind};

    #[test]
    fn the_clicked_copy_belongs_to_its_group_and_wraps() {
        let mut r = Rocket::default();
        r.insert(0, PartKind::NoseCone { shape: NoseShape::TangentOgive, length: 0.2, diameter: 0.07, tip_radius: 0.0 }).unwrap();
        let tube = r.insert(1, PartKind::BodyTube { length: 0.5, diameter: 0.07 }).unwrap();
        let fins =
            r.attach_radial(tube, 0.3, 0.0, 3, RadialKind::FinSet { root_chord: 0.1, tip_chord: 0.05, span: 0.06, sweep: 0.04, thickness: 0.003 }).unwrap();
        let mut ui = SymmetryUi { instance: Some((fins, 4)), rejoin: None };
        assert_eq!((ui.copy_of(fins), ui.copy_in(&r, fins), ui.copy_of(fins + 1)), (4, 1, 0));
        // A rejoin of parts that cannot join is dropped.
        ui.rejoin = Some(Rejoin { part: fins, group: fins });
        ui.validate(&r);
        assert_eq!(ui.rejoin, None);
    }
}
