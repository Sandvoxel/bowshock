//! Which status-strip readouts are changing under a drag (design/components/StatusStrip: *While dragging, values that are
//! changing briefly use `ink` at weight 600 so the eye can track them*).
//!
//! A drag, a handle edit, a lane move or a field scrub is *live* from its first change until the mouse is released (the
//! document's merged run or its drag is open). The balance the strip showed before the first change is kept as the baseline;
//! each frame a readout whose value differs from it is [`Changed`] and the strip draws it semibold. On release the baseline is
//! dropped and every readout is regular again.

use bowshock_sim::from_model::Balance;

use super::Ctx;

/// Which readouts of the strip differ from the baseline.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Changed {
    /// MASS.
    pub mass: bool,
    /// LEN.
    pub length: bool,
    /// Ø REF.
    pub diameter: bool,
    /// CG.
    pub cg: bool,
    /// CP.
    pub cp: bool,
    /// STAB (the margin in calibers).
    pub margin: bool,
}

/// Whether two values differ by more than rounding noise (a part dragged back to where it started is unchanged).
fn differs(a: f64, b: f64) -> bool {
    (a - b).abs() > 1e-9 * a.abs().max(b.abs()).max(1e-3)
}

/// The readouts of `now` that changed since `base`.
pub fn changed(base: &Balance, now: &Balance) -> Changed {
    Changed {
        mass: differs(base.mass, now.mass),
        length: differs(base.length, now.length),
        diameter: differs(base.reference_diameter, now.reference_diameter),
        cg: differs(base.cg, now.cg),
        cp: differs(base.cp, now.cp),
        margin: differs(base.margin_cal, now.margin_cal),
    }
}

/// The baseline of the interaction in progress: `None` when none is, `Some(None)` when it began on a rocket with no balance.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Live(Option<Option<Balance>>);

impl Live {
    /// Updates after a message: `before` is the balance as it was before the message ran, `live` whether an interaction is now
    /// open. The first message of an interaction fixes the baseline; the one that ends it clears it.
    pub fn track(&mut self, before: Option<Balance>, live: bool) {
        match (live, self.0) {
            (true, None) => self.0 = Some(before),
            (false, Some(_)) => self.0 = None,
            _ => {}
        }
    }

    /// The readouts that differ from the baseline now (none when nothing is being dragged).
    pub fn changed(&self, now: Option<&Balance>) -> Changed {
        match (self.0.flatten(), now) {
            (Some(base), Some(now)) => changed(&base, now),
            _ => Changed::default(),
        }
    }
}

impl Ctx {
    /// The status-strip readouts that a drag in progress is changing.
    pub fn changing(&self) -> Changed {
        self.live.changed(self.physics.balance())
    }

    /// Call after every message with the balance as it was before it.
    pub(super) fn track_live(&mut self, before: Option<Balance>) {
        let live = self.doc.editing_live() || self.drag.active();
        self.live.track(before, live);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn balance() -> Balance {
        Balance { mass: 1.0, length: 1.0, reference_diameter: 0.1, cg: 0.6, cp: 0.8, margin_cal: 2.0, cn_alpha: 10.0, motor_mass: 0.2 }
    }

    #[test]
    fn only_the_values_that_moved_are_changed() {
        let base = balance();
        let mut now = base;
        assert_eq!(changed(&base, &now), Changed::default());
        now.length = 1.2;
        now.cg = 0.62;
        now.margin_cal = 1.8;
        let c = changed(&base, &now);
        assert_eq!(c, Changed { length: true, cg: true, margin: true, ..Changed::default() });
        // Rounding noise is not a change: a part dragged back to where it began.
        now = base;
        now.mass += 1e-13;
        assert!(!changed(&base, &now).mass);
    }

    #[test]
    fn a_baseline_lasts_from_the_first_change_to_the_release() {
        let mut live = Live::default();
        let base = balance();
        let mut now = base;
        now.mass = 1.5;
        // Nothing is dragged: nothing is semibold, whatever changed.
        live.track(Some(base), false);
        assert_eq!(live.changed(Some(&now)), Changed::default());
        // The first message of the drag fixes the baseline from before it.
        live.track(Some(base), true);
        assert_eq!(live.changed(Some(&now)), Changed { mass: true, ..Changed::default() });
        // Later messages do not move the baseline.
        let mut later = now;
        later.cp = 0.9;
        live.track(Some(now), true);
        assert_eq!(live.changed(Some(&later)), Changed { mass: true, cp: true, ..Changed::default() });
        // Release: back to regular.
        live.track(Some(later), false);
        assert_eq!(live.changed(Some(&later)), Changed::default());
        // A drag that began on an empty rocket has no baseline to compare with.
        live.track(None, true);
        assert_eq!(live.changed(Some(&later)), Changed::default());
    }
}
