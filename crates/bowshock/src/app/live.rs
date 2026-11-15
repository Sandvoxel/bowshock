//! Which status-strip readouts are changing under a drag (design/components/StatusStrip: *While dragging, values that are
//! changing briefly use `ink` at weight 600 so the eye can track them*).
//!
//! A drag, a handle edit, a lane move or a field scrub is *live* from its first change until the mouse is released (the
//! document's merged run or its drag is open). The balance the strip showed before the first change is kept as the baseline;
//! each frame a readout whose value differs from it is [`Changed`] and the strip draws it semibold. On release the baseline is
//! dropped and every readout is regular again.

use std::time::{Duration, Instant};

use bowshock_sim::from_model::{Balance, Phase};

use super::drag::{self, Landing};
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

impl Live {
    /// The balance the interaction in progress began with: the committed one, which a preview is compared with.
    pub fn baseline(&self) -> Option<Balance> {
        self.0.flatten()
    }
}

/// The balance every surface shows right now, and the committed one it replaces while a change is being previewed.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Shown {
    /// What the status strip, the markers and the lanes draw: the rocket as it would be if released now.
    pub now: Option<Balance>,
    /// The balance as committed (before the interaction began), when it differs from `now`: drawn faint beside it, and the
    /// strip's deltas are measured from it.
    pub committed: Option<Balance>,
}

/// The preview of a carried part's landing: the balance of the rocket as it would be if the drag ended now.
///
/// It is worked out when the landing changes (the part snaps to another node, slides along a tube) and never otherwise, from
/// the latest state only: the app calls [`Ctx::sync_shown`] once per message, so a burst of cursor moves that lands in the
/// same place costs nothing and one that lands in many costs one clone, one balance and one health check each. The health list
/// follows the carry too: of the landed rocket, or of the rocket as it was before the drag while the part has no target (never
/// of the rocket with the part out of it).
#[derive(Debug, Clone, Default)]
pub struct Preview {
    /// What the preview was worked out for: `Some` while a part is carried, with its landing (`None`: no valid target).
    key: Option<(u64, Phase, Option<Landing>)>,
    balance: Option<Balance>,
    /// How long the last preview took to work out (rocket clone, landing, balance).
    pub cost: Duration,
}

impl Ctx {
    /// The balance on show: while a part is carried the preview of its landing (or the committed balance when it has no valid
    /// target); while a handle or a bar is dragged the live one with the committed one beside it; else the cached one.
    pub fn shown(&self) -> Shown {
        let base = self.live.baseline();
        let differs = |a: &Balance, b: &Balance| changed(a, b) != Changed::default();
        if self.drag.active() {
            return match self.preview.balance {
                Some(now) => Shown { now: Some(now), committed: base.filter(|b| differs(b, &now)) },
                None => Shown { now: base, committed: None },
            };
        }
        let now = self.physics.balance().copied();
        let committed = match (base, &now) {
            (Some(b), Some(n)) if self.doc.editing_live() && differs(&b, n) => Some(b),
            _ => None,
        };
        Shown { now, committed }
    }

    /// Works out the preview of the carried part's landing if the landing changed since the last time.
    fn refresh_preview(&mut self) {
        let key = self.drag.active().then(|| (self.doc.revision(), self.physics.phase(), self.drag.landing()));
        if key == self.preview.key {
            return;
        }
        let start = Instant::now();
        let landed = key.as_ref().and_then(|(_, _, landing)| landing.as_ref()).and_then(|_| drag::landed(self));
        self.preview.balance = landed.as_ref().and_then(|rocket| self.physics.balance_of(rocket));
        // No target: the rocket as it was before the drag (a bin ghost has not changed the document; a picked-up part has).
        let warnings = match &landed {
            Some(rocket) => Some(self.physics.warnings_for(rocket, self.preview.balance.as_ref())),
            None => key.as_ref().map(|_| {
                let before = self.doc.drag_base().unwrap_or(self.doc.rocket());
                self.physics.warnings_for(before, self.live.baseline().as_ref().or(self.physics.balance()))
            }),
        };
        self.physics.show_warnings(warnings);
        self.preview.key = key;
        self.preview.cost = start.elapsed();
    }

    /// Call after every message, once [`track_live`](Self::track_live) has run: refreshes the preview and points the strip's
    /// gauge at the margin on show.
    pub(super) fn sync_shown(&mut self) {
        self.refresh_preview();
        let margin = self.shown().now.map(|b| b.margin_cal);
        self.physics.show_margin(margin);
    }
}

impl Ctx {
    /// The status-strip readouts that a drag in progress is changing.
    pub fn changing(&self) -> Changed {
        self.live.changed(self.shown().now.as_ref())
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
