//! The one-time confirmation after an edit lands (design/components/ViewportOverlay, README §7): the part that was just
//! dropped or edited settles into place over 120 ms and gets an accent outline that fades over 400 ms; Station Lanes flashes
//! its bar the same way.
//!
//! The app keeps `flash: Option<(PartId, Instant)>`; this module turns the age into the numbers the views draw with. In a
//! headless `bowshock drive` run nothing animates, so the flash is only taken when `BOWSHOCK_FREEZE_FLASH` is set, and then
//! it is held at its first frame (full outline, part still lifted) so a screenshot can show it.

use std::time::{Duration, Instant};

/// The accent outline fades out over this long.
pub const FLASH_TIME: Duration = Duration::from_millis(400);
/// A released part settles into place over this long.
pub const SETTLE_TIME: Duration = Duration::from_millis(120);
/// How far above its place (metres) a part starts its settle.
const LIFT: f32 = 0.008;
/// The mesh is rebuilt when the level changes by one step of this many per unit.
const STEPS: f32 = 16.0;

/// How long a flash started at `since` has run at `now` (always zero when `freeze` holds it for screenshots).
pub fn age(since: Instant, now: Instant, freeze: bool) -> Duration {
    if freeze {
        Duration::ZERO
    } else {
        now.saturating_duration_since(since)
    }
}

/// Whether the flash has run its course (a frozen one never does).
pub fn done(age: Duration) -> bool {
    age >= FLASH_TIME
}

/// The accent outline's strength, 1 at release falling to 0 over [`FLASH_TIME`] (ease-out), in sixteenths so the mesh is only
/// rebuilt when it visibly changes.
pub fn level(age: Duration) -> f32 {
    let t = (age.as_secs_f32() / FLASH_TIME.as_secs_f32()).min(1.0);
    (((1.0 - t) * (1.0 - t)) * STEPS).round() / STEPS
}

/// How far above its place the part still is (metres): [`LIFT`] at release, 0 after [`SETTLE_TIME`] (ease-out), rounded to
/// quarter millimetres.
pub fn lift(age: Duration) -> f32 {
    let t = (age.as_secs_f32() / SETTLE_TIME.as_secs_f32()).min(1.0);
    ((LIFT * (1.0 - t) * (1.0 - t)) * 4000.0).round() / 4000.0
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_outline_fades_over_400_ms_and_the_part_settles_in_120() {
        assert_eq!(level(Duration::ZERO), 1.0);
        assert!(level(Duration::from_millis(100)) < 1.0 && level(Duration::from_millis(100)) > 0.0);
        assert_eq!(level(FLASH_TIME), 0.0);
        assert_eq!(level(Duration::from_secs(5)), 0.0);
        assert!(lift(Duration::ZERO) > 0.0);
        assert!(lift(Duration::from_millis(60)) < lift(Duration::ZERO));
        assert_eq!(lift(SETTLE_TIME), 0.0);
        // The outline outlives the settle.
        assert!(level(SETTLE_TIME) > 0.0);
        assert!(done(FLASH_TIME) && !done(SETTLE_TIME));
        let since = Instant::now();
        let now = since + Duration::from_secs(1);
        assert_eq!(age(since, now, true), Duration::ZERO);
        assert_eq!(age(since, now, false), Duration::from_secs(1));
    }
}
