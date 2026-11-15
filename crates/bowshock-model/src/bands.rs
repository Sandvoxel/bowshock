//! The stability thresholds of a document (the StabilityGauge's bands, "user-configurable in project settings"): below
//! `marginal` the rocket is unstable, up to `stable` marginal, above `stable` stable, and above `over` over-stable. Stored with
//! the document, so the gauge, the status strip, the Analyze reference lines and the health warnings all read the same numbers.

use serde::{Deserialize, Serialize};

use crate::rocket::Rocket;

/// Margin thresholds in calibers. See the module docs.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct StabilityBands {
    /// Below this the rocket is unstable.
    pub marginal: f64,
    /// From `marginal` up to this the margin is marginal; the usual rule of thumb is one caliber.
    pub stable: f64,
    /// Above this the rocket is over-stable (weathercocking risk); also the end of the gauge's scale.
    pub over: f64,
}

impl Default for StabilityBands {
    /// The kit's defaults: 0.5, 1.0 and 3.0 calibers.
    fn default() -> Self {
        StabilityBands { marginal: 0.5, stable: 1.0, over: 3.0 }
    }
}

impl StabilityBands {
    /// Smallest gap kept between consecutive thresholds, calibers.
    pub const MIN_GAP: f64 = 0.1;
    /// Largest value any threshold may take, calibers.
    pub const MAX: f64 = 20.0;

    /// The bands with non-finite values replaced by the defaults and the thresholds forced into order
    /// (`0 <= marginal < stable < over <= MAX`, each at least [`MIN_GAP`](Self::MIN_GAP) apart).
    pub fn sanitized(self) -> Self {
        let d = StabilityBands::default();
        let pick = |v: f64, fallback: f64| if v.is_finite() { v } else { fallback };
        let marginal = pick(self.marginal, d.marginal).clamp(0.0, Self::MAX - 2.0 * Self::MIN_GAP);
        let stable = pick(self.stable, d.stable).clamp(marginal + Self::MIN_GAP, Self::MAX - Self::MIN_GAP);
        let over = pick(self.over, d.over).clamp(stable + Self::MIN_GAP, Self::MAX);
        StabilityBands { marginal, stable, over }
    }
}

/// Whether `b` is the defaults (so the document need not store it).
pub(crate) fn is_default(b: &StabilityBands) -> bool {
    *b == StabilityBands::default()
}

impl Rocket {
    /// The stability thresholds of this document.
    pub fn stability_bands(&self) -> StabilityBands {
        self.bands
    }

    /// Sets the stability thresholds (forced into order, see [`StabilityBands::sanitized`]).
    pub fn set_stability_bands(&mut self, bands: StabilityBands) {
        self.bands = bands.sanitized();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn defaults_and_ordering() {
        let d = StabilityBands::default();
        assert_eq!((d.marginal, d.stable, d.over), (0.5, 1.0, 3.0));
        let s = StabilityBands { marginal: 2.0, stable: 1.0, over: f64::NAN }.sanitized();
        assert!(s.marginal < s.stable && s.stable < s.over, "{s:?}");
        assert_eq!(s.marginal, 2.0);
        let s = StabilityBands { marginal: -1.0, stable: 0.0, over: 100.0 }.sanitized();
        assert_eq!((s.marginal, s.over), (0.0, StabilityBands::MAX));
    }

    #[test]
    fn bands_are_saved_and_default_documents_stay_unchanged() {
        let mut r = Rocket::default();
        assert!(!r.to_json().contains("bands"));
        r.set_stability_bands(StabilityBands { marginal: 0.8, stable: 1.5, over: 4.0 });
        let again = Rocket::from_json(&r.to_json()).unwrap();
        assert_eq!(again.stability_bands(), StabilityBands { marginal: 0.8, stable: 1.5, over: 4.0 });
    }
}
