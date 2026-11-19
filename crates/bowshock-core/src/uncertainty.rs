//! The uncertainty bands of the 3D engine's results: measured terms in one table, no second sweep.
//!
//! Every number below is a measurement against free flight, a tunnel or a refined grid, taken with the production tiers (`crates/bowshock-cfd/DESIGN3D.md`, items
//! 45, 46, 47, 50 and 52; `HANDOFF3D.md` section 0). The bands are what the application can honestly say about *its own* number today: the Basic Finner is the one
//! rocket measured, so they are the scatter of that comparison, widened where the measurements disagree, not a statistical confidence interval.
//!
//! A [`Band`] is one-sided on each side: the true value is expected within `[value - low, value + high]`. Systematic terms (model bias, stop rule, wall temperature)
//! add linearly on the side they push; the random terms (grid, reference scatter) add in quadrature to both sides.
//!
//! **Not in the bands:** launch lugs and rail buttons are not meshed (the table adds the build-up method's drag for them, `CfdPoint::other`); one lug changed the
//! apogee of a model-scale rocket by +16 % when it was measured. Fin cant, fin sets beyond the first, pods, and a rough or painted surface that is not described
//! to the solver are not in the bands either.

use crate::analysis::Resolution;

/// A one-sided pair of margins around a value: the truth is expected within `[value - low, value + high]` (`low`, `high` >= 0, in the unit of the value).
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Band {
    /// How far below the value the truth may be.
    pub low: f64,
    /// How far above the value the truth may be.
    pub high: f64,
}

impl Band {
    /// The larger of the two margins: the symmetric half width that holds the truth on both sides.
    pub fn half_width(self) -> f64 {
        self.low.max(self.high)
    }
}

/// THE TABLE: every term of every band, as a fraction of the value (drag) or in calibers (centre of pressure). `[low, high]` of a term is the range it was measured
/// over (the pair `(a, b)` is the smaller and the larger magnitude); `None` means the term does not act on that side.
pub mod terms {
    /// Model bias of the CD0 against free flight, fully turbulent smooth model (DESIGN3D items 46, 47; HANDOFF3D section 0: +5.6 / +2.6 / +6.7 % at Mach 0.766 / 1.85 /
    /// 2.741 on the grid-converged model). The model is high, so the flight value is *below* the table: this is a `low` margin, at most [`MODEL_BIAS_CD_MAX`]. A smooth
    /// model with a plausible transition removes most of it (item 50); a painted, rail-buttoned rocket is the fully turbulent case, the conservative side.
    pub const MODEL_BIAS_CD_MAX: f64 = 0.067;
    /// The smallest model bias measured (the term is not subtracted from the value: it is the table's own, documented offset).
    pub const MODEL_BIAS_CD_MIN: f64 = 0.026;
    /// Stop rule (`drift_tol` 5e-3): the stopped drag is 0.3-1.0 % high (item 47). A `low` margin.
    pub const STOP_RULE_CD_MAX: f64 = 0.010;
    /// Grid: the Coarse and Fine tiers (64 fin chord cells) are within 0.8 % of the grid-converged value (item 47, G-S). Random, both sides.
    pub const GRID_CD: f64 = 0.008;
    /// Grid, Draft tier (32 chord cells, wall functions): the fin leading edge is under-resolved, the drag 2.8-6.8 % *low* (item 47). A `high` margin.
    pub const GRID_CD_DRAFT_HIGH: f64 = 0.068;
    /// Wall temperature (item 45): the tiers run adiabatic walls; a wall at the free-stream temperature raises the skin friction x1.16 / x1.35 at Mach 1.8 / 3.1 and the
    /// CD0 3-6 %. A flying rocket is between the two: a `high` margin from Mach 1.5, ramping up from Mach 1.
    pub const WALL_TEMPERATURE_CD_MAX: f64 = 0.06;
    /// Reference scatter: the free-flight CD0 has a 1 % standard error (item 46) and a 2 degree cant of the flown fins; random, both sides.
    pub const REFERENCE_SCATTER_CD: f64 = 0.010;

    /// Subsonic centre of pressure: the model's x_cp is 0.12 caliber *aft* of the data (item 46): the stability margin is overstated. A `low` margin of the margin.
    pub const XCP_SUBSONIC_CAL: f64 = 0.12;
    /// Supersonic centre of pressure: within 0.25 caliber of the data everywhere (HANDOFF3D section 0, G-L1); the sign is not known, both sides.
    pub const XCP_SUPERSONIC_CAL: f64 = 0.25;
    /// CN_alpha, subsonic: +11 % against free flight (+13 % against the tunnel; data scatter 14-17 %, item 46, 51). The normal force is overstated: a `low` margin.
    pub const CN_ALPHA_SUBSONIC: f64 = 0.11;
    /// CN_alpha, supersonic: +3 to +10 % (G-L1). A `low` margin.
    pub const CN_ALPHA_SUPERSONIC: f64 = 0.10;
    /// The Mach number below which the subsonic stability terms apply (the free-flight comparison is Mach 0.79; 0.95 keeps the transonic points on the wider side).
    pub const SUBSONIC_BELOW: f64 = 0.95;
}

/// How far the wall-temperature term has come in at `mach`: 0 up to Mach 1, 1 from Mach 1.5 (the measured points are Mach 1.8 and 3.1), linear between.
fn wall_temperature_weight(mach: f64) -> f64 {
    ((mach - 1.0) / 0.5).clamp(0.0, 1.0)
}

/// The band of a zero-incidence drag coefficient `cd` at `mach` on the grid of `resolution`, absolute (the table above, [`Band::low`] below the value, [`Band::high`]
/// above it).
pub fn cd_band(cd: f64, mach: f64, resolution: Resolution) -> Band {
    use terms::*;
    let draft = resolution == Resolution::Draft;
    let random = (GRID_CD * GRID_CD * f64::from(u8::from(!draft)) + REFERENCE_SCATTER_CD * REFERENCE_SCATTER_CD).sqrt();
    let low = MODEL_BIAS_CD_MAX + STOP_RULE_CD_MAX + random;
    let high = WALL_TEMPERATURE_CD_MAX * wall_temperature_weight(mach) + if draft { GRID_CD_DRAFT_HIGH } else { 0.0 } + random;
    Band { low: low * cd.abs(), high: high * cd.abs() }
}

/// The band of the centre of pressure at `mach` in calibers of the reference diameter: `low` is how far *forward* of the solved x_cp the true one may be (the stability
/// margin falls by that much), `high` how far aft.
pub fn x_cp_band_cal(mach: f64) -> Band {
    use terms::*;
    if mach < SUBSONIC_BELOW {
        Band { low: XCP_SUBSONIC_CAL, high: 0.0 }
    } else {
        Band { low: XCP_SUPERSONIC_CAL, high: XCP_SUPERSONIC_CAL }
    }
}

/// How much of the solved CN_alpha may be too much at `mach`, as a fraction of it: the true normal-force slope is within `[CN_alpha (1 - this), CN_alpha]`.
pub fn cn_alpha_overstated(mach: f64) -> f64 {
    if mach < terms::SUBSONIC_BELOW {
        terms::CN_ALPHA_SUBSONIC
    } else {
        terms::CN_ALPHA_SUPERSONIC
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_drag_band_is_widest_below_the_value_and_grows_with_the_wall_temperature_above_mach_one() {
        let sub = cd_band(0.5, 0.5, Resolution::Coarse);
        let sup = cd_band(0.5, 2.0, Resolution::Coarse);
        assert!(sub.low > sub.high && sub.low > 0.5 * (0.067 + 0.01));
        assert!(sup.high > sub.high + 0.5 * 0.05);
        assert_eq!(wall_temperature_weight(0.9), 0.0);
        assert_eq!(wall_temperature_weight(1.25), 0.5);
        // the drag band is proportional to the value; the draft grid adds its under-resolution above
        assert!((cd_band(1.0, 2.0, Resolution::Coarse).low - 2.0 * sup.low).abs() < 1e-12);
        assert!(cd_band(0.5, 0.5, Resolution::Draft).high > sub.high + 0.5 * 0.06);
    }

    #[test]
    fn the_centre_of_pressure_band_puts_the_subsonic_data_forward() {
        assert_eq!(x_cp_band_cal(0.5), Band { low: 0.12, high: 0.0 });
        assert_eq!(x_cp_band_cal(2.0), Band { low: 0.25, high: 0.25 });
        assert!(cn_alpha_overstated(0.5) > cn_alpha_overstated(2.0));
    }
}
