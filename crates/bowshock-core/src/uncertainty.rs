//! The uncertainty bands of the 3D engine's results: measured terms in one table, no second sweep.
//!
//! Every number below is a measurement against free flight, a tunnel or a refined grid, taken with the production tiers (`crates/bowshock-cfd/DESIGN3D.md`, items
//! 45-47, 50, 52, 55, 57, 62 and 64; `HANDOFF3D.md` section 0). Two rockets were measured against free flight (the Basic Finner) and four more against tunnel data (the Arcas
//! short and long models, the Yuska sounding-rocket stage, the AEDC research model; item 55), so the bands are the spread of those comparisons, widened where the cases
//! disagree, not a statistical confidence interval: the model error is a function of Mach number and, for the drag, of what the body is made of (a boattail, the friction share).
//!
//! A [`Band`] is one-sided on each side: the true value is expected within `[value - low, value + high]`. Systematic terms (model error, stop rule, wall temperature)
//! add linearly on the side they push; the random terms (grid, reference scatter) add in quadrature to both sides. A model that reads *high* puts the truth below the value
//! (a `low` margin); one that reads *low* puts it above (a `high` margin). "+20 % / -5 %" below is "the model read up to 20 % above the data and up to 5 % below it".
//!
//! **Not in the bands:** launch lugs and rail buttons are not meshed (the table adds the build-up method's drag for them, `CfdPoint::other`); one lug changed the
//! apogee of a model-scale rocket by +16 % when it was measured. Fin cant, fin sets beyond the first, pods, and a rough or painted surface that is not described
//! to the solver are not in the bands either. The Yuska axial force (item 55 G-X5, +19..+34 % at every point, `FAIL KNOWN`: the data's base correction and the substitute fin
//! planform are the open questions) is outside the supersonic band; it is the one comparison that is.

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

/// What the drag band depends on in the body besides the Mach number and the grid.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct BodyFeatures {
    /// The body has a boattail (a cone frustum that narrows toward the base, `BodySpec::boattail`): the subsonic pressure drag of the base-wake solve over a boattail reads
    /// high (DESIGN3D items 62, 64).
    pub boattail: bool,
    /// Skin friction over the drag the solve gave at the point, 0 to 1 (0 when it is not known): a friction-dominated body is the other case the subsonic drag reads high on
    /// (item 55 G-X6).
    pub friction_share: f64,
}

impl BodyFeatures {
    /// Whether the subsonic drag of this body falls in the wider "reads high" class of [`terms::CD_HIGH_SUBSONIC_AFTERBODY`]: a boattail, or friction above
    /// [`terms::FRICTION_DOMINATED_SHARE`] of the drag.
    pub fn reads_high_subsonic(self) -> bool {
        self.boattail || self.friction_share >= terms::FRICTION_DOMINATED_SHARE
    }
}

/// THE TABLE: every term of every band, as a fraction of the value (drag, CN_alpha) or in calibers (centre of pressure). Each is a measured range, the larger magnitude taken.
pub mod terms {
    /// The Mach number below which the subsonic terms apply (the free-flight comparison is Mach 0.79; the Arcas, AEDC and Fox data run to 0.95).
    pub const SUBSONIC_BELOW: f64 = 0.95;
    /// The Mach number above which the supersonic terms apply: 0.95 to here is the transonic band, where the fins' drag rise is under-predicted (item 55, items 37/38).
    pub const SUPERSONIC_ABOVE: f64 = 1.2;

    /// Subsonic CD0 of a flat-based, fin-dominated body, model *high*: the Basic Finner against free flight is +4.6 % at Mach 0.766 (Coarse sector; +5.6 % on the grid-converged
    /// model), +9 % against the tunnel (items 46, 47); a smooth model with a plausible transition removes most of it (item 50), a painted, rail-buttoned rocket is the fully
    /// turbulent case, the conservative side. A `low` margin.
    pub const CD_HIGH_SUBSONIC_FLAT_BASE: f64 = 0.067;
    /// Subsonic CD of a body with a boattail or with a friction share of the drag above [`FRICTION_DOMINATED_SHARE`], model *high* by up to 20 %: the AEDC research model's axial
    /// force is +13.6 / +20.4 / +17.1 / +15.6 % at Mach 0.80 / 0.85 / 0.92 / 0.94 with 0.124 of its 0.158 friction (item 55 G-X6); the finless Arcas long model +16.0 / +16.7 % at Mach
    /// 0.6 / 0.8 (G-X4), of which friction is right to 2 % and the excess is the boattail pressure drag of the base-wake solve (item 62, +0.03..+0.045 S; item 64: the inviscid wake
    /// is not grid-converged at the Coarse tier). The free-flight total of a boattailed rocket reads +0.01..+0.02 S high at Mach 0.6, more toward 0.9 (item 62 f). A `low` margin.
    pub const CD_HIGH_SUBSONIC_AFTERBODY: f64 = 0.20;
    /// Friction over the drag above which a body counts as friction-dominated for the subsonic term (the AEDC model 0.79 at Mach 0.8, the finless Arcas long model 0.8; the Basic
    /// Finner's is 0.16 at Mach 1.33, item 55 b).
    pub const FRICTION_DOMINATED_SHARE: f64 = 0.6;
    /// Subsonic CD, model *low*: the Arcas long model with fins on is -4.0 % at Mach 0.9 (item 55 G-X4); nothing else is below the data. A `high` margin.
    pub const CD_LOW_SUBSONIC: f64 = 0.05;
    /// Transonic CD (Mach 0.95-1.2), model *high*: the AEDC model -7.1 / -6.5 % at Mach 1.05 / 1.10 against +6.3 % at 0.98 (item 55 G-X6), the Finner +1.8 % at Mach 1.056 (item 55 b).
    /// A `low` margin.
    pub const CD_HIGH_TRANSONIC: f64 = 0.06;
    /// Transonic CD, model *low* by up to 17 %: the Arcas long model with fins on is -13.5 / -17.1 / -16.8 / -12.3 % at Mach 0.95 / 1.0 / 1.1 / 1.2 (item 55 G-X4: the double-wedge fin's
    /// transonic increment is 30-50 % of the data's, as G-F4, item 37), the AEDC model -10.3 % at Mach 1.20. A `high` margin.
    pub const CD_LOW_TRANSONIC: f64 = 0.17;
    /// Supersonic CD, model *high* (the fully turbulent smooth model, +2.6 / +6.7 % of the Finner at Mach 1.85 / 2.741, items 46, 47). A `low` margin.
    pub const CD_HIGH_SUPERSONIC: f64 = 0.07;
    /// Supersonic CD, model *low*: the Finner is -4.4 % at Mach 2.364 and -2.7 % at 2.970 (item 46); the Arcas supersonic fin increment is under-predicted (G-F4 `KNOWN`, items 37, 55; the
    /// transonic end of it is -12.3 % at Mach 1.2). A `high` margin.
    pub const CD_LOW_SUPERSONIC: f64 = 0.10;
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

    /// Centre of pressure, both sides, every Mach number: the Finner is 0.12 caliber aft of the data subsonically and within 0.25 caliber supersonically (items 46, 51, G-L1), the
    /// Arcas supersonic within 0.26, the AEDC model within 0.25 (item 55 G-X1, G-X3), the Yuska stage -0.04..-0.32 growing with Mach number (G-X2). Outside it: the Arcas short model at
    /// Mach 0.6-1.2, whose data are of a model "almost neutral" at 0-4 degrees (+0.49..+0.88 caliber subsonic, `FAIL KNOWN`), and Yuska A at Mach 3.47 (-0.32). The margin's lower bound
    /// subtracts the whole band (conservative: it is not shifted by the known sign).
    pub const XCP_CAL: f64 = 0.3;
    /// Grid: the Lift tier's CN is not grid-converged under any sizing (4-7 % per refinement step, item 57); +-5 %. Both sides.
    pub const GRID_CN_ALPHA: f64 = 0.05;
    /// CN_alpha, subsonic, model *high* by up to 21 %: +11 % of the Finner against free flight (+13 % tunnel; the data's own scatter is 14-17 %, items 46, 51), +7.5..+21.4 % of the AEDC
    /// model at Mach 0.80-0.98 (item 55 G-X3). A `low` margin; the model is not below the data subsonically.
    pub const CN_ALPHA_HIGH_SUBSONIC: f64 = 0.21;
    /// CN_alpha, supersonic, model *high* by up to 13 %: the Finner +3..+10 % (G-L1), the Yuska stage +0..+13 % (item 55 G-X2), the Arcas +0.4..+5.3 %. A `low` margin.
    pub const CN_ALPHA_HIGH_SUPERSONIC: f64 = 0.13;
    /// CN_alpha, supersonic, model *low* by up to 1 % (the Yuska stage model A roll 45 at Mach 2.38: -0.7 %). A `high` margin.
    pub const CN_ALPHA_LOW_SUPERSONIC: f64 = 0.01;
}

/// How far the wall-temperature term has come in at `mach`: 0 up to Mach 1, 1 from Mach 1.5 (the measured points are Mach 1.8 and 3.1), linear between.
fn wall_temperature_weight(mach: f64) -> f64 {
    ((mach - 1.0) / 0.5).clamp(0.0, 1.0)
}

/// The model error of the zero-incidence drag at `mach`, as `(reads high by up to, reads low by up to)`, fractions of the value (the Mach-banded terms of [`terms`]).
fn cd_model_error(mach: f64, body: BodyFeatures) -> (f64, f64) {
    use terms::*;
    if mach < SUBSONIC_BELOW {
        (if body.reads_high_subsonic() { CD_HIGH_SUBSONIC_AFTERBODY } else { CD_HIGH_SUBSONIC_FLAT_BASE }, CD_LOW_SUBSONIC)
    } else if mach <= SUPERSONIC_ABOVE {
        (CD_HIGH_TRANSONIC, CD_LOW_TRANSONIC)
    } else {
        (CD_HIGH_SUPERSONIC, CD_LOW_SUPERSONIC)
    }
}

/// The band of a zero-incidence drag coefficient `cd` at `mach` on the grid of `resolution`, for a body with the features `body`, absolute (the table above, [`Band::low`] below
/// the value, [`Band::high`] above it).
pub fn cd_band(cd: f64, mach: f64, resolution: Resolution, body: BodyFeatures) -> Band {
    use terms::*;
    let draft = resolution == Resolution::Draft;
    let random = (GRID_CD * GRID_CD * f64::from(u8::from(!draft)) + REFERENCE_SCATTER_CD * REFERENCE_SCATTER_CD).sqrt();
    let (reads_high, reads_low) = cd_model_error(mach, body);
    let low = reads_high + STOP_RULE_CD_MAX + random;
    let high = reads_low + WALL_TEMPERATURE_CD_MAX * wall_temperature_weight(mach) + if draft { GRID_CD_DRAFT_HIGH } else { 0.0 } + random;
    Band { low: low * cd.abs(), high: high * cd.abs() }
}

/// The band of the centre of pressure in calibers of the reference diameter: `low` is how far *forward* of the solved x_cp the true one may be (the stability margin falls by
/// that much), `high` how far aft.
pub fn x_cp_band_cal() -> Band {
    Band { low: terms::XCP_CAL, high: terms::XCP_CAL }
}

/// The band of the normal-force slope CN_alpha at `mach`, as a fraction of the solved value: the true slope is within `[CN_alpha (1 - low), CN_alpha (1 + high)]`. The model bias
/// (high subsonic, mostly high supersonic) and the grid term of the Lift tier add linearly.
pub fn cn_alpha_band(mach: f64) -> Band {
    use terms::*;
    let (reads_high, reads_low) = if mach < SUBSONIC_BELOW { (CN_ALPHA_HIGH_SUBSONIC, 0.0) } else { (CN_ALPHA_HIGH_SUPERSONIC, CN_ALPHA_LOW_SUPERSONIC) };
    Band { low: reads_high + GRID_CN_ALPHA, high: reads_low + GRID_CN_ALPHA }
}

#[cfg(test)]
mod tests {
    use super::*;

    const FLAT: BodyFeatures = BodyFeatures { boattail: false, friction_share: 0.4 };

    #[test]
    fn the_drag_band_follows_the_mach_band_and_the_body() {
        let sub = cd_band(0.5, 0.5, Resolution::Coarse, FLAT);
        let boat = cd_band(0.5, 0.5, Resolution::Coarse, BodyFeatures { boattail: true, ..FLAT });
        let fric = cd_band(0.5, 0.5, Resolution::Coarse, BodyFeatures { boattail: false, friction_share: 0.79 });
        let trans = cd_band(0.5, 1.05, Resolution::Coarse, FLAT);
        let sup = cd_band(0.5, 2.0, Resolution::Coarse, FLAT);
        let random = (0.008f64 * 0.008 + 0.01 * 0.01).sqrt();
        // subsonic: up to +20 % high for a boattailed or a friction-dominated body, 6.7 % for a flat-based fin-dominated one; the high side is the 5 % of the low-reading model
        assert!((boat.low / 0.5 - (0.20 + 0.01 + random)).abs() < 1e-12 && boat == fric);
        assert!((sub.low / 0.5 - (0.067 + 0.01 + random)).abs() < 1e-12 && (sub.high / 0.5 - (0.05 + random)).abs() < 1e-12);
        // transonic: the model reads low by up to 17 %, with the wall-temperature ramp started at Mach 1.05
        assert!((trans.high / 0.5 - (0.17 + 0.06 * 0.1 + random)).abs() < 1e-12 && (trans.low / 0.5 - (0.06 + 0.01 + random)).abs() < 1e-12);
        // supersonic: +7 % / -10 % and the whole wall-temperature term
        assert!((sup.low / 0.5 - (0.07 + 0.01 + random)).abs() < 1e-12 && (sup.high / 0.5 - (0.10 + 0.06 + random)).abs() < 1e-12);
        // the draft grid adds its under-resolution above, and the 20 % class is a Mach band: it does not reach the transonic band
        assert!(cd_band(0.5, 0.5, Resolution::Draft, FLAT).high > sub.high + 0.5 * 0.06);
        assert_eq!(cd_band(0.5, 1.05, Resolution::Coarse, BodyFeatures { boattail: true, ..FLAT }), trans);
        assert_eq!(wall_temperature_weight(0.9), 0.0);
        assert_eq!(wall_temperature_weight(1.25), 0.5);
        // the drag band is proportional to the value
        assert!((cd_band(1.0, 2.0, Resolution::Coarse, FLAT).low - 2.0 * sup.low).abs() < 1e-12);
    }

    #[test]
    fn the_stability_bands_are_the_cross_case_spread() {
        assert_eq!(x_cp_band_cal(), Band { low: 0.3, high: 0.3 });
        let sub = cn_alpha_band(0.5);
        let sup = cn_alpha_band(2.0);
        assert!((sub.low - 0.26).abs() < 1e-12 && (sub.high - 0.05).abs() < 1e-12);
        assert!((sup.low - 0.18).abs() < 1e-12 && (sup.high - 0.06).abs() < 1e-12);
        assert!(sub.low > sup.low);
    }
}
