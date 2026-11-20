//! Static-stability data of the half-scale Arcas sounding-rocket models: NASA TN D-4013 (J. C. Ferris, 1967, Langley 8-ft
//! transonic pressure tunnel, Mach 0.60-1.20, <https://ntrs.nasa.gov/citations/19670020050>) and NASA TN D-4014 (C. D. Babb &
//! D. E. Fuller, 1967, Langley Unitary Plan tunnel, Mach 1.50-4.63, <https://ntrs.nasa.gov/citations/19670020031>).
//! The geometry of the short model (configuration 1, l/d = 18.20) and its axial-force table are in `arcas.rs`; this module holds
//! the normal-force slope, the centre of pressure, the long model (configuration 2, l/d = 23.77) and the test conditions.
//!
//! # Conventions
//! * `CN_alpha` is per RADIAN on the body cross-section `S = pi d^2 / 4` (both reports: `C_N = normal force / (q S)`, `S` = maximum
//!   body cross-section; the balance was in the model, so this is the whole-vehicle normal force).
//! * `x_cp` is given in CALIBERS (body diameters) from the NOSE TIP. Both reports give `x_cp` in "percent body length"; that it is
//!   measured from the nose is an inference (not stated), supported by the moment centres (70 % in D-4013, 63.37 % / 66.36 % in
//!   D-4014, both from the nose) lying forward of every plotted `x_cp` except the short model at Mach 0.8, where `x_cp` = 70.9 % and
//!   D-4013 states the model is "almost neutral" at 0.6, 0.8, 0.9. Conversion: `pct / 100 * (l/d)` with `l/d` = 18.20 (short) and
//!   23.77 (long; D-4013 text rounds this to 23.8, its Fig. 1 and D-4014 say 23.77, Fig. 1(a) of D-4014 gives 53.500 in / 2.250 in =
//!   23.78). `x_cp` does not depend on the moment reference centre.
//! * Roll angle 0 deg, fin cant 0 deg, fins on, unless a column says "fins off". D-4013 also has 22.5 / 45 deg roll and 2 deg
//!   cant; D-4014 has no roll angle (pitch plane only) and 2 deg cant; neither is tabulated here.
//!
//! # How the numbers were obtained
//! Neither report tabulates `C_N`; every number below is a FIGURE READING from a 400 dpi (D-4013) or 300 dpi (D-4014) render of the
//! NTRS scans, calibrated on the printed tick labels (D-4013 Fig. 7(b): 100 px per unit `C_N`, 50.9 px per degree; Fig. 11/12:
//! 19.2 / 18.0 px per percent; D-4014 Fig. 5: 138-141 px per unit `C_N`; Fig. 7: 19.2 px per percent). The reading procedure was a
//! zoomed crop with a ruler overlay; marker centres were taken as the centre of the square / circle / diamond outline (top edge plus
//! half the marker height where markers overlap) and as the mid-height of triangles. The `C_N` curves are non-linear (the slope
//! grows with angle of attack, the reports attribute this to body vortices), so `CN_alpha` is DEFINED here as the secant over
//! alpha in [0, 4] deg, `(C_N(4 deg) - C_N(0)) * 57.2958 / 4`, which is what a solver run at about 4 deg angle of attack would be
//! compared to; the slope at alpha -> 0 is lower (fins off: about half to two thirds of the secant). On every plot the curve was
//! faired through the test points.
//!
//! # Uncertainty
//! * The reports state a balance accuracy of +-0.03 in `C_N`; for the secant of two independent points this is
//!   +-0.03 * sqrt(2) / 4 * 57.2958 = +-0.61 per rad (balance, one standard of the quoted limit).
//! * Pixel reading: D-4013 Fig. 7(b) +-0.03 `C_N` at 4 deg where markers of the four roll angles overlap (the square is the
//!   `phi = 0` marker; +-0.05 at Mach 1.00 and 1.20 where the strands spread), and `C_N(0)` is taken as 0 (the markers at alpha = 0
//!   overlap the zero line within +-0.03). D-4014 Fig. 5: +-0.02 `C_N` per point (both ends read), but +-0.04 for Mach >= 2.96 where
//!   the triangle / half-disc markers are broken up by the dense grid, and for fins off Mach >= 2.96 the curve line was used
//!   instead of the markers. Pixel reading alone is +-0.3 to +-0.7 per rad.
//! * The combined (root-sum-square) uncertainty of the `CN_alpha` columns is about +-0.9 per rad (D-4013 and D-4014 Mach <= 2.30),
//!   +-1.2 per rad (D-4013 Mach 1.00 and 1.20, D-4014 fins on Mach >= 2.96), +-1.3 per rad (D-4014 fins off Mach >= 2.96).
//!   The Mach-to-Mach wiggle of D-4013 (13.6, 12.3, 12.7, 13.8 per rad before the jump to 17 near Mach 1) is within 1.5 sigma of
//!   a smooth curve and should not be over-interpreted.
//! * `x_cp`: +-0.5 percent of body length (+-0.09 cal short, +-0.12 cal long) where the curve is flat or gently sloped;
//!   +-1.5 percent (+-0.27 cal short, +-0.36 cal long) in the steep transonic rise at Mach 0.95 to 1.00 (short) and 1.1 to 1.2
//!   (long), where the Mach tolerance +-0.003 of D-4013 adds up to 0.3 percent for a slope of 100 percent per Mach.
//! * The long-model `C_A,corr` is read like the short-model one in `arcas.rs`: +-0.005 on flat parts, +-0.02 at Mach 0.90 to 1.00.
//!
//! # Where the Mach numbers are measurements and where they are curve values
//! D-4013 tested both models at Mach 0.60, 0.80, 0.90, 1.00, 1.20 and the short model also at 0.95 (text, p. 4). The `x_cp` and
//! `C_A,corr` summary curves (Figs. 11, 12) are faired across Mach, so the values at Mach 0.70 and 1.10 (both models) and 0.95
//! (long model) are readings of the fairing, not of a test point: use them as interpolation only. `CN_alpha` is NaN there. The
//! D-4014 Mach numbers (1.50, 1.80, 2.30, 2.96, 3.96, 4.63) are all test points; Mach accuracy +-0.015 up to 2.96 and +-0.05 above.

const IN: f64 = 0.0254;

/// Length-to-diameter ratio of the short model (configuration 1), 18.20 (D-4013 Fig. 1(a), D-4014 text).
pub const SHORT_L_OVER_D: f64 = 18.20;
/// Length-to-diameter ratio of the long model (configuration 2), 23.77 (D-4013 Fig. 1(a), D-4014 text; 23.8 in the D-4013 text).
pub const LONG_L_OVER_D: f64 = 23.77;

/// Moment reference centre of D-4013, 70 percent of body length from the nose for both models (SYMBOLS section; 12.75 d short,
/// 16.63 d long, Fig. 1(a)). Informational: `x_cp` is independent of it.
pub const TRANSONIC_MOMENT_CENTRE_FRACTION: f64 = 0.70;
/// Moment reference centres of D-4014, 63.37 percent (configuration 1, station 25.950 in) and 66.36 percent (configuration 2,
/// station 35.500 in) of body length from the nose (SYMBOLS section and Fig. 1(a)). Informational.
pub const SUPERSONIC_MOMENT_CENTRE_FRACTIONS: (f64, f64) = (0.6337, 0.6636);

// ----------------------------------------------------------------------------------------------------------------------------
// Long model geometry (configuration 2)
// ----------------------------------------------------------------------------------------------------------------------------

/// Cylinder length of the long model, 41.090 in: station 10.600 in (end of the nose) to 51.690 in (start of the boattail),
/// from D-4014 Fig. 1(a) where configuration 2 ends at station 53.500 in (135.890 cm) and the boattail is 1.810 in long, as on
/// the short model. The short model's cylinder is 28.540 in (`arcas::TUBE_LENGTH`); the difference 12.550 in is the whole
/// difference between the two models (53.500 - 40.950 in).
pub const LONG_TUBE_LENGTH: f64 = 41.090 * IN;
/// Total length of the long model, 53.500 in = 23.78 d (D-4014 Fig. 1(a); 23.77 d in both reports' text and D-4013 Fig. 1(a)).
pub const LONG_TOTAL_LENGTH: f64 = 53.500 * IN;
// Nothing else differs: the nose (10.600 in, same ordinates table), the 1.810 in boattail, the 1.470 in reflex-lip base, the fins
// (one "Fin details" drawing serves both configurations in D-4014 Fig. 1(b); D-4013 Fig. 1(b) "Details of forebody, tail cone, and
// fins" likewise) and the fin station relative to the base (fin leading edge 3.650 in ahead of the base: stations 37.300 in on
// configuration 1, 49.850 in on configuration 2) are common. Only the moment centre moves (see above). The fin anchors that plug
// the boattail slots were flush (balsa) on the long model without fins but not flush on the short model without fins (D-4013
// text, p. 4); this raises the short model's fins-off `C_A,corr` above the long one's for Mach >= 0.975.

// ----------------------------------------------------------------------------------------------------------------------------
// Test conditions
// ----------------------------------------------------------------------------------------------------------------------------

/// D-4013: Langley 8-foot transonic pressure tunnel (slotted walls), stagnation temperature 322 K (120 F) at all Mach numbers,
/// Reynolds number 9.8e6 per metre (3.0e6 per ft). Same constants as `arcas::TRANSONIC_T0_K`, `arcas::TRANSONIC_RE_PER_M`.
pub const TRANSONIC_TUNNEL: &str = "Langley 8-ft transonic pressure tunnel, T0 = 322 K, Re = 9.8e6 /m";
/// D-4013 transition strip: 0.10 in (0.25 cm) wide band of No. 120 carborundum grains 1.25 in (3.17 cm) aft of the nose tip and
/// 0.5 in (1.27 cm) aft of each fin leading edge.
pub const TRANSONIC_TRANSITION_STRIP: &str = "No. 120 carborundum, 0.10 in wide, 1.25 in aft of nose and 0.5 in aft of fin LE";
/// D-4013 support: six-component strain-gage balance inside the model, 1 in (2.54 cm) diameter sting extending 11.5 in (29.21 cm)
/// aft of the model base and ending in a 19.2 deg half-angle flare that joins the tunnel sting.
pub const TRANSONIC_STING: &str = "1 in diameter sting, 11.5 in aft of base, then 19.2 deg half-angle flare";
/// D-4014: Langley Unitary Plan wind tunnel (low Mach section for 1.50 to 2.96, high Mach section for 3.96 and 4.63), stagnation
/// temperature 339 K for Mach 1.50 to 2.96 and 352 K for 3.96 and 4.63 (test-condition table; the 4.63 row is garbled in the OCR
/// but shows 352 K), dewpoint -30 F (239 K), Reynolds number 9.84e6 per metre (3.0e6 per ft) at all Mach numbers.
pub const SUPERSONIC_TUNNEL: &str = "Langley Unitary Plan tunnel, T0 = 339 K (M <= 2.96) / 352 K (M >= 3.96), Re = 9.84e6 /m";
/// D-4014 transition strip: 1/16 in (0.159 cm) wide band of No. 60 carborundum grains 3/4 in (1.91 cm) aft of the nose and 1/2 in
/// (1.27 cm) aft of each fin leading edge.
pub const SUPERSONIC_TRANSITION_STRIP: &str = "No. 60 carborundum, 1/16 in wide, 0.75 in aft of nose and 0.5 in aft of fin LE";
/// D-4014 support: strain-gage balance in the model "rigidly fastened to a sting support"; sting dimensions are not given in the text.
pub const SUPERSONIC_STING: &str = "internal balance on a sting; dimensions not stated";

// ----------------------------------------------------------------------------------------------------------------------------
// Short model (configuration 1)
// ----------------------------------------------------------------------------------------------------------------------------

/// Short model, Mach 0.60 to 1.20: `(M, CN_alpha per rad fins on (phi = 0, fin cant 0), CN_alpha per rad fins off,
/// x_cp in calibers from the nose, fins on cant 0)`.
///
/// `CN_alpha` from D-4013 Fig. 7(b) (report p. 22, PDF p. 24), secant over [0, 4] deg with `C_N(0) = 0`; the figure readings were
/// `C_N(4 deg)` fins on = 0.95, 0.86, 0.885, 0.96, 1.20, 1.16 and fins off = 0.175, 0.16, 0.225, 0.255, 0.21, 0.22 at Mach 0.60,
/// 0.80, 0.90, 0.95, 1.00, 1.20 (and `C_N(2 deg)` fins on 0.44, 0.40, 0.405, 0.47, 0.63, 0.61, i.e. the fins-on curve is 8 to 15
/// percent steeper between 2 and 4 deg than between 0 and 2). `x_cp` from D-4013 Fig. 11 (report p. 60, PDF p. 62), solid line
/// (`delta_F` = 0, -2 <= alpha <= 2 deg), readings in percent body length 74.7, 72.4, 70.9, 71.7, 76.7, 83.7, 87.2, 85.0
/// multiplied by 0.1820 (the legend of Fig. 11: solid = 0 deg cant, dashed = 2 deg cant, dash-dot = fins off; only `C_A,corr` and
/// `C_A,b` have a fins-off curve, there is no fins-off `x_cp`). Rows at 0.70 and 1.10 are faired-curve values, `CN_alpha` NaN.
/// Uncertainties: see the module docs (`CN_alpha` +-0.9 per rad, +-1.2 at Mach 1.00 and 1.20; `x_cp` +-0.09 cal, +-0.27 cal at
/// Mach 0.95 and 1.00).
pub const SHORT_TRANSONIC: [(f64, f64, f64, f64); 8] = [
    (0.60, 13.6, 2.5, 74.7 * SHORT_L_OVER_D / 100.0),
    (0.70, f64::NAN, f64::NAN, 72.4 * SHORT_L_OVER_D / 100.0),
    (0.80, 12.3, 2.3, 70.9 * SHORT_L_OVER_D / 100.0),
    (0.90, 12.7, 3.2, 71.7 * SHORT_L_OVER_D / 100.0),
    (0.95, 13.8, 3.65, 76.7 * SHORT_L_OVER_D / 100.0),
    (1.00, 17.2, 3.0, 83.7 * SHORT_L_OVER_D / 100.0),
    (1.10, f64::NAN, f64::NAN, 87.2 * SHORT_L_OVER_D / 100.0),
    (1.20, 16.6, 3.15, 85.0 * SHORT_L_OVER_D / 100.0),
];

/// Short model, Mach 1.50 to 4.63: `(M, CN_alpha per rad fins on (fin cant 0), CN_alpha per rad fins off,
/// x_cp in calibers from the nose, fins on cant 0)`.
///
/// `CN_alpha` from D-4014 Fig. 5(b) (fins on, report p. 28, PDF p. 30) and Fig. 5(a) (fins off, report p. 25, PDF p. 27), secant
/// over the markers at alpha near 0 and 4 deg. In these figures the six Mach numbers are stacked with a one-unit `C_N` offset each
/// (each curve has its own printed "0" label; the unit is 140.6 px on p. 30 and 138.0 px on p. 27); the measured increments
/// `C_N(4) - C_N(0)` (marker centres) are fins on 0.967, 0.882, 0.708, 0.626, 0.537, 0.521 and fins off 0.172, 0.2005, 0.235, 0.209,
/// 0.254, 0.312 (the last three fins-off values are from the curve line rather than the markers, see module docs); `C_N(0)` is
/// about -0.04 at Mach 1.50 and about 0 at the others (balance zero), so it is included through the increment. At Mach >= 2.96
/// the alpha of the markers is shifted by up to 0.5 deg against the other Mach numbers (flow-angularity correction of the high
/// Mach section), which the increments account for only to the stated uncertainty. `x_cp` from D-4014 Fig. 7 (report p. 41, PDF
/// p. 43), solid line = configuration 1, readings 78.2, 76.0, 72.2, 67.0, 60.5, 56.5 percent body length times 0.1820. The solid and
/// dashed curves cross near Mach 2.5; the identification of the solid curve above the dashed one at Mach >= 2.9 rests on the
/// dash pattern (the dense grid breaks both lines). Uncertainty: `CN_alpha` +-0.9 per rad (+-1.2 fins on, +-1.3 fins off at
/// Mach >= 2.96), `x_cp` +-0.09 cal.
pub const SHORT_SUPERSONIC: [(f64, f64, f64, f64); 6] = [
    (1.50, 13.85, 2.46, 78.2 * SHORT_L_OVER_D / 100.0),
    (1.80, 12.63, 2.87, 76.0 * SHORT_L_OVER_D / 100.0),
    (2.30, 10.14, 3.37, 72.2 * SHORT_L_OVER_D / 100.0),
    (2.96, 8.97, 3.00, 67.0 * SHORT_L_OVER_D / 100.0),
    (3.96, 7.69, 3.64, 60.5 * SHORT_L_OVER_D / 100.0),
    (4.63, 7.46, 4.47, 56.5 * SHORT_L_OVER_D / 100.0),
];

// ----------------------------------------------------------------------------------------------------------------------------
// Long model (configuration 2)
// ----------------------------------------------------------------------------------------------------------------------------

/// Long model, Mach 0.60 to 1.20: `(M, x_cp in calibers from the nose with fins on cant 0, C_A,corr fins off,
/// C_A,corr fins on cant 0)`, alpha = 0, roll 0.
///
/// From D-4013 Fig. 12 (report p. 61, PDF p. 63; same legend as Fig. 11). `x_cp` readings in percent body length: 76.7, 75.9,
/// 75.6, 76.1, 74.0, 71.5, 74.2, 82.0 times 0.2377 (the last is the end of the faired curve at Mach 1.20, read 81.6 at Mach 1.195 and
/// extrapolated 0.4 over the last 0.005 of Mach; at Mach 0.60 the cant-0 and cant-2 curves coincide at 76.7 to 76.8). The fin-cant
/// effect on `x_cp` is small for this model (report text). `C_A,corr` is the axial coefficient with the base-annulus pressure force
/// removed, referenced to the body cross-section (as in `arcas::TRANSONIC` for the short model; the fins-off curve at Mach 0.60 to
/// 0.70 lies on the 0.25 grid line, hence read as 0.250). Rows at 0.70, 0.95 and 1.10 are faired-curve values (the long model was
/// not tested there). Uncertainty: `x_cp` +-0.12 cal, +-0.36 cal for Mach 1.10 and 1.20 (and 0.95 to 1.00); `C_A,corr` +-0.005,
/// +-0.02 at Mach 0.90 to 1.00.
pub const LONG_TRANSONIC: [(f64, f64, f64, f64); 8] = [
    (0.60, 76.7 * LONG_L_OVER_D / 100.0, 0.250, 0.347),
    (0.70, 75.9 * LONG_L_OVER_D / 100.0, 0.250, 0.355),
    (0.80, 75.6 * LONG_L_OVER_D / 100.0, 0.2525, 0.371),
    (0.90, 76.1 * LONG_L_OVER_D / 100.0, 0.272, 0.453),
    (0.95, 74.0 * LONG_L_OVER_D / 100.0, 0.299, 0.595),
    (1.00, 71.5 * LONG_L_OVER_D / 100.0, 0.368, 0.725),
    (1.10, 74.2 * LONG_L_OVER_D / 100.0, 0.408, 0.699),
    (1.20, 82.0 * LONG_L_OVER_D / 100.0, 0.423, 0.622),
];

/// Long model, Mach 1.50 to 4.63: `(M, x_cp in calibers from the nose, fins on cant 0)`.
///
/// From D-4014 Fig. 7 (report p. 41, PDF p. 43), dashed line = configuration 2; readings in percent body length 78.2, 77.3, 71.8,
/// 66.2, 59.5, 55.4 times 0.2377. Uncertainty +-0.7 percent (+-0.17 cal): the two curves lie within 1 percent of each other for
/// Mach 1.5 to 2.5 and the dashed line is broken by the grid. The long model's `x_cp` falls faster with Mach than the short one's
/// (report text). The long model's `CN_alpha` (D-4013 Fig. 9(b), D-4014 Fig. 6) was NOT extracted.
pub const LONG_SUPERSONIC: [(f64, f64); 6] = [
    (1.50, 78.2 * LONG_L_OVER_D / 100.0),
    (1.80, 77.3 * LONG_L_OVER_D / 100.0),
    (2.30, 71.8 * LONG_L_OVER_D / 100.0),
    (2.96, 66.2 * LONG_L_OVER_D / 100.0),
    (3.96, 59.5 * LONG_L_OVER_D / 100.0),
    (4.63, 55.4 * LONG_L_OVER_D / 100.0),
];

#[cfg(test)]
mod tests {
    use super::*;

    fn increasing<T>(v: &[T], key: impl Fn(&T) -> f64) -> bool {
        v.windows(2).all(|w| key(&w[1]) > key(&w[0]))
    }

    #[test]
    fn mach_is_strictly_increasing() {
        assert!(increasing(&SHORT_TRANSONIC, |r| r.0));
        assert!(increasing(&SHORT_SUPERSONIC, |r| r.0));
        assert!(increasing(&LONG_TRANSONIC, |r| r.0));
        assert!(increasing(&LONG_SUPERSONIC, |r| r.0));
        assert!(SHORT_TRANSONIC[7].0 < SHORT_SUPERSONIC[0].0);
    }

    #[test]
    fn long_model_geometry() {
        // nose 10.600 in + cylinder + boattail 1.810 in = total
        assert!((10.600 * IN + LONG_TUBE_LENGTH + 1.810 * IN - LONG_TOTAL_LENGTH).abs() < 1e-12);
        let d = 2.250 * IN;
        assert!((LONG_TOTAL_LENGTH / d - LONG_L_OVER_D).abs() < 0.02);
        // the two models differ only by the cylinder: 53.500 - 40.950 = 41.090 - 28.540 = 12.550 in
        assert!((LONG_TOTAL_LENGTH - 40.950 * IN - (LONG_TUBE_LENGTH - 28.540 * IN)).abs() < 1e-12);
    }

    #[test]
    fn cn_alpha_ranges() {
        for &(m, on, off, _) in SHORT_TRANSONIC.iter().chain(SHORT_SUPERSONIC.iter()) {
            if on.is_nan() {
                assert!(off.is_nan(), "NaN must be paired at M = {m}");
                continue;
            }
            // slender finned body at small angle: per radian on S, between 5 and 20; fins add lift
            assert!((5.0..20.0).contains(&on), "fins-on CN_alpha {on} at M = {m}");
            assert!((1.0..6.0).contains(&off), "fins-off CN_alpha {off} at M = {m}");
            assert!(on > 1.5 * off, "fins must dominate at M = {m}");
        }
    }

    #[test]
    fn cn_alpha_fins_on_decays_with_supersonic_mach() {
        assert!(SHORT_SUPERSONIC.windows(2).all(|w| w[1].1 < w[0].1));
        // the transonic peak near Mach 1 to 1.2 exceeds both the subsonic and the Mach 1.5 values
        let peak = SHORT_TRANSONIC.iter().filter(|r| !r.1.is_nan()).map(|r| r.1).fold(0.0, f64::max);
        assert!(peak > SHORT_SUPERSONIC[0].1 && peak > SHORT_TRANSONIC[0].1);
    }

    #[test]
    fn x_cp_ranges() {
        // short model: 70 to 90 percent in D-4013, 55 to 80 percent in D-4014
        for &(_, _, _, x) in &SHORT_TRANSONIC {
            assert!((0.70 * SHORT_L_OVER_D..0.90 * SHORT_L_OVER_D).contains(&x));
        }
        for &(_, _, _, x) in &SHORT_SUPERSONIC {
            assert!((0.55 * SHORT_L_OVER_D..0.80 * SHORT_L_OVER_D).contains(&x));
        }
        for &(_, x, _, _) in &LONG_TRANSONIC {
            assert!((0.70 * LONG_L_OVER_D..0.85 * LONG_L_OVER_D).contains(&x));
        }
        for &(_, x) in &LONG_SUPERSONIC {
            assert!((0.55 * LONG_L_OVER_D..0.80 * LONG_L_OVER_D).contains(&x));
        }
    }

    #[test]
    fn x_cp_supersonic_moves_forward() {
        assert!(SHORT_SUPERSONIC.windows(2).all(|w| w[1].3 < w[0].3));
        assert!(LONG_SUPERSONIC.windows(2).all(|w| w[1].1 < w[0].1));
        // D-4014 text: the movement is greater for the longer vehicle (in percent of body length)
        let short_drop = (SHORT_SUPERSONIC[0].3 - SHORT_SUPERSONIC[5].3) / SHORT_L_OVER_D;
        let long_drop = (LONG_SUPERSONIC[0].1 - LONG_SUPERSONIC[5].1) / LONG_L_OVER_D;
        assert!(long_drop > short_drop);
    }

    #[test]
    fn long_axial_force_sanity() {
        for &(m, _, off, on) in &LONG_TRANSONIC {
            assert!(on > off, "fins add drag at M = {m}");
            assert!((0.2..0.8).contains(&on) && (0.2..0.5).contains(&off));
        }
        // fins-on C_A,corr peaks near Mach 1.0 (D-4013 Fig. 12)
        let imax = LONG_TRANSONIC.iter().enumerate().max_by(|a, b| a.1 .3.total_cmp(&b.1 .3)).unwrap().0;
        assert!((LONG_TRANSONIC[imax].0 - 1.0).abs() < 0.06);
    }
}
