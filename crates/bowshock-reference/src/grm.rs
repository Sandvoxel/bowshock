//! Martin Generalized Research Model (GRM), body B4 alone and with tail fin F13, tested in the AEDC 4-foot transonic tunnel (4T)
//! as Test No. 1 of G. R. Gomillion, "A Compilation of Static Stability and Fin Loads Data for Slender Body Missile Models With and
//! Without Tail Fins and Wings", AEDC-TR-75-125 (ARO Inc., March 1976, approved for public release). Archive.org copies:
//! volume 1 (text, nomenclature, Figs. 1-10, Tables 1-7) <https://archive.org/details/DTIC_ADB010053> (DTIC ADB010053), volume 2
//! (Appendix A, the tabulated Test 1 data) <https://archive.org/details/DTIC_ADB010054> (DTIC ADB010054).
//!
//! # Where each number comes from
//! * Geometry: volume 1 Fig. 3 (body B4), Fig. 6 and Table 1 (fin F13), section 2.2 (fin mounting), Fig. 1 and the photograph
//!   Fig. 2(a) (installation). Fig. 6 gives the dimensions of the original model drawings, which the report says were used in the
//!   data reduction. Table 1 gives the measured dimensions of the four F13 fins (average b = 2.8111, CR = 3.7478, CT = 1.8815,
//!   X = 2.0527 in.), all within 0.5 % of the drawing values (largest: CT +0.45 %, X -0.4 %).
//! * Aerodynamic coefficients: **table rows**, not figure readings, from volume 2 Appendix A (PDF pp. 19-28 for B4 + F13, parts
//!   22, 24, 30-37, and pp. 123-129 for the body alone, parts 196-202). The OCR text layer of the scan is unreliable (for
//!   example a printed 0.7434 comes out as "0.7*3" followed by a stray character), so every row used here was read on the rendered page image, and every moment `CLM` was
//!   cross-checked against the printed `XCP * CN`. The tables are 4-digit numbers; the uncertainty of this module is therefore the
//!   fit and repeat scatter given below, not the reading.
//!
//! # Definitions in the report (volume 1 Nomenclature, Fig. 10)
//! * `CN = normal force / (q S)` and `CA = axial force / (q S)`, with `S` the *maximum body cross-sectional area*,
//!   `pi d^2 / 4`, `d` = 3.750 in. The pitching moment is `CLM = moment / (q S d)`, positive nose-up (Fig. 10), about the moment
//!   reference point (MRP) on the centreline at 50 % of the model body length. These are the project conventions (area `pi d^2/4`,
//!   calibers); the `CN_alpha` here is per radian.
//! * `XCP = CLM / CN` "missile body centre-of-pressure location" in calibers from the MRP. With CLM positive nose-up and CN
//!   positive upward, a centre of pressure *behind* the MRP gives CLM < 0, hence a negative XCP: XCP is **negative aft**. The data
//!   confirm this: B4 alone has XCP > 0 (unstable, CP ahead of the MRP) and B4 + F13 has XCP about -2.5 (CP aft of the MRP, fins).
//!   Therefore `x_cp(from nose, calibers) = MRP_CAL - XCP = 4.75 - XCP`. Both tables below are already converted that way.
//! * `CAB = (p_inf - p_b) / q_inf`: base axial-force coefficient, positive when the base pressure `p_b` is below free-stream
//!   static pressure (a drag). `p_b` is the average of two static-pressure taps at the model base (section 3.2). The base
//!   pressure force is thus referred to free-stream pressure over the full base area `S`. `CAF = CA - CAB`: "forebody
//!   axial-force coefficient", i.e. the total axial force with the base pressure force removed, which includes pressure and
//!   friction drag of the body and of the fins, and (inferred, the report does not say) any force on the sting side of the
//!   base area. [INFERENCE] whether `S` also covers the sting cross-section: the report does not state it. `CA`, `CAB` and `CAF`
//!   are printed per row and satisfy `CA - CAB = CAF` to the printed digits in the rows checked, except M = 1.30 fins on, alpha = 0 (0.002 off).
//!
//! # Test conditions
//! AEDC 4T tunnel (perforated walls, 0.5-10 % porosity, 4 ft square), December 1970, Mach 0.80-1.30, `Re` = 2.3e6 per ft, roll
//! angle 0, fin deflection 0 on all four fins, alpha from -3 to 32 deg. **Transition: the report states "the details of fixing
//! transition on this test are unknown" (section 3.2; Table 7 column TRANSITION = "Unknown"); no grit strip is documented for
//! Test 1, unlike Tests 2-4, which used No. 60 / 40 grit.** Wall-interference corrections are not mentioned for Test 1. Flow-angularity
//! correction is stated for Tests 3 and 4 but not for Test 1, which agrees with the nonzero `CN` and `CLM` at alpha = 0 seen below.
//! Stagnation temperature is not given in volume 1.
//!
//! # Sting
//! The model is sting-mounted from the base (a pitch sector and boom carry it, pitch centre at tunnel station 108, Fig. 1;
//! the MRP is at that station). The sting diameter and length are **not dimensioned** in volume 1. On the photograph Fig. 2(a) the
//! sting looks like roughly 0.4 d in diameter with about 1 d visible behind the base before it bends into the boom; this is a
//! rough impression from a 110 dpi photograph, not a measurement. The tail fins are mounted in the plane of the model base (section
//! 2.2: "the trailing edge of each fin was in the plane of the model base"), four fins in a "plus" cruciform arrangement.
//!
//! # Uncertainty
//! The report quotes for Test 1 (Table 2, maximum uncertainties from Ref. 1): `dCN` = +-0.067, `dCLM` = +-0.078, `dCA` = +-0.014,
//! `d alpha` = +-0.1 deg, `dM` = +-0.005 (M <= 0.95) or +-0.010 (0.95 < M <= 1.3). These are maxima of the (balance) uncertainty; the
//! scatter of the repeated points in the tables is much smaller for `CN` and about the same for `CAF`:
//! * `CN` at alpha = -0.01 (fins on, M = 0.80, five repeats): 0.0517-0.0645; at alpha = 1.99 (M = 0.85, three repeats): 0.3520-0.3678;
//!   at alpha = 3.96 (M = 0.98 and 1.10, two repeats each): differences 0.017 and 0.031. Hence the fitted fins-on `CN_alpha` (3 points,
//!   alpha = 0, 2, 4 deg) carries about +-0.3 per radian (3 %) of random error, and the body alone (repeat differences 0.001-0.004)
//!   about +-0.1 per radian.
//! * `CAF` at alpha = 0 with fins on: M = 0.80 repeats 0.1334-0.1467; M = 1.05 repeats 0.3402-0.3909 (here `CA` is constant, 0.544-0.549, while
//!   `CAB` drifts from 0.153 to 0.209: the base pressure measurement is what scatters). The `dCA` = 0.014 of Table 2 is the right size
//!   to quote everywhere, and the transonic points M = 0.92-1.10 are best treated as +-0.02.
//! * `x_cp`: the three ways of getting it from the data disagree by 0.1-0.2 caliber with fins on and by up to 0.5 caliber for the
//!   body alone at M >= 1.1 (see [`FINS_ON_DETAIL`] and [`BODY_ALONE_DETAIL`]). The pitching moment has a nonzero value at alpha = 0
//!   (up to 0.25 with fins on at M = 0.98, and +0.09 body alone at M = 1.2), larger than the `CN` offset suggests.
//!
//! # How the tabulated values were derived
//! Rows of the same alpha (+-0.15 deg) were averaged first, so repeats do not weight the fit. Then:
//! * `cn_alpha` (both tables): least-squares slope of `CN` over alpha, **free intercept**, over the distinct alpha groups in
//!   -0.1 <= alpha <= 4.15 deg (rows near 0, 2 and 4 deg; for the body alone also the rows near 1 and 3 deg: 5 groups; fins on: 3
//!   groups, because the fins-on table has only the rows at about 0, 2 and 4 deg), times 180/pi. The secant `(CN(3.96) - CN(0)) / alpha`
//!   with the same two rows agrees with the fit to 0.002 per radian in every case (e.g. 9.848 vs 9.847 at M = 0.80), because of the
//!   3-point structure; it is not tabulated. The intercept `CN0` of the fit is in the detail tables.
//! * `x_cp_cal`: `4.75 - (dCLM/dalpha) / (dCN/dalpha)`, both slopes by the same fit over the same alpha rows. This is the
//!   *incremental* centre of pressure, free of the alpha = 0 offsets of `CN` and `CLM`, which is the quantity to compare with a
//!   symmetric CFD solution at alpha = 2-4 deg. The report's own `XCP` column (`CLM/CN` at alpha near 4 deg, including the offsets)
//!   and the ratio of the moment to force slopes over alpha = 2-6 are in the detail tables; they differ by up to 0.2 caliber with fins on.
//! * `ca_forebody`: mean `CAF` of the rows at alpha = 0 (|alpha| < 0.05 deg): with fins on, 5 rows at M = 0.80, 4 at M = 1.05, 2 at
//!   M = 0.92, 1 at the others. For the body alone the rows with |alpha| <= 1.05 deg (2-4 rows), because `CA` changes by <= 0.002
//!   across them (checked on parts 196, 197, 198). Rounded to 0.001. For M = 1.30 fins on, the printed `CAF` 0.3366 is used; `CA` (0.5350) minus
//!   `CAB` (0.1964) would be 0.3386.

const IN: f64 = 0.0254;

/// Body diameter, 3.750 in (Fig. 3, B4).
pub const BODY_DIAMETER: f64 = 3.750 * IN;
/// Body nose length, 9.375 in = 2.5 d (Fig. 3, B4): tangent ogive, generating radius 24.376 in = 6.5 d. The model nose is a
/// tangent-ogive cylinder.
pub const NOSE_LENGTH_CAL: f64 = 9.375 / 3.750;
/// Tangent-ogive generating radius, 24.376 in (Fig. 3) in calibers; `(L^2 + r^2) / (2 r)` with `L = 2.5 d`, `r = 0.5 d` gives 6.5.
pub const OGIVE_RADIUS_CAL: f64 = 24.376 / 3.750;
/// Total body length, 35.625 in = 9.5 d (Fig. 3, B4). The base is flat; the fin is not part of the length.
pub const TOTAL_LENGTH_CAL: f64 = 35.625 / 3.750;
/// Moment reference point from the nose, calibers: 50 % of the model length, 17.813 in (Fig. 3, mark on B4; section 3.1).
pub const MOMENT_REFERENCE_CAL: f64 = 0.5 * TOTAL_LENGTH_CAL;

/// Number of tail fins: four, in a "plus" cruciform pattern, at roll angle 0 (section 2.2).
pub const FIN_COUNT: usize = 4;
/// Fin F13 root chord, 3.747 in (Fig. 6; measured average 3.7478 in, Table 1), in calibers. Fin root leading edge on the body at
/// `(35.625 - 3.747) / 3.75` = 8.501 d from the nose: the fin trailing edge is in the plane of the base.
pub const FIN_ROOT_CHORD_CAL: f64 = 3.747 / 3.750;
/// Fin F13 tip chord, 1.873 in (Fig. 6; measured average 1.8815 in), in calibers.
pub const FIN_TIP_CHORD_CAL: f64 = 1.873 / 3.750;
/// Fin F13 span, 2.812 in = 0.75 d (Fig. 6, "measured from the root chord to the tip"; measured average 2.8111 in), in calibers.
/// The root chord is at the body surface: AR = b^2 / S_F = 1.0 holds with b = 2.812 and the planform area `S_F` = 0.05488 ft^2
/// = 7.903 in^2 = 0.5 (CR + CT) b, so the planform is the exposed trapezoid; the "0.015" dimension next to the fin root in the Fig. 6
/// side view is the gap between the fin and the body (read from the sketch, not stated in the text).
pub const FIN_SPAN_CAL: f64 = 2.812 / 3.750;
/// Fin F13 taper ratio `CT / CR` = 0.5 (Fig. 6).
pub const FIN_TAPER: f64 = 0.5;
/// Fin F13 leading-edge sweep angle, 33 deg 41 min (Fig. 6), in degrees. The trailing edge is perpendicular to the body axis:
/// the tip leading edge is `b tan(33deg41') = 1.875 in` = 0.5 d behind the root leading edge, equal to `CR - CT` = 1.874 in.
pub const FIN_LE_SWEEP_DEG: f64 = 33.0 + 41.0 / 60.0;
/// Fin F13 planform area `S_F` = 0.05488 ft^2 (Fig. 6), divided by the body reference area `S`: 0.7155.
pub const FIN_PLANFORM_OVER_S: f64 = 0.05488 * 144.0 / (core::f64::consts::PI * 3.750 * 3.750 / 4.0);
/// Fin F13 root thickness `t_R` = 0.187 in (Fig. 6; Nomenclature: "tail fin root thickness") in calibers, i.e. a root
/// thickness-to-chord ratio of 0.050. The thickness at the tip is not dimensioned in the report (the side view of Fig. 6 shows
/// the blade narrowing outboard).
pub const FIN_ROOT_THICKNESS_CAL: f64 = 0.187 / 3.750;
/// Fin section, Fig. 6 and section 2.2: "modified double wedge", leading- and trailing-edge semivertex angle 4 deg, edge radii
/// 0.015 in (Fig. 6), a flat between the two bevels. Streamwise bevel lengths: leading edge `A` = 1.158 in (0.309 d), trailing
/// edge `B` = 1.140 in (0.304 d); fin hinge line at `X` = 2.061 in (0.550 d) from the root leading edge.
pub const FIN_EDGE_HALF_ANGLE_DEG: f64 = 4.0;
/// Fin leading and trailing edge radius, 0.015 in (Fig. 6), in calibers.
pub const FIN_EDGE_RADIUS_CAL: f64 = 0.015 / 3.750;
/// Fin leading-edge streamwise bevel length `A` = 1.158 in (Fig. 6), in calibers.
pub const FIN_LE_BEVEL_CAL: f64 = 1.158 / 3.750;
/// Fin trailing-edge streamwise bevel length `B` = 1.140 in (Fig. 6), in calibers.
pub const FIN_TE_BEVEL_CAL: f64 = 1.140 / 3.750;

/// Reynolds number per metre of Test 1: 2.3e6 per foot (section 3.2; every row of Appendix A: "RX10-6 2.3"). Based on `d`,
/// 7.2e5; based on the body length, 6.8e6. Transition unfixed/unknown, see the module docs.
pub const RE_PER_M: f64 = 2.3e6 / 0.3048;

/// Maximum normal-force coefficient uncertainty of Test 1, Table 2.
pub const D_CN: f64 = 0.067;
/// Maximum pitching-moment coefficient uncertainty of Test 1, Table 2.
pub const D_CLM: f64 = 0.078;
/// Maximum axial-force coefficient uncertainty of Test 1, Table 2.
pub const D_CA: f64 = 0.014;

/// `(M, cn_alpha per rad on pi d^2/4, x_cp in calibers from the nose, C_A forebody)` for body B4 with fin F13, roll 0, fin
/// deflection 0. Appendix A parts 22 (M 0.80), 24 (0.85), 30 (0.92), 31 (0.94), 32 (0.96), 33 (0.98), 34 (1.05), 35 (1.10),
/// 36 (1.20), 37 (1.30). Derivation in the module docs. Uncertainty: `cn_alpha` +-0.3 (random) ; `x_cp` +-0.15 caliber
/// (method-dependent, see [`FINS_ON_DETAIL`]); `ca_forebody` +-0.014 (Table 2), +-0.02 for M 0.92-1.10.
/// Note that CN is not linear: the slope over alpha = 2-6 deg is 0 to 1.5 per radian larger (the `cn_alpha_2_6` column of the detail table).
pub const FINS_ON: [(f64, f64, f64, f64); 10] = [
    (0.80, 9.85, 7.17, 0.139),
    (0.85, 9.85, 7.17, 0.144),
    (0.92, 9.33, 7.09, 0.190),
    (0.94, 10.35, 7.30, 0.205),
    (0.96, 10.88, 7.42, 0.240),
    (0.98, 10.94, 7.39, 0.254),
    (1.05, 11.39, 7.43, 0.360),
    (1.10, 11.18, 7.42, 0.371),
    (1.20, 11.10, 7.38, 0.390),
    (1.30, 10.31, 7.17, 0.337),
];

/// Alternative fins-on quantities, `(M, cn0, cn_alpha_2_6, x_cp_from_report_xcp_at_4deg, x_cp_from_slopes_2_6, ca_total_0, cab_0)`:
/// * `cn0`: intercept of the free-intercept fit of the `cn_alpha` of [`FINS_ON`], i.e. the `CN` offset at alpha = 0 (the rows at
///   alpha = -0.01 give 0.0211-0.0645, except M = 1.20 and 1.30 which have -0.0009 and -0.0253);
/// * `cn_alpha_2_6`: LSQ slope of `CN` over the alpha groups 2, 4, 6 deg (per radian);
/// * `x_cp_from_report_xcp_at_4deg`: `4.75 - XCP` with the report's `XCP = CLM/CN` of the row at alpha 3.95-3.98 (average of repeats);
/// * `x_cp_from_slopes_2_6`: `4.75 - (dCLM/dalpha)/(dCN/dalpha)` over alpha = 2, 4, 6 deg;
/// * `ca_total_0` and `cab_0`: mean `CA` and `CAB` of the alpha = 0 rows (`ca_forebody` of [`FINS_ON`] equals `ca_total_0 - cab_0`).
pub const FINS_ON_DETAIL: [(f64, f64, f64, f64, f64, f64, f64); 10] = [
    (0.80, 0.056, 10.66, 7.27, 7.35, 0.246, 0.106),
    (0.85, 0.032, 11.02, 7.32, 7.40, 0.249, 0.105),
    (0.92, 0.017, 10.82, 7.22, 7.38, 0.292, 0.102),
    (0.94, 0.021, 11.33, 7.43, 7.50, 0.314, 0.109),
    (0.96, 0.020, 11.75, 7.57, 7.56, 0.349, 0.109),
    (0.98, 0.038, 11.81, 7.56, 7.52, 0.394, 0.140),
    (1.05, 0.020, 11.99, 7.57, 7.52, 0.547, 0.187),
    (1.10, 0.020, 11.46, 7.54, 7.41, 0.578, 0.208),
    (1.20, -0.000, 11.09, 7.45, 7.35, 0.569, 0.179),
    (1.30, -0.029, 10.45, 7.19, 7.15, 0.535, 0.196),
];

/// `(M, cn_alpha per rad on pi d^2/4, x_cp in calibers from the nose, C_A forebody)` for body B4 alone (no fins), roll 0.
/// Appendix A parts 196 (M 0.80), 197 (0.85), 198 (0.92), 199 (0.98), 200 (1.10), 201 (1.20), 202 (1.30); no data at 0.94, 0.96,
/// 1.05. `cn_alpha` is the free-intercept slope over alpha = 0-4 deg (5 groups; M = 0.85 has no row near 1 deg: 4 groups).
/// Uncertainty: `cn_alpha` +-0.1; `x_cp` +-0.1 caliber at M <= 0.98, +-0.3 caliber at M >= 1.10 (alpha = 0 moment offset, see
/// [`BODY_ALONE_DETAIL`]); `ca_forebody` +-0.014 (Table 2) from the rows with |alpha| <= 1.05 deg. B4 alone is statically
/// unstable about the MRP; the numbers give the centre of pressure measured from the nose.
pub const BODY_ALONE: [(f64, f64, f64, f64); 7] = [
    (0.80, 2.59, 2.77, 0.139),
    (0.85, 2.56, 2.88, 0.127),
    (0.92, 2.61, 2.85, 0.122),
    (0.98, 2.73, 2.92, 0.130),
    (1.10, 2.69, 2.54, 0.248),
    (1.20, 2.73, 2.45, 0.275),
    (1.30, 2.78, 2.26, 0.264),
];

/// Alternative body-alone quantities, same columns as [`FINS_ON_DETAIL`] (`cn_alpha_2_6` over the groups 2, 3, 4, 5, 6 deg; `x_cp` from the
/// `XCP` of the row at alpha = 4.04-4.12 deg). `cn0` is the intercept of the 0-4 deg fit; `ca_total_0` and `cab_0` the mean `CA` and `CAB`
/// of the |alpha| <= 1.05 rows. The spread of `x_cp` at M >= 1.10 (2.54 / 2.56 / 2.75 at M = 1.10; 2.45 / 1.94 / 2.68 at M = 1.20; 2.26 /
/// 1.97 / 2.52 at M = 1.30 for slope 0-4 / `XCP` at 4 deg / slopes 2-6) comes from a moment offset of CLM = +0.08 to +0.10 at alpha = 0 at M = 1.2 and 1.3.
pub const BODY_ALONE_DETAIL: [(f64, f64, f64, f64, f64, f64, f64); 7] = [
    (0.80, 0.001, 2.65, 2.75, 2.79, 0.242, 0.103),
    (0.85, 0.001, 2.75, 2.70, 2.94, 0.234, 0.107),
    (0.92, 0.002, 2.77, 2.81, 2.95, 0.232, 0.110),
    (0.98, 0.006, 2.90, 2.97, 3.01, 0.255, 0.125),
    (1.10, 0.008, 2.96, 2.56, 2.75, 0.426, 0.179),
    (1.20, -0.003, 2.98, 1.94, 2.68, 0.428, 0.154),
    (1.30, 0.007, 3.02, 1.97, 2.52, 0.410, 0.146),
];

#[cfg(test)]
mod tests {
    use super::*;

    fn strictly_increasing(machs: impl Iterator<Item = f64>) -> bool {
        let m: Vec<f64> = machs.collect();
        m.windows(2).all(|w| w[0] < w[1])
    }

    #[test]
    fn mach_ordering() {
        assert!(strictly_increasing(FINS_ON.iter().map(|r| r.0)));
        assert!(strictly_increasing(BODY_ALONE.iter().map(|r| r.0)));
        assert!(strictly_increasing(FINS_ON_DETAIL.iter().map(|r| r.0)));
        assert!(strictly_increasing(BODY_ALONE_DETAIL.iter().map(|r| r.0)));
        assert_eq!(FINS_ON.len(), FINS_ON_DETAIL.len());
        assert_eq!(BODY_ALONE.len(), BODY_ALONE_DETAIL.len());
    }

    #[test]
    fn body_machs_are_a_subset_of_fin_machs() {
        for b in BODY_ALONE {
            assert!(FINS_ON.iter().any(|f| (f.0 - b.0).abs() < 1e-9), "M = {}", b.0);
        }
    }

    #[test]
    fn geometry_is_consistent() {
        assert!((TOTAL_LENGTH_CAL - 9.5).abs() < 1e-12);
        assert!((MOMENT_REFERENCE_CAL - 4.75).abs() < 1e-12);
        assert!((NOSE_LENGTH_CAL - 2.5).abs() < 1e-12);
        // Tangent ogive: R = (L^2 + r^2) / (2 r) in calibers with L = 2.5, r = 0.5.
        let r = 0.5;
        assert!((OGIVE_RADIUS_CAL - (NOSE_LENGTH_CAL * NOSE_LENGTH_CAL + r * r) / (2.0 * r)).abs() < 1e-3);
        // Trailing edge perpendicular to the axis: root minus tip chord equals the span times tan(sweep) (1.874 vs 1.875 in).
        let off = FIN_ROOT_CHORD_CAL - FIN_TIP_CHORD_CAL;
        assert!((off - FIN_SPAN_CAL * FIN_LE_SWEEP_DEG.to_radians().tan()).abs() < 1e-3);
        assert!((FIN_TIP_CHORD_CAL / FIN_ROOT_CHORD_CAL - FIN_TAPER).abs() < 1e-3);
        // Fin planform area 0.5 (CR + CT) b in the report's 0.05488 ft^2, i.e. 0.7155 S.
        let s_over = 0.5 * (FIN_ROOT_CHORD_CAL + FIN_TIP_CHORD_CAL) * FIN_SPAN_CAL * 3.750 * 3.750 / (core::f64::consts::PI * 3.750 * 3.750 / 4.0);
        assert!((s_over - FIN_PLANFORM_OVER_S).abs() < 2e-3, "{s_over} vs {FIN_PLANFORM_OVER_S}");
    }

    #[test]
    fn data_are_in_range() {
        for (m, cna, xcp, caf) in FINS_ON {
            assert!((0.8..=1.3).contains(&m));
            assert!((8.5..=12.0).contains(&cna), "fins on CN_alpha {cna} at M {m}");
            // Fins sit at 8.5-9.5 d: the CP is behind the MRP (4.75 d) and ahead of the base.
            assert!((6.8..=8.0).contains(&xcp), "fins on x_cp {xcp} at M {m}");
            assert!((0.1..=0.45).contains(&caf), "fins on C_A,forebody {caf} at M {m}");
        }
        for (m, cna, xcp, caf) in BODY_ALONE {
            assert!((2.3..=3.2).contains(&cna), "body CN_alpha {cna} at M {m}");
            assert!((2.0..=3.2).contains(&xcp), "body x_cp {xcp} at M {m}");
            assert!((0.1..=0.3).contains(&caf), "body C_A,forebody {caf} at M {m}");
        }
    }

    #[test]
    fn fins_add_normal_force_and_move_cp_aft() {
        for b in BODY_ALONE {
            let f = FINS_ON.iter().find(|f| (f.0 - b.0).abs() < 1e-9).unwrap();
            assert!(f.1 > 3.0 * b.1, "M {}", b.0);
            assert!(f.2 > b.2 + 3.0, "M {}", b.0);
            assert!(f.3 >= b.3 - 0.02, "M {}", b.0);
        }
    }

    #[test]
    fn forebody_axial_force_rises_through_transonic() {
        // Fins on: the drag rise from M = 0.80 to 1.05 is well above the scatter (0.02).
        assert!(FINS_ON[6].3 - FINS_ON[0].3 > 0.15);
        assert!(BODY_ALONE[4].3 - BODY_ALONE[0].3 > 0.08);
    }
}
