//! One-fifth-scale models of a sounding-rocket second stage with thick wedge fins, NASA TN D-3182 (J. A. Yuska, Lewis Research
//! Center, January 1966, "Static aerodynamic characteristics of a rocket vehicle with thick wedge fins and sweptback leading and
//! trailing edges", <https://ntrs.nasa.gov/citations/19660004864>). Free-stream Mach 1.79 to 3.47, Lewis 10 x 10 ft (Mach 2.00 to
//! 3.47) and 8 x 6 ft (Mach 1.79 to 2.08) supersonic tunnels, alpha = -3 to 10 deg, roll 0 and 45 deg. A power-law ("approximately
//! three-quarter power") nose, a 6.00 in cylinder with four canted thick wedge fins.
//!
//! Provenance. Geometry: Fig. 1 (report p. 7), nose table values are printed numbers; fin dimensions are printed dimension labels, and
//! the axial fin position (root trailing edge at the base) is a drawing reading (see [`FIN_ROOT_LE_FROM_BASE`]). Test conditions:
//! Table II (report p. 6), printed numbers. `C_N,alpha` and `x_cp/d`: Fig. 4 (report p. 12). Axial force: Fig. 5 (report p. 13). The
//! two figures were read from a 300 dpi render of the NTRS scan: every marker centre was located by matching a circle, square or
//! diamond outline template (outline sampled at three radii) in a +-7 px window around a hand-placed seed, then checked on an
//! overlay; axes were calibrated by linear regression on the gridline centres of each panel (residuals below 0.007 Mach, 0.011 in
//! `C_N,alpha`, 0.003 in `x_cp/d`, 0.0006 in `C_A`). The marker abscissae are not exactly the nominal Mach numbers (read 0.00 to 0.03
//! below nominal, e.g. 1.776, 1.876, 1.981, 2.074 against 1.79, 1.88, 1.98, 2.08 in the 8 x 6 ft tunnel), so each marker was snapped to
//! the nearest Table II Mach number. The coefficients below are the marker ordinates.
//!
//! Conventions as in the report: reference area is the body cross-section 0.1964 ft^2 (d = 0.5 ft, i.e. pi d^2/4 of the 6.00 in
//! model) for every coefficient; `C_N,alpha` is per radian at zero normal force (least-squares slope of the `C_N` vs alpha data for
//! alpha between -3 and 4 deg); `x_cp/d` is in calibers from the nose tip (station 0), derived from the moment about the model base and the
//! `C_N` slope, also at zero normal force. These already match this crate's conventions: no conversion was made.
//!
//! What the report says about accuracy (theory against its own data, not measurement error): calculations from the method of Pitts,
//! Nielsen and Kaattari (NACA TR 1307) with the wedge-effectiveness factor agree with the experiments within +9 / -7 percent in `C_N,alpha`
//! and +-1.5 percent of the body length in `x_cp`; NACA TR 1307 itself is quoted at +-10 percent and +-2 percent of body length. No
//! balance or tunnel uncertainty is given. The report notes `C_N` and `C_M` are not zero at zero measured angle of attack (flow
//! angularity combined with angle-of-attack error in the 10 x 10 ft tunnel, and in the 8 x 6 ft tunnel a consistent offset suspected to come
//! from flow angularity or from an error in the fin cant angle acting on the balance normal-force link); the slopes are
//! unaffected by such an offset. Stagnation temperature is not stated in the report (Table II gives total pressure, dynamic pressure
//! and Reynolds number per foot only); no transition strip or boundary-layer state is mentioned in the parts of the text read.

const IN: f64 = 0.0254;

/// Body diameter d = 6.00 in (0.5 ft, report symbol list and Fig. 1).
pub const BODY_DIAMETER: f64 = 6.00 * IN;
/// Reference area, 0.1964 ft^2 (report symbol list) = pi d^2 / 4 for d = 0.5 ft \[m^2\].
pub const REFERENCE_AREA: f64 = 0.1964 * 0.092_903_04;

/// Nose length, 25.9 in = 4.317 d (Fig. 1: station 0 is the nose tip; the radius is 3.000 in at station 25.9).
pub const NOSE_LENGTH: f64 = 25.9 * IN;
/// Length of model A, 65.9 in = 10.98 d (Fig. 1, Table I). Model C has the same length (Table I; the Fig. 1 caption and the
/// text of the Apparatus section are contradictory about whether C derives from A or from the shortened B: the Table I length 65.9 in is used).
pub const MODEL_A_LENGTH: f64 = 65.9 * IN;
/// Length of model B, 61.7 in = 10.28 d: model A with the 4.2 in cylindrical section A removed (Fig. 1; located by the drawing at
/// roughly station 27 to 31 in; its exact station is not needed because the fins and base are unchanged).
pub const MODEL_B_LENGTH: f64 = 61.7 * IN;
/// Length of model C, 65.9 in (Table I).
pub const MODEL_C_LENGTH: f64 = 65.9 * IN;

/// Nose ordinates, `(station from the nose tip, radius)` in inches, Fig. 1 table (printed). Divide by 6.00 for calibers of the
/// 6.00 in body diameter (radius 3.000 in = 0.5 d). The first printed row is station 1.143 in; the tip (0, 0) is not listed.
/// The cylinder begins at 25.9 in (printed radius 3.000 in) although 2.975 in is reached at 24.0 in.
pub const NOSE_PROFILE_IN: [(f64, f64); 17] = [
    (1.143, 0.306),
    (1.905, 0.449),
    (3.810, 0.754),
    (5.714, 1.023),
    (7.619, 1.278),
    (9.524, 1.523),
    (11.428, 1.759),
    (13.333, 1.986),
    (15.238, 2.204),
    (17.143, 2.414),
    (19.048, 2.614),
    (20.952, 2.793),
    (22.857, 2.927),
    (23.810, 2.970),
    (24.000, 2.975),
    (24.762, 2.994),
    (25.900, 3.000),
];

/// Power-law exponent n of `r/R = (x/L)^n` over the full nose length L = 25.9 in, least-squares on the 17 printed ordinates
/// (minimiser of the rms error; the minimax exponent is 0.619). This law is a poor description of the table: the maximum error is
/// 6.1 percent of R (0.18 in, the printed nose is fuller than the power law near x/L = 0.5 and thinner near x/L = 0.05; rms 3.5 percent of R). With n = 0.75 the maximum
/// error is 7.8 percent of R.
pub const NOSE_POWER_N_FULL: f64 = 0.659;
/// The printed ordinates are fitted far better by a three-quarter power law that reaches full radius at a station short of 25.9 in
/// and then follows the cylinder: `r/R = (x/L_p)^n` for `x < L_p`, with `L_p` = 23.1 in and n = 0.760 (rms 0.62 percent of R,
/// maximum error 1.6 percent of R = 0.05 in). With n = 0.75 fixed the best L_p is 23.2 in (rms 0.67 percent, max 1.3 percent of R).
/// This agrees with the report's "approximately three-quarter power nose". Use this pair for the geometry.
pub const NOSE_POWER_N: f64 = 0.760;
/// Station at which the fitted power-law nose of [`NOSE_POWER_N`] reaches full radius, 23.1 in (a fit, not a printed number).
pub const NOSE_POWER_LENGTH: f64 = 23.1 * IN;
// Other fits of the table over the full length 25.9 in (maximum error / rms, as fractions of R = 3.0 in; nothing printed in the
// report): tangent ogive 10.5 % / 6.1 %, von Karman 7.0 % / 4.2 %, cone 14.7 % / 11.1 %, r/R = 1-(1-x/L)^1.57 3.7 % / 1.7 %.

/// Fin root chord (on the body, streamwise), 9.15 in (Fig. 1, printed).
pub const FIN_ROOT_CHORD: f64 = 9.15 * IN;
/// Fin tip chord, 6.54 in (Fig. 1, printed; models A and B). The 0.57 in tip strip "section B" of Fig. 1 is the outer 0.57 in of
/// span, with chord 6.54 in at the outer edge.
pub const FIN_TIP_CHORD: f64 = 6.54 * IN;
/// Fin span, 4.95 in = 4.38 in + 0.57 in (Fig. 1, printed; models A and B), measured from the root chord on the body surface:
/// the area (9.15 + 6.54)/2 x 4.95 = 38.83 in^2, doubled, equals the stated planform area 77.66 in^2 exactly, so the 77.66 in^2 is the
/// area of TWO fins (a fin pair in a plane), not of all four, and the root chord lies at the body surface (radius 3.00 in).
pub const FIN_SPAN: f64 = 4.95 * IN;
/// Fin span of model C, 4.38 in (Fig. 1: section B, the 0.57 in tip strip, removed). Tip chord at the cut about 6.9 in (inferred:
/// the stated area 70.34 in^2 / 2 = 35.17 in^2 requires a mean chord 8.03 in, i.e. 6.91 in at the cut; straight-edge geometry gives 6.84 in; not printed).
pub const FIN_SPAN_MODEL_C: f64 = 4.38 * IN;
/// Leading-edge sweep from the span axis normal to the flow, 45 degrees (Fig. 1 label), so the tip leading edge is a span (4.95 in) aft
/// of the root leading edge.
pub const FIN_LE_SWEEP_DEG: f64 = 45.0;
/// Axial offset of the tip trailing edge aft of the root trailing edge, 2.34 in = 4.95 + 6.54 - 9.15 in
/// (derived from the printed chords and 45 degree sweep; the drawing agrees within about 0.1 in). The trailing edge leans aft with
/// span at about 25.3 degrees to the span axis, the same direction as the leading edge (the report's "sweptback leading and trailing
/// edges"): the tip trailing edge overhangs the base.
pub const FIN_TIP_TE_AFT_OF_ROOT_TE: f64 = 2.34 * IN;
/// Leading-edge fillet radius label "0.1R" in the Fig. 1 planform at the root leading-edge corner. Read as 0.1 in; the unit is not printed and
/// the label could instead denote a leading-edge radius of the wedge, which Fig. 1's section shows as sharp. Not used by the solver.
pub const FIN_LE_RADIUS_LABEL: f64 = 0.1 * IN;
/// Wedge included angle, 10 degrees (Fig. 1 section view, printed). Symmetric double wedge with flat faces from a sharp leading edge to a
/// blunt (flat) trailing edge. Thickness at the root trailing edge 9.15 x 2 tan 5 deg = 1.60 in (the drawing agrees, about 1.65 in); with planar faces the thickness
/// at the tip trailing edge is 1.14 in (inferred, assuming the 10 degrees is the streamwise included angle; not printed).
pub const FIN_WEDGE_INCLUDED_ANGLE_DEG: f64 = 10.0;
/// Number of fins, four, cruciform. Fin cant angle 0.8 degrees (Apparatus section, printed); the sense of the cant (all fins in the
/// same roll direction) is not stated.
pub const FIN_CANT_DEG: f64 = 0.8;
/// Distance of the root leading edge forward of the base station, 9.15 in (reading of Fig. 1: the root trailing edge is at the base
/// station 65.9 in within about 0.15 in, i.e. the 65.9 dimension line and the root trailing edge coincide in the drawing; the drawing scale is
/// 18.8 px/in at 200 dpi and the offset is below 3 px). It follows that the root leading edge is at station 56.75 in on A and C, 52.55 in on
/// B, the tip leading edge 4.95 in further aft, and the tip trailing edge at 68.24 in (A, C) i.e. 2.34 in behind the base. Ambiguity: the
/// drawing is a sketch, and the root trailing edge being exactly flush with the base is a reading, not a printed dimension.
pub const FIN_ROOT_LE_FROM_BASE: f64 = 9.15 * IN;

/// Test conditions, Table II (printed): `(M, total pressure [lb/ft^2 abs], dynamic pressure [lb/ft^2], Reynolds number per ft)`.
/// First six rows 10 x 10 ft tunnel, last four 8 x 6 ft tunnel. Stagnation temperature is not given.
pub const TEST_CONDITIONS: [(f64, f64, f64, f64); 10] = [
    (2.00, 1430.0, 511.0, 2.49e6),
    (2.09, 1492.0, 507.0, 2.48e6),
    (2.38, 1725.0, 482.0, 2.47e6),
    (2.78, 2545.0, 522.0, 2.34e6),
    (3.17, 3570.0, 529.0, 2.41e6),
    (3.47, 4695.0, 544.0, 2.47e6),
    (1.79, 3140.0, 1240.0, 5.03e6),
    (1.88, 3330.0, 1268.0, 5.0e6),
    (1.98, 3548.0, 1284.0, 4.9e6),
    (2.08, 3739.0, 1353.0, 5.03e6),
];

// Data tables: `(M, C_N,alpha per radian, x_cp/d in calibers from the nose tip, C_A at zero angle of attack)`.
//
// C_N,alpha and x_cp/d: Fig. 4 (marker ordinates). Panels (b) and (c) have no ordinate labels of their own for C_N,alpha; they are
// read on the ordinate scale of panel (a) because the gridlines coincide (shared axis, an assumption). Reading uncertainty (about
// +-1.5 px marker centre + calibration): +-0.03 in C_N,alpha (0.0136 per px), +-0.006 in x_cp/d (0.0027 per px), Mach +-0.01
// before snapping (+-0.02 for the snapping itself).
//
// C_A: Fig. 5, "axial-force coefficient at zero normal force" (the text says: at zero measured angle of attack), adjusted to free-stream
// static pressure at the base, so it is a power-on axial force with the base force removed (a jet filling the whole base is assumed).
// Reference area is the body cross-section. Reading uncertainty +-0.002 (0.0007 per px); +-0.004 for markers partly hidden by another symbol.
//
// NAN: the marker is hidden by others and could not be read.

/// Model A, roll 0, 10 x 10 ft tunnel (circles). C_N,alpha and x_cp: 2.00 to 2.78 are free of overlap except at 2.00 / 2.09 where the
/// circle is partly covered by squares and diamonds (+-0.05 in C_N,alpha, +-0.01 in x_cp). The Mach 3.17 x_cp circle coincides with the diamond
/// and was fitted at the position of the roll-45 diamond, 8.198 (it could not be seen separately).
pub const MODEL_A_ROLL0_10X10: [(f64, f64, f64, f64); 6] = [
    (2.00, 11.28, 8.779, 0.451),
    (2.09, 11.47, 8.671, 0.437),
    (2.38, 10.54, 8.543, 0.403),
    (2.78, 9.49, 8.359, 0.347),
    (3.17, 8.93, 8.198, 0.302),
    (3.47, 8.68, 8.254, 0.266),
];

/// Model A, roll 0, 8 x 6 ft tunnel (squares). The Mach 1.98 x_cp square is buried in a cluster of symbols (NAN); its C_N,alpha and C_A
/// squares are partly overlapped by a diamond (+-0.05 and +-0.004). Fig. 4 puts the 8 x 6 ft slopes (12.9 at Mach 1.79) above the
/// extension of the 10 x 10 ft ones; the report attributes offsets in this tunnel to flow angularity or fin cant error.
pub const MODEL_A_ROLL0_8X6: [(f64, f64, f64, f64); 4] =
    [(1.79, 12.87, 8.831, 0.521), (1.88, 12.70, 8.741, 0.492), (1.98, 11.80, f64::NAN, 0.466), (2.08, 10.86, 8.707, 0.445)];

/// Model A, roll 45, 10 x 10 ft tunnel (diamonds). Mach 2.00 diamond is largely hidden behind a square and a circle (C_N,alpha +-0.06,
/// C_A +-0.004, x_cp not readable); the Mach 2.09 diamond was not found (hidden behind the Mach 2.09 circle in Fig. 4; absent from Fig. 5). Mach 2.38 and 2.78
/// x_cp diamonds overlap circles (+-0.01).
pub const MODEL_A_ROLL45_10X10: [(f64, f64, f64, f64); 6] = [
    (2.00, 11.60, f64::NAN, 0.465),
    (2.09, f64::NAN, f64::NAN, f64::NAN),
    (2.38, 11.32, 8.513, 0.403),
    (2.78, 10.26, 8.359, 0.335),
    (3.17, 9.17, 8.198, 0.287),
    (3.47, 8.44, 8.238, 0.262),
];

/// Model B (A without the 4.2 in cylinder section; length 61.7 in), roll 0, 10 x 10 ft tunnel. Five Mach numbers (no marker at 2.09).
/// x_cp is measured from the nose tip of the shortened body. The Mach 3.47 C_N,alpha lies above the Mach 3.17 value (8.84 -> 9.12).
pub const MODEL_B_ROLL0_10X10: [(f64, f64, f64, f64); 5] =
    [(2.00, 11.53, 8.198, 0.456), (2.38, 9.98, 8.018, 0.403), (2.78, 9.41, 7.867, 0.351), (3.17, 8.84, 7.749, 0.287), (3.47, 9.12, 7.677, 0.271)];

/// Model C (fin planform area 70.34 in^2, span 4.38 in; length 65.9 in), roll 0, 10 x 10 ft tunnel. The Mach 3.17 and 3.47 C_N,alpha markers
/// lie at the bottom edge of the plotted C_N,alpha scale (8.0 gridline, 8.03 and 8.15 read; +-0.03 as elsewhere).
pub const MODEL_C_ROLL0_10X10: [(f64, f64, f64, f64); 5] =
    [(2.00, 10.30, 8.576, 0.416), (2.38, 9.21, 8.360, 0.366), (2.78, 9.00, 8.131, 0.325), (3.17, 8.03, 8.002, 0.286), (3.47, 8.15, 7.997, 0.267)];

#[cfg(test)]
mod tests {
    use super::*;

    const ALL: [&[(f64, f64, f64, f64)]; 5] = [&MODEL_A_ROLL0_10X10, &MODEL_A_ROLL0_8X6, &MODEL_A_ROLL45_10X10, &MODEL_B_ROLL0_10X10, &MODEL_C_ROLL0_10X10];

    #[test]
    fn mach_increases_and_matches_table_ii() {
        for t in ALL {
            assert!(t.windows(2).all(|w| w[0].0 < w[1].0));
            for row in t {
                assert!(TEST_CONDITIONS.iter().any(|c| (c.0 - row.0).abs() < 1e-12), "{}", row.0);
            }
        }
    }

    #[test]
    fn dynamic_pressure_matches_total_pressure() {
        // q = 0.7 p M^2 with p = p0 (1 + 0.2 M^2)^-3.5; the printed values agree within 1 %, except the 8 x 6 ft tunnel's Mach 2.08 row, whose printed 1353 is
        // 5.9 % above the isentropic 1278 (checked on the scan: it is printed so, a misprint of the report; its Reynolds number 5.03e6 per ft is that of the other rows).
        for &(m, p0, q, _) in &TEST_CONDITIONS {
            let p = p0 * (1.0 + 0.2 * m * m).powf(-3.5);
            let tol = if (m - 2.08).abs() < 1e-12 { 0.07 } else { 0.01 };
            assert!((0.7 * p * m * m / q - 1.0).abs() < tol, "M {m}");
        }
    }

    #[test]
    fn values_in_range_and_cp_between_nose_and_base() {
        for (t, len_cal) in ALL.iter().zip([10.98, 10.98, 10.98, 10.28, 10.98]) {
            for &(_, cna, xcp, ca) in t.iter() {
                if cna.is_finite() {
                    assert!((8.0..13.5).contains(&cna));
                }
                if xcp.is_finite() {
                    assert!(xcp > 7.5 && xcp < len_cal);
                }
                if ca.is_finite() {
                    assert!((0.25..0.55).contains(&ca));
                }
            }
        }
    }

    #[test]
    fn c_a_falls_with_mach_and_model_c_is_lowest() {
        for t in ALL {
            let v: Vec<f64> = t.iter().map(|r| r.3).filter(|x| x.is_finite()).collect();
            assert!(v.windows(2).all(|w| w[1] < w[0]));
        }
        for (a, c) in MODEL_A_ROLL0_10X10.iter().zip(MODEL_C_ROLL0_10X10.iter()) {
            if a.0 == c.0 {
                assert!(c.3 < a.3);
            }
        }
    }

    #[test]
    fn nose_table_is_monotone_and_power_fit_is_close() {
        assert!(NOSE_PROFILE_IN.windows(2).all(|w| w[1].0 > w[0].0 && w[1].1 > w[0].1));
        let (lp, n, r) = (NOSE_POWER_LENGTH / IN, NOSE_POWER_N, 3.0);
        for &(x, rad) in &NOSE_PROFILE_IN {
            let fit = if x >= lp { r } else { r * (x / lp).powf(n) };
            assert!((fit - rad).abs() < 0.02 * r, "x {x}");
        }
        assert!((NOSE_LENGTH / IN - 25.9).abs() < 1e-9);
    }

    #[test]
    fn fin_geometry_reproduces_stated_areas() {
        let (cr, ct, s) = (FIN_ROOT_CHORD / IN, FIN_TIP_CHORD / IN, FIN_SPAN / IN);
        assert!((2.0 * 0.5 * (cr + ct) * s - 77.66).abs() < 0.02);
        // tip trailing edge aft of root trailing edge = span (45 deg) + tip chord - root chord
        assert!((FIN_TIP_TE_AFT_OF_ROOT_TE / IN - (s + ct - cr)).abs() < 1e-9);
        // model C: 70.34 in^2 within 1 % for straight edges cut at 4.38 in
        let sc = FIN_SPAN_MODEL_C / IN;
        let chord_cut = cr - sc * (cr - ct) / s * 1.0;
        assert!((2.0 * 0.5 * (cr + chord_cut) * sc / 70.34 - 1.0).abs() < 0.015);
        assert!((s - sc - 0.57).abs() < 1e-9);
    }
}
