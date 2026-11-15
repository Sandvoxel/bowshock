//! Half-scale Arcas sounding-rocket model of NASA TN D-4013 (J. C. Ferris, 1967, Mach 0.60-1.20,
//! <https://ntrs.nasa.gov/citations/19670020050>) and TN D-4014 (C. D. Babb & D. E. Fuller, 1967, Mach 1.50-4.63,
//! <https://ntrs.nasa.gov/citations/19670020031>), configuration 1 ("short", l/d = 18.20).
//!
//! Geometry is taken from D-4014 Fig. 1. D-4013 Fig. 1 differs by more than 2 % in fin span (2.2 %) and chords (2.8 %, 3.3 %);
//! both values are given in the constants' docs. All data are figure readings, with the reading uncertainty stated on each table.

const IN: f64 = 0.0254;

/// Body diameter, 2.250 in (D-4014 Fig. 1(a)).
pub const BODY_DIAMETER: f64 = 2.250 * IN;
/// Nose length, 10.600 in (D-4014 Fig. 1(a)). The real nose has tabulated ordinates and reaches full radius at 9.375 in; it is
/// modelled as a tangent ogive of this length (up to 7 % thicker in radius).
pub const NOSE_LENGTH: f64 = 10.600 * IN;
/// Cylinder length, 28.540 in: from the end of the nose to the start of the boattail at station 39.140 in (D-4014 Fig. 1(a)).
pub const TUBE_LENGTH: f64 = 28.540 * IN;
/// Boattail length, 1.810 in (D-4014 Fig. 1(a)); half-angle about 12.2 degrees.
pub const BOATTAIL_LENGTH: f64 = 1.810 * IN;
/// Base diameter, 1.470 in (D-4014 Fig. 1(a) detail A; 0.65 d in D-4013 Fig. 1).
pub const BASE_DIAMETER: f64 = 1.470 * IN;
/// Total length of configuration 1, 40.950 in (D-4014 Fig. 1(a); 18.20 d).
pub const TOTAL_LENGTH: f64 = 40.950 * IN;

/// Fin root chord, 3.380 in (D-4014 Fig. 1(b)); D-4013 Fig. 1: 3.285 in.
pub const FIN_ROOT_CHORD: f64 = 3.380 * IN;
/// Fin tip chord, 2.165 in (D-4014 Fig. 1(b)); D-4013 Fig. 1: 2.093 in.
pub const FIN_TIP_CHORD: f64 = 2.165 * IN;
/// Fin span from the body surface, 2.103 in (D-4014 Fig. 1(b)); D-4013 Fig. 1: 2.149 in.
pub const FIN_SPAN: f64 = 2.103 * IN;
/// Axial offset of the tip leading edge from the root leading edge, 1.215 in (D-4014 Fig. 1(b), 30 degree leading-edge sweep).
pub const FIN_SWEEP: f64 = 1.215 * IN;
/// Mean fin thickness, 0.123 in: the mean of 0.150 in at the root and 0.096 in at the tip (D-4014 Fig. 1(b); which end is which is
/// a probable reading, not certain).
pub const FIN_THICKNESS: f64 = 0.123 * IN;

/// Reynolds number per metre of D-4013 (Fig. 1 text; carborundum transition strips, turbulent flow).
pub const TRANSONIC_RE_PER_M: f64 = 9.8e6;
/// Stagnation temperature of D-4013 \[K\].
pub const TRANSONIC_T0_K: f64 = 322.0;
/// Reynolds number per metre of D-4014 (transition strips, turbulent flow).
pub const SUPERSONIC_RE_PER_M: f64 = 9.84e6;
/// Stagnation temperature of D-4014 at Mach 1.50 to 2.96 \[K\].
pub const SUPERSONIC_T0_K: f64 = 339.0;

/// `(M, C_A,corr fins off, C_A,corr fins on with fin deflection 0)` from D-4013 Fig. 11 (short model, alpha = 0): the axial force
/// with the base-annulus pressure force removed, referenced to the body cross-section; it includes boattail, fins and fin anchors.
/// Reading uncertainty +-0.005 on the flat parts, +-0.02 at Mach 0.90 to 1.00.
pub const TRANSONIC: [(f64, f64, f64); 6] =
    [(0.60, 0.222, 0.300), (0.80, 0.233, 0.332), (0.90, 0.264, 0.42), (0.95, 0.298, 0.57), (1.00, 0.425, 0.691), (1.20, 0.436, 0.600)];

/// `(M, C_A - C_A,c fins off, fins on)` from D-4014 Fig. 5(a)/(b) (`C_A`) and Fig. 4(a) (chamber `C_A,c`), alpha = 0, fin deflection 0.
/// Reading uncertainty +-0.012; the sign of the chamber correction is not stated in the report (inferred: forebody = `C_A - C_A,c`).
pub const SUPERSONIC: [(f64, f64, f64); 4] = [(1.50, 0.340, 0.505), (1.80, 0.322, 0.438), (2.30, 0.258, 0.349), (2.96, 0.210, 0.273)];

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn lengths_add_up() {
        assert!((NOSE_LENGTH + TUBE_LENGTH + BOATTAIL_LENGTH - TOTAL_LENGTH).abs() < 1e-12);
    }
}
