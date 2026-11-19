//! Army-Navy Basic Finner (ANF) reference projectile: geometry and measured static stability derivatives, used to gate the
//! 3D Euler stability solver and to judge the Barrowman-style (`Extended`) estimate.
//!
//! # Source
//!
//! Dupuis, A., "Aeroballistic range and wind tunnel tests of the basic finner reference projectile from subsonic to high
//! supersonic velocities", DRDC Valcartier TM 2002-136 (wind tunnel and free flight), with the supersonic free-flight part
//! in Dupuis & Hathaway, DTIC ADA636861. Neither report was reachable when this module was written (DTIC was under
//! maintenance), so the numbers below are **digitised from Yayla (2021), METU thesis, Figure 3.10 (C_Nα vs Mach, p. 35 of
//! the thesis, PDF p. 57) and Figure 4.4 (C_mα vs Mach, p. 40, PDF p. 62)**, which replot Dupuis's wind-tunnel (WT) and
//! free-flight (FF) points (<https://open.metu.edu.tr/bitstream/handle/11511/93161/index.pdf>). Replace them with the
//! tabulated values from Dupuis when a copy is available.
//!
//! # Drag
//!
//! [`CD0`] is the zero-incidence drag the 3D solver is gated against: **zero-cant** free-flight single fits (supersonic, DREV-TM-9703 Table VIII; Dupuis & Hathaway,
//! DTIC ADA636861, <https://archive.org/details/DTIC_ADA636861>) and, subsonic, the one free-flight point (Mach 0.766, `C_X0` 0.508, pooled, fin cant 2 degrees, 1 % standard
//! error) of DRDC TM 2002-136 Table 7 as replotted in ARL-TR-6725 Fig. 5 (DTIC ADA592550, <https://archive.org/details/DTIC_ADA592550>; the replot only, TM 2002-136 does
//! not allow redistribution of its tables, so no other value of it is committed). The sting-mounted wind-tunnel points of the same figure ([`CD0_TUNNEL`], 0 degree roll,
//! Mach 0.5 / 0.7) are INFO: the sting and its support interfere with the base flow. The solver models uncanted fins;
//! the shots with 2 and 4 degree fin cant spin up and carry roll-induced drag, and the pooled fits of Table IX ([`CD0_POOLED`]) mix all three cants. The range
//! models flew with a cold wall (launched at ambient temperature, so well below the recovery temperature), the solver's wall is adiabatic: the friction of the
//! data is higher (Van Driest II, estimated at about +14 % body and +12 % fin friction at Mach 1.8), a bias of the references no choice of table removes. In Table VIII a shot is zero-cant when its roll-moment-due-to-cant `Cl_delta` is 0.00 (the
//! second line of each entry) and it is the Table IV model of cant 0 at that mid-range Mach number; the Mach number of a single fit is the mid-range one.
//!
//! # Digitisation
//!
//! The thesis pages were rendered at 200 dpi, the axis frames located to the pixel, and the marker centres found by
//! template correlation (green squares = WT, purple circles = FF). Marker overlaps were resolved by hand. Both figures
//! are in 1/deg about the CG; they are converted to per radian here. Digitised, ±3 % on each of C_Nα and C_mα. The centre
//! of pressure is derived, so its error is larger: x_cp − x_cg = −C_mα/C_Nα·d is a ratio of two ±3 % numbers, about
//! ±0.1 cal at 3 cal of lever arm (more in the transonic scatter band, where individual shots differ by 0.05 in C_Nα).
//!
//! Where several wind-tunnel markers sit within ΔM ≈ 0.04 of each other (Fig. 3.10 and 4.4 overlap them), they are
//! averaged into one row; the free-flight shots are kept individually. One free-flight C_mα (M 1.376) was hidden behind a
//! WT marker in Fig. 4.4 and is read by eye (±0.02).
//!
//! # Geometry (Yayla Fig. 3.1, from Dupuis; one caliber = 0.03 m)
//!
//! Verified against the figure: length 10 cal; cone half-angle 10° with nose length 2.84 cal (= 0.5/tan 10°) and nose-tip
//! radius 0.004 cal; cylindrical afterbody 1 cal in diameter; four rectangular fins, chord 1 cal, **exposed** span 1 cal
//! (3 cal tip to tip), root trailing edge flush with the base; fin thickness 0.08 cal; CG 5.5 cal from the nose; fins
//! canted 2° (roll only, ignored by the pitch-plane methods). Yayla's text says "20 degree nose cone", which is the
//! included angle (the figure dimensions 10.0° and 2.84).
//!
//! Fin edges are not dimensioned in Fig. 3.1. Abiza et al. (XFlow LBM study of the ANF, p. 3,
//! <https://www.3ds.com/assets/invest/2024-02/prediction_of_finned_projectile_aerodynamics_using_a_lattice-boltzmann_method_cfd_solution.pdf>)
//! give a sharp leading edge and a blunt 0.08 cal trailing edge; Shelton, Martin & Silva (NASA, Kestrel/FUN3D, p. 2 and
//! Table 1, <https://ntrs.nasa.gov/api/citations/20180006293/downloads/20180006293.pdf>) a leading-edge fillet r/d = 0.005
//! and a 0.1 d trailing edge. The nose geometry above agrees with both.
//!
//! # Measurement spread
//!
//! Two wind tunnels (DREV, ISL) measuring a similar finned projectile differ by about 11 % in C_Nα and 0.4 cal in x_cp at
//! M 3 (Dupuis & Berner, ISBS 2001, <http://www.xrayct.com/documents/data/IBS19/EB11_495.pdf>, Fig. 5); treat the
//! high-supersonic points with that spread in mind, on top of the digitisation error.
//!
//! The reference area is the body cross-section π d²/4 and the reference length d. Pitching moment is about the CG, so
//! `x_cp = x_cg − C_mα / C_Nα` (calibers from the nose tip; C_mα < 0 is stable).

use std::f64::consts::PI;

/// One measured point.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AnfPoint {
    pub mach: f64,
    /// Normal-force slope per radian, reference area π d²/4.
    pub cn_alpha: f64,
    /// Centre of pressure from the nose tip (calibers), from `x_cg − C_mα/C_Nα` with the CG at [`CG_CAL`].
    pub x_cp_cal: f64,
}

/// Total length (calibers).
pub const LENGTH_CAL: f64 = 10.0;
/// Nose cone half-angle (degrees), verified (Fig. 3.1).
pub const NOSE_HALF_ANGLE_DEG: f64 = 10.0;
/// Nose length (calibers), verified (Fig. 3.1: 2.84).
pub const NOSE_LENGTH_CAL: f64 = 2.84;
/// Fin chord (calibers), verified (Fig. 3.1).
pub const FIN_CHORD_CAL: f64 = 1.0;
/// Exposed fin span (calibers), verified (Fig. 3.1).
pub const FIN_SPAN_CAL: f64 = 1.0;
/// Fin thickness (calibers), verified (Fig. 3.1: 0.08).
pub const FIN_THICKNESS_CAL: f64 = 0.08;
/// Number of fins.
pub const FIN_COUNT: u32 = 4;
/// Centre of gravity from the nose tip (calibers), verified (Fig. 3.1: 5.50).
pub const CG_CAL: f64 = 5.5;

const DEG: f64 = 180.0 / PI;

/// Builds a point from the figures' per-degree values.
const fn pt(mach: f64, cn_per_deg: f64, cm_per_deg: f64) -> AnfPoint {
    AnfPoint { mach, cn_alpha: cn_per_deg * DEG, x_cp_cal: CG_CAL - cm_per_deg / cn_per_deg }
}

/// Zero-incidence drag coefficient `C_D0` of the ANF (reference area `pi d^2 / 4`, base drag included) as `(Mach, C_D0)`: what the G-F5 gate of the 3D engine is set
/// against. Fin cant 0 degrees throughout, see the module documentation, "Drag".
///
/// - Supersonic (Mach 1.850, 2.364, 2.741, 2.970): the **single-fit** `C_X0` of the zero-cant shots, Dupuis & Hathaway, DREV-TM-9703 (DTIC ADA636861), **Table VIII**
///   (pdf pp. 44-47): shot DA95021308 (A08) Mach 1.850 0.567; DA95020907 (A07) 2.364 0.478; DA95020906 (A06) 2.741 0.400; the two zero-cant shots at the nominal
///   Mach 3 charge, DA95030603 (A05) 2.969 0.385 and DA95022205 (A03) 2.970 0.376, are one row here at their mean 0.3805 (2 % apart; the printed probable errors of C_X0 round to 0 %). The pooled table has no zero-cant point at Mach 3.147, so the gate point moved to Mach 2.970.
/// - Transonic (Mach 1.056, 1.332): the same table's zero-cant single fits (pdf p. 44; image checked): shot DA95022010 Mach 1.056 `C_X0` 0.868 (`Cl_delta` -0.001) and shot DA95022009
///   Mach 1.332 0.701 (`Cl_delta` -0.011, against 0.29-0.93 of every canted shot of the page: 1.058 / 1.116 / 1.254 / 1.377 with 0.288 / 0.562 / 0.340 / 0.925). The canted shot DA95022219 at Mach 1.058
///   reads the same 0.868, which is the only evidence of the cant's size here (none at 1.056); the other canted shots, 0.854 at 1.116, 0.760 at 1.254, 0.708 at 1.377, are not used. The
///   printed probable error of `C_X0` is 0 % in both. The 8 % gate band is kept: the source scatter does not justify a wider one, but the Mach spread is the limit of what it can resolve:
///   the table's drag falls by 0.6 per Mach number between the two points (0.868 to 0.701), the Mach number of a shot is the mid-range one of its flight, so +-0.02 in Mach is +-1.4 %.
/// - Subsonic (Mach 0.766): the free-flight pooled `C_X0` 0.508 of DRDC TM 2002-136 Table 7 (fin cant 2 degrees, standard error 1 %), read from the ARL-TR-6725 Fig. 5
///   replot (see the module documentation). It is the only subsonic gate point; the fin cant adds roll-induced drag the uncanted solver does not model.
#[allow(clippy::approx_constant)] // 2.718 is a Mach number of the table, not e
pub const CD0: &[(f64, f64)] = &[(0.766, 0.508), (1.056, 0.868), (1.332, 0.701), (1.850, 0.567), (2.364, 0.478), (2.741, 0.400), (2.970, 0.3805)];

/// The sting-mounted wind-tunnel (DRDC WT, 0 degree roll) subsonic `C_D0` as replotted in Fig. 5 of Bhagwandin & Sahu, ARL-TR-6725 (DTIC ADA592550, p. 15, top left panel,
/// filled markers): 0.467 at Mach 0.497 and 0.479 at Mach 0.692, read off the figure at 600 dpi to **+-0.015** (marker size and the 0.2 gridline spacing) and **+-0.02 in
/// Mach**; used at the nominal Mach 0.5 and 0.7. **INFO only** (sting interference), see [`CD0`].
pub const CD0_TUNNEL: &[(f64, f64)] = &[(0.5, 0.467), (0.7, 0.479)];

/// The pooled ("multiple-fit") range values, DREV-TM-9703 **Table IX** (pdf pp. 48-49): `C_X0` 0.585 / 0.484 / 0.435 / 0.373 at Mach 1.832 / 2.375 / 2.718 / 3.147.
/// Each is one fit over shots of fin cant 0, 2 and 4 degrees (unique `C_X0` per group, solved with the roll-coupled 6-DOF model), so it carries whatever drag the
/// canted shots' roll adds and is **INFO only**, not a gate reference. This is the table (and the
/// open circles of Fig. 5 of ARL-TR-6725) the earlier gate was set against.
#[allow(clippy::approx_constant)] // 2.718 is a Mach number of the table, not e
pub const CD0_POOLED: &[(f64, f64)] = &[(1.832, 0.585), (2.375, 0.484), (2.718, 0.435), (3.147, 0.373)];

/// Wind-tunnel points (Dupuis; digitised from Yayla 2021 Fig. 3.10 p. 35 and Fig. 4.4 p. 40, digitised, ±3 %).
/// Clusters within ΔM ≈ 0.04 are averaged.
#[allow(clippy::approx_constant)] // 0.318 is a digitised reading, not 1/π
pub const WIND_TUNNEL: &[AnfPoint] = &[
    pt(0.69, 0.274, -0.742),
    pt(0.79, 0.284, -0.779),
    pt(0.914, 0.313, -0.898),
    pt(1.064, 0.343, -1.021),
    pt(1.175, 0.318, -0.940),
    pt(1.245, 0.303, -0.888),
    pt(1.28, 0.299, -0.880),
    pt(1.36, 0.279, -0.786),
    pt(1.484, 0.247, -0.668),
    pt(1.728, 0.208, -0.507),
    pt(1.99, 0.185, -0.403),
    pt(2.48, 0.163, -0.288),
];

/// Free-flight points (Dupuis; digitised from Yayla 2021 Fig. 3.10 p. 35 and Fig. 4.4 p. 40, digitised, ±3 %). One shot per
/// row; the M 1.376 C_mα is read by eye (±0.02).
pub const FREE_FLIGHT: &[AnfPoint] = &[
    pt(0.727, 0.288, -0.710),
    pt(0.803, 0.292, -0.729),
    pt(0.904, 0.329, -0.810),
    pt(0.929, 0.320, -0.837),
    pt(0.966, 0.320, -0.826),
    pt(1.055, 0.326, -0.881),
    pt(1.058, 0.314, -0.901),
    pt(1.077, 0.370, -0.878),
    pt(1.115, 0.323, -0.952),
    pt(1.134, 0.370, -0.920),
    pt(1.255, 0.329, -0.917),
    pt(1.332, 0.285, -0.872),
    pt(1.376, 0.286, -0.800),
    pt(1.80, 0.197, -0.505),
    pt(1.85, 0.206, -0.479),
    pt(2.351, 0.181, -0.343),
    pt(2.366, 0.177, -0.330),
    pt(2.414, 0.172, -0.314),
    pt(2.665, 0.155, -0.278),
    pt(2.746, 0.155, -0.262),
];

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn drag_tables_are_ordered() {
        for t in [CD0, CD0_TUNNEL, CD0_POOLED] {
            assert!(t.windows(2).all(|w| w[0].0 < w[1].0));
        }
    }

    #[test]
    fn data_is_sane() {
        for set in [WIND_TUNNEL, FREE_FLIGHT] {
            assert!(set.windows(2).all(|w| w[0].mach < w[1].mach));
            for p in set {
                // CNα 8..25 per rad; CP behind the CG and ahead of the base.
                assert!(p.cn_alpha > 8.0 && p.cn_alpha < 25.0, "{p:?}");
                assert!(p.x_cp_cal > CG_CAL && p.x_cp_cal < LENGTH_CAL, "{p:?}");
            }
        }
        assert!((NOSE_LENGTH_CAL - 0.5 / NOSE_HALF_ANGLE_DEG.to_radians().tan()).abs() < 0.01);
    }
}
