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
