//! Tube fins: open tubes standing against the body with air flowing through them, the aerodynamics of an empty body tube on
//! the outside of a rocket (OpenRocket's `TubeFinSet`).
//!
//! A port of OpenRocket's `TubeFinSetCalc` and its base class `TubeCalc` (source:
//! <https://github.com/openrocket/openrocket>, `core/.../aerodynamics/barrowman/`; technical documentation, Niskanen 2013,
//! section 3.1.1). Each tube is computed alone and the results are multiplied by the tube count (OpenRocket's instance count);
//! the interference between tubes and with the body is not modelled except for the interstice area below.
//!
//! * **Normal force.** The tube is a ring airfoil: Ribner (1947), "The ring airfoil in nonaxial flow", J. Aeronautical
//!   Sciences 14(9), eq. (5), with the aspect ratio `AR = 2 r_in / L` and `AR' = 2 AR / pi`:
//!   `CN_alpha A_ref = 2 AR'/(1 + AR') pi^2 r_in L` per tube. There is no Mach dependence and no body-fin interference
//!   factor (OpenRocket applies neither). The angle of attack is limited to the 20 degree stall angle (`CN = CN_alpha
//!   min(alpha, 20 deg)`).
//! * **Centre of pressure.** The fin CP fraction of the chord `L` from the fore end: the quarter chord up to Mach 0.5, the
//!   supersonic formula of `AR beta` above Mach 2 and the shape-preserving interpolation between ([`fin_cp_fraction`]).
//! * **Friction drag.** `Cf A_wet / A_ref` with the rocket's flat-plate `Cf` and the *outer* wetted area of the tube less the
//!   part of the body surface it covers; the inside of the tube is not counted as wetted area. The flow through the bore is
//!   charged as a pressure drop instead (next item). No cylindrical-body correction applies (a tube fin is not a body).
//! * **Pressure drag.** From `TubeCalc`: the Darcy-Weisbach pressure drop of the bore (Swamee-Jain friction factor with
//!   `Re = V (2 r_in) / nu`, `C_D,tube = f L / (2 r_in)`) on the bore area, plus `0.7 (C_stag + C_base)` on the annular
//!   front face; from `TubeFinSetCalc` a further `(C_stag + C_base)` on the interstice between the tube and the body, the
//!   area between them that the neighbouring tube and the body enclose.
//! * **Base drag.** None: `TubeFinSetCalc` does not override `calculateComponentBaseCD`, so an open tube has no base.
//!
//! Interstice, tangent to the body: the tube axis is `D = R + r` from the body axis (`R` the body radius, `r` the tube's outer
//! radius), the tangent from the body axis to the tube circle has length `d = sqrt(D^2 - r^2)`, and with `theta1 =
//! acos(r/D)` the interstice is `d r - r^2 theta1 - R^2 (pi/2 - theta1)`; the covered body area is `2 L R (pi/2 - theta1)`
//! and the tube's exposed outer surface `2 L r (pi - theta1)`. A tube with a gap to the body (not in OpenRocket) is a free
//! tube: no interstice, no covered area, the whole outer surface wetted.

use crate::aero::{fin_cp_fraction, lug_drag, FIN_STALL};
use crate::vehicle::TubeFinSet;
use std::f64::consts::PI;

/// Precomputed aerodynamics of one tube fin set (all its tubes).
#[derive(Clone, Debug, PartialEq)]
pub struct TubeFinCalc {
    count: f64,
    station: f64,
    length: f64,
    inner_radius: f64,
    outer_radius: f64,
    /// Aspect ratio `2 r_in / L`.
    ar: f64,
    /// `2 AR'/(1 + AR') pi^2 r_in L` (m^2): the normal-force slope of one tube times the reference area.
    cna_const: f64,
    /// Interstice area of one tube (m^2).
    interstice: f64,
    /// Wetted area of one tube for friction drag (m^2).
    wetted: f64,
    roughness: f64,
}

impl TubeFinCalc {
    /// Precomputes the geometry of `t`.
    pub fn new(t: &TubeFinSet) -> Self {
        let (l, r_out, r_in) = (t.length.max(1e-9), t.outer_radius, t.inner_radius.clamp(0.0, t.outer_radius));
        let ar = 2.0 * r_in / l;
        let ar_prime = 2.0 * ar / PI;
        let cna_const = 2.0 * (ar_prime / (1.0 + ar_prime)) * PI * PI * r_in * l;
        let (interstice, wetted) = if t.gap <= 1e-9 && t.body_radius > 0.0 && r_out > 0.0 {
            let body = t.body_radius;
            let centres = body + r_out;
            let d = (centres * centres - r_out * r_out).sqrt();
            let theta1 = (r_out / centres).acos();
            let theta2 = 0.5 * PI - theta1;
            let interstice = d * r_out - r_out * r_out * theta1 - body * body * theta2;
            let outer = l * 2.0 * (PI - theta1) * r_out;
            let masked = l * 2.0 * theta2 * body;
            (interstice, outer - masked)
        } else {
            (0.0, l * 2.0 * PI * r_out)
        };
        Self {
            count: f64::from(t.count),
            station: t.station,
            length: l,
            inner_radius: r_in,
            outer_radius: r_out,
            ar,
            cna_const,
            interstice,
            wetted,
            roughness: t.roughness,
        }
    }

    /// Surface roughness height (m).
    pub fn roughness(&self) -> f64 {
        self.roughness
    }

    /// Normal-force slope of the whole set at angle of attack `alpha` (rad) on `a_ref`: `count CN_alpha min(|alpha|, 20 deg)
    /// / |alpha|`, so that `slope * alpha` is OpenRocket's `CN`. Zero for a tube of outer radius under 1 mm, as OpenRocket.
    pub fn cna(&self, alpha: f64, a_ref: f64) -> f64 {
        if self.outer_radius < 0.001 {
            return 0.0;
        }
        let a = alpha.abs();
        let stall = if a > 1e-9 { a.min(FIN_STALL) / a } else { 1.0 };
        self.count * self.cna_const / a_ref * stall
    }

    /// CP station of the set at `mach` (m from the tip).
    pub fn cp(&self, mach: f64) -> f64 {
        self.station + fin_cp_fraction(mach, self.ar) * self.length
    }

    /// Friction drag coefficient of the whole set for flat-plate coefficient `cf`.
    pub fn friction_cd(&self, cf: f64, a_ref: f64) -> f64 {
        self.count * cf * self.wetted / a_ref
    }

    /// Pressure drag coefficient of the whole set (see the module docs) at `mach`, `re_per_m` = V/nu, with the stagnation and
    /// base coefficients `stag`, `base` of that Mach number.
    pub fn pressure_cd(&self, re_per_m: f64, stag: f64, base: f64, a_ref: f64) -> f64 {
        let inner_area = PI * self.inner_radius * self.inner_radius;
        let frontal = PI * (self.outer_radius * self.outer_radius - self.inner_radius * self.inner_radius);
        let tube = lug_drag(re_per_m, 2.0 * self.inner_radius, inner_area, frontal, self.length, self.roughness, stag, base, a_ref);
        self.count * (tube + (stag + base) * self.interstice / a_ref)
    }

    /// Wetted area of one tube (m^2), the part of its outer surface in the flow.
    pub fn wetted_area(&self) -> f64 {
        self.wetted
    }

    /// Interstice area of one tube (m^2).
    pub fn interstice_area(&self) -> f64 {
        self.interstice
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Three tubes of 0.15 m with a 25.4 mm outer and 23.4 mm inner diameter against a 76.2 mm body (reference area
    /// `pi 0.0381^2 = 4.5604e-3 m^2`), at the station 1.0 m.
    fn set() -> TubeFinSet {
        TubeFinSet { count: 3, station: 1.0, length: 0.15, outer_radius: 0.0127, inner_radius: 0.0117, body_radius: 0.0381, gap: 0.0, roughness: 60e-6 }
    }

    const A_REF: f64 = PI * 0.0381 * 0.0381;

    /// Hand calculation of OpenRocket's `TubeFinSetCalc` for [`set`]: AR = 2 (0.0117)/0.15 = 0.156, AR' = 0.099313,
    /// `cnaconst = 2 (AR'/(1+AR')) pi^2 (0.0117)(0.15) = 3.12961e-3 m^2`, so `CN_alpha = 0.68626` per tube and 2.05879 for three.
    #[test]
    fn normal_force_slope_is_ribners_ring_airfoil() {
        let calc = TubeFinCalc::new(&set());
        assert!((calc.cna_const - 3.129_611e-3).abs() < 1e-8, "{}", calc.cna_const);
        assert!((calc.cna(0.0, A_REF) - 2.058_789).abs() < 1e-5, "{}", calc.cna(0.0, A_REF));
        // One tube is a third of it; no Mach dependence.
        let one = TubeFinCalc::new(&TubeFinSet { count: 1, ..set() });
        assert!((one.cna(0.0, A_REF) - 0.686_263).abs() < 1e-5);
    }

    /// CN = CN_alpha min(alpha, 20 deg): the slope falls past the stall angle.
    #[test]
    fn normal_force_stalls_at_twenty_degrees() {
        let calc = TubeFinCalc::new(&set());
        let cn = |a: f64| calc.cna(a, A_REF) * a;
        assert!((cn(0.1) - 2.058_789 * 0.1).abs() < 1e-5);
        assert!((cn(0.5) - 2.058_789 * FIN_STALL).abs() < 1e-5);
    }

    /// The CP is the quarter chord at low Mach (OpenRocket's `SUBSONIC_CP_POS`): 1.0 + 0.25 (0.15) = 1.0375 m.
    #[test]
    fn centre_of_pressure_is_the_quarter_chord_at_low_mach() {
        let calc = TubeFinCalc::new(&set());
        assert!((calc.cp(0.3) - 1.0375).abs() < 1e-12);
        // This tube's AR is 0.156, so even at Mach 3 (AR beta = 0.44, under the bridge start) the CP stays at the quarter chord.
        assert!((calc.cp(3.0) - 1.0375).abs() < 1e-12);
    }

    /// Interstice and wetted area by the geometry of `TubeFinSetCalc`: d = 0.049187, theta1 = 1.318116, theta2 = 0.252680,
    /// interstice = 4.5281e-5 m^2, outer surface 6.9474e-3 less the covered body 2.8881e-3 = 4.0593e-3 m^2.
    #[test]
    fn interstice_and_wetted_area() {
        let calc = TubeFinCalc::new(&set());
        assert!((calc.interstice_area() - 4.528_136e-5).abs() < 1e-9, "{}", calc.interstice_area());
        assert!((calc.wetted_area() - 4.059_310e-3).abs() < 1e-8, "{}", calc.wetted_area());
        // A tube clear of the body wets its whole outer surface and has no interstice.
        let free = TubeFinCalc::new(&TubeFinSet { gap: 0.01, ..set() });
        assert_eq!(free.interstice_area(), 0.0);
        assert!((free.wetted_area() - 0.15 * 2.0 * PI * 0.0127).abs() < 1e-12);
    }

    /// Drag by the formulas of `TubeCalc` / `TubeFinSetCalc` at Mach 0.3, V = 102 m/s, nu = 1.4607e-5 m^2/s: stagnation
    /// 0.869297, base 0.1317, Re(bore) = 163401, Swamee-Jain f = 0.026155, bore drag f L / D = 0.167658, annular front face
    /// 7.6655e-5 m^2: per tube `Cd = 0.037528`, three tubes 0.112583. Friction with Cf = 0.004: 3 (0.004)(4.0593e-3)/A_ref.
    #[test]
    fn drag_matches_the_hand_calculation() {
        let calc = TubeFinCalc::new(&set());
        let stag = 0.85 * (1.0 + 0.09 / 4.0 + 0.3f64.powi(4) / 40.0);
        let base: f64 = 0.12 + 0.13 * 0.09;
        assert!((stag - 0.869_297).abs() < 1e-6 && (base - 0.1317).abs() < 1e-9);
        let pressure = calc.pressure_cd(102.0 / 1.4607e-5, stag, base, A_REF);
        assert!((pressure - 0.112_583).abs() < 2e-5, "{pressure}");
        let friction = calc.friction_cd(0.004, A_REF);
        assert!((friction - 3.0 * 0.004 * 4.059_310e-3 / A_REF).abs() < 1e-7, "{friction}");
        // Still air: only the interstice term is left.
        assert!(calc.pressure_cd(0.0, stag, base, A_REF) > 0.0);
    }
}
