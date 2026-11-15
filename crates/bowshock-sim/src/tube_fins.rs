//! Tube fins: open tubes standing against the body with air flowing through them, the aerodynamics of an empty body tube on
//! the outside of a rocket.
//!
//! Each tube is computed alone and the results are multiplied by the tube count; the interference between tubes and with
//! the body is not modelled except for the interstice area below. Coefficients are on the rocket's reference area (Niskanen
//! 2009, *Development of an Open Source model rocket simulation software*, section 3.1.1).
//!
//! * **Normal force.** The tube is a ring airfoil: Ribner (1947), "The ring airfoil in nonaxial flow", J. Aeronautical
//!   Sciences 14(9), eq. (5), with the aspect ratio `AR = 2 r_in / L` and `AR' = 2 AR / pi`:
//!   `CN_alpha A_ref = 2 AR'/(1 + AR') pi^2 r_in L` per tube. There is no Mach dependence and no body-fin interference
//!   factor. The angle of attack is limited to a 20 degree stall angle (`CN = CN_alpha min(alpha, 20 deg)`).
//! * **Centre of pressure.** The fin CP rule of Niskanen (2009, section 3.2.2) applied to the tube's chord `L` from its fore
//!   end ([`fin_cp_fraction`]): the quarter chord up to Mach 0.5; above Mach 2 Fleeman's empirical
//!   `(x - 0.67)/(2x - 1)` of `x = AR beta` (eq. 3.35; Fleeman 2006, *Tactical Missile Design*, 2nd ed., p. 33); between
//!   them the fifth-order polynomial of eq. 3.36 that meets the Mach 2 value and slope. The empirical formula is a fit for
//!   fins of ordinary aspect ratio: it falls below the quarter chord for `x < 0.84` and has a pole at `x = 0.5`, which long
//!   tubes (low `AR`) reach. No published planform-independent value covers that range, so Bowshock holds the subsonic
//!   thin-airfoil quarter chord for `x <= 0.84` (where the formula itself returns 0.25, so the two meet), joins it to the
//!   formula at `x = 1` with a cubic Hermite bridge that matches value and slope at both ends, and, when the endpoint slope
//!   would make the eq. 3.36 polynomial overshoot and move the CP forward, uses a shape-preserving power of the limiting
//!   monotone quintic instead.
//!   This is our own continuity estimate. A long tube (`AR sqrt(3) <= 0.84`, i.e. `L >= 4.1 r_in`) keeps its CP at the
//!   quarter chord from subsonic speed until `AR beta` reaches 0.84, past Mach 2.
//! * **Friction drag.** `Cf A_wet / A_ref` with the rocket's flat-plate `Cf` and the *outer* wetted area of the tube less the
//!   part of the body surface it covers; the inside of the tube is not counted as wetted area. The flow through the bore is
//!   charged as a pressure drop instead (next item). No cylindrical-body correction applies (a tube fin is not a body).
//! * **Pressure drag.** The bore is a short pipe: the Darcy-Weisbach pressure drop `C_D,bore = f L / (2 r_in)` on the bore
//!   area, with the Swamee-Jain explicit friction factor `f = 0.25 / log10(eps/(3.7 D) + 5.74/Re^0.9)^2`, `D = 2 r_in`,
//!   `Re = V D / nu` (Swamee & Jain 1976, "Explicit equations for pipe-flow problems", J. Hydraulics Division ASCE
//!   102(HY5)), with no compressibility factor. The annular front face carries `0.7 (C_stag + C_base)` and the interstice
//!   between the tube and the body (the area the tube, its neighbour and the body enclose) `(C_stag + C_base)`, with the
//!   blunt-cylinder stagnation coefficient `C_stag = 0.85 q_stag/q` (Niskanen 2009, eqs. B.1-B.2, after Hoerner 1965,
//!   *Fluid-Dynamic Drag*, pp. 15-2, 16-3) and the base coefficient of eq. 3.94 (Fleeman p. 23).
//!   - *Annulus.* The tube wall is a square-edged fin bent into a ring, and the published square-edge fin rule (Niskanen
//!     2009 eqs. 3.90, 3.92-3.93) charges `C_stag + C_base` on its edge area; that is the upper bound. The wall (0.3-1 mm) is
//!     much thinner than the boundary layer at its trailing edge (about 2 mm on a 76 mm tube at 100 m/s), so the trailing
//!     edge does not see a full body-base suction, and the leading edge stagnates only the air that does not turn into the
//!     bore. The factor 0.7 is our estimate of that relief; it is not a published value. With 1.0 instead, the *Tube fin
//!     rocket* comparison case gains +5 % Cd at Mach 0.3 and +8 % at Mach 2, away from the reference runs.
//!   - *Interstice.* The cusp-shaped channel between the tube and the body is a corner a few millimetres across, filled by
//!     the two walls' boundary layers, so it is treated as dead air: a blunt face in front (`C_stag`) and a base behind
//!     (`C_base`), the same blunt-body estimate as for a square edge.
//!
//!   The Mach number enters only through `Re`, `C_stag` and `C_base`, so the drag stays bounded through Mach 1. The whole-rocket
//!   results are compared with independent reference runs in the comparison report under `docs/` (tube fin section).
//! * **Base drag.** None: an open tube has no base.
//!
//! Interstice, tangent to the body: the tube axis is `D = R + r` from the body axis (`R` the body radius, `r` the tube's outer
//! radius), the tangent from the body axis to the tube circle has length `d = sqrt(D^2 - r^2)`, and with `theta1 =
//! acos(r/D)` the interstice is `d r - r^2 theta1 - R^2 (pi/2 - theta1)`; the covered body area is `2 L R (pi/2 - theta1)`
//! and the tube's exposed outer surface `2 L r (pi - theta1)`. A tube with a gap to the body is a free tube: no interstice,
//! no covered area, the whole outer surface wetted.

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
    /// / |alpha|`, so that `slope * alpha` is the stalled `CN`. Zero for a tube of outer radius under 1 mm (a vestigial tube).
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

    /// Ribner's ring airfoil by hand for [`set`]: AR = 2 (0.0117)/0.15 = 0.156, AR' = 0.099313,
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

    /// The CP is the quarter chord at low Mach (Niskanen 2009, section 3.2.2): 1.0 + 0.25 (0.15) = 1.0375 m.
    #[test]
    fn centre_of_pressure_is_the_quarter_chord_at_low_mach() {
        let calc = TubeFinCalc::new(&set());
        assert!((calc.cp(0.3) - 1.0375).abs() < 1e-12);
        // This tube's AR is 0.156, so even at Mach 3 (AR beta = 0.44, under the bridge start) the CP stays at the quarter chord.
        assert!((calc.cp(3.0) - 1.0375).abs() < 1e-12);
    }

    /// The CP of `t` at `mach` as a fraction of its chord from the fore end.
    fn cp_fraction(t: &TubeFinSet, mach: f64) -> f64 {
        (TubeFinCalc::new(t).cp(mach) - t.station) / t.length
    }

    /// Hand values of the CP rule (Niskanen 2009 eqs. 3.35-3.36 with Bowshock's low-aspect-ratio bridge). A short tube,
    /// `AR = 2 (0.0117)/0.03 = 0.78`: `x2 = AR sqrt(3) = 1.351000`, past the formula's `AR beta >= 1` bound, so at Mach 2 the CP
    /// is `(x2 - 0.67)/(2 x2 - 1) = 0.400117`. Mach 1.2 is `t = 0.7/1.5` along the quintic with `delta = 0.150117` and the
    /// endpoint slope `x2 0.34/(2 x2 - 1)^2 = 0.158568` (ratio 1.056, under 5/3): `0.25 + (10 delta - 6 s) t^2 +
    /// (-20 delta + 14 s) t^3 + (15 delta - 11 s) t^4 + (3 s - 4 delta) t^5 = 0.311521`.
    /// A tube of `AR = 2 (0.011)/0.04 = 0.55` has `x2 = 0.952628`, inside the Hermite bridge from 0.84 to 1: 0.305133 at Mach 2,
    /// and its endpoint slope ratio 11.27 takes the shape-preserving branch: `0.25 + delta B(t)^(ratio 3/5)` with the limiting
    /// quintic `B(t) = t^3 (10/3 - 10/3 t + t^2)`, 0.250001 at Mach 1.2.
    #[test]
    fn centre_of_pressure_moves_with_mach_by_the_hand_values() {
        let short = TubeFinSet { length: 0.03, ..set() };
        assert!((cp_fraction(&short, 0.3) - 0.25).abs() < 1e-12);
        assert!((TubeFinCalc::new(&short).cp(0.3) - 1.0075).abs() < 1e-12);
        assert!((cp_fraction(&short, 1.2) - 0.311_520_773).abs() < 1e-8, "{}", cp_fraction(&short, 1.2));
        assert!((cp_fraction(&short, 2.0) - 0.400_117_465).abs() < 1e-8, "{}", cp_fraction(&short, 2.0));
        assert!((TubeFinCalc::new(&short).cp(2.0) - 1.012_003_524).abs() < 1e-8);
        let bridged = TubeFinSet { length: 0.04, inner_radius: 0.011, ..set() };
        assert!((cp_fraction(&bridged, 0.3) - 0.25).abs() < 1e-12);
        assert!((cp_fraction(&bridged, 1.2) - 0.250_001_142).abs() < 1e-8, "{}", cp_fraction(&bridged, 1.2));
        assert!((cp_fraction(&bridged, 2.0) - 0.305_133_187).abs() < 1e-8, "{}", cp_fraction(&bridged, 2.0));
    }

    /// The tubes of the *Tube fin rocket* comparison case: six tubes of `r_in = 12.065 mm`, `L = 76.2 mm` (`AR = 0.3167`,
    /// `AR sqrt(3) = 0.548`, under the 0.84 bridge start). The CP stays at the quarter chord until `AR beta = 0.84` (Mach 2.83),
    /// then moves aft along the bridge; the unbridged formula would put it at `(0.548 - 0.67)/(1.097 - 1) = -1.245` chords,
    /// ahead of the tube, at Mach 2.
    #[test]
    fn long_tube_keeps_its_cp_at_the_quarter_chord() {
        let t = TubeFinSet {
            count: 6,
            station: 0.500_888,
            length: 0.0762,
            outer_radius: 0.012_395_2,
            inner_radius: 0.012_065,
            body_radius: 0.012_395_2,
            gap: 0.0,
            roughness: 60e-6,
        };
        for mach in [0.3, 0.8, 1.0, 1.2, 1.5, 2.0, 2.8] {
            assert!((cp_fraction(&t, mach) - 0.25).abs() < 1e-12, "M {mach}: {}", cp_fraction(&t, mach));
        }
        assert!(cp_fraction(&t, 3.0) > 0.26, "{}", cp_fraction(&t, 3.0));
    }

    /// The CP is continuous in Mach number (no jump at Mach 0.5 or 2, no pole at `AR beta = 0.5`) and moves only aft through
    /// the transonic interpolation, for long, bridged, short and stubby tubes (`AR` 0.156, 0.55, 0.78, 2).
    #[test]
    fn centre_of_pressure_is_continuous_and_monotone_in_mach() {
        let tubes =
            [set(), TubeFinSet { length: 0.04, inner_radius: 0.011, ..set() }, TubeFinSet { length: 0.03, ..set() }, TubeFinSet { length: 0.0117, ..set() }];
        for t in &tubes {
            for edge in [0.5, 2.0] {
                let (below, above) = (cp_fraction(t, edge - 1e-9), cp_fraction(t, edge + 1e-9));
                assert!((below - above).abs() < 1e-7, "AR {}: jump at M {edge}: {below} -> {above}", 2.0 * t.inner_radius / t.length);
            }
            let mut prev = cp_fraction(t, 0.3);
            for i in 1..=2700 {
                let mach = 0.3 + f64::from(i) * 1e-3;
                let x = cp_fraction(t, mach);
                assert!(x.is_finite() && (0.25..0.5).contains(&x), "M {mach}: {x}");
                assert!((x - prev).abs() < 1e-3, "M {mach}: {prev} -> {x}");
                if mach <= 2.0 {
                    assert!(x >= prev - 1e-12, "M {mach}: CP moved forward, {prev} -> {x}");
                }
                prev = x;
            }
        }
    }

    /// Interstice and wetted area by the tangent geometry: d = 0.049187, theta1 = 1.318116, theta2 = 0.252680,
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

    /// Pressure and friction drag by hand at Mach 0.3, V = 102 m/s, nu = 1.4607e-5 m^2/s: stagnation
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

    /// Mach 2, V = 680.588 m/s: stagnation `0.85 (1.84 - 0.76/4 + 0.166/16 + 0.035/64) = 1.411784`, base `0.25/2 = 0.125`,
    /// Re(bore) = 1.09028e6, Swamee-Jain f = 0.025256, `f L / D = 0.161897`. Per tube: bore 0.015267, annular front face
    /// 0.018082, interstice 0.015259, so three tubes 0.145826. The bore keeps its subsonic pipe-flow form; dividing `f` by
    /// `beta = sqrt(M^2 - 1)` would give 0.126468 here and diverge at Mach 1.
    #[test]
    fn supersonic_drag_has_no_prandtl_glauert_factor_on_the_bore() {
        let calc = TubeFinCalc::new(&set());
        let stag: f64 = 0.85 * (1.84 - 0.76 / 4.0 + 0.166 / 16.0 + 0.035 / 64.0);
        assert!((stag - 1.411_784).abs() < 1e-6);
        let pressure = calc.pressure_cd(680.588 / 1.4607e-5, stag, 0.125, A_REF);
        assert!((pressure - 0.145_826).abs() < 2e-5, "{pressure}");
    }
}
