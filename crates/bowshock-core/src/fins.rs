//! Empirical drag of a set of trapezoidal fins: skin friction and leading-edge / trailing-edge pressure drag after S. Niskanen,
//! *Development of an Open Source model rocket simulation software* (M.Sc. thesis, 2009), eqs. 3.78-3.85, 3.89-3.94 and B.1-B.2 (after
//! Hoerner, *Fluid-Dynamic Drag*), plus the Ackeret wave drag of a double wedge above Mach 1. Fin-body interference drag is neglected.

use crate::analysis::FinDrag;
use crate::atmosphere::AirState;
use crate::spec::{FinProfile, FinSpec};
use bowshock_reference::base_drag::base_drag_coefficient;
use std::f64::consts::PI;

/// Mean aerodynamic chord of a trapezoidal fin [m].
pub(crate) fn mean_aerodynamic_chord(root: f64, tip: f64) -> f64 {
    2.0 / 3.0 * (root + tip - root * tip / (root + tip))
}

/// Stagnation pressure over free-stream dynamic pressure `q_stag / q` (Niskanen 2009 eq. 3.85 with B.1-B.2).
fn stagnation_pressure_ratio(mach: f64) -> f64 {
    let m2 = mach * mach;
    if mach < 1.0 {
        1.0 + m2 / 4.0 + m2 * m2 / 40.0
    } else {
        1.84 - 0.76 / m2 + 0.166 / (m2 * m2) + 0.035 / (m2 * m2 * m2)
    }
}

/// Pressure drag coefficient of a rounded leading edge, referenced to its frontal area (Niskanen 2009 eq. 3.89).
fn rounded_leading_edge(mach: f64) -> f64 {
    if mach < 0.9 {
        (1.0 - mach * mach).powf(-0.417) - 1.0
    } else if mach < 1.0 {
        1.0 - 1.785 * (mach - 0.9)
    } else {
        1.214 - 0.502 / (mach * mach) + 0.1095 / mach.powi(4)
    }
}

/// OpenRocket's compressibility factor of the roughness-limited skin friction (same as `Aero::drag` in `bowshock-sim`).
fn roughness_correction(mach: f64) -> f64 {
    if mach < 0.9 {
        1.0 - 0.1 * mach * mach
    } else if mach > 1.1 {
        1.0 / (1.0 + 0.18 * mach * mach)
    } else {
        let c1 = 1.0 - 0.1 * 0.81;
        let c2 = 1.0 / (1.0 + 0.18 * 1.21);
        c2 * (mach - 0.9) / 0.2 + c1 * (1.1 - mach) / 0.2
    }
}

/// Drag of the fin set `f` at Mach `mach` in air `air`, referenced to the area of a circle of diameter `d_ref` [m].
/// `body_length` [m] is the whole-rocket length, the reference length of OpenRocket's roughness-limited skin friction
/// `0.032 (eps / L)^0.2` (as in `Aero::drag` of `bowshock-sim`, which uses the rocket length, not the fin chord).
pub(crate) fn fin_drag(f: &FinSpec, mach: f64, air: &AirState, d_ref: f64, body_length: f64) -> FinDrag {
    let n = f64::from(f.count);
    let (cr, ct, t) = (f.root_chord, f.tip_chord, f.thickness);
    let a_fin = f.span * (cr + ct) / 2.0;
    let mac = mean_aerodynamic_chord(cr, ct);
    let sweep = (f.sweep / f.span).atan();
    let a_wet = 2.0 * n * a_fin;
    let a_front = n * t * f.span;
    let a_ref = PI * d_ref * d_ref / 4.0;

    // skin friction, fully turbulent smooth surface (eqs. 3.78-3.81), floored below by the roughness limit
    let re = air.density * mach * air.sound_speed * mac / air.viscosity;
    let cf = if re < 1e4 { 1.48e-2 } else { 1.0 / (1.50 * re.ln() - 5.6).powi(2) };
    let cf_c = if mach < 1.0 { cf * (1.0 - 0.1 * mach * mach) } else { cf / (1.0 + 0.15 * mach * mach).powf(0.58) };
    // a rough surface floors the friction at the roughness-limited value
    let cf_c = if f.roughness > 0.0 { cf_c.max(0.032 * (f.roughness / body_length.max(1e-9)).powf(0.2) * roughness_correction(mach)) } else { cf_c };
    let friction = cf_c * (1.0 + 2.0 * t / mac) * a_wet / a_ref;

    // leading- and trailing-edge pressure drag (eqs. 3.85-3.94)
    let cd_stag = 0.85 * stagnation_pressure_ratio(mach);
    let le_round = rounded_leading_edge(mach);
    let cd_base = base_drag_coefficient(mach);
    let (le_perp, te) = match &f.profile {
        FinProfile::Square => (cd_stag, cd_base),
        FinProfile::Rounded => (le_round, 0.5 * cd_base),
        FinProfile::Airfoil => (le_round, 0.0),
        FinProfile::DoubleWedge => (0.0, 0.0),
        FinProfile::Custom { section } => {
            let (b, e, rounded) = section_edges(section);
            (b * cd_stag + (1.0 - b) * if rounded { le_round } else { 0.0 }, e * cd_base)
        }
    };
    let mut pressure = (le_perp * sweep.cos().powi(2) + te) * a_front / a_ref;
    let wave = match &f.profile {
        FinProfile::DoubleWedge => Some(4.0 * (t / mac).powi(2)),
        FinProfile::Custom { section } => Some(4.0 * section_slope_integral(section) * (t / mac / 2.0).powi(2)),
        _ => None,
    };
    if let (Some(w), true) = (wave, mach >= 1.0) {
        // Ackeret wave drag, beta floored at M = 1.15
        let beta = (mach * mach - 1.0).max(0.3225).sqrt();
        pressure += w / beta * n * a_fin / a_ref;
    }
    FinDrag { friction, pressure }
}

/// Normalised section `zeta = z / z_max` at the given points.
fn normalised(section: &[[f64; 2]]) -> impl Iterator<Item = [f64; 2]> + '_ {
    let zmax = section.iter().fold(0.0f64, |m, p| m.max(p[1]));
    section.iter().map(move |p| [p[0], p[1] / zmax])
}

/// Blunt leading-edge fraction `zeta(0)`, trailing-edge fraction `zeta(1)` and the leading-edge class of a custom section.
/// The edge is `rounded` when the slope of the first segment is at least 1.5 times that of the second (a sqrt-like nose;
/// a wedge gives 1), or when the second slope is zero while the first is positive. A negative second slope (thickness peaking at
/// the first interior point, as for a double wedge) is a sharp edge. Heuristic.
fn section_edges(section: &[[f64; 2]]) -> (f64, f64, bool) {
    let z: Vec<[f64; 2]> = normalised(section).collect();
    let slope = |k: usize| (z[k][1] - z[k - 1][1]) / (z[k][0] - z[k - 1][0]);
    let (s1, s2) = (slope(1), slope(2));
    let rounded = if s2 < 0.0 {
        false
    } else if s2 == 0.0 {
        s1 > 0.0
    } else {
        s1 / s2 >= 1.5
    };
    (z[0][1], z[z.len() - 1][1], rounded)
}

/// Integral over `0..1` of the squared slope of the normalised section, exact over the linear segments (the blunt leading-edge step is excluded).
fn section_slope_integral(section: &[[f64; 2]]) -> f64 {
    let z: Vec<[f64; 2]> = normalised(section).collect();
    z.windows(2).map(|w| (w[1][1] - w[0][1]).powi(2) / (w[1][0] - w[0][0])).sum()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::atmosphere::Atmosphere;
    use crate::error::Error;

    fn fin(profile: FinProfile) -> FinSpec {
        FinSpec { count: 4, root_chord: 0.12, tip_chord: 0.06, span: 0.08, sweep: 0.06, thickness: 0.003, roughness: 0.0, profile }
    }

    #[test]
    fn rounded_leading_edge_is_continuous() {
        // the published subsonic branch meets the transonic line at M = 0.9 to 1.3e-3 ((0.19)^-0.417 - 1 = 0.99870)
        assert!((rounded_leading_edge(0.9 - 1e-12) - rounded_leading_edge(0.9)).abs() < 1.5e-3);
        assert!((rounded_leading_edge(1.0 - 1e-12) - rounded_leading_edge(1.0)).abs() < 1e-3);
    }

    #[test]
    fn stagnation_pressure_ratio_is_one_at_rest() {
        assert_eq!(stagnation_pressure_ratio(0.0), 1.0);
    }

    #[test]
    fn square_fins_at_mach_half_match_the_formulas() {
        let air = Atmosphere::Isa { altitude_m: 0.0 }.state().unwrap();
        let (m, d_ref) = (0.5, 0.1);
        let got = fin_drag(&fin(FinProfile::Square), m, &air, d_ref, 1.0);

        // independent evaluation: chords 0.12 / 0.06 m, span 0.08 m, sweep 0.06 m, thickness 3 mm, four fins
        let area = 0.08 * 0.09; // span * mean chord
        let mac = 2.0 / 3.0 * (0.18 - 0.0072 / 0.18);
        let a_ref = PI * 0.01 / 4.0;
        let re = air.density * 0.5 * air.sound_speed * mac / air.viscosity;
        let cf = 1.0 / (1.5 * re.ln() - 5.6).powi(2) * (1.0 - 0.1 * 0.25);
        let friction = cf * (1.0 + 0.006 / mac) * 8.0 * area / a_ref;
        let cos2 = 1.0 / (1.0 + (0.06f64 / 0.08).powi(2)); // cos^2(atan(0.75))
        let stag = 0.85 * (1.0 + 0.25 / 4.0 + 0.0625 / 40.0);
        let base = 0.12 + 0.13 * 0.25;
        let pressure = (stag * cos2 + base) * 4.0 * 0.003 * 0.08 / a_ref;

        assert!((got.friction - friction).abs() <= 1e-12 * friction, "{} vs {friction}", got.friction);
        assert!((got.pressure - pressure).abs() <= 1e-12 * pressure, "{} vs {pressure}", got.pressure);
    }

    #[test]
    fn double_wedge_has_wave_drag_only_above_mach_one() {
        let air = Atmosphere::Isa { altitude_m: 0.0 }.state().unwrap();
        let f = fin(FinProfile::DoubleWedge);
        assert_eq!(fin_drag(&f, 0.8, &air, 0.1, 1.0).pressure, 0.0);
        assert!(fin_drag(&f, 1.5, &air, 0.1, 1.0).pressure > 0.0);
    }

    #[test]
    fn invalid_fins_are_rejected_with_the_field_named() {
        type Edit = fn(&mut FinSpec);
        let cases: [(&str, Edit); 12] = [
            ("fins.roughness", |f| f.roughness = -1e-6),
            ("fins.roughness", |f| f.roughness = f64::NAN),
            ("fins.count", |f| f.count = 2),
            ("fins.count", |f| f.count = 9),
            ("fins.root_chord", |f| f.root_chord = 0.0),
            ("fins.root_chord", |f| f.root_chord = f64::NAN),
            ("fins.tip_chord", |f| f.tip_chord = -0.01),
            ("fins.span", |f| f.span = f64::INFINITY),
            ("fins.thickness", |f| f.thickness = -0.003),
            ("fins.sweep", |f| f.sweep = 0.4), // 78.7 deg
            ("fins.sweep", |f| f.sweep = f64::NAN),
            ("fins.thickness", |f| f.thickness = 0.03), // t / mac = 0.32
        ];
        for (field, edit) in cases {
            let mut f = fin(FinProfile::Rounded);
            edit(&mut f);
            match f.validate() {
                Err(Error::InvalidInput(msg)) => assert!(msg.starts_with(field), "expected `{field}`, got `{msg}`"),
                other => panic!("expected InvalidInput for {field}, got {other:?}"),
            }
        }
        assert!(fin(FinProfile::Rounded).validate().is_ok());
    }

    #[test]
    fn roughness_floors_the_friction_at_the_openrocket_limit() {
        let air = Atmosphere::Isa { altitude_m: 0.0 }.state().unwrap();
        let (d_ref, length) = (0.1, 0.3);
        for mach in [0.5, 1.0, 2.0] {
            let smooth = fin(FinProfile::Rounded);
            let rough = FinSpec { roughness: 60e-6, ..smooth.clone() };
            let (a, b) = (fin_drag(&smooth, mach, &air, d_ref, length), fin_drag(&rough, mach, &air, d_ref, length));
            assert_eq!(a.pressure, b.pressure);
            // friction scales with the skin-friction coefficient: recover it from the smooth value
            let limited = 0.032 * (60e-6f64 / length).powf(0.2) * roughness_correction(mach);
            let mac = mean_aerodynamic_chord(0.12, 0.06);
            let re = air.density * mach * air.sound_speed * mac / air.viscosity;
            let cf = 1.0 / (1.5 * re.ln() - 5.6).powi(2);
            let cf_c = if mach < 1.0 { cf * (1.0 - 0.1 * mach * mach) } else { cf / (1.0 + 0.15 * mach * mach).powf(0.58) };
            assert!(limited > cf_c, "test premise: the limit is active at Mach {mach}");
            let expected = a.friction * limited / cf_c;
            close(b.friction, expected, "rough friction");
        }
        // a roughness below the smooth friction changes nothing
        let fine = FinSpec { roughness: 1e-9, ..fin(FinProfile::Rounded) };
        assert_eq!(fin_drag(&fine, 0.5, &air, d_ref, length).friction, fin_drag(&fin(FinProfile::Rounded), 0.5, &air, d_ref, length).friction);
    }

    fn custom(section: &[[f64; 2]]) -> FinSpec {
        fin(FinProfile::Custom { section: section.to_vec() })
    }

    fn close(a: f64, b: f64, what: &str) {
        assert!((a - b).abs() <= 1e-12 * a.abs().max(b.abs()) + 1e-15, "{what}: {a} vs {b}");
    }

    #[test]
    fn custom_double_wedge_matches_double_wedge() {
        let air = Atmosphere::Isa { altitude_m: 0.0 }.state().unwrap();
        let c = custom(&[[0.0, 0.0], [0.5, 1.0], [1.0, 0.0]]);
        let d = fin(FinProfile::DoubleWedge);
        for m in [0.8, 2.0] {
            let (a, b) = (fin_drag(&c, m, &air, 0.1, 1.0), fin_drag(&d, m, &air, 0.1, 1.0));
            close(a.friction, b.friction, "friction");
            close(a.pressure, b.pressure, "pressure");
        }
    }

    #[test]
    fn custom_flat_plate_matches_square() {
        let air = Atmosphere::Isa { altitude_m: 0.0 }.state().unwrap();
        let c = custom(&[[0.0, 1.0], [0.5, 1.0], [1.0, 1.0]]);
        let s = fin(FinProfile::Square);
        for m in [0.5, 1.5] {
            let (a, b) = (fin_drag(&c, m, &air, 0.1, 1.0), fin_drag(&s, m, &air, 0.1, 1.0));
            close(a.friction, b.friction, "friction");
            close(a.pressure, b.pressure, "pressure");
        }
    }

    #[test]
    fn naca0012_has_a_rounded_leading_edge() {
        let xs = [0.0, 0.0025, 0.005, 0.0125, 0.025, 0.05, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0];
        let section: Vec<[f64; 2]> =
            xs.iter().map(|&x: &f64| [x, 0.6 * (0.2969 * x.sqrt() - 0.1260 * x - 0.3516 * x * x + 0.2843 * x.powi(3) - 0.1015 * x.powi(4))]).collect();
        let (b, e, rounded) = section_edges(&section);
        assert!(rounded && b == 0.0 && e > 0.0 && e < 0.05, "b {b} e {e} rounded {rounded}");
        let air = Atmosphere::Isa { altitude_m: 0.0 }.state().unwrap();
        let (n, a) = (fin(FinProfile::Custom { section }), fin(FinProfile::Airfoil));
        // the thin trailing edge (about 2 % of the thickness) adds TE * CD_base on top of the Airfoil leading edge
        let (pn, pa) = (fin_drag(&n, 0.9, &air, 0.1, 1.0).pressure, fin_drag(&a, 0.9, &air, 0.1, 1.0).pressure);
        assert!((pn - pa).abs() < 0.01 * pa, "{pn} vs {pa}");
        let p2 = fin_drag(&n, 2.0, &air, 0.1, 1.0).pressure;
        assert!(p2 > 0.0 && p2.is_finite());
    }

    #[test]
    fn invalid_custom_sections_are_rejected() {
        let bad: [&[[f64; 2]]; 6] = [
            &[[0.0, 0.0], [1.0, 0.0]],
            &[[0.1, 0.0], [0.5, 1.0], [1.0, 0.0]],
            &[[0.0, 0.0], [0.5, 1.0], [0.9, 0.0]],
            &[[0.0, 0.0], [0.5, 1.0], [0.5, 1.0], [1.0, 0.0]],
            &[[0.0, 0.0], [0.5, -1.0], [1.0, 0.0]],
            &[[0.0, 0.0], [0.5, 0.0], [1.0, 0.0]],
        ];
        for s in bad {
            match custom(s).validate() {
                Err(Error::InvalidInput(msg)) => assert!(msg.starts_with("fins.profile.section"), "{msg}"),
                other => panic!("expected InvalidInput for {s:?}, got {other:?}"),
            }
        }
        assert!(custom(&[[0.0, 0.0], [0.5, 1.0], [1.0, 0.0]]).validate().is_ok());
    }
}
