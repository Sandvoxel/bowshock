//! Empirical drag of a set of trapezoidal fins: skin friction and leading-edge / trailing-edge pressure drag after S. Niskanen,
//! *OpenRocket technical documentation* (2013), eqs. 3.78-3.85, 3.89-3.94 and B.1-B.2, plus the Ackeret wave drag of a double
//! wedge above Mach 1. Fin-body interference drag is neglected, as in OpenRocket.

use crate::analysis::FinDrag;
use crate::atmosphere::AirState;
use crate::spec::{FinProfile, FinSpec};
use bowshock_reference::base_drag::base_drag_coefficient;
use std::f64::consts::PI;

/// Mean aerodynamic chord of a trapezoidal fin [m].
pub(crate) fn mean_aerodynamic_chord(root: f64, tip: f64) -> f64 {
    2.0 / 3.0 * (root + tip - root * tip / (root + tip))
}

/// Stagnation pressure over free-stream dynamic pressure `q_stag / q` (OpenRocket eq. 3.85 with B.1-B.2).
fn stagnation_pressure_ratio(mach: f64) -> f64 {
    let m2 = mach * mach;
    if mach < 1.0 {
        1.0 + m2 / 4.0 + m2 * m2 / 40.0
    } else {
        1.84 - 0.76 / m2 + 0.166 / (m2 * m2) + 0.035 / (m2 * m2 * m2)
    }
}

/// Pressure drag coefficient of a rounded leading edge, referenced to its frontal area (OpenRocket eq. 3.89).
fn rounded_leading_edge(mach: f64) -> f64 {
    if mach < 0.9 {
        (1.0 - mach * mach).powf(-0.417) - 1.0
    } else if mach < 1.0 {
        1.0 - 1.785 * (mach - 0.9)
    } else {
        1.214 - 0.502 / (mach * mach) + 0.1095 / mach.powi(4)
    }
}

/// Drag of the fin set `f` at Mach `mach` in air `air`, referenced to the area of a circle of diameter `d_ref` [m].
pub(crate) fn fin_drag(f: &FinSpec, mach: f64, air: &AirState, d_ref: f64) -> FinDrag {
    let n = f64::from(f.count);
    let (cr, ct, t) = (f.root_chord, f.tip_chord, f.thickness);
    let a_fin = f.span * (cr + ct) / 2.0;
    let mac = mean_aerodynamic_chord(cr, ct);
    let sweep = (f.sweep / f.span).atan();
    let a_wet = 2.0 * n * a_fin;
    let a_front = n * t * f.span;
    let a_ref = PI * d_ref * d_ref / 4.0;

    // skin friction, fully turbulent smooth surface (eqs. 3.78-3.81)
    let re = air.density * mach * air.sound_speed * mac / air.viscosity;
    let cf = if re < 1e4 { 1.48e-2 } else { 1.0 / (1.50 * re.ln() - 5.6).powi(2) };
    let cf_c = if mach < 1.0 { cf * (1.0 - 0.1 * mach * mach) } else { cf / (1.0 + 0.15 * mach * mach).powf(0.58) };
    let friction = cf_c * (1.0 + 2.0 * t / mac) * a_wet / a_ref;

    // leading- and trailing-edge pressure drag (eqs. 3.85-3.94)
    let cd_stag = 0.85 * stagnation_pressure_ratio(mach);
    let le_round = rounded_leading_edge(mach);
    let cd_base = base_drag_coefficient(mach);
    let (le_perp, te) = match f.profile {
        FinProfile::Square => (cd_stag, cd_base),
        FinProfile::Rounded => (le_round, 0.5 * cd_base),
        FinProfile::Airfoil => (le_round, 0.0),
        FinProfile::DoubleWedge => (0.0, 0.0),
    };
    let mut pressure = (le_perp * sweep.cos().powi(2) + te) * a_front / a_ref;
    if f.profile == FinProfile::DoubleWedge && mach >= 1.0 {
        // Ackeret wave drag of a double wedge, beta floored at M = 1.15
        let beta = (mach * mach - 1.0).max(0.3225).sqrt();
        pressure += 4.0 * (t / mac).powi(2) / beta * n * a_fin / a_ref;
    }
    FinDrag { friction, pressure }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::atmosphere::Atmosphere;
    use crate::error::Error;

    fn fin(profile: FinProfile) -> FinSpec {
        FinSpec { count: 4, root_chord: 0.12, tip_chord: 0.06, span: 0.08, sweep: 0.06, thickness: 0.003, profile }
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
        let got = fin_drag(&fin(FinProfile::Square), m, &air, d_ref);

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
        assert_eq!(fin_drag(&f, 0.8, &air, 0.1).pressure, 0.0);
        assert!(fin_drag(&f, 1.5, &air, 0.1).pressure > 0.0);
    }

    #[test]
    fn invalid_fins_are_rejected_with_the_field_named() {
        type Edit = fn(&mut FinSpec);
        let cases: [(&str, Edit); 10] = [
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
}
