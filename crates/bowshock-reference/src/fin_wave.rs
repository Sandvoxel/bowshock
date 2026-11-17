//! Wave drag of a thin symmetric fin section at zero incidence in supersonic flow, from Ackeret's linearised theory (J. Ackeret,
//! *Über Luftkräfte bei sehr grossen Geschwindigkeiten insbesondere bei ebenen Strömungen*, Helvetica Physica Acta 1, 1925; the
//! thin-airfoil result as in Liepmann and Roshko, *Elements of Gasdynamics*, 1957, ch. 10).
//!
//! For a section of chord `c` with upper and lower surfaces `z_u(x)` and `z_l(x) = -z_u(x)` the wave drag coefficient on the
//! planform area is
//!
//! `cd_w = (2 / beta) (1 / c) integral over the chord of [(dz_u/dx)^2 + (dz_l/dx)^2] dx`, `beta = sqrt(M^2 - 1)`.
//!
//! With the half thickness `z_u = (t / 2) zeta(xi)`, `xi = x / c`, normalised so that the maximum of `zeta` is 1, and
//! `tau = t / c`, this is `cd_w = S tau^2 / beta` with the shape factor `S = integral over 0..1 of (d zeta / d xi)^2 d xi`
//! (the `2 / beta` times the two surfaces `2` times `(tau / 2)^2`).

/// The Mach number below which `beta = sqrt(M^2 - 1)` is floored: `beta^2` is held at 0.3225 (the value at Mach 1.15), so the
/// `1 / beta` of the linear theory stays finite towards Mach 1. The same treatment serves every section.
const BETA_SQUARED_FLOOR: f64 = 0.3225;

/// Shape factor `S` of a double wedge: `zeta = 2 xi` forward of mid-chord and `2 (1 - xi)` aft, so `S = 2 * 2^2 * (1/2) = 4`.
pub fn double_wedge_slope_integral() -> f64 {
    4.0
}

/// Shape factor `S` of a hexagonal (modified double wedge) section with a wedge of chordwise length `wedge_fraction * c` at the
/// leading and at the trailing edge and a flat mid-section: slope `1 / f` over `f` at each end, so `S = 2 (1/f)^2 f = 2 / f`.
/// It equals the double wedge's 4 at `f = 0.5`. Valid for `0 < f <= 0.5`.
pub fn hexagonal_slope_integral(wedge_fraction: f64) -> f64 {
    2.0 / wedge_fraction
}

/// Shape factor `S` of a biconvex (circular-arc, parabolic in the thin limit) section: `zeta = 4 xi (1 - xi)`, slope
/// `4 (1 - 2 xi)`, so `S = 16 * integral (1 - 2 xi)^2 d xi = 16 / 3`.
pub fn biconvex_slope_integral() -> f64 {
    16.0 / 3.0
}

/// Ackeret wave drag coefficient `S tau^2 / beta` of a fin section with shape factor `slope_integral` and thickness ratio `tau`
/// (thickness over chord) at Mach `mach`, referenced to the fin planform area. Zero below Mach 1; `beta` is floored at its value
/// at Mach 1.15.
pub fn wave_drag_coefficient(slope_integral: f64, tau: f64, mach: f64) -> f64 {
    if mach < 1.0 {
        return 0.0;
    }
    let beta = (mach * mach - 1.0).max(BETA_SQUARED_FLOOR).sqrt();
    slope_integral * tau * tau / beta
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Midpoint-rule integral of the squared slope of the half-thickness `zeta` (peak 1), by a central difference.
    fn numeric_slope_integral(zeta: impl Fn(f64) -> f64) -> f64 {
        let n = 200_000;
        let h = 1.0 / n as f64;
        (0..n)
            .map(|i| {
                let x = (i as f64 + 0.5) * h;
                let d = (zeta(x + 0.25 * h) - zeta(x - 0.25 * h)) / (0.5 * h);
                d * d * h
            })
            .sum()
    }

    #[test]
    fn closed_forms_match_numerical_integration_at_mach_two() {
        let (tau, mach) = (0.05, 2.0);
        let beta = (mach * mach - 1.0f64).sqrt();
        let hex = |f: f64| move |x: f64| (x / f).min(1.0).min((1.0 - x) / f);
        let cases: [(&str, f64, Box<dyn Fn(f64) -> f64>, f64); 4] = [
            ("double wedge", double_wedge_slope_integral(), Box::new(|x: f64| 2.0 * x.min(1.0 - x)), 4.0 * tau * tau / beta),
            ("hexagonal 0.25", hexagonal_slope_integral(0.25), Box::new(hex(0.25)), 2.0 * tau * tau / (0.25 * beta)),
            ("hexagonal 0.1", hexagonal_slope_integral(0.1), Box::new(hex(0.1)), 2.0 * tau * tau / (0.1 * beta)),
            ("biconvex", biconvex_slope_integral(), Box::new(|x: f64| 4.0 * x * (1.0 - x)), 16.0 / 3.0 * tau * tau / beta),
        ];
        for (name, s, zeta, closed) in cases {
            let direct = 2.0 / beta * 2.0 * (tau / 2.0).powi(2) * numeric_slope_integral(zeta);
            let got = wave_drag_coefficient(s, tau, mach);
            assert!((got - closed).abs() <= 1e-12 * closed, "{name}: {got} vs closed form {closed}");
            assert!((got - direct).abs() <= 1e-6 * direct, "{name}: {got} vs numerical {direct}");
        }
    }

    #[test]
    fn hexagonal_at_half_chord_is_the_double_wedge() {
        assert_eq!(hexagonal_slope_integral(0.5), double_wedge_slope_integral());
    }

    #[test]
    fn no_wave_drag_below_mach_one() {
        for m in [0.0, 0.5, 0.99] {
            assert_eq!(wave_drag_coefficient(4.0, 0.05, m), 0.0);
        }
        assert!(wave_drag_coefficient(4.0, 0.05, 1.0).is_finite());
    }
}
