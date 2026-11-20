//! Slope limiters, the shock sensor and the low-Mach floor shared by the 2D face kernels ([`crate::flux`]) and the 3D ones ([`crate::flux3`], the implicit
//! solvers, the GPU kernels): the pieces that do not depend on the dimension.

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Limiter {
    /// Piecewise-constant (first order).
    FirstOrder,
    Minmod,
    VanAlbada,
    /// Venkatakrishnan (1D MUSCL form of the unstructured limiter, Ahmad et al. 2023 Eq. 11): see [`limited_slope`].
    Venkatakrishnan,
}

/// Limited cell slope `phi(a, b)` with `a = q_i - q_{i-1}`, `b = q_{i+1} - q_i`.
/// Face values are `q_i +- phi/2`.
#[inline]
pub fn limited_slope(a: f64, b: f64, lim: Limiter) -> f64 {
    match lim {
        Limiter::FirstOrder => 0.0,
        Limiter::Minmod => {
            if a * b <= 0.0 {
                0.0
            } else if a.abs() < b.abs() {
                a
            } else {
                b
            }
        }
        Limiter::VanAlbada => {
            // smooth van Albada: (a(b^2+e) + b(a^2+e)) / (a^2+b^2+2e); == a when a == b
            const EPS: f64 = 1e-14;
            if a * b <= 0.0 {
                0.0
            } else {
                (a * (b * b + EPS) + b * (a * a + EPS)) / (a * a + b * b + 2.0 * EPS)
            }
        }
        Limiter::Venkatakrishnan => {
            // Venkatakrishnan phi(y) = (y^2 + 2y) / (y^2 + y + 2) with unlimited face delta D = b/2 and neighbour delta A = a (y = A/D = 2a/b):
            // slope = 2 b (a^2 + a b + e) / (2 a^2 + a b + b^2 + 2 e); == a when a == b, <= b and <= 2a (so <= 2 min(a, b)), smooth, monotone in a/b
            const EPS: f64 = 1e-14;
            if a * b <= 0.0 {
                0.0
            } else {
                2.0 * b * (a * a + a * b + EPS) / (2.0 * a * a + a * b + b * b + 2.0 * EPS)
            }
        }
    }
}

/// Shock-sensor lower/upper threshold on the relative cell-to-cell pressure jump
/// `|p_R - p_L| / min(p_R, p_L)` (see [`shock_weight`]).
pub const SHOCK_SENSOR_LO: f64 = 0.002;
pub const SHOCK_SENSOR_HI: f64 = 0.02;

/// Weight `theta in [0, 1]` of the Rusanov flux in [`face_flux`]: smoothstep of the relative pressure jump
/// between the two cells adjacent to the face (Quirk-type shock sensor). `0` in smooth flow (pure HLLC),
/// `1` inside a captured shock (pure Rusanov); C1-continuous so the steady state stays a smooth fixed point.
#[inline]
pub fn shock_weight(pl: f64, pr: f64) -> f64 {
    let s = (pr - pl).abs() / pl.min(pr);
    let t = ((s - SHOCK_SENSOR_LO) / (SHOCK_SENSOR_HI - SHOCK_SENSOR_LO)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

/// Lower bound of the low-Mach scaling factor `z` in [`low_mach_states`].
pub const LOW_MACH_FLOOR: f64 = 0.05;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn limiters_are_consistent() {
        for lim in [Limiter::Minmod, Limiter::VanAlbada, Limiter::Venkatakrishnan] {
            assert!((limited_slope(0.7, 0.7, lim) - 0.7).abs() < 1e-12);
            assert_eq!(limited_slope(1.0, -1.0, lim), 0.0);
        }
        assert_eq!(limited_slope(1.0, 3.0, Limiter::Minmod), 1.0);
    }

    #[test]
    fn venkatakrishnan_properties() {
        let v = |a, b| limited_slope(a, b, Limiter::Venkatakrishnan);
        // zero at extrema / sign changes
        assert_eq!(v(1.0, -1.0), 0.0);
        assert_eq!(v(-0.3, 2.0), 0.0);
        assert_eq!(v(0.0, 1.0), 0.0);
        // exact slope for a == b, closed form 2 a b (a + b) / (2 a^2 + a b + b^2)
        for a in [1e-3, 0.7, 40.0, -2.5] {
            assert!((v(a, a) - a).abs() < 1e-12 * a.abs().max(1.0));
        }
        // bounded by 2 min(|a|, |b|); monotone nondecreasing in r = a / b (b = 1) up to the symmetric point r = 1 (phi(y) overshoots 1 by < 10 % for
        // y = 2 r > 2, as in the paper, and returns to 1 as r -> inf)
        let mut prev = 0.0;
        for k in 1..400 {
            let r = k as f64 * 0.05;
            let s = v(r, 1.0);
            assert!(s <= 2.0 * r.min(1.0) + 1e-12 && s <= 1.10 && (r > 1.0 || s >= prev - 1e-15), "r = {r}");
            assert!((s - 2.0 * r * (r + 1.0) / (2.0 * r * r + r + 1.0)).abs() < 1e-9);
            prev = s;
        }
        // antisymmetric under sign flip
        assert!((v(-0.4, -1.3) + v(0.4, 1.3)).abs() < 1e-15);
    }
}
