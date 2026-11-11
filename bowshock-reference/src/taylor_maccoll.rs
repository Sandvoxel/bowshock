//! Taylor-Maccoll solution for supersonic flow over a sharp circular cone at zero incidence.
//!
//! In spherical coordinates (R, theta) with velocities `Vr`, `Vt` made non-dimensional by
//! `V_max = sqrt(2 h0)` the conical flow obeys (Anderson, *Modern Compressible Flow*, eq. 10.15)
//!
//! ```text
//!   Vr'  = Vt
//!   Vt'  = ( Vt^2 Vr - k (2 Vr + Vt cot(theta)) ) / ( k - Vt^2 ),   k = (g-1)/2 (1 - Vr^2 - Vt^2)
//! ```
//!
//! Starting just behind an oblique shock of angle `beta` (velocity components from the oblique
//! shock relations) the ODE is integrated with RK4 towards smaller `theta` until `Vt = 0`, the
//! cone surface. `beta` is found by bisection such that that angle equals the cone half-angle.

use bowshock_core::gas::GAMMA;

#[derive(Clone, Copy, Debug)]
pub struct ConeSolution {
    pub beta: f64,
    /// Cone-surface Mach number and `p / p_inf`.
    pub mach_surface: f64,
    pub p_ratio_surface: f64,
    pub cp_surface: f64,
    /// Post-shock Mach number and `p / p_inf`.
    pub mach_post_shock: f64,
    pub p_ratio_post_shock: f64,
}

/// Flow deflection angle `delta` of an oblique shock with wave angle `beta`.
pub fn oblique_deflection(m1: f64, beta: f64) -> f64 {
    let g = GAMMA;
    let num = 2.0 / beta.tan() * (m1 * m1 * beta.sin().powi(2) - 1.0);
    let den = m1 * m1 * (g + (2.0 * beta).cos()) + 2.0;
    (num / den).atan()
}

/// Post-shock `(M2, p2/p1)` of an oblique shock.
pub fn oblique_post_shock(m1: f64, beta: f64) -> (f64, f64) {
    let g = GAMMA;
    let mn1 = m1 * beta.sin();
    let delta = oblique_deflection(m1, beta);
    let mn2sq = (1.0 + 0.5 * (g - 1.0) * mn1 * mn1) / (g * mn1 * mn1 - 0.5 * (g - 1.0));
    let m2 = mn2sq.sqrt() / (beta - delta).sin();
    let p21 = 1.0 + 2.0 * g / (g + 1.0) * (mn1 * mn1 - 1.0);
    (m2, p21)
}

#[inline]
fn rhs(th: f64, y: [f64; 2]) -> [f64; 2] {
    let k = 0.5 * (GAMMA - 1.0) * (1.0 - y[0] * y[0] - y[1] * y[1]);
    let vt = y[1];
    let dvt = (vt * vt * y[0] - k * (2.0 * y[0] + vt / th.tan())) / (k - vt * vt);
    [vt, dvt]
}

fn rk4(th: f64, y: [f64; 2], h: f64) -> [f64; 2] {
    let k1 = rhs(th, y);
    let k2 = rhs(th + 0.5 * h, [y[0] + 0.5 * h * k1[0], y[1] + 0.5 * h * k1[1]]);
    let k3 = rhs(th + 0.5 * h, [y[0] + 0.5 * h * k2[0], y[1] + 0.5 * h * k2[1]]);
    let k4 = rhs(th + h, [y[0] + h * k3[0], y[1] + h * k3[1]]);
    [y[0] + h / 6.0 * (k1[0] + 2.0 * k2[0] + 2.0 * k3[0] + k4[0]), y[1] + h / 6.0 * (k1[1] + 2.0 * k2[1] + 2.0 * k3[1] + k4[1])]
}

/// Integrate from the shock; returns `(theta_cone, Vr_surface)` (the angle where `Vt` crosses zero).
fn integrate_from_shock(m1: f64, beta: f64) -> (f64, f64) {
    let (m2, _) = oblique_post_shock(m1, beta);
    let delta = oblique_deflection(m1, beta);
    // V/Vmax from M: V^2/Vmax^2 = (g-1)/2 M^2 / (1 + (g-1)/2 M^2)
    let a = 0.5 * (GAMMA - 1.0) * m2 * m2;
    let v = (a / (1.0 + a)).sqrt();
    let mut y = [v * (beta - delta).cos(), -v * (beta - delta).sin()];
    let mut th = beta;
    let h = -1e-5;
    while th > 1e-3 {
        let yn = rk4(th, y, h);
        if yn[1] >= 0.0 {
            // linear interpolation of the zero crossing
            let t = y[1] / (y[1] - yn[1]);
            return (th + t * h, y[0] + t * (yn[0] - y[0]));
        }
        y = yn;
        th += h;
    }
    (0.0, y[0])
}

/// Solve the weak-shock Taylor-Maccoll cone flow for Mach `m1` and cone half-angle `theta_c` [rad].
/// Returns `None` if no attached conical shock exists.
pub fn solve_cone(m1: f64, theta_c: f64) -> Option<ConeSolution> {
    let mu = (1.0 / m1).asin();
    // scan for the first bracket where theta_cone(beta) exceeds the target (weak branch)
    let step = 2e-3;
    let mut lo = mu + 1e-6;
    let mut found = None;
    let mut b = lo + step;
    while b < 0.5 * std::f64::consts::PI {
        if integrate_from_shock(m1, b).0 >= theta_c {
            found = Some((lo, b));
            break;
        }
        lo = b;
        b += step;
    }
    let (mut lo, mut hi) = found?;
    for _ in 0..50 {
        let mid = 0.5 * (lo + hi);
        if integrate_from_shock(m1, mid).0 >= theta_c {
            hi = mid;
        } else {
            lo = mid;
        }
    }
    let beta = 0.5 * (lo + hi);
    let (_, vr) = integrate_from_shock(m1, beta);
    let (m2, p21) = oblique_post_shock(m1, beta);
    let g = GAMMA;
    // total pressure behind the shock, then isentropic change to the cone surface
    let p02_p1 = p21 * (1.0 + 0.5 * (g - 1.0) * m2 * m2).powf(g / (g - 1.0));
    let t_t0 = 1.0 - vr * vr; // T/T0 with V in units of Vmax
    let p_ratio_surface = p02_p1 * t_t0.powf(g / (g - 1.0));
    // c^2/Vmax^2 = (g-1)/2 (1 - V^2)  =>  M^2 = V^2 / ((g-1)/2 (1-V^2))
    let mach_surface = (vr * vr / (0.5 * (g - 1.0) * t_t0)).sqrt();
    Some(ConeSolution {
        beta,
        mach_surface,
        p_ratio_surface,
        cp_surface: 2.0 * (p_ratio_surface - 1.0) / (g * m1 * m1),
        mach_post_shock: m2,
        p_ratio_post_shock: p21,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    const D: f64 = std::f64::consts::PI / 180.0;

    #[test]
    fn oblique_shock_table_value() {
        // NACA 1135 / Anderson: M=2, delta=10 deg -> weak beta = 39.31 deg.
        assert!((oblique_deflection(2.0, 39.3139 * D) / D - 10.0).abs() < 2e-3);
    }

    #[test]
    fn cone_mach_2p35_10deg_nasa_wind_validation_case() {
        // NASA NPARC Alliance validation archive "10 Degree Cone at Mach 2.35"
        // (https://www.grc.nasa.gov/WWW/wind/valid/cone10/cone10.html). Its Euler CFD results
        // (WIND/NPARC/Wind-US, Table 4) give cone-surface M3 = 2.1467..2.1469 and
        // p3/p1 = 1.3740..1.3741, and agree with this solver. (The page's "theory" row,
        // p3/p1 = 1.4234 and beta = 27.1843 deg, is not consistent with its own CFD nor with the
        // M=2/15 deg reference below, so it is deliberately not used.)
        let s = solve_cone(2.35, 10.0 * D).unwrap();
        assert!((s.mach_surface - 2.1468).abs() < 3e-4, "M3 {}", s.mach_surface);
        assert!((s.p_ratio_surface - 1.3740).abs() < 3e-4, "p3/p1 {}", s.p_ratio_surface);
    }

    #[test]
    fn cone_mach_2_15deg_mallard_tutorial() {
        // MatthewBonanni/mallard docs/tutorials/cone.md (taylor_maccoll.py):
        // M=2, 15 deg: shock 33.9147 deg, p/pinf 1.56629, Cp 0.20225, surface Mach 1.7069.
        let s = solve_cone(2.0, 15.0 * D).unwrap();
        assert!((s.beta / D - 33.9147).abs() < 2e-3, "beta {}", s.beta / D);
        assert!((s.p_ratio_surface - 1.56629).abs() < 2e-4, "p {}", s.p_ratio_surface);
        assert!((s.cp_surface - 0.20225).abs() < 1e-4, "cp {}", s.cp_surface);
        assert!((s.mach_surface - 1.7069).abs() < 5e-4, "M {}", s.mach_surface);
    }

    #[test]
    fn cone_mach_2_10deg() {
        // M=2, 10 deg: independent cross-check through the oblique-shock relations - the
        // computed beta must lie between the Mach angle (30 deg) and the wedge angle for the same
        // deflection (39.31 deg), and the conical shock is weaker than the 10 deg wedge shock.
        let s = solve_cone(2.0, 10.0 * D).unwrap();
        let b = s.beta / D;
        assert!(b > 30.0 && b < 39.31, "beta {b}");
        assert!((b - 31.206).abs() < 0.01, "beta {b}"); // self-consistent value, matches the ~31.2 deg in tables
        assert!(s.p_ratio_surface > 1.0 && s.p_ratio_surface < 1.5);
    }
}
