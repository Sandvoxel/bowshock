//! Independent surface-pressure theory for slender bodies of revolution (no CFD involved):
//!
//! * **tangent cone**: local Cp of the Taylor-Maccoll cone whose half angle equals the local surface slope;
//! * **shock-expansion**: Taylor-Maccoll cone flow at the (conical) nose tip, then Prandtl-Meyer expansion
//!   along the surface through the turning angle `theta_tip - theta_local` (isentropic, surface streamline
//!   only; shock curvature/vorticity and the axisymmetric relief are ignored).
//!
//! Both are classical first estimates; neither is exact on a curved ogive.

use crate::gas::GAMMA;
use crate::taylor_maccoll::{solve_cone, ConeSolution};

/// Prandtl-Meyer function `nu(M)` [rad].
pub fn prandtl_meyer(m: f64) -> f64 {
    let g = GAMMA;
    let k = ((g + 1.0) / (g - 1.0)).sqrt();
    let b = (m * m - 1.0).max(0.0).sqrt();
    k * (b / k).atan() - b.atan()
}

/// Inverse of [`prandtl_meyer`] by bisection.
pub fn mach_from_pm(nu: f64) -> f64 {
    let (mut lo, mut hi) = (1.0f64, 200.0f64);
    for _ in 0..200 {
        let mid = 0.5 * (lo + hi);
        if prandtl_meyer(mid) < nu {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    0.5 * (lo + hi)
}

/// `p/p_inf` -> `Cp` at free-stream Mach `m`.
pub fn cp_from_ratio(p_ratio: f64, m: f64) -> f64 {
    2.0 * (p_ratio - 1.0) / (GAMMA * m * m)
}

/// Tangent-ogive geometry. Lengths are in units of the body diameter `d = 1` unless noted.
#[derive(Clone, Copy, Debug)]
pub struct Ogive {
    /// Nose length / diameter.
    pub fineness: f64,
}

impl Ogive {
    pub fn radius(&self) -> f64 {
        0.5
    }
    pub fn nose_length(&self) -> f64 {
        self.fineness
    }
    /// Ogive arc radius `rho = (L^2 + R^2) / (2R)`.
    pub fn rho(&self) -> f64 {
        let (l, r) = (self.fineness, self.radius());
        (l * l + r * r) / (2.0 * r)
    }
    /// Body radius at `x` (cylinder for `x >= L`).
    pub fn r(&self, x: f64) -> f64 {
        let l = self.fineness;
        if x >= l {
            return self.radius();
        }
        let x = x.max(0.0);
        let rho = self.rho();
        (rho * rho - (l - x).powi(2)).max(0.0).sqrt() - (rho - self.radius())
    }
    /// Surface slope `dr/dx`.
    pub fn slope(&self, x: f64) -> f64 {
        let l = self.fineness;
        if x >= l {
            return 0.0;
        }
        let x = x.max(0.0);
        let rho = self.rho();
        (l - x) / (rho * rho - (l - x).powi(2)).max(1e-300).sqrt()
    }
    /// Half angle of the cone tangent to the nose tip [rad].
    pub fn tip_half_angle(&self) -> f64 {
        self.slope(0.0).atan()
    }
}

/// Theory evaluator at one Mach number.
pub struct SurfaceTheory {
    pub mach: f64,
    pub body: Ogive,
    /// Taylor-Maccoll solution of the tip cone (None if the shock would detach).
    pub tip: Option<ConeSolution>,
}

impl SurfaceTheory {
    pub fn new(mach: f64, body: Ogive) -> Self {
        SurfaceTheory { mach, body, tip: solve_cone(mach, body.tip_half_angle()) }
    }

    /// Tangent-cone Cp at `x`. Below 1 deg of local slope the cone Cp is scaled linearly with the slope
    /// (`Cp -> 0` as `theta -> 0`) because the cone solver does not resolve vanishing cone angles.
    pub fn cp_tangent_cone(&self, x: f64) -> f64 {
        let theta = self.body.slope(x).atan();
        let floor = 1f64.to_radians();
        if theta >= floor {
            solve_cone(self.mach, theta).map_or(f64::NAN, |s| s.cp_surface)
        } else {
            solve_cone(self.mach, floor).map_or(f64::NAN, |s| s.cp_surface) * theta / floor
        }
    }

    /// Shock-expansion Cp at `x`.
    pub fn cp_shock_expansion(&self, x: f64) -> f64 {
        let Some(tip) = &self.tip else { return f64::NAN };
        let theta0 = self.body.tip_half_angle();
        let theta = self.body.slope(x).atan();
        let nu = prandtl_meyer(tip.mach_surface) + (theta0 - theta);
        let m2 = mach_from_pm(nu);
        let t = |m: f64| 1.0 + 0.5 * (GAMMA - 1.0) * m * m;
        let p_ratio = tip.p_ratio_surface * (t(tip.mach_surface) / t(m2)).powf(GAMMA / (GAMMA - 1.0));
        cp_from_ratio(p_ratio, self.mach)
    }

    /// Forebody pressure-drag coefficient referenced to the base area `pi R^2`:
    /// `CD = (1/R^2) * integral Cp d(r^2)`, integrated over the nose with `n` midpoint panels
    /// uniform in `x`.
    pub fn forebody_cd(&self, cp: impl Fn(&Self, f64) -> f64, n: usize) -> f64 {
        let l = self.body.nose_length();
        let r2 = self.body.radius().powi(2);
        let mut s = 0.0;
        for k in 0..n {
            let (x0, x1) = (l * k as f64 / n as f64, l * (k + 1) as f64 / n as f64);
            let xm = 0.5 * (x0 + x1);
            s += cp(self, xm) * (self.body.r(x1).powi(2) - self.body.r(x0).powi(2));
        }
        s / r2
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pm_roundtrip() {
        for m in [1.2, 2.0, 3.7] {
            assert!((mach_from_pm(prandtl_meyer(m)) - m).abs() < 1e-9);
        }
        // known value: nu(2) = 26.38 deg
        assert!((prandtl_meyer(2.0).to_degrees() - 26.3798).abs() < 1e-3);
    }

    #[test]
    fn ogive_geometry() {
        let o = Ogive { fineness: 3.0 };
        assert!((o.rho() - 9.25).abs() < 1e-12);
        assert!(o.r(0.0).abs() < 1e-12);
        assert!((o.r(3.0) - 0.5).abs() < 1e-12);
        assert!((o.tip_half_angle().to_degrees() - (3.0f64 / 8.75).atan().to_degrees()).abs() < 1e-12);
    }

    #[test]
    fn shock_expansion_starts_at_cone_value() {
        let th = SurfaceTheory::new(2.0, Ogive { fineness: 3.0 });
        let tip = th.tip.unwrap();
        assert!((th.cp_shock_expansion(0.0) - tip.cp_surface).abs() < 1e-9);
        assert!((th.cp_tangent_cone(0.0) - tip.cp_surface).abs() < 1e-6);
    }
}
