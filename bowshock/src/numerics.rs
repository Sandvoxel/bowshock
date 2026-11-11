//! Solver settings of a run: scheme, limiter, iteration limits and stop-rule tolerances.

use crate::case::Wake;
use bowshock_core::flux::Limiter;
use bowshock_core::implicit::{DtMode, ImplicitParams};
use bowshock_core::solver::{Bc, Scheme};

/// Sampling interval [iterations] of the monitor history.
pub const MONITOR_EVERY: usize = 50;
/// Mach number below which [`Numerics::auto`] uses the subsonic settings.
pub const REGIME_SPLIT_MACH: f64 = 0.85;

/// Pseudo-time parameters of the wall-resolved RANS runs of the facade: the summed-direction time step with a high CFL limit. The
/// weaker-direction step of [`ImplicitParams::VISCOUS`] (which the flat-plate gates of the solver use) diverged within a few hundred
/// iterations on the wall-resolved body grid of an ogive-cylinder at Mach 0.5 (cfl_max 10, also with a start CFL of 0.1).
const RANS_IMPLICIT: ImplicitParams = ImplicitParams { dt_mode: DtMode::Sum, cfl_max: 50.0, sub_iters: 3, ..ImplicitParams::DEFAULT };

/// Numerical settings of one run.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Numerics {
    pub cfl: f64,
    pub limiter: Limiter,
    pub low_mach: bool,
    pub scheme: Scheme,
    /// First-order start-up iterations.
    pub first_order_iters: usize,
    pub max_iter: usize,
    /// Stop rules other than a non-finite residual do not fire before this many iterations.
    pub min_iter: usize,
    /// Stop at this many orders of density-residual drop below the reference.
    pub drop_orders: f64,
    /// Boundary condition of the outflow plane.
    pub outflow: Bc,
    /// Length of the statistics window of the monitor [iterations].
    pub monitor_window: usize,
    /// Monitor range below which a run counts as stationary; `None`: no "stationary" stop rule.
    pub monitor_tol: Option<f64>,
}

impl Numerics {
    /// Line-implicit scheme, van Albada limiter, low-Mach correction (subsonic regime).
    pub fn subsonic(level: u32, wake: Wake) -> Numerics {
        Numerics {
            cfl: 0.5,
            limiter: Limiter::VanAlbada,
            low_mach: true,
            scheme: Scheme::Implicit(ImplicitParams::DEFAULT),
            first_order_iters: 100,
            max_iter: 10_000 << level,
            min_iter: 0,
            drop_orders: 6.0,
            outflow: Bc::Characteristic,
            monitor_window: 2000,
            monitor_tol: match wake {
                Wake::Base => Some(1e-3),
                Wake::Sting => None,
            },
        }
    }

    /// Explicit SSP-RK3, minmod limiter (transonic regime).
    pub fn transonic() -> Numerics {
        Numerics {
            cfl: 0.5,
            limiter: Limiter::Minmod,
            low_mach: false,
            scheme: Scheme::Explicit,
            first_order_iters: 300,
            max_iter: 60_000,
            min_iter: 0,
            drop_orders: 6.0,
            outflow: Bc::Characteristic,
            monitor_window: 2000,
            monitor_tol: Some(2e-4),
        }
    }

    /// Subsonic settings below [`REGIME_SPLIT_MACH`], transonic settings otherwise; the stationary tolerance follows the wake.
    pub fn auto(mach: f64, level: u32, wake: Wake) -> Numerics {
        let mut n = if mach < REGIME_SPLIT_MACH { Numerics::subsonic(level, wake) } else { Numerics::transonic() };
        n.monitor_tol = Some(match wake {
            Wake::Base => 1e-3,
            Wake::Sting => 2e-4,
        });
        n
    }

    /// Wall-resolved RANS (viscous terms and k-omega SST): line-implicit with the viscous parameters, van Albada limiter and the
    /// low-Mach correction below [`REGIME_SPLIT_MACH`], minmod without correction otherwise; stationary tolerance 1e-4 on the
    /// total CD.
    pub fn viscous(mach: f64, level: u32) -> Numerics {
        let subsonic = mach < REGIME_SPLIT_MACH;
        Numerics {
            cfl: 0.5,
            limiter: if subsonic { Limiter::VanAlbada } else { Limiter::Minmod },
            low_mach: subsonic,
            scheme: Scheme::Implicit(RANS_IMPLICIT),
            first_order_iters: 200,
            max_iter: 80_000 << level,
            min_iter: 0,
            drop_orders: 5.0,
            outflow: Bc::Characteristic,
            monitor_window: 2000,
            monitor_tol: Some(1e-4),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn auto_regime_split_and_tolerance() {
        let lo = Numerics::auto(0.5, 1, Wake::Sting);
        assert_eq!(lo.scheme, Scheme::Implicit(ImplicitParams::DEFAULT));
        assert_eq!(lo.max_iter, 20_000);
        assert_eq!(lo.monitor_tol, Some(2e-4));
        let hi = Numerics::auto(REGIME_SPLIT_MACH, 1, Wake::Base);
        assert_eq!(hi.scheme, Scheme::Explicit);
        assert_eq!(hi.max_iter, 60_000);
        assert_eq!(hi.monitor_tol, Some(1e-3));
        assert_eq!(Numerics::subsonic(0, Wake::Sting).monitor_tol, None);
    }
}
