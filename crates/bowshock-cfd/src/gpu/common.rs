// What the 2D and 3D array-op solvers share: `include!`d by `gpu/mod.rs` before `body.rs` and `body3.rs`, once per precision, into a module that defines `S`,
// `c(f64) -> S`, `up(..)` and `to_f64(..)` (and the imports every array-op file of the module uses): the array type `A`, `G1`, the limiter and the shock weight on arrays, the residual history and the missing-state error.

use crate::gas3::GAMMA;
use crate::gpu::Staging;
use crate::sst::{Turbulence, Viscous, PRANDTL, PRANDTL_T};
use crate::limiter::{Limiter, LOW_MACH_FLOOR, SHOCK_SENSOR_HI, SHOCK_SENSOR_LO};
use luma::{concat, select, Array, Context, Fields, PadMode, Result};

/// Array of one field (`rho`, `u`, ...) over a block of cells or faces.
pub type A = Array<S>;
const G1: f64 = GAMMA - 1.0;

/// Limited slope `phi(a, b)`, `a = q_i - q_{i-1}`, `b = q_{i+1} - q_i`; zero unless `a b > 0`.
/// minmod: the smaller of `a`, `b` in modulus. van Albada: `(a (b^2 + e) + b (a^2 + e)) / (a^2 + b^2 + 2 e)`, `e = 1e-14`.
fn limited_slope(a: &A, b: &A, lim: Limiter) -> A {
    let same_sign = (a * b).gt(c(0.0));
    match lim {
        Limiter::FirstOrder => unreachable!("first order reconstruction has no slopes"),
        Limiter::Minmod => select(&same_sign, select(&a.abs().lt(b.abs()), a, b), c(0.0)),
        Limiter::VanAlbada => {
            let eps = 1e-14;
            let slope = (a * (b * b + c(eps)) + b * (a * a + c(eps))) / (a * a + b * b + c(2.0 * eps));
            select(&same_sign, slope, c(0.0))
        }
        Limiter::Venkatakrishnan => {
            let eps = 1e-14;
            let slope = c(2.0) * b * (a * a + a * b + c(eps)) / (c(2.0) * a * a + a * b + b * b + c(2.0 * eps));
            select(&same_sign, slope, c(0.0))
        }
    }
}

/// Weight of the Rusanov flux: `s = |p_R - p_L| / min(p_L, p_R)`, `t = clamp((s - lo) / (hi - lo), 0, 1)`, `theta = t^2 (3 - 2 t)`.
fn shock_weight(pl: &A, pr: &A) -> A {
    let s = (pr - pl).abs() / pl.minimum(pr);
    let t = ((s - c(SHOCK_SENSOR_LO)) / c(SHOCK_SENSOR_HI - SHOCK_SENSOR_LO)).maximum(c(0.0)).minimum(c(1.0));
    &t * &t * (c(3.0) - c(2.0) * &t)
}

/// The density residual scalars of an `eval` loop, one per iteration, as evaluated GPU arrays.
pub struct Residuals(Vec<A>);

impl Residuals {
    /// Reads the residual history (waits for the GPU).
    pub fn read(&self) -> Result<Vec<f64>> {
        self.0.iter().map(|a| Ok(to_f64(a)?[0])).collect()
    }
}

/// Error of an operation that needs the loop state before one was set.
fn no_state() -> luma::LumaError {
    luma::LumaError::Unsupported("the solver has no state: it is set by from_host or set_state".into())
}
