//! The plausibility guard of a settled 3D drag point (DESIGN3D item 73): a point that ends `Settled` (or in any stationary way) can still be the wrong one of two solutions of the
//! discrete problem. Cancer Descending at Draft, Mach 0.5: `cfl_max` 10 and the plain scheme converge to C_D 0.55 (base C_p -0.12); the DDADI cap of 50 `Settles` in 1 540 iterations at
//! C_D 1.505 (base C_p -1.04), and the default cap of 100 at 1.47. Nothing in the run says so (the residual, the monitor and the stop rule agree), but the base suction is far outside
//! what a rocket base does.
//!
//! The reference is the point's own base pressure, not a second engine: the 3D table is solved without the legacy engine, and the base C_p separates the two states of every case
//! measured. The area-mean base C_p of every point the engine has solved correctly is above -0.25 (Mach 0.1 to 3; Ion Drive -0.12 at most, Byrum -0.18, the example -0.20, Rabia
//! -0.24 on a `Stalled` point, Cancer Descending with its tip trailing edge flush -0.106 to -0.239); the wrong Cancer Descending states are at -0.37 (Mach 0.8) and -1.02 to -1.16
//! (Mach 0.5). A point below [`BASE_SUCTION_SUSPECT`] is solved once more from its seed at `cfl_max` [`RETRY_CFL_MAX`] (the scan of item 73 is reliable there and above 20 it is not),
//! and [`decide`] keeps the answer: the second run when it is not suspect itself or when it reproduces the first within [`AGREE`] (a base that really sucks that hard), otherwise
//! the point is `Failed` and the table interpolates it. The cost is one more run of the suspect points only; a point above the floor is not touched.

use super::{PolarPoint, Quality, Warning};

/// Area-mean base C_p below which a settled point is solved again.
pub(super) const BASE_SUCTION_SUSPECT: f64 = -0.30;

/// The `cfl_max` of the second run.
pub(super) const RETRY_CFL_MAX: f64 = 10.0;

/// Relative difference of the two total drags below which the second run confirms the first.
pub(super) const AGREE: f64 = 0.05;

/// Whether `p` ended as a result (converged or stationary) with a base suction under the floor: the point to solve again.
pub(super) fn suspect(p: &PolarPoint) -> bool {
    matches!(p.quality, Quality::Converged | Quality::Stationary) && p.detail.cp_base < BASE_SUCTION_SUSPECT
    // false for NaN
}

/// The point to keep of a suspect `first` and its second run: `(point, second_kept)`.
pub(super) fn decide(first: PolarPoint, second: PolarPoint) -> (PolarPoint, bool) {
    let usable = |p: &PolarPoint| matches!(p.quality, Quality::Converged | Quality::Stationary);
    let agrees = (second.drag.total - first.drag.total).abs() <= AGREE * first.drag.total.abs();
    if usable(&second) && (!suspect(&second) || agrees) {
        let mut kept = second;
        kept.warnings.push(Warning::RestartedAtLowerCfl { rollbacks: 0, cfl_from: first.detail.cfl_limit.unwrap_or(f64::NAN), cfl_to: RETRY_CFL_MAX });
        return (kept, true);
    }
    let mut failed = first;
    failed.quality = Quality::Failed;
    (failed, false)
}
