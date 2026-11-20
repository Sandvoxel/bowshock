//! The plausibility guard of a settled 3D drag point (DESIGN3D item 73): a point that ends `Settled` (or in any stationary way) can still be the wrong one of two solutions of the
//! discrete problem. Cancer Descending at Draft, Mach 0.5: `cfl_max` 10 and the plain scheme converge to C_D 0.55 (base C_p -0.12); the DDADI cap of 50 `Settles` in 1 540 iterations at
//! C_D 1.505 (base C_p -1.04), and the default cap of 100 at 1.47. Nothing in the run says so (the residual, the monitor and the stop rule agree), but the base suction is far outside
//! what a rocket base does.
//!
//! The reference is the point's own base pressure, not a second engine: the 3D table is solved without the legacy engine, and the base C_p separates the two states of every case
//! measured. The area-mean base C_p of every point the engine has solved correctly is above -0.25 (Mach 0.1 to 3, viscous; Ion Drive -0.12 at most, Byrum -0.18, the example -0.20, Rabia
//! -0.24 on a `Stalled` point, Cancer Descending with its tip trailing edge flush -0.106 to -0.239); the wrong Cancer Descending states are at -0.37 (Mach 0.8) and -1.02 to -1.16
//! (Mach 0.5). A point below [`base_suction_floor`] is solved once more from its seed at `cfl_max` [`RETRY_CFL_MAX`] (the scan of item 73 is reliable there and above 20 it is not),
//! and [`decide`] keeps the answer: the second run when it is not suspect itself or when it reproduces the first within [`AGREE`] (a base that really sucks that hard), otherwise
//! the point is `Failed` and the table interpolates it. The cost is one more run of the suspect points only; a point above the floor is not touched.

use super::{PolarPoint, Quality, Warning};

/// The area-mean base C_p below which a settled viscous point is solved again, by Mach number: -0.30 below Mach 0.85 and from Mach 1.8 (the subsonic rockets measured sit at -0.10 to
/// -0.12 and the wrong Cancer Descending states at -0.37 and below; at Mach 2 the vacuum limit `-2 / (gamma M^2)` is -0.36 anyway), -0.60 from Mach 0.95 to 1.5 where a blunt
/// cylindrical base has its classic suction peak (-0.3 to -0.45 in the literature; the RANS wedge of the table body gives -0.14 / -0.20 / -0.20 / -0.19 at Mach 0.9 / 1.0 / 1.1 / 1.3,
/// the Euler one -0.18 / -0.26 / -0.27 / -0.27, which is why an inviscid point is never suspect), blended linearly in between.
pub(super) fn base_suction_floor(mach: f64) -> f64 {
    let ramp = |from: f64, to: f64| ((mach - from) / (to - from)).clamp(0.0, 1.0);
    // 0 below Mach 0.85, 1 from 0.95 to 1.5, 0 again from 1.8
    let wide = ramp(0.85, 0.95) * (1.0 - ramp(1.5, 1.8));
    -0.30 - 0.30 * wide
}

/// The `cfl_max` of the second run.
pub(super) const RETRY_CFL_MAX: f64 = 10.0;

/// Relative difference of the two total drags below which the second run confirms the first.
pub(super) const AGREE: f64 = 0.05;

/// Whether `p` ended as a result (converged or stationary) with a base suction under the floor of its Mach number: the point to solve again. Viscous points only (`drag.friction`).
pub(super) fn suspect(p: &PolarPoint) -> bool {
    // the comparison is false for a NaN base C_p
    matches!(p.quality, Quality::Converged | Quality::Stationary) && p.drag.friction.is_some() && p.detail.cp_base < base_suction_floor(p.mach)
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::analysis::{Drag, PointDetail};
    use crate::engine::result::StopReason;

    fn point(cd: f64, cp_base: f64, quality: Quality) -> PolarPoint {
        PolarPoint {
            mach: 0.5,
            alpha_deg: 0.0,
            drag: Drag { forebody_pressure: 0.0, friction: Some(0.1), base: 0.0, base_modelled: false, fins: None, total: cd },
            forces: None,
            detail: PointDetail {
                iterations: 100,
                residual_drop_orders: 2.0,
                stop_reason: StopReason::Settled,
                cd_nose: 0.0,
                cd_aft: Vec::new(),
                cd_friction: 0.0,
                cd_base_solver: 0.0,
                cp_base,
                wall: Vec::new(),
                y_plus_max: f64::NAN,
                farfield_dp: 0.0,
                cfl_limit: Some(100.0),
                rollbacks: Vec::new(),
            },
            quality,
            warnings: Vec::new(),
            cd_uncertainty: None,
            cd_band: None,
            x_cp_band: None,
            field: None,
            field3: None,
        }
    }

    /// The Cancer Descending states of item 73: the wrong one (C_D 1.47, base C_p -1.02) is suspect, the right one (0.55, -0.12) and the points of the correct rockets are not.
    #[test]
    fn only_a_deep_base_suction_is_suspect() {
        assert!(suspect(&point(1.47, -1.02, Quality::Stationary)));
        assert!(suspect(&point(0.79, -0.37, Quality::Converged)));
        for cp in [-0.12, -0.20, -0.239, 0.017, f64::NAN] {
            assert!(!suspect(&point(0.55, cp, Quality::Stationary)), "{cp}");
        }
        assert!(!suspect(&point(1.47, -1.02, Quality::Failed)));
    }

    /// The floor is stricter subsonic and supersonic than at the transonic suction peak, and an inviscid point is never suspect.
    #[test]
    fn the_floor_follows_the_mach_number() {
        assert_eq!(base_suction_floor(0.5), -0.30);
        assert_eq!(base_suction_floor(2.3), -0.30);
        assert_eq!(base_suction_floor(1.0), -0.60);
        assert_eq!(base_suction_floor(1.3), -0.60);
        let mut p = point(0.6, -0.45, Quality::Stationary);
        p.mach = 1.1;
        assert!(!suspect(&p));
        p.mach = 0.5;
        assert!(suspect(&p));
        p.drag.friction = None;
        assert!(!suspect(&p));
    }

    /// The second run replaces a wrong first; confirms a first that is real (still suspect, but reproduced); and a point neither run of which is usable is `Failed`.
    #[test]
    fn the_second_run_decides() {
        let (kept, second) = decide(point(1.47, -1.02, Quality::Stationary), point(0.55, -0.12, Quality::Stationary));
        assert!(second && kept.drag.total == 0.55 && matches!(kept.warnings[0], Warning::RestartedAtLowerCfl { cfl_to, .. } if cfl_to == RETRY_CFL_MAX));
        let (kept, second) = decide(point(0.80, -0.34, Quality::Stationary), point(0.81, -0.33, Quality::Stationary));
        assert!(second && kept.quality == Quality::Stationary);
        let (kept, second) = decide(point(1.47, -1.02, Quality::Stationary), point(1.10, -0.60, Quality::Stationary));
        assert!(!second && kept.quality == Quality::Failed);
        let (kept, second) = decide(point(1.47, -1.02, Quality::Stationary), point(0.55, -0.12, Quality::Stalled));
        assert!(!second && kept.quality == Quality::Failed);
    }
}
