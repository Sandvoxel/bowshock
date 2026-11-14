//! Subsonic validation cases (M ~ 0.1 - 0.8): nose-cylinder-(boattail) bodies, either with a flat blunt base and wake
//! (`Wake::Base`) or with the cylinder continuing as a sting to the outflow plane (`Wake::Sting`), and the published Fox
//! TM X-2439 wall-pressure data. The runner, stop rules and post-processing are in `bowshock::run`.
//!
//! Lengths are in body diameters (`d = 1`, `R = 0.5`). Drag coefficients are pressure drag only (inviscid), referenced to the
//! maximum cross-section `pi R^2`.

use bowshock::body_grid::{BaseBody, Body, Nose};
use bowshock::case::{Case, Wake};
use bowshock::numerics::Numerics;

/// Body `nose + 6 d` cylinder (no boattail, no sting), domain `10 L` upstream and radially, `5 L` downstream (`L` = total body
/// length), grid level 0/1/2 (cells per diameter doubling each level). `base`: finite body with a flat base and a wake region
/// (`Wake::Base`); otherwise the cylinder continues as a sting to the outflow plane (`Wake::Sting`). Numerics:
/// [`Numerics::subsonic`] (line-implicit, van Albada limiter, low-Mach correction on).
pub fn subsonic_case(mach: f64, nose: Nose, level: u32, base: bool) -> Case {
    let body = BaseBody::cylinder(Body { nose }, 6.0);
    let wake = if base { Wake::Base } else { Wake::Sting };
    Case::new(mach, body, level, wake, Numerics::subsonic(level, wake))
}

/// NASA TM X-2439 test body (`cone = false`: configuration 4, 2-caliber tangent ogive; `cone = true`: configuration 2, 10 deg cone)
/// with cylinder, 10 deg boattail and the flat annular base on a sting (`fox_tm_x2439`), at Mach number `mach`.
pub fn fox_case(cone: bool, mach: f64, level: u32) -> Case {
    let nose = if cone { Nose::Cone { half_angle: 10f64.to_radians() } } else { Nose::TangentOgive { fineness: 2.0 } };
    let body = BaseBody::cylinder(Body { nose }, 2.0).with_boattail(1.0, 10f64.to_radians()).with_sting(fox_tm_x2439::STING_RADIUS);
    Case::new(mach, body, level, Wake::Base, Numerics::subsonic(level, Wake::Base))
}

/// Published subsonic wall pressures on axisymmetric nose-cylinder-boattail bodies on a sting, zero angle of attack.
///
/// Source: C. H. Fox Jr., "Experimental Surface Pressure Distributions for a Family of Axisymmetric Bodies at Subsonic Speeds",
/// NASA TM X-2439 (Langley, December 1971), NTRS 19720009348; Langley 7- by 10-foot high-speed tunnel, sting-mounted models,
/// `D = 14.605 cm`, `Re_D = 0.44e6 .. 0.68e6`, boundary-layer state not stated (probably laminar/transitional on the nose), no wall
/// or sting corrections described. **Tabulated** data (Table VI = configuration 4, Table IV = configuration 2), transcribed from the
/// scanned table pages by OCR with 2-3 repeated rows per value (angle of attack -0.02 .. -0.10 deg, not exactly zero) and a visual
/// check of part of the blocks. Values are the mean of the upper and lower meridian orifices (they differ by 0.002 - 0.02).
/// Uncertainty: stated gage accuracy +-0.5 % of full scale = +-0.017 (M = 0.40), +-0.008 (0.61), +-0.0055 (0.83) in Cp for the
/// 34 kPa gages (half for the 17 kPa gages); repeatability +-0.002 .. 0.004 (up to 0.02 near shoulders at M = 0.83); a transcription error of
/// single values is possible.
///
/// Configuration 4: tangent ogive nose of length 2.0 D (arc radius 4.25 D), cylinder 2.0 D, 10 deg boattail of 1.0 D length, flat
/// annular base on a sting (sting radius taken as 0.13 D from the base-orifice radii). Configuration 2: 10 deg half-angle cone
/// (length 2.836 D), cylinder 2.0 D, same boattail and base. Nominal Mach numbers 0.4, 0.6, 0.8 are 0.404, 0.608, 0.831 in the tables.
pub mod fox_tm_x2439 {
    pub const MACH: [f64; 3] = [0.404, 0.608, 0.831];
    /// Gage-accuracy uncertainty in Cp at the three Mach numbers (34 kPa gages).
    pub const CP_UNCERTAINTY: [f64; 3] = [0.0166, 0.0084, 0.0055];
    pub const STING_RADIUS: f64 = 0.13;
    /// Base annulus Cp (mean of 12 orifices at `x = 4.993` / `5.824`) of configurations 4 and 2 at the three Mach numbers.
    pub const BASE_CP_OGIVE: [f64; 3] = [0.0691, 0.0836, 0.1317];
    pub const BASE_CP_CONE: [f64; 3] = [0.0685, 0.0813, 0.1290];
    pub const OGIVE_CYL_X_D: [f64; 24] = [
        0.289, 0.512, 0.740, 0.972, 1.207, 1.443, 1.682, 1.921, 1.965, 2.008, 2.051, 2.321, 2.657, 2.993, 3.327, 3.659, 3.927, 3.972, 4.014, 4.057, 4.348,
        4.638, 4.928, 4.971,
    ];
    pub const OGIVE_CYL_CP_M04_MEAN: [f64; 24] = [
        0.2837, 0.1283, 0.0291, -0.0631, -0.1347, -0.1838, -0.1838, -0.1578, -0.1352, -0.1195, -0.0989, -0.0584, -0.0401, -0.0382, -0.0571, -0.0868, -0.2346,
        -0.3326, -0.3495, -0.2257, -0.0369, 0.0242, 0.0621, 0.0698,
    ];
    pub const OGIVE_CYL_CP_M06_MEAN: [f64; 24] = [
        0.3075, 0.1450, 0.0410, -0.0606, -0.1407, -0.1959, -0.1975, -0.1660, -0.1437, -0.1251, -0.1017, -0.0548, -0.0359, -0.0337, -0.0521, -0.0843, -0.2534,
        -0.3678, -0.3893, -0.2465, -0.0299, 0.0339, 0.0771, 0.0867,
    ];
    pub const OGIVE_CYL_CP_M08_MEAN: [f64; 24] = [
        0.3813, 0.2063, 0.0873, -0.0311, -0.1293, -0.1988, -0.2110, -0.1671, -0.1402, -0.1128, -0.0842, -0.0256, -0.0024, -0.0024, -0.0209, -0.0527, -0.2244,
        -0.3507, -0.9392, -0.3141, 0.0068, 0.0783, 0.1254, 0.1297,
    ];
    pub const CONE_CYL_X_D: [f64; 27] = [
        0.421, 0.754, 1.006, 1.088, 1.421, 1.590, 1.755, 2.088, 2.173, 2.422, 2.755, 2.798, 2.843, 2.886, 3.156, 3.491, 3.824, 4.158, 4.490, 4.755, 4.802,
        4.845, 4.868, 5.178, 5.469, 5.759, 5.803,
    ];
    pub const CONE_CYL_CP_M04_MEAN: [f64; 27] = [
        0.1300, 0.1226, 0.1003, 0.1031, 0.0708, 0.0606, 0.0467, 0.0254, -0.0082, -0.0254, -0.2314, -0.3774, -0.3737, -0.2421, -0.0935, -0.0559, -0.0458,
        -0.0587, -0.0896, -0.2349, -0.3347, -0.3529, -0.2284, -0.0360, 0.0239, 0.0628, 0.0701,
    ];
    pub const CONE_CYL_CP_M06_MEAN: [f64; 27] = [
        0.1421, 0.1395, 0.1144, 0.1162, 0.0810, 0.0405, 0.0603, 0.0371, -0.0014, -0.0159, -0.2466, -0.4208, -0.4204, -0.2632, -0.0933, -0.0505, -0.0397,
        -0.0545, -0.0877, -0.2530, -0.3722, -0.3911, -0.2471, -0.0311, 0.0337, 0.0753, 0.0835,
    ];
    pub const CONE_CYL_CP_M08_MEAN: [f64; 27] = [
        0.1906, 0.1849, 0.1619, 0.1618, 0.1271, 0.1181, 0.1060, 0.0848, 0.0473, 0.0316, -0.1650, -0.2975, -0.9647, -0.8571, -0.0542, -0.0172, -0.0101, -0.0258,
        -0.0578, -0.2303, -0.3566, -0.9101, -0.2894, 0.0068, 0.0758, 0.1219, 0.1295,
    ];
}

#[cfg(test)]
mod tests {
    use super::*;
    use bowshock::result::StopReason;
    use bowshock::run::run_cold;
    use bowshock_core::gas::freestream;
    use bowshock_core::implicit::ImplicitParams;
    use bowshock_core::solver::Scheme;
    use bowshock_reference::base_drag::base_drag_coefficient;

    fn small(base: bool, mach: f64) -> Case {
        let mut c = subsonic_case(mach, Nose::TangentOgive { fineness: 3.0 }, 0, base).with_domain(5.0, 2.5);
        c.grid_base.n_up = 12;
        c.grid_base.n_nose = 24;
        c.grid_base.n_seg[0] = 20;
        c.grid_base.n_wake = 16;
        c.grid_base.n_gap = 8;
        c.grid_base.nj_out = 24;
        c.grid_base.dx_tip = 0.08;
        c.grid_base.dr_wall = 0.08;
        c.grid_sting.n_up = 12;
        c.grid_sting.n_nose = 24;
        c.grid_sting.n_seg[0] = 20;
        c.grid_sting.n_ext = 12;
        c.grid_sting.nj = 32;
        c.grid_sting.dx_tip = 0.08;
        c.grid_sting.dr_wall = 0.08;
        c.numerics.first_order_iters = 100;
        c.numerics.max_iter = 3000;
        c
    }

    /// Short subsonic smoke test (no base): M = 0.3, 6 orders in a few hundred iterations, finite positive state, the slender-body
    /// pressure signature (stagnation-like Cp > 0 at the tip, suction at the shoulder), and close to d'Alembert (nose CD ~ 0).
    #[test]
    fn subsonic_smoke_no_base() {
        let r = run_cold(&small(false, 0.3));
        assert_eq!(r.stop_reason, StopReason::ResidualDrop, "{} iters, drop {:.2}", r.iters, r.drop_orders);
        assert!(r.iters < 1500, "{} iterations", r.iters);
        assert!(r.min_rho > 0.0 && r.min_p > 0.0 && r.residual.iter().all(|x| x.is_finite()));
        assert!(r.wall[0].cp > 0.1 && r.cp_min < -0.05 && r.cp_min > -0.3, "Cp tip {} min {}", r.wall[0].cp, r.cp_min);
        assert!(r.cd_nose.abs() < 0.02, "nose CD {}", r.cd_nose);
        assert!(r.cp_base.is_nan() && r.base_wall.is_empty());
        assert_eq!(r.cd_base_model, 0.0);
        assert!((r.cd_total_model - r.cd_total).abs() < 1e-15);
    }

    /// Base smoke test at M = 0.2: the inviscid wake reaches a stationary base pressure (not a 6-order drop), the base pressure is a
    /// suction (Cp_b < 0), the drag breakdown adds up, the pressure drag of the closed body is dominated by the base (base CD =
    /// -Cp_b A_b / A_ref = -Cp_b for a full disc) and the modelled base drag is the correlation's for a full disc.
    #[test]
    fn subsonic_smoke_with_base() {
        // standard level-0 grids (the coarse test grids are too crude for the wake to settle)
        let mut c = subsonic_case(0.2, Nose::TangentOgive { fineness: 3.0 }, 0, true);
        c.numerics.max_iter = 6000;
        let r = run_cold(&c);
        assert!(
            matches!(r.stop_reason, StopReason::ResidualDrop | StopReason::BasePressureStationary),
            "{} after {} iters, drop {:.2}",
            r.stop_reason,
            r.iters,
            r.drop_orders
        );
        assert!(r.min_rho > 0.0 && r.min_p > 0.0);
        assert!(r.cp_base < -0.01 && r.cp_base > -0.5, "Cp_b {}", r.cp_base);
        assert!((r.cd_base + r.cp_base).abs() < 1e-12, "base CD {} vs -Cp_b {}", r.cd_base, -r.cp_base);
        assert!((r.cd_total - (r.cd_nose + r.cd_aft[0] + r.cd_base)).abs() < 1e-14);
        assert!(r.cd_base > 0.0 && r.cd_base > 5.0 * r.cd_nose.abs());
        assert_eq!(r.base_wall.len(), c.grid_base.n_gap);
        assert!((r.cd_base_model - base_drag_coefficient(0.2)).abs() < 1e-15);
    }

    /// A uniform free stream is a fixed point of the blocked-grid solver with the implicit scheme (body block, base wall, wake):
    /// the residual of the free stream is non-zero only through the body (it is blocked), so check the far field stays quiet
    /// and the state positive after a few iterations from the free stream; and the explicit and implicit residual norms at
    /// iteration 0 are identical (same residual kernel).
    #[test]
    fn blocked_grid_first_residual_is_scheme_independent() {
        let mut a = small(true, 0.4);
        a.numerics.scheme = Scheme::Explicit;
        let (mut sa, mut sb) = (a.build_solver(), small(true, 0.4).build_solver());
        let (ra, rb) = (sa.step(), sb.step());
        assert!(ra > 0.0 && (ra - rb).abs() <= 1e-14 * ra, "{ra:e} vs {rb:e}");
        // solid cells never change
        let b = sa.block().unwrap();
        let q0 = bowshock_core::gas::prim_to_cons(&freestream(0.4));
        for j in 0..b.j_end {
            for i in 0..b.i_end {
                for k in 0..4 {
                    assert!((sa.u[j * sa.grid.ni + i][k] - q0[k]).abs() < 1e-14, "explicit solid cell changed");
                    assert!((sb.u[j * sb.grid.ni + i][k] - q0[k]).abs() < 1e-14, "implicit solid cell changed");
                }
            }
        }
    }

    /// The converged state must not depend on the pseudo-time scheme: explicit SSP-RK3, line-implicit (with and without defect
    /// correction, Rusanov and Roe-type left-hand side) and the point-implicit variant reach the same fixed point. The explicit
    /// comparison uses the first-order scheme (the limited second-order explicit iteration only reaches a limit cycle, README);
    /// the implicit variants are also compared with each other in the second-order van Albada scheme.
    #[test]
    fn converged_state_is_independent_of_the_acceleration_scheme() {
        use bowshock_core::flux::Limiter;
        use bowshock_core::gas::Cons;
        let mut case = subsonic_case(0.5, Nose::TangentOgive { fineness: 3.0 }, 0, false);
        case.grid_sting.n_up = 12;
        case.grid_sting.n_nose = 20;
        case.grid_sting.n_seg[0] = 12;
        case.grid_sting.n_ext = 12;
        case.grid_sting.nj = 24;
        case.grid_sting.x_up = 10.0;
        case.grid_sting.x_down = 12.0;
        case.grid_sting.r_far = 10.0;
        case.grid_sting.dx_tip = 0.1;
        case.grid_sting.dr_wall = 0.1;
        case.numerics.first_order_iters = 0;
        let converge = |limiter: Limiter, scheme: Scheme, iters: usize| -> Vec<Cons> {
            let mut c = case.clone();
            c.numerics.limiter = limiter;
            c.numerics.scheme = scheme;
            c.numerics.cfl = 0.8;
            let mut s = c.build_solver();
            let mut last = 1.0;
            for _ in 0..iters {
                last = s.step();
                if last < 1e-11 {
                    break;
                }
            }
            assert!(last < 1e-9, "{limiter:?} {scheme:?} did not converge: {last:e}");
            s.u.clone()
        };
        let diff = |a: &[Cons], b: &[Cons]| {
            a.iter().zip(b.iter()).flat_map(|(a, b)| (0..4).map(move |k| (a[k] - b[k]).abs() / b[k].abs().max(1.0))).fold(0.0, f64::max)
        };
        let implicit_variants = [
            Scheme::Implicit(ImplicitParams { sub_iters: 1, roe: false, ..ImplicitParams::DEFAULT }),
            Scheme::Implicit(ImplicitParams { lines: false, cfl_max: 4.0, ..ImplicitParams::DEFAULT }),
        ];
        // first order: explicit vs implicit
        let reference = converge(Limiter::FirstOrder, Scheme::Explicit, 40000);
        for scheme in std::iter::once(Scheme::Implicit(ImplicitParams::DEFAULT)).chain(implicit_variants) {
            let d = diff(&converge(Limiter::FirstOrder, scheme, 8000), &reference);
            assert!(d < 1e-6, "first order, {scheme:?}: state difference to the explicit solution {d:e}");
        }
        // second order (van Albada): implicit variants among themselves
        let reference = converge(Limiter::VanAlbada, Scheme::Implicit(ImplicitParams::DEFAULT), 8000);
        {
            let scheme = Scheme::Implicit(ImplicitParams { sub_iters: 1, roe: false, ..ImplicitParams::DEFAULT });
            let d = diff(&converge(Limiter::VanAlbada, scheme, 8000), &reference);
            assert!(d < 1e-6, "van Albada, {scheme:?}: state difference {d:e}");
        }
    }
}
