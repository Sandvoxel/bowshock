//! The low-Mach preconditioning of the implicit left-hand side (`ImplicitParams::low_mach_precond`) on the GPU against the f64 CPU `Solver`
//! with the same parameters: the state after 20 implicit iterations of the sting case (level 0, first 5 first order) as in `implicit.rs`.
//! The tolerances are those of `implicit_scheme_state_matches_cpu` (1e-3 f32, 1e-4 double-float). The preconditioning off is covered by
//! `implicit.rs`.

use bowshock_cases::subsonic::subsonic_case;
use bowshock_cfd::implicit::ImplicitParams;
use bowshock_cfd::solver::Scheme;
use bowshock_core::engine::body_grid::Nose;

/// The sting case at `mach` with the preconditioning switched on.
fn preconditioned_case(mach: f64) -> bowshock_core::engine::case::Case {
    let mut case = subsonic_case(mach, Nose::TangentOgive { fineness: 3.0 }, 0, false);
    case.numerics.first_order_iters = 5;
    let Scheme::Implicit(p) = case.numerics.scheme else { panic!("the sting case is implicit") };
    assert!(!p.low_mach_precond, "the subsonic preset leaves the preconditioning off");
    case.numerics.scheme = Scheme::Implicit(ImplicitParams { low_mach_precond: true, ..p });
    case
}

macro_rules! state_check {
    ($module:ident, $tol:expr, $label:expr, $case:expr) => {{
        let case = $case;
        let mut cpu = case.build_solver();
        let ctx = luma::Context::new().unwrap();
        let mut lazy = bowshock_cfd::gpu::$module::LazySolver::from_host(&ctx, &cpu).unwrap();
        let res_cpu: Vec<f64> = (0..20).map(|_| cpu.step()).collect();
        let res_lazy = lazy.run(20, 10).unwrap();
        let ul = lazy.state_f64().unwrap();
        let worst = ul.iter().zip(&cpu.u).fold(0.0f64, |m, (a, b)| (0..4).fold(m, |m, k| m.max((a[k] - b[k]).abs() / (1.0 + b[k].abs()))));
        let worst_res = res_lazy.iter().zip(&res_cpu).fold(0.0f64, |m, (a, b)| m.max((a - b).abs() / b));
        println!(
            "LOWMACH {} {}, 20 it: max |dU|/(1+|U|) = {worst:.2e}, max rel residual diff = {worst_res:.2e}, residual {:.3e} -> {:.3e}",
            stringify!($module),
            $label,
            res_cpu[0],
            res_cpu[19]
        );
        assert!(worst < $tol && res_lazy.iter().all(|r| r.is_finite()), "{}: state {worst:e}, residual {worst_res:e}", stringify!($module));
    }};
}

#[test]
fn lowmach_implicit_state_matches_cpu() {
    for mach in [0.5, 0.1] {
        let label = format!("sting M{mach}");
        state_check!(f32s, 1e-3, label, preconditioned_case(mach));
        state_check!(df32s, 1e-4, label, preconditioned_case(mach));
    }
}
