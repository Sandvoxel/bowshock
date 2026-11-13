//! The line-implicit scheme (`Scheme::Implicit`) in array ops against the f64 CPU `Solver`, on the set-ups of
//! `bowshock-gpu-cases/tests/gpu_vs_cpu.rs` (`implicit_scheme_state_matches_cpu`, `implicit_scheme_converged_state_matches_cpu`):
//! the subsonic ogive-cylinder on the sting grid (no solid block), level 0, van Albada with the low-Mach correction; the state check also
//! runs the base case (solid block, `j_sting = 0`) and the Fox TM X-2439 body (block with sting and boattail).
//!
//! The left-hand side of the lazy scheme is f32 in both precisions (like the hand-written kernels'), so the agreement is that of two
//! converging iterations with slightly different preconditioners, not round-off: the state after 30 iterations
//! (`max |dU| / (1 + |U|)`) is compared with the tolerances of the hand-written kernels, and the converged states (5 orders of
//! density-residual drop, the stopping rule of `bowshock::run`) agree in iteration count and pressure field.

use bowshock::body_grid::Nose;
use bowshock::run::run_on;
use bowshock_cases::subsonic::{fox_case, subsonic_case};

/// State after 30 implicit iterations (the first 5 first order) vs CPU, `Staging::Fluxes`.
macro_rules! state_check {
    ($module:ident, $tol:expr, $label:expr, $case:expr) => {{
        let mut case = $case;
        case.numerics.first_order_iters = 5;
        assert!(matches!(case.numerics.scheme, bowshock_core::solver::Scheme::Implicit(_)));
        let mut cpu = case.build_solver();
        let ctx = luma::Context::new().unwrap();
        let mut lazy = bowshock_lazy::$module::LazySolver::from_host(&ctx, &cpu).unwrap();
        assert_eq!(lazy.staging, bowshock_lazy::Staging::Fluxes);
        let res_cpu: Vec<f64> = (0..30).map(|_| cpu.step()).collect();
        let res_lazy = lazy.run(30, 10).unwrap();
        let ul = lazy.state_f64().unwrap();
        let worst = ul.iter().zip(&cpu.u).fold(0.0f64, |m, (a, b)| (0..4).fold(m, |m, k| m.max((a[k] - b[k]).abs() / (1.0 + b[k].abs()))));
        let worst_res = res_lazy.iter().zip(&res_cpu).fold(0.0f64, |m, (a, b)| m.max((a - b).abs() / b));
        println!(
            "{} {}, 30 it: max |dU|/(1+|U|) = {worst:.2e}, max rel residual diff = {worst_res:.2e}, residual {:.3e} -> {:.3e}",
            stringify!($module),
            $label,
            res_cpu[0],
            res_cpu[29]
        );
        assert!(worst < $tol && res_lazy.iter().all(|r| r.is_finite()), "{}: state {worst:e}, residual {worst_res:e}", stringify!($module));
    }};
}

#[test]
fn implicit_scheme_state_matches_cpu() {
    let cases: [(&str, fn() -> bowshock::case::Case); 3] = [
        ("sting implicit", || subsonic_case(0.5, Nose::TangentOgive { fineness: 3.0 }, 0, false)),
        ("base implicit (j_sting = 0)", || subsonic_case(0.5, Nose::TangentOgive { fineness: 3.0 }, 0, true)),
        ("Fox sting + boattail implicit", || fox_case(false, 0.6, 0)),
    ];
    for (label, case) in cases {
        state_check!(f32s, 1e-3, label, case());
        state_check!(df32s, 1e-4, label, case());
    }
}

/// The options of `ImplicitParams` (scalar dissipation, point-implicit, frozen ghost states, one factorization pass, the weaker-direction
/// time step, a fast CFL ramp) against the CPU, 30 iterations in double-float, and the loop forms (`iterate_until_all`, `iterate_all`
/// replay, `eval_all` loop) against each other: the line solves are traced and replayed like any other kernel.
#[test]
fn implicit_options_and_loop_forms() {
    use bowshock_core::implicit::{DtMode, ImplicitParams};
    use bowshock_core::solver::Scheme;
    let base = match subsonic_case(0.5, Nose::TangentOgive { fineness: 3.0 }, 0, false).numerics.scheme {
        Scheme::Implicit(p) => p,
        s => panic!("{s:?}"),
    };
    let variants = [
        ("roe = false", ImplicitParams { roe: false, ..base }),
        ("lines = false", ImplicitParams { lines: false, ..base }),
        ("implicit_bc = false", ImplicitParams { implicit_bc: false, ..base }),
        ("sub_iters = 1", ImplicitParams { sub_iters: 1, ..base }),
        ("MinDirection, growth 1.2", ImplicitParams { dt_mode: DtMode::MinDirection, growth: 1.2, cfl_max: 20.0, ..base }),
    ];
    let ctx = luma::Context::new().unwrap();
    for (name, params) in variants {
        let mut case = subsonic_case(0.5, Nose::TangentOgive { fineness: 3.0 }, 0, false);
        case.numerics.first_order_iters = 5;
        case.numerics.scheme = Scheme::Implicit(params);
        let mut cpu = case.build_solver();
        let mut lazy = bowshock_lazy::df32s::LazySolver::from_host(&ctx, &cpu).unwrap();
        let res_cpu: Vec<f64> = (0..30).map(|_| cpu.step()).collect();
        let res_lazy = lazy.run(30, 10).unwrap();
        let worst = lazy.state_f64().unwrap().iter().zip(&cpu.u).fold(0.0f64, |m, (a, b)| (0..4).fold(m, |m, k| m.max((a[k] - b[k]).abs() / (1.0 + b[k].abs()))));
        let worst_res = res_lazy.iter().zip(&res_cpu).fold(0.0f64, |m, (a, b)| m.max((a - b).abs() / b));
        println!("{name}: state {worst:.2e}, residual {worst_res:.2e}, residual {:.3e} -> {:.3e}", res_cpu[0], res_cpu[29]);
        assert!(worst < 1e-4 && res_lazy.iter().all(|r| r.is_finite()), "{name}: state {worst:e}");
    }

    let case = subsonic_case(0.5, Nose::TangentOgive { fineness: 3.0 }, 0, false);
    let host = case.build_solver();
    macro_rules! forms {
        ($module:ident) => {{
            let run = |form: usize| {
                let mut s = bowshock_lazy::$module::LazySolver::from_host(&ctx, &host).unwrap();
                let hist = match form {
                    0 => s.run(20, 7).unwrap(),
                    1 => {
                        s.run_replay(20).unwrap();
                        Vec::new()
                    }
                    _ => s.run_eval(20).unwrap().read().unwrap(),
                };
                (s.state_f64().unwrap(), hist)
            };
            let ((u0, h0), (u1, _), (u2, h2)) = (run(0), run(1), run(2));
            let worst = |a: &[[f64; 4]], b: &[[f64; 4]]| a.iter().zip(b).fold(0.0f64, |m, (x, y)| (0..4).fold(m, |m, k| m.max((x[k] - y[k]).abs() / (1.0 + y[k].abs()))));
            let hist = h0.iter().zip(&h2).fold(0.0f64, |m, (a, b)| m.max((a - b).abs() / b));
            println!("{} implicit loop forms: until vs eval state {:.2e}, replay vs eval state {:.2e}, history {hist:.2e}", stringify!($module), worst(&u0, &u2), worst(&u1, &u2));
            assert!(worst(&u0, &u2) < 1e-5 && worst(&u1, &u2) < 1e-5 && hist < 1e-4);
        }};
    }
    forms!(f32s);
    forms!(df32s);
}

/// Converged-state agreement (double-float): the sting case at M = 0.3 and 0.6 iterated by the CPU (`bowshock::run::run_on`) and by
/// the lazy solver to 5 orders of density-residual drop (the stopping rule of `run_on`: `it > 20` and the peak since iteration 20 over
/// the residual `>= 1e5`); iteration counts within `0.05 cpu + 5`, pressure field within `1e-4 p_inf`.
#[test]
fn implicit_scheme_converged_state_matches_cpu() {
    for mach in [0.3, 0.6] {
        let mut case = subsonic_case(mach, Nose::TangentOgive { fineness: 3.0 }, 0, false);
        case.numerics.drop_orders = 5.0;
        case.numerics.max_iter = 4000;
        let mut s = case.build_solver();
        let ctx = luma::Context::new().unwrap();
        let mut lazy = bowshock_lazy::df32s::LazySolver::from_host(&ctx, &s).unwrap();
        let cpu = run_on(&case, &mut s, false, None);
        assert_eq!(cpu.stop_reason, bowshock::result::StopReason::ResidualDrop);

        // the same stopping rule on the lazy residual history, in chunks of 5 iterations
        let drop = 10f64.powf(case.numerics.drop_orders);
        let mut history: Vec<f64> = Vec::new();
        let mut stop = None;
        while stop.is_none() && history.len() < case.numerics.max_iter {
            history.extend(lazy.run(5, 5).unwrap());
            let mut rmax = 0.0f64;
            for (it, &r) in history.iter().enumerate() {
                if it >= 20 {
                    rmax = rmax.max(r);
                }
                if it > 20 && rmax / r >= drop {
                    stop = Some(it + 1);
                    break;
                }
            }
        }
        let iters = stop.expect("the lazy solver did not converge");
        let (pl, pc) = (pressure(&lazy.state_f64().unwrap()), pressure(&s.u));
        let dp = pl.iter().zip(&pc).fold(0.0f64, |m, (a, b)| m.max((a - b).abs())) / s.cfg.freestream[3];
        println!("M = {mach}: CPU {} it, lazy df32 {iters} it ({} run), max |dp| / p_inf = {dp:.2e}", cpu.iters, history.len());
        assert!((iters as f64 - cpu.iters as f64).abs() <= 0.05 * cpu.iters as f64 + 5.0, "iteration counts {iters} vs {}", cpu.iters);
        assert!(dp < 1e-4, "converged pressure fields differ by {dp:e} p_inf");
    }
}

/// Static pressure `(g - 1) (E - (rho u)^2 + (rho v)^2) / (2 rho))` of conserved states.
fn pressure(u: &[[f64; 4]]) -> Vec<f64> {
    u.iter().map(|q| (bowshock_core::gas::GAMMA - 1.0) * (q[3] - 0.5 * (q[1] * q[1] + q[2] * q[2]) / q[0])).collect()
}
