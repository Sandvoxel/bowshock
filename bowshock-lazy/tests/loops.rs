//! The three ways of running the same step (`iterate_until_all` with a recorded residual, `iterate_all` replay, an `eval_all`
//! loop) compute the same states and residual histories: the benchmark compares their speed, this checks that they are the
//! same computation. 20 iterations on the 40x20 cone grid, the first 5 first order (two traced phases).

use bowshock_cases::cone::{build_solver, ConeCase};
use bowshock_reference::taylor_maccoll::solve_cone;

macro_rules! check {
    ($module:ident) => {{
        let tm = solve_cone(2.0, 15f64.to_radians()).unwrap();
        let mut case = ConeCase::new(2.0, 15.0, 40, 20);
        case.first_order_iters = 5;
        let host = build_solver(&case, tm.beta);
        let ctx = luma::Context::new().unwrap();
        let run = |form: usize| {
            let mut s = bowshock_lazy::$module::LazySolver::new(&ctx, &host.grid, &host.cfg).unwrap();
            let history = match form {
                0 => s.run(20, 7).unwrap(),
                1 => {
                    s.run_replay(20).unwrap();
                    Vec::new()
                }
                _ => s.run_eval(20).unwrap().read().unwrap(),
            };
            (s.state_f64().unwrap(), history)
        };
        let (u_until, h_until) = run(0);
        let (u_replay, _) = run(1);
        let (u_eval, h_eval) = run(2);
        let worst = |a: &[[f64; 4]], b: &[[f64; 4]]| a.iter().zip(b).fold(0.0f64, |m, (x, y)| (0..4).fold(m, |m, k| m.max((x[k] - y[k]).abs() / (1.0 + y[k].abs()))));
        let hist = h_until.iter().zip(&h_eval).fold(0.0f64, |m, (a, b)| m.max((a - b).abs() / b));
        println!(
            "{}: until vs eval state {:.2e}, replay vs eval state {:.2e}, residual history until vs eval {:.2e} ({} / {} values)",
            stringify!($module),
            worst(&u_until, &u_eval),
            worst(&u_replay, &u_eval),
            hist,
            h_until.len(),
            h_eval.len()
        );
        assert_eq!((h_until.len(), h_eval.len()), (20, 20));
        assert!(worst(&u_until, &u_eval) <= 1e-6 && worst(&u_replay, &u_eval) <= 1e-6 && hist <= 1e-6);
    }};
}

#[test]
fn f32_loop_forms_compute_the_same_iteration() {
    check!(f32s);
}

#[test]
fn df32_loop_forms_compute_the_same_iteration() {
    check!(df32s);
}

/// Threads that share `luma::default_context()` each trace and replay their own solver (this was wrong before Luma gave every
/// thread its own lazy arrays): every thread must reproduce the state of the same run on a private context.
#[test]
fn threads_on_the_default_context_agree() {
    macro_rules! run {
        ($module:ident, $ctx:expr) => {{
            let tm = solve_cone(2.0, 15f64.to_radians()).unwrap();
            let mut case = ConeCase::new(2.0, 15.0, 40, 20);
            case.first_order_iters = 5;
            let host = build_solver(&case, tm.beta);
            let mut s = bowshock_lazy::$module::LazySolver::new($ctx, &host.grid, &host.cfg).unwrap();
            let history = s.run(20, 7).unwrap();
            (s.state_f64().unwrap(), history)
        }};
    }
    let worst = |a: &[[f64; 4]], b: &[[f64; 4]]| a.iter().zip(b).fold(0.0f64, |m, (x, y)| (0..4).fold(m, |m, k| m.max((x[k] - y[k]).abs() / (1.0 + y[k].abs()))));
    let private = luma::Context::new().unwrap();
    let (want_f32, want_df32) = (run!(f32s, &private), run!(df32s, &private));
    let results: Vec<_> = std::thread::scope(|scope| {
        let threads: Vec<_> = (0..4)
            .map(|t| {
                scope.spawn(move || {
                    let shared = luma::default_context().unwrap();
                    if t % 2 == 0 {
                        (0, run!(f32s, &shared))
                    } else {
                        (1, run!(df32s, &shared))
                    }
                })
            })
            .collect();
        threads.into_iter().map(|t| t.join().unwrap()).collect()
    });
    for (precision, (state, history)) in &results {
        let want = if *precision == 0 { &want_f32 } else { &want_df32 };
        let (d_state, d_hist) = (worst(state, &want.0), history.iter().zip(&want.1).fold(0.0f64, |m, (a, b)| m.max((a - b).abs() / b)));
        println!("default context, {}: state {d_state:.2e}, residual {d_hist:.2e}", if *precision == 0 { "f32" } else { "df32" });
        assert!(d_state <= 1e-6 && d_hist <= 1e-6);
    }
}
