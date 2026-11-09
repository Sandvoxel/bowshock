//! Which part of f32 storage limits the residual? CPU f64 arithmetic (bowshock-cfd) with the *state* constrained to f32 in
//! different ways, 15 deg / minmod, same iteration as `Solver::step` (SSP-RK3, local dt):
//!
//! * `f64`            : reference.
//! * `store f32`      : the state is rounded to f32 after every RK stage (what an f32 `u` buffer does to the update);
//!                      the residual is evaluated on the f32 state.
//! * `eval on f32(u)` : the state is kept in f64, only the copy the residual is evaluated on is rounded to f32
//!                      (state rounding noise in the residual, but updates are not quantised).
//!
//! `cargo run --release -p bowshock-gpu --example f32_cpu_emulation [-- ni nj]`

use bowshock_cfd::cone::{build_solver, ConeCase};
use bowshock_cfd::solver::Solver;
use bowshock_cfd::taylor_maccoll::solve_cone;

type Cons = [f64; 4];

fn r32(x: f64) -> f64 {
    x as f32 as f64
}

#[derive(Clone, Copy, PartialEq, Debug)]
enum V {
    F64,
    StoreF32,
    EvalOnF32,
}

/// One SSP-RK3 step; `ud` is the master state (f64), `s.u` the state the residual is evaluated on.
fn step(s: &mut Solver, ud: &mut Vec<Cons>, v: V) -> f64 {
    let stage_coef = [(1.0, 0.0), (0.25, 0.75), (2.0 / 3.0, 1.0 / 3.0)];
    let u0 = ud.clone();
    let mut rn = 0.0;
    for (k, &(a, b)) in stage_coef.iter().enumerate() {
        match v {
            V::EvalOnF32 => {
                for (q, d) in s.u.iter_mut().zip(ud.iter()) {
                    *q = d.map(r32);
                }
            }
            _ => s.u.clone_from(ud),
        }
        s.eval_residual(k == 0);
        if k == 0 {
            rn = s.density_residual();
        }
        for c in 0..ud.len() {
            for m in 0..4 {
                let x = a * (ud[c][m] + s.dt[c] * s.res[c][m]) + b * u0[c][m];
                ud[c][m] = if v == V::StoreF32 { r32(x) } else { x };
            }
        }
    }
    s.iter += 1;
    rn
}

fn main() {
    let a: Vec<usize> = std::env::args().skip(1).filter_map(|s| s.parse().ok()).collect();
    let (ni, nj) = if a.len() == 2 { (a[0], a[1]) } else { (100, 50) };
    let tm = solve_cone(2.0, 15f64.to_radians()).unwrap();
    let case = ConeCase::new(2.0, 15.0, ni, nj);
    println!("15 deg, minmod, {ni}x{nj}, CFL {}, CPU f64 arithmetic", case.cfl);
    println!("| state handling | peak | min residual | geo-mean of its. 6000-8000 | orders below peak (min) |");
    println!("|---|---|---|---|---|");
    for v in [V::F64, V::StoreF32, V::EvalOnF32] {
        let mut s = build_solver(&case, tm.beta);
        let mut ud = s.u.clone();
        let mut res = Vec::new();
        for _ in 0..8000 {
            res.push(step(&mut s, &mut ud, v));
        }
        let peak = res[20..].iter().cloned().fold(0.0, f64::max);
        let min = res.iter().cloned().fold(f64::MAX, f64::min);
        let tail = &res[6000..8000];
        let gm = 10f64.powf(tail.iter().map(|x| x.log10()).sum::<f64>() / tail.len() as f64);
        println!("| {v:?} | {peak:.2e} | {min:.2e} | {gm:.2e} | {:.2} |", (peak / min).log10());
    }
}
