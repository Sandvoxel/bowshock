//! Where does the f32 residual floor come from? Residual floor of the 15 deg / minmod / 100x50 (and 200x100)
//! case after 6000 GPU iterations as a function of the CFL number (the local time step `dt` scales with it).
//! If the floor is the quantisation of the state update `dt * res` at the f32 ulp of `u`, it scales like `1/CFL`;
//! noise in the flux evaluation would give a CFL-independent floor.
//!
//! (Original f32 precision: `Precision::F32`. See `f32_cpu_emulation` and `f32_diag` for the experiments that identify
//! the cause and `Precision::Extended` for the fix.)
//!
//! `cargo run --release -p bowshock-gpu --example f32_floor`

use bowshock_gpu::{GpuSolver, Mode, Precision};
use bowshock_cases::cone::{build_solver, ConeCase};
use bowshock_reference::taylor_maccoll::solve_cone;

fn main() {
    let tm = solve_cone(2.0, 15f64.to_radians()).unwrap();
    let ctx = luma::default_context().unwrap();
    println!("| grid | CFL | peak residual | geo-mean residual of iterations 5000-6000 | floor x CFL |");
    println!("|---|---|---|---|---|");
    for &(ni, nj) in &[(100usize, 50usize), (200, 100)] {
        for &cfl in &[0.125f64, 0.25, 0.5, 1.0] {
            let mut case = ConeCase::new(2.0, 15.0, ni, nj);
            case.cfl = cfl;
            let host = build_solver(&case, tm.beta);
            let mut g = GpuSolver::new_with(&ctx, &host.grid, &host.cfg, Mode::Guarded, Precision::F32).unwrap();
            let mut res = Vec::new();
            for _ in 0..120 {
                res.extend(g.run(50).unwrap());
            }
            let peak = res[20..].iter().cloned().fold(0.0, f64::max);
            let tail = &res[5000..6000];
            let floor = 10f64.powf(tail.iter().map(|x| x.log10()).sum::<f64>() / tail.len() as f64);
            println!("| {ni}x{nj} | {cfl} | {peak:.2e} | {floor:.2e} | {:.2e} |", floor * cfl);
        }
    }
}
