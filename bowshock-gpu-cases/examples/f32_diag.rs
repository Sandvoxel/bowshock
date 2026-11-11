//! Diagnostics of the f32 residual floor on the state the GPU solver ends up in.
//!
//! Runs the 15 deg / minmod case on the GPU to the floor, then evaluates *that f32 state* with the f64 CPU solver
//! and compares: residual in f64 vs f32, update `dt*res` vs the f32 ulp of the state, and where the residual lives.
//!
//! `cargo run --release -p bowshock-gpu-cases --example f32_diag [-- ni nj]`

use bowshock_cases::cone::{build_solver, ConeCase};
use bowshock_gpu::{GpuSolver, Mode, Precision};
use bowshock_reference::taylor_maccoll::solve_cone;

fn ulp(x: f32) -> f64 {
    let b = x.abs().to_bits();
    (f32::from_bits(b + 1) - f32::from_bits(b)) as f64
}

fn rms(v: impl Iterator<Item = f64>, n: usize) -> f64 {
    (v.map(|x| x * x).sum::<f64>() / n as f64).sqrt()
}

fn main() {
    let a: Vec<usize> = std::env::args().skip(1).filter_map(|s| s.parse().ok()).collect();
    let (ni, nj) = if a.len() == 2 { (a[0], a[1]) } else { (100, 50) };
    let tm = solve_cone(2.0, 15f64.to_radians()).unwrap();
    let case = ConeCase::new(2.0, 15.0, ni, nj);
    let host = build_solver(&case, tm.beta);
    let ctx = luma::default_context().unwrap();
    let n = ni * nj;
    for prec in [Precision::F32, Precision::Compensated, Precision::Extended] {
        println!("\n## {ni}x{nj}, precision mode {prec:?}");
        let mut g = GpuSolver::new_with(&ctx, &host.grid, &host.cfg, Mode::Guarded, prec).unwrap();
        let mut hist = Vec::new();
        for _ in 0..160 {
            hist.extend(g.run(50).unwrap());
        }
        let peak = hist[20..].iter().cloned().fold(0.0, f64::max);
        let tail = &hist[7000..8000];
        let gm = 10f64.powf(tail.iter().map(|x| x.log10()).sum::<f64>() / tail.len() as f64);
        println!("GPU residual after 8000 its: peak {peak:.2e}, geo-mean of 7000-8000 {gm:.2e} ({:.2} orders)", (peak / gm).log10());

        // state in f64 (hi + lo for the compensated mode)
        let s64 = g.state_f64().unwrap();
        // the same state, evaluated by the f64 CPU solver
        let mut cpu = build_solver(&case, tm.beta);
        cpu.iter = case.first_order_iters; // MUSCL on
        cpu.u.clone_from(&s64);
        cpu.eval_residual(true);
        let r64 = cpu.density_residual();
        // ... and by the f32 GPU solver (state upload keeps only the f32 state; stage-1 residual field)
        let mut g2 = GpuSolver::new_with(&ctx, &host.grid, &host.cfg, Mode::Guarded, prec).unwrap();
        g2.set_state(&s64).unwrap();
        g2.iter = case.first_order_iters;
        let r32n = g2.run(1).unwrap()[0];
        let rf = g2.rho_residual_field().unwrap();
        let d_arith = rms(rf.iter().zip(&cpu.res).map(|(a, b)| *a as f64 - b[0]), n);
        println!("rho residual of this state: f64 evaluation {r64:.2e}, f32 GPU evaluation {r32n:.2e}, rms(R32 - R64) {d_arith:.2e}");

        // f64 residual of the f32-ROUNDED state (state-rounding noise only)
        let mut cpu2 = build_solver(&case, tm.beta);
        cpu2.iter = case.first_order_iters;
        for (q, s) in cpu2.u.iter_mut().zip(&s64) {
            *q = s.map(|x| x as f32 as f64);
        }
        cpu2.eval_residual(true);
        println!("f64 evaluation of the f32-rounded state: {:.2e}", cpu2.density_residual());

        // update size vs ulp
        let uf: Vec<[f32; 4]> = s64.iter().map(|q| q.map(|x| x as f32)).collect();
        for (m, name) in ["rho", "rho u", "rho v", "E"].iter().enumerate() {
            let ratios: Vec<f64> = (0..n).map(|c| (cpu.dt[c] * cpu.res[c][m]).abs() / ulp(uf[c][m])).collect();
            let mut sorted = ratios.clone();
            sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());
            let lost = ratios.iter().filter(|&&r| r < 0.5).count() as f64 / n as f64;
            println!(
                "|dt*res|/ulp(U) [{name}]: median {:.2}, mean {:.2}, p90 {:.2}, max {:.1}; fraction < 0.5 ulp (update rounds away): {:.1} %",
                sorted[n / 2],
                ratios.iter().sum::<f64>() / n as f64,
                sorted[n * 9 / 10],
                sorted[n - 1],
                100.0 * lost
            );
        }
        // where does the residual live (f64 eval of this state): share of sum rho_res^2 in i- and j-bins and the 5 largest cells
        let sum2: f64 = cpu.res.iter().map(|r| r[0] * r[0]).sum();
        let mut bi = vec![0.0; 10];
        let mut bj = vec![0.0; 10];
        for j in 0..nj {
            for i in 0..ni {
                let v = cpu.res[j * ni + i][0].powi(2) / sum2;
                bi[i * 10 / ni] += v;
                bj[j * 10 / nj] += v;
            }
        }
        let f = |b: &[f64]| b.iter().map(|x| format!("{:.0}%", 100.0 * x)).collect::<Vec<_>>().join(" ");
        println!("share of sum rho_res^2 (f64 evaluation of the GPU state) by i-decile (tip -> base): {}", f(&bi));
        println!("  by j-decile (wall -> outer boundary): {}", f(&bj));
        let mut idx: Vec<usize> = (0..n).collect();
        idx.sort_by(|&x, &y| cpu.res[y][0].abs().partial_cmp(&cpu.res[x][0].abs()).unwrap());
        let top: Vec<String> = idx[..5].iter().map(|&c| format!("({},{}) {:.1}%", c % ni, c / ni, 100.0 * cpu.res[c][0].powi(2) / sum2)).collect();
        println!("  largest cells (i,j) share: {}", top.join(", "));
    }
}
