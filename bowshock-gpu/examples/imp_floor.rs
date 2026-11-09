use bowshock_cfd::body_grid::Nose;
use bowshock_cfd::solver::Scheme;
use bowshock_cfd::subsonic::SubsonicCase;
use bowshock_gpu::{GpuSolver, Mode, Precision};
fn main() {
    let ctx = luma::default_context().unwrap();
    let mach: f64 = std::env::args().nth(1).and_then(|v| v.parse().ok()).unwrap_or(0.5);
    let case = SubsonicCase::new(mach, Nose::TangentOgive { fineness: 3.0 }, 0, false);
    let host = case.build_solver();
    let (ni, _nj) = (host.grid.ni, host.grid.nj);
    let mut g = GpuSolver::from_host(&ctx, &host, Mode::Guarded, Precision::Extended).unwrap();
    let mut r = Vec::new();
    for _ in 0..1600 / 50 {
        r.extend(g.run(50).unwrap());
    }
    println!("implicit end {:.3e}", r.last().unwrap());
    let f = g.rho_residual_field().unwrap();
    let mut idx: Vec<usize> = (0..f.len()).collect();
    idx.sort_by(|&a, &b| f[b].abs().partial_cmp(&f[a].abs()).unwrap());
    for &c in idx.iter().take(8) {
        println!("  cell ({}, {}) x={:.2} r={:.2} drho/dt {:.3e}", c % ni, c / ni, host.grid.xc[c], host.grid.rc[c], f[c]);
    }
    let sum2: f64 = f.iter().map(|v| (*v as f64).powi(2)).sum();
    let top: f64 = idx.iter().take(20).map(|&c| (f[c] as f64).powi(2)).sum();
    println!("  share of sum r^2 in top 20 cells: {:.2}", top / sum2);
    g.set_scheme(Scheme::Explicit).unwrap();
    g.iter = 1000;
    let mut r = Vec::new();
    for _ in 0..10 {
        r.extend(g.run(50).unwrap());
    }
    print!("explicit after implicit:");
    for k in (0..500).step_by(50) {
        print!(" {:.2e}", r[k]);
    }
    println!();
}
