//! Per-kernel GPU time of the line-implicit scheme on the subsonic sting case (`subsonic_gpu --base off`): `imp_profile [--f32]`.
//! Best of 5 profiled runs of 50 iterations per kernel (the GPU may be shared).
use bowshock_cfd::body_grid::Nose;
use bowshock_cfd::subsonic::SubsonicCase;
use bowshock_gpu::{GpuSolver, Mode, Precision};
use std::collections::BTreeMap;

fn main() {
    let prec = if std::env::args().any(|a| a == "--f32") { Precision::F32 } else { Precision::Extended };
    let ctx = luma::default_context().unwrap();
    for level in [0u32, 1] {
        let mut case = SubsonicCase::new(0.5, Nose::TangentOgive { fineness: 3.0 }, level, false);
        case.first_order_iters = 0;
        let host = case.build_solver();
        let mut g = GpuSolver::from_host(&ctx, &host, Mode::Guarded, prec).unwrap();
        g.run(100).unwrap();
        let n = 50;
        let mut best: BTreeMap<String, f64> = BTreeMap::new();
        let mut best_total = f64::INFINITY;
        for _ in 0..5 {
            let (r, profile) = ctx.profile(|| g.run(n)).unwrap();
            r.unwrap();
            for k in &profile.kernels {
                let us = 1e6 * k.total.as_secs_f64() / n as f64;
                let e = best.entry(k.label.to_string()).or_insert(f64::INFINITY);
                *e = e.min(us);
            }
            best_total = best_total.min(1e6 * profile.total().as_secs_f64() / n as f64);
        }
        println!("L{level} {}x{} {prec:?}, GPU us per iteration (profiling serialises the passes):", host.grid.ni, host.grid.nj);
        let mut rows: Vec<_> = best.into_iter().collect();
        rows.sort_by(|a, b| b.1.total_cmp(&a.1));
        for (label, us) in rows {
            println!("  {label:<20} {us:>8.1}");
        }
        println!("  {:<20} {best_total:>8.1}", "total");
    }
}
