use bowshock_cfd::body_grid::Nose;
use bowshock_cfd::implicit::ImplicitParams;
use bowshock_cfd::solver::Scheme;
use bowshock_cfd::subsonic::SubsonicCase;
use bowshock_gpu::{GpuSolver, Mode, Precision};
use std::time::Instant;
fn main() {
    let ctx = luma::default_context().unwrap();
    for level in [0u32, 1] {
        let mut case = SubsonicCase::new(0.5, Nose::TangentOgive { fineness: 3.0 }, level, true);
        case.first_order_iters = 0;
        let host = case.build_solver();
        let d = ImplicitParams::DEFAULT;
        let variants: Vec<(&str, Scheme)> = vec![
            ("explicit SSP-RK3", Scheme::Explicit),
            ("implicit sub_iters 3 (default)", Scheme::Implicit(d)),
            ("implicit sub_iters 1", Scheme::Implicit(ImplicitParams { sub_iters: 1, ..d })),
            ("implicit sub_iters 2", Scheme::Implicit(ImplicitParams { sub_iters: 2, ..d })),
            ("implicit point (lines=false)", Scheme::Implicit(ImplicitParams { lines: false, ..d })),
        ];
        for prec in [Precision::Extended, Precision::F32] {
            for (name, sch) in &variants {
                let mut g = GpuSolver::from_host(&ctx, &host, Mode::Guarded, prec).unwrap();
                g.set_scheme(*sch).unwrap();
                g.run(1).unwrap();
                let mut best = f64::INFINITY;
                for _ in 0..3 {
                    g.reset().unwrap();
                    ctx.sync().unwrap();
                    let t0 = Instant::now();
                    for _ in 0..10 {
                        g.run(50).unwrap();
                    }
                    best = best.min(1e6 * t0.elapsed().as_secs_f64() / 500.0);
                }
                println!("L{level} {}x{} {prec:?} {name}: {best:.1} us/iter", host.grid.ni, host.grid.nj);
            }
        }
    }
}
