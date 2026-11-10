use bowshock_core::flux::Limiter;
use bowshock_core::solver::Scheme;
use bowshock_cases::transonic::TransonicCase;
use bowshock_gpu::{GpuSolver, Mode, Precision};
fn main() {
    let ctx = luma::default_context().unwrap();
    let mut case = TransonicCase::level(0.8, 0);
    case.limiter = Limiter::VanAlbada;
    case.scheme = Scheme::Implicit(bowshock_core::implicit::ImplicitParams::DEFAULT);
    case.first_order_iters = 100;
    let mut cpu = case.build_solver();
    let rc: Vec<f64> = (0..250).map(|_| cpu.step()).collect();
    let host = case.build_solver();
    let mut g = GpuSolver::from_host(&ctx, &host, Mode::Guarded, Precision::Extended).unwrap();
    let mut r = Vec::new();
    for _ in 0..5 {
        r.extend(g.run(50).unwrap());
    }
    for k in (0..250).step_by(10) {
        println!("[{k}] gpu {:.4e} cpu {:.4e} rel {:.2e}", r[k], rc[k], (r[k] - rc[k]).abs() / rc[k]);
    }
}
