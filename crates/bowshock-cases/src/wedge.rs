//! The cone and ogive cases on the single-cell wedge of the 3D engine (`Case3::wedge`): the CPU Euler solver of the product, run on the 2D case's own meridian
//! grid revolved by the wedge angle, its final state written back into the 2D solver so the cases' own post-processing (wall pressure, shock angle, forebody
//! drag) reads it unchanged. Where the wedge reproduces the frozen 2D results (G-W1 and G-W4 in `tests/wedge_gates.rs`) a validation driver can run on it
//! (`--wedge`); the 2D runners stay until the 2D deletion.

use bowshock_cfd::flux::Limiter;
use bowshock_cfd::solver::Solver;
use bowshock_core::engine::case3::Case3;
use bowshock_core::engine::grid3::{wedge_of_meridian, WEDGE_ANGLE};
use bowshock_core::engine::result::StopReason;

/// What the iteration of a wedge run gives besides the state.
pub struct WedgeRun {
    /// Density residual (L2 of d rho/dt of the 3D solver) per iteration.
    pub residual: Vec<f64>,
    /// Orders between the peak residual of the second-order phase and the last one.
    pub drop_orders: f64,
    pub stop_reason: StopReason,
}

/// Iterates the flow of `s2` (its grid, its free stream: Mach `mach`) on the wedge until the density residual has dropped `drop_orders` orders from the peak of
/// the second-order phase (`first_order_iters` onward, as `StopRule3`), `max_iter`, or a plateau (the 2D runners' rule); writes the final state into `s2` (the
/// radial momentum from the wedge cell's `y`, `z` momenta) and refreshes its primitives.
pub fn run_on_wedge(s2: &mut Solver, mach: f64, limiter: Limiter, first_order_iters: usize, max_iter: usize, drop_orders: f64) -> Result<WedgeRun, String> {
    let g2 = &s2.grid;
    // the lower boundary is the axis for x < 0 and the body from the tip on
    let n_axis = (0..g2.ni).take_while(|&i| g2.x[i + 1] <= 1e-12).count();
    let grid = wedge_of_meridian(g2, WEDGE_ANGLE, n_axis).map_err(|e| e.to_string())?;
    // the case only sets the numerics of the Euler wedge (its body is where the forces would stop: past every grid)
    let mut case = Case3::wedge(mach, Case3::anf_body(), 0);
    case.numerics.limiter = limiter;
    case.numerics.first_order_iters = first_order_iters;
    let mut s3 = case.solver_on(grid);
    let mut residual = Vec::new();
    let mut rmax: f64 = 0.0;
    let mut reason = StopReason::MaxIterations;
    let from = first_order_iters.max(20);
    for it in 0..max_iter {
        let r = s3.step();
        residual.push(r);
        if it >= from {
            rmax = rmax.max(r);
        }
        if !r.is_finite() {
            reason = StopReason::NonFinite;
            break;
        }
        if it > from && rmax / r >= 10f64.powf(drop_orders) {
            reason = StopReason::ResidualDrop;
            break;
        }
        if it >= 8000 && it % 1000 == 0 {
            let m = |a: usize, b: usize| residual[a..b].iter().map(|x| x.log10()).sum::<f64>() / (b - a) as f64;
            if m(it - 2000, it) - m(it - 4000, it - 2000) > -0.05 {
                reason = StopReason::ResidualPlateau;
                break;
            }
        }
    }
    let (ni, nj) = (s2.grid.ni, s2.grid.nj);
    let g3 = &s3.grid;
    for j in 0..nj {
        for i in 0..ni {
            let c = g3.cell(i, j, 0);
            let (q, x) = (s3.u[c], g3.xc[c]);
            let r = x[1].hypot(x[2]);
            // the wedge cell's radial direction at its centroid: y = r sin(theta), z = r cos(theta)
            let m_r = if r > 0.0 { (q[2] * x[1] + q[3] * x[2]) / r } else { 0.0 };
            s2.u[j * ni + i] = [q[0], q[1], m_r, q[4]];
        }
    }
    s2.eval_residual(false);
    let drop_orders = if rmax > 0.0 { (rmax / residual.last().copied().unwrap_or(rmax)).log10() } else { 0.0 };
    Ok(WedgeRun { residual, drop_orders, stop_reason: reason })
}
