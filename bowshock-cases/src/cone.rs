//! Cone validation case: set-up, steady-state iteration and post-processing against Taylor-Maccoll.

use bowshock_core::flux::Limiter;
use bowshock_core::gas::freestream;
use bowshock_core::grid::{cone_grid, ConeGridParams};
use bowshock_core::solver::{Bc, Config, Solver};
use bowshock_reference::taylor_maccoll::ConeSolution;

#[derive(Clone, Debug)]
pub struct ConeCase {
    pub mach: f64,
    pub theta_deg: f64,
    pub ni: usize,
    pub nj: usize,
    pub cfl: f64,
    pub limiter: Limiter,
    pub cluster: f64,
    pub max_iter: usize,
    /// Stop when the density residual has dropped this many orders from its maximum.
    pub drop_orders: f64,
    pub first_order_iters: usize,
    /// Averaging window (fraction of cone length) for cone Cp and the shock-angle fit.
    pub window: (f64, f64),
}

impl ConeCase {
    pub fn new(mach: f64, theta_deg: f64, ni: usize, nj: usize) -> Self {
        ConeCase {
            mach,
            theta_deg,
            ni,
            nj,
            cfl: 0.5,
            limiter: Limiter::Minmod,
            cluster: 0.0,
            max_iter: 40_000,
            drop_orders: 6.0,
            first_order_iters: 300,
            window: (0.4, 0.9),
        }
    }
}

pub struct ConeResult {
    pub iters: usize,
    /// Density residual (L2 of d rho/dt) per iteration.
    pub residual: Vec<f64>,
    pub drop_orders: f64,
    pub stop_reason: &'static str,
    /// Cone surface samples: (x/L, p/p_inf, Cp from extrapolated wall value, Cp of the first cell).
    pub surface: Vec<(f64, f64, f64, f64)>,
    pub cp_avg: f64,
    pub cp_cell_avg: f64,
    pub beta_num: f64,
    pub min_rho: f64,
    pub min_p: f64,
}

/// Build the case grid + solver. `beta` (shock angle estimate, rad) sizes the outer boundary.
pub fn build_solver(case: &ConeCase, beta: f64) -> Solver {
    let length = 1.0;
    let h_end = 1.25 * length * beta.tan();
    let gp = ConeGridParams {
        ni: case.ni,
        nj: case.nj,
        theta: case.theta_deg.to_radians(),
        length,
        x_up: -0.1 * length,
        h_start: 0.4 * h_end,
        h_end,
        cluster: case.cluster,
    };
    let cfg = Config {
        cfl: case.cfl,
        limiter: case.limiter,
        freestream: freestream(case.mach),
        bc: [Bc::Freestream, Bc::Extrapolate, Bc::SlipWall, Bc::Freestream],
        first_order_iters: case.first_order_iters,
    };
    Solver::new(cone_grid(&gp), cfg)
}

pub fn run_cone(case: &ConeCase, tm: &ConeSolution) -> ConeResult {
    let mut s = build_solver(case, tm.beta);
    let mut residual = Vec::new();
    let mut rmax: f64 = 0.0;
    let mut reason = "max iterations";
    for it in 0..case.max_iter {
        let r = s.step();
        residual.push(r);
        if it >= 20 {
            rmax = rmax.max(r);
        }
        if !r.is_finite() {
            reason = "non-finite residual";
            break;
        }
        if it > 20 && rmax / r >= 10f64.powf(case.drop_orders) {
            reason = "residual drop reached";
            break;
        }
        // plateau: mean log10 residual over last 2000 vs previous 2000 iterations
        if it >= 8000 && it % 1000 == 0 {
            let m = |a: usize, b: usize| residual[a..b].iter().map(|x| x.log10()).sum::<f64>() / (b - a) as f64;
            if m(it - 2000, it) - m(it - 4000, it - 2000) > -0.05 {
                reason = "residual plateau";
                break;
            }
        }
    }
    // refresh primitives from the final state
    s.eval_residual(false);
    let drop_orders = if rmax > 0.0 { (rmax / residual.last().copied().unwrap_or(rmax)).log10() } else { 0.0 };
    postprocess(case, &s, residual, drop_orders, reason)
}

fn postprocess(case: &ConeCase, s: &Solver, residual: Vec<f64>, drop_orders: f64, reason: &'static str) -> ConeResult {
    let g = &s.grid;
    let (ni, nj) = (g.ni, g.nj);
    let fs = s.cfg.freestream;
    let p_inf = fs[3];
    let q_inf = 0.5 * fs[0] * fs[1] * fs[1];
    let cp = |p: f64| (p - p_inf) / q_inf;
    let length = g.x[ni];

    // ---- surface pressure (wall face value by linear extrapolation from cells j=0, 1) ----
    let mut surface = Vec::new();
    for i in 0..ni {
        let xm = 0.5 * (g.node(i, 0).0 + g.node(i + 1, 0).0);
        if xm <= 0.0 {
            continue;
        }
        let rm = 0.5 * (g.node(i, 0).1 + g.node(i + 1, 0).1);
        let (p0, p1) = (s.pressure(i, 0), s.pressure(i, 1));
        let d0 = (g.xc[i] - xm).hypot(g.rc[i] - rm);
        let d1 = (g.xc[ni + i] - xm).hypot(g.rc[ni + i] - rm);
        let pw = p0 + (p0 - p1) * d0 / (d1 - d0);
        surface.push((xm / length, pw / p_inf, cp(pw), cp(p0)));
    }
    let (lo, hi) = case.window;
    let sel: Vec<_> = surface.iter().filter(|r| r.0 >= lo && r.0 <= hi).collect();
    let cp_avg = sel.iter().map(|r| r.2).sum::<f64>() / sel.len() as f64;
    let cp_cell_avg = sel.iter().map(|r| r.3).sum::<f64>() / sel.len() as f64;

    // ---- shock angle: per column locate the steepest pressure gradient (parabolic sub-cell
    //      refinement) and fit the line r = a + b x through those points over the window ----
    let mut pts = Vec::new();
    for i in 0..ni {
        let xr = g.xc[i] / length;
        if xr < lo || xr > hi {
            continue;
        }
        // gradient magnitude on the segment between cell centres j and j+1
        let gr: Vec<f64> = (0..nj - 1)
            .map(|j| {
                let (ca, cb) = (j * ni + i, (j + 1) * ni + i);
                let d = (g.xc[cb] - g.xc[ca]).hypot(g.rc[cb] - g.rc[ca]);
                (s.pressure(i, j) - s.pressure(i, j + 1)).abs() / d
            })
            .collect();
        let k = (0..gr.len()).max_by(|&a, &b| gr[a].total_cmp(&gr[b])).unwrap();
        let mut frac = k as f64;
        if k > 0 && k + 1 < gr.len() {
            let (a, b, c) = (gr[k - 1], gr[k], gr[k + 1]);
            let den = a - 2.0 * b + c;
            if den < 0.0 {
                frac += (0.5 * (a - c) / den).clamp(-0.5, 0.5);
            }
        }
        // segment k spans centres k..k+1; its midpoint sits at index k+0.5
        let pos = |arr: &[f64], f: f64| {
            let t = f.clamp(0.0, (nj - 2) as f64);
            let j0 = t.floor() as usize;
            let a = 0.5 * (arr[j0 * ni + i] + arr[(j0 + 1) * ni + i]); // midpoint of segment j0
            let j1 = (j0 + 1).min(nj - 2);
            let b = 0.5 * (arr[j1 * ni + i] + arr[(j1 + 1) * ni + i]);
            a + (t - j0 as f64) * (b - a)
        };
        pts.push((pos(&g.xc, frac), pos(&g.rc, frac)));
    }
    let n = pts.len() as f64;
    let (sx, sr) = (pts.iter().map(|p| p.0).sum::<f64>(), pts.iter().map(|p| p.1).sum::<f64>());
    let sxx = pts.iter().map(|p| p.0 * p.0).sum::<f64>();
    let sxr = pts.iter().map(|p| p.0 * p.1).sum::<f64>();
    let slope = (n * sxr - sx * sr) / (n * sxx - sx * sx);
    let beta_num = slope.atan();

    let mut min_rho = f64::INFINITY;
    let mut min_p = f64::INFINITY;
    for q in &s.u {
        let w = bowshock_core::gas::cons_to_prim(q);
        min_rho = min_rho.min(w[0]);
        min_p = min_p.min(w[3]);
    }
    ConeResult {
        iters: residual.len(),
        residual,
        drop_orders,
        stop_reason: reason,
        surface,
        cp_avg,
        cp_cell_avg,
        beta_num,
        min_rho,
        min_p,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use bowshock_reference::taylor_maccoll::solve_cone;

    /// Regression for the convergence stall (README, "Convergence stall"): with SSP-RK2 + plain HLLC this case
    /// plateaued at ~4 orders (density residual ~3e-4); it must now reach 6 orders well inside the budget.
    #[test]
    fn minmod_10deg_200x100_converges_six_orders() {
        let tm = solve_cone(2.0, 10f64.to_radians()).unwrap();
        let mut case = ConeCase::new(2.0, 10.0, 200, 100);
        case.max_iter = 6000;
        let res = run_cone(&case, &tm);
        assert_eq!(res.stop_reason, "residual drop reached", "only {:.1} orders in {} iters", res.drop_orders, res.iters);
        assert!((res.cp_avg - tm.cp_surface).abs() / tm.cp_surface < 0.003, "Cp {} vs TM {}", res.cp_avg, tm.cp_surface);
    }
}
