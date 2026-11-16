//! CPU runner of a [`Case3`]: iterate the [`Solver3`] to a converged or stationary normal force, and integrate the wall pressure into
//! the force coefficients.
//!
//! Lengths are in nose diameters, the coefficients are referenced to `S_ref = pi/4` (nose diameter 1; the facade rescales to the
//! reference diameter). Pressure forces only (Euler). The solver covers the half domain `theta in [0, pi]`, so forces are doubled.

use crate::engine::case3::Case3;
use crate::engine::run::MONITOR_LIMIT;
use bowshock_cfd::gas3::cons_to_prim3;
use bowshock_cfd::grid3::Face3;
use bowshock_cfd::solver3::Solver3;
use crate::engine::result::StopReason;

/// Iterations between samples of the normal force.
pub const SAMPLE_EVERY: usize = 50;
/// Orders of density-residual drop that end a run as [`StopReason::ResidualDrop`].
pub const DROP_ORDERS: f64 = 6.0;
/// Orders of drop required before the stationary rule may fire.
pub const STATIONARY_DROP_ORDERS: f64 = 2.0;

/// Pressure-force coefficients of the body plus fins.
#[derive(Clone, Copy, Debug)]
pub struct LiftForces {
    /// Normal force coefficient (`+z`, towards the lee side at positive incidence).
    pub cn: f64,
    /// Axial (pressure) force coefficient, `+x` downstream.
    pub ca: f64,
    /// Centre of pressure of the normal force from the nose tip, nose diameters; NaN when `|sum Fz| < 1e-12`.
    pub x_cp: f64,
}

/// Result of one 3D run.
#[derive(Clone, Debug)]
pub struct PointResult3 {
    pub mach: f64,
    /// Incidence \[rad\].
    pub alpha: f64,
    pub cn: f64,
    pub ca: f64,
    /// Nose diameters from the tip.
    pub x_cp: f64,
    pub iterations: usize,
    /// `ResidualDrop` (converged), `CdStationary` (CN stationary), `MaxIterations` or `NonFinite`.
    pub stop_reason: StopReason,
    /// Orders of density-residual drop below the reference (peak) at the end of the run.
    pub drop_orders: f64,
    /// `(iteration, CN)` every [`SAMPLE_EVERY`] iterations.
    pub cn_history: Vec<(usize, f64)>,
}

/// Wall forces of the current state of `s`: the `j = 0` faces on the body (`i >= axis_cells`, centroid `x <=` base station) and every
/// face between a fluid and a solid cell. See [`lift_forces_upto`] for the wall pressure.
pub fn lift_forces(case: &Case3, s: &Solver3) -> LiftForces {
    lift_forces_upto(case, s, case.body.x_base())
}

/// [`lift_forces`] restricted to the wall faces with centroid `x <= x_max`. The wall pressure is extrapolated linearly from the two
/// first fluid cells normal to the wall, as the 2D facade does (`engine::run::wall_distribution`): `pw = p0 + (p0 - p1) d0 / (d1 - d0)`,
/// `d` the distance from the face centre to the cell centroid. A fin face whose second cell is solid or outside the grid uses the first-cell
/// pressure. `dF = (pw - p_inf) area n_into_body`.
pub fn lift_forces_upto(case: &Case3, s: &Solver3, x_max: f64) -> LiftForces {
    let g = &s.grid;
    let (ni, nj, nk) = (g.ni, g.nj, g.nk);
    let p_inf = s.cfg.freestream[4];
    let p = |c: usize| cons_to_prim3(&s.u[c])[4];
    let solid = |i: usize, j: usize, k: usize| g.is_solid(i, j, k);
    let dist = |c: usize, f: &Face3| {
        let x = g.xc[c];
        ((x[0] - f.c[0]).powi(2) + (x[1] - f.c[1]).powi(2) + (x[2] - f.c[2]).powi(2)).sqrt()
    };
    let (mut fx, mut fz, mut mz) = (0.0, 0.0, 0.0);
    // `a`: first fluid cell, `b`: the fluid cell behind it (if any); `sign`: +1 when the face normal points into the body
    let mut add = |a: usize, b: Option<usize>, f: &Face3, sign: f64| {
        if f.c[0] > x_max {
            return;
        }
        let p0 = p(a);
        let pw = match b {
            Some(b) => {
                let (d0, d1) = (dist(a, f), dist(b, f));
                p0 + (p0 - p(b)) * d0 / (d1 - d0)
            }
            None => p0,
        };
        let d = (pw - p_inf) * f.area * sign;
        fx += d * f.n[0];
        fz += d * f.n[2];
        mz += d * f.n[2] * f.c[0];
    };
    for k in 0..nk {
        for i in g.axis_cells..ni {
            let f = &g.jface[k * (nj + 1) * ni + i];
            // normal points into the fluid: into the body is its negative
            if !solid(i, 0, k) {
                add(g.cell(i, 0, k), (nj > 1 && !solid(i, 1, k)).then(|| g.cell(i, 1, k)), f, -1.0);
            }
        }
    }
    for k in 0..nk {
        for j in 0..nj {
            for i in 1..ni {
                let f = &g.iface[(k * nj + j) * (ni + 1) + i];
                match (solid(i - 1, j, k), solid(i, j, k)) {
                    (false, true) => add(g.cell(i - 1, j, k), (i >= 2 && !solid(i - 2, j, k)).then(|| g.cell(i - 2, j, k)), f, 1.0),
                    (true, false) => add(g.cell(i, j, k), (i + 1 < ni && !solid(i + 1, j, k)).then(|| g.cell(i + 1, j, k)), f, -1.0),
                    _ => {}
                }
            }
        }
    }
    for k in 0..nk {
        for j in 1..nj {
            for i in 0..ni {
                let f = &g.jface[(k * (nj + 1) + j) * ni + i];
                match (solid(i, j - 1, k), solid(i, j, k)) {
                    (false, true) => add(g.cell(i, j - 1, k), (j >= 2 && !solid(i, j - 2, k)).then(|| g.cell(i, j - 2, k)), f, 1.0),
                    (true, false) => add(g.cell(i, j, k), (j + 1 < nj && !solid(i, j + 1, k)).then(|| g.cell(i, j + 1, k)), f, -1.0),
                    _ => {}
                }
            }
        }
    }
    for k in 1..nk {
        for j in 0..nj {
            for i in 0..ni {
                let f = &g.kface[(k * nj + j) * ni + i];
                match (solid(i, j, k - 1), solid(i, j, k)) {
                    (false, true) => add(g.cell(i, j, k - 1), (k >= 2 && !solid(i, j, k - 2)).then(|| g.cell(i, j, k - 2)), f, 1.0),
                    (true, false) => add(g.cell(i, j, k), (k + 1 < nk && !solid(i, j, k + 1)).then(|| g.cell(i, j, k + 1)), f, -1.0),
                    _ => {}
                }
            }
        }
    }
    let q = 0.5 * case.mach * case.mach;
    let s_ref = std::f64::consts::FRAC_PI_4;
    let norm = 2.0 / (q * s_ref);
    LiftForces { cn: norm * fz, ca: norm * fx, x_cp: if fz.abs() < 1e-12 { f64::NAN } else { mz / fz } }
}

/// Iterate `s` (built from `case`) until a stop rule fires, in this order at every iteration: a non-finite residual (or, at a sample,
/// CN) gives `NonFinite`; a residual drop of [`DROP_ORDERS`] orders below the peak residual (from iteration 20 cold, from the start
/// warm) gives `ResidualDrop`; at a sample with `it + 1 >= window`, a drop of [`STATIONARY_DROP_ORDERS`] and a range of the last
/// `window / SAMPLE_EVERY` CN samples below `rel_tol |mean|` gives `CdStationary`; the iteration cap gives `MaxIterations`.
///
/// `warm`: keep the field of the previous run, set the free stream of `case` and restart the CFL ramp (no first-order start-up).
pub fn run3_on(case: &Case3, s: &mut Solver3, warm: bool) -> PointResult3 {
    let n = &case.numerics;
    s.cfg.freestream = case.freestream();
    s.restart_ramp();
    if warm {
        s.iter = s.iter.max(n.first_order_iters);
    }
    let skip = if warm { 0 } else { 20 };
    let nwin = (n.window / SAMPLE_EVERY).max(1);
    let (mut rmax, mut r) = (0.0f64, f64::NAN);
    let mut hist: Vec<(usize, f64)> = Vec::new();
    let mut reason = StopReason::MaxIterations;
    let mut it = 0;
    while it < n.max_iter {
        r = s.step();
        if !r.is_finite() {
            reason = StopReason::NonFinite;
            it += 1;
            break;
        }
        if it >= skip {
            rmax = rmax.max(r);
        }
        let drop = if rmax > 0.0 { (rmax / r.max(1e-300)).log10() } else { 0.0 };
        if drop >= DROP_ORDERS {
            reason = StopReason::ResidualDrop;
            it += 1;
            break;
        }
        if (it + 1) % SAMPLE_EVERY == 0 {
            let cn = lift_forces(case, s).cn;
            if !cn.is_finite() || cn.abs() > MONITOR_LIMIT {
                reason = StopReason::NonFinite;
                it += 1;
                break;
            }
            hist.push((it + 1, cn));
            if it + 1 >= n.window && drop >= STATIONARY_DROP_ORDERS && hist.len() >= nwin {
                let w = &hist[hist.len() - nwin..];
                let (lo, hi, sum) = w.iter().fold((f64::INFINITY, f64::NEG_INFINITY, 0.0), |a, &(_, c)| (a.0.min(c), a.1.max(c), a.2 + c));
                if hi - lo < n.rel_tol * (sum / nwin as f64).abs() {
                    reason = StopReason::CdStationary;
                    it += 1;
                    break;
                }
            }
        }
        it += 1;
    }
    let f = lift_forces(case, s);
    if reason != StopReason::NonFinite && !(f.cn.is_finite() && f.ca.is_finite() && f.cn.abs() <= MONITOR_LIMIT) {
        reason = StopReason::NonFinite;
    }
    let drop_orders = if rmax > 0.0 && r.is_finite() { (rmax / r.max(1e-300)).log10() } else { 0.0 };
    PointResult3 { mach: case.mach, alpha: case.alpha, cn: f.cn, ca: f.ca, x_cp: f.x_cp, iterations: it, stop_reason: reason, drop_orders, cn_history: hist }
}

/// Cold run on a fresh solver.
pub fn run3_cold(case: &Case3) -> Result<PointResult3, crate::error::Error> {
    let mut s = case.build_solver()?;
    Ok(run3_on(case, &mut s, false))
}
