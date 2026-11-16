//! CPU runner of a [`Case3`]: iterate the [`Solver3`] to a converged or stationary normal force, and integrate the wall pressure into
//! the force coefficients.
//!
//! Lengths are in nose diameters, the coefficients are referenced to `S_ref = pi/4` (nose diameter 1; the facade rescales to the
//! reference diameter). Pressure forces only (Euler). The solver covers the half domain `theta in [0, pi]`, so forces are doubled.

use crate::engine::case3::Case3;
use crate::engine::result::StopReason;
use crate::engine::run::MONITOR_LIMIT;
use crate::progress::{drop_orders, spread, Progress};
use bowshock_cfd::gas3::cons_to_prim3;
use bowshock_cfd::gas3::Cons3;
use bowshock_cfd::grid3::{Face3, Grid3};
use bowshock_cfd::solver3::Solver3;
use std::ops::ControlFlow;

/// Iterations between samples of the normal force.
pub const SAMPLE_EVERY: usize = 50;
/// Orders of density-residual drop that end a run as [`StopReason::ResidualDrop`].
pub const DROP_ORDERS: f64 = 6.0;
/// Iterations before the stationary rule may fire (it has no residual condition: the residual of a fin-body flow can plateau at
/// 1 to 2 orders of drop with the CN flat to four digits).
pub const STATIONARY_MIN_ITERS: usize = 1000;

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
    forces_on(case, &s.grid, &s.u, s.cfg.freestream[4], x_max)
}

/// [`lift_forces_upto`] on a grid, a conserved state `u` (index `Grid3::cell`, e.g. downloaded from the GPU) and the free-stream pressure.
pub fn forces_on(case: &Case3, g: &Grid3, u: &[Cons3], p_inf: f64, x_max: f64) -> LiftForces {
    let acc = accumulate(g, u, p_inf, x_max, 0.0);
    let (mut fx, mut fz, mut mz) = (0.0, 0.0, 0.0);
    for a in &acc {
        fx += a[0];
        fz += a[1];
        mz += a[2];
    }
    coefficients(case, [fx, fz, mz])
}

/// Number of force parts of [`LiftBreakdown`].
const PARTS: usize = 5;

/// Pressure-force coefficients of one part of the wetted surface; `x_cp` (nose diameters from the tip) is NaN when `|sum Fz| < 1e-12`.
#[derive(Clone, Copy, Debug)]
pub struct PartForces {
    pub cn: f64,
    pub ca: f64,
    pub x_cp: f64,
}

/// [`LiftForces`] split by the surface the pressure acts on. The parts sum to [`forces_on`] (`x_max` = the base station).
#[derive(Clone, Copy, Debug)]
pub struct LiftBreakdown {
    /// Body wall (`j = 0` faces) up to the end of the nose.
    pub nose: PartForces,
    /// Body wall behind the nose up to the base station.
    pub afterbody: PartForces,
    /// Fin side faces (faces between a fluid and a solid cell in `k`).
    pub fin_sides: PartForces,
    /// Fin leading and trailing edge faces (`i` faces of the fin boxes).
    pub fin_edges: PartForces,
    /// Fin tip faces (`j` faces of the fin boxes).
    pub fin_tips: PartForces,
}

impl LiftBreakdown {
    /// The whole fin: sides, edges and tip.
    pub fn fins(&self) -> PartForces {
        let parts = [self.fin_sides, self.fin_edges, self.fin_tips];
        let cn: f64 = parts.iter().map(|p| p.cn).sum();
        let ca: f64 = parts.iter().map(|p| p.ca).sum();
        let m: f64 = parts.iter().filter(|p| p.x_cp.is_finite()).map(|p| p.cn * p.x_cp).sum();
        PartForces { cn, ca, x_cp: if cn.abs() < 1e-12 { f64::NAN } else { m / cn } }
    }
}

/// [`forces_on`] split into nose, afterbody and fin faces (the nose ends at the nose length of `case.body`, the body wall at its base
/// station).
pub fn breakdown_on(case: &Case3, g: &Grid3, u: &[Cons3], p_inf: f64) -> LiftBreakdown {
    let acc = accumulate(g, u, p_inf, case.body.x_base(), case.body.body.nose_length());
    let part = |a: [f64; 3]| {
        let f = coefficients(case, a);
        PartForces { cn: f.cn, ca: f.ca, x_cp: f.x_cp }
    };
    LiftBreakdown { nose: part(acc[0]), afterbody: part(acc[1]), fin_sides: part(acc[2]), fin_edges: part(acc[3]), fin_tips: part(acc[4]) }
}

fn coefficients(case: &Case3, [fx, fz, mz]: [f64; 3]) -> LiftForces {
    let q = 0.5 * case.mach * case.mach;
    let s_ref = std::f64::consts::FRAC_PI_4;
    let norm = 2.0 / (q * s_ref);
    LiftForces { cn: norm * fz, ca: norm * fx, x_cp: if fz.abs() < 1e-12 { f64::NAN } else { mz / fz } }
}

/// Dimensional force sums `[Fx, Fz, Mz]` (half domain, `Mz = sum x Fz`) of the five parts: body wall up to `x_nose`, body wall from
/// `x_nose` to `x_max`, fin `k` faces, fin `i` faces, fin `j` faces.
fn accumulate(g: &Grid3, u: &[Cons3], p_inf: f64, x_max: f64, x_nose: f64) -> [[f64; 3]; PARTS] {
    let (ni, nj, nk) = (g.ni, g.nj, g.nk);
    let p = |c: usize| cons_to_prim3(&u[c])[4];
    let solid = |i: usize, j: usize, k: usize| g.is_solid(i, j, k);
    let dist = |c: usize, f: &Face3| {
        let x = g.xc[c];
        ((x[0] - f.c[0]).powi(2) + (x[1] - f.c[1]).powi(2) + (x[2] - f.c[2]).powi(2)).sqrt()
    };
    let mut acc = [[0.0; 3]; PARTS];
    // `a`: first fluid cell, `b`: the fluid cell behind it (if any); `sign`: +1 when the face normal points into the body
    let mut add = |part: usize, a: usize, b: Option<usize>, f: &Face3, sign: f64| {
        if f.c[0] > x_max {
            return;
        }
        let part = if part == 0 && f.c[0] > x_nose { 1 } else { part };
        let p0 = p(a);
        let pw = match b {
            Some(b) => {
                let (d0, d1) = (dist(a, f), dist(b, f));
                p0 + (p0 - p(b)) * d0 / (d1 - d0)
            }
            None => p0,
        };
        let d = (pw - p_inf) * f.area * sign;
        acc[part][0] += d * f.n[0];
        acc[part][1] += d * f.n[2];
        acc[part][2] += d * f.n[2] * f.c[0];
    };
    for k in 0..nk {
        for i in g.axis_cells..ni {
            let f = &g.jface[k * (nj + 1) * ni + i];
            // normal points into the fluid: into the body is its negative
            if !solid(i, 0, k) {
                add(0, g.cell(i, 0, k), (nj > 1 && !solid(i, 1, k)).then(|| g.cell(i, 1, k)), f, -1.0);
            }
        }
    }
    for k in 0..nk {
        for j in 0..nj {
            for i in 1..ni {
                let f = &g.iface[(k * nj + j) * (ni + 1) + i];
                match (solid(i - 1, j, k), solid(i, j, k)) {
                    (false, true) => add(3, g.cell(i - 1, j, k), (i >= 2 && !solid(i - 2, j, k)).then(|| g.cell(i - 2, j, k)), f, 1.0),
                    (true, false) => add(3, g.cell(i, j, k), (i + 1 < ni && !solid(i + 1, j, k)).then(|| g.cell(i + 1, j, k)), f, -1.0),
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
                    (false, true) => add(4, g.cell(i, j - 1, k), (j >= 2 && !solid(i, j - 2, k)).then(|| g.cell(i, j - 2, k)), f, 1.0),
                    (true, false) => add(4, g.cell(i, j, k), (j + 1 < nj && !solid(i, j + 1, k)).then(|| g.cell(i, j + 1, k)), f, -1.0),
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
                    (false, true) => add(2, g.cell(i, j, k - 1), (k >= 2 && !solid(i, j, k - 2)).then(|| g.cell(i, j, k - 2)), f, 1.0),
                    (true, false) => add(2, g.cell(i, j, k), (k + 1 < nk && !solid(i, j, k + 1)).then(|| g.cell(i, j, k + 1)), f, -1.0),
                    _ => {}
                }
            }
        }
    }
    acc
}

/// Iterate `s` (built from `case`) until a stop rule fires, in this order at every iteration: a non-finite residual (or, at a sample,
/// CN) gives `NonFinite`; a residual drop of [`DROP_ORDERS`] orders below the peak residual (from iteration 20 cold, from the start
/// warm) gives `ResidualDrop`; at a sample with `it + 1 >= max(window, STATIONARY_MIN_ITERS)` and a range of the last
/// `window / SAMPLE_EVERY` CN samples below `rel_tol |mean|` (whatever the residual did) gives `CdStationary`; the iteration cap
/// gives `MaxIterations`.
///
/// `warm`: keep the field of the previous run, set the free stream of `case` and restart the CFL ramp (no first-order start-up).
pub fn run3_on(case: &Case3, s: &mut Solver3, warm: bool) -> PointResult3 {
    s.cfg.freestream = case.freestream();
    s.restart_ramp();
    if warm {
        s.iter = s.iter.max(case.numerics.first_order_iters);
    }
    let mut rule = StopRule3::new(case, warm);
    let mut it = 0;
    while it < case.numerics.max_iter {
        let r = s.step();
        let cn = ((it + 1) % SAMPLE_EVERY == 0).then(|| lift_forces(case, s).cn);
        it += 1;
        if rule.check(it - 1, r, cn).is_some() {
            break;
        }
        if cn.is_some() && rule.report(case, it).is_break() {
            rule.cancel();
            break;
        }
    }
    rule.finish(case, lift_forces(case, s), it)
}

/// The stop rules of a 3D run, shared by the CPU and GPU runners. Feed it every iteration's residual (and, at iterations
/// `it + 1 = 0 mod SAMPLE_EVERY`, the sampled CN); see [`run3_on`] for the order of the rules.
pub struct StopRule3 {
    skip: usize,
    nwin: usize,
    window: usize,
    rel_tol: f64,
    rmax: f64,
    r: f64,
    hist: Vec<(usize, f64)>,
    reason: StopReason,
}

impl StopRule3 {
    pub fn new(case: &Case3, warm: bool) -> StopRule3 {
        let n = &case.numerics;
        StopRule3 {
            skip: if warm { 0 } else { 20 },
            nwin: (n.window / SAMPLE_EVERY).max(1),
            window: n.window,
            rel_tol: n.rel_tol,
            rmax: 0.0,
            r: f64::NAN,
            hist: Vec::new(),
            reason: StopReason::MaxIterations,
        }
    }

    /// Reports the run's progress after `iterations` iterations to the case's sink; `Break` when the sink asks to stop, and `Continue`
    /// without a sink.
    pub fn report(&self, case: &Case3, iterations: usize) -> ControlFlow<()> {
        let Some(sink) = &case.progress else { return ControlFlow::Continue(()) };
        sink.report(&Progress {
            mach: case.mach,
            alpha: case.alpha,
            iteration: iterations,
            max_iterations: case.numerics.max_iter,
            residual_drop_orders: drop_orders(self.rmax, self.r),
            monitor: self.hist.last().map_or(f64::NAN, |h| h.1),
            spread: spread(&self.hist, self.nwin),
            tolerance: Some(self.rel_tol),
        })
    }

    /// Marks the run as ended by the sink: [`StopRule3::finish`] reports [`StopReason::Cancelled`].
    pub fn cancel(&mut self) {
        self.reason = StopReason::Cancelled;
    }

    /// Residual `r` of iteration `it` (0-based) and, at a sample iteration, the CN of the state after it. `Some(reason)` ends the run
    /// with `it + 1` iterations.
    pub fn check(&mut self, it: usize, r: f64, cn: Option<f64>) -> Option<StopReason> {
        self.r = r;
        let stop = |s: &mut Self, why| {
            s.reason = why;
            Some(why)
        };
        if !r.is_finite() {
            return stop(self, StopReason::NonFinite);
        }
        if it >= self.skip {
            self.rmax = self.rmax.max(r);
        }
        let drop = if self.rmax > 0.0 { (self.rmax / r.max(1e-300)).log10() } else { 0.0 };
        if drop >= DROP_ORDERS {
            return stop(self, StopReason::ResidualDrop);
        }
        if let Some(cn) = cn {
            if !cn.is_finite() || cn.abs() > MONITOR_LIMIT {
                return stop(self, StopReason::NonFinite);
            }
            self.hist.push((it + 1, cn));
            if it + 1 >= self.window.max(STATIONARY_MIN_ITERS) && self.hist.len() >= self.nwin {
                let w = &self.hist[self.hist.len() - self.nwin..];
                // a state that no longer changes (every sample bit-identical) without a residual drop is a frozen run, not a converged one
                if drop < 1.0 && w.iter().all(|&(_, c)| c.to_bits() == w[0].1.to_bits()) {
                    return stop(self, StopReason::NonFinite);
                }
                let (lo, hi, sum) = w.iter().fold((f64::INFINITY, f64::NEG_INFINITY, 0.0), |a, &(_, c)| (a.0.min(c), a.1.max(c), a.2 + c));
                if hi - lo < self.rel_tol * (sum / self.nwin as f64).abs() {
                    return stop(self, StopReason::CdStationary);
                }
            }
        }
        None
    }

    /// Result after `iterations` iterations with the final forces `f`.
    pub fn finish(self, case: &Case3, f: LiftForces, iterations: usize) -> PointResult3 {
        let mut reason = self.reason;
        if reason != StopReason::NonFinite && !(f.cn.is_finite() && f.ca.is_finite() && f.cn.abs() <= MONITOR_LIMIT) {
            reason = StopReason::NonFinite;
        }
        let drop_orders = if self.rmax > 0.0 && self.r.is_finite() { (self.rmax / self.r.max(1e-300)).log10() } else { 0.0 };
        PointResult3 {
            mach: case.mach,
            alpha: case.alpha,
            cn: f.cn,
            ca: f.ca,
            x_cp: f.x_cp,
            iterations,
            stop_reason: reason,
            drop_orders,
            cn_history: self.hist,
        }
    }
}

/// Cold run on a fresh solver.
pub fn run3_cold(case: &Case3) -> Result<PointResult3, crate::error::Error> {
    let mut s = case.build_solver()?;
    Ok(run3_on(case, &mut s, false))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Feeds a synthetic run: residual `res(it)` every iteration, `cn(it + 1)` at the sample iterations. The stop, if any.
    fn feed(res: impl Fn(usize) -> f64, cn: impl Fn(usize) -> f64) -> Option<(usize, StopReason)> {
        let mut rule = StopRule3::new(&Case3::tiny_anf(2.0, 2f64.to_radians()), false);
        for it in 0..3000 {
            let sample = (it + 1) % SAMPLE_EVERY == 0;
            if let Some(why) = rule.check(it, res(it), sample.then(|| cn(it + 1))) {
                return Some((it + 1, why));
            }
        }
        None
    }

    #[test]
    fn a_frozen_state_without_a_residual_drop_is_a_failure() {
        // every CN sample bit-identical, the residual flat at 0.5: frozen, not stationary
        let stop = feed(|_| 0.5, |_| 0.316_928_495_035_045);
        assert_eq!(stop, Some((1000, StopReason::NonFinite)));
    }

    #[test]
    fn a_stationary_state_with_a_residual_drop_still_stops_stationary() {
        // identical CN samples but the residual fell 2 orders (a converged limit): CdStationary as before
        let stop = feed(|it| if it < 100 { 1.0 } else { 1e-2 }, |_| 0.3);
        assert_eq!(stop, Some((1000, StopReason::CdStationary)));
        // a CN that wiggles at round-off level without a drop is not frozen (not bit-identical): CdStationary
        let stop = feed(|_| 0.5, |it| 0.3 + 1e-9 * (it % 3) as f64);
        assert_eq!(stop, Some((1000, StopReason::CdStationary)));
    }
}
