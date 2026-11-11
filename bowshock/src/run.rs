//! CPU runner: iterate a [`Solver`] built from a [`Case`] to convergence, the stopping rules and the post-processing of the
//! final state (wall Cp, forebody / afterbody / base pressure drag, field indicators).
//!
//! Lengths are in body diameters (`d = 1`, `R = 0.5`). Drag coefficients are pressure drag only (inviscid), referenced to the
//! maximum cross-section `pi R^2`; one formula covers every wall part, `CD = integral Cp d(r^2 / R^2)` along the surface from the
//! nose tip downstream (the base face is traversed from its outer to its inner radius, so `CD_base = -Cp_b A_b / A_ref`).

use crate::body_grid::BaseBody;
use crate::case::{Case, Wake};
use crate::numerics::{Numerics, MONITOR_EVERY};
use crate::result::{BasePoint, PointResult, StopReason, WallPoint};
use bowshock_core::gas::{cons_to_prim, freestream, prim_to_cons, sound_speed};
use bowshock_core::solver::Solver;
use bowshock_reference::base_drag::base_drag_coefficient;

/// Cold-start run on a fresh solver.
pub fn run_cold(case: &Case) -> PointResult {
    let mut s = case.build_solver();
    run_on(case, &mut s, false, None)
}

/// Iterate `s` (built from `case`) to convergence. `warm`: keep the field of the previous run, only the free stream changes
/// (no first-order start-up; the implicit CFL ramp restarts). `r_ref`: residual reference of the drop criterion (`None` = peak
/// of this run after iteration 20 / from the start when warm).
pub fn run_on(case: &Case, s: &mut Solver, warm: bool, r_ref: Option<f64>) -> PointResult {
    let n = case.numerics;
    s.cfg.freestream = freestream(case.mach);
    s.cfg.cfl = n.cfl;
    s.cfg.limiter = n.limiter;
    s.cfg.first_order_iters = n.first_order_iters;
    s.low_mach = n.low_mach;
    if s.scheme() != n.scheme {
        s.set_scheme(n.scheme);
    }
    if !warm {
        let q = prim_to_cons(&freestream(case.mach));
        s.u.fill(q);
    }
    s.iter = if warm { n.first_order_iters } else { 0 };
    s.restart_ramp();
    let skip = if warm { 0 } else { 20 };
    let sample = case.wake == Wake::Base || n.monitor_tol.is_some();
    let mut residual = Vec::new();
    let mut hist: Vec<(usize, f64)> = Vec::new();
    let mut rmax: f64 = 0.0;
    let mut reason = StopReason::MaxIterations;
    for it in 0..n.max_iter {
        let r = s.step();
        residual.push(r);
        if it >= skip {
            rmax = rmax.max(r);
        }
        if sample && it % MONITOR_EVERY == MONITOR_EVERY - 1 {
            hist.push((it + 1, monitor(case, s)));
        }
        if let Some(why) = stop_reason(&n, case.wake, it, r, r_ref.unwrap_or(rmax), skip, &hist) {
            reason = why;
            break;
        }
    }
    finish(case, s, residual, rmax, hist, reason)
}

/// Stopping rule after iteration `it` with density residual `r`, in this order: non-finite residual (always); then, from
/// `n.min_iter` on, `n.drop_orders` below `reference` ("residual drop reached"); or, with `n.monitor_tol`, every 500 iterations
/// from `2 n.monitor_window` on and once the residual has dropped 2 orders, a monitor range (`hist`, one sample per `MONITOR_EVERY`
/// iterations) below the tolerance over the last `n.monitor_window` iterations (`BasePressureStationary` for a base wake,
/// `CdStationary` for a sting; the inviscid wake can be unsteady, the residual then stalls). `skip` is 20 on a cold start, 0 warm.
pub fn stop_reason(n: &Numerics, wake: Wake, it: usize, r: f64, reference: f64, skip: usize, hist: &[(usize, f64)]) -> Option<StopReason> {
    if !r.is_finite() {
        return Some(StopReason::NonFinite);
    }
    if it + 1 < n.min_iter {
        return None;
    }
    if it > skip && reference / r >= 10f64.powf(n.drop_orders) {
        return Some(StopReason::ResidualDrop);
    }
    if let Some(tol) = n.monitor_tol {
        if it % 500 == 499 && it + 1 >= 2 * n.monitor_window && reference / r >= 100.0 {
            let k = n.monitor_window / MONITOR_EVERY;
            let (lo, hi) = hist[hist.len() - k..].iter().fold((f64::INFINITY, f64::NEG_INFINITY), |(l, h), c| (l.min(c.1), h.max(c.1)));
            if hi - lo < tol {
                return Some(match wake {
                    Wake::Base => StopReason::BasePressureStationary,
                    Wake::Sting => StopReason::CdStationary,
                });
            }
        }
    }
    None
}

/// Convergence monitor of the current state: the nose CD (sting wake) or the area-mean base Cp (base wake).
pub fn monitor(case: &Case, s: &Solver) -> f64 {
    match case.wake {
        Wake::Sting => wall_distribution(&case.body, s).1,
        Wake::Base => base_state(s, case.body.body.radius().powi(2)).2,
    }
}

/// Wall points of the body (and of the sting extension) in axial order, and the nose drag from the extrapolated wall Cp and from
/// the first-cell Cp: `(wall, cd_nose, cd_nose_cell)`.
pub fn wall_distribution(body: &BaseBody, s: &Solver) -> (Vec<WallPoint>, f64, f64) {
    let g = &s.grid;
    let ni = g.ni;
    let fs = s.cfg.freestream;
    let (p_inf, q_inf) = (fs[3], 0.5 * fs[0] * fs[1] * fs[1]);
    let cp = |p: f64| (p - p_inf) / q_inf;
    let (ln, rb2) = (body.body.nose_length(), body.body.radius().powi(2));
    let blk = s.block();
    let mut wall = Vec::new();
    let (mut cd, mut cd_cell) = (0.0, 0.0);
    let i_wall_end = blk.map_or(ni, |b| b.i_end);
    for i in 0..i_wall_end {
        let j0 = blk.map_or(0, |b| b.col_start(i));
        let (n0, n1) = (g.node(i, j0), g.node(i + 1, j0));
        let (xm, rm) = (0.5 * (n0.0 + n1.0), 0.5 * (n0.1 + n1.1));
        if xm <= 0.0 {
            continue;
        }
        let (p0, p1) = (s.pressure(i, j0), s.pressure(i, j0 + 1));
        let d0 = (g.xc[j0 * ni + i] - xm).hypot(g.rc[j0 * ni + i] - rm);
        let d1 = (g.xc[(j0 + 1) * ni + i] - xm).hypot(g.rc[(j0 + 1) * ni + i] - rm);
        let pw = p0 + (p0 - p1) * d0 / (d1 - d0);
        let (s0, s1) = (n0.1 * n0.1 / rb2, n1.1 * n1.1 / rb2);
        if n1.0 <= ln + 1e-12 {
            cd += cp(pw) * (s1 - s0);
            cd_cell += cp(p0) * (s1 - s0);
        }
        wall.push(WallPoint { x: xm, r: rm, s0, s1, cp: cp(pw), cp_cell: cp(p0) });
    }
    (wall, cd, cd_cell)
}

/// Base-face samples and integrals of the current state: `(samples, CD_base, area-mean Cp_b, area-mean first-cell Cp_b)`;
/// NaN means / empty samples without a base. Wall pressure by linear extrapolation from the two first cells behind the base.
pub fn base_state(s: &Solver, rb2: f64) -> (Vec<BasePoint>, f64, f64, f64) {
    let Some(b) = s.block() else {
        return (Vec::new(), 0.0, f64::NAN, f64::NAN);
    };
    let g = &s.grid;
    let ni = g.ni;
    let fs = s.cfg.freestream;
    let (p_inf, q_inf) = (fs[3], 0.5 * fs[0] * fs[1] * fs[1]);
    let cp = |p: f64| (p - p_inf) / q_inf;
    let ie = b.i_end;
    let (mut pts, mut cd, mut mean, mut mean_cell, mut s_sum) = (Vec::new(), 0.0, 0.0, 0.0, 0.0);
    for j in b.j_sting..b.j_end {
        let (n0, n1) = (g.node(ie, j), g.node(ie, j + 1));
        let (xm, rm) = (n0.0, 0.5 * (n0.1 + n1.1));
        let (p0, p1) = (s.pressure(ie, j), s.pressure(ie + 1, j));
        let d0 = (g.xc[j * ni + ie] - xm).hypot(g.rc[j * ni + ie] - rm);
        let d1 = (g.xc[j * ni + ie + 1] - xm).hypot(g.rc[j * ni + ie + 1] - rm);
        let pw = p0 + (p0 - p1) * d0 / (d1 - d0);
        let ds = (n1.1 * n1.1 - n0.1 * n0.1) / rb2;
        cd -= cp(pw) * ds;
        mean += cp(pw) * ds;
        mean_cell += cp(p0) * ds;
        s_sum += ds;
        pts.push(BasePoint { r0: n0.1, r1: n1.1, cp: cp(pw), cp_cell: cp(p0) });
    }
    (pts, cd, mean / s_sum, mean_cell / s_sum)
}

/// Post-process the state in `s` (works for a host `Solver` whose `u` was overwritten with another solver's state).
pub fn finish(case: &Case, s: &Solver, residual: Vec<f64>, rmax: f64, hist: Vec<(usize, f64)>, reason: StopReason) -> PointResult {
    let g = &s.grid;
    let (ni, nj) = (g.ni, g.nj);
    let fs = s.cfg.freestream;
    let p_inf = fs[3];
    let n = &case.numerics;
    let bb = &case.body;
    let rb2 = bb.body.radius().powi(2);
    let blk = s.block();
    let (wall, cd_nose, cd_nose_cell) = wall_distribution(bb, s);
    let mut cd_aft = vec![0.0; bb.segments.len()];
    let stations: Vec<f64> = (0..=bb.segments.len()).map(|k| bb.station(k).0).collect();
    for w in &wall {
        // a face belongs to segment k when stations[k] <= x < stations[k + 1]; the sting extension behind the base has no
        // projected area and is not counted, nor is the nose
        if let Some(k) = (0..cd_aft.len()).find(|&k| w.x >= stations[k] && w.x < stations[k + 1]) {
            cd_aft[k] += w.cp * (w.s1 - w.s0);
        }
    }
    let (base_wall, cd_base, cp_base, cp_base_cell) = base_state(s, rb2);
    let cd_base_model = match (case.wake, bb.sting_radius > 0.0) {
        (Wake::Sting, _) => 0.0,
        (Wake::Base, false) => base_drag_coefficient(case.mach) * bb.base_radius().powi(2) / rb2,
        (Wake::Base, true) => f64::NAN,
    };

    let (cp_min, x_cp_min) = wall.iter().fold((f64::INFINITY, 0.0), |(m, xm), w| if w.cp < m { (w.cp, w.x) } else { (m, xm) });
    let i_wall_end = blk.map_or(ni, |b| b.i_end);
    let mut m_wall_max: f64 = 0.0;
    for i in 0..i_wall_end {
        let j0 = blk.map_or(0, |b| b.col_start(i));
        if s.grid.xc[j0 * ni + i] > 0.0 {
            let w = cons_to_prim(&s.u[j0 * ni + i]);
            m_wall_max = m_wall_max.max(w[1].hypot(w[2]) / sound_speed(&w));
        }
    }
    let dev = |i: usize, j: usize| (s.pressure(i, j) / p_inf - 1.0).abs();
    let mut farfield_dp: f64 = 0.0;
    for i in 0..ni {
        farfield_dp = farfield_dp.max(dev(i, nj - 1));
    }
    for j in 0..nj {
        farfield_dp = farfield_dp.max(dev(0, j)).max(dev(ni - 1, j));
    }
    let (mut min_rho, mut min_p, mut m_max) = (f64::INFINITY, f64::INFINITY, 0.0f64);
    for (c, q) in s.u.iter().enumerate() {
        if blk.is_some_and(|b| b.solid(c % ni, c / ni)) {
            continue;
        }
        let w = cons_to_prim(q);
        min_rho = min_rho.min(w[0]);
        min_p = min_p.min(w[3]);
        m_max = m_max.max(w[1].hypot(w[2]) / sound_speed(&w));
    }
    let nwin = (n.monitor_window / MONITOR_EVERY).min(hist.len());
    let (mut cp_base_mean, mut cp_base_range, mut cd_range) = (cp_base, f64::NAN, f64::NAN);
    match case.wake {
        Wake::Base => {
            if nwin >= 2 {
                let w = &hist[hist.len() - nwin..];
                let (lo, hi) = w.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |(l, h), c| (l.min(c.1), h.max(c.1)));
                cp_base_mean = w.iter().map(|c| c.1).sum::<f64>() / nwin as f64;
                cp_base_range = hi - lo;
            }
        }
        Wake::Sting => {
            if nwin >= 2 {
                let (lo, hi) = hist[hist.len() - nwin..].iter().fold((f64::INFINITY, f64::NEG_INFINITY), |(l, h), c| (l.min(c.1), h.max(c.1)));
                cd_range = hi - lo;
            }
        }
    }
    let fore = cd_aft.iter().fold(cd_nose, |a, c| a + c);
    let (cd_total, cd_total_model) = (fore + cd_base, fore + cd_base_model);
    let last = residual.last().copied().unwrap_or(rmax);
    let drop_orders = if rmax > 0.0 && last > 0.0 { (rmax / last).log10() } else { 0.0 };
    PointResult {
        mach: case.mach,
        alpha: 0.0,
        iters: residual.len(),
        residual,
        residual_peak: rmax,
        drop_orders,
        stop_reason: reason,
        wall,
        base_wall,
        cd_nose,
        cd_nose_cell,
        cd_aft,
        cd_base,
        cd_total,
        cd_base_model,
        cd_total_model,
        cp_base,
        cp_base_cell,
        cp_base_mean,
        cp_base_range,
        cd_range,
        monitor_history: hist,
        cp_min,
        x_cp_min,
        m_max,
        m_wall_max,
        farfield_dp,
        min_rho,
        min_p,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn n(tol: Option<f64>) -> Numerics {
        let mut n = Numerics::transonic();
        n.monitor_tol = tol;
        n.min_iter = 0;
        n
    }

    #[test]
    fn stop_reason_display_padding() {
        assert_eq!(format!("{:<22}|", StopReason::CdStationary), "CD stationary         |");
        assert_eq!(StopReason::BasePressureStationary.to_string(), "base pressure stationary");
        assert_eq!(StopReason::NonFinite.to_string(), "non-finite residual");
    }

    #[test]
    fn stop_reason_order_and_gates() {
        let flat: Vec<(usize, f64)> = (1..=40).map(|k| (k * 50, -0.1)).collect();
        let c = n(Some(1e-3));
        // non-finite beats everything, even before min_iter
        let mut g = c;
        g.min_iter = 10_000;
        assert_eq!(stop_reason(&g, Wake::Base, 5, f64::NAN, 1.0, 0, &[]), Some(StopReason::NonFinite));
        assert_eq!(stop_reason(&g, Wake::Base, 5, 1e-9, 1.0, 0, &[]), None);
        // residual drop before the stationary rule
        assert_eq!(stop_reason(&c, Wake::Base, 4999, 1e-7, 1.0, 20, &flat), Some(StopReason::ResidualDrop));
        // stationary rule: needs it % 500 == 499, 2 orders of drop and a flat history
        assert_eq!(stop_reason(&c, Wake::Base, 4999, 1e-3, 1.0, 20, &flat), Some(StopReason::BasePressureStationary));
        assert_eq!(stop_reason(&c, Wake::Sting, 4999, 1e-3, 1.0, 20, &flat), Some(StopReason::CdStationary));
        assert_eq!(stop_reason(&c, Wake::Base, 4998, 1e-3, 1.0, 20, &flat), None);
        assert_eq!(stop_reason(&c, Wake::Base, 4999, 0.5, 1.0, 20, &flat), None);
        assert_eq!(stop_reason(&n(None), Wake::Base, 4999, 1e-3, 1.0, 20, &flat), None);
        let mut moving = flat.clone();
        moving[39].1 = 0.1;
        assert_eq!(stop_reason(&c, Wake::Base, 4999, 1e-3, 1.0, 20, &moving), None);
    }
}
