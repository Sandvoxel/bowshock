//! CPU runner: iterate a [`Solver`] built from a [`Case`] to convergence, the stopping rules and the post-processing of the
//! final state (wall Cp, forebody / afterbody / base pressure drag, field indicators).
//!
//! Lengths are in body diameters (`d = 1`, `R = 0.5`). Drag coefficients are pressure drag only (inviscid), referenced to the
//! maximum cross-section `pi R^2`; one formula covers every wall part, `CD = integral Cp d(r^2 / R^2)` along the surface from the
//! nose tip downstream (the base face is traversed from its outer to its inner radius, so `CD_base = -Cp_b A_b / A_ref`).

use crate::engine::body_grid::BaseBody;
use crate::engine::case::{Case, Wake};
use crate::engine::numerics::{DRAG_REL_WINDOW, MONITOR_EVERY};
use crate::engine::result::{BasePoint, PointResult, StopReason, WallPoint};
use bowshock_cfd::gas::{cons_to_prim, freestream, prim_to_cons, sound_speed};
use bowshock_cfd::solver::Solver;
use bowshock_cfd::viscous::temperature;
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
        if s.viscous().is_some() {
            // the turbulence state is not part of `u`: back to the free stream
            s.set_viscous(s.viscous());
        }
    }
    s.iter = if warm { n.first_order_iters } else { 0 };
    s.restart_ramp();
    let skip = if warm { 0 } else { 20 };
    let sample = case.wake == Wake::Base || n.monitor_tol.is_some() || n.drag_rel_tol.is_some();
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
        if let Some(why) = stop_reason(case, it, r, r_ref.unwrap_or(rmax), skip, &hist) {
            reason = why;
            break;
        }
    }
    finish(case, s, residual, rmax, hist, reason)
}

/// Largest magnitude of the convergence monitor (a drag or pressure coefficient) of a meaningful solution; a sample beyond it, or a
/// non-finite one, means the state has blown up (the residual of such a state can still be finite).
pub const MONITOR_LIMIT: f64 = 1e3;

/// Stopping rule after iteration `it` with density residual `r`, in this order: non-finite residual, or a monitor sample taken at
/// this iteration (the last of `hist`) that is non-finite or larger than [`MONITOR_LIMIT`] in magnitude (always); then, from
/// `min_iter` on, `drop_orders` below `reference` ("residual drop reached"); or, with a `drag_rel_tol`, a viscous case at a monitor
/// sample (`it + 1 >= DRAG_REL_WINDOW`, residual dropped 2 orders) whose total-CD range over the last `DRAG_REL_WINDOW` iterations
/// is below `drag_rel_tol` times the magnitude of its mean (`CdStationary`); or, with a `monitor_tol`, every 500 iterations
/// from `2 monitor_window` on and once the residual has dropped 2 orders, a monitor range (`hist`, one sample per `MONITOR_EVERY`
/// iterations) below the tolerance over the last `monitor_window` iterations (`BasePressureStationary` for an inviscid base wake,
/// `CdStationary` for a sting and for every viscous case, whose monitor is the total CD; the inviscid wake can be unsteady, the
/// residual then stalls); then, for a viscous case with a `mean_window` and at every monitor sample, [`mean_stationary`]
/// (`MeanStationary`). `skip` is 20 on a cold start, 0 warm.
pub fn stop_reason(case: &Case, it: usize, r: f64, reference: f64, skip: usize, hist: &[(usize, f64)]) -> Option<StopReason> {
    let n = &case.numerics;
    if !r.is_finite() || hist.last().is_some_and(|&(last, m)| last == it + 1 && (!m.is_finite() || m.abs() > MONITOR_LIMIT)) {
        return Some(StopReason::NonFinite);
    }
    if it + 1 < n.min_iter {
        return None;
    }
    if it > skip && reference / r >= 10f64.powf(n.drop_orders) {
        return Some(StopReason::ResidualDrop);
    }
    if let Some(tol) = n.drag_rel_tol {
        let k = DRAG_REL_WINDOW / MONITOR_EVERY;
        if case.viscous.is_some()
            && hist.last().is_some_and(|&(last, _)| last == it + 1)
            && it + 1 >= DRAG_REL_WINDOW
            && reference / r >= 100.0
            && hist.len() >= k
        {
            let w = &hist[hist.len() - k..];
            let (lo, hi) = w.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |(l, h), c| (l.min(c.1), h.max(c.1)));
            let mean = w.iter().map(|c| c.1).sum::<f64>() / k as f64;
            if hi - lo < tol * mean.abs() {
                return Some(StopReason::CdStationary);
            }
        }
    }
    if let Some(tol) = n.monitor_tol {
        if it % 500 == 499 && it + 1 >= 2 * n.monitor_window && reference / r >= 100.0 {
            let k = n.monitor_window / MONITOR_EVERY;
            let (lo, hi) = hist[hist.len() - k..].iter().fold((f64::INFINITY, f64::NEG_INFINITY), |(l, h), c| (l.min(c.1), h.max(c.1)));
            if hi - lo < tol {
                return Some(match (case.viscous.is_some(), case.wake) {
                    (false, Wake::Base) => StopReason::BasePressureStationary,
                    (true, _) | (false, Wake::Sting) => StopReason::CdStationary,
                });
            }
        }
    }
    if let (true, Some(window), Some(&(last, _))) = (case.viscous.is_some(), n.mean_window, hist.last()) {
        if last == it + 1 && mean_stationary(hist, window / MONITOR_EVERY) {
            return Some(StopReason::MeanStationary);
        }
    }
    None
}

/// Mean-stationary test of a monitor history on the samples with iteration `> 2000` (the start-up transient is not judged), with
/// `k` samples per averaging window: `m1` is the mean of the last `k` samples, `m0` the mean of the `k` before them and
/// `m = (m0 + m1) / 2`. True when there are `2k` such samples, `|m1 - m0| <= 2e-3 |m|` (the mean of the oscillation is stationary),
/// `max - min <= 0.05 |m|` over the last `2k` samples (the oscillation is bounded) and the last two samples lie on opposite sides
/// of `m` (the state is at the mean of the cycle). `k = 0` never holds.
pub fn mean_stationary(hist: &[(usize, f64)], k: usize) -> bool {
    let from = hist.partition_point(|&(it, _)| it <= 2000);
    let h = &hist[from..];
    if k == 0 || h.len() < 2 * k {
        return false;
    }
    let w = &h[h.len() - 2 * k..];
    let mean = |s: &[(usize, f64)]| s.iter().map(|c| c.1).sum::<f64>() / k as f64;
    let (m0, m1) = (mean(&w[..k]), mean(&w[k..]));
    let m = 0.5 * (m0 + m1);
    let (lo, hi) = w.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |(l, h), c| (l.min(c.1), h.max(c.1)));
    let (a, b) = (w[2 * k - 2].1 - m, w[2 * k - 1].1 - m);
    (m1 - m0).abs() <= 2e-3 * m.abs() && hi - lo <= 0.05 * m.abs() && a * b < 0.0
}

/// Convergence monitor of the current state: the total CD of a viscous case, else the nose CD (sting wake) or the area-mean base
/// Cp (base wake).
pub fn monitor(case: &Case, s: &Solver) -> f64 {
    let bb = &case.body;
    if case.viscous.is_some() {
        let (wall, cd_nose, _) = wall_distribution(bb, s);
        let fore = aft_drag(bb, &wall).iter().fold(cd_nose, |a, c| a + c);
        return fore + friction(case, s).0 + base_state(s, bb.body.radius().powi(2)).1;
    }
    match case.wake {
        Wake::Sting => wall_distribution(bb, s).1,
        Wake::Base => base_state(s, bb.body.radius().powi(2)).2,
    }
}

/// Pressure drag of each aft segment (`body.segments` order) from the extrapolated wall Cp of `wall`.
fn aft_drag(bb: &BaseBody, wall: &[WallPoint]) -> Vec<f64> {
    let mut cd_aft = vec![0.0; bb.segments.len()];
    let stations: Vec<f64> = (0..=bb.segments.len()).map(|k| bb.station(k).0).collect();
    for w in wall {
        // a face belongs to segment k when stations[k] <= x < stations[k + 1]; the sting extension behind the base has no
        // projected area and is not counted, nor is the nose
        if let Some(k) = (0..cd_aft.len()).find(|&k| w.x >= stations[k] && w.x < stations[k + 1]) {
            cd_aft[k] += w.cp * (w.s1 - w.s0);
        }
    }
    cd_aft
}

/// Skin-friction drag and the largest `y+` of the first cell over the no-slip body-surface faces ahead of the base plane
/// (the sting extension of a `Wake::Sting` grid and the surface of a sting behind the base are not part of the body): `(cd_friction, y_plus_max)`,
/// `(0.0, NaN)` for the Euler equations. With the wall tangent `t = (nr, -nx)` pointing downstream, the first-cell tangential
/// velocity `u_t`, its wall distance `d_n` and `tau_w = (mu(T) + mu_t,w) u_t / d_n` (`mu_t,w` the Aupoix wall eddy viscosity of a rough wall,
/// `Viscous::roughness > 0`, else 0): `CD = sum 2 tau_w t_x r_m len / (q_inf R^2)`, `y+ = d_n sqrt(rho tau_w) / mu`. Works on the
/// conserved state `s.u`.
pub fn friction(case: &Case, s: &Solver) -> (f64, f64) {
    let Some(v) = s.viscous() else {
        return (0.0, f64::NAN);
    };
    let g = &s.grid;
    let ni = g.ni;
    let fs = s.cfg.freestream;
    let q_inf = 0.5 * fs[0] * fs[1] * fs[1];
    let r2 = case.body.body.radius().powi(2);
    let x_base = case.body.x_base();
    let rows = match s.block() {
        None => vec![0],
        Some(b) => vec![b.j_end, b.j_sting],
    };
    let (mut cd, mut y_plus_max) = (0.0, 0.0f64);
    for j in rows {
        for i in 0..ni {
            if !s.noslip_jface(i, j) {
                continue;
            }
            let (n0, n1) = (g.node(i, j), g.node(i + 1, j));
            let xm = 0.5 * (n0.0 + n1.0);
            if xm >= x_base {
                continue;
            }
            let f = &g.jface[j * ni + i];
            // the fluid cell above the face
            let c = j * ni + i;
            let w = cons_to_prim(&s.u[c]);
            let (tx, tr) = (f.nr, -f.nx);
            let u_t = w[1] * tx + w[2] * tr;
            let d_n = ((g.xc[c] - xm) * f.nx + (g.rc[c] - f.rm) * f.nr).abs();
            let mu = v.mu(temperature(&w));
            // rough wall: the discrete wall shear is (mu + mu_t,w) u_t / d_n with the Aupoix wall eddy viscosity of the cell state (0 for a smooth wall)
            let mu_tw = if v.rough() { w[0] * bowshock_cfd::turbulence::aupoix_wall(mu / w[0], w[1].hypot(w[2]), d_n, v.roughness)[2] } else { 0.0 };
            let tau = (mu + mu_tw) * u_t / d_n;
            cd += 2.0 * tau * tx * f.rm * f.len / (q_inf * r2);
            y_plus_max = y_plus_max.max(d_n * (w[0] * tau.abs()).sqrt() / mu);
        }
    }
    (cd, y_plus_max)
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
    let j0 = s.nozzle().map_or(b.j_sting, |n| n.j_end);
    for j in j0..b.j_end {
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
    let cd_aft = aft_drag(bb, &wall);
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
    match (case.viscous.is_some(), case.wake) {
        (false, Wake::Base) => {
            if nwin >= 2 {
                let w = &hist[hist.len() - nwin..];
                let (lo, hi) = w.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |(l, h), c| (l.min(c.1), h.max(c.1)));
                cp_base_mean = w.iter().map(|c| c.1).sum::<f64>() / nwin as f64;
                cp_base_range = hi - lo;
            }
        }
        // the monitor is the forebody CD (sting wake) or, for a viscous run, the total CD
        (true, _) | (false, Wake::Sting) => {
            if nwin >= 2 {
                let (lo, hi) = hist[hist.len() - nwin..].iter().fold((f64::INFINITY, f64::NEG_INFINITY), |(l, h), c| (l.min(c.1), h.max(c.1)));
                cd_range = hi - lo;
            }
        }
    }
    let fore = cd_aft.iter().fold(cd_nose, |a, c| a + c);
    let (cd_friction, y_plus_max) = friction(case, s);
    let (cd_total, cd_total_model) = (fore + cd_friction + cd_base, fore + cd_friction + cd_base_model);
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
        cd_friction,
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
        y_plus_max,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::engine::body_grid::{Body, Nose};
    use crate::engine::numerics::Numerics;
    use bowshock_cfd::solver::{Turbulence, Viscous};

    fn case(tol: Option<f64>, wake: Wake) -> Case {
        let mut n = Numerics::transonic();
        n.monitor_tol = tol;
        n.min_iter = 0;
        Case::new(0.5, BaseBody::cylinder(Body { nose: Nose::TangentOgive { fineness: 3.0 } }, 6.0), 0, wake, n)
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
        let (c, cs) = (case(Some(1e-3), Wake::Base), case(Some(1e-3), Wake::Sting));
        // non-finite beats everything, even before min_iter
        let mut g = case(Some(1e-3), Wake::Base);
        g.numerics.min_iter = 10_000;
        assert_eq!(stop_reason(&g, 5, f64::NAN, 1.0, 0, &[]), Some(StopReason::NonFinite));
        assert_eq!(stop_reason(&g, 5, 1e-9, 1.0, 0, &[]), None);
        // so does a blown-up monitor sampled at this iteration, with a finite residual
        for bad in [f64::NAN, f64::INFINITY, 1.5e3, -1.5e3] {
            assert_eq!(stop_reason(&g, 49, 1e-9, 1.0, 0, &[(50, bad)]), Some(StopReason::NonFinite), "{bad}");
        }
        assert_eq!(stop_reason(&g, 49, 1e-9, 1.0, 0, &[(50, 900.0)]), None);
        assert_eq!(stop_reason(&g, 48, 1e-9, 1.0, 0, &[(50, 1.5e3)]), None);
        // residual drop before the stationary rule
        assert_eq!(stop_reason(&c, 4999, 1e-7, 1.0, 20, &flat), Some(StopReason::ResidualDrop));
        // stationary rule: needs it % 500 == 499, 2 orders of drop and a flat history
        assert_eq!(stop_reason(&c, 4999, 1e-3, 1.0, 20, &flat), Some(StopReason::BasePressureStationary));
        assert_eq!(stop_reason(&cs, 4999, 1e-3, 1.0, 20, &flat), Some(StopReason::CdStationary));
        assert_eq!(stop_reason(&c, 4998, 1e-3, 1.0, 20, &flat), None);
        assert_eq!(stop_reason(&c, 4999, 0.5, 1.0, 20, &flat), None);
        assert_eq!(stop_reason(&case(None, Wake::Base), 4999, 1e-3, 1.0, 20, &flat), None);
        let mut moving = flat.clone();
        moving[39].1 = 0.1;
        assert_eq!(stop_reason(&c, 4999, 1e-3, 1.0, 20, &moving), None);
        // a viscous case stops on a stationary total CD for either wake
        let mut v = case(Some(1e-3), Wake::Base);
        v.viscous = Some(Viscous { re_c: 1e6, sutherland: 0.38, turbulence: Turbulence::Sst, wall_i_start: 28, roughness: 0.0 });
        assert_eq!(stop_reason(&v, 4999, 1e-3, 1.0, 20, &flat), Some(StopReason::CdStationary));
    }

    #[test]
    fn relative_drag_rule_stops_a_flat_viscous_history() {
        let hist = |rel: f64| -> Vec<(usize, f64)> { (1..=10).map(|k| (k * 50, 0.4 * (1.0 + if k % 2 == 0 { rel } else { 0.0 }))).collect() };
        let mut v = case(None, Wake::Base);
        v.viscous = Some(Viscous { re_c: 1e6, sutherland: 0.38, turbulence: Turbulence::Sst, wall_i_start: 28, roughness: 0.0 });
        v.numerics.drag_rel_tol = Some(2e-3);
        // (a) relative range 1e-3, 3 orders of drop: stops at the first sample at iteration 500, not before
        assert_eq!(stop_reason(&v, 499, 1e-3, 1.0, 20, &hist(1e-3)), Some(StopReason::CdStationary));
        assert_eq!(stop_reason(&v, 498, 1e-3, 1.0, 20, &hist(1e-3)), None);
        assert_eq!(stop_reason(&v, 449, 1e-3, 1.0, 20, &hist(1e-3)[..9]), None);
        // (b) only 1 order of drop
        assert_eq!(stop_reason(&v, 499, 1e-1, 1.0, 20, &hist(1e-3)), None);
        // (c) relative range 1e-2
        assert_eq!(stop_reason(&v, 499, 1e-3, 1.0, 20, &hist(1e-2)), None);
        // an inviscid case ignores the rule
        let mut inv = v.clone();
        inv.viscous = None;
        assert_eq!(stop_reason(&inv, 499, 1e-3, 1.0, 20, &hist(1e-3)), None);
    }

    fn cycle(end: usize, mean: f64, amp: f64, drift_per_1000: f64) -> Vec<(usize, f64)> {
        (1..=end / 50)
            .map(|k| {
                let it = k * 50;
                (it, mean + amp * (std::f64::consts::TAU * it as f64 / 6000.0).sin() + drift_per_1000 * it as f64 / 1000.0)
            })
            .collect()
    }

    #[test]
    fn mean_stationary_stops_at_a_mean_crossing_of_a_limit_cycle() {
        let mut v = case(Some(1e-4), Wake::Base);
        v.viscous = Some(Viscous { re_c: 1e6, sutherland: 0.38, turbulence: Turbulence::Sst, wall_i_start: 28, roughness: 0.0 });
        v.numerics.mean_window = Some(6000);
        let full = cycle(40_000, 0.1475, 0.0015, 0.0);
        // first monitor sample that stops: not before 2 windows after the start-up, and with the last two samples across the mean
        let stop = (1..=full.len()).find(|&n| stop_reason(&v, full[n - 1].0 - 1, 1e-3, 1.0, 20, &full[..n]).is_some()).expect("a stop");
        let it = full[stop - 1].0;
        assert!(it >= 2000 + 2 * 6000, "stopped at {it}");
        assert_eq!(stop_reason(&v, it - 1, 1e-3, 1.0, 20, &full[..stop]), Some(StopReason::MeanStationary));
        let (a, b) = (full[stop - 2].1 - 0.1475, full[stop - 1].1 - 0.1475);
        assert!(a * b <= 0.0, "not at a mean crossing: {a} {b}");
        // a call between two monitor samples never stops
        assert_eq!(stop_reason(&v, it, 1e-3, 1.0, 20, &full[..stop]), None);
        // Euler (no viscous model) and a case without a mean window never use the rule
        let e = case(Some(1e-4), Wake::Base);
        assert_eq!(stop_reason(&e, it - 1, 1e-3, 1.0, 20, &full[..stop]), None);
        v.numerics.mean_window = None;
        assert_eq!(stop_reason(&v, it - 1, 1e-3, 1.0, 20, &full[..stop]), None);
    }

    #[test]
    fn mean_stationary_rejects_a_drifting_mean_and_a_large_oscillation() {
        let any = |h: &[(usize, f64)]| (240..=h.len()).any(|n| mean_stationary(&h[..n], 120));
        assert!(any(&cycle(40_000, 0.1475, 0.0015, 0.0)));
        // mean drift of 1.2e-3 per window (0.8 %) > 0.2 %
        assert!(!any(&cycle(40_000, 0.1475, 0.0015, 2e-4)));
        // peak-to-peak 8 % > 5 %
        assert!(!any(&cycle(40_000, 0.1475, 0.006, 0.0)));
        // too short a history: fewer than 2 windows after the start-up
        assert!(!mean_stationary(&cycle(13_000, 0.1475, 0.0015, 0.0), 120));
    }
}
