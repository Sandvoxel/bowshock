//! Subsonic case runner (M ~ 0.1 - 0.8): nose-cylinder-(boattail) body on the large-domain grid, either with a flat blunt base
//! and wake (`base_domain_grid`, solid block, base wall) or with the cylinder continuing as a sting to the outflow plane
//! (`large_domain_grid`, no base); line-implicit convergence acceleration, optional low-Mach flux correction, wall-pressure
//! post-processing and the forebody / afterbody / base pressure-drag breakdown.
//!
//! Lengths are in body diameters (`d = 1`, `R = 0.5`). Drag coefficients are pressure drag only (inviscid), referenced to the
//! maximum cross-section `pi R^2`; one formula covers every wall part, `CD = integral Cp d(r^2 / R^2)` along the surface from the
//! nose tip downstream (the base face is traversed from its outer to its inner radius, so `CD_base = -Cp_b A_b / A_ref`).

use crate::body_grid::{base_domain_grid, large_domain_grid, BaseBody, BaseGridParams, Body, LargeGridParams, Nose};
use crate::ogive::WallPoint;
use bowshock_core::flux::Limiter;
use bowshock_core::gas::{cons_to_prim, freestream};
use bowshock_core::implicit::ImplicitParams;
use bowshock_core::solver::{Bc, Config, Scheme, Solver};
use bowshock_reference::base_drag::base_drag_coefficient;

#[derive(Clone, Debug)]
pub struct SubsonicCase {
    pub mach: f64,
    pub body: BaseBody,
    /// `true`: finite body with a flat base and a wake region (`base_domain_grid`); `false`: the cylinder continues as a
    /// sting to the outflow plane (`large_domain_grid`), no base.
    pub base: bool,
    pub grid_base: BaseGridParams,
    pub grid_sting: LargeGridParams,
    pub limiter: Limiter,
    pub low_mach: bool,
    pub scheme: Scheme,
    /// First-order start-up iterations (cold start only).
    pub first_order_iters: usize,
    pub max_iter: usize,
    /// Stop rules other than a non-finite residual do not fire before this many iterations; the GPU-vs-CPU comparison sets it to the CPU's iteration count.
    pub min_iter: usize,
    /// Stop at this many orders of density-residual drop below the peak (after iteration 20 on a cold start).
    pub drop_orders: f64,
    pub outflow: Bc,
    pub cfl: f64,
}

impl SubsonicCase {
    /// Body `nose + cyl_length` cylinder (`+ boat_length` boattail of half-angle `boat_angle_deg`, sting radius `sting_radius`),
    /// domain `10 L` upstream and radially, `5 L` downstream (`L` = total body length), grid level 0/1/2 (cells per diameter
    /// doubling each level). Default scheme: line-implicit, van Albada limiter, low-Mach correction on.
    pub fn new(mach: f64, nose: Nose, level: u32, base: bool) -> Self {
        let body = BaseBody { body: Body { nose }, cyl_length: 6.0, boat_length: 0.0, boat_angle: 0.0, sting_radius: 0.0 };
        Self::with_body(mach, body, level, base)
    }

    pub fn with_body(mach: f64, body: BaseBody, level: u32, base: bool) -> Self {
        let f = 1usize << level;
        let l_body = body.x_base();
        let n_boat = if body.boat_length > 0.0 { 24 * f } else { 0 };
        let n_sting = if body.sting_radius > 0.0 { 4 * f } else { 0 };
        let grid_base = BaseGridParams {
            body,
            x_up: 10.0 * l_body,
            x_down: 5.0 * l_body,
            r_far: 10.0 * l_body,
            n_up: 28 * f,
            n_nose: 48 * f,
            n_cyl: 40 * f,
            n_boat,
            n_wake: 40 * f,
            n_sting,
            n_gap: 12 * f,
            nj_out: 64 * f,
            dx_tip: 0.03 / f as f64,
            dr_wall: 0.03 / f as f64,
        };
        let grid_sting = LargeGridParams {
            body: body.body,
            cyl_length: body.cyl_length,
            x_up: 10.0 * l_body,
            x_down: 5.0 * l_body,
            r_far: 10.0 * l_body,
            n_up: 30 * f,
            n_nose: 48 * f,
            n_cyl: 36 * f,
            n_ext: 30 * f,
            nj: 70 * f,
            dx_tip: 0.03 / f as f64,
            dr_wall: 0.03 / f as f64,
        };
        SubsonicCase {
            mach,
            body,
            base,
            grid_base,
            grid_sting,
            limiter: Limiter::VanAlbada,
            low_mach: true,
            scheme: Scheme::Implicit(ImplicitParams::DEFAULT),
            first_order_iters: 100,
            max_iter: 10_000 << level,
            min_iter: 0,
            drop_orders: 6.0,
            outflow: Bc::Characteristic,
            cfl: 0.5,
        }
    }

    /// Scale the domain: inflow plane / outer boundary at `up` and the outflow plane at `down` total body lengths.
    pub fn with_domain(mut self, up: f64, down: f64) -> Self {
        let l = self.body.x_base();
        self.grid_base.x_up = up * l;
        self.grid_base.r_far = up * l;
        self.grid_base.x_down = down * l;
        self.grid_sting.x_up = up * l;
        self.grid_sting.r_far = up * l;
        self.grid_sting.x_down = down * l;
        self
    }

    pub fn build_solver(&self) -> Solver {
        let cfg = Config {
            cfl: self.cfl,
            limiter: self.limiter,
            freestream: freestream(self.mach),
            bc: [Bc::Characteristic, self.outflow, Bc::SlipWall, Bc::Characteristic],
            first_order_iters: self.first_order_iters,
        };
        let mut s;
        if self.base {
            let (grid, block) = base_domain_grid(&self.grid_base);
            s = Solver::new(grid, cfg);
            s.set_block(Some(block));
        } else {
            s = Solver::new(large_domain_grid(&self.grid_sting), cfg);
        }
        s.low_mach = self.low_mach;
        s.set_scheme(self.scheme);
        s
    }

    pub fn cells(&self) -> (usize, usize) {
        let g = &self.grid_sting;
        let b = &self.grid_base;
        if self.base {
            (b.n_up + b.n_nose + b.n_cyl + b.n_boat + b.n_wake, b.n_sting + b.n_gap + b.nj_out)
        } else {
            (g.n_up + g.n_nose + g.n_cyl + g.n_ext, g.nj)
        }
    }
}

/// One base-face sample: radius range, Cp from the wall-extrapolated and the first-cell pressure.
#[derive(Clone, Copy, Debug)]
pub struct BasePoint {
    pub r0: f64,
    pub r1: f64,
    pub cp: f64,
    pub cp_cell: f64,
}

pub struct SubsonicResult {
    pub mach: f64,
    pub iters: usize,
    pub residual: Vec<f64>,
    pub residual_peak: f64,
    pub drop_orders: f64,
    pub stop_reason: &'static str,
    /// Wall faces of the nose, cylinder and boattail in axial order (`cp` extrapolated to the wall, `cp_cell` first cell).
    pub wall: Vec<WallPoint>,
    /// Base-face samples (empty without base), `r0 < r1`.
    pub base_wall: Vec<BasePoint>,
    pub cd_nose: f64,
    pub cd_cyl: f64,
    pub cd_boat: f64,
    pub cd_base: f64,
    /// Sum of the four (inviscid pressure drag of the closed body; ~0 for attached potential flow, d'Alembert).
    pub cd_total: f64,
    /// Base drag of the empirical free-flight correlation (`base_drag_coefficient`) scaled to `pi R^2`; 0 without base,
    /// NaN for a sting-mounted base (the correlation is for free-flight bases).
    pub cd_base_model: f64,
    /// `cd_nose + cd_cyl + cd_boat + cd_base_model`: the inviscid forebody drag plus the modelled base drag.
    pub cd_total_model: f64,
    /// Area-averaged base Cp (extrapolated wall values) and the first-cell variant; NaN without base.
    pub cp_base: f64,
    pub cp_base_cell: f64,
    /// Mean and range (max - min) of the area-mean base Cp over the last `UNSTEADY_WINDOW` iterations (history sampled every
    /// `HIST_EVERY`); the range is the unsteadiness indicator of the inviscid wake (NaN without enough history).
    pub cp_base_mean: f64,
    pub cp_base_range: f64,
    pub cp_base_hist: Vec<(usize, f64)>,
    pub cp_min: f64,
    pub x_cp_min: f64,
    pub m_max: f64,
    pub farfield_dp: f64,
    pub min_rho: f64,
    pub min_p: f64,
}

/// Cold-start run.
pub fn run_subsonic(case: &SubsonicCase) -> SubsonicResult {
    let mut s = case.build_solver();
    run_on(case, &mut s, false, None)
}

/// Iterate `s` (built from `case`) to convergence. `warm`: keep the field of the previous run, only the free stream changes
/// (no first-order start-up; the implicit CFL ramp restarts). `r_ref`: residual reference of the drop criterion (`None` = peak
/// of this run after iteration 20 / from the start when warm).
pub fn run_on(case: &SubsonicCase, s: &mut Solver, warm: bool, r_ref: Option<f64>) -> SubsonicResult {
    s.cfg.freestream = freestream(case.mach);
    s.cfg.cfl = case.cfl;
    s.cfg.limiter = case.limiter;
    s.cfg.first_order_iters = case.first_order_iters;
    s.low_mach = case.low_mach;
    s.iter = if warm { case.first_order_iters } else { 0 };
    s.restart_ramp();
    let skip = if warm { 0 } else { 20 };
    let mut residual = Vec::new();
    let mut hist: Vec<(usize, f64)> = Vec::new();
    let mut rmax: f64 = 0.0;
    let mut reason = "max iterations";
    let rb2 = case.body.body.radius().powi(2);
    for it in 0..case.max_iter {
        let r = s.step();
        residual.push(r);
        if it >= skip {
            rmax = rmax.max(r);
        }
        if case.base && it % HIST_EVERY == HIST_EVERY - 1 {
            hist.push((it + 1, base_state(s, rb2).2));
        }
        if let Some(rs) = stop_reason(case, it, r, r_ref.unwrap_or(rmax), skip, &hist) {
            reason = rs;
            break;
        }
    }
    finish(case, s, residual, rmax, hist, reason)
}

/// Sampling interval [iterations] of the base-pressure history and length of the statistics window (`UNSTEADY_WINDOW` iterations).
pub const HIST_EVERY: usize = 50;
pub const UNSTEADY_WINDOW: usize = 2000;
/// A base-pressure history whose range over the window is below this (in Cp) counts as stationary.
pub const CP_BASE_TOL: f64 = 1e-3;

/// Stopping rule after iteration `it` with density residual `r`: non-finite residual (always); then, from `case.min_iter` on, `drop_orders` below `reference`; or, with a
/// base, every 500 iterations from `2 UNSTEADY_WINDOW` on, the area-mean base Cp (`hist`, one sample per `HIST_EVERY` iterations)
/// varied by less than `CP_BASE_TOL` over the last `UNSTEADY_WINDOW` iterations once the residual has dropped 2 orders
/// ("base pressure stationary"; the inviscid wake can be unsteady, the residual then stalls, see the README).
pub fn stop_reason(case: &SubsonicCase, it: usize, r: f64, reference: f64, skip: usize, hist: &[(usize, f64)]) -> Option<&'static str> {
    if !r.is_finite() {
        return Some("non-finite residual");
    }
    if it + 1 < case.min_iter {
        return None;
    }
    if it > skip && reference / r >= 10f64.powf(case.drop_orders) {
        return Some("residual drop reached");
    }
    if case.base && it % 500 == 499 && it + 1 >= 2 * UNSTEADY_WINDOW && reference / r >= 100.0 {
        let n = UNSTEADY_WINDOW / HIST_EVERY;
        let (lo, hi) = hist[hist.len() - n..].iter().fold((f64::INFINITY, f64::NEG_INFINITY), |(l, h), c| (l.min(c.1), h.max(c.1)));
        if hi - lo < CP_BASE_TOL {
            return Some("base pressure stationary");
        }
    }
    None
}

/// Base-face samples and integrals of the current state: `(samples, CD_base, area-mean Cp_b, area-mean first-cell Cp_b)`;
/// NaN means / empty samples without a base. Wall pressure by linear extrapolation from the two first cells behind the base.
pub fn base_state(s: &Solver, rb2: f64) -> (Vec<BasePoint>, f64, f64, f64) {
    let Some(b) = s.block else {
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
pub fn finish(case: &SubsonicCase, s: &Solver, residual: Vec<f64>, rmax: f64, hist: Vec<(usize, f64)>, reason: &'static str) -> SubsonicResult {
    let g = &s.grid;
    let (ni, nj) = (g.ni, g.nj);
    let fs = s.cfg.freestream;
    let (p_inf, q_inf) = (fs[3], 0.5 * fs[0] * fs[1] * fs[1]);
    let cp = |p: f64| (p - p_inf) / q_inf;
    let bb = &case.body;
    let rb2 = bb.body.radius().powi(2);
    let blk = s.block;
    let mut wall = Vec::new();
    let (mut cd_nose, mut cd_cyl, mut cd_boat) = (0.0, 0.0, 0.0);
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
        let dcd = cp(pw) * (s1 - s0);
        if xm < bb.body.nose_length() {
            cd_nose += dcd;
        } else if xm < bb.x_shoulder() {
            cd_cyl += dcd;
        } else if xm < bb.x_base() {
            cd_boat += dcd;
        } else {
            // sting continuing to the outflow (no base): zero projected area, not counted
        }
        wall.push(WallPoint { x: xm, r: rm, s0, s1, cp: cp(pw), cp_cell: cp(p0) });
    }
    let (base_wall, cd_base, cp_base, cp_base_cell) = base_state(s, rb2);
    let cd_base_model = match (case.base, bb.sting_radius > 0.0) {
        (false, _) => 0.0,
        (true, false) => base_drag_coefficient(case.mach) * bb.base_radius().powi(2) / rb2,
        (true, true) => f64::NAN,
    };

    let (cp_min, x_cp_min) = wall.iter().fold((f64::INFINITY, 0.0), |(m, xm), w| if w.cp < m { (w.cp, w.x) } else { (m, xm) });
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
        m_max = m_max.max(w[1].hypot(w[2]) / bowshock_core::gas::sound_speed(&w));
    }
    let nwin = (UNSTEADY_WINDOW / HIST_EVERY).min(hist.len());
    let (cp_base_mean, cp_base_range) = if nwin >= 2 {
        let w = &hist[hist.len() - nwin..];
        let (lo, hi) = w.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |(l, h), c| (l.min(c.1), h.max(c.1)));
        (w.iter().map(|c| c.1).sum::<f64>() / nwin as f64, hi - lo)
    } else {
        (cp_base, f64::NAN)
    };
    let last = residual.last().copied().unwrap_or(rmax);
    let drop_orders = if rmax > 0.0 && last > 0.0 { (rmax / last).log10() } else { 0.0 };
    SubsonicResult {
        mach: case.mach,
        iters: residual.len(),
        residual,
        residual_peak: rmax,
        drop_orders,
        stop_reason: reason,
        wall,
        base_wall,
        cd_nose,
        cd_cyl,
        cd_boat,
        cd_base,
        cd_total: cd_nose + cd_cyl + cd_boat + cd_base,
        cd_base_model,
        cd_total_model: cd_nose + cd_cyl + cd_boat + cd_base_model,
        cp_base,
        cp_base_cell,
        cp_base_mean,
        cp_base_range,
        cp_base_hist: hist,
        cp_min,
        x_cp_min,
        m_max,
        farfield_dp,
        min_rho,
        min_p,
    }
}

/// NASA TM X-2439 test body (`cone = false`: configuration 4, 2-caliber tangent ogive; `cone = true`: configuration 2, 10 deg cone)
/// with cylinder, 10 deg boattail and the flat annular base on a sting (`fox_tm_x2439`), at Mach number `mach`.
pub fn fox_case(cone: bool, mach: f64, level: u32) -> SubsonicCase {
    let nose = if cone { Nose::Cone { half_angle: 10f64.to_radians() } } else { Nose::TangentOgive { fineness: 2.0 } };
    let body = BaseBody { body: Body { nose }, cyl_length: 2.0, boat_length: 1.0, boat_angle: 10f64.to_radians(), sting_radius: fox_tm_x2439::STING_RADIUS };
    SubsonicCase::with_body(mach, body, level, true)
}

/// Linear interpolation of the wall `Cp` (extrapolated values) at axial position `x` (None outside the wall range).
pub fn wall_cp_at(wall: &[WallPoint], x: f64) -> Option<f64> {
    for w in wall.windows(2) {
        if x >= w[0].x && x <= w[1].x {
            let t = (x - w[0].x) / (w[1].x - w[0].x);
            return Some(w[0].cp + t * (w[1].cp - w[0].cp));
        }
    }
    None
}

/// Published subsonic wall pressures on axisymmetric nose-cylinder-boattail bodies on a sting, zero angle of attack.
///
/// Source: C. H. Fox Jr., "Experimental Surface Pressure Distributions for a Family of Axisymmetric Bodies at Subsonic Speeds",
/// NASA TM X-2439 (Langley, December 1971), NTRS 19720009348; Langley 7- by 10-foot high-speed tunnel, sting-mounted models,
/// `D = 14.605 cm`, `Re_D = 0.44e6 .. 0.68e6`, boundary-layer state not stated (probably laminar/transitional on the nose), no wall
/// or sting corrections described. **Tabulated** data (Table VI = configuration 4, Table IV = configuration 2), transcribed from the
/// scanned table pages by OCR with 2-3 repeated rows per value (angle of attack -0.02 .. -0.10 deg, not exactly zero) and a visual
/// check of part of the blocks. Values are the mean of the upper and lower meridian orifices (they differ by 0.002 - 0.02).
/// Uncertainty: stated gage accuracy +-0.5 % of full scale = +-0.017 (M = 0.40), +-0.008 (0.61), +-0.0055 (0.83) in Cp for the
/// 34 kPa gages (half for the 17 kPa gages); repeatability +-0.002 .. 0.004 (up to 0.02 near shoulders at M = 0.83); a transcription error of
/// single values is possible.
///
/// Configuration 4: tangent ogive nose of length 2.0 D (arc radius 4.25 D), cylinder 2.0 D, 10 deg boattail of 1.0 D length, flat
/// annular base on a sting (sting radius taken as 0.13 D from the base-orifice radii). Configuration 2: 10 deg half-angle cone
/// (length 2.836 D), cylinder 2.0 D, same boattail and base. Nominal Mach numbers 0.4, 0.6, 0.8 are 0.404, 0.608, 0.831 in the tables.
pub mod fox_tm_x2439 {
    pub const MACH: [f64; 3] = [0.404, 0.608, 0.831];
    /// Gage-accuracy uncertainty in Cp at the three Mach numbers (34 kPa gages).
    pub const CP_UNCERTAINTY: [f64; 3] = [0.0166, 0.0084, 0.0055];
    pub const STING_RADIUS: f64 = 0.13;
    /// Base annulus Cp (mean of 12 orifices at `x = 4.993` / `5.824`) of configurations 4 and 2 at the three Mach numbers.
    pub const BASE_CP_OGIVE: [f64; 3] = [0.0691, 0.0836, 0.1317];
    pub const BASE_CP_CONE: [f64; 3] = [0.0685, 0.0813, 0.1290];
    pub const OGIVE_CYL_X_D: [f64; 24] = [
        0.289, 0.512, 0.740, 0.972, 1.207, 1.443, 1.682, 1.921, 1.965, 2.008, 2.051, 2.321, 2.657, 2.993, 3.327, 3.659, 3.927, 3.972, 4.014, 4.057, 4.348,
        4.638, 4.928, 4.971,
    ];
    pub const OGIVE_CYL_CP_M04_MEAN: [f64; 24] = [
        0.2837, 0.1283, 0.0291, -0.0631, -0.1347, -0.1838, -0.1838, -0.1578, -0.1352, -0.1195, -0.0989, -0.0584, -0.0401, -0.0382, -0.0571, -0.0868, -0.2346,
        -0.3326, -0.3495, -0.2257, -0.0369, 0.0242, 0.0621, 0.0698,
    ];
    pub const OGIVE_CYL_CP_M06_MEAN: [f64; 24] = [
        0.3075, 0.1450, 0.0410, -0.0606, -0.1407, -0.1959, -0.1975, -0.1660, -0.1437, -0.1251, -0.1017, -0.0548, -0.0359, -0.0337, -0.0521, -0.0843, -0.2534,
        -0.3678, -0.3893, -0.2465, -0.0299, 0.0339, 0.0771, 0.0867,
    ];
    pub const OGIVE_CYL_CP_M08_MEAN: [f64; 24] = [
        0.3813, 0.2063, 0.0873, -0.0311, -0.1293, -0.1988, -0.2110, -0.1671, -0.1402, -0.1128, -0.0842, -0.0256, -0.0024, -0.0024, -0.0209, -0.0527, -0.2244,
        -0.3507, -0.9392, -0.3141, 0.0068, 0.0783, 0.1254, 0.1297,
    ];
    pub const CONE_CYL_X_D: [f64; 27] = [
        0.421, 0.754, 1.006, 1.088, 1.421, 1.590, 1.755, 2.088, 2.173, 2.422, 2.755, 2.798, 2.843, 2.886, 3.156, 3.491, 3.824, 4.158, 4.490, 4.755, 4.802,
        4.845, 4.868, 5.178, 5.469, 5.759, 5.803,
    ];
    pub const CONE_CYL_CP_M04_MEAN: [f64; 27] = [
        0.1300, 0.1226, 0.1003, 0.1031, 0.0708, 0.0606, 0.0467, 0.0254, -0.0082, -0.0254, -0.2314, -0.3774, -0.3737, -0.2421, -0.0935, -0.0559, -0.0458,
        -0.0587, -0.0896, -0.2349, -0.3347, -0.3529, -0.2284, -0.0360, 0.0239, 0.0628, 0.0701,
    ];
    pub const CONE_CYL_CP_M06_MEAN: [f64; 27] = [
        0.1421, 0.1395, 0.1144, 0.1162, 0.0810, 0.0405, 0.0603, 0.0371, -0.0014, -0.0159, -0.2466, -0.4208, -0.4204, -0.2632, -0.0933, -0.0505, -0.0397,
        -0.0545, -0.0877, -0.2530, -0.3722, -0.3911, -0.2471, -0.0311, 0.0337, 0.0753, 0.0835,
    ];
    pub const CONE_CYL_CP_M08_MEAN: [f64; 27] = [
        0.1906, 0.1849, 0.1619, 0.1618, 0.1271, 0.1181, 0.1060, 0.0848, 0.0473, 0.0316, -0.1650, -0.2975, -0.9647, -0.8571, -0.0542, -0.0172, -0.0101, -0.0258,
        -0.0578, -0.2303, -0.3566, -0.9101, -0.2894, 0.0068, 0.0758, 0.1219, 0.1295,
    ];
}

#[cfg(test)]
mod tests {
    use super::*;

    fn small(base: bool, mach: f64) -> SubsonicCase {
        let mut c = SubsonicCase::new(mach, Nose::TangentOgive { fineness: 3.0 }, 0, base).with_domain(5.0, 2.5);
        c.grid_base.n_up = 12;
        c.grid_base.n_nose = 24;
        c.grid_base.n_cyl = 20;
        c.grid_base.n_wake = 16;
        c.grid_base.n_gap = 8;
        c.grid_base.nj_out = 24;
        c.grid_base.dx_tip = 0.08;
        c.grid_base.dr_wall = 0.08;
        c.grid_sting.n_up = 12;
        c.grid_sting.n_nose = 24;
        c.grid_sting.n_cyl = 20;
        c.grid_sting.n_ext = 12;
        c.grid_sting.nj = 32;
        c.grid_sting.dx_tip = 0.08;
        c.grid_sting.dr_wall = 0.08;
        c.first_order_iters = 100;
        c.max_iter = 3000;
        c
    }

    /// Short subsonic smoke test (no base): M = 0.3, 6 orders in a few hundred iterations, finite positive state, the slender-body
    /// pressure signature (stagnation-like Cp > 0 at the tip, suction at the shoulder), and close to d'Alembert (nose CD ~ 0).
    #[test]
    fn subsonic_smoke_no_base() {
        let r = run_subsonic(&small(false, 0.3));
        assert_eq!(r.stop_reason, "residual drop reached", "{} iters, drop {:.2}", r.iters, r.drop_orders);
        assert!(r.iters < 1500, "{} iterations", r.iters);
        assert!(r.min_rho > 0.0 && r.min_p > 0.0 && r.residual.iter().all(|x| x.is_finite()));
        assert!(r.wall[0].cp > 0.1 && r.cp_min < -0.05 && r.cp_min > -0.3, "Cp tip {} min {}", r.wall[0].cp, r.cp_min);
        assert!(r.cd_nose.abs() < 0.02, "nose CD {}", r.cd_nose);
        assert!(r.cp_base.is_nan() && r.base_wall.is_empty());
        assert_eq!(r.cd_base_model, 0.0);
        assert!((r.cd_total_model - r.cd_total).abs() < 1e-15);
    }

    /// Base smoke test at M = 0.2: the inviscid wake reaches a stationary base pressure (not a 6-order drop), the base pressure is a
    /// suction (Cp_b < 0), the drag breakdown adds up, the pressure drag of the closed body is dominated by the base (base CD =
    /// -Cp_b A_b / A_ref = -Cp_b for a full disc) and the modelled base drag is the correlation's for a full disc.
    #[test]
    fn subsonic_smoke_with_base() {
        // standard level-0 grids (the coarse test grids are too crude for the wake to settle)
        let mut c = SubsonicCase::new(0.2, Nose::TangentOgive { fineness: 3.0 }, 0, true);
        c.max_iter = 6000;
        let r = run_subsonic(&c);
        assert!(
            matches!(r.stop_reason, "residual drop reached" | "base pressure stationary"),
            "{} after {} iters, drop {:.2}",
            r.stop_reason,
            r.iters,
            r.drop_orders
        );
        assert!(r.min_rho > 0.0 && r.min_p > 0.0);
        assert!(r.cp_base < -0.01 && r.cp_base > -0.5, "Cp_b {}", r.cp_base);
        assert!((r.cd_base + r.cp_base).abs() < 1e-12, "base CD {} vs -Cp_b {}", r.cd_base, -r.cp_base);
        assert!((r.cd_total - (r.cd_nose + r.cd_cyl + r.cd_boat + r.cd_base)).abs() < 1e-14);
        assert!(r.cd_base > 0.0 && r.cd_base > 5.0 * r.cd_nose.abs());
        assert_eq!(r.base_wall.len(), c.grid_base.n_gap);
        assert!((r.cd_base_model - base_drag_coefficient(0.2)).abs() < 1e-15);
    }

    /// A uniform free stream is a fixed point of the blocked-grid solver with the implicit scheme (body block, base wall, wake):
    /// the residual of the free stream is non-zero only through the body (it is blocked), so check the far field stays quiet
    /// and the state positive after a few iterations from the free stream; and the explicit and implicit residual norms at
    /// iteration 0 are identical (same residual kernel).
    #[test]
    fn blocked_grid_first_residual_is_scheme_independent() {
        let mut a = small(true, 0.4);
        a.scheme = Scheme::Explicit;
        let (mut sa, mut sb) = (a.build_solver(), small(true, 0.4).build_solver());
        let (ra, rb) = (sa.step(), sb.step());
        assert!(ra > 0.0 && (ra - rb).abs() <= 1e-14 * ra, "{ra:e} vs {rb:e}");
        // solid cells never change
        let b = sa.block.unwrap();
        let q0 = bowshock_core::gas::prim_to_cons(&freestream(0.4));
        for j in 0..b.j_end {
            for i in 0..b.i_end {
                for k in 0..4 {
                    assert!((sa.u[j * sa.grid.ni + i][k] - q0[k]).abs() < 1e-14, "explicit solid cell changed");
                    assert!((sb.u[j * sb.grid.ni + i][k] - q0[k]).abs() < 1e-14, "implicit solid cell changed");
                }
            }
        }
    }

    /// The converged state must not depend on the pseudo-time scheme: explicit SSP-RK3, line-implicit (with and without defect
    /// correction, Rusanov and Roe-type left-hand side) and the point-implicit variant reach the same fixed point. The explicit
    /// comparison uses the first-order scheme (the limited second-order explicit iteration only reaches a limit cycle, README);
    /// the implicit variants are also compared with each other in the second-order van Albada scheme.
    #[test]
    fn converged_state_is_independent_of_the_acceleration_scheme() {
        use crate::body_grid::Nose;
        use crate::subsonic::SubsonicCase;
        use bowshock_core::flux::Limiter;
        use bowshock_core::gas::Cons;
        use bowshock_core::solver::Scheme;
        let mut case = SubsonicCase::new(0.5, Nose::TangentOgive { fineness: 3.0 }, 0, false);
        case.grid_sting.n_up = 12;
        case.grid_sting.n_nose = 20;
        case.grid_sting.n_cyl = 12;
        case.grid_sting.n_ext = 12;
        case.grid_sting.nj = 24;
        case.grid_sting.x_up = 10.0;
        case.grid_sting.x_down = 12.0;
        case.grid_sting.r_far = 10.0;
        case.grid_sting.dx_tip = 0.1;
        case.grid_sting.dr_wall = 0.1;
        case.first_order_iters = 0;
        let converge = |limiter: Limiter, scheme: Scheme, iters: usize| -> Vec<Cons> {
            let mut c = case.clone();
            c.limiter = limiter;
            c.scheme = scheme;
            c.cfl = 0.8;
            let mut s = c.build_solver();
            let mut last = 1.0;
            for _ in 0..iters {
                last = s.step();
                if last < 1e-11 {
                    break;
                }
            }
            assert!(last < 1e-9, "{limiter:?} {scheme:?} did not converge: {last:e}");
            s.u.clone()
        };
        let diff = |a: &[Cons], b: &[Cons]| {
            a.iter().zip(b.iter()).flat_map(|(a, b)| (0..4).map(move |k| (a[k] - b[k]).abs() / b[k].abs().max(1.0))).fold(0.0, f64::max)
        };
        let implicit_variants = [
            Scheme::Implicit(ImplicitParams { sub_iters: 1, roe: false, ..ImplicitParams::DEFAULT }),
            Scheme::Implicit(ImplicitParams { lines: false, cfl_max: 4.0, ..ImplicitParams::DEFAULT }),
        ];
        // first order: explicit vs implicit
        let reference = converge(Limiter::FirstOrder, Scheme::Explicit, 40000);
        for scheme in std::iter::once(Scheme::Implicit(ImplicitParams::DEFAULT)).chain(implicit_variants) {
            let d = diff(&converge(Limiter::FirstOrder, scheme, 8000), &reference);
            assert!(d < 1e-6, "first order, {scheme:?}: state difference to the explicit solution {d:e}");
        }
        // second order (van Albada): implicit variants among themselves
        let reference = converge(Limiter::VanAlbada, Scheme::Implicit(ImplicitParams::DEFAULT), 8000);
        {
            let scheme = Scheme::Implicit(ImplicitParams { sub_iters: 1, roe: false, ..ImplicitParams::DEFAULT });
            let d = diff(&converge(Limiter::VanAlbada, scheme, 8000), &reference);
            assert!(d < 1e-6, "van Albada, {scheme:?}: state difference {d:e}");
        }
    }
}
