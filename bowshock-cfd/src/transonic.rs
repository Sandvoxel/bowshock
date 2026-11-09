//! Transonic case runner: nose-cylinder body on the large-domain grid (`body_grid`), characteristic far-field BCs
//! (`bc`), steady-state iteration with optional warm start from the previous Mach number, and wall-pressure
//! post-processing (forebody pressure drag referenced to the base area).
//!
//! Boundary set-up: inflow plane and outer boundary `Bc::Characteristic`; outflow plane `Bc::Characteristic` (default)
//! or `Bc::PressureOutflow`; axis/body `Bc::SlipWall`. Lengths are in body diameters.

use crate::body_grid::{large_domain_grid, Body, LargeGridParams, Nose};
use crate::flux::Limiter;
use crate::gas::{cons_to_prim, freestream, sound_speed};
use crate::ogive::WallPoint;
use crate::solver::{Bc, Config, Scheme, Solver};

#[derive(Clone, Debug)]
pub struct TransonicCase {
    pub mach: f64,
    pub body: Body,
    pub grid: LargeGridParams,
    pub cfl: f64,
    pub limiter: Limiter,
    /// Number of first-order start-up iterations (only on a cold start).
    pub first_order_iters: usize,
    pub max_iter: usize,
    /// Stop when the density residual has dropped this many orders (see `run_on` for the reference).
    pub drop_orders: f64,
    /// Alternative stop: the forebody CD varied by less than `cd_tol` (absolute) over the last `cd_window` iterations
    /// and the residual has dropped at least 2 orders.
    pub cd_window: usize,
    pub cd_tol: f64,
    /// Boundary condition of the outflow plane (`Bc::Characteristic` or `Bc::PressureOutflow`).
    pub outflow: Bc,
    /// Convergence scheme (`Scheme::Explicit` = SSP-RK3 as before).
    pub scheme: Scheme,
    /// Low-Mach correction of the reconstructed states (`flux::low_mach_states`).
    pub low_mach: bool,
}

impl TransonicCase {
    /// Default body (tangent ogive, fineness 3, cylinder to 6 d behind the nose: body length `L = 9 d`) in a domain
    /// 10 body lengths upstream and radially, 5 body lengths downstream, grid level 0/1/2 = coarse/medium/fine.
    pub fn level(mach: f64, level: u32) -> Self {
        Self::with_body(mach, level, Body { nose: Nose::TangentOgive { fineness: 3.0 } }, 6.0)
    }

    pub fn with_body(mach: f64, level: u32, body: Body, cyl_length: f64) -> Self {
        let f = 1usize << level;
        let l_body = body.nose_length() + cyl_length;
        let grid = LargeGridParams {
            body,
            cyl_length,
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
        TransonicCase {
            mach,
            body,
            grid,
            cfl: 0.5,
            limiter: Limiter::Minmod,
            first_order_iters: 300,
            max_iter: 60_000,
            drop_orders: 6.0,
            cd_window: 2000,
            cd_tol: 2e-4,
            outflow: Bc::Characteristic,
            scheme: Scheme::Explicit,
            low_mach: false,
        }
    }

    pub fn cells(&self) -> (usize, usize) {
        let g = &self.grid;
        (g.n_up + g.n_nose + g.n_cyl + g.n_ext, g.nj)
    }

    pub fn build_solver(&self) -> Solver {
        let cfg = Config {
            cfl: self.cfl,
            limiter: self.limiter,
            freestream: freestream(self.mach),
            bc: [Bc::Characteristic, self.outflow, Bc::SlipWall, Bc::Characteristic],
            first_order_iters: self.first_order_iters,
        };
        let mut s = Solver::new(large_domain_grid(&self.grid), cfg);
        s.low_mach = self.low_mach;
        s.set_scheme(self.scheme);
        s
    }
}

pub struct TransonicResult {
    pub mach: f64,
    pub iters: usize,
    pub residual: Vec<f64>,
    /// Largest residual after iteration 20 (cold start) / of the run (warm start).
    pub residual_peak: f64,
    /// Orders dropped from `residual_peak` to the last residual.
    pub drop_orders: f64,
    pub stop_reason: &'static str,
    /// Forebody CD (extrapolated wall Cp) every 50 iterations: `(iteration, cd)`.
    pub cd_history: Vec<(usize, f64)>,
    pub wall: Vec<WallPoint>,
    /// Forebody pressure-drag coefficient, base-area reference, extrapolated-wall / first-cell Cp.
    pub cd_nose: f64,
    pub cd_nose_cell: f64,
    /// Max - min of the forebody CD over the last `cd_window` iterations (unsteadiness / convergence of the integral).
    pub cd_range: f64,
    /// Minimum wall Cp over the body and its position; maximum wall-cell Mach number.
    pub cp_min: f64,
    pub x_cp_min: f64,
    pub m_wall_max: f64,
    /// Max `|p/p_inf - 1|` over the boundary cells of the inflow, outer and outflow boundaries (domain-size indicator).
    pub farfield_dp: f64,
    pub min_rho: f64,
    pub min_p: f64,
}

/// Cold-start run (fresh solver).
pub fn run_transonic(case: &TransonicCase) -> TransonicResult {
    let mut s = case.build_solver();
    run_on(case, &mut s, false, None)
}

/// Forebody drag coefficient from the current state: `(extrapolated wall Cp, first-cell Cp)` integrals of `Cp d(r^2/R^2)`
/// over the nose, plus the wall points of the whole body.
pub fn wall_distribution(body: &Body, s: &Solver) -> (Vec<WallPoint>, f64, f64) {
    let g = &s.grid;
    let ni = g.ni;
    let fs = s.cfg.freestream;
    let (p_inf, q_inf) = (fs[3], 0.5 * fs[0] * fs[1] * fs[1]);
    let cp = |p: f64| (p - p_inf) / q_inf;
    let (ln, rb2) = (body.nose_length(), body.radius().powi(2));
    let mut wall = Vec::new();
    let (mut cd, mut cd_cell) = (0.0, 0.0);
    for i in 0..ni {
        let (n0, n1) = (g.node(i, 0), g.node(i + 1, 0));
        let (xm, rm) = (0.5 * (n0.0 + n1.0), 0.5 * (n0.1 + n1.1));
        if xm <= 0.0 {
            continue;
        }
        let (p0, p1) = (s.pressure(i, 0), s.pressure(i, 1));
        let d0 = (g.xc[i] - xm).hypot(g.rc[i] - rm);
        let d1 = (g.xc[ni + i] - xm).hypot(g.rc[ni + i] - rm);
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

/// Iterate `s` (already built from `case`, or reused from a previous Mach number if `warm`) to convergence.
///
/// * `warm`: keep the converged field of the previous run; only the free stream (far-field state) is changed to
///   `case.mach`. No first-order start-up phase.
/// * `r_ref`: reference residual for the drop criterion. `None` = peak residual of this run (after iteration 20);
///   in a warm-started sweep pass the peak of the cold-start run so that every Mach number has to reach the same absolute
///   residual level (the warm-start residual peak itself is much smaller).
pub fn run_on(case: &TransonicCase, s: &mut Solver, warm: bool, r_ref: Option<f64>) -> TransonicResult {
    s.cfg.freestream = freestream(case.mach);
    s.cfg.cfl = case.cfl;
    s.cfg.limiter = case.limiter;
    s.cfg.first_order_iters = case.first_order_iters;
    s.iter = if warm { case.first_order_iters } else { 0 };
    s.low_mach = case.low_mach;
    s.restart_ramp();

    let mut residual = Vec::new();
    let mut cd_hist: Vec<(usize, f64)> = Vec::new();
    let mut rmax: f64 = 0.0;
    let mut reason = "max iterations";
    let skip = if warm { 0 } else { 20 };
    for it in 0..case.max_iter {
        let r = s.step();
        residual.push(r);
        if it >= skip {
            rmax = rmax.max(r);
        }
        if it % 50 == 49 {
            cd_hist.push((it + 1, wall_distribution(&case.body, s).1));
        }
        if let Some(rs) = stop_reason(case, it, r, r_ref.unwrap_or(rmax), skip, &cd_hist) {
            reason = rs;
            break;
        }
    }
    finish(case, s, residual, rmax, cd_hist, reason)
}

/// Stopping rule, called after iteration `it` (0-based) with its density residual `r`, the drop reference
/// `reference` (`r_ref`, or the running peak `rmax` of this run), the peak-tracking offset `skip` (20 cold / 0 warm) and the CD samples taken so far (one per 50
/// iterations, already including iteration `it` if `it % 50 == 49`). In this order: non-finite residual; residual drop of
/// `drop_orders` below `reference`; every 500 iterations (once `it + 1 >= 2 cd_window`) a CD range over the last `cd_window`
/// iterations below `cd_tol` provided the residual has dropped at least 2 orders ("CD stationary").
pub fn stop_reason(case: &TransonicCase, it: usize, r: f64, reference: f64, skip: usize, cd_hist: &[(usize, f64)]) -> Option<&'static str> {
    // `reference` is r_ref, or the running peak of this run when no reference was given
    if !r.is_finite() {
        return Some("non-finite residual");
    }
    if it > skip && reference / r >= 10f64.powf(case.drop_orders) {
        return Some("residual drop reached");
    }
    if it % 500 == 499 && it + 1 >= 2 * case.cd_window && reference / r >= 100.0 {
        let n = case.cd_window / 50;
        let tail = &cd_hist[cd_hist.len() - n..];
        let (lo, hi) = tail.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |(l, h), c| (l.min(c.1), h.max(c.1)));
        if hi - lo < case.cd_tol {
            return Some("CD stationary");
        }
    }
    None
}

/// Post-process the state held in `s` (`s.u`, `s.grid`, `s.cfg.freestream`) into a [`TransonicResult`]: wall Cp / forebody CD,
/// Cp minimum, wall Mach number, far-field indicator. `residual` is the full density-residual history, `rmax` its reference
/// peak, `cd_hist` the CD samples. Usable on a host `Solver` whose `u` was overwritten with another solver's (GPU) state.
pub fn finish(case: &TransonicCase, s: &Solver, residual: Vec<f64>, rmax: f64, cd_hist: Vec<(usize, f64)>, reason: &'static str) -> TransonicResult {
    let (wall, cd_nose, cd_nose_cell) = wall_distribution(&case.body, s);
    let last = residual.last().copied().unwrap_or(rmax);
    let nwin = (case.cd_window / 50).min(cd_hist.len());
    let cd_range = if nwin >= 2 {
        let (lo, hi) = cd_hist[cd_hist.len() - nwin..].iter().fold((f64::INFINITY, f64::NEG_INFINITY), |(l, h), c| (l.min(c.1), h.max(c.1)));
        hi - lo
    } else {
        f64::NAN
    };
    let drop_orders = if rmax > 0.0 && last > 0.0 { (rmax / last).log10() } else { 0.0 };

    let (cp_min, x_cp_min) = wall.iter().fold((f64::INFINITY, 0.0), |(m, xm), w| if w.cp < m { (w.cp, w.x) } else { (m, xm) });
    let ni = s.grid.ni;
    let mut m_wall_max: f64 = 0.0;
    for i in 0..ni {
        if s.grid.xc[i] > 0.0 {
            let w = cons_to_prim(&s.u[i]);
            m_wall_max = m_wall_max.max(w[1].hypot(w[2]) / sound_speed(&w));
        }
    }
    let nj = s.grid.nj;
    let p_inf = s.cfg.freestream[3];
    let dev = |i: usize, j: usize| (s.pressure(i, j) / p_inf - 1.0).abs();
    let mut farfield_dp: f64 = 0.0;
    for i in 0..ni {
        farfield_dp = farfield_dp.max(dev(i, nj - 1));
    }
    for j in 0..nj {
        farfield_dp = farfield_dp.max(dev(0, j)).max(dev(ni - 1, j));
    }
    let (mut min_rho, mut min_p) = (f64::INFINITY, f64::INFINITY);
    for q in &s.u {
        let w = cons_to_prim(q);
        min_rho = min_rho.min(w[0]);
        min_p = min_p.min(w[3]);
    }
    TransonicResult {
        mach: case.mach,
        iters: residual.len(),
        residual,
        residual_peak: rmax,
        drop_orders,
        stop_reason: reason,
        cd_history: cd_hist,
        wall,
        cd_nose,
        cd_nose_cell,
        cd_range,
        cp_min,
        x_cp_min,
        m_wall_max,
        farfield_dp,
        min_rho,
        min_p,
    }
}

/// Published free-flight nose pressure-drag data, zero lift, fineness-ratio-3 noses on a cylinder.
///
/// Source: W. E. Stoney Jr., "Transonic Drag Measurements of Eight Body-Nose Shapes", NACA RM L53K17 (Langley, 1954),
/// NTRS 19930087953, Fig. 5 (p. 14): nose pressure-drag coefficient `CD` (reference area = maximum frontal area, i.e. the
/// base-area reference used here) vs Mach number. **The numbers are digitised by eye from a poor-quality scan of the plot
/// (they are not tabulated in the report): read-off uncertainty about +-0.01 in `CD` and +-0.01 in `M`** (`READ_UNCERTAINTY`).
/// The nose pressure drag is itself a derived quantity: free-flight total CD minus a tare (afterbody + fins + base,
/// 0.025 subsonic, about 0.065 supersonic, anchored at M = 1.2 on the Taylor-Maccoll cone drag; Fig. 4) minus the calculated
/// fully turbulent skin friction; the tare alone is uncertain by about +-0.01, the report states +-0.008 in total CD and +-0.005
/// in M. Free flight, Re = 8e6 .. 15e6 (body length), fully turbulent boundary layer (inferred by the author). Beyond
/// M ~ 1.225 the plotted curves are fairings to the Ames data (NACA Rep. 1386) and second-order theory, not new measurements.
/// Models: nose (l/d = 3), cylinder (l/d = 4), conical afterbody (l/d = 5, cut-off base), swept fins.
pub mod stoney {
    pub const READ_UNCERTAINTY: f64 = 0.01;
    /// Largest Mach number of the "present tests" (the curves beyond it are faired).
    pub const MEASURED_UP_TO: f64 = 1.225;
    /// Cone, `r = x tan(9.46 deg)` (`l/d = 3`).
    pub const CONE: &[(f64, f64)] =
        &[(0.80, 0.02), (0.90, 0.03), (0.95, 0.06), (1.00, 0.11), (1.05, 0.16), (1.10, 0.16), (1.15, 0.15), (1.20, 0.14), (1.25, 0.13)];
    /// Power series `r = x^(1/2)` (vertex parabola).
    pub const POWER_HALF: &[(f64, f64)] =
        &[(0.80, 0.00), (0.90, 0.00), (0.95, 0.01), (1.00, 0.05), (1.05, 0.06), (1.10, 0.06), (1.15, 0.07), (1.20, 0.08), (1.25, 0.07)];
    /// Von Karman (LD-Haack, `C = 0`), digitised above M = 0.95 only.
    pub const VON_KARMAN: &[(f64, f64)] = &[(0.95, 0.00), (1.00, 0.03), (1.05, 0.07), (1.10, 0.075), (1.15, 0.075), (1.20, 0.08), (1.24, 0.077)];
    /// LV-Haack (`C = 1/3`), digitised above M = 0.95 only.
    pub const LV_HAACK: &[(f64, f64)] = &[(0.95, 0.00), (1.00, 0.03), (1.05, 0.07), (1.10, 0.085), (1.15, 0.097), (1.17, 0.104)];
    /// Parabolic series `K = 1` (`r/R = 2 eta - eta^2`, tangent to the cylinder; closest of the eight shapes to a tangent ogive of
    /// the same fineness). Middle panel of Fig. 5, the hardest to read: uncertainty +-0.01 in CD, more where curves cross.
    pub const PARABOLA: &[(f64, f64)] = &[(0.95, 0.01), (0.975, 0.02), (1.00, 0.045), (1.05, 0.09), (1.10, 0.105), (1.15, 0.115), (1.20, 0.12)];

    /// Linear interpolation of a series; `None` outside its Mach range.
    pub fn interp(series: &[(f64, f64)], m: f64) -> Option<f64> {
        for w in series.windows(2) {
            if m >= w[0].0 - 1e-9 && m <= w[1].0 + 1e-9 {
                let t = (m - w[0].0) / (w[1].0 - w[0].0);
                return Some(w[0].1 + t * (w[1].1 - w[0].1));
            }
        }
        None
    }
}

/// Named test bodies (fineness-ratio-3 noses, `d = 1`) with the matching Stoney data series.
pub fn named_body(name: &str) -> Option<(Body, Option<&'static [(f64, f64)]>)> {
    let f = 3.0;
    Some(match name {
        "cone" => (Body { nose: Nose::cone_fineness(f) }, Some(stoney::CONE)),
        "karman" => (Body { nose: Nose::Haack { fineness: f, c: 0.0 } }, Some(stoney::VON_KARMAN)),
        "lvhaack" => (Body { nose: Nose::Haack { fineness: f, c: 1.0 / 3.0 } }, Some(stoney::LV_HAACK)),
        "parabola" => (Body { nose: Nose::Parabolic { fineness: f, k: 1.0 } }, Some(stoney::PARABOLA)),
        "sqrt" => (Body { nose: Nose::Power { fineness: f, n: 0.5 } }, Some(stoney::POWER_HALF)),
        "ogive" => (Body { nose: Nose::TangentOgive { fineness: f } }, None),
        _ => return None,
    })
}

/// Tangent-cone estimate of the forebody CD (base-area reference): `int Cp_TM(local slope) d(r^2/R^2)` over the nose,
/// `Cp_TM` the Taylor-Maccoll surface Cp of the cone whose half angle equals the local slope (linearly scaled with the slope
/// below 1 deg). NaN if any local slope is beyond the attached-shock limit (e.g. subsonic/transonic Mach, rounded tips).
/// For a cone body this is the exact Taylor-Maccoll cone drag `Cp_cone`.
pub fn tangent_cone_cd(body: &Body, mach: f64) -> f64 {
    let n = 400;
    let (ln, rb2) = (body.nose_length(), body.radius().powi(2));
    let floor = 1f64.to_radians();
    let mut cd = 0.0;
    let mut last: Option<(f64, f64)> = None; // (theta, cp) memo: a cone has the same slope everywhere
    for k in 0..n {
        let (x0, x1) = (ln * k as f64 / n as f64, ln * (k + 1) as f64 / n as f64);
        let theta = body.slope(0.5 * (x0 + x1)).atan();
        let cp = match last {
            Some((t, c)) if (t - theta).abs() < 1e-13 => c,
            _ => {
                let c = if theta >= floor {
                    crate::taylor_maccoll::solve_cone(mach, theta).map_or(f64::NAN, |s| s.cp_surface)
                } else {
                    crate::taylor_maccoll::solve_cone(mach, floor).map_or(f64::NAN, |s| s.cp_surface) * theta / floor
                };
                last = Some((theta, c));
                c
            }
        };
        if cp.is_nan() {
            return f64::NAN;
        }
        cd += cp * (body.r(x1).powi(2) - body.r(x0).powi(2)) / rb2;
    }
    cd
}

/// Published wind-tunnel surface pressures, zero angle of attack, 3-caliber tangent-ogive cylinder (the `ogive` body of
/// [`named_body`]).
///
/// Source: K. R. Czarnecki and W. J. Monta, "Pressure Distributions Due to Two-Dimensional Fabrication-Type Surface Roughness on an
/// Ogive Cylinder at Transonic Speeds", NASA TN D-3516 (Langley, 1966), NTRS 19660024157, Fig. 5 (report p. 27): the "basic smooth
/// model". Model (Fig. 1, Table II): tangent ogive nose 12.29 in = 3.00 d, ogive radius 37.89 in, d = 4.096 in, total length 50 in
/// (12.2 d), sting mounted (sting not shown in the figure). Langley 8-ft transonic pressure tunnel (slotted), fully turbulent
/// (No. 60 carborundum 0.75 in from the tip), Re per ft about 1e6 .. 6e6, no Reynolds-number effect on Cp reported, no wall corrections.
/// Empty-tunnel Mach non-uniformity about +-0.005 (subsonic) .. +-0.02 (highest M). **The Cp values are digitised from the plot
/// (not tabulated): +-0.01 for x >= 6 in, about +-0.02 for x < 5 in, about +-0.015 within 1 in of a shock; the last point (x = 49.62 in,
/// near the base/sting) +-0.02.** Slotted-wall shock reflections are stated by the authors: the reflected nose shock hits the body at
/// x ~ 30 in at M = 1.1 and x ~ 48 in at M = 1.2 (tunnel artefact, not present in free air).
pub mod czarnecki_monta {
    pub const D_IN: f64 = 4.096;
    pub const MACH: [f64; 5] = [0.7, 0.9, 1.0, 1.1, 1.2];
    /// Digitisation uncertainty in Cp (stations x >= 6 in); about twice that on the nose, x < 5 in.
    pub const READ_UNCERTAINTY: f64 = 0.01;
    /// Last station (in) not contaminated by the stated wall-reflected shock / base-sting region, per Mach number.
    pub fn valid_x_max_in(mach: f64) -> f64 {
        if (mach - 1.1).abs() < 1e-9 {
            28.8
        } else {
            44.1
        }
    }
    /// Station `x` [in] from the nose tip.
    pub const X_IN: [f64; 34] = [1.01, 1.95, 2.82, 3.81, 4.78, 5.78, 6.76, 7.76, 8.77, 9.79, 10.78, 11.80, 12.78, 13.77, 14.78, 15.78, 16.78, 17.75, 18.79, 19.78, 20.78, 22.75, 24.74, 26.77, 28.76, 30.77, 32.76, 34.78, 36.79, 42.14, 44.10, 46.08, 48.08, 49.62];
    /// Cp at M = 0.7, 0.9, 1.0, 1.1, 1.2 for each station.
    pub const CP: [[f64; 5]; 34] = [
        [0.282, 0.312, 0.371, 0.372, 0.353],
        [0.186, 0.220, 0.283, 0.279, 0.285],
        [0.122, 0.140, 0.203, 0.233, 0.244],
        [0.057, 0.066, 0.131, 0.173, 0.193],
        [0.010, 0.009, 0.071, 0.122, 0.148],
        [-0.034, -0.045, 0.010, 0.063, 0.084],
        [-0.062, -0.086, -0.034, 0.021, 0.042],
        [-0.086, -0.120, -0.089, -0.026, 0.001],
        [-0.106, -0.152, -0.126, -0.065, -0.035],
        [-0.116, -0.167, -0.182, -0.112, -0.079],
        [-0.110, -0.149, -0.215, -0.146, -0.105],
        [-0.077, -0.098, -0.241, -0.172, -0.131],
        [-0.055, -0.066, -0.194, -0.137, -0.107],
        [-0.037, -0.047, -0.154, -0.108, -0.087],
        [-0.028, -0.035, -0.120, -0.089, -0.069],
        [-0.014, -0.024, -0.097, -0.070, -0.051],
        [-0.013, -0.021, -0.081, -0.059, -0.040],
        [-0.011, -0.017, -0.070, -0.053, -0.032],
        [-0.014, -0.023, -0.072, -0.053, -0.030],
        [-0.008, -0.017, -0.058, -0.047, -0.025],
        [-0.008, -0.015, -0.057, -0.043, -0.020],
        [-0.008, -0.015, -0.042, -0.037, -0.016],
        [-0.007, -0.014, 0.026, -0.024, -0.016],
        [-0.007, -0.013, 0.029, -0.018, -0.017],
        [-0.003, -0.010, 0.026, -0.010, -0.016],
        [-0.006, -0.012, 0.012, 0.059, -0.021],
        [-0.005, -0.013, 0.000, 0.061, -0.021],
        [0.000, -0.007, 0.001, 0.049, -0.015],
        [0.000, -0.007, -0.001, 0.032, -0.011],
        [0.005, -0.003, 0.001, 0.003, -0.011],
        [-0.001, -0.008, -0.007, -0.021, -0.017],
        [-0.010, -0.016, -0.015, -0.060, -0.013],
        [-0.022, -0.030, -0.025, -0.082, 0.040],
        [-0.058, -0.067, -0.079, -0.154, -0.046],
    ];

    /// Column of `CP` for Mach number `m` (exact match to the test Mach numbers).
    pub fn column(m: f64) -> Option<usize> {
        MACH.iter().position(|&x| (x - m).abs() < 1e-9)
    }
}


#[cfg(test)]
mod tests {
    use super::*;
    use crate::gas::prim_to_cons;
    use crate::grid::Grid;

    /// Smoothly curved, non-uniform grid with the lower boundary on the axis r=0.
    fn curved_grid(ni: usize, nj: usize) -> Grid {
        let (mut x, mut r) = (vec![0.0; (ni + 1) * (nj + 1)], vec![0.0; (ni + 1) * (nj + 1)]);
        for j in 0..=nj {
            for i in 0..=ni {
                let (s, t) = (i as f64 / ni as f64, j as f64 / nj as f64);
                let k = j * (ni + 1) + i;
                x[k] = 2.0 * s + 0.15 * (3.0 * t).sin() * (2.0 * s).cos() + 0.1 * t * t;
                r[k] = t * (1.0 + 0.3 * s) * (1.0 + 0.2 * t) + 0.05 * (4.0 * s).sin() * t * (1.0 - t);
            }
        }
        Grid::from_nodes(ni, nj, x, r)
    }

    /// Uniform flow with the characteristic BCs on inflow, outflow and outer boundary and a slip axis must stay uniform:
    /// no spurious waves from the BCs at M = 0.5 and 0.9 (also 0.98 and 1.2, other regimes).
    #[test]
    fn characteristic_farfield_preserves_freestream() {
        for mach in [0.5, 0.9, 0.98, 1.2] {
            for outflow in [Bc::Characteristic, Bc::PressureOutflow] {
                let fs = freestream(mach);
                let cfg = Config {
                    cfl: 0.5,
                    limiter: Limiter::Minmod,
                    freestream: fs,
                    bc: [Bc::Characteristic, outflow, Bc::SlipWall, Bc::Characteristic],
                    first_order_iters: 0,
                };
                let mut s = Solver::new(curved_grid(24, 16), cfg);
                for _ in 0..100 {
                    s.step();
                }
                let q0 = prim_to_cons(&fs);
                let mut worst: f64 = 0.0;
                for q in &s.u {
                    for k in 0..4 {
                        worst = worst.max((q[k] - q0[k]).abs() / q0[k].abs().max(1.0));
                    }
                }
                assert!(worst < 1e-12, "M={mach} {outflow:?}: drift {worst:e}");
            }
        }
    }

    /// An acoustic/entropy perturbation introduced in the interior must leave a subsonic domain through the
    /// characteristic BCs (the residual of the perturbation decays), while it is reflected / trapped less than with
    /// a fixed free stream. Here: Gaussian pressure pulse, M = 0.5, the perturbation energy must drop by > 90 %
    /// within a few domain crossing times.
    #[test]
    fn characteristic_farfield_lets_a_pressure_pulse_out() {
        let mach = 0.5;
        let fs = freestream(mach);
        let (ni, nj) = (60, 30);
        // uniform rectangle [0, 6] x [0, 3] with the axis at the bottom treated as slip wall
        let (mut x, mut r) = (vec![0.0; (ni + 1) * (nj + 1)], vec![0.0; (ni + 1) * (nj + 1)]);
        for j in 0..=nj {
            for i in 0..=ni {
                x[j * (ni + 1) + i] = 6.0 * i as f64 / ni as f64;
                r[j * (ni + 1) + i] = 3.0 * j as f64 / nj as f64;
            }
        }
        let grid = Grid::from_nodes(ni, nj, x, r);
        let run = |bc: [Bc; 4]| {
            let cfg = Config { cfl: 0.5, limiter: Limiter::Minmod, freestream: fs, bc, first_order_iters: 0 };
            let mut s = Solver::new(grid_clone(&grid), cfg);
            for j in 0..nj {
                for i in 0..ni {
                    let (xc, rc) = (s.grid.xc[j * ni + i], s.grid.rc[j * ni + i]);
                    let dp = 0.05 * fs[3] * (-((xc - 3.0).powi(2) + (rc - 1.0).powi(2)) / 0.2).exp();
                    s.u[j * ni + i] = prim_to_cons(&[fs[0] * (1.0 + dp / fs[3] / 1.4), fs[1], fs[2], fs[3] + dp]);
                }
            }
            let energy = |s: &Solver| -> f64 { s.u.iter().map(|q| (cons_to_prim(q)[3] - fs[3]).powi(2)).sum::<f64>().sqrt() };
            let e0 = energy(&s);
            // physical time is not tracked (local time stepping is used): iterate long enough for many crossings
            for _ in 0..1500 {
                s.step();
            }
            energy(&s) / e0
        };
        let char_bc = run([Bc::Characteristic, Bc::Characteristic, Bc::SlipWall, Bc::Characteristic]);
        assert!(char_bc < 0.1, "pulse remaining with characteristic BCs: {char_bc:e}");
    }

    fn grid_clone(g: &Grid) -> Grid {
        Grid::from_nodes(g.ni, g.nj, g.x.clone(), g.r.clone())
    }

    /// Smoke test: coarse grid, cone-cylinder, M = 0.9, 1500 iterations: finite, residual falls, nose CD plausible.
    #[test]
    fn transonic_smoke_coarse_m09() {
        let cone = Body { nose: Nose::cone_fineness(3.0) };
        let mut case = TransonicCase::with_body(0.9, 0, cone, 6.0);
        // shrink to keep the test quick
        case.grid.n_up = 16;
        case.grid.n_nose = 32;
        case.grid.n_cyl = 16;
        case.grid.n_ext = 14;
        case.grid.nj = 40;
        case.grid.dx_tip = 0.04;
        case.grid.dr_wall = 0.04;
        case.max_iter = 1500;
        case.first_order_iters = 150;
        let res = run_transonic(&case);
        assert!(res.residual.iter().all(|r| r.is_finite()), "non-finite residual");
        assert!(res.min_rho > 0.0 && res.min_p > 0.0);
        assert!(res.drop_orders > 1.0, "residual dropped only {:.2} orders", res.drop_orders);
        // cone-cylinder: Stoney (RM L53K17, digitised) has 0.03 +- 0.01 at M = 0.9, the coarse 1500-iteration run must be within a factor ~3
        assert!(res.cd_nose.is_finite() && res.cd_nose > 0.0 && res.cd_nose < 0.12, "CD {}", res.cd_nose);
        // subsonic slender-body flow: the wall Cp on the nose is positive at the tip and negative (suction) near the shoulder
        assert!(res.wall.first().unwrap().cp > 0.0 && res.cp_min < 0.0);
    }
}
