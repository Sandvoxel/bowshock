//! Tangent-ogive + cylinder validation case: body-fitted grid, steady-state iteration, wall-pressure
//! post-processing, and the published wind-tunnel data it is compared with.
//!
//! The geometry (`shock_expansion::Ogive`) is non-dimensionalised with the body diameter `d = 1`
//! (radius `R = 0.5`); the nose runs over `0 <= x <= L_nose` and the cylinder over `L_nose <= x <= L_nose + cyl_length`.

use bowshock_core::flux::Limiter;
use bowshock_core::gas::freestream;
use bowshock_core::grid::Grid;
use bowshock_core::solver::{Bc, Config, Solver};
use bowshock_reference::shock_expansion::{Ogive, SurfaceTheory};

/// Published experimental surface pressures, zero angle of attack.
///
/// Source: E. W. Perkins and L. H. Jorgensen, "Comparison of Experimental and Theoretical Normal-Force
/// Distributions (Including Reynolds Number Effects) on an Ogive-Cylinder Body at Mach Number 1.98",
/// NACA RM A54H23 (1954), NTRS 19930088493. Model: fineness-ratio-3 circular-arc tangent ogive nose
/// (arc radius 9.25 d, Fig. 1), cylinder afterbody to l = 10 d, d = 1.125 in. Data: Table I(a), alpha = 0,
/// Re = 0.39e6 per inch, M = 1.98 (transcribed from the scanned table, 3 decimals as printed).
/// Stated uncertainty of Cp: +-0.005 (p. 6 of the report).
pub mod perkins_jorgensen {
    pub const MACH: f64 = 1.98;
    pub const FINENESS: f64 = 3.0;
    pub const CP_UNCERTAINTY: f64 = 0.005;
    /// Orifice stations x/d from the vertex (Table I(a), orifices 1..15).
    #[allow(clippy::approx_constant)] // 6.28 is a measured station, not tau
    pub const X_OVER_D: [f64; 15] = [0.22, 0.56, 0.89, 1.22, 1.61, 2.06, 2.5, 2.95, 3.62, 4.06, 4.50, 5.39, 6.28, 7.17, 8.06];
    pub const CP: [f64; 15] = [0.260, 0.212, 0.165, 0.115, 0.074, 0.026, -0.014, -0.038, -0.030, -0.017, -0.012, -0.009, -0.001, 0.000, 0.006];

    /// Forebody pressure-drag coefficient (base-area reference) integrated from the nose orifices,
    /// `CD = integral_0^1 Cp d(s)`, `s = (r/R)^2`, trapezoid in `s`, with the unmeasured ends closed by
    /// `cp_tip` at `x = 0` and `cp_tail` at `x = L_nose`.
    pub fn forebody_cd(body: &super::Ogive, cp_tip: f64, cp_tail: f64) -> f64 {
        let l = body.nose_length();
        let r2 = body.radius().powi(2);
        let mut pts: Vec<(f64, f64)> = vec![(0.0, cp_tip)];
        for (x, cp) in X_OVER_D.iter().zip(CP.iter()) {
            if *x < l {
                pts.push((body.r(*x).powi(2) / r2, *cp));
            }
        }
        pts.push((1.0, cp_tail));
        pts.windows(2).map(|w| 0.5 * (w[0].1 + w[1].1) * (w[1].0 - w[0].0)).sum()
    }
}

/// Algebraic body-fitted grid parameters (lengths in body diameters).
#[derive(Clone, Debug)]
pub struct OgiveGridParams {
    pub body: Ogive,
    /// Cells upstream of the tip, along the nose, along the cylinder; and radial cells.
    pub ni_up: usize,
    pub ni_nose: usize,
    pub ni_cyl: usize,
    pub nj: usize,
    /// Cylinder length behind the nose.
    pub cyl_length: f64,
    /// Upstream end of the domain (negative).
    pub x_up: f64,
    /// Outer boundary radius at `x_up` and at the downstream end; straight line in between.
    pub h_up: f64,
    pub h_end: f64,
    /// Wall clustering strength in j (0 = uniform; ~1-3 = tanh clustering towards j=0).
    pub cluster: f64,
    /// Nose-tip clustering exponent: nose nodes at `x = L (k/n)^stretch` (1 = uniform, >1 clusters at the tip).
    pub nose_stretch: f64,
}

/// Generalisation of `grid::cone_grid` to the ogive-cylinder meridian profile. Grid lines of constant
/// `i` are vertical lines `x = x_i`; the lower boundary is the axis for `x < 0` and the body for `x >= 0`;
/// nodes sit exactly at the tip `x = 0` and at the nose/cylinder junction `x = L_nose`.
pub fn ogive_grid(p: &OgiveGridParams) -> Grid {
    let ni = p.ni_up + p.ni_nose + p.ni_cyl;
    let nj = p.nj;
    assert!(p.x_up < 0.0 && p.ni_up >= 1 && p.ni_nose >= 2 && p.ni_cyl >= 1 && nj >= 2);
    let ln = p.body.nose_length();
    let mut xs = Vec::with_capacity(ni + 1);
    for i in 0..p.ni_up {
        xs.push(p.x_up * (1.0 - i as f64 / p.ni_up as f64));
    }
    for k in 0..p.ni_nose {
        xs.push(ln * (k as f64 / p.ni_nose as f64).powf(p.nose_stretch));
    }
    for k in 0..=p.ni_cyl {
        xs.push(ln + p.cyl_length * k as f64 / p.ni_cyl as f64);
    }
    let x_end = ln + p.cyl_length;
    let mut x = vec![0.0; (ni + 1) * (nj + 1)];
    let mut r = vec![0.0; (ni + 1) * (nj + 1)];
    for i in 0..=ni {
        let xi = xs[i];
        let r_in = if xi > 0.0 { p.body.r(xi) } else { 0.0 };
        let r_out = p.h_up + (p.h_end - p.h_up) * (xi - p.x_up) / (x_end - p.x_up);
        for j in 0..=nj {
            let xi_j = j as f64 / nj as f64;
            let s = if p.cluster > 0.0 { 1.0 + ((p.cluster * (xi_j - 1.0)).tanh()) / p.cluster.tanh() } else { xi_j };
            x[j * (ni + 1) + i] = xi;
            r[j * (ni + 1) + i] = r_in + s * (r_out - r_in);
        }
    }
    Grid::from_nodes(ni, nj, x, r)
}

#[derive(Clone, Debug)]
pub struct OgiveCase {
    pub mach: f64,
    pub body: Ogive,
    pub cyl_length: f64,
    pub ni_up: usize,
    pub ni_nose: usize,
    pub ni_cyl: usize,
    pub nj: usize,
    pub cfl: f64,
    pub limiter: Limiter,
    pub cluster: f64,
    pub nose_stretch: f64,
    pub max_iter: usize,
    /// Stop when the density residual has dropped this many orders from its maximum.
    pub drop_orders: f64,
    pub first_order_iters: usize,
}

impl OgiveCase {
    /// Grid level 0/1/2 = coarse/medium/fine: 12 / 24 / 48 cells per diameter along the body
    /// (nose 3 d, cylinder 6 d, 0.5 d upstream), 50 / 100 / 200 radial cells.
    pub fn level(mach: f64, level: u32) -> Self {
        let f = 1usize << level;
        let body = Ogive { fineness: 3.0 };
        OgiveCase {
            mach,
            body,
            cyl_length: 6.0,
            ni_up: 6 * f,
            ni_nose: 36 * f,
            ni_cyl: 72 * f,
            nj: 50 * f,
            cfl: 0.5,
            limiter: Limiter::Minmod,
            cluster: 2.0,
            nose_stretch: 1.0,
            max_iter: 40_000,
            drop_orders: 6.0,
            first_order_iters: 300,
        }
    }

    pub fn cells(&self) -> (usize, usize) {
        (self.ni_up + self.ni_nose + self.ni_cyl, self.nj)
    }

    /// Outer-boundary radius at the downstream end: a conservative estimate of the bow-shock radius
    /// (tip-cone shock angle over the nose, then a Mach-angle + 3 deg ray over the cylinder), 10 % margin.
    fn outer_radius(&self) -> f64 {
        let mu = (1.0 / self.mach).asin();
        let beta0 = bowshock_reference::taylor_maccoll::solve_cone(self.mach, self.body.tip_half_angle()).map_or(0.6 * std::f64::consts::PI / 2.0, |s| s.beta);
        1.1 * (self.body.nose_length() * beta0.tan() + self.cyl_length * (mu + 3f64.to_radians()).tan())
    }

    pub fn grid_params(&self) -> OgiveGridParams {
        OgiveGridParams {
            body: self.body,
            ni_up: self.ni_up,
            ni_nose: self.ni_nose,
            ni_cyl: self.ni_cyl,
            nj: self.nj,
            cyl_length: self.cyl_length,
            x_up: -0.5,
            h_up: 2.0,
            h_end: self.outer_radius(),
            cluster: self.cluster,
            nose_stretch: self.nose_stretch,
        }
    }

    pub fn build_solver(&self) -> Solver {
        let cfg = Config {
            cfl: self.cfl,
            limiter: self.limiter,
            freestream: freestream(self.mach),
            bc: [Bc::Freestream, Bc::Extrapolate, Bc::SlipWall, Bc::Freestream],
            first_order_iters: self.first_order_iters,
        };
        Solver::new(ogive_grid(&self.grid_params()), cfg)
    }
}

/// One wall face of the body.
#[derive(Clone, Copy, Debug)]
pub struct WallPoint {
    /// Face centre (x/d, r/d).
    pub x: f64,
    pub r: f64,
    /// `s = (r/R)^2` at the two face end nodes (for the base-area-referenced drag integral).
    pub s0: f64,
    pub s1: f64,
    /// Cp from the wall value extrapolated linearly from cells j=0,1, and from the first cell.
    pub cp: f64,
    pub cp_cell: f64,
}

pub struct OgiveResult {
    pub iters: usize,
    pub residual: Vec<f64>,
    pub drop_orders: f64,
    pub stop_reason: &'static str,
    pub wall: Vec<WallPoint>,
    /// Forebody (nose) pressure-drag coefficient, base-area reference, from extrapolated / first-cell Cp.
    pub cd_nose: f64,
    pub cd_nose_cell: f64,
    /// Smallest relative distance `(h - r_shock)/h` between the detected bow shock and the outer
    /// boundary over `x > 0.5`; must be clearly positive for the free-stream far-field BC to be valid.
    pub shock_margin: f64,
    pub min_rho: f64,
    pub min_p: f64,
}

pub fn run_ogive(case: &OgiveCase) -> OgiveResult {
    let mut s = case.build_solver();
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
        if it >= 8000 && it % 1000 == 0 {
            let m = |a: usize, b: usize| residual[a..b].iter().map(|x| x.log10()).sum::<f64>() / (b - a) as f64;
            if m(it - 2000, it) - m(it - 4000, it - 2000) > -0.05 {
                reason = "residual plateau";
                break;
            }
        }
    }
    s.eval_residual(false);
    let drop_orders = if rmax > 0.0 { (rmax / residual.last().copied().unwrap_or(rmax)).log10() } else { 0.0 };
    postprocess(case, &s, residual, drop_orders, reason)
}

fn postprocess(case: &OgiveCase, s: &Solver, residual: Vec<f64>, drop_orders: f64, reason: &'static str) -> OgiveResult {
    let g = &s.grid;
    let (ni, nj) = (g.ni, g.nj);
    let fs = s.cfg.freestream;
    let (p_inf, q_inf) = (fs[3], 0.5 * fs[0] * fs[1] * fs[1]);
    let cp = |p: f64| (p - p_inf) / q_inf;
    let (ln, rb2) = (case.body.nose_length(), case.body.radius().powi(2));

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

    // Bow-shock position: outermost cell of each column deviating >2 % from free-stream pressure.
    let mut shock_margin = f64::INFINITY;
    for i in 0..ni {
        if g.xc[i] < 0.5 {
            continue;
        }
        let mut jsh = None;
        for j in (0..nj).rev() {
            if (s.pressure(i, j) / p_inf - 1.0).abs() > 0.02 {
                jsh = Some(j);
                break;
            }
        }
        if let Some(j) = jsh {
            let top = g.node(i, nj).1.max(g.node(i + 1, nj).1);
            let rsh = g.rc[j * ni + i];
            shock_margin = shock_margin.min((top - rsh) / top);
        }
    }

    let mut min_rho = f64::INFINITY;
    let mut min_p = f64::INFINITY;
    for q in &s.u {
        let w = bowshock_core::gas::cons_to_prim(q);
        min_rho = min_rho.min(w[0]);
        min_p = min_p.min(w[3]);
    }
    OgiveResult { iters: residual.len(), residual, drop_orders, stop_reason: reason, wall, cd_nose: cd, cd_nose_cell: cd_cell, shock_margin, min_rho, min_p }
}

/// Linear interpolation of the wall `Cp(x)` (extrapolated-wall value) at `x`; NaN outside the sampled range.
pub fn wall_cp_at(wall: &[WallPoint], x: f64) -> f64 {
    for w in wall.windows(2) {
        if x >= w[0].x && x <= w[1].x {
            let t = (x - w[0].x) / (w[1].x - w[0].x);
            return w[0].cp + t * (w[1].cp - w[0].cp);
        }
    }
    f64::NAN
}

/// Convenience: theory object for a case.
pub fn theory_for(case: &OgiveCase) -> SurfaceTheory {
    SurfaceTheory::new(case.mach, case.body)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn grid_nodes_on_body_and_junction() {
        let case = OgiveCase::level(2.0, 0);
        let g = ogive_grid(&case.grid_params());
        let (ni, _) = case.cells();
        assert_eq!(g.ni, ni);
        let mut found_tip = false;
        let mut found_junction = false;
        for i in 0..=g.ni {
            let (x, r) = g.node(i, 0);
            if x == 0.0 {
                found_tip = true;
                assert_eq!(r, 0.0);
            }
            if (x - 3.0).abs() < 1e-12 {
                found_junction = true;
                assert!((r - 0.5).abs() < 1e-12);
            }
            assert!((r - if x > 0.0 { case.body.r(x) } else { 0.0 }).abs() < 1e-12);
        }
        assert!(found_tip && found_junction);
    }

    /// Short run on the coarse grid stays finite and the nose Cp is positive.
    #[test]
    fn coarse_run_is_sane() {
        let mut case = OgiveCase::level(1.98, 0);
        case.max_iter = 1500;
        let res = run_ogive(&case);
        assert!(res.residual.iter().all(|r| r.is_finite()));
        assert!(res.min_rho > 0.0 && res.min_p > 0.0);
        assert!(res.cd_nose > 0.0 && res.cd_nose < 0.2, "CD = {}", res.cd_nose);
    }

    #[test]
    fn experimental_drag_in_band() {
        let body = Ogive { fineness: 3.0 };
        let cd = perkins_jorgensen::forebody_cd(&body, 0.28, -0.038);
        assert!(cd > 0.05 && cd < 0.15, "CD = {cd}");
    }
}
