//! Flat-plate gates of the 3D viscous solver (G-V1 laminar plate against Blasius, G-V2 turbulent plate against the `cf` correlation, the `y+` sweep and the rough
//! plate G2r against Mills & Hang) on a thin box grid: the cells of the 2D plate grid ([`crate::grid::plate_grid`], flat: `r0` is not used) extruded by `nk`
//! cells of width `dz / nk` between two symmetry planes (`k = 0, nk`), with a one-cell slab of solid cells under the plate for `x >= 0`.
//!
//! * The plate surface is the face between the solid slab and the first fluid row: a no-slip wall, adiabatic unless [`PlateCase::wall_thermal`] says otherwise. The slab is one cell
//!   of the height of the first fluid cell (`dr_wall`), so the leading edge is a forward-facing step of that height (G1 `2e-4`, G2 `4e-6`: a fraction of the
//!   boundary-layer thickness at the first station, `5 x / sqrt(Re_x)` = `8e-4` at `x = 0.005` for G1). Upstream of the leading edge the bottom of the domain
//!   (`y = -dr_wall`) is the slip wall of [`Solver3`] (a `j = 0` boundary), as the 2D plate (`wall_i_start`) has.
//! * Boundaries: `i = 0` and `j = nj` characteristic free stream, `i = ni` characteristic outflow (the 2D plate has `PressureOutflow` there).
//! * `cf` comes from the viscous traction of the plate faces ([`Solver3::wall_tractions`]): the very momentum flux the equations see (`(mu + mu_t,w) u / d`).
//!
//! The reference numbers of the 2D solver on the same cases are frozen in `fixtures/ref2d.json` (`plate-g1`, `plate-g2`, `plate-g2r`); the tests print the 3D ones.

use crate::gas3::freestream3;
use crate::grid::{plate_grid, PlateGridParams};
use crate::grid3::{Grid3, Part, SolidBox};
use crate::implicit_params::ImplicitParams;
use crate::limiter::Limiter;
use crate::solver3::{Config3, RansOpts3, Scheme3, Solver3};
use crate::sst::{Turbulence, Viscous, WallThermal};

/// Sutherland constant of the plate cases (`110.4 K / 300 K`, the 2D tests').
pub const SUTHERLAND: f64 = 0.368;

/// One plate case: the 2D grid parameters, the free stream and the extrusion.
#[derive(Clone, Debug)]
pub struct PlateCase {
    pub grid: PlateGridParams,
    pub mach: f64,
    pub re_c: f64,
    pub turbulence: Turbulence,
    /// Sand-grain roughness of the plate (nose diameters), `0` smooth.
    pub ks: f64,
    /// Number of cells and total width of the extrusion (`k`). The width sets the pseudo time step: `DtMode::MinDirection` takes the direction with the smallest spectral
    /// radius, which for a thin `dz` is `k` (`c / dz`), so a `dz` much larger than the streamwise cell `dx_le` gives a time step (and a turbulence update) several times
    /// that of the 2D plate at the same CFL (`dz = 0.1` took 3000 iterations on G1 against 500 at `dz = 0.02`; the rough plate G2r blew up in 20 iterations at `dz = 0.02`):
    /// `dz = dx_le` here.
    pub nk: usize,
    pub dz: f64,
    /// Thickness of the solid slab under the plate in units of `dr_wall` (the height of the leading-edge step); `0`: no slab, the plate is the `j = 0` boundary of the
    /// grid ([`Grid3::wall_j0`]) and the leading edge is step-free, as in the 2D plate ([`PlateCase::boundary_wall`]).
    pub slab: f64,
    /// Thermal condition of the plate (`Adiabatic`, or `Isothermal(T_w / T_inf)`).
    pub wall_thermal: WallThermal,
}

impl PlateCase {
    /// G1: laminar plate, Mach 0.2, `Re_x = 2e5 x`, the 2D test's grid (`dr_wall` 2e-4, height 0.6).
    pub fn g1() -> PlateCase {
        PlateCase {
            grid: PlateGridParams { r0: 20.0, x_up: 0.5, length: 1.0, height: 0.6, n_up: 20, n_plate: 80, nj: 64, dx_le: 0.005, dr_wall: 2e-4 },
            mach: 0.2,
            re_c: 1e6,
            turbulence: Turbulence::Laminar,
            ks: 0.0,
            nk: 1,
            dz: 0.005,
            slab: 1.0,
            wall_thermal: WallThermal::Adiabatic,
        }
    }

    /// G2: fully turbulent plate, SST, Mach 0.2, `re_c = 2.5e7` (`Re_x = 4.85e6` at the gate station `x = 0.97`), the 2D test's grid (`dr_wall` 4e-6, `y+ ~ 0.36`).
    pub fn g2() -> PlateCase {
        PlateCase {
            grid: PlateGridParams { r0: 20.0, x_up: 0.33, length: 2.0, height: 1.0, n_up: 24, n_plate: 112, nj: 72, dx_le: 0.002, dr_wall: 4e-6 },
            mach: 0.2,
            re_c: 2.5e7,
            turbulence: Turbulence::Sst,
            ks: 0.0,
            nk: 1,
            dz: 0.002,
            slab: 1.0,
            wall_thermal: WallThermal::Adiabatic,
        }
    }

    /// G2 with the first cell height scaled to the first-cell `y+` at the gate station: `y+ ~ 0.36 dr_wall / 4e-6` (the 2D G2 grid has `y+ = 0.358` at `4e-6`).
    pub fn g2_yplus(y_plus: f64) -> PlateCase {
        let mut c = PlateCase::g2();
        c.grid.dr_wall = 4e-6 * y_plus / 0.358;
        c
    }

    /// G2r: the G2 plate fully rough, `ks = 1e-3 x` at the gate station (`x = 0.97`).
    pub fn g2r() -> PlateCase {
        PlateCase { ks: 1e-3 * 0.97, ..PlateCase::g2() }
    }

    /// The viscous parameters of the case.
    pub fn viscous(&self) -> Viscous {
        Viscous { re_c: self.re_c, sutherland: SUTHERLAND, turbulence: self.turbulence, wall_i_start: self.grid.n_up, roughness: self.ks }
    }

    /// Index of the leading-edge column (the first column over the solid slab).
    pub fn i_le(&self) -> usize {
        self.grid.n_up
    }

    /// This plate as the `j = 0` boundary wall of the grid, without the solid slab: the geometry of the 2D plate (a step-free leading edge, the slip floor upstream of it).
    pub fn boundary_wall(self) -> PlateCase {
        PlateCase { slab: 0.0, ..self }
    }

    /// The first fluid row over the plate: `1` over the slab, `0` for the boundary wall.
    pub fn j_first(&self) -> usize {
        usize::from(self.slab > 0.0)
    }
}

/// The box grid of a plate case: nodes `x` and `y = r - r0` of the 2D plate grid, one more row of nodes `y = -slab dr_wall` below the plate (none for the boundary wall,
/// [`PlateCase::boundary_wall`]), `nk + 1` planes `z = k dz / nk`; the solid slab is `i >= n_up, j = 0`, the boundary wall the `j = 0` faces of `i >= n_up`.
pub fn plate_grid3(case: &PlateCase) -> Grid3 {
    let g2 = plate_grid(&case.grid);
    let rows = usize::from(case.slab > 0.0);
    let (ni, nj, nk) = (g2.ni, g2.nj + rows, case.nk);
    let (mut x, mut y, mut z) = (Vec::new(), Vec::new(), Vec::new());
    for k in 0..=nk {
        for j in 0..=nj {
            for i in 0..=ni {
                let (xn, yn) = if j < rows {
                    (g2.node(i, 0).0, -case.slab * case.grid.dr_wall)
                } else {
                    let (xx, rr) = g2.node(i, j - rows);
                    (xx, rr - case.grid.r0)
                };
                x.push(xn);
                y.push(yn);
                z.push(k as f64 * case.dz / nk as f64);
            }
        }
    }
    let mut g = Grid3::try_from_nodes(ni, nj, nk, x, y, z).expect("plate grid");
    if rows == 1 {
        g.mark_solid(SolidBox { i: case.i_le()..ni, j: 0..1, k: 0..nk }, Some(Part::Body(0)));
    } else {
        g.wall_j0 = vec![(case.i_le()..ni, Part::Body(0))];
    }
    g
}

/// The solver of a plate case: `params` (the implicit scheme), the low-Mach flux correction on, viscous terms (and SST for a turbulent case) switched on. A rough plate
/// gets its roughness per wall cell with the 2D ramp (`turbulence::rough_ramp`: the first 8 wall cells carry a growing fraction of `ks`; a step feeds turbulence into the
/// slip cells ahead of the wall start).
pub fn plate_solver3(case: &PlateCase, params: ImplicitParams) -> Solver3 {
    let cfg = Config3 {
        freestream: freestream3(case.mach, 0.0),
        cfl: 0.5,
        limiter: Limiter::VanAlbada,
        first_order_iters: 50,
        low_mach: true,
        scheme: Scheme3::Implicit(params),
    };
    let grid = plate_grid3(case);
    let mut opts = RansOpts3 { wall_thermal: case.wall_thermal, ..RansOpts3::default() };
    if case.ks > 0.0 {
        opts.ks = vec![0.0; grid.ni * grid.nj * grid.nk];
        for i in case.i_le()..grid.ni {
            for k in 0..grid.nk {
                opts.ks[grid.cell(i, case.j_first(), k)] = case.ks * crate::sst::rough_ramp(i, case.i_le());
            }
        }
    }
    let mut s = Solver3::new(grid, cfg);
    s.set_viscous_with(Some(case.viscous()), opts);
    s
}

/// Local skin friction and the wall-unit quantities of the plate face nearest to `x_target`.
#[derive(Clone, Copy, Debug)]
pub struct Station {
    /// Centroid `x` of the face.
    pub x: f64,
    /// `cf = tau_x / (0.5 rho_inf U^2)` with the traction of the wall face (`(mu + mu_t,w) u_t / d`).
    pub cf: f64,
    /// First-cell `y+ = d u_tau / nu` with the wall distance `d` of the centroid of the first cell and `u_tau = sqrt(|tau| / rho)`.
    pub y_plus: f64,
    /// Column `i` of the face.
    pub col: usize,
}

/// [`Station`] at `x_target` on the plate of `case` (from the viscous fluxes of the last residual evaluation).
pub fn station(s: &Solver3, case: &PlateCase, x_target: f64) -> Station {
    let g = &s.grid;
    let r = s.rans().expect("viscous run");
    let wall = s.wall_tractions();
    let t = wall
        .iter()
        .filter(|t| t.dir == 1 && t.idx[0] >= case.i_le() && t.n[1] < -0.5)
        .min_by(|a, b| {
            let xa = (g.jface[(a.idx[2] * (g.nj + 1) + a.idx[1]) * g.ni + a.idx[0]].c[0] - x_target).abs();
            let xb = (g.jface[(b.idx[2] * (g.nj + 1) + b.idx[1]) * g.ni + b.idx[0]].c[0] - x_target).abs();
            xa.total_cmp(&xb)
        })
        .expect("plate faces");
    let [i, j, k] = t.idx;
    let face = &g.jface[(k * (g.nj + 1) + j) * g.ni + i];
    let q = 0.5 * s.cfg.freestream[0] * case.mach * case.mach;
    let c = g.cell(i, j, k);
    let w = s.prim(i, j, k);
    let tau = t.tau[0];
    let nu = r.mu[c] / w[0];
    let d = if r.dwall.is_empty() { (g.xc[c][1] - face.c[1]).abs() } else { r.dwall[c] };
    Station { x: face.c[0], cf: tau / q, y_plus: d * (tau.abs() / w[0]).sqrt() / nu, col: i }
}

/// Result of [`run`].
#[derive(Clone, Copy, Debug)]
pub struct RunInfo {
    pub iters: usize,
    pub peak: f64,
    pub last: f64,
    /// The density residual dropped by the requested number of orders.
    pub converged: bool,
}

impl RunInfo {
    pub fn drop_orders(&self) -> f64 {
        (self.peak / self.last).log10()
    }
}

/// Iterate until the density residual has fallen `drop` orders below its peak (after 20 iterations), `max_iters` at most; `probe(iter, solver)` is called every
/// `every` iterations and may stop the run by returning `true`.
pub fn run(s: &mut Solver3, drop: f64, max_iters: usize, every: usize, mut probe: impl FnMut(usize, &Solver3) -> bool) -> RunInfo {
    let (mut peak, mut last, mut iters, mut converged) = (0.0f64, f64::INFINITY, 0, false);
    for it in 0..max_iters {
        let r = s.step();
        iters = it + 1;
        last = r;
        if it > 20 {
            peak = peak.max(r);
            if peak / r >= 10f64.powf(drop) {
                converged = true;
                break;
            }
        }
        if every > 0 && it % every == every - 1 && probe(iters, s) {
            break;
        }
    }
    RunInfo { iters, peak, last, converged }
}

/// Blasius `cf = 0.664 / sqrt(Re_x)`, `Re_x = re_c M x`.
pub fn blasius(case: &PlateCase, x: f64) -> f64 {
    0.664 / (case.re_c * case.mach * x).sqrt()
}

/// The turbulent correlation of G2, `0.37 / log10(Re_x)^2.584` with `Re_x = 5e6 x` (the 2D test's form: `re_c M = 5e6`).
pub fn turbulent_correlation(x: f64) -> f64 {
    0.37 / (5e6 * x).log10().powf(2.584)
}

/// Mills & Hang fully rough `cf = (3.476 + 0.707 ln(x / ks))^-2.46` (`150 < x / ks < 1.5e7`).
pub fn mills_hang(x: f64, ks: f64) -> f64 {
    (3.476 + 0.707 * (x / ks).ln()).powf(-2.46)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::implicit_params::LineSolve3;

    /// The grid is a flat plate: nodes of the plate row on `y = 0` for the columns of the slab, the first fluid row `dr_wall` thick, and a slab of the 2D
    /// parameters.
    #[test]
    fn grid_is_the_extruded_2d_plate_with_a_slab() {
        let case = PlateCase::g1();
        let g = plate_grid3(&case);
        let g2 = plate_grid(&case.grid);
        assert_eq!((g.ni, g.nj, g.nk), (g2.ni, g2.nj + 1, 1));
        assert_eq!(g.solid.len(), 1);
        let i = case.i_le() + 3;
        let c = g.cell(i, 1, 0);
        assert!((g.xc[c][1] - 0.5 * case.grid.dr_wall).abs() < 1e-15, "first fluid cell centre {}", g.xc[c][1]);
        assert!((g.x[g.ni + 1 + case.i_le()] - 0.0).abs() < 1e-15);
        let vol: f64 = (0..g.ni * g.nj).map(|c| g.vol[c]).sum();
        let area = (g2.node(g2.ni, 0).0 - g2.node(0, 0).0) * (g.y[g.nj * (g.ni + 1)] + case.slab * case.grid.dr_wall);
        assert!((vol - area * case.dz).abs() < 1e-9 * vol);
    }

    /// The boundary-wall plate has the 2D plate's rows (no extra row under it), no solid cell and the plate as the `j = 0` boundary faces of `i >= n_up`.
    #[test]
    fn boundary_wall_grid_is_the_2d_plate() {
        let case = PlateCase::g1().boundary_wall();
        let g = plate_grid3(&case);
        let g2 = plate_grid(&case.grid);
        assert_eq!((g.ni, g.nj, g.nk), (g2.ni, g2.nj, 1));
        assert!(g.solid.is_empty() && g.part.iter().all(|&p| p == 0));
        assert_eq!(g.wall_j0, vec![(case.i_le()..g.ni, Part::Body(0))]);
        let c = g.cell(case.i_le() + 3, case.j_first(), 0);
        assert!((g.xc[c][1] - 0.5 * case.grid.dr_wall).abs() < 1e-15, "first cell centre {}", g.xc[c][1]);
        let s = plate_solver3(&case, ImplicitParams::VISCOUS);
        // the plate faces are no-slip walls with a distance field from the boundary faces; the faces upstream of the leading edge are not
        let tractions = s.wall_tractions();
        assert_eq!(tractions.len(), g.ni - case.i_le());
        assert!(tractions.iter().all(|t| t.dir == 1 && t.idx[1] == 0 && t.idx[0] >= case.i_le()));
    }

    /// G1: the laminar plate against Blasius at `x = 0.5` and `0.9` (`Re_x = 1e5`, `1.8e5`), the stop rule of the 2D test (the density residual 5 orders, or both `cf`
    /// stationary to `1e-5` per 250 iterations, 6000 at most).
    #[test]
    fn g1_laminar_plate_matches_blasius() {
        g1_gate(PlateCase::g1());
    }

    /// G1 on the plate as the `j = 0` boundary wall of the grid ([`PlateCase::boundary_wall`]): no slab, a step-free leading edge as in the 2D plate.
    #[test]
    fn g1_laminar_boundary_wall_plate_matches_blasius() {
        g1_gate(PlateCase::g1().boundary_wall());
    }

    fn g1_gate(case: PlateCase) {
        let mut s = plate_solver3(&case, ImplicitParams::VISCOUS);
        let mut prev = [f64::NAN; 2];
        let t0 = std::time::Instant::now();
        let info = run(&mut s, 5.0, 6000, 250, |_, s| {
            let cf = [station(s, &case, 0.5).cf, station(s, &case, 0.9).cf];
            let settled = cf.iter().zip(&prev).all(|(c, p)| (c / p - 1.0).abs() < 1e-5);
            prev = cf;
            settled
        });
        println!(
            "G1 (3D): {} iterations, {:.1} s, density residual {:.3e} (peak {:.3e}, drop {:.2} orders)",
            info.iters,
            t0.elapsed().as_secs_f64(),
            info.last,
            info.peak,
            info.drop_orders()
        );
        let mut worst = 0.0f64;
        for x_target in [0.5, 0.9] {
            let st = station(&s, &case, x_target);
            let b = blasius(&case, st.x);
            println!("G1 (3D): x = {:.4}  cf = {:.6e}  Blasius = {b:.6e}  error = {:+.2} %", st.x, st.cf, 100.0 * (st.cf / b - 1.0));
            worst = worst.max((st.cf / b - 1.0).abs());
        }
        assert!(worst < 0.03, "G1: skin friction off by {:.2} % from Blasius", 100.0 * worst);
    }

    /// Iterate a case to the stop rule of its 2D test: the density residual `drop` orders below its peak, or `cf` at `x_gate` stationary to `1e-5` per 250
    /// iterations (the 2D G1 rule; also applied to the turbulent plates, which otherwise sit on the limiter plateau of the residual).
    fn converge(case: &PlateCase, params: ImplicitParams, drop: f64, max_iters: usize, x_gate: f64) -> (Solver3, RunInfo, f64) {
        let mut s = plate_solver3(case, params);
        let mut prev = f64::NAN;
        let t0 = std::time::Instant::now();
        let info = run(&mut s, drop, max_iters, 250, |it, s| {
            let cf = station(s, case, x_gate).cf;
            let settled = (cf / prev - 1.0).abs() < 1e-5 || !cf.is_finite();
            prev = cf;
            if it % 1000 == 0 {
                println!("  it {it}  cf({x_gate}) = {cf:.6e}  k residual {:.3e}", s.turb_residual());
            }
            settled
        });
        let wall = t0.elapsed().as_secs_f64();
        println!(
            "  {} iterations, {wall:.1} s ({:.1} ms per iteration), density residual {:.3e} (peak {:.3e}, drop {:.2} orders)",
            info.iters,
            1e3 * wall / info.iters as f64,
            info.last,
            info.peak,
            info.drop_orders()
        );
        (s, info, wall)
    }

    /// First-cell `y+`, `cf` and the maximum eddy viscosity of the column at the gate station.
    fn report(s: &Solver3, case: &PlateCase, x_gate: f64) -> (Station, f64) {
        let st = station(s, case, x_gate);
        let r = s.rans().unwrap();
        let mu_t_max = (0..s.grid.nj).map(|j| r.mu_t[s.grid.cell(st.col, j, 0)]).fold(0.0f64, f64::max) * case.re_c;
        (st, mu_t_max)
    }

    /// G2: fully turbulent plate (`Re_x = 4.85e6` at `x = 0.97`): `cf` within 5 % of `0.37 / log10(5e6 x)^2.584`, first-cell `y+ < 2`, developed eddy viscosity.
    #[test]
    #[ignore]
    fn g2_turbulent_plate_matches_the_correlation() {
        g2_gate(PlateCase::g2());
    }

    /// G2 on the plate as the `j = 0` boundary wall of the grid (no slab, a step-free leading edge as in the 2D plate).
    #[test]
    #[ignore]
    fn g2_turbulent_boundary_wall_plate_matches_the_correlation() {
        g2_gate(PlateCase::g2().boundary_wall());
    }

    fn g2_gate(case: PlateCase) {
        let (s, info, _) = converge(&case, ImplicitParams::VISCOUS, 4.0, 20_000, 0.97);
        for x_target in [0.25, 0.5, 0.75] {
            let st = station(&s, &case, x_target);
            let corr = turbulent_correlation(st.x);
            println!("G2 (3D): x = {:.4}  cf = {:.6e}  correlation = {corr:.6e}  ({:+.2} %)  y+ = {:.3}", st.x, st.cf, 100.0 * (st.cf / corr - 1.0), st.y_plus);
        }
        let (st, mu_t_max) = report(&s, &case, 0.97);
        let corr = turbulent_correlation(st.x);
        println!(
            "G2 (3D): x = {:.4}  cf = {:.6e}  correlation = {corr:.6e}  error = {:+.2} %  y+ = {:.3}  max mu_t/mu_inf = {mu_t_max:.1}  ({} iterations)",
            st.x,
            st.cf,
            100.0 * (st.cf / corr - 1.0),
            st.y_plus,
            info.iters
        );
        assert!((st.cf / corr - 1.0).abs() < 0.05, "G2: skin friction off by {:.2} % from the correlation", 100.0 * (st.cf / corr - 1.0));
        assert!(st.y_plus < 2.0, "G2: y+ = {}", st.y_plus);
        assert!(mu_t_max > 50.0, "G2: max mu_t / mu_inf = {mu_t_max}");
    }

    /// The `y+` sweep of G2: the first cell height of the grid scaled so that the first-cell `y+` at the gate station is 0.5, 1, 5, 15, 30 (the automatic wall
    /// treatment): `cf` of every one within 5 % of the `y+ = 0.5` value.
    #[test]
    #[ignore]
    fn g2_yplus_sweep() {
        let mut base = f64::NAN;
        let mut worst = 0.0f64;
        for y in [0.5, 1.0, 5.0, 15.0, 30.0] {
            let case = PlateCase::g2_yplus(y);
            println!("G2 y+ sweep: target y+ = {y}  (dr_wall = {:.3e})", case.grid.dr_wall);
            let (s, info, _) = converge(&case, ImplicitParams::VISCOUS, 4.0, 6000, 0.97);
            let (st, _) = report(&s, &case, 0.97);
            if base.is_nan() {
                base = st.cf;
            }
            let corr = turbulent_correlation(st.x);
            println!(
                "G2 y+ sweep: y+ = {:.3}  cf = {:.6e}  vs y+ 0.5: {:+.2} %  vs correlation {:+.2} %  ({} iterations)",
                st.y_plus,
                st.cf,
                100.0 * (st.cf / base - 1.0),
                100.0 * (st.cf / corr - 1.0),
                info.iters
            );
            worst = worst.max((st.cf / base - 1.0).abs());
        }
        assert!(worst < 0.05, "y+ sweep: cf varies by {:.2} % from the y+ 0.5 value", 100.0 * worst);
    }

    /// G2r: the G2 plate fully rough, `ks = 1e-3 x` (`x = 0.97`): `cf` within 10 % of Mills & Hang `(3.476 + 0.707 ln(x / ks))^-2.46`.
    #[test]
    #[ignore]
    fn g2r_rough_plate_matches_mills_hang() {
        let case = PlateCase::g2r();
        let (s, info, _) = converge(&case, ImplicitParams::VISCOUS, 4.0, 20_000, 0.97);
        let (st, _) = report(&s, &case, 0.97);
        let corr = mills_hang(st.x, case.ks);
        println!(
            "G2r (3D): x = {:.4}  cf = {:.6e}  Mills-Hang = {corr:.6e}  error = {:+.2} %  y+ = {:.3}  ({} iterations)",
            st.x,
            st.cf,
            100.0 * (st.cf / corr - 1.0),
            st.y_plus,
            info.iters
        );
        assert!(info.converged, "G2r: the density residual did not drop 4 orders");
        assert!((st.cf / corr - 1.0).abs() < 0.10, "G2r: skin friction off by {:.2} % from the Mills & Hang correlation", 100.0 * (st.cf / corr - 1.0));
    }

    /// The rough plate G2r at the first-cell `y+` of the Coarse fin stack and above (the `y+` of the smooth first cell 0.5, 5, 15, 30 at the gate station), Mach 0.2 and 2: `cf` against
    /// Mills & Hang and against the resolved (`y+` 0.5) run, and the wall state of the first cell at the gate station (`ks+`, `y+`, `k_w / u_tau^2`, `omega_w nu / u_tau^2`, `mu_t,w / mu`).
    /// `ROUGH_SWEEP_MACH` (default `0.2,2.0`) and `ROUGH_SWEEP_YP` (default `0.5,5,15,30`) restrict it.
    #[test]
    #[ignore]
    fn g2r_yplus_sweep() {
        let list =
            |key: &str, default: &str| -> Vec<f64> { std::env::var(key).unwrap_or_else(|_| default.into()).split(',').map(|v| v.parse().unwrap()).collect() };
        for mach in list("ROUGH_SWEEP_MACH", "0.2,2.0") {
            let mut base = f64::NAN;
            for y in list("ROUGH_SWEEP_YP", "0.5,5,15,30") {
                let mut case = PlateCase::g2r();
                case.mach = mach;
                case.grid.dr_wall = 4e-6 * y / 0.358;
                let (s, info, _) = converge(&case, ImplicitParams::VISCOUS, 4.0, 12_000, 0.97);
                let (st, _) = report(&s, &case, 0.97);
                let r = s.rans().unwrap();
                let c = s.grid.cell(st.col, case.j_first(), 0);
                let (w, rho, mu) = (r.wall[c], s.prim(st.col, case.j_first(), 0)[0], r.mu[c]);
                let nu = mu / rho;
                let u_tau = (st.cf * 0.5 * s.cfg.freestream[0] * mach * mach / rho).abs().sqrt();
                if base.is_nan() {
                    base = st.cf;
                }
                println!(
                    "G2r y+ sweep: M {mach}  target y+ {y:5.1}: cf {:.5e} vs resolved {:+7.2} %  vs Mills-Hang {:+7.2} %  | first cell y+ {:7.2} ks+ {:7.1} k_w+ {:6.3} omega_w+ {:9.3e} mu_t,w/mu {:8.2}  ({} its)",
                    st.cf,
                    100.0 * (st.cf / base - 1.0),
                    100.0 * (st.cf / mills_hang(st.x, case.ks) - 1.0),
                    st.y_plus,
                    u_tau * case.ks / nu,
                    w[0] / (u_tau * u_tau),
                    w[1] * nu / (u_tau * u_tau),
                    w[2] / mu,
                    info.iters
                );
            }
        }
    }

    /// Run `case` under every `(name, line solve, preconditioning, cfl_max)` and print the iterations to the stop rule, the wall time and `cf` at the gate station.
    fn variants(case: &PlateCase, drop: f64, x_gate: f64, list: &[(&str, LineSolve3, bool, f64)]) {
        for &(name, line_solve, low_mach_precond, cfl_max) in list {
            let params = ImplicitParams { cfl_max, line_solve, low_mach_precond, ..ImplicitParams::VISCOUS };
            let (s, info, wall) = converge(case, params, drop, 6000, x_gate);
            let st = station(&s, case, x_gate);
            let finite = s.u.iter().all(|q| q.iter().all(|v| v.is_finite()));
            println!(
                "VARIANT {name:20} cfl {cfl_max:4}: {:5} iterations {wall:6.1} s  cf({x_gate}) = {:.6e}  converged {}  finite {finite}",
                info.iters, st.cf, info.converged
            );
        }
    }

    /// Block / diagonal / block + `P` at CFL 10, 20, 50 on the laminar plate G1 (the 2D scheme runs at 50 subsonic).
    #[test]
    #[ignore]
    fn scheme_variants_on_the_laminar_plate() {
        use LineSolve3::{Block, DiagonalCorrected as Diagonal};
        let list = [
            ("block", Block, false, 10.0),
            ("diagonal", Diagonal, false, 10.0),
            ("block + P", Block, true, 10.0),
            ("diagonal + P", Diagonal, true, 10.0),
            ("block", Block, false, 20.0),
            ("diagonal", Diagonal, false, 20.0),
            ("block + P", Block, true, 20.0),
            ("block", Block, false, 50.0),
            ("diagonal", Diagonal, false, 50.0),
            ("block + P", Block, true, 50.0),
        ];
        variants(&PlateCase::g1(), 5.0, 0.9, &list);
    }

    /// The same on the turbulent plate G2.
    #[test]
    #[ignore]
    fn scheme_variants_on_the_turbulent_plate() {
        use LineSolve3::{Block, DiagonalCorrected as Diagonal};
        let list = [
            ("block", Block, false, 10.0),
            ("diagonal", Diagonal, false, 10.0),
            ("block + P", Block, true, 10.0),
            ("block", Block, false, 20.0),
            ("diagonal", Diagonal, false, 20.0),
            ("block", Block, false, 50.0),
        ];
        variants(&PlateCase::g2(), 4.0, 0.97, &list);
    }

    /// Near-wall profile of the G2 plate at `YPLUS` (environment variable, default 5) at the gate station: `y+`, `U+` against the law of the wall, `mu_t / mu`, `k+`, `omega+` of
    /// the first cells (`cargo nextest run -p bowshock-cfd plate3::tests::near_wall_profile_report --run-ignored only --no-capture`).
    #[test]
    #[ignore]
    fn near_wall_profile_report() {
        let y: f64 = std::env::var("YPLUS").ok().and_then(|v| v.parse().ok()).unwrap_or(5.0);
        let case = PlateCase::g2_yplus(y);
        let (s, _, _) = converge(&case, ImplicitParams::VISCOUS, 4.0, 3000, 0.97);
        let st = station(&s, &case, 0.97);
        let r = s.rans().unwrap();
        let g = &s.grid;
        let u_tau = (st.cf * 0.5 * case.mach * case.mach).abs().sqrt();
        println!("cf {:.4e} u_tau {u_tau:.4e} y+(first) {:.2}", st.cf, st.y_plus);
        println!(" j   y+      U+     log-law  mut/mu   k+      omega+   dwall");
        for j in 1..30 {
            let c = g.cell(st.col, j, 0);
            let w = s.prim(st.col, j, 0);
            let nu = r.mu[c] / w[0];
            let yp = r.dwall[c] * u_tau / nu;
            let up = w[1] / u_tau;
            let log = crate::turbulence3::law_uplus(yp);
            println!(
                "{j:2} {yp:8.2} {up:7.2} {log:7.2}  {:8.2} {:7.3} {:9.3e} {:.2e}",
                r.mu_t[c] / r.mu[c],
                r.turb[c][0] / (u_tau * u_tau),
                r.turb[c][1] * nu / (u_tau * u_tau),
                r.dwall[c]
            );
        }
        println!(
            "wall state of the first cell: k_w {:.3e} omega_w {:.3e} mu_t_w/mu {:.3}",
            r.wall[g.cell(st.col, case.j_first(), 0)][0],
            r.wall[g.cell(st.col, case.j_first(), 0)][1],
            r.wall[g.cell(st.col, case.j_first(), 0)][2] / r.mu[g.cell(st.col, case.j_first(), 0)]
        );
    }
}

#[cfg(test)]
mod thermal_tests {
    use super::*;
    use crate::gas3::GAMMA;
    use crate::sst::PRANDTL;

    /// Skin friction of the laminar flat plate by the reference-temperature (Eckert) method, `cf = 0.664 sqrt(C*) / sqrt(Re_x)` with the Chapman-Rubesin factor
    /// `C* = rho* mu* / (rho_e mu_e)` at `T* / T_e = 0.5 + 0.039 M_e^2 + 0.5 T_w / T_e` (White, Viscous Fluid Flow, section 7-3), the viscosity law of the solver, and the edge state
    /// the free stream (the weak viscous interaction of the plate is not in it). `t_wall`: `T_w / T_inf`, `None`: the adiabatic wall, `T_aw = 1 + sqrt(Pr) (gamma - 1) / 2 M^2`.
    fn cf_reference_temperature(case: &PlateCase, x: f64, t_wall: Option<f64>) -> f64 {
        let m2 = case.mach * case.mach;
        let t_aw = 1.0 + PRANDTL.sqrt() * 0.5 * (GAMMA - 1.0) * m2;
        let t_w = t_wall.unwrap_or(t_aw);
        let t_star = 0.5 + 0.039 * m2 + 0.5 * t_w;
        let v = case.viscous();
        let c_star = v.mu(t_star) / v.mu(1.0) / t_star;
        0.664 * c_star.sqrt() / (case.re_c * case.mach * x).sqrt()
    }

    /// G1 of the supersonic plate (Mach 2, `Re_x = 5e5` at `x = 0.5`) on the step-free boundary-wall grid, with the wall thermal condition `thermal`.
    fn supersonic_plate(thermal: WallThermal) -> PlateCase {
        PlateCase { mach: 2.0, re_c: 5e5, wall_thermal: thermal, ..PlateCase::g1().boundary_wall() }
    }

    /// `cf` at `x = 0.5` and `0.9` of a converged supersonic laminar plate and the energy component of the viscous flux of its wall face at `x = 0.5` (`kappa dT/dy`, positive
    /// where the fluid is hotter than the wall).
    fn run_plate(case: &PlateCase) -> ([f64; 2], f64) {
        let mut s = plate_solver3(case, ImplicitParams::VISCOUS);
        let mut prev = [f64::NAN; 2];
        run(&mut s, 5.0, 6000, 250, |_, s| {
            let cf = [station(s, case, 0.5).cf, station(s, case, 0.9).cf];
            let settled = cf.iter().zip(&prev).all(|(c, p)| (c / p - 1.0).abs() < 1e-5);
            prev = cf;
            settled
        });
        let st = [station(&s, case, 0.5), station(&s, case, 0.9)];
        let q = st[0].col; // the plate is the `j = 0` boundary: face `(i, 0, 0)` of the j family
        let r = s.rans().expect("viscous run");
        (st.map(|t| t.cf), r.fvj[q][4])
    }

    /// A laminar isothermal plate at Mach 2 against the reference-temperature Blasius: the cold wall (`T_w = T_inf`, well below the adiabatic `T_aw = 1.68`) has a friction
    /// coefficient above the adiabatic plate's by `sqrt(C*_cold / C*_adiabatic)` (about 1.15 here). The method's own accuracy (the `T*` fit, the induced pressure of the weak
    /// interaction that the edge state leaves out) is what the 6 % absolute and 3 % ratio bounds allow. The cold plate takes heat out of the fluid (`kappa dT/dy > 0`
    /// at its wall), the adiabatic plate has none.
    #[test]
    fn isothermal_laminar_plate_follows_the_reference_temperature_blasius() {
        let adiabatic = supersonic_plate(WallThermal::Adiabatic);
        let cold = supersonic_plate(WallThermal::Isothermal(1.0));
        let (cf_a, q_a) = run_plate(&adiabatic);
        let (cf_c, q_c) = run_plate(&cold);
        for (n, x) in [0.5, 0.9].into_iter().enumerate() {
            let (ref_a, ref_c) = (cf_reference_temperature(&adiabatic, x, None), cf_reference_temperature(&cold, x, Some(1.0)));
            println!(
                "x = {x}: cf adiabatic {:.5e} (reference {ref_a:.5e}, {:+.2} %)  cold {:.5e} (reference {ref_c:.5e}, {:+.2} %)  ratio {:.4} (reference {:.4})",
                cf_a[n],
                100.0 * (cf_a[n] / ref_a - 1.0),
                cf_c[n],
                100.0 * (cf_c[n] / ref_c - 1.0),
                cf_c[n] / cf_a[n],
                ref_c / ref_a
            );
            assert!((cf_a[n] / ref_a - 1.0).abs() < 0.06, "adiabatic plate x = {x}: cf {:.5e} against {ref_a:.5e}", cf_a[n]);
            assert!((cf_c[n] / ref_c - 1.0).abs() < 0.06, "cold plate x = {x}: cf {:.5e} against {ref_c:.5e}", cf_c[n]);
            assert!(
                (cf_c[n] / cf_a[n] / (ref_c / ref_a) - 1.0).abs() < 0.03,
                "cold / adiabatic ratio x = {x}: {:.4} against {:.4}",
                cf_c[n] / cf_a[n],
                ref_c / ref_a
            );
        }
        println!("wall energy flux Fv[4] at x = 0.5: adiabatic {q_a:.3e}, cold {q_c:.3e}");
        assert!(q_c > 0.0, "the cold wall must pull heat out of the fluid: kappa dT/dy = {q_c}");
        assert!(q_a.abs() < 1e-2 * q_c, "adiabatic wall energy flux {q_a} against the cold {q_c}");
    }
}
