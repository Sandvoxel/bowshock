//! Viscous terms of the axisymmetric compressible Navier-Stokes equations (laminar; the eddy-viscosity arrays are carried
//! for the turbulence model but stay zero until it is attached).
//!
//! Units: `rho_inf = 1`, `c_inf = 1`, `p_inf = 1/gamma`, `T = gamma p / rho` (`T_inf = 1`), lengths in nose diameters.
//!
//! ```text
//!   mu(T)  = (1 / re_c) T^1.5 (1 + S) / (T + S)            S = sutherland = 110.4 K / T_inf[K],  re_c = rho_inf c_inf d / mu_inf
//!   kappa  = (mu / PRANDTL + mu_t / PRANDTL_T) / (gamma - 1)         q = -kappa grad T        (c_p = 1 / (gamma - 1))
//! ```
//!
//! The residual of a cell uses the total face flux `F - Fv` in place of the Euler flux `F` and the radial-momentum source
//! `(p - t_tt) A / V` in place of `p A / V` (`t_tt` = hoop stress):
//!
//! ```text
//!   V dU/dt = - sum_faces w_f (F - Fv).n + (0, 0, (p - t_tt) A, 0),            w_f = len_f r_m
//!   Fv.n    = [0, txx nx + txr nr, txr nx + trr nr, (u txx + v txr + kappa T_x) nx + (u txr + v trr + kappa T_r) nr]
//!   div     = u_x + v_r + v / r;   txx = mue (2 u_x - 2/3 div);  trr = mue (2 v_r - 2/3 div);  txr = mue (u_r + v_x);  mue = mu + mu_t
//!   t_tt    = (mu_c + mu_t,c) (2 v_c / r_c - 2/3 div_c)
//! ```
//!
//! Three data-parallel passes, called from `Solver::eval_residual` after the inviscid face fluxes:
//!
//! 1. [`cell_pass`]: `mu(T)` and the Green-Gauss gradient `(u_x, u_r, v_x, v_r, T_x, T_r)` of every fluid cell in the meridian
//!    plane, `g = (1/A) sum_faces phi_f n_f len_f` with `phi_f = (phi_c + phi_neighbour) / 2`;
//! 2. [`face_pass`]: the face gradient `g_f = gbar + ((phi_R - phi_L) / |d| - gbar.e) e` (`d` = distance vector of the two
//!    adjacent cell centres, `e = d / |d|`, `gbar` the mean of the two cell gradients: a compact normal derivative with the
//!    averaged tangential part) and the viscous flux of every face into `fvi`, `fvj`; faces on the axis (`r_m = 0`) and inside
//!    the solid block carry no flux;
//! 3. the cell residual in `solver.rs` (with [`hoop_stress`] and the diffusive time-step limit [`diffusion_rate`]).
//!
//! A state across a boundary is the ghost cell of `Solver::w`; the solid cell across a wall of a body block is its ghost as
//! well (block ghost in the j direction, [`wall_ghost`] across the base wall). A ghost sits at the mirror image of the centre
//! of its cell in the face centre and carries the gradient, viscosity and eddy viscosity of that cell (the eddy viscosity with
//! the sign flipped across a no-slip face, so it vanishes at the wall).
//!
//! No-slip walls ([`Walls`]): the face where the viscous flow starts (`Viscous::wall_i_start`) and the body surface, base wall
//! and sting surface of a block. The wall ghost of a no-slip face is `(rho, -u, -v, p)` (adiabatic wall, zero normal pressure
//! gradient), that of a slip face the reflection about the face normal.

use crate::gas::{reflect, Cons, Prim, GAMMA};
use crate::grid::{Face, Grid};
use crate::solver::{pidx, Bc, Block, Viscous, PRANDTL, PRANDTL_T};
use rayon::prelude::*;

impl Viscous {
    /// Dynamic viscosity (Sutherland's law) at the static temperature `t` (free-stream temperature = 1).
    #[inline]
    pub fn mu(&self, t: f64) -> f64 {
        t.powf(1.5) * (1.0 + self.sutherland) / (t + self.sutherland) / self.re_c
    }
}

/// Static temperature `gamma p / rho` of a primitive state.
#[inline]
pub fn temperature(w: &Prim) -> f64 {
    GAMMA * w[3] / w[0]
}

/// Ghost state across a wall face with unit normal `(nx, nr)`: `(rho, -u, -v, p)` on a no-slip face, else the reflection.
#[inline]
pub fn wall_ghost(w: &Prim, f: &Face, noslip: bool) -> Prim {
    if noslip {
        [w[0], -w[1], -w[2], w[3]]
    } else {
        reflect(w, f.nx, f.nr)
    }
}

/// Which wall faces are no-slip (only with viscous terms on). The single classification used by the ghost fills, the flux
/// kernels and the implicit operator.
///
/// * Without a block: a `jmin` face with a slip-wall boundary condition and `i >= wall_i_start`.
/// * With a block: the body-surface faces (`j = j_end`, `wall_i_start <= i < i_end`), every base-wall face (`i = i_end`,
///   `j_sting <= j < j_end`) and every sting-surface face (`j = j_sting > 0`, `i >= i_end`); the axis stays a slip wall.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Walls {
    pub visc: Option<Viscous>,
    pub jmin_slip: bool,
    pub blk: Option<Block>,
}

impl Walls {
    pub(crate) fn new(visc: Option<Viscous>, bc: &[Bc; 4], blk: Option<Block>) -> Walls {
        Walls { visc, jmin_slip: bc[2] == Bc::SlipWall, blk }
    }

    /// Is the j-face `(i, j)` (between cells `j - 1` and `j` of column `i`) a no-slip wall?
    #[inline]
    pub(crate) fn noslip_jface(&self, i: usize, j: usize) -> bool {
        let Some(v) = self.visc else { return false };
        match self.blk {
            None => j == 0 && self.jmin_slip && i >= v.wall_i_start,
            Some(b) => (j == b.j_end && i >= v.wall_i_start && i < b.i_end) || (j == b.j_sting && b.j_sting > 0 && i >= b.i_end),
        }
    }

    /// Is the i-face `(i, j)` (between cells `i - 1` and `i` of row `j`) the no-slip base wall?
    #[inline]
    pub(crate) fn noslip_iface(&self, i: usize, j: usize) -> bool {
        self.visc.is_some() && self.blk.is_some_and(|b| i == b.i_end && j >= b.j_sting && j < b.j_end)
    }
}

/// One side of a face: its primitive state, the fluid cell it belongs to (for a ghost: the adjacent fluid cell) and whether it
/// is a ghost.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Side {
    pub w: Prim,
    pub cell: usize,
    pub ghost: bool,
}

/// The two sides of a face (`l`: lower index, `r`: upper index) and whether the face is a no-slip wall.
#[derive(Clone, Copy, Debug)]
pub(crate) struct FaceSides {
    pub l: Side,
    pub r: Side,
    pub noslip: bool,
}

/// Sides of i-face `(i, j)`; `None` inside the solid block. `w` is the primitive array with ghost layers.
pub(crate) fn iface_sides(grid: &Grid, w: &[Prim], walls: &Walls, i: usize, j: usize) -> Option<FaceSides> {
    let ni = grid.ni;
    let (ii, jj) = (i as isize, j as isize);
    if let Some(b) = walls.blk {
        if j < b.j_end {
            if j < b.j_sting || i < b.i_end {
                return None;
            }
            if i == b.i_end {
                let c = j * ni + i;
                let wr = w[pidx(ni, ii, jj)];
                let noslip = walls.noslip_iface(i, j);
                let wl = wall_ghost(&wr, &grid.iface[j * (ni + 1) + i], noslip);
                return Some(FaceSides { l: Side { w: wl, cell: c, ghost: true }, r: Side { w: wr, cell: c, ghost: false }, noslip });
            }
        }
    }
    let side = |w: Prim, cell: usize, ghost: bool| Side { w, cell, ghost };
    Some(if i == 0 {
        let c = j * ni;
        FaceSides { l: side(w[pidx(ni, -1, jj)], c, true), r: side(w[pidx(ni, 0, jj)], c, false), noslip: false }
    } else if i == ni {
        let c = j * ni + ni - 1;
        FaceSides { l: side(w[pidx(ni, ii - 1, jj)], c, false), r: side(w[pidx(ni, ii, jj)], c, true), noslip: false }
    } else {
        FaceSides { l: side(w[pidx(ni, ii - 1, jj)], j * ni + i - 1, false), r: side(w[pidx(ni, ii, jj)], j * ni + i, false), noslip: false }
    })
}

/// Sides of j-face `(i, j)`; `None` inside the solid block.
pub(crate) fn jface_sides(grid: &Grid, w: &[Prim], walls: &Walls, i: usize, j: usize) -> Option<FaceSides> {
    let (ni, nj) = (grid.ni, grid.nj);
    let (ii, jj) = (i as isize, j as isize);
    let side = |w: Prim, cell: usize, ghost: bool| Side { w, cell, ghost };
    if let Some(b) = walls.blk {
        let j0 = b.col_start(i);
        if j < j0 {
            return None;
        }
        if j == j0 && j0 > 0 {
            // body or sting surface: the solid cell below holds the block ghost
            let c = j * ni + i;
            return Some(FaceSides { l: side(w[pidx(ni, ii, jj - 1)], c, true), r: side(w[pidx(ni, ii, jj)], c, false), noslip: walls.noslip_jface(i, j) });
        }
    }
    Some(if j == 0 {
        let c = i;
        FaceSides { l: side(w[pidx(ni, ii, -1)], c, true), r: side(w[pidx(ni, ii, 0)], c, false), noslip: walls.noslip_jface(i, 0) }
    } else if j == nj {
        let c = (nj - 1) * ni + i;
        FaceSides { l: side(w[pidx(ni, ii, jj - 1)], c, false), r: side(w[pidx(ni, ii, jj)], c, true), noslip: false }
    } else {
        FaceSides { l: side(w[pidx(ni, ii, jj - 1)], (j - 1) * ni + i, false), r: side(w[pidx(ni, ii, jj)], j * ni + i, false), noslip: false }
    })
}

/// Centre of i-face `(i, j)` (mean of its end nodes).
#[inline]
pub(crate) fn iface_centre(grid: &Grid, i: usize, j: usize) -> (f64, f64) {
    let (a, b) = (grid.node(i, j), grid.node(i, j + 1));
    (0.5 * (a.0 + b.0), 0.5 * (a.1 + b.1))
}

/// Centre of j-face `(i, j)`.
#[inline]
pub(crate) fn jface_centre(grid: &Grid, i: usize, j: usize) -> (f64, f64) {
    let (a, b) = (grid.node(i, j), grid.node(i + 1, j));
    (0.5 * (a.0 + b.0), 0.5 * (a.1 + b.1))
}

/// Position of a side: the centre of its cell, or the mirror image of that centre in the face centre for a ghost.
#[inline]
fn side_pos(grid: &Grid, s: &Side, xf: f64, rf: f64) -> (f64, f64) {
    let (x, r) = (grid.xc[s.cell], grid.rc[s.cell]);
    if s.ghost {
        (2.0 * xf - x, 2.0 * rf - r)
    } else {
        (x, r)
    }
}

/// `(|d|, e_x, e_r)`: distance and direction between the two sides.
#[inline]
pub(crate) fn stencil(grid: &Grid, sides: &FaceSides, xf: f64, rf: f64) -> (f64, f64, f64) {
    let (xl, rl) = side_pos(grid, &sides.l, xf, rf);
    let (xr, rr) = side_pos(grid, &sides.r, xf, rf);
    let (dx, dr) = (xr - xl, rr - rl);
    let dd = dx.hypot(dr);
    (dd, dx / dd, dr / dd)
}

/// Face viscosity and eddy viscosity: mean of the two sides, a ghost taking its cell's values (eddy viscosity negated
/// across a no-slip face).
#[inline]
pub(crate) fn face_mu(mu: &[f64], mu_t: &[f64], sides: &FaceSides) -> (f64, f64) {
    let mut_side = |s: &Side| if s.ghost && sides.noslip { -mu_t[s.cell] } else { mu_t[s.cell] };
    (0.5 * (mu[sides.l.cell] + mu[sides.r.cell]), 0.5 * (mut_side(&sides.l) + mut_side(&sides.r)))
}

/// `(u, v, T)` of a primitive state.
#[inline]
fn phi(w: &Prim) -> [f64; 3] {
    [w[1], w[2], temperature(w)]
}

/// Pass 1: viscosity and meridian-plane Green-Gauss gradient `(u_x, u_r, v_x, v_r, T_x, T_r)` of every fluid cell (zero in
/// solid cells).
pub(crate) fn cell_pass(v: &Viscous, grid: &Grid, w: &[Prim], walls: &Walls, grad: &mut [[f64; 6]], mu: &mut [f64]) {
    let ni = grid.ni;
    grad.par_chunks_mut(ni).zip(mu.par_chunks_mut(ni)).enumerate().for_each(|(j, (grow, mrow))| {
        for i in 0..ni {
            if walls.blk.is_some_and(|b| b.solid(i, j)) {
                grow[i] = [0.0; 6];
                mrow[i] = 0.0;
                continue;
            }
            let wc = &w[pidx(ni, i as isize, j as isize)];
            mrow[i] = v.mu(temperature(wc));
            let pc = phi(wc);
            let fl = &grid.iface[j * (ni + 1) + i];
            let fr = &grid.iface[j * (ni + 1) + i + 1];
            let fb = &grid.jface[j * ni + i];
            let ft = &grid.jface[(j + 1) * ni + i];
            let nb_l = iface_sides(grid, w, walls, i, j).expect("fluid cell face").l.w;
            let nb_r = iface_sides(grid, w, walls, i + 1, j).expect("fluid cell face").r.w;
            let nb_b = jface_sides(grid, w, walls, i, j).expect("fluid cell face").l.w;
            let nb_t = jface_sides(grid, w, walls, i, j + 1).expect("fluid cell face").r.w;
            let mut g = [0.0; 6];
            for (f, s, nb) in [(fr, 1.0, &nb_r), (fl, -1.0, &nb_l), (ft, 1.0, &nb_t), (fb, -1.0, &nb_b)] {
                let pn = phi(nb);
                for m in 0..3 {
                    let pf = 0.5 * (pc[m] + pn[m]);
                    g[2 * m] += s * pf * f.nx * f.len;
                    g[2 * m + 1] += s * pf * f.nr * f.len;
                }
            }
            let inv_a = 1.0 / grid.area[j * ni + i];
            for x in g.iter_mut() {
                *x *= inv_a;
            }
            grow[i] = g;
        }
    });
}

/// Viscous flux `Fv.n` of one face (see the module documentation); `f` is the face, `(xf, rf)` its centre.
fn viscous_flux(grid: &Grid, grad: &[[f64; 6]], mu: &[f64], mu_t: &[f64], sides: &FaceSides, f: &Face, xf: f64, rf: f64) -> Cons {
    if f.rm == 0.0 {
        return [0.0; 4];
    }
    let (dd, ex, er) = stencil(grid, sides, xf, rf);
    let (gl, gr) = (&grad[sides.l.cell], &grad[sides.r.cell]);
    let (pl, pr) = (phi(&sides.l.w), phi(&sides.r.w));
    let mut gf = [0.0; 6];
    for m in 0..3 {
        let (gbx, gbr) = (0.5 * (gl[2 * m] + gr[2 * m]), 0.5 * (gl[2 * m + 1] + gr[2 * m + 1]));
        let corr = (pr[m] - pl[m]) / dd - (gbx * ex + gbr * er);
        gf[2 * m] = gbx + corr * ex;
        gf[2 * m + 1] = gbr + corr * er;
    }
    let (mu_f, mut_f) = face_mu(mu, mu_t, sides);
    let (uf, vf) = (0.5 * (sides.l.w[1] + sides.r.w[1]), 0.5 * (sides.l.w[2] + sides.r.w[2]));
    let (u_x, u_r, v_x, v_r, t_x, t_r) = (gf[0], gf[1], gf[2], gf[3], gf[4], gf[5]);
    let div = u_x + v_r + vf / f.rm;
    let mue = mu_f + mut_f;
    let txx = mue * (2.0 * u_x - 2.0 / 3.0 * div);
    let trr = mue * (2.0 * v_r - 2.0 / 3.0 * div);
    let txr = mue * (u_r + v_x);
    let kappa = (mu_f / PRANDTL + mut_f / PRANDTL_T) / (GAMMA - 1.0);
    [0.0, txx * f.nx + txr * f.nr, txr * f.nx + trr * f.nr, (uf * txx + vf * txr + kappa * t_x) * f.nx + (uf * txr + vf * trr + kappa * t_r) * f.nr]
}

/// Pass 2: viscous fluxes of every i-face (`fvi`, `(ni + 1) * nj`) and j-face (`fvj`, `ni * (nj + 1)`).
pub(crate) fn face_pass(grid: &Grid, w: &[Prim], walls: &Walls, grad: &[[f64; 6]], mu: &[f64], mu_t: &[f64], fvi: &mut [Cons], fvj: &mut [Cons]) {
    let ni = grid.ni;
    fvi.par_chunks_mut(ni + 1).enumerate().for_each(|(j, row)| {
        for (i, out) in row.iter_mut().enumerate() {
            *out = match iface_sides(grid, w, walls, i, j) {
                Some(s) => {
                    let (xf, rf) = iface_centre(grid, i, j);
                    viscous_flux(grid, grad, mu, mu_t, &s, &grid.iface[j * (ni + 1) + i], xf, rf)
                }
                None => [0.0; 4],
            };
        }
    });
    fvj.par_chunks_mut(ni).enumerate().for_each(|(j, row)| {
        for (i, out) in row.iter_mut().enumerate() {
            *out = match jface_sides(grid, w, walls, i, j) {
                Some(s) => {
                    let (xf, rf) = jface_centre(grid, i, j);
                    viscous_flux(grid, grad, mu, mu_t, &s, &grid.jface[j * ni + i], xf, rf)
                }
                None => [0.0; 4],
            };
        }
    });
}

/// Hoop stress `t_tt = (mu_c + mu_t,c)(2 v_c / r_c - 2/3 div_c)`, `div_c = u_x + v_r + v_c / r_c`, from the cell gradient `g`.
#[inline]
pub(crate) fn hoop_stress(mue: f64, g: &[f64; 6], v_c: f64, r_c: f64) -> f64 {
    let div = g[0] + g[3] + v_c / r_c;
    mue * (2.0 * v_c / r_c - 2.0 / 3.0 * div)
}

/// Diffusive contribution `2 lv len^2 / A` to the spectral radius `L_d` of one direction (`len`: mean length of the two faces of
/// that direction), with `lv = gamma (mu / PRANDTL + mu_t / PRANDTL_T) / rho`.
#[inline]
pub(crate) fn diffusion_rate(mu_c: f64, mu_t_c: f64, rho_c: f64, len: f64, area: f64) -> f64 {
    let lv = GAMMA * (mu_c / PRANDTL + mu_t_c / PRANDTL_T) / rho_c;
    2.0 * lv * len * len / area
}

/// Diffusive spectral radius `lam = gamma (mu_f / PRANDTL + mu_t,f / PRANDTL_T) / (rho_f |d|)` of the faces `(lower, upper)`
/// of the cell `(i, j)` in the line direction (`0.0` for a face inside the solid block): the implicit operator adds `2 lam I`
/// to the dissipation matrix of the face.
pub(crate) fn face_lambdas(grid: &Grid, w: &[Prim], walls: &Walls, mu: &[f64], mu_t: &[f64], dir_i: bool, i: usize, j: usize) -> (f64, f64) {
    let lam = |sides: Option<FaceSides>, (xf, rf): (f64, f64)| match sides {
        Some(s) => {
            let (dd, _, _) = stencil(grid, &s, xf, rf);
            let (mu_f, mut_f) = face_mu(mu, mu_t, &s);
            let rho_f = 0.5 * (s.l.w[0] + s.r.w[0]);
            GAMMA * (mu_f / PRANDTL + mut_f / PRANDTL_T) / (rho_f * dd)
        }
        None => 0.0,
    };
    if dir_i {
        (lam(iface_sides(grid, w, walls, i, j), iface_centre(grid, i, j)), lam(iface_sides(grid, w, walls, i + 1, j), iface_centre(grid, i + 1, j)))
    } else {
        (lam(jface_sides(grid, w, walls, i, j), jface_centre(grid, i, j)), lam(jface_sides(grid, w, walls, i, j + 1), jface_centre(grid, i, j + 1)))
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    use crate::flux::Limiter;
    use crate::gas::freestream;
    use crate::grid::{plate_grid, PlateGridParams};
    use crate::implicit::ImplicitParams;
    use crate::solver::{Config, Scheme, Solver, Turbulence};

    #[test]
    fn uniform_flow_is_preserved_with_viscous_terms_on_a_curved_axisymmetric_grid() {
        let fs = freestream(0.5);
        let (ni, nj) = (24, 16);
        let cfg = Config {
            cfl: 0.5,
            limiter: Limiter::VanAlbada,
            freestream: fs,
            bc: [Bc::Freestream, Bc::Extrapolate, Bc::SlipWall, Bc::Freestream],
            first_order_iters: 0,
        };
        let mut s = Solver::new(crate::solver::tests::curved_grid(ni, nj), cfg);
        s.set_viscous(Some(Viscous { re_c: 1e4, sutherland: 0.368, turbulence: Turbulence::Laminar, wall_i_start: ni }));
        s.eval_residual(true);
        let m = s.res.iter().flat_map(|r| r.iter()).fold(0.0f64, |a, b| a.max(b.abs()));
        assert!(m < 1e-12, "max |dU/dt| = {m:e}");
    }

    /// Skin friction coefficient `cf = mu_w u_t / d_n / (0.5 M^2)` of the wall face (j = 0, i >= i0) nearest to `x_target`.
    pub(crate) fn cf_at(s: &Solver, v: &Viscous, mach: f64, x_target: f64) -> (f64, f64) {
        let g = &s.grid;
        let (i, _) =
            (v.wall_i_start..g.ni).map(|i| (i, (0.5 * (g.node(i, 0).0 + g.node(i + 1, 0).0) - x_target).abs())).min_by(|a, b| a.1.total_cmp(&b.1)).unwrap();
        let f = &g.jface[i];
        let (xf, rf) = (0.5 * (g.node(i, 0).0 + g.node(i + 1, 0).0), 0.5 * (g.node(i, 0).1 + g.node(i + 1, 0).1));
        let w = s.prim(i, 0);
        let (tx, tr) = (f.nr, -f.nx);
        let ut = w[1] * tx + w[2] * tr;
        let dn = ((g.xc[i] - xf) * f.nx + (g.rc[i] - rf) * f.nr).abs();
        (xf, v.mu(temperature(&w)) * ut / dn / (0.5 * mach * mach))
    }

    /// G1: laminar flat plate (Blasius) in the annulus formulation, `Re_x = re_c M x`.
    ///
    /// The domain height is 0.6: with 0.3 the confinement of the displacement body raises the edge velocity to 1.004-1.007 U
    /// and the measured `cf / Blasius - 1` is +1.4 % (x = 0.50) / +3.2 % (x = 0.89), independent of the wall-normal and
    /// streamwise resolution (n_plate 120, nj 72, dr_wall 1e-4: +1.5 % / +3.5 %); with 0.6 it is +0.64 % / +1.63 %, with 1.2 it
    /// is +0.26 % / +0.76 %. The density residual plateaus about 4.4 orders below its peak (limiter chatter) while `cf` is
    /// stationary to three digits, so the run ends at the 6000-iteration limit (about 25 s in release).
    #[test]
    fn g1_laminar_flat_plate_matches_blasius() {
        let (mach, re_c) = (0.2, 1e6);
        let grid = plate_grid(&PlateGridParams { r0: 20.0, x_up: 0.5, length: 1.0, height: 0.6, n_up: 20, n_plate: 80, nj: 64, dx_le: 0.005, dr_wall: 2e-4 });
        let cfg = Config {
            cfl: 0.5,
            limiter: Limiter::VanAlbada,
            freestream: freestream(mach),
            bc: [Bc::Characteristic, Bc::PressureOutflow, Bc::SlipWall, Bc::Characteristic],
            first_order_iters: 50,
        };
        let mut s = Solver::new(grid, cfg);
        s.low_mach = true;
        s.set_scheme(Scheme::Implicit(ImplicitParams::VISCOUS));
        let v = Viscous { re_c, sutherland: 0.368, turbulence: Turbulence::Laminar, wall_i_start: 20 };
        s.set_viscous(Some(v));
        let (mut peak, mut last, mut iters) = (0.0f64, f64::INFINITY, 0);
        for it in 0..6000 {
            let r = s.step();
            iters = it + 1;
            last = r;
            if it > 20 {
                peak = peak.max(r);
                if peak / r >= 1e5 {
                    break;
                }
            }
        }
        println!("G1: {iters} iterations, density residual {last:.3e} (peak {peak:.3e}, drop {:.2} orders)", (peak / last).log10());
        let mut worst = 0.0f64;
        for x_target in [0.5, 0.9] {
            let (x, cf) = cf_at(&s, &v, mach, x_target);
            let blasius = 0.664 / (re_c * mach * x).sqrt();
            let err = (cf / blasius - 1.0).abs();
            println!("G1: x = {x:.4}  cf = {cf:.6e}  Blasius = {blasius:.6e}  error = {:+.2} %", 100.0 * (cf / blasius - 1.0));
            worst = worst.max(err);
        }
        assert!(worst < 0.03, "G1: skin friction off by {:.2} % from Blasius", 100.0 * worst);
    }
}
