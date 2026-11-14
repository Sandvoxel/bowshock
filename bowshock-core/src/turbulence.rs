//! Menter SST k-omega turbulence model (2003 version) for the viscous solver (`Turbulence::Sst`).
//!
//! The model transports the specific turbulence kinetic energy `k` and the specific dissipation rate `omega` of every cell
//! (`Solver::turb`, `[k, omega]`) with a loosely coupled pseudo-time step at the end of every `Solver::step`; the eddy
//! viscosity `mu_t` it produces (`Solver::mu_t`) enters the mean-flow equations through the viscous terms of `viscous.rs`.
//! Units as in `viscous.rs` (`rho_inf = c_inf = 1`, lengths in nose diameters, `nu = mu / rho`).
//!
//! ```text
//!   constants: a1 = 0.31, beta* = 0.09;  set 1: sigma_k 0.85, sigma_w 0.5, beta 0.075, gamma 5/9;
//!              set 2: sigma_k 1.0, sigma_w 0.856, beta 0.0828, gamma 0.44;    phi = F1 phi_1 + (1 - F1) phi_2
//!   free stream: k_inf = 9e-9,  omega_inf = 1e-6 re_c                         (mu_t / mu = 0.009)
//!   d = distance of the cell centroid to the nearest no-slip wall face
//!   S^2 = 2 (u_x^2 + v_r^2 + (v/r)^2) + (u_r + v_x)^2 - 2/3 div^2,   S = sqrt(max(S^2, 0))     (cell gradients)
//!   X   = 2 rho sigma_w2 (grad k . grad omega) / omega,                CD = max(X, 1e-20)
//!   F1  = tanh(a^4),  a = min( max( sqrt(k) / (beta* omega d), 500 nu / (d^2 omega) ), 4 rho sigma_w2 k / (CD d^2) )
//!   F2  = tanh(b^2),  b = max( 2 sqrt(k) / (beta* omega d), 500 nu / (d^2 omega) )
//!   mu_t = min( rho a1 k / max(a1 omega, S F2), 1e5 / re_c )
//!   P    = min( mu_t S^2, 10 beta* rho k omega )
//!   rho dk/dt     = (-conv_k + diff_k) / V + P - beta* rho omega k
//!   rho domega/dt = (-conv_w + diff_w) / V + gamma rho P / max(mu_t, 1e-30) - beta rho omega^2 + (1 - F1) X
//!   conv = sum over the four faces of (outward mass flow) (phi_upwind - phi_c), inflow faces only; the mass flow of a face is
//!          `F[0] w_f` of the stored inviscid face flux
//!   diff = sum over the four faces of the outward (mu_f + sigma_f mu_t,f) (g_f . n) w_f, `g_f` the corrected face gradient of
//!          `viscous.rs` (a compact normal derivative with the averaged tangential part), `sigma_f` the mean of the blended
//!          `sigma` of the two sides
//! ```
//!
//! Boundary states of the two scalars (the ghost of a face, mirrored in the face centre like the mean-flow ghosts):
//! `Bc::Freestream | Characteristic | PressureOutflow` take `(k_inf, omega_inf)`; `Extrapolate`, slip walls and the axis the
//! cell values; a no-slip wall `k_ghost = -k_c`, `omega_ghost = 2 omega_w - omega_c`, `omega_w = 60 nu_c / (0.075 d_c^2)`
//! (the face values are then `k = 0`, `omega = omega_w`).
//!
//! The update is a sequence of data-parallel kernels ([`update`]): cell gradients of `k`, `omega`; the blending functions; the
//! face coefficients; the per-cell residual and the coefficients of the two linear systems; the line solves; the clipped
//! update with the new eddy viscosity. Per equation, with `D_k = beta* omega`, `D_w = 2 beta omega`, `M-` / `M+` the mass flow
//! in the positive direction through the lower / upper face of a line direction and `G = (mu_f + sigma_f mu_t,f) w_f / |d|`:
//!
//! ```text
//!   diag0 = rho V / dt + rho V D
//!   direction dir:  lo = -(max(M-, 0) + G-),  up = -(max(-M+, 0) + G+),  dd = max(M-, 0) + max(-M+, 0) + G- + G+
//!   i lines:  (diag0 + dd_i) x_c + lo_i x_{c-1} + up_i x_{c+1} = V R_c                    R_c = rho dphi/dt
//!   j lines:  (1 + dd_j / diag0) dphi_c + (lo_j / diag0) dphi_{c-1} + (up_j / diag0) dphi_{c+1} = x_c
//!   phi_new = max(phi + dphi, 0.1 phi), then k >= 1e-3 k_inf, omega >= 1e-3 omega_inf;  then mu_t is recomputed
//! ```
//!
//! A neighbour that is a ghost or a solid cell drops its off-diagonal (the diagonal part stays). The two scalars are solved
//! together with the block-tridiagonal line solver of `implicit.rs` (4x4 blocks that are diagonal: entries 0 and 5 hold the
//! `k` and `omega` coefficients, entries 10 and 15 are `1` on the diagonal block and `0` elsewhere).

use crate::gas::{Cons, Prim};
use crate::grid::{Face, Grid};
use crate::implicit::{solve_lines, Factor, EYE, M4, ZERO};
use crate::solver::{pidx, Bc, Block, Solver, Viscous};
use crate::viscous::{face_mu, iface_centre, iface_sides, jface_centre, jface_sides, stencil, FaceSides, Side, Walls};
use rayon::prelude::*;

/// Free-stream specific turbulence kinetic energy.
pub const K_INF: f64 = 9e-9;

/// Free-stream specific dissipation rate `1e-6 re_c`.
pub fn omega_inf(re_c: f64) -> f64 {
    1e-6 * re_c
}

/// Model constants of the SST model (see the module documentation); public so that GPU ports can take them from here.
pub const A1: f64 = 0.31;
pub const BETA_STAR: f64 = 0.09;
pub const SIGMA_K1: f64 = 0.85;
pub const SIGMA_K2: f64 = 1.0;
pub const SIGMA_W1: f64 = 0.5;
pub const SIGMA_W2: f64 = 0.856;
pub const BETA1: f64 = 0.075;
pub const BETA2: f64 = 0.0828;
pub const GAMMA1: f64 = 5.0 / 9.0;
pub const GAMMA2: f64 = 0.44;

/// Distance of every cell centroid to the nearest no-slip wall face segment of the viscous run `v` with boundary conditions `bc` and
/// body block `blk` (`f64::INFINITY` without a no-slip face and in solid cells): what `Solver::set_viscous` computes for the
/// turbulence model, for ports that keep their own copy of the field.
pub fn wall_distances(grid: &Grid, v: Viscous, bc: &[Bc; 4], blk: Option<Block>) -> Vec<f64> {
    wall_distance(grid, &Walls::new(Some(v), bc, blk))
}

/// Distance of every cell centroid to the nearest no-slip wall face segment (`f64::INFINITY` without a no-slip face and in
/// solid cells); brute force over the wall faces.
pub(crate) fn wall_distance(grid: &Grid, walls: &Walls) -> Vec<f64> {
    let (ni, nj) = (grid.ni, grid.nj);
    let mut segs: Vec<[f64; 4]> = Vec::new();
    for j in 0..=nj {
        for i in 0..ni {
            if walls.noslip_jface(i, j) {
                let (a, b) = (grid.node(i, j), grid.node(i + 1, j));
                segs.push([a.0, a.1, b.0, b.1]);
            }
        }
    }
    for j in 0..nj {
        for i in 0..=ni {
            if walls.noslip_iface(i, j) {
                let (a, b) = (grid.node(i, j), grid.node(i, j + 1));
                segs.push([a.0, a.1, b.0, b.1]);
            }
        }
    }
    let mut d = vec![f64::INFINITY; ni * nj];
    d.par_chunks_mut(ni).enumerate().for_each(|(j, row)| {
        for (i, out) in row.iter_mut().enumerate() {
            if walls.blk.is_some_and(|b| b.solid(i, j)) {
                continue;
            }
            let (px, pr) = (grid.xc[j * ni + i], grid.rc[j * ni + i]);
            for s in &segs {
                let (ex, er) = (s[2] - s[0], s[3] - s[1]);
                let t = (((px - s[0]) * ex + (pr - s[1]) * er) / (ex * ex + er * er)).clamp(0.0, 1.0);
                *out = out.min((px - (s[0] + t * ex)).hypot(pr - (s[1] + t * er)));
            }
        }
    });
    d
}

/// Initialise the turbulence state of `s` (free stream), its wall distance and the eddy viscosity `rho k / omega`.
pub(crate) fn init(s: &mut Solver, re_c: f64) {
    let n = s.grid.ni * s.grid.nj;
    s.turb = vec![[K_INF, omega_inf(re_c)]; n];
    s.dwall = wall_distance(&s.grid, &s.walls());
    let (ni, blk) = (s.grid.ni, s.block);
    for c in 0..n {
        if !blk.is_some_and(|b| b.solid(c % ni, c / ni)) {
            s.mu_t[c] = s.u[c][0] * K_INF / omega_inf(re_c);
        }
    }
}

/// Where a ghost state of a face comes from.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Kind {
    /// The values of the adjacent cell (extrapolation, slip wall, axis).
    Cell,
    /// The free-stream values.
    Inf,
    /// A no-slip wall.
    Wall,
}

fn bc_kind(bc: Bc) -> Kind {
    match bc {
        Bc::Freestream | Bc::Characteristic | Bc::PressureOutflow => Kind::Inf,
        Bc::Extrapolate | Bc::SlipWall => Kind::Cell,
    }
}

/// Read-only data of one update.
struct Ctx<'a> {
    grid: &'a Grid,
    w: &'a [Prim],
    walls: Walls,
    bc: [Bc; 4],
    mu: &'a [f64],
    mu_t: &'a [f64],
    turb: &'a [[f64; 2]],
    dwall: &'a [f64],
    inf: [f64; 2],
}

impl Ctx<'_> {
    #[inline]
    fn solid(&self, i: usize, j: usize) -> bool {
        self.walls.blk.is_some_and(|b| b.solid(i, j))
    }

    #[inline]
    fn rho(&self, c: usize) -> f64 {
        let ni = self.grid.ni;
        self.w[pidx(ni, (c % ni) as isize, (c / ni) as isize)][0]
    }

    /// Sides of i-face `(i, j)` and the origin of its ghost state; `None` inside the solid block.
    fn iface(&self, i: usize, j: usize) -> Option<(FaceSides, Kind)> {
        let sides = iface_sides(self.grid, self.w, &self.walls, i, j)?;
        let kind = if i == 0 {
            bc_kind(self.bc[0])
        } else if i == self.grid.ni {
            bc_kind(self.bc[1])
        } else if sides.l.ghost && sides.noslip {
            Kind::Wall
        } else {
            Kind::Cell
        };
        Some((sides, kind))
    }

    /// Sides of j-face `(i, j)` and the origin of its ghost state; `None` inside the solid block.
    fn jface(&self, i: usize, j: usize) -> Option<(FaceSides, Kind)> {
        let sides = jface_sides(self.grid, self.w, &self.walls, i, j)?;
        let kind = if sides.l.ghost {
            if sides.noslip {
                Kind::Wall
            } else if j == 0 {
                bc_kind(self.bc[2])
            } else {
                Kind::Cell
            }
        } else if sides.r.ghost {
            bc_kind(self.bc[3])
        } else {
            Kind::Cell
        };
        Some((sides, kind))
    }

    /// `[k, omega]` of one side of a face.
    fn kw(&self, s: &Side, kind: Kind) -> [f64; 2] {
        let c = s.cell;
        if !s.ghost {
            return self.turb[c];
        }
        match kind {
            Kind::Cell => self.turb[c],
            Kind::Inf => self.inf,
            Kind::Wall => {
                let [k, om] = self.turb[c];
                let nu = self.mu[c] / self.rho(c);
                let d = self.dwall[c];
                [-k, 2.0 * (60.0 * nu / (BETA1 * d * d)) - om]
            }
        }
    }
}

/// `S^2` of a cell from its gradient `(u_x, u_r, v_x, v_r, ..)`, radial velocity and radius.
#[inline]
fn strain2(g: &[f64; 6], v: f64, r: f64) -> f64 {
    let vr = v / r;
    let div = g[0] + g[3] + vr;
    let shear = g[1] + g[2];
    2.0 * (g[0] * g[0] + g[3] * g[3] + vr * vr) + shear * shear - 2.0 / 3.0 * div * div
}

/// Argument pieces `(sqrt(k) / (beta* omega d), 500 nu / (d^2 omega))` of the blending functions.
#[inline]
fn blend_args(nu: f64, k: f64, om: f64, d: f64) -> (f64, f64) {
    (k.sqrt() / (BETA_STAR * om * d), 500.0 * nu / (d * d * om))
}

/// `F2` of a cell.
#[inline]
fn f2_of(nu: f64, k: f64, om: f64, d: f64) -> f64 {
    let (t1, t2) = blend_args(nu, k, om, d);
    let b = (2.0 * t1).max(t2);
    (b * b).tanh()
}

/// Eddy viscosity `min(rho a1 k / max(a1 omega, S F2), 1e5 / re_c)`.
#[inline]
fn eddy_viscosity(rho: f64, k: f64, om: f64, s: f64, f2: f64, re_c: f64) -> f64 {
    (rho * A1 * k / (A1 * om).max(s * f2)).min(1e5 / re_c)
}

/// Blending functions and cross-diffusion of a cell.
#[derive(Clone, Copy, Default)]
struct Blend {
    f1: f64,
    /// `X = 2 rho sigma_w2 (grad k . grad omega) / omega` (not clamped).
    x: f64,
}

/// Kernel 1: Green-Gauss gradient `(k_x, k_r, omega_x, omega_r)` of every fluid cell, `phi_f = (phi_c + phi_neighbour) / 2`.
fn scalar_gradients(cx: &Ctx) -> Vec<[f64; 4]> {
    let (ni, nj) = (cx.grid.ni, cx.grid.nj);
    let mut out = vec![[0.0; 4]; ni * nj];
    out.par_chunks_mut(ni).enumerate().for_each(|(j, row)| {
        for (i, g_out) in row.iter_mut().enumerate() {
            if cx.solid(i, j) {
                continue;
            }
            let c = j * ni + i;
            let pc = cx.turb[c];
            let (sl, kl) = cx.iface(i, j).expect("fluid cell face");
            let (sr, kr) = cx.iface(i + 1, j).expect("fluid cell face");
            let (sb, kb) = cx.jface(i, j).expect("fluid cell face");
            let (st, kt) = cx.jface(i, j + 1).expect("fluid cell face");
            let faces = [
                (&cx.grid.iface[j * (ni + 1) + i + 1], 1.0, cx.kw(&sr.r, kr)),
                (&cx.grid.iface[j * (ni + 1) + i], -1.0, cx.kw(&sl.l, kl)),
                (&cx.grid.jface[(j + 1) * ni + i], 1.0, cx.kw(&st.r, kt)),
                (&cx.grid.jface[j * ni + i], -1.0, cx.kw(&sb.l, kb)),
            ];
            let mut g = [0.0; 4];
            for (f, sgn, nb) in faces {
                for m in 0..2 {
                    let pf = 0.5 * (pc[m] + nb[m]);
                    g[2 * m] += sgn * pf * f.nx * f.len;
                    g[2 * m + 1] += sgn * pf * f.nr * f.len;
                }
            }
            let inv_a = 1.0 / cx.grid.area[c];
            for x in g.iter_mut() {
                *x *= inv_a;
            }
            *g_out = g;
        }
    });
    out
}

/// Kernel 2: `F1` and the cross-diffusion `X` of every fluid cell.
fn blends(cx: &Ctx, tg: &[[f64; 4]]) -> Vec<Blend> {
    let (ni, nj) = (cx.grid.ni, cx.grid.nj);
    let mut out = vec![Blend::default(); ni * nj];
    out.par_chunks_mut(ni).enumerate().for_each(|(j, row)| {
        for (i, b) in row.iter_mut().enumerate() {
            if cx.solid(i, j) {
                continue;
            }
            let c = j * ni + i;
            let rho = cx.rho(c);
            let nu = cx.mu[c] / rho;
            let [k, om] = cx.turb[c];
            let d = cx.dwall[c];
            let g = &tg[c];
            let x = 2.0 * rho * SIGMA_W2 * (g[0] * g[2] + g[1] * g[3]) / om;
            let cd = x.max(1e-20);
            let (t1, t2) = blend_args(nu, k, om, d);
            let t3 = 4.0 * rho * SIGMA_W2 * k / (cd * d * d);
            let a = t1.max(t2).min(t3);
            *b = Blend { f1: a.powi(4).tanh(), x };
        }
    });
    out
}

/// Scalar diffusion and convection data of one face (`l` / `r`: the two sides, lower / upper index).
#[derive(Clone, Copy, Default)]
struct TFace {
    /// Mass flow `F[0] w_f` in the positive direction.
    m: f64,
    /// Diffusive conductance `G = (mu_f + sigma_f mu_t,f) w_f / |d|` per scalar.
    g: [f64; 2],
    /// Diffusive flux `(mu_f + sigma_f mu_t,f) (g_f . n) w_f` per scalar in the positive direction.
    flux: [f64; 2],
    l: [f64; 2],
    r: [f64; 2],
}

/// Kernel 3 (one face): see the module documentation; `m` is the mass flow of the face, `(xf, rf)` its centre.
fn tface(cx: &Ctx, tg: &[[f64; 4]], blend: &[Blend], sides: &FaceSides, kind: Kind, f: &Face, xf: f64, rf: f64, m: f64) -> TFace {
    let (kl, kr) = (cx.kw(&sides.l, kind), cx.kw(&sides.r, kind));
    if f.rm == 0.0 {
        return TFace { m, g: [0.0; 2], flux: [0.0; 2], l: kl, r: kr };
    }
    let w = f.w();
    let (dd, ex, er) = stencil(cx.grid, sides, xf, rf);
    let (gl, gr) = (&tg[sides.l.cell], &tg[sides.r.cell]);
    let (mu_f, mut_f) = face_mu(cx.mu, cx.mu_t, sides);
    let (f1l, f1r) = (blend[sides.l.cell].f1, blend[sides.r.cell].f1);
    let mut out = TFace { m, g: [0.0; 2], flux: [0.0; 2], l: kl, r: kr };
    for e in 0..2 {
        let (s1, s2) = if e == 0 { (SIGMA_K1, SIGMA_K2) } else { (SIGMA_W1, SIGMA_W2) };
        let sigma = 0.5 * (f1l * s1 + (1.0 - f1l) * s2 + f1r * s1 + (1.0 - f1r) * s2);
        let coef = mu_f + sigma * mut_f;
        let (gbx, gbr) = (0.5 * (gl[2 * e] + gr[2 * e]), 0.5 * (gl[2 * e + 1] + gr[2 * e + 1]));
        let corr = (kr[e] - kl[e]) / dd - (gbx * ex + gbr * er);
        let gn = (gbx + corr * ex) * f.nx + (gbr + corr * er) * f.nr;
        out.flux[e] = coef * gn * w;
        out.g[e] = coef * w / dd;
    }
    out
}

/// Kernel 3: scalar face data of every i-face and j-face.
fn face_pass(cx: &Ctx, tg: &[[f64; 4]], blend: &[Blend], fi: &[Cons], fj: &[Cons]) -> (Vec<TFace>, Vec<TFace>) {
    let (ni, nj) = (cx.grid.ni, cx.grid.nj);
    let mut ti = vec![TFace::default(); (ni + 1) * nj];
    ti.par_chunks_mut(ni + 1).enumerate().for_each(|(j, row)| {
        for (i, out) in row.iter_mut().enumerate() {
            if let Some((sides, kind)) = cx.iface(i, j) {
                let f = &cx.grid.iface[j * (ni + 1) + i];
                let (xf, rf) = iface_centre(cx.grid, i, j);
                *out = tface(cx, tg, blend, &sides, kind, f, xf, rf, fi[j * (ni + 1) + i][0] * f.w());
            }
        }
    });
    let mut tj = vec![TFace::default(); ni * (nj + 1)];
    tj.par_chunks_mut(ni).enumerate().for_each(|(j, row)| {
        for (i, out) in row.iter_mut().enumerate() {
            if let Some((sides, kind)) = cx.jface(i, j) {
                let f = &cx.grid.jface[j * ni + i];
                let (xf, rf) = jface_centre(cx.grid, i, j);
                *out = tface(cx, tg, blend, &sides, kind, f, xf, rf, fj[j * ni + i][0] * f.w());
            }
        }
    });
    (ti, tj)
}

/// Residual and line coefficients of one cell, per scalar.
#[derive(Clone, Copy, Default)]
struct CellTerms {
    /// `R = rho dphi/dt`.
    r: [f64; 2],
    d0: [f64; 2],
    /// `[direction][scalar]`, direction 0 = i, 1 = j.
    lo: [[f64; 2]; 2],
    dd: [[f64; 2]; 2],
    up: [[f64; 2]; 2],
}

/// Kernel 4: residual `R`, `diag0` and the line coefficients of every fluid cell.
fn cell_terms(cx: &Ctx, grad: &[[f64; 6]], blend: &[Blend], dt: &[f64], ti: &[TFace], tj: &[TFace]) -> Vec<CellTerms> {
    let (ni, nj) = (cx.grid.ni, cx.grid.nj);
    let blk = cx.walls.blk;
    let mut out = vec![CellTerms::default(); ni * nj];
    out.par_chunks_mut(ni).enumerate().for_each(|(j, row)| {
        for (i, o) in row.iter_mut().enumerate() {
            if cx.solid(i, j) {
                continue;
            }
            let c = j * ni + i;
            let vol = cx.grid.vol[c];
            let rho = cx.rho(c);
            let [k, om] = cx.turb[c];
            let phi = [k, om];
            let b = blend[c];
            let wc = &cx.w[pidx(ni, i as isize, j as isize)];
            let s2 = strain2(&grad[c], wc[2], cx.grid.rc[c]);
            let mut_c = cx.mu_t[c];
            let prod = (mut_c * s2).min(10.0 * BETA_STAR * rho * k * om);
            let beta = b.f1 * BETA1 + (1.0 - b.f1) * BETA2;
            let gamma = b.f1 * GAMMA1 + (1.0 - b.f1) * GAMMA2;
            let sources = [prod - BETA_STAR * rho * om * k, gamma * rho * prod / mut_c.max(1e-30) - beta * rho * om * om + (1.0 - b.f1) * b.x];
            let dest = [BETA_STAR * om, 2.0 * beta * om];
            let faces = [(&ti[j * (ni + 1) + i], &ti[j * (ni + 1) + i + 1]), (&tj[j * ni + i], &tj[(j + 1) * ni + i])];
            let ends = [(i == blk.map_or(0, |b| b.row_start(j).unwrap_or(ni)), i == ni - 1), (j == blk.map_or(0, |b| b.col_start(i)), j == nj - 1)];
            let mut rate = [0.0; 2];
            for (dir, (lo_f, up_f)) in faces.iter().enumerate() {
                let (a_lo, a_up) = (lo_f.m.max(0.0), (-up_f.m).max(0.0));
                for e in 0..2 {
                    // -conv + diff of this direction
                    rate[e] += a_lo * (lo_f.l[e] - phi[e]) + a_up * (up_f.r[e] - phi[e]) + up_f.flux[e] - lo_f.flux[e];
                    o.lo[dir][e] = if ends[dir].0 { 0.0 } else { -(a_lo + lo_f.g[e]) };
                    o.up[dir][e] = if ends[dir].1 { 0.0 } else { -(a_up + up_f.g[e]) };
                    o.dd[dir][e] = a_lo + a_up + lo_f.g[e] + up_f.g[e];
                }
            }
            for e in 0..2 {
                o.r[e] = rate[e] / vol + sources[e];
                o.d0[e] = rho * vol / dt[c] + rho * vol * dest[e];
            }
        }
    });
    out
}

#[inline]
fn diag(a: f64, b: f64) -> M4 {
    let mut m = ZERO;
    m[0] = a;
    m[5] = b;
    m[10] = 1.0;
    m[15] = 1.0;
    m
}

#[inline]
fn offdiag(a: f64, b: f64) -> M4 {
    let mut m = ZERO;
    m[0] = a;
    m[5] = b;
    m
}

/// One turbulence update at the end of a `Solver::step`, from the primitives, face fluxes, gradients and time step of the
/// residual evaluation of this iteration (see the module documentation): residual and coefficients, the i- and j-line solves,
/// the clipped update of `k`, `omega` and the new eddy viscosity.
pub(crate) fn update(s: &mut Solver) {
    let v = s.viscous.expect("SST needs the viscous terms");
    let (ni, nj) = (s.grid.ni, s.grid.nj);
    let n = ni * nj;
    let inf = [K_INF, omega_inf(v.re_c)];
    let (turb, mu_t, rnorm) = {
        let cx = Ctx { grid: &s.grid, w: &s.w, walls: s.walls(), bc: s.cfg.bc, mu: &s.mu, mu_t: &s.mu_t, turb: &s.turb, dwall: &s.dwall, inf };
        let tg = scalar_gradients(&cx);
        let blend = blends(&cx, &tg);
        let (ti, tj) = face_pass(&cx, &tg, &blend, &s.fi, &s.fj);
        let cells = cell_terms(&cx, &s.grad, &blend, &s.dt, &ti, &tj);

        // coefficients of the two linear systems
        let mut fac_i = Factor { lo: vec![ZERO; n], d: vec![EYE; n], up: vec![ZERO; n] };
        let mut fac_j = Factor { lo: vec![ZERO; n], d: vec![EYE; n], up: vec![ZERO; n] };
        let mut x: Vec<Cons> = vec![[0.0; 4]; n];
        for c in 0..n {
            if cx.solid(c % ni, c / ni) {
                continue;
            }
            let t = &cells[c];
            let vol = s.grid.vol[c];
            x[c] = [vol * t.r[0], vol * t.r[1], 0.0, 0.0];
            fac_i.lo[c] = offdiag(t.lo[0][0], t.lo[0][1]);
            fac_i.up[c] = offdiag(t.up[0][0], t.up[0][1]);
            fac_i.d[c] = diag(t.d0[0] + t.dd[0][0], t.d0[1] + t.dd[0][1]);
            fac_j.lo[c] = offdiag(t.lo[1][0] / t.d0[0], t.lo[1][1] / t.d0[1]);
            fac_j.up[c] = offdiag(t.up[1][0] / t.d0[0], t.up[1][1] / t.d0[1]);
            fac_j.d[c] = diag(1.0 + t.dd[1][0] / t.d0[0], 1.0 + t.dd[1][1] / t.d0[1]);
        }
        solve_lines(s, &fac_i, true, &mut x);
        solve_lines(s, &fac_j, false, &mut x);

        // clipped update and eddy viscosity
        let mut turb = vec![[0.0; 2]; n];
        let mut mu_t = vec![0.0; n];
        turb.par_chunks_mut(ni).zip(mu_t.par_chunks_mut(ni)).enumerate().for_each(|(j, (trow, mrow))| {
            for i in 0..ni {
                let c = j * ni + i;
                if cx.solid(i, j) {
                    continue;
                }
                let [k0, w0] = s.turb[c];
                let k = (k0 + x[c][0]).max(0.1 * k0).max(1e-3 * inf[0]);
                let om = (w0 + x[c][1]).max(0.1 * w0).max(1e-3 * inf[1]);
                let rho = cx.rho(c);
                let nu = cx.mu[c] / rho;
                let g = &s.grad[c];
                let strain = strain2(g, cx.w[pidx(ni, i as isize, j as isize)][2], cx.grid.rc[c]).max(0.0).sqrt();
                trow[i] = [k, om];
                mrow[i] = eddy_viscosity(rho, k, om, strain, f2_of(nu, k, om, cx.dwall[c]), v.re_c);
            }
        });
        let sum: f64 = cells.par_iter().map(|t| t.r[0] * t.r[0]).sum();
        (turb, mu_t, (sum / s.n_active as f64).sqrt())
    };
    s.turb = turb;
    s.mu_t = mu_t;
    s.turb_rnorm = rnorm;
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::flux::Limiter;
    use crate::gas::freestream;
    use crate::grid::{plate_grid, PlateGridParams};
    use crate::implicit::ImplicitParams;
    use crate::solver::{Config, Scheme, Turbulence, Viscous};
    use crate::viscous::temperature;
    use crate::viscous::tests::cf_at;

    /// Flat plate of G2 in the annulus formulation: SST, Mach 0.2, `re_c = 2.5e7`.
    fn plate_solver() -> (Solver, Viscous) {
        let grid = plate_grid(&PlateGridParams { r0: 20.0, x_up: 0.33, length: 2.0, height: 1.0, n_up: 24, n_plate: 112, nj: 72, dx_le: 0.002, dr_wall: 4e-6 });
        let cfg = Config {
            cfl: 0.5,
            limiter: Limiter::VanAlbada,
            freestream: freestream(0.2),
            bc: [Bc::Characteristic, Bc::PressureOutflow, Bc::SlipWall, Bc::Characteristic],
            first_order_iters: 50,
        };
        let mut s = Solver::new(grid, cfg);
        s.low_mach = true;
        s.set_scheme(Scheme::Implicit(ImplicitParams::VISCOUS));
        let v = Viscous { re_c: 2.5e7, sutherland: 0.368, turbulence: Turbulence::Sst, wall_i_start: 24 };
        s.set_viscous(Some(v));
        (s, v)
    }

    #[test]
    fn sst_flat_plate_smoke() {
        let (mut s, _) = plate_solver();
        for _ in 0..300 {
            s.step();
        }
        assert!(s.u.iter().all(|q| q.iter().all(|x| x.is_finite())), "non-finite mean flow");
        assert!(s.turb.iter().all(|t| t[0].is_finite() && t[0] > 0.0 && t[1].is_finite() && t[1] > 0.0), "k, omega must be positive and finite");
        assert!(s.mu_t.iter().all(|m| m.is_finite() && *m >= 0.0), "mu_t must be non-negative");
        assert!(s.turb_residual().is_finite());
    }

    /// G2: fully turbulent flat plate (`Re_x = re_c M x`, `x = 0.97`: `Re_x = 4.85e6`); skin friction within 5 % of the
    /// correlation `0.37 / log10(5e6 x)^2.584`, `y+ < 2` at the first cell, and a developed eddy-viscosity profile.
    #[test]
    #[ignore]
    fn sst_flat_plate_cf() {
        let (mach, re_c) = (0.2, 2.5e7);
        let (mut s, v) = plate_solver();
        let (mut peak, mut last, mut iters) = (0.0f64, f64::INFINITY, 0);
        let mut hist: Vec<(usize, f64)> = Vec::new();
        for it in 0..20_000 {
            let r = s.step();
            iters = it + 1;
            last = r;
            if it > 20 {
                peak = peak.max(r);
            }
            if it % 500 == 499 {
                hist.push((iters, cf_at(&s, &v, mach, 0.97).1));
                println!("G2: it {iters}  density residual {r:.3e}  k residual {:.3e}  cf(0.97) = {:.6e}", s.turb_residual(), hist.last().unwrap().1);
            }
            if it > 20 && peak / r >= 1e4 {
                break;
            }
        }
        println!("G2: {iters} iterations, density residual {last:.3e} (peak {peak:.3e}, drop {:.2} orders)", (peak / last).log10());
        let cf_final = cf_at(&s, &v, mach, 0.97).1;
        let before = hist.iter().rev().find(|(k, _)| k + 2000 <= iters).map(|&(_, c)| c);
        if let Some(c0) = before {
            println!("G2: cf(0.97) change over the last >= 2000 iterations: {:+.3} %", 100.0 * (cf_final / c0 - 1.0));
        }
        for x_target in [0.25, 0.5, 0.75] {
            let (x, cf) = cf_at(&s, &v, mach, x_target);
            let corr = 0.37 / (5e6 * x).log10().powf(2.584);
            println!("G2: x = {x:.4}  cf = {cf:.6e}  correlation = {corr:.6e}  ({:+.2} %)", 100.0 * (cf / corr - 1.0));
        }
        let (x, cf) = cf_at(&s, &v, mach, 0.97);
        let corr = 0.37 / (5e6 * x).log10().powf(2.584);
        // first-cell y+ and the eddy viscosity of the column at the gate station
        let g = &s.grid;
        let i = (v.wall_i_start..g.ni).min_by(|&a, &b| (g.xc[a] - 0.97).abs().total_cmp(&(g.xc[b] - 0.97).abs())).unwrap();
        let f = &g.jface[i];
        let (xf, rf) = (0.5 * (g.node(i, 0).0 + g.node(i + 1, 0).0), 0.5 * (g.node(i, 0).1 + g.node(i + 1, 0).1));
        let dn = ((g.xc[i] - xf) * f.nx + (g.rc[i] - rf) * f.nr).abs();
        let w = s.prim(i, 0);
        let mu_w = v.mu(temperature(&w));
        let ut = w[1] * f.nr - w[2] * f.nx;
        let tau = mu_w * ut / dn;
        let y_plus = dn * (w[0] * tau).sqrt() / mu_w;
        let mu_t_max = (0..g.nj).map(|j| s.mu_t[j * g.ni + i]).fold(0.0f64, f64::max) * re_c;
        println!(
            "G2: x = {x:.4}  cf = {cf:.6e}  correlation = {corr:.6e}  error = {:+.2} %  y+ = {y_plus:.3}  max mu_t/mu_inf = {mu_t_max:.1}",
            100.0 * (cf / corr - 1.0)
        );
        assert!((cf / corr - 1.0).abs() < 0.05, "G2: skin friction off by {:.2} % from the correlation", 100.0 * (cf / corr - 1.0));
        assert!(y_plus < 2.0, "G2: y+ = {y_plus}");
        assert!(mu_t_max > 50.0, "G2: max mu_t / mu_inf = {mu_t_max}");
    }
}
