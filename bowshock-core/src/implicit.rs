//! Line-implicit pseudo-time stepping (Beam-Warming-type approximate factorization, "ADI") for the steady Euler iteration.
//!
//! One iteration solves the backward-Euler pseudo-time step `(I/dt - dR/dU) dU = R` with the right-hand side `R` of the full
//! (second-order, limited) residual `Solver::eval_residual` and the *first-order* flux Jacobians on the left, factorised by
//! direction:
//!
//! ```text
//!   (I - dt J_i) X = dt R           block-tridiagonal system along every i line (one per row j)
//!   (I - dt J_j) dU = X             block-tridiagonal system along every j line (one per column i)
//!   U <- U + dU                     (damped by halving until density and pressure are positive)
//! ```
//!
//! `J_d` is the Jacobian of the residual contribution of the fluxes of direction `d`, with the Rusanov-type linearisation
//! of the face flux `F = (F_L + F_R)/2 - lambda (U_R - U_L)/2`, `lambda = max(|u_n| + c)` of the two adjacent states:
//!
//! ```text
//!   dF/dU_L = (A_L + lambda I)/2,      dF/dU_R = (A_R - lambda I)/2,       A = d(F.n)/dU  (`flux_jacobian`)
//! ```
//!
//! Wall, base-wall and far-field ghost cells enter the first/last row of a line through the Jacobian `T = dU_ghost/dU_interior`
//! of the ghost state (exact for slip walls, extrapolation and fixed free stream, finite-difference for the characteristic and
//! pressure-outflow BCs; `ImplicitParams::implicit_bc = false` freezes the ghost states instead).
//!
//! Every line is independent (lines are solved in parallel); the Jacobian blocks are assembled per cell from the cell states
//! and the face metrics, i.e. the whole step is built from per-face, per-cell and per-line kernels. With
//! `ImplicitParams::lines = false` the off-diagonal blocks are dropped (point/block-Jacobi implicit scheme, the baseline the
//! line solves are compared against).
//!
//! The pseudo-time step `dt = CFL A / (L_i + L_j)` (`DtMode::Sum`) or `CFL A / (2 min(L_i, L_j))` (`DtMode::MinDirection`,
//! sized by the weaker direction; the factorised solves are unconditionally stable in the stiff direction) with a CFL that grows
//! geometrically from `cfl0` to `cfl_max` over the iterations since [`Solver::restart_ramp`].

use crate::flux::shock_weight;
use crate::gas::{cons_to_prim, prim_to_cons, reflect, sound_speed, Cons, Prim, GAMMA};
use crate::grid::Face;
use crate::solver::{pidx, Bc, Solver};
use rayon::prelude::*;

/// Row-major 4x4 block.
pub type M4 = [f64; 16];

const ZERO: M4 = [0.0; 16];
const EYE: M4 = [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0];

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DtMode {
    /// `dt = CFL A / (L_i + L_j)` (the explicit definition).
    Sum,
    /// `dt = CFL A / (2 min(L_i, L_j))`: sized by the weaker direction.
    MinDirection,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ImplicitParams {
    /// Pseudo-time CFL: starts at `cfl0` and is multiplied by `growth` every iteration up to `cfl_max`.
    pub cfl0: f64,
    pub cfl_max: f64,
    pub growth: f64,
    pub dt_mode: DtMode,
    /// Solve block-tridiagonal lines (`true`) or drop the off-diagonal blocks (`false`, point-implicit).
    pub lines: bool,
    /// Linearise the ghost-cell states (BCs) in the implicit operator.
    pub implicit_bc: bool,
    /// Defect-correction passes against the unfactored operator (1 = plain approximate factorization).
    pub sub_iters: usize,
    /// Dissipation matrix of the implicit operator: Roe-type `|A|` of the face (`true`) or the scalar Rusanov `lambda I` (`false`).
    pub roe: bool,
    /// Entropy-fix level (fraction of `c`) of the Roe-type dissipation matrix.
    pub eps_fix: f64,
}

impl ImplicitParams {
    pub const DEFAULT: ImplicitParams = ImplicitParams { cfl0: 1.0, cfl_max: 30.0, growth: 1.02, dt_mode: DtMode::Sum, lines: true, implicit_bc: true, sub_iters: 2, roe: true, eps_fix: 0.1 };
}

impl Solver {
    /// Restart the CFL ramp of the implicit scheme (call at the start of a run / after a change of the free stream).
    pub fn restart_ramp(&mut self) {
        self.ramp_k = 0;
    }

    /// Continue the CFL ramp from iteration `k` (a warm start from a converged neighbouring state needs no gentle start-up).
    pub fn set_ramp(&mut self, k: usize) {
        self.ramp_k = k;
    }
}

/// `A = d(F.n)/dU` for the primitive state `w` (conservative variables `(rho, rho u, rho v, E)`).
pub fn flux_jacobian(w: &Prim, nx: f64, nr: f64) -> M4 {
    let [rho, u, v, p] = *w;
    let g1 = GAMMA - 1.0;
    let q2 = u * u + v * v;
    let th = u * nx + v * nr;
    let phi = 0.5 * g1 * q2;
    let h = GAMMA * p / (g1 * rho) + 0.5 * q2;
    [
        0.0, nx, nr, 0.0,
        phi * nx - u * th, th + (2.0 - GAMMA) * u * nx, u * nr - g1 * v * nx, g1 * nx,
        phi * nr - v * th, v * nx - g1 * u * nr, th + (2.0 - GAMMA) * v * nr, g1 * nr,
        th * (phi - h), h * nx - g1 * u * th, h * nr - g1 * v * th, GAMMA * th,
    ]
}

#[inline]
fn mm(a: &M4, b: &M4) -> M4 {
    let mut c = [0.0; 16];
    for i in 0..4 {
        for k in 0..4 {
            let aik = a[4 * i + k];
            for j in 0..4 {
                c[4 * i + j] += aik * b[4 * k + j];
            }
        }
    }
    c
}

#[inline]
fn mv(a: &M4, x: &Cons) -> Cons {
    let mut y = [0.0; 4];
    for i in 0..4 {
        y[i] = a[4 * i] * x[0] + a[4 * i + 1] * x[1] + a[4 * i + 2] * x[2] + a[4 * i + 3] * x[3];
    }
    y
}

#[inline]
fn axpy(a: &mut M4, s: f64, b: &M4) {
    for k in 0..16 {
        a[k] += s * b[k];
    }
}

/// Inverse of a 4x4 matrix (Gauss-Jordan with partial pivoting).
pub fn inv4(m: &M4) -> M4 {
    let mut a = *m;
    let mut inv = EYE;
    for c in 0..4 {
        let mut piv = c;
        for r in c + 1..4 {
            if a[4 * r + c].abs() > a[4 * piv + c].abs() {
                piv = r;
            }
        }
        if piv != c {
            for k in 0..4 {
                a.swap(4 * c + k, 4 * piv + k);
                inv.swap(4 * c + k, 4 * piv + k);
            }
        }
        let d = 1.0 / a[4 * c + c];
        for k in 0..4 {
            a[4 * c + k] *= d;
            inv[4 * c + k] *= d;
        }
        for r in 0..4 {
            if r != c {
                let f = a[4 * r + c];
                if f != 0.0 {
                    for k in 0..4 {
                        a[4 * r + k] -= f * a[4 * c + k];
                        inv[4 * r + k] -= f * inv[4 * c + k];
                    }
                }
            }
        }
    }
    inv
}

/// Block-tridiagonal solve `lo_k x_{k-1} + d_k x_k + up_k x_{k+1} = rhs_k` (in place in `rhs`); `y` is scratch (`n` blocks).
fn block_thomas(n: usize, lo: &[M4], d: &[M4], up: &[M4], rhs: &mut [Cons], y: &mut [M4]) {
    for k in 0..n {
        let mut m = d[k];
        let mut r = rhs[k];
        if k > 0 {
            let t = mm(&lo[k], &y[k - 1]);
            axpy(&mut m, -1.0, &t);
            let lg = mv(&lo[k], &rhs[k - 1]);
            for q in 0..4 {
                r[q] -= lg[q];
            }
        }
        let minv = inv4(&m);
        rhs[k] = mv(&minv, &r);
        if k + 1 < n {
            y[k] = mm(&minv, &up[k]);
        }
    }
    for k in (0..n.saturating_sub(1)).rev() {
        let t = mv(&y[k], &rhs[k + 1]);
        for q in 0..4 {
            rhs[k][q] -= t[q];
        }
    }
}

/// `|A|` of the Roe-averaged state of `wl`, `wr` for the unit normal `(nx, nr)`: `|A| = p(A)` with the quadratic polynomial
/// `p` interpolating `|lambda|` at the three distinct eigenvalues `u_n`, `u_n + c`, `u_n - c`; the eigenvalues are limited
/// below by `0.1 c` (Harten-type entropy fix, quadratic blend) so the operator keeps some dissipation at stagnation and walls.
pub fn abs_jacobian(wl: &Prim, wr: &Prim, nx: f64, nr: f64, eps_fix: f64) -> M4 {
    let (sl, sr) = (wl[0].sqrt(), wr[0].sqrt());
    let inv = 1.0 / (sl + sr);
    let (u, v) = ((sl * wl[1] + sr * wr[1]) * inv, (sl * wl[2] + sr * wr[2]) * inv);
    let hl = GAMMA * wl[3] / ((GAMMA - 1.0) * wl[0]) + 0.5 * (wl[1] * wl[1] + wl[2] * wl[2]);
    let hr = GAMMA * wr[3] / ((GAMMA - 1.0) * wr[0]) + 0.5 * (wr[1] * wr[1] + wr[2] * wr[2]);
    let h = (sl * hl + sr * hr) * inv;
    let c = ((GAMMA - 1.0) * (h - 0.5 * (u * u + v * v))).max(1e-300).sqrt();
    let a = u * nx + v * nr;
    let eps = eps_fix * c;
    let fix = |l: f64| if l.abs() >= eps { l.abs() } else { 0.5 * (l * l / eps + eps) };
    let (e1, e2, e3) = (fix(a), fix(a + c), fix(a - c));
    let d1 = (e2 - e1) / c;
    let d2 = (e2 + e3 - 2.0 * e1) / (2.0 * c * c);
    // Roe-averaged state expressed as primitive variables for the Jacobian: rho is irrelevant for A(u, v, H)
    let wa: Prim = [1.0, u, v, (GAMMA - 1.0) / GAMMA * (h - 0.5 * (u * u + v * v))];
    let am = flux_jacobian(&wa, nx, nr);
    let mut b1 = am;
    let mut b2 = am;
    for q in 0..4 {
        b1[5 * q] -= a;
        b2[5 * q] -= a + c;
    }
    let prod = mm(&b1, &b2);
    let mut out = [0.0; 16];
    for k in 0..16 {
        out[k] = d1 * b1[k] + d2 * prod[k];
    }
    for q in 0..4 {
        out[5 * q] += e1;
    }
    out
}

#[inline]
fn spectral(w: &Prim, nx: f64, nr: f64) -> f64 {
    (w[1] * nx + w[2] * nr).abs() + sound_speed(w)
}

/// Reflection matrix of the conserved variables about the wall normal `(nx, nr)` (momentum mirrored, scalars kept).
fn reflection(nx: f64, nr: f64) -> M4 {
    [1.0, 0.0, 0.0, 0.0, 0.0, 1.0 - 2.0 * nx * nx, -2.0 * nx * nr, 0.0, 0.0, -2.0 * nx * nr, 1.0 - 2.0 * nr * nr, 0.0, 0.0, 0.0, 0.0, 1.0]
}

/// `T = dU_ghost / dU_interior` of the first ghost layer.
fn ghost_jacobian(bc: Bc, fs: &Prim, edge: &Prim, f: &Face, outward: f64) -> M4 {
    match bc {
        Bc::Freestream => ZERO,
        Bc::Extrapolate => EYE,
        Bc::SlipWall => reflection(f.nx, f.nr),
        Bc::Characteristic | Bc::PressureOutflow => {
            let u0 = prim_to_cons(edge);
            let g0 = prim_to_cons(&Solver::ghost_state(bc, fs, edge, edge, f, outward));
            let mut t = ZERO;
            for q in 0..4 {
                let h = 1e-7 * (1.0 + u0[q].abs());
                let mut up = u0;
                up[q] += h;
                let e = cons_to_prim(&up);
                let g = prim_to_cons(&Solver::ghost_state(bc, fs, &e, &e, f, outward));
                for r in 0..4 {
                    t[4 * r + q] = (g[r] - g0[r]) / h;
                }
            }
            t
        }
    }
}

/// Blocks `(lo, d, up)` of `I - dt J` for a cell with states `wk` (cell), `wm` / `wp` (lower / upper neighbour or ghost),
/// lower / upper face `fl`, `fr` (normals towards the upper neighbour), cell volume `vol`.
fn cell_blocks(wk: &Prim, wm: &Prim, wp: &Prim, fl: &Face, fr: &Face, vol: f64, dt: f64, lines: bool, src: f64, roe: bool, eps_fix: f64) -> (M4, M4, M4) {
    let s = 0.5 * dt / vol;
    let (wl, wr) = (fl.w(), fr.w());
    // dissipation matrices of the lower / upper face
    let (dis_l, dis_r) = if roe {
        {
            // same shock sensor as the face flux: Rusanov-like dissipation inside shocks, Roe-type elsewhere
            let (mut dl, mut dr) = (abs_jacobian(wm, wk, fl.nx, fl.nr, eps_fix), abs_jacobian(wk, wp, fr.nx, fr.nr, eps_fix));
            let (tl, tr) = (shock_weight(wm[3], wk[3]), shock_weight(wk[3], wp[3]));
            if tl > 0.0 || tr > 0.0 {
                let lam_l = spectral(wk, fl.nx, fl.nr).max(spectral(wm, fl.nx, fl.nr));
                let lam_r = spectral(wk, fr.nx, fr.nr).max(spectral(wp, fr.nx, fr.nr));
                for x in dl.iter_mut() {
                    *x *= 1.0 - tl;
                }
                for x in dr.iter_mut() {
                    *x *= 1.0 - tr;
                }
                for q in 0..4 {
                    dl[5 * q] += tl * lam_l;
                    dr[5 * q] += tr * lam_r;
                }
            }
            (dl, dr)
        }
    } else {
        let lam_l = spectral(wk, fl.nx, fl.nr).max(spectral(wm, fl.nx, fl.nr));
        let lam_r = spectral(wk, fr.nx, fr.nr).max(spectral(wp, fr.nx, fr.nr));
        let (mut dl, mut dr) = (ZERO, ZERO);
        for q in 0..4 {
            dl[5 * q] = lam_l;
            dr[5 * q] = lam_r;
        }
        (dl, dr)
    };
    let mut d = EYE;
    let ak_r = flux_jacobian(wk, fr.nx, fr.nr);
    let ak_l = flux_jacobian(wk, fl.nx, fl.nr);
    axpy(&mut d, s * wr, &ak_r);
    axpy(&mut d, -s * wl, &ak_l);
    axpy(&mut d, s * wr, &dis_r);
    axpy(&mut d, s * wl, &dis_l);
    if src != 0.0 {
        // axisymmetric pressure source S = (0, 0, p A / V, 0): -dt dS/dU, `src = A / V`
        let g1 = GAMMA - 1.0;
        let c = -dt * src * g1;
        d[8] += c * 0.5 * (wk[1] * wk[1] + wk[2] * wk[2]);
        d[9] += -c * wk[1];
        d[10] += -c * wk[2];
        d[11] += c;
    }
    if !lines {
        return (ZERO, d, ZERO);
    }
    let mut lo = flux_jacobian(wm, fl.nx, fl.nr);
    axpy(&mut lo, 1.0, &dis_l);
    for x in lo.iter_mut() {
        *x *= -s * wl;
    }
    let mut up = flux_jacobian(wp, fr.nx, fr.nr);
    axpy(&mut up, -1.0, &dis_r);
    for x in up.iter_mut() {
        *x *= s * wr;
    }
    (lo, d, up)
}

/// Blocks `(lo, d, up)` of the factor `I - dt J_dir` for every cell, stored per cell index (`c = j ni + i`).
/// Cells of a solid block have zero blocks and are not part of any line.
struct Factor {
    lo: Vec<M4>,
    d: Vec<M4>,
    up: Vec<M4>,
}

/// Assemble the i factor (`dir_i = true`, lines along i) or the j factor.
fn assemble(s: &Solver, p: &ImplicitParams, dir_i: bool) -> Factor {
    let (ni, nj) = (s.grid.ni, s.grid.nj);
    let blk = s.block;
    let fs = s.cfg.freestream;
    let bcs = s.cfg.bc;
    let (grid, w, dt) = (&s.grid, &s.w, &s.dt);
    let ghost_t = |bc: Bc, edge: &Prim, f: &Face, outward: f64| -> M4 {
        if p.implicit_bc {
            ghost_jacobian(bc, &fs, edge, f, outward)
        } else {
            ZERO
        }
    };
    let n = ni * nj;
    let mut fac = Factor { lo: vec![ZERO; n], d: vec![ZERO; n], up: vec![ZERO; n] };
    let (lo, d, up) = (&mut fac.lo, &mut fac.d, &mut fac.up);
    lo.par_chunks_mut(ni)
        .zip(d.par_chunks_mut(ni))
        .zip(up.par_chunks_mut(ni))
        .enumerate()
        .for_each(|(j, ((lo_r, d_r), up_r))| {
            for i in 0..ni {
                if blk.map_or(false, |b| b.solid(i, j)) {
                    d_r[i] = EYE;
                    continue;
                }
                let c = j * ni + i;
                let (ii, jj) = (i as isize, j as isize);
                let wk = &w[pidx(ni, ii, jj)];
                let (src, k, kn, k0) = if dir_i {
                    (0.0, i, ni, blk.map_or(0, |b| b.row_start(j).unwrap_or(ni)))
                } else {
                    (grid.area[c] / grid.vol[c], j, nj, blk.map_or(0, |b| b.col_start(i)))
                };
                let (fl, fr) = if dir_i {
                    (&grid.iface[j * (ni + 1) + i], &grid.iface[j * (ni + 1) + i + 1])
                } else {
                    (&grid.jface[j * ni + i], &grid.jface[(j + 1) * ni + i])
                };
                let (wm_idx, wp_idx) = if dir_i { (pidx(ni, ii - 1, jj), pidx(ni, ii + 1, jj)) } else { (pidx(ni, ii, jj - 1), pidx(ni, ii, jj + 1)) };
                let wall_lower = k == k0 && k0 > 0;
                let wm = if wall_lower && dir_i {
                    // lower neighbour is the solid body: reflection about the base wall
                    let fb = &grid.iface[j * (ni + 1) + k0];
                    reflect(wk, fb.nx, fb.nr)
                } else {
                    w[wm_idx]
                };
                let (mut l, mut dd, mut u) = cell_blocks(wk, &wm, &w[wp_idx], fl, fr, grid.vol[c], dt[c], p.lines, src, p.roe, p.eps_fix);
                if k == k0 {
                    let t = if wall_lower {
                        // solid body below / behind the cell: slip wall with the face normal
                        reflection(fl.nx, fl.nr)
                    } else {
                        ghost_t(if dir_i { bcs[0] } else { bcs[2] }, wk, fl, -1.0)
                    };
                    let lt = mm(&l, &t);
                    axpy(&mut dd, 1.0, &lt);
                    l = ZERO;
                }
                if k == kn - 1 {
                    let t = ghost_t(if dir_i { bcs[1] } else { bcs[3] }, wk, fr, 1.0);
                    let ut = mm(&u, &t);
                    axpy(&mut dd, 1.0, &ut);
                    u = ZERO;
                }
                lo_r[i] = l;
                d_r[i] = dd;
                up_r[i] = u;
            }
        });
    fac
}

/// Segment start of the line through `(i, j)` in the given direction (first fluid cell; the line length if the row is solid).
#[inline]
fn seg_start(blk: Option<crate::solver::Block>, dir_i: bool, ni: usize, i: usize, j: usize) -> usize {
    match blk {
        Some(b) if dir_i => b.row_start(j).unwrap_or(ni),
        Some(b) => b.col_start(i),
        None => 0,
    }
}

/// `x <- (I - dt J_dir)^{-1} x` for every line of the given direction.
fn solve_lines(s: &Solver, fac: &Factor, dir_i: bool, x: &mut [Cons]) {
    let (ni, nj) = (s.grid.ni, s.grid.nj);
    let blk = s.block;
    if dir_i {
        x.par_chunks_mut(ni).enumerate().for_each(|(j, row)| {
            let k0 = seg_start(blk, true, ni, 0, j);
            let n = ni - k0;
            if n == 0 {
                return;
            }
            let r = j * ni + k0..(j + 1) * ni;
            let mut y = vec![ZERO; n];
            block_thomas(n, &fac.lo[r.clone()], &fac.d[r.clone()], &fac.up[r], &mut row[k0..], &mut y);
        });
    } else {
        let x_in: &[Cons] = x;
        let cols: Vec<Vec<Cons>> = (0..ni)
            .into_par_iter()
            .map(|i| {
                let k0 = seg_start(blk, false, ni, i, 0);
                let n = nj - k0;
                let idx = |k: usize| (k0 + k) * ni + i;
                let lo: Vec<M4> = (0..n).map(|k| fac.lo[idx(k)]).collect();
                let d: Vec<M4> = (0..n).map(|k| fac.d[idx(k)]).collect();
                let up: Vec<M4> = (0..n).map(|k| fac.up[idx(k)]).collect();
                let mut rhs: Vec<Cons> = (0..n).map(|k| x_in[idx(k)]).collect();
                let mut y = vec![ZERO; n];
                block_thomas(n, &lo, &d, &up, &mut rhs, &mut y);
                rhs
            })
            .collect();
        for (i, col) in cols.iter().enumerate() {
            let k0 = seg_start(blk, false, ni, i, 0);
            for (k, v) in col.iter().enumerate() {
                x[(k0 + k) * ni + i] = *v;
            }
        }
    }
}

/// `(I - dt J_dir) x` (matrix-vector product with the stored blocks).
fn apply_factor(s: &Solver, fac: &Factor, dir_i: bool, x: &[Cons]) -> Vec<Cons> {
    let (ni, nj) = (s.grid.ni, s.grid.nj);
    let blk = s.block;
    let mut out = vec![[0.0; 4]; ni * nj];
    out.par_chunks_mut(ni).enumerate().for_each(|(j, row)| {
        for i in 0..ni {
            if blk.map_or(false, |b| b.solid(i, j)) {
                continue;
            }
            let c = j * ni + i;
            let k0 = seg_start(blk, dir_i, ni, i, j);
            let (k, kn, stride) = if dir_i { (i, ni, 1) } else { (j, nj, ni) };
            let mut v = mv(&fac.d[c], &x[c]);
            if k > k0 {
                let a = mv(&fac.lo[c], &x[c - stride]);
                for q in 0..4 {
                    v[q] += a[q];
                }
            }
            if k + 1 < kn {
                let a = mv(&fac.up[c], &x[c + stride]);
                for q in 0..4 {
                    v[q] += a[q];
                }
            }
            row[i] = v;
        }
    });
    out
}

/// One implicit iteration; returns the density residual at the start of the iteration.
///
/// The factorised operator `(I - dt J_i)(I - dt J_j)` differs from the unfactored `I - dt J_i - dt J_j` by the cross term
/// `dt^2 J_i J_j`, which is large where both directions are stiff (it slows the weak direction down to the stiff time scale);
/// `ImplicitParams::sub_iters > 1` repeats the solve as a defect correction against the unfactored operator.
pub(crate) fn step(s: &mut Solver, p: &ImplicitParams) -> f64 {
    let cfl = (p.cfl0 * p.growth.powi(s.ramp_k.min(10_000) as i32)).min(p.cfl_max);
    s.ramp_k += 1;
    s.eval_residual(false);
    let rn = s.density_residual();
    compute_dt(s, cfl, p.dt_mode);

    let (ni, nj) = (s.grid.ni, s.grid.nj);
    let blk = s.block;
    let fi = assemble(s, p, true);
    let fj = assemble(s, p, false);

    let b: Vec<Cons> = s.res.iter().zip(s.dt.iter()).map(|(r, d)| [r[0] * d, r[1] * d, r[2] * d, r[3] * d]).collect();
    let af = |rhs: &[Cons]| -> Vec<Cons> {
        let mut x = rhs.to_vec();
        solve_lines(s, &fi, true, &mut x);
        solve_lines(s, &fj, false, &mut x);
        x
    };
    let mut du = af(&b);
    for _ in 1..p.sub_iters.max(1) {
        let (ai, aj) = (apply_factor(s, &fi, true, &du), apply_factor(s, &fj, false, &du));
        let r: Vec<Cons> = (0..ni * nj)
            .into_par_iter()
            .map(|c| {
                let mut v = [0.0; 4];
                for q in 0..4 {
                    v[q] = b[c][q] - (ai[c][q] + aj[c][q] - du[c][q]);
                }
                v
            })
            .collect();
        let corr = af(&r);
        du.par_iter_mut().zip(corr.par_iter()).for_each(|(d, c)| {
            for q in 0..4 {
                d[q] += c[q];
            }
        });
    }

    // ---- update with positivity damping ----
    s.u.par_chunks_mut(ni).enumerate().for_each(|(j, row)| {
        for i in 0..ni {
            if blk.map_or(false, |b| b.solid(i, j)) {
                continue;
            }
            let dq = du[j * ni + i];
            let mut alpha = 1.0;
            for _ in 0..8 {
                let q: Cons = [row[i][0] + alpha * dq[0], row[i][1] + alpha * dq[1], row[i][2] + alpha * dq[2], row[i][3] + alpha * dq[3]];
                let wq = cons_to_prim(&q);
                if wq[0] > 0.0 && wq[3] > 0.0 && q.iter().all(|v| v.is_finite()) {
                    row[i] = q;
                    break;
                }
                alpha *= 0.5;
            }
        }
    });
    rn
}

/// Local pseudo-time step into `s.dt`.
fn compute_dt(s: &mut Solver, cfl: f64, mode: DtMode) {
    let ni = s.grid.ni;
    let blk = s.block;
    let (grid, w) = (&s.grid, &s.w);
    s.dt.par_chunks_mut(ni).enumerate().for_each(|(j, row)| {
        for i in 0..ni {
            if blk.map_or(false, |b| b.solid(i, j)) {
                row[i] = 0.0;
                continue;
            }
            let wc = &w[pidx(ni, i as isize, j as isize)];
            let lam = |f: &Face| spectral(wc, f.nx, f.nr) * f.len;
            let li = 0.5 * (lam(&grid.iface[j * (ni + 1) + i]) + lam(&grid.iface[j * (ni + 1) + i + 1]));
            let lj = 0.5 * (lam(&grid.jface[j * ni + i]) + lam(&grid.jface[(j + 1) * ni + i]));
            let a = grid.area[j * ni + i];
            row[i] = match mode {
                DtMode::Sum => cfl * a / (li + lj),
                DtMode::MinDirection => cfl * a / (2.0 * li.min(lj)),
            };
        }
    });
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::gas::physical_flux;

    #[test]
    fn flux_jacobian_matches_finite_differences() {
        let w: Prim = [1.2, 0.7, -0.3, 0.9];
        for (nx, nr) in [(1.0, 0.0), (0.0, 1.0), (0.6, -0.8), (-0.28, 0.96)] {
            let a = flux_jacobian(&w, nx, nr);
            let u0 = prim_to_cons(&w);
            for q in 0..4 {
                let h = 1e-6;
                let (mut up, mut um) = (u0, u0);
                up[q] += h;
                um[q] -= h;
                let fp = physical_flux(&cons_to_prim(&up), nx, nr);
                let fm = physical_flux(&cons_to_prim(&um), nx, nr);
                for r in 0..4 {
                    let fd = (fp[r] - fm[r]) / (2.0 * h);
                    assert!((fd - a[4 * r + q]).abs() < 1e-7, "n=({nx},{nr}) A[{r}][{q}] = {} vs fd {fd}", a[4 * r + q]);
                }
            }
        }
    }

    #[test]
    fn block_solver_inverts_a_random_diagonally_dominant_system() {
        let n = 7;
        let mut seed = 12345u64;
        let mut rnd = || {
            seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            ((seed >> 33) as f64 / (1u64 << 31) as f64) - 0.5
        };
        let blocks = |rnd: &mut dyn FnMut() -> f64, diag: f64| -> M4 {
            let mut m = ZERO;
            for k in 0..16 {
                m[k] = 0.3 * rnd();
            }
            for q in 0..4 {
                m[5 * q] += diag;
            }
            m
        };
        let lo: Vec<M4> = (0..n).map(|_| blocks(&mut rnd, 0.0)).collect();
        let d: Vec<M4> = (0..n).map(|_| blocks(&mut rnd, 3.0)).collect();
        let up: Vec<M4> = (0..n).map(|_| blocks(&mut rnd, 0.0)).collect();
        let rhs: Vec<Cons> = (0..n).map(|_| [rnd(), rnd(), rnd(), rnd()]).collect();
        let mut x = rhs.clone();
        let mut y = vec![ZERO; n];
        block_thomas(n, &lo, &d, &up, &mut x, &mut y);
        for k in 0..n {
            let mut r = mv(&d[k], &x[k]);
            if k > 0 {
                let a = mv(&lo[k], &x[k - 1]);
                for q in 0..4 {
                    r[q] += a[q];
                }
            }
            if k + 1 < n {
                let a = mv(&up[k], &x[k + 1]);
                for q in 0..4 {
                    r[q] += a[q];
                }
            }
            for q in 0..4 {
                assert!((r[q] - rhs[k][q]).abs() < 1e-12);
            }
        }
    }

    #[test]
    fn abs_jacobian_squares_to_a_squared() {
        // |A|^2 = A^2 (A diagonalisable, no entropy fix active for these eigenvalues)
        let w: Prim = [1.2, 0.5, -0.2, 0.9];
        for (nx, nr) in [(1.0, 0.0), (0.0, 1.0), (0.6, 0.8), (-0.8, 0.6)] {
            let a = flux_jacobian(&w, nx, nr);
            let ab = abs_jacobian(&w, &w, nx, nr, 0.1);
            let (a2, b2) = (mm(&a, &a), mm(&ab, &ab));
            for k in 0..16 {
                assert!((a2[k] - b2[k]).abs() < 1e-10, "n=({nx},{nr}) entry {k}: {} vs {}", a2[k], b2[k]);
            }
        }
    }

    /// A strongly curved axisymmetric grid (axis at j = 0), with a solid block when `block` is set.
    fn curved(ni: usize, nj: usize) -> crate::grid::Grid {
        let (mut x, mut r) = (vec![0.0; (ni + 1) * (nj + 1)], vec![0.0; (ni + 1) * (nj + 1)]);
        for j in 0..=nj {
            for i in 0..=ni {
                let (s, t) = (i as f64 / ni as f64, j as f64 / nj as f64);
                let k = j * (ni + 1) + i;
                x[k] = 2.0 * s + 0.15 * (3.0 * t).sin() * (2.0 * s).cos() + 0.1 * t * t;
                r[k] = t * (1.0 + 0.3 * s) * (1.0 + 0.2 * t) + 0.05 * (4.0 * s).sin() * t * (1.0 - t);
            }
        }
        crate::grid::Grid::from_nodes(ni, nj, x, r)
    }

    #[test]
    fn implicit_scheme_preserves_the_free_stream() {
        use crate::flux::Limiter;
        use crate::gas::freestream;
        use crate::solver::{Config, Scheme};
        for mach in [0.1, 0.5, 0.9] {
            for outflow in [Bc::Characteristic, Bc::PressureOutflow] {
                let fs = freestream(mach);
                let cfg = Config { cfl: 0.5, limiter: Limiter::VanAlbada, freestream: fs, bc: [Bc::Characteristic, outflow, Bc::SlipWall, Bc::Characteristic], first_order_iters: 0 };
                let mut s = Solver::new(curved(24, 16), cfg);
                s.low_mach = true;
                s.set_scheme(Scheme::Implicit(ImplicitParams { cfl_max: 40.0, growth: 1.2, ..ImplicitParams::DEFAULT }));
                for _ in 0..40 {
                    s.step();
                }
                let q0 = prim_to_cons(&fs);
                let worst = s.u.iter().flat_map(|q| (0..4).map(move |k| (q[k] - q0[k]).abs() / q0[k].abs().max(1.0))).fold(0.0, f64::max);
                assert!(worst < 1e-11, "M={mach} {outflow:?}: free-stream drift {worst:e}");
            }
        }
    }
}
