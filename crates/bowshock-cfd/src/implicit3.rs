//! Line-implicit pseudo-time stepping for [`Solver3`]: the three-factor approximate factorization ("ADI") of
//! [`crate::implicit`] with 5x5 blocks.
//!
//! One iteration solves `(I/dt - dR/dU) dU = R` with the right-hand side `R` of the full (second-order, limited) residual
//! and the *first-order* flux Jacobians on the left, factorised by direction and solved i, then j, then k:
//!
//! ```text
//!   (I - dt J_i) X = dt R,   (I - dt J_j) Y = X,   (I - dt J_k) dU = Y,   U <- U + dU  (halved until rho, p > 0)
//! ```
//!
//! Face linearisation, entropy-fixed Roe `|A|`, shock-sensor blend, ghost Jacobians and the defect-correction loop
//! (`r = b - (A_i du + A_j du + A_k du - 2 du)`) are those of `implicit.rs`. Differences:
//!
//! * Lines are cut at solid cells: a solid cell is the identity row (`d = I`, `lo = up = 0`, `rhs = 0`), so a line through a
//!   fin solves as independent segments. A fluid cell next to a solid cell along the line drops that off-diagonal block `B` and
//!   adds `B T` to its diagonal, `T` the reflection about the wall face normal (as for a wall boundary).
//! * Ghost Jacobians on the first / last cell of a line: characteristic far field `T = 0` (frozen ghost), slip wall and
//!   symmetry planes `T = reflection3(n_face)`, the axis `T = 0` (the ghost depends on another cell, frozen);
//!   `ImplicitParams::implicit_bc = false` freezes the wall ones too.
//! * Euler only: no source term and no viscous terms.
//!
//! `ImplicitParams::line_solve = LineSolve3::DiagonalCorrected` replaces the block lines by diagonalised factors (`diag3`): per cell the eigensystem
//! of the flux Jacobian turns each directional solve into five scalar tridiagonal solves, and one defect-correction pass against the block
//! operator recovers the accuracy of the block scheme.

mod diag3;

use crate::flux::shock_weight;
use crate::gas3::{cons_to_prim3, reflect3, sound_speed3, Cons3, Prim3, GAMMA};
use crate::grid3::Face3;
use crate::implicit::{ImplicitParams, LineSolve3};
use crate::solver3::{pidx3, Solver3};
use rayon::prelude::*;

/// Row-major 5x5 block.
pub type M5 = [f64; 25];

pub(crate) const ZERO5: M5 = [0.0; 25];
pub(crate) const EYE5: M5 = {
    let mut m = [0.0; 25];
    let mut q = 0;
    while q < 5 {
        m[6 * q] = 1.0;
        q += 1;
    }
    m
};

/// `A = d(F.n)/dU` for the primitive state `w` (conservative variables `(rho, rho u, rho v, rho w, E)`).
#[rustfmt::skip]
pub fn flux_jacobian3(w: &Prim3, n: [f64; 3]) -> M5 {
    let [rho, u, v, ww, p] = *w;
    let [nx, ny, nz] = n;
    let g1 = GAMMA - 1.0;
    let q2 = u * u + v * v + ww * ww;
    let th = u * nx + v * ny + ww * nz;
    let phi = 0.5 * g1 * q2;
    let h = GAMMA * p / (g1 * rho) + 0.5 * q2;
    [
        0.0, nx, ny, nz, 0.0,
        phi * nx - u * th, th + (2.0 - GAMMA) * u * nx, u * ny - g1 * v * nx, u * nz - g1 * ww * nx, g1 * nx,
        phi * ny - v * th, v * nx - g1 * u * ny, th + (2.0 - GAMMA) * v * ny, v * nz - g1 * ww * ny, g1 * ny,
        phi * nz - ww * th, ww * nx - g1 * u * nz, ww * ny - g1 * v * nz, th + (2.0 - GAMMA) * ww * nz, g1 * nz,
        th * (phi - h), h * nx - g1 * u * th, h * ny - g1 * v * th, h * nz - g1 * ww * th, GAMMA * th,
    ]
}

#[inline]
fn mm(a: &M5, b: &M5) -> M5 {
    let mut c = [0.0; 25];
    for i in 0..5 {
        for k in 0..5 {
            let aik = a[5 * i + k];
            for j in 0..5 {
                c[5 * i + j] += aik * b[5 * k + j];
            }
        }
    }
    c
}

#[inline]
fn mv(a: &M5, x: &Cons3) -> Cons3 {
    let mut y = [0.0; 5];
    for i in 0..5 {
        y[i] = a[5 * i] * x[0] + a[5 * i + 1] * x[1] + a[5 * i + 2] * x[2] + a[5 * i + 3] * x[3] + a[5 * i + 4] * x[4];
    }
    y
}

#[inline]
fn axpy(a: &mut M5, s: f64, b: &M5) {
    for k in 0..25 {
        a[k] += s * b[k];
    }
}

/// Inverse of a 5x5 matrix (Gauss-Jordan with partial pivoting).
pub fn inv5(m: &M5) -> M5 {
    let mut a = *m;
    let mut inv = EYE5;
    for c in 0..5 {
        let mut piv = c;
        for r in c + 1..5 {
            if a[5 * r + c].abs() > a[5 * piv + c].abs() {
                piv = r;
            }
        }
        if piv != c {
            for k in 0..5 {
                a.swap(5 * c + k, 5 * piv + k);
                inv.swap(5 * c + k, 5 * piv + k);
            }
        }
        let d = 1.0 / a[5 * c + c];
        for k in 0..5 {
            a[5 * c + k] *= d;
            inv[5 * c + k] *= d;
        }
        for r in 0..5 {
            if r != c {
                let f = a[5 * r + c];
                if f != 0.0 {
                    for k in 0..5 {
                        a[5 * r + k] -= f * a[5 * c + k];
                        inv[5 * r + k] -= f * inv[5 * c + k];
                    }
                }
            }
        }
    }
    inv
}

/// Blocks `(lo, d, up)` of `I - dt J` of every cell, stored per cell index (`Grid3::cell`).
pub(crate) struct Factor {
    pub(crate) lo: Vec<M5>,
    pub(crate) d: Vec<M5>,
    pub(crate) up: Vec<M5>,
}

/// Block-tridiagonal solve of the line of `n` cells at `base + m * stride` of `f`: `lo_m x_{m-1} + d_m x_m + up_m x_{m+1} = rhs_m`
/// (in place in `rhs`, local to the line); `y` is scratch (`n` blocks).
fn block_thomas5(n: usize, base: usize, stride: usize, f: &Factor, rhs: &mut [Cons3], y: &mut [M5]) {
    for k in 0..n {
        let c = base + k * stride;
        let mut m = f.d[c];
        let mut r = rhs[k];
        if k > 0 {
            let t = mm(&f.lo[c], &y[k - 1]);
            axpy(&mut m, -1.0, &t);
            let lg = mv(&f.lo[c], &rhs[k - 1]);
            for q in 0..5 {
                r[q] -= lg[q];
            }
        }
        let minv = inv5(&m);
        rhs[k] = mv(&minv, &r);
        if k + 1 < n {
            y[k] = mm(&minv, &f.up[c]);
        }
    }
    for k in (0..n.saturating_sub(1)).rev() {
        let t = mv(&y[k], &rhs[k + 1]);
        for q in 0..5 {
            rhs[k][q] -= t[q];
        }
    }
}

/// `|A|` of the Roe-averaged state of `wl`, `wr` for the unit normal `n`: `|A| = p(A)` with the quadratic polynomial `p`
/// interpolating `|lambda|` at the three distinct eigenvalues `u_n`, `u_n + c`, `u_n - c`; the eigenvalues are limited below by
/// `eps_fix * c` (Harten-type entropy fix, quadratic blend).
pub fn abs_jacobian3(wl: &Prim3, wr: &Prim3, n: [f64; 3], eps_fix: f64) -> M5 {
    let (sl, sr) = (wl[0].sqrt(), wr[0].sqrt());
    let inv = 1.0 / (sl + sr);
    let (u, v, ww) = ((sl * wl[1] + sr * wr[1]) * inv, (sl * wl[2] + sr * wr[2]) * inv, (sl * wl[3] + sr * wr[3]) * inv);
    let ke = |w: &Prim3| 0.5 * (w[1] * w[1] + w[2] * w[2] + w[3] * w[3]);
    let hl = GAMMA * wl[4] / ((GAMMA - 1.0) * wl[0]) + ke(wl);
    let hr = GAMMA * wr[4] / ((GAMMA - 1.0) * wr[0]) + ke(wr);
    let h = (sl * hl + sr * hr) * inv;
    let q2h = 0.5 * (u * u + v * v + ww * ww);
    let c = ((GAMMA - 1.0) * (h - q2h)).max(1e-300).sqrt();
    let a = u * n[0] + v * n[1] + ww * n[2];
    let eps = eps_fix * c;
    let fix = |l: f64| if l.abs() >= eps { l.abs() } else { 0.5 * (l * l / eps + eps) };
    let (e1, e2, e3) = (fix(a), fix(a + c), fix(a - c));
    let d1 = (e2 - e1) / c;
    let d2 = (e2 + e3 - 2.0 * e1) / (2.0 * c * c);
    // Roe-averaged state as primitive variables for the Jacobian: rho is irrelevant for A(u, v, w, H)
    let wa: Prim3 = [1.0, u, v, ww, (GAMMA - 1.0) / GAMMA * (h - q2h)];
    let am = flux_jacobian3(&wa, n);
    let (mut b1, mut b2) = (am, am);
    for q in 0..5 {
        b1[6 * q] -= a;
        b2[6 * q] -= a + c;
    }
    let prod = mm(&b1, &b2);
    let mut out = [0.0; 25];
    for k in 0..25 {
        out[k] = d1 * b1[k] + d2 * prod[k];
    }
    for q in 0..5 {
        out[6 * q] += e1;
    }
    out
}

#[inline]
fn spectral(w: &Prim3, n: [f64; 3]) -> f64 {
    (w[1] * n[0] + w[2] * n[1] + w[3] * n[2]).abs() + sound_speed3(w)
}

/// Reflection matrix of the conserved variables about the wall normal `n` (momentum mirrored, scalars kept).
pub fn reflection3(n: [f64; 3]) -> M5 {
    let mut m = EYE5;
    for a in 0..3 {
        for b in 0..3 {
            m[5 * (1 + a) + 1 + b] -= 2.0 * n[a] * n[b];
        }
    }
    m
}

/// Blocks `(lo, d, up)` of `I - dt J` of a cell with states `wk` (cell), `wm` / `wp` (lower / upper neighbour or ghost), faces
/// `fl`, `fr` (normals towards the upper neighbour) and volume `vol`.
fn cell_blocks3(wk: &Prim3, wm: &Prim3, wp: &Prim3, fl: &Face3, fr: &Face3, vol: f64, dt: f64, p: &ImplicitParams) -> (M5, M5, M5) {
    let s = 0.5 * dt / vol;
    let (al, ar) = (fl.area, fr.area);
    let (dis_l, dis_r) = if p.roe {
        let (mut dl, mut dr) = (abs_jacobian3(wm, wk, fl.n, p.eps_fix), abs_jacobian3(wk, wp, fr.n, p.eps_fix));
        let (tl, tr) = (shock_weight(wm[4], wk[4]), shock_weight(wk[4], wp[4]));
        if tl > 0.0 || tr > 0.0 {
            let lam_l = spectral(wk, fl.n).max(spectral(wm, fl.n));
            let lam_r = spectral(wk, fr.n).max(spectral(wp, fr.n));
            for x in dl.iter_mut() {
                *x *= 1.0 - tl;
            }
            for x in dr.iter_mut() {
                *x *= 1.0 - tr;
            }
            for q in 0..5 {
                dl[6 * q] += tl * lam_l;
                dr[6 * q] += tr * lam_r;
            }
        }
        (dl, dr)
    } else {
        let lam_l = spectral(wk, fl.n).max(spectral(wm, fl.n));
        let lam_r = spectral(wk, fr.n).max(spectral(wp, fr.n));
        let (mut dl, mut dr) = (ZERO5, ZERO5);
        for q in 0..5 {
            dl[6 * q] = lam_l;
            dr[6 * q] = lam_r;
        }
        (dl, dr)
    };
    let mut d = EYE5;
    axpy(&mut d, s * ar, &flux_jacobian3(wk, fr.n));
    axpy(&mut d, -s * al, &flux_jacobian3(wk, fl.n));
    axpy(&mut d, s * ar, &dis_r);
    axpy(&mut d, s * al, &dis_l);
    if !p.lines {
        return (ZERO5, d, ZERO5);
    }
    let mut lo = flux_jacobian3(wm, fl.n);
    axpy(&mut lo, 1.0, &dis_l);
    for x in lo.iter_mut() {
        *x *= -s * al;
    }
    let mut up = flux_jacobian3(wp, fr.n);
    axpy(&mut up, -1.0, &dis_r);
    for x in up.iter_mut() {
        *x *= s * ar;
    }
    (lo, d, up)
}

/// Assemble the factor of direction `dir` (0, 1, 2 = i, j, k).
fn assemble3(s: &Solver3, p: &ImplicitParams, dir: usize) -> Factor {
    let (ni, nj, nk) = (s.grid.ni, s.grid.nj, s.grid.nk);
    let (grid, w, dt, solid) = (&s.grid, &s.w, &s.dt, &s.solid);
    let n = ni * nj * nk;
    let stride = [1, ni, ni * nj][dir];
    let e: [isize; 3] = match dir {
        0 => [1, 0, 0],
        1 => [0, 1, 0],
        _ => [0, 0, 1],
    };
    // ghost Jacobian of the lower (`lower`) / upper end of a line
    let ghost_t = |lower: bool, f: &Face3, i: usize| -> M5 {
        if !p.implicit_bc {
            return ZERO5;
        }
        match (dir, lower) {
            // Characteristic far field (i ends, j = nj): frozen ghost, T = 0. The f32 finite-difference Jacobian at the kink of the
            // characteristic branch switch destabilised the GPU left-hand side at CFL ~ 6 (ANF M 2.48 nk 48: a far-field corner cell lost
            // its pressure and the run froze), so CPU and GPU both use the frozen ghost.
            (0, _) | (1, false) => ZERO5,
            (1, true) => {
                if i < grid.axis_cells {
                    ZERO5
                } else {
                    reflection3(f.n)
                }
            }
            _ => reflection3(f.n),
        }
    };
    let mut fac = Factor { lo: vec![ZERO5; n], d: vec![ZERO5; n], up: vec![ZERO5; n] };
    let plane = ni * nj;
    let Factor { lo, d, up } = &mut fac;
    lo.par_chunks_mut(plane).zip(d.par_chunks_mut(plane)).zip(up.par_chunks_mut(plane)).enumerate().for_each(|(k, ((lo_p, d_p), up_p))| {
        for j in 0..nj {
            for i in 0..ni {
                let (c, cl) = ((k * nj + j) * ni + i, j * ni + i);
                if solid[c] {
                    d_p[cl] = EYE5;
                    continue;
                }
                let (ii, jj, kk) = (i as isize, j as isize, k as isize);
                let (m, nl) = [(i, ni), (j, nj), (k, nk)][dir];
                let wk = &w[pidx3(ni, nj, nk, ii, jj, kk)];
                let (fl, fr) = match dir {
                    0 => {
                        let il = (k * nj + j) * (ni + 1) + i;
                        (&grid.iface[il], &grid.iface[il + 1])
                    }
                    1 => {
                        let jl = (k * (nj + 1) + j) * ni + i;
                        (&grid.jface[jl], &grid.jface[jl + ni])
                    }
                    _ => (&grid.kface[c], &grid.kface[c + plane]),
                };
                let lower_solid = m > 0 && solid[c - stride];
                let upper_solid = m + 1 < nl && solid[c + stride];
                let wm = if lower_solid { reflect3(wk, fl.n) } else { w[pidx3(ni, nj, nk, ii - e[0], jj - e[1], kk - e[2])] };
                let wp = if upper_solid { reflect3(wk, fr.n) } else { w[pidx3(ni, nj, nk, ii + e[0], jj + e[1], kk + e[2])] };
                let (mut l, mut dd, mut u) = cell_blocks3(wk, &wm, &wp, fl, fr, grid.vol[c], dt[c], p);
                if m == 0 {
                    let t = ghost_t(true, fl, i);
                    axpy(&mut dd, 1.0, &mm(&l, &t));
                    l = ZERO5;
                } else if lower_solid {
                    axpy(&mut dd, 1.0, &mm(&l, &reflection3(fl.n)));
                    l = ZERO5;
                }
                if m == nl - 1 {
                    let t = ghost_t(false, fr, i);
                    axpy(&mut dd, 1.0, &mm(&u, &t));
                    u = ZERO5;
                } else if upper_solid {
                    axpy(&mut dd, 1.0, &mm(&u, &reflection3(fr.n)));
                    u = ZERO5;
                }
                lo_p[cl] = l;
                d_p[cl] = dd;
                up_p[cl] = u;
            }
        }
    });
    fac
}

/// `x <- (I - dt J_dir)^{-1} x` for every line of direction `dir`.
fn solve_lines3(s: &Solver3, fac: &Factor, dir: usize, x: &mut [Cons3]) {
    let (ni, nj, nk) = (s.grid.ni, s.grid.nj, s.grid.nk);
    let (nl, stride, nlines) = match dir {
        0 => (ni, 1, nj * nk),
        1 => (nj, ni, ni * nk),
        _ => (nk, ni * nj, ni * nj),
    };
    let base_of = |l: usize| match dir {
        0 => l * ni,
        1 => (l / ni) * ni * nj + l % ni,
        _ => l,
    };
    let x_in: &[Cons3] = x;
    let lines: Vec<Vec<Cons3>> = (0..nlines)
        .into_par_iter()
        .map(|l| {
            let base = base_of(l);
            let mut rhs: Vec<Cons3> = (0..nl).map(|m| x_in[base + m * stride]).collect();
            let mut y = vec![ZERO5; nl];
            block_thomas5(nl, base, stride, fac, &mut rhs, &mut y);
            rhs
        })
        .collect();
    for (l, line) in lines.iter().enumerate() {
        let base = base_of(l);
        for (m, v) in line.iter().enumerate() {
            x[base + m * stride] = *v;
        }
    }
}

/// `(I - dt J_dir) x` (matrix-vector product with the stored blocks; solid rows are the identity).
fn apply_factor3(s: &Solver3, fac: &Factor, dir: usize, x: &[Cons3]) -> Vec<Cons3> {
    let (ni, nj, nk) = (s.grid.ni, s.grid.nj, s.grid.nk);
    let stride = [1, ni, ni * nj][dir];
    let mut out = vec![[0.0; 5]; ni * nj * nk];
    out.par_chunks_mut(ni * nj).enumerate().for_each(|(k, pl)| {
        for j in 0..nj {
            for i in 0..ni {
                let c = (k * nj + j) * ni + i;
                let (m, nl) = [(i, ni), (j, nj), (k, nk)][dir];
                let mut v = mv(&fac.d[c], &x[c]);
                if m > 0 {
                    let a = mv(&fac.lo[c], &x[c - stride]);
                    for q in 0..5 {
                        v[q] += a[q];
                    }
                }
                if m + 1 < nl {
                    let a = mv(&fac.up[c], &x[c + stride]);
                    for q in 0..5 {
                        v[q] += a[q];
                    }
                }
                pl[j * ni + i] = v;
            }
        }
    });
    out
}

/// The correction `dU` of the block scheme for the right-hand side `b = dt R`: the three-factor solve with block-tridiagonal lines and
/// `ImplicitParams::sub_iters - 1` defect-correction passes against the unfactored operator.
fn du_block(s: &Solver3, p: &ImplicitParams, b: &[Cons3]) -> Vec<Cons3> {
    let n = s.u.len();
    let facs = [assemble3(s, p, 0), assemble3(s, p, 1), assemble3(s, p, 2)];
    let af = |rhs: &[Cons3]| -> Vec<Cons3> {
        let mut x = rhs.to_vec();
        for dir in 0..3 {
            solve_lines3(s, &facs[dir], dir, &mut x);
        }
        x
    };
    let mut du = af(b);
    for _ in 1..p.sub_iters.max(1) {
        let a: Vec<Vec<Cons3>> = (0..3).map(|dir| apply_factor3(s, &facs[dir], dir, &du)).collect();
        let r: Vec<Cons3> = (0..n)
            .into_par_iter()
            .map(|c| {
                let mut v = [0.0; 5];
                for q in 0..5 {
                    v[q] = b[c][q] - (a[0][c][q] + a[1][c][q] + a[2][c][q] - 2.0 * du[c][q]);
                }
                v
            })
            .collect();
        let corr = af(&r);
        du.par_iter_mut().zip(corr.par_iter()).for_each(|(d, c)| {
            for q in 0..5 {
                d[q] += c[q];
            }
        });
    }
    du
}

/// One implicit iteration; returns the density residual at the start of the iteration.
///
/// `ImplicitParams::sub_iters > 1` repeats the factorised solve as a defect correction against the unfactored operator
/// `I - dt J_i - dt J_j - dt J_k`; `ImplicitParams::line_solve` selects block lines or the diagonalised factors ([`diag3`]).
pub(crate) fn step(s: &mut Solver3, p: &ImplicitParams) -> f64 {
    let cfl = (p.cfl0 * p.growth.powi(s.ramp_k.min(10_000) as i32)).min(p.cfl_max);
    s.ramp_k += 1;
    s.eval_residual();
    let rn = s.density_residual();
    s.compute_dt(cfl, p.dt_mode);

    let b: Vec<Cons3> = s.res.iter().zip(s.dt.iter()).map(|(r, d)| [r[0] * d, r[1] * d, r[2] * d, r[3] * d, r[4] * d]).collect();
    let du = match p.line_solve {
        LineSolve3::Block => du_block(s, p, &b),
        LineSolve3::DiagonalCorrected => diag3::du3(s, p, &b),
    };

    // ---- update with positivity damping ----
    let solid = &s.solid;
    s.u.par_iter_mut().enumerate().for_each(|(c, uc)| {
        if solid[c] {
            return;
        }
        let dq = du[c];
        let mut alpha = 1.0;
        for _ in 0..8 {
            let q: Cons3 = [uc[0] + alpha * dq[0], uc[1] + alpha * dq[1], uc[2] + alpha * dq[2], uc[3] + alpha * dq[3], uc[4] + alpha * dq[4]];
            let wq = cons_to_prim3(&q);
            if wq[0] > 0.0 && wq[4] > 0.0 && q.iter().all(|v| v.is_finite()) {
                *uc = q;
                break;
            }
            alpha *= 0.5;
        }
    });
    rn
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::flux3::tests::Rng;
    use crate::gas3::{physical_flux3, prim_to_cons3};

    fn unit(rng: &mut Rng) -> [f64; 3] {
        let n = [rng.range(-1.0, 1.0), rng.range(-1.0, 1.0), rng.range(-1.0, 1.0)];
        let l = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
        [n[0] / l, n[1] / l, n[2] / l]
    }

    #[test]
    fn flux_jacobian_matches_finite_differences() {
        let mut rng = Rng::new(1);
        for _ in 0..50 {
            let w: Prim3 = [rng.range(0.5, 2.0), rng.range(-1.5, 1.5), rng.range(-1.5, 1.5), rng.range(-1.5, 1.5), rng.range(0.5, 2.0)];
            let n = unit(&mut rng);
            let a = flux_jacobian3(&w, n);
            let u0 = prim_to_cons3(&w);
            for q in 0..5 {
                let h = 1e-6;
                let (mut up, mut um) = (u0, u0);
                up[q] += h;
                um[q] -= h;
                let fp = physical_flux3(&cons_to_prim3(&up), n);
                let fm = physical_flux3(&cons_to_prim3(&um), n);
                for r in 0..5 {
                    let fd = (fp[r] - fm[r]) / (2.0 * h);
                    assert!((fd - a[5 * r + q]).abs() < 1e-7, "A[{r}][{q}] = {} vs fd {fd}", a[5 * r + q]);
                }
            }
        }
    }

    #[test]
    fn abs_jacobian_squares_to_a_squared() {
        // |A|^2 = A^2 (A diagonalisable; no entropy fix active for these eigenvalues)
        let mut rng = Rng::new(2);
        for _ in 0..50 {
            let w: Prim3 = [rng.range(0.5, 2.0), rng.range(0.3, 1.5), rng.range(-1.5, 1.5), rng.range(-1.5, 1.5), rng.range(0.5, 2.0)];
            let n = unit(&mut rng);
            let a = flux_jacobian3(&w, n);
            let ab = abs_jacobian3(&w, &w, n, 1e-3);
            let (a2, b2) = (mm(&a, &a), mm(&ab, &ab));
            for k in 0..25 {
                assert!((a2[k] - b2[k]).abs() < 1e-9 * (1.0 + a2[k].abs()), "entry {k}: {} vs {}", a2[k], b2[k]);
            }
        }
    }

    #[test]
    fn reflection_matrix_reflects_the_state() {
        let mut rng = Rng::new(3);
        let w: Prim3 = [1.2, 0.5, -0.4, 0.9, 0.8];
        let n = unit(&mut rng);
        let got = mv(&reflection3(n), &prim_to_cons3(&w));
        let want = prim_to_cons3(&reflect3(&w, n));
        for q in 0..5 {
            assert!((got[q] - want[q]).abs() < 1e-14);
        }
    }

    #[test]
    fn inv5_inverts() {
        let mut rng = Rng::new(4);
        let mut m = [0.0; 25];
        for k in 0..25 {
            m[k] = rng.range(-1.0, 1.0);
        }
        for q in 0..5 {
            m[6 * q] += 2.0;
        }
        let p = mm(&m, &inv5(&m));
        for k in 0..25 {
            assert!((p[k] - EYE5[k]).abs() < 1e-13, "{k}: {}", p[k]);
        }
    }

    #[test]
    fn block_solver_inverts_a_random_diagonally_dominant_system() {
        let n = 7;
        let mut rng = Rng::new(5);
        let mut blocks = |diag: f64| -> M5 {
            let mut m = ZERO5;
            for k in 0..25 {
                m[k] = 0.3 * rng.range(-0.5, 0.5);
            }
            for q in 0..5 {
                m[6 * q] += diag;
            }
            m
        };
        let fac = Factor { lo: (0..n).map(|_| blocks(0.0)).collect(), d: (0..n).map(|_| blocks(3.0)).collect(), up: (0..n).map(|_| blocks(0.0)).collect() };
        let mut rng = Rng::new(6);
        let rhs: Vec<Cons3> = (0..n).map(|_| [rng.range(-0.5, 0.5), rng.range(-0.5, 0.5), rng.range(-0.5, 0.5), rng.range(-0.5, 0.5), rng.range(-0.5, 0.5)]).collect();
        let mut x = rhs.clone();
        let mut y = vec![ZERO5; n];
        block_thomas5(n, 0, 1, &fac, &mut x, &mut y);
        for k in 0..n {
            let mut r = mv(&fac.d[k], &x[k]);
            if k > 0 {
                let a = mv(&fac.lo[k], &x[k - 1]);
                for q in 0..5 {
                    r[q] += a[q];
                }
            }
            if k + 1 < n {
                let a = mv(&fac.up[k], &x[k + 1]);
                for q in 0..5 {
                    r[q] += a[q];
                }
            }
            for q in 0..5 {
                assert!((r[q] - rhs[k][q]).abs() < 1e-12);
            }
        }
    }
}
