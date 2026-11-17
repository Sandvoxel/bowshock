//! The diagonalised approximate factorisation of the 3D implicit step ([`LineSolve3::DiagonalCorrected`]).
//!
//! The directional factor `I - dt J_d` of the block scheme is replaced per cell by `T_k (I + dt delta_d Lambda_d) T_k^{-1}`, `T_k` the
//! eigenvectors of the flux Jacobian `A(w_k, n_k)` of the cell (`n_k` the normalised mean of its two face normals): `y = T_k^{-1} rhs`, one scalar
//! tridiagonal solve per characteristic field along the line, `x = T_k y`. The scalar coefficients are the characteristic-field values of the block
//! coefficients (`lo = -s w (A_m + D)`, ...) with the face dissipation `D` taken eigenvalue by eigenvalue. The line ends and the neighbours of
//! solid cells drop the coupling to the ghost (frozen). One factorised solve is only approximately the block one (the dropped ghost coupling, the
//! cell-wise frozen eigenvectors), so [`du3`] corrects it once against the unfactored BLOCK operator: `dU0 = D^-1 b`,
//! `dU = dU0 + D^-1 (b - (I - dt J_i - dt J_j - dt J_k) dU0)`.

use super::*;

/// Row-major `N x N` matrix.
type Mat<const N: usize> = [[f64; N]; N];

/// The diagonal factor of one cell: scalar tridiagonal coefficients of every characteristic field and the eigenvector matrices
/// (`r` the right eigenvectors as columns, `r[row][field]`; `l` the left eigenvectors as rows, `l[field][col]`).
#[derive(Clone, Copy)]
struct DCell<const N: usize> {
    lo: [f64; N],
    d: [f64; N],
    up: [f64; N],
    r: Mat<N>,
    l: Mat<N>,
}

fn identity<const N: usize>() -> Mat<N> {
    let mut m = [[0.0; N]; N];
    for q in 0..N {
        m[q][q] = 1.0;
    }
    m
}

impl<const N: usize> DCell<N> {
    /// The identity row of a solid cell.
    fn unit() -> Self {
        DCell { lo: [0.0; N], d: [1.0; N], up: [0.0; N], r: identity(), l: identity() }
    }
}

/// Scalar coefficients of every field from the block formulas: `lm, lp` eigenvalues of `A` of the lower / upper neighbour at the
/// lower / upper face normal, `lkl, lkr` of the cell itself at the lower / upper face, `dl, dr` face dissipation eigenvalues.
fn coeffs<const N: usize>(
    s: f64,
    wl: f64,
    wr: f64,
    lm: &[f64; N],
    lkl: &[f64; N],
    lkr: &[f64; N],
    lp: &[f64; N],
    dl: &[f64; N],
    dr: &[f64; N],
) -> ([f64; N], [f64; N], [f64; N]) {
    let (mut lo, mut d, mut up) = ([0.0; N], [0.0; N], [0.0; N]);
    for q in 0..N {
        lo[q] = -s * wl * (lm[q] + dl[q]);
        d[q] = 1.0 + s * wr * (lkr[q] + dr[q]) - s * wl * (lkl[q] - dl[q]);
        up[q] = s * wr * (lp[q] - dr[q]);
    }
    (lo, d, up)
}

/// Line end: the coupling to the ghost (or to a solid neighbour) is dropped.
fn end_drop<const N: usize>(cell: &mut DCell<N>, lower: bool) {
    if lower {
        cell.lo = [0.0; N];
    } else {
        cell.up = [0.0; N];
    }
}

/// Eigenvalues `u_n` (three times), `u_n + c`, `u_n - c` of `A(w, n)`.
fn lam3(w: &Prim3, n: [f64; 3]) -> [f64; 5] {
    let th = w[1] * n[0] + w[2] * n[1] + w[3] * n[2];
    let c = sound_speed3(w);
    [th, th, th, th + c, th - c]
}

/// Two unit tangents orthogonal to the unit normal `n` (and to each other).
fn tangents(n: [f64; 3]) -> ([f64; 3], [f64; 3]) {
    // the choice of the pair matters (the scalar systems couple cells in their own bases), so ties must not depend on the rounding: preference
    // to the lower axis within `TIE_TOL` (the GPU solver uses the same rule)
    const TIE_TOL: f64 = 1e-4;
    let a = if n[0].abs() <= n[1].abs() + TIE_TOL && n[0].abs() <= n[2].abs() + TIE_TOL {
        0
    } else if n[1].abs() <= n[2].abs() + TIE_TOL {
        1
    } else {
        2
    };
    let mut e = [0.0; 3];
    e[a] = 1.0;
    // t1 = e x n, normalised
    let mut t1 = [e[1] * n[2] - e[2] * n[1], e[2] * n[0] - e[0] * n[2], e[0] * n[1] - e[1] * n[0]];
    let l = (t1[0] * t1[0] + t1[1] * t1[1] + t1[2] * t1[2]).sqrt();
    for x in t1.iter_mut() {
        *x /= l;
    }
    let t2 = [n[1] * t1[2] - n[2] * t1[1], n[2] * t1[0] - n[0] * t1[2], n[0] * t1[1] - n[1] * t1[0]];
    (t1, t2)
}

/// `(R, L)` of `A(w, n)` in conservative variables: `A = R diag(lam3) L`.
fn eig3(w: &Prim3, n: [f64; 3]) -> (Mat<5>, Mat<5>) {
    let [_rho, u, v, ww, _p] = *w;
    let g1 = GAMMA - 1.0;
    let c = sound_speed3(w);
    let k = 0.5 * (u * u + v * v + ww * ww);
    let h = c * c / g1 + k;
    let th = u * n[0] + v * n[1] + ww * n[2];
    let b2 = g1 / (c * c);
    let b1 = b2 * k;
    let (t1, t2) = tangents(n);
    let vt1 = u * t1[0] + v * t1[1] + ww * t1[2];
    let vt2 = u * t2[0] + v * t2[1] + ww * t2[2];
    let r = [
        [1.0, 0.0, 0.0, 1.0, 1.0],
        [u, t1[0], t2[0], u + c * n[0], u - c * n[0]],
        [v, t1[1], t2[1], v + c * n[1], v - c * n[1]],
        [ww, t1[2], t2[2], ww + c * n[2], ww - c * n[2]],
        [k, vt1, vt2, h + c * th, h - c * th],
    ];
    let l = [
        [1.0 - b1, b2 * u, b2 * v, b2 * ww, -b2],
        [-vt1, t1[0], t1[1], t1[2], 0.0],
        [-vt2, t2[0], t2[1], t2[2], 0.0],
        [0.5 * (b1 - th / c), 0.5 * (-b2 * u + n[0] / c), 0.5 * (-b2 * v + n[1] / c), 0.5 * (-b2 * ww + n[2] / c), 0.5 * b2],
        [0.5 * (b1 + th / c), 0.5 * (-b2 * u - n[0] / c), 0.5 * (-b2 * v - n[1] / c), 0.5 * (-b2 * ww - n[2] / c), 0.5 * b2],
    ];
    (r, l)
}

/// Roe-averaged entropy-fixed `|u_n|, |u_n + c|, |u_n - c|` (the eigenvalues of `abs_jacobian3`).
fn roe_eigs3(wl: &Prim3, wr: &Prim3, n: [f64; 3], eps_fix: f64) -> (f64, f64, f64) {
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
    (fix(a), fix(a + c), fix(a - c))
}

/// Dissipation eigenvalues of the face between `wa` and `wb` (Roe-type with the shock blend towards Rusanov, or Rusanov).
fn face_dis3(wa: &Prim3, wb: &Prim3, n: [f64; 3], p: &ImplicitParams) -> [f64; 5] {
    let lamf = spectral(wa, n).max(spectral(wb, n));
    let (e1, e2, e3) = if p.roe {
        let (e1, e2, e3) = roe_eigs3(wa, wb, n, p.eps_fix);
        let t = shock_weight(wa[4], wb[4]);
        ((1.0 - t) * e1 + t * lamf, (1.0 - t) * e2 + t * lamf, (1.0 - t) * e3 + t * lamf)
    } else {
        (lamf, lamf, lamf)
    };
    [e1, e1, e1, e2, e3]
}

/// Diagonal factor of direction `dir`; mirrors `assemble3` (solid cells the identity row, no ghost coupling).
fn assemble_d3(s: &Solver3, p: &ImplicitParams, dir: usize) -> Vec<DCell<5>> {
    let (ni, nj, nk) = (s.grid.ni, s.grid.nj, s.grid.nk);
    let (grid, w, dt, solid) = (&s.grid, &s.w, &s.dt, &s.solid);
    let n = ni * nj * nk;
    let stride = [1, ni, ni * nj][dir];
    let e: [isize; 3] = match dir {
        0 => [1, 0, 0],
        1 => [0, 1, 0],
        _ => [0, 0, 1],
    };
    let mut out = vec![DCell::<5>::unit(); n];
    let plane = ni * nj;
    out.par_chunks_mut(plane).enumerate().for_each(|(k, pl)| {
        for j in 0..nj {
            for i in 0..ni {
                let (c, cl) = ((k * nj + j) * ni + i, j * ni + i);
                if solid[c] {
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
                // cell normal: normalised mean of the two face normals
                let mut nc = [fl.n[0] + fr.n[0], fl.n[1] + fr.n[1], fl.n[2] + fr.n[2]];
                let nn = (nc[0] * nc[0] + nc[1] * nc[1] + nc[2] * nc[2]).sqrt();
                if nn > 1e-12 {
                    for x in nc.iter_mut() {
                        *x /= nn;
                    }
                } else {
                    nc = fl.n;
                }
                let (r, l) = eig3(wk, nc);
                let s0 = 0.5 * dt[c] / grid.vol[c];
                let (dl, dr) = (face_dis3(&wm, wk, fl.n, p), face_dis3(wk, &wp, fr.n, p));
                let (mut lo, d, mut up) = coeffs(s0, fl.area, fr.area, &lam3(&wm, fl.n), &lam3(wk, fl.n), &lam3(wk, fr.n), &lam3(&wp, fr.n), &dl, &dr);
                if !p.lines {
                    lo = [0.0; 5];
                    up = [0.0; 5];
                }
                let mut cell = DCell { lo, d, up, r, l };
                // line ends (far field, symmetry plane, axis) and the neighbours of solid cells: coupling dropped
                if m == 0 || lower_solid {
                    end_drop(&mut cell, true);
                }
                if m == nl - 1 || upper_solid {
                    end_drop(&mut cell, false);
                }
                pl[cl] = cell;
            }
        }
    });
    out
}

/// Scalar solve of one line of `n` cells; `idx(m)` is the storage index of cell `m`; `rhs` is overwritten by the solution.
fn solve_line<const N: usize>(n: usize, idx: &dyn Fn(usize) -> usize, fac: &[DCell<N>], rhs: &mut [[f64; N]]) {
    let mut cp = vec![[0.0; N]; n];
    for m in 0..n {
        let f = &fac[idx(m)];
        let mut y = [0.0; N];
        for q in 0..N {
            for a in 0..N {
                y[q] += f.l[q][a] * rhs[m][a];
            }
        }
        for q in 0..N {
            let (mut den, mut num) = (f.d[q], y[q]);
            if m > 0 {
                den -= f.lo[q] * cp[m - 1][q];
                num -= f.lo[q] * rhs[m - 1][q];
            }
            cp[m][q] = f.up[q] / den;
            y[q] = num / den;
        }
        rhs[m] = y;
    }
    for m in (0..n.saturating_sub(1)).rev() {
        for q in 0..N {
            rhs[m][q] -= cp[m][q] * rhs[m + 1][q];
        }
    }
    for m in 0..n {
        let f = &fac[idx(m)];
        let y = rhs[m];
        let mut x = [0.0; N];
        for r in 0..N {
            for q in 0..N {
                x[r] += f.r[r][q] * y[q];
            }
        }
        rhs[m] = x;
    }
}

/// One directional diagonal sweep over `nlines` lines of `nl` cells (segment start `k0_of(line)`, first cell at
/// `base_of(line)`, `stride` between cells).
fn solve_dir<const N: usize>(
    fac: &[DCell<N>],
    x: &mut [[f64; N]],
    nlines: usize,
    nl: usize,
    stride: usize,
    base_of: &(dyn Fn(usize) -> usize + Sync),
    k0_of: &(dyn Fn(usize) -> usize + Sync),
) {
    let x_in: &[[f64; N]] = x;
    let lines: Vec<(usize, usize, Vec<[f64; N]>)> = (0..nlines)
        .into_par_iter()
        .map(|l| {
            let (base, k0) = (base_of(l), k0_of(l));
            let n = nl - k0;
            let idx = |m: usize| base + (k0 + m) * stride;
            let mut rhs: Vec<[f64; N]> = (0..n).map(|m| x_in[idx(m)]).collect();
            if n > 0 {
                solve_line(n, &idx, fac, &mut rhs);
            }
            (base, k0, rhs)
        })
        .collect();
    for (base, k0, line) in lines {
        for (m, v) in line.into_iter().enumerate() {
            x[base + (k0 + m) * stride] = v;
        }
    }
}

fn solve_d3(s: &Solver3, fac: &[DCell<5>], dir: usize, x: &mut [Cons3]) {
    let (ni, nj, nk) = (s.grid.ni, s.grid.nj, s.grid.nk);
    let (nl, stride, nlines) = match dir {
        0 => (ni, 1, nj * nk),
        1 => (nj, ni, ni * nk),
        _ => (nk, ni * nj, ni * nj),
    };
    let base_of = move |l: usize| match dir {
        0 => l * ni,
        1 => (l / ni) * ni * nj + l % ni,
        _ => l,
    };
    solve_dir(fac, x, nlines, nl, stride, &base_of, &|_| 0);
}

/// The correction `du` of one diagonal step for the right-hand side `b = dt R`: the diagonal solve `dU0 = D^-1 b` and one defect-correction pass
/// against the unfactored block operator `I - dt J_i - dt J_j - dt J_k` (`dU = dU0 + D^-1 (b - (I - dt J_i - dt J_j - dt J_k) dU0)`).
pub(crate) fn du3(s: &Solver3, p: &ImplicitParams, b: &[Cons3]) -> Vec<Cons3> {
    let n = s.u.len();
    let facs = [assemble_d3(s, p, 0), assemble_d3(s, p, 1), assemble_d3(s, p, 2)];
    let af = |rhs: &[Cons3]| -> Vec<Cons3> {
        let mut x = rhs.to_vec();
        for dir in 0..3 {
            solve_d3(s, &facs[dir], dir, &mut x);
        }
        x
    };
    let mut du = af(b);
    let bf = [assemble3(s, p, 0), assemble3(s, p, 1), assemble3(s, p, 2)];
    let a: Vec<Vec<Cons3>> = (0..3).map(|dir| apply_factor3(s, &bf[dir], dir, &du)).collect();
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
    du
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn eig3_decomposes_the_flux_jacobian() {
        for (w, n) in [
            ([1.2, 0.7, -0.3, 0.4, 0.9], [0.6, 0.0, 0.8]),
            ([0.5, -2.0, 1.1, 0.3, 3.0], [0.0, 1.0, 0.0]),
            ([2.0, 0.0, 0.0, 0.0, 1.0], [1.0, 0.0, 0.0]),
            ([1.0, 0.3, 0.2, -0.5, 1.4], [0.48, -0.6, 0.64]),
        ] {
            let (r, l) = eig3(&w, n);
            let lam = lam3(&w, n);
            let a = flux_jacobian3(&w, n);
            for i in 0..5 {
                for j in 0..5 {
                    let tt: f64 = (0..5).map(|q| r[i][q] * l[q][j]).sum();
                    assert!((tt - if i == j { 1.0 } else { 0.0 }).abs() < 1e-12, "T T^-1 [{i}][{j}] = {tt}");
                    let ta: f64 = (0..5).map(|q| r[i][q] * lam[q] * l[q][j]).sum();
                    assert!((ta - a[5 * i + j]).abs() < 1e-12, "T L T^-1 [{i}][{j}] = {ta} vs {}", a[5 * i + j]);
                }
            }
        }
    }
}
