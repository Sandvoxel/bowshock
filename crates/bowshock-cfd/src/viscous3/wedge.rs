//! Test fixture: an axisymmetric 2D grid (optionally with a body block), its one-cell wedge `grid3::revolve(g2, 1, [0, dtheta])` and one smooth axisymmetric
//! state on both, for the parity tests of the 3D kernels against the validated 2D ones.
//!
//! The 2D grid has the axis `r = 0` at `j = 0` in every column; with a block (`wall`) the cells `i < i_end, j < j_end` are the solid body (its surface is the
//! row `j_end`, its base the column `i_end`), without it every column is fluid down to the axis. A state is a function `f(x, r) = (rho, u, v_r, p)` sampled at
//! the centre of every cell, each scheme at its own centre: the 2D area centroid `(xc, rc)` for the 2D grid, `(x, hypot(y, z))` of `Grid3::xc` (the mean of the 8
//! nodes, which differs from the area centroid by O(h^2) on a stretched or skewed cell) for the wedge. The wedge cell holds the radial velocity turned into
//! `(v, w) = v_r (sin, cos)(theta_m)`, `theta_m = dtheta / 2`. Ghost cells: the domain boundaries `i = 0`, `i = ni`, `j = nj` hold `f` at the mirror image of
//! the cell centre in the face centre (the position the viscous kernels give a ghost), the two `k` faces of the wedge are symmetry planes (the cell reflected
//! in them), the axis ghost is the cell with `(v, w)` negated, a wall ghost is `(rho, -u, -v, p)`.
//!
//! Sampling the wedge at the 2D centroids instead would put the cell centre off the line through the `k`-face centroid by O(h^2): the compact `k` gradient
//! `(phi_R - phi_L) / |d|` with `|d| ~ r dtheta` then amplifies the sampling mismatch like `h^2 / dtheta` (measured, see `viscous3::parity`).

use crate::gas::Prim;
use crate::gas3::{reflect3, Prim3};
use crate::grid::Grid;
use crate::grid3::{revolve, Grid3, SolidBox};
use crate::solver::{pidx, Block};
use crate::solver3::{pidx3, NG};
use crate::viscous::{self, Walls};

pub(crate) struct Wedge {
    pub g2: Grid,
    pub blk: Option<Block>,
    pub g3: Grid3,
    /// Per-cell solid mask of the wedge (`ni * nj`).
    pub solid: Vec<bool>,
    pub theta_m: f64,
    /// Padded primitive arrays of the 2D grid and of the wedge (NaN in the solid cells that are not ghosts).
    pub w2: Vec<Prim>,
    pub w3: Vec<Prim3>,
}

/// Axial position of node column `i` of the grid of `ni` columns.
pub(crate) fn col_x(i: usize, ni: usize) -> f64 {
    let t = i as f64 / ni as f64;
    -0.4 + 2.4 * (t + 0.06 * (std::f64::consts::PI * 2.0 * t).sin())
}

/// Body surface radius `r_b(x)` of the grid: a cylinder, so that the vertical grid lines meet the wall at a right angle (see the `wall_tilt` test of `viscous3`
/// for the tilted wall).
pub(crate) fn body_radius(_x: f64) -> f64 {
    0.3
}

/// Smooth axisymmetric `(rho, u, v_r, p)`, even in `r` except `v_r`, odd (so it is a smooth field across the axis). With a body (`wall`) the velocity is
/// multiplied by the bounded factor `eta = 2 (r^2 - r_b(x)^2) / (1 + r^2) * (x - x_base) / sqrt(1 + (x - x_base)^2)`, which vanishes linearly on the body surface and
/// on the base plane (the no-slip wall ghost `(-u, -v)` is then the second-order continuation of the state), and the variations of `rho` and `p` by `eta^2`,
/// which makes the walls adiabatic and the wall ghost `(rho, p)` a second-order continuation as well.
pub(crate) fn smooth_state(wall: Option<f64>) -> impl Fn(f64, f64) -> Prim {
    move |x, r| {
        let eta = match wall {
            Some(x_base) => {
                let (dx, ds) = (x - x_base, r * r - body_radius(x).powi(2));
                2.0 * ds / (1.0 + r * r) * dx / (1.0 + dx * dx).sqrt()
            }
            None => 1.0,
        };
        let rho = 1.0 + 0.15 * (1.3 * x).sin() * (2.0 * r).cos() * eta * eta;
        let p = 0.7 + 0.1 * x.cos() * r.cos() * eta * eta;
        let u = (0.8 + 0.25 * (1.7 * x).sin() * (0.9 * r).cos() + 0.1 * r * r) * eta;
        let v = r.sin() * (0.3 * x.cos() + 0.1 * r.cos()) * eta;
        [rho, u, v, p]
    }
}

/// The 2D grid: `ni x nj`, columns stretched and skewed, rows clustered towards the body surface; `i_end = ni / 2`, `j_end = nj / 3` with a body.
fn grid2d(ni: usize, nj: usize, wall: bool) -> (Grid, Option<Block>) {
    let (i_end, j_end) = (ni / 2, nj / 3);
    let blk = wall.then_some(Block { i_end, j_end, j_sting: 0 });
    let j0 = blk.map_or(0, |b| b.j_end);
    let mut x = vec![0.0; (ni + 1) * (nj + 1)];
    let mut r = vec![0.0; (ni + 1) * (nj + 1)];
    for i in 0..=ni {
        let xi = col_x(i, ni);
        let r_body = if j0 == 0 { 0.0 } else { body_radius(xi) };
        let r_top = 1.4 + 0.1 * xi;
        for j in 0..=nj {
            let (rr, skew) = if j <= j0 {
                (if j0 == 0 { 0.0 } else { r_body * j as f64 / j0 as f64 }, 0.0)
            } else {
                let s = (j - j0) as f64 / (nj - j0) as f64;
                let a = 1.2;
                (r_body + (r_top - r_body) * ((a * s).exp() - 1.0) / (a.exp() - 1.0), 0.04 * (std::f64::consts::PI * s).sin().powi(2))
            };
            x[j * (ni + 1) + i] = xi + skew;
            r[j * (ni + 1) + i] = rr;
        }
    }
    (Grid::from_nodes(ni, nj, x, r), blk)
}

/// Build the 2D grid, the wedge of opening angle `dtheta` and the state `f` on both. `wall`: with a body block. `matched`: the node radii of the wedge are
/// divided by `cos(dtheta / 2)` so that its flat faces cross the mid-plane at the radii of the 2D grid (the plain `revolve` has its flat faces, a body wall
/// among them, `O(dtheta^2)` inside the 2D surface: a geometric chord effect that the state of the 2D grid, zero on the body surface, would see as an
/// `O(dtheta^2 / h)` change of the wall shear).
pub(crate) fn wedge(ni: usize, nj: usize, wall: bool, dtheta: f64, matched: bool, f: &dyn Fn(f64, f64) -> Prim) -> Wedge {
    let (g2, blk) = grid2d(ni, nj, wall);
    let mut g3 = revolve(&g2, 1, &[0.0, dtheta]).unwrap();
    if matched {
        let inv = 1.0 / (0.5 * dtheta).cos();
        g3 = Grid3::try_from_nodes(ni, nj, 1, g3.x.clone(), g3.y.iter().map(|y| y * inv).collect(), g3.z.iter().map(|z| z * inv).collect()).unwrap();
    }
    if let Some(b) = blk {
        g3.solid = vec![SolidBox { i: 0..b.i_end, j: 0..b.j_end, k: 0..1 }];
    }
    let theta_m = 0.5 * dtheta;
    let solid = super::tests::solid_mask(&g3);
    let fluid = |i: usize, j: usize| !blk.is_some_and(|b| b.solid(i, j));
    let to3 = |w: &Prim, th: f64| -> Prim3 {
        let (s, co) = th.sin_cos();
        [w[0], w[1], w[2] * s, w[2] * co, w[3]]
    };
    let cells2: Vec<Prim> = (0..ni * nj).map(|c| f(g2.xc[c], g2.rc[c])).collect();
    let cells3: Vec<Prim3> = (0..ni * nj).map(|c| to3(&f(g3.xc[c][0], g3.xc[c][1].hypot(g3.xc[c][2])), theta_m)).collect();

    // 3D: padded array, then the first ghost layer of the domain faces from the state at the mirror image of the cell centre in the plane of the face, the
    // symmetry-plane ghosts in k and the axis ghosts in j
    let mut w3 = super::tests::padded(&g3, &solid, &cells3);
    let ghost3 = |c: usize, face: &crate::grid3::Face3| -> Prim3 {
        let x = g3.xc[c];
        let h = (x[0] - face.c[0]) * face.n[0] + (x[1] - face.c[1]) * face.n[1] + (x[2] - face.c[2]) * face.n[2];
        let p = [x[0] - 2.0 * h * face.n[0], x[1] - 2.0 * h * face.n[1], x[2] - 2.0 * h * face.n[2]];
        to3(&f(p[0], p[1].hypot(p[2])), p[1].atan2(p[2]))
    };
    let id3 = |i: isize, j: isize, k: isize| pidx3(ni, nj, 1, i, j, k);
    for j in 0..nj {
        for i in 0..ni {
            if !fluid(i, j) {
                continue;
            }
            let (ii, jj) = (i as isize, j as isize);
            let c = g3.cell(i, j, 0);
            let cell = w3[id3(ii, jj, 0)];
            let (nlo, nhi) = (g3.kface[c].n, g3.kface[c + ni * nj].n);
            for m in 1..=NG as isize {
                w3[id3(ii, jj, -m)] = reflect3(&cell, nlo);
                w3[id3(ii, jj, m)] = reflect3(&cell, nhi);
            }
            let mut set = |gi: isize, gj: isize, w: Prim3| {
                for m in 0..NG as isize {
                    let (di, dj) = ((gi - ii).signum() * m, (gj - jj).signum() * m);
                    w3[id3(gi + di, gj + dj, 0)] = w;
                }
            };
            if i == 0 {
                set(-1, jj, ghost3(c, &g3.iface[j * (ni + 1)]));
            }
            if i == ni - 1 {
                set(ni as isize, jj, ghost3(c, &g3.iface[j * (ni + 1) + ni]));
            }
            if j == nj - 1 {
                set(ii, nj as isize, ghost3(c, &g3.jface[nj * ni + i]));
            }
            if j == 0 {
                set(ii, -1, [cell[0], cell[1], -cell[2], -cell[3], cell[4]]);
            }
        }
    }

    // 2D: the same ghosts, the block ghost rows below the body surface
    let mut w2 = vec![[f64::NAN; 4]; (ni + 2 * NG) * (nj + 2 * NG)];
    let ng = NG as isize;
    for j in -ng..nj as isize + ng {
        for i in -ng..ni as isize + ng {
            let (ci, cj) = (i.clamp(0, ni as isize - 1) as usize, j.clamp(0, nj as isize - 1) as usize);
            if fluid(ci, cj) {
                w2[pidx(ni, i, j)] = cells2[cj * ni + ci];
            }
        }
    }
    let ghost2 = |c: usize, (xf, rf): (f64, f64)| -> Prim { f(2.0 * xf - g2.xc[c], 2.0 * rf - g2.rc[c]) };
    for j in 0..nj {
        let c = j * ni;
        if fluid(0, j) {
            let (a, b) = (g2.node(0, j), g2.node(0, j + 1));
            let w = ghost2(c, (0.5 * (a.0 + b.0), 0.5 * (a.1 + b.1)));
            for m in 1..=NG as isize {
                w2[pidx(ni, -m, j as isize)] = w;
            }
        }
        let c = j * ni + ni - 1;
        let (a, b) = (g2.node(ni, j), g2.node(ni, j + 1));
        let w = ghost2(c, (0.5 * (a.0 + b.0), 0.5 * (a.1 + b.1)));
        for m in 0..NG as isize {
            w2[pidx(ni, ni as isize + m, j as isize)] = w;
        }
    }
    for i in 0..ni {
        let c = (nj - 1) * ni + i;
        let (a, b) = (g2.node(i, nj), g2.node(i + 1, nj));
        let w = ghost2(c, (0.5 * (a.0 + b.0), 0.5 * (a.1 + b.1)));
        for m in 0..NG as isize {
            w2[pidx(ni, i as isize, nj as isize + m)] = w;
        }
        match blk {
            Some(b) if i < b.i_end => {
                for m in 1..=NG {
                    let src = w2[pidx(ni, i as isize, (b.j_end + m - 1) as isize)];
                    w2[pidx(ni, i as isize, b.j_end as isize - m as isize)] = viscous::wall_ghost(&src, &g2.jface[b.j_end * ni + i], true);
                }
            }
            _ => {
                for m in 1..=NG as isize {
                    let src = w2[pidx(ni, i as isize, 0)];
                    w2[pidx(ni, i as isize, -m)] = [src[0], src[1], -src[2], src[3]];
                }
            }
        }
    }
    Wedge { g2, blk, g3, solid, theta_m, w2, w3 }
}

/// Viscous part of the 2D residual `dU/dt` (`[mass, x, r, energy]` per cell, zero in solid cells) with the hoop-stress source, from `viscous::cell_pass`,
/// `face_pass` and `hoop_stress` on the state of `wd`, laminar with the Reynolds number `re_c` of `super::tests::VISC`.
pub(crate) fn viscous_residual_2d(wd: &Wedge, mu_t: &[f64]) -> Vec<[f64; 4]> {
    let g = &wd.g2;
    let (ni, nj) = (g.ni, g.nj);
    let walls = Walls { visc: Some(super::tests::VISC), jmin_slip: false, blk: wd.blk, nozzle: None };
    let (mut grad, mut mu) = (vec![[0.0; 6]; ni * nj], vec![0.0; ni * nj]);
    viscous::cell_pass(&super::tests::VISC, g, &wd.w2, &walls, &mut grad, &mut mu);
    let (mut fvi, mut fvj) = (vec![[0.0; 4]; (ni + 1) * nj], vec![[0.0; 4]; ni * (nj + 1)]);
    viscous::face_pass(g, &wd.w2, &walls, &grad, &mu, mu_t, &[], &mut fvi, &mut fvj);
    let mut res = vec![[0.0; 4]; ni * nj];
    for j in 0..nj {
        for i in 0..ni {
            if wd.blk.is_some_and(|b| b.solid(i, j)) {
                continue;
            }
            let c = j * ni + i;
            let (fl, fr, fb, ft) = (&g.iface[j * (ni + 1) + i], &g.iface[j * (ni + 1) + i + 1], &g.jface[j * ni + i], &g.jface[(j + 1) * ni + i]);
            let (hl, hr, hb, ht) = (&fvi[j * (ni + 1) + i], &fvi[j * (ni + 1) + i + 1], &fvj[j * ni + i], &fvj[(j + 1) * ni + i]);
            let inv_v = 1.0 / g.vol[c];
            let wc = &wd.w2[pidx(ni, i as isize, j as isize)];
            let mut out = [0.0; 4];
            for k in 0..4 {
                out[k] = (hr[k] * fr.w() - hl[k] * fl.w() + ht[k] * ft.w() - hb[k] * fb.w()) * inv_v;
            }
            out[2] -= viscous::hoop_stress(mu[c] + mu_t[c], &grad[c], wc[2], g.rc[c]) * g.area[c] * inv_v;
            res[c] = out;
        }
    }
    res
}

/// Fourth-order central derivative of `g` along coordinate `axis` (0: x, 1: r) at `(x, r)`, step `h`.
fn d(g: &dyn Fn(f64, f64) -> f64, axis: usize, x: f64, r: f64, h: f64) -> f64 {
    let at = |s: f64| if axis == 0 { g(x + s, r) } else { g(x, r + s) };
    (-at(2.0 * h) + 8.0 * at(h) - 8.0 * at(-h) + at(-2.0 * h)) / (12.0 * h)
}

/// The viscous terms of the axisymmetric Navier-Stokes equations of the smooth state `f` at `(x, r)` by high-order differentiation (the continuous operator the
/// two discretisations approximate): `[x-momentum, radial momentum, energy]` per unit volume.
///
/// `R_x = d_x t_xx + (1/r) d_r (r t_xr)`, `R_r = d_x t_xr + (1/r) d_r (r t_rr) - t_tt / r`,
/// `R_E = d_x (u t_xx + v t_xr + k T_x) + (1/r) d_r (r (u t_xr + v t_rr + k T_r))`, `div = u_x + v_r + v / r`, `t_tt = mu (2 v / r - 2/3 div)`.
pub(crate) fn analytic_viscous_residual(f: &dyn Fn(f64, f64) -> Prim, x: f64, r: f64) -> [f64; 3] {
    use crate::gas::GAMMA;
    use crate::solver::PRANDTL;
    let h = 1e-3;
    let vis = super::tests::VISC;
    let u = |x: f64, r: f64| f(x, r)[1];
    let v = |x: f64, r: f64| f(x, r)[2];
    let t = |x: f64, r: f64| {
        let w = f(x, r);
        GAMMA * w[3] / w[0]
    };
    let mu = |x: f64, r: f64| vis.mu(t(x, r));
    let div = |x: f64, r: f64| d(&u, 0, x, r, h) + d(&v, 1, x, r, h) + v(x, r) / r;
    let txx = |x: f64, r: f64| mu(x, r) * (2.0 * d(&u, 0, x, r, h) - 2.0 / 3.0 * div(x, r));
    let trr = |x: f64, r: f64| mu(x, r) * (2.0 * d(&v, 1, x, r, h) - 2.0 / 3.0 * div(x, r));
    let txr = |x: f64, r: f64| mu(x, r) * (d(&u, 1, x, r, h) + d(&v, 0, x, r, h));
    let ttt = |x: f64, r: f64| mu(x, r) * (2.0 * v(x, r) / r - 2.0 / 3.0 * div(x, r));
    let kappa = |x: f64, r: f64| mu(x, r) / PRANDTL / (GAMMA - 1.0);
    let rxr = |x: f64, r: f64| r * txr(x, r);
    let rrr = |x: f64, r: f64| r * trr(x, r);
    let ex = |x: f64, r: f64| u(x, r) * txx(x, r) + v(x, r) * txr(x, r) + kappa(x, r) * d(&t, 0, x, r, h);
    let er = |x: f64, r: f64| r * (u(x, r) * txr(x, r) + v(x, r) * trr(x, r) + kappa(x, r) * d(&t, 1, x, r, h));
    let h2 = 1e-3;
    [
        d(&txx, 0, x, r, h2) + d(&rxr, 1, x, r, h2) / r,
        d(&txr, 0, x, r, h2) + d(&rrr, 1, x, r, h2) / r - ttt(x, r) / r,
        d(&ex, 0, x, r, h2) + d(&er, 1, x, r, h2) / r,
    ]
}
