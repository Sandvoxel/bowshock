//! 3D Euler finite-volume solver on a [`Grid3`] (CPU, f64, `rayon`): MUSCL + HLLC/Rusanov face fluxes of [`crate::flux3`], SSP-RK3
//! with local time stepping (`Scheme3::Explicit`) or the three-factor line-implicit scheme of [`crate::implicit3`].
//!
//! The data flow is the 2D one of [`crate::solver`] with one more direction and no source term:
//! 1. `cons -> prim` per cell into the padded array `w` (2 ghost layers on every side of all three axes, [`pidx3`]);
//! 2. ghost fill (no corner ghosts: all stencils are 1D);
//! 3. flux per i / j / k face (`face_flux_lm3`; the lateral velocity is limited in the face's cylindrical frame, [`face_rot`]);
//! 4. cell residual `R = -(1/V) sum_f S_f F_f`.
//!
//! Boundaries (see [`Solver3::new`] for the grid conventions): `i = 0`, `i = ni`, `j = nj` characteristic far field;
//! `j = 0, i < axis_cells` the symmetry axis (ghost = the cell across the axis, found through the pitch-plane symmetry
//! `theta -> pi - theta` and a sign flip of `v`); `j = 0, i >= axis_cells` slip wall; `k = 0`, `k = nk` symmetry planes.
//! Cells inside a [`crate::grid3::SolidBox`] (fins) are never updated and act as slip walls on their fluid neighbours.

use crate::bc3::{characteristic_ghost3, pressure_outflow_ghost3};
use crate::flux::Limiter;
use crate::flux3::{face_flux_lm3, face_rot};
use crate::gas3::{cons_to_prim3, prim_to_cons3, reflect3, sound_speed3, Cons3, Prim3};
use crate::grid3::{Face3, Grid3};
use crate::implicit::{DtMode, ImplicitParams};
use crate::lowmach5::{spectral_pc3, LowMach};
use crate::viscous3::{diffusion_rates, wall_ghost3};
use rayon::prelude::*;

pub(crate) mod rans;
pub use rans::RansOpts3;

/// Ghost layers per side.
pub(crate) const NG: usize = 2;

/// Index of cell `(i, j, k)` (ghost cells have negative / out-of-range indices) in the padded primitive array.
#[inline]
pub(crate) fn pidx3(ni: usize, nj: usize, _nk: usize, i: isize, j: isize, k: isize) -> usize {
    let ng = NG as isize;
    ((k + ng) as usize * (nj + 2 * NG) + (j + ng) as usize) * (ni + 2 * NG) + (i + ng) as usize
}

/// Convergence acceleration.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Scheme3 {
    /// SSP-RK3 + local time stepping.
    Explicit,
    /// Three-factor approximate-factorization pseudo-time stepping with a CFL ramp (`implicit3.rs`).
    Implicit(ImplicitParams),
}

#[derive(Clone, Debug)]
pub struct Config3 {
    pub freestream: Prim3,
    /// CFL of the explicit scheme (the implicit scheme ramps its own, see [`ImplicitParams`]).
    pub cfl: f64,
    pub limiter: Limiter,
    /// Number of initial iterations run with first-order reconstruction.
    pub first_order_iters: usize,
    pub low_mach: bool,
    pub scheme: Scheme3,
}

pub struct Solver3 {
    pub grid: Grid3,
    pub cfg: Config3,
    /// Conserved variables, `ni*nj*nk`, index `Grid3::cell`.
    pub u: Vec<Cons3>,
    /// Primitive variables with `NG` ghost layers, `(ni+4)*(nj+4)*(nk+4)`, index [`pidx3`].
    pub w: Vec<Prim3>,
    pub res: Vec<Cons3>,
    pub dt: Vec<f64>,
    pub iter: usize,
    /// Face fluxes `F.n` of the three face families (layouts of `Grid3::{iface, jface, kface}`).
    pub(crate) fi: Vec<Cons3>,
    pub(crate) fj: Vec<Cons3>,
    pub(crate) fk: Vec<Cons3>,
    u0: Vec<Cons3>,
    /// Per cell: inside a solid box.
    pub(crate) solid: Vec<bool>,
    pub(crate) n_active: usize,
    /// Iterations since the start of the implicit CFL ramp.
    pub(crate) ramp_k: usize,
    /// Axis cells: `axis_k[i * nk + k]` is the cell index `k'` across the axis of cell `(i, 0, k)`.
    axis_k: Vec<usize>,
    /// `face_rot` of every face of the three families.
    rot_i: Vec<[f64; 2]>,
    rot_j: Vec<[f64; 2]>,
    rot_k: Vec<[f64; 2]>,
    /// Viscous / turbulence state ([`Solver3::set_viscous`], `solver3/rans.rs`); `None`: the Euler equations.
    pub(crate) rans: Option<Box<rans::Rans3>>,
    /// Low-Mach preconditioning of the current implicit step (set by `implicit3::step` from `ImplicitParams::low_mach_precond`).
    pub(crate) precond: Option<LowMach>,
}

impl Solver3 {
    /// Solver on `grid` with the free stream everywhere.
    pub fn new(grid: Grid3, cfg: Config3) -> Solver3 {
        let (ni, nj, nk) = (grid.ni, grid.nj, grid.nk);
        let nc = ni * nj * nk;
        let mut solid = vec![false; nc];
        for b in &grid.solid {
            for k in b.k.clone() {
                for j in b.j.clone() {
                    for i in b.i.clone() {
                        solid[(k * nj + j) * ni + i] = true;
                    }
                }
            }
        }
        let n_active = solid.iter().filter(|s| !**s).count();
        // cell across the axis: the one whose centroid azimuth is closest to pi - theta
        let theta = |i: usize, k: usize| {
            let c = grid.xc[(k * nj) * ni + i];
            c[1].atan2(c[2])
        };
        let mut axis_k = vec![0; grid.axis_cells.min(ni) * nk];
        for i in 0..grid.axis_cells.min(ni) {
            for k in 0..nk {
                let target = std::f64::consts::PI - theta(i, k);
                let mut best = (f64::INFINITY, 0);
                for kk in 0..nk {
                    let d = (theta(i, kk) - target).abs();
                    if d < best.0 {
                        best = (d, kk);
                    }
                }
                axis_k[i * nk + k] = best.1;
            }
        }
        let rot = |f: &[Face3]| f.iter().map(|f| face_rot(f.c)).collect::<Vec<_>>();
        let q = prim_to_cons3(&cfg.freestream);
        Solver3 {
            u: vec![q; nc],
            w: vec![cfg.freestream; (ni + 2 * NG) * (nj + 2 * NG) * (nk + 2 * NG)],
            res: vec![[0.0; 5]; nc],
            dt: vec![0.0; nc],
            iter: 0,
            fi: vec![[0.0; 5]; grid.iface.len()],
            fj: vec![[0.0; 5]; grid.jface.len()],
            fk: vec![[0.0; 5]; grid.kface.len()],
            u0: vec![[0.0; 5]; nc],
            solid,
            n_active,
            ramp_k: 0,
            axis_k,
            rot_i: rot(&grid.iface),
            rot_j: rot(&grid.jface),
            rot_k: rot(&grid.kface),
            rans: None,
            precond: None,
            grid,
            cfg,
        }
    }

    /// Restart the CFL ramp of the implicit scheme (call at the start of a run / after a change of the free stream).
    pub fn restart_ramp(&mut self) {
        self.ramp_k = 0;
    }

    /// Primitive state of interior cell `(i, j, k)` as of the last residual evaluation.
    pub fn prim(&self, i: usize, j: usize, k: usize) -> Prim3 {
        self.w[pidx3(self.grid.ni, self.grid.nj, self.grid.nk, i as isize, j as isize, k as isize)]
    }

    /// L2 norm of `d rho / dt` normalised by sqrt(N) (N = number of fluid cells).
    pub fn density_residual(&self) -> f64 {
        let s: f64 = self.res.par_iter().map(|r| r[0] * r[0]).sum();
        (s / self.n_active as f64).sqrt()
    }

    /// One iteration with the configured [`Scheme3`]; returns the density residual at the start of the iteration.
    pub fn step(&mut self) -> f64 {
        let rn = match self.cfg.scheme {
            Scheme3::Explicit => self.rk3_step(),
            Scheme3::Implicit(p) => crate::implicit3::step(self, &p),
        };
        if self.sst() {
            self.turbulence_update();
        }
        self.iter += 1;
        rn
    }

    fn ghost_char(fs: &Prim3, edge: &Prim3, n: [f64; 3], outward: f64) -> Prim3 {
        characteristic_ghost3(fs, edge, [outward * n[0], outward * n[1], outward * n[2]]).0
    }

    fn fill_ghosts(&mut self) {
        let (ni, nj, nk) = (self.grid.ni, self.grid.nj, self.grid.nk);
        let fs = self.cfg.freestream;
        let p_out = self.rans.as_ref().and_then(|r| r.p_outflow);
        let (w, g, axis_k) = (&mut self.w, &self.grid, &self.axis_k);
        let id = |i: isize, j: isize, k: isize| pidx3(ni, nj, nk, i, j, k);
        let (nii, njj, nkk) = (ni as isize, nj as isize, nk as isize);
        for k in 0..nkk {
            for j in 0..njj {
                let base = (k as usize * nj + j as usize) * (ni + 1);
                let (fl, fr) = (&g.iface[base], &g.iface[base + ni]);
                let gl = Self::ghost_char(&fs, &w[id(0, j, k)], fl.n, -1.0);
                // viscous run: static-pressure outflow at `i = ni` (the characteristic ghost would impose the free-stream normal velocity on the boundary layer)
                let gr = match p_out {
                    Some(p) => pressure_outflow_ghost3(p, &w[id(nii - 1, j, k)], fr.n),
                    None => Self::ghost_char(&fs, &w[id(nii - 1, j, k)], fr.n, 1.0),
                };
                for m in 1..=NG as isize {
                    w[id(-m, j, k)] = gl;
                    w[id(nii + m - 1, j, k)] = gr;
                }
            }
            for i in 0..nii {
                let iu = i as usize;
                if iu < g.axis_cells {
                    let kp = axis_k[iu * nk + k as usize] as isize;
                    for m in 1..=NG as isize {
                        let mut s = w[id(i, m - 1, kp)];
                        s[2] = -s[2];
                        w[id(i, -m, k)] = s;
                    }
                } else {
                    let f = &g.jface[(k as usize * (nj + 1)) * ni + iu];
                    for m in 1..=NG as isize {
                        w[id(i, -m, k)] = reflect3(&w[id(i, m - 1, k)], f.n);
                    }
                }
                let f = &g.jface[(k as usize * (nj + 1) + nj) * ni + iu];
                let gh = Self::ghost_char(&fs, &w[id(i, njj - 1, k)], f.n, 1.0);
                for m in 1..=NG as isize {
                    w[id(i, njj + m - 1, k)] = gh;
                }
            }
        }
        for j in 0..njj {
            for i in 0..nii {
                let (fl, fh) = (&g.kface[j as usize * ni + i as usize], &g.kface[(nk * nj + j as usize) * ni + i as usize]);
                for m in 1..=NG as isize {
                    w[id(i, j, -m)] = reflect3(&w[id(i, j, m - 1)], fl.n);
                    w[id(i, j, nkk + m - 1)] = reflect3(&w[id(i, j, nkk - m)], fh.n);
                }
            }
        }
    }

    /// Evaluate `res = dU/dt` for the current `self.u` (also refreshes `w` and the face fluxes).
    pub fn eval_residual(&mut self) {
        let (ni, nj, nk) = (self.grid.ni, self.grid.nj, self.grid.nk);
        let lim = if self.iter < self.cfg.first_order_iters { Limiter::FirstOrder } else { self.cfg.limiter };
        let low_mach = self.cfg.low_mach;
        {
            let (nw, plane) = (ni + 2 * NG, (ni + 2 * NG) * (nj + 2 * NG));
            let u = &self.u;
            self.w.par_chunks_mut(plane).enumerate().filter(|(kk, _)| *kk >= NG && *kk < nk + NG).for_each(|(kk, pl)| {
                let k = kk - NG;
                for j in 0..nj {
                    for i in 0..ni {
                        pl[(j + NG) * nw + i + NG] = cons_to_prim3(&u[(k * nj + j) * ni + i]);
                    }
                }
            });
        }
        self.fill_ghosts();
        self.viscous_pass();
        let noslip = self.rans.is_some();
        let Solver3 { grid, w, fi, fj, fk, res, solid, rot_i, rot_j, rot_k, .. } = self;
        let has_solid = !grid.solid.is_empty();
        let fam = Family { ni, nj, nk, w, solid, has_solid, lim, low_mach, noslip };
        fam.fluxes(0, &grid.iface, rot_i, fi);
        fam.fluxes(1, &grid.jface, rot_j, fj);
        fam.fluxes(2, &grid.kface, rot_k, fk);
        let (grid, fi, fj, fk) = (&*grid, &*fi, &*fj, &*fk);
        let solid: &[bool] = solid;
        res.par_chunks_mut(ni * nj).enumerate().for_each(|(k, pl)| {
            for j in 0..nj {
                for i in 0..ni {
                    let c = (k * nj + j) * ni + i;
                    if solid[c] {
                        pl[j * ni + i] = [0.0; 5];
                        continue;
                    }
                    let (il, jl) = ((k * nj + j) * (ni + 1) + i, (k * (nj + 1) + j) * ni + i);
                    let (ir, jr, kr) = (il + 1, jl + ni, c + ni * nj);
                    let inv_v = 1.0 / grid.vol[c];
                    let mut out = [0.0; 5];
                    for q in 0..5 {
                        out[q] = -((fi[ir][q] * grid.iface[ir].area - fi[il][q] * grid.iface[il].area)
                            + (fj[jr][q] * grid.jface[jr].area - fj[jl][q] * grid.jface[jl].area)
                            + (fk[kr][q] * grid.kface[kr].area - fk[c][q] * grid.kface[c].area))
                            * inv_v;
                    }
                    pl[j * ni + i] = out;
                }
            }
        });
        self.add_viscous_residual();
    }

    /// Local pseudo-time step into `self.dt` from the primitives of the last residual evaluation:
    /// `CFL V / (L_i + L_j + L_k)` (`DtMode::Sum`) or `CFL V / (3 min L_d)`; `L_d = (lam(lo) + lam(hi)) / 2`,
    /// `lam(f) = (|u.n| + c) area`. Zero in solid cells.
    pub(crate) fn compute_dt(&mut self, cfl: f64, mode: DtMode) {
        let (ni, nj, nk) = (self.grid.ni, self.grid.nj, self.grid.nk);
        let (grid, w, solid) = (&self.grid, &self.w, &self.solid);
        let (rans, lm) = (self.rans.as_deref(), self.precond);
        self.dt.par_chunks_mut(ni * nj).enumerate().for_each(|(k, pl)| {
            for j in 0..nj {
                for i in 0..ni {
                    let c = (k * nj + j) * ni + i;
                    if solid[c] {
                        pl[j * ni + i] = 0.0;
                        continue;
                    }
                    let wc = &w[pidx3(ni, nj, nk, i as isize, j as isize, k as isize)];
                    let cs = sound_speed3(wc);
                    let lam = |f: &Face3| match &lm {
                        Some(l) => spectral_pc3(wc, f.n, l),
                        None => (wc[1] * f.n[0] + wc[2] * f.n[1] + wc[3] * f.n[2]).abs() + cs,
                    } * f.area;
                    let il = (k * nj + j) * (ni + 1) + i;
                    let jl = (k * (nj + 1) + j) * ni + i;
                    let mut li = 0.5 * (lam(&grid.iface[il]) + lam(&grid.iface[il + 1]));
                    let mut lj = 0.5 * (lam(&grid.jface[jl]) + lam(&grid.jface[jl + ni]));
                    let mut lk = 0.5 * (lam(&grid.kface[c]) + lam(&grid.kface[c + ni * nj]));
                    if let Some(r) = rans {
                        let d = diffusion_rates(grid, i, j, k, r.mu[c], r.mu_t[c], wc[0]);
                        (li, lj, lk) = (li + d[0], lj + d[1], lk + d[2]);
                    }
                    pl[j * ni + i] = match mode {
                        DtMode::Sum => cfl * grid.vol[c] / (li + lj + lk),
                        DtMode::MinDirection => cfl * grid.vol[c] / (3.0 * li.min(lj).min(lk)),
                    };
                }
            }
        });
    }

    /// One SSP-RK3 (Shu-Osher) step with local time stepping; returns the density residual of the first stage.
    fn rk3_step(&mut self) -> f64 {
        self.eval_residual();
        let rn = self.density_residual();
        self.compute_dt(self.cfg.cfl, DtMode::Sum);
        self.u0.copy_from_slice(&self.u);
        self.stage(1.0, 0.0);
        self.eval_residual();
        self.stage(0.25, 0.75);
        self.eval_residual();
        self.stage(2.0 / 3.0, 1.0 / 3.0);
        rn
    }

    /// `u = a (u + dt res) + b u0` (fluid cells).
    fn stage(&mut self, a: f64, b: f64) {
        let (res, dt, u0, solid) = (&self.res, &self.dt, &self.u0, &self.solid);
        self.u.par_iter_mut().enumerate().for_each(|(c, q)| {
            if solid[c] {
                return;
            }
            for k in 0..5 {
                q[k] = a * (q[k] + dt[c] * res[c][k]) + b * u0[c][k];
            }
        });
    }
}

/// Everything the face-flux kernel of one direction reads.
struct Family<'a> {
    ni: usize,
    nj: usize,
    nk: usize,
    w: &'a [Prim3],
    solid: &'a [bool],
    has_solid: bool,
    lim: Limiter,
    low_mach: bool,
    /// Viscous run: the solid side of a wall is the no-slip ghost `(rho, -u, -v, -w, p)` instead of the mirror image.
    noslip: bool,
}

impl Family<'_> {
    #[inline]
    fn is_solid(&self, c: [isize; 3]) -> bool {
        self.has_solid
            && c[0] >= 0
            && c[1] >= 0
            && c[2] >= 0
            && (c[0] as usize) < self.ni
            && (c[1] as usize) < self.nj
            && (c[2] as usize) < self.nk
            && self.solid[(c[2] as usize * self.nj + c[1] as usize) * self.ni + c[0] as usize]
    }

    #[inline]
    fn at(&self, c: [isize; 3]) -> Prim3 {
        self.w[pidx3(self.ni, self.nj, self.nk, c[0], c[1], c[2])]
    }

    /// State of the solid side of a wall with outward normal `n` seen from the fluid state `w`: the no-slip ghost (viscous run) or the mirror image (slip wall).
    #[inline]
    fn wall_ghost(&self, w: &Prim3, n: [f64; 3]) -> Prim3 {
        if self.noslip {
            wall_ghost3(w)
        } else {
            reflect3(w, n)
        }
    }

    /// Fluxes of the faces of direction `dir` (`faces` / `rots` / `out` share the layout of `Grid3::{iface, jface, kface}`).
    ///
    /// Solid cells: a face between two solid cells carries no flux; between a fluid cell `a` and a solid cell the right
    /// (solid-side) states are `reflect3(a)` and `reflect3(a-1)` (`a-1`: the next fluid cell behind `a`, `a` itself if that
    /// one is solid too); a fluid face whose outer stencil cell is solid takes the reflection of the adjacent fluid cell.
    fn fluxes(&self, dir: usize, faces: &[Face3], rots: &[[f64; 2]], out: &mut [Cons3]) {
        let nfi = self.ni + (dir == 0) as usize;
        let nfj = self.nj + (dir == 1) as usize;
        let plane = nfi * nfj;
        let e: [isize; 3] = match dir {
            0 => [1, 0, 0],
            1 => [0, 1, 0],
            _ => [0, 0, 1],
        };
        let fstride = [1, nfi, plane][dir];
        out.par_chunks_mut(plane).enumerate().for_each(|(k, po)| {
            for j in 0..nfj {
                for i in 0..nfi {
                    let fidx = k * plane + j * nfi + i;
                    let f = &faces[fidx];
                    if f.area == 0.0 {
                        po[j * nfi + i] = [0.0; 5];
                        continue;
                    }
                    let p0 = [i as isize, j as isize, k as isize];
                    let off = |s: isize| [p0[0] + s * e[0], p0[1] + s * e[1], p0[2] + s * e[2]];
                    let (cm2, cm1, cp1) = (off(-2), off(-1), off(1));
                    let (mut wm2, mut wm1, mut wp0, mut wp1) = (self.at(cm2), self.at(cm1), self.at(p0), self.at(cp1));
                    if self.has_solid {
                        let (sm1, sp0) = (self.is_solid(cm1), self.is_solid(p0));
                        if sm1 && sp0 {
                            po[j * nfi + i] = [0.0; 5];
                            continue;
                        }
                        if sm1 {
                            wm1 = self.wall_ghost(&wp0, f.n);
                            wm2 = if self.is_solid(cp1) { wm1 } else { self.wall_ghost(&wp1, f.n) };
                        } else if sp0 {
                            wp0 = self.wall_ghost(&wm1, f.n);
                            wp1 = if self.is_solid(cm2) { wp0 } else { self.wall_ghost(&wm2, f.n) };
                        } else {
                            if self.is_solid(cm2) {
                                wm2 = self.wall_ghost(&wm1, faces[fidx - fstride].n);
                            }
                            if self.is_solid(cp1) {
                                wp1 = self.wall_ghost(&wp0, faces[fidx + fstride].n);
                            }
                        }
                    }
                    po[j * nfi + i] = face_flux_lm3(&wm2, &wm1, &wp0, &wp1, self.lim, f.n, rots[fidx], self.low_mach);
                }
            }
        });
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::flux3::tests::Rng;
    use crate::gas3::freestream3;
    use crate::grid::{cone_grid, ConeGridParams, Grid};
    use crate::grid3::{revolve, SolidBox};

    fn cfg(fs: Prim3, scheme: Scheme3, limiter: Limiter) -> Config3 {
        Config3 { freestream: fs, cfl: 0.5, limiter, first_order_iters: 0, low_mach: false, scheme }
    }

    fn max_du(s: &Solver3, q0: &Cons3) -> f64 {
        s.u.iter().flat_map(|q| (0..5).map(move |k| (q[k] - q0[k]).abs())).fold(0.0, f64::max)
    }

    #[test]
    fn freestream_is_preserved() {
        let (ni, nj, nk) = (8, 8, 8);
        let mut rng = Rng::new(0x1234_5678);
        let (mut x, mut r) = (vec![0.0; (ni + 1) * (nj + 1)], vec![0.0; (ni + 1) * (nj + 1)]);
        for j in 0..=nj {
            for i in 0..=ni {
                let n = j * (ni + 1) + i;
                let (mut xx, mut rr) = (i as f64 / ni as f64, j as f64 / nj as f64);
                if i > 0 && i < ni && j > 0 && j < nj {
                    xx += 0.1 * (2.0 * rng.next() - 1.0) / ni as f64;
                    rr += 0.1 * (2.0 * rng.next() - 1.0) / nj as f64;
                }
                x[n] = xx;
                r[n] = rr;
            }
        }
        let g2 = Grid::try_from_nodes(ni, nj, x, r).unwrap();
        let theta: Vec<f64> = (0..=nk)
            .map(|k| {
                let s = k as f64 / nk as f64;
                std::f64::consts::PI * (s - 0.06 * (2.0 * std::f64::consts::PI * s).sin())
            })
            .collect();
        let mut g3 = revolve(&g2, nk, &theta).unwrap();
        g3.axis_cells = ni;
        let fs = freestream3(0.8, 6f64.to_radians());
        let q0 = prim_to_cons3(&fs);
        for (name, scheme) in
            [("explicit", Scheme3::Explicit), ("implicit", Scheme3::Implicit(ImplicitParams { cfl_max: 20.0, growth: 5.0, ..ImplicitParams::DEFAULT }))]
        {
            let mut s = Solver3::new(g3.clone(), cfg(fs, scheme, Limiter::VanAlbada));
            s.step();
            let worst = max_du(&s, &q0);
            println!("free-stream {name}: max |dU| = {worst:e}");
            assert!(worst <= 1e-12, "{name}: {worst:e}");
        }
    }

    fn cone2d(ni: usize, nj: usize) -> Grid {
        cone_grid(&ConeGridParams { ni, nj, theta: 15f64.to_radians(), length: 1.0, x_up: -0.5, h_start: 1.0, h_end: 1.0, cluster: 1.5 })
    }

    fn n_up(g: &Grid) -> usize {
        (0..g.ni).filter(|&i| g.x[i + 1] <= 1e-12).count()
    }

    fn cone3(g2: &Grid, nk: usize) -> Grid3 {
        let theta: Vec<f64> = (0..=nk).map(|k| std::f64::consts::PI * k as f64 / nk as f64).collect();
        let mut g = revolve(g2, nk, &theta).unwrap();
        g.axis_cells = n_up(g2);
        g
    }

    #[test]
    fn axisymmetric_flow_stays_axisymmetric() {
        let g2 = cone2d(32, 16);
        let nk = 8;
        let g3 = cone3(&g2, nk);
        let fs = freestream3(2.0, 0.0);
        // The scheme is rotation-equivariant, so azimuthal velocity stays at roundoff for Minmod and first order. The smooth
        // van Albada limiter (shared with 2D, `EPS = 1e-14`) is discontinuous at `b = 0` for `|a| ~ 1e-9`: a roundoff sign flip
        // of `b` changes the slope by up to `a / 2`, which seeds ~1e-11 azimuthal velocity in the first step; it does not grow.
        let ip = ImplicitParams { cfl_max: 20.0, ..ImplicitParams::DEFAULT };
        // The implicit scheme (two defect-correction passes) amplifies roundoff in the azimuthal direction by ~1e5 in the
        // far-field corner before it saturates at ~5e-11 (1e-13 with `sub_iters = 1`); the converged solution is unaffected.
        for (scheme, lim, tol, tol_p) in [
            (Scheme3::Explicit, Limiter::Minmod, 1e-12, 1e-10),
            (Scheme3::Implicit(ip), Limiter::Minmod, 1e-9, 1e-10),
            (Scheme3::Explicit, Limiter::VanAlbada, 1e-9, 1e-8),
        ] {
            let mut s = Solver3::new(g3.clone(), cfg(fs, scheme, lim));
            for _ in 0..50 {
                s.step();
            }
            let (mut az, mut dp) = (0.0f64, 0.0f64);
            for k in 0..nk {
                for j in 0..g3.nj {
                    for i in 0..g3.ni {
                        let c = g3.cell(i, j, k);
                        let w = cons_to_prim3(&s.u[c]);
                        let xc = g3.xc[c];
                        let th = xc[1].atan2(xc[2]);
                        az = az.max((w[2] * th.cos() - w[3] * th.sin()).abs());
                        let w0 = cons_to_prim3(&s.u[g3.cell(i, j, 0)]);
                        dp = dp.max((w[4] - w0[4]).abs());
                    }
                }
            }
            println!("symmetry {scheme:?} {lim:?}: max azimuthal velocity {az:e}, max pressure spread over k {dp:e}");
            assert!(az <= tol && dp <= tol_p, "{az:e} {dp:e}");
        }
    }

    /// Wall `Cp` (first fluid cell) of the 3D solver at `k = 0` and of the 2D solver, converged; max over wall cells of the difference.
    fn cone_error(nk: usize) -> f64 {
        use crate::gas::{cons_to_prim, freestream};
        use crate::solver::{Bc, Config, Scheme, Solver};
        let g2 = cone2d(40, 20);
        let g3 = cone3(&g2, nk);
        let mach = 2.0;
        let ip = ImplicitParams { cfl_max: 20.0, ..ImplicitParams::DEFAULT };
        let mut s3 = Solver3::new(g3, Config3 { first_order_iters: 50, ..cfg(freestream3(mach, 0.0), Scheme3::Implicit(ip), Limiter::VanAlbada) });
        let mut s2 = Solver::new(
            cone2d(40, 20),
            Config {
                cfl: 0.5,
                limiter: Limiter::VanAlbada,
                freestream: freestream(mach),
                bc: [Bc::Characteristic, Bc::Characteristic, Bc::SlipWall, Bc::Characteristic],
                first_order_iters: 50,
            },
        );
        s2.set_scheme(Scheme::Implicit(ip));
        let (mut r0, mut r) = (0.0, f64::INFINITY);
        for it in 0..3000 {
            r = s2.step();
            if it == 20 {
                r0 = r;
            }
            if it > 20 && r < 1e-6 * r0 {
                break;
            }
        }
        println!("2D residual {r:e} (r0 {r0:e}) after {} iterations", s2.iter);
        let (mut r0, mut r) = (0.0, f64::INFINITY);
        let t0 = std::time::Instant::now();
        for it in 0..3000 {
            r = s3.step();
            if it == 20 {
                r0 = r;
            }
            if it > 20 && r < 1e-6 * r0 {
                break;
            }
        }
        let us = t0.elapsed().as_secs_f64() * 1e6 / (s3.iter as f64 * s3.u.len() as f64);
        println!("3D nk={nk}: {us:.2} us per cell per implicit iteration ({} cells, {} threads)", s3.u.len(), rayon::current_num_threads());
        println!("3D nk={nk} residual {r:e} (r0 {r0:e}) after {} iterations", s3.iter);
        let (pinf, q) = (1.0 / crate::gas::GAMMA, 0.5 * mach * mach);
        let mut e = 0.0f64;
        for i in n_up(&g2)..g2.ni {
            let cp2 = (cons_to_prim(&s2.u[i])[3] - pinf) / q;
            let cp3 = (cons_to_prim3(&s3.u[s3.grid.cell(i, 0, 0)])[4] - pinf) / q;
            e = e.max((cp3 - cp2).abs());
        }
        e
    }

    #[test]
    fn wall_cp_converges_to_the_2d_solution() {
        let (e16, e32) = (cone_error(16), cone_error(32));
        println!("cone wall Cp error vs 2D: e(16) = {e16:e}, e(32) = {e32:e}, ratio {:.3}", e32 / e16);
        assert!(e16 <= 1e-2, "e(16) = {e16:e}");
        assert!(e32 <= 0.4 * e16, "e(32) = {e32:e} vs e(16) = {e16:e}");
    }

    /// Rectangular (aligned) revolved grid with one solid box.
    pub(crate) fn fin_grid() -> Grid3 {
        let (ni, nj, nk) = (10, 8, 8);
        let (mut x, mut r) = (vec![0.0; (ni + 1) * (nj + 1)], vec![0.0; (ni + 1) * (nj + 1)]);
        for j in 0..=nj {
            for i in 0..=ni {
                x[j * (ni + 1) + i] = 2.0 * i as f64 / ni as f64;
                r[j * (ni + 1) + i] = j as f64 / nj as f64;
            }
        }
        let g2 = Grid::try_from_nodes(ni, nj, x, r).unwrap();
        let theta: Vec<f64> = (0..=nk).map(|k| std::f64::consts::PI * k as f64 / nk as f64).collect();
        let mut g = revolve(&g2, nk, &theta).unwrap();
        g.axis_cells = ni;
        g.solid = vec![SolidBox { i: 3..5, j: 0..3, k: 3..4 }];
        g
    }

    #[test]
    fn fin_wall_is_impermeable_and_the_run_converges() {
        let g = fin_grid();
        let fs = freestream3(2.0, 0.0);
        let mut s = Solver3::new(g.clone(), cfg(fs, Scheme3::Implicit(ImplicitParams { cfl_max: 20.0, ..ImplicitParams::DEFAULT }), Limiter::VanAlbada));
        s.cfg.first_order_iters = 50;
        let (mut rmax, mut rn) = (0.0f64, 0.0);
        for _ in 0..200 {
            rn = s.step();
            rmax = rmax.max(rn);
        }
        let q0 = prim_to_cons3(&fs);
        assert!(s.u.iter().flatten().all(|v| v.is_finite()));
        println!("fin wall: residual max {rmax:e} -> {rn:e} (drop {:.2} orders)", (rmax / rn).log10());
        assert!(rn <= 1e-2 * rmax, "{rmax:e} -> {rn:e}");
        // solid cells untouched
        for k in 3..4 {
            for j in 0..3 {
                for i in 3..5 {
                    assert_eq!(s.u[g.cell(i, j, k)], q0);
                }
            }
        }
        s.eval_residual();
        let scale = fs[0] * fs[1];
        let mut worst = 0.0f64;
        let mut nwall = 0;
        for k in 0..g.nk {
            for j in 0..g.nj {
                for i in 0..=g.ni {
                    let a = i > 0 && s.solid[g.cell(i - 1, j, k)];
                    let b = i < g.ni && s.solid[g.cell(i, j, k)];
                    if a != b {
                        worst = worst.max(s.fi[(k * g.nj + j) * (g.ni + 1) + i][0].abs());
                        nwall += 1;
                    }
                }
            }
            for j in 0..=g.nj {
                for i in 0..g.ni {
                    let a = j > 0 && s.solid[g.cell(i, j - 1, k)];
                    let b = j < g.nj && s.solid[g.cell(i, j, k)];
                    if a != b {
                        worst = worst.max(s.fj[(k * (g.nj + 1) + j) * g.ni + i][0].abs());
                        nwall += 1;
                    }
                }
            }
        }
        for k in 0..=g.nk {
            for j in 0..g.nj {
                for i in 0..g.ni {
                    let a = k > 0 && s.solid[g.cell(i, j, k - 1)];
                    let b = k < g.nk && s.solid[g.cell(i, j, k)];
                    if a != b {
                        worst = worst.max(s.fk[(k * g.nj + j) * g.ni + i][0].abs());
                        nwall += 1;
                    }
                }
            }
        }
        println!("fin wall: {nwall} wall faces, max |F[0]| = {worst:e} (rho u = {scale})");
        assert!(nwall > 0 && worst <= 1e-14 * scale, "{worst:e}");
    }
}
