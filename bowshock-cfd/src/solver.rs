//! Explicit finite-volume solver: MUSCL (primitive variables) + HLLC/Rusanov shock blend + SSP-RK3 with local time stepping.
//!
//! Per-iteration data flow (every stage is a plain data-parallel kernel):
//! 1. `cons -> prim` per cell, then ghost-cell fill (2 layers per side, no corner ghosts needed);
//! 2. flux per i-face / j-face (`face_flux`: MUSCL reconstruct + HLLC blended to Rusanov in shocks);
//! 3. per-cell residual `dU/dt = -(1/V) sum_f w_f F_f + (0,0,pA/V,0)` and local `dt`.

use crate::bc::{characteristic_ghost, pressure_outflow_ghost};
use crate::flux::{face_flux, Limiter};
use crate::gas::{cons_to_prim, prim_to_cons, reflect, sound_speed, Cons, Prim};
use crate::grid::{Face, Grid};
use rayon::prelude::*;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Bc {
    /// Fixed free-stream ghost states (supersonic inflow and supersonic far-field).
    Freestream,
    /// Zero-gradient (supersonic outflow).
    Extrapolate,
    /// Slip wall: ghost cell = interior cell mirrored with the velocity reflected about the face
    /// normal. Also used for the symmetry axis `r=0` (normal = r direction); axis faces carry
    /// zero flux weight `r_mid = 0`.
    SlipWall,
    /// Characteristic (Riemann-invariant) far field for subsonic / transonic flow: per face the normal Mach
    /// number of the interior cell selects supersonic inflow (free stream), subsonic inflow/outflow
    /// (1-D Riemann invariants, free stream outside) or supersonic outflow (zero gradient), see `bc.rs`.
    Characteristic,
    /// Fixed static pressure `p_inf` outflow (subsonic faces; supersonic faces extrapolate).
    PressureOutflow,
}

#[derive(Clone, Debug)]
pub struct Config {
    pub cfl: f64,
    pub limiter: Limiter,
    pub freestream: Prim,
    /// Boundary conditions on `[imin, imax, jmin, jmax]`.
    pub bc: [Bc; 4],
    /// Number of initial iterations run with first-order reconstruction (start-up robustness).
    pub first_order_iters: usize,
}

const NG: usize = 2;

pub struct Solver {
    pub grid: Grid,
    pub cfg: Config,
    /// Conserved variables, `ni*nj`.
    pub u: Vec<Cons>,
    /// Primitive variables with `NG` ghost layers, `(ni+2NG)*(nj+2NG)`.
    pub w: Vec<Prim>,
    fi: Vec<Cons>,
    fj: Vec<Cons>,
    pub res: Vec<Cons>,
    pub dt: Vec<f64>,
    u0: Vec<Cons>,
    pub iter: usize,
}

#[inline]
fn pidx(ni: usize, i: isize, j: isize) -> usize {
    (j + NG as isize) as usize * (ni + 2 * NG) + (i + NG as isize) as usize
}

impl Solver {
    pub fn new(grid: Grid, cfg: Config) -> Solver {
        let (ni, nj) = (grid.ni, grid.nj);
        let q = prim_to_cons(&cfg.freestream);
        Solver {
            u: vec![q; ni * nj],
            w: vec![cfg.freestream; (ni + 2 * NG) * (nj + 2 * NG)],
            fi: vec![[0.0; 4]; (ni + 1) * nj],
            fj: vec![[0.0; 4]; ni * (nj + 1)],
            res: vec![[0.0; 4]; ni * nj],
            dt: vec![0.0; ni * nj],
            u0: vec![[0.0; 4]; ni * nj],
            iter: 0,
            grid,
            cfg,
        }
    }

    /// Primitive state of interior cell `(i, j)` as of the last residual evaluation.
    pub fn prim(&self, i: usize, j: usize) -> Prim {
        self.w[pidx(self.grid.ni, i as isize, j as isize)]
    }

    /// `outward` is `+1` if the face normal `(nx, nr)` (pointing towards increasing i or j) is the outward
    /// normal of the domain (imax, jmax) and `-1` if it points into the domain (imin, jmin).
    fn ghost_state(bc: Bc, fs: &Prim, edge: &Prim, mirror: &Prim, f: &Face, outward: f64) -> Prim {
        match bc {
            Bc::Freestream => *fs,
            Bc::Extrapolate => *edge,
            Bc::SlipWall => reflect(mirror, f.nx, f.nr),
            Bc::Characteristic => characteristic_ghost(fs, edge, outward * f.nx, outward * f.nr).0,
            Bc::PressureOutflow => pressure_outflow_ghost(fs[3], edge, outward * f.nx, outward * f.nr),
        }
    }

    fn fill_ghosts(&mut self) {
        let (ni, nj) = (self.grid.ni, self.grid.nj);
        let fs = self.cfg.freestream;
        let bc = self.cfg.bc;
        let (w, g) = (&mut self.w, &self.grid);
        for j in 0..nj as isize {
            for k in 1..=NG as isize {
                let f = &g.iface[j as usize * (ni + 1)];
                let (edge, mir) = (w[pidx(ni, 0, j)], w[pidx(ni, k - 1, j)]);
                w[pidx(ni, -k, j)] = Self::ghost_state(bc[0], &fs, &edge, &mir, f, -1.0);
                let f = &g.iface[j as usize * (ni + 1) + ni];
                let n = ni as isize;
                let (edge, mir) = (w[pidx(ni, n - 1, j)], w[pidx(ni, n - k, j)]);
                w[pidx(ni, n + k - 1, j)] = Self::ghost_state(bc[1], &fs, &edge, &mir, f, 1.0);
            }
        }
        for i in 0..ni as isize {
            for k in 1..=NG as isize {
                let f = &g.jface[i as usize];
                let (edge, mir) = (w[pidx(ni, i, 0)], w[pidx(ni, i, k - 1)]);
                w[pidx(ni, i, -k)] = Self::ghost_state(bc[2], &fs, &edge, &mir, f, -1.0);
                let f = &g.jface[nj * ni + i as usize];
                let n = nj as isize;
                let (edge, mir) = (w[pidx(ni, i, n - 1)], w[pidx(ni, i, n - k)]);
                w[pidx(ni, i, n + k - 1)] = Self::ghost_state(bc[3], &fs, &edge, &mir, f, 1.0);
            }
        }
    }

    /// Evaluate `res = dU/dt` for the current `self.u`; optionally the local time step `self.dt`.
    pub fn eval_residual(&mut self, compute_dt: bool) {
        let (ni, nj) = (self.grid.ni, self.grid.nj);
        let nw = ni + 2 * NG;
        let lim = if self.iter < self.cfg.first_order_iters { Limiter::FirstOrder } else { self.cfg.limiter };
        let cfl = self.cfg.cfl;

        // 1. primitives
        {
            let u = &self.u;
            self.w
                .par_chunks_mut(nw)
                .enumerate()
                .filter(|(jj, _)| *jj >= NG && *jj < nj + NG)
                .for_each(|(jj, row)| {
                    let j = jj - NG;
                    for i in 0..ni {
                        row[i + NG] = cons_to_prim(&u[j * ni + i]);
                    }
                });
        }
        self.fill_ghosts();

        let Solver { grid, w, fi, fj, res, dt, .. } = self;
        let (grid, w) = (&*grid, &*w);

        // 2a. i-face fluxes
        fi.par_chunks_mut(ni + 1).enumerate().for_each(|(j, row)| {
            let jj = j as isize;
            for (i, out) in row.iter_mut().enumerate() {
                let ii = i as isize;
                let f = &grid.iface[j * (ni + 1) + i];
                *out = face_flux(
                    &w[pidx(ni, ii - 2, jj)],
                    &w[pidx(ni, ii - 1, jj)],
                    &w[pidx(ni, ii, jj)],
                    &w[pidx(ni, ii + 1, jj)],
                    lim,
                    f.nx,
                    f.nr,
                );
            }
        });
        // 2b. j-face fluxes
        fj.par_chunks_mut(ni).enumerate().for_each(|(j, row)| {
            let jj = j as isize;
            for (i, out) in row.iter_mut().enumerate() {
                let ii = i as isize;
                let f = &grid.jface[j * ni + i];
                *out = face_flux(
                    &w[pidx(ni, ii, jj - 2)],
                    &w[pidx(ni, ii, jj - 1)],
                    &w[pidx(ni, ii, jj)],
                    &w[pidx(ni, ii, jj + 1)],
                    lim,
                    f.nx,
                    f.nr,
                );
            }
        });

        // 3. cell residuals
        let (fi, fj) = (&*fi, &*fj);
        res.par_chunks_mut(ni).zip(dt.par_chunks_mut(ni)).enumerate().for_each(|(j, (rrow, drow))| {
            for i in 0..ni {
                let c = j * ni + i;
                let fl = &grid.iface[j * (ni + 1) + i];
                let fr = &grid.iface[j * (ni + 1) + i + 1];
                let fb = &grid.jface[j * ni + i];
                let ft = &grid.jface[(j + 1) * ni + i];
                let (gl, gr) = (&fi[j * (ni + 1) + i], &fi[j * (ni + 1) + i + 1]);
                let (gb, gt) = (&fj[j * ni + i], &fj[(j + 1) * ni + i]);
                let (wl, wr, wb, wt) = (fl.w(), fr.w(), fb.w(), ft.w());
                let wc = &w[pidx(ni, i as isize, j as isize)];
                let inv_v = 1.0 / grid.vol[c];
                let mut out = [0.0; 4];
                for k in 0..4 {
                    out[k] = -(gr[k] * wr - gl[k] * wl + gt[k] * wt - gb[k] * wb) * inv_v;
                }
                out[2] += wc[3] * grid.area[c] * inv_v; // axisymmetric pressure source p*A/V
                rrow[i] = out;
                if compute_dt {
                    let cs = sound_speed(wc);
                    let lam = |f: &Face| ((wc[1] * f.nx + wc[2] * f.nr).abs() + cs) * f.len;
                    let sum = lam(fl) + lam(fr) + lam(fb) + lam(ft);
                    drow[i] = cfl * grid.area[c] / (0.5 * sum);
                }
            }
        });
    }

    /// L2 norm of `d rho / dt`, normalised by sqrt(N).
    pub fn density_residual(&self) -> f64 {
        let s: f64 = self.res.par_iter().map(|r| r[0] * r[0]).sum();
        (s / self.res.len() as f64).sqrt()
    }

    /// One SSP-RK3 (Shu-Osher) step with local time stepping. Returns the density residual of the first stage.
    ///
    /// RK3 rather than RK2: the amplification factor of RK2 has `|G|^2 = 1 + y^4/4 > 1` on the imaginary axis,
    /// so weakly damped (nearly central) modes of the limited second-order scheme grow slowly
    /// (README, "Convergence stall"); RK3 is stable there up to `|y| < sqrt(3)`.
    pub fn step(&mut self) -> f64 {
        self.eval_residual(true);
        let rn = self.density_residual();
        self.u0.copy_from_slice(&self.u);
        // stage 1: u1 = u0 + dt L(u0)
        self.stage(1.0, 0.0);
        // stage 2: u2 = 3/4 u0 + 1/4 (u1 + dt L(u1))
        self.eval_residual(false);
        self.stage(0.25, 0.75);
        // stage 3: u3 = 1/3 u0 + 2/3 (u2 + dt L(u2))
        self.eval_residual(false);
        self.stage(2.0 / 3.0, 1.0 / 3.0);
        self.iter += 1;
        rn
    }

    /// `u = a (u + dt res) + b u0`.
    fn stage(&mut self, a: f64, b: f64) {
        let (res, dt, u0) = (&self.res, &self.dt, &self.u0);
        self.u.par_iter_mut().enumerate().for_each(|(c, q)| {
            for k in 0..4 {
                q[k] = a * (q[k] + dt[c] * res[c][k]) + b * u0[c][k];
            }
        });
    }

    /// Pressure at the cell centre of `(i, j)` from the conserved state.
    pub fn pressure(&self, i: usize, j: usize) -> f64 {
        cons_to_prim(&self.u[j * self.grid.ni + i])[3]
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::gas::freestream;

    /// Smoothly curved, non-uniform grid with the lower boundary on the axis r=0.
    fn curved_grid(ni: usize, nj: usize) -> Grid {
        let (mut x, mut r) = (vec![0.0; (ni + 1) * (nj + 1)], vec![0.0; (ni + 1) * (nj + 1)]);
        for j in 0..=nj {
            for i in 0..=ni {
                let (s, t) = (i as f64 / ni as f64, j as f64 / nj as f64);
                let k = j * (ni + 1) + i;
                x[k] = 2.0 * s + 0.15 * (3.0 * t).sin() * (2.0 * s).cos() + 0.1 * t * t;
                // r=0 at j=0, strongly curved/stretched outer boundary
                r[k] = t * (1.0 + 0.3 * s) * (1.0 + 0.2 * t) + 0.05 * (4.0 * s).sin() * t * (1.0 - t);
            }
        }
        Grid::from_nodes(ni, nj, x, r)
    }

    #[test]
    fn freestream_preserved_on_curved_axisymmetric_grid() {
        let fs = freestream(2.0);
        for limiter in [Limiter::Minmod, Limiter::VanAlbada] {
            let cfg = Config {
                cfl: 0.5,
                limiter,
                freestream: fs,
                // axis (reflective, uniform axial flow is tangent), freestream/extrapolation elsewhere
                bc: [Bc::Freestream, Bc::Extrapolate, Bc::SlipWall, Bc::Freestream],
                first_order_iters: 0,
            };
            let mut s = Solver::new(curved_grid(24, 16), cfg);
            for _ in 0..50 {
                s.step();
            }
            let q0 = prim_to_cons(&fs);
            let mut worst: f64 = 0.0;
            for q in &s.u {
                for k in 0..4 {
                    worst = worst.max((q[k] - q0[k]).abs() / q0[k].abs().max(1.0));
                }
            }
            assert!(worst < 1e-12, "{limiter:?}: freestream drift {worst:e}");
        }
    }

    #[test]
    fn freestream_residual_is_roundoff_with_all_dirichlet_boundaries() {
        // All four sides fixed free stream; strongly curved grid not touching the axis.
        let fs = freestream(3.0);
        let (ni, nj) = (20, 12);
        let (mut x, mut r) = (vec![0.0; (ni + 1) * (nj + 1)], vec![0.0; (ni + 1) * (nj + 1)]);
        for j in 0..=nj {
            for i in 0..=ni {
                let (s, t) = (i as f64 / ni as f64, j as f64 / nj as f64);
                let k = j * (ni + 1) + i;
                x[k] = s + 0.05 * (5.0 * t).sin();
                r[k] = 0.5 + t + 0.1 * (6.0 * s).sin() * (1.0 + t);
            }
        }
        let cfg = Config {
            cfl: 0.5,
            limiter: Limiter::VanAlbada,
            freestream: fs,
            bc: [Bc::Freestream; 4],
            first_order_iters: 0,
        };
        let mut s = Solver::new(Grid::from_nodes(ni, nj, x, r), cfg);
        s.eval_residual(true);
        let m = s.res.iter().flat_map(|r| r.iter()).fold(0.0f64, |a, b| a.max(b.abs()));
        assert!(m < 1e-12, "max |dU/dt| = {m:e}");
    }
}
