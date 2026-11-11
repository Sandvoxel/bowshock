//! Finite-volume solver: MUSCL (primitive variables) + HLLC/Rusanov shock blend, SSP-RK3 with local time stepping
//! as the reference (`Scheme::Explicit`), optionally accelerated by implicit residual smoothing (`Scheme::Smoothed`)
//! or FAS multigrid (`Scheme::Multigrid`, see `multigrid.rs`); optional low-Mach flux correction and a blocked
//! (solid) body region for base/wake grids.
//!
//! Per-iteration data flow (every stage is a plain data-parallel kernel):
//! 1. `cons -> prim` per cell, then ghost-cell fill (2 layers per side, no corner ghosts needed);
//! 2. flux per i-face / j-face (`face_flux`: MUSCL reconstruct + HLLC blended to Rusanov in shocks);
//! 3. per-cell residual `dU/dt = -(1/V) sum_f w_f F_f + (0,0,pA/V,0)` (+ multigrid forcing) and local `dt`;
//! 4. optional implicit residual smoothing (Jacobi-iterated, all-cells-parallel).
//! 5. optional viscous terms (`Solver::set_viscous`, see `viscous.rs`): per-cell gradients and viscosity, viscous face
//!    fluxes `Fv` that enter the cell residual as `F - Fv`, no-slip wall ghost states.

use crate::bc::{characteristic_ghost, pressure_outflow_ghost};
use crate::flux::{face_flux_lm, Limiter};
use crate::gas::{cons_to_prim, prim_to_cons, reflect, sound_speed, Cons, Prim};
use crate::grid::{Face, Grid};
use crate::viscous::{self, wall_ghost, Walls};
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

/// Solid body region of the structured grid. Cells `i < i_end && j < j_end` are inside the body, and (with a sting) cells
/// `i >= i_end && j < j_sting` are inside the sting; they are never updated.
///
/// * The grid line `j = j_end` (for `i < i_end`) is the body surface (slip wall) and the grid line `i = i_end` (for
///   `j_sting <= j < j_end`) is the flat base wall.
/// * For `i >= i_end` the line `j = j_sting` is the sting surface (slip wall); with `j_sting = 0` there is no sting and the
///   first `j_end` rows behind the base are ordinary wake cells reaching the axis.
///
/// All indices must be even (multigrid coarsening halves them) and `j_sting` is `0` or `>= 2`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Block {
    pub i_end: usize,
    pub j_end: usize,
    pub j_sting: usize,
}

impl Block {
    #[inline]
    pub fn solid(&self, i: usize, j: usize) -> bool {
        if i < self.i_end {
            j < self.j_end
        } else {
            j < self.j_sting
        }
    }
    /// First fluid cell of row `j` (`None` if the whole row is solid).
    #[inline]
    pub fn row_start(&self, j: usize) -> Option<usize> {
        if j < self.j_sting {
            None
        } else if j < self.j_end {
            Some(self.i_end)
        } else {
            Some(0)
        }
    }
    /// First fluid cell of column `i`.
    #[inline]
    pub fn col_start(&self, i: usize) -> usize {
        if i < self.i_end {
            self.j_end
        } else {
            self.j_sting
        }
    }
    pub fn coarsen(&self) -> Option<Block> {
        (self.i_end % 2 == 0 && self.j_end % 2 == 0 && self.j_sting % 2 == 0).then_some(Block {
            i_end: self.i_end / 2,
            j_end: self.j_end / 2,
            j_sting: self.j_sting / 2,
        })
    }
}

/// Variable-coefficient implicit residual smoothing (Jameson / Martinelli): `(1 - eps_i d_ii)(1 - eps_j d_jj) R_bar = R`,
/// each factor a conservative scalar tridiagonal system along the grid lines, solved exactly (Thomas algorithm per line,
/// lines in parallel; a GPU port can use one thread per line or parallel cyclic reduction). The time step is the usual
/// `dt = CFL A / (L_i + L_j)` (`L_d` = spectral-radius sum of the two faces of direction `d`); each direction whose directional
/// CFL number `c_d = dt L_d / A` exceeds `cfl_star` is smoothed with `eps_d = ((c_d / cfl_star)^2 - 1) / 4`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Irs {
    /// Per-direction CFL limit `c*` of the unsmoothed scheme; `<= 0` switches smoothing off.
    pub cfl_star: f64,
}

impl Irs {
    pub const OFF: Irs = Irs { cfl_star: 0.0 };
    pub fn on(&self) -> bool {
        self.cfl_star > 0.0
    }
}

/// FAS multigrid parameters (see `multigrid.rs`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MgParams {
    /// Maximum number of grid levels including the fine grid (stops earlier if a grid dimension is odd or `< 8`).
    pub levels: usize,
    /// SSP-RK3 smoothing steps before / after the coarse-grid correction on every level but the coarsest.
    pub pre: usize,
    pub post: usize,
    /// Cycle index: 1 = V-cycle, 2 = W-cycle.
    pub gamma: usize,
    /// Smoothing steps on the coarsest level.
    pub coarsest: usize,
    /// Implicit residual smoothing inside the smoother (all levels).
    pub irs: Irs,
}

impl MgParams {
    pub const DEFAULT: MgParams = MgParams { levels: 6, pre: 1, post: 1, gamma: 1, coarsest: 4, irs: Irs::OFF };
}

/// Convergence acceleration.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Scheme {
    /// SSP-RK3 + local time stepping (the original scheme).
    Explicit,
    /// SSP-RK3 with implicit residual smoothing (allows a larger CFL).
    Smoothed(Irs),
    /// FAS multigrid cycle with an SSP-RK3 smoother (first-order coarse levels).
    Multigrid(MgParams),
    /// Line-implicit approximate-factorization pseudo-time stepping with a CFL ramp (`implicit.rs`).
    Implicit(crate::implicit::ImplicitParams),
}

/// Turbulence model of a viscous run: `Laminar` (`mu_t = 0`) or the Menter SST k-omega model (`turbulence.rs`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Turbulence {
    Laminar,
    Sst,
}

/// Viscous-flow parameters (non-dimensionalisation of `viscous.rs`: `rho_inf = c_inf = 1`, lengths in nose diameters).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Viscous {
    /// Reynolds number `rho_inf c_inf d_nose / mu_inf` of the free stream.
    pub re_c: f64,
    /// Sutherland constant `110.4 K / T_inf[K]`.
    pub sutherland: f64,
    pub turbulence: Turbulence,
    /// First `i` index (column) of the no-slip surface: the wall upstream of it (e.g. the axis or the approach to a plate)
    /// stays a slip wall.
    pub wall_i_start: usize,
}

/// Laminar Prandtl number.
pub const PRANDTL: f64 = 0.72;
/// Turbulent Prandtl number.
pub const PRANDTL_T: f64 = 0.9;

const NG: usize = 2;

pub struct Solver {
    pub grid: Grid,
    pub cfg: Config,
    /// Conserved variables, `ni*nj`.
    pub u: Vec<Cons>,
    /// Primitive variables with `NG` ghost layers, `(ni+2NG)*(nj+2NG)`.
    pub w: Vec<Prim>,
    pub(crate) fi: Vec<Cons>,
    pub(crate) fj: Vec<Cons>,
    pub res: Vec<Cons>,
    pub dt: Vec<f64>,
    u0: Vec<Cons>,
    pub iter: usize,
    /// Low-Mach correction of the reconstructed states (`flux::low_mach_states`), off by default.
    pub low_mach: bool,
    /// Solid body block (base/wake grids), `None` for ordinary grids.
    pub(crate) block: Option<Block>,
    /// Convergence acceleration (set with [`Solver::set_scheme`]).
    pub(crate) scheme: Scheme,
    pub(crate) irs: Irs,
    /// Multigrid: coarse-grid forcing added to the residual (empty on the finest level), the restricted state at the
    /// start of the coarse cycle, and the coarser levels owned by the finest solver.
    pub(crate) forcing: Vec<Cons>,
    pub(crate) mg_u0: Vec<Cons>,
    pub(crate) coarse: Vec<Solver>,
    /// Implicit-residual-smoothing coefficients per cell (i and j direction) and Thomas-algorithm scratch.
    irs_ei: Vec<f64>,
    irs_ej: Vec<f64>,
    irs_x: Vec<Cons>,
    irs_c: Vec<f64>,
    /// Number of non-solid cells (normalisation of the residual norm).
    pub(crate) n_active: usize,
    /// Iterations since the start of the implicit CFL ramp.
    pub(crate) ramp_k: usize,
    /// Viscous-flow parameters, `None` for the Euler equations (set with [`Solver::set_viscous`]).
    pub(crate) viscous: Option<Viscous>,
    /// Viscous run: cell gradients `(u_x, u_r, v_x, v_r, T_x, T_r)`, laminar viscosity and eddy viscosity per cell (`ni*nj`)
    /// as of the last residual evaluation; empty without viscous terms.
    pub grad: Vec<[f64; 6]>,
    pub mu: Vec<f64>,
    pub mu_t: Vec<f64>,
    /// Viscous run: viscous face fluxes `Fv.n` of the i-faces and j-faces (shapes of `fi`, `fj`).
    fvi: Vec<Cons>,
    fvj: Vec<Cons>,
    /// SST run: specific turbulence kinetic energy and specific dissipation rate `[k, omega]` per cell, the distance of every
    /// cell centroid to the nearest no-slip wall face, and the density residual norm of `k` of the last update; empty / zero
    /// otherwise.
    pub turb: Vec<[f64; 2]>,
    pub(crate) dwall: Vec<f64>,
    pub(crate) turb_rnorm: f64,
}

#[inline]
pub(crate) fn pidx(ni: usize, i: isize, j: isize) -> usize {
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
            low_mach: false,
            block: None,
            scheme: Scheme::Explicit,
            irs: Irs::OFF,
            forcing: Vec::new(),
            mg_u0: Vec::new(),
            coarse: Vec::new(),
            irs_ei: vec![0.0; ni * nj],
            irs_ej: vec![0.0; ni * nj],
            irs_x: Vec::new(),
            irs_c: Vec::new(),
            n_active: ni * nj,
            ramp_k: 0,
            viscous: None,
            grad: Vec::new(),
            mu: Vec::new(),
            mu_t: Vec::new(),
            fvi: Vec::new(),
            fvj: Vec::new(),
            turb: Vec::new(),
            dwall: Vec::new(),
            turb_rnorm: 0.0,
            grid,
            cfg,
        }
    }

    /// Declare the solid body block (cells inside it are frozen at the free stream and excluded from the residual norm).
    pub fn set_block(&mut self, block: Option<Block>) {
        if let Some(b) = block {
            assert!(
                b.i_end >= 1
                    && b.i_end + 2 <= self.grid.ni
                    && b.j_end >= 2
                    && b.j_end + 2 <= self.grid.nj
                    && b.j_end > b.j_sting
                    && (b.j_sting == 0 || b.j_sting >= 2),
                "block out of range"
            );
        }
        self.block = block;
        if !self.turb.is_empty() {
            self.dwall = crate::turbulence::wall_distance(&self.grid, &self.walls());
        }
        let ni = self.grid.ni;
        self.n_active = match block {
            Some(b) => ni * self.grid.nj - b.i_end * b.j_end - (ni - b.i_end) * b.j_sting,
            None => ni * self.grid.nj,
        };
    }

    /// Solid body block (base/wake grids), `None` for ordinary grids.
    pub fn block(&self) -> Option<Block> {
        self.block
    }

    /// Switch the viscous terms on (`Some`) or off (`None`, the Euler equations). Allocates the per-cell gradient, viscosity
    /// and face-flux arrays; with `Turbulence::Sst` also the turbulence state (free stream) and the wall distance, so the
    /// body block must be declared before. Viscous runs support `Scheme::Explicit` and `Scheme::Implicit` only.
    pub fn set_viscous(&mut self, v: Option<Viscous>) {
        assert!(v.is_none() || matches!(self.scheme, Scheme::Explicit | Scheme::Implicit(_)), "viscous terms need Scheme::Explicit or Scheme::Implicit");
        self.viscous = v;
        let (ni, nj) = (self.grid.ni, self.grid.nj);
        if v.is_some() {
            self.grad = vec![[0.0; 6]; ni * nj];
            self.mu = vec![0.0; ni * nj];
            self.mu_t = vec![0.0; ni * nj];
            self.fvi = vec![[0.0; 4]; (ni + 1) * nj];
            self.fvj = vec![[0.0; 4]; ni * (nj + 1)];
        } else {
            self.grad = Vec::new();
            self.mu = Vec::new();
            self.mu_t = Vec::new();
            self.fvi = Vec::new();
            self.fvj = Vec::new();
        }
        self.turb = Vec::new();
        self.dwall = Vec::new();
        self.turb_rnorm = 0.0;
        if let Some(Viscous { re_c, turbulence: Turbulence::Sst, .. }) = v {
            crate::turbulence::init(self, re_c);
        }
    }

    /// Viscous-flow parameters, `None` for the Euler equations.
    pub fn viscous(&self) -> Option<Viscous> {
        self.viscous
    }

    /// No-slip classification of the walls of this solver (see [`Walls`]).
    pub(crate) fn walls(&self) -> Walls {
        Walls::new(self.viscous, &self.cfg.bc, self.block)
    }

    /// Convergence acceleration in use (set with [`Solver::set_scheme`]).
    pub fn scheme(&self) -> Scheme {
        self.scheme
    }

    /// Primitive state of interior cell `(i, j)` as of the last residual evaluation.
    pub fn prim(&self, i: usize, j: usize) -> Prim {
        self.w[pidx(self.grid.ni, i as isize, j as isize)]
    }

    /// `outward` is `+1` if the face normal `(nx, nr)` (pointing towards increasing i or j) is the outward
    /// normal of the domain (imax, jmax) and `-1` if it points into the domain (imin, jmin).
    pub(crate) fn ghost_state(bc: Bc, fs: &Prim, edge: &Prim, mirror: &Prim, f: &Face, outward: f64) -> Prim {
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
        let walls = self.walls();
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
                w[pidx(ni, i, -k)] =
                    if walls.noslip_jface(i as usize, 0) { wall_ghost(&mir, f, true) } else { Self::ghost_state(bc[2], &fs, &edge, &mir, f, -1.0) };
                let f = &g.jface[nj * ni + i as usize];
                let n = nj as isize;
                let (edge, mir) = (w[pidx(ni, i, n - 1)], w[pidx(ni, i, n - k)]);
                w[pidx(ni, i, n + k - 1)] = Self::ghost_state(bc[3], &fs, &edge, &mir, f, 1.0);
            }
        }
    }

    /// Solid cells of the two layers below a body or sting surface (`col_start(i) - 1`, `col_start(i) - 2`) take the mirrored
    /// states of the fluid cells above it (reflected about the wall normal), exactly like wall ghost cells. The base-wall
    /// (i-direction) stencils are handled in the i-face kernel (the corner cells need a different mirror for the two directions).
    fn fill_block_ghosts(&mut self, b: Block) {
        let ni = self.grid.ni;
        let walls = self.walls();
        for i in 0..ni {
            let j0 = b.col_start(i);
            if j0 == 0 {
                continue;
            }
            let f = self.grid.jface[j0 * ni + i];
            for k in 1..=NG {
                let src = self.w[pidx(ni, i as isize, (j0 + k - 1) as isize)];
                self.w[pidx(ni, i as isize, j0 as isize - k as isize)] = wall_ghost(&src, &f, walls.noslip_jface(i, j0));
            }
        }
    }

    /// Evaluate `res = dU/dt` for the current `self.u`; optionally the local time step `self.dt`.
    pub fn eval_residual(&mut self, compute_dt: bool) {
        let (ni, nj) = (self.grid.ni, self.grid.nj);
        let nw = ni + 2 * NG;
        let lim = if self.iter < self.cfg.first_order_iters { Limiter::FirstOrder } else { self.cfg.limiter };
        let cfl = self.cfg.cfl;
        let low_mach = self.low_mach;
        let blk = self.block;
        let irs = self.irs;
        let visc = self.viscous;
        let walls = self.walls();

        // 1. primitives
        {
            let u = &self.u;
            self.w.par_chunks_mut(nw).enumerate().filter(|(jj, _)| *jj >= NG && *jj < nj + NG).for_each(|(jj, row)| {
                let j = jj - NG;
                for i in 0..ni {
                    row[i + NG] = cons_to_prim(&u[j * ni + i]);
                }
            });
        }
        self.fill_ghosts();
        if let Some(b) = blk {
            self.fill_block_ghosts(b);
        }

        let Solver { grid, w, fi, fj, res, dt, forcing, irs_ei, irs_ej, grad, mu, mu_t, fvi, fvj, .. } = self;
        let (grid, w) = (&*grid, &*w);
        let forcing: &[Cons] = forcing;

        // 2a. i-face fluxes
        fi.par_chunks_mut(ni + 1).enumerate().for_each(|(j, row)| {
            let jj = j as isize;
            for (i, out) in row.iter_mut().enumerate() {
                let ii = i as isize;
                let f = &grid.iface[j * (ni + 1) + i];
                if let Some(b) = blk {
                    if j < b.j_end {
                        if j < b.j_sting || i < b.i_end {
                            *out = [0.0; 4]; // inside the body / sting
                            continue;
                        }
                        if i <= b.i_end + 1 {
                            // base wall (i == i_end) and the face next to it: solid stencil cells are the reflection
                            // of the fluid cell across the base about the base-wall normal
                            let fb = &grid.iface[j * (ni + 1) + b.i_end];
                            let (m1, p0, p1) = (w[pidx(ni, ii - 1, jj)], w[pidx(ni, ii, jj)], w[pidx(ni, ii + 1, jj)]);
                            let (m2, m1) = if i == b.i_end {
                                let ns = walls.noslip_iface(i, j);
                                (wall_ghost(&p1, fb, ns), wall_ghost(&p0, fb, ns))
                            } else {
                                (wall_ghost(&m1, fb, walls.noslip_iface(b.i_end, j)), m1)
                            };
                            *out = face_flux_lm(&m2, &m1, &p0, &p1, lim, f.nx, f.nr, low_mach);
                            continue;
                        }
                    }
                }
                *out =
                    face_flux_lm(&w[pidx(ni, ii - 2, jj)], &w[pidx(ni, ii - 1, jj)], &w[pidx(ni, ii, jj)], &w[pidx(ni, ii + 1, jj)], lim, f.nx, f.nr, low_mach);
            }
        });
        // 2b. j-face fluxes
        fj.par_chunks_mut(ni).enumerate().for_each(|(j, row)| {
            let jj = j as isize;
            for (i, out) in row.iter_mut().enumerate() {
                let ii = i as isize;
                let f = &grid.jface[j * ni + i];
                if let Some(b) = blk {
                    if j < b.col_start(i) {
                        *out = [0.0; 4]; // inside the body / sting
                        continue;
                    }
                }
                *out =
                    face_flux_lm(&w[pidx(ni, ii, jj - 2)], &w[pidx(ni, ii, jj - 1)], &w[pidx(ni, ii, jj)], &w[pidx(ni, ii, jj + 1)], lim, f.nx, f.nr, low_mach);
            }
        });

        // 2c. viscous: cell gradients and viscosity, viscous face fluxes
        if let Some(v) = visc {
            viscous::cell_pass(&v, grid, w, &walls, grad, mu);
            viscous::face_pass(grid, w, &walls, grad, mu, mu_t, fvi, fvj);
        }
        let (grad, mu, mu_t): (&[[f64; 6]], &[f64], &[f64]) = (grad, mu, mu_t);
        let (fvi, fvj): (&[Cons], &[Cons]) = (fvi, fvj);

        // 3. cell residuals
        let (fi, fj) = (&*fi, &*fj);
        res.par_chunks_mut(ni).zip(dt.par_chunks_mut(ni)).zip(irs_ei.par_chunks_mut(ni)).zip(irs_ej.par_chunks_mut(ni)).enumerate().for_each(
            |(j, (((rrow, drow), erow_i), erow_j))| {
                for i in 0..ni {
                    if blk.is_some_and(|b| b.solid(i, j)) {
                        rrow[i] = [0.0; 4];
                        drow[i] = 0.0;
                        erow_i[i] = 0.0;
                        erow_j[i] = 0.0;
                        continue;
                    }
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
                    if visc.is_some() {
                        let (hl, hr) = (&fvi[j * (ni + 1) + i], &fvi[j * (ni + 1) + i + 1]);
                        let (hb, ht) = (&fvj[j * ni + i], &fvj[(j + 1) * ni + i]);
                        for k in 0..4 {
                            out[k] = -((gr[k] - hr[k]) * wr - (gl[k] - hl[k]) * wl + (gt[k] - ht[k]) * wt - (gb[k] - hb[k]) * wb) * inv_v;
                        }
                        let t_tt = viscous::hoop_stress(mu[c] + mu_t[c], &grad[c], wc[2], grid.rc[c]);
                        out[2] += (wc[3] - t_tt) * grid.area[c] * inv_v; // axisymmetric pressure source (p - t_tt) A / V
                    } else {
                        for k in 0..4 {
                            out[k] = -(gr[k] * wr - gl[k] * wl + gt[k] * wt - gb[k] * wb) * inv_v;
                        }
                        out[2] += wc[3] * grid.area[c] * inv_v; // axisymmetric pressure source p*A/V
                    }
                    if !forcing.is_empty() {
                        for k in 0..4 {
                            out[k] += forcing[c][k];
                        }
                    }
                    rrow[i] = out;
                    if compute_dt {
                        let cs = sound_speed(wc);
                        let lam = |f: &Face| ((wc[1] * f.nx + wc[2] * f.nr).abs() + cs) * f.len;
                        // diffusive limit of the time step (viscous run)
                        let (xi, xj) = if visc.is_some() {
                            (
                                viscous::diffusion_rate(mu[c], mu_t[c], wc[0], 0.5 * (fl.len + fr.len), grid.area[c]),
                                viscous::diffusion_rate(mu[c], mu_t[c], wc[0], 0.5 * (fb.len + ft.len), grid.area[c]),
                            )
                        } else {
                            (0.0, 0.0)
                        };
                        if irs.on() {
                            let (mut li, mut lj) = (0.5 * (lam(fl) + lam(fr)), 0.5 * (lam(fb) + lam(ft)));
                            if visc.is_some() {
                                li += xi;
                                lj += xj;
                            }
                            let d = cfl * grid.area[c] / (li + lj);
                            drow[i] = d;
                            let coef = |l: f64| {
                                let r = d * l / grid.area[c] / irs.cfl_star;
                                (0.25 * (r * r - 1.0)).max(0.0)
                            };
                            erow_i[i] = coef(li);
                            erow_j[i] = coef(lj);
                        } else {
                            let sum = lam(fl) + lam(fr) + lam(fb) + lam(ft);
                            drow[i] = if visc.is_some() { cfl * grid.area[c] / (0.5 * sum + xi + xj) } else { cfl * grid.area[c] / (0.5 * sum) };
                        }
                    }
                }
            },
        );
    }

    /// L2 norm of `d rho / dt`, normalised by sqrt(N) (N = number of fluid cells). Includes the multigrid forcing.
    pub fn density_residual(&self) -> f64 {
        let s: f64 = self.res.par_iter().map(|r| r[0] * r[0]).sum();
        (s / self.n_active as f64).sqrt()
    }

    /// One iteration with the configured [`Scheme`]; returns the density residual at the start of the iteration
    /// (of the finest grid, before any smoothing of the residual).
    pub fn step(&mut self) -> f64 {
        let rn = match self.scheme {
            Scheme::Explicit => {
                self.irs = Irs::OFF;
                self.rk3_step()
            }
            Scheme::Smoothed(irs) => {
                self.irs = irs;
                self.rk3_step()
            }
            Scheme::Multigrid(p) => crate::multigrid::cycle(self, &p),
            Scheme::Implicit(p) => crate::implicit::step(self, &p),
        };
        if self.sst() {
            crate::turbulence::update(self);
        }
        self.iter += 1;
        rn
    }

    /// Is the SST turbulence model active?
    pub(crate) fn sst(&self) -> bool {
        self.viscous.is_some_and(|v| v.turbulence == Turbulence::Sst)
    }

    /// L2 norm of `rho dk/dt` of the last turbulence update over the fluid cells, normalised by sqrt(N) (`0.0` without SST).
    pub fn turb_residual(&self) -> f64 {
        self.turb_rnorm
    }

    /// One SSP-RK3 (Shu-Osher) step with local time stepping (and the implicit residual smoothing `self.irs`, if any).
    /// Returns the density residual of the first stage (before smoothing). Does not advance `iter`.
    ///
    /// RK3 rather than RK2: the amplification factor of RK2 has `|G|^2 = 1 + y^4/4 > 1` on the imaginary axis,
    /// so weakly damped (nearly central) modes of the limited second-order scheme grow slowly
    /// (README, "Convergence stall"); RK3 is stable there up to `|y| < sqrt(3)`.
    pub(crate) fn rk3_step(&mut self) -> f64 {
        self.eval_residual(true);
        let rn = self.density_residual();
        self.smooth_residual();
        self.u0.copy_from_slice(&self.u);
        // stage 1: u1 = u0 + dt L(u0)
        self.stage(1.0, 0.0);
        // stage 2: u2 = 3/4 u0 + 1/4 (u1 + dt L(u1))
        self.eval_residual(false);
        self.smooth_residual();
        self.stage(0.25, 0.75);
        // stage 3: u3 = 1/3 u0 + 2/3 (u2 + dt L(u2))
        self.eval_residual(false);
        self.smooth_residual();
        self.stage(2.0 / 3.0, 1.0 / 3.0);
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

    /// Implicit residual smoothing of `self.res`: `(1 - eps_i d_ii)(1 - eps_j d_jj) R_bar = R`. Each factor is the
    /// conservative tridiagonal system `-e_- x_{k-1} + (1 + e_- + e_+) x_k - e_+ x_{k+1} = r_k` along a grid line with the
    /// face coefficients `e_+ = (eps_k + eps_{k+1}) / 2` (zero across solid cells), solved exactly with the Thomas algorithm
    /// (lines in parallel).
    fn smooth_residual(&mut self) {
        if !self.irs.on() {
            return;
        }
        let (ni, nj) = (self.grid.ni, self.grid.nj);
        let blk = self.block;
        let solid = move |i: usize, j: usize| blk.is_some_and(|b| b.solid(i, j));
        let mut x = std::mem::take(&mut self.irs_x);
        let mut cp = std::mem::take(&mut self.irs_c);
        x.resize(ni * nj, [0.0; 4]);
        cp.resize(ni * nj, 0.0);
        let (ei, ej) = (&self.irs_ei, &self.irs_ej);
        // i lines: one per row j
        {
            let res = &self.res;
            x.par_chunks_mut(ni).zip(cp.par_chunks_mut(ni)).enumerate().for_each(|(j, (xr, cr))| {
                let e = |i: usize| ei[j * ni + i];
                let face = |i: usize| if solid(i, j) || solid(i + 1, j) { 0.0 } else { 0.5 * (e(i) + e(i + 1)) };
                thomas(ni, |k| res[j * ni + k], |k| if k > 0 { face(k - 1) } else { 0.0 }, |k| if k + 1 < ni { face(k) } else { 0.0 }, xr, cr);
            });
        }
        // j lines: one per column i (the result of the i factor is the right-hand side)
        {
            let rhs = &x;
            let mut out = std::mem::take(&mut self.res);
            let mut col_ptrs: Vec<usize> = (0..ni).collect();
            // gather per column into scratch, solve, scatter: columns are independent
            let results: Vec<Vec<Cons>> = col_ptrs
                .par_iter_mut()
                .map(|i| {
                    let i = *i;
                    let mut xr = vec![[0.0; 4]; nj];
                    let mut cr = vec![0.0; nj];
                    let e = |j: usize| ej[j * ni + i];
                    let face = |j: usize| if solid(i, j) || solid(i, j + 1) { 0.0 } else { 0.5 * (e(j) + e(j + 1)) };
                    thomas(nj, |k| rhs[k * ni + i], |k| if k > 0 { face(k - 1) } else { 0.0 }, |k| if k + 1 < nj { face(k) } else { 0.0 }, &mut xr, &mut cr);
                    xr
                })
                .collect();
            for (i, col) in results.iter().enumerate() {
                for (j, v) in col.iter().enumerate() {
                    out[j * ni + i] = *v;
                }
            }
            self.res = out;
        }
        self.irs_x = x;
        self.irs_c = cp;
    }

    /// Pressure at the cell centre of `(i, j)` from the conserved state.
    pub fn pressure(&self, i: usize, j: usize) -> f64 {
        cons_to_prim(&self.u[j * self.grid.ni + i])[3]
    }
}

/// Thomas algorithm for `-lo_k x_{k-1} + (1 + lo_k + up_k) x_k - up_k x_{k+1} = rhs_k` (`n` unknowns, 4 components).
#[inline]
fn thomas(n: usize, rhs: impl Fn(usize) -> Cons, lo: impl Fn(usize) -> f64, up: impl Fn(usize) -> f64, x: &mut [Cons], cp: &mut [f64]) {
    let mut prev = [0.0; 4];
    for k in 0..n {
        let (a, c) = (lo(k), up(k));
        let b = 1.0 + a + c;
        let m = b + a * if k > 0 { cp[k - 1] } else { 0.0 };
        cp[k] = -c / m;
        let r = rhs(k);
        for q in 0..4 {
            x[k][q] = (r[q] + a * prev[q]) / m;
        }
        prev = x[k];
    }
    for k in (0..n.saturating_sub(1)).rev() {
        let nx = x[k + 1];
        for q in 0..4 {
            x[k][q] -= cp[k] * nx[q];
        }
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    use crate::gas::freestream;

    /// Smoothly curved, non-uniform grid with the lower boundary on the axis r=0.
    pub(crate) fn curved_grid(ni: usize, nj: usize) -> Grid {
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
    fn thomas_solves_the_conservative_smoothing_system() {
        let n = 9;
        let e: Vec<f64> = (0..n + 1).map(|k| if k == 0 || k == n { 0.0 } else { 0.3 + 2.0 * (k as f64).sin().abs() }).collect();
        let rhs: Vec<Cons> = (0..n).map(|k| [k as f64, 1.0, (k as f64).cos(), -0.5 * k as f64]).collect();
        let (mut x, mut cp) = (vec![[0.0; 4]; n], vec![0.0; n]);
        thomas(n, |k| rhs[k], |k| e[k], |k| e[k + 1], &mut x, &mut cp);
        for k in 0..n {
            for q in 0..4 {
                let lo = if k > 0 { x[k - 1][q] } else { 0.0 };
                let up = if k + 1 < n { x[k + 1][q] } else { 0.0 };
                let lhs = (1.0 + e[k] + e[k + 1]) * x[k][q] - e[k] * lo - e[k + 1] * up;
                assert!((lhs - rhs[k][q]).abs() < 1e-12, "k={k} q={q}: {lhs} vs {}", rhs[k][q]);
            }
        }
        // conservative: the sum of the smoothed residual equals the sum of the residual
        for q in 0..4 {
            let (a, b): (f64, f64) = (x.iter().map(|v| v[q]).sum(), rhs.iter().map(|v| v[q]).sum());
            assert!((a - b).abs() < 1e-12);
        }
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
        let cfg = Config { cfl: 0.5, limiter: Limiter::VanAlbada, freestream: fs, bc: [Bc::Freestream; 4], first_order_iters: 0 };
        let mut s = Solver::new(Grid::from_nodes(ni, nj, x, r), cfg);
        s.eval_residual(true);
        let m = s.res.iter().flat_map(|r| r.iter()).fold(0.0f64, |a, b| a.max(b.abs()));
        assert!(m < 1e-12, "max |dU/dt| = {m:e}");
    }
}
