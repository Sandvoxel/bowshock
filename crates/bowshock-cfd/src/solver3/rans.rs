//! Viscous terms and the SST turbulence model of [`Solver3`]: the integration of [`crate::viscous3`], [`crate::walldist3`], [`crate::turbulence3`] (and, in
//! `implicit3`, [`crate::lowmach5`]) into the CPU solver. The 3D counterpart of `Solver::set_viscous` + `turbulence::{init, update}` + the viscous parts of
//! `solver.rs` / `implicit.rs`.
//!
//! # Per iteration (the order the GPU port follows)
//!
//! ```text
//!  1. cons -> prim into the padded array w; ghost fill                       (eval_residual, unchanged)
//!  2. viscous_pass:  viscous3::cell_pass          w -> grad[c] (12), mu[c]
//!                    jet_viscous_fix              (motor jet only) mu, mu_t, scalars, gradient of the Part::Jet cells = the ghost of the exit faces (jet.rs)
//!                    turbulence3::wall_state_pass w, mu, dwall, ks -> wall[c] = [k_w, omega_w, mu_t,w]   (SST only; cells with a no-slip face)
//!                    viscous3::face_pass          w, grad, mu, mu_t, wall, dirs -> fvi, fvj, fvk  (Fv.n of every face)
//!  3. inviscid face fluxes fi, fj, fk (Family::fluxes): the solid side of a wall is the NO-SLIP ghost (rho, -u, -v, -w, p), also in the MUSCL stencil
//!  4. res = -(1/V) sum S F  (unchanged);  add_viscous_residual: res += (1/V) sum S Fv;  on a WEDGE the azimuthal momentum of res is projected out
//!     (res_v, res_w -= (res_v cos(th_m) - res_w sin(th_m)) (cos(th_m), -sin(th_m)), th_m = theta_span / 2): see `add_viscous_residual`
//!  5. compute_dt: CFL V / (L_i + L_j + L_k), L_d = convective (|u.n| + c) A  [preconditioned |u'| + c' with ImplicitParams::low_mach_precond]
//!                 + the diffusive 2 lv A^2 / V (viscous3::diffusion_rates) of the cell
//!  6. implicit step (implicit3): (a) face_lambdas: lambda_pass -> lam_i, lam_j, lam_k (diffusive spectral radius of every face)
//!                                (b) block or diagonal factors with the dissipation of every face + 2 lam_f I, the wall ghost Jacobian
//!                                    diag(1, -1, -1, -1, 1) of a no-slip wall, and P in the time term / P x between the sweeps / P in the defect
//!                                    (LowMach); (c) update with positivity damping
//!  7. turbulence_update (SST): turbulence3::update(w, fi, fj, fk (mass flows), dt, grad, mu, mu_t, wall, dwall) -> turb[c] = [k, omega], mu_t[c],
//!     rnorm (the new mu_t is the one of the next iteration's step 2/4)
//! ```
//!
//! # Arrays (per cell `ni nj nk`, index `Grid3::cell`, unless stated; the GPU needs all of them resident)
//!
//! | array | meaning | written by |
//! |---|---|---|
//! | `grad` (12 f) | Green-Gauss gradient of `(u, v, w, T)` | `cell_pass` |
//! | `mu`, `mu_t` | laminar / eddy viscosity | `cell_pass`; `turbulence3::update` |
//! | `wall` (3 f) | `[k_w, omega_w, mu_t,w]` of the cells with a no-slip face | `wall_state_pass` |
//! | `dwall`, `ks` | wall distance of the centroid (once); sand-grain roughness (`0` smooth, `[]` = all smooth) | set-up |
//! | `turb` (2 f) | `[k, omega]` | `turbulence3::update` |
//! | `dirs` (3 families of faces: `e` 3 f, `d`, `noslip`, `sym`) | geometry of the compact face gradients (once) | `face_dirs` |
//! | `fvi, fvj, fvk` (5 f per face) | viscous flux `Fv.n` | `face_pass` |
//! | `lam_i, lam_j, lam_k` (1 f per face) | diffusive spectral radius of the face (implicit operator) | `lambda_pass` |
//!
//! # Boundaries and walls
//!
//! `RansOpts3::sym` (which sides of the domain box have a mirror-image ghost layer: a symmetry plane or the slip wall `j = 0`) goes to `viscous3::face_dirs`, and
//! `RansOpts3::bc` (the scalar ghost of every side: free stream or the cell's own value) to `turbulence3`. The defaults match the boundary conditions of
//! [`Solver3`]: `i0, i1, j1` far field (`Inf`), `j0` slip wall / axis, `k0, k1` symmetry planes. Every fluid/solid face is a no-slip wall (adiabatic, or isothermal with
//! [`RansOpts3::wall_thermal`]: the viscous ghost `2 T_w - T`, face viscosity `mu(T_w)`, the law of the wall at the wall temperature; the inviscid flux and the implicit wall Jacobian keep the adiabatic ghost) except the exit faces of a motor jet (`Part::Jet` cells, `solver3/jet.rs`: an inflow with the jet state, no wall, no wall state, no wall
//! distance); and so is every `j = 0` boundary face inside a run of `Grid3::wall_j0` (the body surface of a grid without solid
//! cells under it): `viscous3::face_cells` returns the `Wall` side, `fill_ghosts` fills the `j = -1, -2` layers with the no-slip ghost, `assemble3` uses the
//! no-slip ghost Jacobian and `walldist3::wall_faces` lists the face for the wall distance. A `j = 0` face outside the runs stays a slip wall.
//! The wall treatment of `turbulence3` is per wall class: the no-slip faces of the `i` family (a body's base plane, a blunt fin edge) get the plain wall state of a
//! resolved wall unless `RansOpts3::law_i_walls`; the streamwise walls (`j`, `k` families) the automatic all-`y+` treatment (`turbulence3::wall_state_pass`).
//!
//! # Wedge (`Symmetry::Wedge`)
//!
//! The `k` faces carry the pressure only in the inviscid flux (`Family::fluxes`) and the viscous flux of the mirror ghost (the hoop terms: for an axisymmetric
//! state the mirror image of a cell is its rotation by the wedge angle). The mirror also damps a swirl perturbation with `2 mu / (rho (r dtheta / 2)^2)`, an explicit
//! rate no CFL survives (swirl of round-off size grew by 500 per iteration at the axis cells), so the azimuthal momentum of the residual is projected out
//! (step 4); no `k` line, no `k` term in `compute_dt`, two factors in the block and the diagonal solves (`P` is applied between them: a `k` factor of `I` would
//! be a second `P^-1`).

use super::Solver3;
use crate::gas3::{Cons3, Prim3};
use crate::grid3::Grid3;
use crate::sst::{IsoWall, Turbulence, Viscous, WallThermal};
use crate::transition3::{gamma_field, Transition};
use crate::turbulence3::{self, ScalarBc, ScalarSides, Sst, TurbIn};
use crate::viscous3::{self, face_shape, FaceDirs3, Grad3, SymSides, WallTraction};
use crate::walldist3::{wall_distance3, wall_faces};

/// Boundary and roughness options of [`Solver3::set_viscous_with`].
#[derive(Clone, Debug, PartialEq)]
pub struct RansOpts3 {
    /// Sides `[i0, i1, j0, j1, k0, k1]` of the domain box whose ghost layer is the mirror image of the cells ([`viscous3::SymSides`]).
    pub sym: SymSides,
    /// Scalar ghost of the sides of the domain box ([`ScalarBc`]).
    pub bc: ScalarSides,
    /// Sand-grain roughness `ks` of the no-slip walls per cell (the cell next to the wall; `ni nj nk` entries, `0` smooth); empty: `Viscous::roughness`
    /// everywhere (a roughness ramp is the caller's: put it into the array).
    pub ks: Vec<f64>,
    /// Static-pressure outflow `p = p_inf` at `i = ni` (subsonic faces take `rho, u` from the cell; supersonic ones extrapolate) instead of the characteristic
    /// ghost, which would impose the free-stream normal velocity on the boundary layer (what the 2D viscous runs use: `Bc::PressureOutflow`).
    pub pressure_outflow: bool,
    /// The automatic wall treatment of `turbulence3` also for the no-slip faces of the `i` family (the base plane of a body, the blunt leading and trailing edge of a fin).
    /// `false` (default): those walls are plain resolved walls (`k_w = 0`, `omega_w` Menter's value, `mu_t,w = 0`, gradient strain), as in the 2D solver; see
    /// [`turbulence3::wall_state_pass`].
    pub law_i_walls: bool,
    /// Thermal condition of the no-slip walls ([`WallThermal`]; `Adiabatic` is the default and leaves every result bit-identical). An isothermal wall changes the ghost of the viscous
    /// kernels (`T_g = 2 T_w - T`, face viscosity `mu(T_w)`) and gives the law of the wall the density and viscosity at the wall temperature; the inviscid fluxes keep the adiabatic ghost.
    pub wall_thermal: WallThermal,
    /// Prescribed transition (`transition3`; SST only): laminar walls upstream of the given stations, turbulent downstream. `None` (default): fully turbulent, bit-identical to a
    /// solver without the feature.
    pub transition: Option<Transition>,
}

impl Default for RansOpts3 {
    fn default() -> Self {
        RansOpts3 {
            sym: [false, false, true, false, true, true],
            bc: [ScalarBc::Inf, ScalarBc::Inf, ScalarBc::Cell, ScalarBc::Inf, ScalarBc::Cell, ScalarBc::Cell],
            ks: Vec::new(),
            pressure_outflow: true,
            law_i_walls: false,
            wall_thermal: WallThermal::Adiabatic,
            transition: None,
        }
    }
}

/// The viscous / turbulence state of a [`Solver3`].
pub struct Rans3 {
    pub visc: Viscous,
    pub model: Sst,
    /// Back pressure of the `i = ni` outflow (`None`: characteristic ghost).
    pub p_outflow: Option<f64>,
    pub bc: ScalarSides,
    /// Wall class switch of [`RansOpts3::law_i_walls`].
    pub law_i_walls: bool,
    /// Isothermal wall condition of the viscous kernels (`None`: adiabatic), [`RansOpts3::wall_thermal`] resolved against the gas.
    pub iso: Option<IsoWall>,
    /// Geometry of the compact face gradients (`viscous3::face_dirs`).
    pub dirs: FaceDirs3,
    /// Gradient of `(u, v, w, T)`, laminar and eddy viscosity of every cell as of the last residual evaluation.
    pub grad: Vec<Grad3>,
    pub mu: Vec<f64>,
    pub mu_t: Vec<f64>,
    /// `[k_w, omega_w, mu_t,w]` of the cells with a no-slip face (SST; empty for a laminar run: smooth walls with `mu_t,w = 0`).
    pub wall: Vec<[f64; 3]>,
    /// Wall distance of the cell centroids (SST; empty otherwise) and the roughness per cell (empty: smooth).
    pub dwall: Vec<f64>,
    pub ks: Vec<f64>,
    /// Prescribed intermittency per cell (`transition3::gamma_field`: `0` laminar, `1` turbulent; empty: fully turbulent).
    pub gamma: Vec<f64>,
    /// `[k, omega]` per cell (SST; empty otherwise) and the density residual norm `rho dk/dt` of the last update.
    pub turb: Vec<[f64; 2]>,
    pub turb_rnorm: f64,
    /// Viscous face fluxes `Fv.n` (layouts of `Grid3::{iface, jface, kface}`).
    pub fvi: Vec<Cons3>,
    pub fvj: Vec<Cons3>,
    pub fvk: Vec<Cons3>,
    /// Diffusive spectral radius of every face for the implicit operator (`face_lambdas`).
    pub lam: [Vec<f64>; 3],
}

impl Rans3 {
    /// The diffusive spectral radii of the lower and upper face of cell `(i, j, k)` in direction `dir` (as `viscous3::face_lambdas`, from the stored arrays).
    #[inline]
    pub(crate) fn lam_pair(&self, grid: &Grid3, dir: usize, i: usize, j: usize, k: usize) -> (f64, f64) {
        let [n0, n1, _] = face_shape(grid, dir);
        let q = (k * n1 + j) * n0 + i;
        let up = q + [1, n0, n0 * n1][dir];
        (self.lam[dir][q], self.lam[dir][up])
    }
}

impl Solver3 {
    /// Switch the viscous terms on (`Some`) or off (`None`, the Euler equations) with the default boundary options ([`RansOpts3::default`]).
    pub fn set_viscous(&mut self, v: Option<Viscous>) {
        self.set_viscous_with(v, RansOpts3::default());
    }

    /// Switch the viscous terms on (`Some`) or off. Allocates the gradient, viscosity and face-flux arrays; with `Turbulence::Sst` also the turbulence state (the
    /// free stream), the exact wall distance of every cell ([`wall_distance3`], once) and the wall roughness. Every fluid/solid face of the grid is a no-slip
    /// wall, adiabatic unless `opts.wall_thermal` is isothermal; faces with `area == 0` carry no flux.
    pub fn set_viscous_with(&mut self, v: Option<Viscous>, opts: RansOpts3) {
        let Some(v) = v else {
            self.rans = None;
            return;
        };
        let nc = self.u.len();
        let sst = v.turbulence == Turbulence::Sst;
        let model = Sst { re_c: v.re_c };
        let dirs = viscous3::face_dirs(&self.grid, &self.solid, &opts.sym);
        let (mut turb, mut mu_t, mut dwall, mut wall) = (Vec::new(), vec![0.0; nc], Vec::new(), Vec::new());
        let mut ks = Vec::new();
        let mut gamma = Vec::new();
        if sst {
            let faces = wall_faces(&self.grid, &self.solid);
            dwall = wall_distance3(&self.grid, &self.solid, &faces);
            let rho = self.cfg.freestream[0];
            (turb, mu_t) = turbulence3::init(&model, &self.solid, |_| rho);
            if let Some(tr) = opts.transition {
                gamma = gamma_field(&self.grid, &self.solid, &faces, tr);
                for (m, g) in mu_t.iter_mut().zip(&gamma) {
                    *m *= g;
                }
            }
            wall = vec![[0.0; 3]; nc];
            ks = if !opts.ks.is_empty() {
                assert_eq!(opts.ks.len(), nc, "one roughness per cell");
                opts.ks
            } else if v.roughness > 0.0 {
                vec![v.roughness; nc]
            } else {
                Vec::new()
            };
        }
        let g = &self.grid;
        self.rans = Some(Box::new(Rans3 {
            visc: v,
            model,
            bc: opts.bc,
            law_i_walls: opts.law_i_walls,
            iso: opts.wall_thermal.iso(&v),
            p_outflow: opts.pressure_outflow.then_some(self.cfg.freestream[4]),
            dirs,
            grad: vec![[0.0; 12]; nc],
            mu: vec![0.0; nc],
            mu_t,
            wall,
            dwall,
            ks,
            gamma,
            turb,
            turb_rnorm: 0.0,
            fvi: vec![[0.0; 5]; g.iface.len()],
            fvj: vec![[0.0; 5]; g.jface.len()],
            fvk: vec![[0.0; 5]; g.kface.len()],
            lam: [vec![0.0; g.iface.len()], vec![0.0; g.jface.len()], vec![0.0; g.kface.len()]],
        }));
    }

    /// Replaces the flow state by `u` (the conserved variables of every cell, the free stream in the solid ones) and, for an SST run with `turb` given, the `[k, omega]` of the
    /// fluid cells with `mu_t = min(rho k / omega, 1e5 / re_c)` (the value of the initial state's cap; the GPU's `set_state_turb` does the same): a start from another field, for
    /// instance the axisymmetric solution carried to a 3D grid. The iteration count and the CFL ramp are not touched.
    pub fn set_state_turb(&mut self, u: &[Cons3], turb: Option<&[[f64; 2]]>) {
        assert_eq!(u.len(), self.u.len(), "one state per cell");
        self.u.copy_from_slice(u);
        let (Some(r), Some(given)) = (self.rans.as_mut(), turb) else { return };
        if r.turb.is_empty() {
            return;
        }
        assert_eq!(given.len(), r.turb.len(), "one [k, omega] per cell");
        let cap = 1e5 / r.model.re_c;
        for c in 0..given.len() {
            if !self.solid[c] {
                r.turb[c] = given[c];
                r.mu_t[c] = (u[c][0] * given[c][0] / given[c][1]).min(cap) * r.gamma.get(c).copied().unwrap_or(1.0);
            }
        }
    }

    /// Viscous-flow parameters, `None` for the Euler equations.
    pub fn viscous(&self) -> Option<Viscous> {
        self.rans.as_ref().map(|r| r.visc)
    }

    /// The viscous / turbulence state.
    pub fn rans(&self) -> Option<&Rans3> {
        self.rans.as_deref()
    }

    /// Is the SST turbulence model active?
    pub(crate) fn sst(&self) -> bool {
        self.rans.as_ref().is_some_and(|r| r.visc.turbulence == Turbulence::Sst)
    }

    /// L2 norm of `rho dk/dt` of the last turbulence update over the fluid cells, normalised by sqrt(N) (`0.0` without SST).
    pub fn turb_residual(&self) -> f64 {
        self.rans.as_ref().map_or(0.0, |r| r.turb_rnorm)
    }

    /// State of the solid side of a wall seen from the fluid state `w` with outward normal `n`: the no-slip ghost in a viscous run, the mirror image otherwise.
    #[inline]
    pub(crate) fn wall_ghost(&self, w: &Prim3, n: [f64; 3]) -> Prim3 {
        if self.rans.is_some() {
            viscous3::wall_ghost3(w)
        } else {
            crate::gas3::reflect3(w, n)
        }
    }

    /// Jacobian `d U_ghost / d U` of [`Solver3::wall_ghost`] in conservative variables (`diag(1, -1, -1, -1, 1)` no-slip, the reflection otherwise).
    pub(crate) fn wall_jacobian(&self, n: [f64; 3]) -> crate::implicit3::M5 {
        if self.rans.is_some() {
            let mut m = crate::implicit3::ZERO5;
            for (q, s) in [1.0, -1.0, -1.0, -1.0, 1.0].into_iter().enumerate() {
                m[6 * q] = s;
            }
            m
        } else {
            crate::implicit3::reflection3(n)
        }
    }

    /// Step 2 of the iteration: viscosity and gradients, the wall state, the viscous face fluxes. A no-op for the Euler equations.
    pub(crate) fn viscous_pass(&mut self) {
        {
            let Solver3 { grid, w, solid, rans, .. } = self;
            let Some(r) = rans.as_deref_mut() else { return };
            viscous3::cell_pass(&r.visc, grid, solid, w, r.iso, &mut r.grad, &mut r.mu);
        }
        // the jet cells are the ghosts of the exit faces: their viscosity, scalars and gradient (`cell_pass` leaves the solid cells at zero)
        self.jet_viscous_fix();
        let Solver3 { grid, w, solid, rans, .. } = self;
        let Some(r) = rans.as_deref_mut() else { return };
        if !r.wall.is_empty() {
            turbulence3::wall_state_pass(grid, solid, &r.dirs, w, &r.mu, &r.dwall, &r.ks, &r.gamma, r.iso, r.law_i_walls, &mut r.wall);
        }
        viscous3::face_pass(grid, solid, w, &r.dirs, &r.grad, &r.mu, &r.mu_t, &r.wall, r.iso, &mut r.fvi, &mut r.fvj, &mut r.fvk);
    }

    /// Step 4: `res += (1/V) sum S Fv`. On a wedge the azimuthal momentum of the residual is then projected out: the flow of a wedge has none (the pressure of its `k`
    /// faces is radial, and so are the hoop terms of their viscous fluxes), but a swirl perturbation of a cell meets the mirror ghost of the symmetry planes and is
    /// damped by `2 mu / (rho (r dtheta / 2)^2)` per unit time, a rate that the explicit `k` fluxes do not survive at any CFL (the swirl of round-off size grew by
    /// 500 per iteration at the axis cells). With the projection the swirl is neutral, as it is in the inviscid wedge, whose `k` flux has no swirl term.
    pub(crate) fn add_viscous_residual(&mut self) {
        let wedge = self.is_wedge();
        let Solver3 { grid, solid, rans, res, .. } = self;
        let Some(r) = rans.as_deref() else { return };
        viscous3::add_viscous_residual(grid, solid, &r.fvi, &r.fvj, &r.fvk, res);
        if wedge {
            let (s, c) = (0.5 * grid.theta_span).sin_cos();
            for q in res.iter_mut() {
                let t = q[2] * c - q[3] * s;
                q[2] -= t * c;
                q[3] += t * s;
            }
        }
    }

    /// Diffusive spectral radius of every face for the implicit operator (`viscous3::lambda_pass`), from the primitives and viscosities of the last
    /// residual evaluation. A no-op for the Euler equations.
    pub(crate) fn face_lambdas(&mut self) {
        let Solver3 { grid, solid, w, rans, .. } = self;
        let Some(r) = rans.as_deref_mut() else { return };
        let [li, lj, lk] = &mut r.lam;
        viscous3::lambda_pass(grid, solid, w, &r.dirs, &r.mu, &r.mu_t, &r.wall, r.iso, li, lj, lk);
    }

    /// Step 7: one update of `k`, `omega` and the eddy viscosity from the state and face fluxes of this iteration's residual evaluation.
    pub(crate) fn turbulence_update(&mut self) {
        let Solver3 { grid, w, solid, fi, fj, fk, dt, rans, .. } = self;
        let r = rans.as_deref_mut().expect("SST run");
        let cx = TurbIn {
            grid,
            solid,
            w,
            dirs: &r.dirs,
            bc: &r.bc,
            mu: &r.mu,
            mu_t: &r.mu_t,
            grad: &r.grad,
            wall: &r.wall,
            iso: r.iso,
            ks: &r.ks,
            gamma: &r.gamma,
            law_strain: true,
            law_i_walls: r.law_i_walls,
            dwall: &r.dwall,
            fi,
            fj,
            fk,
            dt,
            turb: &r.turb,
        };
        let u = turbulence3::update(&cx, &r.model);
        r.turb = u.turb;
        r.mu_t = u.mu_t;
        r.turb_rnorm = u.rnorm[0];
    }

    /// The viscous traction of every no-slip face ([`viscous3::wall_tractions`]) from the viscous fluxes of the last residual evaluation (empty for Euler).
    pub fn wall_tractions(&self) -> Vec<WallTraction> {
        self.rans.as_ref().map_or_else(Vec::new, |r| viscous3::wall_tractions(&self.grid, &self.solid, &r.dirs, &r.fvi, &r.fvj, &r.fvk))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::gas3::{cons_to_prim3, freestream3, prim_to_cons3};
    use crate::grid3::{Part, SolidBox};
    use crate::implicit_params::{ImplicitParams, LineSolve3};
    use crate::limiter::Limiter;
    use crate::solver3::{Config3, Scheme3};
    use crate::viscous3::diffusion_rates;
    use crate::viscous3::tests::box_grid;

    fn viscous(turbulence: Turbulence, re_c: f64) -> Viscous {
        Viscous { re_c, sutherland: 0.368, turbulence, wall_i_start: 0, roughness: 0.0 }
    }

    /// A `10 x 8 x 6` box of cubes of side 0.1, optionally with a solid block `3..5 x 0..3 x 2..4` on the floor (`j = 0` is the slip wall of the domain).
    fn solver(scheme: Scheme3, mach: f64, limiter: Limiter, block: bool, v: Viscous) -> Solver3 {
        let mut g = box_grid([10, 8, 6], [0.1, 0.1, 0.1]);
        if block {
            g.mark_solid(SolidBox { i: 3..5, j: 0..3, k: 2..4 }, Some(Part::Body(0)));
        }
        // the low-Mach flux correction on, as the 2D viscous tests: the right-hand side dissipation the preconditioned left-hand side stands for
        let cfg = Config3 { freestream: freestream3(mach, 0.0), cfl: 0.5, limiter, first_order_iters: 0, low_mach: true, scheme };
        let mut s = Solver3::new(g, cfg);
        s.set_viscous(Some(v));
        s
    }

    fn implicit_cfl(line_solve: LineSolve3, precond: bool, cfl_max: f64) -> Scheme3 {
        Scheme3::Implicit(ImplicitParams { cfl_max, growth: 1.5, low_mach_precond: precond, line_solve, ..ImplicitParams::VISCOUS })
    }

    fn implicit(line_solve: LineSolve3, precond: bool) -> Scheme3 {
        implicit_cfl(line_solve, precond, 10.0)
    }

    /// The diagonally dominant factorisation ([`ImplicitParams::ddadi`]) of the block lines with the correction relaxed by `omega`.
    fn implicit_dd(precond: bool, cfl_max: f64, omega: f64) -> Scheme3 {
        Scheme3::Implicit(ImplicitParams { cfl_max, growth: 1.5, low_mach_precond: precond, ddadi: Some(omega), ..ImplicitParams::VISCOUS })
    }

    const SCHEMES: [(&str, Option<(LineSolve3, bool)>); 5] = [
        ("explicit", None),
        ("block", Some((LineSolve3::Block, false))),
        ("diagonal", Some((LineSolve3::DiagonalCorrected, false))),
        ("block + P", Some((LineSolve3::Block, true))),
        ("diagonal + P", Some((LineSolve3::DiagonalCorrected, true))),
    ];

    fn scheme_of(c: Option<(LineSolve3, bool)>) -> Scheme3 {
        c.map_or(Scheme3::Explicit, |(l, p)| implicit(l, p))
    }

    fn max_du(s: &Solver3, q0: &crate::gas3::Cons3) -> f64 {
        s.u.iter().flat_map(|q| (0..5).map(move |k| (q[k] - q0[k]).abs())).fold(0.0, f64::max)
    }

    /// A uniform flow stays uniform with viscous terms (laminar and SST) in every scheme: no flux from the gradients of a constant field, no source from the
    /// preconditioner, the outflow ghost of a uniform state is the state.
    #[test]
    fn free_stream_is_preserved_by_every_scheme() {
        let q0 = prim_to_cons3(&freestream3(0.3, 0.0));
        for turbulence in [Turbulence::Laminar, Turbulence::Sst] {
            let dd = [("ddadi", implicit_dd(false, 10.0, 0.5)), ("ddadi + P", implicit_dd(true, 10.0, 0.5))];
            for (name, scheme) in SCHEMES.map(|(n, c)| (n, scheme_of(c))).into_iter().chain(dd) {
                let mut s = solver(scheme, 0.3, Limiter::VanAlbada, false, viscous(turbulence, 2e3));
                for _ in 0..3 {
                    s.step();
                }
                let worst = max_du(&s, &q0);
                assert!(worst <= 1e-12, "{turbulence:?} {name}: max |dU| = {worst:e}");
            }
        }
    }

    /// A no-slip block: the faces of the block carry no mass flux, the flow over its top is brought to rest at the wall (the first cell has lost most of its
    /// speed at `re_c = 500`) and the traction of the top face is a drag.
    #[test]
    fn no_slip_block_is_impermeable_and_drags() {
        let mach = 0.3;
        let mut s = solver(implicit(LineSolve3::Block, false), mach, Limiter::VanAlbada, true, viscous(Turbulence::Laminar, 500.0));
        for _ in 0..150 {
            s.step();
        }
        s.eval_residual();
        let r = s.rans().unwrap();
        let (mut n, mut worst) = (0, 0.0f64);
        for (dir, f) in [(0, &s.fi), (1, &s.fj), (2, &s.fk)] {
            for q in 0..f.len() {
                if r.dirs.family(dir)[q].noslip {
                    worst = worst.max(f[q][0].abs());
                    n += 1;
                }
            }
        }
        // 2 i-sides x 3 x 2 + the top 2 x 2 + 2 k-sides x 2 x 3 (the bottom is the domain's slip floor)
        assert_eq!(n, 12 + 4 + 12, "wall faces");
        assert!(worst <= 1e-14, "mass flux through a wall: {worst:e}");
        let u_top = s.prim(4, 3, 3)[1];
        let mut slip = solver(implicit(LineSolve3::Block, false), mach, Limiter::VanAlbada, true, viscous(Turbulence::Laminar, 500.0));
        slip.set_viscous(None);
        for _ in 0..150 {
            slip.step();
        }
        slip.eval_residual();
        let u_slip = slip.prim(4, 3, 3)[1];
        println!("u over the block / M: no-slip {:.3}, slip {:.3}", u_top / mach, u_slip / mach);
        assert!(u_top < 0.8 * u_slip, "u over the block: no-slip {u_top}, slip {u_slip}");
        let drag: f64 = s.wall_tractions().iter().map(|t| t.tau[0] * t.area).sum();
        println!("viscous drag of the block = {drag:.4e}");
        assert!(drag > 0.0);
    }

    /// `compute_dt` adds the diffusion of the cell to the convective spectral radii: `1 / dt_v - 1 / dt_e = sum_d lv_d / (CFL V)`.
    #[test]
    fn time_step_includes_the_diffusion() {
        let v = viscous(Turbulence::Laminar, 50.0);
        let mut sv = solver(Scheme3::Explicit, 0.3, Limiter::VanAlbada, true, v);
        let mut se = solver(Scheme3::Explicit, 0.3, Limiter::VanAlbada, true, v);
        se.set_viscous(None);
        for s in [&mut sv, &mut se] {
            s.eval_residual();
            s.compute_dt(0.5, crate::implicit_params::DtMode::Sum);
        }
        let g = &sv.grid;
        let r = sv.rans().unwrap();
        let mut checked = 0;
        for k in 0..g.nk {
            for j in 0..g.nj {
                for i in 0..g.ni {
                    let c = g.cell(i, j, k);
                    if sv.solid[c] {
                        assert_eq!(sv.dt[c], 0.0);
                        continue;
                    }
                    let d: f64 = diffusion_rates(g, i, j, k, r.mu[c], r.mu_t[c], sv.prim(i, j, k)[0]).iter().sum();
                    let want = 1.0 / se.dt[c] + d / (0.5 * g.vol[c]);
                    assert!((1.0 / sv.dt[c] - want).abs() <= 1e-9 * want, "cell ({i}, {j}, {k}): {} vs {want}", 1.0 / sv.dt[c]);
                    checked += 1;
                }
            }
        }
        assert_eq!(checked, 10 * 8 * 6 - 2 * 3 * 2);
        // the diffusion matters at this Reynolds number
        assert!(sv.dt[g.cell(8, 6, 3)] < 0.9 * se.dt[g.cell(8, 6, 3)]);
    }

    /// The Jacobian of the no-slip wall ghost is `diag(1, -1, -1, -1, 1)` (the ghost `(rho, -u, -v, -w, p)` in conservative variables).
    #[test]
    fn wall_jacobian_is_the_derivative_of_the_ghost() {
        let s = solver(Scheme3::Explicit, 0.3, Limiter::VanAlbada, true, viscous(Turbulence::Laminar, 500.0));
        let w: Prim3 = [1.2, 0.5, -0.4, 0.9, 0.8];
        let u0 = prim_to_cons3(&w);
        let m = s.wall_jacobian([0.0, 1.0, 0.0]);
        for q in 0..5 {
            let h = 1e-6;
            let (mut up, mut um) = (u0, u0);
            up[q] += h;
            um[q] -= h;
            let gp = prim_to_cons3(&s.wall_ghost(&cons_to_prim3(&up), [0.0, 1.0, 0.0]));
            let gm = prim_to_cons3(&s.wall_ghost(&cons_to_prim3(&um), [0.0, 1.0, 0.0]));
            for r in 0..5 {
                assert!(((gp[r] - gm[r]) / (2.0 * h) - m[5 * r + q]).abs() < 1e-7, "[{r}][{q}]");
            }
        }
    }

    /// Block lines, diagonal factors, block lines with the low-Mach preconditioning and the diagonally dominant block factors (DDADI, with and without the preconditioning)
    /// converge to the same steady state of the viscous flow round the block at Mach 0.1 (first-order reconstruction: smooth, no limiter chatter, CFL 3); the right-hand
    /// side is the same, so the preconditioning, the diagonalisation and the factorisation only change the path, and the preconditioning shortens it (the iteration counts
    /// are printed). The diagonal form with the preconditioning is not in the list: it does not converge here (see the header of `implicit3`).
    #[test]
    fn implicit_variants_converge_to_the_same_viscous_state() {
        let variants = [
            ("block", implicit_cfl(LineSolve3::Block, false, 3.0)),
            ("diagonal", implicit_cfl(LineSolve3::DiagonalCorrected, false, 3.0)),
            ("block + P", implicit_cfl(LineSolve3::Block, true, 3.0)),
            ("ddadi", implicit_dd(false, 3.0, 0.5)),
            ("ddadi + P", implicit_dd(true, 3.0, 0.5)),
        ];
        let mut states: Vec<(&str, Vec<crate::gas3::Cons3>, usize)> = Vec::new();
        for (name, scheme) in variants {
            let mut s = solver(scheme, 0.1, Limiter::FirstOrder, true, viscous(Turbulence::Laminar, 500.0));
            let (mut peak, mut last, mut iters) = (0.0f64, 0.0, 0);
            for it in 0..4000 {
                let rn = s.step();
                iters = it + 1;
                last = rn;
                if it > 5 {
                    peak = peak.max(rn);
                    if rn < 1e-11 * peak {
                        break;
                    }
                }
            }
            println!("{name}: {iters} iterations, density residual {last:.2e} (peak {peak:.2e})");
            assert!(peak / last > 1e10, "{name}: residual {last:e}");
            states.push((name, s.u.clone(), iters));
        }
        for (name, u, _) in &states[1..] {
            let d = u.iter().zip(&states[0].1).flat_map(|(a, b)| (0..5).map(move |k| (a[k] - b[k]).abs())).fold(0.0, f64::max);
            println!("{name} vs block: max |dU| = {d:e}");
            assert!(d < 1e-8, "{name}: {d:e}");
        }
        assert!(states[2].2 < states[0].2, "the preconditioning must shorten the run: {} vs {}", states[2].2, states[0].2);
    }

    /// 400 steps of the viscous block (van Albada, SST, `re_c` 2e4) at `scheme`: the density residual after 100 / 200 / 400 steps (or the step of the first non-finite
    /// value), the final state and the last residual.
    fn run_block(scheme: Scheme3, mach: f64) -> (String, Option<Vec<crate::gas3::Cons3>>, f64) {
        let mut s = solver(scheme, mach, Limiter::VanAlbada, true, viscous(Turbulence::Sst, 2e4));
        let mut outcome = String::new();
        let mut last = f64::NAN;
        for it in 0..400 {
            last = s.step();
            if !last.is_finite() || last > 1e6 {
                return (format!("fails at {it}"), None, last);
            }
            if [99, 199, 399].contains(&it) {
                outcome.push_str(&format!(" {last:.1e}"));
            }
        }
        (outcome, Some(s.u.clone()), last)
    }

    fn max_diff(a: &[crate::gas3::Cons3], b: &[crate::gas3::Cons3]) -> f64 {
        a.iter().zip(b).flat_map(|(a, b)| (0..5).map(move |k| (a[k] - b[k]).abs())).fold(0.0, f64::max)
    }

    /// The plain block factors die above CFL ~40 (the factorisation error grows with the CFL until the defect-correction passes diverge); the diagonally dominant factors do not, but
    /// their accurate solve of the first-order left-hand side against the limited second-order residual has a period-2 mode that the error of the plain factors was damping: at CFL 10
    /// the correction of weight 1 stays on a limit cycle (residual 6e-3 for ever, state 5e-3 off), and the correction of weight 0.5 converges to the state of the plain scheme at CFL 50
    /// where the plain factors are non-finite (Mach 2, 400 steps).
    #[test]
    fn relaxed_ddadi_converges_where_the_plain_factors_fail() {
        let mach = 2.0;
        let (_, reference, r_ref) = run_block(implicit_cfl(LineSolve3::Block, false, 6.0), mach);
        let reference = reference.expect("the plain scheme at CFL 6");
        assert!(r_ref < 1e-12, "the reference: {r_ref:e}");
        let (outcome, state, _) = run_block(implicit_cfl(LineSolve3::Block, false, 50.0), mach);
        assert!(state.is_none(), "the plain factors at CFL 50: {outcome}");
        let (outcome, state, r) = run_block(implicit_dd(false, 10.0, 1.0), mach);
        let d = max_diff(&state.expect("finite"), &reference);
        assert!(r > 1e-4 && d > 1e-4, "an accurate solve without the relaxation stalls on a cycle ({outcome}): residual {r:e}, state {d:e}");
        let (outcome, state, r) = run_block(implicit_dd(false, 50.0, 0.5), mach);
        let d = max_diff(&state.expect("finite"), &reference);
        assert!(r < 1e-10 && d < 1e-8, "the relaxed correction at CFL 50 ({outcome}): residual {r:e}, state {d:e}");
    }

    /// The plain and the diagonally dominant block factors on the viscous block over the CFL, the correction of weight 1 and of weight 0.5: residuals after 100 / 200 / 400 steps and
    /// the largest difference of the state from the converged state of the plain scheme at CFL 6 (`cargo test -- --ignored ddadi_cfl_scan --nocapture`).
    #[test]
    #[ignore]
    fn ddadi_cfl_scan() {
        for mach in [0.5, 2.0] {
            let reference = run_block(implicit_cfl(LineSolve3::Block, false, 6.0), mach).1.expect("the reference run");
            for cfl in [3.0, 6.0, 10.0, 20.0, 50.0, 300.0] {
                let mut row = format!("Mach {mach} cfl {cfl:6.0}:");
                for (name, scheme) in [
                    ("plain", implicit_cfl(LineSolve3::Block, false, cfl)),
                    ("ddadi 1.0", implicit_dd(false, cfl, 1.0)),
                    ("ddadi 0.8", implicit_dd(false, cfl, 0.8)),
                    ("ddadi 0.7", implicit_dd(false, cfl, 0.7)),
                    ("ddadi 0.6", implicit_dd(false, cfl, 0.6)),
                    ("ddadi 0.5", implicit_dd(false, cfl, 0.5)),
                ] {
                    let (outcome, u, _) = run_block(scheme, mach);
                    let d = u.map_or(f64::NAN, |u| max_diff(&u, &reference));
                    row.push_str(&format!("  {name}: {outcome:26} dU {d:.1e}"));
                }
                println!("{row}");
            }
        }
    }

    /// SST on the block: `k`, `omega` positive, `mu_t` non-negative and finite, the update residual finite, the wall state of the wall cells set.
    #[test]
    fn sst_run_stays_positive_and_sets_the_wall_state() {
        let mut s = solver(implicit(LineSolve3::Block, false), 0.3, Limiter::VanAlbada, true, viscous(Turbulence::Sst, 2e4));
        for _ in 0..100 {
            s.step();
        }
        let r = s.rans().unwrap();
        assert!(s.u.iter().all(|q| q.iter().all(|x| x.is_finite())));
        assert!(r.turb.iter().enumerate().all(|(c, t)| s.solid[c] || (t[0].is_finite() && t[0] > 0.0 && t[1].is_finite() && t[1] > 0.0)));
        assert!(r.mu_t.iter().all(|m| m.is_finite() && *m >= 0.0));
        assert!(s.turb_residual().is_finite() && s.turb_residual() > 0.0);
        let c = s.grid.cell(4, 3, 3);
        assert!(r.wall[c][1] > 0.0, "omega_w of a wall cell {:?}", r.wall[c]);
        assert!(r.dwall[c] > 0.0 && r.dwall[c] < 0.06, "wall distance {}", r.dwall[c]);
    }
}
