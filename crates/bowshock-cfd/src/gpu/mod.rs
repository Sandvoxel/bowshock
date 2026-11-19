//! The GPU solver (feature `gpu`): the axisymmetric Euler solver of the CPU modules (MUSCL + HLLC / Rusanov shock blend + optional low-Mach
//! correction, characteristic and pressure-outflow far fields, SSP-RK3 with local time stepping or the line-implicit scheme, optional
//! solid body block) written in plain Luma array ops: no custom kernels. Luma's lazy arrays fuse the expressions; `iterate_until_all`
//! traces one step and replays its kernels, recording the density residual on the GPU.
//!
//! The solver is in `body.rs` (explicit scheme) and `implicit.rs` (line-implicit scheme), each function the mirror of the CPU routine of the
//! same name in the CPU modules of this crate, included once per precision:
//!
//! * [`f32s`]: `Array<f32>` everywhere ([`Precision::F32`]).
//! * [`df32s`]: the state, the residual and the time step in double-float `Array<Df32>` ([`Precision::Extended`]); the
//!   state is a `Df32` record through the loops.
//!
//! The line-implicit scheme's left-hand side is f32 in both. The viscous terms of the compressible Navier-Stokes equations (`viscous.rs`) and the
//! Menter SST k-omega model (`turbulence.rs`) follow, for both schemes and for grids with a solid block; their left-hand sides (the `2 lam I` of
//! the implicit operator, the line solves of the two turbulence scalars) are f32 like the rest. [`GpuSolver`] is the precision-independent handle
//! the other crates use. See the README.
//!
//! The 3D Euler solver of `solver3.rs` is in `body3.rs` (SSP-RK3) and `implicit3.rs` (three-factor line-implicit scheme), [`f32s::LazySolver3`] /
//! [`df32s::LazySolver3`], behind [`GpuSolver3`].

use crate::gas3::Cons3;
use crate::grid::Block;
use crate::grid::Grid;
use crate::grid3::Grid3;
use crate::solver::{Config, Nozzle, Scheme, Solver};
use crate::solver3::{Config3, Solver3};
use crate::sst::Viscous;
use luma::Context;
pub use luma::LumaError;

/// Errors of the GPU solver: a Luma failure or a configuration the GPU solver does not implement.
#[derive(Debug, Clone)]
pub enum GpuError {
    /// Failure reported by Luma (adapter, device, kernel build, buffer transfer).
    Luma(LumaError),
    /// The requested scheme / block / viscous combination is not implemented on the GPU.
    Unsupported(&'static str),
}

impl From<LumaError> for GpuError {
    fn from(e: LumaError) -> GpuError {
        GpuError::Luma(e)
    }
}

impl std::fmt::Display for GpuError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            GpuError::Luma(e) => write!(f, "{e}"),
            GpuError::Unsupported(m) => f.write_str(m),
        }
    }
}

impl std::error::Error for GpuError {}

/// Result of the GPU solver operations.
pub type Result<T> = std::result::Result<T, GpuError>;

/// Precision of the state, the residual and the time step.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum Precision {
    /// `Array<f32>` everywhere ([`f32s`]); the density residual stalls about 4-4.6 orders below its peak at f32 round-off.
    F32,
    /// Double-float (`Array<Df32>`, ~48 bit) state, residual and time step ([`df32s`]): converges like the f64 CPU solver.
    #[default]
    Extended,
}

/// Peak device bytes per cell of a [`GpuSolver3`] (f32, SST, `sub_iters` 2) and the other settings' factors, measured by the process figure of `nvidia-smi -lms 5` less the 256 MiB
/// CUDA context, on the Finner sector at refinements 1 to 2.25 (0.69 to 6.8 M cells), the half domain (1.35 M) and G-F1, Luma 231c986 (DESIGN3D item 49): 2950 to 4091 B per cell
/// (the power-of-two size classes of the line-solve scratch and the plan make it jump with the cell count); the largest is the bound. `sub_iters` 1 / 3 / 4 measured 93 / 101 / 102 % of
/// `sub_iters` 2, double-float 143 %, Euler 85 %.
const BYTES_PER_CELL3: f64 = 4100.0;
const EXTENDED_FACTOR3: f64 = 1.45;
const EULER_FACTOR3: f64 = 0.86;
/// The CUDA context and the kernels of the process.
const CONTEXT_BYTES3: u64 = 300 << 20;

/// The device memory (bytes) a 3D solver of `cells` cells needs at its peak, which is the trace of its loop (persistent arrays, the plan Luma keeps and the intermediates of the step while
/// they are recorded). Linear in the cells, an upper bound of everything measured; `sub_iters` above 3 and `Precision::Extended` raise it, a solver without the SST terms lowers it.
pub fn device_bytes3(cells: usize, prec: Precision, sub_iters: usize, rans: bool) -> u64 {
    let mut per_cell = BYTES_PER_CELL3 + 100.0 * sub_iters.saturating_sub(3) as f64;
    if prec == Precision::Extended {
        per_cell *= EXTENDED_FACTOR3;
    }
    if !rans {
        per_cell *= EULER_FACTOR3;
    }
    CONTEXT_BYTES3 + (cells as f64 * per_cell) as u64
}

/// Iterations of one `iterate_until_all` submission inside [`GpuSolver::run`]: the host waits once per `RUN_SYNC` iterations.
const RUN_SYNC: usize = 50;

/// How much of the order of evaluation is left to Luma's fusion engine (`LazySolver::staging`, see README).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Staging {
    /// No `cache()` anywhere: the whole step is one lazy expression and Luma's fusion decides where to cut.
    Fusion,
    /// The padded primitive record (state with its ghost layers) is evaluated once per residual evaluation.
    Lines,
    /// Additionally the face fluxes are evaluated: three kernels per Runge-Kutta stage (padded state, i fluxes, j fluxes),
    /// the stage update and the residual fuse into the padded-state kernel of the next stage.
    Fluxes,
    /// Additionally the reconstructed states and the shock weight are evaluated before the Riemann solvers.
    Split,
}

/// Single precision.
pub mod f32s {
    pub type S = f32;

    fn c(x: f64) -> S {
        x as f32
    }

    /// Host data (row-major, `i` fastest) as an array of `shape`.
    fn up(ctx: &Context, data: &[f64], shape: impl Into<luma::Shape>) -> Result<A> {
        let v: Vec<f32> = data.iter().map(|&x| x as f32).collect();
        Array::from_slice_on(ctx, &v, shape)
    }

    /// Measured fastest (README, speed table): `Fluxes` is 14 launches per step, `Fusion` (no `cache()`) 67-91.
    const DEFAULT_STAGING: Staging = Staging::Fluxes;

    fn to_f64(a: &A) -> Result<Vec<f64>> {
        Ok(a.to_vec()?.into_iter().map(f64::from).collect())
    }

    /// The density residual of every step, recorded on the GPU.
    fn records(until: luma::Until<'static, State>) -> luma::Until<'static, State> {
        until.record("residual", |s: &State| s[4].clone())
    }

    fn read_history(run: &luma::Run<State>) -> Result<Vec<f64>> {
        let h = run.history("residual").ok_or_else(|| luma::LumaError::Unsupported("the run recorded no residual history".into()))?;
        Ok(h.iter().map(|&r| f64::from(r)).collect())
    }

    /// The density residual of every step of a 3D run, recorded on the GPU (`State3` holds it at index 5); with `monitor`, the three force sums (index 10).
    fn records3(until: luma::Until<'static, State3>, monitor: bool) -> luma::Until<'static, State3> {
        let mut until = until.record("residual", |s: &State3| s[5].clone());
        if monitor {
            for (q, name) in ["fx", "fz", "mz"].into_iter().enumerate() {
                until = until.record(name, move |s: &State3| s[10].slice(0, q..q + 1));
            }
        }
        until
    }

    fn read_history3(run: &luma::Run<State3>) -> Result<Vec<f64>> {
        let h = run.history("residual").ok_or_else(|| luma::LumaError::Unsupported("the run recorded no residual history".into()))?;
        Ok(h.iter().map(|&r| f64::from(r)).collect())
    }

    /// The force monitor of every step (`[Fx, Fz, Mz]`, see `records3`).
    fn read_forces3(run: &luma::Run<State3>) -> Result<Vec<[f64; 3]>> {
        let missing = || luma::LumaError::Unsupported("the run recorded no force history".into());
        let word = |name: &str| run.history(name).ok_or_else(missing);
        let (x, z, m) = (word("fx")?, word("fz")?, word("mz")?);
        Ok((0..x.len()).map(|i| [f64::from(x[i]), f64::from(z[i]), f64::from(m[i])]).collect())
    }

    /// The left-hand side of the implicit scheme runs in f32: the f32 view of `a` (here `a` itself) and back.
    fn lhs(a: &A) -> Array<f32> {
        a.clone()
    }

    fn from_lhs(a: &Array<f32>) -> A {
        a.clone()
    }

    include!("common.rs");
    include!("body.rs");
    include!("body3.rs");
    include!("implicit_common.rs");
    include!("viscous3.rs");
    include!("turbulence3.rs");
    include!("rans3.rs");
    include!("implicit3.rs");
    include!("stages3.rs");
    include!("memory3.rs");
    include!("implicit.rs");
    include!("viscous.rs");
    include!("turbulence.rs");
    include!("control.rs");
}

/// Double-float state, residual and time step.
pub mod df32s {
    use luma::Df32;

    pub type S = Df32;

    fn c(x: f64) -> S {
        Df32::from(x)
    }

    fn up(ctx: &Context, data: &[f64], shape: impl Into<luma::Shape>) -> Result<A> {
        let v: Vec<Df32> = data.iter().map(|&x| Df32::from(x)).collect();
        Array::from_slice_on(ctx, &v, shape)
    }

    /// Measured fastest (README, speed table): `Fluxes` is 14 launches per step, `Fusion` (no `cache()`) 67-91.
    const DEFAULT_STAGING: Staging = Staging::Fluxes;

    fn to_f64(a: &A) -> Result<Vec<f64>> {
        Ok(a.to_vec()?.into_iter().map(|d| d.to_f64()).collect())
    }

    /// The density residual of every step, recorded on the GPU as two `f32` values: the history of `iterate_until_all` is
    /// `f32`, so the double-float residual `hi + lo` is recorded as its high word and its low word.
    fn records(until: luma::Until<'static, State>) -> luma::Until<'static, State> {
        until.record("residual", |s: &State| s[4].to_f32()).record("residual_lo", |s: &State| (&s[4] - s[4].to_f32().to_df32()).to_f32())
    }

    fn read_history(run: &luma::Run<State>) -> Result<Vec<f64>> {
        let missing = || luma::LumaError::Unsupported("the run recorded no residual history".into());
        let (hi, lo) = (run.history("residual").ok_or_else(missing)?, run.history("residual_lo").ok_or_else(missing)?);
        Ok(hi.iter().zip(lo).map(|(&h, &l)| f64::from(h) + f64::from(l)).collect())
    }

    /// The 3D counterpart of `records`: the double-float residual (`State3` index 5) as its high and low word; with `monitor`, the three force sums (`State3`
    /// index 10) as high and low words too.
    fn records3(until: luma::Until<'static, State3>, monitor: bool) -> luma::Until<'static, State3> {
        let mut until = until.record("residual", |s: &State3| s[5].to_f32()).record("residual_lo", |s: &State3| (&s[5] - s[5].to_f32().to_df32()).to_f32());
        if monitor {
            for (q, name) in ["fx", "fz", "mz"].into_iter().enumerate() {
                until = until
                    .record(name, move |s: &State3| s[10].slice(0, q..q + 1).to_f32())
                    .record(&format!("{name}_lo"), move |s: &State3| (&s[10].slice(0, q..q + 1) - s[10].slice(0, q..q + 1).to_f32().to_df32()).to_f32());
            }
        }
        until
    }

    fn read_history3(run: &luma::Run<State3>) -> Result<Vec<f64>> {
        let missing = || luma::LumaError::Unsupported("the run recorded no residual history".into());
        let (hi, lo) = (run.history("residual").ok_or_else(missing)?, run.history("residual_lo").ok_or_else(missing)?);
        Ok(hi.iter().zip(lo).map(|(&h, &l)| f64::from(h) + f64::from(l)).collect())
    }

    /// The force monitor of every step (`[Fx, Fz, Mz]`, see `records3`).
    fn read_forces3(run: &luma::Run<State3>) -> Result<Vec<[f64; 3]>> {
        let missing = || luma::LumaError::Unsupported("the run recorded no force history".into());
        let word = |name: &str| run.history(name).ok_or_else(missing);
        let (x, z, m) = (word("fx")?, word("fz")?, word("mz")?);
        let (xl, zl, ml) = (word("fx_lo")?, word("fz_lo")?, word("mz_lo")?);
        Ok((0..x.len()).map(|i| [f64::from(x[i]) + f64::from(xl[i]), f64::from(z[i]) + f64::from(zl[i]), f64::from(m[i]) + f64::from(ml[i])]).collect())
    }

    /// The left-hand side of the implicit scheme runs in f32: the f32 view of `a` and back.
    fn lhs(a: &A) -> Array<f32> {
        a.to_f32()
    }

    fn from_lhs(a: &Array<f32>) -> A {
        a.to_df32()
    }

    include!("common.rs");
    include!("body.rs");
    include!("body3.rs");
    include!("implicit_common.rs");
    include!("viscous3.rs");
    include!("turbulence3.rs");
    include!("rans3.rs");
    include!("implicit3.rs");
    include!("stages3.rs");
    include!("memory3.rs");
    include!("implicit.rs");
    include!("viscous.rs");
    include!("turbulence.rs");
    include!("control.rs");
}

enum Inner {
    F32(f32s::LazySolver),
    Ext(df32s::LazySolver),
}

/// Runs `$e` on the solver `$s` of whichever precision `$self` holds.
macro_rules! on {
    ($self:expr, $s:ident => $e:expr) => {
        match &$self.inner {
            Inner::F32($s) => $e,
            Inner::Ext($s) => $e,
        }
    };
}

macro_rules! on_mut {
    ($self:expr, $s:ident => $e:expr) => {
        match &mut $self.inner {
            Inner::F32($s) => $e,
            Inner::Ext($s) => $e,
        }
    };
}

/// GPU solver of one grid: [`f32s::LazySolver`] or [`df32s::LazySolver`] behind one type. Mirrors `crate::solver::Solver` for the
/// supported options (explicit SSP-RK3 or line-implicit scheme, solid block, low-Mach correction, viscous terms with the SST model).
pub struct GpuSolver {
    inner: Inner,
    ctx: Context,
    prec: Precision,
    pub ni: usize,
    pub nj: usize,
}

impl GpuSolver {
    /// Uploads the geometry (rounded from the grid's f64 metrics), starts from the free stream.
    pub fn new(ctx: &Context, grid: &Grid, cfg: &Config, prec: Precision) -> Result<GpuSolver> {
        let inner = match prec {
            Precision::F32 => Inner::F32(f32s::LazySolver::new(ctx, grid, cfg)?),
            Precision::Extended => Inner::Ext(df32s::LazySolver::new(ctx, grid, cfg)?),
        };
        Ok(GpuSolver { inner, ctx: ctx.clone(), prec, ni: grid.ni, nj: grid.nj })
    }

    /// The solver of a host `crate::solver::Solver`: its grid and config, low-Mach flag, solid block, scheme (`Scheme::Explicit` or
    /// `Scheme::Implicit`), conserved state, iteration count and, for a viscous host, the viscous parameters with the turbulence state and the
    /// wall distance (the SST state is copied from `host.turb` / `host.mu_t`).
    pub fn from_host(ctx: &Context, host: &Solver, prec: Precision) -> Result<GpuSolver> {
        if !matches!(host.scheme(), Scheme::Explicit | Scheme::Implicit(_)) {
            return Err(GpuError::Unsupported("only Scheme::Explicit and Scheme::Implicit are implemented on the GPU"));
        }
        let inner = match prec {
            Precision::F32 => Inner::F32(f32s::LazySolver::from_host(ctx, host)?),
            Precision::Extended => Inner::Ext(df32s::LazySolver::from_host(ctx, host)?),
        };
        Ok(GpuSolver { inner, ctx: ctx.clone(), prec, ni: host.grid.ni, nj: host.grid.nj })
    }

    pub fn precision(&self) -> Precision {
        self.prec
    }

    pub fn context(&self) -> &Context {
        &self.ctx
    }

    pub fn config(&self) -> &Config {
        on!(self, s => s.config())
    }

    /// Replaces the configuration: free stream, CFL, limiter, `first_order_iters` and the boundary conditions may change between runs
    /// (a Mach sweep) without touching the state. The next [`GpuSolver::run`] uses it (the fixed ghost states are uploaded again).
    pub fn set_config(&mut self, cfg: &Config) -> Result<()> {
        if self.viscous().is_some() && cfg.bc != self.config().bc {
            return Err(GpuError::Unsupported("the boundary conditions of a viscous run (no-slip walls, wall distance) are fixed by set_viscous"));
        }
        on_mut!(self, s => s.set_config(cfg)?);
        Ok(())
    }

    /// Iterations since the last [`GpuSolver::set_state`] / [`GpuSolver::reset`]; the first `Config::first_order_iters` are first order.
    pub fn iter(&self) -> usize {
        on!(self, s => s.iter)
    }

    pub fn set_iter(&mut self, iter: usize) {
        on_mut!(self, s => s.iter = iter)
    }

    /// Switches the low-Mach correction of the reconstructed face states (`crate::flux::low_mach_states`).
    pub fn set_low_mach(&mut self, on: bool) {
        on_mut!(self, s => s.set_low_mach(on))
    }

    /// Declares the solid body block of a base / wake grid (`crate::solver::Solver::set_block`).
    pub fn set_block(&mut self, block: Option<Block>) -> Result<()> {
        if self.viscous().is_some() {
            return Err(GpuError::Unsupported("the solid block must be declared before the viscous terms"));
        }
        on_mut!(self, s => s.set_block(block)?);
        Ok(())
    }

    pub fn block(&self) -> Option<Block> {
        on!(self, s => s.block())
    }

    /// Declares the nozzle exit on the base face (`crate::solver::Solver::set_nozzle`): rows `0..j_end` of the face `i_end` are a fixed
    /// inflow state. Must follow [`GpuSolver::set_block`] and precede [`GpuSolver::set_viscous`].
    pub fn set_nozzle(&mut self, nozzle: Option<Nozzle>) -> Result<()> {
        if self.viscous().is_some() || (nozzle.is_some() && self.block().is_none()) {
            return Err(GpuError::Unsupported("set_nozzle must follow set_block and precede set_viscous"));
        }
        on_mut!(self, s => s.set_nozzle(nozzle)?);
        Ok(())
    }

    /// The nozzle exit on the base face.
    pub fn nozzle(&self) -> Option<Nozzle> {
        on!(self, s => s.nozzle())
    }

    /// Selects the convergence scheme: `Scheme::Explicit` or `Scheme::Implicit` (the CFL ramp restarts); multigrid and residual
    /// smoothing are not ported.
    pub fn set_scheme(&mut self, scheme: Scheme) -> Result<()> {
        if !matches!(scheme, Scheme::Explicit | Scheme::Implicit(_)) {
            return Err(GpuError::Unsupported("only Scheme::Explicit and Scheme::Implicit are implemented on the GPU"));
        }
        on_mut!(self, s => s.set_scheme(scheme)?);
        Ok(())
    }

    pub fn scheme(&self) -> Scheme {
        on!(self, s => s.scheme())
    }

    /// Restarts the CFL ramp of the implicit scheme (at the start of a run and after a change of the free stream).
    pub fn restart_ramp(&mut self) -> Result<()> {
        on_mut!(self, s => s.restart_ramp()?);
        Ok(())
    }

    /// Switches the viscous terms on (`Some`, with the SST model if `v.turbulence` says so) or off (`None`, the Euler equations). `grid` is the grid
    /// of the solver; declare the solid block first. The turbulence state restarts at the free stream, the mean flow is not touched.
    pub fn set_viscous(&mut self, v: Option<Viscous>, grid: &Grid) -> Result<()> {
        if (grid.ni, grid.nj) != (self.ni, self.nj) {
            return Err(GpuError::Unsupported("set_viscous: the grid is not the grid of the solver"));
        }
        on_mut!(self, s => s.set_viscous(v, grid)?);
        Ok(())
    }

    /// The viscous-flow parameters, `None` for the Euler equations.
    pub fn viscous(&self) -> Option<Viscous> {
        on!(self, s => s.viscous())
    }

    /// `([k, omega], mu_t)` of every cell of an SST run (`Solver::turb`, `Solver::mu_t`), as f64.
    pub fn turb_state(&self) -> Result<(Vec<[f64; 2]>, Vec<f64>)> {
        if !self.viscous().is_some_and(|v| v.turbulence == crate::sst::Turbulence::Sst) {
            return Err(GpuError::Unsupported("turb_state needs the SST model"));
        }
        Ok(on!(self, s => s.turb_state_f64()?))
    }

    /// The density residual norm of `k` of the last turbulence update (`Solver::turb_residual`; 0 before the first).
    pub fn turb_residual(&self) -> Result<f64> {
        Ok(on!(self, s => s.turb_residual()?))
    }

    /// Overwrites the conserved state (e.g. with a CPU solution) and restarts the iteration counter and the CFL ramp.
    pub fn set_state(&mut self, u: &[[f64; 4]]) -> Result<()> {
        on_mut!(self, s => s.set_state(u)?);
        Ok(())
    }

    /// Resets to the free stream of the current configuration at iteration 0, the implicit CFL ramp restarted.
    pub fn reset(&mut self) -> Result<()> {
        let fs = crate::gas::prim_to_cons(&self.config().freestream);
        self.set_state(&vec![fs; self.ni * self.nj])
    }

    /// Conserved state, `ni * nj`, same layout as `crate::solver::Solver::u`.
    pub fn state_f64(&self) -> Result<Vec<[f64; 4]>> {
        Ok(on!(self, s => s.state_f64()?))
    }

    /// Runs `n` iterations and returns the density residual of every one (L2 of `d rho / dt` over `sqrt(N)` of the fluid cells, as in
    /// the CPU solver). The residual is recorded on the GPU, the host waits once per `RUN_SYNC` iterations and at the end.
    pub fn run(&mut self, n: usize) -> Result<Vec<f64>> {
        Ok(on_mut!(self, s => s.run(n, RUN_SYNC.min(n.max(1)))?))
    }

    /// Conserved state of the two first cells behind the base face, `[(i_end, j), (i_end + 1, j)]`, for every row `j_sting..j_end` (the
    /// extrapolation stencil of `bowshock_core::engine::run::base_state`). Reads back only these rows.
    pub fn base_cells(&self) -> Result<Vec<[[f64; 4]; 2]>> {
        if self.block().is_none() {
            return Err(GpuError::Unsupported("base_cells needs a solid block"));
        }
        Ok(on!(self, s => s.base_cells()?))
    }
}

/// One wall face of the device force integration: the class (an index chosen by the caller), the first fluid cell `a` and the fluid cell `b` behind it
/// (`Grid3::cell` indices), `k = d0 / (d1 - d0)` of the extrapolation `p_w = p_a + k (p_a - p_b)` (`d` the distances of the cell centroids to the face centre),
/// the weights `cx = w n_x`, `cz = w n_z`, `cm = w n_z c_x` with `w = area * sign` (`Fx = sum (p_w - p_inf) cx`, `Fz`, `Mz` likewise), and whether the
/// face counts for the monitor's cut (`c_x <= x_max`, or the base).
#[derive(Clone, Copy, Debug)]
pub struct WallFace3 {
    pub class: usize,
    pub a: usize,
    pub b: Option<usize>,
    pub k: f64,
    pub cx: f64,
    pub cz: f64,
    pub cm: f64,
    pub cut: bool,
}

/// One no-slip face of the viscous force reduction: the class (an index chosen by the caller), the family `dir` of the face (0: i, 1: j, 2: k), its flat index `q`
/// in the family layout (`viscous3::face_of`), `sigma` (+1 where the fluid cell is the lower side: `viscous3::wall_tractions`), the area and the abscissa of the
/// centroid.
#[derive(Clone, Copy, Debug)]
pub struct TractionFace3 {
    pub class: usize,
    pub dir: usize,
    pub q: usize,
    pub sigma: f64,
    pub area: f64,
    pub cx: f64,
}

/// Everything the viscous kernels produce on a padded host state, downloaded (`LazySolver3::viscous_probe3`): the cell gradients and viscosity, the viscous face
/// fluxes `Fv.n` of the three families (mass component 0), the diffusive spectral radius of every face, the diffusion rates and the viscous residual of every
/// cell, and the viscous wall forces by class (`[Fx, Fz, Mz]`, empty without traction faces).
#[derive(Clone, Debug)]
pub struct VisProbe {
    pub grad: Vec<[f64; 12]>,
    pub mu: Vec<f64>,
    pub fv: [Vec<[f64; 5]>; 3],
    pub lambda: [Vec<f64>; 3],
    pub dr: Vec<[f64; 3]>,
    pub residual: Vec<[f64; 5]>,
    pub tractions: Vec<[f64; 3]>,
}

/// A copy of the loop state of a [`GpuSolver3`] at one iteration ([`GpuSolver3::snapshot`]), in host memory.
#[derive(Clone)]
pub struct Snapshot3(SnapInner);

#[derive(Clone)]
enum SnapInner {
    F32(f32s::Snapshot3),
    Ext(df32s::Snapshot3),
}

enum Inner3 {
    F32(f32s::LazySolver3),
    Ext(df32s::LazySolver3),
}

macro_rules! on3 {
    ($self:expr, $s:ident => $e:expr) => {
        match &$self.inner {
            Inner3::F32($s) => $e,
            Inner3::Ext($s) => $e,
        }
    };
}

macro_rules! on3_mut {
    ($self:expr, $s:ident => $e:expr) => {
        match &mut $self.inner {
            Inner3::F32($s) => $e,
            Inner3::Ext($s) => $e,
        }
    };
}

/// GPU solver of one 3D grid: [`f32s::LazySolver3`] or [`df32s::LazySolver3`] behind one type. Mirrors `crate::solver3::Solver3` (SSP-RK3 with local
/// time stepping or the three-factor line-implicit scheme, characteristic far field, axis, slip wall, symmetry planes, fin solid boxes, low-Mach
/// correction). The left-hand side of the implicit scheme is f32 in both precisions.
pub struct GpuSolver3 {
    inner: Inner3,
    ctx: Context,
    prec: Precision,
    pub ni: usize,
    pub nj: usize,
    pub nk: usize,
}

impl GpuSolver3 {
    /// Uploads the geometry (rounded from the grid's f64 metrics), starts from the free stream.
    pub fn new(ctx: &Context, grid: &Grid3, cfg: &Config3, prec: Precision) -> Result<GpuSolver3> {
        let inner = match prec {
            Precision::F32 => Inner3::F32(f32s::LazySolver3::new(ctx, grid, cfg)?),
            Precision::Extended => Inner3::Ext(df32s::LazySolver3::new(ctx, grid, cfg)?),
        };
        Ok(GpuSolver3 { inner, ctx: ctx.clone(), prec, ni: grid.ni, nj: grid.nj, nk: grid.nk })
    }

    /// The solver of a host [`Solver3`]: its grid and config, conserved state and iteration count.
    pub fn from_host(ctx: &Context, host: &Solver3, prec: Precision) -> Result<GpuSolver3> {
        let inner = match prec {
            Precision::F32 => Inner3::F32(f32s::LazySolver3::from_host(ctx, host)?),
            Precision::Extended => Inner3::Ext(df32s::LazySolver3::from_host(ctx, host)?),
        };
        Ok(GpuSolver3 { inner, ctx: ctx.clone(), prec, ni: host.grid.ni, nj: host.grid.nj, nk: host.grid.nk })
    }

    pub fn precision(&self) -> Precision {
        self.prec
    }

    pub fn context(&self) -> &Context {
        &self.ctx
    }

    pub fn config(&self) -> &Config3 {
        on3!(self, s => s.config())
    }

    /// Replaces the configuration: free stream, CFL, limiter, `first_order_iters` and the low-Mach flag may change between runs (a Mach / incidence
    /// sweep) without touching the state or the geometry. Changing the scheme restarts the CFL ramp.
    pub fn set_config(&mut self, cfg: &Config3) -> Result<()> {
        on3_mut!(self, s => s.set_config(cfg)?);
        Ok(())
    }

    /// Iterations since the last [`GpuSolver3::set_state`] / [`GpuSolver3::reset`]; the first `Config3::first_order_iters` are first order.
    pub fn iter(&self) -> usize {
        on3!(self, s => s.iter)
    }

    pub fn set_iter(&mut self, iter: usize) {
        on3_mut!(self, s => s.iter = iter)
    }

    /// Restarts the CFL ramp (the explicit scheme has a constant CFL: nothing changes until E2).
    pub fn restart_ramp(&mut self) -> Result<()> {
        on3_mut!(self, s => s.restart_ramp()?);
        Ok(())
    }

    /// The four parts of wedge mode one by one: `(skip_k, no_dt_k, k_wall_flux, k_src)` (experiments). They are all on for a grid with `Symmetry::Wedge`, as on the
    /// CPU (`Solver3::is_wedge`): no k factor, no `L_k` in the pseudo-time step, the pressure of the cell as the flux through the two k planes, and the 2D-style
    /// pressure-source block on the diagonal of the j factor.
    pub fn set_wedge_parts(&mut self, skip_k: bool, no_dt_k: bool, k_wall_flux: bool, k_src: bool) {
        on3_mut!(self, s => (s.skip_k, s.no_dt_k, s.k_wall_flux, s.k_src) = (skip_k, no_dt_k, k_wall_flux, k_src))
    }

    /// Whether all four parts of wedge mode are on.
    pub fn wedge_mode(&self) -> bool {
        on3!(self, s => s.skip_k && s.no_dt_k && s.k_wall_flux && s.k_src)
    }

    /// Overwrites the conserved state (e.g. with a CPU solution) and restarts the iteration counter and the CFL ramp.
    pub fn set_state(&mut self, u: &[Cons3]) -> Result<()> {
        on3_mut!(self, s => s.set_state(u)?);
        Ok(())
    }

    /// [`GpuSolver3::set_state`] with the `[k, omega]` of every cell (`Some`; SST runs only, `mu_t = rho k / omega` capped as the host init): a warm start from another field.
    pub fn set_state_turb(&mut self, u: &[Cons3], turb: Option<&[[f64; 2]]>) -> Result<()> {
        on3_mut!(self, s => s.set_state_turb(u, turb)?);
        Ok(())
    }

    /// A copy of the loop state, downloaded to host memory, to return to with [`GpuSolver3::restore`] (bit for bit): the CFL controller of the 3D runner.
    pub fn snapshot(&self) -> Result<Snapshot3> {
        Ok(Snapshot3(match &self.inner {
            Inner3::F32(s) => SnapInner::F32(s.snapshot()?),
            Inner3::Ext(s) => SnapInner::Ext(s.snapshot()?),
        }))
    }

    /// Returns to a [`Snapshot3`] of this solver (state, CFL ramp value, iteration count); with `cfl_cap` the CFL of the restored state is at most that. Set the new
    /// configuration ([`GpuSolver3::set_config`]) first: a changed scheme restarts the ramp.
    pub fn restore(&mut self, snap: &Snapshot3, cfl_cap: Option<f64>) -> Result<()> {
        match (&mut self.inner, &snap.0) {
            (Inner3::F32(s), SnapInner::F32(n)) => s.restore(n, cfl_cap)?,
            (Inner3::Ext(s), SnapInner::Ext(n)) => s.restore(n, cfl_cap)?,
            _ => return Err(GpuError::Unsupported("a snapshot of the other precision")),
        }
        Ok(())
    }

    /// Resets to the free stream of the current configuration at iteration 0, the CFL ramp restarted.
    pub fn reset(&mut self) -> Result<()> {
        let fs = crate::gas3::prim_to_cons3(&self.config().freestream);
        self.set_state(&vec![fs; self.ni * self.nj * self.nk])
    }

    /// Conserved state, `ni * nj * nk`, same layout as `crate::solver3::Solver3::u` (`Grid3::cell`).
    pub fn state_f64(&self) -> Result<Vec<Cons3>> {
        Ok(on3!(self, s => s.state_f64()?))
    }

    /// Runs `n` iterations and returns the density residual of every one (L2 of `d rho / dt` over `sqrt(N)` of the fluid cells, as in the CPU
    /// solver). The residual is recorded on the GPU, the host waits once per `RUN_SYNC` iterations and at the end.
    pub fn run(&mut self, n: usize) -> Result<Vec<f64>> {
        Ok(on3_mut!(self, s => s.run(n, RUN_SYNC.min(n.max(1)))?))
    }

    /// [`GpuSolver3::run`] with the force monitor of every iteration (`[Fx, Fz, Mz]` of the classes of [`GpuSolver3::set_force_monitor`]; entry `i` is the state the
    /// step `i` started from).
    pub fn run_monitored(&mut self, n: usize) -> Result<(Vec<f64>, Vec<[f64; 3]>)> {
        Ok(on3_mut!(self, s => s.run_monitored(n, RUN_SYNC.min(n.max(1)))?))
    }

    /// The force monitor of the loop: see [`LazySolver3::set_force_monitor`] (`faces` / `tfaces` as given to `set_wall_faces` / `set_traction_faces`).
    pub fn set_force_monitor(&mut self, faces: &[WallFace3], tfaces: &[TractionFace3], keep: &[bool]) -> Result<()> {
        on3_mut!(self, s => s.set_force_monitor(faces, tfaces, keep)?);
        Ok(())
    }

    /// GPU time per kernel of one full SSP-RK3 step as the `eval` loop launches it, and where fusion cut the expressions (`Profile::launches`,
    /// `Profile::total`).
    pub fn profile_step(&mut self) -> Result<luma::Profile> {
        Ok(on3_mut!(self, s => s.profile_step()?))
    }

    /// Switches the 3D viscous kernels on (the host geometry of the face passes: `grid`, the symmetry sides of the domain box, the isothermal wall condition, `None`: adiabatic); see `LazySolver3::set_viscous3`.
    pub fn set_viscous(&mut self, grid: &Grid3, v: crate::sst::Viscous, sym: &crate::viscous3::SymSides, iso: Option<crate::sst::IsoWall>) -> Result<()> {
        on3_mut!(self, s => s.set_viscous3(grid, v, sym, iso)?);
        Ok(())
    }

    /// Launches and GPU time of one evaluation of the viscous residual of the current state (kernels alone); needs [`GpuSolver3::set_viscous`].
    pub fn profile_viscous(&mut self) -> Result<luma::Profile> {
        Ok(on3_mut!(self, s => s.profile_viscous3()?))
    }

    /// The stages of one implicit block step and their GPU profiles (best of `reps`), the state unchanged: see `stages3.rs`.
    pub fn profile_stages(&mut self, reps: usize) -> Result<Vec<(String, luma::Profile)>> {
        Ok(on3_mut!(self, s => s.profile_stages3(reps)?))
    }

    /// The device bytes of the arrays the solver holds, `(group, bytes)` (geometry by family, the f32 left-hand-side geometry, the viscous and SST data, the force
    /// arrays, one set of the loop state): see `memory3.rs`. The transients of the step are not in it: the process peak minus the sum of these.
    pub fn memory_report(&self) -> Vec<(&'static str, usize)> {
        on3!(self, s => s.memory_report())
    }

    /// Are the viscous terms on?
    pub fn has_viscous(&self) -> bool {
        on3!(self, s => s.has_viscous())
    }

    /// Sets the motor jet of the run (the solver was built from a host that has one: `Solver3::set_jet`); the jet cells of the current state take it. A warm run at another Mach
    /// number sets its jet this way.
    pub fn set_jet(&mut self, jet: crate::solver3::JetState3) -> Result<()> {
        on3_mut!(self, s => s.set_jet(jet)?);
        Ok(())
    }

    /// The viscous face fluxes of the current state on the host (`None` for the Euler equations): the input of `viscous3::wall_tractions`.
    pub fn viscous_flux(&self) -> Result<Option<[Vec<[f64; 5]>; 3]>> {
        if !on3!(self, s => s.has_viscous()) {
            return Ok(None);
        }
        Ok(Some(on3!(self, s => s.viscous_flux_f64()?)))
    }

    /// `[k, omega]` and `mu_t` of every cell on the host (`None` without SST).
    pub fn turb_state(&self) -> Result<Option<(Vec<[f64; 2]>, Vec<f64>)>> {
        Ok(on3!(self, s => s.turb_f64()?))
    }

    /// Uploads the no-slip faces of the viscous force reduction (see [`TractionFace3`]).
    pub fn set_traction_faces(&mut self, faces: &[TractionFace3], n_classes: usize) -> Result<()> {
        on3_mut!(self, s => s.set_traction_faces(faces, n_classes)?);
        Ok(())
    }

    /// The viscous wall force by class `[Fx, Fz, Mz]` of the current state, reduced on the device.
    pub fn viscous_wall_forces(&self) -> Result<Vec<[f64; 3]>> {
        Ok(on3!(self, s => s.viscous_wall_forces()?))
    }

    /// GPU time per kernel of `n` iterations of the `iterate_all` replay (the profile adds up all `n`).
    pub fn profile_replay(&mut self, n: usize) -> Result<luma::Profile> {
        Ok(on3_mut!(self, s => s.profile_replay(n)?))
    }

    /// Host seconds to trace the lazy expression of one step `n` times, nothing evaluated.
    pub fn trace_step_seconds(&self, n: usize) -> f64 {
        on3!(self, s => s.trace_step_seconds(n))
    }

    /// `n` iterations calling the step every iteration and evaluating the new state (no replay): the loop a host-driven solver would run.
    pub fn run_eval(&mut self, n: usize) -> Result<()> {
        on3_mut!(self, s => {
            s.run_eval(n)?;
        });
        Ok(())
    }

    /// `n` iterations as `iterate_all` replays (no residual history: what a benchmark times); call [`GpuSolver3::sync`] to wait for the GPU.
    pub fn run_replay(&mut self, n: usize) -> Result<()> {
        on3_mut!(self, s => s.run_replay(n)?);
        Ok(())
    }

    /// Waits for the GPU.
    pub fn sync(&self) -> Result<()> {
        on3!(self, s => s.sync()?);
        Ok(())
    }

    /// Uploads the wall faces of the on-device force integration (`n_classes` classes): see [`WallFace3`].
    pub fn set_wall_faces(&mut self, faces: &[WallFace3], n_classes: usize) -> Result<()> {
        on3_mut!(self, s => s.set_wall_faces(faces, n_classes)?);
        Ok(())
    }

    /// `[rho_min, p_min]` of the state over every cell, reduced on the device (see `LazySolver3::state_extrema`): the CFL controller's health check.
    pub fn state_extrema(&self) -> Result<[f64; 2]> {
        Ok(on3!(self, s => s.state_extrema()?))
    }

    /// Pressure forces `[Fx, Fz, Mz]` per class reduced on the device (`2 n_classes` triples: all faces, then the faces flagged `cut`); no state download.
    pub fn wall_forces(&self, p_inf: f64) -> Result<Vec<[f64; 3]>> {
        Ok(on3!(self, s => s.wall_forces(p_inf)?))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const GIB: f64 = (1u64 << 30) as f64;

    /// The estimate is linear in the cells and reproduces the measured peaks of item 49 from above: the Finner sector (688 k cells, 2.54 GiB), refinement 2.0 (4.8 M, 17.9 GiB) and
    /// 2.25 (6.8 M, 21.9 GiB) in f32 with SST, and orders the settings: more `sub_iters`, double-float state and the SST terms cost, Euler saves.
    #[test]
    fn the_device_estimate_bounds_the_measured_peaks() {
        let est = |cells: usize, prec, sub, rans| device_bytes3(cells, prec, sub, rans) as f64 / GIB;
        for (cells, peak_gib) in
            [(687_960usize, 2.54 - 0.25), (1_350_048, 4.48 - 0.25), (2_155_608, 8.2 - 0.25), (4_823_000, 17.87 - 0.25), (6_802_432, 21.86 - 0.25)]
        {
            assert!(est(cells, Precision::F32, 2, true) >= peak_gib, "{cells} cells need {peak_gib} GiB, estimated {}", est(cells, Precision::F32, 2, true));
            assert!(est(cells, Precision::F32, 2, true) < peak_gib * 1.4 + 0.5, "{cells} cells: the estimate is not far above the measurement");
        }
        let (a, b) = (device_bytes3(1_000_000, Precision::F32, 2, true), device_bytes3(2_000_000, Precision::F32, 2, true));
        assert_eq!(b - a, a - CONTEXT_BYTES3);
        assert!(device_bytes3(1_000_000, Precision::F32, 5, true) > a);
        assert!(device_bytes3(1_000_000, Precision::Extended, 2, true) > a);
        assert!(device_bytes3(1_000_000, Precision::F32, 2, false) < a);
        assert_eq!(device_bytes3(1_000_000, Precision::F32, 1, true), a);
    }
}
