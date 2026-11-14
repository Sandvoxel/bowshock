//! `bowshock-gpu`: the axisymmetric Euler solver of `bowshock-core` (MUSCL + HLLC / Rusanov shock blend + optional low-Mach
//! correction, characteristic and pressure-outflow far fields, SSP-RK3 with local time stepping or the line-implicit scheme, optional
//! solid body block) written in plain Luma array ops: no custom kernels. Luma's lazy arrays fuse the expressions; `iterate_until_all`
//! traces one step and replays its kernels, recording the density residual on the GPU.
//!
//! The solver is in `body.rs` (explicit scheme) and `implicit.rs` (line-implicit scheme), each function the mirror of the CPU routine of the
//! same name in `bowshock-core`, included once per precision:
//!
//! * [`f32s`]: `Array<f32>` everywhere ([`Precision::F32`]).
//! * [`df32s`]: the state, the residual and the time step in double-float `Array<Df32>` ([`Precision::Extended`]); the
//!   state is a `Df32` record through the loops.
//!
//! The line-implicit scheme's left-hand side is f32 in both. The viscous terms of the compressible Navier-Stokes equations (`viscous.rs`) and the
//! Menter SST k-omega model (`turbulence.rs`) follow, for both schemes and for grids with a solid block; their left-hand sides (the `2 lam I` of
//! the implicit operator, the line solves of the two turbulence scalars) are f32 like the rest. [`GpuSolver`] is the precision-independent handle
//! the other crates use. See the README.

use bowshock_core::grid::Grid;
use bowshock_core::solver::{Block, Config, Scheme, Solver, Viscous};
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

    fn read_history(run: &luma::Run<State>) -> Vec<f64> {
        run.history("residual").expect("residual history").iter().map(|&r| f64::from(r)).collect()
    }

    /// The left-hand side of the implicit scheme runs in f32: the f32 view of `a` (here `a` itself) and back.
    fn lhs(a: &A) -> Array<f32> {
        a.clone()
    }

    fn from_lhs(a: &Array<f32>) -> A {
        a.clone()
    }

    /// `tanh`: the blending functions of the SST model.
    fn tanh(a: &A) -> A {
        a.tanh()
    }

    include!("body.rs");
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

    fn read_history(run: &luma::Run<State>) -> Vec<f64> {
        let (hi, lo) = (run.history("residual").expect("residual history"), run.history("residual_lo").expect("residual history"));
        hi.iter().zip(lo).map(|(&h, &l)| f64::from(h) + f64::from(l)).collect()
    }

    /// The left-hand side of the implicit scheme runs in f32: the f32 view of `a` and back.
    fn lhs(a: &A) -> Array<f32> {
        a.to_f32()
    }

    fn from_lhs(a: &Array<f32>) -> A {
        a.to_df32()
    }

    /// `tanh` of the blending functions of the SST model, in f32 (`Df32` arrays have no `tanh`; the blending functions need no more).
    fn tanh(a: &A) -> A {
        a.to_f32().tanh().to_df32()
    }

    include!("body.rs");
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

/// GPU solver of one grid: [`f32s::LazySolver`] or [`df32s::LazySolver`] behind one type. Mirrors `bowshock_core::solver::Solver` for the
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

    /// The solver of a host `bowshock_core` solver: its grid and config, low-Mach flag, solid block, scheme (`Scheme::Explicit` or
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

    /// Switches the low-Mach correction of the reconstructed face states (`bowshock_core::flux::low_mach_states`).
    pub fn set_low_mach(&mut self, on: bool) {
        on_mut!(self, s => s.set_low_mach(on))
    }

    /// Declares the solid body block of a base / wake grid (`bowshock_core::solver::Solver::set_block`).
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
    pub fn restart_ramp(&mut self) {
        on_mut!(self, s => s.restart_ramp())
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
        if !self.viscous().is_some_and(|v| v.turbulence == bowshock_core::solver::Turbulence::Sst) {
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
        let fs = bowshock_core::gas::prim_to_cons(&self.config().freestream);
        self.set_state(&vec![fs; self.ni * self.nj])
    }

    /// Conserved state, `ni * nj`, same layout as `bowshock_core::solver::Solver::u`.
    pub fn state_f64(&self) -> Result<Vec<[f64; 4]>> {
        Ok(on!(self, s => s.state_f64()?))
    }

    /// Runs `n` iterations and returns the density residual of every one (L2 of `d rho / dt` over `sqrt(N)` of the fluid cells, as in
    /// `bowshock-core`). The residual is recorded on the GPU, the host waits once per [`RUN_SYNC`] iterations and at the end.
    pub fn run(&mut self, n: usize) -> Result<Vec<f64>> {
        Ok(on_mut!(self, s => s.run(n, RUN_SYNC.min(n.max(1)))?))
    }

    /// Conserved state of the two first cells behind the base face, `[(i_end, j), (i_end + 1, j)]`, for every row `j_sting..j_end` (the
    /// extrapolation stencil of `bowshock::run::base_state`). Reads back only these rows.
    pub fn base_cells(&self) -> Result<Vec<[[f64; 4]; 2]>> {
        if self.block().is_none() {
            return Err(GpuError::Unsupported("base_cells needs a solid block"));
        }
        Ok(on!(self, s => s.base_cells()?))
    }
}
