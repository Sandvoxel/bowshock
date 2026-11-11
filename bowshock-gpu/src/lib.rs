//! `bowshock-gpu`: f32 port of the `bowshock-core` axisymmetric Euler solver (MUSCL + HLLC/Rusanov shock blend +
//! SSP-RK3 with local time stepping) to Luma custom WGSL kernels.
//!
//! Every kernel in [`shaders`] is a direct translation of the corresponding `bowshock-core` function or
//! loop body, with the same data layout (cell-centred conserved state, primitives with two ghost
//! layers, per-face HLLC flux, per-cell residual) so that differences are traceable. Everything is
//! recorded into Luma's automatic batch; the host only synchronises when it reads the residual
//! history (every `K` iterations) or the state.
//!
//! The state is stored in f32 arrays in every case, but [`Precision`] selects how it is accumulated and how the
//! residual is evaluated: plain f32 (the original port, whose residual stalls 4 - 4.6 orders below its peak because of
//! f32 rounding), a compensated `hi + lo` state, or (default) the compensated state plus a double-float evaluation of
//! the residual ([`shaders_ext`]), which converges like the f64 CPU solver. See the README, "The f32 residual floor".
//!
//! With [`GpuSolver::set_viscous`] the line-implicit scheme also runs the viscous terms and the Menter SST k-omega model of
//! `bowshock_core::viscous` / `bowshock_core::turbulence` ([`shaders_visc`]): plain f32 kernels, one per CPU routine, validated against the CPU
//! reference (README, "Viscous terms and the SST model").

pub mod shaders;
pub mod shaders_ext;
pub mod shaders_impl;
pub mod shaders_visc;

use bowshock_core::flux::Limiter;
use bowshock_core::gas::prim_to_cons;
use bowshock_core::grid::Grid;
use bowshock_core::implicit::{DtMode, ImplicitParams};
use bowshock_core::solver::{Bc, Block, Config, Scheme, Solver, Turbulence, Viscous};
use luma::{args, uniform, Array, Context, Kernel};

pub use luma::LumaError;

/// Errors of the GPU solver: a Luma/wgpu failure or a configuration the GPU port does not implement.
#[derive(Debug, Clone)]
pub enum GpuError {
    /// Failure reported by Luma (adapter, device, kernel build, buffer transfer).
    Luma(LumaError),
    /// The requested mode / scheme / block combination is not implemented on the GPU.
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

/// SSP-RK3 (Shu-Osher) stage coefficients `(a, b)` of `u = a (u + dt L(u)) + b u0`, as in
/// `bowshock_core::solver::Solver::step`.
const RK3: [(f32, f32); 3] = [(1.0, 0.0), (0.25, 0.75), ((2.0f64 / 3.0) as f32, (1.0f64 / 3.0) as f32)];

/// Maximum number of iterations recorded between two residual-history reads.
pub const MAX_CHUNK: usize = 256;

/// Declares [`Params`] and its WGSL twin [`PARAMS_WGSL`] from one field list, so the two cannot drift apart.
macro_rules! params {
    ($($(#[$m:meta])* $name:ident : $ty:ident),* $(,)?) => {
        /// Uniform block of every kernel (see `PARAMS_WGSL`, emitted by [`shaders::common`]).
        #[derive(encase::ShaderType, Clone, Copy, Debug, Default)]
        pub struct Params {
            $($(#[$m])* pub $name: $ty,)*
        }

        /// WGSL declaration of [`Params`].
        pub const PARAMS_WGSL: &str = concat!("struct Params {\n", $("    ", stringify!($name), ": ", stringify!($ty), ",\n",)* "}\n");
    };
}

params! {
    ni: u32,
    nj: u32,
    limiter: u32,
    stage: u32,
    cfl: f32,
    fs_rho: f32,
    fs_u: f32,
    fs_v: f32,
    fs_p: f32,
    bc0: u32,
    bc1: u32,
    bc2: u32,
    bc3: u32,
    slot: u32,
    aux: u32,
    /// Always 0. The WGSL error-free transformations read it (`opq`) so that the driver's compiler cannot
    /// reassociate `(a + b) - a` to `b` (see `shaders::common`).
    zero: u32,
    /// RK stage update `u = ra (u + dt res) + rb u0` (stages 2, 3).
    ra: f32,
    rb: f32,
    /// 1: Thornber low-Mach correction of the reconstructed face states (`bowshock_core::flux::low_mach_states`).
    low_mach: u32,
    /// Line-implicit scheme flags: bit 1 block-tridiagonal lines (else point-implicit), 2 implicit BC Jacobians, 4 Roe-type
    /// dissipation matrix (else Rusanov `lambda I`), 8 `DtMode::MinDirection`.
    iflags: u32,
    /// Low parts of the free stream `(rho, u, v, p)` (f64 minus its f32 rounding), used by the extended precision.
    fs_lo_rho: f32,
    fs_lo_u: f32,
    fs_lo_v: f32,
    fs_lo_p: f32,
    /// Solid body block `(i_end, j_end, j_sting)` of `bowshock_core::solver::Block` (all 0: none).
    blk_i: u32,
    blk_j: u32,
    blk_s: u32,
    /// Viscous run: first column `i` of the no-slip surface (`bowshock_core::solver::Viscous::wall_i_start`).
    wall_i0: u32,
    /// Pseudo-time CFL of the implicit scheme and the entropy-fix level of its Roe-type dissipation matrix.
    icfl: f32,
    eps_fix: f32,
    /// Viscous run flags: bit 1 viscous terms (no-slip walls, `shaders_visc`), bit 2 SST turbulence model.
    vflags: u32,
    ipad2: u32,
    /// Viscous run: `1 / re_c` (`bowshock_core::solver::Viscous::re_c`), the Sutherland constant and the free-stream `k` and
    /// `omega` of the SST model.
    mu_inf: f32,
    suth: f32,
    k_inf: f32,
    om_inf: f32,
}

pub fn limiter_code(l: Limiter) -> u32 {
    match l {
        Limiter::FirstOrder => 0,
        Limiter::Minmod => 1,
        Limiter::VanAlbada => 2,
    }
}

fn bc_code(b: Bc) -> u32 {
    match b {
        Bc::Freestream => 0,
        Bc::Extrapolate => 1,
        Bc::SlipWall => 2,
        Bc::Characteristic => 3,
        Bc::PressureOutflow => 4,
    }
}

fn f32x4(v: [f64; 4]) -> [f32; 4] {
    [v[0] as f32, v[1] as f32, v[2] as f32, v[3] as f32]
}

/// Number of partial sums of the residual norm: workgroups of the fused residual kernel (guarded
/// mode) or 128 (direct mode), whichever is larger.
fn n_partial(ni: usize, nj: usize) -> usize {
    (ni.div_ceil(32) * nj.div_ceil(4)).max(128)
}

fn build(ctx: &Context, src: &str, label: &str) -> Result<Kernel> {
    Ok(ctx.kernel(&format!("{}\n{}", shaders::common(), src)).label(label).build()?)
}

fn build_ext(ctx: &Context, src: &str, label: &str) -> Result<Kernel> {
    Ok(ctx.kernel(&format!("{}\n{}", shaders_ext::common_ext(), src)).label(label).build()?)
}

/// How one iteration is mapped to kernel launches.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Mode {
    /// Direct translation of bowshock-core: cons->prim, ghost fill, i-face flux, j-face flux, residual,
    /// update per RK stage (17 launches per iteration incl. residual history).
    Direct,
    /// Index-guarded formulation: no primitive/ghost arrays; one flux kernel (i and j faces, ghost
    /// states evaluated lazily) and one fused residual+update kernel per RK stage (7 launches per
    /// iteration incl. residual history).
    Guarded,
}

/// How the conserved state is stored, updated and evaluated on the GPU. The state always lives in f32 arrays; the
/// variants differ in how many of them hold it and in the precision of the residual evaluation.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum Precision {
    /// One f32 per component; the RK stage update `u = a (u + dt res) + b u0` is rounded to f32 and the residual is
    /// evaluated in f32 (the original port).
    F32,
    /// The state is a pair `hi + lo` of f32 arrays (double-float); the update is accumulated in increment form with
    /// an error-free addition (see `shaders::update`), so increments below one ulp of `u` are not lost. The residual
    /// is still evaluated in f32 on the high part.
    Compensated,
    /// Compensated state plus a double-float (~48 bit) evaluation of the whole residual: primitive variables, MUSCL
    /// reconstruction, HLLC / Rusanov blend, shock sensor and the cell sum (`shaders_ext`).
    #[default]
    Extended,
}

/// Compiled kernels of the solver.
enum Kernels {
    Direct {
        prim: Kernel,
        ghost: Kernel,
        flux_i: Kernel,
        flux_j: Kernel,
        residual: Kernel,
        update: Kernel,
        hist_partial: Kernel,
        hist_final: Kernel,
    },
    /// Also used by the extended precision (different sources, same launch pattern; `fi`/`fj` are twice as long).
    Guarded {
        flux: Kernel,
        residual_update: Kernel,
        hist_final: Kernel,
    },
}

/// GPU solver state. Created from a `bowshock-core` grid and config (geometry is converted to f32 and
/// uploaded once).
pub struct GpuSolver {
    ctx: Context,
    pub ni: usize,
    pub nj: usize,
    cfg: Config,
    mode: Mode,
    k: Kernels,
    /// Conserved state and its copy at the start of the RK step.
    u: Array<[f32; 4]>,
    u0: Array<[f32; 4]>,
    /// Primitive state with two ghost layers.
    w: Array<[f32; 4]>,
    fi: Array<[f32; 4]>,
    fj: Array<[f32; 4]>,
    res: Array<[f32; 4]>,
    dt: Array<f32>,
    /// Stage-1 `d rho / dt` per cell (for the Luma norm and the history kernel).
    rho_res: Array<f32>,
    /// Per-workgroup / per-slice partial sums of `rho_res^2`.
    partial: Array<f32>,
    hist: Array<f32>,
    /// `(nx, nr, len, len * r_mid)` per face.
    iface: Array<[f32; 4]>,
    jface: Array<[f32; 4]>,
    /// `(A, V, x_c, r_c)` per cell.
    cell: Array<[f32; 4]>,
    pub iter: usize,
    /// Low parts of `u` and `u0` ([`Precision::Compensated`] only).
    lo: Option<Lo>,
    prec: Precision,
    /// Low-Mach correction of the reconstructed face states.
    low_mach: bool,
    /// Solid body block (base / wake grids).
    block: Option<Block>,
    /// Number of non-solid cells (normalisation of the residual norm).
    n_active: usize,
    /// Line-implicit scheme (kernels, work arrays, parameters); `None` = explicit SSP-RK3.
    imp: Option<Imp>,
    /// Viscous terms and turbulence model (geometry, state, work arrays); `None` for the Euler equations ([`GpuSolver::set_viscous`]).
    visc: Option<shaders_visc::ViscState>,
    /// Iterations since the start of the implicit CFL ramp ([`GpuSolver::restart_ramp`]).
    ramp_k: usize,
    /// Kernel and output of [`GpuSolver::base_pressures`], built on first use.
    base_probe: Option<(Kernel, Array<[f32; 4]>)>,
}

/// Kernels and work arrays of the line-implicit scheme (`bowshock_core::implicit`, [`shaders_impl`]). Every 4x4 block is stored as
/// four consecutive column vec4; the i-line blocks at `4 (i nj + j)` (coalesced across lines), the j-line blocks at `4 (j ni + i)`.
struct Imp {
    p: ImplicitParams,
    residual: Kernel,
    assemble_i: Kernel,
    assemble_j: Kernel,
    line_i: Kernel,
    line_j: Kernel,
    apply: Kernel,
    update: Kernel,
    /// Viscous run: kernels of the viscous terms and the turbulence model (`residual`, `assemble_*` are then their viscous variants).
    vk: Option<shaders_visc::ViscKernels>,
    /// The solution `dU` and the right-hand side / solution `x` of the line solves.
    x: Array<[f32; 4]>,
    du: Array<[f32; 4]>,
    lo_i: Array<[f32; 4]>,
    d_i: Array<[f32; 4]>,
    up_i: Array<[f32; 4]>,
    lo_j: Array<[f32; 4]>,
    d_j: Array<[f32; 4]>,
    up_j: Array<[f32; 4]>,
    /// Reduced rows of the cyclic-reduction line solves (9 vec4 per row, shared by the i and j lines).
    scr: Array<[f32; 4]>,
}

/// Low (correction) words of the compensated state.
struct Lo {
    u: Array<[f32; 4]>,
    u0: Array<[f32; 4]>,
}

impl GpuSolver {
    /// Uploads the geometry, builds the kernels and initialises the state to the free stream
    /// ([`Mode::Guarded`]).
    pub fn new(grid: &Grid, cfg: &Config) -> Result<GpuSolver> {
        Self::new_on(&luma::default_context()?, grid, cfg, Mode::Guarded)
    }

    /// [`Precision::default`] state precision.
    pub fn new_on(ctx: &Context, grid: &Grid, cfg: &Config, mode: Mode) -> Result<GpuSolver> {
        Self::new_with(ctx, grid, cfg, mode, Precision::default())
    }

    pub fn new_with(ctx: &Context, grid: &Grid, cfg: &Config, mode: Mode, prec: Precision) -> Result<GpuSolver> {
        let (ni, nj) = (grid.ni, grid.nj);
        let comp = prec != Precision::F32;
        let ext = prec == Precision::Extended;
        let k = match mode {
            Mode::Direct if ext => Kernels::Direct {
                prim: build_ext(ctx, shaders_ext::PRIM_EXT, "prim_ext")?,
                ghost: build_ext(ctx, shaders_ext::GHOST_EXT, "ghost_ext")?,
                flux_i: build_ext(ctx, shaders_ext::FLUX_I_EXT, "flux_i_ext")?,
                flux_j: build_ext(ctx, shaders_ext::FLUX_J_EXT, "flux_j_ext")?,
                residual: build_ext(ctx, shaders_ext::RESIDUAL_EXT, "residual_ext")?,
                update: build(ctx, &shaders::update(true), "update")?,
                hist_partial: build(ctx, shaders::HIST_PARTIAL, "hist_partial")?,
                hist_final: build(ctx, shaders::HIST_FINAL, "hist_final")?,
            },
            Mode::Direct => Kernels::Direct {
                prim: build(ctx, shaders::PRIM, "prim")?,
                ghost: build(ctx, shaders::GHOST, "ghost")?,
                flux_i: build(ctx, shaders::FLUX_I, "flux_i")?,
                flux_j: build(ctx, shaders::FLUX_J, "flux_j")?,
                residual: build(ctx, shaders::RESIDUAL, "residual")?,
                update: build(ctx, &shaders::update(comp), "update")?,
                hist_partial: build(ctx, shaders::HIST_PARTIAL, "hist_partial")?,
                hist_final: build(ctx, shaders::HIST_FINAL, "hist_final")?,
            },
            Mode::Guarded if ext => Kernels::Guarded {
                flux: build_ext(ctx, shaders_ext::FLUX_EXT, "flux_ext")?,
                residual_update: build_ext(ctx, shaders_ext::RESIDUAL_UPDATE_EXT, "residual_update_ext")?,
                hist_final: build(ctx, shaders::HIST_FINAL, "hist_final")?,
            },
            Mode::Guarded => Kernels::Guarded {
                flux: build(ctx, shaders::FLUX_GUARDED, "flux_guarded")?,
                residual_update: build(ctx, &shaders::residual_update(comp), "residual_update")?,
                hist_final: build(ctx, shaders::HIST_FINAL, "hist_final")?,
            },
        };
        let face = |f: &bowshock_core::grid::Face| [f.nx as f32, f.nr as f32, f.len as f32, f.w() as f32];
        let iface: Vec<[f32; 4]> = grid.iface.iter().map(face).collect();
        let jface: Vec<[f32; 4]> = grid.jface.iter().map(face).collect();
        // `cell` = (A, V, Cx, Cr): the closure defect `C = sum_f w n - (0, A)` of the f32 face data of every cell, summed in f64 (exactly zero
        // for an exact geometry); the extended-precision residual kernels subtract its effect on a uniform flow (`shaders_ext::dgeo_corr`).
        let cell: Vec<[f32; 4]> = (0..ni * nj)
            .map(|c| {
                let (i, j) = (c % ni, c / ni);
                let (fl, fr, fb, ft) = (iface[j * (ni + 1) + i], iface[j * (ni + 1) + i + 1], jface[j * ni + i], jface[(j + 1) * ni + i]);
                let area = grid.area[c] as f32;
                let sum = |k: usize| fr[3] as f64 * fr[k] as f64 - fl[3] as f64 * fl[k] as f64 + ft[3] as f64 * ft[k] as f64 - fb[3] as f64 * fb[k] as f64;
                [area, grid.vol[c] as f32, sum(0) as f32, (sum(1) - area as f64) as f32]
            })
            .collect();
        let fs = f32x4(cfg.freestream);
        let q0 = f32x4(prim_to_cons(&cfg.freestream));
        let nw = (ni + 4) * (nj + 4);
        Ok(GpuSolver {
            ni,
            nj,
            cfg: cfg.clone(),
            mode,
            k,
            u: Array::from_slice_on(ctx, &vec![q0; ni * nj], ni * nj)?,
            u0: Array::zeros_on(ctx, ni * nj)?,
            w: Array::from_slice_on(ctx, &vec![fs; nw * (1 + ext as usize)], nw * (1 + ext as usize))?,
            fi: Array::zeros_on(ctx, (1 + ext as usize) * (ni + 1) * nj)?,
            fj: Array::zeros_on(ctx, (1 + ext as usize) * ni * (nj + 1))?,
            res: Array::zeros_on(ctx, ni * nj)?,
            dt: Array::zeros_on(ctx, ni * nj)?,
            rho_res: Array::zeros_on(ctx, ni * nj)?,
            partial: Array::zeros_on(ctx, n_partial(ni, nj))?,
            hist: Array::zeros_on(ctx, MAX_CHUNK)?,
            iface: Array::from_slice_on(ctx, &iface, iface.len())?,
            jface: Array::from_slice_on(ctx, &jface, jface.len())?,
            cell: Array::from_slice_on(ctx, &cell, ni * nj)?,
            ctx: ctx.clone(),
            iter: 0,
            lo: if comp { Some(Lo { u: Array::zeros_on(ctx, ni * nj)?, u0: Array::zeros_on(ctx, ni * nj)? }) } else { None },
            prec,
            low_mach: false,
            block: None,
            n_active: ni * nj,
            imp: None,
            visc: None,
            ramp_k: 0,
            base_probe: None,
        })
    }

    /// Overwrites the conserved state (e.g. with a CPU solution) and restarts the iteration counter. In the
    /// compensated precision the low part receives the remainder `u - f32(u)`.
    pub fn set_state(&mut self, u: &[[f64; 4]]) -> Result<()> {
        let q: Vec<[f32; 4]> = u.iter().map(|&v| f32x4(v)).collect();
        self.u.write(&q)?;
        if let Some(lo) = &mut self.lo {
            let r: Vec<[f32; 4]> = u.iter().zip(&q).map(|(v, h)| [0, 1, 2, 3].map(|k| (v[k] - h[k] as f64) as f32)).collect();
            lo.u.write(&r)?;
        }
        self.iter = 0;
        self.ramp_k = 0; // implicit scheme: the CFL ramp restarts with every new state
        Ok(())
    }

    /// Resets to the free stream at iteration 0 (the upload is recorded in order with later work).
    /// The implicit CFL ramp restarts as well, and a viscous run's turbulence state returns to the free stream (`k_inf`, `omega_inf`,
    /// `mu_t = rho k_inf / omega_inf`), as `bowshock::run::run_on` does on the CPU for a cold start.
    pub fn reset(&mut self) -> Result<()> {
        let fs = prim_to_cons(&self.cfg.freestream);
        self.set_state(&vec![fs; self.ni * self.nj])?;
        let rho = self.cfg.freestream[0];
        if let Some(vs) = &mut self.visc {
            vs.reset_turbulence(rho)?;
        }
        Ok(())
    }

    fn params(&self, limiter: u32, stage: u32, slot: u32) -> Params {
        let fs = f32x4(self.cfg.freestream);
        let lo = [0, 1, 2, 3].map(|k| (self.cfg.freestream[k] - fs[k] as f64) as f32);
        Params {
            ni: self.ni as u32,
            nj: self.nj as u32,
            limiter,
            stage,
            cfl: self.cfg.cfl as f32,
            fs_rho: fs[0],
            fs_u: fs[1],
            fs_v: fs[2],
            fs_p: fs[3],
            bc0: bc_code(self.cfg.bc[0]),
            bc1: bc_code(self.cfg.bc[1]),
            bc2: bc_code(self.cfg.bc[2]),
            bc3: bc_code(self.cfg.bc[3]),
            slot,
            aux: 0,
            zero: 0,
            ra: 1.0,
            rb: 0.0,
            low_mach: self.low_mach as u32,
            iflags: 0,
            fs_lo_rho: lo[0],
            fs_lo_u: lo[1],
            fs_lo_v: lo[2],
            fs_lo_p: lo[3],
            blk_i: self.block.map_or(0, |b| b.i_end as u32),
            blk_j: self.block.map_or(0, |b| b.j_end as u32),
            blk_s: self.block.map_or(0, |b| b.j_sting as u32),
            wall_i0: self.visc.as_ref().map_or(0, |v| v.v.wall_i_start as u32),
            icfl: 0.0,
            eps_fix: 0.0,
            vflags: self.visc.as_ref().map_or(0, |v| 1 | ((v.v.turbulence == Turbulence::Sst) as u32) << 1),
            ipad2: 0,
            mu_inf: self.visc.as_ref().map_or(0.0, |v| (1.0 / v.v.re_c) as f32),
            suth: self.visc.as_ref().map_or(0.0, |v| v.v.sutherland as f32),
            k_inf: bowshock_core::turbulence::K_INF as f32,
            om_inf: self.visc.as_ref().map_or(0.0, |v| bowshock_core::turbulence::omega_inf(v.v.re_c) as f32),
        }
    }

    /// Builds the solver of a host `bowshock_core` solver: its grid and config, low-Mach flag, solid block, convergence scheme
    /// (`Scheme::Explicit` or `Scheme::Implicit`) and viscous terms (`Mode::Guarded` + `Scheme::Implicit` only, see
    /// [`GpuSolver::set_viscous`]); the state starts at the free stream, not at `host.u`.
    pub fn from_host(ctx: &Context, host: &Solver, mode: Mode, prec: Precision) -> Result<GpuSolver> {
        let mut g = Self::new_with(ctx, &host.grid, &host.cfg, mode, prec)?;
        g.set_low_mach(host.low_mach);
        g.set_block(host.block())?;
        g.set_scheme(host.scheme())?;
        g.set_viscous(host.viscous(), &host.grid)?;
        Ok(g)
    }

    /// Convergence scheme in use: `Scheme::Explicit` without the line-implicit machinery, else `Scheme::Implicit` with its parameters.
    pub fn scheme(&self) -> Scheme {
        match &self.imp {
            None => Scheme::Explicit,
            Some(i) => Scheme::Implicit(i.p),
        }
    }

    /// Switches the low-Mach correction of the reconstructed face states (`bowshock_core::flux::low_mach_states`) on or off.
    pub fn set_low_mach(&mut self, on: bool) {
        self.low_mach = on;
    }

    /// Declares the solid body block of a base / wake grid (`bowshock_core::solver::Solver::set_block`); only the guarded formulation
    /// supports blocks. Must precede [`GpuSolver::set_viscous`] (the no-slip walls and the wall distance depend on it).
    pub fn set_block(&mut self, block: Option<Block>) -> Result<()> {
        if block.is_some() && self.mode == Mode::Direct {
            return Err(GpuError::Unsupported("solid blocks need Mode::Guarded"));
        }
        if self.visc.is_some() {
            return Err(GpuError::Unsupported("the solid block must be declared before the viscous terms"));
        }
        self.block = block;
        self.n_active = match block {
            Some(b) => self.ni * self.nj - b.i_end * b.j_end - (self.ni - b.i_end) * b.j_sting,
            None => self.ni * self.nj,
        };
        Ok(())
    }

    pub fn block(&self) -> Option<Block> {
        self.block
    }

    /// Selects the convergence scheme: `Scheme::Explicit` (SSP-RK3 with local time steps) or `Scheme::Implicit` (line-implicit
    /// approximate factorization, guarded formulation only; see [`shaders_impl`]). Multigrid and residual smoothing are not ported
    /// (they measured worse than the line-implicit scheme on the CPU).
    pub fn set_scheme(&mut self, scheme: Scheme) -> Result<()> {
        match scheme {
            Scheme::Explicit => {
                if self.visc.is_some() {
                    return Err(GpuError::Unsupported("viscous terms need Scheme::Implicit"));
                }
                self.imp = None;
                Ok(())
            }
            Scheme::Implicit(p) => {
                if self.mode != Mode::Guarded {
                    return Err(GpuError::Unsupported("the implicit scheme needs Mode::Guarded"));
                }
                self.imp = Some(self.build_implicit(p)?);
                self.restart_ramp();
                Ok(())
            }
            _ => Err(GpuError::Unsupported("only Scheme::Explicit and Scheme::Implicit are implemented on the GPU")),
        }
    }

    /// The implicit scheme's parameters, `None` for the explicit scheme.
    pub fn implicit_params(&self) -> Option<ImplicitParams> {
        self.imp.as_ref().map(|i| i.p)
    }

    /// Restarts the CFL ramp of the implicit scheme (call at the start of a run and after a change of the free stream).
    pub fn restart_ramp(&mut self) {
        self.ramp_k = 0;
    }

    /// Switches the viscous terms on (`Some`, with the SST model if `v.turbulence` says so) or off (`None`) for a solver with `Mode::Guarded` and
    /// `Scheme::Implicit` (declare the block and the scheme first; anything else returns [`GpuError::Unsupported`]). `grid` is the grid the solver was
    /// built from: the geometry of the compact face gradients, the cell areas and the wall distance are computed from it in f64 and uploaded as f32
    /// (`shaders_visc`). The turbulence state starts at the free stream; the mean-flow state is not touched.
    pub fn set_viscous(&mut self, v: Option<Viscous>, grid: &Grid) -> Result<()> {
        let Some(v) = v else {
            self.visc = None;
            if let Some(p) = self.imp.as_ref().map(|i| i.p) {
                self.imp = Some(self.build_implicit(p)?);
            }
            return Ok(());
        };
        if self.mode != Mode::Guarded {
            return Err(GpuError::Unsupported("viscous terms need Mode::Guarded"));
        }
        let Some(p) = self.imp.as_ref().map(|i| i.p) else {
            return Err(GpuError::Unsupported("viscous terms need Scheme::Implicit"));
        };
        assert_eq!((grid.ni, grid.nj), (self.ni, self.nj), "set_viscous: the grid of the solver");
        self.visc = Some(shaders_visc::ViscState::new(&self.ctx, grid, &self.cfg, self.block, v)?);
        self.imp = Some(self.build_implicit(p)?);
        Ok(())
    }

    /// The viscous-flow parameters, `None` for the Euler equations.
    pub fn viscous(&self) -> Option<Viscous> {
        self.visc.as_ref().map(|v| v.v)
    }

    /// Turbulence state `(k, omega, mu_t, mu)` of every cell (`ni * nj`, as `bowshock_core::solver::Solver::turb`, `mu_t`, `mu`): `mu_t` is the eddy
    /// viscosity entering the next iteration, `mu` the laminar viscosity of the last residual evaluation. All zero for a laminar run.
    pub fn turb_state(&self) -> Result<Vec<[f32; 4]>> {
        let vs = self.visc.as_ref().ok_or(GpuError::Unsupported("turb_state needs the viscous terms"))?;
        Ok(vs.turb.to_vec()?)
    }

    fn build_implicit(&self, p: ImplicitParams) -> Result<Imp> {
        let ctx = &self.ctx;
        let (ni, nj) = (self.ni, self.nj);
        let n = ni * nj;
        let comp = self.prec != Precision::F32;
        let ext = self.prec == Precision::Extended;
        let viscous = self.visc.is_some();
        let m = || Array::<[f32; 4]>::zeros_on(ctx, 4 * n);
        Ok(Imp {
            p,
            residual: if ext {
                ctx.kernel(&shaders_impl::residual_store_ext(viscous)).label("residual_store_ext").build()?
            } else {
                ctx.kernel(&shaders_impl::with_common(&shaders_impl::residual_store(viscous))).label("residual_store").build()?
            },
            assemble_i: ctx.kernel(&shaders_impl::with_common(&shaders_impl::assemble_i(viscous))).label("assemble_i").build()?,
            assemble_j: ctx.kernel(&shaders_impl::with_common(&shaders_impl::assemble_j(viscous))).label("assemble_j").build()?,
            line_i: ctx.kernel(&shaders_impl::with_common(&shaders_impl::line_i(ni))).label("line_i").build()?,
            line_j: ctx.kernel(&shaders_impl::with_common(&shaders_impl::line_j(nj))).label("line_j").build()?,
            apply: ctx.kernel(&shaders_impl::with_common(&shaders_impl::apply())).label("apply").build()?,
            update: ctx.kernel(&shaders_impl::with_common(&shaders_impl::update(comp))).label("implicit_update").build()?,
            vk: match &self.visc {
                Some(vs) => Some(shaders_visc::ViscKernels::build(ctx, vs.sst.is_some(), ext)?),
                None => None,
            },
            x: Array::zeros_on(ctx, n)?,
            du: Array::zeros_on(ctx, n)?,
            lo_i: m()?,
            d_i: m()?,
            up_i: m()?,
            lo_j: m()?,
            d_j: m()?,
            up_j: m()?,
            scr: Array::zeros_on(ctx, 9 * n)?,
        })
    }

    /// Records one iteration of the line-implicit scheme (see [`shaders_impl`]): face fluxes and cell residual (as the explicit
    /// stage 1, extended-precision residual kept), pseudo-time step and residual history, assembly of the two block-tridiagonal
    /// factors, the factored solve, `sub_iters - 1` defect-correction passes and the positivity-damped state update. A viscous run
    /// adds the cell gradients and the viscous face fluxes after the inviscid fluxes (the residual kernel then subtracts them), and the
    /// SST turbulence update (gradients and blending functions, line coefficients, the two scalar line solves, the clipped update) from the
    /// state of the start of the iteration, i.e. before the mean-flow update ([`shaders_visc`]).
    fn record_implicit(&mut self, slot: Option<u32>) -> Result<()> {
        let (ni, nj) = (self.ni as u32, self.nj as u32);
        let lim = if self.iter < self.cfg.first_order_iters { 0 } else { limiter_code(self.cfg.limiter) };
        let ext = self.prec == Precision::Extended;
        let nwg = (self.ni.div_ceil(32) * self.nj.div_ceil(4)) as u32;
        let imp = self.imp.as_ref().expect("implicit scheme");
        let ip = imp.p;
        let cfl = (ip.cfl0 * ip.growth.powi(self.ramp_k.min(10_000) as i32)).min(ip.cfl_max);
        self.ramp_k += 1;
        let mut p = self.params(lim, 1, slot.unwrap_or(0));
        p.icfl = cfl as f32;
        p.eps_fix = ip.eps_fix as f32;
        p.iflags = ip.lines as u32 | (ip.implicit_bc as u32) << 1 | (ip.roe as u32) << 2 | ((ip.dt_mode == DtMode::MinDirection) as u32) << 3;
        let Kernels::Guarded { flux, hist_final, .. } = &self.k else { unreachable!("implicit scheme needs Mode::Guarded") };
        match &self.lo {
            Some(lo) if ext => flux.launch([ni + 1, 2 * nj + 1], args![&self.u, &lo.u, &self.iface, &self.jface, &mut self.fi, &mut self.fj, uniform(&p)])?,
            _ => flux.launch([ni + 1, 2 * nj + 1], args![&self.u, &self.iface, &self.jface, &mut self.fi, &mut self.fj, uniform(&p)])?,
        }
        if let (Some(vs), Some(vk)) = (self.visc.as_mut(), imp.vk.as_ref()) {
            vk.vgrad.launch([ni, nj], args![&self.u, &self.iface, &self.jface, &vs.vcell, &mut vs.grad, &mut vs.turb, uniform(&p)])?;
            vk.vflux.launch(
                [ni + 1, 2 * nj + 1],
                args![&self.u, &self.iface, &self.jface, &vs.vgi, &vs.vgj, &vs.grad, &vs.turb, &mut vs.fvi, &mut vs.fvj, uniform(&p)],
            )?;
        }
        if let Some(vs) = &self.visc {
            // the viscous residual kernel: `fvi, fvj, grad, turb, vcell` between the (optional) low state and the uniform block
            let mut a = Vec::from(args![
                &self.u,
                &self.fi,
                &self.fj,
                &self.iface,
                &self.jface,
                &self.cell,
                &mut self.res,
                &mut self.dt,
                &mut self.rho_res,
                &mut self.partial
            ]);
            if let (Some(lo), true) = (&self.lo, ext) {
                a.push((&lo.u).into());
            }
            a.extend(args![&vs.fvi, &vs.fvj, &vs.grad, &vs.turb, &vs.vcell, uniform(&p)]);
            imp.residual.launch([ni, nj], a)?;
        } else {
            match &self.lo {
                Some(lo) if ext => imp.residual.launch(
                    [ni, nj],
                    args![
                        &self.u,
                        &self.fi,
                        &self.fj,
                        &self.iface,
                        &self.jface,
                        &self.cell,
                        &mut self.res,
                        &mut self.dt,
                        &mut self.rho_res,
                        &mut self.partial,
                        &lo.u,
                        uniform(&p)
                    ],
                )?,
                _ => imp.residual.launch(
                    [ni, nj],
                    args![
                        &self.u,
                        &self.fi,
                        &self.fj,
                        &self.iface,
                        &self.jface,
                        &self.cell,
                        &mut self.res,
                        &mut self.dt,
                        &mut self.rho_res,
                        &mut self.partial,
                        uniform(&p)
                    ],
                )?,
            }
        }
        if slot.is_some() {
            let pp = Params { aux: nwg, ..p };
            hist_final.launch([256], args![&mut self.hist, &self.partial, uniform(&pp)])?;
        }
        let imp = self.imp.as_mut().expect("implicit scheme");
        if let Some(vs) = &self.visc {
            imp.assemble_i.launch(
                [nj, ni],
                args![&self.u, &self.iface, &self.jface, &self.cell, &self.dt, &mut imp.lo_i, &mut imp.d_i, &mut imp.up_i, &vs.turb, &vs.vgi, uniform(&p)],
            )?;
            imp.assemble_j.launch(
                [ni, nj],
                args![&self.u, &self.iface, &self.jface, &self.cell, &self.dt, &mut imp.lo_j, &mut imp.d_j, &mut imp.up_j, &vs.turb, &vs.vgj, uniform(&p)],
            )?;
        } else {
            imp.assemble_i
                .launch([nj, ni], args![&self.u, &self.iface, &self.jface, &self.cell, &self.dt, &mut imp.lo_i, &mut imp.d_i, &mut imp.up_i, uniform(&p)])?;
            imp.assemble_j
                .launch([ni, nj], args![&self.u, &self.iface, &self.jface, &self.cell, &self.dt, &mut imp.lo_j, &mut imp.d_j, &mut imp.up_j, uniform(&p)])?;
        }
        for pass in 0..ip.sub_iters.max(1) {
            let pp = Params { aux: pass as u32, ..p };
            if pass > 0 {
                imp.apply.launch(
                    [ni, nj],
                    args![&imp.lo_i, &imp.d_i, &imp.up_i, &imp.lo_j, &imp.d_j, &imp.up_j, &imp.du, &self.res, &self.dt, &mut imp.x, uniform(&pp)],
                )?;
            }
            imp.line_i
                .launch_raw([nj, 1, 1], args![&imp.lo_i, &imp.d_i, &imp.up_i, &mut imp.scr, &mut imp.x, &mut imp.du, &self.res, &self.dt, uniform(&pp)])?;
            imp.line_j
                .launch_raw([ni, 1, 1], args![&imp.lo_j, &imp.d_j, &imp.up_j, &mut imp.scr, &mut imp.x, &mut imp.du, &self.res, &self.dt, uniform(&pp)])?;
        }
        if let (Some(vs), Some(vk)) = (self.visc.as_mut(), imp.vk.as_ref()) {
            if let (Some(ss), Some(sk)) = (vs.sst.as_mut(), vk.sst.as_ref()) {
                sk.grad.launch([ni, nj], args![&self.u, &self.iface, &self.jface, &vs.vcell, &vs.turb, &mut ss.tgb, uniform(&p)])?;
                sk.coef.launch(
                    [ni, nj],
                    args![
                        &self.u,
                        &self.iface,
                        &self.jface,
                        &self.cell,
                        &vs.vcell,
                        &self.fi,
                        &self.fj,
                        &self.dt,
                        &vs.turb,
                        &vs.grad,
                        &ss.tgb,
                        &vs.vgi,
                        &vs.vgj,
                        &mut ss.tres,
                        &mut ss.lo_i,
                        &mut ss.d_i,
                        &mut ss.up_i,
                        &mut ss.lo_j,
                        &mut ss.d_j,
                        &mut ss.up_j,
                        uniform(&p)
                    ],
                )?;
                // the two scalar systems: the line solves of the mean flow on the diagonal blocks (pass 0)
                let pp = Params { aux: 0, ..p };
                imp.line_i
                    .launch_raw([nj, 1, 1], args![&ss.lo_i, &ss.d_i, &ss.up_i, &mut imp.scr, &mut ss.tx, &mut ss.tdu, &ss.tres, &self.dt, uniform(&pp)])?;
                imp.line_j
                    .launch_raw([ni, 1, 1], args![&ss.lo_j, &ss.d_j, &ss.up_j, &mut imp.scr, &mut ss.tx, &mut ss.tdu, &ss.tres, &self.dt, uniform(&pp)])?;
                sk.update.launch([ni, nj], args![&self.u, &vs.vcell, &vs.grad, &ss.tdu, &mut vs.turb, uniform(&p)])?;
            }
        }
        match &mut self.lo {
            Some(lo) => imp.update.launch([ni, nj], args![&mut self.u, &imp.du, &mut lo.u, uniform(&p)])?,
            None => imp.update.launch([ni, nj], args![&mut self.u, &imp.du, uniform(&p)])?,
        }
        self.iter += 1;
        Ok(())
    }

    /// Records one SSP-RK3 step (nothing is executed or waited for). `slot` is the residual-history
    /// slot written by the history kernels (`None`: no history, e.g. when only the Luma norm of
    /// `rho_res` is used).
    fn record_step(&mut self, slot: Option<u32>) -> Result<()> {
        if self.imp.is_some() {
            return self.record_implicit(slot);
        }
        if self.visc.is_some() {
            return Err(GpuError::Unsupported("viscous terms need Scheme::Implicit"));
        }
        let (ni, nj) = (self.ni as u32, self.nj as u32);
        let lim = if self.iter < self.cfg.first_order_iters { 0 } else { limiter_code(self.cfg.limiter) };
        let mut p = self.params(lim, 1, slot.unwrap_or(0));
        let hist = slot.is_some();
        let ext = self.prec == Precision::Extended;
        match &self.k {
            Kernels::Direct { prim, ghost, flux_i, flux_j, residual, update, hist_partial, hist_final } => {
                for (stage, (ra, rb)) in RK3.into_iter().enumerate().map(|(k, c)| (k as u32 + 1, c)) {
                    p.stage = stage;
                    (p.ra, p.rb) = (ra, rb);
                    // bowshock-core `eval_residual`: cons->prim, ghost fill, face fluxes, cell residual
                    match &self.lo {
                        Some(lo) if ext => prim.launch([ni, nj], args![&self.u, &lo.u, &mut self.w, uniform(&p)])?,
                        _ => prim.launch([ni, nj], args![&self.u, &mut self.w, uniform(&p)])?,
                    }
                    ghost.launch([ni + nj], args![&mut self.w, &self.iface, &self.jface, uniform(&p)])?;
                    flux_i.launch([ni + 1, nj], args![&self.w, &self.iface, &mut self.fi, uniform(&p)])?;
                    flux_j.launch([ni, nj + 1], args![&self.w, &self.jface, &mut self.fj, uniform(&p)])?;
                    residual.launch(
                        [ni, nj],
                        args![&self.w, &self.fi, &self.fj, &self.iface, &self.jface, &self.cell, &mut self.res, &mut self.dt, &mut self.rho_res, uniform(&p)],
                    )?;
                    if stage == 1 && hist {
                        let pp = Params { aux: 128, ..p };
                        hist_partial.launch([128 * 256], args![&mut self.partial, &self.rho_res, uniform(&pp)])?;
                        hist_final.launch([256], args![&mut self.hist, &self.partial, uniform(&pp)])?;
                    }
                    match &mut self.lo {
                        None => update.launch([ni, nj], args![&mut self.u, &mut self.u0, &self.res, &self.dt, uniform(&p)])?,
                        Some(lo) => update.launch([ni, nj], args![&mut self.u, &mut self.u0, &self.res, &self.dt, &mut lo.u, &mut lo.u0, uniform(&p)])?,
                    }
                }
            }
            Kernels::Guarded { flux, residual_update, hist_final } => {
                p.aux = (self.ni.div_ceil(32) * self.nj.div_ceil(4)) as u32; // workgroups of `residual_update`
                for (stage, (ra, rb)) in RK3.into_iter().enumerate().map(|(k, c)| (k as u32 + 1, c)) {
                    p.stage = stage;
                    (p.ra, p.rb) = (ra, rb);
                    match &self.lo {
                        Some(lo) if ext => {
                            flux.launch([ni + 1, 2 * nj + 1], args![&self.u, &lo.u, &self.iface, &self.jface, &mut self.fi, &mut self.fj, uniform(&p)])?
                        }
                        _ => flux.launch([ni + 1, 2 * nj + 1], args![&self.u, &self.iface, &self.jface, &mut self.fi, &mut self.fj, uniform(&p)])?,
                    }
                    match &mut self.lo {
                        None => residual_update.launch(
                            [ni, nj],
                            args![
                                &mut self.u,
                                &mut self.u0,
                                &self.fi,
                                &self.fj,
                                &self.iface,
                                &self.jface,
                                &self.cell,
                                &mut self.dt,
                                &mut self.rho_res,
                                &mut self.partial,
                                uniform(&p)
                            ],
                        )?,
                        Some(lo) => residual_update.launch(
                            [ni, nj],
                            args![
                                &mut self.u,
                                &mut self.u0,
                                &self.fi,
                                &self.fj,
                                &self.iface,
                                &self.jface,
                                &self.cell,
                                &mut self.dt,
                                &mut self.rho_res,
                                &mut self.partial,
                                &mut lo.u,
                                &mut lo.u0,
                                uniform(&p)
                            ],
                        )?,
                    }
                    if stage == 1 && hist {
                        hist_final.launch([256], args![&mut self.hist, &self.partial, uniform(&p)])?;
                    }
                }
            }
        }
        self.iter += 1;
        Ok(())
    }

    /// Runs `n <= MAX_CHUNK` iterations without synchronising in between and then reads back the
    /// per-iteration density residual (L2 of the stage-1 `d rho/dt` over sqrt(N), as in bowshock-core). This is the
    /// only synchronisation point.
    pub fn run(&mut self, n: usize) -> Result<Vec<f64>> {
        assert!((1..=MAX_CHUNK).contains(&n));
        for s in 0..n {
            self.record_step(Some(s as u32))?;
        }
        let h = self.hist.to_vec()?;
        let cells = self.n_active as f64;
        Ok(h[..n].iter().map(|&s| (s as f64 / cells).sqrt()).collect())
    }

    /// Runs `n` iterations (any `n`, executed in submissions of Luma's choosing) and returns the
    /// residual of the *last* one from Luma's `Array::norm` reduction (no history kernel).
    pub fn run_luma_norm(&mut self, n: usize) -> Result<f64> {
        for _ in 0..n {
            self.record_step(None)?;
        }
        self.luma_residual()
    }

    /// Density residual of the last stage-1 evaluation, via Luma's `norm()` reduction.
    pub fn luma_residual(&self) -> Result<f64> {
        Ok(self.rho_res.norm()? as f64 / (self.n_active as f64).sqrt())
    }

    /// Records `n` iterations without history kernel and without waiting (benchmark helper; Luma
    /// submits a batch every 256 dispatches, so the GPU already runs while recording continues).
    pub fn record(&mut self, n: usize) -> Result<()> {
        for _ in 0..n {
            self.record_step(None)?;
        }
        Ok(())
    }

    /// Conserved state, `ni*nj`, same layout as `bowshock_core::solver::Solver::u` (the f32 high part in the compensated
    /// precision, which is what the flux kernels see).
    pub fn state(&self) -> Result<Vec<[f32; 4]>> {
        Ok(self.u.to_vec()?)
    }

    /// Conserved state as f64: `hi + lo` in the compensated precision.
    pub fn state_f64(&self) -> Result<Vec<[f64; 4]>> {
        let hi = self.u.to_vec()?;
        let lo = match &self.lo {
            Some(lo) => Some(lo.u.to_vec()?),
            None => None,
        };
        Ok(hi.iter().enumerate().map(|(c, h)| [0, 1, 2, 3].map(|k| h[k] as f64 + lo.as_ref().map_or(0.0, |l| l[c][k] as f64))).collect())
    }

    /// Conserved state (`hi + lo` in the compensated precision, as f64) of the two first cells behind the base face,
    /// `[(i_end, j), (i_end + 1, j)]`, for every row `j_sting..j_end` (the extrapolation stencil of
    /// `bowshock::run::base_state`). Reads back only these rows.
    pub fn base_cells(&mut self) -> Result<Vec<[[f64; 4]; 2]>> {
        let b = self.block.ok_or(GpuError::Unsupported("base_cells needs a solid block"))?;
        let rows = b.j_end - b.j_sting;
        if self.base_probe.is_none() {
            let src = format!("{}\n{}", shaders::common(), shaders::base_cells(self.lo.is_some()));
            self.base_probe = Some((self.ctx.kernel(&src).label("base_cells").build()?, Array::zeros_on(&self.ctx, 4 * rows)?));
        }
        let p = self.params(0, 0, 0);
        let (kernel, out) = self.base_probe.as_mut().expect("built above");
        match &self.lo {
            Some(lo) => kernel.launch([rows as u32], args![&self.u, &lo.u, &mut *out, uniform(&p)])?,
            None => kernel.launch([rows as u32], args![&self.u, &mut *out, uniform(&p)])?,
        }
        let v = out.to_vec()?;
        Ok(v.chunks(4).map(|c| [0, 1].map(|n| [0, 1, 2, 3].map(|k| c[n][k] as f64 + c[n + 2][k] as f64))).collect())
    }

    pub fn precision(&self) -> Precision {
        self.prec
    }

    pub fn config(&self) -> &Config {
        &self.cfg
    }

    /// Mutable configuration, e.g. to switch the free stream (Mach number of a warm-started sweep), CFL or limiter between
    /// runs without touching the state: every launch reads the configuration afresh (`params`).
    pub fn config_mut(&mut self) -> &mut Config {
        &mut self.cfg
    }

    /// Stage-1 `d rho / dt` of every cell from the last iteration (diagnostics).
    pub fn rho_residual_field(&self) -> Result<Vec<f32>> {
        Ok(self.rho_res.to_vec()?)
    }

    pub fn mode(&self) -> Mode {
        self.mode
    }

    pub fn context(&self) -> &Context {
        &self.ctx
    }
}

/// Evaluates MUSCL reconstruction, HLLC and the complete face flux (HLLC/Rusanov blend) on the GPU for independent samples (test helper).
/// `states[s] = [m2, m1, p0, p1]` (primitive), `normals[s] = (nx, nr)`. Returns `(left, right, hllc, face_flux)`.
pub fn eval_faces(states: &[[[f32; 4]; 4]], normals: &[[f32; 2]], limiter: Limiter) -> Result<Vec<[[f32; 4]; 4]>> {
    eval_faces_lm(states, normals, limiter, false)
}

/// As [`eval_faces`] with the optional low-Mach correction of the reconstructed states (`left`, `right` and `hllc` are then the
/// corrected states and their HLLC flux).
pub fn eval_faces_lm(states: &[[[f32; 4]; 4]], normals: &[[f32; 2]], limiter: Limiter, low_mach: bool) -> Result<Vec<[[f32; 4]; 4]>> {
    let ctx = luma::default_context()?;
    let n = states.len();
    assert_eq!(n, normals.len());
    let flat: Vec<[f32; 4]> = states.iter().flatten().copied().collect();
    let st = Array::from_slice_on(&ctx, &flat, flat.len())?;
    let nm = Array::from_slice_on(&ctx, normals, n)?;
    let mut out = Array::<[f32; 4]>::zeros_on(&ctx, 4 * n)?;
    let kernel = build(&ctx, shaders::FACE_TEST, "face_test")?;
    let p = Params { ni: n as u32, nj: 1, limiter: limiter_code(limiter), low_mach: low_mach as u32, ..Params::default() };
    kernel.launch([n as u32], args![&st, &nm, &mut out, uniform(&p)])?;
    let o = out.to_vec()?;
    Ok(o.chunks(4).map(|c| [c[0], c[1], c[2], c[3]]).collect())
}

/// As [`eval_faces`] with the double-float kernels of [`Precision::Extended`]; the result is `hi + lo` in f64.
pub fn eval_faces_ext(states: &[[[f32; 4]; 4]], normals: &[[f32; 2]], limiter: Limiter) -> Result<Vec<[[f64; 4]; 4]>> {
    eval_faces_ext_lm(states, normals, limiter, false)
}

/// As [`eval_faces_lm`] with the double-float kernels of [`Precision::Extended`].
pub fn eval_faces_ext_lm(states: &[[[f32; 4]; 4]], normals: &[[f32; 2]], limiter: Limiter, low_mach: bool) -> Result<Vec<[[f64; 4]; 4]>> {
    let ctx = luma::default_context()?;
    let n = states.len();
    assert_eq!(n, normals.len());
    let flat: Vec<[f32; 4]> = states.iter().flatten().copied().collect();
    let st = Array::from_slice_on(&ctx, &flat, flat.len())?;
    let nm = Array::from_slice_on(&ctx, normals, n)?;
    let mut out = Array::<[f32; 4]>::zeros_on(&ctx, 8 * n)?;
    let kernel = build_ext(&ctx, shaders_ext::FACE_TEST_EXT, "face_test_ext")?;
    let p = Params { ni: n as u32, nj: 1, limiter: limiter_code(limiter), low_mach: low_mach as u32, ..Params::default() };
    kernel.launch([n as u32], args![&st, &nm, &mut out, uniform(&p)])?;
    let o = out.to_vec()?;
    Ok(o.chunks(8).map(|c| [0, 1, 2, 3].map(|k| [0, 1, 2, 3].map(|m| c[k][m] as f64 + c[k + 4][m] as f64))).collect())
}
