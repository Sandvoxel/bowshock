//! `bowshock-gpu`: f32 port of the `bowshock-cfd` axisymmetric Euler solver (MUSCL + HLLC/Rusanov shock blend +
//! SSP-RK3 with local time stepping) to Luma custom WGSL kernels.
//!
//! Every kernel in [`shaders`] is a direct translation of the corresponding `bowshock-cfd` function or
//! loop body, with the same data layout (cell-centred conserved state, primitives with two ghost
//! layers, per-face HLLC flux, per-cell residual) so that differences are traceable. Everything is
//! recorded into Luma's automatic batch; the host only synchronises when it reads the residual
//! history (every `K` iterations) or the state.
//!
//! The state is stored in f32 arrays in every case, but [`Precision`] selects how it is accumulated and how the
//! residual is evaluated: plain f32 (the original port, whose residual stalls 4 - 4.6 orders below its peak because of
//! f32 rounding), a compensated `hi + lo` state, or (default) the compensated state plus a double-float evaluation of
//! the residual ([`shaders_ext`]), which converges like the f64 CPU solver. See the README, "The f32 residual floor".

pub mod cone;
pub mod shaders;
pub mod shaders_ext;

use bowshock_cfd::flux::Limiter;
use bowshock_cfd::gas::prim_to_cons;
use bowshock_cfd::grid::Grid;
use bowshock_cfd::solver::{Bc, Config};
use luma::{args, uniform, Array, Context, Kernel};

pub use luma::{LumaError, Result};

/// SSP-RK3 (Shu-Osher) stage coefficients `(a, b)` of `u = a (u + dt L(u)) + b u0`, as in
/// `bowshock_cfd::solver::Solver::step`.
const RK3: [(f32, f32); 3] = [(1.0, 0.0), (0.25, 0.75), ((2.0f64 / 3.0) as f32, (1.0f64 / 3.0) as f32)];

/// Maximum number of iterations recorded between two residual-history reads.
pub const MAX_CHUNK: usize = 256;

/// Uniform block of every kernel; must match `Params` in [`shaders::common`].
#[derive(encase::ShaderType, Clone, Copy, Debug)]
pub struct Params {
    pub ni: u32,
    pub nj: u32,
    pub limiter: u32,
    pub stage: u32,
    pub cfl: f32,
    pub fs_rho: f32,
    pub fs_u: f32,
    pub fs_v: f32,
    pub fs_p: f32,
    pub bc0: u32,
    pub bc1: u32,
    pub bc2: u32,
    pub bc3: u32,
    pub slot: u32,
    pub aux: u32,
    /// Always 0. The WGSL error-free transformations read it (`opq`) so that the driver's compiler cannot
    /// reassociate `(a + b) - a` to `b` (see `shaders::common`).
    pub zero: u32,
    /// RK stage update `u = ra (u + dt res) + rb u0` (stages 2, 3).
    pub ra: f32,
    pub rb: f32,
    pub pad2: u32,
    pub pad3: u32,
    /// Low parts of the free stream `(rho, u, v, p)` (f64 minus its f32 rounding), used by the extended precision.
    pub fs_lo_rho: f32,
    pub fs_lo_u: f32,
    pub fs_lo_v: f32,
    pub fs_lo_p: f32,
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
    ctx.kernel(&format!("{}\n{}", shaders::common(), src)).label(label).build()
}

fn build_ext(ctx: &Context, src: &str, label: &str) -> Result<Kernel> {
    ctx.kernel(&format!("{}\n{}", shaders_ext::common_ext(), src)).label(label).build()
}

/// How one iteration is mapped to kernel launches.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Mode {
    /// Direct translation of bowshock-cfd: cons->prim, ghost fill, i-face flux, j-face flux, residual,
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

/// GPU solver state. Created from a `bowshock-cfd` grid and config (geometry is converted to f32 and
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
        let face = |f: &bowshock_cfd::grid::Face| [f.nx as f32, f.nr as f32, f.len as f32, f.w() as f32];
        let iface: Vec<[f32; 4]> = grid.iface.iter().map(face).collect();
        let jface: Vec<[f32; 4]> = grid.jface.iter().map(face).collect();
        let cell: Vec<[f32; 4]> = (0..ni * nj)
            .map(|c| [grid.area[c] as f32, grid.vol[c] as f32, grid.xc[c] as f32, grid.rc[c] as f32])
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
        })
    }

    /// Overwrites the conserved state (e.g. with a CPU solution) and restarts the iteration counter. In the
    /// compensated precision the low part receives the remainder `u - f32(u)`.
    pub fn set_state(&mut self, u: &[[f64; 4]]) -> Result<()> {
        let q: Vec<[f32; 4]> = u.iter().map(|&v| f32x4(v)).collect();
        self.u.write(&q)?;
        if let Some(lo) = &mut self.lo {
            let r: Vec<[f32; 4]> = u
                .iter()
                .zip(&q)
                .map(|(v, h)| [0, 1, 2, 3].map(|k| (v[k] - h[k] as f64) as f32))
                .collect();
            lo.u.write(&r)?;
        }
        self.iter = 0;
        Ok(())
    }

    /// Resets to the free stream at iteration 0 (the upload is recorded in order with later work).
    pub fn reset(&mut self) -> Result<()> {
        let fs = prim_to_cons(&self.cfg.freestream);
        self.set_state(&vec![fs; self.ni * self.nj])
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
            pad2: 0,
            pad3: 0,
            fs_lo_rho: lo[0],
            fs_lo_u: lo[1],
            fs_lo_v: lo[2],
            fs_lo_p: lo[3],
        }
    }

    /// Records one SSP-RK3 step (nothing is executed or waited for). `slot` is the residual-history
    /// slot written by the history kernels (`None`: no history, e.g. when only the Luma norm of
    /// `rho_res` is used).
    fn record_step(&mut self, slot: Option<u32>) -> Result<()> {
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
                    // bowshock-cfd `eval_residual`: cons->prim, ghost fill, face fluxes, cell residual
                    match &self.lo {
                        Some(lo) if ext => prim.launch([ni, nj], args![&self.u, &lo.u, &mut self.w, uniform(&p)])?,
                        _ => prim.launch([ni, nj], args![&self.u, &mut self.w, uniform(&p)])?,
                    }
                    ghost.launch([ni + nj], args![&mut self.w, &self.iface, &self.jface, uniform(&p)])?;
                    flux_i.launch([ni + 1, nj], args![&self.w, &self.iface, &mut self.fi, uniform(&p)])?;
                    flux_j.launch([ni, nj + 1], args![&self.w, &self.jface, &mut self.fj, uniform(&p)])?;
                    residual.launch(
                        [ni, nj],
                        args![
                            &self.w,
                            &self.fi,
                            &self.fj,
                            &self.iface,
                            &self.jface,
                            &self.cell,
                            &mut self.res,
                            &mut self.dt,
                            &mut self.rho_res,
                            uniform(&p)
                        ],
                    )?;
                    if stage == 1 && hist {
                        let pp = Params { aux: 128, ..p };
                        hist_partial.launch([128 * 256], args![&mut self.partial, &self.rho_res, uniform(&pp)])?;
                        hist_final.launch([256], args![&mut self.hist, &self.partial, uniform(&pp)])?;
                    }
                    match &mut self.lo {
                        None => update.launch(
                            [ni, nj],
                            args![&mut self.u, &mut self.u0, &self.res, &self.dt, uniform(&p)],
                        )?,
                        Some(lo) => update.launch(
                            [ni, nj],
                            args![&mut self.u, &mut self.u0, &self.res, &self.dt, &mut lo.u, &mut lo.u0, uniform(&p)],
                        )?,
                    }
                }
            }
            Kernels::Guarded { flux, residual_update, hist_final } => {
                p.aux = (self.ni.div_ceil(32) * self.nj.div_ceil(4)) as u32; // workgroups of `residual_update`
                for (stage, (ra, rb)) in RK3.into_iter().enumerate().map(|(k, c)| (k as u32 + 1, c)) {
                    p.stage = stage;
                    (p.ra, p.rb) = (ra, rb);
                    match &self.lo {
                        Some(lo) if ext => flux.launch(
                            [ni + 1, 2 * nj + 1],
                            args![&self.u, &lo.u, &self.iface, &self.jface, &mut self.fi, &mut self.fj, uniform(&p)],
                        )?,
                        _ => flux.launch(
                            [ni + 1, 2 * nj + 1],
                            args![&self.u, &self.iface, &self.jface, &mut self.fi, &mut self.fj, uniform(&p)],
                        )?,
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
    /// per-iteration density residual (L2 of the stage-1 `d rho/dt` over sqrt(N), as in bowshock-cfd). This is the
    /// only synchronisation point.
    pub fn run(&mut self, n: usize) -> Result<Vec<f64>> {
        assert!(n >= 1 && n <= MAX_CHUNK);
        for s in 0..n {
            self.record_step(Some(s as u32))?;
        }
        let h = self.hist.to_vec()?;
        let cells = (self.ni * self.nj) as f64;
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
        Ok(self.rho_res.norm()? as f64 / ((self.ni * self.nj) as f64).sqrt())
    }

    /// Records `n` iterations without history kernel and without waiting (benchmark helper; Luma
    /// submits a batch every 256 dispatches, so the GPU already runs while recording continues).
    pub fn record(&mut self, n: usize) -> Result<()> {
        for _ in 0..n {
            self.record_step(None)?;
        }
        Ok(())
    }

    /// Conserved state, `ni*nj`, same layout as `bowshock_cfd::solver::Solver::u` (the f32 high part in the compensated
    /// precision, which is what the flux kernels see).
    pub fn state(&self) -> Result<Vec<[f32; 4]>> {
        self.u.to_vec()
    }

    /// Conserved state as f64: `hi + lo` in the compensated precision.
    pub fn state_f64(&self) -> Result<Vec<[f64; 4]>> {
        let hi = self.u.to_vec()?;
        let lo = match &self.lo {
            Some(lo) => Some(lo.u.to_vec()?),
            None => None,
        };
        Ok(hi
            .iter()
            .enumerate()
            .map(|(c, h)| [0, 1, 2, 3].map(|k| h[k] as f64 + lo.as_ref().map_or(0.0, |l| l[c][k] as f64)))
            .collect())
    }

    pub fn precision(&self) -> Precision {
        self.prec
    }

    /// Stage-1 `d rho / dt` of every cell from the last iteration (diagnostics).
    pub fn rho_residual_field(&self) -> Result<Vec<f32>> {
        self.rho_res.to_vec()
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
pub fn eval_faces(
    states: &[[[f32; 4]; 4]],
    normals: &[[f32; 2]],
    limiter: Limiter,
) -> Result<Vec<[[f32; 4]; 4]>> {
    let ctx = luma::default_context()?;
    let n = states.len();
    assert_eq!(n, normals.len());
    let flat: Vec<[f32; 4]> = states.iter().flatten().copied().collect();
    let st = Array::from_slice_on(&ctx, &flat, flat.len())?;
    let nm = Array::from_slice_on(&ctx, normals, n)?;
    let mut out = Array::<[f32; 4]>::zeros_on(&ctx, 4 * n)?;
    let kernel = build(&ctx, shaders::FACE_TEST, "face_test")?;
    let p = Params {
        ni: n as u32,
        nj: 1,
        limiter: limiter_code(limiter),
        stage: 1,
        cfl: 0.0,
        fs_rho: 0.0,
        fs_u: 0.0,
        fs_v: 0.0,
        fs_p: 0.0,
        bc0: 0,
        bc1: 0,
        bc2: 0,
        bc3: 0,
        slot: 0,
        aux: 0,
        zero: 0,
        ra: 1.0,
        rb: 0.0,
        pad2: 0,
        pad3: 0,
        fs_lo_rho: 0.0,
        fs_lo_u: 0.0,
        fs_lo_v: 0.0,
        fs_lo_p: 0.0,
    };
    kernel.launch([n as u32], args![&st, &nm, &mut out, uniform(&p)])?;
    let o = out.to_vec()?;
    Ok(o.chunks(4).map(|c| [c[0], c[1], c[2], c[3]]).collect())
}

/// As [`eval_faces`] with the double-float kernels of [`Precision::Extended`]; the result is `hi + lo` in f64.
pub fn eval_faces_ext(
    states: &[[[f32; 4]; 4]],
    normals: &[[f32; 2]],
    limiter: Limiter,
) -> Result<Vec<[[f64; 4]; 4]>> {
    let ctx = luma::default_context()?;
    let n = states.len();
    assert_eq!(n, normals.len());
    let flat: Vec<[f32; 4]> = states.iter().flatten().copied().collect();
    let st = Array::from_slice_on(&ctx, &flat, flat.len())?;
    let nm = Array::from_slice_on(&ctx, normals, n)?;
    let mut out = Array::<[f32; 4]>::zeros_on(&ctx, 8 * n)?;
    let kernel = build_ext(&ctx, shaders_ext::FACE_TEST_EXT, "face_test_ext")?;
    let p = Params {
        ni: n as u32,
        nj: 1,
        limiter: limiter_code(limiter),
        stage: 1,
        cfl: 0.0,
        fs_rho: 0.0,
        fs_u: 0.0,
        fs_v: 0.0,
        fs_p: 0.0,
        bc0: 0,
        bc1: 0,
        bc2: 0,
        bc3: 0,
        slot: 0,
        aux: 0,
        zero: 0,
        ra: 1.0,
        rb: 0.0,
        pad2: 0,
        pad3: 0,
        fs_lo_rho: 0.0,
        fs_lo_u: 0.0,
        fs_lo_v: 0.0,
        fs_lo_p: 0.0,
    };
    kernel.launch([n as u32], args![&st, &nm, &mut out, uniform(&p)])?;
    let o = out.to_vec()?;
    Ok(o.chunks(8).map(|c| [0, 1, 2, 3].map(|k| [0, 1, 2, 3].map(|m| c[k][m] as f64 + c[k + 4][m] as f64))).collect())
}
