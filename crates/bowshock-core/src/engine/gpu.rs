//! GPU runner of a [`Case`]: the same procedure, stopping rules and post-processing as the CPU [`crate::engine::run::run_on`], with the
//! iterations executed by `bowshock_cfd::gpu::GpuSolver` (Luma array ops).
//!
//! The GPU records [`SYNC`] iterations back to back and reads the residual history. The monitor of the CPU loop (every
//! [`MONITOR_EVERY`] iterations, which is the synchronisation interval) is sampled once per full chunk: the base-pressure
//! history of an inviscid `Wake::Base` case by a GPU kernel that reads back only the two cells behind the base, the forebody CD
//! of an inviscid `Wake::Sting` case and the total CD of a viscous case (friction included, which reads only the mean state) from
//! the state downloaded into the host solver. The residual-based stopping rules run on the history
//! iteration by iteration, so the reported stop iteration is the CPU-equivalent one; the GPU may execute up to `SYNC - 1`
//! further iterations (`iters_exec`) and the final wall state is the one after those.

use crate::engine::case::Case;

use crate::engine::meshing::Wake;
use crate::engine::numerics::MONITOR_EVERY;
use crate::engine::result::{PointResult, StopReason};
use crate::engine::run::{finish, monitor, report, stop_reason, wall_distribution};
use crate::engine::timing::{record_timing, PointTiming};
use crate::error::Error;
use bowshock_cfd::gas::{cons_to_prim, freestream};
use bowshock_cfd::gpu::{GpuSolver, Precision};
use bowshock_cfd::solver::Solver;
use std::time::Instant;

type Result<T, E = Error> = std::result::Result<T, E>;

/// Iterations between two synchronisations; equal to the monitor sampling interval of the CPU runner.
pub const SYNC: usize = MONITOR_EVERY;

/// GPU solver plus the host solver (grid, config, block, and a state buffer for post-processing) of one case.
pub struct GpuRunner {
    pub gpu: GpuSolver,
    pub host: Solver,
    /// Kernel build + geometry upload, seconds.
    pub setup_seconds: f64,
    /// Host grid + solver construction, default-context creation and `from_host`, seconds (parts of the set-up).
    pub build_seconds: f64,
    pub ctx_seconds: f64,
    pub upload_seconds: f64,
}

/// Result of one GPU run plus its cost.
pub struct GpuRun {
    /// Same fields as the CPU result (post-processed with [`finish`] on the final GPU state);
    /// `iters` / `residual` end at the CPU-equivalent stopping iteration.
    pub result: PointResult,
    /// Iterations after which the stopping rule fired / iterations actually executed (a multiple of [`SYNC`]).
    pub iters_stop: usize,
    pub iters_exec: usize,
    /// Wall time of the iteration loop (kernels, residual-history and state read-back, monitor sampling).
    pub loop_seconds: f64,
}

impl GpuRun {
    /// Microseconds per executed iteration.
    pub fn us_per_iter(&self) -> f64 {
        1e6 * self.loop_seconds / self.iters_exec as f64
    }
}

impl GpuRunner {
    /// Builds the host solver of `case` and uploads it to the default GPU context.
    pub fn new(case: &Case, prec: Precision) -> Result<GpuRunner> {
        let t_build = Instant::now();
        let host = case.try_build_solver()?;
        let build_seconds = t_build.elapsed().as_secs_f64();
        let t0 = Instant::now();
        let ctx = luma::default_context()?;
        let ctx_seconds = t0.elapsed().as_secs_f64();
        // The native CPU solver is faster than the GPU kernels on a software rasterizer, so `Backend::Auto` falls back to it.
        if ctx.is_cpu() {
            return Err(Error::Gpu("no GPU: Luma opened a software (CPU) Vulkan device".into()));
        }
        let t1 = Instant::now();
        let gpu = GpuSolver::from_host(&ctx, &host, prec)?;
        let upload_seconds = t1.elapsed().as_secs_f64();
        Ok(GpuRunner { gpu, host, setup_seconds: t0.elapsed().as_secs_f64(), build_seconds, ctx_seconds, upload_seconds })
    }

    /// Copies the GPU state into the host solver's `u`.
    pub fn download(&mut self) -> Result<()> {
        self.host.u = self.gpu.state_f64()?;
        Ok(())
    }

    /// Iterate to convergence on the GPU: the same procedure and arguments as [`crate::engine::run::run_on`]
    /// (`warm`: keep the field, only change the free stream; `r_ref`: reference residual of the drop criterion).
    pub fn run_on(&mut self, case: &Case, warm: bool, r_ref: Option<f64>) -> Result<GpuRun> {
        let t_run = Instant::now();
        let mut tm = PointTiming { dim: 2, mach: case.mach, warm, cells: self.host.grid.ni * self.host.grid.nj, ..Default::default() };
        if !warm {
            (tm.build_s, tm.ctx_s, tm.upload_s) = (self.build_seconds, self.ctx_seconds, self.upload_seconds);
        }
        let t_reset = Instant::now();
        let n = case.numerics;
        let fs = freestream(case.mach);
        let cfg = &mut self.host.cfg;
        cfg.freestream = fs;
        cfg.cfl = n.cfl;
        cfg.limiter = n.limiter;
        cfg.first_order_iters = n.first_order_iters;
        self.gpu.set_config(&self.host.cfg)?;
        self.host.low_mach = n.low_mach;
        self.gpu.set_low_mach(n.low_mach);
        if self.gpu.scheme() != n.scheme {
            self.gpu.set_scheme(n.scheme)?;
            self.host.set_scheme(n.scheme);
        }
        if warm {
            self.gpu.set_iter(n.first_order_iters);
            self.gpu.restart_ramp()?; // implicit scheme: CFL ramp restarts at every Mach number
        } else {
            self.gpu.reset()?; // free stream of `case.mach`, iteration 0, ramp restarted
        }
        self.gpu.context().sync()?;
        tm.reset_s = t_reset.elapsed().as_secs_f64();

        let viscous = case.viscous.is_some();
        let sampler = (case.wake == Wake::Base && !viscous).then(|| BaseSampler::new(&self.host, case.body.body.radius().powi(2)));
        let sample = case.wake == Wake::Base || viscous || n.monitor_tol.is_some() || n.drag_rel_tol.is_some();
        let mut residual = Vec::new();
        let mut hist: Vec<(usize, f64)> = Vec::new();
        let mut rmax: f64 = 0.0;
        let mut reason = StopReason::MaxIterations;
        let mut iters_stop = None;
        let skip = if warm { 0 } else { 20 };
        let mut done = 0usize;
        let t0 = Instant::now();
        while done < n.max_iter && iters_stop.is_none() {
            let len = SYNC.min(n.max_iter - done);
            let t_chunk = Instant::now();
            let h = self.gpu.run(len)?;
            let dt_chunk = t_chunk.elapsed().as_secs_f64();
            if done == 0 {
                tm.first_chunk_s = dt_chunk;
            } else {
                tm.chunks_rest += 1;
                tm.rest_chunks_s += dt_chunk;
            }
            // monitor sample of iteration `done + SYNC` (needs the state at exactly the end of a full chunk)
            let value = if sample && len == SYNC {
                Some(match &sampler {
                    Some(sampler) => {
                        let t = Instant::now();
                        let cells = self.gpu.base_cells()?;
                        tm.downloads += 1;
                        tm.download_s += t.elapsed().as_secs_f64();
                        sampler.cp_base(&cells)
                    }
                    None => {
                        let t = Instant::now();
                        self.download()?;
                        tm.downloads += 1;
                        tm.download_s += t.elapsed().as_secs_f64();
                        let t = Instant::now();
                        // total CD incl. friction (viscous): needs only the mean state (`u`) on the host, which `download` provides
                        let v = if viscous { monitor(case, &self.host) } else { wall_distribution(&case.body, &self.host).1 };
                        tm.forces_s += t.elapsed().as_secs_f64();
                        v
                    }
                })
            } else {
                None
            };
            // whether the host solver holds the state of the end of this chunk
            let downloaded = value.is_some() && sampler.is_none();
            let last_r = h.last().copied();
            for (k, r) in h.into_iter().enumerate() {
                let it = done + k;
                residual.push(r);
                if it >= skip {
                    rmax = rmax.max(r);
                }
                if sample && it % SYNC == SYNC - 1 {
                    hist.push((it + 1, value.expect("full chunk")));
                }
                if let Some(why) = stop_reason(case, it, r, r_ref.unwrap_or(rmax), skip, &hist) {
                    reason = why;
                    iters_stop = Some(it + 1);
                    break;
                }
            }
            done += len;
            if iters_stop.is_none() && len == SYNC {
                if let Some(tap) = case.fields.as_ref().filter(|t| t.due(done)) {
                    // a snapshot of the run in flight: the state is read back only when the monitor has not already done so
                    if !downloaded {
                        self.download()?;
                    }
                    tap.send(&self.host, case.mach, done, case.wake);
                }
            }
            if iters_stop.is_none() && len == SYNC && report(case, done, last_r.unwrap_or(f64::NAN), r_ref.unwrap_or(rmax), &hist).is_break() {
                reason = StopReason::Cancelled;
                iters_stop = Some(done);
            }
        }
        let loop_seconds = t0.elapsed().as_secs_f64();
        let t_final = Instant::now();
        self.download()?;
        let result = finish(case, &self.host, residual, rmax, hist, reason);
        tm.final_s = t_final.elapsed().as_secs_f64();
        tm.run_s = t_run.elapsed().as_secs_f64();
        tm.iters_stop = iters_stop.unwrap_or(done);
        tm.iters_exec = done;
        tm.stop = format!("{:?}", result.stop_reason);
        record_timing(tm);
        Ok(GpuRun { iters_stop: iters_stop.unwrap_or(done), iters_exec: done, loop_seconds, result })
    }
}

/// Host side of the GPU base-pressure sample: the geometric weights of [`crate::engine::run::base_state`] per base row (extrapolation
/// factor `d0 / (d1 - d0)` and area weight), applied in f64 to the two cells per row of [`GpuSolver::base_cells`].
pub struct BaseSampler {
    rows: Vec<(f64, f64)>,
    /// Leading entries of `base_cells()` that belong to the nozzle exit (not part of the base drag).
    skip: usize,
    p_inf: f64,
    q_inf: f64,
    s_sum: f64,
}

impl BaseSampler {
    /// Weights of the base rows of `s` (which must have a solid block); `rb2` is the squared reference radius.
    pub fn new(s: &Solver, rb2: f64) -> BaseSampler {
        let b = s.block().expect("a case with a base has a solid block");
        let (g, ni, fs) = (&s.grid, s.grid.ni, s.cfg.freestream);
        let ie = b.i_end;
        let j0 = s.nozzle().map_or(b.j_sting, |n| n.j_end);
        let rows: Vec<(f64, f64)> = (j0..b.j_end)
            .map(|j| {
                let (n0, n1) = (g.node(ie, j), g.node(ie, j + 1));
                let (xm, rm) = (n0.0, 0.5 * (n0.1 + n1.1));
                let d0 = (g.xc[j * ni + ie] - xm).hypot(g.rc[j * ni + ie] - rm);
                let d1 = (g.xc[j * ni + ie + 1] - xm).hypot(g.rc[j * ni + ie + 1] - rm);
                (d0 / (d1 - d0), (n1.1 * n1.1 - n0.1 * n0.1) / rb2)
            })
            .collect();
        let s_sum = rows.iter().map(|r| r.1).sum();
        BaseSampler { rows, skip: j0 - b.j_sting, p_inf: fs[3], q_inf: 0.5 * fs[0] * fs[1] * fs[1], s_sum }
    }

    /// Area-mean Cp of the extrapolated base-wall pressure (`base_state(..).2`).
    pub fn cp_base(&self, cells: &[[[f64; 4]; 2]]) -> f64 {
        let mean: f64 = self
            .rows
            .iter()
            .zip(cells.iter().skip(self.skip))
            .map(|(&(k, ds), c)| {
                let (p0, p1) = (cons_to_prim(&c[0])[3], cons_to_prim(&c[1])[3]);
                (p0 + (p0 - p1) * k - self.p_inf) / self.q_inf * ds
            })
            .sum();
        mean / self.s_sum
    }
}
