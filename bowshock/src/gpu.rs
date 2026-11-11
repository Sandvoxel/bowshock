//! GPU runner of a [`Case`]: the same procedure, stopping rules and post-processing as the CPU [`crate::run::run_on`], with the
//! iterations executed by `bowshock_gpu::GpuSolver`.
//!
//! The GPU records [`SYNC`] iterations back to back and reads the residual history. The monitor of the CPU loop (every
//! [`MONITOR_EVERY`] iterations, which is the synchronisation interval) is sampled once per full chunk: the base-pressure
//! history of a `Wake::Base` case by a GPU kernel that reads back only the two cells behind the base, the forebody CD of a
//! `Wake::Sting` case from the state downloaded into the host solver. The residual-based stopping rules run on the history
//! iteration by iteration, so the reported stop iteration is the CPU-equivalent one; the GPU may execute up to `SYNC - 1`
//! further iterations (`iters_exec`) and the final wall state is the one after those.

use crate::case::{Case, Wake};
use crate::numerics::MONITOR_EVERY;
use crate::result::{PointResult, StopReason};
use crate::run::{finish, stop_reason, wall_distribution};
use bowshock_core::gas::{cons_to_prim, freestream};
use bowshock_core::solver::Solver;
use bowshock_gpu::{GpuSolver, Mode, Precision, Result};
use std::time::Instant;

/// Iterations between two synchronisations; equal to the monitor sampling interval of the CPU runner.
pub const SYNC: usize = MONITOR_EVERY;

/// GPU solver plus the host solver (grid, config, block, and a state buffer for post-processing) of one case.
pub struct GpuRunner {
    pub gpu: GpuSolver,
    pub host: Solver,
    /// Kernel build + geometry upload, seconds.
    pub setup_seconds: f64,
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
    pub fn new(case: &Case, mode: Mode, prec: Precision) -> Result<GpuRunner> {
        let host = case.build_solver();
        let t0 = Instant::now();
        let gpu = GpuSolver::from_host(&luma::default_context()?, &host, mode, prec)?;
        Ok(GpuRunner { gpu, host, setup_seconds: t0.elapsed().as_secs_f64() })
    }

    /// Copies the GPU state into the host solver's `u`.
    pub fn download(&mut self) -> Result<()> {
        self.host.u = self.gpu.state_f64()?;
        Ok(())
    }

    /// Iterate to convergence on the GPU: the same procedure and arguments as [`crate::run::run_on`]
    /// (`warm`: keep the field, only change the free stream; `r_ref`: reference residual of the drop criterion).
    pub fn run_on(&mut self, case: &Case, warm: bool, r_ref: Option<f64>) -> Result<GpuRun> {
        let n = case.numerics;
        let fs = freestream(case.mach);
        for cfg in [self.gpu.config_mut(), &mut self.host.cfg] {
            cfg.freestream = fs;
            cfg.cfl = n.cfl;
            cfg.limiter = n.limiter;
            cfg.first_order_iters = n.first_order_iters;
        }
        self.host.low_mach = n.low_mach;
        self.gpu.set_low_mach(n.low_mach);
        if self.gpu.scheme() != n.scheme {
            self.gpu.set_scheme(n.scheme)?;
            self.host.set_scheme(n.scheme);
        }
        if warm {
            self.gpu.iter = n.first_order_iters;
            self.gpu.restart_ramp(); // implicit scheme: CFL ramp restarts at every Mach number
        } else {
            self.gpu.reset()?; // free stream of `case.mach`, iteration 0, ramp restarted
        }
        self.gpu.context().sync()?;

        let sampler = (case.wake == Wake::Base).then(|| BaseSampler::new(&self.host, case.body.body.radius().powi(2)));
        let sample = case.wake == Wake::Base || n.monitor_tol.is_some();
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
            let h = self.gpu.run(len)?;
            // monitor sample of iteration `done + SYNC` (needs the state at exactly the end of a full chunk)
            let value = if sample && len == SYNC {
                Some(match &sampler {
                    Some(sampler) => sampler.cp_base(&self.gpu.base_cells()?),
                    None => {
                        self.download()?;
                        wall_distribution(&case.body, &self.host).1
                    }
                })
            } else {
                None
            };
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
        }
        let loop_seconds = t0.elapsed().as_secs_f64();
        self.download()?;
        let result = finish(case, &self.host, residual, rmax, hist, reason);
        Ok(GpuRun { iters_stop: iters_stop.unwrap_or(done), iters_exec: done, loop_seconds, result })
    }
}

/// Host side of the GPU base-pressure sample: the geometric weights of [`crate::run::base_state`] per base row (extrapolation
/// factor `d0 / (d1 - d0)` and area weight), applied in f64 to the two cells per row of [`GpuSolver::base_cells`].
pub struct BaseSampler {
    rows: Vec<(f64, f64)>,
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
        let rows: Vec<(f64, f64)> = (b.j_sting..b.j_end)
            .map(|j| {
                let (n0, n1) = (g.node(ie, j), g.node(ie, j + 1));
                let (xm, rm) = (n0.0, 0.5 * (n0.1 + n1.1));
                let d0 = (g.xc[j * ni + ie] - xm).hypot(g.rc[j * ni + ie] - rm);
                let d1 = (g.xc[j * ni + ie + 1] - xm).hypot(g.rc[j * ni + ie + 1] - rm);
                (d0 / (d1 - d0), (n1.1 * n1.1 - n0.1 * n0.1) / rb2)
            })
            .collect();
        let s_sum = rows.iter().map(|r| r.1).sum();
        BaseSampler { rows, p_inf: fs[3], q_inf: 0.5 * fs[0] * fs[1] * fs[1], s_sum }
    }

    /// Area-mean Cp of the extrapolated base-wall pressure (`base_state(..).2`).
    pub fn cp_base(&self, cells: &[[[f64; 4]; 2]]) -> f64 {
        let mean: f64 = self
            .rows
            .iter()
            .zip(cells)
            .map(|(&(k, ds), c)| {
                let (p0, p1) = (cons_to_prim(&c[0])[3], cons_to_prim(&c[1])[3]);
                (p0 + (p0 - p1) * k - self.p_inf) / self.q_inf * ds
            })
            .sum();
        mean / self.s_sum
    }
}
