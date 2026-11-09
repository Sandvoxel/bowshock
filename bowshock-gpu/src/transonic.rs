//! Transonic case on the GPU: the nose-cylinder body on the large-domain grid with characteristic far-field BCs
//! (`bowshock_cfd::transonic`), iterated by the GPU solver with the stopping rules and the post-processing of
//! `bowshock_cfd::transonic::run_on` (a warm-started Mach sweep keeps the converged field and only changes the free stream).
//!
//! The GPU records [`SYNC`] iterations back to back, reads the residual history, copies the state into a host
//! `bowshock_cfd::solver::Solver` (only its `u` is used) and takes the forebody-CD sample of the CPU loop (every 50 iterations,
//! which is the synchronisation interval) from it with `bowshock_cfd::transonic::wall_distribution`. The residual-based stopping
//! rules are evaluated on the history iteration by iteration, so the reported stop iteration is the CPU-equivalent one; the GPU
//! may execute up to `SYNC - 1` further iterations (`iters_exec`) and the final wall state is the one after those.

use crate::{GpuSolver, Mode, Precision, Result};
use bowshock_cfd::gas::freestream;
use bowshock_cfd::solver::Solver;
use bowshock_cfd::transonic::{finish, stop_reason, wall_distribution, TransonicCase, TransonicResult};
use std::time::Instant;

/// Iterations between two synchronisations; equal to the CD sampling interval of `bowshock_cfd::transonic::run_on`.
pub const SYNC: usize = 50;

/// Result of one GPU run plus its cost.
pub struct GpuTransonicRun {
    /// Same fields as the CPU result (post-processed with `bowshock_cfd::transonic::finish` on the final GPU state);
    /// `iters` / `residual` end at the CPU-equivalent stopping iteration.
    pub result: TransonicResult,
    /// Iterations after which the stopping rule fired / iterations actually executed (a multiple of [`SYNC`]).
    pub iters_stop: usize,
    pub iters_exec: usize,
    /// Wall time of the iteration loop (kernels, residual-history and state read-back, CD sampling).
    pub loop_seconds: f64,
}

impl GpuTransonicRun {
    pub fn us_per_iter(&self) -> f64 {
        1e6 * self.loop_seconds / self.iters_exec as f64
    }
}

/// GPU solver plus the host solver (grid, config, and a state buffer for post-processing) of one transonic case.
pub struct GpuTransonic {
    pub gpu: GpuSolver,
    pub host: Solver,
    /// Kernel build + geometry upload, seconds.
    pub setup_seconds: f64,
}

impl GpuTransonic {
    pub fn new(case: &TransonicCase, mode: Mode, prec: Precision) -> Result<GpuTransonic> {
        let host = case.build_solver();
        let t0 = Instant::now();
        let gpu = GpuSolver::new_with(&luma::default_context()?, &host.grid, &host.cfg, mode, prec)?;
        Ok(GpuTransonic { gpu, host, setup_seconds: t0.elapsed().as_secs_f64() })
    }

    /// Copies the GPU state into the host solver's `u`.
    pub fn download(&mut self) -> Result<()> {
        self.host.u = self.gpu.state_f64()?;
        Ok(())
    }

    /// Iterate to convergence on the GPU: the same procedure and arguments as `bowshock_cfd::transonic::run_on`
    /// (`warm`: keep the field, only change the free stream; `r_ref`: reference residual of the drop criterion).
    pub fn run_on(&mut self, case: &TransonicCase, warm: bool, r_ref: Option<f64>) -> Result<GpuTransonicRun> {
        let fs = freestream(case.mach);
        for cfg in [self.gpu.config_mut(), &mut self.host.cfg] {
            cfg.freestream = fs;
            cfg.cfl = case.cfl;
            cfg.limiter = case.limiter;
            cfg.first_order_iters = case.first_order_iters;
        }
        if warm {
            self.gpu.iter = case.first_order_iters;
        } else {
            self.gpu.reset()?; // free stream of `case.mach`, iteration 0
        }
        self.gpu.context().sync()?;

        let mut residual = Vec::new();
        let mut cd_hist: Vec<(usize, f64)> = Vec::new();
        let mut rmax: f64 = 0.0;
        let mut reason = "max iterations";
        let mut iters_stop = None;
        let skip = if warm { 0 } else { 20 };
        let mut done = 0usize;
        let t0 = Instant::now();
        while done < case.max_iter && iters_stop.is_none() {
            let n = SYNC.min(case.max_iter - done);
            let hist = self.gpu.run(n)?;
            // CD sample of iteration `done + SYNC` (needs the state at exactly the end of a full chunk)
            let cd = if n == SYNC {
                self.download()?;
                Some(wall_distribution(&case.body, &self.host).1)
            } else {
                None
            };
            for (k, r) in hist.into_iter().enumerate() {
                let it = done + k;
                residual.push(r);
                if it >= skip {
                    rmax = rmax.max(r);
                }
                if it % SYNC == SYNC - 1 {
                    cd_hist.push((it + 1, cd.expect("full chunk")));
                }
                if let Some(why) = stop_reason(case, it, r, r_ref.unwrap_or(rmax), skip, &cd_hist) {
                    reason = why;
                    iters_stop = Some(it + 1);
                    break;
                }
            }
            done += n;
        }
        let loop_seconds = t0.elapsed().as_secs_f64();
        self.download()?;
        let result = finish(case, &self.host, residual, rmax, cd_hist, reason);
        Ok(GpuTransonicRun { iters_stop: iters_stop.unwrap_or(done), iters_exec: done, loop_seconds, result })
    }
}

/// Cold-start GPU run (fresh solver): the counterpart of `bowshock_cfd::transonic::run_transonic`.
pub fn run_transonic(case: &TransonicCase, mode: Mode, prec: Precision) -> Result<GpuTransonicRun> {
    GpuTransonic::new(case, mode, prec)?.run_on(case, false, None)
}
