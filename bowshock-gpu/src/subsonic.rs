//! Subsonic case on the GPU: `bowshock_cfd::subsonic::SubsonicCase` (nose-cylinder body with a flat base and wake, or on a sting, on the
//! large-domain grid with characteristic far field) iterated by the GPU solver with the convergence scheme of the case (line-implicit
//! by default, [`crate::shaders_impl`]), the low-Mach flux correction and the solid-block / base-wall handling of the kernels, with the
//! stopping rules and the post-processing of `bowshock_cfd::subsonic::run_on`.
//!
//! The GPU records [`SYNC`] iterations back to back and reads the residual history. The base-pressure history of the CPU loop (every
//! `HIST_EVERY` = 50 iterations, only with a base) needs the state: it is downloaded into a host `Solver` (only its `u` is used) once per
//! chunk when the case has a base, and never otherwise. The residual-based stopping rules run on the history iteration by iteration, so
//! the reported stop iteration is the CPU-equivalent one; the GPU may execute up to `SYNC - 1` further iterations (`iters_exec`).

use crate::{GpuSolver, Mode, Precision, Result};
use bowshock_cfd::gas::freestream;
use bowshock_cfd::solver::Solver;
use bowshock_cfd::subsonic::{base_state, finish, stop_reason, SubsonicCase, SubsonicResult, HIST_EVERY};
use std::time::Instant;

/// Iterations between two synchronisations; equal to the base-pressure sampling interval of `bowshock_cfd::subsonic::run_on`.
pub const SYNC: usize = HIST_EVERY;

/// Result of one GPU run plus its cost.
pub struct GpuSubsonicRun {
    /// Same fields as the CPU result (post-processed with `bowshock_cfd::subsonic::finish` on the final GPU state);
    /// `iters` / `residual` end at the CPU-equivalent stopping iteration.
    pub result: SubsonicResult,
    /// Iterations after which the stopping rule fired / iterations actually executed (a multiple of [`SYNC`]).
    pub iters_stop: usize,
    pub iters_exec: usize,
    /// Wall time of the iteration loop (kernels, residual-history read-back, base-pressure sampling).
    pub loop_seconds: f64,
}

impl GpuSubsonicRun {
    pub fn us_per_iter(&self) -> f64 {
        1e6 * self.loop_seconds / self.iters_exec as f64
    }
}

/// GPU solver plus the host solver (grid, config, block, and a state buffer for post-processing) of one subsonic case.
pub struct GpuSubsonic {
    pub gpu: GpuSolver,
    pub host: Solver,
    /// Kernel build + geometry upload, seconds.
    pub setup_seconds: f64,
}

impl GpuSubsonic {
    pub fn new(case: &SubsonicCase, mode: Mode, prec: Precision) -> Result<GpuSubsonic> {
        let host = case.build_solver();
        let t0 = Instant::now();
        let gpu = GpuSolver::from_host(&luma::default_context()?, &host, mode, prec)?;
        Ok(GpuSubsonic { gpu, host, setup_seconds: t0.elapsed().as_secs_f64() })
    }

    /// Copies the GPU state into the host solver's `u`.
    pub fn download(&mut self) -> Result<()> {
        self.host.u = self.gpu.state_f64()?;
        Ok(())
    }

    /// Iterate to convergence on the GPU: the same procedure and arguments as `bowshock_cfd::subsonic::run_on`
    /// (`warm`: keep the field, only change the free stream; `r_ref`: reference residual of the drop criterion).
    pub fn run_on(&mut self, case: &SubsonicCase, warm: bool, r_ref: Option<f64>) -> Result<GpuSubsonicRun> {
        let fs = freestream(case.mach);
        for cfg in [self.gpu.config_mut(), &mut self.host.cfg] {
            cfg.freestream = fs;
            cfg.cfl = case.cfl;
            cfg.limiter = case.limiter;
            cfg.first_order_iters = case.first_order_iters;
        }
        self.host.low_mach = case.low_mach;
        self.gpu.set_low_mach(case.low_mach);
        if warm {
            self.gpu.iter = case.first_order_iters;
            self.gpu.restart_ramp();
        } else {
            self.gpu.reset()?; // free stream of `case.mach`, iteration 0, CFL ramp restarted
        }
        self.gpu.context().sync()?;

        let rb2 = case.body.body.radius().powi(2);
        let mut residual = Vec::new();
        let mut hist: Vec<(usize, f64)> = Vec::new();
        let mut rmax: f64 = 0.0;
        let mut reason = "max iterations";
        let mut iters_stop = None;
        let skip = if warm { 0 } else { 20 };
        let mut done = 0usize;
        let t0 = Instant::now();
        while done < case.max_iter && iters_stop.is_none() {
            let n = SYNC.min(case.max_iter - done);
            let h = self.gpu.run(n)?;
            // base-pressure sample of iteration `done + SYNC` (needs the state at exactly the end of a full chunk)
            let cp_base = if case.base && n == SYNC {
                self.download()?;
                Some(base_state(&self.host, rb2).2)
            } else {
                None
            };
            for (k, r) in h.into_iter().enumerate() {
                let it = done + k;
                residual.push(r);
                if it >= skip {
                    rmax = rmax.max(r);
                }
                if case.base && it % HIST_EVERY == HIST_EVERY - 1 {
                    hist.push((it + 1, cp_base.expect("full chunk")));
                }
                if let Some(why) = stop_reason(case, it, r, r_ref.unwrap_or(rmax), skip, &hist) {
                    reason = why;
                    iters_stop = Some(it + 1);
                    break;
                }
            }
            done += n;
        }
        let loop_seconds = t0.elapsed().as_secs_f64();
        self.download()?;
        let result = finish(case, &self.host, residual, rmax, hist, reason);
        Ok(GpuSubsonicRun { iters_stop: iters_stop.unwrap_or(done), iters_exec: done, loop_seconds, result })
    }
}

/// Cold-start GPU run (fresh solver): the counterpart of `bowshock_cfd::subsonic::run_subsonic`.
pub fn run_subsonic(case: &SubsonicCase, mode: Mode, prec: Precision) -> Result<GpuSubsonicRun> {
    GpuSubsonic::new(case, mode, prec)?.run_on(case, false, None)
}
