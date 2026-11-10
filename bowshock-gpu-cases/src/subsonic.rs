//! Subsonic case on the GPU: `bowshock_cases::subsonic::SubsonicCase` (nose-cylinder body with a flat base and wake, or on a sting, on the
//! large-domain grid with characteristic far field) iterated by the GPU solver with the convergence scheme of the case (line-implicit
//! by default, [`bowshock_gpu::shaders_impl`]), the low-Mach flux correction and the solid-block / base-wall handling of the kernels, with the
//! stopping rules and the post-processing of `bowshock_cases::subsonic::run_on`.
//!
//! The GPU records [`SYNC`] iterations back to back and reads the residual history. The base-pressure history of the CPU loop (every
//! `HIST_EVERY` = 50 iterations, only with a base) is sampled by a GPU kernel (the pressures of the two
//! cells behind the base, read back once per chunk when the case has a base, never otherwise); the full state is downloaded once, at the
//! end, for the post-processing. The residual-based stopping rules run on the history iteration by iteration, so
//! the reported stop iteration is the CPU-equivalent one; the GPU may execute up to `SYNC - 1` further iterations (`iters_exec`).

use bowshock_gpu::{GpuSolver, Mode, Precision, Result};
use bowshock_core::gas::{cons_to_prim, freestream};
use bowshock_core::solver::Solver;
use bowshock_cases::subsonic::{finish, stop_reason, SubsonicCase, SubsonicResult, HIST_EVERY};
use std::time::Instant;

/// Iterations between two synchronisations; equal to the base-pressure sampling interval of `bowshock_cases::subsonic::run_on`.
pub const SYNC: usize = HIST_EVERY;

/// Result of one GPU run plus its cost.
pub struct GpuSubsonicRun {
    /// Same fields as the CPU result (post-processed with `bowshock_cases::subsonic::finish` on the final GPU state);
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

    /// Iterate to convergence on the GPU: the same procedure and arguments as `bowshock_cases::subsonic::run_on`
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
        let sampler = case.base.then(|| BaseSampler::new(&self.host, rb2));
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
            let cp_base = if let (Some(sampler), true) = (&sampler, n == SYNC) {
                let pr = self.gpu.base_cells()?;
                Some(sampler.cp_base(&pr))
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

/// Host side of the GPU base-pressure sample: the geometric weights of `bowshock_cases::subsonic::base_state` per base row (extrapolation
/// factor `d0 / (d1 - d0)` and area weight), applied in f64 to the two cells per row of [`GpuSolver::base_cells`].
pub struct BaseSampler {
    rows: Vec<(f64, f64)>,
    p_inf: f64,
    q_inf: f64,
    s_sum: f64,
}

impl BaseSampler {
    pub fn new(s: &Solver, rb2: f64) -> BaseSampler {
        let b = s.block.expect("a case with a base has a solid block");
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

/// Cold-start GPU run (fresh solver): the counterpart of `bowshock_cases::subsonic::run_subsonic`.
pub fn run_subsonic(case: &SubsonicCase, mode: Mode, prec: Precision) -> Result<GpuSubsonicRun> {
    GpuSubsonic::new(case, mode, prec)?.run_on(case, false, None)
}
