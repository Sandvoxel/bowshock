//! Case-independent run loops: CPU f64 reference loop and GPU loop with identical stopping criteria, field
//! comparison. Case modules ([`crate::cone`], [`crate::ogive`]) add their post-processing on top.
//!
//! The convergence criteria are those of `bowshock_cases::cone::run_cone` / `bowshock_cases::ogive::run_ogive`
//! (density-residual drop of `drop_orders` from its maximum after iteration 20, plateau test from iteration
//! 8000, max iterations, non-finite), see [`Monitor`]. The CPU loop uses the public `Solver::step` so that the final
//! state is available for the field comparison.

use bowshock_gpu::{GpuSolver, Mode, Precision, Result};
use bowshock_core::gas::cons_to_prim;
use bowshock_core::solver::Solver;
use std::time::Instant;

/// Convergence monitor: feed it the density residual of every iteration in order.
pub struct Monitor {
    pub residual: Vec<f64>,
    pub rmax: f64,
    drop_orders: f64,
}

impl Monitor {
    pub fn new(drop_orders: f64) -> Self {
        Monitor { residual: Vec::new(), rmax: 0.0, drop_orders }
    }

    /// Records the residual of the next iteration; `Some(reason)` if the run is converged/stalled/diverged.
    pub fn push(&mut self, r: f64) -> Option<&'static str> {
        let it = self.residual.len();
        self.residual.push(r);
        if it >= 20 {
            self.rmax = self.rmax.max(r);
        }
        if !r.is_finite() {
            return Some("non-finite residual");
        }
        if it > 20 && self.rmax / r >= 10f64.powf(self.drop_orders) {
            return Some("residual drop reached");
        }
        if it >= 8000 && it % 1000 == 0 {
            let res = &self.residual;
            let m = |a: usize, b: usize| res[a..b].iter().map(|x| x.log10()).sum::<f64>() / (b - a) as f64;
            if m(it - 2000, it) - m(it - 4000, it - 2000) > -0.05 {
                return Some("residual plateau");
            }
        }
        None
    }

    /// Orders of magnitude between the peak (after iteration 20) and the last recorded residual.
    pub fn drop_orders(&self) -> f64 {
        match self.residual.last() {
            Some(&l) if self.rmax > 0.0 => (self.rmax / l).log10(),
            _ => 0.0,
        }
    }
}

/// Result of one iteration loop (CPU or GPU) on any case.
pub struct Run {
    /// Residual of every executed iteration.
    pub residual: Vec<f64>,
    /// Iteration count at which the stopping criterion fired (CPU-equivalent count).
    pub iters_stop: usize,
    /// Iterations actually executed (GPU: rounded up to a multiple of the sync interval).
    pub iters_exec: usize,
    pub reason: &'static str,
    pub drop_orders: f64,
    /// Wall time of the iteration loop (no setup, no post-processing).
    pub loop_seconds: f64,
    /// GPU only: one-time setup (kernel build + upload) and first-launch warm-up, seconds.
    pub setup_seconds: f64,
    pub warmup_seconds: f64,
    /// Final conserved state, `ni*nj`.
    pub u: Vec<[f64; 4]>,
}

impl Run {
    pub fn us_per_iter(&self) -> f64 {
        1e6 * self.loop_seconds / self.iters_exec as f64
    }
}

/// CPU f64 reference run (bowshock-core, rayon). The final state is copied to [`Run::u`]; `s` keeps it as well.
pub fn cpu_loop(s: &mut Solver, max_iter: usize, drop_orders: f64) -> Run {
    let mut mon = Monitor::new(drop_orders);
    let mut reason = "max iterations";
    let t0 = Instant::now();
    for _ in 0..max_iter {
        let r = s.step();
        if let Some(why) = mon.push(r) {
            reason = why;
            break;
        }
    }
    let loop_seconds = t0.elapsed().as_secs_f64();
    let n = mon.residual.len();
    Run {
        drop_orders: mon.drop_orders(),
        iters_stop: n,
        iters_exec: n,
        residual: mon.residual,
        reason,
        loop_seconds,
        setup_seconds: 0.0,
        warmup_seconds: 0.0,
        u: s.u.clone(),
    }
}

/// GPU run on the grid and configuration of `host`: kernels for `chunk` iterations are recorded back to back; the host
/// synchronises only to read the residual history after every chunk and tests the stopping criteria on it.
pub fn gpu_loop(host: &Solver, max_iter: usize, drop_orders: f64, chunk: usize, mode: Mode, prec: Precision) -> Result<Run> {
    let t0 = Instant::now();
    let mut g = GpuSolver::new_with(&luma::default_context()?, &host.grid, &host.cfg, mode, prec)?;
    let setup_seconds = t0.elapsed().as_secs_f64();
    // warm-up: first launches (driver-side pipeline work), then back to the free stream
    let t0 = Instant::now();
    g.run(1)?;
    g.reset()?;
    g.context().sync()?;
    let warmup_seconds = t0.elapsed().as_secs_f64();

    let mut mon = Monitor::new(drop_orders);
    let mut all = Vec::new();
    let mut reason = "max iterations";
    let mut iters_stop = None;
    let t0 = Instant::now();
    while all.len() < max_iter {
        let n = chunk.min(max_iter - all.len());
        for r in g.run(n)? {
            all.push(r);
            if iters_stop.is_none() {
                if let Some(why) = mon.push(r) {
                    reason = why;
                    iters_stop = Some(mon.residual.len());
                }
            }
        }
        if iters_stop.is_some() {
            break;
        }
    }
    let loop_seconds = t0.elapsed().as_secs_f64();
    let u = g.state_f64()?;
    Ok(Run {
        drop_orders: mon.drop_orders(),
        iters_stop: iters_stop.unwrap_or(all.len()),
        iters_exec: all.len(),
        residual: all,
        reason,
        loop_seconds,
        setup_seconds,
        warmup_seconds,
        u,
    })
}

/// Difference of two conserved fields (GPU vs CPU): max over cells of `|p_gpu - p_cpu| / p_inf`, RMS of the same, and
/// max over cells of `|rho_gpu - rho_cpu| / rho_cpu`.
#[derive(Debug, Clone, Copy)]
pub struct FieldDiff {
    pub max_dp_over_pinf: f64,
    pub rms_dp_over_pinf: f64,
    pub max_rel_rho: f64,
}

pub fn field_diff(gpu: &[[f64; 4]], cpu: &[[f64; 4]], p_inf: f64) -> FieldDiff {
    let (mut dp, mut dr, mut s2) = (0.0f64, 0.0f64, 0.0f64);
    for (a, b) in gpu.iter().zip(cpu) {
        let (wa, wb) = (cons_to_prim(a), cons_to_prim(b));
        let d = (wa[3] - wb[3]).abs() / p_inf;
        dp = dp.max(d);
        s2 += d * d;
        dr = dr.max((wa[0] - wb[0]).abs() / wb[0]);
    }
    FieldDiff { max_dp_over_pinf: dp, rms_dp_over_pinf: (s2 / gpu.len() as f64).sqrt(), max_rel_rho: dr }
}
