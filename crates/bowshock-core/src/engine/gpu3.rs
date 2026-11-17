//! GPU runner of a [`Case3`]: the procedure and stop rule of [`crate::engine::run3::run3_on`] with the iterations executed by
//! `bowshock_cfd::gpu::GpuSolver3` (extended precision).
//!
//! The GPU records [`SYNC`] iterations back to back and reads their residuals. CN is sampled once per full chunk from the downloaded
//! state, so the stop rule sees the same samples as the CPU loop; the reported stop iteration is the CPU-equivalent one, the GPU may
//! execute up to `SYNC - 1` further iterations and the final forces are those of the state after them.

use crate::engine::case3::Case3;
use crate::engine::run3::{forces_on, PointResult3, StopRule3, SAMPLE_EVERY};
use crate::error::Error;
use bowshock_cfd::gpu::{GpuSolver3, Precision};
use bowshock_cfd::solver3::{Config3, Scheme3, Solver3};

/// Iterations between two synchronisations (the CN sampling interval).
pub const SYNC: usize = SAMPLE_EVERY;

/// GPU solver plus the host solver (grid and config) of one 3D case.
pub struct GpuRunner3 {
    pub gpu: GpuSolver3,
    pub host: Solver3,
    /// Kernel build + geometry upload, seconds.
    pub setup_seconds: f64,
}

impl GpuRunner3 {
    /// Builds the host solver of `case` and uploads it to the default GPU context.
    pub fn new(case: &Case3, prec: Precision) -> Result<GpuRunner3, Error> {
        let host = case.build_solver()?;
        let t0 = std::time::Instant::now();
        let ctx = luma::default_context()?;
        if ctx.is_cpu() {
            return Err(Error::Gpu("no GPU: Luma opened a software (CPU) Vulkan device".into()));
        }
        let gpu = GpuSolver3::from_host(&ctx, &host, prec)?;
        Ok(GpuRunner3 { gpu, host, setup_seconds: t0.elapsed().as_secs_f64() })
    }

    /// Iterate to a stop rule: as [`crate::engine::run3::run3_on`] (`warm`: keep the field, set the free stream of `case`, restart
    /// the CFL ramp).
    pub fn run_on(&mut self, case: &Case3, warm: bool) -> Result<PointResult3, Error> {
        let n = &case.numerics;
        let cfg = Config3 {
            freestream: case.freestream(),
            cfl: self.host.cfg.cfl,
            limiter: n.limiter,
            first_order_iters: n.first_order_iters,
            low_mach: n.low_mach,
            scheme: Scheme3::Implicit(n.implicit),
        };
        self.gpu.set_config(&cfg)?;
        self.host.cfg = cfg;
        if warm {
            let it = self.gpu.iter().max(n.first_order_iters);
            self.gpu.set_iter(it);
            self.gpu.restart_ramp()?;
        } else {
            self.gpu.reset()?;
        }
        self.gpu.context().sync()?;

        let x_base = case.body.x_base();
        let p_inf = self.host.cfg.freestream[4];
        let mut rule = StopRule3::new(case, warm);
        let mut done = 0usize;
        let mut stop: Option<usize> = None;
        while done < n.max_iter && stop.is_none() {
            let len = SYNC.min(n.max_iter - done);
            let h = self.gpu.run(len)?;
            let cn = if len == SYNC {
                self.gpu.read_boundary_into(&self.host.grid, &mut self.host.u)?;
                Some(forces_on(case, &self.host.grid, &self.host.u, p_inf, x_base).cn)
            } else {
                None
            };
            for (k, r) in h.into_iter().enumerate() {
                let it = done + k;
                let sample = if it % SYNC == SYNC - 1 { cn } else { None };
                if rule.check(it, r, sample).is_some() {
                    stop = Some(it + 1);
                    break;
                }
            }
            done += len;
            if stop.is_none() && len == SYNC && rule.report(case, done).is_break() {
                rule.cancel();
                stop = Some(done);
            }
        }
        self.gpu.read_boundary_into(&self.host.grid, &mut self.host.u)?;
        let f = forces_on(case, &self.host.grid, &self.host.u, p_inf, x_base);
        Ok(rule.finish(case, f, stop.unwrap_or(done)))
    }
}
