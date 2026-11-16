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
use crate::engine::gpu::{record_timing, PointTiming};
use std::time::Instant;

/// Iterations between two synchronisations (the CN sampling interval).
pub const SYNC: usize = SAMPLE_EVERY;

/// GPU solver plus the host solver (grid and config) of one 3D case.
pub struct GpuRunner3 {
    pub gpu: GpuSolver3,
    pub host: Solver3,
    /// Kernel build + geometry upload, seconds.
    pub setup_seconds: f64,
    /// Parts of the set-up: host grid + solver, default context, `from_host`, seconds.
    pub build_seconds: f64,
    pub ctx_seconds: f64,
    pub upload_seconds: f64,
}

impl GpuRunner3 {
    /// Builds the host solver of `case` and uploads it to the default GPU context.
    pub fn new(case: &Case3, prec: Precision) -> Result<GpuRunner3, Error> {
        let t_build = Instant::now();
        let host = case.build_solver()?;
        let build_seconds = t_build.elapsed().as_secs_f64();
        let t0 = Instant::now();
        let ctx = luma::default_context()?;
        let ctx_seconds = t0.elapsed().as_secs_f64();
        if ctx.is_cpu() {
            return Err(Error::Gpu("no GPU: Luma opened a software (CPU) Vulkan device".into()));
        }
        let t1 = Instant::now();
        let gpu = GpuSolver3::from_host(&ctx, &host, prec)?;
        let upload_seconds = t1.elapsed().as_secs_f64();
        Ok(GpuRunner3 { gpu, host, setup_seconds: t0.elapsed().as_secs_f64(), build_seconds, ctx_seconds, upload_seconds })
    }

    /// Iterate to a stop rule: as [`crate::engine::run3::run3_on`] (`warm`: keep the field, set the free stream of `case`, restart
    /// the CFL ramp).
    pub fn run_on(&mut self, case: &Case3, warm: bool) -> Result<PointResult3, Error> {
        let t_run = Instant::now();
        let g = &self.host.grid;
        let mut tm = PointTiming { dim: 3, mach: case.mach, alpha_deg: case.alpha.to_degrees(), warm, cells: g.ni * g.nj * g.nk, ..Default::default() };
        if !warm {
            (tm.build_s, tm.ctx_s, tm.upload_s) = (self.build_seconds, self.ctx_seconds, self.upload_seconds);
        }
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
        tm.reset_s = t_run.elapsed().as_secs_f64();

        let x_base = case.body.x_base();
        let p_inf = self.host.cfg.freestream[4];
        let mut rule = StopRule3::new(case, warm);
        let mut done = 0usize;
        let mut stop: Option<usize> = None;
        while done < n.max_iter && stop.is_none() {
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
            let cn = if len == SYNC {
                let t = Instant::now();
                let u = self.gpu.state_f64()?;
                tm.downloads += 1;
                tm.download_s += t.elapsed().as_secs_f64();
                let t = Instant::now();
                let cn = forces_on(case, &self.host.grid, &u, p_inf, x_base).cn;
                tm.forces_s += t.elapsed().as_secs_f64();
                Some(cn)
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
        let t_final = Instant::now();
        let f = forces_on(case, &self.host.grid, &self.gpu.state_f64()?, p_inf, x_base);
        let result = rule.finish(case, f, stop.unwrap_or(done));
        tm.final_s = t_final.elapsed().as_secs_f64();
        tm.run_s = t_run.elapsed().as_secs_f64();
        tm.iters_stop = stop.unwrap_or(done);
        tm.iters_exec = done;
        tm.stop = format!("{:?}", result.stop_reason);
        record_timing(tm);
        Ok(result)
    }
}
