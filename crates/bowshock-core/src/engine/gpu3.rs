//! GPU runner of a [`Case3`]: the procedure and stop rule of [`crate::engine::run3::run3_on`] with the iterations executed by
//! `bowshock_cfd::gpu::GpuSolver3` (the facade runs `Precision::F32`; the runner takes any precision).
//!
//! The GPU records [`SYNC`] iterations back to back and reads their residuals. CN is sampled once per full chunk from the downloaded
//! state, so the stop rule sees the same samples as the CPU loop; the reported stop iteration is the CPU-equivalent one, the GPU may
//! execute up to `SYNC - 1` further iterations and the final forces are those of the state after them.

use crate::engine::case3::Case3;
use crate::engine::gpu::{record_timing, PointTiming};
use crate::engine::run3::{
    breakdown_from, drag_parts_from, forces_from, wall_faces, zero_incidence, DragParts, LiftBreakdown, LiftForces, PointResult3, StopRule3, Wall, WallForces,
    SAMPLE_EVERY,
};
use crate::error::Error;
use bowshock_cfd::gpu::{GpuSolver3, Precision, WallFace3};
use bowshock_cfd::grid3::Face3;
use bowshock_cfd::solver3::{Config3, Scheme3, Solver3};
use std::time::Instant;

/// Iterations between two synchronisations (the CN sampling interval).
pub const SYNC: usize = SAMPLE_EVERY;

/// GPU solver plus the host solver (grid and config) of one 3D case.
pub struct GpuRunner3 {
    pub gpu: GpuSolver3,
    pub host: Solver3,
    /// The wall classes of the device force integration, in the order of `GpuSolver3::set_wall_faces`.
    classes: Vec<Wall>,
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
        let host = case.build_solver_cached()?;
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
        let mut runner = GpuRunner3 { gpu, host, classes: Vec::new(), setup_seconds: t0.elapsed().as_secs_f64(), build_seconds, ctx_seconds, upload_seconds };
        runner.upload_wall_faces(case)?;
        Ok(runner)
    }

    /// Uploads the wall faces of the force integration (`run3::wall_faces`, in the order of the CPU integration) to the device.
    fn upload_wall_faces(&mut self, case: &Case3) -> Result<(), Error> {
        let g = &self.host.grid;
        let x_max = case.body.x_base();
        let dist = |c: usize, f: &Face3| {
            let x = g.xc[c];
            ((x[0] - f.c[0]).powi(2) + (x[1] - f.c[1]).powi(2) + (x[2] - f.c[2]).powi(2)).sqrt()
        };
        let mut classes: Vec<Wall> = Vec::new();
        let faces: Vec<WallFace3> = wall_faces(g)
            .into_iter()
            .map(|w| {
                let class = classes.iter().position(|&c| c == w.class).unwrap_or_else(|| {
                    classes.push(w.class);
                    classes.len() - 1
                });
                let f = &w.face;
                let k = w.b.map_or(0.0, |b| {
                    let (d0, d1) = (dist(w.a, f), dist(b, f));
                    d0 / (d1 - d0)
                });
                let wt = f.area * w.sign;
                WallFace3 {
                    class,
                    a: w.a,
                    b: w.b,
                    k,
                    cx: wt * f.n[0],
                    cz: wt * f.n[2],
                    cm: wt * f.n[2] * f.c[0],
                    cut: f.c[0] <= x_max + 1e-9 || w.class == Wall::Base,
                }
            })
            .collect();
        self.gpu.set_wall_faces(&faces, classes.len())?;
        self.classes = classes;
        Ok(())
    }

    /// The wall forces per class reduced on the device, `(all faces, faces up to the base station)`; downloads a few numbers, not the state.
    pub fn wall_forces(&self) -> Result<(WallForces, WallForces), Error> {
        let sums = self.gpu.wall_forces(self.host.cfg.freestream[4])?;
        let n = self.classes.len();
        let by = |off: usize| WallForces { by: self.classes.iter().enumerate().map(|(i, &c)| (c, sums[off + i])).collect() };
        Ok((by(0), by(n)))
    }

    /// [`crate::engine::run3::lift_forces`] of the device state (the sting not counted, the wall faces up to the base station).
    pub fn lift_forces(&self, case: &Case3) -> Result<LiftForces, Error> {
        Ok(forces_from(case, &self.host.grid, &self.wall_forces()?.1))
    }

    /// [`crate::engine::run3::breakdown_on`] of the device state.
    pub fn breakdown(&self, case: &Case3) -> Result<LiftBreakdown, Error> {
        Ok(breakdown_from(case, &self.host.grid, &self.wall_forces()?.1))
    }

    /// [`crate::engine::run3::drag_parts`] of the device state (every wall face, the base included).
    pub fn drag_parts(&self, case: &Case3) -> Result<DragParts, Error> {
        Ok(drag_parts_from(case, &self.host.grid, &self.wall_forces()?.0))
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
            scheme: Scheme3::Implicit(n.implicit_for(true)),
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
                let f = self.lift_forces(case)?;
                tm.forces_s += t.elapsed().as_secs_f64();
                Some(if zero_incidence(&self.host.grid) || case.alpha.abs() < 1e-12 { f.ca } else { f.cn })
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
            // the state leaves the device only at a chunk where a snapshot is due
            if let Some(t) = case.snapshots.as_ref().filter(|t| stop.is_none() && len == SYNC && t.due(done)) {
                t.send(case, &self.host.grid, &self.gpu.state_f64()?, done);
            }
            if stop.is_none() && len == SYNC && rule.report(case, done).is_break() {
                rule.cancel();
                stop = Some(done);
            }
        }
        let t_final = Instant::now();
        let f = self.lift_forces(case)?;
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
