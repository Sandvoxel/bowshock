//! GPU runner of a [`Case3`]: the procedure and stop rule of [`crate::engine::run3::run3_on`] with the iterations executed by
//! `bowshock_cfd::gpu::GpuSolver3` (the facade runs `Precision::F32`; the runner takes any precision).
//!
//! The GPU records [`SYNC`] iterations back to back and reads their residuals. CN is sampled once per full chunk from the downloaded
//! state, so the stop rule sees the same samples as the CPU loop; the reported stop iteration is the CPU-equivalent one, the GPU may
//! execute up to `SYNC - 1` further iterations and the final forces are those of the state after them.

use crate::engine::case3::Case3;
use crate::engine::gpu::{record_timing, PointTiming};
use crate::engine::run3::{
    breakdown_from, drag_parts_from, forces_from, wall_class, wall_faces, zero_incidence, DragParts, LiftBreakdown, LiftForces, PointResult3, StopRule3, Wall,
    WallForces, MEAN_EVERY, SAMPLE_EVERY,
};
use crate::error::Error;
use bowshock_cfd::gpu::{GpuSolver3, Precision, TractionFace3, WallFace3};
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
    /// The viscous parameters of a RANS run (`None`: Euler).
    visc: Option<bowshock_cfd::solver::Viscous>,
    /// Kernel build + geometry upload, seconds.
    pub setup_seconds: f64,
    /// Parts of the set-up: host grid + solver, default context, `from_host`, seconds.
    pub build_seconds: f64,
    pub ctx_seconds: f64,
    pub upload_seconds: f64,
}

impl GpuRunner3 {
    /// Builds the host solver of `case` (viscous with SST when `Case3::viscous` is set: the wall forces of the runner then include the shear of the no-slip faces) and
    /// uploads it to the default GPU context.
    pub fn new(case: &Case3, prec: Precision) -> Result<GpuRunner3, Error> {
        let t_build = Instant::now();
        // the grid (mesher, face geometry) comes from the process-wide cache: one `Analysis::run` per Mach node rebuilds neither it nor, with the shared default
        // context, the compiled kernels of the previous call
        let host = case.build_solver_cached()?;
        let visc = host.rans().map(|r| r.visc);
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
        let mut runner =
            GpuRunner3 { gpu, host, classes: Vec::new(), visc, setup_seconds: t0.elapsed().as_secs_f64(), build_seconds, ctx_seconds, upload_seconds };
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
        // the no-slip faces of a viscous run: their friction joins the classes (a class that has shear faces only is added)
        let mut pending: Vec<(usize, TractionFace3, bool)> = Vec::new();
        if let Some(r) = self.host.rans() {
            let solid: Vec<bool> = g.part.iter().map(|&p| p != bowshock_cfd::grid3::PART_FLUID).collect();
            let zero = |n: usize| vec![[0.0; 5]; n];
            for t in bowshock_cfd::viscous3::wall_tractions(g, &solid, &r.dirs, &zero(g.iface.len()), &zero(g.jface.len()), &zero(g.kface.len())) {
                let [n0, n1, _] = bowshock_cfd::viscous3::face_shape(g, t.dir);
                let q = (t.idx[2] * n1 + t.idx[1]) * n0 + t.idx[0];
                let face = match t.dir {
                    0 => &g.iface[q],
                    1 => &g.jface[q],
                    _ => &g.kface[q],
                };
                let sigma = if t.n[0] * face.n[0] + t.n[1] * face.n[1] + t.n[2] * face.n[2] > 0.0 { 1.0 } else { -1.0 };
                // the solid side: the upper cell where the fluid cell is the lower one (`sigma = +1`), else the lower one; a `j = 0` boundary face of `Grid3::wall_j0` has
                // none (its part is the run's, the class that of `wall_faces`)
                let class = if t.dir == 1 && t.idx[1] == 0 {
                    let Some((_, part)) = g.wall_j0.iter().find(|(r, _)| r.contains(&t.idx[0])) else { continue };
                    wall_class(*part, 1, false)
                } else {
                    let mut sc = t.idx;
                    if sigma < 0.0 {
                        sc[t.dir] -= 1;
                    }
                    let Some(part) = g.part_of(sc[0], sc[1], sc[2]) else { continue };
                    wall_class(part, t.dir, sigma < 0.0)
                };
                let ci = classes.iter().position(|&c| c == class).unwrap_or_else(|| {
                    classes.push(class);
                    classes.len() - 1
                });
                pending.push((
                    ci,
                    TractionFace3 { class: ci, dir: t.dir, q, sigma, area: face.area, cx: face.c[0] },
                    face.c[0] <= x_max + 1e-9 || class == Wall::Base,
                ));
            }
        }
        self.gpu.set_wall_faces(&faces, classes.len())?;
        if self.host.rans().is_some() {
            // the shear sums `[all faces (classes 0..n), faces up to the base station (n..2n)]` like the pressure: the faces of the second set are listed again
            let n = classes.len();
            let tfaces: Vec<TractionFace3> = pending
                .iter()
                .map(|(_, f, _)| *f)
                .chain(pending.iter().filter(|(_, _, cut)| *cut).map(|(ci, f, _)| TractionFace3 { class: n + ci, ..*f }))
                .collect();
            self.gpu.set_traction_faces(&tfaces, 2 * n)?;
        }
        self.classes = classes;
        Ok(())
    }

    /// The wall forces per class reduced on the device, `(all faces, faces up to the base station)`; downloads a few numbers, not the state.
    pub fn wall_forces(&self) -> Result<(WallForces, WallForces), Error> {
        let sums = self.gpu.wall_forces(self.host.cfg.freestream[4])?;
        let n = self.classes.len();
        // a viscous run: the shear of the no-slip faces, reduced on the device from the viscous face fluxes (`[all faces, faces up to the base station]` like the pressure),
        // goes to `WallForces::shear`, as the CPU's `run3::wall_shear` (the base wall's shear included)
        let shear = if self.gpu.has_viscous() { Some(self.gpu.viscous_wall_forces()?) } else { None };
        let by = |off: usize| WallForces {
            by: self.classes.iter().enumerate().map(|(i, &c)| (c, sums[off + i])).collect(),
            shear: shear.as_ref().map_or_else(Vec::new, |f| self.classes.iter().enumerate().map(|(i, &c)| (c, f[off + i])).collect()),
        };
        Ok((by(0), by(n)))
    }

    /// The largest first-cell `y+` of the streamwise no-slip walls of the device state (`run3::y_plus_max` from the downloaded state and viscous face fluxes; `None` for Euler).
    pub fn y_plus_report(&self) -> Result<Option<f64>, Error> {
        let (Some(visc), Some(fluxes)) = (self.visc, self.gpu.viscous_flux()?) else { return Ok(None) };
        let g = &self.host.grid;
        let r = self.host.rans().ok_or_else(|| Error::Gpu("the host solver has no viscous state".into()))?;
        let u = self.gpu.state_f64()?;
        let solid: Vec<bool> = g.part.iter().map(|&p| p != bowshock_cfd::grid3::PART_FLUID).collect();
        let mut y_plus_max = 0.0f64;
        for t in bowshock_cfd::viscous3::wall_tractions(g, &solid, &r.dirs, &fluxes[0], &fluxes[1], &fluxes[2]) {
            if t.dir == 0 {
                continue;
            }
            let [i, j, k] = t.idx;
            let f = match t.dir {
                1 => g.jface[(k * (g.nj + 1) + j) * g.ni + i],
                _ => g.kface[(k * g.nj + j) * g.ni + i],
            };
            // the fluid cell of the face: the cell at the face's own index, or the one below it
            let own = g.cell(i.min(g.ni - 1), j.min(g.nj - 1), k.min(g.nk - 1));
            let cell = if g.part[own] == bowshock_cfd::grid3::PART_FLUID {
                own
            } else if t.dir == 1 {
                g.cell(i, j - 1, k)
            } else {
                g.cell(i, j, k - 1)
            };
            let x = g.xc[cell];
            let d_n = ((x[0] - f.c[0]) * t.n[0] + (x[1] - f.c[1]) * t.n[1] + (x[2] - f.c[2]) * t.n[2]).abs();
            let tau = (t.tau[0] * t.tau[0] + t.tau[1] * t.tau[1] + t.tau[2] * t.tau[2]).sqrt();
            let w = bowshock_cfd::gas3::cons_to_prim3(&u[cell]);
            let mu = visc.mu(bowshock_cfd::gas3::GAMMA * w[4] / w[0]);
            y_plus_max = y_plus_max.max(d_n * (w[0] * tau).sqrt() / mu);
        }
        Ok(Some(y_plus_max))
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
            // the line solve: `Block` unless the case asks for the diagonal one (`numerics.line_solve = Some(DiagonalCorrected)`, Euler only: it is not ported to the viscous
            // step). The diagonal default of `Numerics3::implicit_for(true)` diverges where the block solve does not (the finned half domain at M 3 / 6 degrees: pmin 0.018 at
            // the base plane by iteration 30-100, non-finite at 110 at `cfl_max` 10 and 150 at 5, stable at 3; the wedge: ISSUES #24), so the runner never takes it by default
            scheme: Scheme3::Implicit({
                let mut p = n.implicit_for(true);
                if n.line_solve.is_none() || self.host.rans().is_some() {
                    p.line_solve = bowshock_cfd::implicit::LineSolve3::Block;
                }
                p
            }),
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
            // the mean-stationary rule needs the forces every `MEAN_EVERY` iterations once it is past `mean_from` (a 50-iteration chunk would alias the wake periods): chunks
            // of 10 from there on, 50 before (`SYNC`, the sampling interval of the range rule)
            let len = if rule.wants_forces(done + MEAN_EVERY, MEAN_EVERY) { MEAN_EVERY } else { SYNC }.min(n.max_iter - done);
            let end = done + len;
            let t_chunk = Instant::now();
            let h = self.gpu.run(len)?;
            let dt_chunk = t_chunk.elapsed().as_secs_f64();
            if done == 0 {
                tm.first_chunk_s = dt_chunk;
            } else {
                tm.chunks_rest += 1;
                tm.rest_chunks_s += dt_chunk;
            }
            // the force coefficients of the state after the chunk: for the mean rule (`observe` before the `check`s of the chunk) and for the CN sample of the range rule
            let sample_due = end.is_multiple_of(SYNC);
            let wants = rule.wants_forces(end, MEAN_EVERY);
            let cn = if wants || sample_due {
                let t = Instant::now();
                let f = self.lift_forces(case)?;
                tm.forces_s += t.elapsed().as_secs_f64();
                if wants {
                    rule.observe(end, &f);
                }
                sample_due.then(|| if zero_incidence(&self.host.grid) || case.alpha.abs() < 1e-12 { f.ca } else { f.cn })
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
            done = end;
            // the state leaves the device only at a chunk where a snapshot is due
            let on_grid = done.is_multiple_of(SYNC);
            if let Some(t) = case.snapshots.as_ref().filter(|t| stop.is_none() && on_grid && t.due(done)) {
                t.send(case, &self.host.grid, &self.gpu.state_f64()?, done);
            }
            if stop.is_none() && on_grid && rule.report(case, done).is_break() {
                rule.cancel();
                stop = Some(done);
            }
        }
        let t_final = Instant::now();
        let f = self.lift_forces(case)?;
        let mut result = rule.finish(case, f, stop.unwrap_or(done));
        // the `y+` of the streamwise walls (viscous runs) and the base Cp, from the device state (one state download; the host solver takes it for the pressure integration)
        result.y_plus_max = self.y_plus_report()?.unwrap_or(f64::NAN);
        self.host.u = self.gpu.state_f64()?;
        result.cp_base = crate::engine::run3::base_cp(case, &self.host);
        tm.final_s = t_final.elapsed().as_secs_f64();
        tm.run_s = t_run.elapsed().as_secs_f64();
        tm.iters_stop = stop.unwrap_or(done);
        tm.iters_exec = done;
        tm.stop = format!("{:?}", result.stop_reason);
        record_timing(tm);
        Ok(result)
    }
}
