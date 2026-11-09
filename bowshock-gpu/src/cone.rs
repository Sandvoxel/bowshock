//! Cone case runners (CPU reference loop and GPU loop with identical convergence criteria),
//! post-processing and CPU/GPU comparison metrics.
//!
//! The convergence criteria are those of `bowshock_cfd::cone::run_cone` (density-residual drop of
//! `drop_orders` from its maximum after iteration 20, plateau test from iteration 8000, max
//! iterations, non-finite). The CPU loop here uses the public `Solver::step` so that the final
//! state is available for the field comparison; the criteria are in [`Monitor`].

use crate::{GpuSolver, Mode, Precision, Result};
use bowshock_cfd::cone::{build_solver, ConeCase};
use bowshock_cfd::gas::cons_to_prim;
use bowshock_cfd::grid::Grid;
use bowshock_cfd::taylor_maccoll::ConeSolution;
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

/// Surface and integral results of a converged (or stopped) state.
pub struct Post {
    /// Cone surface samples: (x/L, p/p_inf, Cp from extrapolated wall value, Cp of the first cell).
    pub surface: Vec<(f64, f64, f64, f64)>,
    pub cp_avg: f64,
    pub cp_cell_avg: f64,
    pub beta_num: f64,
    pub min_rho: f64,
    pub min_p: f64,
}

/// Same post-processing as `bowshock_cfd::cone` (copied: it is private there).
pub fn postprocess(case: &ConeCase, g: &Grid, fs: &[f64; 4], u: &[[f64; 4]]) -> Post {
    let (ni, nj) = (g.ni, g.nj);
    let pressure = |i: usize, j: usize| cons_to_prim(&u[j * ni + i])[3];
    let p_inf = fs[3];
    let q_inf = 0.5 * fs[0] * fs[1] * fs[1];
    let cp = |p: f64| (p - p_inf) / q_inf;
    let length = g.x[ni];

    let mut surface = Vec::new();
    for i in 0..ni {
        let xm = 0.5 * (g.node(i, 0).0 + g.node(i + 1, 0).0);
        if xm <= 0.0 {
            continue;
        }
        let rm = 0.5 * (g.node(i, 0).1 + g.node(i + 1, 0).1);
        let (p0, p1) = (pressure(i, 0), pressure(i, 1));
        let d0 = (g.xc[i] - xm).hypot(g.rc[i] - rm);
        let d1 = (g.xc[ni + i] - xm).hypot(g.rc[ni + i] - rm);
        let pw = p0 + (p0 - p1) * d0 / (d1 - d0);
        surface.push((xm / length, pw / p_inf, cp(pw), cp(p0)));
    }
    let (lo, hi) = case.window;
    let sel: Vec<_> = surface.iter().filter(|r| r.0 >= lo && r.0 <= hi).collect();
    let cp_avg = sel.iter().map(|r| r.2).sum::<f64>() / sel.len() as f64;
    let cp_cell_avg = sel.iter().map(|r| r.3).sum::<f64>() / sel.len() as f64;

    let mut pts = Vec::new();
    for i in 0..ni {
        let xr = g.xc[i] / length;
        if xr < lo || xr > hi {
            continue;
        }
        let gr: Vec<f64> = (0..nj - 1)
            .map(|j| {
                let (ca, cb) = (j * ni + i, (j + 1) * ni + i);
                let d = (g.xc[cb] - g.xc[ca]).hypot(g.rc[cb] - g.rc[ca]);
                (pressure(i, j) - pressure(i, j + 1)).abs() / d
            })
            .collect();
        let k = (0..gr.len()).max_by(|&a, &b| gr[a].total_cmp(&gr[b])).unwrap();
        let mut frac = k as f64;
        if k > 0 && k + 1 < gr.len() {
            let (a, b, c) = (gr[k - 1], gr[k], gr[k + 1]);
            let den = a - 2.0 * b + c;
            if den < 0.0 {
                frac += (0.5 * (a - c) / den).clamp(-0.5, 0.5);
            }
        }
        let pos = |arr: &[f64], f: f64| {
            let t = f.clamp(0.0, (nj - 2) as f64);
            let j0 = t.floor() as usize;
            let a = 0.5 * (arr[j0 * ni + i] + arr[(j0 + 1) * ni + i]);
            let j1 = (j0 + 1).min(nj - 2);
            let b = 0.5 * (arr[j1 * ni + i] + arr[(j1 + 1) * ni + i]);
            a + (t - j0 as f64) * (b - a)
        };
        pts.push((pos(&g.xc, frac), pos(&g.rc, frac)));
    }
    let n = pts.len() as f64;
    let (sx, sr) = (pts.iter().map(|p| p.0).sum::<f64>(), pts.iter().map(|p| p.1).sum::<f64>());
    let sxx = pts.iter().map(|p| p.0 * p.0).sum::<f64>();
    let sxr = pts.iter().map(|p| p.0 * p.1).sum::<f64>();
    let beta_num = ((n * sxr - sx * sr) / (n * sxx - sx * sx)).atan();

    let (mut min_rho, mut min_p) = (f64::INFINITY, f64::INFINITY);
    for q in u {
        let w = cons_to_prim(q);
        min_rho = min_rho.min(w[0]);
        min_p = min_p.min(w[3]);
    }
    Post { surface, cp_avg, cp_cell_avg, beta_num, min_rho, min_p }
}

pub struct RunResult {
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
    pub u: Vec<[f64; 4]>,
    pub post: Post,
}

impl RunResult {
    pub fn us_per_iter(&self) -> f64 {
        1e6 * self.loop_seconds / self.iters_exec as f64
    }
}

/// CPU f64 reference run (bowshock-cfd, rayon).
pub fn run_cpu(case: &ConeCase, tm: &ConeSolution) -> RunResult {
    let mut s = build_solver(case, tm.beta);
    let mut mon = Monitor::new(case.drop_orders);
    let mut reason = "max iterations";
    let t0 = Instant::now();
    for _ in 0..case.max_iter {
        let r = s.step();
        if let Some(why) = mon.push(r) {
            reason = why;
            break;
        }
    }
    let loop_seconds = t0.elapsed().as_secs_f64();
    let post = postprocess(case, &s.grid, &s.cfg.freestream, &s.u);
    let n = mon.residual.len();
    RunResult {
        drop_orders: mon.drop_orders(),
        iters_stop: n,
        iters_exec: n,
        residual: mon.residual,
        reason,
        loop_seconds,
        setup_seconds: 0.0,
        warmup_seconds: 0.0,
        u: s.u.clone(),
        post,
    }
}

/// GPU f32 run: kernels for `chunk` iterations are recorded back to back; the host synchronises
/// only to read the residual history after every chunk and tests the stopping criteria on it.
pub fn run_gpu(case: &ConeCase, tm: &ConeSolution, chunk: usize, mode: Mode, prec: Precision) -> Result<RunResult> {
    let host = build_solver(case, tm.beta);
    let t0 = Instant::now();
    let mut g = GpuSolver::new_with(&luma::default_context()?, &host.grid, &host.cfg, mode, prec)?;
    let setup_seconds = t0.elapsed().as_secs_f64();
    // warm-up: first launches (driver-side pipeline work), then back to the free stream
    let t0 = Instant::now();
    g.run(1)?;
    g.reset()?;
    g.context().sync()?;
    let warmup_seconds = t0.elapsed().as_secs_f64();

    let mut mon = Monitor::new(case.drop_orders);
    let mut all = Vec::new();
    let mut reason = "max iterations";
    let mut iters_stop = None;
    let t0 = Instant::now();
    'outer: while all.len() < case.max_iter {
        let n = chunk.min(case.max_iter - all.len());
        let hist = g.run(n)?;
        for r in hist {
            all.push(r);
            if iters_stop.is_none() {
                if let Some(why) = mon.push(r) {
                    reason = why;
                    iters_stop = Some(mon.residual.len());
                }
            }
        }
        if iters_stop.is_some() {
            break 'outer;
        }
    }
    let loop_seconds = t0.elapsed().as_secs_f64();
    let u = g.state_f64()?;
    let post = postprocess(case, &host.grid, &host.cfg.freestream, &u);
    Ok(RunResult {
        drop_orders: mon.drop_orders(),
        iters_stop: iters_stop.unwrap_or(all.len()),
        iters_exec: all.len(),
        residual: all,
        reason,
        loop_seconds,
        setup_seconds,
        warmup_seconds,
        u,
        post,
    })
}

/// GPU-vs-CPU differences of a pair of runs on the same case.
#[derive(Debug, Clone, Copy)]
pub struct Compare {
    /// max over all surface stations of |Cp_gpu - Cp_cpu| / |Cp_cpu|.
    pub max_rel_cp_all: f64,
    /// Same restricted to the averaging window (0.4 <= x/L <= 0.9).
    pub max_rel_cp_window: f64,
    /// (Cp_avg_gpu - Cp_avg_cpu) / Cp_avg_cpu.
    pub avg_cp_rel: f64,
    /// Max over cells of |p_gpu - p_cpu| / p_inf and |rho_gpu - rho_cpu| / rho_cpu (field difference).
    pub max_dp_over_pinf: f64,
    pub max_rel_rho: f64,
    /// RMS over cells of (p_gpu - p_cpu) / p_inf.
    pub rms_dp_over_pinf: f64,
}

pub fn compare(case: &ConeCase, fs: &[f64; 4], gpu: &RunResult, cpu: &RunResult) -> Compare {
    let (mut all, mut win) = (0.0f64, 0.0f64);
    for (a, b) in gpu.post.surface.iter().zip(&cpu.post.surface) {
        let d = (a.2 - b.2).abs() / b.2.abs();
        all = all.max(d);
        if b.0 >= case.window.0 && b.0 <= case.window.1 {
            win = win.max(d);
        }
    }
    let (mut dp, mut dr, mut s2) = (0.0f64, 0.0f64, 0.0f64);
    for (a, b) in gpu.u.iter().zip(&cpu.u) {
        let (wa, wb) = (cons_to_prim(a), cons_to_prim(b));
        let d = (wa[3] - wb[3]).abs() / fs[3];
        dp = dp.max(d);
        s2 += d * d;
        dr = dr.max((wa[0] - wb[0]).abs() / wb[0]);
    }
    Compare {
        max_rel_cp_all: all,
        max_rel_cp_window: win,
        avg_cp_rel: (gpu.post.cp_avg - cpu.post.cp_avg) / cpu.post.cp_avg,
        max_dp_over_pinf: dp,
        max_rel_rho: dr,
        rms_dp_over_pinf: (s2 / gpu.u.len() as f64).sqrt(),
    }
}

/// Roundoff floor of the f32 residual: uploads a (converged) CPU f64 state, runs `n` iterations
/// without limiter start-up phase and returns the density residuals. For a state that is converged
/// to 1e-6 the f64 residual is ~1e-6; the f32 evaluation of the very same field is dominated by
/// rounding noise.
pub fn residual_on_state(case: &ConeCase, tm: &ConeSolution, u: &[[f64; 4]], n: usize, mode: Mode, prec: Precision) -> Result<Vec<f64>> {
    let host = build_solver(case, tm.beta);
    let mut g = GpuSolver::new_with(&luma::default_context()?, &host.grid, &host.cfg, mode, prec)?;
    g.set_state(u)?;
    // limiter start-up phase (first-order iterations) is over for a converged state
    g.iter = case.first_order_iters;
    g.run(n)
}
