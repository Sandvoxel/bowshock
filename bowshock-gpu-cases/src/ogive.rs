//! Tangent-ogive + cylinder case on the GPU: runners for the CPU reference and the GPU solver (same grid, same
//! `Config` from `bowshock_cases::ogive::OgiveCase`, same stopping criteria), the wall post-processing of
//! `bowshock_cases::ogive` (copied: it is private there) and CPU/GPU comparison metrics.

use crate::run::{cpu_loop, field_diff, gpu_loop, Run};
use bowshock::result::WallPoint;
use bowshock_cases::ogive::OgiveCase;
use bowshock_core::gas::cons_to_prim;
use bowshock_core::grid::Grid;
use bowshock_gpu::{Mode, Precision, Result};

/// Wall and integral results of a converged (or stopped) ogive state.
pub struct Post {
    /// Wall faces with `x > 0` (`Cp` from the linearly extrapolated wall pressure and from the first cell).
    pub wall: Vec<WallPoint>,
    /// Forebody (nose) pressure-drag coefficient, base-area reference, from extrapolated / first-cell Cp.
    pub cd_nose: f64,
    pub cd_nose_cell: f64,
    pub min_rho: f64,
    pub min_p: f64,
}

/// Same post-processing as `bowshock_cases::ogive` (wall `Cp`, forebody `CD`, positivity), evaluated on a conserved field `u`.
pub fn postprocess(case: &OgiveCase, g: &Grid, fs: &[f64; 4], u: &[[f64; 4]]) -> Post {
    let (ni, _nj) = (g.ni, g.nj);
    let pressure = |i: usize, j: usize| cons_to_prim(&u[j * ni + i])[3];
    let (p_inf, q_inf) = (fs[3], 0.5 * fs[0] * fs[1] * fs[1]);
    let cp = |p: f64| (p - p_inf) / q_inf;
    let (ln, rb2) = (case.body.nose_length(), case.body.radius().powi(2));

    let mut wall = Vec::new();
    let (mut cd, mut cd_cell) = (0.0, 0.0);
    for i in 0..ni {
        let (n0, n1) = (g.node(i, 0), g.node(i + 1, 0));
        let (xm, rm) = (0.5 * (n0.0 + n1.0), 0.5 * (n0.1 + n1.1));
        if xm <= 0.0 {
            continue;
        }
        let (p0, p1) = (pressure(i, 0), pressure(i, 1));
        let d0 = (g.xc[i] - xm).hypot(g.rc[i] - rm);
        let d1 = (g.xc[ni + i] - xm).hypot(g.rc[ni + i] - rm);
        let pw = p0 + (p0 - p1) * d0 / (d1 - d0);
        let (s0, s1) = (n0.1 * n0.1 / rb2, n1.1 * n1.1 / rb2);
        if n1.0 <= ln + 1e-12 {
            cd += cp(pw) * (s1 - s0);
            cd_cell += cp(p0) * (s1 - s0);
        }
        wall.push(WallPoint { x: xm, r: rm, s0, s1, cp: cp(pw), cp_cell: cp(p0) });
    }
    let (mut min_rho, mut min_p) = (f64::INFINITY, f64::INFINITY);
    for q in u {
        let w = cons_to_prim(q);
        min_rho = min_rho.min(w[0]);
        min_p = min_p.min(w[3]);
    }
    Post { wall, cd_nose: cd, cd_nose_cell: cd_cell, min_rho, min_p }
}

/// One iteration loop (CPU or GPU) plus the wall post-processing of its final state.
pub struct OgiveRun {
    pub run: Run,
    pub post: Post,
}

/// CPU f64 reference run (bowshock-core, rayon) with the stopping criteria of `bowshock_cases::ogive::run_ogive`.
pub fn run_cpu(case: &OgiveCase) -> OgiveRun {
    let mut s = case.build_solver();
    let run = cpu_loop(&mut s, case.max_iter, case.drop_orders);
    let post = postprocess(case, &s.grid, &s.cfg.freestream, &run.u);
    OgiveRun { run, post }
}

/// GPU run on the grid/config of `case.build_solver()`; kernels for `chunk` iterations are recorded back to back and
/// the stopping criteria are tested on the residual history read back after each chunk.
pub fn run_gpu(case: &OgiveCase, chunk: usize, mode: Mode, prec: Precision) -> Result<OgiveRun> {
    let host = case.build_solver();
    let run = gpu_loop(&host, case.max_iter, case.drop_orders, chunk, mode, prec)?;
    let post = postprocess(case, &host.grid, &host.cfg.freestream, &run.u);
    Ok(OgiveRun { run, post })
}

/// Wall faces with `|Cp| <` this are left out of the relative Cp comparison.
pub const REL_CP_FLOOR: f64 = 0.05;

/// GPU-vs-CPU differences of a pair of runs on the same case.
#[derive(Debug, Clone, Copy)]
pub struct Compare {
    /// `(CD_gpu - CD_cpu) / CD_cpu` of the forebody drag (extrapolated-wall Cp) and of its first-cell variant.
    pub cd_rel: f64,
    pub cd_cell_rel: f64,
    /// `(mean Cp_gpu - mean Cp_cpu) / mean Cp_cpu` over the nose wall faces (`x <= L_nose`, plain mean over faces).
    pub avg_cp_nose_rel: f64,
    /// max over the nose faces with `|Cp_cpu| >= REL_CP_FLOOR` of `|Cp_gpu - Cp_cpu| / |Cp_cpu|` (Cp passes through zero on the
    /// nose near `x/d = 2.3`, where a relative difference is meaningless).
    pub max_rel_cp_nose: f64,
    /// max over all wall faces (nose and cylinder) of `|Cp_gpu - Cp_cpu|` (absolute: Cp crosses zero on the cylinder).
    pub max_abs_dcp_wall: f64,
    pub max_dp_over_pinf: f64,
    pub rms_dp_over_pinf: f64,
    pub max_rel_rho: f64,
}

pub fn compare(case: &OgiveCase, fs: &[f64; 4], gpu: &OgiveRun, cpu: &OgiveRun) -> Compare {
    let ln = case.body.nose_length();
    let (mut sg, mut sc) = (0.0, 0.0);
    let (mut max_rel, mut max_abs) = (0.0f64, 0.0f64);
    for (a, b) in gpu.post.wall.iter().zip(&cpu.post.wall) {
        max_abs = max_abs.max((a.cp - b.cp).abs());
        if b.x <= ln {
            sg += a.cp;
            sc += b.cp;
            if b.cp.abs() >= REL_CP_FLOOR {
                max_rel = max_rel.max((a.cp - b.cp).abs() / b.cp.abs());
            }
        }
    }
    let f = field_diff(&gpu.run.u, &cpu.run.u, fs[3]);
    Compare {
        cd_rel: (gpu.post.cd_nose - cpu.post.cd_nose) / cpu.post.cd_nose,
        cd_cell_rel: (gpu.post.cd_nose_cell - cpu.post.cd_nose_cell) / cpu.post.cd_nose_cell,
        avg_cp_nose_rel: (sg - sc) / sc,
        max_rel_cp_nose: max_rel,
        max_abs_dcp_wall: max_abs,
        max_dp_over_pinf: f.max_dp_over_pinf,
        rms_dp_over_pinf: f.rms_dp_over_pinf,
        max_rel_rho: f.max_rel_rho,
    }
}
