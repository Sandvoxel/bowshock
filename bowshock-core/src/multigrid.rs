//! FAS (full approximation storage) multigrid for the steady Euler iteration, built from the same per-cell / per-face
//! kernels as the explicit solver, so that every operation is all-cells-parallel (no sequential sweeps).
//!
//! Hierarchy: the grid is coarsened by removing every second node line (`Grid::coarsen`, 2x2 cells -> 1 cell, requires
//! even `ni`, `nj`); coarse levels use first-order reconstruction (cheap, robust), the same BCs, the free stream and,
//! with a blocked body region, the halved block.
//!
//! One cycle on level `l` (rates `R = dU/dt` as in `Solver::eval_residual`, the coarse problem is `dU/dt = R_c(U) + P`):
//!
//! 1. `pre` SSP-RK3 smoothing steps (local time stepping, optional implicit residual smoothing);
//! 2. defect `d = R_l(U_l) (+ P_l)`; restriction of the state `U_c = sum V U / sum V` and of the defect
//!    `P_c = sum V d / sum V - R_c(U_c)` (volume weighted, fluid cells only);
//! 3. `gamma` cycles on level `l+1` (the coarsest level takes `coarsest` smoothing steps);
//! 4. prolongation of the correction `U_l += I(U_c - U_c^0)` (index-space bilinear, solid coarse cells excluded, damped by
//!    halving until density and pressure are positive);
//! 5. `post` smoothing steps.
//!
//! The converged state of a cycle is the converged state of the fine-grid (second-order) discretisation: the coarse
//! correction vanishes with the fine-grid defect.

use crate::gas::{cons_to_prim, Cons};
use crate::flux::Limiter;
use crate::solver::{Config, MgParams, Scheme, Solver};
use rayon::prelude::*;

impl Solver {
    /// Select the convergence scheme. For `Scheme::Multigrid` the coarse levels are built from the current grid and block
    /// (call [`Solver::set_block`] first); returns the number of levels (1 for the other schemes).
    pub fn set_scheme(&mut self, scheme: Scheme) -> usize {
        self.scheme = scheme;
        self.coarse.clear();
        let Scheme::Multigrid(p) = scheme else {
            return 1;
        };
        let mut levels: Vec<Solver> = Vec::new();
        while levels.len() + 1 < p.levels {
            let fine: &Solver = levels.last().unwrap_or(self);
            let Some(grid) = fine.grid.coarsen() else { break };
            if grid.ni < 8 || grid.nj < 8 {
                break;
            }
            let block = match fine.block {
                Some(b) => match b.coarsen() {
                    Some(c) => Some(c),
                    None => break,
                },
                None => None,
            };
            let cfg = Config { cfl: self.cfg.cfl, limiter: Limiter::FirstOrder, freestream: self.cfg.freestream, bc: self.cfg.bc, first_order_iters: 0 };
            let n = grid.ni * grid.nj;
            let mut s = Solver::new(grid, cfg);
            s.low_mach = self.low_mach;
            s.set_block(block);
            s.forcing = vec![[0.0; 4]; n];
            s.mg_u0 = vec![[0.0; 4]; n];
            s.irs = p.irs;
            levels.push(s);
        }
        let n = levels.len() + 1;
        self.coarse = levels;
        n
    }

    /// Number of multigrid levels (1 without multigrid).
    pub fn mg_levels(&self) -> usize {
        self.coarse.len() + 1
    }
}

/// One multigrid cycle starting on `fine`; returns the density residual at the start of the cycle.
pub(crate) fn cycle(fine: &mut Solver, p: &MgParams) -> f64 {
    let mut coarse = std::mem::take(&mut fine.coarse);
    for c in coarse.iter_mut() {
        c.cfg.freestream = fine.cfg.freestream;
        c.cfg.cfl = fine.cfg.cfl;
        c.low_mach = fine.low_mach;
        c.irs = p.irs;
    }
    fine.irs = p.irs;
    let rn = level(fine, &mut coarse, p);
    fine.coarse = coarse;
    rn
}

fn level(fine: &mut Solver, coarse: &mut [Solver], p: &MgParams) -> f64 {
    let n_pre = if coarse.is_empty() { p.coarsest } else { p.pre };
    let mut rn = f64::NAN;
    for k in 0..n_pre {
        let r = fine.rk3_step();
        if k == 0 {
            rn = r;
        }
    }
    let Some((next, rest)) = coarse.split_first_mut() else {
        return rn;
    };
    fine.eval_residual(false);
    if rn.is_nan() {
        rn = fine.density_residual();
    }
    restrict(fine, next);
    for _ in 0..p.gamma.max(1) {
        level(next, rest, p);
    }
    prolong_correct(next, fine);
    for _ in 0..p.post {
        fine.rk3_step();
    }
    rn
}

/// Restrict state and defect of `fine` (whose `res` holds the defect of its current state) to `coarse` and set the forcing.
fn restrict(fine: &Solver, coarse: &mut Solver) {
    let (fni, cni, cnj) = (fine.grid.ni, coarse.grid.ni, coarse.grid.nj);
    let blk = fine.block;
    let child = |ci: usize, cj: usize| -> [usize; 4] {
        let c0 = 2 * cj * fni + 2 * ci;
        [c0, c0 + 1, c0 + fni, c0 + fni + 1]
    };
    let solid = |c: usize| blk.map_or(false, |b| b.solid(c % fni, c / fni));
    // state
    {
        let (fu, vol) = (&fine.u, &fine.grid.vol);
        coarse.u.par_chunks_mut(cni).enumerate().for_each(|(cj, row)| {
            for ci in 0..cni {
                let (mut s, mut v) = ([0.0; 4], 0.0);
                for c in child(ci, cj) {
                    if solid(c) {
                        continue;
                    }
                    for k in 0..4 {
                        s[k] += vol[c] * fu[c][k];
                    }
                    v += vol[c];
                }
                if v > 0.0 {
                    for k in 0..4 {
                        row[ci][k] = s[k] / v;
                    }
                }
            }
        });
    }
    coarse.mg_u0.copy_from_slice(&coarse.u);
    for f in coarse.forcing.iter_mut() {
        *f = [0.0; 4];
    }
    coarse.eval_residual(false);
    let (fr, vol) = (&fine.res, &fine.grid.vol);
    let cres = &coarse.res;
    coarse.forcing.par_chunks_mut(cni).enumerate().for_each(|(cj, row)| {
        for ci in 0..cni {
            let (mut s, mut v) = ([0.0; 4], 0.0);
            for c in child(ci, cj) {
                if solid(c) {
                    continue;
                }
                for k in 0..4 {
                    s[k] += vol[c] * fr[c][k];
                }
                v += vol[c];
            }
            if v > 0.0 {
                for k in 0..4 {
                    row[ci][k] = s[k] / v - cres[cj * cni + ci][k];
                }
            }
        }
    });
    let _ = cnj;
}

/// `fine.u += I (coarse.u - coarse.mg_u0)`, index-space bilinear (cell-centred weights 3/4, 1/4 per direction) over the
/// non-solid parents, with a positivity safeguard.
fn prolong_correct(coarse: &Solver, fine: &mut Solver) {
    let (fni, cni, cnj) = (fine.grid.ni, coarse.grid.ni, coarse.grid.nj);
    let blk = fine.block;
    let cblk = coarse.block;
    let (cu, c0) = (&coarse.u, &coarse.mg_u0);
    fine.u.par_chunks_mut(fni).enumerate().for_each(|(j, row)| {
        for i in 0..fni {
            if blk.map_or(false, |b| b.solid(i, j)) {
                continue;
            }
            let (ci, cj) = (i / 2, j / 2);
            let ci2 = if i % 2 == 0 { ci.saturating_sub(1) } else { (ci + 1).min(cni - 1) };
            let cj2 = if j % 2 == 0 { cj.saturating_sub(1) } else { (cj + 1).min(cnj - 1) };
            let cand = [(ci, cj, 9.0), (ci2, cj, 3.0), (ci, cj2, 3.0), (ci2, cj2, 1.0)];
            let (mut d, mut wsum) = ([0.0; 4], 0.0);
            for (a, b, wt) in cand {
                if cblk.map_or(false, |bk| bk.solid(a, b)) {
                    continue;
                }
                let c = b * cni + a;
                for k in 0..4 {
                    d[k] += wt * (cu[c][k] - c0[c][k]);
                }
                wsum += wt;
            }
            for k in 0..4 {
                d[k] /= wsum;
            }
            // damp the correction until density and pressure stay positive
            let mut alpha = 1.0;
            for _ in 0..6 {
                let q: Cons = [row[i][0] + alpha * d[0], row[i][1] + alpha * d[1], row[i][2] + alpha * d[2], row[i][3] + alpha * d[3]];
                let w = cons_to_prim(&q);
                if w[0] > 0.0 && w[3] > 0.0 {
                    row[i] = q;
                    break;
                }
                alpha *= 0.5;
            }
        }
    });
}

#[cfg(test)]
mod tests {
    use crate::flux::Limiter;
    use crate::gas::{freestream, prim_to_cons};
    use crate::grid::Grid;
    use crate::solver::{Bc, Config, MgParams, Scheme, Solver};

    fn grid(ni: usize, nj: usize) -> Grid {
        let (mut x, mut r) = (vec![0.0; (ni + 1) * (nj + 1)], vec![0.0; (ni + 1) * (nj + 1)]);
        for j in 0..=nj {
            for i in 0..=ni {
                let (s, t) = (i as f64 / ni as f64, j as f64 / nj as f64);
                x[j * (ni + 1) + i] = 4.0 * s + 0.1 * (3.0 * t).sin();
                r[j * (ni + 1) + i] = 3.0 * t * (1.0 + 0.2 * s);
            }
        }
        Grid::from_nodes(ni, nj, x, r)
    }

    #[test]
    fn hierarchy_is_built_and_the_free_stream_is_preserved() {
        let fs = freestream(0.5);
        let cfg = Config { cfl: 0.5, limiter: Limiter::VanAlbada, freestream: fs, bc: [Bc::Characteristic, Bc::Characteristic, Bc::SlipWall, Bc::Characteristic], first_order_iters: 0 };
        let mut s = Solver::new(grid(64, 32), cfg);
        let n = s.set_scheme(Scheme::Multigrid(MgParams::DEFAULT));
        assert_eq!(n, 3, "64x32 -> 32x16 -> 16x8, the next level would have nj < 8");
        for _ in 0..30 {
            s.step();
        }
        let q0 = prim_to_cons(&fs);
        let worst = s.u.iter().flat_map(|q| (0..4).map(move |k| (q[k] - q0[k]).abs())).fold(0.0, f64::max);
        assert!(worst < 1e-11, "free-stream drift {worst:e}");
    }

    #[test]
    fn multigrid_reaches_the_explicit_fixed_point() {
        // an outflow pulse on a small domain: both schemes must converge to the free stream
        let fs = freestream(0.4);
        let mk = |scheme: Scheme| {
            let cfg = Config { cfl: 0.8, limiter: Limiter::FirstOrder, freestream: fs, bc: [Bc::Characteristic, Bc::Characteristic, Bc::SlipWall, Bc::Characteristic], first_order_iters: 0 };
            let mut s = Solver::new(grid(32, 16), cfg);
            s.set_scheme(scheme);
            for u in s.u.iter_mut() {
                *u = prim_to_cons(&[fs[0] * 1.02, fs[1], fs[2], fs[3] * 1.03]);
            }
            let mut last = 1.0;
            for _ in 0..4000 {
                last = s.step();
                if last < 1e-11 {
                    break;
                }
            }
            (s, last)
        };
        let (a, ra) = mk(Scheme::Explicit);
        let (b, rb) = mk(Scheme::Multigrid(MgParams::DEFAULT));
        assert!(ra < 1e-9 && rb < 1e-9, "residuals {ra:e} {rb:e}");
        let worst = a.u.iter().zip(b.u.iter()).flat_map(|(p, q)| (0..4).map(move |k| (p[k] - q[k]).abs())).fold(0.0, f64::max);
        assert!(worst < 1e-7, "explicit vs multigrid state difference {worst:e}");
    }
}
