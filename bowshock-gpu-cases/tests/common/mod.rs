//! Set-ups shared by the GPU-vs-CPU test files.
#![allow(dead_code)]

use bowshock_core::flux::Limiter;
use bowshock_core::gas::{freestream, prim_to_cons};
use bowshock_core::grid::Grid;
use bowshock_core::solver::{Bc, Config, Solver};

/// Rectangular axisymmetric box `x in [0, 4]`, `r in [0, 2]` (axis at `j = 0`, slightly stretched in both directions so that
/// the geometry is not uniform) with the characteristic far field on the three outer boundaries (`outflow` on `imax`).
pub fn far_field_box(mach: f64, outflow: Bc) -> Solver {
    let (ni, nj) = (40usize, 20usize);
    let mut x = vec![0.0; (ni + 1) * (nj + 1)];
    let mut r = x.clone();
    for j in 0..=nj {
        for i in 0..=ni {
            x[j * (ni + 1) + i] = 4.0 * (i as f64 / ni as f64).powf(1.15);
            r[j * (ni + 1) + i] = 2.0 * (j as f64 / nj as f64).powf(1.1);
        }
    }
    let cfg = Config {
        cfl: 0.5,
        limiter: Limiter::Minmod,
        freestream: freestream(mach),
        bc: [Bc::Characteristic, outflow, Bc::SlipWall, Bc::Characteristic],
        first_order_iters: 5,
    };
    Solver::new(Grid::from_nodes(ni, nj, x, r), cfg)
}

/// Smooth density / pressure / transverse-velocity perturbation of the free stream, strong enough to put the far-field faces
/// in the subsonic inflow, subsonic outflow and (at M = 1.3) supersonic branches of the characteristic BC.
pub fn perturbed_state(s: &Solver) -> Vec<[f64; 4]> {
    let fs = s.cfg.freestream;
    (0..s.grid.ni * s.grid.nj)
        .map(|c| {
            let (xc, rc) = (s.grid.xc[c], s.grid.rc[c]);
            let g = (-((xc - 3.0).powi(2) + (rc - 1.4).powi(2)) / 0.8).exp() + 0.5 * (-((xc - 0.6).powi(2) + (rc - 1.6).powi(2)) / 0.5).exp();
            prim_to_cons(&[fs[0] * (1.0 + 0.25 * g), fs[1] + 0.2 * g, 0.3 * g * (0.5 * xc).sin(), fs[3] * (1.0 + 0.4 * g)])
        })
        .collect()
}

/// `max |a - b| / (1 + |b|)` over the cells and the four components.
pub fn state_diff(a: &[[f64; 4]], b: &[[f64; 4]]) -> f64 {
    a.iter().zip(b).fold(0.0f64, |m, (x, y)| (0..4).fold(m, |m, k| m.max((x[k] - y[k]).abs() / (1.0 + y[k].abs()))))
}

/// `max |a - b| / b` over a residual history.
pub fn residual_diff(a: &[f64], b: &[f64]) -> f64 {
    a.iter().zip(b).fold(0.0f64, |m, (x, y)| m.max((x - y).abs() / y))
}
