//! Wedge and thin-sector grids of the 2D base-wake grid on today's `Grid3`, for GPU-vs-CPU checks and the `bench3` example (stage S2 preparation,
//! before the CPU topology of S1 exists).
//!
//! `wedge_grid` revolves the meridian grid of `base_domain_grid` by a small opening angle into `nk` azimuthal cells, the body block becomes one `SolidBox`
//! over every `k`. What it does **not** do (S1's job): the two axis runs (it sets `axis_cells = 0`, so the whole `j = 0` plane is the slip-wall branch and the
//! axis faces, which have no area, reflect about a zero normal), the generalised axis ghost, the wall classes. Rows of the block that lie at negative radius
//! (the block hangs `n_gap` rows below the surface, below the axis ahead of the nose) are clamped to the axis: zero-volume solid cells.
//! The k planes are the reflecting planes of today's solver. Good enough to compare the GPU solver with the CPU solver on the same grid, wrong physics.

use bowshock_cfd::grid3::{Grid3, SolidBox};
use bowshock_core::engine::body_grid::{base_domain_grid, BaseGridParams};
use bowshock_core::{Analysis, Atmosphere, BodySpec, FlightConditions, Resolution, Wake};

/// The base-grid parameters of the facade case (`Analysis::engine_cases`): the Euler base-wake grid, or, with `rans`, the wall-resolved grid sized for
/// `mach` (ISA at 120 m as the app's table).
pub fn base_grid_params(body: &BodySpec, mach: f64, rans: bool, resolution: Resolution) -> BaseGridParams {
    let mut a = Analysis::new(body).expect("body spec").resolution(resolution);
    a = if rans { a.viscous(FlightConditions { atmosphere: Atmosphere::Isa { altitude_m: 120.0 } }) } else { a.wake(Wake::Base) };
    a.engine_cases(&[mach]).expect("engine case").remove(0).grid_base
}

/// The cell counts of `p` cut to roughly a quarter (a few thousand cells), for CPU references that must stay quick.
pub fn small(p: &BaseGridParams) -> BaseGridParams {
    let mut q = p.clone();
    q.n_up = 6;
    q.n_nose = 10;
    q.n_seg = q.n_seg.iter().map(|&n| (n / 4).max(4)).collect();
    q.n_wake = 8;
    q.n_gap = 6;
    q.nj_out = 10;
    q
}

/// The base-wake grid revolved by `opening_deg` degrees into `nk` equal azimuthal cells (`theta` from 0 to the opening angle), the body block solid
/// over the whole azimuth.
pub fn wedge_grid(p: &BaseGridParams, nk: usize, opening_deg: f64) -> Grid3 {
    let (g2, block) = base_domain_grid(p).expect("the base grid builds");
    let n2 = (g2.ni + 1) * (g2.nj + 1);
    let (mut x, mut y, mut z) = (Vec::with_capacity(n2 * (nk + 1)), Vec::with_capacity(n2 * (nk + 1)), Vec::with_capacity(n2 * (nk + 1)));
    for k in 0..=nk {
        let (s, c) = (opening_deg.to_radians() * k as f64 / nk as f64).sin_cos();
        for n in 0..n2 {
            let r = g2.r[n].max(0.0);
            x.push(g2.x[n]);
            y.push(r * s);
            z.push(r * c);
        }
    }
    let mut g = Grid3::try_from_nodes(g2.ni, g2.nj, nk, x, y, z).expect("the revolved wedge has no negative volume");
    g.solid = vec![SolidBox { i: 0..block.i_end, j: 0..block.j_end, k: 0..nk }];
    g.axis_cells = 0;
    g
}
