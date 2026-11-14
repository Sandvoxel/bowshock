//! A single-Mach-number case: body, grids and numerics, and the solver built from them.

use crate::body_grid::{base_domain_grid, large_domain_grid, BaseBody, BaseGridParams, LargeGridParams};
use crate::error::Error;
use crate::numerics::Numerics;
use bowshock_core::gas::freestream;
use bowshock_core::solver::{Bc, Config, Solver, Turbulence, Viscous};

/// Treatment of the region behind the body.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Wake {
    /// The cylinder continues as a sting to the outflow plane (`large_domain_grid`), no base.
    Sting,
    /// Finite body with a flat base and a wake region (`base_domain_grid`).
    Base,
}

/// One Mach number on one body.
#[derive(Clone, Debug)]
pub struct Case {
    pub mach: f64,
    pub body: BaseBody,
    pub wake: Wake,
    pub grid_base: BaseGridParams,
    pub grid_sting: LargeGridParams,
    pub numerics: Numerics,
    /// Viscous flow parameters (RANS with the SST model); `None`: Euler equations.
    pub viscous: Option<Viscous>,
}

impl Case {
    /// Domain `10 L` upstream and radially, `5 L` downstream (`L` = total body length), grid level 0/1/2 (cells per diameter
    /// doubling each level).
    pub fn new(mach: f64, body: BaseBody, level: u32, wake: Wake, numerics: Numerics) -> Case {
        let f = 1usize << level;
        let dx_tip = 0.03 / f as f64;
        let l_body = body.x_base();
        let n_sting = if body.sting_radius > 0.0 { 4 * f } else { 0 };
        let nseg = body.segments.len();
        // cells per segment: tubes are finer than cones, never finer than the end-cell size `dx_tip` allows
        let count = |per: usize, length: f64| per.min(even_ceil(length / dx_tip)).max(4);
        let n_seg_base: Vec<usize> = body.segments.iter().map(|s| count(if s.slope == 0.0 { 40 * f } else { 24 * f }, s.length)).collect();
        let n_seg_sting: Vec<usize> = body
            .segments
            .iter()
            .enumerate()
            .map(|(k, s)| {
                count(
                    if s.slope != 0.0 {
                        24 * f
                    } else if k + 1 == nseg {
                        36 * f
                    } else {
                        40 * f
                    },
                    s.length,
                )
            })
            .collect();
        let grid_base = BaseGridParams {
            body: body.clone(),
            x_up: 10.0 * l_body,
            x_down: 5.0 * l_body,
            r_far: 10.0 * l_body,
            n_up: 28 * f,
            n_nose: 48 * f,
            n_seg: n_seg_base,
            n_wake: 40 * f,
            n_sting,
            n_gap: 12 * f,
            nj_out: 64 * f,
            dx_tip,
            dr_wall: 0.03 / f as f64,
            gap_one_sided: false,
        };
        let grid_sting = LargeGridParams {
            body: body.clone(),
            x_up: 10.0 * l_body,
            x_down: 5.0 * l_body,
            r_far: 10.0 * l_body,
            n_up: 30 * f,
            n_nose: 48 * f,
            n_seg: n_seg_sting,
            n_ext: 30 * f,
            nj: 70 * f,
            dx_tip,
            dr_wall: 0.03 / f as f64,
        };
        Case { mach, body, wake, grid_base, grid_sting, numerics, viscous: None }
    }

    /// Wall-resolved RANS case: [`Case::new`] with [`Numerics::viscous`], first cell height at the wall for `y+ ~ 1` at the highest
    /// Mach number of the sweep `mach_max` (flat-plate friction estimate over the body length), radial cell counts for a growth
    /// ratio of 1.2, the gap behind the base clustered at the base corner only, and the SST model with the no-slip surface starting at
    /// the nose tip. `re_c = rho c d_nose / mu` of the free stream, `sutherland = 110.4 K / T_inf`.
    pub fn viscous(mach: f64, body: BaseBody, level: u32, wake: Wake, re_c: f64, sutherland: f64, mach_max: f64) -> Case {
        let mut c = Case::new(mach, body, level, wake, Numerics::viscous(mach, level));
        let cf = 0.027 * (re_c * mach_max * c.body.x_base()).powf(-1.0 / 7.0);
        let y1 = 1.0 / (re_c * mach_max * (0.5 * cf).sqrt());
        let count = |h: f64| even_ceil((1.0 + 0.2 * h / y1).ln() / 1.2f64.ln());
        let (re, rs) = (c.body.base_radius(), c.body.sting_radius);
        c.grid_base.dr_wall = y1;
        c.grid_base.nj_out = count(c.grid_base.r_far - re);
        c.grid_base.n_gap = count(re - rs);
        c.grid_base.gap_one_sided = true;
        c.grid_sting.dr_wall = y1;
        c.grid_sting.nj = count(c.grid_sting.r_far);
        let wall_i_start = match wake {
            Wake::Base => c.grid_base.n_up,
            Wake::Sting => c.grid_sting.n_up,
        };
        c.viscous = Some(Viscous { re_c, sutherland, turbulence: Turbulence::Sst, wall_i_start });
        c
    }

    /// Scale the domain: inflow plane / outer boundary at `up` and the outflow plane at `down` total body lengths.
    pub fn with_domain(mut self, up: f64, down: f64) -> Case {
        let l = self.body.x_base();
        self.grid_base.x_up = up * l;
        self.grid_base.r_far = up * l;
        self.grid_base.x_down = down * l;
        self.grid_sting.x_up = up * l;
        self.grid_sting.r_far = up * l;
        self.grid_sting.x_down = down * l;
        self
    }

    /// Cell counts `(ni, nj)` of the grid selected by `wake`.
    pub fn cells(&self) -> (usize, usize) {
        let g = &self.grid_sting;
        let b = &self.grid_base;
        match self.wake {
            Wake::Base => (b.n_up + b.n_nose + b.n_seg.iter().sum::<usize>() + b.n_wake, b.n_sting + b.n_gap + b.nj_out),
            Wake::Sting => (g.n_up + g.n_nose + g.n_seg.iter().sum::<usize>() + g.n_ext, g.nj),
        }
    }

    /// Solver on the grid selected by `wake`, configured from `numerics`; panics on invalid grid parameters.
    pub fn build_solver(&self) -> Solver {
        self.try_build_solver().expect("valid grid parameters")
    }

    /// Like [`Case::build_solver`], returning the grid error instead of panicking.
    pub fn try_build_solver(&self) -> Result<Solver, Error> {
        let n = &self.numerics;
        let cfg = Config {
            cfl: n.cfl,
            limiter: n.limiter,
            freestream: freestream(self.mach),
            bc: [Bc::Characteristic, n.outflow, Bc::SlipWall, Bc::Characteristic],
            first_order_iters: n.first_order_iters,
        };
        let mut s;
        match self.wake {
            Wake::Base => {
                let (grid, block) = base_domain_grid(&self.grid_base)?;
                s = Solver::new(grid, cfg);
                s.set_block(Some(block));
            }
            Wake::Sting => {
                s = Solver::new(large_domain_grid(&self.grid_sting)?, cfg);
            }
        }
        s.low_mach = n.low_mach;
        s.set_scheme(n.scheme);
        s.set_viscous(self.viscous);
        Ok(s)
    }
}

/// Smallest even integer `>= x` (`x >= 0`).
fn even_ceil(x: f64) -> usize {
    let n = x.ceil() as usize;
    n + n % 2
}
