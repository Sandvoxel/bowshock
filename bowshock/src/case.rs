//! A single-Mach-number case: body, grids and numerics, and the solver built from them.

use crate::body_grid::{base_domain_grid, large_domain_grid, BaseBody, BaseGridParams, LargeGridParams};
use crate::numerics::Numerics;
use bowshock_core::gas::freestream;
use bowshock_core::solver::{Bc, Config, Solver};

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
}

impl Case {
    /// Domain `10 L` upstream and radially, `5 L` downstream (`L` = total body length), grid level 0/1/2 (cells per diameter
    /// doubling each level).
    pub fn new(mach: f64, body: BaseBody, level: u32, wake: Wake, numerics: Numerics) -> Case {
        let f = 1usize << level;
        let l_body = body.x_base();
        let n_boat = if body.boat_length > 0.0 { 24 * f } else { 0 };
        let n_sting = if body.sting_radius > 0.0 { 4 * f } else { 0 };
        let grid_base = BaseGridParams {
            body,
            x_up: 10.0 * l_body,
            x_down: 5.0 * l_body,
            r_far: 10.0 * l_body,
            n_up: 28 * f,
            n_nose: 48 * f,
            n_cyl: 40 * f,
            n_boat,
            n_wake: 40 * f,
            n_sting,
            n_gap: 12 * f,
            nj_out: 64 * f,
            dx_tip: 0.03 / f as f64,
            dr_wall: 0.03 / f as f64,
        };
        let grid_sting = LargeGridParams {
            body: body.body,
            cyl_length: body.cyl_length,
            x_up: 10.0 * l_body,
            x_down: 5.0 * l_body,
            r_far: 10.0 * l_body,
            n_up: 30 * f,
            n_nose: 48 * f,
            n_cyl: 36 * f,
            n_ext: 30 * f,
            nj: 70 * f,
            dx_tip: 0.03 / f as f64,
            dr_wall: 0.03 / f as f64,
        };
        Case { mach, body, wake, grid_base, grid_sting, numerics }
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
            Wake::Base => (b.n_up + b.n_nose + b.n_cyl + b.n_boat + b.n_wake, b.n_sting + b.n_gap + b.nj_out),
            Wake::Sting => (g.n_up + g.n_nose + g.n_cyl + g.n_ext, g.nj),
        }
    }

    /// Solver on the grid selected by `wake`, configured from `numerics`.
    pub fn build_solver(&self) -> Solver {
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
                let (grid, block) = base_domain_grid(&self.grid_base);
                s = Solver::new(grid, cfg);
                s.set_block(Some(block));
            }
            Wake::Sting => {
                s = Solver::new(large_domain_grid(&self.grid_sting), cfg);
            }
        }
        s.low_mach = n.low_mach;
        s.set_scheme(n.scheme);
        s
    }
}
