//! A single-Mach-number case: body, grids and numerics, and the solver built from them.

use crate::engine::body_grid::{base_domain_grid, large_domain_grid, BaseBody, BaseGridParams, LargeGridParams};
use crate::engine::meshing::{base_grid, motor_rows, resolve_walls, sting_grid, wall_spacing, Jet, Wake};
use crate::engine::numerics::Numerics;
use crate::error::Error;
use crate::progress::ProgressSink;
use bowshock_cfd::gas::freestream;
use bowshock_cfd::solver::{Bc, Config, Nozzle, Solver};
use bowshock_cfd::sst::{Turbulence, Viscous};

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
    /// Motor jet from the base; `None`: power-off.
    pub motor: Option<Jet>,
    /// Where the run reports its progress and can be stopped; `None`: silent.
    pub progress: Option<ProgressSink>,
    /// What the run does with its flow field (snapshots in flight, the final field in the result); `None`: nothing.
    pub fields: Option<crate::field::FieldTap>,
}

impl Case {
    /// Domain `10 L` upstream and radially, `5 L` downstream (`L` = total body length), grid level 0/1/2 (cells per diameter
    /// doubling each level).
    pub fn new(mach: f64, body: BaseBody, level: u32, wake: Wake, numerics: Numerics) -> Case {
        let grid_base = base_grid(&body, level);
        let grid_sting = sting_grid(&body, level);
        Case { mach, body, wake, grid_base, grid_sting, numerics, viscous: None, motor: None, progress: None, fields: None }
    }

    /// Wall-resolved RANS case: [`Case::new`] with [`Numerics::viscous`], first cell height at the wall for `y+ ~ 1` at the highest
    /// Mach number of the sweep `mach_max` (flat-plate friction estimate over the body length), radial cell counts for a growth
    /// ratio of 1.2, the gap behind the base clustered at the base corner only, and the SST model with the no-slip surface starting at
    /// the nose tip. `re_c = rho c d_nose / mu` of the free stream, `sutherland = 110.4 K / T_inf`.
    pub fn viscous(mach: f64, body: BaseBody, level: u32, wake: Wake, re_c: f64, sutherland: f64, mach_max: f64) -> Case {
        let mut c = Case::new(mach, body, level, wake, Numerics::viscous(mach, level));
        let y1 = wall_spacing(&c.body, re_c, mach_max, 1.0);
        resolve_walls(&mut c.grid_base, &mut c.grid_sting, &c.body, y1);
        let wall_i_start = match wake {
            Wake::Base => c.grid_base.n_up,
            Wake::Sting => c.grid_sting.n_up,
        };
        c.viscous = Some(Viscous { re_c, sutherland, turbulence: Turbulence::Sst, wall_i_start, roughness: 0.0 });
        c
    }

    /// Add a motor jet from the centre of the base to a case built by [`Case::viscous`] (panics otherwise, or without the
    /// base wake): splits the gap behind the base into the nozzle exit (rows clustered at the lip, `n_jet` rows) and the annulus
    /// around it (`n_ann` rows, at least 4), `n_gap = n_jet + n_ann`.
    pub fn with_motor(mut self, jet: Jet) -> Case {
        assert!(self.viscous.is_some() && self.wake == Wake::Base, "a motor needs a viscous case with the base wake");
        motor_rows(&mut self.grid_base, &self.body, &jet);
        self.motor = Some(jet);
        self
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
                if let Some(jet) = &self.motor {
                    let n_jet = self.grid_base.nozzle.map_or(0, |z| z.n_jet);
                    s.set_nozzle(Some(Nozzle { j_end: n_jet, state: jet.state, turb: jet.turb }));
                }
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
