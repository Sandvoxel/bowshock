//! A single-Mach-number case: body, grids and numerics, and the solver built from them.

use crate::engine::body_grid::{base_domain_grid, large_domain_grid, BaseBody, BaseGridParams, LargeGridParams, NozzleRows};
use crate::engine::numerics::Numerics;
use crate::error::Error;
use crate::progress::ProgressSink;
use bowshock_cfd::gas::freestream;
use bowshock_cfd::gas::Prim;
use bowshock_cfd::solver::{Bc, Config, Nozzle, Solver, Turbulence, Viscous};

/// Treatment of the region behind the body.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Wake {
    /// The cylinder continues as a sting to the outflow plane (`large_domain_grid`), no base.
    Sting,
    /// Finite body with a flat base and a wake region (`base_domain_grid`).
    Base,
}

/// Motor jet at the nozzle exit, internal units.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Jet {
    /// Exit radius.
    pub r_exit: f64,
    /// Primitive state at the exit (supersonic, axial).
    pub state: Prim,
    /// `[k, omega]` of the jet.
    pub turb: [f64; 2],
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
        let y1 = self.grid_base.dr_wall;
        let re = self.body.base_radius();
        let n_jet = layer_count(jet.r_exit, y1);
        let n_ann = (2 * layer_count(0.5 * (re - jet.r_exit), y1)).max(4);
        self.grid_base.n_gap = n_jet + n_ann;
        self.grid_base.nozzle = Some(NozzleRows { r_exit: jet.r_exit, n_jet });
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

/// Smallest even integer `>= x` (`x >= 0`).
fn even_ceil(x: f64) -> usize {
    let n = x.ceil() as usize;
    n + n % 2
}

/// Number of rows of a wall-clustered layer of height `h` with first cell height `y1` and growth ratio 1.2 (even).
fn layer_count(h: f64, y1: f64) -> usize {
    even_ceil((1.0 + 0.2 * h / y1).ln() / 1.2f64.ln())
}

/// First cell height at the body wall \[d\] for `y_plus` at the highest Mach number `mach_max` of a sweep: the flat-plate friction over the body length,
/// `cf = 0.027 (re_c M L)^(-1/7)`, `y1 = y+ / (re_c M sqrt(cf/2))` (`re_c = rho c d / mu` of the free stream).
pub(crate) fn wall_spacing(body: &BaseBody, re_c: f64, mach_max: f64, y_plus: f64) -> f64 {
    let cf = 0.027 * (re_c * mach_max * body.x_base()).powf(-1.0 / 7.0);
    y_plus / (re_c * mach_max * (0.5 * cf).sqrt())
}

/// Wall-resolve both meridian grids of a case for the first cell height `y1`: radial cell counts for a growth ratio of 1.2, the gap behind the base clustered at
/// the base corner only.
pub(crate) fn resolve_walls(grid_base: &mut BaseGridParams, grid_sting: &mut LargeGridParams, body: &BaseBody, y1: f64) {
    let (re, rs) = (body.base_radius(), body.sting_radius);
    grid_base.dr_wall = y1;
    grid_base.nj_out = layer_count(grid_base.r_far - re, y1);
    grid_base.n_gap = layer_count(re - rs, y1);
    grid_base.gap_one_sided = true;
    grid_sting.dr_wall = y1;
    grid_sting.nj = layer_count(grid_sting.r_far, y1);
}

/// Longest mean axial cell of an aft segment (tube or cone) of the grids of [`Case::new`] at level 0, in nose diameters; level `l`
/// divides it by `2^l`. A segment has 40 cells (tube), 36 (last tube of a sting grid) or 24 (cone) at level 0 however long it is, so a
/// tube of 40 d, the longest body the facade first accepted, had a mean of 1 d per cell (its largest cell, in the middle of the clustered
/// distribution, 3 d). A longer segment takes more cells to stay within that mean, and so within that largest cell; a shorter one
/// keeps its count.
const MAX_AXIAL_CELL: f64 = 1.0;

/// Cells of an aft segment of `length` at grid level `log2(f)`: `per` (the count of the segment kind, already scaled with `f`), or more
/// so that no cell is longer than [`MAX_AXIAL_CELL`]` / f`, and never finer than the end-cell size `dx_tip`; even, at least 4.
fn segment_cells(per: usize, length: f64, f: usize, dx_tip: f64) -> usize {
    let coarsest = even_ceil(length * f as f64 / MAX_AXIAL_CELL);
    per.max(coarsest).min(even_ceil(length / dx_tip)).max(4)
}

/// Sting-wake grid parameters of [`Case::new`] for `body` at grid `level` (domain `10 L` upstream and radially, `5 L` downstream).
pub(crate) fn sting_grid(body: &BaseBody, level: u32) -> LargeGridParams {
    let f = 1usize << level;
    let dx_tip = 0.03 / f as f64;
    let l_body = body.x_base();
    let nseg = body.segments.len();
    let n_seg_sting: Vec<usize> = body
        .segments
        .iter()
        .enumerate()
        .map(|(k, s)| {
            segment_cells(
                if s.slope != 0.0 {
                    24 * f
                } else if k + 1 == nseg {
                    36 * f
                } else {
                    40 * f
                },
                s.length,
                f,
                dx_tip,
            )
        })
        .collect();
    LargeGridParams {
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
    }
}

/// Base-wake grid parameters of [`Case::new`] for `body` at grid level `level` (domain `10 L` upstream and radially, `5 L` downstream).
pub(crate) fn base_grid(body: &BaseBody, level: u32) -> BaseGridParams {
    let f = 1usize << level;
    let dx_tip = 0.03 / f as f64;
    let l_body = body.x_base();
    let n_sting = if body.sting_radius > 0.0 { 4 * f } else { 0 };
    // cells per segment: tubes are finer than cones, never finer than the end-cell size `dx_tip` allows, never coarser than
    // `MAX_AXIAL_CELL / f` (see `segment_cells`)
    let n_seg: Vec<usize> = body.segments.iter().map(|s| segment_cells(if s.slope == 0.0 { 40 * f } else { 24 * f }, s.length, f, dx_tip)).collect();
    BaseGridParams {
        body: body.clone(),
        x_up: 10.0 * l_body,
        x_down: 5.0 * l_body,
        r_far: 10.0 * l_body,
        n_up: 28 * f,
        n_nose: 48 * f,
        n_seg,
        n_wake: 40 * f,
        n_sting,
        n_gap: 12 * f,
        nj_out: 64 * f,
        dx_tip,
        dr_wall: 0.03 / f as f64,
        gap_one_sided: false,
        nozzle: None,
    }
}
#[cfg(test)]
mod tests {
    use super::*;

    /// A segment that the facade accepted before the cell count followed the length keeps its count at every level; a longer one grows
    /// linearly with the length and never has a cell above `1 / f`.
    #[test]
    fn segment_cells_keep_the_old_counts_and_grow_past_the_old_limit() {
        for level in 0..3u32 {
            let f = 1usize << level;
            let dx_tip = 0.03 / f as f64;
            for per in [24 * f, 40 * f] {
                for k in 1..=4 * per as i32 {
                    let length = f64::from(k) * 0.25 / f as f64;
                    let old = per.min(even_ceil(length / dx_tip)).max(4);
                    let new = segment_cells(per, length, f, dx_tip);
                    if length * f as f64 <= per as f64 {
                        assert_eq!(new, old, "level {level}, per {per}, length {length}");
                    } else {
                        assert!(new > old && length / new as f64 <= 1.0 / f as f64 + 1e-12, "level {level}, per {per}, length {length}: {new}");
                    }
                }
            }
        }
        // the 46 d rocket's 25.7 d tube keeps its 40 cells, an 80 d one (59.7 d tube) gets 60
        assert_eq!(segment_cells(40, 25.7, 1, 0.03), 40);
        assert_eq!(segment_cells(40, 59.7, 1, 0.03), 60);
        assert_eq!(segment_cells(80, 59.7, 2, 0.015), 120);
    }
}
