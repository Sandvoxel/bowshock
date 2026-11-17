//! The flow field of a point: the state the solver iterates, on the grid the drag is computed on.
//!
//! [`Analysis::keep_fields`](crate::Analysis::keep_fields) keeps the final field of every point in [`PolarPoint::field`](crate::PolarPoint::field);
//! [`Analysis::on_field`](crate::Analysis::on_field) hands out a snapshot of the run in flight every few hundred iterations, so a
//! caller can draw the shock and the wake as they form. Both are off unless asked for: on the 1.1 m example body a field of the coarse
//! viscous grid (21 216 cells) is about 0.5 MB and one of the medium grid (42 432 cells) about 1.1 MB.
//!
//! # The grid
//!
//! The grid is structured and body-fitted in the meridian plane `(x, r)`: `x` runs downstream from the nose tip, `r` away from the
//! axis, both in metres (in nose diameters for an analysis built with `Analysis::from_internal`). There are `ni` cells along `i`
//! (downstream) and `nj` along `j` (outwards). Node `(i, j)`, `0 <= i <= ni`, `0 <= j <= nj`, is stored at `j * (ni + 1) + i`; cell
//! `(i, j)` is the quad of the nodes `(i, j)`, `(i + 1, j)`, `(i + 1, j + 1)`, `(i, j + 1)` and is stored at `j * ni + i`.
//!
//! An analysis solves each point on one grid, the one of its wake treatment ([`Field::wake`]): the sting grid, whose row `j = 0`
//! follows the axis, the body surface and the sting, or the base-wake grid, whose rows below the body surface are blocked beside
//! the body (and inside a sting) and carry the wake behind the base. Blocked cells are flagged in [`Field::solid`]; they are not
//! part of the flow (their nodes lie inside the body) and hold the free stream. With
//! [`Analysis::estimate_uncertainty`](crate::Analysis::estimate_uncertainty) the point is also solved one grid level away: the kept
//! field is the one of the requested resolution, which the reported drag comes from.
//!
//! # The state
//!
//! Per cell, the primitive state in the solver's units: density over the free-stream density, the axial and radial velocity over
//! the free-stream speed of sound, and the pressure over `rho_inf c_inf^2`. The free stream is therefore
//! `[1, Mach, 0, 1 / GAMMA]` ([`Field::freestream`]), which is all that is needed to form the Mach number, the pressure
//! coefficient and the density gradient ([`Field::mach_numbers`], [`Field::pressure_coefficients`], [`Field::density_gradient`]).
//! Values are stored as `f32`: a snapshot is for looking at, the drag is integrated from the solver's own `f64` state.

use crate::engine::case::Wake;
use bowshock_cfd::gas::cons_to_prim;
use bowshock_cfd::solver::Solver;
use std::fmt;
use std::sync::Arc;

/// Ratio of specific heats of the gas the solver uses (air).
pub const GAMMA: f64 = bowshock_cfd::gas3::GAMMA;

/// Solver iterations between two snapshots that an interactive caller should ask [`Analysis::on_field`](crate::Analysis::on_field)
/// for (the cost of a snapshot is in the documentation of that method).
pub const SNAPSHOT_INTERVAL: usize = 500;

/// A snapshot of the flow field of one run (see the [module documentation](self) for the layout and the units).
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Field {
    /// Free-stream Mach number of the run.
    pub mach: f64,
    /// Solver iterations done when the snapshot was taken (the iterations of the point for a final field).
    pub iteration: usize,
    /// The grid of the run: the sting grid or the base-wake grid.
    pub wake: Wake,
    /// Cells along `i` (downstream).
    pub ni: usize,
    /// Cells along `j` (away from the axis).
    pub nj: usize,
    /// Axial node coordinates from the nose tip, `(ni + 1) * (nj + 1)`.
    pub x: Vec<f32>,
    /// Radial node coordinates, `(ni + 1) * (nj + 1)`.
    pub r: Vec<f32>,
    /// Per cell: the cell is blocked (inside the body or the sting), not part of the flow.
    pub solid: Vec<bool>,
    /// Density per cell, over the free-stream density.
    pub rho: Vec<f32>,
    /// Axial velocity per cell, over the free-stream speed of sound.
    pub u: Vec<f32>,
    /// Radial velocity per cell, over the free-stream speed of sound.
    pub v: Vec<f32>,
    /// Pressure per cell, over `rho_inf c_inf^2`.
    pub p: Vec<f32>,
    /// The free stream `[rho, u, v, p]` in the same units: `[1, Mach, 0, 1 / GAMMA]`.
    pub freestream: [f64; 4],
}

impl Field {
    /// The state of `s` after `iteration` iterations at free-stream Mach number `mach`, lengths multiplied by `scale`.
    pub(crate) fn of_solver(s: &Solver, mach: f64, iteration: usize, wake: Wake, scale: f64) -> Field {
        let g = &s.grid;
        let (ni, nj) = (g.ni, g.nj);
        let fs = s.cfg.freestream;
        let block = s.block();
        let n = ni * nj;
        let mut f = Field {
            mach,
            iteration,
            wake,
            ni,
            nj,
            x: g.x.iter().map(|x| (x * scale) as f32).collect(),
            r: g.r.iter().map(|r| (r * scale) as f32).collect(),
            solid: Vec::with_capacity(n),
            rho: Vec::with_capacity(n),
            u: Vec::with_capacity(n),
            v: Vec::with_capacity(n),
            p: Vec::with_capacity(n),
            freestream: fs,
        };
        for (c, q) in s.u.iter().enumerate() {
            let solid = block.is_some_and(|b| b.solid(c % ni, c / ni));
            let w = if solid { fs } else { cons_to_prim(q) };
            f.solid.push(solid);
            f.rho.push(w[0] as f32);
            f.u.push(w[1] as f32);
            f.v.push(w[2] as f32);
            f.p.push(w[3] as f32);
        }
        f
    }

    /// Number of cells, `ni * nj`.
    pub fn cells(&self) -> usize {
        self.ni * self.nj
    }

    /// Coordinates `(x, r)` of node `(i, j)`.
    pub fn node(&self, i: usize, j: usize) -> (f32, f32) {
        let k = j * (self.ni + 1) + i;
        (self.x[k], self.r[k])
    }

    /// Centre `(x, r)` of cell `(i, j)`: the mean of its four nodes.
    pub fn centre(&self, i: usize, j: usize) -> (f32, f32) {
        let (a, b, c, d) = (self.node(i, j), self.node(i + 1, j), self.node(i + 1, j + 1), self.node(i, j + 1));
        (0.25 * (a.0 + b.0 + c.0 + d.0), 0.25 * (a.1 + b.1 + c.1 + d.1))
    }

    /// Whether cell `(i, j)` is part of the flow (not blocked).
    pub fn is_fluid(&self, i: usize, j: usize) -> bool {
        !self.solid[j * self.ni + i]
    }

    /// The first row of column `i` that is part of the flow (`nj` if the whole column is blocked).
    pub fn first_fluid_row(&self, i: usize) -> usize {
        (0..self.nj).find(|&j| self.is_fluid(i, j)).unwrap_or(self.nj)
    }

    /// Mach number of cell `c` (`j * ni + i`).
    pub fn mach_number(&self, c: usize) -> f32 {
        self.u[c].hypot(self.v[c]) / (GAMMA as f32 * self.p[c] / self.rho[c]).sqrt()
    }

    /// Pressure coefficient of cell `c`: `(p - p_inf) / (rho_inf u_inf^2 / 2)`.
    pub fn pressure_coefficient(&self, c: usize) -> f32 {
        let [rho, u, _, p] = self.freestream;
        ((f64::from(self.p[c]) - p) / (0.5 * rho * u * u)) as f32
    }

    /// Mach number per cell; NaN in blocked cells.
    pub fn mach_numbers(&self) -> Vec<f32> {
        (0..self.cells()).map(|c| if self.solid[c] { f32::NAN } else { self.mach_number(c) }).collect()
    }

    /// Pressure coefficient per cell; NaN in blocked cells.
    pub fn pressure_coefficients(&self) -> Vec<f32> {
        (0..self.cells()).map(|c| if self.solid[c] { f32::NAN } else { self.pressure_coefficient(c) }).collect()
    }

    /// Magnitude of the gradient of the density (over the free-stream density) per cell, per unit of length of the grid (1/m): what
    /// a schlieren picture shows. From differences of the cell-centre values on the curvilinear grid: central along each grid
    /// direction, one-sided at the boundaries and beside blocked cells. NaN in blocked cells.
    pub fn density_gradient(&self) -> Vec<f32> {
        let (ni, nj) = (self.ni, self.nj);
        let centres: Vec<(f32, f32)> = (0..self.cells()).map(|c| self.centre(c % ni, c / ni)).collect();
        let fluid = |i: usize, j: usize| !self.solid[j * ni + i];
        let rho_inf = self.freestream[0] as f32;
        let mut out = vec![f32::NAN; self.cells()];
        for j in 0..nj {
            for i in 0..ni {
                if !fluid(i, j) {
                    continue;
                }
                let c = j * ni + i;
                // the two ends of the difference along i and along j: the neighbours where they are flow cells, else the cell itself
                let a = if i > 0 && fluid(i - 1, j) { c - 1 } else { c };
                let b = if i + 1 < ni && fluid(i + 1, j) { c + 1 } else { c };
                let d = if j > 0 && fluid(i, j - 1) { c - ni } else { c };
                let e = if j + 1 < nj && fluid(i, j + 1) { c + ni } else { c };
                let diff = |hi: usize, lo: usize| (centres[hi].0 - centres[lo].0, centres[hi].1 - centres[lo].1, self.rho[hi] - self.rho[lo]);
                let (x_xi, r_xi, rho_xi) = diff(b, a);
                let (x_eta, r_eta, rho_eta) = diff(e, d);
                let det = x_xi * r_eta - r_xi * x_eta;
                out[c] = if det.abs() > 0.0 {
                    let gx = (rho_xi * r_eta - rho_eta * r_xi) / det;
                    let gr = (rho_eta * x_xi - rho_xi * x_eta) / det;
                    gx.hypot(gr) / rho_inf
                } else {
                    // a single row or column of flow cells: the gradient along the one direction that has a difference
                    let along = |dx: f32, dr: f32, drho: f32| if dx.hypot(dr) > 0.0 { drho.abs() / dx.hypot(dr) } else { 0.0 };
                    along(x_xi, r_xi, rho_xi).max(along(x_eta, r_eta, rho_eta)) / rho_inf
                };
            }
        }
        out
    }

    /// Pressure coefficient along the body surface, in axial order: for each column whose lower face is a surface (not the axis, and
    /// downstream of the nose tip), the pressure of the first two flow cells extrapolated linearly to the face centre, the rule the
    /// reported wall Cp of a point uses. Behind the base this includes the sting of a sting grid, which the point's own wall list
    /// leaves out.
    pub fn wall(&self) -> Vec<crate::analysis::WallCp> {
        let ni = self.ni;
        let dist = |i: usize, j: usize, at: (f32, f32)| {
            let c = self.centre(i, j);
            f64::from((c.0 - at.0).hypot(c.1 - at.1))
        };
        let mut out = Vec::new();
        for i in 0..ni {
            let j = self.first_fluid_row(i);
            if j + 1 >= self.nj {
                continue;
            }
            let (a, b) = (self.node(i, j), self.node(i + 1, j));
            let mid = (0.5 * (a.0 + b.0), 0.5 * (a.1 + b.1));
            if mid.0 <= 0.0 || mid.1 <= 0.0 {
                continue;
            }
            let (d0, d1) = (dist(i, j, mid), dist(i, j + 1, mid));
            let (p0, p1) = (f64::from(self.p[j * ni + i]), f64::from(self.p[(j + 1) * ni + i]));
            let pw = if d1 > d0 { p0 + (p0 - p1) * d0 / (d1 - d0) } else { p0 };
            let [rho, u, _, p_inf] = self.freestream;
            out.push(crate::analysis::WallCp { x: f64::from(mid.0), r: f64::from(mid.1), cp: (pw - p_inf) / (0.5 * rho * u * u) });
        }
        out
    }

    /// The lower edge of the flow in the meridian plane, from the inflow plane to the outflow plane, as a polyline of `[x, r]`:
    /// the axis ahead of the nose tip, the body surface, the flat base (base-wake grid) and the sting or the axis behind it. The
    /// body is what lies between this line and the axis.
    pub fn surface(&self) -> Vec<[f32; 2]> {
        let mut line: Vec<[f32; 2]> = Vec::with_capacity(self.ni + 3);
        let mut push = |p: (f32, f32)| {
            if line.last() != Some(&[p.0, p.1]) {
                line.push([p.0, p.1]);
            }
        };
        for i in 0..self.ni {
            let j = self.first_fluid_row(i);
            push(self.node(i, j));
            push(self.node(i + 1, j));
        }
        line
    }
}

/// The receiver of [`Analysis::on_field`](crate::Analysis::on_field) and how often it is called.
#[derive(Clone)]
pub(crate) struct FieldSink {
    /// Iterations between two snapshots: a multiple of the progress sampling interval.
    pub(crate) every: usize,
    pub(crate) call: Arc<dyn Fn(Field) + Send + Sync>,
}

impl FieldSink {
    pub(crate) fn new(every: usize, f: impl Fn(Field) + Send + Sync + 'static) -> FieldSink {
        let step = crate::progress::SAMPLE_INTERVAL;
        FieldSink { every: every.max(1).div_ceil(step) * step, call: Arc::new(f) }
    }
}

impl fmt::Debug for FieldSink {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("FieldSink").field("every", &self.every).finish()
    }
}

/// What a run does with its flow field: carried by the solver case.
#[doc(hidden)]
#[derive(Clone)]
pub struct FieldTap {
    /// Length of one solver length unit (a nose diameter) in the unit of the field's coordinates.
    pub(crate) scale: f64,
    /// Keep the final field in the result.
    pub(crate) keep: bool,
    /// Snapshots of the run in flight.
    pub(crate) live: Option<FieldSink>,
}

impl FieldTap {
    /// Whether a snapshot is due after `iteration` iterations.
    pub(crate) fn due(&self, iteration: usize) -> bool {
        self.live.as_ref().is_some_and(|l| iteration.is_multiple_of(l.every))
    }

    /// Hands the state of `s` after `iteration` iterations to the receiver.
    pub(crate) fn send(&self, s: &Solver, mach: f64, iteration: usize, wake: Wake) {
        if let Some(l) = &self.live {
            (l.call)(Field::of_solver(s, mach, iteration, wake, self.scale));
        }
    }

    /// The final field of a run, if it is kept.
    pub(crate) fn last(&self, s: &Solver, mach: f64, iteration: usize, wake: Wake) -> Option<Field> {
        self.keep.then(|| Field::of_solver(s, mach, iteration, wake, self.scale))
    }
}

impl fmt::Debug for FieldTap {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("FieldTap").field("scale", &self.scale).field("keep", &self.keep).field("every", &self.live.as_ref().map(|l| l.every)).finish()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A 3 x 2 grid of unit squares with the density `rho = 1 + 2 x + 3 r` at the cell centres and cell (0, 0) blocked.
    fn linear() -> Field {
        let (ni, nj) = (3usize, 2usize);
        let (mut x, mut r) = (Vec::new(), Vec::new());
        for j in 0..=nj {
            for i in 0..=ni {
                x.push(i as f32);
                r.push(j as f32);
            }
        }
        let n = ni * nj;
        let rho = (0..n).map(|c| 1.0 + 2.0 * ((c % ni) as f32 + 0.5) + 3.0 * ((c / ni) as f32 + 0.5)).collect();
        let mut solid = vec![false; n];
        solid[0] = true;
        Field {
            mach: 2.0,
            iteration: 0,
            wake: Wake::Base,
            ni,
            nj,
            x,
            r,
            solid,
            rho,
            u: vec![2.0; n],
            v: vec![0.0; n],
            p: vec![(1.0 / GAMMA) as f32; n],
            freestream: [1.0, 2.0, 0.0, 1.0 / GAMMA],
        }
    }

    #[test]
    fn the_density_gradient_of_a_linear_field_is_exact_and_blocked_cells_have_none() {
        let f = linear();
        let g = f.density_gradient();
        assert!(g[0].is_nan(), "the blocked cell");
        let exact = 2f32.hypot(3.0);
        for (c, v) in g.iter().enumerate().skip(1) {
            // the cell above the blocked one has no flow cell to its other side along `j`: only the `i` difference is known
            let want = if c == 3 { 2.0 } else { exact };
            assert!((v - want).abs() < 1e-5, "cell {c}: {v} against {want}");
        }
    }

    #[test]
    fn the_free_stream_has_its_mach_number_and_no_pressure_coefficient() {
        let f = linear();
        let mut uniform = f.clone();
        uniform.rho = vec![1.0; f.cells()];
        assert!((uniform.mach_number(3) - 2.0).abs() < 1e-6);
        assert!(uniform.pressure_coefficient(3).abs() < 1e-6);
        assert!(uniform.mach_numbers()[0].is_nan() && uniform.pressure_coefficients()[0].is_nan());
    }

    #[test]
    fn the_surface_steps_down_where_the_blocked_cells_end() {
        // column 0 starts at row 1 (the blocked cell below it), the other columns at row 0
        assert_eq!(linear().surface(), vec![[0.0, 1.0], [1.0, 1.0], [1.0, 0.0], [2.0, 0.0], [3.0, 0.0]]);
    }

    #[test]
    fn the_snapshot_interval_is_rounded_up_to_the_sampling_interval() {
        let step = crate::progress::SAMPLE_INTERVAL;
        assert_eq!(FieldSink::new(1, |_| {}).every, step);
        assert_eq!(FieldSink::new(step + 1, |_| {}).every, 2 * step);
        assert_eq!(FieldSink::new(500, |_| {}).every, 500);
        let tap = FieldTap { scale: 1.0, keep: false, live: Some(FieldSink::new(500, |_| {})) };
        assert!(tap.due(500) && tap.due(1000) && !tap.due(550));
        assert!(!FieldTap { scale: 1.0, keep: true, live: None }.due(500));
    }
}
