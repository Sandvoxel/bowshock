//! CPU runner of a [`Case3`]: iterate the [`Solver3`] to a converged or stationary monitor (the normal force of a half domain, the axial force of a
//! wedge or a sector), and integrate the wall pressure into the force coefficients.
//!
//! Lengths are in nose diameters, the coefficients are referenced to `S_ref = pi/4` (nose diameter 1; the facade rescales to the
//! reference diameter). The pressure force, and for a viscous solver ([`Solver3::viscous`]) the shear of the wall faces ([`WallForces::shear`], from `Solver3::wall_tractions`: the very momentum flux the mean-flow equations see). The solved domain spans `theta_span` of the whole azimuth, so its forces are scaled by
//! `2 pi / theta_span` (the half domain `[0, pi]`: doubled).
//!
//! Wall faces are classified by the part of the solid cell behind them and the direction of the face ([`Wall`]), not by geometry.

use crate::engine::case3::Case3;
use crate::engine::grid3::Domain3;
use crate::engine::result::StopReason;
use crate::engine::run::MONITOR_LIMIT;
use crate::progress::{drop_orders, spread, Progress};
use bowshock_cfd::gas3::cons_to_prim3;
use bowshock_cfd::gas3::Cons3;
use bowshock_cfd::grid3::{Face3, Grid3, Part, Symmetry};
use bowshock_cfd::solver3::Solver3;
use std::ops::ControlFlow;

/// Iterations between samples of the normal force.
pub const SAMPLE_EVERY: usize = 50;
/// Iterations between the force samples of the mean-stationary rule (the oscillation of a subsonic base wake has a period of 50 to 220 iterations: the 50-iteration
/// samples of the range rule alias it).
pub const MEAN_EVERY: usize = 10;
/// Orders of density-residual drop that end a run as [`StopReason::ResidualDrop`].
pub const DROP_ORDERS: f64 = 6.0;

/// Pressure-force coefficients of the body plus fins.
#[derive(Clone, Copy, Debug)]
pub struct LiftForces {
    /// Normal force coefficient (`+z`, towards the lee side at positive incidence).
    pub cn: f64,
    /// Axial (pressure) force coefficient, `+x` downstream.
    pub ca: f64,
    /// Centre of pressure of the normal force from the nose tip, nose diameters; NaN when `|sum Fz| < 1e-12`.
    pub x_cp: f64,
}

/// Result of one 3D run.
#[derive(Clone, Debug)]
pub struct PointResult3 {
    pub mach: f64,
    /// Incidence \[rad\].
    pub alpha: f64,
    pub cn: f64,
    pub ca: f64,
    /// Nose diameters from the tip.
    pub x_cp: f64,
    pub iterations: usize,
    /// `ResidualDrop` (converged), `CdStationary` (CN stationary), `MaxIterations` or `NonFinite`.
    pub stop_reason: StopReason,
    /// Orders of density-residual drop below the reference (peak) at the end of the run.
    pub drop_orders: f64,
    /// `(iteration, CN)` every [`SAMPLE_EVERY`] iterations.
    pub cn_history: Vec<(usize, f64)>,
    /// Largest `y+` of the streamwise no-slip walls ([`y_plus_max`]) and the mean Cp of the base ([`base_cp`]): `NaN` for Euler and from the GPU runner, which does
    /// not fill them (the CPU runner, [`run3_on`], does).
    pub y_plus_max: f64,
    pub cp_base: f64,
    /// Means over the last averaging window of the force samples ([`StopRule3::observe`]) when the run sampled one full window: `cn`, `ca` and `x_cp` above are
    /// these means when `stop_reason` is `MeanStationary`, the last state otherwise (a `MaxIterations` run of an oscillating wake has them here).
    pub window: Option<WindowMean>,
}

/// Mean of the force coefficients over one averaging window of an oscillating run.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WindowMean {
    /// Iteration at the end of the window and its length.
    pub end: usize,
    pub length: usize,
    pub ca: f64,
    /// Zero without incidence.
    pub cn: f64,
    /// `cm / cn`, nose diameters from the tip; NaN without incidence.
    pub x_cp: f64,
    /// Largest relative half range `(max - min) / (2 |mean|)` of CA (and, at incidence, CN and the moment) over the window: the size of the oscillation, for an unsteady-wake warning.
    pub oscillation: f64,
}

/// Wall forces of the current state of `s`: the `j = 0` boundary walls of the grid (the body of a sting grid, up to the base station) and every
/// face between a fluid and a wall cell. See [`lift_forces_upto`] for the wall pressure.
pub fn lift_forces(case: &Case3, s: &Solver3) -> LiftForces {
    lift_forces_upto(case, s, case.body.x_base())
}

/// [`lift_forces`] restricted to the wall faces with centroid `x <= x_max` (the faces of the base are never cut). The wall pressure is extrapolated
/// linearly from the two first fluid cells normal to the wall, as the 2D facade does (`engine::run::wall_distribution`):
/// `pw = p0 + (p0 - p1) d0 / (d1 - d0)`, `d` the distance from the face centre to the cell centroid. A fin face whose second cell is solid or outside
/// the grid uses the first-cell pressure. `dF = (pw - p_inf) area n_into_body`.
pub fn lift_forces_upto(case: &Case3, s: &Solver3, x_max: f64) -> LiftForces {
    forces_from(case, &s.grid, &wall_forces_on(s, x_max))
}

/// [`lift_forces_upto`] on a grid, a conserved state `u` (index `Grid3::cell`, e.g. downloaded from the GPU) and the free-stream pressure. The sting
/// is not counted.
pub fn forces_on(case: &Case3, g: &Grid3, u: &[Cons3], p_inf: f64, x_max: f64) -> LiftForces {
    forces_from(case, g, &wall_forces(g, u, p_inf, x_max))
}

/// [`forces_on`] of wall forces `w` already integrated (the GPU runner reduces them on the device).
pub(crate) fn forces_from(case: &Case3, g: &Grid3, w: &WallForces) -> LiftForces {
    let (mut fx, mut fz, mut mz) = (0.0, 0.0, 0.0);
    for (c, a) in w.by.iter().chain(&w.shear) {
        if !matches!(*c, Wall::Sting | Wall::Jet) {
            fx += a[0];
            fz += a[1];
            mz += a[2];
        }
    }
    coefficients(case, g, [fx, fz, mz])
}

/// Pressure-force coefficients of one part of the wetted surface; `x_cp` (nose diameters from the tip) is NaN when `|sum Fz| < 1e-12`.
#[derive(Clone, Copy, Debug)]
pub struct PartForces {
    pub cn: f64,
    pub ca: f64,
    pub x_cp: f64,
}

/// [`LiftForces`] split by the surface the pressure acts on. The parts sum to [`forces_on`] (`x_max` = the base station) without the base.
#[derive(Clone, Copy, Debug)]
pub struct LiftBreakdown {
    /// Body wall (`j = 0` faces) up to the end of the nose.
    pub nose: PartForces,
    /// Body wall behind the nose up to the base station.
    pub afterbody: PartForces,
    /// Fin side faces (faces between a fluid and a solid cell in `k`).
    pub fin_sides: PartForces,
    /// Fin leading and trailing edge faces (`i` faces of the fin boxes).
    pub fin_edges: PartForces,
    /// Fin tip faces (`j` faces of the fin boxes).
    pub fin_tips: PartForces,
}

impl LiftBreakdown {
    /// The whole fin: sides, edges and tip.
    pub fn fins(&self) -> PartForces {
        let parts = [self.fin_sides, self.fin_edges, self.fin_tips];
        let cn: f64 = parts.iter().map(|p| p.cn).sum();
        let ca: f64 = parts.iter().map(|p| p.ca).sum();
        let m: f64 = parts.iter().filter(|p| p.x_cp.is_finite()).map(|p| p.cn * p.x_cp).sum();
        PartForces { cn, ca, x_cp: if cn.abs() < 1e-12 { f64::NAN } else { m / cn } }
    }
}

/// [`forces_on`] split into nose, afterbody and fin faces (the walls of the grid's parts, the base and the sting not counted).
pub fn breakdown_on(case: &Case3, g: &Grid3, u: &[Cons3], p_inf: f64) -> LiftBreakdown {
    breakdown_from(case, g, &wall_forces(g, u, p_inf, case.body.x_base()))
}

/// [`breakdown_on`] of the solver state: pressure plus, for a viscous solver, the shear of the classes.
pub fn breakdown_of(case: &Case3, s: &Solver3) -> LiftBreakdown {
    breakdown_from(case, &s.grid, &wall_forces_on(s, case.body.x_base()))
}

/// [`breakdown_on`] of wall forces `w` already integrated up to the base station (the GPU runner reduces them on the device).
pub(crate) fn breakdown_from(case: &Case3, g: &Grid3, w: &WallForces) -> LiftBreakdown {
    let part = |classes: &dyn Fn(Wall) -> bool| {
        let mut a = [0.0; 3];
        for (c, f) in w.by.iter().chain(&w.shear) {
            if classes(*c) {
                for q in 0..3 {
                    a[q] += f[q];
                }
            }
        }
        let f = coefficients(case, g, a);
        PartForces { cn: f.cn, ca: f.ca, x_cp: f.x_cp }
    };
    LiftBreakdown {
        nose: part(&|c| c == Wall::Body(0)),
        afterbody: part(&|c| matches!(c, Wall::Body(s) if s > 0)),
        fin_sides: part(&|c| c == Wall::FinSide),
        fin_edges: part(&|c| matches!(c, Wall::FinLe | Wall::FinTe)),
        fin_tips: part(&|c| c == Wall::FinTip),
    }
}

/// Whether the domain is an axisymmetric wedge or a fin sector: zero incidence, the axial force is the monitor and CN is not defined (reported as 0).
pub(crate) fn zero_incidence(g: &Grid3) -> bool {
    matches!(g.sym, Symmetry::Wedge | Symmetry::Sector { .. })
}

pub(crate) fn coefficients(case: &Case3, g: &Grid3, [fx, fz, mz]: [f64; 3]) -> LiftForces {
    let q = 0.5 * case.mach * case.mach;
    let s_ref = std::f64::consts::FRAC_PI_4;
    let norm = (2.0 * std::f64::consts::PI / g.theta_span) / (q * s_ref);
    if zero_incidence(g) {
        return LiftForces { cn: 0.0, ca: norm * fx, x_cp: f64::NAN };
    }
    LiftForces { cn: norm * fz, ca: norm * fx, x_cp: if fz.abs() < 1e-12 { f64::NAN } else { mz / fz } }
}

/// A class of wall faces: the part behind the face and the direction it faces.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Wall {
    /// Surface of body segment `n` (`0` the nose).
    Body(u16),
    /// The rear face of the body (the base).
    Base,
    /// The sting.
    Sting,
    /// Fin side faces (normal in `k`).
    FinSide,
    /// Fin leading edge (the solid is behind the face in `+i`).
    FinLe,
    /// Fin trailing edge.
    FinTe,
    /// Fin tip (normal in `j`).
    FinTip,
    Appendage,
    /// The nozzle exit of a motor-on case (the `+i` faces of `Part::Jet` cells): an inflow with the jet state, not a wall; no pressure force of the exit is counted.
    Jet,
}

/// The class of a wall face between a fluid cell and a cell of `part`: `dir` the direction of the face (0, 1, 2 = `i`, `j`, `k`), `solid_low` whether
/// the solid is on the low-index side of the face.
fn wall_class(part: Part, dir: usize, solid_low: bool) -> Wall {
    match part {
        Part::Body(_) if dir == 0 && solid_low => Wall::Base,
        Part::Body(s) => Wall::Body(s),
        Part::Sting => Wall::Sting,
        Part::Fin(_) => match dir {
            0 if solid_low => Wall::FinTe,
            0 => Wall::FinLe,
            1 => Wall::FinTip,
            _ => Wall::FinSide,
        },
        Part::Appendage(_) => Wall::Appendage,
        Part::Jet => Wall::Jet,
    }
}

/// Dimensional pressure-force sums `[Fx, Fz, Mz]` (`Mz = sum x Fz`) of the solved domain per wall class (not scaled to the whole body).
#[derive(Clone, Debug, Default)]
pub struct WallForces {
    pub by: Vec<(Wall, [f64; 3])>,
    /// The viscous (shear) force `[Fx, Fz, Mz]` per wall class, empty for Euler and for the device-side reduction of the GPU runner (pressure only).
    pub shear: Vec<(Wall, [f64; 3])>,
}

impl WallForces {
    fn add(&mut self, class: Wall, d: [f64; 3]) {
        match self.by.iter_mut().find(|(c, _)| *c == class) {
            Some((_, a)) => {
                for q in 0..3 {
                    a[q] += d[q];
                }
            }
            None => self.by.push((class, d)),
        }
    }

    /// The pressure sums of `class` (zero when no face of it exists).
    pub fn get(&self, class: Wall) -> [f64; 3] {
        self.by.iter().find(|(c, _)| *c == class).map_or([0.0; 3], |(_, a)| *a)
    }

    /// The shear sums of `class`.
    pub fn get_shear(&self, class: Wall) -> [f64; 3] {
        self.shear.iter().find(|(c, _)| *c == class).map_or([0.0; 3], |(_, a)| *a)
    }

    /// Pressure plus shear of `class`.
    pub fn get_total(&self, class: Wall) -> [f64; 3] {
        let (p, f) = (self.get(class), self.get_shear(class));
        [p[0] + f[0], p[1] + f[1], p[2] + f[2]]
    }

    fn add_shear(&mut self, class: Wall, d: [f64; 3]) {
        match self.shear.iter_mut().find(|(c, _)| *c == class) {
            Some((_, a)) => {
                for q in 0..3 {
                    a[q] += d[q];
                }
            }
            None => self.shear.push((class, d)),
        }
    }
}

/// One wall face for the force integration: its class, the part of the wall behind it, the first fluid cell `a`, the fluid cell `b` behind it (if any), the
/// face, `sign` (+1 when the face normal points into the body) and the four nodes `(i, j, k)` of the face in the order [`Grid3`] builds its faces from.
#[derive(Clone, Debug)]
pub(crate) struct WallFaceRef {
    pub class: Wall,
    pub part: Part,
    pub corners: [(usize, usize, usize); 4],
    pub a: usize,
    pub b: Option<usize>,
    pub face: Face3,
    pub sign: f64,
}

/// The wall faces of `g` in the order the force integration visits them: the `j = 0` boundary faces inside the grid's `wall_j0` runs (not on the axis) and
/// every face between a fluid cell and a wall cell (a solid cell that has a part; dead cells are no walls). The GPU runner builds its device face list from it.
pub(crate) fn wall_faces(g: &Grid3) -> Vec<WallFaceRef> {
    let (ni, nj, nk) = (g.ni, g.nj, g.nk);
    let solid = |i: usize, j: usize, k: usize| g.is_solid(i, j, k);
    let mut list: Vec<WallFaceRef> = Vec::new();
    // `a`: first fluid cell, `b`: the fluid cell behind it (if any); `sign`: +1 when the face normal points into the body
    let mut add = |part: Part, class: Wall, a: usize, b: Option<usize>, f: &Face3, sign: f64, corners: [(usize, usize, usize); 4]| {
        list.push(WallFaceRef { class, part, corners, a, b, face: *f, sign });
    };
    let wall_part = |i: usize, j: usize, k: usize| g.part_of(i, j, k);
    let j0_part: Vec<Option<Part>> = (0..ni).map(|i| g.wall_j0.iter().find(|(r, _)| r.contains(&i)).map(|(_, p)| *p)).collect();
    for k in 0..nk {
        for i in 0..ni {
            let Some(part) = j0_part[i] else { continue };
            let f = &g.jface[k * (nj + 1) * ni + i];
            // normal points into the fluid: into the body is its negative
            if !solid(i, 0, k) && !g.is_axis(i, 0) {
                add(
                    part,
                    wall_class(part, 1, false),
                    g.cell(i, 0, k),
                    (nj > 1 && !solid(i, 1, k)).then(|| g.cell(i, 1, k)),
                    f,
                    -1.0,
                    [(i, 0, k), (i, 0, k + 1), (i + 1, 0, k + 1), (i + 1, 0, k)],
                );
            }
        }
    }
    for k in 0..nk {
        for j in 0..nj {
            for i in 1..ni {
                let f = &g.iface[(k * nj + j) * (ni + 1) + i];
                let c = [(i, j, k), (i, j + 1, k), (i, j + 1, k + 1), (i, j, k + 1)];
                if let (false, Some(part)) = (solid(i - 1, j, k), wall_part(i, j, k)) {
                    add(part, wall_class(part, 0, false), g.cell(i - 1, j, k), (i >= 2 && !solid(i - 2, j, k)).then(|| g.cell(i - 2, j, k)), f, 1.0, c);
                } else if let (Some(part), false) = (wall_part(i - 1, j, k), solid(i, j, k)) {
                    add(part, wall_class(part, 0, true), g.cell(i, j, k), (i + 1 < ni && !solid(i + 1, j, k)).then(|| g.cell(i + 1, j, k)), f, -1.0, c);
                }
            }
        }
    }
    for k in 0..nk {
        for j in 1..nj {
            for i in 0..ni {
                let f = &g.jface[(k * (nj + 1) + j) * ni + i];
                let c = [(i, j, k), (i, j, k + 1), (i + 1, j, k + 1), (i + 1, j, k)];
                if let (false, Some(part)) = (solid(i, j - 1, k), wall_part(i, j, k)) {
                    add(part, wall_class(part, 1, false), g.cell(i, j - 1, k), (j >= 2 && !solid(i, j - 2, k)).then(|| g.cell(i, j - 2, k)), f, 1.0, c);
                } else if let (Some(part), false) = (wall_part(i, j - 1, k), solid(i, j, k)) {
                    add(part, wall_class(part, 1, true), g.cell(i, j, k), (j + 1 < nj && !solid(i, j + 1, k)).then(|| g.cell(i, j + 1, k)), f, -1.0, c);
                }
            }
        }
    }
    for k in 1..nk {
        for j in 0..nj {
            for i in 0..ni {
                let f = &g.kface[(k * nj + j) * ni + i];
                let c = [(i, j, k), (i + 1, j, k), (i + 1, j + 1, k), (i, j + 1, k)];
                if let (false, Some(part)) = (solid(i, j, k - 1), wall_part(i, j, k)) {
                    add(part, wall_class(part, 2, false), g.cell(i, j, k - 1), (k >= 2 && !solid(i, j, k - 2)).then(|| g.cell(i, j, k - 2)), f, 1.0, c);
                } else if let (Some(part), false) = (wall_part(i, j, k - 1), solid(i, j, k)) {
                    add(part, wall_class(part, 2, true), g.cell(i, j, k), (k + 1 < nk && !solid(i, j, k + 1)).then(|| g.cell(i, j, k + 1)), f, -1.0, c);
                }
            }
        }
    }
    list
}

/// Wall forces of the state `u` of `g` by class. The faces counted are those of [`wall_faces`] (the `j = 0` boundary faces inside the grid's `wall_j0` runs, not
/// on the axis, and every face between a fluid cell and a wall cell: a solid cell that has a part; dead cells are no walls). Faces with centroid `x > x_max` are
/// left out except the base.
pub fn wall_forces(g: &Grid3, u: &[Cons3], p_inf: f64, x_max: f64) -> WallForces {
    let mut acc = WallForces::default();
    for w in wall_samples(g, u) {
        let (f, class) = (&w.face, w.class);
        if f.c[0] > x_max + 1e-9 && class != Wall::Base {
            continue;
        }
        let d = (w.pw - p_inf) * f.area * w.sign;
        acc.add(class, [d * f.n[0], d * f.n[2], d * f.n[2] * f.c[0]]);
    }
    acc
}

/// A wall face with the wall pressure of the state: `pw = p0 + (p0 - p1) d0 / (d1 - d0)`, `p0`, `p1` the pressures of the first two fluid cells normal to the
/// wall and `d` their distances from the face centre; the first-cell pressure when the second cell is solid or outside the grid.
#[derive(Clone, Debug)]
pub struct WallSample {
    pub class: Wall,
    pub face: Face3,
    /// +1 when the face normal points into the body.
    pub sign: f64,
    pub pw: f64,
    /// Pressure of the first fluid cell.
    pub p0: f64,
}

/// The wall pressure of the wall face `w` in the state whose cell pressures are `p`: `pw = p0 + (p0 - p1) d0 / (d1 - d0)` (see [`WallSample`]). The one rule
/// the force integration and the viewer's wall field share.
pub(crate) fn wall_pressure(g: &Grid3, w: &WallFaceRef, p: impl Fn(usize) -> f64) -> f64 {
    let dist = |c: usize, f: &Face3| {
        let x = g.xc[c];
        ((x[0] - f.c[0]).powi(2) + (x[1] - f.c[1]).powi(2) + (x[2] - f.c[2]).powi(2)).sqrt()
    };
    let p0 = p(w.a);
    match w.b {
        Some(b) => {
            let (d0, d1) = (dist(w.a, &w.face), dist(b, &w.face));
            p0 + (p0 - p(b)) * d0 / (d1 - d0)
        }
        None => p0,
    }
}

/// The [`wall_faces`] of `g` with their wall pressures in the state `u` (index `Grid3::cell`).
pub fn wall_samples(g: &Grid3, u: &[Cons3]) -> Vec<WallSample> {
    let p = |c: usize| cons_to_prim3(&u[c])[4];
    wall_faces(g)
        .into_iter()
        .map(|w| {
            let pw = wall_pressure(g, &w, p);
            WallSample { class: w.class, face: w.face, sign: w.sign, pw, p0: p(w.a) }
        })
        .collect()
}

/// The zero-lift drag of a wedge or a sector split by surface, coefficients on `pi/4` like [`LiftForces::ca`]: what the 2D facade reports as
/// `cd_nose`, `cd_aft`, `cd_base`.
#[derive(Clone, Debug)]
pub struct DragParts {
    pub nose: f64,
    /// One per aft segment (`body.segments` order).
    pub aft: Vec<f64>,
    pub base: f64,
    pub fin_sides: f64,
    pub fin_edges: f64,
    pub fin_tips: f64,
    /// The friction (shear) drag of the same classes, zero for Euler (and for the device-side reduction of the GPU runner).
    pub friction: FrictionParts,
    /// Nose + aft + base + fins, pressure plus friction (the sting not counted).
    pub total: f64,
}

/// The friction drag of the wall classes of [`DragParts`], coefficients on `pi/4`.
#[derive(Clone, Debug, Default)]
pub struct FrictionParts {
    pub nose: f64,
    pub aft: Vec<f64>,
    pub base: f64,
    pub fin_sides: f64,
    pub fin_edges: f64,
    pub fin_tips: f64,
    /// The sum of the parts above.
    pub total: f64,
}

impl DragParts {
    /// The pressure drag: the parts without the friction.
    pub fn pressure_total(&self) -> f64 {
        self.total - self.friction.total
    }

    /// The friction of the body (nose and afterbody) and of the fins.
    pub fn body_friction(&self) -> f64 {
        self.friction.nose + self.friction.aft.iter().sum::<f64>()
    }

    pub fn fin_friction(&self) -> f64 {
        self.friction.fin_sides + self.friction.fin_edges + self.friction.fin_tips
    }
}

/// [`DragParts`] of the state `u` of `g` for the case `case` (axial force only; every segment of the body of `case`).
pub fn drag_parts(case: &Case3, g: &Grid3, u: &[Cons3], p_inf: f64) -> DragParts {
    drag_parts_from(case, g, &wall_forces(g, u, p_inf, f64::INFINITY))
}

/// [`drag_parts`] of wall forces `w` integrated over every face (the GPU runner reduces them on the device).
pub fn drag_parts_from(case: &Case3, g: &Grid3, w: &WallForces) -> DragParts {
    let q = 0.5 * case.mach * case.mach;
    let norm = (2.0 * std::f64::consts::PI / g.theta_span) / (q * std::f64::consts::FRAC_PI_4);
    let cd = |c: Wall| norm * w.get(c)[0];
    let cf = |c: Wall| norm * w.get_shear(c)[0];
    let nseg = case.body.segments.len();
    let aft: Vec<f64> = (0..nseg).map(|s| cd(Wall::Body(s as u16 + 1))).collect();
    let (nose, base) = (cd(Wall::Body(0)), cd(Wall::Base));
    let (fin_sides, fin_edges, fin_tips) = (cd(Wall::FinSide), cd(Wall::FinLe) + cd(Wall::FinTe), cd(Wall::FinTip));
    let mut friction = FrictionParts {
        nose: cf(Wall::Body(0)),
        aft: (0..nseg).map(|s| cf(Wall::Body(s as u16 + 1))).collect(),
        base: cf(Wall::Base),
        fin_sides: cf(Wall::FinSide),
        fin_edges: cf(Wall::FinLe) + cf(Wall::FinTe),
        fin_tips: cf(Wall::FinTip),
        total: 0.0,
    };
    friction.total = friction.nose + friction.aft.iter().sum::<f64>() + friction.base + friction.fin_sides + friction.fin_edges + friction.fin_tips;
    let total = nose + aft.iter().sum::<f64>() + base + fin_sides + fin_edges + fin_tips + friction.total;
    DragParts { nose, aft, base, fin_sides, fin_edges, fin_tips, friction, total }
}

/// [`drag_parts`] of the solver state: the pressure and, for a viscous solver, the friction of every class.
pub fn drag_parts_on(case: &Case3, s: &Solver3) -> DragParts {
    drag_parts_from(case, &s.grid, &wall_forces_on(s, f64::INFINITY))
}

/// The shear force of the no-slip wall faces of a viscous solver per wall class: `Solver3::wall_tractions` (force per area the fluid exerts on the wall) times
/// the face area, `[Fx, Fz, Mz]` with `Mz = x Fz` at the face centre; faces with centroid `x > x_max` are left out except the base. Empty for Euler.
pub fn wall_shear(s: &Solver3, x_max: f64) -> Vec<(Wall, [f64; 3])> {
    let t = s.wall_tractions();
    if t.is_empty() {
        return Vec::new();
    }
    let g = &s.grid;
    let class_of = wall_classes(g);
    let mut acc = WallForces::default();
    for tr in &t {
        let [i, j, k] = tr.idx;
        let Some(&class) = class_of.get(&(tr.dir, i, j, k)) else { continue };
        let f = face_at(g, tr.dir, i, j, k);
        if f.c[0] > x_max + 1e-9 && class != Wall::Base {
            continue;
        }
        let d = [tr.tau[0] * tr.area, tr.tau[2] * tr.area];
        acc.add_shear(class, [d[0], d[1], d[1] * f.c[0]]);
    }
    acc.shear
}

/// The wall forces of the solver state: the pressure of [`wall_forces`] and, for a viscous solver, the shear of [`wall_shear`].
pub fn wall_forces_on(s: &Solver3, x_max: f64) -> WallForces {
    let mut w = wall_forces(&s.grid, &s.u, s.cfg.freestream[4], x_max);
    w.shear = wall_shear(s, x_max);
    w
}

/// `(family, i, j, k)` of every wall face of the grid -> its class: the key is the first node of the face, which is its index in its family
/// (`Grid3::{iface, jface, kface}`: `(k nj + j)(ni + 1) + i`, `(k (nj + 1) + j) ni + i`, `(k nj + j) ni + i`).
fn wall_classes(g: &Grid3) -> std::collections::HashMap<(usize, usize, usize, usize), Wall> {
    wall_faces(g)
        .into_iter()
        .map(|w| {
            let c = w.corners;
            let dir = if c.iter().all(|n| n.0 == c[0].0) {
                0
            } else if c.iter().all(|n| n.1 == c[0].1) {
                1
            } else {
                2
            };
            ((dir, c[0].0, c[0].1, c[0].2), w.class)
        })
        .collect()
}

fn face_at(g: &Grid3, dir: usize, i: usize, j: usize, k: usize) -> Face3 {
    match dir {
        0 => g.iface[(k * g.nj + j) * (g.ni + 1) + i],
        1 => g.jface[(k * (g.nj + 1) + j) * g.ni + i],
        _ => g.kface[(k * g.nj + j) * g.ni + i],
    }
}

/// Largest `y+` of the streamwise no-slip walls (the faces whose normal is `j` or `k`: the body surface, the fin sides and tips; not the `i` faces of the base and of
/// blunt fin edges, where the log law has no meaning): `d_n sqrt(rho |tau|) / mu` of the first cell. `0` for Euler.
pub fn y_plus_max(s: &Solver3) -> f64 {
    y_plus_at(s).into_iter().map(|w| w.0).fold(0.0, f64::max)
}

/// [`y_plus_max`] over the faces of the wall classes for which `keep` holds (the body surface, the fin sides, the fin tips).
pub fn y_plus_max_of(s: &Solver3, keep: impl Fn(Wall) -> bool) -> f64 {
    y_plus_at(s).into_iter().filter(|w| keep(w.4)).map(|w| w.0).fold(0.0, f64::max)
}

/// The `y+` of the first cell at every streamwise no-slip wall face: `(y+, family, [i, j, k] of the face, first fluid cell, wall class)`; empty for Euler.
pub fn y_plus_at(s: &Solver3) -> Vec<(f64, usize, [usize; 3], usize, Wall)> {
    let Some(rans) = s.rans() else { return Vec::new() };
    let g = &s.grid;
    let classes = wall_classes(g);
    let mut out = Vec::new();
    for t in s.wall_tractions() {
        if t.dir == 0 {
            continue;
        }
        let [i, j, k] = t.idx;
        let Some(&class) = classes.get(&(t.dir, i, j, k)) else { continue };
        let f = face_at(g, t.dir, i, j, k);
        // the fluid cell of the face: the cell at the face's own index, or the one below it
        let own = g.cell(i.min(g.ni - 1), j.min(g.nj - 1), k.min(g.nk - 1));
        let cell = if g.part[own] == bowshock_cfd::grid3::PART_FLUID {
            own
        } else if t.dir == 1 {
            g.cell(i, j - 1, k)
        } else {
            g.cell(i, j, k - 1)
        };
        let x = g.xc[cell];
        let d_n = ((x[0] - f.c[0]) * t.n[0] + (x[1] - f.c[1]) * t.n[1] + (x[2] - f.c[2]) * t.n[2]).abs();
        let tau = (t.tau[0] * t.tau[0] + t.tau[1] * t.tau[1] + t.tau[2] * t.tau[2]).sqrt();
        out.push((d_n * (s.u[cell][0] * tau).sqrt() / rans.mu[cell], t.dir, t.idx, cell, class));
    }
    out
}

/// Mean pressure coefficient of the base face (area weighted over the `Wall::Base` faces, wall pressure as the force integration has it); `NaN` without a base.
pub fn base_cp(case: &Case3, s: &Solver3) -> f64 {
    let p_inf = s.cfg.freestream[4];
    let q = 0.5 * case.mach * case.mach;
    let (mut num, mut den) = (0.0, 0.0);
    for w in wall_samples(&s.grid, &s.u) {
        if w.class == Wall::Base {
            let a = w.face.area * w.face.n[0].abs();
            num += (w.pw - p_inf) * a;
            den += a;
        }
    }
    if den > 0.0 {
        num / den / q
    } else {
        f64::NAN
    }
}

/// The quantity a run waits on: CN of a half domain at incidence; the axial force coefficient of a wedge, a sector, or a half domain at zero incidence (CN is
/// zero by symmetry there, only round-off to wait on).
fn monitor(case: &Case3, s: &Solver3) -> f64 {
    let f = lift_forces(case, s);
    if zero_incidence(&s.grid) || case.alpha.abs() < 1e-12 {
        f.ca
    } else {
        f.cn
    }
}

/// Iterate `s` (built from `case`) until a stop rule fires, in this order at every iteration: a non-finite residual (or, at a sample,
/// CN) gives `NonFinite`; a residual drop of [`DROP_ORDERS`] orders below the peak residual of the second-order phase (counted from iteration
/// `first_order_iters` cold, from the start warm: the first-order start-up's residual is another scale, and a drop measured against it fired inside the
/// start-up on the cone and ogive wedges, at 147 iterations with a surface pressure 4 % low) gives `ResidualDrop`; at a sample with
/// `it + 1 >= max(window, the floor of [`Numerics3`])` and a range of the last `window / SAMPLE_EVERY` CN samples below `rel_tol |mean|` (whatever the
/// residual did) gives `CdStationary`; the iteration cap gives `MaxIterations`.
///
/// `warm`: keep the field of the previous run, set the free stream of `case` and restart the CFL ramp (no first-order start-up).
pub fn run3_on(case: &Case3, s: &mut Solver3, warm: bool) -> PointResult3 {
    s.cfg.freestream = case.freestream();
    s.restart_ramp();
    if warm {
        s.iter = s.iter.max(case.numerics.first_order_iters);
    }
    let mut rule = StopRule3::new(case, warm);
    let mut it = 0;
    while it < case.numerics.max_iter {
        let r = s.step();
        let cn = ((it + 1) % SAMPLE_EVERY == 0).then(|| monitor(case, s));
        it += 1;
        if rule.wants_forces(it, MEAN_EVERY) {
            rule.observe(it, &lift_forces(case, s));
        }
        if rule.check(it - 1, r, cn).is_some() {
            break;
        }
        if let Some(t) = case.snapshots.as_ref().filter(|t| t.due(it)) {
            t.send(case, &s.grid, &s.u, it);
        }
        if cn.is_some() && rule.report(case, it).is_break() {
            rule.cancel();
            break;
        }
    }
    let mut r = rule.finish(case, lift_forces(case, s), it);
    r.y_plus_max = y_plus_max(s);
    r.cp_base = base_cp(case, s);
    r
}

/// The stop rules of a 3D run, shared by the CPU and GPU runners. Feed it every iteration's residual (and, at iterations
/// `it + 1 = 0 mod SAMPLE_EVERY`, the sampled CN); see [`run3_on`] for the order of the rules.
pub struct StopRule3 {
    skip: usize,
    nwin: usize,
    window: usize,
    rel_tol: f64,
    rmax: f64,
    r: f64,
    hist: Vec<(usize, f64)>,
    reason: StopReason,
    min_iters: usize,
    mean_window: usize,
    mean_from: usize,
    mean_tol: f64,
    incidence: bool,
    /// `(iteration, [CA, CN, CN x_cp])` every [`MEAN_EVERY`] iterations after `mean_from`.
    samples: Vec<(usize, [f64; 3])>,
    mean_ready: bool,
    last_window: Option<WindowMean>,
}

impl StopRule3 {
    pub fn new(case: &Case3, warm: bool) -> StopRule3 {
        let n = &case.numerics;
        StopRule3 {
            skip: if warm { 0 } else { n.first_order_iters.max(20) },
            nwin: (n.window / SAMPLE_EVERY).max(1),
            window: n.window,
            rel_tol: n.rel_tol,
            rmax: 0.0,
            r: f64::NAN,
            hist: Vec::new(),
            reason: StopReason::MaxIterations,
            min_iters: if warm { n.warm_min_iters } else { n.stationary_min_iters },
            mean_window: n.mean_window,
            mean_from: n.mean_from,
            mean_tol: n.mean_tol,
            incidence: case.alpha.abs() >= 1e-12 && matches!(case.grid.domain, Domain3::Half { .. }),
            samples: Vec::new(),
            mean_ready: false,
            last_window: None,
        }
    }

    /// Whether the runner is to feed [`StopRule3::observe`] after `iterations` iterations: the mean rule is on, the iteration is a multiple of `every` and
    /// past `mean_from`.
    pub fn wants_forces(&self, iterations: usize, every: usize) -> bool {
        self.mean_window > 0 && iterations > self.mean_from && iterations.is_multiple_of(every)
    }

    /// The force coefficients `f` of the state after `iterations` iterations (see [`StopRule3::wants_forces`]); call before [`StopRule3::check`] of the same
    /// iteration. Whether the means of two consecutive windows of `mean_window` iterations agree ([`window_means`]) is decided here and ends the run, as
    /// [`StopReason::MeanStationary`], at the next `check` (after the residual and range rules).
    pub fn observe(&mut self, iterations: usize, f: &LiftForces) {
        if self.mean_window == 0 || !(f.ca.is_finite() && f.cn.is_finite()) {
            return;
        }
        let cm = if self.incidence && f.x_cp.is_finite() { f.cn * f.x_cp } else { 0.0 };
        self.samples.push((iterations, [f.ca, f.cn, cm]));
        if let Some((mean, agree)) = window_means(&self.samples, self.mean_window, self.mean_from, self.mean_tol, self.incidence) {
            self.last_window = Some(mean);
            self.mean_ready = agree;
        }
    }

    /// Reports the run's progress after `iterations` iterations to the case's sink; `Break` when the sink asks to stop, and `Continue`
    /// without a sink.
    pub fn report(&self, case: &Case3, iterations: usize) -> ControlFlow<()> {
        let Some(sink) = &case.progress else { return ControlFlow::Continue(()) };
        sink.report(&Progress {
            mach: case.mach,
            alpha: case.alpha,
            iteration: iterations,
            max_iterations: case.numerics.max_iter,
            residual_drop_orders: drop_orders(self.rmax, self.r),
            monitor: self.hist.last().map_or(f64::NAN, |h| h.1),
            spread: spread(&self.hist, self.nwin),
            tolerance: Some(self.rel_tol),
        })
    }

    /// Marks the run as ended by the sink: [`StopRule3::finish`] reports [`StopReason::Cancelled`].
    pub fn cancel(&mut self) {
        self.reason = StopReason::Cancelled;
    }

    /// Residual `r` of iteration `it` (0-based) and, at a sample iteration, the CN of the state after it. `Some(reason)` ends the run
    /// with `it + 1` iterations.
    pub fn check(&mut self, it: usize, r: f64, cn: Option<f64>) -> Option<StopReason> {
        self.r = r;
        let stop = |s: &mut Self, why| {
            s.reason = why;
            Some(why)
        };
        if !r.is_finite() {
            return stop(self, StopReason::NonFinite);
        }
        if it >= self.skip {
            self.rmax = self.rmax.max(r);
        }
        let drop = if self.rmax > 0.0 { (self.rmax / r.max(1e-300)).log10() } else { 0.0 };
        if drop >= DROP_ORDERS {
            return stop(self, StopReason::ResidualDrop);
        }
        if let Some(cn) = cn {
            if !cn.is_finite() || cn.abs() > MONITOR_LIMIT {
                return stop(self, StopReason::NonFinite);
            }
            self.hist.push((it + 1, cn));
            if it + 1 >= self.window.max(self.min_iters) && self.hist.len() >= self.nwin {
                let w = &self.hist[self.hist.len() - self.nwin..];
                // a state that no longer changes (every sample bit-identical) without a residual drop is a frozen run, not a converged one
                if drop < 1.0 && w.iter().all(|&(_, c)| c.to_bits() == w[0].1.to_bits()) {
                    return stop(self, StopReason::NonFinite);
                }
                let (lo, hi, sum) = w.iter().fold((f64::INFINITY, f64::NEG_INFINITY, 0.0), |a, &(_, c)| (a.0.min(c), a.1.max(c), a.2 + c));
                if hi - lo < self.rel_tol * (sum / self.nwin as f64).abs() {
                    return stop(self, StopReason::CdStationary);
                }
            }
        }
        if self.mean_ready {
            return stop(self, StopReason::MeanStationary);
        }
        None
    }

    /// Result after `iterations` iterations with the final forces `f`.
    pub fn finish(self, case: &Case3, f: LiftForces, iterations: usize) -> PointResult3 {
        let mut reason = self.reason;
        if reason != StopReason::NonFinite && !(f.cn.is_finite() && f.ca.is_finite() && f.cn.abs() <= MONITOR_LIMIT) {
            reason = StopReason::NonFinite;
        }
        let drop_orders = if self.rmax > 0.0 && self.r.is_finite() { (self.rmax / self.r.max(1e-300)).log10() } else { 0.0 };
        let mut f = f;
        if let (StopReason::MeanStationary, Some(w)) = (reason, self.last_window) {
            // the state at the stop is one phase of the oscillation: the result is the mean of the window
            (f.ca, f.cn, f.x_cp) = (w.ca, w.cn, if self.incidence { w.x_cp } else { f.x_cp });
        }
        PointResult3 {
            mach: case.mach,
            alpha: case.alpha,
            cn: f.cn,
            ca: f.ca,
            x_cp: f.x_cp,
            window: self.last_window,
            iterations,
            stop_reason: reason,
            drop_orders,
            cn_history: self.hist,
            y_plus_max: f64::NAN,
            cp_base: f64::NAN,
        }
    }
}

/// Means of CA, CN and the moment over the last window of `length` iterations of `samples` and of the window before it, and whether the two agree: for every
/// monitored coefficient (CA; at incidence also CN and the moment about the nose) `|m1 - m0| <= tol |m| + 1e-4` with `m = (m0 + m1) / 2`. `None` until both windows
/// lie after iteration `from` and hold at least 8 samples each. Returns the second window's means with its oscillation (largest relative half range).
pub fn window_means(samples: &[(usize, [f64; 3])], length: usize, from: usize, tol: f64, incidence: bool) -> Option<(WindowMean, bool)> {
    let &(end, _) = samples.last()?;
    if length == 0 || end < from + 2 * length {
        return None;
    }
    let lo1 = end - length;
    let lo0 = lo1 - length;
    let second: Vec<&[f64; 3]> = samples.iter().filter(|s| s.0 > lo1).map(|s| &s.1).collect();
    let first: Vec<&[f64; 3]> = samples.iter().filter(|s| s.0 > lo0 && s.0 <= lo1).map(|s| &s.1).collect();
    if first.len() < 8 || second.len() < 8 {
        return None;
    }
    let mean = |w: &[&[f64; 3]], c: usize| w.iter().map(|s| s[c]).sum::<f64>() / w.len() as f64;
    let n = if incidence { 3 } else { 1 };
    let (mut agree, mut oscillation) = (true, 0.0f64);
    for c in 0..n {
        let (m0, m1) = (mean(&first, c), mean(&second, c));
        let m = 0.5 * (m0 + m1);
        agree &= (m1 - m0).abs() <= tol * m.abs() + 1e-4;
        let (lo, hi) = second.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |(l, h), s| (l.min(s[c]), h.max(s[c])));
        oscillation = oscillation.max(0.5 * (hi - lo) / m1.abs().max(1e-12));
    }
    let (ca, cn, cm) = (mean(&second, 0), mean(&second, 1), mean(&second, 2));
    let x_cp = if incidence && cn.abs() > 1e-12 { cm / cn } else { f64::NAN };
    Some((WindowMean { end, length, ca, cn: if incidence { cn } else { 0.0 }, x_cp, oscillation }, agree))
}

/// Cold run on a fresh solver.
pub fn run3_cold(case: &Case3) -> Result<PointResult3, crate::error::Error> {
    let mut s = case.build_solver()?;
    Ok(run3_on(case, &mut s, false))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Feeds a synthetic run: residual `res(it)` every iteration, `cn(it + 1)` at the sample iterations. The stop, if any.
    fn feed(res: impl Fn(usize) -> f64, cn: impl Fn(usize) -> f64) -> Option<(usize, StopReason)> {
        let mut rule = StopRule3::new(&Case3::tiny_anf(2.0, 2f64.to_radians()), false);
        for it in 0..3000 {
            let sample = (it + 1) % SAMPLE_EVERY == 0;
            if let Some(why) = rule.check(it, res(it), sample.then(|| cn(it + 1))) {
                return Some((it + 1, why));
            }
        }
        None
    }

    #[test]
    fn a_frozen_state_without_a_residual_drop_is_a_failure() {
        // every CN sample bit-identical, the residual flat at 0.5: frozen, not stationary
        let stop = feed(|_| 0.5, |_| 0.316_928_495_035_045);
        assert_eq!(stop, Some((250, StopReason::NonFinite)));
    }

    #[test]
    fn a_stationary_state_with_a_residual_drop_still_stops_stationary() {
        // identical CN samples but the residual fell 2 orders (a converged limit): CdStationary as before
        let stop = feed(|it| if it < 150 { 1.0 } else { 1e-2 }, |_| 0.3);
        assert_eq!(stop, Some((250, StopReason::CdStationary)));
        // a CN that wiggles at round-off level without a drop is not frozen (not bit-identical): CdStationary
        let stop = feed(|_| 0.5, |it| 0.3 + 1e-9 * (it % 3) as f64);
        assert_eq!(stop, Some((250, StopReason::CdStationary)));
    }

    #[test]
    fn the_residual_drop_is_counted_from_the_second_order_phase() {
        // the first-order start-up (100 iterations) has a residual 6 orders above the second-order phase's, which is flat: no drop within the phase that
        // counts, so no stop on the drop (CN keeps changing, so not stationary either)
        let stop = feed(|it| if it < 100 { 1e3 } else { 1e-3 }, |it| 0.3 + 0.1 * ((it / 50) % 2) as f64);
        assert_eq!(stop, None);
        // the same drop inside the second-order phase stops the run
        let stop = feed(|it| if it < 200 { 1.0 } else { 1e-7 }, |it| 0.3 + 0.1 * ((it / 50) % 2) as f64);
        assert_eq!(stop, Some((201, StopReason::ResidualDrop)));
    }

    /// A subsonic base wake as measured on the GPU (M 0.5, 2 degrees): CA oscillating 57 % peak to peak with a period of 222 iterations plus a second line at 108,
    /// CN 120 % at 108, on a start-up transient that dies out.
    fn wake(it: usize, drift_per_1000: f64) -> LiftForces {
        let t = it as f64;
        let w = |p: f64, ph: f64| (2.0 * std::f64::consts::PI * t / p + ph).sin();
        let settle = 1.0 + 0.5 * (-t / 600.0).exp();
        let trend = 1.0 + drift_per_1000 * t / 1000.0;
        LiftForces {
            ca: 0.234 * trend * settle * (1.0 + 0.22 * w(222.0, 0.3) + 0.08 * w(108.0, 1.1)),
            cn: 0.0766 * settle * (1.0 + 0.5 * w(108.0, 0.7) + 0.1 * w(222.0, 2.0)),
            x_cp: 7.0 + 0.5 * w(150.0, 0.0),
        }
    }

    fn run_wake(drift: f64) -> (Option<(usize, StopReason)>, PointResult3) {
        let case = Case3::tiny_anf(0.5, 2f64.to_radians());
        let mut rule = StopRule3::new(&case, false);
        let mut stop = None;
        let mut last = 0;
        for it in 1..=8000 {
            last = it;
            let f = wake(it, drift);
            if rule.wants_forces(it, MEAN_EVERY) {
                rule.observe(it, &f);
            }
            // the monitor samples of the range rule every 50 iterations; the residual stays high (a limit cycle)
            let sample = (it % SAMPLE_EVERY == 0).then_some(f.cn);
            if let Some(why) = rule.check(it - 1, 1e-2, sample) {
                stop = Some((it, why));
                break;
            }
        }
        (stop, rule.finish(&case, wake(last, drift), last))
    }

    #[test]
    fn an_oscillating_wake_with_a_stationary_mean_stops_on_the_window_mean() {
        let (stop, r) = run_wake(0.0);
        let (it, why) = stop.expect("the run stops");
        assert_eq!(why, StopReason::MeanStationary);
        // two windows of 1500 after iteration 1000, and the start-up transient (600 iterations) decayed
        assert!((4000..=8000).contains(&it), "stopped at {it}");
        // the result is the window mean (exact: 0.234 times the settled factor), within the leftover of a window that is not a whole number of cycles, and the oscillation is reported
        let w = r.window.expect("a window");
        assert!((r.ca - 0.234).abs() < 0.234 * 0.012, "mean CA {} vs 0.234", r.ca);
        assert_eq!(r.ca, w.ca);
        assert!((r.cn - 0.0766).abs() < 0.0766 * 0.04, "mean CN {}", r.cn);
        assert!((r.x_cp - 7.0).abs() < 0.2, "x_cp {}", r.x_cp);
        assert!(w.oscillation > 0.5 && w.oscillation < 0.7, "oscillation {} (CN: 0.5 + 0.1 half range)", w.oscillation);
    }

    #[test]
    fn an_oscillating_wake_whose_mean_drifts_does_not_stop_on_the_mean() {
        // 4 % of CA per 1000 iterations: the means of consecutive windows differ by 6 %
        let (stop, r) = run_wake(0.04);
        assert!(stop.is_none(), "{stop:?}");
        assert!(r.window.is_some());
    }

    #[test]
    fn the_mean_rule_is_off_with_a_zero_window() {
        let mut case = Case3::tiny_anf(0.5, 2f64.to_radians());
        case.numerics.mean_window = 0;
        let mut rule = StopRule3::new(&case, false);
        assert!(!rule.wants_forces(5000, MEAN_EVERY));
        rule.observe(5000, &wake(5000, 0.0));
        assert_eq!(rule.check(4999, 1e-2, None), None);
    }
}
