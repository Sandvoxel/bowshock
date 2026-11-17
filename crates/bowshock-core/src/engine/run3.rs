//! CPU runner of a [`Case3`]: iterate the [`Solver3`] to a converged or stationary monitor (the normal force of a half domain, the axial force of a
//! wedge or a sector), and integrate the wall pressure into the force coefficients.
//!
//! Lengths are in nose diameters, the coefficients are referenced to `S_ref = pi/4` (nose diameter 1; the facade rescales to the
//! reference diameter). Pressure forces only (Euler). The solved domain spans `theta_span` of the whole azimuth, so its forces are scaled by
//! `2 pi / theta_span` (the half domain `[0, pi]`: doubled).
//!
//! Wall faces are classified by the part of the solid cell behind them and the direction of the face ([`Wall`]), not by geometry.

use crate::engine::case3::Case3;
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
    forces_on(case, &s.grid, &s.u, s.cfg.freestream[4], x_max)
}

/// [`lift_forces_upto`] on a grid, a conserved state `u` (index `Grid3::cell`, e.g. downloaded from the GPU) and the free-stream pressure. The sting
/// is not counted.
pub fn forces_on(case: &Case3, g: &Grid3, u: &[Cons3], p_inf: f64, x_max: f64) -> LiftForces {
    let w = wall_forces(g, u, p_inf, x_max);
    let (mut fx, mut fz, mut mz) = (0.0, 0.0, 0.0);
    for (c, a) in &w.by {
        if *c != Wall::Sting {
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
    let w = wall_forces(g, u, p_inf, case.body.x_base());
    let part = |classes: &dyn Fn(Wall) -> bool| {
        let mut a = [0.0; 3];
        for (c, f) in &w.by {
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
fn zero_incidence(g: &Grid3) -> bool {
    matches!(g.sym, Symmetry::Wedge | Symmetry::Sector { .. })
}

fn coefficients(case: &Case3, g: &Grid3, [fx, fz, mz]: [f64; 3]) -> LiftForces {
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
    }
}

/// Dimensional pressure-force sums `[Fx, Fz, Mz]` (`Mz = sum x Fz`) of the solved domain per wall class (not scaled to the whole body).
#[derive(Clone, Debug, Default)]
pub struct WallForces {
    pub by: Vec<(Wall, [f64; 3])>,
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

    /// The sums of `class` (zero when no face of it exists).
    pub fn get(&self, class: Wall) -> [f64; 3] {
        self.by.iter().find(|(c, _)| *c == class).map_or([0.0; 3], |(_, a)| *a)
    }
}

/// A wall face of the solved domain as [`for_each_wall_face`] hands it out.
pub struct WallFaceRef<'a> {
    /// The class of the face ([`wall_class`] of the part behind it and its direction).
    pub class: Wall,
    /// The part of the wall behind the face.
    pub part: Part,
    /// The first fluid cell (`Grid3::cell` index) and the fluid cell behind it along the wall normal, if any.
    pub a: usize,
    pub b: Option<usize>,
    pub face: &'a Face3,
    /// `+1` when the face normal points into the wall (the fluid is on the low-index side of the face), `-1` when into the fluid.
    pub sign: f64,
    /// The four nodes `(i, j, k)` of the face in the order [`Grid3`] builds its faces from.
    pub corners: [(usize, usize, usize); 4],
}

/// Hands every wall face of `g` to `visit`: the `j = 0` boundary faces inside the grid's `wall_j0` runs (not on the axis) and every face between a fluid
/// cell and a wall cell (a solid cell that has a part; dead cells are no walls), in a fixed order. The one place that decides what a wall face
/// is: the force integration ([`wall_forces`]) and the flow field of the viewer (`engine::field3`) both read the faces from here.
pub fn for_each_wall_face(g: &Grid3, mut visit: impl FnMut(&WallFaceRef)) {
    let (ni, nj, nk) = (g.ni, g.nj, g.nk);
    let solid = |i: usize, j: usize, k: usize| g.is_solid(i, j, k);
    let wall_part = |i: usize, j: usize, k: usize| g.part_of(i, j, k);
    let j0_part: Vec<Option<Part>> = (0..ni).map(|i| g.wall_j0.iter().find(|(r, _)| r.contains(&i)).map(|(_, p)| *p)).collect();
    for k in 0..nk {
        for i in 0..ni {
            let Some(part) = j0_part[i] else { continue };
            let f = &g.jface[k * (nj + 1) * ni + i];
            // normal points into the fluid: into the body is its negative
            if !solid(i, 0, k) && !g.is_axis(i, 0) {
                let (a, b) = (g.cell(i, 0, k), (nj > 1 && !solid(i, 1, k)).then(|| g.cell(i, 1, k)));
                let corners = [(i, 0, k), (i, 0, k + 1), (i + 1, 0, k + 1), (i + 1, 0, k)];
                visit(&WallFaceRef { class: wall_class(part, 1, false), part, a, b, face: f, sign: -1.0, corners });
            }
        }
    }
    for k in 0..nk {
        for j in 0..nj {
            for i in 1..ni {
                let f = &g.iface[(k * nj + j) * (ni + 1) + i];
                let corners = [(i, j, k), (i, j + 1, k), (i, j + 1, k + 1), (i, j, k + 1)];
                if let (false, Some(part)) = (solid(i - 1, j, k), wall_part(i, j, k)) {
                    let (a, b) = (g.cell(i - 1, j, k), (i >= 2 && !solid(i - 2, j, k)).then(|| g.cell(i - 2, j, k)));
                    visit(&WallFaceRef { class: wall_class(part, 0, false), part, a, b, face: f, sign: 1.0, corners });
                } else if let (Some(part), false) = (wall_part(i - 1, j, k), solid(i, j, k)) {
                    let (a, b) = (g.cell(i, j, k), (i + 1 < ni && !solid(i + 1, j, k)).then(|| g.cell(i + 1, j, k)));
                    visit(&WallFaceRef { class: wall_class(part, 0, true), part, a, b, face: f, sign: -1.0, corners });
                }
            }
        }
    }
    for k in 0..nk {
        for j in 1..nj {
            for i in 0..ni {
                let f = &g.jface[(k * (nj + 1) + j) * ni + i];
                let corners = [(i, j, k), (i, j, k + 1), (i + 1, j, k + 1), (i + 1, j, k)];
                if let (false, Some(part)) = (solid(i, j - 1, k), wall_part(i, j, k)) {
                    let (a, b) = (g.cell(i, j - 1, k), (j >= 2 && !solid(i, j - 2, k)).then(|| g.cell(i, j - 2, k)));
                    visit(&WallFaceRef { class: wall_class(part, 1, false), part, a, b, face: f, sign: 1.0, corners });
                } else if let (Some(part), false) = (wall_part(i, j - 1, k), solid(i, j, k)) {
                    let (a, b) = (g.cell(i, j, k), (j + 1 < nj && !solid(i, j + 1, k)).then(|| g.cell(i, j + 1, k)));
                    visit(&WallFaceRef { class: wall_class(part, 1, true), part, a, b, face: f, sign: -1.0, corners });
                }
            }
        }
    }
    for k in 1..nk {
        for j in 0..nj {
            for i in 0..ni {
                let f = &g.kface[(k * nj + j) * ni + i];
                let corners = [(i, j, k), (i + 1, j, k), (i + 1, j + 1, k), (i, j + 1, k)];
                if let (false, Some(part)) = (solid(i, j, k - 1), wall_part(i, j, k)) {
                    let (a, b) = (g.cell(i, j, k - 1), (k >= 2 && !solid(i, j, k - 2)).then(|| g.cell(i, j, k - 2)));
                    visit(&WallFaceRef { class: wall_class(part, 2, false), part, a, b, face: f, sign: 1.0, corners });
                } else if let (Some(part), false) = (wall_part(i, j, k - 1), solid(i, j, k)) {
                    let (a, b) = (g.cell(i, j, k), (k + 1 < nk && !solid(i, j, k + 1)).then(|| g.cell(i, j, k + 1)));
                    visit(&WallFaceRef { class: wall_class(part, 2, true), part, a, b, face: f, sign: -1.0, corners });
                }
            }
        }
    }
}

/// The wall pressure of a wall face: the pressure of its first two fluid cells extrapolated linearly to the face centre (the first cell's alone
/// when there is no second), `p` the pressure of a cell.
pub fn wall_pressure(g: &Grid3, w: &WallFaceRef, p: impl Fn(usize) -> f64) -> f64 {
    let dist = |c: usize| {
        let (x, f) = (g.xc[c], &w.face.c);
        ((x[0] - f[0]).powi(2) + (x[1] - f[1]).powi(2) + (x[2] - f[2]).powi(2)).sqrt()
    };
    let p0 = p(w.a);
    match w.b {
        Some(b) => {
            let (d0, d1) = (dist(w.a), dist(b));
            p0 + (p0 - p(b)) * d0 / (d1 - d0)
        }
        None => p0,
    }
}

/// Wall forces of the state `u` of `g` by class (see [`for_each_wall_face`] for the faces). Faces with centroid `x > x_max` are left out except
/// the base.
pub fn wall_forces(g: &Grid3, u: &[Cons3], p_inf: f64, x_max: f64) -> WallForces {
    let mut acc = WallForces::default();
    for_each_wall_face(g, |w| {
        if w.face.c[0] > x_max + 1e-9 && w.class != Wall::Base {
            return;
        }
        let pw = wall_pressure(g, w, |c| cons_to_prim3(&u[c])[4]);
        let d = (pw - p_inf) * w.face.area * w.sign;
        acc.add(w.class, [d * w.face.n[0], d * w.face.n[2], d * w.face.n[2] * w.face.c[0]]);
    });
    acc
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
    /// Nose + aft + base + fins (the sting not counted).
    pub total: f64,
}

/// [`DragParts`] of the state `u` of `g` for the case `case` (axial force only; every segment of the body of `case`).
pub fn drag_parts(case: &Case3, g: &Grid3, u: &[Cons3], p_inf: f64) -> DragParts {
    let w = wall_forces(g, u, p_inf, f64::INFINITY);
    let q = 0.5 * case.mach * case.mach;
    let norm = (2.0 * std::f64::consts::PI / g.theta_span) / (q * std::f64::consts::FRAC_PI_4);
    let cd = |c: Wall| norm * w.get(c)[0];
    let aft: Vec<f64> = (0..case.body.segments.len()).map(|s| cd(Wall::Body(s as u16 + 1))).collect();
    let (nose, base) = (cd(Wall::Body(0)), cd(Wall::Base));
    let (fin_sides, fin_edges, fin_tips) = (cd(Wall::FinSide), cd(Wall::FinLe) + cd(Wall::FinTe), cd(Wall::FinTip));
    let total = nose + aft.iter().sum::<f64>() + base + fin_sides + fin_edges + fin_tips;
    DragParts { nose, aft, base, fin_sides, fin_edges, fin_tips, total }
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
/// CN) gives `NonFinite`; a residual drop of [`DROP_ORDERS`] orders below the peak residual (from iteration 20 cold, from the start
/// warm) gives `ResidualDrop`; at a sample with `it + 1 >= max(window, the floor of [`Numerics3`])` and a range of the last
/// `window / SAMPLE_EVERY` CN samples below `rel_tol |mean|` (whatever the residual did) gives `CdStationary`; the iteration cap
/// gives `MaxIterations`.
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
    rule.finish(case, lift_forces(case, s), it)
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
}

impl StopRule3 {
    pub fn new(case: &Case3, warm: bool) -> StopRule3 {
        let n = &case.numerics;
        StopRule3 {
            skip: if warm { 0 } else { 20 },
            nwin: (n.window / SAMPLE_EVERY).max(1),
            window: n.window,
            rel_tol: n.rel_tol,
            rmax: 0.0,
            r: f64::NAN,
            hist: Vec::new(),
            reason: StopReason::MaxIterations,
            min_iters: if warm { n.warm_min_iters } else { n.stationary_min_iters },
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
        None
    }

    /// Result after `iterations` iterations with the final forces `f`.
    pub fn finish(self, case: &Case3, f: LiftForces, iterations: usize) -> PointResult3 {
        let mut reason = self.reason;
        if reason != StopReason::NonFinite && !(f.cn.is_finite() && f.ca.is_finite() && f.cn.abs() <= MONITOR_LIMIT) {
            reason = StopReason::NonFinite;
        }
        let drop_orders = if self.rmax > 0.0 && self.r.is_finite() { (self.rmax / self.r.max(1e-300)).log10() } else { 0.0 };
        PointResult3 {
            mach: case.mach,
            alpha: case.alpha,
            cn: f.cn,
            ca: f.ca,
            x_cp: f.x_cp,
            iterations,
            stop_reason: reason,
            drop_orders,
            cn_history: self.hist,
        }
    }
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
        let stop = feed(|it| if it < 100 { 1.0 } else { 1e-2 }, |_| 0.3);
        assert_eq!(stop, Some((250, StopReason::CdStationary)));
        // a CN that wiggles at round-off level without a drop is not frozen (not bit-identical): CdStationary
        let stop = feed(|_| 0.5, |it| 0.3 + 1e-9 * (it % 3) as f64);
        assert_eq!(stop, Some((250, StopReason::CdStationary)));
    }
}
