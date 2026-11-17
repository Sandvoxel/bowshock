//! The [`Engine::Solver3`] route of [`Analysis::run`]: every condition on the 3D solver and the base-wake grid, the one engine the product moves to
//! (`DESIGN3D.md`). Selected by [`Analysis::engine`] or `BOWSHOCK_ENGINE=3d`; [`Engine::Legacy`] stays the default until the viscous 3D solver is
//! validated, and then this file is the route (the legacy drag and lift paths of the parent module and the `engine/{run,gpu,case}.rs` 2D solver are deleted).
//!
//! | condition | case | domain |
//! |---|---|---|
//! | zero incidence, no fins | [`Case3::wedge`] | one cell of [`WEDGE_ANGLE`](crate::engine::grid3::WEDGE_ANGLE), the axisymmetric flow; `Block` line solve (the case sets it) |
//! | zero incidence, fins | [`Case3::sector`] | `[0, pi/n]` between the pitch plane and the fin's mid-plane |
//! | incidence | [`Case3::base_half`] | `[0, pi]`, fins between the pitch plane and each other |
//!
//! Results come from the class forces of the wall faces ([`drag_parts`](crate::engine::run3::drag_parts): nose, each aft segment, base, fin sides, edges
//! and tips; on the GPU reduced on the device), the wall pressures of [`wall_samples`] and the solved state (the guards of [`quality3`]). Coefficients
//! are on the nose-base area the engine works on, rescaled to the reference area in one place ([`Analysis::area_ratio`]).
//!
//! What the engine does not do yet fails loudly: a viscous request ([`Analysis::viscous`], roughness, a motor, a drag tolerance) is an error under
//! [`Engine::Solver3`], never a silent Euler run. [`Analysis::wall_flow`] and [`Analysis::tier3`] already map the viscous knobs onto the engine's
//! [`Tier`]s; [`SOLVES_VISCOUS`] is the one switch that lets them through.

use super::*;
use crate::engine::case3::{Tier, WallFlow};
use crate::engine::field3::field3_of;
use crate::engine::run3::{drag_parts, wall_samples, DragParts, Wall};
use bowshock_cfd::gas3::{cons_to_prim3, Cons3};
use bowshock_cfd::grid3::{Grid3, PART_FLUID};

/// Whether the engine solves the viscous terms (`Analysis::viscous`, roughness): flip with the stage that makes it so, and [`Analysis::tier3`] maps them.
const SOLVES_VISCOUS: bool = false;

/// The solved domain of a condition.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Kind {
    Wedge,
    Sector,
    Half,
}

/// What a backend hands back about a finished run: the class forces, the state and its grid.
struct Post3 {
    drag: DragParts,
    u: Vec<Cons3>,
}

/// One backend of the sweep: a cold run that builds the solver state, a warm run on it, and what is read from a state afterwards.
trait Runner3 {
    type State;
    fn cold(&self, case: &Case3) -> Result<(PointResult3, Self::State), Error>;
    fn warm(&self, state: &mut Self::State, case: &Case3) -> Result<PointResult3, Error>;
    fn grid<'a>(&self, state: &'a Self::State) -> &'a Grid3;
    /// The class forces and the state of `state` (the one download of a point).
    fn post(&self, state: &Self::State, case: &Case3) -> Result<Post3, Error>;
}

struct Cpu3;

impl Runner3 for Cpu3 {
    type State = Solver3;

    fn cold(&self, case: &Case3) -> Result<(PointResult3, Solver3), Error> {
        let mut s = case.build_solver_cached()?;
        let r = run3_on(case, &mut s, false);
        Ok((r, s))
    }

    fn warm(&self, s: &mut Solver3, case: &Case3) -> Result<PointResult3, Error> {
        Ok(run3_on(case, s, true))
    }

    fn grid<'a>(&self, s: &'a Solver3) -> &'a Grid3 {
        &s.grid
    }

    fn post(&self, s: &Solver3, case: &Case3) -> Result<Post3, Error> {
        Ok(Post3 { drag: drag_parts(case, &s.grid, &s.u, s.cfg.freestream[4]), u: s.u.clone() })
    }
}

#[cfg(feature = "gpu")]
struct Gpu3;

#[cfg(feature = "gpu")]
impl Runner3 for Gpu3 {
    type State = crate::engine::gpu3::GpuRunner3;

    fn cold(&self, case: &Case3) -> Result<(PointResult3, Self::State), Error> {
        // The wedge (a few thousand cells, solved to a tight stop rule at low Mach number too) runs in extended precision as the axisymmetric solver did; the half
        // domain and the sector (hundreds of thousands of cells, within 0.01 % of extended on the ANF table) in single precision.
        let precision = match case.grid.domain {
            crate::engine::grid3::Domain3::Wedge { .. } => bowshock_cfd::gpu::Precision::Extended,
            _ => bowshock_cfd::gpu::Precision::F32,
        };
        let mut r = crate::engine::gpu3::GpuRunner3::new(case, precision)?;
        let p = r.run_on(case, false)?;
        Ok((p, r))
    }

    fn warm(&self, r: &mut Self::State, case: &Case3) -> Result<PointResult3, Error> {
        r.run_on(case, true)
    }

    fn grid<'a>(&self, r: &'a Self::State) -> &'a Grid3 {
        &r.host.grid
    }

    fn post(&self, r: &Self::State, case: &Case3) -> Result<Post3, Error> {
        Ok(Post3 { drag: r.drag_parts(case)?, u: r.gpu.state_f64()? })
    }
}

/// Quality of a 3D point, first match wins: [`Quality::Failed`] on a non-finite or blown-up state (the run stopped `NonFinite`), a cell whose density or
/// pressure is not positive, a non-finite force or drag, a pressure at the outer boundaries more than 100 % off the free stream, or an area-mean base Cp
/// below the vacuum limit `-2 / (gamma M^2)`: the guards of the 2D facade's `quality`. Otherwise by how the run stopped.
#[allow(clippy::too_many_arguments)]
fn quality3(stop: StopReason, mach: f64, min_rho: f64, min_p: f64, finite: bool, farfield_dp: f64, cp_base: f64) -> Quality {
    let below_vacuum = cp_base < -2.0 / (GAMMA * mach * mach); // false for NaN
    let positive = |x: f64| x > 0.0; // false for NaN
    if matches!(stop, StopReason::NonFinite | StopReason::Cancelled)
        || !positive(min_rho)
        || !positive(min_p)
        || !finite
        || farfield_dp > FARFIELD_DP_FAILED
        || below_vacuum
    {
        return Quality::Failed;
    }
    match stop {
        StopReason::ResidualDrop => Quality::Converged,
        StopReason::CdStationary | StopReason::MeanStationary | StopReason::BasePressureStationary | StopReason::ResidualPlateau => Quality::Stationary,
        StopReason::MaxIterations => Quality::Stalled,
        StopReason::NonFinite | StopReason::Cancelled => Quality::Failed,
    }
}

impl Analysis {
    /// Which domain solves `c`.
    fn kind_of(&self, c: Condition) -> Kind {
        if c.alpha_deg != 0.0 {
            Kind::Half
        } else if self.fins.is_some() {
            Kind::Sector
        } else {
            Kind::Wedge
        }
    }

    /// Ratio of the engine's nose-base area to the reference area: the one factor that takes the engine's coefficients to the facade's.
    fn area_ratio(&self) -> f64 {
        (self.d_nose / self.d_ref).powi(2)
    }

    /// The input checks of the route.
    fn check_solver3(&self, conditions: &[Condition]) -> Result<(), Error> {
        if !SOLVES_VISCOUS && self.flight.is_some() {
            return Err(Error::Unsupported(
                "Engine::Solver3 solves the Euler equations only: Analysis::viscous is not available under it yet (it would run Euler silently)".into(),
            ));
        }
        // the jet patch of the 3D solver does not exist yet: a motor-on request fails whatever else the engine solves
        if self.motor.is_some() && self.flight.is_some() {
            return Err(Error::Unsupported("Engine::Solver3 has no motor jet yet".into()));
        }
        if self.flight.is_none() {
            if self.drag_tolerance.is_some() {
                return Err(Error::InvalidInput("drag_tolerance needs Analysis::viscous".into()));
            }
            if self.roughness_m > 0.0 {
                return Err(Error::InvalidInput("roughness needs Analysis::viscous and a dimensional BodySpec".into()));
            }
            if self.motor.is_some() {
                return Err(Error::InvalidInput("motor-on analysis needs Analysis::viscous".into()));
            }
        }
        for c in conditions {
            let (range, what) = if c.alpha_deg == 0.0 { (MACH_RANGE, "") } else { (LIFT_MACH_RANGE, " for a condition with an angle of attack") };
            if !(c.mach.is_finite() && (range.0..=range.1).contains(&c.mach)) {
                return Err(Error::InvalidInput(format!("Mach number {} is outside [{}, {}]{what}", c.mach, range.0, range.1)));
            }
            if c.alpha_deg != 0.0 && !(c.alpha_deg > 0.0 && c.alpha_deg <= LIFT_ALPHA_MAX_DEG) {
                return Err(Error::InvalidInput(format!("incidence {} deg is outside (0, {LIFT_ALPHA_MAX_DEG}]", c.alpha_deg)));
            }
        }
        if let Some(m) = self.grid_mach {
            if !(m.is_finite() && (MACH_RANGE.0..=MACH_RANGE.1).contains(&m)) {
                return Err(Error::InvalidInput(format!("grid Mach number {m} is outside [{}, {}]", MACH_RANGE.0, MACH_RANGE.1)));
            }
        }
        if !(self.d_ref.is_finite() && self.d_ref > 0.0) {
            return Err(Error::InvalidInput(format!("reference diameter {} must be finite and > 0", self.d_ref)));
        }
        if !(self.roughness_m.is_finite() && self.roughness_m >= 0.0 && self.roughness_m < 0.01 * self.d_nose) {
            return Err(Error::InvalidInput(format!(
                "roughness {} m must be finite, >= 0 and below 1% of the nose diameter ({} m)",
                self.roughness_m,
                0.01 * self.d_nose
            )));
        }
        if let Some(f) = &self.fins {
            f.validate()?;
            if !self.dimensional {
                return Err(Error::InvalidInput("fins need a dimensional BodySpec".into()));
            }
        }
        Ok(())
    }

    /// The flow the walls of a wall-resolved grid are sized for: the viscous knobs of the analysis (`viscous`, `roughness`, the fins' roughness, `grid_mach`)
    /// as the engine's [`WallFlow`]; `None` for an Euler analysis.
    fn wall_flow(&self, machs: &[f64]) -> Result<Option<WallFlow>, Error> {
        let Some(rans) = self.rans(machs)? else {
            return Ok(None);
        };
        Ok(Some(WallFlow {
            re_c: rans.re_c,
            mach_max: rans.mach_max,
            ks_body: self.roughness_m / self.d_nose,
            ks_fin: self.fins.as_ref().map_or(0.0, |f| f.roughness) / self.d_nose,
        }))
    }

    /// The engine's tier of condition `c`: Euler without wall flow; with it the lift tier at an angle of attack and, at zero incidence, `Coarse` for
    /// [`Resolution::Coarse`] and `Fine` for the finer resolutions.
    fn tier3(c: Condition, resolution: Resolution, flow: Option<WallFlow>) -> Tier {
        match flow {
            None => Tier::Euler,
            Some(f) if c.alpha_deg != 0.0 => Tier::Lift(f),
            Some(f) if resolution == Resolution::Coarse => Tier::Coarse(f),
            Some(f) => Tier::Fine(f),
        }
    }

    /// The case of condition `c` at grid `level`, with the numerics overrides, progress, snapshots and the test hook applied.
    fn solver3_case(&self, c: Condition, level: u32, tier: Tier, snapshots: bool) -> Case3 {
        let body = self.body.clone();
        let mut case = match self.kind_of(c) {
            Kind::Wedge => Case3::wedge(c.mach, body, level).with_tier(tier),
            Kind::Sector => Case3::sector(c.mach, body, self.fins3().expect("a sector has fins"), tier, level),
            Kind::Half => Case3::base_half(c.mach, c.alpha_deg.to_radians(), body, self.fins3(), tier, level),
        };
        if self.wake == Some(Wake::Sting) {
            case.grid.base = None;
        }
        if let Some(n) = &self.numerics {
            case.numerics.limiter = n.limiter;
            case.numerics.low_mach = n.low_mach;
            case.numerics.first_order_iters = n.first_order_iters;
            case.numerics.max_iter = n.max_iter;
        }
        if let Some(cap) = self.max_iterations {
            case.numerics.max_iter = case.numerics.max_iter.min(cap);
        }
        case.progress = self.progress.as_ref().map(|p| p.scaled(self.area_ratio()));
        case.snapshots = if snapshots { self.field3_sink.clone() } else { None };
        if let Some(hook) = self.case3_hook {
            hook(&mut case);
        }
        case
    }

    /// The conditions on the 3D solver (see the module docs): the points in the order of `conditions`.
    pub(super) fn run_solver3(&self, conditions: &[Condition]) -> Result<Polar, Error> {
        self.check_solver3(conditions)?;
        // zero incidence first, then by Mach number and angle (stable); results go back to the order asked for
        let mut order: Vec<usize> = (0..conditions.len()).collect();
        order.sort_by(|&a, &b| {
            let (a, b) = (conditions[a], conditions[b]);
            (a.alpha_deg != 0.0).cmp(&(b.alpha_deg != 0.0)).then(a.mach.total_cmp(&b.mach)).then(a.alpha_deg.total_cmp(&b.alpha_deg))
        });
        let sorted: Vec<Condition> = order.iter().map(|&k| conditions[k]).collect();
        let machs: Vec<f64> = sorted.iter().map(|c| c.mach).collect();
        let flow = self.wall_flow(&machs)?;
        let level = self.resolution.level();
        let (mut points, backend, mut fallback) = self.solve3_on(self.backend, &sorted, level, flow, true)?;
        if self.estimate_uncertainty {
            // the neighbouring grid level, cold: the difference of the totals is the point's uncertainty (neither point failed)
            let cmp_level = if level >= 1 { level - 1 } else { level + 1 };
            let b = match backend {
                BackendUsed::Cpu => Backend::Cpu,
                BackendUsed::Gpu => self.backend,
            };
            let (other, _, note) = self.solve3_on(b, &sorted, cmp_level, flow, false)?;
            if let Some(note) = note {
                fallback = Some(format!("uncertainty sweep: {note}"));
            }
            for (p, o) in points.iter_mut().zip(&other) {
                if p.quality != Quality::Failed && o.quality != Quality::Failed {
                    p.cd_uncertainty = Some((p.drag.total - o.drag.total).abs());
                }
            }
        }
        let mut slots: Vec<Option<PolarPoint>> = (0..conditions.len()).map(|_| None).collect();
        for (&k, p) in order.iter().zip(points) {
            slots[k] = Some(p);
        }
        let points = slots.into_iter().map(|p| p.expect("every condition has a result")).collect();
        Ok(Polar { points, backend, fallback, motor_on: false })
    }

    /// `sorted` on `backend`: the points, the backend that ran them and, under [`Backend::Auto`], why the GPU was not used.
    fn solve3_on(
        &self,
        backend: Backend,
        sorted: &[Condition],
        level: u32,
        flow: Option<WallFlow>,
        keep: bool,
    ) -> Result<(Vec<PolarPoint>, BackendUsed, Option<String>), Error> {
        let cpu = || guard(|| self.sweep3(&Cpu3, sorted, level, flow, keep));
        match backend {
            Backend::Cpu => Ok((cpu()?, BackendUsed::Cpu, None)),
            Backend::Gpu => Ok((self.sweep3_gpu(sorted, level, flow, keep)?, BackendUsed::Gpu, None)),
            Backend::Auto => match self.sweep3_gpu(sorted, level, flow, keep) {
                Ok(p) => Ok((p, BackendUsed::Gpu, None)),
                Err(Error::Cancelled) => Err(Error::Cancelled),
                Err(e) => {
                    let why = match e {
                        Error::Internal(m) => m.replacen("solver panicked", "GPU solver panicked", 1),
                        e => format!("GPU unavailable: {e}"),
                    };
                    Ok((cpu()?, BackendUsed::Cpu, Some(why)))
                }
            },
        }
    }

    #[cfg(feature = "gpu")]
    fn sweep3_gpu(&self, sorted: &[Condition], level: u32, flow: Option<WallFlow>, keep: bool) -> Result<Vec<PolarPoint>, Error> {
        guard(|| self.sweep3(&Gpu3, sorted, level, flow, keep))
    }

    #[cfg(not(feature = "gpu"))]
    fn sweep3_gpu(&self, _sorted: &[Condition], _level: u32, _flow: Option<WallFlow>, _keep: bool) -> Result<Vec<PolarPoint>, Error> {
        Err(Error::Unsupported("the `gpu` feature is not enabled".into()))
    }

    /// The sweep loop shared by the backends over `sorted`: a cold first point per domain and Mach number, warm later angles (and, with
    /// [`Analysis::warm_start`], warm later Mach numbers at zero incidence). A warm point that ends `Failed` or `Stalled` is re-run cold; a cold run that
    /// diverged is repeated once with the implicit CFL limit halved.
    fn sweep3<R: Runner3>(&self, runner: &R, sorted: &[Condition], level: u32, flow: Option<WallFlow>, keep: bool) -> Result<Vec<PolarPoint>, Error> {
        let mut points = Vec::with_capacity(sorted.len());
        let mut state: Option<R::State> = None;
        let mut at: Option<(Kind, f64)> = None;
        for &c in sorted {
            let kind = self.kind_of(c);
            let same = at.is_some_and(|(k, m)| k == kind && (m == c.mach || (self.warm_start && kind != Kind::Half)));
            if !same {
                state = None;
            }
            at = Some((kind, c.mach));
            let case = self.solver3_case(c, level, Self::tier3(c, self.resolution, flow), keep);
            if let Some(s) = &mut state {
                let r = runner.warm(s, &case)?;
                if r.stop_reason == StopReason::Cancelled {
                    return Err(Error::Cancelled);
                }
                if matches!(lift_quality(&r), Quality::Converged | Quality::Stationary) {
                    points.push(self.point3(runner, s, &case, c, &r, keep)?);
                    continue;
                }
            }
            let (mut r, mut s) = runner.cold(&case)?;
            let mut next_warm = true;
            if r.stop_reason == StopReason::NonFinite {
                let mut again = case.clone();
                again.numerics.implicit.cfl_max *= 0.5;
                (r, s) = runner.cold(&again)?;
                next_warm = false;
            }
            if r.stop_reason == StopReason::Cancelled {
                return Err(Error::Cancelled);
            }
            if lift_quality(&r) == Quality::Failed {
                next_warm = false;
            }
            points.push(self.point3(runner, &s, &case, c, &r, keep)?);
            state = next_warm.then_some(s);
        }
        Ok(points)
    }

    /// The point of the run `r` of `c` on `case`, left in `state`: [`Drag`] and [`PointDetail`] from the class forces and the wall samples, [`Forces`] at
    /// an angle of attack. What `drag` holds at an angle of attack is the axial force split by surface (its total is `forces.ca`, not a drag).
    fn point3<R: Runner3>(&self, runner: &R, state: &R::State, case: &Case3, c: Condition, r: &PointResult3, keep: bool) -> Result<PolarPoint, Error> {
        let (g, q) = (runner.grid(state), self.area_ratio());
        let post = runner.post(state, case)?;
        let p_inf = case.freestream()[4];
        let q_inf = 0.5 * c.mach * c.mach;
        let d = &post.drag;

        // the guards: the state, the outer boundaries, the base
        let (mut min_rho, mut min_p) = (f64::INFINITY, f64::INFINITY);
        for (cell, u) in post.u.iter().enumerate() {
            if g.part[cell] == PART_FLUID {
                let w = cons_to_prim3(u);
                min_rho = min_rho.min(w[0]);
                min_p = min_p.min(w[4]);
            }
        }
        let farfield_dp = farfield_dp(g, &post.u, p_inf);
        let samples = wall_samples(g, &post.u);
        let cp = |pw: f64| (pw - p_inf) / q_inf;
        let (mut base_area, mut base_cp) = (0.0, 0.0);
        for w in samples.iter().filter(|w| w.class == Wall::Base) {
            base_area += w.face.area;
            base_cp += cp(w.pw) * w.face.area;
        }
        let cp_base = if base_area > 0.0 { base_cp / base_area } else { f64::NAN };
        let has_base = case.grid.base.is_some();

        let forebody = (d.nose + d.aft.iter().sum::<f64>()) * q;
        let fin_pressure = (d.fin_sides + d.fin_edges + d.fin_tips) * q;
        let fins = self.fins.is_some().then_some(FinDrag { friction: 0.0, pressure: fin_pressure });
        let base = d.base * q;
        let total = forebody + base + fin_pressure;
        let drag = Drag { forebody_pressure: forebody, friction: None, base, base_modelled: false, fins, total };

        let forces = (c.alpha_deg != 0.0).then(|| {
            let (cn, ca, x_cp) = (r.cn * q, r.ca * q, r.x_cp * self.d_nose);
            Forces { cn, cm: if x_cp.is_finite() { -cn * x_cp / self.d_ref } else { 0.0 }, x_cp, ca }
        });
        let finite = total.is_finite() && forces.is_none_or(|f| f.cn.is_finite() && f.ca.is_finite());
        let quality = quality3(r.stop_reason, c.mach, min_rho, min_p, finite, farfield_dp, cp_base);

        let mut warnings = Vec::new();
        if (0.7..1.05).contains(&c.mach) && self.has_sharp_corner() {
            warnings.push(Warning::SharpCornerTransonic);
        }
        if farfield_dp > 0.02 {
            warnings.push(Warning::FarFieldDisturbed { dp: farfield_dp });
        }

        // the wall Cp along the body of an axisymmetric or sector domain (a half domain has a ring of faces at every station: none)
        let x_base = self.body.x_base();
        let mut wall: Vec<WallCp> = if c.alpha_deg == 0.0 {
            samples
                .iter()
                .filter(|w| matches!(w.class, Wall::Body(_)) && w.face.c[0] < x_base)
                .map(|w| WallCp { x: w.face.c[0] * self.d_nose, r: w.face.c[1].hypot(w.face.c[2]) * self.d_nose, cp: cp(w.pw) })
                .collect()
        } else {
            Vec::new()
        };
        wall.sort_by(|a, b| a.x.total_cmp(&b.x));
        let detail = PointDetail {
            iterations: r.iterations,
            residual_drop_orders: r.drop_orders,
            stop_reason: r.stop_reason,
            cd_nose: d.nose * q,
            cd_aft: d.aft.iter().map(|a| a * q).collect(),
            cd_friction: 0.0,
            cd_base_solver: if has_base { base } else { f64::NAN },
            cp_base: if has_base { cp_base } else { f64::NAN },
            wall,
            y_plus_max: f64::NAN,
            farfield_dp,
        };
        let field3 = (keep && self.keep_fields).then(|| field3_of(case, g, &post.u, r.iterations, self.field_scale()));
        Ok(PolarPoint { mach: c.mach, alpha_deg: c.alpha_deg, drag, forces, detail, quality, warnings, cd_uncertainty: None, field: None, field3 })
    }
}

/// Largest relative deviation of the pressure from the free stream over the fluid cells of the outer boundaries (inflow, outflow, far field): `|p / p_inf - 1|`.
fn farfield_dp(g: &Grid3, u: &[Cons3], p_inf: f64) -> f64 {
    let (ni, nj, nk) = (g.ni, g.nj, g.nk);
    let mut dp: f64 = 0.0;
    for k in 0..nk {
        for j in 0..nj {
            for i in 0..ni {
                if i != 0 && i != ni - 1 && j != nj - 1 {
                    continue;
                }
                let cell = (k * nj + j) * ni + i;
                if g.part[cell] == PART_FLUID {
                    dp = dp.max((cons_to_prim3(&u[cell])[4] / p_inf - 1.0).abs());
                }
            }
        }
    }
    dp
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::spec::{AftSpec, NoseShape, NoseSpec};

    #[test]
    fn the_facades_area_factor_is_the_engines_nose_to_largest_area_for_the_default_reference() {
        // a nose of 0.1 m and a tube of 0.14 m: the largest section is the tube's
        let spec = BodySpec {
            nose: NoseSpec { shape: NoseShape::TangentOgive, length: 0.3, diameter: 0.1 },
            aft: vec![AftSpec::Transition { length: 0.1, aft_diameter: 0.14 }, AftSpec::Tube { length: 0.5 }],
        };
        let a = Analysis::new(&spec).unwrap();
        assert!((a.area_ratio() - a.body.nose_to_largest_area()).abs() < 1e-12, "{} {}", a.area_ratio(), a.body.nose_to_largest_area());
        // an explicit reference diameter is the facade's own business
        let b = Analysis::new(&spec).unwrap().reference_diameter(0.1);
        assert!((b.area_ratio() - 1.0).abs() < 1e-12);
    }
}
