//! Mach-sweep analysis of a body: zero-lift drag versus Mach number, on the GPU when one is present.

use crate::body_grid::{check_quality, BaseBody};
use crate::case::{Case, Wake};
use crate::error::Error;
use crate::numerics::Numerics;
use crate::result::{PointResult, StopReason};
use crate::run::run_on;
use crate::spec::BodySpec;
use bowshock_core::solver::Solver;
use bowshock_reference::base_drag::base_drag_coefficient;
use std::convert::Infallible;

/// Lowest and highest Mach number accepted by [`Analysis::run`].
const MACH_RANGE: (f64, f64) = (0.1, 3.0);

/// Where a sweep runs.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Backend {
    /// The GPU when one is available, otherwise the CPU (reported in [`Polar::fallback`]).
    Auto,
    Cpu,
    Gpu,
}

/// Backend that produced a [`Polar`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BackendUsed {
    Cpu,
    Gpu,
}

/// Grid resolution (grid level 0, 1, 2).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Resolution {
    Coarse,
    Medium,
    Fine,
}

impl Resolution {
    fn level(self) -> u32 {
        match self {
            Resolution::Coarse => 0,
            Resolution::Medium => 1,
            Resolution::Fine => 2,
        }
    }
}

/// Drag coefficients referenced to the reference area (by default the largest cross-section; `detail` of a point is
/// referenced to the nose base area).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Drag {
    /// Pressure drag of the nose and the aft body.
    pub forebody_pressure: f64,
    /// Skin friction, if it was computed.
    pub friction: Option<f64>,
    pub base: f64,
    /// `base` comes from a correlation (the inviscid wake gives no physical base pressure).
    pub base_modelled: bool,
    pub total: f64,
}

/// Overall trust in a point, derived from how its run stopped.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Quality {
    /// The residual dropped by the requested number of orders.
    Converged,
    /// The residual did not drop enough, but the monitored quantity is stationary.
    Stationary,
    /// The iteration cap was reached without a stop criterion.
    Stalled,
    /// Non-finite or non-physical state, or non-finite drag.
    Failed,
}

/// A condition under which a point is less reliable than its [`Quality`] suggests.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Warning {
    /// Transonic Mach number and a convex profile corner sharper than 2 degrees (shock position and drag are sensitive).
    SharpCornerTransonic,
    /// The base pressure still oscillates by `cp_base_range` over the monitor window although the residual did not drop.
    UnsteadyWake { cp_base_range: f64 },
    /// The pressure at the outer boundaries deviates from the free stream by `dp` (relative): the domain may be too small.
    FarFieldDisturbed { dp: f64 },
}

/// One Mach number of a sweep.
#[derive(Clone, Debug)]
pub struct PolarPoint {
    pub mach: f64,
    /// Angle of attack [rad]; always 0.0.
    pub alpha: f64,
    pub drag: Drag,
    /// Full solver result.
    pub detail: PointResult,
    pub quality: Quality,
    pub warnings: Vec<Warning>,
    /// Absolute difference of the total drag to the run at the neighbouring grid level, if [`Analysis::estimate_uncertainty`] was on and
    /// neither point failed.
    pub cd_uncertainty: Option<f64>,
}

/// Result of a Mach sweep, in the order of the requested Mach numbers.
#[derive(Clone, Debug)]
pub struct Polar {
    pub points: Vec<PolarPoint>,
    pub backend: BackendUsed,
    /// Why the GPU was not used under [`Backend::Auto`].
    pub fallback: Option<String>,
}

/// Configuration of a sweep over Mach number for one body.
#[derive(Clone, Debug)]
pub struct Analysis {
    body: BaseBody,
    /// Nose diameter and reference diameter, in the same unit (metres, or internal units).
    d_nose: f64,
    d_ref: f64,
    resolution: Resolution,
    wake: Wake,
    backend: Backend,
    numerics: Option<Numerics>,
    max_iterations: Option<usize>,
    warm_start: bool,
    estimate_uncertainty: bool,
}

impl Analysis {
    /// Analysis of a body in internal units (nose diameter 1, so drag coefficients refer to the nose base area). Defaults:
    /// medium resolution, sting wake, automatic backend, numerics chosen per Mach number, no warm starts, no uncertainty estimate.
    pub fn from_internal(body: BaseBody) -> Analysis {
        Analysis {
            body,
            d_nose: 1.0,
            d_ref: 1.0,
            resolution: Resolution::Medium,
            wake: Wake::Sting,
            backend: Backend::Auto,
            numerics: None,
            max_iterations: None,
            warm_start: false,
            estimate_uncertainty: false,
        }
    }

    /// Analysis of a dimensional body (same defaults as [`Analysis::from_internal`]); drag coefficients refer to the largest
    /// cross-section of the body unless [`Analysis::reference_diameter`] says otherwise.
    pub fn new(spec: &BodySpec) -> Result<Analysis, Error> {
        let (body, scale) = spec.to_internal()?;
        let mut a = Analysis::from_internal(body);
        a.d_nose = scale.d_nose;
        a.d_ref = spec.reference_diameter();
        Ok(a)
    }

    /// Reference diameter [m] of the drag coefficients (internal units for [`Analysis::from_internal`]).
    pub fn reference_diameter(mut self, d_m: f64) -> Self {
        self.d_ref = d_m;
        self
    }

    pub fn resolution(mut self, r: Resolution) -> Self {
        self.resolution = r;
        self
    }

    pub fn wake(mut self, w: Wake) -> Self {
        self.wake = w;
        self
    }

    pub fn backend(mut self, b: Backend) -> Self {
        self.backend = b;
        self
    }

    /// Fixed numerics for every Mach number instead of [`Numerics::auto`].
    pub fn numerics(mut self, n: Numerics) -> Self {
        self.numerics = Some(n);
        self
    }

    /// Caps `max_iter` of whichever numerics is used.
    pub fn max_iterations(mut self, cap: usize) -> Self {
        self.max_iterations = Some(cap);
        self
    }

    /// Start each Mach number from the converged field of the previous one (default off: measured on the solver, warm-started points
    /// needed 2-5 times more iterations than cold starts at every Mach spacing from 0.05 to 0.3, and the GPU implicit scheme diverges
    /// on a warm start to Mach 0.8). A warm-started point that ends `Failed` or `Stalled` is re-run cold, and the point after a `Failed`
    /// point always starts cold.
    pub fn warm_start(mut self, on: bool) -> Self {
        self.warm_start = on;
        self
    }

    /// Also run the sweep one grid level away (`level - 1`, or `level + 1` on the coarsest level) and report the difference of the
    /// total drag in [`PolarPoint::cd_uncertainty`]. Costs a second sweep.
    pub fn estimate_uncertainty(mut self, on: bool) -> Self {
        self.estimate_uncertainty = on;
        self
    }

    /// Zero-lift drag at each Mach number of `machs` (each in `[0.1, 3.0]`).
    pub fn run(&self, machs: &[f64]) -> Result<Polar, Error> {
        if machs.is_empty() {
            return Err(Error::InvalidInput("no Mach numbers requested".into()));
        }
        for &m in machs {
            if !(m.is_finite() && (MACH_RANGE.0..=MACH_RANGE.1).contains(&m)) {
                return Err(Error::InvalidInput(format!("Mach number {m} is outside [{}, {}]", MACH_RANGE.0, MACH_RANGE.1)));
            }
        }
        if !(self.d_ref.is_finite() && self.d_ref > 0.0) {
            return Err(Error::InvalidInput(format!("reference diameter {} must be finite and > 0", self.d_ref)));
        }

        // cases in ascending Mach order (stable), results restored to input order below
        let mut order: Vec<usize> = (0..machs.len()).collect();
        order.sort_by(|&a, &b| machs[a].total_cmp(&machs[b]));
        let level = self.resolution.level();
        let cases: Vec<Case> = order.iter().map(|&k| self.case(machs[k], level)).collect();
        // the grids do not depend on the Mach number: check the first one
        check_grid(&cases[0])?;

        let cmp_cases: Option<Vec<Case>> = self.estimate_uncertainty.then(|| {
            let cmp_level = if level >= 1 { level - 1 } else { level + 1 };
            order.iter().map(|&k| self.case(machs[k], cmp_level)).collect()
        });
        if let Some(c) = &cmp_cases {
            check_grid(&c[0])?;
        }

        let (results, backend, mut fallback) = self.sweep_on(&cases, self.backend)?;
        let cmp_results = match &cmp_cases {
            Some(c) => {
                let b = match backend {
                    BackendUsed::Cpu => Backend::Cpu,
                    BackendUsed::Gpu => self.backend,
                };
                let (r, _, note) = self.sweep_on(c, b)?;
                if let Some(note) = note {
                    fallback = Some(format!("uncertainty sweep: {note}"));
                }
                Some(r)
            }
            None => None,
        };

        let mut cmp_iter = cmp_results.map(Vec::into_iter);
        let mut slots: Vec<Option<PolarPoint>> = (0..machs.len()).map(|_| None).collect();
        for (&k, detail) in order.iter().zip(results) {
            let other = cmp_iter.as_mut().and_then(Iterator::next);
            slots[k] = Some(self.point(machs[k], detail, other.as_ref()));
        }
        let points = slots.into_iter().map(|p| p.expect("every Mach number has a result")).collect();
        Ok(Polar { points, backend, fallback })
    }

    fn case(&self, mach: f64, level: u32) -> Case {
        let mut numerics = self.numerics.unwrap_or_else(|| Numerics::auto(mach, level, self.wake));
        if let Some(cap) = self.max_iterations {
            numerics.max_iter = numerics.max_iter.min(cap);
        }
        Case::new(mach, self.body.clone(), level, self.wake, numerics)
    }

    /// Drag of one solver result, referenced to the reference area.
    fn drag(&self, mach: f64, detail: &PointResult) -> Drag {
        let q = (self.d_nose / self.d_ref).powi(2);
        let forebody_pressure = detail.cd_aft.iter().fold(detail.cd_nose, |a, c| a + c);
        let base = base_drag_coefficient(mach) * (self.body.base_radius() / self.body.body.radius()).powi(2);
        let total = forebody_pressure + base;
        Drag { forebody_pressure: forebody_pressure * q, friction: None, base: base * q, base_modelled: true, total: total * q }
    }

    fn point(&self, mach: f64, detail: PointResult, other: Option<&PointResult>) -> PolarPoint {
        let drag = self.drag(mach, &detail);
        let quality = quality(&detail, &drag);
        let warnings = self.warnings(mach, &detail);
        let cd_uncertainty = other.and_then(|o| {
            let od = self.drag(mach, o);
            (quality != Quality::Failed && self::quality(o, &od) != Quality::Failed).then(|| (drag.total - od.total).abs())
        });
        PolarPoint { mach, alpha: 0.0, drag, quality, warnings, cd_uncertainty, detail }
    }

    fn warnings(&self, mach: f64, detail: &PointResult) -> Vec<Warning> {
        let mut w = Vec::new();
        if (0.7..1.05).contains(&mach) && self.has_sharp_corner() {
            w.push(Warning::SharpCornerTransonic);
        }
        if self.wake == Wake::Base && detail.cp_base_range.is_finite() && detail.cp_base_range >= 1e-3 && detail.stop_reason != StopReason::ResidualDrop {
            w.push(Warning::UnsteadyWake { cp_base_range: detail.cp_base_range });
        }
        if detail.farfield_dp > 0.02 {
            w.push(Warning::FarFieldDisturbed { dp: detail.farfield_dp });
        }
        w
    }

    /// A convex corner sharper than 2 degrees at the nose end or between two aft segments.
    fn has_sharp_corner(&self) -> bool {
        let limit = 2f64.to_radians();
        let segs = &self.body.segments;
        let Some(first) = segs.first() else { return false };
        let before = self.body.slope(self.body.body.nose_length() * (1.0 - 1e-9));
        before.atan() - first.slope.atan() > limit || segs.windows(2).any(|w| w[0].slope.atan() - w[1].slope.atan() > limit)
    }

    /// Sweeps `cases` (ascending Mach) on `backend`; returns the backend used and, under `Auto`, why the GPU was not.
    fn sweep_on(&self, cases: &[Case], backend: Backend) -> Result<(Vec<PointResult>, BackendUsed, Option<String>), Error> {
        match backend {
            Backend::Cpu => Ok((self.sweep_cpu(cases), BackendUsed::Cpu, None)),
            Backend::Gpu => Ok((self.sweep_gpu_required(cases)?, BackendUsed::Gpu, None)),
            Backend::Auto => Ok(self.sweep_auto(cases)),
        }
    }

    /// The sweep logic shared by the backends: first point cold; later points warm (when enabled) against the residual peak
    /// of the most recent cold start; a warm point that ends `Failed` or `Stalled` is re-run cold on a fresh solver, and a point
    /// after a `Failed` point starts cold on a fresh solver.
    fn sweep<R: Runner>(&self, runner: &R, cases: &[Case]) -> Result<Vec<PointResult>, R::Error> {
        let quality_of = |case: &Case, d: &PointResult| quality(d, &self.drag(case.mach, d));
        let mut out: Vec<PointResult> = Vec::with_capacity(cases.len());
        let mut state: Option<R::State> = None;
        let mut r_ref = 0.0;
        let mut previous_failed = false;
        for case in cases {
            let warm = match (&mut state, previous_failed) {
                (Some(s), false) => {
                    let r = runner.warm(s, case, r_ref)?;
                    matches!(quality_of(case, &r), Quality::Converged | Quality::Stationary).then_some(r)
                }
                _ => None,
            };
            let result = match warm {
                Some(r) => r,
                None => {
                    let (r, s) = runner.cold(case)?;
                    r_ref = r.residual_peak;
                    state = self.warm_start.then_some(s);
                    r
                }
            };
            previous_failed = quality_of(case, &result) == Quality::Failed;
            out.push(result);
        }
        Ok(out)
    }

    fn sweep_cpu(&self, cases: &[Case]) -> Vec<PointResult> {
        self.sweep(&CpuBackend, cases).unwrap_or_else(|e| match e {})
    }

    #[cfg(feature = "gpu")]
    fn sweep_gpu(&self, cases: &[Case]) -> bowshock_gpu::Result<Vec<PointResult>> {
        self.sweep(&GpuBackend, cases)
    }

    #[cfg(feature = "gpu")]
    fn sweep_gpu_required(&self, cases: &[Case]) -> Result<Vec<PointResult>, Error> {
        self.sweep_gpu(cases).map_err(|e| Error::Gpu(e.to_string()))
    }

    #[cfg(not(feature = "gpu"))]
    fn sweep_gpu_required(&self, _cases: &[Case]) -> Result<Vec<PointResult>, Error> {
        Err(Error::Unsupported("the `gpu` feature is not enabled".into()))
    }

    #[cfg(feature = "gpu")]
    fn sweep_auto(&self, cases: &[Case]) -> (Vec<PointResult>, BackendUsed, Option<String>) {
        match self.sweep_gpu(cases) {
            Ok(results) => (results, BackendUsed::Gpu, None),
            Err(e) => (self.sweep_cpu(cases), BackendUsed::Cpu, Some(format!("GPU unavailable: {e}"))),
        }
    }

    #[cfg(not(feature = "gpu"))]
    fn sweep_auto(&self, cases: &[Case]) -> (Vec<PointResult>, BackendUsed, Option<String>) {
        (self.sweep_cpu(cases), BackendUsed::Cpu, None)
    }
}

/// Builds the grids of `case` and checks their quality.
fn check_grid(case: &Case) -> Result<(), Error> {
    let probe = case.try_build_solver()?;
    check_quality(&probe.grid, probe.block())
}

/// Quality of a solver result, first match wins.
fn quality(d: &PointResult, drag: &Drag) -> Quality {
    let positive = |x: f64| x > 0.0; // false for NaN
    if d.stop_reason == StopReason::NonFinite || !positive(d.min_rho) || !positive(d.min_p) || !drag.total.is_finite() {
        return Quality::Failed;
    }
    match d.stop_reason {
        StopReason::ResidualDrop => Quality::Converged,
        StopReason::CdStationary | StopReason::BasePressureStationary | StopReason::ResidualPlateau => Quality::Stationary,
        StopReason::MaxIterations => Quality::Stalled,
        StopReason::NonFinite => Quality::Failed,
    }
}

/// A solver backend of the sweep: cold runs on a fresh solver, warm runs on the solver kept from an earlier point.
trait Runner {
    type State;
    type Error;
    fn cold(&self, case: &Case) -> Result<(PointResult, Self::State), Self::Error>;
    fn warm(&self, state: &mut Self::State, case: &Case, r_ref: f64) -> Result<PointResult, Self::Error>;
}

struct CpuBackend;

impl Runner for CpuBackend {
    type State = Solver;
    type Error = Infallible;

    fn cold(&self, case: &Case) -> Result<(PointResult, Solver), Infallible> {
        let mut s = case.build_solver();
        let r = run_on(case, &mut s, false, None);
        Ok((r, s))
    }

    fn warm(&self, s: &mut Solver, case: &Case, r_ref: f64) -> Result<PointResult, Infallible> {
        Ok(run_on(case, s, true, Some(r_ref)))
    }
}

#[cfg(feature = "gpu")]
struct GpuBackend;

#[cfg(feature = "gpu")]
impl Runner for GpuBackend {
    type State = crate::gpu::GpuRunner;
    type Error = bowshock_gpu::GpuError;

    fn cold(&self, case: &Case) -> bowshock_gpu::Result<(PointResult, crate::gpu::GpuRunner)> {
        let mut r = crate::gpu::GpuRunner::new(case, bowshock_gpu::Mode::Guarded, bowshock_gpu::Precision::Extended)?;
        let run = r.run_on(case, false, None)?;
        Ok((run.result, r))
    }

    fn warm(&self, r: &mut crate::gpu::GpuRunner, case: &Case, r_ref: f64) -> bowshock_gpu::Result<PointResult> {
        Ok(r.run_on(case, true, Some(r_ref))?.result)
    }
}
