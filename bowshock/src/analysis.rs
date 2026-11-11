//! Mach-sweep analysis of a body: zero-lift drag versus Mach number, on the GPU when one is present.

use crate::body_grid::BaseBody;
use crate::case::{Case, Wake};
use crate::error::Error;
use crate::numerics::Numerics;
use crate::result::PointResult;
use crate::run::{run_cold, run_on};
use bowshock_reference::base_drag::base_drag_coefficient;

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

/// Drag coefficients referenced to the maximum cross-section.
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

/// One Mach number of a sweep.
#[derive(Clone, Debug)]
pub struct PolarPoint {
    pub mach: f64,
    /// Angle of attack [rad]; always 0.0.
    pub alpha: f64,
    pub drag: Drag,
    /// Full solver result.
    pub detail: PointResult,
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
    resolution: Resolution,
    wake: Wake,
    backend: Backend,
    numerics: Option<Numerics>,
    max_iterations: Option<usize>,
    warm_start: bool,
}

impl Analysis {
    /// Analysis of a body in internal units (nose diameter 1). Defaults: medium resolution, sting wake, automatic backend,
    /// numerics chosen per Mach number, warm starts.
    pub fn from_internal(body: BaseBody) -> Analysis {
        Analysis { body, resolution: Resolution::Medium, wake: Wake::Sting, backend: Backend::Auto, numerics: None, max_iterations: None, warm_start: true }
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

    /// Start each Mach number from the converged field of the previous one.
    pub fn warm_start(mut self, on: bool) -> Self {
        self.warm_start = on;
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
        if self.wake == Wake::Sting && self.body.boat_length > 0.0 {
            return Err(Error::Unsupported("a sting-mounted body cannot have a boattail yet".into()));
        }

        // cases in ascending Mach order (stable), results restored to input order below
        let mut order: Vec<usize> = (0..machs.len()).collect();
        order.sort_by(|&a, &b| machs[a].total_cmp(&machs[b]));
        let cases: Vec<Case> = order.iter().map(|&k| self.case(machs[k])).collect();

        let (results, backend, fallback) = match self.backend {
            Backend::Cpu => (self.sweep_cpu(&cases), BackendUsed::Cpu, None),
            Backend::Gpu => (self.sweep_gpu_required(&cases)?, BackendUsed::Gpu, None),
            Backend::Auto => self.sweep_auto(&cases),
        };

        let mut slots: Vec<Option<PolarPoint>> = (0..machs.len()).map(|_| None).collect();
        for (&k, detail) in order.iter().zip(results) {
            slots[k] = Some(self.point(machs[k], detail));
        }
        let points = slots.into_iter().map(|p| p.expect("every Mach number has a result")).collect();
        Ok(Polar { points, backend, fallback })
    }

    fn case(&self, mach: f64) -> Case {
        let level = self.resolution.level();
        let mut numerics = self.numerics.unwrap_or_else(|| Numerics::auto(mach, level, self.wake));
        if let Some(cap) = self.max_iterations {
            numerics.max_iter = numerics.max_iter.min(cap);
        }
        Case::new(mach, self.body, level, self.wake, numerics)
    }

    fn point(&self, mach: f64, detail: PointResult) -> PolarPoint {
        let forebody_pressure = detail.cd_nose + detail.cd_cyl + detail.cd_boat;
        let base = base_drag_coefficient(mach) * (self.body.base_radius() / self.body.body.radius()).powi(2);
        let drag = Drag { forebody_pressure, friction: None, base, base_modelled: true, total: forebody_pressure + base };
        PolarPoint { mach, alpha: 0.0, drag, detail }
    }

    fn sweep_cpu(&self, cases: &[Case]) -> Vec<PointResult> {
        let mut out: Vec<PointResult> = Vec::with_capacity(cases.len());
        let mut solver = None;
        for case in cases {
            let result = match (&mut solver, out.first()) {
                (Some(s), Some(first)) => run_on(case, s, true, Some(first.residual_peak)),
                _ if self.warm_start => {
                    let s = solver.insert(case.build_solver());
                    run_on(case, s, false, None)
                }
                _ => run_cold(case),
            };
            out.push(result);
        }
        out
    }

    #[cfg(feature = "gpu")]
    fn sweep_gpu(&self, cases: &[Case]) -> bowshock_gpu::Result<Vec<PointResult>> {
        use crate::gpu::GpuRunner;
        use bowshock_gpu::{Mode, Precision};
        let mut out: Vec<PointResult> = Vec::with_capacity(cases.len());
        let mut runner: Option<GpuRunner> = None;
        for case in cases {
            let result = match (&mut runner, out.first()) {
                (Some(r), Some(first)) => r.run_on(case, true, Some(first.residual_peak))?,
                _ => {
                    let mut r = GpuRunner::new(case, Mode::Guarded, Precision::Extended)?;
                    let run = r.run_on(case, false, None)?;
                    if self.warm_start {
                        runner = Some(r);
                    }
                    run
                }
            };
            out.push(result.result);
        }
        Ok(out)
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
