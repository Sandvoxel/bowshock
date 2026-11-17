//! Analysis of a body over flight conditions: one entry, [`Analysis::run`], takes Mach numbers at zero incidence (the viscous drag
//! solver) and Mach number and angle of attack pairs (the 3D solver, which gives the normal force and centre of pressure).

use crate::atmosphere::{FlightConditions, SUTHERLAND_S};
use crate::engine::body_grid::{check_quality, BaseBody, MAX_STRETCHING, MAX_STRETCHING_VISCOUS_J};
use crate::engine::case::{Case, Jet, Wake};
use crate::engine::case3::Case3;
use crate::engine::field3::{field3_of_solver, Snapshots3};
use crate::engine::grid3::Fins3;
use crate::engine::numerics::Numerics;
use crate::engine::result::{PointResult, StopReason};
use crate::engine::run::run_on;
use crate::engine::run3::{run3_on, PointResult3};
use crate::error::Error;
use crate::field::{Field, FieldSink, FieldTap};
use crate::field3::Field3;
use crate::progress::{Progress, ProgressSink};
use crate::spec::{BodySpec, FinSpec, MotorSpec};
use bowshock_cfd::gas::GAMMA;
use bowshock_cfd::solver::{Solver, Turbulence, Viscous};
use bowshock_cfd::solver3::Solver3;
use bowshock_reference::base_drag::base_drag_coefficient;
use std::ops::ControlFlow;

/// Lowest and highest Mach number of a zero-incidence condition.
const MACH_RANGE: (f64, f64) = (0.1, 3.0);
/// Mach range and largest angle of attack \[deg\] of a condition with an angle of attack.
const LIFT_MACH_RANGE: (f64, f64) = (0.3, 3.0);
const LIFT_ALPHA_MAX_DEG: f64 = 15.0;

/// Where a sweep runs.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Backend {
    /// The GPU when one is available, otherwise the CPU (reported in [`Polar::fallback`]). A software Vulkan device (llvmpipe) counts as no GPU.
    Auto,
    /// The CPU solver.
    Cpu,
    /// The GPU solver; an error if none is available.
    Gpu,
}

/// Backend that produced a [`Polar`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum BackendUsed {
    /// The CPU solver.
    Cpu,
    /// The GPU solver.
    Gpu,
}

/// Grid resolution (grid level 0, 1, 2).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Resolution {
    /// Grid level 0.
    Coarse,
    /// Grid level 1.
    Medium,
    /// Grid level 2.
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

/// Drag of the fin set, both parts on the reference area. Fin-body interference drag is neglected.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct FinDrag {
    /// Skin friction of the fins.
    pub friction: f64,
    /// Pressure drag of the fins: leading and trailing edges, plus Ackeret wave drag of the sharp (double-wedge, hexagonal, biconvex) and custom sections.
    pub pressure: f64,
}

/// Drag coefficients referenced to the reference area (by default the largest cross-section).
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Drag {
    /// Pressure drag of the nose and the aft body.
    pub forebody_pressure: f64,
    /// Skin friction, if it was computed.
    pub friction: Option<f64>,
    /// Base drag: from a correlation when `base_modelled`, otherwise from the solver. With a motor it is the base drag of the annulus
    /// around the nozzle exit only; the jet's thrust is not a drag term. NaN at an angle of attack.
    #[cfg_attr(feature = "serde", serde(deserialize_with = "nan_or_f64"))]
    pub base: f64,
    /// `base` comes from a correlation (the inviscid wake gives no physical base pressure).
    pub base_modelled: bool,
    /// Fin drag; `Some` when fins were given.
    pub fins: Option<FinDrag>,
    /// Total drag: forebody pressure + friction + base + fins (non-finite only for a [`Quality::Failed`] point and at an angle of attack).
    #[cfg_attr(feature = "serde", serde(deserialize_with = "nan_or_f64"))]
    pub total: f64,
}

/// Overall trust in a point, derived from how its run stopped.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Quality {
    /// The residual dropped by the requested number of orders.
    Converged,
    /// The residual did not drop enough, but the monitored quantity is stationary.
    Stationary,
    /// The iteration cap was reached without a stop criterion.
    Stalled,
    /// Non-finite or non-physical state (including a pressure at the outer boundaries more than 100 % off the free stream), or
    /// non-finite drag.
    Failed,
}

/// A condition under which a point is less reliable than its [`Quality`] suggests.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Warning {
    /// Transonic Mach number and a convex profile corner sharper than 2 degrees (shock position and drag are sensitive).
    SharpCornerTransonic,
    /// The base pressure still oscillates by `cp_base_range` over the monitor window although the residual did not drop.
    UnsteadyWake {
        /// Range of the area-mean base Cp over the monitor window.
        cp_base_range: f64,
    },
    /// The pressure at the outer boundaries deviates from the free stream by `dp` (relative): the domain may be too small.
    FarFieldDisturbed {
        /// Relative pressure deviation.
        dp: f64,
    },
    /// The first cell at the wall is too thick for a wall-resolved boundary layer: `y+` of the first cell reaches `y_plus_max`.
    WallResolution {
        /// Largest `y+` of the first wall cell.
        y_plus_max: f64,
    },
    /// The total drag oscillated by `cd_range` (on the reference area) over the last two averaging windows, a bounded limit cycle
    /// with a stationary mean; the point is reported at the crossing of that mean.
    OscillatingDrag {
        /// Max - min of the total drag over the averaging windows.
        cd_range: f64,
    },
}

/// One flight condition of a run: a Mach number and an angle of attack.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Condition {
    /// Mach number.
    pub mach: f64,
    /// Angle of attack \[deg\]; 0 for the zero-lift drag.
    pub alpha_deg: f64,
}

impl Condition {
    /// The condition at Mach number `mach` and angle of attack `alpha_deg` (degrees).
    pub const fn new(mach: f64, alpha_deg: f64) -> Condition {
        Condition { mach, alpha_deg }
    }
}

impl From<f64> for Condition {
    /// A Mach number at zero incidence.
    fn from(mach: f64) -> Condition {
        Condition { mach, alpha_deg: 0.0 }
    }
}

/// Force coefficients of a point at an angle of attack, on the reference area; lengths in metres. Pressure forces only: the 3D solver
/// integrates the Euler equations.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Forces {
    /// Normal force coefficient, positive for positive incidence.
    pub cn: f64,
    /// Pitching-moment coefficient about the nose tip, `-cn x_cp / d_ref` (a normal force behind the tip pitches the nose down, so a
    /// positive CN gives a negative moment), on the reference area and the reference diameter; 0 when `x_cp` is NaN.
    pub cm: f64,
    /// Centre of pressure of the normal force, metres from the nose tip (NaN when the normal force vanishes).
    #[cfg_attr(feature = "serde", serde(deserialize_with = "nan_or_f64"))]
    pub x_cp: f64,
    /// Axial force coefficient of the surface pressure (no friction, no base), positive downstream.
    pub ca: f64,
}

/// One condition of a run (see [`Analysis::run`]): a Mach number at zero incidence, or at an angle of attack.
///
/// At zero incidence the point is the drag solver's: [`PolarPoint::drag`] is complete and `forces` is `None`. At an angle of attack
/// it is the 3D solver's: `forces` is `Some`, [`Drag::forebody_pressure`] is the pressure axial force of every wall face (body and
/// fins; no base, no friction), [`Drag::friction`] and [`Drag::fins`] are `None`, [`Drag::base`] and [`Drag::total`] are NaN; the
/// detail has the iterations, the residual drop and the stop reason, `cd_nose`, `cd_base_solver`, `cp_base` and `y_plus_max` are NaN,
/// `cd_friction` is 0, `cd_aft` and `wall` are empty, `farfield_dp` is 0 (not measured); there are no warnings and no uncertainty.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct PolarPoint {
    /// Mach number of the point.
    pub mach: f64,
    /// Angle of attack \[deg\].
    pub alpha_deg: f64,
    /// Drag coefficients.
    pub drag: Drag,
    /// Normal force, moment, centre of pressure and axial pressure force: `Some` at an angle of attack.
    #[cfg_attr(feature = "serde", serde(default, skip_serializing_if = "Option::is_none"))]
    pub forces: Option<Forces>,
    /// Solver summary of the point.
    pub detail: PointDetail,
    /// Trust in the point.
    pub quality: Quality,
    /// Conditions that make the point less reliable.
    pub warnings: Vec<Warning>,
    /// Absolute difference of the total drag to the run at the neighbouring grid level, if [`Analysis::estimate_uncertainty`] was on and
    /// neither point failed.
    pub cd_uncertainty: Option<f64>,
    /// The final flow field of a zero-incidence point, on the grid its drag was computed on; `Some` only with [`Analysis::keep_fields`].
    #[cfg_attr(feature = "serde", serde(default, skip_serializing_if = "Option::is_none"))]
    pub field: Option<Field>,
    /// The final flow field of a point at an angle of attack (lengths in metres); `Some` only with [`Analysis::keep_fields`].
    #[cfg_attr(feature = "serde", serde(default, skip_serializing_if = "Option::is_none"))]
    pub field3: Option<Field3>,
}

/// Reads an `f64` that a format without NaN (JSON) wrote as `null` back as NaN.
#[cfg(feature = "serde")]
fn nan_or_f64<'de, D: serde::Deserializer<'de>>(d: D) -> Result<f64, D::Error> {
    use serde::Deserialize;
    Ok(Option::<f64>::deserialize(d)?.unwrap_or(f64::NAN))
}

/// Solver summary of one point; lengths in metres (internal units for `from_internal`), coefficients referenced to the reference area.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct PointDetail {
    /// Iterations run.
    pub iterations: usize,
    /// Orders of magnitude the residual dropped from its peak to the last iteration.
    pub residual_drop_orders: f64,
    /// Why the iteration stopped.
    pub stop_reason: StopReason,
    /// Pressure drag of the nose; NaN at an angle of attack.
    #[cfg_attr(feature = "serde", serde(deserialize_with = "nan_or_f64"))]
    pub cd_nose: f64,
    /// Pressure drag of each aft section, one per [`AftSpec`](crate::AftSpec).
    pub cd_aft: Vec<f64>,
    /// Skin-friction drag; 0 for the Euler equations.
    pub cd_friction: f64,
    /// Base drag computed by the solver; NaN without a base wake.
    #[cfg_attr(feature = "serde", serde(deserialize_with = "nan_or_f64"))]
    pub cd_base_solver: f64,
    /// Area-mean base pressure coefficient (over the annulus around the nozzle exit with a motor); NaN without a base wake.
    #[cfg_attr(feature = "serde", serde(deserialize_with = "nan_or_f64"))]
    pub cp_base: f64,
    /// Body-surface faces in axial order (no sting extension).
    pub wall: Vec<WallCp>,
    /// Largest `y+` of the first wall cell; NaN for the Euler equations.
    #[cfg_attr(feature = "serde", serde(deserialize_with = "nan_or_f64"))]
    pub y_plus_max: f64,
    /// Largest relative pressure deviation from the free stream at the outer boundaries.
    pub farfield_dp: f64,
}

/// Pressure coefficient at one body-surface face.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct WallCp {
    /// Axial position of the face centre.
    pub x: f64,
    /// Radius of the face centre.
    pub r: f64,
    /// Pressure coefficient.
    pub cp: f64,
}

/// Result of [`Analysis::run`], in the order of the requested conditions.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Polar {
    /// One point per requested condition.
    pub points: Vec<PolarPoint>,
    /// Backend that ran the sweep.
    pub backend: BackendUsed,
    /// Why the GPU was not used under [`Backend::Auto`].
    pub fallback: Option<String>,
    /// The analysis had a motor: the base flow contains the jet.
    pub motor_on: bool,
}

/// The serialisable configuration of an [`Analysis`] ([`Analysis::spec`], [`Analysis::from_spec`]): the dimensional body and every
/// builder setting except the progress callback. It is what a remote worker needs to rebuild the analysis.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct AnalysisSpec {
    /// The body of [`Analysis::new`].
    pub body: BodySpec,
    /// Reference diameter \[m\] of the coefficients ([`Analysis::reference_diameter`]).
    pub reference_diameter: f64,
    /// Equivalent sand-grain roughness \[m\] ([`Analysis::roughness`]).
    pub roughness_m: f64,
    /// Grid resolution ([`Analysis::resolution`]).
    pub resolution: Resolution,
    /// Wake treatment, `None` for the default ([`Analysis::wake`]).
    pub wake: Option<Wake>,
    /// Flight conditions of a viscous analysis ([`Analysis::viscous`]).
    pub flight: Option<FlightConditions>,
    /// Fins ([`Analysis::fins`]).
    pub fins: Option<FinSpec>,
    /// Motor ([`Analysis::motor`]).
    pub motor: Option<MotorSpec>,
    /// Backend ([`Analysis::backend`]).
    pub backend: Backend,
    /// Iteration cap ([`Analysis::max_iterations`]).
    pub max_iterations: Option<usize>,
    /// Relative drag tolerance ([`Analysis::drag_tolerance`]).
    pub drag_tolerance: Option<f64>,
    /// Warm starts ([`Analysis::warm_start`]).
    pub warm_start: bool,
    /// Grid-level uncertainty estimate ([`Analysis::estimate_uncertainty`]).
    pub estimate_uncertainty: bool,
    /// Keep the final flow field of every point ([`Analysis::keep_fields`]).
    #[cfg_attr(feature = "serde", serde(default))]
    pub keep_fields: bool,
    /// Mach number a viscous grid is sized for ([`Analysis::grid_mach`]), `None` for the highest Mach number of each run.
    #[cfg_attr(feature = "serde", serde(default))]
    pub grid_mach: Option<f64>,
}

/// Configuration of a sweep over Mach number for one body.
#[derive(Clone, Debug)]
pub struct Analysis {
    body: BaseBody,
    /// The dimensional body description of [`Analysis::new`] (`None` for [`Analysis::from_internal`]).
    body_spec: Option<BodySpec>,
    /// Nose diameter and reference diameter, in the same unit (metres, or internal units).
    d_nose: f64,
    d_ref: f64,
    /// Built from a dimensional [`BodySpec`] (`d_nose` in metres).
    dimensional: bool,
    /// Equivalent sand-grain roughness of the body surface \[m\].
    roughness_m: f64,
    resolution: Resolution,
    wake: Option<Wake>,
    flight: Option<FlightConditions>,
    fins: Option<FinSpec>,
    motor: Option<MotorSpec>,
    backend: Backend,
    numerics: Option<Numerics>,
    max_iterations: Option<usize>,
    drag_tolerance: Option<f64>,
    warm_start: bool,
    estimate_uncertainty: bool,
    grid_mach: Option<f64>,
    case3_hook: Option<fn(&mut Case3)>,
    progress: Option<ProgressSink>,
    keep_fields: bool,
    field_sink: Option<FieldSink>,
    field3_sink: Option<Snapshots3>,
}

/// Free stream of a viscous sweep.
struct Rans {
    re_c: f64,
    sutherland: f64,
    mach_max: f64,
    jet: Option<Jet>,
}

impl Analysis {
    /// Analysis of a body in internal units (nose diameter 1, so drag coefficients refer to the nose base area). Defaults:
    /// medium resolution, sting wake, automatic backend, numerics chosen per Mach number, no warm starts, no uncertainty estimate.
    #[doc(hidden)]
    pub fn from_internal(body: BaseBody) -> Analysis {
        Analysis {
            body,
            body_spec: None,
            d_nose: 1.0,
            d_ref: 1.0,
            dimensional: false,
            roughness_m: 0.0,
            resolution: Resolution::Medium,
            wake: None,
            flight: None,
            fins: None,
            motor: None,
            backend: Backend::Auto,
            numerics: None,
            max_iterations: None,
            drag_tolerance: None,
            warm_start: false,
            estimate_uncertainty: false,
            grid_mach: None,
            case3_hook: None,
            progress: None,
            keep_fields: false,
            field_sink: None,
            field3_sink: None,
        }
    }

    /// Analysis of a dimensional body (same defaults as [`Analysis::from_internal`]); drag coefficients refer to the largest
    /// cross-section of the body unless [`Analysis::reference_diameter`] says otherwise.
    pub fn new(spec: &BodySpec) -> Result<Analysis, Error> {
        let (body, scale) = spec.to_internal()?;
        let mut a = Analysis::from_internal(body);
        a.d_nose = scale.d_nose;
        a.d_ref = spec.reference_diameter();
        a.dimensional = true;
        a.body_spec = Some(spec.clone());
        Ok(a)
    }

    /// The serialisable description of this analysis: everything but the progress callback (see [`AnalysisSpec`]).
    /// [`Analysis::from_spec`] rebuilds an analysis that gives the same results, for instance in another process.
    ///
    /// Fails for an analysis from [`Analysis::from_internal`] and for one with a fixed numerics or a 3D case hook (test hooks that
    /// have no serialisable form).
    pub fn spec(&self) -> Result<AnalysisSpec, Error> {
        let Some(body) = &self.body_spec else {
            return Err(Error::Unsupported("only an analysis built from a dimensional BodySpec has a spec".into()));
        };
        if self.numerics.is_some() || self.case3_hook.is_some() {
            return Err(Error::Unsupported("an analysis with fixed numerics or a 3D case hook has no spec".into()));
        }
        Ok(AnalysisSpec {
            body: body.clone(),
            reference_diameter: self.d_ref,
            roughness_m: self.roughness_m,
            resolution: self.resolution,
            wake: self.wake,
            flight: self.flight,
            fins: self.fins.clone(),
            motor: self.motor,
            backend: self.backend,
            max_iterations: self.max_iterations,
            drag_tolerance: self.drag_tolerance,
            warm_start: self.warm_start,
            estimate_uncertainty: self.estimate_uncertainty,
            keep_fields: self.keep_fields,
            grid_mach: self.grid_mach,
        })
    }

    /// An analysis from its description (without a progress callback: add one with [`Analysis::on_progress`]).
    pub fn from_spec(spec: &AnalysisSpec) -> Result<Analysis, Error> {
        let mut a = Analysis::new(&spec.body)?;
        a.d_ref = spec.reference_diameter;
        a.roughness_m = spec.roughness_m;
        a.resolution = spec.resolution;
        a.wake = spec.wake;
        a.flight = spec.flight;
        a.fins = spec.fins.clone();
        a.motor = spec.motor;
        a.backend = spec.backend;
        a.max_iterations = spec.max_iterations;
        a.drag_tolerance = spec.drag_tolerance;
        a.warm_start = spec.warm_start;
        a.estimate_uncertainty = spec.estimate_uncertainty;
        a.keep_fields = spec.keep_fields;
        a.grid_mach = spec.grid_mach;
        Ok(a)
    }

    /// Reference diameter \[m\] of the drag coefficients (internal units for [`Analysis::from_internal`]).
    pub fn reference_diameter(mut self, d_m: f64) -> Self {
        self.d_ref = d_m;
        self
    }

    /// Grid resolution (default [`Resolution::Medium`]).
    pub fn resolution(mut self, r: Resolution) -> Self {
        self.resolution = r;
        self
    }

    /// Mach number a viscous analysis sizes its grid for: the first cell height at the wall (and with it every cell count) follows
    /// it instead of the highest Mach number of the sweep. Runs of different sweeps with the same `grid_mach` share one grid shape,
    /// so a table solved point by point (one [`Analysis::run`] per Mach number) builds, and compiles the GPU kernels of, one grid
    /// instead of one per point. Mach numbers above it get a first cell taller than `y+ = 1`. Default: the highest Mach number of
    /// each [`Analysis::run`]. No effect on the Euler equations. Must lie in `[0.1, 3.0]`.
    pub fn grid_mach(mut self, mach: f64) -> Self {
        self.grid_mach = Some(mach);
        self
    }

    /// Wake treatment (default: sting for the Euler equations, base wake for a viscous analysis).
    pub fn wake(mut self, w: Wake) -> Self {
        self.wake = Some(w);
        self
    }

    /// Solve the Reynolds-averaged Navier-Stokes equations (k-omega SST, wall-resolved grid) at the free stream given by `flight`
    /// instead of the Euler equations: [`Drag::friction`] is reported and, with a base wake, the base drag comes from the solution.
    /// Needs an analysis built with [`Analysis::new`].
    pub fn viscous(mut self, flight: FlightConditions) -> Self {
        self.flight = Some(flight);
        self
    }

    /// Backend to run on (default [`Backend::Auto`]).
    pub fn backend(mut self, b: Backend) -> Self {
        self.backend = b;
        self
    }

    /// Fixed numerics for every Mach number instead of [`Numerics::auto`].
    #[doc(hidden)]
    pub fn numerics(mut self, n: Numerics) -> Self {
        self.numerics = Some(n);
        self
    }

    /// Edits every 3D case after it is built (tests shrink the grid with it). The type of the hook is the engine's own case: it changes
    /// with the engine.
    #[doc(hidden)]
    pub fn case3_hook(mut self, hook: fn(&mut Case3)) -> Self {
        self.case3_hook = Some(hook);
        self
    }

    /// Caps `max_iter` of whichever numerics is used (the 3D iteration cap of a condition with an angle of attack too).
    pub fn max_iterations(mut self, cap: usize) -> Self {
        self.max_iterations = Some(cap);
        self
    }

    /// Calls `f` with the state of every run about once per [`SAMPLE_INTERVAL`](crate::progress::SAMPLE_INTERVAL) iterations, from the
    /// thread that runs the analysis (see [`crate::progress`]). Returning [`ControlFlow::Break`] ends the analysis at once with
    /// [`Error::Cancelled`]. Reporting does not change the iteration or the result of a run that is not cancelled. The monitor of a
    /// viscous drag run and of a 3D run is reported as a coefficient on the reference area.
    pub fn on_progress(mut self, f: impl Fn(&Progress) -> ControlFlow<()> + Send + Sync + 'static) -> Self {
        self.progress = Some(ProgressSink::new(f));
        self
    }

    /// Keep the final flow field of every point in [`PolarPoint::field`] (zero incidence) or [`PolarPoint::field3`] (angle of attack)
    /// (default off: a field is far larger than the rest of a point, see [`crate::field`]). The field is the state the reported
    /// coefficients were integrated from, on the grid of the requested resolution. It does not change the iteration or the result.
    pub fn keep_fields(mut self, on: bool) -> Self {
        self.keep_fields = on;
        self
    }

    /// Calls `f` with a snapshot of the flow field of the run in flight every `every` solver iterations (rounded up to a multiple of
    /// [`SAMPLE_INTERVAL`](crate::progress::SAMPLE_INTERVAL); [`SNAPSHOT_INTERVAL`](crate::field::SNAPSHOT_INTERVAL) is a good value
    /// for a picture that follows the solve), from the thread that runs the analysis, before the progress report of that iteration.
    /// The CPU backend copies the solver's arrays; the GPU backend reads the state back from the device only at these iterations
    /// (a viscous run reads it back every `SAMPLE_INTERVAL` iterations for its drag monitor anyway, so a snapshot adds only the
    /// copy). A snapshot does not change the iteration or the result. The receiver owns the snapshot: keep it short (hand it to a
    /// channel). Conditions at an angle of attack take no `Field` snapshots: [`Analysis::on_field3`] gives theirs.
    ///
    /// Measured on an Apple M1 Max (viscous, Mach 2, the 1.1 m example body, a release build, the GPU backend): every 500 iterations
    /// cost nothing that the run-to-run spread shows, 18.3 s against 18.2 s for 2 000 iterations on the coarse grid and 8.4 s against
    /// 8.4 s for 1 000 on the medium one (two runs each). An Euler run on the GPU does not read the state back otherwise and pays one
    /// read-back per snapshot (not measured).
    pub fn on_field(mut self, every: usize, f: impl Fn(Field) + Send + Sync + 'static) -> Self {
        self.field_sink = Some(FieldSink::new(every, f));
        self
    }

    /// Calls `f` with a [`Field3`] snapshot of every 3D run (a condition at an angle of attack) in flight every `every` solver iterations
    /// (rounded up to a multiple of [`SAMPLE_INTERVAL`](crate::progress::SAMPLE_INTERVAL); [`SNAPSHOT_INTERVAL`](crate::field::SNAPSHOT_INTERVAL)
    /// is a good value), from the thread that runs the analysis; lengths are in metres. The GPU runner downloads the state of the
    /// device every `SAMPLE_INTERVAL` iterations for the normal-force monitor anyway, so a snapshot adds the conversion to a field (about
    /// 6 ms for 350 000 cells); the CPU runner reads its own arrays. A snapshot does not change the iteration or the result. The final
    /// state of a run is not a snapshot: [`Analysis::keep_fields`] keeps it.
    pub fn on_field3(mut self, every: usize, f: impl Fn(Field3) + Send + Sync + 'static) -> Self {
        self.field3_sink = Some(Snapshots3::new(every, self.field_scale(), f));
        self
    }

    /// Stops each point once its total drag varies by less than `rel` (relative to its mean) over the last 500 iterations, instead
    /// of the default rule (1e-4 absolute over 2000 iterations, judged from iteration 4000). Needs [`Analysis::viscous`]; `rel` must
    /// lie in `(0, 0.1]`. Measured on the example rocket of the app (coarse grid), `0.002` ends within 0.4 % of a 12 000-iteration
    /// solution with about 2.8 times fewer iterations.
    pub fn drag_tolerance(mut self, rel: f64) -> Self {
        self.drag_tolerance = Some(rel);
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

    /// Equivalent sand-grain roughness of the body surface in metres (default 0 = smooth). Needs [`Analysis::viscous`] and a
    /// dimensional [`BodySpec`]; must be finite, `>= 0` and below 1 % of the nose diameter.
    pub fn roughness(mut self, ks_m: f64) -> Self {
        self.roughness_m = ks_m;
        self
    }

    /// Add a set of fins to the drag. Needs [`Analysis::viscous`] flight conditions, which give the Reynolds number.
    pub fn fins(mut self, f: FinSpec) -> Self {
        self.fins = Some(f);
        self
    }

    /// Add a rocket motor exhausting through the centre of the base: a motor-on analysis, where the base flow contains the jet
    /// (an air jet at the nozzle exit, see [`MotorSpec`]). Needs [`Analysis::viscous`] and the base wake. The jet's thrust is not
    /// drag: [`Drag::base`] covers only the annulus of the base around the nozzle exit.
    pub fn motor(mut self, m: MotorSpec) -> Self {
        self.motor = Some(m);
        self
    }

    /// Free-stream Reynolds number, Sutherland constant and highest Mach number of a viscous sweep; `None` for the Euler equations.
    fn rans(&self, machs: &[f64]) -> Result<Option<Rans>, Error> {
        let Some(flight) = self.flight else {
            return Ok(None);
        };
        if !self.dimensional {
            return Err(Error::InvalidInput("viscous analysis needs a dimensional BodySpec".into()));
        }
        let air = flight.atmosphere.state()?;
        let re_c = air.density * air.sound_speed * self.d_nose / air.viscosity;
        let sutherland = SUTHERLAND_S / air.temperature_k;
        // jet at the nozzle exit in solver units (rho = 1, c = 1, p = 1/gamma, T = 1 of the free stream)
        let jet = self.motor.map(|m| {
            let t = m.total_temperature_k / (1.0 + 0.2 * m.exit_mach * m.exit_mach) / air.temperature_k;
            let p = m.exit_pressure_pa / air.pressure_pa;
            let rho = p / t;
            let u = m.exit_mach * t.sqrt();
            let k = 1.5 * (0.01 * u).powi(2);
            let mu = Viscous { re_c, sutherland, turbulence: Turbulence::Sst, wall_i_start: 0, roughness: 0.0 }.mu(t);
            Jet { r_exit: 0.5 * m.exit_diameter / self.d_nose, state: [rho, u, 0.0, p / GAMMA], turb: [k, rho * k / (10.0 * mu)] }
        });
        let mach_max = self.grid_mach.unwrap_or_else(|| machs.iter().copied().fold(f64::NEG_INFINITY, f64::max));
        Ok(Some(Rans { re_c, sutherland, mach_max, jet }))
    }

    /// The solver cases of a sweep in ascending Mach order, after the input checks of [`Analysis::run`]; with the Mach numbers' original
    /// positions and the Rans parameters.
    fn prepare(&self, machs: &[f64]) -> Result<(Vec<usize>, Option<Rans>, Vec<Case>), Error> {
        if machs.is_empty() {
            return Err(Error::InvalidInput("no Mach numbers requested".into()));
        }
        for &m in machs {
            if !(m.is_finite() && (MACH_RANGE.0..=MACH_RANGE.1).contains(&m)) {
                return Err(Error::InvalidInput(format!("Mach number {m} is outside [{}, {}]", MACH_RANGE.0, MACH_RANGE.1)));
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
        if let Some(rel) = self.drag_tolerance {
            if self.flight.is_none() {
                return Err(Error::InvalidInput("drag_tolerance needs Analysis::viscous".into()));
            }
            if !(rel.is_finite() && rel > 0.0 && rel <= 0.1) {
                return Err(Error::InvalidInput(format!("drag tolerance {rel} must be finite and in (0, 0.1]")));
            }
        }

        if !(self.roughness_m.is_finite() && self.roughness_m >= 0.0 && self.roughness_m < 0.01 * self.d_nose) {
            return Err(Error::InvalidInput(format!(
                "roughness {} m must be finite, >= 0 and below 1% of the nose diameter ({} m)",
                self.roughness_m,
                0.01 * self.d_nose
            )));
        }
        if self.roughness_m > 0.0 && (self.flight.is_none() || !self.dimensional) {
            return Err(Error::InvalidInput("roughness needs Analysis::viscous and a dimensional BodySpec".into()));
        }
        if let Some(f) = &self.fins {
            f.validate()?;
            if self.flight.is_none() {
                return Err(Error::InvalidInput("fins need flight conditions: call Analysis::viscous".into()));
            }
            if !self.dimensional {
                return Err(Error::InvalidInput("fins need a dimensional BodySpec".into()));
            }
        }

        if let Some(m) = &self.motor {
            if self.flight.is_none() {
                return Err(Error::InvalidInput("motor-on analysis needs Analysis::viscous".into()));
            }
            if self.wake == Some(Wake::Sting) {
                return Err(Error::InvalidInput("motor-on analysis needs the base wake".into()));
            }
            if !self.dimensional {
                return Err(Error::InvalidInput("motor-on analysis needs a dimensional BodySpec".into()));
            }
            m.validate(self.body.base_radius() * 2.0 * self.d_nose)?;
        }
        // cases in ascending Mach order (stable), results restored to input order by the caller
        let mut order: Vec<usize> = (0..machs.len()).collect();
        order.sort_by(|&a, &b| machs[a].total_cmp(&machs[b]));
        let rans = self.rans(machs)?;
        let level = self.resolution.level();
        let cases: Vec<Case> = order.iter().map(|&k| self.case(machs[k], level, rans.as_ref())).collect();
        // the grids do not depend on the Mach number: check the first one
        check_grid(&cases[0])?;
        Ok((order, rans, cases))
    }

    /// The solver cases of a sweep (ascending Mach order) for the validation crates, which drive the solvers themselves.
    #[doc(hidden)]
    pub fn engine_cases(&self, machs: &[f64]) -> Result<Vec<Case>, Error> {
        Ok(self.prepare(machs)?.2)
    }

    /// The grid of the analysis at `mach` with the free stream on it: the state a cold run starts from, as a [`Field`] (iteration
    /// 0). Builds the grid only; nothing is solved. `mach` is checked like the Mach numbers of [`Analysis::run`].
    pub fn initial_field(&self, mach: f64) -> Result<Field, Error> {
        let (_, _, cases) = self.prepare(&[mach])?;
        let case = &cases[0];
        let mut s = case.try_build_solver()?;
        s.u.fill(bowshock_cfd::gas::prim_to_cons(&bowshock_cfd::gas::freestream(mach)));
        Ok(Field::of_solver(&s, mach, 0, case.wake, self.field_scale()))
    }

    /// Length of a solver length unit (a nose diameter) in the unit of a [`Field`]'s coordinates: metres for a dimensional body.
    fn field_scale(&self) -> f64 {
        if self.dimensional {
            self.d_nose
        } else {
            1.0
        }
    }

    /// Solves every flight condition of `conditions` and gives one [`PolarPoint`] per condition, in the order given. This is the only
    /// solve entry: `run(&[0.5, 0.9])` (a `f64` is a [`Condition`] at zero incidence) is the zero-lift drag over a Mach sweep and
    /// `run(&[Condition::new(2.0, 4.0)])` the 3D solve at an angle of attack.
    ///
    /// * A condition at zero incidence (Mach number in `[0.1, 3.0]`) is the axisymmetric drag solver's: the full [`Drag`], and with
    ///   [`Analysis::viscous`] the skin friction, the base drag from the solution, the fins and the motor jet. Several of them are a
    ///   sweep in ascending Mach order ([`Analysis::warm_start`], [`Analysis::estimate_uncertainty`]).
    /// * A condition at an angle of attack (Mach number in `[0.3, 3.0]`, angle in `(0, 15]` degrees) is the 3D Euler solver's: the
    ///   normal force, the pitching moment, the centre of pressure and the axial pressure force ([`PolarPoint::forces`]); see
    ///   [`PolarPoint`] for what the drag of such a point holds. It needs an analysis built with [`Analysis::new`]. The solver
    ///   integrates the Euler equations on a sting-mounted body with thick fins: [`Analysis::viscous`], roughness, the motor and the
    ///   wake are ignored, and so are the fin roughness and the fin-drag model. Fins come from [`Analysis::fins`]; the grid level from
    ///   [`Analysis::resolution`]. The conditions are solved in ascending Mach number, then angle; each Mach number starts cold and its
    ///   later angles start from the field of the previous one, and a warm point that ends `Failed` or `Stalled` is re-run cold. A
    ///   point whose run diverged is re-run once with the implicit CFL limit halved.
    ///
    /// Conditions of both kinds in one call run one after the other: the drag sweep first. The backends are as for
    /// [`Analysis::backend`]; [`Polar::backend`] is the GPU only if every condition ran on it.
    pub fn run<C: Into<Condition> + Copy>(&self, conditions: &[C]) -> Result<Polar, Error> {
        let conditions: Vec<Condition> = conditions.iter().map(|&c| c.into()).collect();
        self.route(&conditions)
    }

    /// THE ROUTE: the one place that decides which engine solves which condition, and the only place that knows there are two. A
    /// condition at zero incidence goes to [`Analysis::run_drag`] (the axisymmetric solver), one at an angle of attack to
    /// [`Analysis::run_lift_points`] (the 3D Euler solver). When one 3D viscous solver does both, this function becomes a single call
    /// of it and the two paths below are deleted.
    ///
    /// Swapping the engine replaces: this function, [`Analysis::run_drag`] with the drag helpers it uses (`prepare`, `case`, `drag`, `point`,
    /// `detail`, `warnings`, `sweep*`, `Runner`), and the 3D path (`check_lift`, `run_lift_points`, `lift_case`, `lift_point`, `lift_loop`,
    /// `lift_gpu`, `LiftRunner` and its two runners, `lift_quality`). It keeps the public types ([`Condition`], [`Forces`], [`PolarPoint`] with
    /// its `alpha_deg`, `forces` and `field3`), the sweep order and the progress, cancel and snapshot contracts: a point at an angle of attack
    /// is built in `lift_point` (what is NaN there is listed on [`PolarPoint`]), the final `Field3` comes from the runner's state
    /// (`LiftRunner::field3`) and the live ones from `Case3::snapshots` ([`Snapshots3`]), which a runner calls where it holds the state
    /// of the grid after a chunk of iterations (`run3_on` and `GpuRunner3::run_on`: three lines each). [`Analysis::case3_hook`] is the tests'
    /// way to shrink the grid of every 3D case.
    fn route(&self, conditions: &[Condition]) -> Result<Polar, Error> {
        if conditions.is_empty() {
            return Err(Error::InvalidInput("no flight conditions requested".into()));
        }
        if let Some(c) = conditions.iter().find(|c| !c.alpha_deg.is_finite()) {
            return Err(Error::InvalidInput(format!("incidence {} deg is not finite", c.alpha_deg)));
        }
        let (drag_at, lift_at): (Vec<usize>, Vec<usize>) = (0..conditions.len()).partition(|&k| conditions[k].alpha_deg == 0.0);
        let lift_conditions: Vec<Condition> = lift_at.iter().map(|&k| conditions[k]).collect();
        if !lift_conditions.is_empty() {
            self.check_lift(&lift_conditions)?;
        }
        let mut slots: Vec<Option<PolarPoint>> = (0..conditions.len()).map(|_| None).collect();
        let mut ran: Option<(BackendUsed, Option<String>)> = None;
        let mut motor_on = false;
        if !drag_at.is_empty() {
            let machs: Vec<f64> = drag_at.iter().map(|&k| conditions[k].mach).collect();
            let polar = self.run_drag(&machs)?;
            motor_on = polar.motor_on;
            ran = Some((polar.backend, polar.fallback));
            for (&k, p) in drag_at.iter().zip(polar.points) {
                slots[k] = Some(p);
            }
        }
        if !lift_at.is_empty() {
            let (points, backend, fallback) = self.run_lift_points(&lift_conditions)?;
            for (&k, p) in lift_at.iter().zip(points) {
                slots[k] = Some(p);
            }
            ran = Some(match ran {
                None => (backend, fallback),
                Some((b, f)) if b == backend => (b, f.or(fallback)),
                Some((_, f)) => (BackendUsed::Cpu, f.or(fallback)),
            });
        }
        let (backend, fallback) = ran.expect("a condition was solved");
        let points = slots.into_iter().map(|p| p.expect("every condition has a result")).collect();
        Ok(Polar { points, backend, fallback, motor_on })
    }

    /// The drag solver's sweep over `machs` (each in `[0.1, 3.0]`, zero incidence).
    fn run_drag(&self, machs: &[f64]) -> Result<Polar, Error> {
        let (order, rans, cases) = self.prepare(machs)?;
        let level = self.resolution.level();
        let cmp_cases: Option<Vec<Case>> = self.estimate_uncertainty.then(|| {
            let cmp_level = if level >= 1 { level - 1 } else { level + 1 };
            order.iter().map(|&k| self.case(machs[k], cmp_level, rans.as_ref())).collect()
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
        Ok(Polar { points, backend, fallback, motor_on: self.motor.is_some() })
    }

    /// Wake treatment in use: the one set with [`Analysis::wake`], else a base wake for a viscous analysis and a sting otherwise.
    fn wake_used(&self) -> Wake {
        self.wake.unwrap_or(if self.flight.is_some() { Wake::Base } else { Wake::Sting })
    }

    fn case(&self, mach: f64, level: u32, rans: Option<&Rans>) -> Case {
        let wake = self.wake_used();
        let mut case = match rans {
            Some(r) => Case::viscous(mach, self.body.clone(), level, wake, r.re_c, r.sutherland, r.mach_max),
            None => Case::new(mach, self.body.clone(), level, wake, Numerics::auto(mach, level, wake)),
        };
        if let Some(v) = case.viscous.as_mut() {
            v.roughness = self.roughness_m / self.d_nose;
        }
        if let Some(n) = self.numerics {
            case.numerics = n;
        }
        case.numerics.drag_rel_tol = self.drag_tolerance;
        if let Some(cap) = self.max_iterations {
            case.numerics.max_iter = case.numerics.max_iter.min(cap);
        }
        let q = if self.flight.is_some() { (self.d_nose / self.d_ref).powi(2) } else { 1.0 };
        case.progress = self.progress.as_ref().map(|p| p.scaled(q));
        // Snapshots come from every run; the final field is kept only on the grid of the requested resolution (the uncertainty
        // sweep runs the neighbouring level, whose drag is not the one reported).
        let keep = self.keep_fields && level == self.resolution.level();
        case.fields = (keep || self.field_sink.is_some()).then(|| FieldTap { scale: self.field_scale(), keep, live: self.field_sink.clone() });
        match rans.and_then(|r| r.jet) {
            Some(jet) => case.with_motor(jet),
            None => case,
        }
    }

    /// Drag of one solver result, referenced to the reference area. A viscous analysis adds the skin friction and, with a base
    /// wake, takes the base drag from the solution instead of the correlation.
    fn drag(&self, mach: f64, detail: &PointResult) -> Drag {
        let q = (self.d_nose / self.d_ref).powi(2);
        let forebody_pressure = detail.cd_aft.iter().fold(detail.cd_nose, |a, c| a + c);
        let viscous = self.flight.is_some();
        let friction = viscous.then_some(detail.cd_friction);
        let base_modelled = !(viscous && self.wake_used() == Wake::Base);
        let base = if base_modelled { base_drag_coefficient(mach) * (self.body.base_radius() / self.body.body.radius()).powi(2) } else { detail.cd_base };
        let mut total = (forebody_pressure + friction.unwrap_or(0.0) + base) * q;
        let fins = self
            .fins
            .as_ref()
            .zip(self.flight.and_then(|f| f.atmosphere.state().ok()))
            .map(|(f, air)| crate::fins::fin_drag(f, mach, &air, self.d_ref, self.body.x_base() * self.d_nose));
        if let Some(fd) = fins {
            total += fd.friction + fd.pressure;
        }
        Drag { forebody_pressure: forebody_pressure * q, friction: friction.map(|f| f * q), base: base * q, base_modelled, fins, total }
    }

    fn point(&self, mach: f64, mut detail: PointResult, other: Option<&PointResult>) -> PolarPoint {
        let field = detail.field.take();
        let drag = self.drag(mach, &detail);
        let quality = quality(&detail, &drag);
        let warnings = self.warnings(mach, &detail);
        let cd_uncertainty = other.and_then(|o| {
            let od = self.drag(mach, o);
            (quality != Quality::Failed && self::quality(o, &od) != Quality::Failed).then(|| (drag.total - od.total).abs())
        });
        let detail = self.detail(&detail);
        PolarPoint { mach, alpha_deg: 0.0, drag, forces: None, quality, warnings, cd_uncertainty, detail, field, field3: None }
    }

    /// Public summary of a solver result: coefficients on the reference area, wall points in metres.
    fn detail(&self, d: &PointResult) -> PointDetail {
        let q = (self.d_nose / self.d_ref).powi(2);
        let base = self.wake_used() == Wake::Base;
        let x_base = self.body.x_base();
        PointDetail {
            iterations: d.iters,
            residual_drop_orders: d.drop_orders,
            stop_reason: d.stop_reason,
            cd_nose: d.cd_nose * q,
            cd_aft: d.cd_aft.iter().map(|c| c * q).collect(),
            cd_friction: d.cd_friction * q,
            cd_base_solver: if base { d.cd_base * q } else { f64::NAN },
            cp_base: if base { d.cp_base } else { f64::NAN },
            wall: d.wall.iter().filter(|w| w.x < x_base).map(|w| WallCp { x: w.x * self.d_nose, r: w.r * self.d_nose, cp: w.cp }).collect(),
            y_plus_max: d.y_plus_max,
            farfield_dp: d.farfield_dp,
        }
    }

    fn warnings(&self, mach: f64, detail: &PointResult) -> Vec<Warning> {
        let mut w = Vec::new();
        if (0.7..1.05).contains(&mach) && self.has_sharp_corner() {
            w.push(Warning::SharpCornerTransonic);
        }
        if self.wake_used() == Wake::Base && detail.cp_base_range.is_finite() && detail.cp_base_range >= 1e-3 && detail.stop_reason != StopReason::ResidualDrop
        {
            w.push(Warning::UnsteadyWake { cp_base_range: detail.cp_base_range });
        }
        if detail.farfield_dp > 0.02 {
            w.push(Warning::FarFieldDisturbed { dp: detail.farfield_dp });
        }
        if detail.y_plus_max > 5.0 {
            w.push(Warning::WallResolution { y_plus_max: detail.y_plus_max });
        }
        if detail.stop_reason == StopReason::MeanStationary {
            let window = self.numerics.and_then(|n| n.mean_window).or(Numerics::viscous(mach, 0).mean_window).unwrap_or(0);
            let k = (window / crate::engine::numerics::MONITOR_EVERY).max(1);
            let h = &detail.monitor_history;
            let (lo, hi) = h[h.len().saturating_sub(2 * k)..].iter().fold((f64::INFINITY, f64::NEG_INFINITY), |(l, h), c| (l.min(c.1), h.max(c.1)));
            w.push(Warning::OscillatingDrag { cd_range: (hi - lo) * (self.d_nose / self.d_ref).powi(2) });
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
            Backend::Cpu => Ok((guard(|| self.sweep_cpu(cases))?, BackendUsed::Cpu, None)),
            Backend::Gpu => Ok((guard(|| self.sweep_gpu_required(cases))?, BackendUsed::Gpu, None)),
            Backend::Auto => self.sweep_auto(cases),
        }
    }

    /// The sweep logic shared by the backends: first point cold; later points warm (when enabled) against the residual peak
    /// of the most recent cold start; a warm point that ends `Failed` or `Stalled` is re-run cold on a fresh solver, and a point
    /// after a `Failed` point starts cold on a fresh solver.
    fn sweep<R: Runner<Error = Error>>(&self, runner: &R, cases: &[Case]) -> Result<Vec<PointResult>, Error> {
        let quality_of = |case: &Case, d: &PointResult| quality(d, &self.drag(case.mach, d));
        let mut out: Vec<PointResult> = Vec::with_capacity(cases.len());
        let mut state: Option<R::State> = None;
        let mut r_ref = 0.0;
        let mut previous_failed = false;
        for case in cases {
            let warm = match (&mut state, previous_failed) {
                (Some(s), false) => {
                    let r = runner.warm(s, case, r_ref)?;
                    if r.stop_reason == StopReason::Cancelled {
                        return Err(Error::Cancelled);
                    }
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
            if result.stop_reason == StopReason::Cancelled {
                return Err(Error::Cancelled);
            }
            previous_failed = quality_of(case, &result) == Quality::Failed;
            out.push(result);
        }
        Ok(out)
    }

    fn sweep_cpu(&self, cases: &[Case]) -> Result<Vec<PointResult>, Error> {
        self.sweep(&CpuBackend, cases)
    }

    #[cfg(feature = "gpu")]
    fn sweep_gpu_required(&self, cases: &[Case]) -> Result<Vec<PointResult>, Error> {
        self.sweep(&GpuBackend, cases)
    }

    #[cfg(not(feature = "gpu"))]
    fn sweep_gpu_required(&self, _cases: &[Case]) -> Result<Vec<PointResult>, Error> {
        Err(Error::Unsupported("the `gpu` feature is not enabled".into()))
    }

    #[cfg(feature = "gpu")]
    fn sweep_auto(&self, cases: &[Case]) -> Result<(Vec<PointResult>, BackendUsed, Option<String>), Error> {
        let why = match guard(|| self.sweep_gpu_required(cases)) {
            Ok(results) => return Ok((results, BackendUsed::Gpu, None)),
            Err(Error::Internal(m)) => m.replacen("solver panicked", "GPU solver panicked", 1),
            Err(Error::Cancelled) => return Err(Error::Cancelled),
            Err(e) => format!("GPU unavailable: {e}"),
        };
        Ok((guard(|| self.sweep_cpu(cases))?, BackendUsed::Cpu, Some(why)))
    }

    #[cfg(not(feature = "gpu"))]
    fn sweep_auto(&self, cases: &[Case]) -> Result<(Vec<PointResult>, BackendUsed, Option<String>), Error> {
        Ok((guard(|| self.sweep_cpu(cases))?, BackendUsed::Cpu, None))
    }
}

/// Quality of a 3D result: `Failed` on a non-finite state or force, else by stop reason like the drag solver.
fn lift_quality(d: &PointResult3) -> Quality {
    if matches!(d.stop_reason, StopReason::NonFinite | StopReason::Cancelled) || !d.cn.is_finite() || !d.ca.is_finite() {
        return Quality::Failed;
    }
    match d.stop_reason {
        StopReason::ResidualDrop => Quality::Converged,
        StopReason::CdStationary | StopReason::MeanStationary | StopReason::BasePressureStationary | StopReason::ResidualPlateau => Quality::Stationary,
        StopReason::MaxIterations => Quality::Stalled,
        StopReason::NonFinite | StopReason::Cancelled => Quality::Failed,
    }
}

/// The conditions at an angle of attack: the 3D Euler solver (see [`Analysis::run`]). Everything here is replaced with the route.
impl Analysis {
    /// The input checks of the 3D path for `conditions`, all at an angle of attack.
    fn check_lift(&self, conditions: &[Condition]) -> Result<(), Error> {
        if !self.dimensional {
            return Err(Error::InvalidInput("a condition with an angle of attack needs a dimensional BodySpec (Analysis::new)".into()));
        }
        for c in conditions {
            if !(c.mach.is_finite() && (LIFT_MACH_RANGE.0..=LIFT_MACH_RANGE.1).contains(&c.mach)) {
                return Err(Error::InvalidInput(format!(
                    "Mach number {} is outside [{}, {}] for a condition with an angle of attack",
                    c.mach, LIFT_MACH_RANGE.0, LIFT_MACH_RANGE.1
                )));
            }
            if !(c.alpha_deg > 0.0 && c.alpha_deg <= LIFT_ALPHA_MAX_DEG) {
                return Err(Error::InvalidInput(format!("incidence {} deg is outside (0, {LIFT_ALPHA_MAX_DEG}]", c.alpha_deg)));
            }
        }
        if !(self.d_ref.is_finite() && self.d_ref > 0.0) {
            return Err(Error::InvalidInput(format!("reference diameter {} must be finite and > 0", self.d_ref)));
        }
        if let Some(f) = &self.fins {
            f.validate()?;
        }
        Ok(())
    }

    /// Solves `conditions` (all at an angle of attack, checked by [`Analysis::check_lift`]) on the 3D solver: the points in the order
    /// of `conditions`, the backend that ran them and, under [`Backend::Auto`], why the GPU was not used.
    fn run_lift_points(&self, conditions: &[Condition]) -> Result<(Vec<PolarPoint>, BackendUsed, Option<String>), Error> {
        // solved in ascending Mach number, then angle (stable); results go back to the order asked for
        let mut order: Vec<usize> = (0..conditions.len()).collect();
        order.sort_by(|&a, &b| conditions[a].mach.total_cmp(&conditions[b].mach).then(conditions[a].alpha_deg.total_cmp(&conditions[b].alpha_deg)));
        let sorted: Vec<Condition> = order.iter().map(|&k| conditions[k]).collect();
        let (solved, backend, fallback) = match self.backend {
            Backend::Cpu => (guard(|| self.lift_loop(&CpuLift, &sorted))?, BackendUsed::Cpu, None),
            Backend::Gpu => (guard(|| self.lift_gpu(&sorted))?, BackendUsed::Gpu, None),
            Backend::Auto => match guard(|| self.lift_gpu(&sorted)) {
                Ok(p) => (p, BackendUsed::Gpu, None),
                Err(Error::Cancelled) => return Err(Error::Cancelled),
                Err(e) => {
                    let why = match e {
                        Error::Internal(m) => m.replacen("solver panicked", "GPU solver panicked", 1),
                        e => format!("GPU unavailable: {e}"),
                    };
                    (guard(|| self.lift_loop(&CpuLift, &sorted))?, BackendUsed::Cpu, Some(why))
                }
            },
        };
        let mut slots: Vec<Option<PolarPoint>> = (0..conditions.len()).map(|_| None).collect();
        for (&k, p) in order.iter().zip(solved) {
            slots[k] = Some(p);
        }
        Ok((slots.into_iter().map(|p| p.expect("every condition has a result")).collect(), backend, fallback))
    }

    /// Fins of the analysis in nose diameters.
    fn fins3(&self) -> Option<Fins3> {
        self.fins.as_ref().map(|f| {
            let d = self.d_nose;
            Fins3 {
                count: f.count,
                root_chord: f.root_chord / d,
                tip_chord: f.tip_chord / d,
                span: f.span / d,
                sweep: f.sweep / d,
                thickness: f.thickness / d,
                te_to_base: f.root_te_to_base / d,
                profile: f.profile.clone(),
            }
        })
    }

    /// The 3D case of one Mach number and incidence \[rad\], with the numerics overrides applied.
    fn lift_case(&self, mach: f64, alpha: f64) -> Case3 {
        let mut case = Case3::new(mach, alpha, self.body.clone(), self.fins3(), self.resolution.level());
        if let Some(n) = &self.numerics {
            case.numerics.limiter = n.limiter;
            case.numerics.low_mach = n.low_mach;
            case.numerics.first_order_iters = n.first_order_iters;
            case.numerics.max_iter = n.max_iter;
        }
        if let Some(cap) = self.max_iterations {
            case.numerics.max_iter = case.numerics.max_iter.min(cap);
        }
        case.progress = self.progress.as_ref().map(|p| p.scaled((self.d_nose / self.d_ref).powi(2)));
        case.snapshots = self.field3_sink.clone();
        if let Some(hook) = self.case3_hook {
            hook(&mut case);
        }
        case
    }

    /// The point of the 3D result `r` of the condition `c`, solved on `case` and left in `state`: coefficients on the reference area,
    /// lengths in metres, the final field when it is kept.
    fn lift_point<R: LiftRunner>(&self, runner: &R, state: &R::State, case: &Case3, c: Condition, r: &PointResult3) -> Result<PolarPoint, Error> {
        let q = (self.d_nose / self.d_ref).powi(2);
        let (cn, ca, x_cp) = (r.cn * q, r.ca * q, r.x_cp * self.d_nose);
        let cm = if x_cp.is_finite() { -cn * x_cp / self.d_ref } else { 0.0 };
        let drag = Drag { forebody_pressure: ca, friction: None, base: f64::NAN, base_modelled: false, fins: None, total: f64::NAN };
        let detail = PointDetail {
            iterations: r.iterations,
            residual_drop_orders: r.drop_orders,
            stop_reason: r.stop_reason,
            cd_nose: f64::NAN,
            cd_aft: Vec::new(),
            cd_friction: 0.0,
            cd_base_solver: f64::NAN,
            cp_base: f64::NAN,
            wall: Vec::new(),
            y_plus_max: f64::NAN,
            farfield_dp: 0.0,
        };
        let field3 = if self.keep_fields { Some(runner.field3(state, case, r.iterations, self.field_scale())?) } else { None };
        Ok(PolarPoint {
            mach: c.mach,
            alpha_deg: c.alpha_deg,
            drag,
            forces: Some(Forces { cn, cm, x_cp, ca }),
            quality: lift_quality(r),
            warnings: Vec::new(),
            cd_uncertainty: None,
            detail,
            field: None,
            field3,
        })
    }

    /// The sweep loop shared by the 3D backends over `sorted` (ascending Mach number, then angle): per Mach number a cold first angle,
    /// warm later ones. A warm point that ends `Failed` or `Stalled` is re-run cold; a cold run that diverged is repeated once with
    /// the implicit CFL limit halved.
    fn lift_loop<R: LiftRunner>(&self, runner: &R, sorted: &[Condition]) -> Result<Vec<PolarPoint>, Error> {
        let mut points = Vec::with_capacity(sorted.len());
        let mut state: Option<R::State> = None;
        let mut state_mach = f64::NAN;
        for &c in sorted {
            if c.mach != state_mach {
                state = None;
                state_mach = c.mach;
            }
            let case = self.lift_case(c.mach, c.alpha_deg.to_radians());
            if let Some(s) = &mut state {
                let r = runner.warm(s, &case)?;
                if r.stop_reason == StopReason::Cancelled {
                    return Err(Error::Cancelled);
                }
                if matches!(lift_quality(&r), Quality::Converged | Quality::Stationary) {
                    points.push(self.lift_point(runner, s, &case, c, &r)?);
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
            points.push(self.lift_point(runner, &s, &case, c, &r)?);
            state = next_warm.then_some(s);
        }
        Ok(points)
    }

    /// GPU backend of the 3D path (`sorted` validated and sorted): single precision (`Precision::F32`, within 0.01 % of extended on the ANF table).
    #[cfg(feature = "gpu")]
    fn lift_gpu(&self, sorted: &[Condition]) -> Result<Vec<PolarPoint>, Error> {
        self.lift_loop(&GpuLift, sorted)
    }

    #[cfg(not(feature = "gpu"))]
    fn lift_gpu(&self, _sorted: &[Condition]) -> Result<Vec<PolarPoint>, Error> {
        Err(Error::Unsupported("the `gpu` feature is not enabled".into()))
    }
}

/// One backend of the 3D sweep: a cold run that builds the solver state, a warm run on it, and the flow field of a state.
trait LiftRunner {
    type State;
    fn cold(&self, case: &Case3) -> Result<(PointResult3, Self::State), Error>;
    fn warm(&self, state: &mut Self::State, case: &Case3) -> Result<PointResult3, Error>;
    /// The field of `state` after `iterations` iterations, in units of `scale` metres per nose diameter.
    fn field3(&self, state: &Self::State, case: &Case3, iterations: usize, scale: f64) -> Result<Field3, Error>;
}

struct CpuLift;

impl LiftRunner for CpuLift {
    type State = Solver3;

    fn cold(&self, case: &Case3) -> Result<(PointResult3, Solver3), Error> {
        let mut s = case.build_solver()?;
        let r = run3_on(case, &mut s, false);
        Ok((r, s))
    }

    fn warm(&self, s: &mut Solver3, case: &Case3) -> Result<PointResult3, Error> {
        Ok(run3_on(case, s, true))
    }

    fn field3(&self, s: &Solver3, case: &Case3, iterations: usize, scale: f64) -> Result<Field3, Error> {
        Ok(field3_of_solver(case, s, iterations, scale))
    }
}

#[cfg(feature = "gpu")]
struct GpuLift;

#[cfg(feature = "gpu")]
impl LiftRunner for GpuLift {
    type State = crate::engine::gpu3::GpuRunner3;

    fn cold(&self, case: &Case3) -> Result<(PointResult3, crate::engine::gpu3::GpuRunner3), Error> {
        let mut r = crate::engine::gpu3::GpuRunner3::new(case, bowshock_cfd::gpu::Precision::F32)?;
        let p = r.run_on(case, false)?;
        Ok((p, r))
    }

    fn warm(&self, r: &mut crate::engine::gpu3::GpuRunner3, case: &Case3) -> Result<PointResult3, Error> {
        r.run_on(case, true)
    }

    fn field3(&self, r: &crate::engine::gpu3::GpuRunner3, case: &Case3, iterations: usize, scale: f64) -> Result<Field3, Error> {
        Ok(crate::engine::field3::field3_of(case, &r.host.grid, &r.gpu.state_f64()?, iterations, scale))
    }
}

/// Runs `f`, turning a panic into `Error::Internal("solver panicked: ...")`.
fn guard<T>(f: impl FnOnce() -> Result<T, Error>) -> Result<T, Error> {
    std::panic::catch_unwind(std::panic::AssertUnwindSafe(f)).unwrap_or_else(|payload| {
        let msg = payload.downcast_ref::<&str>().map(|s| (*s).to_string()).or_else(|| payload.downcast_ref::<String>().cloned());
        Err(Error::Internal(format!("solver panicked: {}", msg.as_deref().unwrap_or("unknown"))))
    })
}

/// Builds the grids of `case` and checks their quality (the wall-resolved grids of a viscous case against the tighter limit in `j`).
fn check_grid(case: &Case) -> Result<(), Error> {
    let probe = case.try_build_solver()?;
    let max_j = if case.viscous.is_some() { MAX_STRETCHING_VISCOUS_J } else { MAX_STRETCHING };
    check_quality(&probe.grid, probe.block(), MAX_STRETCHING, max_j)
}

/// Relative deviation of the outer-boundary pressure from the free stream above which a state is not a meaningful solution.
const FARFIELD_DP_FAILED: f64 = 1.0;

/// Quality of a solver result, first match wins. A point is [`Quality::Failed`] when the run stopped on a non-finite or blown-up
/// state, when the density or pressure of a cell is not positive, when the total drag is not finite, when the pressure at the
/// outer boundaries deviates from the free stream by more than 100 % (`farfield_dp > 1`): the far field of a body in a flow of
/// this size is undisturbed at that distance, so such a state is garbage even if it is finite and the run ended `Stalled`; or
/// when the area-mean base Cp is below the vacuum limit `-2 / (GAMMA M^2)` (the extrapolated base pressure is not positive).
fn quality(d: &PointResult, drag: &Drag) -> Quality {
    let below_vacuum = d.cp_base < -2.0 / (GAMMA * d.mach * d.mach); // false for NaN
    let positive = |x: f64| x > 0.0; // false for NaN
    if matches!(d.stop_reason, StopReason::NonFinite | StopReason::Cancelled)
        || !positive(d.min_rho)
        || !positive(d.min_p)
        || !drag.total.is_finite()
        || d.farfield_dp > FARFIELD_DP_FAILED
        || below_vacuum
    {
        return Quality::Failed;
    }
    match d.stop_reason {
        StopReason::ResidualDrop => Quality::Converged,
        StopReason::CdStationary | StopReason::MeanStationary | StopReason::BasePressureStationary | StopReason::ResidualPlateau => Quality::Stationary,
        StopReason::MaxIterations => Quality::Stalled,
        StopReason::NonFinite | StopReason::Cancelled => Quality::Failed,
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
    type Error = Error;

    fn cold(&self, case: &Case) -> Result<(PointResult, Solver), Error> {
        let mut s = case.try_build_solver()?;
        let r = run_on(case, &mut s, false, None);
        Ok((r, s))
    }

    fn warm(&self, s: &mut Solver, case: &Case, r_ref: f64) -> Result<PointResult, Error> {
        Ok(run_on(case, s, true, Some(r_ref)))
    }
}

#[cfg(feature = "gpu")]
struct GpuBackend;

#[cfg(feature = "gpu")]
impl Runner for GpuBackend {
    type State = crate::engine::gpu::GpuRunner;
    type Error = Error;

    fn cold(&self, case: &Case) -> Result<(PointResult, crate::engine::gpu::GpuRunner), Error> {
        let mut r = crate::engine::gpu::GpuRunner::new(case, bowshock_cfd::gpu::Precision::Extended)?;
        let run = r.run_on(case, false, None)?;
        Ok((run.result, r))
    }

    fn warm(&self, r: &mut crate::engine::gpu::GpuRunner, case: &Case, r_ref: f64) -> Result<PointResult, Error> {
        Ok(r.run_on(case, true, Some(r_ref))?.result)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn result(stop_reason: StopReason, farfield_dp: f64) -> PointResult {
        PointResult {
            mach: 2.0,
            alpha: 0.0,
            iters: 1000,
            residual: Vec::new(),
            residual_peak: 1.0,
            drop_orders: 1.0,
            stop_reason,
            wall: Vec::new(),
            base_wall: Vec::new(),
            cd_nose: 0.1,
            cd_nose_cell: 0.1,
            cd_aft: Vec::new(),
            cd_base: 0.1,
            cd_friction: 0.1,
            cd_total: 0.3,
            cd_base_model: 0.1,
            cd_total_model: 0.3,
            cp_base: -0.1,
            cp_base_cell: -0.1,
            cp_base_mean: f64::NAN,
            cp_base_range: f64::NAN,
            cd_range: f64::NAN,
            monitor_history: Vec::new(),
            cp_min: -0.1,
            x_cp_min: 1.0,
            m_max: 2.0,
            m_wall_max: 1.5,
            farfield_dp,
            min_rho: 0.1,
            min_p: 0.1,
            y_plus_max: 1.0,
            field: None,
        }
    }

    fn drag(total: f64) -> Drag {
        Drag { forebody_pressure: 0.1, friction: Some(0.1), base: 0.1, base_modelled: false, fins: None, total }
    }

    #[test]
    fn quality_fails_on_a_far_field_that_deviates_by_more_than_100_percent() {
        for stop in [StopReason::ResidualDrop, StopReason::CdStationary, StopReason::MaxIterations] {
            let expect = if stop == StopReason::MaxIterations { Quality::Stalled } else { Quality::Converged };
            let expect = if stop == StopReason::CdStationary { Quality::Stationary } else { expect };
            assert_eq!(quality(&result(stop, 0.0), &drag(0.3)), expect);
            assert_eq!(quality(&result(stop, 1.0), &drag(0.3)), expect, "exactly 100 % is not beyond the limit");
            assert_eq!(quality(&result(stop, 1.0 + 1e-9), &drag(0.3)), Quality::Failed);
            assert_eq!(quality(&result(stop, 1.2e7), &drag(-13905.0)), Quality::Failed);
        }
        assert_eq!(quality(&result(StopReason::NonFinite, 0.0), &drag(0.3)), Quality::Failed);
        assert_eq!(quality(&result(StopReason::ResidualDrop, 0.0), &drag(f64::NAN)), Quality::Failed);
    }

    #[test]
    fn quality_fails_on_a_base_cp_below_the_vacuum_limit() {
        let mut r = result(StopReason::ResidualDrop, 0.0);
        r.mach = 1.65;
        let limit = -2.0 / (GAMMA * 1.65 * 1.65);
        for (stop, expect) in
            [(StopReason::ResidualDrop, Quality::Converged), (StopReason::CdStationary, Quality::Stationary), (StopReason::MaxIterations, Quality::Stalled)]
        {
            r.stop_reason = stop;
            r.cp_base = -1473.0;
            assert_eq!(quality(&r, &drag(0.3)), Quality::Failed);
            r.cp_base = limit * (1.0 - 1e-9);
            assert_eq!(quality(&r, &drag(0.3)), expect);
            r.cp_base = limit * (1.0 + 1e-9);
            assert_eq!(quality(&r, &drag(0.3)), Quality::Failed);
            r.cp_base = f64::NAN;
            assert_eq!(quality(&r, &drag(0.3)), expect);
        }
    }
}
