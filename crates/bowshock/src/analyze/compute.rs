//! The Analyze screen's physics: which analyses exist, the inputs they take, and the pure function that runs one off the
//! UI thread. Everything here is SI (metres, radians, m/s); the screen formats at the view.
//!
//! # Methods
//!
//! * **Barrowman** is the linear theory of Barrowman (1967): normal force slope and centre of pressure at zero angle of
//!   attack, whatever angle is set (the angle field is ignored; drag is the zero-angle build-up).
//! * **Extended** is Barrowman plus the body lift and induced drag `bowshock-sim` adds at an angle of attack: the
//!   margin, CP, `CNα` and drag follow the angle the Conditions set.
//! * **CFD table** is the drag at zero incidence solved by `bowshock-core` (see [`super::cfd`]) on a table of Mach numbers and,
//!   unless left out, the margin, CP and `CNα` from its 3D solve at two angles of attack; Extended's change of drag with the angle
//!   is added to the solved drag, and Extended gives whatever the solve did not (what the screens say about it is `cfd::wording`).
//!
//! # References
//!
//! Barrowman (1967); Niskanen, *Development of an Open Source model rocket simulation software* (2009); NACA TN 4197 (Martin, 1958) for the fin flutter
//! velocity.

use std::sync::Arc;
use std::time::Duration;

use super::cfd;
use super::flutter::{FinStock, FlutterAnalysis};
use crate::units::{Quantity, UnitSystem};
use bowshock_model::{CfdGrid, CfdTable, ConfigId, Finish, Rocket, StabilityBands};
use bowshock_motors::MotorDb;
use bowshock_sim::from_model::{self, Phase};
use bowshock_sim::{
    monte_carlo, simulate, vs_alpha, vs_mach, AlphaPoint, Atmosphere, Dispersion, DragBreakdown, DragTable, Launch, MachPoint, MonteCarloStats, SimInput,
    Vehicle,
};

/// The Mach number of the static balance the status strip shows (kept in step with `app::physics`).
pub const BALANCE_MACH: f64 = crate::app::physics::BALANCE_MACH;
/// Flutter velocity over the flight's speed below which a fin set is flagged as marginal (a 25 % allowance on a
/// preliminary-design estimate); below 1.0 the estimate says it flutters.
pub const FLUTTER_MIN_MARGIN: f64 = 1.25;
/// Most configurations drawn at once: the four categorical series colours (never cycled).
pub const MAX_CONFIGS: usize = 4;

/// One of the five analyses of the left list.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Analysis {
    /// Static margin, CP/CG and drag against Mach.
    Aerodynamics,
    /// Static margin, CP/CG and normal force against angle of attack.
    Stability,
    /// Drag by source against Mach.
    Drag,
    /// Fin flutter margin along the flight.
    Flutter,
    /// Landing scatter and apogee spread of a Monte Carlo of flights.
    Dispersion,
    /// A search for the dimensions that reach a goal (its own state and worker; see [`super::optimize`]).
    Optimize,
}

impl Analysis {
    /// Every analysis, in list order.
    pub const ALL: [Analysis; 5] = [Analysis::Aerodynamics, Analysis::Stability, Analysis::Drag, Analysis::Flutter, Analysis::Dispersion];

    /// The list row's name.
    pub fn name(self) -> &'static str {
        match self {
            Analysis::Aerodynamics => "Aerodynamics",
            Analysis::Stability => "Stability",
            Analysis::Drag => "Drag breakdown",
            Analysis::Flutter => "Fin flutter",
            Analysis::Dispersion => "Dispersion",
            Analysis::Optimize => "Optimize",
        }
    }

    /// The small mono note at the right of the row.
    pub fn hint(self) -> &'static str {
        match self {
            Analysis::Aerodynamics | Analysis::Drag => "vs Mach",
            Analysis::Stability => "vs AoA",
            Analysis::Flutter => "margin",
            Analysis::Dispersion => "Monte Carlo",
            Analysis::Optimize => "design",
        }
    }
}

/// How the coefficients are obtained; see the module docs.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Method {
    /// Linear Barrowman theory at zero angle of attack.
    Barrowman,
    /// Barrowman plus body lift and induced drag at the set angle.
    Extended,
    /// A solved CFD table (drag from `bowshock-core`, with its 3D angles when it has them; Extended gives the rest).
    CfdTable,
}

impl Method {
    /// Segment label.
    pub fn name(self) -> &'static str {
        match self {
            Method::Barrowman => "Barrowman",
            Method::Extended => "Extended",
            Method::CfdTable => "CFD table",
        }
    }

    /// What the method means, for its tooltip.
    pub fn tip(self) -> &'static str {
        match self {
            Method::Barrowman => "Barrowman (1967): linear normal force and CP at zero angle of attack; the angle of attack is ignored",
            Method::Extended => "Barrowman plus body lift and induced drag at the set angle of attack",
            Method::CfdTable => super::cfd::method_tip(),
        }
    }
}

/// The surface finish a run overrides the document's finishes with.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FinishChoice {
    /// Each part keeps its own finish.
    AsBuilt,
    /// Every part gets this finish.
    All(Finish),
}

impl FinishChoice {
    /// Every choice, in select order.
    pub const ALL: [FinishChoice; 6] = [
        FinishChoice::AsBuilt,
        FinishChoice::All(Finish::Rough),
        FinishChoice::All(Finish::Unfinished),
        FinishChoice::All(Finish::RegularPaint),
        FinishChoice::All(Finish::SmoothPaint),
        FinishChoice::All(Finish::Polished),
    ];
}

impl std::fmt::Display for FinishChoice {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            FinishChoice::AsBuilt => f.write_str("As built"),
            FinishChoice::All(finish) => write!(f, "{} · {:.0} µm", finish.name(), finish.roughness() * 1e6),
        }
    }
}

/// The Conditions block of the left panel (SI; angles in radians).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Conditions {
    /// Mach range of the Mach sweeps.
    pub mach: (f64, f64),
    /// The single Mach of the angle-of-attack sweep.
    pub mach_fixed: f64,
    /// Angle of attack of the Mach sweeps (rad).
    pub alpha: f64,
    /// Largest angle of the angle-of-attack sweep (rad).
    pub alpha_max: f64,
    /// Altitude above the site that sets the Reynolds number (m).
    pub altitude: f64,
    /// Whether the margin uses the burned-out CG instead of the liftoff one.
    pub burnout: bool,
    /// The surface finish.
    pub finish: FinishChoice,
    /// Monte Carlo runs.
    pub runs: usize,
    /// Mean wind of the Monte Carlo (m/s).
    pub wind: f64,
}

impl Default for Conditions {
    fn default() -> Self {
        Conditions {
            mach: restricted_range().unwrap_or((0.1, 3.0)),
            mach_fixed: BALANCE_MACH,
            alpha: 0.0,
            alpha_max: 15f64.to_radians(),
            altitude: 120.0,
            burnout: false,
            finish: FinishChoice::AsBuilt,
            runs: 100,
            wind: 3.0,
        }
    }
}

/// The one-sigma dispersions of the Monte Carlo: wind speed 1 m/s, wind direction 10°, rail tilt 1°, thrust 3 %, drag
/// coefficient 5 %, dry mass 2 %.
pub fn dispersion() -> Dispersion {
    Dispersion { wind_speed: 1.0, wind_dir: 10f64.to_radians(), launch_angle: 1f64.to_radians(), thrust_scale: 0.03, cd_scale: 0.05, mass: 0.02 }
}

/// A configuration of the document made ready for the worker thread.
#[derive(Debug, Clone)]
pub struct ConfigJob {
    /// The configuration's id.
    pub id: ConfigId,
    /// Display name.
    pub name: String,
    /// The vehicle flown in this configuration.
    pub vehicle: Vehicle,
    /// CG at liftoff, motors loaded (m from the tip).
    pub cg_liftoff: f64,
    /// CG with the motors burned out (m from the tip).
    pub cg_burnout: f64,
    /// Each fin set's panel and shear modulus, in the order of `vehicle.fin_sets`.
    pub stocks: Vec<FinStock>,
    /// The flight input (default launch site), `None` when the configuration has no usable motor set-up.
    pub input: Option<SimInput>,
}

/// The configurations [`jobs`] did not turn into jobs.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Left {
    /// Configurations past [`MAX_CONFIGS`].
    pub over_limit: usize,
    /// Configurations that could not be built, as `(name, why)` (`("G64W → H128W", "motor not found")`).
    pub skipped: Vec<(String, String)>,
}

impl Left {
    /// The muted note for the Analyze method area, `1 configuration skipped: G64W → H128W (motor not found)`; `None` when
    /// nothing was skipped.
    pub fn skipped_note(&self) -> Option<String> {
        if self.skipped.is_empty() {
            return None;
        }
        let list = self.skipped.iter().map(|(n, why)| format!("{n} ({why})")).collect::<Vec<_>>().join(", ");
        let n = self.skipped.len();
        Some(format!("{n} configuration{} skipped: {list}", if n == 1 { "" } else { "s" }))
    }
}

/// Every configuration of `rocket` as a [`ConfigJob`], in the document's order: at most [`MAX_CONFIGS`] of them, the active one
/// always among them (and not necessarily first; see `readout::active_run`). Returns the jobs
/// and how many configurations were left out; the error is why nothing could be built (an empty rocket).
pub fn jobs(rocket: &Rocket, motors: &MotorDb, phase: Phase) -> Result<(Vec<ConfigJob>, Left), String> {
    let active = rocket.active_config_id();
    let mut ids: Vec<ConfigId> = rocket.configurations().iter().map(|c| c.id).collect();
    let mut left = Left { over_limit: ids.len().saturating_sub(MAX_CONFIGS), skipped: Vec::new() };
    if ids.len() > MAX_CONFIGS {
        if let Some(at) = ids.iter().position(|&i| i == active).filter(|&at| at >= MAX_CONFIGS) {
            ids.swap(MAX_CONFIGS - 1, at);
        }
        ids.truncate(MAX_CONFIGS);
    }
    let mut out = Vec::new();
    let mut first_error = None;
    for id in ids {
        let mut copy = rocket.clone();
        copy.select_configuration(id);
        let name = rocket.configuration_name(id).unwrap_or_default();
        let built = from_model::vehicle(&copy, motors, phase).and_then(|vehicle| {
            let cg = |with_motor| from_model::balance(&copy, motors, phase, BALANCE_MACH, 0.0, with_motor).map(|b| b.cg);
            Ok((vehicle, cg(true)?, cg(false)?))
        });
        match built {
            Ok((vehicle, cg_liftoff, cg_burnout)) => {
                let stocks = super::flutter::stocks(&copy, &vehicle);
                let input = from_model::sim_input(&copy, motors, phase, Launch::default()).ok();
                out.push(ConfigJob { id, name, vehicle, cg_liftoff, cg_burnout, stocks, input });
            }
            Err(e) => {
                first_error.get_or_insert(format!("{name}: {e}"));
                let why = match e {
                    bowshock_sim::from_model::BridgeError::UnknownMotor(_) => "motor not found".to_string(),
                    other => other.to_string(),
                };
                left.skipped.push((name, why));
            }
        }
    }
    if out.is_empty() {
        Err(first_error.unwrap_or_else(|| "the rocket has no parts".to_string()))
    } else {
        Ok((out, left))
    }
}

/// Everything one run needs, owned so it can move to a worker thread.
#[derive(Debug, Clone)]
pub struct Request {
    /// Which analysis.
    pub analysis: Analysis,
    /// Which method.
    pub method: Method,
    /// The conditions.
    pub conditions: Conditions,
    /// One job per configuration shown.
    pub jobs: Vec<ConfigJob>,
    /// The document's stability thresholds, for the margin plots' lines and the severities.
    pub bands: StabilityBands,
    /// The solved CFD table the [`Method::CfdTable`] drag comes from. Without one that method is computed as Extended.
    pub cfd: Option<Arc<CfdTable>>,
    /// The per-stage drag tables the Dispersion flights use under [`Method::CfdTable`] (empty: Extended drag).
    pub drag_tables: Vec<Option<DragTable>>,
}

/// A Monte Carlo of one configuration.
#[derive(Debug, Clone, PartialEq)]
pub struct Scatter {
    /// The nominal flight: `(t, east, north, altitude)` samples.
    pub nominal: Vec<(f64, f64, f64, f64)>,
    /// Landing points `[east, north]` (m).
    pub landings: Vec<[f64; 2]>,
    /// Apogee of each run (m).
    pub apogees: Vec<f64>,
    /// Statistics over the runs.
    pub stats: MonteCarloStats,
    /// The nominal flight's landing (m).
    pub nominal_landing: [f64; 2],
}

/// The result of one configuration.
#[derive(Debug, Clone, PartialEq)]
pub enum Data {
    /// A Mach sweep.
    Mach(Vec<MachPoint>),
    /// An angle-of-attack sweep.
    Alpha(Vec<AlphaPoint>),
    /// Fin flutter; `None` for a rocket without fins.
    Flutter(Option<FlutterAnalysis>),
    /// A Monte Carlo; `None` when the configuration cannot fly (no motor).
    Dispersion(Option<Scatter>),
}

/// One configuration's result with what the plots and readouts need of its vehicle.
#[derive(Debug, Clone, PartialEq)]
pub struct Run {
    /// The configuration's id.
    pub id: ConfigId,
    /// Display name.
    pub name: String,
    /// CG at liftoff (m from the tip).
    pub cg_liftoff: f64,
    /// CG at burnout (m from the tip).
    pub cg_burnout: f64,
    /// Reference diameter (m): the length of a caliber.
    pub reference_diameter: f64,
    /// The maximum Mach number of the configuration's nominal flight, for the Mach analyses (where the cursor starts).
    pub max_mach: Option<f64>,
    /// The data.
    pub data: Data,
}

impl Run {
    /// The CG the margin uses.
    pub fn cg(&self, burnout: bool) -> f64 {
        if burnout {
            self.cg_burnout
        } else {
            self.cg_liftoff
        }
    }
}

/// The finished analysis.
#[derive(Debug, Clone, PartialEq)]
pub struct Output {
    /// Which analysis.
    pub analysis: Analysis,
    /// Which method.
    pub method: Method,
    /// The conditions it ran with.
    pub conditions: Conditions,
    /// One run per configuration.
    pub runs: Vec<Run>,
    /// How long the computation took.
    pub elapsed: Duration,
    /// The stability thresholds the request carried.
    pub bands: StabilityBands,
    /// The CFD table the drag comes from, when the method is [`Method::CfdTable`].
    pub cfd: Option<CfdInfo>,
}

/// What the screen says about the 3D stability solve an output was made with.
#[derive(Debug, Clone, PartialEq)]
pub struct LiftInfo {
    /// Solved (Mach, angle) points.
    pub points: usize,
    /// `GPU (Luma)` or `CPU`.
    pub backend: String,
    /// When the solve finished (seconds since the Unix epoch).
    pub solved_at: u64,
}

/// Where a CFD table came from and what it left out: the provenance line's content.
#[derive(Debug, Clone, PartialEq)]
pub struct CfdInfo {
    /// Solved Mach numbers inside the plotted range.
    pub nodes: usize,
    /// `GPU (Luma)` or `CPU`.
    pub backend: String,
    /// Why the GPU was not used, when it was not.
    pub fallback: Option<String>,
    /// When the solve finished (seconds since the Unix epoch).
    pub solved_at: u64,
    /// How long it took (s).
    pub solve_seconds: f64,
    /// The grid it was solved on.
    pub grid: CfdGrid,
    /// The 3D stability solve the margin, CP and `CNα` come from, when the table has a usable one.
    pub lift: Option<LiftInfo>,
    /// The table's first Mach node when the plotted range starts below it (the drag there is the first point scaled by Extended's trend).
    pub scaled_below: Option<f64>,
    /// What was not solved as the document has it, and what the plots do not cover.
    pub notes: Vec<String>,
}

/// Spacing of the Mach sweep's samples, and the finer spacing through the transonic range where the drag and CP change fastest.
const MACH_STEP: f64 = 0.025;
const MACH_STEP_TRANSONIC: f64 = 0.01;
/// The transonic range sampled finely.
const TRANSONIC: (f64, f64) = (0.8, 1.3);
/// Samples of the angle-of-attack sweep.
const ALPHA_POINTS: usize = 31;

/// The Mach numbers of a sweep over `range`: every 0.025, and every 0.01 between 0.8 and 1.3 (about 150 for 0.1 to 3.0), in
/// ascending order with the end points included.
pub fn mach_grid(range: (f64, f64)) -> Vec<f64> {
    let (lo, hi) = (range.0.min(range.1), range.0.max(range.1));
    let steps = |from: f64, to: f64, step: f64| -> Vec<f64> {
        let n = ((to - from) / step + 1e-9).floor() as usize;
        (0..=n).map(|k| from + step * k as f64).collect()
    };
    let mut grid = steps(lo, hi, MACH_STEP);
    grid.push(hi);
    let (a, b) = (TRANSONIC.0.max(lo), TRANSONIC.1.min(hi));
    if b > a {
        grid.extend(steps(a, b, MACH_STEP_TRANSONIC));
    }
    grid.sort_by(f64::total_cmp);
    grid.dedup_by(|x, y| (*x - *y).abs() < 1e-6);
    grid
}

fn lerp_drag(a: &DragBreakdown, b: &DragBreakdown, t: f64) -> DragBreakdown {
    let l = |x: f64, y: f64| x + (y - x) * t;
    DragBreakdown {
        friction: l(a.friction, b.friction),
        pressure: l(a.pressure, b.pressure),
        base: l(a.base, b.base),
        fins: l(a.fins, b.fins),
        protuberances: l(a.protuberances, b.protuberances),
        total: l(a.total, b.total),
    }
}

/// The Mach sweep interpolated at `mach` (clamped to the sweep's ends).
pub fn mach_at(points: &[MachPoint], mach: f64) -> Option<MachPoint> {
    let (first, last) = (points.first()?, points.last()?);
    if mach <= first.mach {
        return Some(first.clone());
    }
    if mach >= last.mach {
        return Some(last.clone());
    }
    let i = points.partition_point(|p| p.mach <= mach);
    let (a, b) = (&points[i - 1], &points[i]);
    let t = if b.mach > a.mach { (mach - a.mach) / (b.mach - a.mach) } else { 0.0 };
    let l = |x: f64, y: f64| x + (y - x) * t;
    Some(MachPoint {
        mach,
        margin_cal: l(a.margin_cal, b.margin_cal),
        cp: l(a.cp, b.cp),
        cn_alpha: l(a.cn_alpha, b.cn_alpha),
        drag: lerp_drag(&a.drag, &b.drag, t),
    })
}

/// The angle sweep interpolated at `alpha` (rad).
pub fn alpha_at(points: &[AlphaPoint], alpha: f64) -> Option<AlphaPoint> {
    let (first, last) = (points.first()?, points.last()?);
    if alpha <= first.alpha {
        return Some(first.clone());
    }
    if alpha >= last.alpha {
        return Some(last.clone());
    }
    let i = points.partition_point(|p| p.alpha <= alpha);
    let (a, b) = (&points[i - 1], &points[i]);
    let t = if b.alpha > a.alpha { (alpha - a.alpha) / (b.alpha - a.alpha) } else { 0.0 };
    let l = |x: f64, y: f64| x + (y - x) * t;
    Some(AlphaPoint {
        alpha,
        margin_cal: l(a.margin_cal, b.margin_cal),
        cp: l(a.cp, b.cp),
        cn_alpha: l(a.cn_alpha, b.cn_alpha),
        cn: l(a.cn, b.cn),
        drag: lerp_drag(&a.drag, &b.drag, t),
    })
}

fn apply_finish(vehicle: &mut Vehicle, finish: FinishChoice) {
    if let FinishChoice::All(f) = finish {
        let r = f.roughness();
        vehicle.body.iter_mut().for_each(|b| b.roughness = r);
        vehicle.fin_sets.iter_mut().for_each(|s| s.roughness = r);
    }
}

fn run_of(job: &ConfigJob, vehicle: &Vehicle, data: Data) -> Run {
    Run {
        id: job.id,
        name: job.name.clone(),
        cg_liftoff: job.cg_liftoff,
        cg_burnout: job.cg_burnout,
        reference_diameter: vehicle.reference_diameter,
        max_mach: None,
        data,
    }
}

/// The Barrowman method drops everything that depends on the angle: the sweep keeps its first (zero-angle) values' slopes.
fn linearise(points: Vec<AlphaPoint>) -> Vec<AlphaPoint> {
    let Some(zero) = points.first().cloned() else { return points };
    points
        .into_iter()
        .map(|p| AlphaPoint { margin_cal: zero.margin_cal, cp: zero.cp, cn_alpha: zero.cn_alpha, cn: zero.cn_alpha * p.alpha, drag: zero.drag, ..p })
        .collect()
}

fn flutter(job: &ConfigJob) -> Option<FlutterAnalysis> {
    let input = SimInput { vehicle: job.vehicle.clone(), ..job.input.clone()? };
    FlutterAnalysis::of_flight(job.stocks.clone(), &input.launch, &simulate(&input))
}

fn scatter(job: &ConfigJob, vehicle: &Vehicle, cond: &Conditions, drag_tables: &[Option<DragTable>]) -> Option<Scatter> {
    let input = job.input.as_ref()?;
    let mut input = SimInput { vehicle: vehicle.clone(), ..input.clone() };
    input.launch.wind_speed = cond.wind;
    if !drag_tables.is_empty() {
        input.drag_tables = drag_tables.to_vec();
    }
    let nominal = simulate(&input);
    let mc = monte_carlo(&input, cond.runs.max(2), &dispersion());
    Some(Scatter {
        nominal: nominal.trajectory.samples.iter().map(|s| (s.t, s.position[0], s.position[1], s.altitude)).collect(),
        nominal_landing: nominal.summary.landing_position,
        landings: mc.runs.iter().map(|r| r.landing).collect(),
        apogees: mc.runs.iter().map(|r| r.apogee).collect(),
        stats: mc.stats,
    })
}

/// Runs the analysis of `request`: pure, CPU-bound, meant for a worker thread.
pub fn compute(request: &Request) -> Output {
    let started = std::time::Instant::now();
    let cond = request.conditions;
    // The CFD table method needs a table; without one the request is the Extended method (the caller says which it wanted).
    let table = request.cfd.as_deref().filter(|_| request.method == Method::CfdTable);
    let runs = request
        .jobs
        .iter()
        .map(|job| {
            let mut vehicle = job.vehicle.clone();
            apply_finish(&mut vehicle, cond.finish);
            let data = match request.analysis {
                Analysis::Aerodynamics | Analysis::Drag => {
                    let alpha = if request.method == Method::Barrowman { 0.0 } else { cond.alpha };
                    let cg = if cond.burnout { job.cg_burnout } else { job.cg_liftoff };
                    let sweep = |alpha: f64| -> Vec<MachPoint> {
                        mach_grid(cond.mach).into_iter().flat_map(|m| vs_mach(&vehicle, cg, (m, m), 1, alpha, &Atmosphere::standard(), cond.altitude)).collect()
                    };
                    Data::Mach(match table {
                        // Stability from the build-up method (or the 3D solve), drag from the table plus the build-up's change with the angle.
                        Some(t) => {
                            let zero = (alpha != 0.0).then(|| sweep(0.0));
                            let points = cfd::mach_points(t, sweep(alpha), zero.as_deref());
                            match t.lift.as_ref().filter(|l| cfd::lift_usable(l)) {
                                Some(l) => cfd::lift_mach_points(l, points, cg, vehicle.reference_diameter, &extended_at_2(&vehicle, cg, cond.altitude)),
                                None => points,
                            }
                        }
                        None => sweep(alpha),
                    })
                }
                Analysis::Stability => {
                    let cg = if cond.burnout { job.cg_burnout } else { job.cg_liftoff };
                    let points = vs_alpha(&vehicle, cg, cond.mach_fixed, (0.0, cond.alpha_max), ALPHA_POINTS);
                    Data::Alpha(match table {
                        Some(t) => {
                            let points = cfd::alpha_points(t, cond.mach_fixed, points);
                            match t.lift.as_ref().filter(|l| cfd::lift_usable(l)) {
                                Some(l) => {
                                    cfd::lift_alpha_points(l, cond.mach_fixed, points, cg, vehicle.reference_diameter, &extended_at_2(&vehicle, cg, 0.0))
                                }
                                None => points,
                            }
                        }
                        None if request.method == Method::Barrowman => linearise(points),
                        None => points,
                    })
                }
                Analysis::Flutter => Data::Flutter(flutter(job)),
                Analysis::Dispersion => {
                    let tables: &[Option<DragTable>] = if request.method == Method::CfdTable { &request.drag_tables } else { &[] };
                    Data::Dispersion(scatter(job, &vehicle, &cond, tables))
                }
                // The optimiser has its own worker and never asks for a computation.
                Analysis::Optimize => Data::Mach(Vec::new()),
            };
            let mut run = run_of(job, &vehicle, data);
            if matches!(request.analysis, Analysis::Aerodynamics | Analysis::Drag) {
                run.max_mach =
                    job.input.as_ref().map(|input| simulate(&SimInput { vehicle: vehicle.clone(), ..input.clone() }).summary.max_mach).filter(|m| *m > 0.0);
            }
            run
        })
        .collect();
    let cfd = table.map(|t| cfd_info(t, request.analysis, &cond));
    Output { analysis: request.analysis, method: request.method, conditions: cond, runs, elapsed: started.elapsed(), bands: request.bands, cfd }
}

/// Extended's `CNα` and CP at 2° at any Mach number: what the 3D solve is scaled by below its first node.
fn extended_at_2<'a>(vehicle: &'a Vehicle, cg: f64, altitude: f64) -> impl Fn(f64) -> Option<cfd::ExtStab> + 'a {
    move |m| {
        let p = vs_mach(vehicle, cg, (m, m), 1, cfd::LIFT_ALPHAS[0].to_radians(), &Atmosphere::standard(), altitude).into_iter().next()?;
        Some(cfd::ExtStab { cn_alpha: p.cn_alpha, cp: p.cp })
    }
}

/// What the screen says about the table an output was made with.
fn cfd_info(table: &CfdTable, analysis: Analysis, cond: &Conditions) -> CfdInfo {
    let (first, last) = (table.points[0].mach, table.points[table.points.len() - 1].mach);
    let mut notes = table.notes.clone();
    let mut scaled_below = None;
    let nodes = match analysis {
        Analysis::Stability => {
            if cond.mach_fixed < first - 1e-9 || cond.mach_fixed > last + 1e-9 {
                notes.push(format!("Mach {:.2} is outside the solved range ({first:.2} to {last:.2}); the drag is held at the nearest end.", cond.mach_fixed));
            }
            table.points.len()
        }
        Analysis::Dispersion => table.points.len(),
        _ => {
            let (lo, hi) = (cond.mach.0.min(cond.mach.1), cond.mach.0.max(cond.mach.1));
            let high = hi > last + 1e-9;
            if lo < first - 1e-9 {
                scaled_below = Some(first);
                notes.push(format!("Below Mach {first:.2} the drag is the table's first point scaled by Extended's Mach trend (shaded)."));
            }
            if high {
                notes.push(format!("The table covers up to Mach {last:.2}; the plots stop there."));
            }
            table.points.iter().filter(|p| p.mach >= lo - 1e-9 && p.mach <= hi + 1e-9).count()
        }
    };
    CfdInfo {
        nodes,
        backend: table.backend.clone(),
        fallback: table.fallback.clone(),
        solved_at: table.solved_at,
        solve_seconds: table.solve_seconds,
        grid: table.grid,
        lift: table.lift.as_ref().filter(|l| cfd::lift_usable(l)).map(|l| LiftInfo {
            points: l.points.len(),
            backend: l.backend.clone(),
            solved_at: l.solved_at,
        }),
        scaled_below,
        notes,
    }
}

/// A table of the output's numbers: the table view and the CSV share it.
#[derive(Debug, Clone, PartialEq)]
pub struct Table {
    /// Column headings (with units).
    pub headers: Vec<String>,
    /// Rows of preformatted cells.
    pub rows: Vec<Vec<String>>,
}

impl Table {
    /// The table as CSV (comma-separated, headings first, cells quoted when they contain a comma).
    pub fn csv(&self) -> String {
        let cell = |s: &String| if s.contains([',', '"', '\n']) { format!("\"{}\"", s.replace('"', "\"\"")) } else { s.clone() };
        let line = |cells: &[String]| cells.iter().map(cell).collect::<Vec<_>>().join(",");
        let mut out = line(&self.headers);
        for row in &self.rows {
            out.push('\n');
            out.push_str(&line(row));
        }
        out.push('\n');
        out
    }
}

fn s(v: f64, decimals: usize) -> String {
    format!("{v:.decimals$}")
}

impl Output {
    /// Whether the margin, CP and `CNα` come from the 3D Euler solve (the labels then say `· CFD (3D)`).
    pub fn lift_3d(&self) -> bool {
        self.cfd.as_ref().is_some_and(|c| c.lift.is_some())
    }

    /// What the output has to say in the method line: the samples it holds.
    pub fn points_text(&self) -> String {
        let n = self.runs.first().map_or(0, |r| match &r.data {
            Data::Mach(p) => self.cfd.as_ref().map_or(p.len(), |c| c.nodes),
            Data::Alpha(p) => p.len(),
            Data::Flutter(f) => f.as_ref().map_or(0, |f| f.trace.points.len()),
            Data::Dispersion(d) => d.as_ref().map_or(0, |d| d.landings.len()),
        });
        match self.analysis {
            Analysis::Aerodynamics | Analysis::Drag => {
                let aoa = if self.method == Method::Barrowman { 0.0 } else { self.conditions.alpha.to_degrees() };
                format!("{n} Mach points at α {aoa:.1}°")
            }
            Analysis::Stability => format!("{n} AoA points at Mach {:.2}", self.conditions.mach_fixed),
            Analysis::Flutter => format!("{n} flight samples"),
            Analysis::Dispersion => format!("{n} runs per configuration"),
            Analysis::Optimize => String::new(),
        }
    }

    /// The numbers as a table in `units`: stations in the length unit, altitudes and landing offsets in the altitude unit, speeds
    /// in the speed unit (each heading names its unit), angles in degrees. A column's decimals follow its unit.
    pub fn table(&self, units: UnitSystem) -> Table {
        let q = |quantity: Quantity, si: f64, extra: usize| {
            let unit = quantity.unit_for(units);
            format!("{:.*}", unit.decimals + extra, unit.from_si(si))
        };
        let sym = |quantity: Quantity| quantity.unit_for(units).symbol;
        let mut rows = Vec::new();
        let headers: Vec<String> = match self.analysis {
            Analysis::Aerodynamics | Analysis::Drag => {
                for run in &self.runs {
                    let Data::Mach(points) = &run.data else { continue };
                    let cg = run.cg(self.conditions.burnout);
                    for p in points {
                        let other = p.drag.protuberances;
                        rows.push(vec![
                            run.name.clone(),
                            s(p.mach, 3),
                            s(p.margin_cal, 3),
                            q(Quantity::Length, p.cp, 1),
                            q(Quantity::Length, cg, 1),
                            s(p.cn_alpha, 3),
                            s(p.drag.total, 3),
                            s(p.drag.friction, 3),
                            s(p.drag.pressure, 3),
                            s(p.drag.base, 3),
                            s(p.drag.fins, 3),
                            s(other, 3),
                        ]);
                    }
                }
                let length = sym(Quantity::Length);
                vec![
                    "Configuration".to_string(),
                    "Mach".to_string(),
                    "Margin (cal)".to_string(),
                    format!("CP ({length})"),
                    format!("CG ({length})"),
                    "CNa (/rad)".to_string(),
                    "Cd".to_string(),
                    "Cd skin".to_string(),
                    "Cd press".to_string(),
                    "Cd base".to_string(),
                    "Cd fins".to_string(),
                    "Cd other".to_string(),
                ]
            }
            Analysis::Stability => {
                for run in &self.runs {
                    let Data::Alpha(points) = &run.data else { continue };
                    let cg = run.cg(self.conditions.burnout);
                    for p in points {
                        rows.push(vec![
                            run.name.clone(),
                            s(p.alpha.to_degrees(), 2),
                            s(p.margin_cal, 3),
                            q(Quantity::Length, p.cp, 1),
                            q(Quantity::Length, cg, 1),
                            s(p.cn_alpha, 3),
                            s(p.cn, 4),
                            s(p.drag.total, 4),
                        ]);
                    }
                }
                let length = sym(Quantity::Length);
                vec![
                    "Configuration".to_string(),
                    "AoA (deg)".to_string(),
                    "Margin (cal)".to_string(),
                    format!("CP ({length})"),
                    format!("CG ({length})"),
                    "CNa (/rad)".to_string(),
                    "CN".to_string(),
                    "Cd".to_string(),
                ]
            }
            Analysis::Flutter => {
                for run in &self.runs {
                    let Data::Flutter(Some(f)) = &run.data else { continue };
                    for p in &f.trace.points {
                        let m = p.margin();
                        rows.push(vec![
                            run.name.clone(),
                            s(p.t, 2),
                            q(Quantity::Altitude, p.altitude, 0),
                            q(Quantity::Velocity, p.speed, 0),
                            s(p.mach, 3),
                            q(Quantity::Velocity, p.flutter_velocity, 0),
                            if m.is_finite() { s(m, 2) } else { "-".to_string() },
                            f.stocks[p.governing.min(f.stocks.len() - 1)].name.clone(),
                        ]);
                    }
                }
                vec![
                    "Configuration".to_string(),
                    "Time (s)".to_string(),
                    format!("Altitude ({})", sym(Quantity::Altitude)),
                    format!("Velocity ({})", sym(Quantity::Velocity)),
                    "Mach".to_string(),
                    format!("Flutter velocity ({})", sym(Quantity::Velocity)),
                    "Flutter margin".to_string(),
                    "Governing fin set".to_string(),
                ]
            }
            Analysis::Dispersion => {
                for run in &self.runs {
                    let Data::Dispersion(Some(d)) = &run.data else { continue };
                    for (landing, apogee) in d.landings.iter().zip(&d.apogees) {
                        rows.push(vec![
                            run.name.clone(),
                            q(Quantity::Altitude, landing[0], 1),
                            q(Quantity::Altitude, landing[1], 1),
                            q(Quantity::Altitude, *apogee, 1),
                        ]);
                    }
                }
                vec![
                    "Configuration".to_string(),
                    format!("Landing east ({})", sym(Quantity::Altitude)),
                    format!("Landing north ({})", sym(Quantity::Altitude)),
                    format!("Apogee ({})", sym(Quantity::Altitude)),
                ]
            }
            Analysis::Optimize => Vec::new(),
        };
        Table { headers, rows }
    }
}

/// `BOWSHOCK_CFD_RESTRICT=1.0,1.7` (a diagnostic, for measuring a part of the table through the app): the Mach range a new analysis starts with, and the 3D angles
/// start off (see [`restricted_range`] users). Unset or malformed: `None`.
pub(crate) fn restricted_range() -> Option<(f64, f64)> {
    let v = std::env::var("BOWSHOCK_CFD_RESTRICT").ok()?;
    let (a, b) = v.split_once(',')?;
    Some((a.trim().parse().ok()?, b.trim().parse().ok()?))
}

#[cfg(test)]
mod tests {
    use super::*;

    const EXAMPLE: &[u8] = include_bytes!("../../assets/examples/hpr_l1_cert.bowshock");

    fn example_jobs() -> Vec<ConfigJob> {
        let rocket = Rocket::from_bytes(EXAMPLE).unwrap();
        jobs(&rocket, &MotorDb::bundled(), Phase::Full).unwrap().0
    }

    fn run(analysis: Analysis, method: Method, conditions: Conditions) -> Output {
        compute(&Request { analysis, method, conditions, jobs: example_jobs(), bands: Default::default(), cfd: None, drag_tables: Vec::new() })
    }

    #[test]
    fn flutter_covers_each_fin_copy_with_its_overrides() {
        let mut rocket = Rocket::from_bytes(EXAMPLE).unwrap();
        let motors = MotorDb::bundled();
        let n = |r: &Rocket| jobs(r, &motors, Phase::Full).unwrap().0[0].clone();
        let before = n(&rocket);
        let fins = rocket.radials().iter().find(|r| r.kind.is_fin() && r.count > 1).unwrap().id;
        assert!(rocket.unlink_field(fins, 1, bowshock_model::RadialField::Span));
        let span = rocket.instance_dim(fins, 1, bowshock_model::Dim::Span).unwrap();
        rocket.set_instance_dim(fins, 1, bowshock_model::Dim::Span, span * 2.0).unwrap();
        let after = n(&rocket);
        assert_eq!(after.vehicle.fin_sets.len(), before.vehicle.fin_sets.len() + 1);
        assert_eq!(after.stocks.len(), after.vehicle.fin_sets.len());
        let worst = |j: &ConfigJob| flutter(j).unwrap().min_margin();
        assert!(worst(&after) < worst(&before), "{} vs {}", worst(&after), worst(&before));
    }

    #[test]
    fn the_cfd_info_says_when_the_range_starts_below_the_first_node() {
        use bowshock_model::{CfdPoint, CfdQuality};
        let point = |mach: f64| CfdPoint {
            mach,
            cd: 0.5,
            friction: 0.1,
            pressure: 0.1,
            base: 0.1,
            fins: 0.1,
            other: 0.1,
            interference: 0.0,
            quality: CfdQuality::Converged,
            restart: None,
        };
        let table = CfdTable {
            key: 1,
            geometry: 1,
            stage: 1,
            solved_at: 0,
            solve_seconds: 1.0,
            backend: "CPU".into(),
            fallback: None,
            altitude: 0.0,
            notes: Vec::new(),
            points: vec![point(0.3), point(1.0), point(2.0)],
            grid: CfdGrid::Coarse,
            lift: None,
        };
        let info = |mach: (f64, f64)| cfd_info(&table, Analysis::Drag, &Conditions { mach, ..Conditions::default() });
        let low = info((0.1, 2.0));
        assert_eq!(low.scaled_below, Some(0.3));
        assert!(low.notes.iter().any(|n| n.starts_with("Below Mach 0.30")) && !low.notes.iter().any(|n| n.contains("plots stop")));
        assert_eq!(info((0.3, 2.0)).scaled_below, None);
        let both = info((0.1, 3.0));
        assert_eq!(both.notes.len(), 2);
    }

    #[test]
    fn mach_sweep_matches_the_balance_and_has_a_transonic_drag_rise() {
        let out = run(Analysis::Aerodynamics, Method::Extended, Conditions::default());
        let Data::Mach(points) = &out.runs[0].data else { panic!() };
        assert_eq!(points.len(), mach_grid((0.1, 3.0)).len());
        assert!(points.len() >= 120, "dense enough for smooth curves: {}", points.len());
        assert!((points[0].mach - 0.1).abs() < 1e-12 && (points.last().unwrap().mach - 3.0).abs() < 1e-9);
        // The margin at Mach 0.3 equals the status strip's (same CP, same CG).
        let rocket = Rocket::from_bytes(EXAMPLE).unwrap();
        let balance = from_model::balance(&rocket, &MotorDb::bundled(), Phase::Full, BALANCE_MACH, 0.0, true).unwrap();
        let at = mach_at(points, BALANCE_MACH).unwrap();
        assert!((at.margin_cal - balance.margin_cal).abs() < 0.05, "{} vs {}", at.margin_cal, balance.margin_cal);
        // Drag rises through the transonic range (Niskanen 2009, section 3.4) and the parts sum to the total.
        let peak = points.iter().max_by(|a, b| a.drag.total.total_cmp(&b.drag.total)).unwrap();
        assert!(peak.mach > 0.8 && peak.mach < 1.6, "peak Cd at Mach {}", peak.mach);
        assert!(peak.drag.total > 1.3 * points[0].drag.total);
        let d = peak.drag;
        assert!((d.friction + d.pressure + d.base + d.fins + d.protuberances - d.total).abs() < 1e-9);
    }

    #[test]
    fn barrowman_ignores_the_angle_and_extended_follows_it() {
        let cond = Conditions { alpha: 8f64.to_radians(), mach: (0.3, 0.3), ..Conditions::default() };
        let barrowman = run(Analysis::Aerodynamics, Method::Barrowman, cond);
        let zero = run(Analysis::Aerodynamics, Method::Extended, Conditions { alpha: 0.0, ..cond });
        let extended = run(Analysis::Aerodynamics, Method::Extended, cond);
        let first = |o: &Output| match &o.runs[0].data {
            Data::Mach(p) => p[0].clone(),
            _ => panic!(),
        };
        assert_eq!(first(&barrowman).margin_cal, first(&zero).margin_cal);
        assert!(first(&extended).drag.total > first(&barrowman).drag.total, "induced drag at 8 degrees");
    }

    #[test]
    fn barrowman_aoa_sweep_is_linear_in_normal_force() {
        let out = run(Analysis::Stability, Method::Barrowman, Conditions::default());
        let Data::Alpha(points) = &out.runs[0].data else { panic!() };
        let k = points[1].cn / points[1].alpha;
        assert!(points.iter().skip(1).all(|p| (p.cn / p.alpha - k).abs() < 1e-9) && points.iter().all(|p| p.margin_cal == points[0].margin_cal));
    }

    #[test]
    fn rougher_finish_has_more_skin_friction() {
        let at = |finish| {
            let out = run(Analysis::Aerodynamics, Method::Extended, Conditions { finish, mach: (0.5, 0.5), ..Conditions::default() });
            match &out.runs[0].data {
                Data::Mach(p) => p[0].drag.friction,
                _ => panic!(),
            }
        };
        assert!(at(FinishChoice::All(Finish::Rough)) > at(FinishChoice::All(Finish::Polished)));
    }

    #[test]
    fn flutter_velocity_is_positive_and_the_margin_follows_it() {
        let out = run(Analysis::Flutter, Method::Extended, Conditions::default());
        let Data::Flutter(Some(f)) = &out.runs[0].data else { panic!("the example has fins") };
        assert!(f.trace.points.iter().all(|p| p.flutter_velocity > 50.0 && p.flutter_velocity.is_finite()));
        assert!(f.trace.points.iter().map(|p| p.speed).fold(0.0, f64::max) > 100.0, "the example flies fast");
        assert!((f.min_margin() - f.trace.worst().flutter_velocity / f.trace.worst().speed).abs() < 1e-12);
        // Hand check of the example's plywood fins (root 120 mm, tip 50 mm, 76 mm span, 3.18 mm; G 0.85 GPa): V_f ≈ 169 m/s at
        // Mach 0.53 against 181 m/s, so the margin is a little under 1.
        assert!((0.8..1.1).contains(&f.min_margin()), "{}", f.min_margin());
    }

    #[test]
    fn monte_carlo_lands_near_the_nominal_flight() {
        let out = run(Analysis::Dispersion, Method::Extended, Conditions { runs: 20, ..Conditions::default() });
        let Data::Dispersion(Some(d)) = &out.runs[0].data else { panic!("the example flies") };
        assert_eq!(d.landings.len(), 20);
        assert!(d.stats.apogee_mean > 500.0, "{}", d.stats.apogee_mean);
        assert!(d.apogees.iter().all(|a| (*a - d.stats.apogee_mean).abs() < 0.3 * d.stats.apogee_mean));
        assert!(!d.nominal.is_empty());
    }

    #[test]
    fn dispersion_uses_cfd_drag_table() {
        use bowshock_model::{CfdGrid, CfdPoint, CfdQuality};
        let point = |mach| CfdPoint {
            mach,
            cd: 1.8,
            friction: 0.4,
            pressure: 0.8,
            base: 0.4,
            fins: 0.2,
            other: 0.0,
            interference: 0.0,
            quality: CfdQuality::Converged,
            restart: None,
        };
        let cfd_table = CfdTable {
            key: 1,
            geometry: 1,
            stage: 1,
            solved_at: 0,
            solve_seconds: 0.0,
            backend: "CPU".to_string(),
            fallback: None,
            altitude: 0.0,
            notes: Vec::new(),
            points: [0.1, 0.5, 1.0, 2.0, 3.0].into_iter().map(point).collect(),
            grid: CfdGrid::Coarse,
            lift: None,
        };
        let drag_table = DragTable::from_cfd(&cfd_table);
        let conditions = Conditions { runs: 16, ..Conditions::default() };
        let request = |method, cfd: Option<Arc<CfdTable>>, drag_tables| Request {
            analysis: Analysis::Dispersion,
            method,
            conditions,
            jobs: example_jobs(),
            bands: Default::default(),
            cfd,
            drag_tables,
        };
        let scatter_of = |out: Output| match out.runs[0].data.clone() {
            Data::Dispersion(Some(d)) => d,
            _ => panic!("the example flies"),
        };
        let ext = scatter_of(compute(&request(Method::Extended, None, Vec::new())));
        let cfd = scatter_of(compute(&request(Method::CfdTable, Some(Arc::new(cfd_table)), vec![Some(drag_table)])));
        let apogee = |s: &Scatter| s.nominal.iter().map(|p| p.3).fold(0.0, f64::max);
        assert!(apogee(&cfd) < apogee(&ext), "{} vs {}", apogee(&cfd), apogee(&ext));
        assert!(cfd.stats.apogee_mean < ext.stats.apogee_mean, "{} vs {}", cfd.stats.apogee_mean, ext.stats.apogee_mean);
    }

    #[test]
    fn every_configuration_is_a_run_and_the_table_matches() {
        let mut rocket = Rocket::from_bytes(EXAMPLE).unwrap();
        let before = rocket.configurations().len();
        let copy = rocket.duplicate_configuration(rocket.active_config_id()).unwrap();
        assert!(rocket.configuration(copy).is_some());
        let (jobs, left) = jobs(&rocket, &MotorDb::bundled(), Phase::Full).unwrap();
        assert_eq!((jobs.len(), left), (before + 1, Left::default()));
        let out = compute(&Request {
            analysis: Analysis::Aerodynamics,
            method: Method::Extended,
            conditions: Conditions::default(),
            jobs,
            bands: Default::default(),
            cfd: None,
            drag_tables: Vec::new(),
        });
        let table = out.table(UnitSystem::MM_G_S);
        assert_eq!(table.rows.len(), out.runs.len() * mach_grid((0.1, 3.0)).len());
        assert!(table.rows.iter().all(|r| r.len() == table.headers.len()));
        assert!(table.csv().lines().count() == table.rows.len() + 1);
        // The CSV follows the unit set: stations in inches, the headings say so.
        let inch = out.table(UnitSystem::MM_G_S.with(crate::units::Family::Length, "in"));
        assert_eq!(&inch.headers[3..5], ["CP (in)", "CG (in)"]);
        assert_eq!(&table.headers[3..5], ["CP (mm)", "CG (mm)"]);
        let (mm, ins): (f64, f64) = (table.rows[0][3].parse().unwrap(), inch.rows[0][3].parse().unwrap());
        assert!((mm / 25.4 - ins).abs() < 0.01, "{mm} mm vs {ins} in");
    }

    #[test]
    fn a_configuration_whose_motor_is_unknown_is_skipped_with_a_note() {
        let mut rocket = Rocket::from_bytes(EXAMPLE).unwrap();
        let copy = rocket.duplicate_configuration(rocket.active_config_id()).unwrap();
        let slots: Vec<_> = rocket.configuration(copy).unwrap().motors.clone();
        for slot in slots {
            if let Some(mut m) = slot.motor {
                m.designation = "ZZZ999".to_string();
                rocket.set_motor(copy, slot.mount, Some(m)).unwrap();
            }
        }
        let (_, left) = jobs(&rocket, &MotorDb::bundled(), Phase::Full).unwrap();
        assert_eq!(left.skipped.len(), 1);
        let note = left.skipped_note().unwrap();
        assert!(note.starts_with("1 configuration skipped: ") && note.ends_with("(motor not found)"), "{note}");
    }

    #[test]
    fn jobs_are_named_like_the_status_strip() {
        let rocket = Rocket::from_bytes(EXAMPLE).unwrap();
        let (jobs, _) = jobs(&rocket, &MotorDb::bundled(), Phase::Full).unwrap();
        for job in &jobs {
            assert_eq!(Some(&job.name), rocket.configuration_name(job.id).as_ref());
        }
    }

    #[test]
    fn an_empty_rocket_has_no_jobs() {
        assert!(jobs(&Rocket::default(), &MotorDb::bundled(), Phase::Full).is_err());
    }

    #[test]
    fn csv_quotes_cells_with_commas() {
        let t = Table { headers: vec!["a".into(), "b".into()], rows: vec![vec!["x, y".into(), "z".into()]] };
        assert_eq!(t.csv(), "a,b\n\"x, y\",z\n");
    }
}
