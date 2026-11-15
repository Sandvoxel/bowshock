//! The Analyze screen's physics: which analyses exist, the inputs they take, and the pure function that runs one off the
//! UI thread. Everything here is SI (metres, radians, m/s); the screen formats at the view.
//!
//! # Methods
//!
//! * **Barrowman** is the linear theory of Barrowman (1967): normal force slope and centre of pressure at zero angle of
//!   attack, whatever angle is set (the angle field is ignored; drag is the zero-angle build-up).
//! * **Extended** is Barrowman plus the body lift and induced drag `bowshock-sim` adds at an angle of attack (the
//!   OpenRocket / RASAero style): the margin, CP, `CNα` and drag follow the angle the Conditions set.
//! * **CFD table** (a table of solved coefficients) is a later milestone and cannot be selected.
//!
//! # References
//!
//! Barrowman (1967); Niskanen, *OpenRocket technical documentation* (2013); NACA TN 4197 (Martin, 1958) for the fin flutter
//! velocity.

use std::time::Duration;

use bowshock_model::{ConfigId, Finish, MaterialId, Rocket};
use bowshock_motors::MotorDb;
use bowshock_sim::from_model::{self, Phase};
use bowshock_sim::{
    fin_flutter, monte_carlo, simulate, vs_alpha, vs_mach, AlphaPoint, Atmosphere, Dispersion, DragBreakdown, Launch, MachPoint, MonteCarloStats, SimInput,
    Vehicle,
};

/// The Mach number of the static balance the status strip shows (kept in step with `app::physics`).
pub const BALANCE_MACH: f64 = crate::app::physics::BALANCE_MACH;
/// Static margin below which a rocket is called unstable-ish: the kit's `MIN 1.0` line.
pub const MIN_MARGIN: f64 = from_model::MIN_MARGIN_CAL;
/// Static margin above which a rocket is over-stable and weathercocks: the kit's `OVER-STABLE 3.0` line.
pub const OVER_STABLE: f64 = 3.0;
/// Flutter velocity over maximum velocity below which fin flutter is a risk (the usual 1.5 guideline).
pub const FLUTTER_MIN_MARGIN: f64 = 1.5;
/// Most configurations drawn at once: the four categorical series colours (never cycled).
pub const MAX_CONFIGS: usize = 4;
/// Why the CFD table method cannot be chosen.
pub const CFD_REASON: &str = "CFD solver integration is a later milestone";

/// One of the five analyses of the left list.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Analysis {
    /// Static margin, CP/CG and drag against Mach.
    Aerodynamics,
    /// Static margin, CP/CG and normal force against angle of attack.
    Stability,
    /// Drag by source against Mach.
    Drag,
    /// Fin flutter velocity against altitude.
    Flutter,
    /// Landing scatter and apogee spread of a Monte Carlo of flights.
    Dispersion,
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
        }
    }

    /// The small mono note at the right of the row.
    pub fn hint(self) -> &'static str {
        match self {
            Analysis::Aerodynamics | Analysis::Drag => "vs Mach",
            Analysis::Stability => "vs AoA",
            Analysis::Flutter => "margin",
            Analysis::Dispersion => "Monte Carlo",
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
    /// A solved CFD table (not available yet).
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

    /// What the method means, for its tooltip (or why it is unavailable).
    pub fn tip(self) -> &'static str {
        match self {
            Method::Barrowman => "Barrowman (1967): linear normal force and CP at zero angle of attack; the angle of attack is ignored",
            Method::Extended => "Barrowman plus body lift and induced drag at the set angle of attack (OpenRocket / RASAero style)",
            Method::CfdTable => CFD_REASON,
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
            mach: (0.1, 3.0),
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
    /// Shear modulus of each fin set's material, in the order of `vehicle.fin_sets` (Pa).
    pub shear: Vec<f64>,
    /// The flight input (default launch site), `None` when the configuration has no usable motor set-up.
    pub input: Option<SimInput>,
}

/// Typical shear modulus of fin stock (Pa): handbook values for the bundled materials, 0.3 GPa for anything else.
pub fn shear_modulus(material: MaterialId) -> f64 {
    match material.key() {
        "fiberglass" => 3.0e9,
        "carbon_fiber" => 5.0e9,
        "birch_plywood" | "aircraft_plywood" => 0.62e9,
        "basswood" => 0.4e9,
        "balsa" => 0.15e9,
        "aluminum" => 26e9,
        "steel" => 79e9,
        "brass" => 35e9,
        "pla" => 1.2e9,
        "abs" => 0.9e9,
        "polycarbonate" => 0.8e9,
        _ => 0.3e9,
    }
}

/// Every configuration of `rocket` (the active one first among at most [`MAX_CONFIGS`]) as a [`ConfigJob`]. Returns the jobs
/// and how many configurations were left out; the error is why nothing could be built (an empty rocket).
pub fn jobs(rocket: &Rocket, motors: &MotorDb, phase: Phase) -> Result<(Vec<ConfigJob>, usize), String> {
    let active = rocket.active_config_id();
    let mut ids: Vec<ConfigId> = rocket.configurations().iter().map(|c| c.id).collect();
    let omitted = ids.len().saturating_sub(MAX_CONFIGS);
    if ids.len() > MAX_CONFIGS {
        if let Some(at) = ids.iter().position(|&i| i == active).filter(|&at| at >= MAX_CONFIGS) {
            ids.swap(MAX_CONFIGS - 1, at);
        }
        ids.truncate(MAX_CONFIGS);
    }
    let fin_materials: Vec<f64> =
        rocket.radials().iter().filter(|r| r.kind.fin_span().is_some() && rocket.radial_station(r).is_some()).map(|r| shear_modulus(r.material)).collect();

    let mut out = Vec::new();
    let mut first_error = None;
    for id in ids {
        let mut copy = rocket.clone();
        copy.select_configuration(id);
        let name = copy.active_configuration().display_name();
        let built = from_model::vehicle(&copy, motors, phase).and_then(|vehicle| {
            let cg = |with_motor| from_model::balance(&copy, motors, phase, BALANCE_MACH, 0.0, with_motor).map(|b| b.cg);
            Ok((vehicle, cg(true)?, cg(false)?))
        });
        match built {
            Ok((vehicle, cg_liftoff, cg_burnout)) => {
                let default = fin_materials.first().copied().unwrap_or(0.3e9);
                let shear = if fin_materials.len() == vehicle.fin_sets.len() { fin_materials.clone() } else { vec![default; vehicle.fin_sets.len()] };
                let input = from_model::sim_input(&copy, motors, phase, Launch::default()).ok();
                out.push(ConfigJob { id, name, vehicle, cg_liftoff, cg_burnout, shear, input });
            }
            Err(e) => {
                first_error.get_or_insert(format!("{name}: {e}"));
            }
        }
    }
    if out.is_empty() {
        Err(first_error.unwrap_or_else(|| "the rocket has no parts".to_string()))
    } else {
        Ok((out, omitted))
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
}

/// A fin-flutter curve of one configuration (worst fin set).
#[derive(Debug, Clone, PartialEq)]
pub struct FlutterCurve {
    /// `(altitude m, flutter velocity m/s)`.
    pub velocity: Vec<(f64, f64)>,
    /// The maximum velocity of the nominal flight (m/s).
    pub max_velocity: f64,
    /// Altitude at which it occurs (m).
    pub max_velocity_altitude: f64,
    /// The apogee of the nominal flight (m).
    pub apogee: f64,
}

impl FlutterCurve {
    /// Flutter velocity over the maximum velocity at altitude `h` (infinite when the rocket never moves).
    pub fn margin_at(&self, h: f64) -> f64 {
        let v = interp(&self.velocity, h);
        if self.max_velocity > 0.0 {
            v / self.max_velocity
        } else {
            f64::INFINITY
        }
    }
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
    Flutter(Option<FlutterCurve>),
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
}

/// Spacing of the Mach sweep's samples.
const MACH_STEP: f64 = 0.05;
/// Samples of the angle-of-attack sweep.
const ALPHA_POINTS: usize = 31;
/// Samples of the flutter curve.
const FLUTTER_POINTS: usize = 41;

/// How many Mach samples cover `range`.
pub fn mach_samples(range: (f64, f64)) -> usize {
    (((range.1 - range.0) / MACH_STEP).round() as usize + 1).clamp(2, 400)
}

/// Linear interpolation of `(x, y)` samples at `x`, clamped to the ends.
pub fn interp(points: &[(f64, f64)], x: f64) -> f64 {
    let (Some(first), Some(last)) = (points.first(), points.last()) else { return f64::NAN };
    if x <= first.0 {
        return first.1;
    }
    if x >= last.0 {
        return last.1;
    }
    let i = points.partition_point(|p| p.0 <= x);
    let (a, b) = (points[i - 1], points[i]);
    if b.0 == a.0 {
        b.1
    } else {
        a.1 + (b.1 - a.1) * (x - a.0) / (b.0 - a.0)
    }
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
    Run { id: job.id, name: job.name.clone(), cg_liftoff: job.cg_liftoff, cg_burnout: job.cg_burnout, reference_diameter: vehicle.reference_diameter, data }
}

/// The Barrowman method drops everything that depends on the angle: the sweep keeps its first (zero-angle) values' slopes.
fn linearise(points: Vec<AlphaPoint>) -> Vec<AlphaPoint> {
    let Some(zero) = points.first().cloned() else { return points };
    points
        .into_iter()
        .map(|p| AlphaPoint { margin_cal: zero.margin_cal, cp: zero.cp, cn_alpha: zero.cn_alpha, cn: zero.cn_alpha * p.alpha, drag: zero.drag, ..p })
        .collect()
}

fn flutter(job: &ConfigJob, vehicle: &Vehicle) -> Option<FlutterCurve> {
    if vehicle.fin_sets.is_empty() {
        return None;
    }
    let launch = job.input.as_ref().map(|i| i.launch.clone()).unwrap_or_default();
    let atmosphere = Atmosphere::new(launch.site_altitude, launch.ground_temperature);
    let (max_velocity, max_velocity_altitude, apogee) = match &job.input {
        Some(input) => {
            let flight = simulate(&SimInput { vehicle: vehicle.clone(), ..input.clone() });
            let top = flight.trajectory.samples.iter().filter(|s| s.t <= flight.summary.time_to_apogee).max_by(|a, b| a.speed.total_cmp(&b.speed));
            (flight.summary.max_velocity, top.map_or(0.0, |s| s.altitude), flight.summary.apogee)
        }
        None => (0.0, 0.0, 0.0),
    };
    let top = apogee.max(100.0);
    let velocity: Vec<(f64, f64)> = (0..FLUTTER_POINTS)
        .map(|i| {
            let h = top * i as f64 / (FLUTTER_POINTS - 1) as f64;
            let worst = vehicle
                .fin_sets
                .iter()
                .zip(&job.shear)
                .map(|(fin, &g)| fin_flutter(fin, g, &atmosphere, h, max_velocity).velocity)
                .fold(f64::INFINITY, f64::min);
            (h, worst)
        })
        .collect();
    Some(FlutterCurve { velocity, max_velocity, max_velocity_altitude, apogee })
}

fn scatter(job: &ConfigJob, vehicle: &Vehicle, cond: &Conditions) -> Option<Scatter> {
    let input = job.input.as_ref()?;
    let mut input = SimInput { vehicle: vehicle.clone(), ..input.clone() };
    input.launch.wind_speed = cond.wind;
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
                    Data::Mach(vs_mach(&vehicle, cg, cond.mach, mach_samples(cond.mach), alpha, &Atmosphere::standard(), cond.altitude))
                }
                Analysis::Stability => {
                    let cg = if cond.burnout { job.cg_burnout } else { job.cg_liftoff };
                    let points = vs_alpha(&vehicle, cg, cond.mach_fixed, (0.0, cond.alpha_max), ALPHA_POINTS);
                    Data::Alpha(if request.method == Method::Barrowman { linearise(points) } else { points })
                }
                Analysis::Flutter => Data::Flutter(flutter(job, &vehicle)),
                Analysis::Dispersion => Data::Dispersion(scatter(job, &vehicle, &cond)),
            };
            run_of(job, &vehicle, data)
        })
        .collect();
    Output { analysis: request.analysis, method: request.method, conditions: cond, runs, elapsed: started.elapsed() }
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
    /// What the output has to say in the method line: the samples it holds.
    pub fn points_text(&self) -> String {
        let n = self.runs.first().map_or(0, |r| match &r.data {
            Data::Mach(p) => p.len(),
            Data::Alpha(p) => p.len(),
            Data::Flutter(c) => c.as_ref().map_or(0, |c| c.velocity.len()),
            Data::Dispersion(d) => d.as_ref().map_or(0, |d| d.landings.len()),
        });
        match self.analysis {
            Analysis::Aerodynamics | Analysis::Drag => {
                let aoa = if self.method == Method::Barrowman { 0.0 } else { self.conditions.alpha.to_degrees() };
                format!("{n} Mach points at α {aoa:.1}°")
            }
            Analysis::Stability => format!("{n} AoA points at Mach {:.2}", self.conditions.mach_fixed),
            Analysis::Flutter => format!("{n} altitudes"),
            Analysis::Dispersion => format!("{n} runs per configuration"),
        }
    }

    /// The numbers as a table: lengths in millimetres, angles in degrees, speeds in m/s.
    pub fn table(&self) -> Table {
        let mut rows = Vec::new();
        let headers: Vec<&str> = match self.analysis {
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
                            s(p.cp * 1e3, 1),
                            s(cg * 1e3, 1),
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
                vec!["Configuration", "Mach", "Margin (cal)", "CP (mm)", "CG (mm)", "CNa (/rad)", "Cd", "Cd skin", "Cd press", "Cd base", "Cd fins", "Cd other"]
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
                            s(p.cp * 1e3, 1),
                            s(cg * 1e3, 1),
                            s(p.cn_alpha, 3),
                            s(p.cn, 4),
                            s(p.drag.total, 4),
                        ]);
                    }
                }
                vec!["Configuration", "AoA (deg)", "Margin (cal)", "CP (mm)", "CG (mm)", "CNa (/rad)", "CN", "Cd"]
            }
            Analysis::Flutter => {
                for run in &self.runs {
                    let Data::Flutter(Some(curve)) = &run.data else { continue };
                    for &(h, v) in &curve.velocity {
                        rows.push(vec![run.name.clone(), s(h, 0), s(v, 1), s(curve.max_velocity, 1), s(curve.margin_at(h), 2)]);
                    }
                }
                vec!["Configuration", "Altitude (m)", "Flutter velocity (m/s)", "Max velocity (m/s)", "Flutter margin"]
            }
            Analysis::Dispersion => {
                for run in &self.runs {
                    let Data::Dispersion(Some(d)) = &run.data else { continue };
                    for (landing, apogee) in d.landings.iter().zip(&d.apogees) {
                        rows.push(vec![run.name.clone(), s(landing[0], 1), s(landing[1], 1), s(*apogee, 1)]);
                    }
                }
                vec!["Configuration", "Landing east (m)", "Landing north (m)", "Apogee (m)"]
            }
        };
        Table { headers: headers.into_iter().map(String::from).collect(), rows }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const EXAMPLE: &str = include_str!("../../assets/examples/hpr_l1_cert.bowshock");

    fn example_jobs() -> Vec<ConfigJob> {
        let rocket = Rocket::from_json(EXAMPLE).unwrap();
        jobs(&rocket, &MotorDb::bundled(), Phase::Full).unwrap().0
    }

    fn run(analysis: Analysis, method: Method, conditions: Conditions) -> Output {
        compute(&Request { analysis, method, conditions, jobs: example_jobs() })
    }

    #[test]
    fn mach_sweep_matches_the_balance_and_has_a_transonic_drag_rise() {
        let out = run(Analysis::Aerodynamics, Method::Extended, Conditions::default());
        let Data::Mach(points) = &out.runs[0].data else { panic!() };
        assert_eq!(points.len(), mach_samples((0.1, 3.0)));
        assert!((points[0].mach - 0.1).abs() < 1e-12 && (points.last().unwrap().mach - 3.0).abs() < 1e-9);
        // The margin at Mach 0.3 equals the status strip's (same CP, same CG).
        let rocket = Rocket::from_json(EXAMPLE).unwrap();
        let balance = from_model::balance(&rocket, &MotorDb::bundled(), Phase::Full, BALANCE_MACH, 0.0, true).unwrap();
        let at = mach_at(points, BALANCE_MACH).unwrap();
        assert!((at.margin_cal - balance.margin_cal).abs() < 0.05, "{} vs {}", at.margin_cal, balance.margin_cal);
        // Drag rises through the transonic range (Niskanen 2013, section 3.4) and the parts sum to the total.
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
        let Data::Flutter(Some(curve)) = &out.runs[0].data else { panic!("the example has fins") };
        assert!(curve.velocity.iter().all(|p| p.1 > 50.0 && p.1.is_finite()));
        assert!(curve.max_velocity > 100.0, "the example flies fast: {}", curve.max_velocity);
        assert!((curve.margin_at(curve.max_velocity_altitude) - interp(&curve.velocity, curve.max_velocity_altitude) / curve.max_velocity).abs() < 1e-12);
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
    fn every_configuration_is_a_run_and_the_table_matches() {
        let mut rocket = Rocket::from_json(EXAMPLE).unwrap();
        let before = rocket.configurations().len();
        let copy = rocket.duplicate_configuration(rocket.active_config_id()).unwrap();
        assert!(rocket.configuration(copy).is_some());
        let (jobs, omitted) = jobs(&rocket, &MotorDb::bundled(), Phase::Full).unwrap();
        assert_eq!((jobs.len(), omitted), (before + 1, 0));
        let out = compute(&Request { analysis: Analysis::Aerodynamics, method: Method::Extended, conditions: Conditions::default(), jobs });
        let table = out.table();
        assert_eq!(table.rows.len(), out.runs.len() * mach_samples((0.1, 3.0)));
        assert!(table.rows.iter().all(|r| r.len() == table.headers.len()));
        assert!(table.csv().lines().count() == table.rows.len() + 1);
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

    #[test]
    fn interpolation_clamps_to_the_ends() {
        let p = [(0.0, 0.0), (2.0, 4.0)];
        assert_eq!((interp(&p, 1.0), interp(&p, -1.0), interp(&p, 9.0)), (2.0, 0.0, 4.0));
    }
}
