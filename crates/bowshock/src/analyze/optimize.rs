//! The design optimiser: moves up to four dimensions of the rocket to reach a goal (apogee, margin, landing distance, mass…)
//! while keeping constraints (margin band, rail-exit speed, Mach, flutter margin, apogee ceiling).
//!
//! Pure and off the UI thread: [`run`] evaluates candidate designs by flying them with the same simulation the Simulate screen
//! uses and searches with the Nelder–Mead downhill simplex (Nelder & Mead, *Computer Journal* 7, 1965; coefficients
//! α = 1, γ = 2, ρ = ½, σ = ½) in the unit box of the variables. Constraints enter as an exterior quadratic penalty
//! (`1e3·Σ violation²`, each violation normalised by its limit), the standard way to give a derivative-free simplex search
//! constraints (Box, *Computer Journal* 8, 1965). The initial simplex is evaluated in parallel.

use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::Arc;

use bowshock_model::{FieldRef, Rocket};
use bowshock_motors::MotorDb;
use bowshock_sim::from_model::{self, Phase};

use super::flutter::{stocks, FlutterAnalysis};
use crate::simulate::flight::{prepare_rocket, Spec};

/// The most variables a problem takes.
pub const MAX_VARIABLES: usize = 4;
/// The bounds of the evaluation budget.
pub const EVALUATIONS: std::ops::RangeInclusive<usize> = 20..=1000;
/// The default evaluation budget.
pub const DEFAULT_EVALUATIONS: usize = 150;
/// Mach at which the static margin is taken (the builder's).
const MARGIN_MACH: f64 = 0.3;
/// Penalty weight of the squared, normalised constraint violations.
const PENALTY: f64 = 1e3;
/// The first simplex step in the unit box.
const STEP: f64 = 0.15;
/// Reflection, expansion, contraction and shrink coefficients.
const ALPHA: f64 = 1.0;
const GAMMA: f64 = 2.0;
const RHO: f64 = 0.5;
const SIGMA: f64 = 0.5;

/// One dimension the search moves, with its range (SI).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Variable {
    /// The part and dimension.
    pub field: FieldRef,
    /// Smallest value (m or rad).
    pub min: f64,
    /// Largest value.
    pub max: f64,
}

/// What the search minimises or aims for.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Goal {
    /// The highest apogee.
    MaxApogee,
    /// The lowest apogee.
    MinApogee,
    /// An apogee (m above the site).
    TargetApogee(f64),
    /// The highest speed before apogee.
    MaxVelocity,
    /// The lowest speed before apogee.
    MinVelocity,
    /// A static margin (calibers).
    TargetMargin(f64),
    /// The landing nearest the pad.
    MinLandingDistance,
    /// The lightest rocket.
    MinMass,
}

impl Goal {
    /// The metric the goal looks at, in the units [`Metrics`] has it.
    fn metric(self, m: &Metrics) -> f64 {
        match self {
            Goal::MaxApogee | Goal::MinApogee | Goal::TargetApogee(_) => m.apogee,
            Goal::MaxVelocity | Goal::MinVelocity => m.max_velocity,
            Goal::TargetMargin(_) => m.margin,
            Goal::MinLandingDistance => m.landing_distance,
            Goal::MinMass => m.mass,
        }
    }

    /// The value to minimise for a metric value.
    fn objective(self, metric: f64) -> f64 {
        match self {
            Goal::MaxApogee | Goal::MaxVelocity => -metric,
            Goal::MinApogee | Goal::MinVelocity | Goal::MinLandingDistance | Goal::MinMass => metric,
            Goal::TargetApogee(t) | Goal::TargetMargin(t) => (metric - t).abs(),
        }
    }
}

/// Limits a design has to keep; `None` is off.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Constraints {
    /// Lowest static margin (calibers).
    pub min_margin: Option<f64>,
    /// Highest static margin (calibers).
    pub max_margin: Option<f64>,
    /// Lowest speed at rail exit (m/s).
    pub min_rail_exit: Option<f64>,
    /// Highest Mach number.
    pub max_mach: Option<f64>,
    /// Lowest fin flutter margin (flutter speed ÷ speed).
    pub min_flutter_margin: Option<f64>,
    /// Highest apogee (m).
    pub max_apogee: Option<f64>,
}

/// A search to run.
#[derive(Debug, Clone)]
pub struct Problem {
    /// The design the search starts from.
    pub rocket: Rocket,
    /// The motors the flight draws on.
    pub motors: Arc<MotorDb>,
    /// The flight every candidate flies.
    pub spec: Spec,
    /// The dimensions to move (1 to [`MAX_VARIABLES`]).
    pub variables: Vec<Variable>,
    /// What to reach.
    pub goal: Goal,
    /// What to keep.
    pub constraints: Constraints,
    /// How many designs to fly at most ([`EVALUATIONS`]).
    pub max_evaluations: usize,
}

/// The numbers of one flown design.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Metrics {
    /// Static margin at liftoff (calibers).
    pub margin: f64,
    /// Liftoff mass (kg).
    pub mass: f64,
    /// Apogee above the site (m).
    pub apogee: f64,
    /// Peak speed before apogee (m/s).
    pub max_velocity: f64,
    /// Peak Mach before apogee.
    pub max_mach: f64,
    /// Speed at rail exit (m/s).
    pub rail_exit: f64,
    /// Distance of the landing from the pad (m).
    pub landing_distance: f64,
    /// Lowest flutter margin; `None` when not asked for or the rocket has no fins.
    pub flutter_margin: Option<f64>,
}

/// What the worker reports.
#[derive(Debug, Clone, PartialEq)]
pub enum Event {
    /// A design was flown.
    Evaluated {
        /// Count of this evaluation, from 0.
        index: usize,
        /// The variables' values as the rocket took them (m or rad).
        values: Vec<f64>,
        /// The goal's metric (apogee, margin…) in SI; `NaN` when the design could not be flown.
        goal: f64,
        /// The value being minimised, penalty included; infinite when the design could not be flown.
        score: f64,
        /// Whether every constraint holds (and the design flew).
        feasible: bool,
        /// Everything measured; `None` when the design could not be flown.
        metrics: Option<Metrics>,
    },
    /// The search ended: the best design's values and why it stopped.
    Done {
        /// The values of the best design.
        best: Vec<f64>,
        /// `evaluation limit`, `converged` or `cancelled`.
        reason: &'static str,
    },
    /// The search could not run.
    Failed(String),
    /// The worker's beat, so the clock on screen moves.
    Tick,
}

/// Why a search step did not produce a score.
enum Stop {
    Limit,
    Cancelled,
}

/// A design flown.
struct Flown {
    values: Vec<f64>,
    metrics: Metrics,
}

/// Flies the design with `values` (SI, one per variable) and measures it. `Err` says why it cannot be flown.
fn fly(problem: &Problem, values: &[f64]) -> Result<Flown, String> {
    let mut rocket = problem.rocket.clone();
    let mut set = Vec::with_capacity(values.len());
    for (v, &value) in problem.variables.iter().zip(values) {
        rocket.set_dim(v.field.part, v.field.dim, value).ok_or("a dimension cannot be set")?;
        set.push(v.field);
    }
    rocket.apply_formulas();
    let actual: Vec<f64> = set.iter().map(|f| rocket.dim(f.part, f.dim).unwrap_or(f64::NAN)).collect();
    let balance = from_model::balance(&rocket, &problem.motors, Phase::Full, MARGIN_MACH, 0.0, true).map_err(|e| e.to_string())?;
    let (input, _delay) = prepare_rocket(rocket.clone(), &problem.motors, &problem.spec)?;
    let flight = bowshock_sim::simulate(&input);
    let s = &flight.summary;
    let flutter_margin = if problem.constraints.min_flutter_margin.is_some() {
        FlutterAnalysis::of_flight(stocks(&rocket, &input.vehicle), &input.launch, &flight).map(|f| f.min_margin())
    } else {
        None
    };
    let metrics = Metrics {
        margin: balance.margin_cal,
        mass: balance.mass,
        apogee: s.apogee,
        max_velocity: s.max_velocity,
        max_mach: s.max_mach,
        rail_exit: s.rail_exit_velocity,
        landing_distance: s.landing.0,
        flutter_margin,
    };
    if !(metrics.apogee.is_finite() && metrics.margin.is_finite()) {
        return Err("the flight gave no result".to_string());
    }
    Ok(Flown { values: actual, metrics })
}

/// The normalised shortfall of `value` against a lower limit (0 when met).
fn below(value: f64, limit: f64) -> f64 {
    if value < limit {
        (limit - value) / if limit.abs() > 0.0 { limit.abs() } else { 1.0 }
    } else {
        0.0
    }
}

/// The normalised excess of `value` over an upper limit (0 when met).
fn above(value: f64, limit: f64) -> f64 {
    if value > limit {
        (value - limit) / if limit.abs() > 0.0 { limit.abs() } else { 1.0 }
    } else {
        0.0
    }
}

/// The score of a measured design (to minimise) and whether it keeps every constraint.
pub fn score(goal: Goal, constraints: &Constraints, m: &Metrics) -> (f64, bool) {
    let mut violations = Vec::new();
    if let Some(l) = constraints.min_margin {
        violations.push(below(m.margin, l));
    }
    if let Some(l) = constraints.max_margin {
        violations.push(above(m.margin, l));
    }
    if let Some(l) = constraints.min_rail_exit {
        violations.push(below(m.rail_exit, l));
    }
    if let Some(l) = constraints.max_mach {
        violations.push(above(m.max_mach, l));
    }
    if let (Some(l), Some(v)) = (constraints.min_flutter_margin, m.flutter_margin) {
        violations.push(below(v, l));
    }
    if let Some(l) = constraints.max_apogee {
        violations.push(above(m.apogee, l));
    }
    let penalty: f64 = PENALTY * violations.iter().map(|v| v * v).sum::<f64>();
    (goal.objective(goal.metric(m)) + penalty, violations.iter().all(|&v| v == 0.0))
}

/// Why `problem` cannot be searched, if it cannot.
pub fn check(problem: &Problem) -> Result<(), String> {
    if problem.variables.is_empty() || problem.variables.len() > MAX_VARIABLES {
        return Err(format!("Choose 1 to {MAX_VARIABLES} variables"));
    }
    for v in &problem.variables {
        if !(v.min.is_finite() && v.max.is_finite() && v.min < v.max) {
            return Err("Each variable needs a minimum below its maximum".to_string());
        }
        if problem.rocket.dim(v.field.part, v.field.dim).is_none() {
            return Err("A variable's part or dimension no longer exists".to_string());
        }
    }
    Ok(())
}

/// The search state shared by the evaluating threads.
struct Search<'a> {
    problem: &'a Problem,
    cancel: &'a AtomicBool,
    emit: &'a (dyn Fn(Event) + Sync),
    count: AtomicUsize,
    limit: usize,
    last_error: std::sync::Mutex<Option<String>>,
}

impl Search<'_> {
    fn value_of(&self, u: &[f64]) -> Vec<f64> {
        self.problem.variables.iter().zip(u).map(|(v, &u)| v.min + u.clamp(0.0, 1.0) * (v.max - v.min)).collect()
    }

    /// Flies the design at `u` in the unit box and reports it. `Err` when the search must stop instead.
    fn eval(&self, u: &[f64]) -> Result<f64, Stop> {
        if self.cancel.load(Ordering::Relaxed) {
            return Err(Stop::Cancelled);
        }
        let index = self.count.fetch_add(1, Ordering::SeqCst);
        if index >= self.limit {
            self.count.fetch_sub(1, Ordering::SeqCst);
            return Err(Stop::Limit);
        }
        let values = self.value_of(u);
        let (event, score) = match fly(self.problem, &values) {
            Ok(flown) => {
                let (score, feasible) = score(self.problem.goal, &self.problem.constraints, &flown.metrics);
                let goal = self.problem.goal.metric(&flown.metrics);
                (Event::Evaluated { index, values: flown.values, goal, score, feasible, metrics: Some(flown.metrics) }, score)
            }
            Err(e) => {
                *self.last_error.lock().unwrap_or_else(|p| p.into_inner()) = Some(e);
                (Event::Evaluated { index, values, goal: f64::NAN, score: f64::INFINITY, feasible: false, metrics: None }, f64::INFINITY)
            }
        };
        (self.emit)(event);
        Ok(score)
    }
}

/// `a + t·(b − a)` clamped into the unit box.
fn along(a: &[f64], b: &[f64], t: f64) -> Vec<f64> {
    a.iter().zip(b).map(|(a, b)| (a + t * (b - a)).clamp(0.0, 1.0)).collect()
}

/// Runs the search, reporting every flown design through `emit`; returns the closing [`Event`] (`Done` or `Failed`).
/// `cancel` is looked at before each design is flown.
pub fn run(problem: &Problem, cancel: &AtomicBool, emit: &(dyn Fn(Event) + Sync)) -> Event {
    if let Err(e) = check(problem) {
        return Event::Failed(e);
    }
    let n = problem.variables.len();
    let search = Search {
        problem,
        cancel,
        emit,
        count: AtomicUsize::new(0),
        limit: problem.max_evaluations.clamp(*EVALUATIONS.start(), *EVALUATIONS.end()),
        last_error: std::sync::Mutex::new(None),
    };
    // The start: the current values, in the unit box; then a step along each axis (the other way when it would leave the box).
    let x0: Vec<f64> = problem
        .variables
        .iter()
        .map(|v| (problem.rocket.dim(v.field.part, v.field.dim).unwrap_or(v.min) - v.min) / (v.max - v.min))
        .map(|u| u.clamp(0.0, 1.0))
        .collect();
    let mut points: Vec<Vec<f64>> = vec![x0.clone()];
    for i in 0..n {
        let mut p = x0.clone();
        p[i] = if p[i] + STEP > 1.0 { p[i] - STEP } else { p[i] + STEP };
        points.push(p.iter().map(|u| u.clamp(0.0, 1.0)).collect());
    }
    let scores: Vec<Result<f64, Stop>> = std::thread::scope(|scope| {
        let handles: Vec<_> = points.iter().map(|p| scope.spawn(|| search.eval(p))).collect();
        handles.into_iter().map(|h| h.join().unwrap_or(Err(Stop::Cancelled))).collect()
    });
    let mut simplex: Vec<(Vec<f64>, f64)> = Vec::with_capacity(n + 1);
    for (p, s) in points.into_iter().zip(scores) {
        match s {
            Ok(s) => simplex.push((p, s)),
            Err(Stop::Cancelled) => return finish(&search, &simplex, "cancelled"),
            Err(Stop::Limit) => {}
        }
    }
    if simplex.iter().all(|(_, s)| !s.is_finite()) {
        let why = search.last_error.lock().unwrap_or_else(|p| p.into_inner()).clone().unwrap_or_else(|| "no design could be flown".to_string());
        return Event::Failed(why);
    }
    let reason = descend(&search, &mut simplex);
    finish(&search, &simplex, reason)
}

fn finish(search: &Search<'_>, simplex: &[(Vec<f64>, f64)], reason: &'static str) -> Event {
    let best = simplex.iter().min_by(|a, b| a.1.total_cmp(&b.1)).map(|(u, _)| search.value_of(u)).unwrap_or_default();
    Event::Done { best, reason }
}

/// The Nelder–Mead iterations on `simplex` (n + 1 scored points of the unit box); returns why it stopped.
fn descend(search: &Search<'_>, simplex: &mut [(Vec<f64>, f64)]) -> &'static str {
    let n = simplex[0].0.len();
    macro_rules! eval {
        ($u:expr) => {
            match search.eval(&$u) {
                Ok(s) => s,
                Err(Stop::Cancelled) => return "cancelled",
                Err(Stop::Limit) => return "evaluation limit",
            }
        };
    }
    loop {
        simplex.sort_by(|a, b| a.1.total_cmp(&b.1));
        let (best, worst) = (simplex[0].1, simplex[simplex.len() - 1].1);
        let diameter = simplex.iter().map(|(p, _)| p.iter().zip(&simplex[0].0).map(|(a, b)| (a - b).powi(2)).sum::<f64>().sqrt()).fold(0.0, f64::max);
        if diameter < 1e-3 || (worst - best).abs() < 1e-6 * best.abs().max(1.0) {
            return "converged";
        }
        if search.cancel.load(Ordering::Relaxed) {
            return "cancelled";
        }
        let last = simplex.len() - 1;
        let centroid: Vec<f64> = (0..n).map(|i| simplex[..last].iter().map(|(p, _)| p[i]).sum::<f64>() / last as f64).collect();
        let worst_point = simplex[last].0.clone();
        let reflected = along(&centroid, &worst_point, -ALPHA);
        let fr = eval!(reflected);
        if fr < simplex[0].1 {
            let expanded = along(&centroid, &worst_point, -GAMMA);
            let fe = eval!(expanded);
            simplex[last] = if fe < fr { (expanded, fe) } else { (reflected, fr) };
        } else if fr < simplex[last - 1].1 {
            simplex[last] = (reflected, fr);
        } else {
            // Outside contraction when the reflection beat the worst point, inside otherwise; either must beat its reference.
            let (target, bound) = if fr < simplex[last].1 { (&reflected, fr) } else { (&worst_point, simplex[last].1) };
            let contracted = along(&centroid, target, RHO);
            let fc = eval!(contracted);
            if fc <= bound {
                simplex[last] = (contracted, fc);
            } else {
                let anchor = simplex[0].0.clone();
                for i in 1..simplex.len() {
                    let p = along(&anchor, &simplex[i].0, SIGMA);
                    let f = eval!(p);
                    simplex[i] = (p, f);
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use bowshock_model::Dim;
    use std::sync::Mutex;

    const EXAMPLE: &[u8] = include_bytes!("../../assets/examples/hpr_l1_cert.bowshock");

    fn problem(goal: Goal, evaluations: usize) -> Problem {
        let rocket = Rocket::from_bytes(EXAMPLE).unwrap();
        let fins = rocket.radials().iter().find(|r| r.kind.is_fin()).map(|r| r.id).unwrap();
        let spec = Spec::of(&rocket.flights()[0]);
        Problem {
            rocket,
            motors: Arc::new(MotorDb::bundled()),
            spec,
            variables: vec![Variable { field: FieldRef::new(fins, Dim::Span), min: 0.03, max: 0.12 }],
            goal,
            constraints: Constraints::default(),
            max_evaluations: evaluations,
        }
    }

    #[test]
    fn the_search_finds_the_fin_span_that_gives_the_target_margin() {
        let p = problem(Goal::TargetMargin(1.5), 60);
        let rows = Mutex::new(Vec::new());
        let end = run(&p, &AtomicBool::new(false), &|e| {
            if let Event::Evaluated { index, metrics, values, .. } = e {
                rows.lock().unwrap().push((index, metrics, values));
            }
        });
        let Event::Done { best, reason } = end else { panic!("{end:?}") };
        let rows = rows.into_inner().unwrap();
        assert!(rows.len() <= 60 && rows.len() >= 3, "{} evaluations ({reason})", rows.len());
        assert!(best[0] >= 0.03 - 1e-9 && best[0] <= 0.12 + 1e-9);
        let (margin, _) =
            rows.iter().filter_map(|(_, m, v)| m.map(|m| (m.margin, v[0]))).min_by(|a, b| (a.0 - 1.5).abs().total_cmp(&(b.0 - 1.5).abs())).unwrap();
        assert!((margin - 1.5).abs() < 0.05, "best margin {margin} ({reason})");
    }

    #[test]
    fn a_cancel_stops_the_search_within_one_evaluation() {
        let p = problem(Goal::MaxApogee, 200);
        let cancel = AtomicBool::new(false);
        let evaluated = AtomicUsize::new(0);
        let end = run(&p, &cancel, &|e| {
            if let Event::Evaluated { .. } = e {
                // Cancel after the third design (the simplex of one variable has two, so this is the first search step).
                if evaluated.fetch_add(1, Ordering::SeqCst) + 1 == 3 {
                    cancel.store(true, Ordering::SeqCst);
                }
            }
        });
        assert!(matches!(end, Event::Done { reason: "cancelled", .. }), "{end:?}");
        assert_eq!(evaluated.load(Ordering::SeqCst), 3);
    }

    #[test]
    fn a_violated_constraint_costs_the_score_and_is_not_feasible() {
        let m = Metrics {
            margin: 0.5,
            mass: 1.0,
            apogee: 1000.0,
            max_velocity: 200.0,
            max_mach: 0.6,
            rail_exit: 15.0,
            landing_distance: 100.0,
            flutter_margin: None,
        };
        let free = score(Goal::MaxApogee, &Constraints::default(), &m);
        let held = score(Goal::MaxApogee, &Constraints { min_margin: Some(1.0), ..Constraints::default() }, &m);
        assert!(free.1 && !held.1 && held.0 > free.0 + 100.0, "{free:?} {held:?}");
    }

    #[test]
    fn problems_without_variables_or_with_empty_ranges_are_refused() {
        let mut p = problem(Goal::MaxApogee, 50);
        p.variables[0].max = p.variables[0].min;
        assert!(matches!(run(&p, &AtomicBool::new(false), &|_| {}), Event::Failed(_)));
        p.variables.clear();
        assert!(check(&p).is_err());
    }
}
