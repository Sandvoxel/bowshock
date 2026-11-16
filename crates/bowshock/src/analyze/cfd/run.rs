use super::fresh::now;
use super::plan::{lift_key, nodes, table_key, Plan, DRAG_TOLERANCE, LIFT_ALPHAS, SOLVER_CEILING};
use crate::analyze::solve::{Iteration, Outcome, PointDone, Slot, Tracker};
use bowshock_core as solver;
use bowshock_model::{CfdGrid, CfdLiftPoint, CfdLiftTable, CfdPoint, CfdQuality, CfdTable};
use bowshock_sim::{vs_mach, Atmosphere};
use std::ops::ControlFlow;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, PoisonError};
use std::time::Instant;

/// What a solve was asked for.
#[derive(Debug, Clone)]
pub struct Job {
    /// The mapped geometry.
    pub plan: Plan,
    /// Altitude that sets the Reynolds number (m).
    pub altitude: f64,
    /// The Mach numbers to solve, ascending.
    pub nodes: Vec<f64>,
    /// The stack the table is for (first flying stage).
    pub stage: u32,
    /// The grid to solve on.
    pub grid: CfdGrid,
}

/// A message from a running solve (the drag table's or the 3D stability solve's).
#[derive(Debug, Clone)]
pub enum Event {
    /// Where the solve runs, sent first (the GPU's name, or `CPU`).
    Device(String),
    /// The running Mach number (and angle) reports, about ten times a second.
    Iteration(Iteration),
    /// A Mach number (and angle) ended.
    Point(PointDone),
    /// A beat every quarter second from the worker's side, so the clock on screen keeps moving between reports (the grid and
    /// the GPU kernels of a point take seconds to build).
    Tick,
    /// The drag solve is over: the table, or why there is none ([`CANCELLED`] for a cancel).
    Finished(Result<CfdTable, String>),
    /// The 3D stability solve is over: the lift table, or why there is none ([`CANCELLED`] for a cancel).
    LiftFinished(Result<CfdLiftTable, String>),
}

/// What a 3D stability solve was asked for.
#[derive(Debug, Clone)]
pub struct LiftJob {
    /// The mapped geometry.
    pub plan: Plan,
    /// The Mach numbers to solve, ascending ([`lift_nodes`]).
    pub nodes: Vec<f64>,
}

/// Keeps the usable points of a 3D solve: points of quality Failed or with a non-finite value are dropped with a note on
/// `notes`. An error when fewer than two Mach numbers keep a point although two or more were asked for.
pub fn reduce_lift(raw: &[solver::LiftPoint], requested: usize, notes: &mut Vec<String>) -> Result<Vec<CfdLiftPoint>, String> {
    let mut points = Vec::with_capacity(raw.len());
    for p in raw {
        let alpha_deg = p.alpha.to_degrees();
        match quality(p.quality).filter(|_| p.cn.is_finite() && p.x_cp.is_finite()) {
            Some(q) => points.push(CfdLiftPoint { mach: p.mach, alpha_deg, cn: p.cn, x_cp: p.x_cp, quality: q }),
            None => notes.push(format!("3D point Mach {:.2}, {alpha_deg:.0}° did not converge and is left out.", p.mach)),
        }
    }
    let mut machs: Vec<f64> = points.iter().map(|p| p.mach).collect();
    machs.dedup_by(|a, b| (*a - *b).abs() < 1e-9);
    if machs.len() < 2 && requested >= 2 {
        return Err("the 3D solver did not converge at enough Mach numbers".to_string());
    }
    if points.is_empty() {
        return Err("the 3D solver returned no usable point".to_string());
    }
    Ok(points)
}

/// Solves the 3D stability table of `job`: blocking, one Mach number (at both angles) after another. `progress` is given
/// [`Event::Device`] first, then the running run's [`Event::Iteration`]s (with its angle) and, when a Mach number's runs are in,
/// an [`Event::Point`] for each of its angles. A set `cancel` ends the run in flight within one report and the solve with
/// [`CANCELLED`]. The Euler solve ignores altitude, viscosity and roughness.
pub fn solve_lift(job: &LiftJob, cancel: &Arc<AtomicBool>, progress: impl Fn(Event) + Send + Sync + 'static) -> Result<CfdLiftTable, String> {
    let started = Instant::now();
    let send: Outbox = Arc::new(progress);
    let total = job.nodes.len();
    let mut raw: Vec<solver::LiftPoint> = Vec::new();
    let (mut gpu, mut cpu) = (0usize, 0usize);
    let mut backend = solver::Backend::Auto;
    send(Event::Device(device_label()));
    for &mach in &job.nodes {
        if cancel.load(Ordering::Relaxed) {
            return Err(CANCELLED.to_string());
        }
        let mut analysis = solver::Analysis::new(&job.plan.body)
            .map_err(|e| e.to_string())?
            .resolution(solver::Resolution::Coarse)
            .backend(backend)
            .on_progress(sink(started, cancel.clone(), send.clone(), true));
        if let Some(fins) = &job.plan.fins {
            analysis = analysis.fins(fins.clone());
        }
        let polar = match analysis.run_lift(&[mach], &LIFT_ALPHAS) {
            Ok(polar) => polar,
            Err(solver::Error::Cancelled) => return Err(CANCELLED.to_string()),
            Err(e) => return Err(format!("Mach {mach:.2}: {e}")),
        };
        let on_cpu = polar.backend == solver::BackendUsed::Cpu;
        if on_cpu {
            cpu += 1;
            backend = solver::Backend::Cpu;
        } else {
            gpu += 1;
        }
        for p in &polar.points {
            let alpha_deg = p.alpha.to_degrees();
            let lift = quality(p.quality).filter(|_| p.cn.is_finite() && p.x_cp.is_finite()).map(|q| CfdLiftPoint {
                mach: p.mach,
                alpha_deg,
                cn: p.cn,
                x_cp: p.x_cp,
                quality: q,
            });
            send(Event::Point(PointDone {
                at: started.elapsed().as_secs_f64(),
                mach: p.mach,
                alpha_deg: Some(alpha_deg),
                outcome: Outcome::from_quality(lift.map(|l| l.quality)),
                point: None,
                lift,
                iterations: p.iterations,
                cpu: on_cpu,
                fallback: polar.fallback.clone(),
            }));
        }
        raw.extend(polar.points);
    }
    let mut notes = Vec::new();
    let points = reduce_lift(&raw, total, &mut notes)?;
    let backend_name = match (gpu, cpu) {
        (_, 0) => "GPU (Luma)",
        (0, _) => "CPU",
        _ => "GPU (Luma) and CPU",
    };
    Ok(CfdLiftTable {
        key: lift_key(job.plan.geometry, &job.nodes),
        solved_at: now(),
        solve_seconds: started.elapsed().as_secs_f64(),
        backend: backend_name.to_string(),
        points,
        notes,
    })
}

/// The text of a cancelled solve's error.
pub const CANCELLED: &str = "cancelled";

pub(super) fn quality(q: solver::Quality) -> Option<CfdQuality> {
    match q {
        solver::Quality::Converged => Some(CfdQuality::Converged),
        solver::Quality::Stationary => Some(CfdQuality::Stationary),
        solver::Quality::Stalled => Some(CfdQuality::Stalled),
        solver::Quality::Failed => None,
    }
}

/// Drag the solver does not model, from the build-up method: the other fin sets, lugs and rail buttons at `mach`.
pub(super) fn built_up(plan: &Plan, mach: f64, altitude: f64) -> f64 {
    if plan.rest.fin_sets.is_empty() && plan.rest.protuberances.is_empty() {
        return 0.0;
    }
    let d = vs_mach(&plan.rest, 0.0, (mach, mach), 1, 0.0, &Atmosphere::standard(), altitude)[0].drag;
    d.fins + d.protuberances
}

/// Seconds between two [`Event::Iteration`]s (about 10 Hz): the solver reports every 50 iterations, which on a fast GPU is
/// several times a second more than a screen needs.
pub const REPORT_INTERVAL: f64 = 0.1;

/// Whether an iteration report goes on: the first of a run (its iteration did not increase) and one `REPORT_INTERVAL` after the
/// last that went.
pub fn report_due(last_at: f64, last_iteration: usize, now: f64, iteration: usize) -> bool {
    iteration <= last_iteration || now - last_at >= REPORT_INTERVAL - 1e-9
}

/// What a solve sends to the screen.
pub(super) type Outbox = Arc<dyn Fn(Event) + Send + Sync>;

/// The solver's progress callback for a solve: ends the run in flight when `cancel` is set (the solver sees it within 50
/// iterations) and forwards the reports as [`Event::Iteration`]s at most every [`REPORT_INTERVAL`]. `lift`: the run is a 3D one, so
/// its angle is reported.
pub(super) fn sink(
    started: Instant,
    cancel: Arc<AtomicBool>,
    send: Outbox,
    lift: bool,
) -> impl Fn(&solver::Progress) -> ControlFlow<()> + Send + Sync + 'static {
    let last = Mutex::new((f64::NEG_INFINITY, 0usize));
    move |p| {
        if cancel.load(Ordering::Relaxed) {
            return ControlFlow::Break(());
        }
        let at = started.elapsed().as_secs_f64();
        let mut last = last.lock().unwrap_or_else(PoisonError::into_inner);
        if report_due(last.0, last.1, at, p.iteration) {
            *last = (at, p.iteration);
            send(Event::Iteration(Iteration {
                at,
                mach: p.mach,
                alpha_deg: lift.then(|| p.alpha.to_degrees()),
                iteration: p.iteration,
                max_iterations: p.max_iterations,
                residual_orders: p.residual_drop_orders,
                monitor: p.monitor,
                spread: p.spread,
                tolerance: p.tolerance,
            }));
        }
        ControlFlow::Continue(())
    }
}

/// Where the solve runs, for the screen: the GPU's name, or `CPU` when there is no GPU the solver can use.
pub(super) fn device_label() -> String {
    solver::gpu_name().unwrap_or_else(|| "CPU".to_string())
}

/// What a table of `job` is made of when its solve ends, whole or early.
pub(in crate::analyze) struct Tally {
    /// The solved points.
    pub(in crate::analyze) points: Vec<CfdPoint>,
    /// Notes on the table, the plan's first.
    pub(in crate::analyze) notes: Vec<String>,
    /// Points solved on the GPU and on the CPU.
    pub(in crate::analyze) gpu: usize,
    pub(in crate::analyze) cpu: usize,
    /// Why the GPU was not used.
    pub(in crate::analyze) fallback: Option<String>,
}

/// The table of `job` from `tally`. `attempted` is how many of the job's Mach numbers were tried (all of them for a whole solve):
/// the key is the one of those nodes, so a table that stopped early reads as not the table of the whole range (see [`partial_of`]).
pub(in crate::analyze) fn table_from(job: &Job, tally: Tally, attempted: usize, seconds: f64) -> CfdTable {
    let Tally { points, mut notes, gpu, cpu, fallback } = tally;
    let stalled = points.iter().filter(|p| p.quality == CfdQuality::Stalled).count();
    if stalled > 0 {
        notes.push(format!("{stalled} point{} stopped at the iteration cap before the drag settled.", if stalled == 1 { "" } else { "s" }));
    }
    let backend = match (gpu, cpu) {
        (_, 0) => "GPU (Luma)",
        (0, _) => "CPU",
        _ => "GPU (Luma) and CPU",
    };
    let altitude = job.altitude.clamp(0.0, SOLVER_CEILING);
    CfdTable {
        key: table_key(job.plan.geometry, altitude, &job.nodes[..attempted.min(job.nodes.len())]),
        geometry: job.plan.geometry,
        stage: job.stage,
        solved_at: now(),
        solve_seconds: seconds,
        backend: backend.to_string(),
        fallback,
        altitude,
        notes,
        points,
        grid: job.grid,
        lift: None,
    }
}

/// The note for a Mach number the solver gave no point at.
pub(super) fn fallback_note(mach: f64) -> String {
    format!("Mach {mach:.2} did not converge and is interpolated from its neighbours.")
}

/// The table of a solve that was stopped after some points, from what `tracker` saw finish: `None` with fewer than two points
/// (a table needs a curve). `seconds` is how long the solve ran. Its key is that of the Mach numbers tried, so it reads as a
/// stale table for the whole range, and [`partial_of`] says how much of the range it covers.
pub fn partial_table(job: &Job, tracker: &Tracker, seconds: f64) -> Option<CfdTable> {
    let points = tracker.points();
    if points.len() < 2 {
        return None;
    }
    let attempted = tracker.done_count();
    let mut notes = job.plan.notes.clone();
    notes.extend(
        tracker
            .units()
            .iter()
            .enumerate()
            .filter(|(i, _)| matches!(tracker.slot(*i), Slot::Done(d) if d.outcome == Outcome::Fallback))
            .map(|(_, u)| fallback_note(u.mach)),
    );
    notes.push(format!("Stopped early: {attempted} of {} Mach points solved.", job.nodes.len()));
    let (cpu, gpu) = tracker.units_by_device();
    Some(table_from(job, Tally { points, notes, gpu, cpu, fallback: tracker.fallback.clone() }, attempted, seconds))
}

/// How much of the table of the current range `table` is, when it is a table that stopped early: `(solved, of)` Mach numbers,
/// the first `solved` of the `of` that `mach` (the range on screen) asks for. `None` for any other table.
pub fn partial_of(plan: &Result<Plan, String>, table: &CfdTable, altitude: f64, mach: (f64, f64)) -> Option<(usize, usize)> {
    let (plan, nodes) = (plan.as_ref().ok()?, nodes(mach)?);
    (1..nodes.len()).find(|&n| table.key == table_key(plan.geometry, altitude, &nodes[..n])).map(|n| (n, nodes.len()))
}

/// Solves the table of `job`: blocking, one Mach number after another. `progress` is given [`Event::Device`] first, then for each
/// point its [`Event::Iteration`]s (about ten a second; the solver reports every 50 iterations) and an [`Event::Point`] when it
/// ends. A set `cancel` ends the point in flight within one report (a few tens of milliseconds on a GPU) and the solve with
/// [`CANCELLED`]; the points finished by then were all sent. The result carries what the solver reports of its backend, the
/// build-up drag the solver left out, and notes on points that did not converge.
pub fn solve(job: &Job, cancel: &Arc<AtomicBool>, progress: impl Fn(Event) + Send + Sync + 'static) -> Result<CfdTable, String> {
    let started = Instant::now();
    let send: Outbox = Arc::new(progress);
    let altitude = job.altitude.clamp(0.0, SOLVER_CEILING);
    let total = job.nodes.len();
    let mut tally = Tally { points: Vec::with_capacity(total), notes: job.plan.notes.clone(), gpu: 0, cpu: 0, fallback: None };
    let mut backend = solver::Backend::Auto;
    if job.altitude > SOLVER_CEILING {
        tally.notes.push("The solver's atmosphere ends at 20 km; solved there.".to_string());
    }
    send(Event::Device(device_label()));
    for &mach in &job.nodes {
        if cancel.load(Ordering::Relaxed) {
            return Err(CANCELLED.to_string());
        }
        let mut analysis = solver::Analysis::new(&job.plan.body)
            .map_err(|e| e.to_string())?
            .viscous(solver::FlightConditions { atmosphere: solver::Atmosphere::Isa { altitude_m: altitude } })
            .drag_tolerance(DRAG_TOLERANCE)
            .resolution(match job.grid {
                // Coarse is the default, for exploring; Medium (Refine) is about 3.5x slower per point and its Cd about 1 % different.
                CfdGrid::Coarse => solver::Resolution::Coarse,
                CfdGrid::Medium => solver::Resolution::Medium,
            })
            .roughness(job.plan.body_roughness)
            .backend(backend)
            .on_progress(sink(started, cancel.clone(), send.clone(), false));
        if let Some(fins) = &job.plan.fins {
            analysis = analysis.fins(fins.clone());
        }
        let polar = match analysis.run(&[mach]) {
            Ok(polar) => polar,
            Err(solver::Error::Cancelled) => return Err(CANCELLED.to_string()),
            Err(e) => return Err(format!("Mach {mach:.2}: {e}")),
        };
        let cpu = polar.backend == solver::BackendUsed::Cpu;
        if cpu {
            tally.cpu += 1;
            // The GPU did not start: do not try it again for every point.
            backend = solver::Backend::Cpu;
        } else {
            tally.gpu += 1;
        }
        tally.fallback = tally.fallback.or(polar.fallback.clone());
        let p = &polar.points[0];
        let stored = quality(p.quality).filter(|_| p.drag.total.is_finite()).map(|q| {
            let fins = p.drag.fins.map_or(0.0, |f| f.friction + f.pressure);
            let other = built_up(&job.plan, mach, altitude);
            let (friction, base) = (p.drag.friction.unwrap_or(0.0), p.drag.base);
            CfdPoint {
                mach,
                cd: p.drag.forebody_pressure + friction + base + fins + other,
                friction,
                pressure: p.drag.forebody_pressure,
                base,
                fins,
                other,
                quality: q,
            }
        });
        match stored {
            Some(point) => tally.points.push(point),
            None => tally.notes.push(fallback_note(mach)),
        }
        send(Event::Point(PointDone {
            at: started.elapsed().as_secs_f64(),
            mach,
            alpha_deg: None,
            outcome: Outcome::from_quality(stored.map(|s| s.quality)),
            point: stored,
            lift: None,
            iterations: p.detail.iterations,
            cpu,
            fallback: polar.fallback.clone(),
        }));
    }
    if tally.points.len() < 2 && total >= 2 {
        return Err("the solver did not converge at enough Mach numbers".to_string());
    }
    Ok(table_from(job, tally, total, started.elapsed().as_secs_f64()))
}
