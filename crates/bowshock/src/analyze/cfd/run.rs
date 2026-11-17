use super::fresh::now;
use super::plan::{lift_key, nodes, table_key, Plan, DRAG_TOLERANCE, LIFT_ALPHAS, SOLVER_CEILING};
use super::remote::{self, Failed};
use super::share::{self, Lost, Stop};
use crate::analyze::solve::{Iteration, MachineInfo, MachineState, Outcome, PointDone, Slot, Tracker};
use crate::app::settings::{Compute, Where};
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
    /// Where to solve: this computer and the machines that are on, one or all of them.
    pub compute: Compute,
    /// Solve on this computer's CPU even if it has a usable GPU (Preferences > Compute); ignored for a remote machine.
    pub cpu_only: bool,
}

/// A message from a running solve (the drag table's or the 3D stability solve's).
#[derive(Debug, Clone)]
pub enum Event {
    /// The machines the solve runs on, sent first.
    Machines(Vec<MachineInfo>),
    /// A machine's state changed (a set-up step, ready on a device, skipped, dropped), `at` seconds into the solve.
    Machine {
        /// Its index in [`Event::Machines`].
        index: usize,
        /// Seconds since the solve began.
        at: f64,
        /// What it is now.
        state: MachineState,
    },
    /// A running Mach number (and angle) reports, about ten times a second per machine.
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
    /// Where to solve: this computer and the machines that are on, one or all of them.
    pub compute: Compute,
    /// Solve on this computer's CPU even if it has a usable GPU (Preferences > Compute); ignored for a remote machine.
    pub cpu_only: bool,
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

/// What a machine is during a solve.
enum Session {
    /// This computer, solving on `backend` (it turns to the CPU for good once the GPU does not start).
    Local { backend: solver::Backend },
    /// A remote GPU.
    Remote(Box<remote::Session>),
}

impl Session {
    /// The backend an analysis for this machine asks for: a remote never falls back to the CPU, its failure is the solve's.
    fn backend(&self) -> solver::Backend {
        match self {
            Session::Local { backend } => *backend,
            Session::Remote(_) => solver::Backend::Gpu,
        }
    }
}

/// The machines of a solve, solved on.
struct Fleet<R> {
    done: share::Finished<R>,
    /// Where each machine is.
    places: Vec<Where>,
    /// The device each machine reported (empty for one that never connected).
    devices: Vec<String>,
}

/// Runs `nodes` nodes of a solve on the machines `compute` names, `work(session, machine, node)` solving one. Sends the machines
/// first and each one's state as it changes; the error is the text of the solve's.
fn run_fleet<R: Send + 'static>(
    compute: &Compute,
    cpu_only: bool,
    nodes: usize,
    started: Instant,
    cancel: &Arc<AtomicBool>,
    send: &Outbox,
    work: impl Fn(&mut Session, usize, usize) -> Result<R, Failed> + Send + Sync + 'static,
) -> Result<Fleet<R>, String> {
    let places = compute.places()?;
    send(Event::Machines(places.iter().map(|w| MachineInfo { name: w.name().to_string(), remote: matches!(w, Where::Remote { .. }) }).collect()));
    let devices = Arc::new(Mutex::new(vec![String::new(); places.len()]));
    let connect = {
        let (places, devices, send) = (places.clone(), devices.clone(), send.clone());
        move |m: usize| -> Result<Session, String> {
            let device = match &places[m] {
                Where::This => {
                    let device = if cpu_only { "CPU".to_string() } else { device_label() };
                    send(Event::Machine { index: m, at: started.elapsed().as_secs_f64(), state: MachineState::Ready(device.clone()) });
                    devices.lock().unwrap_or_else(PoisonError::into_inner)[m].clone_from(&device);
                    return Ok(Session::Local { backend: if cpu_only { solver::Backend::Cpu } else { solver::Backend::Auto } });
                }
                Where::Remote { name, host, .. } => remote::connect(host, name, m, started, &send).inspect(|s| {
                    devices.lock().unwrap_or_else(PoisonError::into_inner)[m].clone_from(&s.label);
                }),
            };
            device.map(|s| Session::Remote(Box::new(s)))
        }
    };
    let left = {
        let send = send.clone();
        move |l: &Lost| {
            let state = if l.dropped { MachineState::Dropped(l.why.clone()) } else { MachineState::Skipped(l.why.clone()) };
            send(Event::Machine { index: l.machine, at: started.elapsed().as_secs_f64(), state });
        }
    };
    let finished =
        share::schedule(places.len(), nodes, cancel, Arc::new(connect), Arc::new(move |m, session: &mut Session, node| work(session, m, node)), Arc::new(left));
    match finished {
        Ok(done) => {
            let devices = devices.lock().unwrap_or_else(PoisonError::into_inner).clone();
            Ok(Fleet { done, places, devices })
        }
        Err(Stop::Cancelled) => Err(CANCELLED.to_string()),
        Err(Stop::NoMachineLeft(lost)) => Err(match (places.len(), lost.first()) {
            (1, Some(only)) => only.why.clone(),
            _ => format!(
                "No machine could finish the solve. {}",
                lost.iter().map(|l| format!("{}: {}", places[l.machine].name(), l.why)).collect::<Vec<_>>().join("; ")
            ),
        }),
    }
}

/// One line per machine that left a solve, for the notes of its table.
fn lost_notes<R>(fleet: &Fleet<R>) -> Vec<String> {
    fleet
        .done
        .lost
        .iter()
        .map(|l| {
            let name = fleet.places[l.machine].name();
            if l.dropped {
                format!("{name} was lost during the solve and its Mach number was solved elsewhere: {}", l.why)
            } else {
                format!("{name} could not be used and was left out: {}", l.why)
            }
        })
        .collect()
}

/// What a machine did of a solve, for the backend text of its table.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Part {
    /// Its device (`NVIDIA GeForce RTX 3090`, `<gpu> on <host>`, `CPU`).
    pub device: String,
    /// Another computer over SSH.
    pub remote: bool,
    /// Units it solved.
    pub units: usize,
    /// Of those, on its CPU.
    pub cpu_units: usize,
}

/// What a table says of where it was solved: on this computer alone `GPU (Luma)`, `CPU` or both; with remote machines
/// `GPU (Luma) · RTX 3090 on this computer, RTX 3080 Laptop GPU on omarchy`.
pub fn backend_text(parts: &[Part]) -> String {
    if parts.iter().all(|p| !p.remote) {
        let cpu: usize = parts.iter().map(|p| p.cpu_units).sum();
        let gpu: usize = parts.iter().map(|p| p.units - p.cpu_units).sum();
        return match (gpu, cpu) {
            (_, 0) => "GPU (Luma)",
            (0, _) => "CPU",
            _ => "GPU (Luma) and CPU",
        }
        .to_string();
    }
    let names: Vec<String> = parts.iter().map(|p| if p.remote { p.device.clone() } else { format!("{} on this computer", p.device) }).collect();
    format!("GPU (Luma) · {}", names.join(", "))
}

/// The parts of `fleet`: what each machine that solved a node did, `cpu(result)` telling whether a node ran on a CPU.
fn parts<R>(fleet: &Fleet<R>, cpu: impl Fn(&R) -> bool) -> Vec<Part> {
    (0..fleet.places.len())
        .filter_map(|m| {
            let mine: Vec<&R> = fleet.done.results.iter().filter(|(who, _)| *who == m).map(|(_, r)| r).collect();
            (!mine.is_empty()).then(|| Part {
                device: fleet.devices[m].clone(),
                remote: matches!(fleet.places[m], Where::Remote { .. }),
                units: mine.len(),
                cpu_units: mine.iter().filter(|r| cpu(r)).count(),
            })
        })
        .collect()
}

/// A Mach number of the 3D solve, solved: its points and where.
struct LiftNode {
    points: Vec<solver::LiftPoint>,
    cpu: bool,
}

/// Solves the 3D stability table of `job`: blocking. Every Mach number (at both angles, the second warm-started from the first)
/// is one node, solved on one machine; with several machines on, they take the next node as they become free. `progress` is given
/// [`Event::Machines`] first, then each machine's [`Event::Iteration`]s (with its angle) and, when a Mach number's runs are in, an
/// [`Event::Point`] for each of its angles. A set `cancel` ends the solve with [`CANCELLED`] at once and the runs in flight within
/// one report. The Euler solve ignores altitude, viscosity and roughness.
pub fn solve_lift(job: &LiftJob, cancel: &Arc<AtomicBool>, progress: impl Fn(Event) + Send + Sync + 'static) -> Result<CfdLiftTable, String> {
    let started = Instant::now();
    let send: Outbox = Arc::new(progress);
    let total = job.nodes.len();
    let work = {
        let (job, cancel, send) = (Arc::new(job.clone()), cancel.clone(), send.clone());
        move |session: &mut Session, machine: usize, node: usize| -> Result<LiftNode, Failed> {
            let mach = job.nodes[node];
            let mut analysis = solver::Analysis::new(&job.plan.body)
                .map_err(|e| Failed::Message(e.to_string()))?
                .resolution(solver::Resolution::Coarse)
                .backend(session.backend());
            if let Some(fins) = &job.plan.fins {
                analysis = analysis.fins(fins.clone());
            }
            let progress = sink(started, cancel.clone(), send.clone(), true, machine);
            let polar = match session {
                Session::Remote(r) => analysis.spec().map_err(|e| Failed::local(mach, e)).and_then(|spec| r.lift(&spec, mach, &LIFT_ALPHAS, &progress))?,
                Session::Local { backend } => {
                    let polar = analysis.on_progress(progress).run_lift(&[mach], &LIFT_ALPHAS).map_err(|e| Failed::local(mach, e))?;
                    if polar.backend == solver::BackendUsed::Cpu {
                        // The GPU did not start: do not try it again for every point.
                        *backend = solver::Backend::Cpu;
                    }
                    polar
                }
            };
            let cpu = polar.backend == solver::BackendUsed::Cpu;
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
                    machine,
                    at: started.elapsed().as_secs_f64(),
                    mach: p.mach,
                    alpha_deg: Some(alpha_deg),
                    outcome: Outcome::from_quality(lift.map(|l| l.quality)),
                    point: None,
                    lift,
                    iterations: p.iterations,
                    cpu,
                    fallback: polar.fallback.clone(),
                }));
            }
            Ok(LiftNode { points: polar.points, cpu })
        }
    };
    let fleet = run_fleet(&job.compute, job.cpu_only, total, started, cancel, &send, work)?;
    let mut notes = lost_notes(&fleet);
    let raw: Vec<solver::LiftPoint> = fleet.done.results.iter().flat_map(|(_, n)| n.points.iter().cloned()).collect();
    let points = reduce_lift(&raw, total, &mut notes)?;
    Ok(CfdLiftTable {
        key: lift_key(job.plan.geometry, &job.nodes),
        solved_at: now(),
        solve_seconds: started.elapsed().as_secs_f64(),
        backend: backend_text(&parts(&fleet, |n| n.cpu)),
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
/// iterations) and forwards the reports as [`Event::Iteration`]s of `machine` at most every [`REPORT_INTERVAL`]. `lift`: the run is
/// a 3D one, so its angle is reported.
pub(super) fn sink(
    started: Instant,
    cancel: Arc<AtomicBool>,
    send: Outbox,
    lift: bool,
    machine: usize,
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
                machine,
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
    // Machines that share the work finish out of order: the table keeps the Mach numbers from the first up to the first gap.
    let points = tracker.prefix_points();
    if points.len() < 2 {
        return None;
    }
    let attempted = tracker.done_prefix();
    let mut notes = job.plan.notes.clone();
    notes.extend(
        tracker
            .units()
            .iter()
            .enumerate()
            .take(attempted)
            .filter(|(i, _)| matches!(tracker.slot(*i), Slot::Done(d) if d.outcome == Outcome::Fallback))
            .map(|(_, u)| fallback_note(u.mach)),
    );
    notes.push(format!("Stopped early: {attempted} of {} Mach points solved.", job.nodes.len()));
    let parts: Vec<Part> = tracker
        .prefix_machines()
        .into_iter()
        .filter_map(|(m, units, cpu_units)| {
            let row = tracker.machines().get(m)?;
            Some(Part { device: row.device().unwrap_or_default().to_string(), remote: row.remote, units, cpu_units })
        })
        .collect();
    let (cpu, gpu) = (parts.iter().map(|p| p.cpu_units).sum(), parts.iter().map(|p| p.units - p.cpu_units).sum());
    let mut table = table_from(job, Tally { points, notes, gpu, cpu, fallback: tracker.fallback.clone() }, attempted, seconds);
    if parts.iter().any(|p| p.remote) {
        table.backend = backend_text(&parts);
    }
    Some(table)
}

/// How much of the table of the current range `table` is, when it is a table that stopped early: `(solved, of)` Mach numbers,
/// the first `solved` of the `of` that `mach` (the range on screen) asks for. `None` for any other table.
pub fn partial_of(plan: &Result<Plan, String>, table: &CfdTable, altitude: f64, mach: (f64, f64)) -> Option<(usize, usize)> {
    let (plan, nodes) = (plan.as_ref().ok()?, nodes(mach)?);
    (1..nodes.len()).find(|&n| table.key == table_key(plan.geometry, altitude, &nodes[..n])).map(|n| (n, nodes.len()))
}

/// A Mach number of the drag table, solved: its point (none when the solver gave none) and where.
struct DragNode {
    point: Option<CfdPoint>,
    cpu: bool,
    fallback: Option<String>,
}

/// Solves the table of `job`: blocking. Every Mach number is one node, solved on one machine; with several machines on they take
/// the next node as they become free. `progress` is given [`Event::Machines`] first, then for each point its [`Event::Iteration`]s
/// (about ten a second per machine; the solver reports every 50 iterations) and an [`Event::Point`] when it ends. A set `cancel`
/// ends the solve with [`CANCELLED`] at once and the points in flight within one report (a few tens of milliseconds on a GPU); the
/// points finished by then were all sent. The result carries which machines solved it, the build-up drag the solver left out, and
/// notes on points that did not converge and machines that were left out.
pub fn solve(job: &Job, cancel: &Arc<AtomicBool>, progress: impl Fn(Event) + Send + Sync + 'static) -> Result<CfdTable, String> {
    let started = Instant::now();
    let send: Outbox = Arc::new(progress);
    let altitude = job.altitude.clamp(0.0, SOLVER_CEILING);
    let total = job.nodes.len();
    let mut notes = job.plan.notes.clone();
    if job.altitude > SOLVER_CEILING {
        notes.push("The solver's atmosphere ends at 20 km; solved there.".to_string());
    }
    let work = {
        let (job, cancel, send) = (Arc::new(job.clone()), cancel.clone(), send.clone());
        move |session: &mut Session, machine: usize, node: usize| -> Result<DragNode, Failed> {
            let mach = job.nodes[node];
            let mut analysis = solver::Analysis::new(&job.plan.body)
                .map_err(|e| Failed::Message(e.to_string()))?
                .viscous(solver::FlightConditions { atmosphere: solver::Atmosphere::Isa { altitude_m: altitude } })
                .drag_tolerance(DRAG_TOLERANCE)
                .resolution(match job.grid {
                    // Coarse is the default, for exploring; Medium (Refine) is about 3.5x slower per point and its Cd about 1 % different.
                    CfdGrid::Coarse => solver::Resolution::Coarse,
                    CfdGrid::Medium => solver::Resolution::Medium,
                })
                .roughness(job.plan.body_roughness)
                .backend(session.backend());
            if let Some(fins) = &job.plan.fins {
                analysis = analysis.fins(fins.clone());
            }
            let progress = sink(started, cancel.clone(), send.clone(), false, machine);
            let polar = match session {
                Session::Remote(r) => analysis.spec().map_err(|e| Failed::local(mach, e)).and_then(|spec| r.drag(&spec, mach, &progress))?,
                Session::Local { backend } => {
                    let polar = analysis.on_progress(progress).run(&[mach]).map_err(|e| Failed::local(mach, e))?;
                    if polar.backend == solver::BackendUsed::Cpu {
                        // The GPU did not start: do not try it again for every point.
                        *backend = solver::Backend::Cpu;
                    }
                    polar
                }
            };
            let cpu = polar.backend == solver::BackendUsed::Cpu;
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
            send(Event::Point(PointDone {
                machine,
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
            Ok(DragNode { point: stored, cpu, fallback: polar.fallback })
        }
    };
    let fleet = run_fleet(&job.compute, job.cpu_only, total, started, cancel, &send, work)?;
    let mut tally = Tally { points: Vec::with_capacity(total), notes, gpu: 0, cpu: 0, fallback: None };
    tally.notes.extend(lost_notes(&fleet));
    for (&mach, (_, node)) in job.nodes.iter().zip(&fleet.done.results) {
        if node.cpu {
            tally.cpu += 1;
        } else {
            tally.gpu += 1;
        }
        tally.fallback = tally.fallback.take().or_else(|| node.fallback.clone());
        match node.point {
            Some(point) => tally.points.push(point),
            None => tally.notes.push(fallback_note(mach)),
        }
    }
    if tally.points.len() < 2 && total >= 2 {
        return Err("the solver did not converge at enough Mach numbers".to_string());
    }
    let mut table = table_from(job, tally, total, started.elapsed().as_secs_f64());
    table.backend = backend_text(&parts(&fleet, |n| n.cpu));
    Ok(table)
}
