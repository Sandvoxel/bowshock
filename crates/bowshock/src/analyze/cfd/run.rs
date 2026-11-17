use super::fields::FieldRef;
use super::fresh::now;
use super::plan::{lift_key, nodes, table_key, Plan, DRAG_TOLERANCE, LIFT_ALPHAS, SHARED_GRID, SOLVER_CEILING};
use super::remote::{self, Failed};
use super::share::{self, Lost, Stop};
use super::wording;
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
    /// The Mach numbers of the stability part, solved at [`LIFT_ALPHAS`] ([`lift_nodes`]), ascending; empty: the drag table alone.
    pub stability: Vec<f64>,
}

impl Job {
    /// The units the screen tracks for it (see [`Tracker::solve_units`]).
    pub fn units(&self) -> Vec<crate::analyze::solve::Unit> {
        Tracker::solve_units(&self.nodes, &self.stability, &LIFT_ALPHAS)
    }

    /// Work items for the machines: one per drag node, then one per stability node (its angles run warm one after the other).
    pub fn work_items(&self) -> usize {
        self.nodes.len() + self.stability.len()
    }
}

/// A message from a running solve (the drag table, and its 3D stability part when it has one).
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
    /// A picture of the flow of the point in flight, every few hundred iterations (the first of a point is the grid with the free
    /// stream on it). A remote machine sends none.
    Field(FieldRef),
    /// The final flow field of a point (the Mach number is the field's), sent before its [`Event::Point`].
    PointField {
        /// The field, on the grid the point's drag was computed on.
        field: FieldRef,
    },
    /// A Mach number (and angle) ended.
    Point(PointDone),
    /// A beat every quarter second from the worker's side, so the clock on screen keeps moving between reports (the grid and
    /// the GPU kernels of a point take seconds to build).
    Tick,
    /// The solve is over: the table (with its 3D stability part), or why there is none ([`CANCELLED`] for a cancel).
    Finished(Result<CfdTable, String>),
}

/// The stored point of a 3D result: `None` for a point of quality Failed, without forces or with a non-finite normal force or centre
/// of pressure. The Euler solve has no drag, so `cd` is empty.
fn lift_point(p: &solver::PolarPoint) -> Option<CfdLiftPoint> {
    let f = p.forces?;
    quality(p.quality).filter(|_| f.cn.is_finite() && f.x_cp.is_finite()).map(|q| CfdLiftPoint {
        mach: p.mach,
        alpha_deg: p.alpha_deg,
        cn: f.cn,
        x_cp: f.x_cp,
        cd: None,
        cm: f.cm.is_finite().then_some(f.cm),
        quality: q,
    })
}

/// Keeps the usable points of a 3D solve: points of quality Failed or with a non-finite value are dropped with a note on
/// `notes`. An error when fewer than two Mach numbers keep a point although two or more were asked for.
pub fn reduce_lift(raw: &[solver::PolarPoint], requested: usize, notes: &mut Vec<String>) -> Result<Vec<CfdLiftPoint>, String> {
    let mut points = Vec::with_capacity(raw.len());
    for p in raw {
        match lift_point(p) {
            Some(l) => points.push(l),
            None => notes.push(wording::angle_left_out(p.mach, p.alpha_deg)),
        }
    }
    let mut machs: Vec<f64> = points.iter().map(|p| p.mach).collect();
    machs.dedup_by(|a, b| (*a - *b).abs() < 1e-9);
    if machs.len() < 2 && requested >= 2 {
        return Err(wording::angles_few().to_string());
    }
    if points.is_empty() {
        return Err(wording::angles_none().to_string());
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
    let parts: Vec<Part> = tracker
        .prefix_machines()
        .into_iter()
        .filter_map(|(m, units, cpu_units)| {
            let row = tracker.machines().get(m)?;
            Some(Part { device: row.device().unwrap_or_default().to_string(), remote: row.remote, units, cpu_units })
        })
        .collect();
    // The 3D angles: the leading Mach numbers with every angle in are kept (like the drag points they read as stale for the whole
    // range); a Mach number with only some of its angles is left out, and says so.
    let (lift_points, lift_kept, half) = tracker.stability_kept();
    let mut stability = None;
    if job.stability.is_empty() {
        notes.push(wording::stopped_early(attempted, job.nodes.len(), None));
    } else {
        notes.push(wording::stopped_early(attempted, job.nodes.len(), Some((lift_kept, job.stability.len()))));
        notes.extend(half.map(wording::half_angle_note));
        if lift_kept > 0 {
            stability = Some(CfdLiftTable {
                key: lift_key(job.plan.geometry, &job.stability[..lift_kept]),
                solved_at: now(),
                solve_seconds: seconds,
                backend: backend_text(&parts),
                points: lift_points,
                notes: Vec::new(),
            });
        }
    }
    let (cpu, gpu) = (parts.iter().map(|p| p.cpu_units).sum(), parts.iter().map(|p| p.units - p.cpu_units).sum());
    let mut table = table_from(job, Tally { points, notes, gpu, cpu, fallback: tracker.fallback.clone() }, attempted, seconds);
    if parts.iter().any(|p| p.remote) {
        table.backend = backend_text(&parts);
    }
    table.lift = stability;
    Some(table)
}

/// How much of the table of the current range `table` is, when it is a table that stopped early: `(solved, of)` Mach numbers,
/// the first `solved` of the `of` that `mach` (the range on screen) asks for. `None` for any other table.
pub fn partial_of(plan: &Result<Plan, String>, table: &CfdTable, altitude: f64, mach: (f64, f64)) -> Option<(usize, usize)> {
    let (plan, nodes) = (plan.as_ref().ok()?, nodes(mach)?);
    (1..nodes.len()).find(|&n| table.key == table_key(plan.geometry, altitude, &nodes[..n])).map(|n| (n, nodes.len()))
}

/// Whether the points of a table share one grid, sized for the table's highest Mach number ([`solver::Analysis::grid_mach`]): the 3D engine's wall-resolved grids are
/// sized from the free stream's Reynolds number, so one table shares one grid shape (and the kernels compiled for it); the legacy 2D engine's grids stay per point
/// ([`SHARED_GRID`]).
pub(super) fn shared_grid() -> bool {
    SHARED_GRID || solver::Engine::from_env() == solver::Engine::Solver3
}

/// Whether the 3D engine (`BOWSHOCK_ENGINE=3d`) solves this process's nodes: it has no 2D grid to show a picture of before the first snapshot.
fn solves_3d() -> bool {
    solver::Engine::from_env() == solver::Engine::Solver3
}

/// What a node needs solved.
#[derive(Clone, Copy, PartialEq, Eq)]
enum NodeKind {
    /// A Mach number at zero incidence.
    Drag,
    /// A Mach number at the angles of attack.
    Lift,
}

/// The [`solver::Analysis`] of one node of either kind: the body, the fins, the resolution (a drag node's is the job's grid, the angles are always solved coarse), the
/// backend and the fields, and, for an engine that solves the viscous terms, the flight conditions, the wall roughness, the drag tolerance (a drag node's stop rule)
/// and the shared grid. The legacy engine's angle solver ignores the viscous settings; the 3D engine reads them for both kinds. An engine that does not solve them
/// yet is not asked (it would fail loudly instead of running Euler).
fn node_analysis(job: &Job, kind: NodeKind, session: &Session, altitude: f64, grid_mach: f64, shared_grid: bool) -> Result<solver::Analysis, Failed> {
    let mut analysis = solver::Analysis::new(&job.plan.body)
        .map_err(|e| Failed::Message(e.to_string()))?
        .resolution(match (kind, job.grid) {
            // Coarse is the default, for exploring; Medium (Refine) is about 3.5x slower per point and its Cd about 1 % different.
            (NodeKind::Lift, _) | (_, CfdGrid::Coarse) => solver::Resolution::Coarse,
            (NodeKind::Drag, CfdGrid::Medium) => solver::Resolution::Medium,
        })
        .backend(session.backend())
        .keep_fields(true);
    analysis = analysis.viscous(solver::FlightConditions { atmosphere: solver::Atmosphere::Isa { altitude_m: altitude } }).roughness(job.plan.body_roughness);
    if kind == NodeKind::Drag {
        analysis = analysis.drag_tolerance(DRAG_TOLERANCE);
    }
    if shared_grid {
        analysis = analysis.grid_mach(grid_mach);
    }
    if let Some(fins) = &job.plan.fins {
        analysis = analysis.fins(fins.clone()).fin_outline(job.plan.outline.clone());
    }
    Ok(analysis)
}

/// A Mach number of the drag table, solved: its point (none when the solver gave none) and where.
struct DragNode {
    point: Option<CfdPoint>,
    cpu: bool,
    fallback: Option<String>,
}

/// A Mach number of the stability part, solved at every angle: its points and where.
struct LiftNode {
    points: Vec<solver::PolarPoint>,
    cpu: bool,
}

/// What a machine solved of a work item (see [`Job::work_items`]).
enum Node {
    Drag(DragNode),
    Lift(LiftNode),
}

impl Node {
    fn cpu(&self) -> bool {
        match self {
            Node::Drag(n) => n.cpu,
            Node::Lift(n) => n.cpu,
        }
    }
}

/// Solves the stability node at `mach`: every angle of [`LIFT_ALPHAS`] in one `run` (the second warm-started from the first), on the
/// coarse grid, with the live snapshots of the running angle and the final field of each. The Euler solve ignores altitude,
/// viscosity and roughness.
fn solve_lift_node(
    job: &Job,
    grid_mach: f64,
    shared_grid: bool,
    mach: f64,
    session: &mut Session,
    machine: usize,
    started: Instant,
    cancel: &Arc<AtomicBool>,
    send: &Outbox,
) -> Result<LiftNode, Failed> {
    let mut analysis = node_analysis(job, NodeKind::Lift, session, job.altitude.clamp(0.0, SOLVER_CEILING), grid_mach, shared_grid)?;
    let progress = sink(started, cancel.clone(), send.clone(), true, machine);
    let conditions: Vec<solver::Condition> = LIFT_ALPHAS.iter().map(|&a| solver::Condition::new(mach, a)).collect();
    let mut polar = match session {
        Session::Remote(r) => analysis.spec().map_err(|e| Failed::local(mach, e)).and_then(|spec| r.solve(&spec, &conditions, &progress))?,
        Session::Local { backend } => {
            // A picture of the flow every few hundred iterations; a remote worker brings back only the final fields.
            let live = send.clone();
            analysis = analysis.on_field3(solver::field::SNAPSHOT_INTERVAL, move |f| live(Event::Field(FieldRef::of(f))));
            let polar = analysis.on_progress(progress).run(&conditions).map_err(|e| Failed::local(mach, e))?;
            if polar.backend == solver::BackendUsed::Cpu {
                // The GPU did not start: do not try it again for every point.
                *backend = solver::Backend::Cpu;
            }
            polar
        }
    };
    let cpu = polar.backend == solver::BackendUsed::Cpu;
    for p in &mut polar.points {
        let lift = lift_point(p);
        // the final field of each angle goes first, then the point (the viewer and the store see the field before the number)
        if let Some(field) = p.field3.take() {
            send(Event::PointField { field: FieldRef::of(field) });
        }
        send(Event::Point(PointDone {
            machine,
            at: started.elapsed().as_secs_f64(),
            mach: p.mach,
            alpha_deg: Some(p.alpha_deg),
            outcome: Outcome::from_quality(lift.map(|l| l.quality)),
            point: None,
            lift,
            iterations: p.detail.iterations,
            cpu,
            fallback: polar.fallback.clone(),
        }));
    }
    Ok(LiftNode { points: polar.points, cpu })
}

/// Solves the table of `job`: blocking. Every Mach number of the drag table is one node, then every Mach number of the stability part
/// (both angles, see [`solve_lift_node`]); each is solved on one machine, and with several machines on they take the next node as they
/// become free. `progress` is given [`Event::Machines`] first, then for each point its [`Event::Iteration`]s
/// (about ten a second per machine; the solver reports every 50 iterations) and an [`Event::Point`] when it ends. A set `cancel`
/// ends the solve with [`CANCELLED`] at once and the points in flight within one report (a few tens of milliseconds on a GPU); the
/// points finished by then were all sent. The result carries which machines solved it, the build-up drag the solver left out, and
/// notes on points that did not converge and machines that were left out.
pub fn solve(job: &Job, cancel: &Arc<AtomicBool>, progress: impl Fn(Event) + Send + Sync + 'static) -> Result<CfdTable, String> {
    solve_sized(job, cancel, progress, shared_grid())
}

/// [`solve`], with the grid of every point sized for the table's highest Mach number (`shared_grid`) or, as before, for the point's own.
pub(super) fn solve_sized(
    job: &Job,
    cancel: &Arc<AtomicBool>,
    progress: impl Fn(Event) + Send + Sync + 'static,
    shared_grid: bool,
) -> Result<CfdTable, String> {
    let started = Instant::now();
    let grid_mach = job.nodes.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    let send: Outbox = Arc::new(progress);
    let altitude = job.altitude.clamp(0.0, SOLVER_CEILING);
    let n_drag = job.nodes.len();
    let total = job.work_items();
    let mut notes = job.plan.notes.clone();
    if job.altitude > SOLVER_CEILING {
        notes.push("The solver's atmosphere ends at 20 km; solved there.".to_string());
    }
    let work = {
        let (job, cancel, send) = (Arc::new(job.clone()), cancel.clone(), send.clone());
        move |session: &mut Session, machine: usize, node: usize| -> Result<Node, Failed> {
            if node >= n_drag {
                return solve_lift_node(&job, grid_mach, shared_grid, job.stability[node - n_drag], session, machine, started, &cancel, &send).map(Node::Lift);
            }
            let mach = job.nodes[node];
            let mut analysis = node_analysis(&job, NodeKind::Drag, session, altitude, grid_mach, shared_grid)?;
            let progress = sink(started, cancel.clone(), send.clone(), false, machine);
            let mut polar = match session {
                Session::Remote(r) => {
                    analysis.spec().map_err(|e| Failed::local(mach, e)).and_then(|spec| r.solve(&spec, &[solver::Condition::from(mach)], &progress))?
                }
                Session::Local { backend } => {
                    // A picture of the flow every few hundred iterations (the first is the grid with the free stream on it, at once).
                    // A remote worker brings back only the final field of the point: its snapshots stay on its side.
                    if !solves_3d() {
                        if let Ok(first) = analysis.initial_field(mach) {
                            send(Event::Field(FieldRef::from_2d(&first)));
                        }
                    }
                    let (live, live3) = (send.clone(), send.clone());
                    analysis = analysis
                        .on_field(solver::field::SNAPSHOT_INTERVAL, move |f| live(Event::Field(FieldRef::from_2d(&f))))
                        .on_field3(solver::field::SNAPSHOT_INTERVAL, move |f| live3(Event::Field(FieldRef::of(f))));
                    let polar = analysis.on_progress(progress).run(&[mach]).map_err(|e| Failed::local(mach, e))?;
                    if polar.backend == solver::BackendUsed::Cpu {
                        // The GPU did not start: do not try it again for every point.
                        *backend = solver::Backend::Cpu;
                    }
                    polar
                }
            };
            let cpu = polar.backend == solver::BackendUsed::Cpu;
            // the final field: the axisymmetric solver's (2D), or the 3D engine's wedge (a Field3 the viewer shows side-on, like a 2D one)
            if let Some(field) = polar.points[0].field.take() {
                send(Event::PointField { field: FieldRef::from_2d(&field) });
            } else if let Some(field) = polar.points[0].field3.take() {
                send(Event::PointField { field: FieldRef::of(field) });
            }
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
                    interference: 0.0,
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
            Ok(Node::Drag(DragNode { point: stored, cpu, fallback: polar.fallback }))
        }
    };
    let fleet = run_fleet(&job.compute, job.cpu_only, total, started, cancel, &send, work)?;
    let mut tally = Tally { points: Vec::with_capacity(n_drag), notes, gpu: 0, cpu: 0, fallback: None };
    tally.notes.extend(lost_notes(&fleet));
    let mut raw: Vec<solver::PolarPoint> = Vec::new();
    for (k, (_, node)) in fleet.done.results.iter().enumerate() {
        match node {
            Node::Drag(node) => {
                if node.cpu {
                    tally.cpu += 1;
                } else {
                    tally.gpu += 1;
                }
                tally.fallback = tally.fallback.take().or_else(|| node.fallback.clone());
                match node.point {
                    Some(point) => tally.points.push(point),
                    None => tally.notes.push(fallback_note(job.nodes[k])),
                }
            }
            Node::Lift(node) => raw.extend(node.points.iter().cloned()),
        }
    }
    if tally.points.len() < 2 && n_drag >= 2 {
        return Err("the solver did not converge at enough Mach numbers".to_string());
    }
    // The angles of the table: a stability part that gave too little leaves the drag table whole, with its reason in the notes.
    let mut stability_notes = Vec::new();
    let stability = if job.stability.is_empty() {
        None
    } else {
        match reduce_lift(&raw, job.stability.len(), &mut stability_notes) {
            Ok(points) => Some(points),
            Err(why) => {
                stability_notes.push(wording::stability_left_out(&why));
                None
            }
        }
    };
    let backend = backend_text(&parts(&fleet, Node::cpu));
    let seconds = started.elapsed().as_secs_f64();
    let mut table = table_from(job, tally, n_drag, seconds);
    table.backend.clone_from(&backend);
    table.lift = stability.map(|points| CfdLiftTable {
        key: lift_key(job.plan.geometry, &job.stability),
        solved_at: now(),
        solve_seconds: seconds,
        backend,
        points,
        notes: stability_notes.clone(),
    });
    if table.lift.is_none() {
        table.notes.extend(stability_notes);
    }
    Ok(table)
}
