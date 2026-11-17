//! The live state of a CFD solve: what the Analyze screen shows while the solver works, as pure data with no clock of its own.
//!
//! A solve is a list of *units*: one per Mach number for the zero-lift drag table, one per (Mach number, angle of attack) for the
//! 3D stability solve. The worker thread sends [`Iteration`] reports (where the running unit is: iteration, residual drop, the
//! monitor the solver's stop rule watches and its recent range) and a [`PointDone`] when a unit ends; [`Tracker`] folds them into
//! what the screen draws: the unit in flight, the finished units with their outcome, the partial curve, a progress fraction
//! weighted by the work each unit takes, and a time estimate from the measured throughput. Every event carries `at`, the seconds
//! since the solve began on the worker's clock, so the arithmetic here is deterministic and tested.
//!
//! # Progress and time
//!
//! Solver iterations are the measure of work (a point stops by its stop rule, so the iteration count differs per Mach number and
//! is far larger through the transonic range than elsewhere). A finished unit costs the iterations it took. A unit to come costs
//! what its finished neighbours in Mach took, interpolated, so a transonic point is expected to cost like the transonic points
//! already solved; until a neighbour is solved it costs [`PRIOR_ITERATIONS`], the average of the example table (HANDOFF.md: about
//! 29 000 iterations over 10 points). The fraction is work done over the work done plus the work expected, and never goes back.
//! The time left is the work left over the measured throughput (iterations per second over the finished units, which includes the
//! grid and kernel set-up of every point), and is not given before [`MIN_DONE_FOR_ETA`] units are finished.
//!
//! # Several machines
//!
//! A solve runs on one or more *machines* (this computer, remote GPUs), each working on its own unit; every report carries the
//! index of its machine. The tracker keeps each machine's state (connecting, ready on a device, skipped, dropped) and the unit it
//! has in flight, and the time left uses the combined throughput: the sum over the machines of their own iterations per second.

use bowshock_model::{CfdLiftPoint, CfdPoint, CfdQuality};

/// Units that must be finished before a time left is given.
pub const MIN_DONE_FOR_ETA: usize = 2;
/// Iterations assumed for a unit whose neighbours are not solved yet: the example table averages about 2 900 per Mach point
/// (HANDOFF.md, RTX 3090, relative drag rule).
pub const PRIOR_ITERATIONS: f64 = 3000.0;
/// Most monitor samples kept per unit for the live trace; past it every second sample is dropped (the trace keeps its shape).
pub const HISTORY_CAP: usize = 240;
/// Smallest share of a running unit's iterations still expected after the cap of its prediction is passed.
const TAIL: f64 = 0.15;

/// Which part of the solve is on show: the drag points (one unit per Mach number at zero incidence) or the 3D angles (one unit per
/// Mach number and angle of attack). One solve has both, the drag first.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Phase {
    /// The drag table's points.
    Drag,
    /// The stability points at angles of attack.
    Stability,
}

/// How a unit ended, as the screen words it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Outcome {
    /// The residual dropped by the requested orders.
    Converged,
    /// The residual did not drop enough but the monitored quantity settled (the solver's stop rule fired).
    Settled,
    /// The iteration cap was reached before the quantity settled: the point is kept, flagged.
    Stalled,
    /// The solver gave no usable point; the table is interpolated from its neighbours there.
    Fallback,
}

impl Outcome {
    /// From the quality of the stored point; `None` is a point the solver could not give.
    pub fn from_quality(q: Option<CfdQuality>) -> Outcome {
        match q {
            Some(CfdQuality::Converged) => Outcome::Converged,
            Some(CfdQuality::Stationary) => Outcome::Settled,
            Some(CfdQuality::Stalled) => Outcome::Stalled,
            None => Outcome::Fallback,
        }
    }

    /// Whether the point can be trusted as the solver's answer (converged or settled).
    pub fn is_settled(self) -> bool {
        matches!(self, Outcome::Converged | Outcome::Settled)
    }

    /// The word for a list row.
    pub fn word(self) -> &'static str {
        match self {
            Outcome::Converged => "converged",
            Outcome::Settled => "settled",
            Outcome::Stalled => "not settled",
            Outcome::Fallback => "fallback",
        }
    }
}

/// One unit of work: a Mach number, and for the 3D solve an angle of attack.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Unit {
    /// Mach number.
    pub mach: f64,
    /// Angle of attack (degrees) for the 3D solve; `None` for the zero-lift drag table.
    pub alpha_deg: Option<f64>,
}

impl Unit {
    fn is(&self, mach: f64, alpha_deg: Option<f64>) -> bool {
        (self.mach - mach).abs() < 1e-6
            && match (self.alpha_deg, alpha_deg) {
                (None, None) => true,
                (Some(a), Some(b)) => (a - b).abs() < 1e-6,
                _ => false,
            }
    }
}

/// A report from the running unit (throttled on the worker).
#[derive(Debug, Clone, PartialEq)]
pub struct Iteration {
    /// The machine that reports (its index in [`Tracker::machines`]).
    pub machine: usize,
    /// Seconds since the solve began.
    pub at: f64,
    /// Mach number of the running unit.
    pub mach: f64,
    /// Its angle of attack (degrees), for the 3D solve.
    pub alpha_deg: Option<f64>,
    /// Iterations done.
    pub iteration: usize,
    /// The iteration cap.
    pub max_iterations: usize,
    /// Orders of magnitude the residual has dropped from its peak.
    pub residual_orders: f64,
    /// What the stop rule watches (Cd of the drag table, CN of the 3D solve); NaN before the first sample.
    pub monitor: f64,
    /// Range of the monitor over the stop rule's window relative to its mean; `None` while the window is not full.
    pub spread: Option<f64>,
    /// The relative range below which the stop rule fires.
    pub tolerance: Option<f64>,
}

/// A unit that ended.
#[derive(Debug, Clone, PartialEq)]
pub struct PointDone {
    /// The machine that solved it.
    pub machine: usize,
    /// Seconds since the solve began.
    pub at: f64,
    /// Mach number.
    pub mach: f64,
    /// Angle of attack (degrees), for the 3D solve.
    pub alpha_deg: Option<f64>,
    /// How it ended.
    pub outcome: Outcome,
    /// The stored drag point (drag table, when the solver gave one).
    pub point: Option<CfdPoint>,
    /// The stored 3D point (3D solve, when the solver gave one).
    pub lift: Option<CfdLiftPoint>,
    /// Iterations it took.
    pub iterations: usize,
    /// It ran on the CPU.
    pub cpu: bool,
    /// Why the GPU was not used, when it was not.
    pub fallback: Option<String>,
}

/// A finished unit.
#[derive(Debug, Clone, PartialEq)]
pub struct Done {
    /// The machine that solved it.
    pub machine: usize,
    /// It ran on the CPU.
    pub cpu: bool,
    /// How it ended.
    pub outcome: Outcome,
    /// Cd (drag table) or CN (3D solve) of the point; `None` for a fallback.
    pub value: Option<f64>,
    /// CP from the nose tip (m), for the 3D solve.
    pub cp: Option<f64>,
    /// Iterations it took.
    pub iterations: usize,
    /// Seconds it took, set-up included.
    pub seconds: f64,
    /// The stored drag point.
    pub point: Option<CfdPoint>,
    /// The stored 3D point.
    pub lift: Option<CfdLiftPoint>,
}

/// Where a unit is.
#[derive(Debug, Clone, PartialEq)]
pub enum Slot {
    /// Not started.
    Pending,
    /// Reporting; `ended` is set once the next unit began (the 3D solve hands its results over per Mach number, so a unit can
    /// have ended without its result being in).
    Running {
        /// The machine running it.
        machine: usize,
        /// Seconds since the solve began when the unit began.
        started: f64,
        /// Iterations reported so far.
        iterations: usize,
        /// When the next unit began, if it has.
        ended: Option<f64>,
    },
    /// Finished.
    Done(Done),
}

/// The unit in flight: where its iteration is, and the monitor's trace.
#[derive(Debug, Clone, PartialEq)]
pub struct Live {
    /// The machine it runs on.
    pub machine: usize,
    /// Index of the unit.
    pub unit: usize,
    /// Iterations done.
    pub iteration: usize,
    /// The iteration cap.
    pub max_iterations: usize,
    /// Orders the residual has dropped.
    pub residual_orders: f64,
    /// The latest monitor sample (NaN before one).
    pub monitor: f64,
    /// The monitor's relative range over the stop rule's window.
    pub spread: Option<f64>,
    /// The stop rule's tolerance.
    pub tolerance: Option<f64>,
    /// `(iteration, monitor)` samples, at most [`HISTORY_CAP`].
    pub history: Vec<(usize, f64)>,
}

/// A machine a solve runs on, as the screen is told of it first.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MachineInfo {
    /// What the screen calls it (`This computer`, the machine's label).
    pub name: String,
    /// It is another computer over SSH, not this one.
    pub remote: bool,
}

/// Where a machine is in the solve.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum MachineState {
    /// Connecting or setting up (a remote's worker is built on its first use); the step, empty until one is known.
    Connecting(String),
    /// Ready, on this device (the GPU's name, `CPU`, or `<gpu> on <host>`).
    Ready(String),
    /// Never took part: it could not connect. The reason.
    Skipped(String),
    /// Took part and was lost: its connection failed mid-solve. The reason; its unit went back to the others.
    Dropped(String),
}

/// A machine of the solve and what it has done.
#[derive(Debug, Clone, PartialEq)]
pub struct MachineRow {
    /// What the screen calls it.
    pub name: String,
    /// It is another computer over SSH.
    pub remote: bool,
    /// Where it is.
    pub state: MachineState,
    /// The device it was ready on, kept after it left.
    device: Option<String>,
    /// Units it finished.
    pub done: usize,
    /// Of those, on its CPU.
    pub cpu_done: usize,
    cursor: f64,
}

impl MachineRow {
    fn new(name: impl Into<String>, remote: bool) -> MachineRow {
        MachineRow { name: name.into(), remote, state: MachineState::Connecting(String::new()), device: None, done: 0, cpu_done: 0, cursor: 0.0 }
    }

    /// Whether it can still take work (not skipped, not dropped).
    pub fn alive(&self) -> bool {
        !matches!(self.state, MachineState::Skipped(_) | MachineState::Dropped(_))
    }

    /// The device it was ready on, once known.
    pub fn device(&self) -> Option<&str> {
        self.device.as_deref()
    }
}

/// The state of a solve, built from its events.
#[derive(Debug, Clone)]
pub struct Tracker {
    units: Vec<Unit>,
    slots: Vec<Slot>,
    /// The units in flight, one per busy machine.
    live: Vec<Live>,
    machines: Vec<MachineRow>,
    /// Why the GPU was not used, when it was not.
    pub fallback: Option<String>,
    clock: f64,
    fraction: f64,
}

impl Tracker {
    /// A solve of `units` (in the order they are asked for); its machines are named by [`Tracker::set_machines`].
    pub fn new(units: Vec<Unit>) -> Tracker {
        let slots = vec![Slot::Pending; units.len()];
        Tracker { units, slots, live: Vec::new(), machines: Vec::new(), fallback: None, clock: 0.0, fraction: 0.0 }
    }

    /// The units of a drag table at `nodes`.
    pub fn drag_units(nodes: &[f64]) -> Vec<Unit> {
        nodes.iter().map(|&mach| Unit { mach, alpha_deg: None }).collect()
    }

    /// The units of a 3D solve at `nodes` and `alphas` (degrees): Mach number outermost, as the solver runs them.
    pub fn stability_units(nodes: &[f64], alphas: &[f64]) -> Vec<Unit> {
        nodes.iter().flat_map(|&mach| alphas.iter().map(move |&a| Unit { mach, alpha_deg: Some(a) })).collect()
    }

    /// The units of a whole solve: the drag points at `nodes`, then the stability points at `stability` Mach numbers and `alphas`.
    pub fn solve_units(nodes: &[f64], stability: &[f64], alphas: &[f64]) -> Vec<Unit> {
        let mut units = Tracker::drag_units(nodes);
        units.extend(Tracker::stability_units(stability, alphas));
        units
    }

    /// Which part of the solve is on show: what is in flight, else the stability part once every drag point is in (and there is
    /// one), else the drag part.
    pub fn phase(&self) -> Phase {
        if let Some((u, _)) = self.lives().next() {
            return if u.alpha_deg.is_some() { Phase::Stability } else { Phase::Drag };
        }
        let drag_in = self.units.iter().zip(&self.slots).filter(|(u, _)| u.alpha_deg.is_none()).all(|(_, s)| matches!(s, Slot::Done(_)));
        if drag_in && self.units.iter().any(|u| u.alpha_deg.is_some()) {
            Phase::Stability
        } else {
            Phase::Drag
        }
    }

    /// The drag units: how many, the first of them.
    pub fn drag_count(&self) -> usize {
        self.units.iter().filter(|u| u.alpha_deg.is_none()).count()
    }

    /// The Mach numbers of the stability units, each once, in order.
    pub fn stability_machs(&self) -> Vec<f64> {
        let mut m: Vec<f64> = Vec::new();
        for u in self.units.iter().filter(|u| u.alpha_deg.is_some()) {
            if m.last().is_none_or(|l| (l - u.mach).abs() > 1e-9) {
                m.push(u.mach);
            }
        }
        m
    }

    /// Every unit, in running order.
    pub fn units(&self) -> &[Unit] {
        &self.units
    }

    /// Where unit `i` is.
    pub fn slot(&self, i: usize) -> &Slot {
        &self.slots[i]
    }

    /// The units in flight (one per busy machine) and their state.
    pub fn lives(&self) -> impl Iterator<Item = (&Unit, &Live)> {
        self.live.iter().map(|l| (&self.units[l.unit], l))
    }

    /// The machines of the solve.
    pub fn machines(&self) -> &[MachineRow] {
        &self.machines
    }

    /// Names the machines of the solve (sent first).
    pub fn set_machines(&mut self, infos: &[MachineInfo]) {
        self.machines = infos.iter().map(|i| MachineRow::new(i.name.clone(), i.remote)).collect();
    }

    fn machine_mut(&mut self, i: usize) -> &mut MachineRow {
        while self.machines.len() <= i {
            self.machines.push(MachineRow::new("This computer", false));
        }
        &mut self.machines[i]
    }

    /// Folds in a change of a machine's state at `at` seconds.
    pub fn on_machine(&mut self, index: usize, at: f64, state: MachineState) {
        self.clock = self.clock.max(at);
        let ready = matches!(state, MachineState::Ready(_));
        let row = self.machine_mut(index);
        if let MachineState::Ready(device) = &state {
            row.device = Some(device.clone());
        }
        row.state = state;
        if ready {
            // its first unit starts now, not when the solve began (a remote's worker may have been built first)
            row.cursor = row.cursor.max(at);
        }
        if !self.machines[index].alive() {
            // what it had in flight went back to the others
            self.live.retain(|l| l.machine != index);
            for slot in &mut self.slots {
                if matches!(slot, Slot::Running { machine, .. } if *machine == index) {
                    *slot = Slot::Pending;
                }
            }
        }
    }

    /// Where the solve runs, for the lines that name it: the devices of the machines that solved something, else of those that are
    /// ready, joined; empty before one is.
    pub fn device(&self) -> String {
        let used: Vec<&MachineRow> = self.machines.iter().filter(|m| m.done > 0).collect();
        let pool: Vec<&MachineRow> = if used.is_empty() { self.machines.iter().collect() } else { used };
        pool.iter().filter_map(|m| m.device()).collect::<Vec<_>>().join(", ")
    }

    /// Drag units finished.
    pub fn drag_done(&self) -> usize {
        self.units.iter().zip(&self.slots).filter(|(u, s)| u.alpha_deg.is_none() && matches!(s, Slot::Done(_))).count()
    }

    /// Units finished.
    pub fn done_count(&self) -> usize {
        self.slots.iter().filter(|s| matches!(s, Slot::Done(_))).count()
    }

    /// Units in all.
    pub fn total(&self) -> usize {
        self.units.len()
    }

    fn find(&self, mach: f64, alpha_deg: Option<f64>) -> Option<usize> {
        self.units.iter().position(|u| u.is(mach, alpha_deg))
    }

    /// Folds in a report from the running unit of a machine.
    pub fn on_iteration(&mut self, it: &Iteration) {
        let Some(idx) = self.find(it.mach, it.alpha_deg) else { return };
        self.clock = self.clock.max(it.at);
        let m = it.machine;
        self.machine_mut(m);
        if let Some(pos) = self.live.iter().position(|l| l.machine == m && l.unit != idx) {
            // The unit before this one on the machine is over (the 3D solve reports a Mach number's results only when all its
            // angles are done).
            let before = self.live.remove(pos).unit;
            if let Slot::Running { ended, .. } = &mut self.slots[before] {
                ended.get_or_insert(it.at);
            }
            let row = &mut self.machines[m];
            row.cursor = row.cursor.max(it.at);
        }
        if let Slot::Pending = self.slots[idx] {
            self.slots[idx] = Slot::Running { machine: m, started: self.machines[m].cursor, iterations: 0, ended: None };
        }
        if let Slot::Running { iterations, .. } = &mut self.slots[idx] {
            *iterations = it.iteration;
        }
        let pos = match self.live.iter().position(|l| l.machine == m) {
            Some(pos) => pos,
            None => {
                self.live.push(Live {
                    machine: m,
                    unit: idx,
                    iteration: 0,
                    max_iterations: it.max_iterations,
                    residual_orders: 0.0,
                    monitor: f64::NAN,
                    spread: None,
                    tolerance: None,
                    history: Vec::new(),
                });
                self.live.len() - 1
            }
        };
        let live = &mut self.live[pos];
        live.iteration = it.iteration;
        live.max_iterations = it.max_iterations;
        live.residual_orders = it.residual_orders;
        live.monitor = it.monitor;
        live.spread = it.spread;
        live.tolerance = it.tolerance;
        if it.monitor.is_finite() && live.history.last().is_none_or(|h| h.0 < it.iteration) {
            live.history.push((it.iteration, it.monitor));
            if live.history.len() > HISTORY_CAP {
                let kept: Vec<(usize, f64)> = live.history.iter().copied().step_by(2).collect();
                live.history = kept;
            }
        }
        self.refresh_fraction();
    }

    /// Folds in a finished unit.
    pub fn on_done(&mut self, d: &PointDone) {
        let Some(idx) = self.find(d.mach, d.alpha_deg) else { return };
        self.clock = self.clock.max(d.at);
        let m = d.machine;
        let cursor = self.machine_mut(m).cursor;
        let (started, ended) = match self.slots[idx] {
            Slot::Running { started, ended, .. } => (started, ended),
            _ => (cursor, None),
        };
        let seconds = (ended.unwrap_or(d.at) - started).max(0.0);
        let (value, cp) = match (&d.point, &d.lift) {
            (Some(p), _) => (Some(p.cd), None),
            (_, Some(l)) => (Some(l.cn), Some(l.x_cp)),
            _ => (None, None),
        };
        self.slots[idx] =
            Slot::Done(Done { machine: m, cpu: d.cpu, outcome: d.outcome, value, cp, iterations: d.iterations, seconds, point: d.point, lift: d.lift });
        self.live.retain(|l| l.unit != idx);
        let row = &mut self.machines[m];
        row.cursor = row.cursor.max(d.at);
        row.done += 1;
        if d.cpu {
            row.cpu_done += 1;
            row.device = Some("CPU".to_string());
            if matches!(row.state, MachineState::Ready(_)) {
                row.state = MachineState::Ready("CPU".to_string());
            }
            self.fallback = d.fallback.clone().or_else(|| self.fallback.take());
        }
        self.refresh_fraction();
    }

    /// Marks the solve as complete.
    pub fn finish(&mut self) {
        self.live.clear();
        self.fraction = 1.0;
    }

    /// Marks the solve as stopped: nothing is in flight any more (its last report stays in the unit's slot).
    pub fn halt(&mut self) {
        self.live.clear();
    }

    /// Seconds since the solve began on the worker's clock, as of the latest event.
    pub fn clock(&self) -> f64 {
        self.clock
    }

    /// Iterations a unit that is not done is expected to take, from its finished neighbours in Mach (same angle of attack when
    /// there are some), nearest-held at the ends, and [`PRIOR_ITERATIONS`] without any. Units that did not settle left out: they
    /// stopped on the iteration cap, not where the physics did.
    fn predicted(&self, unit: usize) -> f64 {
        let want = &self.units[unit];
        let settled = |same_alpha: bool| -> Vec<(f64, f64)> {
            let mut v: Vec<(f64, f64)> = self
                .units
                .iter()
                .zip(&self.slots)
                .filter_map(|(u, s)| match s {
                    Slot::Done(d) if d.outcome.is_settled() && d.iterations > 0 && (!same_alpha || u.alpha_deg == want.alpha_deg) => {
                        Some((u.mach, d.iterations as f64))
                    }
                    _ => None,
                })
                .collect();
            v.sort_by(|a, b| a.0.total_cmp(&b.0));
            v
        };
        let mut known = settled(true);
        if known.is_empty() {
            known = settled(false);
        }
        let (Some(first), Some(last)) = (known.first().copied(), known.last().copied()) else { return PRIOR_ITERATIONS };
        if want.mach <= first.0 {
            return first.1;
        }
        if want.mach >= last.0 {
            return last.1;
        }
        let i = known.partition_point(|p| p.0 <= want.mach);
        let (a, b) = (known[i - 1], known[i]);
        a.1 + (b.1 - a.1) * (want.mach - a.0) / (b.0 - a.0)
    }

    /// Iterations a unit still has to do.
    fn remaining(&self, unit: usize) -> f64 {
        match &self.slots[unit] {
            Slot::Done(_) => 0.0,
            Slot::Running { ended: Some(_), .. } => 0.0,
            Slot::Running { iterations, .. } => {
                let done = *iterations as f64;
                (self.predicted(unit) - done).max(TAIL * done)
            }
            Slot::Pending => self.predicted(unit),
        }
    }

    fn done_iterations(&self) -> f64 {
        self.slots
            .iter()
            .map(|s| match s {
                Slot::Done(d) => d.iterations as f64,
                Slot::Running { iterations, .. } => *iterations as f64,
                Slot::Pending => 0.0,
            })
            .sum()
    }

    fn refresh_fraction(&mut self) {
        let done = self.done_iterations();
        let left: f64 = (0..self.units.len()).map(|i| self.remaining(i)).sum();
        let f = if done + left > 0.0 { done / (done + left) } else { 0.0 };
        // never complete before it is, and never backwards
        self.fraction = self.fraction.max(f.min(0.99));
    }

    /// How far the solve is, 0 to 1: iterations done over iterations done plus expected, never decreasing.
    pub fn fraction(&self) -> f64 {
        self.fraction
    }

    /// Solver iterations per second over the finished units, set-up included, for the whole solve: the sum over the machines that
    /// can still work of their own rate (a machine with no finished unit adds nothing); `None` before one finished.
    pub fn throughput(&self) -> Option<f64> {
        let mut per: Vec<(f64, f64)> = vec![(0.0, 0.0); self.machines.len().max(1)];
        for slot in &self.slots {
            if let Slot::Done(d) = slot {
                if per.len() <= d.machine {
                    per.resize(d.machine + 1, (0.0, 0.0));
                }
                per[d.machine].0 += d.iterations as f64;
                per[d.machine].1 += d.seconds;
            }
        }
        let rate: f64 = per
            .iter()
            .enumerate()
            .filter(|(i, _)| self.machines.get(*i).is_none_or(MachineRow::alive))
            .filter(|(_, (iters, seconds))| *seconds > 0.0 && *iters > 0.0)
            .map(|(_, (iters, seconds))| iters / seconds)
            .sum();
        (rate > 0.0).then_some(rate)
    }

    /// Seconds left: the iterations still to do over the measured throughput. `None` ("estimating…") until
    /// [`MIN_DONE_FOR_ETA`] units are finished.
    pub fn eta(&self) -> Option<f64> {
        if self.done_count() < MIN_DONE_FOR_ETA {
            return None;
        }
        let rate = self.throughput()?;
        let left: f64 = (0..self.units.len()).map(|i| self.remaining(i)).sum();
        Some(left / rate)
    }

    /// How many drag units from the first are all finished: the part of the range a stopped solve covers without a gap (machines that
    /// share the work finish out of order). The stability units come after the drag units and have their own run, see
    /// [`Tracker::stability_kept`].
    pub fn done_prefix(&self) -> usize {
        self.slots.iter().take(self.drag_count()).take_while(|s| matches!(s, Slot::Done(_))).count()
    }

    /// The stability part a stopped solve keeps: the leading Mach numbers (in the order solved) whose angles all finished, with their
    /// points, how many Mach numbers that is, and the Mach number that was left with only some of its angles, if there is one. A
    /// Mach number with an angle missing is not kept: a stability table needs every node at every angle.
    pub fn stability_kept(&self) -> (Vec<CfdLiftPoint>, usize, Option<f64>) {
        let machs = self.stability_machs();
        let (mut points, mut kept, mut half) = (Vec::new(), 0, None);
        for &m in &machs {
            let group: Vec<&Slot> =
                self.units.iter().zip(&self.slots).filter(|(u, _)| u.alpha_deg.is_some() && (u.mach - m).abs() < 1e-9).map(|(_, s)| s).collect();
            if group.iter().all(|s| matches!(s, Slot::Done(_))) {
                points.extend(group.iter().filter_map(|s| if let Slot::Done(d) = s { d.lift } else { None }));
                kept += 1;
            } else {
                half = group.iter().any(|s| matches!(s, Slot::Done(_))).then_some(m);
                break;
            }
        }
        (points, kept, half)
    }

    /// For each machine that solved one of the kept units (the first [`Tracker::done_prefix`] drag units and the stability units of
    /// [`Tracker::stability_kept`]): its index, how many of them, and how many of those on its CPU.
    pub fn prefix_machines(&self) -> Vec<(usize, usize, usize)> {
        let mut out: Vec<(usize, usize, usize)> = Vec::new();
        let (_, kept, _) = self.stability_kept();
        let kept_machs: Vec<f64> = self.stability_machs().into_iter().take(kept).collect();
        let n = self.done_prefix();
        for (i, (u, slot)) in self.units.iter().zip(&self.slots).enumerate() {
            let Slot::Done(d) = slot else { continue };
            let keep = if u.alpha_deg.is_none() { i < n } else { kept_machs.iter().any(|m| (m - u.mach).abs() < 1e-9) };
            if !keep {
                continue;
            }
            match out.iter_mut().find(|e| e.0 == d.machine) {
                Some(e) => {
                    e.1 += 1;
                    e.2 += usize::from(d.cpu);
                }
                None => out.push((d.machine, 1, usize::from(d.cpu))),
            }
        }
        out.sort_unstable();
        out
    }

    /// The drag points of the first [`Tracker::done_prefix`] units.
    pub fn prefix_points(&self) -> Vec<CfdPoint> {
        self.slots.iter().take(self.done_prefix()).filter_map(|s| if let Slot::Done(d) = s { d.point } else { None }).collect()
    }

    /// The Mach numbers (with their angle) of the finished units that did not settle, in order.
    fn unsettled(&self) -> Vec<(Outcome, Unit)> {
        self.units
            .iter()
            .zip(&self.slots)
            .filter_map(|(u, s)| match s {
                Slot::Done(d) if !d.outcome.is_settled() => Some((d.outcome, *u)),
                _ => None,
            })
            .collect()
    }

    /// The one-line account of a finished solve: `Solved 10 Mach points in 2 min 14 s on Apple M1 Max · 9 converged, 1 fallback
    /// (Mach 1.05)`.
    pub fn summary(&self, seconds: f64) -> String {
        let what = super::cfd::solved_what(self.drag_done(), self.stability_machs().len());
        let head = format!("Solved {what} in {} on {}", duration_text(seconds), self.device());
        let settled = self.done_count() - self.unsettled().len();
        let mut parts = vec![if settled == self.done_count() { "all converged".to_string() } else { format!("{settled} converged") }];
        for outcome in [Outcome::Stalled, Outcome::Fallback] {
            let at: Vec<String> = self
                .unsettled()
                .into_iter()
                .filter(|(o, _)| *o == outcome)
                .map(|(_, u)| match u.alpha_deg {
                    Some(a) => format!("Mach {:.2}, {a:.0}°", u.mach),
                    None => format!("Mach {:.2}", u.mach),
                })
                .collect();
            if !at.is_empty() {
                parts.push(format!("{} {} ({})", at.len(), outcome.word(), at.join("; ")));
            }
        }
        format!("{head} · {}", parts.join(", "))
    }

    /// The toast's short form, for the narrow status strip (the verdict comes before the time, which is what a clipped line loses):
    /// `Solved 10 points · 1 not converged · 2 min 14 s`.
    pub fn brief(&self, seconds: f64) -> String {
        let what = super::cfd::solved_brief(self.drag_done(), !self.stability_machs().is_empty());
        let bad = self.unsettled().len();
        let verdict = if bad == 0 { "all converged".to_string() } else { format!("{bad} not converged") };
        format!("Solved {what} · {verdict} · {}", duration_text(seconds))
    }
}

/// `42 s`, `2 min 14 s`: how long a solve took.
pub fn duration_text(seconds: f64) -> String {
    let s = seconds.max(0.0).round() as u64;
    if s < 60 {
        format!("{s} s")
    } else {
        format!("{} min {} s", s / 60, s % 60)
    }
}

/// The time left, coarsely: `a few seconds`, `about 40 s`, `about 2 min`.
pub fn eta_text(seconds: f64) -> String {
    let s = seconds.max(0.0);
    if s < 10.0 {
        "a few seconds".to_string()
    } else if s < 90.0 {
        format!("about {} s", (s / 5.0).round() as u64 * 5)
    } else {
        format!("about {} min", (s / 60.0).round() as u64)
    }
}

/// `12,345`: an iteration count.
pub fn thousands(n: usize) -> String {
    let digits = n.to_string();
    let mut out = String::with_capacity(digits.len() + digits.len() / 3);
    for (i, c) in digits.chars().enumerate() {
        if i > 0 && (digits.len() - i).is_multiple_of(3) {
            out.push(',');
        }
        out.push(c);
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn on_test_gpu(units: Vec<Unit>) -> Tracker {
        let mut t = Tracker::new(units);
        t.set_machines(&[MachineInfo { name: "This computer".into(), remote: false }]);
        t.on_machine(0, 0.0, MachineState::Ready("Test GPU".into()));
        t
    }

    fn tracker(nodes: &[f64]) -> Tracker {
        on_test_gpu(Tracker::drag_units(nodes))
    }

    /// The first unit in flight.
    fn first_live(t: &Tracker) -> Option<(&Unit, &Live)> {
        t.lives().next()
    }

    fn report(at: f64, mach: f64, iteration: usize, monitor: f64) -> Iteration {
        Iteration {
            machine: 0,
            at,
            mach,
            alpha_deg: None,
            iteration,
            max_iterations: 80_000,
            residual_orders: 2.0,
            monitor,
            spread: Some(0.01),
            tolerance: Some(0.002),
        }
    }

    fn point(mach: f64, cd: f64, quality: CfdQuality) -> CfdPoint {
        CfdPoint { mach, cd, friction: 0.1, pressure: 0.1, base: 0.1, fins: 0.1, other: 0.0, interference: 0.0, quality }
    }

    fn done(at: f64, mach: f64, iterations: usize, quality: Option<CfdQuality>) -> PointDone {
        PointDone {
            machine: 0,
            at,
            mach,
            alpha_deg: None,
            outcome: Outcome::from_quality(quality),
            point: quality.map(|q| point(mach, 0.5, q)),
            lift: None,
            iterations,
            cpu: false,
            fallback: None,
        }
    }

    #[test]
    fn a_unit_goes_from_pending_to_running_to_done() {
        let mut t = tracker(&[0.5, 0.9]);
        assert_eq!((t.done_count(), t.total(), first_live(&t).is_none()), (0, 2, true));
        t.on_iteration(&report(3.0, 0.5, 50, f64::NAN));
        assert!(matches!(t.slot(0), Slot::Running { started, iterations: 50, ended: None, .. } if *started == 0.0));
        let (unit, live) = first_live(&t).unwrap();
        assert_eq!((unit.mach, live.iteration), (0.5, 50));
        assert!(live.history.is_empty(), "no monitor sample yet");
        t.on_iteration(&report(4.0, 0.5, 100, 0.52));
        assert_eq!(first_live(&t).unwrap().1.history, vec![(100, 0.52)]);
        t.on_done(&done(20.0, 0.5, 2000, Some(CfdQuality::Converged)));
        assert!(first_live(&t).is_none());
        let Slot::Done(d) = t.slot(0) else { panic!("done") };
        assert_eq!((d.iterations, d.seconds, d.value), (2000, 20.0, Some(0.5)), "the seconds run from the start of the unit");
        // the next unit starts where the last one ended
        t.on_iteration(&report(25.0, 0.9, 50, 0.6));
        assert!(matches!(t.slot(1), Slot::Running { started, .. } if *started == 20.0));
        // a report for a unit that is not in the plan is ignored
        t.on_iteration(&report(26.0, 1.7, 50, 0.6));
        assert_eq!(first_live(&t).unwrap().0.mach, 0.9);
    }

    #[test]
    fn the_prediction_follows_the_mach_neighbours_and_ignores_units_that_did_not_settle() {
        let mut t = tracker(&[0.5, 0.7, 0.9, 1.0, 1.1, 2.0]);
        assert_eq!(t.predicted(1), PRIOR_ITERATIONS, "nothing solved yet");
        t.on_done(&done(20.0, 0.5, 2000, Some(CfdQuality::Converged)));
        t.on_done(&done(60.0, 0.9, 4000, Some(CfdQuality::Stationary)));
        assert!((t.predicted(1) - 3000.0).abs() < 1e-9, "halfway between the neighbours: {}", t.predicted(1));
        assert_eq!(t.predicted(3), 4000.0, "above the last solved point it is held");
        // a point that hit the iteration cap says nothing about its neighbours' cost
        t.on_done(&done(300.0, 1.1, 80_000, Some(CfdQuality::Stalled)));
        assert_eq!(t.predicted(3), 4000.0);
        assert_eq!(t.predicted(5), 4000.0);
    }

    #[test]
    fn the_estimate_waits_for_two_points_then_follows_the_measured_throughput() {
        let mut t = tracker(&[0.5, 0.9, 1.0, 1.1, 2.0]);
        t.on_iteration(&report(5.0, 0.5, 1000, 0.5));
        assert_eq!(t.eta(), None, "estimating…");
        t.on_done(&done(20.0, 0.5, 2000, Some(CfdQuality::Converged)));
        assert_eq!(t.eta(), None, "one point is not a rate");
        t.on_done(&done(60.0, 0.9, 4000, Some(CfdQuality::Converged)));
        // 6000 iterations in 60 s: 100 it/s; three units to come, each expected at the last solved point's 4000
        assert_eq!(t.throughput(), Some(100.0));
        assert_eq!(t.eta(), Some(120.0));
        // work in flight is counted off
        t.on_iteration(&report(70.0, 1.0, 1000, 0.7));
        assert_eq!(t.eta(), Some((3000.0 + 4000.0 + 4000.0) / 100.0));
    }

    #[test]
    fn the_fraction_is_weighted_by_expected_work_and_never_goes_back() {
        let mut t = tracker(&[0.5, 0.9, 1.0, 1.1]);
        let mut last = 0.0;
        let mut step = |t: &mut Tracker, f: &dyn Fn(&mut Tracker)| {
            f(t);
            assert!(t.fraction() >= last, "the bar went back: {} then {}", last, t.fraction());
            assert!(t.fraction() < 1.0);
            last = t.fraction();
        };
        step(&mut t, &|t| t.on_iteration(&report(1.0, 0.5, 1500, 0.5)));
        step(&mut t, &|t| t.on_done(&done(20.0, 0.5, 2000, Some(CfdQuality::Converged))));
        // the 0.9 point is expected to cost what the one solved (2000), then it turns out to cost more: the estimate of the rest rises
        // and the fraction would drop; it holds
        step(&mut t, &|t| t.on_iteration(&report(30.0, 0.9, 1900, 0.6)));
        step(&mut t, &|t| t.on_done(&done(80.0, 0.9, 6000, Some(CfdQuality::Converged))));
        // the 0.9 point cost 6000, three times what its neighbour did: two of four units are done and the work done is 8000 of
        // 8000 + 2 x 6000, 0.4; the bar holds the 3900 / 8185 it had reached while the 0.9 point was running
        assert!((t.fraction() - 3900.0 / 8185.0).abs() < 1e-12, "{}", t.fraction());
        t.finish();
        assert_eq!(t.fraction(), 1.0);
    }

    #[test]
    fn a_running_unit_past_its_prediction_still_has_a_tail() {
        let mut t = tracker(&[0.5, 0.9]);
        t.on_done(&done(20.0, 0.5, 2000, Some(CfdQuality::Converged)));
        t.on_iteration(&report(30.0, 0.9, 5000, 0.6));
        assert_eq!(t.remaining(1), 750.0, "15 % of what it has done");
    }

    #[test]
    fn the_monitor_trace_is_decimated_at_the_cap_and_a_new_unit_starts_a_new_trace() {
        let mut t = tracker(&[0.5, 0.9]);
        for k in 1..=(HISTORY_CAP + 10) {
            t.on_iteration(&report(k as f64 * 0.1, 0.5, k * 50, 0.5 + 1e-4 * k as f64));
        }
        let h = &first_live(&t).unwrap().1.history;
        assert!(h.len() <= HISTORY_CAP && h.len() > HISTORY_CAP / 2);
        assert!(h.windows(2).all(|w| w[0].0 < w[1].0), "still in iteration order");
        assert_eq!(h[0].0, 50, "the start of the trace is kept");
        t.on_done(&done(30.0, 0.5, 12_000, Some(CfdQuality::Converged)));
        t.on_iteration(&report(31.0, 0.9, 50, 0.7));
        assert_eq!(first_live(&t).unwrap().1.history, vec![(50, 0.7)]);
    }

    #[test]
    fn the_stability_solve_splits_a_mach_number_between_its_angles() {
        let units = Tracker::stability_units(&[0.8, 1.1], &[2.0, 6.0]);
        assert_eq!(units.len(), 4);
        let mut t = on_test_gpu(units);
        let r = |at: f64, mach: f64, a: f64, iteration: usize| Iteration { alpha_deg: Some(a), ..report(at, mach, iteration, 0.1) };
        t.on_iteration(&r(2.0, 0.8, 2.0, 50));
        t.on_iteration(&r(10.0, 0.8, 2.0, 1000));
        // the 6 degree run begins: the 2 degree run is over (its result comes with the Mach number's)
        t.on_iteration(&r(20.0, 0.8, 6.0, 50));
        assert!(matches!(t.slot(0), Slot::Running { ended: Some(e), .. } if *e == 20.0));
        let lift = |a: f64| CfdLiftPoint { mach: 0.8, alpha_deg: a, cn: 0.1 * a, x_cp: 0.9, cd: None, cm: None, quality: CfdQuality::Converged };
        for (a, iters) in [(2.0, 1000), (6.0, 400)] {
            t.on_done(&PointDone {
                machine: 0,
                at: 28.0,
                mach: 0.8,
                alpha_deg: Some(a),
                outcome: Outcome::Converged,
                point: None,
                lift: Some(lift(a)),
                iterations: iters,
                cpu: false,
                fallback: None,
            });
        }
        let secs: Vec<f64> = (0..2).map(|i| if let Slot::Done(d) = t.slot(i) { d.seconds } else { f64::NAN }).collect();
        assert_eq!(secs, vec![20.0, 8.0], "2° from 0 s to the start of the 6° run, 6° from there to the hand-over");
        assert!(matches!(t.slot(0), Slot::Done(d) if d.value == Some(0.2) && d.cp == Some(0.9)));
        // the 6° run of the next Mach number is predicted from the 6° run solved, not the 2° one
        assert_eq!(t.predicted(3), 400.0);
        assert_eq!(t.predicted(2), 1000.0);
    }

    #[test]
    fn the_summary_names_what_did_not_converge() {
        let nodes = [0.5, 0.8, 1.05, 2.0];
        let mut t = tracker(&nodes);
        t.on_done(&done(10.0, 0.5, 2000, Some(CfdQuality::Converged)));
        t.on_done(&done(30.0, 0.8, 3000, Some(CfdQuality::Stationary)));
        t.on_done(&done(100.0, 1.05, 80_000, Some(CfdQuality::Stalled)));
        t.on_done(&done(120.0, 2.0, 100, None));
        assert_eq!(t.summary(134.0), "Solved 4 Mach points in 2 min 14 s on Test GPU · 2 converged, 1 not settled (Mach 1.05), 1 fallback (Mach 2.00)");
        let mut all = tracker(&[0.5, 0.9]);
        all.on_done(&done(10.0, 0.5, 2000, Some(CfdQuality::Converged)));
        all.on_done(&done(20.0, 0.9, 2000, Some(CfdQuality::Converged)));
        assert_eq!(all.summary(20.0), "Solved 2 Mach points in 20 s on Test GPU · all converged");
        assert_eq!(all.brief(20.0), "Solved 2 points · all converged · 20 s");
        assert_eq!(t.brief(134.0), "Solved 4 points · 2 not converged · 2 min 14 s");
        // a halted solve has nothing in flight but keeps what it had
        t.on_iteration(&report(150.0, 2.0, 50, 0.5));
        t.halt();
        assert!(first_live(&t).is_none() && t.clock() == 150.0);
        assert_eq!(t.prefix_points().iter().filter(|p| p.quality == CfdQuality::Stalled).count(), 1);
        assert_eq!(t.prefix_points().len(), 3, "a fallback has no point");
    }

    #[test]
    fn a_cpu_point_changes_the_device_and_keeps_the_reason() {
        let mut t = tracker(&[0.5, 0.9]);
        t.on_done(&PointDone { cpu: true, fallback: Some("GPU unavailable: no adapter".into()), ..done(10.0, 0.5, 2000, Some(CfdQuality::Converged)) });
        assert_eq!(t.device(), "CPU");
        assert_eq!(t.fallback.as_deref(), Some("GPU unavailable: no adapter"));
        assert_eq!(t.machines()[0].cpu_done, 1);
    }

    #[test]
    fn the_texts() {
        assert_eq!(duration_text(0.4), "0 s");
        assert_eq!(duration_text(59.6), "1 min 0 s");
        assert_eq!(duration_text(134.0), "2 min 14 s");
        assert_eq!(eta_text(4.0), "a few seconds");
        assert_eq!(eta_text(42.0), "about 40 s");
        assert_eq!(eta_text(100.0), "about 2 min");
        assert_eq!((thousands(7), thousands(1850), thousands(80_000), thousands(1_234_567)), ("7".into(), "1,850".into(), "80,000".into(), "1,234,567".into()));
    }

    fn two_machines(units: Vec<Unit>) -> Tracker {
        let mut t = Tracker::new(units);
        t.set_machines(&[MachineInfo { name: "This computer".into(), remote: false }, MachineInfo { name: "omarchy".into(), remote: true }]);
        t.on_machine(0, 0.0, MachineState::Ready("RTX 3090".into()));
        t.on_machine(1, 120.0, MachineState::Ready("RTX 3080 on omarchy".into()));
        t
    }

    fn on(machine: usize, mut r: Iteration) -> Iteration {
        r.machine = machine;
        r
    }

    #[test]
    fn two_machines_each_have_a_unit_in_flight_and_a_dropped_one_gives_its_unit_back() {
        let mut t = two_machines(Tracker::drag_units(&[0.5, 0.9, 1.1, 1.5]));
        t.on_iteration(&on(0, report(130.0, 0.5, 50, 0.4)));
        t.on_iteration(&on(1, report(131.0, 0.9, 100, 0.5)));
        let lives: Vec<(f64, usize, usize)> = t.lives().map(|(u, l)| (u.mach, l.machine, l.iteration)).collect();
        assert_eq!(lives, vec![(0.5, 0, 50), (0.9, 1, 100)], "one per machine");
        // the second machine's first unit began when it was ready, not when the solve began
        assert!(matches!(t.slot(1), Slot::Running { machine: 1, started, .. } if *started == 120.0));
        // machine 0 finishes its unit and starts the next one while machine 1 goes on
        t.on_done(&PointDone { machine: 0, ..done(140.0, 0.5, 2000, Some(CfdQuality::Converged)) });
        t.on_iteration(&on(0, report(141.0, 1.1, 50, 0.6)));
        assert_eq!(t.lives().map(|(u, _)| u.mach).collect::<Vec<_>>(), vec![0.9, 1.1]);
        assert_eq!((t.machines()[0].done, t.machines()[1].done), (1, 0));
        // machine 1 is lost: its unit is waiting again, machine 0 keeps its own
        t.on_machine(1, 150.0, MachineState::Dropped("connection closed".into()));
        assert!(matches!(t.slot(1), Slot::Pending));
        assert_eq!(t.lives().map(|(u, _)| u.mach).collect::<Vec<_>>(), vec![1.1]);
        assert!(!t.machines()[1].alive() && t.machines()[0].alive());
        // and another machine finishing it later is the one that solved it
        t.on_done(&PointDone { machine: 0, ..done(160.0, 0.9, 2500, Some(CfdQuality::Converged)) });
        assert!(matches!(t.slot(1), Slot::Done(d) if d.machine == 0));
    }

    #[test]
    fn the_time_left_is_over_the_combined_throughput_of_the_machines_still_working() {
        let nodes = [0.5, 0.9, 1.1, 1.5, 1.7, 2.0];
        let mut t = two_machines(Tracker::drag_units(&nodes));
        // machine 0: 2 000 iterations in 20 s (100 it/s); machine 1: 2 000 in 40 s (50 it/s), both starting when ready
        t.on_iteration(&on(0, report(1.0, 0.5, 50, 0.4)));
        t.on_done(&PointDone { machine: 0, ..done(20.0, 0.5, 2000, Some(CfdQuality::Converged)) });
        t.on_iteration(&on(1, report(121.0, 0.9, 50, 0.4)));
        t.on_done(&PointDone { machine: 1, ..done(160.0, 0.9, 2000, Some(CfdQuality::Converged)) });
        assert_eq!(t.done_count(), 2);
        let rate = t.throughput().unwrap();
        assert!((rate - 150.0).abs() < 1e-9, "100 + 50 it/s, not their mean: {rate}");
        let left: f64 = 4.0 * 2000.0;
        assert!((t.eta().unwrap() - left / 150.0).abs() < 1e-9);
        // a lost machine no longer counts
        t.on_machine(1, 161.0, MachineState::Dropped("gone".into()));
        assert!((t.throughput().unwrap() - 100.0).abs() < 1e-9);
        // the device line lists the machines that solved something
        assert_eq!(t.device(), "RTX 3090, RTX 3080 on omarchy");
    }

    #[test]
    fn a_stopped_solve_that_finished_out_of_order_covers_only_the_part_without_a_gap() {
        let mut t = two_machines(Tracker::drag_units(&[0.5, 0.9, 1.1, 1.5]));
        t.on_done(&PointDone { machine: 0, ..done(10.0, 0.5, 2000, Some(CfdQuality::Converged)) });
        t.on_done(&PointDone { machine: 1, ..done(11.0, 1.1, 2000, Some(CfdQuality::Converged)) });
        t.on_done(&PointDone { machine: 0, ..done(12.0, 1.5, 2000, Some(CfdQuality::Converged)) });
        assert_eq!((t.done_count(), t.done_prefix()), (3, 1));
        assert_eq!(t.prefix_points().iter().map(|p| p.mach).collect::<Vec<_>>(), vec![0.5]);
        assert_eq!(t.done_count(), 3, "all three are still solved");
        t.on_done(&PointDone { machine: 1, ..done(13.0, 0.9, 2000, Some(CfdQuality::Converged)) });
        assert_eq!(t.done_prefix(), 4);
    }
}
