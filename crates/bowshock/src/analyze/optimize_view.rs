//! The Optimize analysis: the state of a search (its variables, goal, constraints and the run in progress) and the three columns
//! that show it. It bypasses [`super::compute::compute`] and owns its worker, the way the CFD solve views do: [`super::optimize`]
//! is the engine, this file is the screen.

use std::sync::Arc;
use std::time::Instant;

use bowshock_model::{Dim, FieldRef, FlightId, PartId, Rocket};
use iced::widget::{column, container, row, scrollable, text, Space};
use iced::{alignment, Element, Fill, Length, Task, Theme, Widget};

use super::optimize::{self, Constraints, Event, Goal, Metrics, Problem, Variable};
use super::readout::{Row as ReadoutRow, Section, Tag};
use super::Message;
use crate::app::refs::{dim_name, dim_quantity};
use crate::app::worker;
use crate::app::{self, ui, Ctx};
use crate::simulate::flight::{prepare_rocket, Spec};
use crate::theme::{self, MONO};
use crate::tokens::{SPACE_1, SPACE_2, SPACE_3, TEXT_BODY, TEXT_LABEL};
use crate::units::{Quantity, UnitSystem};
use crate::widgets::button::{button as push, Weight};
use crate::widgets::chip::Severity;
use crate::widgets::field::{checkbox, select};
use crate::widgets::glyph::Mark;
use crate::widgets::plot::{Axis, LabelAt, Panel, PlotGroup, PlotMessage, PlotState, PlotView, Series, SeriesColor, YTicks};
use crate::widgets::unit_input::unit_input;

/// Width of the label column of the left panel's rows.
const LABEL_W: f32 = 70.0;
/// Seconds between the worker's beats.
const BEAT: std::time::Duration = std::time::Duration::from_millis(250);
/// Width kept free at the right of the plot for the direct label (as every Analyze plot).
const MARGIN_PX: f32 = 130.0;

/// The messages of the Optimize screen.
#[derive(Debug, Clone)]
pub enum Msg {
    /// The flight to fly.
    Flight(FlightId),
    /// Add a variable row.
    AddVariable,
    /// Remove a variable row.
    RemoveVariable(usize),
    /// Variable `i` moves another part.
    Part(usize, PartId),
    /// Variable `i` moves another dimension of its part.
    Dimension(usize, Dim),
    /// The smallest value of variable `i` (SI).
    Min(usize, f64),
    /// The largest value of variable `i` (SI).
    Max(usize, f64),
    /// The goal.
    Goal(GoalKind),
    /// The target of a Target goal (SI: m or calibers).
    Target(f64),
    /// A constraint on or off.
    Enable(Limit, bool),
    /// A constraint's value (SI).
    Value(Limit, f64),
    /// The evaluation budget.
    Evaluations(usize),
    /// Start the search.
    Run,
    /// Stop the search after the design in flight.
    Cancel,
    /// Put the best design in the document.
    Apply,
    /// From the score plot.
    Plot(PlotMessage),
}

/// The goals as the pick list names them.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GoalKind {
    MaxApogee,
    MinApogee,
    TargetApogee,
    MaxVelocity,
    MinVelocity,
    TargetMargin,
    MinLanding,
    MinMass,
}

impl GoalKind {
    const ALL: [GoalKind; 8] = [
        GoalKind::MaxApogee,
        GoalKind::MinApogee,
        GoalKind::TargetApogee,
        GoalKind::MaxVelocity,
        GoalKind::MinVelocity,
        GoalKind::TargetMargin,
        GoalKind::MinLanding,
        GoalKind::MinMass,
    ];

    fn name(self) -> &'static str {
        match self {
            GoalKind::MaxApogee => "Maximize apogee",
            GoalKind::MinApogee => "Minimize apogee",
            GoalKind::TargetApogee => "Target apogee",
            GoalKind::MaxVelocity => "Maximize max speed",
            GoalKind::MinVelocity => "Minimize max speed",
            GoalKind::TargetMargin => "Target static margin",
            GoalKind::MinLanding => "Minimize landing distance",
            GoalKind::MinMass => "Minimize mass",
        }
    }

    /// The quantity of the goal's value, and the label it is shown under.
    fn quantity(self) -> (Quantity, &'static str) {
        match self {
            GoalKind::MaxApogee | GoalKind::MinApogee | GoalKind::TargetApogee => (Quantity::Altitude, "Apogee"),
            GoalKind::MaxVelocity | GoalKind::MinVelocity => (Quantity::Velocity, "Max speed"),
            GoalKind::TargetMargin => (Quantity::Scalar, "Static margin"),
            GoalKind::MinLanding => (Quantity::Altitude, "Landing distance"),
            GoalKind::MinMass => (Quantity::Mass, "Liftoff mass"),
        }
    }

    fn target(self) -> Option<(&'static str, Quantity)> {
        match self {
            GoalKind::TargetApogee => Some(("Apogee", Quantity::Altitude)),
            GoalKind::TargetMargin => Some(("Margin", Quantity::Scalar)),
            _ => None,
        }
    }
}

impl std::fmt::Display for GoalKind {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.name())
    }
}

/// The constraints, in the order of [`State::limits`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Limit {
    MinMargin,
    MaxMargin,
    MinRailExit,
    MaxMach,
    MinFlutter,
    MaxApogee,
}

impl Limit {
    const ALL: [Limit; 6] = [Limit::MinMargin, Limit::MaxMargin, Limit::MinRailExit, Limit::MaxMach, Limit::MinFlutter, Limit::MaxApogee];

    fn name(self) -> &'static str {
        match self {
            Limit::MinMargin => "Min margin (cal)",
            Limit::MaxMargin => "Max margin (cal)",
            Limit::MinRailExit => "Min rail-exit speed",
            Limit::MaxMach => "Max Mach",
            Limit::MinFlutter => "Min flutter margin",
            Limit::MaxApogee => "Max apogee",
        }
    }

    fn quantity(self) -> Quantity {
        match self {
            Limit::MinRailExit => Quantity::Velocity,
            Limit::MaxApogee => Quantity::Altitude,
            _ => Quantity::Scalar,
        }
    }

    /// The value of this constraint's metric in a measured design.
    fn measured(self, m: &Metrics) -> Option<f64> {
        match self {
            Limit::MinMargin | Limit::MaxMargin => Some(m.margin),
            Limit::MinRailExit => Some(m.rail_exit),
            Limit::MaxMach => Some(m.max_mach),
            Limit::MinFlutter => m.flutter_margin,
            Limit::MaxApogee => Some(m.apogee),
        }
    }

    fn holds(self, measured: f64, limit: f64) -> bool {
        match self {
            Limit::MinMargin | Limit::MinRailExit | Limit::MinFlutter => measured >= limit,
            Limit::MaxMargin | Limit::MaxMach | Limit::MaxApogee => measured <= limit,
        }
    }

    fn default_value(self) -> f64 {
        match self {
            Limit::MinMargin => 1.0,
            Limit::MaxMargin => 3.0,
            Limit::MinRailExit => 15.0,
            Limit::MaxMach => 0.9,
            Limit::MinFlutter => super::compute::FLUTTER_MIN_MARGIN,
            Limit::MaxApogee => 1500.0,
        }
    }
}

/// One variable row.
#[derive(Debug, Clone, PartialEq)]
struct VarRow {
    part: PartId,
    dim: Dim,
    min: f64,
    max: f64,
}

/// One flown design as the table keeps it.
#[derive(Debug, Clone, PartialEq)]
struct Evaluation {
    index: usize,
    values: Vec<f64>,
    goal: f64,
    score: f64,
    feasible: bool,
    metrics: Option<Metrics>,
}

/// How a run ended.
#[derive(Debug, Clone, PartialEq)]
enum End {
    Done { best: Vec<f64>, reason: &'static str },
    Failed(String),
}

/// A search in progress or finished, with what it was asked (the left panel may change meanwhile).
struct Run {
    stream: worker::StreamHandle,
    variables: Vec<Variable>,
    /// The variables' names and quantities, as the table heads them.
    names: Vec<(String, Quantity)>,
    goal_kind: GoalKind,
    goal: Goal,
    constraints: Constraints,
    budget: usize,
    /// The variables' values when the run began.
    base: Vec<f64>,
    rows: Vec<Evaluation>,
    started: Instant,
    elapsed: f64,
    end: Option<End>,
    cancelling: bool,
}

impl Run {
    fn running(&self) -> bool {
        self.end.is_none()
    }

    /// The row with the lowest score.
    fn best(&self) -> Option<&Evaluation> {
        self.rows.iter().filter(|r| r.score.is_finite()).min_by(|a, b| a.score.total_cmp(&b.score))
    }
}

/// The Optimize screen's state.
pub struct State {
    flight: Option<FlightId>,
    variables: Vec<VarRow>,
    goal: GoalKind,
    target_apogee: f64,
    target_margin: f64,
    limits: [(bool, f64); 6],
    evaluations: usize,
    run: Option<Run>,
    serial: u64,
    seeded: bool,
    /// Whether the chosen flight can fly (for the document revision and flight it was checked at).
    flight_check: Option<(u64, Option<FlightId>, Result<(), String>)>,
    /// Why Run is not possible, as of the last message.
    blocked: Option<String>,
    plot: PlotState,
    group: Option<PlotGroup>,
}

impl Default for State {
    fn default() -> Self {
        State::new()
    }
}

impl State {
    /// An empty setup; the first variable is chosen when the screen first shows a document.
    pub fn new() -> Self {
        State {
            flight: None,
            variables: Vec::new(),
            goal: GoalKind::MaxApogee,
            target_apogee: 1000.0,
            target_margin: 1.5,
            limits: Limit::ALL.map(|l| (false, l.default_value())),
            evaluations: optimize::DEFAULT_EVALUATIONS,
            run: None,
            serial: 0,
            seeded: false,
            flight_check: None,
            blocked: None,
            plot: PlotState::new((0.0, 1.0)),
            group: None,
        }
    }

    /// Whether a search is running (the app keeps its frames coming).
    pub fn running(&self) -> bool {
        self.run.as_ref().is_some_and(Run::running)
    }

    /// Brings the setup up to date with the document: the first variable, vanished parts and flights, and whether the flight flies.
    pub fn sync(&mut self, ctx: &Ctx) {
        let rocket = ctx.doc.rocket();
        if !self.seeded {
            self.seeded = true;
            let fin = rocket.radials().iter().find(|r| r.kind.is_fin()).map(|r| (r.id, Dim::Span));
            let first = fin.or_else(|| rocket.parts().first().map(|p| (p.id, Dim::Length)));
            if let Some((part, dim)) = first {
                if let Some(row) = default_row(rocket, part, dim) {
                    self.variables.push(row);
                }
            }
        }
        self.variables.retain(|v| rocket.dim(v.part, v.dim).is_some());
        if self.flight.is_none_or(|f| !rocket.flights().iter().any(|d| d.id == f)) {
            // The flight selected on the Simulate screen, else the first.
            let selected = ctx.selected_flight.filter(|id| rocket.flights().iter().any(|d| d.id == *id));
            self.flight = selected.or_else(|| rocket.flights().first().map(|f| f.id));
        }
        let stamp = (ctx.doc.revision(), self.flight);
        if self.flight_check.as_ref().map(|c| (c.0, c.1)) != Some(stamp) {
            let check = self
                .spec(ctx)
                .ok_or_else(|| "The document has no flight".to_string())
                .and_then(|spec| prepare_rocket(rocket.clone(), ctx.physics.motors(), &spec).map(|_| ()));
            self.flight_check = Some((stamp.0, stamp.1, check));
        }
    }

    fn spec(&self, ctx: &Ctx) -> Option<Spec> {
        let flights = ctx.doc.rocket().flights();
        let def = self.flight.and_then(|id| flights.iter().find(|f| f.id == id)).or_else(|| flights.first())?;
        Some(Spec::of(def))
    }

    fn goal(&self) -> Goal {
        match self.goal {
            GoalKind::MaxApogee => Goal::MaxApogee,
            GoalKind::MinApogee => Goal::MinApogee,
            GoalKind::TargetApogee => Goal::TargetApogee(self.target_apogee),
            GoalKind::MaxVelocity => Goal::MaxVelocity,
            GoalKind::MinVelocity => Goal::MinVelocity,
            GoalKind::TargetMargin => Goal::TargetMargin(self.target_margin),
            GoalKind::MinLanding => Goal::MinLandingDistance,
            GoalKind::MinMass => Goal::MinMass,
        }
    }

    fn constraints(&self) -> Constraints {
        let on = |l: Limit| self.limits[l as usize].0.then_some(self.limits[l as usize].1);
        Constraints {
            min_margin: on(Limit::MinMargin),
            max_margin: on(Limit::MaxMargin),
            min_rail_exit: on(Limit::MinRailExit),
            max_mach: on(Limit::MaxMach),
            min_flutter_margin: on(Limit::MinFlutter),
            max_apogee: on(Limit::MaxApogee),
        }
    }

    fn variables_list(&self) -> Vec<Variable> {
        self.variables.iter().map(|v| Variable { field: FieldRef::new(v.part, v.dim), min: v.min, max: v.max }).collect()
    }

    /// Why Run is not possible, if it is not.
    fn blocker(&self) -> Option<String> {
        if self.running() {
            return Some("A search is running".to_string());
        }
        if self.variables.is_empty() {
            return Some("Add a variable to move".to_string());
        }
        if self.variables.iter().any(|v| v.min >= v.max) {
            return Some("Each variable needs a minimum below its maximum".to_string());
        }
        for (i, v) in self.variables.iter().enumerate() {
            if self.variables[..i].iter().any(|w| (w.part, w.dim) == (v.part, v.dim)) {
                return Some("Two variables move the same dimension".to_string());
            }
        }
        match &self.flight_check {
            Some((_, _, Err(e))) => Some(e.clone()),
            _ => None,
        }
    }

    /// Checks that a run may begin and returns the problem to search. `Err` is why not.
    fn prepare(&mut self, ctx: &Ctx) -> Result<Problem, String> {
        self.sync(ctx);
        if let Some(why) = self.blocker() {
            return Err(why);
        }
        let spec = self.spec(ctx).ok_or("The document has no flight")?;
        let rocket = ctx.doc.rocket().clone();
        let problem = Problem {
            variables: self.variables_list(),
            goal: self.goal(),
            constraints: self.constraints(),
            max_evaluations: self.evaluations,
            motors: Arc::new(ctx.physics.motors().clone()),
            spec,
            rocket,
        };
        optimize::check(&problem)?;
        self.serial += 1;
        Ok(problem)
    }

    /// Records the run of `problem` that `stream` (of serial `self.serial`) delivers.
    fn begin(&mut self, problem: &Problem, stream: worker::StreamHandle) {
        let base = problem.variables.iter().map(|v| problem.rocket.dim(v.field.part, v.field.dim).unwrap_or(v.min)).collect();
        let names = self
            .variables
            .iter()
            .map(|v| {
                let part = problem.rocket.kind_of(v.part).map(|p| p.name().to_string()).unwrap_or_default();
                (format!("{part} · {}", dim_name(&problem.rocket, v.part, v.dim)), dim_quantity(v.dim))
            })
            .collect();
        self.run = Some(Run {
            stream,
            variables: problem.variables.clone(),
            names,
            goal_kind: self.goal,
            goal: problem.goal,
            constraints: problem.constraints,
            budget: problem.max_evaluations.clamp(*optimize::EVALUATIONS.start(), *optimize::EVALUATIONS.end()),
            base,
            rows: Vec::new(),
            started: Instant::now(),
            elapsed: 0.0,
            end: None,
            cancelling: false,
        });
        self.group = None;
        self.plot = PlotState::new((0.0, 1.0));
    }

    /// Handles one of the screen's messages.
    pub fn update(&mut self, msg: Msg, ctx: &mut Ctx) -> Task<app::Message> {
        self.sync(ctx);
        let task = self.apply_message(msg, ctx);
        self.sync(ctx);
        self.blocked = self.blocker();
        task
    }

    fn apply_message(&mut self, msg: Msg, ctx: &mut Ctx) -> Task<app::Message> {
        match msg {
            Msg::Flight(id) => {
                self.flight = Some(id);
                self.flight_check = None;
            }
            Msg::AddVariable => {
                if self.variables.len() < optimize::MAX_VARIABLES {
                    let rocket = ctx.doc.rocket();
                    let taken = |p: PartId, d: Dim| self.variables.iter().any(|v| (v.part, v.dim) == (p, d));
                    let pick =
                        part_choices(rocket).into_iter().find_map(|c| eligible_dims(rocket, c.id).into_iter().find(|&d| !taken(c.id, d)).map(|d| (c.id, d)));
                    if let Some(row) = pick.and_then(|(p, d)| default_row(rocket, p, d)) {
                        self.variables.push(row);
                    }
                }
            }
            Msg::RemoveVariable(i) => {
                if i < self.variables.len() {
                    self.variables.remove(i);
                }
            }
            Msg::Part(i, part) => {
                let rocket = ctx.doc.rocket();
                let dims = eligible_dims(rocket, part);
                if let Some(v) = self.variables.get_mut(i) {
                    let dim = if dims.contains(&v.dim) { v.dim } else { dims.first().copied().unwrap_or(v.dim) };
                    if let Some(row) = default_row(rocket, part, dim) {
                        *v = row;
                    }
                }
            }
            Msg::Dimension(i, dim) => {
                if let Some(v) = self.variables.get_mut(i) {
                    if let Some(row) = default_row(ctx.doc.rocket(), v.part, dim) {
                        *v = row;
                    }
                }
            }
            Msg::Min(i, x) => {
                let rocket = ctx.doc.rocket();
                if let Some(v) = self.variables.get_mut(i) {
                    v.min = rocket.clamp_dim(v.part, v.dim, x).map_or(x, |c| c.0);
                }
            }
            Msg::Max(i, x) => {
                let rocket = ctx.doc.rocket();
                if let Some(v) = self.variables.get_mut(i) {
                    v.max = rocket.clamp_dim(v.part, v.dim, x).map_or(x, |c| c.0);
                }
            }
            Msg::Goal(g) => self.goal = g,
            Msg::Target(x) => {
                if self.goal == GoalKind::TargetApogee {
                    self.target_apogee = x.max(0.0);
                } else {
                    self.target_margin = x;
                }
            }
            Msg::Enable(l, on) => self.limits[l as usize].0 = on,
            Msg::Value(l, x) => self.limits[l as usize].1 = x,
            Msg::Evaluations(n) => self.evaluations = n.clamp(*optimize::EVALUATIONS.start(), *optimize::EVALUATIONS.end()),
            Msg::Plot(PlotMessage::Hover(None)) => self.plot.set_hover(None),
            Msg::Plot(m) => self.plot.update(m),
            Msg::Cancel => {
                if let Some(run) = self.run.as_mut().filter(|r| r.running()) {
                    run.stream.request_cancel();
                    run.cancelling = true;
                }
            }
            Msg::Apply => self.apply(ctx),
            Msg::Run => match self.prepare(ctx) {
                Ok(problem) => {
                    let (task, stream) = spawn(problem.clone(), self.serial);
                    self.begin(&problem, stream);
                    return task;
                }
                Err(e) => ctx.notify(format!("Cannot optimize: {e}")),
            },
        }
        Task::none()
    }

    /// Apply best was pressed: every variable takes the best design's value, as one undo step.
    fn apply(&mut self, ctx: &mut Ctx) {
        let Some(run) = &self.run else { return };
        let Some(End::Done { best, .. }) = &run.end else { return };
        let (variables, best) = (run.variables.clone(), best.clone());
        ctx.doc.edit("Apply optimization", |r| {
            for (v, &x) in variables.iter().zip(&best) {
                r.set_dim(v.field.part, v.field.dim, x);
            }
            r.apply_formulas();
        });
        ctx.notify("Applied the best design");
    }

    /// A message from the worker of run `serial`.
    pub fn event(&mut self, serial: u64, event: Event, ctx: &mut Ctx) {
        let Some(run) = self.run.as_mut().filter(|r| r.stream.serial() == serial) else { return };
        match event {
            Event::Tick => {}
            Event::Evaluated { index, values, goal, score, feasible, metrics } => {
                run.rows.push(Evaluation { index, values, goal, score, feasible, metrics });
                run.rows.sort_by_key(|r| r.index);
                self.group = Some(score_plot(run));
                if let Some(g) = &self.group {
                    self.plot.set_full(g.x_extent());
                }
            }
            Event::Done { best, reason } => {
                run.elapsed = run.started.elapsed().as_secs_f64();
                run.end = Some(End::Done { best, reason });
                run.stream.abort();
                ctx.notify(format!("Optimization finished: {reason}"));
            }
            Event::Failed(e) => {
                run.elapsed = run.started.elapsed().as_secs_f64();
                ctx.notify(format!("Optimization stopped: {e}"));
                run.end = Some(End::Failed(e));
                run.stream.abort();
            }
        }
        self.blocked = self.blocker();
    }

    // -----------------------------------------------------------------------------------------------------------------
    // Left
    // -----------------------------------------------------------------------------------------------------------------

    /// The left panel's controls (below the analyses list).
    pub fn left<'a>(&'a self, ctx: &'a Ctx) -> Element<'a, Message> {
        let rocket = ctx.doc.rocket();
        let units = &ctx.units;
        let head = |s: &'a str| -> Element<'a, Message> { container(ui::label(s)).padding([SPACE_2, 0.0]).boxed() };
        let opt = |m: Msg| Message::Optimize(m);

        let flights: Vec<FlightChoice> = rocket.flights().iter().map(|f| FlightChoice { id: f.id, name: f.name.clone() }).collect();
        let chosen = flights.iter().find(|f| Some(f.id) == self.flight).cloned();
        let flight = select(flights, chosen, move |c: FlightChoice| Message::Optimize(Msg::Flight(c.id)));

        let parts = part_choices(rocket);
        let mut variables: Vec<Element<'a, Message>> = Vec::new();
        for (i, v) in self.variables.iter().enumerate() {
            let part = parts.iter().find(|c| c.id == v.part).cloned();
            let dims: Vec<DimChoice> = eligible_dims(rocket, v.part).into_iter().map(|d| DimChoice { dim: d, label: dim_name(rocket, v.part, d) }).collect();
            let dim = dims.iter().find(|c| c.dim == v.dim).cloned();
            let q = dim_quantity(v.dim);
            variables.push(
                column![
                    row![
                        select(parts.clone(), part, move |c: PartChoice| Message::Optimize(Msg::Part(i, c.id))),
                        Element::from(crate::widgets::button::icon_button(Mark::Close, "Remove variable").on_press(opt(Msg::RemoveVariable(i))))
                    ]
                    .spacing(SPACE_1)
                    .align_y(iced::Center),
                    select(dims, dim, move |c: DimChoice| Message::Optimize(Msg::Dimension(i, c.dim))),
                    row![
                        unit_input("Min", v.min, q, units).label_width(LABEL_W / 2.0).on_commit(move |x| Message::Optimize(Msg::Min(i, x))),
                        unit_input("Max", v.max, q, units).label_width(LABEL_W / 2.0).on_commit(move |x| Message::Optimize(Msg::Max(i, x))),
                    ]
                    .spacing(SPACE_2),
                ]
                .spacing(SPACE_1)
                .boxed(),
            );
        }
        let mut add = push("Add variable", Weight::Secondary).small().icon(Mark::Plus);
        add = if self.variables.len() < optimize::MAX_VARIABLES { add.on_press(opt(Msg::AddVariable)) } else { add.disabled("At most four variables") };
        variables.push(add.into());

        let goals = select(GoalKind::ALL.to_vec(), Some(self.goal), move |g| Message::Optimize(Msg::Goal(g)));
        let mut goal_rows: Vec<Element<'a, Message>> = vec![goals];
        if let Some((label, q)) = self.goal.target() {
            let value = if self.goal == GoalKind::TargetApogee { self.target_apogee } else { self.target_margin };
            goal_rows.push(unit_input(label, value, q, units).label_width(LABEL_W).on_commit(move |x| Message::Optimize(Msg::Target(x))).into());
        }

        let mut limits: Vec<Element<'a, Message>> = Vec::new();
        for l in Limit::ALL {
            let (on, value) = self.limits[l as usize];
            let toggle = checkbox(l.name(), on, move |b| Message::Optimize(Msg::Enable(l, b)));
            limits.push(if on {
                column![toggle, unit_input("", value, l.quantity(), units).label_width(0.0).on_commit(move |x| Message::Optimize(Msg::Value(l, x)))]
                    .spacing(SPACE_1)
                    .boxed()
            } else {
                toggle
            });
        }

        let evaluations = unit_input("Evaluations", self.evaluations as f64, Quantity::Scalar, units)
            .label_width(LABEL_W + 14.0)
            .on_commit(|x| Message::Optimize(Msg::Evaluations(x.round().max(0.0) as usize)));

        let running = self.running();
        let mut run = push("Run", Weight::Primary);
        run = match &self.blocked {
            None => run.on_press(opt(Msg::Run)),
            Some(why) => run.disabled(why.as_str()),
        };
        let mut cancel = push(if self.run.as_ref().is_some_and(|r| r.cancelling) { "Cancelling…" } else { "Cancel" }, Weight::Secondary);
        cancel = if running && !self.run.as_ref().is_some_and(|r| r.cancelling) {
            cancel.on_press(opt(Msg::Cancel))
        } else {
            cancel.disabled("No search is running")
        };
        let mut apply = push("Apply best", Weight::Secondary);
        apply = if matches!(self.run.as_ref().and_then(|r| r.end.as_ref()), Some(End::Done { .. })) {
            apply.on_press(opt(Msg::Apply))
        } else {
            apply.disabled("Run a search first")
        };
        let (run, cancel, apply): (Element<'a, Message>, Element<'a, Message>, Element<'a, Message>) = (run.into(), cancel.into(), apply.into());

        container(
            column![
                row![run, cancel, apply].spacing(SPACE_2),
                head("FLIGHT"),
                flight,
                head("VARIABLES"),
                column(variables).spacing(SPACE_2),
                head("GOAL"),
                column(goal_rows).spacing(SPACE_1),
                head("CONSTRAINTS"),
                column(limits).spacing(SPACE_1),
                Space::new().height(SPACE_1),
                evaluations,
            ]
            .spacing(SPACE_1),
        )
        .padding([0.0, SPACE_3])
        .boxed()
    }

    // -----------------------------------------------------------------------------------------------------------------
    // Centre
    // -----------------------------------------------------------------------------------------------------------------

    /// The centre column: the score plot and the table of designs.
    pub fn centre<'a>(&'a self, ctx: &'a Ctx) -> Element<'a, Message> {
        let Some(run) = &self.run else {
            let message = self.blocker().map_or_else(|| "Choose the variables and a goal, then Run.".to_string(), |why| format!("Cannot run yet: {why}."));
            return container(text(message).size(TEXT_BODY).style(theme::muted)).center(Fill).boxed();
        };
        let elapsed = if run.running() { run.started.elapsed().as_secs_f64() } else { run.elapsed };
        let status = match &run.end {
            None => format!("OPTIMIZING · {} of {} evaluations · {elapsed:.0} s", run.rows.len(), run.budget),
            Some(End::Done { reason, .. }) => format!("DONE · {reason} · {} evaluations · {elapsed:.0} s", run.rows.len()),
            Some(End::Failed(e)) => format!("STOPPED · {e}"),
        };
        let mut header: Vec<Element<'a, Message>> = vec![ui::label(status)];
        if run.running() {
            header.push(crate::widgets::spinner::spinner(14.0));
        }
        let top = container(row(header).spacing(SPACE_2).align_y(alignment::Vertical::Center)).padding([SPACE_2, SPACE_3]);
        let plot: Element<'a, Message> = match &self.group {
            Some(group) => container(PlotView::new(group, &self.plot).element().map(|m| Message::Optimize(Msg::Plot(m))))
                .width(Fill)
                .height(Length::FillPortion(2))
                .boxed(),
            None => container(Space::new()).height(Length::FillPortion(2)).boxed(),
        };
        column![top, plot, container(table(run, ctx.units)).height(Length::FillPortion(3)).padding([0.0, SPACE_3])].height(Fill).boxed()
    }

    // -----------------------------------------------------------------------------------------------------------------
    // Right
    // -----------------------------------------------------------------------------------------------------------------

    /// The right panel's sections: the best design against the current one, the goal, the constraints and why it stopped.
    pub fn readout(&self, ctx: &Ctx) -> Vec<Section> {
        let Some(run) = &self.run else {
            return vec![Section { title: "Result".into(), rows: vec![plain("No search yet", "", "")] }];
        };
        let units = ctx.units;
        let best = run.best();
        let mut result = vec![plain("Evaluations", format!("{} of {}", run.rows.len(), run.budget), "")];
        result.push(match &run.end {
            None => plain("Status", "running", ""),
            Some(End::Done { reason, .. }) => plain("Stopped", *reason, ""),
            Some(End::Failed(e)) => plain("Stopped", e.clone(), ""),
        });
        if let Some(b) = best {
            let (text, severity) = if b.feasible { ("feasible", Severity::Ok) } else { ("violates a constraint", Severity::Warn) };
            let mut r = plain("Best design", format!("#{}", b.index + 1), "");
            r.tag = Some(Tag::Chip(severity, text.to_string()));
            result.push(r);
        }
        let mut sections = vec![Section { title: "Result".into(), rows: result }];
        if let Some(b) = best {
            let mut rows = Vec::new();
            for (i, (name, q)) in run.names.iter().enumerate() {
                let (value, unit) = ui::readout(b.values[i], *q, units);
                let mut r = plain(name.clone(), value, unit);
                r.share = Some(format!("was {}", ui::readout(run.base[i], *q, units).0));
                rows.push(r);
            }
            sections.push(Section { title: "Variables · best vs current".into(), rows });
            if let Some(m) = &b.metrics {
                let (q, label) = run.goal_kind.quantity();
                let (value, unit) = ui::readout(run.goal_kind_value(m), q, units);
                let mut goal = vec![plain(label, value, unit)];
                if let Some((_, tq)) = run.goal_kind.target() {
                    let target = match run.goal {
                        Goal::TargetApogee(t) | Goal::TargetMargin(t) => t,
                        _ => 0.0,
                    };
                    let (value, unit) = ui::readout(target, tq, units);
                    goal.push(plain("Target", value, unit));
                }
                sections.push(Section { title: "Goal at best".into(), rows: goal });
                let mut rows = Vec::new();
                for l in Limit::ALL {
                    let limit = match l {
                        Limit::MinMargin => run.constraints.min_margin,
                        Limit::MaxMargin => run.constraints.max_margin,
                        Limit::MinRailExit => run.constraints.min_rail_exit,
                        Limit::MaxMach => run.constraints.max_mach,
                        Limit::MinFlutter => run.constraints.min_flutter_margin,
                        Limit::MaxApogee => run.constraints.max_apogee,
                    };
                    let (Some(limit), Some(measured)) = (limit, l.measured(m)) else { continue };
                    let (value, unit) = ui::readout(measured, l.quantity(), units);
                    let mut r = plain(l.name(), value, unit);
                    r.tag = Some(Tag::Chip(if l.holds(measured, limit) { Severity::Ok } else { Severity::Err }, ui::with_unit(limit, l.quantity(), units)));
                    rows.push(r);
                }
                if rows.is_empty() {
                    rows.push(plain("None set", "", ""));
                }
                sections.push(Section { title: "Constraints at best".into(), rows });
            }
        }
        sections
    }
}

impl Run {
    /// The goal's value in a measured design.
    fn goal_kind_value(&self, m: &Metrics) -> f64 {
        match self.goal {
            Goal::MaxApogee | Goal::MinApogee | Goal::TargetApogee(_) => m.apogee,
            Goal::MaxVelocity | Goal::MinVelocity => m.max_velocity,
            Goal::TargetMargin(_) => m.margin,
            Goal::MinLandingDistance => m.landing_distance,
            Goal::MinMass => m.mass,
        }
    }
}

fn plain(label: impl Into<String>, value: impl Into<String>, unit: impl Into<String>) -> ReadoutRow {
    ReadoutRow { label: label.into(), value: value.into(), unit: unit.into(), swatch: None, tag: None, big: false, help: None, share: None, muted: false }
}

// ---------------------------------------------------------------------------------------------------------------------
// Pick lists
// ---------------------------------------------------------------------------------------------------------------------

#[derive(Debug, Clone)]
struct FlightChoice {
    id: FlightId,
    name: String,
}

impl PartialEq for FlightChoice {
    fn eq(&self, other: &Self) -> bool {
        self.id == other.id
    }
}

impl std::fmt::Display for FlightChoice {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.name)
    }
}

#[derive(Debug, Clone)]
struct PartChoice {
    id: PartId,
    label: String,
}

impl PartialEq for PartChoice {
    fn eq(&self, other: &Self) -> bool {
        self.id == other.id
    }
}

impl std::fmt::Display for PartChoice {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.label)
    }
}

#[derive(Debug, Clone)]
struct DimChoice {
    dim: Dim,
    label: String,
}

impl PartialEq for DimChoice {
    fn eq(&self, other: &Self) -> bool {
        self.dim == other.dim
    }
}

impl std::fmt::Display for DimChoice {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.label)
    }
}

/// The dimensions a search may move: everything the direct-edit handles have, except a freeform vertex, the nose shape
/// parameter and an axial part's position (which is its place in the stack).
const DIMS: [Dim; 23] = [
    Dim::Length,
    Dim::Diameter,
    Dim::ForeDiameter,
    Dim::Wall,
    Dim::Position,
    Dim::Span,
    Dim::RootChord,
    Dim::TipChord,
    Dim::Sweep,
    Dim::Thickness,
    Dim::Cant,
    Dim::Angle,
    Dim::Gap,
    Dim::Height,
    Dim::InnerDiameter,
    Dim::Overhang,
    Dim::PackedLength,
    Dim::ShoulderLength,
    Dim::ShoulderDiameter,
    Dim::ShoulderThickness,
    Dim::ForeShoulderLength,
    Dim::ForeShoulderDiameter,
    Dim::ForeShoulderThickness,
];

/// The dimensions of part `id` a variable may be.
fn eligible_dims(rocket: &Rocket, id: PartId) -> Vec<Dim> {
    let axial = rocket.part(id).is_some();
    let mut dims: Vec<Dim> = DIMS.iter().copied().filter(|&d| !(axial && d == Dim::Position) && rocket.dim(id, d).is_some()).collect();
    if rocket.dim(id, Dim::FilletRadius).is_some() {
        dims.push(Dim::FilletRadius);
    }
    dims
}

/// Every part that has a dimension to move, named (twins numbered).
fn part_choices(rocket: &Rocket) -> Vec<PartChoice> {
    let ids: Vec<PartId> =
        rocket.parts().iter().map(|p| p.id).chain(rocket.radials().iter().map(|p| p.id)).chain(rocket.internals().iter().map(|p| p.id)).collect();
    let named: Vec<(PartId, String)> =
        ids.into_iter().filter(|&id| !eligible_dims(rocket, id).is_empty()).filter_map(|id| rocket.kind_of(id).map(|p| (id, p.name().to_string()))).collect();
    let mut seen: std::collections::HashMap<String, usize> = std::collections::HashMap::new();
    named
        .iter()
        .map(|(id, name)| {
            let total = named.iter().filter(|(_, n)| n == name).count();
            let k = seen.entry(name.clone()).and_modify(|k| *k += 1).or_insert(1);
            PartChoice { id: *id, label: if total > 1 { format!("{name} ({k})") } else { name.clone() } }
        })
        .collect()
}

/// A variable on `dim` of `part` with the default range: the current value ±25 %, within what the dimension may be.
fn default_row(rocket: &Rocket, part: PartId, dim: Dim) -> Option<VarRow> {
    let current = rocket.dim(part, dim)?;
    let spread = if current.abs() < 1e-9 {
        if dim_quantity(dim) == Quantity::Angle {
            0.1
        } else {
            0.05
        }
    } else {
        0.25 * current.abs()
    };
    let limit = |x: f64| rocket.clamp_dim(part, dim, x).map_or(x, |c| c.0);
    Some(VarRow { part, dim, min: limit(current - spread), max: limit(current + spread) })
}

// ---------------------------------------------------------------------------------------------------------------------
// Plot and table
// ---------------------------------------------------------------------------------------------------------------------

/// The score of every evaluation and the best so far. Scores above the 90th percentile (a heavily penalised design) are drawn at
/// that height so one bad design does not flatten the rest.
fn score_plot(run: &Run) -> PlotGroup {
    let finite: Vec<f64> = run.rows.iter().map(|r| r.score).filter(|s| s.is_finite()).collect();
    let mut sorted = finite.clone();
    sorted.sort_by(f64::total_cmp);
    let cap = sorted.get((sorted.len() * 9 / 10).min(sorted.len().saturating_sub(1))).copied().unwrap_or(0.0);
    let each: Vec<(f64, f64)> = run.rows.iter().filter(|r| r.score.is_finite()).map(|r| (r.index as f64, r.score.min(cap))).collect();
    let mut best = f64::INFINITY;
    let mut so_far: Vec<(f64, f64)> = Vec::new();
    for r in run.rows.iter().filter(|r| r.score.is_finite()) {
        best = best.min(r.score);
        so_far.push((r.index as f64, best.min(cap)));
    }
    let last = run.rows.last().map_or(1, |r| r.index).max(1) as f64;
    let mut group = PlotGroup::new(Axis { label: "Evaluation".into(), unit: String::new(), prefix: "#".into(), decimals: 0 });
    group = group.panel(
        Panel::new("Score", "lower is better")
            .with(Series::new("Each design", each).color(SeriesColor::Muted))
            .with(Series::new("Best so far", so_far).color(SeriesColor::Accent).label_as("Best so far", LabelAt::End)),
    );
    group.x_range = Some((0.0, last));
    group.y_ticks = YTicks::Even(5);
    group.right_margin = MARGIN_PX;
    group
}

/// The table of designs: number, each variable, the goal, and whether it keeps the constraints; the best row is highlighted.
fn table<'a>(run: &'a Run, units: UnitSystem) -> Element<'a, Message> {
    let best = run.best().map(|b| b.index);
    let (goal_q, goal_label) = run.goal_kind.quantity();
    let cell = |s: String, header: bool, first: bool| -> Element<'a, Message> {
        let t = text(s).font(MONO).size(if header { TEXT_LABEL } else { 12.0 }).width(if first { Length::Fixed(40.0) } else { Length::FillPortion(1) });
        let t = if first { t } else { t.align_x(alignment::Horizontal::Right) };
        (if header { t.style(theme::faint) } else { t.style(theme::ink) }).boxed()
    };
    let mut heads = vec![cell("#".into(), true, true)];
    for (name, q) in &run.names {
        heads.push(cell(format!("{name} · {}", q.unit_for(units).symbol), true, false));
    }
    heads.push(cell(format!("{goal_label} · {}", goal_q.unit_for(units).symbol), true, false));
    heads.push(cell("OK".into(), true, false));
    let head = container(row(heads).spacing(SPACE_2)).padding([SPACE_2, SPACE_3]);
    let rows = column(
        run.rows
            .iter()
            .map(|r| {
                let mut cells = vec![cell(format!("{}", r.index + 1), false, true)];
                for (v, (_, q)) in r.values.iter().zip(&run.names) {
                    cells.push(cell(ui::readout(*v, *q, units).0, false, false));
                }
                cells.push(cell(if r.goal.is_finite() { ui::readout(r.goal, goal_q, units).0 } else { "—".into() }, false, false));
                cells.push(cell(if r.feasible { "✓".into() } else { "✗".into() }, false, false));
                let highlighted = Some(r.index) == best;
                container(row(cells).spacing(SPACE_2))
                    .padding([1.0, SPACE_3])
                    .width(Fill)
                    .style(move |theme: &Theme| container::Style {
                        background: highlighted.then(|| theme::tokens(theme).bg_300.into()),
                        ..container::Style::default()
                    })
                    .boxed()
            })
            .collect::<Vec<_>>(),
    )
    .spacing(1);
    column![head, crate::widgets::field::rule(), scrollable(rows).height(Fill)].boxed()
}

// ---------------------------------------------------------------------------------------------------------------------
// Worker
// ---------------------------------------------------------------------------------------------------------------------

/// Runs the search on its own thread and delivers its events as [`Message::OptEvent`]; a beat sends [`Event::Tick`] every
/// quarter second until the search is over, so the clock on screen moves between evaluations.
fn spawn(problem: Problem, serial: u64) -> (Task<app::Message>, worker::StreamHandle) {
    worker::spawn_stream(
        serial,
        Some((BEAT, || Event::Tick)),
        move |cancel, tx| {
            let sender = std::sync::Mutex::new(tx);
            optimize::run(&problem, &cancel, &move |e| {
                let _ = sender.lock().unwrap_or_else(|p| p.into_inner()).unbounded_send(e);
            })
        },
        |serial, event| app::Message::Analyze(Message::OptEvent(serial, event)),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::app::{App, Message as AppMessage};

    #[test]
    fn apply_best_is_one_undo_step_and_a_run_fills_the_table() {
        let mut app = App::with_example();
        let mut state = State::new();
        state.evaluations = 20;
        state.sync(app.ctx());
        assert_eq!(state.variables.len(), 1, "the first variable is the fin span");
        let before = app.ctx().doc.rocket().clone();
        let problem = state.prepare(app.ctx()).unwrap();
        let serial = state.serial;
        let (_task, stream) = worker::spawn_iter(serial, Vec::new(), |_, e: Event| e);
        state.begin(&problem, stream);
        let cancel = std::sync::atomic::AtomicBool::new(false);
        let events = std::sync::Mutex::new(Vec::new());
        let end = optimize::run(&problem, &cancel, &|e| events.lock().unwrap().push(e));
        let events = events.into_inner().unwrap();
        for e in events {
            state.event(serial, e, app.ctx_mut());
        }
        state.event(serial, end, app.ctx_mut());
        let run = state.run.as_ref().unwrap();
        assert!(!run.rows.is_empty() && matches!(run.end, Some(End::Done { .. })) && state.group.is_some());
        let _ = state.update(Msg::Apply, app.ctx_mut());
        assert_eq!(app.ctx().doc.undo_label(), Some("Apply optimization"));
        let _ = app.update(AppMessage::Undo);
        assert_eq!(app.ctx().doc.rocket(), &before, "one undo step restores the design");
    }
}
